#pragma once
#include <atomic>
#include <cstdint>
#include <cstring>

#include "core.hpp"
#include "desc_cfg.hpp"
#include "dev_core.hpp"
#include "ep.hpp"
#include "libxr_def.hpp"
#include "libxr_mem.hpp"
#include "structure/queue/spsc_queue.hpp"

namespace LibXR::USB
{

/**
 * @brief UAC1 队列式麦克风设备类实现（槽模型）
 *        UAC1 queue-driven microphone device class (slot model)
 *
 * 数据路径：SOF 节拍（OnBusTime）每个服务区间调用 KickOneFrame：取槽 → 从 PCM 队列
 * PopBatch 取整数个音频帧、不足补零 → 提交。没有空闲槽时本帧丢弃并计数。等时请求以
 * FAILED 结束时结束待决请求、重新 Configure 后由下一拍继续。alt 0/1 切换走
 * SET_INTERFACE 流程（MODEL_USB §3.3）：Close 结束本接口端点的待决请求，端点停止未
 * 确认时激活挂起，等停止确认或下一拍继续。
 *
 * Data path: the SOF tick (OnBusTime) calls KickOneFrame once per service interval:
 * acquire a slot, PopBatch whole audio frames from the PCM queue, zero-fill the
 * remainder, then submit. A frame with no free slot is dropped and counted. An
 * isochronous request that ends with FAILED ends the pending requests and re-Configures
 * the endpoint; streaming resumes from the next tick. Alt 0/1 switching follows the
 * SET_INTERFACE flow (MODEL_USB §3.3): Close ends the pending requests of this
 * interface's endpoint, and while the stop is unconfirmed the activation stays pending
 * until the confirmation or the next tick.
 */
template <uint8_t CHANNELS, uint8_t BITS_PER_SAMPLE>
class UAC1MicrophoneQ : public DeviceClass
{
 public:
  static_assert(CHANNELS >= 1 && CHANNELS <= 8, "CHANNELS out of range");
  static_assert(BITS_PER_SAMPLE == 8 || BITS_PER_SAMPLE == 16 || BITS_PER_SAMPLE == 24,
                "BITS_PER_SAMPLE must be 8/16/24");
  // AC / AS 接口对的默认字符串。
  // Default interface strings for the AC / AS pair.
  static constexpr const char* DEFAULT_CONTROL_INTERFACE_STRING = "XRUSB UAC1 Control";
  static constexpr const char* DEFAULT_STREAMING_INTERFACE_STRING =
      "XRUSB UAC1 Streaming";

 private:
  /// 每通道每采样的子帧字节数 / Subframe size (bytes per channel per sample)
  static const constexpr uint8_t K_SUBFRAME_SIZE = (BITS_PER_SAMPLE <= 8)    ? 1
                                                   : (BITS_PER_SAMPLE <= 16) ? 2
                                                                             : 3;
  // 保守默认值：smoke 构造时固定暴露 0 dB，避免主机看到误导性的音量范围。
  // Conservative smoke-safe defaults: expose fixed 0 dB gain by default.
  static constexpr int16_t K_DEFAULT_VOL_MIN = 0;
  static constexpr int16_t K_DEFAULT_VOL_MAX = 0;
  static constexpr int16_t K_DEFAULT_VOL_RES = 1;
  static constexpr Speed K_DEFAULT_SPEED = Speed::FULL;
  static constexpr size_t K_DEFAULT_QUEUE_BYTES = 2048;
  static constexpr uint8_t K_DEFAULT_INTERVAL = 1;

 public:
  /**
   * @brief 构造 UAC1 队列式麦克风
   *        Construct a queue-backed UAC1 microphone
   *
   * @param sample_rate_hz  采样率 / Sampling rate in Hz
   * @param vol_min         最小音量 / Min volume (1/256 dB)
   * @param vol_max         最大音量 / Max volume (1/256 dB)
   * @param vol_res         步进 / Step (1/256 dB)
   * @param speed           USB 速度（绑定前的描述符计算用） / USB speed (descriptor math
   *                        before binding)
   * @param queue_bytes     队列容量 / Queue capacity in bytes
   * @param interval        端点轮询间隔 / Endpoint interval
   * @param iso_in_ep_num   ISO IN 端点号 / Isochronous IN endpoint number
   * @param control_interface_string 控制接口字符串 / Control interface string
   * @param streaming_interface_string 流接口字符串 / Streaming interface string
   */
  UAC1MicrophoneQ(
      Endpoint::EPNumber iso_in_ep_num, uint32_t sample_rate_hz,
      int16_t vol_min = K_DEFAULT_VOL_MIN, int16_t vol_max = K_DEFAULT_VOL_MAX,
      int16_t vol_res = K_DEFAULT_VOL_RES, Speed speed = K_DEFAULT_SPEED,
      size_t queue_bytes = K_DEFAULT_QUEUE_BYTES, uint8_t interval = K_DEFAULT_INTERVAL,
      const char* control_interface_string = DEFAULT_CONTROL_INTERFACE_STRING,
      const char* streaming_interface_string = DEFAULT_STREAMING_INTERFACE_STRING)
      : iso_in_ep_num_(iso_in_ep_num),
        control_interface_string_(control_interface_string),
        streaming_interface_string_(streaming_interface_string),
        vol_min_(vol_min),
        vol_max_(vol_max),
        vol_res_(vol_res),
        interval_(interval),
        speed_(speed),
        sr_hz_(sample_rate_hz),
        supported_rate_(sample_rate_hz),
        pcm_queue_(queue_bytes)
  {
    REQUIRE(sr_hz_ != 0U && sr_hz_ <= 0xffffffU);
    // 构造期采样率是编译期决策：不可行是模板参数错误，保持 fatal。
    // The construction-time rate is a compile-time decision: an infeasible one is
    // a template-argument error and stays fatal.
    REQUIRE(RecomputeTiming());
    // 缓存端点采样率（3 字节小端） / Cache current sampling frequency (3-byte LE)
    sf_cur_[0] = static_cast<uint8_t>(sr_hz_ & 0xFF);
    sf_cur_[1] = static_cast<uint8_t>((sr_hz_ >> 8) & 0xFF);
    sf_cur_[2] = static_cast<uint8_t>((sr_hz_ >> 16) & 0xFF);
  }

  const char* GetInterfaceString(size_t local_interface_index) const override
  {
    // UAC 麦克风暴露 AC + AS 两个接口。
    // The UAC microphone exposes AC + AS interfaces.
    switch (local_interface_index)
    {
      case 0:
        return control_interface_string_;
      case 1:
        return streaming_interface_string_;
      default:
        return nullptr;
    }
  }

  // ===== 生产者接口 / Producer-side API =====
  /**
   * @brief 写入 PCM 字节流（如 S16LE / S24_3LE）
   *        Write interleaved PCM bytes (e.g., S16LE / S24_3LE)
   */
  ErrorCode WritePcm(const void* data, size_t nbytes)
  {
    return pcm_queue_.PushBatch(reinterpret_cast<const uint8_t*>(data), nbytes);
  }
  /** @brief 获取队列当前字节数 / Get current queued bytes */
  size_t QueueSize() const { return pcm_queue_.Size(); }
  /** @brief 获取队列剩余空间 / Get remaining queue capacity (bytes) */
  size_t QueueSpace() { return pcm_queue_.EmptySize(); }
  /** @brief 重置队列为空 / Reset queue to empty */
  void ResetQueue() { pcm_queue_.Reset(); }

  // ===== UAC1 常量 / UAC1 constants =====
  enum : uint8_t
  {
    USB_CLASS_AUDIO = 0x01,
    SUBCLASS_AC = 0x01,
    SUBCLASS_AS = 0x02
  };
  enum : uint8_t
  {
    CS_INTERFACE = 0x24,
    CS_ENDPOINT = 0x25
  };
  enum : uint8_t
  {
    AC_HEADER = 0x01,
    AC_INPUT_TERMINAL = 0x02,
    AC_OUTPUT_TERMINAL = 0x03,
    AC_FEATURE_UNIT = 0x06
  };
  enum : uint8_t
  {
    AS_GENERAL = 0x01,
    AS_FORMAT_TYPE = 0x02
  };
  enum : uint8_t
  {
    EP_GENERAL = 0x01
  };
  enum : uint16_t
  {
    WFORMAT_PCM = 0x0001,
    WFORMAT_UNKNOWN = 0xFFFF
  };
  enum : uint8_t
  {
    FORMAT_TYPE_I = 0x01
  };

  // UAC1 类请求 / Class-specific requests
  enum : uint8_t
  {
    SET_CUR = 0x01,
    GET_CUR = 0x81,
    SET_MIN = 0x02,
    GET_MIN = 0x82,
    SET_MAX = 0x03,
    GET_MAX = 0x83,
    SET_RES = 0x04,
    GET_RES = 0x84
  };
  enum : uint8_t
  {
    FU_MUTE = 0x01,
    FU_VOLUME = 0x02
  };

  // 实体 ID / Entity IDs
  enum : uint8_t
  {
    ID_IT_MIC = 1,
    ID_FU = 2,
    ID_OT_USB = 3
  };

  LIBXR_PACKED_BEGIN
  /**
   * @brief AC 头描述符
   *        AC header descriptor
   */
  struct CSACHeader
  {
    uint8_t bLength = 9, bDescriptorType = CS_INTERFACE, bDescriptorSubtype = AC_HEADER;
    uint16_t bcdADC = 0x0100, wTotalLength = 0;
    uint8_t bInCollection = 1, baInterfaceNr = 0;
  };

  /**
   * @brief AC 输入端子（麦克风）
   *        AC input terminal (Microphone)
   */
  struct ACInputTerminal
  {
    uint8_t bLength = 12, bDescriptorType = CS_INTERFACE,
            bDescriptorSubtype = AC_INPUT_TERMINAL;
    uint8_t bTerminalID = ID_IT_MIC;
    uint16_t wTerminalType = 0x0201;  // 麦克风 / Microphone
    uint8_t bAssocTerminal = 0, bNrChannels = CHANNELS;
    uint16_t wChannelConfig = 0x0000;  // 根据 CHANNELS 设定 / Set per CHANNELS
    uint8_t iChannelNames = 0, iTerminal = 0;
  };

  /**
   * @brief AC 特性单元（静音/音量）
   *        AC feature unit (mute/volume)
   */
  struct ACFeatureUnit
  {
    uint8_t bLength = static_cast<uint8_t>(7 + (CHANNELS + 1) * 1);
    uint8_t bDescriptorType = CS_INTERFACE, bDescriptorSubtype = AC_FEATURE_UNIT;
    uint8_t bUnitID = ID_FU, bSourceID = ID_IT_MIC, bControlSize = 1;
    uint8_t bmaControlsMaster = 0x03;          // 静音 + 音量 / Mute + Volume
    uint8_t bmaControlsCh[CHANNELS] = {0x03};  // 每通道支持 / Per-channel supported
    uint8_t iFeature = 0;
  };

  /**
   * @brief AC 输出端子（USB 流）
   *        AC output terminal (USB streaming)
   */
  struct ACOutputTerminal
  {
    uint8_t bLength = 9, bDescriptorType = CS_INTERFACE,
            bDescriptorSubtype = AC_OUTPUT_TERMINAL;
    uint8_t bTerminalID = ID_OT_USB;
    uint16_t wTerminalType = 0x0101;  // USB 流 / USB Streaming
    uint8_t bAssocTerminal = 0, bSourceID = ID_FU, iTerminal = 0;
  };

  /**
   * @brief AS 通用描述符
   *        AS general descriptor
   */
  struct ASGeneral
  {
    uint8_t bLength = 7, bDescriptorType = CS_INTERFACE, bDescriptorSubtype = AS_GENERAL;
    uint8_t bTerminalLink = ID_OT_USB, bDelay = 1;
    uint16_t wFormatTag = WFORMAT_PCM;
  };

  /**
   * @brief Type I 格式（单一离散采样率）
   *        Type I format (single discrete sampling frequency)
   */
  struct TypeIFormat1
  {
    uint8_t bLength = 11, bDescriptorType = CS_INTERFACE,
            bDescriptorSubtype = AS_FORMAT_TYPE;
    uint8_t bFormatType = FORMAT_TYPE_I, bNrChannels = CHANNELS;
    uint8_t bSubframeSize = K_SUBFRAME_SIZE, bBitResolution = BITS_PER_SAMPLE,
            bSamFreqType = 1;
    uint8_t tSamFreq[3];  // 运行期填充 / Filled at runtime
  };

  /**
   * @brief 类特定端点（通用）
   *        Class-specific endpoint (general)
   */
  struct CSEndpointGeneral
  {
    uint8_t bLength = 7, bDescriptorType = CS_ENDPOINT, bDescriptorSubtype = EP_GENERAL;
    uint8_t bmAttributes = 0x00;
    uint8_t bLockDelayUnits = 0;
    uint16_t wLockDelay = 0;
  };

  /**
   * @brief 标准等时 IN 端点（9 字节，含 bRefresh/bSynchAddress）
   *        Standard isoch IN endpoint (9 bytes, with bRefresh/bSynchAddress)
   */
  struct EndpointDescriptorIso9
  {
    uint8_t bLength = 9;
    uint8_t bDescriptorType = static_cast<uint8_t>(DescriptorType::ENDPOINT);
    uint8_t bEndpointAddress = 0;  // 含方向位 / includes direction bit
    uint8_t bmAttributes = 0x05;   // Isochronous + Async + Data
    uint16_t wMaxPacketSize = 0;   // 运行期填充 / filled at runtime
    uint8_t bInterval = 0x01;      // 1 ms
    uint8_t bRefresh = 0x00;       // 仅反馈端点使用 / feedback only
    uint8_t bSynchAddress = 0x00;  // 无伴随端点 / no sync/feedback ep
  };

  /**
   * @brief UAC1 描述符块
   *        UAC1 descriptor block
   */
  struct UAC1DescBlock
  {
    IADDescriptor iad;             ///< 接口关联描述符 / Interface association descriptor
    InterfaceDescriptor ac_intf;   ///< AC 接口 / AC interface
    CSACHeader ac_hdr;             ///< AC 头 / AC header
    ACInputTerminal it_mic;        ///< 输入端子 / Input terminal
    ACFeatureUnit fu;              ///< 特性单元 / Feature unit
    ACOutputTerminal ot_usb;       ///< 输出端子 / Output terminal
    InterfaceDescriptor as_alt0;   ///< AS Alt 0（无端点）/ AS Alt 0 (no EP)
    InterfaceDescriptor as_alt1;   ///< AS Alt 1（1 个 IN 端点）/ AS Alt 1 (1 IN EP)
    ASGeneral as_gen;              ///< AS 通用 / AS general
    TypeIFormat1 fmt;              ///< 格式描述符 / format descriptor
    EndpointDescriptorIso9 ep_in;  ///< 标准 IN 端点（9B）/ Std IN EP (9B)
    CSEndpointGeneral ep_cs;       ///< 类特定端点 / CS EP
  };
  LIBXR_PACKED_END

  // ===== DeviceClass 接口实现 / DeviceClass implementation =====
  /**
   * @brief 初始化设备（分配端点、填充描述符）
   *        Initialize device (allocate endpoints, populate descriptors)
   *
   * @param endpoint_pool 端点池 / Endpoint pool
   * @param start_itf_num 起始接口号 / Starting interface number
   * @param in_isr 是否在中断中 / Whether in ISR
   */
  void BindEndpoints(EndpointPool& endpoint_pool, uint8_t start_itf_num,
                     bool in_isr) override
  {
    inited_ = false;
    active_ = false;
    activate_pending_ = false;
    acc_rem_ = 0;
    micro_debt_ = 0;
    speed_ = endpoint_pool.GetSpeed();
    RecomputeTiming();

    // 这里只分配端点，不立即开启；真正开启放在 AS Alt=1 切换时。
    // Allocate the endpoint here without starting transfers;
    // actual streaming begins when AS Alt=1 is selected.
    auto ans = endpoint_pool.Get(ep_iso_in_, Endpoint::Direction::IN, iso_in_ep_num_);
    ASSERT(ans == ErrorCode::OK);

    // 包长显式：w_max_packet_size_ 由采样率与分帧推出（RecomputeTiming）。Configure
    // 校验它在当前速度下合法、槽与后端 FIFO 装得下；失败时端点停在 ERROR，composition
    // 以 ConfigurationValid() 拒绝本次配置。等时端点不被后端支持时（FSDEV / CH32
    // DevFS 的 ConfigureHardware 返回 NOT_SUPPORT）也走这一条路：使用者从绑定结果
    // 得到明确的运行期错误，而不是悄悄不工作。
    // Explicit packet size: w_max_packet_size_ is derived from the sample rate and the
    // framing (RecomputeTiming). Configure validates it against the speed, the slots
    // and the backend FIFO; on failure the endpoint stays in ERROR and the composition
    // refuses the configuration. A backend without isochronous support (NOT_SUPPORT
    // from the FSDEV / CH32 DevFS ConfigureHardware) fails the same way: the user gets
    // a clear runtime error from the bind result instead of a silently dead stream.
    (void)ep_iso_in_->Configure(
        {Endpoint::Direction::IN, Endpoint::Type::ISOCHRONOUS, w_max_packet_size_},
        in_isr);

    itf_ac_num_ = start_itf_num;
    itf_as_num_ = static_cast<uint8_t>(start_itf_num + 1);

    // IAD（把 AC + AS 这组接口关联成一个音频功能）
    // IAD for grouping the AC + AS interfaces into one audio function.
    desc_block_.iad = {8,
                       static_cast<uint8_t>(DescriptorType::IAD),
                       itf_ac_num_,
                       2,
                       USB_CLASS_AUDIO,
                       SUBCLASS_AC,
                       0x00,
                       0};

    // AC 接口（标准接口）
    // AC interface (standard).
    desc_block_.ac_intf = {9,
                           static_cast<uint8_t>(DescriptorType::INTERFACE),
                           itf_ac_num_,
                           0,
                           0,
                           USB_CLASS_AUDIO,
                           SUBCLASS_AC,
                           0x00,
                           GetInterfaceStringIndex(0u)};

    // AC Header（声明 AC 类描述符块总长）
    // AC header describing the total AC class-specific descriptor size.
    desc_block_.ac_hdr.baInterfaceNr = itf_as_num_;
    desc_block_.ac_hdr.wTotalLength =
        static_cast<uint16_t>(sizeof(CSACHeader) + sizeof(ACInputTerminal) +
                              sizeof(ACFeatureUnit) + sizeof(ACOutputTerminal));

    // AC Input Terminal 的 wChannelConfig 按通道数设置。
    // Program the AC Input Terminal wChannelConfig from the channel count.
    if constexpr (CHANNELS == 2)
    {
      desc_block_.it_mic.wChannelConfig = 0x0003;  // Left Front | Right Front
    }
    else
    {
      desc_block_.it_mic.wChannelConfig = 0x0000;  // 未声明或单声道 / unspecified or mono
    }

    // AS Alt 0：空闲态，不暴露数据端点。
    // AS Alt 0: idle alternate setting with no data endpoint.
    desc_block_.as_alt0 = {9,
                           static_cast<uint8_t>(DescriptorType::INTERFACE),
                           itf_as_num_,
                           0,
                           0,
                           USB_CLASS_AUDIO,
                           SUBCLASS_AS,
                           0x00,
                           GetInterfaceStringIndex(1u)};

    // AS Alt 1：工作态，暴露 1 个 Iso IN 端点。
    // AS Alt 1: active streaming alternate setting with one Iso IN endpoint.
    desc_block_.as_alt1 = {9,
                           static_cast<uint8_t>(DescriptorType::INTERFACE),
                           itf_as_num_,
                           1,
                           1,
                           USB_CLASS_AUDIO,
                           SUBCLASS_AS,
                           0x00,
                           GetInterfaceStringIndex(1u)};

    // AS General（把 AS 接口绑定到 USB 输出端子）
    // AS general descriptor linking the AS interface to the USB output terminal.
    desc_block_.as_gen = {};
    desc_block_.as_gen.bTerminalLink = ID_OT_USB;
    desc_block_.as_gen.bDelay = 1;
    desc_block_.as_gen.wFormatTag = WFORMAT_PCM;

    // Type I Format：当前实现只暴露一个离散采样率。
    // Type I format descriptor: the current implementation exposes one discrete rate.
    desc_block_.fmt.bFormatType = FORMAT_TYPE_I;
    desc_block_.fmt.bNrChannels = CHANNELS;
    desc_block_.fmt.bSubframeSize = K_SUBFRAME_SIZE;
    desc_block_.fmt.bBitResolution = BITS_PER_SAMPLE;
    desc_block_.fmt.bSamFreqType = 1;
    desc_block_.fmt.tSamFreq[0] = static_cast<uint8_t>(sr_hz_ & 0xFF);
    desc_block_.fmt.tSamFreq[1] = static_cast<uint8_t>((sr_hz_ >> 8) & 0xFF);
    desc_block_.fmt.tSamFreq[2] = static_cast<uint8_t>((sr_hz_ >> 16) & 0xFF);

    // 标准 ISO IN 端点（9 字节，含 bRefresh/bSynchAddress）
    // Standard 9-byte ISO IN endpoint descriptor, including refresh/sync fields.
    desc_block_.ep_in = {};
    desc_block_.ep_in.bEndpointAddress = static_cast<uint8_t>(
        Endpoint::EPNumberToAddr(ep_iso_in_->GetNumber(), Endpoint::Direction::IN));
    desc_block_.ep_in.bmAttributes = 0x05;
    desc_block_.ep_in.wMaxPacketSize = static_cast<uint16_t>(w_max_packet_size_);
    desc_block_.ep_in.bInterval = (speed_ == Speed::HIGH) ? interval_ : 0x01;
    desc_block_.ep_in.bRefresh = 0x00;
    desc_block_.ep_in.bSynchAddress = 0x00;

    desc_block_.ep_cs = {};
    desc_block_.ep_cs.bmAttributes = 0x00;

    // 把整块描述符数据交给 DeviceClass。
    // Publish the full descriptor block to DeviceClass.
    SetData(RawData{reinterpret_cast<uint8_t*>(&desc_block_), sizeof(desc_block_)});

    // 端点完成回调在 owner 中执行；SOF 节拍负责投递下一帧。
    // The completion callback runs in the owner; the SOF tick submits the next frame.
    ep_iso_in_->SetOnComplete(on_in_complete_cb_);

    // Configure 失败（槽装不下一个包、后端不支持等时等）：端点已停在 ERROR，不 Close
    // （Close 会把 ERROR 清成 DISABLED），让 composition 以 ConfigurationValid() 拒绝
    // 本次配置。
    // Configure failed (slot cannot hold one packet, isochronous unsupported, ...): the
    // endpoint stays in ERROR. Do not Close (it would clear ERROR to DISABLED) so the
    // composition refuses this configuration via ConfigurationValid().
    if (ep_iso_in_->GetState() == Endpoint::State::ERROR)
    {
      return;
    }

    ep_iso_in_->Close(in_isr);  // Alternate zero does not activate a data endpoint.
    inited_ = true;
  }

  /**
   * @brief 反初始化设备，释放端点
   *        Deinitialize device and release endpoints
   *
   * @param endpoint_pool 端点池 / Endpoint pool
   * @param in_isr 是否在中断中 / Whether in ISR
   */
  void UnbindEndpoints(EndpointPool& endpoint_pool, bool in_isr) override
  {
    inited_ = false;
    active_ = false;
    activate_pending_ = false;
    acc_rem_ = 0;
    micro_debt_ = 0;
    if (ep_iso_in_ != nullptr)
    {
      ep_iso_in_->Close(in_isr);
      endpoint_pool.Release(ep_iso_in_);
      ep_iso_in_ = nullptr;
    }
    // 解绑后清除速度缓存（MODEL_USB §4.6）；下次绑定时从端点池重新读取。
    // Clear the cached speed on unbind (MODEL_USB §4.6); the next bind re-reads it
    // from the endpoint pool.
    speed_ = K_SPEED_UNBOUND;
  }

  /**
   * @brief UAC1 无类特定描述符读出（配置中已包含）
   *        UAC1 does not use GET_DESCRIPTOR(0x21/0x22) here
   */
  ErrorCode OnGetDescriptor(bool, uint8_t, uint16_t, uint16_t, ConstRawData&) override
  {
    return ErrorCode::NOT_SUPPORT;
  }

  /**
   * @brief 处理类请求：端点采样率控制 + Feature Unit（静音/音量）
   *        Handle class requests: EP sampling-freq control + Feature Unit (mute/volume)
   */
  ErrorCode OnClassRequest(bool /*in_isr*/, uint8_t bRequest, uint16_t wValue,
                           uint16_t wLength, uint16_t wIndex,
                           DeviceClass::ControlTransferResult& r) override
  {
    // ===== 端点采样率控制（收件人：端点地址） / EP sampling-freq control =====
    const uint8_t EP_ADDR = static_cast<uint8_t>(wIndex & 0xFF);
    const uint8_t CS_EP =
        static_cast<uint8_t>((wValue >> 8) & 0xFF);  // 0x01 = 采样率 / Sampling Freq

    if (EP_ADDR == desc_block_.ep_in.bEndpointAddress && CS_EP == 0x01)
    {
      switch (bRequest)
      {
        case GET_CUR:
        case GET_MIN:
        case GET_MAX:
          if (wLength == 3)
          {
            // 单一离散频点：CUR/MIN/MAX 都返回当前采样率。
            // Single discrete rate: CUR/MIN/MAX all report the current frequency.
            r.write_data = ConstRawData{sf_cur_, sizeof(sf_cur_)};
            return ErrorCode::OK;
          }
          break;
        case GET_RES:
        {
          if (wLength == 3)
          {
            static const uint8_t one_hz[3] = {1, 0, 0};
            r.write_data = ConstRawData{one_hz, sizeof(one_hz)};
            return ErrorCode::OK;
          }
          break;
        }
        case SET_CUR:
          if (wLength == 3)
          {
            // OUT 数据阶段由核心接收，完整后经 OnControlData 交给类。
            // The core receives the OUT data stage and hands it over via
            // OnControlData once complete.
            pending_control_ = PendingControl::SET_SF;
            return ErrorCode::OK;
          }
          break;
        default:
          break;
      }

      return ErrorCode::ARG_ERR;  // 长度不符等 / length mismatch, etc.
    }

    // ===== Feature Unit（收件人：Interface / AC 接口） / Feature Unit =====
    const uint8_t CS = static_cast<uint8_t>((wValue >> 8) & 0xFF);  // FU_MUTE / FU_VOLUME
    const uint8_t CH =
        static_cast<uint8_t>(wValue & 0xFF);  // 0=主控, 1..N / 0=master, 1..N
    const uint8_t ENT =
        static_cast<uint8_t>((wIndex >> 8) & 0xFF);           // 实体 ID / entity ID
    const uint8_t ITF = static_cast<uint8_t>(wIndex & 0xFF);  // 接口号 / interface num

    if (ITF != itf_ac_num_ || ENT != ID_FU)
    {
      return ErrorCode::NOT_SUPPORT;
    }
    if (CH > CHANNELS)
    {
      return ErrorCode::ARG_ERR;
    }

    switch (CS)
    {
      case FU_MUTE:
        if (bRequest == SET_CUR && wLength == 1)
        {
          pending_control_ = PendingControl::SET_MUTE;
          return ErrorCode::OK;
        }
        if (bRequest == GET_CUR && wLength == 1)
        {
          r.write_data = ConstRawData{&mute_, 1};
          return ErrorCode::OK;
        }

        return ErrorCode::ARG_ERR;

      case FU_VOLUME:
        switch (bRequest)
        {
          case SET_CUR:
            if (wLength == 2)
            {
              pending_control_ = PendingControl::SET_VOLUME;
              return ErrorCode::OK;
            }
            break;
          case GET_CUR:
            if (wLength == 2)
            {
              r.write_data = ConstRawData{reinterpret_cast<const uint8_t*>(&vol_cur_), 2};
              return ErrorCode::OK;
            }
            break;
          case GET_MIN:
            if (wLength == 2)
            {
              r.write_data = ConstRawData{reinterpret_cast<const uint8_t*>(&vol_min_), 2};
              return ErrorCode::OK;
            }
            break;
          case GET_MAX:
            if (wLength == 2)
            {
              r.write_data = ConstRawData{reinterpret_cast<const uint8_t*>(&vol_max_), 2};
              return ErrorCode::OK;
            }
            break;
          case GET_RES:
            if (wLength == 2)
            {
              r.write_data = ConstRawData{reinterpret_cast<const uint8_t*>(&vol_res_), 2};
              return ErrorCode::OK;
            }
            break;
          case SET_RES:  // 吃下以避免 STALL / accept to avoid STALL
            if (wLength == 2)
            {
              pending_control_ = PendingControl::SET_RES;
              return ErrorCode::OK;
            }
            break;
          default:
            break;
        }

        return ErrorCode::ARG_ERR;

      default:

        return ErrorCode::NOT_SUPPORT;
    }
  }

  /**
   * @brief 处理类请求数据阶段（应用 SET_CUR：采样率 / 静音 / 音量）
   *        Handle the class request data stage (apply SET_CUR: rate / mute / volume)
   */
  ErrorCode OnClassData(bool, uint8_t bRequest, ConstRawData& data) override
  {
    if (bRequest != SET_CUR) return ErrorCode::NOT_SUPPORT;
    const PendingControl pending = pending_control_;
    pending_control_ = PendingControl::NONE;
    switch (pending)
    {
      case PendingControl::SET_SF:
      {
        if (data.size_ != 3U) return ErrorCode::ARG_ERR;
        const uint8_t* bytes = static_cast<const uint8_t*>(data.addr_);
        const uint32_t rate = static_cast<uint32_t>(bytes[0]) |
                              (static_cast<uint32_t>(bytes[1]) << 8U) |
                              (static_cast<uint32_t>(bytes[2]) << 16U);
        if (rate != supported_rate_) return ErrorCode::ARG_ERR;
        Memory::FastCopy(sf_cur_, bytes, sizeof(sf_cur_));
        return ErrorCode::OK;
      }
      case PendingControl::SET_MUTE:
        if (data.size_ != 1U) return ErrorCode::ARG_ERR;
        mute_ = static_cast<const uint8_t*>(data.addr_)[0];
        return ErrorCode::OK;
      case PendingControl::SET_VOLUME:
        if (data.size_ != 2U) return ErrorCode::ARG_ERR;
        std::memcpy(&vol_cur_, data.addr_, sizeof(vol_cur_));
        return ErrorCode::OK;
      case PendingControl::SET_RES:
        return ErrorCode::OK;  // 分辨率固定，数值忽略 / Fixed resolution, value ignored
      default:
        return ErrorCode::NOT_SUPPORT;
    }
  }

  /**
   * @brief 获取接口的当前 Alternate Setting
   *        Get current alternate setting for interface
   */
  ErrorCode GetAltSetting(uint8_t itf, uint8_t& alt) override
  {
    if (!inited_ || itf != itf_as_num_)
    {
      return ErrorCode::NOT_SUPPORT;
    }
    alt = (active_ || activate_pending_) ? 1U : 0U;
    return ErrorCode::OK;
  }

  /**
   * @brief 设置接口的 Alternate Setting（SET_INTERFACE 流程，MODEL_USB §3.3）
   *        Set interface alternate setting (SET_INTERFACE flow, MODEL_USB §3.3)
   */
  ErrorCode SetAltSetting(uint8_t itf, uint8_t alt) override
  {
    if (!inited_ || itf != itf_as_num_ || ep_iso_in_ == nullptr)
    {
      return ErrorCode::NOT_SUPPORT;
    }

    switch (alt)
    {
      case 0:
        // Alt 0：本接口没有数据端点；Close 结束待决请求并解绑端点，停止未确认时由
        // 核心推迟状态阶段。
        // Alt 0: no data endpoint for this interface; Close ends the pending requests
        // and unbinds the endpoint. The core defers the status stage while the stop is
        // unconfirmed.
        active_ = false;
        activate_pending_ = false;
        acc_rem_ = 0;
        micro_debt_ = 0;
        ep_iso_in_->Close(false);
        return ErrorCode::OK;

      case 1:
      {
        // Alt 1：结束当前请求并重新 Configure（§3.3 只涉及这个接口的端点）。端点停止
        // 未确认时保持挂起，等停止确认或下一拍继续（TryActivate）；Configure 失败
        // （如后端不支持等时）返回错误让核心 STALL 该请求。
        // Alt 1: end the current requests and re-Configure (§3.3 covers only this
        // interface's endpoint). While the stop is unconfirmed the activation stays
        // pending for the confirmation or the next tick (TryActivate); a Configure
        // failure (e.g. isochronous unsupported) returns the error so the core stalls
        // the request.
        active_ = false;
        acc_rem_ = 0;
        micro_debt_ = 0;
        activate_pending_ = true;
        ep_iso_in_->Close(false);
        const ErrorCode result = ActivateStreaming(false);
        if (result != ErrorCode::OK)
        {
          activate_pending_ = false;
          return result;
        }
        return ErrorCode::OK;  // 已开启，或等停止确认后由下一拍开启 / On, or waiting
      }

      default:

        return ErrorCode::ARG_ERR;
    }
  }

  /** @brief 返回接口数量（AC+AS=2） / Get number of interfaces (AC+AS=2) */
  size_t GetInterfaceCount() override { return 2; }
  /** @brief 包含 IAD / Has IAD */
  bool HasIAD() override { return true; }
  /** @brief 配置描述符最大尺寸 / Get maximum configuration size */
  size_t GetMaxConfigSize() override { return sizeof(UAC1DescBlock); }

  bool OwnsEndpoint(uint8_t ep_addr) const override
  {
    if (!inited_ || ep_iso_in_ == nullptr)
    {
      return false;
    }
    // alt 0 时端点已关闭（DISABLED），GetAddress() 不带方向位；按类自己声明的 IN
    // 地址比较，关闭与开启两种状态下都成立。
    // At alt 0 the endpoint is closed (DISABLED) and GetAddress() carries no direction
    // bit; compare against the IN address the class declares so both the closed and
    // the open state match.
    return Endpoint::EPNumberToAddr(ep_iso_in_->GetNumber(), Endpoint::Direction::IN) ==
           ep_addr;
  }

  // ===== SOF 节拍 / Bus tick =====
  // 绑定后即需要 SOF（帧节奏、挂起激活推进都靠它）；未绑定不再空转。
  // SOF is needed while bound (frame pacing and deferred activation rely on it);
  // an unbound device no longer wakes the owner.
  bool WantsBusTime() const override { return inited_; }

  void OnBusTime(bool in_isr, const BusTime& time) override
  {
    // 节拍语义（MODEL_USB §3.6.1）：FULL 每帧（1 ms）一个服务区间，一次节拍踢一帧；
    // HIGH 的服务区间为 2^(interval-1) 微帧，微帧差累积到一整个区间才踢。漏掉的区间
    // 不补发：实时流丢弃历史数据。节拍异常（差值 0 / 不连续）按一拍处理。
    // Tick semantics (MODEL_USB §3.6.1): one service interval per 1 ms frame on full
    // speed, one kick per tick; on high speed a service interval is 2^(interval-1)
    // microframes and the microframe delta accrues until a whole interval is due.
    // Missed intervals are not replayed: a real-time stream drops the past. A tick
    // anomaly (zero delta / discontinuity) counts as one interval.
    uint32_t intervals = time.elapsed_intervals == 0U ? 1U : time.elapsed_intervals;
    if (speed_ == Speed::HIGH)
    {
      micro_debt_ += (time.microframe == 0xFFU) ? intervals * 8U : intervals;
      const uint32_t ticks = 1U << (interval_ - 1U);
      if (micro_debt_ < ticks) return;
      micro_debt_ = (micro_debt_ >= 2U * ticks) ? 0U : micro_debt_ - ticks;
    }
    if (!active_)
    {
      // 挂起的激活（停止未确认时推迟）由这一拍推进：TryActivate 内部会重新 Configure
      // 并踢第一帧，这里不能再踢一次，否则同一服务区间会重复投帧。
      // A pending activation (deferred while the stop was unconfirmed) advances on
      // this tick: TryActivate re-Configures and kicks the first frame itself, so
      // this tick must not kick again or one interval would submit twice.
      TryActivate(in_isr);
      return;
    }
    KickOneFrame(in_isr);
  }

 protected:
  // 解绑后 speed_ 的占位值：UAC 只在 FULL/HIGH 下工作，2.0.2 的能力位不会出现 SUPER。
  // Placeholder speed_ while unbound: UAC works on FULL/HIGH only, and the 2.0.2
  // capability bits never yield SUPER.
  static constexpr Speed K_SPEED_UNBOUND = Speed::SUPER;

  // ===== 诊断计数 / Diagnostics counters =====
  /** @brief 补零次数（欠载） / Zero-filled frames (underruns) */
  uint32_t UnderrunCount() const { return underruns_.load(std::memory_order_relaxed); }
  /** @brief 等时请求失败次数 / Failed isochronous requests */
  uint32_t MissedTransferCount() const
  {
    return transport_misses_.load(std::memory_order_relaxed);
  }
  /** @brief 无空闲槽丢弃的帧数 / Frames dropped for lack of a free slot */
  uint32_t DroppedFrameCount() const
  {
    return frame_drops_.load(std::memory_order_relaxed);
  }
  /** @brief 当前速度缓存（解绑后为 K_SPEED_UNBOUND） / Cached speed (K_SPEED_UNBOUND
   *         after unbind) */
  Speed CurrentSpeed() const { return speed_; }

 private:
  /// SET_CUR 数据阶段待应用的控制 / Control to apply from a SET_CUR data stage
  enum class PendingControl : uint8_t
  {
    NONE,
    SET_SF,
    SET_MUTE,
    SET_VOLUME,
    SET_RES
  };

  // ===== 端点传输 / Endpoint transfers =====
  /**
   * @brief IN 传输完成静态回调
   *        Static callback for IN transfer completion
   */
  static void OnInCompleteStatic(bool in_isr, UAC1MicrophoneQ* self,
                                 const Endpoint::Completion& completion)
  {
    if (!self->inited_)
    {
      return;
    }
    self->OnInComplete(in_isr, completion);
  }

  /**
   * @brief IN 传输完成处理：成功不投帧（SOF 节拍负责）；FAILED 走恢复流程
   *        Handle IN completion: OK submits nothing (the SOF tick paces); FAILED runs
   *        the recovery flow
   */
  void OnInComplete(bool in_isr, const Endpoint::Completion& completion)
  {
    if (completion.result == ErrorCode::OK)
    {
      return;  // 下一帧由 SOF 节拍投递 / The next frame comes from the SOF tick
    }
    if (completion.result == ErrorCode::ABORTED)
    {
      // 有意的停止（解绑、SET_INTERFACE alt=0、恢复流程的 Close）：谁发起谁负责
      // 置 activate_pending_，这里不能置——否则 GET_INTERFACE 会把 alt=0 报成 1。
      // An intentional stop (unbind, SET_INTERFACE alt=0, a recovery Close): the
      // initiator owns activate_pending_, and setting it here would make
      // GET_INTERFACE report alt=0 as 1.
      return;
    }

    // 等时请求未完成（FAILED / OUT_OF_RANGE，MODEL_USB §4.6）：只置待决标志。
    // §2.7 禁止在完成上下文里做生命周期操作（Close/Configure），恢复由下一拍
    // 非中断 SOF 经 ActivateStreaming 执行。
    // The isochronous request did not finish (FAILED / OUT_OF_RANGE, MODEL_USB
    // §4.6): only raise the pending flag. §2.7 forbids lifecycle operations
    // (Close/Configure) from the completion context, so the recovery runs from the
    // next non-ISR SOF via ActivateStreaming.
    transport_misses_.fetch_add(1U, std::memory_order_relaxed);
    active_ = false;
    acc_rem_ = 0;
    micro_debt_ = 0;
    activate_pending_ = true;
  }

  /**
   * @brief 挂起的激活有进展就推进（停止确认后重新 Configure 并踢第一帧）
   *        Advance a pending activation (re-Configure and kick once the stop confirms)
   */
  void TryActivate(bool in_isr)
  {
    if (activate_pending_)
    {
      (void)ActivateStreaming(in_isr);
    }
  }

  /**
   * @brief 激活流态：端点已停止时重新 Configure 并投第一帧
   *        Activate streaming: re-Configure the stopped endpoint and kick one frame
   *
   * @return OK：已开启或仍在等停止确认；其他：Configure 失败，流保持关闭
   *         OK: streaming on, or the stop is still unconfirmed; otherwise Configure
   *         failed and streaming stays off
   */
  ErrorCode ActivateStreaming(bool in_isr)
  {
    if (!inited_ || ep_iso_in_ == nullptr)
    {
      return ErrorCode::FAILED;
    }
    // 恢复所需的生命周期操作（Close）在这里做：本函数只从 owner 上下文（非中
    // 断 SOF 或 SET_INTERFACE）进入，满足 §2.7。
    // The lifecycle op the recovery needs (Close) happens here: this function is
    // only entered from the owner context (a non-ISR SOF or SET_INTERFACE), which
    // satisfies §2.7.
    if (!ep_iso_in_->IsStopping() && ep_iso_in_->GetState() != Endpoint::State::DISABLED)
    {
      ep_iso_in_->Close(in_isr);
    }
    if (ep_iso_in_->IsStopping() || ep_iso_in_->GetState() != Endpoint::State::DISABLED)
    {
      return ErrorCode::OK;  // 等停止确认，activate_pending_ 保持 / Wait, stay pending
    }
    const ErrorCode result = ep_iso_in_->Configure(
        {Endpoint::Direction::IN, Endpoint::Type::ISOCHRONOUS, w_max_packet_size_},
        in_isr);
    activate_pending_ = false;
    if (result != ErrorCode::OK)
    {
      return result;  // 保持非流态（NO_BUFF / ARG_ERR / NOT_SUPPORT） / Streaming off
    }
    acc_rem_ = 0;
    micro_debt_ = 0;
    active_ = true;
    KickOneFrame(in_isr);
    return ErrorCode::OK;
  }

  /**
   * @brief 计算并提交一个服务区间的帧（取槽 → PopBatch 加补零 → 提交）
   *        Compute and submit one service interval (acquire → pop with zero fill →
   *        submit)
   */
  void KickOneFrame(bool in_isr)
  {
    if (!active_ || ep_iso_in_ == nullptr)
    {
      return;
    }

    // 本服务区间应发的音频帧数（Bresenham 余数进位）。先推进累计器：丢弃的帧也占用
    // 实时区间，长期速率以真实时间为准。
    // Audio frames due in this service interval (Bresenham remainder carry). The
    // accumulator advances first: a dropped frame still consumes real time, so the
    // long-term rate follows the wall clock.
    size_t frames = base_frames_per_service_;
    acc_rem_ += rem_frames_per_service_;
    if (acc_rem_ >= service_denominator_)
    {
      ++frames;
      acc_rem_ -= service_denominator_;
    }
    const size_t amount = frames * FRAME_BYTES;

    Endpoint::Slot slot;
    if (ep_iso_in_->AcquireSlot(slot, in_isr) != ErrorCode::OK)
    {
      // 无空闲槽：本帧丢弃并计数（MODEL_USB §4.6）。
      // No free slot: drop the frame and count it (MODEL_USB §4.6).
      frame_drops_.fetch_add(1U, std::memory_order_relaxed);
      return;
    }

    REQUIRE(amount <= w_max_packet_size_);
    REQUIRE(slot.data.size_ >= amount);
    // PopBatch 加补零：取走整数个音频帧，不足的部分补零（静音）。
    // PopBatch plus zero fill: take whole audio frames, zero-fill the remainder
    // (silence).
    size_t available = LibXR::min(pcm_queue_.Size(), amount);
    available -= available % FRAME_BYTES;
    if (available != 0U && pcm_queue_.PopBatch(static_cast<uint8_t*>(slot.data.addr_),
                                               available) != ErrorCode::OK)
    {
      available = 0;  // 理论不可达（SPSC 单消费者）；按全补零处理 / Not reachable;
                      // degrade to full zero fill
    }
    if (available < amount)
    {
      underruns_.fetch_add(1U, std::memory_order_relaxed);
      // 本类以静音为欠载回退；USB 核心没有音频语义，不会改写已提交的槽内容。
      // This class falls back to silence on underrun; the USB core has no audio
      // semantics and never overwrites submitted slot contents.
      Memory::FastSet(static_cast<uint8_t*>(slot.data.addr_) + available, 0,
                      amount - available);
    }

    // 提交。ABORTED = 槽在解绑前借出（复位 / 切配置）：本帧丢弃，槽已回到空闲，不重试。
    // Submit. ABORTED = the slot went stale before submission (reset / config switch):
    // drop the frame, the slot is free again, no retry.
    (void)ep_iso_in_->Submit(slot, amount, false, in_isr);
  }

  /**
   * @brief 统一重算时序与包长（运行时 MPS 钳制到描述符宣告值）
   *        Recompute timing and packet sizing (runtime MPS clamped to descriptor)
   */
  bool RecomputeTiming()
  {
    if (interval_ < 1U || interval_ > 16U) return false;
    const uint32_t ticks = speed_ == Speed::HIGH ? (1U << (interval_ - 1U)) : 1U;
    service_denominator_ = speed_ == Speed::HIGH ? 8000U : 1000U;
    const uint64_t numerator = static_cast<uint64_t>(sr_hz_) * ticks;
    base_frames_per_service_ = static_cast<uint32_t>(numerator / service_denominator_);
    rem_frames_per_service_ = static_cast<uint32_t>(numerator % service_denominator_);
    // 一个完整音频帧的余量同时容纳异步源的包长抖动。任何字节级取整都不会拆散采样帧。
    // One extra complete audio frame also leaves room for the async source's
    // packet-size variation. No byte-wise rounding can split a sample frame.
    const uint64_t maximum =
        (static_cast<uint64_t>(base_frames_per_service_) + 1U) * FRAME_BYTES;
    const uint32_t limit = speed_ == Speed::HIGH ? 1024U : 1023U;
    if (maximum > limit) return false;
    w_max_packet_size_ = static_cast<uint16_t>(maximum);
    acc_rem_ = 0;
    return true;
  }

  // 端点与接口状态。
  // Endpoint and interface state.
  Endpoint::EPNumber iso_in_ep_num_;
  Endpoint* ep_iso_in_ = nullptr;
  uint8_t itf_ac_num_ = 0;
  uint8_t itf_as_num_ = 0;
  const char* control_interface_string_ =
      nullptr;  ///< 控制接口字符串 / Control interface string
  const char* streaming_interface_string_ =
      nullptr;  ///< 流接口字符串 / Streaming interface string

  // 运行态状态。
  // Runtime state.
  bool inited_ = false;  ///< 已绑定 / Bound
  bool active_ = false;  ///< alt 1 已生效，等时端点在跑 / alt 1 active, EP running
  bool activate_pending_ = false;  ///< 等端点停止确认后进入流态 / Activation waits for
                                   ///< the stop confirmation
  PendingControl pending_control_ = PendingControl::NONE;

  // 音量与静音状态。
  // Volume and mute state.
  uint8_t mute_ = 0;
  int16_t vol_cur_ = 0;  // 0 dB
  int16_t vol_min_, vol_max_, vol_res_;

  uint8_t interval_;
  Speed speed_;

  // 采样与分帧参数。
  // Sampling and framing parameters.
  uint32_t sr_hz_;
  const uint32_t supported_rate_;
  static constexpr size_t FRAME_BYTES = CHANNELS * K_SUBFRAME_SIZE;
  uint32_t base_frames_per_service_ = 0;
  uint32_t rem_frames_per_service_ = 0;
  uint32_t acc_rem_ = 0;     // 0..999 / fixed-point remainder accumulator
  uint32_t micro_debt_ = 0;  ///< HS：未凑满一个服务区间的微帧 / HS: microframes short
                             ///< of one service interval
  uint16_t w_max_packet_size_ = 0;
  uint32_t service_denominator_ = 1000;

  UAC1DescBlock desc_block_;

  // 端点采样率缓存（Hz，小端 3 字节）与状态。
  // Endpoint sampling-frequency cache (3-byte LE) and flags.
  uint8_t sf_cur_[3] = {0, 0, 0};

  // PCM 字节队列。
  // PCM byte queue.
  LibXR::SPSCQueue<uint8_t> pcm_queue_;

  // 诊断计数。
  // Diagnostics counters.
  std::atomic<uint32_t> underruns_{0};
  std::atomic<uint32_t> transport_misses_{0};
  std::atomic<uint32_t> frame_drops_{0};

  // 端点完成回调包装。
  // Endpoint completion callback wrapper.
  Callback<const Endpoint::Completion&> on_in_complete_cb_ =
      Callback<const Endpoint::Completion&>::Create(OnInCompleteStatic, this);
};

}  // namespace LibXR::USB
