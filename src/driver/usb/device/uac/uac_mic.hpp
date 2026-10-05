#pragma once
#include <cstdint>
#include <cstring>

#include "core.hpp"
#include "desc_cfg.hpp"
#include "dev_core.hpp"
#include "ep.hpp"
#include "libxr_def.hpp"
#include "libxr_mem.hpp"

namespace LibXR::USB
{

/**
 * @brief UAC1 队列式麦克风设备类实现
 *        UAC1 queue‑driven microphone device class implementation
 *
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
   *        Construct a queue‑backed UAC1 microphone
   *
   * @param sample_rate_hz  采样率 / Sampling rate in Hz
   * @param vol_min         最小音量 / Min volume (1/256 dB)
   * @param vol_max         最大音量 / Max volume (1/256 dB)
   * @param vol_res         步进 / Step (1/256 dB)
   * @param speed           USB 速度 / USB device speed
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
        pcm_queue_(queue_bytes)
  {
    RecomputeTiming();
    // 缓存端点采样率（3 字节小端） / Cache current sampling frequency (3‑byte LE)
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

  // ===== 生产者接口 / Producer‑side API =====
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

  // UAC1 类请求 / Class‑specific requests
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
   *        Class‑specific endpoint (general)
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
   * @param in_isr 是否在中断中 / Whether in ISR
   */
  void BindEndpoints(EndpointPool& endpoint_pool, uint8_t start_itf_num, bool) override
  {
    inited_ = false;
    streaming_ = false;
    acc_rem_ = 0;

    if (speed_ == Speed::HIGH)
    {
      ASSERT(w_max_packet_size_ <= 1024);
    }
    else
    {
      ASSERT(interval_ == 1);
      ASSERT(w_max_packet_size_ <= 1023);
    }

    // 这里只分配端点，不立即开启；真正开启放在 AS Alt=1 切换时。
    // Allocate the endpoint here without starting transfers;
    // actual streaming begins when AS Alt=1 is selected.
    auto ans = endpoint_pool.Get(ep_iso_in_, Endpoint::Direction::IN, iso_in_ep_num_);
    ASSERT(ans == ErrorCode::OK);

    ep_iso_in_->Configure({Endpoint::Direction::IN, Endpoint::Type::ISOCHRONOUS,
                           static_cast<uint16_t>(w_max_packet_size_), true});

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
    // IN 传输完成后继续投下一帧。
    // Kick the next frame after each IN transfer completes.
    ep_iso_in_->SetOnTransferCompleteCallback(on_in_complete_cb_);

    // 把整块描述符数据交给 DeviceClass。
    // Publish the full descriptor block to DeviceClass.
    SetData(RawData{reinterpret_cast<uint8_t*>(&desc_block_), sizeof(desc_block_)});

    inited_ = true;
  }

  /**
   * @brief 反初始化设备，释放端点
   *        Deinitialize device and release endpoints
   *
   * @param endpoint_pool 端点池 / Endpoint pool
   * @param in_isr 是否在中断中 / Whether in ISR
   */
  void UnbindEndpoints(EndpointPool& endpoint_pool, bool) override
  {
    streaming_ = false;
    inited_ = false;
    if (ep_iso_in_)
    {
      ep_iso_in_->Close();
      ep_iso_in_->SetActiveLength(0);
      endpoint_pool.Release(ep_iso_in_);
      ep_iso_in_ = nullptr;
    }
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
   *        Handle class requests: EP sampling‑freq control + Feature Unit (mute/volume)
   */
  ErrorCode OnClassRequest(bool /*in_isr*/, uint8_t bRequest, uint16_t wValue,
                           uint16_t wLength, uint16_t wIndex,
                           DeviceClass::ControlTransferResult& r) override
  {
    // ===== 端点采样率控制（收件人：端点地址） / EP sampling‑freq control =====
    const uint8_t EP_ADDR = static_cast<uint8_t>(wIndex & 0xFF);
    const uint8_t CS_EP =
        static_cast<uint8_t>((wValue >> 8) & 0xFF);  // 0x01 = 采样率 / Sampling Freq

    if (EP_ADDR == desc_block_.ep_in.bEndpointAddress && CS_EP == 0x01)
    {
      switch (bRequest)
      {
        case GET_CUR:
          if (wLength == 3)
          {
            r.write_data = ConstRawData{sf_cur_, 3};
            return ErrorCode::OK;
          }
          break;
        case SET_CUR:
          if (wLength == 3)
          {
            pending_set_sf_ = true;
            r.read_data = RawData{sf_cur_, 3};
            return ErrorCode::OK;
          }
          break;
        case GET_MIN:
        case GET_MAX:
          if (wLength == 3)
          {
            r.write_data = ConstRawData{sf_cur_, 3};
            return ErrorCode::OK;
          }  // 单一频点 / single discrete rate
          break;
        case GET_RES:
        {
          static uint8_t one_hz[3] = {1, 0, 0};
          if (wLength == 3)
          {
            r.write_data = ConstRawData{one_hz, 3};
            return ErrorCode::OK;
          }
          break;
        }
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
          r.read_data = RawData{&mute_, 1};
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
              r.read_data = RawData{reinterpret_cast<uint8_t*>(&vol_cur_), 2};
              return ErrorCode::OK;
            }
            break;
          case GET_CUR:
            if (wLength == 2)
            {
              r.write_data = ConstRawData{reinterpret_cast<uint8_t*>(&vol_cur_), 2};
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
              r.read_data = RawData{reinterpret_cast<uint8_t*>(&vol_res_), 2};
              return ErrorCode::OK;
            }
          default:
            break;
        }

        return ErrorCode::ARG_ERR;

      default:

        return ErrorCode::NOT_SUPPORT;
    }
  }

  /**
   * @brief 处理类请求数据阶段（应用 SET_CUR 的采样率）
   *        Handle class request data stage (apply SET_CUR sampling freq)
   */
  ErrorCode OnClassData(bool /*in_isr*/, uint8_t bRequest,
                        LibXR::ConstRawData& /*data*/) override
  {
    if (bRequest == SET_CUR && pending_set_sf_)
    {
      const uint32_t NEW_SR = static_cast<uint32_t>(sf_cur_[0]) |
                              (static_cast<uint32_t>(sf_cur_[1]) << 8) |
                              (static_cast<uint32_t>(sf_cur_[2]) << 16);
      if (NEW_SR > 0 && NEW_SR != sr_hz_)
      {
        sr_hz_ = NEW_SR;
        RecomputeTiming();
      }
      pending_set_sf_ = false;
      return ErrorCode::OK;
    }
    return ErrorCode::OK;
  }

  /**
   * @brief 获取接口的当前 Alternate Setting
   *        Get current alternate setting for interface
   */
  ErrorCode GetAltSetting(uint8_t itf, uint8_t& alt) override
  {
    if (itf != itf_as_num_)
    {
      return ErrorCode::NOT_SUPPORT;
    }
    alt = streaming_ ? 1 : 0;
    return ErrorCode::OK;
  }

  /**
   * @brief 设置接口的 Alternate Setting（切换数据流）
   *        Set interface alternate setting (toggle streaming)
   */
  ErrorCode SetAltSetting(uint8_t itf, uint8_t alt) override
  {
    if (itf != itf_as_num_)
    {
      return ErrorCode::NOT_SUPPORT;
    }
    if (!ep_iso_in_)
    {
      return ErrorCode::FAILED;
    }

    switch (alt)
    {
      case 0:  // Alt 0：无端点 / Alt 0: no endpoint
        streaming_ = false;
        ep_iso_in_->SetActiveLength(0);
        ep_iso_in_->Close();
        return ErrorCode::OK;

      case 1:  // Alt 1：一个 Iso IN 端点 / Alt 1: one Iso IN endpoint
        ep_iso_in_->Configure({Endpoint::Direction::IN, Endpoint::Type::ISOCHRONOUS,
                               static_cast<uint16_t>(w_max_packet_size_), false});
        ep_iso_in_->SetActiveLength(0);
        acc_rem_ = 0;  // 重置余数累加器 / reset remainder accumulator
        streaming_ = true;
        KickOneFrame();
        return ErrorCode::OK;

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
    if (!inited_)
    {
      return false;
    }
    return ep_iso_in_->GetAddress() == ep_addr;
  }

 private:
  // ===== 端点传输 / Endpoint transfers =====
  /**
   * @brief IN 传输完成静态回调
   *        Static callback for IN transfer completion
   */
  static void OnInCompleteStatic(bool in_isr, UAC1MicrophoneQ* self, ConstRawData& data)
  {
    if (!self->inited_)
    {
      return;
    }
    self->OnInComplete(in_isr, data);
  }

  /**
   * @brief IN 传输完成处理：继续投下一帧
   *        Handle IN transfer completion: kick next frame
   */
  void OnInComplete(bool, ConstRawData&)
  {
    if (!streaming_)
    {
      return;
    }
    KickOneFrame();
  }

  /**
   * @brief 计算并投递一帧（按 1ms 变包 + 零填充）
   *        Compute and submit one frame (1 ms variable size + zero fill)
   */
  void KickOneFrame()
  {
    if (!streaming_)
    {
      return;  // 仅 Alt=1 允许 / Only allowed at Alt=1
    }
    if (!ep_iso_in_ || ep_iso_in_->GetState() != Endpoint::State::IDLE)
    {
      return;
    }

    // 本帧应发送字节数 = floor + 余数累加决定是否 +1。
    // Bytes sent this frame = floor(bytes/service) plus one when remainder carries.
    uint16_t to_send = static_cast<uint16_t>(base_bytes_per_service_);
    acc_rem_ += rem_bytes_per_service_;
    if (acc_rem_ >= service_hz_)
    {
      ++to_send;
      acc_rem_ -= service_hz_;
    }

    if (to_send > w_max_packet_size_)
    {
      to_send = static_cast<uint16_t>(w_max_packet_size_);
    }

    auto buf = ep_iso_in_->GetBuffer();
    if (buf.size_ < to_send)
    {
      to_send = static_cast<uint16_t>(buf.size_);
    }

    // 从队列取可用字节 / pop available bytes from queue
    size_t have = pcm_queue_.Size();
    size_t take = (have >= to_send) ? to_send : have;

    if (take)
    {
      pcm_queue_.PopBatch(reinterpret_cast<uint8_t*>(buf.addr_), take);
    }
    if (take < to_send)
    {
      LibXR::Memory::FastSet(static_cast<uint8_t*>(buf.addr_) + take, 0, to_send - take);
    }

    ep_iso_in_->Transfer(to_send);
  }

  /**
   * @brief 统一重算时序与包长（运行时 MPS 钳制到描述符宣告值）
   *        Recompute timing and packet sizing (runtime MPS clamped to descriptor)
   */
  void RecomputeTiming()
  {
    // 1) 计算每秒服务次数（FS=1000Hz；HS=8000/2^(bInterval-1)）。
    // 1) Compute service frequency (FS=1000Hz; HS=8000/2^(bInterval-1)).
    if (speed_ == Speed::HIGH)
    {
      uint8_t eff = interval_ ? interval_ : 1;
      if (eff > 16)
      {
        eff = 16;
      }
      const uint32_t MICROFRAMES = 1u
                                   << (eff - 1u);  // 2^(bInterval-1) 个微帧 / microframes
      service_hz_ = 8000u / MICROFRAMES;  // 8000 微帧/秒 / microframes per second
    }
    else
    {
      service_hz_ = 1000u;  // FS 等时：规范上 bInterval 必须为 1 帧 / FS isochronous
                            // requires bInterval=1
    }

    // 2) 计算每服务周期应送字节。
    // 2) Compute the target bytes per service interval.
    bytes_per_sec_ = static_cast<uint32_t>(sr_hz_) * CHANNELS * K_SUBFRAME_SIZE;
    base_bytes_per_service_ = bytes_per_sec_ / service_hz_;
    rem_bytes_per_service_ = bytes_per_sec_ % service_hz_;
    uint32_t ceil_bpt = base_bytes_per_service_ + (rem_bytes_per_service_ ? 1u : 0u);

    // 3) 钳制每事务上限（FS 1023，HS 1024；此处只做单事务上限，未用 HS multiplier）。
    // 3) Clamp the per-transaction limit (FS 1023, HS 1024; no HS multiplier here).
    const uint32_t PER_TX_LIMIT = (speed_ == Speed::HIGH) ? 1024u : 1023u;
    if (ceil_bpt > PER_TX_LIMIT)
    {
      ceil_bpt = PER_TX_LIMIT;
    }

    w_max_packet_size_ = static_cast<uint16_t>(ceil_bpt);

    // 4) 若已构建过描述符，则运行时能力不得超过宣告值。
    // 4) Once descriptors are built, runtime capability must not exceed the advertised
    // value.
    if (desc_block_.ep_in.wMaxPacketSize != 0 &&
        w_max_packet_size_ > desc_block_.ep_in.wMaxPacketSize)
    {
      w_max_packet_size_ = desc_block_.ep_in.wMaxPacketSize;
    }
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
  bool inited_ = false;
  bool streaming_ = false;

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
  uint32_t bytes_per_sec_ = 0;
  uint32_t base_bytes_per_service_ = 0;
  uint32_t rem_bytes_per_service_ = 0;
  uint32_t acc_rem_ = 0;  // 0..999 / fixed-point remainder accumulator
  uint16_t w_max_packet_size_ = 0;
  uint32_t service_hz_ = 1000;

  UAC1DescBlock desc_block_;

  // 端点采样率缓存（Hz，小端 3 字节）与状态。
  // Endpoint sampling-frequency cache (3-byte LE) and flags.
  uint8_t sf_cur_[3] = {0, 0, 0};
  bool pending_set_sf_ = false;

  // PCM 字节队列。
  // PCM byte queue.
  LibXR::SPSCQueue<uint8_t> pcm_queue_;

  // 端点回调包装。
  // Endpoint callback wrapper.
  LibXR::Callback<LibXR::ConstRawData&> on_in_complete_cb_ =
      LibXR::Callback<LibXR::ConstRawData&>::Create(OnInCompleteStatic, this);
};

}  // namespace LibXR::USB
