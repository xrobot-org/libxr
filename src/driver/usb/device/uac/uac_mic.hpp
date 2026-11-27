#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

#include "device_class.hpp"
#include "ep.hpp"
#include "libxr_cb.hpp"
#include "libxr_def.hpp"
#include "libxr_mem.hpp"
#include "spsc_queue.hpp"

namespace LibXR::USB
{

/**
 * @class UAC1MicrophoneQ
 * @brief UAC1 麦克风，由 PCM 队列供数 / UAC1 microphone fed from a PCM queue
 *
 * 两个接口：音频控制（AC：输入端子 → 特性单元 → 输出端子）和音频流（AS：alt 0 没有端点，
 * alt 1 一个等时 IN 端点）。应用用 WritePcm 写入交错的 PCM 字节；alt 1 生效期间，每个服务
 * 区间（全速每个 SOF，高速凑满 2^(interval-1) 个微帧）提交一个包：借槽、从队列取整数个
 * 音频帧、不足部分补零。
 *
 * Two interfaces: audio control (AC: input terminal -> feature unit -> output terminal)
 * and audio streaming (AS: alt 0 without an endpoint, alt 1 with one isochronous IN
 * endpoint). The application writes interleaved PCM bytes with WritePcm; while alt 1 is
 * in effect, each service interval (every SOF at full speed, 2^(interval-1) microframes
 * at high speed) submits one packet: borrow a slot, take whole audio frames from the
 * queue, fill the rest with zeros.
 *
 * @note 除注明"任何上下文"的函数外，本类的函数都只在设备的 owner 中运行。ep_ 非空当且
 *       仅当已绑定；streaming_ 为真当且仅当 alt 1 生效（此时端点已配置）。
 *       Except for the functions marked "any context", everything here runs in the
 *       device's owner. ep_ is non-null exactly while bound; streaming_ is true exactly
 *       while alt 1 is in effect (the endpoint is configured then).
 * @tparam CHANNELS 通道数 1-8 / Number of channels, 1-8
 * @tparam BITS_PER_SAMPLE 8、16 或 24 / 8, 16 or 24
 */
template <uint8_t CHANNELS, uint8_t BITS_PER_SAMPLE>
class UAC1MicrophoneQ : public DeviceClass
{
  static_assert(CHANNELS >= 1 && CHANNELS <= 8, "CHANNELS out of range");
  static_assert(BITS_PER_SAMPLE == 8 || BITS_PER_SAMPLE == 16 || BITS_PER_SAMPLE == 24,
                "BITS_PER_SAMPLE must be 8/16/24");

  /// 每通道每采样的字节数 / Bytes per channel per sample
  static constexpr uint8_t SUBFRAME_SIZE = BITS_PER_SAMPLE / 8U;
  /// 一个音频帧（所有通道的一次采样）的字节数 / Bytes of one audio frame (one sample of
  /// every channel)
  static constexpr size_t FRAME_BYTES = static_cast<size_t>(CHANNELS) * SUBFRAME_SIZE;

 public:
  /// AC 接口的默认字符串 / Default string of the AC interface
  static constexpr const char* DEFAULT_CONTROL_INTERFACE_STRING = "XRUSB UAC1 Control";
  /// AS 接口的默认字符串 / Default string of the AS interface
  static constexpr const char* DEFAULT_STREAMING_INTERFACE_STRING =
      "XRUSB UAC1 Streaming";

  /// UAC1 类请求 / UAC1 class requests
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

  /// 特性单元的控制选择子 / Feature unit control selectors
  enum : uint8_t
  {
    FU_MUTE = 0x01,
    FU_VOLUME = 0x02
  };

  /// 端点控制选择子：采样率 / Endpoint control selector: sampling frequency
  static constexpr uint8_t SAMPLING_FREQ_CONTROL = 0x01U;

  /// 实体 ID / Entity IDs
  enum : uint8_t
  {
    ID_IT_MIC = 1,
    ID_FU = 2,
    ID_OT_USB = 3
  };

  /**
   * @brief 构造 / Construct
   * @param iso_in_ep_num 等时 IN 端点号 / Isochronous IN endpoint number
   * @param sample_rate_hz 采样率，1..0xffffff / Sampling rate, 1..0xffffff
   * @param vol_min 最小音量（1/256 dB）/ Minimum volume (1/256 dB)
   * @param vol_max 最大音量（1/256 dB）/ Maximum volume (1/256 dB)
   * @param vol_res 音量步进（1/256 dB）/ Volume step (1/256 dB)
   * @param speed 构造时检查这个速度下采样率能否传输；不保存，描述符按实际速度写。
   *        The rate is checked at construction to be carried at this speed; not kept,
   *        the descriptors follow the actual speed.
   * @param queue_bytes PCM 队列字节数 / PCM queue size in bytes
   * @param interval 高速的 bInterval，1..16，默认 4（每毫秒一个包，与全速相同；每个微帧
   *        一个包要显式给 1）；全速固定为 1。
   *        High-speed bInterval, 1..16, 4 by default (one packet per millisecond, as at
   *        full speed; one packet per microframe has to be asked for with 1); always 1
   *        at full speed.
   * @param control_interface_string AC 接口字符串 / AC interface string
   * @param streaming_interface_string AS 接口字符串 / AS interface string
   * @note 分配 PCM 队列和端点回调，只在构造时分配。采样率在 speed 下放不进一个包时
   *       REQUIRE 失败。
   *       Allocates the PCM queue and the endpoint callback, at construction only.
   *       REQUIRE fails when the rate does not fit one packet at speed.
   */
  UAC1MicrophoneQ(
      Endpoint::EPNumber iso_in_ep_num, uint32_t sample_rate_hz, int16_t vol_min = 0,
      int16_t vol_max = 0, int16_t vol_res = 1, Speed speed = Speed::FULL,
      size_t queue_bytes = 2048U, uint8_t interval = 4U,
      const char* control_interface_string = DEFAULT_CONTROL_INTERFACE_STRING,
      const char* streaming_interface_string = DEFAULT_STREAMING_INTERFACE_STRING);

  /**
   * @brief 写入交错的 PCM 字节（如 S16LE、S24_3LE）/ Write interleaved PCM bytes
   *        (for example S16LE, S24_3LE)
   * @return OK；FULL：队列放不下，什么也没写 / OK; FULL: no room, nothing written
   * @note 任何上下文，单一写者（SPSC 队列的生产者）。
   *       Any context, one writer (the producer of the SPSC queue).
   */
  ErrorCode WritePcm(const void* data, size_t nbytes);
  /// 队列中的字节数；任何上下文 / Bytes queued; any context
  size_t QueueSize() const { return pcm_queue_.Size(); }
  /// 队列剩余空间；任何上下文 / Free bytes in the queue; any context
  size_t QueueSpace() const { return pcm_queue_.EmptySize(); }
  /**
   * @brief 清空队列 / Empty the queue
   * @note 清空是消费者一侧的操作：只在没有流传输时调用（未配置或 alt 0）。
   *       Emptying is a consumer-side operation: call it only while not streaming
   *       (unconfigured or alt 0).
   */
  void ResetQueue()
  {
    pcm_queue_.Reset();
    discard_left_ = 0U;
  }

 protected:
  /// @name DeviceClass 的实现 / DeviceClass implementation
  /// @{
  size_t GetInterfaceCount() const override { return 2U; }
  bool HasIAD() const override { return true; }
  size_t GetMaxConfigSize() const override { return sizeof(desc_block_); }
  const char* GetInterfaceString(size_t local_interface_index) const override;

  /**
   * @brief 写 IAD、AC 接口及其类描述符、AS 的两个 alt、格式和端点描述符 / Write the
   *        IAD, the AC interface with its class descriptors, both AS alternates, the
   *        format and the endpoint descriptors
   * @return OK；NOT_SUPPORT：该速度下采样率放不进一个包，或端点查询得 0；NO_BUFF：端点
   *         能给的包长小于采样率需要的包长。
   *         OK; NOT_SUPPORT: the rate does not fit one packet at that speed, or the
   *         endpoint query returned 0; NO_BUFF: the endpoint offers a smaller packet
   *         than the rate needs.
   * @note 记下写块时的速度，供紧接着的 BindEndpoints 使用。
   *       Records the speed of the block for the BindEndpoints that follows.
   */
  ErrorCode WriteDescriptors(const EndpointPool& pool, Speed speed,
                             uint8_t first_interface, ConstRawData& block) override;

  /**
   * @brief 取端点、Configure、比较包长、装回调，再 Close（alt 0）/ Take the endpoint,
   *        configure it, compare the size, install the callback, then Close (alt 0)
   * @return OK；NOT_FOUND：Get 失败；CHECK_ERR：读回的包长与描述符不同；其他：Configure
   *         的错误（后端拒绝这个配置时为它的 CheckHardware 的错误）。失败时已取的端点留在
   *         ep_ 里，由随后的 UnbindEndpoints 收回。
   *         OK; NOT_FOUND: Get failed; CHECK_ERR: the size read back differs from the
   *         descriptor; otherwise the error of Configure (the error of the backend's
   *         CheckHardware when it refuses the configuration). On failure the endpoint
   *         taken stays in ep_ and the following UnbindEndpoints takes it back.
   */
  [[nodiscard]] ErrorCode BindEndpoints(EndpointPool& pool, uint8_t first_interface,
                                        bool in_isr) override;

  /**
   * @brief 停流，关闭并交回端点 / Stop streaming, close and return the endpoint
   * @note 先把 ep_ 置空再 Close，所以 Close 交付的 ABORTED 看到"未绑定"。队列里的 PCM
   *       保留；已拷进槽、被停的包丢弃。
   *       ep_ is cleared before Close, so the ABORTED completions Close delivers see
   *       "unbound". PCM still queued is kept; packets already copied into slots and
   *       stopped are dropped.
   */
  void UnbindEndpoints(EndpointPool& pool, bool in_isr) override;

  /**
   * @brief SET_INTERFACE / SET_INTERFACE
   * @return OK；NOT_SUPPORT：未绑定、不是本类的接口，或 AC 接口的非 0 alt；ARG_ERR：AS
   *         接口的 alt 大于 1；其他：alt 1 的启流错误，见 StartStreaming。
   *         OK; NOT_SUPPORT: unbound, not this class's interface, or a non-zero alt of
   *         the AC interface; ARG_ERR: an AS alt above 1; otherwise the error of
   *         starting alt 1, see StartStreaming.
   * @note AS 接口的 alt 0 和 alt 1 都先 Close 端点：在途和排队的包各以 ABORTED 结束，
   *       回调在 Close 返回前都已进行（硬件上的停止可以还没结束）。alt 1 再由
   *       StartStreaming 重新配置并提交第一个包。
   *       Both alt 0 and alt 1 of the AS interface first Close the endpoint: the
   *       in-flight and queued packets end with ABORTED, and the callbacks have all run
   *       when Close returns (the stop in hardware may still be pending). alt 1 then
   *       reconfigures and submits the first packet through StartStreaming.
   */
  ErrorCode SetAltSetting(bool in_isr, uint8_t itf, uint8_t alt) override;

  /// AS 接口：alt 1 生效时为 1，否则 0；其他接口 NOT_SUPPORT / AS interface: 1 while
  /// alt 1 is in effect, else 0; NOT_SUPPORT for other interfaces
  ErrorCode GetAltSetting(uint8_t itf, uint8_t& alt) override;

  /// 等时 IN 端点地址；由端点号算出，不需要已绑定 / The isochronous IN address;
  /// computed from the number, no bind needed
  bool OwnsEndpoint(uint8_t ep_addr) const override;

  /**
   * @brief 类请求的 SETUP 阶段 / SETUP stage of a class request
   * @return 端点为接收者：采样率的 GET_CUR/MIN/MAX/RES 和 SET_CUR（3 字节）；接口为接收
   *         者：AC 接口上特性单元的静音（1 字节）和音量（2 字节）。方向或长度不符、通道号
   *         大于 CHANNELS 给 ARG_ERR；其他 NOT_SUPPORT（核心 STALL）。
   *         Endpoint recipient: GET_CUR/MIN/MAX/RES and SET_CUR of the sampling
   *         frequency (3 bytes); interface recipient: mute (1 byte) and volume
   *         (2 bytes) of the feature unit on the AC interface. A wrong direction or
   *         length, or a channel above CHANNELS, gives ARG_ERR; anything else
   *         NOT_SUPPORT (the core stalls).
   * @note 应答写进 reply_，按小端编码；下一次请求之前不变。
   *       The reply goes into reply_, little-endian; unchanged until the next request.
   */
  ErrorCode OnControlRequest(bool in_isr, const SetupPacket& setup,
                             ConstRawData& in_data) override;

  /**
   * @brief SET_CUR、SET_RES 的数据阶段 / Data stage of SET_CUR and SET_RES
   * @return OK；ARG_ERR：长度不对，或采样率不是构造时的那个；NOT_SUPPORT：其他。
   *         OK; ARG_ERR: wrong length, or a rate other than the one constructed with;
   *         NOT_SUPPORT: anything else.
   * @note SET_RES 接受但不改变什么：分辨率固定。
   *       SET_RES is accepted and changes nothing: the resolution is fixed.
   */
  ErrorCode OnControlData(bool in_isr, const SetupPacket& setup,
                          ConstRawData data) override;

  /**
   * @brief 帧起始：到了一个服务区间就提交一个包 / Start of frame: submit one packet per
   *        service interval
   * @note 全速每次调用算一个服务区间。高速按帧号和微帧号算出经过的微帧数（只报整帧的后端
   *       一帧算 8 个），累积满一个服务区间提交一次；错过的区间不补发，多出的整区间丢弃。
   *       At full speed every call is one service interval. At high speed the elapsed
   *       microframes are computed from the frame and microframe numbers (8 per frame
   *       for a backend that reports whole frames), and one packet goes out per full
   *       interval accrued; missed intervals are not replayed and surplus whole
   *       intervals are dropped.
   */
  void OnStartOfFrame(bool in_isr, uint16_t frame, uint8_t microframe) override;
  /// @}

  /// @name 诊断计数：任何上下文 / Diagnostics counters: any context
  /// @{

  /// 补过零的包数（欠载）/ Packets zero-filled (underruns)
  uint32_t UnderrunCount() const { return underruns_.load(std::memory_order_relaxed); }
  /// 以 FAILED 或 OUT_OF_RANGE 结束的包数 / Packets ended with FAILED or OUT_OF_RANGE
  uint32_t MissedTransferCount() const
  {
    return transport_misses_.load(std::memory_order_relaxed);
  }
  /// 没有空闲槽而丢弃的服务区间数 / Service intervals dropped for lack of a free slot
  uint32_t DroppedFrameCount() const
  {
    return frame_drops_.load(std::memory_order_relaxed);
  }
  /// @}

 private:
  /// 音频类代码 / Audio class codes
  enum : uint8_t
  {
    USB_CLASS_AUDIO = 0x01,
    SUBCLASS_AC = 0x01,
    SUBCLASS_AS = 0x02
  };
  /// 类特定描述符类型 / Class-specific descriptor types
  enum : uint8_t
  {
    CS_INTERFACE = 0x24,
    CS_ENDPOINT = 0x25
  };
  /// 描述符子类型 / Descriptor subtypes
  enum : uint8_t
  {
    AC_HEADER = 0x01,
    AC_INPUT_TERMINAL = 0x02,
    AC_OUTPUT_TERMINAL = 0x03,
    AC_FEATURE_UNIT = 0x06,
    AS_GENERAL = 0x01,
    AS_FORMAT_TYPE = 0x02,
    EP_GENERAL = 0x01
  };
  /// 没有记下帧号 / No frame number recorded
  static constexpr uint16_t NO_FRAME = 0xffffU;
  /// 后端只报告整帧 / The backend reports whole frames only
  static constexpr uint8_t WHOLE_FRAME = 0xffU;

  LIBXR_PACKED_BEGIN
  /// AC 头（UAC1 4.3.2）/ AC header (UAC1 4.3.2)
  struct ACHeader
  {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint8_t bDescriptorSubtype;
    uint16_t bcdADC;        ///< 1.00
    uint16_t wTotalLength;  ///< AC 类描述符总长 / Total of the AC class descriptors
    uint8_t bInCollection;  ///< AS 接口数 / Number of AS interfaces
    uint8_t baInterfaceNr;  ///< AS 接口号 / AS interface number
  };

  /// 输入端子（UAC1 4.3.2.1）/ Input terminal (UAC1 4.3.2.1)
  struct ACInputTerminal
  {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint8_t bDescriptorSubtype;
    uint8_t bTerminalID;
    uint16_t wTerminalType;  ///< 0x0201 麦克风 / 0x0201 microphone
    uint8_t bAssocTerminal;
    uint8_t bNrChannels;
    uint16_t wChannelConfig;  ///< 空间位置位图 / Spatial location bitmap
    uint8_t iChannelNames;
    uint8_t iTerminal;
  };

  /// 特性单元（UAC1 4.3.2.5）/ Feature unit (UAC1 4.3.2.5)
  struct ACFeatureUnit
  {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint8_t bDescriptorSubtype;
    uint8_t bUnitID;
    uint8_t bSourceID;
    uint8_t bControlSize;               ///< 每项 1 字节 / 1 byte per entry
    uint8_t bmaControls[CHANNELS + 1];  ///< 主通道和各通道 / Master and each channel
    uint8_t iFeature;
  };

  /// 输出端子（UAC1 4.3.2.2）/ Output terminal (UAC1 4.3.2.2)
  struct ACOutputTerminal
  {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint8_t bDescriptorSubtype;
    uint8_t bTerminalID;
    uint16_t wTerminalType;  ///< 0x0101 USB 流 / 0x0101 USB streaming
    uint8_t bAssocTerminal;
    uint8_t bSourceID;
    uint8_t iTerminal;
  };

  /// AS 通用（UAC1 4.5.2）/ AS general (UAC1 4.5.2)
  struct ASGeneral
  {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint8_t bDescriptorSubtype;
    uint8_t bTerminalLink;
    uint8_t bDelay;  ///< 帧 / Frames
    uint16_t wFormatTag;
  };

  /// Type I 格式，一个离散采样率（UAC1 格式 2.2.5）/ Type I format with one discrete
  /// rate (UAC1 formats 2.2.5)
  struct TypeIFormat
  {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint8_t bDescriptorSubtype;
    uint8_t bFormatType;
    uint8_t bNrChannels;
    uint8_t bSubframeSize;
    uint8_t bBitResolution;
    uint8_t bSamFreqType;
    uint8_t tSamFreq[3];  ///< 小端 / Little-endian
  };

  /// 9 字节的等时端点描述符（UAC1 4.6.1.1）/ 9-byte isochronous endpoint descriptor
  /// (UAC1 4.6.1.1)
  struct IsoEndpointDescriptor
  {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint8_t bEndpointAddress;
    uint8_t bmAttributes;  ///< 0x05：等时、异步、数据 / Isochronous, asynchronous, data
    uint16_t wMaxPacketSize;
    uint8_t bInterval;
    uint8_t bRefresh;
    uint8_t bSynchAddress;
  };

  /// 类特定等时端点（UAC1 4.6.1.2）/ Class-specific isochronous endpoint (UAC1 4.6.1.2)
  struct ASEndpointGeneral
  {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint8_t bDescriptorSubtype;
    uint8_t bmAttributes;
    uint8_t bLockDelayUnits;
    uint16_t wLockDelay;
  };

  /// 本类的整个描述符块 / This class's whole descriptor block
  struct DescriptorBlock
  {
    IADDescriptor iad;
    InterfaceDescriptor ac_intf;
    ACHeader ac_header;
    ACInputTerminal input_terminal;
    ACFeatureUnit feature_unit;
    ACOutputTerminal output_terminal;
    InterfaceDescriptor as_alt0;
    InterfaceDescriptor as_alt1;
    ASGeneral as_general;
    TypeIFormat format;
    IsoEndpointDescriptor iso_ep;
    ASEndpointGeneral iso_ep_cs;
  };
  LIBXR_PACKED_END

  /**
   * @brief 一个速度下的分帧 / Framing at one speed
   * @note 每个服务区间发 base_frames 个音频帧，余数 rem_frames / denominator 按
   *       Bresenham 进位；包长多留一个音频帧。
   *       Each service interval carries base_frames audio frames, and the remainder
   *       rem_frames / denominator carries over Bresenham style; the packet leaves room
   *       for one more audio frame.
   */
  struct Timing
  {
    uint16_t packet = 0U;       ///< wMaxPacketSize
    uint8_t interval = 0U;      ///< bInterval
    uint32_t microframes = 0U;  ///< 高速一个区间的微帧数；全速 0 / Microframes per
                                ///< interval at high speed; 0 at full speed
    uint32_t base_frames = 0U;  ///< 每区间的整数音频帧 / Whole frames per interval
    uint32_t rem_frames = 0U;   ///< 余数分子 / Remainder numerator
    uint32_t denominator = 1U;  ///< 余数分母 / Remainder denominator
  };

  /// 某个速度下的分帧；放不进一个包或速度不支持时返回 false / Framing at a speed;
  /// false when one packet cannot hold it or the speed is unsupported
  bool TimingFor(Speed speed, Timing& timing) const;

  /// 等时 IN 端点在某个分帧下的配置 / The isochronous IN configuration for a framing
  static Endpoint::Config IsoConfig(const Timing& timing)
  {
    return {Endpoint::Direction::IN, Endpoint::Type::ISOCHRONOUS, timing.packet};
  }

  /**
   * @brief 重新配置端点并开始流 / Reconfigure the endpoint and start streaming
   * @return OK；CHECK_ERR：读回的包长与绑定时不同（端点已关闭）；其他：Configure 的错误
   *         （STATE_ERR：刚才的停止失败，请求仍在途）。失败时 streaming_ 保持为假。
   *         OK; CHECK_ERR: the size read back differs from the bound one (the endpoint
   *         is closed again); otherwise the error of Configure (STATE_ERR: the stop just
   *         made failed and the request is still in flight). On failure streaming_
   *         stays false.
   * @note 端点必须已 Close。比较的是绑定时的 timing_.packet，不读描述符块：已绑定时
   *       OTHER_SPEED 的 GET_DESCRIPTOR 会改写那个块。
   *       The endpoint must be closed. The comparison uses timing_.packet from the
   *       bind, not the descriptor block: an OTHER_SPEED GET_DESCRIPTOR while bound
   *       rewrites that block.
   */
  ErrorCode StartStreaming(bool in_isr);

  /// 自上一次 SOF 经过的单位数（帧或微帧，与 microframe 一致）；不知道时 0 / Units
  /// (frames or microframes, as microframe) since the previous SOF; 0 when unknown
  uint32_t Elapsed(uint16_t frame, uint8_t microframe);

  /// 提交一个服务区间的包：借槽、取整数个音频帧、补零、提交 / Submit one service
  /// interval: borrow a slot, take whole audio frames, zero-fill, submit
  void KickOneFrame(bool in_isr);

  /**
   * @brief 开始推流时丢弃开始前写入的旧 PCM（消费者一侧出队）/ Drop PCM queued before
   *        streaming starts (dequeued on the consumer side)
   */
  void DropStalePcm();

  /// 等时 IN 的完成回调入口 / Completion entry of the isochronous IN endpoint
  static void OnInCompleteStatic(bool in_isr, UAC1MicrophoneQ* self,
                                 const Endpoint::Completion& completion);

  /// 把 value 的低 bytes 个字节按小端写进 reply_ / Write the low bytes of value into
  /// reply_, little-endian
  ConstRawData Reply(uint32_t value, size_t bytes);

  /// @name 构造时固定 / Fixed at construction
  /// @{
  Endpoint::EPNumber iso_in_num_;                ///< 等时 IN 端点号 / Iso IN number
  const char* control_interface_string_;         ///< AC 接口字符串 / AC string
  const char* streaming_interface_string_;       ///< AS 接口字符串 / AS string
  int16_t vol_min_;                              ///< 最小音量 / Minimum volume
  int16_t vol_max_;                              ///< 最大音量 / Maximum volume
  int16_t vol_res_;                              ///< 音量步进 / Volume step
  uint8_t interval_;                             ///< 高速 bInterval / HS bInterval
  uint32_t sample_rate_;                         ///< 采样率 / Sampling rate
  Callback<const Endpoint::Completion&> in_cb_;  ///< 端点回调 / Endpoint callback
  /// @}

  /// PCM 队列：WritePcm 生产（任何上下文），KickOneFrame 消费（owner）/ PCM queue:
  /// produced by WritePcm (any context), consumed by KickOneFrame (owner)
  LibXR::SPSCQueue<uint8_t> pcm_queue_;

  /// @name 仅 owner / Owner only
  /// @{
  Endpoint* ep_ = nullptr;  ///< 等时 IN，绑定时非空 / Iso IN, non-null while bound
  /// AC 接口号，AS 为它加一；BindEndpoints 写 / AC interface number, AS is one above;
  /// written by BindEndpoints
  uint8_t ac_interface_ = 0U;
  /**
   * @brief alt 1 生效 / alt 1 in effect
   * @note SetAltSetting、UnbindEndpoints 写，SOF 和 GET_INTERFACE 读。
   *       Written by SetAltSetting and UnbindEndpoints, read by SOF and GET_INTERFACE.
   */
  bool streaming_ = false;
  /// 跨开流边界拆散音频帧后，下一次写新流前还需补齐丢弃的尾字节数（0..FRAME_BYTES-1）/
  /// Remaining bytes of a split pre-stream audio frame to discard before streaming
  /// resumes (0..FRAME_BYTES-1)
  size_t discard_left_ = 0U;
  /// 绑定时的分帧；BindEndpoints 写 / Framing at the bind; written by BindEndpoints
  Timing timing_{};
  /// Bresenham 余数，0..denominator-1；KickOneFrame 写 / Bresenham remainder,
  /// 0..denominator-1; written by KickOneFrame
  uint32_t acc_rem_ = 0U;
  /// 高速：还不够一个区间的微帧 / High speed: microframes short of one interval
  uint32_t micro_debt_ = 0U;
  uint16_t last_frame_ = NO_FRAME;  ///< 上一次 SOF 的帧号 / Previous SOF frame
  uint8_t last_microframe_ = 0U;    ///< 上一次 SOF 的微帧 / Previous SOF microframe
  uint8_t mute_ = 0U;               ///< 静音（主机设）/ Mute (set by the host)
  int16_t volume_ = 0;              ///< 当前音量（主机设）/ Volume (set by the host)
  /// 类请求的应答，最长 3 字节 / Reply of a class request, at most 3 bytes
  uint8_t reply_[3] = {};
  /// WriteDescriptors 写出的块 / Block written by WriteDescriptors
  DescriptorBlock desc_block_{};
  /// desc_block_ 对应的速度；WriteDescriptors 写，BindEndpoints 读 / Speed of
  /// desc_block_; written by WriteDescriptors, read by BindEndpoints
  Speed block_speed_ = Speed::FULL;
  /// @}

  /// @name 任何上下文 / Any context
  /// @{
  std::atomic<uint32_t> underruns_{0U};         ///< 见 UnderrunCount / See UnderrunCount
  std::atomic<uint32_t> transport_misses_{0U};  ///< 见 MissedTransferCount
  std::atomic<uint32_t> frame_drops_{0U};       ///< 见 DroppedFrameCount
  /// @}
};

template <uint8_t CHANNELS, uint8_t BITS_PER_SAMPLE>
UAC1MicrophoneQ<CHANNELS, BITS_PER_SAMPLE>::UAC1MicrophoneQ(
    Endpoint::EPNumber iso_in_ep_num, uint32_t sample_rate_hz, int16_t vol_min,
    int16_t vol_max, int16_t vol_res, Speed speed, size_t queue_bytes, uint8_t interval,
    const char* control_interface_string, const char* streaming_interface_string)
    : iso_in_num_(iso_in_ep_num),
      control_interface_string_(control_interface_string),
      streaming_interface_string_(streaming_interface_string),
      vol_min_(vol_min),
      vol_max_(vol_max),
      vol_res_(vol_res),
      interval_(interval),
      sample_rate_(sample_rate_hz),
      in_cb_(Callback<const Endpoint::Completion&>::Create(OnInCompleteStatic, this)),
      pcm_queue_(queue_bytes)
{
  REQUIRE(sample_rate_ != 0U && sample_rate_ <= 0xffffffU);
  Timing timing;
  REQUIRE(TimingFor(speed, timing));
}

template <uint8_t CHANNELS, uint8_t BITS_PER_SAMPLE>
ErrorCode UAC1MicrophoneQ<CHANNELS, BITS_PER_SAMPLE>::WritePcm(const void* data,
                                                               size_t nbytes)
{
  return pcm_queue_.PushBatch(static_cast<const uint8_t*>(data), nbytes);
}

template <uint8_t CHANNELS, uint8_t BITS_PER_SAMPLE>
const char* UAC1MicrophoneQ<CHANNELS, BITS_PER_SAMPLE>::GetInterfaceString(
    size_t local_interface_index) const
{
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

template <uint8_t CHANNELS, uint8_t BITS_PER_SAMPLE>
bool UAC1MicrophoneQ<CHANNELS, BITS_PER_SAMPLE>::TimingFor(Speed speed,
                                                           Timing& timing) const
{
  if (speed != Speed::FULL && speed != Speed::HIGH)
  {
    return false;
  }
  const bool high = speed == Speed::HIGH;
  if (high && (interval_ < 1U || interval_ > 16U))
  {
    return false;
  }
  Timing t;
  t.interval = high ? interval_ : 1U;
  t.microframes = high ? (1UL << (interval_ - 1U)) : 0U;
  t.denominator = high ? 8000U : 1000U;
  // 一个服务区间的采样数 = 采样率 × 区间长度：全速 1 ms，高速 microframes × 125 us。
  // Samples per service interval = rate x interval length: 1 ms at full speed,
  // microframes x 125 us at high speed.
  const uint64_t numerator =
      static_cast<uint64_t>(sample_rate_) * (high ? t.microframes : 1U);
  const uint64_t base = numerator / t.denominator;
  // 多留一个音频帧，异步源的包长可以多一帧，也不会拆散音频帧。
  // One extra audio frame: an asynchronous source may send one more frame, and no
  // audio frame is ever split.
  const uint64_t packet = (base + 1U) * FRAME_BYTES;
  if (packet > (high ? 1024U : 1023U))
  {
    return false;
  }
  t.base_frames = static_cast<uint32_t>(base);
  t.rem_frames = static_cast<uint32_t>(numerator % t.denominator);
  t.packet = static_cast<uint16_t>(packet);
  timing = t;
  return true;
}

template <uint8_t CHANNELS, uint8_t BITS_PER_SAMPLE>
ErrorCode UAC1MicrophoneQ<CHANNELS, BITS_PER_SAMPLE>::WriteDescriptors(
    const EndpointPool& pool, Speed speed, uint8_t first_interface, ConstRawData& block)
{
  Timing t;
  if (!TimingFor(speed, t))
  {
    return ErrorCode::NOT_SUPPORT;
  }
  const uint16_t size = pool.PacketSizeFor(iso_in_num_, IsoConfig(t), speed);
  if (size == 0U)
  {
    return ErrorCode::NOT_SUPPORT;
  }
  if (size != t.packet)
  {
    // 包长小了，这个采样率就传不完。
    // A smaller packet cannot carry the rate.
    return ErrorCode::NO_BUFF;
  }

  const auto ac_itf = first_interface;
  const auto as_itf = static_cast<uint8_t>(first_interface + 1U);
  const auto interface = static_cast<uint8_t>(DescriptorType::INTERFACE);
  DescriptorBlock& d = desc_block_;
  d.iad = {8U,
           static_cast<uint8_t>(DescriptorType::IAD),
           ac_itf,
           2U,
           USB_CLASS_AUDIO,
           SUBCLASS_AC,
           0U,
           0U};
  d.ac_intf = {9U,          interface, ac_itf,
               0U,          0U,        USB_CLASS_AUDIO,
               SUBCLASS_AC, 0U,        GetInterfaceStringIndex(0U)};
  d.ac_header = {sizeof(ACHeader),
                 CS_INTERFACE,
                 AC_HEADER,
                 0x0100U,
                 static_cast<uint16_t>(sizeof(ACHeader) + sizeof(ACInputTerminal) +
                                       sizeof(ACFeatureUnit) + sizeof(ACOutputTerminal)),
                 1U,
                 as_itf};
  // 双声道声明左前、右前；其余不声明位置。
  // Two channels declare left front and right front; others declare no position.
  d.input_terminal = {sizeof(ACInputTerminal),
                      CS_INTERFACE,
                      AC_INPUT_TERMINAL,
                      ID_IT_MIC,
                      0x0201U,
                      0U,
                      CHANNELS,
                      static_cast<uint16_t>(CHANNELS == 2U ? 0x0003U : 0x0000U),
                      0U,
                      0U};
  d.feature_unit.bLength = sizeof(ACFeatureUnit);
  d.feature_unit.bDescriptorType = CS_INTERFACE;
  d.feature_unit.bDescriptorSubtype = AC_FEATURE_UNIT;
  d.feature_unit.bUnitID = ID_FU;
  d.feature_unit.bSourceID = ID_IT_MIC;
  d.feature_unit.bControlSize = 1U;
  // 主通道和每个通道都声明静音和音量；各通道共用同一个值。
  // The master and every channel declare mute and volume; all channels share one value.
  for (uint8_t& controls : d.feature_unit.bmaControls)
  {
    controls = 0x03U;
  }
  d.feature_unit.iFeature = 0U;
  d.output_terminal = {sizeof(ACOutputTerminal),
                       CS_INTERFACE,
                       AC_OUTPUT_TERMINAL,
                       ID_OT_USB,
                       0x0101U,
                       0U,
                       ID_FU,
                       0U};
  d.as_alt0 = {9U,          interface, as_itf,
               0U,          0U,        USB_CLASS_AUDIO,
               SUBCLASS_AS, 0U,        GetInterfaceStringIndex(1U)};
  d.as_alt1 = {9U,          interface, as_itf,
               1U,          1U,        USB_CLASS_AUDIO,
               SUBCLASS_AS, 0U,        GetInterfaceStringIndex(1U)};
  d.as_general = {sizeof(ASGeneral), CS_INTERFACE, AS_GENERAL, ID_OT_USB, 1U, 0x0001U};
  d.format = {
      sizeof(TypeIFormat),
      CS_INTERFACE,
      AS_FORMAT_TYPE,
      0x01U,
      CHANNELS,
      SUBFRAME_SIZE,
      BITS_PER_SAMPLE,
      1U,
      {static_cast<uint8_t>(sample_rate_), static_cast<uint8_t>(sample_rate_ >> 8U),
       static_cast<uint8_t>(sample_rate_ >> 16U)}};
  d.iso_ep = {sizeof(IsoEndpointDescriptor),
              static_cast<uint8_t>(DescriptorType::ENDPOINT),
              Endpoint::EPNumberToAddr(iso_in_num_, Endpoint::Direction::IN),
              0x05U,
              t.packet,
              t.interval,
              0U,
              0U};
  d.iso_ep_cs = {sizeof(ASEndpointGeneral), CS_ENDPOINT, EP_GENERAL, 0U, 0U, 0U};
  block_speed_ = speed;
  block = {&desc_block_, sizeof(desc_block_)};
  return ErrorCode::OK;
}

template <uint8_t CHANNELS, uint8_t BITS_PER_SAMPLE>
ErrorCode UAC1MicrophoneQ<CHANNELS, BITS_PER_SAMPLE>::BindEndpoints(
    EndpointPool& pool, uint8_t first_interface, bool in_isr)
{
  // 组合层刚以当前速度写过块，那次写成功了，分帧一定可行。
  // The composition just wrote the block at the current speed and that write
  // succeeded, so the framing is feasible.
  Timing t;
  const bool feasible = TimingFor(block_speed_, t);
  ASSERT(feasible);
  UNUSED(feasible);
  const ErrorCode result = BindEndpoint(pool, iso_in_num_, IsoConfig(t),
                                        desc_block_.iso_ep.wMaxPacketSize, in_isr, ep_);
  if (result != ErrorCode::OK)
  {
    return result;
  }
  ep_->SetOnComplete(in_cb_);
  // alt 0 没有数据端点在工作，alt 1 时再配置。
  // Alt 0 has no data endpoint at work; alt 1 configures it again.
  ep_->Close(in_isr);
  ac_interface_ = first_interface;
  timing_ = t;
  streaming_ = false;
  return ErrorCode::OK;
}

template <uint8_t CHANNELS, uint8_t BITS_PER_SAMPLE>
void UAC1MicrophoneQ<CHANNELS, BITS_PER_SAMPLE>::UnbindEndpoints(EndpointPool& pool,
                                                                 bool in_isr)
{
  streaming_ = false;
  UnbindEndpoint(pool, ep_, in_isr);
}

template <uint8_t CHANNELS, uint8_t BITS_PER_SAMPLE>
ErrorCode UAC1MicrophoneQ<CHANNELS, BITS_PER_SAMPLE>::SetAltSetting(bool in_isr,
                                                                    uint8_t itf,
                                                                    uint8_t alt)
{
  if (ep_ == nullptr)
  {
    return ErrorCode::NOT_SUPPORT;
  }
  if (itf == ac_interface_)
  {
    // AC 接口只有 alt 0 / The AC interface has alt 0 only
    return alt == 0U ? ErrorCode::OK : ErrorCode::NOT_SUPPORT;
  }
  if (itf != ac_interface_ + 1U)
  {
    return ErrorCode::NOT_SUPPORT;
  }
  if (alt > 1U)
  {
    return ErrorCode::ARG_ERR;
  }
  // 先停流再 Close：Close 交付的 ABORTED 不会再引起提交。
  // Stop streaming before Close: the ABORTED completions Close delivers start nothing.
  streaming_ = false;
  ep_->Close(in_isr);
  return alt == 0U ? ErrorCode::OK : StartStreaming(in_isr);
}

template <uint8_t CHANNELS, uint8_t BITS_PER_SAMPLE>
ErrorCode UAC1MicrophoneQ<CHANNELS, BITS_PER_SAMPLE>::GetAltSetting(uint8_t itf,
                                                                    uint8_t& alt)
{
  if (ep_ == nullptr || itf != ac_interface_ + 1U)
  {
    return ErrorCode::NOT_SUPPORT;
  }
  alt = streaming_ ? 1U : 0U;
  return ErrorCode::OK;
}

template <uint8_t CHANNELS, uint8_t BITS_PER_SAMPLE>
bool UAC1MicrophoneQ<CHANNELS, BITS_PER_SAMPLE>::OwnsEndpoint(uint8_t ep_addr) const
{
  return ep_addr == Endpoint::EPNumberToAddr(iso_in_num_, Endpoint::Direction::IN);
}

template <uint8_t CHANNELS, uint8_t BITS_PER_SAMPLE>
ConstRawData UAC1MicrophoneQ<CHANNELS, BITS_PER_SAMPLE>::Reply(uint32_t value,
                                                               size_t bytes)
{
  ASSERT(bytes <= sizeof(reply_));
  for (size_t i = 0; i < bytes; ++i)
  {
    reply_[i] = static_cast<uint8_t>(value >> (8U * i));
  }
  return {reply_, bytes};
}

template <uint8_t CHANNELS, uint8_t BITS_PER_SAMPLE>
ErrorCode UAC1MicrophoneQ<CHANNELS, BITS_PER_SAMPLE>::OnControlRequest(
    bool in_isr, const SetupPacket& setup, ConstRawData& in_data)
{
  UNUSED(in_isr);
  const auto type = static_cast<RequestType>(setup.bmRequestType & REQ_TYPE_MASK);
  const auto recipient = static_cast<Recipient>(setup.bmRequestType & REQ_RECIPIENT_MASK);
  if (type != RequestType::CLASS)
  {
    return ErrorCode::NOT_SUPPORT;
  }
  const bool input = (setup.bmRequestType & REQ_DIRECTION_MASK) != 0U;
  const auto selector = static_cast<uint8_t>(setup.wValue >> 8U);

  if (recipient == Recipient::ENDPOINT)
  {
    // 端点的采样率控制：只有一个离散采样率，CUR、MIN、MAX 都是它。
    // Sampling frequency control of the endpoint: one discrete rate, which is CUR, MIN
    // and MAX alike.
    if (static_cast<uint8_t>(setup.wIndex) !=
            Endpoint::EPNumberToAddr(iso_in_num_, Endpoint::Direction::IN) ||
        selector != SAMPLING_FREQ_CONTROL)
    {
      return ErrorCode::NOT_SUPPORT;
    }
    if (setup.wLength != 3U)
    {
      return ErrorCode::ARG_ERR;
    }
    switch (setup.bRequest)
    {
      case GET_CUR:
      case GET_MIN:
      case GET_MAX:
        if (!input)
        {
          return ErrorCode::ARG_ERR;
        }
        in_data = Reply(sample_rate_, 3U);
        return ErrorCode::OK;
      case GET_RES:
        if (!input)
        {
          return ErrorCode::ARG_ERR;
        }
        in_data = Reply(1U, 3U);
        return ErrorCode::OK;
      case SET_CUR:
        // 核心收下 3 字节的数据阶段后调用 OnControlData。
        // The core receives the 3-byte data stage, then calls OnControlData.
        return input ? ErrorCode::ARG_ERR : ErrorCode::OK;
      default:
        return ErrorCode::ARG_ERR;
    }
  }

  if (recipient != Recipient::INTERFACE)
  {
    return ErrorCode::NOT_SUPPORT;
  }
  // 特性单元：wIndex 低字节是 AC 接口号，高字节是实体 ID；wValue 低字节是通道。
  // Feature unit: the low byte of wIndex is the AC interface, the high byte the entity
  // ID; the low byte of wValue is the channel.
  if (static_cast<uint8_t>(setup.wIndex) != ac_interface_ ||
      static_cast<uint8_t>(setup.wIndex >> 8U) != ID_FU)
  {
    return ErrorCode::NOT_SUPPORT;
  }
  if (static_cast<uint8_t>(setup.wValue) > CHANNELS)
  {
    return ErrorCode::ARG_ERR;
  }
  const bool set = setup.bRequest == SET_CUR || setup.bRequest == SET_RES;
  if (input == set)
  {
    return ErrorCode::ARG_ERR;
  }
  switch (selector)
  {
    case FU_MUTE:
      if (setup.wLength != 1U || (setup.bRequest != SET_CUR && setup.bRequest != GET_CUR))
      {
        return ErrorCode::ARG_ERR;
      }
      if (input)
      {
        in_data = Reply(mute_, 1U);
      }
      return ErrorCode::OK;

    case FU_VOLUME:
    {
      if (setup.wLength != 2U)
      {
        return ErrorCode::ARG_ERR;
      }
      int16_t value = 0;
      switch (setup.bRequest)
      {
        case SET_CUR:
        case SET_RES:
          return ErrorCode::OK;
        case GET_CUR:
          value = volume_;
          break;
        case GET_MIN:
          value = vol_min_;
          break;
        case GET_MAX:
          value = vol_max_;
          break;
        case GET_RES:
          value = vol_res_;
          break;
        default:
          return ErrorCode::ARG_ERR;
      }
      in_data = Reply(static_cast<uint16_t>(value), 2U);
      return ErrorCode::OK;
    }

    default:
      return ErrorCode::NOT_SUPPORT;
  }
}

template <uint8_t CHANNELS, uint8_t BITS_PER_SAMPLE>
ErrorCode UAC1MicrophoneQ<CHANNELS, BITS_PER_SAMPLE>::OnControlData(
    bool in_isr, const SetupPacket& setup, ConstRawData data)
{
  UNUSED(in_isr);
  const auto* bytes = static_cast<const uint8_t*>(data.addr_);
  const auto recipient = static_cast<Recipient>(setup.bmRequestType & REQ_RECIPIENT_MASK);
  const auto selector = static_cast<uint8_t>(setup.wValue >> 8U);
  // OnControlRequest 已查过接收者、选择子和长度，这里只分派。
  // OnControlRequest checked the recipient, selector and length; this only dispatches.
  if (recipient == Recipient::ENDPOINT && setup.bRequest == SET_CUR)
  {
    if (data.size_ != 3U)
    {
      return ErrorCode::ARG_ERR;
    }
    const uint32_t rate = static_cast<uint32_t>(bytes[0]) |
                          (static_cast<uint32_t>(bytes[1]) << 8U) |
                          (static_cast<uint32_t>(bytes[2]) << 16U);
    return rate == sample_rate_ ? ErrorCode::OK : ErrorCode::ARG_ERR;
  }
  if (recipient != Recipient::INTERFACE)
  {
    return ErrorCode::NOT_SUPPORT;
  }
  if (selector == FU_MUTE && setup.bRequest == SET_CUR)
  {
    if (data.size_ != 1U)
    {
      return ErrorCode::ARG_ERR;
    }
    mute_ = bytes[0];
    return ErrorCode::OK;
  }
  if (selector == FU_VOLUME && (setup.bRequest == SET_CUR || setup.bRequest == SET_RES))
  {
    if (data.size_ != 2U)
    {
      return ErrorCode::ARG_ERR;
    }
    if (setup.bRequest == SET_CUR)
    {
      volume_ = static_cast<int16_t>(static_cast<uint16_t>(bytes[0]) |
                                     static_cast<uint16_t>(bytes[1] << 8U));
    }
    return ErrorCode::OK;
  }
  return ErrorCode::NOT_SUPPORT;
}

template <uint8_t CHANNELS, uint8_t BITS_PER_SAMPLE>
void UAC1MicrophoneQ<CHANNELS, BITS_PER_SAMPLE>::OnStartOfFrame(bool in_isr,
                                                                uint16_t frame,
                                                                uint8_t microframe)
{
  if (!streaming_)
  {
    return;
  }
  if (timing_.microframes != 0U)
  {
    // 差值为 0 或不知道时按一个单位算。
    // A zero or unknown difference counts as one unit.
    const uint32_t elapsed = Elapsed(frame, microframe);
    const uint32_t units = elapsed == 0U ? 1U : elapsed;
    micro_debt_ += (microframe == WHOLE_FRAME) ? units * 8U : units;
    if (micro_debt_ < timing_.microframes)
    {
      return;
    }
    micro_debt_ = (micro_debt_ >= 2U * timing_.microframes)
                      ? 0U
                      : micro_debt_ - timing_.microframes;
  }
  KickOneFrame(in_isr);
}

template <uint8_t CHANNELS, uint8_t BITS_PER_SAMPLE>
uint32_t UAC1MicrophoneQ<CHANNELS, BITS_PER_SAMPLE>::Elapsed(uint16_t frame,
                                                             uint8_t microframe)
{
  uint32_t elapsed = 0U;
  const bool known = last_frame_ != NO_FRAME && frame != NO_FRAME &&
                     (last_microframe_ == WHOLE_FRAME) == (microframe == WHOLE_FRAME);
  if (known)
  {
    // 帧号 11 位，回绕取模。
    // Frame numbers are 11 bits and wrap around.
    if (microframe == WHOLE_FRAME)
    {
      elapsed = (static_cast<uint32_t>(frame) - last_frame_) & 0x7ffU;
    }
    else
    {
      const uint32_t now = static_cast<uint32_t>(frame & 0x7ffU) * 8U + microframe;
      const uint32_t before =
          static_cast<uint32_t>(last_frame_ & 0x7ffU) * 8U + last_microframe_;
      elapsed = (now - before) & 0x3fffU;
    }
  }
  last_frame_ = frame;
  last_microframe_ = microframe;
  return elapsed;
}

template <uint8_t CHANNELS, uint8_t BITS_PER_SAMPLE>
ErrorCode UAC1MicrophoneQ<CHANNELS, BITS_PER_SAMPLE>::StartStreaming(bool in_isr)
{
  const ErrorCode result =
      ConfigureEndpoint(*ep_, IsoConfig(timing_), timing_.packet, in_isr);
  if (result == ErrorCode::CHECK_ERR)
  {
    ep_->Close(in_isr);
  }
  if (result != ErrorCode::OK)
  {
    return result;
  }
  acc_rem_ = 0U;
  micro_debt_ = 0U;
  last_frame_ = NO_FRAME;
  DropStalePcm();
  streaming_ = true;
  KickOneFrame(in_isr);
  return ErrorCode::OK;
}

template <uint8_t CHANNELS, uint8_t BITS_PER_SAMPLE>
void UAC1MicrophoneQ<CHANNELS, BITS_PER_SAMPLE>::DropStalePcm()
{
  uint8_t scratch[64];
  size_t dropped = 0U;
  for (size_t left = pcm_queue_.Size(); left != 0U;)
  {
    const size_t n = LibXR::min(left, sizeof(scratch));
    if (pcm_queue_.PopBatch(scratch, n) != ErrorCode::OK)
    {
      break;
    }
    dropped += n;
    left -= n;
  }
  if constexpr (FRAME_BYTES > 1U)
  {
    // 保持跨流边界累计丢弃字节对齐
    // FRAME_BYTES，防止生产者分片写入半帧导致后续采样/声道错位。 Keep discarded bytes
    // aligned to FRAME_BYTES across stream starts so a split sample does not misalign
    // subsequent audio frames or channels.
    const size_t phase = (FRAME_BYTES - discard_left_) % FRAME_BYTES;
    const size_t rem = (phase + dropped) % FRAME_BYTES;
    discard_left_ = (rem == 0U) ? 0U : (FRAME_BYTES - rem);
  }
}

template <uint8_t CHANNELS, uint8_t BITS_PER_SAMPLE>
void UAC1MicrophoneQ<CHANNELS, BITS_PER_SAMPLE>::KickOneFrame(bool in_isr)
{
  // 先推进余数：丢掉的区间也占实时，长期速率跟着真实时间走。
  // Advance the remainder first: a dropped interval still takes real time, so the
  // long-term rate follows the wall clock.
  size_t frames = timing_.base_frames;
  acc_rem_ += timing_.rem_frames;
  if (acc_rem_ >= timing_.denominator)
  {
    ++frames;
    acc_rem_ -= timing_.denominator;
  }
  const size_t amount = frames * FRAME_BYTES;

  Endpoint::Slot slot;
  if (ep_->AcquireSlot(slot) != ErrorCode::OK)
  {
    // 没有空闲槽：这个区间丢弃。
    // No free slot: this interval is dropped.
    frame_drops_.fetch_add(1U, std::memory_order_relaxed);
    return;
  }
  ASSERT(amount <= timing_.packet && slot.data.size_ >= amount);
  auto* out = static_cast<uint8_t*>(slot.data.addr_);
  if constexpr (FRAME_BYTES > 1U)
  {
    if (discard_left_ != 0U)
    {
      const size_t skip = LibXR::min(pcm_queue_.Size(), discard_left_);
      if (skip != 0U)
      {
        uint8_t discard[FRAME_BYTES];
        if (pcm_queue_.PopBatch(discard, skip) == ErrorCode::OK)
        {
          discard_left_ -= skip;
        }
      }
    }
  }
  size_t available = 0U;
  if (discard_left_ == 0U)
  {
    available = LibXR::min(pcm_queue_.Size(), amount);
    available -= available % FRAME_BYTES;
    if (available != 0U && pcm_queue_.PopBatch(out, available) != ErrorCode::OK)
    {
      // 单一消费者时不会发生；按整包补零处理。
      // Cannot happen with a single consumer; treated as a packet of zeros.
      available = 0U;
    }
  }
  if (available < amount)
  {
    underruns_.fetch_add(1U, std::memory_order_relaxed);
    LibXR::Memory::FastSet(out + available, 0, amount - available);
  }
  // 等时传输没有结尾零长度包；ABORTED 时槽已空闲。
  // Isochronous transfers have no closing zero-length packet; on ABORTED the slot is
  // free.
  const ErrorCode result = ep_->Submit(slot, amount, false, in_isr);
  ASSERT(result == ErrorCode::OK || result == ErrorCode::ABORTED);
  UNUSED(result);
}

template <uint8_t CHANNELS, uint8_t BITS_PER_SAMPLE>
void UAC1MicrophoneQ<CHANNELS, BITS_PER_SAMPLE>::OnInCompleteStatic(
    bool in_isr, UAC1MicrophoneQ* self, const Endpoint::Completion& completion)
{
  UNUSED(in_isr);
  // 解绑中、OK 和 ABORTED（Close 停的）都不用做什么：下一个包由 SOF 提交。FAILED 和
  // OUT_OF_RANGE 只计数，端点仍是 IDLE，下一个区间照常提交。
  // Nothing to do while unbinding, on OK, or on ABORTED (stopped by Close): the SOF
  // submits the next packet. FAILED and OUT_OF_RANGE are only counted; the endpoint
  // stays IDLE and the next interval submits as usual.
  if (self->ep_ == nullptr || completion.result == ErrorCode::OK ||
      completion.result == ErrorCode::ABORTED)
  {
    return;
  }
  self->transport_misses_.fetch_add(1U, std::memory_order_relaxed);
}

}  // namespace LibXR::USB
