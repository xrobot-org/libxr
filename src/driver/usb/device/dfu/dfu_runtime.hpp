#pragma once

#include <atomic>

#include "dfu/dfu_def.hpp"
#include "timebase.hpp"

namespace LibXR::USB
{
/**
 * @brief Runtime DFU 类：只负责 DETACH 后跳转 bootloader
 *        Runtime DFU class: only handles DETACH and later jumps to bootloader.
 */
class DfuRuntimeClass : public DfuInterfaceClassBase
{
 public:
  /// 跳转到 bootloader 的回调，参数为 jump_ctx / Callback that jumps to the bootloader;
  /// its argument is jump_ctx
  using JumpCallback = void (*)(void*);
  /// 默认接口字符串 / Default interface string
  static constexpr const char* DEFAULT_INTERFACE_STRING = "XRUSB DFU RT";
  /// bmAttributes 中的 bitWillDetach：设备收到 DETACH 后自行分离 / bitWillDetach in
  /// bmAttributes: the device detaches by itself after DETACH
  static constexpr uint8_t ATTR_WILL_DETACH = 0x08u;

  /**
   * @brief 构造 Runtime DFU 类
   *        Construct the runtime DFU class.
   *
   * @param jump_to_bootloader 跳转 bootloader 的回调；nullptr 时描述符不声明
   * bitWillDetach / Bootloader jump callback; with nullptr the descriptor does not set
   * bitWillDetach
   * @param jump_ctx 传给回调的参数 / Argument passed to the callback
   * @param detach_timeout_ms DETACH 到跳转之间的等待时间（毫秒）/ Delay between DETACH
   * and the jump, in milliseconds
   * @param interface_string 接口字符串 / Interface string
   * @param webusb_landing_page_url WebUSB 落地页 URL；nullptr 表示不启用 / WebUSB landing
   * page URL; nullptr disables WebUSB
   * @param webusb_vendor_code WebUSB 厂商请求码 / WebUSB vendor request code
   */
  explicit DfuRuntimeClass(
      JumpCallback jump_to_bootloader, void* jump_ctx = nullptr,
      uint16_t detach_timeout_ms = 50u,
      const char* interface_string = DEFAULT_INTERFACE_STRING,
      const char* webusb_landing_page_url = nullptr,
      uint8_t webusb_vendor_code = LibXR::USB::WebUsb::WEBUSB_VENDOR_CODE_DEFAULT)
      : DfuInterfaceClassBase(interface_string, webusb_landing_page_url,
                              webusb_vendor_code, DEFAULT_WINUSB_DEVICE_INTERFACE_GUID),
        jump_to_bootloader_(jump_to_bootloader),
        jump_ctx_(jump_ctx),
        default_detach_timeout_ms_(detach_timeout_ms)
  {
  }

  /**
   * @brief 应用主循环周期调用：DETACH 超时到期后跳到 bootloader /
   *        Called periodically by the application loop: jump to the bootloader once the
   *        DETACH timeout has expired
   *
   * Runtime DFU 只有这一个延迟动作。超时从 DETACH 的状态阶段完成时开始计；跳转在
   * 调用本函数的上下文里执行，不在 USB 中断里。
   * Runtime DFU has only this one deferred action. The timeout starts when the DETACH
   * status stage completes; the jump runs in the context that calls this function,
   * not in the USB interrupt.
   */
  void Process()
  {
    if (launch_phase_.load(std::memory_order_acquire) != WAITING) return;
    if (static_cast<int32_t>(static_cast<uint32_t>(Timebase::GetMilliseconds()) -
                             detach_deadline_ms_) < 0)
      return;
    uint32_t expected = WAITING;
    if (!launch_phase_.compare_exchange_strong(expected, IDLE, std::memory_order_acq_rel))
      return;
    if (jump_to_bootloader_ != nullptr) jump_to_bootloader_(jump_ctx_);
  }

 protected:
  LIBXR_PACKED_BEGIN
  /**
   * @brief DFU Functional Descriptor（Runtime 变体）
   *        DFU Functional Descriptor for the runtime variant.
   */
  struct FunctionalDescriptor
  {
    uint8_t bLength = 9;             ///< 描述符长度 / Descriptor length
    uint8_t bDescriptorType = 0x21;  ///< DFU 功能描述符类型 / DFU functional descriptor
                                     ///< type
    uint8_t bmAttributes = 0u;       ///< 能力位图 / Capability bitmap
    uint16_t wDetachTimeOut = 0;     ///< DETACH 后等待复位的毫秒数 / Milliseconds to wait
                                     ///< for reset after DETACH
    uint16_t wTransferSize = 0;  ///< 最大传输字节数，Runtime 下为 0 / Maximum transfer
                                 ///< bytes; 0 in runtime mode
    uint16_t bcdDFUVersion = 0x0110u;  ///< DFU 版本 1.1 / DFU version 1.1
  };

  /**
   * @brief GETSTATUS 返回包 / GETSTATUS response payload
   */
  struct StatusResponse
  {
    uint8_t bStatus = static_cast<uint8_t>(DFUStatusCode::OK);  ///< 状态码 / Status code
    uint8_t bwPollTimeout[3] = {0, 0, 0};  ///< 轮询超时（毫秒，24 位小端）/ Poll timeout
                                           ///< in milliseconds, 24-bit little-endian
    uint8_t bState = static_cast<uint8_t>(DFUState::APP_IDLE);  ///< 当前 DFU 状态 /
                                                                ///< Current DFU state
    uint8_t iString = 0;  ///< 字符串索引，0 为无 / String index, 0 = none

    /// 把毫秒值的低 24 位写入 bwPollTimeout / Store the low 24 bits of a millisecond
    /// value into bwPollTimeout
    void SetPollTimeout(uint32_t timeout_ms)
    {
      bwPollTimeout[0] = static_cast<uint8_t>(timeout_ms & 0xFFu);
      bwPollTimeout[1] = static_cast<uint8_t>((timeout_ms >> 8) & 0xFFu);
      bwPollTimeout[2] = static_cast<uint8_t>((timeout_ms >> 16) & 0xFFu);
    }
  };

  /**
   * @brief Runtime DFU 的接口描述符块
   *        Runtime DFU descriptor block.
   */
  struct DescriptorBlock
  {
    ConfigDescriptorItem::InterfaceDescriptor interface_desc = {
        9, static_cast<uint8_t>(DescriptorType::INTERFACE), 0, 0, 0, 0xFEu, 0x01u, 0x01u,
        0};  ///< DFU Runtime 接口描述符（类 0xFE、子类 1、协议 1）/ DFU runtime interface
             ///< descriptor (class 0xFE, subclass 1, protocol 1)
    FunctionalDescriptor func_desc = {};  ///< DFU 功能描述符 / DFU functional descriptor
  };
  LIBXR_PACKED_END

  void BindEndpoints(EndpointPool&, uint8_t start_itf_num, bool) override
  {
    // Runtime DFU 没有数据端点；绑定阶段只发布接口/功能描述符，并重置 detach 状态机。
    // Runtime DFU has no data endpoints; bind only publishes the interface /
    // functional descriptors and resets the detach state machine.
    interface_num_ = start_itf_num;
    current_alt_setting_ = 0u;
    detach_pending_ = false;
    launch_phase_.store(IDLE, std::memory_order_release);
    detach_timeout_ms_ = default_detach_timeout_ms_;
    state_ = DFUState::APP_IDLE;
    desc_block_.interface_desc.bInterfaceNumber = interface_num_;
    desc_block_.interface_desc.iInterface = GetInterfaceStringIndex(0u);
    desc_block_.func_desc.bmAttributes =
        (jump_to_bootloader_ != nullptr) ? ATTR_WILL_DETACH : 0u;
    desc_block_.func_desc.wDetachTimeOut =
        (jump_to_bootloader_ != nullptr) ? detach_timeout_ms_ : 0u;
    desc_block_.func_desc.wTransferSize = 0u;
    SetData(RawData{reinterpret_cast<uint8_t*>(&desc_block_), sizeof(desc_block_)});
    inited_ = true;
  }

  void UnbindEndpoints(EndpointPool&, bool) override
  {
    // 解绑阶段只清理 runtime detach 状态；这里不持有额外 backend 资源。
    // Unbind only clears runtime detach state; no backend-owned resources live here.
    detach_pending_ = false;
    launch_phase_.store(IDLE, std::memory_order_release);
    state_ = DFUState::APP_IDLE;
    inited_ = false;
  }

  size_t GetInterfaceCount() override { return 1u; }
  bool HasIAD() override { return false; }
  size_t GetMaxConfigSize() override { return sizeof(desc_block_); }

  ErrorCode SetAltSetting(uint8_t itf, uint8_t alt) override
  {
    if (itf != interface_num_)
    {
      return ErrorCode::NOT_FOUND;
    }
    if (alt != 0u)
    {
      return ErrorCode::NOT_SUPPORT;
    }
    current_alt_setting_ = alt;
    return ErrorCode::OK;
  }

  ErrorCode GetAltSetting(uint8_t itf, uint8_t& alt) override
  {
    if (itf != interface_num_)
    {
      return ErrorCode::NOT_FOUND;
    }
    alt = current_alt_setting_;
    return ErrorCode::OK;
  }

  ErrorCode OnGetDescriptor(bool, uint8_t, uint16_t wValue, uint16_t,
                            ConstRawData& out_data) override
  {
    const uint8_t desc_type = static_cast<uint8_t>((wValue >> 8) & 0xFFu);
    if (desc_type != desc_block_.func_desc.bDescriptorType)
    {
      return ErrorCode::NOT_SUPPORT;
    }

    out_data = {reinterpret_cast<const uint8_t*>(&desc_block_.func_desc),
                sizeof(desc_block_.func_desc)};
    return ErrorCode::OK;
  }

  /// DETACH 的状态阶段完成后启动超时计时，随后由 Process() 在到期时跳转 / After the
  /// DETACH status stage completes, start the timeout; Process() jumps once it expires
  void OnControlComplete(bool, const SetupPacket& setup) override
  {
    if ((setup.bmRequestType & REQ_TYPE_MASK) ==
            static_cast<uint8_t>(RequestType::CLASS) &&
        setup.bRequest == static_cast<uint8_t>(DFURequest::DETACH))
    {
      detach_deadline_ms_ =
          static_cast<uint32_t>(Timebase::GetMilliseconds()) + detach_timeout_ms_;
      launch_phase_.store(WAITING, std::memory_order_release);
    }
  }
  /// DETACH 被中止时取消待执行的跳转并回到 APP_IDLE / When DETACH is aborted, cancel
  /// the pending jump and return to APP_IDLE
  void OnControlAbort(bool, const SetupPacket& setup) override
  {
    if (setup.bRequest == static_cast<uint8_t>(DFURequest::DETACH))
    {
      launch_phase_.store(IDLE, std::memory_order_release);
      detach_pending_ = false;
      state_ = DFUState::APP_IDLE;
    }
  }

  ErrorCode OnClassRequest(bool, uint8_t bRequest, uint16_t wValue, uint16_t wLength,
                           uint16_t wIndex, ControlTransferResult& result) override
  {
    if (!inited_)
    {
      return ErrorCode::INIT_ERR;
    }
    if ((wIndex & 0xFFu) != interface_num_)
    {
      return ErrorCode::NOT_FOUND;
    }

    switch (static_cast<DFURequest>(bRequest))
    {
      case DFURequest::DETACH:
        // Runtime DFU 只在 APP_IDLE 接受 DETACH；真正跳转由 Process() 在
        // 广播给主机的超时到期后再执行。
        // Runtime DFU only accepts DETACH from APP_IDLE; the actual jump is
        // deferred to Process() after the advertised timeout expires.
        if (jump_to_bootloader_ == nullptr)
        {
          return ErrorCode::NOT_SUPPORT;
        }
        if (wLength != 0u || state_ != DFUState::APP_IDLE)
        {
          return ErrorCode::ARG_ERR;
        }
        detach_timeout_ms_ = (wValue == 0u) ? default_detach_timeout_ms_ : wValue;
        detach_pending_ = true;  // Readiness starts at actual control completion.
        state_ = DFUState::APP_DETACH;
        desc_block_.func_desc.wDetachTimeOut = detach_timeout_ms_;
        // 无数据阶段的控制传输由核心自行完成状态阶段（STATUS ZLP）。
        // A control transfer without a data stage finishes its status stage in the core.
        return ErrorCode::OK;

      case DFURequest::GETSTATUS:
        // GETSTATUS 是主机读取剩余 detach 超时的唯一入口。
        // GETSTATUS is the only point where the host observes the remaining
        // detach timeout.
        status_response_.bStatus = static_cast<uint8_t>(DFUStatusCode::OK);
        status_response_.bState = static_cast<uint8_t>(state_);
        if (detach_pending_)
        {
          const uint32_t now_ms =
              static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds());
          const uint32_t remain_ms =
              (static_cast<int32_t>(detach_deadline_ms_ - now_ms) > 0)
                  ? (detach_deadline_ms_ - now_ms)
                  : 0u;
          status_response_.SetPollTimeout(remain_ms);
        }
        else
        {
          status_response_.SetPollTimeout(0u);
        }
        result.InData() = {reinterpret_cast<const uint8_t*>(&status_response_),
                           sizeof(status_response_)};
        return ErrorCode::OK;

      case DFURequest::GETSTATE:
        state_response_ = static_cast<uint8_t>(state_);
        result.InData() = {&state_response_, sizeof(state_response_)};
        return ErrorCode::OK;

      default:
        return ErrorCode::ARG_ERR;
    }
  }

 private:
  static constexpr uint32_t IDLE = 0U, WAITING = 1U;
  // 由 owner 写（DETACH 完成置 WAITING、复位/中止置 IDLE），Process() 读并消费；
  // detach_deadline_ms_ 在 WAITING 的 release 存之前写好。
  // Written by the owner (WAITING when DETACH completes, IDLE on reset or abort), read
  // and consumed by Process(); detach_deadline_ms_ is written before the release store
  // of WAITING.
  std::atomic<uint32_t> launch_phase_{IDLE};
  DescriptorBlock desc_block_ = {};            ///< 描述符缓存 / Descriptor cache
  StatusResponse status_response_ = {};        ///< GETSTATUS 缓冲区 / GETSTATUS buffer
  JumpCallback jump_to_bootloader_ = nullptr;  ///< 跳 boot 回调 / Boot jump callback
  void* jump_ctx_ = nullptr;                   ///< 跳转上下文 / Jump callback context
  uint8_t state_response_ = 0u;  ///< GETSTATE 缓冲字节 / GETSTATE byte buffer
  bool detach_pending_ = false;  ///< 是否等待 detach 超时 / Waiting for detach timeout
  uint16_t default_detach_timeout_ms_ =
      50u;                               ///< 默认 detach 超时 / Default detach timeout
  uint16_t detach_timeout_ms_ = 50u;     ///< 当前 detach 超时 / Active detach timeout
  uint32_t detach_deadline_ms_ = 0u;     ///< detach 截止时刻 / Detach deadline tick
  DFUState state_ = DFUState::APP_IDLE;  ///< Runtime DFU 状态 / Runtime DFU state
};

}  // namespace LibXR::USB
