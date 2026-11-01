#pragma once

#include "dfu/dfu_def.hpp"
#include "timer.hpp"

namespace LibXR::USB
{
/**
 * @brief Runtime DFU 类：只负责 DETACH 后跳转 bootloader
 *        Runtime DFU class: only handles DETACH and later jumps to bootloader.
 */
class DfuRuntimeClass : public DfuInterfaceClassBase
{
 public:
  using JumpCallback = void (*)(void*);
  static constexpr const char* DEFAULT_INTERFACE_STRING = "XRUSB DFU RT";
  static constexpr uint8_t ATTR_WILL_DETACH = 0x08u;

  /**
   * @brief 构造 Runtime DFU 类
   *        Construct the runtime DFU class.
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
    // The class and its registered Timer node have initialization lifetime.
    // Timer only rings the controller doorbell; deadline/state stay owner-only.
    if (jump_to_bootloader_ != nullptr)
    {
      auto timer = Timer::CreateTask(
          +[](DfuRuntimeClass* self)
          {
            if (self->launch_phase_.load(std::memory_order_acquire) == WAITING)
              self->RequestClassService(false);
          },
          this, 1U);
      Timer::Start(timer);
      Timer::Add(timer);
    }
  }

  // Runtime DFU 只有一个延迟动作：DETACH 超时后跳到板级 bootloader 入口。
  // Runtime DFU only has one deferred action: jump to the board-specific
  // bootloader entry after the DETACH timeout expires.
  // Application policy retains the final switch. Processing/timeout readiness
  // does not depend on an external per-class loop; this is only a compatibility
  // application-triggered launch helper.
  void Process() { (void)TryConsumeBootloaderLaunch(); }

  bool ReadyToDetach() const
  {
    return launch_phase_.load(std::memory_order_acquire) == READY;
  }

  bool TryConsumeBootloaderLaunch()
  {
    uint32_t expected = READY;
    if (!launch_phase_.compare_exchange_strong(expected, IDLE, std::memory_order_acq_rel))
      return false;
    if (jump_to_bootloader_) jump_to_bootloader_(jump_ctx_);
    return true;
  }

  void OnService(bool) override
  {
    if (launch_phase_.load(std::memory_order_acquire) == WAITING &&
        static_cast<int32_t>(static_cast<uint32_t>(Timebase::GetMilliseconds()) -
                             detach_deadline_ms_) >= 0)
      launch_phase_.store(READY, std::memory_order_release);
  }

 protected:
  LIBXR_PACKED_BEGIN
  /**
   * @brief DFU Functional Descriptor（Runtime 变体）
   *        DFU Functional Descriptor for the runtime variant.
   */
  struct FunctionalDescriptor
  {
    uint8_t bLength = 9;
    uint8_t bDescriptorType = 0x21;
    uint8_t bmAttributes = 0u;
    uint16_t wDetachTimeOut = 0;
    uint16_t wTransferSize = 0;
    uint16_t bcdDFUVersion = 0x0110u;
  };

  /**
   * @brief GETSTATUS 返回包 / GETSTATUS response payload
   */
  struct StatusResponse
  {
    uint8_t bStatus = static_cast<uint8_t>(DFUStatusCode::OK);
    uint8_t bwPollTimeout[3] = {0, 0, 0};
    uint8_t bState = static_cast<uint8_t>(DFUState::APP_IDLE);
    uint8_t iString = 0;

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
        0};
    FunctionalDescriptor func_desc = {};
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
  static constexpr uint32_t IDLE = 0U, WAITING = 1U, READY = 2U;
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
