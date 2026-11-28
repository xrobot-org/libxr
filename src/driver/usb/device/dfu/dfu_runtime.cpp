#include "dfu_runtime.hpp"

#include "timebase.hpp"

using namespace LibXR::USB;
using LibXR::ConstRawData;
using LibXR::ErrorCode;
using LibXR::Timebase;
using LibXR::USB::SetupPacket;

namespace
{
/// 运行时接口协议 / Runtime interface protocol
constexpr uint8_t PROTOCOL_RUNTIME = 0x01U;

uint32_t NowMs() { return static_cast<uint32_t>(Timebase::GetMilliseconds()); }
}  // namespace

DfuRuntimeClass::DfuRuntimeClass(JumpFun jump_to_bootloader, void* jump_ctx,
                                 uint16_t detach_timeout_ms, const char* interface_string,
                                 const char* webusb_landing_page_url,
                                 uint8_t webusb_vendor_code)
    : DfuInterfaceClassBase(interface_string, PROTOCOL_RUNTIME, webusb_landing_page_url,
                            webusb_vendor_code, DEFAULT_WINUSB_DEVICE_INTERFACE_GUID),
      jump_to_bootloader_(jump_to_bootloader),
      jump_ctx_(jump_ctx),
      default_detach_timeout_ms_(detach_timeout_ms)
{
  const bool can_detach = jump_to_bootloader_ != nullptr;
  functional_.bmAttributes = can_detach ? ATTR_WILL_DETACH : 0U;
  functional_.wDetachTimeOut = can_detach ? detach_timeout_ms : 0U;
  functional_.wTransferSize = 0U;
}

void DfuRuntimeClass::Process()
{
  const uint32_t token = launch_phase_.load(std::memory_order_acquire);
  if (token == IDLE)
  {
    return;
  }
  const uint32_t deadline = detach_deadline_ms_.load(std::memory_order_acquire);
  if (static_cast<int32_t>(NowMs() - deadline) < 0)
  {
    return;
  }
  // 只替换读到的那一代令牌：若期间取消并重新发来 DETACH，新令牌不同，不会误跳。
  // Swap only the generation token read above: a cancel and re-armed DETACH carries a
  // new token and will not jump early.
  uint32_t expected = token;
  if (!launch_phase_.compare_exchange_strong(expected, IDLE, std::memory_order_acq_rel))
  {
    return;
  }
  jump_to_bootloader_(jump_ctx_);
}

ErrorCode DfuRuntimeClass::BindEndpoints(EndpointPool& pool, uint8_t first_interface,
                                         bool in_isr)
{
  UNUSED(pool);
  UNUSED(in_isr);
  interface_ = first_interface;
  state_ = DFUState::APP_IDLE;
  launch_phase_.store(IDLE, std::memory_order_release);
  return ErrorCode::OK;
}

void DfuRuntimeClass::UnbindEndpoints(EndpointPool& pool, bool in_isr)
{
  UNUSED(pool);
  UNUSED(in_isr);
  state_ = DFUState::APP_IDLE;
  launch_phase_.store(IDLE, std::memory_order_release);
}

ErrorCode DfuRuntimeClass::OnControlRequest(bool in_isr, const SetupPacket& setup,
                                            ConstRawData& in_data)
{
  UNUSED(in_isr);
  const ErrorCode check = CheckClassRequest(setup);
  if (check != ErrorCode::OK)
  {
    return check;
  }
  switch (static_cast<DFURequest>(setup.bRequest))
  {
    case DFURequest::DETACH:
      if (jump_to_bootloader_ == nullptr)
      {
        return ErrorCode::NOT_SUPPORT;
      }
      if (state_ != DFUState::APP_IDLE)
      {
        return ErrorCode::ARG_ERR;
      }
      // 计时从状态阶段完成开始（OnControlComplete），没有数据阶段。
      // The timer starts when the status stage completes (OnControlComplete); there is
      // no data stage.
      state_ = DFUState::APP_DETACH;
      return ErrorCode::OK;

    case DFURequest::GETSTATUS:
    {
      // 主机只能从这里看到剩下的等待时间。APP_DETACH 只在 DETACH 完成之后才可能被
      // GETSTATUS 看到（新的 SETUP 会中止 DETACH），所以截止时刻已写好。
      // The host sees the remaining wait only here. APP_DETACH can be seen by a
      // GETSTATUS only after DETACH completed (a new SETUP aborts DETACH), so the
      // deadline is already written.
      uint32_t remain = 0U;
      if (state_ == DFUState::APP_DETACH)
      {
        const auto left = static_cast<int32_t>(
            detach_deadline_ms_.load(std::memory_order_relaxed) - NowMs());
        remain = left > 0 ? static_cast<uint32_t>(left) : 0U;
      }
      status_response_.bStatus = static_cast<uint8_t>(DFUStatusCode::OK);
      status_response_.SetPollTimeout(remain);
      status_response_.bState = static_cast<uint8_t>(state_);
      status_response_.iString = 0U;
      in_data = {&status_response_, sizeof(status_response_)};
      return ErrorCode::OK;
    }

    case DFURequest::GETSTATE:
      state_response_ = static_cast<uint8_t>(state_);
      in_data = {&state_response_, sizeof(state_response_)};
      return ErrorCode::OK;

    default:
      return ErrorCode::ARG_ERR;
  }
}

void DfuRuntimeClass::OnControlComplete(bool in_isr, const SetupPacket& setup)
{
  UNUSED(in_isr);
  if (setup.bRequest != static_cast<uint8_t>(DFURequest::DETACH))
  {
    return;
  }
  // wValue 是主机要的超时，0 时用默认值。
  // wValue is the timeout the host asks for; 0 means the default.
  const uint16_t timeout =
      (setup.wValue == 0U) ? default_detach_timeout_ms_ : setup.wValue;
  next_token_ += 2U;
  detach_deadline_ms_.store(NowMs() + timeout, std::memory_order_release);
  launch_phase_.store(next_token_, std::memory_order_release);
}

void DfuRuntimeClass::OnControlAbort(bool in_isr, const SetupPacket& setup)
{
  UNUSED(in_isr);
  if (setup.bRequest != static_cast<uint8_t>(DFURequest::DETACH))
  {
    return;
  }
  state_ = DFUState::APP_IDLE;
  launch_phase_.store(IDLE, std::memory_order_release);
}
