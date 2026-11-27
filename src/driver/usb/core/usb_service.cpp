#include "usb_service.hpp"

#include <bit>

namespace LibXR::USB
{

void DeviceService::OnControllerInterrupt()
{
  // 先屏蔽再发布：owner 看到硬件事件时屏蔽已经生效，它的开线一定在屏蔽之后。
  // Mask before publishing: once the owner sees the hardware event the mask is in
  // effect, so its unmask always comes after it.
  MaskInterrupt();
  service_.Invoke(HARDWARE_EVENT, true,
                  [this](uint32_t events, bool in_isr) { Serve(events, in_isr); });
}

void DeviceService::Request(uint32_t events, bool in_isr)
{
  ASSERT(events != 0U && (events & ~DEVICE_EVENTS) == 0U);
  Enter(events, in_isr);
}

ErrorCode DeviceService::Register(Endpoint& ep)
{
  const uint32_t number = Endpoint::EPNumberToInt8(ep.GetNumber());
  const bool in = ep.AvailableDirection() == Endpoint::Direction::IN;
  const uint32_t index = 2U * number + (in ? 1U : 0U);
  if (endpoints_[index] != nullptr)
  {
    return ErrorCode::FULL;
  }
  endpoints_[index] = &ep;
  ep.pool_ = this;
  ep.pending_bit_ = 1U << index;
  return ErrorCode::OK;
}

void DeviceService::SetStartGate(StartGate gate)
{
  gate_ = gate;
  // 每个已登记端点都在本轮之后重新检查队列；闸门不允许的端点在 StartNext 里什么也不做。
  // Every registered endpoint re-checks its queue after this pass; one the gate does
  // not allow does nothing in StartNext.
  uint32_t registered = 0U;
  for (uint32_t i = 0U; i < ENDPOINT_BITS; ++i)
  {
    if (endpoints_[i] != nullptr)
    {
      registered |= 1U << i;
    }
  }
  endpoint_pending_.fetch_or(registered, std::memory_order_release);
  service_.Publish(ENDPOINT_EVENT);
}

void DeviceService::CloseAllEndpoints(bool in_isr)
{
  for (Endpoint* ep : endpoints_)
  {
    if (ep != nullptr)
    {
      ep->CloseAfterReset(in_isr);
    }
  }
}

bool DeviceService::AnyStopPending() const
{
  for (const Endpoint* ep : endpoints_)
  {
    if (ep != nullptr && ep->stop_ != Endpoint::Stop::NONE)
    {
      return true;
    }
  }
  return false;
}

void DeviceService::RequestPoll(bool in_isr)
{
  // 只发布、只挂起：节拍上下文永远不成为 owner。没有 pend 时只允许线程调用，它就地
  // 成为 owner（ESP32 的定时器服务任务）。
  // Publish and pend only: the tick context never becomes the owner. Without pend only
  // a thread may call this, and it owns in place (ESP32's timer service task).
  service_.Publish(POLL_EVENT);
  if (PendInterrupt())
  {
    return;
  }
  ASSERT(!in_isr);
  service_.Invoke(POLL_EVENT, in_isr,
                  [this](uint32_t taken, bool owner_isr) { Serve(taken, owner_isr); });
}

void DeviceService::Enter(uint32_t events, bool in_isr)
{
  service_.Publish(events);
  if (PendInterrupt())
  {
    return;
  }
  service_.Invoke(events, in_isr,
                  [this](uint32_t taken, bool owner_isr) { Serve(taken, owner_isr); });
}

void DeviceService::Serve(uint32_t events, bool in_isr)
{
  if ((events & (HARDWARE_EVENT | POLL_EVENT)) != 0U)
  {
    ServiceHardware(in_isr);
    if ((events & HARDWARE_EVENT) != 0U)
    {
      UnmaskInterrupt();
    }
  }
  const uint32_t device =
      events & (DEVICE_EVENTS | STOP_FAILED_EVENT | STOPS_ENDED_EVENT);
  if (device != 0U)
  {
    device_handler_.Run(in_isr, device);
  }
  if ((events & ENDPOINT_EVENT) == 0U)
  {
    return;
  }
  uint32_t bits = endpoint_pending_.exchange(0U, std::memory_order_acq_rel);
  while (bits != 0U)
  {
    const uint32_t index = static_cast<uint32_t>(std::countr_zero(bits));
    bits &= bits - 1U;
    endpoints_[index]->StartNext(in_isr);
  }
}

void DeviceService::RequestEndpoint(uint32_t bit, bool in_isr)
{
  // 先置位再发布：owner 看到 ENDPOINT_EVENT 时一定看到这一位。
  // Set before publishing: an owner that sees ENDPOINT_EVENT sees this bit.
  endpoint_pending_.fetch_or(bit, std::memory_order_release);
  Enter(ENDPOINT_EVENT, in_isr);
}

}  // namespace LibXR::USB
