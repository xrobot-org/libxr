#include "ep_pool.hpp"

using namespace LibXR::USB;

EndpointPool::EndpointPool(size_t endpoint_num) { ASSERT(endpoint_num >= 2); }

LibXR::ErrorCode EndpointPool::Put(Endpoint* ep)
{
  if (ep == nullptr)
  {
    return LibXR::ErrorCode::ARG_ERR;
  }

  ep->Attach(*this);

  const auto ep_num = static_cast<size_t>(ep->GetNumber());
  if (ep_num >= SLOT_COUNT)
  {
    return LibXR::ErrorCode::ARG_ERR;
  }

  const Endpoint::Direction dir = ep->AvailableDirection();

  // BOTH 端点同时登记到该号的两个方向槽；其余按具体方向入位。
  // A BOTH endpoint is registered into both direction slots of its number; others go
  // into their concrete direction slot.
  if (dir == Endpoint::Direction::BOTH)
  {
    for (size_t d = 0; d < DIR_COUNT; ++d)
    {
      if (slots_[ep_num][d] != nullptr)
      {
        return LibXR::ErrorCode::FULL;
      }
    }
    for (size_t d = 0; d < DIR_COUNT; ++d)
    {
      slots_[ep_num][d] = ep;
      use_[ep_num][d] = SlotUse::AVAILABLE;
    }
    Register(ep, ep_num, 0);
    return LibXR::ErrorCode::OK;
  }

  const size_t d = DirIndex(dir);
  if (slots_[ep_num][d] != nullptr)
  {
    return LibXR::ErrorCode::FULL;
  }
  slots_[ep_num][d] = ep;
  use_[ep_num][d] = SlotUse::AVAILABLE;
  Register(ep, ep_num, d);
  return LibXR::ErrorCode::OK;
}

LibXR::ErrorCode EndpointPool::Get(Endpoint*& ep_info, Endpoint::Direction direction,
                                   Endpoint::EPNumber ep_num)
{
  const auto num = static_cast<size_t>(ep_num);
  if (num >= SLOT_COUNT || direction == Endpoint::Direction::BOTH)
  {
    return LibXR::ErrorCode::NOT_FOUND;
  }

  const size_t d = DirIndex(direction);
  Endpoint* ep = slots_[num][d];
  if (ep == nullptr || use_[num][d] != SlotUse::AVAILABLE)
  {
    return LibXR::ErrorCode::NOT_FOUND;
  }

  // 端点登记方向必须与请求方向兼容（具体方向匹配，或登记为 BOTH）。
  // The registered direction must be compatible with the requested one (exact match, or
  // registered as BOTH).
  const Endpoint::Direction avail = ep->AvailableDirection();
  if (avail != direction && avail != Endpoint::Direction::BOTH)
  {
    return LibXR::ErrorCode::NOT_FOUND;
  }

  ep_info = ep;

  // BOTH 端点占用后，两个方向槽一并标记为使用中。
  // For a BOTH endpoint, mark both direction slots as in-use together.
  if (avail == Endpoint::Direction::BOTH)
  {
    use_[num][0] = SlotUse::IN_USE;
    use_[num][1] = SlotUse::IN_USE;
  }
  else
  {
    use_[num][d] = SlotUse::IN_USE;
  }
  return LibXR::ErrorCode::OK;
}

LibXR::ErrorCode EndpointPool::Release(Endpoint* ep_info)
{
  if (ep_info == nullptr)
  {
    return LibXR::ErrorCode::NOT_FOUND;
  }

  const auto num = static_cast<size_t>(ep_info->GetNumber());
  if (num >= SLOT_COUNT)
  {
    return LibXR::ErrorCode::NOT_FOUND;
  }

  bool released = false;
  for (size_t d = 0; d < DIR_COUNT; ++d)
  {
    if (slots_[num][d] == ep_info && use_[num][d] == SlotUse::IN_USE)
    {
      use_[num][d] = SlotUse::AVAILABLE;
      released = true;
    }
  }

  // The completion callback stays until the next binding replaces it: a request that
  // is still being stopped completes to the binding that submitted it.
  return released ? LibXR::ErrorCode::OK : LibXR::ErrorCode::NOT_FOUND;
}

LibXR::ErrorCode EndpointPool::FindEndpoint(uint8_t ep_addr, Endpoint*& ans)
{
  const Endpoint::Direction direction =
      (ep_addr & 0x80) ? Endpoint::Direction::IN : Endpoint::Direction::OUT;

  if ((ep_addr & 0x7F) == 0)
  {
    ans = (direction == Endpoint::Direction::IN) ? ep0_in_ : ep0_out_;
    return (ans != nullptr) ? LibXR::ErrorCode::OK : LibXR::ErrorCode::NOT_FOUND;
  }

  const size_t num = ep_addr & 0x7F;
  if (num >= SLOT_COUNT)
  {
    return LibXR::ErrorCode::NOT_FOUND;
  }

  const size_t d = DirIndex(direction);
  Endpoint* ep = slots_[num][d];
  // 仅「已分配（in-use）」且当前配置地址/方向与请求一致的端点可被反查到。
  // 必须核对 GetAddress()/GetDirection()（当前配置方向）：
  // 一个 BOTH 端点被配置成单一方向后，不会再被另一方向的同号地址查到。
  // Only endpoints that are allocated (in-use) AND whose current configured
  // address/direction match the request are visible here.
  // GetAddress()/GetDirection() (the configured direction) are checked so that once a
  // BOTH endpoint is configured for a single direction, it is no longer reachable via the
  // opposite-direction address of the same number.
  if (ep != nullptr && use_[num][d] == SlotUse::IN_USE && ep->GetAddress() == ep_addr &&
      ep->GetDirection() == direction)
  {
    ans = ep;
    return LibXR::ErrorCode::OK;
  }

  return LibXR::ErrorCode::NOT_FOUND;
}

Endpoint* EndpointPool::GetEndpoint0Out() { return ep0_out_; }

Endpoint* EndpointPool::GetEndpoint0In() { return ep0_in_; }

void EndpointPool::SetEndpoint0(Endpoint* ep0_in, Endpoint* ep0_out)
{
  ep0_in_ = ep0_in;
  ep0_out_ = ep0_out;
  if (ep0_in_)
  {
    ep0_in_->Attach(*this);
    Register(ep0_in_, 0, 1);
  }
  if (ep0_out_)
  {
    ep0_out_->Attach(*this);
    Register(ep0_out_, 0, 0);
  }
}

void EndpointPool::PostControl(uint32_t events, bool in_isr)
{
  PublishControl(events);
  RequestService(in_isr);
}

void EndpointPool::RequestService(bool in_isr)
{
  if (interrupt_depth_.load(std::memory_order_acquire) != 0U)
  {
    irq_deferred_.store(1U, std::memory_order_release);
    service_.Publish(1U);
    return;
  }
  service_.Invoke(
      1U, in_isr,
      [this](uint32_t, bool current_isr)
      {
        if (interrupt_depth_.load(std::memory_order_acquire) != 0U)
        {
          irq_deferred_.store(1U, std::memory_order_release);
          return;
        }
        const uint32_t events = control_pending_.exchange(0U, std::memory_order_acq_rel);
        if ((events & ~ENDPOINT_EVENT) != 0U)
          control_handler_.Run(current_isr, events & ~ENDPOINT_EVENT);
        if ((events & ENDPOINT_EVENT) == 0U) return;
        uint32_t bits = endpoint_pending_.exchange(0U, std::memory_order_acq_rel);
        while (bits != 0U)
        {
          const unsigned i = static_cast<unsigned>(__builtin_ctz(bits));
          bits &= bits - 1U;
          if (by_bit_[i] != nullptr) by_bit_[i]->Service(current_isr);
        }
      });
}

void EndpointPool::Register(Endpoint* ep, size_t number, size_t dir)
{
  const size_t index = number * DIR_COUNT + dir;
  by_bit_[index] = ep;
  ep->pending_bit_ = uint32_t{1} << index;
}

void EndpointPool::KickEndpoints(bool in_isr)
{
  uint32_t bits = 0U;
  for (size_t i = 0; i < 2U * SLOT_COUNT; ++i)
    if (by_bit_[i] != nullptr) bits |= uint32_t{1} << i;
  endpoint_pending_.fetch_or(bits, std::memory_order_release);
  PostControl(ENDPOINT_EVENT, in_isr);
}

void EndpointPool::StopFinished(bool in_isr)
{
  ASSERT(stopping_count_.load(std::memory_order_relaxed) != 0U);
  if (stopping_count_.fetch_sub(1U, std::memory_order_relaxed) == 1U)
    PostControl(STOP_DONE_EVENT, in_isr);
}

void EndpointPool::SetDataEnabled(bool enabled, bool in_isr)
{
  data_enabled_ = enabled;
  if (enabled) KickEndpoints(in_isr);
}

void EndpointPool::SetSuspended(bool suspended, bool in_isr)
{
  suspended_ = suspended;
  if (!suspended) KickEndpoints(in_isr);
}

bool EndpointPool::ConfigurationValid() const
{
  for (size_t n = 1; n < SLOT_COUNT; ++n)
    for (size_t d = 0; d < DIR_COUNT; ++d)
      if (use_[n][d] == SlotUse::IN_USE && slots_[n][d] &&
          slots_[n][d]->GetState() == Endpoint::State::ERROR)
        return false;
  return true;
}
