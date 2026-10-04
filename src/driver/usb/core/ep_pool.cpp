#include "ep_pool.hpp"

using namespace LibXR::USB;

EndpointPool::EndpointPool(size_t endpoint_num) { ASSERT(endpoint_num >= 2); }

LibXR::ErrorCode EndpointPool::Put(Endpoint* ep)
{
  if (ep == nullptr)
  {
    return LibXR::ErrorCode::ARG_ERR;
  }

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
    return LibXR::ErrorCode::OK;
  }

  const size_t d = DirIndex(dir);
  if (slots_[ep_num][d] != nullptr)
  {
    return LibXR::ErrorCode::FULL;
  }
  slots_[ep_num][d] = ep;
  use_[ep_num][d] = SlotUse::AVAILABLE;
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
}
