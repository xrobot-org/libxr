#include "device_class.hpp"

#include "usb/core/usb_service.hpp"

using namespace LibXR::USB;
using LibXR::ErrorCode;
using LibXR::USB::DescriptorType;
using LibXR::USB::Speed;

void DeviceClass::RequestService(bool in_isr)
{
  // 先置请求再发事件位：取走事件位的 owner 一定看到这个请求。
  // Set the request before the event bit: an owner that takes the bit sees it.
  service_pending_.store(1U, std::memory_order_release);
  ASSERT(service_ != nullptr);
  service_->Request(SERVICE_EVENT, in_isr);
}

uint8_t DeviceClass::GetInterfaceStringIndex(size_t local_interface_index) const
{
  if (interface_string_base_ == 0U ||
      GetInterfaceString(local_interface_index) == nullptr)
  {
    return 0U;
  }
  // 只有带字符串的接口占号，按接口顺序连续分配。
  // Only interfaces with a string take an index, assigned in interface order.
  uint8_t index = interface_string_base_;
  for (size_t i = 0; i < local_interface_index; ++i)
  {
    if (GetInterfaceString(i) != nullptr)
    {
      ++index;
    }
  }
  return index;
}

bool DeviceClass::WriteEndpointDescriptor(const EndpointPool& pool, Speed speed,
                                          Endpoint::EPNumber number,
                                          const Endpoint::Config& cfg, uint8_t interval,
                                          EndpointDescriptor& out)
{
  const uint16_t packet = pool.PacketSizeFor(number, cfg, speed);
  const uint8_t packets = cfg.max_packets_per_interval == 1U
                              ? uint8_t{1U}
                              : pool.PacketsPerIntervalFor(number, cfg, speed);
  if (packet == 0U || packets == 0U)
  {
    return false;
  }
  out = {7U,
         static_cast<uint8_t>(DescriptorType::ENDPOINT),
         Endpoint::EPNumberToAddr(number, cfg.direction),
         static_cast<uint8_t>(cfg.type),
         Endpoint::PacketSizeField(packet, packets),
         interval};
  return true;
}

ErrorCode DeviceClass::ConfigureEndpoint(Endpoint& ep, const Endpoint::Config& cfg,
                                         uint16_t descriptor_field, bool in_isr)
{
  const ErrorCode result = ep.Configure(cfg, in_isr);
  if (result != ErrorCode::OK)
  {
    return result;
  }
  // 读回的包长和包数必须等于刚写的描述符里的值。
  // The size and count read back must equal those of the descriptor just written.
  const uint16_t field =
      Endpoint::PacketSizeField(ep.MaxPacketSize(), ep.PacketsPerInterval());
  return field == descriptor_field ? ErrorCode::OK : ErrorCode::CHECK_ERR;
}

ErrorCode DeviceClass::BindEndpoint(EndpointPool& pool, Endpoint::EPNumber number,
                                    const Endpoint::Config& cfg,
                                    uint16_t descriptor_field, bool in_isr, Endpoint*& ep)
{
  if (pool.Get(ep, cfg.direction, number) != ErrorCode::OK)
  {
    ep = nullptr;
    return ErrorCode::NOT_FOUND;
  }
  return ConfigureEndpoint(*ep, cfg, descriptor_field, in_isr);
}

void DeviceClass::UnbindEndpoint(EndpointPool& pool, Endpoint*& ep, bool in_isr)
{
  Endpoint* taken = ep;
  if (taken == nullptr)
  {
    return;
  }
  // 先置空：Close 交付的完成回调看到未绑定。
  // Clear first: the completions Close delivers see "unbound".
  ep = nullptr;
  taken->Close(in_isr);
  (void)pool.Release(taken);
}
