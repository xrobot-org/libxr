/**
 * @file usb_probe_class.hpp
 * @brief 后端测试用的设备类 / Device class for the backend tests.
 *
 * 一个接口，批量 IN/OUT、中断 IN/OUT 和等时 IN 各一个端点，外加三个厂商控制请求（IN 数据
 * 阶段、OUT 数据阶段、无数据阶段）。它不做任何协议处理：端点指针交给测试直接调用端点
 * API，完成回调原样记下来，测试据此检查每个请求恰好完成一次以及长度和状态。
 *
 * One interface with a bulk IN/OUT pair, an interrupt IN/OUT pair and an isochronous IN
 * endpoint, plus three vendor control requests (IN data stage, OUT data stage, no data
 * stage). It has no protocol of its own: the endpoint pointers are handed to the test,
 * which calls the endpoint API directly, and every completion is recorded as it is, so
 * the test can check that each request completes exactly once, with its length and
 * status.
 */

#pragma once

#include <array>
#include <cstring>
#include <vector>

#include "usb/core/ep_pool.hpp"
#include "usb/device/device_class.hpp"

namespace UsbProbe
{
using namespace LibXR;
using namespace LibXR::USB;

/// 一个端点的完成记录 / Completions of one endpoint
struct Log
{
  struct Item
  {
    ErrorCode result;
    size_t actual;
    std::vector<uint8_t> data;  ///< OUT：收到的字节 / OUT: the bytes received
  };
  std::vector<Item> items;
  std::vector<Endpoint::Slot> leases;  ///< 未归还的 OUT 租约 / OUT leases not returned
  Endpoint* endpoint = nullptr;
  bool rearm = true;  ///< OUT：回调里归还租约并重新挂接收 / OUT: return and rearm inside
  /// IN：回调里再提交这么多个包（内容是序号）/ IN: submit this many more from inside
  unsigned chain = 0;
  size_t chain_length = 0;
  uint8_t chain_seed = 0;

  size_t Count(ErrorCode result) const
  {
    size_t count = 0;
    for (const auto& item : items) count += item.result == result ? 1U : 0U;
    return count;
  }
  std::vector<uint8_t> Received() const
  {
    std::vector<uint8_t> all;
    for (const auto& item : items)
      if (item.result == ErrorCode::OK)
        all.insert(all.end(), item.data.begin(), item.data.end());
    return all;
  }

  static void OnComplete(bool in_isr, Log* self, const Endpoint::Completion& done)
  {
    Item item{done.result, done.actual, {}};
    if (done.slot.Valid())
    {
      const auto* bytes = static_cast<const uint8_t*>(done.slot.data.addr_);
      item.data.assign(bytes, bytes + done.actual);
      if (self->rearm)
        (void)self->endpoint->ReturnSlot(done.slot, in_isr);
      else
        self->leases.push_back(done.slot);
    }
    self->items.push_back(std::move(item));
    if (self->chain != 0U && done.result == ErrorCode::OK && !done.slot.Valid())
    {
      --self->chain;
      Endpoint::Slot slot;
      if (self->endpoint->AcquireSlot(slot, in_isr) == ErrorCode::OK)
      {
        std::memset(slot.data.addr_, ++self->chain_seed, self->chain_length);
        (void)self->endpoint->Submit(slot, self->chain_length, false, in_isr);
      }
    }
  }
  Callback<const Endpoint::Completion&> Hook()
  {
    return Callback<const Endpoint::Completion&>::Create(&Log::OnComplete, this);
  }
};

class ProbeClass : public DeviceClass
{
 public:
  using EP = Endpoint::EPNumber;

  uint16_t bulk_packet = 64;
  uint16_t interrupt_packet = 16;
  uint16_t iso_packet = 96;
  bool wants_bus_time = false;

  Endpoint* bulk_out = nullptr;
  Endpoint* bulk_in = nullptr;
  Endpoint* int_out = nullptr;
  Endpoint* int_in = nullptr;
  Endpoint* iso_in = nullptr;
  Log bulk_out_log, bulk_in_log, int_out_log, int_in_log, iso_in_log;

  // Control requests: 0x70 IN (response_size bytes of input), 0x71 OUT (into output),
  // 0x72 without data; anything else is refused.
  std::array<uint8_t, 512> input{}, output{};
  size_t response_size = 0, received = 0;
  unsigned data_count = 0, complete_count = 0, abort_count = 0;
  unsigned requests = 0;    ///< 交给本类的控制请求数 / Control requests handed over
  uint16_t last_value = 0;  ///< 最近一个请求的 wValue / wValue of the latest request
  unsigned binds = 0, unbinds = 0, resets = 0;
  std::vector<uint8_t> resumed;  ///< ENDPOINT_RESUMED 的端点地址 / Endpoint addresses
  std::vector<BusTime> ticks;

  size_t GetControlReceiveCapacity() const override { return output.size(); }
  size_t GetInterfaceCount() override { return 1; }
  size_t GetMaxConfigSize() override { return sizeof(block_); }
  bool HasIAD() override { return false; }
  bool WantsBusTime() const override { return wants_bus_time; }
  void OnBusTime(bool, const BusTime& time) override { ticks.push_back(time); }
  void OnDeviceEvent(bool, DeviceEvent event, uint8_t endpoint) override
  {
    if (event == DeviceEvent::RESET) ++resets;
    if (event == DeviceEvent::ENDPOINT_RESUMED) resumed.push_back(endpoint);
  }
  bool OwnsEndpoint(uint8_t address) const override
  {
    const uint8_t number = address & 0x0FU;
    return bound_ && number >= 1U && number <= 3U;
  }

  void BindEndpoints(EndpointPool& pool, uint8_t number, bool in_isr) override
  {
    if (!pool.IsPlanning()) ++binds;
    Take(pool, bulk_out, bulk_out_log, Endpoint::Direction::OUT, EP::EP1,
         Endpoint::Type::BULK, bulk_packet, in_isr);
    Take(pool, bulk_in, bulk_in_log, Endpoint::Direction::IN, EP::EP1,
         Endpoint::Type::BULK, bulk_packet, in_isr);
    // 中断 OUT 每个租约恰好一个包 / One packet per lease on the interrupt OUT
    Take(pool, int_out, int_out_log, Endpoint::Direction::OUT, EP::EP2,
         Endpoint::Type::INTERRUPT, interrupt_packet, in_isr, interrupt_packet);
    Take(pool, int_in, int_in_log, Endpoint::Direction::IN, EP::EP2,
         Endpoint::Type::INTERRUPT, interrupt_packet, in_isr);
    Take(pool, iso_in, iso_in_log, Endpoint::Direction::IN, EP::EP3,
         Endpoint::Type::ISOCHRONOUS, iso_packet, in_isr);
    block_.interface = {9, 4, number, 0, 5, 0xFF, 0, 0, 0};
    block_.endpoints[0] = {7, 5, 0x01, 2, Packet(bulk_out), 0};
    block_.endpoints[1] = {7, 5, 0x81, 2, Packet(bulk_in), 0};
    block_.endpoints[2] = {7, 5, 0x02, 3, Packet(int_out), 1};
    block_.endpoints[3] = {7, 5, 0x82, 3, Packet(int_in), 1};
    block_.endpoints[4] = {7, 5, 0x83, 1, Packet(iso_in), 1};
    SetData({&block_, sizeof(block_)});
    bound_ = !pool.IsPlanning();
  }

  void UnbindEndpoints(EndpointPool& pool, bool in_isr) override
  {
    if (!pool.IsPlanning()) ++unbinds;
    bound_ = false;
    for (Endpoint** endpoint : {&bulk_out, &bulk_in, &int_out, &int_in, &iso_in})
    {
      if (*endpoint == nullptr) continue;
      (*endpoint)->Close(in_isr);
      pool.Release(*endpoint);
      *endpoint = nullptr;
    }
  }

 protected:
  ErrorCode OnControlRequest(bool, const SetupPacket& setup,
                             ControlTransferResult& result) override
  {
    ++requests;
    last_value = setup.wValue;
    if (setup.bRequest == 0x70 && (setup.bmRequestType & 0x80U) != 0U)
    {
      result.InData() = {input.data(), response_size};
      return ErrorCode::OK;
    }
    if (setup.bRequest == 0x71 && (setup.bmRequestType & 0x80U) == 0U)
      return ErrorCode::OK;
    if (setup.bRequest == 0x72 && setup.wLength == 0U) return ErrorCode::OK;
    return ErrorCode::NOT_SUPPORT;
  }
  ErrorCode OnControlData(bool, const SetupPacket& setup, ConstRawData& data) override
  {
    if ((setup.bmRequestType & 0x80U) == 0U)
    {
      ++data_count;
      received = data.size_;
      std::memcpy(output.data(), data.addr_, data.size_);
    }
    return ErrorCode::OK;
  }
  void OnControlComplete(bool, const SetupPacket&) override { ++complete_count; }
  void OnControlAbort(bool, const SetupPacket&) override { ++abort_count; }

 private:
  static void Take(EndpointPool& pool, Endpoint*& endpoint, Log& log,
                   Endpoint::Direction direction, EP number, Endpoint::Type type,
                   uint16_t packet, bool in_isr, size_t receive_size = 0)
  {
    endpoint = nullptr;
    if (pool.Get(endpoint, direction, number) != ErrorCode::OK) return;
    log.endpoint = endpoint;
    endpoint->SetOnComplete(log.Hook());
    // 包长不超过当前速度下的合法值 / No larger than what is legal at the current speed
    packet = LibXR::min(packet, endpoint->LargestPacketSize(type));
    (void)endpoint->Configure({direction, type, packet, receive_size}, in_isr);
  }
  static uint16_t Packet(const Endpoint* endpoint)
  {
    return endpoint != nullptr ? endpoint->MaxPacketSize() : 0U;
  }

  LIBXR_PACKED_BEGIN
  struct Block
  {
    InterfaceDescriptor interface;
    EndpointDescriptor endpoints[5];
  };
  LIBXR_PACKED_END
  Block block_{};
  bool bound_ = false;
};

}  // namespace UsbProbe
