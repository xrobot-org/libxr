/**
 * @file test_gsusb.cpp
 * @brief gs_usb 类的假后端测试 / Fake-backend tests of the gs_usb class
 *
 * 用真实的设备核心、组合层、端点和 gs_usb 类；假后端与 test_cdc.cpp 的相同（假设备
 * 同时派生 DeviceService 和 DeviceCore，假端点记录每次启动，由测试决定何时完成），另加
 * 可设的 HardwareLimits。主机一侧走真实的 EP0 厂商请求和批量传输；CAN 一侧是记录发送、
 * 注入接收的桩。用例来自 gs_usb 原有的主机测试，另加绑定失败、带在途请求解绑、多包
 * 接收和多线程。
 * Uses the real device core, composition, endpoints and gs_usb class; the fake backend
 * is the one of test_cdc.cpp (the fake device derives from both DeviceService and
 * DeviceCore, fake endpoints record every start and the test decides when it
 * completes), plus settable HardwareLimits. The host side goes through real EP0 vendor
 * requests and bulk transfers; the CAN side is a stub that records sends and injects
 * receptions. The cases come from the existing gs_usb host tests, plus bind failure,
 * unbind with requests in flight, multi-packet receive and threads.
 */

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <deque>
#include <functional>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

#include "dev_core.hpp"
#include "ep.hpp"
#include "ep_pool.hpp"
#include "flash.hpp"
#include "gs_usb.hpp"
#include "lockfree_list.hpp"
#include "raw_sequential.hpp"
#include "test_assert.hpp"
#include "timer.hpp"
#include "usb_service.hpp"

namespace
{
using LibXR::ConstRawData;
using LibXR::ErrorCode;
using LibXR::RawData;
using LibXR::USB::DescriptorStrings;
using LibXR::USB::DeviceClass;
using LibXR::USB::DeviceCore;
using LibXR::USB::DeviceDescriptor;
using LibXR::USB::DeviceService;
using LibXR::USB::Endpoint;
using LibXR::USB::EndpointPool;
using LibXR::USB::GsUsbClass;
using LibXR::USB::SetupPacket;
using LibXR::USB::Speed;
using LibXR::USB::USBSpec;
using Direction = Endpoint::Direction;
using EPNumber = Endpoint::EPNumber;
using Bytes = std::vector<uint8_t>;
namespace Gs = LibXR::USB::GsUsb;

static_assert(LibXR::HW_CACHE_LINE_SIZE <= 64U, "test memory is aligned to 64 bytes");

constexpr auto LANGUAGE = DescriptorStrings::MakeLanguagePack(
    DescriptorStrings::Language::EN_US, "XRobot", "GSUSB", "0001");

/// 端点内存 / Endpoint memory
struct alignas(64) Memory
{
  uint8_t bytes[1024];
};

/// 记录启动、由测试完成的假端点 / Fake endpoint that records starts, completed by tests
class FakeEp final : public Endpoint
{
 public:
  /// 一次 StartHardware / One StartHardware
  struct Start
  {
    size_t length;
    uint8_t* data;
  };

  /// slots：槽数，0 为核心默认 / slots: slot count, 0 for the core default
  FakeEp(EPNumber number, Direction direction, RawData memory, uint8_t slots = 0U)
      : Endpoint(number, direction, memory, slots)
  {
    // 只有一个槽时硬件不直接读写槽（核心对单槽的要求）。
    // With one slot the hardware does not access the slot directly (what the core
    // requires of a single slot).
    limits.direct_memory = slots != 1U;
  }

  /// 硬件完成在途请求；只在 owner 中 / The hardware completes the in-flight request;
  /// owner only
  void Done(bool in_isr, size_t actual, ErrorCode result = ErrorCode::OK)
  {
    TEST_ASSERT(inflight);
    inflight = false;
    if (GetConfig().direction == Direction::IN && result == ErrorCode::OK)
    {
      const uint8_t* data = starts.back().data;
      transfers.emplace_back(data, data + actual);
      transfer_count.store(transfers.size(), std::memory_order_release);
    }
    OnHardwareComplete(in_isr, actual, result);
  }

  /// 主机发来一次 OUT 传输；只在 owner 中 / The host sends one OUT transfer; owner only
  void HostSends(bool in_isr, const Bytes& bytes)
  {
    TEST_ASSERT(inflight && bytes.size() <= starts.back().length);
    if (!bytes.empty())
    {
      std::memcpy(starts.back().data, bytes.data(), bytes.size());
    }
    Done(in_isr, bytes.size());
  }

  size_t LastLength() const { return starts.empty() ? SIZE_MAX : starts.back().length; }

  std::vector<Start> starts;
  std::vector<Bytes> transfers;  ///< 主机取走的 IN 传输 / IN transfers the host took
  std::atomic<size_t> transfer_count{0U};
  bool inflight = false;
  size_t configures = 0;
  size_t closes = 0;
  size_t stops = 0;
  ErrorCode configure_result = ErrorCode::OK;
  HwLimits limits{};  ///< HardwareLimits 的返回值 / What HardwareLimits returns
  /// HardwareLimits 第几次调用之后报 32 字节包 / After this many HardwareLimits calls,
  /// report 32-byte packets
  int shrink_after = -1;
  mutable int limit_calls = 0;

 protected:
  ErrorCode CheckHardware(const Config&) const override { return configure_result; }

  void ConfigureHardware(const Config&) override { ++configures; }

  void CloseHardware(bool) override { ++closes; }

  ErrorCode StartHardware(const HwRequest& request, bool) override
  {
    TEST_ASSERT(!inflight);
    starts.push_back({request.length, request.data});
    inflight = true;
    return ErrorCode::OK;
  }

  StopOutcome StopHardware(bool) override
  {
    ++stops;
    inflight = false;
    return {ErrorCode::ABORTED, 0U};
  }

  ErrorCode StallHardware(bool) override { return ErrorCode::OK; }
  ErrorCode ClearStallHardware(bool) override { return ErrorCode::OK; }

  HwLimits HardwareLimits(Direction, Type) const override
  {
    ++limit_calls;
    if (shrink_after >= 0 && limit_calls > shrink_after)
    {
      return {32U, SIZE_MAX, SIZE_MAX};
    }
    return limits;
  }
};

/// 假端点和端点池：EP1 IN 数据，EP2 OUT 数据 / Fake endpoints and the pool: EP1 IN data,
/// EP2 OUT data
struct Hw
{
  /// data_slots：数据端点的槽数，0 为核心默认 / data_slots: slot count of the data
  /// endpoints, 0 for the core default
  explicit Hw(size_t data_memory = 1024U, uint8_t data_slots = 0U)
      : ep1_in(EPNumber::EP1, Direction::IN, {mem[2].bytes, data_memory}, data_slots),
        ep2_out(EPNumber::EP2, Direction::OUT, {mem[3].bytes, data_memory}, data_slots)
  {
    pool.SetEndpoint0(&in0, &out0);
    TEST_ASSERT(pool.Put(&ep1_in) == ErrorCode::OK);
    TEST_ASSERT(pool.Put(&ep2_out) == ErrorCode::OK);
  }

  Memory mem[4];
  FakeEp in0{EPNumber::EP0, Direction::IN, {mem[0].bytes, 64U}};
  FakeEp out0{EPNumber::EP0, Direction::OUT, {mem[1].bytes, 64U}};
  FakeEp ep1_in;
  FakeEp ep2_out;
  EndpointPool pool;

  std::vector<FakeEp*> All() { return {&in0, &out0, &ep1_in, &ep2_out}; }
};

using Configs = std::initializer_list<const std::initializer_list<DeviceClass*>>;

/// 假后端设备 / Fake backend device
class FakeUsb final : public DeviceService, public DeviceCore
{
 public:
  FakeUsb(Hw& hw, const Configs& configs, Speed speed = Speed::FULL)
      : DeviceCore(*this, hw.pool, USBSpec::USB_2_0, speed,
                   DeviceDescriptor::PacketSize0::SIZE_64, 0x1209U, 0x0001U, 0x0100U,
                   {&LANGUAGE}, configs)
  {
    for (FakeEp* ep : hw.All())
    {
      TEST_ASSERT(Register(*ep) == ErrorCode::OK);
    }
  }

  /// 排一个硬件事件并进入控制器中断 / Queue one hardware event and enter the
  /// controller interrupt
  void Irq(std::function<void(bool)> event)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      events_.push_back(std::move(event));
    }
    OnControllerInterrupt();
  }

  void Setup(SetupPacket setup)
  {
    Irq([this, setup](bool isr) { OnSetup(isr, setup); });
  }
  void Reset(Speed speed = Speed::FULL)
  {
    Irq([this, speed](bool isr) { OnBusReset(isr, speed); });
  }

  void StartController(bool) override {}
  void StopController(bool) override {}

  size_t ready = 0;
  std::atomic<bool> masked{false};

 protected:
  void ServiceHardware(bool in_isr) override
  {
    while (true)
    {
      std::function<void(bool)> event;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (events_.empty())
        {
          return;
        }
        event = std::move(events_.front());
        events_.pop_front();
      }
      event(in_isr);
    }
  }

  void MaskInterrupt() override { masked.store(true, std::memory_order_release); }
  void UnmaskInterrupt() override { masked.store(false, std::memory_order_release); }
  void SetAddress(uint8_t, AddressStage) override {}
  void OnControlReady() override { ++ready; }
  void ResetController(bool) override {}

 private:
  std::mutex mutex_;
  std::deque<std::function<void(bool)>> events_;
};

// ---- 主机一侧 / The host side ----

SetupPacket Make(uint8_t type, uint8_t request, uint16_t value, uint16_t index,
                 uint16_t length)
{
  return SetupPacket{type, request, value, index, length};
}

bool Stalled(Hw& hw) { return hw.in0.IsStalled() && hw.out0.IsStalled(); }

/// 完成一个在途的 IN 请求 / Complete one in-flight IN request
void InDone(FakeUsb& dev, FakeEp& ep)
{
  TEST_ASSERT(ep.inflight);
  const size_t length = ep.LastLength();
  dev.Irq([&ep, length](bool isr) { ep.Done(isr, length); });
}

/// 完成 IN 端点上的全部请求 / Complete every request on an IN endpoint
void Drain(FakeUsb& dev, FakeEp& ep)
{
  for (size_t guard = 0; ep.inflight; ++guard)
  {
    TEST_ASSERT(guard < 100000U);
    InDone(dev, ep);
  }
}

/// 主机在 OUT 端点上发一次传输 / The host sends one transfer on an OUT endpoint
void HostOut(FakeUsb& dev, FakeEp& ep, const Bytes& bytes)
{
  TEST_ASSERT(ep.inflight);
  dev.Irq([&ep, bytes](bool isr) { ep.HostSends(isr, bytes); });
}

Bytes ControlIn(FakeUsb& dev, Hw& hw, SetupPacket setup)
{
  const size_t ready = dev.ready;
  dev.Setup(setup);
  TEST_ASSERT(!Stalled(hw));
  Bytes got;
  while (hw.in0.inflight)
  {
    const FakeEp::Start& start = hw.in0.starts.back();
    got.insert(got.end(), start.data, start.data + start.length);
    InDone(dev, hw.in0);
  }
  dev.Irq([&hw](bool isr) { hw.out0.HostSends(isr, {}); });
  TEST_ASSERT(dev.ready == ready + 1U);
  return got;
}

void ControlOut(FakeUsb& dev, Hw& hw, SetupPacket setup, const Bytes& data)
{
  const size_t ready = dev.ready;
  dev.Setup(setup);
  TEST_ASSERT(!Stalled(hw));
  TEST_ASSERT(data.size() <= 64U);
  HostOut(dev, hw.out0, data);
  TEST_ASSERT(hw.in0.inflight && hw.in0.LastLength() == 0U);
  InDone(dev, hw.in0);
  TEST_ASSERT(dev.ready == ready + 1U);
}

void ControlNone(FakeUsb& dev, Hw& hw, SetupPacket setup)
{
  const size_t ready = dev.ready;
  dev.Setup(setup);
  TEST_ASSERT(!Stalled(hw));
  TEST_ASSERT(hw.in0.inflight && hw.in0.LastLength() == 0U);
  InDone(dev, hw.in0);
  TEST_ASSERT(dev.ready == ready + 1U);
}

/// SETUP 阶段就被拒 / Refused at the SETUP stage
void Refused(FakeUsb& dev, Hw& hw, SetupPacket setup)
{
  const size_t ready = dev.ready;
  dev.Setup(setup);
  TEST_ASSERT(Stalled(hw));
  TEST_ASSERT(dev.ready == ready);
}

/// SETUP 被接受，数据阶段之后被拒 / SETUP accepted, refused after the data stage
void DataRefused(FakeUsb& dev, Hw& hw, SetupPacket setup, const Bytes& data)
{
  const size_t ready = dev.ready;
  dev.Setup(setup);
  TEST_ASSERT(!Stalled(hw));
  HostOut(dev, hw.out0, data);
  TEST_ASSERT(Stalled(hw));
  TEST_ASSERT(dev.ready == ready);
}

Bytes GetDescriptor(FakeUsb& dev, Hw& hw, uint8_t type, uint8_t index, uint16_t length,
                    uint16_t lang = 0U)
{
  return ControlIn(
      dev, hw, Make(0x80U, 6U, static_cast<uint16_t>(type << 8U | index), lang, length));
}

void SetConfiguration(FakeUsb& dev, Hw& hw, uint8_t value)
{
  ControlNone(dev, hw, Make(0x00U, 9U, value, 0U, 0U));
}

void SetAddress(FakeUsb& dev, Hw& hw)
{
  ControlNone(dev, hw, Make(0x00U, 5U, 7U, 0U, 0U));
}

/// Init、复位、编址、配置 1 / Init, reset, address, configuration 1
void Enumerate(FakeUsb& dev, Hw& hw, Speed speed = Speed::FULL)
{
  dev.Init(false);
  dev.Reset(speed);
  SetAddress(dev, hw);
  SetConfiguration(dev, hw, 1U);
}

// ---- gs_usb 请求 / gs_usb requests ----

constexpr uint8_t OUT_VENDOR = 0x41U;  ///< OUT、厂商、接口 / OUT, vendor, interface
constexpr uint8_t IN_VENDOR = 0xC1U;   ///< IN、厂商、接口 / IN, vendor, interface

constexpr uint8_t Req(Gs::BReq request) { return static_cast<uint8_t>(request); }

Bytes U32s(std::initializer_list<uint32_t> values)
{
  Bytes bytes;
  for (uint32_t value : values)
  {
    const auto* p = reinterpret_cast<const uint8_t*>(&value);
    bytes.insert(bytes.end(), p, p + sizeof(value));
  }
  return bytes;
}

uint32_t U32(const Bytes& bytes, size_t offset)
{
  uint32_t value = 0;
  std::memcpy(&value, &bytes[offset], sizeof(value));
  return value;
}

void VendorOut(FakeUsb& dev, Hw& hw, Gs::BReq request, uint16_t value, const Bytes& data)
{
  ControlOut(
      dev, hw,
      Make(OUT_VENDOR, Req(request), value, 0U, static_cast<uint16_t>(data.size())),
      data);
}

Bytes VendorIn(FakeUsb& dev, Hw& hw, Gs::BReq request, uint16_t value, uint16_t length)
{
  return ControlIn(dev, hw, Make(IN_VENDOR, Req(request), value, 0U, length));
}

void HostFormat(FakeUsb& dev, Hw& hw)
{
  VendorOut(dev, hw, Gs::BReq::HOST_FORMAT, 1U, U32s({0x0000BEEFU}));
}

void Mode(FakeUsb& dev, Hw& hw, uint16_t channel, Gs::CanMode mode, uint32_t flags = 0U)
{
  VendorOut(dev, hw, Gs::BReq::MODE, channel, U32s({static_cast<uint32_t>(mode), flags}));
}

/// 主机发往设备的一帧：12 字节头加 8（经典）或 64（FD）字节数据 / A host frame: 12-byte
/// header plus 8 (classic) or 64 (FD) data bytes
Bytes Frame(uint32_t echo_id, uint32_t can_id, uint8_t dlc, uint8_t channel,
            uint8_t flags, const Bytes& data)
{
  const size_t data_size = (flags & Gs::CAN_FLAG_FD) != 0U ? 64U : 8U;
  Bytes frame(12U + data_size, 0U);
  std::memcpy(&frame[0], &echo_id, 4U);
  std::memcpy(&frame[4], &can_id, 4U);
  frame[8] = dlc;
  frame[9] = channel;
  frame[10] = flags;
  std::copy(data.begin(), data.begin() + std::min(data.size(), data_size),
            frame.begin() + 12);
  return frame;
}

Bytes Pattern(size_t size, uint8_t seed)
{
  Bytes bytes(size);
  for (size_t i = 0; i < size; ++i)
  {
    bytes[i] = static_cast<uint8_t>(seed + i * 7U);
  }
  return bytes;
}

/// 记录发送、可注入接收的经典 CAN / Classic CAN that records sends and injects receives
class StubCan : public LibXR::CAN
{
 public:
  ErrorCode SetConfig(const Configuration& cfg) override
  {
    if (configure_result != ErrorCode::OK)
    {
      return configure_result;
    }
    last = cfg;
    ++configures;
    return ErrorCode::OK;
  }
  uint32_t GetClockFreq() const override { return 42000000U; }
  ErrorCode AddMessage(const ClassicPack& pack) override
  {
    tx.push_back(pack);
    return ErrorCode::OK;
  }
  ErrorCode GetErrorState(ErrorState& state) const override
  {
    if (!has_state)
    {
      return ErrorCode::NOT_SUPPORT;
    }
    state = error_state;
    return ErrorCode::OK;
  }

  /// 经已登记的回调注入一帧 / Play a received frame through the registered callbacks
  void Inject(uint32_t id, const Bytes& data, Type type = Type::STANDARD)
  {
    ClassicPack pack{};
    pack.id = id;
    pack.type = type;
    pack.dlc = static_cast<uint8_t>(data.size());
    std::copy(data.begin(), data.end(), pack.data);
    OnMessage(pack, false);
  }

  Configuration last{};
  size_t configures = 0;
  std::vector<ClassicPack> tx;
  bool has_state = false;
  ErrorState error_state{};
  ErrorCode configure_result = ErrorCode::OK;
};

/// 记录发送、可注入接收的 FDCAN / FDCAN that records sends and injects receives
class StubFdCan : public LibXR::FDCAN
{
 public:
  ErrorCode SetConfig(const CAN::Configuration& cfg) override
  {
    static_cast<CAN::Configuration&>(last) = cfg;
    ++configures;
    return ErrorCode::OK;
  }
  ErrorCode SetConfig(const FDCAN::Configuration& cfg) override
  {
    if (fd_configure_result != ErrorCode::OK)
    {
      return fd_configure_result;
    }
    last = cfg;
    ++fd_configures;
    return ErrorCode::OK;
  }
  uint32_t GetClockFreq() const override { return 80000000U; }
  ErrorCode AddMessage(const ClassicPack& pack) override
  {
    tx.push_back(pack);
    return ErrorCode::OK;
  }
  ErrorCode AddMessage(const FDPack& pack) override
  {
    fd_tx.push_back(pack);
    return ErrorCode::OK;
  }

  void InjectFd(uint32_t id, const Bytes& data)
  {
    FDPack pack{};
    pack.id = id;
    pack.type = Type::STANDARD;
    pack.len = static_cast<uint8_t>(data.size());
    std::copy(data.begin(), data.end(), pack.data);
    OnMessage(pack, false);
  }

  void Inject(uint32_t id, const Bytes& data)
  {
    ClassicPack pack{};
    pack.id = id;
    pack.type = Type::STANDARD;
    pack.dlc = static_cast<uint8_t>(data.size());
    std::copy(data.begin(), data.end(), pack.data);
    OnMessage(pack, false);
  }

  FDCAN::Configuration last{};
  size_t configures = 0;
  size_t fd_configures = 0;
  std::vector<ClassicPack> tx;
  std::vector<FDPack> fd_tx;
  ErrorCode fd_configure_result = ErrorCode::OK;
};

/// 记录写入的 GPIO / GPIO that records writes
class StubGpio : public LibXR::GPIO
{
 public:
  bool Read() override { return value; }
  void Write(bool on) override
  {
    value = on;
    ++writes;
  }
  ErrorCode EnableInterrupt() override { return ErrorCode::OK; }
  ErrorCode DisableInterrupt() override { return ErrorCode::OK; }
  ErrorCode SetConfig(Configuration) override { return ErrorCode::OK; }

  bool value = false;
  size_t writes = 0;
};

using Gs2 = GsUsbClass<2>;

/// 两个经典通道的 gs_usb，已枚举并协商字节序 / A two-channel classic gs_usb,
/// enumerated with the byte order negotiated
struct Fixture
{
  explicit Fixture(size_t rx_queue_size = 32U, size_t echo_queue_size = 32U,
                   LibXR::Database* database = nullptr, uint8_t slots = 0U)
      : hw(1024U, slots),
        gs(EPNumber::EP1, EPNumber::EP2, {&can0, &can1}, rx_queue_size, echo_queue_size,
           nullptr, {}, database),
        dev(hw, {{&gs}})
  {
    Enumerate(dev, hw);
    HostFormat(dev, hw);
  }

  /// 启动通道 / Start the channels
  void Start(bool ch0, bool ch1, uint32_t flags = 0U)
  {
    if (ch0)
    {
      Mode(dev, hw, 0U, Gs::CanMode::START, flags);
    }
    if (ch1)
    {
      Mode(dev, hw, 1U, Gs::CanMode::START, flags);
    }
  }

  std::vector<Bytes>& Sent() { return hw.ep1_in.transfers; }

  Hw hw;
  StubCan can0;
  StubCan can1;
  Gs2 gs;
  FakeUsb dev;
};

// ---- 用例 / Cases ----

/// 描述符、字符串、绑定后挂好一个多包接收 / Descriptors, strings, one multi-packet
/// receive armed after the bind
void TestEnumeration()
{
  Hw hw;
  StubCan can0;
  StubCan can1;
  Gs2 gs(EPNumber::EP1, EPNumber::EP2, {&can0, &can1});
  FakeUsb dev(hw, {{&gs}}, Speed::HIGH);
  dev.Init(false);
  dev.Reset(Speed::FULL);

  // 全速：配置头、一个厂商接口、OUT 和 IN 两个批量端点，共 32 字节。
  // Full speed: header, one vendor interface, bulk OUT and IN, 32 bytes in all.
  const Bytes fs = GetDescriptor(dev, hw, 2U, 0U, 255U);
  TEST_ASSERT(fs.size() == 32U && fs[2] == 32U && fs[4] == 1U);
  TEST_ASSERT(fs[9] == 9U && fs[10] == 4U && fs[11] == 0U && fs[12] == 0U);
  TEST_ASSERT(fs[13] == 2U && fs[14] == 0xFFU && fs[15] == 0xFFU && fs[16] == 0xFFU);
  TEST_ASSERT(fs[17] == 4U);
  TEST_ASSERT(fs[18] == 7U && fs[19] == 5U && fs[20] == 0x02U && fs[21] == 2U);
  TEST_ASSERT(fs[22] == 64U && fs[23] == 0U && fs[24] == 0U);
  TEST_ASSERT(fs[25] == 7U && fs[27] == 0x81U && fs[28] == 2U && fs[29] == 64U);

  // 高速的包长 512 / High speed: 512
  const Bytes hs = GetDescriptor(dev, hw, 7U, 0U, 255U);
  TEST_ASSERT(hs.size() == 32U && hs[1] == 7U);
  TEST_ASSERT(hs[22] == 0U && hs[23] == 2U && hs[29] == 0U && hs[30] == 2U);

  // 接口字符串是 4 号 / The interface string is index 4
  const Bytes name = GetDescriptor(dev, hw, 3U, 4U, 255U, 0x0409U);
  TEST_ASSERT(name.size() == 2U + 2U * 12U && name[2] == 'X' && name[2 + 2 * 6] == 'G');

  // 写描述符不碰端点；绑定后只有一个接收在途，长度为 MaxTransferSize（8 个全速包）。
  // Writing descriptors touches no endpoint; after the bind exactly one receive is in
  // flight, MaxTransferSize long (8 full-speed packets).
  TEST_ASSERT(hw.ep1_in.configures == 0U && hw.ep2_out.configures == 0U);
  SetAddress(dev, hw);
  SetConfiguration(dev, hw, 1U);
  TEST_ASSERT(hw.ep1_in.MaxPacketSize() == 64U && hw.ep2_out.MaxPacketSize() == 64U);
  TEST_ASSERT(hw.ep2_out.MaxTransferSize() == 512U);
  TEST_ASSERT(hw.ep2_out.inflight && hw.ep2_out.starts.size() == 1U);
  TEST_ASSERT(hw.ep2_out.LastLength() == 512U);
  TEST_ASSERT(!hw.ep1_in.inflight && !gs.IsHostFormatOK());

  // 高速复位后再配置：包长 512，接收仍为一个请求 512 字节。
  // Configured again after a high-speed reset: 512-byte packets, still one 512-byte
  // receive.
  dev.Reset(Speed::HIGH);
  SetAddress(dev, hw);
  SetConfiguration(dev, hw, 1U);
  TEST_ASSERT(hw.ep1_in.MaxPacketSize() == 512U && hw.ep2_out.MaxPacketSize() == 512U);
  TEST_ASSERT(hw.ep2_out.starts.size() == 2U && hw.ep2_out.LastLength() == 512U);
}

/// 厂商请求：应答、生效、拒绝 / Vendor requests: replies, effects, refusals
void TestRequests()
{
  Hw hw;
  StubCan can0;
  StubCan can1;
  StubGpio identify;
  StubGpio term0;
  Gs2 gs(EPNumber::EP1, EPNumber::EP2, {&can0, &can1}, 32U, 32U, &identify,
         {&term0, nullptr});
  FakeUsb dev(hw, {{&gs}});
  Enumerate(dev, hw);

  // DEVICE_CONFIG：icount = 通道数 - 1，软件版本 2，硬件版本 1。
  // DEVICE_CONFIG: icount = channels - 1, software version 2, hardware version 1.
  const Bytes config = VendorIn(dev, hw, Gs::BReq::DEVICE_CONFIG, 0U, 12U);
  TEST_ASSERT(config.size() == 12U && config[3] == 1U);
  TEST_ASSERT(U32(config, 4) == 2U && U32(config, 8) == 1U);

  // BT_CONST：40 字节；有 IDENTIFY 和终端电阻 GPIO，没有 FD。
  // BT_CONST: 40 bytes; IDENTIFY and termination GPIO present, no FD.
  const Bytes bt = VendorIn(dev, hw, Gs::BReq::BT_CONST, 0U, 40U);
  TEST_ASSERT(bt.size() == 40U);
  const uint32_t feature = U32(bt, 0);
  TEST_ASSERT((feature & Gs::CAN_FEAT_IDENTIFY) != 0U);
  TEST_ASSERT((feature & Gs::CAN_FEAT_TERMINATION) != 0U);
  TEST_ASSERT((feature & Gs::CAN_FEAT_USER_ID) != 0U);
  TEST_ASSERT((feature & Gs::CAN_FEAT_FD) == 0U);
  TEST_ASSERT(U32(bt, 4) == 42000000U && U32(bt, 8) == 1U && U32(bt, 12) == 16U);
  TEST_ASSERT(U32(bt, 32) == 1024U && U32(bt, 36) == 1U);
  // 主机给更长的 wLength 时以短包结束 / A longer wLength ends with a short packet
  TEST_ASSERT(VendorIn(dev, hw, Gs::BReq::BT_CONST, 0U, 64U) == bt);

  // FD 请求在经典 CAN 上被拒 / FD requests are refused on classic CAN
  Refused(dev, hw, Make(IN_VENDOR, Req(Gs::BReq::BT_CONST_EXT), 0U, 0U, 72U));
  Refused(dev, hw, Make(OUT_VENDOR, Req(Gs::BReq::DATA_BITTIMING), 0U, 0U, 20U));

  // 字节序协商之前 MODE 和 BITTIMING 在数据阶段被拒。
  // Before the byte order negotiation MODE and BITTIMING are refused at the data stage.
  DataRefused(dev, hw, Make(OUT_VENDOR, Req(Gs::BReq::MODE), 0U, 0U, 8U), U32s({1U, 0U}));
  DataRefused(dev, hw, Make(OUT_VENDOR, Req(Gs::BReq::BITTIMING), 0U, 0U, 20U),
              U32s({1U, 10U, 2U, 1U, 4U}));
  TEST_ASSERT(can0.configures == 0U);

  // 错误的标记被接受但不算协商 / A wrong marker is accepted but negotiates nothing
  VendorOut(dev, hw, Gs::BReq::HOST_FORMAT, 1U, U32s({0xEFBE0000U}));
  TEST_ASSERT(!gs.IsHostFormatOK());
  HostFormat(dev, hw);
  TEST_ASSERT(gs.IsHostFormatOK());

  // BITTIMING：42 MHz / (4 × (1 + 1 + 10 + 2)) = 750 kbit/s，采样点 12/14。
  // BITTIMING: 42 MHz / (4 × (1 + 1 + 10 + 2)) = 750 kbit/s, sample point 12/14.
  VendorOut(dev, hw, Gs::BReq::BITTIMING, 1U, U32s({1U, 10U, 2U, 1U, 4U}));
  TEST_ASSERT(can1.configures == 1U && can1.last.bitrate == 750000U);
  TEST_ASSERT(can1.last.bit_timing.brp == 4U && can1.last.bit_timing.prop_seg == 1U);
  TEST_ASSERT(can1.last.bit_timing.phase_seg1 == 10U);
  TEST_ASSERT(can1.last.bit_timing.phase_seg2 == 2U && can1.last.bit_timing.sjw == 1U);
  TEST_ASSERT(can1.last.sample_point > 0.857f && can1.last.sample_point < 0.858f);

  // MODE：模式标志进 CAN 配置；不是 RESET/START 的模式在数据阶段被拒。
  // MODE: the mode flags reach the CAN configuration; a mode other than RESET/START is
  // refused at the data stage.
  Mode(dev, hw, 0U, Gs::CanMode::START,
       Gs::GSCAN_MODE_LOOP_BACK | Gs::GSCAN_MODE_ONE_SHOT);
  TEST_ASSERT(can0.configures == 1U && can0.last.mode.loopback);
  TEST_ASSERT(can0.last.mode.one_shot && !can0.last.mode.listen_only);
  DataRefused(dev, hw, Make(OUT_VENDOR, Req(Gs::BReq::MODE), 0U, 0U, 8U), U32s({7U, 0U}));

  // BITTIMING 分母 32 位溢出和除零：必须安全拒绝/不崩溃 / 32-bit wrap & div-by-zero
  // in BITTIMING must not fault
  VendorOut(dev, hw, Gs::BReq::BITTIMING, 0U, U32s({0xFFFFFFFFU, 0U, 0U, 1U, 1U}));
  VendorOut(dev, hw, Gs::BReq::BITTIMING, 0U, U32s({0xFFFFU, 0U, 0U, 1U, 0x10000U}));

  // MODE 配置失败时状态必须回滚且传回错误 / HandleMode rolls back & rejects on failure
  can0.configure_result = ErrorCode::FAILED;
  DataRefused(dev, hw, Make(OUT_VENDOR, Req(Gs::BReq::MODE), 0U, 0U, 8U),
              U32s({static_cast<uint32_t>(Gs::CanMode::RESET), 0U}));
  can0.configure_result = ErrorCode::OK;
  // ch0 仍保持开启状态：收发帧依然入队 / ch0 remains enabled cleanly after rollback
  can0.Inject(0x321U, {9U});
  TEST_ASSERT(hw.ep1_in.inflight);
  Drain(dev, hw.ep1_in);

  // IDENTIFY 写 GPIO / IDENTIFY writes the GPIO
  VendorOut(dev, hw, Gs::BReq::IDENTIFY, 0U, U32s({1U}));
  TEST_ASSERT(identify.value && identify.writes == 1U);
  VendorOut(dev, hw, Gs::BReq::IDENTIFY, 1U, U32s({0U}));
  TEST_ASSERT(!identify.value && identify.writes == 2U);

  // 终端电阻：通道 0 有 GPIO，通道 1 没有；状态都能读回。
  // Termination: channel 0 has a GPIO, channel 1 none; both read back their state.
  VendorOut(dev, hw, Gs::BReq::SET_TERMINATION, 0U, U32s({1U}));
  TEST_ASSERT(term0.value && term0.writes == 1U);
  VendorOut(dev, hw, Gs::BReq::SET_TERMINATION, 1U, U32s({1U}));
  TEST_ASSERT(VendorIn(dev, hw, Gs::BReq::GET_TERMINATION, 0U, 4U) == U32s({1U}));
  TEST_ASSERT(VendorIn(dev, hw, Gs::BReq::GET_TERMINATION, 1U, 4U) == U32s({1U}));
  VendorOut(dev, hw, Gs::BReq::SET_TERMINATION, 0U, U32s({0U}));
  TEST_ASSERT(!term0.value);
  TEST_ASSERT(VendorIn(dev, hw, Gs::BReq::GET_TERMINATION, 0U, 4U) == U32s({0U}));

  // GET_STATE：CAN 不给状态时为 ERROR_ACTIVE；给了按 bus_off、passive、warning 取。
  // GET_STATE: ERROR_ACTIVE when the CAN gives no state; otherwise by bus_off, passive,
  // warning.
  TEST_ASSERT(VendorIn(dev, hw, Gs::BReq::GET_STATE, 1U, 12U) == U32s({0U, 0U, 0U}));
  can1.has_state = true;
  can1.error_state.error_passive = true;
  can1.error_state.rx_error_counter = 130U;
  can1.error_state.tx_error_counter = 7U;
  TEST_ASSERT(VendorIn(dev, hw, Gs::BReq::GET_STATE, 1U, 12U) ==
              U32s({static_cast<uint32_t>(Gs::CanState::ERROR_PASSIVE), 130U, 7U}));
  can1.error_state.bus_off = true;
  TEST_ASSERT(U32(VendorIn(dev, hw, Gs::BReq::GET_STATE, 1U, 12U), 0) ==
              static_cast<uint32_t>(Gs::CanState::BUS_OFF));

  // TIMESTAMP 4 字节；USER_ID 没有数据库时只在内存里往返。
  // TIMESTAMP is 4 bytes; without a database USER_ID round-trips in memory only.
  TEST_ASSERT(VendorIn(dev, hw, Gs::BReq::TIMESTAMP, 0U, 4U).size() == 4U);
  VendorOut(dev, hw, Gs::BReq::SET_USER_ID, 0U, U32s({0xA5A5F00DU}));
  TEST_ASSERT(VendorIn(dev, hw, Gs::BReq::GET_USER_ID, 0U, 4U) == U32s({0xA5A5F00DU}));

  // BERR 只改开关 / BERR only flips the switch
  VendorOut(dev, hw, Gs::BReq::BERR, 1U, U32s({1U}));

  // 拒绝：类请求、方向反了、长度不对、通道越界、未知请求、接收者为端点。
  // Refused: class requests, wrong direction, wrong length, channel out of range,
  // unknown request, endpoint recipient.
  Refused(dev, hw, Make(0xA1U, Req(Gs::BReq::BT_CONST), 0U, 0U, 40U));
  Refused(dev, hw, Make(0x21U, Req(Gs::BReq::MODE), 0U, 0U, 8U));
  Refused(dev, hw, Make(OUT_VENDOR, Req(Gs::BReq::BT_CONST), 0U, 0U, 40U));
  Refused(dev, hw, Make(IN_VENDOR, Req(Gs::BReq::MODE), 0U, 0U, 8U));
  Refused(dev, hw, Make(IN_VENDOR, Req(Gs::BReq::BT_CONST), 0U, 0U, 39U));
  Refused(dev, hw, Make(OUT_VENDOR, Req(Gs::BReq::MODE), 0U, 0U, 4U));
  Refused(dev, hw, Make(OUT_VENDOR, Req(Gs::BReq::MODE), 2U, 0U, 8U));
  Refused(dev, hw, Make(OUT_VENDOR, Req(Gs::BReq::BITTIMING), 2U, 0U, 20U));
  Refused(dev, hw, Make(IN_VENDOR, Req(Gs::BReq::GET_STATE), 2U, 0U, 12U));
  Refused(dev, hw, Make(IN_VENDOR, Req(Gs::BReq::GET_TERMINATION), 2U, 0U, 4U));
  Refused(dev, hw, Make(OUT_VENDOR, Req(Gs::BReq::SET_USER_ID), 0U, 0U, 2U));
  Refused(dev, hw, Make(IN_VENDOR, 0x7FU, 0U, 0U, 4U));
  Refused(dev, hw, Make(0xC2U, Req(Gs::BReq::BT_CONST), 0U, 0x81U, 40U));
  // 拒绝之后请求照常工作 / Requests work as usual after the refusals
  TEST_ASSERT(VendorIn(dev, hw, Gs::BReq::DEVICE_CONFIG, 0U, 12U) == config);
}

/// 主机发来的帧：交给 CAN、回显、丢弃的各种情形 / Host frames: to CAN, echo, and every
/// way to be dropped
void TestHostFrames()
{
  Fixture f;
  f.Start(true, true);
  FakeEp& out = f.hw.ep2_out;
  FakeEp& in = f.hw.ep1_in;

  const Bytes payload{0x11U, 0x22U, 0x33U};
  HostOut(f.dev, out, Frame(0x1234U, 0x123U, 3U, 0U, 0U, payload));
  // 帧到 CAN 一次，回显从 IN 回到主机，接收重新挂好（仍是一个）。
  // The frame reached CAN once, the echo went back on IN, the receive is armed again
  // (still one).
  TEST_ASSERT(f.can0.tx.size() == 1U && f.can0.tx[0].id == 0x123U);
  TEST_ASSERT(f.can0.tx[0].type == LibXR::CAN::Type::STANDARD && f.can0.tx[0].dlc == 3U);
  TEST_ASSERT(std::equal(payload.begin(), payload.end(), f.can0.tx[0].data));
  TEST_ASSERT(in.inflight && in.LastLength() == 20U);
  TEST_ASSERT(out.inflight && out.starts.size() == 2U && out.LastLength() == 512U);
  InDone(f.dev, in);
  const Bytes& echo = f.Sent().back();
  TEST_ASSERT(U32(echo, 0) == 0x1234U && U32(echo, 4) == 0x123U);
  TEST_ASSERT(echo[8] == 3U && echo[9] == 0U && echo[10] == 0U);
  TEST_ASSERT(std::equal(payload.begin(), payload.end(), echo.begin() + 12));

  // 扩展远程帧到通道 1；DLC 超过 8 时取 8。
  // An extended remote frame to channel 1; a DLC above 8 becomes 8.
  HostOut(f.dev, out,
          Frame(0x10U, Gs::CAN_EFF_FLAG | Gs::CAN_RTR_FLAG | 0x12345678U, 12U, 1U, 0U,
                Pattern(8, 1)));
  TEST_ASSERT(f.can1.tx.size() == 1U && f.can1.tx[0].id == 0x12345678U);
  TEST_ASSERT(f.can1.tx[0].type == LibXR::CAN::Type::REMOTE_EXTENDED);
  TEST_ASSERT(f.can1.tx[0].dlc == 8U);
  Drain(f.dev, in);

  // 主机不要回显（echo_id 为 0xFFFFFFFF）：只到 CAN。
  // The host wants no echo (echo_id 0xFFFFFFFF): CAN only.
  const size_t sent = f.Sent().size();
  HostOut(f.dev, out, Frame(0xFFFFFFFFU, 0x7U, 1U, 0U, 0U, {1U}));
  TEST_ASSERT(f.can0.tx.size() == 2U && !in.inflight);

  // 丢弃：通道未启动、通道越界、太短、经典设备上的 FD 帧、零长度；接收照样重挂。
  // Dropped: channel not started, channel out of range, too short, an FD frame on a
  // classic device, zero length; the receive is armed again each time.
  Mode(f.dev, f.hw, 0U, Gs::CanMode::RESET);
  const size_t starts = out.starts.size();
  HostOut(f.dev, out, Frame(0x1235U, 0x124U, 1U, 0U, 0U, {1U}));
  HostOut(f.dev, out, Frame(0x1236U, 0x124U, 1U, 5U, 0U, {1U}));
  HostOut(f.dev, out, Bytes(12U, 0U));
  HostOut(f.dev, out, Frame(0x1237U, 0x124U, 1U, 1U, Gs::CAN_FLAG_FD, {1U}));
  HostOut(f.dev, out, {});
  TEST_ASSERT(f.can0.tx.size() == 2U && f.can1.tx.size() == 1U);
  TEST_ASSERT(!in.inflight && f.Sent().size() == sent);
  TEST_ASSERT(out.inflight && out.starts.size() == starts + 5U);

  // 主机 HALT 数据 OUT：在途接收以 ABORTED 结束，类当场重挂，接收排队到 CLEAR_FEATURE
  // 后启动。类不需要 OnHaltCleared。
  // The host halts data OUT: the receive in flight ends with ABORTED, the class arms
  // again at once, and the receive waits for CLEAR_FEATURE. The class needs no
  // OnHaltCleared.
  Mode(f.dev, f.hw, 0U, Gs::CanMode::START);
  const size_t before_halt = out.starts.size();
  ControlNone(f.dev, f.hw, Make(0x02U, 3U, 0U, 0x02U, 0U));
  TEST_ASSERT(out.IsStalled() && !out.inflight && out.starts.size() == before_halt);
  ControlNone(f.dev, f.hw, Make(0x02U, 1U, 0U, 0x02U, 0U));
  TEST_ASSERT(!out.IsStalled() && out.inflight && out.starts.size() == before_halt + 1U);
  TEST_ASSERT(out.LastLength() == 512U);
  HostOut(f.dev, out, Frame(0x1238U, 0x125U, 1U, 0U, 0U, {2U}));
  TEST_ASSERT(f.can0.tx.size() == 3U && in.inflight);

  // HALT 数据 IN：在途的回显中止、不到主机；HALT 期间收到的帧排队，CLEAR_FEATURE 后发。
  // Halt data IN: the echo in flight is aborted and never reaches the host; frames
  // received while halted queue up and go after CLEAR_FEATURE.
  ControlNone(f.dev, f.hw, Make(0x02U, 3U, 0U, 0x81U, 0U));
  TEST_ASSERT(in.IsStalled() && !in.inflight);
  f.can0.Inject(0x600U, {1U});
  f.can0.Inject(0x601U, {2U});
  TEST_ASSERT(!in.inflight);
  ControlNone(f.dev, f.hw, Make(0x02U, 1U, 0U, 0x81U, 0U));
  TEST_ASSERT(in.inflight);
  Drain(f.dev, in);
  TEST_ASSERT(f.Sent().size() == sent + 2U);
  TEST_ASSERT(U32(f.Sent()[sent], 4) == 0x600U && U32(f.Sent()[sent + 1U], 4) == 0x601U);
}

/**
 * 多包接收与发送：一帧跨多个包，一个请求，以短包结束；请求长度为 MaxTransferSize，
 * 随速度和后端一次请求的上限变化。
 * Multi-packet receive and send: one frame spans several packets in one request and
 * ends with a short packet; the request is MaxTransferSize long, which follows the
 * speed and the backend's limit for one request.
 */
void TestMultiPacket()
{
  // 全速 FD：接收 512 字节 = 8 个包；76 字节的 FD 帧 = 64 + 12 字节两个包。
  // Full-speed FD: a 512-byte receive = 8 packets; a 76-byte FD frame = 64 + 12 bytes.
  {
    Hw hw;
    StubFdCan fd0;
    GsUsbClass<1> gs(EPNumber::EP1, EPNumber::EP2,
                     std::initializer_list<LibXR::FDCAN*>{&fd0});
    FakeUsb dev(hw, {{&gs}}, Speed::HIGH);
    Enumerate(dev, hw);
    TEST_ASSERT(hw.ep2_out.MaxPacketSize() == 64U && hw.ep2_out.LastLength() == 512U);
    const Bytes bt = VendorIn(dev, hw, Gs::BReq::BT_CONST_EXT, 0U, 72U);
    TEST_ASSERT(bt.size() == 72U && (U32(bt, 0) & Gs::CAN_FEAT_FD) != 0U);
    TEST_ASSERT(U32(bt, 4) == 80000000U);
    HostFormat(dev, hw);
    VendorOut(dev, hw, Gs::BReq::DATA_BITTIMING, 0U, U32s({1U, 6U, 2U, 1U, 1U}));
    TEST_ASSERT(fd0.fd_configures == 1U && fd0.last.data_bitrate == 8000000U);
    Mode(dev, hw, 0U, Gs::CanMode::START,
         Gs::GSCAN_MODE_FD | Gs::GSCAN_MODE_HW_TIMESTAMP);
    TEST_ASSERT(fd0.last.fd_mode.fd_enabled);

    const Bytes data = Pattern(64, 3);
    const Bytes frame =
        Frame(0x77U, 0x123U, 15U, 0U, Gs::CAN_FLAG_FD | Gs::CAN_FLAG_BRS, data);
    TEST_ASSERT(frame.size() == 76U && frame.size() % 64U != 0U);
    HostOut(dev, hw.ep2_out, frame);
    TEST_ASSERT(fd0.fd_tx.size() == 1U && fd0.fd_tx[0].len == 64U);
    TEST_ASSERT(fd0.fd_tx[0].id == 0x123U &&
                std::equal(data.begin(), data.end(), fd0.fd_tx[0].data));
    // 回显 80 字节（FD 加时间戳）= 64 + 16，一个请求，不加零长度包。
    // The echo is 80 bytes (FD with timestamp) = 64 + 16, one request, no zero-length
    // packet.
    TEST_ASSERT(hw.ep1_in.inflight && hw.ep1_in.LastLength() == 80U);
    InDone(dev, hw.ep1_in);
    TEST_ASSERT(!hw.ep1_in.inflight && hw.ep1_in.starts.size() == 1U);
    const Bytes& echo = hw.ep1_in.transfers.back();
    TEST_ASSERT(U32(echo, 0) == 0x77U && (echo[10] & Gs::CAN_FLAG_FD) != 0U);
    TEST_ASSERT(std::equal(data.begin(), data.end(), echo.begin() + 12));

    // DLC 9 = 12 字节 / DLC 9 = 12 bytes
    HostOut(dev, hw.ep2_out, Frame(0x78U, 0x124U, 9U, 0U, Gs::CAN_FLAG_FD, data));
    TEST_ASSERT(fd0.fd_tx.size() == 2U && fd0.fd_tx[1].len == 12U);
    Drain(dev, hw.ep1_in);
    // 标了 FD 却只有 20 字节：丢弃 / Flagged FD but only 20 bytes: dropped
    const Bytes short_fd = Frame(0x7BU, 0x1U, 1U, 0U, Gs::CAN_FLAG_FD, {1U});
    HostOut(dev, hw.ep2_out, Bytes(short_fd.begin(), short_fd.begin() + 20));
    TEST_ASSERT(fd0.fd_tx.size() == 2U && !hw.ep1_in.inflight);

    // 设备收到的 FD 帧：20 字节 → DLC 11，发往主机 80 字节。前面先收一帧 64 字节的
    // 0xFF：后一帧 20 字节之后全是 0，不带前一帧的数据。
    // An FD frame the device received: 20 bytes -> DLC 11, 80 bytes to the host. A
    // 64-byte frame of 0xFF comes first: the later frame is all 0 past its 20 bytes and
    // carries nothing of the earlier one.
    fd0.InjectFd(0x54U, Bytes(64U, 0xFFU));
    InDone(dev, hw.ep1_in);
    fd0.InjectFd(0x55U, Pattern(20, 9));
    TEST_ASSERT(hw.ep1_in.inflight && hw.ep1_in.LastLength() == 80U);
    InDone(dev, hw.ep1_in);
    const Bytes& rx = hw.ep1_in.transfers.back();
    TEST_ASSERT(U32(rx, 0) == 0xFFFFFFFFU && U32(rx, 4) == 0x55U && rx[8] == 11U);
    TEST_ASSERT(rx[10] == Gs::CAN_FLAG_FD && rx[12 + 19] == Pattern(20, 9)[19]);
    TEST_ASSERT(std::all_of(rx.begin() + 12 + 20, rx.begin() + 12 + 64,
                            [](uint8_t b) { return b == 0U; }));

    // FD 通道上的经典帧：到 CAN 的经典发送，回显 24 字节（经典加时间戳）。
    // A classic frame on an FD channel: classic send on CAN, a 24-byte echo (classic
    // with timestamp).
    HostOut(dev, hw.ep2_out, Frame(0x79U, 0x125U, 2U, 0U, 0U, {7U, 8U}));
    TEST_ASSERT(fd0.tx.size() == 1U && fd0.tx[0].dlc == 2U);
    TEST_ASSERT(hw.ep1_in.inflight && hw.ep1_in.LastLength() == 24U);
    InDone(dev, hw.ep1_in);
    fd0.Inject(0x56U, {1U, 2U, 3U});
    TEST_ASSERT(hw.ep1_in.inflight && hw.ep1_in.LastLength() == 24U);
    InDone(dev, hw.ep1_in);

    // 高速：一个包 512 字节，接收仍是 512；FD 帧是一个短包。
    // High speed: one packet is 512 bytes, the receive is still 512; an FD frame is one
    // short packet.
    dev.Reset(Speed::HIGH);
    SetAddress(dev, hw);
    SetConfiguration(dev, hw, 1U);
    TEST_ASSERT(hw.ep2_out.MaxPacketSize() == 512U && hw.ep2_out.LastLength() == 512U);
    HostFormat(dev, hw);
    Mode(dev, hw, 0U, Gs::CanMode::START, Gs::GSCAN_MODE_FD);
    HostOut(dev, hw.ep2_out, frame);
    TEST_ASSERT(fd0.fd_tx.size() == 3U && hw.ep1_in.LastLength() == 76U);
    Drain(dev, hw.ep1_in);

    // 不带 FD 标志重新启动后，FD 帧丢弃，经典帧照常。
    // Started again without the FD flag, FD frames are dropped and classic ones pass.
    Mode(dev, hw, 0U, Gs::CanMode::START);
    HostOut(dev, hw.ep2_out, frame);
    HostOut(dev, hw.ep2_out, Frame(0x7AU, 0x126U, 1U, 0U, 0U, {1U}));
    TEST_ASSERT(fd0.fd_tx.size() == 3U && fd0.tx.size() == 2U);
    TEST_ASSERT(hw.ep1_in.LastLength() == 20U);
  }

  // 后端一次请求最多 2 个包：全速时 MaxTransferSize 128，接收 128 字节，FD 帧照常。
  // The backend allows 2 packets per request: at full speed MaxTransferSize is 128, the
  // receive is 128 bytes, FD frames work as usual.
  {
    Hw hw;
    hw.ep1_in.limits.max_packets = 2U;
    hw.ep2_out.limits.max_packets = 2U;
    StubFdCan fd0;
    GsUsbClass<1> gs(EPNumber::EP1, EPNumber::EP2,
                     std::initializer_list<LibXR::FDCAN*>{&fd0});
    FakeUsb dev(hw, {{&gs}});
    Enumerate(dev, hw);
    TEST_ASSERT(hw.ep2_out.MaxTransferSize() == 128U && hw.ep2_out.LastLength() == 128U);
    TEST_ASSERT(hw.ep1_in.MaxTransferSize() == 128U);
    HostFormat(dev, hw);
    Mode(dev, hw, 0U, Gs::CanMode::START,
         Gs::GSCAN_MODE_FD | Gs::GSCAN_MODE_HW_TIMESTAMP);
    HostOut(dev, hw.ep2_out, Frame(0x1U, 0x1U, 15U, 0U, Gs::CAN_FLAG_FD, Pattern(64, 5)));
    TEST_ASSERT(fd0.fd_tx.size() == 1U && hw.ep1_in.LastLength() == 80U);
    TEST_ASSERT(hw.ep2_out.LastLength() == 128U);
  }
}

/// CAN 收到的帧、溢出标志、回显优先、通道轮流、错误帧队列 / Frames from CAN, the overflow
/// flag, echo priority, channel round-robin, the error-frame queue
void TestToHost()
{
  // 经典接收帧带 ECHO_ID_RX / A classic received frame carries ECHO_ID_RX
  {
    Fixture f;
    f.Start(true, true);
    f.can0.Inject(0x456U, {0xAAU, 0xBBU});
    TEST_ASSERT(f.hw.ep1_in.inflight && f.hw.ep1_in.LastLength() == 20U);
    InDone(f.dev, f.hw.ep1_in);
    const Bytes& frame = f.Sent().back();
    TEST_ASSERT(U32(frame, 0) == 0xFFFFFFFFU && U32(frame, 4) == 0x456U);
    TEST_ASSERT(frame[8] == 2U && frame[9] == 0U && frame[10] == 0U);
    TEST_ASSERT(frame[12] == 0xAAU && frame[13] == 0xBBU && frame[14] == 0U);
    // 扩展帧、远程帧的 ID 标志 / ID flags of extended and remote frames
    f.can1.Inject(0x1ABCDEFU, {}, LibXR::CAN::Type::EXTENDED);
    f.can1.Inject(0x12U, {}, LibXR::CAN::Type::REMOTE_STANDARD);
    Drain(f.dev, f.hw.ep1_in);
    TEST_ASSERT(U32(f.Sent()[1], 4) == (0x1ABCDEFU | Gs::CAN_EFF_FLAG));
    TEST_ASSERT(U32(f.Sent()[2], 4) == (0x12U | Gs::CAN_RTR_FLAG) &&
                f.Sent()[2][9] == 1U);
    // 未启动的通道不入队 / A channel not started queues nothing
    Mode(f.dev, f.hw, 1U, Gs::CanMode::RESET);
    f.can1.Inject(0x13U, {1U});
    TEST_ASSERT(!f.hw.ep1_in.inflight);
  }

  // 经典帧的数据区在 DLC 之后补 0：先收一帧 8 个 0xFF，再收一帧 2 字节，后一帧不带前一帧
  // 的数据。
  // A classic frame's data is 0 past its DLC: a frame of eight 0xFF comes first, then a
  // 2-byte one, which carries nothing of the earlier frame.
  {
    Fixture f;
    f.Start(true, true);
    f.can0.Inject(0x101U, Bytes(8U, 0xFFU));
    InDone(f.dev, f.hw.ep1_in);
    f.can0.Inject(0x102U, {0x01U, 0x02U});
    InDone(f.dev, f.hw.ep1_in);
    const Bytes& frame = f.Sent().back();
    TEST_ASSERT(U32(frame, 4) == 0x102U && frame[8] == 2U);
    TEST_ASSERT(frame[12] == 0x01U && frame[13] == 0x02U);
    TEST_ASSERT(std::all_of(frame.begin() + 14, frame.begin() + 20,
                            [](uint8_t b) { return b == 0U; }));
  }

  // 队列 4：两个在途、四个排队、第七个丢弃；溢出标志恰好一次。
  // Queue of 4: two in flight, four queued, the seventh dropped; the overflow flag
  // exactly once.
  {
    Fixture f(4U, 32U);
    f.Start(true, false);
    for (unsigned i = 0; i < 7U; ++i)
    {
      f.can0.Inject(0x100U + i, {static_cast<uint8_t>(i)});
    }
    Drain(f.dev, f.hw.ep1_in);
    TEST_ASSERT(f.Sent().size() == 6U);
    size_t overflow = 0;
    for (size_t i = 0; i < f.Sent().size(); ++i)
    {
      const Bytes& frame = f.Sent()[i];
      TEST_ASSERT(U32(frame, 0) == 0xFFFFFFFFU && frame[9] == 0U);
      TEST_ASSERT(U32(frame, 4) == 0x100U + i && frame[12] == i);
      overflow += (frame[10] & Gs::CAN_FLAG_OVERFLOW) != 0U ? 1U : 0U;
    }
    TEST_ASSERT(overflow == 1U);
  }

  // 溢出标志随该通道的下一帧，即使那一帧是回显，也不会落在别的通道上。
  // The overflow flag rides on the channel's next frame even when it is an echo, and
  // never on another channel's frame.
  {
    Fixture f(4U, 32U);
    f.Start(true, true);
    for (unsigned i = 0; i < 7U; ++i)
    {
      f.can0.Inject(0x100U + i, {static_cast<uint8_t>(i)});
    }
    HostOut(f.dev, f.hw.ep2_out, Frame(0x10U, 0x301U, 1U, 1U, 0U, {0x11U}));
    HostOut(f.dev, f.hw.ep2_out, Frame(0x20U, 0x300U, 1U, 0U, 0U, {0x22U}));
    Drain(f.dev, f.hw.ep1_in);
    TEST_ASSERT(f.Sent().size() == 8U);
    size_t overflow = 0;
    for (const Bytes& frame : f.Sent())
    {
      if ((frame[10] & Gs::CAN_FLAG_OVERFLOW) == 0U)
      {
        continue;
      }
      ++overflow;
      TEST_ASSERT(U32(frame, 0) == 0x20U && frame[9] == 0U);
    }
    TEST_ASSERT(overflow == 1U);
  }

  // 回显先于排队的接收帧 / Echoes go before queued received frames
  {
    Fixture f;
    f.Start(true, false);
    for (unsigned i = 0; i < 6U; ++i)
    {
      f.can0.Inject(0x100U + i, {static_cast<uint8_t>(i)});
    }
    for (unsigned i = 0; i < 3U; ++i)
    {
      HostOut(f.dev, f.hw.ep2_out, Frame(0x3000U + i, 0x300U + i, 1U, 0U, 0U, {0x5AU}));
    }
    TEST_ASSERT(f.can0.tx.size() == 3U);
    Drain(f.dev, f.hw.ep1_in);
    TEST_ASSERT(f.Sent().size() == 9U);
    for (unsigned i = 0; i < 2U; ++i)
    {
      TEST_ASSERT(U32(f.Sent()[i], 0) == 0xFFFFFFFFU && f.Sent()[i][12] == i);
    }
    for (unsigned i = 0; i < 3U; ++i)
    {
      TEST_ASSERT(U32(f.Sent()[2 + i], 0) == 0x3000U + i);
    }
    for (unsigned i = 0; i < 4U; ++i)
    {
      TEST_ASSERT(U32(f.Sent()[5 + i], 0) == 0xFFFFFFFFU && f.Sent()[5 + i][12] == 2 + i);
    }
  }

  // 两个通道交替发送，各自保持顺序 / The two channels alternate, each in its own order
  {
    Fixture f;
    f.Start(true, true);
    for (unsigned i = 0; i < 6U; ++i)
    {
      f.can0.Inject(0x100U + i, {static_cast<uint8_t>(i)});
      f.can1.Inject(0x200U + i, {static_cast<uint8_t>(i)});
    }
    Drain(f.dev, f.hw.ep1_in);
    TEST_ASSERT(f.Sent().size() == 12U);
    unsigned next[2] = {0U, 0U};
    unsigned max_run = 0;
    unsigned run = 0;
    uint8_t last = 0xFFU;
    for (const Bytes& frame : f.Sent())
    {
      const uint8_t ch = frame[9];
      TEST_ASSERT(ch < 2U && frame[12] == next[ch]);
      ++next[ch];
      run = (ch == last) ? run + 1U : 1U;
      max_run = std::max(max_run, run);
      last = ch;
    }
    TEST_ASSERT(next[0] == 6U && next[1] == 6U && max_run <= 2U);
  }

  // 错误帧有自己的队列，数据队列满时照样通过，并排在本通道排队的数据帧之前。
  // Error frames have their own queue: they get through with the data queue full, and
  // go before the channel's queued data frames.
  {
    Fixture f(2U, 32U);
    f.Start(true, false, Gs::GSCAN_MODE_BERR_REPORTING);
    for (unsigned i = 0; i < 6U; ++i)
    {
      f.can0.Inject(0x100U + i, {static_cast<uint8_t>(i)});
    }
    f.can0.Inject(LibXR::CAN::FromErrorID(LibXR::CAN::ErrorID::CAN_ERROR_ID_BUS_OFF), {},
                  LibXR::CAN::Type::ERROR);
    Drain(f.dev, f.hw.ep1_in);
    TEST_ASSERT(f.Sent().size() == 5U);
    const Bytes& error = f.Sent()[2];
    TEST_ASSERT(U32(error, 0) == 0xFFFFFFFFU && error[8] == Gs::CAN_ERR_DLC);
    TEST_ASSERT(U32(error, 4) == (Gs::CAN_ERR_FLAG | Gs::CAN_ERR_BUSOFF | 0x200U));

    // 错误被动：按计数置 TX 方向的位 / Error passive: the TX bit set from the counts
    f.can0.has_state = true;
    f.can0.error_state.tx_error_counter = 130U;
    f.can0.Inject(
        LibXR::CAN::FromErrorID(LibXR::CAN::ErrorID::CAN_ERROR_ID_ERROR_PASSIVE), {},
        LibXR::CAN::Type::ERROR);
    Drain(f.dev, f.hw.ep1_in);
    const Bytes& passive = f.Sent().back();
    TEST_ASSERT(U32(passive, 4) == (Gs::CAN_ERR_FLAG | Gs::CAN_ERR_CRTL | 0x200U));
    TEST_ASSERT(passive[13] == 0x20U && passive[18] == 130U && passive[19] == 0U);

    // 错误报告关掉后不上报 / Nothing once error reporting is off
    VendorOut(f.dev, f.hw, Gs::BReq::BERR, 0U, U32s({0U}));
    f.can0.Inject(LibXR::CAN::FromErrorID(LibXR::CAN::ErrorID::CAN_ERROR_ID_ACK), {},
                  LibXR::CAN::Type::ERROR);
    TEST_ASSERT(!f.hw.ep1_in.inflight);
  }

  // 回显队列满：留住 OUT 槽、不再挂接收（主机被 NAK），每帧只到 CAN 一次；IN 腾出空间
  // 后补推、交回槽、重挂，回显一个不少、顺序不变。
  // Echo queue full: the OUT slot is kept and nothing is armed (the host is NAKed),
  // each frame reaches CAN once; once IN frees room the echo is pushed, the slot handed
  // back and the receive armed again, with every echo in order.
  {
    Fixture f(32U, 4U);
    f.Start(true, false);
    unsigned sent = 0;
    while (f.hw.ep2_out.inflight && sent < 16U)
    {
      HostOut(f.dev, f.hw.ep2_out,
              Frame(0x2000U + sent, 0x200U + sent, 1U, 0U, 0U, {1U}));
      ++sent;
    }
    TEST_ASSERT(!f.hw.ep2_out.inflight && sent == 7U && f.can0.tx.size() == 7U);
    for (unsigned i = 0; i < 16U && !f.hw.ep2_out.inflight; ++i)
    {
      InDone(f.dev, f.hw.ep1_in);
    }
    TEST_ASSERT(f.hw.ep2_out.inflight && f.can0.tx.size() == 7U);
    Drain(f.dev, f.hw.ep1_in);
    TEST_ASSERT(f.Sent().size() == 7U);
    for (unsigned i = 0; i < 7U; ++i)
    {
      TEST_ASSERT(U32(f.Sent()[i], 0) == 0x2000U + i);
    }
  }
}

/// SET_USER_ID 只改内存，100 ms 的定时任务写数据库；新构造的类读回 / SET_USER_ID
/// changes memory only, the 100 ms timer task writes the database; a new class reads it
void TestUserId()
{
  // 自己的定时任务列表：不建定时线程，由测试 Refresh。
  // A timer list of our own: no timer thread is created, the test refreshes.
  LibXR::Timer::list_ = new LibXR::LockFreeList();

  class RamFlash : public LibXR::Flash
  {
   public:
    RamFlash(size_t min_erase, size_t min_write, RawData area)
        : Flash(min_erase, min_write, area), area_(area)
    {
    }
    ErrorCode Erase(size_t offset, size_t size) override
    {
      TEST_ASSERT(offset + size <= area_.size_);
      std::memset(static_cast<uint8_t*>(area_.addr_) + offset, 0xFF, size);
      return ErrorCode::OK;
    }
    ErrorCode Write(size_t offset, ConstRawData data) override
    {
      TEST_ASSERT(offset + data.size_ <= area_.size_);
      std::memcpy(static_cast<uint8_t*>(area_.addr_) + offset, data.addr_, data.size_);
      return ErrorCode::OK;
    }

   private:
    RawData area_;
  };

  std::array<uint8_t, 8192> db_mem{};
  RamFlash flash(256U, 8U, {db_mem.data(), db_mem.size()});
  LibXR::DatabaseRawSequential db(flash, 256U);
  db.Init();

  Fixture f(32U, 32U, &db);
  VendorOut(f.dev, f.hw, Gs::BReq::SET_USER_ID, 0U, U32s({0x11223344U}));
  TEST_ASSERT(VendorIn(f.dev, f.hw, Gs::BReq::GET_USER_ID, 0U, 4U) ==
              U32s({0x11223344U}));
  LibXR::Database::Key<uint32_t> before(db, "user_id", 0U);
  TEST_ASSERT(static_cast<uint32_t>(before) == 0U);
  for (unsigned i = 0; i < 100U; ++i)
  {
    LibXR::Timer::Refresh();
  }
  LibXR::Database::Key<uint32_t> probe(db, "user_id", 0U);
  TEST_ASSERT(static_cast<uint32_t>(probe) == 0x11223344U);

  // 同一个数据库上新构造的类在构造时读出这个值。
  // A class constructed on the same database loads the value at construction.
  Hw hw;
  StubCan can0;
  StubCan can1;
  Gs2 gs(EPNumber::EP1, EPNumber::EP2, {&can0, &can1}, 8U, 8U, nullptr, {}, &db);
  FakeUsb dev(hw, {{&gs}});
  Enumerate(dev, hw);
  TEST_ASSERT(VendorIn(dev, hw, Gs::BReq::GET_USER_ID, 0U, 4U) == U32s({0x11223344U}));
}

/// 绑定后收发都通 / Sending and receiving work after a bind
template <typename CanStub>
void CheckWorks(FakeUsb& dev, Hw& hw, CanStub& can0)
{
  HostFormat(dev, hw);
  Mode(dev, hw, 0U, Gs::CanMode::START);
  const size_t tx = can0.tx.size();
  const size_t sent = hw.ep1_in.transfers.size();
  HostOut(dev, hw.ep2_out, Frame(0x99U, 0x99U, 1U, 0U, 0U, {9U}));
  TEST_ASSERT(can0.tx.size() == tx + 1U);
  Drain(dev, hw.ep1_in);
  TEST_ASSERT(hw.ep1_in.transfers.size() == sent + 1U);
  TEST_ASSERT(U32(hw.ep1_in.transfers.back(), 0) == 0x99U);
  TEST_ASSERT(hw.ep2_out.inflight);
}

/// 绑定失败：Configure 出错、包长不符、一次请求装不下一帧、端点被占、端点不存在 / Bind
/// failure: a Configure error, a size mismatch, one request too small for a frame, an
/// endpoint taken, an endpoint missing
void TestBindFailure()
{
  // OUT 的 Configure 失败：IN 已配置，Unbind 关闭并交回；不挂接收。修好后同一配置可用。
  // OUT's Configure fails: IN was configured and Unbind closes and returns it; nothing
  // armed. Once fixed the same configuration works.
  {
    Hw hw;
    hw.ep2_out.configure_result = ErrorCode::FAILED;
    StubCan can0;
    StubCan can1;
    Gs2 gs(EPNumber::EP1, EPNumber::EP2, {&can0, &can1});
    FakeUsb dev(hw, {{&gs}});
    dev.Init(false);
    dev.Reset();
    Refused(dev, hw, Make(0x00U, 9U, 1U, 0U, 0U));
    TEST_ASSERT(hw.ep1_in.configures == 1U && hw.ep1_in.closes == 1U);
    TEST_ASSERT(hw.ep2_out.configures == 0U && hw.ep2_out.closes == 0U);
    TEST_ASSERT(hw.ep2_out.starts.empty());
    TEST_ASSERT(ControlIn(dev, hw, Make(0x80U, 8U, 0U, 0U, 1U)) == Bytes{0U});
    hw.ep2_out.configure_result = ErrorCode::OK;
    SetConfiguration(dev, hw, 1U);
    CheckWorks(dev, hw, can0);
  }

  // 读回的包长与描述符不同：CHECK_ERR / The size read back differs: CHECK_ERR
  {
    Hw hw;
    StubCan can0;
    StubCan can1;
    Gs2 gs(EPNumber::EP1, EPNumber::EP2, {&can0, &can1});
    FakeUsb dev(hw, {{&gs}});
    dev.Init(false);
    dev.Reset();
    hw.ep1_in.shrink_after = hw.ep1_in.limit_calls + 1;
    Refused(dev, hw, Make(0x00U, 9U, 1U, 0U, 0U));
    TEST_ASSERT(hw.ep1_in.MaxPacketSize() == 32U && hw.ep1_in.closes == 1U);
    TEST_ASSERT(hw.ep2_out.configures == 0U);
    hw.ep1_in.shrink_after = -1;
    SetConfiguration(dev, hw, 1U);
    CheckWorks(dev, hw, can0);
  }

  // FD 一次请求装不下 80 字节的帧：槽只有 64 字节；OUT 后端一次只收一个包；IN 后端
  // 一次最多 79 字节。都拒绝配置，两个端点关闭，不挂接收。
  // FD cannot hold an 80-byte frame in one request: 64-byte slots; an OUT backend taking
  // one packet per request; an IN backend taking at most 79 bytes. Each refuses the
  // configuration, both endpoints closed, nothing armed.
  for (int how = 0; how < 3; ++how)
  {
    Hw hw(how == 0 ? 128U : 1024U);
    if (how == 1)
    {
      hw.ep2_out.limits.max_packets = 1U;
    }
    if (how == 2)
    {
      hw.ep1_in.limits = {64U, SIZE_MAX, 79U};
    }
    StubFdCan fd0;
    GsUsbClass<1> gs(EPNumber::EP1, EPNumber::EP2,
                     std::initializer_list<LibXR::FDCAN*>{&fd0});
    FakeUsb dev(hw, {{&gs}});
    dev.Init(false);
    dev.Reset();
    TEST_ASSERT(GetDescriptor(dev, hw, 2U, 0U, 255U).size() == 32U);
    Refused(dev, hw, Make(0x00U, 9U, 1U, 0U, 0U));
    TEST_ASSERT(hw.ep1_in.MaxPacketSize() == 64U && hw.ep2_out.MaxPacketSize() == 64U);
    TEST_ASSERT(hw.ep1_in.closes == 1U && hw.ep2_out.closes == 1U);
    TEST_ASSERT(hw.ep2_out.starts.empty() && hw.ep1_in.starts.empty());
    if (how != 0)
    {
      hw.ep2_out.limits = {};
      hw.ep1_in.limits = {};
      SetConfiguration(dev, hw, 1U);
      TEST_ASSERT(hw.ep2_out.LastLength() == 512U);
      CheckWorks(dev, hw, fd0);
    }
  }

  // 两个 gs_usb 用同一组端点：第二个 Get 失败，两个都解绑；第一个挂的接收还没启动就
  // 以 ABORTED 结束。
  // Two gs_usb on the same endpoints: the second Get fails and both are unbound; the
  // receive the first one armed ends with ABORTED before it starts.
  {
    Hw hw;
    StubCan can0;
    StubCan can1;
    Gs2 first(EPNumber::EP1, EPNumber::EP2, {&can0, &can1});
    Gs2 second(EPNumber::EP1, EPNumber::EP2, {&can0, &can1});
    FakeUsb dev(hw, {{&first, &second}});
    dev.Init(false);
    dev.Reset();
    const Bytes config = GetDescriptor(dev, hw, 2U, 0U, 255U);
    TEST_ASSERT(config.size() == 9U + 2U * 23U && config[4] == 2U);
    Refused(dev, hw, Make(0x00U, 9U, 1U, 0U, 0U));
    TEST_ASSERT(hw.ep1_in.closes == 1U && hw.ep2_out.closes == 1U);
    TEST_ASSERT(hw.ep2_out.starts.empty() && !hw.ep2_out.inflight);
  }

  // Classic CAN 只需 24 B 主机缓冲，64 B MaxTransferSize 必须能绑定成功；FD 需 80 B
  // 必须失败 Classic CAN only requires 24 B and must bind at 64 B; FD requires 80 B and
  // must fail
  {
    Hw hw;
    hw.ep1_in.limits.max_packets = 1U;
    hw.ep2_out.limits.max_packets = 1U;
    StubCan can0;
    StubCan can1;
    Gs2 gs(EPNumber::EP1, EPNumber::EP2, {&can0, &can1});
    FakeUsb dev(hw, {{&gs}});
    Enumerate(dev, hw);
    TEST_ASSERT(hw.ep1_in.MaxTransferSize() == 64U && hw.ep2_out.inflight);
    HostFormat(dev, hw);
    Mode(dev, hw, 0U, Gs::CanMode::START);
    can0.Inject(0x111U, {1U, 2U});
    TEST_ASSERT(hw.ep1_in.inflight);
    Drain(dev, hw.ep1_in);
  }
  {
    Hw hw;
    hw.ep1_in.limits.max_packets = 1U;
    hw.ep2_out.limits.max_packets = 1U;
    StubFdCan fd0;
    GsUsbClass<1> gs(EPNumber::EP1, EPNumber::EP2,
                     std::initializer_list<LibXR::FDCAN*>{&fd0});
    FakeUsb dev(hw, {{&gs}});
    dev.Init(false);
    dev.Reset();
    Refused(dev, hw, Make(0x00U, 9U, 1U, 0U, 0U));
    TEST_ASSERT(hw.ep1_in.closes == 1U && hw.ep2_out.closes == 1U);
  }

  // 端点没有登记：写描述符失败，GET_DESCRIPTOR 和 SET_CONFIGURATION 都被拒。
  // An endpoint is not registered: writing the descriptors fails, both GET_DESCRIPTOR
  // and SET_CONFIGURATION are refused.
  {
    Hw hw;
    StubCan can0;
    StubCan can1;
    Gs2 gs(EPNumber::EP1, EPNumber::EP9, {&can0, &can1});
    FakeUsb dev(hw, {{&gs}});
    dev.Init(false);
    dev.Reset();
    Refused(dev, hw, Make(0x80U, 6U, 0x0200U, 0U, 255U));
    Refused(dev, hw, Make(0x00U, 9U, 1U, 0U, 0U));
    TEST_ASSERT(hw.ep1_in.configures == 0U && hw.ep2_out.configures == 0U);
  }
}

/**
 * 带在途请求解绑：IN 两帧在途、三帧排队；OUT 一个接收在途，或回显队列满、一个槽留在
 * 类里、两个回显排队。四种解绑：配置 0、复位、同一配置再设一次、Deinit。
 * Unbind with requests in flight: two IN frames in flight and three queued; on OUT
 * either a receive in flight, or the echo queue full with one slot kept by the class
 * and two echoes queued. Four ways to unbind: configuration 0, reset, the same
 * configuration again, Deinit.
 */
void TestUnbindInFlight()
{
  for (int held = 0; held < 2; ++held)
  {
    for (int how = 0; how < 4; ++how)
    {
      Fixture f(32U, held != 0 ? 2U : 32U);
      Hw& hw = f.hw;
      f.Start(true, true, Gs::GSCAN_MODE_BERR_REPORTING);
      for (unsigned i = 0; i < 5U; ++i)
      {
        f.can0.Inject(0x100U + i, {static_cast<uint8_t>(i)});
      }
      TEST_ASSERT(hw.ep1_in.inflight && hw.ep1_in.starts.size() == 1U);
      if (held != 0)
      {
        for (unsigned i = 0; i < 3U; ++i)
        {
          HostOut(f.dev, hw.ep2_out, Frame(0x40U + i, 0x40U, 1U, 1U, 0U, {1U}));
        }
        TEST_ASSERT(!hw.ep2_out.inflight && f.can1.tx.size() == 3U);
      }
      TEST_ASSERT(held != 0 || hw.ep2_out.inflight);

      switch (how)
      {
        case 0:
          SetConfiguration(f.dev, hw, 0U);
          break;
        case 1:
          f.dev.Reset();
          break;
        case 2:
          SetConfiguration(f.dev, hw, 1U);
          break;
        default:
          f.dev.Deinit(false);
          break;
      }

      // 两个端点都停过、关过；在途的 IN 没有到主机；通道和协商都清掉。
      // Both endpoints were stopped and closed; the in-flight IN never reached the host;
      // channels and negotiation are cleared.
      TEST_ASSERT(hw.ep1_in.stops == 1U && hw.ep1_in.closes == 1U);
      TEST_ASSERT(hw.ep2_out.stops == (held != 0 ? 0U : 1U));
      TEST_ASSERT(hw.ep2_out.closes == 1U);
      TEST_ASSERT(f.Sent().empty() && !f.gs.IsHostFormatOK());
      f.can0.Inject(0x1FFU, {1U});
      f.can0.Inject(LibXR::CAN::FromErrorID(LibXR::CAN::ErrorID::CAN_ERROR_ID_ACK), {},
                    LibXR::CAN::Type::ERROR);
      if (how != 2)
      {
        TEST_ASSERT(hw.ep1_in.GetState() == Endpoint::State::DISABLED);
        TEST_ASSERT(hw.ep2_out.GetState() == Endpoint::State::DISABLED);
        TEST_ASSERT(!hw.ep1_in.inflight && !hw.ep2_out.inflight);
        if (how == 3)
        {
          f.dev.Init(false);
        }
        if (how != 0)
        {
          f.dev.Reset();
          SetAddress(f.dev, hw);
        }
        SetConfiguration(f.dev, hw, 1U);
      }

      // 重新绑定后：旧绑定的帧和回显都已丢弃，一个接收在途；收发照常，两个 IN 槽都能用，
      // 留住的 OUT 槽已交回（第二个接收能挂上）。
      // After the new bind: frames and echoes of the old binding are gone, one receive
      // is armed; traffic works, both IN slots are usable, and the kept OUT slot was
      // handed back (a second receive can be armed).
      TEST_ASSERT(!hw.ep1_in.inflight && hw.ep2_out.inflight);
      TEST_ASSERT(hw.ep2_out.LastLength() == 512U);
      HostFormat(f.dev, hw);
      f.Start(true, false);
      f.can0.Inject(0x500U, {5U});
      f.can0.Inject(0x501U, {6U});
      TEST_ASSERT(hw.ep1_in.inflight && hw.ep1_in.starts.size() == 2U);
      Drain(f.dev, hw.ep1_in);
      TEST_ASSERT(f.Sent().size() == 2U && U32(f.Sent()[0], 4) == 0x500U);
      TEST_ASSERT(U32(f.Sent()[1], 4) == 0x501U && f.Sent()[1][10] == 0U);
      CheckWorks(f.dev, hw, f.can0);
      TEST_ASSERT(hw.ep2_out.Receive(64U, false) == ErrorCode::OK);
    }
  }
}

/**
 * 两个线程各在一个通道上收 CAN 帧，一个线程扮控制器中断（完成 IN、在 OUT 上送主机
 * 帧），回显队列只有 4：回显一个不少且有序；CAN 帧按通道有序，丢了就有溢出标志。
 * Two threads receive CAN frames on one channel each and one thread plays the controller
 * interrupt (completes IN, sends host frames on OUT); the echo queue holds only 4: every
 * echo arrives, in order; CAN frames arrive in order per channel, and drops show the
 * overflow flag.
 */
void TestThreads()
{
  Fixture f(16U, 4U);
  Hw& hw = f.hw;
  f.Start(true, true);

  constexpr uint32_t RX_FRAMES = 3000U;
  constexpr uint32_t HOST_FRAMES = 2000U;
  std::vector<Bytes> host;
  for (uint32_t i = 0; i < HOST_FRAMES; ++i)
  {
    Bytes data(8U);
    std::memcpy(data.data(), &i, sizeof(i));
    host.push_back(Frame(i, 0x300U, 4U, static_cast<uint8_t>(i & 1U), 0U, data));
  }
  std::atomic<size_t> host_sent{0U};
  std::atomic<bool> stop{false};

  auto step = [&](bool isr)
  {
    if (hw.ep1_in.inflight)
    {
      hw.ep1_in.Done(isr, hw.ep1_in.LastLength());
    }
    const size_t next = host_sent.load(std::memory_order_relaxed);
    if (hw.ep2_out.inflight && next < host.size())
    {
      host_sent.store(next + 1U, std::memory_order_release);
      hw.ep2_out.HostSends(isr, host[next]);
    }
  };
  std::thread interrupt(
      [&]
      {
        while (!stop.load(std::memory_order_acquire))
        {
          f.dev.Irq(step);
          std::this_thread::yield();
        }
      });

  auto producer = [&](StubCan& can)
  {
    for (uint32_t seq = 0; seq < RX_FRAMES; ++seq)
    {
      Bytes data(4U);
      std::memcpy(data.data(), &seq, sizeof(seq));
      can.Inject(0x100U, data);
      if ((seq & 7U) == 0U)
      {
        std::this_thread::yield();
      }
    }
  };
  std::thread can_a(producer, std::ref(f.can0));
  std::thread can_b(producer, std::ref(f.can1));
  can_a.join();
  can_b.join();

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
  while (host_sent.load(std::memory_order_acquire) < host.size())
  {
    TEST_ASSERT(std::chrono::steady_clock::now() < deadline);
    std::this_thread::yield();
  }
  stop.store(true, std::memory_order_release);
  interrupt.join();
  Drain(f.dev, hw.ep1_in);

  // 回显：全部、有序 / Echoes: all of them, in order
  uint32_t next_echo = 0U;
  uint32_t next_rx[2] = {0U, 0U};
  bool gap[2] = {false, false};
  size_t flags[2] = {0U, 0U};
  for (const Bytes& frame : f.Sent())
  {
    const uint8_t ch = frame[9];
    TEST_ASSERT(ch < 2U);
    flags[ch] += (frame[10] & Gs::CAN_FLAG_OVERFLOW) != 0U ? 1U : 0U;
    if (U32(frame, 0) != 0xFFFFFFFFU)
    {
      TEST_ASSERT(U32(frame, 0) == next_echo && U32(frame, 12) == next_echo);
      ++next_echo;
      continue;
    }
    const uint32_t seq = U32(frame, 12);
    TEST_ASSERT(seq >= next_rx[ch]);
    gap[ch] = gap[ch] || seq != next_rx[ch];
    next_rx[ch] = seq + 1U;
  }
  TEST_ASSERT(next_echo == HOST_FRAMES);
  for (int ch = 0; ch < 2; ++ch)
  {
    gap[ch] = gap[ch] || next_rx[ch] != RX_FRAMES;
    TEST_ASSERT(!gap[ch] || flags[ch] != 0U);
  }
  TEST_ASSERT(f.can0.tx.size() == HOST_FRAMES / 2U &&
              f.can1.tx.size() == HOST_FRAMES / 2U);
  for (size_t i = 0; i < f.can0.tx.size(); ++i)
  {
    uint32_t value = 0U;
    std::memcpy(&value, f.can0.tx[i].data, sizeof(value));
    TEST_ASSERT(value == 2U * i);
  }
  TEST_ASSERT(hw.ep2_out.inflight && !f.dev.masked.load());
}

/// 后端可能给的槽数：核心默认（两个）、1、3、8 / Slot counts a backend may give: the core
/// default (two), 1, 3 and 8
constexpr uint8_t SLOT_COUNTS[] = {0U, 1U, 3U, 8U};

/**
 * @brief 默认、1、3、8 个槽下发往主机的帧和回显队列满的反压 / Frames to the host and
 *        the backpressure of a full echo queue with the default, 1, 3 and 8 slots
 * @note 接收帧按序到达，与 IN 槽数无关。回显队列 4：IN 每个槽装一帧回显、队列装 4 个、OUT
 *       再留住一帧，之后主机被 NAK；全部回显按序到达，OUT 恢复接收。
 *       Received frames arrive in order whatever the IN slot count. Echo queue of 4:
 *       every IN slot carries one echo, the queue holds 4 and OUT holds one more frame,
 *       then the host is NAKed; every echo arrives in order and OUT receives again.
 */
void TestSlotCounts()
{
  for (uint8_t slots : SLOT_COUNTS)
  {
    const size_t count = slots == 0U ? 2U : slots;
    {
      Fixture f(32U, 32U, nullptr, slots);
      f.Start(true, false);
      for (unsigned i = 0; i < 12U; ++i)
      {
        f.can0.Inject(0x100U + i, {static_cast<uint8_t>(i)});
      }
      Drain(f.dev, f.hw.ep1_in);
      TEST_ASSERT(f.Sent().size() == 12U);
      for (unsigned i = 0; i < 12U; ++i)
      {
        TEST_ASSERT(U32(f.Sent()[i], 4) == 0x100U + i && f.Sent()[i][12] == i);
      }
    }
    {
      Fixture f(32U, 4U, nullptr, slots);
      f.Start(true, false);
      unsigned sent = 0;
      while (f.hw.ep2_out.inflight && sent < 32U)
      {
        HostOut(f.dev, f.hw.ep2_out,
                Frame(0x2000U + sent, 0x200U + sent, 1U, 0U, 0U, {1U}));
        ++sent;
      }
      TEST_ASSERT(!f.hw.ep2_out.inflight && sent == count + 4U + 1U);
      TEST_ASSERT(f.can0.tx.size() == sent);
      Drain(f.dev, f.hw.ep1_in);
      TEST_ASSERT(f.Sent().size() == sent && f.hw.ep2_out.inflight);
      for (unsigned i = 0; i < sent; ++i)
      {
        TEST_ASSERT(U32(f.Sent()[i], 0) == 0x2000U + i);
      }
    }
  }
}

/// 公开 DeviceClass 的端点辅助函数 / Exposes the endpoint helpers of DeviceClass
class HelperProbe : public DeviceClass
{
 public:
  using DeviceClass::BindEndpoint;
  using DeviceClass::UnbindEndpoint;
  using DeviceClass::WriteEndpointDescriptor;
};

/// 帧尾：包长 8 时，比主机缓冲短、又正好是整数个包的帧后面跟结尾零长度包
/// End of a frame: with 8-byte packets a frame shorter than the host's buffer that is a
/// whole number of packets is followed by the closing zero-length packet
void TestFrameEnd()
{
  // FD 设备：主机缓冲 80 字节 / An FD device: the host's buffer is 80 bytes
  {
    Hw hw;
    hw.ep1_in.limits.max_packet_size = 8U;
    StubFdCan fd0;
    GsUsbClass<1> gs(EPNumber::EP1, EPNumber::EP2,
                     std::initializer_list<LibXR::FDCAN*>{&fd0});
    FakeUsb dev(hw, {{&gs}});
    Enumerate(dev, hw);
    TEST_ASSERT(hw.ep1_in.MaxPacketSize() == 8U);
    HostFormat(dev, hw);
    Mode(dev, hw, 0U, Gs::CanMode::START,
         Gs::GSCAN_MODE_FD | Gs::GSCAN_MODE_HW_TIMESTAMP);
    FakeEp& in = hw.ep1_in;
    // 带时间戳的经典帧 24 字节 = 3 个整包，比 80 短：后面跟零长度包
    // A classic frame with its timestamp is 24 bytes = 3 whole packets, below 80: the
    // zero-length packet follows
    const size_t before = in.starts.size();
    fd0.Inject(0x123U, {1U, 2U});
    TEST_ASSERT(in.inflight && in.LastLength() == 24U);
    InDone(dev, in);
    TEST_ASSERT(in.inflight && in.LastLength() == 0U && in.starts.size() == before + 2U);
    InDone(dev, in);
    TEST_ASSERT(!in.inflight);
    // 带时间戳的 FD 帧 80 字节 = 10 个整包，正好装满主机的缓冲：不跟
    // An FD frame with its timestamp is 80 bytes = 10 whole packets and fills the host's
    // buffer: nothing follows
    fd0.InjectFd(0x124U, Pattern(64, 5));
    TEST_ASSERT(in.inflight && in.LastLength() == 80U);
    InDone(dev, in);
    TEST_ASSERT(!in.inflight && in.starts.size() == before + 3U);
  }
  // 经典设备：主机缓冲 24 字节，24 字节的帧装满它，不跟；不带时间戳的 20 字节不是 8 的
  // 整数倍，也不跟
  // A classic device: the host's buffer is 24 bytes, a frame of 24 fills it and nothing
  // follows; without a timestamp the frame is 20 bytes, no multiple of 8, and nothing
  // follows either
  for (const bool timestamp : {true, false})
  {
    Hw hw;
    hw.ep1_in.limits.max_packet_size = 8U;
    StubCan can0;
    StubCan can1;
    Gs2 gs(EPNumber::EP1, EPNumber::EP2, {&can0, &can1});
    FakeUsb dev(hw, {{&gs}});
    Enumerate(dev, hw);
    TEST_ASSERT(hw.ep1_in.MaxPacketSize() == 8U);
    HostFormat(dev, hw);
    Mode(dev, hw, 0U, Gs::CanMode::START, timestamp ? Gs::GSCAN_MODE_HW_TIMESTAMP : 0U);
    FakeEp& in = hw.ep1_in;
    const size_t before = in.starts.size();
    can0.Inject(0x123U, {1U, 2U});
    TEST_ASSERT(in.inflight && in.LastLength() == (timestamp ? 24U : 20U));
    InDone(dev, in);
    TEST_ASSERT(!in.inflight && in.starts.size() == before + 1U);
  }
}

/**
 * @brief 端点辅助函数按描述符核对包长和包数 / The endpoint helpers check the packet size
 *        and the count against the descriptor
 * @note 高速中断端点，1024 字节包，后端允许一个服务区间 3 个包：描述符写出 3 个包；绑定时
 *       描述符的包数与读回的不同就是 CHECK_ERR，相同才是 OK。
 *       A high-speed interrupt endpoint with 1024-byte packets and a backend that allows
 *       3 packets per service interval: the descriptor says 3 packets; at the bind a
 *       count in the descriptor other than the one read back is CHECK_ERR, only the same
 *       one is OK.
 */
void TestBindHelperCount()
{
  Hw hw(1024U, 1U);
  hw.ep1_in.limits.max_packets_per_interval = 3U;
  StubCan can0;
  StubCan can1;
  Gs2 gs(EPNumber::EP1, EPNumber::EP2, {&can0, &can1});
  FakeUsb dev(hw, {{&gs}}, Speed::HIGH);
  dev.Init(false);
  dev.Reset(Speed::HIGH);

  const Endpoint::Config cfg{Direction::IN, Endpoint::Type::INTR, 1024U, 3U};
  HelperProbe::EndpointDescriptor desc{};
  TEST_ASSERT(HelperProbe::WriteEndpointDescriptor(hw.pool, Speed::HIGH, EPNumber::EP1,
                                                   cfg, 1U, desc));
  const uint16_t three = Endpoint::PacketSizeField(1024U, 3U);
  TEST_ASSERT(desc.bEndpointAddress == 0x81U && desc.bmAttributes == 3U);
  TEST_ASSERT(desc.wMaxPacketSize == three && desc.bInterval == 1U);

  for (const uint8_t packets : {1U, 2U, 3U})
  {
    const uint16_t field = Endpoint::PacketSizeField(1024U, packets);
    Endpoint* ep = nullptr;
    ErrorCode result = ErrorCode::FAILED;
    dev.Irq(
        [&](bool isr)
        {
          result = HelperProbe::BindEndpoint(hw.pool, EPNumber::EP1, cfg, field, isr, ep);
        });
    TEST_ASSERT(ep == &hw.ep1_in && hw.ep1_in.MaxPacketSize() == 1024U);
    TEST_ASSERT(hw.ep1_in.PacketsPerInterval() == 3U);
    TEST_ASSERT(result == (field == three ? ErrorCode::OK : ErrorCode::CHECK_ERR));
    dev.Irq([&](bool isr) { HelperProbe::UnbindEndpoint(hw.pool, ep, isr); });
    TEST_ASSERT(ep == nullptr && hw.ep1_in.GetState() == Endpoint::State::DISABLED);
  }
}

}  // namespace

void test_usb_gsusb_enum() { TestEnumeration(); }
void test_usb_gsusb_requests() { TestRequests(); }
void test_usb_gsusb_host_frames() { TestHostFrames(); }
void test_usb_gsusb_multi_packet() { TestMultiPacket(); }
void test_usb_gsusb_to_host()
{
  TestToHost();
  TestFrameEnd();
}
void test_usb_gsusb_user_id() { TestUserId(); }
void test_usb_gsusb_bind_fail()
{
  TestBindFailure();
  TestBindHelperCount();
}
void test_usb_gsusb_unbind() { TestUnbindInFlight(); }
void test_usb_gsusb_threads() { TestThreads(); }
void test_usb_gsusb_slots() { TestSlotCounts(); }
