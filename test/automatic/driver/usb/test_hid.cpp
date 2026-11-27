/**
 * @file test_hid.cpp
 * @brief HID 类的假后端测试 / Fake-backend tests of the HID classes
 *
 * 用真实的设备核心、组合层、端点和 HID 类；假后端与 test_cdc.cpp 的相同（同时派生
 * DeviceService 和 DeviceCore，测试把硬件事件排进队列再进入控制器中断），另记下每次 IN
 * 启动时的数据和端点被配置的次数。覆盖报告收发和类请求、绑定失败、带在途请求解绑、
 * 中断端点在多线程和配置切换下的发送。
 * Uses the real device core, composition, endpoints and HID classes; the fake backend
 * is the one of test_cdc.cpp (deriving from both DeviceService and DeviceCore, tests
 * queue hardware events and enter the controller interrupt), and it also records the
 * data of every IN start and how often the endpoint was configured. Covered: reports
 * and class requests, bind failure, unbind with requests in flight, and sending on the
 * interrupt endpoint from threads while the configuration changes.
 */

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <deque>
#include <functional>
#include <map>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

#include "dev_core.hpp"
#include "ep.hpp"
#include "ep_pool.hpp"
#include "hid.hpp"
#include "hid_gamepad.hpp"
#include "hid_keyboard.hpp"
#include "hid_mouse.hpp"
#include "test_assert.hpp"
#include "usb_service.hpp"

namespace
{
using LibXR::Callback;
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
using LibXR::USB::HID;
using LibXR::USB::HIDGamepad;
using LibXR::USB::HIDGamepadBipolar;
using LibXR::USB::HIDKeyboard;
using LibXR::USB::HIDMouse;
using LibXR::USB::SetupPacket;
using LibXR::USB::Speed;
using LibXR::USB::USBSpec;
using Direction = Endpoint::Direction;
using EPNumber = Endpoint::EPNumber;
using Bytes = std::vector<uint8_t>;

static_assert(LibXR::HW_CACHE_LINE_SIZE <= 64U, "test memory is aligned to 64 bytes");

constexpr auto LANGUAGE = DescriptorStrings::MakeLanguagePack(
    DescriptorStrings::Language::EN_US, "XRobot", "HID", "0001");

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
    size_t epoch;  ///< 启动时 Configure 成功过几次 / Successful Configures at the start
    Bytes bytes;   ///< IN：启动时的数据 / IN: the data at the start
  };

  /// slots：槽数，0 为核心默认 / slots: slot count, 0 for the core default
  FakeEp(EPNumber number, Direction direction, RawData memory, uint8_t slots = 0U)
      : Endpoint(number, direction, memory, slots), one_slot(slots == 1U)
  {
  }

  /// 硬件完成在途请求；只在 owner 中 / The hardware completes the in-flight request;
  /// owner only
  void Done(bool in_isr, size_t actual, ErrorCode result = ErrorCode::OK)
  {
    TEST_ASSERT(inflight);
    inflight = false;
    if (GetConfig().direction == Direction::IN && result == ErrorCode::OK)
    {
      const Start& start = starts.back();
      wire.push_back({start.epoch, Bytes(start.data, start.data + actual)});
    }
    OnHardwareComplete(in_isr, actual, result);
  }

  /// 主机发来 OUT 数据；只在 owner 中 / The host sends OUT data; owner only
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
  const Bytes& LastBytes() const { return starts.back().bytes; }

  std::vector<Start> starts;
  /// 主机取走的 IN 请求：(epoch, 字节) / IN requests the host took: (epoch, bytes)
  std::vector<std::pair<size_t, Bytes>> wire;
  bool inflight = false;
  size_t configures = 0;  ///< 成功的 ConfigureHardware / Successful ConfigureHardware
  size_t closes = 0;
  size_t stops = 0;
  size_t stalls = 0;
  size_t clears = 0;
  ErrorCode configure_result = ErrorCode::OK;
  bool complete_in_start = false;  ///< IN 在 StartHardware 内完成 / IN completes inside
  /// HardwareLimits 第几次调用之后报 32 字节包 / After this many HardwareLimits calls,
  /// report 32-byte packets
  int shrink_after = -1;
  mutable int limit_calls = 0;
  /// 只有一个槽：硬件不直接读写槽（核心对单槽的要求）/ One slot only: the hardware
  /// does not access the slot directly (what the core requires of a single slot)
  bool one_slot;

 protected:
  ErrorCode CheckHardware(const Config&) const override { return configure_result; }

  void ConfigureHardware(const Config&) override { ++configures; }

  void CloseHardware(bool) override { ++closes; }

  ErrorCode StartHardware(const HwRequest& request, bool in_isr) override
  {
    TEST_ASSERT(!inflight);
    Start start{request.length, request.data, configures, {}};
    if (GetConfig().direction == Direction::IN)
    {
      start.bytes.assign(request.data, request.data + request.length);
    }
    starts.push_back(std::move(start));
    inflight = true;
    if (complete_in_start && GetConfig().direction == Direction::IN)
    {
      Done(in_isr, request.length);
    }
    return ErrorCode::OK;
  }

  StopOutcome StopHardware(bool) override
  {
    ++stops;
    inflight = false;
    return {ErrorCode::ABORTED, 0U};
  }

  ErrorCode StallHardware(bool) override
  {
    ++stalls;
    return ErrorCode::OK;
  }

  ErrorCode ClearStallHardware(bool) override
  {
    ++clears;
    return ErrorCode::OK;
  }

  HwLimits HardwareLimits(Direction, Type) const override
  {
    ++limit_calls;
    HwLimits limits{};
    if (shrink_after >= 0 && limit_calls > shrink_after)
    {
      limits = {32U, SIZE_MAX, SIZE_MAX};
    }
    limits.direct_memory = !one_slot;
    return limits;
  }
};

/**
 * @brief 假端点和端点池：EP1 IN、EP2 OUT（各 1024 字节），EP3 IN（128 字节，两个 64 字节
 *        槽）/ Fake endpoints and the pool: EP1 IN, EP2 OUT (1024 bytes each), EP3 IN
 *        (128 bytes, two 64-byte slots)
 * @note 槽数只用于 EP1、EP2。 / The slot count applies to EP1 and EP2 only.
 */
struct Hw
{
  /// data_slots：EP1、EP2 的槽数，0 为核心默认 / data_slots: slot count of EP1 and EP2,
  /// 0 for the core default
  explicit Hw(uint8_t data_slots = 0U) : slots(data_slots)
  {
    pool.SetEndpoint0(&in0, &out0);
    TEST_ASSERT(pool.Put(&ep1_in) == ErrorCode::OK);
    TEST_ASSERT(pool.Put(&ep2_out) == ErrorCode::OK);
    TEST_ASSERT(pool.Put(&ep3_in) == ErrorCode::OK);
  }

  uint8_t slots;
  Memory mem[5];
  FakeEp in0{EPNumber::EP0, Direction::IN, {mem[0].bytes, 64U}};
  FakeEp out0{EPNumber::EP0, Direction::OUT, {mem[1].bytes, 64U}};
  FakeEp ep1_in{EPNumber::EP1, Direction::IN, {mem[2].bytes, 1024U}, slots};
  FakeEp ep2_out{EPNumber::EP2, Direction::OUT, {mem[3].bytes, 1024U}, slots};
  FakeEp ep3_in{EPNumber::EP3, Direction::IN, {mem[4].bytes, 128U}};
  EndpointPool pool;

  std::vector<FakeEp*> All() { return {&in0, &out0, &ep1_in, &ep2_out, &ep3_in}; }
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

  /// 在硬件事件处理中当场报告 SETUP；只在 owner 中 / Report a SETUP at once from
  /// the hardware event handling; owner only
  void SetupNow(bool isr, SetupPacket setup) { OnSetup(isr, setup); }

  void Reset(Speed speed = Speed::FULL)
  {
    Irq([this, speed](bool isr) { OnBusReset(isr, speed); });
  }

  bool Inited() const { return IsInited(); }
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
    TEST_ASSERT(guard < 10000U);
    InDone(dev, ep);
  }
}

/// 主机在 OUT 端点上发一个传输 / The host sends one transfer on an OUT endpoint
void HostOut(FakeUsb& dev, FakeEp& ep, const Bytes& bytes)
{
  TEST_ASSERT(ep.inflight);
  dev.Irq([&ep, bytes](bool isr) { ep.HostSends(isr, bytes); });
}

void StatusOut(FakeUsb& dev, Hw& hw)
{
  dev.Irq([&hw](bool isr) { hw.out0.HostSends(isr, {}); });
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
  StatusOut(dev, hw);
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

void Refused(FakeUsb& dev, Hw& hw, SetupPacket setup)
{
  const size_t ready = dev.ready;
  dev.Setup(setup);
  TEST_ASSERT(Stalled(hw));
  TEST_ASSERT(dev.ready == ready);
}

/// SETUP 被接受，数据阶段之后 STALL / SETUP accepted, stalled after the data stage
void RefusedData(FakeUsb& dev, Hw& hw, SetupPacket setup, const Bytes& data)
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

/// 接口为接收者的 GET_DESCRIPTOR / GET_DESCRIPTOR with the interface as recipient
SetupPacket InterfaceDescriptor(uint8_t type, uint8_t index, uint16_t length)
{
  return Make(0x81U, 6U, static_cast<uint16_t>(type << 8U | index), 0U, length);
}

void SetConfiguration(FakeUsb& dev, Hw& hw, uint8_t value)
{
  ControlNone(dev, hw, Make(0x00U, 9U, value, 0U, 0U));
}

/// Init、复位、编址、配置 1 / Init, reset, address, configuration 1
void Enumerate(FakeUsb& dev, Hw& hw, Speed speed = Speed::FULL)
{
  dev.Init(false);
  dev.Reset(speed);
  TEST_ASSERT(dev.Inited());
  ControlNone(dev, hw, Make(0x00U, 5U, 7U, 0U, 0U));
  SetConfiguration(dev, hw, 1U);
}

void SetHalt(FakeUsb& dev, Hw& hw, uint8_t ep_addr)
{
  ControlNone(dev, hw, Make(0x02U, 3U, 0U, ep_addr, 0U));
}

void ClearHalt(FakeUsb& dev, Hw& hw, uint8_t ep_addr)
{
  ControlNone(dev, hw, Make(0x02U, 1U, 0U, ep_addr, 0U));
}

// ---- HID 请求 / HID requests ----

constexpr uint8_t GET_REPORT = 0x01U;
constexpr uint8_t GET_IDLE = 0x02U;
constexpr uint8_t GET_PROTOCOL = 0x03U;
constexpr uint8_t SET_REPORT = 0x09U;
constexpr uint8_t SET_IDLE = 0x0AU;
constexpr uint8_t SET_PROTOCOL = 0x0BU;
constexpr uint8_t INPUT = 1U;
constexpr uint8_t OUTPUT = 2U;
constexpr uint8_t FEATURE = 3U;

uint16_t Report(uint8_t type, uint8_t id)
{
  return static_cast<uint16_t>(type << 8U | id);
}

// ---- 测试用的类 / Classes under test ----

constexpr uint8_t TEST_REPORT_DESC[8] = {0x05, 0x01, 0x09, 0x02, 0xA1, 0x01, 0xC0, 0x00};

/// 通用 HID：记下钩子的调用；特征报告 5 可读写 / Generic HID: records the hook calls;
/// feature report 5 can be read and written
class TestHid : public HID
{
 public:
  TestHid(EPNumber in, EPNumber out, size_t in_size = 8U, size_t out_size = 2U)
      : HID(in, out, in_size, out_size, 10U, 10U)
  {
  }

  using HID::GetInEndpoint;
  using HID::GetOutEndpoint;

  size_t in_completions = 0;
  ErrorCode last_in_result = ErrorCode::OK;
  size_t out_reports = 0;
  Bytes last_out;
  Bytes feature{0x11U, 0x22U, 0x33U};
  size_t feature_sets = 0;

 protected:
  ConstRawData GetReportDesc() const override
  {
    return {TEST_REPORT_DESC, sizeof(TEST_REPORT_DESC)};
  }

  void OnDataInComplete(bool, const Endpoint::Completion& completion) override
  {
    ++in_completions;
    last_in_result = completion.result;
  }

  ErrorCode OnOutReport(bool, ConstRawData report) override
  {
    ++out_reports;
    const auto* data = static_cast<const uint8_t*>(report.addr_);
    last_out.assign(data, data + report.size_);
    return ErrorCode::OK;
  }

  ErrorCode OnGetReport(bool in_isr, ReportType type, uint8_t id,
                        ConstRawData& data) override
  {
    if (type == ReportType::FEATURE && id == 5U)
    {
      data = {feature.data(), feature.size()};
      return ErrorCode::OK;
    }
    return HID::OnGetReport(in_isr, type, id, data);
  }

  ErrorCode OnSetReport(bool in_isr, ReportType type, uint8_t id,
                        ConstRawData data) override
  {
    if (type == ReportType::FEATURE && id == 5U)
    {
      ++feature_sets;
      const auto* bytes = static_cast<const uint8_t*>(data.addr_);
      feature.assign(bytes, bytes + data.size_);
      return ErrorCode::OK;
    }
    return HID::OnSetReport(in_isr, type, id, data);
  }
};

/// 键盘，另数 IN 完成 / Keyboard that also counts IN completions
class TestKeyboard : public HIDKeyboard
{
 public:
  using HIDKeyboard::HIDKeyboard;
  size_t in_completions = 0;

 protected:
  void OnDataInComplete(bool, const Endpoint::Completion&) override { ++in_completions; }
};

struct LedSeen
{
  unsigned calls = 0;
  bool num = false;
  bool caps = false;
  bool scroll = false;
};

Callback<bool, bool, bool> LedCallback(LedSeen& seen)
{
  return Callback<bool, bool, bool>::Create(
      [](bool, LedSeen* s, bool n, bool c, bool sc)
      {
        ++s->calls;
        s->num = n;
        s->caps = c;
        s->scroll = sc;
      },
      &seen);
}

Bytes Iota(size_t size, uint8_t seed)
{
  Bytes bytes(size);
  for (size_t i = 0; i < size; ++i)
  {
    bytes[i] = static_cast<uint8_t>(seed + i);
  }
  return bytes;
}

uint16_t Word(const Bytes& b, size_t at)
{
  return static_cast<uint16_t>(b[at] | (b[at + 1U] << 8U));
}

// ---- 用例 / Cases ----

/// 描述符、字符串、绑定 / Descriptors, strings, binding
void TestEnumeration()
{
  // 键盘带 OUT：接口 3/1/1，两个端点；设备描述符的类字段为 0（按接口）。
  // Keyboard with OUT: interface 3/1/1, two endpoints; the device descriptor's class
  // fields are 0 (per interface).
  {
    Hw hw;
    HIDKeyboard keyboard(EPNumber::EP1, EPNumber::EP2, true);
    FakeUsb dev(hw, {{&keyboard}}, Speed::HIGH);
    dev.Init(false);
    dev.Reset(Speed::FULL);

    const Bytes device = GetDescriptor(dev, hw, 1U, 0U, 18U);
    TEST_ASSERT(device.size() == 18U && device[4] == 0U && device[5] == 0U &&
                device[6] == 0U);

    const Bytes fs = GetDescriptor(dev, hw, 2U, 0U, 255U);
    TEST_ASSERT(fs.size() == 41U && Word(fs, 2) == 41U && fs[4] == 1U);
    TEST_ASSERT(fs[9] == 9U && fs[10] == 4U && fs[11] == 0U && fs[12] == 0U);
    TEST_ASSERT(fs[13] == 2U && fs[14] == 3U && fs[15] == 1U && fs[16] == 1U);
    TEST_ASSERT(fs[17] == 4U);
    TEST_ASSERT(fs[18] == 9U && fs[19] == 0x21U && Word(fs, 20) == 0x0111U);
    TEST_ASSERT(fs[22] == 0U && fs[23] == 1U && fs[24] == 0x22U && Word(fs, 25) == 63U);
    TEST_ASSERT(fs[27] == 7U && fs[28] == 5U && fs[29] == 0x81U && fs[30] == 3U);
    TEST_ASSERT(Word(fs, 31) == 8U && fs[33] == 1U);
    TEST_ASSERT(fs[34] == 7U && fs[35] == 5U && fs[36] == 0x02U && fs[37] == 3U);
    TEST_ASSERT(Word(fs, 38) == 1U && fs[40] == 1U);

    // 高速下包长不变（8、1 都合法）/ The same sizes at high speed (8 and 1 are legal)
    const Bytes hs = GetDescriptor(dev, hw, 7U, 0U, 255U);
    TEST_ASSERT(hs.size() == 41U && hs[1] == 7U && Word(hs, 31) == 8U &&
                Word(hs, 38) == 1U);

    // 接口字符串是 4 号 / The interface string is index 4
    const Bytes name = GetDescriptor(dev, hw, 3U, 4U, 255U, 0x0409U);
    TEST_ASSERT(name.size() == 2U + 2U * 9U && name[2] == 'X' && name[2 + 2 * 6] == 'H');

    // 未配置时没有接口，报告描述符被拒；写描述符不碰端点。
    // Unconfigured there is no interface, so the report descriptor is refused; writing
    // descriptors touches no endpoint.
    Refused(dev, hw, InterfaceDescriptor(0x22U, 0U, 255U));
    TEST_ASSERT(hw.ep1_in.configures == 0U && hw.ep2_out.configures == 0U);

    ControlNone(dev, hw, Make(0x00U, 5U, 3U, 0U, 0U));
    SetConfiguration(dev, hw, 1U);
    TEST_ASSERT(hw.ep1_in.MaxPacketSize() == 8U && hw.ep2_out.MaxPacketSize() == 1U);
    TEST_ASSERT(hw.ep2_out.inflight && hw.ep2_out.LastLength() == 1U);
    TEST_ASSERT(!hw.ep1_in.inflight && hw.ep1_in.starts.empty());

    // 报告描述符 63 字节；HID 描述符与配置描述符里的那份相同；按 wLength 截短。
    // The report descriptor is 63 bytes; the HID descriptor equals the copy in the
    // configuration descriptor; truncated to wLength.
    const Bytes report = ControlIn(dev, hw, InterfaceDescriptor(0x22U, 0U, 255U));
    TEST_ASSERT(report.size() == 63U && report[0] == 0x05U && report[3] == 0x06U &&
                report[62] == 0xC0U);
    const Bytes hid = ControlIn(dev, hw, InterfaceDescriptor(0x21U, 0U, 255U));
    TEST_ASSERT(hid == Bytes(fs.begin() + 18, fs.begin() + 27));
    TEST_ASSERT(ControlIn(dev, hw, InterfaceDescriptor(0x22U, 0U, 4U)).size() == 4U);
    Refused(dev, hw, InterfaceDescriptor(0x22U, 1U, 255U));
    Refused(dev, hw, InterfaceDescriptor(0x23U, 0U, 255U));
  }

  // 鼠标：接口 3/1/2，一个端点，34 字节；手柄：3/0/0，报告描述符 50 字节。
  // Mouse: interface 3/1/2, one endpoint, 34 bytes; gamepad: 3/0/0, report descriptor
  // 50 bytes.
  {
    Hw hw;
    HIDMouse mouse(EPNumber::EP1, 4U);
    FakeUsb dev(hw, {{&mouse}});
    dev.Init(false);
    dev.Reset();
    const Bytes fs = GetDescriptor(dev, hw, 2U, 0U, 255U);
    TEST_ASSERT(fs.size() == 34U && fs[13] == 1U && fs[15] == 1U && fs[16] == 2U);
    TEST_ASSERT(Word(fs, 25) == 52U && Word(fs, 31) == 4U && fs[33] == 4U);
    SetConfiguration(dev, hw, 1U);
    TEST_ASSERT(ControlIn(dev, hw, InterfaceDescriptor(0x22U, 0U, 255U)).size() == 52U);
  }
  {
    Hw hw;
    HIDGamepadBipolar pad(EPNumber::EP1);
    FakeUsb dev(hw, {{&pad}});
    Enumerate(dev, hw);
    const Bytes fs = GetDescriptor(dev, hw, 2U, 0U, 255U);
    TEST_ASSERT(fs[15] == 0U && fs[16] == 0U && Word(fs, 25) == 50U);
    TEST_ASSERT(Word(fs, 31) == 9U);
    const Bytes report = ControlIn(dev, hw, InterfaceDescriptor(0x22U, 0U, 255U));
    TEST_ASSERT(report.size() == 50U && report[20] == 0x16U &&
                Word(report, 21) == 0xF800U);
    TEST_ASSERT(report[23] == 0x26U && Word(report, 24) == 0x07FFU);
  }

  // 100 字节的报告：全速包长 64，高速 100；bInterval 原样写；OUT 一次接收取整到包长。
  // 100-byte reports: packet 64 at full speed, 100 at high speed; bInterval written as
  // given; one OUT receive rounded up to the packet size.
  {
    Hw hw;
    TestHid hid(EPNumber::EP1, EPNumber::EP2, 100U, 100U);
    FakeUsb dev(hw, {{&hid}}, Speed::HIGH);
    dev.Init(false);
    dev.Reset(Speed::HIGH);
    const Bytes hs = GetDescriptor(dev, hw, 2U, 0U, 255U);
    TEST_ASSERT(hs.size() == 41U && hs[15] == 0U && hs[16] == 0U);
    TEST_ASSERT(Word(hs, 31) == 100U && hs[33] == 10U && Word(hs, 38) == 100U &&
                hs[40] == 10U);
    const Bytes other = GetDescriptor(dev, hw, 7U, 0U, 255U);
    TEST_ASSERT(Word(other, 31) == 64U && Word(other, 38) == 64U && other[33] == 10U);
    ControlNone(dev, hw, Make(0x00U, 5U, 3U, 0U, 0U));
    SetConfiguration(dev, hw, 1U);
    TEST_ASSERT(hw.ep2_out.LastLength() == 100U);

    dev.Reset(Speed::FULL);
    ControlNone(dev, hw, Make(0x00U, 5U, 3U, 0U, 0U));
    SetConfiguration(dev, hw, 1U);
    TEST_ASSERT(hw.ep1_in.MaxPacketSize() == 64U && hw.ep2_out.LastLength() == 128U);
    // 一条 100 字节的输入报告是一个请求 / One 100-byte input report is one request
    const Bytes report = Iota(100U, 3U);
    TEST_ASSERT(hid.SendInputReport({report.data(), report.size()}, false) ==
                ErrorCode::OK);
    TEST_ASSERT(hw.ep1_in.LastLength() == 100U && hw.ep1_in.LastBytes() == report);
    // 一条 100 字节的输出报告（两个包）只交给类一次 / One 100-byte output report (two
    // packets) reaches the class once
    HostOut(dev, hw.ep2_out, Iota(100U, 9U));
    TEST_ASSERT(hid.out_reports == 1U && hid.last_out == Iota(100U, 9U));
  }
}

/// 类请求 / Class requests
void TestRequests()
{
  Hw hw;
  TestHid hid(EPNumber::EP1, EPNumber::EP2);
  FakeUsb dev(hw, {{&hid}});
  Enumerate(dev, hw);

  // SET_IDLE 存空闲速率（wValue 高字节，报告号在低字节），GET_IDLE 以一个字节返回。
  // SET_IDLE stores the idle rate (high byte of wValue, report ID in the low byte) and
  // GET_IDLE returns it as one byte.
  TEST_ASSERT(ControlIn(dev, hw, Make(0xA1U, GET_IDLE, 0U, 0U, 1U)) == Bytes{0U});
  ControlNone(dev, hw, Make(0x21U, SET_IDLE, 0x7D00U, 0U, 0U));
  TEST_ASSERT(ControlIn(dev, hw, Make(0xA1U, GET_IDLE, 0U, 0U, 1U)) == Bytes{0x7DU});
  TEST_ASSERT(hid.GetIdleRate() == 0x7DU);
  ControlNone(dev, hw, Make(0x21U, SET_IDLE, 0x0000U, 0U, 0U));
  TEST_ASSERT(hid.GetIdleRate() == 0U);
  Refused(dev, hw, Make(0x21U, SET_IDLE, 0x7D01U, 0U, 0U));
  Refused(dev, hw, Make(0xA1U, GET_IDLE, 1U, 0U, 1U));
  Refused(dev, hw, Make(0xA1U, GET_IDLE, 0U, 0U, 2U));
  Refused(dev, hw, Make(0x21U, GET_IDLE, 0U, 0U, 1U));

  // GET/SET_PROTOCOL：默认报告协议（1）；只接受 0 和 1。
  // GET/SET_PROTOCOL: report protocol (1) by default; only 0 and 1 are accepted.
  TEST_ASSERT(ControlIn(dev, hw, Make(0xA1U, GET_PROTOCOL, 0U, 0U, 1U)) == Bytes{1U});
  ControlNone(dev, hw, Make(0x21U, SET_PROTOCOL, 0U, 0U, 0U));
  TEST_ASSERT(ControlIn(dev, hw, Make(0xA1U, GET_PROTOCOL, 0U, 0U, 1U)) == Bytes{0U});
  Refused(dev, hw, Make(0x21U, SET_PROTOCOL, 2U, 0U, 0U));
  Refused(dev, hw, Make(0xA1U, GET_PROTOCOL, 0U, 0U, 2U));

  // GET_REPORT：默认应答为空；特征报告 5 由派生类给出；类型 0、4 和错误方向被拒。
  // GET_REPORT: the default reply is empty; feature report 5 comes from the derived
  // class; types 0 and 4 and the wrong direction are refused.
  TEST_ASSERT(
      ControlIn(dev, hw, Make(0xA1U, GET_REPORT, Report(INPUT, 0U), 0U, 8U)).empty());
  TEST_ASSERT(ControlIn(dev, hw, Make(0xA1U, GET_REPORT, Report(FEATURE, 5U), 0U, 8U)) ==
              (Bytes{0x11U, 0x22U, 0x33U}));
  TEST_ASSERT(ControlIn(dev, hw, Make(0xA1U, GET_REPORT, Report(FEATURE, 5U), 0U, 2U)) ==
              (Bytes{0x11U, 0x22U}));
  Refused(dev, hw, Make(0xA1U, GET_REPORT, Report(0U, 0U), 0U, 8U));
  Refused(dev, hw, Make(0xA1U, GET_REPORT, Report(4U, 0U), 0U, 8U));
  Refused(dev, hw, Make(0x21U, GET_REPORT, Report(INPUT, 0U), 0U, 8U));

  // SET_REPORT：特征报告进 OnSetReport；输出报告与中断 OUT 进同一个 OnOutReport；
  // 没有数据阶段或方向错误被拒；派生类不认的类型在数据阶段之后 STALL。
  // SET_REPORT: a feature report goes to OnSetReport; an output report goes to the
  // OnOutReport the interrupt OUT uses; no data stage or the wrong direction is
  // refused; a type the derived class does not take stalls after the data stage.
  ControlOut(dev, hw, Make(0x21U, SET_REPORT, Report(FEATURE, 5U), 0U, 2U),
             {0x44U, 0x55U});
  TEST_ASSERT(hid.feature_sets == 1U && hid.feature == (Bytes{0x44U, 0x55U}));
  ControlOut(dev, hw, Make(0x21U, SET_REPORT, Report(OUTPUT, 0U), 0U, 2U),
             {0x66U, 0x77U});
  TEST_ASSERT(hid.out_reports == 1U && hid.last_out == (Bytes{0x66U, 0x77U}));
  Refused(dev, hw, Make(0x21U, SET_REPORT, Report(OUTPUT, 0U), 0U, 0U));
  Refused(dev, hw, Make(0xA1U, SET_REPORT, Report(OUTPUT, 0U), 0U, 2U));
  Refused(dev, hw, Make(0x21U, SET_REPORT, Report(5U, 0U), 0U, 2U));
  RefusedData(dev, hw, Make(0x21U, SET_REPORT, Report(INPUT, 0U), 0U, 1U), {0x01U});

  // 其他：端点为接收者、厂商请求、未知请求都被拒。
  // Others: the endpoint as recipient, a vendor request and an unknown request are all
  // refused.
  Refused(dev, hw, Make(0xA2U, GET_REPORT, Report(INPUT, 0U), 0x81U, 8U));
  Refused(dev, hw, Make(0xC1U, GET_REPORT, Report(INPUT, 0U), 0U, 8U));
  Refused(dev, hw, Make(0xA1U, 0x05U, 0U, 0U, 1U));

  // 重新绑定：协议回到 1，空闲速率回到 0。
  // A new bind: the protocol returns to 1 and the idle rate to 0.
  ControlNone(dev, hw, Make(0x21U, SET_IDLE, 0x2000U, 0U, 0U));
  SetConfiguration(dev, hw, 1U);
  TEST_ASSERT(ControlIn(dev, hw, Make(0xA1U, GET_PROTOCOL, 0U, 0U, 1U)) == Bytes{1U});
  TEST_ASSERT(hid.GetIdleRate() == 0U);

  // 鼠标不收输出报告：SET_REPORT(Output) 在数据阶段之后 STALL。
  // A mouse takes no output report: SET_REPORT(Output) stalls after the data stage.
  {
    Hw mhw;
    HIDMouse mouse(EPNumber::EP1);
    FakeUsb mdev(mhw, {{&mouse}});
    Enumerate(mdev, mhw);
    RefusedData(mdev, mhw, Make(0x21U, SET_REPORT, Report(OUTPUT, 0U), 0U, 1U), {0x01U});
    TEST_ASSERT(ControlIn(mdev, mhw, Make(0xA1U, GET_PROTOCOL, 0U, 0U, 1U)) == Bytes{1U});
  }

  // 键盘 LED 经 SET_REPORT(Output)（没有 OUT 端点时的唯一路径）。
  // The keyboard LED through SET_REPORT(Output) (the only path without an OUT endpoint).
  {
    Hw khw;
    HIDKeyboard keyboard(EPNumber::EP1, EPNumber::EP2);
    LedSeen led;
    keyboard.SetOnLedChangeCallback(LedCallback(led));
    FakeUsb kdev(khw, {{&keyboard}});
    Enumerate(kdev, khw);
    TEST_ASSERT(khw.ep2_out.configures == 0U && khw.ep2_out.starts.empty());
    ControlOut(kdev, khw, Make(0x21U, SET_REPORT, Report(OUTPUT, 0U), 0U, 1U), {0x04U});
    TEST_ASSERT(led.calls == 1U && !led.num && !led.caps && led.scroll);
    TEST_ASSERT(keyboard.GetScrollLock() && !keyboard.GetNumLock());
    Refused(kdev, khw, Make(0x21U, SET_REPORT, Report(OUTPUT, 0U), 0U, 65U));
  }
}

/// 输入报告 / Input reports
void TestInput()
{
  // 通用 HID：参数检查，未绑定 FAILED，两个槽：一条在途一条排队，第三条 BUSY。
  // Generic HID: argument checks, FAILED while unbound, two slots: one in flight and
  // one queued, the third is BUSY.
  {
    Hw hw;
    TestHid hid(EPNumber::EP1, EPNumber::EP_INVALID, 8U, 0U);
    FakeUsb dev(hw, {{&hid}});
    Bytes report = Iota(8U, 0x40U);
    TEST_ASSERT(hid.SendInputReport({report.data(), 8U}, false) == ErrorCode::FAILED);
    Enumerate(dev, hw);
    TEST_ASSERT(hid.GetInEndpoint() == &hw.ep1_in && hid.GetOutEndpoint() == nullptr);
    TEST_ASSERT(hid.SendInputReport({nullptr, 8U}, false) == ErrorCode::ARG_ERR);
    TEST_ASSERT(hid.SendInputReport({report.data(), 0U}, false) == ErrorCode::ARG_ERR);
    TEST_ASSERT(hid.SendInputReport({report.data(), 9U}, false) == ErrorCode::ARG_ERR);
    TEST_ASSERT(hw.ep1_in.starts.empty());

    TEST_ASSERT(hid.SendInputReport({report.data(), 8U}, false) == ErrorCode::OK);
    TEST_ASSERT(hw.ep1_in.inflight && hw.ep1_in.LastBytes() == report);
    const Bytes second = Iota(5U, 0x60U);
    TEST_ASSERT(hid.SendInputReport({second.data(), 5U}, true) == ErrorCode::OK);
    TEST_ASSERT(hid.SendInputReport({report.data(), 8U}, false) == ErrorCode::BUSY);
    TEST_ASSERT(hw.ep1_in.starts.size() == 1U);
    // 调用者改了自己的缓冲不影响已提交的报告 / The caller changing its buffer does not
    // affect a submitted report
    report[0] = 0xEEU;

    InDone(dev, hw.ep1_in);
    TEST_ASSERT(hid.in_completions == 1U && hid.last_in_result == ErrorCode::OK);
    TEST_ASSERT(hw.ep1_in.inflight && hw.ep1_in.LastBytes() == second);
    TEST_ASSERT(hid.SendInputReport({report.data(), 8U}, false) == ErrorCode::OK);
    Drain(dev, hw.ep1_in);
    TEST_ASSERT(hid.in_completions == 3U && hw.ep1_in.wire.size() == 3U);
    TEST_ASSERT(hw.ep1_in.wire[0].second == Iota(8U, 0x40U));
    TEST_ASSERT(hw.ep1_in.wire[1].second == second && hw.ep1_in.wire[2].second == report);

    // 后端在 StartHardware 内完成：每次发送都在返回前完成。
    // The backend completes inside StartHardware: each send completes before it returns.
    hw.ep1_in.complete_in_start = true;
    for (uint8_t i = 0; i < 100U; ++i)
    {
      const Bytes one(8U, i);
      TEST_ASSERT(hid.SendInputReport({one.data(), one.size()}, (i & 1U) != 0U) ==
                  ErrorCode::OK);
      TEST_ASSERT(hw.ep1_in.wire.back().second == one && !hw.ep1_in.inflight);
    }
    TEST_ASSERT(hid.in_completions == 103U);
    hw.ep1_in.complete_in_start = false;

    // 主机 STALL IN：在途报告 ABORTED（已绑定，回调照常）；STALLED 期间的发送排队，
    // CLEAR_FEATURE 后发出。
    // The host halts IN: the in-flight report ends ABORTED (still bound, the callback
    // runs); sends while STALLED are queued and go out after CLEAR_FEATURE.
    TEST_ASSERT(hid.SendInputReport({report.data(), 8U}, false) == ErrorCode::OK);
    SetHalt(dev, hw, 0x81U);
    TEST_ASSERT(hid.in_completions == 104U && hid.last_in_result == ErrorCode::ABORTED);
    TEST_ASSERT(hw.ep1_in.stalls == 1U && !hw.ep1_in.inflight);
    const size_t starts = hw.ep1_in.starts.size();
    TEST_ASSERT(hid.SendInputReport({second.data(), 5U}, false) == ErrorCode::OK);
    TEST_ASSERT(hw.ep1_in.starts.size() == starts);
    ClearHalt(dev, hw, 0x81U);
    TEST_ASSERT(hw.ep1_in.inflight && hw.ep1_in.LastBytes() == second);
    Drain(dev, hw.ep1_in);

    // 解绑后再发不崩溃、返回 FAILED；重新配置后槽数全部可用 / After unbind returns
    // FAILED without crashing and restores all slots on rebind
    SetConfiguration(dev, hw, 0U);
    TEST_ASSERT(hid.SendInputReport({report.data(), 8U}, false) == ErrorCode::FAILED);
    SetConfiguration(dev, hw, 1U);
    TEST_ASSERT(hid.SendInputReport({report.data(), 8U}, false) == ErrorCode::OK);
    Drain(dev, hw.ep1_in);
  }

  // 键盘：按下、六键、幻影状态、松开 / Keyboard: press, six keys, phantom, release
  {
    Hw hw;
    HIDKeyboard keyboard(EPNumber::EP1, EPNumber::EP2, true);
    FakeUsb dev(hw, {{&keyboard}});
    Enumerate(dev, hw);
    using Key = HIDKeyboard::KeyCode;
    const auto code = [](Key key) { return static_cast<uint8_t>(key); };
    TEST_ASSERT(keyboard.PressKey({Key::A, Key::B}, HIDKeyboard::LEFT_SHIFT) ==
                ErrorCode::OK);
    TEST_ASSERT(hw.ep1_in.LastBytes() == (Bytes{HIDKeyboard::LEFT_SHIFT, 0, code(Key::A),
                                                code(Key::B), 0, 0, 0, 0}));
    InDone(dev, hw.ep1_in);
    TEST_ASSERT(keyboard.PressKey({Key::A, Key::B, Key::C, Key::D, Key::E, Key::F},
                                  HIDKeyboard::LEFT_CTRL) == ErrorCode::OK);
    TEST_ASSERT(hw.ep1_in.LastBytes() ==
                (Bytes{HIDKeyboard::LEFT_CTRL, 0, code(Key::A), code(Key::B),
                       code(Key::C), code(Key::D), code(Key::E), code(Key::F)}));
    InDone(dev, hw.ep1_in);
    TEST_ASSERT(
        keyboard.PressKey({Key::A, Key::B, Key::C, Key::D, Key::E, Key::F, Key::G},
                          HIDKeyboard::LEFT_SHIFT, true) == ErrorCode::OK);
    TEST_ASSERT(hw.ep1_in.LastBytes() ==
                (Bytes{HIDKeyboard::LEFT_SHIFT, 0, 1, 1, 1, 1, 1, 1}));
    InDone(dev, hw.ep1_in);
    // 按下紧接松开：两条都提交（两个槽），松开不会丢。
    // Press followed at once by release: both are submitted (two slots), the release
    // is not lost.
    TEST_ASSERT(keyboard.PressKey({Key::Z}) == ErrorCode::OK);
    TEST_ASSERT(keyboard.ReleaseAll() == ErrorCode::OK);
    Drain(dev, hw.ep1_in);
    TEST_ASSERT(hw.ep1_in.wire.back().second == Bytes(8U, 0U));
    TEST_ASSERT(hw.ep1_in.wire[hw.ep1_in.wire.size() - 2U].second[2] == code(Key::Z));
  }

  // 鼠标和手柄 / Mouse and gamepad
  {
    Hw hw;
    HIDMouse mouse(EPNumber::EP1);
    FakeUsb dev(hw, {{&mouse}});
    Enumerate(dev, hw);
    TEST_ASSERT(hw.ep1_in.MaxPacketSize() == 4U);
    TEST_ASSERT(mouse.Move(HIDMouse::LEFT, 5, -3, 1) == ErrorCode::OK);
    TEST_ASSERT(hw.ep1_in.LastBytes() == (Bytes{0x01U, 0x05U, 0xFDU, 0x01U}));
    InDone(dev, hw.ep1_in);
    TEST_ASSERT(mouse.Release() == ErrorCode::OK);
    TEST_ASSERT(hw.ep1_in.LastBytes() == Bytes(4U, 0U));
    InDone(dev, hw.ep1_in);
  }
  {
    Hw hw;
    HIDGamepad pad(EPNumber::EP1);
    HIDGamepadBipolar bipolar(EPNumber::EP3);
    FakeUsb dev(hw, {{&pad, &bipolar}});
    Enumerate(dev, hw);
    TEST_ASSERT(hw.ep1_in.MaxPacketSize() == 9U && hw.ep3_in.MaxPacketSize() == 9U);
    // 单极：-100 夹到 0，3000 夹到 2047 / Unipolar: -100 clamps to 0, 3000 to 2047
    TEST_ASSERT(pad.Send(100, -100, 3000, 1024, 0x03U) == ErrorCode::OK);
    TEST_ASSERT(hw.ep1_in.LastBytes() ==
                (Bytes{100, 0, 0, 0, 0xFF, 0x07, 0x00, 0x04, 0x03}));
    TEST_ASSERT(pad.SendButtons(0x80U) == ErrorCode::OK);
    TEST_ASSERT(pad.SendAxes(1, 2, 3, 4) == ErrorCode::BUSY);
    InDone(dev, hw.ep1_in);
    TEST_ASSERT(hw.ep1_in.LastBytes() ==
                (Bytes{100, 0, 0, 0, 0xFF, 0x07, 0x00, 0x04, 0x80}));
    // BUSY 那次改了轴：下一次只改按键也带着它们 / The BUSY call changed the axes: the
    // next buttons-only send carries them
    TEST_ASSERT(pad.SendButtons(0x01U) == ErrorCode::OK);
    Drain(dev, hw.ep1_in);
    TEST_ASSERT(hw.ep1_in.wire.back().second == (Bytes{1, 0, 2, 0, 3, 0, 4, 0, 0x01}));
    // 双极：中点 0，-3000 夹到 -2048 / Bipolar: midpoint 0, -3000 clamps to -2048
    TEST_ASSERT(bipolar.SendButtons(0x02U) == ErrorCode::OK);
    TEST_ASSERT(hw.ep3_in.LastBytes() == (Bytes{0, 0, 0, 0, 0, 0, 0, 0, 0x02}));
    InDone(dev, hw.ep3_in);
    TEST_ASSERT(bipolar.Send(-3000, 0, 0, 0, 0U) == ErrorCode::OK);
    TEST_ASSERT(hw.ep3_in.LastBytes() == (Bytes{0x00, 0xF8, 0, 0, 0, 0, 0, 0, 0}));
    InDone(dev, hw.ep3_in);
  }
}

/// 输出报告 / Output reports
void TestOutput()
{
  Hw hw;
  TestHid hid(EPNumber::EP1, EPNumber::EP2);
  FakeUsb dev(hw, {{&hid}});
  Enumerate(dev, hw);
  TEST_ASSERT(hid.GetOutEndpoint() == &hw.ep2_out);
  TEST_ASSERT(hw.ep2_out.MaxPacketSize() == 2U && hw.ep2_out.MaxTransferSize() == 512U);
  TEST_ASSERT(hw.ep2_out.inflight && hw.ep2_out.LastLength() == 2U);

  // 每条报告交给类之后槽交回、重新挂接收 / After each report the slot goes back and a
  // receive is armed again
  HostOut(dev, hw.ep2_out, {0x5AU, 0xA5U});
  TEST_ASSERT(hid.out_reports == 1U && hid.last_out == (Bytes{0x5AU, 0xA5U}));
  TEST_ASSERT(hw.ep2_out.inflight && hw.ep2_out.starts.size() == 2U);
  HostOut(dev, hw.ep2_out, {0x01U});
  TEST_ASSERT(hid.out_reports == 2U && hid.last_out == Bytes{0x01U});
  for (int i = 0; i < 50; ++i)
  {
    HostOut(dev, hw.ep2_out, {static_cast<uint8_t>(i), 0x00U});
  }
  TEST_ASSERT(hid.out_reports == 52U && hw.ep2_out.starts.size() == 53U);

  // 硬件错误：不交给类，照样重挂 / A hardware error: nothing reaches the class, the
  // receive is armed again
  dev.Irq([&hw](bool isr) { hw.ep2_out.Done(isr, 0U, ErrorCode::FAILED); });
  TEST_ASSERT(hid.out_reports == 52U && hw.ep2_out.inflight);
  TEST_ASSERT(hw.ep2_out.starts.size() == 54U);

  // 主机 STALL OUT：接收被中止并重新排队，不启动；CLEAR_FEATURE 后启动。
  // The host halts OUT: the receive is aborted and queued again without starting;
  // CLEAR_FEATURE starts it.
  SetHalt(dev, hw, 0x02U);
  TEST_ASSERT(hw.ep2_out.stops == 1U && hw.ep2_out.stalls == 1U && !hw.ep2_out.inflight);
  TEST_ASSERT(hw.ep2_out.starts.size() == 54U);
  ClearHalt(dev, hw, 0x02U);
  TEST_ASSERT(hw.ep2_out.inflight && hw.ep2_out.starts.size() == 55U);
  HostOut(dev, hw.ep2_out, {0x10U, 0x20U});
  TEST_ASSERT(hid.out_reports == 53U && hid.last_out == (Bytes{0x10U, 0x20U}));

  // 键盘 LED 经 OUT 端点 / Keyboard LED through the OUT endpoint
  {
    Hw khw;
    HIDKeyboard keyboard(EPNumber::EP1, EPNumber::EP2, true);
    LedSeen led;
    keyboard.SetOnLedChangeCallback(LedCallback(led));
    FakeUsb kdev(khw, {{&keyboard}});
    Enumerate(kdev, khw);
    HostOut(kdev, khw.ep2_out, {0x03U});
    TEST_ASSERT(led.calls == 1U && led.num && led.caps && !led.scroll);
    TEST_ASSERT(keyboard.GetNumLock() && keyboard.GetCapsLock() &&
                !keyboard.GetScrollLock());
    TEST_ASSERT(khw.ep2_out.inflight && khw.ep2_out.starts.size() == 2U);
    // 零长度的 OUT 包不改 LED / A zero-length OUT packet leaves the LEDs alone
    HostOut(kdev, khw.ep2_out, {});
    TEST_ASSERT(led.calls == 1U && keyboard.GetCapsLock() && khw.ep2_out.inflight);
  }
}

/// 绑定失败 / Bind failure
void TestBindFailure()
{
  // OUT 端点 Configure 失败：IN 已配置，Unbind 关闭并交回；发送 FAILED；修好后可用。
  // The OUT endpoint's Configure fails: IN was configured, Unbind closes and returns it;
  // sends get FAILED; fixed, it works.
  {
    Hw hw;
    hw.ep2_out.configure_result = ErrorCode::FAILED;
    HIDKeyboard keyboard(EPNumber::EP1, EPNumber::EP2, true);
    FakeUsb dev(hw, {{&keyboard}});
    dev.Init(false);
    dev.Reset();
    Refused(dev, hw, Make(0x00U, 9U, 1U, 0U, 0U));
    TEST_ASSERT(hw.ep1_in.configures == 1U && hw.ep1_in.closes == 1U);
    TEST_ASSERT(hw.ep2_out.configures == 0U && hw.ep2_out.closes == 0U);
    TEST_ASSERT(hw.ep1_in.GetState() == Endpoint::State::DISABLED);
    TEST_ASSERT(hw.ep2_out.starts.empty() && hw.ep1_in.starts.empty());
    TEST_ASSERT(keyboard.ReleaseAll() == ErrorCode::FAILED);
    TEST_ASSERT(ControlIn(dev, hw, Make(0x80U, 8U, 0U, 0U, 1U)) == Bytes{0U});
    hw.ep2_out.configure_result = ErrorCode::OK;
    SetConfiguration(dev, hw, 1U);
    TEST_ASSERT(keyboard.ReleaseAll() == ErrorCode::OK && hw.ep1_in.inflight);
    TEST_ASSERT(hw.ep2_out.inflight);
  }

  // 读回的包长与描述符不同：CHECK_ERR；之后正常。
  // The size read back differs from the descriptor: CHECK_ERR; normal afterwards.
  {
    Hw hw;
    TestHid hid(EPNumber::EP1, EPNumber::EP_INVALID, 64U, 0U);
    FakeUsb dev(hw, {{&hid}});
    dev.Init(false);
    dev.Reset();
    hw.ep1_in.shrink_after = hw.ep1_in.limit_calls + 1;
    Refused(dev, hw, Make(0x00U, 9U, 1U, 0U, 0U));
    TEST_ASSERT(hw.ep1_in.MaxPacketSize() == 32U && hw.ep1_in.closes == 1U);
    const Bytes report(64U, 0x5U);
    TEST_ASSERT(hid.SendInputReport({report.data(), 64U}, false) == ErrorCode::FAILED);
    hw.ep1_in.shrink_after = -1;
    SetConfiguration(dev, hw, 1U);
    TEST_ASSERT(hw.ep1_in.MaxPacketSize() == 64U);
    TEST_ASSERT(hid.SendInputReport({report.data(), 64U}, false) == ErrorCode::OK);
  }

  // 一次请求装不下最长报告（EP3 的槽 64 字节，报告 100）：描述符照写（包长 64），
  // SET_CONFIGURATION 以 NO_BUFF 被拒。
  // One request cannot hold the longest report (EP3 slots are 64 bytes, the report is
  // 100): the descriptors are still written (packet 64), SET_CONFIGURATION is refused
  // with NO_BUFF.
  {
    Hw hw;
    TestHid hid(EPNumber::EP3, EPNumber::EP_INVALID, 100U, 0U);
    FakeUsb dev(hw, {{&hid}});
    dev.Init(false);
    dev.Reset();
    const Bytes fs = GetDescriptor(dev, hw, 2U, 0U, 255U);
    TEST_ASSERT(fs.size() == 34U && Word(fs, 31) == 64U);
    Refused(dev, hw, Make(0x00U, 9U, 1U, 0U, 0U));
    TEST_ASSERT(hw.ep3_in.configures == 1U && hw.ep3_in.closes == 1U);
    TEST_ASSERT(hw.ep3_in.MaxTransferSize() == 64U);
  }

  // 两个 HID 用同一个 IN 端点：第二个 Get 失败，两个都解绑。
  // Two HIDs on the same IN endpoint: the second Get fails and both are unbound.
  {
    Hw hw;
    HIDMouse first(EPNumber::EP1);
    HIDKeyboard second(EPNumber::EP1, EPNumber::EP2, true);
    FakeUsb dev(hw, {{&first, &second}});
    dev.Init(false);
    dev.Reset();
    const Bytes config = GetDescriptor(dev, hw, 2U, 0U, 255U);
    TEST_ASSERT(config.size() == 9U + 25U + 32U && config[4] == 2U);
    Refused(dev, hw, Make(0x00U, 9U, 1U, 0U, 0U));
    TEST_ASSERT(hw.ep1_in.configures == 1U && hw.ep1_in.closes == 1U);
    TEST_ASSERT(hw.ep2_out.configures == 0U);
    TEST_ASSERT(first.Release() == ErrorCode::FAILED);
  }

  // OUT 端点没有登记：写描述符失败，GET_DESCRIPTOR 和 SET_CONFIGURATION 都被拒。
  // The OUT endpoint is not registered: writing the descriptors fails, both
  // GET_DESCRIPTOR and SET_CONFIGURATION are refused.
  {
    Hw hw;
    TestHid hid(EPNumber::EP1, EPNumber::EP9);
    FakeUsb dev(hw, {{&hid}});
    dev.Init(false);
    dev.Reset();
    Refused(dev, hw, Make(0x80U, 6U, 0x0200U, 0U, 255U));
    Refused(dev, hw, Make(0x00U, 9U, 1U, 0U, 0U));
    TEST_ASSERT(hw.ep1_in.configures == 0U);
  }
}

/**
 * 带在途请求解绑：一条报告在途、一条排队，OUT 接收在途。四种解绑：配置 0、复位、同一配置
 * 再设一次、Deinit。
 * Unbind with requests in flight: one report in flight and one queued, an OUT receive
 * in flight. Four ways to unbind: configuration 0, reset, the same configuration again,
 * Deinit.
 */
void TestUnbindInFlight()
{
  using Key = HIDKeyboard::KeyCode;
  for (int how = 0; how < 4; ++how)
  {
    Hw hw;
    TestKeyboard keyboard(EPNumber::EP1, EPNumber::EP2, true);
    FakeUsb dev(hw, {{&keyboard}});
    Enumerate(dev, hw);
    HostOut(dev, hw.ep2_out, {0x02U});
    TEST_ASSERT(keyboard.GetCapsLock() && hw.ep2_out.inflight);
    TEST_ASSERT(keyboard.PressKey({Key::A}) == ErrorCode::OK);
    TEST_ASSERT(keyboard.ReleaseAll() == ErrorCode::OK);
    TEST_ASSERT(keyboard.ReleaseAll() == ErrorCode::BUSY);
    TEST_ASSERT(hw.ep1_in.inflight && hw.ep1_in.starts.size() == 1U);
    const size_t out_starts = hw.ep2_out.starts.size();

    switch (how)
    {
      case 0:
        SetConfiguration(dev, hw, 0U);
        break;
      case 1:
        dev.Reset();
        break;
      case 2:
        SetConfiguration(dev, hw, 1U);
        break;
      default:
        dev.Deinit(false);
        break;
    }

    // 两个端点都停过、关过；主机没收到报告；解绑中的完成不进类的回调。
    // Both endpoints were stopped and closed; the host got no report; completions
    // during the unbind do not reach the class's callback.
    TEST_ASSERT(hw.ep1_in.stops == 1U && hw.ep1_in.closes == 1U);
    TEST_ASSERT(hw.ep2_out.stops == 1U && hw.ep2_out.closes == 1U);
    TEST_ASSERT(hw.ep1_in.wire.empty() && hw.ep1_in.starts.size() == 1U);
    TEST_ASSERT(keyboard.in_completions == 0U);
    if (how != 2)
    {
      TEST_ASSERT(hw.ep2_out.starts.size() == out_starts);
      TEST_ASSERT(keyboard.PressKey({Key::B}) == ErrorCode::FAILED);
      for (FakeEp* ep : {&hw.ep1_in, &hw.ep2_out})
      {
        TEST_ASSERT(ep->GetState() == Endpoint::State::DISABLED && !ep->inflight);
      }
      if (how == 3)
      {
        dev.Init(false);
      }
      if (how != 0)
      {
        dev.Reset();
        ControlNone(dev, hw, Make(0x00U, 5U, 7U, 0U, 0U));
      }
      SetConfiguration(dev, hw, 1U);
    }

    // 重新绑定后：两个 IN 槽都能用，OUT 已挂好；LED 状态保留到主机再发。
    // After the new bind: both IN slots work and OUT is armed; the LED state stays
    // until the host sends a new one.
    TEST_ASSERT(hw.ep2_out.inflight && keyboard.GetCapsLock());
    TEST_ASSERT(keyboard.PressKey({Key::C}) == ErrorCode::OK);
    TEST_ASSERT(keyboard.ReleaseAll() == ErrorCode::OK);
    TEST_ASSERT(keyboard.ReleaseAll() == ErrorCode::BUSY);
    Drain(dev, hw.ep1_in);
    TEST_ASSERT(hw.ep1_in.wire.size() == 2U && keyboard.in_completions == 2U);
    TEST_ASSERT(hw.ep1_in.wire[0].second[2] == static_cast<uint8_t>(Key::C));
    HostOut(dev, hw.ep2_out, {0x01U});
    TEST_ASSERT(keyboard.GetNumLock() && !keyboard.GetCapsLock());
  }
}

/**
 * 中断端点在多线程下的发送：三个线程经 HID A 发送（一个以 in_isr=true），一个线程经 HID B
 * 发送；A 在配置 1，B 在配置 2，两者用同一个 EP1 IN。中断线程每次进入中断时完成在途的
 * IN、给 A 的 OUT 送数据，每 7 次切换一次配置。检查：每条启动的报告都属于启动那一刻绑定
 * 端点的类；每个发送者在主机一侧的序号递增，且都是返回 OK 的；报告字节完整。
 * Sending on the interrupt endpoint from threads: three threads send through HID A
 * (one with in_isr=true) and one through HID B; A is in configuration 1 and B in
 * configuration 2, both on EP1 IN. On every interrupt the interrupt thread completes
 * the IN in flight, feeds A's OUT, and every seventh time switches the configuration.
 * Checked: every report started belongs to the class bound to the endpoint at that
 * moment; each sender's sequence on the host side increases and contains only reports
 * that returned OK; the report bytes are intact.
 */
void TestThreads()
{
  Hw hw;
  TestHid a(EPNumber::EP1, EPNumber::EP2);
  TestHid b(EPNumber::EP1, EPNumber::EP_INVALID, 8U, 0U);
  FakeUsb dev(hw, {{&a}, {&b}});
  Enumerate(dev, hw);

  constexpr size_t PER_SENDER = 1500U;
  constexpr uint8_t TAG_A = 0xA1U;
  constexpr uint8_t TAG_B = 0xB2U;
  size_t steps = 0;           // 只在 owner 中 / Owner only
  uint8_t configuration = 1;  // 只在 owner 中 / Owner only
  bool control_open = false;  // 只在 owner 中 / Owner only
  size_t switches = 0;        // 只在 owner 中 / Owner only
  std::atomic<bool> stop{false};
  std::atomic<unsigned> done{0U};

  auto step = [&](bool isr)
  {
    ++steps;
    if (control_open && hw.in0.inflight)
    {
      // 上一次 SET_CONFIGURATION 的状态阶段 / Status stage of the last
      // SET_CONFIGURATION
      hw.in0.Done(isr, 0U);
      control_open = false;
    }
    if (hw.ep1_in.inflight)
    {
      hw.ep1_in.Done(isr, hw.ep1_in.LastLength());
    }
    if (hw.ep2_out.inflight && steps % 3U == 0U)
    {
      hw.ep2_out.HostSends(isr, {static_cast<uint8_t>(steps), 0x00U});
    }
    if (!control_open && steps % 7U == 0U)
    {
      configuration = configuration == 1U ? 2U : 1U;
      dev.SetupNow(isr, Make(0x00U, 9U, configuration, 0U, 0U));
      control_open = true;
      ++switches;
    }
  };

  std::thread interrupt(
      [&]
      {
        while (!stop.load(std::memory_order_acquire))
        {
          dev.Irq(step);
          std::this_thread::yield();
        }
      });

  std::vector<uint32_t> ok[4];
  auto sender = [&](uint8_t id, TestHid& hid, uint8_t tag, bool in_isr)
  {
    for (uint32_t seq = 0; ok[id].size() < PER_SENDER; ++seq)
    {
      uint8_t report[8] = {
          tag, id, 0, 0, 0, 0, static_cast<uint8_t>(~id), static_cast<uint8_t>(tag ^ id)};
      std::memcpy(report + 2, &seq, sizeof(seq));
      const ErrorCode result = hid.SendInputReport({report, sizeof(report)}, in_isr);
      TEST_ASSERT(result == ErrorCode::OK || result == ErrorCode::BUSY ||
                  result == ErrorCode::FAILED);
      if (result == ErrorCode::OK)
      {
        ok[id].push_back(seq);
      }
      else
      {
        std::this_thread::yield();
      }
    }
    done.fetch_add(1U, std::memory_order_release);
  };
  std::thread s0(sender, 0U, std::ref(a), TAG_A, false);
  std::thread s1(sender, 1U, std::ref(a), TAG_A, true);
  std::thread s2(sender, 2U, std::ref(a), TAG_A, false);
  std::thread s3(sender, 3U, std::ref(b), TAG_B, false);
  s0.join();
  s1.join();
  s2.join();
  s3.join();
  stop.store(true, std::memory_order_release);
  interrupt.join();
  TEST_ASSERT(done.load() == 4U);
  Drain(dev, hw.ep1_in);
  TEST_ASSERT(!dev.masked.load());

  // 每次启动的报告属于当时绑定的类：A 在奇数次配置（配置 1）下，B 在偶数次下。
  // Every report started belongs to the class bound at that moment: A under an odd
  // configure count (configuration 1), B under an even one.
  size_t a_starts = 0;
  size_t b_starts = 0;
  for (const FakeEp::Start& start : hw.ep1_in.starts)
  {
    TEST_ASSERT(start.bytes.size() == 8U);
    const uint8_t tag = start.bytes[0];
    TEST_ASSERT(tag == TAG_A || tag == TAG_B);
    TEST_ASSERT((start.epoch % 2U == 1U) == (tag == TAG_A));
    (tag == TAG_A ? a_starts : b_starts)++;
  }

  // 主机一侧：字节完整，每个发送者的序号递增且都返回过 OK。
  // Host side: bytes intact, each sender's sequence increases and every one returned
  // OK.
  std::map<uint8_t, size_t> next_index;
  for (const auto& entry : hw.ep1_in.wire)
  {
    const Bytes& r = entry.second;
    TEST_ASSERT(r.size() == 8U);
    const uint8_t id = r[1];
    TEST_ASSERT(id < 4U && r[6] == static_cast<uint8_t>(~id) &&
                r[7] == static_cast<uint8_t>(r[0] ^ id));
    TEST_ASSERT((id == 3U) == (r[0] == TAG_B));
    uint32_t seq = 0;
    std::memcpy(&seq, r.data() + 2, sizeof(seq));
    size_t& at = next_index[id];
    while (at < ok[id].size() && ok[id][at] != seq)
    {
      ++at;
    }
    TEST_ASSERT(at < ok[id].size());
    ++at;
  }
  TEST_ASSERT(switches > 2U && a_starts > 0U && b_starts > 0U);
}

/// 后端可能给的槽数：核心默认（两个）、1、3、8 / Slot counts a backend may give: the core
/// default (two), 1, 3 and 8
constexpr uint8_t SLOT_COUNTS[] = {0U, 1U, 3U, 8U};

/**
 * @brief 默认、1、3、8 个槽下的输入报告反压和输出报告 / Input-report backpressure and
 *        output reports with the default, 1, 3 and 8 slots
 * @note 每个 IN 槽装一条报告，再多一条得 BUSY；主机取走后按序到达。OUT 照常收。
 *       Every IN slot carries one report and one more gets BUSY; taken by the host they
 *       arrive in order. OUT receives as usual.
 */
void TestSlotCounts()
{
  for (uint8_t slots : SLOT_COUNTS)
  {
    Hw hw(slots);
    TestHid hid(EPNumber::EP1, EPNumber::EP2);
    FakeUsb dev(hw, {{&hid}});
    Enumerate(dev, hw);
    size_t accepted = 0;
    while (true)
    {
      TEST_ASSERT(accepted < 32U);
      const Bytes report = Iota(8U, static_cast<uint8_t>(accepted));
      if (hid.SendInputReport({report.data(), report.size()}, false) != ErrorCode::OK)
      {
        break;
      }
      ++accepted;
    }
    TEST_ASSERT(accepted == (slots == 0U ? 2U : slots));
    Drain(dev, hw.ep1_in);
    TEST_ASSERT(hw.ep1_in.wire.size() == accepted);
    for (size_t i = 0; i < accepted; ++i)
    {
      TEST_ASSERT(hw.ep1_in.wire[i].second == Iota(8U, static_cast<uint8_t>(i)));
    }
    HostOut(dev, hw.ep2_out, {0x5AU, 0xA5U});
    TEST_ASSERT(hid.out_reports == 1U && hw.ep2_out.inflight);
  }
}

}  // namespace

void test_usb_hid_enum() { TestEnumeration(); }
void test_usb_hid_requests() { TestRequests(); }
void test_usb_hid_in() { TestInput(); }
void test_usb_hid_out() { TestOutput(); }
void test_usb_hid_bind_fail() { TestBindFailure(); }
void test_usb_hid_unbind() { TestUnbindInFlight(); }
void test_usb_hid_threads() { TestThreads(); }
void test_usb_hid_slots() { TestSlotCounts(); }
