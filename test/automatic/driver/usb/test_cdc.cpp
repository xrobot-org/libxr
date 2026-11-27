/**
 * @file test_cdc.cpp
 * @brief CDC 类的假后端测试 / Fake-backend tests of the CDC classes
 *
 * 用真实的设备核心、组合层、端点和 CDC 类；假后端设备同时派生 DeviceService 和
 * DeviceCore，测试把"硬件事件"排进它的队列再调用控制器中断入口。假端点记录每次启动，由
 * 测试决定何时完成。主机一侧由辅助函数扮演。覆盖收发、线路编码、绑定失败、带在途请求
 * 解绑和多线程。
 * Uses the real device core, composition, endpoints and CDC classes; the fake backend
 * device derives from both DeviceService and DeviceCore, and a test queues "hardware
 * events" and calls the controller interrupt entry. Fake endpoints record every start
 * and the test decides when it completes. Helpers play the host. Covered: transfers,
 * line coding, bind failure, unbind with requests in flight and threads.
 */

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <deque>
#include <functional>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

#include "cdc_test.hpp"
#include "cdc_to_uart.hpp"
#include "cdc_uart.hpp"
#include "dev_core.hpp"
#include "ep.hpp"
#include "ep_pool.hpp"
#include "libxr_pipe.hpp"
#include "test_assert.hpp"
#include "usb_service.hpp"

namespace
{
using LibXR::Callback;
using LibXR::ConstRawData;
using LibXR::ErrorCode;
using LibXR::RawData;
using LibXR::ReadOperation;
using LibXR::UART;
using LibXR::WriteOperation;
using LibXR::USB::CDCReadTest;
using LibXR::USB::CDCToUart;
using LibXR::USB::CDCUart;
using LibXR::USB::CDCWriteTest;
using LibXR::USB::DescriptorStrings;
using LibXR::USB::DeviceClass;
using LibXR::USB::DeviceCore;
using LibXR::USB::DeviceDescriptor;
using LibXR::USB::DeviceService;
using LibXR::USB::Endpoint;
using LibXR::USB::EndpointPool;
using LibXR::USB::SetupPacket;
using LibXR::USB::Speed;
using LibXR::USB::USBSpec;
using Direction = Endpoint::Direction;
using EPNumber = Endpoint::EPNumber;
using Bytes = std::vector<uint8_t>;
using PollStatus = ReadOperation::OperationPollingStatus;

static_assert(LibXR::HW_CACHE_LINE_SIZE <= 64U, "test memory is aligned to 64 bytes");

constexpr auto LANGUAGE = DescriptorStrings::MakeLanguagePack(
    DescriptorStrings::Language::EN_US, "XRobot", "CDC", "0001");

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
      const uint8_t* data = starts.back().data;
      wire.insert(wire.end(), data, data + actual);
      wire_bytes.store(wire.size(), std::memory_order_release);
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

  std::vector<size_t> Lengths() const
  {
    std::vector<size_t> lengths;
    for (const Start& start : starts)
    {
      lengths.push_back(start.length);
    }
    return lengths;
  }

  std::vector<Start> starts;
  Bytes wire;  ///< 主机取走的 IN 字节 / IN bytes the host took
  std::atomic<size_t> wire_bytes{0U};
  bool inflight = false;
  size_t configures = 0;
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
    starts.push_back({request.length, request.data});
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
 * @brief 假端点和端点池：EP1 IN 数据，EP2 OUT 数据，EP3 IN 通知 / Fake endpoints and the
 *        pool: EP1 IN data, EP2 OUT data, EP3 IN notification
 * @note 槽数只用于两个数据端点，通知端点 128 字节保持默认的两个槽。
 *       The slot count applies to the two data endpoints only; the 128-byte notification
 *       endpoint keeps the default two slots.
 */
struct Hw
{
  /// data_slots：数据端点的槽数，0 为核心默认 / data_slots: slot count of the data
  /// endpoints, 0 for the core default
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

/// 主机在 OUT 端点上发一个包 / The host sends one packet on an OUT endpoint
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

/// Init、复位、编址、配置 1 / Init, reset, address, configuration 1
void Enumerate(FakeUsb& dev, Hw& hw, Speed speed = Speed::FULL)
{
  dev.Init(false);
  dev.Reset(speed);
  TEST_ASSERT(dev.Inited());
  ControlNone(dev, hw, Make(0x00U, 5U, 7U, 0U, 0U));
  SetConfiguration(dev, hw, 1U);
}

// ---- CDC 请求 / CDC requests ----

constexpr uint8_t SET_LINE_CODING = 0x20U;
constexpr uint8_t GET_LINE_CODING = 0x21U;
constexpr uint8_t SET_CONTROL_LINE_STATE = 0x22U;
constexpr uint8_t SEND_BREAK = 0x23U;

/// CDC PSTN 6.3.11 的 7 字节线路编码 / The 7-byte line coding of CDC PSTN 6.3.11
Bytes Coding(uint32_t baud, uint8_t char_format, uint8_t parity, uint8_t data_bits)
{
  Bytes bytes(7);
  std::memcpy(bytes.data(), &baud, sizeof(baud));
  bytes[4] = char_format;
  bytes[5] = parity;
  bytes[6] = data_bits;
  return bytes;
}

Bytes GetLineCoding(FakeUsb& dev, Hw& hw)
{
  return ControlIn(dev, hw, Make(0xA1U, GET_LINE_CODING, 0U, 0U, 7U));
}

void SetLineCoding(FakeUsb& dev, Hw& hw, const Bytes& coding)
{
  ControlOut(dev, hw, Make(0x21U, SET_LINE_CODING, 0U, 0U, 7U), coding);
}

void SetLineState(FakeUsb& dev, Hw& hw, uint16_t state)
{
  ControlNone(dev, hw, Make(0x21U, SET_CONTROL_LINE_STATE, state, 0U, 0U));
}

/// 在途的 SERIAL_STATE 通知 / The SERIAL_STATE notification in flight
Bytes Notification(Hw& hw)
{
  TEST_ASSERT(hw.ep3_in.inflight);
  const FakeEp::Start& start = hw.ep3_in.starts.back();
  return Bytes(start.data, start.data + start.length);
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

/// 非阻塞写，期望立即接纳 / Non-blocking write that must be admitted at once
void Write(CDCUart& cdc, const Bytes& bytes)
{
  WriteOperation op;
  TEST_ASSERT(cdc.Write({bytes.data(), bytes.size()}, op) == ErrorCode::OK);
}

/// 读出读端口里现有的全部字节 / Read every byte the read port holds now
Bytes ReadAll(CDCUart& cdc)
{
  const size_t size = cdc.read_port_->Size();
  Bytes bytes(size);
  if (size != 0U)
  {
    std::atomic<PollStatus> status{PollStatus::READY};
    ReadOperation op(status);
    TEST_ASSERT(cdc.Read({bytes.data(), size}, op) == ErrorCode::OK);
    TEST_ASSERT(status.load() == PollStatus::DONE);
  }
  return bytes;
}

// ---- 用例 / Cases ----

/// 描述符、字符串、绑定 / Descriptors, strings, binding
void TestEnumeration()
{
  Hw hw;
  CDCUart cdc(EPNumber::EP1, EPNumber::EP2, EPNumber::EP3);
  FakeUsb dev(hw, {{&cdc}}, Speed::HIGH);
  dev.Init(false);
  dev.Reset(Speed::FULL);

  // 全速：IAD、两个接口、四个功能描述符、三个端点，共 75 字节。
  // Full speed: IAD, two interfaces, four functional descriptors, three endpoints, 75
  // bytes in all.
  const Bytes fs = GetDescriptor(dev, hw, 2U, 0U, 255U);
  TEST_ASSERT(fs.size() == 75U && fs[2] == 75U && fs[4] == 2U);
  TEST_ASSERT(fs[9] == 8U && fs[10] == 0x0BU && fs[11] == 0U && fs[12] == 2U);
  TEST_ASSERT(fs[13] == 0x02U && fs[14] == 0x02U);
  TEST_ASSERT(fs[17] == 9U && fs[19] == 0U && fs[21] == 1U && fs[25] == 4U);
  TEST_ASSERT(fs[26] == 5U && fs[27] == 0x24U && fs[28] == 0x00U && fs[29] == 0x10U);
  TEST_ASSERT(fs[31] == 5U && fs[33] == 0x01U && fs[35] == 1U);
  TEST_ASSERT(fs[36] == 4U && fs[38] == 0x02U && fs[39] == 0x02U);
  TEST_ASSERT(fs[40] == 5U && fs[42] == 0x06U && fs[43] == 0U && fs[44] == 1U);
  TEST_ASSERT(fs[47] == 0x83U && fs[48] == 3U && fs[49] == 16U && fs[51] == 4U);
  TEST_ASSERT(fs[52] == 9U && fs[54] == 1U && fs[56] == 2U && fs[57] == 0x0AU);
  TEST_ASSERT(fs[60] == 5U);
  TEST_ASSERT(fs[63] == 0x02U && fs[64] == 2U && fs[65] == 64U && fs[66] == 0U);
  TEST_ASSERT(fs[70] == 0x81U && fs[71] == 2U && fs[72] == 64U && fs[73] == 0U);

  // 高速的包长 512，通知仍 16 / High speed: 512, the notification stays 16
  const Bytes hs = GetDescriptor(dev, hw, 7U, 0U, 255U);
  TEST_ASSERT(hs.size() == 75U && hs[1] == 7U);
  TEST_ASSERT(hs[65] == 0U && hs[66] == 2U && hs[72] == 0U && hs[73] == 2U);
  TEST_ASSERT(hs[49] == 16U);

  // 接口字符串是 4、5 号 / The interface strings are indices 4 and 5
  const Bytes control = GetDescriptor(dev, hw, 3U, 4U, 255U, 0x0409U);
  TEST_ASSERT(control.size() == 2U + 2U * 17U && control[2] == 'X');
  const Bytes data = GetDescriptor(dev, hw, 3U, 5U, 255U, 0x0409U);
  TEST_ASSERT(data.size() == 2U + 2U * 14U && data[2 + 2 * 10] == 'D');

  // 写描述符不碰端点；SET_CONFIGURATION 绑定并挂好两个接收，第一个开始。
  // Writing descriptors touches no endpoint; SET_CONFIGURATION binds and arms two
  // receives, the first one starts.
  TEST_ASSERT(hw.ep1_in.configures == 0U && hw.ep2_out.configures == 0U);
  ControlNone(dev, hw, Make(0x00U, 5U, 3U, 0U, 0U));
  SetConfiguration(dev, hw, 1U);
  TEST_ASSERT(hw.ep1_in.MaxPacketSize() == 64U && hw.ep2_out.MaxPacketSize() == 64U);
  TEST_ASSERT(hw.ep3_in.MaxPacketSize() == 16U);
  TEST_ASSERT(hw.ep2_out.inflight && hw.ep2_out.Lengths() == std::vector<size_t>{64U});
  TEST_ASSERT(!hw.ep1_in.inflight && !hw.ep3_in.inflight);

  // 高速复位后再配置：包长 512 / Configured again after a high-speed reset: 512
  dev.Reset(Speed::HIGH);
  ControlNone(dev, hw, Make(0x00U, 5U, 3U, 0U, 0U));
  SetConfiguration(dev, hw, 1U);
  TEST_ASSERT(hw.ep1_in.MaxPacketSize() == 512U && hw.ep2_out.MaxPacketSize() == 512U);
  TEST_ASSERT(hw.ep2_out.LastLength() == 512U);
  TEST_ASSERT(hw.ep1_in.MaxTransferSize() == 512U);
}

/// 只支持 8 数据位、1 停止位、9600..115200 的 UART / A UART with 8 data bits, 1 stop
/// bit and 9600..115200 baud only
class LimitedUart : public UART
{
 public:
  LimitedUart()
      : UART(static_cast<LibXR::ReadPort*>(nullptr),
             static_cast<LibXR::WritePort*>(nullptr)),
        rx(256),
        tx_pipe(1024)
  {
    read_port_ = &rx;
    write_port_ = &tx_pipe.GetWritePort();
  }

  ErrorCode SetConfig(Configuration config, bool = false) override
  {
    if (config.baudrate < 9600U || config.baudrate > 115200U || config.data_bits != 8U ||
        config.stop_bits != 1U)
    {
      return ErrorCode::NOT_SUPPORT;
    }
    applied = config;
    return ErrorCode::OK;
  }

  Configuration applied{115200, Parity::NO_PARITY, 8, 1};
  LibXR::ReadPort rx;
  LibXR::Pipe tx_pipe;
};

/// 线路编码、控制线、通知、请求检查 / Line coding, control lines, notification, request
/// checks
void TestRequests()
{
  Hw hw;
  CDCUart cdc(EPNumber::EP1, EPNumber::EP2, EPNumber::EP3);
  struct Seen
  {
    UART::Configuration cfg{};
    unsigned codings = 0;
    unsigned lines = 0;
    bool dtr = false;
    bool rts = false;
  } seen;
  cdc.SetOnSetLineCodingCallback(Callback<UART::Configuration>::Create(
      [](bool, Seen* s, UART::Configuration cfg)
      {
        s->cfg = cfg;
        ++s->codings;
      },
      &seen));
  cdc.SetOnSetControlLineStateCallback(Callback<bool, bool>::Create(
      [](bool, Seen* s, bool dtr, bool rts)
      {
        s->dtr = dtr;
        s->rts = rts;
        ++s->lines;
      },
      &seen));
  FakeUsb dev(hw, {{&cdc}});
  Enumerate(dev, hw);

  // 默认 115200 8N1 / 115200 8N1 by default
  TEST_ASSERT(GetLineCoding(dev, hw) == Coding(115200, 0, 0, 8));

  // 1.5 停止位、Mark 校验、7 数据位都是合法的 PSTN 取值：应答，原样成为当前值。
  // 1.5 stop bits, mark parity and 7 data bits are legal PSTN values: acknowledged and
  // current as sent.
  SetLineCoding(dev, hw, Coding(9600, 1, 3, 7));
  TEST_ASSERT(GetLineCoding(dev, hw) == Coding(9600, 1, 3, 7));
  TEST_ASSERT(seen.codings == 1U && seen.cfg.baudrate == 9600U &&
              seen.cfg.data_bits == 7U);
  SetLineCoding(dev, hw, Coding(921600, 2, 4, 16));
  TEST_ASSERT(GetLineCoding(dev, hw) == Coding(921600, 2, 4, 16));
  TEST_ASSERT(seen.cfg.stop_bits == 2U);

  // 主机只发 6 字节（短包提前结束数据阶段）：ARG_ERR，STALL，编码不变。
  // The host sends 6 bytes only (a short packet ends the data stage): ARG_ERR, stall,
  // coding unchanged.
  dev.Setup(Make(0x21U, SET_LINE_CODING, 0U, 0U, 7U));
  HostOut(dev, hw.out0, Bytes(6, 0));
  TEST_ASSERT(Stalled(hw));
  TEST_ASSERT(GetLineCoding(dev, hw) == Coding(921600, 2, 4, 16));
  TEST_ASSERT(seen.codings == 2U);

  // SET_CONTROL_LINE_STATE：DTR、RTS 生效，回调，SERIAL_STATE 通知。
  // SET_CONTROL_LINE_STATE: DTR and RTS take effect, callback, SERIAL_STATE.
  TEST_ASSERT(!hw.ep3_in.inflight);
  SetLineState(dev, hw, 0x03U);
  TEST_ASSERT(cdc.IsDtrSet() && cdc.IsRtsSet());
  TEST_ASSERT(seen.lines == 1U && seen.dtr && seen.rts);
  const Bytes on{0xA1U, 0x20U, 0U, 0U, 0U, 0U, 2U, 0U, 0x03U, 0U};
  TEST_ASSERT(Notification(hw) == on);
  InDone(dev, hw.ep3_in);
  SetLineState(dev, hw, 0x02U);
  TEST_ASSERT(!cdc.IsDtrSet() && cdc.IsRtsSet() && !seen.dtr && seen.rts);
  TEST_ASSERT(Notification(hw)[8] == 0U);
  // 两个通知槽：第一个在途时第二个排队 / Two notification slots: the second queues
  // behind the first
  SetLineState(dev, hw, 0x01U);
  TEST_ASSERT(hw.ep3_in.starts.size() == 2U);
  // 两个槽都占着时再要一个：留待，通知端点的完成回调发出它。
  // One more while both slots are taken: kept, and sent by the notification
  // endpoint's completion.
  SetLineState(dev, hw, 0x03U);
  TEST_ASSERT(hw.ep3_in.starts.size() == 2U);
  InDone(dev, hw.ep3_in);
  TEST_ASSERT(hw.ep3_in.starts.size() == 3U && Notification(hw)[8] == 0x03U);
  InDone(dev, hw.ep3_in);
  TEST_ASSERT(hw.ep3_in.starts.size() == 4U && Notification(hw)[8] == 0x03U);
  InDone(dev, hw.ep3_in);

  // SEND_BREAK 只应答 / SEND_BREAK is only acknowledged
  ControlNone(dev, hw, Make(0x21U, SEND_BREAK, 100U, 0U, 0U));

  // 拒绝：方向错、长度错、数据接口、端点为接收者、厂商请求、未知请求。
  // Refused: wrong direction, wrong length, data interface, endpoint recipient, vendor
  // request, unknown request.
  Refused(dev, hw, Make(0x21U, GET_LINE_CODING, 0U, 0U, 7U));
  Refused(dev, hw, Make(0xA1U, SET_LINE_CODING, 0U, 0U, 7U));
  Refused(dev, hw, Make(0x21U, SET_LINE_CODING, 0U, 0U, 6U));
  Refused(dev, hw, Make(0xA1U, GET_LINE_CODING, 0U, 0U, 8U));
  Refused(dev, hw, Make(0x21U, SET_CONTROL_LINE_STATE, 1U, 0U, 2U));
  Refused(dev, hw, Make(0xA1U, GET_LINE_CODING, 0U, 1U, 7U));
  Refused(dev, hw, Make(0xA2U, GET_LINE_CODING, 0U, 0x81U, 7U));
  Refused(dev, hw, Make(0xC1U, GET_LINE_CODING, 0U, 0U, 7U));
  Refused(dev, hw, Make(0x21U, 0x30U, 0U, 0U, 0U));
  TEST_ASSERT(cdc.IsDtrSet());

  // 设备一侧改配置：下一次服务时生效，并发通知。
  // A change from the device side: applied in the next service, with a notification.
  const size_t notes = hw.ep3_in.starts.size();
  TEST_ASSERT(cdc.SetConfig({57600, UART::Parity::EVEN, 8, 1}) == ErrorCode::OK);
  TEST_ASSERT(GetLineCoding(dev, hw) == Coding(57600, 0, 2, 8));
  TEST_ASSERT(hw.ep3_in.starts.size() == notes + 1U);
  InDone(dev, hw.ep3_in);
  TEST_ASSERT(cdc.SetConfig({0, UART::Parity::EVEN, 8, 1}) == ErrorCode::ARG_ERR);
  TEST_ASSERT(cdc.SetConfig({9600, UART::Parity::EVEN, 9, 1}) == ErrorCode::ARG_ERR);
  TEST_ASSERT(GetLineCoding(dev, hw) == Coding(57600, 0, 2, 8));

  // 复位解绑：DTR 清零 / A reset unbinds: DTR cleared
  dev.Reset();
  TEST_ASSERT(!cdc.IsDtrSet() && !cdc.IsRtsSet());
}

/// CDCToUart：最接近的可用编码和两个方向的数据 / CDCToUart: the nearest supported
/// coding and data in both directions
void TestCdcToUart()
{
  Hw hw;
  LimitedUart uart;
  CDCToUart cdc(EPNumber::EP1, EPNumber::EP2, EPNumber::EP3, uart);
  FakeUsb dev(hw, {{&cdc}});
  Enumerate(dev, hw);

  SetLineCoding(dev, hw, Coding(57600, 0, 2, 8));
  TEST_ASSERT(GetLineCoding(dev, hw) == Coding(57600, 0, 2, 8));
  TEST_ASSERT(uart.applied.baudrate == 57600U &&
              uart.applied.parity == UART::Parity::EVEN && uart.applied.stop_bits == 1U);

  // 2 停止位不支持：应答，按 1 位运行 / 2 stop bits unsupported: acknowledged, 1 used
  SetLineCoding(dev, hw, Coding(19200, 2, 1, 8));
  TEST_ASSERT(GetLineCoding(dev, hw) == Coding(19200, 0, 1, 8));
  TEST_ASSERT(uart.applied.parity == UART::Parity::ODD && uart.applied.stop_bits == 1U);

  // 7 数据位也不支持：请求波特率下的 8N1 / 7 data bits unsupported too: 8N1 at the rate
  SetLineCoding(dev, hw, Coding(38400, 1, 2, 7));
  TEST_ASSERT(GetLineCoding(dev, hw) == Coding(38400, 0, 0, 8));

  // 不支持的波特率：之前的编码继续生效 / An unsupported rate: the previous coding stays
  SetLineCoding(dev, hw, Coding(3000000, 0, 0, 8));
  TEST_ASSERT(GetLineCoding(dev, hw) == Coding(38400, 0, 0, 8));
  TEST_ASSERT(uart.applied.baudrate == 38400U);

  // 主机 -> CDC -> UART / Host -> CDC -> UART
  const Bytes hello{'h', 'e', 'l', 'l', 'o'};
  HostOut(dev, hw.ep2_out, hello);
  LibXR::ReadPort& uart_out = uart.tx_pipe.GetReadPort();
  TEST_ASSERT(uart_out.Size() == hello.size());
  Bytes got(hello.size());
  std::atomic<PollStatus> status{PollStatus::READY};
  ReadOperation op(status);
  TEST_ASSERT(uart_out({got.data(), got.size()}, op) == ErrorCode::OK);
  TEST_ASSERT(got == hello);

  // UART -> CDC -> 主机 / UART -> CDC -> host
  const Bytes world{'w', 'o', 'r', 'l', 'd'};
  {
    auto queue = uart.rx.GetReadQueue(false);
    TEST_ASSERT(queue.PushBatch(world.data(), world.size()) == ErrorCode::OK);
    queue.Publish();
  }
  TEST_ASSERT(hw.ep1_in.inflight);
  Drain(dev, hw.ep1_in);
  TEST_ASSERT(hw.ep1_in.wire == world);

  // 复位重新枚举后，两条桥接链不重复挂载且收发继续正常 / After bus reset + re-enumerate,
  // chains do not duplicate and bridging still works
  dev.Reset();
  ControlNone(dev, hw, Make(0x00U, 5U, 7U, 0U, 0U));
  SetConfiguration(dev, hw, 1U);
  {
    auto queue = uart.rx.GetReadQueue(false);
    const Bytes again{'a', 'g', 'a', 'i', 'n'};
    TEST_ASSERT(queue.PushBatch(again.data(), again.size()) == ErrorCode::OK);
    queue.Publish();
  }
  TEST_ASSERT(hw.ep1_in.inflight);
  Drain(dev, hw.ep1_in);
  Bytes expected_wire = world;
  expected_wire.insert(expected_wire.end(), {'a', 'g', 'a', 'i', 'n'});
  TEST_ASSERT(hw.ep1_in.wire == expected_wire);

  // 构造前 UART 已有可读数据且尚未构造 FakeUsb：CDCToUart 构造不能提前触发读回调写
  // CDC（此时 service_ 仍为空），枚举后再把预存字节发往主机 / Pre-buffered UART RX
  // bytes before CDCToUart construction & FakeUsb creation: ctor must not touch
  // service_ == nullptr, and queued bytes flush to host once enumerated
  {
    Hw pre_hw;
    LimitedUart pre_uart;
    const Bytes pre{'p', 'r', 'e', '!'};
    {
      auto queue = pre_uart.rx.GetReadQueue(false);
      TEST_ASSERT(queue.PushBatch(pre.data(), pre.size()) == ErrorCode::OK);
      queue.Publish();
    }
    CDCToUart pre_cdc(EPNumber::EP1, EPNumber::EP2, EPNumber::EP3, pre_uart);
    FakeUsb pre_dev(pre_hw, {{&pre_cdc}});
    Enumerate(pre_dev, pre_hw);
    TEST_ASSERT(pre_hw.ep1_in.inflight);
    Drain(pre_dev, pre_hw.ep1_in);
    TEST_ASSERT(pre_hw.ep1_in.wire == pre);
  }
}

/// 发送：零长度包、多槽、绑定前写入、回调写入、同步完成、STALL / Transmit: closing
/// packets, several slots, writes before the bind, writes from callbacks, synchronous
/// completion, halt
void TestTransmit()
{
  // 满包要结尾零长度包，短包不要；两包一槽也是满包结尾。
  // A full last packet needs the closing packet, a short one does not; two packets in
  // one slot end full too.
  {
    Hw hw;
    CDCUart cdc(EPNumber::EP1, EPNumber::EP2, EPNumber::EP3, 128, 2048, 8);
    FakeUsb dev(hw, {{&cdc}});
    Enumerate(dev, hw);
    const Bytes full = Pattern(64, 1);
    Write(cdc, full);
    Drain(dev, hw.ep1_in);
    TEST_ASSERT(hw.ep1_in.Lengths() == (std::vector<size_t>{64U, 0U}));
    const Bytes shorter = Pattern(33, 2);
    Write(cdc, shorter);
    Drain(dev, hw.ep1_in);
    const Bytes two = Pattern(128, 3);
    Write(cdc, two);
    Drain(dev, hw.ep1_in);
    TEST_ASSERT(hw.ep1_in.Lengths() == (std::vector<size_t>{64U, 0U, 33U, 128U, 0U}));
    Bytes expected = full;
    expected.insert(expected.end(), shorter.begin(), shorter.end());
    expected.insert(expected.end(), two.begin(), two.end());
    TEST_ASSERT(hw.ep1_in.wire == expected);

    // 超过一个槽：512 后面还有数据，不补；剩下的 88 是最后一段。
    // More than one slot: 512 is followed by data, no closing packet; the last 88 ends
    // it.
    const Bytes big = Pattern(600, 4);
    Write(cdc, big);
    TEST_ASSERT(hw.ep1_in.LastLength() == 512U);
    Drain(dev, hw.ep1_in);
    const std::vector<size_t> lengths = hw.ep1_in.Lengths();
    TEST_ASSERT(lengths.size() == 7U && lengths[5] == 512U && lengths[6] == 88U);
    TEST_ASSERT(Bytes(hw.ep1_in.wire.end() - 600, hw.ep1_in.wire.end()) == big);

    // 满包提交之后才来的数据：已经要了零长度包，照发（对 CDC 无害）。
    // Data arriving after a full packet was submitted: the closing packet was already
    // requested and is sent (harmless for CDC).
    Write(cdc, Pattern(64, 5));
    Write(cdc, Pattern(10, 6));
    Drain(dev, hw.ep1_in);
    const std::vector<size_t> tail = hw.ep1_in.Lengths();
    TEST_ASSERT((std::vector<size_t>(tail.end() - 3, tail.end()) ==
                 std::vector<size_t>{64U, 0U, 10U}));
    TEST_ASSERT(cdc.write_port_->Size() == 0U);
  }

  // 绑定之前写入的数据在绑定后发出 / Data written before the bind goes out after it
  for (const bool in_isr : {false, true})
  {
    Hw hw;
    CDCUart cdc(EPNumber::EP1, EPNumber::EP2, EPNumber::EP3, 128, 128, 3);
    FakeUsb dev(hw, {{&cdc}});
    Bytes expected;
    for (uint8_t i = 0; i < 3U; ++i)
    {
      const Bytes packet = Pattern(33, i);
      WriteOperation op;
      TEST_ASSERT(cdc.Write({packet.data(), packet.size()}, op, in_isr) == ErrorCode::OK);
      expected.insert(expected.end(), packet.begin(), packet.end());
    }
    TEST_ASSERT(cdc.write_port_->Size() == 99U && hw.ep1_in.starts.empty());
    Enumerate(dev, hw);
    TEST_ASSERT(hw.ep1_in.Lengths() == std::vector<size_t>{99U});
    Drain(dev, hw.ep1_in);
    TEST_ASSERT(hw.ep1_in.wire == expected && cdc.write_port_->Size() == 0U);
  }

  // 写完成回调里再写；回调的 in_isr 是实际推进的上下文。
  // Writing again from the write callback; the callback's in_isr is the context that
  // advanced it.
  {
    Hw hw;
    CDCUart cdc(EPNumber::EP1, EPNumber::EP2, EPNumber::EP3, 128, 2048, 8);
    FakeUsb dev(hw, {{&cdc}});
    Enumerate(dev, hw);
    struct Context
    {
      CDCUart* cdc;
      Bytes extra = Pattern(33, 9);
      std::vector<bool> seen_isr;
      bool write_again = true;
    } ctx{&cdc};
    auto cb = Callback<ErrorCode>::Create(
        [](bool in_isr, Context* self, ErrorCode result)
        {
          TEST_ASSERT(result == ErrorCode::OK);
          self->seen_isr.push_back(in_isr);
          if (self->write_again)
          {
            self->write_again = false;
            WriteOperation op;
            TEST_ASSERT(self->cdc->Write({self->extra.data(), self->extra.size()}, op,
                                         in_isr) == ErrorCode::OK);
          }
        },
        &ctx);
    WriteOperation with_cb(cb);
    const Bytes a = Pattern(512, 10);
    const Bytes b = Pattern(512, 11);
    const Bytes c = Pattern(20, 12);
    // a 在线程里取走，它的回调写的 extra 进第二个槽；b、c 等槽，在中断里取走。
    // a is taken in the thread and the extra its callback writes fills the second
    // slot; b and c wait for a slot and are taken in the interrupt.
    TEST_ASSERT(cdc.Write({a.data(), a.size()}, with_cb) == ErrorCode::OK);
    TEST_ASSERT(cdc.Write({b.data(), b.size()}, with_cb) == ErrorCode::OK);
    TEST_ASSERT(cdc.Write({c.data(), c.size()}, with_cb) == ErrorCode::OK);
    TEST_ASSERT(ctx.seen_isr == std::vector<bool>{false});
    TEST_ASSERT(hw.ep1_in.LastLength() == 512U && cdc.write_port_->Size() == 532U);
    Drain(dev, hw.ep1_in);
    TEST_ASSERT(ctx.seen_isr == (std::vector<bool>{false, true, true}));
    Bytes expected = a;
    expected.insert(expected.end(), ctx.extra.begin(), ctx.extra.end());
    expected.insert(expected.end(), b.begin(), b.end());
    expected.insert(expected.end(), c.begin(), c.end());
    TEST_ASSERT(hw.ep1_in.wire == expected);
  }

  // 后端在 StartHardware 里同步完成：1024 次写入全部按序发出，零长度包也发。
  // The backend completes synchronously inside StartHardware: 1024 writes all go out
  // in order, closing packets too.
  {
    Hw hw;
    hw.ep1_in.complete_in_start = true;
    CDCUart cdc(EPNumber::EP1, EPNumber::EP2, EPNumber::EP3);
    FakeUsb dev(hw, {{&cdc}});
    Enumerate(dev, hw);
    Bytes expected;
    for (unsigned i = 0; i < 1024U; ++i)
    {
      const Bytes packet = Pattern(33, static_cast<uint8_t>(i));
      Write(cdc, packet);
      expected.insert(expected.end(), packet.begin(), packet.end());
    }
    TEST_ASSERT(!hw.ep1_in.inflight && hw.ep1_in.wire == expected);
    Write(cdc, Pattern(64, 1));
    const std::vector<size_t> lengths = hw.ep1_in.Lengths();
    TEST_ASSERT(lengths.size() == 1026U && lengths[1024] == 64U && lengths[1025] == 0U);
  }

  // 主机 STALL 数据 IN：在途请求中止、字节丢弃；期间写入的数据排队到 CLEAR_FEATURE。
  // The host halts data IN: the request in flight aborts and its bytes are dropped;
  // data written meanwhile waits for CLEAR_FEATURE.
  {
    Hw hw;
    CDCUart cdc(EPNumber::EP1, EPNumber::EP2, EPNumber::EP3);
    FakeUsb dev(hw, {{&cdc}});
    Enumerate(dev, hw);
    Write(cdc, Pattern(40, 1));
    TEST_ASSERT(hw.ep1_in.inflight);
    ControlNone(dev, hw, Make(0x02U, 3U, 0U, 0x81U, 0U));
    TEST_ASSERT(hw.ep1_in.IsStalled() && !hw.ep1_in.inflight && hw.ep1_in.stalls == 1U);
    const Bytes later = Pattern(30, 2);
    Write(cdc, later);
    TEST_ASSERT(hw.ep1_in.starts.size() == 1U && cdc.write_port_->Size() == 0U);
    ControlNone(dev, hw, Make(0x02U, 1U, 0U, 0x81U, 0U));
    TEST_ASSERT(hw.ep1_in.inflight && hw.ep1_in.LastLength() == 30U);
    Drain(dev, hw.ep1_in);
    TEST_ASSERT(hw.ep1_in.wire == later);
  }
}

/// 接收：顺序、短包、零长度包、读端口满时反压、STALL / Receive: order, short and
/// zero-length packets, backpressure from a full read port, halt
void TestReceive()
{
  Hw hw;
  CDCUart cdc(EPNumber::EP1, EPNumber::EP2, EPNumber::EP3, 128);
  FakeUsb dev(hw, {{&cdc}});
  Enumerate(dev, hw);

  Bytes sent;
  auto send = [&](const Bytes& bytes)
  {
    HostOut(dev, hw.ep2_out, bytes);
    sent.insert(sent.end(), bytes.begin(), bytes.end());
  };
  send(Pattern(10, 1));
  send({});
  send(Pattern(64, 2));
  TEST_ASSERT(ReadAll(cdc) == sent);
  sent.clear();

  // 读端口 128：两包填满，第三、四包的槽留在类里，主机被 NAK（没有接收在途）。
  // A 128-byte read port: two packets fill it, the slots of the third and fourth stay
  // with the class and the host is NAKed (no receive in flight).
  for (uint8_t i = 0; i < 4U; ++i)
  {
    send(Pattern(64, static_cast<uint8_t>(10U + i)));
  }
  TEST_ASSERT(!hw.ep2_out.inflight && cdc.read_port_->Size() == 128U);
  const size_t starts = hw.ep2_out.starts.size();

  // 读走 64：一个槽搬进去、交回、重新挂上 / Read 64: one slot moves in, is handed back
  // and armed again
  Bytes got(64);
  std::atomic<PollStatus> status{PollStatus::READY};
  ReadOperation op(status);
  TEST_ASSERT(cdc.Read({got.data(), got.size()}, op) == ErrorCode::OK);
  TEST_ASSERT(hw.ep2_out.inflight && hw.ep2_out.starts.size() == starts + 1U);
  TEST_ASSERT(cdc.read_port_->Size() == 128U);
  Bytes all = got;
  const Bytes rest = ReadAll(cdc);
  all.insert(all.end(), rest.begin(), rest.end());
  const Bytes last = ReadAll(cdc);
  all.insert(all.end(), last.begin(), last.end());
  TEST_ASSERT(all == sent);
  TEST_ASSERT(hw.ep2_out.inflight);

  // 主机 STALL 数据 OUT：接收中止，CLEAR_FEATURE 后重新开始。
  // The host halts data OUT: the receive aborts and starts again after CLEAR_FEATURE.
  ControlNone(dev, hw, Make(0x02U, 3U, 0U, 0x02U, 0U));
  TEST_ASSERT(hw.ep2_out.IsStalled() && !hw.ep2_out.inflight);
  ControlNone(dev, hw, Make(0x02U, 1U, 0U, 0x02U, 0U));
  TEST_ASSERT(hw.ep2_out.inflight && hw.ep2_out.LastLength() == 64U);
  const Bytes after = Pattern(20, 30);
  HostOut(dev, hw.ep2_out, after);
  TEST_ASSERT(ReadAll(cdc) == after);
}

/// 写测试类和读测试类 / The write-test and read-test classes
void TestTestClasses()
{
  {
    Hw hw;
    CDCWriteTest cdc(EPNumber::EP1, EPNumber::EP2, EPNumber::EP3);
    FakeUsb dev(hw, {{&cdc}});
    Enumerate(dev, hw);
    TEST_ASSERT(hw.ep2_out.inflight && hw.ep2_out.starts.size() == 1U);
    TEST_ASSERT(hw.ep2_out.LastLength() == hw.ep2_out.MaxPacketSize() && !cdc.IsDtrSet());

    // 没有 DTR：丢弃并重新挂接收，不发送 IN / No DTR: dropped, armed again, no IN
    HostOut(dev, hw.ep2_out, Pattern(64, 3));
    TEST_ASSERT(hw.ep2_out.inflight && hw.ep2_out.starts.size() == 2U);
    TEST_ASSERT(hw.ep1_in.starts.empty());

    SetLineState(dev, hw, 0x01U);
    TEST_ASSERT(cdc.IsDtrSet() && Notification(hw).size() == 10U);
    InDone(dev, hw.ep3_in);

    // 收到一个 OUT 包后 IN 填满所有槽（长度为 MaxTransferSize，全 0）；每次 IN 完成后
    // 不用再发 OUT 包也会自动续发下一批 IN / One OUT packet fills every free IN slot
    // (length MaxTransferSize, all zeros); InDone keeps pumping IN without new OUT
    const size_t tx_len = hw.ep1_in.MaxTransferSize();
    TEST_ASSERT(tx_len == 512U);
    HostOut(dev, hw.ep2_out, Pattern(10, 7));
    TEST_ASSERT(hw.ep1_in.inflight && hw.ep1_in.LastLength() == tx_len);
    const uint8_t* zeros = hw.ep1_in.starts.back().data;
    TEST_ASSERT(std::all_of(zeros, zeros + tx_len, [](uint8_t b) { return b == 0U; }));
    TEST_ASSERT(hw.ep2_out.inflight && hw.ep2_out.starts.size() == 3U);
    // 两槽都在用（1 在途、1 排队）：先完成一次，应自动补发第 3 次（starts == 2）并
    // 继续在途，无需新的 OUT 包 / Two slots busy: finishing one keeps pumping without
    // another OUT packet
    InDone(dev, hw.ep1_in);
    TEST_ASSERT(hw.ep1_in.inflight && hw.ep1_in.starts.size() == 2U &&
                hw.ep1_in.LastLength() == tx_len);
    const uint8_t* zeros2 = hw.ep1_in.starts.back().data;
    TEST_ASSERT(std::all_of(zeros2, zeros2 + tx_len, [](uint8_t b) { return b == 0U; }));
    InDone(dev, hw.ep1_in);
    TEST_ASSERT(hw.ep1_in.inflight && hw.ep1_in.starts.size() == 3U);

    // DTR=1 时对 ep2_out 发 SET_FEATURE(ENDPOINT_HALT) 触发 ABORTED：不误发 IN /
    // Halting ep2_out while DTR=1 produces ABORTED without triggering an IN submit
    const size_t in_before_halt = hw.ep1_in.starts.size();
    ControlNone(dev, hw, Make(0x02U, 3U, 0U, 0x02U, 0U));
    TEST_ASSERT(hw.ep2_out.IsStalled() && !hw.ep2_out.inflight);
    TEST_ASSERT(hw.ep1_in.starts.size() == in_before_halt);

    // 清 DTR 后清除 EP2 OUT 的 HALT：已排队的 1 个空闲槽发完后停止，重挂后收到 OUT
    // 也不再触发 IN / Clear EP2 OUT HALT after DTR clears: drain the 1 already-queued
    // slot, then submit no more IN even on newly received OUT
    SetLineState(dev, hw, 0x00U);
    InDone(dev, hw.ep3_in);
    const size_t in_starts = hw.ep1_in.starts.size();
    while (hw.ep1_in.inflight)
    {
      InDone(dev, hw.ep1_in);
    }
    TEST_ASSERT(hw.ep1_in.starts.size() == in_starts + 1U && !hw.ep1_in.inflight);
    ControlNone(dev, hw, Make(0x02U, 1U, 0U, 0x02U, 0U));
    TEST_ASSERT(!hw.ep2_out.IsStalled() && hw.ep2_out.inflight);
    HostOut(dev, hw.ep2_out, Pattern(10, 8));
    TEST_ASSERT(hw.ep1_in.starts.size() == in_starts + 1U && !hw.ep1_in.inflight);

    SetConfiguration(dev, hw, 0U);
    TEST_ASSERT(hw.ep1_in.closes == 1U && hw.ep2_out.closes == 1U);
    TEST_ASSERT(hw.ep3_in.closes == 1U);
  }
  {
    Hw hw;
    CDCReadTest cdc(EPNumber::EP1, EPNumber::EP2, EPNumber::EP3);
    FakeUsb dev(hw, {{&cdc}});
    Enumerate(dev, hw);
    const size_t rx_len = hw.ep2_out.MaxTransferSize();
    TEST_ASSERT(rx_len == 512U);
    TEST_ASSERT(hw.ep2_out.inflight && hw.ep2_out.LastLength() == rx_len);
    for (uint8_t i = 0; i < 4U; ++i)
    {
      HostOut(dev, hw.ep2_out, Pattern(64, i));
      TEST_ASSERT(hw.ep2_out.inflight && hw.ep2_out.starts.size() == 2U + i &&
                  hw.ep2_out.LastLength() == rx_len);
    }
    // STALL ABORTED 后不触发 IN；CLEAR_FEATURE 后恢复全部接收 / Halting aborts cleanly
    // without IN traffic, and CLEAR_FEATURE resumes receiving
    ControlNone(dev, hw, Make(0x02U, 3U, 0U, 0x02U, 0U));
    TEST_ASSERT(hw.ep2_out.IsStalled() && !hw.ep2_out.inflight);
    ControlNone(dev, hw, Make(0x02U, 1U, 0U, 0x02U, 0U));
    TEST_ASSERT(!hw.ep2_out.IsStalled() && hw.ep2_out.inflight &&
                hw.ep2_out.LastLength() == rx_len);
    HostOut(dev, hw.ep2_out, Pattern(64, 9));
    TEST_ASSERT(hw.ep2_out.inflight);
    TEST_ASSERT(hw.ep1_in.starts.empty() && hw.ep3_in.starts.empty());
  }
}

/// 一个 CDC 绑定后能收发 / A bound CDC sends and receives
void CheckWorks(FakeUsb& dev, Hw& hw, CDCUart& cdc)
{
  const size_t before = hw.ep1_in.wire.size();
  const Bytes out = Pattern(21, 40);
  Write(cdc, out);
  Drain(dev, hw.ep1_in);
  TEST_ASSERT(Bytes(hw.ep1_in.wire.begin() + static_cast<std::ptrdiff_t>(before),
                    hw.ep1_in.wire.end()) == out);
  const Bytes in = Pattern(17, 41);
  HostOut(dev, hw.ep2_out, in);
  TEST_ASSERT(ReadAll(cdc) == in);
}

/// 绑定失败：Configure 出错、包长不符、端点被占、端点不存在 / Bind failure: a Configure
/// error, a size mismatch, an endpoint taken, an endpoint missing
void TestBindFailure()
{
  // 通知端点 Configure 失败：两个数据端点已配置，Unbind 关闭并交回；不挂接收。
  // The notification endpoint's Configure fails: both data endpoints were configured,
  // Unbind closes and returns them; no receive is armed.
  {
    Hw hw;
    hw.ep3_in.configure_result = ErrorCode::FAILED;
    CDCUart cdc(EPNumber::EP1, EPNumber::EP2, EPNumber::EP3);
    FakeUsb dev(hw, {{&cdc}});
    dev.Init(false);
    dev.Reset();
    Refused(dev, hw, Make(0x00U, 9U, 1U, 0U, 0U));
    TEST_ASSERT(hw.ep1_in.configures == 1U && hw.ep1_in.closes == 1U);
    TEST_ASSERT(hw.ep2_out.configures == 1U && hw.ep2_out.closes == 1U);
    // 被 CheckHardware 拒绝的端点没有碰过硬件 / The endpoint CheckHardware refused never
    // touched the hardware
    TEST_ASSERT(hw.ep3_in.configures == 0U && hw.ep3_in.closes == 0U);
    for (FakeEp* ep : {&hw.ep1_in, &hw.ep2_out, &hw.ep3_in})
    {
      TEST_ASSERT(ep->GetState() == Endpoint::State::DISABLED && ep->starts.empty());
    }
    TEST_ASSERT(ControlIn(dev, hw, Make(0x80U, 8U, 0U, 0U, 1U)) == Bytes{0U});
    // 修好后同一个配置成功：端点都已交回池 / Fixed, the same configuration works: every
    // endpoint went back to the pool
    hw.ep3_in.configure_result = ErrorCode::OK;
    SetConfiguration(dev, hw, 1U);
    CheckWorks(dev, hw, cdc);
  }

  // 读回的包长与描述符不同（后端的上限在查询和 Configure 之间变了）：CHECK_ERR。
  // The size read back differs from the descriptor (the backend's limit changed
  // between the query and Configure): CHECK_ERR.
  {
    Hw hw;
    CDCUart cdc(EPNumber::EP1, EPNumber::EP2, EPNumber::EP3);
    FakeUsb dev(hw, {{&cdc}});
    dev.Init(false);
    dev.Reset();
    hw.ep1_in.shrink_after = hw.ep1_in.limit_calls + 1;
    Refused(dev, hw, Make(0x00U, 9U, 1U, 0U, 0U));
    TEST_ASSERT(hw.ep1_in.MaxPacketSize() == 32U && hw.ep1_in.closes == 1U);
    TEST_ASSERT(hw.ep2_out.configures == 0U);
    hw.ep1_in.shrink_after = -1;
    SetConfiguration(dev, hw, 1U);
    TEST_ASSERT(hw.ep1_in.MaxPacketSize() == 64U);
    CheckWorks(dev, hw, cdc);
  }

  // 两个 CDC 用同一组端点：第二个 Get 失败，两个都解绑。
  // Two CDCs on the same endpoints: the second Get fails and both are unbound.
  {
    Hw hw;
    CDCUart first(EPNumber::EP1, EPNumber::EP2, EPNumber::EP3);
    CDCUart second(EPNumber::EP1, EPNumber::EP2, EPNumber::EP3);
    FakeUsb dev(hw, {{&first, &second}});
    dev.Init(false);
    dev.Reset();
    const Bytes config = GetDescriptor(dev, hw, 2U, 0U, 255U);
    TEST_ASSERT(config.size() == 9U + 2U * 66U && config[4] == 4U);
    Refused(dev, hw, Make(0x00U, 9U, 1U, 0U, 0U));
    TEST_ASSERT(hw.ep1_in.closes == 1U && hw.ep2_out.closes == 1U);
    TEST_ASSERT(hw.ep3_in.closes == 1U && hw.ep2_out.starts.empty());
  }

  // 通知端点没有登记：写描述符失败，GET_DESCRIPTOR 和 SET_CONFIGURATION 都被拒，不绑定。
  // The notification endpoint is not registered: writing the descriptors fails, both
  // GET_DESCRIPTOR and SET_CONFIGURATION are refused, nothing is bound.
  {
    Hw hw;
    CDCUart cdc(EPNumber::EP1, EPNumber::EP2, EPNumber::EP9);
    FakeUsb dev(hw, {{&cdc}});
    dev.Init(false);
    dev.Reset();
    Refused(dev, hw, Make(0x80U, 6U, 0x0200U, 0U, 255U));
    Refused(dev, hw, Make(0x00U, 9U, 1U, 0U, 0U));
    TEST_ASSERT(hw.ep1_in.configures == 0U && hw.ep2_out.configures == 0U);
  }
}

/**
 * 带在途请求解绑：IN 一个在途、一个排队，写端口里还有数据；OUT 一个接收在途、一个槽留在
 * 类里；通知在途。四种解绑：配置 0、复位、同一配置再设一次、Deinit。
 * Unbind with requests in flight: one IN in flight and one queued with more data in the
 * write port; one OUT receive in flight and one slot kept by the class; a notification
 * in flight. Four ways to unbind: configuration 0, reset, the same configuration again,
 * Deinit.
 */
void TestUnbindInFlight()
{
  for (int how = 0; how < 4; ++how)
  {
    Hw hw;
    CDCUart cdc(EPNumber::EP1, EPNumber::EP2, EPNumber::EP3, 64, 2048, 8);
    FakeUsb dev(hw, {{&cdc}});
    Enumerate(dev, hw);
    SetLineState(dev, hw, 0x01U);
    TEST_ASSERT(hw.ep3_in.inflight);

    const Bytes data = Pattern(1500, 50);
    Write(cdc, data);
    TEST_ASSERT(hw.ep1_in.inflight && hw.ep1_in.LastLength() == 512U);
    TEST_ASSERT(cdc.write_port_->Size() == 1500U - 1024U);

    const Bytes first = Pattern(64, 60);
    HostOut(dev, hw.ep2_out, first);
    HostOut(dev, hw.ep2_out, Pattern(64, 61));
    TEST_ASSERT(cdc.read_port_->Size() == 64U && hw.ep2_out.inflight);
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

    // 三个端点都停过、关过；在途和排队的 IN 丢弃，写端口里的保留；不再挂接收。
    // All three endpoints were stopped and closed; in-flight and queued IN dropped,
    // the write port's data kept; no receive armed again.
    TEST_ASSERT(hw.ep1_in.stops == 1U && hw.ep1_in.closes == 1U);
    TEST_ASSERT(hw.ep2_out.stops == 1U && hw.ep2_out.closes == 1U);
    TEST_ASSERT(hw.ep3_in.stops == 1U && hw.ep3_in.closes == 1U);
    TEST_ASSERT(hw.ep1_in.wire.empty());
    TEST_ASSERT(how == 2 || cdc.write_port_->Size() == 1500U - 1024U);
    TEST_ASSERT(!cdc.IsDtrSet());
    TEST_ASSERT(cdc.read_port_->Size() == 64U && ReadAll(cdc) == first);
    if (how != 2)
    {
      TEST_ASSERT(hw.ep2_out.starts.size() == out_starts);
      for (FakeEp* ep : {&hw.ep1_in, &hw.ep2_out, &hw.ep3_in})
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

    // 重新绑定后：写端口剩下的数据发出；两个 IN 槽、两个 OUT 槽都可用。
    // After the new bind: the rest of the write port goes out; both IN slots and both
    // OUT slots are usable.
    TEST_ASSERT(hw.ep1_in.inflight && hw.ep1_in.LastLength() == 476U);
    Drain(dev, hw.ep1_in);
    TEST_ASSERT(hw.ep1_in.wire == Bytes(data.begin() + 1024, data.end()));
    const Bytes again = Pattern(1024, 70);
    Write(cdc, again);
    TEST_ASSERT(hw.ep1_in.LastLength() == 512U && cdc.write_port_->Size() == 0U);
    Drain(dev, hw.ep1_in);
    TEST_ASSERT(Bytes(hw.ep1_in.wire.end() - 1024, hw.ep1_in.wire.end()) == again);
    HostOut(dev, hw.ep2_out, Pattern(64, 80));
    TEST_ASSERT(hw.ep2_out.inflight);
    HostOut(dev, hw.ep2_out, Pattern(64, 81));
    TEST_ASSERT(hw.ep2_out.inflight && cdc.read_port_->Size() == 64U);
    TEST_ASSERT(ReadAll(cdc) == Pattern(64, 80));
    TEST_ASSERT(ReadAll(cdc) == Pattern(64, 81) && hw.ep2_out.inflight);
  }
}

/**
 * 两个线程写、一个线程读、一个线程扮控制器中断，主线程从设备一侧改线路编码。中断线程
 * 每次进入中断时完成在途的 IN 和通知，并在 OUT 上送一段主机数据。
 * Two threads write, one reads, one plays the controller interrupt, and the main thread
 * changes the line coding from the device side. On each interrupt the interrupt thread
 * completes the IN and notification requests in flight and sends a piece of host data
 * on OUT.
 */
void TestThreads()
{
  Hw hw;
  CDCUart cdc(EPNumber::EP1, EPNumber::EP2, EPNumber::EP3, 128, 256, 4);
  FakeUsb dev(hw, {{&cdc}});
  Enumerate(dev, hw);

  constexpr uint32_t RECORDS = 3000U;
  constexpr size_t RECORD = 8U;
  const Bytes host_data = Pattern(40000, 90);
  size_t host_sent = 0;  // 只在 owner 中 / Owner only
  size_t steps = 0;      // 只在 owner 中 / Owner only
  std::atomic<bool> stop{false};
  std::atomic<bool> reader_done{false};

  auto step = [&](bool isr)
  {
    ++steps;
    if (hw.ep1_in.inflight)
    {
      hw.ep1_in.Done(isr, hw.ep1_in.LastLength());
    }
    if (hw.ep3_in.inflight)
    {
      hw.ep3_in.Done(isr, hw.ep3_in.LastLength());
    }
    if (hw.ep2_out.inflight && host_sent < host_data.size())
    {
      const size_t chunk = (steps % 5U == 0U) ? 64U : 1U + (steps * 37U) % 64U;
      const size_t n = std::min(chunk, host_data.size() - host_sent);
      const auto begin = host_data.begin() + static_cast<std::ptrdiff_t>(host_sent);
      hw.ep2_out.HostSends(isr, Bytes(begin, begin + static_cast<std::ptrdiff_t>(n)));
      host_sent += n;
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

  auto writer = [&](uint8_t id)
  {
    for (uint32_t seq = 0; seq < RECORDS; ++seq)
    {
      uint8_t record[RECORD] = {id, 0, 0, 0, 0, static_cast<uint8_t>(~id), 0x5AU, 0xA5U};
      std::memcpy(record + 1, &seq, sizeof(seq));
      WriteOperation op;
      while (cdc.Write({record, sizeof(record)}, op) != ErrorCode::OK)
      {
        std::this_thread::yield();
      }
    }
  };
  std::thread writer_a(writer, 1U);
  std::thread writer_b(writer, 2U);

  Bytes received;
  std::thread reader(
      [&]
      {
        Bytes buffer(256);
        while (received.size() < host_data.size())
        {
          const size_t size = std::min<size_t>(cdc.read_port_->Size(), buffer.size());
          if (size == 0U)
          {
            std::this_thread::yield();
            continue;
          }
          std::atomic<PollStatus> status{PollStatus::READY};
          ReadOperation op(status);
          const ErrorCode result = cdc.Read({buffer.data(), size}, op);
          if (result == ErrorCode::BUSY)
          {
            continue;
          }
          TEST_ASSERT(result == ErrorCode::OK && status.load() == PollStatus::DONE);
          received.insert(received.end(), buffer.begin(),
                          buffer.begin() + static_cast<std::ptrdiff_t>(size));
        }
        reader_done.store(true, std::memory_order_release);
      });

  uint32_t baud = 9600U;
  uint32_t last = baud;
  const size_t total = 2U * RECORDS * RECORD;
  while (!reader_done.load(std::memory_order_acquire) ||
         hw.ep1_in.wire_bytes.load(std::memory_order_acquire) < total)
  {
    TEST_ASSERT(cdc.SetConfig({baud, UART::Parity::NO_PARITY, 8, 1}) == ErrorCode::OK);
    last = baud;
    baud = baud >= 921600U ? 9600U : baud * 2U;
    std::this_thread::yield();
  }
  writer_a.join();
  writer_b.join();
  reader.join();
  stop.store(true, std::memory_order_release);
  interrupt.join();

  // 主机收到的流由 8 字节记录组成，每个写者的序号连续。
  // The stream the host got consists of 8-byte records, each writer's in sequence.
  Drain(dev, hw.ep1_in);
  TEST_ASSERT(hw.ep1_in.wire.size() == total);
  uint32_t next[3] = {0U, 0U, 0U};
  for (size_t at = 0; at < total; at += RECORD)
  {
    const uint8_t* record = hw.ep1_in.wire.data() + at;
    const uint8_t id = record[0];
    TEST_ASSERT((id == 1U || id == 2U) && record[5] == static_cast<uint8_t>(~id));
    TEST_ASSERT(record[6] == 0x5AU && record[7] == 0xA5U);
    uint32_t seq = 0;
    std::memcpy(&seq, record + 1, sizeof(seq));
    TEST_ASSERT(seq == next[id]);
    ++next[id];
  }
  TEST_ASSERT(next[1] == RECORDS && next[2] == RECORDS);
  TEST_ASSERT(received == host_data);

  // 零长度包只跟在满包之后，最后一个满包后面一定有。
  // A zero-length packet only follows a full packet, and the last full one has one.
  const std::vector<size_t> lengths = hw.ep1_in.Lengths();
  for (size_t i = 0; i < lengths.size(); ++i)
  {
    if (lengths[i] == 0U)
    {
      TEST_ASSERT(i > 0U && lengths[i - 1] != 0U && lengths[i - 1] % 64U == 0U);
    }
  }
  TEST_ASSERT(lengths.back() == 0U || lengths.back() % 64U != 0U);

  // 主线程最后一次 SetConfig 已生效 / The main thread's last SetConfig is in effect
  dev.Irq([](bool) {});
  const Bytes coding = GetLineCoding(dev, hw);
  uint32_t seen = 0;
  std::memcpy(&seen, coding.data(), sizeof(seen));
  TEST_ASSERT(seen == last);
  TEST_ASSERT(!dev.masked.load());
}

/**
 * 只有读者和中断：读端口 64 字节，两个接收槽常被留住，接收只能靠读者腾出空间时的那次
 * 请求恢复；丢了这次请求就停住（用例超时）。
 * Only the reader and the interrupt: with a 64-byte read port both receive slots are
 * often held, and receiving resumes only through the request made when the reader frees
 * space; losing that request stalls it (the case times out).
 */
void TestReceiveRace()
{
  Hw hw;
  CDCUart cdc(EPNumber::EP1, EPNumber::EP2, EPNumber::EP3, 64);
  FakeUsb dev(hw, {{&cdc}});
  Enumerate(dev, hw);

  const Bytes host_data = Pattern(20000, 7);
  size_t host_sent = 0;  // 只在 owner 中 / Owner only
  size_t steps = 0;      // 只在 owner 中 / Owner only
  std::atomic<bool> stop{false};
  auto step = [&](bool isr)
  {
    ++steps;
    if (hw.ep2_out.inflight && host_sent < host_data.size())
    {
      const size_t chunk = (steps % 3U == 0U) ? 64U : 1U + (steps * 13U) % 64U;
      const size_t n = std::min(chunk, host_data.size() - host_sent);
      const auto begin = host_data.begin() + static_cast<std::ptrdiff_t>(host_sent);
      hw.ep2_out.HostSends(isr, Bytes(begin, begin + static_cast<std::ptrdiff_t>(n)));
      host_sent += n;
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

  Bytes received;
  Bytes buffer(64);
  for (size_t round = 0; received.size() < host_data.size(); ++round)
  {
    // 读得慢：读端口常满，两个槽都被留住。
    // Read slowly: the read port is often full and both slots are held.
    std::this_thread::yield();
    const size_t want = 1U + (round * 7U) % 16U;
    const size_t size = std::min(cdc.read_port_->Size(), want);
    if (size == 0U)
    {
      std::this_thread::yield();
      continue;
    }
    std::atomic<PollStatus> status{PollStatus::READY};
    ReadOperation op(status);
    const ErrorCode result = cdc.Read({buffer.data(), size}, op);
    if (result == ErrorCode::BUSY)
    {
      continue;
    }
    TEST_ASSERT(result == ErrorCode::OK && status.load() == PollStatus::DONE);
    received.insert(received.end(), buffer.begin(),
                    buffer.begin() + static_cast<std::ptrdiff_t>(size));
  }
  stop.store(true, std::memory_order_release);
  interrupt.join();
  TEST_ASSERT(received == host_data);
}

/// 后端可能给的槽数：核心默认（两个）、1、3、8 / Slot counts a backend may give: the core
/// default (two), 1, 3 and 8
constexpr uint8_t SLOT_COUNTS[] = {0U, 1U, 3U, 8U};

/// 数据端点的实际槽数 / The actual slot count of the data endpoints
size_t SlotsOf(uint8_t slots) { return slots == 0U ? 2U : slots; }

/// 每个空闲的 OUT 槽都挂着接收：再挂一个得 BUSY / Every free OUT slot is receiving:
/// one more receive gets BUSY
bool AllReceiving(FakeEp& ep) { return ep.Receive(64U, false) == ErrorCode::BUSY; }

/// 主机连发 64 字节包直到被 NAK，返回发出的字节 / The host sends 64-byte packets until
/// it is NAKed; returns the bytes sent
Bytes FillUntilNak(FakeUsb& dev, Hw& hw, uint8_t seed)
{
  Bytes sent;
  for (size_t guard = 0; hw.ep2_out.inflight; ++guard)
  {
    TEST_ASSERT(guard < 32U);
    const Bytes bytes = Pattern(64, static_cast<uint8_t>(seed + guard));
    HostOut(dev, hw.ep2_out, bytes);
    sent.insert(sent.end(), bytes.begin(), bytes.end());
  }
  return sent;
}

/// 读到 size 字节为止 / Read until size bytes arrived
Bytes ReadBytes(CDCUart& cdc, size_t size)
{
  Bytes got;
  for (size_t guard = 0; got.size() < size; ++guard)
  {
    TEST_ASSERT(guard < 64U);
    const Bytes part = ReadAll(cdc);
    got.insert(got.end(), part.begin(), part.end());
  }
  return got;
}

/**
 * @brief 槽数为默认、1、3、8 时 CDCUart 的接收反压和发送 / CDCUart receive
 *        backpressure and sending with the default, 1, 3 and 8 slots
 * @note 读端口 128 字节：两个包填满，之后每个 OUT 槽留住一个包，全部留住时主机被 NAK。
 *       读走后数据按序到达，所有槽重新接收；带着留住的槽重新绑定，槽全部交回。
 *       A 128-byte read port: two packets fill it, then every OUT slot holds one packet
 *       and the host is NAKed once all are held. Read out, the data arrives in order and
 *       every slot receives again; a rebind with slots held gives them all back.
 */
void TestSlotCounts()
{
  for (uint8_t slots : SLOT_COUNTS)
  {
    const size_t count = SlotsOf(slots);
    {
      Hw hw(slots);
      CDCUart cdc(EPNumber::EP1, EPNumber::EP2, EPNumber::EP3, 128, 2048, 8);
      FakeUsb dev(hw, {{&cdc}});
      Enumerate(dev, hw);
      TEST_ASSERT(AllReceiving(hw.ep2_out));

      Bytes sent = FillUntilNak(dev, hw, 1U);
      TEST_ASSERT(sent.size() == 64U * (2U + count));
      TEST_ASSERT(cdc.read_port_->Size() == 128U);
      TEST_ASSERT(ReadBytes(cdc, sent.size()) == sent);
      TEST_ASSERT(AllReceiving(hw.ep2_out));

      // HALT 再清除：所有槽仍在收 / A halt and its clear: every slot still receives
      ControlNone(dev, hw, Make(0x02U, 3U, 0U, 0x02U, 0U));
      ControlNone(dev, hw, Make(0x02U, 1U, 0U, 0x02U, 0U));
      TEST_ASSERT(AllReceiving(hw.ep2_out));

      // 再填满后重新绑定：留住的槽全部交回，读端口里的字节保留。
      // Filled again, then rebound: every held slot goes back, the read port keeps its
      // bytes.
      sent = FillUntilNak(dev, hw, 50U);
      TEST_ASSERT(sent.size() == 64U * (2U + count));
      SetConfiguration(dev, hw, 1U);
      TEST_ASSERT(AllReceiving(hw.ep2_out));
      Bytes kept(sent.begin(), sent.begin() + 128);
      TEST_ASSERT(ReadBytes(cdc, 128U) == kept);
      sent = FillUntilNak(dev, hw, 100U);
      TEST_ASSERT(sent.size() == 64U * (2U + count));
      TEST_ASSERT(ReadBytes(cdc, sent.size()) == sent);

      // IN：写端口的数据按槽数排队，主机取走后一个字节不差。
      // IN: the write port's data queues per slot and reaches the host intact.
      const Bytes out = Pattern(1500, 7);
      Write(cdc, out);
      Drain(dev, hw.ep1_in);
      TEST_ASSERT(hw.ep1_in.wire == out);
    }
  }
}

/// 默认、1、3、8 个槽下测试类在每个空闲槽上挂接收 / With the default, 1, 3 and 8 slots
/// the test classes arm a receive on every free slot
void TestTestClassSlots()
{
  for (uint8_t slots : SLOT_COUNTS)
  {
    {
      Hw hw(slots);
      CDCReadTest cdc(EPNumber::EP1, EPNumber::EP2, EPNumber::EP3);
      FakeUsb dev(hw, {{&cdc}});
      Enumerate(dev, hw);
      TEST_ASSERT(AllReceiving(hw.ep2_out));
      TEST_ASSERT(hw.ep2_out.LastLength() == hw.ep2_out.MaxTransferSize());
      HostOut(dev, hw.ep2_out, Pattern(64, 1));
      TEST_ASSERT(hw.ep2_out.inflight && AllReceiving(hw.ep2_out));
    }
    {
      Hw hw(slots);
      CDCWriteTest cdc(EPNumber::EP1, EPNumber::EP2, EPNumber::EP3);
      FakeUsb dev(hw, {{&cdc}});
      Enumerate(dev, hw);
      TEST_ASSERT(AllReceiving(hw.ep2_out));
      TEST_ASSERT(hw.ep2_out.LastLength() == hw.ep2_out.MaxPacketSize());
      HostOut(dev, hw.ep2_out, Pattern(64, 1));
      TEST_ASSERT(hw.ep2_out.inflight && AllReceiving(hw.ep2_out));
      SetLineState(dev, hw, 0x01U);
      InDone(dev, hw.ep3_in);
      HostOut(dev, hw.ep2_out, Pattern(64, 2));
      TEST_ASSERT(hw.ep1_in.inflight &&
                  hw.ep1_in.LastLength() == hw.ep1_in.MaxTransferSize());
      // 所有 IN 槽应都已被填满或提交，再来 AcquireSlot 得 BUSY / All IN slots filled
      Endpoint::Slot extra;
      TEST_ASSERT(hw.ep1_in.AcquireSlot(extra) == ErrorCode::BUSY);
      SetLineState(dev, hw, 0x00U);
      InDone(dev, hw.ep3_in);
      while (hw.ep1_in.inflight)
      {
        InDone(dev, hw.ep1_in);
      }
    }
  }
}

}  // namespace

void test_usb_cdc_enum() { TestEnumeration(); }
void test_usb_cdc_requests()
{
  TestRequests();
  TestCdcToUart();
}
void test_usb_cdc_tx() { TestTransmit(); }
void test_usb_cdc_rx() { TestReceive(); }
void test_usb_cdc_test_classes() { TestTestClasses(); }
void test_usb_cdc_bind_fail() { TestBindFailure(); }
void test_usb_cdc_unbind() { TestUnbindInFlight(); }
void test_usb_cdc_threads()
{
  TestThreads();
  TestReceiveRace();
}
void test_usb_cdc_slots()
{
  TestSlotCounts();
  TestTestClassSlots();
}
