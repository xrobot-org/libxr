/**
 * @file test_uac.cpp
 * @brief UAC1 麦克风的假后端测试 / Fake-backend tests of the UAC1 microphone
 *
 * 用真实的设备核心、组合层、端点和 UAC 类；假后端设备同时派生 DeviceService 和
 * DeviceCore，测试把"硬件事件"（SETUP、复位、SOF、端点完成）排进它的队列再调用控制器中断
 * 入口。主机一侧由辅助函数扮演。覆盖描述符、类请求和音频流、绑定失败、带在途请求
 * 解绑、SET_INTERFACE 重新配置的各条路径和多线程。
 * Uses the real device core, composition, endpoints and UAC class; the fake backend
 * device derives from both DeviceService and DeviceCore, and a test queues "hardware
 * events" (SETUP, reset, SOF, endpoint completion) and calls the controller interrupt
 * entry. Helpers play the host. Covered: descriptors, class requests and the audio
 * stream, bind failure, unbind with requests in flight, the paths of the SET_INTERFACE
 * reconfiguration and threads.
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

#include "dev_core.hpp"
#include "ep.hpp"
#include "ep_pool.hpp"
#include "test_assert.hpp"
#include "uac_mic.hpp"
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
using LibXR::USB::SetupPacket;
using LibXR::USB::Speed;
using LibXR::USB::UAC1MicrophoneQ;
using LibXR::USB::USBSpec;
using Direction = Endpoint::Direction;
using EPNumber = Endpoint::EPNumber;
using State = Endpoint::State;
using Bytes = std::vector<uint8_t>;

static_assert(LibXR::HW_CACHE_LINE_SIZE <= 64U, "test memory is aligned to 64 bytes");

constexpr auto LANGUAGE = DescriptorStrings::MakeLanguagePack(
    DescriptorStrings::Language::EN_US, "XRobot", "UAC", "0001");

constexpr uint8_t WHOLE = 0xffU;  ///< 只报整帧 / Whole frames only

/// 端点内存 / Endpoint memory
struct alignas(64) Memory
{
  uint8_t bytes[512];
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
      packets.emplace_back(data, data + actual);
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

  /// 后端报告挂起的停止超时；只在 owner 中 / The backend reports the pending stop as
  /// timed out; owner only
  void TimedOut(bool in_isr) { OnHardwareComplete(in_isr, 0U, ErrorCode::TIMEOUT); }

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
  std::vector<Bytes> packets;  ///< 主机取走的 IN 包 / IN packets the host took
  bool inflight = false;
  size_t configures = 0;
  size_t closes = 0;
  size_t stops = 0;
  ErrorCode configure_result = ErrorCode::OK;
  bool stuck = false;   ///< 停止一直挂起 / The stop stays pending
  bool reset = false;   ///< 控制器已复位，停止立即成功 / Controller reset, stops succeed
  bool no_iso = false;  ///< 硬件不支持等时 / No isochronous support in hardware
  /// HardwareLimits 第几次调用之后报 32 字节包 / After this many HardwareLimits calls,
  /// report 32-byte packets
  int shrink_after = -1;
  mutable int limit_calls = 0;
  /// 停止、关闭、启动是否都带 in_isr=true / Whether every stop, close and start had
  /// in_isr=true
  bool all_isr = true;
  /// 只有一个槽：硬件不直接读写槽（核心对单槽的要求）/ One slot only: the hardware
  /// does not access the slot directly (what the core requires of a single slot)
  bool one_slot;

 protected:
  ErrorCode CheckHardware(const Config&) const override { return configure_result; }

  void ConfigureHardware(const Config&) override { ++configures; }

  void CloseHardware(bool in_isr) override
  {
    ++closes;
    all_isr = all_isr && in_isr;
  }

  ErrorCode StartHardware(const HwRequest& request, bool in_isr) override
  {
    TEST_ASSERT(!inflight);
    starts.push_back({request.length, request.data});
    inflight = true;
    all_isr = all_isr && in_isr;
    return ErrorCode::OK;
  }

  StopOutcome StopHardware(bool in_isr) override
  {
    ++stops;
    all_isr = all_isr && in_isr;
    if (stuck && !reset)
    {
      return {ErrorCode::PENDING, 0U};
    }
    inflight = false;
    return {ErrorCode::ABORTED, 0U};
  }

  ErrorCode StallHardware(bool) override { return ErrorCode::OK; }
  ErrorCode ClearStallHardware(bool) override { return ErrorCode::OK; }

  HwLimits HardwareLimits(Direction, Type type) const override
  {
    ++limit_calls;
    HwLimits limits{};
    if (no_iso && type == Type::ISOCHRONOUS)
    {
      limits = {0U, SIZE_MAX, SIZE_MAX};
    }
    else if (shrink_after >= 0 && limit_calls > shrink_after)
    {
      limits = {32U, SIZE_MAX, SIZE_MAX};
    }
    limits.direct_memory = !one_slot;
    return limits;
  }
};

/// 假端点和端点池：EP0 和 EP1 IN（等时）/ Fake endpoints and the pool: EP0 and EP1 IN
/// (isochronous)
struct Hw
{
  /// iso_slots：EP1 的槽数，0 为核心默认 / iso_slots: slot count of EP1, 0 for default
  explicit Hw(size_t iso_bytes = 512U, uint8_t iso_slots = 0U)
      : ep1_in{EPNumber::EP1, Direction::IN, {mem[2].bytes, iso_bytes}, iso_slots}
  {
    pool.SetEndpoint0(&in0, &out0);
    TEST_ASSERT(pool.Put(&ep1_in) == ErrorCode::OK);
  }

  Memory mem[3];
  FakeEp in0{EPNumber::EP0, Direction::IN, {mem[0].bytes, 64U}};
  FakeEp out0{EPNumber::EP0, Direction::OUT, {mem[1].bytes, 64U}};
  FakeEp ep1_in;
  EndpointPool pool;

  std::vector<FakeEp*> All() { return {&in0, &out0, &ep1_in}; }
};

using Configs = std::initializer_list<const std::initializer_list<DeviceClass*>>;

/// 假后端设备 / Fake backend device
class FakeUsb final : public DeviceService, public DeviceCore
{
 public:
  FakeUsb(Hw& hw, const Configs& configs, Speed speed = Speed::FULL)
      : DeviceCore(*this, hw.pool, USBSpec::USB_2_0, speed,
                   DeviceDescriptor::PacketSize0::SIZE_64, 0x1209U, 0x0002U, 0x0100U,
                   {&LANGUAGE}, configs),
        hw_(hw)
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
  void Sof(uint16_t frame, uint8_t microframe = WHOLE)
  {
    Irq([this, frame, microframe](bool isr) { OnStartOfFrame(isr, frame, microframe); });
  }

  bool Inited() const { return IsInited(); }
  void StartController(bool) override {}
  void StopController(bool) override {}

  size_t ready = 0;
  size_t resets = 0;

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

  void MaskInterrupt() override {}
  void UnmaskInterrupt() override {}
  void SetAddress(uint8_t, AddressStage) override {}
  void OnControlReady() override { ++ready; }
  void ResetController(bool) override
  {
    ++resets;
    for (FakeEp* ep : hw_.All())
    {
      ep->reset = true;
    }
  }

 private:
  Hw& hw_;
  std::mutex mutex_;
  std::deque<std::function<void(bool)>> events_;
};

/// 测试用的麦克风：单声道 16 位，公开诊断计数 / The microphone under test: mono 16-bit,
/// diagnostics made public
class Mic : public UAC1MicrophoneQ<1, 16>
{
 public:
  using UAC1MicrophoneQ<1, 16>::UAC1MicrophoneQ;
  using UAC1MicrophoneQ<1, 16>::UnderrunCount;
  using UAC1MicrophoneQ<1, 16>::MissedTransferCount;
  using UAC1MicrophoneQ<1, 16>::DroppedFrameCount;
};

/// 双声道 16 位 / Stereo 16-bit
class StereoMic : public UAC1MicrophoneQ<2, 16>
{
 public:
  using UAC1MicrophoneQ<2, 16>::UAC1MicrophoneQ;
};

// ---- 主机一侧 / The host side ----

SetupPacket Make(uint8_t type, uint8_t request, uint16_t value, uint16_t index,
                 uint16_t length)
{
  return SetupPacket{type, request, value, index, length};
}

bool Stalled(Hw& hw) { return hw.in0.IsStalled() && hw.out0.IsStalled(); }

/// 主机取走在途的 IN 包 / The host takes the in-flight IN packet
void InDone(FakeUsb& dev, FakeEp& ep)
{
  TEST_ASSERT(ep.inflight);
  const size_t length = ep.LastLength();
  dev.Irq([&ep, length](bool isr) { ep.Done(isr, length); });
}

/// 在途的 IN 以 result 结束 / The in-flight IN ends with result
void InEnds(FakeUsb& dev, FakeEp& ep, ErrorCode result)
{
  TEST_ASSERT(ep.inflight);
  dev.Irq([&ep, result](bool isr) { ep.Done(isr, 0U, result); });
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
  dev.Irq([&hw, data](bool isr) { hw.out0.HostSends(isr, data); });
  TEST_ASSERT(hw.in0.inflight && hw.in0.LastLength() == 0U);
  InDone(dev, hw.in0);
  TEST_ASSERT(dev.ready == ready + 1U);
}

/// SETUP 被接受，数据阶段被拒 / SETUP accepted, data stage refused
void RefusedData(FakeUsb& dev, Hw& hw, SetupPacket setup, const Bytes& data)
{
  dev.Setup(setup);
  TEST_ASSERT(!Stalled(hw));
  dev.Irq([&hw, data](bool isr) { hw.out0.HostSends(isr, data); });
  TEST_ASSERT(Stalled(hw));
}

/// 无数据阶段的请求；状态阶段必须在 SETUP 处理完时已经发出 / A request without a data
/// stage; the status stage must be out when the SETUP has been handled
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

SetupPacket SetInterface(uint16_t itf, uint16_t alt)
{
  return Make(0x01U, 11U, alt, itf, 0U);
}

uint8_t GetInterface(FakeUsb& dev, Hw& hw, uint16_t itf)
{
  const Bytes got = ControlIn(dev, hw, Make(0x81U, 10U, 0U, itf, 1U));
  TEST_ASSERT(got.size() == 1U);
  return got[0];
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

Bytes Pattern(size_t size, uint8_t seed)
{
  Bytes bytes(size);
  for (size_t i = 0; i < size; ++i)
  {
    bytes[i] = static_cast<uint8_t>(seed + i * 7U + 1U);
  }
  return bytes;
}

Bytes Zeros(size_t size) { return Bytes(size, 0U); }

uint16_t Le16(const Bytes& b, size_t at)
{
  return static_cast<uint16_t>(b[at] | (b[at + 1U] << 8U));
}

// ---- UAC 请求 / UAC requests ----

constexpr uint8_t SET_CUR = Mic::SET_CUR;
constexpr uint8_t GET_CUR = Mic::GET_CUR;
constexpr uint8_t GET_MIN = Mic::GET_MIN;
constexpr uint8_t GET_MAX = Mic::GET_MAX;
constexpr uint8_t SET_RES = Mic::SET_RES;
constexpr uint8_t GET_RES = Mic::GET_RES;
/// 端点为接收者的类请求 / Class request whose recipient is an endpoint
constexpr uint8_t EP_IN = 0xA2U;
constexpr uint8_t EP_OUT = 0x22U;
/// 接口为接收者的类请求 / Class request whose recipient is an interface
constexpr uint8_t ITF_IN = 0xA1U;
constexpr uint8_t ITF_OUT = 0x21U;
/// 特性单元在 AC 接口 0：wIndex / Feature unit on AC interface 0: wIndex
constexpr uint16_t FU = static_cast<uint16_t>(Mic::ID_FU << 8U);
constexpr uint16_t VOLUME = static_cast<uint16_t>(Mic::FU_VOLUME << 8U);
constexpr uint16_t MUTE = static_cast<uint16_t>(Mic::FU_MUTE << 8U);
constexpr uint16_t RATE = static_cast<uint16_t>(Mic::SAMPLING_FREQ_CONTROL << 8U);

/// 配置描述符：全速、高速（OTHER_SPEED）、字符串；写描述符不碰端点；绑定后在 alt 0
/// Configuration descriptor at full and high speed (OTHER_SPEED), strings; writing it
/// touches no endpoint; alt 0 after the bind
void TestEnumeration()
{
  Hw hw;
  Mic mic(EPNumber::EP1, 16000U);
  FakeUsb dev(hw, {{&mic}}, Speed::HIGH);
  dev.Init(false);
  dev.Reset(Speed::FULL);

  // 9 + 108 字节：IAD、AC 接口、头、输入端子、特性单元、输出端子、AS alt 0、alt 1、AS
  // 通用、格式、9 字节等时端点、类特定端点。
  // 9 + 108 bytes: IAD, AC interface, header, input terminal, feature unit, output
  // terminal, AS alt 0, alt 1, AS general, format, 9-byte iso endpoint, class endpoint.
  const Bytes fs = GetDescriptor(dev, hw, 2U, 0U, 255U);
  TEST_ASSERT(fs.size() == 117U && fs[2] == 117U && fs[4] == 2U);
  TEST_ASSERT(fs[9] == 8U && fs[10] == 0x0BU && fs[11] == 0U && fs[12] == 2U);
  TEST_ASSERT(fs[13] == 1U && fs[14] == 1U);
  TEST_ASSERT(fs[17] == 9U && fs[18] == 4U && fs[19] == 0U && fs[21] == 0U);
  TEST_ASSERT(fs[22] == 1U && fs[23] == 1U && fs[25] == 4U);
  TEST_ASSERT(fs[26] == 9U && fs[27] == 0x24U && fs[28] == 1U && Le16(fs, 29) == 0x0100U);
  TEST_ASSERT(Le16(fs, 31) == 39U && fs[33] == 1U && fs[34] == 1U);
  TEST_ASSERT(fs[35] == 12U && fs[37] == 2U && fs[38] == 1U && Le16(fs, 39) == 0x0201U);
  TEST_ASSERT(fs[42] == 1U && Le16(fs, 43) == 0U);
  TEST_ASSERT(fs[47] == 9U && fs[49] == 6U && fs[50] == 2U && fs[51] == 1U);
  TEST_ASSERT(fs[52] == 1U && fs[53] == 3U && fs[54] == 3U && fs[55] == 0U);
  TEST_ASSERT(fs[56] == 9U && fs[58] == 3U && fs[59] == 3U && Le16(fs, 60) == 0x0101U);
  TEST_ASSERT(fs[63] == 2U);
  TEST_ASSERT(fs[65] == 9U && fs[67] == 1U && fs[68] == 0U && fs[69] == 0U);
  TEST_ASSERT(fs[70] == 1U && fs[71] == 2U && fs[73] == 5U);
  TEST_ASSERT(fs[74] == 9U && fs[76] == 1U && fs[77] == 1U && fs[78] == 1U);
  TEST_ASSERT(fs[82] == 5U);
  TEST_ASSERT(fs[83] == 7U && fs[85] == 1U && fs[86] == 3U && fs[87] == 1U);
  TEST_ASSERT(Le16(fs, 88) == 1U);
  TEST_ASSERT(fs[90] == 11U && fs[92] == 2U && fs[93] == 1U && fs[94] == 1U);
  TEST_ASSERT(fs[95] == 2U && fs[96] == 16U && fs[97] == 1U);
  TEST_ASSERT(fs[98] == 0x80U && fs[99] == 0x3EU && fs[100] == 0U);
  // 16000 Hz、1 ms：16 帧，再多留 1 帧 = 34 字节 / 16 frames plus one = 34 bytes
  TEST_ASSERT(fs[101] == 9U && fs[102] == 5U && fs[103] == 0x81U && fs[104] == 5U);
  TEST_ASSERT(Le16(fs, 105) == 34U && fs[107] == 1U && fs[108] == 0U && fs[109] == 0U);
  TEST_ASSERT(fs[110] == 7U && fs[111] == 0x25U && fs[112] == 1U && fs[113] == 0U);

  // 高速默认每毫秒一个包：和全速一样 16 帧再多 1 帧 = 34 字节，bInterval 4。
  // High speed sends one packet per millisecond by default: 16 frames plus one = 34
  // bytes as at full speed, bInterval 4.
  const Bytes hs = GetDescriptor(dev, hw, 7U, 0U, 255U);
  TEST_ASSERT(hs.size() == 117U && hs[1] == 7U);
  TEST_ASSERT(Le16(hs, 105) == 34U && hs[107] == 4U);

  // 接口字符串是 4、5 号 / The interface strings are indices 4 and 5
  const Bytes control = GetDescriptor(dev, hw, 3U, 4U, 255U, 0x0409U);
  TEST_ASSERT(control.size() == 2U + 2U * 18U && control[2] == 'X');
  const Bytes streaming = GetDescriptor(dev, hw, 3U, 5U, 255U, 0x0409U);
  TEST_ASSERT(streaming.size() == 2U + 2U * 20U && streaming[2 + 2 * 11] == 'S');

  // 写描述符不碰端点；SET_CONFIGURATION 配置一次、比较包长、关闭（alt 0）。
  // Writing descriptors touches no endpoint; SET_CONFIGURATION configures once,
  // compares the size and closes (alt 0).
  TEST_ASSERT(hw.ep1_in.configures == 0U);
  ControlNone(dev, hw, Make(0x00U, 5U, 3U, 0U, 0U));
  SetConfiguration(dev, hw, 1U);
  TEST_ASSERT(hw.ep1_in.configures == 1U && hw.ep1_in.closes == 1U);
  TEST_ASSERT(hw.ep1_in.GetState() == State::DISABLED && hw.ep1_in.starts.empty());
  TEST_ASSERT(hw.ep1_in.MaxPacketSize() == 34U);
  TEST_ASSERT(hw.ep1_in.GetType() == Endpoint::Type::ISOCHRONOUS);
  TEST_ASSERT(GetInterface(dev, hw, 1U) == 0U && GetInterface(dev, hw, 0U) == 0U);

  // 高速复位后再配置：默认每毫秒一个包，包长仍是 34
  // Configured again after a high-speed reset: one packet per millisecond by default,
  // so the packet size is still 34
  dev.Reset(Speed::HIGH);
  ControlNone(dev, hw, Make(0x00U, 5U, 3U, 0U, 0U));
  SetConfiguration(dev, hw, 1U);
  TEST_ASSERT(hw.ep1_in.MaxPacketSize() == 34U);

  // 双声道：输入端子声明左前、右前，特性单元 10 字节，包长 (44+1)×4 = 180。
  // Stereo: the input terminal declares left and right front, the feature unit is 10
  // bytes, the packet (44+1)x4 = 180.
  Hw hw2;
  StereoMic stereo(EPNumber::EP1, 44100U);
  FakeUsb dev2(hw2, {{&stereo}});
  dev2.Init(false);
  dev2.Reset();
  const Bytes st = GetDescriptor(dev2, hw2, 2U, 0U, 255U);
  TEST_ASSERT(st.size() == 118U && Le16(st, 31) == 40U);
  TEST_ASSERT(st[42] == 2U && Le16(st, 43) == 0x0003U);
  TEST_ASSERT(st[47] == 10U && st[53] == 3U && st[54] == 3U && st[55] == 3U);
  TEST_ASSERT(st[95] == 2U && st[96] == 2U);
  TEST_ASSERT(st[99] == 0x44U && st[100] == 0xACU && st[101] == 0U);
  TEST_ASSERT(Le16(st, 106) == 180U);
}

/// 流：补零、整帧、余字节留在队列、丢区间、FAILED 只计数、alt 0 停流、非法 alt
/// Streaming: zero fill, whole frames, leftover bytes stay queued, dropped intervals,
/// FAILED only counted, alt 0 stops, illegal alts
void TestStream()
{
  Hw hw;
  Mic mic(EPNumber::EP1, 16000U);
  FakeUsb dev(hw, {{&mic}});
  Enumerate(dev, hw);
  FakeEp& ep = hw.ep1_in;

  // alt 1：配置端点并提交第一个包；队列空，整包补零。状态阶段在 SETUP 处理完时已发出。
  // Alt 1: configure the endpoint and submit the first packet; the queue is empty,
  // the whole packet is zeros. The status stage is out once the SETUP is handled.
  ControlNone(dev, hw, SetInterface(1U, 1U));
  TEST_ASSERT(ep.GetState() == State::IDLE && ep.configures == 2U && ep.closes == 1U);
  TEST_ASSERT(ep.starts.size() == 1U && ep.LastLength() == 32U);
  TEST_ASSERT(mic.UnderrunCount() == 1U && GetInterface(dev, hw, 1U) == 1U);
  InDone(dev, ep);
  TEST_ASSERT(ep.packets.back() == Zeros(32U));
  // 完成不提交下一个包：下一个由 SOF 提交 / A completion submits nothing; the SOF does
  TEST_ASSERT(ep.starts.size() == 1U && !ep.inflight);

  // 16 字节 PCM 加 16 字节零 / 16 bytes of PCM and 16 zeros
  const Bytes pcm = Pattern(16U, 1U);
  TEST_ASSERT(mic.WritePcm(pcm.data(), pcm.size()) == ErrorCode::OK);
  dev.Sof(1U);
  TEST_ASSERT(ep.starts.size() == 2U && ep.LastLength() == 32U);
  TEST_ASSERT(mic.UnderrunCount() == 2U);
  InDone(dev, ep);
  Bytes expected = pcm;
  expected.insert(expected.end(), 16U, 0U);
  TEST_ASSERT(ep.packets.back() == expected);

  // 正好一个区间的 PCM：不补零 / Exactly one interval of PCM: no zero fill
  const Bytes full = Pattern(32U, 40U);
  TEST_ASSERT(mic.WritePcm(full.data(), full.size()) == ErrorCode::OK);
  dev.Sof(2U);
  InDone(dev, ep);
  TEST_ASSERT(ep.packets.back() == full && mic.UnderrunCount() == 2U);

  // 不满一个音频帧的尾字节留在队列，不拆散音频帧。
  // A trailing partial audio frame stays queued; audio frames are never split.
  const Bytes tail{7U, 7U, 7U};
  TEST_ASSERT(mic.WritePcm(tail.data(), tail.size()) == ErrorCode::OK);
  dev.Sof(3U);
  InDone(dev, ep);
  Bytes tail_expected{7U, 7U};
  tail_expected.insert(tail_expected.end(), 30U, 0U);
  TEST_ASSERT(ep.packets.back() == tail_expected && mic.QueueSize() == 1U);

  // 两个槽都占着：这个区间丢弃并计数 / Both slots taken: the interval is dropped and
  // counted
  dev.Sof(4U);
  dev.Sof(5U);
  TEST_ASSERT(ep.starts.size() == 5U && ep.inflight);
  dev.Sof(6U);
  TEST_ASSERT(mic.DroppedFrameCount() == 1U && ep.starts.size() == 5U);
  InDone(dev, ep);
  TEST_ASSERT(ep.starts.size() == 6U && ep.inflight);
  InDone(dev, ep);
  dev.Sof(7U);
  TEST_ASSERT(ep.starts.size() == 7U && mic.DroppedFrameCount() == 1U);
  InDone(dev, ep);

  // FAILED、OUT_OF_RANGE：只计数，端点仍 IDLE，不关闭也不重新配置，下一拍照常提交。
  // FAILED and OUT_OF_RANGE: only counted; the endpoint stays IDLE, is neither closed
  // nor reconfigured, and the next tick submits as usual.
  dev.Sof(8U);
  InEnds(dev, ep, ErrorCode::FAILED);
  TEST_ASSERT(mic.MissedTransferCount() == 1U && ep.GetState() == State::IDLE);
  dev.Sof(9U);
  InEnds(dev, ep, ErrorCode::OUT_OF_RANGE);
  TEST_ASSERT(mic.MissedTransferCount() == 2U);
  TEST_ASSERT(ep.configures == 2U && ep.closes == 1U && ep.stops == 0U);
  dev.Sof(10U);
  TEST_ASSERT(ep.inflight && ep.starts.size() == 10U);
  InDone(dev, ep);
  TEST_ASSERT(GetInterface(dev, hw, 1U) == 1U);

  // alt 0：端点关闭，SOF 不再提交，ABORTED 不计为失败。
  // Alt 0: the endpoint closes, the SOF submits nothing, ABORTED is no failure.
  dev.Sof(11U);
  ControlNone(dev, hw, SetInterface(1U, 0U));
  TEST_ASSERT(ep.GetState() == State::DISABLED && ep.closes == 2U && ep.stops == 1U);
  TEST_ASSERT(!ep.inflight && mic.MissedTransferCount() == 2U);
  TEST_ASSERT(GetInterface(dev, hw, 1U) == 0U);
  dev.Sof(12U);
  TEST_ASSERT(ep.starts.size() == 11U);
  mic.ResetQueue();
  TEST_ASSERT(mic.QueueSize() == 0U);

  // alt 2 不存在；AC 接口只接受 alt 0（PROVISIONAL）；接口 2 不存在。
  // Alt 2 does not exist; the AC interface accepts alt 0 only (PROVISIONAL); there is no
  // interface 2.
  Refused(dev, hw, SetInterface(1U, 2U));
  ControlNone(dev, hw, SetInterface(0U, 0U));
  Refused(dev, hw, SetInterface(0U, 1U));
  Refused(dev, hw, SetInterface(2U, 0U));
  TEST_ASSERT(ep.GetState() == State::DISABLED && GetInterface(dev, hw, 1U) == 0U);
}

/// 类请求：采样率、特性单元，经真实的 EP0 控制传输 / Class requests: sampling
/// frequency and feature unit, through real EP0 control transfers
void TestRequests()
{
  Hw hw;
  Mic mic(EPNumber::EP1, 16000U, -0x2000, 0x0600, 0x0100);
  FakeUsb dev(hw, {{&mic}});
  Enumerate(dev, hw);

  // 采样率：CUR、MIN、MAX 都是 16000（3 字节小端），RES 为 1。
  // Sampling frequency: CUR, MIN and MAX are all 16000 (3 bytes LE), RES is 1.
  const Bytes rate{0x80U, 0x3EU, 0x00U};
  TEST_ASSERT(ControlIn(dev, hw, Make(EP_IN, GET_CUR, RATE, 0x81U, 3U)) == rate);
  TEST_ASSERT(ControlIn(dev, hw, Make(EP_IN, GET_MIN, RATE, 0x81U, 3U)) == rate);
  TEST_ASSERT(ControlIn(dev, hw, Make(EP_IN, GET_MAX, RATE, 0x81U, 3U)) == rate);
  TEST_ASSERT(ControlIn(dev, hw, Make(EP_IN, GET_RES, RATE, 0x81U, 3U)) ==
              (Bytes{1U, 0U, 0U}));
  Refused(dev, hw, Make(EP_IN, GET_CUR, RATE, 0x81U, 2U));
  Refused(dev, hw, Make(EP_OUT, GET_CUR, RATE, 0x81U, 3U));
  Refused(dev, hw, Make(EP_IN, SET_CUR, RATE, 0x81U, 3U));
  Refused(dev, hw, Make(EP_IN, GET_CUR, 0x0200U, 0x81U, 3U));
  Refused(dev, hw, Make(EP_IN, GET_CUR, RATE, 0x82U, 3U));

  // SET_CUR：当前采样率接受，其他在数据阶段被拒。
  // SET_CUR: the current rate is accepted, any other is refused in the data stage.
  ControlOut(dev, hw, Make(EP_OUT, SET_CUR, RATE, 0x81U, 3U), rate);
  RefusedData(dev, hw, Make(EP_OUT, SET_CUR, RATE, 0x81U, 3U), {0x80U, 0xBBU, 0x00U});
  TEST_ASSERT(ControlIn(dev, hw, Make(EP_IN, GET_CUR, RATE, 0x81U, 3U)) == rate);

  // 音量的范围按小端 / Volume range, little-endian
  TEST_ASSERT(ControlIn(dev, hw, Make(ITF_IN, GET_MIN, VOLUME, FU, 2U)) ==
              (Bytes{0x00U, 0xE0U}));
  TEST_ASSERT(ControlIn(dev, hw, Make(ITF_IN, GET_MAX, VOLUME, FU, 2U)) ==
              (Bytes{0x00U, 0x06U}));
  TEST_ASSERT(ControlIn(dev, hw, Make(ITF_IN, GET_RES, VOLUME, FU, 2U)) ==
              (Bytes{0x00U, 0x01U}));
  TEST_ASSERT(ControlIn(dev, hw, Make(ITF_IN, GET_CUR, VOLUME, FU, 2U)) ==
              (Bytes{0U, 0U}));

  // SET_CUR 音量和静音；主通道和通道 1 共用一个值。
  // SET_CUR of volume and mute; the master and channel 1 share one value.
  ControlOut(dev, hw, Make(ITF_OUT, SET_CUR, VOLUME, FU, 2U), {0x00U, 0xF0U});
  TEST_ASSERT(ControlIn(dev, hw, Make(ITF_IN, GET_CUR, VOLUME | 1U, FU, 2U)) ==
              (Bytes{0x00U, 0xF0U}));
  ControlOut(dev, hw, Make(ITF_OUT, SET_CUR, MUTE, FU, 1U), {1U});
  TEST_ASSERT(ControlIn(dev, hw, Make(ITF_IN, GET_CUR, MUTE, FU, 1U)) == Bytes{1U});

  // SET_RES 音量经真实的数据阶段被接受，什么也不改。
  // SET_RES of the volume is accepted through the real data stage and changes nothing.
  ControlOut(dev, hw, Make(ITF_OUT, SET_RES, VOLUME, FU, 2U), {0x05U, 0x00U});
  TEST_ASSERT(ControlIn(dev, hw, Make(ITF_IN, GET_RES, VOLUME, FU, 2U)) ==
              (Bytes{0x00U, 0x01U}));
  TEST_ASSERT(ControlIn(dev, hw, Make(ITF_IN, GET_CUR, VOLUME, FU, 2U)) ==
              (Bytes{0x00U, 0xF0U}));

  // 拒绝：错实体、通道超出、长度不对、方向不对、静音没有 MIN、AS 接口、厂商请求。
  // Refused: wrong entity, channel out of range, wrong length, wrong direction, no MIN
  // for mute, the AS interface, a vendor request.
  Refused(dev, hw, Make(ITF_IN, GET_CUR, VOLUME, 0x0100U, 2U));
  Refused(dev, hw, Make(ITF_IN, GET_CUR, VOLUME | 2U, FU, 2U));
  Refused(dev, hw, Make(ITF_IN, GET_CUR, VOLUME, FU, 1U));
  Refused(dev, hw, Make(ITF_IN, GET_CUR, MUTE, FU, 2U));
  Refused(dev, hw, Make(ITF_OUT, GET_CUR, VOLUME, FU, 2U));
  Refused(dev, hw, Make(ITF_IN, SET_CUR, VOLUME, FU, 2U));
  Refused(dev, hw, Make(ITF_IN, GET_MIN, MUTE, FU, 1U));
  Refused(dev, hw, Make(ITF_IN, GET_CUR, 0x0300U, FU, 1U));
  Refused(dev, hw, Make(ITF_IN, GET_CUR, VOLUME, FU | 1U, 2U));
  Refused(dev, hw, Make(0xC1U, GET_CUR, VOLUME, FU, 2U));

  // 未配置时类请求不再交给类 / Unconfigured, class requests no longer reach the class
  SetConfiguration(dev, hw, 0U);
  Refused(dev, hw, Make(EP_IN, GET_CUR, RATE, 0x81U, 3U));
  Refused(dev, hw, Make(ITF_IN, GET_CUR, VOLUME, FU, 2U));
}

/// 节拍：全速 44.1 kHz 的余数进位；高速按帧号累积微帧 / Pacing: remainder carry at
/// 44.1 kHz full speed; microframes accrued from frame numbers at high speed
void TestPacing()
{
  // 44100 Hz 双声道：每 1 ms 44.1 帧，九个 176 字节后一个 180 字节。
  // 44100 Hz stereo: 44.1 frames per 1 ms, nine packets of 176 bytes, then one of 180.
  {
    Hw hw;
    StereoMic mic(EPNumber::EP1, 44100U);
    FakeUsb dev(hw, {{&mic}});
    Enumerate(dev, hw);
    ControlNone(dev, hw, SetInterface(1U, 1U));
    InDone(dev, hw.ep1_in);
    for (uint16_t frame = 0; frame < 19U; ++frame)
    {
      dev.Sof(frame);
      InDone(dev, hw.ep1_in);
    }
    std::vector<size_t> expected(20U, 176U);
    expected[9] = 180U;
    expected[19] = 180U;
    TEST_ASSERT(hw.ep1_in.Lengths() == expected);
  }

  // 高速 interval 1：每个微帧一个区间（4 字节）；多出的整区间丢弃；只报整帧时一帧算 8 个
  // 微帧。
  // High speed, interval 1: one interval per microframe (4 bytes); surplus whole
  // intervals are dropped; with whole frames only, a frame counts as 8 microframes.
  {
    Hw hw;
    Mic mic(EPNumber::EP1, 16000U, 0, 0, 1, Speed::HIGH, 2048U, 1U);
    FakeUsb dev(hw, {{&mic}}, Speed::HIGH);
    Enumerate(dev, hw, Speed::HIGH);
    FakeEp& ep = hw.ep1_in;
    // 每个微帧一个包要显式给 interval 1：包长 6，bInterval 1
    // One packet per microframe is asked for with interval 1: packet size 6, bInterval 1
    const Bytes hs = GetDescriptor(dev, hw, 2U, 0U, 255U);
    TEST_ASSERT(Le16(hs, 105) == 6U && hs[107] == 1U);
    ControlNone(dev, hw, SetInterface(1U, 1U));
    TEST_ASSERT(ep.MaxPacketSize() == 6U);
    TEST_ASSERT(ep.starts.size() == 1U && ep.LastLength() == 4U);
    InDone(dev, ep);
    dev.Sof(100U, 0U);  // 第一次 SOF：差值不知道，算 1 / First SOF: unknown, counts 1
    TEST_ASSERT(ep.starts.size() == 2U && ep.LastLength() == 4U);
    InDone(dev, ep);
    dev.Sof(100U, 3U);  // 过了 3 个微帧：提交一次 / 3 microframes: one submit
    TEST_ASSERT(ep.starts.size() == 3U);
    InDone(dev, ep);
    dev.Sof(101U, WHOLE);  // 模式变了：算 1 帧 / Mode changed: one frame
    TEST_ASSERT(ep.starts.size() == 4U);
    InDone(dev, ep);
    SetConfiguration(dev, hw, 0U);
  }

  // 高速 interval 3：一个区间 4 个微帧（8000 Hz：4 帧 = 8 字节，包长 10）；不够不提交，
  // 帧号回绕照样算对。
  // High speed, interval 3: an interval is 4 microframes (8000 Hz: 4 frames = 8 bytes,
  // packet 10); no submit before a full one; the frame-number wrap is counted right.
  {
    Hw hw;
    Mic mic(EPNumber::EP1, 8000U, 0, 0, 1, Speed::HIGH, 1024U, 3U);
    FakeUsb dev(hw, {{&mic}}, Speed::HIGH);
    Enumerate(dev, hw, Speed::HIGH);
    FakeEp& ep = hw.ep1_in;
    const Bytes hs = GetDescriptor(dev, hw, 2U, 0U, 255U);
    TEST_ASSERT(Le16(hs, 105) == 10U && hs[107] == 3U);
    const Bytes fs = GetDescriptor(dev, hw, 7U, 0U, 255U);
    TEST_ASSERT(Le16(fs, 105) == 18U && fs[107] == 1U);
    ControlNone(dev, hw, SetInterface(1U, 1U));
    TEST_ASSERT(ep.MaxPacketSize() == 10U && ep.LastLength() == 8U);
    InDone(dev, ep);
    dev.Sof(10U, 0U);  // 1 < 4
    dev.Sof(10U, 2U);  // 3 < 4
    TEST_ASSERT(ep.starts.size() == 1U);
    dev.Sof(10U, 3U);  // 4：提交，剩 0 / 4: submit, 0 left
    TEST_ASSERT(ep.starts.size() == 2U && ep.LastLength() == 8U);
    InDone(dev, ep);
    dev.Sof(10U, 5U);  // 2
    dev.Sof(11U, 1U);  // 2 + 4 = 6：提交，剩 2 / 6: submit, 2 left
    TEST_ASSERT(ep.starts.size() == 3U);
    InDone(dev, ep);
    dev.Sof(11U, 3U);  // 2 + 2 = 4：提交 / 4: submit
    TEST_ASSERT(ep.starts.size() == 4U);
    InDone(dev, ep);
    dev.Sof(13U, 3U);  // 16 个微帧：只提交一次 / 16 microframes: one submit only
    TEST_ASSERT(ep.starts.size() == 5U);
    InDone(dev, ep);
    dev.Sof(13U, 4U);  // 多出的已丢弃：1 < 4 / The surplus was dropped: 1 < 4
    TEST_ASSERT(ep.starts.size() == 5U);
    dev.Sof(2047U, 6U);  // 很多：提交一次，清零 / Many: one submit, cleared
    TEST_ASSERT(ep.starts.size() == 6U);
    InDone(dev, ep);
    dev.Sof(0U, 2U);  // 回绕后 4 个微帧 / 4 microframes across the wrap
    TEST_ASSERT(ep.starts.size() == 7U);
    InDone(dev, ep);

    // 重新选 alt 1 后上一次的帧号作废：第一个 SOF 只算 1。
    // After alt 1 is selected again the previous frame number is void: the first SOF
    // counts 1.
    ControlNone(dev, hw, SetInterface(1U, 0U));
    ControlNone(dev, hw, SetInterface(1U, 1U));
    TEST_ASSERT(ep.starts.size() == 8U);
    InDone(dev, ep);
    dev.Sof(0U, 6U);  // 1 < 4
    TEST_ASSERT(ep.starts.size() == 8U);
    // 只报整帧的后端：一帧算 8 个微帧 / A whole-frame backend: one frame is 8
    // microframes
    dev.Sof(1U, WHOLE);
    TEST_ASSERT(ep.starts.size() == 9U);
    InDone(dev, ep);
  }
}

/// 绑定失败 / Bind failures
void TestBindFailure()
{
  // 后端不支持等时（CheckHardware 报 NOT_SUPPORT）：SET_CONFIGURATION 被拒，端点交回
  // 池；修好后同一个配置可用。
  // A backend without isochronous support (NOT_SUPPORT from CheckHardware):
  // SET_CONFIGURATION is refused and the endpoint goes back to the pool; fixed, the
  // same configuration works.
  {
    Hw hw;
    hw.ep1_in.configure_result = ErrorCode::NOT_SUPPORT;
    Mic mic(EPNumber::EP1, 16000U);
    FakeUsb dev(hw, {{&mic}});
    dev.Init(false);
    dev.Reset();
    Refused(dev, hw, Make(0x00U, 9U, 1U, 0U, 0U));
    TEST_ASSERT(hw.ep1_in.configures == 0U && hw.ep1_in.closes == 0U);
    TEST_ASSERT(hw.ep1_in.GetState() == State::DISABLED);
    TEST_ASSERT(ControlIn(dev, hw, Make(0x80U, 8U, 0U, 0U, 1U)) == Bytes{0U});
    hw.ep1_in.configure_result = ErrorCode::OK;
    SetConfiguration(dev, hw, 1U);
    ControlNone(dev, hw, SetInterface(1U, 1U));
    TEST_ASSERT(hw.ep1_in.inflight && hw.ep1_in.LastLength() == 32U);
  }

  // Configure 选的包长与描述符不同（后端上限在查询和 Configure 之间变了）：CHECK_ERR，
  // 端点关闭并交回。
  // The size Configure chose differs from the descriptor (the backend's limit changed
  // between the query and Configure): CHECK_ERR, the endpoint is closed and returned.
  {
    Hw hw;
    Mic mic(EPNumber::EP1, 16000U);
    FakeUsb dev(hw, {{&mic}});
    dev.Init(false);
    dev.Reset();
    hw.ep1_in.shrink_after = hw.ep1_in.limit_calls + 1;
    Refused(dev, hw, Make(0x00U, 9U, 1U, 0U, 0U));
    TEST_ASSERT(hw.ep1_in.MaxPacketSize() == 32U && hw.ep1_in.closes == 1U);
    TEST_ASSERT(hw.ep1_in.GetState() == State::DISABLED);
    hw.ep1_in.shrink_after = -1;
    SetConfiguration(dev, hw, 1U);
    TEST_ASSERT(hw.ep1_in.MaxPacketSize() == 34U);
  }

  // 后端在查询里说没有等时（包长上限 0）：写描述符失败，GET_DESCRIPTOR 和
  // SET_CONFIGURATION 都被拒，端点没被碰。
  // The backend's query says no isochronous (packet limit 0): writing descriptors fails,
  // GET_DESCRIPTOR and SET_CONFIGURATION are both refused, the endpoint is untouched.
  {
    Hw hw;
    hw.ep1_in.no_iso = true;
    Mic mic(EPNumber::EP1, 16000U);
    FakeUsb dev(hw, {{&mic}});
    dev.Init(false);
    dev.Reset();
    Refused(dev, hw, Make(0x80U, 6U, 0x0200U, 0U, 255U));
    Refused(dev, hw, Make(0x00U, 9U, 1U, 0U, 0U));
    TEST_ASSERT(hw.ep1_in.configures == 0U);
  }

  // 48 kHz 双声道要 196 字节的包，槽只有 128 字节：NO_BUFF，同样不绑定。
  // 48 kHz stereo needs 196-byte packets and the slots hold 128 bytes: NO_BUFF, nothing
  // bound either.
  {
    Hw hw(256U);
    StereoMic mic(EPNumber::EP1, 48000U);
    FakeUsb dev(hw, {{&mic}});
    dev.Init(false);
    dev.Reset();
    Refused(dev, hw, Make(0x80U, 6U, 0x0200U, 0U, 255U));
    Refused(dev, hw, Make(0x00U, 9U, 1U, 0U, 0U));
    TEST_ASSERT(hw.ep1_in.configures == 0U);
  }

  // 端点没有登记 / The endpoint is not registered
  {
    Hw hw;
    Mic mic(EPNumber::EP9, 16000U);
    FakeUsb dev(hw, {{&mic}});
    dev.Init(false);
    dev.Reset();
    Refused(dev, hw, Make(0x00U, 9U, 1U, 0U, 0U));
    TEST_ASSERT(hw.ep1_in.configures == 0U);
  }

  // 两个麦克风用同一个端点：第二个 Get 失败，两个都解绑，第一个的端点交回。
  // Two microphones on one endpoint: the second Get fails, both are unbound and the
  // first one's endpoint goes back.
  {
    Hw hw;
    Mic first(EPNumber::EP1, 16000U);
    Mic second(EPNumber::EP1, 16000U);
    FakeUsb dev(hw, {{&first, &second}});
    dev.Init(false);
    dev.Reset();
    const Bytes config = GetDescriptor(dev, hw, 2U, 0U, 255U);
    TEST_ASSERT(config.size() == 9U + 2U * 108U && config[4] == 4U);
    Refused(dev, hw, Make(0x00U, 9U, 1U, 0U, 0U));
    TEST_ASSERT(hw.ep1_in.configures == 1U && hw.ep1_in.closes == 1U);
    TEST_ASSERT(hw.ep1_in.GetState() == State::DISABLED);
  }
}

/**
 * 带在途请求解绑：一个包在途、一个排队，队列里还有 PCM。四种解绑：配置 0、复位、同一配置
 * 再设一次、Deinit。
 * Unbind with requests in flight: one packet in flight and one queued, more PCM in the
 * queue. Four ways to unbind: configuration 0, reset, the same configuration again,
 * Deinit.
 */
void TestUnbindInFlight()
{
  for (int how = 0; how < 4; ++how)
  {
    Hw hw;
    Mic mic(EPNumber::EP1, 16000U);
    FakeUsb dev(hw, {{&mic}});
    Enumerate(dev, hw);
    FakeEp& ep = hw.ep1_in;
    ControlNone(dev, hw, SetInterface(1U, 1U));
    const Bytes pcm = Pattern(64U, 90U);
    TEST_ASSERT(mic.WritePcm(pcm.data(), pcm.size()) == ErrorCode::OK);
    dev.Sof(1U);
    TEST_ASSERT(ep.inflight && ep.starts.size() == 1U && mic.QueueSize() == 32U);
    const size_t stops = ep.stops;
    const size_t closes = ep.closes;

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

    // 停一次、关一次（同一配置再设时绑定又关一次）；两个 ABORTED 不计为失败；主机没收到
    // 包；队列里的 PCM 保留。
    // One stop, one close (one more from the bind when the same configuration is set
    // again); the two ABORTED are no failures; the host got no packet; the queued PCM
    // is kept.
    TEST_ASSERT(ep.stops == stops + 1U);
    TEST_ASSERT(ep.closes == closes + (how == 2 ? 2U : 1U));
    TEST_ASSERT(!ep.inflight && ep.packets.empty() && ep.starts.size() == 1U);
    TEST_ASSERT(mic.MissedTransferCount() == 0U && mic.QueueSize() == 32U);
    TEST_ASSERT(ep.GetState() == State::DISABLED);
    if (how != 2)
    {
      dev.Sof(2U);
      TEST_ASSERT(ep.starts.size() == 1U);
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

    // 重新绑定后在 alt 0；alt 1 先发出队列里剩下的 32 字节，两个槽都可用。
    // After the new bind at alt 0; alt 1 first sends the remaining 32 bytes, both slots
    // are usable.
    TEST_ASSERT(GetInterface(dev, hw, 1U) == 0U);
    dev.Sof(3U);
    TEST_ASSERT(ep.starts.size() == 1U);
    ControlNone(dev, hw, SetInterface(1U, 1U));
    TEST_ASSERT(ep.inflight && ep.starts.size() == 2U);
    dev.Sof(4U);
    dev.Sof(5U);
    TEST_ASSERT(mic.DroppedFrameCount() == 1U);
    InDone(dev, ep);
    TEST_ASSERT(ep.packets.back() == Bytes(pcm.begin() + 32, pcm.end()));
    InDone(dev, ep);
    TEST_ASSERT(ep.packets.back() == Zeros(32U) && !ep.inflight);
  }
}

/**
 * SET_INTERFACE 重新配置端点的各条路径：流在途时再选 alt 1、重新配置时包长不符、
 * Configure 出错、停止失败、已绑定时读 OTHER_SPEED 描述符。
 * Paths of the endpoint reconfiguration on SET_INTERFACE: alt 1 again while streaming,
 * a size mismatch at the reconfiguration, a Configure error, a failed stop, an
 * OTHER_SPEED descriptor read while bound.
 */
void TestSetInterface()
{
  // 流在途（一个在途、一个排队）时再选 alt 1：同步停止，两个 ABORTED，重新配置并提交
  // 第一个包；状态阶段不推迟。
  // Alt 1 again while streaming (one in flight, one queued): synchronous stop, two
  // ABORTED, reconfigured and the first packet submitted; the status stage is not
  // deferred.
  {
    Hw hw;
    Mic mic(EPNumber::EP1, 16000U);
    FakeUsb dev(hw, {{&mic}});
    Enumerate(dev, hw);
    FakeEp& ep = hw.ep1_in;
    ControlNone(dev, hw, SetInterface(1U, 1U));
    dev.Sof(1U);
    TEST_ASSERT(ep.inflight && ep.starts.size() == 1U);
    ControlNone(dev, hw, SetInterface(1U, 1U));
    TEST_ASSERT(ep.stops == 1U && ep.closes == 2U && ep.configures == 3U);
    // 切换中的停止、关闭和启动拿到的是中断上下文 / The stop, close and start of the
    // switch got the interrupt context
    TEST_ASSERT(ep.all_isr);
    TEST_ASSERT(ep.inflight && ep.starts.size() == 2U && ep.GetState() == State::IDLE);
    TEST_ASSERT(mic.MissedTransferCount() == 0U && GetInterface(dev, hw, 1U) == 1U);
    InDone(dev, ep);
    TEST_ASSERT(ep.packets.size() == 1U);
  }

  // 重新配置时包长变了：CHECK_ERR，请求 STALL，端点关闭，alt 读回 0；修好后可用。
  // The size changed at the reconfiguration: CHECK_ERR, the request stalls, the
  // endpoint is closed and alt reads 0; fixed, it works again.
  {
    Hw hw;
    Mic mic(EPNumber::EP1, 16000U);
    FakeUsb dev(hw, {{&mic}});
    Enumerate(dev, hw);
    FakeEp& ep = hw.ep1_in;
    ControlNone(dev, hw, SetInterface(1U, 1U));
    ep.shrink_after = ep.limit_calls;
    Refused(dev, hw, SetInterface(1U, 1U));
    TEST_ASSERT(ep.MaxPacketSize() == 32U && ep.GetState() == State::DISABLED);
    TEST_ASSERT(ep.stops == 1U && ep.closes == 3U && !ep.inflight);
    TEST_ASSERT(GetInterface(dev, hw, 1U) == 0U);
    dev.Sof(1U);
    TEST_ASSERT(ep.starts.size() == 1U);
    ep.shrink_after = -1;
    ControlNone(dev, hw, SetInterface(1U, 1U));
    TEST_ASSERT(ep.MaxPacketSize() == 34U && ep.inflight && ep.starts.size() == 2U);
  }

  // 重新配置时 Configure 出错：STALL，alt 读回 0；修好后可用。
  // Configure fails at the reconfiguration: stall, alt reads 0; fixed, it works.
  {
    Hw hw;
    Mic mic(EPNumber::EP1, 16000U);
    FakeUsb dev(hw, {{&mic}});
    Enumerate(dev, hw);
    FakeEp& ep = hw.ep1_in;
    ep.configure_result = ErrorCode::FAILED;
    Refused(dev, hw, SetInterface(1U, 1U));
    TEST_ASSERT(ep.GetState() == State::DISABLED && GetInterface(dev, hw, 1U) == 0U);
    ep.configure_result = ErrorCode::OK;
    ControlNone(dev, hw, SetInterface(1U, 1U));
    TEST_ASSERT(ep.inflight);
  }

  // 切换时停止挂起：Close 不等待，Configure 照常成功，硬件动作推迟，请求当场应答；之后
  // 后端把停止报告为超时，设备复位控制器、关闭所有端点、Deinitialize，类解绑。重新枚举
  // 后可用。
  // The stop stays pending during the switch: Close does not wait, Configure succeeds,
  // the hardware steps are deferred and the request is answered at once; later the
  // backend reports the stop as timed out, the device resets the controller, closes
  // every endpoint and deinitializes, and the class is unbound. After a new
  // enumeration it works.
  {
    Hw hw;
    Mic mic(EPNumber::EP1, 16000U);
    FakeUsb dev(hw, {{&mic}});
    Enumerate(dev, hw);
    FakeEp& ep = hw.ep1_in;
    ControlNone(dev, hw, SetInterface(1U, 1U));
    ep.stuck = true;
    const size_t configures = ep.configures;
    const size_t starts = ep.starts.size();
    ControlNone(dev, hw, SetInterface(1U, 1U));
    TEST_ASSERT(dev.resets == 0U && dev.Inited());
    TEST_ASSERT(ep.stops == 1U && ep.configures == configures);
    TEST_ASSERT(ep.GetState() == State::IDLE && ep.starts.size() == starts);
    dev.Irq([&ep](bool isr) { ep.TimedOut(isr); });
    TEST_ASSERT(dev.resets == 1U && !dev.Inited());
    TEST_ASSERT(ep.stops == 2U && !ep.inflight && ep.configures == configures);
    TEST_ASSERT(ep.starts.size() == starts);
    TEST_ASSERT(ep.GetState() == State::DISABLED && mic.MissedTransferCount() == 0U);
    ep.stuck = false;
    Enumerate(dev, hw);
    ControlNone(dev, hw, SetInterface(1U, 1U));
    TEST_ASSERT(ep.inflight && ep.starts.size() == 2U);
  }

  // 高速已绑定时主机读 OTHER_SPEED（全速 34 字节）：类的描述符块被改写，SET_INTERFACE
  // 仍按绑定时的 6 字节比较。
  // At high speed and bound, the host reads OTHER_SPEED (34 bytes at full speed): the
  // class's descriptor block is rewritten, SET_INTERFACE still compares with the 6
  // bytes of the bind.
  {
    Hw hw;
    // interval 1：高速的包长（6）与全速的（34）不同 / interval 1: the high-speed packet
    // size (6) differs from the full-speed one (34)
    Mic mic(EPNumber::EP1, 16000U, 0, 0, 1, Speed::HIGH, 2048U, 1U);
    FakeUsb dev(hw, {{&mic}}, Speed::HIGH);
    Enumerate(dev, hw, Speed::HIGH);
    const Bytes other = GetDescriptor(dev, hw, 7U, 0U, 255U);
    TEST_ASSERT(Le16(other, 105) == 34U);
    ControlNone(dev, hw, SetInterface(1U, 1U));
    TEST_ASSERT(hw.ep1_in.MaxPacketSize() == 6U && hw.ep1_in.LastLength() == 4U);
  }
}

/**
 * 一个线程写 PCM（大小不一的块，满了就让出），一个线程扮控制器中断（SOF，然后主机取走
 * 在途的包）。主机收到的非零采样必须正好是写入的序列，每个包里零只出现在末尾。
 * One thread writes PCM (chunks of varying size, yielding when full), one plays the
 * controller interrupt (SOF, then the host takes the packet in flight). The non-zero
 * samples the host receives must be exactly the written sequence, and zeros appear only
 * at the end of a packet.
 */
void TestThreads()
{
  constexpr uint16_t SAMPLES = 20000U;
  Hw hw;
  Mic mic(EPNumber::EP1, 16000U, 0, 0, 1, Speed::FULL, 256U);
  FakeUsb dev(hw, {{&mic}});
  Enumerate(dev, hw);
  ControlNone(dev, hw, SetInterface(1U, 1U));
  FakeEp& ep = hw.ep1_in;

  std::atomic<bool> written{false};
  std::thread producer(
      [&]()
      {
        Bytes all;
        for (uint16_t s = 1; s <= SAMPLES; ++s)
        {
          all.push_back(static_cast<uint8_t>(s));
          all.push_back(static_cast<uint8_t>(s >> 8U));
        }
        size_t at = 0;
        size_t step = 1;
        while (at < all.size())
        {
          const size_t chunk = std::min(all.size() - at, 1U + (step * 37U) % 61U);
          if (mic.WritePcm(all.data() + at, chunk) == ErrorCode::OK)
          {
            at += chunk;
            ++step;
          }
          else
          {
            std::this_thread::yield();
          }
        }
        written.store(true, std::memory_order_release);
      });

  std::thread interrupt(
      [&]()
      {
        uint16_t frame = 0;
        size_t idle = 0;
        for (size_t guard = 0; guard < 5000000U && idle < 50U; ++guard)
        {
          dev.Sof(frame);
          frame = static_cast<uint16_t>((frame + 1U) & 0x7ffU);
          dev.Irq(
              [&ep](bool isr)
              {
                if (ep.inflight)
                {
                  ep.Done(isr, ep.LastLength());
                }
              });
          const bool drained =
              written.load(std::memory_order_acquire) && mic.QueueSize() < 2U;
          idle = drained ? idle + 1U : 0U;
          if (!drained)
          {
            std::this_thread::yield();
          }
        }
      });

  producer.join();
  interrupt.join();

  std::vector<uint16_t> samples;
  for (const Bytes& packet : ep.packets)
  {
    TEST_ASSERT(packet.size() == 32U);
    bool zero_seen = false;
    for (size_t i = 0; i < packet.size(); i += 2U)
    {
      const uint16_t sample = Le16(packet, i);
      if (sample == 0U)
      {
        zero_seen = true;
        continue;
      }
      TEST_ASSERT(!zero_seen);
      samples.push_back(sample);
    }
  }
  TEST_ASSERT(samples.size() == SAMPLES);
  for (size_t i = 0; i < samples.size(); ++i)
  {
    TEST_ASSERT(samples[i] == i + 1U);
  }
  TEST_ASSERT(mic.DroppedFrameCount() == 0U && mic.MissedTransferCount() == 0U);
}

/// 后端可能给的槽数：核心默认（两个）、1、3、8 / Slot counts a backend may give: the core
/// default (two), 1, 3 and 8
constexpr uint8_t SLOT_COUNTS[] = {0U, 1U, 3U, 8U};

/**
 * @brief 默认、1、3、8 个槽下主机不取包时的丢区间 / Dropped intervals while the host
 *        takes no packet, with the default, 1, 3 and 8 slots
 * @note 每个槽装一个区间，槽都占着时之后的区间丢弃并计数；主机取走后照常提交。
 *       Every slot carries one interval; with all of them taken the following intervals
 *       are dropped and counted; once the host took them, submitting goes on.
 */
void TestSlotCounts()
{
  for (uint8_t slots : SLOT_COUNTS)
  {
    const size_t count = slots == 0U ? 2U : slots;
    Hw hw(512U, slots);
    Mic mic(EPNumber::EP1, 16000U);
    FakeUsb dev(hw, {{&mic}});
    Enumerate(dev, hw);
    FakeEp& ep = hw.ep1_in;
    ControlNone(dev, hw, SetInterface(1U, 1U));
    for (uint16_t frame = 1U; frame <= count + 1U; ++frame)
    {
      dev.Sof(frame);
    }
    TEST_ASSERT(mic.DroppedFrameCount() == 2U && ep.starts.size() == 1U);
    for (size_t guard = 0; ep.inflight; ++guard)
    {
      TEST_ASSERT(guard < 16U);
      InDone(dev, ep);
    }
    TEST_ASSERT(ep.packets.size() == count && ep.starts.size() == count);
    dev.Sof(static_cast<uint16_t>(count + 2U));
    TEST_ASSERT(ep.inflight && mic.DroppedFrameCount() == 2U);
    InDone(dev, ep);
  }
}

}  // namespace

void test_usb_uac_enum() { TestEnumeration(); }
void test_usb_uac_stream() { TestStream(); }
void test_usb_uac_requests() { TestRequests(); }
void test_usb_uac_pacing() { TestPacing(); }
void test_usb_uac_bind_fail() { TestBindFailure(); }
void test_usb_uac_unbind() { TestUnbindInFlight(); }
void test_usb_uac_set_interface() { TestSetInterface(); }
void test_usb_uac_threads() { TestThreads(); }
void test_usb_uac_slots() { TestSlotCounts(); }
