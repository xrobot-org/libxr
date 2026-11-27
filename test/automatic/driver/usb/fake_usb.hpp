/**
 * @file fake_usb.hpp
 * @brief USB 设备类与设备核心的共用假后端 / Shared fake backend for USB classes and
 *        DeviceCore tests
 *
 * 假后端同时实现 DeviceService 与 DeviceCore，把控制器硬件事件排进队列后经
 * OnControllerInterrupt() 交给核心处理；FakeEp 记录每次后端调用并由测试决定请求何时
 * 完成、停止返回什么，以及主机侧控制与批量/中断/等时传输辅助函数。
 * The fake backend derives from DeviceService and DeviceCore, queueing controller
 * hardware events before calling OnControllerInterrupt(). FakeEp records every backend
 * call and lets tests control completion timing and stop outcomes, paired with host-side
 * control and data endpoint helpers.
 */

#pragma once

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <deque>
#include <functional>
#include <initializer_list>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

#include "dev_core.hpp"
#include "device_class.hpp"
#include "ep.hpp"
#include "ep_pool.hpp"
#include "test_assert.hpp"
#include "usb_service.hpp"

namespace FakeUsbTest
{
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
using LibXR::USB::USBSpec;
using Direction = Endpoint::Direction;
using Completion = Endpoint::Completion;
using EPNumber = Endpoint::EPNumber;
using Type = Endpoint::Type;
using Bytes = std::vector<uint8_t>;
using Configs = std::initializer_list<const std::initializer_list<DeviceClass*>>;

static_assert(LibXR::HW_CACHE_LINE_SIZE <= 64U, "test memory is aligned to 64 bytes");

inline constexpr auto DEFAULT_LANGUAGE = DescriptorStrings::MakeLanguagePack(
    DescriptorStrings::Language::EN_US, "XRobot", "Core", "0001");

/// 后端可能给的槽数：核心默认（两个）、1、3、8 / Slot counts a backend may give: the core
/// default (two), 1, 3 and 8
inline constexpr uint8_t SLOT_COUNTS[] = {0U, 1U, 3U, 8U};

/// 数据端点的实际槽数 / The actual slot count of a data endpoint
inline constexpr size_t SlotsOf(uint8_t slots) { return slots == 0U ? 2U : slots; }

inline constexpr uint8_t WHOLE_FRAME = 0xFFU;  ///< 只报整帧 / Whole frames only

/// 仅测试用：记账期间暂停零分配断言计数 / Test-only: suspend allocation counting while
/// recording test bookkeeping
inline thread_local bool* g_alloc_counting_flag = nullptr;

struct ScopedNoAllocCount
{
  ScopedNoAllocCount()
      : flag_(g_alloc_counting_flag), saved_(flag_ != nullptr ? *flag_ : false)
  {
    if (flag_ != nullptr)
    {
      *flag_ = false;
    }
  }
  ~ScopedNoAllocCount()
  {
    if (flag_ != nullptr)
    {
      *flag_ = saved_;
    }
  }
  ScopedNoAllocCount(const ScopedNoAllocCount&) = delete;
  ScopedNoAllocCount& operator=(const ScopedNoAllocCount&) = delete;

  bool* flag_;
  bool saved_;
};

/// 端点内存 / Endpoint memory
template <size_t Size = 1024U>
struct alignas(64) AlignedMemory
{
  uint8_t bytes[Size];
};

/// 记录后端调用、由测试决定完成的假端点 / Fake endpoint that records calls and lets tests
/// control completion
class FakeEp final : public Endpoint
{
 public:
  using Endpoint::StopOutcome;

  /// 一次 StartHardware / One StartHardware
  struct Start
  {
    size_t length;
    uint8_t* data;
    size_t epoch;  ///< 启动时 Configure 成功过几次 / Successful Configures at the start
    Bytes bytes;   ///< IN：启动时的数据 / IN: content at the start
  };

  /// slots：槽数，0 为核心默认 / slots: slot count, 0 for the core default
  FakeEp(EPNumber number, Direction direction, RawData memory, uint8_t slots = 0U)
      : Endpoint(number, direction, memory, slots)
  {
    // 只有一个槽时硬件不直接读写槽（核心对单槽的要求）。
    // With one slot the hardware does not access the slot directly (what the core
    // requires of a single slot).
    limits.direct_memory = (slots != 1U);
  }

  /// 硬件完成在途请求；只在 owner 中 / The hardware completes the in-flight request;
  /// owner only
  void Done(bool in_isr, size_t actual, ErrorCode result = ErrorCode::OK)
  {
    TEST_ASSERT(inflight);
    inflight = false;
    if (GetConfig().direction == Direction::IN && result == ErrorCode::OK)
    {
      ScopedNoAllocCount quiet;
      const uint8_t* data = starts.back().data;
      wire.insert(wire.end(), data, data + actual);
      wire_bytes.store(wire.size(), std::memory_order_release);
      transfers.emplace_back(data, data + actual);
      transfer_count.store(transfers.size(), std::memory_order_release);
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
  const Bytes& LastBytes() const { return starts.back().bytes; }

  /// 在途请求的字节 / Bytes of the request in flight
  Bytes InFlight() const
  {
    TEST_ASSERT(inflight);
    return Bytes(starts.back().data, starts.back().data + starts.back().length);
  }

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
  std::vector<std::thread::id> start_threads;  ///< 每次启动的线程 / Thread of each start
  Bytes wire;  ///< 主机取走的 IN 连续字节流 / Flat IN stream
  std::atomic<size_t> wire_bytes{0U};
  std::vector<Bytes> transfers;  ///< 主机取走的 IN 每次传输 / IN transfers the host took
  std::atomic<size_t> transfer_count{0U};
  bool inflight = false;
  size_t configures = 0;  ///< 成功的 ConfigureHardware / Successful ConfigureHardware
  size_t closes = 0;
  size_t stops = 0;
  size_t stalls = 0;
  size_t clears = 0;
  ErrorCode configure_result = ErrorCode::OK;
  bool complete_in_start = false;  ///< IN 在 StartHardware 内完成 / IN completes inside
  HwLimits limits{};    ///< HardwareLimits 的返回值 / What HardwareLimits returns
  bool no_iso = false;  ///< 硬件不支持等时 / No isochronous support in hardware
  /// HardwareLimits 第几次调用之后报 32 字节包 / After this many HardwareLimits calls,
  /// report 32-byte packets
  int shrink_after = -1;
  mutable int limit_calls = 0;
  /// 停止、关闭、启动是否都带 in_isr=true / Whether every stop, close and start had
  /// in_isr=true
  bool all_isr = true;
  /// StopHardware 依次返回；空时 ABORTED / Returned by StopHardware in turn; ABORTED
  /// when empty
  std::deque<StopOutcome> stop_outcomes;
  bool stuck = false;  ///< 停止一直挂起 / The stop stays pending
  bool reset = false;  ///< 控制器已复位，停止立即成功 / Controller reset, stops succeed
  /// 控制器锁存了一个尚未报告的 SETUP / The controller latched a SETUP not yet
  /// reported
  bool setup_latched = false;
  /// 撤回要等线上一包：装入当作挂起的停止 / The retract waits for a packet on the
  /// wire: the load counts as a pending stop
  bool retract_on_wire = false;
  bool retracting = false;  ///< 挂起的撤回 / A retract is pending
  size_t retracts = 0;      ///< 撤回的装入 / Loads retracted

 protected:
  ErrorCode CheckHardware(const Config&) const override { return configure_result; }

  void ConfigureHardware(const Config&) override
  {
    ScopedNoAllocCount quiet;
    ++configures;
  }

  void CloseHardware(bool in_isr) override
  {
    ScopedNoAllocCount quiet;
    ++closes;
    all_isr = all_isr && in_isr;
  }

  ErrorCode StartHardware(const HwRequest& request, bool in_isr) override
  {
    {
      ScopedNoAllocCount quiet;
      TEST_ASSERT(!inflight);
      all_isr = all_isr && in_isr;
      if (GetConfig().type == Type::CONTROL && setup_latched)
      {
        // 先装入、再复读：有未报告的 SETUP 就撤回，这次装入不上线 / Load, then re-read:
        // with an unreported SETUP the load is retracted and never goes on the wire
        ++retracts;
        if (retract_on_wire)
        {
          retracting = true;
          inflight = true;
          return ErrorCode::OK;
        }
        return ErrorCode::FAILED;
      }
      Bytes bytes;
      if (GetConfig().direction == Direction::IN && request.length != 0U)
      {
        bytes.assign(request.data, request.data + request.length);
      }
      starts.push_back({request.length, request.data, configures, std::move(bytes)});
      start_threads.push_back(std::this_thread::get_id());
      inflight = true;
    }
    if (complete_in_start && GetConfig().direction == Direction::IN)
    {
      Done(in_isr, request.length);
    }
    return ErrorCode::OK;
  }

  StopOutcome StopHardware(bool in_isr) override
  {
    ScopedNoAllocCount quiet;
    ++stops;
    all_isr = all_isr && in_isr;
    if (reset)
    {
      inflight = false;
      return {ErrorCode::ABORTED, 0U};
    }
    if (stuck)
    {
      return {ErrorCode::PENDING, 0U};
    }
    StopOutcome outcome{ErrorCode::ABORTED, 0U};
    if (!stop_outcomes.empty())
    {
      outcome = stop_outcomes.front();
      stop_outcomes.pop_front();
    }
    if (outcome.result != ErrorCode::PENDING)
    {
      inflight = false;
    }
    return outcome;
  }

  ErrorCode StallHardware(bool) override
  {
    ScopedNoAllocCount quiet;
    ++stalls;
    return ErrorCode::OK;
  }

  ErrorCode ClearStallHardware(bool) override
  {
    ScopedNoAllocCount quiet;
    ++clears;
    return ErrorCode::OK;
  }

  HwLimits HardwareLimits(Direction, Type type) const override
  {
    ++limit_calls;
    if (no_iso && type == Type::ISOCHRONOUS)
    {
      HwLimits out = {0U, SIZE_MAX, SIZE_MAX};
      out.direct_memory = limits.direct_memory;
      return out;
    }
    if (shrink_after >= 0 && limit_calls > shrink_after)
    {
      HwLimits out = {32U, SIZE_MAX, SIZE_MAX};
      out.direct_memory = limits.direct_memory;
      return out;
    }
    return limits;
  }
};

/// 假端点池基类：内置 EP0，各用例追加所需数据端点 / Base endpoint pool with EP0; each
/// test adds its data endpoints
struct FakeHw
{
  FakeHw() { pool.SetEndpoint0(&in0, &out0); }

  void Add(FakeEp& ep)
  {
    TEST_ASSERT(pool.Put(&ep) == ErrorCode::OK);
    extra_eps.push_back(&ep);
  }

  std::vector<FakeEp*> All()
  {
    std::vector<FakeEp*> all = {&in0, &out0};
    all.insert(all.end(), extra_eps.begin(), extra_eps.end());
    return all;
  }

  AlignedMemory<64U> ep0_mem[2];
  FakeEp in0{EPNumber::EP0, Direction::IN, {ep0_mem[0].bytes, 64U}};
  FakeEp out0{EPNumber::EP0, Direction::OUT, {ep0_mem[1].bytes, 64U}};
  EndpointPool pool;
  std::vector<FakeEp*> extra_eps;
};

/// 假后端设备 / Fake backend device
class FakeUsb : public DeviceService, public DeviceCore
{
 public:
  FakeUsb(FakeHw& hw, const Configs& configs, Speed speed = Speed::FULL,
          DeviceDescriptor::PacketSize0 ep0 = DeviceDescriptor::PacketSize0::SIZE_64,
          USBSpec spec = USBSpec::USB_2_0, uint16_t pid = 0x0001U,
          const DescriptorStrings::LanguagePack* language = &DEFAULT_LANGUAGE)
      : DeviceCore(*this, hw.pool, spec, speed, ep0, 0x1209U, pid, 0x0100U, {language},
                   configs),
        eps_(hw.All())
  {
    for (FakeEp* ep : eps_)
    {
      TEST_ASSERT(Register(*ep) == ErrorCode::OK);
    }
  }

  FakeUsb(FakeHw& hw, const Configs& configs, Speed speed, USBSpec spec,
          uint16_t pid = 0x0001U)
      : FakeUsb(hw, configs, speed, DeviceDescriptor::PacketSize0::SIZE_64, spec, pid)
  {
  }

  FakeUsb(FakeHw& hw, const Configs& configs, USBSpec spec, uint16_t pid = 0x0001U)
      : FakeUsb(hw, configs, Speed::FULL, DeviceDescriptor::PacketSize0::SIZE_64, spec,
                pid)
  {
  }

  /// 排一个硬件事件并进入控制器中断 / Queue one hardware event and enter the
  /// controller interrupt
  template <typename Event>
  void Irq(Event&& event)
  {
    {
      ScopedNoAllocCount quiet;
      std::function<void(bool)> fn(std::forward<Event>(event));
      std::unique_lock<std::mutex> lock(mutex_);
      cv_.wait(lock,
               [&]
               {
                 return events_.size() < 8U ||
                        servicing_thread_ == std::this_thread::get_id();
               });
      events_.push_back(std::move(fn));
    }
    OnControllerInterrupt();
  }

  void Setup(SetupPacket setup)
  {
    Irq([this, setup](bool isr) { HwSetup(isr, setup); });
  }

  /// 在硬件事件处理中当场报告 SETUP；只在 owner 中 / Report a SETUP at once from
  /// the hardware event handling; owner only
  void HwSetup(bool in_isr, const SetupPacket& setup)
  {
    NotifyLog("setup");
    OnSetup(in_isr, setup);
  }

  void Reset(Speed speed = Speed::FULL)
  {
    Irq([this, speed](bool isr) { OnBusReset(isr, speed); });
  }
  void Suspend()
  {
    Irq([this](bool isr) { OnSuspend(isr); });
  }
  void Resume()
  {
    Irq([this](bool isr) { OnResume(isr); });
  }
  void Disconnect()
  {
    Irq([this](bool isr) { OnDisconnect(isr); });
  }
  void Sof(uint16_t frame, uint8_t microframe = WHOLE_FRAME)
  {
    Irq([this, frame, microframe](bool isr) { OnStartOfFrame(isr, frame, microframe); });
  }

  bool Inited() const { return IsInited(); }

  /**
   * @brief 硬件在本轮读过状态之后锁存一个事件 / The hardware latches an event after
   *        this pass read its status
   * @note 中断线由 owner 屏蔽着，中断在当前 owner 返回之后才进入（Fire），下一次
   *       ServiceHardware 才读到它。
   *       The owner keeps the line masked, so the interrupt is taken only after the
   *       current owner returns (Fire), and the next ServiceHardware reads it.
   */
  void Latch(std::function<void(bool)> event)
  {
    ScopedNoAllocCount quiet;
    std::lock_guard<std::mutex> lock(mutex_);
    next_.push_back(std::move(event));
  }

  /// 进入控制器中断，不带新事件 / Enter the controller interrupt without a new event
  void Fire() { OnControllerInterrupt(); }

  void StartController(bool) override {}
  void StopController(bool) override {}

  std::vector<std::pair<uint8_t, AddressStage>> addresses;
  std::atomic<size_t> ready{0U};
  size_t resets = 0;
  std::atomic<bool> masked{false};
  std::function<void(const char*)> on_log;

 protected:
  void ServiceHardware(bool in_isr) override
  {
    std::vector<std::function<void(bool)>> batch;
    {
      ScopedNoAllocCount quiet;
      std::lock_guard<std::mutex> lock(mutex_);
      servicing_thread_ = std::this_thread::get_id();
      if (!next_.empty())
      {
        for (auto& event : next_)
        {
          events_.push_back(std::move(event));
        }
        next_.clear();
      }
      batch.swap(events_);
    }
    cv_.notify_all();
    for (auto& event : batch)
    {
      event(in_isr);
    }
    {
      ScopedNoAllocCount quiet;
      std::lock_guard<std::mutex> lock(mutex_);
      servicing_thread_ = std::thread::id{};
    }
    ScopedNoAllocCount quiet;
    batch.clear();
  }

  void MaskInterrupt() override { masked.store(true, std::memory_order_release); }
  void UnmaskInterrupt() override { masked.store(false, std::memory_order_release); }

  void SetAddress(uint8_t address, AddressStage stage) override
  {
    ScopedNoAllocCount quiet;
    addresses.emplace_back(address, stage);
    NotifyLog(stage == AddressStage::BEFORE_STATUS ? "address-before" : "address-after");
  }

  void OnControlReady() override
  {
    ready.fetch_add(1U, std::memory_order_relaxed);
    NotifyLog("ready");
  }

  void ResetController(bool) override
  {
    ScopedNoAllocCount quiet;
    ++resets;
    for (FakeEp* ep : eps_)
    {
      ep->reset = true;
    }
    NotifyLog("controller-reset");
  }

 private:
  void NotifyLog(const char* entry)
  {
    if (on_log)
    {
      ScopedNoAllocCount quiet;
      on_log(entry);
    }
  }

  std::vector<FakeEp*> eps_;
  std::mutex mutex_;
  std::condition_variable cv_;
  std::thread::id servicing_thread_{};
  std::vector<std::function<void(bool)>> events_;
  std::vector<std::function<void(bool)>> next_;
};

// ---- 主机一侧 / The host side ----

inline SetupPacket Make(uint8_t type, uint8_t request, uint16_t value, uint16_t index,
                        uint16_t length)
{
  return SetupPacket{type, request, value, index, length};
}

inline bool Stalled(FakeHw& hw) { return hw.in0.IsStalled() && hw.out0.IsStalled(); }

inline void InDone(FakeUsb& dev, FakeEp& ep)
{
  TEST_ASSERT(ep.inflight);
  const size_t length = ep.LastLength();
  dev.Irq([&ep, length](bool isr) { ep.Done(isr, length); });
}

inline void InDone(FakeUsb& dev, FakeHw& hw) { InDone(dev, hw.in0); }

inline void Drain(FakeUsb& dev, FakeEp& ep)
{
  for (size_t guard = 0; ep.inflight; ++guard)
  {
    TEST_ASSERT(guard < 100000U);
    InDone(dev, ep);
  }
}

inline void HostOut(FakeUsb& dev, FakeEp& ep, const Bytes& bytes)
{
  TEST_ASSERT(ep.inflight);
  dev.Irq([&ep, bytes](bool isr) { ep.HostSends(isr, bytes); });
}

inline void StatusOut(FakeUsb& dev, FakeHw& hw)
{
  dev.Irq([&hw](bool isr) { hw.out0.HostSends(isr, {}); });
}

/// 收完 IN 数据阶段（含结尾零长度包），返回收到的字节 / Take the whole IN data stage
/// (closing zero-length packet included) and return the bytes
inline Bytes TakeIn(FakeUsb& dev, FakeHw& hw, std::vector<size_t>* lengths = nullptr)
{
  Bytes got;
  while (hw.in0.inflight)
  {
    const FakeEp::Start& start = hw.in0.starts.back();
    got.insert(got.end(), start.bytes.begin(), start.bytes.end());
    if (lengths != nullptr)
    {
      lengths->push_back(start.length);
    }
    InDone(dev, hw.in0);
  }
  return got;
}

inline Bytes ControlIn(FakeUsb& dev, FakeHw& hw, SetupPacket setup,
                       std::vector<size_t>* lengths, bool finish = true)
{
  const size_t ready = dev.ready.load();
  dev.Setup(setup);
  TEST_ASSERT(!Stalled(hw));
  TEST_ASSERT(hw.out0.inflight && hw.out0.LastLength() == 0U);
  Bytes got = TakeIn(dev, hw, lengths);
  TEST_ASSERT(dev.ready.load() == ready);
  if (finish)
  {
    StatusOut(dev, hw);
    TEST_ASSERT(dev.ready.load() == ready + 1U);
  }
  return got;
}

inline Bytes ControlIn(FakeUsb& dev, FakeHw& hw, SetupPacket setup, bool finish = true)
{
  return ControlIn(dev, hw, setup, nullptr, finish);
}

/**
 * OUT 请求，数据按 EP0 包长分包；返回 false 表示数据阶段之后被 STALL。finish 为 false 时
 * 状态 IN 留在途。
 * OUT request with the data split by EP0 packet size; false means it stalled after the
 * data stage. Without finish the status IN stays in flight.
 */
inline bool TryControlOut(FakeUsb& dev, FakeHw& hw, SetupPacket setup, const Bytes& data,
                          bool finish = true)
{
  const size_t ready = dev.ready.load();
  dev.Setup(setup);
  TEST_ASSERT(!Stalled(hw));
  const size_t packet = hw.out0.MaxPacketSize();
  for (size_t offset = 0; offset < data.size(); offset += packet)
  {
    TEST_ASSERT(hw.out0.inflight && hw.out0.LastLength() == packet);
    const size_t n = (data.size() - offset < packet) ? data.size() - offset : packet;
    const Bytes chunk(data.begin() + static_cast<std::ptrdiff_t>(offset),
                      data.begin() + static_cast<std::ptrdiff_t>(offset + n));
    HostOut(dev, hw.out0, chunk);
  }
  if (Stalled(hw))
  {
    TEST_ASSERT(dev.ready.load() == ready);
    return false;
  }
  TEST_ASSERT(hw.in0.inflight && hw.in0.LastLength() == 0U);
  if (finish)
  {
    InDone(dev, hw.in0);
    TEST_ASSERT(dev.ready.load() == ready + 1U);
  }
  return true;
}

inline void ControlOut(FakeUsb& dev, FakeHw& hw, SetupPacket setup, const Bytes& data)
{
  TEST_ASSERT(TryControlOut(dev, hw, setup, data, true));
}

inline void ControlNone(FakeUsb& dev, FakeHw& hw, SetupPacket setup, bool finish = true)
{
  const size_t ready = dev.ready.load();
  dev.Setup(setup);
  TEST_ASSERT(!Stalled(hw));
  TEST_ASSERT(hw.in0.inflight && hw.in0.LastLength() == 0U);
  if (finish)
  {
    InDone(dev, hw.in0);
    TEST_ASSERT(dev.ready.load() == ready + 1U);
  }
}

inline void Refused(FakeUsb& dev, FakeHw& hw, SetupPacket setup)
{
  const size_t ready = dev.ready.load();
  dev.Setup(setup);
  TEST_ASSERT(Stalled(hw));
  TEST_ASSERT(dev.ready.load() == ready);
}

inline void RefusedData(FakeUsb& dev, FakeHw& hw, SetupPacket setup, const Bytes& data)
{
  TEST_ASSERT(!TryControlOut(dev, hw, setup, data, true));
}

inline Bytes GetDescriptor(FakeUsb& dev, FakeHw& hw, uint8_t type, uint8_t index,
                           uint16_t length, uint16_t lang = 0U)
{
  return ControlIn(
      dev, hw, Make(0x80U, 6U, static_cast<uint16_t>(type << 8U | index), lang, length));
}

inline void SetAddress(FakeUsb& dev, FakeHw& hw, uint8_t address = 7U)
{
  ControlNone(dev, hw, Make(0x00U, 5U, address, 0U, 0U));
}

inline void SetConfiguration(FakeUsb& dev, FakeHw& hw, uint8_t value)
{
  ControlNone(dev, hw, Make(0x00U, 9U, value, 0U, 0U));
}

inline uint8_t GetConfiguration(FakeUsb& dev, FakeHw& hw)
{
  const Bytes got = ControlIn(dev, hw, Make(0x80U, 8U, 0U, 0U, 1U));
  TEST_ASSERT(got.size() == 1U);
  return got[0];
}

inline void SetHalt(FakeUsb& dev, FakeHw& hw, uint8_t ep_addr)
{
  ControlNone(dev, hw, Make(0x02U, 3U, 0U, ep_addr, 0U));
}

inline void ClearHalt(FakeUsb& dev, FakeHw& hw, uint8_t ep_addr)
{
  ControlNone(dev, hw, Make(0x02U, 1U, 0U, ep_addr, 0U));
}

inline void Reenumerate(FakeUsb& dev, FakeHw& hw, Speed speed = Speed::FULL)
{
  dev.Reset(speed);
  TEST_ASSERT(dev.Inited());
  SetAddress(dev, hw, 7U);
  SetConfiguration(dev, hw, 1U);
}

inline void Enumerate(FakeUsb& dev, FakeHw& hw, Speed speed = Speed::FULL)
{
  dev.Init(false);
  Reenumerate(dev, hw, speed);
}

/// 每个空闲的 OUT 槽都挂着接收：再挂一个得 BUSY / Every free OUT slot is receiving:
/// one more receive gets BUSY
inline bool AllReceiving(FakeEp& ep) { return ep.Receive(64U, false) == ErrorCode::BUSY; }

}  // namespace FakeUsbTest
