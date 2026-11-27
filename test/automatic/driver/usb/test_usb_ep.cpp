/**
 * @file test_usb_ep.cpp
 * @brief 端点与设备串行服务的假后端测试 / Fake-backend tests of Endpoint and
 *        DeviceService
 *
 * 假后端端点实现全部后端函数并记录调用，由测试决定请求何时、如何完成：在
 * StartHardware 之内，之后经设备的硬件事件处理，或作为 StopHardware 的返回值。假设备
 * 派生 DeviceService，一个没有挂起函数，一个有（挂起时直接调用中断入口，多线程测试里
 * 由扮演中断的线程运行）。每条断言的依据是头文件的文档。
 * The fake backend endpoint implements every backend function, records the calls and
 * lets the test decide when and how a request completes: inside StartHardware, later
 * through the device's hardware event handling, or as the outcome of StopHardware. The
 * fake device derives from DeviceService, once without and once with the pend function
 * (pending calls the interrupt entry; in the multi-threaded test the thread playing the
 * interrupt runs it). Every assertion rests on the header documentation.
 */

#include <sys/wait.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <functional>
#include <initializer_list>
#include <memory>
#include <thread>
#include <utility>
#include <vector>

#include "driver_async_condition.hpp"
#include "ep.hpp"
#include "libxr_assert.hpp"
#include "test_assert.hpp"
#include "usb_service.hpp"

namespace
{
using LibXR::Callback;
using LibXR::ErrorCode;
using LibXR::RawData;
using LibXR::USB::DeviceService;
using LibXR::USB::Endpoint;
using LibXR::USB::Speed;
using Completion = Endpoint::Completion;
using Direction = Endpoint::Direction;
using EPNumber = Endpoint::EPNumber;
using Gate = DeviceService::StartGate;
using Slot = Endpoint::Slot;
using State = Endpoint::State;
using Type = Endpoint::Type;

static_assert(LibXR::HW_CACHE_LINE_SIZE <= 64U, "test memory is aligned to 64 bytes");

/// 端点内存 / Endpoint memory
struct alignas(64) Memory
{
  uint8_t bytes[2048];
};

/// 后端函数 / Backend functions
enum class Hw : uint8_t
{
  CONFIGURE,
  CLOSE,
  START,
  STOP,
  STALL,
  CLEAR_STALL
};

/// 一次后端调用 / One backend call
struct HwCall
{
  Hw function;
  size_t length;
  const uint8_t* data;
  bool in_isr;
};

/// 回调看到的一次完成 / One completion as the callback saw it
struct Seen
{
  Completion completion;
  bool in_isr;
  size_t hw_calls;  ///< 回调时后端调用的总数 / Backend calls made before the callback
};

/// 记录调用并由测试决定完成的后端端点 / Backend endpoint that records calls
class FakeEndpoint final : public Endpoint
{
 public:
  using Endpoint::GetConfig;
  using Endpoint::HwLimits;
  using Endpoint::HwRequest;
  using Endpoint::StopOutcome;

  FakeEndpoint(EPNumber number, Direction avail, RawData memory, uint8_t slots = 0U)
      : Endpoint(number, avail, memory, slots)
  {
  }

  /// 留到下一次硬件事件处理报告 / Report at the next hardware event handling
  void Later(size_t actual, ErrorCode result = ErrorCode::OK)
  {
    later_.emplace_back(actual, result);
  }

  /// 在 StartHardware 之内报告 / Report from inside StartHardware
  void Report(bool in_isr, size_t actual, ErrorCode result)
  {
    TEST_ASSERT(ahead == 0);
    busy = false;
    OnHardwareComplete(in_isr, actual, result);
  }

  /// 设备的硬件事件处理：交出留下的报告 / Hardware events: make the reports left
  void ServiceLater(bool in_isr)
  {
    while (!later_.empty())
    {
      const auto report = later_.front();
      later_.pop_front();
      Hand(in_isr, report.first, report.second);
    }
  }

  /**
   * @brief 设备的硬件事件处理：推进按节拍的停止 / Hardware events: advance a stop run
   *        by the tick
   * @param ticks 设备至今数过的节拍 / Ticks the device has counted so far
   * @return 停止仍挂起 / Whether the stop is still pending
   */
  bool ServiceTick(bool in_isr, uint32_t ticks)
  {
    if (!tick_stop)
    {
      return false;
    }
    const uint32_t elapsed = ticks - tick_start;
    const bool settled = quiescent || elapsed >= quiet_after;
    if (!settled && elapsed < tick_bound)
    {
      return true;
    }
    tick_stop = false;
    // 判超时之前再读一次状态 / Read the status once more before a timeout
    const bool real = settled || quiescent_on_reread;
    Hand(in_isr, tick_actual, real ? tick_result : ErrorCode::TIMEOUT);
    return false;
  }

  /// 控制器复位：未报告的停止就此结束 / Controller reset: unreported stops end here
  void ResetHw()
  {
    reset = true;
    later_.clear();
    tick_stop = false;
  }

  /// 留下的报告数 / Reports left
  size_t LaterCount() const { return later_.size(); }

  size_t Count(Hw function) const
  {
    size_t count = 0U;
    for (const HwCall& call : calls)
    {
      count += (call.function == function) ? 1U : 0U;
    }
    return count;
  }

  /// 后端调用恰为 expected，依次 / The backend calls are exactly expected, in order
  bool CallsAre(const std::vector<Hw>& expected) const
  {
    if (calls.size() != expected.size())
    {
      return false;
    }
    for (size_t i = 0U; i < expected.size(); ++i)
    {
      if (calls[i].function != expected[i])
      {
        return false;
      }
    }
    return true;
  }

  /// 后端调用恰为 expected，依次 / The backend calls are exactly expected, in order
  bool CallsAre(std::initializer_list<Hw> expected) const
  {
    if (calls.size() != expected.size())
    {
      return false;
    }
    size_t i = 0U;
    for (Hw function : expected)
    {
      if (calls[i++].function != function)
      {
        return false;
      }
    }
    return true;
  }

  /// 第 n 次 START / The n-th START
  const HwCall& Start(size_t n) const
  {
    size_t seen = 0U;
    for (const HwCall& call : calls)
    {
      if (call.function == Hw::START && seen++ == n)
      {
        return call;
      }
    }
    TEST_ASSERT(false);
    return calls.front();
  }

  std::vector<HwCall> calls;
  int depth = 0;                   ///< 正在执行的后端函数 / Backend functions running
  bool nested = false;             ///< 后端函数嵌套过 / A backend function was nested
  bool start_in_callback = false;  ///< 回调中启动过 / Started inside a callback
  const bool* callback_running = nullptr;  ///< 回调是否在运行 / Callback running
  ErrorCode configure_result = ErrorCode::OK;
  ErrorCode clear_result = ErrorCode::OK;
  std::deque<ErrorCode> start_results;  ///< 空时 OK / OK when empty
  /// StopHardware 依次返回；空时 ABORTED、0 / Returned by StopHardware in turn;
  /// ABORTED and 0 when empty
  std::deque<StopOutcome> stop_outcomes;
  HwLimits limits;
  mutable Direction limits_direction = Direction::BOTH;
  mutable Type limits_type = Type::CONTROL;
  Config configured;
  std::function<void(FakeEndpoint&, const HwRequest&, bool)> on_start;
  std::function<void(FakeEndpoint&, bool)> on_stop;
  /// 设置时代替 stop_outcomes / Replaces stop_outcomes when set
  std::function<StopOutcome(FakeEndpoint&)> stop_rule;
  /// 后端确认停稳、报告之前 / The backend confirmed quiescence, before reporting
  std::function<void()> on_stop_end;

  /// @name 后端视角 / The backend's view
  /// @{
  bool busy = false;  ///< 硬件上有请求 / A request is on the hardware
  int ahead = 0;  ///< 最早那个之后硬件上还有几个 / Requests on the hardware behind the
                  ///< earliest
  bool stop_pending = false;  ///< 停止挂起、未报告 / A stop is pending, unreported
  bool reset =
      false;  ///< 控制器已复位，停止立即结束 / Controller reset, stops end at once
  /// @}

  /// @name 按节拍推进的停止 / Stops advanced by the tick
  /// @{
  int tick_min_ms = -1;  ///< 非负时 StopHardware 挂起并起节拍 / If set, stops pend
  bool tick_stop = false;
  uint32_t tick_start = 0U;
  uint32_t quiet_after = UINT32_MAX;  ///< 几个节拍后停稳 / Quiescent after these ticks
  uint32_t tick_bound = 3U;           ///< 超时的节拍数 / Ticks before a timeout
  bool quiescent = false;             ///< 状态读到已停稳 / The status reads quiescent
  /// 到上限时再读一次读到已停稳 / The re-read at the bound finds it quiescent
  bool quiescent_on_reread = false;
  ErrorCode tick_result = ErrorCode::ABORTED;
  size_t tick_actual = 0U;
  /// 开始一次节拍等待；由设备设置 / Start a tick wait; set by the device
  std::function<uint32_t(uint32_t)> start_tick;
  /// @}

 protected:
  ErrorCode CheckHardware(const Config&) const override { return configure_result; }

  void ConfigureHardware(const Config& cfg) override
  {
    Begin(Hw::CONFIGURE, cfg.max_packet_size, nullptr, false);
    TEST_ASSERT(!busy && !stop_pending);
    configured = cfg;
    --depth;
  }

  void CloseHardware(bool in_isr) override
  {
    Begin(Hw::CLOSE, 0U, nullptr, in_isr);
    TEST_ASSERT(!busy && !stop_pending);
    --depth;
  }

  ErrorCode StartHardware(const HwRequest& request, bool in_isr) override
  {
    Begin(Hw::START, request.length, request.data, in_isr);
    // 停止挂起期间什么都不启动；硬件上的请求数不超过后端报的上限
    // Nothing starts while a stop is pending; never more requests on the hardware than
    // the limit the backend gave
    const bool behind = busy;
    TEST_ASSERT(!stop_pending && (!behind || ahead + 2 <= limits.max_requests));
    if (behind)
    {
      ++ahead;
    }
    busy = true;
    if (callback_running != nullptr && *callback_running)
    {
      start_in_callback = true;
    }
    ErrorCode result = ErrorCode::OK;
    if (!start_results.empty())
    {
      result = start_results.front();
      start_results.pop_front();
    }
    if (on_start)
    {
      on_start(*this, request, in_isr);
    }
    if (result != ErrorCode::OK)
    {
      if (behind)
      {
        --ahead;
      }
      else
      {
        busy = false;
      }
    }
    --depth;
    return result;
  }

  StopOutcome StopHardware(bool in_isr) override
  {
    Begin(Hw::STOP, 0U, nullptr, in_isr);
    // 只在有请求在途、没有停止挂起时调用，复位之后那一次例外 / Called only with a request
    // in flight and no stop pending, the one call after a reset excepted
    TEST_ASSERT(reset || (busy && !stop_pending));
    StopOutcome outcome{ErrorCode::ABORTED, 0U};
    if (reset)
    {
      busy = false;
      ahead = 0;
      stop_pending = false;
      --depth;
      return outcome;
    }
    if (stop_rule)
    {
      outcome = stop_rule(*this);
    }
    else if (tick_min_ms >= 0)
    {
      outcome = {ErrorCode::PENDING, 0U};
      tick_stop = true;
      tick_start = start_tick(static_cast<uint32_t>(tick_min_ms));
    }
    else if (!stop_outcomes.empty())
    {
      outcome = stop_outcomes.front();
      stop_outcomes.pop_front();
    }
    if (outcome.result == ErrorCode::PENDING)
    {
      stop_pending = true;
    }
    else if (ahead > 0)
    {
      // 还有一个请求在硬件上：核心再调用一次来取它的结果
      // One more request is on the hardware: the core calls again for its outcome
      --ahead;
    }
    else
    {
      busy = false;
    }
    if (on_stop)
    {
      on_stop(*this, in_isr);
    }
    --depth;
    return outcome;
  }

  ErrorCode StallHardware(bool in_isr) override
  {
    Begin(Hw::STALL, 0U, nullptr, in_isr);
    TEST_ASSERT(!busy && !stop_pending);
    --depth;
    return ErrorCode::OK;
  }

  ErrorCode ClearStallHardware(bool in_isr) override
  {
    Begin(Hw::CLEAR_STALL, 0U, nullptr, in_isr);
    TEST_ASSERT(!stop_pending);
    --depth;
    return clear_result;
  }

  HwLimits HardwareLimits(Direction direction, Type type) const override
  {
    limits_direction = direction;
    limits_type = type;
    return limits;
  }

 private:
  void Begin(Hw function, size_t length, const uint8_t* data, bool in_isr)
  {
    nested = nested || depth != 0;
    ++depth;
    calls.push_back({function, length, data, in_isr});
  }

  /// 一次报告：先更新后端视角，再交给核心 / One report: the backend's view first, then
  /// the core
  void Hand(bool in_isr, size_t actual, ErrorCode result)
  {
    if (stop_pending && result != ErrorCode::TIMEOUT)
    {
      if (ahead > 0)
      {
        // 停止挂起时硬件上有两个请求：每个各报告一次，后一次才是停止结束
        // Two requests were on the hardware at the pending stop: one report for each,
        // and the second ends the stop
        --ahead;
      }
      else
      {
        stop_pending = false;
        busy = false;
        if (on_stop_end)
        {
          on_stop_end();
        }
      }
    }
    else if (!stop_pending)
    {
      if (ahead > 0)
      {
        --ahead;
      }
      else
      {
        busy = false;
      }
    }
    OnHardwareComplete(in_isr, actual, result);
  }

  std::deque<std::pair<size_t, ErrorCode>> later_;
};

/// 端点的完成回调记录 / Records an endpoint's completions
struct Recorder
{
  explicit Recorder(FakeEndpoint& endpoint) : ep(endpoint)
  {
    ep.callback_running = &in_callback;
    ep.SetOnComplete(Callback<const Completion&>::Create(
        [](bool in_isr, Recorder* self, const Completion& completion)
        { self->OnComplete(completion, in_isr); }, this));
  }

  void OnComplete(const Completion& completion, bool in_isr)
  {
    inside_backend = inside_backend || ep.depth != 0;
    seen.push_back({completion, in_isr, ep.calls.size()});
    in_callback = true;
    if (hook)
    {
      hook(completion, in_isr);
    }
    in_callback = false;
  }

  const Completion& At(size_t n) const
  {
    TEST_ASSERT(n < seen.size());
    return seen[n].completion;
  }

  FakeEndpoint& ep;
  std::vector<Seen> seen;
  bool in_callback = false;
  bool inside_backend = false;  ///< 回调在后端函数内运行过 / Ran inside a backend call
  std::function<void(const Completion&, bool)> hook;
};

/// 假设备：可选挂起函数，日志记录入口顺序 / Fake device with an optional pend function
class FakeDevice final : public DeviceService
{
 public:
  static constexpr uint32_t JOB_EVENT = 1U;   ///< 在 owner 中运行任务 / Run a job
  static constexpr uint32_t NOTE_EVENT = 2U;  ///< 只记录 / Just logged

  /// 日志项 / Log entry
  enum class Step : uint8_t
  {
    MASK,
    UNMASK,
    SERVICE,
    PEND,
    HANDLER,
    START  ///< 由测试的 on_start 记录 / Logged by a test's on_start
  };

  // alignas(4)：GCC 13 在 Release 下对两字节的元素误报 -Wstringop-overflow。
  // alignas(4): GCC 13 in Release wrongly reports -Wstringop-overflow for a two-byte
  // element.
  struct alignas(4) Entry
  {
    Step step;
    bool in_isr;
  };

  explicit FakeDevice(bool can_pend)
      : can_pend_(can_pend), tick_(&TickCheck, &TickDone, this)
  {
    SetDeviceHandler(
        Callback<uint32_t>::Create([](bool in_isr, FakeDevice* self, uint32_t events)
                                   { self->OnEvents(events, in_isr); }, this));
  }

  void Add(FakeEndpoint& ep)
  {
    TEST_ASSERT(Register(ep) == ErrorCode::OK);
    eps.push_back(&ep);
    ep.start_tick = [this](uint32_t min_ms)
    {
      (void)tick_.Start(min_ms, min_ms);
      return ticks.load(std::memory_order_acquire);
    };
  }

  /// 控制器复位并关闭所有端点，如同设备核心 / Reset the controller and close every
  /// endpoint, as the device core does
  void ResetAndCloseAll(bool in_isr)
  {
    ++resets;
    for (FakeEndpoint* ep : eps)
    {
      ep->ResetHw();
    }
    CloseAllEndpoints(in_isr);
    for (FakeEndpoint* ep : eps)
    {
      ep->reset = false;
    }
  }

  /// 节拍请求一轮 ServiceHardware；只供测试直接调用 / The tick asks for a pass of
  /// ServiceHardware; for direct test calls
  void Poll(bool in_isr) { RequestPoll(in_isr); }

  /// 控制器中断：挂起的中断在本次返回后接着运行 / Controller interrupt; a pended one
  /// runs right after
  void Irq()
  {
    ++irq_depth_;
    do
    {
      pended_ = false;
      OnControllerInterrupt();
    } while (pended_);
    --irq_depth_;
  }

  /// 在 owner 中运行 job / Run job in the owner
  void AsOwner(std::function<void(bool)> job)
  {
    job_ = std::move(job);
    Request(JOB_EVENT, false);
    TEST_ASSERT(!job_);
  }

  void Note(Step step, bool in_isr)
  {
    if (logging)
    {
      log.push_back({step, in_isr});
    }
  }

  std::vector<Entry> log;
  bool logging = true;
  std::atomic<bool> masked{false};
  /// 设备处理程序每次收到的事件位 / Event bits of each device handler call
  std::vector<uint32_t> handled;
  std::vector<FakeEndpoint*> eps;
  /// 非空时挂起只置这根线，由扮演中断的线程运行 / When set, pending only raises this
  /// line and the thread playing the interrupt runs it
  std::atomic<bool>* pend_line = nullptr;
  /// 置位时，下一次 PendInterrupt 先让控制器中断运行一次再按 can_pend 返回 / When set,
  /// the next PendInterrupt first lets the controller interrupt run once
  bool irq_at_pend = false;

  /// 收到 STOP_FAILED_EVENT 时复位控制器并关闭所有端点 / Reset the controller and close
  /// every endpoint on STOP_FAILED_EVENT
  bool reset_on_failure = false;
  size_t resets = 0U;               ///< 控制器复位次数 / Controller resets
  size_t services = 0U;             ///< ServiceHardware 次数 / ServiceHardware calls
  std::atomic<uint32_t> ticks{0U};  ///< 节拍唤醒次数 / Tick wakes

  /// 收到 STOP_FAILED_EVENT 的处理程序调用次数 / Handler calls with STOP_FAILED_EVENT
  size_t StopFailures() const
  {
    size_t count = 0U;
    for (uint32_t events : handled)
    {
      count += ((events & STOP_FAILED_EVENT) != 0U) ? 1U : 0U;
    }
    return count;
  }

 protected:
  void ServiceHardware(bool in_isr) override
  {
    Note(Step::SERVICE, in_isr);
    ++services;
    const uint32_t now = ticks.load(std::memory_order_acquire);
    bool waiting = false;
    for (FakeEndpoint* ep : eps)
    {
      ep->ServiceLater(in_isr);
      waiting = ep->ServiceTick(in_isr, now) || waiting;
    }
    if (waiting)
    {
      (void)tick_.Start(0U, 0U);
    }
  }

  void MaskInterrupt() override
  {
    masked.store(true, std::memory_order_release);
    Note(Step::MASK, true);
  }

  void UnmaskInterrupt() override
  {
    Note(Step::UNMASK, false);
    masked.store(false, std::memory_order_release);
  }

  bool PendInterrupt() override
  {
    if (irq_at_pend)
    {
      irq_at_pend = false;
      Irq();
    }
    if (!can_pend_)
    {
      return false;
    }
    if (pend_line != nullptr)
    {
      pend_line->store(true, std::memory_order_release);
      return true;
    }
    Note(Step::PEND, false);
    pended_ = true;
    if (irq_depth_ == 0)
    {
      Irq();
    }
    return true;
  }

 private:
  void OnEvents(uint32_t events, bool in_isr)
  {
    Note(Step::HANDLER, in_isr);
    if (logging)
    {
      handled.push_back(events);
    }
    if ((events & STOP_FAILED_EVENT) != 0U && reset_on_failure)
    {
      ResetAndCloseAll(in_isr);
    }
    if ((events & JOB_EVENT) != 0U && job_)
    {
      std::function<void(bool)> job = std::move(job_);
      job_ = nullptr;
      job(in_isr);
    }
  }

  /// 节拍的条件：总是成立，结果函数只请求一轮 ServiceHardware / The tick's condition
  /// always holds; the result function only asks for a pass of ServiceHardware
  static bool TickCheck(void*) { return true; }

  static void TickDone(void* arg, ErrorCode)
  {
    auto* self = static_cast<FakeDevice*>(arg);
    self->ticks.fetch_add(1U, std::memory_order_acq_rel);
    self->RequestPoll(true);
  }

  bool can_pend_;
  int irq_depth_ = 0;
  bool pended_ = false;
  std::function<void(bool)> job_;
  LibXR::DriverAsyncCondition tick_;
};

void SetGate(FakeDevice& dev, Gate gate)
{
  dev.AsOwner([&](bool) { dev.SetStartGate(gate); });
}

void SetBusSpeed(FakeDevice& dev, Speed speed)
{
  dev.AsOwner([&](bool) { dev.SetSpeed(speed); });
}

ErrorCode Configure(FakeDevice& dev, FakeEndpoint& ep, Direction direction, Type type,
                    uint16_t bound)
{
  ErrorCode result = ErrorCode::PENDING;
  dev.AsOwner([&](bool in_isr)
              { result = ep.Configure({direction, type, bound}, in_isr); });
  return result;
}

void Close(FakeDevice& dev, FakeEndpoint& ep)
{
  dev.AsOwner([&](bool in_isr) { ep.Close(in_isr); });
}

void Flush(FakeDevice& dev, FakeEndpoint& ep)
{
  dev.AsOwner([&](bool in_isr) { ep.Flush(in_isr); });
}

ErrorCode Stall(FakeDevice& dev, FakeEndpoint& ep)
{
  ErrorCode result = ErrorCode::PENDING;
  dev.AsOwner([&](bool in_isr) { result = ep.Stall(in_isr); });
  return result;
}

ErrorCode ClearStall(FakeDevice& dev, FakeEndpoint& ep)
{
  ErrorCode result = ErrorCode::PENDING;
  dev.AsOwner([&](bool in_isr) { result = ep.ClearStall(in_isr); });
  return result;
}

/// 一个已登记的端点、它的记录和闸门全开的设备 / One registered endpoint, its
/// recorder and a device with the gate open
struct Rig
{
  Rig(EPNumber number, Direction direction, Type type = Type::BULK, bool can_pend = false,
      size_t memory = 1024U, uint8_t slots = 0U)
      : dev(can_pend), ep(number, direction, {mem.bytes, memory}, slots), rec(ep)
  {
    dev.Add(ep);
    SetGate(dev, Gate::ALL);
    TEST_ASSERT(Configure(dev, ep, direction, type, 64U) == ErrorCode::OK);
    ep.calls.clear();
    dev.log.clear();
  }

  /// 借一个槽并提交 / Borrow a slot and submit it
  Slot Send(size_t length, bool zlp = false)
  {
    Slot slot;
    TEST_ASSERT(ep.AcquireSlot(slot) == ErrorCode::OK);
    TEST_ASSERT(ep.Submit(slot, length, zlp, false) == ErrorCode::OK);
    return slot;
  }

  /// 下一次硬件事件处理时报告完成 / Complete at the next hardware event handling
  void Complete(size_t actual, ErrorCode result = ErrorCode::OK)
  {
    ep.Later(actual, result);
    dev.Irq();
  }

  Memory mem;
  FakeDevice dev;
  FakeEndpoint ep;
  Recorder rec;
};

/// 结束请求的生命周期函数 / Lifecycle function that ends the requests
enum class Ender : uint8_t
{
  FLUSH,
  STALL,
  CLOSE
};

/// 在 owner 中以 how 结束 ep 的请求 / End ep's requests with how, in the owner
void End(FakeDevice& dev, FakeEndpoint& ep, Ender how)
{
  dev.AsOwner(
      [&](bool in_isr)
      {
        if (how == Ender::FLUSH)
        {
          ep.Flush(in_isr);
        }
        else if (how == Ender::STALL)
        {
          TEST_ASSERT(ep.Stall(in_isr) == ErrorCode::OK);
        }
        else
        {
          ep.Close(in_isr);
        }
      });
}

// ---------------------------------------------------------------------------
// Configure
// ---------------------------------------------------------------------------

/// 每种类型和速度选出的包长与上限 / Packet size per type, speed and bound
void CheckPacketSizes()
{
  struct Row
  {
    Speed speed;
    Type type;
    uint16_t bound;
    ErrorCode result;
    uint16_t packet;
  };
  constexpr uint16_t ANY = UINT16_MAX;
  const Row rows[] = {
      {Speed::FULL, Type::CONTROL, ANY, ErrorCode::OK, 64U},
      {Speed::FULL, Type::BULK, ANY, ErrorCode::OK, 64U},
      {Speed::FULL, Type::INTR, ANY, ErrorCode::OK, 64U},
      {Speed::FULL, Type::ISOCHRONOUS, ANY, ErrorCode::OK, 1023U},
      {Speed::HIGH, Type::CONTROL, ANY, ErrorCode::OK, 64U},
      {Speed::HIGH, Type::BULK, ANY, ErrorCode::OK, 512U},
      {Speed::HIGH, Type::INTR, ANY, ErrorCode::OK, 1024U},
      {Speed::HIGH, Type::ISOCHRONOUS, ANY, ErrorCode::OK, 1024U},
      {Speed::LOW, Type::CONTROL, ANY, ErrorCode::OK, 8U},
      {Speed::LOW, Type::INTR, ANY, ErrorCode::OK, 8U},
      {Speed::LOW, Type::BULK, ANY, ErrorCode::ARG_ERR, 0U},
      {Speed::LOW, Type::ISOCHRONOUS, ANY, ErrorCode::ARG_ERR, 0U},
      // 上限低于合法最大值 / Bound below the legal maximum
      {Speed::FULL, Type::BULK, 63U, ErrorCode::OK, 32U},
      {Speed::FULL, Type::CONTROL, 40U, ErrorCode::OK, 32U},
      {Speed::FULL, Type::INTR, 10U, ErrorCode::OK, 10U},
      {Speed::FULL, Type::ISOCHRONOUS, 1000U, ErrorCode::OK, 1000U},
      {Speed::HIGH, Type::INTR, 700U, ErrorCode::OK, 700U},
      {Speed::FULL, Type::BULK, 7U, ErrorCode::ARG_ERR, 0U},
      {Speed::HIGH, Type::BULK, 511U, ErrorCode::ARG_ERR, 0U},
      {Speed::HIGH, Type::CONTROL, 63U, ErrorCode::ARG_ERR, 0U},
      {Speed::LOW, Type::CONTROL, 7U, ErrorCode::ARG_ERR, 0U},
      {Speed::FULL, Type::INTR, 0U, ErrorCode::ARG_ERR, 0U},
  };

  Memory mem;
  FakeDevice dev(false);
  // 两个 1024 字节的槽 / Two slots of 1024 bytes
  FakeEndpoint ep(EPNumber::EP1, Direction::BOTH, {mem.bytes, 2048U});
  dev.Add(ep);
  for (const Row& row : rows)
  {
    SetBusSpeed(dev, row.speed);
    const ErrorCode result = Configure(dev, ep, Direction::IN, row.type, row.bound);
    TEST_ASSERT(result == row.result);
    if (result != ErrorCode::OK)
    {
      TEST_ASSERT(ep.GetState() == State::DISABLED);
      continue;
    }
    TEST_ASSERT(ep.GetState() == State::IDLE);
    TEST_ASSERT(ep.MaxPacketSize() == row.packet);
    TEST_ASSERT(ep.configured.max_packet_size == row.packet);
    // GetConfig 给后端的是选定的包长，不是上限 / GetConfig gives the backend the chosen
    // size, not the bound
    TEST_ASSERT(ep.GetConfig().max_packet_size == row.packet);
    TEST_ASSERT(ep.configured.type == row.type);
    TEST_ASSERT(ep.GetType() == row.type);
    TEST_ASSERT(ep.GetDirection() == Direction::IN);
    TEST_ASSERT(ep.GetAddress() == 0x81U);
    Close(dev, ep);
    TEST_ASSERT(ep.GetDirection() == Direction::BOTH);
    TEST_ASSERT(ep.GetAddress() == 0x01U);
  }
}

/// 槽太小、硬件上限与 MaxTransferSize / Small slots, hardware limits, MaxTransferSize
void CheckSlotsAndLimits()
{
  Memory mem;
  FakeDevice dev(false);
  // 一个 64 字节的槽 / One slot of 64 bytes
  FakeEndpoint small(EPNumber::EP1, Direction::IN, {mem.bytes, 64U}, 1U);
  // 两个 64 字节的槽 / Two slots of 64 bytes
  FakeEndpoint halves(EPNumber::EP2, Direction::BOTH, {mem.bytes + 64, 128U});
  // 两个 512 字节的槽 / Two slots of 512 bytes
  FakeEndpoint big(EPNumber::EP3, Direction::IN, {mem.bytes + 192, 1024U});
  // EP0，一个 128 字节的槽 / EP0, one slot of 128 bytes
  FakeEndpoint ep0(EPNumber::EP0, Direction::IN, {mem.bytes + 1216, 128U});
  dev.Add(small);
  dev.Add(halves);
  dev.Add(big);
  dev.Add(ep0);

  // 硬件直接读写槽时一个槽只够控制端点 / With the hardware accessing the slot directly
  // one slot is enough for a control endpoint only
  for (const Type type : {Type::BULK, Type::INTR, Type::ISOCHRONOUS})
  {
    TEST_ASSERT(Configure(dev, small, Direction::IN, type, 64U) == ErrorCode::NO_BUFF);
    TEST_ASSERT(small.GetState() == State::DISABLED && small.calls.empty());
  }
  TEST_ASSERT(Configure(dev, small, Direction::IN, Type::CONTROL, 64U) == ErrorCode::OK);
  TEST_ASSERT(small.MaxTransferSize() == 64U);
  Close(dev, small);
  small.calls.clear();
  small.limits.direct_memory = false;

  SetBusSpeed(dev, Speed::HIGH);
  TEST_ASSERT(Configure(dev, small, Direction::IN, Type::BULK, 512U) ==
              ErrorCode::NO_BUFF);
  TEST_ASSERT(small.GetState() == State::DISABLED);
  TEST_ASSERT(Configure(dev, small, Direction::IN, Type::INTR, 1024U) == ErrorCode::OK);
  TEST_ASSERT(small.MaxPacketSize() == 64U);
  TEST_ASSERT(small.MaxTransferSize() == 64U);
  Close(dev, small);

  SetBusSpeed(dev, Speed::FULL);
  TEST_ASSERT(Configure(dev, small, Direction::IN, Type::ISOCHRONOUS, 1023U) ==
              ErrorCode::OK);
  TEST_ASSERT(small.MaxPacketSize() == 64U);
  Close(dev, small);
  TEST_ASSERT(Configure(dev, halves, Direction::OUT, Type::ISOCHRONOUS, 1023U) ==
              ErrorCode::OK);
  TEST_ASSERT(halves.MaxPacketSize() == 64U);
  TEST_ASSERT(halves.MaxTransferSize() == 64U);
  Close(dev, halves);

  // 后端上限压低包长 / The backend limit lowers the packet size
  TEST_ASSERT(Configure(dev, big, Direction::IN, Type::BULK, 64U) == ErrorCode::OK);
  TEST_ASSERT(big.MaxTransferSize() == 512U);
  Close(dev, big);
  big.limits.max_packet_size = 32U;
  TEST_ASSERT(Configure(dev, big, Direction::IN, Type::BULK, 64U) == ErrorCode::OK);
  TEST_ASSERT(big.MaxPacketSize() == 32U);
  TEST_ASSERT(big.GetConfig().max_packet_size == 32U);
  TEST_ASSERT(big.MaxTransferSize() == 512U);
  TEST_ASSERT(big.limits_direction == Direction::IN && big.limits_type == Type::BULK);
  Close(dev, big);
  big.limits.max_packet_size = 20U;
  TEST_ASSERT(Configure(dev, big, Direction::IN, Type::BULK, 64U) == ErrorCode::OK);
  TEST_ASSERT(big.MaxPacketSize() == 16U);
  Close(dev, big);
  big.limits.max_packet_size = 100U;
  TEST_ASSERT(Configure(dev, big, Direction::IN, Type::ISOCHRONOUS, 1023U) ==
              ErrorCode::OK);
  TEST_ASSERT(big.MaxPacketSize() == 100U);
  // 等时类型的一个请求是一个服务区间，这里是一个包
  // One request of the isochronous type is one service interval, one packet here
  TEST_ASSERT(big.MaxTransferSize() == 100U);
  Close(dev, big);
  SetBusSpeed(dev, Speed::HIGH);
  big.limits.max_packet_size = 256U;
  TEST_ASSERT(Configure(dev, big, Direction::IN, Type::BULK, 512U) == ErrorCode::NO_BUFF);
  TEST_ASSERT(big.GetState() == State::DISABLED);
  SetBusSpeed(dev, Speed::FULL);

  // 一次请求的包数和字节数上限 / Packet and byte limits of one request
  big.limits = {};
  big.limits.max_packets = 3U;
  TEST_ASSERT(Configure(dev, big, Direction::IN, Type::BULK, 64U) == ErrorCode::OK);
  TEST_ASSERT(big.MaxTransferSize() == 192U);
  Close(dev, big);
  // max_bytes 不小于 max_packet_size / max_bytes is not below max_packet_size
  big.limits = {};
  big.limits.max_packet_size = 64U;
  big.limits.max_bytes = 300U;
  TEST_ASSERT(Configure(dev, big, Direction::IN, Type::BULK, 64U) == ErrorCode::OK);
  TEST_ASSERT(big.MaxTransferSize() == 256U);
  Close(dev, big);
  big.limits = {};
  big.limits.max_packet_size = 64U;
  big.limits.max_packets = 5U;
  big.limits.max_bytes = 200U;
  TEST_ASSERT(Configure(dev, big, Direction::IN, Type::BULK, 64U) == ErrorCode::OK);
  TEST_ASSERT(big.MaxTransferSize() == 192U);
  Close(dev, big);
  big.limits = {};
  big.limits.max_packet_size = 64U;
  big.limits.max_packets = 1U;
  big.limits.max_bytes = 64U;
  TEST_ASSERT(Configure(dev, big, Direction::IN, Type::BULK, 64U) == ErrorCode::OK);
  TEST_ASSERT(big.MaxPacketSize() == 64U && big.MaxTransferSize() == 64U);
  Close(dev, big);
  big.limits = {};

  // 包长的整数倍 / A packet multiple
  Memory iso_mem;
  FakeEndpoint iso(EPNumber::EP4, Direction::IN, {iso_mem.bytes, 2048U});
  dev.Add(iso);
  TEST_ASSERT(Configure(dev, iso, Direction::IN, Type::ISOCHRONOUS, 1023U) ==
              ErrorCode::OK);
  TEST_ASSERT(iso.MaxTransferSize() == 1023U);

  // 控制类型一个包，与端点号无关 / One packet for the control type, whatever the
  // endpoint number
  TEST_ASSERT(Configure(dev, ep0, Direction::IN, Type::CONTROL, 64U) == ErrorCode::OK);
  TEST_ASSERT(ep0.MaxPacketSize() == 64U);
  TEST_ASSERT(ep0.MaxTransferSize() == 64U);
  Close(dev, ep0);
  TEST_ASSERT(Configure(dev, big, Direction::IN, Type::CONTROL, 64U) == ErrorCode::OK);
  TEST_ASSERT(big.MaxPacketSize() == 64U && big.MaxTransferSize() == 64U);
  Close(dev, big);
  ep0.limits.direct_memory = false;
  TEST_ASSERT(Configure(dev, ep0, Direction::IN, Type::BULK, 64U) == ErrorCode::OK);
  TEST_ASSERT(ep0.MaxTransferSize() == 128U);
}

/// 每个错误码；失败保持 DISABLED 且之前的配置不变 / Every error code; a failure stays
/// DISABLED and keeps the previous configuration
void CheckConfigureErrors()
{
  Memory mem;
  FakeDevice dev(false);
  FakeEndpoint ep(EPNumber::EP1, Direction::IN, {mem.bytes, 1024U});
  FakeEndpoint out(EPNumber::EP2, Direction::OUT, {mem.bytes + 1024, 1024U});
  dev.Add(ep);
  dev.Add(out);
  Slot slot;

  TEST_ASSERT(Configure(dev, ep, Direction::IN, Type::INTR, 16U) == ErrorCode::OK);
  TEST_ASSERT(Configure(dev, ep, Direction::IN, Type::BULK, 64U) == ErrorCode::STATE_ERR);
  TEST_ASSERT(ep.MaxPacketSize() == 16U && ep.GetType() == Type::INTR);
  Close(dev, ep);
  const size_t transfer = ep.MaxTransferSize();

  auto unchanged = [&]()
  {
    TEST_ASSERT(ep.GetState() == State::DISABLED);
    TEST_ASSERT(ep.MaxPacketSize() == 16U);
    TEST_ASSERT(ep.GetType() == Type::INTR);
    TEST_ASSERT(ep.MaxTransferSize() == transfer);
    TEST_ASSERT(ep.AcquireSlot(slot) == ErrorCode::STATE_ERR);
  };

  TEST_ASSERT(Configure(dev, ep, Direction::BOTH, Type::BULK, 64U) == ErrorCode::ARG_ERR);
  unchanged();
  TEST_ASSERT(Configure(dev, ep, Direction::OUT, Type::BULK, 64U) == ErrorCode::ARG_ERR);
  unchanged();
  TEST_ASSERT(Configure(dev, out, Direction::IN, Type::BULK, 64U) == ErrorCode::ARG_ERR);
  TEST_ASSERT(out.GetState() == State::DISABLED);
  TEST_ASSERT(Configure(dev, ep, Direction::IN, Type::BULK, 8U - 1U) ==
              ErrorCode::ARG_ERR);
  unchanged();
  SetBusSpeed(dev, Speed::HIGH);
  ep.limits.max_packet_size = 64U;
  TEST_ASSERT(Configure(dev, ep, Direction::IN, Type::BULK, 512U) == ErrorCode::NO_BUFF);
  unchanged();
  ep.limits = {};
  SetBusSpeed(dev, Speed::FULL);
  ep.configure_result = ErrorCode::NOT_SUPPORT;
  TEST_ASSERT(Configure(dev, ep, Direction::IN, Type::BULK, 64U) ==
              ErrorCode::NOT_SUPPORT);
  unchanged();
  ep.configure_result = ErrorCode::OK;
  TEST_ASSERT(Configure(dev, ep, Direction::IN, Type::BULK, 64U) == ErrorCode::OK);
  TEST_ASSERT(ep.MaxPacketSize() == 64U && ep.GetType() == Type::BULK);
}

// ---------------------------------------------------------------------------
// IN
// ---------------------------------------------------------------------------

/// 借、写、提交、完成 / Borrow, fill, submit, complete
void CheckInBasic()
{
  Rig r(EPNumber::EP1, Direction::IN);
  Slot slot;
  TEST_ASSERT(!slot.Valid());
  TEST_ASSERT(r.ep.AcquireSlot(slot) == ErrorCode::OK);
  TEST_ASSERT(slot.Valid());
  TEST_ASSERT(slot.data.size_ == r.ep.MaxTransferSize());
  TEST_ASSERT(r.ep.MaxTransferSize() == 512U);
  uint8_t* data = static_cast<uint8_t*>(slot.data.addr_);
  TEST_ASSERT(data == r.mem.bytes || data == r.mem.bytes + 512);
  std::memset(data, 0xA5, 100U);
  TEST_ASSERT(r.ep.Submit(slot, 100U, false, false) == ErrorCode::OK);
  TEST_ASSERT(r.ep.Count(Hw::START) == 1U);
  TEST_ASSERT(r.ep.Start(0U).length == 100U);
  TEST_ASSERT(r.ep.Start(0U).data == data);
  TEST_ASSERT(!r.ep.Start(0U).in_isr);
  TEST_ASSERT(r.rec.seen.empty());
  // 已提交的句柄不再被接受 / A submitted handle is no longer accepted
  TEST_ASSERT(r.ep.ReleaseSlot(slot) == ErrorCode::STATE_ERR);
  TEST_ASSERT(r.ep.Submit(slot, 10U, false, false) == ErrorCode::STATE_ERR);
  r.Complete(100U);
  TEST_ASSERT(r.rec.seen.size() == 1U);
  TEST_ASSERT(r.rec.At(0U).result == ErrorCode::OK);
  TEST_ASSERT(r.rec.At(0U).actual == 100U);
  TEST_ASSERT(!r.rec.At(0U).slot.Valid());
  TEST_ASSERT(r.rec.seen[0].in_isr);
  TEST_ASSERT(r.ep.Count(Hw::START) == 1U);
}

/// 两个槽按提交顺序启动，下一个在上一个回调之后 / Two slots start in submit order, the
/// next one after the previous callback
void CheckInOrder(bool synchronous)
{
  Rig r(EPNumber::EP1, Direction::IN);
  if (synchronous)
  {
    r.ep.on_start = [](FakeEndpoint& ep, const FakeEndpoint::HwRequest& request,
                       bool in_isr) { ep.Report(in_isr, request.length, ErrorCode::OK); };
  }
  SetGate(r.dev, Gate::CLOSED);
  Slot first;
  Slot second;
  TEST_ASSERT(r.ep.AcquireSlot(first) == ErrorCode::OK);
  TEST_ASSERT(r.ep.AcquireSlot(second) == ErrorCode::OK);
  TEST_ASSERT(r.ep.Submit(second, 10U, false, false) == ErrorCode::OK);
  TEST_ASSERT(r.ep.Submit(first, 20U, false, false) == ErrorCode::OK);
  TEST_ASSERT(r.ep.Count(Hw::START) == 0U);
  SetGate(r.dev, Gate::ALL);
  if (!synchronous)
  {
    TEST_ASSERT(r.ep.Count(Hw::START) == 1U);
    r.Complete(10U);
    TEST_ASSERT(r.ep.Count(Hw::START) == 2U);
    r.Complete(20U);
  }
  TEST_ASSERT(r.ep.Count(Hw::START) == 2U);
  TEST_ASSERT(r.ep.Start(0U).length == 10U);
  TEST_ASSERT(r.ep.Start(0U).data == second.data.addr_);
  TEST_ASSERT(r.ep.Start(1U).length == 20U);
  TEST_ASSERT(r.ep.Start(1U).data == first.data.addr_);
  TEST_ASSERT(r.rec.seen.size() == 2U);
  TEST_ASSERT(r.rec.At(0U).actual == 10U && r.rec.At(1U).actual == 20U);
  // 第一个回调时第二个还没启动 / The second had not started at the first callback
  TEST_ASSERT(r.rec.seen[0].hw_calls == 1U);
  TEST_ASSERT(r.rec.seen[1].hw_calls == 2U);
  TEST_ASSERT(!r.ep.start_in_callback);
}

/// 零长度请求 / Zero-length request
void CheckZeroLength()
{
  Rig r(EPNumber::EP1, Direction::IN);
  r.Send(0U);
  TEST_ASSERT(r.ep.Count(Hw::START) == 1U);
  TEST_ASSERT(r.ep.Start(0U).length == 0U);
  r.Complete(0U);
  TEST_ASSERT(r.rec.seen.size() == 1U);
  TEST_ASSERT(r.rec.At(0U).result == ErrorCode::OK && r.rec.At(0U).actual == 0U);
}

/// AcquireSlot、Submit、ReleaseSlot 的返回码 / Return codes of the IN interface
void CheckInReturnCodes()
{
  Rig r(EPNumber::EP1, Direction::IN);
  Slot a;
  Slot b;
  Slot c;

  // 未启用、不是 IN / Not enabled, not IN
  Memory other;
  FakeEndpoint idle(EPNumber::EP2, Direction::IN, {other.bytes, 1024U});
  FakeEndpoint out(EPNumber::EP3, Direction::OUT, {other.bytes + 1024, 1024U});
  r.dev.Add(idle);
  r.dev.Add(out);
  TEST_ASSERT(idle.AcquireSlot(a) == ErrorCode::STATE_ERR);
  TEST_ASSERT(Configure(r.dev, out, Direction::OUT, Type::BULK, 64U) == ErrorCode::OK);
  TEST_ASSERT(out.AcquireSlot(a) == ErrorCode::STATE_ERR);

  SetGate(r.dev, Gate::CLOSED);
  TEST_ASSERT(r.ep.AcquireSlot(a) == ErrorCode::OK);
  TEST_ASSERT(r.ep.AcquireSlot(b) == ErrorCode::OK);
  TEST_ASSERT(r.ep.AcquireSlot(c) == ErrorCode::BUSY);

  // SIZE_ERR：持有者保留槽 / SIZE_ERR: the holder keeps the slot
  TEST_ASSERT(r.ep.Submit(a, 513U, false, false) == ErrorCode::SIZE_ERR);
  TEST_ASSERT(r.ep.Submit(a, 512U, false, false) == ErrorCode::OK);
  TEST_ASSERT(r.ep.Submit(a, 512U, false, false) == ErrorCode::STATE_ERR);

  // 更早的借出 / An earlier lend
  TEST_ASSERT(r.ep.ReleaseSlot(b) == ErrorCode::OK);
  TEST_ASSERT(r.ep.ReleaseSlot(b) == ErrorCode::STATE_ERR);
  TEST_ASSERT(r.ep.AcquireSlot(c) == ErrorCode::OK);
  TEST_ASSERT(r.ep.Submit(b, 1U, false, false) == ErrorCode::STATE_ERR);
  TEST_ASSERT(r.ep.ReleaseSlot(b) == ErrorCode::STATE_ERR);
  TEST_ASSERT(r.ep.Submit(Slot{}, 1U, false, false) == ErrorCode::STATE_ERR);
  TEST_ASSERT(r.ep.ReleaseSlot(Slot{}) == ErrorCode::STATE_ERR);
  TEST_ASSERT(r.ep.Submit(c, 3U, false, false) == ErrorCode::OK);

  SetGate(r.dev, Gate::ALL);
  r.Complete(512U);
  r.Complete(3U);
  TEST_ASSERT(r.rec.seen.size() == 2U);
  TEST_ASSERT(r.rec.At(0U).actual == 512U && r.rec.At(1U).actual == 3U);

  // STALLED 时也可借 / Borrowing works while STALLED
  TEST_ASSERT(Stall(r.dev, r.ep) == ErrorCode::OK);
  TEST_ASSERT(r.ep.AcquireSlot(a) == ErrorCode::OK);
  TEST_ASSERT(r.ep.ReleaseSlot(a) == ErrorCode::OK);
  TEST_ASSERT(ClearStall(r.dev, r.ep) == ErrorCode::OK);

  // Close 之后作废：Submit 得到 ABORTED，槽变空闲 / Stale after Close: Submit gets
  // ABORTED and the slot is free
  TEST_ASSERT(r.ep.AcquireSlot(a) == ErrorCode::OK);
  TEST_ASSERT(r.ep.AcquireSlot(b) == ErrorCode::OK);
  Close(r.dev, r.ep);
  TEST_ASSERT(r.ep.Submit(a, 1U, false, false) == ErrorCode::ABORTED);
  TEST_ASSERT(r.ep.ReleaseSlot(b) == ErrorCode::OK);
  TEST_ASSERT(r.ep.ReleaseSlot(b) == ErrorCode::STATE_ERR);
  TEST_ASSERT(Configure(r.dev, r.ep, Direction::IN, Type::BULK, 64U) == ErrorCode::OK);
  TEST_ASSERT(r.ep.AcquireSlot(a) == ErrorCode::OK);
  TEST_ASSERT(r.ep.AcquireSlot(b) == ErrorCode::OK);
  TEST_ASSERT(r.rec.seen.size() == 2U);

  // Close 后重绑成更小的 MaxTransferSize：作废的槽以超限长度 Submit 仍必须判 ABORTED
  // 并归还槽 / After Close + rebind to a smaller MaxTransferSize: submitting a stale
  // slot with length > new MaxTransferSize must return ABORTED and free the slot
  Close(r.dev, r.ep);
  r.ep.limits.max_packets = 1U;
  TEST_ASSERT(Configure(r.dev, r.ep, Direction::IN, Type::BULK, 64U) == ErrorCode::OK);
  TEST_ASSERT(r.ep.MaxTransferSize() == 64U);
  TEST_ASSERT(r.ep.Submit(a, 128U, false, false) == ErrorCode::ABORTED);
  TEST_ASSERT(r.ep.ReleaseSlot(b) == ErrorCode::OK);
  TEST_ASSERT(r.ep.AcquireSlot(a) == ErrorCode::OK);
  TEST_ASSERT(r.ep.AcquireSlot(b) == ErrorCode::OK);
  TEST_ASSERT(r.ep.ReleaseSlot(a) == ErrorCode::OK);
  TEST_ASSERT(r.ep.ReleaseSlot(b) == ErrorCode::OK);
}

/// STALLED 时提交排队，ClearStall 后启动 / Submitted while STALLED, started by
/// ClearStall
void CheckInWhileStalled()
{
  Rig r(EPNumber::EP1, Direction::IN);
  TEST_ASSERT(Stall(r.dev, r.ep) == ErrorCode::OK);
  TEST_ASSERT(r.ep.IsStalled());
  r.Send(7U);
  TEST_ASSERT(r.ep.Count(Hw::START) == 0U);
  TEST_ASSERT(ClearStall(r.dev, r.ep) == ErrorCode::OK);
  TEST_ASSERT(r.ep.GetState() == State::IDLE);
  TEST_ASSERT(r.ep.Count(Hw::START) == 1U);
  r.Complete(7U);
  TEST_ASSERT(r.rec.seen.size() == 1U && r.rec.At(0U).result == ErrorCode::OK);
}

// ---------------------------------------------------------------------------
// OUT
// ---------------------------------------------------------------------------

/// Receive、完成借出槽、ReleaseSlot、再挂接收 / Receive, lent on completion, release,
/// re-arm
void CheckOutBasic()
{
  Rig r(EPNumber::EP2, Direction::OUT);
  TEST_ASSERT(r.ep.Receive(128U, false) == ErrorCode::OK);
  TEST_ASSERT(r.ep.Count(Hw::START) == 1U);
  const HwCall first = r.ep.Start(0U);
  TEST_ASSERT(first.length == 128U);
  uint8_t* data = const_cast<uint8_t*>(first.data);
  for (size_t i = 0U; i < 50U; ++i)
  {
    data[i] = static_cast<uint8_t>(i * 3U);
  }
  r.Complete(50U);
  TEST_ASSERT(r.rec.seen.size() == 1U);
  const Completion done = r.rec.At(0U);
  TEST_ASSERT(done.result == ErrorCode::OK);
  TEST_ASSERT(done.actual == 50U);
  TEST_ASSERT(done.slot.Valid());
  TEST_ASSERT(done.slot.data.size_ == 50U);
  TEST_ASSERT(done.slot.data.addr_ == data);
  for (size_t i = 0U; i < 50U; ++i)
  {
    TEST_ASSERT(static_cast<const uint8_t*>(done.slot.data.addr_)[i] ==
                static_cast<uint8_t>(i * 3U));
  }

  // 槽借给接收者：另一个槽在途后再无空闲 / The slot is lent: once the other is in
  // flight nothing is free
  TEST_ASSERT(r.ep.Receive(64U, false) == ErrorCode::OK);
  TEST_ASSERT(r.ep.Receive(64U, false) == ErrorCode::BUSY);
  // OUT 的句柄不能 Submit，槽不受影响 / An OUT handle cannot Submit; slot untouched
  TEST_ASSERT(r.ep.Submit(done.slot, 10U, false, false) == ErrorCode::STATE_ERR);
  TEST_ASSERT(r.ep.ReleaseSlot(done.slot) == ErrorCode::OK);
  TEST_ASSERT(r.ep.ReleaseSlot(done.slot) == ErrorCode::STATE_ERR);
  TEST_ASSERT(r.ep.Receive(64U, false) == ErrorCode::OK);
  TEST_ASSERT(r.ep.Count(Hw::START) == 2U);
  r.Complete(64U);
  TEST_ASSERT(r.ep.Count(Hw::START) == 3U);
  TEST_ASSERT(r.rec.seen.size() == 2U);
  TEST_ASSERT(r.ep.ReleaseSlot(r.rec.At(1U).slot) == ErrorCode::OK);
  r.Complete(0U);
  TEST_ASSERT(r.rec.seen.size() == 3U);
  TEST_ASSERT(r.rec.At(2U).result == ErrorCode::OK && r.rec.At(2U).actual == 0U);
  TEST_ASSERT(r.rec.At(2U).slot.Valid());
  TEST_ASSERT(r.ep.ReleaseSlot(r.rec.At(2U).slot) == ErrorCode::OK);
}

/// 长度规则与返回码 / Length rules and return codes
void CheckOutLengths()
{
  Rig r(EPNumber::EP2, Direction::OUT);
  TEST_ASSERT(r.ep.Receive(100U, false) == ErrorCode::SIZE_ERR);
  TEST_ASSERT(r.ep.Receive(576U, false) == ErrorCode::SIZE_ERR);
  TEST_ASSERT(r.ep.Receive(0U, false) == ErrorCode::SIZE_ERR);
  TEST_ASSERT(r.ep.Count(Hw::START) == 0U);
  // 被拒绝的长度不占槽 / A refused length takes no slot
  TEST_ASSERT(r.ep.Receive(512U, false) == ErrorCode::OK);
  TEST_ASSERT(r.ep.Receive(64U, false) == ErrorCode::OK);
  TEST_ASSERT(r.ep.Receive(64U, false) == ErrorCode::BUSY);
  TEST_ASSERT(r.ep.Start(0U).length == 512U);

  // 未启用、不是 OUT / Not enabled, not OUT
  Memory other;
  FakeEndpoint idle(EPNumber::EP3, Direction::OUT, {other.bytes, 1024U});
  FakeEndpoint in(EPNumber::EP4, Direction::IN, {other.bytes + 1024, 1024U});
  r.dev.Add(idle);
  r.dev.Add(in);
  TEST_ASSERT(idle.Receive(64U, false) == ErrorCode::STATE_ERR);
  TEST_ASSERT(Configure(r.dev, in, Direction::IN, Type::BULK, 64U) == ErrorCode::OK);
  TEST_ASSERT(in.Receive(64U, false) == ErrorCode::STATE_ERR);

  // EP0 状态阶段 / EP0 status stage
  Rig ep0(EPNumber::EP0, Direction::OUT, Type::CONTROL, false, 128U);
  TEST_ASSERT(ep0.ep.MaxTransferSize() == 64U);
  TEST_ASSERT(ep0.ep.Receive(128U, false) == ErrorCode::SIZE_ERR);
  TEST_ASSERT(ep0.ep.Receive(0U, false) == ErrorCode::OK);
  TEST_ASSERT(ep0.ep.Start(0U).length == 0U);
  ep0.Complete(0U);
  TEST_ASSERT(ep0.rec.seen.size() == 1U && ep0.rec.At(0U).result == ErrorCode::OK);
  TEST_ASSERT(ep0.ep.ReleaseSlot(ep0.rec.At(0U).slot) == ErrorCode::OK);
  TEST_ASSERT(ep0.ep.Receive(64U, false) == ErrorCode::OK);

  // 0 字节按控制类型判断，不按端点号 / 0 bytes is decided by the control type, not the
  // endpoint number
  Rig ctl(EPNumber::EP5, Direction::OUT, Type::CONTROL);
  TEST_ASSERT(ctl.ep.MaxTransferSize() == 64U);
  TEST_ASSERT(ctl.ep.Receive(0U, false) == ErrorCode::OK);
  TEST_ASSERT(ctl.ep.Start(0U).length == 0U);
  Close(ep0.dev, ep0.ep);
  ep0.ep.limits.direct_memory = false;
  TEST_ASSERT(Configure(ep0.dev, ep0.ep, Direction::OUT, Type::INTR, 64U) ==
              ErrorCode::OK);
  TEST_ASSERT(ep0.ep.Receive(0U, false) == ErrorCode::SIZE_ERR);
  TEST_ASSERT(ep0.ep.Receive(64U, false) == ErrorCode::OK);
}

/// Close 之后作废的槽 / A slot made stale by Close
void CheckOutStale()
{
  Rig r(EPNumber::EP2, Direction::OUT);
  TEST_ASSERT(r.ep.Receive(64U, false) == ErrorCode::OK);
  r.Complete(64U);
  const Slot lent = r.rec.At(0U).slot;
  TEST_ASSERT(lent.Valid());
  Close(r.dev, r.ep);
  TEST_ASSERT(r.ep.GetState() == State::DISABLED);
  TEST_ASSERT(r.ep.Receive(64U, false) == ErrorCode::STATE_ERR);
  TEST_ASSERT(Configure(r.dev, r.ep, Direction::OUT, Type::BULK, 64U) == ErrorCode::OK);
  // 作废的槽只挡住自己 / The stale slot blocks only itself
  TEST_ASSERT(r.ep.Receive(64U, false) == ErrorCode::OK);
  TEST_ASSERT(r.ep.Receive(64U, false) == ErrorCode::BUSY);
  TEST_ASSERT(r.ep.ReleaseSlot(lent) == ErrorCode::OK);
  TEST_ASSERT(r.ep.ReleaseSlot(lent) == ErrorCode::STATE_ERR);
  TEST_ASSERT(r.ep.Receive(64U, false) == ErrorCode::OK);
}

// ---------------------------------------------------------------------------
// 结尾零长度包 / Closing zero-length packet
// ---------------------------------------------------------------------------

void CheckZlpSent(bool synchronous)
{
  Rig r(EPNumber::EP1, Direction::IN);
  if (synchronous)
  {
    r.ep.on_start = [](FakeEndpoint& ep, const FakeEndpoint::HwRequest& request,
                       bool in_isr) { ep.Report(in_isr, request.length, ErrorCode::OK); };
  }
  r.Send(128U, true);
  if (!synchronous)
  {
    TEST_ASSERT(r.ep.Count(Hw::START) == 1U);
    r.Complete(128U);
    TEST_ASSERT(r.rec.seen.empty());
    TEST_ASSERT(r.ep.Count(Hw::START) == 2U);
    r.Complete(0U);
  }
  TEST_ASSERT(r.ep.Count(Hw::START) == 2U);
  TEST_ASSERT(r.ep.Start(0U).length == 128U);
  TEST_ASSERT(r.ep.Start(1U).length == 0U);
  TEST_ASSERT(r.ep.Start(1U).data == r.ep.Start(0U).data);
  TEST_ASSERT(r.rec.seen.size() == 1U);
  TEST_ASSERT(r.rec.At(0U).result == ErrorCode::OK && r.rec.At(0U).actual == 128U);
}

void CheckZlpNotNeeded()
{
  Rig r(EPNumber::EP1, Direction::IN);
  r.Send(100U, true);
  r.Complete(100U);
  r.Send(0U, true);
  r.Complete(0U);
  r.Send(128U, false);
  r.Complete(128U);
  TEST_ASSERT(r.ep.Count(Hw::START) == 3U);
  TEST_ASSERT(r.rec.seen.size() == 3U);
  TEST_ASSERT(r.rec.At(0U).actual == 100U && r.rec.At(1U).actual == 0U &&
              r.rec.At(2U).actual == 128U);
}

/// 数据发完、零长度包未发时被停止 / Stopped after the data, before the closing packet
void CheckZlpWhileStopping(Ender how)
{
  Rig r(EPNumber::EP1, Direction::IN);
  r.Send(128U, true);
  // 硬件在停止期间发完了数据部分 / The hardware finished the data part during the stop
  r.ep.stop_outcomes = {{ErrorCode::OK, 128U}};
  End(r.dev, r.ep, how);
  if (how == Ender::FLUSH)
  {
    TEST_ASSERT(r.ep.CallsAre({Hw::START, Hw::STOP}));
  }
  else if (how == Ender::STALL)
  {
    TEST_ASSERT(r.ep.CallsAre({Hw::START, Hw::STOP, Hw::STALL}));
  }
  else
  {
    TEST_ASSERT(r.ep.CallsAre({Hw::START, Hw::STOP, Hw::CLOSE}));
  }
  TEST_ASSERT(r.rec.seen.size() == 1U);
  TEST_ASSERT(r.rec.At(0U).result == ErrorCode::ABORTED);
  TEST_ASSERT(r.rec.At(0U).actual == 128U);
  TEST_ASSERT(r.rec.seen[0].hw_calls == r.ep.calls.size());
}

/// 结尾零长度包在途时被停止 / Stopped while the closing packet is on the wire
void CheckZlpOnWireStopped(ErrorCode outcome)
{
  Rig r(EPNumber::EP1, Direction::IN);
  r.Send(128U, true);
  r.Complete(128U);
  TEST_ASSERT(r.ep.Count(Hw::START) == 2U && r.rec.seen.empty());
  r.ep.stop_outcomes = {{outcome, 0U}};
  Flush(r.dev, r.ep);
  TEST_ASSERT(r.rec.seen.size() == 1U);
  // 零长度包已发完则整个请求 OK / With the closing packet sent the request is OK
  TEST_ASSERT(r.rec.At(0U).result == outcome);
  TEST_ASSERT(r.rec.At(0U).actual == 128U);
}

/// 还欠结尾零长度包时数据部分出错：不发零长度包，原样交付 / The data part fails while
/// the closing packet is owed: no closing packet, delivered as reported
void CheckZlpOwedDataFails()
{
  Rig r(EPNumber::EP1, Direction::IN);
  r.Send(128U, true);
  r.Complete(5U, ErrorCode::FAILED);
  TEST_ASSERT(r.ep.Count(Hw::START) == 1U);
  TEST_ASSERT(r.rec.seen.size() == 1U);
  TEST_ASSERT(r.rec.At(0U).result == ErrorCode::FAILED && r.rec.At(0U).actual == 5U);
  r.Send(64U, true);
  r.Complete(64U, ErrorCode::OUT_OF_RANGE);
  TEST_ASSERT(r.ep.Count(Hw::START) == 2U);
  TEST_ASSERT(r.rec.seen.size() == 2U);
  TEST_ASSERT(r.rec.At(1U).result == ErrorCode::OUT_OF_RANGE &&
              r.rec.At(1U).actual == 64U);
}

/// 结尾零长度包启动失败 / The closing packet fails to start
void CheckZlpStartFails()
{
  Rig r(EPNumber::EP1, Direction::IN);
  r.ep.start_results = {ErrorCode::OK, ErrorCode::FAILED};
  r.Send(128U, true);
  r.Complete(128U);
  TEST_ASSERT(r.ep.Count(Hw::START) == 2U);
  TEST_ASSERT(r.rec.seen.size() == 1U);
  TEST_ASSERT(r.rec.At(0U).result == ErrorCode::FAILED);
  TEST_ASSERT(r.rec.At(0U).actual == 128U);
  r.Send(5U);
  r.Complete(5U);
  TEST_ASSERT(r.rec.seen.size() == 2U && r.rec.At(1U).result == ErrorCode::OK);
}

// ---------------------------------------------------------------------------
// Close、Flush、Stall、ClearStall
// ---------------------------------------------------------------------------

void CheckCloseIdle()
{
  Rig r(EPNumber::EP1, Direction::IN);
  Close(r.dev, r.ep);
  TEST_ASSERT(r.ep.GetState() == State::DISABLED);
  TEST_ASSERT(r.ep.Count(Hw::CLOSE) == 1U);
  TEST_ASSERT(r.ep.calls.size() == 1U);
  Close(r.dev, r.ep);
  TEST_ASSERT(r.ep.calls.size() == 1U);
  TEST_ASSERT(r.rec.seen.empty());
  TEST_ASSERT(Configure(r.dev, r.ep, Direction::IN, Type::BULK, 64U) == ErrorCode::OK);
}

/// 停止在途请求：顺序与结果 / Stopping the in-flight request: order and outcome
void CheckCloseInFlight(ErrorCode result, size_t actual)
{
  Rig r(EPNumber::EP1, Direction::IN);
  r.Send(10U);
  r.Send(20U);
  r.ep.stop_outcomes = {{result, actual}};
  Close(r.dev, r.ep);
  TEST_ASSERT(r.ep.GetState() == State::DISABLED);
  // StopHardware、CloseHardware，然后才回调 / StopHardware, CloseHardware, then the
  // callbacks
  TEST_ASSERT(r.ep.CallsAre({Hw::START, Hw::STOP, Hw::CLOSE}));
  TEST_ASSERT(r.rec.seen.size() == 2U);
  TEST_ASSERT(r.rec.seen[0].hw_calls == 3U && r.rec.seen[1].hw_calls == 3U);
  // 在途请求先，按停止结果；排队请求后，ABORTED / The in-flight request first, from the
  // stop outcome; the queued one last, with ABORTED
  TEST_ASSERT(r.rec.At(0U).result == result && r.rec.At(0U).actual == actual);
  TEST_ASSERT(r.rec.At(1U).result == ErrorCode::ABORTED && r.rec.At(1U).actual == 0U);
  TEST_ASSERT(!r.ep.nested && !r.rec.inside_backend);
  TEST_ASSERT(r.dev.StopFailures() == 0U);
  // 返回时已无在途请求，两个槽都空闲 / Nothing in flight on return, both slots free
  TEST_ASSERT(Configure(r.dev, r.ep, Direction::IN, Type::BULK, 64U) == ErrorCode::OK);
  Slot a;
  Slot b;
  TEST_ASSERT(r.ep.AcquireSlot(a) == ErrorCode::OK);
  TEST_ASSERT(r.ep.AcquireSlot(b) == ErrorCode::OK);
}

void CheckFlush()
{
  Rig r(EPNumber::EP1, Direction::IN);
  Flush(r.dev, r.ep);
  TEST_ASSERT(r.ep.calls.empty() && r.rec.seen.empty());
  TEST_ASSERT(r.ep.GetState() == State::IDLE);

  r.Send(10U);
  r.Send(20U);
  r.ep.stop_outcomes = {{ErrorCode::ABORTED, 5U}};
  std::vector<size_t> starts_at_callback;
  bool submitted = false;
  r.rec.hook = [&](const Completion&, bool)
  {
    starts_at_callback.push_back(r.ep.Count(Hw::START));
    if (!submitted)
    {
      submitted = true;
      r.Send(30U);
    }
  };
  Flush(r.dev, r.ep);
  TEST_ASSERT(r.ep.GetState() == State::IDLE);
  TEST_ASSERT(r.rec.seen.size() == 2U);
  TEST_ASSERT(r.rec.At(0U).result == ErrorCode::ABORTED && r.rec.At(0U).actual == 5U);
  TEST_ASSERT(r.rec.At(1U).result == ErrorCode::ABORTED && r.rec.At(1U).actual == 0U);
  // 回调里提交的请求排队，回调之后由 owner 启动 / A request submitted from a callback is
  // queued and the owner starts it after the callbacks
  TEST_ASSERT(starts_at_callback.size() == 2U);
  TEST_ASSERT(starts_at_callback[0] == 1U && starts_at_callback[1] == 1U);
  TEST_ASSERT(r.ep.Count(Hw::START) == 2U && r.ep.Start(1U).length == 30U);
  TEST_ASSERT(r.ep.Count(Hw::CLOSE) == 0U && r.ep.Count(Hw::STALL) == 0U);
  r.rec.hook = nullptr;
  r.Complete(30U);
  TEST_ASSERT(r.rec.seen.size() == 3U && r.rec.At(2U).result == ErrorCode::OK);

  // DISABLED 时不做任何事 / Nothing while DISABLED
  Close(r.dev, r.ep);
  const size_t calls = r.ep.calls.size();
  Flush(r.dev, r.ep);
  TEST_ASSERT(r.ep.calls.size() == calls);
}

/// 停止期间收完的 OUT 按 ABORTED 交付 / An OUT received during the stop is ABORTED
void CheckOutWhileStopping()
{
  Rig r(EPNumber::EP2, Direction::OUT);
  TEST_ASSERT(r.ep.Receive(128U, false) == ErrorCode::OK);
  r.ep.stop_outcomes = {{ErrorCode::OK, 64U}};
  Flush(r.dev, r.ep);
  TEST_ASSERT(r.rec.seen.size() == 1U);
  TEST_ASSERT(r.rec.At(0U).result == ErrorCode::ABORTED);
  TEST_ASSERT(r.rec.At(0U).actual == 64U);
  TEST_ASSERT(!r.rec.At(0U).slot.Valid());
  TEST_ASSERT(r.ep.Receive(64U, false) == ErrorCode::OK);
  TEST_ASSERT(r.ep.Receive(64U, false) == ErrorCode::OK);
  TEST_ASSERT(r.ep.Receive(64U, false) == ErrorCode::BUSY);
}

void CheckStallIdle()
{
  Rig r(EPNumber::EP1, Direction::IN);
  TEST_ASSERT(Stall(r.dev, r.ep) == ErrorCode::OK);
  TEST_ASSERT(r.ep.GetState() == State::STALLED);
  TEST_ASSERT(r.ep.CallsAre({Hw::STALL}));
  // 已是 STALLED：OK，硬件不再动 / Already STALLED: OK, the hardware is left alone
  TEST_ASSERT(Stall(r.dev, r.ep) == ErrorCode::OK);
  TEST_ASSERT(r.ep.GetState() == State::STALLED);
  TEST_ASSERT(r.ep.CallsAre({Hw::STALL}));
  TEST_ASSERT(ClearStall(r.dev, r.ep) == ErrorCode::OK);
  TEST_ASSERT(r.ep.GetState() == State::IDLE);
  TEST_ASSERT(r.ep.CallsAre({Hw::STALL, Hw::CLEAR_STALL}));

  // ClearStallHardware 失败：状态不变，排队请求不启动 / ClearStallHardware fails: the
  // state is unchanged and the queued request does not start
  r.ep.clear_result = ErrorCode::FAILED;
  TEST_ASSERT(Stall(r.dev, r.ep) == ErrorCode::OK);
  r.Send(7U);
  TEST_ASSERT(ClearStall(r.dev, r.ep) == ErrorCode::FAILED);
  TEST_ASSERT(r.ep.GetState() == State::STALLED);
  TEST_ASSERT(r.ep.Count(Hw::START) == 0U);
  r.ep.clear_result = ErrorCode::OK;
  TEST_ASSERT(ClearStall(r.dev, r.ep) == ErrorCode::OK);
  TEST_ASSERT(r.ep.Count(Hw::START) == 1U);
  r.Complete(7U);
  TEST_ASSERT(r.rec.seen.size() == 1U && r.rec.At(0U).result == ErrorCode::OK);
  r.ep.clear_result = ErrorCode::FAILED;
  TEST_ASSERT(ClearStall(r.dev, r.ep) == ErrorCode::FAILED);
  TEST_ASSERT(r.ep.GetState() == State::IDLE);

  Close(r.dev, r.ep);
  TEST_ASSERT(Stall(r.dev, r.ep) == ErrorCode::STATE_ERR);
  TEST_ASSERT(ClearStall(r.dev, r.ep) == ErrorCode::STATE_ERR);
  TEST_ASSERT(r.ep.GetState() == State::DISABLED);
}

void CheckStallInFlight()
{
  Rig r(EPNumber::EP1, Direction::IN);
  r.Send(10U);
  r.Send(20U);
  r.ep.stop_outcomes = {{ErrorCode::ABORTED, 2U}};
  TEST_ASSERT(Stall(r.dev, r.ep) == ErrorCode::OK);
  TEST_ASSERT(r.ep.GetState() == State::STALLED);
  // StallHardware 在回调之前 / StallHardware before the callbacks
  TEST_ASSERT(r.ep.CallsAre({Hw::START, Hw::STOP, Hw::STALL}));
  TEST_ASSERT(r.rec.seen.size() == 2U);
  TEST_ASSERT(r.rec.seen[0].hw_calls == 3U && r.rec.seen[1].hw_calls == 3U);
  TEST_ASSERT(r.rec.At(0U).result == ErrorCode::ABORTED && r.rec.At(0U).actual == 2U);
  TEST_ASSERT(r.rec.At(1U).result == ErrorCode::ABORTED && r.rec.At(1U).actual == 0U);

  r.Send(30U);
  TEST_ASSERT(r.ep.Count(Hw::START) == 1U);
  TEST_ASSERT(ClearStall(r.dev, r.ep) == ErrorCode::OK);
  TEST_ASSERT(r.ep.Count(Hw::START) == 2U);
  r.Complete(30U);
  TEST_ASSERT(r.rec.seen.size() == 3U && r.rec.At(2U).result == ErrorCode::OK);
}

/// IDLE 时 ClearStall 也调用 ClearStallHardware / ClearStall while IDLE also calls
/// ClearStallHardware
void CheckClearStallIdle()
{
  Rig r(EPNumber::EP1, Direction::IN);
  TEST_ASSERT(ClearStall(r.dev, r.ep) == ErrorCode::OK);
  TEST_ASSERT(r.ep.CallsAre({Hw::CLEAR_STALL}));
  TEST_ASSERT(r.ep.GetState() == State::IDLE);

  // 没有 STALL 而有请求在硬件上时：只调用 ClearStallHardware，不停止、不回调，请求留在
  // 硬件上，之后照常完成
  // Not halted and with a request on the hardware: only ClearStallHardware is called, no
  // stop and no callback; the request stays on the hardware and completes as usual
  r.ep.calls.clear();
  r.Send(10U);
  TEST_ASSERT(ClearStall(r.dev, r.ep) == ErrorCode::OK);
  TEST_ASSERT(r.ep.CallsAre({Hw::START, Hw::CLEAR_STALL}));
  TEST_ASSERT(r.ep.busy && r.rec.seen.empty() && r.ep.GetState() == State::IDLE);
  r.Complete(10U);
  TEST_ASSERT(r.rec.seen.size() == 1U && r.rec.At(0).result == ErrorCode::OK &&
              r.rec.At(0).actual == 10U);
}

// ---------------------------------------------------------------------------
// 停止失败 / Failed stop
// ---------------------------------------------------------------------------

/**
 * @brief 停止挂起后报告为 TIMEOUT，控制器复位后结束 / A pending stop reported as
 *        TIMEOUT, ended after the controller reset
 * @note 被停请求在停止时以 ABORTED 回调；TIMEOUT 不再回调、不执行硬件动作；控制器复位
 *       之前端点不能再绑定，Flush、Stall 留下的端点不启动新请求。
 *       The stopped request is called back ABORTED at the stop; TIMEOUT makes no
 *       callback and runs no hardware step; until the controller reset the endpoint
 *       cannot bind again, and after Flush or Stall it starts no new request.
 */
void CheckStopFails(Ender how, bool direct)
{
  Rig r(EPNumber::EP1, Direction::IN);
  r.ep.limits.direct_memory = direct;
  r.Send(10U);
  r.Send(20U);
  r.ep.stop_outcomes = {{ErrorCode::PENDING, 0U}};
  End(r.dev, r.ep, how);
  TEST_ASSERT(r.ep.CallsAre({Hw::START, Hw::STOP}));
  TEST_ASSERT(r.rec.seen.size() == 2U);
  TEST_ASSERT(r.rec.At(0U).result == ErrorCode::ABORTED && r.rec.At(0U).actual == 0U);
  TEST_ASSERT(r.rec.At(1U).result == ErrorCode::ABORTED);

  r.Complete(0U, ErrorCode::TIMEOUT);
  TEST_ASSERT(r.ep.CallsAre({Hw::START, Hw::STOP}));
  TEST_ASSERT(r.rec.seen.size() == 2U);
  // 设备处理程序在同一个 owner 的下一轮收到 STOP_FAILED_EVENT / The device handler gets
  // STOP_FAILED_EVENT in the same owner's next pass
  TEST_ASSERT(r.dev.StopFailures() == 1U);
  TEST_ASSERT(r.dev.handled.back() == DeviceService::STOP_FAILED_EVENT);

  if (how == Ender::CLOSE)
  {
    TEST_ASSERT(r.ep.GetState() == State::DISABLED);
    TEST_ASSERT(Configure(r.dev, r.ep, Direction::IN, Type::BULK, 64U) ==
                ErrorCode::STATE_ERR);
    Flush(r.dev, r.ep);
  }
  else
  {
    TEST_ASSERT(r.ep.GetState() == (how == Ender::STALL ? State::STALLED : State::IDLE));
    r.Send(30U);
  }
  TEST_ASSERT(r.ep.CallsAre({Hw::START, Hw::STOP}));

  r.dev.AsOwner([&](bool in_isr) { r.dev.ResetAndCloseAll(in_isr); });
  TEST_ASSERT(r.ep.CallsAre({Hw::START, Hw::STOP, Hw::STOP, Hw::CLOSE}));
  TEST_ASSERT(r.ep.GetState() == State::DISABLED);
  TEST_ASSERT(r.rec.seen.size() == (how == Ender::CLOSE ? 2U : 3U));
  TEST_ASSERT(r.rec.seen.back().completion.result == ErrorCode::ABORTED);
  TEST_ASSERT(Configure(r.dev, r.ep, Direction::IN, Type::BULK, 64U) == ErrorCode::OK);
  Slot a;
  Slot b;
  TEST_ASSERT(r.ep.AcquireSlot(a) == ErrorCode::OK);
  TEST_ASSERT(r.ep.AcquireSlot(b) == ErrorCode::OK);
  TEST_ASSERT(r.ep.Submit(a, 40U, false, false) == ErrorCode::OK);
  TEST_ASSERT(r.ep.Count(Hw::START) == 2U && r.ep.Start(1U).length == 40U);
}

// ---------------------------------------------------------------------------
// 回调里的生命周期调用 / Lifecycle calls from completion callbacks
// ---------------------------------------------------------------------------

/// Stall 的排队回调里 Close 同一端点 / Close from the queued callback of a Stall
void CheckCloseInQueuedCallback()
{
  Rig r(EPNumber::EP1, Direction::IN);
  SetGate(r.dev, Gate::CLOSED);
  r.Send(10U);
  r.Send(20U);
  bool done = false;
  r.rec.hook = [&](const Completion&, bool in_isr)
  {
    if (!done)
    {
      done = true;
      r.ep.Close(in_isr);
    }
  };
  TEST_ASSERT(Stall(r.dev, r.ep) == ErrorCode::OK);
  TEST_ASSERT(r.ep.GetState() == State::DISABLED);
  TEST_ASSERT(r.ep.CallsAre({Hw::STALL, Hw::CLOSE}));
  TEST_ASSERT(r.rec.seen.size() == 2U);
  TEST_ASSERT(r.rec.At(0U).result == ErrorCode::ABORTED);
  TEST_ASSERT(r.rec.At(1U).result == ErrorCode::ABORTED);
}

/// Close 的回调里 Configure 同一端点 / Configure from a callback of a Close
void CheckConfigureInCloseCallback(bool in_flight)
{
  Rig r(EPNumber::EP1, Direction::IN);
  if (!in_flight)
  {
    SetGate(r.dev, Gate::CLOSED);
  }
  r.Send(10U);
  r.Send(20U);
  ErrorCode inner = ErrorCode::PENDING;
  // 新绑定的回调 / Callback of the new binding
  std::vector<Completion> rebound;
  r.rec.hook = [&](const Completion&, bool in_isr)
  {
    inner = r.ep.Configure({Direction::IN, Type::BULK, 64U}, in_isr);
    r.ep.SetOnComplete(Callback<const Completion&>::Create(
        [](bool, std::vector<Completion>* out, const Completion& completion)
        { out->push_back(completion); }, &rebound));
  };
  Close(r.dev, r.ep);
  TEST_ASSERT(inner == ErrorCode::OK);
  TEST_ASSERT(r.ep.GetState() == State::IDLE);
  if (in_flight)
  {
    TEST_ASSERT(r.ep.CallsAre({Hw::START, Hw::STOP, Hw::CLOSE, Hw::CONFIGURE}));
  }
  else
  {
    TEST_ASSERT(r.ep.CallsAre({Hw::CLOSE, Hw::CONFIGURE}));
  }
  // 旧绑定排队请求的 ABORTED 在重新绑定之后到达，进入当时设置的回调 / The ABORTED of
  // the old binding's queued request arrives after the rebind, in the callback set then
  TEST_ASSERT(r.rec.seen.size() == 1U);
  TEST_ASSERT(rebound.size() == 1U && rebound[0].result == ErrorCode::ABORTED);
  SetGate(r.dev, Gate::ALL);
  r.Send(5U);
  TEST_ASSERT(r.ep.Count(Hw::START) == (in_flight ? 2U : 1U));
}

/// 普通完成的回调里 Close 或 Flush 同一端点 / Close or Flush from the callback of a
/// normal completion
void CheckEndInCompletionCallback(Ender how)
{
  Rig r(EPNumber::EP1, Direction::IN);
  r.Send(10U);
  r.Send(20U);
  bool done = false;
  r.rec.hook = [&](const Completion&, bool in_isr)
  {
    if (done)
    {
      return;
    }
    done = true;
    if (how == Ender::CLOSE)
    {
      r.ep.Close(in_isr);
    }
    else if (how == Ender::STALL)
    {
      TEST_ASSERT(r.ep.Stall(in_isr) == ErrorCode::OK);
    }
    else
    {
      r.ep.Flush(in_isr);
    }
  };
  r.Complete(10U);
  TEST_ASSERT(r.rec.seen.size() == 2U);
  TEST_ASSERT(r.rec.At(0U).result == ErrorCode::OK && r.rec.At(0U).actual == 10U);
  TEST_ASSERT(r.rec.At(1U).result == ErrorCode::ABORTED && r.rec.At(1U).actual == 0U);
  // 第二个请求从未启动，也没有停止 / The second request never started nor was stopped
  if (how == Ender::CLOSE)
  {
    TEST_ASSERT(r.ep.CallsAre({Hw::START, Hw::CLOSE}));
    TEST_ASSERT(r.ep.GetState() == State::DISABLED);
  }
  else if (how == Ender::STALL)
  {
    TEST_ASSERT(r.ep.CallsAre({Hw::START, Hw::STALL}));
    TEST_ASSERT(r.ep.GetState() == State::STALLED);
  }
  else
  {
    TEST_ASSERT(r.ep.CallsAre({Hw::START}));
    TEST_ASSERT(r.ep.GetState() == State::IDLE);
    r.Send(5U);
    TEST_ASSERT(r.ep.Count(Hw::START) == 2U);
  }
}

/// 被停请求的回调里 Stall 同一端点 / Stall from the callback of a stopped request
void CheckStallInStoppedCallback()
{
  Rig r(EPNumber::EP1, Direction::IN);
  r.Send(10U);
  ErrorCode inner = ErrorCode::PENDING;
  r.rec.hook = [&](const Completion&, bool in_isr) { inner = r.ep.Stall(in_isr); };
  Flush(r.dev, r.ep);
  TEST_ASSERT(inner == ErrorCode::OK);
  TEST_ASSERT(r.ep.GetState() == State::STALLED);
  TEST_ASSERT(r.ep.CallsAre({Hw::START, Hw::STOP, Hw::STALL}));
  TEST_ASSERT(r.rec.seen.size() == 1U);
}

/// 一个端点的回调里对另一个端点 Flush、Stall、Close、Configure / Flush, Stall, Close
/// and Configure on another endpoint from one endpoint's callback
void CheckLifecycleOnOtherEndpoint()
{
  Memory mem;
  FakeDevice dev(false);
  FakeEndpoint a(EPNumber::EP1, Direction::IN, {mem.bytes, 1024U});
  FakeEndpoint b(EPNumber::EP2, Direction::IN, {mem.bytes + 1024, 1024U});
  dev.Add(a);
  dev.Add(b);
  Recorder rec_a(a);
  Recorder rec_b(b);
  SetGate(dev, Gate::ALL);
  TEST_ASSERT(Configure(dev, a, Direction::IN, Type::BULK, 64U) == ErrorCode::OK);
  TEST_ASSERT(Configure(dev, b, Direction::IN, Type::BULK, 64U) == ErrorCode::OK);
  a.calls.clear();
  b.calls.clear();
  auto send = [](FakeEndpoint& ep, size_t length)
  {
    Slot slot;
    TEST_ASSERT(ep.AcquireSlot(slot) == ErrorCode::OK);
    TEST_ASSERT(ep.Submit(slot, length, false, false) == ErrorCode::OK);
  };
  auto complete_a = [&]()
  {
    send(a, 1U);
    a.Later(1U);
    dev.Irq();
  };

  int step = 0;
  ErrorCode inner = ErrorCode::PENDING;
  rec_a.hook = [&](const Completion&, bool in_isr)
  {
    switch (step++)
    {
      case 0:
        b.Flush(in_isr);
        break;
      case 1:
        inner = b.Stall(in_isr);
        break;
      case 2:
        b.Close(in_isr);
        break;
      default:
        inner = b.Configure({Direction::IN, Type::BULK, 64U}, in_isr);
        break;
    }
  };
  b.stop_outcomes = {
      {ErrorCode::ABORTED, 1U}, {ErrorCode::ABORTED, 2U}, {ErrorCode::ABORTED, 3U}};

  send(b, 7U);
  complete_a();
  TEST_ASSERT(b.CallsAre({Hw::START, Hw::STOP}));
  TEST_ASSERT(b.GetState() == State::IDLE);
  TEST_ASSERT(rec_b.seen.size() == 1U && rec_b.At(0U).actual == 1U);
  TEST_ASSERT(rec_b.seen[0].in_isr);

  send(b, 8U);
  complete_a();
  TEST_ASSERT(inner == ErrorCode::OK);
  TEST_ASSERT(b.GetState() == State::STALLED);
  TEST_ASSERT(b.CallsAre({Hw::START, Hw::STOP, Hw::START, Hw::STOP, Hw::STALL}));
  TEST_ASSERT(rec_b.seen.size() == 2U && rec_b.At(1U).actual == 2U);

  TEST_ASSERT(ClearStall(dev, b) == ErrorCode::OK);
  send(b, 9U);
  TEST_ASSERT(b.Count(Hw::START) == 3U);
  complete_a();
  TEST_ASSERT(b.GetState() == State::DISABLED);
  TEST_ASSERT(b.calls[b.calls.size() - 2U].function == Hw::STOP);
  TEST_ASSERT(b.calls.back().function == Hw::CLOSE);
  TEST_ASSERT(rec_b.seen.size() == 3U && rec_b.At(2U).actual == 3U);

  inner = ErrorCode::PENDING;
  complete_a();
  TEST_ASSERT(inner == ErrorCode::OK);
  TEST_ASSERT(b.GetState() == State::IDLE);
  TEST_ASSERT(b.calls.back().function == Hw::CONFIGURE);
  TEST_ASSERT(rec_a.seen.size() == 4U);
  TEST_ASSERT(a.Count(Hw::START) == 4U && a.calls.size() == 4U);
  TEST_ASSERT(!a.nested && !b.nested);
}

// ---------------------------------------------------------------------------
// 硬件错误 / Hardware errors
// ---------------------------------------------------------------------------

/// 后端报告的错误原样交付，槽变空闲 / A reported error is delivered as is and the slot
/// becomes free
void CheckHardwareErrors()
{
  Rig r(EPNumber::EP1, Direction::IN);
  r.Send(10U);
  r.Complete(3U, ErrorCode::FAILED);
  r.Send(64U);
  r.Complete(64U, ErrorCode::OUT_OF_RANGE);
  // StartHardware 之内报告的错误 / An error reported inside StartHardware
  r.ep.on_start = [](FakeEndpoint& ep, const FakeEndpoint::HwRequest&, bool in_isr)
  { ep.Report(in_isr, 0U, ErrorCode::FAILED); };
  r.Send(5U);
  r.ep.on_start = nullptr;
  // 停止返回的错误 / An error returned by the stop
  r.Send(10U);
  r.ep.stop_outcomes = {{ErrorCode::FAILED, 2U}};
  Flush(r.dev, r.ep);
  TEST_ASSERT(r.rec.seen.size() == 4U);
  TEST_ASSERT(r.rec.At(0U).result == ErrorCode::FAILED && r.rec.At(0U).actual == 3U);
  TEST_ASSERT(r.rec.At(1U).result == ErrorCode::OUT_OF_RANGE &&
              r.rec.At(1U).actual == 64U);
  TEST_ASSERT(r.rec.At(2U).result == ErrorCode::FAILED && r.rec.At(2U).actual == 0U);
  TEST_ASSERT(r.rec.At(3U).result == ErrorCode::FAILED && r.rec.At(3U).actual == 2U);
  for (const Seen& seen : r.rec.seen)
  {
    TEST_ASSERT(!seen.completion.slot.Valid());
  }
  Slot a;
  Slot b;
  TEST_ASSERT(r.ep.AcquireSlot(a) == ErrorCode::OK);
  TEST_ASSERT(r.ep.AcquireSlot(b) == ErrorCode::OK);

  Rig o(EPNumber::EP2, Direction::OUT);
  TEST_ASSERT(o.ep.Receive(128U, false) == ErrorCode::OK);
  o.Complete(64U, ErrorCode::OUT_OF_RANGE);
  TEST_ASSERT(o.ep.Receive(128U, false) == ErrorCode::OK);
  o.Complete(0U, ErrorCode::FAILED);
  TEST_ASSERT(o.ep.Receive(64U, false) == ErrorCode::OK);
  o.ep.stop_outcomes = {{ErrorCode::FAILED, 10U}};
  Flush(o.dev, o.ep);
  TEST_ASSERT(o.rec.seen.size() == 3U);
  TEST_ASSERT(o.rec.At(0U).result == ErrorCode::OUT_OF_RANGE &&
              o.rec.At(0U).actual == 64U);
  TEST_ASSERT(o.rec.At(1U).result == ErrorCode::FAILED && o.rec.At(1U).actual == 0U);
  TEST_ASSERT(o.rec.At(2U).result == ErrorCode::FAILED && o.rec.At(2U).actual == 10U);
  for (const Seen& seen : o.rec.seen)
  {
    TEST_ASSERT(!seen.completion.slot.Valid());
  }
  // OUT 出错时槽空闲 / On an OUT error the slot is free
  TEST_ASSERT(o.ep.Receive(64U, false) == ErrorCode::OK);
  TEST_ASSERT(o.ep.Receive(64U, false) == ErrorCode::OK);
  TEST_ASSERT(o.ep.Receive(64U, false) == ErrorCode::BUSY);
}

// ---------------------------------------------------------------------------
// 重入与失败 / Re-entrancy and failures
// ---------------------------------------------------------------------------

/// StartHardware 之内的报告被留存 / A report inside StartHardware is held
void CheckHeldStart()
{
  Rig r(EPNumber::EP1, Direction::IN);
  bool held_quietly = false;
  r.ep.on_start =
      [&](FakeEndpoint& ep, const FakeEndpoint::HwRequest& request, bool in_isr)
  {
    const size_t calls = ep.calls.size();
    const size_t seen = r.rec.seen.size();
    ep.Report(in_isr, request.length, ErrorCode::OK);
    held_quietly = ep.calls.size() == calls && r.rec.seen.size() == seen;
  };
  r.Send(9U);
  // 在引起调用的核心函数返回前处理 / Handled before the core function returns
  TEST_ASSERT(held_quietly);
  TEST_ASSERT(r.rec.seen.size() == 1U);
  TEST_ASSERT(r.rec.At(0U).result == ErrorCode::OK && r.rec.At(0U).actual == 9U);
  TEST_ASSERT(!r.ep.nested && !r.rec.inside_backend);
}

/// 回调里再提交 / Submitting again from inside the completion callback
void CheckResubmitInCallback(bool synchronous)
{
  Rig r(EPNumber::EP1, Direction::IN);
  if (synchronous)
  {
    r.ep.on_start = [](FakeEndpoint& ep, const FakeEndpoint::HwRequest& request,
                       bool in_isr) { ep.Report(in_isr, request.length, ErrorCode::OK); };
  }
  size_t next = 1U;
  r.rec.hook = [&](const Completion&, bool in_isr)
  {
    if (next < 5U)
    {
      Slot slot;
      TEST_ASSERT(r.ep.AcquireSlot(slot) == ErrorCode::OK);
      TEST_ASSERT(r.ep.Submit(slot, next, false, in_isr) == ErrorCode::OK);
      ++next;
    }
  };
  r.Send(0U);
  if (!synchronous)
  {
    for (size_t i = 0U; i < 5U; ++i)
    {
      TEST_ASSERT(r.rec.seen.size() == i);
      r.Complete(i);
    }
  }
  TEST_ASSERT(r.rec.seen.size() == 5U);
  for (size_t i = 0U; i < 5U; ++i)
  {
    TEST_ASSERT(r.rec.At(i).result == ErrorCode::OK && r.rec.At(i).actual == i);
    TEST_ASSERT(r.ep.Start(i).length == i);
    TEST_ASSERT(r.rec.seen[i].hw_calls == i + 1U);
  }
  TEST_ASSERT(!r.ep.start_in_callback);
  TEST_ASSERT(!r.ep.nested && !r.rec.inside_backend);
}

/// 没有在途请求时报告被忽略 / A report with nothing in flight is ignored
void CheckIgnoredReport()
{
  Rig r(EPNumber::EP1, Direction::IN);
  r.Complete(10U);
  TEST_ASSERT(r.rec.seen.empty() && r.ep.calls.empty());
  r.Send(4U);
  r.ep.Later(4U);
  r.ep.Later(4U);
  r.dev.Irq();
  TEST_ASSERT(r.rec.seen.size() == 1U);
  TEST_ASSERT(r.ep.calls.size() == 1U);
}

/// 启动失败以 FAILED 完成并尝试下一个 / A failed start completes with FAILED and the
/// next queued one is tried
void CheckFailedStart()
{
  Rig r(EPNumber::EP1, Direction::IN);
  SetGate(r.dev, Gate::CLOSED);
  r.Send(10U);
  r.Send(20U);
  r.ep.start_results = {ErrorCode::FAILED};
  SetGate(r.dev, Gate::ALL);
  TEST_ASSERT(r.rec.seen.size() == 1U);
  TEST_ASSERT(r.rec.At(0U).result == ErrorCode::FAILED && r.rec.At(0U).actual == 0U);
  TEST_ASSERT(r.ep.Count(Hw::START) == 2U);
  TEST_ASSERT(r.ep.Start(1U).length == 20U);
  r.Complete(20U);
  TEST_ASSERT(r.rec.seen.size() == 2U && r.rec.At(1U).result == ErrorCode::OK);
  // 失败的请求的槽已空闲 / The failed request's slot is free
  Slot a;
  Slot b;
  TEST_ASSERT(r.ep.AcquireSlot(a) == ErrorCode::OK);
  TEST_ASSERT(r.ep.AcquireSlot(b) == ErrorCode::OK);
}

// ---------------------------------------------------------------------------
// 设备串行服务 / Device service
// ---------------------------------------------------------------------------

void CheckGate()
{
  Memory mem;
  FakeDevice dev(false);
  FakeEndpoint ep0(EPNumber::EP0, Direction::IN, {mem.bytes, 128U});
  FakeEndpoint ep1(EPNumber::EP1, Direction::IN, {mem.bytes + 128, 1024U});
  dev.Add(ep0);
  dev.Add(ep1);
  Recorder rec0(ep0);
  Recorder rec1(ep1);
  TEST_ASSERT(Configure(dev, ep0, Direction::IN, Type::CONTROL, 64U) == ErrorCode::OK);
  TEST_ASSERT(Configure(dev, ep1, Direction::IN, Type::BULK, 64U) == ErrorCode::OK);
  Slot slot;
  TEST_ASSERT(ep0.AcquireSlot(slot) == ErrorCode::OK);
  TEST_ASSERT(ep0.Submit(slot, 8U, false, false) == ErrorCode::OK);
  TEST_ASSERT(ep1.AcquireSlot(slot) == ErrorCode::OK);
  TEST_ASSERT(ep1.Submit(slot, 9U, false, false) == ErrorCode::OK);
  TEST_ASSERT(ep1.AcquireSlot(slot) == ErrorCode::OK);
  TEST_ASSERT(ep1.Submit(slot, 10U, false, false) == ErrorCode::OK);
  TEST_ASSERT(ep0.Count(Hw::START) == 0U && ep1.Count(Hw::START) == 0U);

  SetGate(dev, Gate::EP0);
  TEST_ASSERT(ep0.Count(Hw::START) == 1U && ep1.Count(Hw::START) == 0U);
  SetGate(dev, Gate::ALL);
  TEST_ASSERT(ep1.Count(Hw::START) == 1U);

  // 关闸不停止在途请求，排队请求等到再开 / Closing stops nothing in flight; queued
  // requests wait for the next opening
  SetGate(dev, Gate::CLOSED);
  TEST_ASSERT(ep0.Count(Hw::STOP) == 0U && ep1.Count(Hw::STOP) == 0U);
  ep1.Later(9U);
  dev.Irq();
  TEST_ASSERT(rec1.seen.size() == 1U && rec1.At(0U).result == ErrorCode::OK);
  TEST_ASSERT(ep1.Count(Hw::START) == 1U);
  SetGate(dev, Gate::EP0);
  TEST_ASSERT(ep1.Count(Hw::START) == 1U);
  SetGate(dev, Gate::ALL);
  TEST_ASSERT(ep1.Count(Hw::START) == 2U);
  TEST_ASSERT(ep1.Start(1U).length == 10U);
}

/// 中断赢得 owner / The interrupt wins
void CheckInterruptWins()
{
  Rig r(EPNumber::EP1, Direction::IN);
  r.Send(3U);
  r.ep.Later(3U);
  r.dev.log.clear();
  r.dev.Irq();
  using Step = FakeDevice::Step;
  TEST_ASSERT(r.dev.log.size() == 3U);
  TEST_ASSERT(r.dev.log[0].step == Step::MASK);
  TEST_ASSERT(r.dev.log[1].step == Step::SERVICE && r.dev.log[1].in_isr);
  TEST_ASSERT(r.dev.log[2].step == Step::UNMASK);
  TEST_ASSERT(!r.dev.masked.load());
  TEST_ASSERT(r.rec.seen.size() == 1U && r.rec.seen[0].in_isr);
}

/// 线程持有 owner 时中断到来 / The interrupt arrives while a thread owns
void CheckInterruptWhileThreadOwns()
{
  using Step = FakeDevice::Step;
  Rig r(EPNumber::EP1, Direction::IN);
  r.Send(3U);
  r.ep.Later(3U);
  r.dev.log.clear();
  bool masked_inside = false;
  size_t log_inside = 0U;
  size_t seen_inside = 0U;
  r.dev.AsOwner(
      [&](bool)
      {
        r.dev.Irq();
        masked_inside = r.dev.masked.load();
        log_inside = r.dev.log.size();
        seen_inside = r.rec.seen.size();
      });
  // 中断只屏蔽、发布并返回 / The interrupt only masked, published and returned
  TEST_ASSERT(masked_inside);
  TEST_ASSERT(log_inside == 2U);
  TEST_ASSERT(seen_inside == 0U);
  TEST_ASSERT(r.dev.log.size() == 4U);
  TEST_ASSERT(r.dev.log[0].step == Step::HANDLER && !r.dev.log[0].in_isr);
  TEST_ASSERT(r.dev.log[1].step == Step::MASK);
  // owner 在同一轮处理硬件事件并开线 / The owner handles it in its pass and unmasks
  TEST_ASSERT(r.dev.log[2].step == Step::SERVICE && !r.dev.log[2].in_isr);
  TEST_ASSERT(r.dev.log[3].step == Step::UNMASK);
  TEST_ASSERT(!r.dev.masked.load());
  TEST_ASSERT(r.rec.seen.size() == 1U && !r.rec.seen[0].in_isr);
}

/// 线程请求 owner：无挂起函数就地运行，有则挂起中断 / A thread requests the owner: in
/// place without the pend function, through the interrupt with it
void CheckThreadEntry(bool can_pend)
{
  using Step = FakeDevice::Step;
  Rig r(EPNumber::EP1, Direction::IN, Type::BULK, can_pend);
  r.dev.Request(FakeDevice::NOTE_EVENT, false);
  if (!can_pend)
  {
    TEST_ASSERT(r.dev.log.size() == 1U);
    TEST_ASSERT(r.dev.log[0].step == Step::HANDLER && !r.dev.log[0].in_isr);
  }
  else
  {
    // 先发布后挂起：中断里的 owner 看到了事件 / Published before pending: the owner in
    // the interrupt saw the event
    TEST_ASSERT(r.dev.log.size() == 5U);
    TEST_ASSERT(r.dev.log[0].step == Step::PEND);
    TEST_ASSERT(r.dev.log[1].step == Step::MASK);
    TEST_ASSERT(r.dev.log[2].step == Step::SERVICE && r.dev.log[2].in_isr);
    TEST_ASSERT(r.dev.log[3].step == Step::UNMASK);
    TEST_ASSERT(r.dev.log[4].step == Step::HANDLER && r.dev.log[4].in_isr);
  }
  r.Send(6U);
  TEST_ASSERT(r.ep.Count(Hw::START) == 1U);
  TEST_ASSERT(r.ep.Start(0U).in_isr == can_pend);
  r.Complete(6U);
  TEST_ASSERT(r.rec.seen.size() == 1U && r.rec.seen[0].in_isr);
  TEST_ASSERT(!r.dev.masked.load());
}

/// 登记 / Registration
void CheckRegister()
{
  Memory mem;
  FakeDevice dev(false);
  FakeEndpoint both(EPNumber::EP1, Direction::BOTH, {mem.bytes, 128U});
  FakeEndpoint out(EPNumber::EP1, Direction::OUT, {mem.bytes + 128, 128U});
  FakeEndpoint in(EPNumber::EP1, Direction::IN, {mem.bytes + 256, 128U});
  FakeEndpoint in_again(EPNumber::EP1, Direction::IN, {mem.bytes + 384, 128U});
  TEST_ASSERT(dev.Register(both) == ErrorCode::OK);
  TEST_ASSERT(dev.Register(out) == ErrorCode::FULL);
  TEST_ASSERT(dev.Register(in) == ErrorCode::OK);
  TEST_ASSERT(dev.Register(in_again) == ErrorCode::FULL);
}

/// owner 的一轮按固定顺序：硬件事件、开线、设备事件、端点 / One owner pass in a fixed
/// order: hardware events, unmask, device events, endpoints
void CheckPassOrder()
{
  using Step = FakeDevice::Step;
  Rig r(EPNumber::EP1, Direction::IN);
  r.ep.on_start = [&](FakeEndpoint&, const FakeEndpoint::HwRequest&, bool in_isr)
  { r.dev.Note(Step::START, in_isr); };
  // 三种工作都在线程持有 owner 时到来，由下一轮一起处理 / All three kinds of work
  // arrive while a thread owns and the next pass handles them together
  r.dev.AsOwner(
      [&](bool)
      {
        r.Send(4U);
        r.dev.Request(FakeDevice::NOTE_EVENT, false);
        r.dev.Irq();
        r.dev.log.clear();
      });
  TEST_ASSERT(r.dev.log.size() == 4U);
  TEST_ASSERT(r.dev.log[0].step == Step::SERVICE);
  TEST_ASSERT(r.dev.log[1].step == Step::UNMASK);
  TEST_ASSERT(r.dev.log[2].step == Step::HANDLER);
  TEST_ASSERT(r.dev.log[3].step == Step::START);
  TEST_ASSERT(r.dev.handled.back() == FakeDevice::NOTE_EVENT);
}

/// 一次 Request 的位可能在两轮里出现：中断在发布之后、本调用尝试成为 owner 之前取走了它
/// / The bit of one Request may show up in two passes: the interrupt took it after the
/// publish and before this call tried to own
void CheckRepeatedDeviceBit()
{
  Rig r(EPNumber::EP1, Direction::IN);
  r.dev.handled.clear();
  r.dev.irq_at_pend = true;
  r.dev.Request(FakeDevice::NOTE_EVENT, false);
  TEST_ASSERT(r.dev.handled.size() == 2U);
  TEST_ASSERT(r.dev.handled[0] == FakeDevice::NOTE_EVENT);
  TEST_ASSERT(r.dev.handled[1] == FakeDevice::NOTE_EVENT);
}

// ---------------------------------------------------------------------------
// 多线程 / Multi-threaded
// ---------------------------------------------------------------------------

/**
 * @brief 几个线程在两槽 IN 端点上提交，另一个线程扮演控制器中断 / Several threads
 *        submit on a two-slot IN endpoint while another plays the controller interrupt
 * @param pend 设备有挂起函数：线程只发布并挂起，owner 总在中断线程里运行。
 *        The device has the pend function: threads only publish and pend, and the owner
 *        always runs in the interrupt thread.
 */
void CheckThreads(bool pend)
{
  constexpr uint32_t THREADS = 4U;
  constexpr uint32_t PER_THREAD = 3000U;
  constexpr uint32_t TOTAL = THREADS * PER_THREAD;

  struct Header
  {
    uint32_t thread;
    uint32_t seq;
  };
  struct Started
  {
    Header header;
    size_t length;
  };

  Memory mem;
  FakeDevice dev(pend);
  dev.logging = false;
  FakeEndpoint ep(EPNumber::EP1, Direction::IN, {mem.bytes, 1024U});
  dev.Add(ep);
  SetGate(dev, Gate::ALL);
  TEST_ASSERT(Configure(dev, ep, Direction::IN, Type::BULK, 64U) == ErrorCode::OK);
  // 从这里起挂起只置线，由中断线程运行 / From here on pending only raises the line and
  // the interrupt thread runs it
  std::atomic<bool> pend_line{false};
  dev.pend_line = &pend_line;

  // 以下只在 owner 中读写 / Read and written in the owner only
  std::vector<Started> started;
  std::vector<Completion> completions;
  std::vector<size_t> started_at_completion;
  started.reserve(TOTAL);
  completions.reserve(TOTAL);
  started_at_completion.reserve(TOTAL);

  std::atomic<bool> line{false};
  std::atomic<uint32_t> done{0U};
  std::atomic<uint32_t> in_flight_starts{0U};
  bool overlap = false;
  size_t starts_in_isr = 0U;  ///< 只在 owner 中写 / Written in the owner only
  ep.on_start =
      [&](FakeEndpoint& self, const FakeEndpoint::HwRequest& request, bool in_isr)
  {
    starts_in_isr += in_isr ? 1U : 0U;
    Header header;
    std::memcpy(&header, request.data, sizeof(header));
    started.push_back({header, request.length});
    overlap = overlap || in_flight_starts.fetch_add(1U) != 0U;
    self.Later(request.length);
    line.store(true, std::memory_order_release);
  };
  struct Sink
  {
    std::vector<Completion>* completions;
    std::vector<size_t>* started_at;
    const std::vector<Started>* started;
    std::atomic<uint32_t>* done;
    std::atomic<uint32_t>* in_flight;
    size_t callbacks_in_isr;
  } sink{&completions, &started_at_completion, &started, &done, &in_flight_starts, 0U};
  ep.SetOnComplete(Callback<const Completion&>::Create(
      [](bool in_isr, Sink* s, const Completion& completion)
      {
        s->callbacks_in_isr += in_isr ? 1U : 0U;
        s->in_flight->fetch_sub(1U);
        s->completions->push_back(completion);
        s->started_at->push_back(s->started->size());
        s->done->fetch_add(1U, std::memory_order_release);
      },
      &sink));

  std::thread irq(
      [&]()
      {
        const auto deadline =
            std::chrono::steady_clock::now() + std::chrono::seconds(120);
        while (done.load(std::memory_order_acquire) < TOTAL)
        {
          TEST_ASSERT(std::chrono::steady_clock::now() < deadline);
          // 硬件事件和挂起都由同一根中断线触发 / Hardware events and pending raise the
          // same interrupt line
          const bool hardware = line.exchange(false, std::memory_order_acq_rel);
          const bool pended = pend_line.exchange(false, std::memory_order_acq_rel);
          if (!dev.masked.load(std::memory_order_acquire) && (hardware || pended))
          {
            dev.Irq();
          }
          else
          {
            std::this_thread::yield();
          }
        }
      });

  std::vector<std::thread> submitters;
  for (uint32_t t = 0U; t < THREADS; ++t)
  {
    submitters.emplace_back(
        [&, t]()
        {
          for (uint32_t seq = 0U; seq < PER_THREAD; ++seq)
          {
            Slot slot;
            ErrorCode result;
            while ((result = ep.AcquireSlot(slot)) == ErrorCode::BUSY)
            {
              std::this_thread::yield();
            }
            TEST_ASSERT(result == ErrorCode::OK);
            const Header header{t, seq};
            std::memcpy(slot.data.addr_, &header, sizeof(header));
            const size_t length = sizeof(header) + (seq * 7U) % 200U;
            TEST_ASSERT(ep.Submit(slot, length, false, false) == ErrorCode::OK);
          }
        });
  }
  for (std::thread& submitter : submitters)
  {
    submitter.join();
  }
  irq.join();

  TEST_ASSERT(!overlap);
  TEST_ASSERT(started.size() == TOTAL);
  TEST_ASSERT(completions.size() == TOTAL);
  std::vector<uint32_t> next_seq(THREADS, 0U);
  for (size_t i = 0U; i < TOTAL; ++i)
  {
    // 第 i 个完成属于第 i 个启动，回调时下一个还没启动 / Completion i belongs to start
    // i, and the next had not started at its callback
    TEST_ASSERT(started_at_completion[i] == i + 1U);
    TEST_ASSERT(completions[i].result == ErrorCode::OK);
    TEST_ASSERT(completions[i].actual == started[i].length);
    const Header header = started[i].header;
    TEST_ASSERT(header.thread < THREADS);
    TEST_ASSERT(header.seq == next_seq[header.thread]);
    TEST_ASSERT(started[i].length == sizeof(Header) + (header.seq * 7U) % 200U);
    ++next_seq[header.thread];
  }
  for (uint32_t t = 0U; t < THREADS; ++t)
  {
    TEST_ASSERT(next_seq[t] == PER_THREAD);
  }
  TEST_ASSERT(!ep.nested);
  TEST_ASSERT(!dev.masked.load());
  if (pend)
  {
    // 有挂起函数时 owner 只在中断里运行 / With the pend function the owner runs only in
    // the interrupt
    TEST_ASSERT(starts_in_isr == TOTAL);
    TEST_ASSERT(sink.callbacks_in_isr == TOTAL);
  }
}

/// 可从任何线程在 owner 中运行任务的设备 / A device that runs a job in the owner from any
/// thread
class ThreadDevice final : public DeviceService
{
 public:
  /// pend 为真时线程只发布并挂起，由 Poll 扮演中断 / With pend, threads only publish
  /// and pend, and Poll plays the interrupt
  explicit ThreadDevice(bool pend) : pend_(pend)
  {
    SetDeviceHandler(Callback<uint32_t>::Create(
        [](bool in_isr, ThreadDevice* self, uint32_t events)
        {
          // 位不计次数，同一个任务的位可能出现两次 / Bits are no count; a job's bit may
          // show up twice
          std::function<void(bool)>* job =
              self->job_.exchange(nullptr, std::memory_order_acq_rel);
          if ((events & 1U) == 0U || job == nullptr)
          {
            return;
          }
          (*job)(in_isr);
          self->done_.store(true, std::memory_order_release);
        },
        this));
  }

  /// 在 owner 中运行 job 并等它做完；只由一个线程调用 / Run job in the owner and wait
  /// for it; called by one thread only
  void RunInOwner(std::function<void(bool)> job)
  {
    done_.store(false, std::memory_order_relaxed);
    job_.store(&job, std::memory_order_release);
    Request(1U, false);
    while (!done_.load(std::memory_order_acquire))
    {
      std::this_thread::yield();
    }
  }

  /// 挂起过就运行中断入口 / Run the interrupt entry if it was pended
  void Poll()
  {
    if (line_.exchange(false, std::memory_order_acq_rel))
    {
      OnControllerInterrupt();
    }
    else
    {
      std::this_thread::yield();
    }
  }

 protected:
  void ServiceHardware(bool) override {}
  void MaskInterrupt() override {}
  void UnmaskInterrupt() override {}
  bool PendInterrupt() override
  {
    if (pend_)
    {
      line_.store(true, std::memory_order_release);
    }
    return pend_;
  }

 private:
  const bool pend_;
  std::atomic<bool> line_{false};
  std::atomic<std::function<void(bool)>*> job_{nullptr};
  std::atomic<bool> done_{false};
};

/**
 * @brief 一个线程反复 Receive，另一个线程在 owner 中反复 Configure、Close / One thread
 *        keeps calling Receive while another keeps configuring and closing in the owner
 * @param pend 设备有挂起函数，owner 只在扮演中断的线程里运行。
 *        The device has the pend function; the owner runs only in the interrupt thread.
 * @note 依据：Receive 只返回 OK、BUSY、SIZE_ERR、STATE_ERR；每个被接受的请求恰好一次
 *       回调；Close 之后槽全部回到空闲。
 *       Basis: Receive returns only OK, BUSY, SIZE_ERR or STATE_ERR; every accepted
 *       request gets exactly one callback; after Close every slot is free again.
 */
void CheckReceiveAgainstClose(bool pend)
{
  constexpr uint32_t ROUNDS = 4000U;
  Memory mem;
  ThreadDevice dev(pend);
  FakeEndpoint ep(EPNumber::EP2, Direction::OUT, {mem.bytes, 1024U});
  TEST_ASSERT(dev.Register(ep) == ErrorCode::OK);

  std::atomic<bool> stop_irq{false};
  std::thread irq(
      [&]()
      {
        while (!stop_irq.load(std::memory_order_acquire))
        {
          dev.Poll();
        }
      });
  dev.RunInOwner([&](bool) { dev.SetStartGate(DeviceService::StartGate::ALL); });

  // 以下只在 owner 中读写 / Read and written in the owner only
  uint32_t starts = 0U;
  ep.on_start =
      [&](FakeEndpoint& self, const FakeEndpoint::HwRequest& request, bool in_isr)
  {
    // 一半当场完成，另一半留在途中等 Close 停止 / Half complete at once, half stay in
    // flight until Close stops them
    if ((starts++ & 1U) == 0U)
    {
      self.Report(in_isr, request.length, ErrorCode::OK);
    }
  };
  struct Count
  {
    FakeEndpoint* ep;
    uint32_t callbacks;
    uint32_t outside_isr;
  } count{&ep, 0U, 0U};
  ep.SetOnComplete(Callback<const Completion&>::Create(
      [](bool in_isr, Count* c, const Completion& completion)
      {
        ++c->callbacks;
        c->outside_isr += in_isr ? 0U : 1U;
        if (completion.slot.Valid())
        {
          TEST_ASSERT(c->ep->ReleaseSlot(completion.slot) == ErrorCode::OK);
        }
      },
      &count));

  std::atomic<bool> stop{false};
  std::atomic<uint32_t> accepted{0U};
  std::atomic<uint32_t> odd{0U};
  std::thread receiver(
      [&]()
      {
        while (!stop.load(std::memory_order_acquire))
        {
          const ErrorCode result = ep.Receive(64U, false);
          if (result == ErrorCode::OK)
          {
            accepted.fetch_add(1U, std::memory_order_relaxed);
          }
          else if (result != ErrorCode::BUSY && result != ErrorCode::STATE_ERR)
          {
            odd.fetch_add(1U, std::memory_order_relaxed);
          }
        }
      });
  for (uint32_t i = 0U; i < ROUNDS; ++i)
  {
    ErrorCode result = ErrorCode::PENDING;
    dev.RunInOwner([&](bool in_isr)
                   { result = ep.Configure({Direction::OUT, Type::BULK, 64U}, in_isr); });
    TEST_ASSERT(result == ErrorCode::OK);
    for (int k = 0; k < 20; ++k)
    {
      std::this_thread::yield();
    }
    dev.RunInOwner([&](bool in_isr) { ep.Close(in_isr); });
  }
  stop.store(true, std::memory_order_release);
  receiver.join();

  // 再进一次 owner，读它写的计数 / Enter the owner once more to read its counters
  uint32_t callbacks = 0U;
  uint32_t outside_isr = 0U;
  dev.RunInOwner(
      [&](bool)
      {
        callbacks = count.callbacks;
        outside_isr = count.outside_isr;
      });
  TEST_ASSERT(odd.load() == 0U);
  TEST_ASSERT(accepted.load() != 0U);
  TEST_ASSERT(callbacks == accepted.load());
  TEST_ASSERT(!pend || outside_isr == 0U);

  // 两个槽都回到空闲 / Both slots are free again
  ErrorCode result = ErrorCode::PENDING;
  dev.RunInOwner(
      [&](bool in_isr)
      {
        ep.on_start = nullptr;
        result = ep.Configure({Direction::OUT, Type::BULK, 64U}, in_isr);
      });
  TEST_ASSERT(result == ErrorCode::OK);
  TEST_ASSERT(ep.Receive(64U, false) == ErrorCode::OK);
  TEST_ASSERT(ep.Receive(64U, false) == ErrorCode::OK);
  TEST_ASSERT(ep.Receive(64U, false) == ErrorCode::BUSY);
  stop_irq.store(true, std::memory_order_release);
  irq.join();
}

// ---------------------------------------------------------------------------
// 不等待的停止 / Stops that do not wait
// ---------------------------------------------------------------------------

#ifdef LIBXR_DEBUG_BUILD
/// 致命错误回调在子进程中的退出码 / Exit code of the fatal callback in the child
constexpr int EP_FATAL_EXIT = 44;

/// 在子进程中运行 func，返回是否以产品断言结束 / Run func in a child; return whether
/// it ended on a product assertion
template <typename Func>
bool EndsFatally(Func&& func)
{
  std::fflush(nullptr);
  const pid_t child = fork();
  TEST_ASSERT(child >= 0);
  if (child == 0)
  {
    auto cb = LibXR::Assert::FatalCallback::Create(
        [](bool, int code, const char*, uint32_t) { _exit(code); }, EP_FATAL_EXIT);
    LibXR::Assert::RegisterFatalErrorCallback(cb);
    func();
    _exit(0);
  }
  int status = 0;
  TEST_ASSERT(waitpid(child, &status, 0) == child);
  TEST_ASSERT(WIFEXITED(status));
  TEST_ASSERT(WEXITSTATUS(status) == 0 || WEXITSTATUS(status) == EP_FATAL_EXIT);
  return WEXITSTATUS(status) == EP_FATAL_EXIT;
}
#endif

/// 一个已登记、已配置的端点，可选硬件是否直接读写槽 / One registered and configured
/// endpoint, with or without direct slot access
struct Bench
{
  explicit Bench(bool direct, Direction direction = Direction::IN, Type type = Type::BULK,
                 uint8_t slots = 0U, size_t memory = 1024U, bool can_pend = false)
      : dev(can_pend), ep(EPNumber::EP1, direction, {mem.bytes, memory}, slots), rec(ep)
  {
    ep.limits.direct_memory = direct;
    dev.Add(ep);
    SetGate(dev, Gate::ALL);
    TEST_ASSERT(Configure(dev, ep, direction, type, 64U) == ErrorCode::OK);
    Clear();
  }

  void Clear()
  {
    ep.calls.clear();
    rec.seen.clear();
    dev.handled.clear();
    dev.log.clear();
  }

  /// 借一个槽并提交 / Borrow a slot and submit it
  void Send(size_t length, bool zlp = false)
  {
    Slot slot;
    TEST_ASSERT(ep.AcquireSlot(slot) == ErrorCode::OK);
    TEST_ASSERT(ep.Submit(slot, length, zlp, false) == ErrorCode::OK);
  }

  /// 下一次 StopHardware 返回 PENDING / The next StopHardware returns PENDING
  void Pend() { ep.stop_outcomes.push_back({ErrorCode::PENDING, 0U}); }

  /// 后端在下一次硬件事件处理里报告 / The backend reports at the next hardware event
  void Report(ErrorCode result = ErrorCode::ABORTED, size_t actual = 0U)
  {
    ep.Later(actual, result);
    dev.Irq();
  }

  /// 能借到的槽数，借到的随即交回 / Slots that can be lent, handed back at once
  size_t FreeSlots()
  {
    Slot a;
    Slot b;
    size_t count = 0U;
    if (ep.AcquireSlot(a) == ErrorCode::OK)
    {
      ++count;
      if (ep.AcquireSlot(b) == ErrorCode::OK)
      {
        ++count;
        TEST_ASSERT(ep.ReleaseSlot(b) == ErrorCode::OK);
      }
      TEST_ASSERT(ep.ReleaseSlot(a) == ErrorCode::OK);
    }
    return count;
  }

  /// 控制器复位并关闭所有端点 / Controller reset and close every endpoint
  void ResetAll()
  {
    dev.AsOwner([this](bool in_isr) { dev.ResetAndCloseAll(in_isr); });
  }

  /// 回调恰为 results，依次 / The callbacks are exactly results, in order
  bool SeenAre(std::initializer_list<ErrorCode> results) const
  {
    if (rec.seen.size() != results.size())
    {
      return false;
    }
    size_t i = 0U;
    for (ErrorCode result : results)
    {
      if (rec.seen[i++].completion.result != result)
      {
        return false;
      }
    }
    return true;
  }

  Memory mem;
  FakeDevice dev;
  FakeEndpoint ep;
  Recorder rec;
};

/// 状态表的行 / Rows of the state table
enum class Row : uint8_t
{
  D0,  ///< DISABLED，没有停止 / DISABLED, no stop
  I0,  ///< IDLE，没有在途 / IDLE, nothing in flight
  I1,  ///< IDLE，一个在途 / IDLE, one in flight
  S0,  ///< STALLED，没有在途 / STALLED, nothing in flight
  DP,  ///< DISABLED，停止挂起，记下 CloseHardware / DISABLED, stop pending, close due
  IP,  ///< IDLE，停止挂起 / IDLE, stop pending
  SP,  ///< STALLED，停止挂起 / STALLED, stop pending
  DF,  ///< DISABLED，停止失败 / DISABLED, stop failed
  IF,  ///< IDLE，停止失败 / IDLE, stop failed
  SF   ///< STALLED，停止失败 / STALLED, stop failed
};

/// 新建一个处于 row 的端点，记录已清空 / A fresh endpoint in row, records cleared
std::unique_ptr<Bench> At(Row row, bool direct)
{
  auto bench = std::make_unique<Bench>(direct);
  Bench& b = *bench;
  switch (row)
  {
    case Row::D0:
      Close(b.dev, b.ep);
      break;
    case Row::I0:
      break;
    case Row::I1:
      b.Send(10U);
      break;
    case Row::S0:
      TEST_ASSERT(Stall(b.dev, b.ep) == ErrorCode::OK);
      break;
    case Row::DP:
    case Row::DF:
      b.Send(10U);
      b.Pend();
      Close(b.dev, b.ep);
      break;
    case Row::IP:
    case Row::IF:
      b.Send(10U);
      b.Pend();
      Flush(b.dev, b.ep);
      break;
    case Row::SP:
    case Row::SF:
      b.Send(10U);
      b.Pend();
      TEST_ASSERT(Stall(b.dev, b.ep) == ErrorCode::OK);
      break;
  }
  if (row == Row::DF || row == Row::IF || row == Row::SF)
  {
    b.Report(ErrorCode::TIMEOUT);
    TEST_ASSERT(b.dev.StopFailures() == 1U);
  }
  b.Clear();
  return bench;
}

/// D0、I0、S0 行 / Rows D0, I0 and S0
void CheckRowsWithoutStop(bool direct)
{
  // D0
  {
    auto b = At(Row::D0, direct);
    TEST_ASSERT(Configure(b->dev, b->ep, Direction::IN, Type::BULK, 64U) ==
                ErrorCode::OK);
    TEST_ASSERT(b->ep.CallsAre({Hw::CONFIGURE}) && b->ep.GetState() == State::IDLE);
    TEST_ASSERT(b->FreeSlots() == 2U);
  }
  {
    auto b = At(Row::D0, direct);
    Close(b->dev, b->ep);
    Flush(b->dev, b->ep);
    TEST_ASSERT(Stall(b->dev, b->ep) == ErrorCode::STATE_ERR);
    TEST_ASSERT(ClearStall(b->dev, b->ep) == ErrorCode::STATE_ERR);
    Slot slot;
    TEST_ASSERT(b->ep.AcquireSlot(slot) == ErrorCode::STATE_ERR);
    b->Report(ErrorCode::OK, 5U);
    b->Report(ErrorCode::ABORTED);
    b->ResetAll();
    TEST_ASSERT(b->ep.calls.empty() && b->rec.seen.empty());
    TEST_ASSERT(b->ep.GetState() == State::DISABLED);
  }

  // I0
  {
    auto b = At(Row::I0, direct);
    TEST_ASSERT(Configure(b->dev, b->ep, Direction::IN, Type::BULK, 64U) ==
                ErrorCode::STATE_ERR);
    Flush(b->dev, b->ep);
    b->Report(ErrorCode::OK, 5U);
    b->Report(ErrorCode::ABORTED);
    TEST_ASSERT(b->ep.calls.empty() && b->rec.seen.empty());
    TEST_ASSERT(ClearStall(b->dev, b->ep) == ErrorCode::OK);
    TEST_ASSERT(b->ep.CallsAre({Hw::CLEAR_STALL}) && b->ep.GetState() == State::IDLE);
    TEST_ASSERT(Stall(b->dev, b->ep) == ErrorCode::OK);
    TEST_ASSERT(b->ep.CallsAre({Hw::CLEAR_STALL, Hw::STALL}) && b->ep.IsStalled());
  }
  {
    auto b = At(Row::I0, direct);
    b->Send(5U);
    TEST_ASSERT(b->ep.CallsAre({Hw::START}));
  }
  // 排队请求在 Close、Flush、复位时各交付一次 ABORTED / A queued request gets one
  // ABORTED from Close, Flush and the reset
  for (int how = 0; how < 3; ++how)
  {
    auto b = At(Row::I0, direct);
    SetGate(b->dev, Gate::CLOSED);
    b->Send(5U);
    if (how == 0)
    {
      Close(b->dev, b->ep);
    }
    else if (how == 1)
    {
      Flush(b->dev, b->ep);
    }
    else
    {
      b->ResetAll();
    }
    TEST_ASSERT(b->SeenAre({ErrorCode::ABORTED}) && b->rec.At(0U).actual == 0U);
    if (how == 1)
    {
      TEST_ASSERT(b->ep.calls.empty() && b->ep.GetState() == State::IDLE);
    }
    else
    {
      TEST_ASSERT(b->ep.CallsAre({Hw::CLOSE}));
      TEST_ASSERT(b->ep.GetState() == State::DISABLED);
    }
  }

  // S0：排队一个请求 / S0 with one queued request
  {
    auto b = At(Row::S0, direct);
    b->Send(7U);
    TEST_ASSERT(b->ep.calls.empty());
    TEST_ASSERT(Configure(b->dev, b->ep, Direction::IN, Type::BULK, 64U) ==
                ErrorCode::STATE_ERR);
    TEST_ASSERT(Stall(b->dev, b->ep) == ErrorCode::OK);
    b->Report(ErrorCode::OK, 5U);
    b->Report(ErrorCode::ABORTED);
    TEST_ASSERT(b->ep.calls.empty() && b->rec.seen.empty());
    TEST_ASSERT(ClearStall(b->dev, b->ep) == ErrorCode::OK);
    TEST_ASSERT(b->ep.CallsAre({Hw::CLEAR_STALL, Hw::START}));
    TEST_ASSERT(b->ep.GetState() == State::IDLE);
  }
  for (int how = 0; how < 3; ++how)
  {
    auto b = At(Row::S0, direct);
    b->Send(7U);
    if (how == 0)
    {
      Close(b->dev, b->ep);
    }
    else if (how == 1)
    {
      Flush(b->dev, b->ep);
    }
    else
    {
      b->ResetAll();
    }
    TEST_ASSERT(b->SeenAre({ErrorCode::ABORTED}));
    if (how == 1)
    {
      TEST_ASSERT(b->ep.calls.empty() && b->ep.IsStalled());
    }
    else
    {
      TEST_ASSERT(b->ep.CallsAre({Hw::CLOSE}));
      TEST_ASSERT(b->ep.GetState() == State::DISABLED);
    }
  }
}

/// I1 行：停止当场给出结果、挂起、结尾零长度包待发 / Row I1: the stop gives the outcome
/// at once, pends, or finds the closing packet due
void CheckRowInFlight(bool direct)
{
  {
    auto b = At(Row::I1, direct);
    TEST_ASSERT(Configure(b->dev, b->ep, Direction::IN, Type::BULK, 64U) ==
                ErrorCode::STATE_ERR);
    // 在途时 ClearStall 照样当场调用 ClearStallHardware / ClearStall with a request in
    // flight still calls ClearStallHardware at once
    TEST_ASSERT(ClearStall(b->dev, b->ep) == ErrorCode::OK);
    TEST_ASSERT(b->ep.CallsAre({Hw::CLEAR_STALL}) && b->rec.seen.empty());
    b->Send(20U);
    TEST_ASSERT(b->ep.CallsAre({Hw::CLEAR_STALL}));
    b->Report(ErrorCode::OK, 10U);
    TEST_ASSERT(b->SeenAre({ErrorCode::OK}) && b->rec.At(0U).actual == 10U);
    TEST_ASSERT(b->ep.CallsAre({Hw::CLEAR_STALL, Hw::START}));
    TEST_ASSERT(b->ep.Start(0U).length == 20U);
  }
  {
    // 后端报告的 ABORTED（控制端点撤回的装入）按普通完成交付 / A reported ABORTED
    // (the retracted load of a control endpoint) is delivered as a plain completion
    auto b = At(Row::I1, direct);
    b->Report(ErrorCode::ABORTED, 3U);
    TEST_ASSERT(b->SeenAre({ErrorCode::ABORTED}) && b->rec.At(0U).actual == 3U);
    TEST_ASSERT(b->FreeSlots() == 2U);
  }
  for (const Ender how : {Ender::CLOSE, Ender::FLUSH, Ender::STALL})
  {
    const std::vector<Hw> step =
        (how == Ender::CLOSE)   ? std::vector<Hw>{Hw::STOP, Hw::CLOSE}
        : (how == Ender::STALL) ? std::vector<Hw>{Hw::STOP, Hw::STALL}
                                : std::vector<Hw>{Hw::STOP};
    const State after = (how == Ender::CLOSE)   ? State::DISABLED
                        : (how == Ender::STALL) ? State::STALLED
                                                : State::IDLE;
    // 当场给出结果：硬件已做完的 IN 为 OK / Outcome at once: OK for an IN the hardware
    // had finished
    {
      auto b = At(Row::I1, direct);
      b->Send(20U);
      b->ep.stop_outcomes = {{ErrorCode::OK, 10U}};
      End(b->dev, b->ep, how);
      TEST_ASSERT(b->ep.CallsAre(step) && b->ep.GetState() == after);
      TEST_ASSERT(b->SeenAre({ErrorCode::OK, ErrorCode::ABORTED}));
      TEST_ASSERT(b->rec.At(0U).actual == 10U && b->rec.At(1U).actual == 0U);
      TEST_ASSERT(b->rec.seen[0].hw_calls == step.size());
      if (how != Ender::CLOSE)
      {
        TEST_ASSERT(b->FreeSlots() == 2U);
      }
    }
    // 挂起：当场 ABORTED，硬件动作推迟到报告 / Pending: ABORTED at once, the hardware
    // step deferred to the report
    {
      auto b = At(Row::I1, direct);
      b->Send(20U);
      b->Pend();
      End(b->dev, b->ep, how);
      TEST_ASSERT(b->ep.CallsAre({Hw::STOP}) && b->ep.GetState() == after);
      TEST_ASSERT(b->SeenAre({ErrorCode::ABORTED, ErrorCode::ABORTED}));
      TEST_ASSERT(b->rec.At(0U).actual == 0U && !b->rec.At(0U).slot.Valid());
      if (how != Ender::CLOSE)
      {
        TEST_ASSERT(b->FreeSlots() == (direct ? 1U : 2U));
      }
      b->Report(ErrorCode::OK, 10U);
      TEST_ASSERT(b->ep.CallsAre(step) && b->rec.seen.size() == 2U);
      if (how != Ender::CLOSE)
      {
        TEST_ASSERT(b->FreeSlots() == 2U);
      }
    }
    // 数据部分已完成、结尾零长度包待发：硬件上没有东西 / Data part done, closing packet
    // due: nothing is on the hardware
    {
      auto b = At(Row::I0, direct);
      b->Send(128U, true);
      SetGate(b->dev, Gate::CLOSED);
      b->Report(ErrorCode::OK, 128U);
      TEST_ASSERT(b->rec.seen.empty());
      b->Clear();
      End(b->dev, b->ep, how);
      const std::vector<Hw> due = (how == Ender::CLOSE)   ? std::vector<Hw>{Hw::CLOSE}
                                  : (how == Ender::STALL) ? std::vector<Hw>{Hw::STALL}
                                                          : std::vector<Hw>{};
      TEST_ASSERT(b->ep.CallsAre(due));
      TEST_ASSERT(b->SeenAre({ErrorCode::ABORTED}) && b->rec.At(0U).actual == 128U);
    }
  }
  {
    auto b = At(Row::I1, direct);
    b->Report(ErrorCode::OK, 10U);
    TEST_ASSERT(b->SeenAre({ErrorCode::OK}) && b->ep.calls.empty());
  }
  {
    auto b = At(Row::I1, direct);
    b->Send(20U);
    b->ResetAll();
    TEST_ASSERT(b->ep.CallsAre({Hw::STOP, Hw::CLOSE}));
    TEST_ASSERT(b->SeenAre({ErrorCode::ABORTED, ErrorCode::ABORTED}));
    TEST_ASSERT(b->ep.GetState() == State::DISABLED && b->dev.resets == 1U);
  }
}

/// DP、IP、SP 行 / Rows DP, IP and SP
void CheckRowsWithPendingStop(bool direct)
{
  // DP
  {
    auto b = At(Row::DP, direct);
    TEST_ASSERT(Configure(b->dev, b->ep, Direction::IN, Type::BULK, 64U) ==
                ErrorCode::OK);
    TEST_ASSERT(b->ep.calls.empty() && b->ep.GetState() == State::IDLE);
    TEST_ASSERT(b->FreeSlots() == (direct ? 1U : 2U));
    b->Report();
    TEST_ASSERT(b->ep.CallsAre({Hw::CLOSE, Hw::CONFIGURE}) && b->rec.seen.empty());
    TEST_ASSERT(b->FreeSlots() == 2U);
  }
  {
    auto b = At(Row::DP, direct);
    Close(b->dev, b->ep);
    Flush(b->dev, b->ep);
    TEST_ASSERT(Stall(b->dev, b->ep) == ErrorCode::STATE_ERR);
    TEST_ASSERT(ClearStall(b->dev, b->ep) == ErrorCode::STATE_ERR);
    Slot slot;
    TEST_ASSERT(b->ep.AcquireSlot(slot) == ErrorCode::STATE_ERR);
    TEST_ASSERT(b->ep.calls.empty());
    // 停止期间完成的请求并入报告，不回调 / A request finished during the stop merges
    // into the report, no callback
    b->Report(ErrorCode::OK, 10U);
    TEST_ASSERT(b->ep.CallsAre({Hw::CLOSE}) && b->rec.seen.empty());
    TEST_ASSERT(b->ep.GetState() == State::DISABLED);
    TEST_ASSERT(Configure(b->dev, b->ep, Direction::IN, Type::BULK, 64U) ==
                ErrorCode::OK);
    TEST_ASSERT(b->ep.CallsAre({Hw::CLOSE, Hw::CONFIGURE}));
  }
  {
    auto b = At(Row::DP, direct);
    b->Report(ErrorCode::TIMEOUT);
    TEST_ASSERT(b->ep.calls.empty() && b->dev.StopFailures() == 1U);
    TEST_ASSERT(Configure(b->dev, b->ep, Direction::IN, Type::BULK, 64U) ==
                ErrorCode::STATE_ERR);
  }
  {
    auto b = At(Row::DP, direct);
    b->ResetAll();
    TEST_ASSERT(b->ep.CallsAre({Hw::STOP, Hw::CLOSE}) && b->rec.seen.empty());
    TEST_ASSERT(Configure(b->dev, b->ep, Direction::IN, Type::BULK, 64U) ==
                ErrorCode::OK);
    TEST_ASSERT(b->FreeSlots() == 2U);
    b->Report();
    TEST_ASSERT(b->ep.CallsAre({Hw::STOP, Hw::CLOSE, Hw::CONFIGURE}));
  }

  // IP 与 SP / IP and SP
  for (const Row row : {Row::IP, Row::SP})
  {
    const bool stalled = row == Row::SP;
    {
      auto b = At(row, direct);
      TEST_ASSERT(Configure(b->dev, b->ep, Direction::IN, Type::BULK, 64U) ==
                  ErrorCode::STATE_ERR);
      Close(b->dev, b->ep);
      TEST_ASSERT(b->ep.calls.empty() && b->ep.GetState() == State::DISABLED);
      b->Report();
      TEST_ASSERT(b->ep.CallsAre({Hw::CLOSE}) && b->rec.seen.empty());
    }
    {
      auto b = At(row, direct);
      b->Send(20U);
      Flush(b->dev, b->ep);
      TEST_ASSERT(b->ep.calls.empty() && b->SeenAre({ErrorCode::ABORTED}));
      b->Report();
      TEST_ASSERT(stalled ? b->ep.CallsAre({Hw::STALL}) : b->ep.calls.empty());
    }
    {
      auto b = At(row, direct);
      TEST_ASSERT(Stall(b->dev, b->ep) == ErrorCode::OK);
      TEST_ASSERT(b->ep.calls.empty() && b->ep.IsStalled());
      b->Report();
      TEST_ASSERT(b->ep.CallsAre({Hw::STALL}));
    }
    {
      auto b = At(row, direct);
      TEST_ASSERT(ClearStall(b->dev, b->ep) == ErrorCode::OK);
      TEST_ASSERT(b->ep.calls.empty() && b->ep.GetState() == State::IDLE);
      b->Report();
      TEST_ASSERT(b->ep.CallsAre({Hw::CLEAR_STALL}));
    }
    {
      // 排队；硬件直接读写槽时被停的槽仍占着 / Queued; with direct access the stopped
      // slot is still taken
      auto b = At(row, direct);
      b->Send(20U);
      TEST_ASSERT(b->ep.calls.empty());
      TEST_ASSERT(b->FreeSlots() == (direct ? 0U : 1U));
      b->Report(ErrorCode::OK, 10U);
      TEST_ASSERT(b->rec.seen.empty());
      TEST_ASSERT(stalled ? b->ep.CallsAre({Hw::STALL}) : b->ep.CallsAre({Hw::START}));
      TEST_ASSERT(b->ep.Count(Hw::START) == 0U || b->ep.Start(0U).length == 20U);
    }
    {
      auto b = At(row, direct);
      b->Send(20U);
      b->Report(ErrorCode::TIMEOUT);
      TEST_ASSERT(b->ep.calls.empty() && b->dev.StopFailures() == 1U);
      TEST_ASSERT(b->ep.IsStalled() == stalled);
    }
    {
      auto b = At(row, direct);
      b->Send(20U);
      b->ResetAll();
      TEST_ASSERT(b->ep.CallsAre({Hw::STOP, Hw::CLOSE}));
      TEST_ASSERT(b->SeenAre({ErrorCode::ABORTED}));
      TEST_ASSERT(b->ep.GetState() == State::DISABLED);
    }
  }
}

/// DF、IF、SF 行 / Rows DF, IF and SF
void CheckRowsWithFailedStop(bool direct)
{
  {
    auto b = At(Row::DF, direct);
    TEST_ASSERT(Configure(b->dev, b->ep, Direction::IN, Type::BULK, 64U) ==
                ErrorCode::STATE_ERR);
    Close(b->dev, b->ep);
    Flush(b->dev, b->ep);
    TEST_ASSERT(Stall(b->dev, b->ep) == ErrorCode::STATE_ERR);
    TEST_ASSERT(ClearStall(b->dev, b->ep) == ErrorCode::STATE_ERR);
    Slot slot;
    TEST_ASSERT(b->ep.AcquireSlot(slot) == ErrorCode::STATE_ERR);
    TEST_ASSERT(b->ep.calls.empty() && b->rec.seen.empty());
    b->ResetAll();
    TEST_ASSERT(b->ep.CallsAre({Hw::STOP, Hw::CLOSE}) && b->rec.seen.empty());
    TEST_ASSERT(Configure(b->dev, b->ep, Direction::IN, Type::BULK, 64U) ==
                ErrorCode::OK);
    TEST_ASSERT(b->FreeSlots() == 2U);
  }
  for (const Row row : {Row::IF, Row::SF})
  {
    {
      auto b = At(row, direct);
      TEST_ASSERT(b->ep.IsStalled() == (row == Row::SF));
      TEST_ASSERT(Configure(b->dev, b->ep, Direction::IN, Type::BULK, 64U) ==
                  ErrorCode::STATE_ERR);
      b->Send(20U);
      TEST_ASSERT(b->FreeSlots() == (direct ? 0U : 1U));
      Flush(b->dev, b->ep);
      TEST_ASSERT(b->SeenAre({ErrorCode::ABORTED}));
      TEST_ASSERT(Stall(b->dev, b->ep) == ErrorCode::OK && b->ep.IsStalled());
      TEST_ASSERT(Stall(b->dev, b->ep) == ErrorCode::OK);
      TEST_ASSERT(ClearStall(b->dev, b->ep) == ErrorCode::OK);
      TEST_ASSERT(b->ep.GetState() == State::IDLE);
      b->Send(30U);
      Close(b->dev, b->ep);
      TEST_ASSERT(b->SeenAre({ErrorCode::ABORTED, ErrorCode::ABORTED}));
      TEST_ASSERT(b->ep.GetState() == State::DISABLED && b->ep.calls.empty());
      TEST_ASSERT(Configure(b->dev, b->ep, Direction::IN, Type::BULK, 64U) ==
                  ErrorCode::STATE_ERR);
    }
    {
      auto b = At(row, direct);
      b->Send(20U);
      b->ResetAll();
      TEST_ASSERT(b->ep.CallsAre({Hw::STOP, Hw::CLOSE}));
      TEST_ASSERT(b->SeenAre({ErrorCode::ABORTED}));
      TEST_ASSERT(Configure(b->dev, b->ep, Direction::IN, Type::BULK, 64U) ==
                  ErrorCode::OK);
      TEST_ASSERT(b->FreeSlots() == 2U);
    }
  }
}

/// 推迟动作的链：后端调用顺序与回调次数 / Chains of deferred steps: backend call order
/// and callback count
void CheckDeferredChains(bool direct)
{
  using Steps = std::function<void(Bench&)>;
  struct Chain
  {
    Steps steps;
    std::vector<Hw> after;  ///< 报告之后的全部调用 / All calls after the report
    State state;
  };
  auto configure = [](Bench& b)
  {
    TEST_ASSERT(Configure(b.dev, b.ep, Direction::IN, Type::BULK, 64U) == ErrorCode::OK);
  };
  auto stall = [](Bench& b) { TEST_ASSERT(Stall(b.dev, b.ep) == ErrorCode::OK); };
  auto clear = [](Bench& b) { TEST_ASSERT(ClearStall(b.dev, b.ep) == ErrorCode::OK); };
  auto close = [](Bench& b) { Close(b.dev, b.ep); };
  auto flush = [](Bench& b) { Flush(b.dev, b.ep); };
  const Chain chains[] = {
      // Close→Configure→Stall→报告 / Close→Configure→Stall→report
      {[&](Bench& b)
       {
         close(b);
         configure(b);
         stall(b);
       },
       {Hw::STOP, Hw::CLOSE, Hw::CONFIGURE, Hw::STALL},
       State::STALLED},
      {[&](Bench& b)
       {
         close(b);
         configure(b);
         stall(b);
         clear(b);
       },
       {Hw::STOP, Hw::CLOSE, Hw::CONFIGURE, Hw::CLEAR_STALL},
       State::IDLE},
      {[&](Bench& b)
       {
         stall(b);
         clear(b);
         stall(b);
       },
       {Hw::STOP, Hw::STALL},
       State::STALLED},
      // 推迟的 ClearStall 被 Close 作废 / A deferred ClearStall is dropped by Close
      {[&](Bench& b)
       {
         flush(b);
         clear(b);
         close(b);
       },
       {Hw::STOP, Hw::CLOSE},
       State::DISABLED},
      // 作废之后重新绑定，新绑定上不再清除 STALL / Bound again after that, the new
      // binding gets no ClearStall
      {[&](Bench& b)
       {
         flush(b);
         clear(b);
         close(b);
         configure(b);
       },
       {Hw::STOP, Hw::CLOSE, Hw::CONFIGURE},
       State::IDLE},
      {[&](Bench& b)
       {
         stall(b);
         close(b);
         configure(b);
       },
       {Hw::STOP, Hw::CLOSE, Hw::CONFIGURE},
       State::IDLE},
      {[&](Bench& b)
       {
         flush(b);
         flush(b);
       },
       {Hw::STOP},
       State::IDLE},
      {[&](Bench& b)
       {
         close(b);
         configure(b);
         close(b);
         configure(b);
       },
       {Hw::STOP, Hw::CLOSE, Hw::CONFIGURE},
       State::IDLE},
      // 新绑定提交的请求在推迟动作之后启动 / A request of the new binding starts after
      // the deferred steps
      {[&](Bench& b)
       {
         close(b);
         configure(b);
         b.Send(30U);
       },
       {Hw::STOP, Hw::CLOSE, Hw::CONFIGURE, Hw::START},
       State::IDLE},
      {[&](Bench& b)
       {
         close(b);
         configure(b);
         b.Send(30U);
         flush(b);
       },
       {Hw::STOP, Hw::CLOSE, Hw::CONFIGURE},
       State::IDLE},
  };
  for (const Chain& chain : chains)
  {
    Bench b(direct);
    b.Send(10U);
    b.Pend();
    chain.steps(b);
    // 每个请求在引起它结束的调用返回前回调 / Every request is called back before the
    // call that ends it returns
    const size_t seen = b.rec.seen.size();
    TEST_ASSERT(seen >= 1U && b.rec.At(0U).result == ErrorCode::ABORTED);
    TEST_ASSERT(b.ep.Count(Hw::STOP) == 1U && b.ep.calls.size() == 2U);
    b.Report();
    TEST_ASSERT(b.rec.seen.size() == seen);
    TEST_ASSERT(b.ep.calls.size() == chain.after.size() + 1U);
    for (size_t i = 0U; i < chain.after.size(); ++i)
    {
      TEST_ASSERT(b.ep.calls[i + 1U].function == chain.after[i]);
    }
    TEST_ASSERT(b.ep.GetState() == chain.state);
  }

  // 被停请求的回调里重新绑定并提交：回调看到停止仍挂起 / Rebinding and submitting from
  // the callback of the stopped request: the callback sees the stop pending
  Bench b(direct);
  b.Send(10U);
  b.Pend();
  ErrorCode inner = ErrorCode::PENDING;
  b.rec.hook = [&](const Completion&, bool in_isr)
  {
    inner = b.ep.Configure({Direction::IN, Type::INTR, 16U}, in_isr);
    b.Send(4U);
  };
  Close(b.dev, b.ep);
  b.rec.hook = nullptr;
  TEST_ASSERT(inner == ErrorCode::OK);
  TEST_ASSERT(b.ep.CallsAre({Hw::START, Hw::STOP}));
  b.Report();
  TEST_ASSERT(b.ep.CallsAre({Hw::START, Hw::STOP, Hw::CLOSE, Hw::CONFIGURE, Hw::START}));
  TEST_ASSERT(b.ep.configured.type == Type::INTR && b.ep.Start(1U).length == 4U);
  TEST_ASSERT(b.rec.seen.size() == 1U);

  // 推迟的动作只执行一次：同一端点下一次停止结束时不重复
  // Deferred steps run once: the end of the next stop on the endpoint repeats none
  Bench again(direct);
  again.Send(10U);
  again.Pend();
  Close(again.dev, again.ep);
  configure(again);
  again.Report();
  TEST_ASSERT(again.ep.CallsAre({Hw::START, Hw::STOP, Hw::CLOSE, Hw::CONFIGURE}));
  again.Send(20U);
  again.Pend();
  Flush(again.dev, again.ep);
  again.Report();
  TEST_ASSERT(again.ep.CallsAre(
      {Hw::START, Hw::STOP, Hw::CLOSE, Hw::CONFIGURE, Hw::START, Hw::STOP}));
  TEST_ASSERT(again.ep.GetState() == State::IDLE);
}

/// 一个槽的端点在旧停止挂起时重新绑定 / A one-slot endpoint bound again while the old
/// stop is pending
void CheckOneSlotRebind(bool direct)
{
  Bench b(direct, Direction::OUT, Type::CONTROL, 1U, 64U);
  TEST_ASSERT(b.ep.Receive(64U, false) == ErrorCode::OK);
  b.Pend();
  Close(b.dev, b.ep);
  TEST_ASSERT(b.SeenAre({ErrorCode::ABORTED}));
  TEST_ASSERT(Configure(b.dev, b.ep, Direction::OUT, Type::CONTROL, 64U) ==
              ErrorCode::OK);
  if (direct)
  {
    // 被停的槽到停止结束才能再借 / The stopped slot is lent only after the stop
    TEST_ASSERT(b.ep.Receive(0U, false) == ErrorCode::BUSY);
    b.Report();
    TEST_ASSERT(b.ep.CallsAre({Hw::START, Hw::STOP, Hw::CLOSE, Hw::CONFIGURE}));
    TEST_ASSERT(b.ep.Receive(0U, false) == ErrorCode::OK);
  }
  else
  {
    TEST_ASSERT(b.ep.Receive(0U, false) == ErrorCode::OK);
    TEST_ASSERT(b.ep.CallsAre({Hw::START, Hw::STOP}));
    b.Report();
  }
  TEST_ASSERT(b.ep.CallsAre({Hw::START, Hw::STOP, Hw::CLOSE, Hw::CONFIGURE, Hw::START}));
  TEST_ASSERT(b.ep.Start(1U).length == 0U && b.rec.seen.size() == 1U);

  if (!direct)
  {
    // 不直接读写槽的数据端点一个槽也够 / One slot suffices for a data endpoint without
    // direct access
    Bench d(false, Direction::OUT, Type::BULK, 1U, 512U);
    TEST_ASSERT(d.ep.Receive(64U, false) == ErrorCode::OK);
    d.Pend();
    Close(d.dev, d.ep);
    TEST_ASSERT(Configure(d.dev, d.ep, Direction::OUT, Type::BULK, 64U) == ErrorCode::OK);
    TEST_ASSERT(d.ep.Receive(64U, false) == ErrorCode::OK);
    d.Report();
    TEST_ASSERT(
        d.ep.CallsAre({Hw::START, Hw::STOP, Hw::CLOSE, Hw::CONFIGURE, Hw::START}));
  }
}

/// GetConfig 是硬件当前的配置，停止挂起时可与新绑定不同 / GetConfig is what the
/// hardware has; while a stop is pending it may differ from the new binding
void CheckHardwareConfig(bool direct)
{
  Bench b(direct);
  b.Send(10U);
  b.Pend();
  Close(b.dev, b.ep);
  TEST_ASSERT(Configure(b.dev, b.ep, Direction::IN, Type::INTR, 16U) == ErrorCode::OK);
  TEST_ASSERT(b.ep.GetType() == Type::INTR && b.ep.MaxPacketSize() == 16U);
  TEST_ASSERT(b.ep.GetConfig().type == Type::BULK);
  TEST_ASSERT(b.ep.GetConfig().max_packet_size == 64U);
  b.Report();
  TEST_ASSERT(b.ep.GetConfig().type == Type::INTR);
  TEST_ASSERT(b.ep.GetConfig().max_packet_size == 16U);
}

/// 停止期间完成、停止之后的迟到报告、被停的槽只放一次 / Completed during the stop, late
/// reports after it, the stopped slot freed once
void CheckLateAfterStop(bool direct)
{
  Bench b(direct, Direction::OUT);
  TEST_ASSERT(b.ep.Receive(64U, false) == ErrorCode::OK);
  const uint8_t* stopped = b.ep.Start(0U).data;
  b.Pend();
  Flush(b.dev, b.ep);
  TEST_ASSERT(b.SeenAre({ErrorCode::ABORTED}) && !b.rec.At(0U).slot.Valid());
  TEST_ASSERT(b.ep.Receive(64U, false) == ErrorCode::OK);
  // 不直接读写槽：被停的槽已放回；直接读写：仍占着 / Without direct access the stopped
  // slot is back; with it, still taken
  TEST_ASSERT(b.ep.Receive(64U, false) == (direct ? ErrorCode::BUSY : ErrorCode::OK));
  TEST_ASSERT(b.ep.Count(Hw::START) == 1U);
  // 硬件其实在停止期间收完了：报告只结束停止，不回调 / The hardware did finish during
  // the stop: the report only ends the stop, no callback
  b.Report(ErrorCode::OK, 64U);
  TEST_ASSERT(b.rec.seen.size() == 1U && b.ep.Count(Hw::START) == 2U);
  TEST_ASSERT((b.ep.Start(1U).data == stopped) == !direct);
  b.Report(ErrorCode::OK, 5U);
  TEST_ASSERT(b.SeenAre({ErrorCode::ABORTED, ErrorCode::OK}));
  TEST_ASSERT(b.rec.At(1U).slot.Valid() && b.rec.At(1U).actual == 5U);
  TEST_ASSERT(b.ep.ReleaseSlot(b.rec.At(1U).slot) == ErrorCode::OK);
  if (!direct)
  {
    b.Report(ErrorCode::OK, 6U);
    TEST_ASSERT(b.rec.seen.size() == 3U);
    TEST_ASSERT(b.ep.ReleaseSlot(b.rec.At(2U).slot) == ErrorCode::OK);
  }
  // 没有在途请求时的迟到报告被忽略 / A late report with nothing in flight is ignored
  const size_t seen = b.rec.seen.size();
  const size_t calls = b.ep.calls.size();
  b.Report(ErrorCode::OK, 64U);
  TEST_ASSERT(b.rec.seen.size() == seen && b.ep.calls.size() == calls);
  TEST_ASSERT(b.ep.Receive(64U, false) == ErrorCode::OK);
  TEST_ASSERT(b.ep.Receive(64U, false) == ErrorCode::OK);
  TEST_ASSERT(b.ep.Receive(64U, false) == ErrorCode::BUSY);
}

/// 每个被接受的请求恰好一次回调，结束它的调用返回后再无回调 / Every accepted request
/// gets exactly one callback, none after the call that ended it returns
void CheckOneCallbackPerRequest(bool direct)
{
  Bench b(direct);
  size_t accepted = 0U;
  auto send = [&](size_t length)
  {
    Slot slot;
    if (b.ep.AcquireSlot(slot) != ErrorCode::OK)
    {
      return;
    }
    TEST_ASSERT(b.ep.Submit(slot, length, false, false) == ErrorCode::OK);
    ++accepted;
  };
  for (uint32_t i = 0U; i < 60U; ++i)
  {
    send(i);
    send(i + 1U);
    const bool pend = (i % 2U) == 0U;
    if (pend)
    {
      b.Pend();
    }
    switch (i % 5U)
    {
      case 0U:
      case 1U:
        Flush(b.dev, b.ep);
        break;
      case 2U:
        TEST_ASSERT(Stall(b.dev, b.ep) == ErrorCode::OK);
        TEST_ASSERT(ClearStall(b.dev, b.ep) == ErrorCode::OK);
        break;
      default:
        Close(b.dev, b.ep);
        TEST_ASSERT(Configure(b.dev, b.ep, Direction::IN, Type::BULK, 64U) ==
                    ErrorCode::OK);
        break;
    }
    // 返回时每个请求都已回调 / On return every request has had its callback
    TEST_ASSERT(b.rec.seen.size() == accepted);
    send(i + 2U);
    if (pend)
    {
      b.Report(ErrorCode::OK, 1U);
    }
    TEST_ASSERT(b.rec.seen.size() <= accepted);
    if (b.ep.busy)
    {
      b.Report(ErrorCode::OK, 1U);
    }
    TEST_ASSERT(b.rec.seen.size() == accepted);
  }
  TEST_ASSERT(b.FreeSlots() == 2U);
}

/// 控制器复位时多个端点上锁存着推迟动作 / A controller reset with deferred steps held
/// on several endpoints
void CheckResetWithDeferredSteps(bool direct)
{
  struct Unit
  {
    Unit(EPNumber number, uint8_t* memory, bool direct_memory)
        : ep(number, Direction::IN, {memory, 1024U}), rec(ep)
    {
      ep.limits.direct_memory = direct_memory;
    }
    FakeEndpoint ep;
    Recorder rec;
  };
  constexpr size_t UNITS = 6U;
  std::unique_ptr<Memory[]> mem(new Memory[UNITS]);
  FakeDevice dev(false);
  dev.reset_on_failure = true;
  std::unique_ptr<Unit> u[UNITS];
  for (size_t i = 0U; i < UNITS; ++i)
  {
    u[i] = std::make_unique<Unit>(static_cast<EPNumber>(i + 1U), mem[i].bytes, direct);
    dev.Add(u[i]->ep);
  }
  SetGate(dev, Gate::ALL);
  auto send = [](Unit& unit, size_t length)
  {
    Slot slot;
    TEST_ASSERT(unit.ep.AcquireSlot(slot) == ErrorCode::OK);
    TEST_ASSERT(unit.ep.Submit(slot, length, false, false) == ErrorCode::OK);
  };
  auto pend = [](Unit& unit) { unit.ep.stop_outcomes = {{ErrorCode::PENDING, 0U}}; };
  for (auto& unit : u)
  {
    TEST_ASSERT(Configure(dev, unit->ep, Direction::IN, Type::BULK, 64U) ==
                ErrorCode::OK);
    send(*unit, 10U);
  }
  // a：SP 且记下 CloseHardware；b：IP 且记下 ClearStall；c：DP；d：I1 加一个排队；
  // e：D0；f：IP，之后超时。
  // a: SP with CloseHardware due; b: IP with ClearStall due; c: DP; d: I1 plus one
  // queued; e: D0; f: IP, timing out later.
  Unit& a = *u[0];
  Unit& b = *u[1];
  Unit& c = *u[2];
  Unit& d = *u[3];
  Unit& e = *u[4];
  Unit& f = *u[5];
  pend(a);
  Close(dev, a.ep);
  TEST_ASSERT(Configure(dev, a.ep, Direction::IN, Type::BULK, 64U) == ErrorCode::OK);
  TEST_ASSERT(Stall(dev, a.ep) == ErrorCode::OK);
  pend(b);
  TEST_ASSERT(Stall(dev, b.ep) == ErrorCode::OK);
  TEST_ASSERT(ClearStall(dev, b.ep) == ErrorCode::OK);
  pend(c);
  Flush(dev, c.ep);
  Close(dev, c.ep);
  send(d, 20U);
  e.ep.stop_outcomes = {{ErrorCode::ABORTED, 0U}};
  Close(dev, e.ep);
  pend(f);
  Flush(dev, f.ep);
  for (auto& unit : u)
  {
    unit->ep.calls.clear();
    unit->rec.seen.clear();
  }

  f.ep.Later(0U, ErrorCode::TIMEOUT);
  dev.Irq();
  TEST_ASSERT(dev.resets == 1U && dev.StopFailures() == 1U);
  for (Unit* unit : {&a, &b, &c, &d, &f})
  {
    // 推迟的动作都不再执行 / None of the deferred steps runs
    TEST_ASSERT(unit->ep.CallsAre({Hw::STOP, Hw::CLOSE}));
    TEST_ASSERT(unit->ep.GetState() == State::DISABLED);
  }
  TEST_ASSERT(e.ep.calls.empty());
  TEST_ASSERT(d.rec.seen.size() == 2U);
  for (Unit* unit : {&a, &b, &c, &e, &f})
  {
    TEST_ASSERT(unit->rec.seen.empty());
  }

  // 重新绑定后的新停止结束时，没有残留的推迟动作 / When a new stop ends after the
  // rebind, no stale deferred step remains
  for (Unit* unit : {&a, &b, &c, &f})
  {
    unit->ep.calls.clear();
    TEST_ASSERT(Configure(dev, unit->ep, Direction::IN, Type::BULK, 64U) ==
                ErrorCode::OK);
    send(*unit, 30U);
    pend(*unit);
    Flush(dev, unit->ep);
    unit->ep.Later(0U, ErrorCode::ABORTED);
    dev.Irq();
    TEST_ASSERT(unit->ep.CallsAre({Hw::CONFIGURE, Hw::START, Hw::STOP}));
    send(*unit, 40U);
    TEST_ASSERT(unit->ep.Count(Hw::START) == 2U);
  }
}

/// 同一次硬件事件处理里既有 TIMEOUT 又有真实报告 / TIMEOUT and a real report in the
/// same hardware event handling
void CheckTimeoutSamePass(bool direct)
{
  Memory mem[2];
  FakeDevice dev(false);
  dev.reset_on_failure = true;
  FakeEndpoint a(EPNumber::EP1, Direction::IN, {mem[0].bytes, 1024U});
  FakeEndpoint b(EPNumber::EP2, Direction::IN, {mem[1].bytes, 1024U});
  a.limits.direct_memory = direct;
  b.limits.direct_memory = direct;
  Recorder rec_a(a);
  Recorder rec_b(b);
  dev.Add(a);
  dev.Add(b);
  SetGate(dev, Gate::ALL);
  for (FakeEndpoint* ep : {&a, &b})
  {
    TEST_ASSERT(Configure(dev, *ep, Direction::IN, Type::BULK, 64U) == ErrorCode::OK);
    Slot slot;
    TEST_ASSERT(ep->AcquireSlot(slot) == ErrorCode::OK);
    TEST_ASSERT(ep->Submit(slot, 10U, false, false) == ErrorCode::OK);
    ep->stop_outcomes = {{ErrorCode::PENDING, 0U}};
  }
  Flush(dev, a);
  Close(dev, b);
  a.calls.clear();
  b.calls.clear();
  a.Later(0U, ErrorCode::TIMEOUT);
  b.Later(0U, ErrorCode::ABORTED);
  dev.Irq();
  // b 的停止正常结束，a 的失败引起复位 / b's stop ends normally, a's failure resets
  TEST_ASSERT(dev.StopFailures() == 1U && dev.resets == 1U);
  TEST_ASSERT(b.CallsAre({Hw::CLOSE}));
  TEST_ASSERT(a.CallsAre({Hw::STOP, Hw::CLOSE}));
  TEST_ASSERT(rec_a.seen.size() == 1U && rec_b.seen.size() == 1U);
}

/// 节拍推进的停止 / Stops advanced by the tick
void CheckTickStops()
{
  using LibXR::DriverAsyncCondition;
  // min_ms 1：第一次检查在 Start 后 1~2 个节拍 / min_ms 1: the first check 1 to 2
  // ticks after Start
  {
    Bench b(true, Direction::IN, Type::BULK, 0U, 1024U, true);
    b.ep.tick_min_ms = 1;
    b.ep.quiet_after = 2U;
    b.Send(10U);
    Flush(b.dev, b.ep);
    b.Send(20U);
    b.Clear();
    const size_t services = b.dev.services;
    DriverAsyncCondition::OnTick(1U);
    TEST_ASSERT(b.dev.ticks.load() == 0U && b.dev.services == services);
    DriverAsyncCondition::OnTick(1U);
    TEST_ASSERT(b.dev.ticks.load() == 1U && b.dev.services > services);
    TEST_ASSERT(b.ep.calls.empty());
    // 节拍只挂起中断，owner 在中断里读状态 / The tick only pends the interrupt; the
    // owner reads the status in the interrupt
    TEST_ASSERT(!b.dev.log.empty() && b.dev.log[0].step == FakeDevice::Step::PEND);
    bool serviced_in_isr = false;
    for (const FakeDevice::Entry& entry : b.dev.log)
    {
      serviced_in_isr =
          serviced_in_isr || (entry.step == FakeDevice::Step::SERVICE && entry.in_isr);
    }
    TEST_ASSERT(serviced_in_isr);
    DriverAsyncCondition::OnTick(1U);
    TEST_ASSERT(b.dev.ticks.load() == 2U);
    TEST_ASSERT(b.ep.CallsAre({Hw::START}) && b.ep.Start(0U).length == 20U);
    TEST_ASSERT(b.dev.StopFailures() == 0U);
    // 没有挂起的停止就没有节拍唤醒 / No tick wake without a pending stop
    DriverAsyncCondition::OnTick(1U);
    DriverAsyncCondition::OnTick(1U);
    TEST_ASSERT(b.dev.ticks.load() == 2U);
  }
  // 可读状态：min_ms 0，每拍查一次 / Readable status: min_ms 0, checked every tick
  {
    Bench b(false, Direction::IN, Type::BULK, 0U, 1024U, true);
    b.ep.tick_min_ms = 0;
    b.Send(10U);
    Flush(b.dev, b.ep);
    b.Clear();
    DriverAsyncCondition::OnTick(1U);
    TEST_ASSERT(b.dev.ticks.load() == 1U && b.ep.stop_pending);
    b.ep.quiescent = true;
    DriverAsyncCondition::OnTick(1U);
    TEST_ASSERT(b.dev.ticks.load() == 2U && !b.ep.stop_pending);
    b.Send(20U);
    TEST_ASSERT(b.ep.CallsAre({Hw::START}));
  }
  // 永不结束：到上限报 TIMEOUT，设备复位 / Never ends: TIMEOUT at the bound, the device
  // resets
  {
    Bench b(true, Direction::IN, Type::BULK, 0U, 1024U, true);
    b.dev.reset_on_failure = true;
    b.ep.tick_min_ms = 0;
    b.ep.tick_bound = 3U;
    b.Send(10U);
    Flush(b.dev, b.ep);
    b.Clear();
    DriverAsyncCondition::OnTick(1U);
    DriverAsyncCondition::OnTick(1U);
    TEST_ASSERT(b.dev.StopFailures() == 0U);
    DriverAsyncCondition::OnTick(1U);
    TEST_ASSERT(b.dev.StopFailures() == 1U && b.dev.resets == 1U);
    TEST_ASSERT(b.ep.CallsAre({Hw::STOP, Hw::CLOSE}) && b.rec.seen.empty());
    DriverAsyncCondition::OnTick(1U);
    TEST_ASSERT(b.dev.ticks.load() == 3U);
  }
  // 到上限那一拍再读一次读到已停稳：真实结果，不复位 / The re-read at the bound finds
  // it quiescent: the real result, no reset
  {
    Bench b(true, Direction::IN, Type::BULK, 0U, 1024U, true);
    b.dev.reset_on_failure = true;
    b.ep.tick_min_ms = 0;
    b.ep.tick_bound = 2U;
    b.ep.quiescent_on_reread = true;
    b.Send(10U);
    Flush(b.dev, b.ep);
    b.Send(20U);
    b.Clear();
    DriverAsyncCondition::OnTick(1U);
    DriverAsyncCondition::OnTick(1U);
    TEST_ASSERT(b.dev.StopFailures() == 0U && b.dev.resets == 0U);
    TEST_ASSERT(b.ep.CallsAre({Hw::START}));
  }
}

/// owner 正在处理时节拍到来 / The tick arrives while the owner is serving
void CheckTickWhileOwnerServes()
{
  using LibXR::DriverAsyncCondition;
  Memory mem;
  FakeDevice dev(true);
  FakeEndpoint ep(EPNumber::EP1, Direction::IN, {mem.bytes, 1024U});
  FakeEndpoint other(EPNumber::EP2, Direction::IN, {mem.bytes + 1024, 1024U});
  Recorder rec(ep);
  Recorder rec_other(other);
  dev.Add(ep);
  dev.Add(other);
  SetGate(dev, Gate::ALL);
  TEST_ASSERT(Configure(dev, ep, Direction::IN, Type::BULK, 64U) == ErrorCode::OK);
  TEST_ASSERT(Configure(dev, other, Direction::IN, Type::BULK, 64U) == ErrorCode::OK);
  ep.tick_min_ms = 0;
  ep.quiet_after = 1U;
  Slot slot;
  TEST_ASSERT(ep.AcquireSlot(slot) == ErrorCode::OK);
  TEST_ASSERT(ep.Submit(slot, 10U, false, false) == ErrorCode::OK);
  Flush(dev, ep);
  TEST_ASSERT(ep.AcquireSlot(slot) == ErrorCode::OK);
  TEST_ASSERT(ep.Submit(slot, 20U, false, false) == ErrorCode::OK);
  TEST_ASSERT(ep.Count(Hw::START) == 1U);

  // 另一个端点的完成回调里来一个节拍 / A tick inside another endpoint's completion
  size_t services_in_hook = 0U;
  size_t services_before = 0U;
  bool pending_in_hook = false;
  rec_other.hook = [&](const Completion&, bool)
  {
    services_before = dev.services;
    DriverAsyncCondition::OnTick(1U);
    services_in_hook = dev.services;
    pending_in_hook = ep.stop_pending;
  };
  TEST_ASSERT(other.AcquireSlot(slot) == ErrorCode::OK);
  TEST_ASSERT(other.Submit(slot, 5U, false, false) == ErrorCode::OK);
  other.Later(5U);
  dev.Irq();
  // 不嵌套：回调里没有处理硬件事件，之后同一个 owner 处理 / Not nested: no hardware
  // handling inside the callback; the same owner does it afterwards
  TEST_ASSERT(dev.ticks.load() == 1U);
  TEST_ASSERT(services_in_hook == services_before && pending_in_hook);
  TEST_ASSERT(!ep.stop_pending && ep.Count(Hw::START) == 2U);
  TEST_ASSERT(ep.Start(1U).length == 20U);
  TEST_ASSERT(!ep.nested && !other.nested && !dev.masked.load());
}

/// 只有 POLL_EVENT 的一轮处理硬件事件但不开线 / A pass with POLL_EVENT only handles the
/// hardware but does not unmask
void CheckPollOnlyPass()
{
  using Step = FakeDevice::Step;
  Bench b(true, Direction::IN, Type::BULK, 0U, 1024U, true);
  std::atomic<bool> line{false};
  b.dev.AsOwner(
      [&](bool in_isr)
      {
        b.dev.pend_line = &line;
        b.dev.log.clear();
        b.dev.Poll(in_isr);
      });
  TEST_ASSERT(line.load());
  TEST_ASSERT(b.dev.log.size() == 1U && b.dev.log[0].step == Step::SERVICE);
  b.dev.pend_line = nullptr;
  b.dev.log.clear();
  b.dev.Irq();
  TEST_ASSERT(b.dev.log.size() == 3U && b.dev.log[2].step == Step::UNMASK);
}

/// 违反契约的调用以断言结束 / Contract violations end in an assertion
void CheckContractAsserts()
{
#ifdef LIBXR_DEBUG_BUILD
  for (const bool direct : {true, false})
  {
    // 对照：正常路径不触发断言 / Control: the normal path does not assert
    TEST_ASSERT(!EndsFatally(
        [direct]()
        {
          auto b = At(Row::IP, direct);
          b->Send(20U);
          b->Report();
        }));
    // TIMEOUT 之后再报告 / A report after TIMEOUT
    for (const Row row : {Row::DF, Row::IF, Row::SF})
    {
      TEST_ASSERT(
          EndsFatally([row, direct]() { At(row, direct)->Report(ErrorCode::OK, 1U); }));
    }
    // 同一次处理里先 TIMEOUT 再真实结果 / TIMEOUT, then the real result, in one handling
    TEST_ASSERT(EndsFatally(
        [direct]()
        {
          auto b = At(Row::I1, direct);
          b->Pend();
          Flush(b->dev, b->ep);
          b->ep.Later(0U, ErrorCode::TIMEOUT);
          b->Report(ErrorCode::ABORTED);
        }));
    // 没有停止挂起时报告 TIMEOUT / TIMEOUT with no stop pending
    TEST_ASSERT(
        EndsFatally([direct]() { At(Row::I1, direct)->Report(ErrorCode::TIMEOUT); }));
    // StopHardware 只能给出五种结果 / StopHardware gives one of five outcomes only
    TEST_ASSERT(EndsFatally(
        [direct]()
        {
          auto b = At(Row::I1, direct);
          b->ep.stop_outcomes = {{ErrorCode::TIMEOUT, 0U}};
          Flush(b->dev, b->ep);
        }));
    // 停止未结束时 Configure 的返回值也是最终结果：后端拒绝就当场返回它的错误，端点
    // 保持 DISABLED；接受之后，停止结束时的硬件配置不再有失败可言
    // The return value of Configure is final also while a stop is pending: a refusal by
    // the backend is returned at once and the endpoint stays DISABLED; once accepted, the
    // hardware configuration at the end of the stop has no way to fail
    {
      auto b = At(Row::DP, direct);
      b->ep.configure_result = ErrorCode::NOT_SUPPORT;
      TEST_ASSERT(Configure(b->dev, b->ep, Direction::IN, Type::BULK, 64U) ==
                  ErrorCode::NOT_SUPPORT);
      TEST_ASSERT(b->ep.GetState() == Endpoint::State::DISABLED);
      b->ep.configure_result = ErrorCode::OK;
      TEST_ASSERT(Configure(b->dev, b->ep, Direction::IN, Type::BULK, 64U) ==
                  ErrorCode::OK);
      b->ep.configure_result = ErrorCode::FAILED;
      b->Report();
      TEST_ASSERT(b->ep.GetState() == Endpoint::State::IDLE);
    }
  }
  // 没有 pend 的后端不能在中断里用节拍唤醒（线程里可以，就地成为 owner，见
  // test_usb_detach_timer.cpp）/ A backend without pend cannot use the tick wake in an
  // interrupt (a thread can and owns in place, see test_usb_detach_timer.cpp)
  TEST_ASSERT(EndsFatally(
      []()
      {
        Bench b(true);
        b.dev.Poll(true);
      }));
#endif
}

/**
 * @brief 多线程测试的设备：任何线程可在 owner 中运行任务，后端报告经硬件事件线
 *        Device of the multi-threaded test: any thread runs a job in the owner, backend
 *        reports go through the hardware event line
 */
class StopDevice final : public DeviceService
{
 public:
  explicit StopDevice(bool pend) : pend_(pend), tick_(&TickCheck, &TickDone, this)
  {
    SetDeviceHandler(Callback<uint32_t>::Create(
        [](bool in_isr, StopDevice* self, uint32_t events)
        {
          std::function<void(bool)>* job =
              self->job_.exchange(nullptr, std::memory_order_acq_rel);
          if ((events & 1U) == 0U || job == nullptr)
          {
            return;
          }
          (*job)(in_isr);
          self->done_.store(true, std::memory_order_release);
        },
        this));
  }

  void Add(FakeEndpoint& ep)
  {
    TEST_ASSERT(Register(ep) == ErrorCode::OK);
    eps_.push_back(&ep);
    ep.start_tick = [this](uint32_t min_ms)
    {
      (void)tick_.Start(min_ms, min_ms);
      return ticks.load(std::memory_order_acquire);
    };
  }

  /// 在 owner 中运行 job 并等它做完；只由一个线程调用 / Run job in the owner and wait;
  /// called by one thread only
  void RunInOwner(std::function<void(bool)> job)
  {
    done_.store(false, std::memory_order_relaxed);
    job_.store(&job, std::memory_order_release);
    Request(1U, false);
    while (!done_.load(std::memory_order_acquire))
    {
      std::this_thread::yield();
    }
  }

  /// 线有效且未屏蔽时运行中断入口 / Run the interrupt entry while the line is raised
  /// and not masked
  void Poll()
  {
    if (!masked_.load(std::memory_order_acquire) &&
        (hw_line.exchange(false, std::memory_order_acq_rel) |
         line_.exchange(false, std::memory_order_acq_rel)))
    {
      OnControllerInterrupt();
    }
    else
    {
      std::this_thread::yield();
    }
  }

  std::atomic<bool> hw_line{false};  ///< 后端有报告 / The backend has a report
  std::atomic<uint32_t> ticks{0U};   ///< 节拍唤醒次数 / Tick wakes

 protected:
  void ServiceHardware(bool in_isr) override
  {
    const uint32_t now = ticks.load(std::memory_order_acquire);
    bool waiting = false;
    for (FakeEndpoint* ep : eps_)
    {
      ep->ServiceLater(in_isr);
      waiting = ep->ServiceTick(in_isr, now) || waiting;
    }
    if (waiting)
    {
      (void)tick_.Start(0U, 0U);
    }
  }
  void MaskInterrupt() override { masked_.store(true, std::memory_order_release); }
  void UnmaskInterrupt() override { masked_.store(false, std::memory_order_release); }
  bool PendInterrupt() override
  {
    if (pend_)
    {
      line_.store(true, std::memory_order_release);
    }
    return pend_;
  }

 private:
  static bool TickCheck(void*) { return true; }
  static void TickDone(void* arg, ErrorCode)
  {
    auto* self = static_cast<StopDevice*>(arg);
    self->ticks.fetch_add(1U, std::memory_order_acq_rel);
    self->RequestPoll(true);
  }

  const bool pend_;
  std::atomic<bool> line_{false};
  std::atomic<bool> masked_{false};
  std::atomic<std::function<void(bool)>*> job_{nullptr};
  std::atomic<bool> done_{false};
  std::vector<FakeEndpoint*> eps_;
  LibXR::DriverAsyncCondition tick_;
};

/**
 * @brief 其他线程在停止挂起期间借槽、提交 / Other threads borrow and submit while
 *        stops are pending
 * @param direct 硬件直接读写槽 / The hardware accesses the slot directly
 * @param pend 设备有挂起函数，停止也经节拍结束 / The device can pend; stops also end
 *        through the tick
 * @note 依据：每个被接受的请求恰好一次回调；同一个槽不同时借给两个持有者；硬件直接读写
 *       的被停槽在停止结束前不借出；停止挂起期间不启动（假后端断言）。
 *       Basis: every accepted request gets exactly one callback; a slot is never lent
 *       to two holders at once; a stopped slot the hardware accesses directly is not
 *       lent before the stop ends; nothing starts while a stop is pending (the fake
 *       backend asserts it).
 */
void CheckStopsAgainstThreads(bool direct, bool pend)
{
  constexpr uint32_t ROUNDS = 1500U;
  constexpr uint32_t SUBMITTERS = 2U;
  Memory mem;
  StopDevice dev(pend);
  FakeEndpoint ep(EPNumber::EP1, Direction::IN, {mem.bytes, 1024U});
  ep.limits.direct_memory = direct;
  dev.Add(ep);
  const uint8_t* base = mem.bytes;

  std::atomic<bool> stop_irq{false};
  std::atomic<bool> stop_tick{false};
  std::thread irq(
      [&]()
      {
        while (!stop_irq.load(std::memory_order_acquire))
        {
          dev.Poll();
        }
      });
  std::thread tick(
      [&]()
      {
        while (pend && !stop_tick.load(std::memory_order_acquire))
        {
          LibXR::DriverAsyncCondition::OnTick(1U);
          std::this_thread::sleep_for(std::chrono::microseconds(20));
        }
      });
  dev.RunInOwner([&](bool) { dev.SetStartGate(Gate::ALL); });

  // 硬件仍在用的槽（只在直接读写时记）/ Slot the hardware still uses (direct only)
  std::atomic<const uint8_t*> held{nullptr};
  // 以下只在 owner 中读写 / Read and written in the owner only
  uint32_t starts = 0U;
  uint32_t stops = 0U;
  uint32_t callbacks = 0U;
  const uint8_t* loaded = nullptr;
  ep.on_start =
      [&](FakeEndpoint& self, const FakeEndpoint::HwRequest& request, bool in_isr)
  {
    loaded = request.data;
    uint32_t header = 0U;
    std::memcpy(&header, request.data, sizeof(header));
    TEST_ASSERT(header == 0x5AA5U);
    if ((starts++ & 1U) == 0U)
    {
      self.Report(in_isr, request.length, ErrorCode::OK);
    }
    else
    {
      self.Later(request.length);
      dev.hw_line.store(true, std::memory_order_release);
    }
  };
  ep.on_stop = [&](FakeEndpoint& self, bool)
  {
    if (direct && self.stop_pending)
    {
      held.store(loaded, std::memory_order_release);
    }
  };
  ep.on_stop_end = [&]() { held.store(nullptr, std::memory_order_release); };
  ep.stop_rule = [&](FakeEndpoint& self) -> FakeEndpoint::StopOutcome
  {
    const uint32_t kind = stops++ % 3U;
    if (kind == 0U)
    {
      return {ErrorCode::ABORTED, 0U};
    }
    if (kind == 2U && pend)
    {
      self.tick_stop = true;
      self.quiet_after = 1U;
      self.tick_bound = UINT32_MAX;
      self.tick_start = self.start_tick(0U);
    }
    else
    {
      self.Later(0U, ErrorCode::ABORTED);
      dev.hw_line.store(true, std::memory_order_release);
    }
    return {ErrorCode::PENDING, 0U};
  };
  ep.SetOnComplete(Callback<const Completion&>::Create(
      [](bool, uint32_t* count, const Completion&) { ++*count; }, &callbacks));

  ErrorCode result = ErrorCode::PENDING;
  dev.RunInOwner([&](bool in_isr)
                 { result = ep.Configure({Direction::IN, Type::BULK, 64U}, in_isr); });
  TEST_ASSERT(result == ErrorCode::OK);

  std::atomic<bool> stop{false};
  std::atomic<uint32_t> accepted{0U};
  std::atomic<uint32_t> odd{0U};
  std::atomic<int> holders[2] = {};
  std::vector<std::thread> submitters;
  for (uint32_t t = 0U; t < SUBMITTERS; ++t)
  {
    submitters.emplace_back(
        [&]()
        {
          while (!stop.load(std::memory_order_acquire))
          {
            Slot slot;
            const ErrorCode lend = ep.AcquireSlot(slot);
            if (lend != ErrorCode::OK)
            {
              odd.fetch_add(
                  (lend == ErrorCode::BUSY || lend == ErrorCode::STATE_ERR) ? 0U : 1U);
              std::this_thread::yield();
              continue;
            }
            auto* data = static_cast<uint8_t*>(slot.data.addr_);
            const size_t index = (data == base) ? 0U : 1U;
            // 不同时借给两个持有者，也不借出硬件仍在用的槽 / Never lent to two holders
            // at once, nor while the hardware still uses it
            if (holders[index].exchange(1) != 0 ||
                held.load(std::memory_order_acquire) == data)
            {
              odd.fetch_add(1U);
            }
            const uint32_t header = 0x5AA5U;
            std::memcpy(data, &header, sizeof(header));
            holders[index].store(0);
            const ErrorCode submit = ep.Submit(slot, 8U, false, false);
            if (submit == ErrorCode::OK)
            {
              accepted.fetch_add(1U, std::memory_order_relaxed);
            }
            else if (submit != ErrorCode::ABORTED)
            {
              odd.fetch_add(1U);
            }
          }
        });
  }

  for (uint32_t i = 0U; i < ROUNDS; ++i)
  {
    dev.RunInOwner(
        [&](bool in_isr)
        {
          switch (i % 6U)
          {
            case 0U:
            case 5U:
              ep.Flush(in_isr);
              break;
            case 1U:
              (void)ep.Stall(in_isr);
              break;
            case 2U:
              (void)ep.ClearStall(in_isr);
              break;
            case 3U:
              ep.Close(in_isr);
              break;
            default:
            {
              const ErrorCode configured =
                  ep.Configure({Direction::IN, Type::BULK, 64U}, in_isr);
              TEST_ASSERT(configured == ErrorCode::OK ||
                          configured == ErrorCode::STATE_ERR);
              break;
            }
          }
        });
    std::this_thread::yield();
  }
  stop.store(true, std::memory_order_release);
  for (std::thread& submitter : submitters)
  {
    submitter.join();
  }

  // 让所有报告和停止结束，再关闭 / Let every report and stop finish, then close
  auto drain = [&]()
  {
    bool idle = false;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(60);
    while (!idle)
    {
      TEST_ASSERT(std::chrono::steady_clock::now() < deadline);
      dev.RunInOwner([&](bool)
                     { idle = !ep.busy && !ep.stop_pending && ep.LaterCount() == 0U; });
    }
  };
  dev.RunInOwner([&](bool in_isr) { (void)ep.ClearStall(in_isr); });
  drain();
  dev.RunInOwner([&](bool in_isr) { ep.Close(in_isr); });
  drain();
  uint32_t delivered = 0U;
  dev.RunInOwner([&](bool) { delivered = callbacks; });
  TEST_ASSERT(odd.load() == 0U);
  TEST_ASSERT(accepted.load() != 0U && delivered == accepted.load());
  TEST_ASSERT(!ep.nested);

  dev.RunInOwner([&](bool in_isr)
                 { result = ep.Configure({Direction::IN, Type::BULK, 64U}, in_isr); });
  TEST_ASSERT(result == ErrorCode::OK);
  Slot a;
  Slot b;
  TEST_ASSERT(ep.AcquireSlot(a) == ErrorCode::OK);
  TEST_ASSERT(ep.AcquireSlot(b) == ErrorCode::OK);
  stop_tick.store(true, std::memory_order_release);
  tick.join();
  stop_irq.store(true, std::memory_order_release);
  irq.join();
}

/// 一个服务区间的包数：上限、类型、速度、包长、槽和后端上限 / Packets per service
/// interval: bound, type, speed, packet size, slot and backend limits
void CheckIntervalPackets()
{
  struct Row
  {
    Speed speed;
    Type type;
    uint16_t bound;       ///< 包长上限 / Packet-size bound
    uint8_t wanted;       ///< 配置里的包数上限 / Count bound in the configuration
    uint8_t hw_interval;  ///< 后端一个区间的上限 / Backend limit per interval
    size_t hw_packets;    ///< 后端一次请求的包数上限 / Backend packets per request
    size_t hw_bytes;      ///< 后端一次请求的字节上限 / Backend bytes per request
    uint8_t packets;      ///< 期望的包数 / Expected count
    size_t transfer;      ///< 期望的 MaxTransferSize / Expected MaxTransferSize
  };
  constexpr size_t ANY = SIZE_MAX;
  const Row rows[] = {
      // 高速等时：上限、后端和规范三者取小 / High-speed isochronous: the smallest of
      // the bound, the backend and the specification
      {Speed::HIGH, Type::ISOCHRONOUS, 1024U, 3U, 3U, 3U, 3072U, 3U, 3072U},
      {Speed::HIGH, Type::ISOCHRONOUS, 1024U, 2U, 3U, 3U, 3072U, 2U, 2048U},
      {Speed::HIGH, Type::ISOCHRONOUS, 1024U, 1U, 3U, 3U, 3072U, 1U, 1024U},
      {Speed::HIGH, Type::ISOCHRONOUS, 1024U, 200U, 3U, 3U, 3072U, 3U, 3072U},
      {Speed::HIGH, Type::ISOCHRONOUS, 1024U, 3U, 2U, 3U, 3072U, 2U, 2048U},
      {Speed::HIGH, Type::ISOCHRONOUS, 1024U, 3U, 1U, 3U, 3072U, 1U, 1024U},
      // 后端一次请求的上限也限制等时的包数 / The backend's limits for one request
      // bound the isochronous count as well
      {Speed::HIGH, Type::ISOCHRONOUS, 1024U, 3U, 3U, 2U, ANY, 2U, 2048U},
      {Speed::HIGH, Type::ISOCHRONOUS, 1024U, 3U, 3U, ANY, 2500U, 2U, 2048U},
      {Speed::HIGH, Type::ISOCHRONOUS, 1024U, 3U, 3U, 1U, 1024U, 1U, 1024U},
      // 包长太小时包数退让（USB 2.0 表 9-14）/ A small packet size lowers the count
      // (USB 2.0 Table 9-14)
      {Speed::HIGH, Type::ISOCHRONOUS, 683U, 3U, 3U, 3U, 3072U, 3U, 2049U},
      {Speed::HIGH, Type::ISOCHRONOUS, 682U, 3U, 3U, 3U, 3072U, 2U, 1364U},
      {Speed::HIGH, Type::ISOCHRONOUS, 513U, 3U, 3U, 3U, 3072U, 2U, 1026U},
      {Speed::HIGH, Type::ISOCHRONOUS, 512U, 3U, 3U, 3U, 3072U, 1U, 512U},
      // 全速只有一个 / Full speed has one only
      {Speed::FULL, Type::ISOCHRONOUS, 1023U, 3U, 3U, 3U, 3072U, 1U, 1023U},
      {Speed::FULL, Type::INTR, 64U, 3U, 3U, ANY, ANY, 1U, 3072U},
      // 高速中断：不改变请求的大小 / High-speed interrupt: the request size is unchanged
      {Speed::HIGH, Type::INTR, 1024U, 3U, 3U, ANY, ANY, 3U, 3072U},
      {Speed::HIGH, Type::INTR, 1024U, 3U, 1U, ANY, ANY, 1U, 3072U},
      {Speed::HIGH, Type::INTR, 1024U, 200U, 200U, ANY, ANY, 3U, 3072U},
      {Speed::HIGH, Type::INTR, 1024U, 2U, 3U, 1U, 1024U, 2U, 1024U},
      {Speed::HIGH, Type::INTR, 600U, 3U, 3U, ANY, ANY, 2U, 3000U},
      // 批量和控制总是一个 / Bulk and control always have one
      {Speed::HIGH, Type::BULK, 512U, 3U, 3U, ANY, ANY, 1U, 3072U},
      {Speed::HIGH, Type::CONTROL, 64U, 3U, 3U, ANY, ANY, 1U, 64U},
  };

  alignas(64) static uint8_t memory[6144];
  FakeDevice dev(false);
  // 两个 3072 字节的槽 / Two slots of 3072 bytes
  FakeEndpoint ep(EPNumber::EP1, Direction::BOTH, {memory, sizeof(memory)});
  dev.Add(ep);
  for (const Row& row : rows)
  {
    SetBusSpeed(dev, row.speed);
    ep.limits = {};
    ep.limits.max_packet_size = 1024U;
    ep.limits.max_packets_per_interval = row.hw_interval;
    ep.limits.max_packets = row.hw_packets;
    ep.limits.max_bytes = row.hw_bytes;
    const Endpoint::Config cfg{Direction::IN, row.type, row.bound, row.wanted};
    ErrorCode result = ErrorCode::PENDING;
    dev.AsOwner([&](bool in_isr) { result = ep.Configure(cfg, in_isr); });
    TEST_ASSERT(result == ErrorCode::OK);
    TEST_ASSERT(ep.PacketsPerInterval() == row.packets);
    TEST_ASSERT(ep.MaxTransferSize() == row.transfer);
    // 后端拿到的是选定的包数，不是上限 / The backend gets the chosen count, not the
    // bound
    TEST_ASSERT(ep.configured.max_packets_per_interval == row.packets);
    TEST_ASSERT(ep.GetConfig().max_packets_per_interval == row.packets);
    Close(dev, ep);
  }

  // 一个服务区间要放进一个槽：两个 1024 字节的槽只够一个 1024 字节的包
  // One service interval has to fit one slot: two slots of 1024 bytes hold one packet of
  // 1024 bytes each
  Memory mem;
  FakeEndpoint narrow(EPNumber::EP2, Direction::IN, {mem.bytes, 2048U});
  dev.Add(narrow);
  SetBusSpeed(dev, Speed::HIGH);
  narrow.limits.max_packets_per_interval = 3U;
  const Endpoint::Config wide{Direction::IN, Type::ISOCHRONOUS, 1024U, 3U};
  ErrorCode result = ErrorCode::PENDING;
  dev.AsOwner([&](bool in_isr) { result = narrow.Configure(wide, in_isr); });
  TEST_ASSERT(result == ErrorCode::OK && narrow.PacketsPerInterval() == 1U);
  TEST_ASSERT(narrow.MaxTransferSize() == 1024U);
  Close(dev, narrow);
  // 中断类型的包数不受槽限制 / The count of the interrupt type is not bound by the slot
  const Endpoint::Config intr{Direction::IN, Type::INTR, 1024U, 3U};
  dev.AsOwner([&](bool in_isr) { result = narrow.Configure(intr, in_isr); });
  TEST_ASSERT(result == ErrorCode::OK && narrow.PacketsPerInterval() == 3U);
  TEST_ASSERT(narrow.MaxTransferSize() == 1024U);
  Close(dev, narrow);

  // 包数上限为 0 是参数错误 / A count bound of 0 is an argument error
  const Endpoint::Config none{Direction::IN, Type::ISOCHRONOUS, 1024U, 0U};
  dev.AsOwner([&](bool in_isr) { result = narrow.Configure(none, in_isr); });
  TEST_ASSERT(result == ErrorCode::ARG_ERR && narrow.GetState() == State::DISABLED);
  // 默认的配置和默认的后端上限都是一个包 / The default configuration and the default
  // backend limit both mean one packet
  narrow.limits = {};
  TEST_ASSERT(Configure(dev, narrow, Direction::IN, Type::ISOCHRONOUS, 1024U) ==
              ErrorCode::OK);
  TEST_ASSERT(narrow.PacketsPerInterval() == 1U && narrow.MaxTransferSize() == 1024U);
  Close(dev, narrow);
  narrow.limits.max_packets_per_interval = 3U;
  TEST_ASSERT(Configure(dev, narrow, Direction::IN, Type::INTR, 1024U) == ErrorCode::OK);
  TEST_ASSERT(narrow.PacketsPerInterval() == 1U);
  Close(dev, narrow);

  // 描述符字段：位 10..0 包长，位 12..11 包数减一 / Descriptor field: bits 10..0 the
  // size, bits 12..11 the count minus one
  static_assert(Endpoint::PacketSizeField(1024U, 1U) == 0x0400U);
  static_assert(Endpoint::PacketSizeField(1024U, 2U) == 0x0C00U);
  static_assert(Endpoint::PacketSizeField(1024U, 3U) == 0x1400U);
  static_assert(Endpoint::PacketSizeField(683U, 3U) == 0x12ABU);
  static_assert(Endpoint::PacketSizeField(64U, 1U) == 64U);
}

void CheckConfigure()
{
  CheckPacketSizes();
  CheckSlotsAndLimits();
  CheckConfigureErrors();
  CheckIntervalPackets();
}

/// 重新配置成硬件上可同时挂 limit 个请求 / Configure again with limit requests on the
/// hardware at once
void AllowAhead(Rig& r, Direction direction, uint8_t limit, bool direct = true)
{
  End(r.dev, r.ep, Ender::CLOSE);
  r.ep.limits.max_requests = limit;
  r.ep.limits.direct_memory = direct;
  TEST_ASSERT(Configure(r.dev, r.ep, direction, Type::BULK, 64U) == ErrorCode::OK);
  r.ep.calls.clear();
  r.rec.seen.clear();
}

/// 后端报 2 时第二个已提交的请求提前交给硬件；完成和回调按提交的先后
/// With a backend limit of 2 the second submitted request reaches the hardware ahead;
/// completions and callbacks follow the order of submission
void CheckAheadOrder()
{
  Rig r(EPNumber::EP1, Direction::IN);
  AllowAhead(r, Direction::IN, 2U);
  r.Send(10U);
  r.Send(20U);
  TEST_ASSERT(r.ep.CallsAre({Hw::START, Hw::START}));
  TEST_ASSERT(r.ep.Start(0).length == 10U && r.ep.Start(1).length == 20U);
  TEST_ASSERT(r.ep.Start(0).data != r.ep.Start(1).data);
  TEST_ASSERT(r.rec.seen.empty());
  // 两个槽都在硬件上：借不到第三个 / Both slots are on the hardware: no third lend
  Slot none;
  TEST_ASSERT(r.ep.AcquireSlot(none) != ErrorCode::OK);
  r.Complete(10U);
  TEST_ASSERT(r.rec.seen.size() == 1U && r.rec.At(0).actual == 10U);
  // 第一个完成后空出的槽再提交：排在第二个后面提前交下去
  // The slot freed by the first completion is submitted again and goes down ahead,
  // behind the second
  r.Send(30U);
  TEST_ASSERT(r.ep.CallsAre({Hw::START, Hw::START, Hw::START}));
  TEST_ASSERT(r.ep.Start(2).length == 30U && r.ep.Start(2).data == r.ep.Start(0).data);
  r.Complete(20U);
  r.Complete(30U);
  TEST_ASSERT(r.rec.seen.size() == 3U && r.rec.At(1).actual == 20U &&
              r.rec.At(2).actual == 30U);
  TEST_ASSERT(!r.ep.busy && r.ep.ahead == 0);

  // 带 ZLP 标记但非整数包长提前提交，AheadAt 校验 owed=false / Ahead ZLP flag with
  // non-packet-multiple length exercises AheadAt owed=false
  r.ep.calls.clear();
  r.rec.seen.clear();
  r.Send(10U, true);
  r.Send(20U, true);
  TEST_ASSERT(r.ep.CallsAre({Hw::START, Hw::START}));
  r.Complete(10U);
  r.Complete(20U);
  TEST_ASSERT(r.rec.seen.size() == 2U && r.rec.At(0).actual == 10U &&
              r.rec.At(1).actual == 20U);
  TEST_ASSERT(!r.ep.busy && r.ep.ahead == 0);

  // 控制端点即使有两个槽、后端报 2：仍然一次一个
  // A control endpoint, even with two slots and a backend limit of 2: still one at a
  // time
  Memory mem;
  FakeDevice dev(false);
  FakeEndpoint ep0(EPNumber::EP0, Direction::IN, {mem.bytes, 512U}, 2U);
  dev.Add(ep0);
  SetGate(dev, Gate::ALL);
  ep0.limits.max_requests = 2U;
  TEST_ASSERT(Configure(dev, ep0, Direction::IN, Type::CONTROL, 64U) == ErrorCode::OK);
  Slot a;
  Slot b;
  TEST_ASSERT(ep0.AcquireSlot(a) == ErrorCode::OK);
  TEST_ASSERT(ep0.AcquireSlot(b) == ErrorCode::OK);
  TEST_ASSERT(ep0.Submit(a, 8U, false, false) == ErrorCode::OK);
  TEST_ASSERT(ep0.Submit(b, 8U, false, false) == ErrorCode::OK);
  TEST_ASSERT(ep0.Count(Hw::START) == 1U);
}

/// 接收同样提前交下去；完成按先后，各借出自己的槽 / Receives go down ahead as well;
/// completions come in order, each lending its own slot
void CheckAheadOut()
{
  Rig r(EPNumber::EP1, Direction::OUT);
  AllowAhead(r, Direction::OUT, 2U);
  TEST_ASSERT(r.ep.Receive(64U, false) == ErrorCode::OK);
  TEST_ASSERT(r.ep.Receive(64U, false) == ErrorCode::OK);
  TEST_ASSERT(r.ep.CallsAre({Hw::START, Hw::START}));
  r.Complete(5U);
  r.Complete(7U);
  TEST_ASSERT(r.rec.seen.size() == 2U);
  TEST_ASSERT(r.rec.At(0).actual == 5U && r.rec.At(0).slot.Valid());
  TEST_ASSERT(r.rec.At(1).actual == 7U && r.rec.At(1).slot.Valid());
  TEST_ASSERT(r.rec.At(0).slot.data.addr_ == r.ep.Start(0).data);
  TEST_ASSERT(r.rec.At(1).slot.data.addr_ == r.ep.Start(1).data);
}

/// 结尾零长度包紧跟它的数据：欠着它的请求后面不提前交；提前交下去的请求自己欠的零长度
/// 包照常在它的数据之后发
/// A closing zero-length packet follows its data: nothing goes down ahead behind a
/// request that owes one, and a request that went down ahead still gets its own closing
/// packet after its data
void CheckAheadZlp()
{
  Rig r(EPNumber::EP1, Direction::IN);
  AllowAhead(r, Direction::IN, 2U);
  r.Send(64U, true);
  r.Send(10U);
  TEST_ASSERT(r.ep.CallsAre({Hw::START}));
  r.Complete(64U);
  TEST_ASSERT(r.ep.CallsAre({Hw::START, Hw::START}) && r.ep.Start(1).length == 0U);
  TEST_ASSERT(r.rec.seen.empty());
  r.Complete(0U);
  TEST_ASSERT(r.rec.seen.size() == 1U && r.rec.At(0).actual == 64U);
  TEST_ASSERT(r.ep.Count(Hw::START) == 3U && r.ep.Start(2).length == 10U);
  r.Complete(10U);
  TEST_ASSERT(r.rec.seen.size() == 2U);

  r.ep.calls.clear();
  r.rec.seen.clear();
  r.Send(10U);
  r.Send(128U, true);
  TEST_ASSERT(r.ep.CallsAre({Hw::START, Hw::START}));
  r.Complete(10U);
  TEST_ASSERT(r.rec.seen.size() == 1U);
  r.Complete(128U);
  TEST_ASSERT(r.rec.seen.size() == 1U && r.ep.Count(Hw::START) == 3U &&
              r.ep.Start(2).length == 0U);
  r.Complete(0U);
  TEST_ASSERT(r.rec.seen.size() == 2U && r.rec.At(1).result == ErrorCode::OK &&
              r.rec.At(1).actual == 128U);
}

/// 提前交下去的请求启动失败：它的 FAILED 回调排在更早那个请求的回调之后
/// A request handed down ahead fails to start: its FAILED callback comes after the
/// callback of the earlier request
void CheckAheadStartFails()
{
  Rig r(EPNumber::EP1, Direction::IN);
  AllowAhead(r, Direction::IN, 2U);
  r.ep.start_results = {ErrorCode::OK, ErrorCode::FAILED};
  r.Send(10U);
  r.Send(20U);
  TEST_ASSERT(r.ep.CallsAre({Hw::START, Hw::START}));
  TEST_ASSERT(r.rec.seen.empty() && r.ep.ahead == 0);
  r.Complete(10U);
  TEST_ASSERT(r.rec.seen.size() == 2U && r.rec.At(0).result == ErrorCode::OK &&
              r.rec.At(0).actual == 10U && r.rec.At(1).result == ErrorCode::FAILED);
  // 两个槽都回来了 / Both slots are back
  r.Send(1U);
  r.Send(2U);
  TEST_ASSERT(r.ep.Count(Hw::START) == 4U);
  r.Complete(1U);
  r.Complete(2U);

  // 更早的请求被停止时：只对硬件上的那个调用 StopHardware，回调仍按先后
  // When the earlier request is stopped: StopHardware is called for the one on the
  // hardware only, and the callbacks still come in order
  r.ep.calls.clear();
  r.rec.seen.clear();
  r.ep.start_results = {ErrorCode::OK, ErrorCode::FAILED};
  r.ep.stop_outcomes = {{ErrorCode::ABORTED, 3U}};
  r.Send(10U);
  r.Send(20U);
  End(r.dev, r.ep, Ender::FLUSH);
  TEST_ASSERT(r.ep.CallsAre({Hw::START, Hw::START, Hw::STOP}));
  TEST_ASSERT(r.rec.seen.size() == 2U && r.rec.At(0).result == ErrorCode::ABORTED &&
              r.rec.At(0).actual == 3U && r.rec.At(1).result == ErrorCode::FAILED);
}

/// 停止：硬件上有两个请求时 StopHardware 调用两次，各取一个结果，回调按先后
/// Stop: with two requests on the hardware StopHardware is called twice, once for each
/// outcome, and the callbacks come in order
void CheckAheadStopNow(Ender how, Direction direction)
{
  Rig r(EPNumber::EP1, direction);
  AllowAhead(r, direction, 2U);
  const bool in = direction == Direction::IN;
  if (in)
  {
    r.Send(10U);
    r.Send(128U, true);
  }
  else
  {
    TEST_ASSERT(r.ep.Receive(64U, false) == ErrorCode::OK);
    TEST_ASSERT(r.ep.Receive(64U, false) == ErrorCode::OK);
  }
  r.ep.stop_outcomes = {{ErrorCode::OK, 10U}, {ErrorCode::OK, 128U}};
  End(r.dev, r.ep, how);
  TEST_ASSERT(r.ep.Count(Hw::STOP) == 2U && !r.ep.busy && r.ep.ahead == 0);
  TEST_ASSERT(r.rec.seen.size() == 2U);
  // IN：第一个硬件已做完，OK；第二个数据做完但零长度包没发，ABORTED、actual 为数据长度。
  // OUT：被停的接收总是 ABORTED。
  // IN: the hardware finished the first, OK; the second has its data done but its
  // closing packet unsent, ABORTED with the data length. OUT: a stopped receive is
  // always ABORTED.
  TEST_ASSERT(r.rec.At(0).result == (in ? ErrorCode::OK : ErrorCode::ABORTED));
  TEST_ASSERT(r.rec.At(0).actual == 10U);
  TEST_ASSERT(r.rec.At(1).result == ErrorCode::ABORTED && r.rec.At(1).actual == 128U);
  TEST_ASSERT(!r.rec.At(0).slot.Valid() && !r.rec.At(1).slot.Valid());
  if (how == Ender::CLOSE)
  {
    TEST_ASSERT(r.ep.calls.back().function == Hw::CLOSE);
    return;
  }
  if (how == Ender::STALL)
  {
    TEST_ASSERT(r.ep.calls.back().function == Hw::STALL);
    TEST_ASSERT(r.ep.ClearStall(false) == ErrorCode::OK);
  }
  // 两个槽都空出来了 / Both slots are free again
  if (in)
  {
    r.Send(1U);
    r.Send(2U);
    TEST_ASSERT(r.ep.busy && r.ep.ahead == 1);
  }
}

/// 停止挂起：两个请求当场各回调一次 ABORTED；后端之后按先后各报告一次，第二次才是停止
/// 结束
/// A pending stop: both requests are called back with ABORTED at once; the backend
/// later reports once for each in order, and only the second report ends the stop
void CheckAheadStopPending(bool direct)
{
  Rig r(EPNumber::EP1, Direction::IN);
  AllowAhead(r, Direction::IN, 2U, direct);
  r.Send(10U);
  r.Send(20U);
  r.ep.stop_outcomes = {{ErrorCode::PENDING, 0U}};
  End(r.dev, r.ep, Ender::FLUSH);
  TEST_ASSERT(r.ep.Count(Hw::STOP) == 1U && r.ep.stop_pending);
  TEST_ASSERT(r.rec.seen.size() == 2U && r.rec.At(0).result == ErrorCode::ABORTED &&
              r.rec.At(1).result == ErrorCode::ABORTED && r.rec.At(1).actual == 0U);
  // 硬件直接读写槽时两个槽都留到停止结束；否则当场放回，新请求排队等停止结束
  // With hardware that accesses the slots both stay until the stop ends; otherwise they
  // are given back at once and a new request waits in the queue for the end of the stop
  Slot slot;
  TEST_ASSERT((r.ep.AcquireSlot(slot) == ErrorCode::OK) == !direct);
  if (!direct)
  {
    TEST_ASSERT(r.ep.Submit(slot, 5U, false, false) == ErrorCode::OK);
  }
  r.Complete(10U, ErrorCode::ABORTED);
  // 第一次报告之后停止还没有结束：什么都不启动 / After the first report the stop has
  // not ended: nothing starts
  TEST_ASSERT(r.ep.stop_pending && r.ep.Count(Hw::START) == 2U);
  TEST_ASSERT(r.rec.seen.size() == 2U);
  r.Complete(0U, ErrorCode::ABORTED);
  TEST_ASSERT(!r.ep.stop_pending && r.rec.seen.size() == 2U);
  if (!direct)
  {
    TEST_ASSERT(r.ep.Count(Hw::START) == 3U && r.ep.Start(2).length == 5U);
    r.Complete(5U);
  }
  r.Send(1U);
  r.Send(2U);
  TEST_ASSERT(r.ep.busy && r.ep.ahead == 1);
}

/// 停止超时：只报一次；控制器复位之后两个槽都回来 / A stop that times out is reported
/// once; after the controller reset both slots are back
void CheckAheadStopTimeout()
{
  Rig r(EPNumber::EP1, Direction::IN);
  AllowAhead(r, Direction::IN, 2U);
  r.Send(10U);
  r.Send(20U);
  r.ep.stop_outcomes = {{ErrorCode::PENDING, 0U}};
  End(r.dev, r.ep, Ender::FLUSH);
  r.Complete(0U, ErrorCode::TIMEOUT);
  TEST_ASSERT(r.dev.StopFailures() == 1U);
  r.dev.AsOwner([&](bool in_isr) { r.dev.ResetAndCloseAll(in_isr); });
  TEST_ASSERT(r.ep.GetState() == State::DISABLED && !r.ep.busy && r.ep.ahead == 0);
  TEST_ASSERT(Configure(r.dev, r.ep, Direction::IN, Type::BULK, 64U) == ErrorCode::OK);
  r.ep.calls.clear();
  r.Send(1U);
  r.Send(2U);
  TEST_ASSERT(r.ep.CallsAre({Hw::START, Hw::START}));
}

/// 四个槽、后端报 limit：硬件上同时挂的请求数取两者较小的，完成和回调按提交的先后
/// Four slots and a backend limit: the requests on the hardware at once are the smaller
/// of the two, completions and callbacks follow the order of submission
void CheckManyAheadOrder(uint8_t limit)
{
  Rig r(EPNumber::EP1, Direction::IN, Type::BULK, false, 2048U, 4U);
  AllowAhead(r, Direction::IN, limit);
  const size_t on_hardware = limit < 4U ? limit : 4U;
  for (size_t i = 0U; i < 4U; ++i)
  {
    r.Send(10U * (i + 1U));
  }
  TEST_ASSERT(r.ep.Count(Hw::START) == on_hardware);
  TEST_ASSERT(r.ep.ahead == static_cast<int>(on_hardware) - 1);
  for (size_t i = 0U; i < on_hardware; ++i)
  {
    TEST_ASSERT(r.ep.Start(i).length == 10U * (i + 1U));
    for (size_t j = 0U; j < i; ++j)
    {
      TEST_ASSERT(r.ep.Start(i).data != r.ep.Start(j).data);
    }
  }
  // 四个槽都提交了：借不到第五个 / All four slots are submitted: no fifth lend
  Slot none;
  TEST_ASSERT(r.ep.AcquireSlot(none) != ErrorCode::OK);
  // 逐个完成：每完成一个，还在排队的下一个就交下去；空出来的槽再提交，排在最后
  // One by one: each completion hands down the next still queued; the freed slot is
  // submitted again and goes to the end
  for (size_t i = 0U; i < 4U; ++i)
  {
    r.Complete(10U * (i + 1U));
    TEST_ASSERT(r.rec.seen.size() == i + 1U && r.rec.At(i).actual == 10U * (i + 1U));
    TEST_ASSERT(r.rec.At(i).result == ErrorCode::OK);
    r.Send(100U + i);
    TEST_ASSERT(r.ep.ahead == static_cast<int>(on_hardware) - 1);
  }
  for (size_t i = 0U; i < 4U; ++i)
  {
    r.Complete(100U + i);
    TEST_ASSERT(r.rec.At(4U + i).actual == 100U + i);
  }
  TEST_ASSERT(r.rec.seen.size() == 8U && !r.ep.busy && r.ep.ahead == 0);
  TEST_ASSERT(r.ep.Count(Hw::START) == 8U);
  for (size_t i = 0U; i < 4U; ++i)
  {
    TEST_ASSERT(r.ep.Start(4U + i).length == 100U + i);
  }
}

/// 接收：四个槽都挂上，各自完成、各借出自己的槽 / Receives: all four slots armed, each
/// completes and lends its own slot
void CheckManyAheadOut()
{
  Rig r(EPNumber::EP1, Direction::OUT, Type::BULK, false, 2048U, 4U);
  AllowAhead(r, Direction::OUT, 4U);
  for (size_t i = 0U; i < 4U; ++i)
  {
    TEST_ASSERT(r.ep.Receive(64U, false) == ErrorCode::OK);
  }
  TEST_ASSERT(r.ep.Receive(64U, false) == ErrorCode::BUSY);
  TEST_ASSERT(r.ep.Count(Hw::START) == 4U && r.ep.ahead == 3);
  for (size_t i = 0U; i < 4U; ++i)
  {
    r.Complete(5U + i);
    TEST_ASSERT(r.rec.seen.size() == i + 1U && r.rec.At(i).actual == 5U + i);
    TEST_ASSERT(r.rec.At(i).slot.Valid());
    TEST_ASSERT(r.rec.At(i).slot.data.addr_ == r.ep.Start(i).data);
  }
  // 槽还借在回调的接收者手里：交回之前不能再挂 / The slots are still lent to the
  // receiver of the callbacks: no receive before they are handed back
  TEST_ASSERT(r.ep.Receive(64U, false) == ErrorCode::BUSY);
  for (size_t i = 0U; i < 4U; ++i)
  {
    TEST_ASSERT(r.ep.ReleaseSlot(r.rec.At(i).slot) == ErrorCode::OK);
    TEST_ASSERT(r.ep.Receive(64U, false) == ErrorCode::OK);
  }
  TEST_ASSERT(r.ep.Count(Hw::START) == 8U && r.ep.ahead == 3);
}

/// 欠着结尾零长度包的请求后面不提前交，哪怕它本身是提前交下去的
/// Nothing goes down ahead behind a request that owes its closing zero-length packet,
/// even one that went down ahead itself
void CheckManyAheadZlp()
{
  Rig r(EPNumber::EP1, Direction::IN, Type::BULK, false, 2048U, 4U);
  AllowAhead(r, Direction::IN, 4U);
  r.Send(10U);
  r.Send(64U, true);
  r.Send(20U);
  r.Send(30U);
  TEST_ASSERT(r.ep.Count(Hw::START) == 2U && r.ep.ahead == 1);
  r.Complete(10U);
  // 欠零长度包的那个成了最早的：后面的仍然不交 / The one owing the packet is the
  // earliest now: what is behind it still stays
  TEST_ASSERT(r.rec.seen.size() == 1U && r.ep.Count(Hw::START) == 2U);
  r.Complete(64U);
  TEST_ASSERT(r.rec.seen.size() == 1U && r.ep.Count(Hw::START) == 3U);
  TEST_ASSERT(r.ep.Start(2).length == 0U && r.ep.Start(2).data == r.ep.Start(1).data);
  r.Complete(0U);
  TEST_ASSERT(r.rec.seen.size() == 2U && r.rec.At(1).actual == 64U);
  // 零长度包发完：剩下两个一起交下去 / The packet is out: the two left go down together
  TEST_ASSERT(r.ep.Count(Hw::START) == 5U && r.ep.ahead == 1);
  TEST_ASSERT(r.ep.Start(3).length == 20U && r.ep.Start(4).length == 30U);
  r.Complete(20U);
  r.Complete(30U);
  TEST_ASSERT(r.rec.seen.size() == 4U && r.rec.At(3).actual == 30U);
}

/// 第三个请求启动失败：后面的不再提前交；它的 FAILED 回调排在前两个之后，然后排队的
/// 请求照常启动
/// The third request fails to start: nothing more goes down ahead; its FAILED callback
/// comes after the first two, then the queued request starts as usual
void CheckManyAheadStartFails()
{
  Rig r(EPNumber::EP1, Direction::IN, Type::BULK, false, 2048U, 4U);
  AllowAhead(r, Direction::IN, 4U);
  r.ep.start_results = {ErrorCode::OK, ErrorCode::OK, ErrorCode::FAILED};
  r.Send(10U);
  r.Send(20U);
  r.Send(30U);
  r.Send(40U);
  TEST_ASSERT(r.ep.Count(Hw::START) == 3U && r.ep.ahead == 1 && r.rec.seen.empty());
  r.Complete(10U);
  TEST_ASSERT(r.rec.seen.size() == 1U && r.ep.Count(Hw::START) == 3U);
  r.Complete(20U);
  TEST_ASSERT(r.rec.seen.size() == 3U && r.rec.At(1).result == ErrorCode::OK &&
              r.rec.At(1).actual == 20U && r.rec.At(2).result == ErrorCode::FAILED);
  TEST_ASSERT(r.ep.Count(Hw::START) == 4U && r.ep.Start(3).length == 40U);
  r.Complete(40U);
  TEST_ASSERT(r.rec.seen.size() == 4U && r.rec.At(3).actual == 40U && !r.ep.busy);

  // 停止时：只对硬件上的两个调用 StopHardware，没能进硬件的那个以 FAILED 结束，排在最后
  // 的那个还在排队，以 ABORTED 结束
  // At a stop: StopHardware is called for the two on the hardware only, the one that did
  // not reach it ends with FAILED and the one still queued with ABORTED
  r.ep.calls.clear();
  r.rec.seen.clear();
  r.ep.start_results = {ErrorCode::OK, ErrorCode::OK, ErrorCode::FAILED};
  r.ep.stop_outcomes = {{ErrorCode::ABORTED, 3U}, {ErrorCode::ABORTED, 0U}};
  r.Send(10U);
  r.Send(20U);
  r.Send(30U);
  r.Send(40U);
  End(r.dev, r.ep, Ender::FLUSH);
  TEST_ASSERT(r.ep.Count(Hw::START) == 3U && r.ep.Count(Hw::STOP) == 2U);
  TEST_ASSERT(r.rec.seen.size() == 4U && r.rec.At(0).result == ErrorCode::ABORTED &&
              r.rec.At(0).actual == 3U && r.rec.At(1).result == ErrorCode::ABORTED &&
              r.rec.At(2).result == ErrorCode::FAILED &&
              r.rec.At(3).result == ErrorCode::ABORTED);
  // 四个槽都回来了 / All four slots are back
  for (size_t i = 0U; i < 4U; ++i)
  {
    r.Send(1U);
  }
  TEST_ASSERT(!r.ep.busy || r.ep.ahead == 3);
}

/// 停止：硬件上有三个请求时 StopHardware 调用三次，各取一个结果，回调按先后
/// Stop: with three requests on the hardware StopHardware is called three times, once
/// for each outcome, and the callbacks come in order
void CheckManyAheadStopNow(Ender how)
{
  Rig r(EPNumber::EP1, Direction::IN, Type::BULK, false, 2048U, 4U);
  AllowAhead(r, Direction::IN, 3U);
  r.Send(10U);
  r.Send(20U);
  r.Send(128U, true);
  r.Send(40U);
  TEST_ASSERT(r.ep.Count(Hw::START) == 3U);
  r.ep.stop_outcomes = {
      {ErrorCode::OK, 10U}, {ErrorCode::ABORTED, 7U}, {ErrorCode::OK, 128U}};
  End(r.dev, r.ep, how);
  TEST_ASSERT(r.ep.Count(Hw::STOP) == 3U && !r.ep.busy && r.ep.ahead == 0);
  TEST_ASSERT(r.rec.seen.size() == 4U);
  TEST_ASSERT(r.rec.At(0).result == ErrorCode::OK && r.rec.At(0).actual == 10U);
  TEST_ASSERT(r.rec.At(1).result == ErrorCode::ABORTED && r.rec.At(1).actual == 7U);
  // 数据做完但零长度包没发：ABORTED、actual 为数据长度 / Data done, closing packet
  // unsent: ABORTED with the data length
  TEST_ASSERT(r.rec.At(2).result == ErrorCode::ABORTED && r.rec.At(2).actual == 128U);
  // 还在排队的那个 / The one still queued
  TEST_ASSERT(r.rec.At(3).result == ErrorCode::ABORTED && r.rec.At(3).actual == 0U);
  if (how == Ender::CLOSE)
  {
    TEST_ASSERT(r.ep.calls.back().function == Hw::CLOSE);
    return;
  }
  if (how == Ender::STALL)
  {
    TEST_ASSERT(r.ep.calls.back().function == Hw::STALL);
    TEST_ASSERT(r.ep.ClearStall(false) == ErrorCode::OK);
  }
  for (size_t i = 0U; i < 4U; ++i)
  {
    r.Send(1U);
  }
  TEST_ASSERT(r.ep.busy && r.ep.ahead == 2);
}

/// 停止挂起：三个请求当场各回调一次；后端之后按先后各报告一次，第三次才是停止结束
/// A pending stop: the three requests are called back at once; the backend later reports
/// once for each in order, and only the third report ends the stop
void CheckManyAheadStopPending(bool direct)
{
  Rig r(EPNumber::EP1, Direction::IN, Type::BULK, false, 2048U, 4U);
  AllowAhead(r, Direction::IN, 3U, direct);
  r.Send(10U);
  r.Send(20U);
  r.Send(30U);
  r.ep.stop_outcomes = {{ErrorCode::PENDING, 0U}};
  End(r.dev, r.ep, Ender::FLUSH);
  TEST_ASSERT(r.ep.Count(Hw::STOP) == 1U && r.ep.stop_pending);
  TEST_ASSERT(r.rec.seen.size() == 3U);
  for (size_t i = 0U; i < 3U; ++i)
  {
    TEST_ASSERT(r.rec.At(i).result == ErrorCode::ABORTED && r.rec.At(i).actual == 0U);
  }
  // 硬件直接读写槽时三个槽都留到停止结束，只有第四个能借；否则四个都能借
  // With hardware that accesses the slots the three stay until the stop ends and only
  // the fourth can be lent; otherwise all four can
  Slot slots[4];
  size_t lent = 0U;
  while (lent < 4U && r.ep.AcquireSlot(slots[lent]) == ErrorCode::OK)
  {
    ++lent;
  }
  TEST_ASSERT(lent == (direct ? 1U : 4U));
  TEST_ASSERT(r.ep.Submit(slots[0], 5U, false, false) == ErrorCode::OK);
  for (size_t i = 1U; i < lent; ++i)
  {
    TEST_ASSERT(r.ep.ReleaseSlot(slots[i]) == ErrorCode::OK);
  }
  // 前两次报告之后停止还没有结束：什么都不启动；每次报告放回一个留着的槽
  // After the first two reports the stop has not ended: nothing starts; each report
  // gives back one slot that was kept
  for (size_t i = 0U; i < 2U; ++i)
  {
    r.Complete(0U, ErrorCode::ABORTED);
    TEST_ASSERT(r.ep.stop_pending && r.ep.Count(Hw::START) == 3U);
    Slot again;
    TEST_ASSERT(r.ep.AcquireSlot(again) == ErrorCode::OK);
    TEST_ASSERT(r.ep.ReleaseSlot(again) == ErrorCode::OK);
  }
  r.Complete(0U, ErrorCode::ABORTED);
  TEST_ASSERT(!r.ep.stop_pending && r.rec.seen.size() == 3U);
  TEST_ASSERT(r.ep.Count(Hw::START) == 4U && r.ep.Start(3).length == 5U);
  r.Complete(5U);
  TEST_ASSERT(r.rec.seen.size() == 4U && r.rec.At(3).actual == 5U);
  for (size_t i = 0U; i < 4U; ++i)
  {
    r.Send(1U);
  }
  TEST_ASSERT(r.ep.busy && r.ep.ahead == 2);
}

/// 停止挂起时最后一个请求当初没能进硬件：后端只为硬件上的两个各报告一次，没进硬件的
/// 那个当场以 FAILED 结束、槽当场放回
/// A pending stop with a last request that did not reach the hardware: the backend
/// reports once for each of the two on the hardware, and the one that never reached it
/// ends with FAILED and gets its slot back at once
void CheckManyAheadStopPendingFailed()
{
  Rig r(EPNumber::EP1, Direction::IN, Type::BULK, false, 2048U, 4U);
  AllowAhead(r, Direction::IN, 4U);
  r.ep.start_results = {ErrorCode::OK, ErrorCode::OK, ErrorCode::FAILED};
  r.Send(10U);
  r.Send(20U);
  r.Send(30U);
  TEST_ASSERT(r.ep.Count(Hw::START) == 3U && r.ep.ahead == 1);
  r.ep.stop_outcomes = {{ErrorCode::PENDING, 0U}};
  End(r.dev, r.ep, Ender::FLUSH);
  TEST_ASSERT(r.ep.Count(Hw::STOP) == 1U && r.ep.stop_pending);
  TEST_ASSERT(r.rec.seen.size() == 3U && r.rec.At(0).result == ErrorCode::ABORTED &&
              r.rec.At(1).result == ErrorCode::ABORTED &&
              r.rec.At(2).result == ErrorCode::FAILED);
  // 留着的是硬件上那两个的槽：能借到的是另外两个 / The slots of the two on the hardware
  // are kept: the other two can be lent
  Slot slots[4];
  size_t lent = 0U;
  while (lent < 4U && r.ep.AcquireSlot(slots[lent]) == ErrorCode::OK)
  {
    ++lent;
  }
  TEST_ASSERT(lent == 2U);
  for (size_t i = 0U; i < lent; ++i)
  {
    TEST_ASSERT(r.ep.ReleaseSlot(slots[i]) == ErrorCode::OK);
  }
  r.Complete(0U, ErrorCode::ABORTED);
  TEST_ASSERT(r.ep.stop_pending);
  r.Complete(0U, ErrorCode::ABORTED);
  TEST_ASSERT(!r.ep.stop_pending && r.rec.seen.size() == 3U);
  for (size_t i = 0U; i < 4U; ++i)
  {
    r.Send(1U);
  }
  TEST_ASSERT(r.ep.Count(Hw::START) == 7U && r.ep.busy && r.ep.ahead == 3);
}

/// 停止超时：只报一次；控制器复位之后四个槽都回来 / A stop that times out is reported
/// once; after the controller reset all four slots are back
void CheckManyAheadStopTimeout()
{
  Rig r(EPNumber::EP1, Direction::IN, Type::BULK, false, 2048U, 4U);
  AllowAhead(r, Direction::IN, 4U);
  for (size_t i = 0U; i < 4U; ++i)
  {
    r.Send(10U);
  }
  r.ep.stop_outcomes = {{ErrorCode::PENDING, 0U}};
  End(r.dev, r.ep, Ender::FLUSH);
  r.Complete(0U, ErrorCode::ABORTED);
  r.Complete(0U, ErrorCode::TIMEOUT);
  TEST_ASSERT(r.dev.StopFailures() == 1U);
  r.dev.AsOwner([&](bool in_isr) { r.dev.ResetAndCloseAll(in_isr); });
  TEST_ASSERT(r.ep.GetState() == State::DISABLED && !r.ep.busy && r.ep.ahead == 0);
  TEST_ASSERT(Configure(r.dev, r.ep, Direction::IN, Type::BULK, 64U) == ErrorCode::OK);
  r.ep.calls.clear();
  for (size_t i = 0U; i < 4U; ++i)
  {
    r.Send(1U);
  }
  TEST_ASSERT(r.ep.Count(Hw::START) == 4U);
}

/// 最多的槽数：八个请求同时在硬件上 / The largest slot count: eight requests on the
/// hardware at once
void CheckMostSlots()
{
  alignas(64) static uint8_t memory[8U * 64U];
  FakeDevice dev(false);
  FakeEndpoint ep(EPNumber::EP1, Direction::IN, {memory, sizeof(memory)},
                  Endpoint::MAX_SLOTS);
  Recorder rec(ep);
  dev.Add(ep);
  SetGate(dev, Gate::ALL);
  ep.limits.max_requests = 200U;
  TEST_ASSERT(Configure(dev, ep, Direction::IN, Type::BULK, 64U) == ErrorCode::OK);
  TEST_ASSERT(ep.MaxTransferSize() == 64U);
  for (size_t i = 0U; i < Endpoint::MAX_SLOTS; ++i)
  {
    Slot slot;
    TEST_ASSERT(ep.AcquireSlot(slot) == ErrorCode::OK);
    TEST_ASSERT(ep.Submit(slot, i + 1U, false, false) == ErrorCode::OK);
  }
  Slot none;
  TEST_ASSERT(ep.AcquireSlot(none) == ErrorCode::BUSY);
  TEST_ASSERT(ep.Count(Hw::START) == Endpoint::MAX_SLOTS && ep.ahead == 7);
  ep.stop_outcomes.assign(Endpoint::MAX_SLOTS, {ErrorCode::ABORTED, 0U});
  End(dev, ep, Ender::CLOSE);
  TEST_ASSERT(ep.Count(Hw::STOP) == Endpoint::MAX_SLOTS && !ep.busy && ep.ahead == 0);
  TEST_ASSERT(rec.seen.size() == Endpoint::MAX_SLOTS);
}

}  // namespace

void test_usb_ep()
{
  CheckConfigure();
  for (const uint8_t limit :
       {uint8_t{1U}, uint8_t{2U}, uint8_t{3U}, uint8_t{4U}, uint8_t{9U}})
  {
    CheckManyAheadOrder(limit);
  }
  CheckManyAheadOut();
  CheckManyAheadZlp();
  CheckManyAheadStartFails();
  for (const Ender how : {Ender::FLUSH, Ender::STALL, Ender::CLOSE})
  {
    CheckManyAheadStopNow(how);
  }
  CheckManyAheadStopPending(true);
  CheckManyAheadStopPending(false);
  CheckManyAheadStopPendingFailed();
  CheckManyAheadStopTimeout();
  CheckMostSlots();
  CheckAheadOrder();
  CheckAheadOut();
  CheckAheadZlp();
  CheckAheadStartFails();
  for (const Ender how : {Ender::FLUSH, Ender::STALL, Ender::CLOSE})
  {
    CheckAheadStopNow(how, Direction::IN);
    CheckAheadStopNow(how, Direction::OUT);
  }
  CheckAheadStopPending(true);
  CheckAheadStopPending(false);
  CheckAheadStopTimeout();
  CheckInBasic();
  CheckInOrder(false);
  CheckInOrder(true);
  CheckZeroLength();
  CheckInReturnCodes();
  CheckInWhileStalled();
  CheckOutBasic();
  CheckOutLengths();
  CheckOutStale();
  CheckZlpSent(false);
  CheckZlpSent(true);
  CheckZlpNotNeeded();
  CheckZlpWhileStopping(Ender::FLUSH);
  CheckZlpWhileStopping(Ender::STALL);
  CheckZlpWhileStopping(Ender::CLOSE);
  CheckZlpOnWireStopped(ErrorCode::OK);
  CheckZlpOnWireStopped(ErrorCode::ABORTED);
  CheckZlpOwedDataFails();
  CheckZlpStartFails();
  CheckCloseIdle();
  CheckCloseInFlight(ErrorCode::ABORTED, 4U);
  CheckCloseInFlight(ErrorCode::OK, 10U);
  CheckFlush();
  CheckOutWhileStopping();
  CheckStallIdle();
  CheckStallInFlight();
  CheckClearStallIdle();
  for (const bool direct : {true, false})
  {
    CheckStopFails(Ender::FLUSH, direct);
    CheckStopFails(Ender::STALL, direct);
    CheckStopFails(Ender::CLOSE, direct);
  }
  CheckHeldStart();
  CheckResubmitInCallback(false);
  CheckResubmitInCallback(true);
  CheckIgnoredReport();
  CheckFailedStart();
  CheckCloseInQueuedCallback();
  CheckConfigureInCloseCallback(false);
  CheckConfigureInCloseCallback(true);
  CheckEndInCompletionCallback(Ender::FLUSH);
  CheckEndInCompletionCallback(Ender::STALL);
  CheckEndInCompletionCallback(Ender::CLOSE);
  CheckStallInStoppedCallback();
  CheckLifecycleOnOtherEndpoint();
  CheckHardwareErrors();
  CheckGate();
  CheckInterruptWins();
  CheckInterruptWhileThreadOwns();
  CheckThreadEntry(false);
  CheckThreadEntry(true);
  CheckRegister();
  CheckPassOrder();
  CheckRepeatedDeviceBit();
  CheckThreads(false);
  CheckThreads(true);
  CheckReceiveAgainstClose(false);
  CheckReceiveAgainstClose(true);
  for (const bool direct : {true, false})
  {
    CheckRowsWithoutStop(direct);
    CheckRowInFlight(direct);
    CheckRowsWithPendingStop(direct);
    CheckRowsWithFailedStop(direct);
    CheckDeferredChains(direct);
    CheckOneSlotRebind(direct);
    CheckHardwareConfig(direct);
    CheckLateAfterStop(direct);
    CheckOneCallbackPerRequest(direct);
    CheckResetWithDeferredSteps(direct);
    CheckTimeoutSamePass(direct);
  }
  CheckTickStops();
  CheckTickWhileOwnerServes();
  CheckPollOnlyPass();
  CheckContractAsserts();
  for (const bool direct : {true, false})
  {
    CheckStopsAgainstThreads(direct, false);
    CheckStopsAgainstThreads(direct, true);
  }
}
