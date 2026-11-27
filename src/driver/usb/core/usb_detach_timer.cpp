#include "usb_detach_timer.hpp"

#include "libxr_assert.hpp"
#include "timebase.hpp"

namespace LibXR::USB
{

namespace
{
/// defer 失败后重试的间隔（毫秒）/ Retry interval after a failed defer (ms)
constexpr uint32_t DEFER_RETRY_MS = 1U;
/// 没有节拍的检测：Timebase 过了 detach_ms 的这么多倍而节拍没动 / Missing-tick check:
/// this many times detach_ms passed on Timebase while the tick did not move
constexpr uint32_t NO_TICK_FACTOR = 3U;

uint32_t NowMs() { return static_cast<uint32_t>(Timebase::GetMilliseconds()); }
}  // namespace

DetachTimer::DetachTimer(DeviceService& service, uint32_t detach_ms)
    : DetachTimer(service, detach_ms, nullptr, nullptr)
{
}

DetachTimer::DetachTimer(DeviceService& service, uint32_t detach_ms, DeferFun defer,
                         void* arg)
    : service_(service),
      detach_ms_(detach_ms),
      defer_(defer),
      defer_arg_(arg),
      condition_(Check, Done, this)
{
  ASSERT(detach_ms > 0U && detach_ms <= DriverAsyncCondition::MAX_WAIT_MS);
  // 没有节拍源时断开永远不结束：在构造时就失败，而不是之后悄悄断开。
  // Without a tick source a detach never ends: fail at construction rather than stay
  // detached silently later.
  REQUIRE(DriverAsyncCondition::HasTickSource());
}

DetachTimer::HaltResult DetachTimer::OnHalted(bool attach_wanted)
{
  const State counting = attach_wanted ? State::DETACHING_WANTED : State::DETACHING;
  State state = state_.load(std::memory_order_acquire);
  while (true)
  {
    switch (state)
    {
      case State::ATTACHED:
      {
        // 先记下检测用的起点，再进入计时状态；计时对象此时一定空闲（只在这里启动）。
        // Record the check's starting point first, then enter counting; the condition
        // is idle here for sure (it is started only here).
        tick_at_halt_ = DriverAsyncCondition::TickCount();
        ms_at_halt_ = NowMs();
        if (!state_.compare_exchange_weak(state, counting, std::memory_order_acq_rel,
                                          std::memory_order_acquire))
        {
          continue;
        }
        const ErrorCode started = condition_.Start(detach_ms_, detach_ms_);
        // BUSY 只可能是 defer 的重试还在等：重试的结果函数看到计时状态，会为这次断开
        // 重新开始计时。
        // BUSY can only be a pending retry of defer: its result function sees the
        // counting state and starts counting for this detach again.
        ASSERT(started == ErrorCode::OK || started == ErrorCode::BUSY);
        UNUSED(started);
        return HaltResult::COUNTING;
      }
      case State::DETACHING:
      case State::DETACHING_WANTED:
        // 总线从第一次断开起一直断着：沿用原来的计时，只改"要接上"。
        // The bus has stayed detached since the first detach: keep the running count,
        // change "attach wanted" only.
        if (state_.compare_exchange_weak(state, counting, std::memory_order_acq_rel,
                                         std::memory_order_acquire))
        {
          return HaltResult::COUNTING;
        }
        break;
      case State::DETACHED:
      case State::ATTACH_DUE:
        // 时间早已够，总线也一直断着。ATTACH_DUE 时已发布的那一轮之后 TakeAttach 返回
        // false。
        // The time passed long ago and the bus stayed detached. From ATTACH_DUE the pass
        // already published finds TakeAttach false afterwards.
        if (state_.compare_exchange_weak(
                state, attach_wanted ? State::ATTACHED : State::DETACHED,
                std::memory_order_acq_rel, std::memory_order_acquire))
        {
          return attach_wanted ? HaltResult::ATTACH_NOW : HaltResult::DETACHED;
        }
        break;
    }
  }
}

bool DetachTimer::WantAttach()
{
  State state = state_.load(std::memory_order_acquire);
  while (true)
  {
    switch (state)
    {
      case State::ATTACHED:
        return true;
      case State::DETACHED:
        if (state_.compare_exchange_weak(state, State::ATTACHED,
                                         std::memory_order_acq_rel,
                                         std::memory_order_acquire))
        {
          return true;
        }
        break;
      case State::DETACHING:
      case State::DETACHING_WANTED:
      {
        // 没有节拍的检测：Timebase 显示早该结束而节拍一次也没来。时间戳回退只会让差值
        // 变小，只会漏报。
        // Missing-tick check: Timebase shows the count should have ended long ago while
        // not one tick came. A timestamp stepping back only shrinks the difference and
        // can only make this miss.
        const uint32_t elapsed = NowMs() - ms_at_halt_;
        ASSERT(elapsed < NO_TICK_FACTOR * detach_ms_ ||
               DriverAsyncCondition::TickCount() != tick_at_halt_);
        UNUSED(elapsed);
        if (state == State::DETACHING_WANTED ||
            state_.compare_exchange_weak(state, State::DETACHING_WANTED,
                                         std::memory_order_acq_rel,
                                         std::memory_order_acquire))
        {
          return false;
        }
        break;
      }
      case State::ATTACH_DUE:
        return false;
    }
  }
}

bool DetachTimer::TakeAttach()
{
  State expected = State::ATTACH_DUE;
  return state_.compare_exchange_strong(
      expected, State::ATTACHED, std::memory_order_acq_rel, std::memory_order_relaxed);
}

bool DetachTimer::Detached() const
{
  return state_.load(std::memory_order_acquire) != State::ATTACHED;
}

bool DetachTimer::Check(void* arg)
{
  UNUSED(arg);
  return true;
}

void DetachTimer::Done(void* arg, ErrorCode result)
{
  UNUSED(result);
  auto* self = static_cast<DetachTimer*>(arg);
  const bool retry = self->retry_;
  self->retry_ = false;
  State state = self->state_.load(std::memory_order_acquire);
  while (true)
  {
    switch (state)
    {
      case State::DETACHING:
      case State::DETACHING_WANTED:
        if (retry)
        {
          // 重试还在等时又开始了一次断开：为它重新计时，计时只会因此变长。
          // A detach started while a retry was pending: count again for it, which only
          // makes the count longer.
          (void)self->condition_.Start(self->detach_ms_, self->detach_ms_);
          return;
        }
        if (state == State::DETACHING)
        {
          if (self->state_.compare_exchange_weak(state, State::DETACHED,
                                                 std::memory_order_acq_rel,
                                                 std::memory_order_acquire))
          {
            return;
          }
          break;
        }
        if (!self->state_.compare_exchange_weak(state, State::ATTACH_DUE,
                                                std::memory_order_acq_rel,
                                                std::memory_order_acquire))
        {
          break;
        }
        state = State::ATTACH_DUE;
        [[fallthrough]];
      case State::ATTACH_DUE:
        if (self->defer_ == nullptr)
        {
          self->service_.RequestPoll(true);
        }
        else if (!self->defer_(self->defer_arg_))
        {
          self->retry_ = true;
          (void)self->condition_.Start(DEFER_RETRY_MS, DEFER_RETRY_MS);
        }
        return;
      case State::ATTACHED:
      case State::DETACHED:
        return;
    }
  }
}

}  // namespace LibXR::USB
