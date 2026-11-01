#include "driver_async_condition.hpp"

#include "libxr_assert.hpp"

using namespace LibXR;

DriverAsyncCondition::DriverAsyncCondition(Check check, Done done, void* arg)
    : check_(check), done_(done), arg_(arg)
{
  ASSERT(check != nullptr && done != nullptr);
  // Linked once and never moved: the tick walks the list without a lock.
  DriverAsyncCondition* head = head_.load(std::memory_order_relaxed);
  do
  {
    next_ = head;
  } while (!head_.compare_exchange_weak(head, this, std::memory_order_release,
                                        std::memory_order_relaxed));
}

DriverAsyncCondition::~DriverAsyncCondition()
{
  // Drivers are never destroyed; tests destroy theirs while no tick runs.
  DriverAsyncCondition* node = head_.load(std::memory_order_acquire);
  if (node == this)
  {
    head_.store(next_, std::memory_order_release);
    return;
  }
  for (; node != nullptr; node = node->next_)
  {
    if (node->next_ == this)
    {
      node->next_ = next_;
      return;
    }
  }
}

ErrorCode DriverAsyncCondition::Wait(uint32_t min_ms, uint32_t max_ms)
{
  if (max_ms < min_ms)
  {
    return ErrorCode::ARG_ERR;
  }
  uint32_t expected = IDLE;
  if (!state_.compare_exchange_strong(expected, SETUP, std::memory_order_acquire,
                                      std::memory_order_relaxed))
  {
    return ErrorCode::BUSY;
  }
  min_ms_ = min_ms;
  max_ms_ = max_ms;
  state_.store(WAITING, std::memory_order_release);
  return ErrorCode::OK;
}

void DriverAsyncCondition::OnTick(uint32_t elapsed_ms)
{
  for (DriverAsyncCondition* node = head_.load(std::memory_order_acquire);
       node != nullptr; node = node->next_)
  {
    node->Tick(elapsed_ms);
  }
}

void DriverAsyncCondition::Tick(uint32_t elapsed_ms)
{
  uint32_t state = state_.load(std::memory_order_acquire);
  if ((state & PHASE_MASK) != WAITING)
  {
    return;
  }
  uint64_t counted = static_cast<uint64_t>(state & COUNT_MASK) + elapsed_ms;
  if (counted > COUNT_MASK)
  {
    counted = COUNT_MASK;
  }
  // The first tick may come right after Wait(): one more tick keeps "at least min_ms".
  bool finished = false;
  ErrorCode result = ErrorCode::OK;
  if (counted >= static_cast<uint64_t>(min_ms_) + elapsed_ms)
  {
    if (check_(arg_))
    {
      finished = true;
    }
    else if (counted >= static_cast<uint64_t>(max_ms_) + elapsed_ms)
    {
      finished = true;
      result = ErrorCode::TIMEOUT;
    }
  }
  if (!finished)
  {
    // Only the tick changes a running wait.
    (void)state_.compare_exchange_strong(state, WAITING | static_cast<uint32_t>(counted),
                                         std::memory_order_acq_rel,
                                         std::memory_order_relaxed);
    return;
  }
  // Idle before done: done may Wait() again on this object.
  if (state_.compare_exchange_strong(state, IDLE, std::memory_order_acq_rel,
                                     std::memory_order_relaxed))
  {
    done_(arg_, result);
  }
}
