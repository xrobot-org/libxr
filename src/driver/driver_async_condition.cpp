#include "driver_async_condition.hpp"

#include "libxr_assert.hpp"

using namespace LibXR;

DriverAsyncCondition::DriverAsyncCondition(CheckFun check, DoneFun done, void* arg)
    : check_(check), done_(done), arg_(arg)
{
  ASSERT(check != nullptr && done != nullptr);
  // 只在这里挂入，之后不移动：节拍遍历链表时不加锁。
  // Linked here once and never moved: the tick walks the list without a lock.
  DriverAsyncCondition* head = head_.load(std::memory_order_relaxed);
  do
  {
    next_ = head;
  } while (!head_.compare_exchange_weak(head, this, std::memory_order_release,
                                        std::memory_order_relaxed));
}

DriverAsyncCondition::~DriverAsyncCondition()
{
  // 只供主机测试，此时没有节拍在运行。
  // Host tests only, while no tick runs.
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

ErrorCode DriverAsyncCondition::Start(uint32_t min_ms, uint32_t max_ms)
{
  if (max_ms < min_ms || max_ms > MAX_WAIT_MS)
  {
    return ErrorCode::ARG_ERR;
  }
  uint32_t expected = MakeState(Phase::IDLE, 0U);
  if (!state_.compare_exchange_strong(expected, MakeState(Phase::SETUP, 0U),
                                      std::memory_order_acquire,
                                      std::memory_order_relaxed))
  {
    return ErrorCode::BUSY;
  }
  // 节拍看到 SETUP 时跳过本对象；上下限由下面的 release 写入一并发布。
  // The tick skips this object while it sees SETUP; the release store below publishes
  // the limits.
  min_ms_ = min_ms;
  max_ms_ = max_ms;
  state_.store(MakeState(Phase::WAITING, 0U), std::memory_order_release);
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
  if (GetPhase(state) != Phase::WAITING)
  {
    return;
  }
  uint64_t counted = static_cast<uint64_t>(GetCount(state)) + elapsed_ms;
  if (counted > MAX_COUNT)
  {
    counted = MAX_COUNT;
  }
  // 第一个节拍可能紧跟在 Start() 之后，却记满一个间隔，所以上下限各多等一个间隔。
  // The first tick may come right after Start() and still counts a whole interval, so
  // both limits wait one interval more.
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
    // 进行中的等待只由节拍修改。
    // Only the tick changes a running wait.
    (void)state_.compare_exchange_strong(
        state, MakeState(Phase::WAITING, static_cast<uint32_t>(counted)),
        std::memory_order_acq_rel, std::memory_order_relaxed);
    return;
  }
  // 先回到 IDLE 再调用 done：done 可以对本对象再次调用 Start()。
  // Back to IDLE before done: done may call Start() on this object again.
  if (state_.compare_exchange_strong(state, MakeState(Phase::IDLE, 0U),
                                     std::memory_order_acq_rel,
                                     std::memory_order_relaxed))
  {
    done_(arg_, result);
  }
}
