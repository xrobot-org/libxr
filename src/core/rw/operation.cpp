#include "operation.hpp"

namespace LibXR
{

ErrorCode AsyncBlockWait::Wait(uint32_t timeout)
{
  DEV_ASSERT(sem_ != nullptr);
  auto wait_ans = sem_->Wait(timeout);
  if (wait_ans == ErrorCode::OK)
  {
    DEV_ASSERT(state_.load(std::memory_order_acquire) == State::CLAIMED);
    state_.store(State::IDLE, std::memory_order_release);
    return result_;
  }

  State expected = State::PENDING;
  if (state_.compare_exchange_strong(expected, State::DETACHED, std::memory_order_acq_rel,
                                     std::memory_order_acquire))
  {
    return ErrorCode::TIMEOUT;
  }

  DEV_ASSERT(expected == State::CLAIMED || expected == State::DETACHED ||
             expected == State::IDLE);
  if (expected == State::DETACHED)
  {
    state_.store(State::IDLE, std::memory_order_release);
    return ErrorCode::TIMEOUT;
  }
  if (expected == State::IDLE)
  {
    return ErrorCode::TIMEOUT;
  }

  Detail::WaitClaimedNotification(*sem_);
  state_.store(State::IDLE, std::memory_order_release);
  return result_;
}

}  // namespace LibXR
