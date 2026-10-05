#include "read_port.hpp"

#include <new>

using namespace LibXR;

ReadPort::ReadPort(size_t buffer_size)
    : queue_data_(buffer_size != 0U ? new (std::align_val_t(LibXR::CONCURRENCY_ALIGNMENT))
                                          SPSCQueue<uint8_t>(buffer_size)
                                    : nullptr)
{
}

ReadPort::ReadQueue ReadPort::GetReadQueue(bool in_isr)
{
  ASSERT_FROM_CALLBACK(queue_data_ != nullptr, in_isr);
  return ReadQueue(*this, in_isr);
}

ReadPort::ReadQueue::~ReadQueue() { ASSERT_FROM_CALLBACK(finished_, in_isr_); }

ErrorCode ReadPort::ReadQueue::PushBatch(const uint8_t* data, size_t size)
{
  ASSERT_FROM_CALLBACK(!finished_, in_isr_);
  if (size != 0U)
  {
    ASSERT_FROM_CALLBACK(data != nullptr, in_isr_);
  }

  const ErrorCode result = port_.queue_data_->PushBatch(data, size);
  if (result == ErrorCode::OK && size != 0U)
  {
    dirty_ = true;
  }
  return result;
}

size_t ReadPort::ReadQueue::EmptySize() const { return port_.queue_data_->EmptySize(); }

size_t ReadPort::ReadQueue::Capacity() const { return port_.queue_data_->MaxSize(); }

void ReadPort::ReadQueue::Publish()
{
  ASSERT_FROM_CALLBACK(!finished_, in_isr_);
  finished_ = true;
  if (dirty_)
  {
    port_.PublishProduced(in_isr_);
  }
}

size_t ReadPort::EmptySize() const
{
  return queue_data_ == nullptr ? 0U : queue_data_->EmptySize();
}

size_t ReadPort::Size() const
{
  return queue_data_ == nullptr ? 0U : queue_data_->Size();
}

size_t ReadPort::Capacity() const
{
  return queue_data_ == nullptr ? 0U : queue_data_->MaxSize();
}

bool ReadPort::Readable() const { return queue_data_ != nullptr; }

bool ReadPort::TryClaimIdle()
{
  uint32_t observed = state_.load(std::memory_order_acquire);
  for (;;)
  {
    if (GetPhase(observed) != Phase::IDLE)
    {
      return false;
    }

    if (state_.compare_exchange_weak(observed, static_cast<uint32_t>(Phase::CLAIMED),
                                     std::memory_order_acq_rel,
                                     std::memory_order_acquire))
    {
      return true;
    }
  }
}

bool ReadPort::HasEnough(size_t available, size_t requested) const
{
  return requested == 0U ? available != 0U : available >= requested;
}

void ReadPort::ReleaseClaimed([[maybe_unused]] bool in_isr)
{
  uint32_t observed = state_.load(std::memory_order_acquire);
  for (;;)
  {
    DEV_ASSERT_FROM_CALLBACK(GetPhase(observed) == Phase::CLAIMED, in_isr);
    const uint32_t desired = WithPhase(observed, Phase::IDLE);
    if (state_.compare_exchange_weak(observed, desired, std::memory_order_acq_rel,
                                     std::memory_order_acquire))
    {
      return;
    }
  }
}

bool ReadPort::ClaimBlockCompletion()
{
  uint32_t observed = state_.load(std::memory_order_acquire);
  for (;;)
  {
    const Phase phase = GetPhase(observed);
    if (phase == Phase::BLOCK_CLAIMED)
    {
      return true;
    }
    if (phase != Phase::CLAIMED && phase != Phase::CLAIMED_WITH_WAITER)
    {
      return false;
    }

    const uint32_t desired = WithPhase(observed, Phase::BLOCK_CLAIMED);
    if (state_.compare_exchange_weak(observed, desired, std::memory_order_acq_rel,
                                     std::memory_order_acquire))
    {
      return true;
    }
  }
}

void ReadPort::ReleaseBlockCompletion([[maybe_unused]] bool in_isr)
{
  uint32_t observed = state_.load(std::memory_order_acquire);
  for (;;)
  {
    DEV_ASSERT_FROM_CALLBACK(GetPhase(observed) == Phase::BLOCK_CLAIMED, in_isr);
    const uint32_t desired = WithPhase(observed, Phase::IDLE);
    if (state_.compare_exchange_weak(observed, desired, std::memory_order_acq_rel,
                                     std::memory_order_acquire))
    {
      return;
    }
  }
}

void ReadPort::PublishProduced(bool in_isr)
{
  state_.fetch_or(EVENT_BIT, std::memory_order_acq_rel);
  ProcessPendingReads(in_isr);
}

void ReadPort::NotifyDataAvailable(bool in_isr)
{
  state_.fetch_or(EVENT_BIT, std::memory_order_acq_rel);
  ProcessPendingReads(in_isr);
}

ErrorCode ReadPort::operator()(RawData data, ReadOperation& op, bool in_isr)
{
  if (!Readable())
  {
    return ErrorCode::NOT_SUPPORT;
  }

  if (!TryClaimIdle())
  {
    return ErrorCode::BUSY;
  }

  const bool is_block = op.type == ReadOperation::OperationType::BLOCK;

  if (data.size_ != 0U)
  {
    ASSERT_FROM_CALLBACK(data.addr_ != nullptr, in_isr);
    if (data.size_ > Capacity())
    {
      ReleaseClaimed(in_isr);
      return ErrorCode::SIZE_ERR;
    }
  }

  if (HasEnough(queue_data_->Size(), data.size_))
  {
    if (data.size_ != 0U)
    {
      [[maybe_unused]] const auto pop_batch_result =
          queue_data_->PopBatch(reinterpret_cast<uint8_t*>(data.addr_), data.size_);
      DEV_ASSERT_FROM_CALLBACK(pop_batch_result == ErrorCode::OK, in_isr);
    }

    Request completed{data, op};
    ReleaseClaimed(in_isr);
    if (data.size_ != 0U)
    {
      OnReadQueueSpaceAvailable(in_isr);
    }
    if (completed.op.type != ReadOperation::OperationType::BLOCK)
    {
      completed.op.UpdateStatus(in_isr, ErrorCode::OK);
    }
    return ErrorCode::OK;
  }

  info_ = Request{data, op};
  info_.op.MarkAsRunning();

  for (;;)
  {
    uint32_t observed = state_.load(std::memory_order_acquire);
    DEV_ASSERT_FROM_CALLBACK(GetPhase(observed) == Phase::CLAIMED, in_isr);

    if (HasEvent(observed))
    {
      if (!state_.compare_exchange_weak(observed, static_cast<uint32_t>(Phase::CLAIMED),
                                        std::memory_order_acq_rel,
                                        std::memory_order_acquire))
      {
        continue;
      }

      if (HasEnough(queue_data_->Size(), data.size_))
      {
        if (info_.op.type == ReadOperation::OperationType::BLOCK)
        {
          CompleteClaimedBlock(in_isr);
          return WaitForBlock(op);
        }
        CompleteClaimedRead(in_isr);
        return ErrorCode::OK;
      }
      continue;
    }

    uint32_t expected = observed;
    if (state_.compare_exchange_weak(expected, static_cast<uint32_t>(Phase::PENDING),
                                     std::memory_order_acq_rel,
                                     std::memory_order_acquire))
    {
      break;
    }
  }

  return is_block ? WaitForBlock(op) : ErrorCode::OK;
}

void ReadPort::ProcessPendingReads(bool in_isr)
{
  for (;;)
  {
    uint32_t observed = state_.load(std::memory_order_acquire);
    if (GetPhase(observed) != Phase::PENDING)
    {
      return;
    }

    if (!state_.compare_exchange_weak(observed, static_cast<uint32_t>(Phase::CLAIMED),
                                      std::memory_order_acq_rel,
                                      std::memory_order_acquire))
    {
      continue;
    }

    if (!HasEnough(queue_data_->Size(), info_.data.size_))
    {
      for (;;)
      {
        uint32_t current = state_.load(std::memory_order_acquire);
        const Phase current_phase = GetPhase(current);
        if (current_phase == Phase::CLAIMED_WITH_WAITER)
        {
          Semaphore* const waiter = info_.op.data.sem_info.sem;
          DEV_ASSERT_FROM_CALLBACK(waiter != nullptr, in_isr);
          const uint32_t desired = WithPhase(current, Phase::IDLE);
          if (state_.compare_exchange_weak(current, desired, std::memory_order_acq_rel,
                                           std::memory_order_acquire))
          {
            waiter->PostFromCallback(in_isr);
            return;
          }
          continue;
        }

        DEV_ASSERT_FROM_CALLBACK(current_phase == Phase::CLAIMED, in_isr);
        const uint32_t desired = WithPhase(current, Phase::PENDING);
        if (state_.compare_exchange_weak(current, desired, std::memory_order_acq_rel,
                                         std::memory_order_acquire))
        {
          if (HasEvent(desired))
          {
            break;
          }
          return;
        }
      }
      continue;
    }

    if (info_.op.type == ReadOperation::OperationType::BLOCK)
    {
      CompleteClaimedBlock(in_isr);
    }
    else
    {
      CompleteClaimedRead(in_isr);
    }
    return;
  }
}

void ReadPort::CompleteClaimedRead(bool in_isr)
{
  Request completed = info_;
  DEV_ASSERT_FROM_CALLBACK(completed.op.type != ReadOperation::OperationType::BLOCK,
                           in_isr);

  if (completed.data.size_ != 0U)
  {
    [[maybe_unused]] const auto pop_batch_result = queue_data_->PopBatch(
        reinterpret_cast<uint8_t*>(completed.data.addr_), completed.data.size_);
    DEV_ASSERT_FROM_CALLBACK(pop_batch_result == ErrorCode::OK, in_isr);
  }

  ReleaseClaimed(in_isr);
  if (completed.data.size_ != 0U)
  {
    OnReadQueueSpaceAvailable(in_isr);
  }
  completed.op.UpdateStatus(in_isr, ErrorCode::OK);
}

void ReadPort::CompleteClaimedBlock(bool in_isr)
{
  Request completed = info_;
  DEV_ASSERT_FROM_CALLBACK(completed.op.type == ReadOperation::OperationType::BLOCK,
                           in_isr);
  [[maybe_unused]] const auto claim_block_completion_result = ClaimBlockCompletion();
  DEV_ASSERT_FROM_CALLBACK(claim_block_completion_result, in_isr);

  if (completed.data.size_ != 0U)
  {
    [[maybe_unused]] const auto pop_batch_result = queue_data_->PopBatch(
        reinterpret_cast<uint8_t*>(completed.data.addr_), completed.data.size_);
    DEV_ASSERT_FROM_CALLBACK(pop_batch_result == ErrorCode::OK, in_isr);
    OnReadQueueSpaceAvailable(in_isr);
  }

  block_result_ = ErrorCode::OK;
  completed.op.data.sem_info.sem->PostFromCallback(in_isr);
}

ErrorCode ReadPort::WaitForBlock(ReadOperation& op)
{
  ErrorCode wait_result = op.data.sem_info.sem->Wait(op.data.sem_info.timeout);
  if (wait_result == ErrorCode::OK)
  {
    DEV_ASSERT(GetPhase(state_.load(std::memory_order_acquire)) == Phase::BLOCK_CLAIMED);
    const ErrorCode result = block_result_;
    ReleaseBlockCompletion(false);
    return result;
  }

  for (;;)
  {
    uint32_t observed = state_.load(std::memory_order_acquire);
    const Phase phase = GetPhase(observed);
    if (phase == Phase::PENDING)
    {
      const uint32_t desired = WithPhase(observed, Phase::IDLE);
      if (state_.compare_exchange_weak(observed, desired, std::memory_order_acq_rel,
                                       std::memory_order_acquire))
      {
        return ErrorCode::TIMEOUT;
      }
      continue;
    }

    if (phase == Phase::CLAIMED)
    {
      const uint32_t desired = WithPhase(observed, Phase::CLAIMED_WITH_WAITER);
      if (state_.compare_exchange_weak(observed, desired, std::memory_order_acq_rel,
                                       std::memory_order_acquire))
      {
        break;
      }
      continue;
    }

    DEV_ASSERT(phase == Phase::CLAIMED_WITH_WAITER || phase == Phase::BLOCK_CLAIMED);
    break;
  }

  do
  {
    wait_result = op.data.sem_info.sem->Wait(UINT32_MAX);
  } while (wait_result == ErrorCode::TIMEOUT);
  REQUIRE(wait_result == ErrorCode::OK);
  const Phase completed_phase = GetPhase(state_.load(std::memory_order_acquire));
  if (completed_phase == Phase::IDLE)
  {
    return ErrorCode::TIMEOUT;
  }

  DEV_ASSERT(completed_phase == Phase::BLOCK_CLAIMED);
  const ErrorCode result = block_result_;
  ReleaseBlockCompletion(false);
  return result;
}

ErrorCode ReadPort::ClearQueuedData(bool in_isr)
{
  if (!Readable())
  {
    return ErrorCode::NOT_SUPPORT;
  }
  if (!TryClaimIdle())
  {
    return ErrorCode::BUSY;
  }

  queue_data_->Reset();
  ReleaseClaimed(in_isr);
  OnReadQueueSpaceAvailable(in_isr);
  return ErrorCode::OK;
}

void ReadPort::BindQueue(SPSCQueue<uint8_t>* queue)
{
  DEV_ASSERT(queue != nullptr);
  DEV_ASSERT(queue_data_ == nullptr);
  queue_data_ = queue;
}
