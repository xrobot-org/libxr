#include "write_port.hpp"

#include <cstring>
#include <new>

using namespace LibXR;

WritePort::WritePort(size_t queue_size, size_t buffer_size)
    : queue_requests_(queue_size != 0U
                          ? new (std::align_val_t(LibXR::CONCURRENCY_ALIGNMENT))
                                SPSCQueue<Request>(ValidateQueueSize(queue_size))
                          : nullptr),
      queue_data_(buffer_size != 0U ? new (std::align_val_t(LibXR::CONCURRENCY_ALIGNMENT))
                                          SPSCQueue<uint8_t>(buffer_size)
                                    : nullptr)
{
}

size_t WritePort::EmptySize() const
{
  return queue_data_ == nullptr ? 0U : queue_data_->EmptySize();
}

size_t WritePort::Size() const
{
  return queue_data_ == nullptr ? 0U : queue_data_->Size();
}

size_t WritePort::Capacity() const
{
  return queue_data_ == nullptr ? 0U : queue_data_->MaxSize();
}

bool WritePort::Writable() const
{
  return queue_data_ != nullptr && write_fun_ != nullptr;
}

WritePort& WritePort::operator=(WriteFun fun)
{
  write_fun_ = fun;
  return *this;
}

bool WritePort::TryClaimProducer()
{
  uint32_t observed = state_.load(std::memory_order_acquire);
  for (;;)
  {
    if (GetPhase(observed) != Phase::IDLE)
    {
      return false;
    }

    const uint32_t desired = WithPhase(observed, Phase::LOCKED);
    if (state_.compare_exchange_weak(observed, desired, std::memory_order_acq_rel,
                                     std::memory_order_acquire))
    {
      return true;
    }
  }
}

void WritePort::ReleaseProducer(Phase next, [[maybe_unused]] bool in_isr)
{
  uint32_t observed = state_.load(std::memory_order_acquire);
  for (;;)
  {
    DEV_ASSERT_FROM_CALLBACK(GetPhase(observed) == Phase::LOCKED, in_isr);
    const uint32_t desired = WithPhase(observed, next);
    if (state_.compare_exchange_weak(observed, desired, std::memory_order_acq_rel,
                                     std::memory_order_acquire))
    {
      return;
    }
  }
}

void WritePort::PublishQueuedRequest(Phase next, [[maybe_unused]] bool in_isr)
{
  uint32_t observed = state_.load(std::memory_order_acquire);
  for (;;)
  {
    DEV_ASSERT_FROM_CALLBACK(GetPhase(observed) == Phase::LOCKED, in_isr);
    const uint32_t released = GetReleasedCount(observed);
    DEV_ASSERT_FROM_CALLBACK(released < MAX_RELEASED_REQUESTS, in_isr);
    const uint32_t desired = MakeState(next, released + 1U);
    if (state_.compare_exchange_weak(observed, desired, std::memory_order_acq_rel,
                                     std::memory_order_acquire))
    {
      return;
    }
  }
}

void WritePort::DecrementReleasedRequest([[maybe_unused]] bool in_isr)
{
  uint32_t observed = state_.load(std::memory_order_acquire);
  for (;;)
  {
    DEV_ASSERT_FROM_CALLBACK(GetReleasedCount(observed) != 0U, in_isr);
    const uint32_t desired = observed - RELEASED_INCREMENT;
    if (state_.compare_exchange_weak(observed, desired, std::memory_order_acq_rel,
                                     std::memory_order_acquire))
    {
      return;
    }
  }
}

void WritePort::NotifyBackend(bool in_isr)
{
  if (write_fun_ != nullptr)
  {
    write_fun_(*this, in_isr);
  }
}

WritePort::WriteQueue WritePort::GetWriteQueue(bool in_isr)
{
  ASSERT_FROM_CALLBACK(queue_requests_ != nullptr, in_isr);

  if (GetReleasedCount(state_.load(std::memory_order_acquire)) == 0U)
  {
    DEV_ASSERT_FROM_CALLBACK(front_remaining_ == 0U, in_isr);
    return WriteQueue(*this, in_isr, 0U);
  }

  Request request{};
  [[maybe_unused]] const auto peek_result = queue_requests_->Peek(request);
  DEV_ASSERT_FROM_CALLBACK(peek_result == ErrorCode::OK, in_isr);
  if (front_remaining_ == 0U)
  {
    DEV_ASSERT_FROM_CALLBACK(request.size != 0U, in_isr);
    front_remaining_ = request.size;
  }
  else
  {
    DEV_ASSERT_FROM_CALLBACK(front_remaining_ <= request.size, in_isr);
  }

  return WriteQueue(*this, in_isr, front_remaining_);
}

void WritePort::WriteQueue::PopAll(uint8_t* destination)
{
  BeginAction();
  const size_t remaining = AvailableSize();
  DEV_ASSERT_FROM_CALLBACK(remaining != 0U, in_isr_);
  DEV_ASSERT_FROM_CALLBACK(destination != nullptr, in_isr_);

  size_t offset = 0U;
  const size_t accepted = port_.queue_data_->ConsumeWithReader(
      remaining,
      [destination, &offset](const uint8_t* first, size_t first_size,
                             const uint8_t* second, size_t second_size) -> size_t
      {
        if (first_size != 0U)
        {
          std::memcpy(destination + offset, first, first_size);
          offset += first_size;
        }
        if (second_size != 0U)
        {
          std::memcpy(destination + offset, second, second_size);
          offset += second_size;
        }
        return first_size + second_size;
      });
  DEV_ASSERT_FROM_CALLBACK(accepted == remaining, in_isr_);
  popped_size_ += accepted;
}

void WritePort::WriteQueue::FailFront(ErrorCode reason)
{
  BeginAction();
  ASSERT_FROM_CALLBACK(reason != ErrorCode::OK, in_isr_);
  DEV_ASSERT_FROM_CALLBACK(front_size_ != 0U, in_isr_);

  const size_t remaining = AvailableSize();
  DEV_ASSERT_FROM_CALLBACK(remaining != 0U, in_isr_);
  const size_t accepted = port_.queue_data_->ConsumeWithReader(
      remaining,
      [](const uint8_t* first, size_t first_size, const uint8_t* second,
         size_t second_size) -> size_t
      {
        UNUSED(first);
        UNUSED(second);
        return first_size + second_size;
      });
  DEV_ASSERT_FROM_CALLBACK(accepted == remaining, in_isr_);
  popped_size_ += accepted;
  settlement_result_ = reason;
}

WritePort::WriteQueue::~WriteQueue() noexcept
{
  port_.SettleWriteQueue(popped_size_, settlement_result_, in_isr_);
}

void WritePort::SettleWriteQueue(size_t accepted, ErrorCode result, bool in_isr) noexcept
{
  if (accepted == 0U)
  {
    return;
  }

  DEV_ASSERT_FROM_CALLBACK(queue_requests_ != nullptr, in_isr);
  DEV_ASSERT_FROM_CALLBACK(accepted <= front_remaining_, in_isr);
  front_remaining_ -= accepted;
  if (front_remaining_ != 0U)
  {
    return;
  }

  Request completed{};
  [[maybe_unused]] const auto pop_result = queue_requests_->Pop(completed);
  DEV_ASSERT_FROM_CALLBACK(pop_result == ErrorCode::OK, in_isr);
  DecrementReleasedRequest(in_isr);
  CompleteRequest(completed, result, in_isr);
}

void WritePort::CompleteRequest(Request& request, ErrorCode result, bool in_isr)
{
  if (request.op.type != WriteOperation::OperationType::BLOCK)
  {
    request.op.UpdateStatus(in_isr, result);
    return;
  }

  for (;;)
  {
    uint32_t observed = state_.load(std::memory_order_acquire);
    const Phase phase = GetPhase(observed);
    if (phase == Phase::BLOCK_WAITING)
    {
      const uint32_t desired = WithPhase(observed, Phase::BLOCK_CLAIMED);
      if (!state_.compare_exchange_weak(observed, desired, std::memory_order_acq_rel,
                                        std::memory_order_acquire))
      {
        continue;
      }

      block_result_ = result;
      request.op.data.sem_info.sem->PostFromCallback(in_isr);
      return;
    }

    if (phase == Phase::BLOCK_DETACHED)
    {
      const uint32_t desired = WithPhase(observed, Phase::IDLE);
      if (!state_.compare_exchange_weak(observed, desired, std::memory_order_acq_rel,
                                        std::memory_order_acquire))
      {
        continue;
      }
      return;
    }

    if (phase == Phase::BLOCK_RETIRE_WAITING)
    {
      const uint32_t desired = WithPhase(observed, Phase::LOCKED);
      if (!state_.compare_exchange_weak(observed, desired, std::memory_order_acq_rel,
                                        std::memory_order_acquire))
      {
        continue;
      }

      Semaphore* waiter = admission_waiter_;
      admission_waiter_ = nullptr;
      DEV_ASSERT_FROM_CALLBACK(waiter != nullptr, in_isr);
      waiter->PostFromCallback(in_isr);
      return;
    }

    DEV_ASSERT_FROM_CALLBACK(false, in_isr);
    return;
  }
}

ErrorCode WritePort::CommitQueued(size_t size, WriteOperation& op, bool in_isr)
{
  DEV_ASSERT_FROM_CALLBACK(queue_requests_ != nullptr, in_isr);
  DEV_ASSERT_FROM_CALLBACK(queue_data_ != nullptr, in_isr);
  DEV_ASSERT_FROM_CALLBACK(size != 0U, in_isr);

  Request request{size, op};
  [[maybe_unused]] const auto push_result = queue_requests_->Push(request);
  DEV_ASSERT_FROM_CALLBACK(push_result == ErrorCode::OK, in_isr);
  request.op.MarkAsRunning();

  PublishQueuedRequest(op.type == WriteOperation::OperationType::BLOCK
                           ? Phase::BLOCK_WAITING
                           : Phase::IDLE,
                       in_isr);
  NotifyBackend(in_isr);

  return op.type == WriteOperation::OperationType::BLOCK ? WaitForBlock(op)
                                                         : ErrorCode::OK;
}

ErrorCode WritePort::CommitAdmission([[maybe_unused]] size_t size, WriteOperation& op,
                                     bool in_isr)
{
  DEV_ASSERT_FROM_CALLBACK(IsAdmissionMode(), in_isr);
  DEV_ASSERT_FROM_CALLBACK(size != 0U, in_isr);

  WriteOperation completed = op;
  completed.MarkAsRunning();
  NotifyBackend(in_isr);
  ReleaseProducer(Phase::IDLE, in_isr);

  if (completed.type != WriteOperation::OperationType::BLOCK)
  {
    completed.UpdateStatus(in_isr, ErrorCode::OK);
  }
  return ErrorCode::OK;
}

bool WritePort::TryRegisterRetirementWait(WriteOperation& op)
{
  DEV_ASSERT(op.type == WriteOperation::OperationType::BLOCK);
  admission_waiter_ = op.data.sem_info.sem;
  uint32_t observed = state_.load(std::memory_order_acquire);
  for (;;)
  {
    if (GetPhase(observed) != Phase::BLOCK_DETACHED)
    {
      admission_waiter_ = nullptr;
      return false;
    }

    const uint32_t desired = WithPhase(observed, Phase::BLOCK_RETIRE_WAITING);
    if (state_.compare_exchange_weak(observed, desired, std::memory_order_acq_rel,
                                     std::memory_order_acquire))
    {
      return true;
    }
  }
}

ErrorCode WritePort::WaitForRetirement(WriteOperation& op)
{
  if (!TryRegisterRetirementWait(op))
  {
    if (LoadPhase() == Phase::IDLE && TryClaimProducer())
    {
      return ErrorCode::OK;
    }
    return ErrorCode::BUSY;
  }

  ErrorCode wait_result = op.data.sem_info.sem->Wait(op.data.sem_info.timeout);
  if (wait_result == ErrorCode::OK)
  {
    DEV_ASSERT(LoadPhase() == Phase::LOCKED);
    return ErrorCode::OK;
  }

  for (;;)
  {
    uint32_t observed = state_.load(std::memory_order_acquire);
    const Phase phase = GetPhase(observed);
    if (phase == Phase::BLOCK_RETIRE_WAITING)
    {
      const uint32_t desired = WithPhase(observed, Phase::BLOCK_DETACHED);
      if (state_.compare_exchange_weak(observed, desired, std::memory_order_acq_rel,
                                       std::memory_order_acquire))
      {
        admission_waiter_ = nullptr;
        return ErrorCode::TIMEOUT;
      }
      continue;
    }

    DEV_ASSERT(phase == Phase::LOCKED);
    break;
  }

  do
  {
    wait_result = op.data.sem_info.sem->Wait(UINT32_MAX);
  } while (wait_result == ErrorCode::TIMEOUT);
  REQUIRE(wait_result == ErrorCode::OK);
  ReleaseProducer(Phase::IDLE, false);
  return ErrorCode::TIMEOUT;
}

ErrorCode WritePort::WaitForBlock(WriteOperation& op)
{
  ErrorCode wait_result = op.data.sem_info.sem->Wait(op.data.sem_info.timeout);
  if (wait_result == ErrorCode::OK)
  {
    DEV_ASSERT(LoadPhase() == Phase::BLOCK_CLAIMED);
    const ErrorCode result = block_result_;
    uint32_t observed = state_.load(std::memory_order_acquire);
    for (;;)
    {
      DEV_ASSERT(GetPhase(observed) == Phase::BLOCK_CLAIMED);
      const uint32_t desired = WithPhase(observed, Phase::IDLE);
      if (state_.compare_exchange_weak(observed, desired, std::memory_order_acq_rel,
                                       std::memory_order_acquire))
      {
        break;
      }
    }
    return result;
  }

  for (;;)
  {
    uint32_t observed = state_.load(std::memory_order_acquire);
    const Phase phase = GetPhase(observed);
    if (phase == Phase::BLOCK_WAITING)
    {
      const uint32_t desired = WithPhase(observed, Phase::BLOCK_DETACHED);
      if (state_.compare_exchange_weak(observed, desired, std::memory_order_acq_rel,
                                       std::memory_order_acquire))
      {
        return ErrorCode::TIMEOUT;
      }
      continue;
    }

    DEV_ASSERT(phase == Phase::BLOCK_CLAIMED);
    break;
  }

  do
  {
    wait_result = op.data.sem_info.sem->Wait(UINT32_MAX);
  } while (wait_result == ErrorCode::TIMEOUT);
  REQUIRE(wait_result == ErrorCode::OK);
  const ErrorCode result = block_result_;
  uint32_t observed = state_.load(std::memory_order_acquire);
  for (;;)
  {
    DEV_ASSERT(GetPhase(observed) == Phase::BLOCK_CLAIMED);
    const uint32_t desired = WithPhase(observed, Phase::IDLE);
    if (state_.compare_exchange_weak(observed, desired, std::memory_order_acq_rel,
                                     std::memory_order_acquire))
    {
      break;
    }
  }
  return result;
}

ErrorCode WritePort::operator()(ConstRawData data, WriteOperation& op, bool in_isr)
{
  if (!Writable())
  {
    return ErrorCode::NOT_SUPPORT;
  }

  if (data.size_ == 0U)
  {
    if (op.type != WriteOperation::OperationType::BLOCK)
    {
      op.UpdateStatus(in_isr, ErrorCode::OK);
    }
    return ErrorCode::OK;
  }

  if (!TryClaimProducer())
  {
    if (op.type == WriteOperation::OperationType::BLOCK &&
        LoadPhase() == Phase::BLOCK_DETACHED && data.size_ <= Capacity())
    {
      const ErrorCode retirement = WaitForRetirement(op);
      if (retirement != ErrorCode::OK)
      {
        return retirement;
      }
    }
    else
    {
      return ErrorCode::BUSY;
    }
  }

  ASSERT_FROM_CALLBACK(data.addr_ != nullptr, in_isr);

  if (queue_data_->EmptySize() < data.size_ ||
      (!IsAdmissionMode() && queue_requests_->EmptySize() < 1U))
  {
    ReleaseProducer(Phase::IDLE, in_isr);
    return ErrorCode::FULL;
  }

  [[maybe_unused]] const auto push_batch_result =
      queue_data_->PushBatch(reinterpret_cast<const uint8_t*>(data.addr_), data.size_);
  DEV_ASSERT_FROM_CALLBACK(push_batch_result == ErrorCode::OK, in_isr);

  return IsAdmissionMode() ? CommitAdmission(data.size_, op, in_isr)
                           : CommitQueued(data.size_, op, in_isr);
}
