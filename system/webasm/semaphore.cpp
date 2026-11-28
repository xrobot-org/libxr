#include "semaphore.hpp"

#include <atomic>

#include "libxr_def.hpp"
#include "timebase.hpp"
#include "timer.hpp"

using namespace LibXR;

static_assert(alignof(libxr_semaphore_handle) >=
              std::atomic_ref<uint32_t>::required_alignment);

namespace
{
bool TryAcquire(std::atomic_ref<uint32_t> count)
{
  uint32_t observed = count.load(std::memory_order_relaxed);
  while (observed > 0U)
  {
    if (count.compare_exchange_weak(observed, observed - 1U, std::memory_order_acquire,
                                    std::memory_order_relaxed))
    {
      return true;
    }
  }
  return false;
}
}  // namespace

Semaphore::Semaphore(uint32_t init_count) : semaphore_handle_(init_count) {}

Semaphore::~Semaphore() {}

void Semaphore::Post()
{
  std::atomic_ref<uint32_t> count(semaphore_handle_);
  count.fetch_add(1U, std::memory_order_release);
}

ErrorCode Semaphore::Wait(uint32_t timeout)
{
  std::atomic_ref<uint32_t> count(semaphore_handle_);
  if (TryAcquire(count))
  {
    return ErrorCode::OK;
  }
  if (timeout == 0U)
  {
    return ErrorCode::TIMEOUT;
  }

  const uint32_t now = Timebase::GetMilliseconds();

  while (uint32_t(Timebase::GetMilliseconds()) - now < timeout)
  {
    if (TryAcquire(count))
    {
      return ErrorCode::OK;
    }
    Timer::RefreshTimerInIdle();
  }
  return ErrorCode::TIMEOUT;
}

void Semaphore::PostFromCallback(bool in_isr)
{
  UNUSED(in_isr);
  Post();
}

size_t Semaphore::Value()
{
  std::atomic_ref<uint32_t> count(semaphore_handle_);
  return count.load(std::memory_order_acquire);
}
