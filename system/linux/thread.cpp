#include "thread.hpp"

#include <cerrno>

#include "libxr_def.hpp"
#include "monotonic_time.hpp"
#include "timebase.hpp"

using namespace LibXR;

extern struct timespec libxr_linux_start_time_spec;

Thread Thread::Current(void) { return Thread(pthread_self()); }

void Thread::Sleep(uint32_t milliseconds)
{
  timespec ts = MonotonicTime::RelativeFromMilliseconds(milliseconds);
  while (clock_nanosleep(CLOCK_MONOTONIC, 0, &ts, &ts) == EINTR)
  {
  }
}

void Thread::SleepUntil(MillisecondTimestamp& last_wakeup_time, uint32_t time_to_sleep)
{
  last_wakeup_time = last_wakeup_time + time_to_sleep;

  const timespec ts =
      MonotonicTime::AddMilliseconds(libxr_linux_start_time_spec, last_wakeup_time);

  while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr) == EINTR)
  {
  }
}

uint32_t Thread::GetTime() { return Timebase::GetMilliseconds(); }

void Thread::Yield() { sched_yield(); }

ErrorCode Thread::Join()
{
  return pthread_join(thread_handle_, nullptr) == 0 ? ErrorCode::OK : ErrorCode::FAILED;
}
