#include "thread.hpp"

#include "timebase.hpp"
#include "timer.hpp"

using namespace LibXR;

Thread Thread::Current(void) { return Thread(); }

void Thread::Sleep(uint32_t milliseconds)
{
  uint32_t now = Timebase::GetMilliseconds();
  while (uint32_t(Timebase::GetMilliseconds()) - now < milliseconds)
  {
    Timer::RefreshTimerInIdle();
  }
}

void Thread::SleepUntil(MillisecondTimestamp& last_wakeup_time, uint32_t time_to_sleep)
{
  while (uint32_t(Timebase::GetMilliseconds()) - last_wakeup_time < time_to_sleep)
  {
    Timer::RefreshTimerInIdle();
  }
  last_wakeup_time = last_wakeup_time + time_to_sleep;
}

uint32_t Thread::GetTime() { return Timebase::GetMilliseconds(); }

void Thread::Yield() {}
