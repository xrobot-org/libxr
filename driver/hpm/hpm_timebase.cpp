#include "hpm_timebase.hpp"

#include "driver_async_condition.hpp"
#if defined(LIBXR_SYSTEM_freertos)
#include "FreeRTOS.h"
#elif defined(LIBXR_SYSTEM_none)
#include "hpm_interrupt.h"
#endif

using namespace LibXR;

namespace
{
MCHTMR_Type* g_timer = HPM_MCHTMR;
uint32_t g_clock_hz = 0u;

static inline uint64_t ConvertTicksToTime(uint64_t ticks, uint32_t freq_hz,
                                          uint64_t scale)
{
  if (freq_hz == 0u) return 0u;

  const uint64_t whole = ticks / freq_hz;
  const uint64_t rem = ticks - (whole * freq_hz);
  const uint64_t frac = (rem * scale) / freq_hz;
  return whole * scale + frac;
}

#if defined(LIBXR_SYSTEM_none)
uint32_t g_tick_cycles = 0u;  // Timer cycles per millisecond.
uint64_t g_deadline = 0u;     // Absolute MTIMECMP of the next tick.

uint64_t ReadMtime(MCHTMR_Type* timer)
{
  // Retry across a low-word rollover: RV32 reads the 64-bit counter in two halves.
  auto* words = reinterpret_cast<volatile uint32_t*>(&timer->MTIME);
  uint32_t high;
  uint32_t low;
  do
  {
    high = words[1];
    low = words[0];
  } while (high != words[1]);
  return (static_cast<uint64_t>(high) << 32U) | low;
}

void WriteMtimecmp(MCHTMR_Type* timer, uint64_t compare)
{
  // The low word goes to its maximum first, so no half-written value is ever reached.
  auto* words = reinterpret_cast<volatile uint32_t*>(&timer->MTIMECMP);
  words[0] = UINT32_MAX;
  words[1] = static_cast<uint32_t>(compare >> 32U);
  words[0] = static_cast<uint32_t>(compare);
}

bool DeadlineReached(uint64_t now, uint64_t deadline)
{
  return ((now - deadline) & (1ULL << 63U)) == 0U;
}
#endif
}  // namespace

HPMTimebase::HPMTimebase(MCHTMR_Type* timer, clock_name_t clock)
{
  g_timer = timer;
  g_clock_hz = clock_get_frequency(clock);
  ConfigureWrapRange(static_cast<uint64_t>(UINT32_MAX) * 1000ULL + 999ULL, UINT32_MAX);
#if defined(LIBXR_SYSTEM_none)
  // Without an RTOS the machine timer compare interrupt is the millisecond tick; MTIME
  // itself keeps running untouched.
  ASSERT(g_clock_hz != 0u && g_clock_hz % 1000u == 0u);
  g_tick_cycles = g_clock_hz / 1000u;
  disable_mchtmr_irq();
  g_deadline = ReadMtime(g_timer) + g_tick_cycles;
  WriteMtimecmp(g_timer, g_deadline);
  enable_mchtmr_irq();
#endif
  SetReady();
}

MicrosecondTimestamp Timebase::GetMicroseconds()
{
  const uint64_t ticks = mchtmr_get_count(g_timer);
  return MicrosecondTimestamp(ConvertTicksToTime(ticks, g_clock_hz, 1000000ULL));
}

MillisecondTimestamp Timebase::GetMilliseconds()
{
  const uint64_t ticks = mchtmr_get_count(g_timer);
  return MillisecondTimestamp(ConvertTicksToTime(ticks, g_clock_hz, 1000ULL));
}

#if defined(LIBXR_SYSTEM_freertos)
static_assert(
    configUSE_TICK_HOOK == 1,
    "LibXR drives its DriverAsyncCondition waits from the FreeRTOS tick hook: set "
    "configUSE_TICK_HOOK to 1 in FreeRTOSConfig.h");

// FreeRTOS 节拍（LibXR 要求 1 kHz）推进驱动的条件等待。
// The FreeRTOS tick (1 kHz, which LibXR requires) advances the drivers' condition waits.
extern "C" void vApplicationTickHook(void)  // NOLINT
{
  DriverAsyncCondition::OnTick();
}
#elif defined(LIBXR_SYSTEM_none)
// 无 RTOS 时由机器定时器比较中断提供毫秒节拍（替换 SDK trap.c 的弱定义）：
// 截止时间按绝对值推进，迟到的毫秒一次补齐。中断是电平型的，MTIMECMP 越过 MTIME
// 后撤销。
//
// Without an RTOS the machine timer compare interrupt is the millisecond tick (replacing
// the weak definition in the SDK's trap.c): the deadline advances in absolute steps and
// late milliseconds are made up at once. The interrupt is level-triggered and clears
// once MTIMECMP is past MTIME.
extern "C" void mchtmr_isr(void)  // NOLINT
{
  uint64_t now = ReadMtime(g_timer);
  uint32_t elapsed_ms = 0u;
  while (DeadlineReached(now, g_deadline))
  {
    const uint64_t overdue = now - g_deadline;
    uint64_t due = 1u;
    if (overdue >= g_tick_cycles)
    {
      due += overdue / g_tick_cycles;
    }
    elapsed_ms += static_cast<uint32_t>(due);
    g_deadline += due * g_tick_cycles;
    WriteMtimecmp(g_timer, g_deadline);
    now = ReadMtime(g_timer);
  }
  if (elapsed_ms != 0u)
  {
    DriverAsyncCondition::OnTick(elapsed_ms);
  }
}
#endif
