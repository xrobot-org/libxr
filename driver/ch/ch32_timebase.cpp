// NOLINTBEGIN(cppcoreguidelines-pro-type-cstyle-cast,performance-no-int-to-ptr)
#include "ch32_timebase.hpp"

#include "driver_async_condition.hpp"
#if defined(LIBXR_SYSTEM_freertos)
#include "FreeRTOS.h"
#endif

using namespace LibXR;

namespace
{
constexpr uint32_t SYSTICK_ENABLE = 1U << 0U;
constexpr uint32_t SYSTICK_CLOCK_HCLK = 1U << 2U;
constexpr uint32_t SYSTICK_AUTO_RELOAD = 1U << 3U;
constexpr uint32_t SYSTICK_COUNT_DOWN = 1U << 4U;
constexpr uint64_t MICROSECONDS_PER_SECOND = 1000000ULL;
constexpr uint64_t MILLISECONDS_PER_SECOND = 1000ULL;

uint32_t timebase_clock_hz = 0U;

uint64_t ReadSysTickCounter()
{
  auto* count_words = reinterpret_cast<volatile uint32_t*>(&SysTick->CNT);
  uint32_t high_before;
  uint32_t low;
  uint32_t high_after;

  // 低位回绕时重新读取，避免拼接出不一致的计数值。
  // Retry across a low-word rollover to obtain a coherent counter value.
  do
  {
    high_before = count_words[1];
    low = count_words[0];
    high_after = count_words[1];
  } while (high_before != high_after);

  return (static_cast<uint64_t>(high_after) << 32U) | low;
}

uint64_t CyclesToUnits(uint64_t cycles, uint64_t units_per_second)
{
  // 分开换算整秒和余数，避免直接乘法溢出。
  // Convert whole seconds and remaining cycles separately to avoid overflow.
  const uint64_t seconds = cycles / timebase_clock_hz;
  const uint64_t remaining_cycles = cycles % timebase_clock_hz;
  return seconds * units_per_second +
         remaining_cycles * units_per_second / timebase_clock_hz;
}

#if defined(LIBXR_SYSTEM_none)
constexpr uint32_t SYSTICK_INTERRUPT = 1U << 1U;
constexpr uint32_t SYSTICK_COMPARE_FLAG = 1U << 0U;

uint32_t tick_cycles = 0U;  // Counter cycles per millisecond.

uint64_t ReadSysTickCompare()
{
  auto* compare_words = reinterpret_cast<volatile uint32_t*>(&SysTick->CMP);
  return (static_cast<uint64_t>(compare_words[1]) << 32U) | compare_words[0];
}

void WriteSysTickCompare(uint64_t compare)
{
  // The low word goes to its maximum first, so no half-written value is ever reached.
  auto* compare_words = reinterpret_cast<volatile uint32_t*>(&SysTick->CMP);
  compare_words[0] = UINT32_MAX;
  compare_words[1] = static_cast<uint32_t>(compare >> 32U);
  compare_words[0] = static_cast<uint32_t>(compare);
}

bool DeadlineReached(uint64_t now, uint64_t deadline)
{
  return ((now - deadline) & (1ULL << 63U)) == 0U;
}
#endif
}  // namespace

CH32Timebase::CH32Timebase()
{
  const uint32_t control = SysTick->CTLR;
  ASSERT((control & (SYSTICK_ENABLE | SYSTICK_CLOCK_HCLK)) ==
             (SYSTICK_ENABLE | SYSTICK_CLOCK_HCLK) &&
         (control & (SYSTICK_AUTO_RELOAD | SYSTICK_COUNT_DOWN)) == 0U &&
         SystemCoreClock >= MICROSECONDS_PER_SECOND);

  timebase_clock_hz = SystemCoreClock;
  ConfigureWrapRange(CyclesToUnits(UINT64_MAX, MICROSECONDS_PER_SECOND), UINT32_MAX);
#if defined(LIBXR_SYSTEM_none)
  // Without an RTOS the compare interrupt of the free-running counter is the millisecond
  // tick; the counter itself keeps running untouched.
  ASSERT(timebase_clock_hz % MILLISECONDS_PER_SECOND == 0U);
  tick_cycles = static_cast<uint32_t>(timebase_clock_hz / MILLISECONDS_PER_SECOND);
  NVIC_DisableIRQ(SysTicK_IRQn);
  WriteSysTickCompare(ReadSysTickCounter() + tick_cycles);
  SysTick->SR = 0U;
  NVIC_ClearPendingIRQ(SysTicK_IRQn);
  SysTick->CTLR = control | SYSTICK_INTERRUPT;
  NVIC_EnableIRQ(SysTicK_IRQn);
#endif
  SetReady();
}

MicrosecondTimestamp Timebase::GetMicroseconds()
{
  return MicrosecondTimestamp(
      CyclesToUnits(ReadSysTickCounter(), MICROSECONDS_PER_SECOND));
}

MillisecondTimestamp Timebase::GetMilliseconds()
{
  return MillisecondTimestamp(static_cast<uint32_t>(
      CyclesToUnits(ReadSysTickCounter(), MILLISECONDS_PER_SECOND)));
}

#if defined(LIBXR_SYSTEM_freertos)
static_assert(
    configUSE_TICK_HOOK == 1,
    "LibXR drives its DriverAsyncCondition waits from the FreeRTOS tick hook: set "
    "configUSE_TICK_HOOK to 1 in FreeRTOSConfig.h");

// FreeRTOS 节拍（LibXR 要求 1 kHz）推进驱动的条件等待；SysTick 比较中断归
// FreeRTOS 移植层。
//
// The FreeRTOS tick (1 kHz, which LibXR requires) advances the drivers' condition waits;
// the SysTick compare interrupt belongs to the FreeRTOS port.
extern "C" void vApplicationTickHook(void)  // NOLINT
{
  DriverAsyncCondition::OnTick();
}
#elif defined(LIBXR_SYSTEM_none)
// 无 RTOS 时由 SysTick 比较中断提供毫秒节拍：截止时间按绝对值推进，迟到的毫秒
// 一次补齐。
//
// Without an RTOS the SysTick compare interrupt is the millisecond tick: the deadline
// advances in absolute steps and late milliseconds are made up at once.
extern "C" __attribute__((interrupt("WCH-Interrupt-fast"))) void SysTick_Handler(void)
{
  if ((SysTick->SR & SYSTICK_COMPARE_FLAG) == 0U)
  {
    return;
  }
  uint64_t deadline = ReadSysTickCompare();
  uint64_t now = ReadSysTickCounter();
  uint32_t elapsed_ms = 0U;
  do
  {
    if (DeadlineReached(now, deadline))
    {
      const uint64_t overdue = now - deadline;
      uint64_t due = 1U;
      if (overdue >= tick_cycles)
      {
        due += overdue / tick_cycles;
      }
      elapsed_ms += static_cast<uint32_t>(due);
      deadline += due * tick_cycles;
      WriteSysTickCompare(deadline);
    }
    SysTick->SR = 0U;
    now = ReadSysTickCounter();
  } while (DeadlineReached(now, deadline));
  if (elapsed_ms != 0U)
  {
    DriverAsyncCondition::OnTick(elapsed_ms);
  }
}
#endif

// NOLINTEND(cppcoreguidelines-pro-type-cstyle-cast,performance-no-int-to-ptr)
