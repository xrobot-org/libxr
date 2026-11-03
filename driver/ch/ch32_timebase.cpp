// NOLINTBEGIN(cppcoreguidelines-pro-type-cstyle-cast,performance-no-int-to-ptr)
#include "ch32_timebase.hpp"

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
// 每微秒、每毫秒的周期数；时钟不是整倍数时为 0。
// Cycles per microsecond and per millisecond; 0 when the clock is not a whole multiple.
uint32_t cycles_per_us = 0U;
uint32_t cycles_per_ms = 0U;

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

uint32_t CyclesPerUnit(uint64_t units_per_second)
{
  return timebase_clock_hz % units_per_second == 0U
             ? static_cast<uint32_t>(timebase_clock_hz / units_per_second)
             : 0U;
}

uint64_t CyclesToUnits(uint64_t cycles, uint64_t units_per_second,
                       uint32_t cycles_per_unit)
{
  // 时钟是整倍数时一次除法就够，这是常见情况。 / One division suffices when the clock
  // is a whole multiple, the common case.
  if (cycles_per_unit != 0U)
  {
    return cycles / cycles_per_unit;
  }
  // 分开换算整秒和余数，避免直接乘法溢出。
  // Convert whole seconds and remaining cycles separately to avoid overflow.
  const uint64_t seconds = cycles / timebase_clock_hz;
  const uint64_t remaining_cycles = cycles % timebase_clock_hz;
  return seconds * units_per_second +
         remaining_cycles * units_per_second / timebase_clock_hz;
}
}  // namespace

CH32Timebase::CH32Timebase()
{
  const uint32_t control = SysTick->CTLR;
  ASSERT((control & (SYSTICK_ENABLE | SYSTICK_CLOCK_HCLK)) ==
             (SYSTICK_ENABLE | SYSTICK_CLOCK_HCLK) &&
         (control & (SYSTICK_AUTO_RELOAD | SYSTICK_COUNT_DOWN)) == 0U &&
         SystemCoreClock >= MICROSECONDS_PER_SECOND);

  timebase_clock_hz = SystemCoreClock;
  cycles_per_us = CyclesPerUnit(MICROSECONDS_PER_SECOND);
  cycles_per_ms = CyclesPerUnit(MILLISECONDS_PER_SECOND);
  ConfigureWrapRange(CyclesToUnits(UINT64_MAX, MICROSECONDS_PER_SECOND, cycles_per_us),
                     UINT32_MAX);
  SetReady();
}

MicrosecondTimestamp Timebase::GetMicroseconds()
{
  return MicrosecondTimestamp(
      CyclesToUnits(ReadSysTickCounter(), MICROSECONDS_PER_SECOND, cycles_per_us));
}

MillisecondTimestamp Timebase::GetMilliseconds()
{
  return MillisecondTimestamp(static_cast<uint32_t>(
      CyclesToUnits(ReadSysTickCounter(), MILLISECONDS_PER_SECOND, cycles_per_ms)));
}

// NOLINTEND(cppcoreguidelines-pro-type-cstyle-cast,performance-no-int-to-ptr)
