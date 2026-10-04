#include "mspm0_timebase.hpp"

using namespace LibXR;

MSPM0Timebase::MSPM0Timebase()
{
  ConfigureWrapRange(static_cast<uint64_t>(UINT32_MAX) * 1000ULL + 999ULL, UINT32_MAX);
  SetReady();
}

namespace
{
struct SysTickSnapshot
{
  uint32_t milliseconds;
  uint32_t period;
  uint32_t value;
};

// 只在快照期间屏蔽中断；不读取会清除 COUNTFLAG 的 CTRL。
// Mask IRQs only for the snapshot; do not read CTRL and consume COUNTFLAG.
// BSP 必须在下一次回绕前处理 SysTick；一个 pending 位不能记录多次回绕。
// The BSP must service SysTick before another wrap; one pending bit is not a counter.
SysTickSnapshot ReadSysTickSnapshot()
{
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();

  SysTickSnapshot snapshot{MSPM0Timebase::sys_tick_ms, DL_SYSTICK_getPeriod(), 0U};
  uint32_t pending_before;
  uint32_t pending_after;
  do
  {
    pending_before = SCB->ICSR & SCB_ICSR_PENDSTSET_Msk;
    snapshot.value = DL_SYSTICK_getValue();
    pending_after = SCB->ICSR & SCB_ICSR_PENDSTSET_Msk;
  } while (pending_before != pending_after);

  if (pending_after != 0U)
  {
    ++snapshot.milliseconds;
  }
  __set_PRIMASK(primask);
  return snapshot;
}
}  // namespace

MicrosecondTimestamp Timebase::GetMicroseconds()
{
  const auto snapshot = ReadSysTickSnapshot();
  // VAL == 0 是回绕/初次装载边界，不再叠加一个完整周期。
  // VAL == 0 is the wrap/initial-load boundary, not an additional full period.
  const uint32_t elapsed = snapshot.value == 0U ? 0U : snapshot.period - snapshot.value;
  return MicrosecondTimestamp(static_cast<uint64_t>(snapshot.milliseconds) * 1000ULL +
                              static_cast<uint64_t>(elapsed) * 1000ULL / snapshot.period);
}

MillisecondTimestamp Timebase::GetMilliseconds()
{
  return ReadSysTickSnapshot().milliseconds;
}
void MSPM0Timebase::OnSysTickInterrupt() { MSPM0Timebase::sys_tick_ms++; }
void MSPM0Timebase::Sync(uint32_t ticks) { MSPM0Timebase::sys_tick_ms = ticks; }

extern "C" void SysTick_Handler(void)  // NOLINT
{
  LibXR::MSPM0Timebase::OnSysTickInterrupt();
}
