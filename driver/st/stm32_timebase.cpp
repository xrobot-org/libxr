#include "stm32_timebase.hpp"

using namespace LibXR;

namespace
{
enum class STM32TimebaseBackend : uint8_t
{
  SYSTICK = 0,
  TIMER = 1,
};

STM32TimebaseBackend g_backend = STM32TimebaseBackend::SYSTICK;

MicrosecondTimestamp GetSysTickMicroseconds()
{
  do
  {
    const uint32_t tick_before = HAL_GetTick();
    const uint32_t counter_before = SysTick->VAL;
    // 读取挂起位，不通过 CTRL 读取并清除 COUNTFLAG。
    // Read the pending bit without reading and clearing COUNTFLAG through CTRL.
    const bool pending = (SCB->ICSR & SCB_ICSR_PENDSTSET_Msk) != 0U;
    const uint32_t counter_after = SysTick->VAL;
    const uint32_t tick_after = HAL_GetTick();
    if (tick_before != tick_after)
    {
      continue;
    }

    const bool crossed_zero =
        counter_before != 0U && (counter_after == 0U || counter_after > counter_before);
    uint32_t logical_tick = tick_after + static_cast<uint32_t>(pending || crossed_zero);
    const uint64_t period_counts = static_cast<uint64_t>(SysTick->LOAD) + 1U;
    uint64_t fraction = 0U;
    if (counter_after != 0U)
    {
      fraction = 1000U - static_cast<uint64_t>(counter_after) * 1000U / period_counts;
      if (fraction == 1000U)
      {
        ++logical_tick;
        fraction = 0U;
      }
    }
    return MicrosecondTimestamp(static_cast<uint64_t>(logical_tick) * 1000U + fraction);
  } while (true);
}

#ifdef HAL_TIM_MODULE_ENABLED
MicrosecondTimestamp GetTimerMicroseconds(TIM_HandleTypeDef* htim)
{
  DEV_ASSERT(htim != nullptr);

  do
  {
    const uint32_t tick_before = HAL_GetTick();
    const uint32_t counter_before = __HAL_TIM_GET_COUNTER(htim);
    const bool pending = __HAL_TIM_GET_FLAG(htim, TIM_FLAG_UPDATE) != 0U;
    const uint32_t counter_after = __HAL_TIM_GET_COUNTER(htim);
    const uint32_t tick_after = HAL_GetTick();
    if (tick_before != tick_after)
    {
      continue;
    }

    const bool period_elapsed = pending || counter_after < counter_before;
    const uint32_t logical_tick = tick_after + static_cast<uint32_t>(period_elapsed);
    const uint64_t period_counts =
        static_cast<uint64_t>(__HAL_TIM_GET_AUTORELOAD(htim)) + 1U;
    const uint64_t fraction =
        static_cast<uint64_t>(counter_after) * 1000U / period_counts;
    return MicrosecondTimestamp(static_cast<uint64_t>(logical_tick) * 1000U + fraction);
  } while (true);
}

#endif
}  // namespace

STM32Timebase::STM32Timebase()
{
  ConfigureWrapRange(static_cast<uint64_t>(UINT32_MAX) * 1000ULL + 999ULL, UINT32_MAX);
  g_backend = STM32TimebaseBackend::SYSTICK;
  SetReady();
}

MicrosecondTimestamp Timebase::GetMicroseconds()
{
  switch (g_backend)
  {
    case STM32TimebaseBackend::SYSTICK:
      return GetSysTickMicroseconds();
#ifdef HAL_TIM_MODULE_ENABLED
    case STM32TimebaseBackend::TIMER:
      return GetTimerMicroseconds(STM32TimerTimebase::htim);
#endif
  }

  DEV_ASSERT(false);
  return MicrosecondTimestamp(0ULL);
}

MillisecondTimestamp Timebase::GetMilliseconds() { return HAL_GetTick(); }

#ifdef HAL_TIM_MODULE_ENABLED

TIM_HandleTypeDef* STM32TimerTimebase::htim = nullptr;

STM32TimerTimebase::STM32TimerTimebase(TIM_HandleTypeDef* timer)
{
  ASSERT(timer != nullptr);

  htim = timer;
  ConfigureWrapRange(static_cast<uint64_t>(UINT32_MAX) * 1000ULL + 999ULL, UINT32_MAX);
  g_backend = STM32TimebaseBackend::TIMER;
  SetReady();
}

#endif
