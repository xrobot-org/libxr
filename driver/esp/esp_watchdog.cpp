#include "esp_watchdog.hpp"

#include <cstdint>

#include "esp_private/periph_ctrl.h"
#include "hal/mwdt_ll.h"
#include "hal/timer_ll.h"
#include "soc/rtc.h"

namespace LibXR
{
namespace
{

constexpr uint32_t MWDT_TICK_US = 500;
constexpr uint32_t MWDT_TICKS_PER_MS = 1000 / MWDT_TICK_US;

bool GetMwdtGroupInfo(wdt_inst_t instance, int* group_id, periph_module_t* periph)
{
  if (instance == WDT_MWDT0)
  {
    *group_id = 0;
    *periph = PERIPH_TIMG0_MODULE;
    return true;
  }

#if SOC_TIMER_GROUPS >= 2
  if (instance == WDT_MWDT1)
  {
    *group_id = 1;
    *periph = PERIPH_TIMG1_MODULE;
    return true;
  }
#endif

  return false;
}

}  // namespace

ESP32Watchdog::ESP32Watchdog(uint32_t timeout_ms, uint32_t feed_ms)
{
  (void)SetConfig({timeout_ms, feed_ms});
  (void)Start();
}

bool ESP32Watchdog::IsInstanceSupported(wdt_inst_t instance)
{
  switch (instance)
  {
    case WDT_MWDT0:
      return true;
#if SOC_TIMER_GROUPS >= 2
    case WDT_MWDT1:
      return true;
#endif
    case WDT_RWDT:
      return true;
    default:
      return false;
  }
}

ErrorCode ESP32Watchdog::InitializeHardware()
{
  if (initialized_)
  {
    return ErrorCode::OK;
  }

  if (!IsInstanceSupported(instance_))
  {
    return ErrorCode::NOT_SUPPORT;
  }

  if (instance_ != WDT_RWDT)
  {
    int group_id = 0;
    periph_module_t periph = PERIPH_TIMG0_MODULE;
    if (!GetMwdtGroupInfo(instance_, &group_id, &periph))
    {
      return ErrorCode::NOT_SUPPORT;
    }

    PERIPH_RCC_ACQUIRE_ATOMIC(periph, ref_count)
    {
      if (ref_count == 0)
      {
        timer_ll_enable_bus_clock(group_id, true);
        timer_ll_reset_register(group_id);
      }
    }
  }

  const uint32_t prescaler = instance_ == WDT_RWDT ? 0U : MWDT_LL_DEFAULT_CLK_PRESCALER;
  wdt_hal_init(&hal_, instance_, prescaler, false);
  initialized_ = true;
  return ErrorCode::OK;
}

ErrorCode ESP32Watchdog::ApplyConfiguration()
{
  if (!initialized_)
  {
    return ErrorCode::INIT_ERR;
  }

  uint64_t ticks64 = 0U;
  if (instance_ == WDT_RWDT)
  {
    const uint32_t slow_clock_hz = rtc_clk_slow_freq_get_hz();
    if (slow_clock_hz == 0U)
    {
      return ErrorCode::INIT_ERR;
    }
    ticks64 = static_cast<uint64_t>(timeout_ms_) * slow_clock_hz / 1000U;
  }
  else
  {
    ticks64 = static_cast<uint64_t>(timeout_ms_) * MWDT_TICKS_PER_MS;
  }
  if ((ticks64 == 0) || (ticks64 > UINT32_MAX))
  {
    return ErrorCode::NOT_SUPPORT;
  }

  const uint32_t stage0_ticks = static_cast<uint32_t>(ticks64);

  wdt_hal_write_protect_disable(&hal_);
  wdt_hal_config_stage(&hal_, WDT_STAGE0, stage0_ticks, WDT_STAGE_ACTION_RESET_SYSTEM);
  wdt_hal_config_stage(&hal_, WDT_STAGE1, 0, WDT_STAGE_ACTION_OFF);
  wdt_hal_config_stage(&hal_, WDT_STAGE2, 0, WDT_STAGE_ACTION_OFF);
  wdt_hal_config_stage(&hal_, WDT_STAGE3, 0, WDT_STAGE_ACTION_OFF);
  wdt_hal_write_protect_enable(&hal_);

  return ErrorCode::OK;
}

ErrorCode ESP32Watchdog::SetConfig(const Configuration& config)
{
  if ((config.timeout_ms == 0) || (config.feed_ms == 0) ||
      (config.feed_ms > config.timeout_ms))
  {
    return ErrorCode::ARG_ERR;
  }

  timeout_ms_ = config.timeout_ms;
  auto_feed_interval_ms = config.feed_ms;

  ErrorCode ret = InitializeHardware();
  if (ret != ErrorCode::OK)
  {
    return ret;
  }

  return ApplyConfiguration();
}

ErrorCode ESP32Watchdog::Feed()
{
  if (!initialized_ || !started_)
  {
    return ErrorCode::INIT_ERR;
  }

  wdt_hal_write_protect_disable(&hal_);
  wdt_hal_feed(&hal_);
  wdt_hal_write_protect_enable(&hal_);
  return ErrorCode::OK;
}

ErrorCode ESP32Watchdog::Start()
{
  ErrorCode ret = InitializeHardware();
  if (ret != ErrorCode::OK)
  {
    return ret;
  }

  ret = ApplyConfiguration();
  if (ret != ErrorCode::OK)
  {
    return ret;
  }

  wdt_hal_write_protect_disable(&hal_);
  wdt_hal_enable(&hal_);
  wdt_hal_feed(&hal_);
  wdt_hal_write_protect_enable(&hal_);

  auto_feed_ = true;
  started_ = true;
  return ErrorCode::OK;
}

ErrorCode ESP32Watchdog::Stop()
{
  if (!initialized_)
  {
    return ErrorCode::INIT_ERR;
  }

  wdt_hal_write_protect_disable(&hal_);
  wdt_hal_disable(&hal_);
  wdt_hal_write_protect_enable(&hal_);

  auto_feed_ = false;
  started_ = false;
  return ErrorCode::OK;
}

}  // namespace LibXR
