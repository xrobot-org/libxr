#include "esp_timebase.hpp"

#include "driver_async_condition.hpp"
#include "esp_attr.h"
#include "esp_freertos_hooks.h"
#include "freertos/FreeRTOS.h"
#if SOC_SYSTIMER_SUPPORTED
#include "esp_private/systimer.h"
#endif

// spi_flash 组件提供，声明在其私有头文件 esp_private/cache_utils.h 中。
// Provided by the spi_flash component, declared in its private esp_private/cache_utils.h.
extern "C" bool spi_flash_cache_enabled(void);

namespace LibXR
{

namespace
{
#if SOC_SYSTIMER_SUPPORTED
systimer_hal_context_t g_systimer_hal = {};
bool g_systimer_ready = false;
#endif
bool g_tick_hook_registered = false;

// FreeRTOS 节拍推进驱动的条件等待，只在 CPU 0 上注册，保证只有一个节拍源。节拍中断在 IRAM
// 中运行；flash cache 关闭期间跳过这一拍，等待只会变长。
// The FreeRTOS tick advances the drivers' condition waits, registered on CPU 0 only so
// there is one tick source. The tick interrupt runs from IRAM; while the flash cache is
// off the tick is skipped, which only lengthens the waits.
void IRAM_ATTR DriverAsyncConditionTick()
{
  if (spi_flash_cache_enabled())
  {
    DriverAsyncCondition::OnTick(portTICK_PERIOD_MS);
  }
}
}  // namespace

ESP32Timebase::ESP32Timebase()
{
  ConfigureWrapRange(UINT64_MAX, UINT32_MAX);
#if SOC_SYSTIMER_SUPPORTED
  if (!g_systimer_ready)
  {
    systimer_hal_init(&g_systimer_hal);

    systimer_hal_tick_rate_ops_t tick_rate_ops = {
        .ticks_to_us = systimer_ticks_to_us,
        .us_to_ticks = systimer_us_to_ticks,
    };
    systimer_hal_set_tick_rate_ops(&g_systimer_hal, &tick_rate_ops);
    systimer_hal_enable_counter(&g_systimer_hal, SYSTIMER_COUNTER_ESPTIMER);
    g_systimer_ready = true;
  }
#endif
  if (!g_tick_hook_registered)
  {
    REQUIRE(esp_register_freertos_tick_hook_for_cpu(DriverAsyncConditionTick, 0) ==
            ESP_OK);
    g_tick_hook_registered = true;
  }
  SetReady();
}

MicrosecondTimestamp IRAM_ATTR Timebase::GetMicroseconds()
{
#if SOC_SYSTIMER_SUPPORTED
  if (g_systimer_ready)
  {
    return systimer_hal_get_time(&g_systimer_hal, SYSTIMER_COUNTER_ESPTIMER);
  }
#endif
  return static_cast<MicrosecondTimestamp>(esp_timer_get_time());
}

MillisecondTimestamp IRAM_ATTR Timebase::GetMilliseconds()
{
  return MillisecondTimestamp(static_cast<uint32_t>(GetMicroseconds() / 1000ULL));
}

}  // namespace LibXR
