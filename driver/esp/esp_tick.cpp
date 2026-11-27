#include "driver_async_condition.hpp"
#include "libxr_system.hpp"

// ESP-IDF 的 DriverAsyncCondition 节拍源；由 system/freertos 的 PlatformInit
// 通过 Detail::RegisterEspTickHook() 注册。
// ESP-IDF tick source of DriverAsyncCondition; registered from system/freertos
// PlatformInit via Detail::RegisterEspTickHook().

#if defined(ESP_PLATFORM)

#include "esp_attr.h"
#include "esp_freertos_hooks.h"

static_assert(configTICK_RATE_HZ <= 1000U && 1000U % configTICK_RATE_HZ == 0U,
              "DriverAsyncCondition needs a whole-millisecond tick period");

extern "C" bool spi_flash_cache_enabled(void);

namespace LibXR::Detail
{
/// 见 system/freertos/libxr_system.cpp 的 PlatformInit / See PlatformInit in
/// system/freertos/libxr_system.cpp
void RegisterEspTickHook();
}  // namespace LibXR::Detail

namespace
{
/**
 * @brief IDF 的 tick 钩子（CPU 0 的节拍中断）/ IDF tick hook (CPU 0's tick interrupt)
 * @note 节拍中断是 IRAM 中断，闪存操作期间（cache 关）也会运行，钩子因此放在 IRAM。
 *       那时跳过这个节拍：OnTick 和各个等待的函数在闪存里。跳过只会让等待变长。
 *       The tick interrupt is an IRAM interrupt and also runs during flash operations
 *       (cache off), so the hook lives in IRAM. Such a tick is skipped then: OnTick and
 *       the functions of the waits are in flash. Skipping only makes a wait longer.
 */
void IRAM_ATTR TickHook()
{
  if (!spi_flash_cache_enabled())
  {
    return;
  }
  LibXR::DriverAsyncCondition::OnTick(portTICK_PERIOD_MS);
}
}  // namespace

void LibXR::Detail::RegisterEspTickHook()
{
  // 只挂 CPU 0：双核时每个核各有节拍中断，钩子挂两核会计两次；CPU 0 推进 xTickCount
  // （IDF port_systick.c 的 xPortSysTickHandler）。
  // CPU 0 only: with two cores each has its own tick interrupt and a hook on both would
  // count twice; CPU 0 advances xTickCount (xPortSysTickHandler in IDF's
  // port_systick.c).
  REQUIRE(esp_register_freertos_tick_hook_for_cpu(TickHook, 0) == ESP_OK);
  DriverAsyncCondition::RegisterTickSource();
}

#endif
