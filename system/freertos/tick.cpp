#include "driver_async_condition.hpp"
#include "libxr_system.hpp"

// DriverAsyncCondition 的节拍源，见 libxr_system.hpp。
// Tick source of DriverAsyncCondition, see libxr_system.hpp.

static_assert(configTICK_RATE_HZ <= 1000U && 1000U % configTICK_RATE_HZ == 0U,
              "DriverAsyncCondition needs a whole-millisecond tick period");

#if defined(ESP_PLATFORM)

#include "esp_attr.h"
#include "esp_freertos_hooks.h"

extern "C" bool spi_flash_cache_enabled(void);

namespace LibXR::Detail
{
/// 见 libxr_system.cpp 的 PlatformInit / See PlatformInit in libxr_system.cpp
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
  // 只挂 CPU 0：双核时每个核各有节拍中断，挂两核会计两次；CPU 0 推进 xTickCount
  // （IDF port_systick.c 的 xPortSysTickHandler）。
  // CPU 0 only: with two cores each has its own tick interrupt and a hook on both would
  // count twice; CPU 0 advances xTickCount (xPortSysTickHandler in IDF's
  // port_systick.c).
  REQUIRE(esp_register_freertos_tick_hook_for_cpu(TickHook, 0) == ESP_OK);
  DriverAsyncCondition::RegisterTickSource();
}

#else

extern "C" BaseType_t __real_xTaskIncrementTick(void);

/**
 * @brief xTaskIncrementTick 的链接时包装 / Link-time wrap of xTaskIncrementTick
 * @note 返回值请求上下文切换，原样传出。 / The return value requests the context
 *       switch and is passed on unchanged.
 */
extern "C" BaseType_t __wrap_xTaskIncrementTick(void)
{
  const BaseType_t switch_required = __real_xTaskIncrementTick();
  LibXR::DriverAsyncCondition::OnTick(portTICK_PERIOD_MS);
  return switch_required;
}

namespace
{
/// 本目标文件被包装引用时随之链接，静态构造登记节拍源 / Linked along with the wrap
/// that references this object file; its static construction registers the tick source
struct TickSourceRegistration
{
  TickSourceRegistration() { LibXR::DriverAsyncCondition::RegisterTickSource(); }
};
#if defined(__GNUC__) || defined(__clang__)
__attribute__((init_priority(101)))
#endif
const TickSourceRegistration TICK_SOURCE_REGISTRATION;
}  // namespace

#endif
