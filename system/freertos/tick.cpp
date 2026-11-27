#include "driver_async_condition.hpp"
#include "libxr_system.hpp"

// DriverAsyncCondition 的节拍源，见 libxr_system.hpp。
// Tick source of DriverAsyncCondition, see libxr_system.hpp.

static_assert(configTICK_RATE_HZ <= 1000U && 1000U % configTICK_RATE_HZ == 0U,
              "DriverAsyncCondition needs a whole-millisecond tick period");

#if !defined(ESP_PLATFORM)

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
