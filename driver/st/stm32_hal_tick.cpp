#include "main.h"

#if defined(LIBXR_SYSTEM_none)

#include "driver_async_condition.hpp"
#include "stm32_timebase.hpp"

// 裸机时 DriverAsyncCondition 的节拍源，见 stm32_timebase.hpp。
// Bare-metal tick source of DriverAsyncCondition, see stm32_timebase.hpp.

// NOLINTBEGIN(bugprone-reserved-identifier,readability-identifier-naming)
extern "C" void __real_HAL_IncTick(void);

extern "C" void __wrap_HAL_IncTick(void)
{
  __real_HAL_IncTick();
  LibXR::DriverAsyncCondition::OnTick(static_cast<uint32_t>(HAL_GetTickFreq()));
}
// NOLINTEND(bugprone-reserved-identifier,readability-identifier-naming)

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
