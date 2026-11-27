#include "driver_async_condition.hpp"
#include "libxr_system.hpp"

// DriverAsyncCondition 的节拍源，见 libxr_system.hpp。
// Tick source of DriverAsyncCondition, see libxr_system.hpp.

static_assert(TX_TIMER_TICKS_PER_SECOND <= 1000U &&
                  1000U % TX_TIMER_TICKS_PER_SECOND == 0U,
              "DriverAsyncCondition needs a whole-millisecond tick period");

// NOLINTBEGIN(bugprone-reserved-identifier,readability-identifier-naming)
extern "C" void __real__tx_timer_interrupt(void);

/// _tx_timer_interrupt 的链接时包装 / Link-time wrap of _tx_timer_interrupt
extern "C" void __wrap__tx_timer_interrupt(void)
{
  __real__tx_timer_interrupt();
  LibXR::DriverAsyncCondition::OnTick(1000U / TX_TIMER_TICKS_PER_SECOND);
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
