#pragma once

#include "libxr.hpp"

#include DEF2STR(LIBXR_CH32_CONFIG_FILE)

namespace LibXR
{

/**
 * @brief CH32 硬件计数器时间基准 / CH32 hardware-counter timebase
 *
 * @note BSP 必须预先将 SysTick 配置为 HCLK 驱动的 64 位自由运行向上计数器。
 *       使用期间不得停止、清零计数器或改变 HCLK；HCLK 不低于 1 MHz。
 *       The BSP must configure SysTick as a 64-bit, HCLK-driven, free-running
 *       up-counter before initialization. The counter must not be stopped or reset,
 *       and HCLK must remain unchanged and at least 1 MHz during use.
 * @note 微秒和毫秒均由硬件计数值换算，不依赖 SysTick 中断的软件计数。
 *       初始化完成后可在任务或中断中读取，比较值和 tick 中断由 BSP 管理。
 *       Microseconds and milliseconds are derived from the hardware counter without
 *       a software interrupt count. Reads are allowed from tasks or interrupts after
 *       initialization; the BSP manages compare deadlines and tick interrupts.
 * @note 裸机（system/none）时 DriverAsyncCondition 的节拍由板级工程给：启动时调用一次
 *       DriverAsyncCondition::RegisterTickSource()，之后每毫秒调用一次
 *       DriverAsyncCondition::OnTick()（间隔固定，elapsed_ms 不变）。本类不提供节拍。
 *       FreeRTOS、ThreadX 下节拍来自系统层，板级工程不调用。
 * @note Bare-metal (system/none), the board project provides DriverAsyncCondition's tick:
 *       it calls DriverAsyncCondition::RegisterTickSource() once at start and then
 *       DriverAsyncCondition::OnTick() every millisecond (a fixed interval, elapsed_ms
 *       unchanged). This class provides no tick. Under FreeRTOS or ThreadX the tick
 *       comes from the system layer and the board project does not call it.
 */
class CH32Timebase : public Timebase
{
 public:
  /**
   * @brief 构造函数 / Constructor
   *
   * 缓存计数器时钟频率，配置时间戳回绕范围并标记时间基就绪。
   * Cache the counter clock frequency, configure timestamp wrap ranges, and mark
   * the timebase ready.
   */
  CH32Timebase();
};

}  // namespace LibXR
