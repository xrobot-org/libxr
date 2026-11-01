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
 *       初始化完成后可在任务或中断中读取。
 *       Microseconds and milliseconds are derived from the hardware counter without
 *       a software interrupt count. Reads are allowed from tasks or interrupts after
 *       initialization.
 * @note 毫秒节拍（DriverAsyncCondition）：使用 FreeRTOS 时比较值和 SysTick
 *       中断归 FreeRTOS 移植层，节拍来自 vApplicationTickHook（configUSE_TICK_HOOK
 *       须为 1）；不使用 RTOS 时本类在构造时打开 SysTick 比较中断作为 1 kHz 节拍并
 *       定义 SysTick_Handler，HCLK 须为 1 kHz 的整数倍。
 *       Millisecond tick (DriverAsyncCondition): with FreeRTOS the compare value and the
 *       SysTick interrupt belong to the FreeRTOS port and the tick comes from
 *       vApplicationTickHook (configUSE_TICK_HOOK must be 1); without an RTOS this class
 *       enables the SysTick compare interrupt as the 1 kHz tick at construction and
 *       defines SysTick_Handler; HCLK must be a multiple of 1 kHz.
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
