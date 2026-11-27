#pragma once

#include <stdint.h>

#include "tx_api.h"

namespace LibXR
{
typedef TX_MUTEX libxr_mutex_handle;
typedef TX_SEMAPHORE libxr_semaphore_handle;
typedef TX_THREAD* libxr_thread_handle;

namespace Detail
{
/**
 * @brief 将有限毫秒等待向上换算为 ThreadX tick / Round a finite millisecond wait up to
 *        ticks.
 * @note 超出范围时取最大有限 tick，避免使用永久等待值。
 *       Saturate at the largest finite tick count, excluding the forever sentinel.
 */
[[nodiscard]] inline ULONG MillisecondsToThreadXTicks(uint32_t milliseconds)
{
  const uint64_t ticks =
      (static_cast<uint64_t>(milliseconds) * TX_TIMER_TICKS_PER_SECOND + 999U) / 1000U;
  return ticks >= static_cast<uint64_t>(TX_WAIT_FOREVER) ? TX_WAIT_FOREVER - 1U
                                                         : static_cast<ULONG>(ticks);
}
}  // namespace Detail

/**
 * @brief  平台初始化函数
 *         Platform initialization function
 * @param  timer_pri  定时器任务的优先级（默认值 2）
 *                    Timer task priority (default: 2)
 * @param  timer_stack_depth  定时器任务的栈深度（默认值 512）
 *                            Timer task stack depth (default: 512)
 *
 * @details
 * 该函数用于初始化 ThreadX 相关资源，如定时器任务。
 * 它设置定时器任务的优先级 `timer_pri` 和栈深度 `timer_stack_depth`。
 *
 * This function initializes ThreadX-related resources, such as the timer task.
 * It sets the timer task priority (`timer_pri`) and stack depth (`timer_stack_depth`).
 */
void PlatformInit(uint32_t timer_pri = 2,
                  uint32_t timer_stack_depth = 512);  // NOLINT

/**
 * @brief DriverAsyncCondition 的节拍源 / Tick source of
 *        DriverAsyncCondition
 *
 * 本目录的 tick.cpp 定义 void __wrap__tx_timer_interrupt(void)：
 * __real__tx_timer_interrupt() 后 OnTick(1000 / TX_TIMER_TICKS_PER_SECOND)（静态断言
 * 整除且不大于 1000）；静态构造调用 RegisterTickSource。CMakeLists.txt 加
 * -Wl,--wrap=_tx_timer_interrupt。不同编译单元（源码）：ST 的 Cortex-M4 移植由
 * tx_initialize_low_level.s 的 SysTick_Handler 调用（BL，第 230 行），定义在
 * ports/cortex_m4/gnu/src/tx_timer_interrupt.S；HPM 由 ports/common/tx_cpu_ts_setup.c
 * 的 system_tick_isr（GPTMR，第 170 行）调用，定义在 ports/riscv32/gnu/src/
 * tx_timer_interrupt.S。定义是汇编，不涉及 LTO。优先级：ST 移植写 SHPR3 = 0x40FF0000，
 * SysTick 为 0x40（第 171 行），ThreadX 没有系统调用上限；RequestPoll 合法。
 * tick.cpp in this directory defines void __wrap__tx_timer_interrupt(void):
 * __real__tx_timer_interrupt(), then OnTick(1000 / TX_TIMER_TICKS_PER_SECOND) (statically
 * asserted to divide and to be at most 1000); its static construction calls
 * RegisterTickSource. CMakeLists.txt adds -Wl,--wrap=_tx_timer_interrupt. Separate units
 * (source): ST's Cortex-M4 port calls it from SysTick_Handler in
 * tx_initialize_low_level.s (BL, line 230), defined in
 * ports/cortex_m4/gnu/src/tx_timer_interrupt.S; HPM calls it from system_tick_isr in
 * ports/common/tx_cpu_ts_setup.c (GPTMR, line 170), defined in
 * ports/riscv32/gnu/src/tx_timer_interrupt.S. The definitions are assembly, so LTO does
 * not apply. Priority: the ST port writes SHPR3 = 0x40FF0000, SysTick 0x40 (line 171);
 * ThreadX has no system call ceiling; RequestPoll is legal.
 */
}  // namespace LibXR
