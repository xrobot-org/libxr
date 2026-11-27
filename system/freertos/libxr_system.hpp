#pragma once

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

// ESP Xtensa 头文件的宏会与 USB 端点类型名称冲突。
// ESP Xtensa headers expose a macro that conflicts with the USB endpoint type.
#if defined(ESP_PLATFORM) && defined(INTERRUPT)
#undef INTERRUPT
#endif

namespace LibXR
{
typedef SemaphoreHandle_t libxr_mutex_handle;
typedef SemaphoreHandle_t libxr_semaphore_handle;
typedef TaskHandle_t libxr_thread_handle;

static_assert(sizeof(TickType_t) >= sizeof(uint32_t),
              "FreeRTOS TickType_t must hold LibXR millisecond timestamps");

extern uint32_t libxr_freertos_timebase_tick_offset;

/**
 * @brief  平台初始化函数
 *         Platform initialization function
 * @param  timer_pri  定时器任务的优先级（默认值 2）
 *                    Timer task priority (default: 2)
 * @param  timer_stack_depth  定时器任务的栈深度（默认值 512）
 *                            Timer task stack depth (default: 512)
 *
 * @details
 * 该函数用于初始化 FreeRTOS 相关资源，如定时器任务。
 * 它设置定时器任务的优先级 `timer_pri` 和栈深度 `timer_stack_depth`。
 *
 * This function initializes FreeRTOS-related resources, such as the timer task.
 * It sets the timer task priority (`timer_pri`) and stack depth (`timer_stack_depth`).
 */
void PlatformInit(uint32_t timer_pri = 2,
                  uint32_t timer_stack_depth = 512);  // NOLINT

/**
 * @brief DriverAsyncCondition 的节拍源 / Tick source of
 *        DriverAsyncCondition
 *
 * 非 ESP：本目录的 tick.cpp 定义
 * BaseType_t __wrap_xTaskIncrementTick(void)，先 r = __real_xTaskIncrementTick()，再
 * DriverAsyncCondition::OnTick(portTICK_PERIOD_MS)，返回 r（它请求上下文切换，必须原样
 * 传出）；静态构造调用 RegisterTickSource。CMakeLists.txt 加
 * -Wl,--wrap=xTaskIncrementTick，并让 tasks.c 不参与 LTO（与 hal_pcd.c 相同的做法）。
 * 两者都只在本目录，裸机工程看不到（否则 __real_xTaskIncrementTick 是未定义引用）。
 * --wrap 只替换未定义的引用：各移植的节拍中断与 tasks.c 是不同的编译单元（源码）：
 * ARM_CM3 port.c:435、ARM_CM4F port.c:526、ARM_CM7/r0p1 port.c:487、ARM_CM33_NTZ
 * port.c:977，WCH RISC-V port.c:322（SysTick_Handler），HPM 移植的 hpm_port.c 和
 * portASM.S。xTaskResumeAll 补回挂起期间的节拍时在 tasks.c 内部调用，不经包装，
 * 不重复计数（节拍中断那次已经经过包装）。
 * 优先级（源码）：CM3/CM7 的 SysTick 为 configKERNEL_INTERRUPT_PRIORITY
 * （port.c:66、63），CM4F/CM33 为 portMIN_INTERRUPT_PRIORITY（port.c:63、88），都是最低；
 * xTaskIncrementTick 在 portDISABLE_INTERRUPTS（BASEPRI 抬到
 * configMAX_SYSCALL_INTERRUPT_PRIORITY）之内调用，所以 OnTick 运行时不高于系统调用
 * 上限，且屏蔽上限以下的中断，必须短。WCH 移植 SysTick 优先级 0xf0（port.c:202），
 * 最低。FreeRTOS 的节拍从不高于上限；RequestPoll（发布 + 挂起）在其中合法。不需要
 * configUSE_TICK_HOOK。
 * ESP-IDF（ESP_PLATFORM）：不包装（IDF 的 FreeRTOS 是 IDF 组件）；PlatformInit 调用
 * driver/esp/esp_tick.cpp 里的 RegisterEspTickHook() 挂接 CPU 0 tick 钩子并调用
 * RegisterTickSource。只挂 CPU 0：双核时每个核各有节拍中断，钩子挂两核会计两次；CPU 0
 * 是推进 xTickCount 的核（IDF FreeRTOS 的 xTaskIncrementTick 只在 CPU 0 计数）。
 * 钩子在节拍中断里运行（1 级，低于系统调用上限），频率 CONFIG_FREERTOS_HZ（IDF 默认
 * 100 Hz）。
 * Not ESP: tick.cpp in this directory defines BaseType_t __wrap_xTaskIncrementTick(void):
 * r = __real_xTaskIncrementTick(), then DriverAsyncCondition::OnTick(portTICK_PERIOD_MS),
 * return r (it requests the context switch and must be passed on unchanged); its static
 * construction calls RegisterTickSource. CMakeLists.txt adds
 * -Wl,--wrap=xTaskIncrementTick and keeps tasks.c out of LTO (as for hal_pcd.c). Both
 * live only in this directory, so bare-metal projects never see them (otherwise
 * __real_xTaskIncrementTick would be an undefined reference). --wrap replaces undefined
 * references only; every port's tick interrupt is a different unit from tasks.c
 * (source): ARM_CM3 port.c:435, ARM_CM4F port.c:526, ARM_CM7/r0p1 port.c:487,
 * ARM_CM33_NTZ port.c:977, WCH RISC-V port.c:322 (SysTick_Handler), and hpm_port.c and
 * portASM.S of the HPM port. When xTaskResumeAll replays ticks pended
 * while the scheduler was suspended it calls inside tasks.c, not through the wrapper, so
 * nothing counts twice (the tick interrupt's call already went through it).
 * Priority (source): SysTick is configKERNEL_INTERRUPT_PRIORITY on CM3/CM7 (port.c:66,
 * 63) and portMIN_INTERRUPT_PRIORITY on CM4F/CM33 (port.c:63, 88), the lowest in each
 * case; xTaskIncrementTick is called inside portDISABLE_INTERRUPTS (BASEPRI raised to
 * configMAX_SYSCALL_INTERRUPT_PRIORITY), so OnTick runs no higher than the system call
 * ceiling, with the interrupts below it masked, and must be short. The WCH port puts
 * SysTick at 0xf0 (port.c:202), the lowest. FreeRTOS's tick is never above the ceiling;
 * RequestPoll (publish + pend) is legal in it. configUSE_TICK_HOOK is not needed.
 * ESP-IDF (ESP_PLATFORM): no wrap (IDF's FreeRTOS is an IDF component); PlatformInit
 * calls RegisterEspTickHook() in driver/esp/esp_tick.cpp to attach the CPU 0 tick hook
 * and call RegisterTickSource. CPU 0 only: with two cores each has its own tick
 * interrupt, and a hook on both would count twice; CPU 0 is the core that advances
 * xTickCount (IDF FreeRTOS counts the tick in xTaskIncrementTick on CPU 0 only). The
 * hook runs in the tick interrupt (level 1, below the system call ceiling) at
 * CONFIG_FREERTOS_HZ (IDF default 100 Hz).
 */
}  // namespace LibXR
