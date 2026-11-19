#pragma once

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

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
}  // namespace LibXR
