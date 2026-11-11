#pragma once

#include <pthread.h>

#include <atomic>
#include <cstdint>

namespace LibXR
{
/// 互斥锁句柄：POSIX 互斥锁 / Mutex handle: a POSIX mutex
typedef pthread_mutex_t libxr_mutex_handle;
/// 基于 futex 的信号量，状态仅为一个原子计数 / Futex-based semaphore whose state is one
/// atomic count
struct libxr_linux_futex_semaphore
{
  std::atomic<uint32_t> count;  ///< 当前可用的信号量计数 / Currently available count
};
/// 信号量句柄：指向 futex 信号量的指针，由 Semaphore 构造时分配 / Semaphore handle: a
/// pointer to the futex semaphore, allocated when the Semaphore is constructed
typedef libxr_linux_futex_semaphore* libxr_semaphore_handle;
/// 线程句柄：POSIX 线程 ID / Thread handle: a POSIX thread id
typedef pthread_t libxr_thread_handle;

/**
 * @brief  平台初始化函数
 *         Platform initialization function
 *
 * @param  timer_pri  定时器任务的优先级（默认值 2）
 *                    Timer task priority (default: 2)
 * @param  timer_stack_depth  定时器任务的栈深度（默认值 65536）
 *                            Timer task stack depth (default: 65536)
 *
 * @details
 * 该函数用于初始化 Linux 主机运行时，包括：
 * - 定时器线程默认参数
 * - 标准输入输出端口
 * - 终端模式
 * - 后台 STDIO 线程
 * 在使用依赖主机运行时的 `LibXR` 组件之前，应调用该函数。
 *
 * This function initializes the Linux host runtime, including:
 * - default timer-thread parameters
 * - STDIO ports
 * - terminal mode
 * - background STDIO threads
 * It should be called before using `LibXR` components that depend on the host runtime.
 *
 */
void PlatformInit(uint32_t timer_pri = 2, uint32_t timer_stack_depth = 65536);
}  // namespace LibXR
