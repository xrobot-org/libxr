#pragma once

#include "libxr_def.hpp"
#include "libxr_system.hpp"

namespace LibXR
{

/**
 * @brief  信号量类，实现线程同步机制
 *         Semaphore class implementing thread synchronization
 *
 * @details
 * 信号量（Semaphore）用于线程间同步，支持普通等待和中断回调模式。
 * 它允许多个线程协调访问共享资源，并支持从 ISR（中断服务例程）中安全调用。
 *
 * The semaphore is used for thread synchronization, supporting both regular
 * wait operations and ISR (Interrupt Service Routine) callback mode.
 * It enables multiple threads to coordinate access to shared resources
 * and provides safe operations from an ISR.
 */
class Semaphore
{
 public:
  /**
   * @brief  构造一个信号量对象
   *         Constructs a semaphore object
   * @param  init_count 信号量的初始值 / Initial count of the semaphore
   *
   * @details
   * 信号量的初始值 `init_count` 定义了可用资源的初始数量。
   * 该信号量可用于控制多个线程对共享资源的访问。
   *
   * The initial count `init_count` defines the initial number of available resources.
   * This semaphore can be used to control access to shared resources by multiple threads.
   */
  Semaphore(uint32_t init_count = 0);

  /**
   * @brief  析构信号量对象，释放资源
   *         Destroys the semaphore object and releases resources
   */
  ~Semaphore();

  /**
   * @brief  释放（增加）信号量
   *         Releases (increments) the semaphore
   *
   * @details
   * 该方法增加信号量的值，表示增加了一个可用资源。
   * 若有等待的线程，该方法可能会解除其中一个线程的阻塞状态。
   *
   * This method increments the semaphore count, indicating that a resource
   * has been made available. If there are waiting threads, one of them may be unblocked.
   */
  void Post();

  /**
   * @brief  从中断回调中释放（增加）信号量
   *         Releases (increments) the semaphore from an ISR (Interrupt Service Routine)
   * @param  in_isr 是否在 ISR（中断服务例程）中调用 / Whether it is called from an ISR
   *
   * @details
   * 该方法在中断服务程序（ISR）中使用，确保安全地增加信号量。
   * 若 `in_isr` 为 `true`，则执行适用于中断环境的信号量释放操作。
   *
   * This method is used in an ISR (Interrupt Service Routine) to safely increment
   * the semaphore. If `in_isr` is `true`, an ISR-safe semaphore release operation is
   * performed.
   */
  void PostFromCallback(bool in_isr);

  /**
   * @brief  等待（减少）信号量
   *         Waits (decrements) the semaphore
   * @param  timeout 超时时间（毫秒），默认 `UINT32_MAX` / Timeout in milliseconds,
   *         default `UINT32_MAX`
   * @return 获得信号量返回 `ErrorCode::OK`，超时返回 `ErrorCode::TIMEOUT`；
   *         ThreadX 后端出错时返回 `ErrorCode::FAILED` / `ErrorCode::OK` when acquired,
   *         `ErrorCode::TIMEOUT` on timeout; `ErrorCode::FAILED` on a ThreadX error
   *
   * @details
   * 该方法尝试减少信号量的值，表示线程正在占用一个资源。
   * 如果信号量的值为 0，调用线程将进入阻塞状态，直到信号量可用或超时。
   *
   * This method attempts to decrement the semaphore count, indicating that
   * a resource is being used by the calling thread.
   * If the semaphore count is 0, the calling thread is blocked until the
   * semaphore becomes available or the timeout expires.
   */
  ErrorCode Wait(uint32_t timeout = UINT32_MAX);

  /**
   * @brief  获取当前信号量的值
   *         Gets the current value of the semaphore
   * @return 当前信号量值 / The current semaphore value
   *
   * @details
   * 该方法返回信号量当前的计数值，表示可用资源的数量。
   *
   * This method returns the current semaphore count, representing
   * the number of available resources.
   */
  size_t Value();

 private:
  libxr_semaphore_handle semaphore_handle_;  ///< 信号量句柄 / Semaphore handle
};

}  // namespace LibXR
