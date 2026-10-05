#pragma once

#include "libxr_def.hpp"
#include "libxr_system.hpp"
#include "libxr_time.hpp"

namespace LibXR
{

/**
 * @brief  线程管理类，提供线程的创建、调度和时间管理功能
 *         Thread management class that provides thread creation, scheduling, and time
 * management
 */
class Thread
{
 public:
  /**
   * @brief  线程优先级枚举
   *         Enumeration for thread priorities
   */
  enum class Priority
  {
    IDLE = 0,      ///< 空闲优先级 / Idle priority
    LOW = 0,       ///< 低优先级 / Low priority
    MEDIUM = 0,    ///< 中等优先级 / Medium priority
    HIGH = 0,      ///< 高优先级 / High priority
    REALTIME = 0,  ///< 实时优先级 / Realtime priority
    NUMBER = 1,    ///< 优先级数量 / Number of priority levels
  };

  /**
   * @brief  默认构造函数，初始化空线程对象
   *         Default constructor initializing an empty thread object
   */
  Thread() {};

  /**
   * @brief  通过线程句柄创建线程对象
   *         Constructor to create a thread object from a thread handle
   * @param  handle 线程句柄 / Thread handle
   */
  Thread(libxr_thread_handle handle) : thread_handle_(handle) {};

  /**
   * @brief  创建并执行新线程
   *         Creates and executes a new thread
   * @tparam ArgType 线程函数的参数类型 / The type of argument for the thread function
   * @param  arg 线程函数的参数 / Argument for the thread function
   * @param  function 线程执行的函数 / Function executed by the thread
   * @param  name 线程名称（未使用） / Thread name (unused)
   * @param  stack_depth 线程栈大小（未使用） / Stack size of the thread (unused)
   * @param  priority 线程优先级（未使用） / Thread priority (unused)
   *
   * @details
   * 此函数是一个简化版本的线程创建，仅执行 `function(arg)`，并且只允许创建一次。
   * 由于 `name`、`stack_depth` 和 `priority` 未实际使用，主要用于测试或占位用途。
   *
   * This function is a simplified version of thread creation, executing only
   * `function(arg)`, and it ensures that the thread is created only once. The parameters
   * `name`, `stack_depth`, and `priority` are unused and mainly serve as placeholders or
   * for testing purposes.
   */
  template <typename ArgType>
  void Create(ArgType arg, void (*function)(ArgType arg), const char* name,
              size_t stack_depth, Thread::Priority priority)
  {
    UNUSED(name);
    UNUSED(stack_depth);
    UNUSED(priority);

    static bool created = false;
    ASSERT(created == false);
    created = true;
    UNUSED(created);

    function(arg);
  }

  /**
   * @brief  获取当前线程对象
   *         Gets the current thread object
   * @return 当前线程对象 / The current thread object
   */
  static Thread Current(void);

  /**
   * @brief  获取当前系统时间（毫秒）
   *         Gets the current system time in milliseconds
   * @return 当前时间（毫秒） / Current time in milliseconds
   */
  static uint32_t GetTime();

  /**
   * @brief  让线程进入休眠状态
   *         Puts the thread to sleep
   * @param  milliseconds 休眠时间（毫秒） / Sleep duration in milliseconds
   */
  static void Sleep(uint32_t milliseconds);

  /**
   * @brief  让线程休眠直到指定时间点
   *         Puts the thread to sleep until a specified time
   * @param  last_wakeup_time 上次唤醒时间 / Last wake-up time
   * @param  time_to_sleep 休眠时长（毫秒） / Sleep duration in milliseconds
   */
  static void SleepUntil(MillisecondTimestamp& last_wakeup_time, uint32_t time_to_sleep);

  /**
   * @brief  让出 CPU 以执行其他线程
   *         Yields CPU execution to allow other threads to run
   */
  static void Yield();

  /**
   * @brief  线程对象转换为线程句柄
   *         Converts the thread object to a thread handle
   * @return 线程句柄 / Thread handle
   */
  operator libxr_thread_handle() { return thread_handle_; }

 private:
  libxr_thread_handle thread_handle_ = 0;  ///< 线程句柄 / Thread handle
};

}  // namespace LibXR
