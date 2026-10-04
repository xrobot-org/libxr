#pragma once

#include "libxr_system.hpp"
#include "libxr_time.hpp"
#include "tx_api.h"

#define LIBXR_PRIORITY_STEP ((TX_MAX_PRIORITIES - 1U) / 5U)

namespace LibXR
{
/**
 * @brief  线程管理类，封装 ThreadX 任务创建和调度
 *         Thread management class encapsulating ThreadX task creation and scheduling
 */
class Thread
{
 public:
  /**
   * @brief 线程优先级枚举（ThreadX 数值越小优先级越高）
   *         Thread priority levels; ThreadX uses smaller values for higher priority
   */
  enum class Priority : UINT
  {
    IDLE = LIBXR_PRIORITY_STEP * 4U,    ///< 空闲优先级 / Idle priority
    LOW = LIBXR_PRIORITY_STEP * 3U,     ///< 低优先级 / Low priority
    MEDIUM = LIBXR_PRIORITY_STEP * 2U,  ///< 中等优先级 / Medium priority
    HIGH = LIBXR_PRIORITY_STEP * 1U,    ///< 高优先级 / High priority
    REALTIME = 1U,                      ///< 实时优先级 / Realtime priority
    NUMBER = 5                          ///< 优先级数量 / Number of priority levels
  };

  /**
   * @brief  默认构造函数，初始化空线程
   *         Default constructor initializing an empty thread
   */
  Thread() : thread_handle_(nullptr) {};

  /**
   * @brief  通过 ThreadX 线程句柄创建线程对象
   *         Constructor to create a thread object from a ThreadX thread handle
   * @param  handle ThreadX 线程句柄 / ThreadX thread handle
   */
  Thread(TX_THREAD* handle) : thread_handle_(handle) {};

  /**
   * @brief  创建新线程
   *         Creates a new thread
   * @tparam ArgType 线程函数的参数类型 / The type of argument for the thread function
   * @param  arg 线程函数的参数 / Argument for the thread function
   * @param  function 线程执行的函数 / Function executed by the thread
   * @param  name 线程名称 / Thread name
   * @param  stack_depth 线程栈大小（字节） / Stack size of the thread (bytes)
   * @param  priority 线程优先级 / Thread priority
   *
   * @details
   * 该方法基于 ThreadX `tx_thread_create()` 创建新线程，执行 `function` 并传递 `arg`
   * 作为参数。 线程优先级 `priority` 必须符合 ThreadX 配置的 `TX_MAX_PRIORITIES`
   * 约束。
   *
   * This method creates a new thread using ThreadX `tx_thread_create()`, executing
   * `function` with `arg` as the argument. The thread priority `priority` must adhere to
   * ThreadX configuration constraints defined by `TX_MAX_PRIORITIES`.
   */
  template <typename ArgType>
  void Create(ArgType arg, void (*function)(ArgType arg), const char* name,
              size_t stack_depth, Thread::Priority priority)
  {
    class ThreadBlock
    {
     public:
      ThreadBlock(decltype(function) fun, ArgType arg) : fun_(fun), arg_(arg) {}
      static void Port(ULONG ptr)
      {
        ThreadBlock* block = reinterpret_cast<ThreadBlock*>(ptr);
        block->fun_(block->arg_);
        delete block;
      }
      decltype(function) fun_;
      ArgType arg_;
    };

    auto* block = new ThreadBlock(function, arg);
    auto stack_buffer = new ULONG[stack_depth / sizeof(ULONG)];
    thread_handle_ = new TX_THREAD;

    UINT status = tx_thread_create(
        thread_handle_, const_cast<char*>(name), ThreadBlock::Port, ULONG(block),
        stack_buffer, stack_depth, static_cast<UINT>(priority),
        static_cast<UINT>(priority), TX_NO_TIME_SLICE, TX_AUTO_START);
    REQUIRE(status == TX_SUCCESS);
  }

  /**
   * @brief  获取当前线程对象
   *         Gets the current thread object
   * @return 当前线程对象 / The current thread object
   */
  static Thread Current(void) { return Thread(tx_thread_identify()); }

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
   * @note 向上取整到 tick，超出单次范围时取 TX_WAIT_FOREVER - 1 个 tick。
   *       Round up to ticks, saturating at TX_WAIT_FOREVER - 1 ticks.
   */
  static void Sleep(uint32_t milliseconds);

  /**
   * @brief  让线程休眠直到指定时间点
   *         Puts the thread to sleep until a specified time
   * @param  last_wakeup_time 上次唤醒时间 / Last wake-up time
   * @param  time_to_sleep 休眠时长（毫秒） / Sleep duration in milliseconds
   * @pre 用 GetTime 初始化计划时间；目标位于当前时间前后 2^31 毫秒以内。
   *      Initialize the schedule with GetTime. The target must be less than 2^31
   *      milliseconds before or after now for unambiguous modular ordering.
   * @note 保留计划唤醒时间；错过期限时不休眠，下一次仍按原周期推进。
   *       Keep the scheduled wake time; overdue calls do not sleep and retain the
   * cadence.
   */
  static void SleepUntil(MillisecondTimestamp& last_wakeup_time, uint32_t time_to_sleep);

  /**
   * @brief  让出 CPU 以执行其他线程
   *         Yields CPU execution to allow other threads to run
   */
  static void Yield();

  /**
   * @brief  线程对象转换为 ThreadX 线程句柄
   *         Converts the thread object to a ThreadX thread handle
   * @return ThreadX 线程句柄 / ThreadX thread handle
   */
  operator TX_THREAD*() { return thread_handle_; }

 private:
  libxr_thread_handle thread_handle_;
};
}  // namespace LibXR
