#pragma once

#include <atomic>
#include <climits>
#include <cstring>

#include "libxr_mem.hpp"
#include "libxr_system.hpp"
#include "libxr_time.hpp"
#include "logger.hpp"

namespace LibXR
{
/**
 * @brief  线程管理类，封装 POSIX 线程创建和调度
 *         Thread management class encapsulating POSIX thread creation and scheduling
 */
class Thread
{
 public:
  /**
   * @brief  线程优先级枚举
   *         Enumeration for thread priorities
   */
  enum class Priority : uint8_t
  {
    IDLE,      ///< 空闲优先级 / Idle priority
    LOW,       ///< 低优先级 / Low priority
    MEDIUM,    ///< 中等优先级 / Medium priority
    HIGH,      ///< 高优先级 / High priority
    REALTIME,  ///< 实时优先级 / Realtime priority
    NUMBER,    ///< 优先级数量 / Number of priority levels
  };

  /**
   * @brief  默认构造函数，初始化空线程
   *         Default constructor initializing an empty thread
   */
  Thread() {};

  /**
   * @brief  通过 POSIX 线程句柄创建线程对象
   *         Constructor to create a thread object from a POSIX thread handle
   * @param  handle POSIX 线程句柄 / POSIX thread handle
   */
  Thread(libxr_thread_handle handle) : thread_handle_(handle) {};

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
   * 该方法基于 POSIX `pthread_create()` 创建新线程，执行 `function` 并传递 `arg`
   * 作为参数。栈大小取 `stack_depth` 与 `PTHREAD_STACK_MIN` 中的较大值。`SCHED_FIFO`
   * 的优先级范围足够时，线程使用 `SCHED_FIFO` 策略，优先级为最低优先级加 `priority`；
   * 范围不足或设置优先级失败时使用默认调度策略。
   * 按这些属性创建失败时，改用默认属性重试一次；进程内第一次重试时输出一条警告。
   *
   * This method creates a new thread using POSIX `pthread_create()`, executing `function`
   * with `arg` as the argument. The stack size is the larger of `stack_depth` and
   * `PTHREAD_STACK_MIN`. When the `SCHED_FIFO` priority range is large enough, the thread
   * uses `SCHED_FIFO` with the minimum priority plus `priority`; otherwise, or when
   * setting the priority fails, it uses the default policy. If creation with these
   * attributes fails, it is retried once with default attributes; the first such retry
   * in the process logs a warning.
   */
  template <typename ArgType>
  void Create(ArgType arg, void (*function)(ArgType arg), const char* name,
              size_t stack_depth, Thread::Priority priority)
  {
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    ConfigureAttributes(attr, stack_depth, priority);

    // 线程数据封装类 / Thread data encapsulation class
    class ThreadBlock
    {
     public:
      // 存储线程函数、参数和名称 / Store the thread function, argument and name
      ThreadBlock(decltype(function) fun, ArgType arg, const char* name)
          : fun_(fun), arg_(arg)
      {
        std::memset(name_, 0, sizeof(name_));
        if (name != nullptr)
        {
          std::strncpy(name_, name, sizeof(name_) - 1);
        }
      }

      // 线程入口：执行用户函数后释放 ThreadBlock，返回 nullptr / Thread entry: runs the
      // user function, frees the ThreadBlock and returns nullptr
      static void* Port(void* arg)
      {
        ThreadBlock* block = static_cast<ThreadBlock*>(arg);

        if (block->name_[0] != '\0')
        {
          pthread_setname_np(pthread_self(), block->name_);
        }

        block->fun_(block->arg_);
        delete block;
        return static_cast<void*>(nullptr);
      }

      decltype(function) fun_;  // 线程执行的函数 / Function executed by the thread
      ArgType arg_;             // 线程函数的参数 / Argument passed to the thread function
      char name_[16];           // 线程名称 / Thread name
    };

    auto block = new ThreadBlock(function, arg, name);

    // 创建线程
    int ans = pthread_create(&this->thread_handle_, &attr, ThreadBlock::Port, block);

    if (ans != 0)
    {
      if (FirstDefaultAttributeRetry())
      {
        XR_LOG_WARN(
            "Failed to create thread: %s (%s), retrying with default attributes. Later "
            "threads that fail the same way retry without this warning.",
            name, strerror(ans));
      }

      // 完全使用系统默认属性（attr = nullptr）
      ans = pthread_create(&this->thread_handle_, nullptr, ThreadBlock::Port, block);

      if (ans != 0)
      {
        XR_LOG_ERROR("Failed to create thread: %s (%s)", name, strerror(ans));
        delete block;
      }
    }

    pthread_attr_destroy(&attr);
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
   * @brief Waits for this POSIX thread to terminate and releases its resources.
   * @return `OK` on success, otherwise `FAILED`.
   */
  ErrorCode Join();

  /**
   * @brief  线程对象转换为 POSIX 线程句柄
   *         Converts the thread object to a POSIX thread handle
   * @return POSIX 线程句柄 / POSIX thread handle
   */
  operator libxr_thread_handle() { return thread_handle_; }

 private:
  /**
   * @brief  记录进程内第一次改用默认属性重试 / Records the first retry with default
   *         attributes in the process
   * @return 第一次调用返回 true，之后返回 false /
   *         true on the first call, false afterwards
   */
  static bool FirstDefaultAttributeRetry()
  {
    static std::atomic_flag retried = ATOMIC_FLAG_INIT;
    return !retried.test_and_set(std::memory_order_relaxed);
  }

  static void ConfigureAttributes(pthread_attr_t& attr, size_t stack_depth,
                                  Thread::Priority priority)
  {
    const size_t stack_size =
        LibXR::max(static_cast<size_t>(PTHREAD_STACK_MIN), stack_depth);
    pthread_attr_setstacksize(&attr, stack_size);
    ConfigureScheduling(attr, priority);
  }

  static void ConfigureScheduling(pthread_attr_t& attr, Thread::Priority priority)
  {
    const int min_priority = sched_get_priority_min(SCHED_FIFO);
    const int max_priority = sched_get_priority_max(SCHED_FIFO);
    if (max_priority - min_priority < static_cast<int>(Priority::REALTIME))
    {
      XR_LOG_WARN(
          "SCHED_FIFO not supported or insufficient range. Using default policy.");
      pthread_attr_setinheritsched(&attr, PTHREAD_INHERIT_SCHED);
      return;
    }

    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&attr, SCHED_FIFO);

    struct sched_param sp;
    Memory::FastSet(&sp, 0, sizeof(sp));
    sp.sched_priority = min_priority + static_cast<int>(priority);

    if (pthread_attr_setschedparam(&attr, &sp) == 0)
    {
      return;
    }

    XR_LOG_WARN("Failed to set thread priority. Falling back to default policy.");
    pthread_attr_setschedpolicy(&attr, SCHED_OTHER);
    pthread_attr_setinheritsched(&attr, PTHREAD_INHERIT_SCHED);
  }

  libxr_thread_handle thread_handle_;  ///< POSIX 线程句柄 / POSIX thread handle
};

}  // namespace LibXR
