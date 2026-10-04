#pragma once

#include "libxr_system.hpp"
#include "libxr_time.hpp"
#include "timebase.hpp"

#define LIBXR_PRIORITY_STEP ((configMAX_PRIORITIES - 1) / 5)

namespace LibXR
{
/**
 * @brief  线程管理类，封装 FreeRTOS 任务创建和调度
 *         Thread management class encapsulating FreeRTOS task creation and scheduling
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
    IDLE = 0,                            ///< 空闲优先级 / Idle priority
    LOW = LIBXR_PRIORITY_STEP * 1,       ///< 低优先级 / Low priority
    MEDIUM = LIBXR_PRIORITY_STEP * 2,    ///< 中等优先级 / Medium priority
    HIGH = LIBXR_PRIORITY_STEP * 3,      ///< 高优先级 / High priority
    REALTIME = LIBXR_PRIORITY_STEP * 4,  ///< 实时优先级 / Realtime priority
    NUMBER = 5                           ///< 优先级数量 / Number of priority levels
  };

  /**
   * @brief  默认构造函数，初始化空线程
   *         Default constructor initializing an empty thread
   */
  Thread() {};

  /**
   * @brief  通过 FreeRTOS 线程句柄创建线程对象
   *         Constructor to create a thread object from a FreeRTOS thread handle
   * @param  handle FreeRTOS 线程句柄 / FreeRTOS thread handle
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
   * 该方法基于 FreeRTOS `xTaskCreate()` 创建新线程，执行 `function` 并传递 `arg`
   * 作为参数。 线程优先级 `priority` 必须符合 FreeRTOS 配置的 `configMAX_PRIORITIES`
   * 约束。
   *
   * This method creates a new thread using FreeRTOS `xTaskCreate()`, executing `function`
   * with `arg` as the argument. The thread priority `priority` must adhere to FreeRTOS
   * configuration constraints defined by `configMAX_PRIORITIES`.
   */
  template <typename ArgType>
  void Create(ArgType arg, void (*function)(ArgType arg), const char* name,
              size_t stack_depth, Thread::Priority priority)
  {
    static_assert(configMAX_PRIORITIES >= 6);

    class ThreadBlock
    {
     public:
      ThreadBlock(decltype(function) fun, ArgType arg) : fun_(fun), arg_(arg) {}

      static void Port(void* arg)
      {
        ThreadBlock* block = static_cast<ThreadBlock*>(arg);
        block->fun_(block->arg_);
        delete block;
      }

      decltype(function) fun_;
      ArgType arg_;
    };

    auto block = new ThreadBlock(function, arg);

    // LibXR 栈深度使用字节；xTaskCreate() 接收 StackType_t 个数。
    // LibXR stack depth is in bytes; xTaskCreate() consumes StackType_t units.
    // ESP-IDF 将 StackType_t 定义为 uint8_t，因此同一换算自然保留字节语义。
    // ESP-IDF defines StackType_t as uint8_t, so the same conversion preserves byte
    // units.
    uint32_t stack_size = static_cast<uint32_t>(stack_depth / sizeof(StackType_t));
    if ((stack_depth % sizeof(StackType_t)) != 0U)
    {
      stack_size += 1U;
    }

    auto ans = xTaskCreate(block->Port, name, stack_size, block,
                           static_cast<uint32_t>(priority), &(this->thread_handle_));
    UNUSED(ans);
    UNUSED(block);
    REQUIRE(ans == pdPASS);
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
  static uint32_t GetTime() { return static_cast<uint32_t>(Timebase::GetMilliseconds()); }

  /**
   * @brief  让线程进入休眠状态
   *         Puts the thread to sleep
   * @param  milliseconds 休眠时间（毫秒） / Sleep duration in milliseconds
   */
  static void Sleep(uint32_t milliseconds)
  {
    vTaskDelay(static_cast<TickType_t>(milliseconds));
  }

  /**
   * @brief  让线程休眠直到指定时间点
   *         Puts the thread to sleep until a specified time
   * @param  last_wakeup_time 上次唤醒时间 / Last wake-up time
   * @param  time_to_sleep 休眠时长（毫秒） / Sleep duration in milliseconds
   */
  static void SleepUntil(MillisecondTimestamp& last_wakeup_time, uint32_t time_to_sleep)
  {
    ASSERT(time_to_sleep > 0U);

    uint32_t current_tick = static_cast<uint32_t>(xTaskGetTickCount());
    uint32_t previous_wake_time =
        static_cast<uint32_t>(last_wakeup_time) + libxr_freertos_timebase_tick_offset;

    if ((previous_wake_time - current_tick) < (UINT32_MAX / 2U))
    {
      previous_wake_time = current_tick;
    }

    TickType_t wake_time = static_cast<TickType_t>(previous_wake_time);
    vTaskDelayUntil(&wake_time, static_cast<TickType_t>(time_to_sleep));
    last_wakeup_time = MillisecondTimestamp(static_cast<uint32_t>(wake_time) -
                                            libxr_freertos_timebase_tick_offset);
  }

  /**
   * @brief  让出 CPU 以执行其他线程
   *         Yields CPU execution to allow other threads to run
   */
  static void Yield() { portYIELD(); }  // NOLINT

  /**
   * @brief  线程对象转换为 FreeRTOS 线程句柄
   *         Converts the thread object to a FreeRTOS thread handle
   * @return FreeRTOS 线程句柄 / FreeRTOS thread handle
   */
  operator libxr_thread_handle() { return thread_handle_; }

 private:
  libxr_thread_handle thread_handle_;  ///< FreeRTOS 线程句柄 / FreeRTOS thread handle
};
}  // namespace LibXR
