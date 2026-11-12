#pragma once

#include <atomic>
#include <cstdint>

#include "libxr_def.hpp"
#include "libxr_time.hpp"

namespace LibXR
{

/**
 * @brief 时间基类，用于提供高精度时间戳。
 *        Timebase class for providing high-precision timestamps.
 *
 * 该类提供了微秒和毫秒级的时间戳获取接口。
 * This class provides interfaces for obtaining timestamps in microseconds and
 * milliseconds.
 */
class Timebase
{
 public:
  /**
   * @brief 默认构造函数。
   *        Default constructor.
   */
  Timebase() = default;

  /**
   * @brief 禁止拷贝构造。
   *        Copy construction is disabled.
   */
  Timebase(const Timebase&) = delete;

  /**
   * @brief 禁止拷贝赋值。
   *        Copy assignment is disabled.
   */
  Timebase& operator=(const Timebase&) = delete;

  /**
   * @brief 获取当前时间的微秒级时间戳。
   *        Gets the current timestamp in microseconds.
   *
   * @return 返回当前的时间戳（单位：微秒）。
   *         Returns the current timestamp (in microseconds).
   */
  static MicrosecondTimestamp GetMicroseconds();

  /**
   * @brief 获取当前时间的毫秒级时间戳。
   *        Gets the current timestamp in milliseconds.
   *
   * @return 返回当前的时间戳（单位：毫秒）。
   *         Returns the current timestamp (in milliseconds).
   */
  static MillisecondTimestamp GetMilliseconds();

  /**
   * @brief 检查时间基是否已经初始化。
   *        Check whether the active timebase backend is initialized.
   */
  [[nodiscard]] static bool IsReady() noexcept { return ready_; }

  /**
   * @brief 微秒级延时 / Delay in microseconds
   * @param us 延时长度（us）/ Delay length (us)
   */
  static inline void DelayMicroseconds(uint32_t us)
  {
    if (us == 0u)
    {
      return;
    }

    const uint64_t START = static_cast<uint64_t>(Timebase::GetMicroseconds());
    while ((static_cast<uint64_t>(Timebase::GetMicroseconds()) - START) < us)
    {
      // busy-wait
    }
  }

  /**
   * @brief 忙等 loops 次循环，用于比 1 us 更短的延时
   *        Busy-wait for loops iterations, for delays shorter than 1 us
   *
   * 每次循环的耗时由平台和编译结果决定，使用者在自己的平台上标定。平台的时基类可以
   * 定义同名的静态函数替换它；使用者把时基类作为类型参数、调用
   * `TimebaseType::BusyLoop()` 时才会用到替换后的版本。
   * The time per iteration depends on the platform and the generated code; users
   * calibrate it on their platform. The timebase class of a platform may define a static
   * function of the same name to replace it; a user reaches the replacement by taking
   * the timebase class as a type parameter and calling `TimebaseType::BusyLoop()`.
   *
   * @param loops 循环次数，必须 >= 1 / Iteration count, must be >= 1.
   */
  LIBXR_ALWAYS_INLINE static void BusyLoop(uint32_t loops)
  {
    DEV_ASSERT(loops >= 1u);
    // 计数器放在寄存器里；atomic_signal_fence 只约束编译器、不产生指令，使空循环不会被
    // 优化掉。volatile 计数器会在每次迭代读写栈并带内存屏障，每次迭代更慢。
    // The counter lives in a register; atomic_signal_fence constrains only the compiler
    // and emits no instruction, so the empty loop is not optimised away. A volatile
    // counter would read and write the stack with a memory barrier on every iteration,
    // which makes each iteration slower.
    while (loops--)
    {
      std::atomic_signal_fence(std::memory_order_seq_cst);
    }
  }

 protected:
  /**
   * @brief 设置时间基就绪状态。
   *        Set the timebase ready flag.
   * @param ready 是否就绪 / Whether the backend is ready.
   */
  static void SetReady(bool ready = true) noexcept { ready_ = ready; }

  /**
   * @brief 配置时间戳回绕上界。
   *        Configure the timestamp wraparound limits.
   * @param max_valid_us 微秒时间戳的有效上界。
   *                     Maximum valid microsecond timestamp value.
   * @param max_valid_ms 毫秒时间戳的有效上界。
   *                     Maximum valid millisecond timestamp value.
   */
  static void ConfigureWrapRange(uint64_t max_valid_us, uint32_t max_valid_ms) noexcept
  {
    Detail::ConfigureTimebaseWrapRange(max_valid_us, max_valid_ms);
  }

  /**
   * @brief 读取当前配置的微秒回绕上界。
   *        Read the configured microsecond wraparound limit.
   */
  [[nodiscard]] static uint64_t GetConfiguredWrapRangeUs() noexcept
  {
    return Detail::TimebaseMaxValidUs();
  }

  /**
   * @brief 读取当前配置的毫秒回绕上界。
   *        Read the configured millisecond wraparound limit.
   */
  [[nodiscard]] static uint32_t GetConfiguredWrapRangeMs() noexcept
  {
    return Detail::TimebaseMaxValidMs();
  }

 private:
  /**
   * @brief 时间基是否已完成初始化。
   *        Whether the timebase backend has been initialized.
   */
  static inline bool ready_ = false;
};

}  // namespace LibXR
