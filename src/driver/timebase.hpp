#pragma once

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
