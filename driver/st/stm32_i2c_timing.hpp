#pragma once

#include <cstdint>

namespace LibXR::STM32I2CTiming
{
/**
 * @brief STM32 TIMINGR 时序计算 / STM32 TIMINGR calculation
 *
 * 按 AN4235 检查数据建立、保持和 SCL 高低电平时间。使用所选 I2C 模式的
 * 上升、下降时间上限；按最短滤波延迟和零边沿延迟限制最高速率。
 * Check setup, hold and SCL timings using AN4235. Use the mode's maximum edge
 * times for data timing, and minimum filter/edge delays to bound the bus speed.
 *
 * @param clock_hz I2C 内核时钟 / I2C kernel clock
 * @param speed_hz 总线频率上限 / Maximum bus frequency
 * @param analog_filter 是否启用模拟滤波 / Whether the analog filter is enabled
 * @param digital_filter 数字滤波周期数，0–15 / Digital filter cycles, 0–15
 * @param timing 成功时写入 TIMINGR / TIMINGR written on success
 * @return 是否找到满足约束的配置 / Whether a valid configuration was found
 */
inline bool Compute(uint32_t clock_hz, uint32_t speed_hz, bool analog_filter,
                    uint32_t digital_filter, uint32_t& timing)
{
  if (clock_hz == 0 || speed_hz == 0 || speed_hz > 1000000 || digital_filter > 15)
  {
    return false;
  }

  struct Limits
  {
    uint32_t rise, fall, setup, valid, low, high;
  };
  const Limits limits = speed_hz <= 100000   ? Limits{1000, 300, 250, 3450, 4700, 4000}
                        : speed_hz <= 400000 ? Limits{300, 300, 100, 900, 1300, 600}
                                             : Limits{120, 120, 50, 450, 500, 260};

  // 时间乘以内核频率计算，避免高频时钟的纳秒取整误差。
  // Scale time by the kernel frequency to avoid rounding a clock period to ns.
  constexpr uint64_t CYCLE = 1000000000ULL;
  const auto ns = [clock_hz](uint32_t value) { return uint64_t(value) * clock_hz; };
  const auto ceil_div = [](uint64_t value, uint64_t divisor)
  { return value / divisor + (value % divisor != 0); };
  const uint64_t af_min = analog_filter ? ns(50) : 0;
  const uint64_t af_max = analog_filter ? ns(260) : 0;
  const uint64_t sync = af_min + (digital_filter + 2ULL) * CYCLE;
  const uint64_t setup_min = ns(limits.rise + limits.setup);
  const int64_t hold_lower = int64_t(ns(limits.fall)) - int64_t(af_min) -
                             int64_t((digital_filter + 3ULL) * CYCLE);
  const int64_t hold_upper = int64_t(ns(limits.valid)) - int64_t(ns(limits.rise)) -
                             int64_t(af_max) - int64_t((digital_filter + 4ULL) * CYCLE);
  if (hold_upper < 0 || hold_lower > hold_upper)
  {
    return false;
  }
  const uint64_t hold_min = hold_lower > 0 ? uint64_t(hold_lower) : 0;
  const uint64_t target = ceil_div(CYCLE * clock_hz, speed_hz);
  uint64_t best_period = UINT64_MAX;
  uint64_t best_balance = UINT64_MAX;
  uint32_t best_timing = 0;

  for (uint32_t presc = 0; presc < 16; ++presc)
  {
    const uint64_t step = (presc + 1ULL) * CYCLE;
    const uint64_t setup_steps = ceil_div(setup_min, step);
    const uint64_t hold_first = ceil_div(hold_min, step);
    const uint64_t hold_last =
        uint64_t(hold_upper) / step < 15 ? uint64_t(hold_upper) / step : 15;
    if (setup_steps == 0 || setup_steps > 16 || hold_first > hold_last)
    {
      continue;
    }
    const uint32_t sdadel = static_cast<uint32_t>((hold_first + hold_last) / 2);
    for (uint32_t scll = 0; scll < 256; ++scll)
    {
      const uint64_t low_raw = (scll + 1ULL) * step;
      const uint64_t low = low_raw + sync;
      // AN4235: tI2CCLK < (tLOW - tAF - tDNF) / 4.
      if (low < ns(limits.low) || low_raw <= 2 * CYCLE)
      {
        continue;
      }
      uint64_t high_min = ns(limits.high);
      if (high_min <= CYCLE)
      {
        high_min = CYCLE + 1;
      }
      if (target > low && target - low > high_min)
      {
        high_min = target - low;
      }
      const uint64_t high_steps = high_min > sync ? ceil_div(high_min - sync, step) : 1;
      if (high_steps > 256)
      {
        continue;
      }
      const uint64_t high = high_steps * step + sync;
      const uint64_t period = low + high;
      const uint64_t balance = low > high ? low - high : high - low;
      if (period < best_period || (period == best_period && balance < best_balance))
      {
        best_period = period;
        best_balance = balance;
        best_timing = (presc << 28) | (uint32_t(setup_steps - 1) << 20) | (sdadel << 16) |
                      (uint32_t(high_steps - 1) << 8) | scll;
      }
    }
  }
  if (best_period == UINT64_MAX || best_period - target > target / 5)
  {
    return false;
  }
  timing = best_timing;
  return true;
}
}  // namespace LibXR::STM32I2CTiming
