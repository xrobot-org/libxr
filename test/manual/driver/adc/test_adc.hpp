/**
 * @file test_adc.hpp
 * @brief 已知输入电压下的 ADC 连续读数测试 / ADC sampling test at a known input voltage.
 *
 * 每次读数都必须有效且处于允许误差内，返回最小值、最大值和平均值。
 * 调用方提供稳定电压，测试不控制信号源。
 * Require every reading to be finite and within tolerance; return min, max and mean.
 * The caller supplies a stable voltage.
 */
#pragma once

#include <cmath>

#include "adc.hpp"
#include "test_assert.hpp"
#include "thread.hpp"

namespace LibXR::Test
{
/** @brief ADC 采样统计 / ADC sample statistics. */
struct ADCTestResult
{
  float minimum;  ///< 最小电压，V / Minimum voltage in V.
  float maximum;  ///< 最大电压，V / Maximum voltage in V.
  float average;  ///< 平均电压，V / Mean voltage in V.
};

/**
 * @brief 检查已知电压下的连续读数 / Check consecutive readings at a known voltage.
 * @pre ADC 和系统已初始化，输入与采样结果已稳定；从普通任务调用。
 *      Initialize the ADC and system, allow the input and sampled data to settle,
 *      and call from normal task context.
 * @param adc 已初始化的 ADC 通道 / Initialized ADC channel.
 * @param expected_voltage 预期电压，V / Expected voltage in V.
 * @param tolerance_voltage 每次读数的绝对误差上限，V / Absolute tolerance per reading in
 * V.
 * @param interval_ms 两次读取之间的等待，0 表示连续读取 / Delay between reads; 0 reads
 * back-to-back.
 * @param samples 读取次数 / Number of readings.
 * @return 本次读数的统计结果 / Statistics for these readings.
 */
inline ADCTestResult TestADC(ADC& adc, float expected_voltage, float tolerance_voltage,
                             uint32_t interval_ms = 1, uint32_t samples = 1000)
{
  TEST_ASSERT(std::isfinite(expected_voltage));
  TEST_ASSERT(std::isfinite(tolerance_voltage) && tolerance_voltage >= 0.0f);
  TEST_ASSERT(samples > 0 && interval_ms < UINT32_MAX / 2U);
  ADCTestResult result{};
  double sum = 0.0;
  for (uint32_t i = 0; i < samples; ++i)
  {
    const float voltage = adc.Read();
    TEST_ASSERT(std::isfinite(voltage));
    // 先检查每个读数，平均值不能掩盖偶发的错误采样。
    // Check each reading first; a mean must not hide occasional bad samples.
    TEST_ASSERT(std::fabs(static_cast<double>(voltage) - expected_voltage) <=
                tolerance_voltage);
    if (i == 0 || voltage < result.minimum)
    {
      result.minimum = voltage;
    }
    if (i == 0 || voltage > result.maximum)
    {
      result.maximum = voltage;
    }
    sum += voltage;
    if (interval_ms != 0 && i + 1U < samples)
    {
      Thread::Sleep(interval_ms);
    }
  }
  result.average = static_cast<float>(sum / samples);
  return result;
}
}  // namespace LibXR::Test
