/**
 * @file test_cycle_value.cpp
 * @brief CycleValue 周期归一化测试 / CycleValue normalization tests.
 *
 * 检查超出周期的初值，以及加、减和取负后的周期值。
 * Check an out-of-range initial value and cyclic results of addition, subtraction and
 * negation.
 */

#include <cmath>

#include "cycle_value.hpp"
#include "libxr.hpp"
#include "libxr_def.hpp"
#include "test.hpp"
#include "test_assert.hpp"

void test_cycle_value()
{
  using LibXR::CycleValue;
  CycleValue<> val(4 * LibXR::PI + LibXR::PI / 2);
  TEST_ASSERT(equal(static_cast<double>(val), LibXR::PI / 2));

  val += LibXR::PI;
  TEST_ASSERT(equal(static_cast<double>(val), 3 * LibXR::PI / 2));

  TEST_ASSERT(equal(static_cast<double>(CycleValue<>(val - 0.0)), 3 * LibXR::PI / 2));

  val -= LibXR::PI;
  TEST_ASSERT(equal(static_cast<double>(val), LibXR::PI / 2));

  auto neg = -val;
  TEST_ASSERT(equal(static_cast<double>(neg), LibXR::TWO_PI - LibXR::PI / 2));
  UNUSED(neg);

  // float 实例按 float 的 2π 归一化。
  // A float instance wraps with the float value of 2π.
  TEST_ASSERT(CycleValue<float>(static_cast<float>(LibXR::TWO_PI)).Value() == 0.0f);

  // float 实例：归一化和差值与 double 参考相差不超过 float 精度。
  // float instance: wrap and difference match a double reference within float precision.
  for (int k = -40; k <= 40; ++k)
  {
    const float RAW = 0.37f * static_cast<float>(k);
    const double REF_WRAP = RAW - LibXR::TWO_PI * std::floor(RAW / LibXR::TWO_PI);
    const double REF_DIFF =
        -RAW - LibXR::TWO_PI * std::floor((LibXR::PI - RAW) / LibXR::TWO_PI);

    const float WRAP = CycleValue<float>(RAW);
    const float DIFF = CycleValue<float>(0.0f) - RAW;
    TEST_ASSERT(WRAP >= 0.0f && WRAP < static_cast<float>(LibXR::TWO_PI));
    TEST_ASSERT(std::abs(WRAP - REF_WRAP) < 1e-5);
    TEST_ASSERT(DIFF >= -static_cast<float>(LibXR::PI) &&
                DIFF < static_cast<float>(LibXR::PI));
    TEST_ASSERT(std::abs(DIFF - REF_DIFF) < 1e-5);
  }
}
