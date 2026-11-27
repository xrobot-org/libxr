/**
 * @file test_lpfilter.cpp
 * @brief 一阶低通滤波器的阶跃响应、系数换算与状态语义测试。
 *        Step response, coefficient conversion and state semantics of the first-order
 *        low-pass filter.
 *
 * 测试项目 / Test items:
 * 1. 首样本吸收与 alpha = dt / (tau + dt) 递推。First-sample absorption and the
 *    alpha = dt / (tau + dt) recursion.
 * 2. dt <= 0 时保持状态。The state is kept when dt <= 0.
 * 3. Reset 之后重新吸收首样本。Reset makes the next sample the state again.
 * 4. 截止频率与固定系数换算为时间常数。A cutoff frequency and a fixed coefficient
 *    convert into a time constant.
 * 5. Eigen 向量的逐轴滤波与标量类型推导。Per-axis filtering of an Eigen vector and the
 *    deduced coefficient type.
 */

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <limits>
#include <type_traits>

#include "libxr_def.hpp"
#include "lpfilter.hpp"
#include "test.hpp"
#include "test_assert.hpp"

void test_lpfilter()
{
  // 检查内容：第一个样本直接成为状态，之后按 alpha 递推。
  // Check: the first sample becomes the state, then the recursion applies alpha.
  {
    LibXR::LPFilter<>::Param param;
    param.time_constant = 0.1f;
    LibXR::LPFilter<> filter(param);

    filter.Update(0.0f, 0.01f);
    TEST_ASSERT(equal(static_cast<double>(filter.State()), 0.0));

    const double alpha = 0.01 / 0.11;
    double expected = 0.0;
    for (int i = 0; i < 10; ++i)
    {
      filter.Update(1.0f, 0.01f);
      expected += alpha * (1.0 - expected);
    }
    TEST_ASSERT(equal(static_cast<double>(filter.State()), expected));

    // 检查内容：dt 不是正数或非有限值时保持状态。
    // Check: a non-positive or non-finite dt keeps the state.
    const float before = filter.State();
    filter.Update(10.0f, 0.0f);
    TEST_ASSERT(equal(static_cast<double>(filter.State()), static_cast<double>(before)));
    filter.Update(10.0f, -0.01f);
    TEST_ASSERT(equal(static_cast<double>(filter.State()), static_cast<double>(before)));
    filter.Update(10.0f, std::numeric_limits<float>::infinity());
    TEST_ASSERT(equal(static_cast<double>(filter.State()), static_cast<double>(before)));
    filter.Update(10.0f, std::numeric_limits<float>::quiet_NaN());
    TEST_ASSERT(equal(static_cast<double>(filter.State()), static_cast<double>(before)));

    // 检查内容：Reset 之后下一次更新重新吸收样本。
    // Check: after Reset the next update absorbs its sample again.
    filter.Reset();
    TEST_ASSERT(!filter.IsInitialized());
    filter.Update(5.0f, 0.01f);
    TEST_ASSERT(filter.IsInitialized());
    TEST_ASSERT(equal(static_cast<double>(filter.State()), 5.0));
  }

  // 检查内容：时间常数、截止频率与固定系数三者互相换算。
  // Check: the time constant, a cutoff frequency and a fixed coefficient convert into
  // each other.
  {
    LibXR::LPFilter<> filter(LibXR::LPFilter<>::Param{.time_constant = 1.0f});

    filter.SetAlpha(0.2f, 0.001f);
    TEST_ASSERT(std::abs(filter.GetParam().time_constant - 0.004f) < 1e-6f);
    TEST_ASSERT(std::abs(filter.GetAlpha(0.001f) - 0.2f) < 1e-6f);
    TEST_ASSERT(std::abs(filter.GetAlpha(0.002f) - 1.0f / 3.0f) < 1e-6f);

    filter.SetCutoffFrequency(40.0f);
    // 独立手推期望值：tau = 1 / (2 * pi * 40) = 1 / (80 * pi) ≈ 0.0039788736 s。
    // Independent hand-computed expectation: tau = 1 / (2 * pi * 40).
    constexpr double EXPECTED_TAU_40HZ = 0.003978873577297384;
    TEST_ASSERT(std::abs(static_cast<double>(filter.GetParam().time_constant) -
                         EXPECTED_TAU_40HZ) < 1e-7);

    // 检查内容：40 Hz 正弦波在 4000 Hz 采样下的稳态幅值应接近 -3 dB (1 / sqrt(2))。
    // Check: a 40 Hz sine sampled at 4000 Hz attenuates to ~1/sqrt(2) (-3 dB).
    filter.Reset();
    const float dt = 1.0f / 4000.0f;
    float steady_peak = 0.0f;
    for (int i = 0; i < 400; ++i)
    {
      const float x = std::sin(2.0f * 3.14159265358979323846f * 40.0f * i * dt);
      const float y = filter.Update(x, dt);
      if (i >= 300)
      {
        steady_peak = std::max(steady_peak, std::abs(y));
      }
    }
    TEST_ASSERT(std::abs(static_cast<double>(steady_peak) - 0.70710678) < 0.03);

    filter.SetParam(LibXR::LPFilter<>::Param{.time_constant = 0.5f});
    TEST_ASSERT(std::abs(filter.GetParam().time_constant - 0.5f) < 1e-6f);
  }

  // 检查内容：SetParam 保留已有状态。
  // Check: SetParam keeps the current state.
  {
    LibXR::LPFilter<> filter(LibXR::LPFilter<>::Param{.time_constant = 0.1f});
    filter.Update(1.0f, 0.1f);
    filter.SetParam(LibXR::LPFilter<>::Param{.time_constant = 0.5f});
    TEST_ASSERT(equal(static_cast<double>(filter.State()), 1.0));
    TEST_ASSERT(std::abs(filter.GetParam().time_constant - 0.5f) < 1e-6f);
  }

  // 检查内容：Eigen 向量逐轴独立滤波，系数类型由 T 推导。
  // Check: an Eigen vector filters per axis and the coefficient type comes from T.
  {
    using Vector = Eigen::Matrix<float, 3, 1>;
    using Filter = LibXR::LPFilter<Vector>;
    static_assert(
        std::is_same_v<decltype(std::declval<Filter>().GetAlpha(0.01f)), float>);

    Filter filter(Filter::Param{.time_constant = 0.1f});
    Vector zero = Vector::Zero();
    Vector one;
    one << 1.0f, 2.0f, 3.0f;

    filter.Update(zero, 0.01f);
    filter.Update(one, 0.01f);
    const float alpha = 0.01f / 0.11f;
    TEST_ASSERT((filter.State() - alpha * one).norm() < 1e-6f);
  }
}
