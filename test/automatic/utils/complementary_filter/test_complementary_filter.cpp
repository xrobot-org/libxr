/**
 * @file test_complementary_filter.cpp
 * @brief 互补滤波器的姿态收敛、零偏估计、失重抑制与边界语义测试。
 *        Attitude convergence, bias estimation, gravity gating and boundary tests
 *        for ComplementaryFilter.
 */

#include <Eigen/Dense>
#include <cmath>
#include <limits>

#include "complementary_filter.hpp"
#include "test.hpp"
#include "test_assert.hpp"

void test_complementary_filter()
{
  using Filter = LibXR::ComplementaryFilter<double>;
  using Vec3 = Eigen::Matrix<double, 3, 1>;

  // 1. 静止倾斜姿态下，加速度计重力方向将横滚/俯仰拉向真值。
  // Check: under static tilt, accelerometer gravity pulls roll/pitch toward truth.
  {
    Filter filter(Filter::Param{.gain = 2.0, .bias_gain = 0.0});
    const double target_roll = 0.25;
    const double target_pitch = -0.15;
    const Eigen::Quaterniond q_true = Eigen::AngleAxisd(target_pitch, Vec3::UnitY()) *
                                      Eigen::AngleAxisd(target_roll, Vec3::UnitX());
    const Vec3 accel = q_true.conjugate() * Vec3(0.0, 0.0, 1.0);

    for (int i = 0; i < 600; ++i)
    {
      filter.Update(Vec3::Zero(), accel, 0.01);
    }

    const Vec3 euler = filter.GetEuler();
    TEST_ASSERT(std::abs(euler(0) - target_roll) < 1e-3);
    TEST_ASSERT(std::abs(euler(1) - target_pitch) < 1e-3);
  }

  // 2. 恒定陀螺零偏下，零偏估计收敛，姿态保持水平。
  // Check: constant gyro bias is estimated while roll/pitch stay near level.
  {
    Filter filter(Filter::Param{.gain = 1.5, .bias_gain = 0.5});
    const Vec3 true_bias(0.04, -0.03, 0.0);
    const Vec3 accel(0.0, 0.0, 1.0);

    for (int i = 0; i < 3000; ++i)
    {
      filter.Update(true_bias, accel, 0.01);
    }

    const Vec3 bias = filter.GetBias();
    const Vec3 euler = filter.GetEuler();
    TEST_ASSERT(std::abs(bias(0) - true_bias(0)) < 2e-3);
    TEST_ASSERT(std::abs(bias(1) - true_bias(1)) < 2e-3);
    TEST_ASSERT(std::abs(euler(0)) < 5e-3);
    TEST_ASSERT(std::abs(euler(1)) < 5e-3);
  }

  // 3. 失重或超重时屏蔽加速度计修正，仅按陀螺积分。
  // Check: out-of-range |accel| (free fall / high-g) disables accel correction.
  {
    Filter filter(Filter::Param{.gain = 5.0, .bias_gain = 0.5});
    const Vec3 low_g(0.0, 0.2, 0.0);
    const Vec3 high_g(0.0, 3.0, 0.0);

    filter.Update(Vec3::Zero(), low_g, 0.05);
    filter.Update(Vec3::Zero(), high_g, 0.05);
    TEST_ASSERT(filter.GetQuaternion().angularDistance(Eigen::Quaterniond::Identity()) <
                1e-12);
    TEST_ASSERT(filter.GetBias().norm() < 1e-12);
  }

  // 4. dt <= 0 或非有限值时保持状态不变，Reset() 清零姿态与零偏。
  // Check: non-positive / non-finite dt keeps state; Reset() clears attitude and bias.
  {
    Filter filter;
    filter.Update(Vec3(0.2, 0.0, 0.0), Vec3(0.0, 0.0, 1.0), 0.05);
    const Eigen::Quaterniond before_q = filter.GetQuaternion();
    const Vec3 before_b = filter.GetBias();

    filter.Update(Vec3(1.0, 1.0, 1.0), Vec3(0.0, 1.0, 0.0), 0.0);
    filter.Update(Vec3(1.0, 1.0, 1.0), Vec3(0.0, 1.0, 0.0), -0.01);
    filter.Update(Vec3(1.0, 1.0, 1.0), Vec3(0.0, 1.0, 0.0),
                  std::numeric_limits<double>::infinity());
    filter.Update(Vec3(1.0, 1.0, 1.0), Vec3(0.0, 1.0, 0.0),
                  std::numeric_limits<double>::quiet_NaN());

    TEST_ASSERT(filter.GetQuaternion().angularDistance(before_q) < 1e-12);
    TEST_ASSERT((filter.GetBias() - before_b).norm() < 1e-12);

    filter.Reset();
    TEST_ASSERT(filter.GetQuaternion().angularDistance(Eigen::Quaterniond::Identity()) <
                1e-12);
    TEST_ASSERT(filter.GetBias().norm() < 1e-12);
  }
}
