/**
 * @file test_ekf.cpp
 * @brief EKF 预测、更新与新息统计测试 / EKF prediction, update, and innovation statistics
 * tests.
 *
 * 测试项目 / Test items:
 * 1. 线性匀速模型的位置与速度估计。 Linear constant-velocity state estimation.
 * 2. 预测步协方差等于 F P Fᵀ + Q。 Prediction covariance equals F P Fᵀ + Q.
 * 3. 角度残差归一化让更新走最短路径。 Wrapped angle residuals take the short path.
 * 4. 自定义状态增量叠加后归一化 yaw。 Custom state addition wraps yaw afterwards.
 * 5. 非线性观测下的一致更新不触发门限，离群更新触发门限。
 *    A consistent nonlinear update stays inside the gate, an outlier trips it.
 * 6. NIS 失败滑窗不超过窗口长度。 The NIS failure window never exceeds its length.
 * 7. Joseph 形式更新保持协方差对称。 The Joseph-form update keeps P symmetric.
 *
 * 测试原理 / Test principles:
 * 1. 所有场景使用可手推的期望值，不依赖随机数据。 Every scenario uses hand-derivable
 * expected values so no random data is needed.
 * 2. 角度场景选取跨越 ±π 的观测，直接暴露未归一化残差会产生 2π 级跳变。
 *    Angle scenarios use observations across ±π, where an unwrapped residual would jump
 * by 2π.
 */

#include "ekf.hpp"
#include "libxr.hpp"
#include "libxr_def.hpp"
#include "test.hpp"
#include "test_assert.hpp"

namespace
{
constexpr double EKF_TEST_PI = 3.14159265358979323846;

// 角度归一化放在文件级函数里：EKF 钩子必须捕获列表为空，不能捕获局部 lambda。
// The angle normalization lives at file scope: an EKF hook must capture nothing, so it
// cannot capture a local lambda.
double WrapAngle(double angle)
{
  while (angle > EKF_TEST_PI)
  {
    angle -= 2.0 * EKF_TEST_PI;
  }
  while (angle <= -EKF_TEST_PI)
  {
    angle += 2.0 * EKF_TEST_PI;
  }
  return angle;
}
}  // namespace

void test_ekf()
{
  using Ekf = LibXR::EKF<double>;
  using EkfState = Ekf::StateVector;
  using EkfStateCov = Ekf::StateCovariance;
  using EkfMeasurement = Ekf::MeasurementVector;
  using EkfMeasurementCov = Ekf::MeasurementCovariance;
  using EkfObservation = Ekf::ObservationMatrix;

  // 检查内容：线性匀速模型在位置观测下收敛到真值，速度被观测序列估计出来。
  // Check: a linear constant-velocity model converges to the true position and estimates
  // velocity from the observation sequence.
  {
    EkfState x0(2);
    x0 << 0.0, 0.0;
    EkfStateCov p0 = EkfStateCov::Identity(2, 2);
    Ekf ekf(x0, p0);

    const double dt = 0.1;
    EkfStateCov f = EkfStateCov::Identity(2, 2);
    f(0, 1) = dt;

    // 分段白噪声加速度过程噪声，连续匀速模型位置通道的常用形式。
    // Piecewise-white-noise acceleration process noise, the common form for the position
    // channel of a constant-velocity model.
    const double accel = 1.0;
    EkfStateCov q = EkfStateCov::Zero(2, 2);
    q(0, 0) = dt * dt * dt * dt / 4.0 * accel;
    q(0, 1) = dt * dt * dt / 2.0 * accel;
    q(1, 0) = q(0, 1);
    q(1, 1) = dt * dt * accel;
    TEST_ASSERT(q(1, 1) > 0.0);

    EkfObservation h(1, 2);
    h << 1.0, 0.0;
    EkfMeasurementCov r(1, 1);
    r << 0.01;

    const double velocity = 0.5;
    for (int i = 1; i <= 40; ++i)
    {
      ekf.Predict(f, q);
      EkfMeasurement z(1);
      z << velocity * i * dt;
      ekf.Update(z, h, r);
    }

    TEST_ASSERT(std::abs(ekf.x(0) - velocity * 40 * dt) < 1e-3);
    TEST_ASSERT(std::abs(ekf.x(1) - velocity) < 1e-3);
    TEST_ASSERT(ekf.UpdateCount() == 40);
  }

  // 检查内容：只预测不更新时，协方差按 F P Fᵀ + Q 增长。
  // Check: prediction without an update grows the covariance by F P Fᵀ + Q.
  {
    EkfState x0(1);
    x0 << 1.0;
    EkfStateCov p0(1, 1);
    p0 << 0.5;
    Ekf ekf(x0, p0);

    EkfStateCov f(1, 1);
    f << 2.0;
    EkfStateCov q(1, 1);
    q << 0.25;

    ekf.Predict(f, q);
    TEST_ASSERT(equal(ekf.P(0, 0), 2.0 * 0.5 * 2.0 + 0.25));
    TEST_ASSERT(equal(ekf.x(0), 2.0));
    TEST_ASSERT(ekf.UpdateCount() == 0);
  }

  // 检查内容：观测跨越 ±π 时，归一化残差把状态推向最近邻而不是绕远 2π。
  // Check: with an observation across ±π, the wrapped residual moves the state to the
  // nearest value instead of the 2π detour.
  {
    EkfState x0(1);
    x0 << -3.10;
    EkfStateCov p0(1, 1);
    p0 << 0.1;
    Ekf ekf(x0, p0);

    EkfStateCov f = EkfStateCov::Identity(1, 1);
    EkfStateCov q = EkfStateCov::Zero(1, 1);
    ekf.Predict(f, q);

    EkfObservation h = EkfObservation::Identity(1, 1);
    EkfMeasurementCov r(1, 1);
    r << 0.01;

    EkfMeasurement z(1);
    z << 3.10;
    auto residual = [](const EkfMeasurement& a, const EkfMeasurement& b)
    {
      EkfMeasurement c = a - b;
      c(0) = std::remainder(c(0), 2.0 * EKF_TEST_PI);
      return c;
    };
    ekf.Update(z, h, r, [](const EkfState& state) { return state; }, residual);

    TEST_ASSERT(std::abs(WrapAngle(ekf.x(0) - (-3.10))) < 0.2);
    TEST_ASSERT(std::abs(WrapAngle(ekf.x(0) - 3.10)) < 0.2);
  }

  // 检查内容：自定义状态增量叠加把越界 yaw 归一化回 [-π, π)。
  // Check: the custom state addition wraps an out-of-range yaw back into [-π, π).
  {
    EkfState x0(1);
    x0 << 3.10;
    EkfStateCov p0(1, 1);
    p0 << 1.0;
    auto x_add = [](const EkfState& a, const EkfState& b)
    {
      EkfState c = a + b;
      c(0) = WrapAngle(c(0));
      return c;
    };
    Ekf ekf(x0, p0, x_add);

    EkfObservation h = EkfObservation::Identity(1, 1);
    EkfMeasurementCov r(1, 1);
    r << 0.5;

    EkfMeasurement z(1);
    z << 3.20;
    ekf.Update(z, h, r);

    TEST_ASSERT(std::abs(ekf.x(0) - WrapAngle(3.10 + (3.20 - 3.10) * (1.0 / 1.5))) <
                1e-9);
    TEST_ASSERT(ekf.x(0) < 0.0);
  }

  // 检查内容：一致的非线性更新不触发新息门限，离群更新触发门限。
  // Check: a consistent nonlinear update stays inside the innovation gate, an outlier
  // trips it.
  {
    EkfState x0(1);
    x0 << 1.0;
    EkfStateCov p0(1, 1);
    p0 << 0.01;
    Ekf ekf(x0, p0);

    EkfObservation h(1, 1);
    h << 2.0;
    EkfMeasurementCov r(1, 1);
    r << 0.01;
    auto observe = [](const EkfState& state)
    {
      EkfMeasurement z(1);
      z << state(0) * state(0);
      return z;
    };

    EkfMeasurement z(1);
    z << 1.0404;
    ekf.Update(z, h, r, observe);
    TEST_ASSERT(ekf.NisFailCount() == 0);
    TEST_ASSERT(ekf.NeesFailCount() == 0);
    TEST_ASSERT(ekf.NisFailureCount() == 0);
    TEST_ASSERT(ekf.UpdateCount() == 1);

    z << 3.0;
    ekf.Update(z, h, r, observe);
    TEST_ASSERT(ekf.last_nis > ekf.nis_threshold);
    TEST_ASSERT(ekf.NisFailCount() == 1);
    TEST_ASSERT(ekf.UpdateCount() == 2);
  }

  // 检查内容：滑窗在更新前设为 2 时，失败记录数不超过窗口长度。
  // Check: with the window set to 2 before the updates, the recorded failures never
  // exceed the window length.
  {
    EkfState x0(1);
    x0 << 1.0;
    EkfStateCov p0(1, 1);
    p0 << 0.01;
    Ekf ekf(x0, p0);
    ekf.SetWindowSize(2);

    EkfObservation h = EkfObservation::Identity(1, 1);
    EkfMeasurementCov r(1, 1);
    r << 0.01;

    EkfMeasurement z(1);
    for (int i = 0; i < 3; ++i)
    {
      z << 3.0 + 2.0 * i;
      ekf.Update(z, h, r);
      TEST_ASSERT(ekf.last_nis > ekf.nis_threshold);
      TEST_ASSERT(ekf.NisFailureCount() <= ekf.WindowSize());
    }
    TEST_ASSERT(ekf.NisFailureCount() == 2);
    TEST_ASSERT(ekf.WindowSize() == 2);
    TEST_ASSERT(ekf.NisFailCount() == 3);
    TEST_ASSERT(ekf.UpdateCount() == 3);

    ekf.SetWindowSize(ekf.WINDOW_CAPACITY + 10U);
    TEST_ASSERT(ekf.WindowSize() == ekf.WINDOW_CAPACITY);
  }

  // 检查内容：多次更新后协方差保持对称，即 Joseph 形式生效。
  // Check: the covariance stays symmetric across updates, which the Joseph form
  // guarantees.
  {
    EkfState x0(2);
    x0 << 0.0, 1.0;
    EkfStateCov p0(2, 2);
    p0 << 1.0, 0.3, 0.3, 1.0;
    Ekf ekf(x0, p0);

    EkfStateCov f = EkfStateCov::Identity(2, 2);
    EkfStateCov q = EkfStateCov::Zero(2, 2);
    EkfObservation h = EkfObservation::Identity(2, 2);
    EkfMeasurementCov r = EkfMeasurementCov::Identity(2, 2) * 0.5;

    for (int i = 0; i < 5; ++i)
    {
      ekf.Predict(f, q);
      EkfMeasurement z(2);
      z << 0.1 * i, 1.0 + 0.05 * i;
      ekf.Update(z, h, r);
      TEST_ASSERT((ekf.P - ekf.P.transpose()).cwiseAbs().maxCoeff() < 1e-12);
      TEST_ASSERT(ekf.P.diagonal().minCoeff() > 0.0);
    }
  }
}
