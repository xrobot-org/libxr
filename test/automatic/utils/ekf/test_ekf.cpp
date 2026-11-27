/**
 * @file test_ekf.cpp
 * @brief EKF 预测、更新、零堆分配与新息统计测试 / EKF prediction, update,
 * zero-allocation, and innovation statistics tests.
 *
 * 测试项目 / Test items:
 * 1. 线性匀速模型的位置与速度估计。 Linear constant-velocity state estimation.
 * 2. 预测步协方差等于 F P Fᵀ + Q。 Prediction covariance equals F P Fᵀ + Q.
 * 3. 角度残差归一化让更新走最短路径。 Wrapped angle residuals take the short path.
 * 4. 自定义状态增量叠加后归一化 yaw。 Custom state addition wraps yaw afterwards.
 * 5. 运行期替换状态增量叠加函数。 Runtime replacement of the state addition hook.
 * 6. 非线性观测下的一致更新不触发门限，离群更新触发门限。
 *    A consistent nonlinear update stays inside the gate, an outlier trips it.
 * 7. 先验 NIS 与显式真值 NEES 数值验证。 Analytical prior NIS and truth-based NEES.
 * 8. Predict / Update 零堆分配验证（EIGEN_RUNTIME_NO_MALLOC）。
 *    Zero heap allocation during Predict and Update (EIGEN_RUNTIME_NO_MALLOC).
 * 9. NIS 失败滑窗不超过窗口长度。 The NIS failure window never exceeds its length.
 * 10. Joseph 形式更新保持协方差对称。 The Joseph-form update keeps P symmetric.
 */

#ifndef EIGEN_RUNTIME_NO_MALLOC
#define EIGEN_RUNTIME_NO_MALLOC
#endif

#include <cmath>
#include <type_traits>

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
  // 编译期检查：禁止危险的默认构造。
  // Compile-time check: dangerous default construction is deleted.
  static_assert(!std::is_default_constructible_v<LibXR::EKF<2, 1, double>>);

  // 检查内容：线性匀速模型在位置观测下收敛到真值，速度被观测序列估计出来。
  // Check: a linear constant-velocity model converges to the true position and estimates
  // velocity from the observation sequence.
  {
    using Ekf = LibXR::EKF<2, 1, double>;
    Ekf::StateVector x0 = Ekf::StateVector::Zero();
    Ekf::StateCovariance p0 = Ekf::StateCovariance::Identity();
    Ekf ekf(x0, p0);

    const double dt = 0.1;
    Ekf::StateCovariance f = Ekf::StateCovariance::Identity();
    f(0, 1) = dt;

    // 分段白噪声加速度过程噪声，连续匀速模型位置通道的常用形式。
    // Piecewise-white-noise acceleration process noise, the common form for the position
    // channel of a constant-velocity model.
    const double accel = 1.0;
    Ekf::StateCovariance q = Ekf::StateCovariance::Zero();
    q(0, 0) = dt * dt * dt * dt / 4.0 * accel;
    q(0, 1) = dt * dt * dt / 2.0 * accel;
    q(1, 0) = q(0, 1);
    q(1, 1) = dt * dt * accel;
    TEST_ASSERT(q(1, 1) > 0.0);

    Ekf::ObservationMatrix h;
    h << 1.0, 0.0;
    Ekf::MeasurementCovariance r;
    r << 0.01;

    const double velocity = 0.5;
    Eigen::internal::set_is_malloc_allowed(false);
    for (int i = 1; i <= 40; ++i)
    {
      ekf.Predict(f, q);
      Ekf::MeasurementVector z;
      z << velocity * i * dt;
      ekf.Update(z, h, r);
    }
    Eigen::internal::set_is_malloc_allowed(true);

    TEST_ASSERT(std::abs(ekf.x(0) - velocity * 40 * dt) < 1e-3);
    TEST_ASSERT(std::abs(ekf.x(1) - velocity) < 1e-3);
    TEST_ASSERT(ekf.UpdateCount() == 40);
  }

  // 检查内容：只预测不更新时，协方差按 F P Fᵀ + Q 增长。
  // Check: prediction without an update grows the covariance by F P Fᵀ + Q.
  {
    using Ekf = LibXR::EKF<1, 1, double>;
    Ekf::StateVector x0;
    x0 << 1.0;
    Ekf::StateCovariance p0;
    p0 << 0.5;
    Ekf ekf(x0, p0);

    Ekf::StateCovariance f;
    f << 2.0;
    Ekf::StateCovariance q;
    q << 0.25;

    ekf.Predict(f, q);
    TEST_ASSERT(equal(ekf.P(0, 0), 2.0 * 0.5 * 2.0 + 0.25));
    TEST_ASSERT(equal(ekf.x(0), 2.0));
    TEST_ASSERT(ekf.UpdateCount() == 0);
  }

  // 检查内容：先验 NIS 与显式真值 NEES 的解析手算验证。
  // Check: analytical prior NIS and truth-based ComputeNEES verification.
  // 设 P^- = 1, R = 1, H = 1, x^- = 0, z = 1：
  // 先验新息 y = 1, S = 2 => NIS = y^2 / S = 0.5 (而非后验 0.1667)。
  // 更新后 K = 0.5, x^+ = 0.5, P^+ = 0.5；若真值 x_true = 0.3，NEES = (0.2)^2 / 0.5 =
  // 0.08。
  {
    using Ekf = LibXR::EKF<1, 1, double>;
    Ekf::StateVector x0;
    x0 << 0.0;
    Ekf::StateCovariance p0;
    p0 << 1.0;
    Ekf ekf(x0, p0);

    Ekf::ObservationMatrix h;
    h << 1.0;
    Ekf::MeasurementCovariance r;
    r << 1.0;
    Ekf::MeasurementVector z;
    z << 1.0;

    Eigen::internal::set_is_malloc_allowed(false);
    ekf.Update(z, h, r);
    Eigen::internal::set_is_malloc_allowed(true);

    TEST_ASSERT(std::abs(ekf.last_innovation(0) - 1.0) < 1e-12);
    TEST_ASSERT(std::abs(ekf.last_nis - 0.5) < 1e-12);
    TEST_ASSERT(std::abs(ekf.x(0) - 0.5) < 1e-12);
    TEST_ASSERT(std::abs(ekf.P(0, 0) - 0.5) < 1e-12);

    Ekf::StateVector x_true;
    x_true << 0.3;
    TEST_ASSERT(std::abs(ekf.ComputeNEES(x_true) - 0.08) < 1e-12);
  }

  // 检查内容：观测跨越 ±π 时，归一化残差把状态推向最近邻而不是绕远 2π。
  // Check: with an observation across ±π, the wrapped residual moves the state to the
  // nearest value instead of the 2π detour.
  {
    using Ekf = LibXR::EKF<1, 1, double>;
    Ekf::StateVector x0;
    x0 << -3.10;
    Ekf::StateCovariance p0;
    p0 << 0.1;
    Ekf ekf(x0, p0);

    Ekf::StateCovariance f = Ekf::StateCovariance::Identity();
    Ekf::StateCovariance q = Ekf::StateCovariance::Zero();
    ekf.Predict(f, q);

    Ekf::ObservationMatrix h = Ekf::ObservationMatrix::Identity();
    Ekf::MeasurementCovariance r;
    r << 0.01;

    Ekf::MeasurementVector z;
    z << 3.10;
    auto residual = [](const Ekf::MeasurementVector& a, const Ekf::MeasurementVector& b)
    {
      Ekf::MeasurementVector c = a - b;
      c(0) = std::remainder(c(0), 2.0 * EKF_TEST_PI);
      return c;
    };
    ekf.Update(z, h, r, [](const Ekf::StateVector& state) { return state; }, residual);

    TEST_ASSERT(std::abs(WrapAngle(ekf.x(0) - (-3.10))) < 0.2);
    TEST_ASSERT(std::abs(WrapAngle(ekf.x(0) - 3.10)) < 0.2);
  }

  // 检查内容：自定义状态增量叠加把越界 yaw 归一化回 [-π, π)。
  // Check: the custom state addition wraps an out-of-range yaw back into [-π, π).
  {
    using Ekf = LibXR::EKF<1, 1, double>;
    Ekf::StateVector x0;
    x0 << 3.10;
    Ekf::StateCovariance p0;
    p0 << 1.0;
    auto x_add = [](const Ekf::StateVector& a, const Ekf::StateVector& b)
    {
      Ekf::StateVector c = a + b;
      c(0) = WrapAngle(c(0));
      return c;
    };
    Ekf ekf(x0, p0, x_add);

    Ekf::ObservationMatrix h = Ekf::ObservationMatrix::Identity();
    Ekf::MeasurementCovariance r;
    r << 0.5;

    Ekf::MeasurementVector z;
    z << 3.20;
    ekf.Update(z, h, r);

    TEST_ASSERT(std::abs(ekf.x(0) - WrapAngle(3.10 + (3.20 - 3.10) * (1.0 / 1.5))) <
                1e-9);
    TEST_ASSERT(ekf.x(0) < 0.0);
  }

  // 检查内容：运行期替换状态增量叠加函数后，下一次更新即按新逻辑回绕 yaw。
  // Check: after replacing the state addition hook at runtime, the next update wraps yaw
  // with the new logic.
  {
    using Ekf = LibXR::EKF<1, 1, double>;
    Ekf::StateVector x0;
    x0 << 3.10;
    Ekf::StateCovariance p0;
    p0 << 1.0;
    Ekf ekf(x0, p0);

    Ekf::ObservationMatrix h = Ekf::ObservationMatrix::Identity();
    Ekf::MeasurementCovariance r;
    r << 0.5;
    Ekf::MeasurementVector z;
    z << 3.20;

    ekf.Update(z, h, r);
    TEST_ASSERT(std::abs(ekf.x(0) - (3.10 + (3.20 - 3.10) * (1.0 / 1.5))) < 1e-9);

    ekf.SetStateAdd(
        [](const Ekf::StateVector& a, const Ekf::StateVector& b)
        {
          Ekf::StateVector c = a + b;
          c(0) = WrapAngle(c(0));
          return c;
        });
    ekf.Update(z, h, r);
    TEST_ASSERT(ekf.x(0) < 0.0);
  }

  // 检查内容：一致的非线性更新不触发新息门限，离群更新触发门限。
  // Check: a consistent nonlinear update stays inside the innovation gate, an outlier
  // trips it.
  {
    using Ekf = LibXR::EKF<1, 1, double>;
    Ekf::StateVector x0;
    x0 << 1.0;
    Ekf::StateCovariance p0;
    p0 << 0.01;
    Ekf ekf(x0, p0);

    Ekf::ObservationMatrix h;
    h << 2.0;
    Ekf::MeasurementCovariance r;
    r << 0.01;
    auto observe = [](const Ekf::StateVector& state)
    {
      Ekf::MeasurementVector z;
      z << state(0) * state(0);
      return z;
    };

    Ekf::MeasurementVector z;
    z << 1.0404;
    ekf.Update(z, h, r, observe);
    TEST_ASSERT(ekf.NisFailCount() == 0);
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
    using Ekf = LibXR::EKF<1, 1, double>;
    Ekf::StateVector x0;
    x0 << 1.0;
    Ekf::StateCovariance p0;
    p0 << 0.01;
    Ekf ekf(x0, p0);
    ekf.SetWindowSize(2);

    Ekf::ObservationMatrix h = Ekf::ObservationMatrix::Identity();
    Ekf::MeasurementCovariance r;
    r << 0.01;

    Ekf::MeasurementVector z;
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
    using Ekf = LibXR::EKF<2, 2, double>;
    Ekf::StateVector x0;
    x0 << 0.0, 1.0;
    Ekf::StateCovariance p0;
    p0 << 1.0, 0.3, 0.3, 1.0;
    Ekf ekf(x0, p0);

    Ekf::StateCovariance f = Ekf::StateCovariance::Identity();
    Ekf::StateCovariance q = Ekf::StateCovariance::Zero();
    Ekf::ObservationMatrix h = Ekf::ObservationMatrix::Identity();
    Ekf::MeasurementCovariance r = Ekf::MeasurementCovariance::Identity() * 0.5;

    Eigen::internal::set_is_malloc_allowed(false);
    for (int i = 0; i < 5; ++i)
    {
      ekf.Predict(f, q);
      Ekf::MeasurementVector z;
      z << 0.1 * i, 1.0 + 0.05 * i;
      ekf.Update(z, h, r);
      TEST_ASSERT((ekf.P - ekf.P.transpose()).cwiseAbs().maxCoeff() < 1e-12);
      TEST_ASSERT(ekf.P.diagonal().minCoeff() > 0.0);
    }
    Eigen::internal::set_is_malloc_allowed(true);
  }
}
