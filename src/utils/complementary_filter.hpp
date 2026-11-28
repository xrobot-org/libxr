#pragma once

#ifndef LIBXR_NO_EIGEN

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>

#include "libxr_def.hpp"

namespace LibXR
{

/**
 * @brief 互补滤波姿态估计（角度由陀螺积分、加速度计做慢修正）。
 *        Complementary attitude estimate: the gyro integrates the attitude and the
 *        accelerometer corrects it slowly.
 *
 * 内部维护一个四元数姿态状态、一个陀螺零偏状态和一个修正增益；每次 `Update()`
 * 先用陀螺推一步，再用“实测重力方向 × 预测重力方向”的叉积作为误差去修正角速度（并
 * 把该误差积分进零偏）。增益越大跟得越快、越信加速度计。
 * Maintains a quaternion attitude state, a gyro-bias state, and one correction gain.
 * Each `Update()` integrates the gyro first,
 * then rotates the state by the cross product of the measured and predicted gravity
 * directions, and integrates that error into the bias. A larger gain follows faster and
 * trusts the accelerometer more.
 *
 * 该滤波只对**可观测**的横滚/俯仰有效；没有磁力计时航向不可观测，`GetEuler()` 的
 * yaw 只是陀螺积分的结果，会漂。
 * Only roll and pitch are observable; without a magnetometer the heading is not, so the
 * yaw from `GetEuler()` is a pure gyro integral and will drift.
 *
 * @tparam Scalar 标量类型 / Scalar type.
 */
template <typename Scalar = LIBXR_DEFAULT_SCALAR>
class ComplementaryFilter
{
 public:
  /// @brief 修正增益与零偏积分增益 / Correction and bias-integration gains.
  struct Param
  {
    Scalar gain = static_cast<Scalar>(0.5);        ///< 加速度计修正强度 / accel gain
    Scalar bias_gain = static_cast<Scalar>(0.05);  ///< 零偏积分增益 / bias gain
  };

  ComplementaryFilter() = default;

  /// @brief 用参数构造 / Construct with parameters.
  explicit ComplementaryFilter(const Param& param) : param_(param)
  {
    ASSERT(std::isfinite(param_.gain) && param_.gain >= static_cast<Scalar>(0));
    ASSERT(std::isfinite(param_.bias_gain) && param_.bias_gain >= static_cast<Scalar>(0));
  }

  /// @brief 回到“水平、零偏为零” / Return to "level, zero bias".
  void Reset()
  {
    q_ = Eigen::Quaternion<Scalar>(static_cast<Scalar>(1), static_cast<Scalar>(0),
                                   static_cast<Scalar>(0), static_cast<Scalar>(0));
    bias_.setZero();
    tilt_initialized_ = false;
  }

  /// @brief 设置修正增益 / Set the correction gain.
  void SetGain(Scalar gain)
  {
    ASSERT(std::isfinite(gain) && gain >= static_cast<Scalar>(0));
    param_.gain = gain;
  }

  /// @brief 设置零偏积分增益 / Set the bias integration gain.
  void SetBiasGain(Scalar gain)
  {
    ASSERT(std::isfinite(gain) && gain >= static_cast<Scalar>(0));
    param_.bias_gain = gain;
  }

  /**
   * @brief 推进一步 / Advance one step.
   * @param gyro  机体角速度（rad/s）/ Body angular rate in rad/s.
   * @param accel 机体加速度（单位 g，静止模长约 1）/ Body acceleration in g (~1 at
   *              rest).
   * @param dt    步长（s）/ Step in seconds.
   */
  void Update(const Eigen::Matrix<Scalar, 3, 1>& gyro,
              const Eigen::Matrix<Scalar, 3, 1>& accel, Scalar dt)
  {
    if (!(std::isfinite(dt) && dt > static_cast<Scalar>(0)) || !gyro.allFinite())
    {
      return;
    }

    Eigen::Matrix<Scalar, 3, 1> rate = gyro - bias_;
    const Scalar accel_norm = accel.allFinite() ? accel.norm() : static_cast<Scalar>(0);

    // 加速度计只在“有可信重力方向”时参与修正：自由落体或剧烈机动时 |a| 远离 1g。
    // The accelerometer only corrects when it carries a believable gravity direction: in
    // free fall or hard manoeuvres |a| moves away from 1g.
    const Scalar g_tol = static_cast<Scalar>(0.25);
    const bool accel_valid = std::isfinite(accel_norm) &&
                             accel_norm > static_cast<Scalar>(1) - g_tol &&
                             accel_norm < static_cast<Scalar>(1) + g_tol;

    if (accel_valid)
    {
      const Eigen::Matrix<Scalar, 3, 1> measured = accel / accel_norm;
      if (!tilt_initialized_)
      {
        // 首帧有效重力直接对齐倾角，避免倒置上电（180° 反向）时叉积为零而卡死在水平。
        // Align tilt from the first valid gravity sample so upside-down startup (180 deg)
        // does not get stuck at identity with a zero cross product.
        q_ = Eigen::Quaternion<Scalar>::FromTwoVectors(measured, UpVector());
        q_.normalize();
        tilt_initialized_ = true;
      }
      else
      {
        // 预测的“上”方向：把世界系的 +Z 转到机体系再比 / predicted up in body frame
        const Eigen::Matrix<Scalar, 3, 1> predicted = q_.conjugate() * UpVector();
        Eigen::Matrix<Scalar, 3, 1> error = measured.cross(predicted);
        if (measured.dot(predicted) < static_cast<Scalar>(-0.999) &&
            error.norm() < static_cast<Scalar>(1e-3))
        {
          error = measured.unitOrthogonal();
        }

        rate += param_.gain * error;
        bias_ -= param_.bias_gain * error * dt;
      }
    }

    Integrate(rate, dt);
    q_.normalize();
  }

  /// @brief 当前四元数 / Current quaternion.
  [[nodiscard]] Eigen::Quaternion<Scalar> GetQuaternion() const { return q_; }

  /// @brief 当前欧拉角（横滚、俯仰、航向，弧度）/ Current Euler angles in radians.
  [[nodiscard]] Eigen::Matrix<Scalar, 3, 1> GetEuler() const
  {
    const Eigen::Matrix<Scalar, 3, 3> r = q_.toRotationMatrix();
    Eigen::Matrix<Scalar, 3, 1> euler;
    euler(0) = std::atan2(r(2, 1), r(2, 2));  // roll
    euler(1) = std::asin(
        -std::clamp(r(2, 0), static_cast<Scalar>(-1), static_cast<Scalar>(1)));  // pitch
    euler(2) = std::atan2(r(1, 0), r(0, 0));                                     // yaw
    return euler;
  }

  /// @brief 当前陀螺零偏估计（rad/s）/ Current gyro bias estimate.
  [[nodiscard]] Eigen::Matrix<Scalar, 3, 1> GetBias() const { return bias_; }

 private:
  static Eigen::Matrix<Scalar, 3, 1> UpVector()
  {
    return Eigen::Matrix<Scalar, 3, 1>(static_cast<Scalar>(0), static_cast<Scalar>(0),
                                       static_cast<Scalar>(1));
  }

  /// @brief 按角速度把四元数推进一步（一阶龙格-库塔）/ Integrate the quaternion (RK1).
  void Integrate(const Eigen::Matrix<Scalar, 3, 1>& rate, Scalar dt)
  {
    const Scalar angle = rate.norm() * dt;
    if (angle <= static_cast<Scalar>(0))
    {
      return;
    }
    const Eigen::Matrix<Scalar, 3, 1> axis = rate.normalized();
    const Scalar half = angle * static_cast<Scalar>(0.5);
    const Eigen::Quaternion<Scalar> delta(std::cos(half), axis(0) * std::sin(half),
                                          axis(1) * std::sin(half),
                                          axis(2) * std::sin(half));
    q_ = (q_ * delta).normalized();
  }

  Param param_{};
  Eigen::Quaternion<Scalar> q_{static_cast<Scalar>(1), static_cast<Scalar>(0),
                               static_cast<Scalar>(0), static_cast<Scalar>(0)};
  Eigen::Matrix<Scalar, 3, 1> bias_{Eigen::Matrix<Scalar, 3, 1>::Zero()};
  bool tilt_initialized_ = false;
};

}  // namespace LibXR

#endif  // LIBXR_NO_EIGEN
