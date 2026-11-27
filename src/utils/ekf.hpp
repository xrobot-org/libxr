#pragma once

#ifndef LIBXR_NO_EIGEN

#include <Eigen/Dense>
#include <array>

#include "libxr_def.hpp"

namespace LibXR
{

using DefaultScalar = LIBXR_DEFAULT_SCALAR;

/**
 * @class EKF
 * @brief 支持非线性状态转移与非线性观测的固定维度扩展卡尔曼滤波器。
 *        Fixed-size extended Kalman filter with optional nonlinear transition and
 *        observation.
 *
 * 状态维度 `StateDim` 与观测维度 `MeasDim` 在编译期固定，内部全部使用栈上固定尺寸
 * Eigen 矩阵与定容环形缓冲，预测与更新过程零堆分配。
 * Both `StateDim` and `MeasDim` are fixed at compile time; all internal Eigen matrices
 * and the innovation failure ring buffer use fixed storage with zero heap allocation
 * during `Predict()` and `Update()`.
 *
 * `x` 与 `P` 为公有成员，可在两次预测之间直接改写，但改写后的协方差必须仍描述新状态。
 * `x` and `P` are public and may be edited between predictions; afterwards the
 * covariance must still describe the edited state.
 *
 * @tparam StateDim 状态向量维度，必须大于 0。State vector dimension, must be > 0.
 * @tparam MeasDim 观测向量维度，必须大于 0。Measurement vector dimension, must be > 0.
 * @tparam Scalar 标量类型，默认为 DefaultScalar。
 *                Scalar type, defaults to DefaultScalar.
 * @tparam WindowCapacity NIS 失败环形缓冲容量，同时是默认窗口长度。
 *                        Capacity of the NIS failure ring buffer, also the default
 *                        window length.
 */
template <int StateDim, int MeasDim, typename Scalar = DefaultScalar,
          size_t WindowCapacity = 100>
class EKF
{
  static_assert(StateDim > 0, "StateDim must be positive");
  static_assert(MeasDim > 0, "MeasDim must be positive");

 public:
  /// 状态向量。State vector.
  using StateVector = Eigen::Matrix<Scalar, StateDim, 1>;

  /// 状态协方差。State covariance.
  using StateCovariance = Eigen::Matrix<Scalar, StateDim, StateDim>;

  /// 状态转移矩阵，与状态协方差同形。State transition matrix, same shape as covariance.
  using TransitionMatrix = StateCovariance;

  /// 观测向量。Measurement vector.
  using MeasurementVector = Eigen::Matrix<Scalar, MeasDim, 1>;

  /// 观测噪声协方差。Measurement noise covariance.
  using MeasurementCovariance = Eigen::Matrix<Scalar, MeasDim, MeasDim>;

  /// 观测矩阵，行数为观测维度。Observation matrix, one row per measurement.
  using ObservationMatrix = Eigen::Matrix<Scalar, MeasDim, StateDim>;

  /// 卡尔曼增益矩阵。Kalman gain matrix.
  using KalmanGain = Eigen::Matrix<Scalar, StateDim, MeasDim>;

  /// 状态增量叠加函数指针，可调用对象不得捕获任何变量。
  /// State increment function pointer; the callable must capture nothing.
  using StateAddFunction = StateVector (*)(const StateVector&, const StateVector&);

  StateVector x = StateVector::Zero();              ///< 状态向量 State vector
  StateCovariance P = StateCovariance::Identity();  ///< 状态协方差 State covariance

  /// NIS 失败环形缓冲容量。Capacity of the NIS failure ring buffer.
  static constexpr size_t WINDOW_CAPACITY = WindowCapacity;

  Scalar nis_threshold = static_cast<Scalar>(0.711);  ///< NIS 门限 NIS threshold

  Scalar last_nis{};  ///< 最近一次更新的先验 NIS。Prior NIS of last update
  MeasurementVector last_innovation =  ///< 最近一次更新的先验新息。Prior innovation
      MeasurementVector::Zero();

  /**
   * @brief 默认状态增量叠加：普通加法。
   *        Default state increment: plain addition.
   */
  static StateVector PlusState(const StateVector& a, const StateVector& b)
  {
    return a + b;
  }

  /**
   * @brief 默认残差相减：普通减法。
   *        Default residual subtraction: plain subtraction.
   */
  static MeasurementVector MinusMeasurement(const MeasurementVector& a,
                                            const MeasurementVector& b)
  {
    return a - b;
  }

  EKF() = delete;

  /**
   * @brief 由初始状态、初始协方差和状态增量叠加方式构造。
   *        Constructs from an initial state, covariance, and state increment.
   * @param x0 初始状态向量。Initial state vector.
   * @param P0 初始协方差。Initial covariance.
   * @param x_add 状态增量叠加方式，默认普通加法；不能为空且不得捕获变量。
   *              State increment, plain addition by default; must be non-null and
   *              capture nothing.
   */
  EKF(const StateVector& x0, const StateCovariance& P0,
      StateAddFunction x_add = PlusState)
      : x(x0), P(P0), x_add_(x_add)
  {
    ASSERT(x_add_ != nullptr);
  }

  /**
   * @brief 运行期替换状态增量叠加函数。
   *        Replaces the state increment at runtime.
   * @param x_add 新的叠加函数，不能为空。New addition function, must not be null.
   */
  void SetStateAdd(StateAddFunction x_add)
  {
    ASSERT(x_add != nullptr);
    x_add_ = x_add;
  }

  /**
   * @brief 使用线性状态转移矩阵预测一步。
   *        Predicts one step with a linear transition.
   * @param F 状态转移矩阵。State transition matrix.
   * @param Q 过程噪声协方差。Process noise covariance.
   * @return 预测后的状态。Predicted state.
   */
  StateVector Predict(const TransitionMatrix& F, const StateCovariance& Q)
  {
    return Predict(F, Q, [&F](const StateVector& prior) { return F * prior; });
  }

  /**
   * @brief 使用自定义非线性状态转移预测一步。
   *        Predicts one step with a custom nonlinear transition.
   * @param F 状态转移矩阵，同时用于协方差传播。State transition matrix, also used for
   *          the covariance propagation.
   * @param Q 过程噪声协方差。Process noise covariance.
   * @param f 非线性状态转移函数，按模板参数传入，编译期内联。Nonlinear transition,
   *          passed as a template parameter and inlined at compile time.
   * @return 预测后的状态。Predicted state.
   */
  template <typename TransitionFunction>
  StateVector Predict(const TransitionMatrix& F, const StateCovariance& Q,
                      TransitionFunction&& f)
  {
    P = (F * P * F.transpose() + Q).eval();
    x = f(x);
    return x;
  }

  /**
   * @brief 使用线性观测矩阵更新状态。
   *        Updates the state with a linear observation.
   * @param z 观测向量。Measurement vector.
   * @param H 观测矩阵。Observation matrix.
   * @param R 观测噪声协方差。Measurement noise covariance.
   * @return 更新后的状态。Updated state.
   */
  StateVector Update(const MeasurementVector& z, const ObservationMatrix& H,
                     const MeasurementCovariance& R)
  {
    return Update(
        z, H, R, [&H](const StateVector& state) { return H * state; }, MinusMeasurement);
  }

  /**
   * @brief 使用自定义非线性观测函数更新状态。
   *        Updates the state with a custom nonlinear observation.
   * @param z 观测向量。Measurement vector.
   * @param H 观测函数在当前状态处的雅可比。Jacobian of the observation.
   * @param R 观测噪声协方差。Measurement noise covariance.
   * @param h 非线性观测函数，按模板参数传入。Nonlinear observation, passed as a
   *          template parameter.
   * @return 更新后的状态。Updated state.
   */
  template <typename ObservationFunction>
  StateVector Update(const MeasurementVector& z, const ObservationMatrix& H,
                     const MeasurementCovariance& R, ObservationFunction&& h)
  {
    return Update(z, H, R, h, MinusMeasurement);
  }

  /**
   * @brief 使用自定义非线性观测与残差方式更新状态。
   *        Updates the state with a custom nonlinear observation and residual.
   * @param z 观测向量。Measurement vector.
   * @param H 观测函数在当前状态处的雅可比。Jacobian of the observation.
   * @param R 观测噪声协方差。Measurement noise covariance.
   * @param h 非线性观测函数，按模板参数传入。Nonlinear observation, passed as a
   *          template parameter.
   * @param z_subtract 残差计算方式，默认普通减法。Residual computation, plain
   *                   subtraction by default.
   * @return 更新后的状态。Updated state.
   */
  template <typename ObservationFunction, typename ResidualFunction>
  StateVector Update(const MeasurementVector& z, const ObservationMatrix& H,
                     const MeasurementCovariance& R, ObservationFunction&& h,
                     ResidualFunction&& z_subtract)
  {
    // NIS 使用先验新息与先验新息协方差 S = H P^- H^T + R，并与卡尔曼增益共用 S^-1。
    // NIS uses the prior innovation and prior covariance S = H P^- H^T + R, sharing S^-1
    // with the Kalman gain.
    const MeasurementVector y = z_subtract(z, h(x));
    const MeasurementCovariance S = H * P * H.transpose() + R;
    const MeasurementCovariance S_inv = S.inverse();
    const Scalar nis = (y.transpose() * S_inv * y)(0, 0);

    const KalmanGain K = P * H.transpose() * S_inv;
    const StateCovariance i_kh = StateCovariance::Identity() - K * H;
    P = (i_kh * P * i_kh.transpose() + K * R * K.transpose()).eval();
    x = x_add_(x, K * y);

    if (nis > nis_threshold)
    {
      ++nis_count_;
    }
    ++total_count_;

    last_nis = nis;
    last_innovation = y;

    if constexpr (WINDOW_CAPACITY > 0)
    {
      nis_window_[nis_window_head_] = nis > nis_threshold ? 1 : 0;
      nis_window_head_ = (nis_window_head_ + 1) % WINDOW_CAPACITY;
      if (nis_window_count_ < WINDOW_CAPACITY)
      {
        ++nis_window_count_;
      }
    }

    return x;
  }

  /**
   * @brief 计算给定真值状态下的归一化估计误差平方（NEES）。
   *        Computes the Normalized Estimation Error Squared (NEES) against a known
   *        ground-truth state.
   * @param x_true 真实状态向量。Ground-truth state vector.
   * @return NEES 值 `(x - x_true)^T P^{-1} (x - x_true)`。NEES value.
   */
  Scalar ComputeNEES(const StateVector& x_true) const
  {
    const StateVector err = x - x_true;
    return (err.transpose() * P.inverse() * err)(0, 0);
  }

  /**
   * @brief 设置活跃窗口长度，上限为 WINDOW_CAPACITY。
   *        Sets the active window length, capped at WINDOW_CAPACITY.
   * @param size 期望的窗口长度，超过容量时按容量处理。Requested length, clamped to the
   *             capacity.
   */
  void SetWindowSize(size_t size)
  {
    window_size_ = size < WINDOW_CAPACITY ? size : WINDOW_CAPACITY;
  }

  /// 返回当前活跃窗口长度。Returns the active window length.
  size_t WindowSize() const { return window_size_; }

  /**
   * @brief 返回活跃窗口内 NIS 超限次数。
   *        Returns NIS failures inside the active window.
   */
  int NisFailureCount() const
  {
    const size_t length =
        nis_window_count_ < window_size_ ? nis_window_count_ : window_size_;
    int failures = 0;
    for (size_t i = 0; i < length; ++i)
    {
      const size_t index = (nis_window_head_ + WINDOW_CAPACITY - 1 - i) % WINDOW_CAPACITY;
      failures += nis_window_[index];
    }
    return failures;
  }

  /// 返回 NIS 超限的总次数。Returns the total NIS failure count.
  int NisFailCount() const { return nis_count_; }

  /// 返回已完成的更新次数。Returns the number of completed updates.
  int UpdateCount() const { return total_count_; }

 private:
  StateAddFunction x_add_ = PlusState;
  std::array<int, WINDOW_CAPACITY> nis_window_{};
  size_t nis_window_head_ = 0;
  size_t nis_window_count_ = 0;
  size_t window_size_ = WINDOW_CAPACITY;
  int nis_count_ = 0;
  int total_count_ = 0;
};

}  // namespace LibXR

#endif
