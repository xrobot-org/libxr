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
 * @brief 支持非线性状态转移与非线性观测的扩展卡尔曼滤波器。
 *        Extended Kalman filter with optional nonlinear transition and observation.
 *
 * `x` 与 `P` 为公有成员，可在两次预测之间直接改写，但改写后的协方差必须仍描述新状态。
 * `x` and `P` are public and may be edited between predictions; afterwards the
 * covariance must still describe the edited state.
 *
 * 协方差更新使用 Joseph 形式保持对称正定，新息统计使用定容环形缓冲，不做堆分配。
 * The covariance update uses the Joseph form to stay symmetric positive semi-definite;
 * the innovation statistics use a fixed-capacity ring buffer with no heap allocation.
 *
 * @tparam Scalar 标量类型，默认为 DefaultScalar。
 *                Scalar type, defaults to DefaultScalar.
 * @tparam WindowCapacity NIS 失败环形缓冲容量，同时是默认窗口长度。
 *                        Capacity of the NIS failure ring buffer, also the default
 *                        window length.
 */
template <typename Scalar = DefaultScalar, size_t WindowCapacity = 100>
class EKF
{
 public:
  /// 状态向量。State vector.
  using StateVector = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;

  /// 状态协方差。State covariance.
  using StateCovariance = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;

  /// 状态转移矩阵，与状态协方差同形。State transition matrix, same shape as covariance.
  using TransitionMatrix = StateCovariance;

  /// 观测向量。Measurement vector.
  using MeasurementVector = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;

  /// 观测噪声协方差。Measurement noise covariance.
  using MeasurementCovariance = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;

  /// 观测矩阵，行数为观测维度。Observation matrix, one row per measurement.
  using ObservationMatrix = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;

  /// 状态增量叠加函数指针，可调用对象不得捕获任何变量。
  /// State increment function pointer; the callable must capture nothing.
  using StateAddFunction = StateVector (*)(const StateVector&, const StateVector&);

  StateVector x;      ///< 状态向量 State vector
  StateCovariance P;  ///< 状态协方差 State covariance

  /// NIS 失败环形缓冲容量。Capacity of the NIS failure ring buffer.
  static constexpr size_t WINDOW_CAPACITY = WindowCapacity;

  Scalar nis_threshold = static_cast<Scalar>(0.711);   ///< NIS 门限 NIS threshold
  Scalar nees_threshold = static_cast<Scalar>(0.711);  ///< NEES 门限 NEES threshold

  Scalar last_nis{};                ///< 最近一次更新的 NIS。NIS of the last update
  Scalar last_nees{};               ///< 最近一次更新的 NEES。NEES of the last update
  MeasurementVector last_residual;  ///< 最近一次更新后的残差 Posterior residual

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

  /**
   * @brief 构造空滤波器，状态维度为 0。
   *        Constructs an empty filter with a zero-sized state.
   */
  EKF() = default;

  /**
   * @brief 由初始状态、初始协方差和状态增量叠加方式构造。
   *        Constructs from an initial state, covariance, and state increment.
   * @param x0 初始状态向量。Initial state vector.
   * @param P0 初始协方差，方阵且维度与 x0 一致。Initial covariance, square and
   *           matching x0.
   * @param x_add 状态增量叠加方式，默认普通加法；不得捕获变量。
   *              State increment, plain addition by default; must capture nothing.
   */
  EKF(const StateVector& x0, const StateCovariance& P0,
      StateAddFunction x_add = PlusState)
      : x(x0), P(P0), I_(StateCovariance::Identity(x0.rows(), x0.rows())), x_add_(x_add)
  {
    ASSERT(P0.rows() == P0.cols());
    ASSERT(P0.rows() == x0.size());
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
   * @param F 状态转移矩阵，同时用于协方差传播。State transition matrix, also used for the
   *          covariance propagation.
   * @param Q 过程噪声协方差。Process noise covariance.
   * @param f 非线性状态转移函数，按模板参数传入，编译期内联。Nonlinear transition,
   *          passed as a template parameter and inlined at compile time.
   * @return 预测后的状态。Predicted state.
   */
  template <typename TransitionFunction>
  StateVector Predict(const TransitionMatrix& F, const StateCovariance& Q,
                      TransitionFunction&& f)
  {
    ASSERT(F.rows() == F.cols());
    ASSERT(F.rows() == x.size());
    ASSERT(Q.rows() == Q.cols());
    ASSERT(Q.rows() == x.size());

    P = F * P * F.transpose() + Q;
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
   * @param h 非线性观测函数，按模板参数传入。Nonlinear observation, passed as a template
   *          parameter.
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
   * @param h 非线性观测函数，按模板参数传入。Nonlinear observation, passed as a template
   *          parameter.
   * @param z_subtract 残差计算方式，默认普通减法。Residual computation, plain subtraction
   *                   by default.
   * @return 更新后的状态。Updated state.
   */
  template <typename ObservationFunction, typename ResidualFunction>
  StateVector Update(const MeasurementVector& z, const ObservationMatrix& H,
                     const MeasurementCovariance& R, ObservationFunction&& h,
                     ResidualFunction&& z_subtract)
  {
    ASSERT(H.cols() == x.size());
    ASSERT(H.rows() == R.rows());
    ASSERT(H.rows() == R.cols());
    ASSERT(H.rows() == z.size());

    const StateVector x_prior = x;
    const StateCovariance K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
    P = (I_ - K * H) * P * (I_ - K * H).transpose() + K * R * K.transpose();
    x = x_add_(x, K * z_subtract(z, h(x)));

    const MeasurementVector residual = z_subtract(z, h(x));
    const MeasurementCovariance S = H * P * H.transpose() + R;
    const Scalar nis = (residual.transpose() * S.inverse() * residual)(0, 0);
    const StateVector dx = x - x_prior;
    const Scalar nees = (dx.transpose() * P.inverse() * dx)(0, 0);

    if (nis > nis_threshold)
    {
      ++nis_count_;
    }
    if (nees > nees_threshold)
    {
      ++nees_count_;
    }
    ++total_count_;

    last_nis = nis;
    last_nees = nees;
    last_residual = residual;

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

  /// 返回 NEES 超限的总次数。Returns the total NEES failure count.
  int NeesFailCount() const { return nees_count_; }

  /// 返回已完成的更新次数。Returns the number of completed updates.
  int UpdateCount() const { return total_count_; }

 private:
  StateCovariance I_;
  StateAddFunction x_add_;
  std::array<int, WINDOW_CAPACITY> nis_window_{};
  size_t nis_window_head_ = 0;
  size_t nis_window_count_ = 0;
  size_t window_size_ = WINDOW_CAPACITY;
  int nis_count_ = 0;
  int nees_count_ = 0;
  int total_count_ = 0;
};

}  // namespace LibXR

#endif
