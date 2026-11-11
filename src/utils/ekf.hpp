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
 * 状态向量 `x` 与协方差矩阵 `P` 是公有成员，调用方可以在两次预测之间直接改写状态
 * （固定某个自由度、限制角速度、按观测修正某个分量），但改写后的协方差必须仍然描述
 * 改写后的状态，否则后续更新会按旧不确定性加权。
 * The state vector `x` and covariance `P` are public so callers may adjust the state
 * between predictions (pin a degree of freedom, clamp a rate, correct one component
 * from an observation); after such an edit the covariance must still describe the
 * edited state, otherwise later updates weigh the state with stale uncertainty.
 *
 * 协方差更新使用 Joseph 形式，数值上保持对称正定，适用于过程噪声与观测噪声都接近
 * 奇异或者状态维度动态变化的场景。
 * The covariance update uses the Joseph form, which stays symmetric positive
 * semi-definite and suits near-singular process or measurement noise.
 *
 * 角度类状态通过可自定义钩子处理：`x_add` 决定状态增量如何叠加（例如把 yaw 叠加后
 * 归一化到 [-π, π)），`z_subtract` 决定残差如何相减（例如把角度残差归一化），`f` 与
 * `h` 分别承载非线性状态转移与非线性观测。
 * Angular states use customizable hooks: `x_add` defines how a state increment is
 * applied (for example wrapping yaw into [-π, π) after the addition), `z_subtract`
 * defines how a residual is formed (for example wrapping an angle residual), and `f`
 * and `h` carry the nonlinear transition and observation.
 *
 * 钩子分两种形式：每次调用传入的 `f`、`h`、`z_subtract` 是模板参数，编译期内联；构造时
 * 传入并长期保存的 `x_add` 是裸函数指针（与 RBTree 的比较器一致）。两种形式都在编译期
 * 绑定，可调用对象零存储开销；滤波器只依赖 Eigen 动态矩阵的常规存储行为，并且不依赖
 * 异常与 RTTI，可在裸金属、RTOS 与 Linux 上使用。
 * The hooks take two forms: the per-call `f`, `h` and `z_subtract` are template
 * parameters that inline at compile time, and `x_add`, supplied at construction and
 * kept for the object lifetime, is a plain function pointer, matching RBTree's
 * comparator. Both forms bind at compile time and add zero storage for the callable;
 * the filter relies only on Eigen's usual dynamic-matrix storage and needs neither
 * exceptions nor RTTI, so it runs on bare-metal, RTOS and Linux targets alike.
 *
 * 新息统计：每次更新计算归一化新息平方（NIS）与归一化估计误差平方（NEES），与
 * `nis_threshold` / `nees_threshold` 比较后计数。近期 NIS 失败记录放在编译期固定容量的
 * 环形缓冲里，容量由 `WindowCapacity` 决定，活跃窗口长度用 `SetWindowSize` 调整；
 * 统计缓冲区不分配堆内存。
 * Innovation statistics: every update computes the normalized innovation squared (NIS)
 * and the normalized estimation error squared (NEES) and compares them with
 * `nis_threshold` / `nees_threshold`. Recent NIS failures live in a ring buffer whose
 * capacity is fixed at compile time by `WindowCapacity`, with the active window length
 * set through `SetWindowSize`; the statistics buffer allocates no heap memory.
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
  /// 状态向量类型。 State vector type.
  using StateVector = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;

  /// 状态协方差矩阵类型。 State covariance matrix type.
  using StateCovariance = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;

  /// 状态转移矩阵类型，与状态协方差同形。 State transition matrix type, same shape as
  /// the state covariance.
  using TransitionMatrix = StateCovariance;

  /// 观测向量类型。 Measurement vector type.
  using MeasurementVector = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;

  /// 观测噪声协方差矩阵类型。 Measurement noise covariance matrix type.
  using MeasurementCovariance = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;

  /// 观测矩阵类型，行数为观测维度、列数为状态维度。 Observation matrix type, with one
  /// row per measurement and one column per state component.
  using ObservationMatrix = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;

  /**
   * @brief 状态增量叠加函数指针，只接受捕获列表为空的可调用对象。
   *        State increment application function pointer; the callable captures nothing.
   *
   * 构造后长期持有的钩子使用裸函数指针（与 RBTree 的比较器一致）：钩子自身不分配堆
   * 内存、可拷贝、可放在裸金属和 RTOS 目标上。需要携带状态的叠加逻辑时，把状态放在
   * 函数内的静态上下文或全局状态里。
   * A hook stored for the object lifetime is a plain function pointer, matching
   * RBTree's comparator: no heap allocation, copyable, and usable on bare-metal and
   * RTOS targets. Stateful addition logic keeps its state in static or global storage
   * inside the function.
   */
  using StateAddFunction = StateVector (*)(const StateVector&, const StateVector&);

  StateVector x;      ///< 状态向量 State vector
  StateCovariance P;  ///< 状态协方差 State covariance

  /// NIS 失败环形缓冲容量，等于模板参数 WindowCapacity。
  /// Capacity of the NIS failure ring buffer, equal to the WindowCapacity argument.
  static constexpr size_t WINDOW_CAPACITY = WindowCapacity;

  Scalar nis_threshold = static_cast<Scalar>(0.711);   ///< NIS 门限 NIS threshold
  Scalar nees_threshold = static_cast<Scalar>(0.711);  ///< NEES 门限 NEES threshold

  Scalar last_nis{};                ///< 最近一次更新的 NIS NIS of the last update
  Scalar last_nees{};               ///< 最近一次更新的 NEES NEES of the last update
  MeasurementVector last_residual;  ///< 最近一次更新的残差 Residual of the last update

  /**
   * @brief 默认状态增量叠加：普通加法。
   *        Default state increment application: plain addition.
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
   *        Construct an empty filter with a zero-sized state.
   */
  EKF() = default;

  /**
   * @brief 使用初始状态、初始协方差和状态增量叠加方式构造滤波器。
   *        Construct a filter from an initial state, covariance, and state increment
   *        application.
   *
   * @param x0 初始状态向量。 Initial state vector.
   * @param P0 初始状态协方差，必须为方阵且维度与 x0 一致。
   *           Initial state covariance, square and matching x0.
   * @param x_add 状态增量叠加方式，默认普通加法。必须是捕获列表为空的可调用对象。
   *               State increment application, plain addition by default; the callable
   *               must capture nothing.
   */
  EKF(const StateVector& x0, const StateCovariance& P0,
      StateAddFunction x_add = PlusState)
      : x(x0), P(P0), I_(StateCovariance::Identity(x0.rows(), x0.rows())), x_add_(x_add)
  {
    ASSERT(P0.rows() == P0.cols());
    ASSERT(P0.rows() == x0.size());
  }

  /**
   * @brief 运行期替换状态增量叠加函数，只接受捕获列表为空的可调用对象。
   *        Replace the state increment application at runtime; the callable captures
   *        nothing.
   *
   * 下一次更新开始使用新的叠加逻辑，已估计的状态与协方差保持不变。
   * The next update uses the new application; the estimated state and covariance are
   * kept as they are.
   *
   * @param x_add 新的状态增量叠加函数，不能为空。
   *               New state increment application, must not be null.
   */
  void SetStateAdd(StateAddFunction x_add)
  {
    ASSERT(x_add != nullptr);
    x_add_ = x_add;
  }

  /**
   * @brief 使用线性状态转移矩阵预测一步。
   *        Predict one step with a linear transition matrix.
   *
   * @param F 状态转移矩阵。 State transition matrix.
   * @param Q 过程噪声协方差。 Process noise covariance.
   * @return 预测后的状态向量。 Predicted state vector.
   */
  StateVector Predict(const TransitionMatrix& F, const StateCovariance& Q)
  {
    return Predict(F, Q, [&F](const StateVector& prior) { return F * prior; });
  }

  /**
   * @brief 使用自定义非线性状态转移预测一步。
   *        Predict one step with a custom nonlinear transition.
   *
   * @param F 状态转移矩阵，同时用于协方差传播。
   *           State transition matrix, also used to propagate the covariance.
   * @param Q 过程噪声协方差。 Process noise covariance.
   * @param f 非线性状态转移函数，按模板参数传入，可内联且不分配内存。
   *           Nonlinear transition function, taken as a template parameter so it inlines
   *           and allocates nothing.
   * @return 预测后的状态向量。 Predicted state vector.
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
   *        Update the state with a linear observation matrix.
   *
   * @param z 观测向量。 Measurement vector.
   * @param H 观测矩阵。 Observation matrix.
   * @param R 观测噪声协方差。 Measurement noise covariance.
   * @return 更新后的状态向量。 Updated state vector.
   */
  StateVector Update(const MeasurementVector& z, const ObservationMatrix& H,
                     const MeasurementCovariance& R)
  {
    return Update(
        z, H, R, [&H](const StateVector& state) { return H * state; }, MinusMeasurement);
  }

  /**
   * @brief 使用自定义非线性观测函数更新状态。
   *        Update the state with a custom nonlinear observation function.
   *
   * @param z 观测向量。 Measurement vector.
   * @param H 观测函数在当前状态处的雅可比。 Jacobian of the observation at the current
   *          state.
   * @param R 观测噪声协方差。 Measurement noise covariance.
   * @param h 非线性观测函数，按模板参数传入，可内联且不分配内存。
   *           Nonlinear observation function, taken as a template parameter so it
   *           inlines and allocates nothing.
   * @return 更新后的状态向量。 Updated state vector.
   */
  template <typename ObservationFunction>
  StateVector Update(const MeasurementVector& z, const ObservationMatrix& H,
                     const MeasurementCovariance& R, ObservationFunction&& h)
  {
    return Update(z, H, R, h, MinusMeasurement);
  }

  /**
   * @brief 使用自定义非线性观测函数和自定义残差方式更新状态。
   *        Update the state with a custom nonlinear observation and residual handling.
   *
   * 残差与 NIS/NEES 均在状态与协方差更新之后计算，因此 `last_residual` 是更新后的
   * 后验残差，不是更新前的预测残差。
   * The residual and the NIS/NEES statistics are computed after the state and
   * covariance are updated, so `last_residual` is the post-update residual rather
   * than the pre-update innovation.
   *
   * @param z 观测向量。 Measurement vector.
   * @param H 观测函数在当前状态处的雅可比。 Jacobian of the observation at the current
   *          state.
   * @param R 观测噪声协方差。 Measurement noise covariance.
   * @param h 非线性观测函数，按模板参数传入，可内联且不分配内存。
   *           Nonlinear observation function, taken as a template parameter so it
   *           inlines and allocates nothing.
   * @param z_subtract 残差计算方式，默认普通减法；同样是模板参数。
   *                   Residual computation, plain subtraction by default; also a
   *                   template parameter.
   * @return 更新后的状态向量。 Updated state vector.
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
   * @brief 设置活跃窗口长度，取值上限为 WINDOW_CAPACITY。
   *        Set the active window length, capped at WINDOW_CAPACITY.
   *
   * @param size 期望的窗口长度，超过容量时按容量处理。
   *             Requested window length; values above the capacity are clamped.
   */
  void SetWindowSize(size_t size)
  {
    window_size_ = size < WINDOW_CAPACITY ? size : WINDOW_CAPACITY;
  }

  /**
   * @brief 返回当前活跃窗口长度。
   *        Return the current active window length.
   */
  size_t WindowSize() const { return window_size_; }

  /**
   * @brief 返回活跃窗口内 NIS 超限的次数。
   *        Return the number of NIS failures inside the active window.
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

  /**
   * @brief 返回 NIS 超限的更新次数。
   *        Return the number of updates whose NIS exceeded the threshold.
   */
  int NisFailCount() const { return nis_count_; }

  /**
   * @brief 返回 NEES 超限的更新次数。
   *        Return the number of updates whose NEES exceeded the threshold.
   */
  int NeesFailCount() const { return nees_count_; }

  /**
   * @brief 返回已完成的更新次数。
   *        Return the number of completed updates.
   */
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
