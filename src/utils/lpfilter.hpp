#pragma once

#include <cmath>

#include "libxr_def.hpp"

namespace LibXR
{

using DefaultScalar = LIBXR_DEFAULT_SCALAR;

namespace Detail
{
/**
 * @brief 取滤波器系数类型：标量取自身，类类型取其 Scalar 成员。
 *        Coefficient type: the type itself for a scalar, the nested Scalar otherwise.
 */
template <typename T>
struct ScalarOf
{
  using type = T;  ///< 标量类型本身。The scalar type itself.
};

/**
 * @brief 带 Scalar 成员的类类型特化。Specialization for types with a Scalar member.
 */
template <typename T>
  requires requires { typename T::Scalar; }
struct ScalarOf<T>
{
  using type = typename T::Scalar;  ///< 嵌套的 Scalar 类型。The nested Scalar type.
};
}  // namespace Detail

/**
 * @class LPFilter
 * @brief 一阶低通滤波器（指数滑动平均）。
 *        First-order low-pass filter (exponential moving average).
 *
 * 递推为 `state += alpha * (x - state)`，其中 `alpha = dt / (tau + dt)`。第一个样本直接
 * 成为初始状态，调用方不需要自己判断初始化。
 * The recursion is `state += alpha * (x - state)` with `alpha = dt / (tau + dt)`. The
 * first sample becomes the initial state, so callers never track initialization.
 *
 * @tparam T 被滤波的数据类型，如 float 或 Eigen 向量。
 *           Filtered type, such as float or an Eigen vector.
 * @tparam Scalar 系数与时间常数的类型，默认取自 T。
 *                 Coefficient and time-constant type, taken from T by default.
 */
template <typename T = DefaultScalar,
          typename Scalar = typename Detail::ScalarOf<T>::type>
class LPFilter
{
 public:
  /**
   * @brief 滤波器参数。Filter parameters.
   */
  struct Param
  {
    Scalar time_constant = Scalar(0);  ///< 时间常数 tau，必须大于 0。Time constant, > 0.
  };

  /**
   * @brief 由参数构造滤波器。Constructs a filter from its parameters.
   * @param param 滤波器参数，时间常数必须大于 0。
   *              Filter parameters; the time constant must be positive.
   */
  explicit LPFilter(const Param& param) : param_(param)
  {
    ASSERT(std::isfinite(param_.time_constant) && param_.time_constant > Scalar(0));
    ZeroState(state_);
  }

  /**
   * @brief 替换参数，保留当前状态。Replaces the parameters, keeping the state.
   * @param param 新参数，时间常数必须大于 0。
   *              New parameters; the time constant must be positive.
   */
  void SetParam(const Param& param)
  {
    ASSERT(std::isfinite(param.time_constant) && param.time_constant > Scalar(0));
    param_ = param;
  }

  /**
   * @brief 返回当前参数。Returns the current parameters.
   */
  const Param& GetParam() const { return param_; }

  /**
   * @brief 按截止频率设置时间常数。Sets the time constant from a cutoff frequency.
   * @param cutoff_frequency 截止频率，必须大于 0。Cutoff frequency, must be positive.
   */
  void SetCutoffFrequency(Scalar cutoff_frequency)
  {
    ASSERT(std::isfinite(cutoff_frequency) && cutoff_frequency > Scalar(0));
    param_.time_constant = Scalar(1) / (static_cast<Scalar>(TWO_PI) * cutoff_frequency);
  }

  /**
   * @brief 按给定采样间隔下的系数设置时间常数。
   *        Sets the time constant from a coefficient at a given sample interval.
   *
   * 系数与采样间隔相关，同一个系数在不同采样率下不是同一个滤波器。
   * The coefficient depends on the sample interval; the same coefficient at another
   * rate is a different filter.
   *
   * @param alpha 系数，取值 (0, 1)。Coefficient in (0, 1).
   * @param dt 采样间隔，必须大于 0。Sample interval, must be positive.
   */
  void SetAlpha(Scalar alpha, Scalar dt)
  {
    ASSERT(std::isfinite(alpha) && alpha > Scalar(0) && alpha < Scalar(1));
    ASSERT(std::isfinite(dt) && dt > Scalar(0));
    param_.time_constant = dt * (Scalar(1) - alpha) / alpha;
  }

  /**
   * @brief 返回给定采样间隔下的系数。Returns the coefficient for a sample interval.
   * @param dt 采样间隔，必须大于 0。Sample interval, must be positive.
   */
  Scalar GetAlpha(Scalar dt) const
  {
    ASSERT(std::isfinite(dt) && dt > Scalar(0));
    return dt / (param_.time_constant + dt);
  }

  /**
   * @brief 重置滤波器状态，下一次更新重新吸收样本。
   *        Resets the filter state so that the next update absorbs its sample.
   */
  void Reset()
  {
    ZeroState(state_);
    initialized_ = false;
  }

  /**
   * @brief 按采样间隔推进一次。Advances one sample by the given interval.
   *
   * `dt <= 0` 或非有限值时保持状态不变；第一个样本直接成为状态。
   * Keeps the state when `dt` is non-positive or non-finite; the first sample becomes
   * the state.
   *
   * @param x 本次样本。Sample.
   * @param dt 距上次样本的间隔。Interval since the previous sample.
   * @return 滤波后的状态。Filtered state.
   */
  const T& Update(const T& x, Scalar dt)
  {
    if (!initialized_)
    {
      state_ = x;
      initialized_ = true;
      return state_;
    }

    if (!(std::isfinite(dt) && dt > Scalar(0)))
    {
      return state_;
    }

    state_ = state_ + GetAlpha(dt) * (x - state_);
    return state_;
  }

  /**
   * @brief 返回当前状态（首次 Update 前或 Reset 后为零值）。
   *        Returns the current state (zero before the first Update or after Reset).
   */
  const T& State() const { return state_; }

  /**
   * @brief 返回是否已吸收过首样本。Returns whether the first sample has been absorbed.
   */
  bool IsInitialized() const { return initialized_; }

 private:
  static void ZeroState(T& s)
  {
    if constexpr (requires { s.setZero(); })
    {
      s.setZero();
    }
    else
    {
      s = T{};
    }
  }

  Param param_;
  T state_{};
  bool initialized_ = false;
};

}  // namespace LibXR
