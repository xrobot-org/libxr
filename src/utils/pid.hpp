#pragma once

#include <algorithm>
#include <cmath>
#include <utility>

#include "cycle_value.hpp"

namespace LibXR
{
using DefaultScalar = LIBXR_DEFAULT_SCALAR;

constexpr DefaultScalar PID_SIGMA = 1e-6f;

/**
 * @brief 通用 PID 控制器类。
 *        Generic PID controller.
 *
 * 支持周期角度处理、积分限幅与输出限幅，适用于基础控制应用。
 * Supports cyclic inputs, integral/output saturation. No derivative filtering.
 *
 * @tparam Scalar 控制器标量类型，默认为 DefaultScalar。
 *                Scalar type for internal calculations.
 */
template <typename Scalar = DefaultScalar>
class PID
{
 public:
  /**
   * @brief PID 参数结构体。
   *        Structure holding PID parameters.
   */
  struct Param
  {
    Scalar k = 1.0;          ///< 全局比例因子 / Global gain
    Scalar p = 0.0;          ///< 比例项 / Proportional gain
    Scalar i = 0.0;          ///< 积分项 / Integral gain
    Scalar d = 0.0;          ///< 微分项 / Derivative gain
    Scalar i_limit = 0.0;    ///< 积分限幅 / Integral limit
    Scalar out_limit = 0.0;  ///< 输出限幅 / Output limit
    bool cycle = false;      ///< 是否处理周期误差 / Whether input is cyclic
  };

  /**
   * @brief 构造 PID 控制器。
   *        Construct a PID controller.
   *
   * @tparam P 用于构造 Param 的参数类型 / Type used to construct Param
   * @param p PID 参数 / PID parameters
   */
  template <typename P>
  PID(P&& p) : param_(std::forward<P>(p))
  {
    Reset();
  }

  /**
   * @brief 使用反馈值计算 PID 输出。
   *        Compute output from feedback only.
   *
   *        Define:
   *        err    = (cycle ? CycleValue(sp) - fb : sp - fb)
   *        e_k    = k * err
   *        fb_d_k = k * (fb - last_fb) / dt
   *
   *        out = p * e_k + i * ∫(e_k)dt - d * fb_d_k + feed_forward
   *
   * @param sp 期望值 / Setpoint
   * @param fb 反馈值 / Feedback
   * @param dt 控制周期（秒） / Time step in seconds
   * @return 控制器输出 / Controller output
   */
  Scalar Calculate(Scalar sp, Scalar fb, Scalar dt)
  {
    if (!std::isfinite(sp) || !std::isfinite(fb) || !std::isfinite(dt) || dt <= Scalar(0))
    {
      return last_out_;
    }

    // Compute error
    const Scalar ERR = param_.cycle ? (CycleValue<Scalar>(sp) - fb) : (sp - fb);
    const Scalar E_K = ERR * param_.k;

    // Derivative from feedback change (scaled by k)
    Scalar fb_dot = (fb - last_fb_) / dt;
    if (!std::isfinite(fb_dot))
    {
      fb_dot = Scalar(0);
    }

    Scalar fb_d_k = fb_dot * param_.k;
    if (!std::isfinite(fb_d_k))
    {
      fb_d_k = Scalar(0);
    }

    // Compute PD
    const Scalar OUTPUT_PD = (E_K * param_.p) - (fb_d_k * param_.d);

    // -------------------------------
    // Integrator update: anti-windup + allow unwind
    // Rule: i_limit == 0 disables I (force i_ = 0)
    // -------------------------------
    if (param_.i > PID_SIGMA && param_.i_limit > PID_SIGMA)
    {
      Scalar i_candidate = i_ + E_K * dt;

      if (std::isfinite(i_candidate))
      {
        // Clamp integrator state
        i_candidate = std::clamp(i_candidate, -param_.i_limit, param_.i_limit);

        bool accept = true;

        // Output-limit-aware gating (windup prevention + unwind)
        if (param_.out_limit > PID_SIGMA)
        {
          const Scalar OUT_BEFORE = OUTPUT_PD + (i_ * param_.i) + feed_forward_;
          const Scalar OUT_AFTER = OUTPUT_PD + (i_candidate * param_.i) + feed_forward_;

          if (std::isfinite(OUT_BEFORE) && std::isfinite(OUT_AFTER))
          {
            const bool BEFORE_SAT = (std::abs(OUT_BEFORE) > param_.out_limit);
            const bool AFTER_SAT = (std::abs(OUT_AFTER) > param_.out_limit);

            if (AFTER_SAT)
            {
              // If saturated (or would be), only allow integral update if it reduces
              // saturation magnitude
              // - If not saturated before but would saturate after: reject (prevent
              // windup)
              // - If already saturated: allow only if |out_after| < |out_before| (unwind)
              accept = BEFORE_SAT && (std::abs(OUT_AFTER) < std::abs(OUT_BEFORE));
            }
          }
          else
          {
            accept = false;
          }
        }

        if (accept)
        {
          i_ = i_candidate;
        }
      }
    }
    else
    {
      // Disable I
      i_ = Scalar(0);
    }

    const Scalar I_OUT = i_ * param_.i;

    // Apply output limits
    Scalar output = OUTPUT_PD + I_OUT + feed_forward_;
    if (std::isfinite(output) && param_.out_limit > PID_SIGMA)
    {
      output = std::clamp(output, -param_.out_limit, param_.out_limit);
    }

    // Store states
    last_err_ = ERR;
    last_fb_ = fb;  // store raw feedback
    last_out_ = output;
    last_der_ = fb_d_k;  // store scaled derivative: k * d(fb)/dt
    return output;
  }

  /**
   * @brief 使用外部导数计算 PID 输出。
   *        Compute output using external feedback derivative.
   *
   *        Define:
   *        err    = (cycle ? CycleValue(sp) - fb : sp - fb)
   *        e_k    = k * err
   *        fb_d_k = k * fb_dot
   *
   *        out = p * e_k + i * ∫(e_k)dt - d * fb_d_k + feed_forward
   *
   * @param sp 期望值 / Setpoint
   * @param fb 反馈值 / Feedback
   * @param fb_dot 反馈导数 / Feedback rate (d(fb)/dt)
   * @param dt 控制周期 / Delta time
   * @return 控制器输出 / Controller output
   */
  Scalar Calculate(Scalar sp, Scalar fb, Scalar fb_dot, Scalar dt)
  {
    if (!std::isfinite(sp) || !std::isfinite(fb) || !std::isfinite(fb_dot) ||
        !std::isfinite(dt) || dt <= Scalar(0))
    {
      return last_out_;
    }

    // Compute error
    const Scalar ERR = param_.cycle ? (CycleValue<Scalar>(sp) - fb) : (sp - fb);
    const Scalar E_K = ERR * param_.k;

    // Use externally provided derivative (scaled by k)
    Scalar fb_d_k = fb_dot * param_.k;
    if (!std::isfinite(fb_d_k))
    {
      fb_d_k = Scalar(0);
    }

    // Compute PD
    const Scalar OUTPUT_PD = (E_K * param_.p) - (fb_d_k * param_.d);

    // -------------------------------
    // Integrator update: anti-windup + allow unwind
    // Rule: i_limit == 0 disables I (force i_ = 0)
    // -------------------------------
    if (param_.i > PID_SIGMA && param_.i_limit > PID_SIGMA)
    {
      Scalar i_candidate = i_ + E_K * dt;

      if (std::isfinite(i_candidate))
      {
        // Clamp integrator state
        i_candidate = std::clamp(i_candidate, -param_.i_limit, param_.i_limit);

        bool accept = true;

        if (param_.out_limit > PID_SIGMA)
        {
          const Scalar OUT_BEFORE = OUTPUT_PD + (i_ * param_.i) + feed_forward_;
          const Scalar OUT_AFTER = OUTPUT_PD + (i_candidate * param_.i) + feed_forward_;

          if (std::isfinite(OUT_BEFORE) && std::isfinite(OUT_AFTER))
          {
            const bool BEFORE_SAT = (std::abs(OUT_BEFORE) > param_.out_limit);
            const bool AFTER_SAT = (std::abs(OUT_AFTER) > param_.out_limit);

            if (AFTER_SAT)
            {
              accept = BEFORE_SAT && (std::abs(OUT_AFTER) < std::abs(OUT_BEFORE));
            }
          }
          else
          {
            accept = false;
          }
        }

        if (accept)
        {
          i_ = i_candidate;
        }
      }
    }
    else
    {
      // Disable I
      i_ = Scalar(0);
    }

    const Scalar I_OUT = i_ * param_.i;

    // Apply output limits
    Scalar output = OUTPUT_PD + I_OUT + feed_forward_;
    if (std::isfinite(output) && param_.out_limit > PID_SIGMA)
    {
      output = std::clamp(output, -param_.out_limit, param_.out_limit);
    }

    // Store states
    last_err_ = ERR;
    last_fb_ = fb;  // store raw feedback
    last_out_ = output;
    last_der_ = fb_d_k;  // store scaled derivative: k * d(fb)/dt
    return output;
  }

  /// 设置全局比例系数 / Set global proportional gain
  void SetK(Scalar k) { param_.k = k; }
  /// 设置 P 项系数 / Set proportional gain
  void SetP(Scalar p) { param_.p = p; }
  /// 设置 I 项系数 / Set integral gain
  void SetI(Scalar i) { param_.i = i; }
  /// 设置 D 项系数 / Set derivative gain
  void SetD(Scalar d) { param_.d = d; }
  /// 设置积分限幅 / Set integral limit
  void SetILimit(Scalar limit) { param_.i_limit = limit; }
  /// 设置输出限幅 / Set output limit
  void SetOutLimit(Scalar limit) { param_.out_limit = limit; }

  /// 获取全局比例系数 / Get global proportional gain
  Scalar K() const { return param_.k; }
  /// 获取 P 项系数 / Get proportional gain
  Scalar P() const { return param_.p; }
  /// 获取 I 项系数 / Get integral gain
  Scalar I() const { return param_.i; }
  /// 获取 D 项系数 / Get derivative gain
  Scalar D() const { return param_.d; }
  /// 获取积分限幅 / Get integral limit
  Scalar ILimit() const { return param_.i_limit; }
  /// 获取输出限幅 / Get output limit
  Scalar OutLimit() const { return param_.out_limit; }
  /// 获取上一次误差 / Get last error
  Scalar LastError() const { return last_err_; }
  /// 获取上一次反馈值（未缩放） / Get last feedback (raw)
  Scalar LastFeedback() const { return last_fb_; }
  /// 获取上一次输出 / Get last output
  Scalar LastOutput() const { return last_out_; }
  /// 获取上一次导数（k * d(fb)/dt 或 k * fb_dot） / Get last derivative (scaled by k)
  Scalar LastDerivative() const { return last_der_; }

  /**
   * @brief 重置控制器状态。
   *        Reset all internal states.
   */
  void Reset()
  {
    i_ = 0;
    last_err_ = 0;
    last_fb_ = 0;
    last_out_ = 0;
    last_der_ = 0;
  }

  /**
   * @brief 设置累计误差 / Set integral error
   *
   * @param err 累计误差 / Integral error
   */
  void SetIntegralError(Scalar err) { i_ = err; }

  /**
   * @brief 获取累计误差 / Get integral error
   *
   * @return 累计误差 / Integral error
   */
  Scalar GetIntegralError() const { return i_; }

  /**
   * @brief 设置前馈项 / Set feedforward
   *
   * @param feed_forward 前馈项 / Feedforward
   */
  void SetFeedForward(Scalar feed_forward) { feed_forward_ = feed_forward; }

  /**
   * @brief 获取前馈项 / Get feedforward
   *
   * @return 前馈项 / Feedforward
   */
  Scalar GetFeedForward() const { return feed_forward_; }

 private:
  Param param_;              ///< PID 参数 / PID parameter set
  Scalar i_ = 0;             ///< 积分状态 / Integral state
  Scalar last_err_ = 0;      ///< 上次误差 / Last error
  Scalar last_fb_ = 0;       ///< 上次反馈（未缩放） / Last feedback (raw)
  Scalar last_der_ = 0;      ///< 上次导数（k 缩放） / Last derivative (scaled by k)
  Scalar last_out_ = 0;      ///< 上次输出 / Last output
  Scalar feed_forward_ = 0;  ///< 前馈项 / Feedforward term
};

}  // namespace LibXR
