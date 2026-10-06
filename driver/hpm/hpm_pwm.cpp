#include "hpm_pwm.hpp"

using namespace LibXR;

uint8_t HPMPWM::ResolveGptmrReloadCmpIndex(uint8_t duty_cmp_index)
{
  if (duty_cmp_index >= GPTMR_CH_CMP_COUNT)
  {
    return INVALID_CMP_INDEX;
  }
  return static_cast<uint8_t>(duty_cmp_index == 0u ? 1u : 0u);
}

/**
 * @brief 构造 PWM 实例 / Construct PWM instance.
 *
 * 说明：
 * - 当芯片存在原生 PWM 外设时，`pwm_` 按原生路径使用；
 * - 当芯片仅有 GPTMR 输出比较能力时，`gptmr_` 走 fallback 路径。
 * Notes:
 * - If native PWM exists, `pwm_` is used in native path.
 * - If only GPTMR compare output is available, `gptmr_` is used as fallback.
 */
HPMPWM::HPMPWM(LibXRHpmPwmType* pwm, clock_name_t clock, uint8_t pwm_index,
               uint8_t cmp_index, Polarity polarity)
    : pwm_(pwm),
      gptmr_(reinterpret_cast<GPTMR_Type*>(pwm)),
      clock_(clock),
      pwm_index_(pwm_index),
      cmp_index_(cmp_index),
      invert_(polarity == Polarity::INVERTED),
      reload_(0),
      configured_(false)
{
}

ErrorCode HPMPWM::SetDutyCycle(float value)
{
  if (!configured_)
  {
    return ErrorCode::INIT_ERR;
  }

  if (value < 0.0f)
  {
    value = 0.0f;
  }
  else if (value > 1.0f)
  {
    value = 1.0f;
  }

#if LIBXR_HPM_PWM_SUPPORTED
  const float duty_percent = value * 100.0f;
  if (pwm_update_duty_edge_aligned(pwm_, cmp_index_, duty_percent) != status_success)
  {
    return ErrorCode::FAILED;
  }
#elif LIBXR_HPM_GPTMR_PWM_FALLBACK
  // GPTMR 比较值不允许落在 0 或 reload，避免输出退化成常高/常低。
  // Keep compare away from 0/reload to avoid constant-high/constant-low output.
  uint32_t cmp = static_cast<uint32_t>(static_cast<float>(reload_) * value);
  if (cmp == 0u)
  {
    cmp = 1u;
  }
  else if (cmp >= reload_)
  {
    cmp = reload_ - 1u;
  }
  gptmr_update_cmp(gptmr_, pwm_index_, cmp_index_, cmp);
#else
  (void)value;
  return ErrorCode::NOT_SUPPORT;
#endif

  return ErrorCode::OK;
}

ErrorCode HPMPWM::SetConfig(Configuration config)
{
  if (config.frequency == 0u)
  {
    return ErrorCode::ARG_ERR;
  }

  const uint32_t clock_hz = clock_get_frequency(clock_);

  if (clock_hz == 0u)
  {
    return ErrorCode::INIT_ERR;
  }

  uint32_t reload = clock_hz / config.frequency;
  if (reload == 0u || reload > 0xFFFFFFu)
  {
    return ErrorCode::INIT_ERR;
  }
  reload_ = reload;

#if LIBXR_HPM_PWM_SUPPORTED
  pwm_stop_counter(pwm_);

  pwm_config_t pwm_config{};
  pwm_cmp_config_t cmp_config{};

  pwm_get_default_pwm_config(pwm_, &pwm_config);
  pwm_get_default_cmp_config(pwm_, &cmp_config);

  pwm_config.enable_output = true;
  pwm_config.invert_output = invert_;
  pwm_config.dead_zone_in_half_cycle = 0;

  pwm_set_reload(pwm_, 0, reload_);
  pwm_set_start_count(pwm_, 0, 0);

  cmp_config.mode = pwm_cmp_mode_output_compare;
  cmp_config.cmp = reload_ + 1;
  cmp_config.update_trigger = pwm_shadow_register_update_on_modify;

  if (pwm_setup_waveform(pwm_, pwm_index_, &pwm_config, cmp_index_, &cmp_config, 1) !=
      status_success)
  {
    return ErrorCode::INIT_ERR;
  }

  pwm_issue_shadow_register_lock_event(pwm_);
#elif LIBXR_HPM_GPTMR_PWM_FALLBACK
  if (gptmr_ == nullptr)
  {
    return ErrorCode::ARG_ERR;
  }

  const uint8_t reload_cmp_index = ResolveGptmrReloadCmpIndex(cmp_index_);
  if (reload_cmp_index == INVALID_CMP_INDEX)
  {
    return ErrorCode::ARG_ERR;
  }

  // 重新配置前先停计数器，避免切换参数时输出毛刺。
  // Stop counter before reconfiguration to avoid output glitches.
  gptmr_stop_counter(gptmr_, pwm_index_);

  gptmr_channel_config_t cfg;
  gptmr_channel_get_default_config(gptmr_, &cfg);
  cfg.mode = gptmr_work_mode_no_capture;
  cfg.cmp_initial_polarity_high = invert_;
  cfg.enable_cmp_output = true;
  cfg.reload = reload_;
  cfg.cmp[cmp_index_] = reload_ / 2u;
  cfg.cmp[reload_cmp_index] = reload_;

  if (gptmr_channel_config(gptmr_, pwm_index_, &cfg, false) != status_success)
  {
    return ErrorCode::INIT_ERR;
  }
  gptmr_channel_reset_count(gptmr_, pwm_index_);
#else
  return ErrorCode::NOT_SUPPORT;
#endif

  configured_ = true;
  return ErrorCode::OK;
}

ErrorCode HPMPWM::Enable()
{
  if (!configured_)
  {
    return ErrorCode::INIT_ERR;
  }

#if LIBXR_HPM_PWM_SUPPORTED
  pwm_start_counter(pwm_);
#elif LIBXR_HPM_GPTMR_PWM_FALLBACK
  gptmr_start_counter(gptmr_, pwm_index_);
#else
  return ErrorCode::NOT_SUPPORT;
#endif

  return ErrorCode::OK;
}

ErrorCode HPMPWM::Disable()
{
#if LIBXR_HPM_PWM_SUPPORTED
  pwm_stop_counter(pwm_);
#elif LIBXR_HPM_GPTMR_PWM_FALLBACK
  gptmr_stop_counter(gptmr_, pwm_index_);
#else
  return ErrorCode::NOT_SUPPORT;
#endif
  return ErrorCode::OK;
}
