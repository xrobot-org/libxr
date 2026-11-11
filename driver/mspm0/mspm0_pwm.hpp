#pragma once

#include "pwm.hpp"
#include "ti_msp_dl_config.h"  // IWYU pragma: keep; required for GPTIMER_Regs and DL_TIMER_CC_INDEX

namespace LibXR
{

/**
 * @brief MSPM0 通用定时器 PWM 输出 / MSPM0 general-purpose timer PWM output
 */
class MSPM0PWM : public PWM
{
 public:
  /// 一个 PWM 通道的硬件资源描述 / Hardware resources of one PWM channel
  struct Resources
  {
    GPTIMER_Regs* timer;        ///< 定时器寄存器 / Timer registers
    DL_TIMER_CC_INDEX channel;  ///< 捕获比较通道 / Capture/compare channel
    uint32_t clock_freq;  ///< 分频前的定时器时钟（Hz，必须大于 0）/ Timer clock before
                          ///< the dividers (Hz, > 0)
  };

  /**
   * @brief 构造 PWM 通道 / Construct a PWM channel
   * @param res 硬件资源 / Hardware resources
   */
  MSPM0PWM(Resources res);

  /**
   * @brief 由 SysConfig 的定时器时钟反推源时钟 / Source clock from SysConfig's timer
   *        clock.
   * @param timer 定时器实例 / Timer instance.
   * @param timer_clock_hz SysConfig 的 NAME_INST_CLK_FREQ（分频后）/ SysConfig's
   *        NAME_INST_CLK_FREQ (after the dividers).
   * @return 分频前的定时器时钟 / Timer clock before the dividers.
   */
  static uint32_t SourceClockFromSysCfg(GPTIMER_Regs* timer, uint32_t timer_clock_hz);

  ErrorCode SetDutyCycle(float value);

  ErrorCode SetConfig(Configuration config);

  ErrorCode Enable();

  ErrorCode Disable();

 private:
  GPTIMER_Regs* timer_;
  DL_TIMER_CC_INDEX channel_;
  uint32_t clock_freq_;
};

/**
 * @brief 由 SysConfig 宏生成 MSPM0PWM 资源 / MSPM0PWM resources from SysConfig macros.
 * @note 须在 SYSCFG_DL_init() 之后求值：源时钟由 NAME_INST_CLK_FREQ 和 SysConfig
 *       写入定时器的分频值求出。
 *       Evaluate after SYSCFG_DL_init(): the source clock is derived from
 *       NAME_INST_CLK_FREQ and the divider values SysConfig wrote to the timer.
 */
#define MSPM0_PWM_CH(name, ch)                                                      \
  ::LibXR::MSPM0PWM::Resources                                                      \
  {                                                                                 \
    name##_INST, DL_TIMER_CC_##ch##_INDEX,                                          \
        ::LibXR::MSPM0PWM::SourceClockFromSysCfg(name##_INST, name##_INST_CLK_FREQ) \
  }

}  // namespace LibXR