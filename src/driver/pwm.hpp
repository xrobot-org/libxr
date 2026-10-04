#pragma once

#include "libxr.hpp"

namespace LibXR
{

/**
 * @class PWM
 * @brief PWM（脉冲宽度调制）控制的抽象基类。
 * Abstract base class for PWM (Pulse Width Modulation) control.
 */
class PWM
{
 public:
  PWM() = default;

  /**
   * @struct Configuration
   * @brief PWM 配置参数 / Configuration parameters for PWM.
   */
  typedef struct
  {
    uint32_t frequency;  ///< PWM 信号的频率（Hz） / PWM signal frequency in Hz.
  } Configuration;

  /**
   * @brief 设置 PWM 信号的占空比 / Sets the duty cycle of the PWM signal.
   * @param value 占空比，浮点值（0.0 到 1.0）。
   * The duty cycle as a floating-point value (0.0 to 1.0).
   * @return 返回操作结果的错误码 / ErrorCode indicating success or failure.
   */
  virtual ErrorCode SetDutyCycle(float value) = 0;

  /**
   * @brief 配置 PWM 参数 / Configures the PWM settings.
   * @param config 配置结构体，包含 PWM 设置。
   * The configuration structure containing PWM settings.
   * @return 返回操作结果的错误码 / ErrorCode indicating success or failure.
   */
  virtual ErrorCode SetConfig(Configuration config) = 0;

  /**
   * @brief 启用 PWM 输出 / Enables the PWM output.
   * @return 返回操作结果的错误码 / ErrorCode indicating success or failure.
   */
  virtual ErrorCode Enable() = 0;

  /**
   * @brief 禁用 PWM 输出 / Disables the PWM output.
   * @return 返回操作结果的错误码 / ErrorCode indicating success or failure.
   */
  virtual ErrorCode Disable() = 0;
};

}  // namespace LibXR
