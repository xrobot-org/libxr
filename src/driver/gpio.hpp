#pragma once

#include "libxr.hpp"

namespace LibXR
{

/**
 * @class GPIO
 * @brief 通用输入输出（GPIO）接口类。
 *        General Purpose Input/Output (GPIO) interface class.
 */
class GPIO
{
 public:
  /**
   * @enum Direction
   * @brief 定义 GPIO 引脚的方向类型 / Defines the direction types for GPIO pins.
   */
  enum class Direction : uint8_t
  {
    INPUT,                 ///< 输入模式 / Input mode.
    OUTPUT_PUSH_PULL,      ///< 推挽输出模式 / Push-pull output mode.
    OUTPUT_OPEN_DRAIN,     ///< 开漏输出模式 / Open-drain output mode.
    FALL_INTERRUPT,        ///< 下降沿中断模式 / Falling edge interrupt mode.
    RISING_INTERRUPT,      ///< 上升沿中断模式 / Rising edge interrupt mode.
    FALL_RISING_INTERRUPT  ///< 双沿触发中断模式 / Both edge interrupt mode.
  };

  /**
   * @enum Pull
   * @brief 定义 GPIO 引脚的上拉/下拉模式。
   * Defines the pull-up/pull-down configurations for GPIO pins.
   */
  enum class Pull : uint8_t
  {
    NONE,  ///< 无上拉或下拉 / No pull-up or pull-down.
    UP,    ///< 上拉模式 / Pull-up mode.
    DOWN   ///< 下拉模式 / Pull-down mode.
  };

  /**
   * @struct Configuration
   * @brief 存储 GPIO 配置参数的结构体 / Structure storing GPIO configuration parameters.
   */
  struct Configuration
  {
    Direction direction;  ///< GPIO 引脚方向 / GPIO pin direction.
    Pull pull;            ///< GPIO 上拉/下拉配置 / GPIO pull-up/pull-down configuration.
  };

  using Callback = LibXR::Callback<>;

  /**
   * @brief GPIO 事件的回调函数 / Callback function for GPIO events.
   */
  Callback callback_;

  /**
   * @brief 默认构造函数 / Default constructor.
   */
  GPIO() {}

  /**
   * @brief 虚析构函数 / Virtual destructor.
   */
  virtual ~GPIO() = default;

  /**
   * @brief 读取 GPIO 引脚状态 / Reads the GPIO pin state.
   * @return 返回引脚状态，true 表示高电平，false 表示低电平。
   * Returns the pin state, true for high, false for low.
   */
  virtual bool Read() = 0;

  /**
   * @brief 写入 GPIO 引脚状态 / Writes the GPIO pin state.
   * @param value 要写入的状态，true 表示高电平，false 表示低电平。
   * The value to write, true for high, false for low.
   */
  virtual void Write(bool value) = 0;

  /**
   * @brief 使能 GPIO 引脚中断 / Enables the GPIO pin interrupt.
   * @return 操作结果的错误码 / Error code indicating the result of the operation.
   */
  virtual ErrorCode EnableInterrupt() = 0;

  /**
   * @brief 禁用 GPIO 引脚中断 / Disables the GPIO pin interrupt.
   * @return 操作结果的错误码 / Error code indicating the result of the operation.
   */
  virtual ErrorCode DisableInterrupt() = 0;

  /**
   * @brief 配置 GPIO 引脚参数 / Configures the GPIO pin settings.
   * @param config 需要应用的 GPIO 配置 / The GPIO configuration to apply.
   * @return 操作结果的错误码 / Error code indicating the result of the operation.
   */
  virtual ErrorCode SetConfig(Configuration config) = 0;

  /**
   * @brief 注册 GPIO 事件回调函数 / Registers a callback function for GPIO events.
   * @param callback 要注册的回调函数 / The callback function to register.
   * @return 操作结果的错误码 / Error code indicating the result of the operation.
   */
  ErrorCode RegisterCallback(Callback callback)
  {
    callback_ = callback;
    return ErrorCode::OK;
  }
};

}  // namespace LibXR
