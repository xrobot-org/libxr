#pragma once

#include "libxr.hpp"

namespace LibXR
{

/**
 * @class DAC
 * @brief 数字模拟转换器（DAC）基类
 *        Abstract base class for Digital-to-Analog Converter (DAC)
 *
 * 该类定义了 DAC 设备的基本接口，所有 DAC 设备应继承此类并实现 `Write` 方法。
 * This class defines the basic interface for a DAC device. All DAC devices should
 * inherit from this class and implement the `Write` method.
 */
class DAC
{
 public:
  /**
   * @brief 默认构造函数 / Default constructor
   */
  DAC() = default;

  /**
   * @brief 输出 DAC 电压 / Outputs the DAC voltage
   * @param voltage 需要输出的模拟电压值 / The analog voltage value to be output
   *
   * @return 错误码 / ErrorCode
   *
   * 该方法为纯虚函数，子类必须实现此方法以提供具体的 DAC 输出功能。
   * This is a pure virtual function. Subclasses must implement this method to provide
   * specific DAC output functionality.
   */
  virtual ErrorCode Write(float voltage) = 0;
};

}  // namespace LibXR
