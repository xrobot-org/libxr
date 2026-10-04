#pragma once

#include "libxr.hpp"

namespace LibXR
{

/**
 * @class ADC
 * @brief 模拟数字转换器（ADC）基类
 *        Abstract base class for Analog-to-Digital Converter (ADC)
 *
 * 该类定义了 ADC 设备的基本接口，所有 ADC 设备应继承此类并实现 `Read` 方法。
 * This class defines the basic interface for an ADC device. All ADC devices should
 * inherit from this class and implement the `Read` method.
 */
class ADC
{
 public:
  /**
   * @brief 默认构造函数 / Default constructor
   */
  ADC() = default;

  /**
   * @brief 虚析构函数 / Virtual destructor
   */
  virtual ~ADC() = default;

  /**
   * @brief 读取 ADC 值 / Reads the ADC value
   * @return 读取的模拟电压值（范围通常由硬件决定，例如 0-3.3V）
   *         The read analog voltage value (range typically determined by hardware, e.g.,
   *         0-3.3V)
   *
   * 该方法为纯虚函数，子类必须实现此方法以提供具体的 ADC 读取功能。
   * This is a pure virtual function. Subclasses must implement this method to provide
   * specific ADC reading functionality.
   */
  virtual float Read() = 0;
};

}  // namespace LibXR
