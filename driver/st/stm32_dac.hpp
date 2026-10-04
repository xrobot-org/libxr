#include "main.h"

#if defined(HAL_DAC_MODULE_ENABLED)

#ifdef DAC
#undef DAC
#endif

#include "dac.hpp"

namespace LibXR
{

/**
 * @brief STM32 DAC 驱动实现 / STM32 DAC driver implementation
 */
class STM32DAC : public DAC
{
 public:
  /**
   * @brief 构造 DAC 对象 / Construct DAC object
   */
  STM32DAC(DAC_HandleTypeDef* hdac, uint32_t channel, float init_voltage = 0.0f,
           float vref = 3.3f);

  /**
   * @brief 输出电压 / Output analog voltage
   * @param voltage 模拟电压值 / Analog voltage value
   *
   * @return ErrorCode 错误码 / Error code
   */
  ErrorCode Write(float voltage) override;

 private:
  DAC_HandleTypeDef* hdac_;  ///< DAC 外设句柄 / DAC device handle
  uint32_t channel_;         ///< DAC 通道 / DAC channel
  float vref_;               ///< DAC 参考电压 / DAC reference voltage
  uint32_t align_;           ///< DAC 对齐方式 / DAC alignment
  uint16_t resolution_;      ///< DAC 分辨率 / DAC resolution
};

}  // namespace LibXR

#endif
