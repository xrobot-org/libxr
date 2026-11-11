#pragma once

#include <cstdint>

#include "dac.hpp"
#include "esp_def.hpp"
#include "soc/soc_caps.h"

#if SOC_DAC_SUPPORTED
#include "driver/gpio.h"
#include "esp_private/gpio.h"
#include "hal/dac_ll.h"
#include "soc/dac_periph.h"
#endif

namespace LibXR
{

/**
 * @brief ESP32 内置 DAC 通道 / ESP32 built-in DAC channel
 */
class ESP32DAC : public DAC
{
 public:
  /**
   * @brief 构造 DAC 通道并输出初始电压 / Construct a DAC channel and output the initial
   *        voltage
   * @param channel_id DAC 通道号，须小于 SOC_DAC_CHAN_NUM / DAC channel index, below
   *        SOC_DAC_CHAN_NUM
   * @param init_voltage 初始输出电压（V）/ Initial output voltage (V)
   * @param reference_voltage 满量程对应的电压（V），大于 0 / Voltage (V) at full scale, >
   * 0
   * @note 参数非法或芯片无 DAC 时对象保持未就绪，Write() 返回 NOT_SUPPORT。
   *       With invalid arguments or no DAC on the chip the object stays not ready and
   *       Write() returns NOT_SUPPORT.
   */
  explicit ESP32DAC(uint8_t channel_id = 0, float init_voltage = 0.0f,
                    float reference_voltage = 3.3f)
      : channel_id_(channel_id), reference_voltage_(reference_voltage)
  {
#if SOC_DAC_SUPPORTED
    if ((channel_id_ >= SOC_DAC_CHAN_NUM) || (reference_voltage_ <= 0.0f))
    {
      return;
    }

    ready_ = true;
    dac_ll_power_on(ToChannel(channel_id_));
    dac_ll_rtc_sync_by_adc(false);
    (void)gpio_config_as_analog(
        static_cast<gpio_num_t>(dac_periph_signal.dac_channel_io_num[channel_id_]));
    (void)Write(init_voltage);
#else
    (void)init_voltage;
#endif
  }

  ErrorCode Write(float voltage) override
  {
#if SOC_DAC_SUPPORTED
    if (!ready_)
    {
      return ErrorCode::NOT_SUPPORT;
    }

    if (voltage < 0.0f)
    {
      voltage = 0.0f;
    }
    else if (voltage > reference_voltage_)
    {
      voltage = reference_voltage_;
    }

    constexpr uint32_t MAX_CODE = (1U << SOC_DAC_RESOLUTION) - 1U;
    const float scale = static_cast<float>(MAX_CODE) / reference_voltage_;
    uint32_t code = static_cast<uint32_t>((voltage * scale) + 0.5f);
    if (code > MAX_CODE)
    {
      code = MAX_CODE;
    }

    dac_ll_update_output_value(ToChannel(channel_id_), static_cast<uint8_t>(code));
    return ErrorCode::OK;
#else
    (void)voltage;
    return ErrorCode::NOT_SUPPORT;
#endif
  }

 private:
#if SOC_DAC_SUPPORTED
  static dac_channel_t ToChannel(uint8_t channel_id)
  {
    return channel_id == 0 ? DAC_CHAN_0 : DAC_CHAN_1;
  }
#endif

  uint8_t channel_id_;
  float reference_voltage_;
  bool ready_ = false;
};

}  // namespace LibXR
