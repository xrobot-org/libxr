#pragma once

#include <cstdint>

#include "driver/gpio.h"
#include "esp_def.hpp"
#include "gpio.hpp"
#include "hal/gpio_hal.h"

static inline gpio_dev_t* LibXREspGpioHw() { return GPIO_HAL_GET_HW(0); }

namespace LibXR
{
/**
 * @brief ESP32 GPIO 驱动实现 / ESP32 GPIO driver implementation
 */
class ESP32GPIO : public GPIO
{
 public:
  /**
   * @brief 构造 GPIO 对象 / Construct GPIO object
   * @param gpio_num GPIO 编号 / GPIO number
   */
  explicit ESP32GPIO(gpio_num_t gpio_num) : gpio_num_(gpio_num)
  {
    const bool valid = (gpio_num_ >= 0) && (gpio_num_ < GPIO_NUM_MAX);
    ASSERT(valid);
    if (valid)
    {
      map_[gpio_num_] = this;
    }
  }

  bool Read() override
  {
    const bool valid = (gpio_num_ >= 0) && (gpio_num_ < GPIO_NUM_MAX);
    ASSERT(valid);
    if (!valid)
    {
      return false;
    }

    gpio_hal_context_t hal = {.dev = LibXREspGpioHw()};
    return gpio_hal_get_level(&hal, gpio_num_) != 0;
  }

  void Write(bool value) override
  {
    const bool valid = (gpio_num_ >= 0) && (gpio_num_ < GPIO_NUM_MAX);
    ASSERT(valid);
    if (!valid)
    {
      return;
    }

    gpio_hal_context_t hal = {.dev = LibXREspGpioHw()};
    gpio_hal_set_level(&hal, gpio_num_, value ? 1 : 0);
  }

  ErrorCode EnableInterrupt() override
  {
    if (!GPIO_IS_VALID_GPIO(gpio_num_))
    {
      return ErrorCode::ARG_ERR;
    }

    if (!isr_service_installed_)
    {
      if (gpio_install_isr_service(0) != ESP_OK)
      {
        return ErrorCode::INIT_ERR;
      }
      isr_service_installed_ = true;
    }

    if (!isr_handler_added_)
    {
      if (gpio_isr_handler_add(
              gpio_num_, ESP32GPIO::InterruptDispatcher,
              reinterpret_cast<void*>(static_cast<uintptr_t>(gpio_num_))) != ESP_OK)
      {
        return ErrorCode::INIT_ERR;
      }
      isr_handler_added_ = true;
    }

    if (gpio_intr_enable(gpio_num_) != ESP_OK)
    {
      return ErrorCode::FAILED;
    }
    return ErrorCode::OK;
  }

  ErrorCode DisableInterrupt() override
  {
    if (!GPIO_IS_VALID_GPIO(gpio_num_))
    {
      return ErrorCode::ARG_ERR;
    }

    if (!isr_handler_added_)
    {
      return ErrorCode::OK;
    }

    if (gpio_intr_disable(gpio_num_) != ESP_OK)
    {
      return ErrorCode::FAILED;
    }
    return ErrorCode::OK;
  }

  ErrorCode SetConfig(Configuration config) override
  {
    if (!GPIO_IS_VALID_GPIO(gpio_num_))
    {
      return ErrorCode::ARG_ERR;
    }

    gpio_hal_context_t hal = {.dev = LibXREspGpioHw()};

    // Align with ST/CH reconfigure semantics: no full reset, only re-apply mode fields.
    gpio_hal_set_output_enable_ctrl(&hal, gpio_num_, false, false);
    gpio_hal_func_sel(&hal, gpio_num_, PIN_FUNC_GPIO);

    gpio_hal_pullup_dis(&hal, gpio_num_);
    gpio_hal_pulldown_dis(&hal, gpio_num_);
    gpio_hal_od_disable(&hal, gpio_num_);
    gpio_hal_set_intr_type(&hal, gpio_num_, GPIO_INTR_DISABLE);

    switch (config.pull)
    {
      case Pull::NONE:
        break;
      case Pull::UP:
        gpio_hal_pullup_en(&hal, gpio_num_);
        break;
      case Pull::DOWN:
        gpio_hal_pulldown_en(&hal, gpio_num_);
        break;
    }

    switch (config.direction)
    {
      case Direction::INPUT:
        gpio_hal_input_enable(&hal, gpio_num_);
        gpio_hal_output_disable(&hal, gpio_num_);
        break;
      case Direction::OUTPUT_PUSH_PULL:
        gpio_hal_input_disable(&hal, gpio_num_);
        gpio_hal_matrix_out_default(&hal, gpio_num_);
        gpio_hal_output_enable(&hal, gpio_num_);
        break;
      case Direction::OUTPUT_OPEN_DRAIN:
        gpio_hal_input_enable(&hal, gpio_num_);
        gpio_hal_matrix_out_default(&hal, gpio_num_);
        gpio_hal_output_enable(&hal, gpio_num_);
        gpio_hal_od_enable(&hal, gpio_num_);
        break;
      case Direction::FALL_INTERRUPT:
        gpio_hal_input_enable(&hal, gpio_num_);
        gpio_hal_output_disable(&hal, gpio_num_);
        gpio_hal_set_intr_type(&hal, gpio_num_, GPIO_INTR_NEGEDGE);
        break;
      case Direction::RISING_INTERRUPT:
        gpio_hal_input_enable(&hal, gpio_num_);
        gpio_hal_output_disable(&hal, gpio_num_);
        gpio_hal_set_intr_type(&hal, gpio_num_, GPIO_INTR_POSEDGE);
        break;
      case Direction::FALL_RISING_INTERRUPT:
        gpio_hal_input_enable(&hal, gpio_num_);
        gpio_hal_output_disable(&hal, gpio_num_);
        gpio_hal_set_intr_type(&hal, gpio_num_, GPIO_INTR_ANYEDGE);
        break;
    }

    return ErrorCode::OK;
  }

  static void InterruptDispatcher(void* arg);

 private:
  gpio_num_t gpio_num_;
  bool isr_handler_added_ = false;
  static inline bool isr_service_installed_ = false;
  static inline ESP32GPIO* map_[GPIO_NUM_MAX];
};

}  // namespace LibXR
