#pragma once

#include "gpio.hpp"
#include "libxr.hpp"

#include DEF2STR(LIBXR_CH32_CONFIG_FILE)

namespace LibXR
{

/**
 * @brief CH32 GPIO 端口分组索引 / CH32 GPIO port group index
 */
typedef enum
{
#if defined(GPIOA)
  CH32_GPIOA,  ///< GPIOA 端口 / GPIOA port.
#endif
#if defined(GPIOB)
  CH32_GPIOB,  ///< GPIOB 端口 / GPIOB port.
#endif
#if defined(GPIOC)
  CH32_GPIOC,  ///< GPIOC 端口 / GPIOC port.
#endif
#if defined(GPIOD)
  CH32_GPIOD,  ///< GPIOD 端口 / GPIOD port.
#endif
#if defined(GPIOE)
  CH32_GPIOE,  ///< GPIOE 端口 / GPIOE port.
#endif
#if defined(GPIOF)
  CH32_GPIOF,  ///< GPIOF 端口 / GPIOF port.
#endif
#if defined(GPIOG)
  CH32_GPIOG,  ///< GPIOG 端口 / GPIOG port.
#endif
#if defined(GPIOH)
  CH32_GPIOH,  ///< GPIOH 端口 / GPIOH port.
#endif
#if defined(GPIOI)
  CH32_GPIOI,  ///< GPIOI 端口 / GPIOI port.
#endif
  CH32_GPIO_NUMBER  ///< 可用 GPIO 端口数量 / Number of available GPIO ports.
} ch32_gpio_group_t;

/**
 * @brief 根据 GPIO 端口指针获取对应的 RCC APB2 外设时钟掩码
 *        Get the RCC APB2 peripheral clock mask for a GPIO port.
 * @param port GPIO 端口寄存器指针 / Pointer to the GPIO port registers.
 * @return RCC 外设时钟位掩码 / RCC peripheral clock bitmask.
 */
uint32_t ch32_get_gpio_periph(GPIO_TypeDef* port);

/**
 * @brief CH32 GPIO 驱动实现 / CH32 GPIO driver implementation
 */
class CH32GPIO final : public GPIO
{
 public:
  /**
   * @brief 构造 GPIO 对象 / Construct GPIO object
   */
  CH32GPIO(GPIO_TypeDef* port, uint16_t pin,
           GPIO::Direction direction = GPIO::Direction::OUTPUT_PUSH_PULL,
           GPIO::Pull pull = GPIO::Pull::NONE, IRQn_Type irq = NonMaskableInt_IRQn);

  inline bool Read() override { return (port_->INDR & pin_) != (uint32_t)Bit_RESET; }

  inline void Write(bool value) override
  {
    if (value)
    {
      port_->BSHR = pin_;
    }
    else
    {
      port_->BCR = pin_;
    }
  }

  ErrorCode EnableInterrupt() override;

  ErrorCode DisableInterrupt() override;

  inline ErrorCode SetConfig(Configuration config) override
  {
    GPIO_InitTypeDef gpio_init = {};
    gpio_init.GPIO_Pin = pin_;
    gpio_init.GPIO_Speed = GPIO_Speed_50MHz;

    switch (config.direction)
    {
      case Direction::INPUT:
      case Direction::RISING_INTERRUPT:
      case Direction::FALL_INTERRUPT:
      case Direction::FALL_RISING_INTERRUPT:
        gpio_init.GPIO_Mode = (config.pull == Pull::UP)     ? GPIO_Mode_IPU
                              : (config.pull == Pull::DOWN) ? GPIO_Mode_IPD
                                                            : GPIO_Mode_IN_FLOATING;
        break;

      case Direction::OUTPUT_PUSH_PULL:
        gpio_init.GPIO_Mode = GPIO_Mode_Out_PP;
        break;

      case Direction::OUTPUT_OPEN_DRAIN:
        gpio_init.GPIO_Mode = GPIO_Mode_Out_OD;
        break;
    }

    GPIO_Init(port_, &gpio_init);

    switch (config.direction)
    {
      case Direction::RISING_INTERRUPT:
        ConfigureEXTI(EXTI_Trigger_Rising);
        break;
      case Direction::FALL_INTERRUPT:
        ConfigureEXTI(EXTI_Trigger_Falling);
        break;
      case Direction::FALL_RISING_INTERRUPT:
        ConfigureEXTI(EXTI_Trigger_Rising_Falling);
        break;
      default:
        break;
    }

    return ErrorCode::OK;
  }

  /**
   * @brief 在外部中断触发时调用当前实例的回调 / Invoke the instance callback on EXTI
   *        interrupt.
   */
  void OnInterrupt();

  /**
   * @brief 根据 EXTI 线路号检查并分发 GPIO 中断 / Check and dispatch GPIO interrupt by
   *        EXTI line index.
   * @param line EXTI 线路号（`0..15`） / EXTI line index (`0..15`).
   */
  static void CheckInterrupt(uint32_t line);

  /// EXTI 线路映射表 / EXTI line map
  static inline CH32GPIO* map_[16] = {nullptr};

 private:
  GPIO_TypeDef* port_;
  uint16_t pin_;
  IRQn_Type irq_;

  void ConfigureEXTI(EXTITrigger_TypeDef trigger);

  static uint8_t GetEXTIID(uint16_t pin);
};

}  // namespace LibXR
