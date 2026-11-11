#pragma once

#include "gpio.hpp"
#include "libxr.hpp"

#include DEF2STR(LIBXR_CH32_CONFIG_FILE)

namespace LibXR
{

/// GPIO 端口编号，按芯片实际具备的端口顺序排列，CH32_GPIO_NUMBER 为端口总数 / GPIO port
/// index in the order of the ports the chip has; CH32_GPIO_NUMBER is the port count
typedef enum
{
#if defined(GPIOA)
  CH32_GPIOA,
#endif
#if defined(GPIOB)
  CH32_GPIOB,
#endif
#if defined(GPIOC)
  CH32_GPIOC,
#endif
#if defined(GPIOD)
  CH32_GPIOD,
#endif
#if defined(GPIOE)
  CH32_GPIOE,
#endif
#if defined(GPIOF)
  CH32_GPIOF,
#endif
#if defined(GPIOG)
  CH32_GPIOG,
#endif
#if defined(GPIOH)
  CH32_GPIOH,
#endif
#if defined(GPIOI)
  CH32_GPIOI,
#endif
  CH32_GPIO_NUMBER
} ch32_gpio_group_t;

/**
 * @brief 获取 GPIO 端口对应的 APB2 外设时钟位 / Get the APB2 peripheral clock bit of a
 * GPIO port
 * @param port GPIO 端口寄存器指针 / GPIO port register pointer
 * @return RCC_APB2Periph_GPIOx 时钟位（仅识别 GPIOA..GPIOE），未识别的端口返回 0 /
 * The RCC_APB2Periph_GPIOx bit (only GPIOA..GPIOE are recognized); 0 for other ports
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

  /// 在中断上下文中执行已注册的回调（回调为空则不执行）/ Run the registered callback in
  /// interrupt context (nothing if no callback is set)
  void OnInterrupt();

  /**
   * @brief 检查并处理一条 EXTI 线的挂起中断 / Check and handle the pending interrupt of
   * one EXTI line
   * @param line EXTI 线路号（EXTI_LineX）/ EXTI line (EXTI_LineX)
   * @note 若该线挂起则先清除挂起位，再调用 map_ 中对应 GPIO 的 OnInterrupt()；由 EXTI
   *       中断服务函数调用。 / If the line is pending, clears the pending bit and calls
   *       OnInterrupt() of the GPIO in map_; called from the EXTI ISRs.
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
