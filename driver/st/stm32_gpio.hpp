#pragma once

#include "gpio.hpp"
#include "main.h"

#ifdef HAL_GPIO_MODULE_ENABLED

#if (defined(GPIO_CRL_MODE0) || defined(GPIO_CRL_MODE0_Msk)) && \
    (defined(GPIO_CRL_CNF0) || defined(GPIO_CRL_CNF0_Msk))
#define XR_STM32_GPIO_HAS_F1_LAYOUT 1
#else
#define XR_STM32_GPIO_HAS_F1_LAYOUT 0
#endif

#if defined(GPIO_MODER_MODE0) || defined(GPIO_MODER_MODE0_Msk) || \
    defined(GPIO_MODER_MODER0) || defined(GPIO_MODER_MODER0_Msk)
#define XR_STM32_GPIO_HAS_MODER_FIELD 1
#else
#define XR_STM32_GPIO_HAS_MODER_FIELD 0
#endif

#if defined(GPIO_OTYPER_OT0) || defined(GPIO_OTYPER_OT0_Msk) || \
    defined(GPIO_OTYPER_OT_0) || defined(GPIO_OTYPER_OT_0_Msk)
#define XR_STM32_GPIO_HAS_OTYPER_FIELD 1
#else
#define XR_STM32_GPIO_HAS_OTYPER_FIELD 0
#endif

#if defined(GPIO_PUPDR_PUPD0) || defined(GPIO_PUPDR_PUPD0_Msk) || \
    defined(GPIO_PUPDR_PUPDR0) || defined(GPIO_PUPDR_PUPDR0_Msk)
#define XR_STM32_GPIO_HAS_PUPDR_FIELD 1
#else
#define XR_STM32_GPIO_HAS_PUPDR_FIELD 0
#endif

#if XR_STM32_GPIO_HAS_MODER_FIELD && XR_STM32_GPIO_HAS_OTYPER_FIELD && \
    XR_STM32_GPIO_HAS_PUPDR_FIELD
#define XR_STM32_GPIO_HAS_MODER_LAYOUT 1
#else
#define XR_STM32_GPIO_HAS_MODER_LAYOUT 0
#endif

#if defined(GPIO_OSPEEDR_OSPEED0) || defined(GPIO_OSPEEDR_OSPEED0_Msk) ||   \
    defined(GPIO_OSPEEDR_OSPEEDR0) || defined(GPIO_OSPEEDR_OSPEEDR0_Msk) || \
    defined(GPIO_OSPEEDER_OSPEEDR0) || defined(GPIO_OSPEEDER_OSPEEDR0_Msk)
#define XR_STM32_GPIO_HAS_OSPEEDR 1
#else
#define XR_STM32_GPIO_HAS_OSPEEDR 0
#endif

#if XR_STM32_GPIO_HAS_F1_LAYOUT && XR_STM32_GPIO_HAS_MODER_LAYOUT
#error \
    "Ambiguous STM32 GPIO layout detection: both F1 and MODER layout signatures are present."
#endif

namespace LibXR
{
/**
 * @brief STM32 GPIO 驱动实现 / STM32 GPIO driver implementation
 */
class STM32GPIO final : public GPIO
{
 public:
  /**
   * @brief 构造 GPIO 对象 / Construct GPIO object
   */
  STM32GPIO(GPIO_TypeDef* port, uint16_t pin, IRQn_Type irq = NonMaskableInt_IRQn);

  inline bool Read() { return (port_->IDR & pin_) != 0u; }

  inline void Write(bool value)
  {
    if (value)
    {
      port_->BSRR = static_cast<uint32_t>(pin_);
    }
    else
    {
      port_->BSRR = static_cast<uint32_t>(pin_) << 16;
    }
  }

  ErrorCode EnableInterrupt();

  ErrorCode DisableInterrupt();

  inline ErrorCode SetConfig(Configuration config)
  {
    const bool IS_NON_IRQ_DIRECTION =
        static_cast<uint8_t>(config.direction) <=
        static_cast<uint8_t>(LibXR::GPIO::Direction::OUTPUT_OPEN_DRAIN);

    if (IS_NON_IRQ_DIRECTION)
    {
      const uint32_t PIN_POS = static_cast<uint32_t>(__builtin_ctz(pin_));

#if XR_STM32_GPIO_HAS_F1_LAYOUT
      // 寄存器快路径（F1 类）：CRL/CRH 格式 / Register fast path (F1 class): CRL/CRH.
      volatile uint32_t* cr = (PIN_POS < 8u) ? &port_->CRL : &port_->CRH;
      const uint32_t SHIFT = (PIN_POS & 0x7u) * 4u;
      uint32_t mode_cnf = 0u;

      if (config.direction == Direction::INPUT)
      {
        if (config.pull == Pull::UP || config.pull == Pull::DOWN)
        {
          // MODE=00, CNF=10（上拉/下拉输入）/ Input with pull-up/pull-down.
          mode_cnf = 0x8u;
          if (config.pull == Pull::UP)
          {
            port_->BSRR = static_cast<uint32_t>(pin_);
          }
          else
          {
            port_->BSRR = static_cast<uint32_t>(pin_) << 16u;
          }
        }
        else
        {
          // MODE=00, CNF=01（浮空输入）/ Floating input.
          mode_cnf = 0x4u;
        }
      }
      else if (config.direction == Direction::OUTPUT_PUSH_PULL)
      {
        // MODE=11(50MHz), CNF=00（推挽输出）/ General purpose output push-pull.
        mode_cnf = 0x3u;
      }
      else
      {
        // MODE=11(50MHz), CNF=01（开漏输出）/ General purpose output open-drain.
        mode_cnf = 0x7u;
      }

      uint32_t reg = *cr;
      reg &= ~(0xFu << SHIFT);
      reg |= (mode_cnf << SHIFT);
      *cr = reg;
      return ErrorCode::OK;

#elif XR_STM32_GPIO_HAS_MODER_LAYOUT
      // 寄存器快路径（MODER 类）：MODER/OTYPER/PUPDR/OSPEEDR / Register fast path (MODER
      // class).
      const uint32_t SHIFT = PIN_POS * 2u;
      const uint32_t MASK2 = 0x3u << SHIFT;

      uint32_t moder = port_->MODER;
      moder &= ~MASK2;
      if (config.direction != Direction::INPUT)
      {
        moder |= (0x1u << SHIFT);  // MODER=01：通用输出 / General purpose output.
      }
      port_->MODER = moder;

      if (config.direction != Direction::INPUT)
      {
        const uint32_t PIN_MASK = 0x1u << PIN_POS;
        if (config.direction == Direction::OUTPUT_OPEN_DRAIN)
        {
          port_->OTYPER |= PIN_MASK;
        }
        else
        {
          port_->OTYPER &= ~PIN_MASK;
        }

#if XR_STM32_GPIO_HAS_OSPEEDR
        uint32_t ospeedr = port_->OSPEEDR;
        ospeedr &= ~MASK2;
        ospeedr |= (0x3u << SHIFT);  // SWD 方向切换用最高速率 / Fastest slew for SWD.
        port_->OSPEEDR = ospeedr;
#endif
      }

      uint32_t pupdr = port_->PUPDR;
      pupdr &= ~MASK2;
      if (config.pull == Pull::UP)
      {
        pupdr |= (0x1u << SHIFT);
      }
      else if (config.pull == Pull::DOWN)
      {
        pupdr |= (0x2u << SHIFT);
      }
      port_->PUPDR = pupdr;
      return ErrorCode::OK;
#endif
    }

    GPIO_InitTypeDef gpio_init = {};

    gpio_init.Pin = pin_;

    switch (config.direction)
    {
      case Direction::INPUT:
        gpio_init.Mode = GPIO_MODE_INPUT;
        break;
      case Direction::OUTPUT_PUSH_PULL:
        gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
        break;
      case Direction::OUTPUT_OPEN_DRAIN:
        gpio_init.Mode = GPIO_MODE_OUTPUT_OD;
        break;
      case Direction::FALL_INTERRUPT:
        gpio_init.Mode = GPIO_MODE_IT_FALLING;
        break;
      case Direction::RISING_INTERRUPT:
        gpio_init.Mode = GPIO_MODE_IT_RISING;
        break;
      case Direction::FALL_RISING_INTERRUPT:
        gpio_init.Mode = GPIO_MODE_IT_RISING_FALLING;
        break;
    }

    switch (config.pull)
    {
      case Pull::NONE:
        gpio_init.Pull = GPIO_NOPULL;
        break;
      case Pull::UP:
        gpio_init.Pull = GPIO_PULLUP;
        break;
      case Pull::DOWN:
        gpio_init.Pull = GPIO_PULLDOWN;
        break;
    }

    gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;

    HAL_GPIO_Init(port_, &gpio_init);

    return ErrorCode::OK;
  }

  /// 按 EXTI 线号（0-15）索引的实例表，供 EXTI 回调找回对象 / Instance table indexed by
  /// EXTI line (0-15), used by the EXTI callback to find the object
  static STM32GPIO* map[16];  // NOLINT

 private:
  GPIO_TypeDef* port_;
  uint16_t pin_;
  IRQn_Type irq_;
};

}  // namespace LibXR

#endif
