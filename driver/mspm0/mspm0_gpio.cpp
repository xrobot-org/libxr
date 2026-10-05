#include "mspm0_gpio.hpp"

#include "mspm0_group1_shared.hpp"

using namespace LibXR;

MSPM0GPIO* MSPM0GPIO::instance_map_[MAX_PORTS][32] = {{nullptr}};

/**
 * @brief 计算 GPIO 中断极性掩码
 *
 * @param pin 引脚号 (0-31)
 * @param direction 中断触发方向
 * @return uint32_t 极性掩码，用于配置中断触发边沿
 */
static constexpr uint32_t MSPM0_GPIO_GetPolarityMask(uint8_t pin,  // NOLINT
                                                     LibXR::GPIO::Direction direction)
{
  uint32_t config_bits = 0;
  switch (direction)
  {
    case LibXR::GPIO::Direction::RISING_INTERRUPT:
      config_bits = GPIO_POLARITY15_0_DIO0_RISE;
      break;
    case LibXR::GPIO::Direction::FALL_INTERRUPT:
      config_bits = GPIO_POLARITY15_0_DIO0_FALL;
      break;
    case LibXR::GPIO::Direction::FALL_RISING_INTERRUPT:
      config_bits = GPIO_POLARITY15_0_DIO0_RISE_FALL;
      break;
    default:
      return 0;
  }

  uint32_t shift = (pin % 16) * 2;
  return config_bits << shift;
}
#ifdef GPIOA_BASE
static void gpioa_irq_thunk() { LibXR::MSPM0GPIO::OnInterrupt(GPIOA); }
#endif

#ifdef GPIOB_BASE
static void gpiob_irq_thunk() { LibXR::MSPM0GPIO::OnInterrupt(GPIOB); }
#endif

#ifdef GPIOC_BASE
static void gpioc_irq_thunk() { LibXR::MSPM0GPIO::OnInterrupt(GPIOC); }
#endif

MSPM0GPIO::MSPM0GPIO(GPIO_Regs* port, uint32_t pin_mask, uint32_t pincm)
    : port_(port), pin_mask_(pin_mask), pincm_(pincm)
{
  int pin_idx = __builtin_ctz(pin_mask_);
  int port_idx = GetPortIndex(reinterpret_cast<uint32_t>(port_));

  ASSERT(port_idx >= 0 && port_idx < LibXR::MAX_PORTS);
  // 端口来自固定配置，前提已由 ASSERT 检查；关闭检查时告知编译器该前提成立。
  // The port is fixed configuration checked by the ASSERT above; with checks off, tell the
  // compiler that the precondition holds.
  if (port_idx < 0 || port_idx >= LibXR::MAX_PORTS)
  {
    __builtin_unreachable();
  }
  ASSERT(instance_map_[port_idx][pin_idx] == nullptr);

  instance_map_[port_idx][pin_idx] = this;

  switch (port_idx)
  {
    case 0:
#ifdef GPIOA_BASE
      LibXR::MSPM0Group1Shared::RegisterGPIOA(&gpioa_irq_thunk);
#endif
      break;

    case 1:
#ifdef GPIOB_BASE
      LibXR::MSPM0Group1Shared::RegisterGPIOB(&gpiob_irq_thunk);
#endif
      break;

    case 2:
#ifdef GPIOC_BASE
      LibXR::MSPM0Group1Shared::RegisterGPIOC(&gpioc_irq_thunk);
#endif
      break;
    default:
      return;
  }
}

bool MSPM0GPIO::Read() { return DL_GPIO_readPins(port_, pin_mask_) == pin_mask_; }

void MSPM0GPIO::Write(bool value)
{
  if (current_direction_ == LibXR::GPIO::Direction::OUTPUT_OPEN_DRAIN)
  {
    if (value)
    {
      DL_GPIO_disableOutput(port_, pin_mask_);
    }
    else
    {
      DL_GPIO_enableOutput(port_, pin_mask_);
    }
  }
  else
  {
    if (value)
    {
      DL_GPIO_setPins(port_, pin_mask_);
    }
    else
    {
      DL_GPIO_clearPins(port_, pin_mask_);
    }
  }
}

ErrorCode MSPM0GPIO::EnableInterrupt()
{
  DL_GPIO_enableInterrupt(port_, pin_mask_);
  return ErrorCode::OK;
}

ErrorCode MSPM0GPIO::DisableInterrupt()
{
  DL_GPIO_disableInterrupt(port_, pin_mask_);
  return ErrorCode::OK;
}

ErrorCode MSPM0GPIO::SetConfig(Configuration config)
{
  current_direction_ = config.direction;

  DL_GPIO_disableOutput(port_, pin_mask_);
  DL_GPIO_disableInterrupt(port_, pin_mask_);
  DL_GPIO_clearInterruptStatus(port_, pin_mask_);

  switch (config.direction)
  {
    case Direction::INPUT:
    {
      DL_GPIO_RESISTOR res = DL_GPIO_RESISTOR_NONE;
      if (config.pull == Pull::UP)
      {
        res = DL_GPIO_RESISTOR_PULL_UP;
      }
      else if (config.pull == Pull::DOWN)
      {
        res = DL_GPIO_RESISTOR_PULL_DOWN;
      }
      DL_GPIO_initDigitalInputFeatures(pincm_, DL_GPIO_INVERSION_DISABLE, res,
                                       DL_GPIO_HYSTERESIS_DISABLE,
                                       DL_GPIO_WAKEUP_DISABLE);
      break;
    }

    case Direction::OUTPUT_PUSH_PULL:
    {
      DL_GPIO_RESISTOR res = DL_GPIO_RESISTOR_NONE;
      if (config.pull == Pull::UP)
      {
        res = DL_GPIO_RESISTOR_PULL_UP;
      }
      else if (config.pull == Pull::DOWN)
      {
        res = DL_GPIO_RESISTOR_PULL_DOWN;
      }

      DL_GPIO_initDigitalOutputFeatures(pincm_, DL_GPIO_INVERSION_DISABLE, res,
                                        DL_GPIO_DRIVE_STRENGTH_LOW, DL_GPIO_HIZ_DISABLE);

      volatile uint32_t* pincm_reg = &IOMUX->SECCFG.PINCM[pincm_];
      *pincm_reg |= IOMUX_PINCM_INENA_ENABLE;

      DL_GPIO_clearPins(port_, pin_mask_);

      DL_GPIO_enableOutput(port_, pin_mask_);
      break;
    }

    case Direction::OUTPUT_OPEN_DRAIN:
    {
      /* 伪开漏：输入模式(高阻) + 软件控制拉低 / Pseudo open-drain: Input mode(HiZ) +
       * software pull-low */
      DL_GPIO_RESISTOR res = DL_GPIO_RESISTOR_NONE;
      if (config.pull == Pull::UP)
      {
        res = DL_GPIO_RESISTOR_PULL_UP;
      }

      DL_GPIO_initDigitalInputFeatures(pincm_, DL_GPIO_INVERSION_DISABLE, res,
                                       DL_GPIO_HYSTERESIS_DISABLE,
                                       DL_GPIO_WAKEUP_DISABLE);

      DL_GPIO_clearPins(port_, pin_mask_);
      DL_GPIO_disableOutput(port_, pin_mask_);
      break;
    }

    case Direction::FALL_INTERRUPT:
    case Direction::RISING_INTERRUPT:
    case Direction::FALL_RISING_INTERRUPT:
    {
      DL_GPIO_RESISTOR res = DL_GPIO_RESISTOR_NONE;
      if (config.pull == Pull::UP)
      {
        res = DL_GPIO_RESISTOR_PULL_UP;
      }
      else if (config.pull == Pull::DOWN)
      {
        res = DL_GPIO_RESISTOR_PULL_DOWN;
      }

      DL_GPIO_initDigitalInputFeatures(pincm_, DL_GPIO_INVERSION_DISABLE, res,
                                       DL_GPIO_HYSTERESIS_DISABLE,
                                       DL_GPIO_WAKEUP_DISABLE);

      uint32_t pol_mask =
          MSPM0_GPIO_GetPolarityMask(__builtin_ctz(pin_mask_), config.direction);
      if (pol_mask)
      {
        uint32_t pin_idx = __builtin_ctz(pin_mask_);

        constexpr uint32_t BITS_PER_PIN = 2;
        constexpr uint32_t CLEAR_PATTERN = 0x3U;

        uint32_t shift = (pin_idx % 16) * BITS_PER_PIN;
        uint32_t clear_mask = CLEAR_PATTERN << shift;

        if (pin_idx < 16)
        {
          uint32_t current_polarity = DL_GPIO_getLowerPinsPolarity(port_);
          current_polarity &= ~clear_mask;
          current_polarity |= pol_mask;
          port_->POLARITY15_0 = current_polarity;
        }
        else
        {
          uint32_t current_polarity = DL_GPIO_getUpperPinsPolarity(port_);
          current_polarity &= ~clear_mask;
          current_polarity |= pol_mask;
          port_->POLARITY31_16 = current_polarity;
        }
      }

      DL_GPIO_clearInterruptStatus(port_, pin_mask_);
      DL_GPIO_enableInterrupt(port_, pin_mask_);
    }
    break;

    default:
      return ErrorCode::FAILED;
  }

  return ErrorCode::OK;
}

void MSPM0GPIO::OnInterruptDispatch(GPIO_Regs* port, int port_idx)
{
  if (port_idx < 0 || port_idx >= LibXR::MAX_PORTS)
  {
    return;
  }

  uint32_t pending_pins = DL_GPIO_getEnabledInterruptStatus(port, 0xFFFFFFFF);

  if (pending_pins != 0)
  {
    DL_GPIO_clearInterruptStatus(port, pending_pins);
  }

  while (pending_pins)
  {
    uint32_t pin_idx = __builtin_ctz(pending_pins);
    uint32_t pin_mask = (1U << pin_idx);

    pending_pins &= ~pin_mask;

    auto* instance = instance_map_[port_idx][pin_idx];
    if (instance)
    {
      instance->callback_.Run(true);
    }
  }
}
