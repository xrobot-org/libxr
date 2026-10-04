#include "stm32_pwm.hpp"

#ifdef HAL_TIM_MODULE_ENABLED

using namespace LibXR;

#if defined(STM32H5)
namespace
{
uint32_t GetH5TimerClock(uint32_t peripheral_clock, uint32_t apb_prescaler)
{
  const bool timpre = (RCC->CFGR1 & RCC_CFGR1_TIMPRE) != 0U;
  // APB 编码 0..3 为不分频，4 为二分频，5 为四分频。
  // APB encodings 0..3 mean divide by one, 4 by two, and 5 by four.
  if (apb_prescaler <= (timpre ? 5U : 4U))
  {
    return HAL_RCC_GetHCLKFreq();
  }
  return peripheral_clock * (timpre ? 4U : 2U);
}
}  // namespace
#endif

#if defined(STM32H7)
namespace
{
uint32_t GetH7TimerClock(uint32_t peripheral_clock)
{
  const uint32_t hclk = HAL_RCC_GetHCLKFreq();
  const uint32_t multiplier = (RCC->CFGR & RCC_CFGR_TIMPRE) != 0U ? 4U : 2U;
  // APB 分频为 1/2/4/8/16；TIMPRE 倍频后的时钟不超过 HCLK。
  // APB divisors are 1/2/4/8/16; the TIMPRE multiplier is capped at HCLK.
  // 使用 HAL 解析 PCLK，兼容 D2CFGR 和 CDCFGR 的 H7 型号。
  // Let HAL resolve PCLK for both D2CFGR and CDCFGR H7 variants.
  const uint32_t timer_clock = peripheral_clock * multiplier;
  return timer_clock < hclk ? timer_clock : hclk;
}
}  // namespace
#endif

STM32PWM::STM32PWM(TIM_HandleTypeDef* htim, uint32_t channel, bool complementary)
    : htim_(htim), channel_(channel), complementary_(complementary)
{
}

ErrorCode STM32PWM::SetDutyCycle(float value)
{
  if (value < 0.0f)
  {
    value = 0.0f;
  }
  else if (value > 1.0f)
  {
    value = 1.0f;
  }

  uint32_t pulse =
      static_cast<uint32_t>(static_cast<float>(htim_->Init.Period + 1) * value);

  // 按通道直接写比较寄存器，映射与 __HAL_TIM_SET_COMPARE 相同（未列出的通道值写最后一个
  // 寄存器）。HAL 宏用条件表达式取用 volatile 赋值的结果，C++20 下会产生弃用警告。
  // Write the compare register of the channel directly, with the same mapping as
  // __HAL_TIM_SET_COMPARE (an unlisted channel value writes the last register). The HAL
  // macro uses the result of a volatile assignment in a conditional expression, which is
  // deprecated in C++20.
  switch (channel_)
  {
    case TIM_CHANNEL_1:
      htim_->Instance->CCR1 = pulse;
      break;
    case TIM_CHANNEL_2:
      htim_->Instance->CCR2 = pulse;
      break;
    case TIM_CHANNEL_3:
      htim_->Instance->CCR3 = pulse;
      break;
#if defined(TIM_CHANNEL_5) && defined(TIM_CHANNEL_6)
    case TIM_CHANNEL_4:
      htim_->Instance->CCR4 = pulse;
      break;
    case TIM_CHANNEL_5:
      htim_->Instance->CCR5 = pulse;
      break;
    default:
      htim_->Instance->CCR6 = pulse;
      break;
#else
    default:
      htim_->Instance->CCR4 = pulse;
      break;
#endif
  }

  return ErrorCode::OK;
}

ErrorCode STM32PWM::SetConfig(Configuration config)
{
  uint32_t clock_freq = 0;  // NOLINT

#if defined(STM32F0) || defined(STM32G0)
  // STM32F0 and STM32 G0 do not have PCLK1/PCLK2，use HCLK as reference clock
  clock_freq = HAL_RCC_GetHCLKFreq();
#else
  uint32_t& target_freq = config.frequency;
  if (target_freq == 0)
  {
    return ErrorCode::ARG_ERR;
  }
  else if (
#if defined(TIM1)
      htim_->Instance == TIM1 ||
#endif
#if defined(TIM8)
      htim_->Instance == TIM8 ||
#endif
#if defined(TIM9)
      htim_->Instance == TIM9 ||
#endif
#if defined(TIM10)
      htim_->Instance == TIM10 ||
#endif
#if defined(TIM11)
      htim_->Instance == TIM11 ||
#endif
#if defined(TIM15)
      htim_->Instance == TIM15 ||
#endif
#if defined(TIM16)
      htim_->Instance == TIM16 ||
#endif
#if defined(TIM17)
      htim_->Instance == TIM17 ||
#endif
#if defined(TIM20)
      htim_->Instance == TIM20 ||
#endif
      false)
  {
    clock_freq = HAL_RCC_GetPCLK2Freq();
#if defined(STM32H5)
    clock_freq = GetH5TimerClock(clock_freq,
                                 (RCC->CFGR2 & RCC_CFGR2_PPRE2) >> RCC_CFGR2_PPRE2_Pos);
#elif defined(STM32H7)
    clock_freq = GetH7TimerClock(clock_freq);
#elif defined(RCC_CFGR_PPRE2)
    if ((RCC->CFGR & RCC_CFGR_PPRE2) != RCC_CFGR_PPRE2_DIV1)
    {
      clock_freq *= 2;
    }
#endif
  }
  else if (
#if defined(TIM2)
      htim_->Instance == TIM2 ||
#endif
#if defined(TIM3)
      htim_->Instance == TIM3 ||
#endif
#if defined(TIM4)
      htim_->Instance == TIM4 ||
#endif
#if defined(TIM5)
      htim_->Instance == TIM5 ||
#endif
#if defined(TIM6)
      htim_->Instance == TIM6 ||
#endif
#if defined(TIM7)
      htim_->Instance == TIM7 ||
#endif
#if defined(TIM12)
      htim_->Instance == TIM12 ||
#endif
#if defined(TIM13)
      htim_->Instance == TIM13 ||
#endif
#if defined(TIM14)
      htim_->Instance == TIM14 ||
#endif
      false)
  {
    clock_freq = HAL_RCC_GetPCLK1Freq();
#if defined(STM32H5)
    clock_freq = GetH5TimerClock(clock_freq,
                                 (RCC->CFGR2 & RCC_CFGR2_PPRE1) >> RCC_CFGR2_PPRE1_Pos);
#elif defined(STM32H7)
    clock_freq = GetH7TimerClock(clock_freq);
#elif defined(RCC_CFGR_PPRE1)
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1)
    {
      clock_freq *= 2;
    }
#endif
  }
  else
  {
    return ErrorCode::NOT_SUPPORT;
  }
#endif

  if (clock_freq == 0)
  {
    return ErrorCode::INIT_ERR;
  }

  bool found = false;

  uint32_t prescaler = 1;
  uint32_t period = 0;

  for (prescaler = 1; prescaler <= 0xFFFF; ++prescaler)
  {
    uint32_t temp_period = clock_freq / (prescaler * config.frequency);
    if (temp_period <= 0x10000)
    {
      period = temp_period;
      found = true;
      break;
    }
  }

  if (!found || period == 0)
  {
    return ErrorCode::INIT_ERR;
  }

  htim_->Init.Prescaler = prescaler - 1;
  htim_->Init.Period = period - 1;

  if (HAL_TIM_PWM_Init(htim_) != HAL_OK)
  {
    return ErrorCode::INIT_ERR;
  }
  return ErrorCode::OK;
}

ErrorCode STM32PWM::Enable()
{
  if (!complementary_)
  {
    if (HAL_TIM_PWM_Start(htim_, channel_) != HAL_OK)
    {
      return ErrorCode::FAILED;
    }
  }
  else
  {
    if (HAL_TIMEx_PWMN_Start(htim_, channel_) != HAL_OK)
    {
      return ErrorCode::FAILED;
    }
  }
  return ErrorCode::OK;
}

ErrorCode STM32PWM::Disable()
{
  if (!complementary_)
  {
    if (HAL_TIM_PWM_Stop(htim_, channel_) != HAL_OK)
    {
      return ErrorCode::FAILED;
    }
  }
  else
  {
    if (HAL_TIMEx_PWMN_Stop(htim_, channel_) != HAL_OK)
    {
      return ErrorCode::FAILED;
    }
  }
  return ErrorCode::OK;
}

#endif
