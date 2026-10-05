#include "mspm0_pwm.hpp"

using namespace LibXR;

static constexpr uint32_t MAX_TIMER_LOAD = 65535;
static constexpr uint32_t MAX_PRESCALER = 256;
static constexpr uint32_t MAX_DIVIDE_RATIO = 8;

MSPM0PWM::MSPM0PWM(MSPM0PWM::Resources res)
    : timer_(res.timer), channel_(res.channel), clock_freq_(res.clock_freq)
{
  ASSERT(timer_ != nullptr);
  ASSERT(clock_freq_ > 0);
}

ErrorCode MSPM0PWM::SetDutyCycle(float value)
{
  if (value < 0.0f)
  {
    value = 0.0f;
  }
  if (value > 1.0f)
  {
    value = 1.0f;
  }

  uint32_t period = DL_Timer_getLoadValue(timer_);
  uint32_t compare_value = 0;
  DL_TIMER_COUNT_MODE count_mode = DL_Timer_getCounterMode(timer_);

  switch (count_mode)
  {
    case DL_TIMER_COUNT_MODE_DOWN:
      compare_value = static_cast<uint32_t>(static_cast<float>(period) * (1.0f - value));
      break;

    case DL_TIMER_COUNT_MODE_UP:
    case DL_TIMER_COUNT_MODE_UP_DOWN:
      compare_value = static_cast<uint32_t>(static_cast<float>(period) * value);
      break;

    default:
      return ErrorCode::NOT_SUPPORT;
  }

  if (compare_value > period)
  {
    compare_value = period;
  }

  DL_Timer_setCaptureCompareValue(timer_, compare_value, channel_);

  return ErrorCode::OK;
}

uint32_t MSPM0PWM::SourceClockFromSysCfg(GPTIMER_Regs* timer, uint32_t timer_clock_hz)
{
  DL_Timer_ClockConfig config;
  DL_Timer_getClockConfig(timer, &config);
  return timer_clock_hz * (static_cast<uint32_t>(config.divideRatio) + 1U) *
         (static_cast<uint32_t>(config.prescale) + 1U);
}

ErrorCode MSPM0PWM::SetConfig(Configuration config)
{
  const uint32_t SOURCE_CLOCK = clock_freq_;
  if (config.frequency > SOURCE_CLOCK || config.frequency == 0)
  {
    return ErrorCode::ARG_ERR;
  }

  const uint32_t TOTAL_CYCLES_NEEDED = SOURCE_CLOCK / config.frequency;
  uint32_t min_total_prescale = (TOTAL_CYCLES_NEEDED + MAX_TIMER_LOAD) >> 16;
  if (min_total_prescale == 0)
  {
    min_total_prescale = 1;
  }

  uint32_t best_div = (min_total_prescale + MAX_PRESCALER - 1) >> 8;
  if (best_div > MAX_DIVIDE_RATIO)
  {
    return ErrorCode::NOT_SUPPORT;  // Frequency too low
  }
  if (best_div == 0)
  {
    best_div = 1;
  }

  uint32_t best_prescaler = (min_total_prescale + best_div - 1) / best_div;
  if (best_prescaler == 0)
  {
    best_prescaler = 1;
  }

  uint32_t best_load = (SOURCE_CLOCK / (best_div * best_prescaler * config.frequency));
  if (best_load == 0)
  {
    return ErrorCode::NOT_SUPPORT;  // Frequency too high to represent
  }
  best_load -= 1;

  DL_Timer_ClockConfig clock_config;
  DL_Timer_getClockConfig(timer_, &clock_config);

  clock_config.divideRatio = static_cast<DL_TIMER_CLOCK_DIVIDE>(best_div - 1);
  clock_config.prescale = static_cast<uint8_t>(best_prescaler - 1);

  DL_Timer_setClockConfig(timer_, &clock_config);
  DL_Timer_setLoadValue(timer_, best_load);

  return ErrorCode::OK;
}

ErrorCode MSPM0PWM::Enable()
{
  DL_Timer_startCounter(timer_);
  return ErrorCode::OK;
}

ErrorCode MSPM0PWM::Disable()
{
  DL_Timer_stopCounter(timer_);
  return ErrorCode::OK;
}
