#include "stm32_watchdog.hpp"

#if defined(HAL_IWDG_MODULE_ENABLED)

using namespace LibXR;

STM32Watchdog::STM32Watchdog(IWDG_HandleTypeDef* hiwdg, uint32_t timeout_ms,
                             uint32_t feed_ms, uint32_t clock)
    : hiwdg_(hiwdg), clock_(clock)
{
  ASSERT(hiwdg);
  const ErrorCode config_result = SetConfig({timeout_ms, feed_ms});
  ASSERT(config_result == ErrorCode::OK);
  Start();
}

ErrorCode STM32Watchdog::SetConfig(const Configuration& config)
{
  if (config.feed_ms == 0 || config.timeout_ms == 0 || config.feed_ms > config.timeout_ms)
  {
    return ErrorCode::ARG_ERR;
  }

  /* 分频与重载自动计算 / Prescaler & reload auto calculation */
  static const struct
  {
    uint8_t pr;
    uint16_t div;
  } TABLE[] = {{0, 4}, {1, 8}, {2, 16}, {3, 32}, {4, 64}, {5, 128}, {6, 256}};
  uint32_t lsi = clock_;
  uint8_t best_pr = 6;
  uint16_t best_rlr = 0xFFF;
  bool found = false;

  for (const auto& item : TABLE)
  {
    uint32_t prescaler = item.div;
    uint32_t reload = (config.timeout_ms * lsi) / (1000 * prescaler);
    if (reload == 0)
    {
      reload = 1;
    }
    if (reload > 1)
    {
      reload--;
    }
    if (reload <= 0xFFF)
    {
      best_pr = item.pr;
      best_rlr = static_cast<uint16_t>(reload);
      found = true;
      break;
    }
  }
  if (!found)
  {
    return ErrorCode::NOT_SUPPORT;
  }

  timeout_ms_ = config.timeout_ms;
  auto_feed_interval_ms = config.feed_ms;

  hiwdg_->Init.Prescaler = best_pr;
  hiwdg_->Init.Reload = best_rlr;
#if defined(IWDG_WINDOW_DISABLE)
  // LibXR 按周期喂狗，没有窗口的概念；关闭窗口，任何时刻喂狗都有效。
  // LibXR feeds the watchdog periodically and has no window concept; disabling the
  // window makes a refresh valid at any time.
  hiwdg_->Init.Window = IWDG_WINDOW_DISABLE;
#endif
#if defined(IWDG)
  hiwdg_->Instance = IWDG;  // NOLINT
#elif defined(IWDG1)
  hiwdg_->Instance = IWDG1;  // NOLINT
#endif

  return ErrorCode::OK;
}

ErrorCode STM32Watchdog::Feed()
{
  return HAL_IWDG_Refresh(hiwdg_) == HAL_OK ? ErrorCode::OK : ErrorCode::FAILED;
}

ErrorCode STM32Watchdog::Start()
{
  auto_feed_ = true;
  if (HAL_IWDG_Init(hiwdg_) != HAL_OK)
  {
    return ErrorCode::FAILED;
  }
  return ErrorCode::OK;
}

ErrorCode STM32Watchdog::Stop()
{
  /* STM32 IWDG 不支持关闭 */
  auto_feed_ = false;
  return ErrorCode::NOT_SUPPORT;
}

#endif
