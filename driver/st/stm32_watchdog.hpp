#pragma once

#include "main.h"
#include "watchdog.hpp"

#if defined(HAL_IWDG_MODULE_ENABLED)

namespace LibXR
{

/**
 * @brief STM32 IWDG 看门狗驱动 / STM32 independent watchdog driver
 *
 */
class STM32Watchdog : public Watchdog
{
 public:
  /**
   * @brief 构造看门狗并立即启动 / Construct the watchdog and start it immediately
   * @param hiwdg HAL IWDG 句柄，不能为空 / HAL IWDG handle, must not be null
   * @param timeout_ms 溢出复位时间（ms）/ Overflow reset time (ms)
   * @param feed_ms 自动喂狗周期（ms），不大于 timeout_ms / Auto-feed interval (ms), not
   *        above timeout_ms
   * @param clock IWDG 计数时钟（Hz），默认 LSI_VALUE / IWDG counter clock (Hz), LSI_VALUE
   *        by default
   */
  explicit STM32Watchdog(IWDG_HandleTypeDef* hiwdg, uint32_t timeout_ms = 1000,
                         uint32_t feed_ms = 250, uint32_t clock = LSI_VALUE);

  ErrorCode SetConfig(const Configuration& config) override;

  ErrorCode Feed() override;

  ErrorCode Start() override;

  ErrorCode Stop() override;

  IWDG_HandleTypeDef* hiwdg_;  ///< STM32 HAL IWDG handle
  uint32_t clock_;             ///< LSI clock in Hz
};

}  // namespace LibXR

#endif
