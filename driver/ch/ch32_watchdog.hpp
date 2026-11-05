#pragma once

#include "watchdog.hpp"

#include DEF2STR(LIBXR_CH32_CONFIG_FILE)

namespace LibXR
{

/**
 * @brief CH32 IWDG 看门狗驱动 / CH32 independent watchdog driver
 *
 * IWDG 由 LSI 计时，启动后只有复位才能停下。构造时按超时时间配置并启动。
 * The IWDG counts on the LSI and, once started, stops only on a reset. The constructor
 * configures it for the timeout and starts it.
 */
class CH32Watchdog : public Watchdog
{
 public:
  /**
   * @brief 构造并启动看门狗 / Construct and start the watchdog
   * @param timeout_ms 溢出时间（毫秒） / Overflow time in milliseconds
   * @param feed_ms 自动喂狗周期（毫秒） / Auto feed interval in milliseconds
   * @param lsi_hz LSI 频率，CH32V2/V3 为约 40 kHz / LSI frequency, about 40 kHz on
   *        CH32V2/V3
   */
  explicit CH32Watchdog(uint32_t timeout_ms = 1000, uint32_t feed_ms = 250,
                        uint32_t lsi_hz = 40000);

  ErrorCode SetConfig(const Configuration& config) override;

  ErrorCode Feed() override;

  ErrorCode Start() override;

  ErrorCode Stop() override;

 private:
  uint32_t lsi_hz_;        ///< LSI 频率 / LSI frequency (Hz)
  uint8_t prescaler_ = 0;  ///< IWDG_PSCR 分频值 / IWDG_PSCR prescaler code
  uint16_t reload_ = 0;    ///< IWDG_RLDR 重装载值 / IWDG_RLDR reload value
};

}  // namespace LibXR
