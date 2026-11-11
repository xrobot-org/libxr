#pragma once

#include <cstdint>

#include "esp_def.hpp"
#include "hal/wdt_hal.h"
#include "soc/soc_caps.h"
#include "watchdog.hpp"

namespace LibXR
{

#if CONFIG_ESP_TASK_WDT_EN && !CONFIG_ESP_TASK_WDT_USE_ESP_TIMER
#define LIBXR_ESP_WDT_MWDT0_RESERVED 1
#else
#define LIBXR_ESP_WDT_MWDT0_RESERVED 0
#endif

#if SOC_TIMER_GROUPS >= 2
#if CONFIG_ESP_INT_WDT
#define LIBXR_ESP_WDT_MWDT1_RESERVED 1
#else
#define LIBXR_ESP_WDT_MWDT1_RESERVED 0
#endif
#else
#define LIBXR_ESP_WDT_MWDT1_RESERVED 1
#endif

/// 默认使用的看门狗实例：优先 MWDT1，其次 MWDT0，都被 ESP-IDF 占用时用 RWDT /
/// Default watchdog instance: MWDT1 first, then MWDT0, RWDT when ESP-IDF reserves both
#if (LIBXR_ESP_WDT_MWDT1_RESERVED == 0)
constexpr wdt_inst_t ESP32_WATCHDOG_DEFAULT_INSTANCE = WDT_MWDT1;
#elif (LIBXR_ESP_WDT_MWDT0_RESERVED == 0)
constexpr wdt_inst_t ESP32_WATCHDOG_DEFAULT_INSTANCE = WDT_MWDT0;
#else
constexpr wdt_inst_t ESP32_WATCHDOG_DEFAULT_INSTANCE = WDT_RWDT;
#endif

static_assert(!(ESP32_WATCHDOG_DEFAULT_INSTANCE == WDT_MWDT0 &&
                LIBXR_ESP_WDT_MWDT0_RESERVED),
              "LibXR ESP32Watchdog selected MWDT0, but MWDT0 is reserved by ESP-IDF.");
static_assert(!(ESP32_WATCHDOG_DEFAULT_INSTANCE == WDT_MWDT1 &&
                LIBXR_ESP_WDT_MWDT1_RESERVED),
              "LibXR ESP32Watchdog selected MWDT1, but MWDT1 is reserved by ESP-IDF.");

/**
 * @brief ESP32 硬件看门狗 / ESP32 hardware watchdog
 */
class ESP32Watchdog : public Watchdog
{
 public:
  /**
   * @brief 构造看门狗并立即启动 / Construct the watchdog and start it immediately
   * @param timeout_ms 溢出复位时间（ms）/ Overflow reset time (ms)
   * @param feed_ms 自动喂狗周期（ms），应小于 timeout_ms / Auto-feed interval (ms),
   *        should be below timeout_ms
   */
  explicit ESP32Watchdog(uint32_t timeout_ms = 1000, uint32_t feed_ms = 250);

  ErrorCode SetConfig(const Configuration& config) override;
  ErrorCode Feed() override;
  ErrorCode Start() override;
  ErrorCode Stop() override;

 private:
  static bool IsInstanceSupported(wdt_inst_t instance);
  ErrorCode InitializeHardware();
  ErrorCode ApplyConfiguration();

  wdt_hal_context_t hal_{};
  const wdt_inst_t instance_ = ESP32_WATCHDOG_DEFAULT_INSTANCE;
  bool initialized_ = false;
  bool started_ = false;
};

}  // namespace LibXR

#undef LIBXR_ESP_WDT_MWDT0_RESERVED
#undef LIBXR_ESP_WDT_MWDT1_RESERVED
