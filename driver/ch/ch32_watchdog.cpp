#include "ch32_watchdog.hpp"

using namespace LibXR;

namespace
{
// IWDG 的计数时钟是 LSI，寄存器更新也在 LSI 时钟下完成；先确保它在运行。
// The IWDG counts on the LSI, and its register updates complete on that clock too; make
// sure it runs first.
void StartLsi()
{
  RCC_LSICmd(ENABLE);
  while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET)
  {
  }
}
}  // namespace

CH32Watchdog::CH32Watchdog(uint32_t timeout_ms, uint32_t feed_ms, uint32_t lsi_hz)
    : lsi_hz_(lsi_hz)
{
  ASSERT(lsi_hz > 0);
  const ErrorCode config_result = SetConfig({timeout_ms, feed_ms});
  ASSERT(config_result == ErrorCode::OK);
  Start();
}

ErrorCode CH32Watchdog::SetConfig(const Configuration& config)
{
  if (config.feed_ms == 0 || config.timeout_ms == 0 ||
      config.feed_ms >= config.timeout_ms)
  {
    return ErrorCode::ARG_ERR;
  }

  // 取能放下超时时间的最小分频（12 位重装载值），分辨率最高。
  // Take the smallest prescaler whose 12-bit reload holds the timeout, for the finest
  // resolution.
  bool found = false;
  for (uint8_t code = 0; code <= 6; ++code)
  {
    const uint32_t divider = 4U << code;
    uint64_t reload = (static_cast<uint64_t>(config.timeout_ms) * lsi_hz_) /
                      (1000U * static_cast<uint64_t>(divider));
    if (reload > 1)
    {
      reload--;
    }
    if (reload == 0)
    {
      reload = 1;
    }
    if (reload <= 0xFFF)
    {
      prescaler_ = code;
      reload_ = static_cast<uint16_t>(reload);
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

  StartLsi();
  // PSCR/RLDR 只能在上一次更新完成（PVU/RVU 为 0）后写。
  // PSCR/RLDR may be written only after the previous update has completed (PVU/RVU
  // clear).
  while (IWDG_GetFlagStatus(IWDG_FLAG_PVU | IWDG_FLAG_RVU) != RESET)
  {
  }
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
  IWDG_SetPrescaler(prescaler_);
  IWDG_SetReload(reload_);
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Disable);
  // 计数器还是旧配置留下的值，新超时要到下一次喂狗才生效；等更新完成后喂一次，让它
  // 立即生效，否则旧的剩余时间可能比新的喂狗周期短。
  // The counter still holds what the old configuration left, and the new timeout applies
  // from the next feed; feed once the update has completed so that it applies at once.
  // Otherwise the time left could be shorter than the new feed interval.
  while (IWDG_GetFlagStatus(IWDG_FLAG_PVU | IWDG_FLAG_RVU) != RESET)
  {
  }
  IWDG_ReloadCounter();
  return ErrorCode::OK;
}

ErrorCode CH32Watchdog::Feed()
{
  IWDG_ReloadCounter();
  return ErrorCode::OK;
}

ErrorCode CH32Watchdog::Start()
{
  StartLsi();
  IWDG_ReloadCounter();
  IWDG_Enable();
  auto_feed_ = true;
  return ErrorCode::OK;
}

ErrorCode CH32Watchdog::Stop()
{
  // IWDG 启动后只有复位才能停下。 / Once started, the IWDG stops only on a reset.
  auto_feed_ = false;
  return ErrorCode::NOT_SUPPORT;
}
