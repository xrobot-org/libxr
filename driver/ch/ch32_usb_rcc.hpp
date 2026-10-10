#pragma once

#include <cstdint>

#include "libxr_def.hpp"
#include DEF2STR(LIBXR_CH32_CONFIG_FILE)

namespace LibXR::CH32UsbRcc
{

inline uint32_t GetSysclkHz()
{
  RCC_ClocksTypeDef clk{};
  RCC_GetClocksFreq(&clk);
  return clk.SYSCLK_Frequency;
}

inline void ConfigureUsb48MFromSysclk()
{
  // 传统 CH32 器件直接从 SYSCLK / PLL 分频得到共享 USB 48 MHz 时钟。
  // Legacy CH32 parts derive the shared USB 48 MHz clock directly from SYSCLK / PLL
  // dividers.
  const uint32_t sysclk_hz = GetSysclkHz();

#if defined(RCC_USBCLKSource_PLLCLK_Div1) && defined(RCC_USBCLKSource_PLLCLK_Div2) && \
    defined(RCC_USBCLKSource_PLLCLK_Div3)
  if (sysclk_hz == 144000000u)
  {
    RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_Div3);
    return;
  }
  if (sysclk_hz == 96000000u)
  {
    RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_Div2);
    return;
  }
  if (sysclk_hz == 48000000u)
  {
    RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_Div1);
    return;
  }
#if defined(RCC_USB5PRE_JUDGE) && defined(RCC_USBCLKSource_PLLCLK_Div5)
  if (sysclk_hz == 240000000u)
  {
    ASSERT(RCC_USB5PRE_JUDGE() == SET);
    RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_Div5);
    return;
  }
#endif
#elif defined(RCC_USBCLK48MCLKSource_PLLCLK) && \
    defined(RCC_USBFSCLKSource_PLLCLK_Div1) &&  \
    defined(RCC_USBFSCLKSource_PLLCLK_Div2) && defined(RCC_USBFSCLKSource_PLLCLK_Div3)
  RCC_USBCLK48MConfig(RCC_USBCLK48MCLKSource_PLLCLK);

  if (sysclk_hz == 144000000u)
  {
    RCC_USBFSCLKConfig(RCC_USBFSCLKSource_PLLCLK_Div3);
    return;
  }
  if (sysclk_hz == 96000000u)
  {
    RCC_USBFSCLKConfig(RCC_USBFSCLKSource_PLLCLK_Div2);
    return;
  }
  if (sysclk_hz == 48000000u)
  {
    RCC_USBFSCLKConfig(RCC_USBFSCLKSource_PLLCLK_Div1);
    return;
  }
#endif

  ASSERT(false);
}

#if defined(RCC_HSBHSPLLCLKSource_HSE) && defined(RCC_USBPLL_Div1) &&                   \
    defined(RCC_USBPLL_Div2) && defined(RCC_USBPLL_Div3) && defined(RCC_USBPLL_Div4) && \
    defined(RCC_USBPLL_Div5) && defined(RCC_USBPLL_Div6) && defined(RCC_USBPLL_Div7) && \
    defined(RCC_USBPLL_Div8) && defined(RCC_USBHSPLLCKREFCLK_3M) &&                     \
    defined(RCC_USBHSPLLCKREFCLK_4M) && defined(RCC_USBHSPLLCKREFCLK_5M) &&             \
    defined(RCC_USBHSPLLCKREFCLK_8M)
struct UsbHsPllConfig
{
  uint32_t divider_cfg = 0u;
  uint32_t ref_cfg = 0u;
};

struct UsbHsPllTableEntry
{
  uint32_t hse_hz = 0u;
  uint32_t divider_cfg = 0u;
  uint32_t ref_cfg = 0u;
};

constexpr bool TryGetUsbHsPllConfigForHse(uint32_t hse_hz, UsbHsPllConfig& cfg)
{
  // 这里保留显式查表，不做动态推导；
  // USBHS 合法参考时钟是离散集合，精确表更容易审计。
  // Keep this as an explicit lookup table instead of inferring combinations dynamically;
  // legal USBHS reference clocks are discrete and easier to audit in table form.
  constexpr UsbHsPllTableEntry table[] = {
      {3000000u, RCC_USBPLL_Div1, RCC_USBHSPLLCKREFCLK_3M},
      {4000000u, RCC_USBPLL_Div1, RCC_USBHSPLLCKREFCLK_4M},
      {5000000u, RCC_USBPLL_Div1, RCC_USBHSPLLCKREFCLK_5M},
      {6000000u, RCC_USBPLL_Div2, RCC_USBHSPLLCKREFCLK_3M},
      {8000000u, RCC_USBPLL_Div2, RCC_USBHSPLLCKREFCLK_4M},
      {9000000u, RCC_USBPLL_Div3, RCC_USBHSPLLCKREFCLK_3M},
      {10000000u, RCC_USBPLL_Div2, RCC_USBHSPLLCKREFCLK_5M},
      {12000000u, RCC_USBPLL_Div3, RCC_USBHSPLLCKREFCLK_4M},
      {15000000u, RCC_USBPLL_Div3, RCC_USBHSPLLCKREFCLK_5M},
      {16000000u, RCC_USBPLL_Div4, RCC_USBHSPLLCKREFCLK_4M},
      {18000000u, RCC_USBPLL_Div6, RCC_USBHSPLLCKREFCLK_3M},
      {20000000u, RCC_USBPLL_Div5, RCC_USBHSPLLCKREFCLK_4M},
      {21000000u, RCC_USBPLL_Div7, RCC_USBHSPLLCKREFCLK_3M},
      {24000000u, RCC_USBPLL_Div6, RCC_USBHSPLLCKREFCLK_4M},
      {25000000u, RCC_USBPLL_Div5, RCC_USBHSPLLCKREFCLK_5M},
      {28000000u, RCC_USBPLL_Div7, RCC_USBHSPLLCKREFCLK_4M},
      {30000000u, RCC_USBPLL_Div6, RCC_USBHSPLLCKREFCLK_5M},
      {32000000u, RCC_USBPLL_Div8, RCC_USBHSPLLCKREFCLK_4M},
      {35000000u, RCC_USBPLL_Div7, RCC_USBHSPLLCKREFCLK_5M},
      {40000000u, RCC_USBPLL_Div8, RCC_USBHSPLLCKREFCLK_5M},
      {48000000u, RCC_USBPLL_Div6, RCC_USBHSPLLCKREFCLK_8M},
      {56000000u, RCC_USBPLL_Div7, RCC_USBHSPLLCKREFCLK_8M},
      {64000000u, RCC_USBPLL_Div8, RCC_USBHSPLLCKREFCLK_8M},
  };

  for (const auto& entry : table)
  {
    if (entry.hse_hz != hse_hz)
    {
      continue;
    }
    cfg.divider_cfg = entry.divider_cfg;
    cfg.ref_cfg = entry.ref_cfg;
    return true;
  }

  return false;
}

inline void ConfigureUsbHsPhyFromHse()
{
  // USBHS PHY 路径通过 HSE -> (分频, 参考时钟) 查表，
  // 保证合法组合保持显式且可审计。
  // The USBHS PHY path uses an HSE -> (divider, ref clock) lookup so the
  // legal combinations stay explicit and auditable.
  UsbHsPllConfig cfg = {};
  constexpr uint32_t hse_hz = static_cast<uint32_t>(HSE_VALUE);
  constexpr bool hse_supported = []
  {
    UsbHsPllConfig probe = {};
    return TryGetUsbHsPllConfigForHse(hse_hz, probe);
  }();
  static_assert(hse_supported, "HSE_VALUE is not a legal USBHS PHY PLL reference clock");
  (void)TryGetUsbHsPllConfigForHse(hse_hz, cfg);

  RCC_USBHSPLLCLKConfig(RCC_HSBHSPLLCLKSource_HSE);
  RCC_USBHSConfig(cfg.divider_cfg);
  RCC_USBHSPLLCKREFCLKConfig(cfg.ref_cfg);
  RCC_USBHSPHYPLLALIVEcmd(ENABLE);
}
#endif

inline void ConfigureUsb48M()
{
#if defined(RCC_USBCLK48MCLKSource_USBPHY) && defined(RCC_HSBHSPLLCLKSource_HSE)
  // 部分 CH32V30x 可以从 USBHS PHY PLL 提供共享 48 MHz USB 时钟；
  // 这种情况下先选 PHY 路径，再让各控制器自己打开总线时钟。
  // Some CH32V30x parts can source the shared 48 MHz USB clock from the USBHS PHY PLL;
  // in that case, select the PHY path first and let each controller enable its own bus
  // clock.
  ConfigureUsbHsPhyFromHse();
  RCC_USBCLK48MConfig(RCC_USBCLK48MCLKSource_USBPHY);
#else
  // 经典路径：直接从 SYSCLK / PLL 分频得到共享 48 MHz 时钟。
  // Classic path: derive the shared 48 MHz clock directly from SYSCLK / PLL dividers.
  ConfigureUsb48MFromSysclk();
#endif
}

}  // namespace LibXR::CH32UsbRcc
