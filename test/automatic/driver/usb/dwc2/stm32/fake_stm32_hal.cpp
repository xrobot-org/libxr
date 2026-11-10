// The low-level HAL functions the STM32 OTG backend calls at construction, at connect and
// on enumeration done, written out as the register operations the STM32 HAL performs
// (core initialization, device initialization, soft connect and disconnect, EP0 packet
// size and turnaround time). They act on the controller model through the same
// registers as the backend.
#include "main.h"

FakeCortexState fake_cortex = {};

namespace
{
USB_OTG_DeviceTypeDef* Device(const USB_OTG_GlobalTypeDef* core)
{
  return reinterpret_cast<USB_OTG_DeviceTypeDef*>(reinterpret_cast<uintptr_t>(core) +
                                                  USB_OTG_DEVICE_BASE);
}
USB_OTG_INEndpointTypeDef* InEp(const USB_OTG_GlobalTypeDef* core, uint32_t num)
{
  return reinterpret_cast<USB_OTG_INEndpointTypeDef*>(reinterpret_cast<uintptr_t>(core) +
                                                      USB_OTG_IN_ENDPOINT_BASE +
                                                      num * USB_OTG_EP_REG_SIZE);
}
USB_OTG_OUTEndpointTypeDef* OutEp(const USB_OTG_GlobalTypeDef* core, uint32_t num)
{
  return reinterpret_cast<USB_OTG_OUTEndpointTypeDef*>(reinterpret_cast<uintptr_t>(core) +
                                                       USB_OTG_OUT_ENDPOINT_BASE +
                                                       num * USB_OTG_EP_REG_SIZE);
}
volatile uint32_t& ClockGate(const USB_OTG_GlobalTypeDef* core)
{
  return *reinterpret_cast<volatile uint32_t*>(reinterpret_cast<uintptr_t>(core) +
                                               USB_OTG_PCGCCTL_BASE);
}

constexpr unsigned WAIT_LIMIT = 200000U;

bool WaitSet(const volatile uint32_t& reg, uint32_t bits)
{
  for (unsigned i = 0; i < WAIT_LIMIT; ++i)
    if ((reg & bits) == bits) return true;
  return false;
}
bool WaitClear(const volatile uint32_t& reg, uint32_t bits)
{
  for (unsigned i = 0; i < WAIT_LIMIT; ++i)
    if ((reg & bits) == 0U) return true;
  return false;
}

HAL_StatusTypeDef CoreReset(USB_OTG_GlobalTypeDef* core)
{
  if (!WaitSet(core->GRSTCTL, USB_OTG_GRSTCTL_AHBIDL)) return HAL_TIMEOUT;
  core->GRSTCTL |= USB_OTG_GRSTCTL_CSRST;
  return WaitClear(core->GRSTCTL, USB_OTG_GRSTCTL_CSRST) ? HAL_OK : HAL_TIMEOUT;
}

HAL_StatusTypeDef FlushTx(USB_OTG_GlobalTypeDef* core, uint32_t fifo)
{
  if (!WaitSet(core->GRSTCTL, USB_OTG_GRSTCTL_AHBIDL)) return HAL_TIMEOUT;
  core->GRSTCTL = USB_OTG_GRSTCTL_TXFFLSH | (fifo << 6U);
  return WaitClear(core->GRSTCTL, USB_OTG_GRSTCTL_TXFFLSH) ? HAL_OK : HAL_TIMEOUT;
}

HAL_StatusTypeDef FlushRx(USB_OTG_GlobalTypeDef* core)
{
  if (!WaitSet(core->GRSTCTL, USB_OTG_GRSTCTL_AHBIDL)) return HAL_TIMEOUT;
  core->GRSTCTL = USB_OTG_GRSTCTL_RXFFLSH;
  return WaitClear(core->GRSTCTL, USB_OTG_GRSTCTL_RXFFLSH) ? HAL_OK : HAL_TIMEOUT;
}
}  // namespace

extern "C"
{
  void HAL_PCD_MspInit(PCD_HandleTypeDef* hpcd)
  {
    if (hpcd != nullptr && hpcd->Instance != nullptr) ++fake_cortex.msp_inits;
  }

  HAL_StatusTypeDef USB_CoreInit(USB_OTG_GlobalTypeDef* core, USB_OTG_CfgTypeDef cfg)
  {
    HAL_StatusTypeDef result;
    if (cfg.phy_itface == USB_OTG_ULPI_PHY)
    {
      core->GCCFG &= ~USB_OTG_GCCFG_PWRDWN;
      core->GUSBCFG &= ~USB_OTG_GUSBCFG_PHYSEL;
      result = CoreReset(core);
    }
    else
    {
      core->GUSBCFG |= USB_OTG_GUSBCFG_PHYSEL;
      result = CoreReset(core);
      core->GCCFG |= USB_OTG_GCCFG_PWRDWN;
    }
    if (cfg.dma_enable == 1U)
    {
      core->GAHBCFG |= USB_OTG_GAHBCFG_HBSTLEN_2;
      core->GAHBCFG |= USB_OTG_GAHBCFG_DMAEN;
    }
    return result;
  }

  HAL_StatusTypeDef USB_SetCurrentMode(USB_OTG_GlobalTypeDef* core,
                                       USB_OTG_ModeTypeDef mode)
  {
    core->GUSBCFG &= ~(USB_OTG_GUSBCFG_FHMOD | USB_OTG_GUSBCFG_FDMOD);
    if (mode != USB_DEVICE_MODE) return HAL_ERROR;
    core->GUSBCFG |= USB_OTG_GUSBCFG_FDMOD;
    return WaitClear(core->GINTSTS, USB_OTG_GINTSTS_CMOD) ? HAL_OK : HAL_ERROR;
  }

  HAL_StatusTypeDef USB_DevInit(USB_OTG_GlobalTypeDef* core, USB_OTG_CfgTypeDef cfg)
  {
    HAL_StatusTypeDef result = HAL_OK;
    for (auto& fifo : core->DIEPTXF) fifo = 0U;
    if (cfg.vbus_sensing_enable == 0U)
    {
      Device(core)->DCTL |= USB_OTG_DCTL_SDIS;
      core->GCCFG &= ~USB_OTG_GCCFG_VBDEN;
      core->GOTGCTL |= USB_OTG_GOTGCTL_BVALOEN;
      core->GOTGCTL |= USB_OTG_GOTGCTL_BVALOVAL;
    }
    else
    {
      core->GCCFG |= USB_OTG_GCCFG_VBDEN;
    }
    ClockGate(core) = 0U;
    // Device speed: high speed or full speed on a ULPI PHY, full speed on the embedded
    // PHY.
    uint32_t speed = 3U;
    if (cfg.phy_itface == USB_OTG_ULPI_PHY) speed = cfg.speed == USBD_HS_SPEED ? 0U : 1U;
    Device(core)->DCFG = (Device(core)->DCFG & ~USB_OTG_DCFG_DSPD) | speed;
    if (FlushTx(core, 0x10U) != HAL_OK) result = HAL_ERROR;
    if (FlushRx(core) != HAL_OK) result = HAL_ERROR;
    Device(core)->DIEPMSK = 0U;
    Device(core)->DOEPMSK = 0U;
    Device(core)->DAINTMSK = 0U;
    for (uint32_t i = 0; i < cfg.dev_endpoints; ++i)
    {
      if ((InEp(core, i)->DIEPCTL & USB_OTG_DIEPCTL_EPENA) != 0U)
        InEp(core, i)->DIEPCTL = i == 0U ? USB_OTG_DIEPCTL_SNAK
                                         : (USB_OTG_DIEPCTL_EPDIS | USB_OTG_DIEPCTL_SNAK);
      else
        InEp(core, i)->DIEPCTL = 0U;
      InEp(core, i)->DIEPTSIZ = 0U;
      InEp(core, i)->DIEPINT = 0xFB7FU;
      if ((OutEp(core, i)->DOEPCTL & USB_OTG_DOEPCTL_EPENA) != 0U)
        OutEp(core, i)->DOEPCTL = i == 0U
                                      ? USB_OTG_DOEPCTL_SNAK
                                      : (USB_OTG_DOEPCTL_EPDIS | USB_OTG_DOEPCTL_SNAK);
      else
        OutEp(core, i)->DOEPCTL = 0U;
      OutEp(core, i)->DOEPTSIZ = 0U;
      OutEp(core, i)->DOEPINT = 0xFB7FU;
    }
    core->GINTMSK = 0U;
    core->GINTSTS = 0xBFFFFFFFU;
    if (cfg.dma_enable == 0U) core->GINTMSK |= USB_OTG_GINTMSK_RXFLVLM;
    core->GINTMSK |= USB_OTG_GINTMSK_USBSUSPM | USB_OTG_GINTMSK_USBRST |
                     USB_OTG_GINTMSK_ENUMDNEM | USB_OTG_GINTMSK_IEPINT |
                     USB_OTG_GINTMSK_OEPINT | USB_OTG_GINTMSK_IISOIXFRM |
                     USB_OTG_GINTMSK_WUIM;
    if (cfg.Sof_enable != 0U) core->GINTMSK |= USB_OTG_GINTMSK_SOFM;
    if (cfg.vbus_sensing_enable == 1U)
      core->GINTMSK |= USB_OTG_GINTMSK_SRQIM | USB_OTG_GINTMSK_OTGINT;
    return result;
  }

  HAL_StatusTypeDef USB_DevConnect(const USB_OTG_GlobalTypeDef* core)
  {
    ClockGate(core) &= ~(USB_OTG_PCGCCTL_STOPCLK | USB_OTG_PCGCCTL_GATECLK);
    Device(core)->DCTL &= ~USB_OTG_DCTL_SDIS;
    return HAL_OK;
  }

  HAL_StatusTypeDef USB_DevDisconnect(const USB_OTG_GlobalTypeDef* core)
  {
    ClockGate(core) &= ~(USB_OTG_PCGCCTL_STOPCLK | USB_OTG_PCGCCTL_GATECLK);
    Device(core)->DCTL |= USB_OTG_DCTL_SDIS;
    return HAL_OK;
  }

  HAL_StatusTypeDef USB_ActivateSetup(const USB_OTG_GlobalTypeDef* core)
  {
    InEp(core, 0)->DIEPCTL &= ~USB_OTG_DIEPCTL_MPSIZ;
    Device(core)->DCTL |= USB_OTG_DCTL_CGINAK;
    return HAL_OK;
  }

  uint8_t USB_GetDevSpeed(const USB_OTG_GlobalTypeDef* core)
  {
    const uint32_t speed = (Device(core)->DSTS & USB_OTG_DSTS_ENUMSPD) >> 1U;
    if (speed == 0U) return USBD_HS_SPEED;
    if (speed == 1U || speed == 3U) return USBD_FS_SPEED;
    return 0xFU;
  }

  HAL_StatusTypeDef USB_SetTurnaroundTime(USB_OTG_GlobalTypeDef* core, uint32_t hclk,
                                          uint8_t speed)
  {
    // 9 at high speed; at full speed 6 for an AHB clock of 32 MHz and above.
    const uint32_t turnaround =
        speed == USBD_HS_SPEED ? 9U : (hclk >= 32000000U ? 6U : 15U);
    core->GUSBCFG = (core->GUSBCFG & ~USB_OTG_GUSBCFG_TRDT) |
                    (turnaround << USB_OTG_GUSBCFG_TRDT_Pos);
    return HAL_OK;
  }

  uint32_t HAL_RCC_GetHCLKFreq(void) { return 275000000U; }

  void SCB_CleanDCache_by_Addr(volatile void*, int32_t) { ++fake_cortex.cache_cleans; }

  void SCB_InvalidateDCache_by_Addr(volatile void*, int32_t)
  {
    ++fake_cortex.cache_invalidates;
  }

}  // extern "C"
