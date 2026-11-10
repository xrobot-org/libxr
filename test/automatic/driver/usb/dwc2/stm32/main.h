// Fake CubeMX main.h for host tests of the STM32 OTG backend: the register structures
// and bit names of the CMSIS device header that the backend uses, the low-level HAL
// functions it calls and the Cortex-M intrinsics of its hardware guard. The register
// structures sit at the addresses of OTG_HS and OTG_FS on an STM32H7, where the test maps
// the controller model (fake_dwc2.hpp). Bit positions are those of the reference manual.
#pragma once

#include <stddef.h>
#include <stdint.h>

#define __IO volatile
#define HAL_PCD_MODULE_ENABLED
#define STM32H7
#define __DCACHE_PRESENT 1U

typedef struct
{
  __IO uint32_t GOTGCTL;
  __IO uint32_t GOTGINT;
  __IO uint32_t GAHBCFG;
  __IO uint32_t GUSBCFG;
  __IO uint32_t GRSTCTL;
  __IO uint32_t GINTSTS;
  __IO uint32_t GINTMSK;
  __IO uint32_t GRXSTSR;
  __IO uint32_t GRXSTSP;
  __IO uint32_t GRXFSIZ;
  __IO uint32_t DIEPTXF0_HNPTXFSIZ;
  __IO uint32_t HNPTXSTS;
  uint32_t Reserved30[2];
  __IO uint32_t GCCFG;
  __IO uint32_t CID;
  uint32_t Reserved40[48];
  __IO uint32_t HPTXFSIZ;
  __IO uint32_t DIEPTXF[0x0F];
} USB_OTG_GlobalTypeDef;

typedef struct
{
  __IO uint32_t DCFG;
  __IO uint32_t DCTL;
  __IO uint32_t DSTS;
  uint32_t Reserved0C;
  __IO uint32_t DIEPMSK;
  __IO uint32_t DOEPMSK;
  __IO uint32_t DAINT;
  __IO uint32_t DAINTMSK;
  uint32_t Reserved20;
  uint32_t Reserved9;
  __IO uint32_t DVBUSDIS;
  __IO uint32_t DVBUSPULSE;
  __IO uint32_t DTHRCTL;
  __IO uint32_t DIEPEMPMSK;
  __IO uint32_t DEACHINT;
  __IO uint32_t DEACHMSK;
} USB_OTG_DeviceTypeDef;

typedef struct
{
  __IO uint32_t DIEPCTL;
  uint32_t Reserved04;
  __IO uint32_t DIEPINT;
  uint32_t Reserved0C;
  __IO uint32_t DIEPTSIZ;
  __IO uint32_t DIEPDMA;
  __IO uint32_t DTXFSTS;
  uint32_t Reserved18;
} USB_OTG_INEndpointTypeDef;

typedef struct
{
  __IO uint32_t DOEPCTL;
  uint32_t Reserved04;
  __IO uint32_t DOEPINT;
  uint32_t Reserved0C;
  __IO uint32_t DOEPTSIZ;
  __IO uint32_t DOEPDMA;
  uint32_t Reserved18[2];
} USB_OTG_OUTEndpointTypeDef;

#define USB1_OTG_HS_PERIPH_BASE 0x40040000UL
#define USB2_OTG_FS_PERIPH_BASE 0x40080000UL
#define USB_OTG_HS ((USB_OTG_GlobalTypeDef*)USB1_OTG_HS_PERIPH_BASE)
#define USB_OTG_FS ((USB_OTG_GlobalTypeDef*)USB2_OTG_FS_PERIPH_BASE)

#define USB_OTG_DEVICE_BASE 0x800UL
#define USB_OTG_IN_ENDPOINT_BASE 0x900UL
#define USB_OTG_OUT_ENDPOINT_BASE 0xB00UL
#define USB_OTG_EP_REG_SIZE 0x20UL
#define USB_OTG_PCGCCTL_BASE 0xE00UL
#define USB_OTG_FIFO_BASE 0x1000UL
#define USB_OTG_FIFO_SIZE 0x1000UL

#define USB_OTG_GOTGCTL_BVALOEN (1UL << 6)
#define USB_OTG_GOTGCTL_BVALOVAL (1UL << 7)
#define USB_OTG_GOTGINT_SEDET (1UL << 2)

#define USB_OTG_GAHBCFG_GINT (1UL << 0)
#define USB_OTG_GAHBCFG_HBSTLEN_1 (1UL << 2)
#define USB_OTG_GAHBCFG_HBSTLEN_2 (1UL << 3)
#define USB_OTG_GAHBCFG_DMAEN (1UL << 5)

#define USB_OTG_GUSBCFG_PHYSEL (1UL << 6)
#define USB_OTG_GUSBCFG_TRDT (0xFUL << 10)
#define USB_OTG_GUSBCFG_TRDT_Pos 10U
#define USB_OTG_GUSBCFG_FHMOD (1UL << 29)
#define USB_OTG_GUSBCFG_FDMOD (1UL << 30)

#define USB_OTG_GRSTCTL_CSRST (1UL << 0)
#define USB_OTG_GRSTCTL_RXFFLSH (1UL << 4)
#define USB_OTG_GRSTCTL_TXFFLSH (1UL << 5)
#define USB_OTG_GRSTCTL_AHBIDL (1UL << 31)

#define USB_OTG_GINTSTS_CMOD (1UL << 0)
#define USB_OTG_GINTSTS_MMIS (1UL << 1)
#define USB_OTG_GINTSTS_OTGINT (1UL << 2)
#define USB_OTG_GINTSTS_SOF (1UL << 3)
#define USB_OTG_GINTSTS_RXFLVL (1UL << 4)
#define USB_OTG_GINTSTS_BOUTNAKEFF (1UL << 7)
#define USB_OTG_GINTSTS_USBSUSP (1UL << 11)
#define USB_OTG_GINTSTS_USBRST (1UL << 12)
#define USB_OTG_GINTSTS_ENUMDNE (1UL << 13)
#define USB_OTG_GINTSTS_IEPINT (1UL << 18)
#define USB_OTG_GINTSTS_OEPINT (1UL << 19)
#define USB_OTG_GINTSTS_IISOIXFR (1UL << 20)
#define USB_OTG_GINTSTS_SRQINT (1UL << 30)
#define USB_OTG_GINTSTS_WKUINT (1UL << 31)

#define USB_OTG_GINTMSK_OTGINT (1UL << 2)
#define USB_OTG_GINTMSK_SOFM (1UL << 3)
#define USB_OTG_GINTMSK_RXFLVLM (1UL << 4)
#define USB_OTG_GINTMSK_GONAKEFFM (1UL << 7)
#define USB_OTG_GINTMSK_USBSUSPM (1UL << 11)
#define USB_OTG_GINTMSK_USBRST (1UL << 12)
#define USB_OTG_GINTMSK_ENUMDNEM (1UL << 13)
#define USB_OTG_GINTMSK_IEPINT (1UL << 18)
#define USB_OTG_GINTMSK_OEPINT (1UL << 19)
#define USB_OTG_GINTMSK_IISOIXFRM (1UL << 20)
#define USB_OTG_GINTMSK_SRQIM (1UL << 30)
#define USB_OTG_GINTMSK_WUIM (1UL << 31)

#define USB_OTG_GRXSTSP_EPNUM (0xFUL << 0)
#define USB_OTG_GRXSTSP_BCNT (0x7FFUL << 4)
#define USB_OTG_GRXSTSP_BCNT_Pos 4U
#define USB_OTG_GRXSTSP_PKTSTS (0xFUL << 17)
#define USB_OTG_GRXSTSP_PKTSTS_Pos 17U

#define USB_OTG_GCCFG_PWRDWN (1UL << 16)
#define USB_OTG_GCCFG_VBDEN (1UL << 21)

#define USB_OTG_DCFG_DSPD (3UL << 0)
#define USB_OTG_DCFG_DAD (0x7FUL << 4)
#define USB_OTG_DCFG_DAD_Pos 4U

#define USB_OTG_DCTL_RWUSIG (1UL << 0)
#define USB_OTG_DCTL_SDIS (1UL << 1)
#define USB_OTG_DCTL_CGINAK (1UL << 8)
#define USB_OTG_DCTL_SGONAK (1UL << 9)
#define USB_OTG_DCTL_CGONAK (1UL << 10)

#define USB_OTG_DSTS_SUSPSTS (1UL << 0)
#define USB_OTG_DSTS_ENUMSPD (3UL << 1)
#define USB_OTG_DSTS_FNSOF (0x3FFFUL << 8)
#define USB_OTG_DSTS_FNSOF_Pos 8U

#define USB_OTG_DIEPMSK_XFRCM (1UL << 0)
#define USB_OTG_DIEPMSK_EPDM (1UL << 1)
#define USB_OTG_DIEPMSK_TOM (1UL << 3)
#define USB_OTG_DIEPMSK_INEPNEM (1UL << 6)

#define USB_OTG_DOEPMSK_XFRCM (1UL << 0)
#define USB_OTG_DOEPMSK_EPDM (1UL << 1)
#define USB_OTG_DOEPMSK_STUPM (1UL << 3)
#define USB_OTG_DOEPMSK_OTEPSPRM (1UL << 5)

#define USB_OTG_DIEPCTL_MPSIZ (0x7FFUL << 0)
#define USB_OTG_DIEPCTL_USBAEP (1UL << 15)
#define USB_OTG_DIEPCTL_NAKSTS (1UL << 17)
#define USB_OTG_DIEPCTL_EPTYP (3UL << 18)
#define USB_OTG_DIEPCTL_STALL (1UL << 21)
#define USB_OTG_DIEPCTL_TXFNUM (0xFUL << 22)
#define USB_OTG_DIEPCTL_TXFNUM_Pos 22U
#define USB_OTG_DIEPCTL_CNAK (1UL << 26)
#define USB_OTG_DIEPCTL_SNAK (1UL << 27)
#define USB_OTG_DIEPCTL_SD0PID_SEVNFRM (1UL << 28)
#define USB_OTG_DIEPCTL_SODDFRM (1UL << 29)
#define USB_OTG_DIEPCTL_EPDIS (1UL << 30)
#define USB_OTG_DIEPCTL_EPENA (1UL << 31)

#define USB_OTG_DIEPINT_XFRC (1UL << 0)
#define USB_OTG_DIEPINT_EPDISD (1UL << 1)
#define USB_OTG_DIEPINT_TOC (1UL << 3)
#define USB_OTG_DIEPINT_ITTXFE (1UL << 4)
#define USB_OTG_DIEPINT_INEPNE (1UL << 6)
#define USB_OTG_DIEPINT_TXFE (1UL << 7)

#define USB_OTG_DIEPTSIZ_XFRSIZ (0x7FFFFUL << 0)
#define USB_OTG_DIEPTSIZ_PKTCNT (0x3FFUL << 19)
#define USB_OTG_DIEPTSIZ_MULCNT_Pos 29U

#define USB_OTG_DTXFSTS_INEPTFSAV (0xFFFFUL << 0)

#define USB_OTG_DOEPCTL_MPSIZ (0x7FFUL << 0)
#define USB_OTG_DOEPCTL_USBAEP (1UL << 15)
#define USB_OTG_DOEPCTL_NAKSTS (1UL << 17)
#define USB_OTG_DOEPCTL_EPTYP (3UL << 18)
#define USB_OTG_DOEPCTL_STALL (1UL << 21)
#define USB_OTG_DOEPCTL_CNAK (1UL << 26)
#define USB_OTG_DOEPCTL_SNAK (1UL << 27)
#define USB_OTG_DOEPCTL_SD0PID_SEVNFRM (1UL << 28)
#define USB_OTG_DOEPCTL_SODDFRM (1UL << 29)
#define USB_OTG_DOEPCTL_EPDIS (1UL << 30)
#define USB_OTG_DOEPCTL_EPENA (1UL << 31)

#define USB_OTG_DOEPINT_XFRC (1UL << 0)
#define USB_OTG_DOEPINT_EPDISD (1UL << 1)
#define USB_OTG_DOEPINT_STUP (1UL << 3)
#define USB_OTG_DOEPINT_OTEPDIS (1UL << 4)
#define USB_OTG_DOEPINT_OTEPSPR (1UL << 5)
#define USB_OTG_DOEPINT_NAK (1UL << 13)
#define USB_OTG_DOEPINT_STPKTRX (1UL << 15)

#define USB_OTG_DOEPTSIZ_XFRSIZ (0x7FFFFUL << 0)
#define USB_OTG_DOEPTSIZ_PKTCNT (0x3FFUL << 19)
#define USB_OTG_DOEPTSIZ_STUPCNT (3UL << 29)

#define USB_OTG_PCGCCTL_STOPCLK (1UL << 0)
#define USB_OTG_PCGCCTL_GATECLK (1UL << 1)

#define USB_OTG_CORE_ID_300A 0x4F54300AU
#define USB_OTG_CORE_ID_310A 0x4F54310AU

#define STS_GOUT_NAK 1U
#define STS_DATA_UPDT 2U
#define STS_XFER_COMP 3U
#define STS_SETUP_COMP 4U
#define STS_SETUP_UPDT 6U

#define USBD_HS_SPEED 0U
#define USBD_HSINFS_SPEED 1U
#define USBD_FS_SPEED 2U

#define USB_OTG_ULPI_PHY 1U
#define USB_OTG_EMBEDDED_PHY 2U

typedef enum
{
  HAL_OK = 0x00U,
  HAL_ERROR = 0x01U,
  HAL_BUSY = 0x02U,
  HAL_TIMEOUT = 0x03U
} HAL_StatusTypeDef;

typedef enum
{
  USB_DEVICE_MODE = 0,
  USB_HOST_MODE = 1,
  USB_DRD_MODE = 2
} USB_OTG_ModeTypeDef;

typedef struct
{
  uint32_t dev_endpoints;
  uint32_t Host_channels;
  uint32_t dma_enable;
  uint32_t speed;
  uint32_t ep0_mps;
  uint32_t phy_itface;
  uint32_t Sof_enable;
  uint32_t low_power_enable;
  uint32_t lpm_enable;
  uint32_t battery_charging_enable;
  uint32_t vbus_sensing_enable;
  uint32_t use_dedicated_ep1;
  uint32_t use_external_vbus;
} USB_OTG_CfgTypeDef;

typedef struct
{
  USB_OTG_GlobalTypeDef* Instance;
} PCD_HandleTypeDef;

typedef enum
{
  OTG_HS_IRQn = 77,
  OTG_FS_IRQn = 101
} IRQn_Type;

#ifdef __cplusplus
extern "C"
{
#endif

  void HAL_PCD_MspInit(PCD_HandleTypeDef* hpcd);
  HAL_StatusTypeDef USB_CoreInit(USB_OTG_GlobalTypeDef* USBx, USB_OTG_CfgTypeDef cfg);
  HAL_StatusTypeDef USB_SetCurrentMode(USB_OTG_GlobalTypeDef* USBx,
                                       USB_OTG_ModeTypeDef mode);
  HAL_StatusTypeDef USB_DevInit(USB_OTG_GlobalTypeDef* USBx, USB_OTG_CfgTypeDef cfg);
  HAL_StatusTypeDef USB_DevConnect(const USB_OTG_GlobalTypeDef* USBx);
  HAL_StatusTypeDef USB_DevDisconnect(const USB_OTG_GlobalTypeDef* USBx);
  HAL_StatusTypeDef USB_ActivateSetup(const USB_OTG_GlobalTypeDef* USBx);
  uint8_t USB_GetDevSpeed(const USB_OTG_GlobalTypeDef* USBx);
  HAL_StatusTypeDef USB_SetTurnaroundTime(USB_OTG_GlobalTypeDef* USBx, uint32_t hclk,
                                          uint8_t speed);
  uint32_t HAL_RCC_GetHCLKFreq(void);
  void SCB_CleanDCache_by_Addr(volatile void* addr, int32_t dsize);
  void SCB_InvalidateDCache_by_Addr(volatile void* addr, int32_t dsize);

  /// Processor state the intrinsics below act on; the test reads and sets it.
  typedef struct
  {
    uint32_t primask;
    uint32_t ipsr;
    uint32_t msp_inits;
    uint32_t cache_cleans;
    uint32_t cache_invalidates;
  } FakeCortexState;
  extern FakeCortexState fake_cortex;

#ifdef __cplusplus
}
#endif

static inline uint32_t __get_PRIMASK(void) { return fake_cortex.primask; }
static inline void __set_PRIMASK(uint32_t value) { fake_cortex.primask = value; }
static inline void __disable_irq(void) { fake_cortex.primask = 1U; }
static inline void __DMB(void) {}
static inline uint32_t __get_IPSR(void) { return fake_cortex.ipsr; }
