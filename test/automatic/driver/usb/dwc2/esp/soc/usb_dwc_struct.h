// Fake soc/usb_dwc_struct.h for host tests of the ESP32-S2/S3 USB backend: the register
// block of the USB-OTG (DWC2) controller as volatile bit-field unions, with the type,
// register and field names the backend uses (ESP-IDF 5.3 to 5.5). Offsets and bit
// positions are those of the ESP32-S3 register description; only the fields the backend
// touches are named, the rest is reserved. The test maps the controller model
// (fake_dwc2.hpp) at the register base address 0x60080000.
#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

  typedef union
  {
    struct
    {
      uint32_t reserved_0 : 4;
      uint32_t avalidoven : 1;
      uint32_t avalidovval : 1;
      uint32_t bvalidoven : 1;
      uint32_t bvalidovval : 1;
      uint32_t reserved_8 : 24;
    };
    uint32_t val;
  } usb_dwc_gotgctl_reg_t;

  typedef union
  {
    struct
    {
      uint32_t reserved_0 : 2;
      uint32_t sesenddet : 1;
      uint32_t reserved_3 : 29;
    };
    uint32_t val;
  } usb_dwc_gotgint_reg_t;

  typedef union
  {
    struct
    {
      uint32_t glbllntrmsk : 1;
      uint32_t hbstlen : 4;
      uint32_t dmaen : 1;
      uint32_t reserved_6 : 1;
      uint32_t nptxfemplvl : 1;
      uint32_t ptxfemplvl : 1;
      uint32_t reserved_9 : 23;
    };
    uint32_t val;
  } usb_dwc_gahbcfg_reg_t;

  typedef union
  {
    struct
    {
      uint32_t toutcal : 3;
      uint32_t reserved_3 : 3;
      uint32_t physel : 1;
      uint32_t reserved_7 : 1;
      uint32_t srpcap : 1;
      uint32_t hnpcap : 1;
      uint32_t usbtrdtim : 4;
      uint32_t reserved_14 : 15;
      uint32_t forcehstmode : 1;
      uint32_t forcedevmode : 1;
      uint32_t reserved_31 : 1;
    };
    uint32_t val;
  } usb_dwc_gusbcfg_reg_t;

  typedef union
  {
    struct
    {
      uint32_t csftrst : 1;
      uint32_t reserved_1 : 3;
      uint32_t rxfflsh : 1;
      uint32_t txfflsh : 1;
      uint32_t txfnum : 5;
      uint32_t reserved_11 : 20;
      uint32_t ahbidle : 1;
    };
    uint32_t val;
  } usb_dwc_grstctl_reg_t;

  typedef union
  {
    struct
    {
      uint32_t curmod : 1;
      uint32_t modemis : 1;
      uint32_t otgint : 1;
      uint32_t sof : 1;
      uint32_t rxflvl : 1;
      uint32_t reserved_5 : 1;
      uint32_t ginnakeff : 1;
      uint32_t goutnakeff : 1;
      uint32_t reserved_8 : 2;
      uint32_t erlysusp : 1;
      uint32_t usbsusp : 1;
      uint32_t usbrst : 1;
      uint32_t enumdone : 1;
      uint32_t reserved_14 : 4;
      uint32_t iepint : 1;
      uint32_t oepint : 1;
      uint32_t incompisoin : 1;
      uint32_t incompip : 1;
      uint32_t reserved_22 : 9;
      uint32_t wkupint : 1;
    };
    uint32_t val;
  } usb_dwc_gintsts_reg_t;

  typedef union
  {
    struct
    {
      uint32_t reserved_0 : 2;
      uint32_t otgintmsk : 1;
      uint32_t sofmsk : 1;
      uint32_t rxflvlmsk : 1;
      uint32_t reserved_5 : 6;
      uint32_t usbsuspmsk : 1;
      uint32_t usbrstmsk : 1;
      uint32_t enumdonemsk : 1;
      uint32_t reserved_14 : 4;
      uint32_t iepintmsk : 1;
      uint32_t oepintmsk : 1;
      uint32_t incompisoinmsk : 1;
      uint32_t reserved_21 : 10;
      uint32_t wkupintmsk : 1;
    };
    uint32_t val;
  } usb_dwc_gintmsk_reg_t;

  typedef union
  {
    struct
    {
      uint32_t chnum : 4;
      uint32_t bcnt : 11;
      uint32_t dpid : 2;
      uint32_t pktsts : 4;
      uint32_t fn : 4;
      uint32_t reserved_25 : 7;
    };
    uint32_t val;
  } usb_dwc_grxstsr_reg_t;

  typedef usb_dwc_grxstsr_reg_t usb_dwc_grxstsp_reg_t;

  typedef union
  {
    struct
    {
      uint32_t rxfdep : 16;
      uint32_t reserved_16 : 16;
    };
    uint32_t val;
  } usb_dwc_grxfsiz_reg_t;

  typedef union
  {
    struct
    {
      uint32_t nptxfstaddr : 16;
      uint32_t nptxfdep : 16;
    };
    uint32_t val;
  } usb_dwc_gnptxfsiz_reg_t;

  typedef union
  {
    struct
    {
      uint32_t reserved_0 : 16;
      uint32_t dfifodepth : 16;
    };
    uint32_t val;
  } usb_dwc_ghwcfg3_reg_t;

  typedef union
  {
    struct
    {
      uint32_t inepntxfstaddr : 16;
      uint32_t inepntxfdep : 16;
    };
    uint32_t val;
  } usb_dwc_dieptxf_reg_t;

  typedef union
  {
    struct
    {
      uint32_t devspd : 2;
      uint32_t nzstsouthshk : 1;
      uint32_t reserved_3 : 1;
      uint32_t devaddr : 7;
      uint32_t reserved_11 : 12;
      uint32_t descdma : 1;
      uint32_t reserved_24 : 8;
    };
    uint32_t val;
  } usb_dwc_dcfg_reg_t;

  typedef union
  {
    struct
    {
      uint32_t rmtwkupsig : 1;
      uint32_t sftdiscon : 1;
      uint32_t gnpinnaksts : 1;
      uint32_t goutnaksts : 1;
      uint32_t reserved_4 : 3;
      uint32_t sgnpinnak : 1;
      uint32_t cgnpinnak : 1;
      uint32_t sgoutnak : 1;
      uint32_t cgoutnak : 1;
      uint32_t reserved_11 : 21;
    };
    uint32_t val;
  } usb_dwc_dctl_reg_t;

  typedef union
  {
    struct
    {
      uint32_t suspsts : 1;
      uint32_t enumspd : 2;
      uint32_t reserved_3 : 5;
      uint32_t soffn : 14;
      uint32_t reserved_22 : 10;
    };
    uint32_t val;
  } usb_dwc_dsts_reg_t;

  typedef union
  {
    struct
    {
      uint32_t xfercomplmsk : 1;
      uint32_t epdisbldmsk : 1;
      uint32_t ahberrmsk : 1;
      uint32_t timeoutmsk : 1;
      uint32_t reserved_4 : 2;
      uint32_t inepnakeffmsk : 1;
      uint32_t reserved_7 : 25;
    };
    uint32_t val;
  } usb_dwc_diepmsk_reg_t;

  typedef union
  {
    struct
    {
      uint32_t xfercomplmsk : 1;
      uint32_t epdisbldmsk : 1;
      uint32_t ahberrmsk : 1;
      uint32_t setupmsk : 1;
      uint32_t outtknepdismsk : 1;
      uint32_t stsphsercvdmsk : 1;
      uint32_t back2backsetup : 1;
      uint32_t reserved_7 : 1;
      uint32_t outpkterrmsk : 1;
      uint32_t bnaoutintrmsk : 1;
      uint32_t reserved_10 : 2;
      uint32_t bbleerrmsk : 1;
      uint32_t nakmsk : 1;
      uint32_t nyetmsk : 1;
      uint32_t reserved_15 : 17;
    };
    uint32_t val;
  } usb_dwc_doepmsk_reg_t;

  typedef union
  {
    uint32_t val;
  } usb_dwc_daint_reg_t;

  typedef usb_dwc_daint_reg_t usb_dwc_daintmsk_reg_t;
  typedef usb_dwc_daint_reg_t usb_dwc_diepempmsk_reg_t;

  typedef union
  {
    struct
    {
      uint32_t mps : 2;
      uint32_t reserved_2 : 13;
      uint32_t usbactep : 1;
      uint32_t reserved_16 : 1;
      uint32_t naksts : 1;
      uint32_t eptype : 2;
      uint32_t reserved_20 : 1;
      uint32_t stall : 1;
      uint32_t txfnum : 4;
      uint32_t cnak : 1;
      uint32_t snak : 1;
      uint32_t reserved_28 : 2;
      uint32_t epdis : 1;
      uint32_t epena : 1;
    };
    uint32_t val;
  } usb_dwc_diepctl0_reg_t;

  typedef union
  {
    struct
    {
      uint32_t mps : 11;
      uint32_t reserved_11 : 4;
      uint32_t usbactep : 1;
      uint32_t dpid : 1;
      uint32_t naksts : 1;
      uint32_t eptype : 2;
      uint32_t reserved_20 : 1;
      uint32_t stall : 1;
      uint32_t txfnum : 4;
      uint32_t cnak : 1;
      uint32_t snak : 1;
      uint32_t setd0pid : 1;
      uint32_t setd1pid : 1;
      uint32_t epdis : 1;
      uint32_t epena : 1;
    };
    uint32_t val;
  } usb_dwc_diepctl_reg_t;

  typedef union
  {
    struct
    {
      uint32_t xfercompl : 1;
      uint32_t epdisbld : 1;
      uint32_t ahberr : 1;
      uint32_t timeout : 1;
      uint32_t intkntxfemp : 1;
      uint32_t intknepmis : 1;
      uint32_t inepnakeff : 1;
      uint32_t txfemp : 1;
      uint32_t reserved_8 : 24;
    };
    uint32_t val;
  } usb_dwc_diepint_reg_t;

  typedef usb_dwc_diepint_reg_t usb_dwc_diepint0_reg_t;

  typedef union
  {
    struct
    {
      uint32_t xfersize : 7;
      uint32_t reserved_7 : 12;
      uint32_t pktcnt : 2;
      uint32_t reserved_21 : 11;
    };
    uint32_t val;
  } usb_dwc_dieptsiz0_reg_t;

  typedef union
  {
    struct
    {
      uint32_t xfersize : 19;
      uint32_t pktcnt : 10;
      uint32_t mc : 2;
      uint32_t reserved_31 : 1;
    };
    uint32_t val;
  } usb_dwc_dieptsiz_reg_t;

  typedef union
  {
    struct
    {
      uint32_t dmaaddr : 32;
    };
    uint32_t val;
  } usb_dwc_diepdma0_reg_t;

  typedef union
  {
    struct
    {
      uint32_t dmaddr : 32;
    };
    uint32_t val;
  } usb_dwc_diepdma_reg_t;

  typedef union
  {
    struct
    {
      uint32_t ineptxfspcavail : 16;
      uint32_t reserved_16 : 16;
    };
    uint32_t val;
  } usb_dwc_dtxfsts_reg_t;

  typedef union
  {
    struct
    {
      uint32_t mps : 2;
      uint32_t reserved_2 : 13;
      uint32_t usbactep : 1;
      uint32_t reserved_16 : 1;
      uint32_t naksts : 1;
      uint32_t eptype : 2;
      uint32_t snp : 1;
      uint32_t stall : 1;
      uint32_t reserved_22 : 4;
      uint32_t cnak : 1;
      uint32_t snak : 1;
      uint32_t reserved_28 : 2;
      uint32_t epdis : 1;
      uint32_t epena : 1;
    };
    uint32_t val;
  } usb_dwc_doepctl0_reg_t;

  typedef union
  {
    struct
    {
      uint32_t mps : 11;
      uint32_t reserved_11 : 4;
      uint32_t usbactep : 1;
      uint32_t dpid : 1;
      uint32_t naksts : 1;
      uint32_t eptype : 2;
      uint32_t snp : 1;
      uint32_t stall : 1;
      uint32_t reserved_22 : 4;
      uint32_t cnak : 1;
      uint32_t snak : 1;
      uint32_t setd0pid : 1;
      uint32_t setd1pid : 1;
      uint32_t epdis : 1;
      uint32_t epena : 1;
    };
    uint32_t val;
  } usb_dwc_doepctl_reg_t;

  typedef union
  {
    struct
    {
      uint32_t xfercompl : 1;
      uint32_t epdisbld : 1;
      uint32_t ahberr : 1;
      uint32_t setup : 1;
      uint32_t outtknepdis : 1;
      uint32_t stsphsercvd : 1;
      uint32_t back2backsetup : 1;
      uint32_t reserved_7 : 1;
      uint32_t outpkterr : 1;
      uint32_t bnaintr : 1;
      uint32_t reserved_10 : 1;
      uint32_t pktdrpsts : 1;
      uint32_t bbleerr : 1;
      uint32_t nakintrpt : 1;
      uint32_t nyepintrpt : 1;
      uint32_t stuppktrcvd : 1;
      uint32_t reserved_16 : 16;
    };
    uint32_t val;
  } usb_dwc_doepint_reg_t;

  typedef usb_dwc_doepint_reg_t usb_dwc_doepint0_reg_t;

  typedef union
  {
    struct
    {
      uint32_t xfersize : 7;
      uint32_t reserved_7 : 12;
      uint32_t pktcnt : 1;
      uint32_t reserved_20 : 9;
      uint32_t supcnt : 2;
      uint32_t reserved_31 : 1;
    };
    uint32_t val;
  } usb_dwc_doeptsiz0_reg_t;

  typedef union
  {
    struct
    {
      uint32_t xfersize : 19;
      uint32_t pktcnt : 10;
      uint32_t rxdpid : 2;
      uint32_t reserved_31 : 1;
    };
    uint32_t val;
  } usb_dwc_doeptsiz_reg_t;

  typedef union
  {
    struct
    {
      uint32_t dmaaddr : 32;
    };
    uint32_t val;
  } usb_dwc_doepdma_reg_t;

  typedef usb_dwc_doepdma_reg_t usb_dwc_doepdma0_reg_t;

  typedef union
  {
    struct
    {
      uint32_t stoppclk : 1;
      uint32_t gatehclk : 1;
      uint32_t pwrclmp : 1;
      uint32_t rstpdwnmodule : 1;
      uint32_t reserved_4 : 28;
    };
    uint32_t val;
  } usb_dwc_pcgcctl_reg_t;

  typedef struct
  {
    volatile usb_dwc_diepctl_reg_t diepctl_reg;
    uint32_t reserved_04;
    volatile usb_dwc_diepint_reg_t diepint_reg;
    uint32_t reserved_0c;
    volatile usb_dwc_dieptsiz_reg_t dieptsiz_reg;
    volatile usb_dwc_diepdma_reg_t diepdma_reg;
    volatile usb_dwc_dtxfsts_reg_t dtxfsts_reg;
    uint32_t reserved_1c;
  } usb_dwc_in_ep_regs_t;

  typedef struct
  {
    volatile usb_dwc_doepctl_reg_t doepctl_reg;
    uint32_t reserved_04;
    volatile usb_dwc_doepint_reg_t doepint_reg;
    uint32_t reserved_0c;
    volatile usb_dwc_doeptsiz_reg_t doeptsiz_reg;
    volatile usb_dwc_doepdma_reg_t doepdma_reg;
    uint32_t reserved_18[2];
  } usb_dwc_out_ep_regs_t;

  typedef struct
  {
    volatile usb_dwc_gotgctl_reg_t gotgctl_reg;      // 0x000
    volatile usb_dwc_gotgint_reg_t gotgint_reg;      // 0x004
    volatile usb_dwc_gahbcfg_reg_t gahbcfg_reg;      // 0x008
    volatile usb_dwc_gusbcfg_reg_t gusbcfg_reg;      // 0x00C
    volatile usb_dwc_grstctl_reg_t grstctl_reg;      // 0x010
    volatile usb_dwc_gintsts_reg_t gintsts_reg;      // 0x014
    volatile usb_dwc_gintmsk_reg_t gintmsk_reg;      // 0x018
    volatile usb_dwc_grxstsr_reg_t grxstsr_reg;      // 0x01C
    volatile usb_dwc_grxstsp_reg_t grxstsp_reg;      // 0x020
    volatile usb_dwc_grxfsiz_reg_t grxfsiz_reg;      // 0x024
    volatile usb_dwc_gnptxfsiz_reg_t gnptxfsiz_reg;  // 0x028
    uint32_t reserved_02c[6];                        // 0x02C
    uint32_t reserved_044[2];                        // 0x044: GHWCFG1, GHWCFG2
    volatile usb_dwc_ghwcfg3_reg_t ghwcfg3_reg;      // 0x04C
    uint32_t reserved_050[45];                       // 0x050
    volatile usb_dwc_dieptxf_reg_t dieptxf_regs[4];  // 0x104
    uint32_t reserved_114[443];                      // 0x114
    volatile usb_dwc_dcfg_reg_t dcfg_reg;            // 0x800
    volatile usb_dwc_dctl_reg_t dctl_reg;            // 0x804
    volatile usb_dwc_dsts_reg_t dsts_reg;            // 0x808
    uint32_t reserved_80c;
    volatile usb_dwc_diepmsk_reg_t diepmsk_reg;        // 0x810
    volatile usb_dwc_doepmsk_reg_t doepmsk_reg;        // 0x814
    volatile usb_dwc_daint_reg_t daint_reg;            // 0x818
    volatile usb_dwc_daintmsk_reg_t daintmsk_reg;      // 0x81C
    uint32_t reserved_820[5];                          // 0x820
    volatile usb_dwc_diepempmsk_reg_t diepempmsk_reg;  // 0x834
    uint32_t reserved_838[50];                         // 0x838
    volatile usb_dwc_diepctl0_reg_t diepctl0_reg;      // 0x900
    uint32_t reserved_904;
    volatile usb_dwc_diepint0_reg_t diepint0_reg;  // 0x908
    uint32_t reserved_90c;
    volatile usb_dwc_dieptsiz0_reg_t dieptsiz0_reg;  // 0x910
    volatile usb_dwc_diepdma0_reg_t diepdma0_reg;    // 0x914
    volatile usb_dwc_dtxfsts_reg_t dtxfsts0_reg;     // 0x918
    uint32_t reserved_91c;
    usb_dwc_in_ep_regs_t in_eps[6];                // 0x920
    uint32_t reserved_9e0[72];                     // 0x9E0
    volatile usb_dwc_doepctl0_reg_t doepctl0_reg;  // 0xB00
    uint32_t reserved_b04;
    volatile usb_dwc_doepint0_reg_t doepint0_reg;  // 0xB08
    uint32_t reserved_b0c;
    volatile usb_dwc_doeptsiz0_reg_t doeptsiz0_reg;  // 0xB10
    volatile usb_dwc_doepdma0_reg_t doepdma0_reg;    // 0xB14
    uint32_t reserved_b18[2];
    usb_dwc_out_ep_regs_t out_eps[6];            // 0xB20
    uint32_t reserved_be0[136];                  // 0xBE0
    volatile usb_dwc_pcgcctl_reg_t pcgcctl_reg;  // 0xE00
  } usb_dwc_dev_t;

#ifdef __cplusplus
}

static_assert(offsetof(usb_dwc_dev_t, grxfsiz_reg) == 0x024, "GRXFSIZ");
static_assert(offsetof(usb_dwc_dev_t, ghwcfg3_reg) == 0x04C, "GHWCFG3");
static_assert(offsetof(usb_dwc_dev_t, dieptxf_regs) == 0x104, "DIEPTXF1");
static_assert(offsetof(usb_dwc_dev_t, dcfg_reg) == 0x800, "DCFG");
static_assert(offsetof(usb_dwc_dev_t, diepempmsk_reg) == 0x834, "DIEPEMPMSK");
static_assert(offsetof(usb_dwc_dev_t, diepctl0_reg) == 0x900, "DIEPCTL0");
static_assert(offsetof(usb_dwc_dev_t, in_eps) == 0x920, "DIEPCTL1");
static_assert(offsetof(usb_dwc_dev_t, doepctl0_reg) == 0xB00, "DOEPCTL0");
static_assert(offsetof(usb_dwc_dev_t, out_eps) == 0xB20, "DOEPCTL1");
static_assert(offsetof(usb_dwc_dev_t, pcgcctl_reg) == 0xE00, "PCGCCTL");
static_assert(sizeof(usb_dwc_in_ep_regs_t) == 0x20, "IN endpoint block");
static_assert(sizeof(usb_dwc_out_ep_regs_t) == 0x20, "OUT endpoint block");
#endif
