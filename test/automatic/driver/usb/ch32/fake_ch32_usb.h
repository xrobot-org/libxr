// CH32 USB 后端主机测试用的假 CH32V30x 配置头文件：USBHS 和 USBFS 设备寄存器块做成
// 普通内存（INT_FG 写 1 清），给出后端用到的 ch32v30x_usb.h 位、PFIC 和 RCC 函数。
// Fake CH32V30x configuration header for host tests of the CH32 USB backend: the
// USBHS and USBFS device register blocks as plain memory (INT_FG clears on writing 1),
// with the ch32v30x_usb.h bits, PFIC and RCC functions the backend uses.
#pragma once

#include <cstddef>
#include <cstdint>

#define __IO volatile

/// 写 1 清的 8 位标志寄存器；可设“第 n 次读时置位”模拟写寄存器期间完成的事务
/// 8-bit flag register cleared by writing 1; "set on the n-th read" emulates a
/// transaction completing while registers are written
/// 第几次读时置位（0 关闭）与置哪些位；单字节布局不变，所以放在外面
/// On which read to set (0 = off) and which bits; kept outside to keep the one-byte
/// layout
inline int fake_int_fg_reads_until_set = 0;
inline uint8_t fake_int_fg_set_bits = 0U;
/// 非零时：下一次写 1 清 UIF_TRANSFER 之后，把这个值写进 target 指向的 INT_ST（一次性）
/// When non-zero: after the next write-1-clear of UIF_TRANSFER this value is stored in
/// the INT_ST that target points at (once)
inline uint8_t fake_int_fg_after_clear_st = 0U;
inline volatile uint8_t* fake_int_st_after_clear_target = nullptr;

struct FakeW1C8
{
  volatile uint8_t value;
  operator uint8_t() const
  {
    if (fake_int_fg_reads_until_set > 0 && --fake_int_fg_reads_until_set == 0)
    {
      const_cast<FakeW1C8*>(this)->value =
          static_cast<uint8_t>(value | fake_int_fg_set_bits);
    }
    return value;
  }
  FakeW1C8& operator=(uint8_t bits)
  {
    value = static_cast<uint8_t>(value & ~bits);
    if ((bits & 0x02U) != 0U && fake_int_fg_after_clear_st != 0U &&
        fake_int_st_after_clear_target != nullptr)
    {
      *fake_int_st_after_clear_target = fake_int_fg_after_clear_st;
      fake_int_fg_after_clear_st = 0U;
    }
    return *this;
  }
};

typedef struct
{
  __IO uint8_t CONTROL;
  __IO uint8_t HOST_CTRL;
  __IO uint8_t INT_EN;
  __IO uint8_t DEV_AD;
  __IO uint16_t FRAME_NO;
  __IO uint8_t SUSPEND;
  __IO uint8_t RESERVED0;
  __IO uint8_t SPEED_TYPE;
  __IO uint8_t MIS_ST;
  FakeW1C8 INT_FG;
  __IO uint8_t INT_ST;
  __IO uint16_t RX_LEN;
  __IO uint16_t RESERVED1;
  __IO uint32_t ENDP_CONFIG;
  __IO uint32_t ENDP_TYPE;
  __IO uint32_t BUF_MODE;
  __IO uint32_t UEP0_DMA;
  __IO uint32_t UEP1_RX_DMA;
  __IO uint32_t UEP2_RX_DMA;
  __IO uint32_t UEP3_RX_DMA;
  __IO uint32_t UEP4_RX_DMA;
  __IO uint32_t UEP5_RX_DMA;
  __IO uint32_t UEP6_RX_DMA;
  __IO uint32_t UEP7_RX_DMA;
  __IO uint32_t UEP8_RX_DMA;
  __IO uint32_t UEP9_RX_DMA;
  __IO uint32_t UEP10_RX_DMA;
  __IO uint32_t UEP11_RX_DMA;
  __IO uint32_t UEP12_RX_DMA;
  __IO uint32_t UEP13_RX_DMA;
  __IO uint32_t UEP14_RX_DMA;
  __IO uint32_t UEP15_RX_DMA;
  __IO uint32_t UEP1_TX_DMA;
  __IO uint32_t UEP2_TX_DMA;
  __IO uint32_t UEP3_TX_DMA;
  __IO uint32_t UEP4_TX_DMA;
  __IO uint32_t UEP5_TX_DMA;
  __IO uint32_t UEP6_TX_DMA;
  __IO uint32_t UEP7_TX_DMA;
  __IO uint32_t UEP8_TX_DMA;
  __IO uint32_t UEP9_TX_DMA;
  __IO uint32_t UEP10_TX_DMA;
  __IO uint32_t UEP11_TX_DMA;
  __IO uint32_t UEP12_TX_DMA;
  __IO uint32_t UEP13_TX_DMA;
  __IO uint32_t UEP14_TX_DMA;
  __IO uint32_t UEP15_TX_DMA;
  __IO uint16_t UEP0_MAX_LEN;
  __IO uint16_t RESERVED2;
  __IO uint16_t UEP1_MAX_LEN;
  __IO uint16_t RESERVED3;
  __IO uint16_t UEP2_MAX_LEN;
  __IO uint16_t RESERVED4;
  __IO uint16_t UEP3_MAX_LEN;
  __IO uint16_t RESERVED5;
  __IO uint16_t UEP4_MAX_LEN;
  __IO uint16_t RESERVED6;
  __IO uint16_t UEP5_MAX_LEN;
  __IO uint16_t RESERVED7;
  __IO uint16_t UEP6_MAX_LEN;
  __IO uint16_t RESERVED8;
  __IO uint16_t UEP7_MAX_LEN;
  __IO uint16_t RESERVED9;
  __IO uint16_t UEP8_MAX_LEN;
  __IO uint16_t RESERVED10;
  __IO uint16_t UEP9_MAX_LEN;
  __IO uint16_t RESERVED11;
  __IO uint16_t UEP10_MAX_LEN;
  __IO uint16_t RESERVED12;
  __IO uint16_t UEP11_MAX_LEN;
  __IO uint16_t RESERVED13;
  __IO uint16_t UEP12_MAX_LEN;
  __IO uint16_t RESERVED14;
  __IO uint16_t UEP13_MAX_LEN;
  __IO uint16_t RESERVED15;
  __IO uint16_t UEP14_MAX_LEN;
  __IO uint16_t RESERVED16;
  __IO uint16_t UEP15_MAX_LEN;
  __IO uint16_t RESERVED17;
  __IO uint16_t UEP0_TX_LEN;
  __IO uint8_t UEP0_TX_CTRL;
  __IO uint8_t UEP0_RX_CTRL;
  __IO uint16_t UEP1_TX_LEN;
  __IO uint8_t UEP1_TX_CTRL;
  __IO uint8_t UEP1_RX_CTRL;
  __IO uint16_t UEP2_TX_LEN;
  __IO uint8_t UEP2_TX_CTRL;
  __IO uint8_t UEP2_RX_CTRL;
  __IO uint16_t UEP3_TX_LEN;
  __IO uint8_t UEP3_TX_CTRL;
  __IO uint8_t UEP3_RX_CTRL;
  __IO uint16_t UEP4_TX_LEN;
  __IO uint8_t UEP4_TX_CTRL;
  __IO uint8_t UEP4_RX_CTRL;
  __IO uint16_t UEP5_TX_LEN;
  __IO uint8_t UEP5_TX_CTRL;
  __IO uint8_t UEP5_RX_CTRL;
  __IO uint16_t UEP6_TX_LEN;
  __IO uint8_t UEP6_TX_CTRL;
  __IO uint8_t UEP6_RX_CTRL;
  __IO uint16_t UEP7_TX_LEN;
  __IO uint8_t UEP7_TX_CTRL;
  __IO uint8_t UEP7_RX_CTRL;
  __IO uint16_t UEP8_TX_LEN;
  __IO uint8_t UEP8_TX_CTRL;
  __IO uint8_t UEP8_RX_CTRL;
  __IO uint16_t UEP9_TX_LEN;
  __IO uint8_t UEP9_TX_CTRL;
  __IO uint8_t UEP9_RX_CTRL;
  __IO uint16_t UEP10_TX_LEN;
  __IO uint8_t UEP10_TX_CTRL;
  __IO uint8_t UEP10_RX_CTRL;
  __IO uint16_t UEP11_TX_LEN;
  __IO uint8_t UEP11_TX_CTRL;
  __IO uint8_t UEP11_RX_CTRL;
  __IO uint16_t UEP12_TX_LEN;
  __IO uint8_t UEP12_TX_CTRL;
  __IO uint8_t UEP12_RX_CTRL;
  __IO uint16_t UEP13_TX_LEN;
  __IO uint8_t UEP13_TX_CTRL;
  __IO uint8_t UEP13_RX_CTRL;
  __IO uint16_t UEP14_TX_LEN;
  __IO uint8_t UEP14_TX_CTRL;
  __IO uint8_t UEP14_RX_CTRL;
  __IO uint16_t UEP15_TX_LEN;
  __IO uint8_t UEP15_TX_CTRL;
  __IO uint8_t UEP15_RX_CTRL;
} USBHSD_TypeDef;

typedef struct
{
  __IO uint8_t BASE_CTRL;
  __IO uint8_t UDEV_CTRL;
  __IO uint8_t INT_EN;
  __IO uint8_t DEV_ADDR;
  __IO uint8_t Reserve0;
  __IO uint8_t MIS_ST;
  FakeW1C8 INT_FG;
  __IO uint8_t INT_ST;
  __IO uint16_t RX_LEN;
  __IO uint16_t Reserve1;
  __IO uint8_t UEP4_1_MOD;
  __IO uint8_t UEP2_3_MOD;
  __IO uint8_t UEP5_6_MOD;
  __IO uint8_t UEP7_MOD;
  __IO uint32_t UEP0_DMA;
  __IO uint32_t UEP1_DMA;
  __IO uint32_t UEP2_DMA;
  __IO uint32_t UEP3_DMA;
  __IO uint32_t UEP4_DMA;
  __IO uint32_t UEP5_DMA;
  __IO uint32_t UEP6_DMA;
  __IO uint32_t UEP7_DMA;
  __IO uint16_t UEP0_TX_LEN;
  __IO uint8_t UEP0_TX_CTRL;
  __IO uint8_t UEP0_RX_CTRL;
  __IO uint16_t UEP1_TX_LEN;
  __IO uint8_t UEP1_TX_CTRL;
  __IO uint8_t UEP1_RX_CTRL;
  __IO uint16_t UEP2_TX_LEN;
  __IO uint8_t UEP2_TX_CTRL;
  __IO uint8_t UEP2_RX_CTRL;
  __IO uint16_t UEP3_TX_LEN;
  __IO uint8_t UEP3_TX_CTRL;
  __IO uint8_t UEP3_RX_CTRL;
  __IO uint16_t UEP4_TX_LEN;
  __IO uint8_t UEP4_TX_CTRL;
  __IO uint8_t UEP4_RX_CTRL;
  __IO uint16_t UEP5_TX_LEN;
  __IO uint8_t UEP5_TX_CTRL;
  __IO uint8_t UEP5_RX_CTRL;
  __IO uint16_t UEP6_TX_LEN;
  __IO uint8_t UEP6_TX_CTRL;
  __IO uint8_t UEP6_RX_CTRL;
  __IO uint16_t UEP7_TX_LEN;
  __IO uint8_t UEP7_TX_CTRL;
  __IO uint8_t UEP7_RX_CTRL;
  __IO uint32_t Reserve2;
  __IO uint32_t OTG_CR;
  __IO uint32_t OTG_SR;
} USBFSD_TypeDef;

inline USBHSD_TypeDef fake_usbhsd{};
inline USBFSD_TypeDef fake_usbfsd{};
#define USBHSD (&fake_usbhsd)
#define USBFSD (&fake_usbfsd)
#define USBFSH (&fake_usbfsd)

// ---- PFIC ----
enum IRQn_Type
{
  USBFS_IRQn = 83,
  USBHS_IRQn = 85
};

/// 假 PFIC：每条线的使能、挂起，以及屏蔽次数 / Fake PFIC: enable and pending per line,
/// and the number of masks
struct FakePfic
{
  bool enabled[128];
  bool pending[128];
  unsigned disables;
};
inline FakePfic fake_pfic{};
inline void NVIC_EnableIRQ(IRQn_Type irq) { fake_pfic.enabled[irq] = true; }
inline void NVIC_DisableIRQ(IRQn_Type irq)
{
  fake_pfic.enabled[irq] = false;
  ++fake_pfic.disables;
}
inline void NVIC_SetPendingIRQ(IRQn_Type irq) { fake_pfic.pending[irq] = true; }

// ---- RCC ----
#define ENABLE 1
#define RCC_AHBPeriph_USBHS 0x00000800U
#define RCC_AHBPeriph_USBFS 0x00001000U
#define RCC_USBCLKSource_PLLCLK_Div1 0U
#define RCC_USBCLKSource_PLLCLK_Div2 1U
#define RCC_USBCLKSource_PLLCLK_Div3 2U
typedef struct
{
  uint32_t SYSCLK_Frequency;
} RCC_ClocksTypeDef;
inline void RCC_GetClocksFreq(RCC_ClocksTypeDef* clocks)
{
  clocks->SYSCLK_Frequency = 144000000U;
}
inline void RCC_USBCLKConfig(uint32_t) {}
inline void RCC_AHBPeriphClockCmd(uint32_t, int) {}

// ---- USBHS (ch32v30x_usb.h) ----
#define USBHS_UC_SPEED_HIGH 0x20
#define USBHS_UC_DEV_PU_EN 0x10
#define USBHS_UC_INT_BUSY 0x08
#define USBHS_UC_RESET_SIE 0x04
#define USBHS_UC_CLR_ALL 0x02
#define USBHS_UC_DMA_EN 0x01
#define USBHS_UIE_SETUP_ACT 0x20
#define USBHS_UIE_SOF_ACT 0x08
#define USBHS_UIE_SUSPEND 0x04
#define USBHS_UIE_TRANSFER 0x02
#define USBHS_UIE_DETECT 0x01
#define USBHS_MICRO_FRAME_NUM 0xE000
#define USBHS_SOF_FRAME_NUM 0x07FF
#define USBHS_USB_SPEED_TYPE 0x03
#define USBHS_USB_SPEED_FULL 0x00
#define USBHS_USB_SPEED_HIGH 0x01
#define USBHS_UMS_SIE_FREE 0x20
#define USBHS_UMS_BUS_RESET 0x08
#define USBHS_UMS_SUSPEND 0x04
#define USBHS_UIF_SETUP_ACT 0x20
#define USBHS_UIF_SUSPEND 0x04
#define USBHS_UIF_TRANSFER 0x02
#define USBHS_UIF_BUS_RST 0x01
#define USBHS_UIS_IS_NAK 0x80
#define USBHS_UIS_TOG_OK 0x40
#define USBHS_UIS_TOKEN_MASK 0x30
#define USBHS_UIS_TOKEN_OUT 0x00
#define USBHS_UIS_TOKEN_SOF 0x10
#define USBHS_UIS_TOKEN_IN 0x20
#define USBHS_UIS_TOKEN_SETUP 0x30
#define USBHS_UIS_ENDP_MASK 0x0F
#define USBHS_UH_PHY_SUSPENDM 0x10
#define USBHS_UEP0_R_EN 0x00010000
#define USBHS_UEP15_R_EN 0x80000000
#define USBHS_UEP0_T_EN 0x00000001
#define USBHS_UEP15_T_EN 0x00008000
#define USBHS_UEP0_R_TYPE 0x00010000
#define USBHS_UEP0_T_TYPE 0x00000001
#define USBHS_UEP_R_RES_ACK 0x00
#define USBHS_UEP_R_RES_MASK 0x03
#define USBHS_UEP_R_RES_NAK 0x02
#define USBHS_UEP_R_RES_STALL 0x03
#define USBHS_UEP_R_TOG_DATA1 0x08
#define USBHS_UEP_R_TOG_MDATA 0x18
#define USBHS_UEP_R_TOG_MASK 0x18
#define USBHS_UEP_T_RES_ACK 0x00
#define USBHS_UEP_T_RES_MASK 0x03
#define USBHS_UEP_T_RES_NAK 0x02
#define USBHS_UEP_T_RES_STALL 0x03
#define USBHS_UEP_T_TOG_AUTO 0x20
#define USBHS_UEP_T_TOG_DATA0 0x00
#define USBHS_UEP_T_TOG_DATA1 0x08
#define USBHS_UEP_T_TOG_MASK 0x18
#define USBHS_UEP_T_TOG_MDATA 0x18

// ---- USBFS (ch32v30x_usb.h) ----
#define USBFS_UC_DEV_PU_EN 0x20
#define USBFS_UC_INT_BUSY 0x08
#define USBFS_UC_RESET_SIE 0x04
#define USBFS_UC_CLR_ALL 0x02
#define USBFS_UC_DMA_EN 0x01
#define USBFS_UIE_DEV_SOF 0x80
#define USBFS_UIE_HST_SOF 0x08
#define USBFS_UIE_SUSPEND 0x04
#define USBFS_UIE_TRANSFER 0x02
#define USBFS_UIE_BUS_RST 0x01
#define USBFS_UDA_GP_BIT 0x80
#define USBFS_UMS_SIE_FREE 0x20
#define USBFS_UMS_BUS_RESET 0x08
#define USBFS_UMS_SUSPEND 0x04
#define USBFS_UIF_FIFO_OV 0x10
#define USBFS_UIF_HST_SOF 0x08
#define USBFS_UIF_SUSPEND 0x04
#define USBFS_UIF_TRANSFER 0x02
#define USBFS_UIF_DETECT 0x01
#define USBFS_UIF_BUS_RST 0x01
#define USBFS_UIS_IS_NAK 0x80
#define USBFS_UIS_TOG_OK 0x40
#define USBFS_UIS_TOKEN_MASK 0x30
#define USBFS_UIS_TOKEN_OUT 0x00
#define USBFS_UIS_TOKEN_SOF 0x10
#define USBFS_UIS_TOKEN_IN 0x20
#define USBFS_UIS_TOKEN_SETUP 0x30
#define USBFS_UIS_ENDP_MASK 0x0F
#define USBFS_UD_PD_DIS 0x80
#define USBFS_UD_PORT_EN 0x01
#define USBFS_UEP1_RX_EN 0x80
#define USBFS_UEP1_TX_EN 0x40
#define USBFS_UEP1_BUF_MOD 0x10
#define USBFS_UEP4_RX_EN 0x08
#define USBFS_UEP4_TX_EN 0x04
#define USBFS_UEP4_BUF_MOD 0x01
#define USBFS_UEP3_RX_EN 0x80
#define USBFS_UEP3_TX_EN 0x40
#define USBFS_UEP3_BUF_MOD 0x10
#define USBFS_UEP2_RX_EN 0x08
#define USBFS_UEP2_TX_EN 0x04
#define USBFS_UEP2_BUF_MOD 0x01
#define USBFS_UEP6_RX_EN 0x80
#define USBFS_UEP6_TX_EN 0x40
#define USBFS_UEP6_BUF_MOD 0x10
#define USBFS_UEP5_RX_EN 0x08
#define USBFS_UEP5_TX_EN 0x04
#define USBFS_UEP5_BUF_MOD 0x01
#define USBFS_UEP7_RX_EN 0x08
#define USBFS_UEP7_TX_EN 0x04
#define USBFS_UEP7_BUF_MOD 0x01
#define USBFS_UEP_T_AUTO_TOG 0x08
#define USBFS_UEP_T_TOG 0x04
#define USBFS_UEP_T_RES_MASK 0x03
#define USBFS_UEP_T_RES_ACK 0x00
#define USBFS_UEP_T_RES_NONE 0x01
#define USBFS_UEP_T_RES_NAK 0x02
#define USBFS_UEP_T_RES_STALL 0x03
#define USBFS_UEP_R_AUTO_TOG 0x08
#define USBFS_UEP_R_TOG 0x04
#define USBFS_UEP_R_RES_MASK 0x03
#define USBFS_UEP_R_RES_ACK 0x00
#define USBFS_UEP_R_RES_NONE 0x01
#define USBFS_UEP_R_RES_NAK 0x02
#define USBFS_UEP_R_RES_STALL 0x03
