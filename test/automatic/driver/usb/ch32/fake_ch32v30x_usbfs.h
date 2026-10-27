// Fake CH32V30x configuration header for host tests of the CH32 USBFS endpoint: the
// USBOTG_FS device register block of ch32v30x.h as plain memory, and the register bits
// of ch32v30x_usb.h the endpoint uses.
#pragma once

#include <cstddef>
#include <cstdint>

#define __IO volatile

typedef struct
{
  __IO uint8_t BASE_CTRL;
  __IO uint8_t UDEV_CTRL;
  __IO uint8_t INT_EN;
  __IO uint8_t DEV_ADDR;
  __IO uint8_t Reserve0;
  __IO uint8_t MIS_ST;
  __IO uint8_t INT_FG;
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

inline USBFSD_TypeDef fake_usbfsd{};
#define USBFSD (&fake_usbfsd)

#define USBFS_U_TOG_OK 0x40
#define USBFS_UIF_TRANSFER 0x02
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
