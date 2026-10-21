#pragma once

#include <cstdint>

#include "ch32_interrupt_guard.h"
#include "ch32_usb.hpp"
#include "ch32_usb_rcc.hpp"
#include "usb/backend/fsdev.hpp"

// Hardware-access traits of the CH32 DevFS (classic FSDEV/PMA) peripheral path.
#if defined(RCC_APB1Periph_USB)

namespace LibXR
{

/**
 * @brief CH32 DevFS 的硬件访问 / Hardware access of the CH32 DevFS
 *
 * 寄存器布局与 STM32 经典型 FSDEV 相同（16 位寄存器、EPnR 位语义、PMA 每 16 位
 * 数据字占两个半字地址、ADDR/COUNT 单位为存储字节）；上拉由 EXTEN_USBD_PU_EN
 * 控制，PMA 上限随 CAN 占用变化（构造时传入）。
 *
 * The register layout matches the STM32 classic FSDEV (16-bit registers, EPnR bit
 * semantics, two half-word addresses per 16-bit PMA word, ADDR/COUNT in stored
 * bytes); the pull-up is EXTEN_USBD_PU_EN and the usable PMA shrinks when CAN is
 * enabled (passed in at construction).
 */
struct Ch32DevFsHw
{
  static constexpr uint32_t kEpCtrRx = 0x8000U;
  static constexpr uint32_t kEpDtogRx = 0x4000U;
  static constexpr uint32_t kEpStatRx = 0x3000U;
  static constexpr uint32_t kEpSetup = 0x0800U;
  static constexpr uint32_t kEpType = 0x0600U;
  static constexpr uint32_t kEpKind = 0x0100U;
  static constexpr uint32_t kEpCtrTx = 0x0080U;
  static constexpr uint32_t kEpDtogTx = 0x0040U;
  static constexpr uint32_t kEpStatTx = 0x0030U;
  static constexpr uint32_t kEpAddr = 0x000FU;
  static constexpr uint32_t kEpErrRx = 0U;
  static constexpr uint32_t kEpErrTx = 0U;
  static constexpr uint32_t kEpKeepMask =
      kEpCtrRx | kEpSetup | kEpType | kEpKind | kEpCtrTx | kEpAddr;

  static constexpr uint16_t kIstrCtr = 0x8000U;
  static constexpr uint16_t kIstrReset = 0x0400U;
  static constexpr uint16_t kIstrSof = 0x0200U;
  static constexpr uint16_t kIstrSusp = 0x0800U;
  static constexpr uint16_t kIstrWkup = 0x1000U;
  static constexpr uint16_t kIstrNoise = 0x7100U;  // ESOF | ERR | PMAOVR
  static constexpr uint16_t kCntrRunMask = 0x8000U | 0x0400U | 0x0800U | 0x1000U;
  static constexpr uint16_t kCntrSofm = 0x0200U;

  static constexpr uint8_t kEndpoints = 8U;
  static constexpr size_t kPmaSize = 512U;
  // 经典型布局：ADDR 字段按偶数截断，缓冲地址必须偶数对齐。
  static constexpr size_t kPmaAlign = 2U;

  static constexpr uintptr_t kRegBase =
#if defined(USB_BASE)
      USB_BASE
#else
      0x40005C00UL
#endif
      ;
  static constexpr uintptr_t kPmaAddr =
#if defined(PMAAddr)
      PMAAddr
#else
      0x40006000UL
#endif
      ;

  static volatile uint16_t* EpReg(uint8_t num)
  {
    // NOLINTNEXTLINE(performance-no-int-to-ptr)
    return reinterpret_cast<volatile uint16_t*>(kRegBase +
                                                static_cast<uintptr_t>(num) * 4U);
  }

  static uint32_t ReadEp(uint8_t num) { return *EpReg(num); }
  static void WriteEp(uint8_t num, uint32_t value)
  {
    *EpReg(num) = static_cast<uint16_t>(value);
  }

  static volatile uint16_t* Cntr()
  {
    // NOLINTNEXTLINE(performance-no-int-to-ptr)
    return reinterpret_cast<volatile uint16_t*>(kRegBase + 0x40U);
  }
  static uint16_t ReadCntr() { return *Cntr(); }
  static void WriteCntr(uint16_t value) { *Cntr() = value; }

  static volatile uint16_t* Istr()
  {
    // NOLINTNEXTLINE(performance-no-int-to-ptr)
    return reinterpret_cast<volatile uint16_t*>(kRegBase + 0x44U);
  }
  static uint16_t ReadIstr() { return *Istr(); }
  static void ClearIstr(uint16_t flags) { *Istr() = static_cast<uint16_t>(~flags); }

  static uint16_t ReadFnr()
  {
    // NOLINTNEXTLINE(performance-no-int-to-ptr)
    return *reinterpret_cast<volatile uint16_t*>(kRegBase + 0x48U);
  }
  static void WriteDaddr(uint16_t value)
  {
    // NOLINTNEXTLINE(performance-no-int-to-ptr)
    *reinterpret_cast<volatile uint16_t*>(kRegBase + 0x4CU) = value;
  }

  static volatile uint16_t* BtField(uint8_t num, unsigned field)
  {
    // BTABLE = 0：字段 m 的半字偏移是 8n+2m，APB 字节偏移再乘 2（每数据字占 4 B）。
    // NOLINTNEXTLINE(performance-no-int-to-ptr)
    return reinterpret_cast<volatile uint16_t*>(
        kPmaAddr + (static_cast<uintptr_t>(num) * 8U + 2U * field) * 2U);
  }

  static void SetTxBuf(uint8_t num, uint16_t addr)
  {
    *BtField(num, 0U) = static_cast<uint16_t>(addr & 0xFFFEU);
  }
  static void SetTxCount(uint8_t num, uint16_t bytes) { *BtField(num, 1U) = bytes; }
  static void SetRxBuf(uint8_t num, uint16_t addr)
  {
    *BtField(num, 2U) = static_cast<uint16_t>(addr & 0xFFFEU);
  }
  static void SetRxCountEncoded(uint8_t num, uint16_t mps)
  {
    *BtField(num, 3U) = EncodeRxCount(mps);
  }
  static uint16_t GetRxCount(uint8_t num)
  {
    return static_cast<uint16_t>(*BtField(num, 3U) & 0x03FFU);
  }

  static uint16_t EncodeRxCount(uint16_t mps)
  {
    if (mps == 0U)
    {
      return 0x8000U;  // BL_SIZE | 0 块 / BL_SIZE with no block
    }
    if (mps <= 62U)
    {
      return static_cast<uint16_t>(((mps + 1U) / 2U) << 10U);
    }
    return static_cast<uint16_t>(0x8000U | ((((mps + 31U) / 32U) - 1U) << 10U));
  }

  static void PmaWrite(uint16_t addr, const uint8_t* src, size_t len)
  {
    // NOLINTNEXTLINE(performance-no-int-to-ptr)
    volatile uint16_t* p = reinterpret_cast<volatile uint16_t*>(
        kPmaAddr + static_cast<uintptr_t>(addr) * 2U);
    for (size_t i = 0; i < len; i += 2)
    {
      uint16_t word = src[i];
      if (i + 1 < len)
      {
        word |= static_cast<uint16_t>(src[i + 1]) << 8U;
      }
      *p = word;
      p += 2;
    }
  }

  static void PmaRead(uint8_t* dst, uint16_t addr, size_t len)
  {
    // NOLINTNEXTLINE(performance-no-int-to-ptr)
    const volatile uint16_t* p = reinterpret_cast<const volatile uint16_t*>(
        kPmaAddr + static_cast<uintptr_t>(addr) * 2U);
    for (size_t i = 0; i < len; i += 2)
    {
      const uint16_t word = *p;
      p += 2;
      dst[i] = static_cast<uint8_t>(word & 0xFFU);
      if (i + 1 < len)
      {
        dst[i + 1] = static_cast<uint8_t>(word >> 8U);
      }
    }
  }

  static void InitRegs()
  {
    WriteCntr(0x0001U);  // FRES：强复位并顺带清 PDWN / Force reset, clears PDWN too
    WriteCntr(0U);
    // 直接写 0 清全部挂起标志；ClearIstr(0) 是写 0xFFFF（保持），语义不同。
    *Istr() = 0U;
    // NOLINTNEXTLINE(performance-no-int-to-ptr)
    *reinterpret_cast<volatile uint16_t*>(kRegBase + 0x50U) = 0U;  // BTABLE = 0
  }

  static void SuspendRegs() { WriteCntr(static_cast<uint16_t>(ReadCntr() | 0x0008U)); }
  static void SuspendClocked() {}
  static void ResumeRegs()
  {
    WriteCntr(static_cast<uint16_t>(ReadCntr() & ~(0x0004U | 0x0008U)));
  }

  static void Connect(bool on)
  {
#if defined(EXTEN_USBD_PU_EN)
    if (on)
    {
      EXTEN->EXTEN_CTR |= EXTEN_USBD_PU_EN;
    }
    else
    {
      EXTEN->EXTEN_CTR &= static_cast<uint16_t>(~EXTEN_USBD_PU_EN);
    }
#else
    (void)on;
#endif
  }

  static void MspInit()
  {
    // FSDEV 使用共享 USB 48 MHz 时钟；如果这个时钟来自 USBHS PHY PLL，
    // 则必须先打开 USBHS 依赖时钟，再打开 USBDEV 本体时钟。
    // FSDEV uses the shared USB 48 MHz clock; when that clock comes from the USBHS
    // PHY PLL, keep the USBHS dependency clock enabled before USBDEV itself.
    CH32UsbRcc::ConfigureUsb48M();
#if defined(RCC_USBCLK48MCLKSource_USBPHY) && defined(RCC_AHBPeriph_USBHS)
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_USBHS, ENABLE);
#endif
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);

#if defined(RCC_APB2Periph_GPIOA) && defined(GPIOA) && defined(GPIO_Pin_11) &&        \
    defined(GPIO_Pin_12) && defined(GPIO_Mode_Out_PP) && defined(GPIO_Speed_50MHz) && \
    defined(GPIO_Mode_IN_FLOATING)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitTypeDef gpio{};
    gpio.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &gpio);
    GPIO_ResetBits(GPIOA, GPIO_Pin_11 | GPIO_Pin_12);

    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &gpio);
#endif
  }

  static uintptr_t EnterGuard() { return libxr_ch32_interrupt_save_and_disable(); }

  static void ExitGuard(uintptr_t saved)
  {
    libxr_ch32_interrupt_restore(static_cast<uint32_t>(saved));
  }
};

}  // namespace LibXR

#endif  // defined(RCC_APB1Periph_USB)
