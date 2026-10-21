#pragma once

#include "main.h"

// 先记下控制器是否存在：`USB` 宏（寄存器指针）与命名空间 `LibXR::USB` 冲突，下面要
// #undef 它。守卫必须在 #undef 之前求值，否则永远为假（F1/F103 就是这样被整体跳过的）。
// Record the controller first: the `USB` macro (register pointer) collides with the
// `LibXR::USB` namespace and is undefined below, so the guard has to be evaluated
// before that.
#if defined(USB) || defined(USB_DRD_FS)
#define LIBXR_STM32_USB_FSDEV_PRESENT 1
#endif

#undef USB

#include "usb/backend/fsdev.hpp"

#if defined(HAL_PCD_MODULE_ENABLED) && defined(LIBXR_STM32_USB_FSDEV_PRESENT)

namespace LibXR
{

// PMA sizes of the classic FSDEV controllers and of USB_DRD_FS. F1/F3 with CAN share
// the packet memory; the share is decided at build time the same way the old driver
// did. Define LIBXR_STM32_USB_PMA_SIZE manually for an unknown part number.
#if defined(PMA_END_ADDR)
#define LIBXR_STM32_USB_PMA_SIZE PMA_END_ADDR
// --- F0: USB FS Device, PMA = 1024B (dedicated) -------------------------
#elif defined(STM32F0)
#define LIBXR_STM32_USB_PMA_SIZE 1024u

// --- F1: F102/F103 USB FS Device, PMA = 512B，与 CAN 共用，不能并行使用 ---
#elif defined(STM32F102xB) || defined(STM32F102xC) || defined(STM32F103x6) || \
    defined(STM32F103xB) || defined(STM32F103xE) || defined(STM32F103xG)
#define LIBXR_STM32_USB_PMA_SIZE 512u

#if defined(HAL_CAN_MODULE_ENABLED)
#error "STM32F102/F103 cannot use CAN at the same time with USB."
#endif

// --- F3 小容量带 USB：PMA = 512B，专用给 USB --------------------
#elif defined(STM32F302xB) || defined(STM32F302xC) || defined(STM32F303xB) || \
    defined(STM32F303xC) || defined(STM32F373xC)
#define LIBXR_STM32_USB_PMA_SIZE 512u

// --- F3 大容量（和 CAN 共享 1KB）：F302x6/8,D/E & F303xD/E -------
// - 总共 1KB USB+CAN SRAM
// - 若 CAN 在用：USB 只能用前 768B，最后 256B 给 CAN
#elif defined(STM32F302x6) || defined(STM32F302x8) || defined(STM32F302xD) || \
    defined(STM32F302xE) || defined(STM32F303xD) || defined(STM32F303xE)

#if defined(HAL_CAN_MODULE_ENABLED)
// 给 USB 768B
#define LIBXR_STM32_USB_PMA_SIZE 768u
#else
// 整 1KB 给 USB
#define LIBXR_STM32_USB_PMA_SIZE 1024u
#endif

// --- L0: USB FS Device, PMA = 1024B (dedicated) -------------------------
#elif defined(STM32L0)
#define LIBXR_STM32_USB_PMA_SIZE 1024u

// --- L1: USB FS Device, PMA = 512B (dedicated) --------------------------
#elif defined(STM32L1)
#define LIBXR_STM32_USB_PMA_SIZE 512u

// --- G4: USB FS Device (V1), PMA = 1024B (dedicated) --------------------
#elif defined(STM32G4)
#define LIBXR_STM32_USB_PMA_SIZE 1024u

// --- G0: USB_DRD_FS (V2), PMA = 2048B (dedicated) -----------------------
#elif defined(STM32G0)
#define LIBXR_STM32_USB_PMA_SIZE 2048u

// --- C0: USB_DRD_FS (V2), PMA = 2048B (dedicated) -----------------------
#elif defined(STM32C0)
#define LIBXR_STM32_USB_PMA_SIZE 2048u

// --- H5（H503/563/573/562 DRD_FS），PMA = 2048B -------------------------
#elif defined(STM32H503xx) || defined(STM32H563xx) || defined(STM32H573xx) || \
    defined(STM32H562xx)
#define LIBXR_STM32_USB_PMA_SIZE 2048u

// --- WB55/35: USB FS Device, PMA = 1024B (专门给 USB) ------------------
#elif defined(STM32WB)
#define LIBXR_STM32_USB_PMA_SIZE 1024u

// --- L4: USB FS Device (非 OTG)，PMA = 1024B ----------------------------
#elif defined(STM32L4)
#define LIBXR_STM32_USB_PMA_SIZE 1024u

// --- L5: USB FS Device/DRD，PMA = 1024B -------------------------------
#elif defined(STM32L5)
#define LIBXR_STM32_USB_PMA_SIZE 1024u

// --- U5: USB_DRD_FS (V2)，PMA = 2048B -----------------------------------
#elif defined(STM32U5)
#define LIBXR_STM32_USB_PMA_SIZE 2048u

// --- U0: USB_DRD_FS (V2，1KB 专用) --------------------------------------
#elif defined(STM32U0)
#define LIBXR_STM32_USB_PMA_SIZE 1024u

// --- U3: USB_DRD_FS (V2，2KB 专用) --------------------------------------
#elif defined(STM32U3)
#define LIBXR_STM32_USB_PMA_SIZE 2048u

#else
#error \
    "Unknown STM32 USB FS/DRD with PMA. Please define LIBXR_STM32_USB_PMA_SIZE manually."
#endif

/// 链接期确认 HAL_PCD_IRQHandler 已被 --wrap 接管（stm32_usb_wrap.cpp）。
/// Checks at link time that HAL_PCD_IRQHandler is wrapped by stm32_usb_wrap.cpp.
void STM32FsDevRequireWrap();

/**
 * @brief 经典型 FSDEV 的硬件访问（F1/F3/F0/L0/L1/G4/WB/L4/L5）
 *        Hardware access of the classic FSDEV (F1/F3/F0/L0/L1/G4/WB/L4/L5)
 *
 * 寄存器 16 位；PMA 以半字访问，每 16 位数据字占 `PMA_ACCESS` 个半字地址
 * （F1/F3 部分 = 2，其余 = 1）。ADDR/COUNT 单位是存储字节。
 *
 * 16-bit registers; the PMA is accessed as half words with every 16-bit data word
 * occupying `PMA_ACCESS` half-word addresses (F1/F3 parts = 2, the rest = 1). ADDR and
 * COUNT are in stored bytes.
 */
// H5（USB_DRD_FS）的 CMSIS 不再提供经典型宏名，经典 traits 只在非 DRD 芯片编译。
#if !defined(USB_DRD_FS)
struct Stm32FsDevClassicHw
{
  static constexpr uint32_t kEpCtrRx = USB_EP_CTR_RX;
  static constexpr uint32_t kEpDtogRx = USB_EP_DTOG_RX;
  static constexpr uint32_t kEpStatRx = USB_EPRX_STAT;
  static constexpr uint32_t kEpSetup = USB_EP_SETUP;
  static constexpr uint32_t kEpType = USB_EP_T_FIELD;
  static constexpr uint32_t kEpKind = USB_EP_KIND;
  static constexpr uint32_t kEpCtrTx = USB_EP_CTR_TX;
  static constexpr uint32_t kEpDtogTx = USB_EP_DTOG_TX;
  static constexpr uint32_t kEpStatTx = USB_EPTX_STAT;
  static constexpr uint32_t kEpAddr = USB_EPADDR_FIELD;
  static constexpr uint32_t kEpErrRx = 0U;
  static constexpr uint32_t kEpErrTx = 0U;
  // 经典型没有 ERR 位：CTR/SETUP/类型/KIND/地址写 1 保持。
  static constexpr uint32_t kEpKeepMask =
      kEpCtrRx | kEpSetup | kEpType | kEpKind | kEpCtrTx | kEpAddr;

  static constexpr uint16_t kIstrCtr = USB_ISTR_CTR;
  static constexpr uint16_t kIstrReset = USB_ISTR_RESET;
  static constexpr uint16_t kIstrSof = USB_ISTR_SOF;
  static constexpr uint16_t kIstrSusp = USB_ISTR_SUSP;
  static constexpr uint16_t kIstrWkup = USB_ISTR_WKUP;
  static constexpr uint16_t kIstrNoise = USB_ISTR_ESOF | USB_ISTR_ERR | USB_ISTR_PMAOVR;
  static constexpr uint16_t kCntrRunMask =
      USB_CNTR_CTRM | USB_CNTR_RESETM | USB_CNTR_SUSPM | USB_CNTR_WKUPM;
  static constexpr uint16_t kCntrSofm = USB_CNTR_SOFM;

  static constexpr uint8_t kEndpoints = 8U;
  static constexpr size_t kPmaSize = LIBXR_STM32_USB_PMA_SIZE;
  // 经典型 ADDR 字段按偶数截断（& 0xFFFE），布局必须保证缓冲地址偶数对齐。
  static constexpr size_t kPmaAlign = 2U;

  static USB_TypeDef* Regs() { return reinterpret_cast<USB_TypeDef*>(USB_BASE); }

  static volatile uint16_t* EpReg(uint8_t num)
  {
    return reinterpret_cast<volatile uint16_t*>(USB_BASE +
                                                static_cast<uintptr_t>(num) * 4U);
  }

  static uint32_t ReadEp(uint8_t num) { return *EpReg(num); }
  static void WriteEp(uint8_t num, uint32_t value)
  {
    *EpReg(num) = static_cast<uint16_t>(value);
  }
  static uint16_t ReadCntr() { return Regs()->CNTR; }
  static void WriteCntr(uint16_t value) { Regs()->CNTR = value; }
  static uint16_t ReadIstr() { return Regs()->ISTR; }
  static void ClearIstr(uint16_t flags) { Regs()->ISTR = static_cast<uint16_t>(~flags); }
  static uint16_t ReadFnr() { return Regs()->FNR; }
  static void WriteDaddr(uint16_t value) { Regs()->DADDR = value; }

  static uintptr_t PmaBase() { return reinterpret_cast<uintptr_t>(Regs()) + 0x400U; }

  static volatile uint16_t* BtField(uint8_t num, unsigned field)
  {
    // BTABLE = 0：字段 m 的半字偏移是 8n+2m，APB 字节偏移再乘 PMA_ACCESS（HAL 同式）。
    return reinterpret_cast<volatile uint16_t*>(
        PmaBase() + (static_cast<uintptr_t>(num) * 8U + 2U * field) * kPmaAccess);
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

  /// COUNT_RX 块编码：≤62 B 按 2 B 块，>62 B 按 32 B 块 / COUNT_RX block encoding:
  /// blocks of 2 B up to 62 B, then blocks of 32 B
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
    volatile uint16_t* p = reinterpret_cast<volatile uint16_t*>(
        PmaBase() + static_cast<uintptr_t>(addr) * kPmaAccess);
    for (size_t i = 0; i < len; i += 2)
    {
      uint16_t word = src[i];
      if (i + 1 < len)
      {
        word |= static_cast<uint16_t>(src[i + 1]) << 8U;
      }
      *p = word;
      p += kPmaAccess;
    }
  }

  static void PmaRead(uint8_t* dst, uint16_t addr, size_t len)
  {
    const volatile uint16_t* p = reinterpret_cast<const volatile uint16_t*>(
        PmaBase() + static_cast<uintptr_t>(addr) * kPmaAccess);
    for (size_t i = 0; i < len; i += 2)
    {
      const uint16_t word = *p;
      p += kPmaAccess;
      dst[i] = static_cast<uint8_t>(word & 0xFFU);
      if (i + 1 < len)
      {
        dst[i + 1] = static_cast<uint8_t>(word >> 8U);
      }
    }
  }

  static void InitRegs()
  {
    Regs()->CNTR = USB_CNTR_FRES;  // 强复位并顺带清 PDWN / Force reset, clears PDWN too
    Regs()->CNTR = 0U;
    Regs()->ISTR = 0U;
    Regs()->BTABLE = 0U;
  }

  static void SuspendRegs() { Regs()->CNTR |= USB_CNTR_FSUSP; }
  static void SuspendClocked() {}
  static void ResumeRegs()
  {
    Regs()->CNTR &= static_cast<uint16_t>(~(USB_CNTR_LP_MODE | USB_CNTR_FSUSP));
  }

  static void Connect(bool on)
  {
#if defined(USB_BCDR_DPPU)
    if (on)
    {
      Regs()->BCDR |= USB_BCDR_DPPU;
    }
    else
    {
      Regs()->BCDR &= static_cast<uint16_t>(~USB_BCDR_DPPU);
    }
#else
    // F1/F3 没有内部上拉：上电时序由用户实现的 HAL_PCDEx_SetConnectionState 控制。
    PCD_HandleTypeDef hpcd = {};
    hpcd.Instance = Regs();
    HAL_PCDEx_SetConnectionState(&hpcd, on ? 1U : 0U);
#endif
  }

  static void MspInit()
  {
    // 时钟、引脚与 NVIC 由 CubeMX 的 HAL_PCD_MspInit 提供；句柄只读 Instance。
    PCD_HandleTypeDef hpcd = {};
    hpcd.Instance = Regs();
    HAL_PCD_MspInit(&hpcd);
    STM32FsDevRequireWrap();
  }

  static uintptr_t EnterGuard()
  {
    const uint32_t saved = __get_PRIMASK();
    __disable_irq();
    __DMB();
    return saved;
  }

  static void ExitGuard(uintptr_t saved)
  {
    __DMB();
    __set_PRIMASK(static_cast<uint32_t>(saved));
  }

 private:
  // HAL 的 PMA_ACCESS：F1/F3 的 HAL 不提供这个宏，而它们的 PMA 与 F1 一样按半字
  // 间隔 2 访问（PMA 与 CAN 共享 SRAM 的那批 IP），其余经典型为 1。
  // HAL's PMA_ACCESS: the F1/F3 HAL does not define it, yet their PMA is accessed with
  // half-word stride 2 like F1 (the IP whose PMA is shared with CAN); the other classic
  // parts use 1.
  static constexpr unsigned kPmaAccess =
#if defined(PMA_ACCESS)
      PMA_ACCESS
#elif defined(STM32F1) || defined(STM32F3)
      2U
#else
      1U
#endif
      ;
};
#endif  // !defined(USB_DRD_FS)

#ifdef __DOXYGEN__
/**
 * @brief DRD（USB_DRD_FS）的硬件访问变体 / Hardware access of the DRD variant
 * (USB_DRD_FS)
 *
 * CHEP 寄存器 32 位，低 16 位与经典型一致，高位是 ERR/NAK/DEVADDR/LSEP（写 1
 * 保持）；CNTR/ISTR 32 位（挂起位为 SUSPEN/SUSPRDY）；BTABLE 为固定在 PMA 0 的
 * 32 位 TXBD/RXBD；PMA 以 32 位字访问、字节偏移 1:1。
 *
 * The CHEP registers are 32-bit, the low half identical to the classic layout with
 * ERR/NAK/DEVADDR/LSEP in the high half (write 1 to keep); CNTR/ISTR are 32-bit with
 * SUSPEN/SUSPRDY; the BTABLE is the 32-bit TXBD/RXBD table fixed at PMA 0; the PMA is
 * accessed as 32-bit words with a 1:1 byte offset.
 */
using Stm32FsDevDrdHw = Stm32FsDevClassicHw;
#endif

#if defined(USB_DRD_FS)

struct Stm32FsDevDrdHw
{
  // H5 的 CMSIS 把 RXBD BLSIZE 移进了 LL USB 头；这里按 RM 直接取 RXBD count bit31。
  static constexpr uint32_t kRxBdBlsize = 0x80000000UL;
  static constexpr uint32_t kEpCtrRx = USB_CHEP_VTRX;
  static constexpr uint32_t kEpDtogRx = USB_CHEP_DTOG_RX;
  static constexpr uint32_t kEpStatRx = USB_CHEP_RX_STRX;
  static constexpr uint32_t kEpSetup = USB_CHEP_SETUP;
  static constexpr uint32_t kEpType = USB_CHEP_UTYPE;
  static constexpr uint32_t kEpKind = USB_CHEP_KIND;
  static constexpr uint32_t kEpCtrTx = USB_CHEP_VTTX;
  static constexpr uint32_t kEpDtogTx = USB_CHEP_DTOG_TX;
  static constexpr uint32_t kEpStatTx = USB_CHEP_TX_STTX;
  static constexpr uint32_t kEpAddr = USB_CHEP_ADDR;
  static constexpr uint32_t kEpErrRx = USB_CHEP_ERRRX;
  static constexpr uint32_t kEpErrTx = USB_CHEP_ERRTX;
  // HAL USB_CHEP_REG_MASK：VTRX/VTTX/ERR/NAK/LSEP/DEVADDR 写 1 保持。
  static constexpr uint32_t kEpKeepMask = USB_CHEP_REG_MASK;

  static constexpr uint16_t kIstrCtr = USB_ISTR_CTR;
  static constexpr uint16_t kIstrReset = USB_ISTR_RESET;
  static constexpr uint16_t kIstrSof = USB_ISTR_SOF;
  static constexpr uint16_t kIstrSusp = USB_ISTR_SUSP;
  static constexpr uint16_t kIstrWkup = USB_ISTR_WKUP;
  static constexpr uint16_t kIstrNoise = USB_ISTR_ESOF | USB_ISTR_ERR | USB_ISTR_PMAOVR;
  static constexpr uint16_t kCntrRunMask =
      USB_CNTR_CTRM | USB_CNTR_RESETM | USB_CNTR_SUSPM | USB_CNTR_WKUPM;
  static constexpr uint16_t kCntrSofm = USB_CNTR_SOFM;

  static constexpr uint8_t kEndpoints = 8U;
  static constexpr size_t kPmaSize = LIBXR_STM32_USB_PMA_SIZE;
  // DRD 的 TXBD/RXBD 地址字段按 4 字节截断（& 0xFFFC），布局必须 4 字节对齐。
  static constexpr size_t kPmaAlign = 4U;

  static USB_DRD_TypeDef* Regs()
  {
#if defined(USB_DRD_FS_BASE)  // H5 只有 USB_DRD_FS_BASE，G0/C0/U5 两者都有
    return reinterpret_cast<USB_DRD_TypeDef*>(USB_DRD_FS_BASE);
#else
    return reinterpret_cast<USB_DRD_TypeDef*>(USB_BASE);
#endif
  }

  static volatile uint32_t* Chep(uint8_t num) { return &Regs()->CHEP0R + num; }

  static uint32_t ReadEp(uint8_t num) { return *Chep(num); }
  static void WriteEp(uint8_t num, uint32_t value) { *Chep(num) = value; }
  static uint16_t ReadCntr() { return static_cast<uint16_t>(Regs()->CNTR); }
  static void WriteCntr(uint16_t value) { Regs()->CNTR = value; }
  static uint16_t ReadIstr() { return static_cast<uint16_t>(Regs()->ISTR); }
  static void ClearIstr(uint16_t flags)
  {
    Regs()->ISTR = static_cast<uint32_t>(~flags);  // 高位写 1 保持
  }
  static uint16_t ReadFnr() { return static_cast<uint16_t>(Regs()->FNR); }
  static void WriteDaddr(uint16_t value) { Regs()->DADDR = value; }

  static uintptr_t PmaBase()
  {
#if defined(USB_DRD_PMAADDR)
    return USB_DRD_PMAADDR;
#else
    return reinterpret_cast<uintptr_t>(Regs()) + 0x800U;
#endif
  }

  static volatile uint32_t* Bd(uint8_t num, bool rx)
  {
    return reinterpret_cast<volatile uint32_t*>(PmaBase()) + num * 2U + (rx ? 1U : 0U);
  }

  static void SetTxBuf(uint8_t num, uint16_t addr)
  {
    volatile uint32_t* bd = Bd(num, false);
    *bd = (*bd & 0xFFFF0000U) | (addr & 0xFFFCU);
  }
  static void SetTxCount(uint8_t num, uint16_t bytes)
  {
    volatile uint32_t* bd = Bd(num, false);
    *bd = (*bd & 0x0000FFFFU) | (static_cast<uint32_t>(bytes) << 16U);
  }
  static void SetRxBuf(uint8_t num, uint16_t addr)
  {
    volatile uint32_t* bd = Bd(num, true);
    *bd = (*bd & 0xFFFF0000U) | (addr & 0xFFFCU);
  }
  static void SetRxCountEncoded(uint8_t num, uint16_t mps)
  {
    volatile uint32_t* bd = Bd(num, true);
    *bd = (*bd & 0x0000FFFFU) | EncodeRxCount(mps);
  }
  static uint16_t GetRxCount(uint8_t num)
  {
    return static_cast<uint16_t>((*Bd(num, true) >> 16U) & 0x03FFU);
  }

  static uint32_t EncodeRxCount(uint16_t mps)
  {
    if (mps == 0U)
    {
      return kRxBdBlsize;
    }
    if (mps <= 62U)
    {
      return (static_cast<uint32_t>(mps) + 1U) / 2U << 26U;
    }
    return ((static_cast<uint32_t>(mps) + 31U) / 32U - 1U) << 26U | kRxBdBlsize;
  }

  // PMA 只支持 32 位访问；尾部的 1–3 字节单独处理（HAL 同式）。
  static void PmaWrite(uint16_t addr, const uint8_t* src, size_t len)
  {
    volatile uint32_t* p = reinterpret_cast<volatile uint32_t*>(PmaBase() + addr);
    const size_t words = len / 4U;
    for (size_t i = 0; i < words; ++i)
    {
      uint32_t value;
      std::memcpy(&value, src + i * 4U, 4U);
      *p++ = value;
    }
    const size_t rest = len % 4U;
    if (rest != 0U)
    {
      uint32_t value = 0U;
      std::memcpy(&value, src + words * 4U, rest);
      *p = value;
    }
  }

  static void PmaRead(uint8_t* dst, uint16_t addr, size_t len)
  {
    const volatile uint32_t* p =
        reinterpret_cast<const volatile uint32_t*>(PmaBase() + addr);
    const size_t words = len / 4U;
    for (size_t i = 0; i < words; ++i)
    {
      const uint32_t value = *p++;
      std::memcpy(dst + i * 4U, &value, 4U);
    }
    const size_t rest = len % 4U;
    if (rest != 0U)
    {
      const uint32_t value = *p;
      std::memcpy(dst + words * 4U, &value, rest);
    }
  }

  static void InitRegs()
  {
    Regs()->CNTR = USB_CNTR_USBRST;
    Regs()->CNTR &= ~USB_CNTR_USBRST;
    Regs()->CNTR &= ~USB_CNTR_HOST;  // 设备模式
    Regs()->ISTR = 0U;
  }

  static void SuspendRegs() { Regs()->CNTR |= USB_CNTR_SUSPEN; }
  static void SuspendClocked() { Regs()->CNTR |= USB_CNTR_SUSPRDY; }
  static void ResumeRegs()
  {
    Regs()->CNTR &= ~USB_CNTR_SUSPRDY;
    Regs()->CNTR &= ~USB_CNTR_SUSPEN;
  }

  static void Connect(bool on)
  {
    if (on)
    {
      Regs()->BCDR |= USB_BCDR_DPPU;
    }
    else
    {
      Regs()->BCDR &= ~USB_BCDR_DPPU;
    }
  }

  static void MspInit()
  {
    PCD_HandleTypeDef hpcd = {};
    hpcd.Instance = Regs();
    HAL_PCD_MspInit(&hpcd);
    STM32FsDevRequireWrap();
  }

  static uintptr_t EnterGuard()
  {
    const uint32_t saved = __get_PRIMASK();
    __disable_irq();
    __DMB();
    return saved;
  }

  static void ExitGuard(uintptr_t saved)
  {
    __DMB();
    __set_PRIMASK(static_cast<uint32_t>(saved));
  }
};

using Stm32FsDevHw = Stm32FsDevDrdHw;

#else

using Stm32FsDevHw = Stm32FsDevClassicHw;

#endif

/**
 * @brief STM32 FSDEV / DRD USB 设备 / STM32 FSDEV or DRD USB device
 *
 * 不使用 PCD 句柄：构造时借临时句柄调用 CubeMX 的 HAL_PCD_MspInit()（时钟、引脚、
 * NVIC），之后寄存器全部由共用运行期（usb/backend/fsdev.hpp）直接读写。CubeMX 保持
 * 默认设置：链接时包装 HAL_PCD_Init() 与 HAL_PCD_IRQHandler()，main() 里的
 * MX_USB_PCD_Init() 不再初始化控制器，生成的中断函数进入 Dispatch()。
 *
 * No PCD handle is used: the constructor calls the CubeMX HAL_PCD_MspInit() (clocks,
 * pins, NVIC) through a temporary handle, and from then on the shared runtime
 * (usb/backend/fsdev.hpp) programs the registers directly. CubeMX keeps its default
 * settings: the link wraps HAL_PCD_Init() and HAL_PCD_IRQHandler(), so
 * MX_USB_PCD_Init() no longer initializes the controller and the generated interrupt
 * handler enters Dispatch().
 */
class STM32USBDeviceDevFs : public USB::FSDevDevice<Stm32FsDevHw>
{
 public:
  using USB::FSDevDevice<Stm32FsDevHw>::FSDevDevice;
};

}  // namespace LibXR

#endif
