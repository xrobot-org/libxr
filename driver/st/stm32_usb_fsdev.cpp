#include "stm32_usb_fsdev.hpp"

#include <cstddef>
#include <cstring>

#include "stm32_usb_wrap.hpp"

#if defined(HAL_PCD_MODULE_ENABLED) && (defined(USB_BASE) || defined(USB_DRD_FS))

namespace LibXR
{

namespace
{

using USB::FSDevDevice;

#if !defined(USB_DRD_FS)

// 经典型：寄存器模型就是基类那一份，这里核对设备头文件的偏移和位。
// Classic: the register model is the base class's; the device header's offsets and bits
// are checked here.
static_assert(offsetof(USB_TypeDef, CNTR) == USB::FSDEV_REG_CNTR &&
                  offsetof(USB_TypeDef, ISTR) == USB::FSDEV_REG_ISTR &&
                  offsetof(USB_TypeDef, FNR) == USB::FSDEV_REG_FNR &&
                  offsetof(USB_TypeDef, DADDR) == USB::FSDEV_REG_DADDR &&
                  offsetof(USB_TypeDef, BTABLE) == USB::FSDEV_REG_BTABLE,
              "USB_TypeDef does not match the FSDEV register offsets");
static_assert(USB_EP_CTR_RX == USB::FSDEV_EP_CTR_RX &&
                  USB_EP_SETUP == USB::FSDEV_EP_SETUP &&
                  USB_EP_CTR_TX == USB::FSDEV_EP_CTR_TX &&
                  USB_EPTX_STAT == USB::FSDEV_EP_STAT_TX &&
                  USB_EPRX_STAT == USB::FSDEV_EP_STAT_RX,
              "the EPnR bits of the device header differ from the FSDEV constants");

USB_TypeDef* Regs() { return reinterpret_cast<USB_TypeDef*>(USB_BASE); }

// PMA 里每个 16 位数据字占几个 16 位地址，取 HAL 的 PMA_ACCESS；没有时按手册回退。
// How many 16-bit addresses one data word takes in the PMA, from HAL's PMA_ACCESS;
// without it, as the manuals say.
constexpr uint8_t PMA_WORD_STRIDE =
#if defined(PMA_ACCESS)
    PMA_ACCESS
#elif defined(STM32F1) || defined(STM32L1) || defined(STM32F302xB) ||       \
    defined(STM32F302xC) || defined(STM32F303xB) || defined(STM32F303xC) || \
    defined(STM32F373xC)
    2U
#else
    1U
#endif
    ;

FSDevDevice::MemoryMap ControllerMemoryMap()
{
  return {USB_BASE, USB_BASE + 0x400U, LIBXR_STM32_USB_PMA_SIZE, PMA_WORD_STRIDE, 2U};
}

#else  // defined(USB_DRD_FS)

USB_DRD_TypeDef* Regs() { return USB_DRD_FS; }

/// RXBD 的 BLSIZE 位（RM 给出，H5 的 CMSIS 未给）/ BLSIZE of RXBD (from the RM)
constexpr uint32_t RX_BD_BLSIZE = 0x80000000UL;

#if !defined(USB_DRD_PMAADDR)
#error "USB_DRD_PMAADDR is not defined by the device header of this USB_DRD_FS part."
#endif

FSDevDevice::MemoryMap ControllerMemoryMap()
{
  return {reinterpret_cast<uintptr_t>(Regs()), USB_DRD_PMAADDR, LIBXR_STM32_USB_PMA_SIZE,
          1U, 4U};
}

uint32_t EncodeRxCount(uint16_t mps)
{
  if (mps == 0U)
  {
    return RX_BD_BLSIZE;
  }
  if (mps <= 62U)
  {
    return (static_cast<uint32_t>(mps) + 1U) / 2U << 26U;
  }
  return ((static_cast<uint32_t>(mps) + 31U) / 32U - 1U) << 26U | RX_BD_BLSIZE;
}

/// 端点 num 的缓冲描述符：TXBD 在前，RXBD 在后 / Buffer descriptor of endpoint num
volatile uint32_t* Bd(uintptr_t packet_memory, uint8_t num, bool rx)
{
  // NOLINTNEXTLINE(performance-no-int-to-ptr)
  return reinterpret_cast<volatile uint32_t*>(packet_memory) + num * 2U + (rx ? 1U : 0U);
}

#endif  // !defined(USB_DRD_FS)

}  // namespace

STM32USBDeviceDevFs::STM32USBDeviceDevFs(
    const std::initializer_list<EPConfig>& rx_ep_cfgs,
    const std::initializer_list<EPConfig>& tx_ep_cfgs,
    USB::DeviceDescriptor::PacketSize0 packet_size, uint16_t vid, uint16_t pid,
    uint16_t bcd,
    const std::initializer_list<const USB::DescriptorStrings::LanguagePack*>& languages,
    const std::initializer_list<const std::initializer_list<USB::DeviceClass*>>& configs,
    ConstRawData uid, size_t pma_limit, uint8_t slot_count)
    : USB::FSDevDevice(ControllerMemoryMap(), rx_ep_cfgs, tx_ep_cfgs, packet_size, vid,
                       pid, bcd, languages, configs, uid, pma_limit, slot_count)
{
#if defined(USB_DRD_FS)
  // CHEPnR 高 16 位的 ERR/NAK/DEVADDR/LSEP 写 1 保持（HAL 的 USB_CHEP_REG_MASK）。
  // ERR/NAK/DEVADDR/LSEP in the high half of CHEPnR keep on a written 1 (HAL's
  // USB_CHEP_REG_MASK).
  ep_err_rx_ = USB_CHEP_ERRRX;
  ep_err_tx_ = USB_CHEP_ERRTX;
  ep_keep_mask_ = USB_CHEP_REG_MASK;
#endif

  // 时钟、引脚、NVIC 由 CubeMX 的 HAL_PCD_MspInit 提供；F3 的中断重映射也在其中。
  // Clocks, pins and NVIC come from CubeMX's HAL_PCD_MspInit; so does F3's interrupt
  // remap.
  PCD_HandleTypeDef hpcd = {};
  hpcd.Instance = Regs();
  HAL_PCD_MspInit(&hpcd);
  STM32USBRequireWrap();

#if defined(LIBXR_STM32_FSDEV_IRQS)
  const IRQn_Type lines[] = {LIBXR_STM32_FSDEV_IRQS};
#elif defined(STM32F1)
  // RM0008 p.202、p.625：LP 与 CAN_RX0 共 20，HP 与 CAN_TX 共 19。
  // RM0008 p.202, p.625: LP shares 20 with CAN_RX0, HP shares 19 with CAN_TX.
  const IRQn_Type lines[] = {USB_LP_CAN1_RX0_IRQn, USB_HP_CAN1_TX_IRQn};
#elif defined(STM32F3)
  // RM0316 p.251：USB_IT_RMP=1 时用专用线 74/75，否则与 CAN 共用 19/20。
  // RM0316 p.251: dedicated lines 74/75 with USB_IT_RMP=1, else 19/20 shared with CAN.
  const bool remapped = (SYSCFG->CFGR1 & SYSCFG_CFGR1_USB_IT_RMP) != 0U;
  const IRQn_Type lines[] = {remapped ? USB_LP_IRQn : USB_LP_CAN_RX0_IRQn,
                             remapped ? USB_HP_IRQn : USB_HP_CAN_TX_IRQn};
#elif defined(STM32G4) || defined(STM32WB) || defined(STM32L1)
  const IRQn_Type lines[] = {USB_LP_IRQn, USB_HP_IRQn};
#elif defined(STM32F0) || defined(STM32L0) || defined(STM32L4)
  const IRQn_Type lines[] = {USB_IRQn};
#elif defined(USB_DRD_FS)
  const IRQn_Type lines[] = {USB_DRD_FS_IRQn};
#else
#error "Unknown FSDEV interrupt lines: define LIBXR_STM32_FSDEV_IRQS."
#endif
  static_assert(sizeof(lines) / sizeof(lines[0]) <= 2U, "at most two lines");
  for (const IRQn_Type line : lines)
  {
    lines_[line_count_++] = line;
  }

  // owner 总在控制器中断里（PendInterrupt 挂起 NVIC 线），可以用 EP0 的快速确认和状态
  // OUT 的晚放行。
  // The owner always runs in the controller interrupt (PendInterrupt pends the NVIC
  // line), so the fast acknowledgement of EP0 and the late release of the status OUT
  // may be used.
  EnableEp0FastAck();
  EnableEp0LateStatus();
  // 构造时断开一次（含 D+ 拉低，见类说明），Start 在 DETACH_US 之后接上。
  // Detach once at construction (with D+ held low, see the class notes); Start attaches
  // after DETACH_US.
  DetachAtConstruction();
}

#if LIBXR_STM32_FSDEV_DP_DETACH
namespace
{
/// USB 外设时钟：开 / 关；本系列没有 HAL 宏时什么也不做 / USB peripheral clock on / off;
/// nothing where the family has no HAL macro
bool UsbClock(bool on)
{
#if defined(__HAL_RCC_USB_CLK_ENABLE) && defined(__HAL_RCC_USB_CLK_DISABLE)
  if (on)
  {
    __HAL_RCC_USB_CLK_ENABLE();
  }
  else
  {
    __HAL_RCC_USB_CLK_DISABLE();
  }
  return true;
#elif defined(__HAL_RCC_USB_FS_CLK_ENABLE) && defined(__HAL_RCC_USB_FS_CLK_DISABLE)
  if (on)
  {
    __HAL_RCC_USB_FS_CLK_ENABLE();
  }
  else
  {
    __HAL_RCC_USB_FS_CLK_DISABLE();
  }
  return true;
#else
  UNUSED(on);
  return false;
#endif
}
}  // namespace
#endif

bool STM32USBDeviceDevFs::HoldDetached()
{
#if LIBXR_STM32_FSDEV_DP_DETACH
  // 收发器断电，再关时钟；寄存器和 PMA 保留。 / Transceiver powered down, then the clock
  // off; registers and PMA are kept.
  WriteCntr(static_cast<uint16_t>(USB::FSDEV_CNTR_FRES | USB::FSDEV_CNTR_PDWN));
  const bool clock_off = UsbClock(false);
  __HAL_RCC_GPIOA_CLK_ENABLE();
  // 先把输出锁存写 0，再切成推挽输出，切换时不会先出一个高电平。 / The output latch is
  // written 0 before the switch to push-pull output, so no high level appears first.
#if defined(GPIO_CRH_MODE12)
  // F1：PA12 是 CRH 的 [19:16]（MODE、CNF）。 / F1: PA12 is CRH[19:16] (MODE, CNF).
  dp_saved_[0] = GPIOA->CRH & (0xFUL << 16U);
  dp_saved_[1] = GPIOA->ODR & (1UL << 12U);
  GPIOA->BSRR = 1UL << (12U + 16U);
  GPIOA->CRH = (GPIOA->CRH & ~(0xFUL << 16U)) | (0x2UL << 16U);  // 推挽输出 2 MHz / push-pull out
#else
  dp_saved_[0] = GPIOA->MODER & (3UL << 24U);
  dp_saved_[1] = GPIOA->ODR & (1UL << 12U);
  dp_saved_[2] = GPIOA->OTYPER & (1UL << 12U);
  dp_saved_[3] = GPIOA->PUPDR & (3UL << 24U);
  dp_saved_[4] = GPIOA->AFR[1] & (0xFUL << 16U);
  GPIOA->BSRR = 1UL << (12U + 16U);
  GPIOA->OTYPER &= ~(1UL << 12U);
  GPIOA->PUPDR &= ~(3UL << 24U);
  GPIOA->MODER = (GPIOA->MODER & ~(3UL << 24U)) | (1UL << 24U);  // 通用输出 / output
#endif
  UNUSED(clock_off);
  return true;
#else
  return false;
#endif
}

void STM32USBDeviceDevFs::ReleaseDetached()
{
#if LIBXR_STM32_FSDEV_DP_DETACH
  // 引脚先回到 USB 用的配置，再开时钟；InitController 随后清 PDWN。 / The pin returns to
  // its USB configuration first, then the clock comes on; InitController clears PDWN
  // afterwards.
#if defined(GPIO_CRH_MODE12)
  GPIOA->CRH = (GPIOA->CRH & ~(0xFUL << 16U)) | dp_saved_[0];
  GPIOA->BSRR = dp_saved_[1] != 0U ? (1UL << 12U) : (1UL << (12U + 16U));
#else
  GPIOA->AFR[1] = (GPIOA->AFR[1] & ~(0xFUL << 16U)) | dp_saved_[4];
  GPIOA->PUPDR = (GPIOA->PUPDR & ~(3UL << 24U)) | dp_saved_[3];
  GPIOA->OTYPER = (GPIOA->OTYPER & ~(1UL << 12U)) | dp_saved_[2];
  GPIOA->BSRR = dp_saved_[1] != 0U ? (1UL << 12U) : (1UL << (12U + 16U));
  GPIOA->MODER = (GPIOA->MODER & ~(3UL << 24U)) | dp_saved_[0];
#endif
  (void)UsbClock(true);
#endif
}

void STM32USBDeviceDevFs::MaskInterrupt()
{
  for (uint8_t i = 0U; i < line_count_; ++i)
  {
    NVIC_DisableIRQ(lines_[i]);
  }
}

void STM32USBDeviceDevFs::UnmaskInterrupt()
{
  for (uint8_t i = 0U; i < line_count_; ++i)
  {
    NVIC_EnableIRQ(lines_[i]);
  }
}

bool STM32USBDeviceDevFs::PendInterrupt()
{
  NVIC_SetPendingIRQ(lines_[0]);
  return true;
}

#if !defined(USB_DRD_FS)

void STM32USBDeviceDevFs::Connect(bool on)
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
#elif defined(SYSCFG_PMC_USB_PU)
  // L1：内部上拉是 SYSCFG_PMC 的 USB_PU 位 / L1: the pull-up is USB_PU of SYSCFG_PMC
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  if (on)
  {
    SET_BIT(SYSCFG->PMC, SYSCFG_PMC_USB_PU);
  }
  else
  {
    CLEAR_BIT(SYSCFG->PMC, SYSCFG_PMC_USB_PU);
  }
#else
  // F1/F3 没有内部上拉，由用户的 HAL_PCDEx_SetConnectionState 控制。
  // F1/F3 have no internal pull-up; the user's HAL_PCDEx_SetConnectionState controls it.
  PCD_HandleTypeDef hpcd = {};
  hpcd.Instance = Regs();
  HAL_PCDEx_SetConnectionState(&hpcd, on ? 1U : 0U);
#endif
}

#else  // defined(USB_DRD_FS)

void STM32USBDeviceDevFs::Connect(bool on)
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

uint32_t STM32USBDeviceDevFs::ReadRegister(uint32_t offset)
{
  // NOLINTNEXTLINE(performance-no-int-to-ptr)
  return *reinterpret_cast<volatile uint32_t*>(memory_map_.registers + offset);
}

void STM32USBDeviceDevFs::WriteRegister(uint32_t offset, uint32_t value)
{
  // NOLINTNEXTLINE(performance-no-int-to-ptr)
  *reinterpret_cast<volatile uint32_t*>(memory_map_.registers + offset) = value;
}

void STM32USBDeviceDevFs::SetTxBuf(uint8_t num, uint16_t addr)
{
  volatile uint32_t* bd = Bd(memory_map_.packet_memory, num, false);
  *bd = (*bd & 0xFFFF0000U) | (addr & 0xFFFCU);
}

void STM32USBDeviceDevFs::SetTxCount(uint8_t num, uint16_t bytes)
{
  volatile uint32_t* bd = Bd(memory_map_.packet_memory, num, false);
  *bd = (*bd & 0x0000FFFFU) | (static_cast<uint32_t>(bytes) << 16U);
}

void STM32USBDeviceDevFs::SetRxBuf(uint8_t num, uint16_t addr)
{
  volatile uint32_t* bd = Bd(memory_map_.packet_memory, num, true);
  *bd = (*bd & 0xFFFF0000U) | (addr & 0xFFFCU);
}

void STM32USBDeviceDevFs::SetRxCountEncoded(uint8_t num, uint16_t mps)
{
  volatile uint32_t* bd = Bd(memory_map_.packet_memory, num, true);
  *bd = (*bd & 0x0000FFFFU) | EncodeRxCount(mps);
}

uint16_t STM32USBDeviceDevFs::GetRxCount(uint8_t num)
{
  // 接收后 RXBD 计数要过几个周期才更新；HAL 的 PCD_GET_EP_RX_CNT 同样先空转。
  // The RXBD count settles a few cycles after a reception; HAL's PCD_GET_EP_RX_CNT spins
  // first too.
  volatile uint32_t settle = PCD_RX_PMA_CNT;
  while (settle > 0U)
  {
    settle = settle - 1U;
  }
  return static_cast<uint16_t>((*Bd(memory_map_.packet_memory, num, true) >> 16U) &
                               0x03FFU);
}

// 等时双缓冲：缓冲 0 是 TXBD，缓冲 1 是 RXBD（与经典型的发送、接收位置对应）。
// Isochronous double buffer: buffer 0 is the TXBD, buffer 1 the RXBD (the transmit and
// receive locations of the classic layout).
void STM32USBDeviceDevFs::SetIsoTxCount(uint8_t num, uint8_t buffer, uint16_t bytes)
{
  volatile uint32_t* bd = Bd(memory_map_.packet_memory, num, buffer == 1U);
  *bd = (*bd & 0x0000FFFFU) | (static_cast<uint32_t>(bytes & 0x03FFU) << 16U);
}

void STM32USBDeviceDevFs::SetIsoRxSize(uint8_t num, uint8_t buffer, uint16_t mps)
{
  volatile uint32_t* bd = Bd(memory_map_.packet_memory, num, buffer == 1U);
  *bd = (*bd & 0x0000FFFFU) | EncodeRxCount(mps);
}

uint16_t STM32USBDeviceDevFs::GetIsoRxCount(uint8_t num, uint8_t buffer)
{
  // 与 GetRxCount 一样先空转，等计数更新。 / Spin first as GetRxCount does, until the
  // count settles.
  volatile uint32_t settle = PCD_RX_PMA_CNT;
  while (settle > 0U)
  {
    settle = settle - 1U;
  }
  return static_cast<uint16_t>(
      (*Bd(memory_map_.packet_memory, num, buffer == 1U) >> 16U) & 0x03FFU);
}

void STM32USBDeviceDevFs::PmaWrite(uint16_t addr, const uint8_t* src, size_t len)
{
  // NOLINTNEXTLINE(performance-no-int-to-ptr)
  volatile uint32_t* p =
      reinterpret_cast<volatile uint32_t*>(memory_map_.packet_memory + addr);
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

void STM32USBDeviceDevFs::PmaRead(uint8_t* dst, uint16_t addr, size_t len)
{
  // NOLINTNEXTLINE(performance-no-int-to-ptr)
  const volatile uint32_t* p =
      reinterpret_cast<const volatile uint32_t*>(memory_map_.packet_memory + addr);
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

void STM32USBDeviceDevFs::InitRegs()
{
  Regs()->CNTR = USB_CNTR_USBRST;
  Regs()->CNTR &= ~USB_CNTR_USBRST;
  Regs()->CNTR &= ~USB_CNTR_HOST;
  Regs()->ISTR = 0U;
}

void STM32USBDeviceDevFs::SuspendRegs() { Regs()->CNTR |= USB_CNTR_SUSPEN; }

void STM32USBDeviceDevFs::SuspendClocked() { Regs()->CNTR |= USB_CNTR_SUSPRDY; }

void STM32USBDeviceDevFs::ResumeRegs()
{
  Regs()->CNTR &= ~USB_CNTR_SUSPRDY;
  Regs()->CNTR &= ~USB_CNTR_SUSPEN;
}

#endif  // !defined(USB_DRD_FS)

}  // namespace LibXR

extern "C" void libxr_stm32_fsdev_dispatch(PCD_HandleTypeDef*)
{
  LibXR::STM32USBDeviceDevFs::Dispatch();
}

#endif
