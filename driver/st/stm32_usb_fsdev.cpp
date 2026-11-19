#include "stm32_usb_fsdev.hpp"

#include <cstddef>

#include "stm32_usb_wrap.hpp"

#if defined(HAL_PCD_MODULE_ENABLED) && (defined(USB_BASE) || defined(USB_DRD_FS))

namespace LibXR
{

namespace
{

uintptr_t EnterGuard(void*)
{
  const uint32_t saved = __get_PRIMASK();
  __disable_irq();
  __DMB();
  return saved;
}

void LeaveGuard(void*, uintptr_t saved)
{
  __DMB();
  __set_PRIMASK(static_cast<uint32_t>(saved));
}

#if !defined(USB_DRD_FS)

// 经典型（F1/F3/F0/L0/L1/G4/WB/L4/L5）：寄存器模型就是基类里的那一份。这里核对设备
// 头文件给出的偏移和位与基类的常量一致。
// Classic controllers (F1/F3/F0/L0/L1/G4/WB/L4/L5): the register model is the one of the
// base class. The offsets and bits the device header gives are checked against the
// constants of the base class here.
static_assert(offsetof(USB_TypeDef, CNTR) == USB::FSDEV_REG_CNTR &&
                  offsetof(USB_TypeDef, ISTR) == USB::FSDEV_REG_ISTR &&
                  offsetof(USB_TypeDef, FNR) == USB::FSDEV_REG_FNR &&
                  offsetof(USB_TypeDef, DADDR) == USB::FSDEV_REG_DADDR &&
                  offsetof(USB_TypeDef, BTABLE) == USB::FSDEV_REG_BTABLE,
              "USB_TypeDef does not match the FSDEV register offsets");
static_assert(
    USB_EP_CTR_RX == USB::FSDEV_EP_CTR_RX && USB_EP_DTOG_RX == USB::FSDEV_EP_DTOG_RX &&
        USB_EPRX_STAT == USB::FSDEV_EP_STAT_RX && USB_EP_SETUP == USB::FSDEV_EP_SETUP &&
        USB_EP_T_FIELD == USB::FSDEV_EP_TYPE && USB_EP_KIND == USB::FSDEV_EP_KIND &&
        USB_EP_CTR_TX == USB::FSDEV_EP_CTR_TX &&
        USB_EP_DTOG_TX == USB::FSDEV_EP_DTOG_TX &&
        USB_EPTX_STAT == USB::FSDEV_EP_STAT_TX && USB_EPADDR_FIELD == USB::FSDEV_EP_ADDR,
    "the EPnR bits of the device header differ from the FSDEV constants");
static_assert(USB_ISTR_CTR == USB::FSDEV_ISTR_CTR &&
                  USB_ISTR_RESET == USB::FSDEV_ISTR_RESET &&
                  USB_ISTR_SOF == USB::FSDEV_ISTR_SOF &&
                  USB_ISTR_SUSP == USB::FSDEV_ISTR_SUSP &&
                  USB_ISTR_WKUP == USB::FSDEV_ISTR_WKUP &&
                  (USB_ISTR_ESOF | USB_ISTR_ERR | USB_ISTR_PMAOVR) ==
                      USB::FSDEV_ISTR_NOISE,
              "the ISTR flags of the device header differ from the FSDEV constants");
// CNTR 的低功耗位：F1 的设备头文件叫 USB_CNTR_LP_MODE，其余经典型叫 USB_CNTR_LPMODE
// （F3 两个名字都有）。
// The low-power bit of CNTR: the F1 device header calls it USB_CNTR_LP_MODE, the other
// classic parts USB_CNTR_LPMODE (F3 has both names).
#if defined(USB_CNTR_LPMODE)
static_assert(USB_CNTR_LPMODE == USB::FSDEV_CNTR_LPMODE, "CNTR low-power bit differs");
#else
static_assert(USB_CNTR_LP_MODE == USB::FSDEV_CNTR_LPMODE, "CNTR low-power bit differs");
#endif
static_assert((USB_CNTR_CTRM | USB_CNTR_RESETM | USB_CNTR_SUSPM | USB_CNTR_WKUPM) ==
                      USB::FSDEV_CNTR_RUN_MASK &&
                  USB_CNTR_SOFM == USB::FSDEV_CNTR_SOFM &&
                  USB_CNTR_FSUSP == USB::FSDEV_CNTR_FSUSP &&
                  USB_CNTR_FRES == USB::FSDEV_CNTR_FRES,
              "the CNTR bits of the device header differ from the FSDEV constants");

USB_TypeDef* Regs() { return reinterpret_cast<USB_TypeDef*>(USB_BASE); }

// PMA 里每个 16 位数据字占几个 16 位地址，取 HAL 的 PMA_ACCESS。HAL 没有给出这个宏时
// 按手册回退：每个 32 位字只用 16 位的（间隔 2）是 F1、L1 以及 F3 的 xB/xC 和 F373xC
// （RM0365 表 172、RM0316 表 178 的“1 x 16 bits / word”），其余经典型两个半字都用
// （间隔 1）。
// How many 16-bit addresses one 16-bit data word takes in the PMA, taken from HAL's
// PMA_ACCESS. Without the macro the value falls back as the manuals say: only 16 bits of
// every 32-bit word are used (stride 2) on F1, L1 and the F3 xB/xC and F373xC parts
// ("1 x 16 bits / word" in RM0365 table 172 and RM0316 table 178); the other classic
// parts use both half words (stride 1).
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

USB::FSDevDevice::MemoryMap ControllerMemoryMap()
{
  // 经典型的 PMA 紧跟在寄存器块之后 0x400 处；缓冲地址字段按偶数截断，所以按 2 字节对齐。
  // The PMA of the classic controllers follows the register block at 0x400; the buffer
  // address field is truncated to even values, hence the 2-byte alignment.
  return {USB_BASE, USB_BASE + 0x400U, LIBXR_STM32_USB_PMA_SIZE, PMA_WORD_STRIDE, 2U};
}

#else  // defined(USB_DRD_FS)

// DRD：寄存器偏移和低 16 位的位与经典型相同，这里核对；32 位访问、BTABLE、PMA 和
// 挂起序列由本文件后面的覆盖函数实现。
// DRD: the register offsets and the bits of the low half are those of the classic
// controllers, which is checked here; the 32-bit access, the BTABLE, the PMA and the
// suspend sequence are implemented by the overrides further down in this file.
static_assert(offsetof(USB_DRD_TypeDef, CNTR) == USB::FSDEV_REG_CNTR &&
                  offsetof(USB_DRD_TypeDef, ISTR) == USB::FSDEV_REG_ISTR &&
                  offsetof(USB_DRD_TypeDef, FNR) == USB::FSDEV_REG_FNR &&
                  offsetof(USB_DRD_TypeDef, DADDR) == USB::FSDEV_REG_DADDR,
              "USB_DRD_TypeDef does not match the FSDEV register offsets");
static_assert(
    USB_CHEP_VTRX == USB::FSDEV_EP_CTR_RX && USB_CHEP_DTOG_RX == USB::FSDEV_EP_DTOG_RX &&
        USB_CHEP_RX_STRX == USB::FSDEV_EP_STAT_RX &&
        USB_CHEP_SETUP == USB::FSDEV_EP_SETUP && USB_CHEP_UTYPE == USB::FSDEV_EP_TYPE &&
        USB_CHEP_KIND == USB::FSDEV_EP_KIND && USB_CHEP_VTTX == USB::FSDEV_EP_CTR_TX &&
        USB_CHEP_DTOG_TX == USB::FSDEV_EP_DTOG_TX &&
        USB_CHEP_TX_STTX == USB::FSDEV_EP_STAT_TX && USB_CHEP_ADDR == USB::FSDEV_EP_ADDR,
    "the CHEPnR bits of the device header differ from the FSDEV constants");
static_assert(USB_ISTR_CTR == USB::FSDEV_ISTR_CTR &&
                  USB_ISTR_RESET == USB::FSDEV_ISTR_RESET &&
                  USB_ISTR_SOF == USB::FSDEV_ISTR_SOF &&
                  USB_ISTR_SUSP == USB::FSDEV_ISTR_SUSP &&
                  USB_ISTR_WKUP == USB::FSDEV_ISTR_WKUP &&
                  (USB_ISTR_ESOF | USB_ISTR_ERR | USB_ISTR_PMAOVR) ==
                      USB::FSDEV_ISTR_NOISE,
              "the ISTR flags of the device header differ from the FSDEV constants");
static_assert((USB_CNTR_CTRM | USB_CNTR_RESETM | USB_CNTR_SUSPM | USB_CNTR_WKUPM) ==
                      USB::FSDEV_CNTR_RUN_MASK &&
                  USB_CNTR_SOFM == USB::FSDEV_CNTR_SOFM,
              "the CNTR bits of the device header differ from the FSDEV constants");

/// 控制器的寄存器块；各系列的设备头文件都定义了 USB_DRD_FS，基地址宏的名字则不统一
/// Register block of the controller. Every family's device header defines USB_DRD_FS,
/// while the name of the base-address macro differs between families.
USB_DRD_TypeDef* Regs() { return USB_DRD_FS; }

// H5 的 CMSIS 把 RXBD BLSIZE 移进了 LL USB 头；这里按 RM 直接取 RXBD count bit31。
// The H5 CMSIS moved RXBD BLSIZE into the LL USB header; per the RM, RXBD count bit31
// is taken directly here.
constexpr uint32_t RX_BD_BLSIZE = 0x80000000UL;

// 包内存的地址由设备头文件给出：它与寄存器块的间距随系列不同，不能从寄存器基址推算。
// The address of the packet memory comes from the device header: its distance from the
// register block differs between families and cannot be derived from the register base.
#if !defined(USB_DRD_PMAADDR)
#error "USB_DRD_PMAADDR is not defined by the device header of this USB_DRD_FS part."
#endif

USB::FSDevDevice::MemoryMap ControllerMemoryMap()
{
  // TXBD/RXBD 的地址字段按 4 字节截断，所以按 4 字节对齐；字间隔在 DRD 上不使用。
  // The address field of TXBD/RXBD is truncated to 4 bytes, hence the 4-byte alignment;
  // the word stride is not used on DRD.
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

#endif  // !defined(USB_DRD_FS)

}  // namespace

// 构造函数在本文件里，所以构建设备对象时总会链接本文件及其分发函数；
// stm32_usb_wrap.cpp 中被包装的 HAL 入口随后把控制器交给运行期。没有设备对象时本文件
// 不参与链接，弱引用为空，CubeMX 代码继续驱动 HAL。
// The constructor is in this file, so building a device object always links this file
// and its dispatch function; the wrapped HAL entry points in stm32_usb_wrap.cpp then
// hand the controller over to the runtime. Without a device object the file stays out
// of the link, the weak reference is null and CubeMX code keeps driving HAL.
STM32USBDeviceDevFs::STM32USBDeviceDevFs(
    const std::initializer_list<EPConfig>& rx_ep_cfgs,
    const std::initializer_list<EPConfig>& tx_ep_cfgs,
    USB::DeviceDescriptor::PacketSize0 packet_size, uint16_t vid, uint16_t pid,
    uint16_t bcd,
    const std::initializer_list<const USB::DescriptorStrings::LanguagePack*>& lang_list,
    const std::initializer_list<const std::initializer_list<USB::ConfigDescriptorItem*>>&
        configs,
    ConstRawData uid, size_t pma_limit, uint8_t slot_count)
    : USB::FSDevDevice(ControllerMemoryMap(), rx_ep_cfgs, tx_ep_cfgs, packet_size, vid,
                       pid, bcd, lang_list, configs, uid, pma_limit, slot_count)
{
  SetHardwareGuard(this, &EnterGuard, &LeaveGuard);

#if defined(USB_DRD_FS)
  // CHEPnR 的高 16 位还有 ERR/NAK/DEVADDR/LSEP，写 1 保持（HAL 的 USB_CHEP_REG_MASK）。
  // The high half of CHEPnR also holds ERR/NAK/DEVADDR/LSEP, kept by writing 1 (HAL's
  // USB_CHEP_REG_MASK).
  ep_err_rx_ = USB_CHEP_ERRRX;
  ep_err_tx_ = USB_CHEP_ERRTX;
  ep_keep_mask_ = USB_CHEP_REG_MASK;
#endif

  // 时钟、引脚与 NVIC 由 CubeMX 的 HAL_PCD_MspInit 提供；句柄只读 Instance。
  // Clocks, pins and NVIC are provided by the CubeMX HAL_PCD_MspInit; the handle is
  // only read for Instance.
  PCD_HandleTypeDef hpcd = {};
  hpcd.Instance = Regs();
  HAL_PCD_MspInit(&hpcd);
  STM32USBRequireWrap();

  InitController();
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
  // L1：没有 BCDR，内部上拉是 SYSCFG_PMC 的 USB_PU 位（HAL 的
  // __HAL_SYSCFG_USBPULLUP_ENABLE 写的也是它）。
  // L1: there is no BCDR, the internal pull-up is the USB_PU bit of SYSCFG_PMC (the
  // bit HAL's __HAL_SYSCFG_USBPULLUP_ENABLE writes).
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
  // F1/F3 没有内部上拉：上电时序由用户实现的 HAL_PCDEx_SetConnectionState 控制。
  // F1/F3 have no internal pull-up: the power-on sequence is controlled by the
  // user-implemented HAL_PCDEx_SetConnectionState.
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

namespace
{
/// 端点 num 的缓冲描述符：TXBD 在前，RXBD 在后 / Buffer descriptor of endpoint num: TXBD
/// first, RXBD second
volatile uint32_t* Bd(uintptr_t packet_memory, uint8_t num, bool rx)
{
  // NOLINTNEXTLINE(performance-no-int-to-ptr)
  return reinterpret_cast<volatile uint32_t*>(packet_memory) + num * 2U + (rx ? 1U : 0U);
}
}  // namespace

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
  // 接收完成后，RX 缓冲描述符里的计数要过几个周期才更新；HAL 的 PCD_GET_EP_RX_CNT
  // 同样先空转 PCD_RX_PMA_CNT 次再读。
  // After a reception the count in the RX buffer descriptor takes a few cycles to
  // update; HAL's PCD_GET_EP_RX_CNT spins PCD_RX_PMA_CNT times before it reads, too.
  volatile uint32_t settle = PCD_RX_PMA_CNT;
  while (settle > 0U)
  {
    settle = settle - 1U;
  }
  return static_cast<uint16_t>((*Bd(memory_map_.packet_memory, num, true) >> 16U) &
                               0x03FFU);
}

// PMA 只支持 32 位访问；尾部的 1–3 字节单独处理（HAL 同式）。
// The PMA supports 32-bit access only; the trailing 1-3 bytes are handled separately
// (same as HAL).
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
  Regs()->CNTR &= ~USB_CNTR_HOST;  // 设备模式 / Device mode.
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
