// NOLINTBEGIN(cppcoreguidelines-pro-type-cstyle-cast,performance-no-int-to-ptr)
#include "ch32_usb_fsdev.hpp"

#include "ch32_usb_rcc.hpp"
#include "ch32_usbcan_shared.hpp"

#if defined(RCC_APB1Periph_USB)

#if !defined(CAN1)
#error "The USBD low-priority vector is defined together with CAN1 in ch32_usbcan_shared."
#endif

namespace LibXR
{

namespace
{

using USB::FSDevDevice;

// 手册 21.3（p.326）：端点寄存器 0x40005C00，通用寄存器 0x40005C40 起，偏移与
// FSDevDevice 的经典型相同。
// Manual 21.3 (p.326): endpoint registers at 0x40005C00, common registers from
// 0x40005C40; the offsets are those of FSDevDevice's classic variant.
constexpr uintptr_t REGISTERS = 0x40005C00UL;
// 手册 p.322：共享的专用 SRAM 在 0x40006000，512 字节。描述表每个 16 位字段占 4 个
// 字节的地址（p.335-336 的偏移 x*16、+4、+8、+12），包缓冲区也一样。
// Manual p.322: the shared dedicated SRAM is at 0x40006000, 512 bytes. Each 16-bit field
// of the description table takes 4 bytes of address (offsets x*16, +4, +8, +12 on
// p.335-336), and so do the packet buffers.
constexpr uintptr_t PACKET_MEMORY = 0x40006000UL;
constexpr size_t PACKET_MEMORY_SIZE = 512U;
constexpr uint8_t PMA_WORD_STRIDE = 2U;
// 缓冲区地址按 2 字节对齐（p.336）。 / Buffer addresses are 2-byte aligned (p.336).
constexpr uint8_t PMA_ALIGNMENT = 2U;

static_assert(PACKET_MEMORY_SIZE == CH32UsbCanShared::USBD_PMA_BYTES_SOLO,
              "the PMA size differs from the size shared with CAN");

FSDevDevice::MemoryMap ControllerMemoryMap()
{
  return {REGISTERS, PACKET_MEMORY, PACKET_MEMORY_SIZE, PMA_WORD_STRIDE, PMA_ALIGNMENT};
}

size_t PmaLimit(size_t requested)
{
  const size_t available = CH32UsbCanShared::usb_pma_limit_bytes();
  ASSERT(requested <= available);
  // 布局就此固定，之后不能再让 CAN 占用更多。
  // The layout is fixed from here on; CAN may not take more afterwards.
  CH32UsbCanShared::usb_pma_configured.store(true, std::memory_order_release);
  return requested == 0U ? available : requested;
}

void OnLowPriorityInterrupt() { FSDevDevice::Dispatch(); }

void OnHighPriorityInterrupt() { FSDevDevice::Dispatch(); }

/// 高优先级线取低优先级线的优先级 / The high-priority line takes the low-priority line's
/// priority
void MatchHighPriority()
{
  NVIC->IPRIOR[static_cast<uint32_t>(USB_HP_CAN1_TX_IRQn)] =
      NVIC->IPRIOR[static_cast<uint32_t>(USB_LP_CAN1_RX0_IRQn)];
}

}  // namespace

CH32USBDeviceFS::CH32USBDeviceFS(
    const std::initializer_list<EPConfig>& rx_ep_cfgs,
    const std::initializer_list<EPConfig>& tx_ep_cfgs,
    USB::DeviceDescriptor::PacketSize0 packet_size, uint16_t vid, uint16_t pid,
    uint16_t bcd,
    const std::initializer_list<const USB::DescriptorStrings::LanguagePack*>& languages,
    const std::initializer_list<const std::initializer_list<USB::DeviceClass*>>& configs,
    ConstRawData uid, size_t pma_limit, uint8_t slot_count)
    : USB::FSDevDevice(ControllerMemoryMap(), rx_ep_cfgs, tx_ep_cfgs, packet_size, vid,
                       pid, bcd, languages, configs, uid, PmaLimit(pma_limit), slot_count)
{
  // 手册 p.321 21.2.2 2)：先保证 USB 时钟是 48 MHz，再打开 USBD 的接口时钟。48 MHz 取自
  // USBHS 的 PHY 时，USBHS 的时钟也要开着。
  // Manual p.321 21.2.2 2): make the USB clock 48 MHz first, then enable the interface
  // clock of USBD. When the 48 MHz come from the USBHS PHY, the USBHS clock stays on too.
  CH32UsbRcc::ConfigureUsb48M();
#if defined(RCC_USBCLK48MCLKSource_USBPHY) && defined(RCC_AHBPeriph_USBHS)
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_USBHS, ENABLE);
#endif
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);

#if defined(RCC_APB2Periph_GPIOA) && defined(GPIOA) && defined(GPIO_Pin_11) &&        \
    defined(GPIO_Pin_12) && defined(GPIO_Mode_Out_PP) && defined(GPIO_Speed_50MHz) && \
    defined(GPIO_Mode_IN_FLOATING)
  // 手册 p.321 21.2.2 1)：使能 USBD 之前引脚输出低，免得主机提前看到设备；之后改为浮空
  // 输入，由内部收发器接管。
  // Manual p.321 21.2.2 1): the pins drive low before USBD is enabled so that the host
  // sees no device early; they become floating inputs afterwards and the internal
  // transceiver takes over.
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

  // 全速：上拉接在 D+。 / Full speed: the pull-up is on D+.
  EXTEN->EXTEN_CTR &= ~EXTEN_USBD_LS;

  // owner 总在控制器中断里（PendInterrupt 挂起低优先级线），Dispatch 从低、高两条优先级
  // 相同的线进入，两者不会互相打断，所以可以用 EP0 的快速确认和状态 OUT 的晚放行（见类
  // 说明）。CTR_RX 没清时控制器丢弃新的 SETUP（手册 p.324），与 FSDevDevice 针对的行为
  // 相同。
  // The owner always runs in the controller interrupt (PendInterrupt pends the
  // low-priority line); Dispatch is entered from the low- and high-priority lines, which
  // have the same priority and never interrupt each other, so the fast acknowledgement
  // of EP0 and the late release of the status OUT may be used (see the class notes). The
  // controller drops a new SETUP while CTR_RX is set (manual p.324), the behaviour
  // FSDevDevice addresses.
  EnableEp0FastAck();
  EnableEp0LateStatus();
  // 手册 p.323 说 NAK 状态下 OUT 包得到 NAK；CH32V203 上实测 EP0 为 CONTROL 类型时照收
  // 并 ACK 主机的每个 OUT 数据包，未读的上一包被覆盖（类说明）。BULK 闸关掉这个窗口，
  // 早到的包由 EnableEp0EarlyOut 接住。
  // The manual (p.323) says an OUT packet is answered NAK in the NAK state; measured on a
  // CH32V203, EP0 typed CONTROL takes and ACKs every OUT data packet of the host and an
  // unread previous packet is overwritten (class notes). The BULK gate closes that
  // window; an early packet is caught by EnableEp0EarlyOut.
  EnableEp0EarlyOut();
  EnableEp0BulkGate();
  // 实测：OUT 包等着时其间有 IN 被 ACK，之后被 NAK 的 OUT 包把 COUNTn_RX 写成 0x3FE（类
  // 说明）。处理端点时先于它的 IN 完成锁存计数。
  // Measured: with an IN ACKed while an OUT packet waits, a NAKed OUT packet after it
  // writes COUNTn_RX 0x3FE (class notes). The count is latched when the endpoint is
  // handled, before its IN completion.
  EnableRxCountLatch();

  // 先登记入口再开线。中断源此时全关，到 Start 才打开。两条线同一优先级。
  // Register the entries before the lines are enabled. Every interrupt source is still
  // off and comes on at Start. Both lines get the same priority.
  CH32UsbCanShared::register_usb_lp_irq(&OnLowPriorityInterrupt);
  CH32UsbCanShared::register_usb_hp_irq(&OnHighPriorityInterrupt);
  MatchHighPriority();
  // 构造时断开一次（含 D+ 拉低，见类说明），Start 在 DETACH_US 之后接上。Dispatch 在
  // D+ 被拉住期间不进。
  // Detach once at construction (with D+ held low, see the class notes); Start attaches
  // after DETACH_US. Dispatch stays out while D+ is held.
  DetachAtConstruction();
  NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
  NVIC_EnableIRQ(USB_HP_CAN1_TX_IRQn);
}

bool CH32USBDeviceFS::HoldDetached()
{
#if LIBXR_CH32_USBD_DP_DETACH
  // 收发器断电，再关 USBD 的接口时钟；寄存器和 PMA 保留。 / Transceiver powered down,
  // then the interface clock of USBD off; registers and PMA are kept.
  WriteCntr(static_cast<uint16_t>(USB::FSDEV_CNTR_FRES | USB::FSDEV_CNTR_PDWN));
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, DISABLE);
  // PA12 是 CFGHR 的 [19:16]（MODE、CNF）。先把输出锁存写 0，再切成推挽输出。
  // PA12 is CFGHR[19:16] (MODE, CNF). The output latch is written 0 before the switch to
  // push-pull output.
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  dp_saved_[0] = GPIOA->CFGHR & (0xFUL << 16U);
  dp_saved_[1] = GPIOA->OUTDR & (1UL << 12U);
  GPIOA->BCR = 1UL << 12U;
  GPIOA->CFGHR =
      (GPIOA->CFGHR & ~(0xFUL << 16U)) | (0x2UL << 16U);  // 推挽 2 MHz / push-pull
  return true;
#else
  return false;
#endif
}

void CH32USBDeviceFS::ReleaseDetached()
{
#if LIBXR_CH32_USBD_DP_DETACH
  // 引脚先回到原来的配置（构造时是浮空输入），再开时钟；InitController 随后清 PDWN。
  // The pin returns to its previous configuration first (floating input since the
  // construction), then the clock comes on; InitController clears PDWN afterwards.
  GPIOA->CFGHR = (GPIOA->CFGHR & ~(0xFUL << 16U)) | dp_saved_[0];
  if (dp_saved_[1] != 0U)
  {
    GPIOA->BSHR = 1UL << 12U;
  }
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);
#endif
}

void CH32USBDeviceFS::Connect(bool on)
{
  if (on)
  {
    EXTEN->EXTEN_CTR |= EXTEN_USBD_PU_EN;
  }
  else
  {
    EXTEN->EXTEN_CTR &= ~EXTEN_USBD_PU_EN;
  }
}

void CH32USBDeviceFS::MaskInterrupt()
{
  NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
  NVIC_DisableIRQ(USB_HP_CAN1_TX_IRQn);
}

void CH32USBDeviceFS::UnmaskInterrupt()
{
  // 应用改过低优先级线的优先级时，高优先级线在这里跟上。 / Where the application changed
  // the low-priority line's priority, the high-priority line follows here.
  MatchHighPriority();
  NVIC_EnableIRQ(USB_HP_CAN1_TX_IRQn);
  NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
}

bool CH32USBDeviceFS::PendInterrupt()
{
  NVIC_SetPendingIRQ(USB_LP_CAN1_RX0_IRQn);
  return true;
}

}  // namespace LibXR

#endif  // defined(RCC_APB1Periph_USB)

// NOLINTEND(cppcoreguidelines-pro-type-cstyle-cast,performance-no-int-to-ptr)
