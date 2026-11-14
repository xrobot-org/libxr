// ch32_usb_devfs.cpp  (classic FSDEV / PMA)
#include "ch32_usb_dev.hpp"
#include "ch32_usb_guard.hpp"
#include "ch32_usb_rcc.hpp"
#include "ch32_usbcan_shared.hpp"

#if defined(RCC_APB1Periph_USB)

namespace
{
void UsbIrqThunk() { LibXR::CH32USBDeviceFS::Dispatch(); }
}  // namespace

// NOLINTNEXTLINE(readability-identifier-naming)
extern "C" __attribute__((interrupt("WCH-Interrupt-fast"))) void USBWakeUp_IRQHandler(
    void)
{
  EXTI_ClearITPendingBit(EXTI_Line18);
}

namespace LibXR
{

namespace
{
// 外设库头文件没有给出 USBD 的基址，这里按参考手册写出：端点寄存器从 0x40005C00
// 开始，包缓冲区（与 CAN 共用的 512 B）在 0x40006000。包缓冲区里每个 16 位数据字占
// 两个 16 位地址，缓冲地址按 2 字节对齐。
// The peripheral library headers give no base address for USBD, so they are written
// out as the reference manual states: the endpoint registers start at 0x40005C00 and the
// packet buffer (512 B shared with CAN) is at 0x40006000. Every 16-bit data word of the
// packet buffer takes two 16-bit addresses, and buffer addresses are aligned to 2 bytes.
constexpr USB::FSDevDevice::MemoryMap MEMORY_MAP = {0x40005C00UL, 0x40006000UL, 512U, 2U,
                                                    2U};
}  // namespace

CH32USBDeviceFS::CH32USBDeviceFS(
    const std::initializer_list<EPConfig>& rx_ep_cfgs,
    const std::initializer_list<EPConfig>& tx_ep_cfgs,
    USB::DeviceDescriptor::PacketSize0 packet_size, uint16_t vid, uint16_t pid,
    uint16_t bcd,
    const std::initializer_list<const USB::DescriptorStrings::LanguagePack*> LANG_LIST,
    const std::initializer_list<const std::initializer_list<USB::ConfigDescriptorItem*>>
        CONFIGS,
    ConstRawData uid)
    : USB::FSDevDevice(MEMORY_MAP, rx_ep_cfgs, tx_ep_cfgs, packet_size, vid, pid, bcd,
                       LANG_LIST, CONFIGS, uid, CH32UsbCanShared::usb_pma_limit_bytes())
{
  ConfigureCH32USBGuard(*this);

  // FSDEV 使用共享 USB 48 MHz 时钟；如果这个时钟来自 USBHS PHY PLL，
  // 则必须先打开 USBHS 依赖时钟，再打开 USBDEV 本体时钟。
  // FSDEV uses the shared USB 48 MHz clock; when that clock comes from the USBHS
  // PHY PLL, keep the USBHS dependency clock enabled before USBDEV itself.
  CH32UsbRcc::ConfigureUsb48M();
#if defined(RCC_USBCLK48MCLKSource_USBPHY) && defined(RCC_AHBPeriph_USBHS)
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_USBHS, ENABLE);
#endif
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);

  InitController();
  CH32UsbCanShared::usb_pma_configured.store(true, std::memory_order_release);
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

void CH32USBDeviceFS::Start(bool in_isr)
{
  // 回调与向量就绪后再放行中断和连接主机。
  // Publish the handler and enable the vectors before the runtime opens the interrupt
  // sources and the pull-up.
  CH32UsbCanShared::register_usb_irq(&UsbIrqThunk);
  CH32UsbCanShared::usb_inited.store(true, std::memory_order_release);
  NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
  NVIC_EnableIRQ(USB_HP_CAN1_TX_IRQn);
  NVIC_EnableIRQ(USBWakeUp_IRQn);
  USB::FSDevDevice::Start(in_isr);
}

void CH32USBDeviceFS::Stop(bool in_isr)
{
  // 先屏蔽 USB 中断源，再解除回调；共享 CAN 中断线可能仍然开启。
  // Mask the USB interrupt sources before unregistering from live shared CAN IRQs.
  USB::FSDevDevice::Stop(in_isr);
  CH32UsbCanShared::usb_inited.store(false, std::memory_order_release);
  CH32UsbCanShared::register_usb_irq(nullptr);

  if (!CH32UsbCanShared::can1_active())
  {
    NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
    NVIC_DisableIRQ(USB_HP_CAN1_TX_IRQn);
  }
  NVIC_DisableIRQ(USBWakeUp_IRQn);
}

}  // namespace LibXR

#endif  // defined(RCC_APB1Periph_USB)
