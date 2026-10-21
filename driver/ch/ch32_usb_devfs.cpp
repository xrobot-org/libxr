// ch32_usb_devfs.cpp  (classic FSDEV / PMA)
#include "ch32_usb_dev.hpp"
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
#if defined(EXTI_Line18)
  EXTI_ClearITPendingBit(EXTI_Line18);
#endif
}

namespace LibXR
{

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
  USB::FSDevDevice<Ch32DevFsHw>::Start(in_isr);
}

void CH32USBDeviceFS::Stop(bool in_isr)
{
  // 先屏蔽 USB 中断源，再解除回调；共享 CAN 中断线可能仍然开启。
  // Mask the USB interrupt sources before unregistering from live shared CAN IRQs.
  USB::FSDevDevice<Ch32DevFsHw>::Stop(in_isr);
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
