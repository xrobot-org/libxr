#include "ch32_usbcan_shared.hpp"

namespace LibXR::CH32UsbCanShared
{
void register_usb_irq(IrqFn fn) { usb_irq_cb.store(fn, std::memory_order_release); }

void register_can1_rx0(IrqFn fn)
{
  if constexpr (K_USB_CAN_IRQ_SHARE)
  {
    can1_rx0_cb.store(fn, std::memory_order_release);
  }
  else
  {
    (void)fn;
  }
}

void register_can1_tx(IrqFn fn)
{
  if constexpr (K_USB_CAN_IRQ_SHARE)
  {
    can1_tx_cb.store(fn, std::memory_order_release);
  }
  else
  {
    (void)fn;
  }
}
}  // namespace LibXR::CH32UsbCanShared

#if defined(RCC_APB1Periph_USB) && defined(CAN1)

// NOLINTNEXTLINE(readability-identifier-naming)
extern "C" __attribute__((interrupt("WCH-Interrupt-fast"))) void
USB_LP_CAN1_RX0_IRQHandler(void)
{
  using namespace LibXR::CH32UsbCanShared;

  if (auto fn = usb_irq_cb.load(std::memory_order_acquire))
  {
    fn();
  }
  if (auto fn = can1_rx0_cb.load(std::memory_order_acquire))
  {
    fn();
  }
}

// NOLINTNEXTLINE(readability-identifier-naming)
extern "C" __attribute__((interrupt("WCH-Interrupt-fast"))) void
USB_HP_CAN1_TX_IRQHandler(void)
{
  using namespace LibXR::CH32UsbCanShared;

  if (auto fn = usb_irq_cb.load(std::memory_order_acquire))
  {
    fn();
  }
  if (auto fn = can1_tx_cb.load(std::memory_order_acquire))
  {
    fn();
  }
}

#endif
