#pragma once
#include <atomic>
#include <cstdint>

#include "ch32_can_caps.hpp"

namespace LibXR::CH32UsbCanShared
{
/**
 * @brief USB/CAN 共享中断与资源状态 / Shared USB/CAN IRQ and resource state
 */
using IrqFn = void (*)();

inline std::atomic_bool usb_inited{false};
inline std::atomic_bool can1_inited{false};
inline std::atomic_bool can2_inited{false};
// 端点对象在 Stop/Start 后仍保留 PMA 地址；分配后不能再增加 CAN 占用。
// Endpoint objects retain their PMA addresses across Stop/Start.
inline std::atomic_bool usb_pma_configured{false};

static constexpr uint16_t USBD_PMA_BYTES_SOLO = 512;
static constexpr uint16_t USBD_PMA_BYTES_WITHCAN = 384;
static constexpr uint16_t USBD_PMA_BYTES_WITHCAN2 = 256;

// USB/CAN 共享中断拓扑的编译期能力标志。
// Build-time capability flags for USB/CAN shared interrupt topology.
#if defined(RCC_APB1Periph_USB)
inline constexpr bool K_HAS_USB_DEV_FS = true;
#else
inline constexpr bool K_HAS_USB_DEV_FS = false;
#endif

#if defined(CAN1)
inline constexpr bool K_HAS_CAN1 = true;
#else
inline constexpr bool K_HAS_CAN1 = false;
#endif

#if LIBXR_CH32_HAS_CAN2
inline constexpr bool K_HAS_CAN2 = true;
#else
inline constexpr bool K_HAS_CAN2 = false;
#endif

inline constexpr bool K_SINGLE_CAN1 = K_HAS_CAN1 && !K_HAS_CAN2;
// CAN1 在双 CAN 器件上仍使用 USB 命名的共享中断向量。
// CAN1 uses the USB-named IRQ vectors even on dual-CAN devices.
inline constexpr bool K_USB_CAN_IRQ_SHARE = K_HAS_USB_DEV_FS && K_HAS_CAN1;
// 单 CAN、双 CAN 器件上的 FSDEV 都与 CAN 共享 PMA。
// FSDEV shares PMA with CAN on both single- and dual-CAN devices.
inline constexpr bool K_USB_CAN_SHARE = K_HAS_USB_DEV_FS && K_HAS_CAN1;

inline constexpr bool usb_can_irq_share_enabled() { return K_USB_CAN_IRQ_SHARE; }
inline constexpr bool usb_can_share_enabled() { return K_USB_CAN_SHARE; }

inline uint16_t usb_pma_limit_bytes()
{
  if constexpr (!K_USB_CAN_SHARE)
  {
    return USBD_PMA_BYTES_SOLO;
  }

  // CAN2 使用高编号过滤器组，即使应用没有创建 CAN1 对象也必须预留。
  // CAN2 uses the upper filter banks, including when no CAN1 object was created.
  if constexpr (K_HAS_CAN2)
  {
    if (can2_inited.load(std::memory_order_acquire))
    {
      return USBD_PMA_BYTES_WITHCAN2;
    }
  }
  return can1_inited.load(std::memory_order_acquire) ? USBD_PMA_BYTES_WITHCAN
                                                     : USBD_PMA_BYTES_SOLO;
}

inline std::atomic<IrqFn> usb_irq_cb{nullptr};
inline std::atomic<IrqFn> can1_rx0_cb{nullptr};
inline std::atomic<IrqFn> can1_tx_cb{nullptr};

// 非内联注册使静态库链接必须带入共享 ISR 所在的目标文件。
// Out-of-line registration retains the shared ISR object when linking libxr.a.
void register_usb_irq(IrqFn fn);
void register_can1_rx0(IrqFn fn);
void register_can1_tx(IrqFn fn);

inline bool can1_active()
{
  if constexpr (!K_USB_CAN_IRQ_SHARE)
  {
    return false;
  }
  return (can1_rx0_cb.load(std::memory_order_acquire) != nullptr) ||
         (can1_tx_cb.load(std::memory_order_acquire) != nullptr);
}
}  // namespace LibXR::CH32UsbCanShared
