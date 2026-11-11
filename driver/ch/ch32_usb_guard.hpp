#pragma once
#include "ch32_interrupt_guard.h"
#include "usb/core/ep_pool.hpp"

namespace LibXR
{
/**
 * @brief 为端点池安装硬件保护：以关闭全局中断实现临界区 / Install the hardware guard of
 * an endpoint pool: critical sections are made by disabling interrupts
 * @param pool 要配置的端点池 / Endpoint pool to configure
 */
inline void ConfigureCH32USBGuard(USB::EndpointPool& pool)
{
  pool.SetHardwareGuard(
      nullptr, [](void*) -> uintptr_t { return libxr_ch32_interrupt_save_and_disable(); },
      [](void*, uintptr_t saved)
      { libxr_ch32_interrupt_restore(static_cast<uint32_t>(saved)); });
}
}  // namespace LibXR
