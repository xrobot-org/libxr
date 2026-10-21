#pragma once
#include "ch32_interrupt_guard.h"
#include "usb/core/ep_pool.hpp"

namespace LibXR
{
inline void ConfigureCH32USBGuard(USB::EndpointPool& pool)
{
  pool.SetHardwareGuard(
      nullptr, [](void*) -> uintptr_t { return libxr_ch32_interrupt_save_and_disable(); },
      [](void*, uintptr_t saved)
      { libxr_ch32_interrupt_restore(static_cast<uint32_t>(saved)); });
}
}  // namespace LibXR
