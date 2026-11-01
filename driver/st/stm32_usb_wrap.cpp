#include "main.h"

#if defined(HAL_PCD_MODULE_ENABLED)

// CubeMX keeps its default code: MX_USB_*_PCD_Init() calls HAL_PCD_Init() and the
// generated interrupt handlers call HAL_PCD_IRQHandler(). The link wraps both (see
// driver/st/CMakeLists.txt). This file holds only the wrappers, so it is linked
// whenever CubeMX references them. The register runtimes in stm32_usb_otg.cpp (OTG)
// and stm32_usb_fsdev.cpp (FSDEV/DRD) are linked only when a device object is
// constructed, and they are reached through weak references. Without them the
// controller stays with HAL, for a project that drives USB through ST's middleware.

extern "C" void libxr_stm32_otg_dispatch(PCD_HandleTypeDef* hpcd) __attribute__((weak));
extern "C" void libxr_stm32_fsdev_dispatch(PCD_HandleTypeDef* hpcd) __attribute__((weak));

extern "C" HAL_StatusTypeDef __real_HAL_PCD_Init(PCD_HandleTypeDef* hpcd);
extern "C" void __real_HAL_PCD_IRQHandler(PCD_HandleTypeDef* hpcd);

extern "C" HAL_StatusTypeDef __wrap_HAL_PCD_Init(PCD_HandleTypeDef* hpcd)
{
  // The driver initializes the core when its device object is constructed.
  if (libxr_stm32_otg_dispatch != nullptr || libxr_stm32_fsdev_dispatch != nullptr)
  {
    return HAL_OK;
  }
  return __real_HAL_PCD_Init(hpcd);
}

extern "C" void __wrap_HAL_PCD_IRQHandler(PCD_HandleTypeDef* hpcd)
{
  if (libxr_stm32_otg_dispatch != nullptr)
  {
    libxr_stm32_otg_dispatch(hpcd);
    return;
  }
  if (libxr_stm32_fsdev_dispatch != nullptr)
  {
    libxr_stm32_fsdev_dispatch(hpcd);
    return;
  }
  __real_HAL_PCD_IRQHandler(hpcd);
}

#endif
