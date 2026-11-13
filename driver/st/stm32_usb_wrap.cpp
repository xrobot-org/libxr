#include "main.h"

#if defined(HAL_PCD_MODULE_ENABLED)

// CubeMX 保持默认代码：MX_USB_*_PCD_Init() 调用 HAL_PCD_Init()，
// 生成的中断函数调用 HAL_PCD_IRQHandler()。
// 链接时包装这两个函数（见 driver/st/CMakeLists.txt）。
// 本文件只包含这些包装函数，因此只要 CubeMX 引用它们就会被链接。
// stm32_usb_otg.cpp（OTG）和 stm32_usb_fsdev.cpp（FSDEV/DRD）
// 中的寄存器运行期只在构造设备对象时才被链接，并通过弱引用访问。没有它们时，
// 控制器仍由 HAL 管理，适用于通过 ST 中间件驱动 USB 的工程。
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
  // 设备对象构造时由驱动初始化 core。
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
