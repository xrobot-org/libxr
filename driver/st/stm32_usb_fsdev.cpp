#include "stm32_usb_fsdev.hpp"

// `USB` is undefined by the header (it collides with the LibXR::USB namespace), so the
// guard uses the presence flag the header records before that.
#if defined(HAL_PCD_MODULE_ENABLED) && defined(LIBXR_STM32_USB_FSDEV_PRESENT)

// Defined by the linker wrap in driver/st/CMakeLists.txt (see stm32_usb_wrap.cpp).
extern "C" void __wrap_HAL_PCD_IRQHandler(PCD_HandleTypeDef* hpcd);

namespace LibXR
{

// The constructor in stm32_usb_fsdev.hpp requires this symbol, so building a device
// object always links this file and its dispatch function; the wrapped HAL entry points
// in stm32_usb_wrap.cpp then hand the controller over to the runtime. Without a device
// object the file stays out of the link, the weak reference is null and CubeMX code
// keeps driving HAL.

void STM32FsDevRequireWrap()
{
  // Read through a volatile pointer so the comparison is made on the linked addresses.
  void (*volatile handler)(PCD_HandleTypeDef*) = &HAL_PCD_IRQHandler;
  REQUIRE(handler == &__wrap_HAL_PCD_IRQHandler);
}

}  // namespace LibXR

extern "C" void libxr_stm32_fsdev_dispatch(PCD_HandleTypeDef*)
{
  LibXR::STM32USBDeviceDevFs::Dispatch();
}

#endif
