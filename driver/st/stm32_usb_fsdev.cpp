#include "stm32_usb_fsdev.hpp"

// `USB` 宏已被头文件取消定义（它与 LibXR::USB 命名空间冲突），
// 因此守卫改用头文件在此之前记录的存在标志。
// `USB` is undefined by the header (it collides with the LibXR::USB namespace), so the
// guard uses the presence flag the header records before that.
#if defined(HAL_PCD_MODULE_ENABLED) && defined(LIBXR_STM32_USB_FSDEV_PRESENT)

// 由 driver/st/CMakeLists.txt 中的链接器 wrap 定义（见 stm32_usb_wrap.cpp）。
// Defined by the linker wrap in driver/st/CMakeLists.txt (see stm32_usb_wrap.cpp).
extern "C" void __wrap_HAL_PCD_IRQHandler(PCD_HandleTypeDef* hpcd);

namespace LibXR
{

// stm32_usb_fsdev.hpp 中的构造函数需要该符号，因此构建设备对象时总会链接本文件
// 及其分发函数；stm32_usb_wrap.cpp 中被包装的 HAL 入口随后把控制器交给运行期。
// 没有设备对象时本文件不参与链接，弱引用为空，CubeMX 代码继续驱动 HAL。
// The constructor in stm32_usb_fsdev.hpp requires this symbol, so building a device
// object always links this file and its dispatch function; the wrapped HAL entry points
// in stm32_usb_wrap.cpp then hand the controller over to the runtime. Without a device
// object the file stays out of the link, the weak reference is null and CubeMX code
// keeps driving HAL.

void STM32FsDevRequireWrap()
{
  // 通过 volatile 指针读取，使比较针对链接后的地址。
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
