#include "stm32_usb_fsdev.hpp"

#include "stm32_usb_wrap.hpp"

#if defined(HAL_PCD_MODULE_ENABLED) && (defined(USB_BASE) || defined(USB_DRD_FS))

namespace LibXR
{

// stm32_usb_fsdev.hpp 中的构造函数需要该符号，
// 因此构建设备对象时总会链接本文件及其分发函数；stm32_usb_wrap.cpp 中被包装的 HAL 入口随
// 后把控制器交给运行期。没有设备对象时本文件不参与链接，弱引用为空，
// CubeMX 代码继续驱动 HAL。
// The constructor in stm32_usb_fsdev.hpp requires this symbol, so building a device
// object always links this file and its dispatch function; the wrapped HAL entry points
// in stm32_usb_wrap.cpp then hand the controller over to the runtime. Without a device
// object the file stays out of the link, the weak reference is null and CubeMX code
// keeps driving HAL.

void STM32FsDevRequireWrap() { STM32USBRequireWrap(); }

}  // namespace LibXR

extern "C" void libxr_stm32_fsdev_dispatch(PCD_HandleTypeDef*)
{
  LibXR::STM32USBDeviceDevFs::Dispatch();
}

#endif
