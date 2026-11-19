#pragma once

#include "main.h"

namespace LibXR
{

/**
 * @brief 确认链接器对 HAL_PCD_IRQHandler 的 --wrap 已生效 / Check that the linker wrap of
 *        HAL_PCD_IRQHandler is in effect
 * @note 寄存器运行期靠 stm32_usb_wrap.cpp 里的包装函数接收 CubeMX 中断函数的调用。包装没
 *       有生效时中断到不了驱动，设备不枚举且没有任何提示，所以设备对象构造时在这里直接报
 *       致命错误。比较的是 stm32_usb_wrap.cpp 自己对 HAL_PCD_IRQHandler 的引用被链接到哪
 *       里：能发现链接选项缺失；开了链接时优化而 HAL 源文件仍参与其中时，CubeMX 代码里的
 *       调用可能已被编译器直接绑定，这里不一定看得出来，见 stm32_usb_wrap.cpp。
 *       The register runtimes receive the calls of the CubeMX interrupt handlers through
 *       the wrappers in stm32_usb_wrap.cpp. Without the wrap no interrupt reaches the
 *       driver and the device silently fails to enumerate, so constructing a device
 *       object stops here with a fatal error instead. The comparison is made on where
 *       stm32_usb_wrap.cpp's own reference to HAL_PCD_IRQHandler was linked: it finds a
 *       missing link option; with link-time optimization and the HAL source still part of
 *       it, the compiler may have bound the calls in the CubeMX code directly, which this
 *       does not always show, see stm32_usb_wrap.cpp.
 */
void STM32USBRequireWrap();

#if defined(HAL_PCD_MODULE_ENABLED)
/**
 * @brief CubeMX 交给 HAL_PCD_Init 的句柄 / The handle CubeMX passed to HAL_PCD_Init
 * @return 寄存器运行期接管时由 __wrap_HAL_PCD_Init 记下的指针；工程没有调用
 *         HAL_PCD_Init 时为 nullptr / The pointer __wrap_HAL_PCD_Init recorded when a
 *         register runtime took over; nullptr when the project never called HAL_PCD_Init
 * @note 句柄里 Instance 和 Init 是 MX_USB_*_PCD_Init 写好的；HAL 自己的初始化没有运行，
 *       State、Lock、端点数组等保持调用之前的值。
 *       Instance and Init were filled in by MX_USB_*_PCD_Init; HAL's own initialization
 *       did not run, so State, Lock, the endpoint arrays and so on keep their values from
 *       before the call.
 */
PCD_HandleTypeDef* STM32USBInitHandle();
#endif

}  // namespace LibXR
