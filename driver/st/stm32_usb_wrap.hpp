#pragma once

namespace LibXR
{

/**
 * @brief 确认链接器对 HAL_PCD_IRQHandler 的 --wrap 已生效 / Check that the linker wrap of
 *        HAL_PCD_IRQHandler is in effect
 *
 * 寄存器运行期靠 stm32_usb_wrap.cpp 里的包装函数接收 CubeMX 中断函数的调用。包装没有
 * 生效时中断到不了驱动，设备不枚举且没有任何提示，所以设备对象构造时在这里直接报致命
 * 错误。
 *
 * The register runtimes receive the calls of the CubeMX interrupt handlers through the
 * wrappers in stm32_usb_wrap.cpp. Without the wrap no interrupt reaches the driver and
 * the device silently fails to enumerate, so constructing a device object stops here
 * with a fatal error instead.
 */
void STM32USBRequireWrap();

}  // namespace LibXR
