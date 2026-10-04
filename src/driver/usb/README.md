# XRUSB

<div align="center">

<img src="https://github.com/xrobot-org/LibXR_CppCodeGenerator/raw/master/imgs/XRobot.jpeg" width="300">

LibXR 的 USB 设备协议栈 / The USB device stack of LibXR

![License](https://img.shields.io/badge/license-Apache--2.0-blue)
[![Documentation](https://img.shields.io/badge/docs-online-brightgreen)](https://xrobot.work/libxr/)
[![FOSSA Status](https://app.fossa.com/api/projects/git%2Bgithub.com%2FJiu-xiao%2Flibxr.svg?type=shield)](https://app.fossa.com/projects/git%2Bgithub.com%2FJiu-xiao%2Flibxr?ref=badge_shield)

</div>

## 介绍 / Introduction

XRUSB 是 [LibXR](https://github.com/xrobot-org/libxr) 的 USB 设备协议栈，用 C++20 编写，位于
`src/driver/usb`。`core/` 生成设备、配置、字符串和 BOS 描述符并管理端点池，`device/` 是设备核心、
组合设备和各设备类。一个组合设备可以包含多个设备类，初始化时各设备类从端点池申请所需的端点。这个目录
只有平台无关的代码，USB 设备控制器的驱动位于各平台的 `driver/<平台>/`，例如 `driver/st/stm32_usb_dev.cpp`、
`driver/ch/ch32_usb_otghs.cpp` 和 `driver/esp/esp_usb_dev.cpp`。

XRUSB is the USB device stack of [LibXR](https://github.com/xrobot-org/libxr), written in C++20,
in `src/driver/usb`. `core/` builds the device, configuration, string and BOS descriptors and
manages the endpoint pool; `device/` holds the device core, the composite device and the device
classes. A composite device can contain several device classes, and each class takes the
endpoints it needs from the endpoint pool at initialization. This directory holds only
platform-independent code; the USB device controller drivers are in `driver/<platform>/` of each
platform, such as `driver/st/stm32_usb_dev.cpp`, `driver/ch/ch32_usb_otghs.cpp` and
`driver/esp/esp_usb_dev.cpp`.

[平台外设支持列表](../../../doc/support.md)中的 `USB-DEVICE` 指 XRUSB 使用的 USB 设备控制器。ESP32-C3 和
ESP32-C6 的 `CDC-JTAG` 由 `driver/esp/esp_cdc_jtag.*` 实现，使用芯片自带的 USB Serial/JTAG 控制器，不经过
XRUSB。

`USB-DEVICE` in the [platform peripheral support list](../../../doc/support.md) means the USB
device controller used by XRUSB. `CDC-JTAG` on ESP32-C3 and ESP32-C6 is implemented by
`driver/esp/esp_cdc_jtag.*` on the chip's own USB Serial/JTAG controller and does not use XRUSB.

## 设备类 / Device Classes

| 设备类 / Class | 头文件 / Header | 说明 / Notes |
| --- | --- | --- |
| CDC-ACM | `device/cdc/cdc_uart.hpp`、`cdc_to_uart.hpp` | `CDCUart` 作为 `LibXR::UART` 使用；`CDCToUart` 把 CDC 与一个 UART 双向桥接 / `CDCUart` is used as a `LibXR::UART`; `CDCToUart` bridges CDC and a UART in both directions |
| HID | `device/hid/hid_keyboard.hpp`、`hid_mouse.hpp`、`hid_gamepad.hpp` | 键盘、鼠标和手柄；其他报告由 `HID` 派生 / Keyboard, mouse and gamepad; other reports derive from `HID` |
| UAC | `device/uac/uac_mic.hpp` | UAC1 麦克风 / UAC1 microphone |
| GS USB | `device/gsusb/gs_usb.hpp` | CAN 与 CAN FD 适配器，Linux 内核自带的 gs_usb 驱动把它作为 SocketCAN 接口 / CAN and CAN FD adapter that the gs_usb driver of the Linux kernel exposes as a SocketCAN interface |
| DAPLink V1 | `device/dap/daplink_v1.hpp` | CMSIS-DAP v1（HID），SWD 与 JTAG / CMSIS-DAP v1 (HID), SWD and JTAG |
| DAPLink V2 | `device/dap/daplink_v2.hpp` | CMSIS-DAP v2（Bulk），SWD 与 JTAG，可用于 Keil 和 OpenOCD / CMSIS-DAP v2 (Bulk), SWD and JTAG, works with Keil and OpenOCD |
| DFU | `device/dfu/dfu.hpp`、`dfu_bootloader.hpp` | DFU 运行时接口和单镜像 bootloader / DFU runtime interface and single-image bootloader |
| BOS | `device/bos/webusb.hpp`、`winusb_msos20.hpp` | WebUSB 与 WinUSB MS OS 2.0 描述符 / WebUSB and WinUSB MS OS 2.0 descriptors |

`device/cdc/cdc_test.hpp` 中的 `CDCWriteTest` 和 `CDCReadTest` 用于测试 CDC 的传输。

`CDCWriteTest` and `CDCReadTest` in `device/cdc/cdc_test.hpp` test CDC transfers.

## 设备控制器 / Device Controllers

| 平台 / Platform | 控制器 / Controller | 驱动 / Driver | 测试设备 / Tested on |
| --- | --- | --- | --- |
| STM32 | FSDEV（`USB_BASE`、`USB_DRD_FS`） | `driver/st/stm32_usb_dev.cpp` | STM32F103、STM32G431 |
| STM32 | OTG FS（`USB_OTG_FS`） | `driver/st/stm32_usb_dev.cpp` | STM32F407 |
| STM32 | OTG HS（`USB_OTG_HS`） | `driver/st/stm32_usb_dev.cpp` | STM32F407、STM32H750 |
| ESP32-S3 | OTG FS | `driver/esp/esp_usb_dev.cpp` | ESP32-S3 |
| CH32 | FSDEV | `driver/ch/ch32_usb_devfs.cpp` | CH32V203 |
| CH32 | USBFS（OTG FS） | `driver/ch/ch32_usb_otgfs.cpp` | CH32V307、CH32V203、CH32V208 |
| CH32 | USBHS（OTG HS） | `driver/ch/ch32_usb_otghs.cpp` | CH32V307 |

STM32 驱动按 CMSIS 设备头文件中定义的外设宏编译对应的控制器。

The STM32 driver compiles the controllers whose peripheral macros the CMSIS device header defines.

## 文档 / Documentation

用法见 LibXR 文档中的 [XRUSB 协议栈](https://xrobot.work/docs/xrusb)。

Usage is described in [XRUSB Stack](https://xrobot.work/en/docs/xrusb) of the LibXR documentation.
