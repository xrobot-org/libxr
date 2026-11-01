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

## 2.0.2 迁移说明 / Migration notes for 2.0.2

从 2.0.2 起，端点缓冲改由槽模型统一管理：端点内存以"槽"为单位交给核心，硬件 DMA 直接指向槽，
不再区分软件缓冲与硬件缓冲。旧写法全部编译失败，不提供兼容垫片 / Starting with 2.0.2, endpoint
buffers are managed by the slot model: endpoint memory is handed to the core in slots and the
hardware DMA points at the slots directly; there is no separate software buffer and hardware
buffer. Old code fails to compile and no compatibility shims are provided.

使用方需要做的事 / What users need to change:

- 端点缓冲区数组放在 DMA 可访问的内存段（`LIBXR_DMA_BUFFER`）并按 cache line 对齐，
  每个单向端点块长不小于 2 倍最大包长 / Put the endpoint buffer arrays in a DMA-accessible
  section (`LIBXR_DMA_BUFFER`) and align them to the cache line; each single-direction endpoint
  block is at least twice the maximum packet size.
- 设备构造函数按平台接收端点列表：CH32 OTG FS/HS 接收 RX/TX 两个列表（EP0 第一项），
  ESP32-S3 保持逐端点声明，STM32 由 CodeGenerator 生成 / Device constructors take endpoint
  lists per platform: CH32 OTG FS/HS take RX and TX lists (EP0 first), ESP32-S3 keeps the
  per-endpoint declaration, and STM32 is generated by the CodeGenerator.
- `DapLinkV2Class` 不再在 USB 上下文里执行命令：应用起一个线程循环调用 `Step()`，例如 `while (true) { dap.Step(); }`，SWD/JTAG 操作只在这个线程里发生。延时类命令在这个线程里睡到到期。线程应当和 USB 中断分在不同的核上，并且能访问 SWD 引脚（例如 ESP32-S3 的专用 GPIO 要在运行该线程的核上创建）；libxr 不安排线程在哪个核/ `DapLinkV2Class` no longer executes commands in the USB context: the application starts one thread that calls `Step()` in a loop, for example `while (true) { dap.Step(); }`, and all SWD/JTAG work happens in that thread. Delay-type commands sleep until they are due in this thread. The thread should run on a different core than the USB interrupt and must be able to reach the SWD pins (for example ESP32-S3 dedicated GPIO is created on the core that runs the thread); libxr does not place the thread.
- `DapLinkV2Class` 的两个数据端点都用借用模式：OUT 每个包借给回调、拷进命令池后立即归还，两个槽交替接收；IN 方向响应先写进类里的暂存槽，由 USB 侧在 IN 槽有空时拷进去提交（一个响应一次拷贝），所以 SWD 执行不必等主机取走前面的响应。通告给主机的 DAP 包长取配置值与 IN 槽容量中较小者，所以 IN 端点的缓冲块要不小于 2 倍 DAP 包长（1024 字节包长用 2048 字节）；在途命令数为 4 / `DapLinkV2Class` uses borrow mode on both data endpoints: every OUT packet is lent to the callback, copied into the command pool and given back at once, and the two slots alternate; a response is first written into a staging slot of the class and the USB side copies it into an IN slot and submits it when one is free (one copy per response), so SWD execution does not wait for the host to take earlier responses. The DAP packet size announced to the host is the smaller of the configured value and an IN slot, so the IN endpoint block must be at least twice the DAP packet size (2048 bytes for a 1024 byte packet); four commands are in flight.
- `DapLinkV2Class` 的 MS OS 2.0 描述符集改由 `DeviceComposition` 生成：类用 `GetWinUsbInterfaceGuid()` 声明自己需要 WinUSB，单个功能的非复合设备用设备级形式（否则 Windows 不绑定，问题代码 28），其余用配置/功能子集形式。`DapLinkV2Class::GetWinUsbMsOs20DescriptorSet` 和 DAP 自带的 BOS 能力已删除，改用 `DeviceComposition::WinUsbBosCapability()`；DFU 仍然自带 MS OS 2.0 描述符 / The MS OS 2.0 descriptor set of `DapLinkV2Class` is now built by `DeviceComposition`: a class declares that it wants WinUSB through `GetWinUsbInterfaceGuid()`, a single function on a non-composite device gets the device-level form (otherwise Windows does not bind, problem code 28) and every other case the configuration/function subset form. `DapLinkV2Class::GetWinUsbMsOs20DescriptorSet` and the BOS capability of the DAP class are gone, use `DeviceComposition::WinUsbBosCapability()`; DFU still carries its own MS OS 2.0 descriptors.
- ESP32-S3 的 EP0 缓冲对半分给 OUT 和 IN，至少 2 倍 EP0 包长（64 字节包长用 128 字节）/ On ESP32-S3 the EP0 buffer is split in halves for OUT and IN, so it is at least twice the EP0 packet size (128 bytes for 64-byte packets).
- `HID::SendInputReport` 与 `CDC::SendSerialState` 新增 `in_isr` 参数 / `HID::SendInputReport`
  and `CDC::SendSerialState` take a new `in_isr` argument.
- 传入 `Database` 的 `GsUsbClass` 在构造时创建 `user_id` 键，`Database` 须先于它就绪，键添加失败为致命错误 / A `GsUsbClass` given a `Database` creates the `user_id` key at construction; the `Database` must be ready before it, and a failed key add is fatal.

## 文档 / Documentation

用法见 LibXR 文档中的 [XRUSB 协议栈](https://xrobot.work/docs/xrusb)。

Usage is described in [XRUSB Stack](https://xrobot.work/en/docs/xrusb) of the LibXR documentation.
