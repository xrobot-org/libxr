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
只有平台无关的代码，USB 设备控制器的驱动位于各平台的 `driver/<平台>/`，例如 `driver/st/stm32_usb_otg.cpp`、
`driver/ch/ch32_usb_otghs.cpp` 和 `driver/esp/esp_usb_dev.cpp`。

XRUSB is the USB device stack of [LibXR](https://github.com/xrobot-org/libxr), written in C++20,
in `src/driver/usb`. `core/` builds the device, configuration, string and BOS descriptors and
manages the endpoint pool; `device/` holds the device core, the composite device and the device
classes. A composite device can contain several device classes, and each class takes the
endpoints it needs from the endpoint pool at initialization. This directory holds only
platform-independent code; the USB device controller drivers are in `driver/<platform>/` of each
platform, such as `driver/st/stm32_usb_otg.cpp`, `driver/ch/ch32_usb_otghs.cpp` and
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

设备类在总线复位、拔插或重新配置时对用户侧数据的处理：`HID::SendInputReport` 在设备未配置期间立即返回失败，不排队；`GsUsbClass` 在主机复位通道（MODE RESET）时丢弃该通道排着队的接收帧、错误帧、回显、被回显队列压住的回显以及因 CAN 发送队列满而暂存的主机帧，主机启动通道（MODE START）时再丢弃一次（覆盖没有收到复位的重新绑定），没有启动的通道的帧不交给 IN 端点，最后一个已启动的通道复位时清空共享 IN 端点（已经打包进槽里的帧不会在下次打开时交给主机），其他通道的帧和回显保留；`UAC1MicrophoneQ` 在主机开始推流（切到发送音频的 alt 设置）时丢弃之前写入的 PCM，录音从最新的数据开始；`DapLinkV2Class` 丢弃绑定变化前收到的命令，调试端口、时钟等在下一个会话的第一条命令之前由 Step 线程恢复默认值，注册了会话复位闸门并且闸门返回 false 时除外。

How the device classes treat data on the user side across a bus reset, a cable replug or a reconfiguration: `HID::SendInputReport` fails at once while the device is not configured and queues nothing; `GsUsbClass` drops the receive frames, error frames and echoes of a channel, the echo held by echo-queue back-pressure, and the host frames staged because the CAN transmit queue was full, when the host resets the channel (MODE RESET) and again when it starts it (MODE START, which also covers a rebind that got no reset), never hands the frames of a channel that is not started to the IN endpoint, flushes the shared IN endpoint when the last started channel is reset (so the frames already packed into its slots do not reach the host at the next open), and keeps the frames and echoes of the other channels; `UAC1MicrophoneQ` drops the PCM written before the host starts streaming (selects the alt setting that sends audio), so a capture starts from fresh data; `DapLinkV2Class` drops the commands received before the binding changed, and the Step thread puts the debug port, clock and the rest back to their defaults before the first command of the next session, unless a session reset gate is registered and returns false.

## 设备控制器 / Device Controllers

| 平台 / Platform | 控制器 / Controller | 驱动 / Driver | 测试设备 / Tested on |
| --- | --- | --- | --- |
| STM32 | FSDEV（`USB_BASE`、`USB_DRD_FS`） | `driver/st/stm32_usb_fsdev.cpp` | STM32F103、STM32G431 |
| STM32 | OTG FS（`USB_OTG_FS`） | `driver/st/stm32_usb_otg.cpp` | STM32F407 |
| STM32 | OTG HS（`USB_OTG_HS`） | `driver/st/stm32_usb_otg.cpp` | STM32F407、STM32H750 |
| ESP32-S2/S3 | OTG FS | `driver/esp/esp_usb_dev.cpp` | ESP32-S2, ESP32-S3 |
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
- 设备构造函数按平台接收端点列表。CH32 OTG FS/HS 接收 RX、TX 两个列表，下标就是端点号（EP0 第一项），
  EP0 的 RX[0] 与 TX[0] 是同一块，缺席的方向传 `{nullptr, 0}`。STM32 OTG FS/HS 接收 `HardwareConfig`
  （PHY、DMA、VBUS 检测、是否以高速枚举、使用 DMA 时的 EP0 OUT 缓冲）、RX FIFO 字节数、RX 列表和
  `{槽内存, TX FIFO 字节数}` 的 TX 列表；STM32 FSDEV 接收 `{槽内存, PMA 字节数}` 的 RX、TX 列表。用
  CodeGenerator 生成的 STM32 工程需要配套版本的 CodeGenerator。ESP32-S2/S3 保持逐端点声明 / Device
  constructors take endpoint lists per platform. CH32 OTG FS/HS take an RX and a TX list indexed by
  endpoint number (EP0 first); EP0's RX[0] and TX[0] are the same block, and an absent direction
  passes `{nullptr, 0}`. STM32 OTG FS/HS take a `HardwareConfig` (PHY, DMA, VBUS sensing, whether to
  enumerate at high speed, the EP0 OUT buffer used with DMA), the RX FIFO size, an RX list and a TX
  list of `{slot memory, TX FIFO bytes}`; STM32 FSDEV takes RX and TX lists of
  `{slot memory, PMA bytes}`. STM32 projects generated by the CodeGenerator need the matching
  CodeGenerator release. ESP32-S2/S3 keeps the per-endpoint declaration.
- `driver/st` 以 `-Wl,--wrap=HAL_PCD_Init` 和 `-Wl,--wrap=HAL_PCD_IRQHandler` 链接，CubeMX 生成的代码
  保持原样：`MX_USB_*_PCD_Init()` 不再初始化控制器，CubeMX 生成的中断函数进入 libxr 的驱动 /
  `driver/st` links with `-Wl,--wrap=HAL_PCD_Init` and `-Wl,--wrap=HAL_PCD_IRQHandler`, so the CubeMX
  code stays as generated: `MX_USB_*_PCD_Init()` no longer initializes the controller and the CubeMX
  interrupt handler enters the libxr driver.
- 只有 ESP32 的时基为驱动注册毫秒节拍（ESP32-S2/S3 USB 后端的停止超时用它）；STM32、CH32、HPM 和 MSPM0
  的时基与之前相同，不占用 `HAL_IncTick`、`vApplicationTickHook` 或 `SysTick_Handler`，也不要求
  `configUSE_TICK_HOOK` / Only the ESP32 timebase registers a millisecond tick for the drivers (the
  ESP32-S2/S3 USB backend's stop timeout uses it); the STM32, CH32, HPM and MSPM0 timebases are unchanged
  and take none of `HAL_IncTick`, `vApplicationTickHook` or `SysTick_Handler`, nor do they require
  `configUSE_TICK_HOOK`.
- DFU 运行时和 DFU 引导程序不创建 Timer 任务：擦写等慢操作和 DETACH 后的跳转都在应用调用 `Process()`
  的上下文里进行，`DFUClass::Process()` 是新接口 / The DFU runtime and the DFU bootloader create no Timer
  task: erase/program and the jump after DETACH run in the context that calls `Process()`;
  `DFUClass::Process()` is new.
- CDC：`CDCUart` 接受任何 7 字节的 SET_LINE_CODING；`CDCToUart` 不再 STALL，应用 UART 能接受的最接近的
  配置，GET_LINE_CODING 返回实际生效的值 / CDC: `CDCUart` accepts every 7-byte SET_LINE_CODING;
  `CDCToUart` never STALLs it, applies the nearest configuration the UART accepts, and GET_LINE_CODING
  returns the applied value.
- ESP32-S2/S3 的端点缓冲必须在 DMA 可用的内存里，起点按 USB DMA 对齐（`CONFIG_USB_ALIGN_SIZE`，或 L1 cache
  line）对齐，构造时检查。EP0 缓冲对半分给 OUT 和 IN，至少 2 倍 EP0 包长（64 字节包长用 128 字节）/
  On ESP32-S2/S3 the endpoint buffers must be in DMA-capable memory and start on the USB DMA alignment
  (`CONFIG_USB_ALIGN_SIZE`, or the L1 cache line); the constructor checks it. The EP0 buffer is split in
  halves for OUT and IN, so it is at least twice the EP0 packet size (128 bytes for 64-byte packets).
- 后端从 `DeviceCore<Capabilities>` 派生：`FullSpeedCapabilities` 或 `HighSpeedCapabilities` 给出支持的
  速度和 BOS 功能。原来的 `DeviceCore::Context` 改名为 `ControlContext`。设备核心在总线复位
  时读一次总线速度；控制器在速度协商结束前就报告复位时，后端要在速度确定后再报告一次复位 / Backends
  derive from `DeviceCore<Capabilities>`: `FullSpeedCapabilities` or `HighSpeedCapabilities` names the
  supported speeds and the BOS feature. The former `DeviceCore::Context` is now
  `ControlContext`. The device core reads the bus speed once, at the bus reset; a controller that
  reports the reset before speed negotiation ends has to report the reset again once the speed is
  known.
- 后端的帧中断（SOF）在设备上总线且没有挂起时一直打开：启动或总线复位、枚举完成时打开，挂起、断开和
  停止时关闭，恢复时重新打开，不看设备类是否需要帧节拍。停止超时计帧、FIFO 冲刷轮询、逐帧检查停止都
  依赖它。`DeviceCore::OnSof` 只在有设备类的 `NeedsFrameTick()` 为真时才向它发节拍事件 / A backend keeps
  its frame interrupt (SOF) on while the device is on the bus and not suspended: on at start or at the bus
  reset and enumeration done, off at a suspend, a disconnect and a stop, on again at the resume,
  whatever the classes need. The stop timeout frame count, the FIFO flush polling and the per-frame stop
  check depend on it. `DeviceCore::OnSof` posts the tick event only while some class's `NeedsFrameTick()`
  is true.
- 端点完成回调的签名是 `Callback<const Endpoint::Completion&>`，经 `Endpoint::SetOnComplete()` 设置，
  `Completion` 带结果、实际字节数和 OUT 租约的槽。IN 数据用 `AcquireSlot()` 借槽、写入后 `Submit()`；OUT
  数据用 `Receive()` 挂接收，完成时借出的槽用 `ReturnSlot()` 归还并重新挂接收，或用 `ReleaseSlot()` 只
  归还。每次借出的句柄只在这一次借出内有效，重复归还或用旧句柄归还得到 `STATE_ERR` / The endpoint
  completion callback is `Callback<const Endpoint::Completion&>`, set with `Endpoint::SetOnComplete()`;
  a `Completion` carries the result, the byte count and the slot of an OUT lease. IN data borrows a
  slot with `AcquireSlot()`, fills it and calls `Submit()`; OUT data is armed with `Receive()`, and the
  slot lent with a completion goes back with `ReturnSlot()` (rearming) or `ReleaseSlot()` (no rearm).
  A handle is valid only for the lend that produced it; a double return or a return through an older
  handle gets `STATE_ERR`.
- 解绑、Flush 或 STALL 结束的请求以 `ErrorCode::ABORTED` 完成；解绑之前借出的槽在 `Submit()` 或
  `ReturnSlot()` 时得到 `ABORTED`，槽回到空闲 / Requests ended by an unbind, a flush or a STALL complete
  with `ErrorCode::ABORTED`; a slot lent before an unbind gets `ABORTED` from `Submit()` or
  `ReturnSlot()` and becomes free.
- 类回调（BindEndpoints/UnbindEndpoints、OnDeviceEvent、控制请求、OnService、OnFrameTick、端点完成）都在
  设备的 owner 中执行。owner 由赢得 `SerializedService::Invoke` 的调用者执行：控制器中断，或者任何调用
  Submit/Receive/ReturnSlot/RequestClassService/Init 的应用线程，所以每个回调都可能在中断里
  （`in_isr=true`）或任意应用线程里（`in_isr=false`）运行，必须不阻塞。线程持有 owner 时，中断只发布工作，
  要等这个线程处理 / Class callbacks (BindEndpoints/UnbindEndpoints, OnDeviceEvent, control requests,
  OnService, OnFrameTick, endpoint completions) all run in the device's owner. The owner pass runs in
  whichever caller wins `SerializedService::Invoke`: the controller interrupt, or any application
  thread that calls Submit/Receive/ReturnSlot/RequestClassService/Init. Every callback can therefore run
  in an interrupt (`in_isr=true`) or in an arbitrary application thread (`in_isr=false`), and must not
  block. While a thread owns, interrupt work is only published and waits for that thread.
- 总线复位以 `DeviceEvent::RESET` 通知各设备类（`OnDeviceEvent`）；后端不另外提供复位计数 / A bus reset
  reaches every class as `DeviceEvent::RESET` (`OnDeviceEvent`); backends provide no separate reset
  counter.
- `DapLinkV1Class` 和 `DapLinkV2Class` 不在 USB 上下文里执行命令：应用起一个线程循环调用 `Step()`，例如
  `while (true) { dap.Step(); }`，SWD/JTAG 操作只在这个线程里发生（SWD 引脚须能在该线程所在的核上
  使用），延时类命令也在这个线程里等待。Bind/Unbind 只发布会话开始/结束，由 Step 线程关闭 SWD、复位 DAP
  状态；绑定变化前取到的命令被丢弃而不执行 / `DapLinkV1Class` and `DapLinkV2Class` do not execute
  commands in the USB context: the application starts one thread that calls `Step()` in a loop, for
  example `while (true) { dap.Step(); }`. All SWD/JTAG work happens in that thread (the SWD pins must be
  usable from its core), and delay-type commands wait in it. Bind/unbind only publish the start or end
  of a session; the Step thread closes SWD and resets the DAP state, and a command taken before the
  binding changed is dropped, not executed.
- `DapLinkV2Class` 去掉了模板参数 `QueuedRequestBufferSize` 和 `QueuedCommandCountMax`：每个
  DAP_QueueCommands / DAP_ExecuteCommands 包就地执行自己的命令 / `DapLinkV2Class` drops the template
  parameters `QueuedRequestBufferSize` and `QueuedCommandCountMax`: every DAP_QueueCommands /
  DAP_ExecuteCommands packet runs its own commands in place.
- `DapLinkV1Class::Step()` 是新接口，DAP v1 不再注册 Timer 任务 / `DapLinkV1Class::Step()` is new, and
  DAP v1 no longer registers a Timer task.
- `DapLinkV2Class` 的两个数据端点都用借用模式：OUT 每个包借给回调、拷进命令池后立即归还，两个槽交替接收；
  IN 方向的响应直接写进一个空闲的 IN 槽并提交，只有两个 IN 槽都在用时才先写进类里的暂存区，等 IN 槽空出后
  拷进去提交。通告给主机的 DAP 包长取配置值与 IN 槽容量中较小者，所以 IN 端点的缓冲块要不小于 2 倍 DAP
  包长（1024 字节包长用 2048 字节）；在途命令数为 4 / `DapLinkV2Class` uses borrow mode on both data
  endpoints: every OUT packet is lent to the callback, copied into the command pool and given back at
  once, and the two slots alternate. A response is written straight into a free IN slot and
  submitted; only while both IN slots are busy is it staged in the class first and copied into an IN
  slot when one is free. The DAP packet size announced to the host is the smaller of the configured
  value and an IN slot, so the IN endpoint block must be at least twice the DAP packet size (2048 bytes
  for a 1024 byte packet); four commands are in flight.
- 串口上的 DAP 传输（`DapSerialTransport`）不在 libxr 中，由 XR-DAP 固件提供 / The DAP transport over a
  serial link (`DapSerialTransport`) is not part of libxr; the XR-DAP firmware provides it.
- MS OS 2.0 描述符集由 `DeviceComposition` 生成：类用 `GetWinUsbInterfaceGuid()` 声明自己需要 WinUSB，
  单个功能的非复合设备用设备级形式（否则 Windows 不绑定，问题代码 28），其余用配置/功能子集形式。
  `DapLinkV2Class::GetWinUsbMsOs20DescriptorSet` 和 DAP 自带的 BOS 能力已删除，改用
  `DeviceComposition::WinUsbBosCapability()`。DFU 类的构造函数去掉了 WinUSB vendor code 参数，vendor code
  由组合设备的 WinUSB BOS 能力给出 / The MS OS 2.0 descriptor set is built by `DeviceComposition`: a class declares that it
  wants WinUSB through `GetWinUsbInterfaceGuid()`, a single function on a non-composite device gets the
  device-level form (otherwise Windows does not bind, problem code 28) and every other case the
  configuration/function subset form. `DapLinkV2Class::GetWinUsbMsOs20DescriptorSet` and the BOS
  capability of the DAP class are gone, use `DeviceComposition::WinUsbBosCapability()`. The DFU class
  constructors no longer take a WinUSB vendor code; the WinUSB BOS capability of the
  composition supplies it.
- HID 的 SET_IDLE 只记录空闲率，可由 `GetIDLERate()` 读出，类不按空闲率重发报告 / HID SET_IDLE only
  records the idle rate, readable through `GetIDLERate()`; the class does not resend reports by it.
- `HID::SendInputReport` 与 `CDC::SendSerialState` 新增 `in_isr` 参数 / `HID::SendInputReport`
  and `CDC::SendSerialState` take a new `in_isr` argument.
- `GsUsbClass` 的 echo 表示“设备的 CAN 驱动已接受这一帧”。CAN 发送队列满（`AddMessage` 返回 `FULL`）时，帧进入该通道自己的暂存队列（构造参数 `tx_stage_size`，默认 10，即 Linux gs_usb 驱动每通道的在途帧上限），暂不回 echo，OUT 端点照常接收，另一个通道不受影响；类服务和 SOF 节拍（只在有暂存帧时请求，全速 1 ms，高速 125 us）按顺序重试，驱动接受后才回 echo。每通道的流控靠主机每通道的在途帧上限；超过容量的帧被丢弃，仍然回 echo。通道复位、重新启动或总线复位时，该通道暂存的帧被丢弃，都不回 echo；通道复位时该通道排着队的接收帧、错误帧和 echo 也丢弃（主机关闭通道时已经释放了 tx 上下文），最后一个通道复位时共享 IN 端点上排队和在途的帧一起结束；还没交给 CAN 驱动的不发送，驱动已接受的队头已经交出，仍可能发到总线上 / An echo of `GsUsbClass` means "the device's CAN driver accepted the frame". When the CAN transmit queue is full (`AddMessage` returns `FULL`), the frame goes to its channel's own staging queue (constructor argument `tx_stage_size`, default 10, the per-channel in-flight limit of the Linux gs_usb driver) and gets no echo yet, the OUT endpoint keeps receiving, and the other channel is not affected; the class service and the SOF tick (requested only while frames are staged; 1 ms at full speed, 125 us at high speed) retry in order, and the echo is sent after the driver accepted the frame. Per-channel flow control rests on the host's per-channel in-flight limit; a frame beyond the capacity is dropped and still echoed. A channel reset, a restart of the channel or a bus reset drops the frames staged for it, none is echoed; a channel reset also drops the channel's queued receive frames, error frames and echoes (the host freed its tx contexts when it closed the channel), and the reset of the last started channel ends the frames queued and in flight on the shared IN endpoint; those not yet handed to the CAN driver are never sent, while an accepted head frame has been handed over and may still go out on the bus.
- 传入 `Database` 的 `GsUsbClass` 在构造时创建 `user_id` 键，`Database` 须先于它就绪，键添加失败为致命错误 / A `GsUsbClass` given a `Database` creates the `user_id` key at construction; the `Database` must be ready before it, and a failed key add is fatal.

## 文档 / Documentation

用法见 LibXR 文档中的 [XRUSB 协议栈](https://xrobot.work/docs/xrusb)。

Usage is described in [XRUSB Stack](https://xrobot.work/en/docs/xrusb) of the LibXR documentation.
