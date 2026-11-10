# USB 仿真后端 / USB simulation backend

`usb_sim.hpp` 是一个仿真的 USB 设备控制器：它只实现 `Endpoint`、`DeviceCore` 和 `EndpointPool`
对后端声明的契约，上面是真实的端点核心、DeviceCore 和设备类。主机侧按包驱动（SETUP、OUT、IN 令牌、
SOF、总线复位、挂起、恢复、断开）。违反契约的调用直接断言，所以仿真是契约的参考实现；和它不一致的
真实后端有 bug。

`usb_sim.hpp` is a simulated USB device controller. It implements only the backend contract that
`Endpoint`, `DeviceCore` and `EndpointPool` declare; everything above it is the real endpoint core,
DeviceCore and device classes. The host side works per packet (SETUP, OUT and IN tokens, SOF, bus reset,
suspend, resume, disconnect). A call that breaks the contract asserts, so the simulation is the
reference implementation of the contract and a real backend that differs from it has a bug.

| 文件 / File | 内容 / Content |
| --- | --- |
| `usb_sim.hpp` | 仿真控制器、主机事务层 / simulated controller and host transaction layer |
| `usb_sim_profiles.hpp` | 硬件档位：STM32F103 FSDEV、STM32F407 OTG FS、ESP32-S3、CH32 OTG FS / hardware profiles |
| `test_usb_sim_contract.cpp` | 后端契约测试 / backend contract tests |
| `test_usb_sim_profiles.cpp` | 档位上的设备类、控制器限制、帧中断 / classes on the profiles, controller limits, the frame interrupt |
| `usbip_server.hpp`, `usbip_lib.sh` | USB/IP 服务端和挂接函数 / USB/IP server and the attach helpers |
| `usb_sim_usbip.cpp`, `usb_sim_classes.cpp` | CDC 回环；HID、gs_usb、DFU、UAC 服务端 / CDC loopback; HID, gs_usb, DFU and UAC servers |
| `acm_echo.py`, `plug_cycle.sh`, `class_checks.py`, `classes_e2e.sh`, `uac_check.py`, `gs_burst.py` | 主机侧检查 / host-side checks |

## 帧中断 / The frame interrupt

真实后端只在自己的重估点看 `WantsBusTime()`：`Start()`、`OnStopsSettled()`，以及类的需要变化时
`DeviceCore` 调用的 `OnBusTimeNeedChanged()`。仿真照做，由 `Profile::sof_policy` 选择：

A real backend looks at `WantsBusTime()` only at its own re-evaluation points: `Start()`,
`OnStopsSettled()`, and `OnBusTimeNeedChanged()`, which `DeviceCore` calls when a class's need
changes. The simulation does the same, selected by `Profile::sof_policy`:

| 策略 / Policy | 后端 / Backends | 重估点 / Re-evaluation points |
| --- | --- | --- |
| `AT_STOPS` (默认 / default) | STM32 OTG、ESP32 DWC2、FSDEV | `Start()`、`OnStopsSettled()`、`OnBusTimeNeedChanged()`；有待决停止时一直开 / while a stop is pending |
| `AT_CONTROL_READY` | CH32 OTG FS/HS | `OnControlReady()` 里打开；`OnBusTimeNeedChanged()` 里随需要开关 / turned on there; on or off with the need in `OnBusTimeNeedChanged()` |
| `ALWAYS` | 无（理想控制器）/ none (ideal controller) | 每帧都送，只用来把设备类和策略隔开 / every frame, to separate a class from the policy |

真实后端重估帧中断开关的地方（读代码）：STM32 OTG 是 `Connect()`（`Start()`）、`OnStopsSettled()`、
启动待决停止时（`StopHardware` 里端点仍在使能）、TX FIFO 清空完成时（`PollTxFlush`，总线复位和端点
停止都会清空；`RequestTxFlush` 先无条件打开 SOFM），以及 `OnBusTimeNeedChanged()`；ESP32 是复位处理、
`OnStopsSettled()`、启动待决停止时和 `OnBusTimeNeedChanged()`。仿真的 `AT_STOPS` 只建模了
`Start()`、待决停止开始、`OnStopsSettled()` 和 `OnBusTimeNeedChanged()`，没有建模复位时的 TX FIFO 清空，
EP0 的停止也总是立即完成。

在 STM32H723（OTG_HS）上的测量是：没有 `OnBusTimeNeedChanged()` 时枚举之后帧中断已经打开，取消配置后
仍然开着，UAC 麦克风连续出流；有它时取消配置后关闭、重新配置后打开，出流相同。读代码解释了后两点：
取消配置时端点上没有待决停止，后端没有别的重估点，所以开关不变。枚举之后是什么让它打开没有确定：读代码
只能列出候选（绑定之后的某个待决停止，或者总线复位时 TX FIFO 清空在类还处于绑定状态时完成），没有寄存器
跟踪。因此仿真的 STM32 OTG、ESP32 档位在没有 `OnBusTimeNeedChanged()` 时枚举之后帧中断是关的，这一点与
测量不符，不能用来推断真实后端上 UAC 会在一个包之后停下。用例检查的是通知带来的变化：配置后打开、
取消配置后关闭（`TestFrameInterruptFollowsTheConfiguration`），运行中才需要帧时间的类得到 SOF
（`test_usb_sim_contract.cpp` 的 `TestFrameInterruptFollowsARuntimeNeed`）。

Where the real backends re-evaluate the frame interrupt enable (from reading the code): on STM32 OTG
`Connect()` (`Start()`), `OnStopsSettled()`, when a pending stop starts (`StopHardware` with the endpoint
still enabled), when a TX FIFO flush completes (`PollTxFlush`; a bus reset and an endpoint stop both
flush, and `RequestTxFlush` turns SOFM on unconditionally first) and `OnBusTimeNeedChanged()`; on ESP32
the reset handling, `OnStopsSettled()`, when a pending stop starts and `OnBusTimeNeedChanged()`. The
simulated `AT_STOPS` models only `Start()`, the start of a pending stop, `OnStopsSettled()` and
`OnBusTimeNeedChanged()`; it does not model the TX FIFO flush at a bus reset, and an EP0 stop always
completes at once.

Measured on an STM32H723 (OTG_HS): without `OnBusTimeNeedChanged()` the frame interrupt is already on
after enumeration and stays on after the deconfiguration, and the UAC microphone streams continuously;
with it, the deconfiguration turns it off, configuring turns it on again, and streaming is the same.
Reading the code explains the last two points: at the deconfiguration no stop is pending on the
endpoints, the backend has no other re-evaluation point, and the enable stays as it was. What turns it on
after enumeration was not determined: the code only gives candidates (a pending stop after the binding,
or the TX FIFO flush of a bus reset completing while the class is still bound), and there is no register
trace. So without `OnBusTimeNeedChanged()` the simulated STM32 OTG and ESP32 profiles have the frame
interrupt off after enumeration, which differs from the measurement, and the simulation must not be read
as saying that a UAC stops after one packet on a real backend. The cases check what the notification
changes: on once configured and off after the deconfiguration
(`TestFrameInterruptFollowsTheConfiguration`), and a class that needs bus time only at run time gets
SOFs (`TestFrameInterruptFollowsARuntimeNeed` in `test_usb_sim_contract.cpp`).

## 经 USB/IP 交给内核 / Handing the device to the kernel over USB/IP

`usb_sim_usbip` 和 `usb_sim_classes` 不是 ctest 目标，需要带 `vhci-hcd` 的 Linux 主机：先
`sudo modprobe vhci-hcd`，启动服务端，再 `sudo usbip --tcp-port 3240 attach -r 127.0.0.1 -b 1-1`。
脚本里的挂接、摘除和设备选择都在 `usbip_lib.sh`：端口从导入时的远端 URL 找，设备按 sysfs 路径在
vhci 之下判定，从不只凭端口号或 VID:PID，因为主机上可能有同 ID 的真实设备（例如真实的 gs_usb
适配器）。脚本以调用者身份运行，`sudo -n` 只用于 `usbip` 和仿真适配器接口上的 `ip link`。

`usb_sim_usbip` and `usb_sim_classes` are not ctest targets and need a Linux host with `vhci-hcd`:
`sudo modprobe vhci-hcd`, start the server, then
`sudo usbip --tcp-port 3240 attach -r 127.0.0.1 -b 1-1`. Attaching, detaching and picking the device
are in `usbip_lib.sh`: the port is found from the remote URL used at import and a device is the one
whose sysfs path lies under vhci, never the port number or VID:PID alone, because the host may have a
real device with the same ids (a real gs_usb adapter, say). The scripts run as the calling user;
`sudo -n` is for `usbip` and for `ip link` on the simulated adapter's interface only.

| 环境变量 / Variable | 作用 / Effect |
| --- | --- |
| `USBIP_SOF=stops\|control\|always` | 仿真控制器的帧中断策略（上表），默认 `stops` / frame interrupt policy of the simulated controller, `stops` by default |
| `USBIP_BULK_PER_MS=N` | 每毫秒最多完成 N 个批量/中断事务（全速总线一帧的容量），0 为不限 / at most N bulk or interrupt transactions per millisecond, 0 is unlimited |
| `USBIP_TRACE=1`, `USBIP_RECORD=<file>`, `USBIP_BIND=<addr>` | URB 跟踪、录制、监听地址 / URB trace, recording, listen address |
| `GS_BUS_RATE=N` | `gsusb2` 的总线每毫秒送出的帧数，默认 8 / frames per millisecond of the `gsusb2` bus, 8 by default |

`classes_e2e.sh <build-dir>` 对每个类报告 PASS、FAIL 或 COULD NOT RUN（退出码 0、1、3）。hidraw、
usbfs（dfu-util）和 ALSA 采集在多数主机上需要 root 或 audio 组；打开失败时报告 COULD NOT RUN 并附上
内核能看到的部分（驱动绑定、读到的描述符），不算通过。`plug_cycle.sh` 反复挂接 CDC 设备并在传输中途
摘除。CDCUart 在拔插后保留发送队列里没交给端点的字节，所以重新打开串口后开头可能先收到上一次连接
留下的数据（`acm_echo.py` 的 `DRAIN` 就是为此）。

`classes_e2e.sh <build-dir>` reports PASS, FAIL or COULD NOT RUN per class (exit status 0, 1, 3).
hidraw, usbfs (dfu-util) and ALSA capture need root or the audio group on most hosts; when the open
fails the class is COULD NOT RUN together with what the kernel did show (the driver bound, the
descriptors it read), never a pass. `plug_cycle.sh` attaches the CDC device over and over and detaches
it in the middle of a transfer. CDCUart keeps the bytes of its transmit queue that were not yet handed
to the endpoint across a replug, so a reopened port may first receive data left over from the previous
connection (`DRAIN` in `acm_echo.py` is for that).

`CLASS_PRIV="sudo -n"` 给 `classes_e2e.sh` 里恰好三个程序加前缀，并且每个都只指向仿真设备：
`class_checks.py hid`（只打开 vhci 之下的 hidraw 节点）、`dfu-util`（`-d 1209:0005 -S <序列号> -p <vhci
设备的总线端口>`，序列号和端口从 vhci 下的设备读出）和 `arecord`（USB id 为 1209:0006 且 sysfs 路径在
vhci 之下的声卡）。`CLASSES="hid gsusb dfu uac"` 选择要跑的类。`<build-dir>` 要用绝对路径。

`CLASS_PRIV="sudo -n"` prefixes exactly three programs in `classes_e2e.sh`, each already limited to the
simulated device: `class_checks.py hid` (opens only a hidraw node under vhci), `dfu-util` (`-d 1209:0005
-S <serial> -p <bus-port of the vhci device>`, serial and port read from the device under vhci) and
`arecord` (the sound card with USB id 1209:0006 whose sysfs path lies under vhci). `CLASSES="hid gsusb dfu
uac"` selects the classes. `<build-dir>` must be an absolute path.

## gs_usb 突发实验 / gs_usb burst experiment

`usb_sim_classes gsusb2` 是两通道 gs_usb，CAN 总线每毫秒最多送出 `GS_BUS_RATE` 帧，把通道 0 发出的
帧作为通道 1 的接收帧交回。`gs_burst.py` 在通道 0 上尽快发一批帧，报告通道 1 收到什么和内核的
`rx_over_errors`。`USBIP_BULK_PER_MS` 设到 32 以下（全速总线一帧能装下的 24 字节批量事务不超过
32 个）时，USB 的事务数比 CAN 总线更紧：
`GS_BUS_RATE=8 USBIP_BULK_PER_MS=24 build/test/usb_sim_classes gsusb2 3240`，挂接后
`python3 gs_burst.py 600`。预期结果是通道 1 收到全部 600 帧、按序、无缺号，`rx_over_errors` 为 0：
IN 端点上回显帧和接收帧在两类都有待发时轮流发送（`gs_usb.hpp` 的 `PickFrame`），突发期间通道 1 的
接收帧每两个事务至少得到一个。如果 IN 端点总是先发回显，通道 1 的接收帧在突发期间得不到事务，接收
队列（每通道 32 帧）满了就丢帧，内核会报告 `rx_over_errors` 并且缺号。`test_gsusb.cpp` 的
`TestHostBurstOnARateLimitedBus` 是同一实验的确定性版本，`TestEchoAndReceivedFramesAlternate` 检查
轮流的顺序，两者都在默认 `ctest` 里。

`usb_sim_classes gsusb2` is a two-channel gs_usb whose CAN bus delivers at most `GS_BUS_RATE` frames per
millisecond and returns channel 0's transmissions as received frames on channel 1. `gs_burst.py` sends a
burst on channel 0 as fast as the adapter accepts it and reports what channel 1 received and the
kernel's `rx_over_errors`. With `USBIP_BULK_PER_MS` at 32 or below (a full-speed frame holds at most 32
bulk transactions of 24 bytes) the USB transactions are scarcer than the CAN bus:
`GS_BUS_RATE=8 USBIP_BULK_PER_MS=24 build/test/usb_sim_classes gsusb2 3240`, then
`python3 gs_burst.py 600` after attaching. The expected result is that channel 1 receives all 600
frames, in order, with no gap and `rx_over_errors` 0: on the IN endpoint, echo frames and received
frames are sent alternately while both kinds are pending (`PickFrame` in `gs_usb.hpp`), so during the
burst channel 1's received frames get at least every second transaction. If the IN endpoint always
sent echoes first, channel 1's received frames would get no transaction during the burst, the receive
queue (32 frames per channel) would overflow and drop frames, and the kernel would report
`rx_over_errors` and the sequence would have gaps. `TestHostBurstOnARateLimitedBus` in `test_gsusb.cpp`
is the deterministic version of the same experiment and `TestEchoAndReceivedFramesAlternate` checks the
order of the alternation; both are part of the default `ctest`.
