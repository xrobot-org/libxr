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
| `sim_findings.hpp` | 已知缺陷用例的运行入口 / runner for the known-defect cases |
| `usbip_server.hpp`, `usbip_lib.sh` | USB/IP 服务端和挂接函数 / USB/IP server and the attach helpers |
| `usb_sim_usbip.cpp`, `usb_sim_classes.cpp` | CDC 回环；HID、gs_usb、DFU、UAC 服务端 / CDC loopback; HID, gs_usb, DFU and UAC servers |
| `acm_echo.py`, `plug_cycle.sh`, `class_checks.py`, `classes_e2e.sh`, `uac_check.py`, `gs_burst.py` | 主机侧检查 / host-side checks |

## 帧中断 / The frame interrupt

候选线的 `DeviceCore` 没有"需求变化"回调，真实后端只在自己的重估点看 `WantsBusTime()`。仿真照做，
由 `Profile::sof_policy` 选择：

The candidate `DeviceCore` has no "need changed" hook, and the real backends look at `WantsBusTime()`
only at their own re-evaluation points. The simulation does the same, selected by `Profile::sof_policy`:

| 策略 / Policy | 后端 / Backends | 重估点 / Re-evaluation points |
| --- | --- | --- |
| `AT_STOPS` (默认 / default) | STM32 OTG、ESP32 DWC2、FSDEV | `Start()`、`OnStopsSettled()`；有待决停止时一直开 / while a stop is pending |
| `AT_CONTROL_READY` | CH32 OTG FS/HS | `OnControlReady()` 里打开，之后不关 / turned on there, never off |
| `ALWAYS` | 无（理想控制器）/ none (ideal controller) | 每帧都送，只用来把设备类和策略隔开 / every frame, to separate a class from the policy |

## 发现：不在默认运行里的用例 / Findings: cases kept out of the default run

下面的用例断言了候选线的库目前做不到的行为。断言没有放宽，也没有 `add_test`，默认 `ctest` 不跑它们。
测试程序不带参数时只跑常规用例；`--list-findings` 列出已知缺陷用例，`--finding <名字>` 单独运行，
断言失败时进程以 SIGABRT 退出并输出位置。库修好后同一条命令返回 0，用例应移进常规用例。条目编号来自
`C-missing-fixes.md`（Line A 的修复在候选线上缺什么）。

The cases below assert behaviour the candidate library does not have yet. Their assertions are not
weakened and they have no `add_test`, so the default `ctest` does not run them. A test program without
arguments runs the regular cases only; `--list-findings` lists the known-defect cases and
`--finding <name>` runs one, aborting with its location when an assertion fails. Once the library is fixed
the same command returns 0 and the case belongs among the regular ones. Item numbers refer to
`C-missing-fixes.md` (which Line A fixes the candidate still lacks).

| 程序 / Program | 用例 / Case | 条目 / Item | 现象 / Symptom |
| --- | --- | --- | --- |
| `test_usb_sim_profiles` | `uac_frame_ticks_otg_fs` | 2（帧中断跟随 `WantsBusTime()`，Line A 65cd2abf）/ 2 (frame interrupt follows `WantsBusTime()`, Line A 65cd2abf) | STM32 OTG 风格的后端：UAC 在 SET_INTERFACE 之后 12 帧只出 1 个包，帧中断没有打开 / UAC sends 1 packet in 12 frames after SET_INTERFACE, the frame interrupt is never turned on |
| `test_usb_sim_profiles` | `uac_frame_ticks_esp32s3` | 2 | 同上，ESP32-S3 风格 / same, ESP32-S3 style |
| `usb_sim_classes gsusb2` + `gs_burst.py`（手动 / manual） | gs_usb 突发 / gs_usb burst | 无编号（新）/ none (new) | 通道 0 突发时通道 1 的接收帧丢失，内核 `rx_over_errors` 增加；回显和接收轮流发送后不再丢 / channel 1 loses received frames during a burst on channel 0 and the kernel counts `rx_over_errors`; with echo and receive served alternately nothing is lost |
| `classes_e2e.sh`（`CLASSES=uac USBIP_SOF=stops`，手动 / manual） | UAC 经 `snd-usb-audio` 采集 / UAC capture through `snd-usb-audio` | 2 | 默认策略下 `arecord` 在约 0.6 s 后报 `read error: Input/output error`：561 个等时 IN URB 只有第一个带数据（一个 96 字节包）；`USBIP_SOF=control` 和 `always` 下 3 s 采集 144000 个样本连续无断点 / with the default policy `arecord` fails after about 0.6 s with `read error: Input/output error`: of 561 isochronous IN URBs only the first carries data (one 96-byte packet); with `USBIP_SOF=control` and `always` a 3 s capture gives 144000 continuous samples |

运行 / Run: `build/test/test_usb_sim_profiles --finding uac_frame_ticks_otg_fs`

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
`rx_over_errors`。服务端不限带宽时不丢帧；`USBIP_BULK_PER_MS` 设到 32 以下（一个全速帧装不下更多
24 字节的批量事务）后，通道 1 的接收帧大量丢失并上报溢出：
`GS_BUS_RATE=8 USBIP_BULK_PER_MS=24 build/test/usb_sim_classes gsusb2 3240`，挂接后
`python3 gs_burst.py 600`。原因在 `gs_usb.hpp`：主机帧一入队就回显，`PickFrame` 总是先发回显，
接收队列（默认 32 帧）满了就丢。这不是 `C-missing-fixes.md` 里的条目。

`usb_sim_classes gsusb2` is a two-channel gs_usb whose CAN bus delivers at most `GS_BUS_RATE` frames per
millisecond and returns channel 0's transmissions as received frames on channel 1. `gs_burst.py` sends a
burst on channel 0 as fast as the adapter accepts it and reports what channel 1 received and the
kernel's `rx_over_errors`. With an unlimited server nothing is lost; with `USBIP_BULK_PER_MS` at 32 or
below (one full-speed frame holds no more 24-byte bulk transactions) most of channel 1's received frames
are lost and the overflow is reported. The cause is in `gs_usb.hpp`: a host frame is echoed as soon as it
is queued, `PickFrame` always serves the echo first, and the receive queue (32 frames by default) drops
when full. This is not an item of `C-missing-fixes.md`.
