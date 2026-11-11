# 调试探针仿真 / Debug probe simulation

`swd_target.hpp` 是一个在引脚级工作的 SWD 目标：ADIv5 SW-DP、AHB-AP 和最小的 Cortex-M 调试寄存器。
每个 SWCLK 上升沿它采样 SWDIO，并在轮到它时驱动下一位，所以主机侧的时序错误（空闲周期把 SWDIO
驱动成高电平、转向周期少一个）会直接表现为错位或总线冲突。它有三种接法：`SimSwd`（实现 `Debug::Swd`，
按位生成波形）、`SimSwclkPin` / `SimSwdioPin`（给 `SwdGeneralGPIO` 这类位操作引擎）和
`swd_target_bitbang`（remote_bitbang 服务端，给真实 OpenOCD 校准模型）。上面是真实的 DAP v2 类和
USB 仿真后端（`../../usb/sim/`）。

`swd_target.hpp` is an SWD target that works at the pin level: ADIv5 SW-DP, an AHB-AP and minimal
Cortex-M debug registers. On every SWCLK rising edge it samples SWDIO and drives the next bit where its
turn comes, so a timing mistake on the host side (an idle cycle driving SWDIO high, a missing turnaround
cycle) shows up as misalignment or bus contention. It connects three ways: `SimSwd` (implements
`Debug::Swd`, generates the waveform bit by bit), `SimSwclkPin` / `SimSwdioPin` (for bit-banging engines
such as `SwdGeneralGPIO`) and `swd_target_bitbang` (a remote_bitbang server for real OpenOCD to calibrate
the model). On top of it sit the real DAP v2 class and the USB simulation backend (`../../usb/sim/`).

| 文件 / File | 内容 / Content |
| --- | --- |
| `swd_target.hpp` | 目标模型和三种接法 / target model and its connections |
| `test_swd_target.cpp` | 目标模型和 `SwdGeneralGPIO` 的单元测试（ctest `swd_target`）/ unit tests of the model and of `SwdGeneralGPIO` |
| `test_dap_fuzz.cpp` | DAP v2 随机测试，经 USB 仿真后端到 SWD 目标（ctest `dap_fuzz`）/ randomised DAP v2 test through the USB simulation to the SWD target |
| `test_dap_replay.cpp`, `replay/*.txt` | 回归基准：回放 OpenOCD、pyOCD、probe-rs 录下的会话，应答与录制逐字节比较（ctest `dap_replay`）/ regression baseline: replay of sessions recorded from OpenOCD, pyOCD and probe-rs, the responses compared byte for byte with the recording |
| `dap_sim_usbip.cpp` | 仿真探针经 USB/IP 交给内核，真实调试器可以连 / the simulated probe over USB/IP, for a real debugger |
| `dap_e2e.sh`, `dap_record.sh`, `openocd_dap_sim.cfg` | 端到端会话与录制 / end-to-end sessions and recording |
| `swd_target_bitbang.cpp`, `openocd_calibrate.sh`, `pyocd_calibrate.sh`, `openocd_sim.cfg` | 用真实 OpenOCD / pyOCD 校准目标模型 / calibrating the model with real OpenOCD / pyOCD |
| `keil_sim.ini` | Keil uVision 的调试初始化脚本，只在 Windows 上用，Linux 上没有跑过 / Keil uVision debug init script, Windows only, not run on Linux |

## 回放是回归基准 / The replay is a regression baseline

`replay/*.txt` 里的应答是录制那天本实现自己的输出，真实工具只证明当时的会话跑通了（镜像对得上）。
回放能发现命令层、流水线或 SWD 引擎的行为变了，发现不了录制时就已经错的应答，它不是一致性测试。

The responses in `replay/*.txt` are the output of this implementation on the day of the recording; the
real tool only showed that the session completed (the memory image matched). The replay finds a change in
the command layer, the pipelining or the SWD engine; it cannot find a response that was already wrong when
it was recorded, and it is not a conformance test.

## 原子命令组与录制的会话 / Atomic command groups and the recorded sessions

`replay/*.txt` 里的会话是用一个声明了原子命令能力位（`DAP_Info(CAPABILITIES)` 的 0x10）的固件录的，
那种固件把 `DAP_QueueCommands` 包的应答推迟到触发它们的 `DAP_ExecuteCommands` 包。这里的 DAP v2 类
不声明 0x10：每个 `DAP_QueueCommands` / `DAP_ExecuteCommands` 包就地执行并立即应答，应答是
`DAP_ExecuteCommands` 的格式 `[0x7F, n, ...]`（见 `src/driver/usb/README.md` 的迁移说明和
`test_daplink_v2.cpp` 的 `TestQueueAndExecuteAnswerPerPacket`）。因此：

- `test_dap_replay` 只在 `DAP_Info(CAPABILITIES)` 的应答里不比较 0x10 这一位（录制是 `00 01 11`，
  这里是 `00 01 01`），其余字节逐字节比较；录制里没有排队命令。
- `test_dap_fuzz` 期望每个组包由它自己的应答就地回答，不等触发包。

The sessions in `replay/*.txt` were recorded with firmware that announced the atomic-commands capability
bit (0x10 in `DAP_Info(CAPABILITIES)`); such firmware defers the responses of `DAP_QueueCommands` packets
to the `DAP_ExecuteCommands` packet that triggers them. The DAP v2 class here does not announce 0x10:
every `DAP_QueueCommands` / `DAP_ExecuteCommands` packet runs its commands in place and is answered at
once, in the `DAP_ExecuteCommands` format `[0x7F, n, ...]` (see the migration notes in
`src/driver/usb/README.md` and `TestQueueAndExecuteAnswerPerPacket` in `test_daplink_v2.cpp`). As a
consequence:

- `test_dap_replay` leaves out only the 0x10 bit of the `DAP_Info(CAPABILITIES)` response (`00 01 11` in
  the recordings, `00 01 01` here) and compares every other byte exactly; the recordings contain no
  queued commands.
- `test_dap_fuzz` expects every group packet to be answered by its own response in place, without
  waiting for a trigger packet.

## 在 Linux 主机上用真实调试器 / Real debuggers on a Linux host

`dap_e2e.sh <build-dir>` 启动 `dap_sim_usbip`，经 vhci 挂接，用 OpenOCD 跑两个会话（寄存器、64 KiB 下载、
复位、单步；带和不带注入的 WAIT），目标侧的协议错误和总线冲突计数必须为 0。`dap_record.sh` 录下会话供
回放。`openocd_calibrate.sh` 用 remote_bitbang 把目标模型对着真实 OpenOCD 校准，不经过 USB。主机上有真实的
CMSIS-DAP 探针时设备的选择规则见 `dap_e2e.sh` 开头：VID:PID 1209:0002 在主机上必须只属于一个设备且在
vhci 之下，`DAP_PRIV="sudo -n"` 只给调试器加前缀；pyOCD 和 probe-rs 会打开找到的每个探针，所以只有
`DAP_TOOLS_SCAN_OK=1` 时才运行。

`dap_e2e.sh` 和 `dap_record.sh` 自己启动 `dap_sim_usbip`，用空闲的 TCP 端口和随机 busid，并在挂接前确认
连上的是自己启动的服务端（`../../usb/sim/usbip_lib.sh`），不会连到系统 usbipd 的默认端口。缺 usbip、
`sudo -n`、openocd 或 `vhci_hcd` 模块时退出码为 3（COULD NOT RUN），1 是检查失败。


`dap_e2e.sh <build-dir>` starts `dap_sim_usbip`, attaches it through vhci and runs two OpenOCD sessions
(registers, 64 KiB download, reset, step; with and without injected WAITs); the target's protocol-error
and contention counters must be 0. `dap_record.sh` records sessions for the replay. `openocd_calibrate.sh`
calibrates the target model against real OpenOCD over remote_bitbang, without USB. When the host has real
CMSIS-DAP probes, the selection rule is at the top of `dap_e2e.sh`: VID:PID 1209:0002 must belong to
exactly one device on the host and it must lie under vhci, and `DAP_PRIV="sudo -n"` prefixes the debugger
only; pyOCD and probe-rs open every probe they find, so they run only with `DAP_TOOLS_SCAN_OK=1`.

`dap_e2e.sh` and `dap_record.sh` start `dap_sim_usbip` themselves, on a free TCP port with a random busid,
and check that they reached the server they started before they attach (`../../usb/sim/usbip_lib.sh`);
the stock port of a system usbipd is never used. A missing usbip, `sudo -n`, openocd or `vhci_hcd`
module exits with status 3 (COULD NOT RUN); 1 is a failed check.
