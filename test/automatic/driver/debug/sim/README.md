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
| `test_dap_replay.cpp`, `replay/*.txt` | 回放 OpenOCD、pyOCD、probe-rs 录下的会话（ctest `dap_replay`）/ replay of sessions recorded from OpenOCD, pyOCD and probe-rs |
| `dap_sim_usbip.cpp` | 仿真探针经 USB/IP 交给内核，真实调试器可以连 / the simulated probe over USB/IP, for a real debugger |
| `dap_e2e.sh`, `dap_record.sh`, `openocd_dap_sim.cfg` | 端到端会话与录制 / end-to-end sessions and recording |
| `swd_target_bitbang.cpp`, `openocd_calibrate.sh`, `pyocd_calibrate.sh`, `openocd_sim.cfg` | 用真实 OpenOCD / pyOCD 校准目标模型 / calibrating the model with real OpenOCD / pyOCD |
| `keil_sim.ini` | Keil uVision 的调试初始化脚本，只在 Windows 上用，Linux 上没有跑过 / Keil uVision debug init script, Windows only, not run on Linux |

## 与候选线库的一处差别 / One difference from the candidate library

1. 每个 `DAP_QueueCommands` / `DAP_ExecuteCommands` 包就地执行并立即应答，应答是
   `DAP_ExecuteCommands` 的格式 `[0x7F, n, ...]`，能力字节不带原子命令位（0x10）。候选线的
   README 迁移说明和 `test_daplink_v2.cpp` 的 `TestQueueAndExecuteAnswerPerPacket` 都这样规定，所以
   随机测试不再有"排队包的应答随触发包"，回放测试只在 `DAP_Info(CAPABILITIES)` 的应答里不比较
   0x10 这一位（`00 01 11` 对 `00 01 01`），其余字节逐字节比较。Line A 的做法见
   `C-missing-fixes.md` 第 12a 项。

1. Every `DAP_QueueCommands` / `DAP_ExecuteCommands` packet runs in place and is answered at once, in the
   `DAP_ExecuteCommands` format `[0x7F, n, ...]`, and the capabilities byte carries no atomic-commands
   bit (0x10). The candidate's README migration note and `TestQueueAndExecuteAnswerPerPacket` in
   `test_daplink_v2.cpp` say so; the random test therefore has no "responses of queued packets come with
   the trigger", and the replay test leaves out only that bit of the `DAP_Info(CAPABILITIES)` response
   (`00 01 11` against `00 01 01`) and compares every other byte exactly. Line A's behaviour is item 12a
   of `C-missing-fixes.md`.

## 在 Linux 主机上用真实调试器 / Real debuggers on a Linux host

`dap_e2e.sh <build-dir>` 启动 `dap_sim_usbip`，经 vhci 挂接，用 OpenOCD 跑两个会话（寄存器、64 KiB 下载、
复位、单步；带和不带注入的 WAIT），目标侧的协议错误和总线冲突计数必须为 0。`dap_record.sh` 录下会话供
回放。`openocd_calibrate.sh` 用 remote_bitbang 把目标模型对着真实 OpenOCD 校准，不经过 USB。主机上有真实的
CMSIS-DAP 探针时设备的选择规则见 `dap_e2e.sh` 开头：VID:PID 1209:0002 在主机上必须只属于一个设备且在
vhci 之下，`DAP_PRIV="sudo -n"` 只给调试器加前缀；pyOCD 和 probe-rs 会打开找到的每个探针，所以只有
`DAP_TOOLS_SCAN_OK=1` 时才运行。

`dap_e2e.sh <build-dir>` starts `dap_sim_usbip`, attaches it through vhci and runs two OpenOCD sessions
(registers, 64 KiB download, reset, step; with and without injected WAITs); the target's protocol-error
and contention counters must be 0. `dap_record.sh` records sessions for the replay. `openocd_calibrate.sh`
calibrates the target model against real OpenOCD over remote_bitbang, without USB. When the host has real
CMSIS-DAP probes, the selection rule is at the top of `dap_e2e.sh`: VID:PID 1209:0002 must belong to
exactly one device on the host and it must lie under vhci, and `DAP_PRIV="sudo -n"` prefixes the debugger
only; pyOCD and probe-rs open every probe they find, so they run only with `DAP_TOOLS_SCAN_OK=1`.
