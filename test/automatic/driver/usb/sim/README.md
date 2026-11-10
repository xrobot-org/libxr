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
| `test_usb_sim_profiles` | `uac_frame_ticks_otg_fs` | 2 | STM32 OTG 风格的后端：UAC 在 SET_INTERFACE 之后 12 帧只出 1 个包，帧中断没有打开 / UAC sends 1 packet in 12 frames after SET_INTERFACE, the frame interrupt is never turned on |
| `test_usb_sim_profiles` | `uac_frame_ticks_esp32s3` | 2 | 同上，ESP32-S3 风格 / same, ESP32-S3 style |

运行 / Run: `build/test/test_usb_sim_profiles --finding uac_frame_ticks_otg_fs`
