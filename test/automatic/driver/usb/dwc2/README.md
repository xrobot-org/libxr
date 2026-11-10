# DWC2 后端测试 / Tests of the DWC2 backends

STM32 OTG（`driver/st/stm32_usb_otg.cpp`）和 ESP32-S2/S3（`driver/esp/esp_usb*.cpp`）两个 USB 设备后端
用的都是 Synopsys DWC2 控制器。这里把**真实的后端源码**原样编译到主机上，让它读写一个控制器模型，
再由主机脚本在总线上发令牌，检查设备类依赖的端点契约。

The STM32 OTG backend (`driver/st/stm32_usb_otg.cpp`) and the ESP32-S2/S3 backend
(`driver/esp/esp_usb*.cpp`) both drive a Synopsys DWC2 controller. Here the **real backend source** is
compiled unchanged for the host and reads and writes a model of the controller, while a host script
sends tokens on the bus and the endpoint contract the device classes rely on is checked.

| 文件 / File | 内容 / Content |
| --- | --- |
| `mmio_trap.hpp` | 固定地址上的寄存器区：访问被拦截并单步执行，后端源码不用改 / register area at a fixed address: accesses are trapped and single-stepped, the backend source stays as it is |
| `fake_dwc2.hpp` | 控制器模型：寄存器、收发 FIFO、端点与核心中断、内部 DMA、总线事件 / controller model: registers, FIFOs, endpoint and core interrupts, internal DMA, bus events |
| `dwc2_host.hpp` | 主机脚本：控制传输、批量/中断读写、SOF、复位、挂起 / host script: control transfers, bulk and interrupt data, SOF, reset, suspend |
| `usb_probe_class.hpp` | 记录每一次完成的设备类 / device class that records every completion |
| `usb_backend_cases.hpp` | 两个后端共用的契约用例 / contract cases shared by both backends |
| `test_findings.hpp` | `--list-findings` / `--finding <名字>` 的入口 / runner for `--list-findings` and `--finding <name>` |
| `stm32/` | 假的 CubeMX `main.h`、后端用到的 LL HAL 函数、`test_stm32_usb_otg` / fake CubeMX `main.h`, the LL HAL functions the backend calls, `test_stm32_usb_otg` |
| `esp/` | 假的 ESP-IDF 头文件（ESP32-S3，IDF 5.3–5.5 的接口）、`test_esp_usb` / fake ESP-IDF headers (ESP32-S3, the IDF 5.3 to 5.5 API), `test_esp_usb` |

两个测试程序都在默认的 `ctest` 里（`stm32_usb_otg`、`esp_usb`），只在 Linux x86-64 上构建；其他平台和不支持
`-fstrict-volatile-bitfields` 的编译器（`test_esp_usb`）在 CMake 配置时打印一行 STATUS 说明跳过了它们。
Both programs are in the default `ctest` (`stm32_usb_otg`, `esp_usb`) and are built on Linux x86-64
only; on other platforms, and with a compiler that lacks `-fstrict-volatile-bitfields` (`test_esp_usb`),
CMake prints a STATUS line at configure time saying they are skipped.

## 控制器模型 / The controller model

模型按参考手册的设备编程模型写，不参照驱动的期望；手册没有写明的地方是模型的假设，在下面单列。
手册写明"必须"而驱动没有照做的操作
（对未使能的端点写 EPDIS、不经全局 OUT NAK 就禁用 OUT 端点、AHBIDL 为 0 时或上一次冲刷未完成时写 TXFFLSH、
冲刷期间写同一个发送 FIFO 或使能用它的端点、端点使能期间改写传输长度寄存器、
发送 FIFO 溢出、接收 FIFO 为空时弹出、DMA 访问可达内存之外的地址、总线复位后第一个令牌到来时
还没做完"USB 复位时的端点初始化"的 5 步）被记下来，每个用例结束时
检查没有这样的记录。手册没有给出时间的地方是模型的参数：FIFO 冲刷位读回几次才清零、EPDIS 写入
后过多久端点才禁用、DMA 模式下 SETUP 的事件何时出现；默认运行把这些取值都跑一遍。

The model follows the device programming model of the reference manual, not what a driver expects;
where the manual is silent the model makes an assumption, listed below.
Operations the manual marks as mandatory and a driver does not follow (EPDIS on an endpoint that is
not enabled, an OUT endpoint disabled without the global OUT NAK, TXFFLSH written while AHBIDL is 0 or
the previous flush has not finished, a transmit FIFO written or an endpoint using it enabled during its
flush, a transfer size register written
while the endpoint is enabled, a transmit FIFO overflow, a pop of the empty receive FIFO, a DMA
access outside the reachable memory, and the five steps of "Endpoint initialization on USB reset" not
done when the first token after a bus reset arrives) are recorded, and every case checks at its end that nothing
was recorded. Where the manual gives no timing the model has a parameter: how many reads still see a
FIFO flush bit set, how long EPDIS takes to disable the endpoint, when the SETUP event appears in DMA
mode; the default run goes through these values.

### 模型的假设 / Model assumptions

下面的行为手册没有写明，是模型的选择，不是手册的结论；它们在 `fake_dwc2.hpp` 里标为 `ASSUMPTION`。
真实控制器与此不同时，依赖它们的用例会给出错误的结论。

The behaviour below is not stated in the reference manual. It is a choice of the model, not a
conclusion of the manual, and is marked `ASSUMPTION` in `fake_dwc2.hpp`. Where a real controller
differs, the cases that depend on it draw a wrong conclusion.

| 行为 / Behaviour | 手册有什么 / What the manual has | 为什么这样选 / Why chosen | 依赖的用例 / Cases that depend on it |
| --- | --- | --- | --- |
| IISOIXFR 挂起期间 DSTS.FNSOF 仍是刚结束的那一帧 / DSTS.FNSOF still shows the frame that ended while IISOIXFR is pending | IISOIXFR 与 EOPF 一起置位；FNSOF 是"收到的 SOF 的帧号"，没有写更新时间 / IISOIXFR is raised with EOPF; FNSOF is "the frame number of the received SOF", no update time | 中断先于下一个 SOF 被处理时就是这样；驱动靠它判断哪个端点错过了帧 / what an interrupt serviced before the next SOF sees; the driver uses it to find the endpoint that missed its frame | `TestIsochronousMissedFrame`, `TestTwoIsochronousStreams` |
| 等时 IN 端点在 EPENA 为 0 或奇偶不是 EONUM 的帧里收到 IN 令牌时回零长包 / an isochronous IN endpoint answers a zero-length packet when EPENA is clear or the frame has the other parity than EONUM | 只有 FIFO 里缺数据（零长包）和 NAKSTS 置位（零长包）两种情形 / only a FIFO lacking the data and NAKSTS set (zero-length packet) are stated | 主机每帧都看到一个包，和有数据的帧一致 / the host sees one packet per frame, as in a frame with data | `TestIsochronousFrames`, `TestUacStreams`（ESP 的 `isochronous_frames`、`uac_streams` 发现也用它 / the ESP findings `isochronous_frames` and `uac_streams` use it too） |
| 3.00a 之后的核心在 DMA 模式收到 SETUP 包时置 XFRC\|STPKTRX / cores after 3.00a raise XFRC\|STPKTRX when the SETUP packet arrives in DMA mode | STPKTRX 只在 SETUP_COMPLETE 状态出现时（SETUP 阶段结束）置位 / STPKTRX is set only when the SETUP_COMPLETE status is seen (end of the SETUP stage) | 后端按核心版本分了支，两种时序都要跑（`early` 模式）/ the backend branches on the core revision, both timings run (the `early` modes) | 所有控制传输用例的 `DMA, early SETUP event` 模式 / the `DMA, early SETUP event` mode of every control case |
| 3.10a 核心在非 DMA 模式弹出 SETUP 数据状态字时置 XFRC\|STPKTRX / a 3.10a core raises XFRC\|STPKTRX when the SETUP data status word is popped without DMA | RM0385 写明 STPKTRX 只用于缓冲 DMA 模式 / RM0385 says STPKTRX is for buffer DMA mode only | 后端对 3.10a 非 DMA 有单独的判断 / the backend tests for core 3.10a without DMA | 所有控制传输用例的 `FIFO mode, core 3.10a` 模式 / the `FIFO mode, core 3.10a` mode of every control case |
| 发送 FIFO 冲刷：内容到 TXFFLSH 自清时才清掉；AHBIDL 在冲刷期间和 TXFFLSH 清零后的前两次读取里为 0（只在慢冲刷模式）/ transmit FIFO flush: the content is dropped when TXFFLSH clears; AHBIDL reads 0 while the flush runs and for the first two reads after TXFFLSH cleared (slow-flush modes only) | 应用写 TXFFLSH 之前要检查 AHBIDL，之后要等核心清掉该位（8 个时钟）；没有给出 AHBIDL 为 0 的时长 / the application must check AHBIDL before writing TXFFLSH and wait until the core clears the bit (eight clocks); no duration of AHBIDL = 0 is given | 让跳过 AHBIDL 检查或不等 TXFFLSH 清零的驱动能被发现；模型把这样的写入记为违反 / lets a driver that skips the AHBIDL check or does not wait for TXFFLSH be caught; the model records such writes as violations | 慢冲刷模式下的 `TestBusResetMidTransfer`、`TestStopIn`、`TestStopOut` 等所有走停止序列的用例 / every case that runs a stop sequence in the slow-flush modes, such as `TestBusResetMidTransfer`, `TestStopIn`, `TestStopOut` |

时间参数（冲刷位读回几次才清零、EPDIS 生效前的寄存器访问次数）也是手册没有给出的，
见上一节；默认运行把多个取值都跑一遍，所以不依赖某一个取值。
The timing parameters (reads that still see a flush bit set, register accesses before EPDIS acts) are
likewise not given by the manual, see the section above; the default run goes through several
values, so no case depends on one of them.

STM32 的测试跑 FIFO 模式和内部 DMA 两种方式、后端区分的三代核心（2.81a、3.10a、3.30a）、全速和
高速；ESP 的后端只有 DMA 一种方式。
The STM32 test runs FIFO mode and the internal DMA, the three core generations the backend
distinguishes (2.81a, 3.10a, 3.30a), full speed and high speed; the ESP backend only has the DMA
mode.

## 设备类用到的接口行为由哪个用例覆盖 / Which case covers what each class uses

用例名是 `usb_backend_cases.hpp` 里的函数；"发现"表示该后端目前做不到，用例不在默认运行里（见下一节）。
Case names are functions of `usb_backend_cases.hpp`; "finding" means the backend does not have the
behaviour yet and the case is not in the default run (see the next section).

| 接口行为 / Interface behaviour | 用到它的设备类 / Classes that use it | 用例 / Case | STM32 OTG | ESP32-S2/S3 |
| --- | --- | --- | --- | --- |
| 总线复位、枚举、SET_ADDRESS 在状态阶段之后生效、SET_CONFIGURATION / bus reset, enumeration, SET_ADDRESS effective after its status stage, SET_CONFIGURATION | 全部 / all | `TestEnumeration` | 默认 / default | 默认 / default |
| 控制读：单包、多包、包长整数倍时以零长包结束、按 wLength 截断 / control read: one packet, several, zero-length packet after a packet multiple, cut at wLength | 全部（描述符）；HID（报告描述符）、DFU（UPLOAD、GETSTATUS）、UAC（GET_CUR）、gs_usb、DAP v2（MS OS 2.0 描述符）/ all (descriptors); HID (report descriptor), DFU (UPLOAD, GETSTATUS), UAC (GET_CUR), gs_usb, DAP v2 (MS OS 2.0 descriptors) | `TestControlRead` | 默认 / default | 默认 / default |
| 控制写：单包和多包的 OUT 数据阶段、零长状态阶段 / control write: OUT data stage of one and several packets, zero-length status | CDC（SET_LINE_CODING）、HID（SET_REPORT）、DFU（DNLOAD）、UAC（SET_CUR）、gs_usb（位时序等）/ CDC (SET_LINE_CODING), HID (SET_REPORT), DFU (DNLOAD), UAC (SET_CUR), gs_usb (bit timing and others) | `TestControlWrite` | 默认 / default | 默认 / default |
| 无数据阶段的请求 / request without a data stage | CDC（SET_CONTROL_LINE_STATE）、HID（SET_IDLE）、DFU（CLRSTATUS、ABORT）、UAC（SET_INTERFACE）/ CDC (SET_CONTROL_LINE_STATE), HID (SET_IDLE), DFU (CLRSTATUS, ABORT), UAC (SET_INTERFACE) | `TestControlWrite` | 默认 / default | 默认 / default |
| 请求被 STALL，下一个 SETUP 恢复 / a stalled request, recovery on the next SETUP | 全部（不支持的请求）；DFU（出错状态）/ all (unsupported requests); DFU (error state) | `TestControlStall` | 默认 / default | 默认 / default |
| 带 OUT 数据阶段的请求被 STALL / a stalled request with an OUT data stage | DFU、UAC、gs_usb | `TestControlStallOutData` | 默认 / default | 默认 / default |
| 背靠背的 SETUP / back-to-back SETUPs | 全部（主机重试）/ all (host retries) | `TestControlBackToBackSetups` | 默认 / default | 默认 / default |
| SETUP 打断 IN 数据阶段 / SETUP during an IN data stage | 全部 / all | `TestControlSetupDuringInData` | 默认 / default | 默认 / default |
| SETUP 打断 OUT 数据阶段、代替控制读的状态阶段 / SETUP during an OUT data stage, in place of a control read status | 全部 / all | `TestControlSetupDuringOutData`, `TestControlSetupSkipsStatus` | 默认 / default | 默认 / default |
| SETUP 代替控制写的状态阶段 / SETUP in place of a control write status | 全部 / all | `TestControlSetupAfterWriteData` | 默认 / default | 默认 / default |
| 批量 IN：单包、多包、短包结束、要求的零长包、单独的零长包、主机不轮询 / bulk IN: one packet, several, short end, requested zero-length packet, zero-length alone, host not polling | CDC（含零长包）、gs_usb、DAP v2（含零长包）/ CDC (with the zero-length packet), gs_usb, DAP v2 (with the zero-length packet) | `TestBulkIn` | 默认 / default | 默认 / default |
| 批量 OUT：短包、收满、多包、零长包、数据比挂的接收长、未挂接收时 NAK / bulk OUT: short packet, exact fill, several packets, zero-length, more data than armed, NAK with nothing armed | CDC、gs_usb、DAP v2 | `TestBulkOut` | 默认 / default | 默认 / default |
| 中断 IN/OUT / interrupt IN and OUT | CDC（通知）、HID、DAP v1 / CDC (notifications), HID, DAP v1 | `TestInterrupt` | 默认 / default | 默认 / default |
| 完成回调恰好一次、回调里重新提交 / exactly one completion, re-arming inside the callback | CDC、gs_usb、HID、DAP v1、DAP v2 | `TestCallbackRearm`（以及其余用例的完成计数 / and the completion counts of the other cases） | 默认 / default | 默认 / default |
| 端点 STALL 与清除，数据翻转回到 DATA0，ENDPOINT_RESUMED / endpoint halt and clear, data toggle back at DATA0, ENDPOINT_RESUMED | CDC、gs_usb、HID、DAP v1、DAP v2 | `TestEndpointHalt` | 默认 / default | 默认 / default |
| 停止在途的 IN 请求（Flush、Close）/ stopping an IN request in flight (Flush, Close) | CDC、gs_usb、HID、DAP v1、DAP v2、UAC | `TestStopIn` | 默认 / default | 默认，报告的已发送长度是发现 / default; the reported sent length is a finding |
| 停止时主机取走最后一个包 / the host takes the last packet during the stop | 同上 / the same | `TestStopInRace` | 默认 / default | 发现 / finding |
| 停止在途的 OUT 请求 / stopping an OUT request in flight | CDC、gs_usb、HID、DAP v1、DAP v2 | `TestStopOut` | 默认 / default | 默认，不经全局 OUT NAK 是发现 / default; the missing global OUT NAK is a finding |
| 重新配置 1 → 0 → 1 / reconfiguration 1 → 0 → 1 | 全部 / all | `TestReconfigure` | 默认 / default | 默认，关闭的端点仍应答 NAK 是发现 / default; a closed endpoint that still answers NAK is a finding |
| 传输进行中总线复位；复位后端点不再 STALL、OUT 端点 NAK / bus reset in the middle of transfers; no STALL and NAKed OUT endpoints after a reset | 全部 / all | `TestBusResetMidTransfer`, `TestBusResetClearsHalt` | 默认 / default | 默认 / default |
| 处理别的中断时到来的总线复位 / a bus reset that arrives while another interrupt is handled | 全部 / all | `TestInterruptClearKeepsOthers`（ESP） | 未运行 / not run | 默认 / default |
| 挂起与恢复 / suspend and resume | 全部 / all | `TestSuspendResume` | 默认 / default | 默认 / default |
| 断开与重连、Stop()/Start() / disconnect and reconnect, Stop() and Start() | 全部；DFU（下载后重新枚举）/ all; DFU (enumeration after a download) | `TestDisconnectReconnect`（STM32）、`TestStopStart`（ESP） | 默认 / default | 默认 / default |
| 端点的停止序列进行中调用 Stop() / Stop() during the stop sequence of an endpoint | 全部 / all | `TestStopDuringInStop`（STM32） | 默认；非 DMA 的 OUT 端点是发现 / default; an OUT endpoint without DMA is a finding | 未运行 / not run |
| 帧节拍：枚举后帧中断就是开的，需要帧节拍的类每个 SOF 收到一次 OnFrameTick，不需要的类收不到；帧中断不随配置变化，挂起关、恢复开 / frame ticks: the frame interrupt is on after enumeration, a class that needs frame ticks gets one OnFrameTick per SOF and one that does not gets none; the frame interrupt ignores the configuration, off at suspend, on at resume | UAC | `TestFrameTicks`, `TestUacFrameInterruptIgnoresConfiguration`, `TestSuspendResume` | 默认 / default | 默认 / default |
| 核心对同一个 SOF 令牌把帧中断置位两次（STM32H723 实测，`Controller::sof_twice`）：每帧（高速时每微帧）仍只送一次 OnFrameTick，复位之后同一帧号的第一个 SOF 照常送出 / the core flags one SOF token twice (measured on an STM32H723, `Controller::sof_twice`): still one OnFrameTick per frame (microframe at high speed), and after a reset the first SOF with the same number is delivered | UAC | `TestFrameTicksWhenSofIsRaisedTwice` | 默认 / default | 不适用：ESP 后端没有这个处理，也没有硬件证据 / not applicable: the ESP backend has no such handling and there is no hardware evidence |
| 等时 IN：每帧一个包、奇偶帧 / isochronous IN: one packet per frame, even and odd frames | UAC | `TestIsochronousFrames`, `TestUacStreams` | 默认 / default | 发现 / finding |
| 等时 IN 错过了它的帧，以 FAILED 结束 / isochronous IN that missed its frame ends with FAILED | UAC | `TestIsochronousMissedFrame` | 默认 / default | 发现 / finding |
| 帧中断被关掉时流停止 / the stream stops with the frame interrupt off | UAC | `TestUacStopsWithoutFrameInterrupt` | 默认 / default | 未运行（依赖等时出流）/ not run (needs the isochronous stream) |

DFU 只用控制端点；等时 OUT 没有设备类使用，没有用例。
DFU uses the control endpoint only; no class uses isochronous OUT and no case covers it.

## 发现：不在默认运行里的用例 / Findings: cases kept out of the default run

下面的用例断言了后端目前做不到的行为。断言没有放宽，默认 `ctest` 不跑它们。`--list-findings` 列出，
`--finding <名字>` 单独运行，断言失败时进程以 SIGABRT 退出并输出位置；后端修好后同一条命令返回 0，
用例应移进常规用例。

The cases below assert behaviour a backend does not have yet. Their assertions are not weakened and
the default `ctest` does not run them. `--list-findings` lists them and `--finding <name>` runs one,
aborting with its location when an assertion fails; once the backend is fixed the same command
returns 0 and the case belongs among the regular ones.

| 程序 / Program | 用例 / Case | 手册或契约的要求 / What the manual or the contract requires | 后端的做法 / What the backend does |
| --- | --- | --- | --- |
| `test_stm32_usb_otg` | `dma_data_with_status_phase` | XFRC 表示编程的传输完成；STSPHSRX 表示主机进入了控制写的状态阶段 / XFRC reports the programmed transfer complete; STSPHSRX reports that the host entered the status stage of a control write | 两者在同一次中断里读到时，数据阶段的完成被当成"不是数据"丢掉 / read by the same interrupt, the completion of the data stage is dropped as "not data" |
| `test_stm32_usb_otg` | `stop_leaves_out_endpoint_enabled` | 禁用 OUT 端点要等全局 OUT NAK 生效后写 EPDIS；传输长度寄存器只在端点禁用时改写 / an OUT endpoint is disabled with EPDIS once the global OUT NAK is in effect; the transfer size register is written only while the endpoint is disabled | 非 DMA 模式下、停止序列还在等全局 OUT NAK 时调用 Stop()：停止被直接确认，端点在硬件里仍使能，下一次会话在使能状态下改写该寄存器 / without DMA, Stop() while a stop sequence waits for the global OUT NAK: the stop is confirmed at once, the endpoint stays enabled in the hardware and the next session rewrites the register while it is enabled |
| `test_esp_usb` | `stop_in_progress` | 完成里的长度是主机已取走的字节数 / the length in a completion is what the host took | 被停止的 IN 请求报告取进 FIFO 的字节数 / a stopped IN request reports the bytes fetched into the FIFO |
| `test_esp_usb` | `stop_in_race` | 只能对已使能的端点写 EPDIS；NAK 生效之前核心仍可能发出数据 / EPDIS only on an enabled endpoint; the core may still send data before the NAK is effective | 主机在停止过程中取走最后一个包后仍写 EPDIS，EPDISD 不会出现，停止超时 / EPDIS is written after the host took the last packet during the stop; EPDISD never comes and the stop times out |
| `test_esp_usb` | `out_disable_without_global_nak` | 禁用 OUT 端点之前先进入全局 OUT NAK / the global OUT NAK comes before an OUT endpoint is disabled | 直接写 SNAK 和 EPDIS / writes SNAK and EPDIS directly |
| `test_esp_usb` | `close_keeps_endpoint_active` | 停用端点要清 USBAEP，之后核心不应答发给它的令牌 / deactivating an endpoint clears USBAEP, after which the core ignores its tokens | 关闭端点只关中断屏蔽位，端点继续应答 NAK / closing an endpoint only drops the interrupt mask and the endpoint keeps answering NAK |
| `test_esp_usb` | `isochronous_frames`, `uac_streams` | 等时 IN 只在 EONUM 与帧号奇偶相同的帧里发数据，每次传输都要设置 / isochronous IN sends only in the frame whose parity equals EONUM, which is programmed for each transfer | 只在激活端点时设成偶数帧：UAC1 麦克风 3000 帧里发出 1500 个包 / set to even once when the endpoint is activated: the UAC1 microphone sends 1500 packets in 3000 frames |
| `test_esp_usb` | `isochronous_missed_frame` | 错过帧的等时 IN 由"未完成的等时 IN 传输"中断报告，应用禁用端点 / an isochronous IN that missed its frame is reported by the incomplete isochronous IN interrupt and the application disables the endpoint | 没有打开也没有处理这个中断，请求一直挂着 / the interrupt is neither unmasked nor handled and the request stays armed |

运行 / Run: `build/test/test_esp_usb --finding uac_streams`

`ctest` 里的 `stm32_usb_otg_findings` 和 `esp_usb_findings`（标签 `findings`，`ctest -L findings` 单独运行）
对每个程序执行 `--check-findings`：在子进程里逐个运行上表的用例，要求每个仍然在断言失败处中止，且断言
文字含有用例登记的片段（`test_findings.hpp` 的 `fails_with`）。某个发现开始通过，或因别的原因失败，这个
测试就失败；后端修好之后，把用例移进常规用例并从列表里删掉。

The ctests `stm32_usb_otg_findings` and `esp_usb_findings` (label `findings`; `ctest -L findings` runs
only them) execute `--check-findings` of each program: every case of the table above runs in a child
process and must still abort at a failed assertion whose text contains the fragment registered with the
case (`fails_with` in `test_findings.hpp`). The test fails when a finding starts to pass or fails for
another reason; once the backend is fixed, move the case among the regular ones and delete it from the
list.

## 内存检查 / Memory checking

两个测试在 AddressSanitizer 的默认设置下（含 LeakSanitizer）通过。后端和 `DeviceCore` 在构造和初始化时
创建的端点对象、描述符和回调块库里没有释放接口（初始化期分配、长期持有），`lsan_suppressions.hpp` 只
抑制这几个分配点；测试自己的对象在用例结束时释放，泄漏仍会被报告。

Both tests pass under the default AddressSanitizer settings, LeakSanitizer included. The endpoint
objects, descriptors and callback blocks that the backend and `DeviceCore` create at construction and
initialization have no release interface in the library (allocated once at initialization and kept);
`lsan_suppressions.hpp` suppresses just those allocation sites, and the objects of the test itself are
released at the end of a case, so a leak of theirs is still reported.

## 诊断输出 / Diagnostic output

设置环境变量 `DWC2_LOG` 时，任何用例都会把每次寄存器写和总线事件写到标准错误。

With the environment variable `DWC2_LOG` set, any case writes every register write and bus event to
standard error.

这两个程序不能在 gdb 里运行：寄存器访问靠处理器的单步陷阱拦截，而 gdb 自己占用它。排查问题用上面的
日志；不属于寄存器区的非法访问会打印地址、指令地址和调用栈。
The two programs do not run under gdb: register accesses are intercepted with the single-step trap
of the processor, which gdb takes for itself. Use the log above instead; an invalid access outside
the register area prints its address, the instruction address and a backtrace.
