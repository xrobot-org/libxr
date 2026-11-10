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

两个测试程序都在默认的 `ctest` 里（`stm32_usb_otg`、`esp_usb`），只在 Linux x86-64 上构建。
Both programs are in the default `ctest` (`stm32_usb_otg`, `esp_usb`) and are built on Linux x86-64
only.

## 控制器模型 / The controller model

模型按参考手册的设备编程模型写，不参照驱动的期望。手册写明"必须"而驱动没有照做的操作
（对未使能的端点写 EPDIS、不经全局 OUT NAK 就禁用 OUT 端点、端点使能期间改写传输长度寄存器、
发送 FIFO 溢出、接收 FIFO 为空时弹出、DMA 访问可达内存之外的地址）被记下来，每个用例结束时
检查没有这样的记录。手册没有给出时间的地方是模型的参数：FIFO 冲刷位读回几次才清零、EPDIS 写入
后过多久端点才禁用、DMA 模式下 SETUP 的事件何时出现；默认运行把这些取值都跑一遍。

The model follows the device programming model of the reference manual, not what a driver expects.
Operations the manual marks as mandatory and a driver does not follow (EPDIS on an endpoint that is
not enabled, an OUT endpoint disabled without the global OUT NAK, a transfer size register written
while the endpoint is enabled, a transmit FIFO overflow, a pop of the empty receive FIFO, a DMA
access outside the reachable memory) are recorded, and every case checks at its end that nothing
was recorded. Where the manual gives no timing the model has a parameter: how many reads still see a
FIFO flush bit set, how long EPDIS takes to disable the endpoint, when the SETUP event appears in DMA
mode; the default run goes through these values.

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
| 带 OUT 数据阶段的请求被 STALL / a stalled request with an OUT data stage | DFU、UAC、gs_usb | `TestControlStallOutData` | 默认 / default | 发现 / finding |
| 背靠背的 SETUP / back-to-back SETUPs | 全部（主机重试）/ all (host retries) | `TestControlBackToBackSetups` | 默认 / default | 发现 / finding |
| SETUP 打断 IN 数据阶段 / SETUP during an IN data stage | 全部 / all | `TestControlSetupDuringInData` | 默认 / default | 发现 / finding |
| SETUP 打断 OUT 数据阶段、代替控制读的状态阶段 / SETUP during an OUT data stage, in place of a control read status | 全部 / all | `TestControlSetupDuringOutData`, `TestControlSetupSkipsStatus` | 默认 / default | 默认 / default |
| SETUP 代替控制写的状态阶段 / SETUP in place of a control write status | 全部 / all | `TestControlSetupAfterWriteData` | 默认 / default | 发现 / finding |
| 批量 IN：单包、多包、短包结束、要求的零长包、单独的零长包、主机不轮询 / bulk IN: one packet, several, short end, requested zero-length packet, zero-length alone, host not polling | CDC（含零长包）、gs_usb、DAP v2（含零长包）/ CDC (with the zero-length packet), gs_usb, DAP v2 (with the zero-length packet) | `TestBulkIn` | 默认 / default | 默认 / default |
| 批量 OUT：短包、收满、多包、零长包、数据比挂的接收长、未挂接收时 NAK / bulk OUT: short packet, exact fill, several packets, zero-length, more data than armed, NAK with nothing armed | CDC、gs_usb、DAP v2 | `TestBulkOut` | 默认 / default | 默认 / default |
| 中断 IN/OUT / interrupt IN and OUT | CDC（通知）、HID、DAP v1 / CDC (notifications), HID, DAP v1 | `TestInterrupt` | 默认 / default | 默认 / default |
| 完成回调恰好一次、回调里重新提交 / exactly one completion, re-arming inside the callback | CDC、gs_usb、HID、DAP v1、DAP v2 | `TestCallbackRearm`（以及其余用例的完成计数 / and the completion counts of the other cases） | 默认 / default | 默认 / default |
| 端点 STALL 与清除，数据翻转回到 DATA0，ENDPOINT_RESUMED / endpoint halt and clear, data toggle back at DATA0, ENDPOINT_RESUMED | CDC、gs_usb、HID、DAP v1、DAP v2 | `TestEndpointHalt` | 默认 / default | 默认 / default |
| 停止在途的 IN 请求（Flush、Close）/ stopping an IN request in flight (Flush, Close) | CDC、gs_usb、HID、DAP v1、DAP v2、UAC | `TestStopIn` | 默认 / default | 默认，报告的已发送长度是发现 / default; the reported sent length is a finding |
| 停止时主机取走最后一个包 / the host takes the last packet during the stop | 同上 / the same | `TestStopInRace` | 默认 / default | 发现 / finding |
| 停止在途的 OUT 请求 / stopping an OUT request in flight | CDC、gs_usb、HID、DAP v1、DAP v2 | `TestStopOut` | 默认 / default | 默认，不经全局 OUT NAK 是发现 / default; the missing global OUT NAK is a finding |
| 重新配置 1 → 0 → 1 / reconfiguration 1 → 0 → 1 | 全部 / all | `TestReconfigure` | 默认 / default | 默认，关闭的端点仍应答 NAK 是发现 / default; a closed endpoint that still answers NAK is a finding |
| 传输进行中总线复位 / bus reset in the middle of transfers | 全部 / all | `TestBusResetMidTransfer` | 默认 / default | 默认 / default |
| 挂起与恢复 / suspend and resume | 全部 / all | `TestSuspendResume` | 默认 / default | 默认 / default |
| 断开与重连、Stop()/Start() / disconnect and reconnect, Stop() and Start() | 全部；DFU（下载后重新枚举）/ all; DFU (enumeration after a download) | `TestDisconnectReconnect`（STM32）、`TestStopStart`（ESP） | 默认 / default | 默认 / default |
| IN 端点的停止序列进行中调用 Stop() / Stop() during the stop sequence of an IN endpoint | 全部 / all | `TestStopDuringInStop`（STM32） | 默认 / default | 未运行 / not run |
| 帧节拍：每个 SOF 一次 OnBusTime，帧中断跟随 WantsBusTime() / frame ticks: one OnBusTime per SOF, the frame interrupt follows WantsBusTime() | UAC | `TestFrameTicks`, `TestUacFrameInterruptFollowsConfiguration` | 默认 / default | 默认 / default |
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
| `test_esp_usb` | `setup_after_write_data` | DMA 模式下要使能 EP0 OUT 才能接收 SETUP / in DMA mode EP0 OUT must be enabled to receive a SETUP | 控制写的数据收完后 EP0 OUT 处于禁用，直到状态阶段完成；主机放弃状态阶段时新的 SETUP 收不到 / EP0 OUT stays disabled from the end of a control write data stage to the end of its status stage; a SETUP sent instead of the status is not received |
| `test_stm32_usb_otg` | `dma_data_with_status_phase` | XFRC 表示编程的传输完成；STSPHSRX 表示主机进入了控制写的状态阶段 / XFRC reports the programmed transfer complete; STSPHSRX reports that the host entered the status stage of a control write | 两者在同一次中断里读到时，数据阶段的完成被当成"不是数据"丢掉 / read by the same interrupt, the completion of the data stage is dropped as "not data" |
| `test_esp_usb` | `control_stall_out_data` | DMA 模式下要使能 EP0 OUT 才能接收 SETUP / in DMA mode EP0 OUT must be enabled to receive a SETUP | 带 OUT 数据阶段的请求被 STALL 之后没有重新使能，下一个 SETUP 收不到 / not enabled again after a stalled request with an OUT data stage, the next SETUP is not received |
| `test_esp_usb` | `back_to_back_setups` | 处理最后收到的那个 SETUP / the last SETUP received is the one to process | 总是读缓冲区开头，即第一个 SETUP / always reads the start of the buffer, which is the first SETUP |
| `test_esp_usb` | `setup_during_in_data` | 停止 IN 传输要禁用端点并冲刷发送 FIFO / stopping an IN transfer takes the endpoint disable and a transmit FIFO flush | SETUP 打断 IN 数据阶段时只清软件状态，旧包留在 FIFO 里，先于新请求的数据发出 / a SETUP that ends an IN data stage only clears the software state; the old packet stays in the FIFO and goes out before the data of the new request |
| `test_esp_usb` | `stop_in_progress` | 完成里的长度是主机已取走的字节数 / the length in a completion is what the host took | 被停止的 IN 请求报告取进 FIFO 的字节数 / a stopped IN request reports the bytes fetched into the FIFO |
| `test_esp_usb` | `stop_in_race` | 只能对已使能的端点写 EPDIS；NAK 生效之前核心仍可能发出数据 / EPDIS only on an enabled endpoint; the core may still send data before the NAK is effective | 主机在停止过程中取走最后一个包后仍写 EPDIS，EPDISD 不会出现，停止超时 / EPDIS is written after the host took the last packet during the stop; EPDISD never comes and the stop times out |
| `test_esp_usb` | `stop_in_immediate_disable` | 等待 EPDISD 中断 / wait for the EPDISD interrupt | 写 EPDIS 之后在同一次中断里读出并清掉全部端点中断位：此时已经置位的 EPDISD 没被处理就清掉了 / after writing EPDIS the same interrupt reads and clears every endpoint interrupt bit: an EPDISD already set is cleared unhandled |
| `test_esp_usb` | `out_disable_without_global_nak` | 禁用 OUT 端点之前先进入全局 OUT NAK / the global OUT NAK comes before an OUT endpoint is disabled | 直接写 SNAK 和 EPDIS / writes SNAK and EPDIS directly |
| `test_esp_usb` | `close_keeps_endpoint_active` | 停用端点要清 USBAEP，之后核心不应答发给它的令牌 / deactivating an endpoint clears USBAEP, after which the core ignores its tokens | 关闭端点只关中断屏蔽位，端点继续应答 NAK / closing an endpoint only drops the interrupt mask and the endpoint keeps answering NAK |
| `test_esp_usb` | `isochronous_frames`, `uac_streams` | 等时 IN 只在 EONUM 与帧号奇偶相同的帧里发数据，每次传输都要设置 / isochronous IN sends only in the frame whose parity equals EONUM, which is programmed for each transfer | 只在激活端点时设成偶数帧：UAC1 麦克风 3000 帧里发出 1500 个包 / set to even once when the endpoint is activated: the UAC1 microphone sends 1500 packets in 3000 frames |
| `test_esp_usb` | `isochronous_missed_frame` | 错过帧的等时 IN 由"未完成的等时 IN 传输"中断报告，应用禁用端点 / an isochronous IN that missed its frame is reported by the incomplete isochronous IN interrupt and the application disables the endpoint | 没有打开也没有处理这个中断，请求一直挂着 / the interrupt is neither unmasked nor handled and the request stays armed |
| `test_esp_usb` | `interrupt_clear_loses_reset` | 中断状态位写 1 清除 / interrupt status bits are cleared by writing 1 | 用位域赋值清一个位，读改写把此刻挂起的其他位一起清掉：处理挂起中断时到来的总线复位丢失 / clears one bit with a bit-field assignment; the read-modify-write clears every other pending bit: a bus reset that arrives while the suspend interrupt is handled is lost |

运行 / Run: `build/test/test_esp_usb --finding uac_streams`

## 帧中断的诊断输出 / Diagnostic output for the frame interrupt

`test_stm32_usb_otg --trace-frame-interrupt [N [R [P]]]`（`test_esp_usb` 相同）让 UAC1 麦克风从上电走到
取消配置，每次写中断屏蔽寄存器时打印阶段、帧中断屏蔽位的新旧值和写寄存器的指令及其调用者的地址
（可执行文件内的偏移，用 `addr2line -f -C -e <程序> <偏移>` 解析）。N 是冲刷位写 1 之后仍读到 1 的
次数，R 为 1 时复位的开始和结束在同一次中断里被看到，P 为 1 时主机在开流之前先选中备用设置 1 再选回 0
且不取数据。设置环境变量 `DWC2_LOG` 时，任何用例都会把每次寄存器写和总线事件写到标准错误。

`test_stm32_usb_otg --trace-frame-interrupt [N [R [P]]]` (and the same for `test_esp_usb`) takes a UAC1
microphone from power-up to the deconfiguration and prints, at every write of the interrupt mask
register, the phase, the old and new value of the frame interrupt mask and the addresses of the
writing instruction and its callers (offsets into the executable, resolved with
`addr2line -f -C -e <program> <offset>`). N is how many reads still see a flush bit set after it was
written, R = 1 makes one interrupt see both the start and the end of the reset, and P = 1 makes the
host select alternate setting 1 and then 0 again, without taking data, before it starts the stream.
With the environment variable `DWC2_LOG` set, any case writes every register write and bus event to
standard error.

这两个程序不能在 gdb 里运行：寄存器访问靠处理器的单步陷阱拦截，而 gdb 自己占用它。排查问题用上面的
日志；不属于寄存器区的非法访问会打印地址、指令地址和调用栈。
The two programs do not run under gdb: register accesses are intercepted with the single-step trap
of the processor, which gdb takes for itself. Use the log above instead; an invalid access outside
the register area prints its address, the instruction address and a backtrace.
