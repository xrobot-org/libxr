# SPI 手动测试 / Manual SPI test

将主机 MOSI 接到 MISO，调用 `ReadAndWrite()` 同时发送和接收已知数据。每个指定长度都分别检查阻塞、轮询和回调完成方式，逐字节验证接收数据和发送缓冲区。

Wire the master's MOSI to MISO and use `ReadAndWrite()` to transmit and receive known data simultaneously. For every selected length, check blocking, polling and callback completion, received bytes and the transmit buffer contents.

## 接线和准备 / Wiring and setup

初始化为 **8 位帧、全双工主机**，MISO 上不能同时连接其他输出。调用方完成时钟、模式、片选和中断／DMA 配置，并独占总线。先选择适合回环接线的保守速率。

Configure an **8-bit full-duplex master**, with no other output driving MISO. The caller configures clock, mode, chip select and interrupt/DMA resources and reserves the bus. Start at a conservative rate suitable for the loopback wiring.

传入两个独立的 RAM 工作缓冲区，不得相互重叠，也不能使用后端当前正在操作的缓冲区。各次传输的长度须满足实际容量、对齐和内存访问要求。`TestSPILoopback()` 不更改 SPI 配置。

Supply two separate RAM work buffers, disjoint from each other and the backend's active buffers. Transfer lengths must meet the actual capacity, alignment and memory-access requirements. `TestSPILoopback()` does not change the SPI configuration.

## 调用 / Usage

把 `test/` 和 `test/manual/driver/` 加入应用的头文件搜索路径，从普通任务调用。由应用提供缓冲区和适用的长度列表：

Add `test/` and `test/manual/driver/` to the application's include paths and call from a normal task. Supply the buffers and appropriate lengths from the application:

```cpp
#include "spi/test_spi.hpp"

void CheckSPI(LibXR::SPI& spi, LibXR::Semaphore& semaphore, LibXR::RawData tx,
              LibXR::RawData rx, std::initializer_list<size_t> lengths)
{
  LibXR::Test::TestSPILoopback(spi, semaphore, tx, rx, lengths);
}
```

`semaphore` 是阻塞方式等待完成所用的信号量，由调用方提供并保留，可供依次执行的各项测试复用。

`semaphore` is the semaphore for blocking completion. The caller provides and retains it and may reuse it for tests run one after another.

选择短包、较长包以及合法的边界长度。如果后端按长度选择不同传输方式，应由调用工程选择能触及这些分支的长度；通用测试不内置 DMA 阈值。

Select short, longer and valid boundary lengths. If the backend chooses a transfer path based on length, the calling project should select lengths that exercise those paths. The generic test contains no DMA threshold.

最后两个参数为 `timeout_ms` 和 `iterations`，默认 1000 ms 和 1000 轮。总传输次数为“长度数量 × 3 × 轮数”，最后一个参数填 1 可快速检查。

The final arguments are `timeout_ms` and `iterations`, defaulting to 1000 ms and 1000 rounds. Total transfers are “number of lengths × 3 × rounds”; use one round for a quick check.

## 检查内容 / Checks

发送内容随位置、长度、轮次和完成方式变化。接收区预先填入错误值，防止没有复制数据却误通过。完成后，收发内容都与生成规则比较，不会仅凭两个缓冲区相等就判定成功。

Transmit data varies with position, length, round and completion mode. Prefill the receive area with incorrect bytes to detect missing copies. After completion, compare both buffers against the generated pattern rather than merely checking that they match each other.

测试复用与 I2C 测试相同的完成等待辅助代码，允许同步或异步完成。回调只更新原子状态，所有报错在调用线程进行。超时参数不能强行中断后端调用内部的同步执行；失败触发 `TEST_ASSERT` 并停止，不带着未完成的操作返回。

The test reuses the completion-wait helper used by I2C tests and accepts inline or deferred completion. Callbacks only update atomic state; diagnostics run in the calling thread. The timeout cannot preempt synchronous work inside the backend call. Failure triggers `TEST_ASSERT` and stops rather than returning with a pending operation.

两个工作缓冲区都会被改写，正常结束后保留最后一帧数据。测试不创建辅助线程，回调和等待对象在轮次间复用；CI 不自动运行接线测试。

Both work buffers are overwritten and retain the final frame on success. No helper threads are created; callback and wait state are reused between rounds. CI does not run the wired test automatically.

## 配置和耗时 / Configuration and timing

`TestSPIConfig()` 设置一组配置，然后用阻塞方式连续全双工收发，检查接收内容和发送区保持不变，并返回每次传输耗时之和。计时只包住 `ReadAndWrite()` 调用，包含等待实际完成的时间，不包含数据准备和核对，也不添加逐次休眠。

`TestSPIConfig()` applies one configuration, repeats blocking full-duplex transfers, checks RX and unchanged TX contents, and returns the sum of transfer times. Each interval covers `ReadAndWrite()` through completion. Data preparation and checking are outside the interval; no per-transfer sleep is added.

仍使用 8 位全双工主机和 MOSI→MISO 接线，关闭双缓冲。传入等长、独立的工作缓冲区，其大小就是单次传输长度，须满足后端容量、对齐和内存访问要求。调用前总线须空闲，微秒时间基已初始化且分辨率足够。

Use an 8-bit full-duplex master with MOSI wired to MISO and double buffering disabled. Supply equal-size, separate work buffers; their size is the transfer length and must meet backend capacity, alignment and memory-access requirements. Start with an idle bus and an initialized microsecond timebase of sufficient resolution.

```cpp
uint64_t CheckSPIConfig(LibXR::SPI& spi, LibXR::Semaphore& semaphore,
                        LibXR::SPI::Configuration config, LibXR::RawData tx,
                        LibXR::RawData rx, uint64_t min_us, uint64_t max_us)
{
  // 100 次传输的耗时范围，由调用工程预先确定。
  // The calling project determines the time bounds for 100 transfers in advance.
  return LibXR::Test::TestSPIConfig(spi, semaphore, config, tx, rx, min_us, max_us);
}
```

最后两个可选参数是传输次数 `iterations`（默认 100）和单次阻塞超时 `timeout_ms`（默认 1000）。耗时上下限须在运行前确定：线路时间可按“字节数 × 8 × 次数 ÷ SCK 频率”估算，再计入调用、调度和硬件间隙。选择足够长的传输，让这些开销不至于掩盖快慢分频的差别。

The final optional arguments are `iterations` (100 by default) and `timeout_ms` (1000 per blocking call). Set bounds before running: estimate wire time as “bytes × 8 × transfers ÷ SCK frequency”, then allow for calls, scheduling and hardware gaps. Use transfers long enough that overhead does not hide the difference between fast and slow prescalers.

依次用 A、A、B、A 调用，可以检查重复设置、切换及恢复。也可传入不同的时钟极性和相位，检查设置后能否正常回环。正常返回后保留新配置，需要恢复时由调用方重新设置。

Call with A, A, B, A to check repeated setup, switching and restoration. Other polarity and phase settings can also be supplied to check loopback after setup. The new configuration remains active on return; the caller explicitly restores it when needed.

`GetBusSpeed()` 根据配置和后端报告的源时钟计算，不能单独证明分频已作用于硬件。实测耗时补充检查实际传输速度，但包含软件开销，不是精确的 SCK 频率测量。

`GetBusSpeed()` calculates a value from configuration and the reported source clock; it cannot alone prove that the hardware prescaler changed. Measured duration adds an observable transfer-rate check, but includes software overhead and is not a precise SCK frequency measurement.

## 验证范围 / Scope

这项回环检查等长全双工数据通路。它不验证单独 `Read()`／`Write()` 的空方向处理、`MemRead()`／`MemWrite()` 的从机协议、`Transfer()` 或双缓冲的专门行为。片选时序、实际时钟频率和模式是否符合外部设备要求，也不能仅靠同一控制器的自回环证明。

This loopback checks equal-length full-duplex data transfers. It does not validate empty-direction handling in separate `Read()`/`Write()` calls, slave protocols in `MemRead()`/`MemWrite()`, or the dedicated behavior of `Transfer()` and double buffering. Self-loopback on one controller also cannot independently prove chip-select timing, actual clock frequency or mode compliance with an external device.
