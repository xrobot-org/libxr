# 系统手动测试 / Manual system tests

这些测试通过 LibXR 公共 API 检查实际系统，由用户选择调用，不进入 CI 的自动运行清单。测试需要支持并发任务的系统；不适用于 `Thread::Create` 同步执行的裸机后端。

These tests use LibXR public APIs on the target system. The user selects and calls them; CI does not run them. They require concurrent tasks and do not support the bare-metal backend whose `Thread::Create` runs synchronously.

## 使用方法 / Usage

先完成系统初始化，再从普通任务中调用测试。需要协作任务的测试借用用户提供的 `LibXR::Test::ManualWorker&`（定义在 `test/manual/test_worker.hpp`），要求它处于空闲状态且测试期间不被其他代码使用。测试不会在每轮创建辅助线程，也不会调用 `Join`；返回前会确认任务已经完成。

Initialize the system, then call the tests from a normal task. Tests that need a cooperating task borrow a caller-provided `LibXR::Test::ManualWorker&` (defined in `test/manual/test_worker.hpp`). It must be idle and reserved for the test until it returns. Tests neither create helper threads per round nor call `Join`; they confirm job completion before returning.

把 `test/` 和本目录加入应用的头文件搜索路径，例如：

Add `test/` and this directory to the application's include paths, for example:

```cmake
target_include_directories(app PRIVATE
  path/to/libxr/test
  path/to/libxr/test/manual/system
)
```

```cpp
#include "mutex/test_mutex.hpp"
#include "semaphore/test_semaphore.hpp"
#include "thread/test_thread.hpp"
#include "timer/test_timer.hpp"

// 由应用任务调用；ManualWorker 须常驻（如静态存储期），且处于空闲状态。
// Call from an application task; ManualWorker must be persistent (e.g. static storage)
// and idle.
void RunSystemTests()
{
  static LibXR::Test::ManualWorker worker;
  constexpr uint32_t tick_ms = 1;  // 根据实际调度 tick 设置 / Set for the actual scheduler tick.
  LibXR::Test::TestThread(worker, tick_ms);
  LibXR::Test::TestSemaphore(worker, tick_ms);
  LibXR::Test::TestMutex(worker);

  // Timer 没有删除已注册任务的接口；对象须保留到复位，只运行一次。
  // Registered Timer tasks cannot be removed; retain this object until reset and run once.
  static LibXR::Test::TimerTest timer_test;
  timer_test.Run();
}
```

`worker` 必须在另一个任务中运行，不能从它自己的回调里调用这些测试。前三项可以重复执行。Timer 测试对象的地址必须保持不变，每个对象的 `Run()` 只能调用一次。

`worker` must run in another task; do not call these tests from its own callback. The first three tests can be repeated. Each Timer test object must stay at a stable address and its `Run()` may be called only once.

## 检查内容 / Checks

| 入口 / Entry | 检查内容 / Checks |
| --- | --- |
| `TestThread` | 连续变化延时并推进同一周期计划 / Vary delays and advance one continuous schedule |
| `TestSemaphore` | 交替成批通知和边收边发，检查丢失与多余通知 / Alternate buffered and overlapping batches; check missing or extra notifications |
| `TestMutex` | 反复争用同一把锁，共享计数每轮增加两次，临界区不能重叠 / Repeated contention, two counter updates per round and no overlapping critical sections |
| `TimerTest::Run` | 连续执行指定次数后停止，检查进度和多余回调 / Run the requested count, check progress, then stop and watch for extra calls |

延时测试的 `tick_ms` 是调度 tick 的毫秒数，向上取整：例如 1000 Hz 填 1，100 Hz 填 10。测试允许 tick 边界和整数毫秒读数造成的误差。默认每次等待的上限为 1000 ms，可用 `timeout_ms` 调整；Thread 和 Semaphore 要求至少留出 16 个 tick。

For delay tests, `tick_ms` is the scheduler tick period rounded up to milliseconds: use 1 for 1000 Hz or 10 for 100 Hz. Checks allow for tick boundaries and integer millisecond sampling. The default deadline for each wait is 1000 ms, adjusted with `timeout_ms`; Thread and Semaphore require room for at least 16 ticks.

## 调整压测轮数 / Set the run length

默认 Thread 运行 100 轮，Semaphore、Mutex 各运行 1000 轮，Timer 运行 1000 次回调。最后一个参数控制次数，填 1 可做快速检查。可以把上面的调用改为：

Thread defaults to 100 rounds; Semaphore and Mutex to 1000 rounds; Timer to 1000 callbacks. The last argument sets the count. Use 1 for a quick check. For longer runs, replace the calls above with:

```cpp
LibXR::Test::TestThread(worker, tick_ms, 1000, 100);
LibXR::Test::TestSemaphore(worker, tick_ms, 1000, 10000);
LibXR::Test::TestMutex(worker, 1000, 10000);
timer_test.Run(10, 1000, 1000);
```

循环复用同一批对象和回调，不在每一轮重新创建资源。增加轮数就能延长测试，无需在外层反复构造并调用测试。

The loops reuse the same objects and callbacks rather than creating resources each round. Increase the count to run longer instead of repeatedly constructing and invoking the tests.

超时限制一次等待或一次进度检查，不限制整段压测的总时长。检查失败仍立即停止，不通过重试掩盖失败。Mutex 的轮数上限为 `UINT32_MAX / 2`，以便核对两方更新的总数。

Timeouts apply to individual waits or progress checks, not the duration of the whole run. A failed check still stops immediately, without retrying away the failure. Mutex rounds are limited to `UINT32_MAX / 2` so the total number of updates can be checked.

Timer 每次回调都会报告进度，达到次数后自行停止，再观察四个周期。它始终只注册一个测试任务，不修改其他定时任务或全局配置。默认周期 10 ms、1000 次回调，运行约需十秒，具体受调度影响。

Timer reports progress on every callback, stops at the requested count and observes four more periods. It registers only one test task and leaves other timers and global settings alone. At the default 10 ms period and 1000 callbacks, expect roughly ten seconds, depending on scheduling.

这些测试增加两个执行上下文之间的重复交接与争用，不代表覆盖所有调度交错。

These tests increase repeated handoffs and contention between two execution contexts; they do not cover every possible interleaving.

所有检查使用共享的 `test/test_assert.hpp`，失败立即打印位置并停止。当前仅检查任务上下文；`PostFromCallback(false)` 不代表真实中断测试。

All checks use the shared `test/test_assert.hpp` and stop immediately with the failure location. This set tests task context only; calls with `in_isr=false` do not establish real-interrupt behavior.
