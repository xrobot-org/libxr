/**
 * @file test_usb_detach_timer.cpp
 * @brief DetachTimer 与节拍源登记的测试 / Tests of DetachTimer and the tick source
 *        registration
 *
 * 测试直接调用 OnTick()，扮演节拍。覆盖：状态和转换、断开期间的 Start / Stop / 复位、
 * 计时不重新开始、Start() 的 BUSY 不出现、没有 pend 时经 defer 交给线程、第二次登记和
 * 没有节拍的检测。
 * The test calls OnTick() itself and plays the tick. Covered: the states and
 * transitions, Start / Stop / reset during a detach, the count not restarting, no BUSY
 * from Start(), handing over to a thread through defer without pend, a second
 * registration and the missing-tick check.
 */

#include <sys/wait.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <thread>

#include "driver_async_condition.hpp"
#include "libxr_assert.hpp"
#include "test_assert.hpp"
#include "usb/core/usb_detach_timer.hpp"
#include "usb/core/usb_service.hpp"

namespace
{
using LibXR::DriverAsyncCondition;
using LibXR::USB::DetachTimer;
using LibXR::USB::DeviceService;
using HaltResult = DetachTimer::HaltResult;

constexpr uint32_t DETACH_MS = 5U;
constexpr int FATAL_EXIT = 77;

/// 设备服务的替身：记下 pend 和 ServiceHardware / Stand-in for the device service: it
/// records pends and ServiceHardware
class FakeService final : public DeviceService
{
 public:
  explicit FakeService(bool can_pend) : can_pend_(can_pend) {}

  /// 挂起之后由扮演中断的测试运行 / After a pend the test playing the interrupt runs it
  void Irq()
  {
    while (pended)
    {
      pended = false;
      OnControllerInterrupt();
    }
  }

  void Poll(bool in_isr) { RequestPoll(in_isr); }

  unsigned pends = 0;
  unsigned passes = 0;  ///< ServiceHardware 的次数 / ServiceHardware calls
  bool pended = false;
  bool last_in_isr = true;

 protected:
  void ServiceHardware(bool in_isr) override
  {
    ++passes;
    last_in_isr = in_isr;
  }
  void MaskInterrupt() override {}
  void UnmaskInterrupt() override {}
  bool PendInterrupt() override
  {
    if (!can_pend_)
    {
      return false;
    }
    ++pends;
    pended = true;
    return true;
  }

 private:
  bool can_pend_;
};

void Ticks(int count)
{
  for (int i = 0; i < count; ++i)
  {
    DriverAsyncCondition::OnTick(1U);
  }
}

/// 在子进程里运行 func，返回它是否经致命错误结束 / Run func in a child process and
/// return whether it ended through the fatal callback
template <typename Func>
bool EndsFatally(Func&& func)
{
  std::fflush(nullptr);
  const pid_t child = fork();
  TEST_ASSERT(child >= 0);
  if (child == 0)
  {
    auto cb = LibXR::Assert::FatalCallback::Create(
        [](bool, int code, const char*, uint32_t) { _exit(code); }, FATAL_EXIT);
    LibXR::Assert::RegisterFatalErrorCallback(cb);
    func();
    _exit(0);
  }
  int status = 0;
  TEST_ASSERT(waitpid(child, &status, 0) == child);
  TEST_ASSERT(WIFEXITED(status));
  TEST_ASSERT(WEXITSTATUS(status) == 0 || WEXITSTATUS(status) == FATAL_EXIT);
  return WEXITSTATUS(status) == FATAL_EXIT;
}

/// 计时从断开开始，到点之前一直断开，不要接上时到点后停在 DETACHED、不叫醒 owner；
/// 之后的 Start 就地接上
/// The count starts at the detach and the device stays detached until it ends; with no
/// attach wanted it stops in DETACHED without waking the owner; a later Start attaches
/// in place
void TestDetachWithoutAttach()
{
  FakeService service(true);
  DetachTimer timer(service, DETACH_MS);
  TEST_ASSERT(!timer.Detached());
  TEST_ASSERT(timer.WantAttach());  // 从未断开 / never detached
  TEST_ASSERT(timer.OnHalted(false) == HaltResult::COUNTING);
  TEST_ASSERT(timer.Detached());
  Ticks(static_cast<int>(DETACH_MS) + 1);
  TEST_ASSERT(service.pends == 0U && timer.Detached());
  TEST_ASSERT(!timer.TakeAttach());
  TEST_ASSERT(timer.WantAttach());
  TEST_ASSERT(!timer.Detached());
}

/// 复位的断开：到点之前不叫醒，到点时只 pend 一次，owner 取走一次
/// A reset's detach: nothing wakes before the time, the tick pends once when it has
/// passed and the owner takes it once
void TestResetAttachesFromTick()
{
  FakeService service(true);
  DetachTimer timer(service, DETACH_MS);
  TEST_ASSERT(timer.OnHalted(true) == HaltResult::COUNTING);
  // 第一个节拍可能紧跟在断开之后：DETACH_MS 个节拍还不够 / The first tick may come right
  // after the detach: DETACH_MS ticks are not enough yet
  Ticks(static_cast<int>(DETACH_MS));
  TEST_ASSERT(service.pends == 0U && !timer.TakeAttach());
  Ticks(1);
  TEST_ASSERT(service.pends == 1U);
  TEST_ASSERT(!timer.WantAttach());  // 交给 owner 了 / handed to the owner
  TEST_ASSERT(timer.TakeAttach());
  TEST_ASSERT(!timer.TakeAttach() && !timer.Detached());
  Ticks(10);
  TEST_ASSERT(service.pends == 1U);
}

/// 断开期间 Start：只记下要接上，到点后由 owner 接上
/// Start during the detach: only records the attach, and the owner attaches when the
/// time has passed
void TestStartDuringDetach()
{
  FakeService service(true);
  DetachTimer timer(service, DETACH_MS);
  TEST_ASSERT(timer.OnHalted(false) == HaltResult::COUNTING);
  Ticks(2);
  TEST_ASSERT(!timer.WantAttach());
  TEST_ASSERT(!timer.WantAttach());  // 再来一次也一样 / again, the same
  Ticks(static_cast<int>(DETACH_MS) - 2);
  TEST_ASSERT(service.pends == 0U);
  Ticks(1);
  TEST_ASSERT(service.pends == 1U && timer.TakeAttach());
}

/// 断开期间 Stop：撤销"要接上"，到点后不接上
/// Stop during the detach: "attach wanted" is withdrawn and nothing attaches
void TestStopCancelsAttach()
{
  FakeService service(true);
  DetachTimer timer(service, DETACH_MS);
  TEST_ASSERT(timer.OnHalted(true) == HaltResult::COUNTING);
  Ticks(2);
  TEST_ASSERT(timer.OnHalted(false) == HaltResult::COUNTING);
  Ticks(static_cast<int>(DETACH_MS) + 2);
  TEST_ASSERT(service.pends == 0U && timer.Detached() && !timer.TakeAttach());
  // 同样适用于 Start 记下的接上 / The same holds for an attach recorded by Start
  TEST_ASSERT(timer.WantAttach());
  TEST_ASSERT(timer.OnHalted(false) == HaltResult::COUNTING);
  TEST_ASSERT(!timer.WantAttach());
  TEST_ASSERT(timer.OnHalted(false) == HaltResult::COUNTING);
  Ticks(static_cast<int>(DETACH_MS) + 2);
  TEST_ASSERT(service.pends == 0U && !timer.TakeAttach());
}

/// 断开期间再复位：沿用原来的计时，不从头算
/// Another reset during the detach keeps the running count and does not start over
void TestResetDuringDetachKeepsCount()
{
  FakeService service(true);
  DetachTimer timer(service, DETACH_MS);
  TEST_ASSERT(timer.OnHalted(true) == HaltResult::COUNTING);
  Ticks(4);
  TEST_ASSERT(timer.OnHalted(true) == HaltResult::COUNTING);
  Ticks(2);
  TEST_ASSERT(service.pends == 1U && timer.TakeAttach());
}

/// 时间已够、还没接上时再断开：不再等；复位就地接上，停止保持断开
/// A detach after the time has passed but before an attach: no more wait; a reset
/// attaches in place, a stop stays detached
void TestHaltAfterTimePassed()
{
  FakeService service(true);
  DetachTimer timer(service, DETACH_MS);
  TEST_ASSERT(timer.OnHalted(false) == HaltResult::COUNTING);
  Ticks(static_cast<int>(DETACH_MS) + 1);
  TEST_ASSERT(timer.OnHalted(false) == HaltResult::DETACHED);
  TEST_ASSERT(timer.OnHalted(true) == HaltResult::ATTACH_NOW);
  TEST_ASSERT(!timer.Detached());
  // 已 pend、owner 还没取走时又复位：就地接上，那一轮什么也取不到
  // A reset after the pend and before the owner took it: attach in place, the pass
  // takes nothing
  TEST_ASSERT(timer.OnHalted(true) == HaltResult::COUNTING);
  Ticks(static_cast<int>(DETACH_MS) + 1);
  TEST_ASSERT(service.pends == 1U);
  TEST_ASSERT(timer.OnHalted(true) == HaltResult::ATTACH_NOW);
  TEST_ASSERT(!timer.TakeAttach());
}

/// 任意交错的断开、Start、节拍都不让计时对象的 Start() 返回 BUSY（那会断言），也不丢
/// 接上
/// No interleaving of detaches, Starts and ticks makes the condition's Start() return
/// BUSY (which would assert), and no attach is lost
void TestNoBusy()
{
  FakeService service(true);
  DetachTimer timer(service, DETACH_MS);
  for (int round = 0; round < 50; ++round)
  {
    (void)timer.OnHalted((round % 3) == 0);
    Ticks(round % 4);
    (void)timer.OnHalted((round % 2) == 0);
    if ((round % 5) == 0)
    {
      (void)timer.WantAttach();
    }
    Ticks(round % 7);
    if (timer.TakeAttach())
    {
      continue;
    }
    // 要接上的最终都会接上 / A wanted attach always comes in the end
    if (!timer.WantAttach())
    {
      Ticks(static_cast<int>(DETACH_MS) + 1);
      TEST_ASSERT(timer.TakeAttach());
    }
    TEST_ASSERT(!timer.Detached());
  }
}

/// 没有 pend 的后端：结果函数调用 defer；defer 失败 1 ms 后再试；任务里的 RequestPoll
/// 就地成为 owner
/// A backend without pend: the result function calls defer; a failed defer is tried
/// again 1 ms later; RequestPoll from a task owns in place
struct Deferred
{
  unsigned calls = 0;
  unsigned failures = 0;  ///< 先失败这么多次 / Fail this many times first
  FakeService* service = nullptr;
};

bool Defer(void* arg)
{
  auto* d = static_cast<Deferred*>(arg);
  ++d->calls;
  if (d->failures > 0U)
  {
    --d->failures;
    return false;
  }
  return true;
}

void TestDeferWithoutPend()
{
  FakeService service(false);
  Deferred deferred;
  deferred.failures = 1U;
  DetachTimer timer(service, DETACH_MS, Defer, &deferred);
  TEST_ASSERT(timer.OnHalted(true) == HaltResult::COUNTING);
  Ticks(static_cast<int>(DETACH_MS) + 1);
  TEST_ASSERT(deferred.calls == 1U && service.pends == 0U);
  Ticks(1);
  TEST_ASSERT(deferred.calls == 1U);
  Ticks(1);
  TEST_ASSERT(deferred.calls == 2U);
  Ticks(5);
  TEST_ASSERT(deferred.calls == 2U);
  // 交给的任务：就地成为 owner，ServiceHardware 在线程上下文 / The task handed over to:
  // it owns in place, ServiceHardware in thread context
  service.Poll(false);
  TEST_ASSERT(service.passes == 1U && !service.last_in_isr);
  TEST_ASSERT(timer.TakeAttach());
}

/// 没有 pend 的后端在中断里调用 RequestPoll 仍然断言 / A backend without pend calling
/// RequestPoll in an interrupt still asserts
void TestPollInIsrWithoutPendFatal()
{
#ifdef LIBXR_DEBUG_BUILD
  // 这项检查是 ASSERT，只在 LIBXR_DEBUG_BUILD 下生效 / The check is an ASSERT, active in
  // LIBXR_DEBUG_BUILD only
  TEST_ASSERT(EndsFatally(
      []
      {
        FakeService no_pend(false);
        no_pend.Poll(true);
      }));
#endif
}

/// 重试还在等时开始新的断开：为它重新计时，不提前接上
/// A new detach while a retry is pending: counted again for it, no early attach
void TestDetachDuringRetry()
{
  FakeService service(false);
  Deferred deferred;
  deferred.failures = 1U;
  DetachTimer timer(service, DETACH_MS, Defer, &deferred);
  TEST_ASSERT(timer.OnHalted(true) == HaltResult::COUNTING);
  Ticks(static_cast<int>(DETACH_MS) + 1);
  TEST_ASSERT(deferred.calls == 1U);
  // owner 在重试之前就地接上（复位时间已过），随后又停止 / The owner attaches in place
  // before the retry (a reset after the time), then stops
  TEST_ASSERT(timer.OnHalted(true) == HaltResult::ATTACH_NOW);
  TEST_ASSERT(timer.OnHalted(true) == HaltResult::COUNTING);
  Ticks(2);  // 重试的节拍 / the retry's tick
  TEST_ASSERT(deferred.calls == 1U && timer.Detached());
  Ticks(static_cast<int>(DETACH_MS));
  TEST_ASSERT(deferred.calls == 1U);
  Ticks(1);
  TEST_ASSERT(deferred.calls == 2U && timer.TakeAttach());
}

/// 第二次登记节拍源是致命错误 / A second tick source registration is fatal
void TestSecondRegistrationFatal()
{
  TEST_ASSERT(DriverAsyncCondition::HasTickSource());
  TEST_ASSERT(EndsFatally([] { DriverAsyncCondition::RegisterTickSource(); }));
}

/// 计时中 Start 时 Timebase 已过 3 倍断开时间而节拍没动：断言；节拍动过就不断言
/// Start while counting, Timebase past 3 times the detach time and the tick not moved:
/// asserts; once the tick moved it does not
void TestMissingTickCheck()
{
#ifdef LIBXR_DEBUG_BUILD
  // 这项检查是 ASSERT，只在 LIBXR_DEBUG_BUILD 下生效 / The check is an ASSERT, active in
  // LIBXR_DEBUG_BUILD only
  TEST_ASSERT(EndsFatally(
      []
      {
        FakeService service(true);
        DetachTimer timer(service, DETACH_MS);
        (void)timer.OnHalted(false);
        std::this_thread::sleep_for(std::chrono::milliseconds(3U * DETACH_MS + 5U));
        (void)timer.WantAttach();
      }));
#endif
  TEST_ASSERT(!EndsFatally(
      []
      {
        FakeService service(true);
        DetachTimer timer(service, DETACH_MS);
        (void)timer.OnHalted(false);
        Ticks(1);
        std::this_thread::sleep_for(std::chrono::milliseconds(3U * DETACH_MS + 5U));
        TEST_ASSERT(!timer.WantAttach());
      }));
  // 不到 3 倍时不断言 / Below 3 times it does not assert
  TEST_ASSERT(!EndsFatally(
      []
      {
        FakeService service(true);
        DetachTimer timer(service, DETACH_MS);
        (void)timer.OnHalted(false);
        TEST_ASSERT(!timer.WantAttach());
      }));
}

/// OnTick 计数 / OnTick counts
void TestTickCount()
{
  const uint32_t before = DriverAsyncCondition::TickCount();
  Ticks(3);
  TEST_ASSERT(DriverAsyncCondition::TickCount() - before == 3U);
}

}  // namespace

void test_usb_detach_timer_states()
{
  TestDetachWithoutAttach();
  TestHaltAfterTimePassed();
  TestTickCount();
}
void test_usb_detach_timer_reset() { TestResetAttachesFromTick(); }
void test_usb_detach_timer_start() { TestStartDuringDetach(); }
void test_usb_detach_timer_stop() { TestStopCancelsAttach(); }
void test_usb_detach_timer_keep_count() { TestResetDuringDetachKeepsCount(); }
void test_usb_detach_timer_no_busy() { TestNoBusy(); }
void test_usb_detach_timer_defer()
{
  TestDeferWithoutPend();
  TestDetachDuringRetry();
}
void test_usb_detach_timer_second_source() { TestSecondRegistrationFatal(); }
void test_usb_detach_timer_no_tick() { TestMissingTickCheck(); }
void test_usb_detach_timer_poll_isr() { TestPollInIsrWithoutPendFatal(); }
