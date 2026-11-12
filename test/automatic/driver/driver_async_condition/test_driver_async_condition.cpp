/**
 * @file test_driver_async_condition.cpp
 * @brief DriverAsyncCondition 等待测试 / DriverAsyncCondition wait tests.
 *
 * 由测试直接调用 OnTick()，检查等待下限、检查次数、超时、链式等待、粗节拍的上下界，
 * 以及 Start() 与节拍并发。
 * Call OnTick() from the test and check the lower bound, the check count, timeouts,
 * chained waits, the bounds with coarse ticks, and Start() racing the tick.
 */

#include <atomic>
#include <thread>
#include <vector>

#include "driver_async_condition.hpp"
#include "test_assert.hpp"

namespace
{
using LibXR::DriverAsyncCondition;
using LibXR::ErrorCode;

struct Probe
{
  bool ready = false;              ///< 条件是否成立 / Whether the condition holds.
  int checks = 0;                  ///< 条件函数被调用的次数 / Calls of the condition.
  std::vector<ErrorCode> results;  ///< 收到的结果 / Results received.
  /// 非空时由结果函数对它调用 Start() / Started from the result function when set.
  DriverAsyncCondition* next = nullptr;
};

bool Check(void* arg)
{
  auto* probe = static_cast<Probe*>(arg);
  ++probe->checks;
  return probe->ready;
}

void Done(void* arg, ErrorCode result)
{
  auto* probe = static_cast<Probe*>(arg);
  probe->results.push_back(result);
  if (probe->next != nullptr)
  {
    TEST_ASSERT(probe->next->Start(2, 2) == ErrorCode::OK);
  }
}

void Ticks(int count, uint32_t elapsed_ms = 1)
{
  for (int i = 0; i < count; ++i)
  {
    DriverAsyncCondition::OnTick(elapsed_ms);
  }
}

void TestHoldingCheckLastsAtLeastMin()
{
  Probe probe;
  probe.ready = true;
  DriverAsyncCondition delay(Check, Done, &probe);
  TEST_ASSERT(delay.Start(25, 25) == ErrorCode::OK);
  TEST_ASSERT(delay.Start(25, 25) == ErrorCode::BUSY);
  // 第一个节拍可能紧跟在 Start() 之后，所以要再过 25 个节拍才开始检查。
  // The first tick may come right after Start(), so checking needs 25 more ticks.
  Ticks(25);
  TEST_ASSERT(probe.results.empty() && probe.checks == 0);
  Ticks(1);
  TEST_ASSERT(probe.results == std::vector<ErrorCode>{ErrorCode::OK});
  Ticks(10);
  TEST_ASSERT(probe.results.size() == 1 && probe.checks == 1);
}

void TestCheckStartsAfterMinAndTimesOutAtMax()
{
  Probe probe;
  DriverAsyncCondition condition(Check, Done, &probe);
  TEST_ASSERT(condition.Start(3, 6) == ErrorCode::OK);
  Ticks(3);
  TEST_ASSERT(probe.checks == 0);
  Ticks(1);
  TEST_ASSERT(probe.checks == 1 && probe.results.empty());
  Ticks(2);
  TEST_ASSERT(probe.checks == 3 && probe.results.empty());
  Ticks(1);
  TEST_ASSERT(probe.results == std::vector<ErrorCode>{ErrorCode::TIMEOUT});
  // 条件已成立时，第一次检查就结束。
  // A condition that holds finishes on the first check.
  probe = Probe{};
  TEST_ASSERT(condition.Start(0, 100) == ErrorCode::OK);
  probe.ready = true;
  Ticks(1);
  TEST_ASSERT(probe.results == std::vector<ErrorCode>{ErrorCode::OK} &&
              probe.checks == 1);
}

void TestBusyAndArguments()
{
  Probe probe;
  DriverAsyncCondition condition(Check, Done, &probe);
  TEST_ASSERT(condition.Start(5, 4) == ErrorCode::ARG_ERR);
  TEST_ASSERT(condition.Start(0, DriverAsyncCondition::MAX_WAIT_MS + 1U) ==
              ErrorCode::ARG_ERR);
  // 被拒绝的参数不占用对象。
  // Rejected arguments leave the object free.
  TEST_ASSERT(condition.Start(1, DriverAsyncCondition::MAX_WAIT_MS) == ErrorCode::OK);
  TEST_ASSERT(condition.Start(1, 5) == ErrorCode::BUSY);
}

void TestDoneChainsTheNextCondition()
{
  // 两段等待，后一段由前一段的结果函数启动。
  // Two stages; the result function of the first starts the second.
  Probe second_probe;
  second_probe.ready = true;
  DriverAsyncCondition second(Check, Done, &second_probe);
  Probe first_probe;
  first_probe.ready = true;
  first_probe.next = &second;
  DriverAsyncCondition first(Check, Done, &first_probe);
  TEST_ASSERT(first.Start(1, 1) == ErrorCode::OK);
  Ticks(2);
  TEST_ASSERT(first_probe.results.size() == 1 && second.Start(0, 0) == ErrorCode::BUSY);
  // 第二段从第一段结束起至少再等自己的 2 ms：再过两个节拍；如果链表在每一轮里先访问
  // 第二段，则是三个。
  // The second stage waits at least its own 2 ms after the first finished: two more
  // ticks, or three when the list visits it before the first in each pass.
  Ticks(1);
  TEST_ASSERT(second_probe.results.empty());
  int extra = 1;
  while (second_probe.results.empty() && extra < 3)
  {
    Ticks(1);
    ++extra;
  }
  TEST_ASSERT(extra >= 2 && second_probe.results.size() == 1);
  Ticks(3);
  TEST_ASSERT(second_probe.results.size() == 1);

  // 同一个对象也可以在自己的结果函数里再次开始等待。
  // The same object may also start again from its own result function.
  Probe self_probe;
  self_probe.ready = true;
  DriverAsyncCondition self(Check, Done, &self_probe);
  self_probe.next = &self;
  TEST_ASSERT(self.Start(0, 0) == ErrorCode::OK);
  Ticks(1);
  TEST_ASSERT(self_probe.results.size() == 1 && self.Start(0, 0) == ErrorCode::BUSY);
  self_probe.next = nullptr;
  Ticks(3);
  TEST_ASSERT(self_probe.results.size() == 2);
  Ticks(3);
  TEST_ASSERT(self_probe.results.size() == 2);
}

void TestCoarseTickBounds()
{
  // 节拍为 10 ms 时，N ms 后到期的检查或超时，落在计数达到“N 向上取整到 10 再加 10”
  // 的那个节拍上：不早于 N ms，也不晚于这个上界。
  // With 10 ms ticks, a check or timeout due after N ms comes on the tick that counts N
  // rounded up to 10, plus 10: never before N ms have passed, never later than that.
  Probe probe;
  probe.ready = true;
  DriverAsyncCondition condition(Check, Done, &probe);
  TEST_ASSERT(condition.Start(25, 25) == ErrorCode::OK);
  Ticks(3, 10);
  TEST_ASSERT(probe.checks == 0 && probe.results.empty());
  Ticks(1, 10);
  TEST_ASSERT(probe.results == std::vector<ErrorCode>{ErrorCode::OK});

  probe = Probe{};
  TEST_ASSERT(condition.Start(25, 25) == ErrorCode::OK);
  Ticks(3, 10);
  TEST_ASSERT(probe.checks == 0 && probe.results.empty());
  Ticks(1, 10);
  TEST_ASSERT(probe.results == std::vector<ErrorCode>{ErrorCode::TIMEOUT});

  probe = Probe{};
  TEST_ASSERT(condition.Start(20, 40) == ErrorCode::OK);
  Ticks(2, 10);
  TEST_ASSERT(probe.checks == 0);
  Ticks(1, 10);
  TEST_ASSERT(probe.checks == 1 && probe.results.empty());
  Ticks(1, 10);
  TEST_ASSERT(probe.checks == 2 && probe.results.empty());
  Ticks(1, 10);
  TEST_ASSERT(probe.results == std::vector<ErrorCode>{ErrorCode::TIMEOUT});
}

struct RaceProbe
{
  std::atomic<uint32_t> finished{0};  ///< 已结束的等待数 / Waits that ended.
  std::atomic<uint32_t> timeouts{0};  ///< 以 TIMEOUT 结束的等待数 / Ended with TIMEOUT.
};

bool NeverHolds(void*) { return false; }

void CountDone(void* arg, ErrorCode result)
{
  auto* probe = static_cast<RaceProbe*>(arg);
  if (result == ErrorCode::TIMEOUT)
  {
    probe->timeouts.fetch_add(1, std::memory_order_relaxed);
  }
  probe->finished.fetch_add(1, std::memory_order_release);
}

void TestStartRacesTheTick()
{
  // 一个线程不停调用 OnTick()，测试线程不停调用 Start()：每次被接受的等待都恰好以
  // TIMEOUT 结束一次。在 ThreadSanitizer 下，这也检查上下限是由状态字发布的。
  // One thread keeps calling OnTick() while the test thread keeps calling Start():
  // every accepted wait ends exactly once, with TIMEOUT. Under ThreadSanitizer this
  // also checks that the state word publishes the limits.
  constexpr uint32_t STARTS = 20000U;
  RaceProbe probe;
  DriverAsyncCondition condition(NeverHolds, CountDone, &probe);
  std::atomic<bool> stop{false};
  std::thread ticker(
      [&stop]()
      {
        while (!stop.load(std::memory_order_relaxed))
        {
          DriverAsyncCondition::OnTick(1);
        }
      });
  uint32_t busy = 0;
  for (uint32_t i = 0; i < STARTS; ++i)
  {
    const uint32_t limit = i % 4U;
    while (condition.Start(limit, limit) == ErrorCode::BUSY)
    {
      ++busy;
    }
  }
  while (probe.finished.load(std::memory_order_acquire) != STARTS)
  {
    std::this_thread::yield();
  }
  stop.store(true, std::memory_order_relaxed);
  ticker.join();
  TEST_ASSERT(probe.timeouts.load() == STARTS);
  TEST_ASSERT(busy > 0U);
}
}  // namespace

void test_driver_async_condition()
{
  TestHoldingCheckLastsAtLeastMin();
  TestCheckStartsAfterMinAndTimesOutAtMax();
  TestBusyAndArguments();
  TestDoneChainsTheNextCondition();
  TestCoarseTickBounds();
  TestStartRacesTheTick();
}
