/**
 * @file test_driver_async_condition.cpp
 * @brief DriverAsyncCondition：等待下限、检查、超时、链式等待、粗节拍与并发 Start /
 * DriverAsyncCondition: the lower bound, checks, timeouts, chained waits, coarse ticks
 * and concurrent Start.
 */

#include <atomic>
#include <cstdio>
#include <thread>
#include <vector>

#include "driver_async_condition.hpp"
#include "test_assert.hpp"

using LibXR::DriverAsyncCondition;
using LibXR::ErrorCode;

namespace
{
struct Probe
{
  bool ready = false;
  int checks = 0;
  std::vector<ErrorCode> results;
  DriverAsyncCondition* next = nullptr;  ///< Started from done when set.
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
  // The first tick may come right after Start(): 25 more ticks are needed.
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
  TEST_ASSERT(condition.Start(1, 5) == ErrorCode::OK);
  TEST_ASSERT(condition.Start(1, 5) == ErrorCode::BUSY);
}

void TestDoneChainsTheNextCondition()
{
  // Two stages, each started from the done of the one before; the same object may also
  // wait again from its own done.
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

void TestCoarseTick()
{
  // A 10 ms tick (HAL_TICK_FREQ_10HZ... 100HZ): still at least min_ms.
  Probe probe;
  probe.ready = true;
  DriverAsyncCondition delay(Check, Done, &probe);
  TEST_ASSERT(delay.Start(25, 25) == ErrorCode::OK);
  Ticks(3, 10);
  TEST_ASSERT(probe.results.empty());
  Ticks(1, 10);
  TEST_ASSERT(probe.results.size() == 1);
}

void TestCoarseTickBounds()
{
  // With 10 ms ticks, a check or timeout due after N ms comes on the tick that counts N
  // rounded up to 10, plus 10: never before N ms have passed, never later than that.
  Probe probe;
  DriverAsyncCondition condition(Check, Done, &probe);
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
  std::atomic<uint32_t> finished{0};
  std::atomic<uint32_t> timeouts{0};
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
  // Start() on one thread while OnTick() runs on another: every accepted wait ends
  // exactly once, with TIMEOUT. Run under ThreadSanitizer, this also checks that the
  // limits are published by the state word.
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

int main()
{
  TestHoldingCheckLastsAtLeastMin();
  TestCheckStartsAfterMinAndTimesOutAtMax();
  TestBusyAndArguments();
  TestDoneChainsTheNextCondition();
  TestCoarseTick();
  TestCoarseTickBounds();
  TestStartRacesTheTick();
  std::puts(
      "DriverAsyncCondition lower bound, checks, timeout, chains, coarse ticks and "
      "concurrent Start passed");
}
