/**
 * @file test_driver_async_condition.cpp
 * @brief DriverAsyncCondition：等待下限、检查、超时与链式等待 /
 * DriverAsyncCondition: the lower bound, checks, timeouts and chained waits.
 */

#include <cstdio>
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
  DriverAsyncCondition* next = nullptr;  ///< Waited for from done when set.
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
    TEST_ASSERT(probe->next->Wait(2, 2) == ErrorCode::OK);
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
  TEST_ASSERT(delay.Wait(25, 25) == ErrorCode::OK);
  TEST_ASSERT(delay.Wait(25, 25) == ErrorCode::BUSY);
  // The first tick may come right after Wait(): 25 more ticks are needed.
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
  TEST_ASSERT(condition.Wait(3, 6) == ErrorCode::OK);
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
  TEST_ASSERT(condition.Wait(0, 100) == ErrorCode::OK);
  probe.ready = true;
  Ticks(1);
  TEST_ASSERT(probe.results == std::vector<ErrorCode>{ErrorCode::OK} &&
              probe.checks == 1);
}

void TestBusyAndArguments()
{
  Probe probe;
  DriverAsyncCondition condition(Check, Done, &probe);
  TEST_ASSERT(condition.Wait(5, 4) == ErrorCode::ARG_ERR);
  TEST_ASSERT(condition.Wait(1, 5) == ErrorCode::OK);
  TEST_ASSERT(condition.Wait(1, 5) == ErrorCode::BUSY);
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
  TEST_ASSERT(first.Wait(1, 1) == ErrorCode::OK);
  Ticks(2);
  TEST_ASSERT(first_probe.results.size() == 1 && second.Wait(0, 0) == ErrorCode::BUSY);
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
  TEST_ASSERT(self.Wait(0, 0) == ErrorCode::OK);
  Ticks(1);
  TEST_ASSERT(self_probe.results.size() == 1 && self.Wait(0, 0) == ErrorCode::BUSY);
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
  TEST_ASSERT(delay.Wait(25, 25) == ErrorCode::OK);
  Ticks(3, 10);
  TEST_ASSERT(probe.results.empty());
  Ticks(1, 10);
  TEST_ASSERT(probe.results.size() == 1);
}
}  // namespace

int main()
{
  TestHoldingCheckLastsAtLeastMin();
  TestCheckStartsAfterMinAndTimesOutAtMax();
  TestBusyAndArguments();
  TestDoneChainsTheNextCondition();
  TestCoarseTick();
  std::puts("DriverAsyncCondition lower bound, checks, timeout and chains passed");
}
