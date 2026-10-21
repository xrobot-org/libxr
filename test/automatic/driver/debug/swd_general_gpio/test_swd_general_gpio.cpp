/**
 * @file test_swd_general_gpio.cpp
 * @brief SwdGeneralGPIO 主机测试 / Host tests for SwdGeneralGPIO.
 */

#include <vector>

#include "debug/swd_general_gpio.hpp"
#include "gpio.hpp"
#include "test_assert.hpp"

using namespace LibXR;

namespace
{
/// A pin that records every level written to it and every clock edge seen on the other.
class RecordingPin : public GPIO
{
 public:
  bool Read() override { return level; }
  void Write(bool value) override
  {
    level = value;
    writes.push_back(value);
  }
  ErrorCode EnableInterrupt() override { return ErrorCode::OK; }
  ErrorCode DisableInterrupt() override { return ErrorCode::OK; }
  ErrorCode SetConfig(Configuration) override { return ErrorCode::OK; }

  bool level = true;
  std::vector<bool> writes;
};

/// The SWDIO level at every rising edge of SWCLK, as a target would sample it.
struct ClockedPin : public RecordingPin
{
  RecordingPin* data = nullptr;
  std::vector<bool> sampled;
  void Write(bool value) override
  {
    const bool rising = value && !level;
    RecordingPin::Write(value);
    if (rising && data != nullptr) sampled.push_back(data->level);
  }
};

/// Idle cycles keep SWDIO low: a high level would be read by the target as a start bit.
void TestIdleClocksDriveSwdioLow()
{
  ClockedPin swclk;
  RecordingPin swdio;
  swclk.data = &swdio;
  Debug::SwdGeneralGPIO<ClockedPin, RecordingPin> swd(swclk, swdio, 0u,
                                                      Debug::SwdGpioTiming{});
  swclk.sampled.clear();

  swd.IdleClocks(4);
  TEST_ASSERT(swclk.sampled.size() == 4);
  for (const bool level : swclk.sampled)
  {
    TEST_ASSERT(!level);
  }
  TEST_ASSERT(!swdio.level);  // and it stays low afterwards
}

/// The request of a transfer goes out as the ADIv5 packet request, LSB first: a DP read
/// of IDCODE is 0xA5 on the wire.
void TestRequestOnTheWire()
{
  ClockedPin swclk;
  RecordingPin swdio;
  swclk.data = &swdio;
  Debug::SwdGeneralGPIO<ClockedPin, RecordingPin> swd(swclk, swdio, 0u,
                                                      Debug::SwdGpioTiming{});
  swclk.sampled.clear();

  Debug::SwdProtocol::Response resp{};
  (void)swd.Transfer(
      Debug::SwdProtocol::make_dp_read_req(Debug::SwdProtocol::DpReadReg::IDCODE), resp);
  TEST_ASSERT(swclk.sampled.size() >= 8);
  uint8_t request = 0;
  for (unsigned i = 0; i < 8; ++i)
  {
    if (swclk.sampled[i]) request = static_cast<uint8_t>(request | (1u << i));
  }
  TEST_ASSERT(request == 0xA5u);
}

using Gpio = Debug::SwdGeneralGPIO<ClockedPin, RecordingPin>;

/// SetClockHz takes the path without delay (c cycles per clock) only when it does not
/// overshoot (f >= f_cpu / c); otherwise it picks the N >= 1 (a + b·N) closest to the
/// requested period, the slower one on a tie. Synthetic port:
/// 100 MHz CPU, b = 10 cycles, a = 15 cycles, c = 12 cycles.
void TestDelayLoopsPickTheNearestStep()
{
  constexpr uint32_t CPU = 100'000'000u;
  constexpr Debug::SwdGpioTiming T{100u, 150u, 120u};
  TEST_ASSERT(Gpio::DelayLoops(CPU, 3'000'000u, T) == 2u);  // 33.3: 35 beats 25
  TEST_ASSERT(Gpio::DelayLoops(CPU, 4'000'000u, T) == 1u);  // 25: exact
  TEST_ASSERT(Gpio::DelayLoops(CPU, 5'000'000u, T) == 1u);  // 20: 25 beats 12
  // 16.7 cycles: 12 would be nearer than 25 but faster than requested, so N = 1.
  TEST_ASSERT(Gpio::DelayLoops(CPU, 6'000'000u, T) == 1u);
  TEST_ASSERT(Gpio::DelayLoops(CPU, 10'000'000u, T) == 0u);  // 10: no delay
  TEST_ASSERT(Gpio::DelayLoops(CPU, 1'000'000u, T) == 9u);   // 100: 95 / 105 tie -> 105
  TEST_ASSERT(Gpio::DelayLoops(CPU, 2'000'000u, T) == 4u);   // 50: 45 / 55 tie -> 55
  TEST_ASSERT(Gpio::DelayLoops(CPU, 50'000u, T) == 199u);    // 2000: 1995 / 2005 tie
  // 18.5 cycles, halfway between 12 and 25: the path without delay would overshoot.
  TEST_ASSERT(Gpio::DelayLoops(37'000'000u, 2'000'000u, T) == 1u);
  // 12 cycles exactly (f == f_cpu / c): the path without delay does not overshoot.
  TEST_ASSERT(Gpio::DelayLoops(96'000'000u, 8'000'000u, T) == 0u);
  // 12.5 cycles: just above c, so N = 1 although 12 is far nearer than 25.
  TEST_ASSERT(Gpio::DelayLoops(CPU, 8'000'000u, T) == 1u);
  // An unknown CPU clock keeps the path without delay.
  TEST_ASSERT(Gpio::DelayLoops(0u, 1'000'000u, T) == 0u);
  // Defaults: 2 cycles per step, no intercept, 1 cycle without delay.
  TEST_ASSERT(Gpio::DelayLoops(CPU, 1'000'000u, Debug::SwdGpioTiming{}) == 50u);
}

/// Over a sweep: the path without delay is chosen exactly when it does not overshoot;
/// otherwise the chosen N >= 1 is at least as close to the request as its neighbours,
/// the slower on a tie, and within half a step when N >= 2.
void TestDelayLoopsNearestStepSweep()
{
  constexpr uint32_t CPU = 64'000'000u;
  constexpr Debug::SwdGpioTiming T{88u, 160u, 110u};
  const uint64_t TARGET = static_cast<uint64_t>(CPU) * 10u;
  auto cost = [&](uint32_t n) -> uint64_t
  { return T.intercept_cycles_x10 + uint64_t{T.step_cycles_x10} * n; };
  auto err = [&](uint64_t c, uint32_t hz) -> uint64_t
  {
    const uint64_t X = c * hz;
    return X > TARGET ? X - TARGET : TARGET - X;
  };
  for (uint32_t hz = 50'000u; hz <= 20'000'000u; hz += 25'000u)
  {
    const uint32_t N = Gpio::DelayLoops(CPU, hz, T);
    const bool FAST_OK = uint64_t{T.fast_cycles_x10} * hz >= TARGET;
    TEST_ASSERT((N == 0u) == FAST_OK);
    if (N == 0u)
    {
      continue;
    }
    const uint64_t E = err(cost(N), hz);
    for (const uint32_t OTHER : {N - 1u, N + 1u})
    {
      if (OTHER == 0u)
      {
        continue;
      }
      const uint64_t OE = err(cost(OTHER), hz);
      TEST_ASSERT(E <= OE);
      if (E == OE)
      {
        TEST_ASSERT(cost(N) > cost(OTHER));
      }
    }
    if (N >= 2u)
    {
      // |a + b·N - target| <= b / 2, all scaled by hz and by 10.
      TEST_ASSERT(2u * E <= uint64_t{T.step_cycles_x10} * hz);
    }
  }
}
}  // namespace

void test_swd_general_gpio()
{
  TestIdleClocksDriveSwdioLow();
  TestRequestOnTheWire();
  TestDelayLoopsPickTheNearestStep();
  TestDelayLoopsNearestStepSweep();
}
