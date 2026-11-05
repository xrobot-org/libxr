/**
 * @file test_swd_general_gpio.cpp
 * @brief SwdGeneralGPIO 主机测试 / Host tests for SwdGeneralGPIO.
 */

#include <cstdio>
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
  Debug::SwdGeneralGPIO<ClockedPin, RecordingPin> swd(swclk, swdio, 0u);
  swclk.sampled.clear();

  swd.IdleClocks(4);
  TEST_ASSERT(swclk.sampled.size() == 4);
  for (const bool level : swclk.sampled)
  {
    TEST_ASSERT(!level);
  }
  TEST_ASSERT(!swdio.level);  // and it stays low afterwards
}
}  // namespace

int main()
{
  TestIdleClocksDriveSwdioLow();
  std::puts("SwdGeneralGPIO: idle clocks drive SWDIO low");
}
