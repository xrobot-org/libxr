#pragma once
#include "core.hpp"

namespace LibXR::USB
{
constexpr uint32_t SpeedBit(Speed speed) { return 1U << static_cast<unsigned>(speed); }

struct FullSpeedCapabilities
{
  static constexpr uint32_t SPEEDS = SpeedBit(Speed::FULL);
  static constexpr bool BOS = true;
  static constexpr bool BUS_TIME = true;
};
struct HighSpeedCapabilities : FullSpeedCapabilities
{
  static constexpr uint32_t SPEEDS = SpeedBit(Speed::FULL) | SpeedBit(Speed::HIGH);
};

// Address writes differ between controllers; the protocol commit is always the
// completed status stage, while these phases preserve hardware programming needs.
enum class ControlContext : uint8_t
{
  UNKNOWN,
  SETUP_BEFORE_STATUS,
  STATUS_IN_ARMED,
  DATA_OUT,
  STATUS_OUT,
  DATA_IN,
  STATUS_IN_COMPLETE,
  ZLP
};
}  // namespace LibXR::USB
