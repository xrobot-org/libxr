#pragma once
#include <cstdint>

namespace LibXR::USB
{
enum class DeviceEvent : uint8_t
{
  RESET,
  ENDPOINT_RESUMED
};

struct BusTime
{
  uint8_t microframe = 0xff;  ///< 0xff when the controller only reports full frames.
  uint32_t elapsed_intervals = 0;
};
}  // namespace LibXR::USB
