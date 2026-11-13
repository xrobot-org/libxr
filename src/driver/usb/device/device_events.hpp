#pragma once
#include <cstdint>

namespace LibXR::USB
{
enum class DeviceEvent : uint8_t
{
  RESET,
  ENDPOINT_RESUMED
};

/// 交给类的总线时间信息 / Bus time information handed to the classes
struct BusTime
{
  /// 控制器只报告完整帧时为 0xff / 0xff when the controller only reports full frames.
  uint8_t microframe = 0xff;
  /// 自上一次节拍以来经过的帧数（控制器报告微帧时为微帧数）；无法确定时为 0 / Frames
  /// elapsed since the previous tick (microframes when the controller reports them); 0
  /// when it cannot be determined
  uint32_t elapsed_intervals = 0;
};
}  // namespace LibXR::USB
