#pragma once
#include "core.hpp"

namespace LibXR::USB
{
constexpr uint32_t SpeedBit(Speed speed) { return 1U << static_cast<unsigned>(speed); }

/// 仅支持全速的设备能力策略（含 BOS）/ Capability policy of a full-speed-only device
/// (with BOS)
struct FullSpeedCapabilities
{
  /// 支持的总线速度位图，按 SpeedBit 编码 / Bitmap of supported bus speeds, encoded with
  /// SpeedBit
  static constexpr uint32_t SPEEDS = SpeedBit(Speed::FULL);
  /// 是否编译 BOS 描述符支持 / Whether BOS descriptor support is compiled in
  static constexpr bool BOS = true;
  /// 是否编译总线时间（按 SOF 向类报告经过的帧数）的支持 / Whether support for the bus
  /// time (elapsed frames reported to the classes at SOF) is compiled in
  static constexpr bool BUS_TIME = true;
};

/// 支持全速与高速的设备能力策略，其余继承全速策略 / Capability policy of a device that
/// supports full and high speed; the rest is inherited from the full-speed policy
struct HighSpeedCapabilities : FullSpeedCapabilities
{
  /// 支持的总线速度位图：全速和高速 / Supported bus speeds: full and high
  static constexpr uint32_t SPEEDS = SpeedBit(Speed::FULL) | SpeedBit(Speed::HIGH);
};

// 不同控制器的地址写入方式不同；协议上的提交始终是完成的状态阶段，
// 而这些阶段保留了硬件编程的需要。
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
