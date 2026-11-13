#pragma once
#include <atomic>

#include "device_composition.hpp"

namespace LibXR::USB::Detail
{
template <bool Enabled>
struct BosFeature
{
};
/// 启用 BOS 时的特化：持有 BOS 管理器并随当前配置选择 BOS 能力 / Specialization used when
/// BOS is enabled: owns the BOS manager and selects the BOS capabilities of the current
/// configuration
template <>
struct BosFeature<true>
{
  /// BOS 管理器；InitializeBos 之前为 nullptr / BOS manager; nullptr before InitializeBos
  BosManager* manager = nullptr;

  /// 按所有 class 及 WinUSB 能力的总大小创建 BOS 管理器 / Create the BOS manager sized
  /// for all classes' capabilities plus the WinUSB capability
  /// @param composition 提供 class 列表的组合管理器 / Composition providing the class
  /// list
  void InitializeBos(const DeviceComposition& composition)
  {
    size_t capacity = 0, bytes = BOS_HEADER_SIZE + USB2_EXT_CAP_SIZE;
    for (size_t i = 0; i < composition.ClassCount(); ++i)
    {
      auto* item = composition.ClassAt(i);
      capacity += item->GetBosCapabilityCount();
      for (size_t j = 0; j < item->GetBosCapabilityCount(); ++j)
        if (auto* cap = item->GetBosCapability(j))
          bytes += cap->GetCapabilityDescriptor().size_;
    }
    if (auto* cap = composition.WinUsbBosCapability())
    {
      ++capacity;
      bytes += cap->GetCapabilityDescriptor().size_;
    }
    manager = new BosManager(bytes, capacity);
  }
  /// 重建 BOS 能力集合：WinUSB 能力加上当前配置中各 class 的 BOS 能力 / Rebuild the BOS
  /// capability set: the WinUSB capability plus those of the classes in the current
  /// configuration
  /// @param composition 提供当前配置的组合管理器 / Composition providing the current
  /// configuration
  void SelectBos(const DeviceComposition& composition)
  {
    manager->ClearCapabilities();
    if (auto* cap = composition.WinUsbBosCapability()) manager->AddCapability(cap);
    const size_t configuration = composition.GetCurrentConfig();
    const size_t index = configuration == 0U ? 0U : configuration - 1U;
    for (size_t i = 0; i < composition.ConfigItemCount(index); ++i)
    {
      auto* item = composition.ConfigItem(index, i);
      if (!item) continue;
      for (size_t j = 0; j < item->GetBosCapabilityCount(); ++j)
        if (auto* cap = item->GetBosCapability(j)) manager->AddCapability(cap);
    }
  }
};

template <bool Enabled>
struct HighSpeedFeature
{
};
/// 启用高速时的特化：保存高速设备所需的描述符状态 / Specialization used when high speed
/// is enabled: holds the descriptor state a high-speed device needs
template <>
struct HighSpeedFeature<true>
{
  /// 设备限定描述符缓冲区（10 字节）/ Device qualifier descriptor buffer (10 bytes)
  uint8_t qualifier[10]{};
  /// 当前控制请求是否正在应答 OTHER_SPEED_CONFIGURATION；为真时发送时把描述符类型改为 7 /
  /// Whether the current control request answers OTHER_SPEED_CONFIGURATION; when set,
  /// the descriptor type is rewritten to 7 while sending
  bool other_speed_descriptor = false;
};

/// 总线时间功能的状态；功能关闭时为空 / State of the bus time feature; empty when the
/// feature is off
template <bool Enabled>
struct BusTimeFeature
{
};
/// 总线时间功能开启时的状态 / State of the bus time feature when it is on
template <>
struct BusTimeFeature<true>
{
  /// 最近一次 SOF：低 16 位是帧号（0xffff 表示未知），高 16 位是微帧号；全 1 表示还没有
  /// 收到 SOF。由后端在 SOF 时写入。
  /// The latest SOF: the frame number in the low 16 bits (0xffff when unknown) and the
  /// microframe in the high 16 bits; all ones before the first SOF. Written by the
  /// backend at SOF.
  std::atomic<uint32_t> observed{0xffffffffU};
  /// 最近一次 SOF 时的毫秒时间 / Millisecond time of the latest SOF
  std::atomic<uint32_t> observed_ms{0};
  /// 上一次交给类时的 observed，用来计算经过的帧数 / observed at the previous delivery
  /// to the classes, used to compute the elapsed frames
  uint32_t previous = 0xffffffffU;
  /// 上一次交给类时的 observed_ms / observed_ms at the previous delivery to the classes
  uint32_t previous_ms = 0;
};
}  // namespace LibXR::USB::Detail
