#pragma once
#include <atomic>

#include "device_composition.hpp"

namespace LibXR::USB::Detail
{
template <bool Enabled>
struct BosFeature
{
};
template <>
struct BosFeature<true>
{
  BosManager* manager = nullptr;
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
template <>
struct HighSpeedFeature<true>
{
  uint8_t qualifier[10]{};
  bool other_speed_descriptor = false;
};

template <bool Enabled>
struct BusTimeFeature
{
};
template <>
struct BusTimeFeature<true>
{
  std::atomic<uint32_t> observed{0xffffffffU};
  std::atomic<uint32_t> observed_ms{0};
  uint32_t previous = 0xffffffffU;
  uint32_t previous_ms = 0;
};
}  // namespace LibXR::USB::Detail
