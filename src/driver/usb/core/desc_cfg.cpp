#include "desc_cfg.hpp"

#include <cstddef>
#include <cstdint>

#include "libxr_mem.hpp"

using namespace LibXR::USB;

namespace
{
using Header = ConfigDescriptorItem::Header;

static size_t calc_max_config_size(
    const std::initializer_list<const std::initializer_list<ConfigDescriptorItem*>>&
        configs)
{
  // 初始化时一次性为最大的 configuration 预留缓冲区。
  // Reserve enough space for the largest configuration during initialization.
  size_t max_config_size = 0;

  for (const auto& cfg_group : configs)
  {
    size_t config_size = sizeof(Header);
    for (auto* item : cfg_group)
    {
      if (item != nullptr)
      {
        config_size += item->GetMaxConfigSize();
      }
    }

    if (config_size > max_config_size)
    {
      max_config_size = config_size;
    }
  }

  return max_config_size;
}

}  // namespace

LibXR::RawData ConfigDescriptorItem::GetData() { return data_; }

ConfigDescriptor::ConfigDescriptor(size_t buffer_size, uint8_t bmAttributes,
                                   uint8_t bMaxPower)
    : bm_attributes_(bmAttributes), b_max_power_(bMaxPower)
{
  ASSERT(buffer_size > 0);
  buffer_.addr_ = new uint8_t[buffer_size];
  buffer_.size_ = buffer_size;
}

size_t ConfigDescriptor::CalcMaxConfigSize(
    const std::initializer_list<const std::initializer_list<ConfigDescriptorItem*>>&
        configs)
{
  return calc_max_config_size(configs);
}

LibXR::ErrorCode ConfigDescriptor::BuildConfigDescriptor(
    ConfigDescriptorItem* const* items, size_t item_num, uint8_t configuration_value,
    uint8_t i_configuration)
{
  if (items == nullptr || item_num == 0)
  {
    return LibXR::ErrorCode::ARG_ERR;
  }

  uint8_t* buffer = reinterpret_cast<uint8_t*>(buffer_.addr_);
  Header* header = reinterpret_cast<Header*>(buffer);

  // 先清空配置头，再按顺序拼接每个配置项的数据块。
  // Start from a clean configuration header, then append each item block in order.
  *header = Header{};
  header->bLength = sizeof(Header);
  header->bDescriptorType = 0x02;
  header->bConfigurationValue = configuration_value;
  header->iConfiguration = i_configuration;
  header->bmAttributes = bm_attributes_;
  header->bMaxPower = b_max_power_;

  size_t offset = sizeof(Header);
  uint8_t total_interfaces = 0;

  for (size_t i = 0; i < item_num; ++i)
  {
    auto* item = items[i];
    if (item == nullptr)
    {
      continue;
    }

    auto data = item->GetData();
    ASSERT(offset + data.size_ <= buffer_.size_);
    LibXR::Memory::FastCopy(&buffer[offset], data.addr_, data.size_);
    offset += data.size_;

    total_interfaces = static_cast<uint8_t>(total_interfaces + item->GetInterfaceCount());
  }

  ASSERT(offset <= 0xFFFF);
  header->wTotalLength = static_cast<uint16_t>(offset);
  header->bNumInterfaces = total_interfaces;

  buffer_index_ = offset;
  return LibXR::ErrorCode::OK;
}

LibXR::RawData ConfigDescriptor::GetData() const
{
  return {buffer_.addr_, buffer_index_};
}
