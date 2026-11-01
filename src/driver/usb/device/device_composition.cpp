#include "device_composition.hpp"

#include <cstddef>
#include <cstdint>

namespace
{
using LibXR::USB::BOS_HEADER_SIZE;
using LibXR::USB::BosCapability;
using LibXR::USB::CFG_REMOTE_WAKEUP;
using LibXR::USB::CFG_SELF_POWERED;
using LibXR::USB::ConfigDescriptorItem;
using LibXR::USB::DESCRIPTOR_TYPE_DEVICE_CAPABILITY;
using LibXR::USB::DescriptorStrings;
using LibXR::USB::DEV_CAPABILITY_TYPE_USB20EXT;
using LibXR::USB::DeviceClass;

struct InterfaceStringLayout
{
  size_t count = 0;
  size_t max_len = 0;
};

// 在 class 去重前，先统计原始配置项数量。
// Count the raw number of config items before unique-class deduplication.
static size_t calc_total_item_num(
    const std::initializer_list<const std::initializer_list<ConfigDescriptorItem*>>&
        configs)
{
  // 这里先统计原始 item 数量；最终唯一 class 表会在构造函数里去重。
  // Count raw items first; the final unique-class table is deduplicated in the
  // constructor.
  size_t total = 0;
  for (const auto& group : configs)
  {
    total += group.size();
  }
  return total;
}

// 构造期唯一 class 表很小，线性查重已经足够。
// The constructor-time unique-class table is tiny, so a linear contains check is
// sufficient.
static bool contains_class(DeviceClass* const* list, size_t count,
                           const DeviceClass* item)
{
  for (size_t i = 0; i < count; ++i)
  {
    if (list[i] == item)
    {
      return true;
    }
  }
  return false;
}

// 计算 UTF-8 接口字符串转换成 UTF-16LE 后的有效载荷长度。
// Compute the UTF-16LE payload size produced from a UTF-8 interface string.
static size_t calc_utf16le_len_runtime(const char* input)
{
  if (input == nullptr)
  {
    return 0;
  }

  size_t len = 0;
  for (size_t i = 0; input[i];)
  {
    unsigned char c = static_cast<unsigned char>(input[i]);
    if (c < 0x80)
    {
      len += 2;
      i += 1;
    }
    else if ((c & 0xE0) == 0xC0)
    {
      len += 2;
      i += 2;
    }
    else if ((c & 0xF0) == 0xE0)
    {
      len += 2;
      i += 3;
    }
    else
    {
      i += 4;
    }
  }
  return len;
}

// 把 UTF-8 接口字符串转换成 USB 字符串描述符使用的 UTF-16LE；
// 不支持的码点保持原先的保守跳过策略。
// Convert a UTF-8 interface string into UTF-16LE for USB string descriptors.
// Unsupported code points are skipped in the same conservative way as before.
static void to_utf16le(const char* str, uint8_t* buffer)
{
  size_t len = 0;
  const unsigned char* s = reinterpret_cast<const unsigned char*>(str);

  while (*s)
  {
    uint32_t codepoint = 0;

    if (*s < 0x80)
    {
      codepoint = *s++;
    }
    else if ((*s & 0xE0) == 0xC0)
    {
      codepoint = (*s & 0x1F) << 6;
      s++;
      codepoint |= (*s & 0x3F);
      s++;
    }
    else if ((*s & 0xF0) == 0xE0)
    {
      codepoint = (*s & 0x0F) << 12;
      s++;
      codepoint |= (*s & 0x3F) << 6;
      s++;
      codepoint |= (*s & 0x3F);
      s++;
    }
    else if ((*s & 0xF8) == 0xF0)
    {
      s++;
      if (*s)
      {
        s++;
      }
      if (*s)
      {
        s++;
      }
      if (*s)
      {
        s++;
      }
      continue;
    }
    else
    {
      s++;
      continue;
    }

    buffer[len++] = codepoint & 0xFF;
    buffer[len++] = (codepoint >> 8) & 0xFF;
  }
}

// 预先算出接口字符串总数，以及运行时生成描述符所需的最大缓冲区。
// Pre-compute the interface-string count and the largest runtime descriptor
// buffer needed.
static InterfaceStringLayout calc_interface_string_layout(DeviceClass* const* classes,
                                                          size_t class_count)
{
  // 接口字符串描述符在运行时生成，但其源字符串指针和最大 UTF-16LE
  // 空间可在构造期一次算清。
  // Interface strings are generated at runtime, but their source pointers and
  // maximum UTF-16LE descriptor size can be fixed up front during construction.
  InterfaceStringLayout layout{};
  for (size_t class_index = 0; class_index < class_count; ++class_index)
  {
    auto* device_class = classes[class_index];
    if (device_class == nullptr)
    {
      continue;
    }

    const size_t interface_num = device_class->GetInterfaceCount();
    for (size_t i = 0; i < interface_num; ++i)
    {
      const char* str = device_class->GetInterfaceString(i);
      if (str == nullptr || str[0] == '\0')
      {
        continue;
      }

      ++layout.count;
      const size_t utf16_len = calc_utf16le_len_runtime(str);
      if (utf16_len > layout.max_len)
      {
        layout.max_len = utf16_len;
      }
    }
  }
  return layout;
}

// 一个 configuration 只要有多个 item，或包含任意 IAD，就按复合设备处理。
// A configuration is treated as composite if it exposes multiple items or any IAD.
static bool config_contains_iad(const std::initializer_list<ConfigDescriptorItem*>& group)
{
  for (auto* item : group)
  {
    if (item != nullptr && item->HasIAD())
    {
      return true;
    }
  }
  return false;
}

static bool is_composite_config(const std::initializer_list<ConfigDescriptorItem*>& group)
{
  return (group.size() > 1) || config_contains_iad(group);
}

// 只有“单 class、单接口、无 IAD”的简单形态，
// 才允许覆盖 device descriptor 的类字段。
// Device-descriptor override is only valid for the simple
// "single class, single interface, no IAD" shape.
static bool is_device_descriptor_override_eligible(ConfigDescriptorItem* const* items,
                                                   size_t item_num)
{
  if (item_num != 1)
  {
    return false;
  }

  auto* item = items[0];
  if (item == nullptr)
  {
    return false;
  }

  if (item->HasIAD())
  {
    return false;
  }

  return (item->GetInterfaceCount() == 1);
}

static bool is_composite_device(
    const std::initializer_list<const std::initializer_list<ConfigDescriptorItem*>>&
        configs)
{
  for (const auto& group : configs)
  {
    if (is_composite_config(group))
    {
      return true;
    }
  }
  return false;
}

}  // namespace

using namespace LibXR::USB;
using LibXR::ConstRawData;
using LibXR::RawData;

DeviceComposition::DeviceComposition(
    EndpointPool& endpoint_pool,
    const std::initializer_list<const DescriptorStrings::LanguagePack*>& lang_list,
    const std::initializer_list<const std::initializer_list<ConfigDescriptorItem*>>&
        configs,
    ConstRawData uid, uint8_t bmAttributes, uint8_t bMaxPower)
    : endpoint_pool_(endpoint_pool),
      bm_attributes_(bmAttributes),
      composite_(is_composite_device(configs)),
      config_num_(configs.size()),
      items_(new ConfigItems[config_num_]),
      classes_(new DeviceClass*[calc_total_item_num(configs)]),
      strings_(lang_list, reinterpret_cast<const uint8_t*>(uid.addr_), uid.size_),
      config_desc_(ConfigDescriptor::CalcMaxConfigSize(configs), bmAttributes, bMaxPower)
{
  ASSERT(config_num_ > 0);

  // 将所有配置项整理成：
  // 1) 每个 configuration 自己的 item 表
  // 2) 一份用于字符串注册的唯一 class 表
  // Flatten every configuration item into:
  // 1) per-config item tables for descriptor building / binding
  // 2) one unique class table for string registration
  size_t config_index = 0;
  for (const auto& cfg_group : configs)
  {
    items_[config_index].item_num = cfg_group.size();
    items_[config_index].items = new ConfigDescriptorItem*[cfg_group.size()];

    size_t item_index = 0;
    for (auto* item : cfg_group)
    {
      items_[config_index].items[item_index++] = item;
      auto* device_class = static_cast<DeviceClass*>(item);
      if (device_class != nullptr &&
          !contains_class(classes_, class_count_, device_class))
      {
        device_class->service_pool_ = &endpoint_pool_;
        classes_[class_count_++] = device_class;
      }
    }

    ++config_index;
  }

  // 初始化阶段一次性算出接口字符串容量和最大描述符空间。
  // Pre-compute interface-string storage once during initialization.
  const auto interface_string_layout =
      calc_interface_string_layout(classes_, class_count_);
  interface_string_count_ = interface_string_layout.count;
  if (interface_string_count_ > 0)
  {
    interface_strings_ = new const char*[interface_string_count_];
    interface_string_buffer_.addr_ = new uint8_t[interface_string_layout.max_len + 2];
    interface_string_buffer_.size_ = interface_string_layout.max_len + 2;
  }

  RegisterInterfaceStrings();
  BuildWinUsbDescriptorSet();
}

void DeviceComposition::BuildWinUsbDescriptorSet()
{
  size_t count = 0;
  for (size_t config = 0; config < config_num_; ++config)
  {
    for (size_t i = 0; i < items_[config].item_num; ++i)
    {
      auto* device_class = static_cast<DeviceClass*>(items_[config].items[i]);
      if (device_class != nullptr && device_class->GetWinUsbInterfaceGuid() != nullptr)
      {
        ++count;
      }
    }
  }
  if (count == 0)
  {
    return;
  }

  // 接口号按配置内的累加得到，和 BindEndpoints 分配的一致。
  // Interface numbers accumulate within a configuration, as BindEndpoints assigns them.
  auto* functions = new WinUsbMsOs20::WinUsbFunctionInfo[count];
  size_t n = 0;
  for (size_t config = 0; config < config_num_; ++config)
  {
    size_t start_itf = 0;
    for (size_t i = 0; i < items_[config].item_num; ++i)
    {
      auto* item = items_[config].items[i];
      if (item == nullptr)
      {
        continue;
      }
      auto* device_class = static_cast<DeviceClass*>(item);
      if (const char* guid = device_class->GetWinUsbInterfaceGuid())
      {
        functions[n++] = {static_cast<uint8_t>(config), static_cast<uint8_t>(start_itf),
                          guid};
      }
      start_itf += item->GetInterfaceCount();
    }
  }
  winusb_set_.Build(functions, count, !composite_ && count == 1);
  delete[] functions;
  winusb_cap_ = new WinUsbMsOs20::MsOs20BosCapability(winusb_set_.Data());
}

const DeviceComposition::ConfigItems& DeviceComposition::CurrentConfigItems() const
{
  DEV_ASSERT(config_num_ > 0);
  DEV_ASSERT(current_cfg_ < config_num_);
  return items_[current_cfg_];
}

void DeviceComposition::Init(bool in_isr)
{
  endpoint_pool_.SetDataEnabled(false, in_isr);
  configured_ = false;
  current_cfg_ = 0;
}

void DeviceComposition::Deinit(bool in_isr)
{
  endpoint_pool_.SetDataEnabled(false, in_isr);
  UnbindEndpoints(in_isr);
  configured_ = false;
  current_cfg_ = 0;
}

LibXR::ErrorCode DeviceComposition::BeginSwitchConfig(size_t index, bool in_isr)
{
  if (index > config_num_) return ErrorCode::NOT_FOUND;
  if (index != 0 &&
      (!descriptors_prepared_ ||
       GetConfigDescriptor(index - 1U, endpoint_pool_.GetSpeed()).size_ == 0U))
    return ErrorCode::NOT_SUPPORT;
  endpoint_pool_.SetDataEnabled(false, in_isr);
  UnbindEndpoints(in_isr);
  configured_ = false;
  current_cfg_ = 0;
  return ErrorCode::OK;
}

LibXR::ErrorCode DeviceComposition::FinishSwitchConfig(size_t index, bool in_isr)
{
  if (index == 0) return ErrorCode::OK;
  current_cfg_ = static_cast<uint8_t>(index - 1U);
  BindEndpoints(in_isr);
  if (!endpoint_pool_.ConfigurationValid())
  {
    UnbindEndpoints(in_isr);
    current_cfg_ = 0;
    return ErrorCode::INIT_ERR;
  }
  configured_ = true;
  endpoint_pool_.SetDataEnabled(true, in_isr);
  return ErrorCode::OK;
}

LibXR::ErrorCode DeviceComposition::BuildConfigDescriptor()
{
  // ConfigDescriptor 只构建当前激活的 configuration；
  // current_cfg_ 同时决定 item 表和对外可见的 bConfigurationValue。
  // ConfigDescriptor builds the active configuration only;
  // current_cfg_ supplies both the item table and the externally visible
  // bConfigurationValue.
  const auto& config = CurrentConfigItems();
  return config_desc_.BuildConfigDescriptor(config.items, config.item_num,
                                            static_cast<uint8_t>(current_cfg_ + 1),
                                            i_configuration_);
}

RawData DeviceComposition::GetConfigDescriptor() const { return config_desc_.GetData(); }

LibXR::ErrorCode DeviceComposition::GetStringDescriptor(uint8_t string_index,
                                                        uint16_t lang, ConstRawData& data)
{
  if (string_index == 0)
  {
    data = ConstRawData(strings_.GetLangIDData());
    return LibXR::ErrorCode::OK;
  }

  if (string_index > static_cast<uint8_t>(DescriptorStrings::Index::SERIAL_NUMBER_STRING))
  {
    // 接口字符串与内建字符串集合共用同一套语言号校验。
    // Interface strings share the same language gate as the built-in string set.
    if (!strings_.HasLanguage(lang))
    {
      return LibXR::ErrorCode::NOT_FOUND;
    }
    return GenerateInterfaceString(string_index, data);
  }

  auto ec =
      strings_.GenerateString(static_cast<DescriptorStrings::Index>(string_index), lang);
  if (ec != LibXR::ErrorCode::OK)
  {
    return ec;
  }

  data = ConstRawData(strings_.GetData());
  return LibXR::ErrorCode::OK;
}

bool DeviceComposition::IsComposite() const { return composite_; }

LibXR::ErrorCode DeviceComposition::TryOverrideDeviceDescriptor(
    DeviceDescriptor& descriptor)
{
  if (config_num_ != 1)
  {
    return LibXR::ErrorCode::NOT_SUPPORT;
  }

  const auto config = items_[0];
  if (!is_device_descriptor_override_eligible(config.items, config.item_num))
  {
    return LibXR::ErrorCode::NOT_SUPPORT;
  }

  return config.items[0]->WriteDeviceDescriptor(descriptor);
}

size_t DeviceComposition::GetConfigNum() const { return config_num_; }

size_t DeviceComposition::GetCurrentConfig() const
{
  return configured_ ? static_cast<size_t>(current_cfg_ + 1u) : 0u;
}

uint16_t DeviceComposition::GetDeviceStatus() const
{
  return ((bm_attributes_ & CFG_SELF_POWERED) ? 0x01 : 0x00) |
         ((bm_attributes_ & CFG_REMOTE_WAKEUP) ? 0x02 : 0x00);
}

DeviceClass* DeviceComposition::FindClassByInterfaceNumber(size_t index) const
{
  if (!configured_)
  {
    return nullptr;
  }

  const auto& config = CurrentConfigItems();

  // 每个 item 都声明自己占用的接口数量；
  // 沿扁平表前进，直到目标接口落入某个 item 的区间。
  // Each item reports how many interfaces it occupies; walk the flattened list
  // until the requested interface falls into one item's local range.
  int interface_index = -1;
  for (size_t i = 0; i < config.item_num; ++i)
  {
    auto* item = config.items[i];
    if (item == nullptr)
    {
      continue;
    }

    interface_index += static_cast<int>(item->GetInterfaceCount());
    if (interface_index >= static_cast<int>(index))
    {
      return static_cast<DeviceClass*>(item);
    }
  }

  return nullptr;
}

DeviceClass* DeviceComposition::FindClassByEndpointAddress(uint8_t addr) const
{
  if (!configured_)
  {
    return nullptr;
  }

  const auto& config = CurrentConfigItems();

  for (size_t i = 0; i < config.item_num; ++i)
  {
    auto* item = config.items[i];
    if (item == nullptr)
    {
      continue;
    }

    if (item->OwnsEndpoint(addr))
    {
      return static_cast<DeviceClass*>(item);
    }
  }

  return nullptr;
}

void DeviceComposition::BindEndpoints(bool in_isr)
{
  if (ep_assigned_)
  {
    return;
  }
  ep_assigned_ = true;

  const auto& config = CurrentConfigItems();

  // start_itf 记录 configuration 级别的全局接口号分配游标。
  // start_itf tracks the configuration-global interface number assigned to each class.
  size_t start_itf = 0;
  for (size_t i = 0; i < config.item_num; ++i)
  {
    auto* item = config.items[i];
    if (item == nullptr)
    {
      continue;
    }

    item->BindEndpoints(endpoint_pool_, static_cast<uint8_t>(start_itf), in_isr);
    start_itf += item->GetInterfaceCount();
  }
}

void DeviceComposition::UnbindEndpoints(bool in_isr)
{
  if (!ep_assigned_)
  {
    return;
  }
  ep_assigned_ = false;

  // Unbind 走的仍是当前激活 item 表，但这里不再需要重新分配接口号。
  // Unbind walks the same active item table, but no interface renumbering is needed here.
  const auto& config = CurrentConfigItems();
  for (size_t i = 0; i < config.item_num; ++i)
  {
    auto* item = config.items[i];
    if (item == nullptr)
    {
      continue;
    }

    item->UnbindEndpoints(endpoint_pool_, in_isr);
  }
}

LibXR::ErrorCode DeviceComposition::GenerateInterfaceString(uint8_t string_index,
                                                            ConstRawData& data)
{
  // 接口字符串位于内建 manufacturer/product/serial 之后的索引区间。
  // Interface strings live after the built-in manufacturer/product/serial range.
  const size_t base_index =
      static_cast<uint8_t>(DescriptorStrings::Index::SERIAL_NUMBER_STRING);
  if (string_index <= base_index)
  {
    return LibXR::ErrorCode::NOT_FOUND;
  }

  const size_t extra_index = static_cast<size_t>(string_index - base_index - 1u);
  if (extra_index >= interface_string_count_ || interface_string_buffer_.addr_ == nullptr)
  {
    return LibXR::ErrorCode::NOT_FOUND;
  }

  uint8_t* buffer = reinterpret_cast<uint8_t*>(interface_string_buffer_.addr_);
  const char* str = interface_strings_[extra_index];
  const size_t utf16_len = calc_utf16le_len_runtime(str);
  ASSERT(utf16_len + 2 <= 255);

  // USB 字符串描述符格式：[bLength][bDescriptorType=0x03][UTF-16LE payload...]
  // USB string descriptor layout: [bLength][bDescriptorType=0x03][UTF-16LE payload...]
  buffer[1] = 0x03;
  buffer[0] = static_cast<uint8_t>(utf16_len + 2);
  to_utf16le(str, buffer + 2);
  data = ConstRawData(interface_string_buffer_.addr_, buffer[0]);
  return LibXR::ErrorCode::OK;
}

void DeviceComposition::RegisterInterfaceStrings()
{
  if (interface_string_count_ == 0)
  {
    return;
  }

  // USB 字符串索引 1..3 保留给厂商/产品/序列号。
  // USB string indices 1..3 are reserved for manufacturer/product/serial.
  uint8_t next_index =
      static_cast<uint8_t>(DescriptorStrings::Index::SERIAL_NUMBER_STRING) + 1u;
  size_t registered_count = 0u;

  for (size_t class_index = 0; class_index < class_count_; ++class_index)
  {
    auto* device_class = classes_[class_index];
    if (device_class == nullptr)
    {
      continue;
    }

    const size_t interface_num = device_class->GetInterfaceCount();
    device_class->SetInterfaceStringBaseIndex(0u);
    bool class_has_string = false;

    for (size_t i = 0; i < interface_num; ++i)
    {
      const char* str = device_class->GetInterfaceString(i);
      if (str == nullptr || str[0] == '\0')
      {
        continue;
      }

      // 这里保留一张扁平源字符串表，
      // 这样 DeviceCore 之后按索引取接口字符串时，不必再重新遍历全部 class。
      // 保留一张扁平源字符串表，
      // 这样 DeviceCore 之后可以按索引重新生成任意接口字符串，而不必重扫全部 class。
      // Keep a flat source-string table so DeviceCore can later regenerate any
      // interface string by index without re-scanning all classes.
      DEV_ASSERT(registered_count < interface_string_count_);
      if (!class_has_string)
      {
        device_class->SetInterfaceStringBaseIndex(next_index);
        class_has_string = true;
      }
      interface_strings_[registered_count++] = str;
      ++next_index;
    }
  }

  DEV_ASSERT(registered_count == interface_string_count_);
}

LibXR::ErrorCode DeviceComposition::PrepareDescriptors(uint32_t speeds, bool in_isr)
{
  if (descriptors_prepared_) return ErrorCode::OK;
  if (descriptors_ == nullptr) descriptors_ = new DescriptorCache[config_num_ * 3U];
  const Speed current_speed = endpoint_pool_.GetSpeed();
  endpoint_pool_.SetPlanning(true);
  ErrorCode result = ErrorCode::OK;
  // Prepare the largest supported speed first. Memory grows only in this
  // initialization-time phase, never during a host-selected configuration.
  for (int speed = static_cast<int>(Speed::HIGH); speed >= 0; --speed)
  {
    if ((speeds & (1U << speed)) == 0U) continue;
    endpoint_pool_.SetSpeed(static_cast<Speed>(speed));
    for (size_t cfg = 0; cfg < config_num_; ++cfg)
    {
      current_cfg_ = static_cast<uint8_t>(cfg);
      BindEndpoints(in_isr);
      result = endpoint_pool_.ConfigurationValid() ? BuildConfigDescriptor()
                                                   : ErrorCode::INIT_ERR;
      if (result == ErrorCode::OK)
      {
        const RawData data = config_desc_.GetData();
        auto& cache = descriptors_[cfg * 3U + static_cast<size_t>(speed)];
        cache.data = {new uint8_t[data.size_], data.size_};
        Memory::FastCopy(cache.data.addr_, data.addr_, data.size_);
      }
      UnbindEndpoints(in_isr);
      if (result != ErrorCode::OK) break;
    }
    if (result != ErrorCode::OK) break;
  }
  current_cfg_ = 0;
  configured_ = false;
  endpoint_pool_.SetSpeed(current_speed);
  endpoint_pool_.SetPlanning(false);
  descriptors_prepared_ = result == ErrorCode::OK;
  return result;
}

LibXR::ConstRawData DeviceComposition::GetConfigDescriptor(size_t index,
                                                           Speed speed) const
{
  const auto raw_speed = static_cast<size_t>(speed);
  if (descriptors_ == nullptr || index >= config_num_ || raw_speed >= 3U)
    return {nullptr, 0};
  const auto& cache = descriptors_[index * 3U + raw_speed];
  return {cache.data.addr_, cache.data.size_};
}

size_t DeviceComposition::ControlReceiveCapacity() const
{
  size_t capacity = 64;
  for (size_t i = 0; i < class_count_; ++i)
    capacity = LibXR::max(capacity, classes_[i]->GetControlReceiveCapacity());
  return capacity;
}

bool DeviceComposition::ContainsActiveClass(const DeviceClass* item) const
{
  if (!configured_) return false;
  const auto& config = CurrentConfigItems();
  for (size_t i = 0; i < config.item_num; ++i)
    if (config.items[i] == item) return true;
  return false;
}

void DeviceComposition::Notify(bool in_isr, DeviceEvent event, uint8_t endpoint)
{
  for (size_t i = 0; i < class_count_; ++i)
  {
    if (event == DeviceEvent::RESET || ContainsActiveClass(classes_[i]))
    {
      if (endpoint == 0U || classes_[i]->OwnsEndpoint(endpoint))
        classes_[i]->OnDeviceEvent(in_isr, event, endpoint);
    }
  }
}

bool DeviceComposition::WantsBusTime() const
{
  for (size_t i = 0; i < class_count_; ++i)
    if (classes_[i]->WantsBusTime()) return true;
  return false;
}

void DeviceComposition::NotifyBusTime(bool in_isr, const BusTime& time)
{
  for (size_t i = 0; i < class_count_; ++i)
    if (ContainsActiveClass(classes_[i]) && classes_[i]->WantsBusTime())
      classes_[i]->OnBusTime(in_isr, time);
}
