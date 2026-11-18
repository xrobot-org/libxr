#include "device_composition.hpp"

#include "libxr_mem.hpp"
#include "winusb_msos20.hpp"

using namespace LibXR::USB;
using LibXR::ConstRawData;
using LibXR::ErrorCode;
using LibXR::RawData;
using LibXR::USB::BosCapability;
using LibXR::USB::BosManager;
using LibXR::USB::CFG_SELF_POWERED;
using LibXR::USB::DescriptorStrings;
using LibXR::USB::DeviceDescriptor;
using LibXR::USB::Speed;
namespace WinUsb = LibXR::USB::WinUsbMsOs20;

namespace
{

using ConfigList = std::initializer_list<const std::initializer_list<DeviceClass*>>;

/// 接口字符串号从序列号之后开始 / Interface string indices start after the serial number
constexpr uint8_t FIRST_INTERFACE_STRING =
    static_cast<uint8_t>(DescriptorStrings::Index::SERIAL_NUMBER_STRING) + 1U;

/// UTF-8 字符串转成 UTF-16LE 后的字节数（只支持 BMP）/ Bytes of a UTF-8 string as
/// UTF-16LE (BMP only)
size_t Utf16Length(const char* text)
{
  size_t length = 0;
  for (size_t i = 0; text[i] != '\0';)
  {
    const auto c = static_cast<unsigned char>(text[i]);
    if (c < 0x80U)
    {
      i += 1U;
    }
    else if ((c & 0xE0U) == 0xC0U)
    {
      i += 2U;
    }
    else if ((c & 0xF0U) == 0xE0U)
    {
      i += 3U;
    }
    else
    {
      // BMP 之外的码点跳过，不占字节。
      // Code points outside the BMP are skipped and take no bytes.
      i += 4U;
      continue;
    }
    length += 2U;
  }
  return length;
}

/// 把 UTF-8 写成 UTF-16LE，规则同 Utf16Length / Write UTF-8 as UTF-16LE, same rules as
/// Utf16Length
void ToUtf16(const char* text, uint8_t* out)
{
  const auto* s = reinterpret_cast<const unsigned char*>(text);
  while (*s != 0U)
  {
    uint32_t code = 0U;
    if (*s < 0x80U)
    {
      code = *s++;
    }
    else if ((*s & 0xE0U) == 0xC0U)
    {
      code = (s[0] & 0x1FU) << 6U | (s[1] & 0x3FU);
      s += 2;
    }
    else if ((*s & 0xF0U) == 0xE0U)
    {
      code = (s[0] & 0x0FU) << 12U | (s[1] & 0x3FU) << 6U | (s[2] & 0x3FU);
      s += 3;
    }
    else
    {
      s += 4;
      continue;
    }
    *out++ = static_cast<uint8_t>(code);
    *out++ = static_cast<uint8_t>(code >> 8U);
  }
}

/// 所有配置中的类指针总数（去重前）/ Number of class pointers over all configurations
/// (before deduplication)
size_t ClassSlots(const ConfigList& configs)
{
  size_t total = 0;
  for (const auto& group : configs)
  {
    total += group.size();
  }
  return total;
}

/// 一个要 WinUSB 的功能 / One function that wants WinUSB
struct WinUsbFunction
{
  uint8_t configuration_index;  ///< 从 0 起的配置序号 / Zero-based configuration index
  uint8_t first_interface;      ///< 第一个接口号 / First interface number
  const char* guid;             ///< 接口 GUID / Interface GUID
};

/**
 * @brief 由各功能写出 MS OS 2.0 描述符集 / Write the MS OS 2.0 descriptor set of the
 *        functions
 * @note 只有一个功能且不是复合设备时用设备级形式，否则每个配置一个配置子集、每个功能一个
 *       功能子集。内存只分配这一次。
 *       A single function on a non-composite device gets the device-level form;
 *       otherwise one configuration subset per configuration and one function subset
 *       per function. The memory is allocated this once.
 */
ConstRawData BuildWinUsbSet(const WinUsbFunction* functions, size_t count,
                            bool device_level)
{
  using Property =
      WinUsb::DeviceInterfaceGuidsRegProperty<WinUsb::GUID_MULTI_SZ_UTF16_BYTES>;
  constexpr size_t FEATURES =
      sizeof(WinUsb::MsOs20FeatureCompatibleId) + sizeof(Property);
  using ConfigHeader = WinUsb::MsOs20SubsetHeaderConfiguration;
  using FunctionHeader = WinUsb::MsOs20SubsetHeaderFunction;

  size_t total = sizeof(WinUsb::MsOs20SetHeader);
  for (size_t i = 0; i < count; ++i)
  {
    const bool new_config = i == 0U || functions[i].configuration_index !=
                                           functions[i - 1U].configuration_index;
    total += device_level ? FEATURES
                          : (new_config ? sizeof(ConfigHeader) : 0U) +
                                sizeof(FunctionHeader) + FEATURES;
  }

  auto* out = new uint8_t[total];
  size_t offset = 0;
  auto put = [&](const void* source, size_t length)
  {
    LibXR::Memory::FastCopy(out + offset, source, length);
    offset += length;
  };
  // 子集头的最后两个字节是它到集尾（或下一个同级子集）的长度。
  // The last two bytes of a subset header are its length up to the next sibling.
  auto close = [&](size_t header_offset, size_t header_size)
  {
    const auto length = static_cast<uint16_t>(offset - header_offset);
    LibXR::Memory::FastCopy(out + header_offset + header_size - sizeof(length), &length,
                            sizeof(length));
  };

  WinUsb::MsOs20SetHeader set;
  set.wTotalLength = static_cast<uint16_t>(total);
  put(&set, sizeof(set));
  WinUsb::MsOs20FeatureCompatibleId compat;
  Property property;
  size_t config_offset = 0;
  for (size_t i = 0; i < count; ++i)
  {
    property.Init(functions[i].guid);
    if (device_level)
    {
      put(&compat, sizeof(compat));
      put(&property, sizeof(property));
      continue;
    }
    if (i == 0U ||
        functions[i].configuration_index != functions[i - 1U].configuration_index)
    {
      if (i != 0U)
      {
        close(config_offset, sizeof(ConfigHeader));
      }
      config_offset = offset;
      ConfigHeader config;
      config.bConfigurationValue = functions[i].configuration_index;
      put(&config, sizeof(config));
    }
    const size_t function_offset = offset;
    FunctionHeader function;
    function.bFirstInterface = functions[i].first_interface;
    put(&function, sizeof(function));
    put(&compat, sizeof(compat));
    put(&property, sizeof(property));
    close(function_offset, sizeof(FunctionHeader));
  }
  if (!device_level && count != 0U)
  {
    close(config_offset, sizeof(ConfigHeader));
  }
  ASSERT(offset == total);
  return {out, total};
}

}  // namespace

DeviceComposition::DeviceComposition(
    EndpointPool& pool, DeviceService& service,
    const std::initializer_list<const DescriptorStrings::LanguagePack*>& languages,
    const std::initializer_list<const std::initializer_list<DeviceClass*>>& configs,
    ConstRawData uid, uint8_t bm_attributes, uint8_t max_power)
    : pool_(pool),
      configs_(new Config[configs.size()]),
      config_count_(static_cast<uint8_t>(configs.size())),
      classes_(new DeviceClass*[ClassSlots(configs)]),
      bm_attributes_(bm_attributes),
      max_power_(max_power),
      strings_(languages, static_cast<const uint8_t*>(uid.addr_), uid.size_),
      // BOS 的容量按全部类的能力加一个 WinUSB 能力算；在构造体之前算，所以就地遍历。
      // The BOS is sized for every class's capabilities plus one WinUSB capability;
      // computed before the body, hence the walk in place.
      bos_(
          [&configs]
          {
            size_t bytes = LibXR::USB::BOS_HEADER_SIZE + LibXR::USB::USB2_EXT_CAP_SIZE +
                           sizeof(WinUsb::MsOs20PlatformCapability);
            for (const auto& group : configs)
            {
              for (DeviceClass* item : group)
              {
                for (size_t i = 0; i < item->GetBosCapabilityCount(); ++i)
                {
                  BosCapability* cap = item->GetBosCapability(i);
                  bytes += (cap != nullptr) ? cap->GetCapabilityDescriptor().size_ : 0U;
                }
              }
            }
            return bytes;
          }(),
          [&configs]
          {
            size_t count = 1U;
            for (const auto& group : configs)
            {
              for (DeviceClass* item : group)
              {
                count += item->GetBosCapabilityCount();
              }
            }
            return count;
          }())
{
  ASSERT(config_count_ > 0U && configs.size() <= UINT8_MAX);

  // 整理配置表和去重的类表；给每个类写入服务指针。
  // Build the configuration tables and the deduplicated class table; give every class
  // its service pointer.
  size_t largest = 0;
  size_t config_index = 0;
  for (const auto& group : configs)
  {
    Config& config = configs_[config_index++];
    config.classes = new DeviceClass*[group.size()];
    config.count = group.size();
    composite_ = composite_ || group.size() > 1U;
    size_t size = sizeof(DeviceClass::Header);
    size_t n = 0;
    for (DeviceClass* item : group)
    {
      ASSERT(item != nullptr);
      config.classes[n++] = item;
      size += item->GetMaxConfigSize();
      composite_ = composite_ || item->HasIAD();
      bool known = false;
      for (size_t i = 0; i < class_count_; ++i)
      {
        known = known || classes_[i] == item;
      }
      if (!known)
      {
        item->service_ = &service;
        classes_[class_count_++] = item;
      }
    }
    largest = (size > largest) ? size : largest;
  }
  config_buffer_ = {new uint8_t[largest], largest};

  RegisterInterfaceStrings();

  // WinUSB：接口号按配置内累加，与 SetConfiguration 给类的一致。
  // WinUSB: interface numbers accumulate within a configuration, as SetConfiguration
  // gives them to the classes.
  size_t functions = 0;
  for (size_t c = 0; c < config_count_; ++c)
  {
    for (size_t i = 0; i < configs_[c].count; ++i)
    {
      functions +=
          (configs_[c].classes[i]->GetWinUsbInterfaceGuid() != nullptr) ? 1U : 0U;
    }
  }
  if (functions != 0U)
  {
    auto* info = new WinUsbFunction[functions];
    size_t n = 0;
    for (size_t c = 0; c < config_count_; ++c)
    {
      size_t first_interface = 0;
      for (size_t i = 0; i < configs_[c].count; ++i)
      {
        DeviceClass* item = configs_[c].classes[i];
        if (const char* guid = item->GetWinUsbInterfaceGuid())
        {
          info[n++] = {static_cast<uint8_t>(c), static_cast<uint8_t>(first_interface),
                       guid};
        }
        first_interface += item->GetInterfaceCount();
      }
    }
    winusb_cap_ = new WinUsb::MsOs20BosCapability(
        BuildWinUsbSet(info, functions, !composite_ && functions == 1U));
    delete[] info;
    bos_.AddCapability(winusb_cap_);
  }
  for (size_t i = 0; i < class_count_; ++i)
  {
    for (size_t j = 0; j < classes_[i]->GetBosCapabilityCount(); ++j)
    {
      if (BosCapability* cap = classes_[i]->GetBosCapability(j))
      {
        bos_.AddCapability(cap);
      }
    }
  }
}

ErrorCode DeviceComposition::SetConfiguration(uint8_t value, Speed speed, bool in_isr)
{
  if (value > config_count_)
  {
    return ErrorCode::ARG_ERR;
  }
  Unbind(in_isr);
  if (value == 0U)
  {
    return ErrorCode::OK;
  }
  const Config& config = configs_[value - 1U];
  uint8_t first_interface = 0;
  size_t bound = 0;
  ErrorCode result = ErrorCode::OK;
  for (size_t i = 0; i < config.count && result == ErrorCode::OK; ++i)
  {
    DeviceClass* item = config.classes[i];
    ConstRawData block{nullptr, 0};
    result = item->WriteDescriptors(pool_, speed, first_interface, block);
    if (result != ErrorCode::OK)
    {
      break;
    }
    result = item->BindEndpoints(pool_, first_interface, in_isr);
    bound = i + 1U;
    first_interface = static_cast<uint8_t>(first_interface + item->GetInterfaceCount());
  }
  if (result != ErrorCode::OK)
  {
    // 失败的类也解绑：它的 Unbind 撤销自己做了一半的绑定。
    // The failing class is unbound too: its Unbind undoes its half-done bind.
    for (size_t i = 0; i < bound; ++i)
    {
      config.classes[i]->UnbindEndpoints(pool_, in_isr);
    }
    return result;
  }
  configuration_ = value;
  return ErrorCode::OK;
}

ErrorCode DeviceComposition::WriteConfigDescriptor(uint8_t index, Speed speed,
                                                   RawData& data)
{
  if (index >= config_count_)
  {
    return ErrorCode::ARG_ERR;
  }
  const Config& config = configs_[index];
  auto* buffer = static_cast<uint8_t*>(config_buffer_.addr_);
  size_t offset = sizeof(DeviceClass::Header);
  uint8_t first_interface = 0;
  for (size_t i = 0; i < config.count; ++i)
  {
    DeviceClass* item = config.classes[i];
    ConstRawData block{nullptr, 0};
    const ErrorCode result = item->WriteDescriptors(pool_, speed, first_interface, block);
    if (result != ErrorCode::OK)
    {
      return result;
    }
    ASSERT(block.size_ <= item->GetMaxConfigSize());
    ASSERT(offset + block.size_ <= config_buffer_.size_);
    LibXR::Memory::FastCopy(buffer + offset, block.addr_, block.size_);
    offset += block.size_;
    first_interface = static_cast<uint8_t>(first_interface + item->GetInterfaceCount());
  }
  ASSERT(offset <= UINT16_MAX);
  DeviceClass::Header header{};
  header.bLength = sizeof(DeviceClass::Header);
  header.bDescriptorType =
      static_cast<uint8_t>(LibXR::USB::DescriptorType::CONFIGURATION);
  header.wTotalLength = static_cast<uint16_t>(offset);
  header.bNumInterfaces = first_interface;
  header.bConfigurationValue = static_cast<uint8_t>(index + 1U);
  header.iConfiguration = 0U;
  header.bmAttributes = bm_attributes_;
  header.bMaxPower = max_power_;
  LibXR::Memory::FastCopy(buffer, &header, sizeof(header));
  data = {buffer, offset};
  return ErrorCode::OK;
}

ErrorCode DeviceComposition::GetStringDescriptor(uint8_t index, uint16_t lang,
                                                 ConstRawData& data)
{
  if (index == 0U)
  {
    data = strings_.GetLangIDData();
    return ErrorCode::OK;
  }
  if (index >= FIRST_INTERFACE_STRING)
  {
    // 接口字符串与内建字符串用同一个语言检查。
    // Interface strings pass the same language check as the built-in ones.
    if (!strings_.HasLanguage(lang))
    {
      return ErrorCode::NOT_FOUND;
    }
    return GenerateInterfaceString(index, data);
  }
  const ErrorCode result =
      strings_.GenerateString(static_cast<DescriptorStrings::Index>(index), lang);
  if (result != ErrorCode::OK)
  {
    return result;
  }
  data = strings_.GetData();
  return ErrorCode::OK;
}

DeviceClass* DeviceComposition::FindClassByInterface(uint8_t itf) const
{
  if (configuration_ == 0U)
  {
    return nullptr;
  }
  const Config& config = configs_[configuration_ - 1U];
  size_t first = 0;
  for (size_t i = 0; i < config.count; ++i)
  {
    first += config.classes[i]->GetInterfaceCount();
    if (itf < first)
    {
      return config.classes[i];
    }
  }
  return nullptr;
}

DeviceClass* DeviceComposition::FindClassByEndpoint(uint8_t ep_addr) const
{
  if (configuration_ == 0U)
  {
    return nullptr;
  }
  const Config& config = configs_[configuration_ - 1U];
  for (size_t i = 0; i < config.count; ++i)
  {
    if (config.classes[i]->OwnsEndpoint(ep_addr))
    {
      return config.classes[i];
    }
  }
  return nullptr;
}

uint16_t DeviceComposition::DeviceStatus() const
{
  return ((bm_attributes_ & CFG_SELF_POWERED) != 0U) ? 1U : 0U;
}

size_t DeviceComposition::ControlReceiveCapacity() const
{
  size_t capacity = 64U;
  for (size_t i = 0; i < class_count_; ++i)
  {
    const size_t wanted = classes_[i]->GetControlReceiveCapacity();
    capacity = (wanted > capacity) ? wanted : capacity;
  }
  return capacity;
}

ErrorCode DeviceComposition::WriteDeviceDescriptor(DeviceDescriptor& descriptor)
{
  // 只有一个配置、一个类、一个接口、没有 IAD 的设备才由类改写。
  // Only a device with one configuration, one class, one interface and no IAD lets
  // its class rewrite the descriptor.
  if (config_count_ != 1U || composite_ ||
      configs_[0].classes[0]->GetInterfaceCount() != 1U)
  {
    return ErrorCode::NOT_SUPPORT;
  }
  return configs_[0].classes[0]->WriteDeviceDescriptor(descriptor);
}

void DeviceComposition::ServiceClasses(bool in_isr)
{
  for (size_t i = 0; i < class_count_; ++i)
  {
    if (classes_[i]->service_pending_.exchange(false, std::memory_order_acq_rel))
    {
      classes_[i]->OnService(in_isr);
    }
  }
}

void DeviceComposition::StartOfFrame(bool in_isr, uint16_t frame, uint8_t microframe)
{
  if (configuration_ == 0U)
  {
    return;
  }
  const Config& config = configs_[configuration_ - 1U];
  for (size_t i = 0; i < config.count; ++i)
  {
    config.classes[i]->OnStartOfFrame(in_isr, frame, microframe);
  }
}

void DeviceComposition::Unbind(bool in_isr)
{
  if (configuration_ == 0U)
  {
    return;
  }
  const Config& config = configs_[configuration_ - 1U];
  // 先记为未配置：解绑期间的回调看到的已是未配置。
  // Record unconfigured first: callbacks during the unbind already see it.
  configuration_ = 0U;
  for (size_t i = 0; i < config.count; ++i)
  {
    config.classes[i]->UnbindEndpoints(pool_, in_isr);
  }
}

void DeviceComposition::RegisterInterfaceStrings()
{
  size_t longest = 0;
  for (size_t c = 0; c < class_count_; ++c)
  {
    for (size_t i = 0; i < classes_[c]->GetInterfaceCount(); ++i)
    {
      if (const char* text = classes_[c]->GetInterfaceString(i))
      {
        ++interface_string_count_;
        const size_t length = Utf16Length(text);
        longest = (length > longest) ? length : longest;
      }
    }
  }
  if (interface_string_count_ == 0U)
  {
    return;
  }
  ASSERT(FIRST_INTERFACE_STRING + interface_string_count_ <= 256U &&
         longest + 2U <= 255U);
  interface_strings_ = new const char*[interface_string_count_];
  string_buffer_ = {new uint8_t[longest + 2U], longest + 2U};

  size_t n = 0;
  for (size_t c = 0; c < class_count_; ++c)
  {
    DeviceClass* item = classes_[c];
    for (size_t i = 0; i < item->GetInterfaceCount(); ++i)
    {
      if (const char* text = item->GetInterfaceString(i))
      {
        if (item->interface_string_base_ == 0U)
        {
          item->interface_string_base_ = static_cast<uint8_t>(FIRST_INTERFACE_STRING + n);
        }
        interface_strings_[n++] = text;
      }
    }
  }
}

ErrorCode DeviceComposition::GenerateInterfaceString(uint8_t index, ConstRawData& data)
{
  const size_t n = static_cast<size_t>(index - FIRST_INTERFACE_STRING);
  if (n >= interface_string_count_)
  {
    return ErrorCode::NOT_FOUND;
  }
  auto* buffer = static_cast<uint8_t*>(string_buffer_.addr_);
  const char* text = interface_strings_[n];
  const size_t length = Utf16Length(text) + 2U;
  buffer[0] = static_cast<uint8_t>(length);
  buffer[1] = static_cast<uint8_t>(LibXR::USB::DescriptorType::STRING);
  ToUtf16(text, buffer + 2);
  data = {buffer, length};
  return ErrorCode::OK;
}
