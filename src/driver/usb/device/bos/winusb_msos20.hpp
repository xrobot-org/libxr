#pragma once
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "libxr_mem.hpp"
#include "libxr_type.hpp"
#include "usb/core/bos.hpp"
#include "usb/core/core.hpp"

namespace LibXR::USB::WinUsbMsOs20
{

// ---- constants ----

// wIndex values for vendor request
static constexpr uint16_t MSOS20_DESCRIPTOR_INDEX = 0x0007;
static constexpr uint16_t MSOS20_SET_ALT_ENUMERATION = 0x0008;

// MS OS 2.0 descriptor types
static constexpr uint16_t MS_OS_20_SET_HEADER_DESCRIPTOR = 0x0000;
static constexpr uint16_t MS_OS_20_SUBSET_HEADER_CONFIGURATION = 0x0001;
static constexpr uint16_t MS_OS_20_SUBSET_HEADER_FUNCTION = 0x0002;
static constexpr uint16_t MS_OS_20_FEATURE_COMPATIBLE_ID = 0x0003;
static constexpr uint16_t MS_OS_20_FEATURE_REG_PROPERTY = 0x0004;

// Registry property data type
static constexpr uint16_t REG_MULTI_SZ = 0x0007;

// MS OS 2.0 platform capability UUID: D8DD60DF-4589-4CC7-9CD2-659D9E648A9F
static constexpr uint8_t MSOS20_PLATFORM_CAPABILITY_UUID[16] = {
    0xDF, 0x60, 0xDD, 0xD8, 0x89, 0x45, 0xC7, 0x4C,
    0x9C, 0xD2, 0x65, 0x9D, 0x9E, 0x64, 0x8A, 0x9F,
};

// "{xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx}" + UTF-16 NUL
static constexpr uint16_t GUID_CHARS_WITH_BRACES = 38;
static constexpr uint16_t GUID_STR_UTF16_BYTES = (GUID_CHARS_WITH_BRACES + 1) * 2;  // 78
static constexpr uint16_t GUID_MULTI_SZ_UTF16_BYTES =
    static_cast<uint16_t>(GUID_STR_UTF16_BYTES + 2u);

// "DeviceInterfaceGUIDs" (UTF-16LE) + NUL terminator
static constexpr uint8_t PROP_NAME_DEVICE_INTERFACE_GUIDS_UTF16[] = {
    'D', 0x00, 'e', 0x00, 'v', 0x00, 'i', 0x00, 'c', 0x00, 'e', 0x00, 'I',  0x00,
    'n', 0x00, 't', 0x00, 'e', 0x00, 'r', 0x00, 'f', 0x00, 'a', 0x00, 'c',  0x00,
    'e', 0x00, 'G', 0x00, 'U', 0x00, 'I', 0x00, 'D', 0x00, 's', 0x00, 0x00, 0x00};
static constexpr uint16_t PROP_NAME_DEVICE_INTERFACE_GUIDS_BYTES =
    static_cast<uint16_t>(sizeof(PROP_NAME_DEVICE_INTERFACE_GUIDS_UTF16));

LIBXR_PACKED_BEGIN

// ---- MS OS 2.0 basic blocks ----

// 9.22.1 MS OS 2.0 descriptor set header
struct MsOs20SetHeader
{
  uint16_t wLength = 0x000A;
  uint16_t wDescriptorType = MS_OS_20_SET_HEADER_DESCRIPTOR;
  uint32_t dwWindowsVersion = 0x06030000;  // 0x06030000 = Win 8.1
  uint16_t wTotalLength = 0;               // sizeof(whole set)
};

// 9.22.2 Configuration subset header
struct MsOs20SubsetHeaderConfiguration
{
  uint16_t wLength = 0x0008;
  uint16_t wDescriptorType = MS_OS_20_SUBSET_HEADER_CONFIGURATION;
  // MS OS 2.0 uses a zero-based configuration index here. This is not the
  // USB configuration descriptor's bConfigurationValue, which normally starts at 1.
  uint8_t bConfigurationValue = 0;
  uint8_t bReserved = 0;
  uint16_t wTotalLength = 0;  // sizeof(this cfg subset)
};

// 9.22.3 Function subset header
struct MsOs20SubsetHeaderFunction
{
  uint16_t wLength = 0x0008;
  uint16_t wDescriptorType = MS_OS_20_SUBSET_HEADER_FUNCTION;
  uint8_t bFirstInterface = 0;
  uint8_t bReserved = 0;
  uint16_t wTotalLength = 0;  // sizeof(this function subset)
};

// 9.22.4 Compatible ID descriptor
struct MsOs20FeatureCompatibleId
{
  uint16_t wLength = 0x0014;
  uint16_t wDescriptorType = MS_OS_20_FEATURE_COMPATIBLE_ID;

  // "WINUSB" + padding NULs
  uint8_t CompatibleID[8] = {'W', 'I', 'N', 'U', 'S', 'B', 0x00, 0x00};

  // Optional; usually all zeros
  uint8_t SubCompatibleID[8] = {0};
};

// 9.22.5 Registry property descriptor header (common part)
struct MsOs20FeatureRegPropertyHeader
{
  uint16_t wLength = 0;  // sizeof(whole reg property feature)
  uint16_t wDescriptorType = MS_OS_20_FEATURE_REG_PROPERTY;
  uint16_t wPropertyDataType = 0;  // e.g. REG_MULTI_SZ
  uint16_t wPropertyNameLength = 0;
};

// ---- MS OS 2.0 Platform Capability (BOS Device Capability) ----
//
// Exposed via BOS Platform Capability. Host can query the MS OS 2.0 descriptor
// set using a vendor request with matching bMS_VendorCode and wIndex=0x0007.
struct MsOs20PlatformCapability
{
  uint8_t bLength = 0x1C;
  uint8_t bDescriptorType = 0x10;     // DEVICE_CAPABILITY
  uint8_t bDevCapabilityType = 0x05;  // PLATFORM
  uint8_t bReserved = 0x00;

  uint8_t PlatformCapabilityUUID[16] = {
      0xDF, 0x60, 0xDD, 0xD8, 0x89, 0x45, 0xC7, 0x4C,
      0x9C, 0xD2, 0x65, 0x9D, 0x9E, 0x64, 0x8A, 0x9F,
  };

  uint32_t dwWindowsVersion = 0x06030000;      // Win 8.1+
  uint16_t wMSOSDescriptorSetTotalLength = 0;  // sizeof(MS OS 2.0 set)
  uint8_t bMS_VendorCode = 0x20;               // vendor request bRequest
  uint8_t bAltEnumCode = 0x00;                 // optional, often 0
};

template <uint16_t PropertyDataBytes>
struct DeviceInterfaceGuidsRegProperty
{
  static_assert(PropertyDataBytes >= 4u,
                "REG_MULTI_SZ property data must fit the double-NUL terminator");

  MsOs20FeatureRegPropertyHeader header;
  uint8_t name[PROP_NAME_DEVICE_INTERFACE_GUIDS_BYTES];
  uint16_t wPropertyDataLength = 0;
  uint8_t data[PropertyDataBytes] = {};

  void Init(const char* guid)
  {
    *this = DeviceInterfaceGuidsRegProperty<PropertyDataBytes>{};
    header.wDescriptorType = MS_OS_20_FEATURE_REG_PROPERTY;
    header.wPropertyDataType = REG_MULTI_SZ;
    header.wPropertyNameLength = PROP_NAME_DEVICE_INTERFACE_GUIDS_BYTES;
    Memory::FastCopy(name, PROP_NAME_DEVICE_INTERFACE_GUIDS_UTF16,
                     PROP_NAME_DEVICE_INTERFACE_GUIDS_BYTES);

    const size_t max_guid_chars =
        (PropertyDataBytes >= 4u) ? ((PropertyDataBytes - 4u) / 2u) : 0u;
    size_t guid_len = 0u;
    if (guid != nullptr)
    {
      while (guid[guid_len] != '\0' && guid_len < GUID_CHARS_WITH_BRACES &&
             guid_len < max_guid_chars)
      {
        data[guid_len * 2u] = static_cast<uint8_t>(guid[guid_len]);
        data[guid_len * 2u + 1u] = 0x00u;
        ++guid_len;
      }
    }

    if (PropertyDataBytes >= 4u)
    {
      data[guid_len * 2u] = 0x00u;
      data[guid_len * 2u + 1u] = 0x00u;
      data[guid_len * 2u + 2u] = 0x00u;
      data[guid_len * 2u + 3u] = 0x00u;
    }

    wPropertyDataLength = static_cast<uint16_t>((guid_len * 2u) + 4u);
    header.wLength = static_cast<uint16_t>(sizeof(*this));
  }
};

template <uint16_t PropertyDataBytes>
struct DeviceScopedWinUsbMsOs20DescSet
{
  MsOs20SetHeader set;
  MsOs20FeatureCompatibleId compat;
  DeviceInterfaceGuidsRegProperty<PropertyDataBytes> prop;

  void Init(const char* guid, uint32_t windows_version = 0x06030000)
  {
    *this = DeviceScopedWinUsbMsOs20DescSet<PropertyDataBytes>{};

    set.wLength = static_cast<uint16_t>(sizeof(MsOs20SetHeader));
    set.wDescriptorType = MS_OS_20_SET_HEADER_DESCRIPTOR;
    set.dwWindowsVersion = windows_version;
    set.wTotalLength = static_cast<uint16_t>(sizeof(*this));

    compat.wLength = static_cast<uint16_t>(sizeof(MsOs20FeatureCompatibleId));
    compat.wDescriptorType = MS_OS_20_FEATURE_COMPATIBLE_ID;

    prop.Init(guid);
  }
};

template <uint16_t PropertyDataBytes>
struct FunctionScopedWinUsbMsOs20DescSet
{
  MsOs20SetHeader set;
  MsOs20SubsetHeaderConfiguration cfg;
  MsOs20SubsetHeaderFunction func;
  MsOs20FeatureCompatibleId compat;
  DeviceInterfaceGuidsRegProperty<PropertyDataBytes> prop;

  void Init(uint8_t configuration_index, uint8_t first_interface, const char* guid,
            uint32_t windows_version = 0x06030000)
  {
    *this = FunctionScopedWinUsbMsOs20DescSet<PropertyDataBytes>{};

    set.wLength = static_cast<uint16_t>(sizeof(MsOs20SetHeader));
    set.wDescriptorType = MS_OS_20_SET_HEADER_DESCRIPTOR;
    set.dwWindowsVersion = windows_version;
    set.wTotalLength = static_cast<uint16_t>(sizeof(*this));

    cfg.wLength = static_cast<uint16_t>(sizeof(cfg));
    cfg.wDescriptorType = MS_OS_20_SUBSET_HEADER_CONFIGURATION;
    cfg.bConfigurationValue = configuration_index;
    cfg.bReserved = 0u;
    cfg.wTotalLength = static_cast<uint16_t>(
        sizeof(*this) -
        offsetof(FunctionScopedWinUsbMsOs20DescSet<PropertyDataBytes>, cfg));

    func.wLength = static_cast<uint16_t>(sizeof(func));
    func.wDescriptorType = MS_OS_20_SUBSET_HEADER_FUNCTION;
    func.bFirstInterface = first_interface;
    func.bReserved = 0u;
    func.wTotalLength = static_cast<uint16_t>(
        sizeof(*this) -
        offsetof(FunctionScopedWinUsbMsOs20DescSet<PropertyDataBytes>, func));

    compat.wLength = static_cast<uint16_t>(sizeof(MsOs20FeatureCompatibleId));
    compat.wDescriptorType = MS_OS_20_FEATURE_COMPATIBLE_ID;

    prop.Init(guid);
  }

  void SetFirstInterface(uint8_t first_interface)
  {
    func.bFirstInterface = first_interface;
  }
};

LIBXR_PACKED_END

// ---- sanity checks ----
static_assert(sizeof(MsOs20SetHeader) == 10, "SetHeader size mismatch");
static_assert(sizeof(MsOs20SubsetHeaderConfiguration) == 8, "CfgHeader size mismatch");
static_assert(sizeof(MsOs20SubsetHeaderFunction) == 8, "FuncHeader size mismatch");
static_assert(sizeof(MsOs20FeatureCompatibleId) == 20, "CompatibleId size mismatch");
static_assert(sizeof(MsOs20PlatformCapability) == 28, "PlatformCapability size mismatch");
static_assert(sizeof(DeviceScopedWinUsbMsOs20DescSet<GUID_MULTI_SZ_UTF16_BYTES>) ==
                  0x00A2,
              "Device-scoped WinUSB MS OS 2.0 set size mismatch");
static_assert(sizeof(FunctionScopedWinUsbMsOs20DescSet<GUID_MULTI_SZ_UTF16_BYTES>) ==
                  0x00B2,
              "Function-scoped WinUSB MS OS 2.0 set size mismatch");

// ---- descriptor set of a composition ----

/// One WinUSB function of a composition: where it sits and the interface GUID Windows
/// registers for it.
struct WinUsbFunctionInfo
{
  uint8_t configuration_index = 0u;  ///< zero-based configuration index
  uint8_t first_interface = 0u;      ///< first interface number of the function
  const char* guid = nullptr;        ///< DeviceInterfaceGUID, "{...}"
};

/// The MS OS 2.0 descriptor set of a composition, built once from the functions its
/// classes declare. A single function on a non-composite device gets the device-level
/// form (Windows ignores subset headers on such a device); every other case gets one
/// configuration subset per configuration with one function subset per function.
class WinUsbDescriptorSet
{
 public:
  /// @param functions Functions in configuration order
  /// @param device_level Device-level form; needs exactly one function
  void Build(const WinUsbFunctionInfo* functions, size_t count, bool device_level)
  {
    using Property = DeviceInterfaceGuidsRegProperty<GUID_MULTI_SZ_UTF16_BYTES>;
    data_ = nullptr;
    size_ = 0u;
    if (functions == nullptr || count == 0u) return;
    ASSERT(!device_level || count == 1u);

    constexpr size_t FEATURES = sizeof(MsOs20FeatureCompatibleId) + sizeof(Property);
    size_t total = sizeof(MsOs20SetHeader);
    if (device_level)
    {
      total += FEATURES;
    }
    else
    {
      for (size_t i = 0u; i < count; ++i)
      {
        if (i == 0u ||
            functions[i].configuration_index != functions[i - 1u].configuration_index)
        {
          total += sizeof(MsOs20SubsetHeaderConfiguration);
        }
        total += sizeof(MsOs20SubsetHeaderFunction) + FEATURES;
      }
    }

    auto* out = new uint8_t[total];
    size_t off = 0u;
    auto put = [&](const void* source, size_t length)
    {
      Memory::FastCopy(out + off, source, length);
      off += length;
    };
    auto patch_total_length = [&](size_t header_offset, size_t header_size)
    {
      const uint16_t length = static_cast<uint16_t>(off - header_offset);
      Memory::FastCopy(out + header_offset + header_size - sizeof(uint16_t), &length,
                       sizeof(length));
    };

    MsOs20SetHeader set;
    set.wLength = static_cast<uint16_t>(sizeof(set));
    set.wTotalLength = static_cast<uint16_t>(total);
    put(&set, sizeof(set));

    MsOs20FeatureCompatibleId compat;
    Property property;
    if (device_level)
    {
      property.Init(functions[0].guid);
      put(&compat, sizeof(compat));
      put(&property, sizeof(property));
    }
    else
    {
      size_t configuration_offset = 0u;
      for (size_t i = 0u; i < count; ++i)
      {
        if (i == 0u ||
            functions[i].configuration_index != functions[i - 1u].configuration_index)
        {
          if (i != 0u)
            patch_total_length(configuration_offset,
                               sizeof(MsOs20SubsetHeaderConfiguration));
          configuration_offset = off;
          MsOs20SubsetHeaderConfiguration configuration;
          configuration.bConfigurationValue = functions[i].configuration_index;
          put(&configuration, sizeof(configuration));
        }
        const size_t function_offset = off;
        MsOs20SubsetHeaderFunction function;
        function.bFirstInterface = functions[i].first_interface;
        put(&function, sizeof(function));
        property.Init(functions[i].guid);
        put(&compat, sizeof(compat));
        put(&property, sizeof(property));
        patch_total_length(function_offset, sizeof(MsOs20SubsetHeaderFunction));
      }
      patch_total_length(configuration_offset, sizeof(MsOs20SubsetHeaderConfiguration));
    }
    ASSERT(off == total);
    data_ = out;
    size_ = total;
  }

  LibXR::ConstRawData Data() const { return LibXR::ConstRawData{data_, size_}; }

 private:
  const uint8_t* data_ = nullptr;
  size_t size_ = 0u;
};

// ---- platform capability helper ----

inline void init_msos20_platform_capability(MsOs20PlatformCapability& cap,
                                            uint16_t msos_descriptor_set_total_length,
                                            uint8_t vendor_code = 0x20,
                                            uint32_t windows_version = 0x06030000)
{
  cap = MsOs20PlatformCapability{};
  Memory::FastCopy(cap.PlatformCapabilityUUID, MSOS20_PLATFORM_CAPABILITY_UUID,
                   sizeof(MSOS20_PLATFORM_CAPABILITY_UUID));
  cap.dwWindowsVersion = windows_version;
  cap.wMSOSDescriptorSetTotalLength = msos_descriptor_set_total_length;
  cap.bMS_VendorCode = vendor_code;
  cap.bAltEnumCode = 0x00;
}

// ---- BOS capability wrapper ----
//
// Purpose:
// - Register a BOS Platform Capability (MS OS 2.0) to BosManager.
// - Handle vendor requests:
//   - wIndex=0x0007 (IN): return MS OS 2.0 descriptor set.
//   - wIndex=0x0008 (OUT): Set Alt Enumeration (usually no data stage).
class MsOs20BosCapability final : public LibXR::USB::BosCapability
{
 public:
  explicit MsOs20BosCapability(LibXR::ConstRawData descriptor_set,
                               uint8_t vendor_code = 0x20,
                               uint32_t windows_version = 0x06030000)
      : descriptor_set_(descriptor_set),
        vendor_code_(vendor_code),
        windows_version_(windows_version)
  {
    RefreshPlatformCap();
  }

  void SetDescriptorSet(LibXR::ConstRawData descriptor_set)
  {
    descriptor_set_ = descriptor_set;
    RefreshPlatformCap();
  }

  void SetVendorCode(uint8_t vendor_code)
  {
    vendor_code_ = vendor_code;
    platform_cap_.bMS_VendorCode = vendor_code_;
  }

  LibXR::ConstRawData GetCapabilityDescriptor() const override
  {
    return LibXR::ConstRawData(&platform_cap_, sizeof(platform_cap_));
  }

  ErrorCode OnVendorRequest(bool /*in_isr*/, const SetupPacket* setup,
                            LibXR::USB::BosVendorResult& result) override
  {
    if (setup == nullptr)
    {
      return ErrorCode::ARG_ERR;
    }

    const uint8_t BM = setup->bmRequestType;

    // type must be Vendor (bits[6:5] == 10)
    if ((BM & 0x60) != 0x40)
    {
      return ErrorCode::NOT_SUPPORT;
    }

    // recipient must be Device (bits[4:0] == 00000)
    if ((BM & 0x1F) != 0x00)
    {
      return ErrorCode::NOT_SUPPORT;
    }

    // vendor code must match
    if (setup->bRequest != vendor_code_)
    {
      return ErrorCode::NOT_SUPPORT;
    }

    // wIndex == 0x0007: MS OS 2.0 descriptor set (Vendor IN)
    if (setup->wIndex == MSOS20_DESCRIPTOR_INDEX)
    {
      // direction must be IN
      if ((BM & 0x80) == 0)
      {
        return ErrorCode::NOT_SUPPORT;
      }

      if (descriptor_set_.addr_ == nullptr || descriptor_set_.size_ == 0 ||
          descriptor_set_.size_ > 0xFFFF)
      {
        return ErrorCode::ARG_ERR;
      }

      // Host wLength==0 is meaningless here; reject.
      if (setup->wLength == 0)
      {
        return ErrorCode::NOT_SUPPORT;
      }

      result.handled = true;
      result.in_data = descriptor_set_;  // USB stack truncates by wLength
      result.write_zlp = false;
      result.early_read_zlp = true;
      return ErrorCode::OK;
    }

    // wIndex == 0x0008: Set Alt Enumeration (Vendor OUT, usually wLength==0)
    if (setup->wIndex == MSOS20_SET_ALT_ENUMERATION)
    {
      // direction should be OUT
      if ((BM & 0x80) != 0)
      {
        // Strict behavior: reject IN.
        return ErrorCode::NOT_SUPPORT;
      }

      // Usually no data stage. If wLength != 0, we still ACK to keep flow smooth.
      if (setup->wLength != 0)
      {
        // no-op
      }

      result.handled = true;
      result.in_data = {nullptr, 0};
      result.write_zlp = true;  // complete status stage
      result.early_read_zlp = true;
      return ErrorCode::OK;
    }

    return ErrorCode::NOT_SUPPORT;
  }

 private:
  void RefreshPlatformCap()
  {
    const uint16_t total_length = (descriptor_set_.size_ <= 0xFFFF)
                                      ? static_cast<uint16_t>(descriptor_set_.size_)
                                      : 0u;
    init_msos20_platform_capability(platform_cap_, total_length, vendor_code_,
                                    windows_version_);
  }

  // Cached MS OS 2.0 descriptor set bytes.
  LibXR::ConstRawData descriptor_set_{nullptr, 0};

  // Vendor request bRequest code used for MS OS 2.0 query.
  uint8_t vendor_code_ = 0x20;

  // Windows version field in Platform Capability.
  uint32_t windows_version_ = 0x06030000;

  // BOS Platform Capability descriptor block.
  MsOs20PlatformCapability platform_cap_{};
};

}  // namespace LibXR::USB::WinUsbMsOs20
