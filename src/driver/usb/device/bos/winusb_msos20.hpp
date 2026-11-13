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

// ---- 常量 ----
// ---- constants ----

// vendor 请求的 wIndex 取值 / wIndex values for vendor request
static constexpr uint16_t MSOS20_DESCRIPTOR_INDEX = 0x0007;
static constexpr uint16_t MSOS20_SET_ALT_ENUMERATION = 0x0008;

// MS OS 2.0 描述符类型 / MS OS 2.0 descriptor types
static constexpr uint16_t MS_OS_20_SET_HEADER_DESCRIPTOR = 0x0000;
static constexpr uint16_t MS_OS_20_SUBSET_HEADER_CONFIGURATION = 0x0001;
static constexpr uint16_t MS_OS_20_SUBSET_HEADER_FUNCTION = 0x0002;
static constexpr uint16_t MS_OS_20_FEATURE_COMPATIBLE_ID = 0x0003;
static constexpr uint16_t MS_OS_20_FEATURE_REG_PROPERTY = 0x0004;

// 注册表属性数据类型 / Registry property data type
static constexpr uint16_t REG_MULTI_SZ = 0x0007;

// MS OS 2.0 平台能力 UUID：D8DD60DF-4589-4CC7-9CD2-659D9E648A9F。
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

// "DeviceInterfaceGUIDs"（UTF-16LE）加 NUL 结束符。
// "DeviceInterfaceGUIDs" (UTF-16LE) + NUL terminator
static constexpr uint8_t PROP_NAME_DEVICE_INTERFACE_GUIDS_UTF16[] = {
    'D', 0x00, 'e', 0x00, 'v', 0x00, 'i', 0x00, 'c', 0x00, 'e', 0x00, 'I',  0x00,
    'n', 0x00, 't', 0x00, 'e', 0x00, 'r', 0x00, 'f', 0x00, 'a', 0x00, 'c',  0x00,
    'e', 0x00, 'G', 0x00, 'U', 0x00, 'I', 0x00, 'D', 0x00, 's', 0x00, 0x00, 0x00};
static constexpr uint16_t PROP_NAME_DEVICE_INTERFACE_GUIDS_BYTES =
    static_cast<uint16_t>(sizeof(PROP_NAME_DEVICE_INTERFACE_GUIDS_UTF16));

LIBXR_PACKED_BEGIN

// ---- MS OS 2.0 基本块 ----
// ---- MS OS 2.0 basic blocks ----

/**
 * @brief MS OS 2.0 描述符集头（规范 9.22.1）/ MS OS 2.0 descriptor set header
 *        (specification 9.22.1)
 */
struct MsOs20SetHeader
{
  uint16_t wLength = 0x000A;  ///< 本头长度，固定 10 / Length of this header, always 10
  uint16_t wDescriptorType =
      MS_OS_20_SET_HEADER_DESCRIPTOR;  ///< 描述符类型 / Descriptor type
  uint32_t dwWindowsVersion =
      0x06030000;  ///< 目标 Windows 版本，0x06030000 = Win 8.1 / Target Windows version
  uint16_t wTotalLength = 0;  ///< 整个描述符集的字节数 / Size of the whole set in bytes
};

/**
 * @brief 配置子集头（规范 9.22.2）/ Configuration subset header (specification 9.22.2)
 */
struct MsOs20SubsetHeaderConfiguration
{
  uint16_t wLength = 0x0008;  ///< 本头长度，固定 8 / Length of this header, always 8
  uint16_t wDescriptorType =
      MS_OS_20_SUBSET_HEADER_CONFIGURATION;  ///< 描述符类型 / Descriptor type
  /// 配置索引，从 0 起；不是 USB 配置描述符的 bConfigurationValue（通常从 1 起）/
  /// Zero-based configuration index; this is not the USB configuration descriptor's
  /// bConfigurationValue, which normally starts at 1.
  uint8_t bConfigurationValue = 0;
  uint8_t bReserved = 0;      ///< 保留，置 0 / Reserved, zero
  uint16_t wTotalLength = 0;  ///< 本配置子集的字节数 / Size of this configuration subset
};

/**
 * @brief 功能子集头（规范 9.22.3）/ Function subset header (specification 9.22.3)
 */
struct MsOs20SubsetHeaderFunction
{
  uint16_t wLength = 0x0008;  ///< 本头长度，固定 8 / Length of this header, always 8
  uint16_t wDescriptorType =
      MS_OS_20_SUBSET_HEADER_FUNCTION;  ///< 描述符类型 / Descriptor type
  uint8_t bFirstInterface =
      0;                  ///< 该功能的首个接口号 / First interface number of the function
  uint8_t bReserved = 0;  ///< 保留，置 0 / Reserved, zero
  uint16_t wTotalLength = 0;  ///< 本功能子集的字节数 / Size of this function subset
};

/**
 * @brief Compatible ID 描述符（规范 9.22.4）/ Compatible ID descriptor (specification
 * 9.22.4)
 */
struct MsOs20FeatureCompatibleId
{
  uint16_t wLength = 0x0014;  ///< 本描述符长度，固定 20 / Descriptor length, always 20
  uint16_t wDescriptorType =
      MS_OS_20_FEATURE_COMPATIBLE_ID;  ///< 描述符类型 / Descriptor type

  /// "WINUSB" 加填充 NUL / "WINUSB" followed by padding NULs
  uint8_t CompatibleID[8] = {'W', 'I', 'N', 'U', 'S', 'B', 0x00, 0x00};

  /// 可选，通常全 0 / Optional, usually all zeros
  uint8_t SubCompatibleID[8] = {0};
};

/**
 * @brief 注册表属性描述符的公共头（规范 9.22.5）/ Common header of the registry property
 * descriptor (specification 9.22.5)
 */
struct MsOs20FeatureRegPropertyHeader
{
  uint16_t wLength = 0;  ///< 整个注册表属性描述符的字节数 / Size of the whole registry
                         ///< property descriptor
  uint16_t wDescriptorType =
      MS_OS_20_FEATURE_REG_PROPERTY;  ///< 描述符类型 / Descriptor type
  uint16_t wPropertyDataType = 0;     ///< 属性数据类型，如 REG_MULTI_SZ / Property data
                                      ///< type, e.g. REG_MULTI_SZ
  uint16_t wPropertyNameLength = 0;   ///< 属性名的字节数（含 NUL）/ Size of the property
                                      ///< name in bytes, including the NUL
};

// ---- MS OS 2.0 平台能力（BOS 设备能力） ----
// ---- MS OS 2.0 Platform Capability (BOS Device Capability) ----

/**
 * @brief MS OS 2.0 BOS 平台能力描述符 / MS OS 2.0 BOS platform capability descriptor
 *
 * 通过 BOS 平台能力公布；主机用匹配 bMS_VendorCode、wIndex=0x0007 的 vendor 请求读取
 * MS OS 2.0 描述符集。
 * Exposed via the BOS platform capability. The host queries the MS OS 2.0 descriptor set
 * with a vendor request carrying the matching bMS_VendorCode and wIndex=0x0007.
 */
struct MsOs20PlatformCapability
{
  uint8_t bLength = 0x1C;          ///< 描述符长度，固定 28 / Descriptor length, always 28
  uint8_t bDescriptorType = 0x10;  ///< DEVICE_CAPABILITY
  uint8_t bDevCapabilityType = 0x05;  ///< PLATFORM
  uint8_t bReserved = 0x00;           ///< 保留，置 0 / Reserved, zero

  /// MS OS 2.0 平台能力 UUID / MS OS 2.0 platform capability UUID
  uint8_t PlatformCapabilityUUID[16] = {
      0xDF, 0x60, 0xDD, 0xD8, 0x89, 0x45, 0xC7, 0x4C,
      0x9C, 0xD2, 0x65, 0x9D, 0x9E, 0x64, 0x8A, 0x9F,
  };

  uint32_t dwWindowsVersion = 0x06030000;  ///< 最低 Windows 版本，Win 8.1+ / Minimum
                                           ///< Windows version, Win 8.1+
  uint16_t wMSOSDescriptorSetTotalLength =
      0;  ///< MS OS 2.0 描述符集的字节数 / Size of the MS OS 2.0 descriptor set in bytes
  uint8_t bMS_VendorCode = 0x20;  ///< 读取描述符集所用 vendor 请求的 bRequest / bRequest
                                  ///< of the vendor request that fetches the set
  uint8_t bAltEnumCode = 0x00;  ///< 备用枚举命令码，0 表示不支持 / Alternate enumeration
                                ///< code, 0 means not supported
};

/**
 * @brief DeviceInterfaceGUIDs 注册表属性（REG_MULTI_SZ）/ DeviceInterfaceGUIDs registry
 * property (REG_MULTI_SZ)
 * @tparam PropertyDataBytes 属性数据区字节数，至少 4（双 NUL 结尾）/ Size of the
 * property data area in bytes, at least 4 (double-NUL terminator)
 */
template <uint16_t PropertyDataBytes>
struct DeviceInterfaceGuidsRegProperty
{
  static_assert(PropertyDataBytes >= 4u,
                "REG_MULTI_SZ property data must fit the double-NUL terminator");

  MsOs20FeatureRegPropertyHeader header;  ///< 公共头 / Common header
  uint8_t
      name[PROP_NAME_DEVICE_INTERFACE_GUIDS_BYTES];  ///< 属性名 "DeviceInterfaceGUIDs"
                                                     ///< （UTF-16LE，含 NUL）/ Property
                                                     ///< name (UTF-16LE, with NUL)
  uint16_t wPropertyDataLength = 0;  ///< 属性数据的有效字节数 / Valid property data bytes
  uint8_t data[PropertyDataBytes] =
      {};  ///< 属性数据：GUID 的 UTF-16LE 加双 NUL / Property
           ///< data: the GUID in UTF-16LE plus a double NUL

  /**
   * @brief 填充属性 / Fill in the property
   * @param guid 形如 "{xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx}" 的 GUID 字符串；空指针得到
   * 空列表，超出 38 个字符或数据区容量的部分被截断 / GUID string such as
   * "{xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx}"; null yields an empty list, characters
   * beyond 38 or beyond the data area are truncated
   */
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

/**
 * @brief 设备级 WinUSB 描述符集：集头 + Compatible ID + GUID 属性 / Device-scoped WinUSB
 * descriptor set: set header, compatible ID and GUID property
 * @tparam PropertyDataBytes GUID 属性数据区字节数 / Size of the GUID property data area
 */
template <uint16_t PropertyDataBytes>
struct DeviceScopedWinUsbMsOs20DescSet
{
  MsOs20SetHeader set;               ///< 描述符集头 / Descriptor set header
  MsOs20FeatureCompatibleId compat;  ///< WINUSB Compatible ID
  DeviceInterfaceGuidsRegProperty<PropertyDataBytes>
      prop;  ///< 接口 GUID 属性 / Interface GUID property

  /**
   * @brief 填充整个描述符集 / Fill in the whole descriptor set
   * @param guid 设备接口 GUID 字符串 / Device interface GUID string
   * @param windows_version 写入集头的目标 Windows 版本，默认 Win 8.1 / Target Windows
   * version written into the set header, Win 8.1 by default
   */
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

/**
 * @brief 功能级 WinUSB 描述符集：集头 + 配置子集 + 功能子集 + Compatible ID + GUID 属性 /
 * Function-scoped WinUSB descriptor set: set header, configuration subset, function
 * subset, compatible ID and GUID property
 * @tparam PropertyDataBytes GUID 属性数据区字节数 / Size of the GUID property data area
 */
template <uint16_t PropertyDataBytes>
struct FunctionScopedWinUsbMsOs20DescSet
{
  MsOs20SetHeader set;                  ///< 描述符集头 / Descriptor set header
  MsOs20SubsetHeaderConfiguration cfg;  ///< 配置子集头 / Configuration subset header
  MsOs20SubsetHeaderFunction func;      ///< 功能子集头 / Function subset header
  MsOs20FeatureCompatibleId compat;     ///< WINUSB Compatible ID
  DeviceInterfaceGuidsRegProperty<PropertyDataBytes>
      prop;  ///< 接口 GUID 属性 / Interface GUID property

  /**
   * @brief 填充整个描述符集，子集长度按各自到集尾的距离计算 / Fill in the whole set; the
   * subset lengths are the distance from each subset header to the end of the set
   * @param configuration_index 从 0 起的配置索引 / Zero-based configuration index
   * @param first_interface 功能的首个接口号 / First interface number of the function
   * @param guid 设备接口 GUID 字符串 / Device interface GUID string
   * @param windows_version 写入集头的目标 Windows 版本，默认 Win 8.1 / Target Windows
   * version written into the set header, Win 8.1 by default
   */
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

  /**
   * @brief 修改功能子集头中的首个接口号 / Change the first interface number in the
   * function subset header
   * @param first_interface 新的首个接口号 / New first interface number
   */
  void SetFirstInterface(uint8_t first_interface)
  {
    func.bFirstInterface = first_interface;
  }
};

LIBXR_PACKED_END

// ---- 合理性检查 ----
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

// ---- 组合设备的描述符集 ----
// ---- descriptor set of a composition ----

/// 组合设备中的一个 WinUSB 功能：其所在位置，以及 Windows 为它注册的接口 GUID。
/// One WinUSB function of a composition: where it sits and the interface GUID Windows
/// registers for it.
struct WinUsbFunctionInfo
{
  /// 从 0 开始的配置索引 / zero-based configuration index
  uint8_t configuration_index = 0u;
  /// 该功能的第一个接口号 / first interface number of the function
  uint8_t first_interface = 0u;
  const char* guid = nullptr;  ///< DeviceInterfaceGUID, "{...}"
};

/// 组合设备的 MS OS 2.0 描述符集，由各类声明的功能一次性构建。
/// 非复合设备上只有一个功能时使用设备级形式（此类设备上 Windows 会忽略子集头）；
/// 其余情况下每个配置一个配置子集，每个功能一个功能子集。
/// The MS OS 2.0 descriptor set of a composition, built once from the functions its
/// classes declare. A single function on a non-composite device gets the device-level
/// form (Windows ignores subset headers on such a device); every other case gets one
/// configuration subset per configuration with one function subset per function.
class WinUsbDescriptorSet
{
 public:
  /// @param functions 按配置顺序排列的功能 / Functions in configuration order
  /// @param count functions 的条目数；0（或空指针）使集合为空 / Number of entries in
  /// functions; 0 (or a null pointer) leaves the set empty
  /// @param device_level 设备级形式；必须恰有一个功能 / Device-level form; needs exactly
  /// one function
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

  /// 构建好的描述符集；Build() 产生之前为空。内存由 Build() 分配，
  /// 在程序生命周期内保持有效。
  /// The built descriptor set; empty until Build() has produced one. The memory is
  /// allocated by Build() and stays valid for the life of the program.
  LibXR::ConstRawData Data() const { return LibXR::ConstRawData{data_, size_}; }

 private:
  const uint8_t* data_ = nullptr;
  size_t size_ = 0u;
};

// ---- 平台能力辅助 ----
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

/**
 * @brief MS OS 2.0 BOS 平台能力包装器 / MS OS 2.0 BOS platform capability wrapper
 *
 * 向 BosManager 登记 MS OS 2.0 平台能力，并处理 vendor 请求：wIndex=0x0007（IN）返回
 * MS OS 2.0 描述符集，wIndex=0x0008（OUT）为 Set Alt Enumeration（通常无数据阶段）。
 * Registers the MS OS 2.0 platform capability with the BosManager and handles the vendor
 * requests: wIndex=0x0007 (IN) returns the MS OS 2.0 descriptor set, wIndex=0x0008 (OUT)
 * is Set Alt Enumeration (usually without a data stage).
 */
class MsOs20BosCapability final : public LibXR::USB::BosCapability
{
 public:
  /**
   * @brief 构造能力对象 / Construct the capability
   * @param descriptor_set 描述符集字节，须保持有效；大于 0xFFFF 字节时能力里的总长度为 0
   * / Descriptor set bytes, must stay valid; above 0xFFFF bytes the total length in the
   * capability is 0
   * @param vendor_code 读取描述符集所用 vendor 请求的 bRequest / bRequest of the vendor
   * request that fetches the set
   * @param windows_version 目标 Windows 版本，默认 Win 8.1 / Target Windows version, Win
   * 8.1 by default
   */
  explicit MsOs20BosCapability(LibXR::ConstRawData descriptor_set,
                               uint8_t vendor_code = 0x20,
                               uint32_t windows_version = 0x06030000)
      : descriptor_set_(descriptor_set),
        vendor_code_(vendor_code),
        windows_version_(windows_version)
  {
    RefreshPlatformCap();
  }

  /**
   * @brief 更换描述符集并刷新能力里的总长度 / Replace the descriptor set and refresh the
   * total length in the capability
   * @param descriptor_set 新的描述符集字节，须保持有效 / New descriptor set bytes, must
   * stay valid
   */
  void SetDescriptorSet(LibXR::ConstRawData descriptor_set)
  {
    descriptor_set_ = descriptor_set;
    RefreshPlatformCap();
  }

  /**
   * @brief 修改 vendor 请求码并同步到能力描述符 / Change the vendor request code and
   * mirror it into the capability descriptor
   * @param vendor_code 新的 bRequest / New bRequest
   */
  void SetVendorCode(uint8_t vendor_code)
  {
    vendor_code_ = vendor_code;
    platform_cap_.bMS_VendorCode = vendor_code_;
  }

  LibXR::ConstRawData GetCapabilityDescriptor() const override
  {
    return LibXR::ConstRawData(&platform_cap_, sizeof(platform_cap_));
  }

  /**
   * @brief 处理 MS OS 2.0 的 vendor 请求 / Handle the MS OS 2.0 vendor requests
   *
   * 请求必须是 Vendor 类型、Device 接收者，bRequest 等于 vendor 码；否则返回
   * NOT_SUPPORT。wIndex=0x0007 的 IN 请求返回描述符集，wIndex=0x0008 的 OUT 请求只回
   * 状态阶段。空 setup 返回 ARG_ERR，描述符集为空或超过 0xFFFF 字节时返回 ARG_ERR。
   * The request must be of Vendor type with a Device recipient and bRequest equal to the
   * vendor code, otherwise NOT_SUPPORT is returned. An IN request with wIndex=0x0007
   * returns the descriptor set, an OUT request with wIndex=0x0008 only completes the
   * status stage. A null setup yields ARG_ERR, as does an empty descriptor set or one
   * larger than 0xFFFF bytes.
   */
  ErrorCode OnVendorRequest(bool /*in_isr*/, const SetupPacket* setup,
                            LibXR::USB::BosVendorResult& result) override
  {
    if (setup == nullptr)
    {
      return ErrorCode::ARG_ERR;
    }

    const uint8_t BM = setup->bmRequestType;

    // 类型必须为 Vendor（bits[6:5] == 10） / type must be Vendor (bits[6:5] == 10)
    if ((BM & 0x60) != 0x40)
    {
      return ErrorCode::NOT_SUPPORT;
    }

    // 接收方必须为 Device（bits[4:0] == 00000）。
    // recipient must be Device (bits[4:0] == 00000)
    if ((BM & 0x1F) != 0x00)
    {
      return ErrorCode::NOT_SUPPORT;
    }

    // vendor 码必须匹配 / vendor code must match
    if (setup->bRequest != vendor_code_)
    {
      return ErrorCode::NOT_SUPPORT;
    }

    // wIndex == 0x0007：MS OS 2.0 描述符集（Vendor IN）。
    // wIndex == 0x0007: MS OS 2.0 descriptor set (Vendor IN)
    if (setup->wIndex == MSOS20_DESCRIPTOR_INDEX)
    {
      // 方向必须为 IN / direction must be IN
      if ((BM & 0x80) == 0)
      {
        return ErrorCode::NOT_SUPPORT;
      }

      if (descriptor_set_.addr_ == nullptr || descriptor_set_.size_ == 0 ||
          descriptor_set_.size_ > 0xFFFF)
      {
        return ErrorCode::ARG_ERR;
      }

      // 主机的 wLength==0 在此没有意义，拒绝。
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

    // wIndex == 0x0008：Set Alt Enumeration（Vendor OUT，通常 wLength==0）。
    // wIndex == 0x0008: Set Alt Enumeration (Vendor OUT, usually wLength==0)
    if (setup->wIndex == MSOS20_SET_ALT_ENUMERATION)
    {
      // 方向应为 OUT / direction should be OUT
      if ((BM & 0x80) != 0)
      {
        // 严格行为：拒绝 IN / Strict behavior: reject IN.
        return ErrorCode::NOT_SUPPORT;
      }

      // 通常没有数据阶段。若 wLength != 0，仍然 ACK 以保持流程顺畅。
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

  // 缓存的 MS OS 2.0 描述符集字节 / Cached MS OS 2.0 descriptor set bytes.
  LibXR::ConstRawData descriptor_set_{nullptr, 0};

  // 用于 MS OS 2.0 查询的 vendor 请求 bRequest 码。
  // Vendor request bRequest code used for MS OS 2.0 query.
  uint8_t vendor_code_ = 0x20;

  // 平台能力中的 Windows 版本字段 / Windows version field in Platform Capability.
  uint32_t windows_version_ = 0x06030000;

  // BOS 平台能力描述符块 / BOS Platform Capability descriptor block.
  MsOs20PlatformCapability platform_cap_{};
};

}  // namespace LibXR::USB::WinUsbMsOs20
