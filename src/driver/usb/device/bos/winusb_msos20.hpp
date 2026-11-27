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

/**
 * @brief MS OS 2.0 描述符集头部（9.22.1） / MS OS 2.0 descriptor set header (9.22.1).
 */
struct MsOs20SetHeader
{
  uint16_t wLength = 0x000A;  ///< 结构体长度（10 字节） / Header length (10 bytes).
  uint16_t wDescriptorType =
      MS_OS_20_SET_HEADER_DESCRIPTOR;      ///< 描述符类型 / Descriptor type.
  uint32_t dwWindowsVersion = 0x06030000;  ///< 最低 Windows 版本（0x06030000 = Win 8.1）
                                           ///< / Minimum Windows version.
  uint16_t wTotalLength =
      0;  ///< 整个描述符集总长度 / Total length of the descriptor set.
};

/**
 * @brief MS OS 2.0 配置子集头（9.22.2） / MS OS 2.0 configuration subset header
 *        (9.22.2).
 */
struct MsOs20SubsetHeaderConfiguration
{
  uint16_t wLength = 0x0008;  ///< 结构体长度（8 字节） / Header length (8 bytes).
  uint16_t wDescriptorType =
      MS_OS_20_SUBSET_HEADER_CONFIGURATION;  ///< 描述符类型 / Descriptor type.
  // MS OS 2.0 uses a zero-based configuration index here. This is not the
  // USB configuration descriptor's bConfigurationValue, which normally starts at 1.
  uint8_t bConfigurationValue =
      0;                  ///< 从 0 开始的配置索引 / Zero-based configuration index.
  uint8_t bReserved = 0;  ///< 保留字节（0） / Reserved byte (0).
  uint16_t wTotalLength =
      0;  ///< 本配置子集总长度 / Total length of this configuration subset.
};

/**
 * @brief MS OS 2.0 功能接口子集头（9.22.3） / MS OS 2.0 function subset header
 *        (9.22.3).
 */
struct MsOs20SubsetHeaderFunction
{
  uint16_t wLength = 0x0008;  ///< 结构体长度（8 字节） / Header length (8 bytes).
  uint16_t wDescriptorType =
      MS_OS_20_SUBSET_HEADER_FUNCTION;  ///< 描述符类型 / Descriptor type.
  uint8_t bFirstInterface =
      0;                  ///< 功能首个接口号 / First interface index of the function.
  uint8_t bReserved = 0;  ///< 保留字节（0） / Reserved byte (0).
  uint16_t wTotalLength =
      0;  ///< 本功能子集总长度 / Total length of this function subset.
};

/**
 * @brief MS OS 2.0 兼容 ID 特性描述符（9.22.4） / MS OS 2.0 compatible ID feature
 *        descriptor (9.22.4).
 */
struct MsOs20FeatureCompatibleId
{
  uint16_t wLength = 0x0014;  ///< 结构体长度（20 字节） / Descriptor length (20 bytes).
  uint16_t wDescriptorType =
      MS_OS_20_FEATURE_COMPATIBLE_ID;  ///< 描述符类型 / Descriptor type.

  /// 兼容 ID ASCII 串（"WINUSB" 补零对齐） / Compatible ID ("WINUSB" + trailing NULs).
  uint8_t CompatibleID[8] = {'W', 'I', 'N', 'U', 'S', 'B', 0x00, 0x00};

  /// 可选子兼容 ID（通常全 0） / Optional sub-compatible ID (usually all zeros).
  uint8_t SubCompatibleID[8] = {0};
};

/**
 * @brief MS OS 2.0 注册表属性特性描述符固定头（9.22.5） / MS OS 2.0 registry property
 *        feature descriptor header (9.22.5).
 */
struct MsOs20FeatureRegPropertyHeader
{
  uint16_t wLength = 0;  ///< 完整注册表属性描述符长度 / Total descriptor length.
  uint16_t wDescriptorType =
      MS_OS_20_FEATURE_REG_PROPERTY;  ///< 描述符类型 / Descriptor type.
  uint16_t wPropertyDataType =
      0;  ///< 注册表值类型（如 REG_MULTI_SZ） / Registry property data type.
  uint16_t wPropertyNameLength =
      0;  ///< 属性名 UTF-16LE 字节长度 / Property name length in bytes.
};

/**
 * @brief MS OS 2.0 平台能力 BOS 子描述符 / MS OS 2.0 BOS platform capability
 *        descriptor.
 *
 * Host can query the MS OS 2.0 descriptor set using a vendor request with
 * matching `bMS_VendorCode` and `wIndex=0x0007`.
 */
struct MsOs20PlatformCapability
{
  uint8_t bLength =
      0x1C;  ///< 能力描述符长度（28 字节） / Capability descriptor length (28 bytes).
  uint8_t bDescriptorType =
      0x10;  ///< 描述符类型（DEVICE_CAPABILITY） / Descriptor type (DEVICE_CAPABILITY).
  uint8_t bDevCapabilityType =
      0x05;  ///< 设备能力类型（PLATFORM） / Device capability type (PLATFORM).
  uint8_t bReserved = 0x00;  ///< 保留字段（0） / Reserved byte (0).

  /// MS OS 2.0 平台能力 UUID / MS OS 2.0 platform capability UUID.
  uint8_t PlatformCapabilityUUID[16] = {
      0xDF, 0x60, 0xDD, 0xD8, 0x89, 0x45, 0xC7, 0x4C,
      0x9C, 0xD2, 0x65, 0x9D, 0x9E, 0x64, 0x8A, 0x9F,
  };

  uint32_t dwWindowsVersion =
      0x06030000;  ///< 目标最低 Windows 版本 / Minimum Windows version (Win 8.1+).
  uint16_t wMSOSDescriptorSetTotalLength =
      0;  ///< MS OS 2.0 描述符集总字节数 / Total MS OS 2.0 descriptor set length.
  uint8_t bMS_VendorCode =
      0x20;  ///< Vendor 请求 `bRequest` 码 / Vendor request `bRequest` code.
  uint8_t bAltEnumCode =
      0x00;  ///< 可选备用枚举码（常为 0） / Optional alternate enumeration code.
};

/**
 * @brief `DeviceInterfaceGUIDs`（`REG_MULTI_SZ`）注册表属性块 / `DeviceInterfaceGUIDs`
 *        (`REG_MULTI_SZ`) registry property descriptor block.
 * @tparam PropertyDataBytes GUID 属性数据缓冲区字节数 / Property data buffer size in
 * bytes.
 */
template <uint16_t PropertyDataBytes>
struct DeviceInterfaceGuidsRegProperty
{
  static_assert(PropertyDataBytes >= 4u,
                "REG_MULTI_SZ property data must fit the double-NUL terminator");

  MsOs20FeatureRegPropertyHeader header;  ///< 属性描述符头 / Property descriptor header.
  uint8_t name[PROP_NAME_DEVICE_INTERFACE_GUIDS_BYTES];  ///< UTF-16LE 属性名
                                                         ///< `"DeviceInterfaceGUIDs"` /
                                                         ///< UTF-16LE property name.
  uint16_t wPropertyDataLength =
      0;  ///< 编码后 GUID 数据长度（字节） / Encoded GUID data length in bytes.
  uint8_t data[PropertyDataBytes] =
      {};  ///< UTF-16LE `REG_MULTI_SZ` 数据区 / UTF-16LE `REG_MULTI_SZ` payload buffer.

  /**
   * @brief 使用指定 GUID 字符串初始化属性描述符 / Initialize the property descriptor from
   * a GUID string.
   * @param guid 形如 `{xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx}` 的 ASCII GUID 字符串 /
   *        ASCII GUID string formatted with braces.
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
 * @brief 设备作用域 WinUSB MS OS 2.0 描述符集 / Device-scoped WinUSB MS OS 2.0
 *        descriptor set.
 * @tparam PropertyDataBytes GUID 属性缓冲区字节数 / Property data buffer size in bytes.
 */
template <uint16_t PropertyDataBytes>
struct DeviceScopedWinUsbMsOs20DescSet
{
  MsOs20SetHeader set;  ///< 描述符集头部 / Descriptor set header.
  MsOs20FeatureCompatibleId
      compat;  ///< WinUSB 兼容 ID 特性块 / WinUSB compatible ID feature block.
  DeviceInterfaceGuidsRegProperty<PropertyDataBytes>
      prop;  ///< DeviceInterfaceGUIDs 注册表属性块 / `DeviceInterfaceGUIDs` property
             ///< block.

  /**
   * @brief 初始化设备作用域 WinUSB 描述符集 / Initialize the device-scoped WinUSB
   *        descriptor set.
   * @param guid 设备接口 GUID 字符串 / Device interface GUID string.
   * @param windows_version 最低 Windows 版本（默认 Win 8.1 `0x06030000`） / Minimum
   * Windows version.
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
 * @brief 接口功能作用域 WinUSB MS OS 2.0 描述符集（用于复合设备）/
 *        Function-scoped WinUSB MS OS 2.0 descriptor set (for composite devices).
 * @tparam PropertyDataBytes GUID 属性缓冲区字节数 / Property data buffer size in bytes.
 */
template <uint16_t PropertyDataBytes>
struct FunctionScopedWinUsbMsOs20DescSet
{
  MsOs20SetHeader set;                  ///< 描述符集头部 / Descriptor set header.
  MsOs20SubsetHeaderConfiguration cfg;  ///< 配置子集头 / Configuration subset header.
  MsOs20SubsetHeaderFunction func;      ///< 功能接口子集头 / Function subset header.
  MsOs20FeatureCompatibleId
      compat;  ///< WinUSB 兼容 ID 特性块 / WinUSB compatible ID feature block.
  DeviceInterfaceGuidsRegProperty<PropertyDataBytes>
      prop;  ///< DeviceInterfaceGUIDs 注册表属性块 / `DeviceInterfaceGUIDs` property
             ///< block.

  /**
   * @brief 初始化功能作用域 WinUSB 描述符集 / Initialize the function-scoped WinUSB
   *        descriptor set.
   * @param configuration_index 从 0 开始的配置索引 / Zero-based configuration index.
   * @param first_interface 目标功能首个接口号 / First USB interface index of the
   * function.
   * @param guid 设备接口 GUID 字符串 / Device interface GUID string.
   * @param windows_version 最低 Windows 版本（默认 Win 8.1 `0x06030000`） / Minimum
   * Windows version.
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
   * @brief 更新功能子集的首个接口编号 / Update the function's first interface number.
   * @param first_interface 新的首接口编号 / New first USB interface index.
   */
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
/**
 * @brief MS OS 2.0 BOS 平台能力处理类 / BOS capability handler serving an MS OS 2.0
 *        descriptor set.
 *
 * - wIndex=0x0007 (IN): return MS OS 2.0 descriptor set.
 * - wIndex=0x0008 (OUT): Set Alt Enumeration (usually no data stage).
 */
class MsOs20BosCapability final : public LibXR::USB::BosCapability
{
 public:
  /**
   * @brief 构造 MS OS 2.0 BOS 能力处理器 / Construct an MS OS 2.0 BOS capability handler.
   * @param descriptor_set MS OS 2.0 描述符集只读切片 / MS OS 2.0 descriptor set bytes.
   * @param vendor_code 主机查询使用的 `bMS_VendorCode` / Vendor request code for host
   * queries.
   * @param windows_version 最低兼容 Windows 版本 / Minimum supported Windows version.
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
   * @brief 更新返回的 MS OS 2.0 描述符集 / Update the served MS OS 2.0 descriptor set.
   * @param descriptor_set 新的描述符集只读切片 / New descriptor set byte view.
   */
  void SetDescriptorSet(LibXR::ConstRawData descriptor_set)
  {
    descriptor_set_ = descriptor_set;
    RefreshPlatformCap();
  }

  /**
   * @brief 更新 `bMS_VendorCode` 请求码 / Update the `bMS_VendorCode` request byte.
   * @param vendor_code 新的 Vendor 请求码 / New vendor request code.
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
