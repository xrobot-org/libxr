#pragma once

#include <cstddef>
#include <cstdint>

#include "crc.hpp"
#include "dev_core.hpp"
#include "flash.hpp"
#include "timebase.hpp"
#include "webusb.hpp"
#include "winusb_msos20.hpp"

namespace LibXR::USB
{
/**
 * @brief DFU 类请求码 / DFU class request codes
 */
enum class DFURequest : uint8_t
{
  DETACH = 0x00,
  DNLOAD = 0x01,
  UPLOAD = 0x02,
  GETSTATUS = 0x03,
  CLRSTATUS = 0x04,
  GETSTATE = 0x05,
  ABORT = 0x06,
};

/**
 * @brief DFU 协议状态 / DFU protocol state
 */
enum class DFUState : uint8_t
{
  APP_IDLE = 0x00,
  APP_DETACH = 0x01,
  DFU_IDLE = 0x02,
  DFU_DNLOAD_SYNC = 0x03,
  DFU_DNBUSY = 0x04,
  DFU_DNLOAD_IDLE = 0x05,
  DFU_MANIFEST_SYNC = 0x06,
  DFU_MANIFEST = 0x07,
  DFU_MANIFEST_WAIT_RESET = 0x08,
  DFU_UPLOAD_IDLE = 0x09,
  DFU_ERROR = 0x0A,
};

/**
 * @brief DFU 状态码 / DFU status code
 */
enum class DFUStatusCode : uint8_t
{
  OK = 0x00,
  ERR_TARGET = 0x01,
  ERR_FILE = 0x02,
  ERR_WRITE = 0x03,
  ERR_ERASE = 0x04,
  ERR_CHECK_ERASED = 0x05,
  ERR_PROG = 0x06,
  ERR_VERIFY = 0x07,
  ERR_ADDRESS = 0x08,
  ERR_NOTDONE = 0x09,
  ERR_FIRMWARE = 0x0A,
  ERR_VENDOR = 0x0B,
  ERR_USBR = 0x0C,
  ERR_POR = 0x0D,
  ERR_UNKNOWN = 0x0E,
  ERR_STALLEDPKT = 0x0F,
};

/**
 * @brief DFU 功能能力集合 / DFU functional capability set
 */
struct DFUCapabilities
{
  bool can_download = true;
  bool can_upload = true;
  bool manifestation_tolerant = true;
  bool will_detach = false;
  uint16_t detach_timeout_ms = 1000u;
  uint16_t transfer_size = 1024u;
};

/**
 * @brief DFU 单接口类公共基类 / Common base for single-interface DFU classes
 */
class DfuInterfaceClassBase : public DeviceClass
{
 protected:
  static constexpr const char* DEFAULT_WINUSB_DEVICE_INTERFACE_GUID =
      "{4066E5F4-3B02-4B90-9475-12F770A7841B}";

  // Shared single-interface DFU state:
  // - one interface string
  // - optional WebUSB BOS capability
  // - the WinUSB interface GUID; the composition builds the MS OS 2.0 descriptor set
  // - current interface/alt-setting pair
  DfuInterfaceClassBase(
      const char* interface_string, const char* webusb_landing_page_url = nullptr,
      uint8_t webusb_vendor_code = LibXR::USB::WebUsb::WEBUSB_VENDOR_CODE_DEFAULT,
      const char* winusb_device_interface_guid = DEFAULT_WINUSB_DEVICE_INTERFACE_GUID)
      : interface_string_(interface_string),
        winusb_device_interface_guid_(winusb_device_interface_guid),
        webusb_cap_(webusb_landing_page_url, webusb_vendor_code)
  {
  }

  const char* GetInterfaceString(size_t local_interface_index) const override
  {
    return (local_interface_index == 0u) ? interface_string_ : nullptr;
  }

  /// nullptr 或空串表示不请求 WinUSB 绑定。 / nullptr or "" requests no WinUSB binding.
  const char* GetWinUsbInterfaceGuid() const override
  {
    return (winusb_device_interface_guid_ != nullptr &&
            winusb_device_interface_guid_[0] != '\0')
               ? winusb_device_interface_guid_
               : nullptr;
  }

  size_t GetBosCapabilityCount() override { return webusb_cap_.Enabled() ? 1u : 0u; }

  BosCapability* GetBosCapability(size_t index) override
  {
    return (index == 0u && webusb_cap_.Enabled()) ? &webusb_cap_ : nullptr;
  }

  uint8_t interface_num_ = 0u;
  uint8_t current_alt_setting_ = 0u;
  bool inited_ = false;

 private:
  const char* interface_string_ = nullptr;
  const char* winusb_device_interface_guid_ = nullptr;

 protected:
  LibXR::USB::WebUsb::WebUsbBosCapability webusb_cap_;
};

}  // namespace LibXR::USB
