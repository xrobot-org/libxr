#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "libxr_mem.hpp"
#include "libxr_type.hpp"
#include "usb/core/bos.hpp"
#include "usb/core/core.hpp"

namespace LibXR::USB::WebUsb
{

// ---- WebUSB constants ----

static constexpr uint16_t WEBUSB_REQUEST_GET_URL = 0x0002u;
static constexpr uint8_t WEBUSB_VENDOR_CODE_DEFAULT = 0x01u;
static constexpr uint8_t WEBUSB_URL_DESCRIPTOR_TYPE = 0x03u;
static constexpr uint8_t WEBUSB_URL_SCHEME_HTTP = 0x00u;
static constexpr uint8_t WEBUSB_URL_SCHEME_HTTPS = 0x01u;
static constexpr uint8_t WEBUSB_LANDING_PAGE_INDEX = 0x01u;

// WebUSB platform capability UUID: 3408b638-09a9-47a0-8bfd-a0768815b665
static constexpr uint8_t WEBUSB_PLATFORM_CAPABILITY_UUID[16] = {
    0x38, 0xB6, 0x08, 0x34, 0xA9, 0x09, 0xA0, 0x47,
    0x8B, 0xFD, 0xA0, 0x76, 0x88, 0x15, 0xB6, 0x65,
};

LIBXR_PACKED_BEGIN

/**
 * @brief WebUSB BOS 平台能力描述符 / WebUSB BOS platform capability descriptor
 */
struct WebUsbPlatformCapability
{
  uint8_t bLength = 0x18u;
  uint8_t bDescriptorType = 0x10u;
  uint8_t bDevCapabilityType = 0x05u;
  uint8_t bReserved = 0x00u;
  uint8_t PlatformCapabilityUUID[16] = {};
  uint16_t bcdVersion = 0x0100u;
  uint8_t bVendorCode = WEBUSB_VENDOR_CODE_DEFAULT;
  uint8_t iLandingPage = 0u;
};
LIBXR_PACKED_END

static_assert(sizeof(WebUsbPlatformCapability) == 24,
              "WebUSB platform capability size mismatch.");

/**
 * @brief WebUSB BOS 能力包装器 / WebUSB BOS capability wrapper
 *
 * landing page URL 会被编码成 WebUSB URL descriptor，并在 GET_URL vendor request
 * 中直接返回。
 * The landing-page URL is encoded as a WebUSB URL descriptor and returned
 * directly from the GET_URL vendor request.
 */
class WebUsbBosCapability final : public LibXR::USB::BosCapability
{
 public:
  /**
   * @brief 构造能力对象，并可选缓存 landing page URL
   *        Construct the capability and optionally cache the landing-page URL.
   */
  explicit WebUsbBosCapability(const char* landing_page_url = nullptr,
                               uint8_t vendor_code = WEBUSB_VENDOR_CODE_DEFAULT)
      : vendor_code_(vendor_code)
  {
    url_descriptor_.addr_ = url_descriptor_storage_;
    LibXR::Memory::FastCopy(platform_cap_.PlatformCapabilityUUID,
                            WEBUSB_PLATFORM_CAPABILITY_UUID,
                            sizeof(WEBUSB_PLATFORM_CAPABILITY_UUID));
    platform_cap_.bVendorCode = vendor_code_;
    const bool landing_page_ok = SetLandingPageUrl(landing_page_url);
    ASSERT(landing_page_ok);
  }

  WebUsbBosCapability(const WebUsbBosCapability&) = delete;
  WebUsbBosCapability& operator=(const WebUsbBosCapability&) = delete;

  /**
   * @brief 当前是否启用 WebUSB / Whether WebUSB is currently enabled
   */
  bool Enabled() const { return enabled_; }

  /**
   * @brief 更新 GET_URL 使用的 vendor code
   *        Update the vendor code used by GET_URL.
   */
  void SetVendorCode(uint8_t vendor_code)
  {
    vendor_code_ = vendor_code;
    platform_cap_.bVendorCode = vendor_code_;
  }

  /**
   * @brief 解析并缓存 landing page URL
   *        Parse and cache the landing-page URL.
   *
   * 仅接受 `http://` / `https://` 前缀；空字符串表示禁用 WebUSB。
   * Only `http://` / `https://` prefixes are accepted; an empty string disables
   * WebUSB.
   */
  bool SetLandingPageUrl(const char* landing_page_url)
  {
    url_descriptor_.size_ = 0u;

    if (landing_page_url == nullptr || landing_page_url[0] == '\0')
    {
      enabled_ = false;
      platform_cap_.iLandingPage = 0u;
      return true;
    }

    uint8_t scheme = WEBUSB_URL_SCHEME_HTTPS;
    const char* url_body = nullptr;
    if (!ParseLandingPageUrl(landing_page_url, scheme, url_body))
    {
      enabled_ = false;
      platform_cap_.iLandingPage = 0u;
      return false;
    }

    const size_t URL_BODY_LEN = std::strlen(url_body);
    if (URL_BODY_LEN == 0u || URL_BODY_LEN > 252u)
    {
      // URL descriptor 的总长度由 1 字节表示，且包含前 3 字节头部。
      // The URL descriptor length is stored in one byte and includes the
      // 3-byte header.
      enabled_ = false;
      platform_cap_.iLandingPage = 0u;
      return false;
    }

    url_descriptor_storage_[0] = static_cast<uint8_t>(URL_BODY_LEN + 3u);
    url_descriptor_storage_[1] = WEBUSB_URL_DESCRIPTOR_TYPE;
    url_descriptor_storage_[2] = scheme;
    LibXR::Memory::FastCopy(url_descriptor_storage_ + 3u, url_body, URL_BODY_LEN);

    url_descriptor_.size_ = URL_BODY_LEN + 3u;
    enabled_ = true;
    platform_cap_.iLandingPage = WEBUSB_LANDING_PAGE_INDEX;
    return true;
  }

  /**
   * @brief 获取 BOS capability 描述符 / Get the BOS capability descriptor
   */
  ConstRawData GetCapabilityDescriptor() const override
  {
    if (!enabled_)
    {
      return {nullptr, 0};
    }
    return {reinterpret_cast<const uint8_t*>(&platform_cap_), sizeof(platform_cap_)};
  }

  /**
   * @brief 处理 WebUSB GET_URL vendor request
   *        Handle the WebUSB GET_URL vendor request.
   */
  ErrorCode OnVendorRequest(bool, const SetupPacket* setup,
                            LibXR::USB::BosVendorResult& result) override
  {
    if (!enabled_)
    {
      return ErrorCode::NOT_SUPPORT;
    }
    if (setup == nullptr)
    {
      return ErrorCode::ARG_ERR;
    }

    const uint8_t BM = setup->bmRequestType;

    if ((BM & 0x60u) != 0x40u)
    {
      return ErrorCode::NOT_SUPPORT;
    }
    if ((BM & 0x1Fu) != 0x00u)
    {
      return ErrorCode::NOT_SUPPORT;
    }
    if ((BM & 0x80u) == 0u)
    {
      return ErrorCode::NOT_SUPPORT;
    }
    if (setup->bRequest != vendor_code_ || setup->wIndex != WEBUSB_REQUEST_GET_URL)
    {
      return ErrorCode::NOT_SUPPORT;
    }
    if ((setup->wValue & 0xFFu) != WEBUSB_LANDING_PAGE_INDEX || setup->wLength == 0u)
    {
      return ErrorCode::ARG_ERR;
    }

    result.handled = true;
    result.in_data = {reinterpret_cast<const uint8_t*>(url_descriptor_.addr_),
                      url_descriptor_.size_};
    result.write_zlp = false;
    result.early_read_zlp = true;
    return ErrorCode::OK;
  }

 private:
  /**
   * @brief 解析 URL scheme，并返回去掉前缀后的正文
   *        Parse the URL scheme and return the prefix-stripped body.
   */
  static bool ParseLandingPageUrl(const char* input, uint8_t& scheme, const char*& body)
  {
    static constexpr const char HTTPS_PREFIX[] = "https://";
    static constexpr const char HTTP_PREFIX[] = "http://";

    if (std::strncmp(input, HTTPS_PREFIX, sizeof(HTTPS_PREFIX) - 1u) == 0)
    {
      scheme = WEBUSB_URL_SCHEME_HTTPS;
      body = input + sizeof(HTTPS_PREFIX) - 1u;
      return true;
    }
    if (std::strncmp(input, HTTP_PREFIX, sizeof(HTTP_PREFIX) - 1u) == 0)
    {
      scheme = WEBUSB_URL_SCHEME_HTTP;
      body = input + sizeof(HTTP_PREFIX) - 1u;
      return true;
    }

    return false;
  }

  uint8_t vendor_code_ = WEBUSB_VENDOR_CODE_DEFAULT;
  bool enabled_ = false;
  uint8_t url_descriptor_storage_[255] =
      {};  ///< URL 描述符静态缓冲 / Static URL descriptor buffer
  RawData url_descriptor_{nullptr, 0};  ///< 已编码 URL 描述符 / Encoded URL descriptor
  WebUsbPlatformCapability platform_cap_{};
};

}  // namespace LibXR::USB::WebUsb
