#pragma once

#include "ch32_usb.hpp"
#include "ch32_usb_endpoint.hpp"
#include "libxr_def.hpp"
#include "libxr_type.hpp"
#include "usb/core/ep_pool.hpp"
#include "usb/device/dev_core.hpp"

namespace LibXR
{

class CH32USBDevice : public USB::EndpointPool, public USB::DeviceCore
{
 public:
  /**
   * @brief 构造 CH32 USB 设备核心对象 / Construct CH32 USB device core
   */
  CH32USBDevice(
      ch32_usb_dev_id_t id, size_t max_ep_num,
      USB::DeviceDescriptor::PacketSize0 packet_size, uint16_t vid, uint16_t pid,
      uint16_t bcd,
      const std::initializer_list<const USB::DescriptorStrings::LanguagePack*> LANG_LIST,
      const std::initializer_list<const std::initializer_list<USB::ConfigDescriptorItem*>>
          CONFIGS,
      USB::Speed speed = USB::Speed::FULL, USB::USBSpec spec = USB::USBSpec::USB_2_1)
      : USB::EndpointPool(max_ep_num),
        USB::DeviceCore(*this, spec, speed, packet_size, vid, pid, bcd, LANG_LIST,
                        CONFIGS),
        id_(id)
  {
  }

  void Init(bool in_isr) override { USB::DeviceCore::Init(in_isr); }
  void Deinit(bool in_isr) override { USB::DeviceCore::Deinit(in_isr); }

  uint8_t id_;
};

#if defined(RCC_APB1Periph_USB) || defined(__DOXYGEN__)

/**
 * @brief CH32 FSDEV 设备驱动 / CH32 FSDEV device driver
 */
class CH32USBDeviceFS : public USB::EndpointPool, public USB::DeviceCore
{
 public:
  /**
   * @brief FSDEV 端点配置 / FSDEV endpoint configuration
   */
  struct EPConfig
  {
    LibXR::RawData buffer;
    int8_t is_in;

    EPConfig(LibXR::RawData buffer) : buffer(buffer), is_in(-1) {}

    EPConfig(LibXR::RawData buffer, bool is_in) : buffer(buffer), is_in(is_in ? 1 : 0) {}
  };

  /**
   * @brief 构造 FSDEV 设备对象 / Construct FSDEV device object
   */
  CH32USBDeviceFS(
      const std::initializer_list<EPConfig> EP_CFGS,
      USB::DeviceDescriptor::PacketSize0 packet_size, uint16_t vid, uint16_t pid,
      uint16_t bcd,
      const std::initializer_list<const USB::DescriptorStrings::LanguagePack*> LANG_LIST,
      const std::initializer_list<const std::initializer_list<USB::ConfigDescriptorItem*>>
          CONFIGS,
      ConstRawData uid = {nullptr, 0});

  ErrorCode SetAddress(uint8_t address, USB::DeviceCore::Context context) override;

  void Start(bool in_isr) override;
  void Stop(bool in_isr) override;

  static inline CH32USBDeviceFS* self_ = nullptr;
};

#endif  // defined(RCC_APB1Periph_USB) || defined(__DOXYGEN__)

#if defined(USBFSD) || defined(__DOXYGEN__)

/**
 * @brief CH32 OTG FS 设备驱动 / CH32 OTG FS device driver
 */
class CH32USBOtgFS : public USB::EndpointPool, public USB::DeviceCore
{
 public:
  /**
   * @brief OTG FS 端点配置 / OTG FS endpoint configuration
   */
  struct EPConfig
  {
    LibXR::RawData buffer;
    int8_t is_in;

    EPConfig(LibXR::RawData buffer) : buffer(buffer), is_in(-1) {}
    EPConfig(LibXR::RawData buffer, bool is_in) : buffer(buffer), is_in(is_in ? 1 : 0) {}
  };

  /**
   * @brief 构造 OTG FS 设备对象 / Construct OTG FS device object
   */
  CH32USBOtgFS(
      const std::initializer_list<EPConfig> EP_CFGS,
      USB::DeviceDescriptor::PacketSize0 packet_size, uint16_t vid, uint16_t pid,
      uint16_t bcd,
      const std::initializer_list<const USB::DescriptorStrings::LanguagePack*> LANG_LIST,
      const std::initializer_list<const std::initializer_list<USB::ConfigDescriptorItem*>>
          CONFIGS,
      ConstRawData uid = {nullptr, 0});

  ErrorCode SetAddress(uint8_t address, USB::DeviceCore::Context context) override;

  void Start(bool in_isr) override;
  void Stop(bool in_isr) override;

  static inline CH32USBOtgFS* self_ = nullptr;
};

#endif  // defined(USBFSD) || defined(__DOXYGEN__)

#if defined(USBHSD) || defined(__DOXYGEN__)

/**
 * @brief CH32 OTG HS 设备驱动 / CH32 OTG HS device driver
 */
class CH32USBOtgHS : public USB::EndpointPool, public USB::DeviceCore
{
 public:
  /**
   * @brief OTG HS 端点配置 / OTG HS endpoint configuration
   */
  struct EPConfig
  {
    RawData buffer_tx;
    RawData buffer_rx;
    bool double_buffer;
    bool is_in;

    EPConfig(RawData buffer)
        : buffer_tx(buffer), buffer_rx(buffer), double_buffer(false), is_in(false)
    {
    }

    EPConfig(RawData buffer, bool is_in)
        : buffer_tx(buffer), buffer_rx(buffer), double_buffer(true), is_in(is_in)
    {
    }

    EPConfig(RawData buffer_tx, RawData buffer_rx)
        : buffer_tx(buffer_tx), buffer_rx(buffer_rx), double_buffer(false), is_in(false)
    {
    }
  };

  /**
   * @brief 构造 OTG HS 设备对象 / Construct OTG HS device object
   */
  CH32USBOtgHS(
      const std::initializer_list<EPConfig> EP_CFGS, uint16_t vid, uint16_t pid,
      uint16_t bcd,
      const std::initializer_list<const USB::DescriptorStrings::LanguagePack*> LANG_LIST,
      const std::initializer_list<const std::initializer_list<USB::ConfigDescriptorItem*>>
          CONFIGS,
      ConstRawData uid = {nullptr, 0});

  ErrorCode SetAddress(uint8_t address, USB::DeviceCore::Context context) override;

  void Start(bool in_isr) override;
  void Stop(bool in_isr) override;

  static inline CH32USBOtgHS* self_ = nullptr;
};

#endif  // defined(USBHSD) || defined(__DOXYGEN__)

}  // namespace LibXR
