#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>

#include "core.hpp"
#include "desc_str.hpp"
#include "ep_pool.hpp"
#include "lockfree_list.hpp"

namespace LibXR::USB
{

/**
 * @brief USB 描述符类型
 *        USB Descriptor type
 */
enum class DescriptorType : uint8_t
{
  DEVICE = 0x01,             ///< 设备描述符（Device Descriptor）
  CONFIGURATION = 0x02,      ///< 配置描述符（Configuration Descriptor）
  STRING = 0x03,             ///< 字符串描述符（String Descriptor）
  INTERFACE = 0x04,          ///< 接口描述符（Interface Descriptor）
  ENDPOINT = 0x05,           ///< 端点描述符（Endpoint Descriptor）
  IAD = 0x0B,                ///< 接口关联描述符（Interface Association Descriptor）
  BOS = 0x0F,                ///< 设备能力描述符（BOS Descriptor）
  DEVICE_CAPABILITY = 0x10,  ///< 设备能力子描述符（Device Capability Descriptor）
  CS_INTERFACE = 0x24,       ///< 类特定接口描述符（Class-Specific Interface Descriptor）

};

/**
 * @brief USB描述符基类
 *        USB descriptor base class
 *
 */
class DeviceDescriptor
{
 public:
  /// USB 设备/接口类代码（bDeviceClass 取值）/ USB device or interface class codes
  /// (values of bDeviceClass)
  enum class ClassID : uint8_t
  {
    PER_INTERFACE = 0x00,         ///< 每个接口自定义类 / Per-interface
    AUDIO = 0x01,                 ///< 音频类 / Audio
    COMM = 0x02,                  ///< 通信类 / Communications (CDC)
    HID = 0x03,                   ///< 人机接口类 / Human Interface Device
    PHYSICAL = 0x05,              ///< 物理设备类 / Physical Device
    IMAGE = 0x06,                 ///< 图像/扫描仪类 / Imaging
    PRINTER = 0x07,               ///< 打印机类 / Printer
    MASS_STORAGE = 0x08,          ///< 大容量存储类 / Mass Storage
    HUB = 0x09,                   ///< 集线器类 / Hub
    CDC_DATA = 0x0A,              ///< CDC 数据类 / CDC Data
    SMART_CARD = 0x0B,            ///< 智能卡类 / Smart Card
    CONTENT_SECURITY = 0x0D,      ///< 内容安全类 / Content Security
    VIDEO = 0x0E,                 ///< 视频类 / Video
    PERSONAL_HEALTHCARE = 0x0F,   ///< 个人医疗健康类 / Personal Healthcare
    BILLBOARD = 0x11,             ///< Billboard类（USB Type-C专用）/ Billboard (Type-C)
    TYPE_C_BRIDGE = 0x12,         ///< Type-C Bridge类
    BULK_DISPLAY = 0x13,          ///< Bulk Display类
    MCTP = 0x14,                  ///< MCTP类
    I3C = 0x3C,                   ///< I3C类
    DIAGNOSTIC = 0xDC,            ///< 诊断类 / Diagnostic
    WIRELESS = 0xE0,              ///< 无线控制类 / Wireless Controller
    MISCELLANEOUS = 0xEF,         ///< 杂项类 / Miscellaneous
    APPLICATION_SPECIFIC = 0xFE,  ///< 应用专用类 / Application Specific
    VENDOR_SPECIFIC = 0xFF        ///< 厂商自定义类 / Vendor Specific
  };

  /**
   * @brief 控制端点0最大包长度枚举
   *        Packet size for endpoint 0 (bMaxPacketSize0)
   */
  enum class PacketSize0 : uint8_t
  {
    SIZE_8 = 8,    ///< 8字节 / 8 bytes      (Low Speed / Full Speed)
    SIZE_16 = 16,  ///< 16字节 / 16 bytes    (Full Speed)
    SIZE_32 = 32,  ///< 32字节 / 32 bytes    (Full Speed)
    SIZE_64 = 64,  ///< 64字节 / 64 bytes    (Full Speed / High Speed)
    SIZE_512 = 0,  ///< 512字节 / 512 bytes  (SuperSpeed)
  };

  static constexpr uint8_t DEVICE_DESC_LENGTH =
      18;  ///< 设备描述符长度（固定18字节）/ Device descriptor length (18 bytes)

  LIBXR_PACKED_BEGIN
  /// 设备描述符线上格式（18 字节，字段按 USB 2.0 表 9-8）/ Device descriptor wire
  /// layout (18 bytes, fields per USB 2.0 table 9-8)
  struct Data
  {
    uint8_t bLength;  ///< 描述符长度（固定18）/ Descriptor length (always 18)
    DescriptorType bDescriptorType;  ///< 描述符类型（0x01，设备描述符）/ Descriptor type
                                     ///< (0x01, device)
    USBSpec bcdUSB;  ///< USB协议版本 / USB specification release (e.g. 0x0200 for USB2.0)
    ClassID bDeviceClass;         ///< 设备类代码 / Device class code
    uint8_t bDeviceSubClass;      ///< 设备子类代码 / Device subclass code
    uint8_t bDeviceProtocol;      ///< 协议代码 / Protocol code
    PacketSize0 bMaxPacketSize0;  ///< 控制端点0最大包长 / Max packet size for endpoint 0
    uint16_t idVendor;            ///< 厂商ID（VID）/ Vendor ID
    uint16_t idProduct;           ///< 产品ID（PID）/ Product ID
    uint16_t bcdDevice;           ///< 设备版本号 / Device release number
    uint8_t iManufacturer;        ///< 厂商字符串索引 / Index of manufacturer string
    uint8_t iProduct;             ///< 产品字符串索引 / Index of product string
    uint8_t iSerialNumber;        ///< 序列号字符串索引 / Index of serial number string
    uint8_t bNumConfigurations;   ///< 支持的配置数 / Number of possible configurations
  };
  LIBXR_PACKED_END

  static_assert(sizeof(Data) == 18, "DeviceDescriptor must be 18 bytes");

  Data data_;  ///< 设备描述符数据实例 / Internal data instance

  /**
   * @brief 构造函数，初始化设备描述符
   *        Constructor, initialize device descriptor fields
   * @param spec USB协议版本 / USB specification (e.g. USBSpec(0x0200))
   * @param packet_size 端点0最大包长 / Max packet size for EP0
   * @param vid 厂商ID / Vendor ID
   * @param pid 产品ID / Product ID
   * @param bcd 设备版本号 / Device release number
   * @param num_configs 配置数 / Number of configurations
   *
   * 默认类为MISCELLANEOUS、子类0x02、协议0x01。
   * Device class defaults to MISCELLANEOUS, subclass 0x02, protocol 0x01.
   */
  DeviceDescriptor(USBSpec spec, PacketSize0 packet_size, uint16_t vid, uint16_t pid,
                   uint16_t bcd, uint8_t num_configs);
  /**
   * @brief 获取设备描述符的原始数据指针及长度
   *        Get the raw device descriptor data pointer and length
   * @return RawData 结构，包含指针与长度 / RawData struct containing pointer and size
   */
  RawData GetData();

  /**
   * @brief 获取USB协议版本
   *        Get USB specification
   *
   * @return USBSpec
   */
  USBSpec GetUSBSpec() const;
};
}  // namespace LibXR::USB
