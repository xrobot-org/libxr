#pragma once

#include <cstddef>
#include <cstdint>
#include <initializer_list>

#include "bos.hpp"
#include "core.hpp"
#include "desc_dev.hpp"
#include "ep_pool.hpp"
#include "libxr_type.hpp"

namespace LibXR::USB
{
/**
 * @brief 配置描述符属性位（bmAttributes）
 *        USB configuration descriptor attribute bits (bmAttributes)
 */
constexpr uint8_t CFG_BUS_POWERED = 0x80;    ///< 总线供电 / Bus-powered
constexpr uint8_t CFG_SELF_POWERED = 0x40;   ///< 自供电 / Self-powered
constexpr uint8_t CFG_REMOTE_WAKEUP = 0x20;  ///< 远程唤醒 / Remote wakeup

class ConfigDescriptor;

/**
 * @brief USB 配置项基类（功能块）/ USB configuration item base (functional block)
 *
 * 一个配置项通常包含一个功能集合，可由多个 Interface/IAD 组成。
 * A configuration item usually represents one function group, consisting of multiple
 * Interface descriptors and/or an IAD.
 *
 * 同时作为 BOS capability 提供者（BosCapabilityProvider）。
 * Also serves as a BOS capability provider (BosCapabilityProvider).
 */
class ConfigDescriptorItem : public BosCapabilityProvider
{
 public:
  virtual ~ConfigDescriptorItem() = default;

  LIBXR_PACKED_BEGIN
  /**
   * @brief 配置描述符头（9 字节）/ Configuration descriptor header (9 bytes)
   */
  struct Header
  {
    uint8_t bLength = 9;          ///< 描述符长度 / Descriptor length
    uint8_t bDescriptorType;      ///< 描述符类型（0x02）/ Descriptor type (0x02)
    uint16_t wTotalLength;        ///< 配置总长度 / Total configuration length
    uint8_t bNumInterfaces;       ///< 接口数量 / Number of interfaces
    uint8_t bConfigurationValue;  ///< 配置值 / Configuration value
    uint8_t iConfiguration;       ///< 配置字符串索引 / Configuration string index
    uint8_t bmAttributes;         ///< 属性位 / Attributes
    uint8_t bMaxPower;            ///< 最大电流（2mA 单位）/ Max power (2mA units)
  };

  /**
   * @brief IAD（8 字节）/ Interface Association Descriptor (8 bytes)
   */
  struct IADDescriptor
  {
    uint8_t bLength = 8;             ///< 描述符长度 / Descriptor length
    uint8_t bDescriptorType = 0x0B;  ///< 描述符类型 / Descriptor type
    uint8_t bFirstInterface;         ///< 首接口号 / First interface number
    uint8_t bInterfaceCount;         ///< 接口数量 / Interface count
    uint8_t bFunctionClass;          ///< 功能类 / Function class
    uint8_t bFunctionSubClass;       ///< 功能子类 / Function subclass
    uint8_t bFunctionProtocol;       ///< 功能协议 / Function protocol
    uint8_t iFunction;               ///< 功能字符串索引 / Function string index
  };

  /**
   * @brief 接口描述符（9 字节）/ Interface descriptor (9 bytes)
   */
  struct InterfaceDescriptor
  {
    uint8_t bLength = 9;             ///< 描述符长度 / Descriptor length
    uint8_t bDescriptorType = 0x04;  ///< 描述符类型 / Descriptor type
    uint8_t bInterfaceNumber;        ///< 接口号 / Interface number
    uint8_t bAlternateSetting;       ///< 备用设置号 / Alternate setting
    uint8_t bNumEndpoints;           ///< 端点数量 / Number of endpoints
    uint8_t bInterfaceClass;         ///< 接口类 / Interface class
    uint8_t bInterfaceSubClass;      ///< 接口子类 / Interface subclass
    uint8_t bInterfaceProtocol;      ///< 接口协议 / Interface protocol
    uint8_t iInterface;              ///< 接口字符串索引 / Interface string index
  };

  /**
   * @brief 端点描述符（7 字节）/ Endpoint descriptor (7 bytes)
   */
  struct EndpointDescriptor
  {
    uint8_t bLength = 7;             ///< 描述符长度 / Descriptor length
    uint8_t bDescriptorType = 0x05;  ///< 描述符类型 / Descriptor type
    uint8_t bEndpointAddress;        ///< 端点地址 / Endpoint address
    uint8_t bmAttributes;            ///< 端点属性 / Endpoint attributes
    uint16_t wMaxPacketSize;         ///< 最大包长 / Maximum packet size
    uint8_t bInterval;               ///< 轮询间隔 / Polling interval
  };
  LIBXR_PACKED_END

  /**
   * @brief 绑定端点资源 / Bind endpoint resources
   * @param endpoint_pool 端点池 / Endpoint pool
   * @param start_itf_num 起始接口号 / Start interface number
   * @param in_isr 是否在中断中 / Whether in ISR
   */
  virtual void BindEndpoints(EndpointPool& endpoint_pool, uint8_t start_itf_num,
                             bool in_isr) = 0;

  /**
   * @brief 解绑端点资源 / Unbind endpoint resources
   * @param endpoint_pool 端点池 / Endpoint pool
   * @param in_isr 是否在中断中 / Whether in ISR
   */
  virtual void UnbindEndpoints(EndpointPool& endpoint_pool, bool in_isr) = 0;

  /**
   * @brief 可选：覆盖设备描述符字段 / Optional: override device descriptor fields
   * @param header 设备描述符 / Device descriptor
   * @return OK：已覆盖；NOT_SUPPORT：不支持 / OK: applied; NOT_SUPPORT: not supported
   */
  virtual ErrorCode WriteDeviceDescriptor(DeviceDescriptor& header)
  {
    UNUSED(header);
    return ErrorCode::NOT_SUPPORT;
  }

  /**
   * @brief 可选：设置接口备用设置 / Optional: set interface alternate setting
   * @param itf 接口号 / Interface number
   * @param alt 备用设置 / Alternate setting
   * @return OK：已切换，设备核心随后以 ZLP 完成 SET_INTERFACE；
   *         返回其他错误码时，设备核心 STALL 该请求。实现可以返回
   *         NOT_SUPPORT（没有该备用设置）、NOT_FOUND（接口号不属于本类）
   *         或 BUSY（当前状态不允许切换）等。默认实现只接受备用设置 0。
   *         OK: switched, and the device core completes SET_INTERFACE with a ZLP;
   *         for any other code the device core stalls the request. Implementations
   *         may return, for example, NOT_SUPPORT (no such alternate setting),
   *         NOT_FOUND (the interface number does not belong to this class) or BUSY
   *         (the current state does not allow a switch). The default implementation
   *         accepts only alternate setting 0.
   */
  virtual ErrorCode SetAltSetting(uint8_t itf, uint8_t alt)
  {
    UNUSED(itf);
    return (alt == 0) ? ErrorCode::OK : ErrorCode::NOT_SUPPORT;
  }

  /**
   * @brief 可选：获取接口备用设置 / Optional: get interface alternate setting
   * @param itf 接口号 / Interface number
   * @param alt 输出：备用设置 / Output: alternate setting
   * @return OK：已写入 alt；其他错误码表示未写入。设备核心不检查返回值，
   *         GET_INTERFACE 总是回复 alt，调用前 alt 为 0。默认实现返回 NOT_SUPPORT。
   *         OK: alt was written; any other code means it was not. The device core
   *         does not check the return value and always answers GET_INTERFACE with
   *         alt, which it sets to 0 before the call. The default implementation
   *         returns NOT_SUPPORT.
   */
  virtual ErrorCode GetAltSetting(uint8_t itf, uint8_t& alt)
  {
    UNUSED(itf);
    UNUSED(alt);
    return ErrorCode::NOT_SUPPORT;
  }

  /**
   * @brief 可选：端点归属判定 / Optional: endpoint ownership
   * @param ep_addr 端点地址 / Endpoint address
   * @return true：归属；false：不归属 / true: owned; false: not owned
   */
  virtual bool OwnsEndpoint(uint8_t ep_addr) const
  {
    UNUSED(ep_addr);
    return false;
  }

  /**
   * @brief 最大配置描述符占用 / Maximum bytes required in configuration descriptor
   * @return 最大字节数 / Maximum bytes
   */
  virtual size_t GetMaxConfigSize() = 0;

  /**
   * @brief 接口数量 / Number of interfaces contributed
   * @return 接口数量 / Interface count
   */
  virtual size_t GetInterfaceCount() = 0;

  /**
   * @brief 是否包含 IAD / Whether an IAD is used
   * @return true：包含；false：不包含 / true: used; false: not used
   */
  virtual bool HasIAD() = 0;

 protected:
  /**
   * @brief 获取内部数据缓存 / Get internal data cache
   * @return RawData 数据结构 / RawData
   */
  RawData GetData();

  /**
   * @brief 设置内部数据缓存 / Set internal data cache
   * @param data 数据缓存 / Data cache
   */
  void SetData(RawData data) { data_ = data; }

  friend class ConfigDescriptor;

 private:
  RawData data_{nullptr, 0};  ///< 内部数据缓存 / Internal data cache
};

/**
 * @brief 配置描述符字节构造器
 *        Configuration descriptor byte builder.
 *
 * @note 该类只负责把已经准备好的配置项描述符块拼成一个 configuration descriptor。
 *       端点绑定、BOS 聚合、字符串分配、配置切换等职责应由上层 composition 管理。
 */
class ConfigDescriptor
{
  using Header = ConfigDescriptorItem::Header;

 public:
  /**
   * @brief 构造函数 / Constructor
   *
   * @param buffer_size 配置描述符缓冲区大小 / Configuration descriptor buffer size
   * @param bmAttributes 配置属性 / bmAttributes
   * @param bMaxPower 最大电流（单位 2mA）/ Max power (2mA units)
   */
  explicit ConfigDescriptor(size_t buffer_size, uint8_t bmAttributes = CFG_BUS_POWERED,
                            uint8_t bMaxPower = 50);

  ConfigDescriptor(const ConfigDescriptor&) = delete;
  ConfigDescriptor& operator=(const ConfigDescriptor&) = delete;

  /**
   * @brief 计算所有 configuration 中需要的最大缓冲区大小
   *        Calculate the maximum buffer size required across all configurations.
   */
  static size_t CalcMaxConfigSize(
      const std::initializer_list<const std::initializer_list<ConfigDescriptorItem*>>&
          configs);

  /**
   * @brief 构建指定 configuration 的描述符
   *        Build the descriptor for the specified configuration.
   *
   * @param items 配置项列表 / Configuration item list
   * @param item_num 配置项数量 / Configuration item count
   * @param configuration_value 配置值 / Configuration value
   * @param i_configuration 配置字符串索引 / Configuration string index
   * @return 错误码 / Error code
   */
  ErrorCode BuildConfigDescriptor(ConfigDescriptorItem* const* items, size_t item_num,
                                  uint8_t configuration_value,
                                  uint8_t i_configuration = 0);

  /**
   * @brief 获取配置描述符数据 / Get configuration descriptor data
   * @return RawData 数据结构 / RawData
   */
  [[nodiscard]] RawData GetData() const;

 private:
  uint8_t bm_attributes_ = CFG_BUS_POWERED;  ///< 配置属性 / bmAttributes
  uint8_t b_max_power_ = 50;  ///< 最大电流（2mA 单位）/ Max power (2mA units)

  RawData buffer_{nullptr, 0};  ///< 配置描述符缓冲区 / Configuration descriptor buffer
  size_t buffer_index_ = 0;     ///< 当前有效字节数 / Current valid byte count
};

}  // namespace LibXR::USB
