#pragma once

#include <cstddef>
#include <cstdint>
#include <initializer_list>

#include "device_class.hpp"
#include "libxr_def.hpp"
#include "libxr_type.hpp"
#include "usb/core/bos.hpp"
#include "usb/core/core.hpp"
#include "usb/core/desc_cfg.hpp"
#include "usb/core/desc_dev.hpp"
#include "usb/core/desc_str.hpp"
#include "usb/core/ep_pool.hpp"

namespace LibXR::USB
{

class DeviceService;

/**
 * @class DeviceComposition
 * @brief 设备的类和配置 / The classes and configurations of a device
 *
 * 持有构造时给出的配置列表和类表、字符串描述符、BOS，以及配置描述符的缓冲；记录主机选中
 * 的配置。配置描述符在每次请求时由各类的 WriteDescriptors 写成，不缓存。唯一使用者是
 * 设备核心。
 * Holds the configuration list and class table given at construction, the string
 * descriptors, the BOS and the buffer for configuration descriptors, and records the
 * configuration the host selected. A configuration descriptor is written by the
 * classes' WriteDescriptors on every request and not cached. The device core is the only
 * user.
 *
 * @note 内存只在构造时分配。除构造外，所有函数只在设备的 owner 中调用。
 *       Memory is allocated at construction only. Apart from the constructor, every
 *       function is called in the device's owner only.
 */
class DeviceComposition
{
 public:
  /**
   * @brief 构造 / Construct
   * @param pool 端点池，供类写描述符和绑定 / Endpoint pool for the classes' descriptors
   *        and binding
   * @param service 设备服务，交给每个类的 RequestService / Device service handed to
   *        each class's RequestService
   * @param languages 字符串语言包 / String language packs
   * @param configs 每个配置的类列表；同一个类可以出现在多个配置中 / Class list of each
   *        configuration; one class may appear in several
   * @param uid 序列号字符串后缀，可为空 / Serial-number suffix, may be empty
   * @param bm_attributes 配置描述符 bmAttributes / bmAttributes of the configuration
   *        descriptors
   * @param max_power 配置描述符 bMaxPower（2 mA）/ bMaxPower (2 mA units)
   * @note 调用者：设备核心构造时一次。去重得到类表，给每个类写入服务指针和接口字符串
   *       号，按所有配置中最大的 GetMaxConfigSize 之和分配配置描述符缓冲，建 BOS。
   *       Caller: the device core, once at construction. Builds the deduplicated class
   *       table, writes each class's service pointer and interface string base,
   *       allocates the configuration descriptor buffer for the largest sum of
   *       GetMaxConfigSize over the configurations and builds the BOS.
   */
  DeviceComposition(
      EndpointPool& pool, DeviceService& service,
      const std::initializer_list<const DescriptorStrings::LanguagePack*>& languages,
      const std::initializer_list<const std::initializer_list<DeviceClass*>>& configs,
      ConstRawData uid = {nullptr, 0}, uint8_t bm_attributes = CFG_BUS_POWERED,
      uint8_t max_power = 50U);

  /// 不可复制：类按地址持有本对象的数据 / Not copyable: classes hold this object's data
  /// by address
  DeviceComposition(const DeviceComposition&) = delete;
  /// 不可复制赋值 / Not copy-assignable
  DeviceComposition& operator=(const DeviceComposition&) = delete;

  /**
   * @brief 配置个数 / Number of configurations
   * @note 调用者：设备核心（设备描述符、SET_CONFIGURATION 检查）。
   *       Caller: the device core (device descriptor, SET_CONFIGURATION check).
   */
  uint8_t ConfigCount() const { return config_count_; }

  /// 当前配置值，0 表示未配置；调用者：设备核心 / Current configuration value, 0 when
  /// not configured; caller: the device core
  uint8_t GetConfiguration() const { return configuration_; }

  /**
   * @brief 切换配置 / Switch the configuration
   * @param value 配置值，0 表示取消配置 / Configuration value, 0 to deconfigure
   * @param speed 当前总线速度 / Current bus speed
   * @param in_isr 是否在中断中 / Whether in an ISR
   * @return OK；ARG_ERR：value 大于 ConfigCount()，什么都不改；其他：某个类的
   *         WriteDescriptors 或 BindEndpoints 的错误，此时已回到未配置。
   *         OK; ARG_ERR: value exceeds ConfigCount() and nothing changes; otherwise the
   *         error of a class's WriteDescriptors or BindEndpoints, and the device is
   *         back to unconfigured.
   * @note 依次：检查 value；当前配置的每个类 UnbindEndpoints，记为未配置；value 非 0 时
   *       对新配置的每个类按接口号顺序 WriteDescriptors(speed) 再 BindEndpoints；全部
   *       成功才记为 value。同一个 value 也重新解绑和绑定。调用者：设备核心，处理
   *       SET_CONFIGURATION、复位、Deinit、断开和停止失败时。
   *       In order: check value; UnbindEndpoints on every class of the current
   *       configuration and record unconfigured; for a non-zero value, WriteDescriptors
   *       (speed) and then BindEndpoints on each class of the new one in interface
   *       order; record value only if all succeed. The same value unbinds and binds
   *       again. Caller: the device core on SET_CONFIGURATION, reset, Deinit,
   *       disconnect and a failed stop.
   */
  [[nodiscard]] ErrorCode SetConfiguration(uint8_t value, Speed speed, bool in_isr);

  /**
   * @brief 写一个配置描述符 / Write one configuration descriptor
   * @param index 从 0 起的配置序号 / Zero-based configuration index
   * @param speed 描述符对应的速度 / Speed the descriptor is for
   * @param data 输出：本对象缓冲中的描述符，到下一次调用前有效；设备核心回答
   *        OTHER_SPEED_CONFIGURATION 时改写其类型字节。
   *        Output: the descriptor in this object's buffer, valid until the next call;
   *        the device core rewrites its type byte when answering
   *        OTHER_SPEED_CONFIGURATION.
   * @return OK；ARG_ERR：index 越界；其他：某个类 WriteDescriptors 的错误。
   *         OK; ARG_ERR: index out of range; otherwise a class's WriteDescriptors error.
   * @note 不分配内存，不改配置状态。调用者：设备核心，GET_DESCRIPTOR(CONFIGURATION /
   *       OTHER_SPEED_CONFIGURATION)。
   *       Allocates nothing and does not change the configuration state. Caller: the
   *       device core on GET_DESCRIPTOR (CONFIGURATION or OTHER_SPEED_CONFIGURATION).
   */
  [[nodiscard]] ErrorCode WriteConfigDescriptor(uint8_t index, Speed speed,
                                                RawData& data);

  /**
   * @brief 字符串描述符 / String descriptor
   * @param index 字符串号 / String index
   * @param lang 语言 ID / Language ID
   * @param data 输出：到下一次调用前有效 / Output: valid until the next call
   * @return OK 或 NOT_FOUND / OK or NOT_FOUND
   * @note 调用者：设备核心，GET_DESCRIPTOR(STRING) / Caller: the device core on
   *       GET_DESCRIPTOR(STRING)
   */
  [[nodiscard]] ErrorCode GetStringDescriptor(uint8_t index, uint16_t lang,
                                              ConstRawData& data);

  /**
   * @brief 设备的 BOS / The device's BOS
   * @note 能力集合在构造时由全部类（与配置无关）和 WinUSB 能力组成，之后不变。调用者：
   *       设备核心，GET_DESCRIPTOR(BOS) 和设备为接收者的厂商请求。
   *       The capability set is made at construction from every class (whatever the
   *       configuration) plus the WinUSB capability and does not change afterwards.
   *       Caller: the device core on GET_DESCRIPTOR(BOS) and on vendor requests whose
   *       recipient is the device.
   */
  BosManager& Bos() { return bos_; }

  /**
   * @brief 当前配置中拥有该接口号的类 / Class of the current configuration that owns
   *        the interface number
   * @return 类；未配置或不存在时为 nullptr / The class; nullptr when unconfigured or
   *         absent
   * @note 调用者：设备核心，接口为接收者的请求 / Caller: the device core, for requests
   *       whose recipient is an interface
   */
  DeviceClass* FindClassByInterface(uint8_t itf) const;

  /**
   * @brief 当前配置中拥有该端点地址的类 / Class of the current configuration that owns
   *        the endpoint address
   * @return 类；未配置或不存在时为 nullptr / The class; nullptr when unconfigured or
   *         absent
   * @note 调用者：设备核心，端点为接收者的类请求和 CLEAR_FEATURE(ENDPOINT_HALT) /
   *       Caller: the device core, for class requests whose recipient is an endpoint
   *       and for CLEAR_FEATURE(ENDPOINT_HALT)
   */
  DeviceClass* FindClassByEndpoint(uint8_t ep_addr) const;

  /// 设备 GET_STATUS 的值（自供电位）；调用者：设备核心 / Device GET_STATUS value (the
  /// self-powered bit); caller: the device core
  uint16_t DeviceStatus() const;

  /**
   * @brief 所有类的 GetControlReceiveCapacity 的最大值，至少 64 / Largest
   *        GetControlReceiveCapacity of the classes, at least 64
   * @note 调用者：设备核心构造时。
   *       Caller: the device core at construction.
   */
  size_t ControlReceiveCapacity() const;

  /**
   * @brief 单功能设备由类改写设备描述符 / A single-function device lets its class
   *        rewrite the device descriptor
   * @return OK：已改写；NOT_SUPPORT：复合设备或类不改写。
   *         OK: rewritten; NOT_SUPPORT: composite device, or the class does not.
   * @note 调用者：设备核心构造时一次 / Caller: the device core, once at construction
   */
  ErrorCode WriteDeviceDescriptor(DeviceDescriptor& descriptor);

  /**
   * @brief 对置了服务请求的每个类调用 OnService / Call OnService on every class that
   *        requested service
   * @note 所有类，不论是否在当前配置中；先清请求再调用。调用者：设备核心，收到
   *       DeviceClass::SERVICE_EVENT 时。
   *       Every class, configured or not; the request is cleared before the call.
   *       Caller: the device core on DeviceClass::SERVICE_EVENT.
   */
  void ServiceClasses(bool in_isr);

  /// 把帧起始交给当前配置的每个类；调用者：设备核心 / Hand the start of frame to every
  /// class of the current configuration; caller: the device core
  void StartOfFrame(bool in_isr, uint16_t frame, uint8_t microframe);

 private:
  /// 一个配置的类列表 / Class list of one configuration
  struct Config
  {
    DeviceClass** classes = nullptr;  ///< 类指针表 / Class pointers
    size_t count = 0;                 ///< 类个数 / Number of classes
  };

  /// 当前配置的每个类 UnbindEndpoints，记为未配置 / UnbindEndpoints on every class
  /// of the current configuration and record unconfigured
  void Unbind(bool in_isr);

  /// 为每个类分配接口字符串号，构造时 / Assign interface string indices, at
  /// construction
  void RegisterInterfaceStrings();

  /// 生成一个接口字符串描述符 / Generate one interface string descriptor
  ErrorCode GenerateInterfaceString(uint8_t index, ConstRawData& data);

  EndpointPool& pool_;  ///< 端点池；构造时写 / Endpoint pool; set at construction

  /// @name 构造时固定 / Fixed at construction
  /// @{
  Config* configs_ = nullptr;        ///< 每个配置的类列表 / Class list per configuration
  uint8_t config_count_ = 0U;        ///< 配置个数 / Number of configurations
  DeviceClass** classes_ = nullptr;  ///< 去重后的类表 / Deduplicated class table
  size_t class_count_ = 0;           ///< 类个数 / Number of classes
  bool composite_ = false;           ///< 是否复合设备 / Whether composite
  uint8_t bm_attributes_ = CFG_BUS_POWERED;   ///< 配置 bmAttributes / bmAttributes
  uint8_t max_power_ = 50U;                   ///< 配置 bMaxPower / bMaxPower
  DescriptorStrings strings_;                 ///< 字符串描述符 / String descriptors
  const char** interface_strings_ = nullptr;  ///< 接口字符串表 / Interface strings
  size_t interface_string_count_ = 0;         ///< 接口字符串个数 / Their count
  /// 配置描述符缓冲，WriteConfigDescriptor 写 / Configuration descriptor buffer,
  /// written by WriteConfigDescriptor
  RawData config_buffer_{nullptr, 0};
  /// 接口字符串描述符缓冲 / Interface string descriptor buffer
  RawData string_buffer_{nullptr, 0};
  /// 由类的 GUID 生成的 WinUSB 能力；没有类要时为 nullptr / WinUSB capability made
  /// from the classes' GUIDs; nullptr when no class wants it
  BosCapability* winusb_cap_ = nullptr;
  BosManager bos_;  ///< BOS，能力集合固定 / The BOS, with a fixed capability set
  /// @}

  /**
   * @brief 当前配置值，0 表示未配置 / Current configuration value, 0 when unconfigured
   * @note 唯一记录"已配置"的地方。SetConfiguration 和 Unbind 在 owner 中写；
   *       GetConfiguration、FindClass*、StartOfFrame 在 owner 中读。
   *       The only record of "configured". Written by SetConfiguration and Unbind in
   *       the owner; read by GetConfiguration, FindClass* and StartOfFrame in the
   *       owner.
   */
  uint8_t configuration_ = 0U;
};

}  // namespace LibXR::USB
