#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <initializer_list>

#include "device_class.hpp"
#include "libxr_type.hpp"
#include "usb/core/bos.hpp"
#include "usb/core/desc_cfg.hpp"
#include "usb/core/desc_str.hpp"
#include "usb/core/ep_pool.hpp"
#include "winusb_msos20.hpp"

namespace LibXR::USB
{
/**
 * @brief USB 组合管理器：class 列表、字符串、BOS、配置切换
 *        USB composition manager: class list, strings, BOS, and configuration switching.
 */
class DeviceComposition
{
  /**
   * @brief 单个 configuration 的扁平 item 表
   *        Flattened item table for one configuration.
   */
  struct ConfigItems
  {
    ConfigDescriptorItem** items = nullptr;  ///< 配置项指针表 / Item pointer table
    size_t item_num = 0;                     ///< 配置项数量 / Item count
  };

 public:
  /**
   * @brief 主机是否已选中非 0 配置，可在任意上下文读取
   *        Whether the host has selected a non-zero configuration; readable from any
   *        context
   */
  bool IsConfigured() const { return configured_.load(std::memory_order_acquire); }

  /**
   * @brief 构造 USB 组合管理器
   *        Construct the USB composition manager.
   *
   * 负责把 class/config/string/BOS 相关静态信息整理成 DeviceCore 可直接使用的形态。
   * This flattens class/config/string/BOS metadata into the shape consumed by DeviceCore.
   */
  DeviceComposition(
      EndpointPool& endpoint_pool,
      const std::initializer_list<const DescriptorStrings::LanguagePack*>& lang_list,
      const std::initializer_list<const std::initializer_list<ConfigDescriptorItem*>>&
          configs,
      ConstRawData uid = {nullptr, 0}, uint8_t bmAttributes = CFG_BUS_POWERED,
      uint8_t bMaxPower = 50);

  DeviceComposition(const DeviceComposition&) = delete;
  DeviceComposition& operator=(const DeviceComposition&) = delete;
  DeviceComposition(DeviceComposition&&) = delete;
  DeviceComposition& operator=(DeviceComposition&&) = delete;

  /**
   * @brief 初始化当前 configuration 的运行态资源
   *        Initialize runtime resources for the active configuration.
   */
  void Init(bool in_isr);

  /**
   * @brief 释放当前 configuration 的运行态资源
   *        Release runtime resources for the active configuration.
   */
  void Deinit(bool in_isr);

  /**
   * @brief 切换到指定 configuration value
   *        Switch to the given configuration value.
   *
   * 第一步校验并解绑当前配置；端点停止确认之后第二步绑定新配置。
   * The first step validates and unbinds the current configuration; after the endpoint
   * stops are confirmed, the second step binds the new one.
   */
  ErrorCode BeginSwitchConfig(size_t index, bool in_isr);

  /**
   * @brief 切换配置的第二步：绑定新配置并使能数据端点
   *        Second step of a configuration switch: bind the new configuration and enable
   *        the data endpoints.
   *
   * @param index configuration value，0 表示未配置 / Configuration value, 0 means
   * unconfigured
   * @param in_isr 是否在 ISR / Whether in ISR context
   * @return OK；端点规划无效时为 INIT_ERR（此时已回到未配置状态）/ OK; INIT_ERR when the
   * endpoint plan is invalid (the state falls back to unconfigured)
   */
  ErrorCode FinishSwitchConfig(size_t index, bool in_isr);

  /**
   * @brief 生成当前 configuration 描述符缓存
   *        Build the current configuration descriptor cache.
   */
  ErrorCode BuildConfigDescriptor();

  /// 为 speeds 中的每个速度和每个 configuration 预先生成并缓存配置描述符，仅在初始化时
  /// 分配内存；已准备好时直接返回 OK / Build and cache the configuration descriptor of
  /// every configuration for each speed in speeds; memory is allocated only at
  /// initialization time; returns OK at once when already prepared
  /// @param speeds SpeedBit 编码的速度位图 / Speed bitmap encoded with SpeedBit
  /// @param in_isr 是否在 ISR / Whether in ISR context
  /// @return OK；某个配置的端点规划无效时为 INIT_ERR / OK; INIT_ERR when the endpoint
  /// plan of some configuration is invalid
  ErrorCode PrepareDescriptors(uint32_t speeds, bool in_isr);

  /// 取 PrepareDescriptors 缓存的配置描述符 / Get a configuration descriptor cached by
  /// PrepareDescriptors
  /// @param index 从 0 开始的 configuration 序号（configuration value 减 1）/ Zero-based
  /// configuration index (configuration value minus 1)
  /// @param speed 总线速度 / Bus speed
  /// @return 描述符数据；未缓存或参数越界时为空 / The descriptor data; empty when not
  /// cached or out of range
  ConstRawData GetConfigDescriptor(size_t index, Speed speed) const;

  /// 所有类要求的控制 OUT 数据阶段缓冲区大小的最大值，至少 64 字节 / Largest control OUT
  /// data-stage buffer size required by any class, at least 64 bytes
  size_t ControlReceiveCapacity() const;

  /// 取某个配置中的第 item 个描述符项，不做越界检查 / Get the item-th descriptor item of
  /// a configuration; not range checked
  /// @param config 从 0 开始的 configuration 序号 / Zero-based configuration index
  /// @param item 配置内的项序号，小于 ConfigItemCount / Item index inside the
  /// configuration, below ConfigItemCount
  ConfigDescriptorItem* ConfigItem(size_t config, size_t item) const
  {
    return items_[config].items[item];
  }
  /// 某个配置的描述符项数量 / Number of descriptor items in a configuration
  /// @param config 从 0 开始的 configuration 序号 / Zero-based configuration index
  size_t ConfigItemCount(size_t config) const { return items_[config].item_num; }

  /// 组合中的设备类数量 / Number of device classes in the composition
  size_t ClassCount() const { return class_count_; }
  /// 对应的 BOS 平台能力；没有类需要 WinUSB 时为 nullptr / The matching BOS platform
  /// capability; nullptr when no class wants WinUSB
  BosCapability* WinUsbBosCapability() const { return winusb_cap_; }

  /// 按序号取设备类，不做越界检查 / Get a device class by index; not range checked
  /// @param index 小于 ClassCount 的序号 / Index below ClassCount
  DeviceClass* ClassAt(size_t index) const { return classes_[index]; }

  /// 向设备类转发设备事件：RESET 发给所有类，其他事件只发给当前配置中的类；endpoint 非 0
  /// 时只发给拥有该端点的类 / Forward a device event to the classes: RESET reaches every
  /// class, other events only the classes of the current configuration; a non-zero
  /// endpoint restricts delivery to the class that owns it
  /// @param in_isr 是否在 ISR / Whether in ISR context
  /// @param event 事件类型 / Event type
  /// @param endpoint 端点地址，0 表示不限定 / Endpoint address, 0 for no restriction
  void Notify(bool in_isr, DeviceEvent event, uint8_t endpoint = 0);

  /// 向当前配置中需要帧节拍的类发送 OnFrameTick / Send OnFrameTick to the classes of the
  /// current configuration that need it
  /// @param in_isr 是否在 ISR / Whether in ISR context
  /// @param time 帧节拍信息 / Frame tick information
  void NotifyFrameTick(bool in_isr, const FrameTick& time);

  /// 任一类是否需要帧节拍 / Whether any class needs frame ticks
  bool NeedsFrameTick() const;

  /**
   * @brief 获取当前 configuration 描述符缓存
   *        Get the current configuration descriptor cache.
   */
  [[nodiscard]] RawData GetConfigDescriptor() const;

  /**
   * @brief 获取字符串描述符（含 interface string）
   *        Get a string descriptor, including runtime-generated interface strings.
   */
  ErrorCode GetStringDescriptor(uint8_t string_index, uint16_t lang, ConstRawData& data);

  /**
   * @brief 是否为复合设备 / Whether this is a composite device
   */
  [[nodiscard]] bool IsComposite() const;

  /**
   * @brief 用当前配置覆盖 device descriptor 的类字段
   *        Try to override device-descriptor class fields from the current configuration.
   */
  ErrorCode TryOverrideDeviceDescriptor(DeviceDescriptor& descriptor);

  /**
   * @brief 配置数量 / Number of configurations
   */
  [[nodiscard]] size_t GetConfigNum() const;

  /**
   * @brief 当前 configuration value / Current configuration value
   */
  [[nodiscard]] size_t GetCurrentConfig() const;

  /**
   * @brief 设备 GET_STATUS 返回值 / Device-level GET_STATUS value
   */
  [[nodiscard]] uint16_t GetDeviceStatus() const;

  /**
   * @brief 按接口号查找所属 class
   *        Find the owning class by interface number.
   */
  [[nodiscard]] DeviceClass* FindClassByInterfaceNumber(size_t index) const;

  /**
   * @brief 按端点地址查找所属 class
   *        Find the owning class by endpoint address.
   */
  [[nodiscard]] DeviceClass* FindClassByEndpointAddress(uint8_t addr) const;

 private:
  /**
   * @brief 获取当前激活 configuration 的扁平 item 表
   *        Get the flattened item table of the active configuration.
   */
  [[nodiscard]] const ConfigItems& CurrentConfigItems() const;

  /**
   * @brief 绑定当前配置的全部端点 / Bind all endpoints for the active configuration
   */
  void BindEndpoints(bool in_isr);

  /**
   * @brief 解绑当前配置的全部端点 / Unbind all endpoints for the active configuration
   */
  void UnbindEndpoints(bool in_isr);

  bool ContainsActiveClass(const DeviceClass* item) const;

  /**
   * @brief 为所有 class 分配并登记 interface string 索引
   *        Allocate and register interface-string indices for all classes.
   */
  void RegisterInterfaceStrings();

  /**
   * @brief 运行时生成 interface string 描述符
   *        Generate one interface-string descriptor at runtime.
   */
  ErrorCode GenerateInterfaceString(uint8_t string_index, ConstRawData& data);

  std::atomic<bool> configured_{
      false};  ///< 是否已进入非 0 配置态 / Whether a non-zero config is active
  bool ep_assigned_ = false;  ///< 端点是否已绑定 / Whether endpoints are assigned

  EndpointPool& endpoint_pool_;  ///< 端点池引用 / Endpoint pool reference
  uint8_t current_cfg_ = 0;      ///< 当前配置索引 / Current configuration index
  uint8_t i_configuration_ = 0;  ///< 配置字符串索引 / Configuration string index
  uint8_t bm_attributes_ = CFG_BUS_POWERED;  ///< 配置属性 / bmAttributes

  const bool composite_ = false;  ///< 是否为复合设备 / Whether composite device
  WinUsbMsOs20::WinUsbDescriptorSet winusb_set_;  ///< MS OS 2.0 描述符集 / descriptor set
  WinUsbMsOs20::MsOs20BosCapability* winusb_cap_ = nullptr;  ///< 对应的 BOS 能力 / its
                                                             ///< BOS capability
  void BuildWinUsbDescriptorSet();
  const size_t config_num_ = 0;      ///< 配置数量 / Configuration count
  ConfigItems* items_ = nullptr;     ///< 配置项集合 / Configuration item set
  DeviceClass** classes_ = nullptr;  ///< 唯一 class 表 / Unique class table
  size_t class_count_ = 0;           ///< 唯一 class 数量 / Unique class count

  DescriptorStrings strings_;  ///< 字符串描述符管理 / String descriptor manager
  const char** interface_strings_ =
      nullptr;  ///< 接口字符串源表 / Interface string source table
  size_t interface_string_count_ =
      0;  ///< 接口字符串总数量 / Total interface string count
  RawData interface_string_buffer_{
      nullptr, 0};  ///< 临时字符串描述符缓冲区 / Temp interface string descriptor buffer
  struct DescriptorCache
  {
    RawData data{nullptr, 0};
  };
  DescriptorCache* descriptors_ = nullptr;
  bool descriptors_prepared_ = false;
  ConfigDescriptor config_desc_;  ///< 配置描述符构造器 / Configuration descriptor builder
};

}  // namespace LibXR::USB
