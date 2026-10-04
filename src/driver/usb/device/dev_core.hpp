#pragma once

#include <cstddef>
#include <cstdint>
#include <initializer_list>

#include "device_class.hpp"
#include "device_composition.hpp"
#include "libxr_type.hpp"
#include "usb/core/core.hpp"
#include "usb/core/ep_pool.hpp"

namespace LibXR::USB
{

/**
 * @brief USB 设备协议栈核心：EP0 控制传输、描述符、配置、标准/类/厂商请求
 *        USB device core: EP0 control transfer, descriptors, configuration,
 *        and standard/class/vendor requests.
 */
class DeviceCore
{
 public:
  /**
   * @brief 控制传输上下文 / Control transfer context
   *
   * 说明：UNKNOWN 表示当前 EP0 传输上下文为空或未定义。
   * Note: UNKNOWN means the current EP0 transfer context is idle or undefined.
   */
  enum Context : uint8_t
  {
    UNKNOWN = 0,          ///< 未知 / Unknown
    SETUP_BEFORE_STATUS,  ///< Setup handled, before STATUS IN ZLP is armed
    STATUS_IN_ARMED,      ///< STATUS IN ZLP armed, but not yet completed
    DATA_OUT,             ///< OUT data stage
    STATUS_OUT,           ///< OUT status stage
    DATA_IN,              ///< IN data stage
    STATUS_IN_COMPLETE,   ///< IN status stage completed
    ZLP                   ///< ZLP stage marker
  };

  /**
   * @brief 构造函数 / Constructor
   *
   * @param ep_pool     端点池 / Endpoint pool
   * @param spec        USB 规范版本 / USB specification
   * @param speed       设备速度 / Device speed
   * @param packet_size EP0 包长 / EP0 packet size
   * @param vid         Vendor ID
   * @param pid         Product ID
   * @param bcd         设备版本号（BCD）/ Device release number (BCD)
   * @param lang_list   字符串语言包列表 / String language pack list
   * @param configs     配置列表（每个子列表为一个 configuration）
   *                   Config list (each sub-list is one configuration)
   * @param uid         UID 原始数据（可选）/ UID raw data (optional)
   */
  DeviceCore(
      EndpointPool& ep_pool, USBSpec spec, Speed speed,
      DeviceDescriptor::PacketSize0 packet_size, uint16_t vid, uint16_t pid, uint16_t bcd,
      const std::initializer_list<const DescriptorStrings::LanguagePack*>& lang_list,
      const std::initializer_list<const std::initializer_list<ConfigDescriptorItem*>>&
          configs,
      ConstRawData uid = {nullptr, 0});

  /**
   * @brief 初始化 / Initialize
   *
   * @param in_isr 是否在 ISR / Whether in ISR context
   */
  virtual void Init(bool in_isr);

  /**
   * @brief 反初始化 / Deinitialize
   *
   * @param in_isr 是否在 ISR / Whether in ISR context
   */
  virtual void Deinit(bool in_isr);

  /**
   * @brief 启动设备（由子类实现）/ Start device (implemented by derived class)
   *
   * @param in_isr 是否在 ISR / Whether in ISR context
   */
  virtual void Start(bool in_isr) = 0;

  /**
   * @brief 停止设备（由子类实现）/ Stop device (implemented by derived class)
   *
   * @param in_isr 是否在 ISR / Whether in ISR context
   */
  virtual void Stop(bool in_isr) = 0;

  /**
   * @brief 处理 Setup 包 / Handle Setup packet
   * @param in_isr 是否在 ISR / Whether in ISR context
   * @param setup  Setup 包 / Setup packet
   */
  void OnSetupPacket(bool in_isr, const SetupPacket* setup);

  /**
   * @brief 查询设备核心是否已经完成初始化 / Query whether the device core is
   *        initialized
   * @return true：已初始化；false：未初始化 / true: initialized; false: not
   *         initialized
   */
  [[nodiscard]] bool IsInited() const { return state_.inited; }

 protected:
  /**
   * @brief 设置设备地址（由子类实现）
   *        Set device address (implemented by derived class).
   *
   * @param address 设备地址 / Device address
   * @param state   当前上下文 / Current context
   * @return 错误码 / Error code
   */
  virtual ErrorCode SetAddress(uint8_t address, Context state) = 0;

  /**
   * @brief 启用远程唤醒 / Enable remote wakeup
   */
  virtual void EnableRemoteWakeup() {}

  /**
   * @brief 禁用远程唤醒 / Disable remote wakeup
   */
  virtual void DisableRemoteWakeup() {}

  /**
   * @brief 远程唤醒是否启用 / Whether remote wakeup is enabled
   * @return true：已启用；false：未启用 / true: enabled; false: disabled
   */
  virtual bool IsRemoteWakeupEnabled() const { return false; }

  /**
   * @brief 获取设备速度 / Get device speed
   * @return 设备速度 / Device speed
   */
  [[nodiscard]] Speed GetSpeed() const;

 private:
  static void OnEP0OutCompleteStatic(bool in_isr, DeviceCore* self,
                                     LibXR::ConstRawData& data);
  static void OnEP0InCompleteStatic(bool in_isr, DeviceCore* self,
                                    LibXR::ConstRawData& data);

  static bool IsValidUSBCombination(USBSpec spec, Speed speed,
                                    DeviceDescriptor::PacketSize0 packet_size);

  void OnEP0OutComplete(bool in_isr, LibXR::ConstRawData& data);
  void OnEP0InComplete(bool in_isr, LibXR::ConstRawData& data);

  void ReadZLP(Context context = Context::ZLP);
  void WriteZLP(Context context = Context::ZLP);
  void ResetControlTransferState();

  void DevWriteEP0Data(LibXR::ConstRawData data, size_t packet_max_length,
                       size_t request_size = 0, bool early_read_zlp = false);
  void DevReadEP0Data(LibXR::RawData data, size_t packet_max_length);
  void ArmStatusOutIfNeeded();

  ErrorCode ProcessStandardRequest(bool in_isr, const SetupPacket*& setup,
                                   RequestDirection direction, Recipient recipient);

  ErrorCode RespondWithStatus(const SetupPacket* setup, Recipient recipient);
  ErrorCode ClearFeature(const SetupPacket* setup, Recipient recipient);
  ErrorCode ApplyFeature(const SetupPacket* setup, Recipient recipient);
  ErrorCode SendDescriptor(bool in_isr, const SetupPacket* setup, Recipient recipient);
  ErrorCode PrepareAddressChange(uint16_t address);
  ErrorCode SwitchConfiguration(uint16_t value, bool in_isr);
  ErrorCode SendConfiguration();

  void StallControlEndpoint();
  void ClearControlEndpointStall();
  void ResetClassRequestState();

  ErrorCode ProcessClassRequest(bool in_isr, const SetupPacket* setup,
                                RequestDirection direction, Recipient recipient);

  ErrorCode ProcessVendorRequest(bool in_isr, const SetupPacket*& setup,
                                 RequestDirection direction, Recipient recipient);

 private:
  DeviceComposition composition_;  ///< USB 组合管理器 / USB composition manager
  DeviceDescriptor device_desc_;   ///< 设备描述符 / Device descriptor

  struct
  {
    EndpointPool& pool;        ///< 端点池引用 / Endpoint pool reference
    Endpoint* in0 = nullptr;   ///< EP0 IN 端点 / EP0 IN endpoint
    Endpoint* out0 = nullptr;  ///< EP0 OUT 端点 / EP0 OUT endpoint
    LibXR::Callback<LibXR::ConstRawData&> ep0_in_cb;  ///< EP0 IN 回调 / EP0 IN callback
    LibXR::Callback<LibXR::ConstRawData&>
        ep0_out_cb;  ///< EP0 OUT 回调 / EP0 OUT callback
  } endpoint_;

  struct
  {
    bool inited = false;                    ///< 是否已初始化 / Whether initialized
    Speed speed = Speed::FULL;              ///< 设备速度 / Device speed
    Context in0 = Context::UNKNOWN;         ///< EP0 IN 上下文 / EP0 IN context
    Context out0 = Context::UNKNOWN;        ///< EP0 OUT 上下文 / EP0 OUT context
    ConstRawData write_remain{nullptr, 0};  ///< IN 剩余待发送 / Remaining IN payload
    RawData read_remain{nullptr, 0};        ///< OUT 剩余待接收 / Remaining OUT buffer
    uint8_t pending_addr = 0xFF;            ///< 待生效地址 / Pending address
    uint8_t* out0_buffer = nullptr;         ///< EP0 OUT 缓冲区 / EP0 OUT buffer
    bool need_write_zlp = false;            ///< 是否需要发送 ZLP / Whether to send ZLP
    bool status_out_armed =
        false;  ///< STATUS OUT 已经预先挂起 / STATUS OUT already armed
  } state_;

  struct
  {
    bool write = false;  ///< 是否存在 IN 数据阶段 / Whether IN data stage exists
    bool read = false;   ///< 是否存在 OUT 数据阶段 / Whether OUT data stage exists
    bool class_in_data_status_pending =
        false;  ///< class IN data 阶段后等待 STATUS OUT / Waiting for STATUS OUT after a
                ///< class IN data stage
    DeviceClass* class_ptr = nullptr;  ///< 当前处理类 / Current class handler
    uint8_t b_request = 0;             ///< 当前请求码 / Current request code
    ConstRawData data{nullptr, 0};     ///< 数据阶段数据 / Data stage payload
  } class_req_;
};

}  // namespace LibXR::USB
