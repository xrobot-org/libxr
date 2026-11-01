#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

#include "device_events.hpp"
#include "libxr_type.hpp"
#include "usb/core/desc_cfg.hpp"
#include "usb/core/ep_pool.hpp"

namespace LibXR::USB
{
class DeviceComposition;
template <typename Capabilities>
class DeviceCore;

/**
 * @brief USB 设备类接口基类 / USB device class interface base
 *
 * 所有自定义 USB 类（HID/CDC/MSC 等）应派生自本类。
 * All custom USB classes (HID/CDC/MSC, etc.) should derive from this class.
 */
class DeviceClass : public ConfigDescriptorItem
{
 public:
  /**
   * @brief 默认构造 / Default constructor
   */
  DeviceClass() = default;

  /**
   * @brief 返回本类暴露的第 N 个接口字符串
   *        Return the string for the Nth local interface exposed by this class.
   *
   * @param local_interface_index 类内局部接口序号 / Class-local interface index
   * @return UTF-8 字符串，返回 nullptr 表示不提供 / UTF-8 string, or nullptr if unused
   */
  virtual const char* GetInterfaceString(size_t local_interface_index) const
  {
    UNUSED(local_interface_index);
    return nullptr;
  }

  /**
   * @brief 返回本类提供的 BOS capability 数量
   *        Return the number of BOS capabilities exposed by this class.
   */
  size_t GetBosCapabilityCount() override { return 0u; }

  /**
   * @brief 返回指定 BOS capability
   *        Return the BOS capability at the given index.
   */
  BosCapability* GetBosCapability(size_t index) override
  {
    UNUSED(index);
    return nullptr;
  }

  /**
   * @brief 本类需要 WinUSB 绑定时返回设备接口 GUID，否则返回 nullptr
   *        The device interface GUID when this class wants WinUSB to bind, else nullptr
   *
   * MS OS 2.0 描述符集由组合设备统一生成：只有一个功能的非复合设备用设备级形式，其余用
   * 配置/功能子集形式，类自己不需要关心。
   * The MS OS 2.0 descriptor set is built by the composition: a single function on a
   * non-composite device gets the device-level form and every other case the
   * configuration/function subset form, so the class does not have to care.
   */
  virtual const char* GetWinUsbInterfaceGuid() const { return nullptr; }
  virtual void OnService(bool in_isr) { UNUSED(in_isr); }
  virtual size_t GetControlReceiveCapacity() const { return 64U; }
  virtual bool WantsBusTime() const { return false; }
  virtual void OnBusTime(bool in_isr, const BusTime& time)
  {
    UNUSED(in_isr);
    UNUSED(time);
  }
  virtual void OnDeviceEvent(bool in_isr, DeviceEvent event, uint8_t endpoint = 0)
  {
    UNUSED(in_isr);
    UNUSED(event);
    UNUSED(endpoint);
  }

 protected:
  /// 请 owner 调用本类的 OnService；可在任何上下文调用 / Ask the owner to run OnService;
  /// any context
  void RequestClassService(bool in_isr)
  {
    service_pending_.store(1U, std::memory_order_release);
    if (service_pool_) service_pool_->PostControl(EndpointPool::CLASS_EVENT, in_isr);
  }

  /**
   * @brief 返回已分配的接口字符串索引
   *        Return the assigned USB string index for a local interface.
   *
   * @param local_interface_index 类内局部接口序号 / Class-local interface index
   * @return USB 字符串索引；0 表示未分配 / USB string index; 0 means unassigned
   */
  [[nodiscard]] uint8_t GetInterfaceStringIndex(size_t local_interface_index) const;

  /**
   * @brief 控制请求（Class/Vendor）处理结果 / Control request (Class/Vendor) handling
   * result
   *
   */
  struct ControlTransferResult
  {
    /// IN 数据阶段发送的数据（Device->Host）；OUT 数据阶段由核心收进自己的缓冲区，
    /// 完整后经 OnControlData 交给类 / IN data stage payload (Device->Host). OUT data is
    /// received into the core's own buffer and handed over by OnControlData.
    ConstRawData write_data{nullptr, 0};

    ConstRawData& InData() { return write_data; }
    const ConstRawData& InData() const { return write_data; }
  };

  /**
   * @brief 处理标准请求 GET_DESCRIPTOR（类特定描述符）
   *        Handle standard GET_DESCRIPTOR request (class-specific descriptors).
   *
   * @param in_isr   是否在 ISR / Whether in ISR context
   * @param bRequest 请求码 / Request code
   * @param wValue   wValue
   * @param wLength  wLength
   * @param out_data 输出：返回给主机的描述符数据（Device->Host）
   *                 Output: descriptor data to return (Device->Host)
   * @return 错误码 / Error code
   */
  virtual ErrorCode OnGetDescriptor(bool in_isr, uint8_t bRequest, uint16_t wValue,
                                    uint16_t wLength, ConstRawData& out_data)
  {
    UNUSED(in_isr);
    UNUSED(bRequest);
    UNUSED(wValue);
    UNUSED(wLength);
    UNUSED(out_data);
    return ErrorCode::NOT_SUPPORT;
  }

  /**
   * @brief 处理 Class-specific 请求（Setup stage）/ Handle class-specific request (Setup
   * stage)
   *
   * @param in_isr   是否在 ISR / Whether in ISR context
   * @param bRequest 请求码 / Request code
   * @param wValue   wValue
   * @param wLength  wLength
   * @param wIndex   wIndex
   * @param result   输出：控制传输结果 / Output: control transfer result
   * @return 错误码 / Error code
   */
  virtual ErrorCode OnClassRequest(bool in_isr, uint8_t bRequest, uint16_t wValue,
                                   uint16_t wLength, uint16_t wIndex,
                                   ControlTransferResult& result)
  {
    UNUSED(in_isr);
    UNUSED(bRequest);
    UNUSED(wValue);
    UNUSED(wLength);
    UNUSED(wIndex);
    UNUSED(result);
    return ErrorCode::NOT_SUPPORT;
  }

  /**
   * @brief 处理 Class request 数据阶段 / Handle class request data stage
   *
   * @param in_isr   是否在 ISR / Whether in ISR context
   * @param bRequest 请求码 / Request code
   * @param data     数据阶段数据 / Data stage payload
   * @return 错误码 / Error code
   *
   * @note 当 OnClassRequest 返回需要 OUT/IN data stage 时，数据阶段完成后回调此函数。
   *       When OnClassRequest requires an OUT/IN data stage, this callback is invoked
   * after completion.
   */
  virtual ErrorCode OnClassData(bool in_isr, uint8_t bRequest, LibXR::ConstRawData& data)
  {
    UNUSED(in_isr);
    UNUSED(bRequest);
    UNUSED(data);
    return ErrorCode::NOT_SUPPORT;
  }

  /**
   * @brief 类请求的 IN 数据阶段在 STATUS OUT 完成后回调
   *        Called after the STATUS OUT completes for a Class IN data request.
   *
   * @param in_isr   是否在 ISR / Whether in ISR context
   * @param bRequest 请求码 / Request code
   */
  virtual void OnClassInDataStatusComplete(bool in_isr, uint8_t bRequest)
  {
    UNUSED(in_isr);
    UNUSED(bRequest);
  }

  /**
   * @brief 处理 Vendor request（Setup stage）/ Handle vendor request (Setup stage)
   *
   * @param in_isr   是否在 ISR / Whether in ISR context
   * @param bRequest 请求码 / Request code
   * @param wValue   wValue
   * @param wLength  wLength
   * @param wIndex   wIndex
   * @param result   输出：控制传输结果 / Output: control transfer result
   * @return 错误码 / Error code
   */
  virtual ErrorCode OnVendorRequest(bool in_isr, uint8_t bRequest, uint16_t wValue,
                                    uint16_t wLength, uint16_t wIndex,
                                    ControlTransferResult& result)
  {
    UNUSED(in_isr);
    UNUSED(bRequest);
    UNUSED(wValue);
    UNUSED(wLength);
    UNUSED(wIndex);
    UNUSED(result);
    return ErrorCode::NOT_SUPPORT;
  }

  virtual ErrorCode OnControlRequest(bool in_isr, const SetupPacket& setup,
                                     ControlTransferResult& result)
  {
    if ((setup.bmRequestType & REQ_TYPE_MASK) ==
        static_cast<uint8_t>(RequestType::VENDOR))
      return OnVendorRequest(in_isr, setup.bRequest, setup.wValue, setup.wLength,
                             setup.wIndex, result);
    return OnClassRequest(in_isr, setup.bRequest, setup.wValue, setup.wLength,
                          setup.wIndex, result);
  }
  virtual ErrorCode OnControlData(bool in_isr, const SetupPacket& setup,
                                  ConstRawData& data)
  {
    if ((setup.bmRequestType & REQ_DIRECTION_MASK) != 0U) return ErrorCode::OK;
    return OnClassData(in_isr, setup.bRequest, data);
  }
  virtual void OnControlComplete(bool in_isr, const SetupPacket& setup)
  {
    if ((setup.bmRequestType & REQ_DIRECTION_MASK) != 0U)
      OnClassInDataStatusComplete(in_isr, setup.bRequest);
  }
  virtual void OnControlAbort(bool in_isr, const SetupPacket& setup)
  {
    UNUSED(in_isr);
    UNUSED(setup);
  }

 private:
  // 这些辅助函数只在初始化期由 DeviceComposition 调用，不属于对外类接口。
  // These helpers are driven by DeviceComposition during initialization-time string
  // registration and are not part of the public class contract.
  void SetInterfaceStringBaseIndex(uint8_t string_index);

  EndpointPool* service_pool_ = nullptr;  // Immutable after composition construction.
  std::atomic<uint32_t> service_pending_{0};
  friend class DeviceComposition;
  template <typename Capabilities>
  friend class DeviceCore;

  uint8_t interface_string_base_index_ =
      0u;  ///< 首个接口字符串索引 / First interface string index
};

}  // namespace LibXR::USB
