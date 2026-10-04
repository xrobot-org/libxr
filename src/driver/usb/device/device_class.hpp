#pragma once

#include <cstddef>
#include <cstdint>

#include "libxr_type.hpp"
#include "usb/core/desc_cfg.hpp"

namespace LibXR::USB
{
class DeviceComposition;
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

 protected:
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
    RawData read_data{nullptr, 0};  ///< OUT 数据阶段接收缓冲区（Host->Device）/ OUT data
                                    ///< stage buffer (Host->Device)
    ConstRawData write_data{nullptr, 0};  ///< IN 数据阶段发送数据（Device->Host）/ IN
                                          ///< data stage payload (Device->Host)
    bool read_zlp = false;   ///< 期望 STATUS OUT（arm OUT 等待 ZLP）/ Expect STATUS OUT
                             ///< (arm OUT for ZLP)
    bool write_zlp = false;  ///< 发送 STATUS IN（发送 ZLP）/ Send STATUS IN (send ZLP)

    RawData& OutData() { return read_data; }
    const RawData& OutData() const { return read_data; }

    ConstRawData& InData() { return write_data; }
    const ConstRawData& InData() const { return write_data; }

    bool& ExpectStatusOutZLP() { return read_zlp; }
    bool ExpectStatusOutZLP() const { return read_zlp; }

    bool& SendStatusInZLP() { return write_zlp; }
    bool SendStatusInZLP() const { return write_zlp; }
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

 private:
  // 这些辅助函数只在初始化期由 DeviceComposition 调用，不属于对外类接口。
  // These helpers are driven by DeviceComposition during initialization-time string
  // registration and are not part of the public class contract.
  void SetInterfaceStringBaseIndex(uint8_t string_index);

  friend class DeviceComposition;
  friend class DeviceCore;

  uint8_t interface_string_base_index_ =
      0u;  ///< 首个接口字符串索引 / First interface string index
};

}  // namespace LibXR::USB
