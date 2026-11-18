#pragma once

#include <cstddef>
#include <cstdint>

#include "device_class.hpp"
#include "libxr_def.hpp"
#include "libxr_type.hpp"
#include "webusb.hpp"

namespace LibXR::USB
{

/// DFU 类请求码（DFU 1.1 表 3.2）/ DFU class request codes (DFU 1.1 table 3.2)
enum class DFURequest : uint8_t
{
  DETACH = 0x00,     ///< 进入 DFU 模式 / Enter DFU mode
  DNLOAD = 0x01,     ///< 下载一块 / Download one block
  UPLOAD = 0x02,     ///< 上传一块 / Upload one block
  GETSTATUS = 0x03,  ///< 读状态 / Get status
  CLRSTATUS = 0x04,  ///< 清错误 / Clear status
  GETSTATE = 0x05,   ///< 读状态号 / Get state
  ABORT = 0x06,      ///< 放弃当前会话 / Abort the session
};

/// DFU 协议状态（DFU 1.1 6.1.2）/ DFU protocol state (DFU 1.1 6.1.2)
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

/// DFU 状态码（DFU 1.1 6.1.2）/ DFU status code (DFU 1.1 6.1.2)
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

/// DFU 功能能力 / DFU functional capabilities
struct DFUCapabilities
{
  bool can_download = true;  ///< bitCanDnload / bitCanDnload
  bool can_upload = true;    ///< bitCanUpload / bitCanUpload
  /// bitManifestationTolerant：manifest 之后仍能响应请求 / bitManifestationTolerant:
  /// the device still answers requests after manifestation
  bool manifestation_tolerant = true;
  /// bitWillDetach：DETACH 后设备自行分离 / bitWillDetach: the device detaches by
  /// itself after DETACH
  bool will_detach = false;
  /// wDetachTimeOut，毫秒 / wDetachTimeOut in milliseconds
  uint16_t detach_timeout_ms = 1000U;
  /// wTransferSize：一次控制传输的最大字节数 / wTransferSize: largest bytes per
  /// control transfer
  uint16_t transfer_size = 1024U;
};

LIBXR_PACKED_BEGIN
/// DFU 功能描述符（DFU 1.1 4.1.3），9 字节 / DFU functional descriptor (DFU 1.1
/// 4.1.3), 9 bytes
struct DFUFunctionalDescriptor
{
  uint8_t bLength = 9U;              ///< 描述符长度 / Descriptor length
  uint8_t bDescriptorType = 0x21U;   ///< DFU 功能描述符 / DFU functional descriptor
  uint8_t bmAttributes = 0U;         ///< 能力位图 / Capability bitmap
  uint16_t wDetachTimeOut = 0U;      ///< DETACH 后等复位的毫秒数 / Wait after DETACH, ms
  uint16_t wTransferSize = 0U;       ///< 一次传输的最大字节数 / Largest transfer
  uint16_t bcdDFUVersion = 0x0110U;  ///< DFU 1.1 / DFU 1.1
};

/// GETSTATUS 的应答，6 字节 / GETSTATUS answer, 6 bytes
struct DFUStatusResponse
{
  uint8_t bStatus = 0U;                     ///< 状态码 / Status code
  uint8_t bwPollTimeout[3] = {0U, 0U, 0U};  ///< 轮询等待，毫秒，24 位小端 / Poll wait
  uint8_t bState = 0U;                      ///< 状态号 / State
  uint8_t iString = 0U;                     ///< 字符串号，0 为无 / String index, 0 none

  /// 写入毫秒值的低 24 位 / Store the low 24 bits of a millisecond value
  void SetPollTimeout(uint32_t timeout_ms)
  {
    bwPollTimeout[0] = static_cast<uint8_t>(timeout_ms & 0xFFU);
    bwPollTimeout[1] = static_cast<uint8_t>((timeout_ms >> 8U) & 0xFFU);
    bwPollTimeout[2] = static_cast<uint8_t>((timeout_ms >> 16U) & 0xFFU);
  }
};
LIBXR_PACKED_END

static_assert(sizeof(DFUFunctionalDescriptor) == 9U, "DFU functional descriptor");
static_assert(sizeof(DFUStatusResponse) == 6U, "DFU status response");

/**
 * @class DfuInterfaceClassBase
 * @brief 单接口、无端点的 DFU 类的公共部分 / Common part of the single-interface DFU
 *        classes without endpoints
 *
 * 一个接口（类 0xFE、子类 1），接口描述符之后紧跟 DFU 功能描述符；接口字符串、可选的
 * WebUSB 能力和 WinUSB GUID。不用任何端点：全部请求走 EP0。
 * One interface (class 0xFE, subclass 1) whose interface descriptor is followed by the
 * DFU functional descriptor; an interface string, an optional WebUSB capability and the
 * WinUSB GUID. No endpoint is used: every request goes through EP0.
 *
 * @note 除构造外只在设备的 owner 中运行。
 *       Apart from construction, runs in the device's owner only.
 */
class DfuInterfaceClassBase : public DeviceClass
{
 public:
  /// 默认的 WinUSB 设备接口 GUID / Default WinUSB device interface GUID
  static constexpr const char* DEFAULT_WINUSB_DEVICE_INTERFACE_GUID =
      "{4066E5F4-3B02-4B90-9475-12F770A7841B}";

 protected:
  /**
   * @brief 构造 / Construct
   * @param interface_string 接口字符串 / Interface string
   * @param protocol 接口协议：1 运行时，2 DFU 模式 / Interface protocol: 1 runtime,
   *        2 DFU mode
   * @param webusb_landing_page_url WebUSB 落地页；nullptr 不启用 / WebUSB landing page;
   *        nullptr disables it
   * @param webusb_vendor_code WebUSB 厂商请求码 / WebUSB vendor request code
   * @param winusb_device_interface_guid WinUSB GUID；nullptr 或空串不请求绑定 / WinUSB
   *        GUID; nullptr or "" requests no binding
   */
  DfuInterfaceClassBase(const char* interface_string, uint8_t protocol,
                        const char* webusb_landing_page_url, uint8_t webusb_vendor_code,
                        const char* winusb_device_interface_guid);

  /// @name DeviceClass 的实现 / DeviceClass implementation
  /// @{
  size_t GetInterfaceCount() const override { return 1U; }
  bool HasIAD() const override { return false; }
  size_t GetMaxConfigSize() const override { return sizeof(desc_block_); }
  const char* GetInterfaceString(size_t local_interface_index) const override;
  const char* GetWinUsbInterfaceGuid() const override;
  size_t GetBosCapabilityCount() override;
  BosCapability* GetBosCapability(size_t index) override;

  /**
   * @brief 写接口描述符和功能描述符 / Write the interface and functional descriptors
   * @return 总是 OK：不用端点，与速度无关 / Always OK: no endpoint, independent of the
   *         speed
   */
  ErrorCode WriteDescriptors(const EndpointPool& pool, Speed speed,
                             uint8_t first_interface, ConstRawData& block) override;

  /**
   * @brief 接口为接收者的 GET_DESCRIPTOR(0x21) / GET_DESCRIPTOR(0x21) to the interface
   * @note 功能描述符；其他 NOT_SUPPORT。
   *       The functional descriptor; others NOT_SUPPORT.
   */
  ErrorCode OnGetDescriptor(bool in_isr, const SetupPacket& setup,
                            ConstRawData& data) override;
  /// @}

  /**
   * @brief 是否本接口的类请求，且方向和长度符合请求码 / Whether this is a class
   *        request to this interface with the direction and length its code needs
   * @return OK；NOT_SUPPORT：不是类请求、接收者或 wIndex 不对；ARG_ERR：方向不对，或
   *         GETSTATUS 的 wLength 不是 6、GETSTATE 不是 1、DETACH 不是 0。
   *         OK; NOT_SUPPORT: not a class request, or wrong recipient or wIndex;
   *         ARG_ERR: wrong direction, or wLength not 6 for GETSTATUS, 1 for GETSTATE or
   *         0 for DETACH.
   * @note 调用者：两个派生类的 OnControlRequest。UPLOAD、GETSTATUS、GETSTATE 是 IN，其余
   *       是 OUT。
   *       Callers: the OnControlRequest of both derived classes. UPLOAD, GETSTATUS and
   *       GETSTATE are IN, the others OUT.
   */
  ErrorCode CheckClassRequest(const SetupPacket& setup) const;

  /// 功能描述符；派生类构造时写，之后只读 / Functional descriptor; written by the
  /// derived constructor, read-only afterwards
  DFUFunctionalDescriptor functional_{};

  /// 本接口号；派生类 BindEndpoints 写，CheckClassRequest 读 / This interface's
  /// number; written by the derived BindEndpoints, read by CheckClassRequest
  uint8_t interface_ = 0U;

 private:
  LIBXR_PACKED_BEGIN
  /// 接口描述符加功能描述符 / Interface descriptor plus functional descriptor
  struct DescriptorBlock
  {
    InterfaceDescriptor interface_desc;
    DFUFunctionalDescriptor functional;
  };
  LIBXR_PACKED_END

  /// @name 构造时固定 / Fixed at construction
  /// @{
  const char* interface_string_;              ///< 接口字符串 / Interface string
  const char* winusb_device_interface_guid_;  ///< WinUSB GUID
  uint8_t protocol_;                          ///< 接口协议 / Interface protocol
  WebUsb::WebUsbBosCapability webusb_cap_;    ///< WebUSB 能力 / WebUSB capability
  /// @}

  /// WriteDescriptors 写出的块 / Block written by WriteDescriptors
  DescriptorBlock desc_block_{};
};

}  // namespace LibXR::USB
