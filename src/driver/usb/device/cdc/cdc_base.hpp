#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

#include "device_class.hpp"
#include "ep.hpp"
#include "libxr_cb.hpp"
#include "libxr_def.hpp"
#include "uart.hpp"

namespace LibXR::USB
{

/**
 * @class CDCBase
 * @brief USB CDC ACM（抽象控制模型）设备类的公共部分 / Common part of a USB CDC ACM
 *        (Abstract Control Model) device class
 *
 * 两个接口（通信、数据）、三个端点（数据 IN、数据 OUT 批量，通知 IN 中断），类请求
 * SET/GET_LINE_CODING、SET_CONTROL_LINE_STATE、SEND_BREAK，以及 SERIAL_STATE 通知。数据
 * 端点上的收发由派生类在 OnDataInComplete、OnDataOutComplete 中实现。
 * Two interfaces (communication, data), three endpoints (bulk data IN and OUT, interrupt
 * notification IN), the class requests SET/GET_LINE_CODING, SET_CONTROL_LINE_STATE and
 * SEND_BREAK, and the SERIAL_STATE notification. Transfers on the data endpoints are the
 * derived class's job, in OnDataInComplete and OnDataOutComplete.
 *
 * @note 除注明"任何上下文"的函数外，本类的函数都只在设备的 owner 中运行。端点指针非空
 *       当且仅当已绑定；它们只由 owner 写和读。
 *       Except for the functions marked "any context", everything here runs in the
 *       device's owner. The endpoint pointers are non-null exactly while bound; only the
 *       owner writes and reads them.
 */
class CDCBase : public DeviceClass
{
 public:
  /// 通信接口的默认字符串 / Default string of the communication interface
  static constexpr const char* DEFAULT_CONTROL_INTERFACE_STRING = "XRUSB CDC Control";
  /// 数据接口的默认字符串 / Default string of the data interface
  static constexpr const char* DEFAULT_DATA_INTERFACE_STRING = "XRUSB CDC Data";

  /**
   * @brief 构造 / Construct
   * @param data_in_ep_num 数据 IN 端点号 / Data IN endpoint number
   * @param data_out_ep_num 数据 OUT 端点号 / Data OUT endpoint number
   * @param comm_ep_num 通知 IN 端点号 / Notification IN endpoint number
   * @param control_interface_string 通信接口字符串 / Communication interface string
   * @param data_interface_string 数据接口字符串 / Data interface string
   * @note 分配三个端点回调（Callback::Create），只在构造时分配。
   *       Allocates the three endpoint callbacks (Callback::Create), at construction
   *       only.
   */
  CDCBase(Endpoint::EPNumber data_in_ep_num, Endpoint::EPNumber data_out_ep_num,
          Endpoint::EPNumber comm_ep_num,
          const char* control_interface_string = DEFAULT_CONTROL_INTERFACE_STRING,
          const char* data_interface_string = DEFAULT_DATA_INTERFACE_STRING);

  /// 主机是否置了 DTR；任何上下文 / Whether the host set DTR; any context
  bool IsDtrSet() const;
  /// 主机是否置了 RTS；任何上下文 / Whether the host set RTS; any context
  bool IsRtsSet() const;

  /**
   * @brief 请求发送一次 SERIAL_STATE 通知 / Ask for one SERIAL_STATE notification
   * @return 总是 OK / Always OK
   * @note 任何上下文。只记下请求并 RequestService，通知由 owner 在 OnService 或通知端点
   *       的完成回调里发出；没有空闲槽时留到下一次。未绑定时请求保留到绑定后的下一次
   *       服务。
   *       Any context. Records the request and calls RequestService; the owner sends
   *       the notification in OnService or in the notification endpoint's completion,
   *       keeping it for the next one when no slot is free. While unbound the request
   *       waits for the next service after a bind.
   */
  ErrorCode SendSerialState(bool in_isr);

  /**
   * @brief 设置 SET_CONTROL_LINE_STATE 回调（DTR，RTS）/ Set the
   *        SET_CONTROL_LINE_STATE callback (DTR, RTS)
   * @note 在设备启动前设置；回调在 owner 中运行。
   *       Set before the device starts; the callback runs in the owner.
   */
  void SetOnSetControlLineStateCallback(Callback<bool, bool> cb);

  /**
   * @brief 设置 SET_LINE_CODING 回调（生效的 UART 配置）/ Set the SET_LINE_CODING
   *        callback (the UART configuration in effect)
   * @note 在设备启动前设置；回调在 owner 中运行。
   *       Set before the device starts; the callback runs in the owner.
   */
  void SetOnSetLineCodingCallback(Callback<UART::Configuration> cb);

 protected:
  LIBXR_PACKED_BEGIN
  /// CDC PSTN 6.3.11 线路编码，7 字节 / CDC PSTN 6.3.11 line coding, 7 bytes
  struct CDCLineCoding
  {
    uint32_t dwDTERate;   ///< 波特率 / Baud rate
    uint8_t bCharFormat;  ///< 停止位：0=1，1=1.5，2=2 / Stop bits: 0=1, 1=1.5, 2=2
    /// 校验：0 无，1 奇，2 偶，3 Mark，4 Space / Parity: 0 none, 1 odd, 2 even,
    /// 3 mark, 4 space
    uint8_t bParityType;
    uint8_t bDataBits;  ///< 数据位：5、6、7、8、16 / Data bits: 5, 6, 7, 8 or 16
  };
  LIBXR_PACKED_END
  static_assert(sizeof(CDCLineCoding) == 7U, "CDC line coding is 7 bytes");

  /// @name DeviceClass 的实现 / DeviceClass implementation
  /// @{
  size_t GetInterfaceCount() const override { return 2U; }
  bool HasIAD() const override { return true; }
  size_t GetMaxConfigSize() const override { return sizeof(desc_block_); }
  const char* GetInterfaceString(size_t local_interface_index) const override;

  /**
   * @brief 写 IAD、两个接口、功能描述符和三个端点描述符 / Write the IAD, both
   *        interfaces, the functional descriptors and the three endpoint descriptors
   * @return OK；NOT_SUPPORT：某个端点在该速度下查询得 0。
   *         OK; NOT_SUPPORT: an endpoint's query returned 0 at that speed.
   */
  ErrorCode WriteDescriptors(const EndpointPool& pool, Speed speed,
                             uint8_t first_interface, ConstRawData& block) override;

  /**
   * @brief 取三个端点、Configure、比较包长、装回调 / Take the three endpoints,
   *        configure them, compare the sizes, install the callbacks
   * @return OK；NOT_FOUND：Get 失败；CHECK_ERR：读回的包长与描述符不同；其他：Configure
   *         的错误。失败时已取的端点留在成员里，由随后的 UnbindEndpoints 收回。
   *         OK; NOT_FOUND: a Get failed; CHECK_ERR: a size read back differs from the
   *         descriptor; otherwise the error of Configure. On failure the endpoints taken
   *         stay in the members and the following UnbindEndpoints takes them back.
   * @note 派生类重写时先调用本函数，OK 后再挂接收、开始发送。
   *       A derived override calls this first and arms receives or starts sending only
   *       on OK.
   */
  [[nodiscard]] ErrorCode BindEndpoints(EndpointPool& pool, uint8_t first_interface,
                                        bool in_isr) override;

  /**
   * @brief 关闭并交回已取的端点，清 DTR/RTS / Close and return the endpoints taken,
   *        clear DTR/RTS
   * @note 先把指针置空再 Close，所以 Close 交付的完成回调看到"未绑定"，什么也不做。
   *       派生类重写时先取出要用的端点指针，再调用本函数。
   *       The pointer is cleared before Close, so the completions Close delivers see
   *       "unbound" and do nothing. A derived override reads the endpoint pointers it
   *       needs before calling this.
   */
  void UnbindEndpoints(EndpointPool& pool, bool in_isr) override;

  /// 三个端点地址之一；由端点号算出，不需要已绑定 / One of the three endpoint
  /// addresses; computed from the numbers, no bind needed
  bool OwnsEndpoint(uint8_t ep_addr) const override;

  /**
   * @brief 类请求的 SETUP 阶段 / SETUP stage of a class request
   * @return 只接受接收者为通信接口的四个 CDC 请求，方向和长度必须与请求相符，否则
   *         ARG_ERR 或 NOT_SUPPORT（核心 STALL）。
   *         Only the four CDC requests whose recipient is the communication interface,
   *         with the direction and length the request needs; otherwise ARG_ERR or
   *         NOT_SUPPORT (the core stalls).
   * @note SET_CONTROL_LINE_STATE 在这里生效；GET_LINE_CODING 的应答是当前线路编码的
   *       一份拷贝，在下一次 GET_LINE_CODING 之前不变。
   *       SET_CONTROL_LINE_STATE takes effect here; the GET_LINE_CODING reply is a copy
   *       of the current line coding that stays unchanged until the next
   *       GET_LINE_CODING.
   */
  ErrorCode OnControlRequest(bool in_isr, const SetupPacket& setup,
                             ConstRawData& in_data) override;

  /**
   * @brief SET_LINE_CODING 的数据阶段 / Data stage of SET_LINE_CODING
   * @return OK：只查 7 字节长度，取值不查（CDC PSTN 6.3.10）；ARG_ERR：长度不对。
   *         OK: only the 7-byte length is checked, not the values (CDC PSTN 6.3.10);
   *         ARG_ERR: wrong length.
   */
  ErrorCode OnControlData(bool in_isr, const SetupPacket& setup,
                          ConstRawData data) override;

  /// 发出待发的 SERIAL_STATE；派生类重写时调用本函数 / Send a pending SERIAL_STATE; a
  /// derived override calls this
  void OnService(bool in_isr) override;
  /// @}

  /// @name 派生类实现 / Implemented by the derived class
  /// @{

  /**
   * @brief 数据 OUT 端点的完成 / Completion of the data OUT endpoint
   * @note 只在已绑定时调用。OK 时 completion.slot 借给派生类，用完 ReleaseSlot。
   *       Called only while bound. On OK completion.slot is lent to the derived class,
   *       which calls ReleaseSlot when done.
   */
  virtual void OnDataOutComplete(bool in_isr, const Endpoint::Completion& completion) = 0;

  /// 数据 IN 端点的完成；只在已绑定时调用 / Completion of the data IN endpoint; called
  /// only while bound
  virtual void OnDataInComplete(bool in_isr, const Endpoint::Completion& completion) = 0;

  /**
   * @brief 把主机设的线路编码应用到实际端口 / Apply the host's line coding to the
   *        actual port
   * @param in_isr owner 是否在中断中；在 SET_LINE_CODING 的 OnControlData 中调用 /
   *        Whether the owner is in an ISR; called from OnControlData of SET_LINE_CODING
   * @param coding 输入主机设的值，输出生效值；GET_LINE_CODING 之后返回它。
   *        In: the host's coding; out: the applied one, which GET_LINE_CODING returns
   *        afterwards.
   * @param config 与 coding 对应的 UART 配置，输出生效值 / The UART configuration of
   *        coding; out: the applied one
   * @note 默认没有实际端口，原样接受。
   *       By default there is no actual port and the coding is taken as is.
   */
  virtual void ApplyLineCoding(bool in_isr, CDCLineCoding& coding,
                               UART::Configuration& config);
  /// @}

  /// @name 给派生类用 / For derived classes
  /// @{

  /// 线路编码转 UART 配置：1.5 停止位按 1，Mark/Space 按无校验 / Line coding to UART
  /// configuration: 1.5 stop bits as 1, mark/space parity as none
  static UART::Configuration ToUartConfiguration(const CDCLineCoding& coding);
  /// UART 配置转线路编码 / UART configuration to line coding
  static CDCLineCoding ToLineCoding(const UART::Configuration& cfg);

  /// 当前线路编码；仅 owner / Current line coding; owner only
  CDCLineCoding& GetLineCoding() { return line_coding_; }
  /// 数据 IN 端点，未绑定时为空；仅 owner / Data IN endpoint, null while unbound;
  /// owner only
  Endpoint* GetDataInEndpoint() const { return data_in_; }
  /// 数据 OUT 端点，未绑定时为空；仅 owner / Data OUT endpoint, null while unbound;
  /// owner only
  Endpoint* GetDataOutEndpoint() const { return data_out_; }
  /// @}

 private:
  /// CDC 功能描述符子类型 / CDC functional descriptor subtypes
  enum class DescriptorSubtype : uint8_t
  {
    HEADER = 0x00,           ///< 头 / Header
    CALL_MANAGEMENT = 0x01,  ///< 呼叫管理 / Call management
    ACM = 0x02,              ///< 抽象控制模型 / Abstract control model
    UNION = 0x06             ///< 联合 / Union
  };

  /// CDC 类请求 / CDC class requests
  enum class ClassRequest : uint8_t
  {
    SET_LINE_CODING = 0x20,         ///< 设置线路编码 / Set line coding
    GET_LINE_CODING = 0x21,         ///< 读线路编码 / Get line coding
    SET_CONTROL_LINE_STATE = 0x22,  ///< 设置 DTR/RTS / Set DTR/RTS
    SEND_BREAK = 0x23               ///< 发 BREAK（忽略）/ Send BREAK (ignored)
  };

  LIBXR_PACKED_BEGIN
  /// SERIAL_STATE 通知，10 字节 / SERIAL_STATE notification, 10 bytes
  struct SerialStateNotification
  {
    uint8_t bmRequestType;  ///< 0xA1
    uint8_t bNotification;  ///< SERIAL_STATE（0x20）
    uint16_t wValue;        ///< 0
    uint16_t wIndex;        ///< 通信接口号 / Communication interface number
    uint16_t wLength;       ///< 2
    uint16_t serialState;   ///< 状态位图 / State bitmap
  };

  /// 头功能描述符 / Header functional descriptor
  struct HeaderDescriptor
  {
    uint8_t bFunctionLength;
    uint8_t bDescriptorType;
    uint8_t bDescriptorSubtype;
    uint16_t bcdCDC;  ///< CDC 版本 / CDC version
  };

  /// 呼叫管理功能描述符 / Call management functional descriptor
  struct CallManagementDescriptor
  {
    uint8_t bFunctionLength;
    uint8_t bDescriptorType;
    uint8_t bDescriptorSubtype;
    uint8_t bmCapabilities;  ///< 能力 / Capabilities
    uint8_t bDataInterface;  ///< 数据接口号 / Data interface number
  };

  /// ACM 功能描述符 / ACM functional descriptor
  struct AcmDescriptor
  {
    uint8_t bFunctionLength;
    uint8_t bDescriptorType;
    uint8_t bDescriptorSubtype;
    uint8_t bmCapabilities;  ///< 能力 / Capabilities
  };

  /// 联合功能描述符 / Union functional descriptor
  struct UnionDescriptor
  {
    uint8_t bFunctionLength;
    uint8_t bDescriptorType;
    uint8_t bDescriptorSubtype;
    uint8_t bMasterInterface;  ///< 主接口 / Master interface
    uint8_t bSlaveInterface0;  ///< 从接口 / Slave interface
  };

  /// 本类的整个描述符块 / This class's whole descriptor block
  struct DescriptorBlock
  {
    IADDescriptor iad;
    InterfaceDescriptor comm_intf;
    HeaderDescriptor header;
    CallManagementDescriptor call_management;
    AcmDescriptor acm;
    UnionDescriptor union_desc;
    EndpointDescriptor comm_ep;
    InterfaceDescriptor data_intf;
    EndpointDescriptor data_out_ep;
    EndpointDescriptor data_in_ep;
  };
  LIBXR_PACKED_END

  /// 数据 IN 端点的配置（包长上限为高速批量）/ Data IN configuration (bound: high-speed
  /// bulk)
  static constexpr Endpoint::Config DATA_IN_CONFIG{Endpoint::Direction::IN,
                                                   Endpoint::Type::BULK, 512U};
  /// 数据 OUT 端点的配置 / Data OUT configuration
  static constexpr Endpoint::Config DATA_OUT_CONFIG{Endpoint::Direction::OUT,
                                                    Endpoint::Type::BULK, 512U};
  /// 通知端点的配置：10 字节的通知不需要更大的包 / Notification configuration: a
  /// 10-byte notification needs no larger packet
  static constexpr Endpoint::Config COMM_CONFIG{Endpoint::Direction::IN,
                                                Endpoint::Type::INTR, 16U};

  /// DTR 位 / DTR bit
  static constexpr uint32_t CONTROL_LINE_DTR = 0x01U;
  /// RTS 位 / RTS bit
  static constexpr uint32_t CONTROL_LINE_RTS = 0x02U;

  /// 有待发通知且有空闲槽时发出 SERIAL_STATE / Send SERIAL_STATE when one is pending
  /// and a slot is free
  void TrySendSerialState(bool in_isr);

  /// 三个端点的完成回调入口 / Completion entries of the three endpoints
  static void OnDataOutCompleteStatic(bool in_isr, CDCBase* self,
                                      const Endpoint::Completion& completion);
  static void OnDataInCompleteStatic(bool in_isr, CDCBase* self,
                                     const Endpoint::Completion& completion);
  static void OnCommCompleteStatic(bool in_isr, CDCBase* self,
                                   const Endpoint::Completion& completion);

  /// @name 构造时固定 / Fixed at construction
  /// @{
  Endpoint::EPNumber data_in_num_;        ///< 数据 IN 端点号 / Data IN number
  Endpoint::EPNumber data_out_num_;       ///< 数据 OUT 端点号 / Data OUT number
  Endpoint::EPNumber comm_num_;           ///< 通知端点号 / Notification number
  const char* control_interface_string_;  ///< 通信接口字符串 / Comm interface string
  const char* data_interface_string_;     ///< 数据接口字符串 / Data interface string
  Callback<const Endpoint::Completion&> data_out_cb_;  ///< 数据 OUT 回调 / Data OUT
  Callback<const Endpoint::Completion&> data_in_cb_;   ///< 数据 IN 回调 / Data IN
  Callback<const Endpoint::Completion&> comm_cb_;      ///< 通知回调 / Notification
  /// @}

  /// @name 应用在启动前设置，owner 读 / Set by the application before start, read by
  ///       the owner
  /// @{
  Callback<bool, bool> on_set_control_line_state_;    ///< DTR/RTS 回调 / DTR/RTS callback
  Callback<UART::Configuration> on_set_line_coding_;  ///< 线路编码回调 / Coding callback
  /// @}

  /// @name 仅 owner / Owner only
  /// @{
  Endpoint* data_in_ = nullptr;   ///< 数据 IN，绑定时非空 / Data IN, non-null if bound
  Endpoint* data_out_ = nullptr;  ///< 数据 OUT，绑定时非空 / Data OUT, non-null if bound
  Endpoint* comm_in_ = nullptr;   ///< 通知 IN，绑定时非空 / Notification IN, if bound
  /// 通信接口号；BindEndpoints 写，类请求和通知读 / Communication interface number;
  /// written by BindEndpoints, read by class requests and notifications
  uint8_t comm_interface_ = 0U;
  /// 当前线路编码；SET_LINE_CODING 和派生类写 / Current line coding; written by
  /// SET_LINE_CODING and the derived class
  CDCLineCoding line_coding_{115200U, 0U, 0U, 8U};
  /// GET_LINE_CODING 的应答，取请求那一刻的拷贝 / GET_LINE_CODING reply, a copy taken
  /// at the request
  CDCLineCoding line_coding_reply_{};
  /// WriteDescriptors 写出的块；BindEndpoints 比较包长时读 / Block written by
  /// WriteDescriptors; read by BindEndpoints to compare the sizes
  DescriptorBlock desc_block_{};
  /// @}

  /// @name 任何上下文 / Any context
  /// @{
  /// DTR/RTS：owner 写，任何上下文读 / DTR/RTS: written by the owner, read anywhere
  std::atomic<uint32_t> control_line_state_{0U};
  /// 有待发的 SERIAL_STATE：任何上下文置位，owner 清除 / SERIAL_STATE pending: set
  /// anywhere, cleared by the owner
  std::atomic<bool> serial_state_pending_{false};
  /// @}
};

}  // namespace LibXR::USB
