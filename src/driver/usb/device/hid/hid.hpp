#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

#include "device_class.hpp"
#include "ep.hpp"
#include "libxr_cb.hpp"
#include "libxr_def.hpp"
#include "libxr_type.hpp"

namespace LibXR::USB
{

/**
 * @class HID
 * @brief USB HID（人机接口设备）类的公共部分 / Common part of a USB HID (human interface
 *        device) class
 *
 * 一个接口、一个中断 IN 端点和可选的一个中断 OUT 端点；类请求 GET/SET_REPORT、
 * GET/SET_IDLE、GET/SET_PROTOCOL；接口为接收者的 GET_DESCRIPTOR（HID 描述符、报告
 * 描述符）。派生类给出报告描述符，按需重写报告钩子。
 *
 * One interface, one interrupt IN endpoint and an optional interrupt OUT endpoint; the
 * class requests GET/SET_REPORT, GET/SET_IDLE and GET/SET_PROTOCOL; GET_DESCRIPTOR with
 * the interface as recipient (HID descriptor, report descriptor). A derived class gives
 * the report descriptor and overrides the report hooks it needs.
 *
 * @note 除注明"任何上下文"的函数外，本类的函数都只在设备的 owner 中运行。
 *       Except for the functions marked "any context", everything here runs in the
 *       device's owner.
 */
class HID : public DeviceClass
{
 public:
  /// 默认接口字符串 / Default interface string
  static constexpr const char* DEFAULT_INTERFACE_STRING = "XRUSB HID";

  /// 启动协议（HID 1.11 4.2、4.3）/ Boot protocol (HID 1.11 4.2, 4.3)
  enum class BootProtocol : uint8_t
  {
    NONE = 0,      ///< 不支持启动协议 / No boot protocol
    KEYBOARD = 1,  ///< 启动键盘 / Boot keyboard
    MOUSE = 2      ///< 启动鼠标 / Boot mouse
  };

  /// 报告类型（GET/SET_REPORT 的 wValue 高字节）/ Report type (high byte of wValue)
  enum class ReportType : uint8_t
  {
    INPUT = 1,   ///< 输入报告 / Input report
    OUTPUT = 2,  ///< 输出报告 / Output report
    FEATURE = 3  ///< 特征报告 / Feature report
  };

  /**
   * @brief 构造 / Construct
   * @param in_ep_num 中断 IN 端点号 / Interrupt IN endpoint number
   * @param out_ep_num 中断 OUT 端点号；EP_INVALID 表示没有 OUT 端点。
   *        Interrupt OUT endpoint number; EP_INVALID for no OUT endpoint.
   * @param in_report_size 最长输入报告的字节数，1..1024 / Bytes of the longest input
   *        report, 1..1024
   * @param out_report_size 最长输出报告的字节数；有 OUT 端点时 1..1024，否则可为 0。
   *        Bytes of the longest output report; 1..1024 with an OUT endpoint, may be 0
   *        otherwise.
   * @param in_interval IN 端点描述符的 bInterval，原样写入 / bInterval of the IN
   *        endpoint descriptor, written as given
   * @param out_interval OUT 端点描述符的 bInterval，原样写入 / bInterval of the OUT
   *        endpoint descriptor, written as given
   * @param boot 写进接口描述符的启动协议 / Boot protocol written into the interface
   *        descriptor
   * @param interface_string 接口字符串 / Interface string
   * @note 分配两个端点回调（Callback::Create），只在构造时分配。
   *       Allocates the two endpoint callbacks (Callback::Create), at construction only.
   */
  HID(Endpoint::EPNumber in_ep_num, Endpoint::EPNumber out_ep_num, size_t in_report_size,
      size_t out_report_size, uint8_t in_interval, uint8_t out_interval,
      BootProtocol boot = BootProtocol::NONE,
      const char* interface_string = DEFAULT_INTERFACE_STRING);

  /**
   * @brief 发送一条输入报告 / Send one input report
   * @param report 报告字节，1..in_report_size / Report bytes, 1..in_report_size
   * @param in_isr 是否在中断中 / Whether in an ISR
   * @return OK：已提交；BUSY：IN 端点的槽都在用（BSP 给两个槽时，一条在途、一条排队）；
   *         FAILED：未绑定，或发送期间被解绑，报告没有提交；ARG_ERR：参数不合法。
   *         OK: submitted; BUSY: every slot of the IN endpoint is in use (with the two
   *         slots a BSP usually gives, one report in flight and one queued); FAILED: not
   *         bound, or unbound during the call, and the report was not submitted;
   *         ARG_ERR: bad arguments.
   * @note 任何上下文。借槽、拷贝、提交；可能就地运行 owner，完成回调可能在返回前运行。
   *       OK 之后被解绑的报告以 ABORTED 结束，主机收不到。
   *       Any context. Borrows a slot, copies and submits; may run the owner in place,
   *       so the completion can run before this returns. A report unbound after OK ends
   *       with ABORTED and never reaches the host.
   */
  ErrorCode SendInputReport(ConstRawData report, bool in_isr);

  /// 主机用 SET_IDLE 设的空闲速率（4 ms 为单位，0 表示无限）；任何上下文 / Idle rate
  /// the host set with SET_IDLE (4 ms units, 0 for indefinite); any context
  uint8_t GetIdleRate() const;

 protected:
  /// @name DeviceClass 的实现 / DeviceClass implementation
  /// @{
  size_t GetInterfaceCount() const override { return 1U; }
  bool HasIAD() const override { return false; }
  size_t GetMaxConfigSize() const override;
  const char* GetInterfaceString(size_t local_interface_index) const override;
  /// 64、最长输入报告、最长输出报告中最大的 / The largest of 64, the longest input
  /// report and the longest output report
  size_t GetControlReceiveCapacity() const override;

  /**
   * @brief 写接口、HID 描述符和端点描述符 / Write the interface, HID and endpoint
   *        descriptors
   * @return OK；NOT_SUPPORT：某个端点在该速度下查询得 0。
   *         OK; NOT_SUPPORT: an endpoint's query returned 0 at that speed.
   */
  ErrorCode WriteDescriptors(const EndpointPool& pool, Speed speed,
                             uint8_t first_interface, ConstRawData& block) override;

  /**
   * @brief 取端点、Configure、检查包长和请求容量、挂 OUT 接收 / Take the endpoints,
   *        configure them, check packet size and request capacity, arm the OUT receive
   * @return OK；NOT_FOUND：Get 失败；CHECK_ERR：读回的包长与描述符不同；NO_BUFF：一次
   *         请求装不下最长报告；其他：Configure 的错误。失败时已取的端点留在成员里，由
   *         随后的 UnbindEndpoints 收回；SendInputReport 只在成功后才可用。
   *         OK; NOT_FOUND: a Get failed; CHECK_ERR: a size read back differs from the
   *         descriptor; NO_BUFF: one request cannot hold the longest report; otherwise
   *         the error of Configure. On failure the endpoints taken stay in the members
   *         and the following UnbindEndpoints takes them back; SendInputReport works only
   *         after a success.
   * @note 协议回到报告协议，空闲速率回到 0。
   *       The protocol returns to the report protocol and the idle rate to 0.
   */
  [[nodiscard]] ErrorCode BindEndpoints(EndpointPool& pool, uint8_t first_interface,
                                        bool in_isr) override;

  /**
   * @brief 停止发送，关闭并交回已取的端点 / Stop senders, close and return the
   *        endpoints taken
   * @note 撤下 SendInputReport 用的指针；每个端点指针在 Close 之前置空，所以 Close
   *       交付的完成回调看到"未绑定"，什么也不做。
   *       Withdraws the pointer SendInputReport uses; each endpoint pointer is cleared
   *       before its Close, so the completions Close delivers see "unbound" and do
   *       nothing.
   */
  void UnbindEndpoints(EndpointPool& pool, bool in_isr) override;

  /// IN（和 OUT）端点地址；由端点号算出，不需要已绑定 / The IN (and OUT) endpoint
  /// address; computed from the numbers, no bind needed
  bool OwnsEndpoint(uint8_t ep_addr) const override;

  /**
   * @brief 单功能设备：类、子类、协议都写 0，由接口描述符说明 / Single-function
   *        device: class, subclass and protocol are 0, the interface descriptor tells
   * @note HID 1.11 5.1：HID 类由接口描述符标明，不用设备描述符。
   *       HID 1.11 5.1: the HID class is identified by the interface descriptor, not
   *       the device descriptor.
   */
  ErrorCode WriteDeviceDescriptor(DeviceDescriptor& descriptor) override;

  /**
   * @brief HID 描述符（0x21）和报告描述符（0x22），索引只能是 0 / The HID descriptor
   *        (0x21) and the report descriptor (0x22), index 0 only
   * @note HID 描述符指向描述符块里的那一份，它与速度无关，重写描述符块不改变它。
   *       The HID descriptor points at the copy in the descriptor block; it does not
   *       depend on the speed, so rewriting the block leaves it unchanged.
   */
  ErrorCode OnGetDescriptor(bool in_isr, const SetupPacket& setup,
                            ConstRawData& data) override;

  /**
   * @brief 类请求的 SETUP 阶段 / SETUP stage of a class request
   * @return 只接受接收者为接口的六个 HID 请求，方向、长度、报告类型和报告号必须合法，
   *         否则 ARG_ERR 或 NOT_SUPPORT（核心 STALL）。
   *         Only the six HID requests with the interface as recipient, with a legal
   *         direction, length, report type and report ID; otherwise ARG_ERR or
   *         NOT_SUPPORT (the core stalls).
   * @note SET_IDLE、SET_PROTOCOL 在这里生效；GET_REPORT 交给 OnGetReport；SET_REPORT
   *       的数据在 OnControlData 交给 OnSetReport。
   *       SET_IDLE and SET_PROTOCOL take effect here; GET_REPORT goes to OnGetReport;
   *       the SET_REPORT data goes to OnSetReport in OnControlData.
   */
  ErrorCode OnControlRequest(bool in_isr, const SetupPacket& setup,
                             ConstRawData& in_data) override;

  /// SET_REPORT 的数据阶段，交给 OnSetReport / Data stage of SET_REPORT, handed to
  /// OnSetReport
  ErrorCode OnControlData(bool in_isr, const SetupPacket& setup,
                          ConstRawData data) override;
  /// @}

  /// @name 派生类实现 / Implemented by the derived class
  /// @{

  /// 报告描述符；内容在对象生存期内不变 / The report descriptor; its content never
  /// changes during the object's life
  virtual ConstRawData GetReportDesc() const = 0;

  /**
   * @brief GET_REPORT / GET_REPORT
   * @param in_isr owner 是否在中断中 / Whether the owner is in an ISR
   * @param type 报告类型（wValue 高字节），已检查为 INPUT、OUTPUT 或 FEATURE / Report
   *        type (high byte of wValue), checked to be INPUT, OUTPUT or FEATURE
   * @param report_id 报告号（wValue 低字节）/ Report ID (low byte of wValue)
   * @param data 输出：应答，核心按 wLength 截短；保持有效且不变，直到本次传输结束。
   *        Output: the reply, truncated by the core to wLength; it stays valid and
   *        unchanged until the transfer ends.
   * @return OK；其他：核心 STALL。默认 OK 且应答为空。
   *         OK; otherwise the core stalls. The default is OK with an empty reply.
   */
  virtual ErrorCode OnGetReport(bool in_isr, ReportType type, uint8_t report_id,
                                ConstRawData& data);

  /**
   * @brief SET_REPORT 的数据 / Data of SET_REPORT
   * @param in_isr owner 是否在中断中 / Whether the owner is in an ISR
   * @param type 报告类型（wValue 高字节），已检查为 INPUT、OUTPUT 或 FEATURE / Report
   *        type (high byte of wValue), checked to be INPUT, OUTPUT or FEATURE
   * @param report_id 报告号（wValue 低字节）/ Report ID (low byte of wValue)
   * @param data 收到的字节，只在本次调用内有效 / The bytes received, valid during this
   *        call only
   * @return OK：核心发状态阶段；其他：核心 STALL。默认：输出报告交给 OnOutReport，其他
   *         类型 NOT_SUPPORT。
   *         OK: the core sends the status stage; otherwise the core stalls. Default: an
   *         output report goes to OnOutReport, other types get NOT_SUPPORT.
   */
  virtual ErrorCode OnSetReport(bool in_isr, ReportType type, uint8_t report_id,
                                ConstRawData data);

  /**
   * @brief 收到一条输出报告：来自中断 OUT，或来自 SET_REPORT(Output) / An output
   *        report arrived, from the interrupt OUT endpoint or from SET_REPORT(Output)
   * @param in_isr owner 是否在中断中 / Whether the owner is in an ISR
   * @param report 报告字节，只在本次调用内有效 / Report bytes, valid during this call
   *        only
   * @return OK；其他：来自 SET_REPORT 时核心 STALL，来自中断 OUT 时忽略。默认
   *         NOT_SUPPORT。
   *         OK; otherwise the core stalls when it came from SET_REPORT and the result
   *         is ignored when it came from the OUT endpoint. The default is NOT_SUPPORT.
   */
  virtual ErrorCode OnOutReport(bool in_isr, ConstRawData report);

  /**
   * @brief OUT 端点的完成；只在已绑定时调用 / Completion of the OUT endpoint; called
   *        only while bound
   * @note 默认：OK 时把收到的字节交给 OnOutReport，交回槽；然后不论结果都重新挂接收
   *       （STALLED 时排队到 ClearStall）。重写者自己负责交回 completion.slot 和重挂。
   *       Default: on OK the received bytes go to OnOutReport and the slot is handed
   *       back; then a receive is armed again whatever the result (while STALLED it
   *       waits for ClearStall). An override hands completion.slot back and re-arms by
   *       itself.
   */
  virtual void OnDataOutComplete(bool in_isr, const Endpoint::Completion& completion);

  /// IN 端点的完成；只在已绑定时调用；默认什么也不做 / Completion of the IN
  /// endpoint; called only while bound; does nothing by default
  virtual void OnDataInComplete(bool in_isr, const Endpoint::Completion& completion);
  /// @}

  /// @name 给派生类用 / For derived classes
  /// @{

  /// IN 端点，未绑定时为空；仅 owner / IN endpoint, null while unbound; owner only
  Endpoint* GetInEndpoint() const { return in_ep_; }
  /// OUT 端点，未绑定或没有时为空；仅 owner / OUT endpoint, null while unbound or
  /// absent; owner only
  Endpoint* GetOutEndpoint() const { return out_ep_; }
  /**
   * @brief 一次 OUT 接收的字节数 / Bytes of one OUT receive
   * @note 最长输出报告向上取整到包长；仅 owner，已绑定时。
   *       The longest output report rounded up to the packet size; owner only, while
   *       bound.
   */
  size_t OutReceiveSize() const;
  /// @}

 private:
  /// HID 描述符类型 / HID descriptor types
  enum class HidDescriptorType : uint8_t
  {
    HID = 0x21,    ///< HID 描述符 / HID descriptor
    REPORT = 0x22  ///< 报告描述符 / Report descriptor
  };

  /// HID 类请求（HID 1.11 7.2）/ HID class requests (HID 1.11 7.2)
  enum class ClassRequest : uint8_t
  {
    GET_REPORT = 0x01,    ///< 读报告 / Get report
    GET_IDLE = 0x02,      ///< 读空闲速率 / Get idle
    GET_PROTOCOL = 0x03,  ///< 读协议 / Get protocol
    SET_REPORT = 0x09,    ///< 写报告 / Set report
    SET_IDLE = 0x0A,      ///< 写空闲速率 / Set idle
    SET_PROTOCOL = 0x0B   ///< 写协议 / Set protocol
  };

  LIBXR_PACKED_BEGIN
  /// HID 描述符（HID 1.11 6.2.1），带一个报告描述符 / HID descriptor (HID 1.11 6.2.1)
  /// with one report descriptor
  struct HidDescriptor
  {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint16_t bcdHID;  ///< HID 版本 / HID version
    uint8_t bCountryCode;
    uint8_t bNumDescriptors;
    uint8_t bReportDescriptorType;
    uint16_t wReportDescriptorLength;
  };

  /// 本类的整个描述符块；没有 OUT 端点时不含最后一项 / This class's whole descriptor
  /// block; without an OUT endpoint the last item is left out
  struct DescriptorBlock
  {
    InterfaceDescriptor intf;
    HidDescriptor hid;
    EndpointDescriptor ep_in;
    EndpointDescriptor ep_out;
  };
  LIBXR_PACKED_END

  /// 包长上限：高速中断端点的最大包 / Packet-size bound: the largest high-speed
  /// interrupt packet
  static constexpr size_t MAX_PACKET = 1024U;

  /// IN 端点的配置 / IN endpoint configuration
  Endpoint::Config InConfig() const;
  /// OUT 端点的配置 / OUT endpoint configuration
  Endpoint::Config OutConfig() const;
  /// 是否有 OUT 端点 / Whether there is an OUT endpoint
  bool HasOut() const { return out_num_ != Endpoint::EPNumber::EP_INVALID; }

  /// BindEndpoint，再查一次请求装得下最长报告 / BindEndpoint, then check that one
  /// request holds the longest report
  static ErrorCode BindOne(EndpointPool& pool, Endpoint::EPNumber number,
                           const Endpoint::Config& cfg, uint16_t descriptor_field,
                           size_t report_size, bool in_isr, Endpoint*& ep);

  /// 两个端点的完成回调入口 / Completion entries of the two endpoints
  static void OnDataOutCompleteStatic(bool in_isr, HID* self,
                                      const Endpoint::Completion& completion);
  static void OnDataInCompleteStatic(bool in_isr, HID* self,
                                     const Endpoint::Completion& completion);

  /// @name 构造时固定 / Fixed at construction
  /// @{
  Endpoint::EPNumber in_num_;     ///< IN 端点号 / IN endpoint number
  Endpoint::EPNumber out_num_;    ///< OUT 端点号或 EP_INVALID / OUT number or invalid
  size_t in_report_size_;         ///< 最长输入报告 / Longest input report
  size_t out_report_size_;        ///< 最长输出报告 / Longest output report
  uint8_t in_interval_;           ///< IN 的 bInterval / bInterval of IN
  uint8_t out_interval_;          ///< OUT 的 bInterval / bInterval of OUT
  BootProtocol boot_;             ///< 启动协议 / Boot protocol
  const char* interface_string_;  ///< 接口字符串 / Interface string
  Callback<const Endpoint::Completion&> out_cb_;  ///< OUT 回调 / OUT callback
  Callback<const Endpoint::Completion&> in_cb_;   ///< IN 回调 / IN callback
  /// @}

  /// @name 仅 owner / Owner only
  /// @{
  /// IN 端点，从 Get 成功到解绑非空 / IN endpoint, non-null from a successful Get to
  /// the unbind
  Endpoint* in_ep_ = nullptr;
  /// OUT 端点，从 Get 成功到解绑非空 / OUT endpoint, non-null from a successful Get to
  /// the unbind
  Endpoint* out_ep_ = nullptr;
  /**
   * @brief 当前协议 / Current protocol
   * @note 0 启动，1 报告；SET_PROTOCOL 和绑定写，GET_PROTOCOL 读。
   *       0 boot, 1 report; written by SET_PROTOCOL and the bind, read by GET_PROTOCOL.
   */
  uint8_t protocol_ = 1U;
  /// GET_IDLE 的应答，取请求那一刻的拷贝 / GET_IDLE reply, a copy taken at the request
  uint8_t idle_reply_ = 0U;
  /**
   * @brief WriteDescriptors 写出的块 / Block written by WriteDescriptors
   * @note BindEndpoints 比较包长、OnGetDescriptor 取 HID 描述符时读。
   *       Read by BindEndpoints to compare the sizes and by OnGetDescriptor for the HID
   *       descriptor.
   */
  DescriptorBlock desc_block_{};
  /// @}

  /// @name 任何上下文 / Any context
  /// @{
  /**
   * @brief SendInputReport 用的 IN 端点 / IN endpoint for SendInputReport
   * @note 绑定成功后 owner 写入，解绑开始时 owner 清空，SendInputReport 读。
   *       Written by the owner after a successful bind, cleared by the owner when the
   *       unbind starts, read by SendInputReport.
   */
  std::atomic<Endpoint*> sender_ep_{nullptr};
  /// 空闲速率：SET_IDLE 和绑定写（owner），任何上下文读 / Idle rate: written by
  /// SET_IDLE and the bind (owner), read anywhere
  std::atomic<uint8_t> idle_rate_{0U};
  /// @}
};

}  // namespace LibXR::USB
