#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "daplink_v1_def.hpp"
#include "debug/jtag_dp.hpp"
#include "debug/swd.hpp"
#include "device_class.hpp"
#include "ep.hpp"
#include "gpio.hpp"
#include "libxr_cb.hpp"
#include "libxr_def.hpp"
#include "libxr_mem.hpp"
#include "libxr_type.hpp"
#include "semaphore.hpp"
#include "timebase.hpp"

namespace LibXR::USB
{
#ifndef LIBXR_USB_DAPLINK_V1_BUILD_CONFIG_DEFINED
#define LIBXR_USB_DAPLINK_V1_BUILD_CONFIG_DEFINED
/// DAP v1 的编译配置；profile 头文件可以先定义它 / Build configuration of DAP v1; a
/// profile header may define it first
namespace DapLinkV1BuildConfig
{
/// 是否编入 JTAG 支持；未包含 profile 头文件时为 true / Whether JTAG support is
/// compiled in; true when no profile header is included
static constexpr bool ENABLE_JTAG = true;
}  // namespace DapLinkV1BuildConfig
#endif

/**
 * @class DapLinkV1Class
 * @brief CMSIS-DAP v1（HID）USB 类 / CMSIS-DAP v1 (HID) USB class
 *
 * 一个 HID 接口，中断 IN 和中断 OUT 各一个，报告固定 64 字节；支持 SWD 和 JTAG
 * （daplink_v1_profile_swd.hpp 可去掉 JTAG），可选 nRESET。HID 部分只有本类需要的：
 * HID 和报告描述符，SET_IDLE / GET_IDLE；报告只走中断端点，GET_REPORT、SET_REPORT 和
 * 协议请求被拒绝。
 * One HID interface with one interrupt IN and one interrupt OUT endpoint and fixed
 * 64-byte reports; SWD and JTAG (daplink_v1_profile_swd.hpp leaves JTAG out),
 * optional nRESET. The HID part is only what this class needs: the HID and report
 * descriptors and SET_IDLE / GET_IDLE; reports travel only on the interrupt endpoints,
 * and GET_REPORT, SET_REPORT and the protocol requests are refused.
 *
 * @note 三种上下文。owner：描述符、绑定、解绑、端点完成回调、控制请求和 OnService；
 *       OUT 完成时它只把借来的槽放进环并叫醒 Step 线程。Step 线程（应用调用 Step() 的
 *       线程）：直接从借来的 OUT 槽读命令并执行，借 IN 槽写响应并在本线程提交（Submit
 *       可能就地运行 owner），然后交回 OUT 槽并请 owner 补挂接收。响应提交之前 OUT 槽
 *       一直借着，所有槽都借出时主机被 NAK（背压）。
 *       Three contexts. The owner: descriptors, bind, unbind, the endpoint completion
 *       callbacks, control requests and OnService; on an OUT completion it only puts the
 *       lent slot into the ring and wakes the Step thread. The Step thread (the thread
 *       that calls Step()): reads the command straight from the lent OUT slot, executes
 *       it, borrows an IN slot for the response and submits it from this thread (Submit
 *       may run the owner in place), then hands the OUT slot back and asks the owner to
 *       re-arm. The OUT slot stays lent until the response is submitted; with every slot
 *       lent the host is NAKed (backpressure).
 * @note 会话：每次绑定和解绑使 epoch_ 加一；Step 线程据此丢弃旧会话的报告，并在自己
 *       的线程里关闭 SWD、恢复默认状态。
 *       Session: every bind and unbind adds one to epoch_; with it the Step thread drops
 *       reports of an old session, and closes SWD and restores the defaults in its own
 *       thread.
 *
 * @tparam SwdPort SWD 链路类型 / SWD link type
 */
template <typename SwdPort>
class DapLinkV1Class : public DeviceClass
{
 public:
  /// 编译期是否包含 JTAG 支持 / Whether JTAG support is compiled in
  static constexpr bool JTAG_ENABLED = DapLinkV1BuildConfig::ENABLE_JTAG;
  /// 默认的接口字符串 / Default interface string
  static constexpr const char* DEFAULT_INTERFACE_STRING = "CMSIS-DAP";

  /// DAP_Info 返回的字符串；空指针项返回空字符串 / Strings returned by DAP_Info; a null
  /// entry yields an empty string
  struct InfoStrings
  {
    const char* vendor = nullptr;          ///< 厂商名 / Vendor name
    const char* product = nullptr;         ///< 产品名 / Product name
    const char* serial = nullptr;          ///< 序列号 / Serial number
    const char* firmware_ver = nullptr;    ///< 固件版本 / Firmware version
    const char* device_vendor = nullptr;   ///< 目标器件厂商 / Target device vendor
    const char* device_name = nullptr;     ///< 目标器件名 / Target device name
    const char* board_vendor = nullptr;    ///< 板卡厂商 / Board vendor
    const char* board_name = nullptr;      ///< 板卡名 / Board name
    const char* product_fw_ver = nullptr;  ///< 产品固件版本 / Product firmware version
  };

  /**
   * @brief 构造 / Construct
   * @param in_ep_num 中断 IN 端点号 / Interrupt IN endpoint number
   * @param out_ep_num 中断 OUT 端点号 / Interrupt OUT endpoint number
   * @param swd_link SWD 链路，须比本对象长寿 / SWD link; must outlive this object
   * @param nreset_gpio 可选的 nRESET 引脚 / Optional nRESET pin
   * @param interface_string 接口字符串，须保持有效 / Interface string; must stay valid
   * @note 不碰 SWD/JTAG，不起线程。 / Touches neither SWD nor JTAG; starts no thread.
   */
  explicit DapLinkV1Class(Endpoint::EPNumber in_ep_num, Endpoint::EPNumber out_ep_num,
                          SwdPort& swd_link, LibXR::GPIO* nreset_gpio = nullptr,
                          const char* interface_string = DEFAULT_INTERFACE_STRING);

  /// 设置 DAP_Info 的字符串；启动前 / Set the DAP_Info strings; before start
  void SetInfoStrings(const InfoStrings& info);

  /**
   * @brief 设置 JTAG 后端；启动前 / Set the JTAG backend; before start
   * @param jtag JTAG 驱动；非空时 DAP_Info 才声明 JTAG。未编译 JTAG 时忽略。
   *        JTAG driver; JTAG is advertised by DAP_Info only when non-null. Ignored when
   *        JTAG is not compiled in.
   */
  void SetJtag(LibXR::Debug::Jtag* jtag);

  /// DAP 会话状态；在 Step 线程里或 Step 不运行时读 / DAP session state; read in the
  /// Step thread or while Step does not run
  const LibXR::USB::DapLinkV1Def::State& GetState() const;

  /// 端点已绑定（主机已配置本接口）；任何上下文 / The endpoints are bound (the host
  /// has configured this interface); any context
  bool IsInited() const { return in_ep_.load(std::memory_order_acquire) != nullptr; }

  /**
   * @brief 在调用线程里做一次 DAP 工作：等一个报告，执行并提交响应 / Do one unit of DAP
   *        work in the calling thread: wait for a report, execute it and submit the
   *        response
   * @param timeout_ms 等报告或 IN 槽的最长时间；0 不等 / Longest wait for a report or an
   *        IN slot; 0 does not wait
   * @return OK：做了一件事（执行并提交、丢弃一个报告，或落实了会话的开始/结束）；
   *         TIMEOUT：期限内没有工作，或响应还在等 IN 槽（下一次 Step 接着提交）。
   *         OK: something was done (a report executed and submitted or dropped, or the
   *         session start/end applied); TIMEOUT: no work arrived in time, or the
   *         response still waits for an IN slot (the next Step goes on submitting it).
   * @note 只在一个应用线程里循环调用。延时类命令在这个线程里等到完成。
   *       Call it in a loop from one application thread only. Delay-type commands wait
   *       in this thread until they are done.
   */
  ErrorCode Step(uint32_t timeout_ms = UINT32_MAX);

 protected:
  /// @name DeviceClass 的实现 / DeviceClass implementation
  /// @{
  size_t GetInterfaceCount() const override { return 1U; }
  bool HasIAD() const override { return false; }
  size_t GetMaxConfigSize() const override { return sizeof(desc_block_); }
  const char* GetInterfaceString(size_t local_interface_index) const override;

  /// 接口级识别类：设备类设为 PER_INTERFACE（符合 USB HID 1.11 §4.2）/
  /// Interface-level class: set device class to PER_INTERFACE (USB HID 1.11 §4.2)
  ErrorCode WriteDeviceDescriptor(DeviceDescriptor& descriptor) override;

  /**
   * @brief 写接口、HID 描述符和两个中断端点描述符 / Write the interface, the HID
   *        descriptor and the two interrupt endpoint descriptors
   * @return OK；NOT_SUPPORT：某个端点在该速度下的包长不是 64（报告按一个包收发）。
   *         OK; NOT_SUPPORT: an endpoint's packet size at that speed is not 64 (a report
   *         travels as one packet).
   */
  ErrorCode WriteDescriptors(const EndpointPool& pool, Speed speed,
                             uint8_t first_interface, ConstRawData& block) override;

  /**
   * @brief 取两个端点、Configure、比较包长，开始新会话并挂接收 / Take both endpoints,
   *        configure them, compare the sizes, start a new session and arm the receives
   * @return OK；NOT_FOUND：Get 失败；CHECK_ERR：读回的包长与描述符不同；其他：Configure
   *         的错误。失败时已取的端点留在成员里，由随后的 UnbindEndpoints 收回。
   *         OK; NOT_FOUND: a Get failed; CHECK_ERR: a size read back differs from the
   *         descriptor; otherwise the error of Configure. On failure the endpoints taken
   *         stay in the members and the following UnbindEndpoints takes them back.
   */
  [[nodiscard]] ErrorCode BindEndpoints(EndpointPool& pool, uint8_t first_interface,
                                        bool in_isr) override;

  /**
   * @brief 结束会话，关闭并交回两个端点 / End the session, close and return both
   *        endpoints
   * @note IN 引用置空、epoch 加一（先于 Close，见 SubmitCurrent），每个端点先把指针置空
   *       再 Close、Release，最后叫醒 Step。环里和 Step 手里的 OUT 槽由 Step 交回（Close
   *       后作废，交回得 OK）。SWD 由 Step 线程关闭。
   *       The IN reference is cleared and the epoch moves (before Close, see
   *       SubmitCurrent), each endpoint's pointer is cleared before its Close and
   *       Release, and Step is woken last. The OUT slots in the ring and in Step's hands
   *       are handed back by Step (stale after Close, handing back gives OK). The Step
   *       thread closes SWD.
   */
  void UnbindEndpoints(EndpointPool& pool, bool in_isr) override;

  /// 两个端点地址之一；由端点号算出 / One of the two endpoint addresses; computed from
  /// the numbers
  bool OwnsEndpoint(uint8_t ep_addr) const override;

  /**
   * @brief HID 类描述符（0x21）和报告描述符（0x22）/ The HID class descriptor (0x21)
   *        and the report descriptor (0x22)
   * @return OK；NOT_SUPPORT：其他类型（核心 STALL）/ OK; NOT_SUPPORT: any other type
   *         (the core stalls)
   */
  ErrorCode OnGetDescriptor(bool in_isr, const SetupPacket& setup,
                            ConstRawData& data) override;

  /**
   * @brief HID 类请求 / HID class requests
   * @return SET_IDLE（无数据阶段）、GET_IDLE（1 字节）为 OK；方向或长度不符 ARG_ERR；
   *         其他 NOT_SUPPORT（核心 STALL）。
   *         OK for SET_IDLE (no data stage) and GET_IDLE (1 byte); ARG_ERR for a wrong
   *         direction or length; NOT_SUPPORT otherwise (the core stalls).
   */
  ErrorCode OnControlRequest(bool in_isr, const SetupPacket& setup,
                             ConstRawData& in_data) override;

  /// Step 交回 OUT 槽后补挂接收；未绑定时什么也不做 / Re-arm the receives after Step
  /// handed an OUT slot back; nothing while unbound
  void OnService(bool in_isr) override;
  /// @}

 private:
  /// 一个收到的报告：OUT 槽、所属端点和收到时的 epoch / One received report: the OUT
  /// slot, its endpoint and the epoch it was received in
  struct Lease
  {
    Endpoint::Slot slot{};   ///< 借来的 OUT 槽 / The lent OUT slot
    Endpoint* ep = nullptr;  ///< 槽所属端点 / Endpoint of the slot
    uint32_t epoch = 0u;     ///< 收到时的 epoch_ / epoch_ when received
  };

  LIBXR_PACKED_BEGIN
  /// HID 1.11 类描述符，9 字节 / HID 1.11 class descriptor, 9 bytes
  struct HidDescriptor
  {
    uint8_t bLength;                   ///< 9
    uint8_t bDescriptorType;           ///< 0x21
    uint16_t bcdHID;                   ///< 0x0111
    uint8_t bCountryCode;              ///< 0
    uint8_t bNumDescriptors;           ///< 1
    uint8_t bReportDescriptorType;     ///< 0x22
    uint16_t wReportDescriptorLength;  ///< 报告描述符长度 / Report descriptor length
  };

  /// 本类的描述符块 / This class's descriptor block
  struct DescriptorBlock
  {
    InterfaceDescriptor intf;   ///< 接口描述符 / Interface descriptor
    HidDescriptor hid;          ///< HID 描述符 / HID descriptor
    EndpointDescriptor ep_in;   ///< IN 端点描述符 / IN endpoint descriptor
    EndpointDescriptor ep_out;  ///< OUT 端点描述符 / OUT endpoint descriptor
  };
  LIBXR_PACKED_END

  /**
   * @brief 报告描述符：厂商自定义的 64 字节输入、输出、特征报告 / Report descriptor:
   *        vendor-defined 64-byte input, output and feature reports
   * @note 依次：Usage Page (0xFF00)、Usage (1)、Collection (Application)、Logical
   *       Minimum (0)、Logical Maximum (255)、Report Size (8)；三组 Report Count (64)、
   *       Usage (1) 与 Input / Output / Feature (Data, Variable, Absolute)；End
   *       Collection。
   *       In order: Usage Page (0xFF00), Usage (1), Collection (Application), Logical
   *       Minimum (0), Logical Maximum (255), Report Size (8); three groups of Report
   *       Count (64), Usage (1) and Input / Output / Feature (Data, Variable, Absolute);
   *       End Collection.
   */
  static constexpr uint8_t REPORT_DESC[] = {
      0x06, 0x00, 0xFF, 0x09, 0x01, 0xA1, 0x01, 0x15, 0x00, 0x26, 0xFF,
      0x00, 0x75, 0x08, 0x95, 0x40, 0x09, 0x01, 0x81, 0x02, 0x95, 0x40,
      0x09, 0x01, 0x91, 0x02, 0x95, 0x40, 0x09, 0x01, 0xB1, 0x02, 0xC0};

  /// HID 类描述符；内容固定 / The HID class descriptor; fixed content
  static constexpr HidDescriptor HID_DESCRIPTOR{
      9U, 0x21U, 0x0111U, 0U, 1U, 0x22U, static_cast<uint16_t>(sizeof(REPORT_DESC))};

  static constexpr uint16_t MAX_REQ = DapLinkV1Def::MAX_REQUEST_SIZE;    ///< 64
  static constexpr uint16_t MAX_RESP = DapLinkV1Def::MAX_RESPONSE_SIZE;  ///< 64
  static constexpr uint8_t JTAG_MAX_DEVICES = 16u;  ///< JTAG 链上限 / JTAG chain limit
  /// 环装得下后端可能给的全部 OUT 槽 / The ring holds every OUT slot a backend may give
  static constexpr uint32_t LEASE_RING_SIZE = Endpoint::MAX_SLOTS;
  /// 中断端点的配置；包长上限即报告长度 / Interrupt endpoint configurations; the bound
  /// is the report length
  static constexpr Endpoint::Config IN_CONFIG{Endpoint::Direction::IN,
                                              Endpoint::Type::INTR, MAX_RESP};
  static constexpr Endpoint::Config OUT_CONFIG{Endpoint::Direction::OUT,
                                               Endpoint::Type::INTR, MAX_REQ};
  /// 端点轮询间隔：全速 1 ms，高速 1 微帧 / Polling interval: 1 ms at full speed, one
  /// microframe at high speed
  static constexpr uint8_t INTERVAL = 1U;
  /// HID 接口类 / HID interface class
  static constexpr uint8_t CLASS_HID = 0x03U;
  /// HID 类请求 / HID class requests
  static constexpr uint8_t REQUEST_GET_IDLE = 0x02U;
  static constexpr uint8_t REQUEST_SET_IDLE = 0x0AU;

  static constexpr uint8_t DAP_OK = 0x00u;     ///< DAP_OK
  static constexpr uint8_t DAP_ERROR = 0xFFu;  ///< DAP_ERROR

  /// 将枚举或整数转为 uint8_t / Cast an enum or integer to uint8_t
  template <typename E>
  static constexpr uint8_t to_u8(E e)
  {
    return static_cast<uint8_t>(e);
  }

  /// 未知命令响应：单字节 0xFF / Unknown command response: one byte 0xFF
  static inline ErrorCode build_unknown_cmd_response(uint8_t* resp, uint16_t cap,
                                                     uint16_t& out_len)
  {
    if (!resp || cap < 1u)
    {
      out_len = 0u;
      return ErrorCode::NOT_FOUND;
    }
    resp[0] = 0xFFu;
    out_len = 1u;
    return ErrorCode::OK;
  }

  /// 命令状态响应 [cmd, status] / Command status response [cmd, status]
  static inline ErrorCode build_cmd_status_response(uint8_t cmd, uint8_t status,
                                                    uint8_t* resp, uint16_t cap,
                                                    uint16_t& out_len)
  {
    if (!resp || cap < 2u)
    {
      out_len = 0u;
      return ErrorCode::NOT_FOUND;
    }
    resp[0] = cmd;
    resp[1] = status;
    out_len = 2u;
    return ErrorCode::OK;
  }

  /// OUT 完成：把借来的槽放进环，叫醒 Step / OUT completion: put the lent slot into the
  /// ring and wake Step
  static void OnDataOutCompleteStatic(bool in_isr, DapLinkV1Class* self,
                                      const Endpoint::Completion& completion);
  /// IN 完成：叫醒等 IN 槽的 Step / IN completion: wake a Step waiting for an IN slot
  static void OnDataInCompleteStatic(bool in_isr, DapLinkV1Class* self,
                                     const Endpoint::Completion& completion);

  /// 挂接收直到没有空闲槽；仅 owner / Arm receives until no slot is free; owner only
  void TopUpReceives(bool in_isr);
  /// Step 线程：从环里取一个报告 / Step thread: take one report from the ring
  bool TakeLease(Lease& lease);
  /// Step 线程：交回手里的 OUT 槽并请 owner 补挂 / Step thread: hand back the OUT slot
  /// in hand and ask the owner to re-arm
  void DropCurrent();
  /// Step 线程：借 IN 槽提交响应；true 表示已提交 / Step thread: borrow an IN slot and
  /// submit the response; true when submitted
  bool SubmitCurrent();
  /// Step 线程：epoch 变了就关闭 SWD/JTAG 并恢复默认值 / Step thread: on a changed
  /// epoch close SWD/JTAG and restore the defaults
  void SyncSession(uint32_t epoch);

  ErrorCode ProcessOneCommand(bool in_isr, const uint8_t* req, uint16_t req_len,
                              uint8_t* resp, uint16_t resp_cap, uint16_t& out_len);
  void BuildNotSupportResponse(uint8_t* resp, uint16_t resp_cap, uint16_t& out_len);

  ErrorCode HandleInfo(bool in_isr, const uint8_t* req, uint16_t req_len, uint8_t* resp,
                       uint16_t resp_cap, uint16_t& out_len);
  ErrorCode HandleHostStatus(bool in_isr, const uint8_t* req, uint16_t req_len,
                             uint8_t* resp, uint16_t resp_cap, uint16_t& out_len);
  ErrorCode HandleConnect(bool in_isr, const uint8_t* req, uint16_t req_len,
                          uint8_t* resp, uint16_t resp_cap, uint16_t& out_len);
  ErrorCode HandleDisconnect(bool in_isr, const uint8_t* req, uint16_t req_len,
                             uint8_t* resp, uint16_t resp_cap, uint16_t& out_len);
  ErrorCode HandleTransferConfigure(bool in_isr, const uint8_t* req, uint16_t req_len,
                                    uint8_t* resp, uint16_t resp_cap, uint16_t& out_len);
  ErrorCode HandleTransfer(bool in_isr, const uint8_t* req, uint16_t req_len,
                           uint8_t* resp, uint16_t resp_cap, uint16_t& out_len);
  ErrorCode HandleTransferBlock(bool in_isr, const uint8_t* req, uint16_t req_len,
                                uint8_t* resp, uint16_t resp_cap, uint16_t& out_len);
  ErrorCode HandleTransferAbort(bool in_isr, const uint8_t* req, uint16_t req_len,
                                uint8_t* resp, uint16_t resp_cap, uint16_t& out_len);
  ErrorCode HandleWriteABORT(bool in_isr, const uint8_t* req, uint16_t req_len,
                             uint8_t* resp, uint16_t resp_cap, uint16_t& out_len);
  ErrorCode HandleDelay(bool in_isr, const uint8_t* req, uint16_t req_len, uint8_t* resp,
                        uint16_t resp_cap, uint16_t& out_len);
  ErrorCode HandleResetTarget(bool in_isr, const uint8_t* req, uint16_t req_len,
                              uint8_t* resp, uint16_t resp_cap, uint16_t& out_len);
  ErrorCode HandleSWJPins(bool in_isr, const uint8_t* req, uint16_t req_len,
                          uint8_t* resp, uint16_t resp_cap, uint16_t& out_len);
  ErrorCode HandleSWJClock(bool in_isr, const uint8_t* req, uint16_t req_len,
                           uint8_t* resp, uint16_t resp_cap, uint16_t& out_len);
  ErrorCode HandleSWJSequence(bool in_isr, const uint8_t* req, uint16_t req_len,
                              uint8_t* resp, uint16_t resp_cap, uint16_t& out_len);
  ErrorCode HandleSWDConfigure(bool in_isr, const uint8_t* req, uint16_t req_len,
                               uint8_t* resp, uint16_t resp_cap, uint16_t& out_len);
  ErrorCode HandleSWDSequence(bool in_isr, const uint8_t* req, uint16_t req_len,
                              uint8_t* resp, uint16_t resp_cap, uint16_t& out_len);
  ErrorCode HandleJTAGSequence(bool in_isr, const uint8_t* req, uint16_t req_len,
                               uint8_t* resp, uint16_t resp_cap, uint16_t& out_len);
  ErrorCode HandleJTAGConfigure(bool in_isr, const uint8_t* req, uint16_t req_len,
                                uint8_t* resp, uint16_t resp_cap, uint16_t& out_len);
  ErrorCode HandleJTAGIdCode(bool in_isr, const uint8_t* req, uint16_t req_len,
                             uint8_t* resp, uint16_t resp_cap, uint16_t& out_len);
  ErrorCode HandleQueueCommands(bool in_isr, const uint8_t* req, uint16_t req_len,
                                uint8_t* resp, uint16_t resp_cap, uint16_t& out_len);
  ErrorCode HandleExecuteCommands(bool in_isr, const uint8_t* req, uint16_t req_len,
                                  uint8_t* resp, uint16_t resp_cap, uint16_t& out_len);

  ErrorCode BuildInfoStringResponse(uint8_t cmd, const char* str, uint8_t* resp,
                                    uint16_t resp_cap, uint16_t& out_len);
  ErrorCode BuildInfoU8Response(uint8_t cmd, uint8_t val, uint8_t* resp,
                                uint16_t resp_cap, uint16_t& out_len);
  ErrorCode BuildInfoU16Response(uint8_t cmd, uint16_t val, uint8_t* resp,
                                 uint16_t resp_cap, uint16_t& out_len);
  ErrorCode BuildInfoU32Response(uint8_t cmd, uint32_t val, uint8_t* resp,
                                 uint16_t resp_cap, uint16_t& out_len);

  uint8_t MapAckToDapResp(LibXR::Debug::SwdProtocol::Ack ack) const;
  uint8_t MapAckToDapResp(LibXR::Debug::JtagProtocol::Ack ack) const;
  ErrorCode HandleJtagTransfer(bool in_isr, const uint8_t* req, uint16_t req_len,
                               uint8_t* resp, uint16_t resp_cap, uint16_t& out_len);
  ErrorCode HandleJtagTransferBlock(bool in_isr, const uint8_t* req, uint16_t req_len,
                                    uint8_t* resp, uint16_t resp_cap, uint16_t& out_len);
  ErrorCode HandleJtagWriteAbort(bool in_isr, const uint8_t* req, uint16_t req_len,
                                 uint8_t* resp, uint16_t resp_cap, uint16_t& out_len);
  void SetTransferAbortFlag(bool on);
  void DriveReset(bool release);
  void ResetJtagChainState();

  /// @name 构造时固定 / Fixed at construction
  /// @{
  SwdPort& swd_;                        ///< SWD 链路 / SWD link
  LibXR::GPIO* nreset_gpio_ = nullptr;  ///< 可选的 nRESET / Optional nRESET
  const char* interface_string_;        ///< 接口字符串 / Interface string
  Endpoint::EPNumber in_num_;           ///< 中断 IN 端点号 / Interrupt IN number
  Endpoint::EPNumber out_num_;          ///< 中断 OUT 端点号 / Interrupt OUT number
  Callback<const Endpoint::Completion&> out_cb_;  ///< OUT 回调 / OUT callback
  Callback<const Endpoint::Completion&> in_cb_;   ///< IN 回调 / IN callback
  /// @}

  /// @name 启动前设置，Step 线程读 / Set before start, read by the Step thread
  /// @{
  LibXR::Debug::Jtag* jtag_ = nullptr;  ///< JTAG 链路 / JTAG link
  /// DAP_Info 字符串 / DAP_Info strings
  InfoStrings info_{"XRobot", "DAPLinkV1", "00000001", "1.0.0", "XRUSB",
                    "XRDAP",  "XRobot",    "DAP_DEMO", "0.1.0"};
  /// @}

  /// @name owner 与 Step 线程之间 / Between the owner and the Step thread
  /// @{
  /// 收到的报告，按到达顺序；owner 写入，Step 取走 / Received reports in arrival order;
  /// written by the owner, taken by Step
  Lease lease_ring_[LEASE_RING_SIZE] = {};
  std::atomic<uint32_t> lease_head_{0u};  ///< owner 写 / Written by the owner
  std::atomic<uint32_t> lease_tail_{0u};  ///< Step 写 / Written by Step
  /// 每个报告、每次绑定解绑发一次 / Posted per report and per bind or unbind
  LibXR::Semaphore cmd_sem_{0};
  /// 每次 IN 完成、每次解绑发一次 / Posted per IN completion and per unbind
  LibXR::Semaphore in_sem_{0};
  /// 会话号：绑定和解绑时 owner 加一，Step 读 / Session number: the owner adds one at
  /// bind and unbind, Step reads it
  std::atomic<uint32_t> epoch_{0u};
  /// 绑定的 IN 端点：owner 写，Step 借槽时读 / Bound IN endpoint: written by the owner,
  /// read by Step to borrow slots
  std::atomic<Endpoint*> in_ep_{nullptr};
  /// @}

  /// @name 仅 owner / Owner only
  /// @{
  Endpoint* data_in_ =
      nullptr;  ///< 中断 IN，绑定时非空 / Interrupt IN, non-null if bound
  Endpoint* data_out_ = nullptr;  ///< 中断 OUT，绑定时非空 / Interrupt OUT, if bound
  DescriptorBlock desc_block_{};  ///< WriteDescriptors 写出的块 / Written block
  /// SET_IDLE 设的空闲率，GET_IDLE 返回 / Idle rate set by SET_IDLE, returned by GET_IDLE
  uint8_t idle_rate_ = 0U;
  /// @}

  /// @name 仅 Step 线程 / Step thread only
  /// @{
  Lease cur_{};                   ///< 手里的报告 / Report in hand
  bool cur_done_ = false;         ///< 已执行，响应在 resp_buf_ / Executed, response ready
  uint8_t resp_buf_[MAX_RESP]{};  ///< 响应缓冲 / Response buffer
  uint32_t session_epoch_ = 0u;   ///< 已落实的 epoch / Epoch applied to the session
  LibXR::Debug::JtagProtocol::ChainConfig jtag_chain_{};  ///< JTAG 链 / JTAG chain
  uint8_t jtag_ir_len_[JTAG_MAX_DEVICES] = {};  ///< JTAG IR 长度 / JTAG IR lengths
  /// SWJ 引脚电平的影子 / Shadow of the SWJ pin levels
  uint8_t swj_shadow_ = static_cast<uint8_t>(LibXR::USB::DapLinkV1Def::DAP_SWJ_SWDIO_TMS |
                                             LibXR::USB::DapLinkV1Def::DAP_SWJ_NRESET);
  bool last_nreset_level_high_ = true;  ///< 上一次 nRESET 电平 / Last nRESET level
  LibXR::USB::DapLinkV1Def::State dap_state_{};  ///< DAP 状态 / DAP state
  uint32_t swj_clock_hz_ = 1000000u;             ///< SWJ 时钟 / SWJ clock (Hz)
  uint32_t match_mask_ = 0xFFFFFFFFu;            ///< 匹配掩码 / Match mask
  /// @}
};

template <typename SwdPort>
DapLinkV1Class<SwdPort>::DapLinkV1Class(Endpoint::EPNumber in_ep_num,
                                        Endpoint::EPNumber out_ep_num, SwdPort& swd_link,
                                        LibXR::GPIO* nreset_gpio,
                                        const char* interface_string)
    : swd_(swd_link),
      nreset_gpio_(nreset_gpio),
      interface_string_(interface_string),
      in_num_(in_ep_num),
      out_num_(out_ep_num),
      out_cb_(
          Callback<const Endpoint::Completion&>::Create(OnDataOutCompleteStatic, this)),
      in_cb_(Callback<const Endpoint::Completion&>::Create(OnDataInCompleteStatic, this))
{
  if constexpr (JTAG_ENABLED)
  {
    jtag_chain_.ir_length = jtag_ir_len_;
    jtag_chain_.count = 0u;
    jtag_chain_.index = 0u;
  }
}

template <typename SwdPort>
void DapLinkV1Class<SwdPort>::SetInfoStrings(const InfoStrings& info)
{
  info_ = info;
}

template <typename SwdPort>
void DapLinkV1Class<SwdPort>::SetJtag(LibXR::Debug::Jtag* jtag)
{
  if constexpr (JTAG_ENABLED)
  {
    jtag_ = jtag;
  }
  else
  {
    (void)jtag;
  }
}

template <typename SwdPort>
const LibXR::USB::DapLinkV1Def::State& DapLinkV1Class<SwdPort>::GetState() const
{
  return dap_state_;
}

template <typename SwdPort>
const char* DapLinkV1Class<SwdPort>::GetInterfaceString(
    size_t local_interface_index) const
{
  return (local_interface_index == 0U) ? interface_string_ : nullptr;
}

template <typename SwdPort>
ErrorCode DapLinkV1Class<SwdPort>::WriteDeviceDescriptor(DeviceDescriptor& descriptor)
{
  descriptor.data_.bDeviceClass = DeviceDescriptor::ClassID::PER_INTERFACE;
  descriptor.data_.bDeviceSubClass = 0;
  descriptor.data_.bDeviceProtocol = 0;
  return ErrorCode::OK;
}

template <typename SwdPort>
ErrorCode DapLinkV1Class<SwdPort>::WriteDescriptors(const EndpointPool& pool, Speed speed,
                                                    uint8_t first_interface,
                                                    ConstRawData& block)
{
  // 报告固定 64 字节，必须正好是一个包。
  // Reports are 64 bytes and must be exactly one packet.
  bool ok = WriteEndpointDescriptor(pool, speed, in_num_, IN_CONFIG, INTERVAL,
                                    desc_block_.ep_in);
  ok = WriteEndpointDescriptor(pool, speed, out_num_, OUT_CONFIG, INTERVAL,
                               desc_block_.ep_out) &&
       ok;
  if (!ok || desc_block_.ep_in.wMaxPacketSize != MAX_RESP ||
      desc_block_.ep_out.wMaxPacketSize != MAX_REQ)
  {
    return ErrorCode::NOT_SUPPORT;
  }
  desc_block_.intf = {9U,
                      static_cast<uint8_t>(DescriptorType::INTERFACE),
                      first_interface,
                      0U,
                      2U,
                      CLASS_HID,
                      0U,
                      0U,
                      GetInterfaceStringIndex(0U)};
  desc_block_.hid = HID_DESCRIPTOR;
  block = {&desc_block_, sizeof(desc_block_)};
  return ErrorCode::OK;
}

template <typename SwdPort>
ErrorCode DapLinkV1Class<SwdPort>::BindEndpoints(EndpointPool& pool,
                                                 uint8_t first_interface, bool in_isr)
{
  UNUSED(first_interface);
  ErrorCode result = BindEndpoint(pool, in_num_, IN_CONFIG,
                                  desc_block_.ep_in.wMaxPacketSize, in_isr, data_in_);
  if (result == ErrorCode::OK)
  {
    result = BindEndpoint(pool, out_num_, OUT_CONFIG, desc_block_.ep_out.wMaxPacketSize,
                          in_isr, data_out_);
  }
  if (result != ErrorCode::OK)
  {
    return result;
  }
  data_in_->SetOnComplete(in_cb_);
  data_out_->SetOnComplete(out_cb_);
  idle_rate_ = 0U;
  // 新会话：上一个会话留在环里的报告由 Step 丢弃，会话状态由 Step 在第一条命令前复位。
  // A new session: reports of the previous session left in the ring are dropped by
  // Step, which also resets the session state before the first command.
  epoch_.fetch_add(1u, std::memory_order_acq_rel);
  in_ep_.store(data_in_, std::memory_order_release);
  TopUpReceives(in_isr);
  cmd_sem_.PostFromCallback(in_isr);
  return ErrorCode::OK;
}

template <typename SwdPort>
void DapLinkV1Class<SwdPort>::UnbindEndpoints(EndpointPool& pool, bool in_isr)
{
  in_ep_.store(nullptr, std::memory_order_release);
  epoch_.fetch_add(1u, std::memory_order_acq_rel);
  UnbindEndpoint(pool, data_in_, in_isr);
  UnbindEndpoint(pool, data_out_, in_isr);
  cmd_sem_.PostFromCallback(in_isr);
  in_sem_.PostFromCallback(in_isr);
}

template <typename SwdPort>
bool DapLinkV1Class<SwdPort>::OwnsEndpoint(uint8_t ep_addr) const
{
  return ep_addr == Endpoint::EPNumberToAddr(in_num_, Endpoint::Direction::IN) ||
         ep_addr == Endpoint::EPNumberToAddr(out_num_, Endpoint::Direction::OUT);
}

template <typename SwdPort>
ErrorCode DapLinkV1Class<SwdPort>::OnGetDescriptor(bool in_isr, const SetupPacket& setup,
                                                   ConstRawData& data)
{
  UNUSED(in_isr);
  switch (setup.wValue >> 8U)
  {
    case 0x21U:
      data = {&HID_DESCRIPTOR, sizeof(HID_DESCRIPTOR)};
      return ErrorCode::OK;
    case 0x22U:
      data = {REPORT_DESC, sizeof(REPORT_DESC)};
      return ErrorCode::OK;
    default:
      return ErrorCode::NOT_SUPPORT;
  }
}

template <typename SwdPort>
ErrorCode DapLinkV1Class<SwdPort>::OnControlRequest(bool in_isr, const SetupPacket& setup,
                                                    ConstRawData& in_data)
{
  UNUSED(in_isr);
  const auto type = static_cast<RequestType>(setup.bmRequestType & REQ_TYPE_MASK);
  const auto recipient = static_cast<Recipient>(setup.bmRequestType & REQ_RECIPIENT_MASK);
  if (type != RequestType::CLASS || recipient != Recipient::INTERFACE)
  {
    return ErrorCode::NOT_SUPPORT;
  }
  const bool input = (setup.bmRequestType & REQ_DIRECTION_MASK) != 0U;
  switch (setup.bRequest)
  {
    case REQUEST_SET_IDLE:
      // wValue 高字节是空闲率，低字节是报告号；本类只有报告 0。
      // The high byte of wValue is the rate, the low byte the report ID; only report 0.
      if (input || setup.wLength != 0U || (setup.wValue & 0xFFU) != 0U)
      {
        return ErrorCode::ARG_ERR;
      }
      idle_rate_ = static_cast<uint8_t>(setup.wValue >> 8U);
      return ErrorCode::OK;
    case REQUEST_GET_IDLE:
      if (!input || setup.wLength != 1U || (setup.wValue & 0xFFU) != 0U)
      {
        return ErrorCode::ARG_ERR;
      }
      // idle_rate_ 只被 SET_IDLE 改，它到来之前本次传输已经结束。
      // idle_rate_ changes only with SET_IDLE, which arrives after this transfer ended.
      in_data = {&idle_rate_, 1U};
      return ErrorCode::OK;
    default:
      return ErrorCode::NOT_SUPPORT;
  }
}

template <typename SwdPort>
void DapLinkV1Class<SwdPort>::OnService(bool in_isr)
{
  if (data_out_ != nullptr)
  {
    TopUpReceives(in_isr);
  }
}

template <typename SwdPort>
void DapLinkV1Class<SwdPort>::TopUpReceives(bool in_isr)
{
  // 每个接收一个报告；没有空闲槽（借给 Step 或已在收）时 Receive 返回 BUSY。
  // One report per receive; with no free slot (lent to Step or receiving) Receive
  // returns BUSY.
  while (data_out_->Receive(MAX_REQ, in_isr) == ErrorCode::OK)
  {
  }
}

template <typename SwdPort>
void DapLinkV1Class<SwdPort>::OnDataOutCompleteStatic(
    bool in_isr, DapLinkV1Class* self, const Endpoint::Completion& completion)
{
  if (self->data_out_ == nullptr)
  {
    // 未绑定时 OUT 只会以 ABORTED 结束，不借出槽（见 Endpoint::Close）。
    // While unbound an OUT only ends with ABORTED and lends no slot (see
    // Endpoint::Close).
    ASSERT(!completion.slot.Valid());
    return;
  }
  if (completion.result != ErrorCode::OK)
  {
    // Stall 或 Flush 停了接收：重新挂上；STALLED 期间挂的排队到 ClearStall。
    // A Stall or Flush stopped the receive: arm it again; one armed while STALLED waits
    // until ClearStall.
    self->TopUpReceives(in_isr);
    return;
  }
  // 环按最多的槽数分配：借出的槽不会多于环的容量。先写环再发布。
  // The ring is sized for the most slots: the lent slots never exceed it. The entry is
  // written before it is published.
  const uint32_t head = self->lease_head_.load(std::memory_order_relaxed);
  ASSERT(head - self->lease_tail_.load(std::memory_order_acquire) < LEASE_RING_SIZE);
  self->lease_ring_[head % LEASE_RING_SIZE] = {
      completion.slot, self->data_out_, self->epoch_.load(std::memory_order_relaxed)};
  self->lease_head_.store(head + 1u, std::memory_order_release);
  self->cmd_sem_.PostFromCallback(in_isr);
}

template <typename SwdPort>
void DapLinkV1Class<SwdPort>::OnDataInCompleteStatic(
    bool in_isr, DapLinkV1Class* self, const Endpoint::Completion& completion)
{
  UNUSED(completion);
  if (self->data_in_ != nullptr)
  {
    self->in_sem_.PostFromCallback(in_isr);
  }
}

template <typename SwdPort>
bool DapLinkV1Class<SwdPort>::TakeLease(Lease& lease)
{
  const uint32_t tail = lease_tail_.load(std::memory_order_relaxed);
  if (tail == lease_head_.load(std::memory_order_acquire))
  {
    return false;
  }
  lease = lease_ring_[tail % LEASE_RING_SIZE];
  lease_tail_.store(tail + 1u, std::memory_order_release);
  return true;
}

template <typename SwdPort>
void DapLinkV1Class<SwdPort>::DropCurrent()
{
  // 当前绑定的槽变空闲，解绑作废的槽也交回 OK；补挂由 owner 在 OnService 里做，所以
  // 本线程不会在已换绑的端点上挂接收。
  // A slot of the current binding becomes free and one made stale by an unbind is
  // handed back with OK too; the owner re-arms in OnService, so this thread never arms
  // a receive on an endpoint that has been rebound.
  if (cur_.slot.Valid())
  {
    (void)cur_.ep->ReleaseSlot(cur_.slot);
    RequestService(false);
  }
  cur_ = {};
  cur_done_ = false;
}

template <typename SwdPort>
bool DapLinkV1Class<SwdPort>::SubmitCurrent()
{
  Endpoint* in = in_ep_.load(std::memory_order_acquire);
  if (in == nullptr)
  {
    return false;
  }
  Endpoint::Slot slot;
  if (in->AcquireSlot(slot) != ErrorCode::OK)
  {
    return false;
  }
  // 借到之后再比 epoch：读引用与借之间可能已解绑并换了别的绑定。解绑先于 Close 把 epoch
  // 加一，借出的比较交换看到新绑定时这里一定看到加一（EndpointSlots 的比较交换都是
  // acq_rel，推断）；比较之后才解绑的话槽作废，Submit 得 ABORTED。
  // Compare the epoch after the lend: an unbind and another bind may have come between
  // reading the reference and the lend. The unbind moves the epoch before Close, so
  // once the lending compare-and-swap sees the new binding this load sees the increment
  // (every compare-and-swap of EndpointSlots is acq_rel; inferred); an unbind after the
  // comparison makes the slot stale and Submit returns ABORTED.
  if (cur_.epoch != epoch_.load(std::memory_order_acquire))
  {
    (void)in->ReleaseSlot(slot);
    return false;
  }
  // 包长是 64（WriteDescriptors），槽至少一个包。
  // The packet is 64 bytes (WriteDescriptors) and a slot holds at least one packet.
  ASSERT(slot.data.size_ >= MAX_RESP);
  Memory::FastCopy(slot.data.addr_, resp_buf_, MAX_RESP);
  const ErrorCode result = in->Submit(slot, MAX_RESP, false, false);
  ASSERT(result == ErrorCode::OK || result == ErrorCode::ABORTED);
  return result == ErrorCode::OK;
}

template <typename SwdPort>
ErrorCode DapLinkV1Class<SwdPort>::Step(uint32_t timeout_ms)
{
  if (!cur_.slot.Valid())
  {
    if (!TakeLease(cur_))
    {
      if (cmd_sem_.Wait(timeout_ms) != ErrorCode::OK)
      {
        return ErrorCode::TIMEOUT;
      }
      if (!TakeLease(cur_))
      {
        // 被绑定/解绑叫醒，没有报告：把会话的开始/结束落实到 SWD。
        // Woken by a bind/unbind with no report: apply the session start/end to SWD.
        SyncSession(epoch_.load(std::memory_order_acquire));
        return ErrorCode::OK;
      }
    }
  }

  if (!cur_done_)
  {
    // 收到之后解绑或重新绑定过：属于已结束的会话，丢弃。
    // Unbound or rebound since it was received: it belongs to an ended session.
    if (cur_.epoch != epoch_.load(std::memory_order_acquire))
    {
      DropCurrent();
      SyncSession(epoch_.load(std::memory_order_acquire));
      return ErrorCode::OK;
    }
    SyncSession(cur_.epoch);
    Memory::FastSet(resp_buf_, 0, MAX_RESP);
    uint16_t length = 0;
    const auto result = ProcessOneCommand(
        false, static_cast<const uint8_t*>(cur_.slot.data.addr_),
        static_cast<uint16_t>(cur_.slot.data.size_), resp_buf_, MAX_RESP, length);
    UNUSED(result);
    ASSERT(length <= MAX_RESP);
    cur_done_ = true;
  }

  // IN 槽都忙时等主机取走一个；超时返回时报告和响应仍由本线程持有。
  // With every IN slot busy wait for the host to take one; on a timeout the report and
  // the response stay with this thread.
  while (true)
  {
    if (cur_.epoch != epoch_.load(std::memory_order_acquire))
    {
      DropCurrent();
      SyncSession(epoch_.load(std::memory_order_acquire));
      return ErrorCode::OK;
    }
    if (SubmitCurrent())
    {
      DropCurrent();
      return ErrorCode::OK;
    }
    if (in_sem_.Wait(timeout_ms) != ErrorCode::OK)
    {
      return ErrorCode::TIMEOUT;
    }
  }
}

template <typename SwdPort>
void DapLinkV1Class<SwdPort>::SyncSession(uint32_t epoch)
{
  if (epoch == session_epoch_)
  {
    return;
  }
  session_epoch_ = epoch;

  swd_.Close();
  if constexpr (JTAG_ENABLED)
  {
    if (jtag_ != nullptr)
    {
      jtag_->Close();
    }
  }

  match_mask_ = 0xFFFFFFFFu;
  dap_state_ = {};
  dap_state_.debug_port = LibXR::USB::DapLinkV1Def::DebugPort::DISABLED;
  dap_state_.transfer_abort = false;

  swj_clock_hz_ = 1000000u;
  (void)swd_.SetClockHz(swj_clock_hz_);
  if constexpr (JTAG_ENABLED)
  {
    if (jtag_ != nullptr)
    {
      (void)jtag_->SetClockHz(swj_clock_hz_);
    }
  }

  if constexpr (JTAG_ENABLED)
  {
    ResetJtagChainState();
  }

  last_nreset_level_high_ = true;
  swj_shadow_ = static_cast<uint8_t>(LibXR::USB::DapLinkV1Def::DAP_SWJ_SWDIO_TMS |
                                     LibXR::USB::DapLinkV1Def::DAP_SWJ_NRESET);
}

template <typename SwdPort>
ErrorCode DapLinkV1Class<SwdPort>::ProcessOneCommand(bool in_isr, const uint8_t* req,
                                                     uint16_t req_len, uint8_t* resp,
                                                     uint16_t resp_cap, uint16_t& out_len)
{
  out_len = 0u;

  if (!req || !resp || req_len < 1u || resp_cap < 1u)
  {
    if (resp && resp_cap >= 1u)
    {
      BuildNotSupportResponse(resp, resp_cap, out_len);
    }
    return ErrorCode::ARG_ERR;
  }

  const uint8_t CMD = req[0];

  switch (CMD)
  {
    case to_u8(LibXR::USB::DapLinkV1Def::CommandId::INFO):
      return HandleInfo(in_isr, req, req_len, resp, resp_cap, out_len);
    case to_u8(LibXR::USB::DapLinkV1Def::CommandId::HOST_STATUS):
      return HandleHostStatus(in_isr, req, req_len, resp, resp_cap, out_len);
    case to_u8(LibXR::USB::DapLinkV1Def::CommandId::CONNECT):
      return HandleConnect(in_isr, req, req_len, resp, resp_cap, out_len);
    case to_u8(LibXR::USB::DapLinkV1Def::CommandId::DISCONNECT):
      return HandleDisconnect(in_isr, req, req_len, resp, resp_cap, out_len);
    case to_u8(LibXR::USB::DapLinkV1Def::CommandId::TRANSFER_CONFIGURE):
      return HandleTransferConfigure(in_isr, req, req_len, resp, resp_cap, out_len);
    case to_u8(LibXR::USB::DapLinkV1Def::CommandId::TRANSFER):
      return HandleTransfer(in_isr, req, req_len, resp, resp_cap, out_len);
    case to_u8(LibXR::USB::DapLinkV1Def::CommandId::TRANSFER_BLOCK):
      return HandleTransferBlock(in_isr, req, req_len, resp, resp_cap, out_len);
    case to_u8(LibXR::USB::DapLinkV1Def::CommandId::TRANSFER_ABORT):
      return HandleTransferAbort(in_isr, req, req_len, resp, resp_cap, out_len);
    case to_u8(LibXR::USB::DapLinkV1Def::CommandId::WRITE_ABORT):
      return HandleWriteABORT(in_isr, req, req_len, resp, resp_cap, out_len);
    case to_u8(LibXR::USB::DapLinkV1Def::CommandId::DELAY):
      return HandleDelay(in_isr, req, req_len, resp, resp_cap, out_len);
    case to_u8(LibXR::USB::DapLinkV1Def::CommandId::RESET_TARGET):
      return HandleResetTarget(in_isr, req, req_len, resp, resp_cap, out_len);
    case to_u8(LibXR::USB::DapLinkV1Def::CommandId::SWJ_PINS):
      return HandleSWJPins(in_isr, req, req_len, resp, resp_cap, out_len);
    case to_u8(LibXR::USB::DapLinkV1Def::CommandId::SWJ_CLOCK):
      return HandleSWJClock(in_isr, req, req_len, resp, resp_cap, out_len);
    case to_u8(LibXR::USB::DapLinkV1Def::CommandId::SWJ_SEQUENCE):
      return HandleSWJSequence(in_isr, req, req_len, resp, resp_cap, out_len);
    case to_u8(LibXR::USB::DapLinkV1Def::CommandId::SWD_CONFIGURE):
      return HandleSWDConfigure(in_isr, req, req_len, resp, resp_cap, out_len);
    case to_u8(LibXR::USB::DapLinkV1Def::CommandId::SWD_SEQUENCE):
      return HandleSWDSequence(in_isr, req, req_len, resp, resp_cap, out_len);
    case to_u8(LibXR::USB::DapLinkV1Def::CommandId::JTAG_SEQUENCE):
      if constexpr (JTAG_ENABLED)
      {
        return HandleJTAGSequence(in_isr, req, req_len, resp, resp_cap, out_len);
      }
      BuildNotSupportResponse(resp, resp_cap, out_len);
      return ErrorCode::NOT_SUPPORT;
    case to_u8(LibXR::USB::DapLinkV1Def::CommandId::JTAG_CONFIGURE):
      if constexpr (JTAG_ENABLED)
      {
        return HandleJTAGConfigure(in_isr, req, req_len, resp, resp_cap, out_len);
      }
      BuildNotSupportResponse(resp, resp_cap, out_len);
      return ErrorCode::NOT_SUPPORT;
    case to_u8(LibXR::USB::DapLinkV1Def::CommandId::JTAG_IDCODE):
      if constexpr (JTAG_ENABLED)
      {
        return HandleJTAGIdCode(in_isr, req, req_len, resp, resp_cap, out_len);
      }
      BuildNotSupportResponse(resp, resp_cap, out_len);
      return ErrorCode::NOT_SUPPORT;
    case to_u8(LibXR::USB::DapLinkV1Def::CommandId::QUEUE_COMMANDS):
      return HandleQueueCommands(in_isr, req, req_len, resp, resp_cap, out_len);
    case to_u8(LibXR::USB::DapLinkV1Def::CommandId::EXECUTE_COMMANDS):
      return HandleExecuteCommands(in_isr, req, req_len, resp, resp_cap, out_len);
    default:
      (void)build_unknown_cmd_response(resp, resp_cap, out_len);
      return ErrorCode::NOT_SUPPORT;
  }
}

template <typename SwdPort>
void DapLinkV1Class<SwdPort>::BuildNotSupportResponse(uint8_t* resp, uint16_t resp_cap,
                                                      uint16_t& out_len)
{
  if (!resp || resp_cap < 1u)
  {
    out_len = 0u;
    return;
  }
  resp[0] = 0xFFu;
  out_len = 1u;
}

template <typename SwdPort>
ErrorCode DapLinkV1Class<SwdPort>::HandleInfo(bool /*in_isr*/, const uint8_t* req,
                                              uint16_t req_len, uint8_t* resp,
                                              uint16_t resp_cap, uint16_t& out_len)
{
  if (req_len < 2u)
  {
    resp[0] = to_u8(LibXR::USB::DapLinkV1Def::CommandId::INFO);
    resp[1] = 0u;
    out_len = 2u;
    return ErrorCode::ARG_ERR;
  }

  const uint8_t INFO_ID = req[1];

  resp[0] = to_u8(LibXR::USB::DapLinkV1Def::CommandId::INFO);

  switch (INFO_ID)
  {
    case to_u8(LibXR::USB::DapLinkV1Def::InfoId::VENDOR):
      return BuildInfoStringResponse(resp[0], info_.vendor, resp, resp_cap, out_len);
    case to_u8(LibXR::USB::DapLinkV1Def::InfoId::PRODUCT):
      return BuildInfoStringResponse(resp[0], info_.product, resp, resp_cap, out_len);
    case to_u8(LibXR::USB::DapLinkV1Def::InfoId::SERIAL_NUMBER):
      return BuildInfoStringResponse(resp[0], info_.serial, resp, resp_cap, out_len);
    case to_u8(LibXR::USB::DapLinkV1Def::InfoId::FIRMWARE_VERSION):
      return BuildInfoStringResponse(resp[0], info_.firmware_ver, resp, resp_cap,
                                     out_len);
    case to_u8(LibXR::USB::DapLinkV1Def::InfoId::DEVICE_VENDOR):
      return BuildInfoStringResponse(resp[0], info_.device_vendor, resp, resp_cap,
                                     out_len);
    case to_u8(LibXR::USB::DapLinkV1Def::InfoId::DEVICE_NAME):
      return BuildInfoStringResponse(resp[0], info_.device_name, resp, resp_cap, out_len);
    case to_u8(LibXR::USB::DapLinkV1Def::InfoId::BOARD_VENDOR):
      return BuildInfoStringResponse(resp[0], info_.board_vendor, resp, resp_cap,
                                     out_len);
    case to_u8(LibXR::USB::DapLinkV1Def::InfoId::BOARD_NAME):
      return BuildInfoStringResponse(resp[0], info_.board_name, resp, resp_cap, out_len);
    case to_u8(LibXR::USB::DapLinkV1Def::InfoId::PRODUCT_FIRMWARE_VERSION):
      return BuildInfoStringResponse(resp[0], info_.product_fw_ver, resp, resp_cap,
                                     out_len);
    case to_u8(LibXR::USB::DapLinkV1Def::InfoId::CAPABILITIES):
    {
      uint8_t caps = LibXR::USB::DapLinkV1Def::DAP_CAP_SWD;
      if constexpr (JTAG_ENABLED)
      {
        if (jtag_ != nullptr)
        {
          caps = static_cast<uint8_t>(caps | LibXR::USB::DapLinkV1Def::DAP_CAP_JTAG);
        }
      }
      return BuildInfoU8Response(resp[0], caps, resp, resp_cap, out_len);
    }
    case to_u8(LibXR::USB::DapLinkV1Def::InfoId::PACKET_COUNT):
      return BuildInfoU8Response(resp[0], 1, resp, resp_cap, out_len);
    case to_u8(LibXR::USB::DapLinkV1Def::InfoId::PACKET_SIZE):
      return BuildInfoU16Response(resp[0], MAX_RESP, resp, resp_cap, out_len);
    case to_u8(LibXR::USB::DapLinkV1Def::InfoId::TIMESTAMP_CLOCK):
      return BuildInfoU32Response(resp[0], 1000000U, resp, resp_cap, out_len);
    default:
      resp[1] = 0u;
      out_len = 2u;
      return ErrorCode::OK;
  }
}

template <typename SwdPort>
ErrorCode DapLinkV1Class<SwdPort>::BuildInfoStringResponse(uint8_t cmd, const char* str,
                                                           uint8_t* resp,
                                                           uint16_t resp_cap,
                                                           uint16_t& out_len)
{
  resp[0] = cmd;
  resp[1] = 0u;

  if (!str)
  {
    out_len = 2u;
    return ErrorCode::OK;
  }

  const size_t N_WITH_NUL = std::strlen(str) + 1;
  const size_t MAX_PAYLOAD = (resp_cap >= 2u) ? (resp_cap - 2u) : 0u;
  if (MAX_PAYLOAD == 0u)
  {
    out_len = 2u;
    return ErrorCode::OK;
  }

  const size_t COPY_N = (N_WITH_NUL > MAX_PAYLOAD) ? MAX_PAYLOAD : N_WITH_NUL;
  Memory::FastCopy(&resp[2], str, COPY_N);
  resp[1] = static_cast<uint8_t>(COPY_N);
  out_len = static_cast<uint16_t>(COPY_N + 2u);
  return ErrorCode::OK;
}

template <typename SwdPort>
ErrorCode DapLinkV1Class<SwdPort>::BuildInfoU8Response(uint8_t cmd, uint8_t val,
                                                       uint8_t* resp, uint16_t resp_cap,
                                                       uint16_t& out_len)
{
  if (resp_cap < 3u)
  {
    out_len = 0u;
    return ErrorCode::NOT_FOUND;
  }
  resp[0] = cmd;
  resp[1] = 1u;
  resp[2] = val;
  out_len = 3u;
  return ErrorCode::OK;
}

template <typename SwdPort>
ErrorCode DapLinkV1Class<SwdPort>::BuildInfoU16Response(uint8_t cmd, uint16_t val,
                                                        uint8_t* resp, uint16_t resp_cap,
                                                        uint16_t& out_len)
{
  if (resp_cap < 4u)
  {
    out_len = 0u;
    return ErrorCode::NOT_FOUND;
  }
  resp[0] = cmd;
  resp[1] = 2u;
  Memory::FastCopy(&resp[2], &val, sizeof(val));
  out_len = 4u;
  return ErrorCode::OK;
}

template <typename SwdPort>
ErrorCode DapLinkV1Class<SwdPort>::BuildInfoU32Response(uint8_t cmd, uint32_t val,
                                                        uint8_t* resp, uint16_t resp_cap,
                                                        uint16_t& out_len)
{
  if (resp_cap < 6u)
  {
    out_len = 0u;
    return ErrorCode::NOT_FOUND;
  }
  resp[0] = cmd;
  resp[1] = 4u;
  Memory::FastCopy(&resp[2], &val, sizeof(val));
  out_len = 6u;
  return ErrorCode::OK;
}

template <typename SwdPort>
ErrorCode DapLinkV1Class<SwdPort>::HandleHostStatus(bool /*in_isr*/,
                                                    const uint8_t* /*req*/,
                                                    uint16_t /*req_len*/, uint8_t* resp,
                                                    uint16_t resp_cap, uint16_t& out_len)
{
  if (resp_cap < 2u)
  {
    out_len = 0u;
    return ErrorCode::NOT_FOUND;
  }
  resp[0] = to_u8(LibXR::USB::DapLinkV1Def::CommandId::HOST_STATUS);
  resp[1] = to_u8(LibXR::USB::DapLinkV1Def::Status::OK);
  out_len = 2u;
  return ErrorCode::OK;
}

template <typename SwdPort>
ErrorCode DapLinkV1Class<SwdPort>::HandleConnect(bool /*in_isr*/, const uint8_t* req,
                                                 uint16_t req_len, uint8_t* resp,
                                                 uint16_t resp_cap, uint16_t& out_len)
{
  if (resp_cap < 2u)
  {
    out_len = 0u;
    return ErrorCode::NOT_FOUND;
  }

  resp[0] = to_u8(LibXR::USB::DapLinkV1Def::CommandId::CONNECT);

  uint8_t port = 0u;
  if (req_len >= 2u)
  {
    port = req[1];
  }

  if (port == 0u)
  {
    port = to_u8(LibXR::USB::DapLinkV1Def::Port::SWD);
  }

  if (port == to_u8(LibXR::USB::DapLinkV1Def::Port::SWD))
  {
    (void)swd_.EnterSwd();
    (void)swd_.SetClockHz(swj_clock_hz_);

    dap_state_.debug_port = LibXR::USB::DapLinkV1Def::DebugPort::SWD;
    dap_state_.transfer_abort = false;

    resp[1] = to_u8(LibXR::USB::DapLinkV1Def::Port::SWD);
  }
  else if (port == to_u8(LibXR::USB::DapLinkV1Def::Port::JTAG))
  {
    if constexpr (!JTAG_ENABLED)
    {
      resp[1] = to_u8(LibXR::USB::DapLinkV1Def::Port::DISABLED);
    }
    else
    {
      if (jtag_ == nullptr)
      {
        resp[1] = to_u8(LibXR::USB::DapLinkV1Def::Port::DISABLED);
      }
      else
      {
        static constexpr uint8_t SWD_TO_JTAG_SEQUENCE[2] = {0x3Cu, 0xE7u};
        (void)swd_.SetClockHz(swj_clock_hz_);
        (void)swd_.LineReset();
        (void)swd_.SeqWriteBits(16u, SWD_TO_JTAG_SEQUENCE);
        (void)swd_.LineReset();
        (void)jtag_->SetClockHz(swj_clock_hz_);
        (void)jtag_->ResetTap();

        dap_state_.debug_port = LibXR::USB::DapLinkV1Def::DebugPort::JTAG;
        dap_state_.transfer_abort = false;
        ResetJtagChainState();
        resp[1] = to_u8(LibXR::USB::DapLinkV1Def::Port::JTAG);
      }
    }
  }
  else
  {
    resp[1] = to_u8(LibXR::USB::DapLinkV1Def::Port::DISABLED);
  }

  out_len = 2u;
  return ErrorCode::OK;
}

template <typename SwdPort>
ErrorCode DapLinkV1Class<SwdPort>::HandleDisconnect(bool /*in_isr*/,
                                                    const uint8_t* /*req*/,
                                                    uint16_t /*req_len*/, uint8_t* resp,
                                                    uint16_t resp_cap, uint16_t& out_len)
{
  if (resp_cap < 2u)
  {
    out_len = 0u;
    return ErrorCode::NOT_FOUND;
  }

  swd_.Close();
  if constexpr (JTAG_ENABLED)
  {
    if (jtag_ != nullptr)
    {
      jtag_->Close();
    }
  }
  dap_state_.debug_port = LibXR::USB::DapLinkV1Def::DebugPort::DISABLED;
  dap_state_.transfer_abort = false;

  resp[0] = to_u8(LibXR::USB::DapLinkV1Def::CommandId::DISCONNECT);
  resp[1] = to_u8(LibXR::USB::DapLinkV1Def::Status::OK);
  out_len = 2u;
  return ErrorCode::OK;
}

template <typename SwdPort>
ErrorCode DapLinkV1Class<SwdPort>::HandleTransferConfigure(
    bool /*in_isr*/, const uint8_t* req, uint16_t req_len, uint8_t* resp,
    uint16_t resp_cap, uint16_t& out_len)
{
  if (resp_cap < 2u)
  {
    out_len = 0u;
    return ErrorCode::NOT_FOUND;
  }

  resp[0] = to_u8(LibXR::USB::DapLinkV1Def::CommandId::TRANSFER_CONFIGURE);

  if (req_len < 6u)
  {
    resp[1] = to_u8(LibXR::USB::DapLinkV1Def::Status::ERROR);
    out_len = 2u;
    return ErrorCode::ARG_ERR;
  }

  const uint8_t IDLE = req[1];

  uint16_t wait_retry = 0u;
  uint16_t match_retry = 0u;
  Memory::FastCopy(&wait_retry, &req[2], sizeof(wait_retry));
  Memory::FastCopy(&match_retry, &req[4], sizeof(match_retry));

  dap_state_.transfer_cfg.idle_cycles = IDLE;
  dap_state_.transfer_cfg.retry_count = wait_retry;
  dap_state_.transfer_cfg.match_retry = match_retry;

  LibXR::Debug::Swd::TransferPolicy pol = swd_.GetTransferPolicy();
  pol.idle_cycles = IDLE;
  pol.wait_retry = wait_retry;
  swd_.SetTransferPolicy(pol);

  resp[1] = to_u8(LibXR::USB::DapLinkV1Def::Status::OK);
  out_len = 2u;
  return ErrorCode::OK;
}

template <typename SwdPort>
ErrorCode DapLinkV1Class<SwdPort>::HandleTransferAbort(bool /*in_isr*/,
                                                       const uint8_t* /*req*/,
                                                       uint16_t /*req_len*/,
                                                       uint8_t* resp, uint16_t resp_cap,
                                                       uint16_t& out_len)
{
  if (resp_cap < 2u)
  {
    out_len = 0u;
    return ErrorCode::NOT_FOUND;
  }

  SetTransferAbortFlag(true);

  resp[0] = to_u8(LibXR::USB::DapLinkV1Def::CommandId::TRANSFER_ABORT);
  resp[1] = to_u8(LibXR::USB::DapLinkV1Def::Status::OK);
  out_len = 2u;
  return ErrorCode::OK;
}

template <typename SwdPort>
ErrorCode DapLinkV1Class<SwdPort>::HandleWriteABORT(bool /*in_isr*/, const uint8_t* req,
                                                    uint16_t req_len, uint8_t* resp,
                                                    uint16_t resp_cap, uint16_t& out_len)
{
  if (resp_cap < 2u)
  {
    out_len = 0u;
    return ErrorCode::NOT_FOUND;
  }

  if constexpr (JTAG_ENABLED)
  {
    if (dap_state_.debug_port == LibXR::USB::DapLinkV1Def::DebugPort::JTAG)
    {
      return HandleJtagWriteAbort(false, req, req_len, resp, resp_cap, out_len);
    }
  }

  resp[0] = to_u8(LibXR::USB::DapLinkV1Def::CommandId::WRITE_ABORT);

  if (req_len < 6u)
  {
    resp[1] = to_u8(LibXR::USB::DapLinkV1Def::Status::ERROR);
    out_len = 2u;
    return ErrorCode::ARG_ERR;
  }

  uint32_t flags = 0u;
  Memory::FastCopy(&flags, &req[2], sizeof(flags));

  LibXR::Debug::SwdProtocol::Ack ack = LibXR::Debug::SwdProtocol::Ack::PROTOCOL;
  const ErrorCode EC = swd_.WriteAbortTxn(flags, ack);
  resp[1] = (EC == ErrorCode::OK && ack == LibXR::Debug::SwdProtocol::Ack::OK)
                ? to_u8(LibXR::USB::DapLinkV1Def::Status::OK)
                : to_u8(LibXR::USB::DapLinkV1Def::Status::ERROR);

  out_len = 2u;
  return ErrorCode::OK;
}

template <typename SwdPort>
ErrorCode DapLinkV1Class<SwdPort>::HandleDelay(bool /*in_isr*/, const uint8_t* req,
                                               uint16_t req_len, uint8_t* resp,
                                               uint16_t resp_cap, uint16_t& out_len)
{
  if (resp_cap < 2u)
  {
    out_len = 0u;
    return ErrorCode::NOT_FOUND;
  }

  resp[0] = to_u8(LibXR::USB::DapLinkV1Def::CommandId::DELAY);

  if (req_len < 3u)
  {
    resp[1] = to_u8(LibXR::USB::DapLinkV1Def::Status::ERROR);
    out_len = 2u;
    return ErrorCode::ARG_ERR;
  }

  uint16_t us = 0u;
  Memory::FastCopy(&us, &req[1], sizeof(us));

  LibXR::Timebase::DelayMicroseconds(us);

  resp[1] = to_u8(LibXR::USB::DapLinkV1Def::Status::OK);
  out_len = 2u;
  return ErrorCode::OK;
}

template <typename SwdPort>
ErrorCode DapLinkV1Class<SwdPort>::HandleResetTarget(bool in_isr, const uint8_t* /*req*/,
                                                     uint16_t /*req_len*/, uint8_t* resp,
                                                     uint16_t resp_cap, uint16_t& out_len)
{
  if (!resp || resp_cap < 3u)
  {
    out_len = 0u;
    return ErrorCode::NOT_FOUND;
  }

  resp[0] = to_u8(LibXR::USB::DapLinkV1Def::CommandId::RESET_TARGET);

  uint8_t execute = 0u;
  if (nreset_gpio_ != nullptr)
  {
    DriveReset(false);
    LibXR::Timebase::DelayMicroseconds(1000u);
    DriveReset(true);
    LibXR::Timebase::DelayMicroseconds(1000u);
    execute = 1u;
  }

  resp[1] = DAP_OK;
  resp[2] = execute;
  out_len = 3u;
  return ErrorCode::OK;
}

template <typename SwdPort>
ErrorCode DapLinkV1Class<SwdPort>::HandleSWJPins(bool /*in_isr*/, const uint8_t* req,
                                                 uint16_t req_len, uint8_t* resp,
                                                 uint16_t resp_cap, uint16_t& out_len)
{
  if (!resp || resp_cap < 2u)
  {
    out_len = 0u;
    return ErrorCode::NOT_FOUND;
  }

  resp[0] = to_u8(LibXR::USB::DapLinkV1Def::CommandId::SWJ_PINS);

  if (!req || req_len < 7u)
  {
    resp[1] = 0u;
    out_len = 2u;
    return ErrorCode::ARG_ERR;
  }

  const uint8_t PIN_OUT = req[1];
  const uint8_t PIN_SEL = req[2];

  uint32_t wait_us = 0u;
  Memory::FastCopy(&wait_us, &req[3], sizeof(wait_us));

  swj_shadow_ = static_cast<uint8_t>((swj_shadow_ & static_cast<uint8_t>(~PIN_SEL)) |
                                     (PIN_OUT & PIN_SEL));

  if ((PIN_SEL & LibXR::USB::DapLinkV1Def::DAP_SWJ_NRESET) != 0u)
  {
    const bool LEVEL_HIGH = ((PIN_OUT & LibXR::USB::DapLinkV1Def::DAP_SWJ_NRESET) != 0u);
    DriveReset(LEVEL_HIGH);
  }

  auto read_pins = [&]() -> uint8_t
  {
    uint8_t pin_in = swj_shadow_;

    if (nreset_gpio_ != nullptr)
    {
      if (nreset_gpio_->Read())
      {
        pin_in |= LibXR::USB::DapLinkV1Def::DAP_SWJ_NRESET;
      }
      else
      {
        pin_in = static_cast<uint8_t>(
            pin_in & static_cast<uint8_t>(~LibXR::USB::DapLinkV1Def::DAP_SWJ_NRESET));
      }
    }

    return pin_in;
  };

  uint8_t pin_in = 0u;
  if (wait_us == 0u || PIN_SEL == 0u)
  {
    pin_in = read_pins();
  }
  else
  {
    const uint64_t START = LibXR::Timebase::GetMicroseconds();
    const uint8_t EXPECT = static_cast<uint8_t>(PIN_OUT & PIN_SEL);

    do
    {
      pin_in = read_pins();
      if ((pin_in & PIN_SEL) == EXPECT)
      {
        break;
      }
    } while ((static_cast<uint64_t>(LibXR::Timebase::GetMicroseconds()) - START) <
             wait_us);
  }

  resp[1] = pin_in;
  out_len = 2u;
  return ErrorCode::OK;
}

template <typename SwdPort>
ErrorCode DapLinkV1Class<SwdPort>::HandleSWJClock(bool /*in_isr*/, const uint8_t* req,
                                                  uint16_t req_len, uint8_t* resp,
                                                  uint16_t resp_cap, uint16_t& out_len)
{
  if (resp_cap < 2u)
  {
    out_len = 0u;
    return ErrorCode::NOT_FOUND;
  }

  resp[0] = to_u8(LibXR::USB::DapLinkV1Def::CommandId::SWJ_CLOCK);

  if (req_len < 5u)
  {
    resp[1] = to_u8(LibXR::USB::DapLinkV1Def::Status::ERROR);
    out_len = 2u;
    return ErrorCode::ARG_ERR;
  }

  uint32_t hz = 0u;
  Memory::FastCopy(&hz, &req[1], sizeof(hz));

  swj_clock_hz_ = hz;
  (void)swd_.SetClockHz(hz);
  if constexpr (JTAG_ENABLED)
  {
    if (jtag_ != nullptr)
    {
      (void)jtag_->SetClockHz(hz);
    }
  }

  resp[1] = to_u8(LibXR::USB::DapLinkV1Def::Status::OK);
  out_len = 2u;
  return ErrorCode::OK;
}

template <typename SwdPort>
ErrorCode DapLinkV1Class<SwdPort>::HandleSWJSequence(bool /*in_isr*/, const uint8_t* req,
                                                     uint16_t req_len, uint8_t* resp,
                                                     uint16_t resp_cap, uint16_t& out_len)
{
  if (!resp || resp_cap < 2u)
  {
    out_len = 0u;
    return ErrorCode::NOT_FOUND;
  }

  resp[0] = to_u8(LibXR::USB::DapLinkV1Def::CommandId::SWJ_SEQUENCE);
  resp[1] = to_u8(LibXR::USB::DapLinkV1Def::Status::OK);
  out_len = 2u;

  if (!req || req_len < 2u)
  {
    resp[1] = to_u8(LibXR::USB::DapLinkV1Def::Status::ERROR);
    return ErrorCode::ARG_ERR;
  }

  uint32_t bit_count = req[1];
  if (bit_count == 0u)
  {
    bit_count = 256u;
  }

  const uint32_t BYTE_COUNT = (bit_count + 7u) / 8u;
  if (req_len < (2u + BYTE_COUNT))
  {
    resp[1] = to_u8(LibXR::USB::DapLinkV1Def::Status::ERROR);
    return ErrorCode::ARG_ERR;
  }

  const uint8_t* DATA = &req[2];
  ErrorCode ec = ErrorCode::OK;
  if constexpr (JTAG_ENABLED)
  {
    if (jtag_ != nullptr &&
        dap_state_.debug_port == LibXR::USB::DapLinkV1Def::DebugPort::JTAG)
    {
      uint32_t bit_idx = 0u;
      while (bit_idx < bit_count)
      {
        const bool TMS_BIT = (((DATA[bit_idx / 8u] >> (bit_idx & 7u)) & 0x01u) != 0u);
        uint32_t run = 1u;
        while ((bit_idx + run) < bit_count)
        {
          const uint32_t NEXT_I = bit_idx + run;
          const bool NEXT_BIT = (((DATA[NEXT_I / 8u] >> (NEXT_I & 7u)) & 0x01u) != 0u);
          if (NEXT_BIT != TMS_BIT)
          {
            break;
          }
          ++run;
        }
        ec = jtag_->Sequence(run, TMS_BIT, nullptr, nullptr);
        if (ec != ErrorCode::OK)
        {
          break;
        }
        bit_idx += run;
      }
    }
    else
    {
      ec = swd_.SeqWriteBits(bit_count, DATA);
    }
  }
  else
  {
    ec = swd_.SeqWriteBits(bit_count, DATA);
  }
  if (ec != ErrorCode::OK)
  {
    resp[1] = to_u8(LibXR::USB::DapLinkV1Def::Status::ERROR);
    return ErrorCode::OK;
  }

  swj_shadow_ = static_cast<uint8_t>(
      swj_shadow_ & static_cast<uint8_t>(~LibXR::USB::DapLinkV1Def::DAP_SWJ_SWCLK_TCK));

  bool last_swdio = false;
  if (bit_count != 0u)
  {
    const uint32_t LAST_I = bit_count - 1u;
    last_swdio = (((DATA[LAST_I / 8u] >> (LAST_I & 7u)) & 0x01u) != 0u);
  }

  if (last_swdio)
  {
    swj_shadow_ |= LibXR::USB::DapLinkV1Def::DAP_SWJ_SWDIO_TMS;
  }
  else
  {
    swj_shadow_ = static_cast<uint8_t>(
        swj_shadow_ & static_cast<uint8_t>(~LibXR::USB::DapLinkV1Def::DAP_SWJ_SWDIO_TMS));
  }

  return ErrorCode::OK;
}

template <typename SwdPort>
ErrorCode DapLinkV1Class<SwdPort>::HandleSWDConfigure(bool /*in_isr*/,
                                                      const uint8_t* /*req*/,
                                                      uint16_t /*req_len*/, uint8_t* resp,
                                                      uint16_t resp_cap,
                                                      uint16_t& out_len)
{
  if (resp_cap < 2u)
  {
    out_len = 0u;
    return ErrorCode::NOT_FOUND;
  }

  resp[0] = to_u8(LibXR::USB::DapLinkV1Def::CommandId::SWD_CONFIGURE);
  resp[1] = to_u8(LibXR::USB::DapLinkV1Def::Status::OK);
  out_len = 2u;
  return ErrorCode::OK;
}

template <typename SwdPort>
ErrorCode DapLinkV1Class<SwdPort>::HandleSWDSequence(bool /*in_isr*/, const uint8_t* req,
                                                     uint16_t req_len, uint8_t* resp,
                                                     uint16_t resp_cap, uint16_t& out_len)
{
  if (!req || !resp || resp_cap < 2u)
  {
    out_len = 0u;
    return ErrorCode::ARG_ERR;
  }

  resp[0] = to_u8(LibXR::USB::DapLinkV1Def::CommandId::SWD_SEQUENCE);
  resp[1] = DAP_OK;
  out_len = 2u;

  if (req_len < 2u)
  {
    resp[1] = DAP_ERROR;
    out_len = 2u;
    return ErrorCode::ARG_ERR;
  }

  const uint8_t SEQ_CNT = req[1];
  uint16_t req_off = 2u;
  uint16_t resp_off = 2u;

  for (uint32_t s = 0; s < SEQ_CNT; ++s)
  {
    if (req_off >= req_len)
    {
      resp[1] = DAP_ERROR;
      out_len = 2u;
      return ErrorCode::ARG_ERR;
    }

    const uint8_t INFO = req[req_off++];
    uint32_t cycles = static_cast<uint32_t>(INFO & 0x3Fu);
    if (cycles == 0u)
    {
      cycles = 64u;
    }

    const bool MODE_IN = ((INFO & 0x80u) != 0u);
    const uint16_t BYTES = static_cast<uint16_t>((cycles + 7u) / 8u);

    if (!MODE_IN)
    {
      if (req_off + BYTES > req_len)
      {
        resp[1] = DAP_ERROR;
        out_len = 2u;
        return ErrorCode::ARG_ERR;
      }

      const uint8_t* DATA = &req[req_off];
      req_off = static_cast<uint16_t>(req_off + BYTES);

      const ErrorCode EC = swd_.SeqWriteBits(cycles, DATA);
      if (EC != ErrorCode::OK)
      {
        resp[1] = DAP_ERROR;
        out_len = 2u;
        return ErrorCode::OK;
      }
    }
    else
    {
      if (resp_off + BYTES > resp_cap)
      {
        resp[1] = DAP_ERROR;
        out_len = 2u;
        return ErrorCode::NOT_FOUND;
      }

      Memory::FastSet(&resp[resp_off], 0, BYTES);
      const ErrorCode EC = swd_.SeqReadBits(cycles, &resp[resp_off]);
      if (EC != ErrorCode::OK)
      {
        resp[1] = DAP_ERROR;
        out_len = 2u;
        return ErrorCode::OK;
      }

      resp_off = static_cast<uint16_t>(resp_off + BYTES);
    }
  }

  out_len = resp_off;
  return ErrorCode::OK;
}

template <typename SwdPort>
ErrorCode DapLinkV1Class<SwdPort>::HandleJTAGSequence(bool /*in_isr*/, const uint8_t* req,
                                                      uint16_t req_len, uint8_t* resp,
                                                      uint16_t resp_cap,
                                                      uint16_t& out_len)
{
  if (!req || !resp || resp_cap < 2u)
  {
    out_len = 0u;
    return ErrorCode::ARG_ERR;
  }

  resp[0] = to_u8(LibXR::USB::DapLinkV1Def::CommandId::JTAG_SEQUENCE);
  resp[1] = DAP_OK;
  out_len = 2u;

  if (jtag_ == nullptr)
  {
    resp[1] = DAP_ERROR;
    return ErrorCode::OK;
  }

  if (req_len < 2u)
  {
    resp[1] = DAP_ERROR;
    return ErrorCode::ARG_ERR;
  }

  const uint8_t SEQ_CNT = req[1];
  uint16_t req_off = 2u;
  uint16_t resp_off = 2u;

  for (uint32_t s = 0; s < SEQ_CNT; ++s)
  {
    if (req_off >= req_len)
    {
      resp[1] = DAP_ERROR;
      out_len = 2u;
      return ErrorCode::ARG_ERR;
    }

    const uint8_t INFO = req[req_off++];
    uint32_t cycles = static_cast<uint32_t>(INFO & 0x3Fu);
    if (cycles == 0u)
    {
      cycles = 64u;
    }

    const bool TMS = ((INFO & LibXR::Debug::JtagProtocol::JTAG_SEQUENCE_TMS) != 0u);
    const bool CAPTURE = ((INFO & LibXR::Debug::JtagProtocol::JTAG_SEQUENCE_TDO) != 0u);
    const uint16_t BYTES = static_cast<uint16_t>((cycles + 7u) / 8u);

    if (req_off + BYTES > req_len)
    {
      resp[1] = DAP_ERROR;
      out_len = 2u;
      return ErrorCode::ARG_ERR;
    }

    if (CAPTURE && (resp_off + BYTES > resp_cap))
    {
      resp[1] = DAP_ERROR;
      out_len = 2u;
      return ErrorCode::NOT_FOUND;
    }

    const uint8_t* DATA = &req[req_off];
    uint8_t* OUT = CAPTURE ? &resp[resp_off] : nullptr;
    if (CAPTURE)
    {
      Memory::FastSet(OUT, 0, BYTES);
    }

    const ErrorCode EC = jtag_->Sequence(cycles, TMS, DATA, OUT);
    if (EC != ErrorCode::OK)
    {
      resp[1] = DAP_ERROR;
      out_len = 2u;
      return ErrorCode::OK;
    }

    req_off = static_cast<uint16_t>(req_off + BYTES);
    if (CAPTURE)
    {
      resp_off = static_cast<uint16_t>(resp_off + BYTES);
    }
  }

  out_len = resp_off;
  return ErrorCode::OK;
}

template <typename SwdPort>
ErrorCode DapLinkV1Class<SwdPort>::HandleJTAGConfigure(bool /*in_isr*/,
                                                       const uint8_t* req,
                                                       uint16_t req_len, uint8_t* resp,
                                                       uint16_t resp_cap,
                                                       uint16_t& out_len)
{
  if (!req || !resp || resp_cap < 2u)
  {
    out_len = 0u;
    return ErrorCode::ARG_ERR;
  }

  resp[0] = to_u8(LibXR::USB::DapLinkV1Def::CommandId::JTAG_CONFIGURE);
  resp[1] = DAP_OK;
  out_len = 2u;

  if (jtag_ == nullptr)
  {
    resp[1] = DAP_ERROR;
    return ErrorCode::OK;
  }

  if (req_len < 2u)
  {
    resp[1] = DAP_ERROR;
    return ErrorCode::ARG_ERR;
  }

  const uint8_t COUNT = req[1];
  if (COUNT > JTAG_MAX_DEVICES || req_len < static_cast<uint16_t>(2u + COUNT))
  {
    resp[1] = DAP_ERROR;
    return ErrorCode::ARG_ERR;
  }

  Memory::FastSet(jtag_ir_len_, 0, sizeof(jtag_ir_len_));

  uint32_t bits = 0u;
  for (uint32_t i = 0; i < COUNT; ++i)
  {
    const uint8_t LEN = req[static_cast<uint16_t>(2u + i)];
    jtag_ir_len_[i] = LEN;
    bits += LEN;
  }

  jtag_chain_.count = COUNT;
  if (jtag_chain_.index >= jtag_chain_.count)
  {
    jtag_chain_.index = 0u;
  }
  jtag_chain_.ir_before_bits_len = 0u;
  jtag_chain_.ir_after_bits_len = 0u;
  jtag_chain_.dr_before_bits_len = 0u;
  jtag_chain_.dr_after_bits_len = 0u;

  return ErrorCode::OK;
}

template <typename SwdPort>
ErrorCode DapLinkV1Class<SwdPort>::HandleJTAGIdCode(bool /*in_isr*/, const uint8_t* req,
                                                    uint16_t req_len, uint8_t* resp,
                                                    uint16_t resp_cap, uint16_t& out_len)
{
  if (!req || !resp || resp_cap < 2u)
  {
    out_len = 0u;
    return ErrorCode::ARG_ERR;
  }

  if (resp_cap < 6u)
  {
    out_len = 0u;
    return ErrorCode::NOT_FOUND;
  }

  resp[0] = to_u8(LibXR::USB::DapLinkV1Def::CommandId::JTAG_IDCODE);
  resp[1] = DAP_ERROR;
  out_len = 2u;

  if (jtag_ == nullptr ||
      dap_state_.debug_port != LibXR::USB::DapLinkV1Def::DebugPort::JTAG)
  {
    return ErrorCode::OK;
  }

  if (req_len < 2u)
  {
    return ErrorCode::ARG_ERR;
  }

  const uint8_t INDEX = req[1];
  if (INDEX >= jtag_chain_.count)
  {
    return ErrorCode::OK;
  }

  jtag_chain_.index = INDEX;
  LibXR::Debug::JtagProtocol::UpdateChainCache(jtag_chain_);

  LibXR::Debug::JtagDp dp(*jtag_, &jtag_chain_);
  uint32_t idcode = 0u;
  ErrorCode EC = dp.ReadIdCode(idcode);
  if (EC == ErrorCode::OK &&
      (idcode == 0u || idcode == 0xFFFF'FFFFu || ((idcode & 0x1u) == 0u)) &&
      jtag_ir_len_[INDEX] != 4u)
  {
    uint32_t fallback_idcode = 0u;
    EC = dp.ReadIdCodeWithIr(0x01u, fallback_idcode);
    if (EC != ErrorCode::OK)
    {
      return EC;
    }
    if (fallback_idcode != 0u && fallback_idcode != 0xFFFF'FFFFu &&
        ((fallback_idcode & 0x1u) != 0u))
    {
      idcode = fallback_idcode;
    }
  }
  if (EC != ErrorCode::OK)
  {
    return EC;
  }

  resp[1] = DAP_OK;
  resp[2] = static_cast<uint8_t>(idcode & 0xFFu);
  resp[3] = static_cast<uint8_t>((idcode >> 8) & 0xFFu);
  resp[4] = static_cast<uint8_t>((idcode >> 16) & 0xFFu);
  resp[5] = static_cast<uint8_t>((idcode >> 24) & 0xFFu);
  out_len = 6u;
  return ErrorCode::OK;
}

template <typename SwdPort>
ErrorCode DapLinkV1Class<SwdPort>::HandleQueueCommands(bool /*in_isr*/,
                                                       const uint8_t* /*req*/,
                                                       uint16_t /*req_len*/,
                                                       uint8_t* resp, uint16_t resp_cap,
                                                       uint16_t& out_len)
{
  return build_cmd_status_response(
      to_u8(LibXR::USB::DapLinkV1Def::CommandId::QUEUE_COMMANDS), DAP_ERROR, resp,
      resp_cap, out_len);
}

template <typename SwdPort>
ErrorCode DapLinkV1Class<SwdPort>::HandleExecuteCommands(bool /*in_isr*/,
                                                         const uint8_t* /*req*/,
                                                         uint16_t /*req_len*/,
                                                         uint8_t* resp, uint16_t resp_cap,
                                                         uint16_t& out_len)
{
  return build_cmd_status_response(
      to_u8(LibXR::USB::DapLinkV1Def::CommandId::EXECUTE_COMMANDS), DAP_ERROR, resp,
      resp_cap, out_len);
}

template <typename SwdPort>
uint8_t DapLinkV1Class<SwdPort>::MapAckToDapResp(LibXR::Debug::SwdProtocol::Ack ack) const
{
  switch (ack)
  {
    case LibXR::Debug::SwdProtocol::Ack::OK:
      return 1u;
    case LibXR::Debug::SwdProtocol::Ack::WAIT:
      return 2u;
    case LibXR::Debug::SwdProtocol::Ack::FAULT:
      return 4u;
    case LibXR::Debug::SwdProtocol::Ack::NO_ACK:
    case LibXR::Debug::SwdProtocol::Ack::PROTOCOL:
    default:
      return 7u;
  }
}

template <typename SwdPort>
uint8_t DapLinkV1Class<SwdPort>::MapAckToDapResp(
    LibXR::Debug::JtagProtocol::Ack ack) const
{
  switch (ack)
  {
    case LibXR::Debug::JtagProtocol::Ack::OK:
      return 1u;
    case LibXR::Debug::JtagProtocol::Ack::WAIT:
      return 2u;
    case LibXR::Debug::JtagProtocol::Ack::FAULT:
      return 4u;
    case LibXR::Debug::JtagProtocol::Ack::NO_ACK:
    case LibXR::Debug::JtagProtocol::Ack::PROTOCOL:
    default:
      return 7u;
  }
}

template <typename SwdPort>
void DapLinkV1Class<SwdPort>::SetTransferAbortFlag(bool on)
{
  dap_state_.transfer_abort = on;
}

template <typename SwdPort>
ErrorCode DapLinkV1Class<SwdPort>::HandleTransfer(bool /*in_isr*/, const uint8_t* req,
                                                  uint16_t req_len, uint8_t* resp,
                                                  uint16_t resp_cap, uint16_t& out_len)
{
  out_len = 0u;
  if (!req || !resp || resp_cap < 3u)
  {
    return ErrorCode::ARG_ERR;
  }

  if constexpr (JTAG_ENABLED)
  {
    if (dap_state_.debug_port == LibXR::USB::DapLinkV1Def::DebugPort::JTAG)
    {
      return HandleJtagTransfer(false, req, req_len, resp, resp_cap, out_len);
    }
  }

  resp[0] = to_u8(LibXR::USB::DapLinkV1Def::CommandId::TRANSFER);
  resp[1] = 0u;
  resp[2] = 0u;
  uint16_t resp_off = 3u;

  if (req_len < 3u)
  {
    resp[2] = LibXR::USB::DapLinkV1Def::DAP_TRANSFER_ERROR;
    out_len = 3u;
    return ErrorCode::ARG_ERR;
  }

  if (dap_state_.transfer_abort)
  {
    dap_state_.transfer_abort = false;
    resp[2] = LibXR::USB::DapLinkV1Def::DAP_TRANSFER_ERROR;
    out_len = 3u;
    return ErrorCode::OK;
  }

  const uint8_t COUNT = req[2];
  uint16_t req_off = 3u;

  auto ack_to_dap = [&](LibXR::Debug::SwdProtocol::Ack ack) -> uint8_t
  {
    switch (ack)
    {
      case LibXR::Debug::SwdProtocol::Ack::OK:
        return LibXR::USB::DapLinkV1Def::DAP_TRANSFER_OK;
      case LibXR::Debug::SwdProtocol::Ack::WAIT:
        return LibXR::USB::DapLinkV1Def::DAP_TRANSFER_WAIT;
      case LibXR::Debug::SwdProtocol::Ack::FAULT:
        return LibXR::USB::DapLinkV1Def::DAP_TRANSFER_FAULT;
      default:
        return 0x07u;
    }
  };

  auto push_u32 = [&](uint32_t VALUE) -> bool
  {
    if (resp_off + 4u > resp_cap)
    {
      return false;
    }
    Memory::FastCopy(&resp[resp_off], &VALUE, sizeof(VALUE));
    resp_off = static_cast<uint16_t>(resp_off + 4u);
    return true;
  };

  auto push_timestamp = [&]() -> bool
  {
    const uint32_t T = static_cast<uint32_t>(LibXR::Timebase::GetMicroseconds());
    return push_u32(T);
  };

  auto ensure_space = [&](uint16_t bytes) -> bool
  { return (resp_off + bytes) <= resp_cap; };

  auto bytes_for_read = [&](bool need_ts) -> uint16_t
  { return static_cast<uint16_t>(need_ts ? 8u : 4u); };

  uint8_t response_count = 0u;
  uint8_t response_value = 0u;
  bool check_write = false;

  struct PendingApRead
  {
    bool valid = false;
    bool need_ts = false;
  } pending;

  auto emit_read_with_ts = [&](bool need_ts, uint32_t data) -> bool
  {
    if (need_ts)
    {
      if (!push_timestamp())
      {
        return false;
      }
    }
    if (!push_u32(data))
    {
      return false;
    }
    response_count++;
    return true;
  };

  auto complete_pending_by_rdbuff = [&]() -> bool
  {
    if (!pending.valid)
    {
      return true;
    }

    if (!ensure_space(bytes_for_read(pending.need_ts)))
    {
      response_value = LibXR::USB::DapLinkV1Def::DAP_TRANSFER_ERROR;
      return false;
    }

    uint32_t rdata = 0u;
    LibXR::Debug::SwdProtocol::Ack ack = LibXR::Debug::SwdProtocol::Ack::PROTOCOL;
    const ErrorCode EC = swd_.DpReadRdbuffTxn(rdata, ack);

    const uint8_t V = ack_to_dap(ack);
    if (V != LibXR::USB::DapLinkV1Def::DAP_TRANSFER_OK)
    {
      response_value = V;
      return false;
    }
    if (EC != ErrorCode::OK)
    {
      response_value = LibXR::USB::DapLinkV1Def::DAP_TRANSFER_ERROR;
      return false;
    }

    if (!emit_read_with_ts(pending.need_ts, rdata))
    {
      response_value = LibXR::USB::DapLinkV1Def::DAP_TRANSFER_ERROR;
      return false;
    }

    pending.valid = false;
    pending.need_ts = false;
    check_write = false;
    response_value = LibXR::USB::DapLinkV1Def::DAP_TRANSFER_OK;
    return true;
  };

  auto flush_pending_if_any = [&]() -> bool
  { return pending.valid ? complete_pending_by_rdbuff() : true; };

  for (uint32_t i = 0; i < COUNT; ++i)
  {
    if (req_off >= req_len)
    {
      response_value = LibXR::USB::DapLinkV1Def::DAP_TRANSFER_ERROR;
      break;
    }

    const uint8_t RQ = req[req_off++];

    const bool AP = LibXR::USB::DapLinkV1Def::req_is_ap(RQ);
    const bool RNW = LibXR::USB::DapLinkV1Def::req_is_read(RQ);
    const uint8_t ADDR2B = LibXR::USB::DapLinkV1Def::req_addr2b(RQ);

    const bool TS = LibXR::USB::DapLinkV1Def::req_need_timestamp(RQ);
    const bool MATCH_VALUE =
        ((RQ & LibXR::USB::DapLinkV1Def::DAP_TRANSFER_MATCH_VALUE) != 0u);
    const bool MATCH_MASK =
        ((RQ & LibXR::USB::DapLinkV1Def::DAP_TRANSFER_MATCH_MASK) != 0u);

    if (TS && (MATCH_VALUE || MATCH_MASK))
    {
      response_value = LibXR::USB::DapLinkV1Def::DAP_TRANSFER_ERROR;
      break;
    }

    LibXR::Debug::SwdProtocol::Ack ack = LibXR::Debug::SwdProtocol::Ack::PROTOCOL;
    ErrorCode EC = ErrorCode::OK;

    if (!RNW)
    {
      if (!flush_pending_if_any())
      {
        break;
      }

      if (req_off + 4u > req_len)
      {
        response_value = LibXR::USB::DapLinkV1Def::DAP_TRANSFER_ERROR;
        break;
      }

      uint32_t wdata = 0u;
      Memory::FastCopy(&wdata, &req[req_off], sizeof(wdata));
      req_off = static_cast<uint16_t>(req_off + 4u);

      if (MATCH_MASK)
      {
        match_mask_ = wdata;
        response_value = LibXR::USB::DapLinkV1Def::DAP_TRANSFER_OK;
        response_count++;
        continue;
      }

      if (AP)
      {
        EC = swd_.ApWriteTxn(ADDR2B, wdata, ack);
      }
      else
      {
        EC = swd_.DpWriteTxn(static_cast<LibXR::Debug::SwdProtocol::DpWriteReg>(ADDR2B),
                             wdata, ack);
      }

      response_value = ack_to_dap(ack);
      if (response_value != LibXR::USB::DapLinkV1Def::DAP_TRANSFER_OK)
      {
        break;
      }
      if (EC != ErrorCode::OK)
      {
        response_value = LibXR::USB::DapLinkV1Def::DAP_TRANSFER_ERROR;
        break;
      }

      if (TS)
      {
        if (!push_timestamp())
        {
          response_value = LibXR::USB::DapLinkV1Def::DAP_TRANSFER_ERROR;
          break;
        }
      }

      response_count++;
      check_write = true;
    }
    else
    {
      if (MATCH_VALUE)
      {
        if (!flush_pending_if_any())
        {
          break;
        }

        if (req_off + 4u > req_len)
        {
          response_value = LibXR::USB::DapLinkV1Def::DAP_TRANSFER_ERROR;
          break;
        }

        uint32_t match_val = 0u;
        Memory::FastCopy(&match_val, &req[req_off], sizeof(match_val));
        req_off = static_cast<uint16_t>(req_off + 4u);

        uint32_t rdata = 0u;
        uint32_t retry = dap_state_.transfer_cfg.match_retry;
        bool matched = false;

        while (true)
        {
          if (AP)
          {
            EC = swd_.ApReadTxn(ADDR2B, rdata, ack);
            if (EC == ErrorCode::OK && ack == LibXR::Debug::SwdProtocol::Ack::OK)
            {
              check_write = false;
            }
          }
          else
          {
            EC = swd_.DpReadTxn(static_cast<LibXR::Debug::SwdProtocol::DpReadReg>(ADDR2B),
                                rdata, ack);
          }

          response_value = ack_to_dap(ack);
          if (response_value != LibXR::USB::DapLinkV1Def::DAP_TRANSFER_OK)
          {
            break;
          }
          if (EC != ErrorCode::OK)
          {
            response_value = LibXR::USB::DapLinkV1Def::DAP_TRANSFER_ERROR;
            break;
          }

          if ((rdata & match_mask_) == (match_val & match_mask_))
          {
            matched = true;
            break;
          }

          if (retry == 0u)
          {
            break;
          }
          --retry;
        }

        if (response_value != LibXR::USB::DapLinkV1Def::DAP_TRANSFER_OK)
        {
          break;
        }

        if (!matched)
        {
          response_value =
              static_cast<uint8_t>(LibXR::USB::DapLinkV1Def::DAP_TRANSFER_OK |
                                   LibXR::USB::DapLinkV1Def::DAP_TRANSFER_MISMATCH);
          break;
        }

        response_value = LibXR::USB::DapLinkV1Def::DAP_TRANSFER_OK;
        response_count++;
        continue;
      }

      if (!AP)
      {
        if (!flush_pending_if_any())
        {
          break;
        }

        uint32_t rdata = 0u;
        EC = swd_.DpReadTxn(static_cast<LibXR::Debug::SwdProtocol::DpReadReg>(ADDR2B),
                            rdata, ack);

        response_value = ack_to_dap(ack);
        if (response_value != LibXR::USB::DapLinkV1Def::DAP_TRANSFER_OK)
        {
          break;
        }
        if (EC != ErrorCode::OK)
        {
          response_value = LibXR::USB::DapLinkV1Def::DAP_TRANSFER_ERROR;
          break;
        }

        if (!ensure_space(bytes_for_read(TS)))
        {
          response_value = LibXR::USB::DapLinkV1Def::DAP_TRANSFER_ERROR;
          break;
        }

        if (!emit_read_with_ts(TS, rdata))
        {
          response_value = LibXR::USB::DapLinkV1Def::DAP_TRANSFER_ERROR;
          break;
        }

        response_value = LibXR::USB::DapLinkV1Def::DAP_TRANSFER_OK;
        continue;
      }

      if (!pending.valid)
      {
        uint32_t dummy_posted = 0u;
        EC = swd_.ApReadPostedTxn(ADDR2B, dummy_posted, ack);

        response_value = ack_to_dap(ack);
        if (response_value != LibXR::USB::DapLinkV1Def::DAP_TRANSFER_OK)
        {
          break;
        }
        if (EC != ErrorCode::OK)
        {
          response_value = LibXR::USB::DapLinkV1Def::DAP_TRANSFER_ERROR;
          break;
        }

        pending.valid = true;
        pending.need_ts = TS;
        response_value = LibXR::USB::DapLinkV1Def::DAP_TRANSFER_OK;
      }
      else
      {
        if (!ensure_space(bytes_for_read(pending.need_ts)))
        {
          response_value = LibXR::USB::DapLinkV1Def::DAP_TRANSFER_ERROR;
          break;
        }

        uint32_t posted_prev = 0u;
        EC = swd_.ApReadPostedTxn(ADDR2B, posted_prev, ack);

        const uint8_t CUR_V = ack_to_dap(ack);
        if (CUR_V != LibXR::USB::DapLinkV1Def::DAP_TRANSFER_OK || EC != ErrorCode::OK)
        {
          const uint8_t PRIOR_FAIL = (CUR_V != LibXR::USB::DapLinkV1Def::DAP_TRANSFER_OK)
                                         ? CUR_V
                                         : LibXR::USB::DapLinkV1Def::DAP_TRANSFER_ERROR;

          if (!complete_pending_by_rdbuff())
          {
            break;
          }

          response_value = PRIOR_FAIL;
          break;
        }

        if (!emit_read_with_ts(pending.need_ts, posted_prev))
        {
          response_value = LibXR::USB::DapLinkV1Def::DAP_TRANSFER_ERROR;
          break;
        }

        pending.valid = true;
        pending.need_ts = TS;
        response_value = LibXR::USB::DapLinkV1Def::DAP_TRANSFER_OK;
      }
    }

    if (dap_state_.transfer_abort)
    {
      dap_state_.transfer_abort = false;
      break;
    }
  }

  if (pending.valid)
  {
    const uint8_t PRIOR_FAIL = response_value;

    if (!complete_pending_by_rdbuff())
    {
    }
    else
    {
      if (PRIOR_FAIL != 0u && PRIOR_FAIL != LibXR::USB::DapLinkV1Def::DAP_TRANSFER_OK)
      {
        response_value = PRIOR_FAIL;
      }
    }
  }

  if (response_value == LibXR::USB::DapLinkV1Def::DAP_TRANSFER_OK && check_write)
  {
    uint32_t dummy = 0u;
    LibXR::Debug::SwdProtocol::Ack ack = LibXR::Debug::SwdProtocol::Ack::PROTOCOL;
    const ErrorCode EC = swd_.DpReadRdbuffTxn(dummy, ack);
    const uint8_t V = ack_to_dap(ack);

    if (V != LibXR::USB::DapLinkV1Def::DAP_TRANSFER_OK)
    {
      response_value = V;
    }
    else if (EC != ErrorCode::OK)
    {
      response_value = LibXR::USB::DapLinkV1Def::DAP_TRANSFER_ERROR;
    }
  }

  resp[1] = response_count;
  resp[2] = response_value;
  out_len = resp_off;
  return ErrorCode::OK;
}

template <typename SwdPort>
ErrorCode DapLinkV1Class<SwdPort>::HandleTransferBlock(bool /*in_isr*/,
                                                       const uint8_t* req,
                                                       uint16_t req_len, uint8_t* resp,
                                                       uint16_t resp_cap,
                                                       uint16_t& out_len)
{
  if constexpr (JTAG_ENABLED)
  {
    if (dap_state_.debug_port == LibXR::USB::DapLinkV1Def::DebugPort::JTAG)
    {
      return HandleJtagTransferBlock(false, req, req_len, resp, resp_cap, out_len);
    }
  }

  if (!resp || resp_cap < 4u)
  {
    out_len = 0u;
    return ErrorCode::NOT_FOUND;
  }

  resp[0] = to_u8(LibXR::USB::DapLinkV1Def::CommandId::TRANSFER_BLOCK);
  resp[1] = 0u;
  resp[2] = 0u;
  resp[3] = 0u;
  out_len = 4u;

  if (!req || req_len < 5u)
  {
    resp[3] = LibXR::USB::DapLinkV1Def::DAP_TRANSFER_ERROR;
    return ErrorCode::ARG_ERR;
  }

  if (dap_state_.transfer_abort)
  {
    dap_state_.transfer_abort = false;
    resp[3] = LibXR::USB::DapLinkV1Def::DAP_TRANSFER_ERROR;
    return ErrorCode::OK;
  }

  uint16_t count = 0u;
  Memory::FastCopy(&count, &req[2], sizeof(count));

  const uint8_t DAP_RQ = req[4];

  if ((DAP_RQ & (LibXR::USB::DapLinkV1Def::DAP_TRANSFER_MATCH_VALUE |
                 LibXR::USB::DapLinkV1Def::DAP_TRANSFER_MATCH_MASK)) != 0u)
  {
    resp[3] = LibXR::USB::DapLinkV1Def::DAP_TRANSFER_ERROR;
    return ErrorCode::NOT_SUPPORT;
  }
  if (LibXR::USB::DapLinkV1Def::req_need_timestamp(DAP_RQ))
  {
    resp[3] = LibXR::USB::DapLinkV1Def::DAP_TRANSFER_ERROR;
    return ErrorCode::NOT_SUPPORT;
  }

  if (count == 0u)
  {
    const uint16_t DONE0 = 0u;
    Memory::FastCopy(&resp[1], &DONE0, sizeof(DONE0));
    resp[3] = LibXR::USB::DapLinkV1Def::DAP_TRANSFER_OK;
    out_len = 4u;
    return ErrorCode::OK;
  }

  const bool AP = LibXR::USB::DapLinkV1Def::req_is_ap(DAP_RQ);
  const bool RNW = LibXR::USB::DapLinkV1Def::req_is_read(DAP_RQ);
  const uint8_t ADDR2B = LibXR::USB::DapLinkV1Def::req_addr2b(DAP_RQ);

  uint16_t done = 0u;
  uint8_t xresp = 0u;

  uint16_t req_off = 5u;
  uint16_t resp_off = 4u;

  if (!RNW)
  {
    for (uint32_t i = 0; i < count; ++i)
    {
      LibXR::Debug::SwdProtocol::Ack ack = LibXR::Debug::SwdProtocol::Ack::PROTOCOL;
      ErrorCode EC = ErrorCode::OK;

      if (req_off + 4u > req_len)
      {
        xresp |= LibXR::USB::DapLinkV1Def::DAP_TRANSFER_ERROR;
        break;
      }

      uint32_t wdata = 0u;
      Memory::FastCopy(&wdata, &req[req_off], sizeof(wdata));
      req_off = static_cast<uint16_t>(req_off + 4u);

      if (AP)
      {
        EC = swd_.ApWriteTxn(ADDR2B, wdata, ack);
      }
      else
      {
        EC = swd_.DpWriteTxn(static_cast<LibXR::Debug::SwdProtocol::DpWriteReg>(ADDR2B),
                             wdata, ack);
      }

      xresp = MapAckToDapResp(ack);
      if (xresp == 0u)
      {
        xresp |= LibXR::USB::DapLinkV1Def::DAP_TRANSFER_ERROR;
        break;
      }

      if (ack != LibXR::Debug::SwdProtocol::Ack::OK)
      {
        break;
      }

      if (EC != ErrorCode::OK)
      {
        xresp |= LibXR::USB::DapLinkV1Def::DAP_TRANSFER_ERROR;
        break;
      }

      done = static_cast<uint16_t>(i + 1u);
    }

    Memory::FastCopy(&resp[1], &done, sizeof(done));
    resp[3] = xresp;
    out_len = resp_off;
    return ErrorCode::OK;
  }

  if (!AP)
  {
    for (uint32_t i = 0; i < count; ++i)
    {
      LibXR::Debug::SwdProtocol::Ack ack = LibXR::Debug::SwdProtocol::Ack::PROTOCOL;
      ErrorCode EC = ErrorCode::OK;
      uint32_t rdata = 0u;

      if (resp_off + 4u > resp_cap)
      {
        xresp |= LibXR::USB::DapLinkV1Def::DAP_TRANSFER_ERROR;
        break;
      }

      EC = swd_.DpReadTxn(static_cast<LibXR::Debug::SwdProtocol::DpReadReg>(ADDR2B),
                          rdata, ack);

      xresp = MapAckToDapResp(ack);
      if (xresp == 0u)
      {
        xresp |= LibXR::USB::DapLinkV1Def::DAP_TRANSFER_ERROR;
        break;
      }

      if (ack != LibXR::Debug::SwdProtocol::Ack::OK)
      {
        break;
      }

      if (EC != ErrorCode::OK)
      {
        xresp |= LibXR::USB::DapLinkV1Def::DAP_TRANSFER_ERROR;
        break;
      }

      Memory::FastCopy(&resp[resp_off], &rdata, sizeof(rdata));
      resp_off = static_cast<uint16_t>(resp_off + 4u);
      done = static_cast<uint16_t>(i + 1u);
    }

    Memory::FastCopy(&resp[1], &done, sizeof(done));
    resp[3] = xresp;
    out_len = resp_off;
    return ErrorCode::OK;
  }

  {
    LibXR::Debug::SwdProtocol::Ack ack = LibXR::Debug::SwdProtocol::Ack::PROTOCOL;
    ErrorCode EC = ErrorCode::OK;

    uint32_t dummy_posted = 0u;
    EC = swd_.ApReadPostedTxn(ADDR2B, dummy_posted, ack);
    xresp = MapAckToDapResp(ack);

    if (xresp == 0u)
    {
      xresp |= LibXR::USB::DapLinkV1Def::DAP_TRANSFER_ERROR;
      goto out_ap_read;  // NOLINT
    }
    if (ack != LibXR::Debug::SwdProtocol::Ack::OK)
    {
      goto out_ap_read;  // NOLINT
    }
    if (EC != ErrorCode::OK)
    {
      xresp |= LibXR::USB::DapLinkV1Def::DAP_TRANSFER_ERROR;
      goto out_ap_read;  // NOLINT
    }

    for (uint32_t i = 1; i < count; ++i)
    {
      if (resp_off + 4u > resp_cap)
      {
        xresp |= LibXR::USB::DapLinkV1Def::DAP_TRANSFER_ERROR;
        goto out_ap_read;  // NOLINT
      }

      uint32_t posted_prev = 0u;
      EC = swd_.ApReadPostedTxn(ADDR2B, posted_prev, ack);
      const uint8_t CUR = MapAckToDapResp(ack);

      if (CUR == 0u)
      {
        xresp = CUR | LibXR::USB::DapLinkV1Def::DAP_TRANSFER_ERROR;
        goto out_ap_read;  // NOLINT
      }

      if (ack != LibXR::Debug::SwdProtocol::Ack::OK || EC != ErrorCode::OK)
      {
        if (resp_off + 4u <= resp_cap)
        {
          uint32_t last = 0u;
          LibXR::Debug::SwdProtocol::Ack ack2 = LibXR::Debug::SwdProtocol::Ack::PROTOCOL;
          const ErrorCode EC2 = swd_.DpReadRdbuffTxn(last, ack2);
          const uint8_t V2 = MapAckToDapResp(ack2);

          if (V2 == 1u && EC2 == ErrorCode::OK)
          {
            Memory::FastCopy(&resp[resp_off], &last, sizeof(last));
            resp_off = static_cast<uint16_t>(resp_off + 4u);
            done = static_cast<uint16_t>(i);
          }
          else
          {
            xresp = V2;
            if (EC2 != ErrorCode::OK)
            {
              xresp |= LibXR::USB::DapLinkV1Def::DAP_TRANSFER_ERROR;
            }
            goto out_ap_read;  // NOLINT
          }
        }

        xresp = CUR;
        if (EC != ErrorCode::OK)
        {
          xresp |= LibXR::USB::DapLinkV1Def::DAP_TRANSFER_ERROR;
        }
        goto out_ap_read;  // NOLINT
      }

      Memory::FastCopy(&resp[resp_off], &posted_prev, sizeof(posted_prev));
      resp_off = static_cast<uint16_t>(resp_off + 4u);
      done = static_cast<uint16_t>(i);
      xresp = CUR;
    }

    if (resp_off + 4u > resp_cap)
    {
      xresp |= LibXR::USB::DapLinkV1Def::DAP_TRANSFER_ERROR;
      goto out_ap_read;  // NOLINT
    }

    {
      uint32_t last = 0u;
      LibXR::Debug::SwdProtocol::Ack ack2 = LibXR::Debug::SwdProtocol::Ack::PROTOCOL;
      const ErrorCode EC2 = swd_.DpReadRdbuffTxn(last, ack2);
      const uint8_t V2 = MapAckToDapResp(ack2);

      xresp = V2;
      if (V2 != 1u)
      {
        goto out_ap_read;  // NOLINT
      }
      if (EC2 != ErrorCode::OK)
      {
        xresp |= LibXR::USB::DapLinkV1Def::DAP_TRANSFER_ERROR;
        goto out_ap_read;  // NOLINT
      }

      Memory::FastCopy(&resp[resp_off], &last, sizeof(last));
      resp_off = static_cast<uint16_t>(resp_off + 4u);
      done = count;
    }

  out_ap_read:
    Memory::FastCopy(&resp[1], &done, sizeof(done));
    resp[3] = xresp;
    out_len = resp_off;
    return ErrorCode::OK;
  }
}

template <typename SwdPort>
ErrorCode DapLinkV1Class<SwdPort>::HandleJtagTransfer(bool /*in_isr*/, const uint8_t* req,
                                                      uint16_t req_len, uint8_t* resp,
                                                      uint16_t resp_cap,
                                                      uint16_t& out_len)
{
  out_len = 0u;
  if (!req || !resp || resp_cap < 3u)
  {
    return ErrorCode::ARG_ERR;
  }

  resp[0] = to_u8(LibXR::USB::DapLinkV1Def::CommandId::TRANSFER);
  resp[1] = 0u;
  resp[2] = 0u;
  uint16_t resp_off = 3u;

  if (req_len < 3u || jtag_ == nullptr || jtag_chain_.count == 0u)
  {
    resp[2] = LibXR::USB::DapLinkV1Def::DAP_TRANSFER_ERROR;
    out_len = 3u;
    return ErrorCode::OK;
  }

  if (dap_state_.transfer_abort)
  {
    dap_state_.transfer_abort = false;
    resp[2] = LibXR::USB::DapLinkV1Def::DAP_TRANSFER_ERROR;
    out_len = 3u;
    return ErrorCode::OK;
  }

  const uint8_t TAP_INDEX = req[1];
  if (TAP_INDEX >= jtag_chain_.count)
  {
    resp[2] = LibXR::USB::DapLinkV1Def::DAP_TRANSFER_ERROR;
    out_len = 3u;
    return ErrorCode::OK;
  }

  jtag_chain_.index = TAP_INDEX;
  LibXR::Debug::JtagProtocol::UpdateChainCache(jtag_chain_);

  LibXR::Debug::JtagDp dp(*jtag_, &jtag_chain_);

  const uint8_t COUNT = req[2];
  uint16_t req_off = 3u;

  auto push_u32 = [&](uint32_t VALUE) -> bool
  {
    if (resp_off + 4u > resp_cap)
    {
      return false;
    }
    Memory::FastCopy(&resp[resp_off], &VALUE, sizeof(VALUE));
    resp_off = static_cast<uint16_t>(resp_off + 4u);
    return true;
  };

  auto push_timestamp = [&]() -> bool
  {
    const uint32_t T = static_cast<uint32_t>(LibXR::Timebase::GetMicroseconds());
    return push_u32(T);
  };

  auto ensure_space = [&](uint16_t bytes) -> bool
  { return (resp_off + bytes) <= resp_cap; };

  auto bytes_for_read = [&](bool need_ts) -> uint16_t
  { return static_cast<uint16_t>(need_ts ? 8u : 4u); };

  auto idle_after_attempt = [&]()
  {
    if (dap_state_.transfer_cfg.idle_cycles != 0u)
    {
      jtag_->IdleClocks(dap_state_.transfer_cfg.idle_cycles);
    }
  };

  auto dp_read_retry = [&](uint8_t addr2b, uint32_t& val,
                           LibXR::Debug::JtagProtocol::Ack& ack) -> ErrorCode
  {
    uint32_t retry = dap_state_.transfer_cfg.retry_count;
    while (true)
    {
      const ErrorCode EC = dp.DpReadTxn(addr2b, val, ack);
      idle_after_attempt();
      if (ack != LibXR::Debug::JtagProtocol::Ack::WAIT || retry == 0u ||
          dap_state_.transfer_abort)
      {
        return EC;
      }
      --retry;
    }
  };

  auto dp_write_retry = [&](uint8_t addr2b, uint32_t val,
                            LibXR::Debug::JtagProtocol::Ack& ack) -> ErrorCode
  {
    uint32_t retry = dap_state_.transfer_cfg.retry_count;
    while (true)
    {
      const ErrorCode EC = dp.DpWriteTxn(addr2b, val, ack);
      idle_after_attempt();
      if (ack != LibXR::Debug::JtagProtocol::Ack::WAIT || retry == 0u ||
          dap_state_.transfer_abort)
      {
        return EC;
      }
      --retry;
    }
  };

  auto ap_read_retry = [&](uint8_t addr2b, uint32_t& val,
                           LibXR::Debug::JtagProtocol::Ack& ack) -> ErrorCode
  {
    uint32_t retry = dap_state_.transfer_cfg.retry_count;
    while (true)
    {
      const ErrorCode EC = dp.ApReadTxn(addr2b, val, ack);
      idle_after_attempt();
      if (ack != LibXR::Debug::JtagProtocol::Ack::WAIT || retry == 0u ||
          dap_state_.transfer_abort)
      {
        return EC;
      }
      --retry;
    }
  };

  auto ap_write_retry = [&](uint8_t addr2b, uint32_t val,
                            LibXR::Debug::JtagProtocol::Ack& ack) -> ErrorCode
  {
    uint32_t retry = dap_state_.transfer_cfg.retry_count;
    while (true)
    {
      const ErrorCode EC = dp.ApWriteTxn(addr2b, val, ack);
      idle_after_attempt();
      if (ack != LibXR::Debug::JtagProtocol::Ack::WAIT || retry == 0u ||
          dap_state_.transfer_abort)
      {
        return EC;
      }
      --retry;
    }
  };

  auto check_posted_ap_write_jtag = [&](LibXR::Debug::JtagProtocol::Ack& ack) -> ErrorCode
  {
    uint32_t ctrl_stat = 0u;
    return dp_read_retry(
        static_cast<uint8_t>(LibXR::Debug::SwdProtocol::DpReadReg::CTRL_STAT), ctrl_stat,
        ack);
  };

  uint8_t response_count = 0u;
  uint8_t response_value = LibXR::USB::DapLinkV1Def::DAP_TRANSFER_OK;
  bool pending_ap_write = false;

  auto check_posted_ap_write_if_pending = [&]() -> bool
  {
    if (!pending_ap_write)
    {
      return true;
    }

    LibXR::Debug::JtagProtocol::Ack ack = LibXR::Debug::JtagProtocol::Ack::PROTOCOL;
    const ErrorCode EC = check_posted_ap_write_jtag(ack);
    const uint8_t V = MapAckToDapResp(ack);

    if (V != LibXR::USB::DapLinkV1Def::DAP_TRANSFER_OK)
    {
      response_value = V;
      return false;
    }
    if (EC != ErrorCode::OK)
    {
      response_value = LibXR::USB::DapLinkV1Def::DAP_TRANSFER_ERROR;
      return false;
    }

    pending_ap_write = false;
    return true;
  };

  auto should_check_pending_ap_write_at_tail = [&]() -> bool
  {
    return response_value == LibXR::USB::DapLinkV1Def::DAP_TRANSFER_OK ||
           response_value ==
               static_cast<uint8_t>(LibXR::USB::DapLinkV1Def::DAP_TRANSFER_OK |
                                    LibXR::USB::DapLinkV1Def::DAP_TRANSFER_MISMATCH);
  };

  auto emit_read_with_ts = [&](bool need_ts, uint32_t data) -> bool
  {
    if (need_ts)
    {
      if (!push_timestamp())
      {
        return false;
      }
    }
    if (!push_u32(data))
    {
      return false;
    }
    response_count++;
    return true;
  };

  for (uint32_t i = 0; i < COUNT; ++i)
  {
    if (req_off >= req_len)
    {
      response_value = LibXR::USB::DapLinkV1Def::DAP_TRANSFER_ERROR;
      break;
    }

    const uint8_t RQ = req[req_off++];
    const bool AP = LibXR::USB::DapLinkV1Def::req_is_ap(RQ);
    const bool RNW = LibXR::USB::DapLinkV1Def::req_is_read(RQ);
    const uint8_t ADDR2B = LibXR::USB::DapLinkV1Def::req_addr2b(RQ);

    const bool TS = LibXR::USB::DapLinkV1Def::req_need_timestamp(RQ);
    const bool MATCH_VALUE =
        ((RQ & LibXR::USB::DapLinkV1Def::DAP_TRANSFER_MATCH_VALUE) != 0u);
    const bool MATCH_MASK =
        ((RQ & LibXR::USB::DapLinkV1Def::DAP_TRANSFER_MATCH_MASK) != 0u);

    if (TS && (MATCH_VALUE || MATCH_MASK))
    {
      response_value = LibXR::USB::DapLinkV1Def::DAP_TRANSFER_ERROR;
      break;
    }

    LibXR::Debug::JtagProtocol::Ack ack = LibXR::Debug::JtagProtocol::Ack::PROTOCOL;
    ErrorCode EC = ErrorCode::OK;

    if (!RNW)
    {
      if (req_off + 4u > req_len)
      {
        response_value = LibXR::USB::DapLinkV1Def::DAP_TRANSFER_ERROR;
        break;
      }

      uint32_t wdata = 0u;
      Memory::FastCopy(&wdata, &req[req_off], sizeof(wdata));
      req_off = static_cast<uint16_t>(req_off + 4u);

      if (MATCH_MASK)
      {
        match_mask_ = wdata;
        response_value = LibXR::USB::DapLinkV1Def::DAP_TRANSFER_OK;
        response_count++;
        continue;
      }

      if (AP)
      {
        EC = ap_write_retry(ADDR2B, wdata, ack);
      }
      else
      {
        EC = dp_write_retry(ADDR2B, wdata, ack);
      }

      response_value = MapAckToDapResp(ack);
      if (response_value != LibXR::USB::DapLinkV1Def::DAP_TRANSFER_OK)
      {
        break;
      }
      if (EC != ErrorCode::OK)
      {
        response_value = LibXR::USB::DapLinkV1Def::DAP_TRANSFER_ERROR;
        break;
      }

      if (TS)
      {
        if (!push_timestamp())
        {
          response_value = LibXR::USB::DapLinkV1Def::DAP_TRANSFER_ERROR;
          break;
        }
      }

      response_count++;
      if (AP)
      {
        pending_ap_write = true;
      }
    }
    else
    {
      if (MATCH_VALUE)
      {
        if (!check_posted_ap_write_if_pending())
        {
          break;
        }

        if (req_off + 4u > req_len)
        {
          response_value = LibXR::USB::DapLinkV1Def::DAP_TRANSFER_ERROR;
          break;
        }

        uint32_t match_val = 0u;
        Memory::FastCopy(&match_val, &req[req_off], sizeof(match_val));
        req_off = static_cast<uint16_t>(req_off + 4u);

        uint32_t rdata = 0u;
        uint32_t retry = dap_state_.transfer_cfg.match_retry;
        bool matched = false;

        while (true)
        {
          if (AP)
          {
            EC = ap_read_retry(ADDR2B, rdata, ack);
          }
          else
          {
            EC = dp_read_retry(ADDR2B, rdata, ack);
          }

          response_value = MapAckToDapResp(ack);
          if (response_value != LibXR::USB::DapLinkV1Def::DAP_TRANSFER_OK)
          {
            break;
          }
          if (EC != ErrorCode::OK)
          {
            response_value = LibXR::USB::DapLinkV1Def::DAP_TRANSFER_ERROR;
            break;
          }

          if ((rdata & match_mask_) == (match_val & match_mask_))
          {
            matched = true;
            break;
          }

          if (retry == 0u)
          {
            break;
          }
          --retry;
        }

        if (response_value != LibXR::USB::DapLinkV1Def::DAP_TRANSFER_OK)
        {
          break;
        }

        if (!matched)
        {
          response_value =
              static_cast<uint8_t>(LibXR::USB::DapLinkV1Def::DAP_TRANSFER_OK |
                                   LibXR::USB::DapLinkV1Def::DAP_TRANSFER_MISMATCH);
          break;
        }

        response_value = LibXR::USB::DapLinkV1Def::DAP_TRANSFER_OK;
        response_count++;
        continue;
      }

      uint32_t rdata = 0u;
      if (!check_posted_ap_write_if_pending())
      {
        break;
      }

      if (AP)
      {
        EC = ap_read_retry(ADDR2B, rdata, ack);
      }
      else
      {
        EC = dp_read_retry(ADDR2B, rdata, ack);
      }

      response_value = MapAckToDapResp(ack);
      if (response_value != LibXR::USB::DapLinkV1Def::DAP_TRANSFER_OK)
      {
        break;
      }
      if (EC != ErrorCode::OK)
      {
        response_value = LibXR::USB::DapLinkV1Def::DAP_TRANSFER_ERROR;
        break;
      }

      if (!ensure_space(bytes_for_read(TS)))
      {
        response_value = LibXR::USB::DapLinkV1Def::DAP_TRANSFER_ERROR;
        break;
      }

      if (!emit_read_with_ts(TS, rdata))
      {
        response_value = LibXR::USB::DapLinkV1Def::DAP_TRANSFER_ERROR;
        break;
      }

      response_value = LibXR::USB::DapLinkV1Def::DAP_TRANSFER_OK;
    }

    if (dap_state_.transfer_abort)
    {
      dap_state_.transfer_abort = false;
      break;
    }
  }

  if (pending_ap_write && should_check_pending_ap_write_at_tail())
  {
    const uint8_t PRIOR_RESPONSE = response_value;
    if (check_posted_ap_write_if_pending())
    {
      response_value = PRIOR_RESPONSE;
    }
  }

  resp[1] = response_count;
  resp[2] = response_value;
  out_len = resp_off;
  return ErrorCode::OK;
}

template <typename SwdPort>
ErrorCode DapLinkV1Class<SwdPort>::HandleJtagTransferBlock(
    bool /*in_isr*/, const uint8_t* req, uint16_t req_len, uint8_t* resp,
    uint16_t resp_cap, uint16_t& out_len)
{
  if (!resp || resp_cap < 4u)
  {
    out_len = 0u;
    return ErrorCode::NOT_FOUND;
  }

  resp[0] = to_u8(LibXR::USB::DapLinkV1Def::CommandId::TRANSFER_BLOCK);
  resp[1] = 0u;
  resp[2] = 0u;
  resp[3] = 0u;
  out_len = 4u;

  if (!req || req_len < 5u || jtag_ == nullptr || jtag_chain_.count == 0u)
  {
    resp[3] = LibXR::USB::DapLinkV1Def::DAP_TRANSFER_ERROR;
    return ErrorCode::ARG_ERR;
  }

  if (dap_state_.transfer_abort)
  {
    dap_state_.transfer_abort = false;
    resp[3] = LibXR::USB::DapLinkV1Def::DAP_TRANSFER_ERROR;
    return ErrorCode::OK;
  }

  const uint8_t TAP_INDEX = req[1];
  if (TAP_INDEX >= jtag_chain_.count)
  {
    resp[3] = LibXR::USB::DapLinkV1Def::DAP_TRANSFER_ERROR;
    return ErrorCode::OK;
  }

  jtag_chain_.index = TAP_INDEX;
  LibXR::Debug::JtagProtocol::UpdateChainCache(jtag_chain_);

  uint16_t count = 0u;
  Memory::FastCopy(&count, &req[2], sizeof(count));

  const uint8_t DAP_RQ = req[4];

  if ((DAP_RQ & (LibXR::USB::DapLinkV1Def::DAP_TRANSFER_MATCH_VALUE |
                 LibXR::USB::DapLinkV1Def::DAP_TRANSFER_MATCH_MASK)) != 0u)
  {
    resp[3] = LibXR::USB::DapLinkV1Def::DAP_TRANSFER_ERROR;
    return ErrorCode::NOT_SUPPORT;
  }
  if (LibXR::USB::DapLinkV1Def::req_need_timestamp(DAP_RQ))
  {
    resp[3] = LibXR::USB::DapLinkV1Def::DAP_TRANSFER_ERROR;
    return ErrorCode::NOT_SUPPORT;
  }

  if (count == 0u)
  {
    const uint16_t DONE0 = 0u;
    Memory::FastCopy(&resp[1], &DONE0, sizeof(DONE0));
    resp[3] = LibXR::USB::DapLinkV1Def::DAP_TRANSFER_OK;
    out_len = 4u;
    return ErrorCode::OK;
  }

  LibXR::Debug::JtagDp dp(*jtag_, &jtag_chain_);

  auto idle_after_attempt = [&]()
  {
    if (dap_state_.transfer_cfg.idle_cycles != 0u)
    {
      jtag_->IdleClocks(dap_state_.transfer_cfg.idle_cycles);
    }
  };

  auto dp_read_retry = [&](uint8_t addr2b, uint32_t& val,
                           LibXR::Debug::JtagProtocol::Ack& ack) -> ErrorCode
  {
    uint32_t retry = dap_state_.transfer_cfg.retry_count;
    while (true)
    {
      const ErrorCode EC = dp.DpReadTxn(addr2b, val, ack);
      idle_after_attempt();
      if (ack != LibXR::Debug::JtagProtocol::Ack::WAIT || retry == 0u ||
          dap_state_.transfer_abort)
      {
        return EC;
      }
      --retry;
    }
  };

  auto dp_write_retry = [&](uint8_t addr2b, uint32_t val,
                            LibXR::Debug::JtagProtocol::Ack& ack) -> ErrorCode
  {
    uint32_t retry = dap_state_.transfer_cfg.retry_count;
    while (true)
    {
      const ErrorCode EC = dp.DpWriteTxn(addr2b, val, ack);
      idle_after_attempt();
      if (ack != LibXR::Debug::JtagProtocol::Ack::WAIT || retry == 0u ||
          dap_state_.transfer_abort)
      {
        return EC;
      }
      --retry;
    }
  };

  auto ap_read_retry = [&](uint8_t addr2b, uint32_t& val,
                           LibXR::Debug::JtagProtocol::Ack& ack) -> ErrorCode
  {
    uint32_t retry = dap_state_.transfer_cfg.retry_count;
    while (true)
    {
      const ErrorCode EC = dp.ApReadTxn(addr2b, val, ack);
      idle_after_attempt();
      if (ack != LibXR::Debug::JtagProtocol::Ack::WAIT || retry == 0u ||
          dap_state_.transfer_abort)
      {
        return EC;
      }
      --retry;
    }
  };

  auto ap_write_retry = [&](uint8_t addr2b, uint32_t val,
                            LibXR::Debug::JtagProtocol::Ack& ack) -> ErrorCode
  {
    uint32_t retry = dap_state_.transfer_cfg.retry_count;
    while (true)
    {
      const ErrorCode EC = dp.ApWriteTxn(addr2b, val, ack);
      idle_after_attempt();
      if (ack != LibXR::Debug::JtagProtocol::Ack::WAIT || retry == 0u ||
          dap_state_.transfer_abort)
      {
        return EC;
      }
      --retry;
    }
  };

  auto check_posted_ap_write_jtag = [&](LibXR::Debug::JtagProtocol::Ack& ack) -> ErrorCode
  {
    uint32_t ctrl_stat = 0u;
    return dp_read_retry(
        static_cast<uint8_t>(LibXR::Debug::SwdProtocol::DpReadReg::CTRL_STAT), ctrl_stat,
        ack);
  };

  const bool AP = LibXR::USB::DapLinkV1Def::req_is_ap(DAP_RQ);
  const bool RNW = LibXR::USB::DapLinkV1Def::req_is_read(DAP_RQ);
  const uint8_t ADDR2B = LibXR::USB::DapLinkV1Def::req_addr2b(DAP_RQ);

  uint16_t done = 0u;
  uint8_t xresp = 0u;

  uint16_t req_off = 5u;
  uint16_t resp_off = 4u;

  if (!RNW)
  {
    for (uint32_t i = 0; i < count; ++i)
    {
      LibXR::Debug::JtagProtocol::Ack ack = LibXR::Debug::JtagProtocol::Ack::PROTOCOL;
      ErrorCode EC = ErrorCode::OK;

      if (req_off + 4u > req_len)
      {
        xresp |= LibXR::USB::DapLinkV1Def::DAP_TRANSFER_ERROR;
        break;
      }

      uint32_t wdata = 0u;
      Memory::FastCopy(&wdata, &req[req_off], sizeof(wdata));
      req_off = static_cast<uint16_t>(req_off + 4u);

      if (AP)
      {
        EC = ap_write_retry(ADDR2B, wdata, ack);
      }
      else
      {
        EC = dp_write_retry(ADDR2B, wdata, ack);
      }

      xresp = MapAckToDapResp(ack);
      if (xresp == 0u)
      {
        xresp |= LibXR::USB::DapLinkV1Def::DAP_TRANSFER_ERROR;
        break;
      }

      if (ack != LibXR::Debug::JtagProtocol::Ack::OK)
      {
        break;
      }

      if (EC != ErrorCode::OK)
      {
        xresp |= LibXR::USB::DapLinkV1Def::DAP_TRANSFER_ERROR;
        break;
      }

      done = static_cast<uint16_t>(i + 1u);
    }

    if (AP && xresp == LibXR::USB::DapLinkV1Def::DAP_TRANSFER_OK && done > 0u)
    {
      LibXR::Debug::JtagProtocol::Ack ack = LibXR::Debug::JtagProtocol::Ack::PROTOCOL;
      const ErrorCode EC = check_posted_ap_write_jtag(ack);
      const uint8_t V = MapAckToDapResp(ack);
      if (V != LibXR::USB::DapLinkV1Def::DAP_TRANSFER_OK)
      {
        xresp = V;
      }
      else if (EC != ErrorCode::OK)
      {
        xresp |= LibXR::USB::DapLinkV1Def::DAP_TRANSFER_ERROR;
      }
    }

    Memory::FastCopy(&resp[1], &done, sizeof(done));
    resp[3] = xresp;
    out_len = resp_off;
    return ErrorCode::OK;
  }

  for (uint32_t i = 0; i < count; ++i)
  {
    LibXR::Debug::JtagProtocol::Ack ack = LibXR::Debug::JtagProtocol::Ack::PROTOCOL;
    ErrorCode EC = ErrorCode::OK;
    uint32_t rdata = 0u;

    if (resp_off + 4u > resp_cap)
    {
      xresp |= LibXR::USB::DapLinkV1Def::DAP_TRANSFER_ERROR;
      break;
    }

    if (AP)
    {
      EC = ap_read_retry(ADDR2B, rdata, ack);
    }
    else
    {
      EC = dp_read_retry(ADDR2B, rdata, ack);
    }

    xresp = MapAckToDapResp(ack);
    if (xresp == 0u)
    {
      xresp |= LibXR::USB::DapLinkV1Def::DAP_TRANSFER_ERROR;
      break;
    }

    if (ack != LibXR::Debug::JtagProtocol::Ack::OK)
    {
      break;
    }

    if (EC != ErrorCode::OK)
    {
      xresp |= LibXR::USB::DapLinkV1Def::DAP_TRANSFER_ERROR;
      break;
    }

    Memory::FastCopy(&resp[resp_off], &rdata, sizeof(rdata));
    resp_off = static_cast<uint16_t>(resp_off + 4u);
    done = static_cast<uint16_t>(i + 1u);
  }

  Memory::FastCopy(&resp[1], &done, sizeof(done));
  resp[3] = xresp;
  out_len = resp_off;
  return ErrorCode::OK;
}

template <typename SwdPort>
ErrorCode DapLinkV1Class<SwdPort>::HandleJtagWriteAbort(bool /*in_isr*/,
                                                        const uint8_t* req,
                                                        uint16_t req_len, uint8_t* resp,
                                                        uint16_t resp_cap,
                                                        uint16_t& out_len)
{
  if (!resp || resp_cap < 2u)
  {
    out_len = 0u;
    return ErrorCode::ARG_ERR;
  }

  resp[0] = to_u8(LibXR::USB::DapLinkV1Def::CommandId::WRITE_ABORT);
  resp[1] = to_u8(LibXR::USB::DapLinkV1Def::Status::ERROR);
  out_len = 2u;

  if (!req || req_len < 6u || jtag_ == nullptr || jtag_chain_.count == 0u)
  {
    return ErrorCode::ARG_ERR;
  }

  const uint8_t TAP_INDEX = req[1];
  if (TAP_INDEX >= jtag_chain_.count)
  {
    return ErrorCode::OK;
  }

  jtag_chain_.index = TAP_INDEX;
  LibXR::Debug::JtagProtocol::UpdateChainCache(jtag_chain_);

  uint32_t flags = 0u;
  Memory::FastCopy(&flags, &req[2], sizeof(flags));

  LibXR::Debug::JtagDp dp(*jtag_, &jtag_chain_);
  LibXR::Debug::JtagProtocol::Ack ack = LibXR::Debug::JtagProtocol::Ack::PROTOCOL;
  const ErrorCode EC = dp.WriteAbort(flags, ack);
  const uint8_t V = MapAckToDapResp(ack);

  if (EC == ErrorCode::OK && V == LibXR::USB::DapLinkV1Def::DAP_TRANSFER_OK)
  {
    resp[1] = to_u8(LibXR::USB::DapLinkV1Def::Status::OK);
  }

  return ErrorCode::OK;
}

template <typename SwdPort>
void DapLinkV1Class<SwdPort>::DriveReset(bool release)
{
  last_nreset_level_high_ = release;

  if (release)
  {
    swj_shadow_ |= LibXR::USB::DapLinkV1Def::DAP_SWJ_NRESET;
  }
  else
  {
    swj_shadow_ = static_cast<uint8_t>(
        swj_shadow_ & static_cast<uint8_t>(~LibXR::USB::DapLinkV1Def::DAP_SWJ_NRESET));
  }

  if (nreset_gpio_ != nullptr)
  {
    (void)nreset_gpio_->Write(release);
  }
}

template <typename SwdPort>
void DapLinkV1Class<SwdPort>::ResetJtagChainState()
{
  if constexpr (JTAG_ENABLED)
  {
    jtag_chain_.ir_length = jtag_ir_len_;
    jtag_chain_.count = 0u;
    jtag_chain_.index = 0u;
    jtag_chain_.ir_before_bits_len = 0u;
    jtag_chain_.ir_after_bits_len = 0u;
    jtag_chain_.dr_before_bits_len = 0u;
    jtag_chain_.dr_after_bits_len = 0u;
    Memory::FastSet(jtag_ir_len_, 0, sizeof(jtag_ir_len_));
  }
}

}  // namespace LibXR::USB
