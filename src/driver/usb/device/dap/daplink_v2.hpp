#pragma once

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "daplink_v2_def.hpp"
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
#include "structure/slot_pool.hpp"
#include "timebase.hpp"

namespace LibXR::USB
{
#ifndef LIBXR_USB_DAPLINK_V2_BUILD_CONFIG_DEFINED
#define LIBXR_USB_DAPLINK_V2_BUILD_CONFIG_DEFINED
/// DAP v2 的编译配置；profile 头文件可以先定义它 / Build configuration of DAP v2; a
/// profile header may define it first
namespace DapLinkV2BuildConfig
{
/// 是否编入 JTAG 支持；未包含 profile 头文件时为 true / Whether JTAG support is
/// compiled in; true when no profile header is included
static constexpr bool ENABLE_JTAG = true;
}  // namespace DapLinkV2BuildConfig
#endif

/**
 * @class DapLinkV2Class
 * @brief CMSIS-DAP v2（批量）USB 类 / CMSIS-DAP v2 (bulk) USB class
 *
 * 一个厂商接口，两个批量端点：OUT 收命令，IN 发响应；由 WinUSB 绑定。支持 SWD 和
 * JTAG（daplink_v2_profile_swd.hpp 可去掉 JTAG），可选 nRESET。
 * One vendor interface with two bulk endpoints: OUT receives commands, IN sends
 * responses; WinUSB binds it. SWD and JTAG (daplink_v2_profile_swd.hpp leaves JTAG
 * out), optional nRESET.
 *
 * @note 三种上下文。owner（设备的串行执行者）：WriteDescriptors、BindEndpoints、
 *       UnbindEndpoints、端点完成回调和 OnService；它把 OUT 包拼成命令交给 Step 线程，
 *       并把暂存的响应提交给 IN。Step 线程（应用调用 Step() 的线程）：执行命令，SWD 和
 *       JTAG 只在这里访问；没有暂存响应在排队时，它把响应直接写进借来的 IN 槽并在本线程
 *       提交，Submit 可能就地运行 owner。设置函数（SetInfoStrings、SetJtag、
 *       SetCommandHook）在设备启动前调用。
 *       Three contexts. The owner (the device's serialized executor): WriteDescriptors,
 *       BindEndpoints, UnbindEndpoints, the endpoint completion callbacks and OnService;
 *       it assembles OUT packets into commands for the Step thread and submits staged
 *       responses to IN. The Step thread (the thread that calls Step()): executes the
 *       commands and is the only one to touch SWD and JTAG; with no staged response
 *       waiting it writes the response straight into a borrowed IN slot and submits it
 *       from this thread, where Submit may run the owner in place. The setters
 *       (SetInfoStrings, SetJtag, SetCommandHook) are called before the device starts.
 * @note 会话：每次绑定和解绑使 epoch_ 加一。命令带着收到时的 epoch，Step 线程据此丢弃
 *       旧会话的命令和响应，并在自己的线程里关闭 SWD、恢复默认状态（SyncSession）。
 *       Session: every bind and unbind adds one to epoch_. A command carries the epoch
 *       it was received in; with it the Step thread drops commands and responses of an
 *       old session, and it closes SWD and restores the defaults in its own thread
 *       (SyncSession).
 *
 * @tparam SwdPort SWD 链路类型 / SWD link type
 * @tparam DefaultDapPacketSize 全速批量下的 DAP 包长下限 / DAP packet size the
 *         response slots must at least hold
 * @tparam AdvertisedPacketCount DAP_Info 通告的在途命令数（实际最多 4）/ Commands in
 *         flight announced by DAP_Info (4 at most in effect)
 * @tparam MaxDapPacketSize 主机可见的 DAP 包长上限 / Upper bound of the DAP packet size
 *         the host sees
 */
template <typename SwdPort, uint16_t DefaultDapPacketSize = 512u,
          uint8_t AdvertisedPacketCount = 8u, uint16_t MaxDapPacketSize = 1024u>
class DapLinkV2Class : public DeviceClass
{
 public:
  /// 编译期是否包含 JTAG 支持 / Whether JTAG support is compiled in
  static constexpr bool JTAG_ENABLED = DapLinkV2BuildConfig::ENABLE_JTAG;
  /// 默认的接口字符串 / Default interface string
  static constexpr const char* DEFAULT_INTERFACE_STRING = "CMSIS-DAP v2";

  /// DAP_Info 返回的字符串；空指针项返回空字符串 / Strings returned by DAP_Info; a null
  /// entry yields an empty string
  struct InfoStrings
  {
    const char* vendor = nullptr;          ///< 厂商字符串 / Vendor string
    const char* product = nullptr;         ///< 产品字符串 / Product string
    const char* serial = nullptr;          ///< 序列号字符串 / Serial string
    const char* firmware_ver = nullptr;    ///< 固件版本 / Firmware version
    const char* device_vendor = nullptr;   ///< 设备厂商 / Device vendor
    const char* device_name = nullptr;     ///< 设备名称 / Device name
    const char* board_vendor = nullptr;    ///< 板卡厂商 / Board vendor
    const char* board_name = nullptr;      ///< 板卡名称 / Board name
    const char* product_fw_ver = nullptr;  ///< 产品固件版本 / Product FW version
  };

  /**
   * @brief 构造 / Construct
   * @param data_in_ep_num 批量 IN 端点号 / Bulk IN endpoint number
   * @param data_out_ep_num 批量 OUT 端点号 / Bulk OUT endpoint number
   * @param swd_link SWD 链路，须比本对象长寿 / SWD link; must outlive this object
   * @param nreset_gpio 可选的 nRESET 引脚 / Optional nRESET pin
   * @param interface_string 接口字符串，须保持有效 / Interface string; must stay valid
   * @note 不碰 SWD/JTAG，不起线程；命令由应用线程循环调用 Step() 执行。
   *       Touches neither SWD nor JTAG and starts no thread; commands run when the
   *       application thread calls Step() in a loop.
   */
  explicit DapLinkV2Class(Endpoint::EPNumber data_in_ep_num,
                          Endpoint::EPNumber data_out_ep_num, SwdPort& swd_link,
                          LibXR::GPIO* nreset_gpio = nullptr,
                          const char* interface_string = DEFAULT_INTERFACE_STRING)
      : swd_(swd_link),
        nreset_gpio_(nreset_gpio),
        interface_string_(interface_string),
        data_in_num_(data_in_ep_num),
        data_out_num_(data_out_ep_num),
        data_out_cb_(
            Callback<const Endpoint::Completion&>::Create(OnDataOutCompleteStatic, this)),
        data_in_cb_(
            Callback<const Endpoint::Completion&>::Create(OnDataInCompleteStatic, this))
  {
  }

  /// 设置 DAP_Info 的字符串；启动前 / Set the DAP_Info strings; before start
  void SetInfoStrings(const InfoStrings& info) { info_ = info; }

  /**
   * @brief 设置 JTAG 后端 / Set the JTAG backend
   * @param jtag JTAG 驱动，其时钟设为当前 SWJ 时钟；未编译 JTAG 时忽略。
   *        JTAG driver, its clock set to the current SWJ clock; ignored when JTAG is not
   *        compiled in.
   * @note 启动前调用。 / Call before start.
   */
  void SetJtag(LibXR::Debug::Jtag* jtag)
  {
    if constexpr (JTAG_ENABLED)
    {
      jtag_ = jtag;
      jtag_chain_.ir_length = jtag_ir_len_;
      ResetJtagChainState();
      if (jtag_ != nullptr)
      {
        (void)jtag_->SetClockHz(swj_clock_hz_);
      }
    }
    else
    {
      (void)jtag;
    }
  }

  /// DAP 会话状态；在 Step 线程里或 Step 不运行时读 / DAP session state; read in the
  /// Step thread or while Step does not run
  const LibXR::USB::DapLinkV2Def::State& GetState() const { return dap_state_; }

  /// 端点已绑定（主机已配置本接口）；任何上下文 / The endpoints are bound (the host
  /// has configured this interface); any context
  bool IsInited() const { return in_ep_ref_.load(std::memory_order_acquire) != nullptr; }

  /**
   * @brief 命令钩子，在标准分发之前调用 / Command hook called before the standard
   *        dispatch
   * @return true：命令已处理（响应已写）/ true: the command was handled (the response,
   *         if any, written)
   */
  using CommandHookFun = bool (*)(void* ctx, const uint8_t* req, uint16_t req_len,
                                  uint8_t* resp, uint16_t resp_cap, uint16_t& out_len);

  /// 注册命令钩子，ctx 原样传回；启动前 / Register the command hook, ctx passed
  /// through; before start
  void SetCommandHook(CommandHookFun hook, void* ctx = nullptr)
  {
    command_hook_ = hook;
    command_hook_ctx_ = ctx;
  }

  /**
   * @brief 在调用线程里同步执行一条 DAP 命令，供 USB 以外的传输使用（例如 TCP）
   *        / Execute one DAP command synchronously in the caller's thread, for transports
   *        other than USB (for example TCP)
   * @param req 请求 / Request
   * @param req_len 请求长度 / Request length
   * @param resp 响应缓冲 / Response buffer
   * @param resp_cap 响应缓冲容量 / Response buffer capacity
   * @param out_len 响应长度，0 表示没有响应 / Response length; 0 means no response
   * @return 命令处理结果 / Result of the command
   * @note 延时类命令在本调用里等到完成。本入口不检查、也不排他 USB：调用方先停掉 USB，
   *       二者不能同时驱动同一个实例。
   *       Delay-type commands are waited for inside this call. This entry neither checks
   *       nor excludes USB: the caller stops USB first, and the two must not drive the
   *       same instance at the same time.
   */
  ErrorCode ExecuteExternal(const uint8_t* req, uint16_t req_len, uint8_t* resp,
                            uint16_t resp_cap, uint16_t& out_len)
  {
    out_len = 0u;
    return ProcessOneCommand(req, req_len, resp, resp_cap, out_len);
  }

  /**
   * @brief 在调用线程里做一次 DAP 工作：等一条命令并执行它 / Do one unit of DAP work in
   *        the calling thread: wait for a command and execute it
   * @param timeout_ms 等命令或响应缓冲的最长时间；0 不等 / Longest wait for a command
   *        or a response buffer; 0 does not wait
   * @return OK：做了一件事（执行或丢弃一条命令，或落实了会话的开始/结束）；TIMEOUT：
   *         期限内没有工作，或命令还在等响应缓冲（下一次 Step 接着等）。
   *         OK: something was done (a command executed or dropped, or the session
   *         start/end applied); TIMEOUT: no work arrived in time, or the command still
   *         waits for a response buffer (the next Step goes on waiting).
   * @note 只在一个应用线程里循环调用，例如 while (true) { dap.Step(); }。没有暂存响应
   *       在排队且有空闲 IN 槽时，响应直接写进 IN 槽并由本线程提交；否则写进暂存槽，由
   *       owner 在 IN 槽空出时拷进去提交。暂存槽用完时在这里等。
   *       Call it in a loop from one application thread only, for example
   *       while (true) { dap.Step(); }. With no staged response waiting and an IN slot
   *       free, the response is written into the IN slot and submitted by this thread;
   *       otherwise it goes into a staging slot and the owner copies it into an IN slot
   *       and submits it when one is free. With the staging slots used up it waits here.
   */
  ErrorCode Step(uint32_t timeout_ms = UINT32_MAX)
  {
    if (cur_cmd_ == SLOT_NONE)
    {
      size_t slot = SLOT_NONE;
      if (cmd_pool_.Take(slot) != ErrorCode::OK)
      {
        if (cmd_sem_.Wait(timeout_ms) != ErrorCode::OK)
        {
          return ErrorCode::TIMEOUT;
        }
        if (cmd_pool_.Take(slot) != ErrorCode::OK)
        {
          // 被绑定/解绑叫醒，没有命令：把会话的开始/结束落实到 SWD。
          // Woken by a bind/unbind with no command: apply the session start/end to SWD.
          SyncSession(epoch_.load(std::memory_order_acquire));
          return ErrorCode::OK;
        }
      }
      cur_cmd_ = slot;
      cur_epoch_ = cmd_pool_.At(slot).epoch;
    }
    // 收到这条命令之后解绑或重新绑定过：它属于已结束的会话，丢弃。
    // Unbound or rebound since this command was received: it belongs to an ended
    // session and is dropped.
    if (!StepStillBound())
    {
      AbandonCurrent();
      SyncSession(epoch_.load(std::memory_order_acquire));
      return ErrorCode::OK;
    }
    SyncSession(cur_epoch_);
    if (cur_resp_ == SLOT_NONE && !cur_in_.Valid())
    {
      TakeInSlot();
    }
    while (cur_resp_ == SLOT_NONE && !cur_in_.Valid())
    {
      if (resp_pool_.Acquire(cur_resp_) == ErrorCode::OK)
      {
        break;
      }
      cur_resp_ = SLOT_NONE;
      if (slot_sem_.Wait(timeout_ms) != ErrorCode::OK)
      {
        return ErrorCode::TIMEOUT;
      }
      if (!StepStillBound())
      {
        AbandonCurrent();
        return ErrorCode::OK;
      }
    }
    ExecuteCurrent();
    return ErrorCode::OK;
  }

 protected:
  /// @name DeviceClass 的实现 / DeviceClass implementation
  /// @{
  size_t GetInterfaceCount() const override { return 1U; }
  bool HasIAD() const override { return false; }
  size_t GetMaxConfigSize() const override { return sizeof(desc_block_); }

  const char* GetInterfaceString(size_t local_interface_index) const override
  {
    return (local_interface_index == 0U) ? interface_string_ : nullptr;
  }

  /// 要 WinUSB 绑定：设备接口 GUID / WinUSB should bind: the device interface GUID
  const char* GetWinUsbInterfaceGuid() const override
  {
    return "{CDB3B5AD-293B-4663-AA36-1AAE46463776}";
  }

  /**
   * @brief 写接口描述符和两个批量端点描述符 / Write the interface descriptor and the two
   *        bulk endpoint descriptors
   * @return OK；NOT_SUPPORT：某个端点在该速度下查询得 0。
   *         OK; NOT_SUPPORT: an endpoint's query returned 0 at that speed.
   */
  ErrorCode WriteDescriptors(const EndpointPool& pool, Speed speed,
                             uint8_t first_interface, ConstRawData& block) override
  {
    bool ok = WriteEndpointDescriptor(pool, speed, data_out_num_, DATA_OUT_CONFIG, 0U,
                                      desc_block_.ep_out);
    ok = WriteEndpointDescriptor(pool, speed, data_in_num_, DATA_IN_CONFIG, 0U,
                                 desc_block_.ep_in) &&
         ok;
    if (!ok)
    {
      return ErrorCode::NOT_SUPPORT;
    }
    desc_block_.intf = {9U,
                        static_cast<uint8_t>(DescriptorType::INTERFACE),
                        first_interface,
                        0U,
                        2U,
                        VENDOR_SPECIFIC,
                        0U,
                        0U,
                        GetInterfaceStringIndex(0U)};
    block = {&desc_block_, sizeof(desc_block_)};
    return ErrorCode::OK;
  }

  /**
   * @brief 取两个端点、Configure、比较包长，开始新会话并挂接收 / Take both endpoints,
   *        configure them, compare the sizes, start a new session and arm the receives
   * @return OK；NOT_FOUND：Get 失败；CHECK_ERR：读回的包长与描述符不同；其他：Configure
   *         的错误。失败时已取的端点留在成员里，由随后的 UnbindEndpoints 收回。
   *         OK; NOT_FOUND: a Get failed; CHECK_ERR: a size read back differs from the
   *         descriptor; otherwise the error of Configure. On failure the endpoints taken
   *         stay in the members and the following UnbindEndpoints takes them back.
   * @note 通告给主机的 DAP 包长取 MaxDapPacketSize 与 IN 的 MaxTransferSize() 中较小的：
   *       响应原地写在 IN 槽里。不碰 SWD，会话状态由 Step 线程复位。
   *       The DAP packet size announced to the host is the smaller of MaxDapPacketSize
   *       and the IN endpoint's MaxTransferSize(): the response is written in place in
   *       an IN slot. SWD is not touched; the Step thread resets the session state.
   */
  [[nodiscard]] ErrorCode BindEndpoints(EndpointPool& pool, uint8_t first_interface,
                                        bool in_isr) override
  {
    UNUSED(first_interface);
    ErrorCode result = BindEndpoint(pool, data_out_num_, DATA_OUT_CONFIG,
                                    desc_block_.ep_out.wMaxPacketSize, in_isr, data_out_);
    if (result == ErrorCode::OK)
    {
      result = BindEndpoint(pool, data_in_num_, DATA_IN_CONFIG,
                            desc_block_.ep_in.wMaxPacketSize, in_isr, data_in_);
    }
    if (result != ErrorCode::OK)
    {
      return result;
    }
    const size_t in_capacity = data_in_->MaxTransferSize();
    dap_packet_size_.store((in_capacity < MAX_DAP_PACKET_SIZE)
                               ? static_cast<uint16_t>(in_capacity)
                               : MAX_DAP_PACKET_SIZE,
                           std::memory_order_release);
    data_out_->SetOnComplete(data_out_cb_);
    data_in_->SetOnComplete(data_in_cb_);
    ResetChannels();
    in_ep_ref_.store(data_in_, std::memory_order_release);
    TopUpReceives(in_isr);
    return ErrorCode::OK;
  }

  /**
   * @brief 结束会话，关闭并交回两个端点 / End the session, close and return both
   *        endpoints
   * @note 依次：IN 引用置空（Step 不再借新槽），置传输中止标志，ResetChannels（epoch 加
   *       一，交回留住的 OUT 槽，排空命令池和暂存池），叫醒 Step，最后每个端点先把指针
   *       置空再 Close、Release。epoch 先于 Close 加一，见 TakeInSlot。Close 交付的完成
   *       回调看到指针为空，什么也不做。SWD 由 Step 线程关闭。
   *       In order: the IN reference is cleared (Step borrows no new slot), the transfer
   *       abort flag is set, ResetChannels (epoch plus one, the held OUT slots handed
   *       back, the command and staging pools drained), Step is woken, and last each
   *       endpoint's pointer is cleared before its Close and Release. The epoch moves
   *       before Close, see TakeInSlot. The completions Close delivers see a null pointer
   *       and do nothing. The Step thread closes SWD.
   */
  void UnbindEndpoints(EndpointPool& pool, bool in_isr) override
  {
    in_ep_ref_.store(nullptr, std::memory_order_release);
    transfer_abort_.store(true, std::memory_order_release);
    ResetChannels();
    cmd_sem_.PostFromCallback(in_isr);
    slot_sem_.PostFromCallback(in_isr);
    UnbindEndpoint(pool, data_in_, in_isr);
    UnbindEndpoint(pool, data_out_, in_isr);
  }

  /// 两个端点地址之一；由端点号算出 / One of the two endpoint addresses; computed from
  /// the numbers
  bool OwnsEndpoint(uint8_t ep_addr) const override
  {
    return ep_addr == Endpoint::EPNumberToAddr(data_in_num_, Endpoint::Direction::IN) ||
           ep_addr == Endpoint::EPNumberToAddr(data_out_num_, Endpoint::Direction::OUT);
  }

  /**
   * @brief 提交暂存的响应，处理留住的包，补挂接收 / Submit the staged responses, handle
   *        the held packets, re-arm the receives
   * @note Step 线程在放出命令槽或发布暂存响应后经 RequestService 请求；未绑定时什么也
   *       不做。
   *       Requested through RequestService by the Step thread after it gave back a
   *       command slot or published a staged response; does nothing while unbound.
   */
  void OnService(bool in_isr) override
  {
    if (data_in_ == nullptr || data_out_ == nullptr)
    {
      return;
    }
    SendStagedResponses(in_isr);
    DrainHeldPackets(in_isr);
    TopUpReceives(in_isr);
  }
  /// @}

 private:
  /// 将枚举或整数转为 uint8_t / Cast an enum or integer to uint8_t
  template <typename E>
  static constexpr uint8_t ToU8(E e)
  {
    return static_cast<uint8_t>(e);
  }

  static constexpr uint8_t DAP_OK = 0x00u;     ///< DAP_OK
  static constexpr uint8_t DAP_ERROR = 0xFFu;  ///< DAP_ERROR

  /// 未知命令响应：单字节 0xFF / Unknown command response: one byte 0xFF
  static inline ErrorCode BuildUnknownCmdResponse(uint8_t* resp, uint16_t cap,
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
  static inline ErrorCode BuildCmdStatusResponse(uint8_t cmd, uint8_t status,
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

  /// OUT 完成的入口；未绑定时 OUT 只以 ABORTED 结束、不借出槽 / Entry of an OUT
  /// completion; while unbound an OUT only ends with ABORTED and lends no slot
  static void OnDataOutCompleteStatic(bool in_isr, DapLinkV2Class* self,
                                      const Endpoint::Completion& completion)
  {
    if (self->data_out_ == nullptr)
    {
      ASSERT(!completion.slot.Valid());
      return;
    }
    self->OnDataOutComplete(in_isr, completion);
  }

  /// IN 完成的入口；未绑定时什么也不做 / Entry of an IN completion; nothing while unbound
  static void OnDataInCompleteStatic(bool in_isr, DapLinkV2Class* self,
                                     const Endpoint::Completion& completion)
  {
    UNUSED(completion);
    if (self->data_in_ != nullptr)
    {
      self->SendStagedResponses(in_isr);
    }
  }

  /**
   * @brief OUT 完成：借来的槽里是刚收到的一个或几个包 / OUT completion: the lent slot
   *        holds the packet or packets just received
   * @note 包拼进命令池的槽，拼完的命令发布给 Step 线程，槽立即交回。命令池没有空槽时
   *       包不拷走，槽按到达顺序留在 held_ 里，端点没有空槽便 NAK，直到 Step 放出命令
   *       槽、OnService 处理留住的包。结果不是 OK（Stall 或 Flush 停了接收）时字节流失去
   *       同步，丢弃正在拼装的命令并重新挂接收；STALLED 期间挂的接收排队到 ClearStall。
   *       Packets are assembled into command pool slots, a finished command is
   *       published to the Step thread and the slot goes back at once. With no free
   *       command slot the packet is not copied and the slot stays in held_, in arrival
   *       order; the endpoint NAKs for lack of a slot until Step gives back a command
   *       slot and OnService handles the held packets. A result other than OK (a Stall
   *       or Flush stopped the receive) loses byte-stream sync: the command being
   *       assembled is dropped and the receive armed again; one armed while STALLED
   *       waits in the queue until ClearStall.
   */
  void OnDataOutComplete(bool in_isr, const Endpoint::Completion& completion)
  {
    if (armed_n_ > 0u)
    {
      // 按提交顺序完成：完成的是最早挂的那个 / Completions come in submission order:
      // this one is the oldest armed
      armed_head_ = RingAt(armed_head_, 1u, ARMED_RING_SIZE);
      --armed_n_;
    }
    if (completion.result != ErrorCode::OK)
    {
      ResetStream();
      TopUpReceives(in_isr);
      return;
    }
    const uint16_t size = static_cast<uint16_t>(completion.actual);
    if (held_n_ > 0u || !ConsumePacket(completion.slot, size, in_isr))
    {
      ASSERT(held_n_ < HELD_RING_SIZE);
      held_[RingAt(held_head_, held_n_, HELD_RING_SIZE)] = {completion.slot, size};
      ++held_n_;
    }
    TopUpReceives(in_isr);
  }

  /**
   * @brief 把暂存的响应按发布顺序拷进空闲的 IN 槽并提交 / Copy the staged responses, in
   *        publish order, into free IN slots and submit them
   * @note 仅 owner。IN 槽都忙时留到下一次 IN 完成。旧会话的响应丢弃：这里与解绑、
   *       绑定串行，比较结果在提交之前不变。响应短于 DAP 包长时要求结尾零长度包，是否
   *       真的发由 Submit 按包长定。
   *       Owner only. With every IN slot busy it waits for the next IN completion. A
   *       response of an old session is dropped: this is serialized with unbind and
   *       bind, so the comparison holds until the submit. A response shorter than the
   *       DAP packet asks for the closing zero-length packet; Submit decides from the
   *       packet size whether one is sent.
   */
  void SendStagedResponses(bool in_isr)
  {
    while (true)
    {
      if (send_idx_ == SLOT_NONE && resp_pool_.Take(send_idx_) != ErrorCode::OK)
      {
        send_idx_ = SLOT_NONE;
        return;
      }
      const ResponseSlot& response = resp_pool_.At(send_idx_);
      if (response.epoch == epoch_.load(std::memory_order_acquire))
      {
        Endpoint::Slot slot;
        if (data_in_->AcquireSlot(slot) != ErrorCode::OK)
        {
          return;
        }
        const size_t length = std::min<size_t>(response.len, slot.data.size_);
        Memory::FastCopy(slot.data.addr_, response.payload, length);
        const ErrorCode result =
            data_in_->Submit(slot, length, length < GetDapPacketSize(), in_isr);
        ASSERT(result == ErrorCode::OK);
        UNUSED(result);
      }
      resp_pool_.Release(send_idx_);
      send_idx_ = SLOT_NONE;
      // 这条暂存响应已进端点队列或已丢弃：计数归零后 Step 可以直接提交而不越过它。
      // This staged response is queued on the endpoint or dropped: once the count is
      // zero Step may submit directly without overtaking it.
      uint32_t staged = staged_outstanding_.load(std::memory_order_acquire);
      while (staged != 0u && !staged_outstanding_.compare_exchange_weak(
                                 staged, staged - 1u, std::memory_order_acq_rel,
                                 std::memory_order_acquire))
      {
      }
      slot_sem_.PostFromCallback(in_isr);
    }
  }

  /**
   * @brief Step 线程：没有暂存响应在排队时借一个 IN 槽来直接写响应 / Step thread:
   *        borrow an IN slot to write the response in place when no staged response
   *        waits
   * @note 借到的槽可能属于借之前就已结束的绑定（读引用与借之间发生了解绑和别的绑定）。
   *       提交前 CompleteResponse 比较 epoch：解绑先于 Close 把 epoch 加一，借出的比较
   *       交换看到新绑定时，本线程随后读 epoch 一定看到加一（EndpointSlots 的比较交换
   *       都是 acq_rel，推断）；比较之后才发生的解绑使槽作废，Submit 得 ABORTED。
   *       The slot may belong to a binding that ended before the lend (an unbind and
   *       another bind between reading the reference and the lend). CompleteResponse
   *       compares the epoch before the submit: the unbind adds one to the epoch before
   *       Close, and once the lending compare-and-swap sees the new binding, this
   *       thread's later epoch load sees the increment (every compare-and-swap of
   *       EndpointSlots is acq_rel; inferred). An unbind after the comparison makes the
   *       slot stale and Submit returns ABORTED.
   */
  void TakeInSlot()
  {
    Endpoint* ep = in_ep_ref_.load(std::memory_order_acquire);
    if (ep == nullptr || staged_outstanding_.load(std::memory_order_acquire) != 0u)
    {
      return;
    }
    Endpoint::Slot slot;
    if (ep->AcquireSlot(slot) != ErrorCode::OK)
    {
      return;
    }
    cur_in_ = slot;
    cur_in_ep_ = ep;
  }

  /**
   * @brief Step 线程：提交直接写好的 IN 槽 / Step thread: submit the IN slot written in
   *        place
   * @note 短于 DAP 包长的响应要求结尾零长度包：主机按 DAP 包长读，整包结尾的短响应要
   *       零长度包结束读；等于 DAP 包长的读满即止，补了会占住一个 IN 槽。ABORTED 表示
   *       槽在 Close 之前借出、现已空闲。
   *       A response shorter than the DAP packet asks for the closing zero-length
   *       packet: the host reads DAP-packet-sized buffers, so a short response ending
   *       on a whole packet needs one to end the read; one exactly as long as the DAP
   *       packet ends the read by filling it, and a zero-length packet would keep an IN
   *       slot busy. ABORTED means the slot was lent before a Close and is free again.
   */
  void SubmitInSlot(uint16_t size)
  {
    Endpoint* ep = cur_in_ep_;
    const Endpoint::Slot slot = cur_in_;
    cur_in_ = {};
    cur_in_ep_ = nullptr;
    const ErrorCode result = ep->Submit(slot, size, size < GetDapPacketSize(), false);
    ASSERT(result == ErrorCode::OK || result == ErrorCode::ABORTED);
    UNUSED(result);
  }

  /**
   * @brief 会话开始或结束：放掉 owner 一侧持有的东西 / Session start or end: give back
   *        what the owner side holds
   * @note 仅 owner（绑定、解绑）。epoch 加一，字节流归零，留住的 OUT 槽交回，排空命令池
   *       和暂存池里已就绪的槽。Step 线程手里的槽由它自己放（AbandonCurrent）。
   *       Owner only (bind, unbind). Adds one to the epoch, resets the byte stream,
   *       hands the held OUT slots back and drains the ready slots of the command and
   *       staging pools. The slots in the Step thread's hands are given back by it
   *       (AbandonCurrent).
   */
  void ResetChannels()
  {
    epoch_.fetch_add(1u, std::memory_order_acq_rel);
    ResetStream();
    for (uint8_t i = 0u; i < held_n_; ++i)
    {
      (void)data_out_->ReleaseSlot(held_[RingAt(held_head_, i, HELD_RING_SIZE)].slot);
    }
    held_head_ = 0u;
    held_n_ = 0u;
    armed_head_ = 0u;
    armed_n_ = 0u;
    if (send_idx_ != SLOT_NONE)
    {
      resp_pool_.Release(send_idx_);
      send_idx_ = SLOT_NONE;
    }
    size_t slot = SLOT_NONE;
    while (cmd_pool_.Take(slot) == ErrorCode::OK)
    {
      cmd_pool_.Release(slot);
    }
    while (resp_pool_.Take(slot) == ErrorCode::OK)
    {
      resp_pool_.Release(slot);
    }
    staged_outstanding_.store(0u, std::memory_order_release);
  }

  /// 字节流状态归零，放掉正在拼装的命令槽 / Reset the byte-stream state, give back the
  /// command slot being assembled
  void ResetStream()
  {
    if (rx_slot_ != SLOT_NONE)
    {
      cmd_pool_.Release(rx_slot_);
      rx_slot_ = SLOT_NONE;
    }
    rx_off_ = 0u;
    known_total_ = 0u;
    pad_base_ = 0u;
    pad_skip_ = 0u;
  }

  /**
   * @brief 消费一个收到的包（或几个包）/ Consume one received chunk
   * @return false：命令池没有空槽，什么都没动，槽要留着 / false: no free command slot,
   *         nothing was touched and the slot must be kept
   * @note 仅 owner。拷进命令槽后立即交回槽。主机传输的结束：短包、零长度包或收满 DAP
   *       包长；整包结尾的命令按自己的长度判断（OpenOCD 在整倍数长度时不补零长度包）。
   *       上一条命令以整包结束后紧接着的全零整包是补零，整段丢弃，不需要命令槽。
   *       DAP_TransferAbort 收到即置标志，不进命令池。
   *       Owner only. The slot is handed back right after the copy. The host transfer
   *       ends with a short packet, a zero-length packet or a full DAP packet; a command
   *       ending on a whole packet is judged by its own length (OpenOCD sends no
   *       zero-length packet for multiples). An all-zero whole packet right after a
   *       command that ended on a whole packet is padding, dropped as a whole without a
   *       command slot. DAP_TransferAbort sets the flag on reception and does not join
   *       the command pool.
   */
  bool ConsumePacket(const Endpoint::Slot& slot, uint16_t n, bool in_isr)
  {
    const uint16_t mps = OutChunkSize();
    const uint16_t packet = GetDapPacketSize();
    const uint8_t* data = static_cast<const uint8_t*>(slot.data.addr_);

    if (pad_skip_ > 0u && n <= pad_skip_)
    {
      pad_skip_ = static_cast<uint16_t>(pad_skip_ - n);
      (void)data_out_->ReleaseSlot(slot);
      return true;
    }
    if (rx_off_ == 0u && pad_skip_ == 0u && pad_base_ != 0u && n == mps &&
        IsAllZero(data, n))
    {
      const uint16_t used = static_cast<uint16_t>(pad_base_ + n);
      pad_skip_ = used < packet ? static_cast<uint16_t>(packet - used) : 0u;
      pad_base_ = 0u;
      (void)data_out_->ReleaseSlot(slot);
      return true;
    }

    // 从这里起字节要进命令槽；没有空槽就什么都不动。
    // From here the bytes go into a command slot; with no free one nothing is touched.
    if (rx_slot_ == SLOT_NONE)
    {
      if (cmd_pool_.Acquire(rx_slot_) != ErrorCode::OK)
      {
        rx_slot_ = SLOT_NONE;
        return false;
      }
      rx_off_ = 0u;
      known_total_ = 0u;
    }
    if (pad_skip_ > 0u)
    {
      // 补零之后紧跟着下一条命令的字节。
      // The padding is followed by the bytes of the next command.
      data += pad_skip_;
      n = static_cast<uint16_t>(n - pad_skip_);
      pad_skip_ = 0u;
    }
    pad_base_ = 0u;

    CommandSlot& command = cmd_pool_.At(rx_slot_);
    if (static_cast<size_t>(rx_off_) + n > MAX_DAP_PACKET_SIZE)
    {
      // 超出命令槽：这条命令无法保留，丢弃并重新开始。
      // Beyond the command slot: the command cannot be kept; drop it and start over.
      (void)data_out_->ReleaseSlot(slot);
      ResetStream();
      return true;
    }
    Memory::FastCopy(command.data + rx_off_, data, n);
    rx_off_ = static_cast<uint16_t>(rx_off_ + n);
    (void)data_out_->ReleaseSlot(slot);

    bool done = (n == 0u) || (n % mps != 0u) || (rx_off_ >= packet);
    bool padded_candidate = false;
    if (!done)
    {
      uint16_t length = 0u;
      const CommandScan scan = ScanCommand(command.data, rx_off_, length);
      if (scan == CommandScan::COMPLETE && length <= rx_off_)
      {
        done = true;
        padded_candidate = true;
      }
      else if (scan == CommandScan::COMPLETE)
      {
        known_total_ = std::min<uint16_t>(length, packet);
      }
      else if (scan == CommandScan::NEED_MORE)
      {
        known_total_ = 0u;
      }
      else
      {
        // 长度未知（JTAG、厂商命令）：按包长收，靠短包或收满结束。
        // Length unknown (JTAG, vendor commands): receive up to the packet size and
        // rely on a short packet or a full buffer.
        known_total_ = packet;
      }
    }
    if (!done)
    {
      return true;
    }

    if (rx_off_ == 0u)
    {
      // 只有零长度包的空传输不是命令。
      // An empty transfer (a zero-length packet only) is no command.
      cmd_pool_.Release(rx_slot_);
    }
    else if (command.data[0] == ToU8(LibXR::USB::DapLinkV2Def::CommandId::TRANSFER_ABORT))
    {
      SetTransferAbortFlag(true);
      cmd_pool_.Release(rx_slot_);
    }
    else
    {
      command.len = rx_off_;
      command.epoch = epoch_.load(std::memory_order_relaxed);
      cmd_pool_.Publish(rx_slot_);
      cmd_sem_.PostFromCallback(in_isr);
    }
    rx_slot_ = SLOT_NONE;
    pad_base_ = padded_candidate ? rx_off_ : 0u;
    rx_off_ = 0u;
    known_total_ = 0u;
    return true;
  }

  /// 命令槽放出后按到达顺序处理留住的包；仅 owner / After a command slot came back,
  /// handle the held packets in arrival order; owner only
  void DrainHeldPackets(bool in_isr)
  {
    while (held_n_ > 0u)
    {
      const HeldPacket& held = held_[held_head_];
      if (!ConsumePacket(held.slot, held.size, in_isr))
      {
        return;
      }
      held_head_ = RingAt(held_head_, 1u, HELD_RING_SIZE);
      --held_n_;
    }
  }

  /// 每个空闲槽都挂上接收，大小见 PlanNextSize；仅 owner / Arm a receive on every free
  /// slot, sizes from PlanNextSize; owner only
  void TopUpReceives(bool in_isr)
  {
    while (true)
    {
      const size_t size = PlanNextSize();
      if (data_out_->Receive(size, in_isr) != ErrorCode::OK)
      {
        break;
      }
      ASSERT(armed_n_ < ARMED_RING_SIZE);
      armed_sz_[RingAt(armed_head_, armed_n_, ARMED_RING_SIZE)] = size;
      ++armed_n_;
    }
  }

  /**
   * @brief 下一个接收请求的大小 / Size of the next receive request
   * @note 已确定属于当前传输的字节（补零的剩余、已知长度命令的剩余）一次要完，扣掉已
   *       排队请求会收走的；否则一个端点包，收到后再看内容。仅 owner。
   *       Bytes known to belong to the current transfer (the rest of the padding, the
   *       rest of a command of known length) are asked for at once, minus what the
   *       queued requests will take; otherwise one endpoint packet, and the content
   *       decides after it arrives. Owner only.
   */
  size_t PlanNextSize() const
  {
    const size_t mps = OutChunkSize();
    size_t cap = data_out_->MaxTransferSize();
    cap -= cap % mps;
    if (cap < mps) cap = mps;
    size_t committed = 0u;
    for (uint8_t i = 0u; i < armed_n_; ++i)
    {
      committed += armed_sz_[RingAt(armed_head_, i, ARMED_RING_SIZE)];
    }
    size_t want = 0u;
    if (pad_skip_ > committed)
    {
      want = pad_skip_ - committed;
    }
    else if (rx_off_ > 0u && known_total_ > rx_off_ + committed)
    {
      want = known_total_ - rx_off_ - committed;
    }
    if (want == 0u)
    {
      return mps;
    }
    want = ((want + mps - 1u) / mps) * mps;
    return std::min(want, cap);
  }

  /// Step 线程：执行手里的命令，响应写进 IN 槽或暂存槽 / Step thread: execute the
  /// command in hand into the IN slot or the staging slot
  void ExecuteCurrent()
  {
    CommandSlot& command = cmd_pool_.At(cur_cmd_);
    uint8_t* resp = nullptr;
    uint16_t cap = RESP_SLOT_SIZE;
    if (cur_in_.Valid())
    {
      resp = static_cast<uint8_t*>(cur_in_.data.addr_);
      cap = static_cast<uint16_t>(std::min<size_t>(cur_in_.data.size_, RESP_SLOT_SIZE));
    }
    else
    {
      resp = resp_pool_.At(cur_resp_).payload;
    }
    uint16_t size = 0;
    const auto result = ProcessOneCommand(command.data, command.len, resp, cap, size);
    UNUSED(result);
    if (size == 0u)
    {
      // 没有响应的命令：放出两个槽，owner 可以接着处理留住的包。
      // A command without a response: both slots go back and the owner can go on with
      // the held packets.
      AbandonCurrent();
      RequestService(false);
      return;
    }
    ASSERT(size <= cap);
    CompleteResponse(size);
  }

  /// Step 线程：这条命令收到以来没有解绑或重新绑定 / Step thread: no unbind or bind since
  /// this command was received
  bool StepStillBound() const
  {
    return cur_epoch_ == epoch_.load(std::memory_order_acquire);
  }

  /**
   * @brief Step 线程：epoch 变了就关闭 SWD/JTAG 并恢复会话默认值 / Step thread: on a
   *        changed epoch close SWD/JTAG and restore the session defaults
   * @note 绑定和解绑只发布 epoch，不碰这些状态。
   *       Bind and unbind only publish the epoch and never touch this state.
   */
  void SyncSession(uint32_t epoch)
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

    dap_state_ = {};
    dap_state_.debug_port = LibXR::USB::DapLinkV2Def::DebugPort::DISABLED;
    transfer_abort_.store(false, std::memory_order_release);

    swj_clock_hz_ = 1'000'000u;
    (void)swd_.SetClockHz(swj_clock_hz_);
    if constexpr (JTAG_ENABLED)
    {
      if (jtag_ != nullptr)
      {
        (void)jtag_->SetClockHz(swj_clock_hz_);
      }
    }

    // SWJ 影子默认值：SWDIO=1，nRESET=1，SWCLK=0。
    // SWJ shadow defaults: SWDIO=1, nRESET=1, SWCLK=0.
    last_nreset_level_high_ = true;
    swj_shadow_ = static_cast<uint8_t>(LibXR::USB::DapLinkV2Def::DAP_SWJ_SWDIO_TMS |
                                       LibXR::USB::DapLinkV2Def::DAP_SWJ_NRESET);
    ResetJtagChainState();
  }

  /// Step 线程：放出持有的命令槽和响应缓冲，不发布 / Step thread: give back the command
  /// slot and the response buffer it holds, without publishing
  void AbandonCurrent()
  {
    if (cur_in_.Valid())
    {
      (void)cur_in_ep_->ReleaseSlot(cur_in_);
      cur_in_ = {};
      cur_in_ep_ = nullptr;
    }
    if (cur_resp_ != SLOT_NONE)
    {
      resp_pool_.Release(cur_resp_);
      cur_resp_ = SLOT_NONE;
    }
    if (cur_cmd_ != SLOT_NONE)
    {
      cmd_pool_.Release(cur_cmd_);
      cur_cmd_ = SLOT_NONE;
    }
  }

  /**
   * @brief Step 线程：发布响应，放出命令槽 / Step thread: publish the response and give
   *        back the command slot
   * @note 写在 IN 槽里的响应先比较 epoch（见 TakeInSlot）再由本线程提交；暂存的响应
   *       带着命令的 epoch 发布，是否属于当前会话由 owner 发送时判断，因为本线程的检查
   *       与发布之间仍可能发生解绑。最后请 owner 处理留住的包、补挂接收。
   *       A response in an IN slot is checked against the epoch (see TakeInSlot) and
   *       then submitted by this thread; a staged one is published with the command's
   *       epoch and the owner decides when it sends whether it still belongs to the
   *       current session, because an unbind could still come between a check here and
   *       the publication. Last the owner is asked to handle held packets and re-arm.
   */
  void CompleteResponse(uint16_t size)
  {
    if (cur_in_.Valid())
    {
      if (!StepStillBound())
      {
        AbandonCurrent();
        return;
      }
      SubmitInSlot(size);
    }
    else
    {
      ResponseSlot& response = resp_pool_.At(cur_resp_);
      response.len = size;
      response.epoch = cur_epoch_;
      staged_outstanding_.fetch_add(1u, std::memory_order_acq_rel);
      resp_pool_.Publish(cur_resp_);
      cur_resp_ = SLOT_NONE;
    }
    cmd_pool_.Release(cur_cmd_);
    cur_cmd_ = SLOT_NONE;
    RequestService(false);
  }

 private:
  /**
   * @brief 获取有效的 DAP 包长 / Get effective DAP packet size
   */
  uint16_t GetDapPacketSize() const
  {
    return dap_packet_size_.load(std::memory_order_acquire);
  }

  /**
   * @brief 按 DAP 包长和缓冲容量截短响应长度 / Clip response length by DAP packet size
   *        and buffer capacity
   */
  uint16_t ClipResponseLength(uint16_t len, uint16_t cap) const
  {
    const uint16_t DAP_PS = GetDapPacketSize();
    if (len > DAP_PS)
    {
      len = DAP_PS;
    }
    if (len > RESP_SLOT_SIZE)
    {
      len = RESP_SLOT_SIZE;
    }
    if (len > cap)
    {
      len = cap;
    }
    return len;
  }

  /**
   * @brief 从打包的队列流中解析一条排队命令的长度 / Parse one queued command length from
   *        a packed queue stream
   * @param req 请求指针 / request pointer
   * @param avail 队列流中的可用字节数 / available bytes in the queue stream
   * @param cmd_len 解析出的命令长度 / parsed command length
   * @return true 解析成功 / true parse success
   * @return false 解析失败 / false parse failure
   */
  bool ParseQueuedCommandLength(const uint8_t* req, uint16_t avail,
                                uint16_t& cmd_len) const
  {
    cmd_len = 0u;
    if (!req || avail < 1u)
    {
      return false;
    }

    const uint8_t CMD = req[0];
    switch (CMD)
    {
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::INFO):
        cmd_len = 2u;
        return avail >= cmd_len;
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::HOST_STATUS):
        cmd_len = 3u;
        return avail >= cmd_len;
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::CONNECT):
        cmd_len = 2u;
        return avail >= cmd_len;
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::DISCONNECT):
        cmd_len = 1u;
        return avail >= cmd_len;
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::TRANSFER_CONFIGURE):
        cmd_len = 6u;
        return avail >= cmd_len;
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::TRANSFER_ABORT):
        cmd_len = 1u;
        return avail >= cmd_len;
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::WRITE_ABORT):
        cmd_len = 6u;
        return avail >= cmd_len;
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::DELAY):
        cmd_len = 3u;
        return avail >= cmd_len;
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::RESET_TARGET):
        cmd_len = 1u;
        return avail >= cmd_len;
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::SWJ_PINS):
        cmd_len = 7u;
        return avail >= cmd_len;
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::SWJ_CLOCK):
        cmd_len = 5u;
        return avail >= cmd_len;
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::SWD_CONFIGURE):
        cmd_len = 2u;
        return avail >= cmd_len;
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::SWJ_SEQUENCE):
      {
        if (avail < 2u)
        {
          return false;
        }
        const uint32_t BIT_COUNT = (req[1] == 0u) ? 256u : static_cast<uint32_t>(req[1]);
        const uint32_t BYTE_COUNT = (BIT_COUNT + 7u) / 8u;
        const uint32_t NEED = 2u + BYTE_COUNT;
        if (NEED > avail)
        {
          return false;
        }
        cmd_len = static_cast<uint16_t>(NEED);
        return true;
      }
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::SWD_SEQUENCE):
      {
        if (avail < 2u)
        {
          return false;
        }
        const uint8_t SEQ_CNT = req[1];
        uint32_t off = 2u;
        for (uint32_t i = 0u; i < SEQ_CNT; ++i)
        {
          if (off >= avail)
          {
            return false;
          }

          const uint8_t INFO = req[off++];
          uint32_t cycles = static_cast<uint32_t>(INFO & 0x3Fu);
          if (cycles == 0u)
          {
            cycles = 64u;
          }

          const bool MODE_IN = ((INFO & 0x80u) != 0u);
          const uint32_t BYTES = (cycles + 7u) / 8u;
          if (!MODE_IN)
          {
            if (off + BYTES > avail)
            {
              return false;
            }
            off += BYTES;
          }
        }
        cmd_len = static_cast<uint16_t>(off);
        return true;
      }
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::TRANSFER):
      {
        if (avail < 3u)
        {
          return false;
        }
        const uint8_t COUNT = req[2];
        uint32_t off = 3u;
        for (uint32_t i = 0u; i < COUNT; ++i)
        {
          if (off >= avail)
          {
            return false;
          }

          const uint8_t RQ = req[off++];
          const bool RNW = LibXR::USB::DapLinkV2Def::req_is_read(RQ);
          const bool MATCH_VALUE =
              ((RQ & LibXR::USB::DapLinkV2Def::DAP_TRANSFER_MATCH_VALUE) != 0u);
          if (!RNW || MATCH_VALUE)
          {
            if (off + 4u > avail)
            {
              return false;
            }
            off += 4u;
          }
        }
        cmd_len = static_cast<uint16_t>(off);
        return true;
      }
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::TRANSFER_BLOCK):
      {
        if (avail < 5u)
        {
          return false;
        }
        uint16_t count = 0u;
        Memory::FastCopy(&count, &req[2], sizeof(count));
        const bool RNW = LibXR::USB::DapLinkV2Def::req_is_read(req[4]);
        uint32_t need = 5u;
        if (!RNW)
        {
          need += static_cast<uint32_t>(count) * 4u;
        }
        if (need > avail)
        {
          return false;
        }
        cmd_len = static_cast<uint16_t>(need);
        return true;
      }

      // 拒绝队列流中嵌套的 queue/execute / Reject nested queue/execute in queued stream.
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::QUEUE_COMMANDS):
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::EXECUTE_COMMANDS):
      default:
        return false;
    }
  }

  /**
   * @brief 依次执行请求里的 count 条命令，响应拼成 [0x7F, count, 各条响应...]
   *        Run the count commands of the request one after another; the response is
   *        [0x7F, count, each command's response...]
   *
   * 命令就地读自请求（请求在执行期间一直有效），延时类命令在本线程里等到完成。
   * The commands are read in place from the request (it stays valid while they run);
   * delay-type commands wait in this thread until they are done.
   *
   * @param cmds     第一条命令 / First command
   * @param cmds_len 命令部分的长度，最后一条命令之后的字节忽略 / Length of the command
   *                 part; bytes after the last command are ignored
   * @param count    命令数 / Number of commands
   * @param resp     响应缓冲 / Response buffer
   * @param resp_cap 响应缓冲容量 / Response buffer capacity
   * @param out_len  响应长度 / Response length
   * @return false 表示格式或响应空间不对 / false when the format or the response space
   *         does not fit
   */
  bool TryExecuteQueuedCommandStream(const uint8_t* cmds, uint16_t cmds_len,
                                     uint8_t count, uint8_t* resp, uint16_t resp_cap,
                                     uint16_t& out_len)
  {
    out_len = 0u;
    if (!resp || resp_cap < 2u)
    {
      return false;
    }
    resp[0] = ToU8(LibXR::USB::DapLinkV2Def::CommandId::EXECUTE_COMMANDS);
    resp[1] = count;
    out_len = 2u;

    uint16_t req_off = 0u;
    uint16_t resp_off = 2u;
    for (uint32_t i = 0u; i < count; ++i)
    {
      if (req_off >= cmds_len)
      {
        return false;
      }

      uint16_t cmd_len = 0u;
      if (!ParseQueuedCommandLength(&cmds[req_off],
                                    static_cast<uint16_t>(cmds_len - req_off), cmd_len) ||
          cmd_len == 0u)
      {
        return false;
      }

      if (resp_off >= resp_cap)
      {
        return false;
      }

      uint16_t cmd_out = 0u;
      const ErrorCode ans =
          ProcessOneCommand(&cmds[req_off], cmd_len, &resp[resp_off],
                            static_cast<uint16_t>(resp_cap - resp_off), cmd_out);
      UNUSED(ans);

      // 每条命令至少要有 [cmd, status] 两个字节；0 字节会破坏响应流对齐。
      // Every command answers with at least [cmd, status]; zero bytes would
      // desynchronise the response stream.
      if (cmd_out == 0u)
      {
        return false;
      }

      if (static_cast<uint32_t>(resp_off) + static_cast<uint32_t>(cmd_out) >
          static_cast<uint32_t>(resp_cap))
      {
        return false;
      }

      req_off = static_cast<uint16_t>(req_off + cmd_len);
      resp_off = static_cast<uint16_t>(resp_off + cmd_out);
    }

    // 最后一条命令之后的字节（补零）忽略。 / Bytes after the last command (padding) are
    // ignored.
    out_len = resp_off;
    return true;
  }

  /// 一个端点包的大小，不超过 DAP 包长；仅 owner，已绑定时 / One endpoint packet, not
  /// above the DAP packet size; owner only, while bound
  uint16_t OutChunkSize() const
  {
    const uint16_t mps = data_out_->MaxPacketSize();
    const uint16_t packet = GetDapPacketSize();
    return mps > packet ? packet : mps;
  }

  /// 长 size 的环里从 head 起第 offset 项 / Entry offset after head in a ring of size
  static uint8_t RingAt(uint8_t head, uint8_t offset, uint8_t size)
  {
    return static_cast<uint8_t>((head + offset) % size);
  }

  static bool IsAllZero(const uint8_t* data, size_t size)
  {
    for (size_t i = 0u; i < size; ++i)
    {
      if (data[i] != 0u) return false;
    }
    return true;
  }

  enum class CommandScan : uint8_t
  {
    COMPLETE,   ///< 长度已知且已收齐 / Length known and fully received
    NEED_MORE,  ///< 命令已识别但还没收齐 / Recognised but not fully received
    UNKNOWN,    ///< 无法由内容确定长度 / Length cannot be derived from the content
  };

  /// 已识别的、可由内容确定长度的命令 / Commands whose length follows from the content
  static bool IsScannableCommand(uint8_t id)
  {
    switch (id)
    {
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::INFO):
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::HOST_STATUS):
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::CONNECT):
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::DISCONNECT):
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::TRANSFER_CONFIGURE):
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::TRANSFER_ABORT):
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::WRITE_ABORT):
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::DELAY):
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::RESET_TARGET):
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::SWJ_PINS):
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::SWJ_CLOCK):
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::SWD_CONFIGURE):
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::SWJ_SEQUENCE):
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::SWD_SEQUENCE):
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::TRANSFER):
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::TRANSFER_BLOCK):
        return true;
      default:
        return false;
    }
  }

  /**
   * @brief 按命令自己的内容判断已收的 avail 字节是否构成一条完整命令。
   *        Decide from the command's own content whether the received avail bytes make
   *        one whole command.
   */
  CommandScan ScanCommand(const uint8_t* req, uint16_t avail, uint16_t& length) const
  {
    length = 0u;
    if (avail < 1u) return CommandScan::NEED_MORE;
    const uint8_t id = req[0];
    if (id == ToU8(LibXR::USB::DapLinkV2Def::CommandId::QUEUE_COMMANDS) ||
        id == ToU8(LibXR::USB::DapLinkV2Def::CommandId::EXECUTE_COMMANDS))
    {
      // [ID, 命令数, 命令...]：逐条累加子命令长度。
      // [ID, count, commands...]: add up the lengths of the sub-commands.
      if (avail < 2u) return CommandScan::NEED_MORE;
      uint16_t off = 2u;
      for (uint8_t i = 0u; i < req[1]; ++i)
      {
        if (off >= avail) return CommandScan::NEED_MORE;
        if (!IsScannableCommand(req[off])) return CommandScan::UNKNOWN;
        uint16_t sub = 0u;
        if (!ParseQueuedCommandLength(req + off, static_cast<uint16_t>(avail - off), sub))
          return CommandScan::NEED_MORE;
        off = static_cast<uint16_t>(off + sub);
      }
      length = off;
      return CommandScan::COMPLETE;
    }
    if (!IsScannableCommand(id)) return CommandScan::UNKNOWN;
    return ParseQueuedCommandLength(req, avail, length) ? CommandScan::COMPLETE
                                                        : CommandScan::NEED_MORE;
  }

 private:
  /**
   * @brief 处理一条命令 / Process one command
   *
   * @param req      请求缓冲 / Request buffer
   * @param req_len  请求长度 / Request length
   * @param resp     响应缓冲 / Response buffer
   * @param resp_cap 响应容量 / Response capacity
   * @param out_len  输出响应长度 / Output response length
   * @return 错误码 / Error code
   */
  ErrorCode ProcessOneCommand(const uint8_t* req, uint16_t req_len, uint8_t* resp,
                              uint16_t resp_cap, uint16_t& out_len)
  {
    out_len = 0;

    if (!req || !resp || req_len < 1u || resp_cap < 1u)
    {
      if (resp && resp_cap >= 1u)
      {
        BuildNotSupportResponse(resp, resp_cap, out_len);
      }
      return ErrorCode::ARG_ERR;
    }

    if (command_hook_ != nullptr &&
        command_hook_(command_hook_ctx_, req, req_len, resp, resp_cap, out_len))
    {
      return ErrorCode::OK;
    }

    const uint8_t CMD = req[0];

    switch (CMD)
    {
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::INFO):
        return HandleInfo(req, req_len, resp, resp_cap, out_len);
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::HOST_STATUS):
        return HandleHostStatus(req, req_len, resp, resp_cap, out_len);
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::CONNECT):
        return HandleConnect(req, req_len, resp, resp_cap, out_len);
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::DISCONNECT):
        return HandleDisconnect(req, req_len, resp, resp_cap, out_len);
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::TRANSFER_CONFIGURE):
        return HandleTransferConfigure(req, req_len, resp, resp_cap, out_len);
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::TRANSFER):
        return HandleTransfer(req, req_len, resp, resp_cap, out_len);
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::TRANSFER_BLOCK):
        return HandleTransferBlock(req, req_len, resp, resp_cap, out_len);
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::TRANSFER_ABORT):
        return HandleTransferAbort(req, req_len, resp, resp_cap, out_len);
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::WRITE_ABORT):
        return HandleWriteABORT(req, req_len, resp, resp_cap, out_len);
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::DELAY):
        return HandleDelay(req, req_len, resp, resp_cap, out_len);
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::RESET_TARGET):
        return HandleResetTarget(req, req_len, resp, resp_cap, out_len);

      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::SWJ_PINS):
        return HandleSWJPins(req, req_len, resp, resp_cap, out_len);
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::SWJ_CLOCK):
        return HandleSWJClock(req, req_len, resp, resp_cap, out_len);
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::SWJ_SEQUENCE):
        return HandleSWJSequence(req, req_len, resp, resp_cap, out_len);
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::SWD_CONFIGURE):
        return HandleSWDConfigure(req, req_len, resp, resp_cap, out_len);
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::SWD_SEQUENCE):
        return HandleSWDSequence(req, req_len, resp, resp_cap, out_len);

      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::JTAG_SEQUENCE):
        if constexpr (JTAG_ENABLED)
        {
          return HandleJTAGSequence(req, req_len, resp, resp_cap, out_len);
        }
        (void)BuildUnknownCmdResponse(resp, resp_cap, out_len);
        return ErrorCode::NOT_SUPPORT;
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::JTAG_CONFIGURE):
        if constexpr (JTAG_ENABLED)
        {
          return HandleJTAGConfigure(req, req_len, resp, resp_cap, out_len);
        }
        (void)BuildUnknownCmdResponse(resp, resp_cap, out_len);
        return ErrorCode::NOT_SUPPORT;
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::JTAG_IDCODE):
        if constexpr (JTAG_ENABLED)
        {
          return HandleJTAGIdCode(req, req_len, resp, resp_cap, out_len);
        }
        (void)BuildUnknownCmdResponse(resp, resp_cap, out_len);
        return ErrorCode::NOT_SUPPORT;

      // CMSIS-DAP 参考实现把 DAP_QueueCommands 包改写成 DAP_ExecuteCommands 执行，
      // 响应也是 DAP_ExecuteCommands 的格式；每个包一条响应，按包的顺序发出。
      // The CMSIS-DAP reference implementation rewrites a DAP_QueueCommands packet into
      // DAP_ExecuteCommands and runs it, so the response has the DAP_ExecuteCommands
      // format; every packet gets one response, in packet order.
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::QUEUE_COMMANDS):
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::EXECUTE_COMMANDS):
        return HandleExecuteCommands(req, req_len, resp, resp_cap, out_len);

      default:
        (void)BuildUnknownCmdResponse(resp, resp_cap, out_len);
        return ErrorCode::NOT_SUPPORT;
    }
  }

  /**
   * @brief 构造 NOT_SUPPORT 响应 / Build NOT_SUPPORT response
   *
   * @param resp     响应缓冲 / Response buffer
   * @param resp_cap 响应容量 / Response capacity
   * @param out_len  输出响应长度 / Output response length
   */
  void BuildNotSupportResponse(uint8_t* resp, uint16_t resp_cap, uint16_t& out_len)
  {
    if (!resp || resp_cap < 1u)
    {
      out_len = 0u;
      return;
    }
    resp[0] = 0xFFu;
    out_len = 1u;
  }

 private:
  ErrorCode HandleInfo(const uint8_t* req, uint16_t req_len, uint8_t* resp,
                       uint16_t resp_cap, uint16_t& out_len)
  {
    if (req_len < 2u)
    {
      resp[0] = ToU8(LibXR::USB::DapLinkV2Def::CommandId::INFO);
      resp[1] = 0u;
      out_len = 2u;
      return ErrorCode::ARG_ERR;
    }

    const uint8_t INFO_ID = req[1];

    resp[0] = ToU8(LibXR::USB::DapLinkV2Def::CommandId::INFO);

    switch (INFO_ID)
    {
      case ToU8(LibXR::USB::DapLinkV2Def::InfoId::VENDOR):
        return BuildInfoStringResponse(resp[0], info_.vendor, resp, resp_cap, out_len);
      case ToU8(LibXR::USB::DapLinkV2Def::InfoId::PRODUCT):
        return BuildInfoStringResponse(resp[0], info_.product, resp, resp_cap, out_len);
      case ToU8(LibXR::USB::DapLinkV2Def::InfoId::SERIAL_NUMBER):
        return BuildInfoStringResponse(resp[0], info_.serial, resp, resp_cap, out_len);
      case ToU8(LibXR::USB::DapLinkV2Def::InfoId::FIRMWARE_VERSION):
        return BuildInfoStringResponse(resp[0], info_.firmware_ver, resp, resp_cap,
                                       out_len);

      case ToU8(LibXR::USB::DapLinkV2Def::InfoId::DEVICE_VENDOR):
        return BuildInfoStringResponse(resp[0], info_.device_vendor, resp, resp_cap,
                                       out_len);
      case ToU8(LibXR::USB::DapLinkV2Def::InfoId::DEVICE_NAME):
        return BuildInfoStringResponse(resp[0], info_.device_name, resp, resp_cap,
                                       out_len);
      case ToU8(LibXR::USB::DapLinkV2Def::InfoId::BOARD_VENDOR):
        return BuildInfoStringResponse(resp[0], info_.board_vendor, resp, resp_cap,
                                       out_len);
      case ToU8(LibXR::USB::DapLinkV2Def::InfoId::BOARD_NAME):
        return BuildInfoStringResponse(resp[0], info_.board_name, resp, resp_cap,
                                       out_len);
      case ToU8(LibXR::USB::DapLinkV2Def::InfoId::PRODUCT_FIRMWARE_VERSION):
        return BuildInfoStringResponse(resp[0], info_.product_fw_ver, resp, resp_cap,
                                       out_len);

      case ToU8(LibXR::USB::DapLinkV2Def::InfoId::CAPABILITIES):
      {
        uint8_t caps = LibXR::USB::DapLinkV2Def::DAP_CAP_SWD;
        if constexpr (JTAG_ENABLED)
        {
          if (jtag_ != nullptr)
          {
            caps = static_cast<uint8_t>(caps | LibXR::USB::DapLinkV2Def::DAP_CAP_JTAG);
          }
        }
        return BuildInfoU8Response(resp[0], caps, resp, resp_cap, out_len);
      }
      case ToU8(LibXR::USB::DapLinkV2Def::InfoId::PACKET_COUNT):
        return BuildInfoU8Response(resp[0], PACKET_COUNT_EFFECTIVE, resp, resp_cap,
                                   out_len);
      case ToU8(LibXR::USB::DapLinkV2Def::InfoId::PACKET_SIZE):
      {
        const uint16_t DAP_PS = GetDapPacketSize();
        return BuildInfoU16Response(resp[0], DAP_PS, resp, resp_cap, out_len);
      }
      case ToU8(LibXR::USB::DapLinkV2Def::InfoId::TIMESTAMP_CLOCK):
        return BuildInfoU32Response(resp[0], 1000000U, resp, resp_cap, out_len);

      default:
        resp[1] = 0u;
        out_len = 2u;
        return ErrorCode::OK;
    }
  }

  ErrorCode BuildInfoStringResponse(uint8_t cmd, const char* str, uint8_t* resp,
                                    uint16_t resp_cap, uint16_t& out_len)
  {
    resp[0] = cmd;
    resp[1] = 0u;

    if (!str)
    {
      out_len = 2u;
      return ErrorCode::OK;
    }

    const size_t N_WITH_NUL = std::strlen(str) + 1;  // include '\0'
    const size_t MAX_PAYLOAD = (resp_cap >= 2u) ? (resp_cap - 2u) : 0u;
    if (MAX_PAYLOAD == 0u)
    {
      out_len = 2u;
      return ErrorCode::OK;
    }

    const size_t COPY_N = (N_WITH_NUL > MAX_PAYLOAD) ? MAX_PAYLOAD : N_WITH_NUL;
    Memory::FastCopy(&resp[2], str, COPY_N);

    // 即使被截断也保证以结束符收尾 / Ensure termination even when truncated
    resp[2 + COPY_N - 1] = 0x00;

    resp[1] = static_cast<uint8_t>(COPY_N);
    out_len = static_cast<uint16_t>(2u + COPY_N);
    return ErrorCode::OK;
  }

  ErrorCode BuildInfoU8Response(uint8_t cmd, uint8_t val, uint8_t* resp,
                                uint16_t resp_cap, uint16_t& out_len)
  {
    if (resp_cap < 3u)
    {
      out_len = 0;
      return ErrorCode::NOT_FOUND;
    }
    resp[0] = cmd;
    resp[1] = 1u;
    resp[2] = val;
    out_len = 3u;
    return ErrorCode::OK;
  }

  ErrorCode BuildInfoU16Response(uint8_t cmd, uint16_t val, uint8_t* resp,
                                 uint16_t resp_cap, uint16_t& out_len)
  {
    if (resp_cap < 4u)
    {
      out_len = 0;
      return ErrorCode::NOT_FOUND;
    }
    resp[0] = cmd;
    resp[1] = 2u;

    // 仅限小端设备：直接 memcpy 到载荷（不受对齐影响）。
    // Little-endian device only: direct memcpy to payload (alignment-safe).
    Memory::FastCopy(&resp[2], &val, sizeof(val));

    out_len = 4u;
    return ErrorCode::OK;
  }

  ErrorCode BuildInfoU32Response(uint8_t cmd, uint32_t val, uint8_t* resp,
                                 uint16_t resp_cap, uint16_t& out_len)
  {
    if (resp_cap < 6u)
    {
      out_len = 0;
      return ErrorCode::NOT_FOUND;
    }
    resp[0] = cmd;
    resp[1] = 4u;

    // 仅限小端设备：直接 memcpy 到载荷（不受对齐影响）。
    // Little-endian device only: direct memcpy to payload (alignment-safe).
    Memory::FastCopy(&resp[2], &val, sizeof(val));

    out_len = 6u;
    return ErrorCode::OK;
  }

 private:
  ErrorCode HandleHostStatus(const uint8_t* /*req*/, uint16_t /*req_len*/, uint8_t* resp,
                             uint16_t resp_cap, uint16_t& out_len)
  {
    if (resp_cap < 2u)
    {
      out_len = 0;
      return ErrorCode::NOT_FOUND;
    }
    resp[0] = ToU8(LibXR::USB::DapLinkV2Def::CommandId::HOST_STATUS);
    resp[1] = ToU8(LibXR::USB::DapLinkV2Def::Status::OK);
    out_len = 2u;
    return ErrorCode::OK;
  }

  ErrorCode HandleConnect(const uint8_t* req, uint16_t req_len, uint8_t* resp,
                          uint16_t resp_cap, uint16_t& out_len)
  {
    if (resp_cap < 2u)
    {
      out_len = 0;
      return ErrorCode::NOT_FOUND;
    }

    resp[0] = ToU8(LibXR::USB::DapLinkV2Def::CommandId::CONNECT);

    uint8_t port = 0u;
    if (req_len >= 2u)
    {
      port = req[1];
    }

    if (port == 0u || port == ToU8(LibXR::USB::DapLinkV2Def::Port::SWD))
    {
      (void)swd_.EnterSwd();
      (void)swd_.SetClockHz(swj_clock_hz_);

      dap_state_.debug_port = LibXR::USB::DapLinkV2Def::DebugPort::SWD;
      transfer_abort_.store(false, std::memory_order_release);

      resp[1] = ToU8(LibXR::USB::DapLinkV2Def::Port::SWD);
    }
    else if (port == ToU8(LibXR::USB::DapLinkV2Def::Port::JTAG))
    {
      if constexpr (!JTAG_ENABLED)
      {
        resp[1] = ToU8(LibXR::USB::DapLinkV2Def::Port::DISABLED);
      }
      else
      {
        if (jtag_ == nullptr)
        {
          resp[1] = ToU8(LibXR::USB::DapLinkV2Def::Port::DISABLED);
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

          dap_state_.debug_port = LibXR::USB::DapLinkV2Def::DebugPort::JTAG;
          transfer_abort_.store(false, std::memory_order_release);
          ResetJtagChainState();
          resp[1] = ToU8(LibXR::USB::DapLinkV2Def::Port::JTAG);
        }
      }
    }
    else
    {
      resp[1] = ToU8(LibXR::USB::DapLinkV2Def::Port::DISABLED);
    }

    out_len = 2u;
    return ErrorCode::OK;
  }

  ErrorCode HandleDisconnect(const uint8_t* /*req*/, uint16_t /*req_len*/, uint8_t* resp,
                             uint16_t resp_cap, uint16_t& out_len)
  {
    if (resp_cap < 2u)
    {
      out_len = 0;
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
    dap_state_.debug_port = LibXR::USB::DapLinkV2Def::DebugPort::DISABLED;
    transfer_abort_.store(false, std::memory_order_release);

    resp[0] = ToU8(LibXR::USB::DapLinkV2Def::CommandId::DISCONNECT);
    resp[1] = ToU8(LibXR::USB::DapLinkV2Def::Status::OK);
    out_len = 2u;
    return ErrorCode::OK;
  }

  ErrorCode HandleTransferConfigure(const uint8_t* req, uint16_t req_len, uint8_t* resp,
                                    uint16_t resp_cap, uint16_t& out_len)
  {
    if (resp_cap < 2u)
    {
      out_len = 0;
      return ErrorCode::NOT_FOUND;
    }

    resp[0] = ToU8(LibXR::USB::DapLinkV2Def::CommandId::TRANSFER_CONFIGURE);

    // Req: [0]=0x04 [1]=idle_cycles [2..3]=wait_retry [4..5]=match_retry
    if (req_len < 6u)
    {
      resp[1] = ToU8(LibXR::USB::DapLinkV2Def::Status::ERROR);
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

    // 映射到 SWD 事务策略 / Map to SWD transaction policy
    Debug::Swd::TransferPolicy pol = swd_.GetTransferPolicy();
    pol.idle_cycles = IDLE;
    pol.wait_retry = wait_retry;
    swd_.SetTransferPolicy(pol);

    resp[1] = ToU8(LibXR::USB::DapLinkV2Def::Status::OK);
    out_len = 2u;
    return ErrorCode::OK;
  }

  ErrorCode HandleTransferAbort(const uint8_t* /*req*/, uint16_t /*req_len*/,
                                uint8_t* /*resp*/, uint16_t /*resp_cap*/,
                                uint16_t& out_len)
  {
    // DAP_TransferAbort 没有响应。USB 路径在接收时就置位标志（见 OnDataOutComplete），
    // 这里服务批命令和 ExecuteExternal 里夹带的中止。
    // DAP_TransferAbort has no response. The USB path sets the flag on reception (see
    // OnDataOutComplete); this serves an abort carried inside a batch or ExecuteExternal.
    SetTransferAbortFlag(true);
    out_len = 0u;
    return ErrorCode::OK;
  }

  ErrorCode HandleWriteABORT(const uint8_t* req, uint16_t req_len, uint8_t* resp,
                             uint16_t resp_cap, uint16_t& out_len)
  {
    if (resp_cap < 2u)
    {
      out_len = 0;
      return ErrorCode::NOT_FOUND;
    }

    if constexpr (JTAG_ENABLED)
    {
      if (dap_state_.debug_port == LibXR::USB::DapLinkV2Def::DebugPort::JTAG)
      {
        return HandleJtagWriteAbort(req, req_len, resp, resp_cap, out_len);
      }
    }

    resp[0] = ToU8(LibXR::USB::DapLinkV2Def::CommandId::WRITE_ABORT);

    if (req_len < 6u)
    {
      resp[1] = ToU8(LibXR::USB::DapLinkV2Def::Status::ERROR);
      out_len = 2u;
      return ErrorCode::ARG_ERR;
    }

    uint32_t flags = 0u;
    Memory::FastCopy(&flags, &req[2], sizeof(flags));

    LibXR::Debug::SwdProtocol::Ack ack = LibXR::Debug::SwdProtocol::Ack::PROTOCOL;

    const ErrorCode EC = swd_.WriteAbortTxn(flags, ack);
    resp[1] = (EC == ErrorCode::OK && ack == LibXR::Debug::SwdProtocol::Ack::OK)
                  ? ToU8(LibXR::USB::DapLinkV2Def::Status::OK)
                  : ToU8(LibXR::USB::DapLinkV2Def::Status::ERROR);

    out_len = 2u;
    return ErrorCode::OK;
  }

  ErrorCode HandleDelay(const uint8_t* req, uint16_t req_len, uint8_t* resp,
                        uint16_t resp_cap, uint16_t& out_len)
  {
    if (resp_cap < 2u)
    {
      out_len = 0;
      return ErrorCode::NOT_FOUND;
    }

    resp[0] = ToU8(LibXR::USB::DapLinkV2Def::CommandId::DELAY);

    if (req_len < 3u)
    {
      resp[1] = ToU8(LibXR::USB::DapLinkV2Def::Status::ERROR);
      out_len = 2u;
      return ErrorCode::ARG_ERR;
    }

    uint16_t us = 0u;
    Memory::FastCopy(&us, &req[1], sizeof(us));

    // 在执行命令的线程里按微秒等待（Step 线程或 ExecuteExternal 的调用者）。
    // Wait the exact microseconds in the thread that executes the command (the Step
    // thread or the caller of ExecuteExternal).
    LibXR::Timebase::DelayMicroseconds(us);

    resp[1] = ToU8(LibXR::USB::DapLinkV2Def::Status::OK);
    out_len = 2u;
    return ErrorCode::OK;
  }

  ErrorCode HandleResetTarget(const uint8_t* /*req*/, uint16_t /*req_len*/, uint8_t* resp,
                              uint16_t resp_cap, uint16_t& out_len)
  {
    if (!resp || resp_cap < 3u)
    {
      out_len = 0u;
      return ErrorCode::NOT_FOUND;
    }

    resp[0] = ToU8(LibXR::USB::DapLinkV2Def::CommandId::RESET_TARGET);

    uint8_t execute = 0u;
    if (nreset_gpio_ != nullptr)
    {
      // nRESET 拉低 1 ms、释放后再等 1 ms，都在执行命令的线程里等。
      // Hold nRESET low for 1 ms and wait 1 ms after releasing it, both in the thread
      // that executes the command.
      DriveReset(false);
      LibXR::Timebase::DelayMicroseconds(1000u);
      DriveReset(true);
      LibXR::Timebase::DelayMicroseconds(1000u);
      execute = 1u;
    }

    // 始终返回 DAP_OK；未实现复位时 Execute=0。
    // Always return DAP_OK; if reset is not implemented, Execute=0.
    resp[1] = DAP_OK;
    resp[2] = execute;
    out_len = 3u;
    return ErrorCode::OK;
  }

 private:
  ErrorCode HandleSWJPins(const uint8_t* req, uint16_t req_len, uint8_t* resp,
                          uint16_t resp_cap, uint16_t& out_len)
  {
    if (!resp || resp_cap < 2u)
    {
      out_len = 0u;
      return ErrorCode::NOT_FOUND;
    }

    resp[0] = ToU8(LibXR::USB::DapLinkV2Def::CommandId::SWJ_PINS);

    // Req: [0]=0x10 [1]=PinOut [2]=PinSelect [3..6]=PinWait(us)
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

    // 0) 将所有选中引脚的请求状态锁存到影子值（即使不受支持）。
    // 0) Latch requested states into shadow for ALL selected pins (even if unsupported)
    swj_shadow_ = static_cast<uint8_t>((swj_shadow_ & static_cast<uint8_t>(~PIN_SEL)) |
                                       (PIN_OUT & PIN_SEL));

    // 1) 仅 nRESET 有物理支持（尽力而为）。
    // 1) Only physically support nRESET (best-effort)
    if ((PIN_SEL & LibXR::USB::DapLinkV2Def::DAP_SWJ_NRESET) != 0u)
    {
      const bool LEVEL_HIGH =
          ((PIN_OUT & LibXR::USB::DapLinkV2Def::DAP_SWJ_NRESET) != 0u);
      // DriveReset 更新 last_nreset_level_high_ 和影子值，存在 GPIO 时同时写 GPIO。
      // DriveReset updates last_nreset_level_high_ and shadow, and writes GPIO if
      // present.
      DriveReset(LEVEL_HIGH);
    }

    // 2) PinWait：在执行命令的线程里等 (PinInput & PinSelect) 匹配
    //    (PinOut & PinSelect) 或超时；shadow-only 的引脚总是立即匹配。
    //    PinWait: in the thread that executes the command, wait until
    //    (PinInput & PinSelect) matches (PinOut & PinSelect) or the timeout; shadow-only
    //    pins always match at once.
    const uint8_t EXPECT = static_cast<uint8_t>(PIN_OUT & PIN_SEL);
    uint8_t pin_in = ReadSwjPins();
    if (wait_us != 0u && PIN_SEL != 0u)
    {
      const uint64_t START = LibXR::Timebase::GetMicroseconds();
      while ((pin_in & PIN_SEL) != EXPECT &&
             (static_cast<uint64_t>(LibXR::Timebase::GetMicroseconds()) - START) <
                 wait_us)
      {
        pin_in = ReadSwjPins();
      }
    }

    resp[1] = pin_in;
    out_len = 2u;
    return ErrorCode::OK;
  }

  /**
   * @brief 读回 SWJ 引脚电平：从 shadow 出发，nRESET 若接了 GPIO 则用实际电平覆盖。
   *        Sample the SWJ pins: start from the shadow and override nRESET with the wired
   *        GPIO level when present.
   */
  uint8_t ReadSwjPins() const
  {
    uint8_t pin_in = swj_shadow_;

    if (nreset_gpio_ != nullptr)
    {
      if (nreset_gpio_->Read())
      {
        pin_in |= LibXR::USB::DapLinkV2Def::DAP_SWJ_NRESET;
      }
      else
      {
        pin_in = static_cast<uint8_t>(
            pin_in & static_cast<uint8_t>(~LibXR::USB::DapLinkV2Def::DAP_SWJ_NRESET));
      }
    }

    return pin_in;
  }

  ErrorCode HandleSWJClock(const uint8_t* req, uint16_t req_len, uint8_t* resp,
                           uint16_t resp_cap, uint16_t& out_len)
  {
    if (resp_cap < 2u)
    {
      out_len = 0;
      return ErrorCode::NOT_FOUND;
    }

    resp[0] = ToU8(LibXR::USB::DapLinkV2Def::CommandId::SWJ_CLOCK);

    if (req_len < 5u)
    {
      resp[1] = ToU8(LibXR::USB::DapLinkV2Def::Status::ERROR);
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

    resp[1] = ToU8(LibXR::USB::DapLinkV2Def::Status::OK);
    out_len = 2u;
    return ErrorCode::OK;
  }

  ErrorCode HandleSWJSequence(const uint8_t* req, uint16_t req_len, uint8_t* resp,
                              uint16_t resp_cap, uint16_t& out_len)
  {
    if (!resp || resp_cap < 2u)
    {
      out_len = 0u;
      return ErrorCode::NOT_FOUND;
    }

    resp[0] = ToU8(LibXR::USB::DapLinkV2Def::CommandId::SWJ_SEQUENCE);
    resp[1] = ToU8(LibXR::USB::DapLinkV2Def::Status::OK);
    out_len = 2u;

    // Req: [0]=0x12 [1]=bit_count(0=>256) [2..]=data (LSB-first)
    if (!req || req_len < 2u)
    {
      resp[1] = ToU8(LibXR::USB::DapLinkV2Def::Status::ERROR);
      return ErrorCode::ARG_ERR;
    }

    const uint8_t RAW_COUNT = req[1];
    const uint32_t BIT_COUNT =
        (RAW_COUNT == 0u) ? 256u : static_cast<uint32_t>(RAW_COUNT);
    const uint32_t BYTE_COUNT = (BIT_COUNT + 7u) / 8u;

    if (2u + BYTE_COUNT > req_len)
    {
      resp[1] = ToU8(LibXR::USB::DapLinkV2Def::Status::ERROR);
      return ErrorCode::ARG_ERR;
    }

    const uint8_t* data = &req[2];

    // 委托给 SwdPort 实现（此处不使用 RawMode，也不做引脚级控制）。
    // Delegate to SwdPort implementation (no RawMode / no pin-level control here)
    const ErrorCode EC = swd_.SeqWriteBits(BIT_COUNT, data);
    if (EC != ErrorCode::OK)
    {
      resp[1] = ToU8(LibXR::USB::DapLinkV2Def::Status::ERROR);
      // 保持传输层返回 OK，使主机仍能收到有效响应。
      // Keep transport-level OK so host still gets a valid response.
      return ErrorCode::OK;
    }

    // 保持影子语义：SWCLK=0，SWDIO 为最后一位。
    // Maintain shadow semantics: keep SWCLK=0, SWDIO=last bit
    swj_shadow_ = static_cast<uint8_t>(
        swj_shadow_ & static_cast<uint8_t>(~LibXR::USB::DapLinkV2Def::DAP_SWJ_SWCLK_TCK));

    bool last_swdio = false;
    if (BIT_COUNT != 0u)
    {
      const uint32_t LAST_I = BIT_COUNT - 1u;
      last_swdio = (((data[LAST_I / 8u] >> (LAST_I & 7u)) & 0x01u) != 0u);
    }

    if (last_swdio)
    {
      swj_shadow_ |= LibXR::USB::DapLinkV2Def::DAP_SWJ_SWDIO_TMS;
    }
    else
    {
      swj_shadow_ = static_cast<uint8_t>(
          swj_shadow_ &
          static_cast<uint8_t>(~LibXR::USB::DapLinkV2Def::DAP_SWJ_SWDIO_TMS));
    }

    return ErrorCode::OK;
  }

  ErrorCode HandleSWDConfigure(const uint8_t* req, uint16_t req_len, uint8_t* resp,
                               uint16_t resp_cap, uint16_t& out_len)
  {
    if (resp_cap < 2u)
    {
      out_len = 0;
      return ErrorCode::NOT_FOUND;
    }

    resp[0] = ToU8(LibXR::USB::DapLinkV2Def::CommandId::SWD_CONFIGURE);

    if (req == nullptr || req_len < 2u)
    {
      resp[1] = ToU8(LibXR::USB::DapLinkV2Def::Status::ERROR);
      out_len = 2u;
      return ErrorCode::ARG_ERR;
    }

    const uint8_t cfg = req[1];
    dap_state_.swd_cfg.turnaround = static_cast<uint8_t>((cfg & 0x03u) + 1u);
    dap_state_.swd_cfg.data_phase = ((cfg & 0x04u) != 0u);

    resp[1] = ToU8(LibXR::USB::DapLinkV2Def::Status::OK);
    out_len = 2u;
    return ErrorCode::OK;
  }

  ErrorCode HandleSWDSequence(const uint8_t* req, uint16_t req_len, uint8_t* resp,
                              uint16_t resp_cap, uint16_t& out_len)
  {
    if (!req || !resp || resp_cap < 2u)
    {
      out_len = 0u;
      return ErrorCode::ARG_ERR;
    }

    resp[0] = ToU8(LibXR::USB::DapLinkV2Def::CommandId::SWD_SEQUENCE);
    resp[1] = DAP_OK;
    out_len = 2u;

    // Req: [0]=0x1D [1]=SequenceCount [ ... sequences ... ]
    // 每条序列：
    //   INFO：[7]=方向（1=输入，0=输出），[5:0]=周期数（0 表示 64）
    //   输出：后接 ceil(cycles/8) 字节数据（LSB 在前）
    //   输入：请求中不含数据；响应追加 ceil(cycles/8) 字节数据（LSB 在前）
    // Each sequence:
    //   INFO: [7]=Direction (1=input,0=output), [5:0]=cycles (0=>64)
    //   If output: followed by ceil(cycles/8) bytes data (LSB-first)
    //   If input : no data in request; response appends ceil(cycles/8) bytes data
    //   (LSB-first)
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
        // 输出：数据字节在请求中 / Output: data bytes are in request
        if (req_off + BYTES > req_len)
        {
          resp[1] = DAP_ERROR;
          out_len = 2u;
          return ErrorCode::ARG_ERR;
        }

        const uint8_t* data = &req[req_off];
        req_off = static_cast<uint16_t>(req_off + BYTES);

        const ErrorCode EC = swd_.SeqWriteBits(cycles, data);
        if (EC != ErrorCode::OK)
        {
          resp[1] = DAP_ERROR;
          out_len = 2u;
          // 返回 OK，使主机收到有效的响应包。
          // Return OK so host gets a valid response packet
          return ErrorCode::OK;
        }
      }
      else
      {
        // 输入：采集到的数据追加到响应中 / Input: captured data is appended to response
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

  ErrorCode HandleJTAGSequence(const uint8_t* req, uint16_t req_len, uint8_t* resp,
                               uint16_t resp_cap, uint16_t& out_len)
  {
    if (!req || !resp || resp_cap < 2u)
    {
      out_len = 0u;
      return ErrorCode::ARG_ERR;
    }

    resp[0] = ToU8(LibXR::USB::DapLinkV2Def::CommandId::JTAG_SEQUENCE);
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

      const uint8_t* data = &req[req_off];
      uint8_t* out = CAPTURE ? &resp[resp_off] : nullptr;
      if (CAPTURE)
      {
        Memory::FastSet(out, 0, BYTES);
      }

      const ErrorCode ec = jtag_->Sequence(cycles, TMS, data, out);
      if (ec != ErrorCode::OK)
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

  ErrorCode HandleJTAGConfigure(const uint8_t* req, uint16_t req_len, uint8_t* resp,
                                uint16_t resp_cap, uint16_t& out_len)
  {
    if (!req || !resp || resp_cap < 2u)
    {
      out_len = 0u;
      return ErrorCode::ARG_ERR;
    }

    resp[0] = ToU8(LibXR::USB::DapLinkV2Def::CommandId::JTAG_CONFIGURE);
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

    const uint8_t count = req[1];
    if (count > JTAG_MAX_DEVICES || req_len < static_cast<uint16_t>(2u + count))
    {
      resp[1] = DAP_ERROR;
      return ErrorCode::ARG_ERR;
    }

    Memory::FastSet(jtag_ir_len_, 0, sizeof(jtag_ir_len_));

    uint32_t bits = 0u;
    for (uint32_t i = 0; i < count; ++i)
    {
      const uint8_t len = req[static_cast<uint16_t>(2u + i)];
      jtag_ir_len_[i] = len;
      bits += len;
    }

    jtag_chain_.count = count;
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

  ErrorCode HandleJTAGIdCode(const uint8_t* req, uint16_t req_len, uint8_t* resp,
                             uint16_t resp_cap, uint16_t& out_len)
  {
    if (!req || !resp || resp_cap < 6u)
    {
      out_len = 0u;
      return ErrorCode::ARG_ERR;
    }

    resp[0] = ToU8(LibXR::USB::DapLinkV2Def::CommandId::JTAG_IDCODE);
    resp[1] = DAP_ERROR;
    out_len = 2u;

    if (jtag_ == nullptr ||
        dap_state_.debug_port != LibXR::USB::DapLinkV2Def::DebugPort::JTAG)
    {
      return ErrorCode::OK;
    }

    if (req_len < 2u)
    {
      return ErrorCode::ARG_ERR;
    }

    const uint8_t index = req[1];
    if (index >= jtag_chain_.count)
    {
      return ErrorCode::OK;
    }

    jtag_chain_.index = index;
    LibXR::Debug::JtagProtocol::UpdateChainCache(jtag_chain_);

    LibXR::Debug::JtagDp dp(*jtag_, &jtag_chain_);
    uint32_t idcode = 0u;
    ErrorCode ec = dp.ReadIdCode(idcode);
    if (ec == ErrorCode::OK &&
        (idcode == 0u || idcode == 0xFFFF'FFFFu || ((idcode & 0x1u) == 0u)) &&
        jtag_ir_len_[index] != 4u)
    {
      uint32_t fallback_idcode = 0u;
      ec = dp.ReadIdCodeWithIr(0x01u, fallback_idcode);
      if (ec != ErrorCode::OK)
      {
        return ec;
      }
      if (fallback_idcode != 0u && fallback_idcode != 0xFFFF'FFFFu &&
          ((fallback_idcode & 0x1u) != 0u))
      {
        idcode = fallback_idcode;
      }
    }
    if (ec != ErrorCode::OK)
    {
      return ec;
    }

    resp[1] = DAP_OK;
    Memory::FastCopy(&resp[2], &idcode, sizeof(idcode));
    out_len = 6u;
    return ErrorCode::OK;
  }

  /**
   * @brief 校验 [ID, 命令数, 命令...]：每条命令都能按自己的内容确定长度，且都在请求之内
   *        Validate [ID, count, commands...]: every command's length follows from its
   *        content and all of them lie within the request
   * @note 最后一条命令之后的字节忽略（主机可能把请求补到包长），与 CMSIS-DAP 参考实现
   *       相同。
   *       Bytes after the last command are ignored (a host may pad the request to the
   *       packet size), as in the CMSIS-DAP reference.
   */
  bool ValidateInlineCommands(const uint8_t* req, uint16_t req_len) const
  {
    if (!req || req_len < 2u)
    {
      return false;
    }

    const uint8_t NUM = req[1];
    uint16_t req_off = 2u;
    for (uint32_t i = 0u; i < NUM; ++i)
    {
      if (req_off >= req_len)
      {
        return false;
      }

      uint16_t cmd_len = 0u;
      if (!ParseQueuedCommandLength(&req[req_off],
                                    static_cast<uint16_t>(req_len - req_off), cmd_len) ||
          cmd_len == 0u)
      {
        return false;
      }
      req_off = static_cast<uint16_t>(req_off + cmd_len);
    }

    return req_off <= req_len;
  }

  /**
   * @brief DAP_ExecuteCommands（也用于 DAP_QueueCommands）/ DAP_ExecuteCommands (also
   *        used for DAP_QueueCommands)
   *
   * 请求 [ID, 命令数, 命令...]，响应 [0x7F, 命令数, 各条命令的响应...]。
   * Request [ID, count, commands...], response [0x7F, count, each command's response...].
   */
  ErrorCode HandleExecuteCommands(const uint8_t* req, uint16_t req_len, uint8_t* resp,
                                  uint16_t resp_cap, uint16_t& out_len)
  {
    if (!ValidateInlineCommands(req, req_len) ||
        !TryExecuteQueuedCommandStream(&req[2], static_cast<uint16_t>(req_len - 2u),
                                       req[1], resp, resp_cap, out_len))
    {
      return BuildCmdStatusResponse(
          ToU8(LibXR::USB::DapLinkV2Def::CommandId::EXECUTE_COMMANDS), DAP_ERROR, resp,
          resp_cap, out_len);
    }
    return ErrorCode::OK;
  }

 private:
  uint8_t MapAckToDapResp(LibXR::Debug::SwdProtocol::Ack ack) const
  {
    static constexpr uint8_t ACK_MAP[8] = {7u, 1u, 2u, 7u, 4u, 7u, 7u, 7u};
    return ACK_MAP[static_cast<uint8_t>(ack) & 0x07u];
  }

  uint8_t MapAckToDapResp(LibXR::Debug::JtagProtocol::Ack ack) const
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

  static inline uint16_t LoadU16Le(const uint8_t* p)
  {
    return static_cast<uint16_t>(
        static_cast<uint16_t>(p[0]) |
        static_cast<uint16_t>(static_cast<uint16_t>(p[1]) << 8u));
  }

  static inline void StoreU16Le(uint8_t* p, uint16_t v)
  {
    p[0] = static_cast<uint8_t>(v & 0xFFu);
    p[1] = static_cast<uint8_t>((v >> 8u) & 0xFFu);
  }

  static inline uint32_t LoadU32Le(const uint8_t* p)
  {
    return static_cast<uint32_t>(
        static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8u) |
        (static_cast<uint32_t>(p[2]) << 16u) | (static_cast<uint32_t>(p[3]) << 24u));
  }

  static inline void StoreU32Le(uint8_t* p, uint32_t v)
  {
    p[0] = static_cast<uint8_t>(v & 0xFFu);
    p[1] = static_cast<uint8_t>((v >> 8u) & 0xFFu);
    p[2] = static_cast<uint8_t>((v >> 16u) & 0xFFu);
    p[3] = static_cast<uint8_t>((v >> 24u) & 0xFFu);
  }

  ErrorCode TransferTxnFast(const LibXR::Debug::SwdProtocol::Request& req,
                            LibXR::Debug::SwdProtocol::Response& resp)
  {
    const ErrorCode EC0 = swd_.Transfer(req, resp);
    if (EC0 != ErrorCode::OK)
    {
      return EC0;
    }

    if (resp.ack == LibXR::Debug::SwdProtocol::Ack::WAIT)
    {
      return swd_.TransferWithRetry(req, resp);
    }

    if (resp.ack == LibXR::Debug::SwdProtocol::Ack::FAULT)
    {
      const auto POL = swd_.GetTransferPolicy();
      if (POL.clear_sticky_on_fault)
      {
        LibXR::Debug::SwdProtocol::Ack abort_ack =
            LibXR::Debug::SwdProtocol::Ack::PROTOCOL;
        (void)swd_.WriteAbort(LibXR::Debug::SwdProtocol::DP_ABORT_STKCMPCLR |
                                  LibXR::Debug::SwdProtocol::DP_ABORT_STKERRCLR |
                                  LibXR::Debug::SwdProtocol::DP_ABORT_WDERRCLR |
                                  LibXR::Debug::SwdProtocol::DP_ABORT_ORUNERRCLR,
                              abort_ack);
      }
    }

    return ErrorCode::OK;
  }

  ErrorCode DpReadRdbuffFast(uint32_t& val, LibXR::Debug::SwdProtocol::Ack& ack_out)
  {
    LibXR::Debug::SwdProtocol::Response swd_resp = {};
    const auto req = LibXR::Debug::SwdProtocol::MakeDpReadRequest(
        LibXR::Debug::SwdProtocol::DpReadReg::RDBUFF);
    const ErrorCode ec = TransferTxnFast(req, swd_resp);
    ack_out = swd_resp.ack;
    if (ec != ErrorCode::OK)
    {
      return ec;
    }
    if (swd_resp.ack != LibXR::Debug::SwdProtocol::Ack::OK || !swd_resp.parity_ok)
    {
      return ErrorCode::FAILED;
    }
    val = swd_resp.rdata;
    return ErrorCode::OK;
  }

  ErrorCode CompletePendingApReadFast(uint32_t& val,
                                      LibXR::Debug::SwdProtocol::Ack& ack_out)
  {
    return DpReadRdbuffFast(val, ack_out);
  }

  ErrorCode CheckPostedApWriteFast(LibXR::Debug::SwdProtocol::Ack& ack_out)
  {
    uint32_t value = 0u;
    return DpReadRdbuffFast(value, ack_out);
  }

  ErrorCode ApReadPostedFast(uint8_t addr2b, uint32_t& posted,
                             LibXR::Debug::SwdProtocol::Ack& ack_out)
  {
    LibXR::Debug::SwdProtocol::Response swd_resp = {};
    const auto req = LibXR::Debug::SwdProtocol::MakeApReadRequest(addr2b);
    const ErrorCode ec = TransferTxnFast(req, swd_resp);
    ack_out = swd_resp.ack;
    if (ec != ErrorCode::OK)
    {
      return ec;
    }
    if (swd_resp.ack != LibXR::Debug::SwdProtocol::Ack::OK || !swd_resp.parity_ok)
    {
      return ErrorCode::FAILED;
    }
    posted = swd_resp.rdata;
    return ErrorCode::OK;
  }

  void SetTransferAbortFlag(bool on)
  {
    transfer_abort_.store(on, std::memory_order_release);
  }

 private:
  ErrorCode HandleTransfer(const uint8_t* req, uint16_t req_len, uint8_t* resp,
                           uint16_t resp_cap, uint16_t& out_len)
  {
    out_len = 0u;
    if (!req || !resp || resp_cap < 3u)
    {
      return ErrorCode::ARG_ERR;
    }

    if constexpr (JTAG_ENABLED)
    {
      if (dap_state_.debug_port == LibXR::USB::DapLinkV2Def::DebugPort::JTAG)
      {
        return HandleJtagTransfer(req, req_len, resp, resp_cap, out_len);
      }
    }

    resp[0] = ToU8(LibXR::USB::DapLinkV2Def::CommandId::TRANSFER);
    // resp[1]：成功的传输数；resp[2]：应答值（ACK 位、ERROR、MISMATCH）。
    // resp[1]: number of successful transfers; resp[2]: response value (ACK bits,
    // ERROR, MISMATCH).
    resp[1] = 0u;
    resp[2] = 0u;
    uint16_t resp_off = 3u;

    // Req: [0]=CMD [1]=DAP index [2]=count [3..]=transfers...
    if (req_len < 3u)
    {
      resp[2] = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
      out_len = 3u;
      return ErrorCode::ARG_ERR;
    }

    // 传输开始时清掉旧的中止标志（一个在没有传输时到来的中止不应中止下一条传输）；
    // 之后到来的中止在传输执行中置位。
    // Clear a stale abort flag when a transfer starts (an abort that arrives while no
    // transfer runs must not abort the next one); an abort that arrives later sets it
    // during the execution.
    transfer_abort_.store(false, std::memory_order_release);

    const uint8_t COUNT = req[2];
    uint16_t req_off = 3u;

    auto push_u32 = [&](uint32_t v) -> bool
    {
      if (resp_off + 4u > resp_cap)
      {
        return false;
      }
      Memory::FastCopy(&resp[resp_off], &v, sizeof(v));
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
    // 保持参考实现的行为：没有任何传输执行时，response_value 保持为 0。
    // Keep reference behavior: if no transfer executes, response_value remains 0.
    uint8_t response_value = 0u;

    // 目标会以 posted 方式处理 AP 写入。保持写批量的快速，
    // 并在传输边界处用一次 DP RDBUFF 读取统一检查。
    // AP writes are posted by the target. Keep the write batch fast and check
    // it once at a transfer boundary with a DP RDBUFF read.
    bool pending_ap_write = false;

    // -------- posted 读流水线状态（仅 AP 读） --------
    // -------- posted-read pipeline state (AP read only) --------
    struct PendingApRead
    {
      bool valid = false;
      // 此 AP 读传输是否需要时间戳。
      // Whether this AP read transfer needs timestamp.
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

    // 通过 DP RDBUFF 完成待完成的 AP 读。
    // RDBUFF 是 AP 读结果流水线的出口，不是 AP 写的检查器。
    // Complete a pending AP read by DP RDBUFF.
    // RDBUFF is the AP read-result pipe outlet, not the AP-write checker.
    auto complete_pending_ap_read = [&]() -> bool
    {
      if (!pending.valid)
      {
        return true;
      }

      if (!ensure_space(bytes_for_read(pending.need_ts)))
      {
        response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
        return false;
      }

      uint32_t rdata = 0u;
      LibXR::Debug::SwdProtocol::Ack ack = LibXR::Debug::SwdProtocol::Ack::PROTOCOL;
      const ErrorCode EC = CompletePendingApReadFast(rdata, ack);

      const uint8_t V = MapAckToDapResp(ack);
      if (V != LibXR::USB::DapLinkV2Def::DAP_TRANSFER_OK)
      {
        response_value = V;
        return false;
      }
      if (EC != ErrorCode::OK)
      {
        response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
        return false;
      }

      if (!emit_read_with_ts(pending.need_ts, rdata))
      {
        response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
        return false;
      }

      pending.valid = false;
      pending.need_ts = false;

      // 成功路径保持 OK / Successful path keeps OK.
      response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_OK;
      return true;
    };

    auto check_posted_ap_write_if_pending = [&]() -> bool
    {
      if (!pending_ap_write)
      {
        return true;
      }
      LibXR::Debug::SwdProtocol::Ack check_ack = LibXR::Debug::SwdProtocol::Ack::PROTOCOL;
      const ErrorCode check_ec = CheckPostedApWriteFast(check_ack);
      response_value = MapAckToDapResp(check_ack);
      if (response_value != LibXR::USB::DapLinkV2Def::DAP_TRANSFER_OK)
      {
        return false;
      }
      if (check_ec != ErrorCode::OK)
      {
        response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
        return false;
      }
      pending_ap_write = false;
      return true;
    };

    // 在非 AP 操作之前先完成待完成的 AP 读，以保持响应顺序。
    // Complete pending AP read before non-AP operations to keep response order.
    auto complete_pending_ap_read_if_any = [&]() -> bool
    { return pending.valid ? complete_pending_ap_read() : true; };

    auto close_posted_state = [&]() -> bool
    { return complete_pending_ap_read_if_any() && check_posted_ap_write_if_pending(); };

    // -------- 主循环 --------
    // -------- main loop --------
    for (uint32_t i = 0; i < COUNT; ++i)
    {
      if (req_off >= req_len)
      {
        response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
        break;
      }

      const uint8_t RQ = req[req_off++];

      const bool AP = LibXR::USB::DapLinkV2Def::req_is_ap(RQ);
      const bool RNW = LibXR::USB::DapLinkV2Def::req_is_read(RQ);
      const uint8_t ADDR2B = LibXR::USB::DapLinkV2Def::req_addr2b(RQ);

      const bool TS = LibXR::USB::DapLinkV2Def::req_need_timestamp(RQ);
      const bool MATCH_VALUE =
          ((RQ & LibXR::USB::DapLinkV2Def::DAP_TRANSFER_MATCH_VALUE) != 0u);
      const bool MATCH_MASK =
          ((RQ & LibXR::USB::DapLinkV2Def::DAP_TRANSFER_MATCH_MASK) != 0u);

      // 规范规定：时间戳不能与匹配位组合使用。
      // Spec rule: timestamp cannot be combined with match bits.
      if (TS && (MATCH_VALUE || MATCH_MASK))
      {
        response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
        break;
      }

      LibXR::Debug::SwdProtocol::Ack ack = LibXR::Debug::SwdProtocol::Ack::PROTOCOL;
      ErrorCode ec = ErrorCode::OK;

      if (!RNW)
      {
        // ---------------- WRITE ----------------
        // 写入可以紧跟在之前的 AP 写入之后，无需强制恢复。
        // 仅在使用写数据之前关闭 AP 读流水线。
        // A write can follow previous AP writes without forcing a recovery.
        // Only close an AP-read pipeline before consuming write data.
        if (!complete_pending_ap_read_if_any())
        {
          break;
        }

        if (req_off + 4u > req_len)
        {
          response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
          break;
        }

        uint32_t wdata = LoadU32Le(&req[req_off]);
        req_off = static_cast<uint16_t>(req_off + 4u);

        if (MATCH_MASK)
        {
          match_mask_ = wdata;
          response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_OK;
          // MATCH_MASK 算作一次成功的传输。
          // MATCH_MASK counts as a successful transfer.
          response_count++;
          continue;
        }
        if (AP)
        {
          ec = swd_.ApWriteTxn(ADDR2B, wdata, ack);
        }
        else
        {
          ec = swd_.DpWriteTxn(static_cast<LibXR::Debug::SwdProtocol::DpWriteReg>(ADDR2B),
                               wdata, ack);
        }

        response_value = MapAckToDapResp(ack);
        if (response_value != LibXR::USB::DapLinkV2Def::DAP_TRANSFER_OK)
        {
          break;
        }
        if (ec != ErrorCode::OK)
        {
          response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
          break;
        }

        if (AP)
        {
          pending_ap_write = true;
        }

        if (TS)
        {
          if (!push_timestamp())
          {
            response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
            break;
          }
        }

        response_count++;
      }
      else
      {
        // ---------------- READ ----------------

        if (MATCH_VALUE)
        {
          // MATCH_VALUE 会执行真实读取，可能观察到之前的 AP 写入，
          // 所以先关闭 posted 读和 posted AP 写两种状态。
          // MATCH_VALUE performs real reads and can observe earlier AP writes,
          // so close both posted read and posted AP-write state first.
          if (!close_posted_state())
          {
            break;
          }

          if (req_off + 4u > req_len)
          {
            response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
            break;
          }

          uint32_t match_val = LoadU32Le(&req[req_off]);
          req_off = static_cast<uint16_t>(req_off + 4u);

          uint32_t rdata = 0u;
          uint32_t retry = dap_state_.transfer_cfg.match_retry;
          bool matched = false;

          while (true)
          {
            if (AP)
            {
              // 包含 RDBUFF 的读取 / Includes the RDBUFF read.
              ec = swd_.ApReadTxn(ADDR2B, rdata, ack);
              if (ec == ErrorCode::OK && ack == LibXR::Debug::SwdProtocol::Ack::OK)
              {
                // ApReadTxn 仅完成 AP 读流水线。
                // ApReadTxn completes the AP read pipeline only.
              }
            }
            else
            {
              ec = swd_.DpReadTxn(
                  static_cast<LibXR::Debug::SwdProtocol::DpReadReg>(ADDR2B), rdata, ack);
            }

            response_value = MapAckToDapResp(ack);
            if (response_value != LibXR::USB::DapLinkV2Def::DAP_TRANSFER_OK)
            {
              break;
            }
            if (ec != ErrorCode::OK)
            {
              response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
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

          if (response_value != LibXR::USB::DapLinkV2Def::DAP_TRANSFER_OK)
          {
            break;
          }

          if (!matched)
          {
            response_value =
                static_cast<uint8_t>(LibXR::USB::DapLinkV2Def::DAP_TRANSFER_OK |
                                     LibXR::USB::DapLinkV2Def::DAP_TRANSFER_MISMATCH);
            // MISMATCH 不增加 response_count / MISMATCH does not increase response_count.
            break;
          }

          response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_OK;
          response_count++;
          continue;
        }

        // ---- 普通读 ----
        // ---- Normal read ----
        if (!AP)
        {
          // DP 读不参与 posted 流水线 / DP read does not participate in posted pipelines.
          if (!close_posted_state())
          {
            break;
          }

          uint32_t rdata = 0u;
          ec = swd_.DpReadTxn(static_cast<LibXR::Debug::SwdProtocol::DpReadReg>(ADDR2B),
                              rdata, ack);

          response_value = MapAckToDapResp(ack);
          if (response_value != LibXR::USB::DapLinkV2Def::DAP_TRANSFER_OK)
          {
            break;
          }
          if (ec != ErrorCode::OK)
          {
            response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
            break;
          }

          if (!ensure_space(bytes_for_read(TS)))
          {
            response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
            break;
          }

          if (!emit_read_with_ts(TS, rdata))
          {
            response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
            break;
          }

          response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_OK;
          continue;
        }

        // AP 读可能观察到之前的 AP 写入，
        // 所以在启动或延伸 posted 读流水线之前先关闭 posted 写入。
        // AP reads can observe previous AP writes, so close posted writes
        // before starting or extending the posted-read pipeline.
        if (!check_posted_ap_write_if_pending())
        {
          break;
        }

        // AP 普通读：posted 读流水线 / AP normal read: posted-read pipeline.
        if (!pending.valid)
        {
          // 本段连续 AP 读中的第一次 AP 读：
          // 启动一次 posted AP 读并丢弃返回的 posted 数据。
          // First AP read in this contiguous AP-read segment:
          // start one posted AP read and discard returned posted data.
          uint32_t dummy_posted = 0u;
          ec = ApReadPostedFast(ADDR2B, dummy_posted, ack);

          response_value = MapAckToDapResp(ack);
          if (response_value != LibXR::USB::DapLinkV2Def::DAP_TRANSFER_OK)
          {
            break;
          }
          if (ec != ErrorCode::OK)
          {
            response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
            {
              break;
            }
          }

          pending.valid = true;
          pending.need_ts = TS;

          // 传输尚未“完成”。
          // 数据将由下一次 AP 读或末尾的 RDBUFF 产生。
          // Transfer is not "complete" yet.
          // Data will be produced by next AP read or tail RDBUFF.
          response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_OK;
        }
        else
        {
          // 存在待完成的读：当前 AP 读返回的是上一次 AP 读的 posted 数据。
          // Pending exists: current AP read returns posted data
          // from the previous AP read.
          if (!ensure_space(bytes_for_read(pending.need_ts)))
          {
            response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
            break;
          }

          uint32_t posted_prev = 0u;
          ec = ApReadPostedFast(ADDR2B, posted_prev, ack);

          const uint8_t CUR_V = MapAckToDapResp(ack);
          if (CUR_V != LibXR::USB::DapLinkV2Def::DAP_TRANSFER_OK || ec != ErrorCode::OK)
          {
            // 当前 AP 读失败：尽力完成待完成的 AP 读。
            // 否则 response_count 可能少算一次，流水线也可能残留脏状态。
            // Current AP read failed: try best effort to complete pending AP read.
            // Otherwise, response_count may miss one and pipeline may remain dirty.
            const uint8_t PROOR_FAIL =
                (CUR_V != LibXR::USB::DapLinkV2Def::DAP_TRANSFER_OK)
                    ? CUR_V
                    : LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;

            if (!complete_pending_ap_read())
            {
              // 待完成的读本身失败：返回较早的失败
              // （complete_pending_ap_read 已写入 response_value）。
              // Pending itself failed: return earlier failure
              // (complete_pending_ap_read already wrote response_value).
              break;
            }

            // 待完成的读已成功完成：保持当前失败。
            // Pending completion succeeded: keep current failure.
            response_value = PROOR_FAIL;
            break;
          }

          // 当前 AP 读成功：先输出待完成的读（使用 posted_prev），
          // 再把当前读标记为新的待完成读。
          // Current AP read succeeded: emit pending first (using posted_prev),
          // then mark current read as new pending.
          if (!emit_read_with_ts(pending.need_ts, posted_prev))
          {
            response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
            break;
          }

          pending.valid = true;
          pending.need_ts = TS;

          response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_OK;
        }
      }

      if (transfer_abort_.load(std::memory_order_acquire))
      {
        transfer_abort_.store(false, std::memory_order_release);
        break;
      }
    }

    // 若末尾仍存在待完成的 AP 读：
    // - response_value == OK 时：用一次 RDBUFF 正常收尾；
    // - response_value != OK 时：先尝试完成待完成的读；若成功，保持原失败，
    //   否则采用待完成读的失败。
    // If pending AP read still exists at tail:
    // - when response_value == OK: normal tail by one RDBUFF;
    // - when response_value != OK: try pending completion first; if completion
    //   succeeds, keep original failure; otherwise use pending failure.
    if (pending.valid)
    {
      const uint8_t PRIOR_FAIL = response_value;

      if (!complete_pending_ap_read())
      {
        // 此处以待完成读的失败为准（response_value 已写入）。
        // Pending failure wins here (response_value already written).
      }
      else
      {
        // 待完成的读已成功完成：保持原失败
        // （若原来为 OK，则保持 OK）。
        // Pending completion succeeded: keep original failure
        // (if original was OK, keep OK).
        if (PRIOR_FAIL != 0u && PRIOR_FAIL != LibXR::USB::DapLinkV2Def::DAP_TRANSFER_OK)
        {
          response_value = PRIOR_FAIL;
        }
      }
    }

    if (response_value == LibXR::USB::DapLinkV2Def::DAP_TRANSFER_OK &&
        !check_posted_ap_write_if_pending())
    {
      // response_value 已由恢复辅助函数设置。
      // response_value is already set by the recovery helper.
    }
    resp[1] = response_count;
    resp[2] = response_value;
    out_len = resp_off;
    return ErrorCode::OK;
  }
  ErrorCode HandleTransferBlock(const uint8_t* req, uint16_t req_len, uint8_t* resp,
                                uint16_t resp_cap, uint16_t& out_len)
  {
    if constexpr (JTAG_ENABLED)
    {
      if (dap_state_.debug_port == LibXR::USB::DapLinkV2Def::DebugPort::JTAG)
      {
        return HandleJtagTransferBlock(req, req_len, resp, resp_cap, out_len);
      }
    }

    // Req:  [0]=0x06 [1]=index [2..3]=count [4]=request [5..]=data(write)
    // Resp: [0]=0x06 [1..2]=done [3]=resp [4..]=data(read)
    if (!resp || resp_cap < 4u)
    {
      out_len = 0u;
      return ErrorCode::NOT_FOUND;
    }

    resp[0] = ToU8(LibXR::USB::DapLinkV2Def::CommandId::TRANSFER_BLOCK);
    resp[1] = 0u;
    resp[2] = 0u;
    resp[3] = 0u;
    out_len = 4u;

    if (!req || req_len < 5u)
    {
      resp[3] = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
      return ErrorCode::ARG_ERR;
    }

    // 传输开始时清掉旧的中止标志（一个在没有传输时到来的中止不应中止下一条传输）；
    // 之后到来的中止在传输执行中置位。
    // Clear a stale abort flag when a transfer starts (an abort that arrives while no
    // transfer runs must not abort the next one); an abort that arrives later sets it
    // during the execution.
    transfer_abort_.store(false, std::memory_order_release);

    uint16_t count = 0u;
    count = LoadU16Le(&req[2]);

    const uint8_t DAP_RQ = req[4];

    // TransferBlock 不支持匹配和时间戳。
    // TransferBlock does not support match or timestamp
    if ((DAP_RQ & (LibXR::USB::DapLinkV2Def::DAP_TRANSFER_MATCH_VALUE |
                   LibXR::USB::DapLinkV2Def::DAP_TRANSFER_MATCH_MASK)) != 0u)
    {
      resp[3] = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
      return ErrorCode::NOT_SUPPORT;
    }
    if (LibXR::USB::DapLinkV2Def::req_need_timestamp(DAP_RQ))
    {
      resp[3] = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
      return ErrorCode::NOT_SUPPORT;
    }

    // Count==0：返回 OK / Count==0: return OK
    if (count == 0u)
    {
      const uint16_t DONE0 = 0u;
      StoreU16Le(&resp[1], DONE0);
      resp[3] = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_OK;
      out_len = 4u;
      return ErrorCode::OK;
    }

    const bool AP = LibXR::USB::DapLinkV2Def::req_is_ap(DAP_RQ);
    const bool RNW = LibXR::USB::DapLinkV2Def::req_is_read(DAP_RQ);
    const uint8_t ADDR2B = LibXR::USB::DapLinkV2Def::req_addr2b(DAP_RQ);

    uint16_t done = 0u;
    uint8_t xresp = 0u;

    uint16_t req_off = 5u;
    uint16_t resp_off = 4u;

    // WRITE 路径：保持原有行为 / WRITE path: keep original behavior
    if (!RNW)
    {
      const uint32_t REQ_NEED =
          static_cast<uint32_t>(req_off) + (static_cast<uint32_t>(count) * 4u);
      if (REQ_NEED > req_len)
      {
        xresp |= LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
        StoreU16Le(&resp[1], done);
        resp[3] = xresp;
        out_len = resp_off;
        return ErrorCode::OK;
      }
      if (AP)
      {
        auto swd_req = LibXR::Debug::SwdProtocol::MakeApWriteRequest(ADDR2B, 0u);
        LibXR::Debug::SwdProtocol::Response swd_resp = {};
        for (uint32_t i = 0; i < count; ++i)
        {
          uint32_t wdata = LoadU32Le(&req[req_off]);
          req_off = static_cast<uint16_t>(req_off + 4u);
          swd_req.wdata = wdata;
          const ErrorCode ec = TransferTxnFast(swd_req, swd_resp);
          xresp = MapAckToDapResp(swd_resp.ack);
          if (swd_resp.ack != LibXR::Debug::SwdProtocol::Ack::OK)
          {
            break;
          }
          if (ec != ErrorCode::OK)
          {
            xresp |= LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
            break;
          }
          done = static_cast<uint16_t>(i + 1u);
        }
        if (xresp == LibXR::USB::DapLinkV2Def::DAP_TRANSFER_OK && done != 0u)
        {
          LibXR::Debug::SwdProtocol::Ack check_ack =
              LibXR::Debug::SwdProtocol::Ack::PROTOCOL;
          const ErrorCode check_ec = CheckPostedApWriteFast(check_ack);
          xresp = MapAckToDapResp(check_ack);
          if (check_ack == LibXR::Debug::SwdProtocol::Ack::OK &&
              check_ec != ErrorCode::OK)
          {
            xresp |= LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
          }
        }
      }
      else
      {
        for (uint32_t i = 0; i < count; ++i)
        {
          LibXR::Debug::SwdProtocol::Ack ack = LibXR::Debug::SwdProtocol::Ack::PROTOCOL;
          ErrorCode ec = ErrorCode::OK;
          uint32_t wdata = LoadU32Le(&req[req_off]);
          req_off = static_cast<uint16_t>(req_off + 4u);
          ec = swd_.DpWriteTxn(static_cast<LibXR::Debug::SwdProtocol::DpWriteReg>(ADDR2B),
                               wdata, ack);
          xresp = MapAckToDapResp(ack);
          if (ack != LibXR::Debug::SwdProtocol::Ack::OK)
          {
            break;
          }
          if (ec != ErrorCode::OK)
          {
            xresp |= LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
            break;
          }
          done = static_cast<uint16_t>(i + 1u);
        }
      }
      StoreU16Le(&resp[1], done);
      resp[3] = xresp;
      out_len = resp_off;
      return ErrorCode::OK;
    }

    // READ 路径 / READ path
    if (!AP)
    {
      const uint32_t RESP_NEED =
          static_cast<uint32_t>(resp_off) + (static_cast<uint32_t>(count) * 4u);
      if (RESP_NEED > resp_cap)
      {
        xresp |= LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
        StoreU16Le(&resp[1], done);
        resp[3] = xresp;
        out_len = resp_off;
        return ErrorCode::OK;
      }

      // DP 读：保持原有行为 / DP read: keep original behavior
      for (uint32_t i = 0; i < count; ++i)
      {
        LibXR::Debug::SwdProtocol::Ack ack = LibXR::Debug::SwdProtocol::Ack::PROTOCOL;
        ErrorCode ec = ErrorCode::OK;
        uint32_t rdata = 0u;
        ec = swd_.DpReadTxn(static_cast<LibXR::Debug::SwdProtocol::DpReadReg>(ADDR2B),
                            rdata, ack);
        xresp = MapAckToDapResp(ack);
        if (ack != LibXR::Debug::SwdProtocol::Ack::OK)
        {
          break;
        }
        if (ec != ErrorCode::OK)
        {
          xresp |= LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
          break;
        }
        StoreU32Le(&resp[resp_off], rdata);
        resp_off = static_cast<uint16_t>(resp_off + 4u);
        done = static_cast<uint16_t>(i + 1u);
      }
      StoreU16Le(&resp[1], done);
      resp[3] = xresp;
      out_len = resp_off;
      return ErrorCode::OK;
    }

    // AP 读：posted 读流水线 / AP read: posted-read pipeline
    {
      const uint32_t RESP_NEED =
          static_cast<uint32_t>(resp_off) + (static_cast<uint32_t>(count) * 4u);
      if (RESP_NEED > resp_cap)
      {
        xresp |= LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
        StoreU16Le(&resp[1], done);
        resp[3] = xresp;
        out_len = resp_off;
        return ErrorCode::OK;
      }

      auto ap_read_req = LibXR::Debug::SwdProtocol::MakeApReadRequest(ADDR2B);
      LibXR::Debug::SwdProtocol::Response ap_read_resp = {};
      ErrorCode ec = TransferTxnFast(ap_read_req, ap_read_resp);
      xresp = MapAckToDapResp(ap_read_resp.ack);
      if (ap_read_resp.ack != LibXR::Debug::SwdProtocol::Ack::OK)
      {
        goto out_ap_read;  // NOLINT
      }
      if (ec != ErrorCode::OK || !ap_read_resp.parity_ok)
      {
        xresp |= LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
        goto out_ap_read;  // NOLINT
      }

      // i=1..COUNT-1：每次 AP 读返回上一次的 posted 数据。
      // i=1..COUNT-1: each AP read returns previous posted data.
      for (uint32_t i = 1; i < count; ++i)
      {
        ec = TransferTxnFast(ap_read_req, ap_read_resp);
        const uint8_t CUR = MapAckToDapResp(ap_read_resp.ack);

        if (ap_read_resp.ack != LibXR::Debug::SwdProtocol::Ack::OK ||
            ec != ErrorCode::OK || !ap_read_resp.parity_ok)
        {
          // 当前 AP 读失败；尝试完成上一次 AP 读的结果。
          // Current AP read failed; try to complete the previous AP read result.
          if (resp_off + 4u <= resp_cap)
          {
            uint32_t rdbuff_data = 0u;
            LibXR::Debug::SwdProtocol::Ack rdbuff_ack =
                LibXR::Debug::SwdProtocol::Ack::PROTOCOL;
            const ErrorCode ec2 = CompletePendingApReadFast(rdbuff_data, rdbuff_ack);
            const uint8_t v2 = MapAckToDapResp(rdbuff_ack);

            if (v2 == LibXR::USB::DapLinkV2Def::DAP_TRANSFER_OK && ec2 == ErrorCode::OK)
            {
              StoreU32Le(&resp[resp_off], rdbuff_data);
              resp_off = static_cast<uint16_t>(resp_off + 4u);
              // done 包含传输 [0..i-1]。
              // done includes transfer [0..i-1]
              done = static_cast<uint16_t>(i);
            }
            else
            {
              xresp = v2;
              if (ec2 != ErrorCode::OK)
              {
                xresp |= LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
              }
              goto out_ap_read;  // NOLINT
            }
          }

          xresp = CUR;
          if (ec != ErrorCode::OK)
          {
            xresp |= LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
          }
          goto out_ap_read;  // NOLINT
        }

        StoreU32Le(&resp[resp_off], ap_read_resp.rdata);
        resp_off = static_cast<uint16_t>(resp_off + 4u);
        done = static_cast<uint16_t>(i);  // completed [0..i-1]
        xresp = CUR;
      }

      // 通过 DP RDBUFF 完成最后一次 posted AP 读。
      // Complete the final posted AP read by DP RDBUFF.
      {
        uint32_t rdbuff_data = 0u;
        LibXR::Debug::SwdProtocol::Ack rdbuff_ack =
            LibXR::Debug::SwdProtocol::Ack::PROTOCOL;
        const ErrorCode ec2 = CompletePendingApReadFast(rdbuff_data, rdbuff_ack);
        const uint8_t v2 = MapAckToDapResp(rdbuff_ack);

        xresp = v2;
        if (v2 != LibXR::USB::DapLinkV2Def::DAP_TRANSFER_OK)
        {
          goto out_ap_read;  // NOLINT
        }
        if (ec2 != ErrorCode::OK)
        {
          xresp |= LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
          goto out_ap_read;  // NOLINT
        }

        StoreU32Le(&resp[resp_off], rdbuff_data);
        resp_off = static_cast<uint16_t>(resp_off + 4u);
        done = count;
      }

    out_ap_read:
      StoreU16Le(&resp[1], done);
      resp[3] = xresp;
      out_len = resp_off;
      return ErrorCode::OK;
    }
  }

  ErrorCode HandleJtagTransfer(const uint8_t* req, uint16_t req_len, uint8_t* resp,
                               uint16_t resp_cap, uint16_t& out_len)
  {
    out_len = 0u;
    if (!req || !resp || resp_cap < 3u)
    {
      return ErrorCode::ARG_ERR;
    }

    resp[0] = ToU8(LibXR::USB::DapLinkV2Def::CommandId::TRANSFER);
    resp[1] = 0u;
    resp[2] = 0u;
    uint16_t resp_off = 3u;

    if (req_len < 3u || jtag_ == nullptr || jtag_chain_.count == 0u)
    {
      resp[2] = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
      out_len = 3u;
      return ErrorCode::OK;
    }

    // 传输开始时清掉旧的中止标志（一个在没有传输时到来的中止不应中止下一条传输）；
    // 之后到来的中止在传输执行中置位。
    // Clear a stale abort flag when a transfer starts (an abort that arrives while no
    // transfer runs must not abort the next one); an abort that arrives later sets it
    // during the execution.
    transfer_abort_.store(false, std::memory_order_release);

    const uint8_t TAP_INDEX = req[1];
    if (TAP_INDEX >= jtag_chain_.count)
    {
      resp[2] = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
      out_len = 3u;
      return ErrorCode::OK;
    }

    jtag_chain_.index = TAP_INDEX;
    LibXR::Debug::JtagProtocol::UpdateChainCache(jtag_chain_);
    LibXR::Debug::JtagDp dp(*jtag_, &jtag_chain_);

    const uint8_t COUNT = req[2];
    uint16_t req_off = 3u;

    auto push_u32 = [&](uint32_t v) -> bool
    {
      if (resp_off + 4u > resp_cap)
      {
        return false;
      }
      Memory::FastCopy(&resp[resp_off], &v, sizeof(v));
      resp_off = static_cast<uint16_t>(resp_off + 4u);
      return true;
    };

    auto push_timestamp = [&]() -> bool
    {
      const uint32_t t = static_cast<uint32_t>(LibXR::Timebase::GetMicroseconds());
      return push_u32(t);
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
        const ErrorCode ec = dp.DpReadTxn(addr2b, val, ack);
        idle_after_attempt();
        if (ack != LibXR::Debug::JtagProtocol::Ack::WAIT || retry == 0u ||
            transfer_abort_.load(std::memory_order_acquire))
        {
          return ec;
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
        const ErrorCode ec = dp.DpWriteTxn(addr2b, val, ack);
        idle_after_attempt();
        if (ack != LibXR::Debug::JtagProtocol::Ack::WAIT || retry == 0u ||
            transfer_abort_.load(std::memory_order_acquire))
        {
          return ec;
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
        const ErrorCode ec = dp.ApReadTxn(addr2b, val, ack);
        idle_after_attempt();
        if (ack != LibXR::Debug::JtagProtocol::Ack::WAIT || retry == 0u ||
            transfer_abort_.load(std::memory_order_acquire))
        {
          return ec;
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
        const ErrorCode ec = dp.ApWriteTxn(addr2b, val, ack);
        idle_after_attempt();
        if (ack != LibXR::Debug::JtagProtocol::Ack::WAIT || retry == 0u ||
            transfer_abort_.load(std::memory_order_acquire))
        {
          return ec;
        }
        --retry;
      }
    };

    auto check_posted_ap_write_jtag =
        [&](LibXR::Debug::JtagProtocol::Ack& ack) -> ErrorCode
    {
      uint32_t ctrl_stat = 0u;
      return dp_read_retry(
          static_cast<uint8_t>(LibXR::Debug::SwdProtocol::DpReadReg::CTRL_STAT),
          ctrl_stat, ack);
    };

    uint8_t response_count = 0u;
    uint8_t response_value = 0u;
    bool pending_ap_write = false;

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

    auto check_posted_ap_write_if_pending = [&]() -> bool
    {
      if (!pending_ap_write)
      {
        return true;
      }

      LibXR::Debug::JtagProtocol::Ack ack = LibXR::Debug::JtagProtocol::Ack::PROTOCOL;
      const ErrorCode ec = check_posted_ap_write_jtag(ack);
      const uint8_t v = MapAckToDapResp(ack);

      if (v != LibXR::USB::DapLinkV2Def::DAP_TRANSFER_OK)
      {
        response_value = v;
        return false;
      }
      if (ec != ErrorCode::OK)
      {
        response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
        return false;
      }

      pending_ap_write = false;
      return true;
    };

    auto should_check_pending_ap_write_at_tail = [&]() -> bool
    {
      return response_value == LibXR::USB::DapLinkV2Def::DAP_TRANSFER_OK ||
             response_value ==
                 static_cast<uint8_t>(LibXR::USB::DapLinkV2Def::DAP_TRANSFER_OK |
                                      LibXR::USB::DapLinkV2Def::DAP_TRANSFER_MISMATCH);
    };

    for (uint32_t i = 0; i < COUNT; ++i)
    {
      if (req_off >= req_len)
      {
        response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
        break;
      }

      const uint8_t RQ = req[req_off++];
      const bool AP = LibXR::USB::DapLinkV2Def::req_is_ap(RQ);
      const bool RNW = LibXR::USB::DapLinkV2Def::req_is_read(RQ);
      const uint8_t ADDR2B = LibXR::USB::DapLinkV2Def::req_addr2b(RQ);
      const bool TS = LibXR::USB::DapLinkV2Def::req_need_timestamp(RQ);
      const bool MATCH_VALUE =
          ((RQ & LibXR::USB::DapLinkV2Def::DAP_TRANSFER_MATCH_VALUE) != 0u);
      const bool MATCH_MASK =
          ((RQ & LibXR::USB::DapLinkV2Def::DAP_TRANSFER_MATCH_MASK) != 0u);

      if (TS && (MATCH_VALUE || MATCH_MASK))
      {
        response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
        break;
      }

      LibXR::Debug::JtagProtocol::Ack ack = LibXR::Debug::JtagProtocol::Ack::PROTOCOL;
      ErrorCode ec = ErrorCode::OK;

      if (!RNW)
      {
        if (req_off + 4u > req_len)
        {
          response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
          break;
        }

        uint32_t wdata = 0u;
        Memory::FastCopy(&wdata, &req[req_off], sizeof(wdata));
        req_off = static_cast<uint16_t>(req_off + 4u);

        if (MATCH_MASK)
        {
          match_mask_ = wdata;
          response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_OK;
          response_count++;
          continue;
        }

        if (AP)
        {
          ec = ap_write_retry(ADDR2B, wdata, ack);
        }
        else
        {
          ec = dp_write_retry(ADDR2B, wdata, ack);
        }

        response_value = MapAckToDapResp(ack);
        if (response_value != LibXR::USB::DapLinkV2Def::DAP_TRANSFER_OK)
        {
          break;
        }
        if (ec != ErrorCode::OK)
        {
          response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
          break;
        }

        if (TS)
        {
          if (!push_timestamp())
          {
            response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
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
            response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
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
              ec = ap_read_retry(ADDR2B, rdata, ack);
            }
            else
            {
              ec = dp_read_retry(ADDR2B, rdata, ack);
            }

            response_value = MapAckToDapResp(ack);
            if (response_value != LibXR::USB::DapLinkV2Def::DAP_TRANSFER_OK)
            {
              break;
            }
            if (ec != ErrorCode::OK)
            {
              response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
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

          if (response_value != LibXR::USB::DapLinkV2Def::DAP_TRANSFER_OK)
          {
            break;
          }

          if (!matched)
          {
            response_value =
                static_cast<uint8_t>(LibXR::USB::DapLinkV2Def::DAP_TRANSFER_OK |
                                     LibXR::USB::DapLinkV2Def::DAP_TRANSFER_MISMATCH);
            break;
          }

          response_count++;
          continue;
        }

        if (!AP)
        {
          if (!check_posted_ap_write_if_pending())
          {
            break;
          }

          uint32_t rdata = 0u;
          ec = dp_read_retry(ADDR2B, rdata, ack);

          response_value = MapAckToDapResp(ack);
          if (response_value != LibXR::USB::DapLinkV2Def::DAP_TRANSFER_OK)
          {
            break;
          }
          if (ec != ErrorCode::OK)
          {
            response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
            break;
          }

          if (!ensure_space(bytes_for_read(TS)))
          {
            response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
            break;
          }

          if (!emit_read_with_ts(TS, rdata))
          {
            response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
            break;
          }

          response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_OK;
          continue;
        }

        uint32_t rdata = 0u;
        if (!check_posted_ap_write_if_pending())
        {
          break;
        }

        ec = ap_read_retry(ADDR2B, rdata, ack);

        response_value = MapAckToDapResp(ack);
        if (response_value != LibXR::USB::DapLinkV2Def::DAP_TRANSFER_OK)
        {
          break;
        }
        if (ec != ErrorCode::OK)
        {
          response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
          break;
        }

        if (!ensure_space(bytes_for_read(TS)))
        {
          response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
          break;
        }

        if (!emit_read_with_ts(TS, rdata))
        {
          response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
          break;
        }

        response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_OK;
      }

      if (transfer_abort_.load(std::memory_order_acquire))
      {
        transfer_abort_.store(false, std::memory_order_release);
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

  ErrorCode HandleJtagTransferBlock(const uint8_t* req, uint16_t req_len, uint8_t* resp,
                                    uint16_t resp_cap, uint16_t& out_len)
  {
    if (!req || !resp || resp_cap < 4u)
    {
      out_len = 0u;
      return ErrorCode::ARG_ERR;
    }

    resp[0] = ToU8(LibXR::USB::DapLinkV2Def::CommandId::TRANSFER_BLOCK);
    resp[1] = 0u;
    resp[2] = 0u;
    resp[3] = 0u;
    out_len = 4u;

    if (req_len < 5u || jtag_ == nullptr || jtag_chain_.count == 0u)
    {
      resp[3] = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
      return ErrorCode::OK;
    }

    // 传输开始时清掉旧的中止标志（一个在没有传输时到来的中止不应中止下一条传输）；
    // 之后到来的中止在传输执行中置位。
    // Clear a stale abort flag when a transfer starts (an abort that arrives while no
    // transfer runs must not abort the next one); an abort that arrives later sets it
    // during the execution.
    transfer_abort_.store(false, std::memory_order_release);

    const uint8_t TAP_INDEX = req[1];
    if (TAP_INDEX >= jtag_chain_.count)
    {
      resp[3] = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
      return ErrorCode::OK;
    }

    jtag_chain_.index = TAP_INDEX;
    LibXR::Debug::JtagProtocol::UpdateChainCache(jtag_chain_);

    uint16_t count = 0u;
    Memory::FastCopy(&count, &req[2], sizeof(count));
    const uint8_t DAP_RQ = req[4];

    if ((DAP_RQ & (LibXR::USB::DapLinkV2Def::DAP_TRANSFER_MATCH_VALUE |
                   LibXR::USB::DapLinkV2Def::DAP_TRANSFER_MATCH_MASK)) != 0u)
    {
      resp[3] = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
      return ErrorCode::NOT_SUPPORT;
    }
    if (LibXR::USB::DapLinkV2Def::req_need_timestamp(DAP_RQ))
    {
      resp[3] = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
      return ErrorCode::NOT_SUPPORT;
    }

    if (count == 0u)
    {
      const uint16_t DONE0 = 0u;
      Memory::FastCopy(&resp[1], &DONE0, sizeof(DONE0));
      resp[3] = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_OK;
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
        const ErrorCode ec = dp.DpReadTxn(addr2b, val, ack);
        idle_after_attempt();
        if (ack != LibXR::Debug::JtagProtocol::Ack::WAIT || retry == 0u ||
            transfer_abort_.load(std::memory_order_acquire))
        {
          return ec;
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
        const ErrorCode ec = dp.DpWriteTxn(addr2b, val, ack);
        idle_after_attempt();
        if (ack != LibXR::Debug::JtagProtocol::Ack::WAIT || retry == 0u ||
            transfer_abort_.load(std::memory_order_acquire))
        {
          return ec;
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
        const ErrorCode ec = dp.ApReadTxn(addr2b, val, ack);
        idle_after_attempt();
        if (ack != LibXR::Debug::JtagProtocol::Ack::WAIT || retry == 0u ||
            transfer_abort_.load(std::memory_order_acquire))
        {
          return ec;
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
        const ErrorCode ec = dp.ApWriteTxn(addr2b, val, ack);
        idle_after_attempt();
        if (ack != LibXR::Debug::JtagProtocol::Ack::WAIT || retry == 0u ||
            transfer_abort_.load(std::memory_order_acquire))
        {
          return ec;
        }
        --retry;
      }
    };

    auto check_posted_ap_write_jtag =
        [&](LibXR::Debug::JtagProtocol::Ack& ack) -> ErrorCode
    {
      uint32_t ctrl_stat = 0u;
      return dp_read_retry(
          static_cast<uint8_t>(LibXR::Debug::SwdProtocol::DpReadReg::CTRL_STAT),
          ctrl_stat, ack);
    };

    const bool AP = LibXR::USB::DapLinkV2Def::req_is_ap(DAP_RQ);
    const bool RNW = LibXR::USB::DapLinkV2Def::req_is_read(DAP_RQ);
    const uint8_t ADDR2B = LibXR::USB::DapLinkV2Def::req_addr2b(DAP_RQ);

    uint16_t done = 0u;
    uint8_t xresp = 0u;
    uint16_t req_off = 5u;
    uint16_t resp_off = 4u;

    if (!RNW)
    {
      for (uint32_t i = 0; i < count; ++i)
      {
        if (req_off + 4u > req_len)
        {
          xresp |= LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
          break;
        }

        uint32_t wdata = 0u;
        Memory::FastCopy(&wdata, &req[req_off], sizeof(wdata));
        req_off = static_cast<uint16_t>(req_off + 4u);

        LibXR::Debug::JtagProtocol::Ack ack = LibXR::Debug::JtagProtocol::Ack::PROTOCOL;
        ErrorCode ec =
            AP ? ap_write_retry(ADDR2B, wdata, ack) : dp_write_retry(ADDR2B, wdata, ack);

        xresp = MapAckToDapResp(ack);
        if (xresp == 0u)
        {
          xresp |= LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
          break;
        }

        if (ack != LibXR::Debug::JtagProtocol::Ack::OK)
        {
          break;
        }

        if (ec != ErrorCode::OK)
        {
          xresp |= LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
          break;
        }

        done = static_cast<uint16_t>(i + 1u);
      }

      if (AP && xresp == LibXR::USB::DapLinkV2Def::DAP_TRANSFER_OK && done > 0u)
      {
        LibXR::Debug::JtagProtocol::Ack ack = LibXR::Debug::JtagProtocol::Ack::PROTOCOL;
        const ErrorCode ec = check_posted_ap_write_jtag(ack);
        const uint8_t v = MapAckToDapResp(ack);
        if (v != LibXR::USB::DapLinkV2Def::DAP_TRANSFER_OK)
        {
          xresp = v;
        }
        else if (ec != ErrorCode::OK)
        {
          xresp |= LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
        }
      }

      Memory::FastCopy(&resp[1], &done, sizeof(done));
      resp[3] = xresp;
      out_len = resp_off;
      return ErrorCode::OK;
    }

    for (uint32_t i = 0; i < count; ++i)
    {
      if (resp_off + 4u > resp_cap)
      {
        xresp |= LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
        break;
      }

      LibXR::Debug::JtagProtocol::Ack ack = LibXR::Debug::JtagProtocol::Ack::PROTOCOL;
      ErrorCode ec = ErrorCode::OK;
      uint32_t rdata = 0u;

      if (AP)
      {
        ec = ap_read_retry(ADDR2B, rdata, ack);
      }
      else
      {
        ec = dp_read_retry(ADDR2B, rdata, ack);
      }

      xresp = MapAckToDapResp(ack);
      if (xresp == 0u)
      {
        xresp |= LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
        break;
      }

      if (ack != LibXR::Debug::JtagProtocol::Ack::OK)
      {
        break;
      }

      if (ec != ErrorCode::OK)
      {
        xresp |= LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
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

  ErrorCode HandleJtagWriteAbort(const uint8_t* req, uint16_t req_len, uint8_t* resp,
                                 uint16_t resp_cap, uint16_t& out_len)
  {
    if (!req || !resp || resp_cap < 2u)
    {
      out_len = 0u;
      return ErrorCode::ARG_ERR;
    }

    resp[0] = ToU8(LibXR::USB::DapLinkV2Def::CommandId::WRITE_ABORT);
    resp[1] = DAP_ERROR;
    out_len = 2u;

    if (req_len < 6u || jtag_ == nullptr || jtag_chain_.count == 0u)
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
    const ErrorCode ec = dp.WriteAbort(flags, ack);
    if (ec == ErrorCode::OK && ack == LibXR::Debug::JtagProtocol::Ack::OK)
    {
      resp[1] = DAP_OK;
    }
    return ErrorCode::OK;
  }

 private:
  void DriveReset(bool release)
  {
    last_nreset_level_high_ = release;

    // 无论 GPIO 是否存在都更新影子值 / Update shadow regardless of whether GPIO exists
    if (release)
    {
      swj_shadow_ |= LibXR::USB::DapLinkV2Def::DAP_SWJ_NRESET;
    }
    else
    {
      swj_shadow_ = static_cast<uint8_t>(
          swj_shadow_ & static_cast<uint8_t>(~LibXR::USB::DapLinkV2Def::DAP_SWJ_NRESET));
    }

    if (nreset_gpio_ != nullptr)
    {
      (void)nreset_gpio_->Write(release);
    }
  }

  void ResetJtagChainState()
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

 private:
  static constexpr uint16_t DEFAULT_DAP_PACKET_SIZE = DefaultDapPacketSize;
  static constexpr uint8_t PACKET_COUNT_ADVERTISED = AdvertisedPacketCount;
  static constexpr uint8_t PACKET_COUNT_EFFECTIVE =
      (PACKET_COUNT_ADVERTISED > 4u) ? 4u : PACKET_COUNT_ADVERTISED;
  /**
   * @brief OpenOCD 安全的 DAP 包长上限 / OpenOCD-safe upper bound of the DAP packet size
   * @note OpenOCD 0.12 对 CMD_DAP_TFER 按 (packet_size - 4) / 5 算队列长度，
   *       transfer_count 是 u8，所以主机可见的包长不超过 255 * 5 + 4。
   *       OpenOCD 0.12 computes the CMD_DAP_TFER queue length as (packet_size - 4) / 5
   *       and transfer_count is a u8, so the packet size the host sees stays at or below
   *       255 * 5 + 4.
   */
  static constexpr uint16_t OPENOCD_SAFE_DAP_PACKET_SIZE =
      static_cast<uint16_t>((255u * 5u) + 4u);
  static constexpr uint8_t JTAG_MAX_DEVICES = 16u;  ///< JTAG 链上限 / JTAG chain limit
  /// 主机可见的 DAP 包长上限 / Upper bound of the DAP packet size the host sees
  static constexpr uint16_t MAX_DAP_PACKET_SIZE = MaxDapPacketSize;
  static constexpr uint16_t RESP_SLOT_SIZE = MAX_DAP_PACKET_SIZE;  ///< 暂存槽 / Staging
  static_assert(PACKET_COUNT_ADVERTISED > 0u, "PACKET_COUNT_ADVERTISED must be > 0");
  static_assert(MAX_DAP_PACKET_SIZE >= DEFAULT_DAP_PACKET_SIZE,
                "MAX_DAP_PACKET_SIZE must be >= DEFAULT_DAP_PACKET_SIZE");
  static_assert(MAX_DAP_PACKET_SIZE <= OPENOCD_SAFE_DAP_PACKET_SIZE,
                "MAX_DAP_PACKET_SIZE exceeds OpenOCD-safe limit");

  /// 批量端点的配置；包长上限为高速批量 / Bulk endpoint configurations; the bound is
  /// high-speed bulk
  static constexpr Endpoint::Config DATA_OUT_CONFIG{Endpoint::Direction::OUT,
                                                    Endpoint::Type::BULK, 512U};
  static constexpr Endpoint::Config DATA_IN_CONFIG{Endpoint::Direction::IN,
                                                   Endpoint::Type::BULK, 512U};
  /// 厂商自定义接口类 / Vendor-specific interface class
  static constexpr uint8_t VENDOR_SPECIFIC = 0xFFU;

  /// 暂存的响应 / A staged response
  struct ResponseSlot
  {
    uint16_t len = 0u;    ///< 响应长度 / Response length
    uint32_t epoch = 0u;  ///< 所属命令的 epoch / Epoch of the command it answers
    uint8_t payload[RESP_SLOT_SIZE] = {};  ///< 响应 / Response
  };

  /// 一条收齐的命令 / One fully received command
  struct CommandSlot
  {
    uint16_t len = 0u;    ///< 命令长度 / Command length
    uint32_t epoch = 0u;  ///< 收到时的 epoch_ / epoch_ when it was received
    uint8_t data[MAX_DAP_PACKET_SIZE] = {};  ///< 命令 / Command
  };

  /// 命令池满时留住的 OUT 槽 / An OUT slot held while the command pool is full
  struct HeldPacket
  {
    Endpoint::Slot slot;  ///< 借来的槽 / The lent slot
    uint16_t size = 0u;   ///< 收到的字节数 / Bytes received
  };

  LIBXR_PACKED_BEGIN
  /// 本类的描述符块：接口和两个端点 / This class's descriptor block: the interface and
  /// two endpoints
  struct DescriptorBlock
  {
    InterfaceDescriptor intf;   ///< 接口描述符 / Interface descriptor
    EndpointDescriptor ep_out;  ///< OUT 端点描述符 / OUT endpoint descriptor
    EndpointDescriptor ep_in;   ///< IN 端点描述符 / IN endpoint descriptor
  };
  LIBXR_PACKED_END

  /// 不指向任何池槽 / Refers to no pool slot
  static constexpr size_t SLOT_NONE = static_cast<size_t>(-1);
  /**
   * @brief 命令池的槽数 / Number of command pool slots
   * @note 等于实际的在途命令数：大命令执行时后面的小命令各占一个槽，下一条大命令仍有槽
   *       可收。
   *       As many as commands in flight: while a big command runs, the small ones
   *       behind it take one slot each and the next big one still finds a slot.
   */
  static constexpr size_t CMD_SLOT_COUNT = PACKET_COUNT_EFFECTIVE;
  /// 留住的 OUT 槽的环：最多是全部槽 / Ring of the held OUT slots: every slot at most
  static constexpr uint8_t HELD_RING_SIZE = Endpoint::MAX_SLOTS;
  /**
   * @brief 排队接收大小的环 / Ring of the queued receive sizes
   * @note Stall、Flush 逐个交付 ABORTED 时，停下的槽已空出，回调里新挂的接收排在还没
   *       回调的旧请求之后，所以最多 2 × 槽数 - 1 项。
   *       While Stall or Flush delivers its ABORTED completions one by one, the stopped
   *       slots are already free, and receives armed in a callback queue behind old
   *       requests whose callbacks are still due: at most 2 x slots - 1 entries.
   */
  static constexpr uint8_t ARMED_RING_SIZE = 2U * Endpoint::MAX_SLOTS;

  /// @name 构造时固定 / Fixed at construction
  /// @{
  SwdPort& swd_;                        ///< SWD 链路 / SWD link
  LibXR::GPIO* nreset_gpio_ = nullptr;  ///< 可选的 nRESET / Optional nRESET
  const char* interface_string_;        ///< 接口字符串 / Interface string
  Endpoint::EPNumber data_in_num_;      ///< 批量 IN 端点号 / Bulk IN number
  Endpoint::EPNumber data_out_num_;     ///< 批量 OUT 端点号 / Bulk OUT number
  Callback<const Endpoint::Completion&> data_out_cb_;  ///< OUT 回调 / OUT callback
  Callback<const Endpoint::Completion&> data_in_cb_;   ///< IN 回调 / IN callback
  /// @}

  /// @name 启动前设置，Step 线程读 / Set before start, read by the Step thread
  /// @{
  LibXR::Debug::Jtag* jtag_ = nullptr;     ///< JTAG 链路 / JTAG link
  CommandHookFun command_hook_ = nullptr;  ///< 命令钩子 / Command hook
  void* command_hook_ctx_ = nullptr;       ///< 钩子上下文 / Hook context
  /// DAP_Info 字符串 / DAP_Info strings
  InfoStrings info_{"XRobot", "DAPLinkV2", "00000001", "2.0.0", "XRUSB",
                    "XRDAP",  "XRobot",    "DAP_DEMO", "0.1.0"};
  /// @}

  /// @name owner 与 Step 线程之间 / Between the owner and the Step thread
  /// @{
  /// 命令池：owner Acquire、拷入、Publish；Step Take、执行、Release / Command pool: the
  /// owner acquires, fills and publishes; Step takes, executes and releases
  LibXR::SlotPool<CommandSlot> cmd_pool_{CMD_SLOT_COUNT};
  /**
   * @brief 暂存池：Step Acquire、写入、Publish；owner Take、拷进 IN 槽提交、Release /
   *        Staging pool: Step acquires, writes and publishes; the owner takes, copies
   *        into an IN slot, submits and releases
   */
  LibXR::SlotPool<ResponseSlot> resp_pool_{PACKET_COUNT_EFFECTIVE};
  /**
   * @brief 已发布、还没进端点队列的暂存响应数；Step 加，owner 减；为 0 时 Step 才直接提交
   *        / Staged responses published but not yet queued; Step adds, the owner
   *        subtracts; Step submits directly only at zero
   */
  std::atomic<uint32_t> staged_outstanding_{0u};
  /// 每发布一条命令、每次绑定解绑发一次；Step 等它 / Posted per published command and
  /// per bind or unbind; Step waits on it
  LibXR::Semaphore cmd_sem_{0};
  /// 每放出一个暂存槽、每次解绑发一次；Step 等暂存槽时等它 / Posted per freed staging
  /// slot and per unbind; Step waits on it for a staging slot
  LibXR::Semaphore slot_sem_{0};
  /// 会话号：绑定和解绑时 owner 加一，Step 读 / Session number: the owner adds one at
  /// bind and unbind, Step reads it
  std::atomic<uint32_t> epoch_{0u};
  /// 通告的 DAP 包长：owner 在绑定时写，两边读 / Announced DAP packet size: written by
  /// the owner at bind, read by both
  std::atomic<uint16_t> dap_packet_size_{MAX_DAP_PACKET_SIZE};
  /// 绑定的 IN 端点：owner 写，Step 借槽时读 / Bound IN endpoint: written by the owner,
  /// read by Step to borrow slots
  std::atomic<Endpoint*> in_ep_ref_{nullptr};
  /// DAP_TransferAbort：owner 或 Step 置，Step 读、清 / DAP_TransferAbort: set by the
  /// owner or Step, read and cleared by Step
  std::atomic<bool> transfer_abort_{false};
  /// @}

  /// @name 仅 owner / Owner only
  /// @{
  Endpoint* data_in_ = nullptr;   ///< 批量 IN，绑定时非空 / Bulk IN, non-null if bound
  Endpoint* data_out_ = nullptr;  ///< 批量 OUT，绑定时非空 / Bulk OUT, non-null if bound
  DescriptorBlock desc_block_{};  ///< WriteDescriptors 写出的块 / Written block
  size_t rx_slot_ = SLOT_NONE;    ///< 正在拼装的命令槽 / Command slot being assembled
  size_t send_idx_ = SLOT_NONE;   ///< 已取出、等 IN 槽的暂存响应 / Staged response taken,
                                  ///< waiting for an IN slot
  /// 命令池满时留住的槽，按到达顺序的环 / Slots held while the pool is full, a ring in
  /// arrival order
  HeldPacket held_[HELD_RING_SIZE] = {};
  uint8_t held_head_ = 0u;  ///< 最早留住的一项 / Oldest held entry
  uint8_t held_n_ = 0u;     ///< held_ 的个数 / Entries in held_
  /// 排队接收的大小，按挂出顺序的环 / Queued receive sizes, a ring in arming order
  size_t armed_sz_[ARMED_RING_SIZE] = {};
  uint8_t armed_head_ = 0u;    ///< 最早挂出的一项 / Oldest armed entry
  uint8_t armed_n_ = 0u;       ///< 排队接收的个数 / Queued receives
  uint16_t rx_off_ = 0u;       ///< 拼装中命令的已收字节 / Bytes of the command so far
  uint16_t known_total_ = 0u;  ///< 由内容得知的命令总长，0 未知 / Known total, 0 unknown
  /// 上一条命令以整包结束时的字节数；其后的全零整包视为补零 / Bytes of the previous
  /// command when it ended on a whole packet; a following all-zero packet is padding
  uint16_t pad_base_ = 0u;
  uint16_t pad_skip_ = 0u;  ///< 本次传输剩下的补零 / Padding this transfer still has
  /// @}

  /// @name 仅 Step 线程 / Step thread only
  /// @{
  size_t cur_cmd_ = SLOT_NONE;     ///< 持有的命令槽 / Command slot held
  size_t cur_resp_ = SLOT_NONE;    ///< 持有的暂存槽 / Staging slot held
  Endpoint::Slot cur_in_{};        ///< 借来直接写响应的 IN 槽 / IN slot for the response
  Endpoint* cur_in_ep_ = nullptr;  ///< cur_in_ 所属端点 / Endpoint of cur_in_
  uint32_t cur_epoch_ = 0u;        ///< 持有命令的 epoch / Epoch of the command held
  uint32_t session_epoch_ = 0u;    ///< 已落实的 epoch / Epoch applied to the session
  LibXR::Debug::JtagProtocol::ChainConfig jtag_chain_{};  ///< JTAG 链 / JTAG chain
  uint8_t jtag_ir_len_[JTAG_MAX_DEVICES] = {};  ///< JTAG IR 长度 / JTAG IR lengths
  /// SWJ 引脚电平的影子 / Shadow of the SWJ pin levels
  uint8_t swj_shadow_ = static_cast<uint8_t>(LibXR::USB::DapLinkV2Def::DAP_SWJ_SWDIO_TMS |
                                             LibXR::USB::DapLinkV2Def::DAP_SWJ_NRESET);
  bool last_nreset_level_high_ = true;  ///< 上一次 nRESET 电平 / Last nRESET level
  LibXR::USB::DapLinkV2Def::State dap_state_{};  ///< DAP 状态 / DAP state
  uint32_t swj_clock_hz_ = 1000000u;             ///< SWJ 时钟 / SWJ clock (Hz)
  uint32_t match_mask_ = 0xFFFFFFFFu;            ///< 匹配掩码 / Match mask
  /// @}
};

}  // namespace LibXR::USB
