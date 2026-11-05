#pragma once

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "daplink_v2_def.hpp"
#include "debug/jtag_dp.hpp"
#include "debug/swd.hpp"
#include "dev_core.hpp"
#include "gpio.hpp"
#include "libxr_def.hpp"
#include "libxr_mem.hpp"
#include "libxr_type.hpp"
#include "semaphore.hpp"
#include "structure/slot_pool.hpp"
#include "thread.hpp"
#include "timebase.hpp"
#include "timer.hpp"
#include "usb/core/desc_cfg.hpp"

namespace LibXR::USB
{
#ifndef LIBXR_USB_DAPLINK_V2_BUILD_CONFIG_DEFINED
#define LIBXR_USB_DAPLINK_V2_BUILD_CONFIG_DEFINED
namespace DapLinkV2BuildConfig
{
static constexpr bool kEnableJtag = true;
}
#endif

/**
 * @brief CMSIS-DAP v2（Bulk）USB 类，支持 SWD 和 JTAG（daplink_v2_profile_swd.hpp
 *        可关闭 JTAG），可选 nRESET 控制 / CMSIS-DAP v2 (Bulk) USB class with SWD and
 *        JTAG (JTAG can be disabled with daplink_v2_profile_swd.hpp), optional
 *        nRESET control.
 *
 *
 * @tparam SwdPort SWD link type
 */
template <typename SwdPort, uint16_t DefaultDapPacketSize = 512u,
          uint8_t AdvertisedPacketCount = 8u, uint16_t MaxDapPacketSize = 1024u,
          uint16_t QueuedRequestBufferSize = 2048u, uint16_t QueuedCommandCountMax = 255u>
class DapLinkV2Class : public DeviceClass
{
 public:
  static constexpr bool JTAG_ENABLED = DapLinkV2BuildConfig::kEnableJtag;
  static constexpr const char* DEFAULT_INTERFACE_STRING = "CMSIS-DAP v2";

  /**
   * @brief Info string set
   */
  struct InfoStrings
  {
    const char* vendor = nullptr;        ///< Vendor string
    const char* product = nullptr;       ///< Product string
    const char* serial = nullptr;        ///< Serial string
    const char* firmware_ver = nullptr;  ///< Firmware version

    const char* device_vendor = nullptr;   ///< Device vendor
    const char* device_name = nullptr;     ///< Device name
    const char* board_vendor = nullptr;    ///< Board vendor
    const char* board_name = nullptr;      ///< Board name
    const char* product_fw_ver = nullptr;  ///< Product FW version
  };

 public:
  /**
   * @brief Constructor
   *
   * @param data_in_ep_num  Bulk IN endpoint number (required)
   * @param data_out_ep_num Bulk OUT endpoint number (required)
   * @param swd_link        SWD link reference
   * @param nreset_gpio     Optional nRESET GPIO
   *
   */
  explicit DapLinkV2Class(Endpoint::EPNumber data_in_ep_num,
                          Endpoint::EPNumber data_out_ep_num, SwdPort& swd_link,
                          LibXR::GPIO* nreset_gpio = nullptr,
                          const char* interface_string = DEFAULT_INTERFACE_STRING)
      : DeviceClass(),
        swd_(swd_link),
        nreset_gpio_(nreset_gpio),
        interface_string_(interface_string),
        data_in_ep_num_(data_in_ep_num),
        data_out_ep_num_(data_out_ep_num)
  {
    (void)swd_.SetClockHz(swj_clock_hz_);

    // 命令由应用线程循环调用 Step() 执行（见 Step）；类自己不起线程，也不起定时任务。
    // Commands run when the application thread calls Step() in a loop (see Step); the
    // class starts no thread and no timer task of its own.
  }

  /**
   * @brief Virtual destructor
   */
  ~DapLinkV2Class() override = default;

  DapLinkV2Class(const DapLinkV2Class&) = delete;
  DapLinkV2Class& operator=(const DapLinkV2Class&) = delete;

 public:
  const char* GetInterfaceString(size_t local_interface_index) const override
  {
    return (local_interface_index == 0u) ? interface_string_ : nullptr;
  }

  /**
   * @brief Set info strings
   * @param info Info string set
   */
  void SetInfoStrings(const InfoStrings& info) { info_ = info; }

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

  /**
   * @brief Get internal state
   * @return State reference
   */
  const LibXR::USB::DapLinkV2Def::State& GetState() const { return dap_state_; }

  /**
   * @brief Whether initialized
   * @return true if initialized
   */
  bool IsInited() const { return inited_; }

  /**
   * @brief Whether IN endpoint is busy
   *
   * @return true
   * @return false
   */
  bool EpInBusy()
  {
    auto ep = ep_data_in_;
    return ep != nullptr && ep->GetState() != Endpoint::State::IDLE;
  }

  /**
   * @brief Whether OUT endpoint is busy
   *
   * @return true
   * @return false
   */
  bool EpOutBusy()
  {
    auto ep = ep_data_out_;
    return ep != nullptr && ep->GetState() != Endpoint::State::IDLE;
  }

  /// 命令钩子：在标准命令分发之前调用。返回 true 表示命令已被处理（包括写入响应）。
  /// Command hook called before the standard dispatch. Returning true means the command
  /// was handled (the response, if any, has been written).
  using CommandHook = bool (*)(void* ctx, const uint8_t* req, uint16_t req_len,
                               uint8_t* resp, uint16_t resp_cap, uint16_t& out_len);

  /// 注册命令钩子，ctx 原样传回。 / Register the command hook; ctx is passed through.
  void SetCommandHook(CommandHook hook, void* ctx = nullptr)
  {
    command_hook_ = hook;
    command_hook_ctx_ = ctx;
  }

  /**
   * @brief 在调用线程里同步执行一条 DAP 命令，供 USB 以外的传输使用（例如 TCP）。
   *        Execute one DAP command synchronously in the caller's thread, for transports
   *        other than USB (for example TCP).
   *
   * 延时类命令（DAP_Delay、ResetTarget、带等待的 SWJ_Pins，以及批里的这些命令）在本
   * 调用内等到完成才返回，所以返回时响应已完整。out_len 为 0 表示该命令没有响应。
   * 本入口不检查、也不排他 USB：启用它之前由调用方负责停掉 USB，二者不能同时驱动同
   * 一个类实例。
   * Delay-type commands (DAP_Delay, ResetTarget, SWJ_Pins with a wait, and such
   * commands inside a batch) are waited for inside this call, so the response is
   * complete on return. out_len of 0 means the command has no response. This entry
   * neither checks nor excludes USB: the caller stops USB before using it, and the two
   * must not drive the same instance at the same time.
   *
   * @param req       请求缓冲 / Request buffer
   * @param req_len   请求长度 / Request length
   * @param resp      响应缓冲 / Response buffer
   * @param resp_cap  响应缓冲容量 / Response buffer capacity
   * @param out_len   响应长度 / Response length
   * @return ErrorCode 命令处理结果 / Command processing result
   */
  ErrorCode ExecuteExternal(const uint8_t* req, uint16_t req_len, uint8_t* resp,
                            uint16_t resp_cap, uint16_t& out_len)
  {
    external_active_ = true;
    external_size_ = 0u;
    uint16_t size = 0u;
    const ErrorCode EC = ProcessOneCommand(false, req, req_len, resp, resp_cap, size);

    // 命令挂起后，续做点把完成的响应交给 CompleteResponse，这里等它走完。
    // A suspended command finishes through CompleteResponse; wait for that here.
    while (pending_action_.load(std::memory_order_acquire) !=
           static_cast<uint8_t>(PendingAction::NONE))
    {
      ServicePendingAction(false);
      if (pending_action_.load(std::memory_order_acquire) ==
          static_cast<uint8_t>(PendingAction::NONE))
      {
        break;
      }
      LibXR::Thread::Sleep(1);
    }
    if (size == 0u)
    {
      size = external_size_;
    }
    external_active_ = false;
    out_len = size;
    return EC;
  }

  /**
   * @brief 在调用线程里做一次 DAP 工作：等一条命令（或续做延时动作）并执行它。
   *        Do one unit of DAP work in the calling thread: wait for a command (or carry a
   *        delayed action on) and execute it.
   *
   * 应用起一个线程循环调用即可（例如 while (true) { dap.Step(); }）。线程应当和 USB 中断
   * 分在不同的核上：同核时小命令慢 6%~15%，尾部延迟最差。SWD 引脚要能在运行 Step 的核上
   * 访问（例如 ESP32-S3 的专用 GPIO 必须在这个核上创建）；libxr
   * 不安排线程在哪个核，由应用 决定。SWD/JTAG 的操作只在 这个线程里发生，USB
   * 回调只接收命令、发送响应，不执行命令。延时类命令（DAP_Delay、 ResetTarget、带等待的
   * SWJ_Pins）在线程里睡到到期，不占用 USB 上下文。 Run it from one application thread,
   * for example while (true) { dap.Step(); }. Put the thread on a different core than the
   * USB interrupt: on the same core small commands are 6%-15% slower and the tail latency
   * is the worst. The SWD pins must be reachable from the core that runs Step (for
   * example ESP32-S3 dedicated GPIO must be created on that core); libxr does not place
   * the thread, the application decides. SWD/JTAG work happens in this thread only; the
   * USB callbacks receive commands and send responses but never execute them. Delay-type
   * commands (DAP_Delay, ResetTarget, SWJ_Pins with a wait) sleep in this thread until
   * they are due and do not occupy the USB context.
   *
   * @param timeout_ms 等待命令的最长时间；0 为不等待 / Longest wait for a command; 0 does
   *                   not wait
   * @return OK 做了一件事（执行一条命令或推进一个延时动作）；TIMEOUT 在期限内没有工作
   *         / OK when something was done (a command executed or a delayed action
   *         advanced); TIMEOUT when no work arrived in time
   */
  ErrorCode Step(uint32_t timeout_ms = UINT32_MAX)
  {
    if (pending_action_.load(std::memory_order_acquire) !=
        static_cast<uint8_t>(PendingAction::NONE))
    {
      RunPendingAction();
      return ErrorCode::OK;
    }
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
          return ErrorCode::OK;  // woken by a bind/unbind, nothing to execute
        }
      }
      cur_cmd_ = slot;
      cur_epoch_ = epoch_.load(std::memory_order_acquire);
    }
    if (!inited_)
    {
      AbandonCurrent();
      return ErrorCode::OK;
    }
    // 没有暂存的响应在排队、又有空闲的 IN 槽时，响应直接写进 IN 槽，
    // 由本线程提交，不经过类服务也不拷贝。否则响应先写进响应池的暂存槽，
    // 不等主机取走前面的响应：SWD 继续往前执行，由 USB 侧在 IN 槽有空时
    // 把暂存的响应拷进去提交。暂存槽用完（客户端在途数超过通告值）时在这里等；
    // 命令继续由本线程持有，超时返回后下一次 Step 接着等。
    // With no staged response waiting and an IN slot free, the response is written
    // straight into the IN slot and submitted by this thread, without the class service
    // and without a copy. Otherwise it is first written into a staging slot of the
    // response pool and does not wait for the host to take earlier responses: SWD goes on
    // executing, and the USB side copies the staged responses into an IN slot and submits
    // them as slots come free. With the staging slots used up (a client with more in
    // flight than advertised) wait here; the command stays with this thread and the next
    // Step goes on waiting after a timeout.
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
      if (!inited_)
      {
        AbandonCurrent();
        return ErrorCode::OK;
      }
    }
    ExecuteCurrent(false);
    // 延时类命令在这里睡到到期并发布响应，一次 Step 就把一条命令做完。
    // A delay-type command sleeps until it is due and publishes its response here, so
    // one Step finishes one command.
    RunPendingAction();
    return ErrorCode::OK;
  }

 protected:
  /**
   * @brief Bind endpoint resources
   * @param endpoint_pool Endpoint pool
   * @param start_itf_num Start interface number
   * @param in_isr Whether called from ISR context
   */
  void BindEndpoints(EndpointPool& endpoint_pool, uint8_t start_itf_num,
                     bool in_isr) override
  {
    inited_ = false;

    interface_num_ = start_itf_num;

    // Allocate endpoints
    auto ans =
        endpoint_pool.Get(ep_data_out_, Endpoint::Direction::OUT, data_out_ep_num_);
    ASSERT(ans == ErrorCode::OK);

    ans = endpoint_pool.Get(ep_data_in_, Endpoint::Direction::IN, data_in_ep_num_);
    ASSERT(ans == ErrorCode::OK);

    // Configure endpoints with the largest packet size the slots and the controller
    // hold at the current speed (0 when there is none). A failed Configure leaves the
    // endpoint in ERROR and the composition rejects the configuration. Both directions
    // work in borrow mode: the OUT slots lend each received packet to the callback, and
    // the response is written straight into an IN slot.
    const uint16_t out_packet = ep_data_out_->LargestPacketSize(Endpoint::Type::BULK);
    const uint16_t in_packet = ep_data_in_->LargestPacketSize(Endpoint::Type::BULK);
    (void)ep_data_out_->Configure(
        {Endpoint::Direction::OUT, Endpoint::Type::BULK, out_packet}, in_isr);
    (void)ep_data_in_->Configure(
        {Endpoint::Direction::IN, Endpoint::Type::BULK, in_packet}, in_isr);

    // 通告给主机的 DAP 包长不能超过 IN 槽：响应原地写在槽里。
    // The DAP packet size announced to the host cannot exceed an IN slot: the response is
    // written in place.
    {
      const size_t in_capacity = ep_data_in_->GetGranted().slot_capacity;
      const uint16_t configured = ConfiguredDapPacketSize();
      dap_packet_size_.store((in_capacity != 0U && in_capacity < configured)
                                 ? static_cast<uint16_t>(in_capacity)
                                 : configured,
                             std::memory_order_release);
    }

    // Hook completion callbacks (they run in the owner and must stay shallow).
    ep_data_out_->SetOnComplete(on_data_out_cb_);
    ep_data_in_->SetOnComplete(on_data_in_cb_);

    // Interface descriptor (vendor specific, 2 endpoints)
    desc_block_.intf = {9,
                        static_cast<uint8_t>(DescriptorType::INTERFACE),
                        interface_num_,
                        0,
                        2,
                        0xFF,  // vendor specific
                        0x00,
                        0x00,
                        GetInterfaceStringIndex(0u)};

    desc_block_.ep_out = {7,
                          static_cast<uint8_t>(DescriptorType::ENDPOINT),
                          static_cast<uint8_t>(ep_data_out_->GetAddress()),
                          static_cast<uint8_t>(Endpoint::Type::BULK),
                          ep_data_out_->MaxPacketSize(),
                          0};

    desc_block_.ep_in = {7,
                         static_cast<uint8_t>(DescriptorType::ENDPOINT),
                         static_cast<uint8_t>(ep_data_in_->GetAddress()),
                         static_cast<uint8_t>(Endpoint::Type::BULK),
                         ep_data_in_->MaxPacketSize(),
                         0};

    SetData(RawData{reinterpret_cast<uint8_t*>(&desc_block_), sizeof(desc_block_)});

    // Runtime defaults
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

    // SWJ shadow defaults: SWDIO=1, nRESET=1, SWCLK=0
    last_nreset_level_high_ = true;
    swj_shadow_ = static_cast<uint8_t>(DapLinkV2Def::DAP_SWJ_SWDIO_TMS |
                                       DapLinkV2Def::DAP_SWJ_NRESET);

    ResetQueuedCommandState();
    ResetJtagChainState();
    ResetChannels();
    CancelPendingAction();

    inited_ = true;
    TopUpReceives(in_isr);
  }

  /**
   * @brief Unbind endpoint resources
   * @param endpoint_pool Endpoint pool
   * @param in_isr Whether called from ISR context
   */
  void UnbindEndpoints(EndpointPool& endpoint_pool, bool in_isr) override
  {
    inited_ = false;
    ResetQueuedCommandState();

    dap_state_.debug_port = LibXR::USB::DapLinkV2Def::DebugPort::DISABLED;
    // 正在执行的传输尽快结束；下一条传输开始时会清掉这个标志。
    // End a transfer in progress soon; the next transfer clears the flag when it starts.
    transfer_abort_.store(true, std::memory_order_release);

    // 借出的槽由解绑作废；在途拷贝传输的 ABORTED 完成在 Close 里同步送达，
    // 此时 inited_ 已为 false，回调入口直接忽略。
    // Lent slots are voided by the unbind; the ABORTED completions of the in-flight
    // copy transfers arrive synchronously inside Close, where the callback entry
    // ignores them because inited_ is already false.
    ResetChannels();
    CancelPendingAction();
    // 叫醒可能在等命令或等 IN 槽的 Step 线程。
    // Wake a Step thread that may be waiting for a command or for an IN slot.
    cmd_sem_.PostFromCallback(in_isr);
    slot_sem_.PostFromCallback(in_isr);

    if (ep_data_in_)
    {
      ep_data_in_->Close(in_isr);
      endpoint_pool.Release(ep_data_in_);
      ep_data_in_ = nullptr;
    }

    if (ep_data_out_)
    {
      ep_data_out_->Close(in_isr);
      endpoint_pool.Release(ep_data_out_);
      ep_data_out_ = nullptr;
    }

    swd_.Close();
    if constexpr (JTAG_ENABLED)
    {
      if (jtag_ != nullptr)
      {
        jtag_->Close();
      }
    }

    // Reset shadow defaults
    last_nreset_level_high_ = true;
    swj_shadow_ = static_cast<uint8_t>(DapLinkV2Def::DAP_SWJ_SWDIO_TMS |
                                       DapLinkV2Def::DAP_SWJ_NRESET);
  }

  /**
   * @brief Number of interfaces contributed
   * @return Interface count
   */
  size_t GetInterfaceCount() override { return 1; }

  /**
   * @brief Whether an IAD is used
   * @return true if this class uses an IAD
   */
  bool HasIAD() override { return false; }

  /**
   * @brief Endpoint ownership check
   * @param ep_addr Endpoint address
   * @return true if owned by this class
   */
  bool OwnsEndpoint(uint8_t ep_addr) const override
  {
    if (!inited_)
    {
      return false;
    }
    return (ep_data_in_ && ep_addr == ep_data_in_->GetAddress()) ||
           (ep_data_out_ && ep_addr == ep_data_out_->GetAddress());
  }

  /**
   * @brief Maximum bytes required in configuration descriptor
   * @return Maximum bytes
   */
  size_t GetMaxConfigSize() override { return sizeof(desc_block_); }

  /**
   * @brief 需要 WinUSB 绑定：返回设备接口 GUID，MS OS 2.0 描述符集由组合设备生成
   *        Wants WinUSB to bind: returns the device interface GUID; the MS OS 2.0
   *        descriptor set is built by the composition
   */
  const char* GetWinUsbInterfaceGuid() const override
  {
    return "{CDB3B5AD-293B-4663-AA36-1AAE46463776}";
  }

  /**
   * @brief 类服务（owner 上下文）：处理已收到的命令，或续做被延时命令（§10.3-a）。
   *        Class service (owner context): process the received command, or resume a
   *        delayed one (§10.3-a).
   */
  void OnService(bool in_isr) override
  {
    if (!inited_)
    {
      return;
    }
    // 服务上下文只做 USB 侧的事：把暂存的响应提交给 IN
    // 槽，处理被留住的包，补挂接收。命令由 Step() 执行。 The service context only does
    // the USB side: submit the staged responses to IN slots, handle packets that were
    // held and re-arm the receives. Commands are executed by Step().
    SendStagedResponses(in_isr);
    DrainHeldPackets(in_isr);
    TopUpReceives(in_isr);
  }

 private:
  // ============================================================================
  // Local helpers (kept with original comments, moved inside the class)
  // ============================================================================

  // Cast enum/integer to uint8_t.
  template <typename E>
  static constexpr uint8_t ToU8(E e)
  {
    return static_cast<uint8_t>(e);
  }

  // CMSIS-DAP status bytes
  static constexpr uint8_t DAP_OK = 0x00u;     ///< DAP_OK
  static constexpr uint8_t DAP_ERROR = 0xFFu;  ///< DAP_ERROR

  // Unknown command response: single byte 0xFF.
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

  // Command status response: <cmd, status>.
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

 private:
  // ============================================================================
  // USB callbacks
  // ============================================================================

  /// 被延时命令的续做动作（§10.3-a）/ Deferred action of a delayed command (§10.3-a)
  enum class PendingAction : uint8_t
  {
    NONE = 0,
    DELAY,       ///< DAP_Delay 到期待完成 / DAP_Delay awaiting its deadline
    RESET_LOW,   ///< nRESET 拉低后的 1 ms / 1 ms after asserting nRESET
    RESET_HIGH,  ///< nRESET 释放后的 1 ms / 1 ms after releasing nRESET
    SWJ_PINS     ///< PinWait 到匹配或超时 / PinWait until a match or the timeout
  };

  /**
   * @brief OUT complete callback static entry
   */
  static void OnDataOutCompleteStatic(bool in_isr, DapLinkV2Class* self,
                                      const Endpoint::Completion& completion)
  {
    if (self && self->inited_)
    {
      self->OnDataOutComplete(in_isr, completion);
    }
  }

  /**
   * @brief IN complete callback static entry
   */
  static void OnDataInCompleteStatic(bool in_isr, DapLinkV2Class* self,
                                     const Endpoint::Completion& completion)
  {
    if (self && self->inited_)
    {
      self->OnDataInComplete(in_isr, completion);
    }
  }

  /**
   * @brief OUT 完成回调：借出一个槽，里面是刚收到的一个（或几个）包。
   *        OUT completion: one slot is lent, holding the packet (or packets) just
   *        received.
   *
   * 包按命令拼装进命令池的槽里，拼完的命令发布给 Step 线程。槽立刻归还，另一个槽一直
   * 在收下一个包，所以两个槽交替，不用等回调处理完。命令池没有空槽时，这个包不拷走，
   * 租约留着（最多两个），端点没有可用的槽就会 NAK，等 Step 线程放出命令槽后再处理。
   * Packets are assembled into slots of the command pool and a finished command is
   * published to the Step thread. The slot is given back at once and the other slot is
   * already receiving the next packet, so the two alternate and the callback's work does
   * not delay the wire. With no free command slot the packet is not copied: the lease is
   * kept (two at most) and the endpoint NAKs for lack of a slot until the Step thread
   * gives a command slot back.
   */
  void OnDataOutComplete(bool in_isr, const Endpoint::Completion& completion)
  {
    if (!inited_ || !ep_data_in_ || !ep_data_out_) return;
    if (armed_n_ > 0u)
    {
      armed_sz_[0] = armed_sz_[1];
      --armed_n_;
    }
    if (completion.result != ErrorCode::OK)
    {
      // ABORTED 只在解绑时出现（回调入口已按 inited_
      // 拦截）；其它错误意味着字节流失去同步， 丢弃正在拼装的命令重新开始。 ABORTED only
      // appears while unbinding (the entry guard filtered it through inited_); any other
      // error means the byte stream lost sync: drop the command being assembled and start
      // over.
      ResetStream();
      TopUpReceives(in_isr);
      return;
    }
    const uint16_t size = static_cast<uint16_t>(completion.actual);
    if (held_n_ > 0u || !ConsumePacket(completion.slot, size, in_isr))
    {
      REQUIRE(held_n_ < 2u);
      held_[held_n_++] = {completion.slot, size};
    }
    TopUpReceives(in_isr);
  }

  /// IN 完成：一个槽回到空闲，叫醒在等 IN 槽的 Step 线程 / IN completion: a slot is free
  /// again, wake a Step thread waiting for one
  void OnDataInComplete(bool in_isr, const Endpoint::Completion& completion)
  {
    UNUSED(completion);
    if (!inited_ || !ep_data_in_ || !ep_data_out_) return;
    SendStagedResponses(in_isr);
  }

  /// 把暂存的响应按发布顺序拷进空闲的 IN 槽并提交。两个 IN 槽都忙时留到下一次 IN 完成。
  /// 拷贝是一个响应的字节数（最多一个 DAP 包），相对一次 SWD 传输和 USB 传输可以忽略。
  /// Copy the staged responses, in the order they were published, into free IN slots and
  /// submit them. With both IN slots busy this waits for the next IN completion. The copy
  /// is one response (a DAP packet at most), negligible against an SWD and a USB
  /// transfer.
  void SendStagedResponses(bool in_isr)
  {
    if (!inited_ || !ep_data_in_) return;
    while (true)
    {
      if (send_idx_ == SLOT_NONE && resp_pool_.Take(send_idx_) != ErrorCode::OK)
      {
        send_idx_ = SLOT_NONE;
        return;
      }
      Endpoint::Slot slot;
      if (ep_data_in_->AcquireSlot(slot, in_isr) != ErrorCode::OK)
      {
        return;  // 两个 IN 槽都忙：下一次 IN 完成再来 / both IN slots busy: next IN
                 // completion
      }
      const ResponseSlot& response = resp_pool_.At(send_idx_);
      const size_t length = std::min<size_t>(response.len, slot.data.size_);
      Memory::FastCopy(slot.data.addr_, response.payload, length);
      const ErrorCode ec =
          ep_data_in_->Submit(slot, length, NeedZlp(*ep_data_in_, length), in_isr);
      if (ec != ErrorCode::OK && ec != ErrorCode::ABORTED)
      {
        ep_data_in_->ReleaseSlot(slot, in_isr);
      }
      resp_pool_.Release(send_idx_);
      send_idx_ = SLOT_NONE;
      // 这条暂存响应已进端点队列：计数归零后 Step 可以直接提交而不越过它。
      // This staged response is in the endpoint queue: once the count is zero Step may
      // submit directly without overtaking it.
      uint32_t staged = staged_outstanding_.load(std::memory_order_acquire);
      while (staged != 0u && !staged_outstanding_.compare_exchange_weak(
                                 staged, staged - 1u, std::memory_order_acq_rel,
                                 std::memory_order_acquire))
      {
      }
      slot_sem_.PostFromCallback(in_isr);  // 一个暂存槽空出来了 / a staging slot is free
    }
  }

  /// 响应恰好等于 DAP 包长时主机读满 DAP 包长缓冲就结束，不补零长包；补了会留下一个主机
  /// 不取的零长包，占住 IN 槽，深流水下把命令流堵死。
  /// A response exactly as long as the DAP packet needs no ZLP: the host's
  /// DAP-packet-sized read completes when the buffer is full. A ZLP would stay armed,
  /// keeping an IN slot occupied, which stalls the command stream under a deep pipeline.
  bool NeedZlp(const Endpoint& ep, size_t length) const
  {
    const size_t mps = ep.MaxPacketSize();
    return (mps != 0U) && (length % mps == 0U) && (length < GetDapPacketSize());
  }

  /// Step 线程：没有暂存响应在排队时借一个空闲 IN 槽来直接写响应；借不到就走暂存槽。
  /// Step thread: borrow a free IN slot to write the response in place when no staged
  /// response is waiting; without one the staging slot is used.
  void TakeInSlot()
  {
    Endpoint* ep = ep_data_in_;
    if (ep == nullptr || staged_outstanding_.load(std::memory_order_acquire) != 0u)
    {
      return;
    }
    Endpoint::Slot slot;
    if (ep->AcquireSlot(slot, false) != ErrorCode::OK)
    {
      return;
    }
    cur_in_ = slot;
    cur_in_ep_ = ep;
  }

  /// Step 线程：提交直接写好的 IN 槽。解绑作废的槽 Submit 得 ABORTED，已回到空闲。
  /// Step thread: submit the IN slot written in place. A slot voided by an unbind gets
  /// ABORTED from Submit and is free again.
  void SubmitInSlot(uint16_t size)
  {
    Endpoint* ep = cur_in_ep_;
    const Endpoint::Slot slot = cur_in_;
    cur_in_ = {};
    cur_in_ep_ = nullptr;
    const ErrorCode ec = ep->Submit(slot, size, NeedZlp(*ep, size), false);
    if (ec != ErrorCode::OK && ec != ErrorCode::ABORTED)
    {
      ep->ReleaseSlot(slot, false);
    }
  }

  /// 解绑/重新绑定：放掉 USB 侧持有的东西，排空命令池里已就绪的槽。Step 线程手里的槽由它
  /// 自己放（见 AbandonCurrent），epoch 让它知道自己的命令已经作废。
  /// Unbind/bind: give back what the USB side holds and drain the ready slots of the
  /// command pool. The slots in the Step thread's hands are given back by it (see
  /// AbandonCurrent); the epoch tells it that its command has been voided.
  void ResetChannels()
  {
    epoch_.fetch_add(1u, std::memory_order_acq_rel);
    ResetStream();
    held_n_ = 0u;
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

  /// 字节流状态归零：放掉正在拼装的命令槽 / Reset the byte-stream state: give back the
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

  /// 一个收到的包（或几个包）：拷进命令槽、判断命令是否收齐，租约在拷完后立刻归还。返回
  /// false 表示命令池没有空槽，什么都没动，租约要留着。
  /// One received chunk: copy it into the command slot, decide whether the command is
  /// complete, and give the lease back right after the copy. Returns false when the
  /// command pool has no free slot: nothing was touched and the lease must be kept.
  bool ConsumePacket(const Endpoint::Slot& slot, uint16_t n, bool in_isr)
  {
    const uint16_t mps = OutChunkSize();
    const uint16_t packet = GetDapPacketSize();
    const uint8_t* data = static_cast<const uint8_t*>(slot.data.addr_);

    // 上一条命令的传输被主机补零到 DAP 包长：补零部分整段丢弃，不需要命令槽，也不受命令池
    // 是否有空槽限制，否则主机的写结束不了。
    // The previous command's transfer was padded by the host to the DAP packet size: the
    // padding is dropped without a command slot and regardless of whether the command
    // pool has a free slot, or the host's write could not finish.
    if (pad_skip_ > 0u && n <= pad_skip_)
    {
      pad_skip_ = static_cast<uint16_t>(pad_skip_ - n);
      ep_data_out_->ReleaseSlot(slot, in_isr);
      return true;
    }
    // 上一条命令以整包结束后紧接着的整包全零不是命令（命令号 0 后面跟信息 ID 0 不是合法
    // 命令）：它是补零，这次传输还剩的字节数记在 pad_skip_ 里。
    // An all-zero whole packet right after a command that ended on a whole packet is not
    // a command (command 0 with info ID 0 is not valid): it is padding, and the bytes
    // this transfer still has are kept in pad_skip_.
    if (rx_off_ == 0u && pad_skip_ == 0u && pad_base_ != 0u && n == mps &&
        IsAllZero(data, n))
    {
      const uint16_t used = static_cast<uint16_t>(pad_base_ + n);
      pad_skip_ = used < packet ? static_cast<uint16_t>(packet - used) : 0u;
      pad_base_ = 0u;
      ep_data_out_->ReleaseSlot(slot, in_isr);
      return true;
    }

    // 从这里起要把字节拷进命令槽；没有空槽就什么都不动。
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
      // Beyond the command slot: the command cannot be kept, drop it and start over.
      ep_data_out_->ReleaseSlot(slot, in_isr);
      ResetStream();
      return true;
    }
    Memory::FastCopy(command.data + rx_off_, data, n);
    rx_off_ = static_cast<uint16_t>(rx_off_ + n);
    ep_data_out_->ReleaseSlot(slot, in_isr);

    // 主机传输的结束：短包、零长包，或收满 DAP 包长。恰好整包结束的命令没有这些提示
    // （OpenOCD 的 bulk 写在整倍数长度时不补零长包），所以整包到来后按命令自己的长度
    // 判断：完整就结束，不完整就只再要缺的那一段。
    // End of the host transfer: a short packet, a zero-length packet, or a full DAP
    // packet. A command that ends exactly on a packet boundary has none of these (the
    // OpenOCD bulk write sends no ZLP for multiples of the packet size), so after a
    // whole packet the command's own length decides: complete ends it, incomplete asks
    // only for the missing part.
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
        // 长度未知的命令（JTAG、厂商命令）：按包长收，靠短包或收满结束。
        // Length unknown (JTAG, vendor commands): receive up to the packet size and rely
        // on a short packet or a full buffer.
        known_total_ = packet;
      }
    }
    if (!done)
    {
      return true;
    }

    if (rx_off_ == 0u)
    {
      cmd_pool_.Release(rx_slot_);  // 空传输（只有零长包）不是命令 / an empty transfer
    }
    else if (command.data[0] == ToU8(LibXR::USB::DapLinkV2Def::CommandId::TRANSFER_ABORT))
    {
      // DAP_TransferAbort 收到即生效、没有响应：不进命令池。Step 线程正在执行传输时下一次
      // 检查标志即中止。
      // DAP_TransferAbort takes effect on reception and has no response: it does not
      // join the command pool. A transfer the Step thread is executing aborts at its
      // next flag check.
      SetTransferAbortFlag(true);
      cmd_pool_.Release(rx_slot_);
    }
    else
    {
      command.len = rx_off_;
      cmd_pool_.Publish(rx_slot_);
      cmd_sem_.PostFromCallback(in_isr);
    }
    rx_slot_ = SLOT_NONE;
    pad_base_ = padded_candidate ? rx_off_ : 0u;
    rx_off_ = 0u;
    known_total_ = 0u;
    return true;
  }

  /// 命令槽放出来之后，处理被留住的包（按到达顺序）/ After a command slot came back,
  /// handle the packets that were held (in arrival order)
  void DrainHeldPackets(bool in_isr)
  {
    while (held_n_ > 0u)
    {
      if (!ConsumePacket(held_[0].slot, held_[0].size, in_isr))
      {
        return;
      }
      held_[0] = held_[1];
      --held_n_;
    }
  }

  /// 保持两个槽都在收：下一个请求的大小按已知情况定，见 PlanNextSize。
  /// Keep both slots receiving; the size of the next request depends on what is known,
  /// see PlanNextSize.
  void TopUpReceives(bool in_isr)
  {
    if (!inited_ || !ep_data_out_) return;
    while (armed_n_ < 2u)
    {
      const size_t size = PlanNextSize();
      if (ep_data_out_->Receive(size, in_isr) != ErrorCode::OK)
      {
        break;
      }
      armed_sz_[armed_n_++] = size;
    }
  }

  /// 下一个接收请求要多大。已经确定属于当前传输的字节一次要完（补零的剩余部分、已知长度
  /// 的命令的剩余部分），扣掉已排队请求会收走的；否则只要一个端点包，收到后再看内容。
  /// How large the next receive request is. Bytes known to belong to the current
  /// transfer (the rest of the padding, the rest of a command of known length) are
  /// asked for at once, minus what the requests already queued will take; otherwise one
  /// endpoint packet, and the content decides after it arrives.
  size_t PlanNextSize() const
  {
    const size_t mps = OutChunkSize();
    size_t cap = ep_data_out_->GetGranted().slot_capacity;
    cap -= cap % mps;
    if (cap < mps) cap = mps;
    size_t committed = 0u;
    for (uint8_t i = 0u; i < armed_n_; ++i)
    {
      committed += armed_sz_[i];
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

  /// 延时命令：睡到到期再续做；每毫秒检查一次，解绑时提前结束。
  /// A delayed command: sleep until it is due and continue it; checked every millisecond,
  /// and ended early on an unbind.
  void RunPendingAction()
  {
    while (pending_action_.load(std::memory_order_acquire) !=
           static_cast<uint8_t>(PendingAction::NONE))
    {
      if (!inited_)
      {
        CancelPendingAction();
        AbandonCurrent();
        return;
      }
      ServicePendingAction(false);
      if (pending_action_.load(std::memory_order_acquire) ==
          static_cast<uint8_t>(PendingAction::NONE))
      {
        return;
      }
      LibXR::Thread::Sleep(1);
    }
  }

  /// 执行手里的这条命令（Step
  /// 线程）：命令槽和响应缓冲（IN 槽或暂存响应槽）都由本线程持有，到响应发布才放出。
  /// 延时命令（size 为 0）一直持有它们，续做完成后才发布。 Execute the command in hand
  /// (Step thread): the command slot and the response buffer (an IN slot or a staging
  /// slot) are held by this thread until the response is published. A delayed command
  /// (size 0) keeps holding them and is published once it is carried on to completion.
  void ExecuteCurrent(bool in_isr)
  {
    CommandSlot& command = cmd_pool_.At(cur_cmd_);
    uint8_t* resp = nullptr;
    uint16_t cap = RESP_SLOT_SIZE;
    if (cur_in_.Valid())
    {
      // IN 槽不小于通告的 DAP 包长（见 BindEndpoints）。
      // An IN slot is not smaller than the announced DAP packet size (see BindEndpoints).
      resp = static_cast<uint8_t*>(cur_in_.data.addr_);
      cap = static_cast<uint16_t>(std::min<size_t>(cur_in_.data.size_, RESP_SLOT_SIZE));
    }
    else
    {
      resp = resp_pool_.At(cur_resp_).payload;
    }
    uint16_t size = 0;
    const auto result =
        ProcessOneCommand(in_isr, command.data, command.len, resp, cap, size);
    UNUSED(result);
    if (size == 0u)
    {
      if (pending_action_.load(std::memory_order_acquire) ==
          static_cast<uint8_t>(PendingAction::NONE))
      {
        // 没有响应的命令：不发布，放出两个槽，USB 侧可以继续处理被留住的包。
        // A command without a response: nothing is published, both slots are given back
        // and the USB side can go on with the packets it held.
        AbandonCurrent();
        RequestClassService(false);
        return;
      }
      // 延时命令的响应未就绪；两个槽继续持有，pending_action_ 已登记，Step() 会续做。
      // The delayed command's response is not ready; both slots stay held, the pending
      // action is registered and Step() carries it on.
      return;
    }
    REQUIRE(size <= cap);
    CompleteResponse(size, in_isr);
  }

  /// 这条命令开始以来没有发生解绑/重新绑定 / No unbind/bind since this command began
  bool StepStillBound() const
  {
    return inited_ && cur_epoch_ == epoch_.load(std::memory_order_acquire);
  }

  /// 放出本线程持有的命令槽和响应缓冲（不发布） / Give back the command slot and the
  /// response buffer this thread holds (without publishing)
  void AbandonCurrent()
  {
    if (cur_in_.Valid())
    {
      cur_in_ep_->ReleaseSlot(cur_in_, false);
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

  /// 发布响应并放出命令槽：直接写在 IN 槽里的响应由本线程提交，
  /// 暂存的响应交给 USB 侧提交；延时的响应在续做完成后也从这里发布。
  /// 类服务随后处理被留住的包和补挂接收。
  /// Publish the response and give back the command slot: a response written in an IN
  /// slot is submitted by this thread, a staged one is left to the USB side; a deferred
  /// response is published here once its continuation completes. The class service then
  /// handles held packets and re-arms the receives.
  void CompleteResponse(uint16_t size, bool in_isr)
  {
    UNUSED(in_isr);
    if (external_active_)
    {
      // ExecuteExternal 自己交付响应，不进 USB。
      // ExecuteExternal delivers the response itself; it does not go to USB.
      external_size_ = size;
      return;
    }
    if (!StepStillBound())
    {
      // 命令执行期间发生了解绑/重新绑定：这条响应作废。
      // Unbound or rebound while the command ran: the response is void.
      AbandonCurrent();
      return;
    }
    if (cur_in_.Valid())
    {
      SubmitInSlot(size);
    }
    else
    {
      resp_pool_.At(cur_resp_).len = size;
      staged_outstanding_.fetch_add(1u, std::memory_order_acq_rel);
      resp_pool_.Publish(cur_resp_);
      cur_resp_ = SLOT_NONE;
    }
    cmd_pool_.Release(cur_cmd_);
    cur_cmd_ = SLOT_NONE;
    RequestClassService(false);
  }

  /// 保留队尾响应槽并登记延时动作；deadline 从现在起算（ms）。
  /// Reserve the tail response slot and register the deferred action; the deadline
  /// counts from now (ms).
  void BeginDeferredAction(PendingAction action, uint32_t delay_ms, uint8_t* resp,
                           uint16_t resp_cap)
  {
    deferred_resp_ = resp;
    deferred_resp_cap_ = resp_cap;
    action_deadline_ms_ =
        static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds()) + delay_ms;
    pending_action_.store(static_cast<uint8_t>(action), std::memory_order_release);
  }

  void CancelPendingAction()
  {
    pending_action_.store(static_cast<uint8_t>(PendingAction::NONE),
                          std::memory_order_release);
    deferred_resp_ = nullptr;
    deferred_resp_cap_ = 0u;
  }

  /// 延时命令的续做点（owner 里执行）：到点后完成被延时的响应。
  /// Resume point of a delayed command (runs in the owner): finish the deferred
  /// response once the delay has elapsed.
  void ServicePendingAction(bool in_isr)
  {
    const uint32_t now = static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds());
    const auto action =
        static_cast<PendingAction>(pending_action_.load(std::memory_order_acquire));
    if (action == PendingAction::RESET_LOW)
    {
      if (static_cast<int32_t>(now - action_deadline_ms_) < 0) return;
      DriveReset(true);
      BeginDeferredAction(PendingAction::RESET_HIGH, 1u, deferred_resp_,
                          deferred_resp_cap_);
      return;
    }
    if (action == PendingAction::SWJ_PINS)
    {
      // PinWait：每拍采样一次，匹配或超时即完成（§10.3-a 的 Timer 路径轮询）。
      // PinWait: sample once per tick; finish on a match or at the timeout (the
      // Timer-path polling of §10.3-a).
      const uint8_t pin_in = ReadSwjPins();
      const bool matched =
          swj_pin_sel_ != 0u && (pin_in & swj_pin_sel_) == swj_pin_expect_;
      if (!matched && static_cast<int32_t>(now - action_deadline_ms_) < 0) return;
      deferred_resp_[1] = pin_in;
      FinishDeferredResponse(2u, in_isr);
      return;
    }
    if (static_cast<int32_t>(now - action_deadline_ms_) < 0) return;
    if (action == PendingAction::RESET_HIGH)
    {
      deferred_resp_[1] = DAP_OK;
      deferred_resp_[2] = 1u;
      FinishDeferredResponse(3u, in_isr);
      return;
    }
    deferred_resp_[1] = ToU8(LibXR::USB::DapLinkV2Def::Status::OK);
    FinishDeferredResponse(2u, in_isr);
  }

  void FinishDeferredResponse(uint16_t size, bool in_isr)
  {
    pending_action_.store(static_cast<uint8_t>(PendingAction::NONE),
                          std::memory_order_release);
    if (batch_active_)
    {
      // 批处理里的延时命令：响应已写进保留的槽，继续跑剩下的命令。
      // A delay command inside a batch: its response is already in the reserved
      // slot, so carry on with the rest of the batch.
      ContinueBatch(size, in_isr);
      return;
    }
    REQUIRE(size != 0U && size <= deferred_resp_cap_);
    CompleteResponse(size, in_isr);
  }

 private:
  /**
   * @brief Get effective DAP packet size
   */
  uint16_t GetDapPacketSize() const
  {
    return dap_packet_size_.load(std::memory_order_acquire);
  }

  /// 配置给出的 DAP 包长（还没有按 IN 槽收敛） / The DAP packet size the configuration
  /// gives (before it is limited by the IN slot)
  static constexpr uint16_t ConfiguredDapPacketSize() { return MAX_DAP_PACKET_SIZE; }

  /**
   * @brief Clip response length by DAP packet size and buffer capacity
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

  void ResetQueuedCommandState()
  {
    queued_request_length_ = 0u;
    queued_command_count_ = 0u;
  }

  // ============================================================================
  // Queued command helpers
  // ============================================================================

  /**
   * @brief Parse one queued command length from a packed queue stream
   * @param req request pointer
   * @param avail available bytes in the queue stream
   * @param cmd_len parsed command length
   * @return true parse success
   * @return false parse failure
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

      // Reject nested queue/execute in queued stream.
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::QUEUE_COMMANDS):
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::EXECUTE_COMMANDS):
      default:
        return false;
    }
  }

  /**
   * @brief Execute the commands in the batch buffer into one EXECUTE_COMMANDS response
   * @param in_isr current context flag
   * @param resp response buffer
   * @param resp_cap response capacity
   * @param out_len output response length
   * @return true execute success
   * @return false execute failure
   */
  /**
   * @brief 批处理游标（非阻塞续做）/ Batch cursor (resumed without blocking)
   *
   * 批执行里遇到延时命令时保存游标后返回，响应槽保持保留；延时到期由 Timer 路径
   * 经 ContinueBatch 从游标处继续，全部完成后才入队。这样 owner 路径不忙等，秒级
   * 的 SWJ_Pins 等待也不会拖住 USB 服务。
   * When a batch hits a delay command it saves the cursor and returns with the
   * response slot still reserved; once the delay elapses the Timer path resumes from
   * the cursor through ContinueBatch and only queues the response when the whole
   * batch is done. The owner path never busy-waits, so even a seconds-long SWJ_Pins
   * wait cannot stall USB service.
   */
  void ContinueBatch(uint16_t last_cmd_size, bool in_isr)
  {
    uint16_t req_off = batch_req_off_;
    uint16_t resp_off = static_cast<uint16_t>(batch_resp_off_ + last_cmd_size);
    uint16_t remaining = batch_remaining_;
    uint8_t* resp = batch_resp_;
    const uint16_t resp_cap = batch_resp_cap_;

    // 跳过刚完成的那条命令，游标指向它之后。
    // Skip the command that just finished; the cursor points past it.
    uint16_t cmd_len = 0u;
    if (remaining == 0u ||
        !ParseQueuedCommandLength(&queued_request_buffer_[req_off],
                                  static_cast<uint16_t>(queued_request_length_ - req_off),
                                  cmd_len) ||
        cmd_len == 0u)
    {
      FinishBatch(false, in_isr);
      return;
    }
    req_off = static_cast<uint16_t>(req_off + cmd_len);
    --remaining;

    while (remaining > 0u)
    {
      if (req_off >= queued_request_length_ ||
          !ParseQueuedCommandLength(
              &queued_request_buffer_[req_off],
              static_cast<uint16_t>(queued_request_length_ - req_off), cmd_len) ||
          cmd_len == 0u || resp_off >= resp_cap)
      {
        FinishBatch(false, in_isr);
        return;
      }

      uint16_t cmd_out = 0u;
      const ErrorCode ans = ProcessOneCommand(
          in_isr, &queued_request_buffer_[req_off], cmd_len, &resp[resp_off],
          static_cast<uint16_t>(resp_cap - resp_off), cmd_out);
      UNUSED(ans);

      if (pending_action_.load(std::memory_order_acquire) !=
          static_cast<uint8_t>(PendingAction::NONE))
      {
        // 又一条延时命令：更新游标继续挂起，等下一次续做。
        // Another delay command: update the cursor, stay suspended and resume later.
        batch_req_off_ = req_off;
        batch_resp_off_ = resp_off;
        batch_remaining_ = remaining;
        return;
      }

      if (cmd_out == 0u ||
          static_cast<uint32_t>(resp_off) + static_cast<uint32_t>(cmd_out) >
              static_cast<uint32_t>(resp_cap))
      {
        FinishBatch(false, in_isr);
        return;
      }

      req_off = static_cast<uint16_t>(req_off + cmd_len);
      resp_off = static_cast<uint16_t>(resp_off + cmd_out);
      --remaining;
    }

    FinishBatch(true, in_isr, resp_off);
  }

  /// 结束批处理：ok 时入队已建好的响应，否则回 DAP_ERROR（2 字节）。
  /// Finish the batch: queue the built response when ok, otherwise answer DAP_ERROR
  /// (2 bytes).
  void FinishBatch(bool ok, bool in_isr, uint16_t size = 0u)
  {
    uint8_t* resp = batch_resp_;
    const uint16_t resp_cap = batch_resp_cap_;
    batch_active_ = false;
    batch_req_off_ = 0u;
    batch_resp_off_ = 0u;
    batch_remaining_ = 0u;
    batch_resp_ = nullptr;
    batch_resp_cap_ = 0u;
    ResetQueuedCommandState();

    if (!ok)
    {
      if (resp != nullptr && resp_cap >= 2u)
      {
        resp[0] = ToU8(LibXR::USB::DapLinkV2Def::CommandId::EXECUTE_COMMANDS);
        resp[1] = DAP_ERROR;
      }
      size = 2u;
    }
    CompleteResponse(size, in_isr);
  }

  bool TryExecuteQueuedCommandStream(bool in_isr, uint8_t* resp, uint16_t resp_cap,
                                     uint16_t& out_len)
  {
    out_len = 0u;
    if (!resp || resp_cap < 1u)
    {
      return false;
    }

    if (resp_cap < 2u)
    {
      return false;
    }
    resp[0] = ToU8(LibXR::USB::DapLinkV2Def::CommandId::EXECUTE_COMMANDS);
    resp[1] = static_cast<uint8_t>(queued_command_count_);
    out_len = 2u;

    if (queued_command_count_ == 0u)
    {
      return true;
    }

    uint16_t req_off = 0u;
    uint16_t resp_off = 2u;
    for (uint32_t i = 0u; i < queued_command_count_; ++i)
    {
      if (req_off >= queued_request_length_)
      {
        return false;
      }

      uint16_t cmd_len = 0u;
      if (!ParseQueuedCommandLength(
              &queued_request_buffer_[req_off],
              static_cast<uint16_t>(queued_request_length_ - req_off), cmd_len) ||
          cmd_len == 0u)
      {
        return false;
      }

      if (resp_off >= resp_cap)
      {
        return false;
      }

      uint16_t cmd_out = 0u;
      const ErrorCode ans = ProcessOneCommand(
          in_isr, &queued_request_buffer_[req_off], cmd_len, &resp[resp_off],
          static_cast<uint16_t>(resp_cap - resp_off), cmd_out);
      UNUSED(ans);

      // 命令挂起了（含延时的 DAP_Delay / ResetTarget / SWJ_Pins）：保存批游标
      // 后返回，响应槽保持保留，延时到期由 Timer 路径续做——owner 不忙等。
      // The command suspended itself (DAP_Delay / ResetTarget / SWJ_Pins with a
      // wait): save the batch cursor and return with the response slot still
      // reserved; the Timer path resumes once the delay elapses, so the owner never
      // busy-waits.
      if (pending_action_.load(std::memory_order_acquire) !=
          static_cast<uint8_t>(PendingAction::NONE))
      {
        batch_active_ = true;
        batch_req_off_ = req_off;
        batch_resp_off_ = resp_off;
        batch_remaining_ = static_cast<uint16_t>(queued_command_count_ - i);
        batch_resp_ = resp;
        batch_resp_cap_ = resp_cap;
        out_len = 0u;
        return true;
      }

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

    if (req_off != queued_request_length_)
    {
      return false;
    }

    out_len = resp_off;
    return true;
  }

  /// 一个端点包的大小；端点包长还没确定时用 DAP 包长。
  /// One endpoint packet; the DAP packet size while the endpoint packet size is not set.
  uint16_t OutChunkSize() const
  {
    const uint16_t mps = ep_data_out_ ? ep_data_out_->MaxPacketSize() : 0u;
    const uint16_t packet = GetDapPacketSize();
    return (mps == 0u || mps > packet) ? packet : mps;
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

  /// 按命令自己的内容判断已收的 avail 字节是否构成一条完整命令。
  /// Decide from the command's own content whether the received avail bytes make one
  /// whole command.
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
  // ============================================================================
  // Command dispatch
  // ============================================================================

  /**
   * @brief Process one command
   *
   * @param in_isr   ISR context flag
   * @param req      Request buffer
   * @param req_len  Request length
   * @param resp     Response buffer
   * @param resp_cap Response capacity
   * @param out_len  Output response length
   * @return Error code
   */
  ErrorCode ProcessOneCommand(bool in_isr, const uint8_t* req, uint16_t req_len,
                              uint8_t* resp, uint16_t resp_cap, uint16_t& out_len)
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
        return HandleInfo(in_isr, req, req_len, resp, resp_cap, out_len);
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::HOST_STATUS):
        return HandleHostStatus(in_isr, req, req_len, resp, resp_cap, out_len);
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::CONNECT):
        return HandleConnect(in_isr, req, req_len, resp, resp_cap, out_len);
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::DISCONNECT):
        return HandleDisconnect(in_isr, req, req_len, resp, resp_cap, out_len);
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::TRANSFER_CONFIGURE):
        return HandleTransferConfigure(in_isr, req, req_len, resp, resp_cap, out_len);
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::TRANSFER):
        return HandleTransfer(in_isr, req, req_len, resp, resp_cap, out_len);
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::TRANSFER_BLOCK):
        return HandleTransferBlock(in_isr, req, req_len, resp, resp_cap, out_len);
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::TRANSFER_ABORT):
        return HandleTransferAbort(in_isr, req, req_len, resp, resp_cap, out_len);
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::WRITE_ABORT):
        return HandleWriteABORT(in_isr, req, req_len, resp, resp_cap, out_len);
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::DELAY):
        return HandleDelay(in_isr, req, req_len, resp, resp_cap, out_len);
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::RESET_TARGET):
        return HandleResetTarget(in_isr, req, req_len, resp, resp_cap, out_len);

      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::SWJ_PINS):
        return HandleSWJPins(in_isr, req, req_len, resp, resp_cap, out_len);
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::SWJ_CLOCK):
        return HandleSWJClock(in_isr, req, req_len, resp, resp_cap, out_len);
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::SWJ_SEQUENCE):
        return HandleSWJSequence(in_isr, req, req_len, resp, resp_cap, out_len);
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::SWD_CONFIGURE):
        return HandleSWDConfigure(in_isr, req, req_len, resp, resp_cap, out_len);
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::SWD_SEQUENCE):
        return HandleSWDSequence(in_isr, req, req_len, resp, resp_cap, out_len);

      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::JTAG_SEQUENCE):
        if constexpr (JTAG_ENABLED)
        {
          return HandleJTAGSequence(in_isr, req, req_len, resp, resp_cap, out_len);
        }
        (void)BuildUnknownCmdResponse(resp, resp_cap, out_len);
        return ErrorCode::NOT_SUPPORT;
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::JTAG_CONFIGURE):
        if constexpr (JTAG_ENABLED)
        {
          return HandleJTAGConfigure(in_isr, req, req_len, resp, resp_cap, out_len);
        }
        (void)BuildUnknownCmdResponse(resp, resp_cap, out_len);
        return ErrorCode::NOT_SUPPORT;
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::JTAG_IDCODE):
        if constexpr (JTAG_ENABLED)
        {
          return HandleJTAGIdCode(in_isr, req, req_len, resp, resp_cap, out_len);
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
        return HandleExecuteCommands(in_isr, req, req_len, resp, resp_cap, out_len);

      default:
        (void)BuildUnknownCmdResponse(resp, resp_cap, out_len);
        return ErrorCode::NOT_SUPPORT;
    }
  }

  /**
   * @brief Build NOT_SUPPORT response
   *
   * @param resp     Response buffer
   * @param resp_cap Response capacity
   * @param out_len  Output response length
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
  // ============================================================================
  // DAP_Info
  // ============================================================================

  ErrorCode HandleInfo(bool /*in_isr*/, const uint8_t* req, uint16_t req_len,
                       uint8_t* resp, uint16_t resp_cap, uint16_t& out_len)
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

    // Ensure termination even when truncated
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

    // Little-endian device only: direct memcpy to payload (alignment-safe).
    Memory::FastCopy(&resp[2], &val, sizeof(val));

    out_len = 6u;
    return ErrorCode::OK;
  }

 private:
  // ============================================================================
  // Simple control handlers
  // ============================================================================

  ErrorCode HandleHostStatus(bool /*in_isr*/, const uint8_t* /*req*/,
                             uint16_t /*req_len*/, uint8_t* resp, uint16_t resp_cap,
                             uint16_t& out_len)
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

  ErrorCode HandleConnect(bool /*in_isr*/, const uint8_t* req, uint16_t req_len,
                          uint8_t* resp, uint16_t resp_cap, uint16_t& out_len)
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
      ResetQueuedCommandState();

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
          ResetQueuedCommandState();
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

  ErrorCode HandleDisconnect(bool /*in_isr*/, const uint8_t* /*req*/,
                             uint16_t /*req_len*/, uint8_t* resp, uint16_t resp_cap,
                             uint16_t& out_len)
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
    ResetQueuedCommandState();

    resp[0] = ToU8(LibXR::USB::DapLinkV2Def::CommandId::DISCONNECT);
    resp[1] = ToU8(LibXR::USB::DapLinkV2Def::Status::OK);
    out_len = 2u;
    return ErrorCode::OK;
  }

  ErrorCode HandleTransferConfigure(bool /*in_isr*/, const uint8_t* req, uint16_t req_len,
                                    uint8_t* resp, uint16_t resp_cap, uint16_t& out_len)
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

    // Map to SWD transaction policy
    Debug::Swd::TransferPolicy pol = swd_.GetTransferPolicy();
    pol.idle_cycles = IDLE;
    pol.wait_retry = wait_retry;
    swd_.SetTransferPolicy(pol);

    resp[1] = ToU8(LibXR::USB::DapLinkV2Def::Status::OK);
    out_len = 2u;
    return ErrorCode::OK;
  }

  ErrorCode HandleTransferAbort(bool /*in_isr*/, const uint8_t* /*req*/,
                                uint16_t /*req_len*/, uint8_t* /*resp*/,
                                uint16_t /*resp_cap*/, uint16_t& out_len)
  {
    // DAP_TransferAbort 没有响应。USB 路径在接收时就置位标志（见 OnDataOutComplete），
    // 这里服务批命令和 ExecuteExternal 里夹带的中止。
    // DAP_TransferAbort has no response. The USB path sets the flag on reception (see
    // OnDataOutComplete); this serves an abort carried inside a batch or ExecuteExternal.
    SetTransferAbortFlag(true);
    out_len = 0u;
    return ErrorCode::OK;
  }

  ErrorCode HandleWriteABORT(bool in_isr, const uint8_t* req, uint16_t req_len,
                             uint8_t* resp, uint16_t resp_cap, uint16_t& out_len)
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
        return HandleJtagWriteAbort(in_isr, req, req_len, resp, resp_cap, out_len);
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

  ErrorCode HandleDelay(bool /*in_isr*/, const uint8_t* req, uint16_t req_len,
                        uint8_t* resp, uint16_t resp_cap, uint16_t& out_len)
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

    if (us == 0u)
    {
      resp[1] = ToU8(LibXR::USB::DapLinkV2Def::Status::OK);
      out_len = 2u;
      return ErrorCode::OK;
    }

    // §10.3-a：延时命令不在回调/owner 上下文忙等；响应槽保留给线程上下文的
    // Timer 路径，到期（向上取整到 1 ms）后续做并入队。
    // §10.3-a: a delayed command never busy-waits in the callback/owner context;
    // its response slot is reserved for the Timer path in thread context, which
    // completes and queues the response once the delay elapses (rounded up to 1 ms).
    BeginDeferredAction(PendingAction::DELAY, (static_cast<uint32_t>(us) + 999u) / 1000u,
                        resp, resp_cap);
    out_len = 0u;
    return ErrorCode::OK;
  }

  ErrorCode HandleResetTarget(bool /*in_isr*/, const uint8_t* /*req*/,
                              uint16_t /*req_len*/, uint8_t* resp, uint16_t resp_cap,
                              uint16_t& out_len)
  {
    if (!resp || resp_cap < 3u)
    {
      out_len = 0u;
      return ErrorCode::NOT_FOUND;
    }

    resp[0] = ToU8(LibXR::USB::DapLinkV2Def::CommandId::RESET_TARGET);

    if (nreset_gpio_ != nullptr)
    {
      DriveReset(false);
      // 拉低与释放之间的两段 1 ms 延时同样走 §10.3-a 的 Timer 路径。
      // Both 1 ms delays between asserting and releasing nRESET use the §10.3-a
      // Timer path as well.
      BeginDeferredAction(PendingAction::RESET_LOW, 1u, resp, resp_cap);
      out_len = 0u;
      return ErrorCode::OK;
    }

    // Always return DAP_OK; if reset is not implemented, Execute=0.
    resp[1] = DAP_OK;
    resp[2] = 0u;
    out_len = 3u;
    return ErrorCode::OK;
  }

 private:
  // ============================================================================
  // SWJ / SWD handlers
  // ============================================================================
  ErrorCode HandleSWJPins(bool /*in_isr*/, const uint8_t* req, uint16_t req_len,
                          uint8_t* resp, uint16_t resp_cap, uint16_t& out_len)
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

    // 0) Latch requested states into shadow for ALL selected pins (even if unsupported)
    swj_shadow_ = static_cast<uint8_t>((swj_shadow_ & static_cast<uint8_t>(~PIN_SEL)) |
                                       (PIN_OUT & PIN_SEL));

    // 1) Only physically support nRESET (best-effort)
    if ((PIN_SEL & LibXR::USB::DapLinkV2Def::DAP_SWJ_NRESET) != 0u)
    {
      const bool LEVEL_HIGH =
          ((PIN_OUT & LibXR::USB::DapLinkV2Def::DAP_SWJ_NRESET) != 0u);
      // DriveReset updates last_nreset_level_high_ and shadow, and writes GPIO if
      // present.
      DriveReset(LEVEL_HIGH);
    }

    // 2) PinWait：等 (PinInput & PinSelect) 匹配 (PinOut & PinSelect) 或超时。
    //    不在回调/owner 里忙等：先采样一次；已经匹配（shadow-only 引脚总是如此）
    //    或无需等待就立即返回，否则按 §10.3-a 交给 Timer 路径每拍（1 ms）复查，
    //    直到匹配或超时。
    //    PinWait: wait until (PinInput & PinSelect) matches (PinOut & PinSelect), or
    //    the timeout. Never busy-wait in the callback/owner: sample once here and
    //    return at once when it already matches (shadow-only pins always do) or no
    //    wait was requested; otherwise the §10.3-a Timer path re-samples every tick
    //    (1 ms) until a match or the timeout.
    swj_pin_sel_ = PIN_SEL;
    swj_pin_expect_ = static_cast<uint8_t>(PIN_OUT & PIN_SEL);
    const uint8_t pin_in = ReadSwjPins();
    if (wait_us == 0u || PIN_SEL == 0u || (pin_in & PIN_SEL) == swj_pin_expect_)
    {
      resp[1] = pin_in;
      out_len = 2u;
      return ErrorCode::OK;
    }

    BeginDeferredAction(PendingAction::SWJ_PINS, (wait_us + 999u) / 1000u, resp,
                        resp_cap);
    out_len = 0u;
    return ErrorCode::OK;
  }

  /// 读回 SWJ 引脚电平：从 shadow 出发，nRESET 若接了 GPIO 则用实际电平覆盖。
  /// Sample the SWJ pins: start from the shadow and override nRESET with the wired
  /// GPIO level when present.
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

  ErrorCode HandleSWJClock(bool /*in_isr*/, const uint8_t* req, uint16_t req_len,
                           uint8_t* resp, uint16_t resp_cap, uint16_t& out_len)
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

  ErrorCode HandleSWJSequence(bool /*in_isr*/, const uint8_t* req, uint16_t req_len,
                              uint8_t* resp, uint16_t resp_cap, uint16_t& out_len)
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

    // Delegate to SwdPort implementation (no RawMode / no pin-level control here)
    const ErrorCode EC = swd_.SeqWriteBits(BIT_COUNT, data);
    if (EC != ErrorCode::OK)
    {
      resp[1] = ToU8(LibXR::USB::DapLinkV2Def::Status::ERROR);
      // Keep transport-level OK so host still gets a valid response.
      return ErrorCode::OK;
    }

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

  ErrorCode HandleSWDConfigure(bool /*in_isr*/, const uint8_t* req, uint16_t req_len,
                               uint8_t* resp, uint16_t resp_cap, uint16_t& out_len)
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

  ErrorCode HandleSWDSequence(bool /*in_isr*/, const uint8_t* req, uint16_t req_len,
                              uint8_t* resp, uint16_t resp_cap, uint16_t& out_len)
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
        // Output: data bytes are in request
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
          // Return OK so host gets a valid response packet
          return ErrorCode::OK;
        }
      }
      else
      {
        // Input: captured data is appended to response
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

  ErrorCode HandleJTAGSequence(bool /*in_isr*/, const uint8_t* req, uint16_t req_len,
                               uint8_t* resp, uint16_t resp_cap, uint16_t& out_len)
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

  ErrorCode HandleJTAGConfigure(bool /*in_isr*/, const uint8_t* req, uint16_t req_len,
                                uint8_t* resp, uint16_t resp_cap, uint16_t& out_len)
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

  ErrorCode HandleJTAGIdCode(bool /*in_isr*/, const uint8_t* req, uint16_t req_len,
                             uint8_t* resp, uint16_t resp_cap, uint16_t& out_len)
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

  /// 校验 [ID, 命令数, 命令...] 并把命令部分拷进批处理缓冲；格式或容量不对返回 false。
  /// Validate [ID, count, commands...] and copy the command part into the batch buffer;
  /// false when the format or the capacity does not fit.
  bool AppendInlineCommands(const uint8_t* req, uint16_t req_len)
  {
    if (!req || req_len < 2u)
    {
      return false;
    }

    const uint8_t NUM = req[1];
    if (NUM == 0u)
    {
      return true;
    }
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

    if (req_off != req_len)
    {
      return false;
    }

    if (static_cast<uint32_t>(queued_request_length_) +
                static_cast<uint32_t>(req_len - 2u) >
            static_cast<uint32_t>(QUEUED_REQ_BUFFER_SIZE) ||
        static_cast<uint32_t>(queued_command_count_) + static_cast<uint32_t>(NUM) >
            static_cast<uint32_t>(QUEUED_CMD_COUNT_MAX))
    {
      return false;
    }

    if (NUM != 0u)
    {
      Memory::FastCopy(&queued_request_buffer_[queued_request_length_], &req[2],
                       req_len - 2u);
      queued_request_length_ =
          static_cast<uint16_t>(queued_request_length_ + (req_len - 2u));
      queued_command_count_ = static_cast<uint16_t>(queued_command_count_ + NUM);
    }
    return true;
  }

  /**
   * @brief DAP_ExecuteCommands（也用于 DAP_QueueCommands）/ DAP_ExecuteCommands (also
   *        used for DAP_QueueCommands)
   *
   * 请求 [ID, 命令数, 命令...]，响应 [0x7F, 命令数, 各条命令的响应...]。
   * Request [ID, count, commands...], response [0x7F, count, each command's response...].
   */
  ErrorCode HandleExecuteCommands(bool in_isr, const uint8_t* req, uint16_t req_len,
                                  uint8_t* resp, uint16_t resp_cap, uint16_t& out_len)
  {
    ResetQueuedCommandState();
    if (!AppendInlineCommands(req, req_len))
    {
      ResetQueuedCommandState();
      return BuildCmdStatusResponse(
          ToU8(LibXR::USB::DapLinkV2Def::CommandId::EXECUTE_COMMANDS), DAP_ERROR, resp,
          resp_cap, out_len);
    }
    const bool ok = TryExecuteQueuedCommandStream(in_isr, resp, resp_cap, out_len);

    if (batch_active_)
    {
      // 批内命令挂起：响应未就绪（out_len == 0），入队状态与响应槽都保留，
      // 由 Timer 路径续做后入队。
      // A command inside the batch suspended: the response is not ready (out_len ==
      // 0); the queued state and the response slot stay reserved for the Timer-path
      // continuation.
      return ErrorCode::OK;
    }

    ResetQueuedCommandState();

    if (!ok)
    {
      return BuildCmdStatusResponse(
          ToU8(LibXR::USB::DapLinkV2Def::CommandId::EXECUTE_COMMANDS), DAP_ERROR, resp,
          resp_cap, out_len);
    }

    return ErrorCode::OK;
  }

 private:
  // ============================================================================
  // Transfer helpers
  // ============================================================================

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
    const auto req = LibXR::Debug::SwdProtocol::make_dp_read_req(
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
    const auto req = LibXR::Debug::SwdProtocol::make_ap_read_req(addr2b);
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
  // ============================================================================
  // DAP_Transfer / DAP_TransferBlock
  // ============================================================================

  // The following long functions are kept as-is; API docs live at the header boundary.

  ErrorCode HandleTransfer(bool in_isr, const uint8_t* req, uint16_t req_len,
                           uint8_t* resp, uint16_t resp_cap, uint16_t& out_len)
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
        return HandleJtagTransfer(in_isr, req, req_len, resp, resp_cap, out_len);
      }
    }

    resp[0] = ToU8(LibXR::USB::DapLinkV2Def::CommandId::TRANSFER);
    resp[1] = 0u;  // response_count = #successful transfers
    resp[2] = 0u;  // response_value (ACK bits / ERROR / MISMATCH)
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
    // Keep reference behavior: if no transfer executes, response_value remains 0.
    uint8_t response_value = 0u;

    // AP writes are posted by the target. Keep the write batch fast and check
    // it once at a transfer boundary with a DP RDBUFF read.
    bool pending_ap_write = false;

    // -------- posted-read pipeline state (AP read only) --------
    struct PendingApRead
    {
      bool valid = false;
      bool need_ts = false;  // Whether this AP read transfer needs timestamp.
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

      // Successful path keeps OK.
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

    // Complete pending AP read before non-AP operations to keep response order.
    auto complete_pending_ap_read_if_any = [&]() -> bool
    { return pending.valid ? complete_pending_ap_read() : true; };

    auto close_posted_state = [&]() -> bool
    { return complete_pending_ap_read_if_any() && check_posted_ap_write_if_pending(); };

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
          response_count++;  // MATCH_MASK counts as a successful transfer.
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
              ec = swd_.ApReadTxn(ADDR2B, rdata, ack);  // Includes RDBUFF.
              if (ec == ErrorCode::OK && ack == LibXR::Debug::SwdProtocol::Ack::OK)
              {
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
            // MISMATCH does not increase response_count.
            break;
          }

          response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_OK;
          response_count++;
          continue;
        }

        // ---- Normal read ----
        if (!AP)
        {
          // DP read does not participate in posted pipelines.
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

        // AP reads can observe previous AP writes, so close posted writes
        // before starting or extending the posted-read pipeline.
        if (!check_posted_ap_write_if_pending())
        {
          break;
        }

        // AP normal read: posted-read pipeline.
        if (!pending.valid)
        {
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

          // Transfer is not "complete" yet.
          // Data will be produced by next AP read or tail RDBUFF.
          response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_OK;
        }
        else
        {
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
            // Current AP read failed: try best effort to complete pending AP read.
            // Otherwise, response_count may miss one and pipeline may remain dirty.
            const uint8_t PROOR_FAIL =
                (CUR_V != LibXR::USB::DapLinkV2Def::DAP_TRANSFER_OK)
                    ? CUR_V
                    : LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;

            if (!complete_pending_ap_read())
            {
              // Pending itself failed: return earlier failure
              // (complete_pending_ap_read already wrote response_value).
              break;
            }

            // Pending completion succeeded: keep current failure.
            response_value = PROOR_FAIL;
            break;
          }

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

    // If pending AP read still exists at tail:
    // - when response_value == OK: normal tail by one RDBUFF;
    // - when response_value != OK: try pending completion first; if completion
    //   succeeds, keep original failure; otherwise use pending failure.
    if (pending.valid)
    {
      const uint8_t PRIOR_FAIL = response_value;

      if (!complete_pending_ap_read())
      {
        // Pending failure wins here (response_value already written).
      }
      else
      {
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
      // response_value is already set by the recovery helper.
    }
    resp[1] = response_count;
    resp[2] = response_value;
    out_len = resp_off;
    return ErrorCode::OK;
  }
  ErrorCode HandleTransferBlock(bool in_isr, const uint8_t* req, uint16_t req_len,
                                uint8_t* resp, uint16_t resp_cap, uint16_t& out_len)
  {
    if constexpr (JTAG_ENABLED)
    {
      if (dap_state_.debug_port == LibXR::USB::DapLinkV2Def::DebugPort::JTAG)
      {
        return HandleJtagTransferBlock(in_isr, req, req_len, resp, resp_cap, out_len);
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

    // Count==0: return OK
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

    // WRITE path: keep original behavior
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
        auto swd_req = LibXR::Debug::SwdProtocol::make_ap_write_req(ADDR2B, 0u);
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

    // READ path
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

      // DP read: keep original behavior
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

    // AP read: posted-read pipeline
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

      auto ap_read_req = LibXR::Debug::SwdProtocol::make_ap_read_req(ADDR2B);
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

      // i=1..COUNT-1: each AP read returns previous posted data.
      for (uint32_t i = 1; i < count; ++i)
      {
        ec = TransferTxnFast(ap_read_req, ap_read_resp);
        const uint8_t CUR = MapAckToDapResp(ap_read_resp.ack);

        if (ap_read_resp.ack != LibXR::Debug::SwdProtocol::Ack::OK ||
            ec != ErrorCode::OK || !ap_read_resp.parity_ok)
        {
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
              done = static_cast<uint16_t>(i);  // done includes transfer [0..i-1]
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

  ErrorCode HandleJtagTransfer(bool /*in_isr*/, const uint8_t* req, uint16_t req_len,
                               uint8_t* resp, uint16_t resp_cap, uint16_t& out_len)
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

  ErrorCode HandleJtagTransferBlock(bool /*in_isr*/, const uint8_t* req, uint16_t req_len,
                                    uint8_t* resp, uint16_t resp_cap, uint16_t& out_len)
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

  ErrorCode HandleJtagWriteAbort(bool /*in_isr*/, const uint8_t* req, uint16_t req_len,
                                 uint8_t* resp, uint16_t resp_cap, uint16_t& out_len)
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
  // ============================================================================
  // Reset helpers
  // ============================================================================

  void DriveReset(bool release)
  {
    last_nreset_level_high_ = release;

    // Update shadow regardless of whether GPIO exists
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
  // OpenOCD 0.12 computes pending_queue_len = (packet_size - 4) / 5 for CMD_DAP_TFER.
  // CMD_DAP_TFER transfer_count is u8, so cap host-visible packet size to avoid
  // transfer_count > 255 and "expected X, got Y" mismatch.
  static constexpr uint16_t OPENOCD_SAFE_DAP_PACKET_SIZE =
      static_cast<uint16_t>((255u * 5u) + 4u);
  static constexpr uint8_t JTAG_MAX_DEVICES = 16u;
  // Host-visible CMSIS-DAP packet size; endpoint MPS limits are handled by
  // Endpoint-owned logical segmentation/reassembly.
  static constexpr uint16_t MAX_DAP_PACKET_SIZE = MaxDapPacketSize;
  static constexpr uint16_t QUEUED_REQ_BUFFER_SIZE = QueuedRequestBufferSize;
  static constexpr uint16_t QUEUED_CMD_COUNT_MAX = QueuedCommandCountMax;
  static constexpr uint16_t RESP_SLOT_SIZE = MAX_DAP_PACKET_SIZE;
  static constexpr uint16_t QUEUED_REQ_BUFFER_ALLOC_SIZE = QUEUED_REQ_BUFFER_SIZE;
  static_assert(PACKET_COUNT_ADVERTISED > 0u, "PACKET_COUNT_ADVERTISED must be > 0");
  static_assert(PACKET_COUNT_EFFECTIVE > 0u, "PACKET_COUNT_EFFECTIVE must be > 0");
  static_assert(MAX_DAP_PACKET_SIZE >= DEFAULT_DAP_PACKET_SIZE,
                "MAX_DAP_PACKET_SIZE must be >= DEFAULT_DAP_PACKET_SIZE");
  static_assert(MAX_DAP_PACKET_SIZE <= OPENOCD_SAFE_DAP_PACKET_SIZE,
                "MAX_DAP_PACKET_SIZE exceeds OpenOCD-safe limit");
  static_assert(((OPENOCD_SAFE_DAP_PACKET_SIZE - 4u) / 5u) <= 255u,
                "OPENOCD_SAFE_DAP_PACKET_SIZE too large for CMD_DAP_TFER u8 count");
  static_assert(QUEUED_REQ_BUFFER_SIZE > 0u, "QUEUED_REQ_BUFFER_SIZE must be > 0");
  static_assert(QUEUED_CMD_COUNT_MAX > 0u, "QUEUED_CMD_COUNT_MAX must be > 0");
  static_assert(RESP_SLOT_SIZE >= DEFAULT_DAP_PACKET_SIZE,
                "RESP_SLOT_SIZE must cover FS bulk packet size");

  struct ResponseSlot
  {
    uint16_t len = 0u;
    uint8_t payload[RESP_SLOT_SIZE] = {};
  };

  struct CommandSlot
  {
    uint16_t len = 0u;
    uint8_t data[MAX_DAP_PACKET_SIZE] = {};
  };

  static constexpr size_t SLOT_NONE = static_cast<size_t>(-1);
  /// 命令池槽数等于通告的在途命令数：一条大命令在执行时，后面的小命令只占一个槽，下一条大
  /// 命令仍有槽可以接收，接收和执行才能真正重叠。
  /// The command pool has as many slots as the advertised commands in flight: while a big
  /// command executes, the small commands behind it take one slot each and the next big
  /// command still finds a slot to be received into, so reception and execution really
  /// overlap.
  static constexpr size_t CMD_SLOT_COUNT = PACKET_COUNT_EFFECTIVE;

  uint8_t queued_request_buffer_[QUEUED_REQ_BUFFER_ALLOC_SIZE] = {};
  uint16_t queued_request_length_ = 0u;
  uint16_t queued_command_count_ = 0u;

  // 批处理续做状态（非阻塞：延时命令挂起后由 Timer 路径从游标继续）
  // Batch continuation state (non-blocking: a delay command suspends and the Timer
  // path resumes from the cursor)
  bool batch_active_ = false;
  uint16_t batch_req_off_ = 0u;
  uint16_t batch_resp_off_ = 0u;
  uint16_t batch_remaining_ = 0u;
  uint8_t* batch_resp_ = nullptr;
  uint16_t batch_resp_cap_ = 0u;

  // ---------------------------------------------------------------------------
  // Command and response staging / 命令与响应暂存
  // ---------------------------------------------------------------------------

  /// 命令池：USB 服务上下文 Acquire、拷入、Publish，Step 线程 Take、执行、Release。
  /// Command pool: the USB service context Acquires, copies in and Publishes; the Step
  /// thread Takes, executes and Releases.
  LibXR::SlotPool<CommandSlot> cmd_pool_{CMD_SLOT_COUNT};
  size_t rx_slot_ = SLOT_NONE;  ///< USB 侧正在拼装的命令槽 / Command slot being assembled
  size_t cur_cmd_ =
      SLOT_NONE;  ///< Step 线程持有的命令槽 / Command slot the Step thread holds
  /// 响应池（暂存）：Step 线程 Acquire、写入、Publish，USB 服务上下文 Take、拷进 IN
  /// 槽提交、 Release。槽数等于通告的在途命令数，所以执行不必等主机取走前面的响应。
  /// Response pool (staging): the Step thread Acquires, writes and Publishes; the USB
  /// service context Takes, copies into an IN slot, submits and Releases. It has as many
  /// slots as the advertised commands in flight, so execution does not wait for the host
  /// to take earlier responses.
  LibXR::SlotPool<ResponseSlot> resp_pool_{PACKET_COUNT_EFFECTIVE};
  size_t cur_resp_ =
      SLOT_NONE;  ///< Step 线程持有的暂存响应槽 / Staging slot the Step thread holds
  size_t send_idx_ = SLOT_NONE;  ///< USB 侧已取出、等 IN 槽的暂存响应 / Staged response
                                 ///< taken by the USB side, waiting for an IN slot
  /// 已发布、还没进端点队列的暂存响应数：为 0 时 Step 才直接提交，响应顺序不变。
  /// Staged responses published but not yet in the endpoint queue: Step submits directly
  /// only when it is zero, so responses keep their order. Word-sized: some targets have
  /// no byte atomics.
  std::atomic<uint32_t> staged_outstanding_{0u};
  Endpoint::Slot cur_in_{};        ///< Step 线程借来直接写响应的 IN 槽 / IN slot the Step
                                   ///< thread borrowed to write the response in place
  Endpoint* cur_in_ep_ = nullptr;  ///< cur_in_ 所属端点 / Endpoint cur_in_ belongs to
  /// 每发布一条命令发一次，Step() 在它上面等 / Posted per published command, Step() waits
  /// on it
  LibXR::Semaphore cmd_sem_{0};
  /// 每提交一个暂存响应发一次（一个暂存槽空出来），Step() 等暂存槽时在它上面等
  /// Posted per submitted staged response (a staging slot came free); Step() waits on it
  /// for a slot
  LibXR::Semaphore slot_sem_{0};
  /// 解绑/重新绑定时加一：Step 线程用它丢弃跨越重绑的响应 / Bumped on unbind/bind: the
  /// Step thread drops a response that spans a rebind
  std::atomic<uint32_t> epoch_{0u};
  uint32_t cur_epoch_ =
      0u;  ///< 手里这条命令开始时的 epoch_ / epoch_ when the command in hand began
  /// 通告给主机的 DAP 包长：不超过配置值，也不超过 IN 槽 / The DAP packet size announced
  /// to the host: not above the configured one and not above an IN slot
  std::atomic<uint16_t> dap_packet_size_{ConfiguredDapPacketSize()};

  // OUT 字节流状态，只在 USB 服务上下文里访问 / OUT byte-stream state, USB service
  // context only
  struct HeldPacket
  {
    Endpoint::Slot slot;
    uint16_t size = 0u;
  };
  HeldPacket held_[2] =
      {};  ///< 命令池满时留住的租约 / Leases kept while the pool is full
  uint8_t held_n_ = 0u;
  size_t armed_sz_[2] = {};  ///< 已排队接收请求的大小，按到达顺序 / Sizes of the queued
                             ///< receive requests in arrival order
  uint8_t armed_n_ = 0u;
  uint16_t rx_off_ =
      0u;  ///< 正在拼装的命令已收字节数 / Bytes of the command being assembled
  uint16_t known_total_ = 0u;  ///< 由内容得知的命令总长，0 为未知 / Total length known
                               ///< from the content, 0 when unknown
  /// 上一条命令以整包结束时的字节数；其后的整包全零视为补零 / Bytes of the previous
  /// command when it ended on a whole packet; a following all-zero whole packet is
  /// padding
  uint16_t pad_base_ = 0u;
  uint16_t pad_skip_ =
      0u;  ///< 这次传输还剩的补零字节 / Padding bytes this transfer still has
  std::atomic<bool> transfer_abort_{false};  ///< DAP_TransferAbort 标志 / Abort flag

  // §10.3-a：延时命令状态。pending_action_ 由 owner 写、Timer 线程读（只读它来
  // 按门铃）；deadline 与响应 stash 在 store（release）之前写好，owner 续做时在
  // load（acquire）之后读回。
  // §10.3-a: delayed-command state. The owner writes pending_action_ and the Timer
  // thread only reads it to ring the service; the deadline and the response stash
  // are written before the store (release) and read back after the load (acquire)
  // when the owner resumes.
  std::atomic<uint8_t> pending_action_{static_cast<uint8_t>(PendingAction::NONE)};
  bool external_active_ =
      false;  ///< ExecuteExternal 正在执行 / ExecuteExternal in progress
  uint16_t external_size_ =
      0u;  ///< ExecuteExternal 收到的响应长度 / Response length it received
  CommandHook command_hook_ = nullptr;  ///< 命令钩子 / Command hook
  void* command_hook_ctx_ = nullptr;    ///< 钩子上下文 / Hook context
  uint32_t action_deadline_ms_ = 0u;    ///< 续做时刻（ms） / Resume deadline (ms)
  uint8_t* deferred_resp_ = nullptr;    ///< 保留的响应缓冲 / Reserved response buffer
  uint16_t deferred_resp_cap_ = 0u;     ///< 保留的响应容量 / Reserved response capacity
  uint8_t swj_pin_sel_ = 0u;            ///< PinWait 引脚选择 / PinWait pin select
  uint8_t swj_pin_expect_ = 0u;         ///< PinWait 期望电平 / PinWait expected level

 private:
  SwdPort& swd_;  ///< SWD link

  LibXR::Debug::Jtag* jtag_ = nullptr;                    ///< JTAG link
  LibXR::Debug::JtagProtocol::ChainConfig jtag_chain_{};  ///< JTAG chain state
  uint8_t jtag_ir_len_[JTAG_MAX_DEVICES] = {};            ///< JTAG IR lengths

  LibXR::GPIO* nreset_gpio_ = nullptr;  ///< Optional nRESET GPIO

  uint8_t swj_shadow_ =
      static_cast<uint8_t>(DapLinkV2Def::DAP_SWJ_SWDIO_TMS |
                           DapLinkV2Def::DAP_SWJ_NRESET);  ///< Shadow SWJ pin levels

  bool last_nreset_level_high_ = true;  ///< Last nRESET level (high = release)

  LibXR::USB::DapLinkV2Def::State dap_state_{};  ///< DAP state
  InfoStrings info_{"XRobot", "DAPLinkV2", "00000001", "2.0.0", "XRUSB",
                    "XRDAP",  "XRobot",    "DAP_DEMO", "0.1.0"};  ///< Info strings

  uint32_t swj_clock_hz_ = 1000000u;  ///< SWJ clock (Hz)

  const char* interface_string_ = nullptr;  ///< Interface string

  Endpoint::EPNumber data_in_ep_num_;   ///< Bulk IN EP number
  Endpoint::EPNumber data_out_ep_num_;  ///< Bulk OUT EP number

  Endpoint* ep_data_in_ = nullptr;   ///< Bulk IN endpoint
  Endpoint* ep_data_out_ = nullptr;  ///< Bulk OUT endpoint

  std::atomic<bool> inited_{false};  ///< Initialized flag
  uint8_t interface_num_ = 0;        ///< Interface number

  LIBXR_PACKED_BEGIN
  /**
   * @brief Descriptor block (Interface + 2x Endpoint)
   *
   */
  struct DapLinkV2DescBlock
  {
    InterfaceDescriptor intf;   ///< Interface descriptor
    EndpointDescriptor ep_out;  ///< OUT endpoint descriptor
    EndpointDescriptor ep_in;   ///< IN endpoint descriptor
  } desc_block_{};
  LIBXR_PACKED_END

 private:
  uint32_t match_mask_ = 0xFFFFFFFFu;  ///< Match mask

  // 完成回调在 owner 里执行，必须保持浅操作；命令处理走 RequestClassService +
  // OnService，响应拷进借来的 IN 槽后 Submit。
  // Completion callbacks run in the owner and must stay shallow; command processing
  // goes through RequestClassService + OnService and responses are copied into a
  // borrowed IN slot and submitted.
  LibXR::Callback<const Endpoint::Completion&> on_data_out_cb_ =
      LibXR::Callback<const Endpoint::Completion&>::Create(OnDataOutCompleteStatic, this);

  LibXR::Callback<const Endpoint::Completion&> on_data_in_cb_ =
      LibXR::Callback<const Endpoint::Completion&>::Create(OnDataInCompleteStatic, this);
};

}  // namespace LibXR::USB
