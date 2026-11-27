#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <initializer_list>

#include "can.hpp"
#include "database.hpp"
#include "device_class.hpp"
#include "ep.hpp"
#include "gpio.hpp"
#include "gs_usb_protocol.hpp"
#include "libxr_cb.hpp"
#include "libxr_def.hpp"
#include "libxr_mem.hpp"
#include "structure/queue/spsc_queue.hpp"
#include "timebase.hpp"
#include "timer.hpp"

namespace LibXR::USB
{

/**
 * @class GsUsbClass
 * @brief Linux gs_usb 协议的 CAN 适配器设备类（经典 CAN 与 CAN FD）/ CAN adapter device
 *        class for the Linux gs_usb protocol (classic CAN and CAN FD)
 * @tparam CanChNum CAN 通道数 / Number of CAN channels
 *
 * 一个厂商接口、两个批量端点。主机经厂商请求设置字节序、定时、模式等，经批量 OUT 发帧，
 * 经批量 IN 收帧和发帧的回显。一帧是一次批量传输：主机到设备 20 或 76 字节，设备到主机
 * 20、24、76 或 80 字节；一帧可跨多个包，以短包结束。
 * One vendor interface and two bulk endpoints. The host sets byte order, timing, mode
 * and so on with vendor requests, sends frames on bulk OUT and receives frames and the
 * echoes of its own frames on bulk IN. One frame is one bulk transfer: 20 or 76 bytes
 * from the host, 20, 24, 76 or 80 bytes to the host; a frame may span several packets
 * and ends with a short packet.
 *
 * 发往主机：每个通道一个接收队列和一个错误帧队列（生产者是 CAN 回调），一个回显队列
 * （生产者是 OUT 完成）。生产者只入队并 RequestService；owner 借 IN 槽、选帧、打包、
 * 提交。回显与接收帧在两者都有待发时轮流发送，只有一类时直接发送，接收帧在通道间轮流。
 * 接收队列满时丢帧并按通道计数，溢出标志随该通道下一帧上报。
 * To the host: per channel one receive queue and one error-frame queue (filled by the
 * CAN callbacks), and one echo queue (filled by the OUT completion). Producers only
 * enqueue and call RequestService; the owner borrows an IN slot, picks a frame, packs
 * and submits it. Echoes and received frames alternate while both are pending and go out
 * directly when only one kind is, with received frames drained round-robin over the
 * channels. A full receive queue drops the frame and counts it per channel, and the
 * overflow flag rides on that channel's next frame.
 *
 * 会话边界与发送暂存：通道复位（MODE RESET）或启动（MODE START）时丢弃该通道上一次
 * 会话留下的接收帧、错误帧、回显、被回显队列压住的回显以及暂存的主机帧；没有启动的
 * 通道的帧不再交给 IN 端点；最后一个启动通道复位时 Flush 共享 IN 端点。CAN 发送队列
 * 满（AddMessage 返回 FULL）时将主机帧暂存进该通道自己的暂存队列，不回 echo、不占用
 * OUT 槽，后续由服务与 SOF 按顺序重试，驱动接受后再回 echo；若暂存队列满，则按原策略
 * 丢弃并照常回 echo。
 * Session boundaries and transmit staging: a channel reset (MODE RESET) or start
 * (MODE START) drops that channel's queued receive frames, error frames, echoes, any
 * echo held by echo-queue back-pressure and staged host frames; frames of an unstarted
 * channel are never handed to IN, and resetting the last started channel flushes the
 * shared IN endpoint. When a CAN transmit queue is full (AddMessage returns FULL), the
 * host frame is staged in that channel's own staging queue without holding the OUT slot
 * or sending an echo yet; service and SOF retry in order and emit the echo once the
 * driver accepts the frame (if the staging queue overflows, the excess frame is dropped
 * and echoed immediately).
 *
 * 来自主机：任何时刻只有一个 OUT 接收在途，长度为 MaxTransferSize()（多个包），主机的
 * 短包结束请求，所以一次完成恰好是一帧。帧先交给 CAN，再把回显入队；回显队列满时留着
 * 这个槽（主机被 NAK），owner 腾出空间后补推回显、交回槽、再挂接收。
 * From the host: exactly one OUT receive is armed at any time, MaxTransferSize() long
 * (several packets); the host's short packet ends the request, so one completion is
 * exactly one frame. The frame goes to CAN first and its echo is queued next; with the
 * echo queue full the slot is kept (the host is NAKed), and once the owner frees space
 * it pushes the echo, hands the slot back and arms the receive again.
 *
 * @note 除注明"任何上下文"的函数外，本类的函数都只在设备的 owner 中运行。端点指针非空
 *       当且仅当已绑定；它们只由 owner 写和读。CAN 回调在 CAN 驱动的上下文运行，只读
 *       原子的通道开关、写本通道的队列和丢帧计数。
 *       Except for the functions marked "any context", everything here runs in the
 *       device's owner. The endpoint pointers are non-null exactly while bound; only the
 *       owner writes and reads them. The CAN callbacks run in the CAN driver's context
 *       and only read the atomic channel switches and write their channel's queues and
 *       drop count.
 */
template <size_t CanChNum>
class GsUsbClass : public DeviceClass
{
 public:
  static_assert(CanChNum > 0U && CanChNum <= 255U, "CanChNum must be in [1, 255]");

  /// CAN 通道数 / Number of CAN channels
  static constexpr uint8_t CAN_CH_NUM = static_cast<uint8_t>(CanChNum);
  /// 默认接口字符串 / Default interface string
  static constexpr const char* DEFAULT_INTERFACE_STRING = "XRUSB GS USB";
  /// USER_ID 写回数据库的周期（毫秒）/ Period of the USER_ID write-back (ms)
  static constexpr uint32_t USER_ID_WRITE_PERIOD_MS = 100U;
  /// 每通道错误帧队列容量 / Error-frame queue capacity per channel
  static constexpr size_t ERR_QUEUE_SIZE = 8U;
  /// 每通道发送暂存队列默认容量（Linux gs_usb 驱动每通道最多 10 帧在途）/
  /// Default per-channel TX staging capacity (Linux gs_usb in-flight limit per channel)
  static constexpr size_t DEFAULT_TX_STAGE_SIZE = 10U;

  /**
   * @brief 构造：经典 CAN / Construct: classic CAN
   * @param data_in_ep_num 批量 IN 端点号 / Bulk IN endpoint number
   * @param data_out_ep_num 批量 OUT 端点号 / Bulk OUT endpoint number
   * @param cans CAN 列表，个数等于 CAN_CH_NUM，不得为空指针 / CAN list, CAN_CH_NUM
   *        entries, no null pointer
   * @param rx_queue_size 每通道接收队列容量 / Receive queue capacity per channel
   * @param echo_queue_size 回显队列容量 / Echo queue capacity
   * @param identify_gpio IDENTIFY 指示 GPIO，可为空 / IDENTIFY GPIO, may be null
   * @param termination_gpios 每通道终端电阻 GPIO：空列表或 CAN_CH_NUM 个（可含空指针）/
   *        Per-channel termination GPIOs: empty or CAN_CH_NUM entries (null allowed)
   * @param database 保存 USER_ID 的数据库，可为空 / Database keeping USER_ID, may be null
   * @param interface_string 接口字符串 / Interface string
   * @param tx_stage_size 每通道发送暂存队列容量（至少 1）/ Per-channel TX staging queue
   *        capacity (at least 1)
   * @note 在 CAN 上登记接收回调、分配队列；有数据库时建 USER_ID 键并启动写回定时任务。
   *       只在构造时分配内存。
   *       Registers the receive callbacks on the CANs and allocates the queues; with a
   *       database, creates the USER_ID key and starts the write-back timer task.
   *       Allocates memory at construction only.
   */
  GsUsbClass(Endpoint::EPNumber data_in_ep_num, Endpoint::EPNumber data_out_ep_num,
             std::initializer_list<LibXR::CAN*> cans, size_t rx_queue_size = 32U,
             size_t echo_queue_size = 32U, LibXR::GPIO* identify_gpio = nullptr,
             std::initializer_list<LibXR::GPIO*> termination_gpios = {},
             LibXR::Database* database = nullptr,
             const char* interface_string = DEFAULT_INTERFACE_STRING,
             size_t tx_stage_size = DEFAULT_TX_STAGE_SIZE)
      : data_in_num_(data_in_ep_num),
        data_out_num_(data_out_ep_num),
        interface_string_(interface_string),
        identify_gpio_(identify_gpio),
        echo_queue_(echo_queue_size)
  {
    ASSERT(cans.size() == CAN_CH_NUM);
    size_t i = 0U;
    for (LibXR::CAN* can : cans)
    {
      ASSERT(can != nullptr);
      cans_[i++] = can;
    }
    Init(rx_queue_size, termination_gpios, database, tx_stage_size);
  }

  /**
   * @brief 构造：CAN FD / Construct: CAN FD
   * @param data_in_ep_num 批量 IN 端点号 / Bulk IN endpoint number
   * @param data_out_ep_num 批量 OUT 端点号 / Bulk OUT endpoint number
   * @param fd_cans FDCAN 列表，个数等于 CAN_CH_NUM，不得为空指针 / FDCAN list,
   *        CAN_CH_NUM entries, no null pointer
   * @param rx_queue_size 每通道接收队列容量 / Receive queue capacity per channel
   * @param echo_queue_size 回显队列容量 / Echo queue capacity
   * @param identify_gpio IDENTIFY 指示 GPIO，可为空 / IDENTIFY GPIO, may be null
   * @param termination_gpios 每通道终端电阻 GPIO：空列表或 CAN_CH_NUM 个（可含空指针）/
   *        Per-channel termination GPIOs: empty or CAN_CH_NUM entries (null allowed)
   * @param database 保存 USER_ID 的数据库，可为空 / Database keeping USER_ID, may be null
   * @param interface_string 接口字符串 / Interface string
   * @param tx_stage_size 每通道发送暂存队列容量（至少 1）/ Per-channel TX staging queue
   *        capacity (at least 1)
   * @note 行为同经典 CAN 的构造；另声明 FD 能力、登记 FD 接收回调。
   *       The behaviour is that of the classic constructor; it also announces the FD
   *       features and registers the FD receive callbacks.
   */
  GsUsbClass(Endpoint::EPNumber data_in_ep_num, Endpoint::EPNumber data_out_ep_num,
             std::initializer_list<LibXR::FDCAN*> fd_cans, size_t rx_queue_size = 32U,
             size_t echo_queue_size = 32U, LibXR::GPIO* identify_gpio = nullptr,
             std::initializer_list<LibXR::GPIO*> termination_gpios = {},
             LibXR::Database* database = nullptr,
             const char* interface_string = DEFAULT_INTERFACE_STRING,
             size_t tx_stage_size = DEFAULT_TX_STAGE_SIZE)
      : fd_supported_(true),
        data_in_num_(data_in_ep_num),
        data_out_num_(data_out_ep_num),
        interface_string_(interface_string),
        identify_gpio_(identify_gpio),
        echo_queue_(echo_queue_size)
  {
    ASSERT(fd_cans.size() == CAN_CH_NUM);
    size_t i = 0U;
    for (LibXR::FDCAN* can : fd_cans)
    {
      ASSERT(can != nullptr);
      fdcans_[i] = can;
      cans_[i] = can;
      ++i;
    }
    Init(rx_queue_size, termination_gpios, database, tx_stage_size);
  }

  /// 主机是否已用小端标记协商字节序；任何上下文 / Whether the host negotiated the byte
  /// order with the little-endian marker; any context
  bool IsHostFormatOK() const
  {
    return host_format_ok_.load(std::memory_order_acquire) != 0U;
  }

 protected:
  /// @name DeviceClass 的实现 / DeviceClass implementation
  /// @{

  size_t GetInterfaceCount() const override { return 1U; }
  bool HasIAD() const override { return false; }
  size_t GetMaxConfigSize() const override { return sizeof(desc_block_); }

  const char* GetInterfaceString(size_t local_interface_index) const override
  {
    return local_interface_index == 0U ? interface_string_ : nullptr;
  }

  /**
   * @brief 写厂商接口和两个批量端点描述符 / Write the vendor interface and the two bulk
   *        endpoint descriptors
   * @return OK；NOT_SUPPORT：某个端点在该速度下查询得 0。
   *         OK; NOT_SUPPORT: an endpoint's query returned 0 at that speed.
   */
  ErrorCode WriteDescriptors(const EndpointPool& pool, Speed speed,
                             uint8_t first_interface, ConstRawData& block) override
  {
    bool ok = WriteEndpointDescriptor(pool, speed, data_in_num_, DATA_IN_CONFIG, 0U,
                                      desc_block_.ep_in);
    ok = WriteEndpointDescriptor(pool, speed, data_out_num_, DATA_OUT_CONFIG, 0U,
                                 desc_block_.ep_out) &&
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
                        VENDOR_CLASS,
                        VENDOR_CLASS,
                        VENDOR_CLASS,
                        GetInterfaceStringIndex(0U)};
    block = {&desc_block_, sizeof(desc_block_)};
    return ErrorCode::OK;
  }

  /**
   * @brief 取两个端点、Configure、比较包长、挂第一个接收 / Take both endpoints,
   *        configure them, compare the sizes, arm the first receive
   * @return OK；NOT_FOUND：Get 失败；CHECK_ERR：读回的包长与描述符不同；NO_BUFF：某个
   *         端点一次请求装不下最长的一帧（MaxTransferSize() 小于 80）；其他：Configure
   *         的错误。失败时已取的端点留在成员里，由随后的 UnbindEndpoints 收回。
   *         OK; NOT_FOUND: a Get failed; CHECK_ERR: a size read back differs from the
   *         descriptor; NO_BUFF: one request of an endpoint cannot hold the longest
   *         frame (MaxTransferSize() below 80); otherwise the error of Configure. On
   *         failure the endpoints taken stay in the members and the following
   *         UnbindEndpoints takes them back.
   * @note 队列里留下的帧属于上一次绑定（或解绑那一刻的 CAN 回调），在这里丢弃。
   *       Frames left in the queues belong to the previous binding (or to a CAN
   *       callback racing the unbind) and are dropped here.
   */
  [[nodiscard]] ErrorCode BindEndpoints(EndpointPool& pool, uint8_t first_interface,
                                        bool in_isr) override
  {
    UNUSED(first_interface);
    ErrorCode result = BindEndpoint(pool, data_in_num_, DATA_IN_CONFIG,
                                    desc_block_.ep_in.wMaxPacketSize, in_isr, data_in_);
    if (result == ErrorCode::OK)
    {
      result = BindEndpoint(pool, data_out_num_, DATA_OUT_CONFIG,
                            desc_block_.ep_out.wMaxPacketSize, in_isr, data_out_);
    }
    const size_t min_transfer = HostBufferSize(fd_supported_);
    if (result == ErrorCode::OK && (data_in_->MaxTransferSize() < min_transfer ||
                                    data_out_->MaxTransferSize() < min_transfer))
    {
      // 一帧必须是一个请求：拆成两个请求会让主机和设备对帧边界的理解错开。
      // One frame must be one request: split over two, host and device would disagree
      // on the frame boundaries.
      result = ErrorCode::NO_BUFF;
    }
    if (result != ErrorCode::OK)
    {
      return result;
    }
    DiscardQueuedFrames();
    data_in_->SetOnComplete(data_in_cb_);
    data_out_->SetOnComplete(data_out_cb_);
    ArmOut(in_isr);
    return ErrorCode::OK;
  }

  /**
   * @brief 停用全部通道，关闭并交回端点，交回留住的 OUT 槽 / Disable every channel,
   *        close and return the endpoints, hand back the OUT slot kept
   * @note 先清通道开关，CAN 回调随即停止入队。先把指针置空再 Close，所以 Close 交付的
   *       完成回调看到"未绑定"，什么也不做。被压住的回显随这次绑定丢弃。终端电阻状态
   *       保留，与 GPIO 一致。
   *       The channel switches are cleared first, so the CAN callbacks stop queueing.
   *       The pointer is cleared before Close, so the completions Close delivers see
   *       "unbound" and do nothing. A held-back echo is dropped with this binding. The
   *       termination state is kept, matching the GPIO.
   */
  void UnbindEndpoints(EndpointPool& pool, bool in_isr) override
  {
    host_format_ok_.store(0U, std::memory_order_release);
    for (uint8_t ch = 0U; ch < CAN_CH_NUM; ++ch)
    {
      can_enabled_[ch].store(0U, std::memory_order_release);
      fd_enabled_[ch].store(0U, std::memory_order_release);
      berr_enabled_[ch].store(0U, std::memory_order_release);
      timestamps_enabled_[ch].store(0U, std::memory_order_release);
    }
    for (uint8_t ch = 0U; ch < CAN_CH_NUM; ++ch)
    {
      DropStagedFrames(ch);
    }
    Endpoint* out = data_out_;
    UnbindEndpoint(pool, data_in_, in_isr);
    UnbindEndpoint(pool, data_out_, in_isr);
    if (held_slot_.Valid())
    {
      // Close 使这个槽作废，交回得 OK 并释放它。
      // Close made the slot stale; handing it back returns OK and frees it.
      ASSERT(out != nullptr);
      const ErrorCode result = out->ReleaseSlot(held_slot_);
      ASSERT(result == ErrorCode::OK);
      UNUSED(result);
      held_slot_ = Endpoint::Slot{};
    }
  }

  /// 两个端点地址之一；由端点号算出，不需要已绑定 / One of the two endpoint
  /// addresses; computed from the numbers, no bind needed
  bool OwnsEndpoint(uint8_t ep_addr) const override
  {
    return ep_addr == Endpoint::EPNumberToAddr(data_in_num_, Endpoint::Direction::IN) ||
           ep_addr == Endpoint::EPNumberToAddr(data_out_num_, Endpoint::Direction::OUT);
  }

  /**
   * @brief gs_usb 厂商请求的 SETUP 阶段 / SETUP stage of a gs_usb vendor request
   * @return 只接受接收者为接口的厂商请求。读请求必须是 IN 且 wLength 不小于应答；写请求
   *         必须是 OUT、wLength 等于结构体大小，按通道的请求 wValue 必须是有效通道；否则
   *         ARG_ERR。FD 请求在经典 CAN 上和未知请求给 NOT_SUPPORT。核心都 STALL。
   *         Only vendor requests whose recipient is the interface. A read must be IN with
   *         wLength not below the reply; a write must be OUT with wLength equal to the
   *         structure, and a per-channel request needs a valid channel in wValue;
   *         otherwise ARG_ERR. FD requests on classic CAN and unknown requests get
   *         NOT_SUPPORT. The core stalls on both.
   * @note 写请求的数据在 OnControlData 生效。会变的应答（时间戳、终端电阻、状态、
   *       USER_ID）在这里拷进 reply_，到下一次请求前不变。
   *       Writes take effect in OnControlData. Replies that change (timestamp,
   *       termination, state, USER_ID) are copied into reply_ here and stay unchanged
   *       until the next request.
   */
  ErrorCode OnControlRequest(bool in_isr, const SetupPacket& setup,
                             ConstRawData& in_data) override
  {
    UNUSED(in_isr);
    const auto type = static_cast<RequestType>(setup.bmRequestType & REQ_TYPE_MASK);
    const auto recipient =
        static_cast<Recipient>(setup.bmRequestType & REQ_RECIPIENT_MASK);
    if (type != RequestType::VENDOR || recipient != Recipient::INTERFACE)
    {
      return ErrorCode::NOT_SUPPORT;
    }
    const bool channel_ok = setup.wValue < CAN_CH_NUM;
    switch (static_cast<GsUsb::BReq>(setup.bRequest))
    {
      case GsUsb::BReq::BT_CONST:
        return Reply(setup, &bt_const_, sizeof(bt_const_), in_data);

      case GsUsb::BReq::BT_CONST_EXT:
        if (!fd_supported_)
        {
          return ErrorCode::NOT_SUPPORT;
        }
        return Reply(setup, &bt_const_ext_, sizeof(bt_const_ext_), in_data);

      case GsUsb::BReq::DEVICE_CONFIG:
        return Reply(setup, &dev_cfg_, sizeof(dev_cfg_), in_data);

      case GsUsb::BReq::TIMESTAMP:
        reply_.timestamp_us = static_cast<uint32_t>(LibXR::Timebase::GetMicroseconds());
        return Reply(setup, &reply_.timestamp_us, sizeof(reply_.timestamp_us), in_data);

      case GsUsb::BReq::GET_TERMINATION:
        if (!channel_ok)
        {
          return ErrorCode::ARG_ERR;
        }
        reply_.term.state = static_cast<uint32_t>(term_state_[setup.wValue]);
        return Reply(setup, &reply_.term, sizeof(reply_.term), in_data);

      case GsUsb::BReq::GET_STATE:
        if (!channel_ok)
        {
          return ErrorCode::ARG_ERR;
        }
        reply_.state = ReadState(static_cast<uint8_t>(setup.wValue));
        return Reply(setup, &reply_.state, sizeof(reply_.state), in_data);

      case GsUsb::BReq::GET_USER_ID:
        reply_.user_id = user_id_.load(std::memory_order_relaxed);
        return Reply(setup, &reply_.user_id, sizeof(reply_.user_id), in_data);

      case GsUsb::BReq::HOST_FORMAT:
        return Expect(setup, sizeof(GsUsb::HostConfig), true);

      case GsUsb::BReq::BITTIMING:
        return Expect(setup, sizeof(GsUsb::DeviceBitTiming), channel_ok);

      case GsUsb::BReq::DATA_BITTIMING:
        if (!fd_supported_)
        {
          return ErrorCode::NOT_SUPPORT;
        }
        return Expect(setup, sizeof(GsUsb::DeviceBitTiming), channel_ok);

      case GsUsb::BReq::MODE:
        return Expect(setup, sizeof(GsUsb::DeviceMode), channel_ok);

      case GsUsb::BReq::BERR:
        return Expect(setup, sizeof(uint32_t), channel_ok);

      case GsUsb::BReq::IDENTIFY:
        return Expect(setup, sizeof(GsUsb::Identify), channel_ok);

      case GsUsb::BReq::SET_TERMINATION:
        return Expect(setup, sizeof(GsUsb::DeviceTerminationState), channel_ok);

      case GsUsb::BReq::SET_USER_ID:
        return Expect(setup, sizeof(uint32_t), true);

      default:
        return ErrorCode::NOT_SUPPORT;
    }
  }

  /**
   * @brief 写请求的数据阶段 / Data stage of a write request
   * @return OK；ARG_ERR：长度不对，或 BITTIMING、DATA_BITTIMING、MODE 在字节序协商之前，
   *         或 MODE 不是 RESET/START；其他：CAN 的 SetConfig 的错误。
   *         OK; ARG_ERR: wrong length, or BITTIMING, DATA_BITTIMING or MODE before the
   *         byte order negotiation, or a MODE other than RESET/START; otherwise the
   *         error of the CAN's SetConfig.
   * @note 通道取 wValue，SETUP 阶段已检查。
   *       The channel is wValue, checked in the SETUP stage.
   */
  ErrorCode OnControlData(bool in_isr, const SetupPacket& setup,
                          ConstRawData data) override
  {
    const auto ch = static_cast<uint8_t>(setup.wValue);
    switch (static_cast<GsUsb::BReq>(setup.bRequest))
    {
      case GsUsb::BReq::HOST_FORMAT:
      {
        GsUsb::HostConfig cfg{};
        if (!Load(data, cfg))
        {
          return ErrorCode::ARG_ERR;
        }
        host_format_ok_.store(cfg.byte_order == HOST_BYTE_ORDER ? 1U : 0U,
                              std::memory_order_release);
        return ErrorCode::OK;
      }

      case GsUsb::BReq::BITTIMING:
      {
        GsUsb::DeviceBitTiming bt{};
        return Load(data, bt) ? HandleBitTiming(ch, bt) : ErrorCode::ARG_ERR;
      }

      case GsUsb::BReq::DATA_BITTIMING:
      {
        GsUsb::DeviceBitTiming bt{};
        return Load(data, bt) ? HandleDataBitTiming(ch, bt) : ErrorCode::ARG_ERR;
      }

      case GsUsb::BReq::MODE:
      {
        GsUsb::DeviceMode mode{};
        return Load(data, mode) ? HandleMode(in_isr, ch, mode) : ErrorCode::ARG_ERR;
      }

      case GsUsb::BReq::BERR:
      {
        uint32_t on = 0U;
        if (!Load(data, on))
        {
          return ErrorCode::ARG_ERR;
        }
        berr_enabled_[ch].store(on != 0U ? 1U : 0U, std::memory_order_release);
        return ErrorCode::OK;
      }

      case GsUsb::BReq::IDENTIFY:
      {
        GsUsb::Identify identify{};
        if (!Load(data, identify))
        {
          return ErrorCode::ARG_ERR;
        }
        if (identify_gpio_ != nullptr)
        {
          (void)identify_gpio_->Write(identify.mode ==
                                      static_cast<uint32_t>(GsUsb::IdentifyMode::ON));
        }
        return ErrorCode::OK;
      }

      case GsUsb::BReq::SET_TERMINATION:
      {
        GsUsb::DeviceTerminationState st{};
        if (!Load(data, st))
        {
          return ErrorCode::ARG_ERR;
        }
        const bool on = st.state != 0U;
        term_state_[ch] = on ? GsUsb::TerminationState::ON : GsUsb::TerminationState::OFF;
        if (termination_gpios_[ch] != nullptr)
        {
          (void)termination_gpios_[ch]->Write(on);
        }
        return ErrorCode::OK;
      }

      case GsUsb::BReq::SET_USER_ID:
      {
        uint32_t user_id = 0U;
        if (!Load(data, user_id))
        {
          return ErrorCode::ARG_ERR;
        }
        // 这里只改内存值并置待写标志，数据库由定时任务写。
        // Only the value in memory and the pending flag change here; the timer task
        // writes the database.
        user_id_.store(user_id, std::memory_order_relaxed);
        user_id_write_pending_.store(1U, std::memory_order_release);
        return ErrorCode::OK;
      }

      default:
        return ErrorCode::NOT_SUPPORT;
    }
  }

  /// 补推被压住的回显，再把排队的帧发往主机 / Push a held-back echo, then send queued
  /// frames to the host
  void OnService(bool in_isr) override { Pump(in_isr); }

  /// SOF 节拍：仅在有发送暂存帧时重试 / SOF tick: retry only while frames are staged
  void OnStartOfFrame(bool in_isr, uint16_t frame, uint8_t microframe) override
  {
    UNUSED(frame);
    UNUSED(microframe);
    if (stage_pending_)
    {
      Pump(in_isr);
    }
  }

  /// @}

 private:
  LIBXR_PACKED_BEGIN
  /// gs_usb 帧头，12 字节 / gs_usb frame header, 12 bytes
  struct WireHeader
  {
    uint32_t echo_id;  ///< 回显号；设备收到的帧为 ECHO_ID_RX / Echo ID; ECHO_ID_RX for RX
    uint32_t can_id;   ///< CAN ID 及标志 / CAN ID with flags
    uint8_t can_dlc;   ///< DLC
    uint8_t channel;   ///< 通道 / Channel
    uint8_t flags;     ///< GsUsb::CAN_FLAG_*
    uint8_t reserved;  ///< 保留 / Reserved
  };

  /// 本类的描述符块 / This class's descriptor block
  struct DescriptorBlock
  {
    InterfaceDescriptor intf;   ///< 厂商接口 / Vendor interface
    EndpointDescriptor ep_out;  ///< 批量 OUT / Bulk OUT
    EndpointDescriptor ep_in;   ///< 批量 IN / Bulk IN
  };
  LIBXR_PACKED_END

  /// 发往主机的一帧；FD 由 hdr.flags 的 CAN_FLAG_FD 表示 / One frame for the host; FD
  /// is CAN_FLAG_FD in hdr.flags
  /// @note 局部的 QueueItem 不清零（每帧都要建一个）：Pack 发出 data 的前 8（经典）或
  ///       64（FD）字节，填数据的地方把长度之后的字节补 0。
  ///       A local QueueItem is not zeroed (one is made per frame): Pack sends the first
  ///       8 (classic) or 64 (FD) bytes of data, and whoever fills the data zeroes the
  ///       bytes past its length.
  struct QueueItem
  {
    WireHeader hdr;         ///< 帧头 / Header
    uint32_t timestamp_us;  ///< 入队时的时间戳 / Timestamp taken at enqueue
    uint8_t data[64];       ///< 数据：经典 8 字节，FD 64 字节 / Data: 8 classic, 64 FD
  };

  /// CAN 回调的参数 / Argument of a CAN callback
  struct ChannelContext
  {
    GsUsbClass* self;  ///< 本类 / This class
    uint8_t channel;   ///< 通道 / Channel
  };

  /// 一次控制请求的 IN 应答；OnControlRequest 写 / IN reply of one control request;
  /// written by OnControlRequest
  union ReplyBuffer
  {
    uint32_t timestamp_us;               ///< TIMESTAMP
    GsUsb::DeviceTerminationState term;  ///< GET_TERMINATION
    GsUsb::DeviceState state;            ///< GET_STATE
    uint32_t user_id;                    ///< GET_USER_ID
  };

  static constexpr size_t WIRE_HDR_SIZE = sizeof(WireHeader);  ///< 帧头 / Header
  static constexpr size_t WIRE_CLASSIC_DATA_SIZE = 8U;  ///< 经典数据 / Classic data
  static constexpr size_t WIRE_FD_DATA_SIZE = 64U;      ///< FD 数据 / FD data
  static constexpr size_t WIRE_TS_SIZE = 4U;            ///< 时间戳 / Timestamp
  /// 经典帧，20 字节 / Classic frame, 20 bytes
  static constexpr size_t WIRE_CLASSIC_SIZE = WIRE_HDR_SIZE + WIRE_CLASSIC_DATA_SIZE;
  /// FD 帧，76 字节 / FD frame, 76 bytes
  static constexpr size_t WIRE_FD_SIZE = WIRE_HDR_SIZE + WIRE_FD_DATA_SIZE;
  /// 最长的帧（FD 加时间戳），80 字节 / Longest frame (FD with timestamp), 80 bytes
  static constexpr size_t WIRE_MAX_SIZE = WIRE_FD_SIZE + WIRE_TS_SIZE;
  /// 主机接收缓冲的大小：设备能发的最大帧，带时间戳 / Size of the host's receive
  /// buffer: the largest frame the device can send, with its timestamp
  static constexpr size_t HostBufferSize(bool fd)
  {
    return (fd ? WIRE_FD_SIZE : WIRE_CLASSIC_SIZE) + WIRE_TS_SIZE;
  }
  /// 设备收到的帧的回显号 / Echo ID of a frame the device received
  static constexpr uint32_t ECHO_ID_RX = 0xFFFFFFFFU;
  /// HOST_FORMAT 的小端标记 / Little-endian marker of HOST_FORMAT
  static constexpr uint32_t HOST_BYTE_ORDER = 0x0000BEEFU;
  /// 厂商类代码 / Vendor-specific class code
  static constexpr uint8_t VENDOR_CLASS = 0xFFU;

  static_assert(WIRE_HDR_SIZE == 12U, "gs_usb header is 12 bytes");

  /// 批量 IN 的配置（包长上限为高速批量）/ Bulk IN configuration (bound: high-speed
  /// bulk)
  static constexpr Endpoint::Config DATA_IN_CONFIG{Endpoint::Direction::IN,
                                                   Endpoint::Type::BULK, 512U};
  /// 批量 OUT 的配置 / Bulk OUT configuration
  static constexpr Endpoint::Config DATA_OUT_CONFIG{Endpoint::Direction::OUT,
                                                    Endpoint::Type::BULK, 512U};

  /// 构造的共同部分：GPIO、队列、能力、CAN 回调、USER_ID / Common part of the
  /// constructors: GPIOs, queues, features, CAN callbacks, USER_ID
  void Init(size_t rx_queue_size, std::initializer_list<LibXR::GPIO*> termination_gpios,
            LibXR::Database* database, size_t tx_stage_size)
  {
    ASSERT(tx_stage_size >= 1U);
    ASSERT(termination_gpios.size() == 0U || termination_gpios.size() == CAN_CH_NUM);
    size_t i = 0U;
    for (LibXR::GPIO* gpio : termination_gpios)
    {
      termination_gpios_[i++] = gpio;
    }
    for (uint8_t ch = 0U; ch < CAN_CH_NUM; ++ch)
    {
      rx_queues_[ch] = new LibXR::SPSCQueue<QueueItem>(rx_queue_size);
      err_queues_[ch] = new LibXR::SPSCQueue<QueueItem>(ERR_QUEUE_SIZE);
      stage_queues_[ch] = new LibXR::SPSCQueue<QueueItem>(tx_stage_size);
    }
    InitFeatures();
    RegisterCanCallbacks();
    InitUserIdStorage(database);
  }

  /// 设备配置和定时能力 / Device configuration and timing capabilities
  void InitFeatures()
  {
    dev_cfg_.icount = static_cast<uint8_t>(CAN_CH_NUM - 1U);
    dev_cfg_.sw_version = 2U;
    dev_cfg_.hw_version = 1U;

    uint32_t feature = GsUsb::CAN_FEAT_LISTEN_ONLY | GsUsb::CAN_FEAT_LOOP_BACK |
                       GsUsb::CAN_FEAT_TRIPLE_SAMPLE | GsUsb::CAN_FEAT_ONE_SHOT |
                       GsUsb::CAN_FEAT_HW_TIMESTAMP | GsUsb::CAN_FEAT_BERR_REPORTING |
                       GsUsb::CAN_FEAT_GET_STATE | GsUsb::CAN_FEAT_USER_ID;
    if (fd_supported_)
    {
      feature |= GsUsb::CAN_FEAT_FD | GsUsb::CAN_FEAT_BT_CONST_EXT;
    }
    if (identify_gpio_ != nullptr)
    {
      feature |= GsUsb::CAN_FEAT_IDENTIFY;
    }
    for (LibXR::GPIO* gpio : termination_gpios_)
    {
      if (gpio != nullptr)
      {
        feature |= GsUsb::CAN_FEAT_TERMINATION;
      }
    }

    const uint32_t fclk = cans_[0]->GetClockFreq();
    bt_const_.feature = feature;
    bt_const_.fclk_can = fclk;
    bt_const_.btc = {1U, 16U, 1U, 8U, 4U, 1U, 1024U, 1U};
    bt_const_ext_.feature = feature;
    bt_const_ext_.fclk_can = fclk;
    bt_const_ext_.btc = bt_const_.btc;
    bt_const_ext_.dbtc = bt_const_.btc;
  }

  /// 在每个 CAN 上登记接收回调（含错误帧，FD 另登记 FD 帧）/ Register the receive
  /// callbacks on each CAN (error frames included, plus FD frames for FD)
  void RegisterCanCallbacks()
  {
    for (uint8_t ch = 0U; ch < CAN_CH_NUM; ++ch)
    {
      contexts_[ch] = {this, ch};
      const auto cb = LibXR::CAN::Callback::Create(OnCanRxStatic, &contexts_[ch]);
      for (LibXR::CAN::Type type :
           {LibXR::CAN::Type::STANDARD, LibXR::CAN::Type::EXTENDED,
            LibXR::CAN::Type::REMOTE_STANDARD, LibXR::CAN::Type::REMOTE_EXTENDED,
            LibXR::CAN::Type::ERROR})
      {
        cans_[ch]->Register(cb, type);
      }
      if (fd_supported_)
      {
        const auto fd_cb =
            LibXR::FDCAN::CallbackFD::Create(OnFdCanRxStatic, &contexts_[ch]);
        fdcans_[ch]->Register(fd_cb, LibXR::CAN::Type::STANDARD);
        fdcans_[ch]->Register(fd_cb, LibXR::CAN::Type::EXTENDED);
      }
    }
  }

  /// 有数据库时建 USER_ID 键、读出初值、启动写回定时任务 / With a database, create the
  /// USER_ID key, load its value and start the write-back timer task
  void InitUserIdStorage(LibXR::Database* database)
  {
    if (database == nullptr)
    {
      return;
    }
    user_id_key_ = new LibXR::Database::Key<uint32_t>(*database, "user_id", 0U);
    user_id_.store(static_cast<uint32_t>(*user_id_key_), std::memory_order_relaxed);
    const auto timer =
        LibXR::Timer::CreateTask(OnUserIdTimerStatic, this, USER_ID_WRITE_PERIOD_MS);
    LibXR::Timer::Add(timer);
    LibXR::Timer::Start(timer);
  }

  /// 定时任务：有待写的 USER_ID 时写数据库 / Timer task: write USER_ID to the database
  /// when one is pending
  static void OnUserIdTimerStatic(GsUsbClass* self)
  {
    if (self->user_id_write_pending_.exchange(0U, std::memory_order_acq_rel) != 0U)
    {
      (void)self->user_id_key_->Set(self->user_id_.load(std::memory_order_relaxed));
    }
  }

  /// IN 读请求：检查方向和长度，指向应答 / IN read request: check direction and length,
  /// point at the reply
  static ErrorCode Reply(const SetupPacket& setup, const void* data, size_t size,
                         ConstRawData& in_data)
  {
    if ((setup.bmRequestType & REQ_DIRECTION_MASK) == 0U || setup.wLength < size)
    {
      return ErrorCode::ARG_ERR;
    }
    in_data = {data, size};
    return ErrorCode::OK;
  }

  /// OUT 写请求：检查方向、长度和通道 / OUT write request: check direction, length and
  /// channel
  static ErrorCode Expect(const SetupPacket& setup, size_t size, bool channel_ok)
  {
    const bool out = (setup.bmRequestType & REQ_DIRECTION_MASK) == 0U;
    return (out && setup.wLength == size && channel_ok) ? ErrorCode::OK
                                                        : ErrorCode::ARG_ERR;
  }

  /// 数据阶段恰好是一个 T 时拷出 / Copy the data stage out when it is exactly one T
  template <typename T>
  static bool Load(ConstRawData data, T& value)
  {
    if (data.size_ != sizeof(T))
    {
      return false;
    }
    std::memcpy(&value, data.addr_, sizeof(T));
    return true;
  }

  /// GET_STATE 的应答：总线状态和错误计数 / GET_STATE reply: bus state and error counts
  GsUsb::DeviceState ReadState(uint8_t ch) const
  {
    GsUsb::DeviceState st{static_cast<uint32_t>(GsUsb::CanState::ERROR_ACTIVE), 0U, 0U};
    LibXR::CAN::ErrorState es{};
    if (cans_[ch]->GetErrorState(es) != ErrorCode::OK)
    {
      return st;
    }
    GsUsb::CanState state = GsUsb::CanState::ERROR_ACTIVE;
    if (es.bus_off)
    {
      state = GsUsb::CanState::BUS_OFF;
    }
    else if (es.error_passive)
    {
      state = GsUsb::CanState::ERROR_PASSIVE;
    }
    else if (es.error_warning)
    {
      state = GsUsb::CanState::ERROR_WARNING;
    }
    st.state = static_cast<uint32_t>(state);
    st.rxerr = es.rx_error_counter;
    st.txerr = es.tx_error_counter;
    return st;
  }

  /// BITTIMING：仲裁相位 / BITTIMING: arbitration phase
  ErrorCode HandleBitTiming(uint8_t ch, const GsUsb::DeviceBitTiming& bt)
  {
    if (!IsHostFormatOK())
    {
      return ErrorCode::ARG_ERR;
    }
    LibXR::CAN::Configuration& cfg = config_[ch];
    cfg.bit_timing = {bt.brp, bt.prop_seg, bt.phase_seg1, bt.phase_seg2, bt.sjw};
    const uint64_t tseg1 = static_cast<uint64_t>(bt.prop_seg) + bt.phase_seg1;
    const uint64_t tq_num = 1ULL + tseg1 + bt.phase_seg2;
    const uint64_t denom = static_cast<uint64_t>(bt.brp) * tq_num;
    if (denom != 0ULL)
    {
      cfg.bitrate = static_cast<uint32_t>(cans_[ch]->GetClockFreq() / denom);
      cfg.sample_point = static_cast<float>(static_cast<double>(1ULL + tseg1) /
                                            static_cast<double>(tq_num));
    }
    else
    {
      cfg.bitrate = 0U;
      cfg.sample_point = 0.0f;
    }
    if (fd_supported_)
    {
      static_cast<LibXR::CAN::Configuration&>(fd_config_[ch]) = cfg;
    }
    return cans_[ch]->SetConfig(cfg);
  }

  /// DATA_BITTIMING：FD 数据相位 / DATA_BITTIMING: FD data phase
  ErrorCode HandleDataBitTiming(uint8_t ch, const GsUsb::DeviceBitTiming& bt)
  {
    if (!IsHostFormatOK())
    {
      return ErrorCode::ARG_ERR;
    }
    LibXR::FDCAN::Configuration& fd_cfg = fd_config_[ch];
    static_cast<LibXR::CAN::Configuration&>(fd_cfg) = config_[ch];
    fd_cfg.data_timing = {bt.brp, bt.prop_seg, bt.phase_seg1, bt.phase_seg2, bt.sjw};
    const uint64_t tseg1 = static_cast<uint64_t>(bt.prop_seg) + bt.phase_seg1;
    const uint64_t tq_num = 1ULL + tseg1 + bt.phase_seg2;
    const uint64_t denom = static_cast<uint64_t>(bt.brp) * tq_num;
    if (denom != 0ULL)
    {
      fd_cfg.data_bitrate = static_cast<uint32_t>(fdcans_[ch]->GetClockFreq() / denom);
      fd_cfg.data_sample_point = static_cast<float>(static_cast<double>(1ULL + tseg1) /
                                                    static_cast<double>(tq_num));
    }
    else
    {
      fd_cfg.data_bitrate = 0U;
      fd_cfg.data_sample_point = 0.0f;
    }
    return fdcans_[ch]->SetConfig(fd_cfg);
  }

  /// MODE：启动或复位通道，并设模式标志 / MODE: start or reset the channel and set the
  /// mode flags
  ErrorCode HandleMode(bool in_isr, uint8_t ch, const GsUsb::DeviceMode& mode)
  {
    if (!IsHostFormatOK())
    {
      return ErrorCode::ARG_ERR;
    }
    bool start = false;
    switch (static_cast<GsUsb::CanMode>(mode.mode))
    {
      case GsUsb::CanMode::RESET:
        break;
      case GsUsb::CanMode::START:
        start = true;
        break;
      default:
        return ErrorCode::ARG_ERR;
    }
    const bool fd = fd_supported_ && (mode.flags & GsUsb::GSCAN_MODE_FD) != 0U;
    const uint32_t prev_can_enabled = can_enabled_[ch].load(std::memory_order_relaxed);
    const uint32_t prev_fd_enabled = fd_enabled_[ch].load(std::memory_order_relaxed);
    const uint32_t prev_berr_enabled = berr_enabled_[ch].load(std::memory_order_relaxed);
    // RESET 和重复 START 都先停收再 SetConfig/清队列，防止配置与清理间隙 CAN
    // 回调继续入旧帧。 Stop callbacks before SetConfig and queue drop on both RESET and
    // repeated START so CAN ISRs cannot enqueue stale-config frames during the
    // transition.
    can_enabled_[ch].store(0U, std::memory_order_release);
    fd_enabled_[ch].store(0U, std::memory_order_release);
    berr_enabled_[ch].store(0U, std::memory_order_release);

    LibXR::CAN::Configuration cfg = config_[ch];
    cfg.mode.loopback = (mode.flags & GsUsb::GSCAN_MODE_LOOP_BACK) != 0U;
    cfg.mode.listen_only = (mode.flags & GsUsb::GSCAN_MODE_LISTEN_ONLY) != 0U;
    cfg.mode.triple_sampling = (mode.flags & GsUsb::GSCAN_MODE_TRIPLE_SAMPLE) != 0U;
    cfg.mode.one_shot = (mode.flags & GsUsb::GSCAN_MODE_ONE_SHOT) != 0U;

    ErrorCode result = cans_[ch]->SetConfig(cfg);
    if (result != ErrorCode::OK)
    {
      berr_enabled_[ch].store(prev_berr_enabled, std::memory_order_release);
      fd_enabled_[ch].store(prev_fd_enabled, std::memory_order_release);
      can_enabled_[ch].store(prev_can_enabled, std::memory_order_release);
      return result;
    }
    if (fd_supported_)
    {
      LibXR::FDCAN::Configuration fd_cfg = fd_config_[ch];
      static_cast<LibXR::CAN::Configuration&>(fd_cfg) = cfg;
      fd_cfg.fd_mode.fd_enabled = fd;
      result = fdcans_[ch]->SetConfig(fd_cfg);
      if (result != ErrorCode::OK)
      {
        (void)cans_[ch]->SetConfig(config_[ch]);
        berr_enabled_[ch].store(prev_berr_enabled, std::memory_order_release);
        fd_enabled_[ch].store(prev_fd_enabled, std::memory_order_release);
        can_enabled_[ch].store(prev_can_enabled, std::memory_order_release);
        return result;
      }
      fd_config_[ch] = fd_cfg;
    }
    config_[ch] = cfg;
    DropStagedFrames(ch);
    DropChannelFrames(in_isr, ch);
    timestamps_enabled_[ch].store(
        (mode.flags & GsUsb::GSCAN_MODE_HW_TIMESTAMP) != 0U ? 1U : 0U,
        std::memory_order_release);
    if (start)
    {
      berr_enabled_[ch].store(
          (mode.flags & GsUsb::GSCAN_MODE_BERR_REPORTING) != 0U ? 1U : 0U,
          std::memory_order_release);
      fd_enabled_[ch].store(fd ? 1U : 0U, std::memory_order_release);
      can_enabled_[ch].store(1U, std::memory_order_release);
    }
    else if (!AnyChannelStarted())
    {
      FlushInEndpoint(in_isr);
    }
    return ErrorCode::OK;
  }

  /// @name 发往主机 / To the host
  /// @{

  /// 经典 CAN 回调入口；CAN 驱动的上下文 / Classic CAN callback entry; the CAN driver's
  /// context
  static void OnCanRxStatic(bool in_isr, ChannelContext* ctx,
                            const LibXR::CAN::ClassicPack& pack)
  {
    ctx->self->OnCanRx(in_isr, ctx->channel, pack);
  }

  /// FD 回调入口；CAN 驱动的上下文 / FD callback entry; the CAN driver's context
  static void OnFdCanRxStatic(bool in_isr, ChannelContext* ctx,
                              const LibXR::FDCAN::FDPack& pack)
  {
    ctx->self->OnFdCanRx(in_isr, ctx->channel, pack);
  }

  /// 收到经典帧或错误帧：入本通道的队列 / A classic or error frame arrived: queue it on
  /// its channel
  void OnCanRx(bool in_isr, uint8_t ch, const LibXR::CAN::ClassicPack& pack)
  {
    QueueItem item;
    if (pack.type == LibXR::CAN::Type::ERROR)
    {
      // 错误帧来自 CAN 的错误中断，与接收中断不一定同优先级，所以有自己的队列，每个队列
      // 只有一个生产者。
      // Error frames come from the CAN error interrupt, whose priority may differ from
      // the receive interrupt's, so they have their own queue and each queue has one
      // producer.
      if (MakeErrorFrame(ch, pack, item) && err_queues_[ch]->Push(item) == ErrorCode::OK)
      {
        RequestService(in_isr);
      }
      return;
    }
    if (can_enabled_[ch].load(std::memory_order_acquire) == 0U)
    {
      return;
    }
    uint32_t id = 0U;
    switch (pack.type)
    {
      case LibXR::CAN::Type::EXTENDED:
        id = (pack.id & GsUsb::CAN_EFF_MASK) | GsUsb::CAN_EFF_FLAG;
        break;
      case LibXR::CAN::Type::REMOTE_STANDARD:
        id = (pack.id & GsUsb::CAN_SFF_MASK) | GsUsb::CAN_RTR_FLAG;
        break;
      case LibXR::CAN::Type::REMOTE_EXTENDED:
        id = (pack.id & GsUsb::CAN_EFF_MASK) | GsUsb::CAN_EFF_FLAG | GsUsb::CAN_RTR_FLAG;
        break;
      default:
        id = pack.id & GsUsb::CAN_SFF_MASK;
        break;
    }
    const uint8_t dlc = pack.dlc <= 8U ? pack.dlc : 8U;
    item.hdr = {ECHO_ID_RX, id, dlc, ch, 0U, 0U};
    item.timestamp_us = MakeTimestampUs(ch);
    std::memcpy(item.data, pack.data, dlc);
    std::memset(item.data + dlc, 0, WIRE_CLASSIC_DATA_SIZE - dlc);
    PushRx(in_isr, ch, item);
  }

  /// 收到 FD 帧：入本通道的队列 / An FD frame arrived: queue it on its channel
  void OnFdCanRx(bool in_isr, uint8_t ch, const LibXR::FDCAN::FDPack& pack)
  {
    if (fd_enabled_[ch].load(std::memory_order_acquire) == 0U)
    {
      return;
    }
    uint32_t id = 0U;
    if (pack.type == LibXR::CAN::Type::EXTENDED)
    {
      id = (pack.id & GsUsb::CAN_EFF_MASK) | GsUsb::CAN_EFF_FLAG;
    }
    else
    {
      id = pack.id & GsUsb::CAN_SFF_MASK;
    }
    const uint8_t len = pack.len <= 64U ? pack.len : 64U;
    QueueItem item;
    item.hdr = {ECHO_ID_RX, id, LenToDlc(len), ch, GsUsb::CAN_FLAG_FD, 0U};
    item.timestamp_us = MakeTimestampUs(ch);
    LibXR::Memory::FastCopy(item.data, pack.data, len);
    std::memset(item.data + len, 0, WIRE_FD_DATA_SIZE - len);
    PushRx(in_isr, ch, item);
  }

  /// 入接收队列；满时丢帧并计数 / Queue on the receive queue; when full, drop and count
  void PushRx(bool in_isr, uint8_t ch, const QueueItem& item)
  {
    if (rx_queues_[ch]->Push(item) != ErrorCode::OK)
    {
      rx_drop_[ch].fetch_add(1U, std::memory_order_relaxed);
      return;
    }
    RequestService(in_isr);
  }

  /**
   * @brief 把错误帧转成 SocketCAN 的错误帧 / Turn an error event into a SocketCAN error
   *        frame
   * @return true：要上报；false：该通道没开错误报告或不是错误号。
   *         true: report it; false: error reporting is off for the channel, or not an
   *         error ID.
   * @note 数据字节与 Linux can/error.h 对齐。
   *       The data bytes follow Linux can/error.h.
   */
  bool MakeErrorFrame(uint8_t ch, const LibXR::CAN::ClassicPack& pack, QueueItem& item)
  {
    if (berr_enabled_[ch].load(std::memory_order_acquire) == 0U ||
        !LibXR::CAN::IsErrorId(pack.id))
    {
      return false;
    }
    constexpr uint8_t CRTL_RX_WARNING = 0x04U;
    constexpr uint8_t CRTL_TX_WARNING = 0x08U;
    constexpr uint8_t CRTL_RX_PASSIVE = 0x10U;
    constexpr uint8_t CRTL_TX_PASSIVE = 0x20U;
    constexpr uint8_t PROT_FORM = 0x02U;
    constexpr uint8_t PROT_STUFF = 0x04U;
    constexpr uint8_t PROT_BIT0 = 0x08U;
    constexpr uint8_t PROT_BIT1 = 0x10U;
    constexpr uint8_t PROT_TX = 0x80U;
    constexpr uint8_t PROT_LOC_ACK = 0x19U;
    constexpr uint32_t ERR_CNT = 0x00000200U;

    LibXR::CAN::ErrorState es{};
    const bool counts = cans_[ch]->GetErrorState(es) == ErrorCode::OK;
    const uint32_t txerr = counts ? es.tx_error_counter : 0U;
    const uint32_t rxerr = counts ? es.rx_error_counter : 0U;

    uint32_t id = GsUsb::CAN_ERR_FLAG | ERR_CNT;
    uint8_t* d = item.data;
    std::memset(d, 0, WIRE_CLASSIC_DATA_SIZE);
    const auto error = LibXR::CAN::ToErrorID(pack.id);
    switch (error)
    {
      case LibXR::CAN::ErrorID::CAN_ERROR_ID_BUS_OFF:
        id |= GsUsb::CAN_ERR_BUSOFF;
        break;

      case LibXR::CAN::ErrorID::CAN_ERROR_ID_ERROR_WARNING:
      case LibXR::CAN::ErrorID::CAN_ERROR_ID_ERROR_PASSIVE:
      {
        id |= GsUsb::CAN_ERR_CRTL;
        const bool passive = error == LibXR::CAN::ErrorID::CAN_ERROR_ID_ERROR_PASSIVE;
        const uint32_t limit = passive ? 128U : 96U;
        const uint8_t tx_bit = passive ? CRTL_TX_PASSIVE : CRTL_TX_WARNING;
        const uint8_t rx_bit = passive ? CRTL_RX_PASSIVE : CRTL_RX_WARNING;
        uint8_t ctrl = 0U;
        if (counts && txerr >= limit)
        {
          ctrl |= tx_bit;
        }
        if (counts && rxerr >= limit)
        {
          ctrl |= rx_bit;
        }
        // 计数不可用或都没过线时两个方向都报。
        // Without counts, or with neither over the line, report both directions.
        d[1] = ctrl != 0U ? ctrl : static_cast<uint8_t>(tx_bit | rx_bit);
        break;
      }

      case LibXR::CAN::ErrorID::CAN_ERROR_ID_ACK:
        id |= GsUsb::CAN_ERR_ACK | GsUsb::CAN_ERR_PROT;
        d[2] = PROT_TX;
        d[3] = PROT_LOC_ACK;
        break;

      case LibXR::CAN::ErrorID::CAN_ERROR_ID_STUFF:
        id |= GsUsb::CAN_ERR_PROT;
        d[2] = PROT_STUFF | PROT_TX;
        break;

      case LibXR::CAN::ErrorID::CAN_ERROR_ID_FORM:
        id |= GsUsb::CAN_ERR_PROT;
        d[2] = PROT_FORM | PROT_TX;
        break;

      case LibXR::CAN::ErrorID::CAN_ERROR_ID_BIT0:
        id |= GsUsb::CAN_ERR_PROT;
        d[2] = PROT_BIT0 | PROT_TX;
        break;

      case LibXR::CAN::ErrorID::CAN_ERROR_ID_BIT1:
        id |= GsUsb::CAN_ERR_PROT;
        d[2] = PROT_BIT1 | PROT_TX;
        break;

      default:
        id |= GsUsb::CAN_ERR_PROT;
        d[2] = PROT_TX;
        break;
    }
    d[6] = static_cast<uint8_t>(txerr > 255U ? 255U : txerr);
    d[7] = static_cast<uint8_t>(rxerr > 255U ? 255U : rxerr);
    item.hdr = {ECHO_ID_RX, id, GsUsb::CAN_ERR_DLC, ch, 0U, 0U};
    item.timestamp_us = MakeTimestampUs(ch);
    return true;
  }

  /// 补推被压住的回显，重试暂存的主机帧，再发帧 / Push a held-back echo, retry
  /// staged host frames, then send frames
  void Pump(bool in_isr)
  {
    if (held_slot_.Valid())
    {
      const uint8_t hold_ch = held_echo_.hdr.channel;
      if (hold_ch >= CAN_CH_NUM ||
          can_enabled_[hold_ch].load(std::memory_order_relaxed) == 0U)
      {
        const Endpoint::Slot slot = held_slot_;
        held_slot_ = Endpoint::Slot{};
        ReleaseOut(in_isr, slot);
      }
      else if (echo_queue_.Push(held_echo_) == ErrorCode::OK)
      {
        const Endpoint::Slot slot = held_slot_;
        held_slot_ = Endpoint::Slot{};
        ReleaseOut(in_isr, slot);
      }
    }
    ServiceStagedFrames();
    SendFrames(in_isr);
  }

  /// 有空闲 IN 槽和排队的帧时打包提交，直到其一用完 / Pack and submit while an IN slot
  /// is free and a frame is queued
  void SendFrames(bool in_isr)
  {
    while (data_in_ != nullptr)
    {
      Endpoint::Slot slot;
      if (data_in_->AcquireSlot(slot) != ErrorCode::OK)
      {
        // 没有空闲槽：IN 的完成回调会再来。
        // No free slot: the IN completion comes back here.
        return;
      }
      QueueItem item;
      if (!PickFrame(item))
      {
        (void)data_in_->ReleaseSlot(slot);
        return;
      }
      const size_t size = Pack(item, slot.data);
      // 主机按设备的能力申请接收缓冲：带时间戳的最大帧（Linux gs_usb 的 hf_size_rx）。
      // 比它短的帧靠短包结束传输，长度正好是包长整数倍时要结尾零长度包；等于缓冲大小的
      // 帧已经装满缓冲，不要。包长为 32、64、512 时帧长 20、24、76 都不是整数倍，线上
      // 没有零长度包。
      // The host sizes its receive buffer from the device's capabilities: the largest
      // frame with a timestamp (hf_size_rx of the Linux gs_usb driver). A shorter frame
      // ends its transfer by a short packet and needs the closing zero-length packet when
      // it is a whole number of packets; a frame of the buffer's size fills the buffer
      // and needs none. With packets of 32, 64 or 512 bytes the frame lengths 20, 24 and
      // 76 are no multiples, so no zero-length packet goes onto the wire.
      const bool zlp = size < HostBufferSize(fd_supported_);
      const ErrorCode result = data_in_->Submit(slot, size, zlp, in_isr);
      // ABORTED：端点在借出之后被关闭，槽已空闲，这一帧随绑定丢弃。
      // ABORTED: the endpoint closed after the lend; the slot is free and the frame is
      // dropped with the binding.
      ASSERT(result == ErrorCode::OK || result == ErrorCode::ABORTED);
      if (result != ErrorCode::OK)
      {
        return;
      }
    }
  }

  /// 选下一帧：回显与接收帧都有待发时轮流，只有一类时直接发；接收帧通道间轮流，通道内
  /// 先错误帧后接收帧 / Pick the next frame: echoes and received frames alternate while
  /// both are pending and go out directly when only one kind is; received frames go round
  /// robin across channels, error frames before received ones within one
  bool PickFrame(QueueItem& item)
  {
    if (rx_turn_)
    {
      if (PickReceived(item))
      {
        rx_turn_ = false;
        ApplyOverflowFlag(item);
        return true;
      }
      if (PickEcho(item))
      {
        ApplyOverflowFlag(item);
        return true;
      }
      return false;
    }
    if (PickEcho(item))
    {
      rx_turn_ = true;
      ApplyOverflowFlag(item);
      return true;
    }
    if (PickReceived(item))
    {
      ApplyOverflowFlag(item);
      return true;
    }
    return false;
  }

  /// 取下一个回显；未启动通道的回显丢弃 / Take the next echo; echoes of unstarted
  /// channels are discarded
  bool PickEcho(QueueItem& item)
  {
    while (echo_queue_.Pop(item) == ErrorCode::OK)
    {
      if (item.hdr.channel < CAN_CH_NUM &&
          can_enabled_[item.hdr.channel].load(std::memory_order_relaxed) != 0U)
      {
        return true;
      }
    }
    return false;
  }

  /// 通道间轮流取一帧接收帧或错误帧；跳过未启动通道 / Take one received or error frame
  /// round-robin across started channels
  bool PickReceived(QueueItem& item)
  {
    for (uint8_t k = 0U; k < CAN_CH_NUM; ++k)
    {
      const auto ch = static_cast<uint8_t>((rr_ + k) % CAN_CH_NUM);
      if (can_enabled_[ch].load(std::memory_order_relaxed) == 0U)
      {
        continue;
      }
      if (err_queues_[ch]->Pop(item) == ErrorCode::OK ||
          rx_queues_[ch]->Pop(item) == ErrorCode::OK)
      {
        rr_ = static_cast<uint8_t>((ch + 1U) % CAN_CH_NUM);
        return true;
      }
    }
    return false;
  }

  /**
   * @brief 该通道丢过帧时给这一帧置溢出标志 / Set the overflow flag on this frame when
   *        its channel dropped frames
   * @note 主机对任何一帧（接收、错误或回显）都照常处理，再按帧的通道记一次接收溢出，
   *       所以标志带在该通道的下一帧上，不另造帧。
   *       The host handles any frame (received, error or echo) as usual and then counts
   *       one receive overflow for the frame's channel, so the flag rides on the
   *       channel's next frame and no extra frame is made.
   */
  void ApplyOverflowFlag(QueueItem& item)
  {
    const uint8_t ch = item.hdr.channel;
    const uint32_t drops = rx_drop_[ch].load(std::memory_order_relaxed);
    if (drops != rx_drop_reported_[ch])
    {
      item.hdr.flags |= GsUsb::CAN_FLAG_OVERFLOW;
      rx_drop_reported_[ch] = drops;
    }
  }

  /// 把一帧写进槽，返回字节数 / Write one frame into the slot, return its length
  size_t Pack(const QueueItem& item, RawData out) const
  {
    const bool fd = (item.hdr.flags & GsUsb::CAN_FLAG_FD) != 0U;
    const bool ts =
        timestamps_enabled_[item.hdr.channel].load(std::memory_order_relaxed) != 0U;
    const size_t payload = fd ? WIRE_FD_DATA_SIZE : WIRE_CLASSIC_DATA_SIZE;
    const size_t total = WIRE_HDR_SIZE + payload + (ts ? WIRE_TS_SIZE : 0U);
    // 绑定时已检查一个槽装得下最长的一帧。
    // The bind checked that one slot holds the longest frame.
    ASSERT(total <= out.size_);
    auto* bytes = static_cast<uint8_t*>(out.addr_);
    std::memcpy(bytes, &item.hdr, WIRE_HDR_SIZE);
    LibXR::Memory::FastCopy(bytes + WIRE_HDR_SIZE, item.data, payload);
    if (ts)
    {
      std::memcpy(bytes + WIRE_HDR_SIZE + payload, &item.timestamp_us, WIRE_TS_SIZE);
    }
    return total;
  }

  /// 丢弃全部队列里的帧，丢帧计数视为已报告；仅 owner（消费者一侧）/ Drop every
  /// queued frame and treat the drop counts as reported; owner only (consumer side)
  void DiscardQueuedFrames()
  {
    while (echo_queue_.Pop() == ErrorCode::OK)
    {
    }
    for (uint8_t ch = 0U; ch < CAN_CH_NUM; ++ch)
    {
      while (rx_queues_[ch]->Pop() == ErrorCode::OK)
      {
      }
      while (err_queues_[ch]->Pop() == ErrorCode::OK)
      {
      }
      rx_drop_reported_[ch] = rx_drop_[ch].load(std::memory_order_relaxed);
      DropStagedFrames(ch);
    }
    rr_ = 0U;
    rx_turn_ = false;
  }

  /// 丢弃单通道未提交到端点的接收帧、错误帧、回显及被压住的回显 / Drop one
  /// channel's queued receive frames, error frames, echoes and held-back echo
  void DropChannelFrames(bool in_isr, uint8_t ch)
  {
    while (rx_queues_[ch]->Pop() == ErrorCode::OK)
    {
    }
    while (err_queues_[ch]->Pop() == ErrorCode::OK)
    {
    }
    rx_drop_reported_[ch] = rx_drop_[ch].load(std::memory_order_relaxed);
    QueueItem item;
    for (size_t n = echo_queue_.Size(); n != 0U; --n)
    {
      if (echo_queue_.Pop(item) != ErrorCode::OK)
      {
        break;
      }
      if (item.hdr.channel != ch)
      {
        (void)echo_queue_.Push(item);
      }
    }
    if (held_slot_.Valid() && held_echo_.hdr.channel == ch)
    {
      const Endpoint::Slot slot = held_slot_;
      held_slot_ = Endpoint::Slot{};
      if (data_out_ != nullptr)
      {
        ReleaseOut(in_isr, slot);
      }
    }
  }

  /// 有没有仍在运行的通道 / Whether any channel is started
  bool AnyChannelStarted() const
  {
    for (uint8_t ch = 0U; ch < CAN_CH_NUM; ++ch)
    {
      if (can_enabled_[ch].load(std::memory_order_relaxed) != 0U)
      {
        return true;
      }
    }
    return false;
  }

  /// 最后一个通道复位：清空全通道残留并 Flush 共享 IN 端点 / Last channel reset: clear
  /// remaining queues and flush the shared IN endpoint
  void FlushInEndpoint(bool in_isr)
  {
    for (uint8_t ch = 0U; ch < CAN_CH_NUM; ++ch)
    {
      DropChannelFrames(in_isr, ch);
    }
    rr_ = 0U;
    rx_turn_ = false;
    if (data_in_ != nullptr)
    {
      data_in_->Flush(in_isr);
    }
  }

  /// 任一通道暂存队列非空 / Whether any channel staging queue has frames
  bool AnyStaged() const
  {
    for (uint8_t ch = 0U; ch < CAN_CH_NUM; ++ch)
    {
      if (stage_queues_[ch]->Size() != 0U)
      {
        return true;
      }
    }
    return false;
  }

  /// 丢弃单通道暂存的主机帧 / Drop host frames staged for one channel
  void DropStagedFrames(uint8_t ch)
  {
    while (stage_queues_[ch]->Pop() == ErrorCode::OK)
    {
    }
    stage_accepted_[ch] = false;
    stage_pending_ = AnyStaged();
  }

  /// 把一帧提交到 CAN 驱动 / Submit one host frame to the CAN driver
  ErrorCode SubmitToCan(uint8_t ch, const WireHeader& hdr, const uint8_t* payload)
  {
    const bool fd = (hdr.flags & GsUsb::CAN_FLAG_FD) != 0U;
    const bool eff = (hdr.can_id & GsUsb::CAN_EFF_FLAG) != 0U;
    const bool rtr = (hdr.can_id & GsUsb::CAN_RTR_FLAG) != 0U;
    const uint32_t id = hdr.can_id & (eff ? GsUsb::CAN_EFF_MASK : GsUsb::CAN_SFF_MASK);
    LibXR::CAN::Type type = eff ? LibXR::CAN::Type::EXTENDED : LibXR::CAN::Type::STANDARD;
    if (rtr)
    {
      type = eff ? LibXR::CAN::Type::REMOTE_EXTENDED : LibXR::CAN::Type::REMOTE_STANDARD;
    }
    if (fd)
    {
      LibXR::FDCAN::FDPack pack;
      pack.id = id;
      pack.type = type;
      pack.len = DlcToLen(hdr.can_dlc);
      LibXR::Memory::FastCopy(pack.data, payload, pack.len);
      return fdcans_[ch]->AddMessage(pack);
    }
    LibXR::CAN::ClassicPack pack;
    pack.id = id;
    pack.type = type;
    pack.dlc = hdr.can_dlc <= 8U ? hdr.can_dlc : 8U;
    std::memcpy(pack.data, payload, pack.dlc);
    return cans_[ch]->AddMessage(pack);
  }

  /// CAN 驱动队列暂满或外设忙，需暂存等待重试 / CAN driver queue full or peripheral
  /// busy: stage and retry later
  static bool IsCanTxBusy(ErrorCode result)
  {
    return result == ErrorCode::FULL || result == ErrorCode::BUSY;
  }

  /// 按通道顺序重试暂存主机帧，接受后才入 echo 队列 / Retry staged host frames in
  /// order per channel, queueing the echo only once accepted
  void ServiceStagedFrames()
  {
    if (!stage_pending_)
    {
      return;
    }
    for (uint8_t ch = 0U; ch < CAN_CH_NUM; ++ch)
    {
      QueueItem item;
      while (stage_queues_[ch]->PeekBatch(&item, 1U) == ErrorCode::OK)
      {
        const bool fd = (item.hdr.flags & GsUsb::CAN_FLAG_FD) != 0U;
        if (!stage_accepted_[ch])
        {
          if (can_enabled_[ch].load(std::memory_order_relaxed) == 0U ||
              (fd && fd_enabled_[ch].load(std::memory_order_relaxed) == 0U))
          {
            DropStagedFrames(ch);
            break;
          }
          // FULL/BUSY 保留队头等待下一次排空；OK
          // 或其他错误消费该帧并按原语义回显以归还主机 tx_context。 FULL/BUSY keeps the
          // head for retry; OK or other driver errors consume it and emit the echo as
          // before so the host reclaims its tx_context.
          if (IsCanTxBusy(SubmitToCan(ch, item.hdr, item.data)))
          {
            break;
          }
          stage_accepted_[ch] = true;
          stage_ts_[ch] = MakeTimestampUs(ch);
        }
        if (item.hdr.echo_id != ECHO_ID_RX)
        {
          item.timestamp_us = stage_ts_[ch];
          if (echo_queue_.Push(item) != ErrorCode::OK)
          {
            break;
          }
        }
        stage_accepted_[ch] = false;
        (void)stage_queues_[ch]->Pop();
      }
    }
    stage_pending_ = AnyStaged();
  }

  /// @}
  /// @name 来自主机 / From the host
  /// @{

  /// IN 完成回调入口：未绑定时什么也不做 / IN completion entry: nothing while unbound
  static void OnDataInCompleteStatic(bool in_isr, GsUsbClass* self,
                                     const Endpoint::Completion& completion)
  {
    UNUSED(completion);
    if (self->data_in_ != nullptr)
    {
      // 一帧结束（主机取走或中止）：槽已空闲，继续发。
      // A frame ended (taken by the host or aborted): the slot is free, keep sending.
      self->Pump(in_isr);
    }
  }

  /// OUT 完成回调入口：未绑定时什么也不做 / OUT completion entry: nothing while unbound
  static void OnDataOutCompleteStatic(bool in_isr, GsUsbClass* self,
                                      const Endpoint::Completion& completion)
  {
    if (self->data_out_ == nullptr)
    {
      // 未绑定时 OUT 只会以 ABORTED 结束，不借出槽（见 Endpoint::Close）。
      // While unbound an OUT only ends with ABORTED and lends no slot (see
      // Endpoint::Close).
      ASSERT(!completion.slot.Valid());
      return;
    }
    self->OnDataOut(in_isr, completion);
  }

  /// 主机发来一帧（或接收中止）/ The host sent one frame (or the receive was aborted)
  void OnDataOut(bool in_isr, const Endpoint::Completion& completion)
  {
    if (completion.result != ErrorCode::OK)
    {
      // 失败或被 STALL 中止：槽已空闲。STALLED 时挂的接收排队到 ClearStall。
      // Failed, or aborted by a halt: the slot is free. A receive armed while STALLED
      // waits for ClearStall.
      ArmOut(in_isr);
      return;
    }
    QueueItem echo;
    if (ForwardHostFrame(completion.slot.data, echo) &&
        echo_queue_.Push(echo) != ErrorCode::OK)
    {
      // 回显队列满：留住槽，不再挂接收，主机被 NAK；Pump 腾出空间后补推。
      // Echo queue full: keep the slot and arm nothing, so the host is NAKed; Pump
      // pushes the echo once there is room.
      held_slot_ = completion.slot;
      held_echo_ = echo;
    }
    else
    {
      ReleaseOut(in_isr, completion.slot);
    }
    Pump(in_isr);
  }

  /**
   * @brief 把主机的一帧交给 CAN（队满时暂存），并做出它的回显 / Hand one host frame to
   *        CAN (staging if full) and make its echo
   * @param frame 收到的字节 / The bytes received
   * @param echo 输出：回显 / Output: the echo
   * @return true：已立即接受或暂存满丢弃且要回显；false：已入暂存队列、丢弃或不要回显。
   *         true: accepted now or dropped on full stage and to be echoed; false:
   *         staged, dropped or no echo wanted.
   */
  bool ForwardHostFrame(ConstRawData frame, QueueItem& echo)
  {
    if (!IsHostFormatOK() || frame.size_ < WIRE_CLASSIC_SIZE)
    {
      return false;
    }
    WireHeader hdr;
    std::memcpy(&hdr, frame.addr_, WIRE_HDR_SIZE);
    const uint8_t* payload = static_cast<const uint8_t*>(frame.addr_) + WIRE_HDR_SIZE;
    const uint8_t ch = hdr.channel;
    if (ch >= CAN_CH_NUM)
    {
      return false;
    }
    const bool fd = (hdr.flags & GsUsb::CAN_FLAG_FD) != 0U;
    if (fd)
    {
      if (fd_enabled_[ch].load(std::memory_order_relaxed) == 0U ||
          frame.size_ < WIRE_FD_SIZE)
      {
        return false;
      }
    }
    else if (can_enabled_[ch].load(std::memory_order_relaxed) == 0U)
    {
      return false;
    }

    const size_t data_size = fd ? WIRE_FD_DATA_SIZE : WIRE_CLASSIC_DATA_SIZE;
    echo.hdr = hdr;
    LibXR::Memory::FastCopy(echo.data, payload, data_size);

    // 若本通道有暂存帧，先尝试排空两个 SOF 之间硬件已消化的空位，再决定直接发送或暂存；
    // 若暂存仍满（如超过每通道 tx_stage_size 上限），不阻塞共享 OUT 端点，丢弃并回发 echo
    // 以便主机回收 tx_context。
    // If this channel has staged frames, drain slots freed between SOFs first; only
    // stage if traffic remains backed up. On full stage, drop and echo immediately so
    // the shared OUT endpoint is never stalled and the host reclaims its tx_context.
    if (stage_queues_[ch]->Size() != 0U)
    {
      ServiceStagedFrames();
    }
    if (stage_queues_[ch]->Size() != 0U || IsCanTxBusy(SubmitToCan(ch, hdr, payload)))
    {
      echo.timestamp_us = 0U;
      if (stage_queues_[ch]->Push(echo) == ErrorCode::OK)
      {
        stage_pending_ = true;
        return false;
      }
    }

    if (hdr.echo_id == ECHO_ID_RX)
    {
      return false;
    }
    echo.timestamp_us = MakeTimestampUs(ch);
    return true;
  }

  /// 交回收到数据的槽，再挂接收 / Hand back the received slot, then arm the receive
  void ReleaseOut(bool in_isr, const Endpoint::Slot& slot)
  {
    const ErrorCode result = data_out_->ReleaseSlot(slot);
    ASSERT(result == ErrorCode::OK);
    UNUSED(result);
    ArmOut(in_isr);
  }

  /**
   * @brief 挂唯一的一个 OUT 接收，长度为 MaxTransferSize() / Arm the single OUT
   *        receive, MaxTransferSize() long
   * @note 多个包的请求：主机一帧以短包结束，请求随之结束，所以长度只是上限。调用点
   *       （绑定、OUT 完成、交回留住的槽）互斥，任何时刻最多一个接收在途或一个槽留住。
   *       A multi-packet request: the host's frame ends with a short packet and so does
   *       the request, so the length is only a bound. The call sites (bind, OUT
   *       completion, hand-back of the kept slot) exclude each other, so at most one
   *       receive is armed or one slot kept at any time.
   */
  void ArmOut(bool in_isr)
  {
    const ErrorCode result = data_out_->Receive(data_out_->MaxTransferSize(), in_isr);
    ASSERT(result == ErrorCode::OK);
    UNUSED(result);
  }

  /// @}

  /// 时间戳开着时取当前微秒，否则 0；任何上下文 / Current microseconds when timestamps
  /// are on, else 0; any context
  uint32_t MakeTimestampUs(uint8_t ch) const
  {
    if (timestamps_enabled_[ch].load(std::memory_order_acquire) == 0U)
    {
      return 0U;
    }
    return static_cast<uint32_t>(LibXR::Timebase::GetMicroseconds());
  }

  /// FD 的 DLC 转长度 / FD DLC to length
  static uint8_t DlcToLen(uint8_t dlc)
  {
    static constexpr uint8_t TABLE[16] = {0U, 1U,  2U,  3U,  4U,  5U,  6U,  7U,
                                          8U, 12U, 16U, 20U, 24U, 32U, 48U, 64U};
    return dlc < 16U ? TABLE[dlc] : 64U;
  }

  /// FD 的长度转 DLC（向上取到能装下的长度）/ FD length to DLC (rounded up to a length
  /// that fits)
  static uint8_t LenToDlc(uint8_t len)
  {
    for (uint8_t dlc = 0U; dlc < 15U; ++dlc)
    {
      if (DlcToLen(dlc) >= len)
      {
        return dlc;
      }
    }
    return 15U;
  }

  /// @name 构造时固定 / Fixed at construction
  /// @{
  LibXR::CAN* cans_[CanChNum] = {};      ///< 各通道的 CAN / CAN of each channel
  LibXR::FDCAN* fdcans_[CanChNum] = {};  ///< FD 时同 cans_ / Same as cans_ for FD
  bool fd_supported_ = false;            ///< 是否 FD / Whether FD
  Endpoint::EPNumber data_in_num_;       ///< 批量 IN 端点号 / Bulk IN number
  Endpoint::EPNumber data_out_num_;      ///< 批量 OUT 端点号 / Bulk OUT number
  const char* interface_string_;         ///< 接口字符串 / Interface string
  LibXR::GPIO* identify_gpio_;           ///< IDENTIFY GPIO，可为空 / May be null
  /// 终端电阻 GPIO，可为空 / Termination GPIOs, may be null
  LibXR::GPIO* termination_gpios_[CanChNum] = {};
  /// USER_ID 的数据库键；有数据库时构造时建 / Database key of USER_ID; made at
  /// construction when there is a database
  LibXR::Database::Key<uint32_t>* user_id_key_ = nullptr;
  ChannelContext contexts_[CanChNum] = {};  ///< CAN 回调参数 / CAN callback arguments
  GsUsb::DeviceConfig dev_cfg_{};           ///< DEVICE_CONFIG 应答 / Reply
  GsUsb::DeviceBTConst bt_const_{};         ///< BT_CONST 应答 / Reply
  GsUsb::DeviceBTConstExtended bt_const_ext_{};  ///< BT_CONST_EXT 应答 / Reply
  /// 批量 IN 完成回调 / Bulk IN completion callback
  Callback<const Endpoint::Completion&> data_in_cb_ =
      Callback<const Endpoint::Completion&>::Create(OnDataInCompleteStatic, this);
  /// 批量 OUT 完成回调 / Bulk OUT completion callback
  Callback<const Endpoint::Completion&> data_out_cb_ =
      Callback<const Endpoint::Completion&>::Create(OnDataOutCompleteStatic, this);
  /// @}

  /// @name 队列：单生产者单消费者，消费者是 owner / Queues: single producer, single
  ///       consumer, the consumer is the owner
  /// @{
  /// 回显队列，生产者是 OUT 完成（owner）/ Echo queue, filled by the OUT completion
  /// (owner)
  LibXR::SPSCQueue<QueueItem> echo_queue_;
  /// 每通道接收队列，生产者是 CAN 接收回调 / Per-channel receive queue, filled by the
  /// CAN receive callback
  LibXR::SPSCQueue<QueueItem>* rx_queues_[CanChNum] = {};
  /// 每通道错误帧队列，生产者是 CAN 错误回调 / Per-channel error-frame queue, filled by
  /// the CAN error callback
  LibXR::SPSCQueue<QueueItem>* err_queues_[CanChNum] = {};
  /// 每通道发送暂存队列；只在 owner 上下文中读写 / Per-channel TX staging queue; owner
  /// context only
  LibXR::SPSCQueue<QueueItem>* stage_queues_[CanChNum] = {};
  /// @}

  /// @name 仅 owner / Owner only
  /// @{
  Endpoint* data_in_ = nullptr;   ///< 批量 IN，绑定时非空 / Bulk IN, non-null if bound
  Endpoint* data_out_ = nullptr;  ///< 批量 OUT，绑定时非空 / Bulk OUT, non-null if bound
  /// WriteDescriptors 写出的块；BindEndpoints 比较包长时读 / Block written by
  /// WriteDescriptors; read by BindEndpoints to compare the sizes
  DescriptorBlock desc_block_{};
  /// IN 应答；OnControlRequest 写，核心发送时读 / IN reply; written by
  /// OnControlRequest, read by the core while sending
  ReplyBuffer reply_{};
  /// 各通道经典配置；控制请求写 / Classic configuration per channel; written by
  /// control requests
  LibXR::CAN::Configuration config_[CanChNum] = {};
  /// 各通道 FD 配置；控制请求写 / FD configuration per channel; written by control
  /// requests
  LibXR::FDCAN::Configuration fd_config_[CanChNum] = {};
  /// 终端电阻状态；SET_TERMINATION 写，GET_TERMINATION 读，解绑不变 / Termination
  /// state; written by SET_TERMINATION, read by GET_TERMINATION, kept over an unbind
  GsUsb::TerminationState term_state_[CanChNum] = {};
  uint8_t rr_ = 0U;  ///< 下一个先看的通道 / Channel looked at first next time
  bool rx_turn_ =
      false;  ///< 下一帧先尝试接收队列 / Next frame tries receive queues first
  /// 暂存队列队头已被驱动接受、只等回显入队 / Staging queue head already accepted by
  /// CAN driver, awaiting echo queue space
  bool stage_accepted_[CanChNum] = {};
  uint32_t stage_ts_[CanChNum] =
      {};                       ///< 队头被接受时的时间戳 / Accept timestamp per channel
  bool stage_pending_ = false;  ///< 至少一个暂存队列非空 / Some staging queue non-empty
  /// 回显队列满时留住的 OUT 槽；Valid() 表示有 / OUT slot kept while the echo queue is
  /// full; Valid() when there is one
  Endpoint::Slot held_slot_{};
  /// held_slot_ 那一帧的回显，只在 held_slot_ 有效时有意义 / Echo of the frame in
  /// held_slot_, meaningful only while held_slot_ is valid
  QueueItem held_echo_{};
  /// 已上报的丢帧数 / Drop counts already reported
  uint32_t rx_drop_reported_[CanChNum] = {};
  /// @}

  /// @name 任何上下文 / Any context
  /// @{
  /// 字节序已协商；owner 写，任何上下文读 / Byte order negotiated; written by the
  /// owner, read anywhere
  std::atomic<uint32_t> host_format_ok_{0U};
  /// 通道已启动（经典帧）；owner 写，CAN 回调读 / Channel started (classic frames);
  /// written by the owner, read by the CAN callbacks
  std::atomic<uint32_t> can_enabled_[CanChNum] = {};
  /// 通道以 FD 启动；owner 写，CAN 回调读 / Channel started with FD; written by the
  /// owner, read by the CAN callbacks
  std::atomic<uint32_t> fd_enabled_[CanChNum] = {};
  /// 错误报告打开；owner 写，CAN 回调读 / Error reporting on; written by the owner,
  /// read by the CAN callbacks
  std::atomic<uint32_t> berr_enabled_[CanChNum] = {};
  /// 时间戳打开；owner 写，CAN 回调和 owner 读 / Timestamps on; written by the owner,
  /// read by the CAN callbacks and the owner
  std::atomic<uint32_t> timestamps_enabled_[CanChNum] = {};
  /// 丢帧计数；本通道的 CAN 回调加，owner 读 / Drop count; added by the channel's CAN
  /// callback, read by the owner
  std::atomic<uint32_t> rx_drop_[CanChNum] = {};
  /// USER_ID 的内存值；owner 写，定时任务读 / USER_ID in memory; written by the owner,
  /// read by the timer task
  std::atomic<uint32_t> user_id_{0U};
  /// USER_ID 待写；owner 置位，定时任务清除 / USER_ID write pending; set by the owner,
  /// cleared by the timer task
  std::atomic<uint32_t> user_id_write_pending_{0U};
  /// @}
};

}  // namespace LibXR::USB
