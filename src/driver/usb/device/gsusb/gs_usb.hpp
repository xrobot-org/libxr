#pragma once
#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <initializer_list>
#include <new>
#include <type_traits>

#include "can.hpp"
#include "database.hpp"
#include "dev_core.hpp"
#include "gpio.hpp"
#include "gs_usb_protocol.hpp"
#include "libxr_def.hpp"
#include "libxr_mem.hpp"
#include "structure/queue/spsc_queue.hpp"
#include "timebase.hpp"
#include "timer.hpp"
#include "usb/core/desc_cfg.hpp"

namespace LibXR::USB
{

/**
 * @class GsUsbClass
 * @brief GsUsb 设备类，实现 Linux gs_usb 协议（经典 CAN + CAN FD） /
 *        GsUsb device class implementing Linux gs_usb protocol (Classic CAN + CAN FD)
 * @tparam CanChNum 编译期固定的 CAN 通道数 / Compile-time fixed CAN channel count
 *
 * 接收路径：每个 CAN 源一个 SPSC 队列（rx_queue_size 为每通道容量），echo 单独一个
 * SPSC 队列（生产者是 USB OUT 处理）。CAN 回调与 OUT 处理只入队并请求类服务，
 * 服务里取槽、打包、提交。echo 与接收帧在两者都有待发时轮流发送，只有一类时直接发送；
 * 接收帧在通道间轮流；RX 队列满时丢帧并按通道计数，溢出标志随该通道发往主机的下一帧
 * （RX、错误帧或 echo）上报。
 *
 * Receive path: one SPSC queue per CAN source (rx_queue_size is the per-channel
 * capacity) plus one echo SPSC queue whose producer is the USB OUT handling. CAN
 * callbacks and the OUT handler only enqueue and request the class service; the
 * service acquires slots, packs and submits. Echo frames and received frames alternate
 * while both are pending and go out directly when only one kind is; received frames
 * are drained round-robin across the channels; a full RX queue drops the frame and
 * counts it per channel, and the overflow flag rides on that channel's next frame to
 * the host (RX, error or echo).
 *
 * 发送路径：echo 的含义是“设备的 CAN 驱动已接受这一帧”。OUT 帧交给 CAN 驱动；驱动的
 * 发送队列满（FULL）时，帧进入该通道自己的暂存队列，不回 echo，OUT 租约立即归还，
 * 另一个通道不受影响。暂存队列非空期间，该通道的新帧排在队尾，保持顺序。服务和 SOF
 * 节拍（只在有暂存帧时请求）按顺序重试，驱动接受一帧才回它的 echo。每通道的流控靠
 * 主机每通道的在途帧上限（Linux gs_usb 驱动为 10 帧，等于默认的暂存容量）：主机收到
 * echo 才继续发，所以暂存队列不会溢出。超过容量时（主机不遵守该上限）这一帧被丢弃，
 * 仍然回 echo。通道复位、重新启动或端点解绑时，该通道的暂存帧被丢弃：还没交给 CAN
 * 驱动的帧不会发送，也不回 echo；队头那一帧如果驱动已经接受（只差 echo 队列腾位），
 * 已经交给了驱动，仍可能发到总线上，只是它的 echo 不再回。
 *
 * Transmit path: an echo means "the device's CAN driver accepted this frame". An OUT
 * frame goes to the CAN driver; when the driver's transmit queue is full (FULL) the
 * frame goes to that channel's own staging queue, gets no echo yet, and the OUT lease
 * goes back at once, so the other channel is not held up. While a channel's staging
 * queue is not empty, its new frames go to the end of it to keep the order. The
 * service and the SOF tick (requested only while something is staged) retry in order,
 * and a frame's echo is sent when the driver accepts it. Per-channel flow control rests
 * on the host's per-channel in-flight limit (10 frames in the Linux gs_usb driver,
 * equal to the default staging capacity): the host sends on only after an echo, so the
 * staging queue does not overflow. Beyond the capacity (a host that ignores that limit)
 * the frame is dropped and still echoed. A channel reset, a restart of the channel or
 * an endpoint unbind drops the channel's staged frames: a frame not yet handed to the
 * CAN driver is never sent and gets no echo; the head frame, if the driver already
 * accepted it (only room in the echo queue is missing), has been handed over and may
 * still go out on the bus, only its echo is not sent.
 */
template <std::size_t CanChNum>
class GsUsbClass : public DeviceClass
{
 public:
  static_assert(CanChNum > 0 && CanChNum <= 255, "CanChNum must be in (0, 255]");
  /// CAN 通道数 / Number of CAN channels
  static constexpr uint8_t CAN_CH_NUM = static_cast<uint8_t>(CanChNum);
  /// GS USB 功能的默认接口字符串。
  /// Default interface string for the GS USB function.
  static constexpr const char* DEFAULT_INTERFACE_STRING = "XRUSB GS USB";

  /// SET_USER_ID 落库周期（暂定 100 ms）。
  /// USER_ID write-back period (tentatively 100 ms).
  static constexpr uint32_t USER_ID_WRITE_PERIOD_MS = 100;

  /// 每通道错误帧队列容量。
  /// Per-channel error-frame queue capacity.
  static constexpr std::size_t ERR_QUEUE_SIZE = 8;

  /// 每通道暂存队列的默认容量：Linux gs_usb 驱动每通道最多 10 帧在途（GS_MAX_TX_URBS）。
  /// Default per-channel staging capacity: the Linux gs_usb driver keeps at most 10
  /// frames in flight per channel (GS_MAX_TX_URBS).
  static constexpr std::size_t DEFAULT_TX_STAGE_SIZE = 10;

 private:
  /// 暂存容量至少为 1：0 在构造时被拒绝（release 构建里同样检查）。
  /// The staging capacity is at least 1: 0 is rejected at construction (checked in
  /// release builds too).
  static std::size_t CheckedStageSize(std::size_t tx_stage_size)
  {
    REQUIRE(tx_stage_size >= 1U);
    return tx_stage_size;
  }

  // ===== Linux gs_usb 线缆格式（固定 12 字节头） / Linux gs_usb wire format
  // (fixed 12-byte header) =====
  LIBXR_PACKED_BEGIN

  /**
   * @brief gs_usb 线缆格式头（12 字节） / gs_usb wire-format header (12 bytes)
   */
  struct WireHeader
  {
    uint32_t echo_id;  ///< 回显 ID / Echo ID
    uint32_t can_id;   ///< CAN ID（含标志位） / CAN ID (with flags)
    uint8_t can_dlc;   ///< DLC
    uint8_t channel;   ///< 通道号 / Channel index
    uint8_t flags;     ///< 帧标志（GS_CAN_FLAG_*） / Frame flags (GS_CAN_FLAG_*)
    uint8_t reserved;  ///< 保留 / Reserved
  };
  LIBXR_PACKED_END

  static constexpr uint32_t ECHO_ID_RX =
      0xFFFFFFFFu;  ///< RX 帧 echo_id 固定值 / Fixed echo_id for RX frames

  static constexpr std::size_t WIRE_HDR_SIZE =
      sizeof(WireHeader);  ///< Header 长度 / Header size
  static constexpr std::size_t WIRE_CLASSIC_DATA_SIZE =
      8;  ///< classic 数据长度 / Classic data size
  static constexpr std::size_t WIRE_FD_DATA_SIZE = 64;  ///< FD 数据长度 / FD data size
  static constexpr std::size_t WIRE_TS_SIZE =
      4;  ///< 时间戳字段长度 / Timestamp field size

  static constexpr std::size_t WIRE_CLASSIC_SIZE =
      WIRE_HDR_SIZE + 8;  ///< classic 总长度 / Classic total size
  static constexpr std::size_t WIRE_CLASSIC_TS_SIZE =
      WIRE_HDR_SIZE + 8 + 4;  ///< classic+ts 总长度 / Classic+ts total size
  static constexpr std::size_t WIRE_FD_SIZE =
      WIRE_HDR_SIZE + 64;  ///< FD 总长度 / FD total size
  static constexpr std::size_t WIRE_FD_TS_SIZE =
      WIRE_HDR_SIZE + 64 + 4;  ///< FD+ts 总长度 / FD+ts total size
  static constexpr std::size_t WIRE_MAX_SIZE =
      WIRE_FD_TS_SIZE;  ///< 最大线缆帧长度 / Maximum wire frame size

 public:
  /**
   * @brief 构造：经典 CAN / Construct: Classic CAN
   * @param cans 经典 CAN 指针列表，数量必须等于 CAN_CH_NUM / Classic CAN pointers, count
   * must equal CAN_CH_NUM
   * @param data_in_ep_num Bulk IN 端点号 / Bulk IN endpoint number
   * @param data_out_ep_num Bulk OUT 端点号 / Bulk OUT endpoint number
   * @param rx_queue_size 每通道接收队列容量 / Per-channel RX queue capacity
   * @param echo_queue_size 回显队列容量 / Echo queue capacity
   * @param identify_gpio Identify GPIO（可选） / Identify GPIO (optional)
   * @param termination_gpios 每通道终端电阻 GPIO（可选） / Per-channel termination GPIOs
   * (optional)
   * @param database 数据库存储（可选，用于 USER_ID） / Database storage (optional, used
   * by USER_ID)
   * @param interface_string 接口字符串 / Interface string
   * @param tx_stage_size 每通道暂存队列容量（至少 1），默认 10（Linux gs_usb 驱动每通道
   * 在途帧上限）/ Per-channel staging capacity (at least 1) for host frames while the CAN
   * transmit queue is full; default 10 (the per-channel in-flight limit of the Linux
   * gs_usb driver)
   */
  GsUsbClass(Endpoint::EPNumber data_in_ep_num, Endpoint::EPNumber data_out_ep_num,
             std::initializer_list<LibXR::CAN*> cans, size_t rx_queue_size = 32,
             size_t echo_queue_size = 32, LibXR::GPIO* identify_gpio = nullptr,
             std::initializer_list<LibXR::GPIO*> termination_gpios = {},
             LibXR::Database* database = nullptr,
             const char* interface_string = DEFAULT_INTERFACE_STRING,
             size_t tx_stage_size = DEFAULT_TX_STAGE_SIZE)
      : data_in_ep_num_(data_in_ep_num),
        data_out_ep_num_(data_out_ep_num),
        interface_string_(interface_string),
        identify_gpio_(identify_gpio),
        database_(database),
        echo_queue_(echo_queue_size),
        rx_queues_(rx_queue_size),
        err_queues_(ERR_QUEUE_SIZE),
        stage_queues_(CheckedStageSize(tx_stage_size))
  {
    ASSERT(cans.size() == CAN_CH_NUM);
    std::size_t i = 0;
    for (auto* p : cans)
    {
      cans_[i++] = p;
    }

    if (termination_gpios.size() == 0)
    {
      termination_gpios_.fill(nullptr);
    }
    else
    {
      ASSERT(termination_gpios.size() == CAN_CH_NUM);
      i = 0;
      for (auto* g : termination_gpios)
      {
        termination_gpios_[i++] = g;
      }
    }

    InitDeviceConfigClassic();
    RegisterCanCallbacks();
    InitUserIdStorage();
  }

  /**
   * @brief 构造：FDCAN / Construct: FDCAN
   * @param fd_cans FDCAN 指针列表，数量必须等于 CAN_CH_NUM / FDCAN pointers, count must
   * equal CAN_CH_NUM
   * @param data_in_ep_num Bulk IN 端点号（必填） / Bulk IN endpoint number (required)
   * @param data_out_ep_num Bulk OUT 端点号（必填） / Bulk OUT endpoint number (required)
   * @param rx_queue_size 每通道接收队列容量 / Per-channel RX queue capacity
   * @param echo_queue_size 回显队列容量 / Echo queue capacity
   * @param identify_gpio Identify GPIO（可选） / Identify GPIO (optional)
   * @param termination_gpios 每通道终端电阻 GPIO（可选） / Per-channel termination GPIOs
   * (optional)
   * @param database 数据库存储（可选，用于 USER_ID） / Database storage (optional, used
   * by USER_ID)
   * @param interface_string 接口字符串 / Interface string
   * @param tx_stage_size 每通道暂存队列容量（至少 1），默认 10（Linux gs_usb 驱动每通道
   * 在途帧上限）/ Per-channel staging capacity (at least 1) for host frames while the CAN
   * transmit queue is full; default 10 (the per-channel in-flight limit of the Linux
   * gs_usb driver)
   */
  GsUsbClass(Endpoint::EPNumber data_in_ep_num, Endpoint::EPNumber data_out_ep_num,
             std::initializer_list<LibXR::FDCAN*> fd_cans, size_t rx_queue_size = 32,
             size_t echo_queue_size = 32, LibXR::GPIO* identify_gpio = nullptr,
             std::initializer_list<LibXR::GPIO*> termination_gpios = {},
             LibXR::Database* database = nullptr,
             const char* interface_string = DEFAULT_INTERFACE_STRING,
             size_t tx_stage_size = DEFAULT_TX_STAGE_SIZE)
      : fd_supported_(true),
        data_in_ep_num_(data_in_ep_num),
        data_out_ep_num_(data_out_ep_num),
        interface_string_(interface_string),
        identify_gpio_(identify_gpio),
        database_(database),
        echo_queue_(echo_queue_size),
        rx_queues_(rx_queue_size),
        err_queues_(ERR_QUEUE_SIZE),
        stage_queues_(CheckedStageSize(tx_stage_size))
  {
    ASSERT(fd_cans.size() == CAN_CH_NUM);
    std::size_t i = 0;
    for (auto* p : fd_cans)
    {
      fdcans_[i] = p;
      cans_[i] = p;  // 向上转 CAN* / Upcast to CAN*
      ++i;
    }

    if (termination_gpios.size() == 0)
    {
      termination_gpios_.fill(nullptr);
    }
    else
    {
      ASSERT(termination_gpios.size() == CAN_CH_NUM);
      i = 0;
      for (auto* g : termination_gpios)
      {
        termination_gpios_[i++] = g;
      }
    }

    InitDeviceConfigFd();
    RegisterCanCallbacks();
    InitUserIdStorage();
  }

  /**
   * @brief Host 字节序协商是否通过 / Whether host format negotiation passed
   * @return true 协商通过 / Passed
   * @return false 协商未通过 / Not passed
   */
  bool IsHostFormatOK() const
  {
    return host_format_ok_.load(std::memory_order_acquire) != 0U;
  }

  void OnService(bool in_isr) override
  {
    if (!inited_) return;

    // echo 队列满时被压住的帧：现在有空位就补推并归还 OUT 租约（主机恢复收发）。
    // A frame held by echo backpressure: push it now that a slot freed up and give the
    // OUT lease back (the host resumes).
    if (out_hold_)
    {
      if (echo_queue_.Push(out_hold_item_) == ErrorCode::OK)
      {
        out_hold_ = false;
        ReturnLease(in_isr, out_hold_slot_);
      }
      // echo 队列仍满：继续持有租约背压，等下一次服务。
      // Echo queue still full: keep holding the lease and wait for the next service.
    }

    // CAN 发送队列曾经满而暂存的主机帧：按顺序重试提交，驱动接受后回 echo。
    // Host frames staged while a CAN transmit queue was full: retry their submission in
    // order and send the echo once the driver accepts a frame.
    ServiceStagedFrames();

    // 取槽 → 选帧 → 打包 → 提交；IN 槽都在途时停下，完成回调再请求服务。
    // Acquire a slot, pick a frame, pack and submit; stop when no IN slot is free, the
    // completion callback re-requests the service.
    while (true)
    {
      Endpoint::Slot slot;
      if (ep_data_in_ == nullptr ||
          ep_data_in_->AcquireSlot(slot, in_isr) != ErrorCode::OK)
      {
        break;
      }

      QueueItem item;
      if (!PickFrame(item))
      {
        ep_data_in_->ReleaseSlot(slot, in_isr);
        break;
      }

      ApplyOverflowFlag(item);

      const std::size_t size = PackQueueItemToWire(
          item, static_cast<uint8_t*>(slot.data.addr_), slot.data.size_);
      if (size == 0U)
      {
        // Granted 校验已保证槽装得下一帧，这里正常不可达。
        // The Granted check guarantees the slot holds one frame; unreachable normally.
        ep_data_in_->ReleaseSlot(slot, in_isr);
        break;
      }

      if (ep_data_in_->Submit(slot, size, false, in_isr) == ErrorCode::ABORTED)
      {
        // 槽在解绑前借出：本帧随解绑丢弃。
        // The slot went stale before submission: the frame is dropped with the binding.
        break;
      }
    }
  }

  /**
   * @brief 只在有帧暂存时要 SOF 节拍 / The SOF tick is wanted only while frames are
   *        staged
   *
   * CAN 驱动没有发送完成通知，队列腾位只能轮询。有暂存帧时每个 SOF 重试一次（全速
   * 1 ms，高速 125 us），没有时不占 SOF 中断。
   * The CAN driver has no transmit-complete notification, so freed room can only be
   * polled for. While frames are staged the retry runs once per SOF (1 ms at full
   * speed, 125 us at high speed); with nothing staged the class costs no SOF interrupt.
   */
  bool NeedsFrameTick() const override
  {
    return stage_pending_.load(std::memory_order_relaxed);
  }

  void OnFrameTick(bool in_isr, const FrameTick& time) override
  {
    UNUSED(time);
    // 直接走服务：重试成功后要推 echo，并把 echo 发给主机。
    // Run the service itself: after a successful retry the echo is queued and sent to
    // the host.
    if (stage_pending_.load(std::memory_order_relaxed)) OnService(in_isr);
  }

 protected:
  /**
   * @brief 初始化接口与端点资源 / Initialize interface and endpoints
   * @param endpoint_pool 端点池 / Endpoint pool
   * @param start_itf_num 起始接口号 / Starting interface number
   * @param in_isr 是否在中断中 / Whether in ISR
   */
  void BindEndpoints(EndpointPool& endpoint_pool, uint8_t start_itf_num,
                     bool in_isr) override
  {
    inited_ = false;
    interface_num_ = start_itf_num;
    out_hold_ = false;

    auto ans = endpoint_pool.Get(ep_data_in_, Endpoint::Direction::IN, data_in_ep_num_);
    ASSERT(ans == ErrorCode::OK);

    ans = endpoint_pool.Get(ep_data_out_, Endpoint::Direction::OUT, data_out_ep_num_);
    ASSERT(ans == ErrorCode::OK);

    const uint16_t in_packet = ep_data_in_->LargestPacketSize(Endpoint::Type::BULK);
    const uint16_t out_packet = ep_data_out_->LargestPacketSize(Endpoint::Type::BULK);
    (void)ep_data_in_->Configure(
        {Endpoint::Direction::IN, Endpoint::Type::BULK, in_packet}, in_isr);
    // receive_size = 0：ReturnSlot 按整槽重新挂接收，一帧一个请求，短包结束。
    // receive_size = 0: ReturnSlot rearms the whole slot; one frame per request, ended
    // by the short packet.
    (void)ep_data_out_->Configure(
        {Endpoint::Direction::OUT, Endpoint::Type::BULK, out_packet, 0}, in_isr);

    // 单帧上限 WIRE_MAX_SIZE：IN 槽要装下一整帧提交，OUT 槽要装下一整帧接收。
    // BSP 槽数组不足时绑定即断言；每端点（两个槽）需要 >= 2 * WIRE_MAX_SIZE。
    // WIRE_MAX_SIZE is the per-frame bound: the IN slot must hold a whole frame for
    // submission and the OUT slot a whole frame for reception. A BSP slot array below
    // that asserts at bind time; each endpoint (two slots) needs >= 2 * WIRE_MAX_SIZE.
    // 用 REQUIRE：槽太小在 release 构建里也不能静默通过——IN 槽 < 80 会静默
    // 丢弃 FD 帧，OUT 槽 < 76 会把帧切成两段破坏同步。
    // Use REQUIRE: an undersized slot must not pass silently in release builds —
    // an IN slot below 80 silently drops FD frames and an OUT slot below 76 splits
    // frames and breaks synchronisation.
    REQUIRE(ep_data_in_->GetGranted().slot_capacity >= WIRE_MAX_SIZE);
    REQUIRE(ep_data_out_->GetGranted().slot_capacity >= WIRE_MAX_SIZE);

    desc_block_.intf = {9,
                        static_cast<uint8_t>(DescriptorType::INTERFACE),
                        interface_num_,
                        0,
                        2,
                        0xFF,
                        0xFF,
                        0xFF,
                        GetInterfaceStringIndex(0u)};

    desc_block_.ep_out = {7,
                          static_cast<uint8_t>(DescriptorType::ENDPOINT),
                          ep_data_out_->GetAddress(),
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

    ep_data_out_->SetOnComplete(on_data_out_cb_);
    ep_data_in_->SetOnComplete(on_data_in_cb_);

    host_format_ok_.store(0U, std::memory_order_release);

    for (uint8_t i = 0; i < can_count_; ++i)
    {
      can_enabled_[i] = false;
      berr_enabled_[i] = false;
      fd_enabled_[i] = false;
      term_state_[i] = GsUsb::TerminationState::OFF;
      timestamps_enabled_ch_[i] = false;
    }

    // 先置 inited_ 再挂初始接收：RearmOut 会检查 inited_，否则初始那一次被静默跳过，
    // OUT 端点永远收不到主机的帧。
    // Set inited_ before arming the initial receive: RearmOut checks inited_, otherwise
    // that first arm is silently skipped and the OUT endpoint never receives a frame.
    inited_ = true;

    // 挂初始接收：一条 OUT 租约在途，处理完 ReturnSlot 自动重挂。
    // Arm the initial receive: one OUT lease in flight, ReturnSlot rearms it.
    RearmOut(in_isr);
  }

  /**
   * @brief 释放端点资源并复位状态 / Release endpoint resources and reset state
   * @param endpoint_pool 端点池 / Endpoint pool
   * @param in_isr 是否在中断中 / Whether in ISR
   */
  void UnbindEndpoints(EndpointPool& endpoint_pool, bool in_isr) override
  {
    inited_ = false;
    host_format_ok_.store(0U, std::memory_order_release);

    // 持有的 OUT 租约先释放（避免新绑定少一个槽）；被压住的 echo 随之丢弃，它属于
    // 这里结束的这次绑定。
    // Release the held OUT lease first so the next binding keeps both slots; the
    // backpressured echo is dropped with it, since it belongs to the binding that ends
    // here.
    if (out_hold_ && ep_data_out_ != nullptr)
    {
      ep_data_out_->ReleaseSlot(out_hold_slot_, in_isr);
      out_hold_ = false;
    }
    out_armed_ = false;

    // 暂存的主机帧同样属于这次绑定：没交给 CAN 驱动的不再发送也不回 echo；驱动已接受
    // 的队头已经交出，仍可能上总线，只是不回它的 echo。
    // The staged host frames belong to this binding too: those not yet handed to the CAN
    // driver are never sent and get no echo; an accepted head frame has been handed over
    // and may still go out on the bus, only its echo is not sent.
    for (uint8_t i = 0; i < can_count_; ++i)
    {
      DropStagedFrames(i);
    }

    for (uint8_t i = 0; i < can_count_; ++i)
    {
      can_enabled_[i] = false;
      berr_enabled_[i] = false;
      fd_enabled_[i] = false;
      term_state_[i] = GsUsb::TerminationState::OFF;
      timestamps_enabled_ch_[i] = false;
    }

    if (ep_data_in_ != nullptr)
    {
      ep_data_in_->Close(in_isr);
      endpoint_pool.Release(ep_data_in_);
      ep_data_in_ = nullptr;
    }

    if (ep_data_out_ != nullptr)
    {
      ep_data_out_->Close(in_isr);
      endpoint_pool.Release(ep_data_out_);
      ep_data_out_ = nullptr;
    }
  }

  /**
   * @brief 返回接口数量（实现侧固定 1） / Return interface count (fixed to 1)
   * @return size_t 接口数 / Interface count
   */
  size_t GetInterfaceCount() override { return 1; }

  /**
   * @brief 是否包含 IAD / Whether class has IAD
   * @return true 有 IAD / Has IAD
   * @return false 无 IAD / No IAD
   */
  bool HasIAD() override { return false; }

  /**
   * @brief 判断端点地址是否属于本类 / Check whether an endpoint belongs to this class
   * @param ep_addr 端点地址 / Endpoint address
   * @return true 属于 / Owned
   * @return false 不属于 / Not owned
   */
  bool OwnsEndpoint(uint8_t ep_addr) const override
  {
    if (!inited_)
    {
      return false;
    }

    return (ep_data_in_ && ep_data_in_->GetAddress() == ep_addr) ||
           (ep_data_out_ && ep_data_out_->GetAddress() == ep_addr);
  }

  /**
   * @brief 返回描述符块最大长度 / Return max configuration descriptor block size
   * @return size_t 长度 / Size
   */
  size_t GetMaxConfigSize() override { return sizeof(desc_block_); }

  /**
   * @brief 标准 Class Request 处理（此类不支持） / Handle standard class request (not
   * supported)
   * @return ErrorCode 错误码 / Error code
   */
  ErrorCode OnClassRequest(bool, uint8_t, uint16_t, uint16_t, uint16_t,
                           DeviceClass::ControlTransferResult&) override
  {
    return ErrorCode::NOT_SUPPORT;
  }

  /**
   * @brief Vendor Request（gs_usb BREQ）处理 / Handle vendor requests (gs_usb BREQ)
   * @param in_isr 是否在中断上下文 / Whether in ISR
   * @param bRequest 请求号（GsUsb::BReq） / Request number (GsUsb::BReq)
   * @param wValue wValue
   * @param wLength 数据阶段长度 / Data stage length
   * @param wIndex wIndex
   * @param result IN 数据阶段设置 / IN data stage setup
   * @return ErrorCode 错误码 / Error code
   */
  ErrorCode OnVendorRequest(bool in_isr, uint8_t bRequest, uint16_t wValue,
                            uint16_t wLength, uint16_t wIndex,
                            DeviceClass::ControlTransferResult& result) override
  {
    UNUSED(in_isr);
    UNUSED(wIndex);

    auto req = static_cast<GsUsb::BReq>(bRequest);

    switch (req)
    {
      // ===== Device -> Host =====
      case GsUsb::BReq::BT_CONST:
      {
        if (wLength < sizeof(bt_const_))
        {
          return ErrorCode::ARG_ERR;
        }
        result.write_data =
            ConstRawData{reinterpret_cast<const uint8_t*>(&bt_const_), sizeof(bt_const_)};
        return ErrorCode::OK;
      }

      case GsUsb::BReq::BT_CONST_EXT:
      {
        if (!fd_supported_)
        {
          return ErrorCode::NOT_SUPPORT;
        }
        if (wLength < sizeof(bt_const_ext_))
        {
          return ErrorCode::ARG_ERR;
        }
        result.write_data = ConstRawData{reinterpret_cast<const uint8_t*>(&bt_const_ext_),
                                         sizeof(bt_const_ext_)};
        return ErrorCode::OK;
      }

      case GsUsb::BReq::DEVICE_CONFIG:
      {
        if (wLength < sizeof(dev_cfg_))
        {
          return ErrorCode::ARG_ERR;
        }
        result.write_data =
            ConstRawData{reinterpret_cast<const uint8_t*>(&dev_cfg_), sizeof(dev_cfg_)};
        return ErrorCode::OK;
      }

      case GsUsb::BReq::TIMESTAMP:
      {
        ctrl_buf_.timestamp_us = MakeTimestampUsGlobal();
        if (wLength < sizeof(ctrl_buf_.timestamp_us))
        {
          return ErrorCode::ARG_ERR;
        }

        result.write_data =
            ConstRawData{reinterpret_cast<const uint8_t*>(&ctrl_buf_.timestamp_us),
                         sizeof(ctrl_buf_.timestamp_us)};
        return ErrorCode::OK;
      }

      case GsUsb::BReq::GET_TERMINATION:
      {
        if (wValue >= can_count_)
        {
          return ErrorCode::ARG_ERR;
        }
        ctrl_buf_.term.state = static_cast<uint32_t>(term_state_[wValue]);
        if (wLength < sizeof(ctrl_buf_.term))
        {
          return ErrorCode::ARG_ERR;
        }

        result.write_data = ConstRawData{
            reinterpret_cast<const uint8_t*>(&ctrl_buf_.term), sizeof(ctrl_buf_.term)};
        return ErrorCode::OK;
      }

      case GsUsb::BReq::GET_STATE:
      {
        if (wValue >= can_count_)
        {
          return ErrorCode::ARG_ERR;
        }

        GsUsb::CanState st = GsUsb::CanState::ERROR_ACTIVE;
        uint32_t rxerr = 0;
        uint32_t txerr = 0;

        auto* can = cans_[wValue];
        if (can != nullptr)
        {
          LibXR::CAN::ErrorState es;
          if (can->GetErrorState(es) == ErrorCode::OK)
          {
            if (es.bus_off)
            {
              st = GsUsb::CanState::BUS_OFF;
            }
            else if (es.error_passive)
            {
              st = GsUsb::CanState::ERROR_PASSIVE;
            }
            else if (es.error_warning)
            {
              st = GsUsb::CanState::ERROR_WARNING;
            }
            else
            {
              st = GsUsb::CanState::ERROR_ACTIVE;
            }

            rxerr = es.rx_error_counter;
            txerr = es.tx_error_counter;
          }
        }

        ctrl_buf_.dev_state.state = static_cast<uint32_t>(st);
        ctrl_buf_.dev_state.rxerr = rxerr;
        ctrl_buf_.dev_state.txerr = txerr;

        if (wLength < sizeof(ctrl_buf_.dev_state))
        {
          return ErrorCode::ARG_ERR;
        }

        result.write_data =
            ConstRawData{reinterpret_cast<const uint8_t*>(&ctrl_buf_.dev_state),
                         sizeof(ctrl_buf_.dev_state)};
        return ErrorCode::OK;
      }

      case GsUsb::BReq::GET_USER_ID:
      {
        // 只读 RAM 值；构造期已从 Database 载入，SET 后由 Timer 任务落库。
        // Read the RAM value only; it was loaded from the Database at construction and
        // SET writes it back through the Timer task.
        ctrl_buf_.user_id = user_id_ram_;
        if (wLength < sizeof(ctrl_buf_.user_id))
        {
          return ErrorCode::ARG_ERR;
        }

        result.write_data =
            ConstRawData{reinterpret_cast<const uint8_t*>(&ctrl_buf_.user_id),
                         sizeof(ctrl_buf_.user_id)};
        return ErrorCode::OK;
      }

      // ===== Host -> Device（有 DATA 阶段，数据经 OnControlData 交给 OnClassData）/
      // ===== Host -> Device (with a DATA stage delivered to OnClassData via
      // OnControlData)
      case GsUsb::BReq::HOST_FORMAT:
      {
        if (wLength != sizeof(GsUsb::HostConfig))
        {
          return ErrorCode::ARG_ERR;
        }
        return ErrorCode::OK;
      }

      case GsUsb::BReq::BITTIMING:
      {
        if (wLength != sizeof(GsUsb::DeviceBitTiming) || wValue >= can_count_)
        {
          return ErrorCode::ARG_ERR;
        }
        ctrl_target_channel_ = static_cast<uint8_t>(wValue);
        return ErrorCode::OK;
      }

      case GsUsb::BReq::DATA_BITTIMING:
      {
        if (!fd_supported_)
        {
          return ErrorCode::NOT_SUPPORT;
        }
        if (wLength != sizeof(GsUsb::DeviceBitTiming) || wValue >= can_count_)
        {
          return ErrorCode::ARG_ERR;
        }
        ctrl_target_channel_ = static_cast<uint8_t>(wValue);
        return ErrorCode::OK;
      }

      case GsUsb::BReq::MODE:
      {
        if (wLength != sizeof(GsUsb::DeviceMode) || wValue >= can_count_)
        {
          return ErrorCode::ARG_ERR;
        }
        ctrl_target_channel_ = static_cast<uint8_t>(wValue);
        return ErrorCode::OK;
      }

      case GsUsb::BReq::BERR:
      {
        if (wLength != sizeof(uint32_t) || wValue >= can_count_)
        {
          return ErrorCode::ARG_ERR;
        }
        ctrl_target_channel_ = static_cast<uint8_t>(wValue);
        return ErrorCode::OK;
      }

      case GsUsb::BReq::IDENTIFY:
      {
        if (wLength != sizeof(GsUsb::Identify) || wValue >= can_count_)
        {
          return ErrorCode::ARG_ERR;
        }
        ctrl_target_channel_ = static_cast<uint8_t>(wValue);
        return ErrorCode::OK;
      }

      case GsUsb::BReq::SET_TERMINATION:
      {
        if (wLength != sizeof(GsUsb::DeviceTerminationState) || wValue >= can_count_)
        {
          return ErrorCode::ARG_ERR;
        }
        ctrl_target_channel_ = static_cast<uint8_t>(wValue);
        return ErrorCode::OK;
      }

      case GsUsb::BReq::SET_USER_ID:
      {
        if (wLength != sizeof(uint32_t))
        {
          return ErrorCode::ARG_ERR;
        }
        return ErrorCode::OK;
      }

      default:
        return ErrorCode::NOT_SUPPORT;
    }
  }

  /**
   * @brief Vendor Request 的 DATA 阶段处理 / Handle DATA stage for vendor requests
   * @param in_isr 是否在中断上下文 / Whether in ISR
   * @param bRequest 请求号 / Request number
   * @param data DATA 阶段数据（由核心缓冲，需要保留须自行拷贝） / DATA stage data
   * (buffered by the core; copy it to keep it)
   * @return ErrorCode 错误码 / Error code
   */
  ErrorCode OnClassData(bool in_isr, uint8_t bRequest, LibXR::ConstRawData& data) override
  {
    UNUSED(in_isr);

    auto req = static_cast<GsUsb::BReq>(bRequest);

    switch (req)
    {
      case GsUsb::BReq::HOST_FORMAT:
        if (data.size_ != sizeof(GsUsb::HostConfig))
        {
          return ErrorCode::ARG_ERR;
        }
        Memory::FastCopy(&ctrl_buf_.host_cfg, data.addr_, sizeof(GsUsb::HostConfig));
        return HandleHostFormat(ctrl_buf_.host_cfg);

      case GsUsb::BReq::BITTIMING:
        if (data.size_ != sizeof(GsUsb::DeviceBitTiming))
        {
          return ErrorCode::ARG_ERR;
        }
        Memory::FastCopy(&ctrl_buf_.bt, data.addr_, sizeof(GsUsb::DeviceBitTiming));
        return HandleBitTiming(ctrl_target_channel_, ctrl_buf_.bt);

      case GsUsb::BReq::DATA_BITTIMING:
        if (!fd_supported_)
        {
          return ErrorCode::NOT_SUPPORT;
        }
        if (data.size_ != sizeof(GsUsb::DeviceBitTiming))
        {
          return ErrorCode::ARG_ERR;
        }
        Memory::FastCopy(&ctrl_buf_.bt, data.addr_, sizeof(GsUsb::DeviceBitTiming));
        return HandleDataBitTiming(ctrl_target_channel_, ctrl_buf_.bt);

      case GsUsb::BReq::MODE:
        if (data.size_ != sizeof(GsUsb::DeviceMode))
        {
          return ErrorCode::ARG_ERR;
        }
        Memory::FastCopy(&ctrl_buf_.mode, data.addr_, sizeof(GsUsb::DeviceMode));
        return HandleMode(ctrl_target_channel_, ctrl_buf_.mode);

      case GsUsb::BReq::BERR:
        if (data.size_ != sizeof(uint32_t))
        {
          return ErrorCode::ARG_ERR;
        }
        Memory::FastCopy(&ctrl_buf_.berr_on, data.addr_, sizeof(uint32_t));
        return HandleBerr(ctrl_target_channel_, ctrl_buf_.berr_on);

      case GsUsb::BReq::IDENTIFY:
        if (data.size_ != sizeof(GsUsb::Identify))
        {
          return ErrorCode::ARG_ERR;
        }
        Memory::FastCopy(&ctrl_buf_.identify, data.addr_, sizeof(GsUsb::Identify));
        return HandleIdentify(ctrl_target_channel_, ctrl_buf_.identify);

      case GsUsb::BReq::SET_TERMINATION:
        if (data.size_ != sizeof(GsUsb::DeviceTerminationState))
        {
          return ErrorCode::ARG_ERR;
        }
        Memory::FastCopy(&ctrl_buf_.term, data.addr_,
                         sizeof(GsUsb::DeviceTerminationState));
        return HandleSetTermination(ctrl_target_channel_, ctrl_buf_.term);

      case GsUsb::BReq::SET_USER_ID:
        if (data.size_ != sizeof(uint32_t))
        {
          return ErrorCode::ARG_ERR;
        }
        // 中断/owner 里只改内存值并置待写标志，Database 由 Timer 周期任务写入。
        // Only update the in-memory value and set the pending flag here; the Timer task
        // writes the Database.
        Memory::FastCopy(&user_id_ram_, data.addr_, sizeof(uint32_t));
        user_id_write_pending_.store(1U, std::memory_order_release);
        return ErrorCode::OK;

      case GsUsb::BReq::DEVICE_CONFIG:
      case GsUsb::BReq::BT_CONST:
      case GsUsb::BReq::BT_CONST_EXT:
      case GsUsb::BReq::TIMESTAMP:
      case GsUsb::BReq::GET_TERMINATION:
      case GsUsb::BReq::GET_STATE:
      case GsUsb::BReq::GET_USER_ID:
        return ErrorCode::OK;

      default:
        return ErrorCode::NOT_SUPPORT;
    }
  }

  /**
   * @brief 设备事件：端点解除 STALL 后恢复收发 / Device events: restart after an
   * endpoint stall is cleared
   */
  void OnDeviceEvent(bool in_isr, DeviceEvent event, uint8_t endpoint) override
  {
    if (event != DeviceEvent::ENDPOINT_RESUMED || !inited_)
    {
      return;
    }

    if (ep_data_out_ != nullptr && endpoint == ep_data_out_->GetAddress())
    {
      // 被压住时租约仍在手里，服务补推后 ReturnSlot 重挂；否则直接重挂接收。
      // While backpressured the lease is still held and the service returns it; otherwise
      // arm the receive directly.
      if (!out_hold_)
      {
        RearmOut(in_isr);
      }
      RequestClassService(in_isr);
    }

    if (ep_data_in_ != nullptr && endpoint == ep_data_in_->GetAddress())
    {
      RequestClassService(in_isr);
    }
  }

  // ================= Bulk 端点回调 / Bulk endpoint callbacks =================

  /**
   * @brief Bulk OUT 完成静态回调包装 / Static wrapper for Bulk OUT completion
   * @param in_isr 是否在中断上下文 / Whether in ISR
   * @param self this 指针 / this pointer
   * @param completion 完成信息 / Completion info
   */
  static void OnDataOutCompleteStatic(bool in_isr, GsUsbClass* self,
                                      const Endpoint::Completion& completion)
  {
    if (!self->inited_)
    {
      // 解绑后交付的租约立即归还，不重新挂接收。
      // A lease delivered after unbinding is returned at once, without rearming.
      if (completion.slot.Valid() && self->ep_data_out_)
      {
        self->ep_data_out_->ReleaseSlot(completion.slot, in_isr);
      }
      return;
    }
    self->OnDataOutComplete(in_isr, completion);
  }

  /**
   * @brief Bulk IN 完成静态回调包装 / Static wrapper for Bulk IN completion
   * @param in_isr 是否在中断上下文 / Whether in ISR
   * @param self this 指针 / this pointer
   * @param completion 完成信息 / Completion info
   */
  static void OnDataInCompleteStatic(bool in_isr, GsUsbClass* self,
                                     const Endpoint::Completion& completion)
  {
    UNUSED(completion);
    if (!self->inited_)
    {
      return;
    }
    // 一帧已被主机取走：请求服务继续从队列取帧发送。
    // A frame was taken by the host: ask the service to keep draining the queues.
    self->RequestClassService(in_isr);
  }

 private:
  // ================= 成员 / Members =================
  std::array<LibXR::CAN*, CanChNum> cans_{};  ///< CAN 通道列表 / CAN channel pointers
  std::array<LibXR::FDCAN*, CanChNum>
      fdcans_{};               ///< FDCAN 通道列表 / FDCAN channel pointers
  bool fd_supported_ = false;  ///< 是否支持 FD / FD supported

  uint8_t can_count_ = CAN_CH_NUM;  ///< 实际通道数 / Actual channel count

  Endpoint::EPNumber data_in_ep_num_;   ///< Bulk IN 端点号 / Bulk IN endpoint number
  Endpoint::EPNumber data_out_ep_num_;  ///< Bulk OUT 端点号 / Bulk OUT endpoint number

  Endpoint* ep_data_in_ = nullptr;   ///< Bulk IN 端点对象 / Bulk IN endpoint object
  Endpoint* ep_data_out_ = nullptr;  ///< Bulk OUT 端点对象 / Bulk OUT endpoint object

  std::atomic<uint32_t> inited_{0};         ///< 是否已初始化 / Initialized
  uint8_t interface_num_ = 0;               ///< 接口号 / Interface number
  const char* interface_string_ = nullptr;  ///< 接口字符串 / Interface string

  LibXR::GPIO* identify_gpio_ =
      nullptr;  ///< Identify GPIO（可选） / Identify GPIO (optional)
  std::array<LibXR::GPIO*, CanChNum>
      termination_gpios_{};  ///< 终端电阻 GPIO（可选） / Termination GPIOs (optional)

  LibXR::Database* database_ = nullptr;  ///< 数据库存储（可选，用于 USER_ID） / Database
                                         ///< storage (optional, used by USER_ID)
  uint32_t user_id_ram_ = 0;             ///< USER_ID 的 RAM 值 / USER_ID RAM value
  // 构造期创建一次的数据库键；SET_USER_ID 只置标志，Timer 任务负责写库。
  // Database key created once at construction; SET_USER_ID only sets a flag, the Timer
  // task writes it back.
  LibXR::Database::Key<uint32_t>* user_id_key_ = nullptr;
  std::atomic<uint32_t> user_id_write_pending_{
      0};  ///< USER_ID 待写标志 / USER_ID write-pending flag
  LibXR::Timer::TimerHandle user_id_timer_ = nullptr;  ///< USER_ID 落库定时任务 /
                                                       ///< USER_ID write-back timer task

  Callback<const Endpoint::Completion&> on_data_out_cb_ =
      Callback<const Endpoint::Completion&>::Create(OnDataOutCompleteStatic,
                                                    this);  ///< OUT 回调 / OUT callback

  Callback<const Endpoint::Completion&> on_data_in_cb_ =
      Callback<const Endpoint::Completion&>::Create(OnDataInCompleteStatic,
                                                    this);  ///< IN 回调 / IN callback

  GsUsb::DeviceConfig dev_cfg_{};    ///< 设备配置 / Device configuration
  GsUsb::DeviceBTConst bt_const_{};  ///< BT 常量（classic） / BT constants (classic)
  GsUsb::DeviceBTConstExtended
      bt_const_ext_{};  ///< BT 常量（扩展/FD） / BT constants (extended/FD)

  /**
   * @brief 控制传输复用缓冲区 / Union buffer for control transfers
   */
  union
  {
    GsUsb::HostConfig host_cfg;          ///< HOST_FORMAT
    GsUsb::DeviceBitTiming bt;           ///< BITTIMING / DATA_BITTIMING
    GsUsb::DeviceMode mode;              ///< MODE
    uint32_t berr_on;                    ///< BERR
    GsUsb::Identify identify;            ///< IDENTIFY
    uint32_t timestamp_us;               ///< TIMESTAMP
    GsUsb::DeviceTerminationState term;  ///< SET_TERMINATION / GET_TERMINATION
    GsUsb::DeviceState dev_state;        ///< GET_STATE
    uint32_t user_id;                    ///< GET_USER_ID
  } ctrl_buf_{};

  std::array<LibXR::CAN::Configuration, CanChNum>
      config_{};  ///< 经典 CAN 配置 / Classic CAN configuration
  std::array<LibXR::FDCAN::Configuration, CanChNum>
      fd_config_{};  ///< FD 配置 / FD configuration

  std::atomic<uint32_t> host_format_ok_{0};  ///< Public cross-context status snapshot.

  std::atomic<uint32_t>
      can_enabled_[CanChNum]{};  ///< 通道启用（classic） / Channel enabled (classic)
  std::atomic<uint32_t>
      berr_enabled_[CanChNum]{};  ///< 错误报告启用 / Bus error reporting enabled
  std::atomic<uint32_t>
      fd_enabled_[CanChNum]{};  ///< 通道启用（FD） / Channel enabled (FD)

  std::atomic<uint32_t>
      timestamps_enabled_ch_[CanChNum]{};  ///< 通道时间戳启用 / Per-channel timestamp
                                           ///< enabled
  GsUsb::TerminationState term_state_[CanChNum] = {
      GsUsb::TerminationState::OFF};  ///< 终端电阻状态 / Termination state
  uint8_t ctrl_target_channel_ =
      0;  ///< 控制请求目标通道 / Control request target channel

  LIBXR_PACKED_BEGIN
  /**
   * @brief 本类的接口与端点描述符块 / Descriptor block for interface and endpoints
   */
  struct GsUsbDescBlock
  {
    InterfaceDescriptor intf;   ///< Interface 描述符 / Interface descriptor
    EndpointDescriptor ep_out;  ///< OUT 端点描述符 / OUT endpoint descriptor
    EndpointDescriptor ep_in;   ///< IN 端点描述符 / IN endpoint descriptor
  } desc_block_{};
  LIBXR_PACKED_END

  // ================= CAN RX 回调 & Bulk IN 发送队列 / CAN RX callbacks & Bulk IN TX
  // queues =================

  /**
   * @brief CAN RX 回调上下文 / CAN RX callback context
   */
  struct CanRxCtx
  {
    GsUsbClass* self;  ///< 实例指针 / Instance pointer
    uint8_t ch;        ///< 通道号 / Channel index
  };

  /**
   * @brief FDCAN RX 回调上下文 / FDCAN RX callback context
   */
  struct FdCanRxCtx
  {
    GsUsbClass* self;  ///< 实例指针 / Instance pointer
    uint8_t ch;        ///< 通道号 / Channel index
  };

  CanRxCtx can_rx_ctx_[CanChNum]{};  ///< classic RX 上下文数组 / Classic RX contexts
  LibXR::CAN::Callback
      can_rx_cb_[CanChNum]{};  ///< classic RX 回调数组 / Classic RX callbacks

  FdCanRxCtx fd_can_rx_ctx_[CanChNum]{};  ///< FD RX 上下文数组 / FD RX contexts
  LibXR::FDCAN::CallbackFD
      fd_can_rx_cb_[CanChNum]{};  ///< FD RX 回调数组 / FD RX callbacks

  /**
   * @brief classic CAN RX 静态回调入口 / Static entry for classic CAN RX callback
   */
  static void OnCanRxStatic(bool in_isr, CanRxCtx* ctx,
                            const LibXR::CAN::ClassicPack& pack)
  {
    if (!ctx || !ctx->self || !ctx->self->inited_)
    {
      return;
    }
    ctx->self->OnCanRx(in_isr, ctx->ch, pack);
  }

  /**
   * @brief FD CAN RX 静态回调入口 / Static entry for FD CAN RX callback
   */
  static void OnFdCanRxStatic(bool in_isr, FdCanRxCtx* ctx,
                              const LibXR::FDCAN::FDPack& pack)
  {
    if (!ctx || !ctx->self || !ctx->self->inited_)
    {
      return;
    }
    ctx->self->OnFdCanRx(in_isr, ctx->ch, pack);
  }

  /**
   * @brief 发送队列元素（包含 header/data/timestamp） / TX queue item
   * (header/data/timestamp)
   */
  struct QueueItem
  {
    WireHeader hdr;                ///< 线缆头 / Wire header
    bool is_fd;                    ///< 是否 FD / Is FD
    uint8_t data_len;              ///< payload 长度（8/64） / Payload length (8/64)
    uint32_t timestamp_us;         ///< 时间戳 / Timestamp
    std::array<uint8_t, 64> data;  ///< 数据段 / Data bytes
  };

  /**
   * @brief 每通道一个 SPSC 队列的定长数组 / Fixed-size array of one SPSC queue per
   *        channel
   *
   * 队列容量是构造参数，元素不可默认构造也不可移动，用 placement new 就地构造。
   * The queue capacity is a constructor argument; the elements are neither default
   * constructible nor movable, so they are constructed in place.
   */
  class RxQueueArray
  {
   public:
    explicit RxQueueArray(size_t capacity)
    {
      for (std::size_t i = 0; i < CanChNum; ++i)
      {
        new (Slot(i)) LibXR::SPSCQueue<QueueItem>(capacity);
      }
    }

    ~RxQueueArray()
    {
      for (std::size_t i = 0; i < CanChNum; ++i)
      {
        Slot(i)->~SPSCQueue();
      }
    }

    RxQueueArray(const RxQueueArray&) = delete;
    RxQueueArray& operator=(const RxQueueArray&) = delete;

    LibXR::SPSCQueue<QueueItem>& operator[](std::size_t index) { return *Slot(index); }

   private:
    LibXR::SPSCQueue<QueueItem>* Slot(std::size_t index)
    {
      return std::launder(reinterpret_cast<LibXR::SPSCQueue<QueueItem>*>(
          storage_ + index * sizeof(LibXR::SPSCQueue<QueueItem>)));
    }

    alignas(LibXR::SPSCQueue<QueueItem>)
        std::byte storage_[sizeof(LibXR::SPSCQueue<QueueItem>) * CanChNum];
  };

  LibXR::SPSCQueue<QueueItem>
      echo_queue_;          ///< Echo 队列（回送 echo_id，生产者是 OUT 处理） / Echo queue
                            ///< (echoes echo_id; produced by the OUT handler)
  RxQueueArray rx_queues_;  ///< 每通道一个接收队列（Device->Host） / One RX queue per
                            ///< channel (Device->Host)
  // 错误帧由 CAN 的错误中断产生，数据帧由接收中断产生，两者优先级不必相同，所以错误帧
  // 单独一个队列，保证每个队列只有一个生产者上下文。
  // Error frames come from the CAN error interrupt and data frames from the receive
  // interrupt, whose priorities need not be equal, so error frames get their own queue
  // and every queue keeps one producer context.
  RxQueueArray err_queues_;
  /// 每通道暂存队列：CAN 发送队列满时的主机帧，只在 owner 里读写 / Per-channel staging
  /// queue for host frames while the CAN transmit queue is full; owner context only
  RxQueueArray stage_queues_;

  // 暂存队列队头的状态，只在 owner 访问：队头帧已被 CAN 驱动接受、只等 echo 队列腾位时
  // stage_accepted_ 为真（不会再次提交）；stage_ts_ 是驱动接受它的时间戳。
  // State of the head of each staging queue, owner only: stage_accepted_ is set when the
  // CAN driver already accepted the head frame and only room in the echo queue is
  // missing (it is never submitted again); stage_ts_ is the time it was accepted.
  bool stage_accepted_[CanChNum]{};
  uint32_t stage_ts_[CanChNum]{};
  /// 有帧在暂存队列里（给 SOF 中断里的 NeedsFrameTick 读）/ Some frame is staged (read
  /// from the SOF interrupt path by NeedsFrameTick)
  std::atomic<bool> stage_pending_{false};

  // 发往主机的调度状态，只在 owner（完成回调与服务）上下文访问。
  // Host-send scheduling state, accessed only in the owner (completions and service).
  uint8_t rr_ = 0;  ///< 通道轮流游标 / Round-robin channel cursor
  /// 下一帧先取接收帧（刚发过 echo）/ The next pick tries a received frame first (an
  /// echo went out last)
  bool rx_turn_ = false;

  // echo 背压：OUT 租约被持有，echo 帧暂存，服务补推后归还租约。
  // Echo backpressure: the OUT lease is held with the frame stashed until the service
  // pushes it and returns the lease.
  bool out_hold_ = false;
  // 是否有一条 Bulk OUT 接收在途（RearmOut 幂等用）/ Whether a Bulk OUT receive is in
  // flight (keeps RearmOut idempotent)
  bool out_armed_ = false;
  Endpoint::Slot out_hold_slot_{};
  QueueItem out_hold_item_{};

  std::atomic<uint32_t> rx_drop_[CanChNum]{};  ///< 每通道丢帧计数（仅该通道生产者写） /
                                               ///< Per-channel dropped-frame count
                                               ///< (written by that channel's producer)
  uint32_t rx_drop_reported_[CanChNum] = {};   ///< 已上报的丢帧数（仅服务上下文读写） /
                                               ///< Reported drop count (service context
                                               ///< only)

  /**
   * @brief 构造期注册 CAN RX 回调（含错误帧；FD 时另注册 FD 回调） /
   *        Register CAN RX callbacks at construction (plus error frames; FD callbacks
   *        when supported)
   */
  void RegisterCanCallbacks()
  {
    for (uint8_t ch = 0; ch < can_count_; ++ch)
    {
      if (!cans_[ch])
      {
        continue;
      }

      can_rx_ctx_[ch].self = this;
      can_rx_ctx_[ch].ch = ch;
      can_rx_cb_[ch] = LibXR::CAN::Callback::Create(OnCanRxStatic, &can_rx_ctx_[ch]);

      cans_[ch]->Register(can_rx_cb_[ch], LibXR::CAN::Type::STANDARD);
      cans_[ch]->Register(can_rx_cb_[ch], LibXR::CAN::Type::EXTENDED);
      cans_[ch]->Register(can_rx_cb_[ch], LibXR::CAN::Type::REMOTE_STANDARD);
      cans_[ch]->Register(can_rx_cb_[ch], LibXR::CAN::Type::REMOTE_EXTENDED);
      cans_[ch]->Register(can_rx_cb_[ch], LibXR::CAN::Type::ERROR);
    }

    if (fd_supported_)
    {
      for (uint8_t ch = 0; ch < can_count_; ++ch)
      {
        if (!fdcans_[ch])
        {
          continue;
        }

        fd_can_rx_ctx_[ch].self = this;
        fd_can_rx_ctx_[ch].ch = ch;
        fd_can_rx_cb_[ch] =
            LibXR::FDCAN::CallbackFD::Create(OnFdCanRxStatic, &fd_can_rx_ctx_[ch]);

        fdcans_[ch]->Register(fd_can_rx_cb_[ch], LibXR::CAN::Type::STANDARD);
        fdcans_[ch]->Register(fd_can_rx_cb_[ch], LibXR::CAN::Type::EXTENDED);
      }
    }
  }

  /**
   * @brief 构造期初始化 USER_ID 存储（Database 键 + 落库定时任务） /
   *        Initialize USER_ID storage at construction (Database key + write-back task)
   */
  void InitUserIdStorage()
  {
    if (database_ == nullptr)
    {
      return;
    }

    user_id_key_ =
        new LibXR::Database::Key<uint32_t>(*database_, "user_id", user_id_ram_);
    user_id_ram_ = static_cast<uint32_t>(*user_id_key_);

    user_id_timer_ =
        LibXR::Timer::CreateTask(OnUserIdTimerStatic, this, USER_ID_WRITE_PERIOD_MS);
    LibXR::Timer::Add(user_id_timer_);
    LibXR::Timer::Start(user_id_timer_);
  }

  /**
   * @brief USER_ID 落库定时任务入口 / Timer task entry that writes USER_ID back
   */
  static void OnUserIdTimerStatic(GsUsbClass* self)
  {
    if (self->user_id_write_pending_.exchange(0U, std::memory_order_acq_rel) == 0U)
    {
      return;
    }
    if (self->user_id_key_ != nullptr)
    {
      (void)self->user_id_key_->Set(self->user_id_ram_);
    }
  }

  /**
   * @brief classic CAN RX 处理 / Handle classic CAN RX
   */
  void OnCanRx(bool in_isr, uint8_t ch, const LibXR::CAN::ClassicPack& pack)
  {
    if (ch >= can_count_ || !inited_.load(std::memory_order_acquire))
    {
      return;
    }

    if (pack.type == LibXR::CAN::Type::ERROR)
    {
      QueueItem qi;
      if (ErrorPackToHostErrorFrame(ch, pack, qi) &&
          err_queues_[ch].Push(qi) == ErrorCode::OK)
      {
        RequestClassService(in_isr);
      }
      return;
    }

    if (!can_enabled_[ch].load(std::memory_order_relaxed))
    {
      return;
    }

    QueueItem qi;
    ClassicPackToQueueItem(pack, ch, qi);
    PushRx(ch, qi, in_isr);
  }

  /**
   * @brief FD CAN RX 处理 / Handle FD CAN RX
   */
  void OnFdCanRx(bool in_isr, uint8_t ch, const LibXR::FDCAN::FDPack& pack)
  {
    // CAN 生产者只碰自己的队列与服务请求，不碰绑定持有的端点指针。
    // CAN producers only touch their own queue and the service request, never the
    // binding-owned endpoint pointers.
    if (!fd_supported_ || ch >= can_count_ || !inited_.load(std::memory_order_acquire) ||
        !fd_enabled_[ch].load(std::memory_order_relaxed))
    {
      return;
    }

    QueueItem qi;
    FdPackToQueueItem(pack, ch, qi);

    const auto& fd_cfg = fd_config_[ch];
    if (fd_cfg.fd_mode.brs)
    {
      qi.hdr.flags |= GsUsb::CAN_FLAG_BRS;
    }
    if (fd_cfg.fd_mode.esi)
    {
      qi.hdr.flags |= GsUsb::CAN_FLAG_ESI;
    }

    PushRx(ch, qi, in_isr);
  }

  /**
   * @brief 入队一帧 RX；队列满则丢帧并计数（溢出标志随下一帧上报） /
   *        Enqueue one RX frame; a full queue drops it and counts the loss (the
   *        overflow flag rides on the next frame)
   */
  void PushRx(uint8_t ch, const QueueItem& qi, bool in_isr)
  {
    if (rx_queues_[ch].Push(qi) != ErrorCode::OK)
    {
      rx_drop_[ch].fetch_add(1U, std::memory_order_relaxed);
      return;
    }
    RequestClassService(in_isr);
  }

  /**
   * @brief 选下一帧发往主机：echo 与接收帧都有待发时轮流，只有一类时直接发；接收帧通道间
   *        轮流，通道内 FIFO /
   *        Pick the next frame for the host: echoes and received frames alternate while
   *        both are pending and a lone kind goes out directly; received frames go round
   *        robin across channels, FIFO within a channel
   *
   * echo 一律优先会让突发期间的接收帧得不到发送机会，RX 队列溢出。
   * Serving echoes first would starve the received frames during a host burst until
   * their queue overflows.
   */
  bool PickFrame(QueueItem& item)
  {
    if (rx_turn_)
    {
      if (PickReceived(item))
      {
        rx_turn_ = false;
        return true;
      }
      return echo_queue_.Pop(item) == ErrorCode::OK;
    }
    if (echo_queue_.Pop(item) == ErrorCode::OK)
    {
      rx_turn_ = true;
      return true;
    }
    return PickReceived(item);
  }

  /// 通道间轮流取一帧接收帧（错误帧先于同通道的数据帧）/ Take one received frame,
  /// channels round-robin (an error frame before the channel's data frames)
  bool PickReceived(QueueItem& item)
  {
    for (std::size_t k = 0; k < can_count_; ++k)
    {
      const uint8_t ch = static_cast<uint8_t>((rr_ + k) % can_count_);
      if (err_queues_[ch].Pop(item) == ErrorCode::OK ||
          rx_queues_[ch].Pop(item) == ErrorCode::OK)
      {
        rr_ = static_cast<uint8_t>((ch + 1U) % can_count_);
        return true;
      }
    }
    return false;
  }

  /**
   * @brief 通道丢过帧时在该通道发往主机的下一帧置 OVERFLOW 标志 /
   *        Set CAN_FLAG_OVERFLOW on the channel's next frame to the host after drops
   *
   * 主机对任何一帧（RX 数据帧、错误帧或 echo）都先按原样处理，再按该帧所属通道
   * 记一次 RX 溢出，所以标志带在该通道的下一帧上，不论是哪一类，也不另造帧。
   * The host handles any frame (RX data, error or echo) as usual and then counts one
   * RX overflow for that frame's channel, so the flag rides on the channel's next
   * frame of whatever kind, and no extra frame is made up.
   */
  void ApplyOverflowFlag(QueueItem& item)
  {
    const uint8_t ch = item.hdr.channel;
    if (ch >= can_count_)
    {
      return;
    }
    const uint32_t drops = rx_drop_[ch].load(std::memory_order_relaxed);
    if (rx_drop_reported_[ch] != drops)
    {
      item.hdr.flags |= GsUsb::CAN_FLAG_OVERFLOW;
      rx_drop_reported_[ch] = drops;
    }
  }

  /**
   * @brief 归还 OUT 租约（按 receive_size 自动重挂接收） / Return the OUT lease (the
   *        receive is rearmed per receive_size)
   */
  void ReturnLease(bool in_isr, const Endpoint::Slot& slot)
  {
    // OK：租约重新挂接收，这条接收算作在途，RearmOut 不再挂第二条；ABORTED：槽随
    // 解绑作废，无需处理。
    // OK: the lease rearms the receive, which counts as in flight so RearmOut arms no
    // second one; ABORTED: the slot went stale, nothing to do.
    if (ep_data_out_->ReturnSlot(slot, in_isr) == ErrorCode::OK) out_armed_ = true;
  }

  /**
   * @brief 挂一条整槽接收（单条 OUT 租约在途） / Arm one whole-slot receive (a single
   *        OUT lease in flight)
   */
  void RearmOut(bool in_isr)
  {
    // out_hold_ 表示背压中（租约在手里），out_armed_ 表示已经有一条接收在途：
    // HALT 后的 ABORTED 重挂与 ENDPOINT_RESUMED 重挂可能同时发生，没有在途
    // 判断就会挂出第二条接收，两个租约同时在途时后到的完成会覆盖
    // out_hold_slot_（前一个槽泄漏、echo 丢失）。
    // out_hold_ means backpressured (lease held), out_armed_ means a receive is
    // already in flight: the ABORTED rearm after a HALT and the ENDPOINT_RESUMED
    // rearm can both fire, and without the in-flight check a second receive is
    // armed; the later completion then overwrites out_hold_slot_ (leaking the
    // earlier slot and losing the echo).
    if (!inited_ || ep_data_out_ == nullptr || out_hold_ || out_armed_)
    {
      return;
    }
    if (ep_data_out_->Receive(ep_data_out_->GetGranted().slot_capacity, in_isr) ==
        ErrorCode::OK)
    {
      out_armed_ = true;
    }
  }

  // ================= 业务处理函数 / Handlers =================

  /**
   * @brief 是否存在任意 termination GPIO / Whether any termination GPIO exists
   * @return true 存在 / Exists
   * @return false 不存在 / Not exists
   */
  bool HasAnyTerminationGpio() const
  {
    for (uint8_t i = 0; i < can_count_; ++i)
    {
      if (termination_gpios_[i] != nullptr)
      {
        return true;
      }
    }
    return false;
  }

  /**
   * @brief 初始化 classic CAN 设备配置与能力位 / Initialize classic CAN device config and
   * feature bits
   */
  void InitDeviceConfigClassic()
  {
    can_count_ = CAN_CH_NUM;
    ASSERT(cans_[0] != nullptr);

    dev_cfg_.reserved1 = 0;
    dev_cfg_.reserved2 = 0;
    dev_cfg_.reserved3 = 0;
    dev_cfg_.icount = static_cast<uint8_t>(can_count_ - 1);
    dev_cfg_.sw_version = 2;
    dev_cfg_.hw_version = 1;

    const uint32_t FCLK = cans_[0]->GetClockFreq();

    bt_const_.feature = GsUsb::CAN_FEAT_LISTEN_ONLY | GsUsb::CAN_FEAT_LOOP_BACK |
                        GsUsb::CAN_FEAT_TRIPLE_SAMPLE | GsUsb::CAN_FEAT_ONE_SHOT |
                        GsUsb::CAN_FEAT_HW_TIMESTAMP | GsUsb::CAN_FEAT_BERR_REPORTING |
                        GsUsb::CAN_FEAT_GET_STATE | GsUsb::CAN_FEAT_USER_ID;

    if (identify_gpio_)
    {
      bt_const_.feature |= GsUsb::CAN_FEAT_IDENTIFY;
    }
    if (HasAnyTerminationGpio())
    {
      bt_const_.feature |= GsUsb::CAN_FEAT_TERMINATION;
    }

    bt_const_.fclk_can = FCLK;
    bt_const_.btc.tseg1_min = 1;
    bt_const_.btc.tseg1_max = 16;
    bt_const_.btc.tseg2_min = 1;
    bt_const_.btc.tseg2_max = 8;
    bt_const_.btc.sjw_max = 4;
    bt_const_.btc.brp_min = 1;
    bt_const_.btc.brp_max = 1024;
    bt_const_.btc.brp_inc = 1;

    bt_const_ext_.feature = bt_const_.feature;
    bt_const_ext_.fclk_can = FCLK;
    bt_const_ext_.btc = bt_const_.btc;
    bt_const_ext_.dbtc = bt_const_.btc;

    Memory::FastSet(config_.data(), 0, sizeof(config_));
    Memory::FastSet(fd_config_.data(), 0, sizeof(fd_config_));
  }

  /**
   * @brief 初始化 FD 设备配置与能力位 / Initialize FD device config and feature bits
   */
  void InitDeviceConfigFd()
  {
    can_count_ = CAN_CH_NUM;
    ASSERT(cans_[0] != nullptr);

    dev_cfg_.reserved1 = 0;
    dev_cfg_.reserved2 = 0;
    dev_cfg_.reserved3 = 0;
    dev_cfg_.icount = static_cast<uint8_t>(can_count_ - 1);
    dev_cfg_.sw_version = 2;
    dev_cfg_.hw_version = 1;

    const uint32_t FCLK = cans_[0]->GetClockFreq();

    bt_const_.feature = GsUsb::CAN_FEAT_LISTEN_ONLY | GsUsb::CAN_FEAT_LOOP_BACK |
                        GsUsb::CAN_FEAT_TRIPLE_SAMPLE | GsUsb::CAN_FEAT_ONE_SHOT |
                        GsUsb::CAN_FEAT_HW_TIMESTAMP | GsUsb::CAN_FEAT_BERR_REPORTING |
                        GsUsb::CAN_FEAT_GET_STATE | GsUsb::CAN_FEAT_USER_ID |
                        GsUsb::CAN_FEAT_FD | GsUsb::CAN_FEAT_BT_CONST_EXT;

    if (identify_gpio_)
    {
      bt_const_.feature |= GsUsb::CAN_FEAT_IDENTIFY;
    }
    if (HasAnyTerminationGpio())
    {
      bt_const_.feature |= GsUsb::CAN_FEAT_TERMINATION;
    }

    bt_const_.fclk_can = FCLK;
    bt_const_.btc.tseg1_min = 1;
    bt_const_.btc.tseg1_max = 16;
    bt_const_.btc.tseg2_min = 1;
    bt_const_.btc.tseg2_max = 8;
    bt_const_.btc.sjw_max = 4;
    bt_const_.btc.brp_min = 1;
    bt_const_.btc.brp_max = 1024;
    bt_const_.btc.brp_inc = 1;

    bt_const_ext_.feature = bt_const_.feature;
    bt_const_ext_.fclk_can = FCLK;
    bt_const_ext_.btc = bt_const_.btc;
    bt_const_ext_.dbtc = bt_const_.btc;

    Memory::FastSet(config_.data(), 0, sizeof(config_));
    Memory::FastSet(fd_config_.data(), 0, sizeof(fd_config_));
  }

  /**
   * @brief 处理 HOST_FORMAT / Handle HOST_FORMAT
   * @param cfg HostConfig
   * @return ErrorCode 错误码 / Error code
   */
  ErrorCode HandleHostFormat(const GsUsb::HostConfig& cfg)
  {
    host_format_ok_.store(cfg.byte_order == 0x0000beefu ? 1U : 0U,
                          std::memory_order_release);
    return ErrorCode::OK;
  }

  /**
   * @brief 处理 BITTIMING（仲裁相位） / Handle BITTIMING (arbitration phase)
   */
  ErrorCode HandleBitTiming(uint8_t ch, const GsUsb::DeviceBitTiming& bt)
  {
    if (!host_format_ok_ || ch >= can_count_ || !cans_[ch])
    {
      return ErrorCode::ARG_ERR;
    }

    const uint32_t TSEG1 = bt.prop_seg + bt.phase_seg1;
    const uint32_t TSEG2 = bt.phase_seg2;
    const uint32_t TQ_NUM = 1u + TSEG1 + TSEG2;

    const uint32_t FCLK = cans_[ch]->GetClockFreq();

    auto& cfg = config_[ch];
    cfg.bit_timing.brp = bt.brp;
    cfg.bit_timing.prop_seg = bt.prop_seg;
    cfg.bit_timing.phase_seg1 = bt.phase_seg1;
    cfg.bit_timing.phase_seg2 = bt.phase_seg2;
    cfg.bit_timing.sjw = bt.sjw;

    if (bt.brp != 0u && TQ_NUM != 0u)
    {
      cfg.bitrate = FCLK / (bt.brp * TQ_NUM);
      cfg.sample_point = static_cast<float>(1u + TSEG1) / static_cast<float>(TQ_NUM);
    }
    else
    {
      cfg.bitrate = 0;
      cfg.sample_point = 0.0f;
    }

    if (fd_supported_ && fdcans_[ch])
    {
      auto& fd_cfg = fd_config_[ch];
      static_cast<CAN::Configuration&>(fd_cfg) = cfg;
    }

    return cans_[ch]->SetConfig(cfg);
  }

  /**
   * @brief 处理 DATA_BITTIMING（数据相位） / Handle DATA_BITTIMING (data phase)
   */
  ErrorCode HandleDataBitTiming(uint8_t ch, const GsUsb::DeviceBitTiming& bt)
  {
    if (!host_format_ok_)
    {
      return ErrorCode::ARG_ERR;
    }
    if (!fd_supported_ || ch >= can_count_ || !fdcans_[ch])
    {
      return ErrorCode::NOT_SUPPORT;
    }

    const uint32_t TSEG1 = bt.prop_seg + bt.phase_seg1;
    const uint32_t TSEG2 = bt.phase_seg2;
    const uint32_t TQ_NUM = 1u + TSEG1 + TSEG2;

    const uint32_t FCLK = fdcans_[ch]->GetClockFreq();

    auto& fd_cfg = fd_config_[ch];
    static_cast<CAN::Configuration&>(fd_cfg) = config_[ch];

    fd_cfg.data_timing.brp = bt.brp;
    fd_cfg.data_timing.prop_seg = bt.prop_seg;
    fd_cfg.data_timing.phase_seg1 = bt.phase_seg1;
    fd_cfg.data_timing.phase_seg2 = bt.phase_seg2;
    fd_cfg.data_timing.sjw = bt.sjw;

    if (bt.brp != 0u && TQ_NUM != 0u)
    {
      fd_cfg.data_bitrate = FCLK / (bt.brp * TQ_NUM);
      fd_cfg.data_sample_point =
          static_cast<float>(1u + TSEG1) / static_cast<float>(TQ_NUM);
    }
    else
    {
      fd_cfg.data_bitrate = 0;
      fd_cfg.data_sample_point = 0.0f;
    }

    return fdcans_[ch]->SetConfig(fd_cfg);
  }

  /// 暂存队列里有没有帧 / Whether any staging queue holds a frame
  bool AnyStaged()
  {
    for (uint8_t i = 0; i < can_count_; ++i)
    {
      if (stage_queues_[i].Size() != 0U) return true;
    }
    return false;
  }

  /**
   * @brief 把一帧提交给 CAN 驱动 / Submit one frame to the CAN driver
   * @return 驱动的返回值；FULL 表示发送队列满、帧没有被接受 / The driver's result; FULL
   *         means the transmit queue is full and the frame was not accepted
   */
  ErrorCode SubmitToCan(uint8_t ch, bool is_fd, const WireHeader& hdr,
                        const uint8_t* payload)
  {
    if (is_fd)
    {
      LibXR::FDCAN::FDPack fd{};
      HostWireToFdPack(hdr, payload, fd);
      return fdcans_[ch]->AddMessage(fd);
    }
    LibXR::CAN::ClassicPack classic{};
    HostWireToClassicPack(hdr, payload, classic);
    return cans_[ch]->AddMessage(classic);
  }

  /**
   * @brief 丢弃该通道暂存的主机帧，一律不回 echo / Drop the host frames staged for this
   *        channel; none of them is echoed
   *
   * 还没交给 CAN 驱动的帧因此不会发送。队头如果已被驱动接受（stage_accepted_，只差
   * echo 队列腾位），它已经交给了驱动，不会被撤回，仍可能发到总线上，只是不再回它的
   * echo。
   * A frame not yet handed to the CAN driver is therefore never sent. A head frame the
   * driver already accepted (stage_accepted_, only room in the echo queue is missing)
   * has been handed over and is not withdrawn: it may still go out on the bus, only its
   * echo is not sent.
   *
   * 只在 owner 中调用，owner 是暂存队列唯一的生产者和消费者。
   * Owner only: the owner is the only producer and consumer of the staging queues.
   */
  void DropStagedFrames(uint8_t ch)
  {
    QueueItem item;
    while (stage_queues_[ch].Pop(item) == ErrorCode::OK)
    {
    }
    stage_accepted_[ch] = false;
    stage_pending_.store(AnyStaged(), std::memory_order_relaxed);
  }

  /**
   * @brief 按顺序重试各通道暂存的主机帧；驱动接受一帧后才回它的 echo /
   *        Retry the staged host frames of every channel in order; a frame's echo is
   *        sent only after the driver accepted it
   *
   * 每个通道只看自己的队头：一个通道的发送队列还是满的，只停下这个通道，其他通道照常。
   * 帧被驱动接受后 echo 队列恰好满时，队头保留并标记为已接受，下次只补推 echo，不会
   * 再次提交。
   * Each channel looks at its own head only: a channel whose transmit queue is still full
   * stops that channel alone. When the echo queue happens to be full right after the
   * driver accepted a frame, the head stays, marked as accepted, and only its echo is
   * pushed next time; the frame is never submitted again.
   */
  void ServiceStagedFrames()
  {
    if (!stage_pending_.load(std::memory_order_relaxed)) return;

    for (uint8_t ch = 0; ch < can_count_; ++ch)
    {
      QueueItem item;
      while (stage_queues_[ch].Peek(item) == ErrorCode::OK)
      {
        if (!stage_accepted_[ch])
        {
          // 正常情况下通道停止时已经丢弃了暂存帧；这里是兜底，通道已停就不再发送。
          // The staged frames are normally dropped when the channel stops; this is a
          // safety net so that a stopped channel never sends them.
          if (!can_enabled_[ch].load(std::memory_order_relaxed) ||
              (item.is_fd && !fd_enabled_[ch].load(std::memory_order_relaxed)))
          {
            DropStagedFrames(ch);
            break;
          }

          // 仍然满：这个通道停在队头，下一次服务或 SOF 节拍再试。
          // Still full: this channel stays at its head until the next service or SOF
          // tick.
          if (SubmitToCan(ch, item.is_fd, item.hdr, item.data.data()) == ErrorCode::FULL)
          {
            break;
          }
          // 其他结果（OK 或别的错误码）与直接提交时一样处理：这一帧不再提交。
          // Any other result (OK or another error code) is handled as for a direct
          // submission: the frame is not submitted again.
          stage_accepted_[ch] = true;
          stage_ts_[ch] = MakeTimestampUs(ch);
        }

        if (item.hdr.echo_id != ECHO_ID_RX)
        {
          item.timestamp_us = stage_ts_[ch];
          // echo 队列满：队头保持“已接受”，下次服务只补推 echo。
          // Echo queue full: the head stays "accepted" and the next service only pushes
          // the echo.
          if (echo_queue_.Push(item) != ErrorCode::OK) break;
        }
        stage_accepted_[ch] = false;
        (void)stage_queues_[ch].Pop(item);
      }
    }
    stage_pending_.store(AnyStaged(), std::memory_order_relaxed);
  }

  /**
   * @brief 通道启动：丢弃上一次会话留下、还没发给主机的接收帧、错误帧和这个通道的回显。
   *        CAN 帧有时效性，回显 ID 属于上一个主机会话；其他通道的回显保留，顺序不变。
   *        在 owner 里执行：owner 是接收队列和错误队列的消费者，也是回显队列唯一的生产者
   *        和消费者。已经交给 IN 端点的帧不在这里处理。
   *        Channel start: drop the receive frames, the error frames and this channel's
   *        echoes that the previous session left unsent. CAN frames are time-bound and
   *        echo IDs belong to the previous host session; the echoes of other channels
   *        stay, in order. Runs in the owner, which consumes the receive and error
   *        queues and is the only producer and consumer of the echo queue. Frames
   *        already handed to the IN endpoint are not touched here.
   *
   *        这个通道暂存的主机帧也在这里丢弃（DropStagedFrames）：还没交给 CAN 驱动的
   *        不发送、不回 echo；驱动已接受的队头已经交出，可能仍会上总线，只是不回它的
   *        echo。
   *        The host frames staged for this channel are dropped here too
   *        (DropStagedFrames): those not yet handed to the CAN driver are never sent and
   *        get no echo; an accepted head frame has been handed over and may still go out
   *        on the bus, only its echo is not sent.
   */
  void DropStaleFrames(uint8_t ch)
  {
    DropStagedFrames(ch);
    QueueItem item;
    while (rx_queues_[ch].Pop(item) == ErrorCode::OK)
    {
    }
    while (err_queues_[ch].Pop(item) == ErrorCode::OK)
    {
    }
    for (size_t n = echo_queue_.Size(); n != 0U; --n)
    {
      if (echo_queue_.Pop(item) != ErrorCode::OK) break;
      if (item.hdr.channel != ch) (void)echo_queue_.Push(item);
    }
  }

  /**
   * @brief 处理 MODE（启动/复位 + 标志位） / Handle MODE (start/reset + flags)
   */
  ErrorCode HandleMode(uint8_t ch, const GsUsb::DeviceMode& mode)
  {
    if (!host_format_ok_ || ch >= can_count_ || !cans_[ch])
    {
      return ErrorCode::ARG_ERR;
    }

    switch (static_cast<GsUsb::CanMode>(mode.mode))
    {
      case GsUsb::CanMode::RESET:
        can_enabled_[ch] = false;
        fd_enabled_[ch] = false;
        // 这个通道暂存的主机帧属于要结束的会话：丢弃，都不回 echo；没交给 CAN 驱动的
        // 不发送，驱动已接受的队头已经交出，仍可能上总线。
        // The host frames staged for this channel belong to the session that ends: drop
        // them, none is echoed; those not yet handed to the CAN driver are never sent, an
        // accepted head frame has been handed over and may still go out on the bus.
        DropStagedFrames(ch);
        break;

      case GsUsb::CanMode::START:
        DropStaleFrames(ch);
        can_enabled_[ch] = true;
        if (fd_supported_ && fdcans_[ch] && (mode.flags & GsUsb::GSCAN_MODE_FD))
        {
          fd_enabled_[ch] = true;
        }
        break;

      default:
        return ErrorCode::ARG_ERR;
    }

    auto& cfg = config_[ch];
    cfg.mode.loopback = (mode.flags & GsUsb::GSCAN_MODE_LOOP_BACK) != 0;
    cfg.mode.listen_only = (mode.flags & GsUsb::GSCAN_MODE_LISTEN_ONLY) != 0;
    cfg.mode.triple_sampling = (mode.flags & GsUsb::GSCAN_MODE_TRIPLE_SAMPLE) != 0;
    cfg.mode.one_shot = (mode.flags & GsUsb::GSCAN_MODE_ONE_SHOT) != 0;

    timestamps_enabled_ch_[ch] = (mode.flags & GsUsb::GSCAN_MODE_HW_TIMESTAMP) != 0;
    berr_enabled_[ch] = (mode.flags & GsUsb::GSCAN_MODE_BERR_REPORTING) != 0;

    const ErrorCode EC = cans_[ch]->SetConfig(cfg);

    if (fd_supported_ && fdcans_[ch])
    {
      auto& fd_cfg = fd_config_[ch];
      static_cast<CAN::Configuration&>(fd_cfg) = cfg;
      fd_cfg.fd_mode.fd_enabled = (mode.flags & GsUsb::GSCAN_MODE_FD) != 0;
      (void)fdcans_[ch]->SetConfig(fd_cfg);
    }

    return EC;
  }

  /**
   * @brief 处理 BERR 开关 / Handle BERR enable/disable
   */
  ErrorCode HandleBerr(uint8_t ch, uint32_t berr_on)
  {
    if (ch >= can_count_)
    {
      return ErrorCode::ARG_ERR;
    }
    berr_enabled_[ch] = (berr_on != 0);
    return ErrorCode::OK;
  }

  /**
   * @brief 处理 IDENTIFY / Handle IDENTIFY
   */
  ErrorCode HandleIdentify(uint8_t, const GsUsb::Identify& id)
  {
    const bool ON = (id.mode == static_cast<uint32_t>(GsUsb::IdentifyMode::ON));
    if (identify_gpio_)
    {
      (void)identify_gpio_->Write(ON);
    }
    return ErrorCode::OK;
  }

  /**
   * @brief 处理 SET_TERMINATION / Handle SET_TERMINATION
   */
  ErrorCode HandleSetTermination(uint8_t ch, const GsUsb::DeviceTerminationState& st)
  {
    if (ch >= can_count_)
    {
      return ErrorCode::ARG_ERR;
    }

    term_state_[ch] = static_cast<GsUsb::TerminationState>(
        st.state != 0 ? static_cast<uint32_t>(GsUsb::TerminationState::ON)
                      : static_cast<uint32_t>(GsUsb::TerminationState::OFF));

    if (termination_gpios_[ch])
    {
      const bool ON = (term_state_[ch] == GsUsb::TerminationState::ON);
      (void)termination_gpios_[ch]->Write(ON);
    }

    return ErrorCode::OK;
  }

  /**
   * @brief Bulk OUT 完成处理（Host->Device） / Handle Bulk OUT completion (Host->Device)
   * @param in_isr 是否在中断上下文 / Whether in ISR
   * @param completion 完成信息（含借出的租约） / Completion info (with the lent lease)
   */
  void OnDataOutComplete(bool in_isr, const Endpoint::Completion& completion)
  {
    // 这一条接收已经结束（无论成功与否）：清掉在途标记，RearmOut 才能再挂。
    // This receive is over either way: clear the in-flight flag so RearmOut can
    // arm again.
    out_armed_ = false;

    if (completion.result != ErrorCode::OK)
    {
      // 异常完成（ABORTED 等）没有可用租约；保持 OUT 可收。
      // An abnormal completion (ABORTED etc.) carries no usable lease; keep OUT armed.
      if (completion.slot.Valid() && ep_data_out_)
      {
        ep_data_out_->ReleaseSlot(completion.slot, in_isr);
      }
      RearmOut(in_isr);
      return;
    }

    const std::size_t RXLEN = completion.actual;
    if (host_format_ok_.load(std::memory_order_acquire) == 0U ||
        RXLEN < WIRE_CLASSIC_SIZE)
    {
      ReturnLease(in_isr, completion.slot);
      return;
    }

    // 解析 wire 头并校验 / Parse the wire header and validate.
    const WireHeader& wh =
        *reinterpret_cast<const WireHeader*>(completion.slot.data.addr_);
    const uint8_t CH = wh.channel;
    const bool IS_FD = (wh.flags & GsUsb::CAN_FLAG_FD) != 0;
    const uint8_t* payload =
        reinterpret_cast<const uint8_t*>(completion.slot.data.addr_) + WIRE_HDR_SIZE;

    LibXR::CAN::ClassicPack classic{};
    LibXR::FDCAN::FDPack fd{};
    bool accept = false;

    if (CH < can_count_ && cans_[CH] && !IS_FD &&
        can_enabled_[CH].load(std::memory_order_relaxed) && RXLEN >= WIRE_CLASSIC_SIZE)
    {
      HostWireToClassicPack(wh, payload, classic);
      accept = true;
    }
    else if (CH < can_count_ && cans_[CH] && IS_FD && fd_supported_ && fdcans_[CH] &&
             fd_enabled_[CH].load(std::memory_order_relaxed) && RXLEN >= WIRE_FD_SIZE)
    {
      HostWireToFdPack(wh, payload, fd);
      accept = true;
    }

    if (!accept)
    {
      ReturnLease(in_isr, completion.slot);
      return;
    }

    // echo 的含义是“CAN 驱动已接受这一帧”。CAN TX 先提交，且只提交一次。
    // 该通道的暂存队列非空时，新帧不直接交给驱动，而是排到暂存队列末尾，保持顺序。
    // CAN 发送队列满（FULL）时驱动没有接受这一帧：把它暂存到该通道自己的队列，现在不回
    // echo，OUT 租约立即归还（不阻塞另一个通道，两个通道共用同一条 OUT 端点）；服务和
    // SOF 节拍按顺序重试，驱动接受后才回 echo（见 ServiceStagedFrames）。每通道的流控靠
    // 主机每通道的在途帧上限：Linux gs_usb 驱动每通道最多 10 帧在途，收到 echo 才发下
    // 一帧，所以默认容量的暂存队列不会满。暂存队列满了（主机不遵守该上限）时，这一帧
    // 被丢弃，仍然回 echo。
    // The echo means "the CAN driver accepted this frame". Submit the CAN TX first and
    // exactly once. While the channel's staging queue is not empty a new frame is not
    // handed to the driver but goes to the end of that queue, to keep the order. When
    // the CAN transmit queue is full (FULL) the driver did not accept the frame: it is
    // staged in the channel's own queue, no echo is sent yet, and the OUT lease goes
    // back at once (the other channel is not held up, both share one OUT endpoint); the
    // service and the SOF tick retry in order, and the echo is sent after the driver
    // accepted the frame (see ServiceStagedFrames). Per-channel flow control rests on
    // the host's per-channel in-flight limit: the Linux gs_usb driver keeps at most 10
    // frames per channel in flight and sends the next only after an echo, so a staging
    // queue of the default capacity does not fill up. When it is full (a host that
    // ignores that limit) the frame is dropped and still echoed.
    const bool NEED_ECHO = wh.echo_id != ECHO_ID_RX;
    QueueItem qi;
    qi.hdr = wh;
    qi.is_fd = IS_FD;
    qi.data_len = IS_FD ? 64u : 8u;
    qi.timestamp_us = 0U;
    Memory::FastCopy(qi.data.data(), payload, qi.data_len);

    auto& stage = stage_queues_[CH];
    if (stage.Size() != 0U || (IS_FD ? fdcans_[CH]->AddMessage(fd)
                                     : cans_[CH]->AddMessage(classic)) == ErrorCode::FULL)
    {
      if (stage.Push(qi) == ErrorCode::OK)
      {
        stage_pending_.store(true, std::memory_order_relaxed);
        // 请求服务：先重试一次，并让 owner 在这一轮之后打开 SOF 节拍。
        // Request the service: it retries once now and makes the owner switch the SOF
        // tick on after this pass.
        RequestClassService(in_isr);
        ReturnLease(in_isr, completion.slot);
        return;
      }
      // 暂存队列满：丢弃这一帧，下面照常回 echo（原行为）。
      // The staging queue is full: the frame is dropped and echoed below as before.
    }

    // TX echo：host 通过 echo_id 跟踪 TX buffer；设备需回送 echo_id。
    // TX echo: the host tracks TX buffers via echo_id, and the device should echo it
    // back.
    if (NEED_ECHO)
    {
      qi.timestamp_us = MakeTimestampUs(CH);

      if (echo_queue_.Push(qi) != ErrorCode::OK)
      {
        // echo 队列满：不归还 OUT 租约（主机收到 NAK），不再静默丢弃；
        // 服务腾出空间后补推并归还租约。
        // Echo queue full: keep the OUT lease (the host gets NAK) instead of dropping
        // the echo silently; the service pushes it later and returns the lease.
        out_hold_ = true;
        out_hold_slot_ = completion.slot;
        out_hold_item_ = qi;
        RequestClassService(in_isr);
        return;
      }
      RequestClassService(in_isr);
    }

    ReturnLease(in_isr, completion.slot);
  }

  // ================= 转换函数 / Converters =================

  /**
   * @brief wire -> ClassicPack / Convert wire frame to ClassicPack
   */
  static void HostWireToClassicPack(const WireHeader& wh, const uint8_t* payload,
                                    LibXR::CAN::ClassicPack& pack)
  {
    const uint32_t CID = wh.can_id;
    const bool IS_EFF = (CID & GsUsb::CAN_EFF_FLAG) != 0;
    const bool IS_RTR = (CID & GsUsb::CAN_RTR_FLAG) != 0;

    if (IS_EFF)
    {
      pack.id = CID & GsUsb::CAN_EFF_MASK;
      pack.type = IS_RTR ? LibXR::CAN::Type::REMOTE_EXTENDED : LibXR::CAN::Type::EXTENDED;
    }
    else
    {
      pack.id = CID & GsUsb::CAN_SFF_MASK;
      pack.type = IS_RTR ? LibXR::CAN::Type::REMOTE_STANDARD : LibXR::CAN::Type::STANDARD;
    }

    uint8_t dlc = wh.can_dlc;
    if (dlc > 8u)
    {
      dlc = 8u;
    }
    pack.dlc = dlc;

    if (dlc > 0u)
    {
      Memory::FastCopy(pack.data, payload, dlc);
    }
  }

  /**
   * @brief wire -> FDPack / Convert wire frame to FDPack
   */
  static void HostWireToFdPack(const WireHeader& wh, const uint8_t* payload,
                               LibXR::FDCAN::FDPack& pack)
  {
    const uint32_t CID = wh.can_id;
    const bool IS_EFF = (CID & GsUsb::CAN_EFF_FLAG) != 0;
    const bool IS_RTR = (CID & GsUsb::CAN_RTR_FLAG) != 0;

    if (IS_EFF)
    {
      pack.id = CID & GsUsb::CAN_EFF_MASK;
      pack.type = IS_RTR ? LibXR::CAN::Type::REMOTE_EXTENDED : LibXR::CAN::Type::EXTENDED;
    }
    else
    {
      pack.id = CID & GsUsb::CAN_SFF_MASK;
      pack.type = IS_RTR ? LibXR::CAN::Type::REMOTE_STANDARD : LibXR::CAN::Type::STANDARD;
    }

    uint8_t len = DlcToLen(wh.can_dlc);
    if (len > 64)
    {
      len = 64;
    }
    pack.len = len;

    if (len > 0u)
    {
      Memory::FastCopy(pack.data, payload, len);
    }
  }

  /**
   * @brief ClassicPack -> QueueItem / Convert ClassicPack to QueueItem
   */
  void ClassicPackToQueueItem(const LibXR::CAN::ClassicPack& pack, uint8_t ch,
                              QueueItem& qi)
  {
    uint32_t cid = 0;
    switch (pack.type)
    {
      case LibXR::CAN::Type::STANDARD:
        cid = (pack.id & GsUsb::CAN_SFF_MASK);
        break;
      case LibXR::CAN::Type::EXTENDED:
        cid = (pack.id & GsUsb::CAN_EFF_MASK) | GsUsb::CAN_EFF_FLAG;
        break;
      case LibXR::CAN::Type::REMOTE_STANDARD:
        cid = (pack.id & GsUsb::CAN_SFF_MASK) | GsUsb::CAN_RTR_FLAG;
        break;
      case LibXR::CAN::Type::REMOTE_EXTENDED:
        cid = (pack.id & GsUsb::CAN_EFF_MASK) | GsUsb::CAN_EFF_FLAG | GsUsb::CAN_RTR_FLAG;
        break;
      default:
        cid = pack.id & GsUsb::CAN_SFF_MASK;
        break;
    }

    qi.hdr.echo_id = ECHO_ID_RX;
    qi.hdr.can_id = cid;
    qi.hdr.can_dlc = (pack.dlc <= 8u) ? pack.dlc : 8u;
    qi.hdr.channel = ch;
    qi.hdr.flags = 0;
    qi.hdr.reserved = 0;

    qi.is_fd = false;
    qi.data_len = 8;
    qi.timestamp_us = MakeTimestampUs(ch);

    Memory::FastSet(qi.data.data(), 0, 8);
    if (qi.hdr.can_dlc > 0u)
    {
      Memory::FastCopy(qi.data.data(), pack.data, qi.hdr.can_dlc);
    }
  }

  /**
   * @brief FDPack -> QueueItem / Convert FDPack to QueueItem
   */
  void FdPackToQueueItem(const LibXR::FDCAN::FDPack& pack, uint8_t ch, QueueItem& qi)
  {
    uint32_t cid = 0;
    switch (pack.type)
    {
      case LibXR::CAN::Type::STANDARD:
        cid = (pack.id & GsUsb::CAN_SFF_MASK);
        break;
      case LibXR::CAN::Type::EXTENDED:
        cid = (pack.id & GsUsb::CAN_EFF_MASK) | GsUsb::CAN_EFF_FLAG;
        break;
      case LibXR::CAN::Type::REMOTE_STANDARD:
        cid = (pack.id & GsUsb::CAN_SFF_MASK) | GsUsb::CAN_RTR_FLAG;
        break;
      case LibXR::CAN::Type::REMOTE_EXTENDED:
        cid = (pack.id & GsUsb::CAN_EFF_MASK) | GsUsb::CAN_EFF_FLAG | GsUsb::CAN_RTR_FLAG;
        break;
      default:
        cid = pack.id & GsUsb::CAN_SFF_MASK;
        break;
    }

    qi.hdr.echo_id = ECHO_ID_RX;
    qi.hdr.can_id = cid;
    qi.hdr.can_dlc = LenToDlc(pack.len);
    qi.hdr.channel = ch;
    qi.hdr.flags = GsUsb::CAN_FLAG_FD;
    qi.hdr.reserved = 0;

    qi.is_fd = true;
    qi.data_len = 64;
    qi.timestamp_us = MakeTimestampUs(ch);

    const uint8_t LEN = (pack.len <= 64) ? pack.len : 64;

    if (LEN > 0u)
    {
      Memory::FastCopy(qi.data.data(), pack.data, LEN);
    }
    if (LEN < 64u)
    {
      Memory::FastSet(qi.data.data() + LEN, 0, 64u - LEN);
    }
  }

  /**
   * @brief QueueItem -> wire bytes / Convert QueueItem to wire bytes
   * @param qi 队列元素 / Queue item
   * @param out 输出缓冲区 / Output buffer
   * @param cap 输出缓冲容量 / Output buffer capacity
   * @return std::size_t 实际写入长度（0 表示失败） / Bytes written (0 means failed)
   */
  std::size_t PackQueueItemToWire(const QueueItem& qi, uint8_t* out,
                                  std::size_t cap) const
  {
    if (cap < WIRE_HDR_SIZE)
    {
      return 0;
    }

    const uint8_t CH = qi.hdr.channel;
    const bool TS = (CH < can_count_)
                        ? timestamps_enabled_ch_[CH].load(std::memory_order_relaxed) != 0U
                        : false;
    const std::size_t PAYLOAD = qi.is_fd ? WIRE_FD_DATA_SIZE : WIRE_CLASSIC_DATA_SIZE;
    const std::size_t TOTAL = WIRE_HDR_SIZE + PAYLOAD + (TS ? WIRE_TS_SIZE : 0);

    if (TOTAL > cap)
    {
      return 0;
    }

    Memory::FastCopy(out, &qi.hdr, WIRE_HDR_SIZE);
    Memory::FastCopy(out + WIRE_HDR_SIZE, qi.data.data(), PAYLOAD);

    if (TS)
    {
      Memory::FastCopy(out + WIRE_HDR_SIZE + PAYLOAD, &qi.timestamp_us, WIRE_TS_SIZE);
    }

    return TOTAL;
  }

  /**
   * @brief 将 LibXR 错误帧转换为 Host 错误帧（SocketCAN 语义） /
   *        Convert LibXR error pack to host error frame (SocketCAN semantics)
   * @param ch 通道号 / Channel index
   * @param err_pack 错误帧 / Error frame
   * @param qi 输出队列元素 / Output queue item
   * @return true 转换成功并应上报 / Converted and should report
   * @return false 不上报 / Do not report
   */
  bool ErrorPackToHostErrorFrame(uint8_t ch, const LibXR::CAN::ClassicPack& err_pack,
                                 QueueItem& qi)
  {
    if (ch >= can_count_ || !cans_[ch])
    {
      return false;
    }
    if (!berr_enabled_[ch].load(std::memory_order_relaxed))
    {
      return false;
    }
    if (!LibXR::CAN::IsErrorId(err_pack.id))
    {
      return false;
    }

    // 与 Linux can/error.h 对齐（沿用之前版本的映射）。
    // Keep these definitions aligned with Linux can/error.h, matching the previous
    // mapping.
    constexpr uint8_t LNX_CAN_ERR_CRTL_UNSPEC = 0x00;
    constexpr uint8_t LNX_CAN_ERR_CRTL_RX_WARNING = 0x04;
    constexpr uint8_t LNX_CAN_ERR_CRTL_TX_WARNING = 0x08;
    constexpr uint8_t LNX_CAN_ERR_CRTL_RX_PASSIVE = 0x10;
    constexpr uint8_t LNX_CAN_ERR_CRTL_TX_PASSIVE = 0x20;

    constexpr uint8_t LNX_CAN_ERR_PROT_UNSPEC = 0x00;
    constexpr uint8_t LNX_CAN_ERR_PROT_FORM = 0x02;
    constexpr uint8_t LNX_CAN_ERR_PROT_STUFF = 0x04;
    constexpr uint8_t LNX_CAN_ERR_PROT_BIT0 = 0x08;
    constexpr uint8_t LNX_CAN_ERR_PROT_BIT1 = 0x10;
    constexpr uint8_t LNX_CAN_ERR_PROT_TX = 0x80;

    constexpr uint8_t LNX_CAN_ERR_PROT_LOC_UNSPEC = 0x00;
    constexpr uint8_t LNX_CAN_ERR_PROT_LOC_ACK = 0x19;

    constexpr uint32_t LNX_CAN_ERR_CNT = 0x00000200U;

    bool ec_valid = false;
    uint32_t txerr = 0, rxerr = 0;
    {
      LibXR::CAN::ErrorState es;
      if (cans_[ch]->GetErrorState(es) == ErrorCode::OK)
      {
        ec_valid = true;
        txerr = es.tx_error_counter;
        rxerr = es.rx_error_counter;
      }
    }

    const uint8_t TXERR_U8 = (txerr > 255U) ? 255U : static_cast<uint8_t>(txerr);
    const uint8_t RXERR_U8 = (rxerr > 255U) ? 255U : static_cast<uint8_t>(rxerr);

    const auto EID = LibXR::CAN::ToErrorID(err_pack.id);

    uint32_t cid = GsUsb::CAN_ERR_FLAG;
    std::array<uint8_t, 8> d{};
    switch (EID)
    {
      case LibXR::CAN::ErrorID::CAN_ERROR_ID_BUS_OFF:
        cid |= GsUsb::CAN_ERR_BUSOFF;
        break;

      case LibXR::CAN::ErrorID::CAN_ERROR_ID_ERROR_WARNING:
      case LibXR::CAN::ErrorID::CAN_ERROR_ID_ERROR_PASSIVE:
      {
        cid |= GsUsb::CAN_ERR_CRTL;
        uint8_t ctrl = LNX_CAN_ERR_CRTL_UNSPEC;

        if (ec_valid)
        {
          if (EID == LibXR::CAN::ErrorID::CAN_ERROR_ID_ERROR_PASSIVE)
          {
            if (txerr >= 128U)
            {
              ctrl |= LNX_CAN_ERR_CRTL_TX_PASSIVE;
            }
            if (rxerr >= 128U)
            {
              ctrl |= LNX_CAN_ERR_CRTL_RX_PASSIVE;
            }
          }
          else
          {
            if (txerr >= 96U)
            {
              ctrl |= LNX_CAN_ERR_CRTL_TX_WARNING;
            }
            if (rxerr >= 96U)
            {
              ctrl |= LNX_CAN_ERR_CRTL_RX_WARNING;
            }
          }
        }

        if (ctrl == LNX_CAN_ERR_CRTL_UNSPEC)
        {
          ctrl = (EID == LibXR::CAN::ErrorID::CAN_ERROR_ID_ERROR_PASSIVE)
                     ? static_cast<uint8_t>(LNX_CAN_ERR_CRTL_TX_PASSIVE |
                                            LNX_CAN_ERR_CRTL_RX_PASSIVE)
                     : static_cast<uint8_t>(LNX_CAN_ERR_CRTL_TX_WARNING |
                                            LNX_CAN_ERR_CRTL_RX_WARNING);
        }

        d[1] = ctrl;
        break;
      }

      case LibXR::CAN::ErrorID::CAN_ERROR_ID_ACK:
        cid |= GsUsb::CAN_ERR_ACK;
        cid |= GsUsb::CAN_ERR_PROT;
        d[2] = static_cast<uint8_t>(LNX_CAN_ERR_PROT_UNSPEC | LNX_CAN_ERR_PROT_TX);
        d[3] = LNX_CAN_ERR_PROT_LOC_ACK;
        break;

      case LibXR::CAN::ErrorID::CAN_ERROR_ID_STUFF:
        cid |= GsUsb::CAN_ERR_PROT;
        d[2] = static_cast<uint8_t>(LNX_CAN_ERR_PROT_STUFF | LNX_CAN_ERR_PROT_TX);
        d[3] = LNX_CAN_ERR_PROT_LOC_UNSPEC;
        break;

      case LibXR::CAN::ErrorID::CAN_ERROR_ID_FORM:
        cid |= GsUsb::CAN_ERR_PROT;
        d[2] = static_cast<uint8_t>(LNX_CAN_ERR_PROT_FORM | LNX_CAN_ERR_PROT_TX);
        d[3] = LNX_CAN_ERR_PROT_LOC_UNSPEC;
        break;

      case LibXR::CAN::ErrorID::CAN_ERROR_ID_BIT0:
        cid |= GsUsb::CAN_ERR_PROT;
        d[2] = static_cast<uint8_t>(LNX_CAN_ERR_PROT_BIT0 | LNX_CAN_ERR_PROT_TX);
        d[3] = LNX_CAN_ERR_PROT_LOC_UNSPEC;
        break;

      case LibXR::CAN::ErrorID::CAN_ERROR_ID_BIT1:
        cid |= GsUsb::CAN_ERR_PROT;
        d[2] = static_cast<uint8_t>(LNX_CAN_ERR_PROT_BIT1 | LNX_CAN_ERR_PROT_TX);
        d[3] = LNX_CAN_ERR_PROT_LOC_UNSPEC;
        break;

      default:
        cid |= GsUsb::CAN_ERR_PROT;
        d[2] = static_cast<uint8_t>(LNX_CAN_ERR_PROT_UNSPEC | LNX_CAN_ERR_PROT_TX);
        d[3] = LNX_CAN_ERR_PROT_LOC_UNSPEC;
        break;
    }

    cid |= LNX_CAN_ERR_CNT;
    d[6] = TXERR_U8;
    d[7] = RXERR_U8;

    qi.hdr.echo_id = ECHO_ID_RX;
    qi.hdr.can_id = cid;
    qi.hdr.can_dlc = GsUsb::CAN_ERR_DLC;
    qi.hdr.channel = ch;
    qi.hdr.flags = 0;
    qi.hdr.reserved = 0;

    qi.is_fd = false;
    qi.data_len = 8;
    qi.timestamp_us = MakeTimestampUs(ch);

    Memory::FastCopy(qi.data.data(), d.data(), 8);

    return true;
  }

  // ================= 工具函数 / Utilities =================

  /**
   * @brief 获取通道时间戳（us, 32-bit） / Get per-channel timestamp (us, 32-bit)
   * @param ch 通道号 / Channel index
   * @return uint32_t 时间戳 / Timestamp
   */
  uint32_t MakeTimestampUs(uint8_t ch) const
  {
    if (ch < can_count_ && timestamps_enabled_ch_[ch].load(std::memory_order_relaxed))
    {
      return static_cast<uint32_t>(LibXR::Timebase::GetMicroseconds() & 0xFFFFFFFFu);
    }
    return 0u;
  }

  /**
   * @brief 获取全局时间戳（us, 32-bit） / Get global timestamp (us, 32-bit)
   * @return uint32_t 时间戳 / Timestamp
   */
  uint32_t MakeTimestampUsGlobal() const
  {
    return static_cast<uint32_t>(LibXR::Timebase::GetMicroseconds() & 0xFFFFFFFFu);
  }

  /**
   * @brief DLC 转长度（FD 表） / DLC to length (FD table)
   */
  static uint8_t DlcToLen(uint8_t dlc)
  {
    static constexpr uint8_t TABLE[16] = {0, 1,  2,  3,  4,  5,  6,  7,
                                          8, 12, 16, 20, 24, 32, 48, 64};
    return (dlc < 16) ? TABLE[dlc] : 64;
  }

  /**
   * @brief 长度转 DLC（FD 表） / Length to DLC (FD table)
   */
  static uint8_t LenToDlc(uint8_t len)
  {
    if (len <= 8)
    {
      return len;
    }
    if (len <= 12)
    {
      return 9;
    }
    if (len <= 16)
    {
      return 10;
    }
    if (len <= 20)
    {
      return 11;
    }
    if (len <= 24)
    {
      return 12;
    }
    if (len <= 32)
    {
      return 13;
    }
    if (len <= 48)
    {
      return 14;
    }
    return 15;
  }
};

}  // namespace LibXR::USB
