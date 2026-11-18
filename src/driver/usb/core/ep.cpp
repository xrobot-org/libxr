#include "ep.hpp"

#include <algorithm>

#include "usb_service.hpp"

namespace LibXR::USB
{

namespace
{

/// packet_ 中标记控制类型的位，包长在低 16 位 / Bit of packet_ marking the control type;
/// the packet size is in the low 16 bits
constexpr uint32_t CONTROL_TYPE_BIT = 1U << 16U;

/// 构造参数给出的槽数 / Slot count resolved from the constructor argument
uint8_t SlotCount(Endpoint::EPNumber number, uint8_t slot_count)
{
  if (slot_count != 0U)
  {
    return slot_count;
  }
  return number == Endpoint::EPNumber::EP0 ? 1U : 2U;
}

/// 每个槽的字节数，缓存行整数倍 / Bytes per slot, a cache-line multiple
size_t SlotSize(size_t memory_size, uint8_t slot_count)
{
  return memory_size / slot_count / HW_CACHE_LINE_SIZE * HW_CACHE_LINE_SIZE;
}

/// 配置的方向是否可用 / Whether the configured direction is available
bool DirectionFits(Endpoint::Direction avail, Endpoint::Direction direction)
{
  return direction != Endpoint::Direction::BOTH &&
         (avail == Endpoint::Direction::BOTH || avail == direction);
}

/**
 * @brief 类型在速度下的最大包长 / Largest packet of a type at a speed
 * @note 0 表示该速度下不能使用该类型（USB 2.0 5.5.3、5.6.3、5.7.3、5.8.3）。
 *       0 when the type cannot be used at that speed (USB 2.0 5.5.3, 5.6.3, 5.7.3,
 *       5.8.3).
 */
size_t SpeedLimit(Endpoint::Type type, Speed speed)
{
  const bool low = speed == Speed::LOW;
  const bool full = speed == Speed::FULL;
  const bool high = speed == Speed::HIGH;
  switch (type)
  {
    case Endpoint::Type::CONTROL:
      return low ? 8U : ((full || high) ? 64U : 0U);
    case Endpoint::Type::BULK:
      return full ? 64U : (high ? 512U : 0U);
    case Endpoint::Type::ISOCHRONOUS:
      return full ? 1023U : (high ? 1024U : 0U);
    case Endpoint::Type::INTR:
      return low ? 8U : (full ? 64U : (high ? 1024U : 0U));
  }
  return 0U;
}

/// 包长对类型和速度是否合法 / Whether a packet size is legal for the type and speed
bool IsLegalPacket(Endpoint::Type type, Speed speed, size_t packet)
{
  const size_t limit = SpeedLimit(type, speed);
  if (packet == 0U || packet > limit)
  {
    return false;
  }
  if (type == Endpoint::Type::CONTROL || type == Endpoint::Type::BULK)
  {
    // 控制和批量只有固定几个值：全速 8、16、32、64，其余速度只有上限本身。
    // Control and bulk allow a few values only: 8, 16, 32 or 64 at full speed, the
    // limit itself at the other speeds.
    return packet == limit ||
           (speed == Speed::FULL && (packet == 8U || packet == 16U || packet == 32U));
  }
  return true;
}

/// 不超过 room 的最大合法包长，0 表示没有 / Largest legal size up to room, 0 if none
size_t LargestLegal(Endpoint::Type type, Speed speed, size_t room)
{
  size_t packet = std::min(room, SpeedLimit(type, speed));
  while (packet != 0U && !IsLegalPacket(type, speed, packet))
  {
    --packet;
  }
  return packet;
}

/**
 * @brief 某个速度下一个服务区间最多几个包 / Most packets in one service interval at a
 *        speed
 * @note 只有高速的等时和中断端点可以多于一个（USB 2.0 5.9）。这里不按类型区分：高速的
 *       批量和控制端点包长是 512 和 64，达不到 SmallestPacketFor 对两个包要求的 513。
 *       Only high-speed isochronous and interrupt endpoints may have more than one (USB
 *       2.0 5.9). The type is not looked at here: high-speed bulk and control endpoints
 *       have packets of 512 and 64 bytes, below the 513 SmallestPacketFor asks for two
 *       packets.
 */
size_t IntervalLimit(Speed speed) { return speed == Speed::HIGH ? 3U : 1U; }

/// 一个服务区间 packets 个包时允许的最小包长（USB 2.0 表 9-14）/ Smallest packet size
/// allowed with packets packets in one service interval (USB 2.0 Table 9-14)
size_t SmallestPacketFor(size_t packets)
{
  return packets >= 3U ? 683U : (packets == 2U ? 513U : 1U);
}

/// 不超过 room 的最大合法包数；room 为 0 时得 0 / Largest legal count up to room; 0
/// when room is 0
uint8_t LargestInterval(Speed speed, size_t packet, size_t room)
{
  size_t packets = std::min(room, IntervalLimit(speed));
  while (packets > 1U && packet < SmallestPacketFor(packets))
  {
    --packets;
  }
  return static_cast<uint8_t>(packets);
}

/// 后端一次请求最多的字节数 / Bytes the backend takes in one request
size_t RequestLimit(size_t max_packets, size_t max_bytes, size_t packet)
{
  const size_t by_packets =
      (max_packets > SIZE_MAX / packet) ? SIZE_MAX : max_packets * packet;
  return std::min(by_packets, max_bytes);
}

/// StopHardware 可返回的结果 / Outcomes StopHardware may return
bool IsStopOutcome(ErrorCode result)
{
  return result == ErrorCode::OK || result == ErrorCode::ABORTED ||
         result == ErrorCode::FAILED || result == ErrorCode::OUT_OF_RANGE ||
         result == ErrorCode::PENDING;
}

/// OnHardwareComplete 可收到的结果 / Results OnHardwareComplete may receive
bool IsReportResult(ErrorCode result)
{
  return result == ErrorCode::OK || result == ErrorCode::FAILED ||
         result == ErrorCode::OUT_OF_RANGE || result == ErrorCode::ABORTED ||
         result == ErrorCode::TIMEOUT;
}

}  // namespace

Endpoint::Endpoint(EPNumber number, Direction avail, RawData slots, uint8_t slot_count)
    : number_(number),
      avail_direction_(avail),
      memory_(slots),
      slot_size_(SlotSize(slots.size_, SlotCount(number, slot_count))),
      slot_states_(SlotCount(number, slot_count))
{
  ASSERT(number < EPNumber::EP_MAX_NUM);
  ASSERT(slot_count <= MAX_SLOTS);
  ASSERT(slots.addr_ != nullptr);
  ASSERT(reinterpret_cast<uintptr_t>(slots.addr_) % HW_CACHE_LINE_SIZE == 0U);
  ASSERT(slots.size_ % HW_CACHE_LINE_SIZE == 0U);
  ASSERT(slot_size_ != 0U);
}

ErrorCode Endpoint::AcquireSlot(Slot& slot)
{
  EndpointSlots::Lend lend;
  const ErrorCode result = slot_states_.LendForSending(lend);
  if (result != ErrorCode::OK)
  {
    return result;
  }
  slot.data = SlotData(lend.index, capacity_.load(std::memory_order_relaxed));
  slot.index_ = lend.index;
  slot.stamp_ = lend.stamp;
  return ErrorCode::OK;
}

ErrorCode Endpoint::Submit(const Slot& slot, size_t length, bool zlp, bool in_isr)
{
  if (length > MaxTransferSize())
  {
    return ErrorCode::SIZE_ERR;
  }
  const ErrorCode result =
      slot_states_.Queue({slot.index_, slot.stamp_}, {length, zlp}, true);
  if (result == ErrorCode::OK)
  {
    NotifyOwner(in_isr);
  }
  return result;
}

ErrorCode Endpoint::ReleaseSlot(const Slot& slot)
{
  return slot_states_.Release({slot.index_, slot.stamp_});
}

ErrorCode Endpoint::Receive(size_t bytes, bool in_isr)
{
  EndpointSlots::Lend lend;
  ErrorCode result = slot_states_.ClaimForReceiving(lend);
  if (result != ErrorCode::OK)
  {
    return result;
  }
  // 认领成功之后才读随绑定变化的量；之后被停用则 Queue 返回 ABORTED。
  // What changes with the binding is read only after the claim succeeded; a later
  // disable makes Queue return ABORTED.
  const uint32_t word = packet_.load(std::memory_order_relaxed);
  const size_t packet = static_cast<uint16_t>(word);
  const bool control = (word & CONTROL_TYPE_BIT) != 0U;
  const bool legal = bytes <= capacity_.load(std::memory_order_relaxed) &&
                     (bytes == 0U ? control : bytes % packet == 0U);
  if (!legal)
  {
    (void)slot_states_.Release(lend);
    return ErrorCode::SIZE_ERR;
  }
  result = slot_states_.Queue(lend, {bytes, false}, false);
  if (result == ErrorCode::ABORTED)
  {
    // 认领之后端点被关闭，槽已空闲。
    // The endpoint was closed after the claim; the slot is free again.
    return ErrorCode::STATE_ERR;
  }
  if (result == ErrorCode::OK)
  {
    NotifyOwner(in_isr);
  }
  return result;
}

ErrorCode Endpoint::Configure(const Config& cfg, bool in_isr)
{
  UNUSED(in_isr);
  ASSERT(pool_ != nullptr);
  // 停止失败的端点等控制器复位，之前不能再绑定。
  // An endpoint whose stop failed waits for the controller reset and cannot bind
  // before it.
  if (GetState() != State::DISABLED || stop_ == Stop::FAILED)
  {
    return ErrorCode::STATE_ERR;
  }
  const Speed speed = pool_->GetSpeed();
  if (!DirectionFits(avail_direction_, cfg.direction) ||
      cfg.max_packets_per_interval == 0U ||
      LargestLegal(cfg.type, speed, cfg.max_packet_size) == 0U)
  {
    return ErrorCode::ARG_ERR;
  }
  const uint16_t packet = PacketSizeFor(cfg, speed);
  if (packet == 0U)
  {
    return ErrorCode::NO_BUFF;
  }
  const uint8_t interval = PacketsPerIntervalFor(cfg, speed);
  const HwLimits hw = HardwareLimits(cfg.direction, cfg.type);

  Config chosen = cfg;
  chosen.max_packet_size = packet;
  chosen.max_packets_per_interval = interval;
  // 能否配置在这里全部判定完；之后的 ConfigureHardware 不会失败，推迟到停止结束再做也
  // 不影响返回值。
  // Whether the configuration is possible is decided in full here; the later
  // ConfigureHardware cannot fail, so deferring it to the end of a stop leaves the
  // return value final.
  const ErrorCode check = CheckHardware(chosen);
  if (check != ErrorCode::OK)
  {
    return check;
  }
  if (stop_ == Stop::NONE)
  {
    ConfigureHardware(chosen);
    hw_config_ = chosen;
  }
  else
  {
    // 上一次 Close 的停止未结束：CloseHardware 和 ConfigureHardware 在停止结束时依次
    // 调用。
    // The stop of the last Close has not ended: CloseHardware and ConfigureHardware run
    // in this order when it ends.
    ASSERT((deferred_ & DEFER_CLOSE) != 0U);
  }

  const size_t limit = RequestLimit(hw.max_packets, hw.max_bytes, packet);
  const bool control = cfg.type == Type::CONTROL;
  ASSERT(hw.max_requests >= 1U);
  max_requests_ = control ? uint8_t{1U} : hw.max_requests;
  // 等时类型的一个请求是一个服务区间；包数选定时已保证它放得进槽和后端的上限。
  // One request of the isochronous type is one service interval; the choice of the count
  // made sure that it fits the slot and the backend limit.
  size_t capacity = std::min(slot_size_, limit) / packet * packet;
  if (control)
  {
    capacity = packet;
  }
  else if (cfg.type == Type::ISOCHRONOUS)
  {
    capacity = static_cast<size_t>(interval) * packet;
  }
  config_ = chosen;
  capacity_.store(static_cast<uint32_t>(capacity), std::memory_order_relaxed);
  packet_.store(packet | (control ? CONTROL_TYPE_BIT : 0U) |
                    (static_cast<uint32_t>(interval) << INTERVAL_PACKETS_SHIFT),
                std::memory_order_relaxed);
  // 启用记账的比较交换以 release 发布上面两个值；借出或认领成功之后再读它们。
  // The compare-and-swap that enables the bookkeeping publishes both values with
  // release; they are read after a lend or claim succeeded.
  slot_states_.Enable(cfg.direction == Direction::IN);
  SetState(State::IDLE);
  return ErrorCode::OK;
}

void Endpoint::Close(bool in_isr)
{
  if (GetState() == State::DISABLED)
  {
    return;
  }
  SetState(State::DISABLED);
  EndRequests(in_isr, Ending::CLOSE);
}

void Endpoint::Flush(bool in_isr)
{
  if (GetState() == State::DISABLED)
  {
    return;
  }
  EndRequests(in_isr, Ending::FLUSH);
}

ErrorCode Endpoint::Stall(bool in_isr)
{
  const State state = GetState();
  if (state == State::DISABLED)
  {
    return ErrorCode::STATE_ERR;
  }
  if (state == State::STALLED)
  {
    return ErrorCode::OK;
  }
  EndRequests(in_isr, Ending::STALL);
  return ErrorCode::OK;
}

ErrorCode Endpoint::ClearStall(bool in_isr)
{
  if (GetState() == State::DISABLED)
  {
    return ErrorCode::STATE_ERR;
  }
  if (stop_ != Stop::NONE)
  {
    // 停止未结束：状态当场回到 IDLE，ClearStallHardware 推迟到停止结束。
    // A stop is pending: the state is IDLE at once and ClearStallHardware waits for the
    // end of the stop.
    deferred_ |= DEFER_CLEAR_STALL;
    SetState(State::IDLE);
    return ErrorCode::OK;
  }
  const ErrorCode result = ClearStallHardware(in_isr);
  if (result != ErrorCode::OK)
  {
    return result;
  }
  SetState(State::IDLE);
  NotifyOwner(in_isr);
  return ErrorCode::OK;
}

void Endpoint::OnHardwareComplete(bool in_isr, size_t actual, ErrorCode result)
{
  ASSERT(IsReportResult(result));
  // 报告 TIMEOUT 之后再报告是后端违约，丢弃。
  // A report after TIMEOUT breaks the backend contract and is dropped.
  ASSERT(stop_ != Stop::FAILED);
  if (stop_ == Stop::FAILED)
  {
    return;
  }
  if (stop_ == Stop::PENDING)
  {
    if (result == ErrorCode::TIMEOUT)
    {
      // 停止失败：被停的槽和推迟的动作都留给控制器复位之后的 CloseAfterReset。
      // The stop failed: the stopped slot and the deferred steps are left to
      // CloseAfterReset after the controller reset.
      stop_ = Stop::FAILED;
      pool_->StopFailed();
      return;
    }
    // 请求已在停止时回调过；这里只放仍留着的槽。被停时硬件上有几个请求，后端就按先后
    // 报告几次，最后一次才是停止结束，那时再执行推迟的动作。
    // The requests were called back at the stop; only a slot still held is freed here.
    // The backend reports once for each request that was on the hardware at the stop, in
    // order, and only the last report ends the stop and runs the deferred steps.
    if (inflight_ != NO_REQUEST)
    {
      const uint8_t index = inflight_;
      inflight_ = PopAhead();
      slot_states_.FinishFree(index);
    }
    ASSERT(stop_reports_ != 0U);
    if (--stop_reports_ != 0U)
    {
      return;
    }
    stop_ = Stop::NONE;
    closing_ = Closing::NONE;
    ApplyDeferred(in_isr);
    pool_->StopEnded();
    return;
  }
  if (inflight_ == NO_REQUEST || inflight_failed_)
  {
    return;
  }
  ASSERT(result != ErrorCode::TIMEOUT);
  if (in_backend_)
  {
    ASSERT(!held_);
    held_ = true;
    held_actual_ = actual;
    held_result_ = result;
    return;
  }
  HandleReport(in_isr, result, actual);
}

void Endpoint::StartNext(bool in_isr)
{
  while (GetState() == State::IDLE && stop_ == Stop::NONE &&
         pool_->CanStart(number_ == EPNumber::EP0))
  {
    if (inflight_ != NO_REQUEST)
    {
      if (closing_ == Closing::DUE)
      {
        closing_ = Closing::ON_WIRE;
        StartInflight(in_isr, 0U);
        continue;
      }
      // 硬件还挂得下，而且硬件上的请求都不欠结尾零长度包（零长度包要紧跟它的数据）、
      // 也没有没能进硬件的：把下一个排队的请求提前交下去。
      // The hardware can hold another request, none of those on it owes a closing
      // zero-length packet (that packet has to follow its data) and none failed to
      // reach it: the next queued request is handed down ahead.
      if (ahead_count_ + 1U >= max_requests_ || closing_ != Closing::NONE ||
          ahead_failed_)
      {
        return;
      }
      if (ahead_count_ != 0U && AheadAt(static_cast<uint8_t>(ahead_count_ - 1U)).owed)
      {
        return;
      }
      const EndpointSlots::Queued queued = slot_states_.TakeNext();
      if (queued.index == EndpointSlots::NO_SLOT)
      {
        return;
      }
      ASSERT(ahead_count_ < sizeof(ahead_));
      ahead_[ahead_count_++] = queued.index;
      StartAhead(in_isr);
      continue;
    }
    const EndpointSlots::Queued next = slot_states_.TakeNext();
    if (next.index == EndpointSlots::NO_SLOT)
    {
      return;
    }
    const size_t length = next.request.length;
    inflight_ = next.index;
    data_length_ = length;
    const bool owed = next.request.zlp && length != 0U && length % MaxPacketSize() == 0U;
    closing_ = owed ? Closing::OWED : Closing::NONE;
    StartInflight(in_isr, length);
  }
}

void Endpoint::StartInflight(bool in_isr, size_t length)
{
  const HwRequest request{static_cast<uint8_t*>(SlotData(inflight_, length).addr_),
                          length};
  in_backend_ = true;
  const ErrorCode result = StartHardware(request, in_isr);
  in_backend_ = false;
  if (result != ErrorCode::OK)
  {
    ASSERT(!held_);
    FinishInflight(in_isr, ErrorCode::FAILED,
                   closing_ == Closing::ON_WIRE ? data_length_ : 0U);
    return;
  }
  if (held_)
  {
    held_ = false;
    HandleReport(in_isr, held_result_, held_actual_);
  }
}

Endpoint::Ahead Endpoint::AheadAt(uint8_t position) const
{
  ASSERT(position < ahead_count_);
  const uint8_t index = ahead_[position];
  const EndpointSlots::Request request = slot_states_.RequestOf(index);
  const size_t length = request.length;
  const bool owed = request.zlp && length != 0U && length % MaxPacketSize() == 0U;
  return {index, length, owed, ahead_failed_ && position + 1U == ahead_count_};
}

uint8_t Endpoint::PopAhead()
{
  if (ahead_count_ == 0U)
  {
    return NO_REQUEST;
  }
  const uint8_t index = ahead_[0];
  --ahead_count_;
  for (uint8_t i = 0U; i < ahead_count_; ++i)
  {
    ahead_[i] = ahead_[i + 1U];
  }
  if (ahead_count_ == 0U)
  {
    ahead_failed_ = false;
  }
  return index;
}

void Endpoint::FreeAhead(bool keep_started)
{
  if (keep_started)
  {
    if (ahead_failed_)
    {
      slot_states_.FinishFree(ahead_[--ahead_count_]);
      ahead_failed_ = false;
    }
    return;
  }
  for (uint8_t i = 0U; i < ahead_count_; ++i)
  {
    slot_states_.FinishFree(ahead_[i]);
  }
  ahead_count_ = 0U;
  ahead_failed_ = false;
}

void Endpoint::StartAhead(bool in_isr)
{
  const Ahead ahead = AheadAt(static_cast<uint8_t>(ahead_count_ - 1U));
  const HwRequest request{
      static_cast<uint8_t*>(SlotData(ahead.index, ahead.length).addr_), ahead.length};
  in_backend_ = true;
  const ErrorCode result = StartHardware(request, in_isr);
  in_backend_ = false;
  if (result != ErrorCode::OK)
  {
    // 没有进硬件。它的 FAILED 回调排在更早的请求的回调之后，见 FinishInflight。
    // It did not reach the hardware. Its FAILED callback comes after the callbacks of
    // the earlier requests, see FinishInflight.
    ahead_failed_ = true;
  }
  // 硬件上有更早的请求时后端不在 StartHardware 之内报告；留存到的只能是更早那个请求
  // 的完成。
  // With an earlier request on the hardware the backend does not report from inside
  // StartHardware; a report held here can only be the earlier request's completion.
  ASSERT(!held_);
  if (held_)
  {
    held_ = false;
    HandleReport(in_isr, held_result_, held_actual_);
  }
}

void Endpoint::HandleReport(bool in_isr, ErrorCode result, size_t actual)
{
  if (closing_ == Closing::OWED && result == ErrorCode::OK)
  {
    // 结尾零长度包留给端点阶段启动，这里不启动任何东西。
    // The closing packet is left to the endpoint step; nothing starts here.
    closing_ = Closing::DUE;
  }
  else
  {
    FinishInflight(in_isr, result, closing_ == Closing::ON_WIRE ? data_length_ : actual);
  }
  NotifyOwner(in_isr);
}

void Endpoint::FinishInflight(bool in_isr, ErrorCode result, size_t actual)
{
  const uint8_t index = inflight_;
  // 提前交下去的请求里最早的那个顶上来成为在途请求；先改好状态再回调。
  // The earliest of the requests handed down ahead moves up and becomes the one in
  // flight; the state is settled before the callback.
  const Ahead next = ahead_count_ != 0U ? AheadAt(0U) : Ahead{};
  (void)PopAhead();
  inflight_ = next.index;
  data_length_ = next.length;
  closing_ = next.owed ? Closing::OWED : Closing::NONE;
  inflight_failed_ = next.failed;
  const bool out = config_.direction == Direction::OUT;
  if (!out || result != ErrorCode::OK)
  {
    slot_states_.FinishFree(index);
    Deliver(in_isr, result, actual, Slot{});
  }
  else
  {
    Slot slot;
    slot.stamp_ = slot_states_.FinishLent(index);
    slot.index_ = index;
    slot.data = SlotData(index, actual);
    Deliver(in_isr, result, actual, slot);
  }
  // 顶上来的请求当初没能进硬件：现在以 FAILED 结束，排在上面那次回调之后。上面的回调
  // 可能已经用 Close、Flush 或 Stall 结束了它，所以重新看状态。
  // The request that moved up never reached the hardware: it ends with FAILED now,
  // after the callback above. That callback may already have ended it with Close, Flush
  // or Stall, so the state is read again.
  if (inflight_failed_ && inflight_ != NO_REQUEST && stop_ == Stop::NONE)
  {
    const uint8_t failed = inflight_;
    inflight_ = NO_REQUEST;
    inflight_failed_ = false;
    closing_ = Closing::NONE;
    slot_states_.FinishFree(failed);
    Deliver(in_isr, ErrorCode::FAILED, 0U, Slot{});
  }
}

void Endpoint::EndRequests(bool in_isr, Ending ending)
{
  // 先做完端点自己的事（移出队列、停止、硬件操作或记下推迟），最后才回调：回调可以调用
  // 本端点的生命周期函数，看到的总是已收尾的状态。
  // The endpoint's own work (dequeue, stop, hardware step or recording it as deferred)
  // comes first and the callbacks last: a callback may call this endpoint's lifecycle
  // functions and always sees the finished state.
  const uint8_t dropped =
      (ending == Ending::CLOSE) ? slot_states_.Disable() : slot_states_.DropQueued();
  if (ending == Ending::CLOSE)
  {
    deferred_ &= static_cast<uint8_t>(~DEFER_CLEAR_STALL);
  }
  else if (ending == Ending::STALL)
  {
    SetState(State::STALLED);
  }

  // 已有停止未结束时在途的槽属于那个停止，不再停。
  // With a stop already pending the in-flight slot belongs to it and is not stopped
  // again.
  const bool stopping = inflight_ != NO_REQUEST && stop_ == Stop::NONE;
  // 提前交下去的请求，和在途请求一起结束。
  // The requests handed down ahead end together with the one in flight.
  Ahead aheads[MAX_SLOTS - 1U];
  StopOutcome stopped_ahead[MAX_SLOTS - 1U];
  const uint8_t ahead_count = stopping ? ahead_count_ : uint8_t{0U};
  for (uint8_t i = 0U; i < ahead_count; ++i)
  {
    aheads[i] = AheadAt(i);
    stopped_ahead[i] = {ErrorCode::ABORTED, 0U};
  }
  StopOutcome stopped{ErrorCode::ABORTED, 0U};
  bool pending = false;
  if (stopping)
  {
    if (inflight_failed_)
    {
      // 没能进硬件的请求：硬件上没有东西。 / A request that never reached the
      // hardware: nothing is on the hardware.
      stopped = {ErrorCode::FAILED, 0U};
    }
    else if (closing_ == Closing::DUE)
    {
      // 数据部分已完成、零长度包未启动：硬件上没有东西。
      // Data part done, closing packet not started: nothing is on the hardware.
      stopped = {ErrorCode::ABORTED, data_length_};
    }
    else
    {
      stopped = StopHardware(in_isr);
      ASSERT(IsStopOutcome(stopped.result));
      pending = stopped.result == ErrorCode::PENDING;
    }
    for (uint8_t i = 0U; i < ahead_count; ++i)
    {
      if (aheads[i].failed)
      {
        stopped_ahead[i] = {ErrorCode::FAILED, 0U};
      }
      else if (!pending)
      {
        // 第一次调用已经停掉了硬件上的全部请求，之后每次只取下一个的结果。
        // The first call stopped every request on the hardware; each further one only
        // fetches the outcome of the next.
        stopped_ahead[i] = StopHardware(in_isr);
        ASSERT(IsStopOutcome(stopped_ahead[i].result) &&
               stopped_ahead[i].result != ErrorCode::PENDING);
      }
    }
  }
  if (pending)
  {
    stop_ = Stop::PENDING;
    // 后端为硬件上的每个请求报告一次；没能进硬件的那个不算。
    // The backend reports once for each request on the hardware; the one that did not
    // reach it does not count.
    const bool last_failed = ahead_count != 0U && aheads[ahead_count - 1U].failed;
    stop_reports_ = static_cast<uint8_t>(1U + ahead_count - (last_failed ? 1U : 0U));
    const bool direct = HardwareLimits(config_.direction, config_.type).direct_memory;
    if (!direct)
    {
      // 硬件不直接访问槽：后端从此不再读写它们，当场放回。
      // The hardware never accesses the slots itself: the backend no longer touches
      // them, so they are given back at once.
      const uint8_t index = inflight_;
      inflight_ = NO_REQUEST;
      slot_states_.FinishFree(index);
    }
    FreeAhead(direct);
  }
  if (stop_ == Stop::NONE)
  {
    if (ending == Ending::CLOSE)
    {
      CloseHardware(in_isr);
    }
    else if (ending == Ending::STALL)
    {
      (void)StallHardware(in_isr);
    }
    if (stopping)
    {
      const uint8_t index = inflight_;
      inflight_ = NO_REQUEST;
      slot_states_.FinishFree(index);
      FreeAhead(false);
    }
  }
  else if (ending == Ending::CLOSE)
  {
    deferred_ |= DEFER_CLOSE;
  }

  if (stopping)
  {
    // 被停的 OUT 和还欠结尾零长度包的 IN 即使硬件报 OK 也没有完成。
    // A stopped OUT, and an IN still owing its closing packet, is not complete even
    // when the hardware says OK.
    const bool out = config_.direction == Direction::OUT;
    const bool incomplete = out || closing_ == Closing::OWED || closing_ == Closing::DUE;
    ErrorCode result = pending ? ErrorCode::ABORTED : stopped.result;
    if (result == ErrorCode::OK && incomplete)
    {
      result = ErrorCode::ABORTED;
    }
    const bool data_done = closing_ == Closing::ON_WIRE || closing_ == Closing::DUE;
    const size_t actual = data_done ? data_length_ : (pending ? 0U : stopped.actual);
    closing_ = Closing::NONE;
    inflight_failed_ = false;
    Deliver(in_isr, result, actual, Slot{});
    for (uint8_t i = 0U; i < ahead_count; ++i)
    {
      // 它的数据做完了而结尾零长度包还没发，同样算没有完成。
      // With its data done and its closing packet not sent it is not complete either.
      const Ahead& ahead = aheads[i];
      const bool ahead_pending = pending && !ahead.failed;
      ErrorCode ahead_result =
          ahead_pending ? ErrorCode::ABORTED : stopped_ahead[i].result;
      size_t ahead_actual = ahead_pending ? 0U : stopped_ahead[i].actual;
      if (ahead_result == ErrorCode::OK && (out || ahead.owed))
      {
        ahead_result = ErrorCode::ABORTED;
        ahead_actual = out ? ahead_actual : ahead.length;
      }
      Deliver(in_isr, ahead_result, ahead_actual, Slot{});
    }
  }
  for (uint8_t i = 0U; i < dropped; ++i)
  {
    Deliver(in_isr, ErrorCode::ABORTED, 0U, Slot{});
  }
}

void Endpoint::ApplyDeferred(bool in_isr)
{
  // 先清零再动作：动作和其后的回调看到的都是 deferred_ 为 0。
  // Cleared before acting: the steps and any later callback see deferred_ as 0.
  const uint8_t deferred = deferred_;
  deferred_ = 0U;
  if ((deferred & DEFER_CLOSE) != 0U)
  {
    CloseHardware(in_isr);
    if (GetState() != State::DISABLED)
    {
      ConfigureHardware(config_);
      hw_config_ = config_;
    }
  }
  const State state = GetState();
  if (state == State::STALLED)
  {
    (void)StallHardware(in_isr);
  }
  else if (state == State::IDLE && (deferred & DEFER_CLEAR_STALL) != 0U)
  {
    (void)ClearStallHardware(in_isr);
  }
  NotifyOwner(in_isr);
}

void Endpoint::CloseAfterReset(bool in_isr)
{
  if (stop_ != Stop::NONE)
  {
    // 复位之后这次停止立即返回；被停的请求早已回调过。
    // After the reset this stop returns at once; the stopped request was called back
    // long ago.
    const StopOutcome outcome = StopHardware(in_isr);
    ASSERT(outcome.result == ErrorCode::ABORTED);
    UNUSED(outcome);
    if (inflight_ != NO_REQUEST)
    {
      const uint8_t index = inflight_;
      inflight_ = NO_REQUEST;
      slot_states_.FinishFree(index);
    }
    FreeAhead(false);
    stop_ = Stop::NONE;
    stop_reports_ = 0U;
    deferred_ = 0U;
    closing_ = Closing::NONE;
    if (GetState() == State::DISABLED)
    {
      CloseHardware(in_isr);
      return;
    }
  }
  Close(in_isr);
}

void Endpoint::Deliver(bool in_isr, ErrorCode result, size_t actual, const Slot& slot)
{
  on_complete_.Run(in_isr, Completion{result, actual, slot});
}

uint16_t Endpoint::PacketSizeFor(const Config& cfg, Speed speed) const
{
  if (!DirectionFits(avail_direction_, cfg.direction))
  {
    return 0U;
  }
  const HwLimits hw = HardwareLimits(cfg.direction, cfg.type);
  ASSERT(hw.max_packets >= 1U && hw.max_bytes >= hw.max_packet_size);
  // 硬件直接读写槽时，被停的槽留到停止结束：非控制端点只有一个槽就会无槽可用。
  // Where the hardware accesses the slot directly the stopped slot is kept until the
  // stop ends, so a non-control endpoint with one slot would have none left.
  const bool one_slot = slot_size_ == memory_.size_;
  if (hw.direct_memory && one_slot && cfg.type != Type::CONTROL)
  {
    return 0U;
  }
  const size_t room = std::min(
      {static_cast<size_t>(cfg.max_packet_size), slot_size_, hw.max_packet_size});
  return static_cast<uint16_t>(LargestLegal(cfg.type, speed, room));
}

uint8_t Endpoint::PacketsPerIntervalFor(const Config& cfg, Speed speed) const
{
  const size_t packet = PacketSizeFor(cfg, speed);
  if (packet == 0U)
  {
    return 0U;
  }
  const HwLimits hw = HardwareLimits(cfg.direction, cfg.type);
  ASSERT(hw.max_packets_per_interval >= 1U);
  // 包数上限为 0 时 room 为 0，结果也是 0。
  // A count bound of 0 makes room 0, and the result 0 as well.
  size_t room =
      std::min<size_t>(cfg.max_packets_per_interval, hw.max_packets_per_interval);
  if (cfg.type == Type::ISOCHRONOUS)
  {
    // 等时类型的一个请求是一个服务区间：整个区间放进一个槽，且不超过后端一次请求的上限。
    // One request of the isochronous type is one service interval: the whole interval
    // fits one slot and stays within the backend's limit for one request.
    room = std::min({room, slot_size_ / packet, hw.max_packets, hw.max_bytes / packet});
  }
  return LargestInterval(speed, packet, room);
}

void Endpoint::NotifyOwner(bool in_isr) { pool_->RequestEndpoint(pending_bit_, in_isr); }

RawData Endpoint::SlotData(uint8_t index, size_t size) const
{
  return {static_cast<uint8_t*>(memory_.addr_) + index * slot_size_, size};
}

}  // namespace LibXR::USB
