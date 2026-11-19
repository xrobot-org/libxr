// NOLINTBEGIN(cppcoreguidelines-pro-type-cstyle-cast,performance-no-int-to-ptr)
#include <cstdint>
#include <cstring>

#include "ch32_usb_endpoint.hpp"
#include "timebase.hpp"

using namespace LibXR;

#if defined(USBFSD)

namespace
{

using Endpoint = USB::Endpoint;

volatile uint8_t* TxCtrl(Endpoint::EPNumber ep)
{
  return &USBFSD->UEP0_TX_CTRL + 4 * Endpoint::EPNumberToInt8(ep);
}

volatile uint8_t* RxCtrl(Endpoint::EPNumber ep)
{
  return &USBFSD->UEP0_RX_CTRL + 4 * Endpoint::EPNumberToInt8(ep);
}

void SetTxLen(Endpoint::EPNumber ep, size_t length)
{
  *(&USBFSD->UEP0_TX_LEN + 2 * Endpoint::EPNumberToInt8(ep)) =
      static_cast<uint16_t>(length);
}

volatile uint32_t* Dma(Endpoint::EPNumber ep)
{
  return &USBFSD->UEP0_DMA + Endpoint::EPNumberToInt8(ep);
}

/// 设 DMA 地址；硬件双缓冲不用 / Set the DMA address; the hardware double buffer is
/// not used
void SetDma(Endpoint::EPNumber ep, const void* value)
{
  *Dma(ep) = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(value));
  switch (ep)
  {
    case Endpoint::EPNumber::EP1:
      USBFSD->UEP4_1_MOD &= ~USBFS_UEP1_BUF_MOD;
      break;
    case Endpoint::EPNumber::EP2:
      USBFSD->UEP2_3_MOD &= ~USBFS_UEP2_BUF_MOD;
      break;
    case Endpoint::EPNumber::EP3:
      USBFSD->UEP2_3_MOD &= ~USBFS_UEP3_BUF_MOD;
      break;
    case Endpoint::EPNumber::EP4:
      USBFSD->UEP4_1_MOD &= ~USBFS_UEP4_BUF_MOD;
      break;
    case Endpoint::EPNumber::EP5:
      USBFSD->UEP5_6_MOD &= ~USBFS_UEP5_BUF_MOD;
      break;
    case Endpoint::EPNumber::EP6:
      USBFSD->UEP5_6_MOD &= ~USBFS_UEP6_BUF_MOD;
      break;
    case Endpoint::EPNumber::EP7:
      USBFSD->UEP7_MOD &= ~USBFS_UEP7_BUF_MOD;
      break;
    default:
      break;
  }
}

/// UEPn 的模式寄存器和位 / Mode register and bits of UEPn
struct ModeBits
{
  volatile uint8_t* reg;
  uint8_t tx_en;
  uint8_t rx_en;
};

ModeBits Mode(Endpoint::EPNumber ep)
{
  switch (ep)
  {
    case Endpoint::EPNumber::EP1:
      return {&USBFSD->UEP4_1_MOD, USBFS_UEP1_TX_EN, USBFS_UEP1_RX_EN};
    case Endpoint::EPNumber::EP2:
      return {&USBFSD->UEP2_3_MOD, USBFS_UEP2_TX_EN, USBFS_UEP2_RX_EN};
    case Endpoint::EPNumber::EP3:
      return {&USBFSD->UEP2_3_MOD, USBFS_UEP3_TX_EN, USBFS_UEP3_RX_EN};
    case Endpoint::EPNumber::EP4:
      return {&USBFSD->UEP4_1_MOD, USBFS_UEP4_TX_EN, USBFS_UEP4_RX_EN};
    case Endpoint::EPNumber::EP5:
      return {&USBFSD->UEP5_6_MOD, USBFS_UEP5_TX_EN, USBFS_UEP5_RX_EN};
    case Endpoint::EPNumber::EP6:
      return {&USBFSD->UEP5_6_MOD, USBFS_UEP6_TX_EN, USBFS_UEP6_RX_EN};
    case Endpoint::EPNumber::EP7:
      return {&USBFSD->UEP7_MOD, USBFS_UEP7_TX_EN, USBFS_UEP7_RX_EN};
    default:
      return {nullptr, 0U, 0U};
  }
}

/// EP0 没有使能位（总是使能）/ EP0 has no enable bits (always enabled)
void Enable(Endpoint::EPNumber ep, bool tx, bool on)
{
  const ModeBits mode = Mode(ep);
  if (mode.reg == nullptr)
  {
    return;
  }
  const uint8_t bit = tx ? mode.tx_en : mode.rx_en;
  *mode.reg = static_cast<uint8_t>(on ? (*mode.reg | bit) : (*mode.reg & ~bit));
}

/**
 * @brief SIE 此刻空闲，或总线正在复位 / The SIE is free now, or the bus is in reset
 * @note SIE_FREE 反映整个 SIE：别的端点正在进行的事务也让它为 0（实测）。总线复位期间
 *       SIE_FREE 一直为 0（CH32V208 实测），而复位中不会有事务，所以 MIS_ST.BUS_RESET
 *       也算停稳。
 *       SIE_FREE covers the whole SIE: a transaction of another endpoint in progress also
 *       reads 0 (measured). SIE_FREE stays 0 during a bus reset (measured on a CH32V208)
 *       and no transaction runs during a reset, so MIS_ST.BUS_RESET counts as quiescent
 *       too.
 */
bool SieQuiet()
{
  return (USBFSD->MIS_ST & (USBFS_UMS_SIE_FREE | USBFS_UMS_BUS_RESET)) != 0U;
}

}  // namespace

CH32EndpointOtgFs::CH32EndpointOtgFs(EPNumber ep_num, Direction dir, RawData buffer,
                                     uint8_t* shared_block)
    : Endpoint(ep_num, dir, buffer,
               (shared_block == nullptr && ep_num != EPNumber::EP0) ? 2U : 1U),
      single_direction_(shared_block == nullptr && ep_num != EPNumber::EP0),
      block_dma_base_(shared_block != nullptr ? shared_block
                                              : static_cast<uint8_t*>(buffer.addr_))
{
  map_[EPNumberToInt8(ep_num)][static_cast<uint8_t>(dir)] = this;
  // 共享块两个方向都把 UEPn_DMA 指向块基址。 / Both directions of a shared block point
  // UEPn_DMA at the block base.
  SetDma(ep_num, block_dma_base_);
  if (dir == Direction::IN)
  {
    SetTxLen(ep_num, 0U);
    *TxCtrl(ep_num) = USBFS_UEP_T_RES_NAK;
  }
  else
  {
    *RxCtrl(ep_num) = USBFS_UEP_R_RES_NAK;
  }
}

void CH32EndpointOtgFs::SetResponse(uint8_t res)
{
  if (GetDirection() == Direction::IN)
  {
    volatile uint8_t* ctrl = TxCtrl(GetNumber());
    *ctrl = static_cast<uint8_t>((*ctrl & ~USBFS_UEP_T_RES_MASK) | res);
  }
  else
  {
    volatile uint8_t* ctrl = RxCtrl(GetNumber());
    *ctrl = static_cast<uint8_t>((*ctrl & ~USBFS_UEP_R_RES_MASK) | res);
  }
}

void CH32EndpointOtgFs::ConfigureHardware(const Config& cfg)
{
  const EPNumber ep = GetNumber();
  if (!single_direction_ && ep != EPNumber::EP0)
  {
    // 共享块两个使能保持不变，各方向以 NAK 为闸门。 / A shared block keeps both enables
    // on; NAK gates each direction.
    Enable(ep, true, true);
    Enable(ep, false, true);
  }
  if (cfg.direction == Direction::IN)
  {
    SetTxLen(ep, 0U);
    // DATA0，自动翻转不开。 / DATA0, automatic toggle off.
    *TxCtrl(ep) = USBFS_UEP_T_RES_NAK;
    Enable(ep, true, true);
    if (single_direction_)
    {
      Enable(ep, false, false);
    }
  }
  else
  {
    *RxCtrl(ep) = USBFS_UEP_R_RES_NAK;
    Enable(ep, false, true);
    if (single_direction_)
    {
      Enable(ep, true, false);
    }
  }
  SetDma(ep, block_dma_base_);
  transfer_active_ = false;
  transfer_requested_ = 0U;
  transfer_accumulated_ = 0U;
  transfer_cursor_ = nullptr;
  transfer_remaining_ = 0U;
}

void CH32EndpointOtgFs::CloseHardware(bool in_isr)
{
  UNUSED(in_isr);
  const bool is_in = GetDirection() == Direction::IN;
  if (is_in)
  {
    *TxCtrl(GetNumber()) = USBFS_UEP_T_RES_NAK;
  }
  else
  {
    *RxCtrl(GetNumber()) = USBFS_UEP_R_RES_NAK;
  }
  if (single_direction_)
  {
    Enable(GetNumber(), is_in, false);
  }
  transfer_active_ = false;
}

ErrorCode CH32EndpointOtgFs::StartHardware(const HwRequest& request, bool in_isr)
{
  UNUSED(in_isr);
  const EPNumber ep = GetNumber();
  const bool is_ep0 = ep == EPNumber::EP0;
  const bool iso = GetType() == Type::ISOCHRONOUS;

  transfer_requested_ = request.length;
  transfer_accumulated_ = 0U;
  transfer_cursor_ = request.data;
  transfer_remaining_ = 0U;
  transfer_active_ = true;

  // 共享块不能动 UEPn_DMA：硬件的 IN 区固定在基址 +64，改了会把发送区推到 +128，OUT
  // 也会收进 IN 区。共享端点的数据由 CPU 在槽和这块缓冲之间逐包拷贝。
  // A shared block must not move UEPn_DMA: the hardware IN area is fixed at base +64,
  // moving it would transmit from +128 and receive OUT into the IN area. A shared
  // endpoint's data is copied by the CPU, packet by packet, between the slot and that
  // buffer.
  const bool shared = !single_direction_ && !is_ep0;
  if (!shared)
  {
    SetDma(ep, request.data);
  }

  if (GetDirection() == Direction::IN)
  {
    const size_t mps = MaxPacketSize();
    const size_t chunk = request.length > mps ? mps : request.length;
    if (shared && chunk != 0U)
    {
      std::memcpy(block_dma_base_ + SHARED_BLOCK_HALF, request.data, chunk);
    }
    transfer_remaining_ = request.length - chunk;
    transfer_cursor_ += chunk;
    SetTxLen(ep, chunk);
    volatile uint8_t* ctrl = TxCtrl(ep);
    if (!is_ep0)
    {
      *ctrl = static_cast<uint8_t>((iso ? USBFS_UEP_T_RES_NONE : USBFS_UEP_T_RES_ACK) |
                                   (*ctrl & ~USBFS_UEP_T_RES_MASK));
    }
    else
    {
      // EP0 IN 每次启动翻一次 toggle。 / EP0 IN flips the toggle at every start.
      *ctrl = static_cast<uint8_t>(USBFS_UEP_T_RES_ACK | (tog_ ? USBFS_UEP_T_TOG : 0U));
      tog_ = !tog_;
    }
    return ErrorCode::OK;
  }

  volatile uint8_t* ctrl = RxCtrl(ep);
  if (!is_ep0)
  {
    *ctrl = static_cast<uint8_t>((iso ? USBFS_UEP_R_RES_NONE : USBFS_UEP_R_RES_ACK) |
                                 (*ctrl & ~USBFS_UEP_R_RES_MASK));
  }
  else
  {
    // EP0 OUT：数据阶段按 tog_ 交替（SETUP 后 DATA1），状态阶段（长度 0）固定 DATA1。
    // EP0 OUT: the data stage alternates per tog_ (DATA1 after SETUP); the status stage
    // (length 0) is always DATA1.
    const bool data1 = request.length == 0U || tog_;
    *ctrl = static_cast<uint8_t>(USBFS_UEP_R_RES_ACK | (data1 ? USBFS_UEP_R_TOG : 0U));
  }
  return ErrorCode::OK;
}

Endpoint::StopOutcome CH32EndpointOtgFs::StopHardware(bool in_isr)
{
  UNUSED(in_isr);
  // 停止失败或控制器复位之后，硬件上已没有这个请求。
  // After a failed stop or a controller reset the request is no longer on the hardware.
  if (!transfer_active_)
  {
    return {ErrorCode::ABORTED, 0U};
  }
  const EPNumber ep = GetNumber();
  const bool is_in = GetDirection() == Direction::IN;
  const bool is_ep0 = ep == EPNumber::EP0;

  // 1) NAK 从下一个事务起生效；写入时已在线上的事务照常完成。
  // 1) NAK applies from the next transaction on; one already on the wire completes.
  if (!stopping_)
  {
    SetResponse(is_in ? USBFS_UEP_T_RES_NAK : USBFS_UEP_R_RES_NAK);
  }

  // 2) 线上还有事务就不等：停止未结束。处理 SETUP 期间控制器对所有令牌回 NAK，控制端点
  //    当场停稳。
  // 2) A transaction on the wire is not waited for: the stop is pending. During SETUP
  //    handling the controller NAKs every token, so the control endpoint is quiescent at
  //    once.
  if (!(is_ep0 && setup_held_) && !SieQuiet())
  {
    BeginPending();
    return {ErrorCode::PENDING, 0U};
  }

  // 3) 收下锁存的本端点完成（令牌 SETUP 不算）；别的端点的留给事件循环，并挂起本线。
  // 3) Collect a latched completion of this endpoint (a SETUP token never is); another
  //    endpoint's stays for the event loop, and the line is set pending.
  bool mine = false;
  uint8_t int_st = 0U;
  uint16_t rx_len = 0U;
  if (!transfer_held_ && (USBFSD->INT_FG & USBFS_UIF_TRANSFER) != 0U)
  {
    int_st = USBFSD->INT_ST;
    const uint8_t token = is_in ? USBFS_UIS_TOKEN_IN : USBFS_UIS_TOKEN_OUT;
    if ((int_st & USBFS_UIS_ENDP_MASK) == EPNumberToInt8(ep) &&
        (int_st & USBFS_UIS_TOKEN_MASK) == token)
    {
      mine = (int_st & USBFS_UIS_IS_NAK) == 0U;
      rx_len = USBFSD->RX_LEN;
      USBFSD->INT_FG = USBFS_UIF_TRANSFER;
      // 这个事件不再经过事件循环：主机 ACK 过的包在这里推进 toggle。
      // This event no longer goes through the event loop: a packet the host ACKed
      // advances the toggle here.
      if (mine && (is_in || (int_st & USBFS_UIS_TOG_OK) != 0U))
      {
        AdvanceToggle();
      }
    }
    else
    {
      NVIC_SetPendingIRQ(USBFS_IRQn);
    }
  }
  const StopOutcome outcome =
      SettleStop({mine, rx_len, (int_st & USBFS_UIS_TOG_OK) != 0U});
  EndPending();
  transfer_active_ = false;
  return outcome;
}

Endpoint::StopOutcome CH32EndpointOtgFs::SettleStop(const StopEvent& event)
{
  const bool mine = event.mine;
  const size_t size = event.size;
  const bool tog_ok = event.tog_ok;
  const bool is_ep0 = GetNumber() == EPNumber::EP0;
  const bool iso = GetType() == Type::ISOCHRONOUS;
  const size_t mps = MaxPacketSize();
  if (GetDirection() == Direction::IN)
  {
    size_t chunk = mps;
    if (transfer_remaining_ == 0U)
    {
      chunk = transfer_requested_ == 0U
                  ? 0U
                  : transfer_requested_ - (transfer_requested_ - 1U) / mps * mps;
    }
    size_t confirmed = transfer_requested_ - transfer_remaining_ - chunk;
    if (mine)
    {
      confirmed += chunk;
      if (transfer_remaining_ == 0U)
      {
        return {ErrorCode::OK, confirmed};
      }
    }
    return {ErrorCode::ABORTED, confirmed};
  }

  // OUT：本端点的完成事件是已应答的整包（实测），必须计入；TOG_OK=0 是主机重发的旧包。
  // 只计字节数：被停的 OUT 请求以 ABORTED 交付、不借出槽，共享端点的这一包不拷进槽。
  // OUT: a completion event of this endpoint is a whole acknowledged packet (measured)
  // and must be counted; TOG_OK=0 is a resend of an old packet. Only the bytes are
  // counted: a stopped OUT request is delivered ABORTED and lends no slot, so a shared
  // endpoint's packet is not copied into the slot.
  bool done = false;
  if (mine && (iso || tog_ok))
  {
    if (is_ep0)
    {
      tog_ = !tog_;
    }
    const size_t room = transfer_requested_ - transfer_accumulated_;
    const size_t taken = size > room ? room : size;
    transfer_accumulated_ += taken;
    done = transfer_accumulated_ >= transfer_requested_ || size < mps ||
           !single_direction_ || is_ep0 || iso;
  }
  return {done ? ErrorCode::OK : ErrorCode::ABORTED, transfer_accumulated_};
}

void CH32EndpointOtgFs::BeginPending()
{
  if (stopping_)
  {
    return;
  }
  stopping_ = true;
  // NAK 已写下。此刻已经锁存的 SOF（本轮快照里的，或快照之后到的）早于 NAK，不能证明
  // 停稳：标志留着（它还要报告给设备类），处理到它时本停止跳过一次。恰好这时到来的
  // SOF 被算进去也只是多等一帧。
  // The NAK is written. A SOF latched by now (in this round's snapshot or arrived after
  // it) precedes the NAK and proves nothing: its flag stays (it is still reported to the
  // device classes) and this stop skips it once when it is handled. A SOF arriving right
  // now and counted as latched only costs one more frame.
  sof_before_stop_ = (USBFSD->INT_FG & USBFS_UIF_HST_SOF) != 0U;
  ++pending_stops_;
}

void CH32EndpointOtgFs::EndPending()
{
  if (!stopping_)
  {
    return;
  }
  stopping_ = false;
  --pending_stops_;
}

void CH32EndpointOtgFs::FinishStop(bool in_isr, const StopEvent& event)
{
  const StopOutcome outcome = SettleStop(event);
  EndPending();
  transfer_active_ = false;
  OnHardwareComplete(in_isr, outcome.actual, outcome.result);
}

void CH32EndpointOtgFs::FinishQuietStops(bool in_isr, bool force, bool only_ep0)
{
  if (pending_stops_ == 0U)
  {
    return;
  }
  // 还有传输事件没处理时不判断：它可能就是被停端点的完成，由事件循环交给 FinishStop。
  // Nothing is decided while a transfer event waits: it may be the stopped endpoint's
  // completion, which the event loop hands to FinishStop.
  if (!force && (!SieQuiet() || (USBFSD->INT_FG & USBFS_UIF_TRANSFER) != 0U))
  {
    return;
  }
  for (uint8_t n = 0U; n < (only_ep0 ? 1U : EP_OTG_FS_MAX_SIZE); ++n)
  {
    for (CH32EndpointOtgFs* ep : map_[n])
    {
      if (ep != nullptr && ep->stopping_)
      {
        ep->FinishStop(in_isr, {});
      }
    }
  }
}

void CH32EndpointOtgFs::FinishStopsAtSof(bool in_isr)
{
  for (auto& pair : map_)
  {
    for (CH32EndpointOtgFs* ep : pair)
    {
      if (ep == nullptr || !ep->stopping_)
      {
        continue;
      }
      if (ep->sof_before_stop_)
      {
        ep->sof_before_stop_ = false;
        continue;
      }
      ep->FinishStop(in_isr, {});
    }
  }
}

ErrorCode CH32EndpointOtgFs::StallHardware(bool in_isr)
{
  UNUSED(in_isr);
  SetResponse(GetDirection() == Direction::IN ? USBFS_UEP_T_RES_STALL
                                              : USBFS_UEP_R_RES_STALL);
  return ErrorCode::OK;
}

ErrorCode CH32EndpointOtgFs::ClearStallHardware(bool in_isr)
{
  UNUSED(in_isr);
  if (GetNumber() == EPNumber::EP0)
  {
    // 下一个 SETUP 已把 EP0 恢复为 NAK/DATA1；再复位 toggle 会让数据阶段首包变成
    // DATA0。 / The next SETUP has already restored EP0 to NAK/DATA1; resetting the
    // toggle again would send the first data packet as DATA0.
    return ErrorCode::OK;
  }
  // 翻转位回到 DATA0（USB 2.0 9.4.5）。RES 只在 STALL 时改成 NAK：端点没有 STALL 时硬件
  // 上可能挂着请求（主机对没有 STALL 的端点也会清 HALT，例如 libusb_clear_halt），它的
  // ACK 原样留着，请求继续等主机。调用者是处理 SETUP 的那一轮（全速控制器一轮一个
  // 令牌），INT_BUSY 让控制器对其他事务回 NAK，所以这次读改写不会撞上本端点的事务；主机
  // 只看到它会重试的 NAK。
  // The toggle returns to DATA0 (USB 2.0 9.4.5). RES becomes NAK only when it is STALL:
  // an endpoint that is not halted may have a request on the hardware (a host clears the
  // halt of an endpoint that is not halted too, e.g. libusb_clear_halt); its ACK stays
  // and the request goes on waiting for the host. The caller is the round that handles
  // the SETUP (the full-speed controller has one token per round), and INT_BUSY makes
  // the controller NAK other transactions, so this read-modify-write cannot meet a
  // transaction of this endpoint; the host sees only NAKs, which it retries.
  if (GetDirection() == Direction::IN)
  {
    volatile uint8_t* ctrl = TxCtrl(GetNumber());
    uint8_t res = *ctrl & USBFS_UEP_T_RES_MASK;
    if (res == USBFS_UEP_T_RES_STALL)
    {
      res = USBFS_UEP_T_RES_NAK;
    }
    *ctrl =
        static_cast<uint8_t>((*ctrl & ~(USBFS_UEP_T_RES_MASK | USBFS_UEP_T_TOG)) | res);
  }
  else
  {
    volatile uint8_t* ctrl = RxCtrl(GetNumber());
    uint8_t res = *ctrl & USBFS_UEP_R_RES_MASK;
    if (res == USBFS_UEP_R_RES_STALL)
    {
      res = USBFS_UEP_R_RES_NAK;
    }
    *ctrl =
        static_cast<uint8_t>((*ctrl & ~(USBFS_UEP_R_RES_MASK | USBFS_UEP_R_TOG)) | res);
  }
  return ErrorCode::OK;
}

Endpoint::HwLimits CH32EndpointOtgFs::HardwareLimits(Direction direction, Type type) const
{
  // 手册表 23-4（p.359）：只有 EP3 单向等时可超过 64 字节（1023），其余最大 64；EP0
  // 64（p.355）。控制和等时一次请求一包；共享块 OUT 槽只有 64 字节、每包完成，也是一包；
  // 其余由软件逐包推进，包数不限。
  // Manual table 23-4 (p.359): only a single-direction isochronous EP3 goes above 64
  // bytes (1023), everything else is 64 at most; EP0 is 64 (p.355). Control and
  // isochronous carry one packet per request; a shared OUT slot has 64 bytes and
  // completes per packet, so one packet too; the rest advances packet by packet in
  // software, with no packet limit.
  // 单向端点和 EP0 由硬件直接读写槽；共享端点的槽只由 CPU 拷贝访问。
  // The hardware reads and writes the slot of a single-direction endpoint and of EP0; a
  // shared endpoint's slot is accessed by the CPU copy only.
  const bool direct = single_direction_ || GetNumber() == EPNumber::EP0;
  if (type == Type::CONTROL)
  {
    return {64U, 1U, SIZE_MAX, direct};
  }
  if (type == Type::ISOCHRONOUS)
  {
    const bool big = GetNumber() == EPNumber::EP3 && single_direction_;
    return {big ? 1023U : 64U, 1U, SIZE_MAX, direct};
  }
  if (!single_direction_ && direction == Direction::OUT)
  {
    return {64U, 1U, SIZE_MAX, direct};
  }
  return {64U, SIZE_MAX, SIZE_MAX, direct};
}

void CH32EndpointOtgFs::AdvanceToggle()
{
  // 等时固定 DATA0。EP0 的相位由 StartHardware 每次整字节写入，这里翻的位不起作用。
  // Isochronous stays DATA0. EP0's phase is written whole by every StartHardware, so the
  // bit flipped here has no effect on it.
  if (GetType() == Type::ISOCHRONOUS)
  {
    return;
  }
  if (GetDirection() == Direction::IN)
  {
    volatile uint8_t* ctrl = TxCtrl(GetNumber());
    *ctrl = static_cast<uint8_t>(*ctrl ^ USBFS_UEP_T_TOG);
  }
  else
  {
    volatile uint8_t* ctrl = RxCtrl(GetNumber());
    *ctrl = static_cast<uint8_t>(*ctrl ^ USBFS_UEP_R_TOG);
  }
}

void CH32EndpointOtgFs::TransferComplete(bool in_isr, size_t size, bool tog_ok)
{
  // 发送完成事件是主机 ACK 过的包，TOG_OK 的接收事件是硬件应答过的新包：主机已推进它的
  // 相位，软件跟着翻，没有请求在途时也翻。TOG_OK=0 是主机重发的旧包，不翻。
  // A transmit completion event is a packet the host ACKed, and a receive event with
  // TOG_OK is a new packet the hardware acknowledged: the host has advanced its phase and
  // software follows, also with no request in flight. TOG_OK=0 is a resend of an old
  // packet and does not flip.
  if (GetDirection() == Direction::IN || tog_ok)
  {
    AdvanceToggle();
  }
  if (!transfer_active_)
  {
    return;
  }
  if (stopping_)
  {
    // 停止未结束时到来的本端点完成：线上的那个事务结束了，停止随之结束。
    // A completion of this endpoint while its stop is pending: the transaction on the
    // wire has ended, and so does the stop.
    FinishStop(in_isr, {true, size, tog_ok});
    return;
  }
  const EPNumber ep = GetNumber();
  const bool is_ep0 = ep == EPNumber::EP0;
  const bool iso = GetType() == Type::ISOCHRONOUS;

  if (GetDirection() == Direction::IN)
  {
    if (transfer_remaining_ > 0U)
    {
      const size_t mps = MaxPacketSize();
      const size_t chunk = transfer_remaining_ > mps ? mps : transfer_remaining_;
      if (single_direction_ || is_ep0)
      {
        SetDma(ep, transfer_cursor_);
      }
      else
      {
        // 共享块的 IN 区固定在基址 +64、只有一个 64 字节槽：包长小于 64 时请求可有多包，
        // 下一包先挪到 IN 区开头。
        // A shared block's IN area is fixed at base +64 with one 64-byte slot: with a
        // packet below 64 a request can span packets, and the next one moves to the
        // start of the IN area first.
        std::memmove(block_dma_base_ + SHARED_BLOCK_HALF, transfer_cursor_, chunk);
      }
      SetTxLen(ep, chunk);
      transfer_cursor_ += chunk;
      transfer_remaining_ -= chunk;
      SetResponse(iso ? USBFS_UEP_T_RES_NONE : USBFS_UEP_T_RES_ACK);
      return;
    }
    SetResponse(USBFS_UEP_T_RES_NAK);
    transfer_active_ = false;
    OnHardwareComplete(in_isr, transfer_requested_);
    return;
  }

  if (!iso && !tog_ok)
  {
    // toggle 不符：主机重发了已收下的包。丢掉它，原样重新挂接收。
    // Toggle mismatch: the host resent a packet already taken. Drop it and arm the
    // receive again as it was.
    SetResponse(USBFS_UEP_R_RES_ACK);
    return;
  }

  if (!is_ep0 && single_direction_ && !iso)
  {
    // 单向：DMA 逐包推进，收满或短包时整个请求完成一次。
    // Single direction: the DMA advances per packet and the request completes once when
    // full or on a short packet.
    transfer_accumulated_ += size;
    if (transfer_accumulated_ < transfer_requested_ && size == MaxPacketSize())
    {
      transfer_cursor_ += size;
      *Dma(ep) = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(transfer_cursor_));
      return;
    }
    SetResponse(USBFS_UEP_R_RES_NAK);
    transfer_active_ = false;
    OnHardwareComplete(in_isr, transfer_accumulated_);
    return;
  }

  // 共享块、EP0 和等时：每包完成。回到 NAK：标志清掉后 INT_BUSY 不再挡事务，下一个接收
  // 要等核心来挂，其间到达的包会被硬件应答而没人收。
  // Shared block, EP0 and isochronous: one completion per packet. Back to NAK: once the
  // flag is cleared INT_BUSY holds nothing off, and the next receive waits for the core;
  // a packet arriving in between would be acknowledged and taken by nobody.
  SetResponse(USBFS_UEP_R_RES_NAK);
  if (!single_direction_ && !is_ep0)
  {
    // 共享端点：包在后端的缓冲里，拷进槽。
    // Shared endpoint: the packet is in the backend's buffer and is copied into the slot.
    size = size > transfer_requested_ ? transfer_requested_ : size;
    if (size != 0U)
    {
      std::memcpy(transfer_cursor_, block_dma_base_, size);
    }
  }
  transfer_accumulated_ = size;
  transfer_active_ = false;
  if (is_ep0)
  {
    // 数据阶段的包 DATA1/DATA0 交替，完成一次翻一次；状态阶段由 StartHardware 定 DATA1。
    // Data-stage packets alternate DATA1/DATA0, one flip per completion; StartHardware
    // forces DATA1 for the status stage.
    tog_ = !tog_;
  }
  OnHardwareComplete(in_isr, size);
}

#endif  // defined(USBFSD)

// NOLINTEND(cppcoreguidelines-pro-type-cstyle-cast,performance-no-int-to-ptr)
