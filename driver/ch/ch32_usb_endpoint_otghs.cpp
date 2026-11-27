// NOLINTBEGIN(cppcoreguidelines-pro-type-cstyle-cast,performance-no-int-to-ptr)
#include <cstdint>

#include "ch32_usb_endpoint.hpp"
#include "timebase.hpp"

using namespace LibXR;

#if LIBXR_CH32_HAS_USBHS

namespace
{

using Endpoint = USB::Endpoint;

constexpr uint8_t IN_IDX = static_cast<uint8_t>(Endpoint::Direction::IN);
constexpr uint8_t OUT_IDX = static_cast<uint8_t>(Endpoint::Direction::OUT);

/**
 * @brief 写下 NAK 之后经过这么久，停止按时间结束（µs）/ Time after the NAK at which a
 *        stop ends by time (µs)
 * @note 写 NAK 时已在线上的事务照常完成；线上最长的一包是全速 1023 字节的等时包，
 *       约 0.8 ms（STM32H723 实测最长 807.6 µs）。约 1-3% 的停止打断的是以 NAK 结束
 *       的令牌，之后没有任何事件（CH32V307 实测），只能按时间结束。
 *       A transaction on the wire at the NAK completes as usual; the longest packet on
 *       the wire is a full-speed isochronous packet of 1023 bytes, about 0.8 ms (807.6 µs
 *       at most measured on an STM32H723). About 1-3 % of the stops cut into a token
 *       that ends with NAK and are followed by no event at all (measured on a CH32V307);
 *       they can only end by time.
 */
constexpr uint64_t STOP_AGE_US = 1000U;

static_assert(static_cast<uint32_t>(USBHS_UEP15_T_EN) ==
                  (static_cast<uint32_t>(USBHS_UEP0_T_EN) << 15U),
              "T_EN bits are 1 << n");
static_assert(static_cast<uint32_t>(USBHS_UEP15_R_EN) ==
                  (static_cast<uint32_t>(USBHS_UEP0_R_EN) << 15U),
              "R_EN bits are 1 << n");
static_assert(offsetof(USBHSD_TypeDef, UEP1_MAX_LEN) -
                      offsetof(USBHSD_TypeDef, UEP0_MAX_LEN) ==
                  4U,
              "UEPn_MAX_LEN stride");

volatile uint8_t* TxCtrl(Endpoint::EPNumber ep)
{
  return &USBHSD->UEP0_TX_CTRL + 4 * Endpoint::EPNumberToInt8(ep);
}

volatile uint8_t* RxCtrl(Endpoint::EPNumber ep)
{
  return &USBHSD->UEP0_RX_CTRL + 4 * Endpoint::EPNumberToInt8(ep);
}

void SetTxLen(Endpoint::EPNumber ep, size_t length)
{
  *(&USBHSD->UEP0_TX_LEN + 2 * Endpoint::EPNumberToInt8(ep)) =
      static_cast<uint16_t>(length);
}

void SetRxMaxLen(Endpoint::EPNumber ep, uint16_t length)
{
  *(&USBHSD->UEP0_MAX_LEN + 2 * Endpoint::EPNumberToInt8(ep)) = length;
}

volatile uint32_t* TxDma(Endpoint::EPNumber ep)
{
  return ep == Endpoint::EPNumber::EP0
             ? &USBHSD->UEP0_DMA
             : &USBHSD->UEP1_TX_DMA + (Endpoint::EPNumberToInt8(ep) - 1);
}

volatile uint32_t* RxDma(Endpoint::EPNumber ep)
{
  return ep == Endpoint::EPNumber::EP0
             ? &USBHSD->UEP0_DMA
             : &USBHSD->UEP1_RX_DMA + (Endpoint::EPNumberToInt8(ep) - 1);
}

uint32_t Address(const void* pointer)
{
  return static_cast<uint32_t>(reinterpret_cast<uintptr_t>(pointer));
}

/// 设 DMA 地址；BUF_MOD 恒 0，两个槽由软件切换 / Set the DMA address; BUF_MOD stays
/// 0, software switches between the two slots
void SetDma(volatile uint32_t* dma, Endpoint::EPNumber ep, const void* buffer)
{
  *dma = Address(buffer);
  USBHSD->BUF_MODE &= ~(1U << Endpoint::EPNumberToInt8(ep));
}

void EnableTx(Endpoint::EPNumber ep, bool on)
{
  const uint32_t bit = USBHS_UEP0_T_EN << Endpoint::EPNumberToInt8(ep);
  USBHSD->ENDP_CONFIG = on ? (USBHSD->ENDP_CONFIG | bit) : (USBHSD->ENDP_CONFIG & ~bit);
}

void EnableRx(Endpoint::EPNumber ep, bool on)
{
  const uint32_t bit = USBHS_UEP0_R_EN << Endpoint::EPNumberToInt8(ep);
  USBHSD->ENDP_CONFIG = on ? (USBHSD->ENDP_CONFIG | bit) : (USBHSD->ENDP_CONFIG & ~bit);
}

/**
 * @brief SIE 此刻空闲，或总线正在复位 / The SIE is free now, or the bus is in reset
 * @note SIE_FREE 反映整个 SIE：别的端点正在进行的事务也让它为 0（实测）。总线复位期间
 *       SIE_FREE 一直为 0（板上实测），而复位中不会有事务，所以 MIS_ST.BUS_RESET 也算
 *       停稳。
 *       SIE_FREE covers the whole SIE: a transaction of another endpoint in progress also
 *       reads 0 (measured). SIE_FREE stays 0 during a bus reset (measured on the board)
 *       and no transaction runs during a reset, so MIS_ST.BUS_RESET counts as quiescent
 *       too.
 */
bool SieQuiet()
{
  return (USBHSD->MIS_ST & (USBHS_UMS_SIE_FREE | USBHS_UMS_BUS_RESET)) != 0U;
}

}  // namespace

CH32EndpointOtgHs::CH32EndpointOtgHs(EPNumber ep_num, Direction dir, RawData buffer)
    : Endpoint(ep_num, dir, buffer), dma_base_(buffer.addr_)
{
  map_[EPNumberToInt8(ep_num)][static_cast<uint8_t>(dir)] = this;
}

void CH32EndpointOtgHs::InitHardware()
{
  for (uint8_t i = 0U; i < EP_OTG_HS_MAX_SIZE; ++i)
  {
    const auto ep_num = static_cast<EPNumber>(i);
    for (uint8_t d = 0U; d < 2U; ++d)
    {
      CH32EndpointOtgHs* ep = map_[i][d];
      if (ep == nullptr)
      {
        continue;
      }
      if (static_cast<Direction>(d) == Direction::IN)
      {
        SetDma(TxDma(ep_num), ep_num, ep->dma_base_);
        SetTxLen(ep_num, 0U);
        *TxCtrl(ep_num) = USBHS_UEP_T_RES_NAK;
      }
      else
      {
        SetDma(RxDma(ep_num), ep_num, ep->dma_base_);
        *RxCtrl(ep_num) = USBHS_UEP_R_RES_NAK;
      }
    }
  }
}

void CH32EndpointOtgHs::SetResponse(uint8_t res)
{
  if (GetDirection() == Direction::IN)
  {
    volatile uint8_t* ctrl = TxCtrl(GetNumber());
    *ctrl = static_cast<uint8_t>((*ctrl & ~USBHS_UEP_T_RES_MASK) | res);
  }
  else
  {
    volatile uint8_t* ctrl = RxCtrl(GetNumber());
    *ctrl = static_cast<uint8_t>((*ctrl & ~USBHS_UEP_R_RES_MASK) | res);
  }
}

void CH32EndpointOtgHs::ConfigureHardware(const Config& cfg)
{
  const EPNumber ep = GetNumber();
  const uint8_t n = EPNumberToInt8(ep);
  const bool iso = cfg.type == Type::ISOCHRONOUS;
  const bool auto_tog = !iso && ep != EPNumber::EP0;

  if (cfg.direction == Direction::IN)
  {
    *TxCtrl(ep) = USBHS_UEP_T_RES_NAK | (auto_tog ? USBHS_UEP_T_TOG_AUTO : 0U);
    SetTxLen(ep, 0U);
    USBHSD->ENDP_TYPE = iso ? (USBHSD->ENDP_TYPE | (USBHS_UEP0_T_TYPE << n))
                            : (USBHSD->ENDP_TYPE & ~(USBHS_UEP0_T_TYPE << n));
    iso_in_ = iso;
    iso_free_pass_ = 0U;
    if (iso)
    {
      IdleIsoIn();
    }
    // 发送使能从配置开到关闭，其间不动：等时事务不受 INT_BUSY 约束，在主机取包的那一刻
    // 改 ENDP_CONFIG 会让 SIE 停在一次发不完的发送上，见类说明。
    // Transmit stays enabled from configuration to close and is not touched in between:
    // isochronous transactions are not held off by INT_BUSY, and changing ENDP_CONFIG at
    // the moment the host takes a packet leaves the SIE in a transmission that never
    // ends, see the class notes.
    EnableTx(ep, true);
    if (map_[n][OUT_IDX] == nullptr)
    {
      EnableRx(ep, false);
    }
    SetDma(TxDma(ep), ep, SlotMemory().addr_);
  }
  else
  {
    // OUT 不开 R_TOG_AUTO：SOF 以端点号 = 帧号 bit10:7 的传输上报，控制器会在这些 SOF
    // 上翻转该号 OUT 的 RX toggle（板上实测）；软件在每个 TOG_OK 包后翻转。
    // OUT runs without R_TOG_AUTO: a SOF is reported as a transfer to endpoint number =
    // frame bits 10:7 and the controller flips that OUT's RX toggle on those SOFs
    // (measured on the board); software flips it after every TOG_OK packet.
    *RxCtrl(ep) = USBHS_UEP_R_RES_NAK;
    SetRxMaxLen(ep, cfg.max_packet_size);
    USBHSD->ENDP_TYPE = iso ? (USBHSD->ENDP_TYPE | (USBHS_UEP0_R_TYPE << n))
                            : (USBHSD->ENDP_TYPE & ~(USBHS_UEP0_R_TYPE << n));
    EnableRx(ep, true);
    if (map_[n][IN_IDX] == nullptr)
    {
      EnableTx(ep, false);
    }
    SetDma(RxDma(ep), ep, SlotMemory().addr_);
  }

  transfer_active_ = false;
  transfer_requested_ = 0U;
  transfer_accumulated_ = 0U;
  transfer_cursor_ = nullptr;
  transfer_remaining_ = 0U;
  tx_tog_armed_ = 0U;
  tog_window_ = 0U;
}

void CH32EndpointOtgHs::CloseHardware(bool in_isr)
{
  UNUSED(in_isr);
  if (GetDirection() == Direction::IN)
  {
    *TxCtrl(GetNumber()) = USBHS_UEP_T_RES_NAK;
    EnableTx(GetNumber(), false);
    iso_in_ = false;
    iso_loaded_ = false;
  }
  else
  {
    *RxCtrl(GetNumber()) = USBHS_UEP_R_RES_NAK;
    EnableRx(GetNumber(), false);
  }
  transfer_active_ = false;
}

ErrorCode CH32EndpointOtgHs::StartHardware(const HwRequest& request, bool in_isr)
{
  UNUSED(in_isr);
  const EPNumber ep = GetNumber();
  const bool is_in = GetDirection() == Direction::IN;
  const bool is_ep0 = ep == EPNumber::EP0;
  const bool iso = GetType() == Type::ISOCHRONOUS;

  // UEPn_TX_LEN、TX_CTRL、RX_CTRL 在同一个 32 位字里。挂 OUT 接收、写 RX_CTRL 时 IN 恰好
  // 完成一个包，控制器对 IN toggle 的自动推进会丢，下一个 IN 用旧相位发出，主机当作重复包
  // 丢掉（深流水下发生；去掉补推，15 x 10 万条命令出错 29 次，保留 0 次）。写之前
  // UIF_TRANSFER 未置位、写之后已置位，说明期间完成了事务：记下 IN 写前的 toggle，由
  // RepairLostToggle 在下一个好包时核对。先读 toggle 再读 UIF_TRANSFER：两次读之间完成的
  // 事务已让 UIF_TRANSFER 置位，不记。
  // UEPn_TX_LEN, TX_CTRL and RX_CTRL share one 32-bit word. If an IN packet completes
  // while the writes below arm an OUT receive, the controller loses its automatic
  // advance of the IN toggle: the next IN goes out with the old phase and the host drops
  // it as a repeat (seen at pipeline depth 4; without the repair 15 runs of 100000
  // commands gave 29 errors, with it 0). UIF_TRANSFER clear before the writes and set
  // after them means a transaction completed meanwhile: the IN toggle from before is
  // kept and RepairLostToggle checks it at the next good packet. The toggle is read
  // first, then UIF_TRANSFER: a transaction completing in between has already set
  // UIF_TRANSFER and is not recorded.
  CH32EndpointOtgHs* other = nullptr;
  uint8_t other_tog = 0U;
  if (!is_ep0 && !is_in && !iso)
  {
    other = map_[EPNumberToInt8(ep)][IN_IDX];
    if (other != nullptr && other->GetState() != State::DISABLED &&
        other->GetType() != Type::ISOCHRONOUS)
    {
      other_tog = *TxCtrl(ep) & USBHS_UEP_T_TOG_MASK;
    }
    else
    {
      other = nullptr;
    }
  }
  const bool idle = (USBHSD->INT_FG & USBHS_UIF_TRANSFER) == 0U;

  transfer_requested_ = request.length;
  transfer_accumulated_ = 0U;
  transfer_cursor_ = request.data;
  transfer_remaining_ = 0U;
  transfer_active_ = true;

  if (is_in && iso)
  {
    // 等时 IN 的寄存器只在本端点刚发完一包之后写（见 OnIsoInSent）。这一轮里它刚发完
    // 一包、之后还没有再发，就现在装入；否则等它下一次发出（没有包时发的是零长度包）。
    // The registers of an isochronous IN are written only right after a packet of this
    // endpoint has gone out (see OnIsoInSent). If one went out in this pass and none
    // since, the packet is loaded now; otherwise at its next transmission (a zero-length
    // packet when nothing is loaded).
    iso_loaded_ = false;
    if (iso_free_pass_ == pass_ && !IsoInSent())
    {
      LoadIsoIn();
    }
    return ErrorCode::OK;
  }

  if (is_in)
  {
    // 槽就是 DMA 缓冲；超过一个包的请求按包长分包，余下由完成路径续发。
    // The slot is the DMA buffer; a request longer than one packet is split at the
    // packet size and the completion path sends the rest.
    SetDma(TxDma(ep), ep, request.data);
    const size_t mps = MaxPacketSize();
    const size_t chunk = request.length > mps ? mps : request.length;
    transfer_remaining_ = request.length - chunk;
    transfer_cursor_ += chunk;
    SetTxLen(ep, chunk);
    volatile uint8_t* ctrl = TxCtrl(ep);
    if (!is_ep0)
    {
      *ctrl = static_cast<uint8_t>((*ctrl & ~USBHS_UEP_T_RES_MASK) | USBHS_UEP_T_RES_ACK);
      tx_tog_armed_ = *ctrl & USBHS_UEP_T_TOG_MASK;
    }
    else
    {
      // EP0 IN 每次启动翻一次 toggle。 / EP0 IN flips the toggle at every start.
      *ctrl =
          static_cast<uint8_t>(USBHS_UEP_T_RES_ACK |
                               (*ctrl & ~(USBHS_UEP_T_RES_MASK | USBHS_UEP_T_TOG_MDATA)) |
                               (tog0_ ? USBHS_UEP_T_TOG_DATA1 : 0U));
      tog0_ = !tog0_;
    }
  }
  else
  {
    SetDma(RxDma(ep), ep, request.data);
    volatile uint8_t* ctrl = RxCtrl(ep);
    if (iso)
    {
      *ctrl =
          static_cast<uint8_t>(USBHS_UEP_R_RES_ACK |
                               (*ctrl & ~(USBHS_UEP_R_RES_MASK | USBHS_UEP_R_TOG_MDATA)));
    }
    else if (is_ep0 && request.length == 0U)
    {
      // 控制传输的状态 OUT 固定从 DATA1 开始。 / The status OUT of a control transfer
      // always starts with DATA1.
      tog0_ = true;
      *ctrl =
          static_cast<uint8_t>(USBHS_UEP_R_RES_ACK |
                               (*ctrl & ~(USBHS_UEP_R_RES_MASK | USBHS_UEP_R_TOG_MDATA)) |
                               USBHS_UEP_R_TOG_DATA1);
    }
    else
    {
      // 其余只重开 ACK：EP0 数据包的 toggle 由完成路径逐包推进，批量和中断由软件翻转。
      // Otherwise only ACK is reopened: the completion path advances the EP0 data
      // toggle per packet, and software flips the bulk and interrupt toggle.
      *ctrl = static_cast<uint8_t>((*ctrl & ~USBHS_UEP_R_RES_MASK) | USBHS_UEP_R_RES_ACK);
    }
  }

  if (other != nullptr && idle && (USBHSD->INT_FG & USBHS_UIF_TRANSFER) != 0U)
  {
    other->tog_window_ = static_cast<uint8_t>(TOG_WINDOW | other_tog);
  }
  return ErrorCode::OK;
}

void CH32EndpointOtgHs::RepairLostToggle()
{
  // 没有记下窗口时和挂包时的值比。同号 OUT 在 UIF_TRANSFER 已置位的那一轮里重新挂接收
  // 时，IN 的推进照样会丢，而那时不记窗口（CH32V307 实测：7 次现场一致，都是“本端点 IN
  // 的完成不带 IS_NAK、toggle 没动”，之后下一包被主机丢掉）。
  // Without a recorded window the value at arming is compared. When the OUT of the same
  // number is armed again in a round whose UIF_TRANSFER is already set, the advance of
  // the IN is lost all the same, and no window is recorded then (measured on a
  // CH32V307: 7 identical captures, each "an IN completion of this endpoint without
  // IS_NAK and the toggle unmoved", after which the host dropped the next packet).
  const uint8_t before =
      tog_window_ != 0U ? static_cast<uint8_t>(tog_window_ & ~TOG_WINDOW) : tx_tog_armed_;
  tog_window_ = 0U;
  // 好包之后 toggle 必然离开写之前的值；没离开就是这次推进丢了。
  // After a good packet the toggle must have left its value from before the writes; if
  // it has not, the advance was lost.
  volatile uint8_t* ctrl = TxCtrl(GetNumber());
  if ((*ctrl & USBHS_UEP_T_TOG_MASK) == before)
  {
    *ctrl = static_cast<uint8_t>(*ctrl ^ USBHS_UEP_T_TOG_DATA1);
  }
}

Endpoint::StopOutcome CH32EndpointOtgHs::StopHardware(bool in_isr)
{
  UNUSED(in_isr);
  // 控制器复位之后，硬件上已没有这个请求。
  // After a controller reset the request is no longer on the hardware.
  if (!transfer_active_)
  {
    return {ErrorCode::ABORTED, 0U};
  }
  const EPNumber ep = GetNumber();
  const uint8_t n = EPNumberToInt8(ep);
  const bool is_in = GetDirection() == Direction::IN;

  if (iso_in_)
  {
    // 等时 IN：包还没装上硬件就当场结束；装上了而线上有事务时不等（可能正是它）。
    // Isochronous IN: a packet not on the hardware yet ends at once; with one loaded and
    // a transaction on the wire (which may be this one) there is no wait.
    if (iso_loaded_ && !SieQuiet())
    {
      BeginPending();
      return {ErrorCode::PENDING, 0U};
    }
    const StopOutcome outcome = SettleStop({}, false);
    transfer_active_ = false;
    return outcome;
  }

  // 1) NAK 从下一个事务起生效；写入时已在线上的事务照常完成，可能把一个包写进槽。
  // 1) NAK applies from the next transaction on; one already on the wire completes and
  //    may write a packet into the slot.
  if (!stopping_)
  {
    const bool idle = (USBHSD->INT_FG & USBHS_UIF_TRANSFER) == 0U;
    SetResponse(is_in ? USBHS_UEP_T_RES_NAK : USBHS_UEP_R_RES_NAK);
    // 写 NAK 的同时挂着的 IN 包恰好被 ACK，控制器对 IN toggle 的自动推进会丢，下一包用
    // 旧相位发出，主机当作重复包丢掉（CH32V307 实测：6.2 万次停止 53 次）。写之前
    // UIF_TRANSFER 未置位、写之后已置位且 INT_ST 指向本端点的 IN，而 toggle 没动，就是
    // 这种情况：补推一次。UIF_TRANSFER 置位期间控制器对后续事务回忙 NAK，补推不会再撞上
    // 一次完成；INT_ST 要到下一个令牌才会被改写，所以紧跟着写来读。
    // If the armed IN packet is ACKed just as NAK is written, the controller loses its
    // automatic advance of the IN toggle: the next packet goes out with the old phase
    // and the host drops it as a repeat (measured on a CH32V307: 53 of 62,000 stops).
    // UIF_TRANSFER clear before the write and set after it, with INT_ST pointing at this
    // endpoint's IN and the toggle unmoved, is that case: the advance is redone. While
    // UIF_TRANSFER is set the controller busy-NAKs further transactions, so the redo
    // cannot meet another completion; INT_ST is rewritten only by the next token, hence
    // it is read right after the write.
    if (is_in && idle && (USBHSD->INT_FG & USBHS_UIF_TRANSFER) != 0U &&
        (USBHSD->INT_ST & (USBHS_UIS_TOKEN_MASK | USBHS_UIS_ENDP_MASK)) ==
            (USBHS_UIS_TOKEN_IN | n) &&
        ep != EPNumber::EP0 && GetType() != Type::ISOCHRONOUS && !InPacketAcked())
    {
      volatile uint8_t* ctrl = TxCtrl(ep);
      *ctrl = static_cast<uint8_t>(*ctrl ^ USBHS_UEP_T_TOG_DATA1);
    }
  }

  // 2) 线上还有事务就不等：停止未结束。处理 SETUP 期间（SETUP_ACT 未清）控制器对所有
  //    令牌回忙 NAK，控制端点当场停稳（CH32V307 实测）；此前锁存的事件属于被 SETUP
  //    作废的那次传输，不收。
  // 2) A transaction on the wire is not waited for: the stop is pending. During SETUP
  //    handling (SETUP_ACT not cleared) the controller busy-NAKs every token, so the
  //    control endpoint is quiescent at once (measured on a CH32V307); an event latched
  //    before belongs to the transfer the SETUP voided and is not collected.
  const bool at_setup = ep == EPNumber::EP0 && setup_held_;
  if (!at_setup && !SieQuiet())
  {
    BeginPending();
    return {ErrorCode::PENDING, 0U};
  }

  // 3) 收下锁存的本端点完成；别的端点的留给事件循环，并挂起本线保证它被服务。
  // 3) Collect a latched completion of this endpoint; another endpoint's stays for the
  //    event loop, and the line is set pending so that it gets serviced.
  bool mine = false;
  uint8_t int_st = 0U;
  uint16_t rx_len = 0U;
  if (!at_setup && !transfer_held_ && (USBHSD->INT_FG & USBHS_UIF_TRANSFER) != 0U)
  {
    int_st = USBHSD->INT_ST;
    const uint8_t token = is_in ? USBHS_UIS_TOKEN_IN : USBHS_UIS_TOKEN_OUT;
    if ((int_st & USBHS_UIS_ENDP_MASK) == n && (int_st & USBHS_UIS_TOKEN_MASK) == token)
    {
      mine = true;
      rx_len = USBHSD->RX_LEN;
      USBHSD->INT_FG = USBHS_UIF_TRANSFER;
    }
    else
    {
      NVIC_SetPendingIRQ(USBHS_IRQn);
    }
  }
  // 带 IS_NAK 的 OUT 包，以及 SETUP 已锁存时的 OUT 包，按 toggle 正确处理，见
  // ServiceHardware。
  // An OUT packet with IS_NAK, and one with a SETUP latched, is taken as in step, see
  // ServiceHardware.
  const bool setup_latched = (USBHSD->INT_FG & USBHS_UIF_SETUP_ACT) != 0U;
  const StopOutcome outcome = SettleStop(
      {mine, (int_st & USBHS_UIS_IS_NAK) == 0U,
       setup_latched || (int_st & (USBHS_UIS_TOG_OK | USBHS_UIS_IS_NAK)) != 0U, rx_len},
      true);
  EndPending();
  transfer_active_ = false;
  return outcome;
}

Endpoint::StopOutcome CH32EndpointOtgHs::SettleStop(const StopEvent& event, bool flip_out)
{
  const bool mine = event.mine;
  const bool in_acked = event.in_acked;
  const bool out_tog_ok = event.out_tog_ok;
  const size_t rx_len = event.rx_len;
  const EPNumber ep = GetNumber();
  const bool is_ep0 = ep == EPNumber::EP0;
  const bool iso = GetType() == Type::ISOCHRONOUS;
  const size_t mps = MaxPacketSize();
  if (iso_in_)
  {
    // 装上的包已发出（toggle 已变）：寄存器留给 OnIsoInSent 在这次发送之后改写。没发出：
    // 撤回，换成没有包的状态；这次写不在“刚发完一包”的时刻，调用者已确认 SIE 空闲。
    // The loaded packet has gone out (the toggle moved): the registers are left to
    // OnIsoInSent, which rewrites them after this transmission. Not gone out: it is
    // retracted and the no-packet state loaded; this write is not at "a packet has just
    // gone out", and the caller has made sure the SIE is free.
    const bool sent = iso_loaded_ && IsoInSent();
    if (iso_loaded_ && !sent)
    {
      IdleIsoIn();
    }
    iso_loaded_ = false;
    return {sent ? ErrorCode::OK : ErrorCode::ABORTED, sent ? transfer_requested_ : 0U};
  }
  if (GetDirection() == Direction::IN)
  {
    // 挂着的包：只有最后一包可能短于包长。 / The armed packet: only the last one can be
    // shorter than the packet size.
    size_t chunk = mps;
    if (transfer_remaining_ == 0U)
    {
      chunk = transfer_requested_ == 0U
                  ? 0U
                  : transfer_requested_ - (transfer_requested_ - 1U) / mps * mps;
    }
    size_t confirmed = transfer_requested_ - transfer_remaining_ - chunk;
    const bool auto_tog = !is_ep0 && !iso;
    const bool acked = (mine && in_acked) || (auto_tog && InPacketAcked());
    if (auto_tog && acked)
    {
      RepairLostToggle();
    }
    if (acked)
    {
      confirmed += chunk;
      if (transfer_remaining_ == 0U)
      {
        return {ErrorCode::OK, confirmed};
      }
    }
    return {ErrorCode::ABORTED, confirmed};
  }

  // OUT：本端点的完成事件是已应答、已写进槽的整包（实测），必须计入。
  // OUT: a completion event of this endpoint is a whole packet already acknowledged and
  // written to the slot (measured) and must be counted.
  bool done = false;
  if (mine)
  {
    bool tog_ok = true;
    if (is_ep0)
    {
      if (rx_len > 0U)
      {
        *RxCtrl(ep) = static_cast<uint8_t>(*RxCtrl(ep) ^ USBHS_UEP_R_TOG_DATA1);
      }
    }
    else if (!iso)
    {
      tog_ok = out_tog_ok;
      if (tog_ok && flip_out)
      {
        *RxCtrl(ep) = static_cast<uint8_t>(*RxCtrl(ep) ^ USBHS_UEP_R_TOG_DATA1);
      }
    }
    if (tog_ok)
    {
      transfer_accumulated_ += rx_len;
      done = transfer_accumulated_ >= transfer_requested_ || rx_len < mps || iso;
    }
  }
  return {done ? ErrorCode::OK : ErrorCode::ABORTED, transfer_accumulated_};
}

void CH32EndpointOtgHs::BeginPending()
{
  if (stopping_)
  {
    return;
  }
  stopping_ = true;
  stop_us_ = static_cast<uint64_t>(Timebase::GetMicroseconds());
  ++pending_stops_;
  // 等待已在运行时返回 BUSY：它结束后的那一轮也会查到这个停止。
  // BUSY while the wait already runs: the pass after it ends checks this stop as well.
  (void)tick_->Start(1U, 1U);
}

void CH32EndpointOtgHs::EndPending()
{
  if (!stopping_)
  {
    return;
  }
  stopping_ = false;
  --pending_stops_;
}

void CH32EndpointOtgHs::FinishStop(bool in_isr, const StopOutcome& outcome)
{
  EndPending();
  transfer_active_ = false;
  OnHardwareComplete(in_isr, outcome.actual, outcome.result);
}

void CH32EndpointOtgHs::FinishQuietStops(bool in_isr, bool force, bool only_ep0)
{
  if (pending_stops_ == 0U)
  {
    return;
  }
  // 还有传输事件没处理时不判断：它可能就是被停端点的完成，由事件循环交给 FinishStop。
  // Nothing is decided while a transfer event waits: it may be the stopped endpoint's
  // completion, which the event loop hands to FinishStop.
  // 先读 SIE 空闲再读事件标志：两次读之间结束的事务会留下标志；之后开始的事务被 NAK。
  // The free SIE is read before the event flag: a transaction ending between the two
  // reads leaves the flag, and one starting later is NAKed.
  if (!force && (!SieQuiet() || (USBHSD->INT_FG & USBHS_UIF_TRANSFER) != 0U))
  {
    return;
  }
  for (uint8_t n = 0U; n < (only_ep0 ? 1U : EP_OTG_HS_MAX_SIZE); ++n)
  {
    for (CH32EndpointOtgHs* ep : map_[n])
    {
      if (ep != nullptr && ep->stopping_)
      {
        ep->FinishStop(in_isr, ep->SettleStop({}, false));
      }
    }
  }
}

void CH32EndpointOtgHs::AgeStops(bool in_isr)
{
  if (pending_stops_ == 0U)
  {
    return;
  }
  const uint64_t now = static_cast<uint64_t>(Timebase::GetMicroseconds());
  for (auto& pair : map_)
  {
    for (CH32EndpointOtgHs* ep : pair)
    {
      if (ep != nullptr && ep->stopping_ && now - ep->stop_us_ >= STOP_AGE_US)
      {
        ep->FinishStop(in_isr, ep->SettleStop({}, false));
      }
    }
  }
  if (pending_stops_ != 0U)
  {
    (void)tick_->Start(1U, 1U);
  }
}

ErrorCode CH32EndpointOtgHs::StallHardware(bool in_isr)
{
  UNUSED(in_isr);
  SetResponse(GetDirection() == Direction::IN ? USBHS_UEP_T_RES_STALL
                                              : USBHS_UEP_R_RES_STALL);
  return ErrorCode::OK;
}

ErrorCode CH32EndpointOtgHs::ClearStallHardware(bool in_isr)
{
  UNUSED(in_isr);
  if (GetNumber() == EPNumber::EP0)
  {
    // 下一个 SETUP 已在 ServiceHardware 里把 EP0 恢复为 NAK/DATA1；再写会让数据阶段首包
    // 变成 DATA0，主机当作重复包丢掉。
    // The next SETUP has already restored EP0 to NAK/DATA1 in ServiceHardware; writing
    // again would send the first data packet as DATA0, which the host drops as a repeat.
    return ErrorCode::OK;
  }
  // 主机也会对没有 STALL 的端点清 HALT（例如 libusb_clear_halt），这时硬件上可能挂着
  // 请求：toggle 回到 DATA0，请求原样留着继续等主机（ApplyClearToggle）。在事件轮里
  // （处理 SETUP 时）推迟到本轮末尾，见 FinishClearToggles；IN 包已被 ACK 而完成还没
  // 交付时也推迟：那个完成锁存着，下一轮先交付它。
  // A host clears the halt of an endpoint that is not halted too (e.g.
  // libusb_clear_halt), and a request may be on the hardware then: the toggle returns to
  // DATA0 and the request stays and waits for the host (ApplyClearToggle). Within an
  // event round (handling the SETUP) this waits for the end of the round, see
  // FinishClearToggles; it also waits when an IN packet was ACKed and its completion is
  // not delivered yet: that completion is latched and the next round delivers it first.
  const bool in_acked = GetDirection() == Direction::IN && InPacketAcked();
  if (setup_held_ || transfer_held_ || in_acked)
  {
    clear_tog_pending_ |=
        1U << (2U * EPNumberToInt8(GetNumber()) + static_cast<uint8_t>(GetDirection()));
    return ErrorCode::OK;
  }
  ApplyClearToggle();
  return ErrorCode::OK;
}

void CH32EndpointOtgHs::ApplyClearToggle()
{
  if (GetDirection() == Direction::IN)
  {
    // 自动 toggle 保持；挂着的包改用 DATA0 发出，按 DATA0 认出已发。窗口作废。
    // The automatic toggle stays; an armed packet goes out as DATA0 and is recognised as
    // sent against DATA0. The window is void.
    volatile uint8_t* ctrl = TxCtrl(GetNumber());
    uint8_t res = *ctrl & USBHS_UEP_T_RES_MASK;
    if (res == USBHS_UEP_T_RES_STALL)
    {
      res = USBHS_UEP_T_RES_NAK;
    }
    *ctrl = static_cast<uint8_t>(res | USBHS_UEP_T_TOG_AUTO | USBHS_UEP_T_TOG_DATA0);
    tx_tog_armed_ = USBHS_UEP_T_TOG_DATA0;
    tog_window_ = 0U;
  }
  else
  {
    volatile uint8_t* ctrl = RxCtrl(GetNumber());
    uint8_t res = *ctrl & USBHS_UEP_R_RES_MASK;
    if (res == USBHS_UEP_R_RES_STALL)
    {
      res = USBHS_UEP_R_RES_NAK;
    }
    *ctrl = static_cast<uint8_t>(
        (*ctrl & ~(USBHS_UEP_R_RES_MASK | USBHS_UEP_R_TOG_MASK)) | res);
  }
}

void CH32EndpointOtgHs::FinishClearToggles()
{
  if (clear_tog_pending_ == 0U)
  {
    return;
  }
  const uint32_t pending = clear_tog_pending_;
  clear_tog_pending_ = 0U;
  for (uint8_t n = 1U; n < EP_OTG_HS_MAX_SIZE; ++n)
  {
    for (uint8_t d = 0U; d < 2U; ++d)
    {
      CH32EndpointOtgHs* ep = map_[n][d];
      // 本轮锁存的 IN 完成在这之前已交付（按 toggle 扫描），SETUP_ACT 和 INT_BUSY 挂着时
      // 不会有新的事务完成，所以这里没有已发未交付的 IN 包。
      // The IN completions latched in this round were delivered before this (the scan by
      // toggle), and no new transaction completes while SETUP_ACT and INT_BUSY hold, so
      // no IN packet here is sent and not yet delivered.
      if (ep != nullptr && (pending & (1U << (2U * n + d))) != 0U)
      {
        ep->ApplyClearToggle();
      }
    }
  }
}

Endpoint::HwLimits CH32EndpointOtgHs::HardwareLimits(Direction direction, Type type) const
{
  UNUSED(direction);
  // 手册 22.2.2（p.340）：端点 0 最大 64 字节，其余端点 1024 字节。批量取高速规定的
  // 512；中断取 64、等时取 1023，比手册窄，更大的包长没有在板上验证过。控制和等时一次
  // 请求只有一包（等时 IN 硬件一次只发一包）；批量和中断由软件逐包推进，包数不限。
  // Manual 22.2.2 (p.340): endpoint 0 takes 64 bytes at most, every other endpoint 1024.
  // Bulk uses the 512 that high speed prescribes; interrupt uses 64 and isochronous
  // 1023, narrower than the manual: larger packets have not been verified on a board.
  // Control and isochronous carry one packet per request (the hardware sends one
  // isochronous IN packet at a time); software advances bulk and interrupt packet by
  // packet, with no packet limit.
  switch (type)
  {
    case Type::CONTROL:
      return {64U, 1U, SIZE_MAX};
    case Type::ISOCHRONOUS:
      return {1023U, 1U, SIZE_MAX};
    case Type::BULK:
      return {512U, SIZE_MAX, SIZE_MAX};
    case Type::INTR:
      return {64U, SIZE_MAX, SIZE_MAX};
  }
  return {};
}

bool CH32EndpointOtgHs::InPacketAcked() const
{
  if (!transfer_active_ || GetDirection() != Direction::IN ||
      GetNumber() == EPNumber::EP0 || iso_in_)
  {
    return false;
  }
  return (*TxCtrl(GetNumber()) & USBHS_UEP_T_TOG_MASK) != tx_tog_armed_;
}

bool CH32EndpointOtgHs::IsoInSent() const
{
  return (*TxCtrl(GetNumber()) & USBHS_UEP_T_TOG_MASK) != tx_tog_armed_;
}

void CH32EndpointOtgHs::IdleIsoIn()
{
  const EPNumber ep = GetNumber();
  SetTxLen(ep, 0U);
  *TxCtrl(ep) = USBHS_UEP_T_RES_ACK | USBHS_UEP_T_TOG_AUTO | USBHS_UEP_T_TOG_DATA0;
  tx_tog_armed_ = USBHS_UEP_T_TOG_DATA0;
  iso_loaded_ = false;
}

void CH32EndpointOtgHs::LoadIsoIn()
{
  const EPNumber ep = GetNumber();
  *TxDma(ep) = Address(transfer_cursor_);
  SetTxLen(ep, transfer_requested_);
  *TxCtrl(ep) = USBHS_UEP_T_RES_ACK | USBHS_UEP_T_TOG_AUTO | USBHS_UEP_T_TOG_DATA0;
  tx_tog_armed_ = USBHS_UEP_T_TOG_DATA0;
  iso_loaded_ = true;
}

void CH32EndpointOtgHs::OnIsoInSent(bool in_isr)
{
  const bool sent = iso_loaded_;
  iso_free_pass_ = pass_;
  // 先把寄存器换成没有包的状态：下面的回调之后核心才启动下一个请求，那时再装入。
  // The registers go to the no-packet state first: the core starts the next request
  // only after the callback below, and it is loaded then.
  IdleIsoIn();
  if (!transfer_active_)
  {
    return;
  }
  if (stopping_)
  {
    FinishStop(in_isr, {sent ? ErrorCode::OK : ErrorCode::ABORTED,
                        sent ? transfer_requested_ : 0U});
    return;
  }
  if (sent)
  {
    transfer_active_ = false;
    OnHardwareComplete(in_isr, transfer_requested_);
    return;
  }
  // 在等的请求：刚才发出去的是零长度包，现在装入。 / A waiting request: what went
  // out was a zero-length packet, and the request is loaded now.
  LoadIsoIn();
}

void CH32EndpointOtgHs::ServiceIsoIn(bool in_isr)
{
  for (uint8_t i = 1U; i < EP_OTG_HS_MAX_SIZE; ++i)
  {
    CH32EndpointOtgHs* ep_in = map_[i][IN_IDX];
    if (ep_in != nullptr && ep_in->iso_in_ && ep_in->IsoInSent())
    {
      ep_in->OnIsoInSent(in_isr);
    }
  }
}

void CH32EndpointOtgHs::CompleteAckedInPackets(bool in_isr)
{
  // INT_BUSY 让控制器在 UIF_TRANSFER 清零前对后续事务回忙 NAK，而 INT_ST 记录最近一次
  // 事务：IN 包刚被 ACK、主机紧接着的 IN（下一包、零长度包或别的端点）被忙 NAK 后，
  // INT_ST 就不再指向这个包，按 INT_ST 分发会丢掉这次完成，控制器随后把同一包再发一遍。
  // 所以每次 UIF_TRANSFER 都按 toggle 检查挂着的 IN 包。UIF_TRANSFER 清零前不会有新事务
  // 完成，续挂的包不会在同一次检查里被误判为已发。
  // With INT_BUSY the controller busy-NAKs further transactions until UIF_TRANSFER is
  // cleared, while INT_ST describes the latest transaction: when the IN right after an
  // ACKed packet (the next packet, a ZLP, or another endpoint) is busy-NAKed, INT_ST no
  // longer points at the packet, dispatch by INT_ST loses the completion and the
  // controller sends the packet again. Hence every UIF_TRANSFER checks the toggle of
  // each armed IN packet. No transaction completes before UIF_TRANSFER is cleared, so a
  // packet re-armed here is not taken for a sent one in the same check.
  for (uint8_t i = 1U; i < EP_OTG_HS_MAX_SIZE; ++i)
  {
    CH32EndpointOtgHs* ep_in = map_[i][IN_IDX];
    if (ep_in != nullptr && ep_in->InPacketAcked())
    {
      ep_in->TransferComplete(in_isr, 0U, true);
    }
  }
}

void CH32EndpointOtgHs::TransferComplete(bool in_isr, size_t size, bool tog_ok)
{
  const EPNumber ep = GetNumber();
  const bool is_in = GetDirection() == Direction::IN;
  const bool is_ep0 = ep == EPNumber::EP0;
  const bool iso = GetType() == Type::ISOCHRONOUS;

  // 软件 toggle：TOG_OK 的包已被应答、主机已翻相位，软件跟着翻，没有请求在途时也翻。
  // TOG_OK=0 是主机重发的旧包，不翻。
  // Software toggle: a TOG_OK packet was acknowledged and the host advanced its phase,
  // so software advances too, also with nothing in flight. TOG_OK=0 is a resend of an
  // old packet and does not flip.
  if (!is_in && !is_ep0 && !iso && tog_ok)
  {
    *RxCtrl(ep) = static_cast<uint8_t>(*RxCtrl(ep) ^ USBHS_UEP_R_TOG_DATA1);
  }
  if (!transfer_active_ || iso_in_)
  {
    return;
  }
  if (stopping_)
  {
    // 停止未结束时到来的本端点完成：线上那个事务结束了。批量和中断 OUT 的软件 toggle
    // 上面已经翻过。
    // A completion of this endpoint while its stop is pending: the transaction on the
    // wire is over. The software toggle of a bulk or interrupt OUT was flipped above.
    FinishStop(in_isr, SettleStop({true, true, tog_ok, size}, false));
    return;
  }

  if (is_in)
  {
    if (!is_ep0 && !iso)
    {
      RepairLostToggle();
    }
    const size_t mps = MaxPacketSize();
    if (transfer_remaining_ > 0U)
    {
      const size_t chunk = transfer_remaining_ > mps ? mps : transfer_remaining_;
      SetDma(TxDma(ep), ep, transfer_cursor_);
      SetTxLen(ep, chunk);
      transfer_cursor_ += chunk;
      transfer_remaining_ -= chunk;
      SetResponse(USBHS_UEP_T_RES_ACK);
      tx_tog_armed_ = *TxCtrl(ep) & USBHS_UEP_T_TOG_MASK;
      return;
    }
    SetResponse(USBHS_UEP_T_RES_NAK);
    transfer_active_ = false;
    OnHardwareComplete(in_isr, transfer_requested_);
    return;
  }

  if (is_ep0)
  {
    // 不把 EP0 OUT 留在 ACK：核心处理完这个包后才挂下一个。收到数据的包推进一次 toggle
    // （多包 OUT 数据阶段在这里推进）。
    // EP0 OUT is not left at ACK: the core arms the next packet after handling this
    // one. A packet with data advances the toggle once (the multi-packet OUT data stage
    // advances here).
    volatile uint8_t* ctrl = RxCtrl(ep);
    const uint8_t tog = size > 0U ? USBHS_UEP_R_TOG_DATA1 : 0U;
    *ctrl = static_cast<uint8_t>(((*ctrl ^ tog) & ~USBHS_UEP_R_RES_MASK) |
                                 USBHS_UEP_R_RES_NAK);
    transfer_active_ = false;
    OnHardwareComplete(in_isr, size);
    return;
  }

  if (!iso && !tog_ok)
  {
    // toggle 不符：主机重发了已收下的包（它没收到 ACK）。丢掉它，原样重新挂接收。
    // Toggle mismatch: the host resent a packet already taken (it missed the ACK). Drop
    // it and arm the receive again as it was.
    SetResponse(USBHS_UEP_R_RES_ACK);
    return;
  }

  if (!iso)
  {
    // 批量和中断：RX DMA 逐包推进，收满或短包时整个请求完成一次。
    // Bulk and interrupt: the RX DMA advances per packet and the request completes once
    // when full or on a short packet.
    transfer_accumulated_ += size;
    if (transfer_accumulated_ < transfer_requested_ && size == MaxPacketSize())
    {
      transfer_cursor_ += size;
      *RxDma(ep) = Address(transfer_cursor_);
      return;
    }
  }
  else
  {
    transfer_accumulated_ = size;
  }
  SetResponse(USBHS_UEP_R_RES_NAK);
  transfer_active_ = false;
  OnHardwareComplete(in_isr, transfer_accumulated_);
}

#endif  // LIBXR_CH32_HAS_USBHS

// NOLINTEND(cppcoreguidelines-pro-type-cstyle-cast,performance-no-int-to-ptr)
