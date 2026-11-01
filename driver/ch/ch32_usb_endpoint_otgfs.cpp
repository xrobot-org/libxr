#include <cstdint>

#include "ch32_usb_endpoint.hpp"
#include "ep.hpp"

using namespace LibXR;

#if defined(USBFSD)

// NOLINTBEGIN

static inline volatile uint8_t* get_tx_ctrl_addr(USB::Endpoint::EPNumber ep)
{
  return &USBFSD->UEP0_TX_CTRL + 4 * (USB::Endpoint::EPNumberToInt8(ep));
}
static inline volatile uint8_t* get_rx_ctrl_addr(USB::Endpoint::EPNumber ep)
{
  return &USBFSD->UEP0_RX_CTRL + 4 * (USB::Endpoint::EPNumberToInt8(ep));
}
static inline volatile uint16_t* get_tx_len_addr(USB::Endpoint::EPNumber ep)
{
  return &USBFSD->UEP0_TX_LEN + 2 * (USB::Endpoint::EPNumberToInt8(ep));
}
static inline volatile uint32_t* get_dma_addr(USB::Endpoint::EPNumber ep)
{
  return &USBFSD->UEP0_DMA + USB::Endpoint::EPNumberToInt8(ep);
}

static void set_dma_buffer(USB::Endpoint::EPNumber ep_num, void* value,
                           bool double_buffer)
{
  *get_dma_addr(ep_num) = (uint32_t)value;

  switch (ep_num)
  {
    case USB::Endpoint::EPNumber::EP1:
      if (double_buffer)
        USBFSD->UEP4_1_MOD |= USBFS_UEP1_BUF_MOD;
      else
        USBFSD->UEP4_1_MOD &= ~USBFS_UEP1_BUF_MOD;
      break;
    case USB::Endpoint::EPNumber::EP2:
      if (double_buffer)
        USBFSD->UEP2_3_MOD |= USBFS_UEP2_BUF_MOD;
      else
        USBFSD->UEP2_3_MOD &= ~USBFS_UEP2_BUF_MOD;
      break;
    case USB::Endpoint::EPNumber::EP3:
      if (double_buffer)
        USBFSD->UEP2_3_MOD |= USBFS_UEP3_BUF_MOD;
      else
        USBFSD->UEP2_3_MOD &= ~USBFS_UEP3_BUF_MOD;
      break;
    case USB::Endpoint::EPNumber::EP4:
      if (double_buffer)
        USBFSD->UEP4_1_MOD |= USBFS_UEP4_BUF_MOD;
      else
        USBFSD->UEP4_1_MOD &= ~USBFS_UEP4_BUF_MOD;
      break;
    case USB::Endpoint::EPNumber::EP5:
      if (double_buffer)
        USBFSD->UEP5_6_MOD |= USBFS_UEP5_BUF_MOD;
      else
        USBFSD->UEP5_6_MOD &= ~USBFS_UEP5_BUF_MOD;
      break;
    case USB::Endpoint::EPNumber::EP6:
      if (double_buffer)
        USBFSD->UEP5_6_MOD |= USBFS_UEP6_BUF_MOD;
      else
        USBFSD->UEP5_6_MOD &= ~USBFS_UEP6_BUF_MOD;
      break;
    case USB::Endpoint::EPNumber::EP7:
      if (double_buffer)
        USBFSD->UEP7_MOD |= USBFS_UEP7_BUF_MOD;
      else
        USBFSD->UEP7_MOD &= ~USBFS_UEP7_BUF_MOD;
      break;
    default:
      break;
  }
}

static void set_tx_len(USB::Endpoint::EPNumber ep_num, uint32_t value)
{
  *get_tx_len_addr(ep_num) = value;
}

static void enable_tx(USB::Endpoint::EPNumber ep_num)
{
  switch (ep_num)
  {
    case USB::Endpoint::EPNumber::EP1:
      USBFSD->UEP4_1_MOD |= USBFS_UEP1_TX_EN;
      break;
    case USB::Endpoint::EPNumber::EP2:
      USBFSD->UEP2_3_MOD |= USBFS_UEP2_TX_EN;
      break;
    case USB::Endpoint::EPNumber::EP3:
      USBFSD->UEP2_3_MOD |= USBFS_UEP3_TX_EN;
      break;
    case USB::Endpoint::EPNumber::EP4:
      USBFSD->UEP4_1_MOD |= USBFS_UEP4_TX_EN;
      break;
    case USB::Endpoint::EPNumber::EP5:
      USBFSD->UEP5_6_MOD |= USBFS_UEP5_TX_EN;
      break;
    case USB::Endpoint::EPNumber::EP6:
      USBFSD->UEP5_6_MOD |= USBFS_UEP6_TX_EN;
      break;
    case USB::Endpoint::EPNumber::EP7:
      USBFSD->UEP7_MOD |= USBFS_UEP7_TX_EN;
      break;
    default:
      break;
  }
}
static void disable_tx(USB::Endpoint::EPNumber ep_num)
{
  switch (ep_num)
  {
    case USB::Endpoint::EPNumber::EP1:
      USBFSD->UEP4_1_MOD &= ~USBFS_UEP1_TX_EN;
      break;
    case USB::Endpoint::EPNumber::EP2:
      USBFSD->UEP2_3_MOD &= ~USBFS_UEP2_TX_EN;
      break;
    case USB::Endpoint::EPNumber::EP3:
      USBFSD->UEP2_3_MOD &= ~USBFS_UEP3_TX_EN;
      break;
    case USB::Endpoint::EPNumber::EP4:
      USBFSD->UEP4_1_MOD &= ~USBFS_UEP4_TX_EN;
      break;
    case USB::Endpoint::EPNumber::EP5:
      USBFSD->UEP5_6_MOD &= ~USBFS_UEP5_TX_EN;
      break;
    case USB::Endpoint::EPNumber::EP6:
      USBFSD->UEP5_6_MOD &= ~USBFS_UEP6_TX_EN;
      break;
    case USB::Endpoint::EPNumber::EP7:
      USBFSD->UEP7_MOD &= ~USBFS_UEP7_TX_EN;
      break;
    default:
      break;
  }
}
static void enable_rx(USB::Endpoint::EPNumber ep_num)
{
  switch (ep_num)
  {
    case USB::Endpoint::EPNumber::EP1:
      USBFSD->UEP4_1_MOD |= USBFS_UEP1_RX_EN;
      break;
    case USB::Endpoint::EPNumber::EP2:
      USBFSD->UEP2_3_MOD |= USBFS_UEP2_RX_EN;
      break;
    case USB::Endpoint::EPNumber::EP3:
      USBFSD->UEP2_3_MOD |= USBFS_UEP3_RX_EN;
      break;
    case USB::Endpoint::EPNumber::EP4:
      USBFSD->UEP4_1_MOD |= USBFS_UEP4_RX_EN;
      break;
    case USB::Endpoint::EPNumber::EP5:
      USBFSD->UEP5_6_MOD |= USBFS_UEP5_RX_EN;
      break;
    case USB::Endpoint::EPNumber::EP6:
      USBFSD->UEP5_6_MOD |= USBFS_UEP6_RX_EN;
      break;
    case USB::Endpoint::EPNumber::EP7:
      USBFSD->UEP7_MOD |= USBFS_UEP7_RX_EN;
      break;
    default:
      break;
  }
}
static void disable_rx(USB::Endpoint::EPNumber ep_num)
{
  switch (ep_num)
  {
    case USB::Endpoint::EPNumber::EP1:
      USBFSD->UEP4_1_MOD &= ~USBFS_UEP1_RX_EN;
      break;
    case USB::Endpoint::EPNumber::EP2:
      USBFSD->UEP2_3_MOD &= ~USBFS_UEP2_RX_EN;
      break;
    case USB::Endpoint::EPNumber::EP3:
      USBFSD->UEP2_3_MOD &= ~USBFS_UEP3_RX_EN;
      break;
    case USB::Endpoint::EPNumber::EP4:
      USBFSD->UEP4_1_MOD &= ~USBFS_UEP4_RX_EN;
      break;
    case USB::Endpoint::EPNumber::EP5:
      USBFSD->UEP5_6_MOD &= ~USBFS_UEP5_RX_EN;
      break;
    case USB::Endpoint::EPNumber::EP6:
      USBFSD->UEP5_6_MOD &= ~USBFS_UEP6_RX_EN;
      break;
    case USB::Endpoint::EPNumber::EP7:
      USBFSD->UEP7_MOD &= ~USBFS_UEP7_RX_EN;
      break;
    default:
      break;
  }
}
// NOLINTEND

// NOLINTNEXTLINE
static LibXR::RawData select_buffer(USB::Endpoint::EPNumber ep_num,
                                    USB::Endpoint::Direction dir,
                                    const LibXR::RawData& buffer)
{
  if (ep_num == USB::Endpoint::EPNumber::EP0)
  {
    return buffer;
  }

  const size_t half = 64U;
  ASSERT(buffer.size_ >= 128U);

  if (dir == USB::Endpoint::Direction::OUT)
  {
    return LibXR::RawData(buffer.addr_, half);
  }

  return LibXR::RawData(reinterpret_cast<uint8_t*>(buffer.addr_) + half, half);
}

CH32EndpointOtgFs::CH32EndpointOtgFs(EPNumber ep_num, Direction dir,
                                     LibXR::RawData buffer, bool single_direction)
    : Endpoint(ep_num, dir,
               (single_direction || ep_num == EPNumber::EP0)
                   ? buffer
                   : select_buffer(ep_num, dir, buffer),
               (single_direction && ep_num != EPNumber::EP0) ? 2 : 1),
      single_direction_(single_direction),
      dma_buffer_(buffer)
{
  map_otg_fs_[EPNumberToInt8(GetNumber())][static_cast<uint8_t>(dir)] = this;

  // 共享双向端点：IN 对象拿到的地址是 OUT+64，块基址要回退 64。
  // For a shared bidirectional endpoint the IN object receives OUT+64; step
  // back to the block base.
  block_dma_base_ = (!single_direction && ep_num != EPNumber::EP0 && dir == Direction::IN)
                        ? static_cast<uint8_t*>(buffer.addr_) - SHARED_BLOCK_HALF
                        : static_cast<uint8_t*>(buffer.addr_);

  set_dma_buffer(GetNumber(), block_dma_base_, false);

  if (dir == Direction::IN)
  {
    set_tx_len(GetNumber(), 0);
    *get_tx_ctrl_addr(GetNumber()) = USBFS_UEP_T_RES_NAK;
  }
  else
  {
    *get_rx_ctrl_addr(GetNumber()) = USBFS_UEP_R_RES_NAK;
  }
}

ErrorCode CH32EndpointOtgFs::ConfigureHardware(const Config& cfg)
{
  // 包长已由核心按 MaxPacketSizeLimit 收敛；状态由基类管理，这里只编程寄存器。
  is_isochronous_ = (cfg.type == Type::ISOCHRONOUS);

  if (IsPlanning())
  {
    return ErrorCode::OK;
  }

  const bool is_ep0 = (GetNumber() == EPNumber::EP0);
  const uint8_t tx_toggle = is_isochronous_ ? 0U : USBFS_UEP_T_AUTO_TOG;
  const uint8_t rx_toggle = is_isochronous_ ? 0U : USBFS_UEP_R_AUTO_TOG;

  if (!single_direction_ && !is_ep0)
  {
    // 单 bank 共享双向：IN 地址固定在 OUT +64，两个使能保持稳定，NAK 是各方向
    // 的激活闸门。
    enable_tx(GetNumber());
    enable_rx(GetNumber());
  }
  if (GetDirection() == Direction::IN)
  {
    set_tx_len(GetNumber(), 0);
    *get_tx_ctrl_addr(GetNumber()) = USBFS_UEP_T_RES_NAK | tx_toggle;
    enable_tx(GetNumber());
    if (single_direction_) disable_rx(GetNumber());
  }
  else
  {
    *get_rx_ctrl_addr(GetNumber()) = USBFS_UEP_R_RES_NAK | rx_toggle;
    enable_rx(GetNumber());
    if (single_direction_) disable_tx(GetNumber());
  }

  set_dma_buffer(GetNumber(), block_dma_base_, false);
  transfer_active_ = false;
  transfer_tag_ = 0;
  transfer_requested_ = 0;
  transfer_accumulated_ = 0;
  transfer_cursor_ = nullptr;
  transfer_remaining_ = 0;
  return ErrorCode::OK;
}

void CH32EndpointOtgFs::CloseHardware(bool /*in_isr*/)
{
  if (GetDirection() == Direction::IN)
  {
    *get_tx_ctrl_addr(GetNumber()) = USBFS_UEP_T_RES_NAK;
    if (single_direction_) disable_tx(GetNumber());
  }
  else
  {
    *get_rx_ctrl_addr(GetNumber()) = USBFS_UEP_R_RES_NAK;
    if (single_direction_) disable_rx(GetNumber());
  }
  transfer_active_ = false;
}

ErrorCode CH32EndpointOtgFs::StartHardware(const HwRequest& request, bool /*in_isr*/)
{
  const bool is_in = (GetDirection() == Direction::IN);
  const bool is_ep0 = (GetNumber() == EPNumber::EP0);

  transfer_tag_ = request.tag;
  transfer_requested_ = request.length;
  transfer_accumulated_ = 0;
  transfer_active_ = true;
  transfer_dma_ = request.slot;
  transfer_cursor_ = static_cast<uint8_t*>(request.slot.addr_);
  transfer_remaining_ = 0;
  hw_completed_ = false;

  if (is_in)
  {
    // 槽内存就是 DMA 缓冲：类在 Submit 前已把数据写进槽，无需再拷贝。
    // 共享双向端点不能动 UEPn_DMA：硬件的 IN 区固定在块基址 +64，写了 IN 槽
    // 地址会把发送区推到 base+128、并让 OUT 收到 IN 区里去。
    // Shared bidirectional endpoints must not touch UEPn_DMA: the hardware IN
    // area is fixed at block base +64, so writing the IN slot address would
    // transmit from base+128 and let OUT land in the IN area.
    if (single_direction_ || is_ep0)
    {
      set_dma_buffer(GetNumber(), request.slot.addr_, false);
    }
    // 超过一个包长的 IN 请求按 MPS 分包：先发第一包，余下由完成路径续发。
    // IN requests longer than one packet are split at MPS: send the first
    // packet here and continue from the completion path.
    const size_t mps = MaxPacketSize();
    const size_t chunk = (mps > 0U && request.length > mps) ? mps : request.length;
    transfer_remaining_ = request.length - chunk;
    transfer_cursor_ += chunk;
    set_tx_len(GetNumber(), chunk);
    auto addr = get_tx_ctrl_addr(GetNumber());

    if (!is_ep0)
    {
      *addr = (is_isochronous_ ? USBFS_UEP_T_RES_NONE : USBFS_UEP_T_RES_ACK) |
              (*addr & (~USBFS_UEP_T_RES_MASK));
    }
    else
    {
      // EP0 IN 按 start 次数翻转 toggle（HS 已上板方案）。
      *addr = USBFS_UEP_T_RES_ACK | (tog_ ? USBFS_UEP_T_TOG : 0);
      tog_ = !tog_;
    }
    return ErrorCode::OK;
  }

  // OUT：共享模式每包完成且不推进 DMA（UEPn_DMA 收发共用，推进会破坏 IN 固定
  // +64 的硬件布局）；单向模式按包推进 DMA，多包请求一次完成。
  if (single_direction_ || is_ep0)
  {
    set_dma_buffer(GetNumber(), request.slot.addr_, false);
  }
  auto addr = get_rx_ctrl_addr(GetNumber());
  if (!is_ep0)
  {
    *addr = (is_isochronous_ ? USBFS_UEP_R_RES_NONE : USBFS_UEP_R_RES_ACK) |
            (*addr & (~USBFS_UEP_R_RES_MASK));
  }
  else
  {
    // EP0 OUT：数据阶段按 tog_ 交替（SETUP 后为 DATA1，之后逐包翻转）；状态
    // 阶段（长度 0）固定 DATA1。多包控制写数据阶段第二包是 DATA0，若每包都按
    // DATA1 挂接收会全部被丢。
    // EP0 OUT: the data stage alternates per tog_ (DATA1 after SETUP, flipping
    // every packet); the status stage (length 0) is always DATA1. The second
    // packet of a multi-packet control write is DATA0, so arming DATA1 for every
    // packet would drop them all.
    const bool data1 = (request.length == 0U) ? true : tog_;
    *addr = USBFS_UEP_R_RES_ACK | (data1 ? USBFS_UEP_R_TOG : 0);
  }
  return ErrorCode::OK;
}

USB::Endpoint::StopResult CH32EndpointOtgFs::StopHardware(bool /*in_isr*/)
{
  // RES=NAK 立即生效：控制器从下一个事务起不再访问槽；在途的最后一个事务仍会
  // 完成（最多一个包落进槽），此后 INT_BUSY=1 自动 NAK 全部事务。无法也无须
  // 中止在途事务，迟到的完成由核心按标签丢弃（与 FSDEV 先例一致）。
  if (GetDirection() == Direction::IN)
  {
    *get_tx_ctrl_addr(GetNumber()) =
        (*get_tx_ctrl_addr(GetNumber()) & ~USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_RES_NAK;
  }
  else
  {
    *get_rx_ctrl_addr(GetNumber()) =
        (*get_rx_ctrl_addr(GetNumber()) & ~USBFS_UEP_R_RES_MASK) | USBFS_UEP_R_RES_NAK;
  }
  transfer_active_ = false;
  return StopResult::DONE;
}

size_t CH32EndpointOtgFs::ReapProgress(const HwRequest& request, bool& retired)
{
  // 只有硬件真正报过完成，才能认为主机把数据都取走了（与 STM32 OTG 的 XFRC
  // 判据同口径）。否则 retired 保持 false：IN 报 0（没发出去的不能算成功），
  // OUT 报已经落进槽的字节数（这些数据确实收到了）。
  // Only a real hardware completion proves the host took everything (same
  // criterion as the STM32 OTG XFRC check). Otherwise retired stays false:
  // IN reports 0 (unsent bytes must not count as success) and OUT reports the
  // bytes already in the slot (those really were received).
  if (hw_completed_)
  {
    retired = true;
    return request.length;
  }
  retired = false;
  return (GetDirection() == Direction::OUT) ? transfer_accumulated_ : 0U;
}

ErrorCode CH32EndpointOtgFs::StallHardware(bool /*in_isr*/)
{
  if (GetDirection() == Direction::IN)
  {
    *get_tx_ctrl_addr(GetNumber()) |= USBFS_UEP_T_RES_STALL;
  }
  else
  {
    *get_rx_ctrl_addr(GetNumber()) |= USBFS_UEP_R_RES_STALL;
  }
  return ErrorCode::OK;
}

ErrorCode CH32EndpointOtgFs::ClearStallHardware(bool /*in_isr*/)
{
  if (GetNumber() == EPNumber::EP0)
  {
    // 下一个 SETUP 已在中断里把 EP0 恢复为 NAK/DATA1；这里再复位 toggle 会让数据阶段
    // 首包变成 DATA0。 / The next SETUP has already restored EP0 to NAK/DATA1 in the
    // interrupt; resetting the toggle here would send the first data packet as DATA0.
    return ErrorCode::OK;
  }
  if (GetDirection() == Direction::IN)
    *get_tx_ctrl_addr(GetNumber()) = USBFS_UEP_T_RES_NAK | USBFS_UEP_T_AUTO_TOG;
  else
    *get_rx_ctrl_addr(GetNumber()) = USBFS_UEP_R_RES_NAK | USBFS_UEP_R_AUTO_TOG;
  tog_ = false;
  return ErrorCode::OK;
}

size_t CH32EndpointOtgFs::MaxPacketSizeLimit(Type type) const
{
  // 手册表 23-4（p.359）/EP0（p.355）：EP1-15 除 EP3 外最大 64 B；EP3 仅同步
  // 传输且单缓冲时最大 1023 B；EP0 收发共用 64 B。
  // 手册表 23-4（p.359）：只有 EP3 在同步传输且单缓冲时才支持 >64 的包长，
  // 其余端点（含 EP0）最大 64 B。
  // Manual table 23-4 (p.359): only EP3 in isochronous single-buffer mode
  // supports a packet size above 64; everything else (EP0 included) is 64.
  if (type == Type::ISOCHRONOUS)
  {
    return (GetNumber() == EPNumber::EP3 && single_direction_) ? 1023U : 64U;
  }
  return 64U;
}

void CH32EndpointOtgFs::TransferComplete(size_t size)
{
  // 停止后/新请求挂载前到达的迟到完成：不该再动寄存器，也不是本请求的完成，
  // 直接丢弃（核心按标签丢弃，这里守住硬件侧）。
  // A completion arriving after a stop or before the next request is armed must
  // not touch the registers and is not a completion of the current request; drop
  // it (the core also drops it by tag, this guards the hardware side).
  if (!transfer_active_)
  {
    return;
  }

  const bool IS_IN = (GetDirection() == Direction::IN);
  const bool IS_EP0 = (GetNumber() == EPNumber::EP0);
  const bool IS_ISO = (GetType() == Type::ISOCHRONOUS);

  // UIF_TRANSFER/INT_FG 会在 IRQ handler 完成分发后统一清掉。

  if (IS_IN)
  {
    // 还有后续分包：续发下一包，整请求发完才向核心报一次完成。
    // More packets to go: send the next one and report a single completion for
    // the whole request once everything is out.
    if (transfer_remaining_ > 0U)
    {
      const size_t mps = MaxPacketSize();
      const size_t chunk =
          (mps > 0U && transfer_remaining_ > mps) ? mps : transfer_remaining_;
      if (single_direction_ || IS_EP0)
      {
        set_dma_buffer(GetNumber(), transfer_cursor_, false);
      }
      else
      {
        // 共享模式只有 64 B 的 IN 区，请求长度不会超过一个包；走到这里说明
        // 配置有误，按剩余字节整包发出，不再续发。
        // Shared mode only has a 64 B IN area, so a request never exceeds one
        // packet; reaching here means a misconfiguration — send what is left as
        // one packet and stop splitting.
        transfer_remaining_ = 0U;
      }
      if (transfer_remaining_ > 0U)
      {
        set_tx_len(GetNumber(), chunk);
        transfer_cursor_ += chunk;
        transfer_remaining_ -= chunk;
        *get_tx_ctrl_addr(GetNumber()) =
            (*get_tx_ctrl_addr(GetNumber()) & ~USBFS_UEP_T_RES_MASK) |
            (IS_ISO ? USBFS_UEP_T_RES_NONE : USBFS_UEP_T_RES_ACK);
        return;
      }
      set_tx_len(GetNumber(), chunk);
    }
    // 完成后恢复 NAK；IN 一次请求一次完成，actual = 请求长度。
    *get_tx_ctrl_addr(GetNumber()) =
        (*get_tx_ctrl_addr(GetNumber()) & ~USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_RES_NAK;
    transfer_active_ = false;
    transfer_remaining_ = 0U;
    hw_completed_ = true;
    OnHardwareComplete(true, transfer_tag_, transfer_requested_);
    return;
  }

  // TOG 不匹配表示数据同步失败：丢弃本包并按原请求重新挂接收。
  if (!IS_ISO)
  {
    const bool TOG_OK = ((USBFSD->INT_ST & USBFS_U_TOG_OK) == USBFS_U_TOG_OK);  // NOLINT
    if (!TOG_OK)
    {
      // 重试只补收没到的部分：单向模式可能已经 ACK 了前几包，整请求重来会丢掉
      // 它们（shared/EP0/ISO 每包完成，remaining == requested，行为不变）。
      // Retry only the missing part: in single-direction mode earlier packets may
      // already be ACKed and a full restart would drop them (shared/EP0/ISO
      // complete per packet, so remaining == requested and nothing changes).
      HwRequest retry{};
      const size_t remaining = transfer_requested_ - transfer_accumulated_;
      retry.slot = LibXR::RawData(transfer_cursor_, remaining);
      retry.length = remaining;
      retry.tag = transfer_tag_;
      (void)StartHardware(retry, true);
      return;
    }
  }

  if (!IS_EP0 && single_direction_ && !IS_ISO)
  {
    // 单向：按包推进 DMA，累计到请求长度或收到短包才一次完成。
    transfer_accumulated_ += size;
    if (transfer_accumulated_ < transfer_requested_ && size == MaxPacketSize())
    {
      transfer_cursor_ += size;
      *get_dma_addr(GetNumber()) = reinterpret_cast<uint32_t>(transfer_cursor_);
      return;  // RES 仍为 ACK，继续收下一包
    }
    *get_rx_ctrl_addr(GetNumber()) =
        (*get_rx_ctrl_addr(GetNumber()) & ~USBFS_UEP_R_RES_MASK) | USBFS_UEP_R_RES_NAK;
    transfer_active_ = false;
    hw_completed_ = true;
    OnHardwareComplete(true, transfer_tag_, transfer_accumulated_);
    return;
  }

  // 共享模式（含 EP0 与 ISO）：每包完成，actual = RX_LEN，不推进 DMA；包间由
  // INT_BUSY 自动 NAK 挡住，类消费槽后重新挂接收。
  if (!IS_EP0)
  {
    *get_rx_ctrl_addr(GetNumber()) =
        (*get_rx_ctrl_addr(GetNumber()) & ~USBFS_UEP_R_RES_MASK) | USBFS_UEP_R_RES_NAK;
  }
  transfer_accumulated_ = size;
  transfer_active_ = false;
  hw_completed_ = true;
  if (IS_EP0)
  {
    // 数据阶段的包按 DATA1/DATA0 交替：完成一次翻转一次。状态阶段（长度 0）
    // 由 StartHardware 强制 DATA1，不依赖这里的值。
    // Data-stage packets alternate DATA1/DATA0: flip per completion. The status
    // stage (length 0) is forced to DATA1 by StartHardware and does not depend
    // on this value.
    tog_ = !tog_;
  }
  OnHardwareComplete(true, transfer_tag_, size);
}

#endif
