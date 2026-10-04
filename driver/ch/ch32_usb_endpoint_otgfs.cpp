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

  if (!double_buffer)
  {
    return;
  }

  switch (ep_num)
  {
    case USB::Endpoint::EPNumber::EP1:
      USBFSD->UEP4_1_MOD |= USBFS_UEP1_BUF_MOD;
      break;
    case USB::Endpoint::EPNumber::EP2:
      USBFSD->UEP2_3_MOD |= USBFS_UEP2_BUF_MOD;
      break;
    case USB::Endpoint::EPNumber::EP3:
      USBFSD->UEP2_3_MOD |= USBFS_UEP3_BUF_MOD;
      break;
    case USB::Endpoint::EPNumber::EP4:
      USBFSD->UEP4_1_MOD |= USBFS_UEP4_BUF_MOD;
      break;
    case USB::Endpoint::EPNumber::EP5:
      USBFSD->UEP5_6_MOD |= USBFS_UEP5_BUF_MOD;
      break;
    case USB::Endpoint::EPNumber::EP6:
      USBFSD->UEP5_6_MOD |= USBFS_UEP6_BUF_MOD;
      break;
    case USB::Endpoint::EPNumber::EP7:
      USBFSD->UEP7_MOD |= USBFS_UEP7_BUF_MOD;
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

  const size_t half = buffer.size_ / 2u;
  ASSERT(half > 0u);

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
                   : select_buffer(ep_num, dir, buffer)),
      single_direction_(single_direction),
      dma_buffer_(buffer)
{
  map_otg_fs_[EPNumberToInt8(GetNumber())][static_cast<uint8_t>(dir)] = this;

  set_dma_buffer(GetNumber(), dma_buffer_.addr_, false);

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

void CH32EndpointOtgFs::Configure(const Config& cfg)
{
  auto& ep_cfg = GetConfig();
  ep_cfg = cfg;
  is_isochronous_ = (cfg.type == Type::ISOCHRONOUS);

  const bool is_ep0 = (GetNumber() == EPNumber::EP0);
  const bool is_bidir_noniso = !is_ep0 && !single_direction_ && !is_isochronous_;
  const bool is_single_noniso = !is_ep0 && single_direction_ && !is_isochronous_;
  const uint16_t type_limit = is_isochronous_ ? 1023u : 64u;
  const uint16_t requested_mps = LibXR::min<uint16_t>(cfg.max_packet_size, type_limit);

  if (is_bidir_noniso)
  {
    // 当前共享双向非等时 OTGFS 路径现在只按方向二分原始内存。
    // Current shared bidirectional non-iso OTGFS path only splits raw memory by
    // direction.
    ASSERT(dma_buffer_.size_ >= static_cast<size_t>(requested_mps) * 2u);
  }

  ep_cfg.double_buffer = is_single_noniso;

  // OTGFS 包长同时受请求值、当前有效缓冲区和 USB FS 类型上限约束。
  // OTGFS MPS is clamped by request, effective buffer, and USB FS type limit.
  ep_cfg.max_packet_size = LibXR::min<uint16_t>(
      requested_mps,
      LibXR::min<uint16_t>(static_cast<uint16_t>(GetBuffer().size_), type_limit));

  set_tx_len(GetNumber(), 0);

  if (!is_isochronous_)
  {
    *get_rx_ctrl_addr(GetNumber()) = USBFS_UEP_R_RES_NAK | USBFS_UEP_R_AUTO_TOG;
    *get_tx_ctrl_addr(GetNumber()) = USBFS_UEP_T_RES_NAK | USBFS_UEP_T_AUTO_TOG;
    enable_tx(GetNumber());
    enable_rx(GetNumber());
  }
  else
  {
    *get_rx_ctrl_addr(GetNumber()) = USBFS_UEP_R_RES_NAK;
    *get_tx_ctrl_addr(GetNumber()) = USBFS_UEP_T_RES_NAK;
    if (GetDirection() == Direction::IN)
    {
      enable_tx(GetNumber());
    }
    else
    {
      enable_rx(GetNumber());
    }
  }

  set_dma_buffer(GetNumber(), dma_buffer_.addr_, ep_cfg.double_buffer);

  SetState(State::IDLE);
}

void CH32EndpointOtgFs::Close()
{
  disable_tx(GetNumber());
  disable_rx(GetNumber());

  *get_tx_ctrl_addr(GetNumber()) = USBFS_UEP_T_RES_NAK;
  *get_rx_ctrl_addr(GetNumber()) = USBFS_UEP_R_RES_NAK;

  SetState(State::DISABLED);
}

ErrorCode CH32EndpointOtgFs::Transfer(size_t size)
{
  if (GetState() == State::BUSY)
  {
    return ErrorCode::BUSY;
  }

  auto buffer = GetBuffer();
  if (buffer.size_ < size)
  {
    return ErrorCode::NO_BUFF;
  }

  bool is_in = (GetDirection() == Direction::IN);

  if (is_in && UseDoubleBuffer())
  {
    SwitchBuffer();
  }

  if (is_in)
  {
    set_tx_len(GetNumber(), size);
    auto addr = get_tx_ctrl_addr(GetNumber());

    if (GetNumber() != EPNumber::EP0)
    {
      *addr = (is_isochronous_ ? USBFS_UEP_T_RES_NONE : USBFS_UEP_T_RES_ACK) |
              (*addr & (~USBFS_UEP_T_RES_MASK));
    }
    else
    {
      *addr = USBFS_UEP_T_RES_ACK | (tog_ ? USBFS_UEP_T_TOG : 0);
    }
  }
  else
  {
    auto addr = get_rx_ctrl_addr(GetNumber());

    if (GetNumber() != EPNumber::EP0)
    {
      *addr = (is_isochronous_ ? USBFS_UEP_R_RES_NONE : USBFS_UEP_R_RES_ACK) |
              (*addr & (~USBFS_UEP_R_RES_MASK));
    }
    else
    {
      *addr = USBFS_UEP_R_RES_ACK | (tog_ ? USBFS_UEP_R_TOG : 0);
    }
  }

  if (GetNumber() == EPNumber::EP0)
  {
    tog_ = !tog_;
  }

  last_transfer_size_ = size;
  SetState(State::BUSY);
  return ErrorCode::OK;
}

ErrorCode CH32EndpointOtgFs::Stall()
{
  const bool is_in = (GetDirection() == Direction::IN);
  if (GetState() != State::IDLE && !(GetState() == State::BUSY && !is_in))
  {
    return ErrorCode::BUSY;
  }

  if (is_in)
  {
    *get_tx_ctrl_addr(GetNumber()) |= USBFS_UEP_T_RES_STALL;
  }
  else
  {
    *get_rx_ctrl_addr(GetNumber()) |= USBFS_UEP_R_RES_STALL;
  }
  SetState(State::STALLED);
  return ErrorCode::OK;
}

ErrorCode CH32EndpointOtgFs::ClearStall()
{
  if (GetState() != State::STALLED)
  {
    return ErrorCode::FAILED;
  }

  bool is_in = (GetDirection() == Direction::IN);
  if (is_in)
  {
    *get_tx_ctrl_addr(GetNumber()) &= ~USBFS_UEP_T_RES_STALL;
  }
  else
  {
    *get_rx_ctrl_addr(GetNumber()) &= ~USBFS_UEP_R_RES_STALL;
  }
  SetState(State::IDLE);
  return ErrorCode::OK;
}

void CH32EndpointOtgFs::TransferComplete(size_t size)
{
  const bool IS_IN = (GetDirection() == Direction::IN);
  const bool IS_OUT = !IS_IN;
  const bool IS_EP0 = (GetNumber() == EPNumber::EP0);
  const bool IS_ISO = (GetType() == Type::ISOCHRONOUS);

  // UIF_TRANSFER/INT_FG 会在 IRQ handler 完成分发后统一清掉。
  // UIF_TRANSFER/INT_FG are cleared by the IRQ handler after dispatch.

  if (IS_IN)
  {
    // 完成后恢复到 NAK。
    // Restore NAK on completion.
    *get_tx_ctrl_addr(GetNumber()) =
        (*get_tx_ctrl_addr(GetNumber()) & ~USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_RES_NAK;

    size = last_transfer_size_;
  }
  else
  {
    // 对 non-EP0 OUT 端点，完成后恢复到 NAK。
    // For non-EP0 OUT endpoints, restore NAK on completion.
    if (!IS_EP0)
    {
      *get_rx_ctrl_addr(GetNumber()) =
          (*get_rx_ctrl_addr(GetNumber()) & ~USBFS_UEP_R_RES_MASK) | USBFS_UEP_R_RES_NAK;
    }
  }

  // TOG 不匹配表示数据同步失败。
  // TOG mismatch indicates data synchronization failure.
  if (IS_OUT)
  {
    const bool TOG_OK = ((USBFSD->INT_ST & USBFS_U_TOG_OK) == USBFS_U_TOG_OK);  // NOLINT
    if (!TOG_OK)
    {
      SetState(State::IDLE);
      (void)Transfer(last_transfer_size_);
      return;
    }
  }

  // 对 non-EP0、non-ISO 端点推进软件侧 data toggle。
  // Update the software data toggle for non-EP0 non-ISO endpoints.
  if (GetState() == State::BUSY && !IS_EP0 && !IS_ISO)
  {
    tog_ = !tog_;
  }

  if (IS_EP0 && IS_OUT)
  {
    tog_ = true;
    *get_rx_ctrl_addr(GetNumber()) = USBFS_UEP_R_RES_ACK;
  }

  OnTransferCompleteCallback(true, size);
}

void CH32EndpointOtgFs::SwitchBuffer()
{
  if (GetDirection() == Direction::IN)
  {
    tog_ = (*get_tx_ctrl_addr(GetNumber()) & USBFS_UEP_T_TOG) == USBFS_UEP_T_TOG;
    SetActiveBlock(!tog_);
  }
  else
  {
    tog_ = (*get_rx_ctrl_addr(GetNumber()) & USBFS_UEP_R_TOG) == USBFS_UEP_R_TOG;
    SetActiveBlock(tog_);
  }
}

#endif
