#include <cstdint>

#include "ch32_usb_endpoint.hpp"
#include "ep.hpp"
#include "libxr_time.hpp"
#include "timebase.hpp"

using namespace LibXR;

#if defined(USBHSD)

// NOLINTBEGIN
static inline volatile uint8_t* get_tx_control_addr(USB::Endpoint::EPNumber ep_num)
{
  return reinterpret_cast<volatile uint8_t*>(
      reinterpret_cast<volatile uint8_t*>(&USBHSD->UEP0_TX_CTRL) +
      static_cast<int>(ep_num) * 4);
}

static inline volatile uint8_t* get_rx_control_addr(USB::Endpoint::EPNumber ep_num)
{
  return reinterpret_cast<volatile uint8_t*>(
      reinterpret_cast<volatile uint8_t*>(&USBHSD->UEP0_TX_CTRL) +
      static_cast<int>(ep_num) * 4 + 1);
}

static inline volatile uint16_t* get_tx_len_addr(USB::Endpoint::EPNumber ep_num)
{
  return reinterpret_cast<volatile uint16_t*>(
      reinterpret_cast<volatile uint8_t*>(&USBHSD->UEP0_TX_LEN) +
      static_cast<int>(ep_num) * 4);
}

static_assert(offsetof(USBHSD_TypeDef, UEP1_MAX_LEN) -
                  offsetof(USBHSD_TypeDef, UEP0_MAX_LEN) ==
              4);

static inline volatile uint16_t* get_rx_max_len_addr(USB::Endpoint::EPNumber ep_num)
{
  return reinterpret_cast<volatile uint16_t*>(
      reinterpret_cast<volatile uint8_t*>(&USBHSD->UEP0_MAX_LEN) +
      static_cast<int>(ep_num) * 4);
}

static inline volatile uint32_t* get_tx_dma_addr(USB::Endpoint::EPNumber ep_num)
{
  if (ep_num == USB::Endpoint::EPNumber::EP0)
  {
    return &USBHSD->UEP0_DMA;
  }
  return reinterpret_cast<volatile uint32_t*>(
      reinterpret_cast<volatile uint32_t*>(&USBHSD->UEP1_TX_DMA) +
      (static_cast<int>(ep_num) - 1));
}

static inline volatile uint32_t* get_rx_dma_addr(USB::Endpoint::EPNumber ep_num)
{
  if (ep_num == USB::Endpoint::EPNumber::EP0)
  {
    return &USBHSD->UEP0_DMA;
  }
  return reinterpret_cast<volatile uint32_t*>(
      reinterpret_cast<volatile uint32_t*>(&USBHSD->UEP1_RX_DMA) +
      (static_cast<int>(ep_num) - 1));
}

static void set_tx_len(USB::Endpoint::EPNumber ep_num, uint32_t value)
{
  *get_tx_len_addr(ep_num) = value;
}

static void set_tx_dma_buffer(USB::Endpoint::EPNumber ep_num, void* buffer,
                              uint32_t buffer_size, bool double_buffer)
{
  uint8_t* buf_base = reinterpret_cast<uint8_t*>(buffer);
  uint8_t* buf_alt = buf_base + buffer_size / 2;

  if (ep_num == USB::Endpoint::EPNumber::EP0)
  {
    USBHSD->UEP0_DMA = (uint32_t)buf_base;
  }
  else
  {
    *get_tx_dma_addr(ep_num) = (uint32_t)buf_base;
    if (double_buffer)
    {
      *get_rx_dma_addr(ep_num) = (uint32_t)buf_alt;
    }
  }

  int IDX = static_cast<int>(ep_num);
  if (double_buffer && ep_num != USB::Endpoint::EPNumber::EP0)
  {
    USBHSD->BUF_MODE |= (1 << IDX);
  }
  else
  {
    USBHSD->BUF_MODE &= ~(1 << IDX);
  }
}

static void set_rx_dma_buffer(USB::Endpoint::EPNumber ep_num, void* buffer,
                              uint32_t buffer_size, bool double_buffer)
{
  uint8_t* buf_base = reinterpret_cast<uint8_t*>(buffer);
  uint8_t* buf_alt = buf_base + buffer_size / 2;

  if (ep_num == USB::Endpoint::EPNumber::EP0)
  {
    USBHSD->UEP0_DMA = (uint32_t)buf_base;
  }
  else
  {
    *get_rx_dma_addr(ep_num) = (uint32_t)buf_base;
    if (double_buffer)
    {
      *get_tx_dma_addr(ep_num) = (uint32_t)buf_alt;
    }
  }

  int IDX = static_cast<int>(ep_num);
  if (double_buffer && ep_num != USB::Endpoint::EPNumber::EP0)
  {
    USBHSD->BUF_MODE |= (1 << IDX);
  }
  else
  {
    USBHSD->BUF_MODE &= ~(1 << IDX);
  }
}

static void enable_tx(USB::Endpoint::EPNumber ep_num)
{
  switch (ep_num)
  {
    case USB::Endpoint::EPNumber::EP0:
      USBHSD->ENDP_CONFIG |= USBHS_UEP0_T_EN;
      break;
    case USB::Endpoint::EPNumber::EP1:
      USBHSD->ENDP_CONFIG |= USBHS_UEP1_T_EN;
      break;
    case USB::Endpoint::EPNumber::EP2:
      USBHSD->ENDP_CONFIG |= USBHS_UEP2_T_EN;
      break;
    case USB::Endpoint::EPNumber::EP3:
      USBHSD->ENDP_CONFIG |= USBHS_UEP3_T_EN;
      break;
    case USB::Endpoint::EPNumber::EP4:
      USBHSD->ENDP_CONFIG |= USBHS_UEP4_T_EN;
      break;
    case USB::Endpoint::EPNumber::EP5:
      USBHSD->ENDP_CONFIG |= USBHS_UEP5_T_EN;
      break;
    case USB::Endpoint::EPNumber::EP6:
      USBHSD->ENDP_CONFIG |= USBHS_UEP6_T_EN;
      break;
    case USB::Endpoint::EPNumber::EP7:
      USBHSD->ENDP_CONFIG |= USBHS_UEP7_T_EN;
      break;
    case USB::Endpoint::EPNumber::EP8:
      USBHSD->ENDP_CONFIG |= USBHS_UEP8_T_EN;
      break;
    case USB::Endpoint::EPNumber::EP9:
      USBHSD->ENDP_CONFIG |= USBHS_UEP9_T_EN;
      break;
    case USB::Endpoint::EPNumber::EP10:
      USBHSD->ENDP_CONFIG |= USBHS_UEP10_T_EN;
      break;
    case USB::Endpoint::EPNumber::EP11:
      USBHSD->ENDP_CONFIG |= USBHS_UEP11_T_EN;
      break;
    case USB::Endpoint::EPNumber::EP12:
      USBHSD->ENDP_CONFIG |= USBHS_UEP12_T_EN;
      break;
    case USB::Endpoint::EPNumber::EP13:
      USBHSD->ENDP_CONFIG |= USBHS_UEP13_T_EN;
      break;
    case USB::Endpoint::EPNumber::EP14:
      USBHSD->ENDP_CONFIG |= USBHS_UEP14_T_EN;
      break;
    case USB::Endpoint::EPNumber::EP15:
      USBHSD->ENDP_CONFIG |= USBHS_UEP15_T_EN;
      break;
    default:
      break;
  }
}

static void disable_tx(USB::Endpoint::EPNumber ep_num)
{
  switch (ep_num)
  {
    case USB::Endpoint::EPNumber::EP0:
      USBHSD->ENDP_CONFIG &= ~USBHS_UEP0_T_EN;
      break;
    case USB::Endpoint::EPNumber::EP1:
      USBHSD->ENDP_CONFIG &= ~USBHS_UEP1_T_EN;
      break;
    case USB::Endpoint::EPNumber::EP2:
      USBHSD->ENDP_CONFIG &= ~USBHS_UEP2_T_EN;
      break;
    case USB::Endpoint::EPNumber::EP3:
      USBHSD->ENDP_CONFIG &= ~USBHS_UEP3_T_EN;
      break;
    case USB::Endpoint::EPNumber::EP4:
      USBHSD->ENDP_CONFIG &= ~USBHS_UEP4_T_EN;
      break;
    case USB::Endpoint::EPNumber::EP5:
      USBHSD->ENDP_CONFIG &= ~USBHS_UEP5_T_EN;
      break;
    case USB::Endpoint::EPNumber::EP6:
      USBHSD->ENDP_CONFIG &= ~USBHS_UEP6_T_EN;
      break;
    case USB::Endpoint::EPNumber::EP7:
      USBHSD->ENDP_CONFIG &= ~USBHS_UEP7_T_EN;
      break;
    case USB::Endpoint::EPNumber::EP8:
      USBHSD->ENDP_CONFIG &= ~USBHS_UEP8_T_EN;
      break;
    case USB::Endpoint::EPNumber::EP9:
      USBHSD->ENDP_CONFIG &= ~USBHS_UEP9_T_EN;
      break;
    case USB::Endpoint::EPNumber::EP10:
      USBHSD->ENDP_CONFIG &= ~USBHS_UEP10_T_EN;
      break;
    case USB::Endpoint::EPNumber::EP11:
      USBHSD->ENDP_CONFIG &= ~USBHS_UEP11_T_EN;
      break;
    case USB::Endpoint::EPNumber::EP12:
      USBHSD->ENDP_CONFIG &= ~USBHS_UEP12_T_EN;
      break;
    case USB::Endpoint::EPNumber::EP13:
      USBHSD->ENDP_CONFIG &= ~USBHS_UEP13_T_EN;
      break;
    case USB::Endpoint::EPNumber::EP14:
      USBHSD->ENDP_CONFIG &= ~USBHS_UEP14_T_EN;
      break;
    case USB::Endpoint::EPNumber::EP15:
      USBHSD->ENDP_CONFIG &= ~USBHS_UEP15_T_EN;
      break;
    default:
      break;
  }
}

static void enable_rx(USB::Endpoint::EPNumber ep_num)
{
  switch (ep_num)
  {
    case USB::Endpoint::EPNumber::EP0:
      USBHSD->ENDP_CONFIG |= USBHS_UEP0_R_EN;
      break;
    case USB::Endpoint::EPNumber::EP1:
      USBHSD->ENDP_CONFIG |= USBHS_UEP1_R_EN;
      break;
    case USB::Endpoint::EPNumber::EP2:
      USBHSD->ENDP_CONFIG |= USBHS_UEP2_R_EN;
      break;
    case USB::Endpoint::EPNumber::EP3:
      USBHSD->ENDP_CONFIG |= USBHS_UEP3_R_EN;
      break;
    case USB::Endpoint::EPNumber::EP4:
      USBHSD->ENDP_CONFIG |= USBHS_UEP4_R_EN;
      break;
    case USB::Endpoint::EPNumber::EP5:
      USBHSD->ENDP_CONFIG |= USBHS_UEP5_R_EN;
      break;
    case USB::Endpoint::EPNumber::EP6:
      USBHSD->ENDP_CONFIG |= USBHS_UEP6_R_EN;
      break;
    case USB::Endpoint::EPNumber::EP7:
      USBHSD->ENDP_CONFIG |= USBHS_UEP7_R_EN;
      break;
    case USB::Endpoint::EPNumber::EP8:
      USBHSD->ENDP_CONFIG |= USBHS_UEP8_R_EN;
      break;
    case USB::Endpoint::EPNumber::EP9:
      USBHSD->ENDP_CONFIG |= USBHS_UEP9_R_EN;
      break;
    case USB::Endpoint::EPNumber::EP10:
      USBHSD->ENDP_CONFIG |= USBHS_UEP10_R_EN;
      break;
    case USB::Endpoint::EPNumber::EP11:
      USBHSD->ENDP_CONFIG |= USBHS_UEP11_R_EN;
      break;
    case USB::Endpoint::EPNumber::EP12:
      USBHSD->ENDP_CONFIG |= USBHS_UEP12_R_EN;
      break;
    case USB::Endpoint::EPNumber::EP13:
      USBHSD->ENDP_CONFIG |= USBHS_UEP13_R_EN;
      break;
    case USB::Endpoint::EPNumber::EP14:
      USBHSD->ENDP_CONFIG |= USBHS_UEP14_R_EN;
      break;
    case USB::Endpoint::EPNumber::EP15:
      USBHSD->ENDP_CONFIG |= USBHS_UEP15_R_EN;
      break;
    default:
      break;
  }
}

static void disable_rx(USB::Endpoint::EPNumber ep_num)
{
  switch (ep_num)
  {
    case USB::Endpoint::EPNumber::EP0:
      USBHSD->ENDP_CONFIG &= ~USBHS_UEP0_R_EN;
      break;
    case USB::Endpoint::EPNumber::EP1:
      USBHSD->ENDP_CONFIG &= ~USBHS_UEP1_R_EN;
      break;
    case USB::Endpoint::EPNumber::EP2:
      USBHSD->ENDP_CONFIG &= ~USBHS_UEP2_R_EN;
      break;
    case USB::Endpoint::EPNumber::EP3:
      USBHSD->ENDP_CONFIG &= ~USBHS_UEP3_R_EN;
      break;
    case USB::Endpoint::EPNumber::EP4:
      USBHSD->ENDP_CONFIG &= ~USBHS_UEP4_R_EN;
      break;
    case USB::Endpoint::EPNumber::EP5:
      USBHSD->ENDP_CONFIG &= ~USBHS_UEP5_R_EN;
      break;
    case USB::Endpoint::EPNumber::EP6:
      USBHSD->ENDP_CONFIG &= ~USBHS_UEP6_R_EN;
      break;
    case USB::Endpoint::EPNumber::EP7:
      USBHSD->ENDP_CONFIG &= ~USBHS_UEP7_R_EN;
      break;
    case USB::Endpoint::EPNumber::EP8:
      USBHSD->ENDP_CONFIG &= ~USBHS_UEP8_R_EN;
      break;
    case USB::Endpoint::EPNumber::EP9:
      USBHSD->ENDP_CONFIG &= ~USBHS_UEP9_R_EN;
      break;
    case USB::Endpoint::EPNumber::EP10:
      USBHSD->ENDP_CONFIG &= ~USBHS_UEP10_R_EN;
      break;
    case USB::Endpoint::EPNumber::EP11:
      USBHSD->ENDP_CONFIG &= ~USBHS_UEP11_R_EN;
      break;
    case USB::Endpoint::EPNumber::EP12:
      USBHSD->ENDP_CONFIG &= ~USBHS_UEP12_R_EN;
      break;
    case USB::Endpoint::EPNumber::EP13:
      USBHSD->ENDP_CONFIG &= ~USBHS_UEP13_R_EN;
      break;
    case USB::Endpoint::EPNumber::EP14:
      USBHSD->ENDP_CONFIG &= ~USBHS_UEP14_R_EN;
      break;
    case USB::Endpoint::EPNumber::EP15:
      USBHSD->ENDP_CONFIG &= ~USBHS_UEP15_R_EN;
      break;
    default:
      break;
  }
}
// NOLINTEND

CH32EndpointOtgHs::CH32EndpointOtgHs(EPNumber ep_num, Direction dir,
                                     LibXR::RawData buffer, bool double_buffer)
    : Endpoint(ep_num, dir, buffer), hw_double_buffer_(double_buffer), dma_buffer_(buffer)
{
  map_otg_hs_[EPNumberToInt8(GetNumber())][static_cast<uint8_t>(dir)] = this;

  if (dir == Direction::IN)
  {
    set_tx_dma_buffer(GetNumber(), dma_buffer_.addr_, dma_buffer_.size_, double_buffer);
  }
  else
  {
    set_rx_dma_buffer(GetNumber(), dma_buffer_.addr_, dma_buffer_.size_, double_buffer);
  }

  if (dir == Direction::IN)
  {
    set_tx_len(GetNumber(), 0);
    *get_tx_control_addr(GetNumber()) = USBHS_UEP_T_RES_NAK;
  }
  else
  {
    *get_rx_control_addr(GetNumber()) = USBHS_UEP_R_RES_NAK;
  }
}

void CH32EndpointOtgHs::Configure(const Config& cfg)
{
  auto& ep_cfg = GetConfig();
  ep_cfg = cfg;

  const int EP_IDX = EPNumberToInt8(GetNumber());

  const uint8_t IN_IDX = static_cast<uint8_t>(Direction::IN);
  const uint8_t OUT_IDX = static_cast<uint8_t>(Direction::OUT);

  const bool HAS_IN = (map_otg_hs_[EP_IDX][IN_IDX] != nullptr);
  const bool HAS_OUT = (map_otg_hs_[EP_IDX][OUT_IDX] != nullptr);

  // 双缓冲策略：EP0 不使用 double buffer。
  // Double-buffer policy: EP0 does not use double buffering.
  bool enable_double = (GetNumber() != EPNumber::EP0) && hw_double_buffer_;
  if (enable_double && HAS_IN && HAS_OUT)
  {
    ASSERT(false);  // 双缓冲端点必须是单向
                    // / Double-buffer endpoints must be single-direction.
    enable_double = false;
  }
  ep_cfg.double_buffer = enable_double;

  const size_t buffer_size = GetBuffer().size_;
  if (ep_cfg.type == Type::BULK)
  {
    // CH32 USBHS 的 BULK 端点始终使用硬件固定的 512 字节包长。
    // CH32 USBHS bulk endpoints always use the hardware-fixed 512-byte packet size.
    ASSERT(buffer_size >= 512u);
    ep_cfg.max_packet_size = 512u;
  }
  else if (ep_cfg.max_packet_size > buffer_size)
  {
    // 非 BULK 端点的包长按缓冲区大小截断。
    // Clamp non-bulk packet sizes to the endpoint buffer size.
    ep_cfg.max_packet_size = buffer_size;
  }

  if (GetDirection() == Direction::IN)
  {
    if (GetType() != Type::ISOCHRONOUS && GetNumber() != EPNumber::EP0)
    {
      *get_tx_control_addr(GetNumber()) = USBHS_UEP_T_RES_NAK | USBHS_UEP_T_TOG_AUTO;
    }
    else
    {
      *get_tx_control_addr(GetNumber()) = USBHS_UEP_T_RES_NAK;
    }
    set_tx_len(GetNumber(), 0);
  }
  else
  {
    if (GetType() != Type::ISOCHRONOUS && GetNumber() != EPNumber::EP0)
    {
      *get_rx_control_addr(GetNumber()) = USBHS_UEP_R_RES_NAK | USBHS_UEP_R_TOG_AUTO;
    }
    else
    {
      *get_rx_control_addr(GetNumber()) = USBHS_UEP_R_RES_NAK;
    }

    *get_rx_max_len_addr(GetNumber()) = ep_cfg.max_packet_size;
  }

  const int IDX = static_cast<int>(GetNumber());
  if (GetDirection() == Direction::IN)
  {
    if (GetType() == Type::ISOCHRONOUS)
    {
      USBHSD->ENDP_TYPE |= (USBHS_UEP0_T_TYPE << IDX);
    }
    else
    {
      USBHSD->ENDP_TYPE &= ~(USBHS_UEP0_T_TYPE << IDX);
    }
  }
  else
  {
    if (GetType() == Type::ISOCHRONOUS)
    {
      USBHSD->ENDP_TYPE |= (USBHS_UEP0_R_TYPE << IDX);
    }
    else
    {
      USBHSD->ENDP_TYPE &= ~(USBHS_UEP0_R_TYPE << IDX);
    }
  }

  if (GetDirection() == Direction::IN)
  {
    if (GetType() != Type::ISOCHRONOUS)
    {
      enable_tx(GetNumber());
    }
    else
    {
      disable_tx(GetNumber());
    }

    if (!HAS_OUT)
    {
      disable_rx(GetNumber());
    }

    set_tx_dma_buffer(GetNumber(), dma_buffer_.addr_, dma_buffer_.size_, enable_double);
  }
  else
  {
    enable_rx(GetNumber());

    if (!HAS_IN)
    {
      disable_tx(GetNumber());
    }

    set_rx_dma_buffer(GetNumber(), dma_buffer_.addr_, dma_buffer_.size_, enable_double);
  }

  SetState(State::IDLE);
}

void CH32EndpointOtgHs::Close()
{
  disable_tx(GetNumber());
  disable_rx(GetNumber());

  *get_tx_control_addr(GetNumber()) = USBHS_UEP_T_RES_NAK;
  *get_rx_control_addr(GetNumber()) = USBHS_UEP_R_RES_NAK;

  SetState(State::DISABLED);
}

ErrorCode CH32EndpointOtgHs::Transfer(size_t size)
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

  bool IS_IN = (GetDirection() == Direction::IN);

  if (IS_IN && UseDoubleBuffer() && GetType() != Type::ISOCHRONOUS)
  {
    SwitchBuffer();
  }

  if (GetNumber() == EPNumber::EP0)
  {
    if (!IS_IN && size == 0)
    {
      tog0_ = true;
      tog1_ = false;
    }
  }

  last_transfer_size_ = size;
  SetState(State::BUSY);

  if (IS_IN)
  {
    if (GetType() == Type::ISOCHRONOUS)
    {
      enable_tx(GetNumber());
    }

    set_tx_len(GetNumber(), size);
    auto addr = get_tx_control_addr(GetNumber());

    if (GetType() != Type::ISOCHRONOUS)
    {
      if (GetNumber() != EPNumber::EP0)
      {
        *addr = (*addr & ~USBHS_UEP_T_RES_MASK) | USBHS_UEP_T_RES_ACK;
      }
      else
      {
        *addr = USBHS_UEP_T_RES_ACK |
                (*addr & (~(USBHS_UEP_T_RES_MASK | USBHS_UEP_T_TOG_MDATA))) |
                (tog0_ ? USBHS_UEP_T_TOG_DATA1 : 0);
      }
    }
    else
    {
      *addr = (uint8_t)((*addr & ~(USBHS_UEP_T_RES_MASK | USBHS_UEP_T_TOG_MASK)) |
                        USBHS_UEP_T_TOG_AUTO);
    }
  }
  else
  {
    auto addr = get_rx_control_addr(GetNumber());

    if (GetType() != Type::ISOCHRONOUS)
    {
      if (GetNumber() != EPNumber::EP0)
      {
        *addr = (*addr & ~USBHS_UEP_R_RES_MASK) | USBHS_UEP_R_RES_ACK;
      }
      else
      {
        if (size == 0u)
        {
          // 控制传输的 status OUT 固定从 DATA1 开始。
          // Control status OUT always starts from DATA1.
          *addr = USBHS_UEP_R_RES_ACK |
                  (*addr & (~(USBHS_UEP_R_RES_MASK | USBHS_UEP_R_TOG_MDATA))) |
                  USBHS_UEP_R_TOG_DATA1;
        }
        else
        {
          // EP0 多包 OUT 续挂时，这里只重新打开 ACK，不再用软件重算 DATA0/DATA1。
          // 每个包成功收完后，由完成路径推进一次硬件 RX toggle。
          // For EP0 multi-packet OUT, only reopen ACK here and keep the current
          // hardware DATA0/DATA1 phase; the packet-complete path advances RX toggle.
          *addr = (*addr & ~USBHS_UEP_R_RES_MASK) | USBHS_UEP_R_RES_ACK;
        }
      }
    }
    else
    {
      *addr = USBHS_UEP_R_RES_ACK |
              (*addr & (~(USBHS_UEP_R_RES_MASK | USBHS_UEP_R_TOG_MDATA)));
    }
  }

  if (GetNumber() == EPNumber::EP0 && IS_IN)
  {
    tog0_ = !tog0_;
  }

  return ErrorCode::OK;
}

ErrorCode CH32EndpointOtgHs::Stall()
{
  const bool IS_IN = (GetDirection() == Direction::IN);
  if (GetState() != State::IDLE && !(GetState() == State::BUSY && !IS_IN))
  {
    return ErrorCode::BUSY;
  }

  if (IS_IN)
  {
    *get_tx_control_addr(GetNumber()) |= USBHS_UEP_T_RES_STALL;
  }
  else
  {
    *get_rx_control_addr(GetNumber()) |= USBHS_UEP_R_RES_STALL;
  }
  SetState(State::STALLED);
  return ErrorCode::OK;
}

ErrorCode CH32EndpointOtgHs::ClearStall()
{
  if (GetState() != State::STALLED)
  {
    return ErrorCode::FAILED;
  }

  bool IS_IN = (GetDirection() == Direction::IN);
  if (IS_IN)
  {
    *get_tx_control_addr(GetNumber()) &= ~USBHS_UEP_T_RES_STALL;
  }
  else
  {
    *get_rx_control_addr(GetNumber()) &= ~USBHS_UEP_R_RES_STALL;
  }
  SetState(State::IDLE);
  return ErrorCode::OK;
}

void CH32EndpointOtgHs::TransferComplete(size_t size)
{
  const bool IS_IN = (GetDirection() == Direction::IN);
  const bool IS_OUT = !IS_IN;
  const bool IS_EP0 = (GetNumber() == EPNumber::EP0);
  const bool IS_ISO = (GetType() == Type::ISOCHRONOUS);

  // UIF_TRANSFER/INT_FG 会在 IRQ handler 完成分发后统一清掉。
  // UIF_TRANSFER/INT_FG are cleared by the IRQ handler after dispatch.

  if (IS_IN)
  {
    auto* tx_ctrl = get_tx_control_addr(GetNumber());
    *tx_ctrl = (*tx_ctrl & ~USBHS_UEP_T_RES_MASK) | USBHS_UEP_T_RES_NAK;

    size = last_transfer_size_;

    if (IS_ISO)
    {
      set_tx_len(GetNumber(), 0);
      disable_tx(GetNumber());
    }
  }
  else
  {
    // 对 non-EP0 OUT 端点，完成后恢复到 NAK。
    // For non-EP0 OUT endpoints, restore NAK on completion.
    if (!IS_EP0)
    {
      auto* rx_ctrl = get_rx_control_addr(GetNumber());
      *rx_ctrl = (*rx_ctrl & ~USBHS_UEP_R_RES_MASK) | USBHS_UEP_R_RES_NAK;
    }
  }

  // TOG 不匹配表示数据同步失败。
  // TOG mismatch indicates data synchronization failure.
  if (IS_OUT && !IS_EP0)
  {
    const bool TOG_OK =
        ((USBHSD->INT_ST & USBHS_UIS_TOG_OK) == USBHS_UIS_TOG_OK);  // NOLINT
    if (!TOG_OK)
    {
      SetState(State::IDLE);
      (void)Transfer(last_transfer_size_);
      return;
    }
  }

  if (IS_EP0 && IS_OUT)
  {
    auto* rx_ctrl = get_rx_control_addr(GetNumber());
    // 这里不要直接把 EP0 OUT 留在 ACK。
    // 上层控制传输栈会在“当前包已经被完整消费”后再重新挂接收；
    // 否则下一轮 session 可能撞上陈旧的 EP0 软件状态，导致 DFU 数据块重复或丢失。
    // Do not leave EP0 OUT open here. The upper control stack will re-arm it after the
    // current packet has been fully consumed; otherwise the next session may race stale
    // EP0 software state and duplicate/skip DFU payload blocks.
    if (size > 0u)
    {
      // 每成功收完一个 EP0 OUT 包，都要推进一次 RX DATA0/DATA1 相位。
      // CH32 USBHS 在多包 EP0 OUT 上，如果这里只是重新开 ACK 而不消费当前 toggle，
      // 后续包会停在这里。
      // Advance the RX DATA0/DATA1 phase once per successfully received EP0 OUT packet.
      // On CH32 USBHS, EP0 multi-packet OUT will stall if we only reopen ACK
      // without consuming the current hardware toggle.
      *rx_ctrl = static_cast<uint8_t>(
          ((*rx_ctrl ^ USBHS_UEP_R_TOG_DATA1) & ~USBHS_UEP_R_RES_MASK) |
          USBHS_UEP_R_RES_NAK);
    }
    else
    {
      *rx_ctrl =
          static_cast<uint8_t>((*rx_ctrl & ~USBHS_UEP_R_RES_MASK) | USBHS_UEP_R_RES_NAK);
    }
  }
  OnTransferCompleteCallback(true, size);
}

void CH32EndpointOtgHs::SwitchBuffer()
{
  if (GetDirection() == Direction::IN)
  {
    const auto* tx_ctrl = get_tx_control_addr(GetNumber());
    const bool TOG_IS_DATA1 =
        ((*tx_ctrl & USBHS_UEP_T_TOG_MASK) == USBHS_UEP_T_TOG_DATA1);
    SetActiveBlock(!TOG_IS_DATA1);
  }
  else
  {
    const auto* rx_ctrl = get_rx_control_addr(GetNumber());
    const bool TOG_IS_DATA1 =
        ((*rx_ctrl & USBHS_UEP_R_TOG_MASK) == USBHS_UEP_R_TOG_DATA1);
    SetActiveBlock(TOG_IS_DATA1);
  }
}

#endif
