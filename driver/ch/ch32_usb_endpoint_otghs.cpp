#include <cstdint>

#include "ch32_usb_endpoint.hpp"
#include "ep.hpp"

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

static void set_tx_dma_buffer(USB::Endpoint::EPNumber ep_num, void* buffer)
{
  if (ep_num == USB::Endpoint::EPNumber::EP0)
  {
    USBHSD->UEP0_DMA = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(buffer));
  }
  else
  {
    *get_tx_dma_addr(ep_num) = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(buffer));
  }
  // BUF_MOD 恒 0：硬件双缓冲不再使用，双槽由软件按请求切换 DMA 指针实现。
  // BUF_MOD stays 0: the hardware double buffer is not used; software switches the DMA
  // pointer between the two slots per request.
  USBHSD->BUF_MODE &= ~(1 << static_cast<int>(ep_num));
}

static void set_rx_dma_buffer(USB::Endpoint::EPNumber ep_num, void* buffer)
{
  if (ep_num == USB::Endpoint::EPNumber::EP0)
  {
    USBHSD->UEP0_DMA = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(buffer));
  }
  else
  {
    *get_rx_dma_addr(ep_num) = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(buffer));
  }
  USBHSD->BUF_MODE &= ~(1 << static_cast<int>(ep_num));
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
                                     LibXR::RawData buffer)
    : Endpoint(ep_num, dir, buffer), dma_buffer_(buffer)
{
  map_otg_hs_[EPNumberToInt8(GetNumber())][static_cast<uint8_t>(dir)] = this;

  if (dir == Direction::IN)
  {
    set_tx_dma_buffer(GetNumber(), dma_buffer_.addr_);
    set_tx_len(GetNumber(), 0);
    *get_tx_control_addr(GetNumber()) = USBHS_UEP_T_RES_NAK;
  }
  else
  {
    set_rx_dma_buffer(GetNumber(), dma_buffer_.addr_);
    *get_rx_control_addr(GetNumber()) = USBHS_UEP_R_RES_NAK;
  }
}

ErrorCode CH32EndpointOtgHs::ConfigureHardware(const Config& cfg)
{
  // 包长已由核心按 MaxPacketSizeLimit 收敛；状态由基类管理，这里只编程寄存器。
  // The core has already clamped the packet size to MaxPacketSizeLimit; the state is
  // managed by the base class, so only the registers are programmed here.
  if (IsPlanning())
  {
    return ErrorCode::OK;
  }

  const int ep_idx = EPNumberToInt8(GetNumber());
  const uint8_t in_idx = static_cast<uint8_t>(Direction::IN);
  const uint8_t out_idx = static_cast<uint8_t>(Direction::OUT);
  const bool has_in = map_otg_hs_[ep_idx][in_idx] != nullptr;
  const bool has_out = map_otg_hs_[ep_idx][out_idx] != nullptr;

  if (GetDirection() == Direction::IN)
  {
    if (cfg.type != Type::ISOCHRONOUS && GetNumber() != EPNumber::EP0)
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
    if (cfg.type != Type::ISOCHRONOUS && GetNumber() != EPNumber::EP0)
    {
      *get_rx_control_addr(GetNumber()) = USBHS_UEP_R_RES_NAK | USBHS_UEP_R_TOG_AUTO;
    }
    else
    {
      *get_rx_control_addr(GetNumber()) = USBHS_UEP_R_RES_NAK;
    }

    *get_rx_max_len_addr(GetNumber()) = cfg.max_packet_size;
  }

  const int IDX = ep_idx;
  if (GetDirection() == Direction::IN)
  {
    if (cfg.type == Type::ISOCHRONOUS)
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
    if (cfg.type == Type::ISOCHRONOUS)
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
    if (cfg.type != Type::ISOCHRONOUS)
    {
      enable_tx(GetNumber());
    }
    else
    {
      disable_tx(GetNumber());
    }

    if (!has_out)
    {
      disable_rx(GetNumber());
    }

    set_tx_dma_buffer(GetNumber(), SlotMemory().addr_);
  }
  else
  {
    enable_rx(GetNumber());

    if (!has_in)
    {
      disable_tx(GetNumber());
    }

    set_rx_dma_buffer(GetNumber(), SlotMemory().addr_);
  }

  transfer_active_ = false;
  transfer_tag_ = 0;
  transfer_requested_ = 0;
  transfer_accumulated_ = 0;
  transfer_cursor_ = nullptr;
  return ErrorCode::OK;
}

void CH32EndpointOtgHs::CloseHardware(bool /*in_isr*/)
{
  if (GetDirection() == Direction::IN)
  {
    *get_tx_control_addr(GetNumber()) = USBHS_UEP_T_RES_NAK;
    disable_tx(GetNumber());
  }
  else
  {
    *get_rx_control_addr(GetNumber()) = USBHS_UEP_R_RES_NAK;
    disable_rx(GetNumber());
  }
  transfer_active_ = false;
}

ErrorCode CH32EndpointOtgHs::StartHardware(const HwRequest& request, bool /*in_isr*/)
{
  const bool IS_IN = (GetDirection() == Direction::IN);
  const bool IS_EP0 = (GetNumber() == EPNumber::EP0);

  transfer_tag_ = request.tag;
  transfer_requested_ = request.length;
  transfer_accumulated_ = 0;
  transfer_active_ = true;
  transfer_dma_ = request.slot;
  transfer_cursor_ = static_cast<uint8_t*>(request.slot.addr_);
  transfer_remaining_ = 0;
  hw_completed_ = false;
  // IRQ 窄恢复路径读取（EP0 IN 无 ACK 补偿）
  // Read by the narrow IRQ recovery path (EP0 IN has no ACK compensation).
  last_transfer_size_ = request.length;

  if (IS_IN)
  {
    // 槽内存就是 DMA 缓冲：类在 Submit 前已把数据写进槽，无需再拷贝。
    // The slot memory is the DMA buffer: the class wrote the data into the slot before
    // Submit, so no copy is needed.
    set_tx_dma_buffer(GetNumber(), request.slot.addr_);
    // 超过一个包长的 IN 请求按 MPS 分包（HS BULK 为 512，槽可达 1024）。
    // Split IN requests longer than one packet at MPS (HS bulk is 512, slots
    // can be 1024).
    const size_t mps = MaxPacketSize();
    const size_t chunk = (mps > 0U && request.length > mps) ? mps : request.length;
    transfer_remaining_ = request.length - chunk;
    transfer_cursor_ += chunk;

    if (GetType() == Type::ISOCHRONOUS)
    {
      enable_tx(GetNumber());
    }

    set_tx_len(GetNumber(), chunk);
    auto addr = get_tx_control_addr(GetNumber());

    if (GetType() != Type::ISOCHRONOUS)
    {
      if (!IS_EP0)
      {
        *addr = (*addr & ~USBHS_UEP_T_RES_MASK) | USBHS_UEP_T_RES_ACK;
        tx_tog_armed_ = *addr & USBHS_UEP_T_TOG_MASK;
      }
      else
      {
        // EP0 IN 按 start 次数翻转 toggle（HS 已上板方案）。
        // EP0 IN flips the toggle per start count (the scheme verified on the HS board).
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
    if (IS_EP0 && request.length == 0)
    {
      // 控制传输的 status OUT 固定从 DATA1 开始。
      // The status OUT of a control transfer always starts from DATA1.
      tog0_ = true;
    }

    set_rx_dma_buffer(GetNumber(), request.slot.addr_);

    auto addr = get_rx_control_addr(GetNumber());

    if (GetType() != Type::ISOCHRONOUS)
    {
      if (!IS_EP0)
      {
        *addr = (*addr & ~USBHS_UEP_R_RES_MASK) | USBHS_UEP_R_RES_ACK;
      }
      else
      {
        if (request.length == 0u)
        {
          // 控制传输的 status OUT 固定从 DATA1 开始。
          // The status OUT of a control transfer always starts from DATA1.
          *addr = USBHS_UEP_R_RES_ACK |
                  (*addr & (~(USBHS_UEP_R_RES_MASK | USBHS_UEP_R_TOG_MDATA))) |
                  USBHS_UEP_R_TOG_DATA1;
        }
        else
        {
          // EP0 多包 OUT 续挂时，这里只重新打开 ACK，不再用软件重算 DATA0/DATA1。
          // 每个包成功收完后，由完成路径推进一次硬件 RX toggle。
          // When a multi-packet EP0 OUT is re-armed, only ACK is re-enabled here and
          // DATA0/DATA1 is not recomputed in software. The completion path advances the
          // hardware RX toggle once after each packet is received successfully.
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

  if (IS_EP0 && IS_IN)
  {
    tog0_ = !tog0_;
  }

  return ErrorCode::OK;
}

USB::Endpoint::StopResult CH32EndpointOtgHs::StopHardware(bool /*in_isr*/)
{
  // RES=NAK 立即生效：控制器从下一个事务起不再访问槽；
  // 在途的最后一个事务仍会完成（最多一个包落进槽），此后 INT_BUSY=1 自动 NAK 全部事务。
  // 无法也无须中止在途事务，迟到的完成由核心按标签丢弃（与 FSDEV 先例一致）。
  // RES=NAK takes effect immediately: from the next transaction on the controller no
  // longer accesses the slot; the last in-flight transaction still completes (at most one
  // packet lands in the slot), after which INT_BUSY=1 NAKs all transactions
  // automatically. An in-flight transaction can not and need not be aborted; a late
  // completion is dropped by the core by tag (same as the FSDEV precedent).
  if (GetDirection() == Direction::IN)
  {
    *get_tx_control_addr(GetNumber()) =
        (*get_tx_control_addr(GetNumber()) & ~USBHS_UEP_T_RES_MASK) | USBHS_UEP_T_RES_NAK;
  }
  else
  {
    *get_rx_control_addr(GetNumber()) =
        (*get_rx_control_addr(GetNumber()) & ~USBHS_UEP_R_RES_MASK) | USBHS_UEP_R_RES_NAK;
  }
  transfer_active_ = false;
  return StopResult::DONE;
}

size_t CH32EndpointOtgHs::ReapProgress(const HwRequest& request, bool& retired)
{
  // 同 OtgFs：只有硬件真正报过完成才算成功，否则 IN 报 0、OUT 报已收字节数。
  // Same as OtgFs: only a real hardware completion counts as success; otherwise
  // IN reports 0 and OUT reports the bytes received.
  if (hw_completed_)
  {
    retired = true;
    return request.length;
  }
  retired = false;
  return (GetDirection() == Direction::OUT) ? transfer_accumulated_ : 0U;
}

ErrorCode CH32EndpointOtgHs::StallHardware(bool /*in_isr*/)
{
  if (GetDirection() == Direction::IN)
  {
    *get_tx_control_addr(GetNumber()) |= USBHS_UEP_T_RES_STALL;
  }
  else
  {
    *get_rx_control_addr(GetNumber()) |= USBHS_UEP_R_RES_STALL;
  }
  return ErrorCode::OK;
}

ErrorCode CH32EndpointOtgHs::ClearStallHardware(bool /*in_isr*/)
{
  if (GetNumber() == EPNumber::EP0)
  {
    // 下一个 SETUP 已在中断里把 EP0 恢复为 NAK/DATA1（PrepareEp0ForSetup）；这里再写
    // 一遍会把数据阶段首包改成 DATA0，主机当作重复包丢弃，STALL 后的下一个请求超时。
    // The next SETUP has already restored EP0 to NAK/DATA1 in the interrupt
    // (PrepareEp0ForSetup); writing it again here would send the first data packet as
    // DATA0, which the host drops as a repeat, so the request after a STALL timed out.
    return ErrorCode::OK;
  }
  if (GetDirection() == Direction::IN)
  {
    *get_tx_control_addr(GetNumber()) = USBHS_UEP_T_RES_NAK | USBHS_UEP_T_TOG_AUTO;
    tx_tog_armed_ = USBHS_UEP_T_TOG_DATA0;
  }
  else
    *get_rx_control_addr(GetNumber()) = USBHS_UEP_R_RES_NAK | USBHS_UEP_R_TOG_AUTO;
  tog0_ = false;
  return ErrorCode::OK;
}

bool CH32EndpointOtgHs::InPacketAcked() const
{
  if (!transfer_active_ || GetDirection() != Direction::IN ||
      GetNumber() == EPNumber::EP0 || GetType() == Type::ISOCHRONOUS)
  {
    return false;
  }
  return (*get_tx_control_addr(GetNumber()) & USBHS_UEP_T_TOG_MASK) != tx_tog_armed_;
}

void CH32EndpointOtgHs::CompleteAckedInPackets()
{
  // INT_BUSY 让控制器在 UIF_TRANSFER 清零前对后续事务回忙 NAK，
  // 而 INT_ST 记录的是最近一次事务：IN 包刚被 ACK，
  // 主机紧接着发来的 IN（多包传输的下一包、ZLP，或别的端点的事务）被忙 NAK 后，
  // INT_ST 就变成 NAK 或别的端点，按 INT_ST 分发会丢掉这次完成，RES 仍为 ACK，
  // 控制器随后把同一包再发一遍（toggle 已翻转，主机照单全收）。
  // 所以每次 UIF_TRANSFER 都按 toggle 检查挂着的 IN 包是否已被 ACK。
  // UIF_TRANSFER 清零前不会有新事务完成，同一次检查不会把续挂的包误判为已发。
  // With INT_BUSY the controller busy-NAKs further transactions until UIF_TRANSFER is
  // cleared, while INT_ST describes the latest transaction: when the IN right after an
  // ACKed packet (the next packet of a multi-packet transfer, a ZLP, or another
  // endpoint's transaction) is busy-NAKed, INT_ST shows a NAK or another endpoint and
  // dispatching by INT_ST loses the completion. RES stays ACK and the controller sends
  // the same packet again (with the toggle already advanced, so the host accepts it).
  // Hence every UIF_TRANSFER checks the toggle of each armed IN packet. No transaction
  // completes before UIF_TRANSFER is cleared, so a packet re-armed here is not taken
  // for a sent one in the same check.
  for (uint8_t i = 1; i < EP_OTG_HS_MAX_SIZE; ++i)
  {
    auto* ep_in = map_otg_hs_[i][static_cast<uint8_t>(Direction::IN)];
    if (ep_in != nullptr && ep_in->InPacketAcked())
    {
      ep_in->TransferComplete(0u);
    }
  }
}

size_t CH32EndpointOtgHs::MaxPacketSizeLimit(Type type) const
{
  // CH32 USBHS 的 BULK 端点始终使用硬件固定的 512 字节包长；CONTROL/INTERRUPT
  // 为 64；ISO 上限 1023（手册 p.340/p.344）。
  // The BULK endpoints of CH32 USBHS always use the hardware-fixed 512-byte packet size;
  // CONTROL/INTERRUPT use 64; ISO is limited to 1023 (manual p.340/p.344).
  if (type == Type::BULK)
  {
    return 512U;
  }
  if (type == Type::ISOCHRONOUS)
  {
    return 1023U;
  }
  return 64U;
}

void CH32EndpointOtgHs::TransferComplete(size_t size)
{
  // 同 OtgFs：停止后/未挂载时的迟到完成直接丢弃，不再动寄存器。
  // Same as OtgFs: drop completions that arrive after a stop or with nothing
  // armed, without touching the registers.
  if (!transfer_active_)
  {
    return;
  }

  const bool IS_IN = (GetDirection() == Direction::IN);
  const bool IS_EP0 = (GetNumber() == EPNumber::EP0);
  const bool IS_ISO = (GetType() == Type::ISOCHRONOUS);

  // UIF_TRANSFER/INT_FG 会在 IRQ handler 完成分发后统一清掉。
  // UIF_TRANSFER/INT_FG are cleared by the IRQ handler once dispatch has finished.

  if (IS_IN)
  {
    auto* tx_ctrl = get_tx_control_addr(GetNumber());

    // 还有后续分包：续发下一包，整请求发完才向核心报一次完成。
    // More packets to go: send the next one and report a single completion for
    // the whole request.
    if (transfer_remaining_ > 0U && !IS_ISO)
    {
      const size_t mps = MaxPacketSize();
      const size_t chunk =
          (mps > 0U && transfer_remaining_ > mps) ? mps : transfer_remaining_;
      set_tx_dma_buffer(GetNumber(), transfer_cursor_);
      set_tx_len(GetNumber(), chunk);
      transfer_cursor_ += chunk;
      transfer_remaining_ -= chunk;
      *tx_ctrl = (*tx_ctrl & ~USBHS_UEP_T_RES_MASK) | USBHS_UEP_T_RES_ACK;
      tx_tog_armed_ = *tx_ctrl & USBHS_UEP_T_TOG_MASK;
      return;
    }

    *tx_ctrl = (*tx_ctrl & ~USBHS_UEP_T_RES_MASK) | USBHS_UEP_T_RES_NAK;

    if (IS_ISO)
    {
      set_tx_len(GetNumber(), 0);
      disable_tx(GetNumber());
    }

    transfer_active_ = false;
    transfer_remaining_ = 0U;
    hw_completed_ = true;
    OnHardwareComplete(true, transfer_tag_, transfer_requested_);
    return;
  }

  // TOG 不匹配表示数据同步失败：丢弃本包并按原请求重新挂接收。
  // A TOG mismatch means data synchronization failed: drop this packet and re-arm the
  // receive with the original request.
  if (!IS_EP0 && !IS_ISO)
  {
    const bool TOG_OK =
        ((USBHSD->INT_ST & USBHS_UIS_TOG_OK) == USBHS_UIS_TOG_OK);  // NOLINT
    if (!TOG_OK)
    {
      HwRequest retry{};
      retry.slot = transfer_dma_;
      retry.length = transfer_requested_;
      retry.tag = transfer_tag_;
      (void)StartHardware(retry, true);
      return;
    }
  }

  if (IS_EP0)
  {
    auto* rx_ctrl = get_rx_control_addr(GetNumber());
    // 这里不要直接把 EP0 OUT 留在 ACK：上层控制传输栈会在当前包被完整消费后再重新挂接收，
    // 否则下一轮 session 可能撞上陈旧的 EP0 软件状态。
    // Do not leave EP0 OUT in ACK here: the upper control transfer stack re-arms the
    // receive after the current packet is fully consumed, otherwise the next session may
    // run into stale EP0 software state.
    if (size > 0u)
    {
      // 每成功收完一个 EP0 OUT 包，都推进一次硬件 RX DATA0/DATA1 相位（多包
      // OUT 数据阶段在此推进；只重开 ACK 不消费 toggle 会让后续包停住）。
      // After every EP0 OUT packet is received successfully, the hardware RX DATA0/DATA1
      // phase advances once (the multi-packet OUT data stage advances here; re-enabling
      // only ACK without consuming the toggle would stall the following packets).
      *rx_ctrl = static_cast<uint8_t>(
          ((*rx_ctrl ^ USBHS_UEP_R_TOG_DATA1) & ~USBHS_UEP_R_RES_MASK) |
          USBHS_UEP_R_RES_NAK);
    }
    else
    {
      *rx_ctrl =
          static_cast<uint8_t>((*rx_ctrl & ~USBHS_UEP_R_RES_MASK) | USBHS_UEP_R_RES_NAK);
    }
    transfer_active_ = false;
    hw_completed_ = true;
    OnHardwareComplete(true, transfer_tag_, size);
    return;
  }

  if (!IS_ISO)
  {
    // 非 EP0 非 ISO：DMA 寄存器分立，按包推进 RX DMA，多包请求一次完成。
    // Non-EP0 non-ISO: the DMA registers are separate, so the RX DMA advances per packet
    // and a multi-packet request completes once.
    transfer_accumulated_ += size;
    if (transfer_accumulated_ < transfer_requested_ && size == MaxPacketSize())
    {
      transfer_cursor_ += size;
      *get_rx_dma_addr(GetNumber()) =
          static_cast<uint32_t>(reinterpret_cast<uintptr_t>(transfer_cursor_));
      // RES 仍为 ACK，继续收下一包
      // RES stays ACK, keep receiving the next packet.
      return;
    }
    auto* rx_ctrl = get_rx_control_addr(GetNumber());
    *rx_ctrl = (*rx_ctrl & ~USBHS_UEP_R_RES_MASK) | USBHS_UEP_R_RES_NAK;
    transfer_active_ = false;
    hw_completed_ = true;
    OnHardwareComplete(true, transfer_tag_, transfer_accumulated_);
    return;
  }

  // ISO：每包完成 / ISO: every packet completes.
  auto* rx_ctrl = get_rx_control_addr(GetNumber());
  *rx_ctrl = (*rx_ctrl & ~USBHS_UEP_R_RES_MASK) | USBHS_UEP_R_RES_NAK;
  transfer_accumulated_ = size;
  transfer_active_ = false;
  hw_completed_ = true;
  OnHardwareComplete(true, transfer_tag_, size);
}

#endif
