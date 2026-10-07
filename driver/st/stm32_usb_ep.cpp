#include "stm32_usb_ep.hpp"

#include "stm32_dcache.hpp"
using namespace LibXR;

#if defined(HAL_PCD_MODULE_ENABLED)

static inline bool is_power_of_two(unsigned int n) { return n > 0 && (n & (n - 1)) == 0; }

#if defined(USB_OTG_HS) || defined(USB_OTG_FS)

STM32Endpoint::STM32Endpoint(EPNumber ep_num, stm32_usb_dev_id_t id,
                             PCD_HandleTypeDef* hpcd, Direction dir, size_t fifo_size,
                             LibXR::RawData buffer)
    : Endpoint(ep_num, dir, buffer), hpcd_(hpcd), fifo_size_(fifo_size), id_(id)
{
  ASSERT(fifo_size >= 8);
  ASSERT(is_power_of_two(fifo_size) || fifo_size % 4 == 0);
  ASSERT(is_power_of_two(buffer.size_) || buffer.size_ % 4 == 0);

#if defined(USB_OTG_HS)
  if (id == STM32_USB_OTG_HS)
  {
    map_otg_hs_[EPNumberToInt8(GetNumber())][static_cast<uint8_t>(dir)] = this;
  }
#endif
#if defined(USB_OTG_FS)
  if (id == STM32_USB_OTG_FS)
  {
    map_otg_fs_[EPNumberToInt8(GetNumber())][static_cast<uint8_t>(dir)] = this;
  }
#endif

  if (dir == Direction::IN)
  {
    HAL_PCDEx_SetTxFiFo(hpcd_, EPNumberToInt8(GetNumber()), fifo_size / 4);
  }
  else if (dir == Direction::OUT && ep_num == USB::Endpoint::EPNumber::EP0)
  {
    HAL_PCDEx_SetRxFiFo(hpcd_, fifo_size / 4);
  }
}
#endif

#if defined(USB_BASE) || defined(USB_DRD_FS)
STM32Endpoint::STM32Endpoint(EPNumber ep_num, stm32_usb_dev_id_t id,
                             PCD_HandleTypeDef* hpcd, Direction dir,
                             size_t hw_buffer_offset, size_t hw_buffer_size,
                             LibXR::RawData buffer)
    : Endpoint(ep_num, dir, buffer), hpcd_(hpcd), hw_buffer_size_(hw_buffer_size), id_(id)
{
  ASSERT(hw_buffer_size >= 8);

  ASSERT(is_power_of_two(hw_buffer_size));
  ASSERT(is_power_of_two(buffer.size_) || buffer.size_ % 4 == 0);

  map_fs_[EPNumberToInt8(GetNumber())][static_cast<uint8_t>(dir)] = this;

  size_t buffer_offset = hw_buffer_offset;

  HAL_PCDEx_PMAConfig(hpcd_, EPNumberToAddr(GetNumber(), dir), PCD_SNG_BUF,
                      buffer_offset);
}
#endif

void STM32Endpoint::Configure(const Config& cfg)
{
  ASSERT(cfg.direction == Direction::IN || cfg.direction == Direction::OUT);

  uint8_t addr = EPNumberToAddr(GetNumber(), cfg.direction);
  uint8_t type = static_cast<uint8_t>(cfg.type);
  auto& ep_cfg = GetConfig();
  size_t packet_size_limit = 0;

  ep_cfg = cfg;

  switch (cfg.type)
  {
    case Type::BULK:
#if defined(PCD_SPEED_HIGH_IN_FULL)
      if (hpcd_->Init.speed == PCD_SPEED_FULL ||
          hpcd_->Init.speed == PCD_SPEED_HIGH_IN_FULL)
#else
      if (hpcd_->Init.speed == PCD_SPEED_FULL)
#endif
      {
        packet_size_limit = 64;
      }
      else
      {
        packet_size_limit = 512;
      }
      break;
    case Type::INTERRUPT:
#if defined(PCD_SPEED_HIGH_IN_FULL)
      if (hpcd_->Init.speed == PCD_SPEED_FULL ||
          hpcd_->Init.speed == PCD_SPEED_HIGH_IN_FULL)
#else
      if (hpcd_->Init.speed == PCD_SPEED_FULL)
#endif
      {
        packet_size_limit = 64;
      }
      else
      {
        packet_size_limit = 1024;
      }
      break;
    case Type::ISOCHRONOUS:
#if defined(PCD_SPEED_HIGH_IN_FULL)
      if (hpcd_->Init.speed == PCD_SPEED_FULL ||
          hpcd_->Init.speed == PCD_SPEED_HIGH_IN_FULL)
#else
      if (hpcd_->Init.speed == PCD_SPEED_FULL)
#endif
      {
        packet_size_limit = 1023;
      }
      else
      {
        packet_size_limit = 1024;
      }
      break;
    case Type::CONTROL:
      packet_size_limit = 64;
      break;
    default:
      break;
  }

#if defined(USB_OTG_FS) || defined(USB_OTG_HS)
  if (packet_size_limit > fifo_size_)
  {
    packet_size_limit = fifo_size_;
  }
#endif

#if defined(USB_BASE) || defined(USB_DRD_FS)
  if (packet_size_limit > hw_buffer_size_)
  {
    packet_size_limit = hw_buffer_size_;
  }
#endif

  auto buffer = GetBuffer();

  if (packet_size_limit > buffer.size_)
  {
    packet_size_limit = buffer.size_;
  }

  size_t max_packet_size = cfg.max_packet_size;

  if (max_packet_size > packet_size_limit)
  {
    max_packet_size = packet_size_limit;
  }

  ep_cfg.max_packet_size = max_packet_size;

  if (max_packet_size < 8)
  {
    max_packet_size = 8;
  }

  if (HAL_PCD_EP_Open(hpcd_, addr, max_packet_size, type) == HAL_OK)
  {
    SetState(State::IDLE);
  }
  else
  {
    SetState(State::ERROR);
  }
}

#if defined(USB_OTG_FS) || defined(USB_OTG_HS)
static bool wait_otg_in_ep_flag(USB_OTG_INEndpointTypeDef* in_ep, uint32_t flag)
{
  // 正常情况下几个 PHY 时钟内置位；上限只防止 PHY 时钟停止时卡死。
  // Normally set within a few PHY clocks; the bound only guards a stopped PHY clock.
  constexpr uint32_t MAX_POLLS = 10000U;
  for (uint32_t i = 0U; i < MAX_POLLS; ++i)
  {
    if ((in_ep->DIEPINT & flag) != 0U)
    {
      return true;
    }
  }
  return false;
}

static void disable_otg_in_ep(PCD_HandleTypeDef* hpcd, uint8_t ep_num)
{
  const uintptr_t base = reinterpret_cast<uintptr_t>(hpcd->Instance);
  auto* in_ep = reinterpret_cast<USB_OTG_INEndpointTypeDef*>(
      base + USB_OTG_IN_ENDPOINT_BASE + ep_num * USB_OTG_EP_REG_SIZE);
  auto* device = reinterpret_cast<USB_OTG_DeviceTypeDef*>(base + USB_OTG_DEVICE_BASE);

  if ((in_ep->DIEPCTL & USB_OTG_DIEPCTL_EPENA) == 0U)
  {
    return;
  }

  // HAL_PCD_EP_Close 只请求禁用；HAL 稍后在 EPDISD 中断里清空该 TxFIFO，
  // 会清掉关闭后重新开始的传输已写入的数据。这里按参考手册的禁用流程等禁用完成，
  // 清掉 EPDISD 并立即清空 FIFO。
  // HAL_PCD_EP_Close only requests the disable; HAL later flushes this TxFIFO in the
  // EPDISD interrupt, which would discard data already written for a transfer
  // restarted after the close. Follow the reference-manual disable sequence, wait
  // for completion, clear EPDISD and flush the FIFO now.
  device->DIEPEMPMSK &= ~(1UL << ep_num);
  in_ep->DIEPCTL |= USB_OTG_DIEPCTL_SNAK;
  (void)wait_otg_in_ep_flag(in_ep, USB_OTG_DIEPINT_INEPNE);
  in_ep->DIEPCTL |= USB_OTG_DIEPCTL_EPDIS | USB_OTG_DIEPCTL_SNAK;
  (void)wait_otg_in_ep_flag(in_ep, USB_OTG_DIEPINT_EPDISD);
  in_ep->DIEPINT = USB_OTG_DIEPINT_INEPNE | USB_OTG_DIEPINT_EPDISD;
  (void)USB_FlushTxFifo(hpcd->Instance, ep_num);
}
#endif

void STM32Endpoint::Close()
{
  uint8_t addr = EPNumberToAddr(GetNumber(), GetDirection());
#if defined(USB_OTG_FS) || defined(USB_OTG_HS)
  bool is_otg = false;
#if defined(USB_OTG_FS)
  is_otg = is_otg || id_ == STM32_USB_OTG_FS;
#endif
#if defined(USB_OTG_HS)
  is_otg = is_otg || id_ == STM32_USB_OTG_HS;
#endif
  if (is_otg && GetDirection() == Direction::IN)
  {
    disable_otg_in_ep(hpcd_, EPNumberToInt8(GetNumber()));
  }
#endif
  HAL_PCD_EP_Close(hpcd_, addr);
  SetState(State::DISABLED);
}

ErrorCode STM32Endpoint::Transfer(size_t size)
{
  if (GetState() == State::BUSY)
  {
    return ErrorCode::BUSY;
  }

  bool is_in = GetDirection() == Direction::IN;
  auto ep_addr = EPNumberToAddr(GetNumber(), GetDirection());

  PCD_EPTypeDef* ep = is_in ? &hpcd_->IN_ep[ep_addr & EP_ADDR_MSK]
                            : &hpcd_->OUT_ep[ep_addr & EP_ADDR_MSK];

  auto buffer = GetBuffer();

  if (buffer.size_ < size)
  {
    return ErrorCode::NO_BUFF;
  }

  ep->xfer_buff = reinterpret_cast<uint8_t*>(buffer.addr_);

  if (UseDoubleBuffer() && GetDirection() == Direction::IN && size > 0)
  {
    SwitchBuffer();
  }

  ep->xfer_len = size;
  ep->xfer_count = 0U;
  ep->is_in = is_in ? 1U : 0U;
  ep->num = ep_addr & EP_ADDR_MSK;
  last_transfer_size_ = size;

#if defined(USB_OTG_FS) || defined(USB_OTG_HS)
  if (hpcd_->Init.dma_enable == 1U)
  {
    ep->dma_addr = reinterpret_cast<uint32_t>(ep->xfer_buff);

    if (is_in == true)
    {
      STM32_CleanDCacheByAddr(buffer.addr_, size);
    }
  }
#endif

#if defined(USB_BASE) || defined(USB_DRD_FS)
  if (is_in)
  {
    ep->xfer_fill_db = 0U;
    ep->xfer_len_db = 0U;
  }
#endif

  SetState(State::BUSY);

#if defined(USB_OTG_FS) || defined(USB_OTG_HS)
  auto ans = USB_EPStartXfer(hpcd_->Instance, ep, hpcd_->Init.dma_enable);
#else
  auto ans = USB_EPStartXfer(hpcd_->Instance, ep);
  if (size == 0 && GetNumber() == USB::Endpoint::EPNumber::EP0 &&
      GetDirection() == Direction::OUT)
  {
    OnTransferCompleteCallback(false, 0);
  }
#endif

  if (ans == HAL_OK)
  {
    return ErrorCode::OK;
  }
  else
  {
    SetState(State::ERROR);
    return ErrorCode::FAILED;
  }
}

ErrorCode STM32Endpoint::Stall()
{
  const bool is_in = (GetDirection() == Direction::IN);
  if (GetState() != State::IDLE && !(GetState() == State::BUSY && !is_in))
  {
    return ErrorCode::BUSY;
  }

  uint8_t addr = EPNumberToAddr(GetNumber(), GetDirection());
  if (HAL_PCD_EP_SetStall(hpcd_, addr) == HAL_OK)
  {
    SetState(State::STALLED);
    return ErrorCode::OK;
  }
  else
  {
    SetState(State::ERROR);
    return ErrorCode::FAILED;
  }
}

ErrorCode STM32Endpoint::ClearStall()
{
  if (GetState() != State::STALLED)
  {
    return ErrorCode::FAILED;
  }

  uint8_t addr = EPNumberToAddr(GetNumber(), GetDirection());

  if (GetNumber() == USB::Endpoint::EPNumber::EP0)
  {
    SetState(State::IDLE);
    return ErrorCode::OK;
  }

  if (HAL_PCD_EP_ClrStall(hpcd_, addr) == HAL_OK)
  {
    SetState(State::IDLE);
    return ErrorCode::OK;
  }
  else
  {
    SetState(State::ERROR);
    return ErrorCode::FAILED;
  }
}

size_t STM32Endpoint::MaxTransferSize() const
{
  if (GetNumber() == USB::Endpoint::EPNumber::EP0)
  {
    return MaxPacketSize();
  }
  else
  {
    return GetBuffer().size_;
  }
}

// --- HAL C 回调桥接 ---
// NOLINTNEXTLINE
static STM32Endpoint* GetEndpoint(PCD_HandleTypeDef* hpcd, uint8_t epnum, bool is_in)
{
  auto id = STM32USBDeviceGetID(hpcd);
#if defined(USB_OTG_HS)
  if (id == STM32_USB_OTG_HS)
  {
    return STM32Endpoint::map_otg_hs_[epnum & 0x7F][static_cast<uint8_t>(is_in)];
  }
#endif
#if defined(USB_OTG_FS)
  if (id == STM32_USB_OTG_FS)
  {
    return STM32Endpoint::map_otg_fs_[epnum & 0x7F][static_cast<uint8_t>(is_in)];
  }
#endif
#if defined(USB_BASE) || defined(USB_DRD_FS)
  if (id == STM32_USB_FS_DEV)
  {
    return STM32Endpoint::map_fs_[epnum & 0x7F][static_cast<uint8_t>(is_in)];
  }
#endif
  return nullptr;
}

extern "C" void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef* hpcd, uint8_t epnum)
{
  auto id = STM32USBDeviceGetID(hpcd);

  if (id >= STM32_USB_DEV_ID_NUM || STM32USBDevice::map_[id] == nullptr)
  {
    return;
  }

  auto ep = GetEndpoint(hpcd, epnum, true);

  if (!ep || ep->hpcd_ != hpcd)
  {
    return;
  }

  ep->OnTransferCompleteCallback(true, ep->last_transfer_size_);
}

extern "C" void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef* hpcd, uint8_t epnum)
{
  auto id = STM32USBDeviceGetID(hpcd);

  if (id >= STM32_USB_DEV_ID_NUM || STM32USBDevice::map_[id] == nullptr)
  {
    return;
  }

  auto ep = GetEndpoint(hpcd, epnum, false);

  if (!ep || ep->hpcd_ != hpcd)
  {
    return;
  }

  PCD_EPTypeDef* ep_handle = &hpcd->OUT_ep[epnum & EP_ADDR_MSK];

  size_t actual_transfer_size = ep_handle->xfer_count;

  if (STM32USBUsesDma(hpcd))
  {
    STM32_InvalidateDCacheByAddr(ep->GetBuffer().addr_, actual_transfer_size);
  }

  ep->OnTransferCompleteCallback(true, actual_transfer_size);
}

extern "C" void HAL_PCD_ISOINIncompleteCallback(PCD_HandleTypeDef* hpcd, uint8_t epnum)
{
  auto id = STM32USBDeviceGetID(hpcd);

  if (id >= STM32_USB_DEV_ID_NUM || STM32USBDevice::map_[id] == nullptr)
  {
    return;
  }

  auto ep = GetEndpoint(hpcd, epnum, true);

  if (!ep || ep->hpcd_ != hpcd)
  {
    return;
  }

  ep->OnTransferCompleteCallback(true, 0);
}

extern "C" void HAL_PCD_ISOOUTIncompleteCallback(PCD_HandleTypeDef* hpcd, uint8_t epnum)
{
  auto id = STM32USBDeviceGetID(hpcd);

  if (id >= STM32_USB_DEV_ID_NUM || STM32USBDevice::map_[id] == nullptr)
  {
    return;
  }

  auto ep = GetEndpoint(hpcd, epnum, false);

  if (!ep || ep->hpcd_ != hpcd)
  {
    return;
  }

  ep->OnTransferCompleteCallback(true, 0);
}
#endif
