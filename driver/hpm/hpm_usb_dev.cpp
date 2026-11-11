#include "hpm_usb_dev.hpp"

#include <cstring>

#include "hpm_interrupt.h"
#include "hpm_l1c_drv.h"
#include "hpm_misc.h"

#if __has_include("board.h")
#include "board.h"
#define LIBXR_HPM_USB_HAS_BOARD_HELPER 1
#else
#define LIBXR_HPM_USB_HAS_BOARD_HELPER 0
#endif

using namespace LibXR;

namespace
{

static ATTR_PLACE_AT_NONCACHEABLE_BSS_WITH_ALIGNMENT(
    USB_SOC_DCD_DATA_RAM_ADDRESS_ALIGNMENT)
    HPMUSBDcdData g_hpm_usb_dcd_data[HPMUSBDevice::MAX_USB_INSTANCES];

constexpr uint32_t HPM_USB_INTR_UI = HPM_BITSMASK(1, 0);
constexpr uint32_t HPM_USB_INTR_UEI = HPM_BITSMASK(1, 1);
constexpr uint32_t HPM_USB_INTR_PCI = HPM_BITSMASK(1, 2);
constexpr uint32_t HPM_USB_INTR_URI = HPM_BITSMASK(1, 6);
constexpr uint32_t HPM_USB_INTR_SLI = HPM_BITSMASK(1, 8);

constexpr uint32_t HPM_USB_DTD_ACTIVE = HPM_BITSMASK(1, 7);
constexpr uint32_t HPM_USB_DTD_HALTED = HPM_BITSMASK(1, 6);
constexpr uint32_t HPM_USB_DTD_BUFFER_ERR = HPM_BITSMASK(1, 5);
constexpr uint32_t HPM_USB_DTD_XACT_ERR = HPM_BITSMASK(1, 3);
constexpr uint32_t HPM_USB_DTD_IOC = HPM_BITSMASK(1, 15);
constexpr uint32_t HPM_USB_DTD_TOTAL_BYTES_SHIFT = 16;
constexpr uint32_t HPM_USB_DTD_TOTAL_BYTES_MASK = 0x7FFFu
                                                  << HPM_USB_DTD_TOTAL_BYTES_SHIFT;

constexpr uint32_t HPM_USB_DQH_IOS = HPM_BITSMASK(1, 15);
constexpr uint32_t HPM_USB_DQH_MPS_SHIFT = 16;
constexpr uint32_t HPM_USB_DQH_MPS_MASK = 0x7FFu << HPM_USB_DQH_MPS_SHIFT;
constexpr uint32_t HPM_USB_DQH_ZLT = HPM_BITSMASK(1, 29);
constexpr uint32_t HPM_USB_DQH_MULT_SHIFT = 30;

uint8_t EpAddrToIndex(uint8_t ep_addr)
{
  const uint8_t ep_num = ep_addr & 0x0Fu;
  const uint8_t dir = (ep_addr & 0x80u) ? 1u : 0u;
  return static_cast<uint8_t>(ep_num * 2u + dir);
}

uint8_t EpIndexToCompleteBit(uint8_t ep_idx)
{
  return static_cast<uint8_t>(ep_idx / 2u + ((ep_idx & 0x01u) ? 16u : 0u));
}

uint32_t ToUsbAddress(const void* ptr)
{
  return core_local_mem_to_sys_address(
      0, static_cast<uint32_t>(reinterpret_cast<uintptr_t>(ptr)));
}

uintptr_t CacheAlignDown(uintptr_t addr)
{
  return addr & ~(static_cast<uintptr_t>(HPM_L1C_CACHELINE_SIZE) - 1u);
}

uintptr_t CacheAlignUp(uintptr_t addr)
{
  return (addr + HPM_L1C_CACHELINE_SIZE - 1u) &
         ~(static_cast<uintptr_t>(HPM_L1C_CACHELINE_SIZE) - 1u);
}

void CleanDCache(void* addr, size_t size)
{
  if (addr == nullptr || size == 0 || !l1c_dc_is_enabled())
  {
    return;
  }

  const uintptr_t start = CacheAlignDown(reinterpret_cast<uintptr_t>(addr));
  const uintptr_t end = CacheAlignUp(reinterpret_cast<uintptr_t>(addr) + size);
  l1c_dc_writeback(static_cast<uint32_t>(start), static_cast<uint32_t>(end - start));
}

void InvalidateDCache(void* addr, size_t size)
{
  if (addr == nullptr || size == 0 || !l1c_dc_is_enabled())
  {
    return;
  }

  const uintptr_t start = CacheAlignDown(reinterpret_cast<uintptr_t>(addr));
  const uintptr_t end = CacheAlignUp(reinterpret_cast<uintptr_t>(addr) + size);
  l1c_dc_invalidate(static_cast<uint32_t>(start), static_cast<uint32_t>(end - start));
}

uint32_t BuildQhCaps(uint16_t max_packet_size, USB::Endpoint::Type type, bool setup)
{
  uint32_t caps = HPM_USB_DQH_ZLT |
                  ((static_cast<uint32_t>(max_packet_size) << HPM_USB_DQH_MPS_SHIFT) &
                   HPM_USB_DQH_MPS_MASK);

  if (setup)
  {
    caps |= HPM_USB_DQH_IOS;
  }

  if (type == USB::Endpoint::Type::ISOCHRONOUS)
  {
    caps |= 1u << HPM_USB_DQH_MULT_SHIFT;
  }

  return caps;
}

uint32_t BuildDtdToken(uint32_t total_bytes, bool interrupt_on_complete)
{
  uint32_t token = HPM_USB_DTD_ACTIVE | ((total_bytes << HPM_USB_DTD_TOTAL_BYTES_SHIFT) &
                                         HPM_USB_DTD_TOTAL_BYTES_MASK);
  if (interrupt_on_complete)
  {
    token |= HPM_USB_DTD_IOC;
  }
  return token;
}

uint32_t DtdRemainingBytes(const HPMUSBDtd* qtd)
{
  return (qtd->token & HPM_USB_DTD_TOTAL_BYTES_MASK) >> HPM_USB_DTD_TOTAL_BYTES_SHIFT;
}

bool DtdHasError(const HPMUSBDtd* qtd)
{
  return (qtd->token &
          (HPM_USB_DTD_HALTED | HPM_USB_DTD_BUFFER_ERR | HPM_USB_DTD_XACT_ERR)) != 0;
}

bool DtdIsActive(const HPMUSBDtd* qtd) { return (qtd->token & HPM_USB_DTD_ACTIVE) != 0; }

uint16_t PacketSizeValue(USB::DeviceDescriptor::PacketSize0 packet_size)
{
  return static_cast<uint16_t>(packet_size);
}

void Increment(volatile uint32_t& counter) { counter = counter + 1u; }

}  // namespace

HPMUSBDevice* HPMUSBDevice::instances_[HPMUSBDevice::MAX_USB_INSTANCES] = {};

HPMUSBEndpoint::HPMUSBEndpoint(HPMUSBDevice& device, EPNumber ep_num, Direction dir,
                               LibXR::RawData buffer)
    : USB::Endpoint(ep_num, dir, buffer), device_(device)
{
  ASSERT(buffer.addr_ != nullptr);
  ASSERT(buffer.size_ > 0);
  device_.RegisterEndpoint(this);
}

void HPMUSBEndpoint::Configure(const Config& cfg)
{
  ASSERT(cfg.direction == Direction::IN || cfg.direction == Direction::OUT);

  auto& ep_cfg = GetConfig();
  ep_cfg = cfg;

  size_t packet_size_limit = 0;
  const bool high_speed = device_.speed_ == USB::Speed::HIGH;

  switch (cfg.type)
  {
    case Type::BULK:
      packet_size_limit = high_speed ? 512u : 64u;
      break;
    case Type::INTERRUPT:
      packet_size_limit = high_speed ? 1024u : 64u;
      break;
    case Type::ISOCHRONOUS:
      packet_size_limit = high_speed ? 1024u : 1023u;
      break;
    case Type::CONTROL:
      packet_size_limit = 64u;
      break;
    default:
      packet_size_limit = 64u;
      break;
  }

  auto buffer = GetBuffer();
  packet_size_limit = LibXR::min<size_t>(packet_size_limit, buffer.size_);

  size_t max_packet_size = cfg.max_packet_size;
  if (max_packet_size > packet_size_limit)
  {
    max_packet_size = packet_size_limit;
  }
  if (max_packet_size < 8u)
  {
    max_packet_size = 8u;
  }

  ep_cfg.max_packet_size = static_cast<uint16_t>(max_packet_size);
  ep_cfg.double_buffer = false;

  const uint8_t ep_addr = USB::Endpoint::EPNumberToAddr(GetNumber(), cfg.direction);
  if (device_.OpenEndpoint(ep_addr, cfg.type, ep_cfg.max_packet_size))
  {
    SetState(State::IDLE);
  }
  else
  {
    SetState(State::ERROR);
  }
}

void HPMUSBEndpoint::Close()
{
  if (device_.running_)
  {
    usb_dcd_edpt_close(device_.instance_, GetAddress());
  }
  SetState(State::DISABLED);
}

ErrorCode HPMUSBEndpoint::Transfer(size_t size)
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

  if (GetDirection() == Direction::IN && UseDoubleBuffer() && size > 0)
  {
    SwitchBuffer();
    buffer = GetBuffer();
  }

  last_transfer_size_ = size;

  if (GetDirection() == Direction::IN)
  {
    CleanDCache(buffer.addr_, size);
  }
  else
  {
    InvalidateDCache(buffer.addr_, buffer.size_);
  }

  SetState(State::BUSY);

  if (device_.SubmitTransfer(GetAddress(), static_cast<uint8_t*>(buffer.addr_),
                             static_cast<uint32_t>(size)))
  {
    return ErrorCode::OK;
  }

  SetState(State::ERROR);
  return ErrorCode::FAILED;
}

ErrorCode HPMUSBEndpoint::Stall()
{
  const bool is_in = (GetDirection() == Direction::IN);
  if (GetState() != State::IDLE && !(GetState() == State::BUSY && !is_in))
  {
    return ErrorCode::BUSY;
  }

  usb_dcd_edpt_stall(device_.instance_, GetAddress());
  SetState(State::STALLED);
  return ErrorCode::OK;
}

ErrorCode HPMUSBEndpoint::ClearStall()
{
  if (GetState() != State::STALLED)
  {
    return ErrorCode::FAILED;
  }

  usb_dcd_edpt_clear_stall(device_.instance_, GetAddress());
  SetState(State::IDLE);
  return ErrorCode::OK;
}

size_t HPMUSBEndpoint::MaxTransferSize() const
{
  if (GetNumber() == EPNumber::EP0)
  {
    return MaxPacketSize();
  }
  return GetBuffer().size_;
}

void HPMUSBEndpoint::TransferComplete(bool in_isr)
{
  auto* qhd = device_.Qhd(EpAddrToIndex(GetAddress()));
  auto* qtd = qhd->attached_qtd;

  if (qtd == nullptr)
  {
    Increment(device_.diag_transfer_error_count_);
    SetState(State::ERROR);
    return;
  }

  size_t actual = 0;
  bool failed = false;

  while (qtd != nullptr)
  {
    if (DtdHasError(qtd))
    {
      failed = true;
      break;
    }

    if (DtdIsActive(qtd))
    {
      return;
    }

    actual += static_cast<size_t>(qtd->expected_bytes) - DtdRemainingBytes(qtd);
    qtd->in_use = 0;

    if (qtd->next == USB_SOC_DCD_QTD_NEXT_INVALID)
    {
      break;
    }

    qtd = reinterpret_cast<HPMUSBDtd*>(static_cast<uintptr_t>(qtd->next));
  }

  qhd->attached_qtd = nullptr;

  if (failed)
  {
    Increment(device_.diag_transfer_error_count_);
    SetState(State::ERROR);
    return;
  }

  if (GetDirection() == Direction::OUT)
  {
    InvalidateDCache(GetBuffer().addr_, actual);
  }
  else
  {
    actual = last_transfer_size_;
  }

  OnTransferCompleteCallback(in_isr, actual);
}

HPMUSBDevice::HPMUSBDevice(
    USB_Type* instance, uint32_t irq, const std::initializer_list<EPConfig> EP_CFGS,
    USB::DeviceDescriptor::PacketSize0 packet_size, uint16_t vid, uint16_t pid,
    uint16_t bcd,
    const std::initializer_list<const USB::DescriptorStrings::LanguagePack*> LANG_LIST,
    const std::initializer_list<const std::initializer_list<USB::ConfigDescriptorItem*>>
        CONFIGS,
    ConstRawData uid, USB::Speed speed, bool auto_board_init, USB::USBSpec spec,
    BoardConfig board_config)
    : USB::EndpointPool(EP_CFGS.size() * 2u),
      USB::DeviceCore(*this, spec, speed, packet_size, vid, pid, bcd, LANG_LIST, CONFIGS,
                      uid),
      instance_(instance),
      irq_(irq),
      index_(ResolveIndex(instance)),
      speed_(speed),
      ep0_packet_size_(packet_size),
      auto_board_init_(auto_board_init),
      board_config_(board_config)
{
  ASSERT(instance_ != nullptr);
  ASSERT(irq_ != INVALID_IRQ);
  ASSERT(index_ < MAX_USB_INSTANCES);
  ASSERT(EP_CFGS.size() > 0);
  ASSERT(EP_CFGS.size() <= USB_SOC_DCD_MAX_ENDPOINT_COUNT);

  auto cfg = EP_CFGS.begin();

  auto* ep0_out = new HPMUSBEndpoint(*this, USB::Endpoint::EPNumber::EP0,
                                     USB::Endpoint::Direction::OUT, cfg->buffer_out);
  auto* ep0_in = new HPMUSBEndpoint(*this, USB::Endpoint::EPNumber::EP0,
                                    USB::Endpoint::Direction::IN, cfg->buffer_in);
  USB::EndpointPool::SetEndpoint0(ep0_in, ep0_out);

  USB::Endpoint::EPNumber ep_num = USB::Endpoint::EPNumber::EP1;
  for (++cfg; cfg != EP_CFGS.end(); ++cfg, ep_num = USB::Endpoint::NextEPNumber(ep_num))
  {
    if (cfg->is_in == -1)
    {
      USB::EndpointPool::Put(new HPMUSBEndpoint(
          *this, ep_num, USB::Endpoint::Direction::OUT, cfg->buffer_out));
      USB::EndpointPool::Put(new HPMUSBEndpoint(
          *this, ep_num, USB::Endpoint::Direction::IN, cfg->buffer_in));
    }
    else
    {
      const auto dir =
          cfg->is_in ? USB::Endpoint::Direction::IN : USB::Endpoint::Direction::OUT;
      const auto buffer = cfg->is_in ? cfg->buffer_in : cfg->buffer_out;
      USB::EndpointPool::Put(new HPMUSBEndpoint(*this, ep_num, dir, buffer));
    }
  }
}

void HPMUSBDevice::Start(bool in_isr)
{
  UNUSED(in_isr);

  ASSERT(index_ < MAX_USB_INSTANCES);
  ASSERT(instances_[index_] == nullptr || instances_[index_] == this);

  instances_[index_] = this;
  Increment(diag_start_count_);

  ConfigureHardware();
  ResetDcdData();

  if (!IsInited())
  {
    USB::DeviceCore::Init(false);
  }

  const uint32_t int_mask = HPM_USB_INTR_UI | HPM_USB_INTR_UEI | HPM_USB_INTR_PCI |
                            HPM_USB_INTR_URI | HPM_USB_INTR_SLI;
  usb_clear_status_flags(instance_, usb_get_status_flags(instance_));
  usb_enable_interrupts(instance_, int_mask);

  intc_m_enable_irq_with_priority(irq_, 2);
  running_ = true;
  usb_dcd_connect(instance_);
}

void HPMUSBDevice::Stop(bool in_isr)
{
  if (running_)
  {
    usb_dcd_disconnect(instance_);
    usb_disable_interrupts(instance_, 0xFFFFFFFFu);
    intc_m_disable_irq(irq_);
  }

  if (IsInited())
  {
    USB::DeviceCore::Deinit(in_isr);
  }

  usb_dcd_deinit(instance_);
  running_ = false;

  if (index_ < MAX_USB_INSTANCES && instances_[index_] == this)
  {
    instances_[index_] = nullptr;
  }
}

ErrorCode HPMUSBDevice::SetAddress(uint8_t address, USB::DeviceCore::Context context)
{
  if (context == USB::DeviceCore::Context::STATUS_IN_ARMED)
  {
    usb_dcd_set_address(instance_, address);
    diag_last_address_ = address;
  }
  return ErrorCode::OK;
}

HPMUSBDevice::Diagnostics HPMUSBDevice::GetDiagnostics() const
{
  Diagnostics diag{};
  diag.start_count = diag_start_count_;
  diag.irq_count = diag_irq_count_;
  diag.reset_count = diag_reset_count_;
  diag.setup_count = diag_setup_count_;
  diag.complete_count = diag_complete_count_;
  diag.error_count = diag_error_count_;
  diag.port_change_count = diag_port_change_count_;
  diag.suspend_count = diag_suspend_count_;
  diag.transfer_error_count = diag_transfer_error_count_;
  diag.last_status = diag_last_status_;
  diag.last_complete = diag_last_complete_;
  diag.last_setup_status = diag_last_setup_status_;
  diag.last_usbcmd = diag_last_usbcmd_;
  diag.last_usbmode = diag_last_usbmode_;
  diag.last_portsc1 = diag_last_portsc1_;
  if (instance_ != nullptr)
  {
    diag.current_status = usb_get_status_flags(instance_);
    diag.current_setup_status = usb_dcd_get_edpt_setup_status(instance_);
    diag.current_complete = usb_dcd_get_edpt_complete_status(instance_);
    diag.current_usbcmd = instance_->USBCMD;
    diag.current_usbmode = instance_->USBMODE;
    diag.current_portsc1 = instance_->PORTSC1;
    diag.current_otgsc = instance_->OTGSC;
    diag.current_phy_ctrl0 = instance_->PHY_CTRL0;
    diag.current_phy_ctrl1 = instance_->PHY_CTRL1;
    diag.current_phy_status = instance_->PHY_STATUS;
  }
  diag.last_address = diag_last_address_;
  diag.last_setup_request_type = diag_last_setup_request_type_;
  diag.last_setup_request = diag_last_setup_request_;
  diag.last_setup_value = diag_last_setup_value_;
  diag.last_setup_index = diag_last_setup_index_;
  diag.last_setup_length = diag_last_setup_length_;
  return diag;
}

void HPMUSBDevice::HandleInterrupt()
{
  if (!running_)
  {
    usb_clear_status_flags(instance_, usb_get_status_flags(instance_));
    return;
  }

  uint32_t status = usb_get_status_flags(instance_);
  status &= usb_get_interrupts(instance_);
  usb_clear_status_flags(instance_, status);
  Increment(diag_irq_count_);
  diag_last_status_ = status;
  diag_last_usbcmd_ = instance_->USBCMD;
  diag_last_usbmode_ = instance_->USBMODE;
  diag_last_portsc1_ = instance_->PORTSC1;

  if (status == 0)
  {
    return;
  }

  if ((status & HPM_USB_INTR_UEI) != 0)
  {
    Increment(diag_error_count_);
  }

  if ((status & HPM_USB_INTR_PCI) != 0)
  {
    Increment(diag_port_change_count_);
  }

  if ((status & HPM_USB_INTR_SLI) != 0)
  {
    Increment(diag_suspend_count_);
  }

  if ((status & HPM_USB_INTR_URI) != 0)
  {
    HandleBusReset(true);
  }

  if ((status & HPM_USB_INTR_UI) != 0)
  {
    HandleTransferComplete(true);
    HandleSetupPacket(true);
  }
}

void HPMUSBDevice::HandleBusReset(bool in_isr)
{
  Increment(diag_reset_count_);
  usb_dcd_bus_reset(instance_, PacketSizeValue(ep0_packet_size_));

  if (IsInited())
  {
    USB::DeviceCore::Deinit(in_isr);
  }

  ResetDcdData();
  USB::DeviceCore::Init(in_isr);
  diag_last_usbcmd_ = instance_->USBCMD;
  diag_last_usbmode_ = instance_->USBMODE;
  diag_last_portsc1_ = instance_->PORTSC1;
}

uint8_t HPMUSBDevice::ResolveIndex(USB_Type* instance)
{
#if defined(HPM_USB0)
  if (instance == HPM_USB0)
  {
    return 0;
  }
#endif
  return INVALID_INSTANCE_INDEX;
}

HPMUSBDevice* HPMUSBDevice::GetByIrq(uint32_t irq)
{
#if defined(IRQn_USB0)
  if (irq == IRQn_USB0)
  {
    return instances_[0];
  }
#endif
  return nullptr;
}

HPMUSBDqh* HPMUSBDevice::Qhd(uint8_t ep_idx)
{
  ASSERT(index_ < MAX_USB_INSTANCES);
  ASSERT(ep_idx < USB_SOS_DCD_MAX_QHD_COUNT);
  return &g_hpm_usb_dcd_data[index_].qhd[ep_idx];
}

HPMUSBDtd* HPMUSBDevice::Qtd(uint8_t ep_idx)
{
  ASSERT(index_ < MAX_USB_INSTANCES);
  ASSERT(ep_idx < USB_SOS_DCD_MAX_QHD_COUNT);
  return &g_hpm_usb_dcd_data[index_].qtd[ep_idx * USB_SOC_DCD_QTD_COUNT_EACH_ENDPOINT];
}

HPMUSBEndpoint* HPMUSBDevice::Endpoint(uint8_t ep_idx)
{
  ASSERT(ep_idx < USB_SOS_DCD_MAX_QHD_COUNT);
  return endpoints_[ep_idx];
}

bool HPMUSBDevice::OpenEndpoint(uint8_t ep_addr, USB::Endpoint::Type type,
                                uint16_t max_packet_size)
{
  const uint8_t ep_idx = EpAddrToIndex(ep_addr);
  auto* qhd = Qhd(ep_idx);

  std::memset(qhd, 0, sizeof(*qhd));
  qhd->caps = BuildQhCaps(max_packet_size, type, ep_idx == 0);
  qhd->qtd_overlay.next = USB_SOC_DCD_QTD_NEXT_INVALID;

  usb_endpoint_config_t cfg{};
  cfg.ep_addr = ep_addr;
  cfg.xfer = static_cast<uint8_t>(type);
  cfg.max_packet_size = max_packet_size;

  usb_dcd_edpt_open(instance_, &cfg);
  return true;
}

bool HPMUSBDevice::SubmitTransfer(uint8_t ep_addr, uint8_t* buffer, uint32_t total_bytes)
{
  const uint8_t ep_num = ep_addr & 0x0Fu;
  const uint8_t ep_idx = EpAddrToIndex(ep_addr);

  if (ep_num >= USB_SOC_DCD_MAX_ENDPOINT_COUNT)
  {
    return false;
  }

  uint32_t qtd_count = (total_bytes + 0x3FFFu) / 0x4000u;
  if (qtd_count == 0)
  {
    qtd_count = 1;
  }
  if (qtd_count > USB_SOC_DCD_QTD_COUNT_EACH_ENDPOINT)
  {
    return false;
  }

  if (ep_num == 0)
  {
    while ((usb_dcd_get_edpt_setup_status(instance_) & HPM_BITSMASK(1, 0)) != 0)
    {
    }
  }

  auto* qhd = Qhd(ep_idx);
  auto* first = Qtd(ep_idx);
  auto* prev = static_cast<HPMUSBDtd*>(nullptr);
  uint8_t* next_buffer = buffer;
  uint32_t remain = total_bytes;

  qhd->attached_buffer = ToUsbAddress(buffer);

  for (uint32_t i = 0; i < qtd_count; ++i)
  {
    auto* qtd = first + i;
    const uint32_t chunk = (remain > 0x4000u) ? 0x4000u : remain;
    remain -= chunk;

    std::memset(qtd, 0, sizeof(*qtd));
    qtd->next = USB_SOC_DCD_QTD_NEXT_INVALID;
    qtd->expected_bytes = static_cast<uint16_t>(chunk);
    qtd->in_use = 1;

    if (next_buffer != nullptr)
    {
      uint32_t page = ToUsbAddress(next_buffer);
      qtd->buffer[0] = page;
      page &= 0xFFFFF000u;
      for (uint8_t p = 1; p < USB_SOC_DCD_QHD_BUFFER_COUNT; ++p)
      {
        page += 0x1000u;
        qtd->buffer[p] = page;
      }
      next_buffer += chunk;
    }

    qtd->token = BuildDtdToken(chunk, i == (qtd_count - 1u));

    if (prev != nullptr)
    {
      prev->next = reinterpret_cast<uint32_t>(qtd);
    }
    prev = qtd;
  }

  qhd->attached_qtd = first;
  qhd->qtd_overlay.next = ToUsbAddress(first);

  usb_dcd_edpt_xfer(instance_, ep_idx);
  return true;
}

void HPMUSBDevice::RegisterEndpoint(HPMUSBEndpoint* ep)
{
  ASSERT(ep != nullptr);
  const uint8_t ep_idx = EpAddrToIndex(
      USB::Endpoint::EPNumberToAddr(ep->GetNumber(), ep->AvailableDirection()));
  ASSERT(ep_idx < USB_SOS_DCD_MAX_QHD_COUNT);
  endpoints_[ep_idx] = ep;
}

void HPMUSBDevice::ResetDcdData()
{
  ASSERT(index_ < MAX_USB_INSTANCES);
  auto& dcd = g_hpm_usb_dcd_data[index_];
  std::memset(&dcd, 0, sizeof(dcd));

  const uint16_t ep0_mps = PacketSizeValue(ep0_packet_size_);
  dcd.qhd[0].caps = BuildQhCaps(ep0_mps, USB::Endpoint::Type::CONTROL, true);
  dcd.qhd[1].caps = BuildQhCaps(ep0_mps, USB::Endpoint::Type::CONTROL, false);
  dcd.qhd[0].qtd_overlay.next = USB_SOC_DCD_QTD_NEXT_INVALID;
  dcd.qhd[1].qtd_overlay.next = USB_SOC_DCD_QTD_NEXT_INVALID;

  usb_dcd_set_edpt_list_addr(instance_, ToUsbAddress(dcd.qhd));
}

void HPMUSBDevice::HandleTransferComplete(bool in_isr)
{
  const uint32_t complete = usb_dcd_get_edpt_complete_status(instance_);
  if (complete == 0)
  {
    return;
  }

  usb_dcd_clear_edpt_complete_status(instance_, complete);
  Increment(diag_complete_count_);
  diag_last_complete_ = complete;

  for (uint8_t ep_idx = 0; ep_idx < USB_SOS_DCD_MAX_QHD_COUNT; ++ep_idx)
  {
    if ((complete & HPM_BITSMASK(1, EpIndexToCompleteBit(ep_idx))) == 0)
    {
      continue;
    }

    auto* ep = Endpoint(ep_idx);
    if (ep != nullptr)
    {
      ep->TransferComplete(in_isr);
    }
    else
    {
      auto* qhd = Qhd(ep_idx);
      if (qhd->attached_qtd != nullptr)
      {
        qhd->attached_qtd->in_use = 0;
        qhd->attached_qtd = nullptr;
      }
    }
  }
}

void HPMUSBDevice::HandleSetupPacket(bool in_isr)
{
  const uint32_t setup_status = usb_dcd_get_edpt_setup_status(instance_);
  if (setup_status == 0)
  {
    return;
  }

  usb_dcd_clear_edpt_setup_status(instance_, setup_status);
  Increment(diag_setup_count_);
  diag_last_setup_status_ = setup_status;

  auto* ep0_out = GetEndpoint0Out();
  auto* ep0_in = GetEndpoint0In();
  if (ep0_out != nullptr)
  {
    ep0_out->SetState(USB::Endpoint::State::IDLE);
  }
  if (ep0_in != nullptr)
  {
    ep0_in->SetState(USB::Endpoint::State::IDLE);
  }

  USB::SetupPacket setup{};
  const volatile usb_control_request_t& src = Qhd(0)->setup_request;
  setup.bmRequestType = src.bmRequestType;
  setup.bRequest = src.bRequest;
  setup.wValue = src.wValue;
  setup.wIndex = src.wIndex;
  setup.wLength = src.wLength;
  diag_last_setup_request_type_ = setup.bmRequestType;
  diag_last_setup_request_ = setup.bRequest;
  diag_last_setup_value_ = setup.wValue;
  diag_last_setup_index_ = setup.wIndex;
  diag_last_setup_length_ = setup.wLength;

  OnSetupPacket(in_isr, &setup);
}

void HPMUSBDevice::ConfigureHardware()
{
#if LIBXR_HPM_USB_HAS_BOARD_HELPER
  if (auto_board_init_)
  {
    board_init_usb(instance_);
  }
  else
#endif
  {
#if defined(clock_usb0)
    if (instance_ == HPM_USB0)
    {
      clock_add_to_group(clock_usb0, 0);
    }
#endif
  }

  usb_dcd_init(instance_);

  if (board_config_.post_dcd_init != nullptr)
  {
    board_config_.post_dcd_init(instance_);
  }

  if (speed_ == USB::Speed::FULL)
  {
    instance_->PORTSC1 |= USB_PORTSC1_PFSC_MASK;
  }
}

#if defined(HPM_USB0) && defined(IRQn_USB0)
SDK_DECLARE_EXT_ISR_M(IRQn_USB0, libxr_hpm_usb0_isr)
void libxr_hpm_usb0_isr(void)
{
  auto* usb = LibXR::HPMUSBDevice::GetByIrq(IRQn_USB0);
  if (usb != nullptr)
  {
    usb->HandleInterrupt();
  }
}
#endif
