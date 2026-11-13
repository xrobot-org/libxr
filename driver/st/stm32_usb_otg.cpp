#include "stm32_usb_otg.hpp"

#if defined(HAL_PCD_MODULE_ENABLED) && (defined(USB_OTG_FS) || defined(USB_OTG_HS))

#include <cstring>

#include "stm32_dcache.hpp"
#include "stm32_usb_wrap.hpp"

using namespace LibXR;

namespace
{
constexpr uint8_t OTG_FS_INDEX = 0;
constexpr uint8_t OTG_HS_INDEX = 1;

#if defined(USB_OTG_FS)
#if defined(USB_OTG_FS_MAX_IN_ENDPOINTS)
constexpr uint8_t OTG_FS_ENDPOINTS = USB_OTG_FS_MAX_IN_ENDPOINTS;
#elif defined(STM32H7) || defined(STM32N6)
constexpr uint8_t OTG_FS_ENDPOINTS = 9;
#elif defined(STM32F1)
// F105/F107：EP0 加 3 个 IN、3 个 OUT 端点（RM0008 "OTG_FS main features"）。
// F105/F107: EP0 plus 3 IN and 3 OUT endpoints (RM0008 "OTG_FS main features").
constexpr uint8_t OTG_FS_ENDPOINTS = 4;
#else
constexpr uint8_t OTG_FS_ENDPOINTS = 6;
#endif
#if defined(USB_OTG_FS_TOTAL_FIFO_SIZE)
constexpr size_t OTG_FS_FIFO = USB_OTG_FS_TOTAL_FIFO_SIZE;
#elif defined(STM32H7) || defined(STM32N6)
constexpr size_t OTG_FS_FIFO = 4096;
#else
constexpr size_t OTG_FS_FIFO = 1280;
#endif
#endif

#if defined(USB_OTG_HS)
#if defined(USB_OTG_HS_MAX_IN_ENDPOINTS)
constexpr uint8_t OTG_HS_ENDPOINTS = USB_OTG_HS_MAX_IN_ENDPOINTS;
#else
constexpr uint8_t OTG_HS_ENDPOINTS = 9;
#endif
#if defined(USB_OTG_HS_TOTAL_FIFO_SIZE)
constexpr size_t OTG_HS_FIFO = USB_OTG_HS_TOTAL_FIFO_SIZE;
#else
constexpr size_t OTG_HS_FIFO = 4096;
#endif
#endif

constexpr uint32_t PKTCNT_POS = 19U;
// DIEPTSIZ 的 MULCNT 字段（位 30:29）；器件头文件里的名字不统一（MULCNT、MCNT）。
// MULCNT field of DIEPTSIZ (bits 30:29); the device headers name it differently (MULCNT,
// MCNT).
constexpr uint32_t MULCNT_POS = 29U;
// EP0 OUT 可接收的连续 SETUP 包数。
// Back-to-back SETUP packets EP0 OUT accepts.
constexpr uint32_t SETUP_COUNT = 3U;
constexpr size_t SETUP_SIZE = 8U;
constexpr size_t EP0_DMA_SIZE = 64U;

uint32_t Address(const void* pointer)
{
  return static_cast<uint32_t>(reinterpret_cast<uintptr_t>(pointer));
}

/// DIEPCTL0 的 MPSIZ 编码：64、32、16 和 8 字节。
/// MPSIZ code of DIEPCTL0: 64, 32, 16 and 8 bytes.
uint32_t Ep0SizeCode(uint32_t packet)
{
  return packet >= 64U ? 0U : packet >= 32U ? 1U : packet >= 16U ? 2U : 3U;
}

uint32_t EndpointType(USB::Endpoint::Type type)
{
  return static_cast<uint32_t>(type) << 18U;
}
}  // namespace

// ---------------------------------------------------------------------------
// Device
// ---------------------------------------------------------------------------

STM32USBOtgDevice::STM32USBOtgDevice(
    uint8_t index, USB_OTG_GlobalTypeDef* core, uint8_t endpoint_count, size_t total_fifo,
    const HardwareConfig& hw, size_t rx_fifo_size,
    const std::initializer_list<RawData>& rx_ep_cfgs,
    const std::initializer_list<EPInConfig>& tx_ep_cfgs,
    USB::DeviceDescriptor::PacketSize0 packet_size, uint16_t vid, uint16_t pid,
    uint16_t bcd,
    const std::initializer_list<const USB::DescriptorStrings::LanguagePack*>& lang_list,
    const std::initializer_list<const std::initializer_list<USB::ConfigDescriptorItem*>>&
        configs,
    ConstRawData uid)
    : USB::EndpointPool(rx_ep_cfgs.size() + tx_ep_cfgs.size()),
      STM32USBDeviceCore(*this, USB::USBSpec::USB_2_1,
                         hw.high_speed ? USB::Speed::HIGH : USB::Speed::FULL, packet_size,
                         vid, pid, bcd, lang_list, configs, uid),
      index_(index),
      core_(core),
      endpoint_count_(endpoint_count),
      hw_(hw),
      bus_speed_(hw.high_speed ? USB::Speed::HIGH : USB::Speed::FULL)
{
  REQUIRE(index < MAX_DEVICES && endpoint_count <= MAX_ENDPOINTS);
  REQUIRE(rx_ep_cfgs.size() > 0 && rx_ep_cfgs.size() <= endpoint_count);
  REQUIRE(tx_ep_cfgs.size() > 0 && tx_ep_cfgs.size() <= endpoint_count);
  REQUIRE(rx_fifo_size % 4U == 0U && 64U * rx_ep_cfgs.size() <= rx_fifo_size);
  // RX FIFO 除了一个最大包及其状态字，还要容纳 SETUP（5 × 控制端点数 + 8）、每个 OUT
  // 端点的传输完成状态（2 × OUT 端点数，含 EP0）和全局 OUT NAK（1），单位为 32 位字
  // （RM0090 "FIFO RAM allocation"，设备模式）。剩下的才是一个 OUT 包能用的空间。
  // Besides one largest packet and its status word the RX FIFO holds the SETUP packets
  // (5 x control endpoints + 8), the transfer-complete status of every OUT endpoint (2 x
  // OUT endpoints, EP0 included) and the global OUT NAK (1), in 32-bit words (RM0090
  // "FIFO RAM allocation", device mode). What is left is the room of one OUT packet.
  const size_t rx_overhead = (5U + 8U + 1U + 2U * rx_ep_cfgs.size() + 1U) * 4U;
  REQUIRE(rx_fifo_size > rx_overhead);
  rx_packet_limit_ = rx_fifo_size - rx_overhead;
  REQUIRE(!hw.dma || (hw.ep0_dma_buffer.addr_ != nullptr &&
                      hw.ep0_dma_buffer.size_ >= EP0_DMA_SIZE &&
                      Address(hw.ep0_dma_buffer.addr_) % 32U == 0U));

  SetHardwareGuard(
      this,
      [](void*) -> uintptr_t
      {
        const uint32_t saved = __get_PRIMASK();
        __disable_irq();
        __DMB();
        return saved;
      },
      [](void*, uintptr_t saved)
      {
        __DMB();
        __set_PRIMASK(static_cast<uint32_t>(saved));
      });

  // 时钟、引脚和 NVIC 由 CubeMX 配置；HAL_PCD_MspInit() 只读取 Instance。
  // Clocks, pins and NVIC come from CubeMX; HAL_PCD_MspInit() only reads Instance.
  {
    PCD_HandleTypeDef msp = {};
    msp.Instance = core_;
    HAL_PCD_MspInit(&msp);
  }
  STM32USBRequireWrap();

  // 与 HAL_PCD_Init() 相同的 core 初始化，但不使用句柄。
  // The same core initialization as HAL_PCD_Init(), without the handle.
  core_->GAHBCFG &= ~USB_OTG_GAHBCFG_GINT;
  USB_OTG_CfgTypeDef cfg = {};
  cfg.dev_endpoints = endpoint_count_;
  // 只有全速 OTG 的系列（F105/F107、L4、U575 等），HAL 没有高速和 ULPI 的定义。
  // On families whose OTG is full speed only (F105/F107, L4, U575 ...) HAL has no
  // definitions for high speed and ULPI.
#if defined(USBD_HS_SPEED)
  cfg.speed = hw.high_speed ? USBD_HS_SPEED : USBD_FS_SPEED;
#else
  REQUIRE(!hw.high_speed);
  cfg.speed = USBD_FS_SPEED;
#endif
  cfg.dma_enable = hw.dma ? 1U : 0U;
  cfg.phy_itface = USB_OTG_EMBEDDED_PHY;
#if defined(USB_OTG_ULPI_PHY)
  if (hw.phy == Phy::ULPI) cfg.phy_itface = USB_OTG_ULPI_PHY;
#else
  REQUIRE(hw.phy != Phy::ULPI);
#endif
#if defined(USB_OTG_HS_EMBEDDED_PHY)
  if (hw.phy == Phy::EMBEDDED_HS) cfg.phy_itface = USB_OTG_HS_EMBEDDED_PHY;
#else
  REQUIRE(hw.phy != Phy::EMBEDDED_HS);
#endif
  cfg.vbus_sensing_enable = hw.vbus_sensing ? 1U : 0U;
  REQUIRE(USB_CoreInit(core_, cfg) == HAL_OK);
  // GSNPSID 紧跟在 CID 之后（偏移 0x40）；器件头文件没有这个成员，HAL 也这样读。
  // GSNPSID follows CID (offset 0x40); the device headers have no member for it, and HAL
  // reads it the same way.
  core_id_ = *(reinterpret_cast<const volatile uint32_t*>(&core_->CID) + 1U);
  REQUIRE(USB_SetCurrentMode(core_, USB_DEVICE_MODE) == HAL_OK);
  REQUIRE(USB_DevInit(core_, cfg) == HAL_OK);
  (void)USB_DevDisconnect(core_);
  core_->GINTMSK = 0U;
  core_->GINTSTS = 0xBFFFFFFFU;

  size_t fifo_bytes = rx_fifo_size;
  for (const auto& tx : tx_ep_cfgs)
  {
    REQUIRE(tx.fifo_size >= 8U && tx.fifo_size % 4U == 0U);
    fifo_bytes += tx.fifo_size;
  }
  REQUIRE(fifo_bytes <= total_fifo);

  auto rx = rx_ep_cfgs.begin();
  auto tx = tx_ep_cfgs.begin();
  auto* ep0_out = new STM32OtgEndpoint(*this, USB::Endpoint::EPNumber::EP0,
                                       USB::Endpoint::Direction::OUT, rx_fifo_size, *rx);
  auto* ep0_in =
      new STM32OtgEndpoint(*this, USB::Endpoint::EPNumber::EP0,
                           USB::Endpoint::Direction::IN, tx->fifo_size, tx->buffer);
  SetEndpoint0(ep0_in, ep0_out);
  auto number = USB::Endpoint::EPNumber::EP1;
  for (++rx; rx != rx_ep_cfgs.end(); ++rx)
  {
    Put(new STM32OtgEndpoint(*this, number, USB::Endpoint::Direction::OUT, rx_fifo_size,
                             *rx));
    number = USB::Endpoint::NextEPNumber(number);
  }
  number = USB::Endpoint::EPNumber::EP1;
  for (++tx; tx != tx_ep_cfgs.end(); ++tx)
  {
    Put(new STM32OtgEndpoint(*this, number, USB::Endpoint::Direction::IN, tx->fifo_size,
                             tx->buffer));
    number = USB::Endpoint::NextEPNumber(number);
  }
  ProgramFifos();
}

void STM32USBOtgDevice::ProgramFifos()
{
  // FIFO RAM：先 RX，再 TX0 及各 IN 端点的 TX FIFO，按顺序排布（IN 端点 n
  // 使用 TX FIFO n）。
  // FIFO RAM: RX first, then TX0 and the TX FIFO of each IN endpoint in order (IN
  // endpoint n uses TX FIFO n).
  const uint32_t rx_words = static_cast<uint32_t>(out_[0]->fifo_size_ / 4U);
  core_->GRXFSIZ = rx_words;
  uint32_t offset = rx_words;
  for (uint8_t fifo = 0; fifo < endpoint_count_ && in_[fifo] != nullptr; ++fifo)
  {
    const uint32_t words = static_cast<uint32_t>(in_[fifo]->fifo_size_ / 4U);
    if (fifo == 0U)
      core_->DIEPTXF0_HNPTXFSIZ = (words << 16U) | offset;
    else
      core_->DIEPTXF[fifo - 1U] = (words << 16U) | offset;
    offset += words;
  }
}

USB_OTG_DeviceTypeDef* STM32USBOtgDevice::Device() const
{
  return reinterpret_cast<USB_OTG_DeviceTypeDef*>(reinterpret_cast<uintptr_t>(core_) +
                                                  USB_OTG_DEVICE_BASE);
}

USB_OTG_INEndpointTypeDef* STM32USBOtgDevice::InEp(uint8_t num) const
{
  return reinterpret_cast<USB_OTG_INEndpointTypeDef*>(reinterpret_cast<uintptr_t>(core_) +
                                                      USB_OTG_IN_ENDPOINT_BASE +
                                                      num * USB_OTG_EP_REG_SIZE);
}

USB_OTG_OUTEndpointTypeDef* STM32USBOtgDevice::OutEp(uint8_t num) const
{
  return reinterpret_cast<USB_OTG_OUTEndpointTypeDef*>(
      reinterpret_cast<uintptr_t>(core_) + USB_OTG_OUT_ENDPOINT_BASE +
      num * USB_OTG_EP_REG_SIZE);
}

volatile uint32_t& STM32USBOtgDevice::Fifo(uint8_t num) const
{
  return *reinterpret_cast<volatile uint32_t*>(
      reinterpret_cast<uintptr_t>(core_) + USB_OTG_FIFO_BASE + num * USB_OTG_FIFO_SIZE);
}

uint32_t STM32USBOtgDevice::CoreId() const { return core_id_; }

void STM32USBOtgDevice::Start(bool)
{
  USB::EndpointPool::HardwareScope hardware(this);
  devices_[index_] = this;
  Connect();
}

void STM32USBOtgDevice::Connect()
{
  discard_ = false;
  core_->GINTSTS = 0xBFFFFFFFU;
  uint32_t mask = USB_OTG_GINTMSK_USBRST | USB_OTG_GINTMSK_ENUMDNEM |
                  USB_OTG_GINTMSK_USBSUSPM | USB_OTG_GINTMSK_WUIM |
                  USB_OTG_GINTMSK_IEPINT | USB_OTG_GINTMSK_OEPINT;
  if (!hw_.dma) mask |= USB_OTG_GINTMSK_RXFLVLM;
  if (hw_.vbus_sensing) mask |= USB_OTG_GINTMSK_SRQIM | USB_OTG_GINTMSK_OTGINT;
  core_->GINTMSK = mask;
  UpdateTick();
  core_->GAHBCFG |= USB_OTG_GAHBCFG_GINT;
  (void)USB_DevConnect(core_);
}

void STM32USBOtgDevice::Stop(bool in_isr)
{
  {
    USB::EndpointPool::HardwareScope hardware(this);
    (void)USB_DevDisconnect(core_);
    core_->GAHBCFG &= ~USB_OTG_GAHBCFG_GINT;
    if (devices_[index_] == this) devices_[index_] = nullptr;
  }
  // 离开总线后不再有事件到达端点；确认所有等待中的停止。
  // Off the bus nothing reaches an endpoint any more; confirm every waiting stop.
  ForceStopAll(in_isr);
}

void STM32USBOtgDevice::OnStopTimeout(bool in_isr)
{
  // 永不确认的停止让 core 处于未知状态：设备离开总线，确认所有等待中的停止并结束会话。
  // 设备保持断开，直到应用调用 Stop() 和 Start()。
  // A stop that never confirms leaves the core in an unknown state. The device leaves the
  // bus, every waiting stop is confirmed and the session ends. The device stays off the
  // bus until the application calls Stop() and Start().
  {
    USB::EndpointPool::HardwareScope hardware(this);
    (void)USB_DevDisconnect(core_);
    core_->GAHBCFG &= ~USB_OTG_GAHBCFG_GINT;
  }
  ForceStopAll(in_isr);
  OnDisconnect(in_isr);
}

void STM32USBOtgDevice::OnStopsSettled()
{
  USB::EndpointPool::HardwareScope hardware(this);
  UpdateTick();
}

USB::Speed STM32USBOtgDevice::ReadBusSpeed() const { return bus_speed_; }

ErrorCode STM32USBOtgDevice::SetAddress(uint8_t address, USB::ControlContext context)
{
  // core 在状态阶段之前采用新地址（RM0090 "Endpoint initialization on
  // SetAddress command"）。
  // The core takes the new address before the status stage (RM0090 "Endpoint
  // initialization on SetAddress command").
  if (context == USB::ControlContext::SETUP_BEFORE_STATUS)
  {
    USB::EndpointPool::HardwareScope hardware(this);
    Device()->DCFG =
        (Device()->DCFG & ~USB_OTG_DCFG_DAD) |
        ((static_cast<uint32_t>(address) << USB_OTG_DCFG_DAD_Pos) & USB_OTG_DCFG_DAD);
  }
  return ErrorCode::OK;
}

void STM32USBOtgDevice::OnControllerReset() { discard_ = false; }

void STM32USBOtgDevice::OnControlReady() { ArmSetup(); }

void STM32USBOtgDevice::ArmSetup()
{
  auto* out0 = OutEp(0);
  // 较新的 core 在 EP0 OUT 仍使能时不得重新配置（与 HAL 一致）。
  // Newer cores must not be reprogrammed while EP0 OUT is still enabled (as HAL does).
  if (CoreId() > USB_OTG_CORE_ID_300A && (out0->DOEPCTL & USB_OTG_DOEPCTL_EPENA) != 0U)
    return;
  out0->DOEPTSIZ =
      USB_OTG_DOEPTSIZ_STUPCNT | (1U << PKTCNT_POS) | (SETUP_COUNT * SETUP_SIZE);
  if (hw_.dma)
  {
    out0->DOEPDMA = Address(hw_.ep0_dma_buffer.addr_);
    out0->DOEPCTL |= USB_OTG_DOEPCTL_EPENA | USB_OTG_DOEPCTL_USBAEP;
  }
}

void STM32USBOtgDevice::Dispatch(USB_OTG_GlobalTypeDef* core)
{
  uint8_t index = MAX_DEVICES;
#if defined(USB_OTG_FS)
  if (core == USB_OTG_FS) index = OTG_FS_INDEX;
#endif
#if defined(USB_OTG_HS)
  if (core == USB_OTG_HS) index = OTG_HS_INDEX;
#endif
  if (index >= MAX_DEVICES) return;
  STM32USBOtgDevice* device = devices_[index];
  if (device == nullptr)
  {
    // 未启动：在驱动接管控制器之前保持中断线静默。
    // Not started: keep the line quiet until the driver owns the controller.
    core->GAHBCFG &= ~USB_OTG_GAHBCFG_GINT;
    return;
  }
  USB::EndpointPool::InterruptScope batch(*device);
  device->HandleInterrupt();
}

void STM32USBOtgDevice::HandleInterrupt()
{
  // 一次快照、固定顺序、不重复读取：先处理总线复位，
  // 使同一快照中旧会话的端点事件随该复位结束。
  // One snapshot, a fixed order, no re-read: a bus reset first, so that endpoint events
  // of the old session in the same snapshot are ended by the reset.
  const uint32_t status = core_->GINTSTS & core_->GINTMSK;
  if (status & USB_OTG_GINTSTS_USBRST) OnUsbReset();
  if (status & USB_OTG_GINTSTS_ENUMDNE) OnEnumerationDone();
  if (status & USB_OTG_GINTSTS_RXFLVL) DrainRxFifo();
  if (status & USB_OTG_GINTSTS_OEPINT) HandleOutEndpoints();
  if (status & USB_OTG_GINTSTS_IEPINT) HandleInEndpoints();
  if (status & USB_OTG_GINTSTS_BOUTNAKEFF) HandleGlobalOutNak();
  if (status & USB_OTG_GINTSTS_SOF) HandleSof();
  if (status & USB_OTG_GINTSTS_USBSUSP)
  {
    core_->GINTSTS = USB_OTG_GINTSTS_USBSUSP;
    if ((Device()->DSTS & USB_OTG_DSTS_SUSPSTS) != 0U) OnSuspend(true);
  }
  if (status & USB_OTG_GINTSTS_WKUINT)
  {
    core_->GINTSTS = USB_OTG_GINTSTS_WKUINT;
    Device()->DCTL &= ~USB_OTG_DCTL_RWUSIG;
    OnResume(true);
  }
  if (status & USB_OTG_GINTSTS_SRQINT) core_->GINTSTS = USB_OTG_GINTSTS_SRQINT;
  if (status & USB_OTG_GINTSTS_OTGINT)
  {
    const uint32_t otg = core_->GOTGINT;
    core_->GOTGINT = otg;
    if ((otg & USB_OTG_GOTGINT_SEDET) != 0U) OnDisconnect(true);
  }
  PollTxFlush();
}

void STM32USBOtgDevice::OnUsbReset()
{
  core_->GINTSTS = USB_OTG_GINTSTS_USBRST;
  // 旧会话的数据被丢弃，直到 owner 应用该复位。
  // Data of the old session is dropped until the owner has applied the reset.
  discard_ = true;
  Device()->DCTL &= ~USB_OTG_DCTL_RWUSIG;
  RequestTxFlush(FLUSH_ALL);
  for (uint8_t i = 0; i < endpoint_count_; ++i)
  {
    // 处于停止中的端点保留其事件；停止流程本身会轮询寄存器。
    // Endpoints in a stop keep their events; the stop polls the registers anyway.
    if (in_[i] == nullptr || in_[i]->stop_stage_ == STM32OtgEndpoint::StopStage::NONE)
      InEp(i)->DIEPINT = 0xFB7FU;
    InEp(i)->DIEPCTL &= ~USB_OTG_DIEPCTL_STALL;
    if (out_[i] == nullptr || out_[i]->stop_stage_ == STM32OtgEndpoint::StopStage::NONE)
      OutEp(i)->DOEPINT = 0xFB7FU;
    OutEp(i)->DOEPCTL &= ~USB_OTG_DOEPCTL_STALL;
    OutEp(i)->DOEPCTL |= USB_OTG_DOEPCTL_SNAK;
  }
  Device()->DAINTMSK |= 0x10001U;
  Device()->DOEPMSK |= USB_OTG_DOEPMSK_STUPM | USB_OTG_DOEPMSK_XFRCM |
                       USB_OTG_DOEPMSK_EPDM | USB_OTG_DOEPMSK_OTEPSPRM;
  Device()->DIEPMSK |= USB_OTG_DIEPMSK_TOM | USB_OTG_DIEPMSK_XFRCM | USB_OTG_DIEPMSK_EPDM;
  Device()->DCFG &= ~USB_OTG_DCFG_DAD;
  ArmSetup();
}

void STM32USBOtgDevice::OnEnumerationDone()
{
  core_->GINTSTS = USB_OTG_GINTSTS_ENUMDNE;
  (void)USB_ActivateSetup(core_);
  const uint8_t speed = USB_GetDevSpeed(core_);
#if defined(USBD_HS_SPEED)
  bus_speed_ = speed == USBD_HS_SPEED ? USB::Speed::HIGH : USB::Speed::FULL;
#else
  bus_speed_ = USB::Speed::FULL;
#endif
  (void)USB_SetTurnaroundTime(core_, HAL_RCC_GetHCLKFreq(), speed);
  OnBusReset(true);
}

void STM32USBOtgDevice::DrainRxFifo()
{
  // 有界：每次中断最多处理这么多条状态项；其余由电平中断带出。
  // Bounded: at most this many status entries per interrupt; the level interrupt brings
  // the rest.
  for (unsigned i = 0; i < 16U && (core_->GINTSTS & USB_OTG_GINTSTS_RXFLVL) != 0U; ++i)
  {
    const uint32_t entry = core_->GRXSTSP;
    const uint8_t num = static_cast<uint8_t>(entry & USB_OTG_GRXSTSP_EPNUM);
    const size_t bytes = (entry & USB_OTG_GRXSTSP_BCNT) >> USB_OTG_GRXSTSP_BCNT_Pos;
    const uint32_t kind = (entry & USB_OTG_GRXSTSP_PKTSTS) >> USB_OTG_GRXSTSP_PKTSTS_Pos;
    if (kind == STS_DATA_UPDT)
    {
      STM32OtgEndpoint* ep = num < MAX_ENDPOINTS ? out_[num] : nullptr;
      if (ep != nullptr && (!discard_ || num == 0U))
      {
        ep->ReadFifo(bytes);
      }
      else
      {
        for (size_t word = 0; word < (bytes + 3U) / 4U; ++word)
        {
          const uint32_t dropped = Fifo(0);
          UNUSED(dropped);
        }
      }
    }
    else if (kind == STS_SETUP_UPDT)
    {
      for (size_t word = 0; word < (bytes + 3U) / 4U; ++word)
      {
        const uint32_t value = Fifo(0);
        if (word < SETUP_SIZE / 4U) std::memcpy(setup_ + word * 4U, &value, 4U);
      }
    }
  }
}

void STM32USBOtgDevice::DeliverSetup(const uint8_t* packet)
{
  USB::SetupPacket setup;
  std::memcpy(&setup, packet, sizeof(setup));
  OnSetupPacket(true, &setup);
}

void STM32USBOtgDevice::HandleOutEndpoints()
{
  const uint32_t core_id = CoreId();
  uint32_t bits = (Device()->DAINT & Device()->DAINTMSK) >> 16U;
  while (bits != 0U)
  {
    const uint8_t num = static_cast<uint8_t>(__builtin_ctz(bits));
    bits &= bits - 1U;
    auto* regs = OutEp(num);
    const uint32_t events = regs->DOEPINT;
    const uint32_t handled =
        events & (USB_OTG_DOEPINT_XFRC | USB_OTG_DOEPINT_STUP | USB_OTG_DOEPINT_EPDISD |
                  USB_OTG_DOEPINT_OTEPDIS | USB_OTG_DOEPINT_OTEPSPR |
                  USB_OTG_DOEPINT_STPKTRX | USB_OTG_DOEPINT_NAK);
    regs->DOEPINT = handled;
    STM32OtgEndpoint* ep = num < MAX_ENDPOINTS ? out_[num] : nullptr;
    const bool setup = (events & USB_OTG_DOEPINT_STUP) != 0U;
    if ((events & USB_OTG_DOEPINT_XFRC) != 0U && ep != nullptr)
    {
      // EP0 OUT 也会把 SETUP 和状态阶段事件报告为 XFRC（core 的 C 类和 E 类）；
      // 只有数据传输才完成请求（与 HAL 一致）。
      // EP0 OUT also reports SETUP and status-phase events as XFRC (classes C and E of
      // the core); only a data transfer completes the request (as HAL does).
      bool data = !setup;
      if (num == 0U && hw_.dma)
        data =
            data && (events & USB_OTG_DOEPINT_OTEPSPR) == 0U &&
            !(core_id > USB_OTG_CORE_ID_300A && (events & USB_OTG_DOEPINT_STPKTRX) != 0U);
      else if (num == 0U)
        data = data && !(core_id == USB_OTG_CORE_ID_310A &&
                         (events & USB_OTG_DOEPINT_STPKTRX) != 0U);
      if (data) ep->OnTransferComplete();
    }
    if (setup && num == 0U)
    {
      if (hw_.dma)
      {
        // 最后一个 SETUP 位于 core 前移后的 DMA 地址的正下方。
        // The last SETUP sits right below the DMA address the core advanced to.
        STM32_InvalidateDCacheByAddr(hw_.ep0_dma_buffer.addr_, EP0_DMA_SIZE);
        const uintptr_t start = reinterpret_cast<uintptr_t>(hw_.ep0_dma_buffer.addr_);
        uintptr_t packet = static_cast<uintptr_t>(regs->DOEPDMA) - SETUP_SIZE;
        if (packet < start || packet + SETUP_SIZE > start + EP0_DMA_SIZE) packet = start;
        DeliverSetup(reinterpret_cast<const uint8_t*>(packet));
        if (core_id > USB_OTG_CORE_ID_300A) ArmSetup();
      }
      else
      {
        DeliverSetup(setup_);
      }
    }
    if ((events & USB_OTG_DOEPINT_EPDISD) != 0U && ep != nullptr) ep->OnDisabled();
  }
}

void STM32USBOtgDevice::HandleInEndpoints()
{
  const uint32_t empty_mask = Device()->DIEPEMPMSK;
  const uint32_t mask = Device()->DIEPMSK;
  uint32_t bits = Device()->DAINT & Device()->DAINTMSK & 0xFFFFU;
  while (bits != 0U)
  {
    const uint8_t num = static_cast<uint8_t>(__builtin_ctz(bits));
    bits &= bits - 1U;
    auto* regs = InEp(num);
    const uint32_t raw = regs->DIEPINT;
    const uint32_t events =
        raw & (mask | (((empty_mask >> num) & 1U) != 0U ? USB_OTG_DIEPINT_TXFE : 0U));
    STM32OtgEndpoint* ep = num < MAX_ENDPOINTS ? in_[num] : nullptr;
    if ((events & USB_OTG_DIEPINT_XFRC) != 0U)
    {
      Device()->DIEPEMPMSK &= ~(1UL << num);
      regs->DIEPINT = USB_OTG_DIEPINT_XFRC;
      if (ep != nullptr) ep->OnTransferComplete();
    }
    if ((events & USB_OTG_DIEPINT_TOC) != 0U) regs->DIEPINT = USB_OTG_DIEPINT_TOC;
    if ((raw & USB_OTG_DIEPINT_ITTXFE) != 0U) regs->DIEPINT = USB_OTG_DIEPINT_ITTXFE;
    if ((events & USB_OTG_DIEPINT_INEPNE) != 0U)
    {
      regs->DIEPINT = USB_OTG_DIEPINT_INEPNE;
      if (ep != nullptr) ep->OnNakEffective();
    }
    if ((events & USB_OTG_DIEPINT_EPDISD) != 0U)
    {
      regs->DIEPINT = USB_OTG_DIEPINT_EPDISD;
      if (ep != nullptr) ep->OnDisabled();
    }
    if ((events & USB_OTG_DIEPINT_TXFE) != 0U)
    {
      if (ep != nullptr)
        ep->FillFifo();
      else
        Device()->DIEPEMPMSK &= ~(1UL << num);
    }
  }
}

void STM32USBOtgDevice::HandleGlobalOutNak()
{
  // 全局 NAK 开启期间该标志保持置位；通知一次即可。
  // The flag stays set while the global NAK is on; one notification is enough.
  core_->GINTMSK &= ~USB_OTG_GINTMSK_GONAKEFFM;
  for (uint8_t i = 1; i < endpoint_count_; ++i)
    if (out_[i] != nullptr) out_[i]->OnGlobalOutNak();
}

void STM32USBOtgDevice::HandleSof()
{
  core_->GINTSTS = USB_OTG_GINTSTS_SOF;
  // 停止流程的兜底：每帧检查一次寄存器状态。
  // Backstop for stops: the register state is checked once per frame.
  for (uint8_t i = 0; i < endpoint_count_; ++i)
  {
    if (in_[i] != nullptr && in_[i]->stop_stage_ != STM32OtgEndpoint::StopStage::NONE)
      in_[i]->PollStop();
    if (out_[i] != nullptr && out_[i]->stop_stage_ != STM32OtgEndpoint::StopStage::NONE)
      out_[i]->PollStop();
  }
  const uint32_t position =
      (Device()->DSTS & USB_OTG_DSTS_FNSOF) >> USB_OTG_DSTS_FNSOF_Pos;
  if (bus_speed_ == USB::Speed::HIGH)
    OnSof(true, static_cast<uint16_t>(position >> 3U),
          static_cast<uint8_t>(position & 7U));
  else
    OnSof(true, static_cast<uint16_t>(position & 0x7FFU));
}

void STM32USBOtgDevice::UpdateInNakMask()
{
  // NAK 生效期间 INEPNE 保持置位；仅当停止流程在等待它时才取消屏蔽。
  // INEPNE stays set while the NAK is in effect; it is unmasked only while a stop waits
  // for it.
  for (uint8_t i = 0; i < endpoint_count_; ++i)
  {
    if (in_[i] != nullptr && in_[i]->stop_stage_ == STM32OtgEndpoint::StopStage::IN_NAK)
    {
      Device()->DIEPMSK |= USB_OTG_DIEPMSK_INEPNEM;
      return;
    }
  }
  Device()->DIEPMSK &= ~USB_OTG_DIEPMSK_INEPNEM;
}

bool STM32USBOtgDevice::AcquireGlobalOutNak()
{
  if (global_nak_users_++ == 0U)
  {
    core_->GINTMSK |= USB_OTG_GINTMSK_GONAKEFFM;
    Device()->DCTL |= USB_OTG_DCTL_SGONAK;
  }
  return (core_->GINTSTS & USB_OTG_GINTSTS_BOUTNAKEFF) != 0U;
}

void STM32USBOtgDevice::ReleaseGlobalOutNak()
{
  if (global_nak_users_ == 0U || --global_nak_users_ != 0U) return;
  core_->GINTMSK &= ~USB_OTG_GINTMSK_GONAKEFFM;
  Device()->DCTL |= USB_OTG_DCTL_CGONAK;
}

void STM32USBOtgDevice::RequestTxFlush(uint8_t fifo)
{
  flush_pending_ |= fifo == FLUSH_ALL ? (1UL << 16U) : (1UL << fifo);
  core_->GINTMSK |= USB_OTG_GINTMSK_SOFM;
  PollTxFlush();
}

void STM32USBOtgDevice::PollTxFlush()
{
  // 刷新需要几个时钟，从不等待其完成；仍在进行的刷新在下次中断或总线 tick 时再次检查。
  // A flush takes a few clocks; it is never waited for. A running one is checked again
  // at the next interrupt or bus tick.
  if (flush_active_ == NO_FLUSH && flush_pending_ == 0U) return;
  for (unsigned round = 0; round < 2U; ++round)
  {
    if (flush_active_ != NO_FLUSH)
    {
      if ((core_->GRSTCTL & USB_OTG_GRSTCTL_TXFFLSH) != 0U) return;
      const uint8_t done = flush_active_;
      flush_active_ = NO_FLUSH;
      for (uint8_t i = 0; i < endpoint_count_; ++i)
        if (in_[i] != nullptr && (done == FLUSH_ALL || done == i)) in_[i]->OnFlushDone();
    }
    if (flush_pending_ == 0U)
    {
      // 使总线 tick 保持开启的刷新已完成。
      // The flush that kept the bus tick on has finished.
      UpdateTick();
      return;
    }
    if ((core_->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL) == 0U) return;
    uint8_t fifo;
    if ((flush_pending_ & (1UL << 16U)) != 0U)
    {
      fifo = FLUSH_ALL;
      flush_pending_ = 0U;
    }
    else
    {
      fifo = static_cast<uint8_t>(__builtin_ctz(flush_pending_));
      flush_pending_ &= ~(1UL << fifo);
    }
    core_->GRSTCTL = USB_OTG_GRSTCTL_TXFFLSH | (static_cast<uint32_t>(fifo) << 6U);
    flush_active_ = fifo;
  }
}

void STM32USBOtgDevice::UpdateTick()
{
  bool needed = WantsBusTime() || flush_pending_ != 0U || flush_active_ != NO_FLUSH;
  for (uint8_t i = 0; i < endpoint_count_ && !needed; ++i)
  {
    needed =
        (in_[i] != nullptr && in_[i]->stop_stage_ != STM32OtgEndpoint::StopStage::NONE) ||
        (out_[i] != nullptr && out_[i]->stop_stage_ != STM32OtgEndpoint::StopStage::NONE);
  }
  if (needed)
    core_->GINTMSK |= USB_OTG_GINTMSK_SOFM;
  else
    core_->GINTMSK &= ~USB_OTG_GINTMSK_SOFM;
}

void STM32USBOtgDevice::ForceStopAll(bool in_isr)
{
  {
    USB::EndpointPool::HardwareScope hardware(this);
    global_nak_users_ = 0U;
    core_->GINTMSK &= ~USB_OTG_GINTMSK_GONAKEFFM;
    Device()->DCTL |= USB_OTG_DCTL_CGONAK;
    flush_pending_ = 0U;
    flush_active_ = NO_FLUSH;
  }
  for (uint8_t i = 0; i < endpoint_count_; ++i)
  {
    if (in_[i] != nullptr && in_[i]->stop_stage_ != STM32OtgEndpoint::StopStage::NONE)
      in_[i]->ConfirmStop(in_isr);
    if (out_[i] != nullptr && out_[i]->stop_stage_ != STM32OtgEndpoint::StopStage::NONE)
      out_[i]->ConfirmStop(in_isr);
  }
}

// ---------------------------------------------------------------------------
// Endpoint
// ---------------------------------------------------------------------------

STM32OtgEndpoint::STM32OtgEndpoint(STM32USBOtgDevice& device, EPNumber number,
                                   Direction direction, size_t fifo_size, RawData buffer)
    : Endpoint(number, direction, buffer), device_(device), fifo_size_(fifo_size)
{
  const uint8_t num = EPNumberToInt8(number);
  REQUIRE(num < device.endpoint_count_);
  if (direction == Direction::IN)
    device.in_[num] = this;
  else
    device.out_[num] = this;
}

size_t STM32OtgEndpoint::MaxPacketSizeLimit(Type type) const
{
  // 一个包必须整个放进它的 FIFO：IN 是该端点自己的 TX FIFO，OUT 是 RX FIFO 扣除固定
  // 开销之后的部分。
  // A packet has to fit into its FIFO as a whole: the endpoint's own TX FIFO for IN, the
  // RX FIFO less its fixed overhead for OUT.
  const size_t room = IsIn() ? fifo_size_ : device_.rx_packet_limit_;
  return type == Type::CONTROL ? LibXR::min<size_t>(64U, room) : room;
}

ErrorCode STM32OtgEndpoint::ConfigureHardware(const Config& cfg)
{
  ResetTransfer();
  stop_stage_ = StopStage::NONE;
  const uint8_t num = Num();
  const uint32_t packet = cfg.max_packet_size;
  auto* dev = device_.Device();
  if (IsIn())
  {
    auto* regs = device_.InEp(num);
    dev->DAINTMSK |= 1UL << num;
    if (num == 0U)
    {
      regs->DIEPCTL = (regs->DIEPCTL & ~USB_OTG_DIEPCTL_MPSIZ) | Ep0SizeCode(packet);
      return ErrorCode::OK;
    }
    regs->DIEPCTL = (regs->DIEPCTL & ~(USB_OTG_DIEPCTL_MPSIZ | USB_OTG_DIEPCTL_EPTYP |
                                       USB_OTG_DIEPCTL_TXFNUM)) |
                    (packet & USB_OTG_DIEPCTL_MPSIZ) | EndpointType(cfg.type) |
                    (static_cast<uint32_t>(num) << USB_OTG_DIEPCTL_TXFNUM_Pos) |
                    USB_OTG_DIEPCTL_SD0PID_SEVNFRM | USB_OTG_DIEPCTL_USBAEP;
    return ErrorCode::OK;
  }
  dev->DAINTMSK |= (1UL << num) << 16U;
  // EP0 OUT 跟随 EP0 IN 的包大小。
  // EP0 OUT follows the EP0 IN packet size.
  if (num == 0U) return ErrorCode::OK;
  auto* regs = device_.OutEp(num);
  regs->DOEPCTL = (regs->DOEPCTL & ~(USB_OTG_DOEPCTL_MPSIZ | USB_OTG_DOEPCTL_EPTYP)) |
                  (packet & USB_OTG_DOEPCTL_MPSIZ) | EndpointType(cfg.type) |
                  USB_OTG_DOEPCTL_SD0PID_SEVNFRM | USB_OTG_DOEPCTL_USBAEP;
  return ErrorCode::OK;
}

void STM32OtgEndpoint::CloseHardware(bool in_isr)
{
  UNUSED(in_isr);
  ResetTransfer();
  stop_stage_ = StopStage::NONE;
  const uint8_t num = Num();
  if (num == 0U) return;  // EP0 保持激活 / EP0 stays active.
  auto* dev = device_.Device();
  if (IsIn())
  {
    dev->DIEPEMPMSK &= ~(1UL << num);
    dev->DAINTMSK &= ~(1UL << num);
    device_.InEp(num)->DIEPCTL &=
        ~(USB_OTG_DIEPCTL_USBAEP | USB_OTG_DIEPCTL_MPSIZ | USB_OTG_DIEPCTL_TXFNUM |
          USB_OTG_DIEPCTL_SD0PID_SEVNFRM | USB_OTG_DIEPCTL_EPTYP);
  }
  else
  {
    dev->DAINTMSK &= ~((1UL << num) << 16U);
    device_.OutEp(num)->DOEPCTL &=
        ~(USB_OTG_DOEPCTL_USBAEP | USB_OTG_DOEPCTL_MPSIZ |
          USB_OTG_DOEPCTL_SD0PID_SEVNFRM | USB_OTG_DOEPCTL_EPTYP);
  }
}

void STM32OtgEndpoint::ResetTransfer()
{
  hw_tag_ = 0U;
  hw_data_ = nullptr;
  hw_length_ = 0U;
  hw_programmed_ = 0U;
  hw_capacity_ = 0U;
  hw_done_ = 0U;
}

ErrorCode STM32OtgEndpoint::StartHardware(const HwRequest& request, bool in_isr)
{
  UNUSED(in_isr);
  const uint8_t num = Num();
  const uint32_t packet = Packet();
  const bool iso = GetConfig().type == Type::ISOCHRONOUS;
  if (num != 0U)
  {
    // 包数和长度要放得进 DxEPTSIZ 的 PKTCNT 和 XFRSIZ 字段（两个方向的字段宽度相同）。
    // The packet count and the length have to fit the PKTCNT and XFRSIZ fields of
    // DxEPTSIZ (the fields are equally wide in both directions).
    const size_t packets = (request.length + packet - 1U) / packet;
    if (packets > (USB_OTG_DIEPTSIZ_PKTCNT >> PKTCNT_POS) ||
        request.length > USB_OTG_DIEPTSIZ_XFRSIZ - 3U)
      return ErrorCode::SIZE_ERR;
  }
  hw_tag_ = request.tag;
  hw_data_ = static_cast<uint8_t*>(request.slot.addr_);
  hw_length_ = request.length;
  hw_capacity_ = request.slot.size_;
  hw_done_ = 0U;
  stopped_actual_ = 0U;
  stopped_sent_all_ = false;
  const bool odd_frame =
      (((device_.Device()->DSTS & USB_OTG_DSTS_FNSOF) >> USB_OTG_DSTS_FNSOF_Pos) & 1U) !=
      0U;

  if (IsIn())
  {
    auto* regs = device_.InEp(num);
    const uint32_t length = static_cast<uint32_t>(request.length);
    const uint32_t packets = length == 0U ? 1U : (length + packet - 1U) / packet;
    uint32_t size = (packets << PKTCNT_POS) | length;
    if (iso) size |= 1UL << MULCNT_POS;
    regs->DIEPTSIZ = size;
    if (Dma())
    {
      if (length != 0U) STM32_CleanDCacheByAddr(hw_data_, length);
      regs->DIEPDMA = hw_data_ != nullptr ? Address(hw_data_) : Address(device_.setup_);
    }
    if (iso)
      regs->DIEPCTL |=
          odd_frame ? USB_OTG_DIEPCTL_SD0PID_SEVNFRM : USB_OTG_DIEPCTL_SODDFRM;
    regs->DIEPCTL |= USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA;
    if (!Dma() && length != 0U)
    {
      // 立即写入当前能放下的数据，而不是等待下一次 TXFE 中断。
      // Write what fits now instead of waiting for the next TXFE interrupt.
      FillFifo();
      if (hw_done_ < hw_length_) device_.Device()->DIEPEMPMSK |= 1UL << num;
    }
    return ErrorCode::OK;
  }

  auto* regs = device_.OutEp(num);
  if (num == 0U)
  {
    // 一次一个包；EP0 持续接收 SETUP 包。
    // One packet at a time; EP0 keeps accepting SETUP packets.
    hw_capacity_ = LibXR::min(hw_capacity_, static_cast<size_t>(packet));
    hw_programmed_ = packet;
    regs->DOEPTSIZ = USB_OTG_DOEPTSIZ_STUPCNT | (1UL << PKTCNT_POS) | packet;
    if (Dma()) regs->DOEPDMA = Address(device_.hw_.ep0_dma_buffer.addr_);
  }
  else
  {
    // 请求长度是包大小的整数倍（Endpoint::Receive）；OUT 大小保持字对齐（RM0090
    // "Generic non-isochronous OUT data transfers"）。
    // The request length is a packet multiple (Endpoint::Receive); OUT sizes are kept
    // word aligned (RM0090 "Generic non-isochronous OUT data transfers").
    const uint32_t length = static_cast<uint32_t>(request.length);
    const uint32_t packets = length == 0U ? 1U : (length + packet - 1U) / packet;
    const uint32_t size = ((length == 0U ? packet : length) + 3U) & ~3U;
    hw_programmed_ = size;
    regs->DOEPTSIZ = (packets << PKTCNT_POS) | size;
    if (Dma())
    {
      // 先清理（clean）这段缓存：之后被换出的脏行不得覆盖已接收的字节。
      // Clean first: a dirty line evicted later must not overwrite received bytes.
      STM32_CleanDCacheByAddr(hw_data_, request.slot.size_);
      regs->DOEPDMA = Address(hw_data_);
    }
    if (iso)
      regs->DOEPCTL |=
          odd_frame ? USB_OTG_DOEPCTL_SD0PID_SEVNFRM : USB_OTG_DOEPCTL_SODDFRM;
  }
  regs->DOEPCTL |= USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA;
  return ErrorCode::OK;
}

void STM32OtgEndpoint::FillFifo()
{
  const uint8_t num = Num();
  if (hw_tag_ == 0U || hw_data_ == nullptr)
  {
    device_.Device()->DIEPEMPMSK &= ~(1UL << num);
    return;
  }
  auto* regs = device_.InEp(num);
  volatile uint32_t& fifo = device_.Fifo(num);
  // 受空闲 FIFO 空间限制；仅传输完整的包。
  // Bounded by the free FIFO space; whole packets only.
  while (hw_done_ < hw_length_)
  {
    const size_t length =
        LibXR::min(hw_length_ - hw_done_, static_cast<size_t>(Packet()));
    const uint32_t words = static_cast<uint32_t>((length + 3U) / 4U);
    if ((regs->DTXFSTS & USB_OTG_DTXFSTS_INEPTFSAV) < words) break;
    const uint8_t* source = hw_data_ + hw_done_;
    for (uint32_t i = 0; i < words; ++i)
    {
      uint32_t value = 0U;
      std::memcpy(&value, source + i * 4U, LibXR::min<size_t>(4U, length - i * 4U));
      fifo = value;
    }
    hw_done_ += length;
  }
  if (hw_done_ >= hw_length_) device_.Device()->DIEPEMPMSK &= ~(1UL << num);
}

void STM32OtgEndpoint::ReadFifo(size_t bytes)
{
  volatile uint32_t& fifo = device_.Fifo(0);
  const size_t room = hw_tag_ != 0U && hw_data_ != nullptr && hw_capacity_ > hw_done_
                          ? hw_capacity_ - hw_done_
                          : 0U;
  const size_t take = LibXR::min(bytes, room);
  uint8_t* target = take != 0U ? hw_data_ + hw_done_ : nullptr;
  size_t offset = 0;
  for (; offset + 4U <= take; offset += 4U)
  {
    const uint32_t value = fifo;
    std::memcpy(target + offset, &value, 4U);
  }
  for (; offset < bytes; offset += 4U)
  {
    const uint32_t value = fifo;
    if (offset < take) std::memcpy(target + offset, &value, take - offset);
  }
  hw_done_ += take;
}

size_t STM32OtgEndpoint::DmaOutBytes() const
{
  const size_t remaining = device_.OutEp(Num())->DOEPTSIZ & USB_OTG_DOEPTSIZ_XFRSIZ;
  const size_t actual = hw_programmed_ > remaining ? hw_programmed_ - remaining : 0U;
  return LibXR::min(actual, hw_capacity_);
}

void STM32OtgEndpoint::OnTransferComplete()
{
  const uint32_t tag = hw_tag_;
  if (tag == 0U) return;  // 已停止，或为重复调用 / Stopped, or a duplicate.
  size_t actual = hw_length_;
  if (!IsIn())
  {
    if (Dma())
    {
      actual = DmaOutBytes();
      if (Num() == 0U)
      {
        STM32_InvalidateDCacheByAddr(device_.hw_.ep0_dma_buffer.addr_, EP0_DMA_SIZE);
        if (actual != 0U) std::memcpy(hw_data_, device_.hw_.ep0_dma_buffer.addr_, actual);
      }
      else if (actual != 0U)
      {
        STM32_InvalidateDCacheByAddr(hw_data_, actual);
      }
    }
    else
    {
      actual = hw_done_;
    }
  }
  ResetTransfer();
  OnHardwareComplete(true, tag, actual, ErrorCode::OK);
}

USB::Endpoint::StopResult STM32OtgEndpoint::StopHardware(bool in_isr)
{
  UNUSED(in_isr);
  const uint8_t num = Num();
  if (IsIn())
  {
    auto* regs = device_.InEp(num);
    device_.Device()->DIEPEMPMSK &= ~(1UL << num);
    if ((regs->DIEPCTL & USB_OTG_DIEPCTL_EPENA) == 0U)
    {
      // 端点已不在使能状态：请求已经在硬件里发完。它的 XFRC 可能还没处理，也可能在这次
      // 停止返回之后、ReapProgress 之前被中断清掉，所以在这里记下“已全部发出”。
      // The endpoint is no longer enabled: the request has been sent in the hardware. Its
      // XFRC may be unhandled, or be cleared by the interrupt after this stop returns and
      // before ReapProgress, so "everything was sent" is recorded here.
      stopped_sent_all_ = hw_tag_ != 0U;
      ResetTransfer();
      return StopResult::DONE;
    }
    // RM0090 "IN endpoint disable"：SNAK、INEPNE、EPDIS、EPDISD，然后刷新 FIFO。
    // RM0090 "IN endpoint disable": SNAK, INEPNE, EPDIS, EPDISD, then the FIFO flush.
    stop_stage_ = StopStage::IN_NAK;
    device_.UpdateInNakMask();
    regs->DIEPCTL |= USB_OTG_DIEPCTL_SNAK;
    device_.UpdateTick();
    return StopResult::PENDING;
  }
  auto* regs = device_.OutEp(num);
  if (num == 0U)
  {
    // EP0 OUT 不能被禁用（RM0090 DOEPCTL0.EPDIS）。它从不写入槽：使用 DMA
    // 时接收到设备的 EP0 缓冲区，不使用 DMA 时数据被丢弃。
    // EP0 OUT cannot be disabled (RM0090 DOEPCTL0.EPDIS). It never writes a slot: with
    // DMA it receives into the device's EP0 buffer, without DMA the data is dropped.
    regs->DOEPCTL |= USB_OTG_DOEPCTL_SNAK;
    stopped_actual_ = Dma() ? 0U : hw_done_;
    ResetTransfer();
    return StopResult::DONE;
  }
  if ((regs->DOEPCTL & USB_OTG_DOEPCTL_EPENA) == 0U)
  {
    // 端点已经不在使能状态：请求可能已在硬件里完成，只是完成中断还没被处理，这时数据
    // 已经在槽里，按 DOEPTSIZ 如实报告。
    // The endpoint is no longer enabled: the request may already have completed in the
    // hardware with its completion interrupt not handled yet, in which case the data is
    // in the slot and is reported from DOEPTSIZ.
    if (Dma() && hw_data_ != nullptr)
    {
      stopped_actual_ = DmaOutBytes();
      if (stopped_actual_ != 0U) STM32_InvalidateDCacheByAddr(hw_data_, stopped_actual_);
    }
    else
    {
      stopped_actual_ = Dma() ? 0U : hw_done_;
    }
    ResetTransfer();
    return StopResult::DONE;
  }
  // RM0090 "Disabling an OUT endpoint"：全局 OUT NAK、EPDIS、EPDISD。
  // RM0090 "Disabling an OUT endpoint": global OUT NAK, EPDIS, EPDISD.
  stop_stage_ = StopStage::OUT_GLOBAL_NAK;
  if (device_.AcquireGlobalOutNak())
  {
    regs->DOEPCTL |= USB_OTG_DOEPCTL_SNAK | USB_OTG_DOEPCTL_EPDIS;
    stop_stage_ = StopStage::OUT_DISABLE;
  }
  device_.UpdateTick();
  return StopResult::PENDING;
}

void STM32OtgEndpoint::PollStop()
{
  const uint8_t num = Num();
  switch (stop_stage_)
  {
    case StopStage::IN_NAK:
    {
      auto* regs = device_.InEp(num);
      if ((regs->DIEPCTL & USB_OTG_DIEPCTL_EPENA) == 0U)
      {
        // 在 NAK 之前已完成；FIFO 为空。
        // Completed before the NAK; the FIFO is empty.
        ConfirmStop(true);
      }
      else if ((regs->DIEPCTL & USB_OTG_DIEPCTL_NAKSTS) != 0U)
      {
        regs->DIEPCTL |= USB_OTG_DIEPCTL_SNAK | USB_OTG_DIEPCTL_EPDIS;
        stop_stage_ = StopStage::IN_DISABLE;
        device_.UpdateInNakMask();
      }
      break;
    }
    case StopStage::IN_DISABLE:
      if ((device_.InEp(num)->DIEPCTL & USB_OTG_DIEPCTL_EPENA) == 0U)
      {
        stop_stage_ = StopStage::IN_FLUSH;
        device_.RequestTxFlush(num);
      }
      break;
    case StopStage::OUT_GLOBAL_NAK:
    {
      auto* regs = device_.OutEp(num);
      if ((regs->DOEPCTL & USB_OTG_DOEPCTL_EPENA) == 0U)
      {
        device_.ReleaseGlobalOutNak();
        ConfirmStop(true);
      }
      else if ((device_.core_->GINTSTS & USB_OTG_GINTSTS_BOUTNAKEFF) != 0U)
      {
        regs->DOEPCTL |= USB_OTG_DOEPCTL_SNAK | USB_OTG_DOEPCTL_EPDIS;
        stop_stage_ = StopStage::OUT_DISABLE;
      }
      break;
    }
    case StopStage::OUT_DISABLE:
      if ((device_.OutEp(num)->DOEPCTL & USB_OTG_DOEPCTL_EPENA) == 0U)
      {
        device_.ReleaseGlobalOutNak();
        ConfirmStop(true);
      }
      break;
    case StopStage::IN_FLUSH:
    case StopStage::NONE:
      break;
  }
}

void STM32OtgEndpoint::OnNakEffective() { PollStop(); }

void STM32OtgEndpoint::OnDisabled() { PollStop(); }

void STM32OtgEndpoint::OnGlobalOutNak()
{
  if (stop_stage_ == StopStage::OUT_GLOBAL_NAK) PollStop();
}

void STM32OtgEndpoint::OnFlushDone()
{
  if (stop_stage_ == StopStage::IN_FLUSH) ConfirmStop(true);
}

void STM32OtgEndpoint::ConfirmStop(bool in_isr)
{
  if (!IsIn())
  {
    if (Dma() && Num() != 0U && hw_data_ != nullptr)
    {
      stopped_actual_ = DmaOutBytes();
      if (stopped_actual_ != 0U) STM32_InvalidateDCacheByAddr(hw_data_, stopped_actual_);
    }
    else if (!Dma())
    {
      stopped_actual_ = hw_done_;
    }
  }
  stop_stage_ = StopStage::NONE;
  ResetTransfer();
  device_.UpdateInNakMask();
  OnHardwareStopped(in_isr);
}

size_t STM32OtgEndpoint::ReapProgress(const HwRequest& request, bool& retired)
{
  retired = false;
  if (!IsIn()) return stopped_actual_;
  auto* regs = device_.InEp(Num());
  // 停止时已发完，或者有尚未处理的传输完成：主机已取走全部数据。
  // Sent completely when the stop came, or a transfer complete is still unhandled: the
  // host took everything.
  retired = stopped_sent_all_ || (regs->DIEPINT & USB_OTG_DIEPINT_XFRC) != 0U;
  if (retired) return request.length;
  const size_t packet = Packet();
  const size_t remaining = (regs->DIEPTSIZ & USB_OTG_DIEPTSIZ_PKTCNT) >> PKTCNT_POS;
  const size_t total =
      request.length == 0U ? 1U : (request.length + packet - 1U) / packet;
  // 包离开 FIFO 时 PKTCNT 递减；最后一个包可能尚未收到主机的 ACK。
  // PKTCNT drops when a packet leaves the FIFO; the last one may lack the host's ACK.
  size_t sent = total > remaining ? total - remaining : 0U;
  if (sent != 0U) --sent;
  return LibXR::min(sent * packet, request.length);
}

ErrorCode STM32OtgEndpoint::StallHardware(bool in_isr)
{
  UNUSED(in_isr);
  const uint8_t num = Num();
  if (IsIn())
  {
    auto* regs = device_.InEp(num);
    if ((regs->DIEPCTL & USB_OTG_DIEPCTL_EPENA) == 0U && num != 0U)
      regs->DIEPCTL &= ~USB_OTG_DIEPCTL_EPDIS;
    regs->DIEPCTL |= USB_OTG_DIEPCTL_STALL;
  }
  else
  {
    auto* regs = device_.OutEp(num);
    if ((regs->DOEPCTL & USB_OTG_DOEPCTL_EPENA) == 0U && num != 0U)
      regs->DOEPCTL &= ~USB_OTG_DOEPCTL_EPDIS;
    regs->DOEPCTL |= USB_OTG_DOEPCTL_STALL;
    // 使用 DMA 时只有 EP0 OUT 处于使能状态 core 才保存 SETUP；被 STALL 的请求没有状态阶段
    // 来重新使能它（HAL_PCD_EP_SetStall 也这样做）。
    // With DMA a SETUP is stored only while EP0 OUT is enabled, and a stalled request has
    // no status stage to enable it again (as HAL_PCD_EP_SetStall does).
    if (num == 0U && Dma()) device_.ArmSetup();
  }
  return ErrorCode::OK;
}

ErrorCode STM32OtgEndpoint::ClearStallHardware(bool in_isr)
{
  UNUSED(in_isr);
  const uint8_t num = Num();
  // 下一个 SETUP 会清除 EP0 STALL / The next SETUP clears an EP0 STALL.
  if (num == 0U) return ErrorCode::OK;
  const bool data0 = GetConfig().type == Type::BULK || GetConfig().type == Type::INTR;
  if (IsIn())
  {
    auto* regs = device_.InEp(num);
    regs->DIEPCTL &= ~USB_OTG_DIEPCTL_STALL;
    if (data0) regs->DIEPCTL |= USB_OTG_DIEPCTL_SD0PID_SEVNFRM;
  }
  else
  {
    auto* regs = device_.OutEp(num);
    regs->DOEPCTL &= ~USB_OTG_DOEPCTL_STALL;
    if (data0) regs->DOEPCTL |= USB_OTG_DOEPCTL_SD0PID_SEVNFRM;
  }
  return ErrorCode::OK;
}

// ---------------------------------------------------------------------------
// 实例与中断向量 / Instances and vectors
// ---------------------------------------------------------------------------

#if defined(USB_OTG_FS)
STM32USBDeviceOtgFS::STM32USBDeviceOtgFS(
    const HardwareConfig& hw, size_t rx_fifo_size,
    const std::initializer_list<RawData> RX_EP_CFGS,
    const std::initializer_list<EPInConfig> TX_EP_CFGS,
    USB::DeviceDescriptor::PacketSize0 packet_size, uint16_t vid, uint16_t pid,
    uint16_t bcd,
    const std::initializer_list<const USB::DescriptorStrings::LanguagePack*> LANG_LIST,
    const std::initializer_list<const std::initializer_list<USB::ConfigDescriptorItem*>>
        CONFIGS,
    ConstRawData uid)
    : STM32USBOtgDevice(OTG_FS_INDEX, USB_OTG_FS, OTG_FS_ENDPOINTS, OTG_FS_FIFO, hw,
                        rx_fifo_size, RX_EP_CFGS, TX_EP_CFGS, packet_size, vid, pid, bcd,
                        LANG_LIST, CONFIGS, uid)
{
}

#endif

#if defined(USB_OTG_HS)
STM32USBDeviceOtgHS::STM32USBDeviceOtgHS(
    const HardwareConfig& hw, size_t rx_fifo_size,
    const std::initializer_list<RawData> RX_EP_CFGS,
    const std::initializer_list<EPInConfig> TX_EP_CFGS,
    USB::DeviceDescriptor::PacketSize0 packet_size, uint16_t vid, uint16_t pid,
    uint16_t bcd,
    const std::initializer_list<const USB::DescriptorStrings::LanguagePack*> LANG_LIST,
    const std::initializer_list<const std::initializer_list<USB::ConfigDescriptorItem*>>
        CONFIGS,
    ConstRawData uid)
    : STM32USBOtgDevice(OTG_HS_INDEX, USB_OTG_HS, OTG_HS_ENDPOINTS, OTG_HS_FIFO, hw,
                        rx_fifo_size, RX_EP_CFGS, TX_EP_CFGS, packet_size, vid, pid, bcd,
                        LANG_LIST, CONFIGS, uid)
{
}

#endif

// 由包装后的 HAL_PCD_IRQHandler() 进入（stm32_usb_wrap.cpp）。句柄仅作提示：当 main()
// 从未运行 MX_USB_OTG_*_PCD_Init() 时其 Instance 为空，因此改由当前活动异常确定控制器。
// Entry from the wrapped HAL_PCD_IRQHandler() (stm32_usb_wrap.cpp). The handle is
// only a hint: when main() never ran MX_USB_OTG_*_PCD_Init() its Instance is null, so the
// active exception names the controller instead.
extern "C" void libxr_stm32_otg_dispatch(PCD_HandleTypeDef* hpcd)
{
  USB_OTG_GlobalTypeDef* core = hpcd != nullptr ? hpcd->Instance : nullptr;
  if (core == nullptr)
  {
    const int32_t irq = static_cast<int32_t>(__get_IPSR()) - 16;
#if defined(USB_OTG_FS)
    if (irq == OTG_FS_IRQn) core = USB_OTG_FS;
#endif
#if defined(USB_OTG_HS)
    if (irq == OTG_HS_IRQn) core = USB_OTG_HS;
#endif
    if (core == nullptr) return;
  }
  STM32USBOtgDevice::Dispatch(core);
}

#endif
