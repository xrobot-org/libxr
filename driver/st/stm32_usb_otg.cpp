#include "stm32_usb_otg.hpp"

#if defined(HAL_PCD_MODULE_ENABLED) && (defined(USB_OTG_FS) || defined(USB_OTG_HS))

#include <cstring>

#include "stm32_dcache.hpp"
#include "stm32_usb_wrap.hpp"
#include "timebase.hpp"

using namespace LibXR;
using LibXR::USB::Endpoint;

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
// MULCNT field of DIEPTSIZ (bits 30:29); the device headers name it differently.
constexpr uint32_t MULCNT_POS = 29U;
// EP0 OUT 可接收的连续 SETUP 包数。
// Back-to-back SETUP packets EP0 OUT accepts.
constexpr uint32_t SETUP_COUNT = 3U;
constexpr size_t SETUP_SIZE = 8U;
constexpr size_t EP0_DMA_SIZE = 64U;
// 一次请求的上限：PKTCNT 10 位；XFRSIZ 19 位，OUT 要按字对齐，取 524287 向下的 4 的倍数。
// Limits of one request: PKTCNT has 10 bits; XFRSIZ has 19 bits and OUT sizes are word
// aligned, so the multiple of 4 below 524287.
constexpr size_t MAX_PACKETS = 1023U;
constexpr size_t MAX_BYTES = 524284U;
// INEPNE 之外的全部 DIEPINT 事件位；INEPNE 由 IN 停止自己管理。
// Every DIEPINT event bit except INEPNE; the IN stop manages INEPNE itself.
constexpr uint32_t DIEPINT_ALL_BUT_NAK_EFFECTIVE = 0xFB7FU & ~USB_OTG_DIEPINT_INEPNE;

uint32_t Address(const void* pointer)
{
  return static_cast<uint32_t>(reinterpret_cast<uintptr_t>(pointer));
}

/// DIEPCTL0 的 MPSIZ 编码：64、32、16 和 8 字节 / MPSIZ code of DIEPCTL0
uint32_t Ep0SizeCode(uint32_t packet)
{
  return packet >= 64U ? 0U : packet >= 32U ? 1U : packet >= 16U ? 2U : 3U;
}

uint32_t EndpointType(Endpoint::Type type) { return static_cast<uint32_t>(type) << 18U; }

/// 当前帧号是否为奇数 / Whether the current frame number is odd
bool OddFrame(USB_OTG_DeviceTypeDef* device)
{
  return (((device->DSTS & USB_OTG_DSTS_FNSOF) >> USB_OTG_DSTS_FNSOF_Pos) & 1U) != 0U;
}

/**
 * @brief 读寄存器直到 done 成立，最多 HANDSHAKE_SPINS 次 / Read registers until done
 *        holds, HANDSHAKE_SPINS times at most
 * @note 只用于与总线无关的握手。
 *       Bus-independent handshakes only.
 */
template <typename Done>
bool Spin(Done done)
{
  for (uint32_t i = 0; i < STM32OtgEndpoint::HANDSHAKE_SPINS; ++i)
  {
    if (done())
    {
      return true;
    }
  }
  return false;
}
}  // namespace

// ---------------------------------------------------------------------------
// Controller
// ---------------------------------------------------------------------------

STM32OtgController::STM32OtgController(
    USB_OTG_GlobalTypeDef* core, IRQn_Type irqn, uint8_t endpoint_count,
    size_t total_fifo, const HardwareConfig& hw, size_t rx_fifo_size,
    const std::initializer_list<RawData>& rx_ep_cfgs,
    const std::initializer_list<EPInConfig>& tx_ep_cfgs)
    : core_(core), irqn_(irqn), endpoint_count_(endpoint_count), hw_(hw)
{
  REQUIRE(endpoint_count <= 16U);
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
#if defined(USBD_HS_SPEED)
  cfg.speed = hw.high_speed ? USBD_HS_SPEED : USBD_FS_SPEED;
#else
  REQUIRE(!hw.high_speed);
  cfg.speed = USBD_FS_SPEED;
#endif
  cfg.dma_enable = hw.dma ? 1U : 0U;
  cfg.phy_itface = USB_OTG_EMBEDDED_PHY;
#if defined(USB_OTG_ULPI_PHY)
  if (hw.phy == Phy::ULPI)
  {
    cfg.phy_itface = USB_OTG_ULPI_PHY;
  }
#else
  REQUIRE(hw.phy != Phy::ULPI);
#endif
#if defined(USB_OTG_HS_EMBEDDED_PHY)
  if (hw.phy == Phy::EMBEDDED_HS)
  {
    cfg.phy_itface = USB_OTG_HS_EMBEDDED_PHY;
  }
#else
  REQUIRE(hw.phy != Phy::EMBEDDED_HS);
#endif
  cfg.vbus_sensing_enable = hw.vbus_sensing ? 1U : 0U;
  REQUIRE(USB_CoreInit(core_, cfg) == HAL_OK);
  // GSNPSID 紧跟在 CID 之后（偏移 0x40）；器件头文件没有这个成员，HAL 也这样读。
  // GSNPSID follows CID (offset 0x40); the device headers lack it, HAL reads it so too.
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
  auto* ep0_out = new STM32OtgEndpoint(*this, Endpoint::EPNumber::EP0,
                                       Endpoint::Direction::OUT, 0U, *rx);
  auto* ep0_in = new STM32OtgEndpoint(*this, Endpoint::EPNumber::EP0,
                                      Endpoint::Direction::IN, tx->fifo_size, tx->buffer);
  SetEndpoint0(ep0_in, ep0_out);
  auto number = Endpoint::EPNumber::EP1;
  for (++rx; rx != rx_ep_cfgs.end(); ++rx)
  {
    REQUIRE(Put(new STM32OtgEndpoint(*this, number, Endpoint::Direction::OUT, 0U, *rx)) ==
            ErrorCode::OK);
    number = Endpoint::NextEPNumber(number);
  }
  number = Endpoint::EPNumber::EP1;
  for (++tx; tx != tx_ep_cfgs.end(); ++tx)
  {
    REQUIRE(Put(new STM32OtgEndpoint(*this, number, Endpoint::Direction::IN,
                                     tx->fifo_size, tx->buffer)) == ErrorCode::OK);
    number = Endpoint::NextEPNumber(number);
  }
  ProgramFifos(rx_fifo_size);
}

void STM32OtgController::ProgramFifos(size_t rx_fifo_size)
{
  // FIFO RAM：先 RX，再 TX0 及各 IN 端点的 TX FIFO（IN 端点 n 使用 TX FIFO n）。
  // FIFO RAM: RX first, then TX0 and the TX FIFO of each IN endpoint (IN endpoint n
  // uses TX FIFO n).
  const uint32_t rx_words = static_cast<uint32_t>(rx_fifo_size / 4U);
  core_->GRXFSIZ = rx_words;
  uint32_t offset = rx_words;
  for (uint8_t fifo = 0; fifo < endpoint_count_ && in_[fifo] != nullptr; ++fifo)
  {
    const uint32_t words = static_cast<uint32_t>(in_[fifo]->fifo_size_ / 4U);
    if (fifo == 0U)
    {
      core_->DIEPTXF0_HNPTXFSIZ = (words << 16U) | offset;
    }
    else
    {
      core_->DIEPTXF[fifo - 1U] = (words << 16U) | offset;
    }
    offset += words;
  }
}

USB_OTG_DeviceTypeDef* STM32OtgController::Device() const
{
  return reinterpret_cast<USB_OTG_DeviceTypeDef*>(reinterpret_cast<uintptr_t>(core_) +
                                                  USB_OTG_DEVICE_BASE);
}

USB_OTG_INEndpointTypeDef* STM32OtgController::InEp(uint8_t num) const
{
  return reinterpret_cast<USB_OTG_INEndpointTypeDef*>(reinterpret_cast<uintptr_t>(core_) +
                                                      USB_OTG_IN_ENDPOINT_BASE +
                                                      num * USB_OTG_EP_REG_SIZE);
}

USB_OTG_OUTEndpointTypeDef* STM32OtgController::OutEp(uint8_t num) const
{
  return reinterpret_cast<USB_OTG_OUTEndpointTypeDef*>(
      reinterpret_cast<uintptr_t>(core_) + USB_OTG_OUT_ENDPOINT_BASE +
      num * USB_OTG_EP_REG_SIZE);
}

volatile uint32_t& STM32OtgController::Fifo(uint8_t num) const
{
  return *reinterpret_cast<volatile uint32_t*>(
      reinterpret_cast<uintptr_t>(core_) + USB_OTG_FIFO_BASE + num * USB_OTG_FIFO_SIZE);
}

void STM32OtgController::ArmSetup()
{
  auto* out0 = OutEp(0);
  // 较新的 core 在 EP0 OUT 仍使能时不得重新配置（与 HAL 一致）。
  // Newer cores must not be reprogrammed while EP0 OUT is still enabled (as HAL does).
  if (core_id_ > USB_OTG_CORE_ID_300A && (out0->DOEPCTL & USB_OTG_DOEPCTL_EPENA) != 0U)
  {
    return;
  }
  out0->DOEPTSIZ =
      USB_OTG_DOEPTSIZ_STUPCNT | (1U << PKTCNT_POS) | (SETUP_COUNT * SETUP_SIZE);
  if (hw_.dma)
  {
    out0->DOEPDMA = Address(hw_.ep0_dma_buffer.addr_);
    out0->DOEPCTL |= USB_OTG_DOEPCTL_EPENA | USB_OTG_DOEPCTL_USBAEP;
  }
}

void STM32OtgController::DrainRxFifo()
{
  // 有界：每次最多处理这么多条状态项；其余由电平中断或停止的下一次循环带出。
  // Bounded: at most this many status entries per call; the level interrupt or the next
  // turn of a stop's loop brings the rest.
  for (unsigned i = 0; i < 16U && (core_->GINTSTS & USB_OTG_GINTSTS_RXFLVL) != 0U; ++i)
  {
    const uint32_t entry = core_->GRXSTSP;
    const uint8_t num = static_cast<uint8_t>(entry & USB_OTG_GRXSTSP_EPNUM);
    const size_t bytes = (entry & USB_OTG_GRXSTSP_BCNT) >> USB_OTG_GRXSTSP_BCNT_Pos;
    const uint32_t kind = (entry & USB_OTG_GRXSTSP_PKTSTS) >> USB_OTG_GRXSTSP_PKTSTS_Pos;
    if (kind == STS_DATA_UPDT && out_[num] != nullptr)
    {
      out_[num]->ReadFifo(bytes);
    }
    else
    {
      // SETUP 拷进 setup_；其他数据（没有端点对象）丢弃。
      // A SETUP goes into rx_setup_; other data (no endpoint object) is dropped.
      if (kind == STS_SETUP_UPDT)
      {
        setup_pending_ = true;
      }
      for (size_t word = 0; word < (bytes + 3U) / 4U; ++word)
      {
        const uint32_t value = Fifo(0);
        if (kind == STS_SETUP_UPDT && word < SETUP_SIZE / 4U)
        {
          std::memcpy(rx_setup_ + word * 4U, &value, 4U);
        }
      }
    }
  }
}

void STM32OtgController::RequestGlobalOutNak()
{
  // 没有 OUT 停止在等而 GONAKEFF 仍置位：是上一次全局 OUT NAK 留下的（撤销还没生效，或
  // FIFO 模式下撤销之后才弹出那一次的状态项）。
  // GONAKEFF still set with no OUT stop waiting is left from the previous global OUT NAK
  // (its withdrawal has not taken effect yet, or in FIFO mode its status entry was popped
  // after the withdrawal).
  auto* core = core_;
  if ((core->GINTSTS & USB_OTG_GINTSTS_BOUTNAKEFF) != 0U)
  {
    Device()->DCTL |= USB_OTG_DCTL_CGONAK;
    (void)Spin([core]() { return (core->GINTSTS & USB_OTG_GINTSTS_BOUTNAKEFF) == 0U; });
  }
  Device()->DCTL |= USB_OTG_DCTL_SGONAK;
}

bool STM32OtgController::SetupLatched() const
{
  // STUP 要到主机的下一个令牌才置位（STM32H723 实测晚 0.7~1.1 ms），所以还要看更早
  // 出现的标志：STPKTRX，以及 FIFO 模式下已弹出或还在 RX FIFO 队首的 SETUP。
  // STUP is set only at the host's next token (0.7 to 1.1 ms later, measured on an
  // STM32H723), so the earlier indications count too: STPKTRX and, in FIFO mode, a SETUP
  // already popped or still at the head of the RX FIFO.
  if (setup_pending_ ||
      (OutEp(0)->DOEPINT & (USB_OTG_DOEPINT_STUP | USB_OTG_DOEPINT_STPKTRX)) != 0U)
  {
    return true;
  }
  if (hw_.dma || (core_->GINTSTS & USB_OTG_GINTSTS_RXFLVL) == 0U)
  {
    return false;
  }
  const uint32_t head = core_->GRXSTSR;
  const uint32_t kind = (head & USB_OTG_GRXSTSP_PKTSTS) >> USB_OTG_GRXSTSP_PKTSTS_Pos;
  return (head & USB_OTG_GRXSTSP_EPNUM) == 0U && kind == STS_SETUP_UPDT;
}

void STM32OtgController::Connect()
{
  core_->GINTSTS = 0xBFFFFFFFU;
  uint32_t mask =
      USB_OTG_GINTMSK_USBRST | USB_OTG_GINTMSK_ENUMDNEM | USB_OTG_GINTMSK_USBSUSPM |
      USB_OTG_GINTMSK_WUIM | USB_OTG_GINTMSK_IEPINT | USB_OTG_GINTMSK_OEPINT |
      USB_OTG_GINTMSK_IISOIXFRM | USB_OTG_GINTMSK_PXFRM_IISOOXFRM | USB_OTG_GINTMSK_SOFM;
  if (!hw_.dma)
  {
    mask |= USB_OTG_GINTMSK_RXFLVLM;
  }
  if (hw_.vbus_sensing)
  {
    mask |= USB_OTG_GINTMSK_SRQIM | USB_OTG_GINTMSK_OTGINT;
  }
  core_->GINTMSK = mask;
  core_->GAHBCFG |= USB_OTG_GAHBCFG_GINT;
  (void)USB_DevConnect(core_);
}

// ---------------------------------------------------------------------------
// Device
// ---------------------------------------------------------------------------

STM32USBOtgDevice::STM32USBOtgDevice(
    uint8_t index, USB_OTG_GlobalTypeDef* core, IRQn_Type irqn, uint8_t endpoint_count,
    size_t total_fifo, const HardwareConfig& hw, size_t rx_fifo_size,
    const std::initializer_list<RawData>& rx_ep_cfgs,
    const std::initializer_list<EPInConfig>& tx_ep_cfgs,
    USB::DeviceDescriptor::PacketSize0 packet_size, uint16_t vid, uint16_t pid,
    uint16_t bcd,
    const std::initializer_list<const USB::DescriptorStrings::LanguagePack*>& lang_list,
    const std::initializer_list<const std::initializer_list<USB::DeviceClass*>>& configs,
    ConstRawData uid)
    : STM32OtgController(core, irqn, endpoint_count, total_fifo, hw, rx_fifo_size,
                         rx_ep_cfgs, tx_ep_cfgs),
      USB::DeviceCore(*this, *this, USB::USBSpec::USB_2_1,
                      hw.high_speed ? USB::Speed::HIGH : USB::Speed::FULL, packet_size,
                      vid, pid, bcd, lang_list, configs, uid),
      index_(index)
{
  REQUIRE(index < MAX_DEVICES);
  for (uint8_t i = 0; i < endpoint_count_; ++i)
  {
    if (in_[i] != nullptr)
    {
      REQUIRE(Register(*in_[i]) == ErrorCode::OK);
    }
    if (out_[i] != nullptr)
    {
      REQUIRE(Register(*out_[i]) == ErrorCode::OK);
    }
  }
  devices_[index_] = this;
}

void STM32USBOtgDevice::StartController(bool in_isr)
{
  UNUSED(in_isr);
  // 断开时间未到时只记下"要接上"并使能中断线，到点后 owner 接上。
  // While the detach time has not passed only "attach wanted" is recorded and the line
  // enabled; the owner attaches once the time has passed.
  if (detach_.WantAttach())
  {
    MaskInterrupt();
    last_sof_position_ = NO_SOF_POSITION;
    Connect();
  }
  UnmaskInterrupt();
}

void STM32USBOtgDevice::StopController(bool in_isr)
{
  UNUSED(in_isr);
  HaltController();
  core_->GAHBCFG &= ~USB_OTG_GAHBCFG_GINT;
  core_->GINTMSK &= ~USB_OTG_GINTMSK_SOFM;
  (void)detach_.OnHalted(false);
}

void STM32USBOtgDevice::MaskInterrupt() { NVIC_DisableIRQ(irqn_); }

void STM32USBOtgDevice::UnmaskInterrupt() { NVIC_EnableIRQ(irqn_); }

bool STM32USBOtgDevice::PendInterrupt()
{
  NVIC_SetPendingIRQ(irqn_);
  return true;
}

void STM32USBOtgDevice::SetAddress(uint8_t address, AddressStage stage)
{
  // core 在状态阶段之前采用新地址（RM0090 "Endpoint initialization on SetAddress
  // command"）。
  // The core takes the new address before the status stage (RM0090 "Endpoint
  // initialization on SetAddress command").
  if (stage == AddressStage::BEFORE_STATUS)
  {
    Device()->DCFG =
        (Device()->DCFG & ~USB_OTG_DCFG_DAD) |
        ((static_cast<uint32_t>(address) << USB_OTG_DCFG_DAD_Pos) & USB_OTG_DCFG_DAD);
  }
}

void STM32USBOtgDevice::OnControlReady() { ArmSetup(); }

void STM32USBOtgDevice::HaltController()
{
  (void)USB_DevDisconnect(core_);
  // 断开后总线上没有包，NAK 生效和禁用只取决于芯片，照停止的步骤做完（RM0468 p.2807、
  // p.2799-2800）。只写 EPDIS 而不等握手会把端点留在使能状态：STM32H723 实测重新接上后
  // EP0 IN 还带着旧请求，主机枚举晚 5.7~7.1 s 或失败。
  // Detached, no packet is on the bus, so the NAK and the disable depend on the chip
  // alone and the stop procedure is carried through (RM0468 p.2807, p.2799-2800).
  // Writing EPDIS without the handshakes leaves endpoints enabled: measured on an
  // STM32H723, EP0 IN still held the old request after the re-attach and the host
  // enumerated 5.7 to 7.1 s late or not at all.
  for (uint8_t i = 0; i < endpoint_count_; ++i)
  {
    if (in_[i] == nullptr)
    {
      continue;
    }
    auto* in = InEp(i);
    if ((in->DIEPCTL & USB_OTG_DIEPCTL_EPENA) != 0U)
    {
      in->DIEPINT = USB_OTG_DIEPINT_INEPNE;
      in->DIEPCTL |= USB_OTG_DIEPCTL_SNAK;
      (void)Spin(
          [in]()
          {
            return (in->DIEPINT & USB_OTG_DIEPINT_INEPNE) != 0U ||
                   (in->DIEPCTL & USB_OTG_DIEPCTL_EPENA) == 0U;
          });
    }
    (void)in_[i]->DisableIn();
  }
  bool out_enabled = false;
  for (uint8_t i = 1; i < endpoint_count_; ++i)
  {
    out_enabled = out_enabled || (OutEp(i)->DOEPCTL & USB_OTG_DOEPCTL_EPENA) != 0U;
  }
  if (out_enabled)
  {
    // 未结束的 OUT 停止已经请求过全局 OUT NAK。 / A pending OUT stop has requested the
    // global OUT NAK already.
    if (out_stops_ == 0U)
    {
      RequestGlobalOutNak();
    }
    (void)Spin(
        [this]()
        {
          if (!hw_.dma)
          {
            DrainRxFifo();
          }
          return (core_->GINTSTS & USB_OTG_GINTSTS_BOUTNAKEFF) != 0U;
        });
    for (uint8_t i = 1; i < endpoint_count_; ++i)
    {
      if (out_[i] != nullptr)
      {
        (void)out_[i]->DisableOut();
      }
    }
  }
  // EP0 OUT 不能禁用（RM0468 p.2736），只置 NAK。 / EP0 OUT cannot be disabled (RM0468
  // p.2736); it only gets NAK.
  if ((OutEp(0)->DOEPCTL & USB_OTG_DOEPCTL_EPENA) != 0U)
  {
    OutEp(0)->DOEPCTL |= USB_OTG_DOEPCTL_SNAK;
  }
  (void)USB_FlushTxFifo(core_, 0x10U);
  (void)USB_FlushRxFifo(core_);
  for (uint8_t i = 0; i < endpoint_count_; ++i)
  {
    InEp(i)->DIEPINT = 0xFB7FU;
    OutEp(i)->DOEPINT = 0xFB7FU;
  }
  Device()->DIEPEMPMSK = 0U;
  last_sof_position_ = NO_SOF_POSITION;
  // 硬件上已没有任何请求：未结束的停止作废、不再报告，全局 OUT NAK 撤销。
  // The hardware holds no request any more: pending stops are void and reported no
  // more, and the global OUT NAK is withdrawn.
  for (uint8_t i = 0; i < endpoint_count_; ++i)
  {
    for (STM32OtgEndpoint* ep : {in_[i], out_[i]})
    {
      if (ep != nullptr)
      {
        ep->active_ = false;
        ep->stopping_ = false;
        ep->self_stop_ = false;
        ep->stop_failed_ = false;
        ep->stop_sofs_ = 0U;
      }
    }
  }
  in_stops_ = 0U;
  out_stops_ = 0U;
  setup_pending_ = false;
  Device()->DIEPMSK &= ~USB_OTG_DIEPMSK_INEPNEM;
  core_->GINTMSK &= ~USB_OTG_GINTMSK_GONAKEFFM;
  Device()->DCTL |= USB_OTG_DCTL_CGONAK;
}

void STM32USBOtgDevice::ResetController(bool in_isr)
{
  UNUSED(in_isr);
  // 断开要保持到主机看得见，否则主机不复位总线，核心等不到 OnBusReset（STM32H723 实测：
  // 只断开 8~10 µs 时主机 10 次里 9 次看不见）。这里只断开，不等：detach_ 按节拍计
  // RESET_DETACH_MS，到点后 owner 在 ServiceHardware 开头接上。
  // The detach has to last until the host sees it, or the host does not reset the bus
  // and the core never gets OnBusReset (measured on an STM32H723: a detach of 8 to 10 us
  // went unseen 9 times out of 10). Only the detach happens here, no wait: detach_
  // counts RESET_DETACH_MS on the tick and the owner attaches at the start of
  // ServiceHardware once it has passed.
  HaltController();
  if (detach_.OnHalted(true) == USB::DetachTimer::HaltResult::ATTACH_NOW)
  {
    last_sof_position_ = NO_SOF_POSITION;
    Connect();
  }
}

void STM32USBOtgDevice::Dispatch(USB_OTG_GlobalTypeDef* core)
{
  uint8_t index = MAX_DEVICES;
#if defined(USB_OTG_FS)
  if (core == USB_OTG_FS)
  {
    index = OTG_FS_INDEX;
  }
#endif
#if defined(USB_OTG_HS)
  if (core == USB_OTG_HS)
  {
    index = OTG_HS_INDEX;
  }
#endif
  if (index >= MAX_DEVICES)
  {
    return;
  }
  STM32USBOtgDevice* device = devices_[index];
  if (device == nullptr)
  {
    // 设备还没构造：在驱动接管控制器之前保持中断线静默。
    // No device yet: keep the line quiet until the driver owns the controller.
    core->GAHBCFG &= ~USB_OTG_GAHBCFG_GINT;
    return;
  }
  device->OnControllerInterrupt();
}

void STM32USBOtgDevice::ServiceHardware(bool in_isr)
{
  // 断开时间已到、要接上：先接上，再读这一轮的快照。
  // The detach time has passed and an attach is wanted: attach first, then take this
  // pass's snapshot.
  if (detach_.TakeAttach())
  {
    last_sof_position_ = NO_SOF_POSITION;
    Connect();
  }
  // 一次快照，不重读。顺序：总线复位、枚举完成（报告复位）、RX FIFO、IN（完成先于
  // SETUP）、OUT（数据完成、EP0 IN 完成、SETUP）、等时漏帧、SOF、恢复、挂起、断开。
  // One snapshot, no re-read. Order: bus reset, enumeration done (reports the reset),
  // RX FIFO, IN (completions before the SETUP), OUT (data completion, EP0 IN completion,
  // SETUP), missed isochronous frame, SOF, resume, suspend, disconnect.
  const uint32_t status = core_->GINTSTS & core_->GINTMSK;
  if ((status & USB_OTG_GINTSTS_USBRST) != 0U)
  {
    OnUsbReset(in_isr);
  }
  if ((status & USB_OTG_GINTSTS_ENUMDNE) != 0U)
  {
    OnEnumerationDone(in_isr);
  }
  if ((status & USB_OTG_GINTSTS_RXFLVL) != 0U)
  {
    DrainRxFifo();
  }
  if ((status & USB_OTG_GINTSTS_BOUTNAKEFF) != 0U)
  {
    HandleGlobalOutNak(in_isr);
  }
  if ((status & USB_OTG_GINTSTS_IEPINT) != 0U)
  {
    HandleInEndpoints(in_isr);
  }
  if ((status & USB_OTG_GINTSTS_OEPINT) != 0U)
  {
    HandleOutEndpoints(in_isr);
  }
  if ((status & USB_OTG_GINTSTS_IISOIXFR) != 0U)
  {
    HandleIsoIncomplete(in_isr);
  }
  if ((status & USB_OTG_GINTSTS_PXFR_INCOMPISOOUT) != 0U)
  {
    // 未完成的等时 OUT（RM0468 p.2803-2804）：仍使能、EONUM 等于本帧奇偶的端点在这一帧
    // 没收到包。手册的处理是禁用端点、等禁用完成、再为下一帧重新使能；这里不禁用，只把
    // 奇偶位改成下一帧，请求继续在硬件上等，也不向核心报告（等时 OUT 的服务区间没有包时
    // 不报告，见 Endpoint 的类说明）。禁用要先请求全局 OUT NAK，那会丢掉别的等时 OUT 包，
    // 而且赶不上下一帧。STM32H723 实测（FIFO 和 DMA）：约 5 万次改写读回都生效；开流后
    // 第一包被丢从 300 次里 137 次降到 0，主机隔帧漏发时的丢包从 40% 降到 0；有 2 次正
    // 赶上包在接收，数据和计数都没有出错。要在 SOF 之前处理：晚了帧号已变，奇偶对不上，
    // 什么也不做。
    // Incomplete isochronous OUT (RM0468 p.2803-2804): an endpoint still enabled whose
    // EONUM equals the parity of this frame received nothing in it. The manual disables
    // the endpoint, waits for the disable and enables it again for the next frame; here
    // it stays enabled and only its parity bit is moved to the next frame, the request
    // keeps waiting on the hardware and nothing is reported to the core (a service
    // interval of an isochronous OUT without a packet reports nothing, see the class
    // note of Endpoint). A disable needs global OUT NAK first, which drops the
    // isochronous OUT packets of other endpoints and misses the next frame. Measured on
    // an STM32H723 (FIFO and DMA): about 50,000 rewrites all read back as written; first
    // packet lost after a start from 137 of 300 to 0, loss with the host skipping every
    // other frame from 40 % to 0; twice the write met a packet being received and
    // neither data nor counts went wrong. This must run before the SOF: once the frame
    // number has moved on the parity no longer matches and nothing is done.
    const bool frame_odd = OddFrame(Device());
    core_->GINTSTS = USB_OTG_GINTSTS_PXFR_INCOMPISOOUT;
    for (uint8_t i = 1; i < endpoint_count_; ++i)
    {
      STM32OtgEndpoint* ep = out_[i];
      if (ep == nullptr || !ep->active_ || ep->stopping_ ||
          ep->GetType() != Endpoint::Type::ISOCHRONOUS)
      {
        continue;
      }
      auto* regs = OutEp(i);
      const uint32_t control = regs->DOEPCTL;
      // DOEPCTL 的 EONUM 与 DIEPCTL 同为位 16，器件头文件只给了 DIEPCTL 的名字。
      // EONUM is bit 16 in DOEPCTL as in DIEPCTL; the device header names only the
      // latter.
      const bool armed_odd = (control & USB_OTG_DIEPCTL_EONUM_DPID) != 0U;
      if ((control & USB_OTG_DOEPCTL_EPENA) == 0U || armed_odd != frame_odd)
      {
        continue;
      }
      regs->DOEPCTL |=
          frame_odd ? USB_OTG_DOEPCTL_SD0PID_SEVNFRM : USB_OTG_DOEPCTL_SODDFRM;
    }
  }
  if ((status & USB_OTG_GINTSTS_SOF) != 0U)
  {
    HandleSof(in_isr);
  }
  if ((status & USB_OTG_GINTSTS_WKUINT) != 0U)
  {
    core_->GINTSTS = USB_OTG_GINTSTS_WKUINT;
    Device()->DCTL &= ~USB_OTG_DCTL_RWUSIG;
    core_->GINTMSK |= USB_OTG_GINTMSK_SOFM;
    OnResume(in_isr);
  }
  if ((status & USB_OTG_GINTSTS_USBSUSP) != 0U)
  {
    core_->GINTSTS = USB_OTG_GINTSTS_USBSUSP;
    if ((Device()->DSTS & USB_OTG_DSTS_SUSPSTS) != 0U)
    {
      core_->GINTMSK &= ~USB_OTG_GINTMSK_SOFM;
      FinishPendingStops(in_isr);
      OnSuspend(in_isr);
    }
  }
  if ((status & USB_OTG_GINTSTS_SRQINT) != 0U)
  {
    core_->GINTSTS = USB_OTG_GINTSTS_SRQINT;
  }
  if ((status & USB_OTG_GINTSTS_OTGINT) != 0U)
  {
    const uint32_t otg = core_->GOTGINT;
    core_->GOTGINT = otg;
    if ((otg & USB_OTG_GOTGINT_SEDET) != 0U)
    {
      core_->GINTMSK &= ~USB_OTG_GINTMSK_SOFM;
      FinishPendingStops(in_isr);
      OnDisconnect(in_isr);
    }
  }
}

void STM32USBOtgDevice::HandleGlobalOutNak(bool in_isr)
{
  // 全局 OUT NAK 生效后各 OUT 端点可以禁用（RM0468 p.2799-2800）。这是电平状态：处理一次
  // 就屏蔽，禁用握手没有完成的端点留给 SOF 计数再查。
  // With the global OUT NAK in effect the OUT endpoints may be disabled (RM0468
  // p.2799-2800). The status is a level: it is masked after one handling, and an endpoint
  // whose disable handshake did not finish is left to the SOF count.
  for (uint8_t i = 1; i < endpoint_count_; ++i)
  {
    STM32OtgEndpoint* ep = out_[i];
    if (ep != nullptr && ep->stopping_ && !ep->stop_failed_)
    {
      (void)ep->FinishStop(in_isr, true);
    }
  }
  if (out_stops_ != 0U)
  {
    core_->GINTMSK &= ~USB_OTG_GINTMSK_GONAKEFFM;
  }
}

void STM32USBOtgDevice::FinishPendingStops(bool in_isr)
{
  if (in_stops_ == 0U && out_stops_ == 0U)
  {
    return;
  }
  for (uint8_t i = 0; i < endpoint_count_; ++i)
  {
    STM32OtgEndpoint* ep = in_[i];
    if (ep != nullptr && ep->stopping_ && !ep->stop_failed_ &&
        !ep->FinishStop(in_isr, true))
    {
      ep->stop_failed_ = true;
      ep->OnHardwareComplete(in_isr, 0U, ErrorCode::TIMEOUT);
    }
  }
  if (out_stops_ == 0U)
  {
    return;
  }
  // 全局 OUT NAK 已在停止时请求；总线上没有包，生效只取决于芯片。FIFO 模式要先弹出 RX
  // FIFO 里的状态项（RM0468 p.2798）。
  // The global OUT NAK was requested at the stop; with no packet on the bus its effect
  // depends on the chip alone. FIFO mode pops the RX FIFO status entries first (RM0468
  // p.2798).
  const bool effective = Spin(
      [this]()
      {
        if (!hw_.dma)
        {
          DrainRxFifo();
        }
        return (core_->GINTSTS & USB_OTG_GINTSTS_BOUTNAKEFF) != 0U;
      });
  for (uint8_t i = 1; i < endpoint_count_; ++i)
  {
    STM32OtgEndpoint* ep = out_[i];
    if (ep != nullptr && ep->stopping_ && !ep->stop_failed_ &&
        !(effective && ep->FinishStop(in_isr, true)))
    {
      ep->stop_failed_ = true;
      ep->OnHardwareComplete(in_isr, 0U, ErrorCode::TIMEOUT);
    }
  }
}

void STM32USBOtgDevice::CountStopTimeouts(bool in_isr)
{
  // FIFO 模式下全局 OUT NAK 靠弹出 RX FIFO 里的“已生效”状态项才生效（RM0468 p.2798）。
  // 这一项紧挨着一个 SETUP 项出现时内容会错乱，GONAKEFF 不再置位，RX FIFO 之后一直是
  // 空的（STM32H723 实测：134 219 次请求里 4 次，都夹着一个 SETUP）。请求已过一帧、仍未
  // 生效、RX FIFO 为空，就撤销后重新请求（实测 4 次都在 2.3 µs 内生效）。
  // In FIFO mode the global OUT NAK takes effect when its "effective" status entry is
  // popped from the RX FIFO (RM0468 p.2798). When that entry appears right next to a
  // SETUP entry its content can be garbled; GONAKEFF then never sets and the RX FIFO
  // stays empty (measured on an STM32H723: 4 of 134,219 requests, each with a SETUP in
  // between). A request one frame old, still not effective, with an empty RX FIFO is
  // withdrawn and made again (measured: effective within 2.3 us in 4 of 4).
  // DMA 模式下同样会丢（实测 2 次现场：GONSTS 为 1，GONAKEFF 16 帧里一直是 0，RX 状态
  // 队首是 EP0 的 SETUP 完成项），也重新请求；这时控制器自己排空 RX FIFO，RXFLVL 不
  // 适用，只看 GONAKEFF（实测遇到 1 次，重新请求后在下一帧生效）。
  // The same loss occurs in DMA mode (two captures: GONSTS at 1, GONAKEFF at 0 through
  // 16 frames, an EP0 SETUP-done entry at the head of the RX status queue) and is
  // re-requested as well; there the controller drains the RX FIFO itself and RXFLVL
  // does not apply, so only GONAKEFF is looked at (met once, effective in the frame
  // after the re-request).
  const uint32_t not_yet = hw_.dma
                               ? USB_OTG_GINTSTS_BOUTNAKEFF
                               : (USB_OTG_GINTSTS_BOUTNAKEFF | USB_OTG_GINTSTS_RXFLVL);
  if (out_stops_ != 0U && (core_->GINTSTS & not_yet) == 0U)
  {
    for (uint8_t i = 1; i < endpoint_count_; ++i)
    {
      const STM32OtgEndpoint* ep = out_[i];
      if (ep != nullptr && ep->stopping_ && !ep->stop_failed_ && ep->stop_sofs_ != 0U)
      {
        Device()->DCTL |= USB_OTG_DCTL_CGONAK;
        Device()->DCTL |= USB_OTG_DCTL_SGONAK;
        break;
      }
    }
  }
  for (uint8_t i = 0; i < endpoint_count_; ++i)
  {
    for (STM32OtgEndpoint* ep : {in_[i], out_[i]})
    {
      if (ep == nullptr || !ep->stopping_ || ep->stop_failed_)
      {
        continue;
      }
      if (ep->stop_sofs_ < STM32OtgEndpoint::STOP_TIMEOUT_SOFS)
      {
        ++ep->stop_sofs_;
        continue;
      }
      // 到上限再查一次；后端自己发起的停止没有核心在等，不报 TIMEOUT，继续等。
      // One more check at the bound; a backend-issued stop has no core waiting for it,
      // so no TIMEOUT is reported and it keeps waiting.
      if (!ep->FinishStop(in_isr, false) && !ep->self_stop_)
      {
        ep->stop_failed_ = true;
        ep->OnHardwareComplete(in_isr, 0U, ErrorCode::TIMEOUT);
      }
    }
  }
}

void STM32USBOtgDevice::OnUsbReset(bool in_isr)
{
  // RM0468 p.2793 "Endpoint initialization on USB reset"。端点上的请求由 ENUMDNE 报告的
  // 复位关闭（每个端点自己的停止，含 TX FIFO 刷新，p.2807 要求刷新在禁用之后）。这里
  // 给所有端点写 SNAK，让旧数据不再上线；XFRC 和 INEPNE 留给停止判断结果。
  // RM0468 p.2793 "Endpoint initialization on USB reset". Requests on the endpoints are
  // closed by the reset reported at ENUMDNE (each endpoint's own stop, TX FIFO flush
  // included; p.2807 puts the flush after the disable). Here every endpoint gets SNAK
  // so old data stays off the wire; XFRC and INEPNE are left for the stop's result.
  core_->GINTSTS = USB_OTG_GINTSTS_USBRST;
  last_sof_position_ = NO_SOF_POSITION;
  Device()->DCTL &= ~USB_OTG_DCTL_RWUSIG;
  for (uint8_t i = 0; i < endpoint_count_; ++i)
  {
    auto* in = InEp(i);
    auto* out = OutEp(i);
    in->DIEPINT = DIEPINT_ALL_BUT_NAK_EFFECTIVE & ~USB_OTG_DIEPINT_XFRC;
    in->DIEPCTL = (in->DIEPCTL & ~USB_OTG_DIEPCTL_STALL) | USB_OTG_DIEPCTL_SNAK;
    out->DOEPINT = 0xFB7FU & ~USB_OTG_DOEPINT_XFRC;
    out->DOEPCTL = (out->DOEPCTL & ~USB_OTG_DOEPCTL_STALL) | USB_OTG_DOEPCTL_SNAK;
  }
  // 复位期间总线上没有包：未结束的停止在这里结束，早于 ENUMDNE 报告的复位。
  // No packet is on the bus during the reset: pending stops end here, before the reset
  // that ENUMDNE reports.
  FinishPendingStops(in_isr);
  setup_pending_ = false;
  Device()->DIEPEMPMSK = 0U;
  Device()->DAINTMSK = 0x10001U;
  Device()->DOEPMSK =
      USB_OTG_DOEPMSK_STUPM | USB_OTG_DOEPMSK_XFRCM | USB_OTG_DOEPMSK_OTEPSPRM;
  Device()->DIEPMSK = USB_OTG_DIEPMSK_TOM | USB_OTG_DIEPMSK_XFRCM |
                      (in_stops_ != 0U ? USB_OTG_DIEPMSK_INEPNEM : 0U);
  Device()->DCFG &= ~USB_OTG_DCFG_DAD;
  ArmSetup();
}

void STM32USBOtgDevice::OnEnumerationDone(bool in_isr)
{
  core_->GINTSTS = USB_OTG_GINTSTS_ENUMDNE;
  (void)USB_ActivateSetup(core_);
  const uint8_t speed = USB_GetDevSpeed(core_);
  (void)USB_SetTurnaroundTime(core_, HAL_RCC_GetHCLKFreq(), speed);
  core_->GINTMSK |= USB_OTG_GINTMSK_SOFM;
#if defined(USBD_HS_SPEED)
  OnBusReset(in_isr, speed == USBD_HS_SPEED ? USB::Speed::HIGH : USB::Speed::FULL);
#else
  UNUSED(speed);
  OnBusReset(in_isr, USB::Speed::FULL);
#endif
}

void STM32USBOtgDevice::HandleInEndpoints(bool in_isr)
{
  uint32_t bits = Device()->DAINT & Device()->DAINTMSK & 0xFFFFU;
  while (bits != 0U)
  {
    const uint8_t num = static_cast<uint8_t>(__builtin_ctz(bits));
    bits &= bits - 1U;
    auto* regs = InEp(num);
    const uint32_t raw = regs->DIEPINT;
    const bool fill =
        ((Device()->DIEPEMPMSK >> num) & 1U) != 0U && (raw & USB_OTG_DIEPINT_TXFE) != 0U;
    STM32OtgEndpoint* ep = in_[num];
    if ((raw & USB_OTG_DIEPINT_XFRC) != 0U)
    {
      Device()->DIEPEMPMSK &= ~(1UL << num);
      regs->DIEPINT = USB_OTG_DIEPINT_XFRC;
      if (ep != nullptr)
      {
        ep->Complete(in_isr);
      }
    }
    if ((raw & USB_OTG_DIEPINT_TOC) != 0U)
    {
      regs->DIEPINT = USB_OTG_DIEPINT_TOC;
    }
    if ((raw & USB_OTG_DIEPINT_ITTXFE) != 0U)
    {
      regs->DIEPINT = USB_OTG_DIEPINT_ITTXFE;
    }
    if (ep != nullptr && ep->stopping_ && !ep->stop_failed_)
    {
      // NAK 生效，或请求已自己结束（上面的 XFRC 已经报告并清掉 stopping_ 时不会到这里）。
      // The NAK took effect, or the request ended by itself (when the XFRC above was
      // reported and cleared stopping_, this is not reached).
      if (((raw & USB_OTG_DIEPINT_INEPNE) != 0U ||
           (regs->DIEPCTL & USB_OTG_DIEPCTL_EPENA) == 0U) &&
          !ep->FinishStop(in_isr, true))
      {
        // 禁用握手没有在上限内完成：清掉标志，免得掩码开着时反复进中断；SOF 计数到
        // 上限时再查，仍不行就报 TIMEOUT。
        // The disable handshake did not finish within its bound: the flag is cleared so
        // that the open mask does not keep raising the interrupt; the SOF count checks
        // again at its bound and reports TIMEOUT if it still fails.
        regs->DIEPINT = USB_OTG_DIEPINT_INEPNE;
      }
    }
    else if ((raw & USB_OTG_DIEPINT_INEPNE) != 0U &&
             (Device()->DIEPMSK & USB_OTG_DIEPMSK_INEPNEM) != 0U)
    {
      // 别的端点在等 INEPNE 时掩码是开着的；这个端点没有停止，清掉以免反复进中断。
      // The mask is open while another endpoint waits for INEPNE; this endpoint has no
      // stop, so the flag is cleared to keep the interrupt from repeating.
      regs->DIEPINT = USB_OTG_DIEPINT_INEPNE;
    }
    if (fill)
    {
      if (ep != nullptr)
      {
        ep->FillFifo();
      }
      else
      {
        Device()->DIEPEMPMSK &= ~(1UL << num);
      }
    }
  }
}

void STM32USBOtgDevice::HandleOutEndpoints(bool in_isr)
{
  uint32_t bits = (Device()->DAINT & Device()->DAINTMSK) >> 16U;
  while (bits != 0U)
  {
    const uint8_t num = static_cast<uint8_t>(__builtin_ctz(bits));
    bits &= bits - 1U;
    auto* regs = OutEp(num);
    const uint32_t events = regs->DOEPINT;
    regs->DOEPINT =
        events & (USB_OTG_DOEPINT_XFRC | USB_OTG_DOEPINT_STUP | USB_OTG_DOEPINT_EPDISD |
                  USB_OTG_DOEPINT_OTEPDIS | USB_OTG_DOEPINT_OTEPSPR |
                  USB_OTG_DOEPINT_STPKTRX | USB_OTG_DOEPINT_NAK);
    if (num == 0U)
    {
      // STPKTRX 先到、STUP 要等主机的下一个令牌：这里清掉了 STPKTRX，所以记下“SETUP 已
      // 收到、还没有交付”，交付时清零。
      // STPKTRX comes first and STUP only at the host's next token: STPKTRX was cleared
      // above, so "a SETUP was received and not handed over" is remembered until the
      // hand-over.
      if ((events & USB_OTG_DOEPINT_STPKTRX) != 0U)
      {
        setup_pending_ = true;
      }
      HandleSetup(in_isr, regs, events);
    }
    else if ((events & USB_OTG_DOEPINT_XFRC) != 0U && out_[num] != nullptr)
    {
      out_[num]->Complete(in_isr);
    }
  }
}

void STM32USBOtgDevice::HandleSetup(bool in_isr, USB_OTG_OUTEndpointTypeDef* out0,
                                    uint32_t events)
{
  const bool setup = (events & USB_OTG_DOEPINT_STUP) != 0U;
  // SETUP 先拷出来：下面报告的完成可能让核心重新装填 EP0 OUT（DMA 时改 DOEPDMA），
  // 停止也可能再弹 RX FIFO（覆盖 setup_）。
  // Copy the SETUP out first: a completion reported below may make the core re-arm EP0
  // OUT (DOEPDMA changes with DMA), and a stop may pop the RX FIFO again (overwriting
  // rx_setup_).
  USB::SetupPacket packet = {};
  if (setup)
  {
    if (hw_.dma)
    {
      // 最后一个 SETUP 位于 core 前移后的 DMA 地址的正下方。
      // The last SETUP sits right below the DMA address the core advanced to.
      STM32_InvalidateDCacheByAddr(hw_.ep0_dma_buffer.addr_, EP0_DMA_SIZE);
      const uintptr_t start = reinterpret_cast<uintptr_t>(hw_.ep0_dma_buffer.addr_);
      uintptr_t at = static_cast<uintptr_t>(out0->DOEPDMA) - SETUP_SIZE;
      if (at < start || at + SETUP_SIZE > start + EP0_DMA_SIZE)
      {
        at = start;
      }
      std::memcpy(&packet, reinterpret_cast<const void*>(at), sizeof(packet));
    }
    else
    {
      std::memcpy(&packet, rx_setup_, sizeof(packet));
    }
  }
  STM32OtgEndpoint* ep = out_[0];
  if ((events & USB_OTG_DOEPINT_XFRC) != 0U)
  {
    // EP0 OUT 也会把 SETUP 和状态阶段事件报告为 XFRC。数据完成的判据：FIFO 模式下 XFRC
    // 只来自“传输完成”状态项（310A 的 STPKTRX 除外，同 HAL）；DMA 模式下没有 SETUP 时
    // 同 HAL，有 SETUP 时看数据请求在途且 PKTCNT 已减到 0（SETUP 只减 STUPCNT）。
    // EP0 OUT also reports SETUP and status-phase events as XFRC. A data completion is:
    // in FIFO mode, XFRC from the "transfer complete" entry (except STPKTRX on 310A, as
    // HAL); with DMA, as HAL without a SETUP, and with a SETUP, a data request in flight
    // whose PKTCNT reached 0 (a SETUP decrements STUPCNT only).
    bool data = false;
    if (!hw_.dma)
    {
      data = !(core_id_ == USB_OTG_CORE_ID_310A && (events & USB_OTG_DOEPINT_STPKTRX));
    }
    else if (!setup)
    {
      data = (events & USB_OTG_DOEPINT_OTEPSPR) == 0U &&
             !(core_id_ > USB_OTG_CORE_ID_300A && (events & USB_OTG_DOEPINT_STPKTRX));
    }
    else
    {
      data = (out0->DOEPTSIZ & USB_OTG_DOEPTSIZ_PKTCNT) == 0U;
    }
    if (data && ep != nullptr)
    {
      ep->Complete(in_isr);
    }
    // DMA 时 EP0 OUT 只有在使能时 core 才存 SETUP，传输完成让它停在禁用；没有新 SETUP
    // 时在这里重新装填。
    // With DMA a SETUP is stored only while EP0 OUT is enabled and a completion leaves
    // it disabled; without a new SETUP it is re-armed here.
    if (hw_.dma && !setup)
    {
      ArmSetup();
    }
  }
  if (!setup)
  {
    return;
  }
  // 顺序约定：交 SETUP 之前报告 EP0 上硬件已标记的完成。IN 的事件在第 1 步快照之后才
  // 置位的，这里再读一次。
  // Ordering contract: report every completion the hardware flagged on EP0 before the
  // SETUP. An IN completion flagged after the snapshot is caught by this second read.
  auto* in0 = InEp(0);
  if ((in0->DIEPINT & USB_OTG_DIEPINT_XFRC) != 0U)
  {
    Device()->DIEPEMPMSK &= ~1UL;
    in0->DIEPINT = USB_OTG_DIEPINT_XFRC;
    if (in_[0] != nullptr)
    {
      in_[0]->Complete(in_isr);
    }
  }
  // 控制端点上未结束的停止在交 SETUP 之前结束：core 收到 SETUP 时已让 EP0 IN 回 NAK
  // （RM0468 p.2797），禁用握手由芯片定上限。
  // A pending stop on the control endpoint ends before the SETUP is handed over: the
  // core made EP0 IN NAK when the SETUP arrived (RM0468 p.2797), so the disable
  // handshake is bounded by the chip.
  if (in_[0] != nullptr && in_[0]->stopping_ && !in_[0]->stop_failed_)
  {
    (void)in_[0]->FinishStop(in_isr, true);
  }
  setup_pending_ = false;
  OnSetup(in_isr, packet);
  if (hw_.dma && core_id_ > USB_OTG_CORE_ID_300A)
  {
    ArmSetup();
  }
}

void STM32USBOtgDevice::HandleIsoIncomplete(bool in_isr)
{
  // 参考手册 "Incomplete isochronous IN data transfers"：至少一个等时 IN 端点在刚结束的
  // 帧里没有发出数据。端点仍使能、且奇偶帧位等于中断到来时帧号奇偶的，就是没发完的那
  // 个；同步停止它，请求以 FAILED 结束（停止前已发完的照实报 OK）。帧号在清标志之前读。
  // "Incomplete isochronous IN data transfers" (reference manual): at least one
  // isochronous IN endpoint sent nothing in the frame that ended. The one still enabled
  // whose even/odd bit equals the parity of the frame number read now is it; stop it
  // synchronously and end the request with FAILED (OK if it was sent before the stop).
  // The frame number is read before the flag is cleared.
  const bool ended_odd = OddFrame(Device());
  core_->GINTSTS = USB_OTG_GINTSTS_IISOIXFR;
  for (uint8_t i = 1; i < endpoint_count_; ++i)
  {
    STM32OtgEndpoint* ep = in_[i];
    if (ep == nullptr || !ep->active_ || ep->GetType() != Endpoint::Type::ISOCHRONOUS)
    {
      continue;
    }
    const uint32_t control = InEp(i)->DIEPCTL;
    const bool armed_odd = (control & USB_OTG_DIEPCTL_EONUM_DPID) != 0U;
    if ((control & USB_OTG_DIEPCTL_EPENA) == 0U || armed_odd != ended_odd)
    {
      continue;
    }
    if (ep->stopping_)
    {
      continue;
    }
    const auto outcome = ep->StopIn();
    if (outcome.result == ErrorCode::PENDING)
    {
      // 后端自己发起的停止：核心没有在等，结束时作为普通完成报告 FAILED。
      // A stop issued by the backend: the core is not waiting, and its end is reported
      // as a plain completion with FAILED.
      ep->self_stop_ = true;
      ep->stop_report_ = ErrorCode::FAILED;
      continue;
    }
    ep->active_ = false;
    ep->OnHardwareComplete(
        in_isr, outcome.actual,
        outcome.result == ErrorCode::OK ? ErrorCode::OK : ErrorCode::FAILED);
  }
}

void STM32USBOtgDevice::HandleSof(bool in_isr)
{
  core_->GINTSTS = USB_OTG_GINTSTS_SOF;
  const uint32_t position =
      (Device()->DSTS & USB_OTG_DSTS_FNSOF) >> USB_OTG_DSTS_FNSOF_Pos;
  // 同一个 SOF 令牌，core 可能约 1.4 µs 后再置一次 GINTSTS.SOF（STM32H723 内置全速 PHY
  // 实测），两次 DSTS.FNSOF 相同，所以刚处理过的位置跳过。高速时低 3 位是微帧号。
  // The core can set GINTSTS.SOF again about 1.4 µs later for the same token (measured
  // on an STM32H723 with the embedded full-speed PHY) with the same DSTS.FNSOF, so a
  // position just handled is skipped. At high speed the low 3 bits are the microframe.
  if (position == last_sof_position_)
  {
    return;
  }
  last_sof_position_ = position;
  if (in_stops_ != 0U || out_stops_ != 0U)
  {
    CountStopTimeouts(in_isr);
  }
  if (GetSpeed() == USB::Speed::HIGH)
  {
    OnStartOfFrame(in_isr, static_cast<uint16_t>(position >> 3U),
                   static_cast<uint8_t>(position & 7U));
  }
  else
  {
    OnStartOfFrame(in_isr, static_cast<uint16_t>(position & 0x7FFU));
  }
}

// ---------------------------------------------------------------------------
// Endpoint
// ---------------------------------------------------------------------------

STM32OtgEndpoint::STM32OtgEndpoint(STM32OtgController& controller, EPNumber number,
                                   Direction direction, size_t fifo_size, RawData buffer)
    : Endpoint(number, direction, buffer), controller_(controller), fifo_size_(fifo_size)
{
  const uint8_t num = EPNumberToInt8(number);
  REQUIRE(num < controller.endpoint_count_);
  if (direction == Direction::IN)
  {
    controller.in_[num] = this;
  }
  else
  {
    controller.out_[num] = this;
  }
}

Endpoint::HwLimits STM32OtgEndpoint::HardwareLimits(Direction direction, Type type) const
{
  const size_t room =
      direction == Direction::IN ? fifo_size_ : controller_.rx_packet_limit_;
  HwLimits limits;
  if (type == Type::CONTROL)
  {
    // DIEPTSIZ0 XFRSIZ 7 位、PKTCNT 2 位，DOEPTSIZ0 PKTCNT 1 位（RM0468
    // p.2734、p.2740）。
    // DIEPTSIZ0 has a 7-bit XFRSIZ and a 2-bit PKTCNT, DOEPTSIZ0 a 1-bit PKTCNT
    // (RM0468 p.2734, p.2740).
    limits.max_packet_size = LibXR::min<size_t>(64U, room);
    limits.max_packets = 1U;
    limits.max_bytes = limits.max_packet_size;
    // EP0 OUT 总是收进设备的 EP0 缓冲或由 CPU 从 RX FIFO 拷贝；EP0 IN 在 DMA 时由 core
    // 直接读槽。
    // EP0 OUT always receives into the device's EP0 buffer or is copied from the RX FIFO
    // by the CPU; with DMA the core reads the EP0 IN slot itself.
    limits.direct_memory = direction == Direction::IN && Dma();
    return limits;
  }
  limits.direct_memory = Dma();
  limits.max_packet_size = room;
  if (type == Type::ISOCHRONOUS)
  {
    // MULCNT=1：一帧一个事务，奇偶帧位按一次使能设定。
    // MULCNT=1: one transaction per frame; the even/odd bit is set per enable.
    limits.max_packets = 1U;
    limits.max_bytes = room;
    return limits;
  }
  limits.max_packets = MAX_PACKETS;
  limits.max_bytes = MAX_BYTES;
  return limits;
}

void STM32OtgEndpoint::ConfigureHardware(const Config& cfg)
{
  const uint8_t num = Num();
  const uint32_t packet = cfg.max_packet_size;
  auto* dev = controller_.Device();
  if (IsIn())
  {
    auto* regs = controller_.InEp(num);
    dev->DAINTMSK |= 1UL << num;
    if (num == 0U)
    {
      regs->DIEPCTL = (regs->DIEPCTL & ~USB_OTG_DIEPCTL_MPSIZ) | Ep0SizeCode(packet);
      return;
    }
    regs->DIEPCTL = (regs->DIEPCTL & ~(USB_OTG_DIEPCTL_MPSIZ | USB_OTG_DIEPCTL_EPTYP |
                                       USB_OTG_DIEPCTL_TXFNUM)) |
                    (packet & USB_OTG_DIEPCTL_MPSIZ) | EndpointType(cfg.type) |
                    (static_cast<uint32_t>(num) << USB_OTG_DIEPCTL_TXFNUM_Pos) |
                    USB_OTG_DIEPCTL_SD0PID_SEVNFRM | USB_OTG_DIEPCTL_USBAEP;
    return;
  }
  dev->DAINTMSK |= (1UL << num) << 16U;
  // EP0 OUT 跟随 EP0 IN 的包大小。
  // EP0 OUT follows the EP0 IN packet size.
  if (num == 0U)
  {
    return;
  }
  auto* regs = controller_.OutEp(num);
  regs->DOEPCTL = (regs->DOEPCTL & ~(USB_OTG_DOEPCTL_MPSIZ | USB_OTG_DOEPCTL_EPTYP)) |
                  (packet & USB_OTG_DOEPCTL_MPSIZ) | EndpointType(cfg.type) |
                  USB_OTG_DOEPCTL_SD0PID_SEVNFRM | USB_OTG_DOEPCTL_USBAEP;
}

void STM32OtgEndpoint::CloseHardware(bool in_isr)
{
  UNUSED(in_isr);
  const uint8_t num = Num();
  // EP0 保持激活 / EP0 stays active.
  if (num == 0U)
  {
    return;
  }
  // 关闭的端点保持 USBAEP 并置 NAK：清掉 USBAEP 的端点对令牌不应答，主机记传输错误，
  // Windows usbser 随后停用管道（STM32H723 实测：线程里的 Close 到 Configure 之间）。
  // A closed endpoint keeps USBAEP and NAKs: one with USBAEP cleared does not answer
  // tokens, the host counts transaction errors and Windows usbser then halts the pipe
  // (measured on an STM32H723, between a Close and a Configure made in a thread).
  auto* dev = controller_.Device();
  if (IsIn())
  {
    dev->DIEPEMPMSK &= ~(1UL << num);
    dev->DAINTMSK &= ~(1UL << num);
    controller_.InEp(num)->DIEPCTL |= USB_OTG_DIEPCTL_SNAK;
    return;
  }
  dev->DAINTMSK &= ~((1UL << num) << 16U);
  controller_.OutEp(num)->DOEPCTL |= USB_OTG_DOEPCTL_SNAK;
}

ErrorCode STM32OtgEndpoint::StartHardware(const HwRequest& request, bool in_isr)
{
  UNUSED(in_isr);
  ASSERT(!active_);
  const uint8_t num = Num();
  const uint32_t packet = MaxPacketSize();
  const bool iso = GetType() == Type::ISOCHRONOUS;
  // 核心按 HardwareLimits 保证长度。
  // The core keeps the length within HardwareLimits.
  ASSERT(num == 0U || request.length <= MAX_BYTES);
  const uint32_t length = static_cast<uint32_t>(request.length);
  const uint32_t packets = length == 0U ? 1U : (length + packet - 1U) / packet;
  hw_data_ = request.data;
  hw_length_ = request.length;
  hw_done_ = 0U;
  auto* dev = controller_.Device();

  if (IsIn())
  {
    auto* regs = controller_.InEp(num);
    // 总线复位清掉非 EP0 端点的 USBAEP（RM0468 p.2732）；复位后、重新配置之前不启动。
    // A bus reset clears USBAEP of every endpoint but EP0 (RM0468 p.2732); nothing
    // starts between the reset and the next configuration.
    if (num != 0U && (regs->DIEPCTL & USB_OTG_DIEPCTL_USBAEP) == 0U)
    {
      return ErrorCode::STATE_ERR;
    }
    uint32_t size = (packets << PKTCNT_POS) | length;
    if (iso)
    {
      size |= 1UL << MULCNT_POS;
    }
    regs->DIEPTSIZ = size;
    if (Dma())
    {
      if (length != 0U)
      {
        STM32_CleanDCacheByAddr(hw_data_, length);
      }
      regs->DIEPDMA = Address(hw_data_);
    }
    if (iso)
    {
      regs->DIEPCTL |=
          OddFrame(dev) ? USB_OTG_DIEPCTL_SD0PID_SEVNFRM : USB_OTG_DIEPCTL_SODDFRM;
    }
    active_ = true;
    regs->DIEPCTL |= USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA;
    if (!Dma() && length != 0U)
    {
      // 立即写入当前能放下的数据，而不是等待下一次 TXFE 中断。
      // Write what fits now instead of waiting for the next TXFE interrupt.
      FillFifo();
      if (hw_done_ < hw_length_)
      {
        dev->DIEPEMPMSK |= 1UL << num;
      }
    }
    if (num == 0U && controller_.SetupLatched())
    {
      // 装入之后发现硬件已收到还没有交付的 SETUP：上面的 CNAK 撤掉了 core 为它置的
      // NAK，这个应答属于被取代的传输，撤回。
      // A SETUP not handed over yet was found after the load: the CNAK above removed the
      // NAK the core set for it, and this reply belongs to the superseded transfer, so
      // it is retracted.
      dev->DIEPEMPMSK &= ~1UL;
      regs->DIEPCTL |= USB_OTG_DIEPCTL_SNAK;
      if (DisableIn())
      {
        active_ = false;
        return ErrorCode::FAILED;
      }
      // 禁用没有在芯片的上限内完成：这个应答已有一包在线上。不等它；按后端自己发起的
      // 停止处理，在交付那个 SETUP 之前结束并报告 ABORTED。
      // The disable did not finish within the chip's bound: a packet of this reply is
      // on the wire. It is not waited for; this becomes a stop issued by the backend,
      // ended and reported as ABORTED before that SETUP is handed over.
      self_stop_ = true;
      stop_report_ = ErrorCode::ABORTED;
      BeginPending();
    }
    return ErrorCode::OK;
  }

  auto* regs = controller_.OutEp(num);
  if (num == 0U)
  {
    // 已收到还没有交付的 SETUP 时不装入：DMA 模式下改写 DOEPDMA 会丢掉它的位置。
    // Nothing is loaded while a SETUP waits to be handed over: with DMA, rewriting
    // DOEPDMA would lose where it was stored.
    if (controller_.SetupLatched())
    {
      return ErrorCode::FAILED;
    }
    // 一次一个包，收进 EP0 缓冲（DMA）或由 CPU 拷进槽（FIFO）；EP0 持续接收 SETUP。
    // 长度 0（状态阶段）也编程一个满包，数据丢弃。
    // One packet at a time into the EP0 buffer (DMA) or copied into the slot by the CPU
    // (FIFO); EP0 keeps accepting SETUPs. Length 0 (status stage) also programs a full
    // packet and drops its data.
    hw_programmed_ = packet;
    regs->DOEPTSIZ = USB_OTG_DOEPTSIZ_STUPCNT | (1UL << PKTCNT_POS) | packet;
    if (Dma())
    {
      regs->DOEPDMA = Address(controller_.hw_.ep0_dma_buffer.addr_);
    }
  }
  else
  {
    if ((regs->DOEPCTL & USB_OTG_DOEPCTL_USBAEP) == 0U)
    {
      return ErrorCode::STATE_ERR;
    }
    // 长度是包长的整数倍（Endpoint::Receive）；OUT 大小按字对齐（RM0468 p.2801）。
    // The length is a packet multiple (Endpoint::Receive); OUT sizes are word aligned
    // (RM0468 p.2801).
    ASSERT(length != 0U);
    const uint32_t size = (length + 3U) & ~3U;
    hw_programmed_ = size;
    regs->DOEPTSIZ = (packets << PKTCNT_POS) | size;
    if (Dma())
    {
      // 先 clean：之后被换出的脏行不得覆盖已接收的字节。核心保证到缓存行的整数倍可写。
      // Clean first: a dirty line evicted later must not overwrite received bytes. The
      // core promises the bytes up to a cache-line multiple are writable.
      STM32_CleanDCacheByAddr(hw_data_, (size + HW_CACHE_LINE_SIZE - 1U) /
                                            HW_CACHE_LINE_SIZE * HW_CACHE_LINE_SIZE);
      regs->DOEPDMA = Address(hw_data_);
    }
    if (iso)
    {
      regs->DOEPCTL |=
          OddFrame(dev) ? USB_OTG_DOEPCTL_SD0PID_SEVNFRM : USB_OTG_DOEPCTL_SODDFRM;
    }
  }
  active_ = true;
  regs->DOEPCTL |= USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA;
  if (num == 0U && controller_.SetupLatched())
  {
    // 装入之后才看到 SETUP：EP0 OUT 不能禁用，写回 NAK；SETUP 的接收不受影响。
    // The SETUP showed up after the load: EP0 OUT cannot be disabled, so the NAK is
    // written back; SETUP reception is not affected.
    regs->DOEPCTL |= USB_OTG_DOEPCTL_SNAK;
    active_ = false;
    return ErrorCode::FAILED;
  }
  return ErrorCode::OK;
}

void STM32OtgEndpoint::FillFifo()
{
  const uint8_t num = Num();
  auto* dev = controller_.Device();
  if (!active_ || stopping_)
  {
    dev->DIEPEMPMSK &= ~(1UL << num);
    return;
  }
  auto* regs = controller_.InEp(num);
  volatile uint32_t& fifo = controller_.Fifo(num);
  // 受空闲 FIFO 空间限制；只写整包。
  // Bounded by the free FIFO space; whole packets only.
  while (hw_done_ < hw_length_)
  {
    const size_t length = LibXR::min(hw_length_ - hw_done_, size_t{MaxPacketSize()});
    const uint32_t words = static_cast<uint32_t>((length + 3U) / 4U);
    if ((regs->DTXFSTS & USB_OTG_DTXFSTS_INEPTFSAV) < words)
    {
      break;
    }
    const uint8_t* source = hw_data_ + hw_done_;
    for (uint32_t i = 0; i < words; ++i)
    {
      uint32_t value = 0U;
      std::memcpy(&value, source + i * 4U, LibXR::min<size_t>(4U, length - i * 4U));
      fifo = value;
    }
    hw_done_ += length;
  }
  if (hw_done_ >= hw_length_)
  {
    dev->DIEPEMPMSK &= ~(1UL << num);
  }
}

void STM32OtgEndpoint::ReadFifo(size_t bytes)
{
  volatile uint32_t& fifo = controller_.Fifo(0);
  // 停止写下之后到达的数据只弹出、不进槽。
  // Data arriving after the stop was issued is popped and never reaches the slot.
  const size_t room =
      active_ && !stopping_ && hw_length_ > hw_done_ ? hw_length_ - hw_done_ : 0U;
  const size_t take = LibXR::min(bytes, room);
  uint8_t* target = hw_data_ + hw_done_;
  size_t offset = 0;
  for (; offset + 4U <= take; offset += 4U)
  {
    const uint32_t value = fifo;
    std::memcpy(target + offset, &value, 4U);
  }
  for (; offset < bytes; offset += 4U)
  {
    const uint32_t value = fifo;
    if (offset < take)
    {
      std::memcpy(target + offset, &value, take - offset);
    }
  }
  hw_done_ += take;
}

size_t STM32OtgEndpoint::OutBytes() const
{
  if (!Dma())
  {
    return hw_done_;
  }
  const size_t remaining = controller_.OutEp(Num())->DOEPTSIZ & USB_OTG_DOEPTSIZ_XFRSIZ;
  const size_t received = hw_programmed_ > remaining ? hw_programmed_ - remaining : 0U;
  const size_t actual = LibXR::min(received, hw_length_);
  if (Num() == 0U)
  {
    STM32_InvalidateDCacheByAddr(controller_.hw_.ep0_dma_buffer.addr_, EP0_DMA_SIZE);
    if (actual != 0U)
    {
      std::memcpy(hw_data_, controller_.hw_.ep0_dma_buffer.addr_, actual);
    }
  }
  else if (actual != 0U)
  {
    STM32_InvalidateDCacheByAddr(hw_data_, actual);
  }
  return actual;
}

size_t STM32OtgEndpoint::InBytesSent() const
{
  const size_t packet = MaxPacketSize();
  const size_t remaining =
      (controller_.InEp(Num())->DIEPTSIZ & USB_OTG_DIEPTSIZ_PKTCNT) >> PKTCNT_POS;
  const size_t total = hw_length_ == 0U ? 1U : (hw_length_ + packet - 1U) / packet;
  // 包离开 FIFO 时 PKTCNT 递减；最后一个包可能还没有主机的 ACK。
  // PKTCNT drops when a packet leaves the FIFO; the last one may lack the host's ACK.
  size_t sent = total > remaining ? total - remaining : 0U;
  if (sent != 0U)
  {
    --sent;
  }
  return LibXR::min(sent * packet, hw_length_);
}

void STM32OtgEndpoint::Complete(bool in_isr)
{
  if (!active_ || stop_failed_)
  {
    return;
  }
  // 请求在停止未结束时自己完成：这次报告同时结束那次停止。端点已不使能，硬件不再碰槽。
  // The request completed by itself while a stop was pending: this report ends that
  // stop too. The endpoint is disabled and the hardware no longer touches the slot.
  const size_t actual =
      IsIn() ? hw_length_ : (stopping_ && !Dma() ? hw_done_ : OutBytes());
  EndPending();
  active_ = false;
  OnHardwareComplete(in_isr, actual, ErrorCode::OK);
}

Endpoint::StopOutcome STM32OtgEndpoint::StopHardware(bool in_isr)
{
  UNUSED(in_isr);
  // 控制器停止或复位之后，硬件上已没有这个请求。
  // After the controller was stopped or reset the request is no longer on the hardware.
  if (!active_)
  {
    return {ErrorCode::ABORTED, 0U};
  }
  // 核心来停的时候，后端自己发起的停止（或已报过 TIMEOUT 的停止）可能还没有结束：从这里
  // 起它算核心的停止。
  // When the core stops the request, a stop the backend issued itself (or one already
  // reported as TIMEOUT) may still be pending: from here on it is the core's stop.
  self_stop_ = false;
  stop_failed_ = false;
  StopOutcome outcome;
  if (IsIn())
  {
    outcome = StopIn();
  }
  else if (Num() == 0U)
  {
    outcome = StopOutEp0();
  }
  else
  {
    outcome = StopOut();
  }
  if (outcome.result != ErrorCode::PENDING)
  {
    EndPending();
    active_ = false;
  }
  return outcome;
}

void STM32OtgEndpoint::BeginPending()
{
  if (stopping_)
  {
    return;
  }
  stopping_ = true;
  stop_sofs_ = 0U;
  if (IsIn())
  {
    if (controller_.in_stops_++ == 0U)
    {
      controller_.Device()->DIEPMSK |= USB_OTG_DIEPMSK_INEPNEM;
    }
  }
  else if (controller_.out_stops_++ == 0U)
  {
    controller_.core_->GINTMSK |= USB_OTG_GINTMSK_GONAKEFFM;
  }
}

void STM32OtgEndpoint::EndPending()
{
  self_stop_ = false;
  stop_failed_ = false;
  if (!stopping_)
  {
    return;
  }
  stopping_ = false;
  stop_sofs_ = 0U;
  if (IsIn())
  {
    if (--controller_.in_stops_ == 0U)
    {
      controller_.Device()->DIEPMSK &= ~USB_OTG_DIEPMSK_INEPNEM;
    }
  }
  else if (--controller_.out_stops_ == 0U)
  {
    controller_.core_->GINTMSK &= ~USB_OTG_GINTMSK_GONAKEFFM;
    controller_.Device()->DCTL |= USB_OTG_DCTL_CGONAK;
  }
}

bool STM32OtgEndpoint::FinishStop(bool in_isr, bool quiescent)
{
  ASSERT(stopping_);
  if (IsIn())
  {
    auto* regs = controller_.InEp(Num());
    const bool enabled = (regs->DIEPCTL & USB_OTG_DIEPCTL_EPENA) != 0U;
    if (enabled && !quiescent && (regs->DIEPINT & USB_OTG_DIEPINT_INEPNE) == 0U)
    {
      return false;
    }
    if (!DisableIn())
    {
      return false;
    }
  }
  else
  {
    const bool enabled =
        (controller_.OutEp(Num())->DOEPCTL & USB_OTG_DOEPCTL_EPENA) != 0U;
    const bool nak_effective =
        (controller_.core_->GINTSTS & USB_OTG_GINTSTS_BOUTNAKEFF) != 0U;
    if (enabled && !nak_effective)
    {
      return false;
    }
    if (!DisableOut())
    {
      return false;
    }
  }
  const StopOutcome outcome = DisabledOutcome();
  // 后端自己发起的停止没有核心在等：作为普通完成报告。
  // A stop the backend issued has no core waiting: it is reported as a plain completion.
  const ErrorCode result =
      (self_stop_ && outcome.result != ErrorCode::OK) ? stop_report_ : outcome.result;
  EndPending();
  active_ = false;
  OnHardwareComplete(in_isr, outcome.actual, result);
  return true;
}

Endpoint::StopOutcome STM32OtgEndpoint::StopIn()
{
  // RM0468 p.2807：SNAK → INEPNE → EPDIS|SNAK → EPDISD → 刷新 TX FIFO。进入时 NAKSTS 已
  // 为 1 的，NAK 早已生效（SETUP 时 core 自己置的，p.2797；总线复位时写的；STM32H723
  // 实测 SETUP 之后 12 900 次都是这样），当场禁用。否则写 SNAK：NAKSTS 写完立刻读作 1，
  // 不能当作已生效，生效由 INEPNE 报告，要等线上的包结束，这里不等。
  // RM0468 p.2807: SNAK → INEPNE → EPDIS|SNAK → EPDISD → TX FIFO flush. With NAKSTS
  // already 1 on entry the NAK has long been in effect (set by the core at a SETUP,
  // p.2797, or written at the bus reset; 12,900 of 12,900 after a SETUP on an
  // STM32H723) and the endpoint is disabled at once. Otherwise SNAK is written: NAKSTS
  // reads 1 right after the write and proves nothing; INEPNE reports the effect, which
  // has to wait for the packet on the wire and is not waited for here.
  const uint8_t num = Num();
  auto* regs = controller_.InEp(num);
  controller_.Device()->DIEPEMPMSK &= ~(1UL << num);
  const uint32_t ctl = regs->DIEPCTL;
  if ((ctl & USB_OTG_DIEPCTL_EPENA) != 0U)
  {
    bool effective = !stopping_ && (ctl & USB_OTG_DIEPCTL_NAKSTS) != 0U;
    if (stopping_)
    {
      effective = (regs->DIEPINT & USB_OTG_DIEPINT_INEPNE) != 0U;
    }
    else if (!effective)
    {
      regs->DIEPINT = USB_OTG_DIEPINT_INEPNE;
      regs->DIEPCTL |= USB_OTG_DIEPCTL_SNAK;
      effective = (regs->DIEPINT & USB_OTG_DIEPINT_INEPNE) != 0U ||
                  (regs->DIEPCTL & USB_OTG_DIEPCTL_EPENA) == 0U;
    }
    if (!effective || !DisableIn())
    {
      BeginPending();
      return {ErrorCode::PENDING, 0U};
    }
  }
  return DisabledOutcome();
}

bool STM32OtgEndpoint::DisableIn()
{
  const uint8_t num = Num();
  auto* regs = controller_.InEp(num);
  auto* core = controller_.core_;
  if ((regs->DIEPCTL & USB_OTG_DIEPCTL_EPENA) != 0U)
  {
    regs->DIEPINT = USB_OTG_DIEPINT_EPDISD;
    regs->DIEPCTL |= USB_OTG_DIEPCTL_EPDIS | USB_OTG_DIEPCTL_SNAK;
    if (!Spin([regs]() { return (regs->DIEPINT & USB_OTG_DIEPINT_EPDISD) != 0U; }))
    {
      return false;
    }
  }
  regs->DIEPINT = USB_OTG_DIEPINT_INEPNE | USB_OTG_DIEPINT_EPDISD;
  // 禁用之后刷新 TX FIFO（p.2807）；请求已自己发完时 FIFO 是空的，刷新无害。
  // The TX FIFO is flushed after the disable (p.2807); when the request was sent whole
  // the FIFO is empty and the flush harmless.
  if (!Spin([core]() { return (core->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL) != 0U; }))
  {
    return false;
  }
  core->GRSTCTL = USB_OTG_GRSTCTL_TXFFLSH | (static_cast<uint32_t>(num) << 6U);
  return Spin([core]() { return (core->GRSTCTL & USB_OTG_GRSTCTL_TXFFLSH) == 0U; });
}

Endpoint::StopOutcome STM32OtgEndpoint::DisabledOutcome()
{
  // 端点已不使能。XFRC 置位表示整个请求已完成；没有 XFRC（停住了，或控制器复位清掉了
  // 事件）时 IN 按 PKTCNT 算主机取走的字节，OUT 报已收到的字节。停止未结束期间 FIFO
  // 模式不再往槽里拷，OUT 的字节数取停止时已拷的。
  // The endpoint is disabled now. XFRC means the whole request completed; without it
  // (stopped, or a controller reset cleared the events) IN counts what the host took by
  // PKTCNT and OUT reports what was received. While a stop is pending FIFO mode copies
  // nothing more into the slot, so OUT takes the bytes copied when the stop was issued.
  if (IsIn())
  {
    auto* regs = controller_.InEp(Num());
    if ((regs->DIEPINT & USB_OTG_DIEPINT_XFRC) != 0U)
    {
      regs->DIEPINT = USB_OTG_DIEPINT_XFRC;
      return {ErrorCode::OK, hw_length_};
    }
    return {ErrorCode::ABORTED, InBytesSent()};
  }
  auto* regs = controller_.OutEp(Num());
  const size_t actual = (stopping_ && !Dma()) ? hw_done_ : OutBytes();
  if ((regs->DOEPINT & USB_OTG_DOEPINT_XFRC) != 0U)
  {
    regs->DOEPINT = USB_OTG_DOEPINT_XFRC;
    return {ErrorCode::OK, actual};
  }
  return {ErrorCode::ABORTED, actual};
}

Endpoint::StopOutcome STM32OtgEndpoint::StopOut()
{
  // RM0468 p.2799-2800：全局 OUT NAK → EPDIS|SNAK → EPDISD → 清全局 OUT NAK。全局 OUT
  // NAK 生效要等线上的包结束（FIFO 模式还要等 RX FIFO 里的状态项被弹出，p.2798），这里
  // 不等：没有生效就返回 PENDING，由 GONAKEFF 中断结束。几个 OUT 端点的停止共用同一个
  // 全局 OUT NAK，最后一个结束时才撤销。
  // RM0468 p.2799-2800: global OUT NAK → EPDIS|SNAK → EPDISD → clear the global OUT NAK.
  // The global OUT NAK takes effect after the packet on the wire (in FIFO mode also after
  // the RX FIFO status entries are popped, p.2798) and is not waited for: without it
  // PENDING is returned and the GONAKEFF interrupt ends the stop. The stops of several
  // OUT endpoints share the one global OUT NAK; the last to end withdraws it.
  auto* regs = controller_.OutEp(Num());
  auto* core = controller_.core_;
  if (!Dma() && !stopping_)
  {
    // 已在 RX FIFO 里的数据属于停止之前，先收下。
    // Data already in the RX FIFO predates the stop and is taken first.
    controller_.DrainRxFifo();
  }
  if ((regs->DOEPCTL & USB_OTG_DOEPCTL_EPENA) != 0U)
  {
    // 别的 OUT 端点的停止已经占着全局 OUT NAK 时共用，不再写 SGONAK。
    // When the stop of another OUT endpoint holds the global OUT NAK already it is
    // shared and SGONAK is not written again.
    if (controller_.out_stops_ == 0U)
    {
      controller_.RequestGlobalOutNak();
    }
    const bool effective = (core->GINTSTS & USB_OTG_GINTSTS_BOUTNAKEFF) != 0U;
    // 停止写下之后本端点不再读写槽：先置 stopping_，禁用握手里弹出的数据只丢弃。
    // Once the stop is issued the endpoint no longer touches the slot: stopping_ is set
    // first, so data popped during the disable handshake is only dropped.
    BeginPending();
    if (!effective || !DisableOut())
    {
      return {ErrorCode::PENDING, 0U};
    }
  }
  return DisabledOutcome();
}

bool STM32OtgEndpoint::DisableOut()
{
  auto* regs = controller_.OutEp(Num());
  if ((regs->DOEPCTL & USB_OTG_DOEPCTL_EPENA) == 0U)
  {
    return true;
  }
  regs->DOEPINT = USB_OTG_DOEPINT_EPDISD;
  regs->DOEPCTL |= USB_OTG_DOEPCTL_EPDIS | USB_OTG_DOEPCTL_SNAK;
  const bool fifo = !Dma();
  STM32OtgController& controller = controller_;
  const bool disabled = Spin(
      [regs, fifo, &controller]()
      {
        if (fifo)
        {
          controller.DrainRxFifo();
        }
        return (regs->DOEPINT & USB_OTG_DOEPINT_EPDISD) != 0U;
      });
  if (disabled)
  {
    regs->DOEPINT = USB_OTG_DOEPINT_EPDISD;
  }
  return disabled;
}

Endpoint::StopOutcome STM32OtgEndpoint::StopOutEp0()
{
  // EP0 OUT 不能禁用（RM0468 p.2736），只写 SNAK。硬件从不写槽：DMA 收进 EP0 缓冲，FIFO
  // 模式由 CPU 拷贝，所以写完 SNAK 就算停稳。保持 EPENA 和 STUPCNT，DMA 靠它们继续收
  // SETUP。
  // EP0 OUT cannot be disabled (RM0468 p.2736); only SNAK is written. The hardware never
  // writes the slot: DMA receives into the EP0 buffer and FIFO mode copies by CPU, so
  // the endpoint is quiescent once SNAK is written. EPENA and STUPCNT stay, DMA needs
  // them to keep receiving SETUPs.
  auto* regs = controller_.OutEp(0);
  regs->DOEPCTL |= USB_OTG_DOEPCTL_SNAK;
  const size_t actual = Dma() ? 0U : hw_done_;
  if ((regs->DOEPINT & USB_OTG_DOEPINT_XFRC) != 0U &&
      (regs->DOEPINT & USB_OTG_DOEPINT_STUP) == 0U)
  {
    regs->DOEPINT = USB_OTG_DOEPINT_XFRC;
    return {ErrorCode::OK, actual};
  }
  return {ErrorCode::ABORTED, actual};
}

ErrorCode STM32OtgEndpoint::StallHardware(bool in_isr)
{
  UNUSED(in_isr);
  const uint8_t num = Num();
  if (IsIn())
  {
    auto* regs = controller_.InEp(num);
    if ((regs->DIEPCTL & USB_OTG_DIEPCTL_EPENA) == 0U && num != 0U)
    {
      regs->DIEPCTL &= ~USB_OTG_DIEPCTL_EPDIS;
    }
    regs->DIEPCTL |= USB_OTG_DIEPCTL_STALL;
    return ErrorCode::OK;
  }
  auto* regs = controller_.OutEp(num);
  if ((regs->DOEPCTL & USB_OTG_DOEPCTL_EPENA) == 0U && num != 0U)
  {
    regs->DOEPCTL &= ~USB_OTG_DOEPCTL_EPDIS;
  }
  regs->DOEPCTL |= USB_OTG_DOEPCTL_STALL;
  // DMA 时只有 EP0 OUT 使能 core 才存 SETUP；被 STALL 的请求没有状态阶段来重新使能它
  // （HAL_PCD_EP_SetStall 也这样做）。
  // With DMA a SETUP is stored only while EP0 OUT is enabled, and a stalled request has
  // no status stage to enable it again (as HAL_PCD_EP_SetStall does).
  if (num == 0U && Dma())
  {
    controller_.ArmSetup();
  }
  return ErrorCode::OK;
}

ErrorCode STM32OtgEndpoint::ClearStallHardware(bool in_isr)
{
  UNUSED(in_isr);
  const uint8_t num = Num();
  // 下一个 SETUP 会清除 EP0 STALL / The next SETUP clears an EP0 STALL.
  if (num == 0U)
  {
    return ErrorCode::OK;
  }
  // 一次写入，EPENA、EPDIS 写 0（0 不改变它们）：读改写会把读到的 EPENA=1 写回，端点在
  // TX FIFO 里已有包时这次写入让 IN 端点再也不发数据（STM32H723 FIFO 模式实测，只有复位
  // 能恢复）。CNAK、SNAK、SD1PID 也写 0，NAK 状态不变。
  // One write with EPENA and EPDIS written as 0 (0 leaves them unchanged): a
  // read-modify-write writes back the EPENA=1 it read, and with a packet already in the
  // TX FIFO that write keeps the IN endpoint from ever sending again (measured on an
  // STM32H723 in FIFO mode, only a reset recovers it). CNAK, SNAK and SD1PID are written
  // as 0 too, so the NAK state stays.
  constexpr uint32_t KEEP =
      ~(USB_OTG_DIEPCTL_EPENA | USB_OTG_DIEPCTL_EPDIS | USB_OTG_DIEPCTL_SODDFRM |
        USB_OTG_DIEPCTL_SD0PID_SEVNFRM | USB_OTG_DIEPCTL_SNAK | USB_OTG_DIEPCTL_CNAK |
        USB_OTG_DIEPCTL_STALL);
  static_assert(USB_OTG_DIEPCTL_EPENA == USB_OTG_DOEPCTL_EPENA &&
                    USB_OTG_DIEPCTL_SNAK == USB_OTG_DOEPCTL_SNAK &&
                    USB_OTG_DIEPCTL_STALL == USB_OTG_DOEPCTL_STALL &&
                    USB_OTG_DIEPCTL_SD0PID_SEVNFRM == USB_OTG_DOEPCTL_SD0PID_SEVNFRM,
                "DIEPCTL and DOEPCTL share the bit layout used here");
  const uint32_t data0 = (GetType() == Type::BULK || GetType() == Type::INTR)
                             ? USB_OTG_DIEPCTL_SD0PID_SEVNFRM
                             : 0U;
  volatile uint32_t& ctl =
      IsIn() ? controller_.InEp(num)->DIEPCTL : controller_.OutEp(num)->DOEPCTL;
  ctl = (ctl & KEEP) | data0;
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
    const std::initializer_list<const std::initializer_list<USB::DeviceClass*>> CONFIGS,
    ConstRawData uid)
    : STM32USBOtgDevice(OTG_FS_INDEX, USB_OTG_FS, OTG_FS_IRQn, OTG_FS_ENDPOINTS,
                        OTG_FS_FIFO, hw, rx_fifo_size, RX_EP_CFGS, TX_EP_CFGS,
                        packet_size, vid, pid, bcd, LANG_LIST, CONFIGS, uid)
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
    const std::initializer_list<const std::initializer_list<USB::DeviceClass*>> CONFIGS,
    ConstRawData uid)
    : STM32USBOtgDevice(OTG_HS_INDEX, USB_OTG_HS, OTG_HS_IRQn, OTG_HS_ENDPOINTS,
                        OTG_HS_FIFO, hw, rx_fifo_size, RX_EP_CFGS, TX_EP_CFGS,
                        packet_size, vid, pid, bcd, LANG_LIST, CONFIGS, uid)
{
}
#endif

// 由包装后的 HAL_PCD_IRQHandler() 进入（stm32_usb_wrap.cpp）。句柄只作提示：main()
// 没运行 MX_USB_OTG_*_PCD_Init() 时其 Instance 为空，改由当前异常号确定控制器。
// Entry from the wrapped HAL_PCD_IRQHandler() (stm32_usb_wrap.cpp). The handle is a
// hint only: when main() never ran MX_USB_OTG_*_PCD_Init() its Instance is null, so the
// active exception names the controller.
extern "C" void libxr_stm32_otg_dispatch(PCD_HandleTypeDef* hpcd)
{
  USB_OTG_GlobalTypeDef* core = hpcd != nullptr ? hpcd->Instance : nullptr;
  if (core == nullptr)
  {
    const int32_t irq = static_cast<int32_t>(__get_IPSR()) - 16;
#if defined(USB_OTG_FS)
    if (irq == OTG_FS_IRQn)
    {
      core = USB_OTG_FS;
    }
#endif
#if defined(USB_OTG_HS)
    if (irq == OTG_HS_IRQn)
    {
      core = USB_OTG_HS;
    }
#endif
    if (core == nullptr)
    {
      return;
    }
  }
  STM32USBOtgDevice::Dispatch(core);
}

#endif
