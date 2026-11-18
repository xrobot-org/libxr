#include "esp_usb.hpp"

#if SOC_USB_OTG_SUPPORTED && defined(CONFIG_IDF_TARGET_ESP32S3) && \
    CONFIG_IDF_TARGET_ESP32S3

#include <atomic>
#include <cstring>

#include "esp32s3/rom/usb/cdc_acm.h"
#include "esp32s3/rom/usb/usb_dc.h"
#include "esp32s3/rom/usb/usb_device.h"
#include "esp_memory_utils.h"
#include "esp_private/usb_phy.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "soc/interrupts.h"
#include "soc/usb_dwc_cfg.h"
#include "soc/usb_dwc_struct.h"

using namespace LibXR;
using LibXR::USB::Endpoint;

namespace
{
// 端点寄存器块（DWC2 设备模式，IN 在 0x900、OUT 在 0xB00，每个端点 0x20）。
// Endpoint register block (DWC2 device mode, IN at 0x900, OUT at 0xB00, 0x20 each).
struct EpRegs
{
  volatile uint32_t ctl;
  uint32_t reserved_04;
  volatile uint32_t intr;
  uint32_t reserved_0c;
  volatile uint32_t tsiz;
  volatile uint32_t dma;
  volatile uint32_t txfsts;
  uint32_t reserved_1c;
};

// DxEPCTL。EPENA 由应用置、core 清；EPDIS、SNAK、CNAK、SD0PID、SD1PID 只写。写控制寄存器
// 一律整字写，写回值里这几位只放本次要的：读出的 EPENA=1 写回去会在 core 刚清掉它之后
// 重新使能端点。
// DxEPCTL. EPENA is set by the application and cleared by the core; EPDIS, SNAK, CNAK,
// SD0PID and SD1PID are write-only. Control registers are always written as a whole
// word holding only the wanted ones of these bits: writing back a read EPENA=1 would
// re-enable an endpoint the core has just disabled.
constexpr uint32_t CTL_MPS = 0x7FFU;
constexpr uint32_t CTL_MPS0 = 0x3U;
constexpr uint32_t CTL_USBACTEP = 1UL << 15U;
constexpr uint32_t CTL_DPID = 1UL << 16U;
constexpr uint32_t CTL_NAKSTS = 1UL << 17U;
constexpr uint32_t CTL_EPTYPE = 3UL << 18U;
constexpr uint32_t CTL_STALL = 1UL << 21U;
constexpr uint32_t CTL_TXFNUM = 0xFUL << 22U;
constexpr uint32_t CTL_CNAK = 1UL << 26U;
constexpr uint32_t CTL_SNAK = 1UL << 27U;
constexpr uint32_t CTL_SD0PID = 1UL << 28U;
constexpr uint32_t CTL_SD1PID = 1UL << 29U;
constexpr uint32_t CTL_EPDIS = 1UL << 30U;
constexpr uint32_t CTL_EPENA = 1UL << 31U;
constexpr uint32_t CTL_KEEP =
    ~(CTL_CNAK | CTL_SNAK | CTL_SD0PID | CTL_SD1PID | CTL_EPDIS | CTL_EPENA);

// DIEPINT / DOEPINT
constexpr uint32_t INT_XFRC = 1UL << 0U;
constexpr uint32_t INT_EPDISD = 1UL << 1U;
constexpr uint32_t INT_TOC = 1UL << 3U;
constexpr uint32_t INT_STUP = 1UL << 3U;
constexpr uint32_t INT_STSPHSRCVD = 1UL << 5U;
constexpr uint32_t INT_INEPNE = 1UL << 6U;
// DOEPINT.StupPktRcvd：内部 DMA 下 EP0 OUT 收到 SETUP 时置位，早于 STUP
// DOEPINT.StupPktRcvd: set when EP0 OUT received a SETUP with the internal DMA, earlier
// than STUP
constexpr uint32_t INT_STPKTRX = 1UL << 15U;
constexpr uint32_t DIEPINT_ALL = 0x7FFFU;
constexpr uint32_t DOEPINT_ALL = 0xFFFFU;

// DxEPTSIZ
constexpr uint32_t TSIZ_PKTCNT_POS = 19U;
constexpr uint32_t TSIZ_XFRSIZ = 0x7FFFFU;
constexpr uint32_t TSIZ0_XFRSIZ = 0x7FU;
constexpr uint32_t TSIZ_MC_ONE = 1UL << 29U;
constexpr uint32_t TSIZ0_SUPCNT_POS = 29U;
// EP0 OUT 能连续收的 SETUP 个数 / Back-to-back SETUP packets EP0 OUT accepts
constexpr uint32_t SETUP_COUNT = 3U;

// GINTSTS / GINTMSK：标志写 1 清，整字写只含目标位。
// GINTSTS / GINTMSK: flags are cleared by writing 1, as a whole word with only the
// wanted bits.
constexpr uint32_t GINT_OTGINT = 1UL << 2U;
constexpr uint32_t GINT_SOF = 1UL << 3U;
constexpr uint32_t GINT_GOUTNAKEFF = 1UL << 7U;
constexpr uint32_t GINT_USBSUSP = 1UL << 11U;
constexpr uint32_t GINT_USBRST = 1UL << 12U;
constexpr uint32_t GINT_ENUMDONE = 1UL << 13U;
constexpr uint32_t GINT_IEPINT = 1UL << 18U;
constexpr uint32_t GINT_OEPINT = 1UL << 19U;
constexpr uint32_t GINT_INCOMPISOIN = 1UL << 20U;
constexpr uint32_t GINT_WKUPINT = 1UL << 31U;
// GOUTNAKEFF 只在有 OUT 端点的停止未结束时加进掩码（它是电平状态）。
// GOUTNAKEFF joins the mask only while the stop of an OUT endpoint is pending (it is a
// level status).
constexpr uint32_t GINT_ENABLED = GINT_OTGINT | GINT_SOF | GINT_USBSUSP | GINT_USBRST |
                                  GINT_ENUMDONE | GINT_IEPINT | GINT_OEPINT |
                                  GINT_INCOMPISOIN | GINT_WKUPINT;

// DCTL：SGOUTNAK、CGOUTNAK 只写，读出为 0，所以读改写只会写入要的那一位。
// DCTL: SGOUTNAK and CGOUTNAK are write-only and read as 0, so a read-modify-write
// writes only the wanted one.
constexpr uint32_t DCTL_RWUSIG = 1UL << 0U;
constexpr uint32_t DCTL_SFTDISCON = 1UL << 1U;
constexpr uint32_t DCTL_SGOUTNAK = 1UL << 9U;
constexpr uint32_t DCTL_CGOUTNAK = 1UL << 10U;

// GRSTCTL
constexpr uint32_t GRST_CSFTRST = 1UL << 0U;
constexpr uint32_t GRST_RXFFLSH = 1UL << 4U;
constexpr uint32_t GRST_TXFFLSH = 1UL << 5U;
constexpr uint32_t GRST_TXFNUM_POS = 6U;
constexpr uint32_t GRST_TXFNUM_ALL = 0x10U;
constexpr uint32_t GRST_AHBIDL = 1UL << 31U;

// DSTS.ENUMSPD：全速（30/60 MHz 或 48 MHz 的 PHY 时钟）/ DSTS.ENUMSPD: full speed
// (30/60 MHz or 48 MHz PHY clock)
constexpr uint32_t ENUM_SPEED_FULL_30_60 = 1U;
constexpr uint32_t ENUM_SPEED_FULL_48 = 3U;
// GAHBCFG.HBSTLEN 的 INCR4 / INCR4 encoding of GAHBCFG.HBSTLEN
constexpr uint32_t DMA_BURST_INCR4 = 3U;

// 一次请求的上限：PKTCNT 7 位，XFERSIZE 16 位，OUT 大小按字对齐（ESP-IDF usb_dwc_cfg.h
// 的 OTG_PACKET_COUNT_WIDTH、OTG_TRANS_COUNT_WIDTH）。
// Limits of one request: 7-bit PKTCNT, 16-bit XFERSIZE, OUT sizes word aligned (ESP-IDF
// usb_dwc_cfg.h, OTG_PACKET_COUNT_WIDTH and OTG_TRANS_COUNT_WIDTH).
constexpr size_t MAX_PACKETS = (1U << OTG_PACKET_COUNT_WIDTH) - 1U;
constexpr size_t MAX_IN_BYTES = (1U << OTG_TRANS_COUNT_WIDTH) - 1U;
constexpr size_t MAX_OUT_BYTES = MAX_IN_BYTES & ~size_t{3U};
// RX FIFO 下限（字）/ Lower bound of the RX FIFO (words)
constexpr uint32_t MIN_RX_FIFO_WORDS = 88U;

usb_dwc_dev_t* Dwc() { return &USB_DWC; }

EpRegs* InEp(uint8_t num)
{
  return reinterpret_cast<EpRegs*>(reinterpret_cast<uintptr_t>(&USB_DWC) + 0x900U +
                                   num * 0x20U);
}

EpRegs* OutEp(uint8_t num)
{
  return reinterpret_cast<EpRegs*>(reinterpret_cast<uintptr_t>(&USB_DWC) + 0xB00U +
                                   num * 0x20U);
}

uint32_t Address(const void* pointer)
{
  return static_cast<uint32_t>(reinterpret_cast<uintptr_t>(pointer));
}

void WriteDctl(uint32_t set, uint32_t clear)
{
  Dwc()->dctl_reg.val = (Dwc()->dctl_reg.val & ~clear) | set;
}

void WriteGintmsk(uint32_t set, uint32_t clear)
{
  Dwc()->gintmsk_reg.val = (Dwc()->gintmsk_reg.val & ~clear) | set;
}

/// DIEPCTL0 的 MPSIZ 编码：64、32、16 和 8 字节 / MPSIZ code of DIEPCTL0
uint32_t Ep0SizeCode(uint32_t packet)
{
  return packet >= 64U ? 0U : packet >= 32U ? 1U : packet >= 16U ? 2U : 3U;
}

uint32_t EndpointType(Endpoint::Type type) { return static_cast<uint32_t>(type) << 18U; }

/// 当前帧号是否为奇数 / Whether the current frame number is odd
bool OddFrame() { return ((Dwc()->dsts_reg.val >> 8U) & 1U) != 0U; }

int64_t NowUs() { return esp_timer_get_time(); }

/**
 * @brief 读寄存器直到 done 成立，最多 HANDSHAKE_SPINS 次 / Read registers until done
 *        holds, HANDSHAKE_SPINS times at most
 * @note 只用于与总线无关的握手。
 *       Bus-independent handshakes only.
 */
template <typename Done>
bool Spin(Done done)
{
  for (uint32_t i = 0; i < ESP32USBEndpoint::HANDSHAKE_SPINS; ++i)
  {
    if (done())
    {
      return true;
    }
  }
  return done();
}

/**
 * @brief 请求全局 OUT NAK / Request the global OUT NAK
 * @note 调用者：第一个需要它的 OUT 停止；已有停止占着时共用，不再调用。GOUTNAKEFF 清零之
 *       后才允许写 SGOUTNAK（同一 IP 的 RM0468 p.2717）：仍置位时是上一次留下的，先撤销一
 *       次并等它清零，只取决于芯片。
 *       Caller: the first OUT stop that needs it; a stop that finds another one holding
 *       it shares it and does not call this. SGOUTNAK may be written only once GOUTNAKEFF
 *       is clear (RM0468 p.2717 for the same IP): a flag still set is left from the
 *       previous global OUT NAK, so the NAK is withdrawn once more and the flag awaited,
 *       which depends on the chip alone.
 */
void RequestGlobalOutNak()
{
  auto* dev = Dwc();
  if ((dev->gintsts_reg.val & GINT_GOUTNAKEFF) != 0U)
  {
    WriteDctl(DCTL_CGOUTNAK, 0U);
    (void)Spin([dev]() { return (dev->gintsts_reg.val & GINT_GOUTNAKEFF) == 0U; });
  }
  WriteDctl(DCTL_SGOUTNAK, 0U);
}

/// 等 condition 成立，最多 bound_us / Wait for condition, at most bound_us
template <typename Condition>
bool WaitUntil(Condition condition, int64_t bound_us)
{
  const int64_t start = NowUs();
  while (!condition())
  {
    if (NowUs() - start > bound_us)
    {
      return condition();
    }
  }
  return true;
}
}  // namespace

// ---------------------------------------------------------------------------
// Controller
// ---------------------------------------------------------------------------

ESP32USBController::ESP32USBController(const std::initializer_list<EPConfig>& ep_cfgs,
                                       size_t packet_size)
{
  REQUIRE(ep_cfgs.size() > 0U && ep_cfgs.size() <= ENDPOINT_COUNT);
  // EP0 的两个 DMA 缓冲在本对象里，对象必须在内部 DMA 可达 RAM 中。
  // The two EP0 DMA buffers live in this object, which must be in internal DMA-capable
  // RAM.
  REQUIRE(esp_ptr_dma_capable(setup_buffer_) &&
          esp_ptr_dma_capable(ep0_out_buffer_ + EP0_BUFFER_SIZE - 1U));

  auto cfg = ep_cfgs.begin();
  // EP0 的 OUT 与 IN 各占缓冲的一半，各自能放下一个 EP0 包。
  // EP0 OUT and IN each take half of the buffer, each half holds one EP0 packet.
  const size_t ep0_half = cfg->buffer.size_ / 2U;
  REQUIRE(ep0_half >= packet_size && ep0_half % HW_CACHE_LINE_SIZE == 0U);
  auto* ep0_out =
      new ESP32USBEndpoint(*this, Endpoint::EPNumber::EP0, Endpoint::Direction::OUT,
                           {cfg->buffer.addr_, ep0_half});
  auto* ep0_in = new ESP32USBEndpoint(
      *this, Endpoint::EPNumber::EP0, Endpoint::Direction::IN,
      {static_cast<uint8_t*>(cfg->buffer.addr_) + ep0_half, ep0_half});
  SetEndpoint0(ep0_in, ep0_out);
  tx_fifo_words_[0] = EP0_FIFO_WORDS;
  uint32_t tx_words = EP0_FIFO_WORDS;

  auto number = Endpoint::EPNumber::EP1;
  for (++cfg; cfg != ep_cfgs.end(); ++cfg, number = Endpoint::NextEPNumber(number))
  {
    using Hint = EPConfig::DirectionHint;
    const uint8_t num = Endpoint::EPNumberToInt8(number);
    const bool has_in = cfg->direction_hint != Hint::OUT_ONLY;
    const bool has_out = cfg->direction_hint != Hint::IN_ONLY;
    RawData out_memory = cfg->buffer;
    RawData in_memory = cfg->buffer;
    if (has_in && has_out)
    {
      const size_t half = cfg->buffer.size_ / 2U;
      REQUIRE(half % HW_CACHE_LINE_SIZE == 0U);
      out_memory = {cfg->buffer.addr_, half};
      in_memory = {static_cast<uint8_t*>(cfg->buffer.addr_) + half, half};
    }
    if (has_in)
    {
      // 本控制器只有 4 个专用 TX FIFO：IN 端点号 1..4（ESP-IDF OTG_NUM_IN_EPS 含 EP0）。
      // This controller has four dedicated TX FIFOs: IN endpoint numbers 1..4 (ESP-IDF
      // OTG_NUM_IN_EPS counts EP0).
      REQUIRE(num <= MAX_IN_NUMBER);
      REQUIRE(cfg->in_fifo_bytes >= 64U && cfg->in_fifo_bytes % 4U == 0U);
      tx_fifo_words_[num] = static_cast<uint32_t>(cfg->in_fifo_bytes / 4U);
      tx_words += tx_fifo_words_[num];
      REQUIRE(Put(new ESP32USBEndpoint(*this, number, Endpoint::Direction::IN,
                                       in_memory)) == ErrorCode::OK);
    }
    if (has_out)
    {
      REQUIRE(Put(new ESP32USBEndpoint(*this, number, Endpoint::Direction::OUT,
                                       out_memory)) == ErrorCode::OK);
    }
  }
  // ROM 留下的 USB 栈先拆掉，再接管 core；内部 PHY 打开后 core 才有时钟。
  // Tear down the ROM's USB stack before taking over the core; the core has a clock
  // only once the internal PHY is open.
  usb_dev_deinit();
  usb_dw_ctrl_deinit();
  uart_acm_dev = nullptr;
  usb_phy_config_t phy_config = {
      .controller = USB_PHY_CTRL_OTG,
      .target = USB_PHY_TARGET_INT,
      .otg_mode = USB_OTG_MODE_DEVICE,
      .otg_speed = USB_PHY_SPEED_UNDEFINED,
      .ext_io_conf = nullptr,
      .otg_io_conf = nullptr,
  };
  usb_phy_handle_t phy = nullptr;
  REQUIRE(usb_new_phy(&phy_config, &phy) == ESP_OK);

  // 可用的 FIFO RAM 取 GHWCFG3.DFIFODEPTH：ESP32-S3 上读出 200 字，而不是 ESP-IDF
  // OTG_DFIFO_DEPTH 的 256。顶上 56 字是 core 的 EP info 区（GDFIFOCFG.EPInfoBaseAddr =
  // 200，内部 DMA 时存各端点的 DMA 地址），FIFO 覆盖它时 IN 数据里出现整包的 0（实测）。
  // The usable FIFO RAM is GHWCFG3.DFIFODEPTH: 200 words read on an ESP32-S3, not the
  // 256 of ESP-IDF's OTG_DFIFO_DEPTH. The top 56 words are the core's EP info area
  // (GDFIFOCFG.EPInfoBaseAddr = 200, holding the endpoints' DMA addresses with the
  // internal DMA); a FIFO covering it put whole packets of 0 into the IN data
  // (measured).
  const uint32_t depth = Dwc()->ghwcfg3_reg.dfifodepth;
  REQUIRE(tx_words + MIN_RX_FIFO_WORDS <= depth);
  rx_fifo_words_ = depth - tx_words;
  // RX FIFO 要容纳 SETUP（13 字）、全局 OUT NAK（1 字）、每个 OUT 端点的完成状态（各 2
  // 字），以及两个包和各自的状态字。
  // The RX FIFO holds the SETUP packets (13 words), the global OUT NAK (1 word), the
  // completion status of each OUT endpoint (2 words each) and two packets with a status
  // word each.
  rx_packet_limit_ = ((rx_fifo_words_ - 14U - 2U * ENDPOINT_COUNT) / 2U - 1U) * 4U;

  REQUIRE(ResetCore());
}

bool ESP32USBController::ResetCore()
{
  auto* dev = Dwc();
  dev->gahbcfg_reg.glbllntrmsk = 0;
  if (!WaitUntil([dev]() { return (dev->grstctl_reg.val & GRST_AHBIDL) != 0U; },
                 CORE_RESET_BOUND_US))
  {
    return false;
  }
  dev->gusbcfg_reg.forcehstmode = 0;
  dev->gusbcfg_reg.forcedevmode = 1;
  dev->gusbcfg_reg.physel = 1;
  dev->gusbcfg_reg.usbtrdtim = 5;
  dev->gusbcfg_reg.toutcal = 7;
  dev->gusbcfg_reg.srpcap = 0;
  dev->gusbcfg_reg.hnpcap = 0;
  dev->grstctl_reg.val = GRST_CSFTRST;
  if (!WaitUntil([dev]() { return (dev->grstctl_reg.val & GRST_CSFTRST) == 0U; },
                 CORE_RESET_BOUND_US) ||
      !WaitUntil([dev]() { return (dev->grstctl_reg.val & GRST_AHBIDL) != 0U; },
                 CORE_RESET_BOUND_US))
  {
    return false;
  }
  WriteDctl(DCTL_SFTDISCON, 0U);
  dev->pcgcctl_reg.stoppclk = 0;
  dev->pcgcctl_reg.gatehclk = 0;
  dev->pcgcctl_reg.pwrclmp = 0;
  dev->pcgcctl_reg.rstpdwnmodule = 0;
  // 没有 VBUS 检测：B 会话有效位强制为 1。
  // No VBUS sensing: the B-session valid bit is forced to 1.
  dev->gotgctl_reg.avalidoven = 0;
  dev->gotgctl_reg.bvalidoven = 1;
  dev->gotgctl_reg.bvalidovval = 1;
  ProgramFifos();
  dev->grstctl_reg.val = GRST_TXFFLSH | (GRST_TXFNUM_ALL << GRST_TXFNUM_POS);
  if (!WaitUntil([dev]() { return (dev->grstctl_reg.val & GRST_TXFFLSH) == 0U; },
                 CORE_RESET_BOUND_US))
  {
    return false;
  }
  dev->grstctl_reg.val = GRST_RXFFLSH;
  if (!WaitUntil([dev]() { return (dev->grstctl_reg.val & GRST_RXFFLSH) == 0U; },
                 CORE_RESET_BOUND_US))
  {
    return false;
  }
  dev->gintsts_reg.val = 0xFFFFFFFFU;
  dev->gotgint_reg.val = 0xFFFFFFFFU;
  dev->gintmsk_reg.val = 0U;
  dev->gahbcfg_reg.hbstlen = DMA_BURST_INCR4;
  dev->gahbcfg_reg.dmaen = 1;
  dev->dcfg_reg.descdma = 0;
  dev->dcfg_reg.nzstsouthshk = 1;
  dev->dcfg_reg.devaddr = 0;
  dev->daintmsk_reg.val = 0U;
  dev->doepmsk_reg.val = 0U;
  dev->diepmsk_reg.val = 0U;
  dev->diepempmsk_reg.val = 0U;
  return true;
}

void ESP32USBController::ProgramFifos()
{
  // FIFO RAM：先 RX，再 TX0 和 IN 端点 1..4 的 TX FIFO（IN 端点 n 用 TX FIFO n）。
  // FIFO RAM: RX first, then TX0 and the TX FIFOs of IN endpoints 1..4 (IN endpoint n
  // uses TX FIFO n).
  auto* dev = Dwc();
  dev->grxfsiz_reg.val = rx_fifo_words_;
  uint32_t offset = rx_fifo_words_;
  dev->gnptxfsiz_reg.val = (tx_fifo_words_[0] << 16U) | offset;
  offset += tx_fifo_words_[0];
  for (uint8_t num = 1U; num <= MAX_IN_NUMBER; ++num)
  {
    const uint32_t words = tx_fifo_words_[num];
    dev->dieptxf_regs[num - 1U].val = words == 0U ? 0U : ((words << 16U) | offset);
    offset += words;
  }
}

void ESP32USBController::Connect()
{
  auto* dev = Dwc();
  dev->gintsts_reg.val = 0xFFFFFFFFU;
  dev->gintmsk_reg.val = GINT_ENABLED;
  dev->gahbcfg_reg.glbllntrmsk = 1;
  WriteDctl(0U, DCTL_SFTDISCON);
}

void ESP32USBController::ArmSetup()
{
  auto* out0 = OutEp(0);
  out0->tsiz = (SETUP_COUNT << TSIZ0_SUPCNT_POS) | (1UL << TSIZ_PKTCNT_POS) |
               (SETUP_COUNT * SETUP_SIZE);
  out0->dma = Address(setup_buffer_);
  out0->ctl = (out0->ctl & CTL_KEEP) | CTL_SNAK | CTL_EPENA;
  if (out_[0] != nullptr)
  {
    out_[0]->ep0_armed_ = ESP32USBEndpoint::Ep0Armed::SETUP;
  }
}

bool ESP32USBController::SetupLatched() const
{
  return setup_pending_ || (OutEp(0)->intr & (INT_STUP | INT_STPKTRX)) != 0U;
}

// ---------------------------------------------------------------------------
// Device
// ---------------------------------------------------------------------------

ESP32USBDevice::ESP32USBDevice(
    const std::initializer_list<EPConfig> ep_cfgs,
    USB::DeviceDescriptor::PacketSize0 packet_size, uint16_t vid, uint16_t pid,
    uint16_t bcd,
    const std::initializer_list<const USB::DescriptorStrings::LanguagePack*> lang_list,
    const std::initializer_list<const std::initializer_list<USB::DeviceClass*>> configs,
    ConstRawData uid)
    : ESP32USBController(ep_cfgs, static_cast<size_t>(packet_size)),
      USB::DeviceCore(*this, *this, USB::USBSpec::USB_2_1, USB::Speed::FULL, packet_size,
                      vid, pid, bcd, lang_list, configs, uid)
{
  for (uint8_t i = 0U; i < ENDPOINT_COUNT; ++i)
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
  // 不带 ESP_INTR_FLAG_IRAM：处理链会进入核心和类代码（在 flash 里），写 flash 期间中断
  // 只是被推迟。不带 ESP_INTR_FLAG_SHARED：屏蔽靠改接中断源，共享时会挡住别的源。
  // Without ESP_INTR_FLAG_IRAM: the handler chain reaches the core and class code (in
  // flash), so while flash is written the interrupt is only deferred. Without
  // ESP_INTR_FLAG_SHARED: masking reroutes the source, which would block other sources
  // of a shared vector.
  REQUIRE(esp_intr_alloc(ETS_USB_INTR_SOURCE, ESP_INTR_FLAG_INTRDISABLED, IsrEntry, this,
                         &intr_handle_) == ESP_OK);
  intr_cpu_ = esp_intr_get_cpu(intr_handle_);
  intr_number_ = static_cast<uint32_t>(esp_intr_get_intno(intr_handle_));
}

void ESP32USBDevice::IsrEntry(void* arg)
{
  static_cast<ESP32USBDevice*>(arg)->OnControllerInterrupt();
}

void ESP32USBDevice::StartController(bool in_isr)
{
  UNUSED(in_isr);
  WaitDetached();
  MaskInterrupt();
  Connect();
  UnmaskInterrupt();
}

bool ESP32USBDevice::HaltController()
{
  WriteDctl(DCTL_SFTDISCON, 0U);
  halted_at_us_ = NowUs();
  halted_ = true;
  reset_enum_pending_ = false;
  // core 复位之后硬件上没有任何请求：未结束的停止作废，不再报告。
  // After the core reset the hardware holds no request: pending stops are void and
  // reported no more.
  for (uint8_t i = 0U; i < ENDPOINT_COUNT; ++i)
  {
    for (ESP32USBEndpoint* ep : {in_[i], out_[i]})
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
  // ResetCore 之后总线保持断开，全局中断和所有中断源都屏蔽。
  // After ResetCore the bus stays detached and the global interrupt and every source
  // are masked.
  return ResetCore();
}

void ESP32USBDevice::StopController(bool in_isr)
{
  UNUSED(in_isr);
  (void)HaltController();
}

void ESP32USBDevice::MaskInterrupt()
{
  // 与 esp_intr_disable 对外设源做的相同（改接到 6 号中断，IDF intr_alloc.c），但不取
  // IDF 的中断分配锁：那把锁在本核关中断。实测改接后约 0.2 µs 挂起位清零
  // （14-hw-esp-mask.md），这段时间内中断可能再进一次，入口可以重复进入。
  // What esp_intr_disable does for a peripheral source (route it to interrupt 6, IDF
  // intr_alloc.c), without the IDF interrupt-allocation lock, which disables interrupts
  // on its core. Measured: the pending bit clears about 0.2 us after the reroute
  // (14-hw-esp-mask.md); the interrupt may enter once more meanwhile, and the entry
  // tolerates that.
  esp_rom_route_intr_matrix(intr_cpu_, ETS_USB_INTR_SOURCE, DISABLED_CPU_INTERRUPT);
}

void ESP32USBDevice::UnmaskInterrupt()
{
  esp_rom_route_intr_matrix(intr_cpu_, ETS_USB_INTR_SOURCE, intr_number_);
}

void ESP32USBDevice::SetAddress(uint8_t address, AddressStage stage)
{
  // core 在状态阶段之前采用新地址（与 TinyUSB 相同）。
  // The core takes the new address before the status stage (as TinyUSB does).
  if (stage == AddressStage::BEFORE_STATUS)
  {
    Dwc()->dcfg_reg.devaddr = address;
  }
}

void ESP32USBDevice::OnControlReady() { ArmSetup(); }

void ESP32USBDevice::ResetController(bool in_isr)
{
  UNUSED(in_isr);
  if (!HaltController())
  {
    // core 复位没有完成：保持断开、全局中断屏蔽，主机看到设备消失。
    // The core reset did not finish: stay detached with the global interrupt masked;
    // the host sees the device disappear.
    return;
  }
  WaitDetached();
  Connect();
}

void ESP32USBDevice::WaitDetached()
{
  // 断开要保持到主机看得见，否则主机不复位总线（实测：只断开几微秒到几十微秒时主机
  // 多半不重新枚举）。核心没有给后端的延时接续，只能在这里等。
  // The detach has to last until the host sees it, or the host does not reset the bus
  // (measured: a detach of a few to a few tens of microseconds is mostly not enumerated
  // again). The core has no deferred continuation for the backend, so the wait happens
  // here.
  if (!halted_)
  {
    return;
  }
  while (NowUs() - halted_at_us_ < RESET_DETACH_US)
  {
  }
  halted_ = false;
}

void ESP32USBDevice::ServiceHardware(bool in_isr)
{
  // 一次快照，不重读；还有标志时电平中断会再进来。顺序：总线复位（当场报告）、枚举
  // 完成、全局 OUT NAK 生效、IN（完成、NAK 生效）、OUT（数据完成、EP0 IN 完成、
  // SETUP）、等时漏帧、SOF（停止的超时计数）、恢复、挂起、OTG。
  // One snapshot, no re-read; the level interrupt comes back for flags left. Order: bus
  // reset (reported at once), enumeration done, global OUT NAK effective, IN
  // (completions, NAK effective), OUT (data completion, EP0 IN completion, SETUP),
  // missed isochronous frame, SOF (timeout count of the stops), resume, suspend, OTG.
  auto* dev = Dwc();
  const uint32_t status = dev->gintsts_reg.val & dev->gintmsk_reg.val;
  if ((status & GINT_USBRST) != 0U)
  {
    OnUsbReset(in_isr);
  }
  if ((status & GINT_ENUMDONE) != 0U)
  {
    dev->gintsts_reg.val = GINT_ENUMDONE;
    reset_enum_pending_ = false;
    const uint32_t speed = dev->dsts_reg.enumspd;
    REQUIRE_FROM_CALLBACK(speed == ENUM_SPEED_FULL_30_60 || speed == ENUM_SPEED_FULL_48,
                          in_isr);
  }
  if ((status & GINT_GOUTNAKEFF) != 0U)
  {
    HandleGlobalOutNak(in_isr);
  }
  if ((status & GINT_IEPINT) != 0U)
  {
    HandleInEndpoints(in_isr);
  }
  if ((status & GINT_OEPINT) != 0U)
  {
    HandleOutEndpoints(in_isr);
  }
  if ((status & GINT_INCOMPISOIN) != 0U)
  {
    HandleIsoIncomplete(in_isr);
  }
  if ((status & GINT_SOF) != 0U)
  {
    dev->gintsts_reg.val = GINT_SOF;
    if (in_stops_ != 0U || out_stops_ != 0U)
    {
      CountStopTimeouts(in_isr);
    }
    OnStartOfFrame(in_isr, static_cast<uint16_t>((dev->dsts_reg.val >> 8U) & 0x7FFU));
  }
  if ((status & GINT_WKUPINT) != 0U)
  {
    dev->gintsts_reg.val = GINT_WKUPINT;
    WriteGintmsk(GINT_SOF, 0U);
    OnResume(in_isr);
  }
  if ((status & GINT_USBSUSP) != 0U)
  {
    dev->gintsts_reg.val = GINT_USBSUSP;
    if (reset_enum_pending_)
    {
      RecoverFalseSuspend();
    }
    else
    {
      WriteGintmsk(0U, GINT_SOF);
      FinishPendingStops(in_isr);
      OnSuspend(in_isr);
    }
  }
  if ((status & GINT_OTGINT) != 0U)
  {
    const uint32_t otg = dev->gotgint_reg.val;
    dev->gotgint_reg.val = otg;
  }
}

void ESP32USBDevice::OnUsbReset(bool in_isr)
{
  // RM0468 p.2793 "Endpoint initialization on USB reset"（同一个 DWC2 IP）。所有端点置
  // NAK、清 STALL；端点事件里 XFRC 和 INEPNE 留给停止判断结果，其余清掉，免得复位前的
  // SETUP 在复位后被当成新的。未结束的停止在报告复位之前结束；在途请求由下面报告的复位
  // 关闭（每个端点自己的停止，含 TX FIFO 刷新，复位期间总线上没有包，当场完成）。
  // RM0468 p.2793 "Endpoint initialization on USB reset" (the same DWC2 IP). Every
  // endpoint gets NAK and loses its STALL; of the endpoint events XFRC and INEPNE stay
  // for the stop's result and the rest is cleared, so that a SETUP from before the reset
  // is not taken for a new one. Pending stops end before the reset is reported;
  // requests in flight are closed by the reset reported below (each endpoint's own
  // stop, TX FIFO flush included; no packet is on the bus during the reset, so it
  // completes at once).
  auto* dev = Dwc();
  dev->gintsts_reg.val = GINT_USBRST;
  reset_enum_pending_ = true;
  WriteDctl(0U, DCTL_RWUSIG);
  for (uint8_t i = 0U; i < ENDPOINT_COUNT; ++i)
  {
    EpRegs* out = OutEp(i);
    out->intr = DOEPINT_ALL & ~INT_XFRC;
    out->ctl = (out->ctl & CTL_KEEP & ~CTL_STALL) | CTL_SNAK;
    if (i <= MAX_IN_NUMBER)
    {
      EpRegs* in = InEp(i);
      in->intr = DIEPINT_ALL & ~(INT_XFRC | INT_INEPNE);
      in->ctl = (in->ctl & CTL_KEEP & ~CTL_STALL) | CTL_SNAK;
    }
  }
  FinishPendingStops(in_isr);
  setup_pending_ = false;
  dev->diepempmsk_reg.val = 0U;
  dev->daintmsk_reg.val = (1UL << 16U) | 1UL;
  dev->doepmsk_reg.val = INT_XFRC | INT_STUP | INT_STSPHSRCVD;
  dev->diepmsk_reg.val = INT_XFRC | INT_TOC | (in_stops_ != 0U ? INT_INEPNE : 0U);
  dev->dcfg_reg.devaddr = 0;
  WriteGintmsk(GINT_SOF, 0U);
  // ESP32-S3 只有全速；速度在 ENUMDONE 里检查。在 USBRST 就报告，数据端点在 ENUMDONE
  // 之前就已关闭。
  // The ESP32-S3 is full speed only; the speed is checked at ENUMDONE. Reporting at
  // USBRST closes the data endpoints before ENUMDONE.
  OnBusReset(in_isr, USB::Speed::FULL);
}

void ESP32USBDevice::RecoverFalseSuspend()
{
  // ESP32-S3 板上实测：总线复位的 SE0 期间 core 会在 ENUMDONE 之前报 USBSUSP，
  // 之后线路一直是 SE0，设备不再回到总线；在复位窗口内软断开 20 µs 再连接可恢复枚举。
  // Measured on an ESP32-S3: during the SE0 of a bus reset the core can raise
  // USBSUSP before ENUMDONE, after which the line stays SE0 and the device does not come
  // back; a soft detach of 20 us inside the reset window restores enumeration.
  reset_enum_pending_ = false;
  WriteDctl(DCTL_SFTDISCON, 0U);
  esp_rom_delay_us(20);
  WriteDctl(0U, DCTL_SFTDISCON);
}

void ESP32USBDevice::HandleInEndpoints(bool in_isr)
{
  auto* dev = Dwc();
  uint32_t bits = dev->daint_reg.val & dev->daintmsk_reg.val & 0xFFFFU;
  while (bits != 0U)
  {
    const auto num = static_cast<uint8_t>(__builtin_ctz(bits));
    bits &= bits - 1U;
    EpRegs* regs = InEp(num);
    const uint32_t raw = regs->intr;
    regs->intr = raw & (INT_XFRC | INT_TOC);
    ESP32USBEndpoint* ep = num < ENDPOINT_COUNT ? in_[num] : nullptr;
    if ((raw & INT_XFRC) != 0U && ep != nullptr)
    {
      ep->Complete(in_isr);
    }
    if (ep != nullptr && ep->stopping_ && !ep->stop_failed_)
    {
      // NAK 生效，或请求已自己结束（上面的 XFRC 已经报告并清掉 stopping_ 时不会到这里）。
      // The NAK took effect, or the request ended by itself (when the XFRC above was
      // reported and cleared stopping_, this is not reached).
      if (((raw & INT_INEPNE) != 0U || (regs->ctl & CTL_EPENA) == 0U) &&
          !ep->FinishStop(in_isr, true))
      {
        // 禁用握手没有在上限内完成：清掉标志，免得掩码开着时反复进中断；SOF 计数到
        // 上限时再查，仍不行就报 TIMEOUT。
        // The disable handshake did not finish within its bound: the flag is cleared so
        // that the open mask does not keep raising the interrupt; the SOF count checks
        // again at its bound and reports TIMEOUT if it still fails.
        regs->intr = INT_INEPNE;
      }
    }
    else if ((raw & INT_INEPNE) != 0U && (dev->diepmsk_reg.val & INT_INEPNE) != 0U)
    {
      // 别的端点在等 INEPNE 时掩码是开着的；这个端点没有停止，清掉以免反复进中断。
      // The mask is open while another endpoint waits for INEPNE; this endpoint has no
      // stop, so the flag is cleared to keep the interrupt from repeating.
      regs->intr = INT_INEPNE;
    }
  }
}

void ESP32USBDevice::HandleOutEndpoints(bool in_isr)
{
  auto* dev = Dwc();
  uint32_t bits = (dev->daint_reg.val & dev->daintmsk_reg.val) >> 16U;
  while (bits != 0U)
  {
    const auto num = static_cast<uint8_t>(__builtin_ctz(bits));
    bits &= bits - 1U;
    EpRegs* regs = OutEp(num);
    const uint32_t events = regs->intr;
    regs->intr = events;
    if (num == 0U)
    {
      // STPKTRX 先到、STUP 要等主机的下一个令牌：这里清掉了 STPKTRX，所以记下“SETUP 已
      // 收到、还没有交付”，交付时清零。
      // STPKTRX comes first and STUP only at the host's next token: STPKTRX was cleared
      // above, so "a SETUP was received and not handed over" is remembered until the
      // hand-over.
      if ((events & INT_STPKTRX) != 0U)
      {
        setup_pending_ = true;
      }
      HandleEp0Out(in_isr, events);
    }
    else if ((events & INT_XFRC) != 0U && num < ENDPOINT_COUNT && out_[num] != nullptr)
    {
      out_[num]->Complete(in_isr);
    }
  }
}

void ESP32USBDevice::HandleGlobalOutNak(bool in_isr)
{
  // 全局 OUT NAK 生效后各 OUT 端点可以禁用（RM0468 p.2799-2800）。这是电平状态：处理一次
  // 就屏蔽，禁用握手没有完成的端点留给 SOF 计数再查。
  // With the global OUT NAK in effect the OUT endpoints may be disabled (RM0468
  // p.2799-2800). The status is a level: it is masked after one handling, and an
  // endpoint whose disable handshake did not finish is left to the SOF count.
  for (uint8_t i = 1U; i < ENDPOINT_COUNT; ++i)
  {
    ESP32USBEndpoint* ep = out_[i];
    if (ep != nullptr && ep->stopping_ && !ep->stop_failed_)
    {
      (void)ep->FinishStop(in_isr, true);
    }
  }
  if (out_stops_ != 0U)
  {
    WriteGintmsk(0U, GINT_GOUTNAKEFF);
  }
}

void ESP32USBDevice::FinishPendingStops(bool in_isr)
{
  if (in_stops_ == 0U && out_stops_ == 0U)
  {
    return;
  }
  for (uint8_t i = 0U; i < ENDPOINT_COUNT; ++i)
  {
    ESP32USBEndpoint* ep = in_[i];
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
  // 全局 OUT NAK 已在停止时请求；总线上没有包，生效只取决于芯片（内部 DMA 下 core 自己
  // 排空 RX FIFO）。
  // The global OUT NAK was requested at the stop; with no packet on the bus its effect
  // depends on the chip alone (with the internal DMA the core drains the RX FIFO
  // itself).
  auto* dev = Dwc();
  const bool effective =
      Spin([dev]() { return (dev->gintsts_reg.val & GINT_GOUTNAKEFF) != 0U; });
  for (uint8_t i = 1U; i < ENDPOINT_COUNT; ++i)
  {
    ESP32USBEndpoint* ep = out_[i];
    if (ep != nullptr && ep->stopping_ && !ep->stop_failed_ &&
        !(effective && ep->FinishStop(in_isr, true)))
    {
      ep->stop_failed_ = true;
      ep->OnHardwareComplete(in_isr, 0U, ErrorCode::TIMEOUT);
    }
  }
}

void ESP32USBDevice::CountStopTimeouts(bool in_isr)
{
  for (uint8_t i = 0U; i < ENDPOINT_COUNT; ++i)
  {
    for (ESP32USBEndpoint* ep : {in_[i], out_[i]})
    {
      if (ep == nullptr || !ep->stopping_ || ep->stop_failed_)
      {
        continue;
      }
      if (ep->stop_sofs_ < ESP32USBEndpoint::STOP_TIMEOUT_SOFS)
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

void ESP32USBDevice::HandleEp0Out(bool in_isr, uint32_t events)
{
  EpRegs* out0 = OutEp(0);
  const bool setup = (events & INT_STUP) != 0U;
  // SETUP 先拷出来：下面报告的完成可能让核心当场 OnControlReady，把 DOEPDMA0 指回开头。
  // core 把 SETUP 写在 DOEPDMA0 前移之前的位置：连续的 SETUP 排在 SETUP 缓冲里，数据
  // 阶段挂着时到来的写在 EP0 OUT 缓冲开头（板上实测）。
  // Copy the SETUP out first: a completion reported below may make the core call
  // OnControlReady at once, which points DOEPDMA0 back at the start. The core writes a
  // SETUP where DOEPDMA0 pointed before it advanced: back-to-back SETUPs line up in the
  // SETUP buffer, one arriving with a data stage armed lands at the start of the EP0
  // OUT buffer (measured on the board).
  USB::SetupPacket packet = {};
  if (setup)
  {
    const uintptr_t start = reinterpret_cast<uintptr_t>(setup_buffer_);
    uintptr_t at = static_cast<uintptr_t>(out0->dma) - SETUP_SIZE;
    if (at < start || at + SETUP_SIZE > start + EP0_BUFFER_SIZE)
    {
      at = (at == reinterpret_cast<uintptr_t>(ep0_out_buffer_)) ? at : start;
    }
    std::atomic_signal_fence(std::memory_order_seq_cst);
    std::memcpy(&packet, reinterpret_cast<const void*>(at), sizeof(packet));
    // 顺序约定：交 SETUP 之前报告 EP0 上硬件已标记的完成。EP0 IN 的事件可能在 IN 的
    // 处理之后才置位，这里再读一次。
    // Ordering contract: report every completion the hardware flagged on EP0 before
    // the SETUP. An EP0 IN event may have been set after the IN handling; read again.
    EpRegs* in0 = InEp(0);
    if ((in0->intr & INT_XFRC) != 0U)
    {
      in0->intr = INT_XFRC;
      if (in_[0] != nullptr)
      {
        in_[0]->Complete(in_isr);
      }
    }
  }
  ESP32USBEndpoint* ep = out_[0];
  // EP0 OUT 的数据或状态完成：装的是数据请求，且 PKTCNT 已减到 0（SETUP 只减 SUPCNT，
  // RM0468 p.2740 的 DOEPTSIZ0 字段说明）。
  // A data or status completion of EP0 OUT: armed for a data request whose PKTCNT
  // reached 0 (a SETUP decrements SUPCNT only, DOEPTSIZ0 in RM0468 p.2740).
  if ((events & INT_XFRC) != 0U && ep != nullptr && ep->active_ &&
      ep->ep0_armed_ == ESP32USBEndpoint::Ep0Armed::DATA &&
      ((out0->tsiz >> TSIZ_PKTCNT_POS) & 1U) == 0U)
  {
    ep->Complete(in_isr);
  }
  if (!setup)
  {
    // 数据包收完或主机进入控制写的状态阶段后，core 关掉了 EP0 OUT；没有请求在途时重新
    // 使能它收 SETUP，保持 NAK。
    // After a data packet, or when the host entered the status stage of a control
    // write, the core disabled EP0 OUT; with no request in flight it is enabled again for
    // SETUPs with the NAK kept.
    if ((events & (INT_XFRC | INT_STSPHSRCVD)) != 0U && (out0->ctl & CTL_EPENA) == 0U &&
        (ep == nullptr || !ep->active_))
    {
      ArmSetup();
    }
    return;
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
  ArmSetup();
  setup_pending_ = false;
  OnSetup(in_isr, packet);
}

void ESP32USBDevice::HandleIsoIncomplete(bool in_isr)
{
  // 至少一个等时 IN 端点在刚结束的帧里没有发出数据。ESP32-S3 在新一帧的 SOF 之后才报，
  // 此时 DSTS 已是新帧号（板上实测），所以结束帧的奇偶与读到的相反。端点仍使能、
  // 奇偶位等于结束帧的就是没发完的那个；停止它，请求以 FAILED 结束（停止前已发完的
  // 报 OK）。停止不等总线：没有当场停稳的按后端自己发起的停止处理，结束时报告。
  // At least one isochronous IN endpoint sent nothing in the frame that ended. The
  // ESP32-S3 raises this after the SOF of the new frame, when DSTS already holds the new
  // number (measured on the board), so the ended frame has the opposite parity.
  // The endpoint still enabled with the ended frame's parity is the one; stop it and
  // end the request with FAILED (OK if it was sent before the stop). The stop does not
  // wait for the bus: one not quiescent at once becomes a stop issued by the backend
  // and is reported when it ends.
  const bool ended_odd = !OddFrame();
  Dwc()->gintsts_reg.val = GINT_INCOMPISOIN;
  for (uint8_t i = 1U; i <= MAX_IN_NUMBER; ++i)
  {
    ESP32USBEndpoint* ep = in_[i];
    if (ep == nullptr || !ep->active_ || ep->GetType() != Endpoint::Type::ISOCHRONOUS)
    {
      continue;
    }
    const uint32_t control = InEp(i)->ctl;
    if ((control & CTL_EPENA) == 0U || ((control & CTL_DPID) != 0U) != ended_odd)
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

// ---------------------------------------------------------------------------
// Endpoint
// ---------------------------------------------------------------------------

ESP32USBEndpoint::ESP32USBEndpoint(ESP32USBController& controller, EPNumber number,
                                   Direction direction, RawData buffer)
    : Endpoint(number, direction, buffer), controller_(controller)
{
  // 槽就是 DMA 的目标（EP0 OUT 除外），必须是内部 DMA 可达 RAM。内部 RAM 不经数据
  // cache，所以不做 cache 同步。
  // The slot is the DMA target (EP0 OUT excepted) and must be internal DMA-capable RAM.
  // Internal RAM bypasses the data cache, so no cache maintenance is done.
  const auto* start = static_cast<const uint8_t*>(buffer.addr_);
  REQUIRE(esp_ptr_dma_capable(start) && esp_ptr_dma_capable(start + buffer.size_ - 1U));
  const uint8_t num = EPNumberToInt8(number);
  REQUIRE(num < ESP32USBController::ENDPOINT_COUNT);
  if (direction == Direction::IN)
  {
    controller.in_[num] = this;
  }
  else
  {
    controller.out_[num] = this;
  }
}

Endpoint::HwLimits ESP32USBEndpoint::HardwareLimits(Direction direction, Type type) const
{
  const size_t room = direction == Direction::IN ? controller_.tx_fifo_words_[Num()] * 4U
                                                 : controller_.rx_packet_limit_;
  HwLimits limits;
  if (type == Type::CONTROL)
  {
    // DIEPTSIZ0：XFERSIZE 7 位、PKTCNT 2 位；DOEPTSIZ0：PKTCNT 1 位（usb_dwc_struct.h）。
    // DIEPTSIZ0: 7-bit XFERSIZE, 2-bit PKTCNT; DOEPTSIZ0: 1-bit PKTCNT
    // (usb_dwc_struct.h).
    limits.max_packet_size = room < 64U ? room : 64U;
    limits.max_packets = 1U;
    limits.max_bytes = limits.max_packet_size;
    // EP0 OUT 收进控制器的 EP0 缓冲，由 CPU 拷进槽；其余端点的槽由 DMA 直接读写。
    // EP0 OUT lands in the controller's EP0 buffer and the CPU copies it into the slot;
    // the DMA reads and writes every other endpoint's slot directly.
    limits.direct_memory = direction == Direction::IN;
    return limits;
  }
  limits.max_packet_size = room;
  if (type == Type::ISOCHRONOUS)
  {
    // 每个请求只安排下一帧的一个包。
    // Each request schedules one packet for the next frame.
    limits.max_packets = 1U;
    limits.max_bytes = room;
    return limits;
  }
  limits.max_packets = MAX_PACKETS;
  limits.max_bytes = direction == Direction::IN ? MAX_IN_BYTES : MAX_OUT_BYTES;
  return limits;
}

void ESP32USBEndpoint::ConfigureHardware(const Config& cfg)
{
  const uint8_t num = Num();
  const uint32_t packet = cfg.max_packet_size;
  auto* dev = Dwc();
  if (IsIn())
  {
    EpRegs* regs = InEp(num);
    if (num == 0U)
    {
      regs->ctl = (regs->ctl & CTL_KEEP & ~CTL_MPS0) | Ep0SizeCode(packet);
    }
    else
    {
      regs->ctl = (regs->ctl & CTL_KEEP & ~(CTL_MPS | CTL_EPTYPE | CTL_TXFNUM)) |
                  (packet & CTL_MPS) | EndpointType(cfg.type) |
                  (static_cast<uint32_t>(num) << 22U) | CTL_USBACTEP | CTL_SD0PID;
    }
    dev->daintmsk_reg.val |= 1UL << num;
    return;
  }
  // EP0 OUT 的包长跟随 EP0 IN（DOEPCTL0.MPS 只读）。
  // EP0 OUT follows the EP0 IN packet size (DOEPCTL0.MPS is read-only).
  if (num != 0U)
  {
    EpRegs* regs = OutEp(num);
    regs->ctl = (regs->ctl & CTL_KEEP & ~(CTL_MPS | CTL_EPTYPE)) | (packet & CTL_MPS) |
                EndpointType(cfg.type) | CTL_USBACTEP | CTL_SD0PID;
  }
  dev->daintmsk_reg.val |= (1UL << num) << 16U;
}

void ESP32USBEndpoint::CloseHardware(bool in_isr)
{
  UNUSED(in_isr);
  const uint8_t num = Num();
  // EP0 保持激活，继续收 SETUP。
  // EP0 stays active and keeps receiving SETUPs.
  if (num == 0U)
  {
    return;
  }
  // 关闭的端点保持 USBACTEP 并置 NAK：清掉 USBACTEP 的端点对令牌不应答，主机记传输错误
  // 并可能停用管道（同一个 IP 在 STM32H723 上、Windows 主机实测）。
  // A closed endpoint keeps USBACTEP and NAKs: one with USBACTEP cleared does not answer
  // tokens, the host counts transaction errors and may halt the pipe (measured with a
  // Windows host on the same IP in an STM32H723).
  auto* dev = Dwc();
  EpRegs* regs = IsIn() ? InEp(num) : OutEp(num);
  ASSERT((regs->ctl & CTL_EPENA) == 0U);
  dev->daintmsk_reg.val &= ~(IsIn() ? (1UL << num) : ((1UL << num) << 16U));
  regs->ctl = (regs->ctl & CTL_KEEP) | CTL_SNAK;
}

ErrorCode ESP32USBEndpoint::StartHardware(const HwRequest& request, bool in_isr)
{
  UNUSED(in_isr);
  ASSERT(!active_);
  const uint8_t num = Num();
  const uint32_t packet = MaxPacketSize();
  const bool iso = GetType() == Type::ISOCHRONOUS;
  const auto length = static_cast<uint32_t>(request.length);
  // 核心按 HardwareLimits 保证长度和包数。
  // The core keeps the length and the packet count within HardwareLimits.
  const uint32_t packets = length == 0U ? 1U : (length + packet - 1U) / packet;
  ASSERT(packets <= MAX_PACKETS && length <= MAX_IN_BYTES);
  hw_data_ = request.data;
  hw_length_ = request.length;
  uint32_t parity = 0U;
  if (iso)
  {
    // 等时端点只在帧号奇偶等于端点奇偶位的帧里传输：安排在下一帧。
    // An isochronous endpoint transfers only in a frame whose parity equals its parity
    // bit: schedule it for the next frame.
    parity = OddFrame() ? CTL_SD0PID : CTL_SD1PID;
  }

  if (IsIn())
  {
    EpRegs* regs = InEp(num);
    hw_programmed_ = length;
    regs->tsiz = (num == 0U ? 0U : TSIZ_MC_ONE) | (packets << TSIZ_PKTCNT_POS) | length;
    regs->dma = Address(hw_data_);
    active_ = true;
    regs->ctl = (regs->ctl & CTL_KEEP) | CTL_CNAK | CTL_EPENA | parity;
    if (num == 0U && controller_.SetupLatched())
    {
      // 装入之后发现硬件已收到还没有交付的 SETUP：上面的 CNAK 撤掉了 core 为它置的
      // NAK，这个应答属于被取代的传输，撤回。
      // A SETUP not handed over yet was found after the load: the CNAK above removed the
      // NAK the core set for it, and this reply belongs to the superseded transfer, so
      // it is retracted.
      regs->ctl = (regs->ctl & CTL_KEEP) | CTL_SNAK;
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

  EpRegs* regs = OutEp(num);
  if (num == 0U)
  {
    // 已收到还没有交付的 SETUP 时不装入：改写 DOEPDMA0 会丢掉它的位置。
    // Nothing is loaded while a SETUP waits to be handed over: rewriting DOEPDMA0 would
    // lose where it was stored.
    if (controller_.SetupLatched())
    {
      return ErrorCode::FAILED;
    }
    // 一次一个包，收进控制器的 EP0 OUT 缓冲；长度 0（状态阶段）也编程一个满包。
    // One packet at a time into the controller's EP0 OUT buffer; length 0 (status
    // stage) also programs a full packet.
    hw_programmed_ = packet;
    regs->tsiz = (1UL << TSIZ_PKTCNT_POS) | packet;
    regs->dma = Address(controller_.ep0_out_buffer_);
    ep0_armed_ = Ep0Armed::DATA;
  }
  else
  {
    // 长度是包长的整数倍（Endpoint::Receive）；OUT 大小按字对齐（RM0468 p.2801）。核心
    // 保证到缓存行的整数倍可写，所以向上取到 4 字节不越界。
    // The length is a packet multiple (Endpoint::Receive); OUT sizes are word aligned
    // (RM0468 p.2801). The core promises writable bytes up to a cache-line multiple, so
    // rounding up to 4 bytes stays inside.
    ASSERT(length != 0U);
    const uint32_t size = (length + 3U) & ~3U;
    hw_programmed_ = size;
    regs->tsiz = (packets << TSIZ_PKTCNT_POS) | size;
    regs->dma = Address(hw_data_);
  }
  active_ = true;
  regs->ctl = (regs->ctl & CTL_KEEP) | CTL_CNAK | CTL_EPENA | parity;
  if (num == 0U && controller_.SetupLatched())
  {
    // 装入之后才看到 SETUP：EP0 OUT 不能禁用，写回 NAK；SETUP 的接收不受影响。
    // The SETUP showed up after the load: EP0 OUT cannot be disabled, so the NAK is
    // written back; SETUP reception is not affected.
    regs->ctl = (regs->ctl & CTL_KEEP) | CTL_SNAK;
    active_ = false;
    return ErrorCode::FAILED;
  }
  return ErrorCode::OK;
}

size_t ESP32USBEndpoint::OutBytes() const
{
  const uint8_t num = Num();
  const uint32_t mask = num == 0U ? TSIZ0_XFRSIZ : TSIZ_XFRSIZ;
  const size_t remaining = OutEp(num)->tsiz & mask;
  const size_t received = hw_programmed_ > remaining ? hw_programmed_ - remaining : 0U;
  const size_t actual = received < hw_length_ ? received : hw_length_;
  if (num == 0U && actual != 0U)
  {
    std::atomic_signal_fence(std::memory_order_seq_cst);
    std::memcpy(hw_data_, controller_.ep0_out_buffer_, actual);
  }
  return actual;
}

size_t ESP32USBEndpoint::InBytesSent() const
{
  const size_t packet = MaxPacketSize();
  const uint32_t mask = Num() == 0U ? 0x3U : 0x7FU;
  const size_t remaining = (InEp(Num())->tsiz >> TSIZ_PKTCNT_POS) & mask;
  const size_t total = hw_length_ == 0U ? 1U : (hw_length_ + packet - 1U) / packet;
  // PKTCNT 在包离开 FIFO 时递减，最后一个包可能还没有主机的 ACK，所以少算一个。
  // PKTCNT drops when a packet leaves the FIFO and the last one may lack the host's
  // ACK, so one packet is not counted.
  size_t sent = total > remaining ? total - remaining : 0U;
  if (sent != 0U)
  {
    --sent;
  }
  const size_t bytes = sent * packet;
  return bytes < hw_length_ ? bytes : hw_length_;
}

void ESP32USBEndpoint::Complete(bool in_isr)
{
  if (!active_ || stop_failed_)
  {
    return;
  }
  // 请求在停止未结束时自己完成：这次报告同时结束那次停止。端点已不使能，硬件不再碰槽。
  // The request completed by itself while a stop was pending: this report ends that
  // stop too. The endpoint is disabled and the hardware no longer touches the slot.
  const size_t actual = IsIn() ? hw_length_ : OutBytes();
  EndPending();
  active_ = false;
  OnHardwareComplete(in_isr, actual, ErrorCode::OK);
}

Endpoint::StopOutcome ESP32USBEndpoint::StopHardware(bool in_isr)
{
  UNUSED(in_isr);
  // 控制器复位之后，硬件上已没有这个请求。
  // After a controller reset the request is no longer on the hardware.
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

void ESP32USBEndpoint::BeginPending()
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
      Dwc()->diepmsk_reg.val |= INT_INEPNE;
    }
  }
  else if (controller_.out_stops_++ == 0U)
  {
    WriteGintmsk(GINT_GOUTNAKEFF, 0U);
  }
}

void ESP32USBEndpoint::EndPending()
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
      Dwc()->diepmsk_reg.val &= ~INT_INEPNE;
    }
  }
  else if (--controller_.out_stops_ == 0U)
  {
    WriteGintmsk(0U, GINT_GOUTNAKEFF);
    WriteDctl(DCTL_CGOUTNAK, 0U);
  }
}

bool ESP32USBEndpoint::FinishStop(bool in_isr, bool quiescent)
{
  ASSERT(stopping_);
  if (IsIn())
  {
    EpRegs* regs = InEp(Num());
    const bool enabled = (regs->ctl & CTL_EPENA) != 0U;
    if (enabled && !quiescent && (regs->intr & INT_INEPNE) == 0U)
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
    const bool enabled = (OutEp(Num())->ctl & CTL_EPENA) != 0U;
    const bool nak_effective = (Dwc()->gintsts_reg.val & GINT_GOUTNAKEFF) != 0U;
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

Endpoint::StopOutcome ESP32USBEndpoint::StopIn()
{
  // RM0468 p.2806-2807：SNAK → INEPNE → EPDIS|SNAK → EPDISD → 刷新本端点的 TX FIFO。
  // 进入时 NAKSTS 已为 1 的，NAK 早已生效（SETUP 时 core 自己置的，p.2797；总线复位时
  // 写的），当场禁用。否则写 SNAK：NAKSTS 写完立刻读作 1，不能当作已生效，生效由 INEPNE
  // 报告，要等线上的包结束（S3 实测最长约 48 µs，一个 64 字节全速包），这里不等。
  // RM0468 p.2806-2807: SNAK → INEPNE → EPDIS|SNAK → EPDISD → flush this endpoint's TX
  // FIFO. With NAKSTS already 1 on entry the NAK has long been in effect (set by the
  // core at a SETUP, p.2797, or written at the bus reset) and the endpoint is disabled
  // at once. Otherwise SNAK is written: NAKSTS reads 1 right after the write and proves
  // nothing; INEPNE reports the effect, which has to wait for the packet on the wire
  // (about 48 µs at most measured on the S3, one 64-byte full-speed packet) and is not
  // waited for here.
  EpRegs* regs = InEp(Num());
  const uint32_t ctl = regs->ctl;
  if ((ctl & CTL_EPENA) != 0U)
  {
    bool effective = !stopping_ && (ctl & CTL_NAKSTS) != 0U;
    if (stopping_)
    {
      effective = (regs->intr & INT_INEPNE) != 0U;
    }
    else if (!effective)
    {
      regs->intr = INT_INEPNE;
      regs->ctl = (regs->ctl & CTL_KEEP) | CTL_SNAK;
      effective = (regs->intr & INT_INEPNE) != 0U || (regs->ctl & CTL_EPENA) == 0U;
    }
    if (!effective || !DisableIn())
    {
      BeginPending();
      return {ErrorCode::PENDING, 0U};
    }
  }
  return DisabledOutcome();
}

bool ESP32USBEndpoint::DisableIn()
{
  const uint8_t num = Num();
  EpRegs* regs = InEp(num);
  auto* dev = Dwc();
  if ((regs->ctl & CTL_EPENA) != 0U)
  {
    regs->intr = INT_EPDISD;
    regs->ctl = (regs->ctl & CTL_KEEP) | CTL_SNAK | CTL_EPDIS;
    if (!Spin(
            [regs]()
            {
              return (regs->intr & INT_EPDISD) != 0U ||
                     ((regs->ctl & CTL_EPENA) == 0U && (regs->intr & INT_XFRC) != 0U);
            }))
    {
      return false;
    }
  }
  regs->intr = INT_INEPNE | INT_EPDISD;
  // 端点已不使能；core 不在读 FIFO 之后刷新本端点的 TX FIFO（RM0468 p.2674）。请求已
  // 自己发完时 FIFO 是空的，刷新无害。
  // The endpoint is disabled; once the core no longer reads the FIFO, flush this
  // endpoint's TX FIFO (RM0468 p.2674). When the request was sent whole the FIFO is
  // empty and the flush harmless.
  if (!Spin([dev]() { return (dev->grstctl_reg.val & GRST_AHBIDL) != 0U; }))
  {
    return false;
  }
  dev->grstctl_reg.val = GRST_TXFFLSH | (static_cast<uint32_t>(num) << GRST_TXFNUM_POS);
  return Spin([dev]() { return (dev->grstctl_reg.val & GRST_TXFFLSH) == 0U; });
}

Endpoint::StopOutcome ESP32USBEndpoint::DisabledOutcome()
{
  // 端点已不使能。XFRC 置位表示整个请求已完成；没有 XFRC（停住了，或控制器复位清掉了
  // 事件）时 IN 按 PKTCNT 算主机取走的字节，OUT 报已收到的字节。
  // The endpoint is disabled now. XFRC means the whole request completed; without it
  // (stopped, or a controller reset cleared the events) IN counts what the host took by
  // PKTCNT and OUT reports what was received.
  if (IsIn())
  {
    EpRegs* regs = InEp(Num());
    if ((regs->intr & INT_XFRC) != 0U)
    {
      regs->intr = INT_XFRC;
      return {ErrorCode::OK, hw_length_};
    }
    return {ErrorCode::ABORTED, InBytesSent()};
  }
  EpRegs* regs = OutEp(Num());
  const size_t actual = OutBytes();
  if ((regs->intr & INT_XFRC) != 0U)
  {
    regs->intr = INT_XFRC;
    return {ErrorCode::OK, actual};
  }
  return {ErrorCode::ABORTED, actual};
}

Endpoint::StopOutcome ESP32USBEndpoint::StopOut()
{
  // RM0468 p.2798-2799：全局 OUT NAK → EPDIS|SNAK → EPDISD → 撤销全局 OUT NAK。内部
  // DMA 下 core 自己排空 RX FIFO，GOUTNAKEFF 不需要软件弹 FIFO 就会置位（S3 上实测
  // 成立）。全局 OUT NAK 生效要等线上的包结束，这里不等：没有生效就返回 PENDING，由
  // GOUTNAKEFF 中断结束。几个 OUT 端点的停止共用同一个全局 OUT NAK，最后一个结束时才
  // 撤销。
  // RM0468 p.2798-2799: global OUT NAK → EPDIS|SNAK → EPDISD → release the global OUT
  // NAK. With the internal DMA the core drains the RX FIFO itself and GOUTNAKEFF sets
  // without software popping the FIFO (measured on the S3). The global OUT NAK takes
  // effect after the packet on the wire, which is not waited for: without it PENDING is
  // returned and the GOUTNAKEFF interrupt ends the stop. The stops of several OUT
  // endpoints share the one global OUT NAK; the last to end withdraws it.
  EpRegs* regs = OutEp(Num());
  if ((regs->ctl & CTL_EPENA) != 0U)
  {
    // 别的 OUT 端点的停止已经占着全局 OUT NAK 时共用，不再写 SGOUTNAK。
    // When the stop of another OUT endpoint holds the global OUT NAK already it is
    // shared and SGOUTNAK is not written again.
    if (controller_.out_stops_ == 0U)
    {
      RequestGlobalOutNak();
    }
    const bool effective = (Dwc()->gintsts_reg.val & GINT_GOUTNAKEFF) != 0U;
    BeginPending();
    if (!effective || !DisableOut())
    {
      return {ErrorCode::PENDING, 0U};
    }
  }
  return DisabledOutcome();
}

bool ESP32USBEndpoint::DisableOut()
{
  EpRegs* regs = OutEp(Num());
  if ((regs->ctl & CTL_EPENA) == 0U)
  {
    return true;
  }
  regs->intr = INT_EPDISD;
  regs->ctl = (regs->ctl & CTL_KEEP) | CTL_SNAK | CTL_EPDIS;
  const bool disabled = Spin(
      [regs]()
      {
        return (regs->intr & INT_EPDISD) != 0U ||
               ((regs->ctl & CTL_EPENA) == 0U && (regs->intr & INT_XFRC) != 0U);
      });
  if (disabled)
  {
    regs->intr = INT_EPDISD;
  }
  return disabled;
}

Endpoint::StopOutcome ESP32USBEndpoint::StopOutEp0()
{
  // EP0 OUT 不能禁用（RM0468 p.2736），只写 SNAK。硬件从不写 EP0 OUT 的槽（收进控制器
  // 的 EP0 OUT 缓冲），所以写完 SNAK 就算停稳。保持 EPENA，DMA 靠它继续收 SETUP。
  // EP0 OUT cannot be disabled (RM0468 p.2736); only SNAK is written. The hardware never
  // writes the EP0 OUT slot (data lands in the controller's EP0 OUT buffer), so the
  // endpoint is quiescent once SNAK is written. EPENA stays, the DMA needs it to keep
  // receiving SETUPs.
  EpRegs* regs = OutEp(0);
  regs->ctl = (regs->ctl & CTL_KEEP) | CTL_SNAK;
  if (ep0_armed_ == Ep0Armed::DATA && (regs->intr & INT_XFRC) != 0U &&
      ((regs->tsiz >> TSIZ_PKTCNT_POS) & 1U) == 0U)
  {
    return {ErrorCode::OK, OutBytes()};
  }
  return {ErrorCode::ABORTED, 0U};
}

ErrorCode ESP32USBEndpoint::StallHardware(bool in_isr)
{
  UNUSED(in_isr);
  EpRegs* regs = IsIn() ? InEp(Num()) : OutEp(Num());
  regs->ctl = (regs->ctl & CTL_KEEP) | CTL_STALL;
  return ErrorCode::OK;
}

ErrorCode ESP32USBEndpoint::ClearStallHardware(bool in_isr)
{
  UNUSED(in_isr);
  const uint8_t num = Num();
  // 下一个 SETUP 会清除 EP0 STALL / The next SETUP clears an EP0 STALL.
  if (num == 0U)
  {
    return ErrorCode::OK;
  }
  const bool data0 = GetType() == Type::BULK || GetType() == Type::INTR;
  EpRegs* regs = IsIn() ? InEp(num) : OutEp(num);
  regs->ctl = (regs->ctl & CTL_KEEP & ~CTL_STALL) | (data0 ? CTL_SD0PID : 0U);
  return ErrorCode::OK;
}

#endif
