/**
 * @file fake_dwc2.hpp
 * @brief DWC2（STM32 OTG_FS/HS、ESP32-S2/S3 USB-OTG）设备控制器的行为模型 / Behaviour
 *        model of a DWC2 device controller (STM32 OTG_FS/HS, ESP32-S2/S3 USB-OTG).
 *
 * 模型按参考手册的设备编程模型实现寄存器、FIFO 和中断，不参照任何驱动的期望：
 * - 接收 FIFO：SETUP 数据、SETUP 阶段结束、OUT 数据、OUT 传输完成和全局 OUT NAK 各是
 *   一个状态字；读 GRXSTSP 弹出状态字，弹出"传输完成"时置 XFRC 并清 EPENA，弹出
 *   "SETUP 阶段结束"时置 STUP，弹出"全局 OUT NAK"时置 GONAKEFF。SETUP 阶段结束在
 *   主机发出 SETUP 之后的第一个 IN/OUT 令牌时产生。
 * - OUT：EPENA=1 且 NAK 清除时收包，每个包 PKTCNT 减一；PKTCNT 归零或收到短包时置
 *   NAK。SETUP 不看 NAK、STALL 和 EPENA，并把 EP0 两个方向置 NAK、清掉 EP0 的 STALL。
 * - IN：数据写进发送 FIFO 时 XFRSIZ 减少，包被主机确认时 PKTCNT 减一，两者归零时置
 *   XFRC 并清 EPENA；SNAK 置 INEPNE；对已使能的端点写 EPDIS 置 EPDISD；TXFFLSH 清空
 *   发送 FIFO，过一会儿自清。
 * - 等时 IN 只在 EONUM 与帧号奇偶相同的帧里发数据；一帧结束时仍未发出的端点置
 *   IISOIXFR。
 * - 总线复位置 USBRST 并清掉非 0 端点的 USBAEP；复位结束置 ENUMDNE。地址比较用
 *   DCFG.DAD，SET_ADDRESS 的状态阶段仍按旧地址应答。
 * - 内部 DMA：核心直接从 DIEPDMA 取数据、往 DOEPDMA 写数据，不产生 RXFLVL；SETUP 写到
 *   DOEPDMA0，EP0 OUT 未使能时留在接收 FIFO 里，使能后再写出。
 * 手册写明"必须"而驱动没有做到的操作记入 violations，测试在每个用例结束时检查它为空。
 * 手册没有写明的行为不是手册的结论：模型里这样的地方都标了"ASSUMPTION"，说明依据和选择
 * 的理由，README 的"模型的假设"一节列出它们和依赖它们的用例。
 *
 * The model implements registers, FIFOs and interrupts from the device programming model
 * of the reference manual, not from what a driver expects:
 * - Receive FIFO: SETUP data, SETUP stage done, OUT data, OUT transfer completed and
 *   global OUT NAK are one status word each. Reading GRXSTSP pops a status word; popping
 *   "transfer completed" sets XFRC and clears EPENA, popping "SETUP stage done" sets
 *   STUP, popping "global OUT NAK" sets GONAKEFF. SETUP stage done is produced by the
 *   first IN or OUT token the host sends after a SETUP.
 * - OUT: packets are taken while EPENA is set and NAK is clear, each one decrements
 *   PKTCNT; NAK is set when PKTCNT reaches zero or a short packet arrives. SETUP ignores
 *   NAK, STALL and EPENA, sets NAK on both directions of EP0 and clears the EP0 STALL.
 * - IN: XFRSIZ drops as data enters the transmit FIFO, PKTCNT drops when the host
 *   acknowledges a packet, XFRC is set and EPENA cleared when both are zero; SNAK sets
 *   INEPNE; EPDIS on an enabled endpoint sets EPDISD; TXFFLSH empties the transmit FIFO
 *   and clears itself a moment later.
 * - Isochronous IN sends data only in a frame whose parity equals EONUM; an endpoint
 *   that has not sent by the end of that frame raises IISOIXFR. The interrupt belongs to
 *   the frame that ended: until the driver clears it, DSTS still shows that frame, and
 *   the next frame number appears with the SOF the same host step latches.
 * - A bus reset sets USBRST and clears USBAEP of the non-zero endpoints; the end of the
 *   reset sets ENUMDNE. Addresses are compared with DCFG.DAD, and the status stage of
 *   SET_ADDRESS is still answered at the old address.
 * - Internal DMA: the core fetches from DIEPDMA and stores to DOEPDMA itself and raises
 *   no RXFLVL; a SETUP is stored at DOEPDMA0, and stays in the receive FIFO while EP0 OUT
 *   is disabled until it is enabled.
 * Operations the manual marks as mandatory and a driver does not follow are recorded in
 * violations; the tests check that it is empty at the end of every case.
 * Behaviour the manual does not state is not a conclusion of the manual: every such place
 * in the model is marked "ASSUMPTION" with what it rests on and why it was chosen, and
 * the "Model assumptions" section of README.md lists them with the cases that depend on
 * them.
 */

#pragma once

#include <cstdarg>
#include <cstring>
#include <deque>
#include <functional>
#include <string>
#include <vector>

#include "mmio_trap.hpp"

namespace Dwc2
{

// Register offsets.
inline constexpr uint32_t GOTGCTL = 0x000, GOTGINT = 0x004, GAHBCFG = 0x008;
inline constexpr uint32_t GUSBCFG = 0x00C, GRSTCTL = 0x010, GINTSTS = 0x014;
inline constexpr uint32_t GINTMSK = 0x018, GRXSTSR = 0x01C, GRXSTSP = 0x020;
inline constexpr uint32_t GRXFSIZ = 0x024, DIEPTXF0 = 0x028, GCCFG = 0x038;
inline constexpr uint32_t GSNPSID = 0x040, GHWCFG1 = 0x044, DIEPTXF1 = 0x104;
inline constexpr uint32_t DCFG = 0x800, DCTL = 0x804, DSTS = 0x808, DIEPMSK = 0x810;
inline constexpr uint32_t DOEPMSK = 0x814, DAINT = 0x818, DAINTMSK = 0x81C;
inline constexpr uint32_t DIEPEMPMSK = 0x834, DIEP = 0x900, DOEP = 0xB00;
inline constexpr uint32_t PCGCCTL = 0xE00, FIFO = 0x1000, FIFO_STRIDE = 0x1000;
inline constexpr uint32_t EP_CTL = 0x00, EP_INT = 0x08, EP_TSIZ = 0x10, EP_DMA = 0x14;
inline constexpr uint32_t EP_TXFSTS = 0x18;

// GINTSTS / GINTMSK.
inline constexpr uint32_t GINT_MMIS = 1U << 1, GINT_OTG = 1U << 2, GINT_SOF = 1U << 3;
inline constexpr uint32_t GINT_RXFLVL = 1U << 4, GINT_NPTXFE = 1U << 5;
inline constexpr uint32_t GINT_GINAKEFF = 1U << 6, GINT_GONAKEFF = 1U << 7;
inline constexpr uint32_t GINT_ESUSP = 1U << 10, GINT_USBSUSP = 1U << 11;
inline constexpr uint32_t GINT_USBRST = 1U << 12, GINT_ENUMDNE = 1U << 13;
inline constexpr uint32_t GINT_EOPF = 1U << 15, GINT_IEPINT = 1U << 18;
inline constexpr uint32_t GINT_OEPINT = 1U << 19, GINT_IISOIXFR = 1U << 20;
inline constexpr uint32_t GINT_INCOMPISOOUT = 1U << 21, GINT_PTXFE = 1U << 26;
inline constexpr uint32_t GINT_SRQ = 1U << 30, GINT_WKUP = 1U << 31;
inline constexpr uint32_t GINT_W1C = 0xF8F0FC0AU;

// Endpoint control.
inline constexpr uint32_t CTL_MPSIZ = 0x7FFU, CTL_USBAEP = 1U << 15;
inline constexpr uint32_t CTL_DPID = 1U << 16, CTL_NAKSTS = 1U << 17;
inline constexpr uint32_t CTL_EPTYP = 3U << 18, CTL_SNPM = 1U << 20;
inline constexpr uint32_t CTL_STALL = 1U << 21, CTL_TXFNUM = 0xFU << 22;
inline constexpr uint32_t CTL_CNAK = 1U << 26, CTL_SNAK = 1U << 27;
inline constexpr uint32_t CTL_SD0PID = 1U << 28, CTL_SD1PID = 1U << 29;
inline constexpr uint32_t CTL_EPDIS = 1U << 30, CTL_EPENA = 1U << 31;

// Endpoint interrupts.
inline constexpr uint32_t INT_XFRC = 1U << 0, INT_EPDISD = 1U << 1, INT_STUP = 1U << 3;
inline constexpr uint32_t INT_TOC = 1U << 3, INT_ITTXFE = 1U << 4;
inline constexpr uint32_t INT_OTEPDIS = 1U << 4, INT_STSPHSRX = 1U << 5;
inline constexpr uint32_t INT_INEPNE = 1U << 6, INT_B2BSTUP = 1U << 6;
inline constexpr uint32_t INT_TXFE = 1U << 7, INT_NAK = 1U << 13;
inline constexpr uint32_t INT_STPKTRX = 1U << 15;

// Device control, status and reset.
inline constexpr uint32_t DCTL_RWUSIG = 1U << 0, DCTL_SDIS = 1U << 1;
inline constexpr uint32_t DCTL_GINSTS = 1U << 2, DCTL_GONSTS = 1U << 3;
inline constexpr uint32_t DCTL_SGINAK = 1U << 7, DCTL_CGINAK = 1U << 8;
inline constexpr uint32_t DCTL_SGONAK = 1U << 9, DCTL_CGONAK = 1U << 10;
inline constexpr uint32_t RST_CSRST = 1U << 0, RST_RXFFLSH = 1U << 4;
inline constexpr uint32_t RST_TXFFLSH = 1U << 5, RST_AHBIDL = 1U << 31;
inline constexpr uint32_t AHB_GINT = 1U << 0, AHB_DMAEN = 1U << 5;
inline constexpr uint32_t AHB_TXFELVL = 1U << 7;
inline constexpr uint32_t OTG_SEDET = 1U << 2, OTGCTL_BVALOEN = 1U << 6;

// Receive status packet codes.
inline constexpr uint32_t PKT_GONAK = 1, PKT_OUT_DATA = 2, PKT_OUT_DONE = 3;
inline constexpr uint32_t PKT_SETUP_DONE = 4, PKT_SETUP_DATA = 6;

inline constexpr uint32_t SNPSID_281A = 0x4F54281AU;
inline constexpr uint32_t SNPSID_300A = 0x4F54300AU;
inline constexpr uint32_t SNPSID_310A = 0x4F54310AU;
inline constexpr uint32_t SNPSID_330A = 0x4F54330AU;
inline constexpr uint32_t SNPSID_400A = 0x4F54400AU;

enum class Type : uint8_t
{
  CONTROL = 0,
  ISOCHRONOUS = 1,
  BULK = 2,
  INTERRUPT = 3
};

/// 主机看到的握手 / Handshake as the host sees it
enum class Resp : uint8_t
{
  ACK,
  NAK,
  STALL,
  TIMEOUT,  ///< 设备不应答 / The device does not answer
  DATA      ///< IN：设备发了一个数据包 / IN: the device sent a data packet
};

struct InResult
{
  Resp resp = Resp::TIMEOUT;
  uint8_t pid = 0;  ///< 0 = DATA0, 1 = DATA1
  std::vector<uint8_t> data;
};

struct Config
{
  uintptr_t base = 0;
  uint32_t snpsid = SNPSID_330A;
  uint8_t endpoints = 9;       ///< 端点号 0..endpoints-1 / Endpoint numbers
  uint16_t fifo_depth = 1024;  ///< FIFO RAM，单位字 / FIFO RAM in words
  bool hs_phy = false;         ///< 接了高速 PHY / A high-speed PHY is attached
  uint32_t ghwcfg[4] = {0, 0, 0, 0};
  /// DMA 能到达的内存；为空时不允许 DMA / Memory the DMA reaches; none means no DMA
  const MmioTrap::LowArena* dma_memory = nullptr;
};

class Controller : public MmioTrap::Device
{
 public:
  explicit Controller(const Config& config) : config_(config)
  {
    PowerOnReset();
    MmioTrap::Map(config.base, FIFO + 16U * FIFO_STRIDE, this);
  }
  ~Controller() override { MmioTrap::Unmap(config_.base, this); }
  Controller(const Controller&) = delete;
  Controller& operator=(const Controller&) = delete;

  // -------------------------------------------------------------------------
  // Test controls
  // -------------------------------------------------------------------------

  /// 上电复位：寄存器回到复位值，主机侧状态清空 / Power-on reset: registers at their
  /// reset values, host side state cleared
  void PowerOnReset()
  {
    CoreReset(false);
    vbus_ = true;
    suspended_ = false;
    frame_ = 0;
    violations.clear();
    notes.clear();
    tolerated.clear();
    before_read = nullptr;
    before_write = nullptr;
    after_write = nullptr;
    log = std::getenv("DWC2_LOG") != nullptr ? stderr : nullptr;
  }

  /// TXFFLSH/RXFFLSH/CSRST 写 1 之后，紧接着的多少次 GRSTCTL 读仍然读到 1；总线上再发生
  /// 任何事时也已经清零（手册：8 个时钟）/ How many immediately following reads of
  /// GRSTCTL still see TXFFLSH/RXFFLSH/CSRST set; any later bus event finds them clear
  /// (the manual gives eight clocks)
  unsigned self_clear_reads = 1;

  /// 发送 FIFO 的内容要到 TXFFLSH 自清时才被清掉（之前主机仍能取走，驱动仍能读到
  /// DTXFSTS）；这个开关为真时 AHBIDL 在冲刷期间读到 0，TXFFLSH 清零之后的前两次
  /// GRSTCTL 读取仍读到 0（总线事件不会提前结束它）。
  /// ASSUMPTION：手册要求应用在写 TXFFLSH 之前检查 AHBIDL（RM0468 GRSTCTL 的 TXFFLSH），
  /// 没有给出 AHBIDL 为 0 持续多久；这样选是为了让不检查 AHBIDL 的驱动被发现。
  /// The content of the transmit FIFO is only dropped when TXFFLSH clears itself (until
  /// then the host can still take it and the driver still reads DTXFSTS); with this set
  /// AHBIDL reads 0 while the flush runs, and for the first two GRSTCTL reads after
  /// TXFFLSH has cleared (a bus event does not end it early). ASSUMPTION: the manual has
  /// the application check AHBIDL before it writes TXFFLSH (RM0468 GRSTCTL, TXFFLSH) and
  /// gives no duration of AHBIDL = 0; chosen so that a driver that skips the AHBIDL check
  /// is caught.
  bool ahbidl_follows_flush = false;

  /// EPDIS 写入之后，再过多少次寄存器访问端点才真正禁用并置 EPDISD（0 表示立即）；
  /// 总线上再发生任何事时也已经生效。手册只要求等待 EPDISD 中断，没有给出时间。
  /// How many further register accesses pass before a written EPDIS disables the
  /// endpoint and sets EPDISD (0 = at once); any later bus event finds it done. The
  /// manual only asks to wait for the EPDISD interrupt and gives no time.
  unsigned disable_latency = 0;

  /// DMA 模式下 SGONAK 写入之后，再过多少次寄存器访问全局 OUT NAK 才生效（0 表示立即）；
  /// 总线上再发生任何事时也已经生效。非 DMA 模式下它在应用弹出接收 FIFO 里的状态字时
  /// 生效。手册只要求等待 GONAKEFF，没有给出时间。
  /// In DMA mode, how many further register accesses pass before a written SGONAK takes
  /// effect (0 = at once); any later bus event finds it done. Without DMA it takes effect
  /// when the application pops its status word from the receive FIFO. The manual only
  /// asks to wait for GONAKEFF and gives no time.
  unsigned gonak_latency = 0;

  /// ASSUMPTION：手册只写了缓冲 DMA 模式下 STPKTRX 在 SETUP 阶段结束（第一个 IN/OUT 令牌
  /// 之后的 SETUP_COMPLETE 状态）时置位；3.00a 之后的核心在收到 SETUP 包时就置
  /// XFRC|STPKTRX 手册里没有。选它是因为后端按核心版本分了这条分支（stm32_usb_otg.cpp 的
  /// STPKTRX 判断）， 两种时序都要跑到。 ASSUMPTION: the manual only has STPKTRX set at
  /// the end of the SETUP stage in buffer DMA mode (the SETUP_COMPLETE status after the
  /// first IN or OUT token). Cores after 3.00a raising XFRC|STPKTRX when the SETUP packet
  /// arrives is not in the manual. It is modelled because the backend branches on the
  /// core revision here (the STPKTRX test in stm32_usb_otg.cpp) and both timings have to
  /// be run.
  bool setup_packet_event_early = false;
  /// ASSUMPTION：RM0385 写明 STPKTRX 只用于缓冲 DMA 模式；3.10a 核心在非 DMA 模式弹出
  /// SETUP 数据状态字时置 XFRC|STPKTRX 手册里没有。选它的理由同上（后端对 3.10a 的非 DMA
  /// 模式有单独的判断）。
  /// ASSUMPTION: the manual (RM0385) says STPKTRX is for buffer DMA mode only; a 3.10a
  /// core raising XFRC|STPKTRX when the SETUP data status word is popped without DMA is
  /// not in it. Chosen for the same reason (the backend tests for core 3.10a without
  /// DMA).
  bool slave_setup_event = false;
  /// STM32 OTG_HS 内嵌全速 PHY 上测得：每个 SOF 令牌 GINTSTS.SOF 被置位两次，间隔约
  /// 1.4 us，两次读到的 DSTS.FNSOF 相同。开启后，驱动清除某个 SOF 之后，模型对同一令牌
  /// 再置位一次 / Measured on an STM32 OTG_HS with the embedded full-speed PHY: the core
  /// sets GINTSTS.SOF twice per SOF token, about 1.4 us apart, with the same DSTS.FNSOF.
  /// When on, the model sets the flag once more for the same token after the driver has
  /// cleared the first one
  bool sof_twice = false;

  std::vector<std::string> violations;  ///< 违反手册"必须"的操作 / Mandatory rules broken
  std::vector<std::string> notes;       ///< 值得记录但不判错 / Recorded, not judged
  /// 含有这些文字的违反只记进 notes：已经作为"发现"单独列出的偏差 / Violations with
  /// one of these texts only go to notes: deviations already listed as a finding
  std::vector<std::string> tolerated;
  /// 读寄存器（包括读改写指令的读）之前调用，不自动清掉 / Called before a register
  /// is read (the read of a read-modify-write included); stays installed
  std::function<void(uint32_t offset)> before_read;
  /// 写生效之前/之后调用一次（一次性钩子自己清掉）/ Called before / after a write lands
  std::function<void(uint32_t offset, uint32_t value)> before_write;
  std::function<void(uint32_t offset, uint32_t value)> after_write;
  /// 非空时把每次寄存器写和总线事件写到这里 / When set, every register write and bus
  /// event is written here
  FILE* log = nullptr;

  uint32_t Peek(uint32_t offset) { return Value(offset); }
  /// 寄存器现状，诊断用 / The register state, for diagnosis
  void Dump(FILE* file)
  {
    std::fprintf(file, "GINTSTS %08X GINTMSK %08X GAHBCFG %08X GRSTCTL %08X DCTL %08X\n",
                 Gintsts(), gintmsk_, gahbcfg_, Value(GRSTCTL), Value(DCTL));
    std::fprintf(file, "DAINT %08X DAINTMSK %08X DIEPMSK %08X DOEPMSK %08X rx %zu\n",
                 Daint(), daintmsk_, diepmsk_, doepmsk_, rx_.size());
    for (uint8_t i = 0; i < config_.endpoints; ++i)
    {
      if (i != 0U && ((in_[i].ctl | out_[i].ctl) & (CTL_USBAEP | CTL_EPENA)) == 0U)
        continue;
      std::fprintf(file, "EP%u IN ctl %08X int %08X tsiz %08X fifo %u | ", i, in_[i].ctl,
                   InInt(in_[i]), in_[i].tsiz, TxUsed(in_[i]));
      std::fprintf(file, "OUT ctl %08X int %08X tsiz %08X\n", out_[i].ctl, out_[i].intr,
                   out_[i].tsiz);
    }
  }
  bool IrqLine() { return (gahbcfg_ & AHB_GINT) != 0U && (Gintsts() & gintmsk_) != 0U; }
  bool SofMasked() const { return (gintmsk_ & GINT_SOF) == 0U; }
  bool Connected() const { return vbus_ && (dctl_ & DCTL_SDIS) == 0U; }
  bool DmaEnabled() const { return (gahbcfg_ & AHB_DMAEN) != 0U; }
  uint16_t Frame() const { return frame_; }
  /// 驱动读 DSTS 看到的帧号。
  /// ASSUMPTION：手册写 IISOIXFR 与 EOPF 一起置位（周期帧间隔到达，下一个 SOF 之前），
  /// DSTS.FNSOF 是"收到的 SOF 的帧号"，但没有写 FNSOF 何时更新。模型认为中断在下一个 SOF
  /// 之前被处理：IISOIXFR 挂起期间 FNSOF 仍是刚结束的那一帧，驱动清掉标志之后才是新帧。
  /// 选它是因为驱动靠这个奇偶判断哪个等时端点错过了帧；中断处理慢于一帧时读到的是新帧。
  /// The frame number the driver reads from DSTS.
  /// ASSUMPTION: the manual has IISOIXFR raised together with EOPF (the periodic frame
  /// interval is reached, before the next SOF) and DSTS.FNSOF is "the frame number of the
  /// received SOF", but it does not say when FNSOF updates. The model takes the interrupt
  /// to be serviced before the next SOF: while IISOIXFR is pending FNSOF still shows the
  /// frame that just ended, and the new frame appears once the driver clears the flag.
  /// Chosen because the driver uses this parity to find the isochronous endpoint that
  /// missed its frame; a handler slower than a frame reads the new frame.
  uint16_t VisibleFrame() const
  {
    if ((latched_ & GINT_IISOIXFR) == 0U) return frame_;
    return static_cast<uint16_t>((frame_ - 1U) & (HighSpeed() ? 0x3FFFU : 0x7FFU));
  }
  bool HighSpeed() const { return enumspd_ == 0U; }
  uint8_t Address() const { return static_cast<uint8_t>((dcfg_ >> 4U) & 0x7FU); }
  size_t RxFifoWords() const { return rx_.size(); }
  size_t TxFifoWords(uint8_t num) const { return TxUsed(in_[num]); }
  uint32_t InCtl(uint8_t num) const { return in_[num].ctl; }
  uint32_t OutCtl(uint8_t num) const { return out_[num].ctl; }

  // -------------------------------------------------------------------------
  // Host side: one call is one event on the bus
  // -------------------------------------------------------------------------

  void HostVbus(bool on)
  {
    Tick("VBUS", on ? 1U : 0U);
    if (vbus_ == on) return;
    vbus_ = on;
    if (!VbusSensed()) return;
    if (on)
      latched_ |= GINT_SRQ;
    else
      gotgint_ |= OTG_SEDET;
  }

  /// 复位信号开始 / Start of the reset signalling
  void HostResetBegin()
  {
    Tick("reset begins");
    if (!Connected()) return;
    suspended_ = false;
    in_reset_ = true;
    latched_ |= GINT_USBRST;
    for (uint8_t i = 1; i < config_.endpoints; ++i)
    {
      in_[i].ctl &= ~CTL_USBAEP;
      out_[i].ctl &= ~CTL_USBAEP;
    }
    set_address_stage_ = false;
    setup_done_due_ = false;
    control_write_data_ = false;
    reset_steps_due_ = true;
    for (auto& done : snak_since_reset_) done = false;
  }

  /// 复位信号结束，速度确定 / End of the reset signalling, the speed is known
  void HostResetEnd()
  {
    Tick("reset ends");
    if (!Connected() || !in_reset_) return;
    in_reset_ = false;
    const uint32_t dspd = dcfg_ & 3U;
    enumspd_ = (config_.hs_phy && dspd == 0U) ? 0U : (dspd == 1U ? 1U : 3U);
    latched_ |= GINT_ENUMDNE;
  }

  void HostSuspend()
  {
    Tick("suspend");
    if (!Connected()) return;
    suspended_ = true;
    latched_ |= GINT_ESUSP | GINT_USBSUSP;
  }

  void HostResume()
  {
    Tick("resume");
    if (!Connected() || !suspended_) return;
    suspended_ = false;
    latched_ |= GINT_WKUP;
  }

  /// 一帧（高速时一个微帧）结束，下一个 SOF 到达 / One frame (microframe at high speed)
  /// ends and the next SOF arrives
  void HostSof()
  {
    Tick("SOF");
    if (!Connected() || suspended_ || in_reset_) return;
    const uint32_t parity = frame_ & 1U;
    latched_ |= GINT_EOPF;
    for (uint8_t i = 1; i < config_.endpoints; ++i)
    {
      const InEp& in = in_[i];
      if (TypeOf(in.ctl) == Type::ISOCHRONOUS && (in.ctl & CTL_EPENA) != 0U &&
          ((in.ctl & CTL_DPID) != 0U) == (parity != 0U))
        latched_ |= GINT_IISOIXFR;
      const OutEp& out = out_[i];
      if (TypeOf(out.ctl) == Type::ISOCHRONOUS && (out.ctl & CTL_EPENA) != 0U &&
          ((out.ctl & CTL_DPID) != 0U) == (parity != 0U))
        latched_ |= GINT_INCOMPISOOUT;
    }
    frame_ = static_cast<uint16_t>((frame_ + 1U) & (HighSpeed() ? 0x3FFFU : 0x7FFU));
    latched_ |= GINT_SOF;
    sof_again_ = sof_twice;
  }

  /// 把下一个 SOF 将带的帧号安排为 next（复位后主机重新计帧）/ Make next the frame
  /// number the next SOF carries (the host counts again after a reset)
  void HostSetNextFrame(uint16_t next)
  {
    frame_ = static_cast<uint16_t>((next - 1U) & (HighSpeed() ? 0x3FFFU : 0x7FFU));
  }

  Resp HostSetup(uint8_t address, const uint8_t* packet)
  {
    Tick("SETUP", packet[1]);
    if (!Listening(address)) return Resp::TIMEOUT;
    CheckResetSteps();
    OutEp& out = out_[0];
    InEp& in = in_[0];
    const uint32_t count = (out.tsiz >> 29U) & 3U;
    if (count == 0U) out.intr |= INT_B2BSTUP;
    out.tsiz = (out.tsiz & ~(3U << 29U)) | (((count - 1U) & 3U) << 29U);
    out.ctl |= CTL_NAKSTS | CTL_DPID;
    out.ctl &= ~CTL_STALL;
    SetInNak(in);
    in.ctl |= CTL_DPID;
    in.ctl &= ~CTL_STALL;
    setup_done_due_ = true;
    set_address_stage_ = packet[0] == 0x00U && packet[1] == 0x05U;
    old_address_ = Address();
    const uint16_t length = static_cast<uint16_t>(packet[6] | (packet[7] << 8U));
    control_write_ = (packet[0] & 0x80U) == 0U && length != 0U;
    control_write_data_ = false;
    if (DmaEnabled())
    {
      dma_setups_.emplace_back(packet, packet + 8);
      DeliverDmaSetups();
    }
    else
    {
      uint32_t words[2];
      std::memcpy(words, packet, 8);
      rx_.push_back({Status(PKT_SETUP_DATA, 0, 8, 0), true});
      rx_.push_back({words[0], false});
      rx_.push_back({words[1], false});
    }
    return Resp::ACK;
  }

  Resp HostOut(uint8_t address, uint8_t num, uint8_t pid, const uint8_t* data,
               size_t length)
  {
    Tick("OUT", num);
    if (!Listening(address) || num >= config_.endpoints) return Resp::TIMEOUT;
    CheckResetSteps();
    OutEp& out = out_[num];
    if ((out.ctl & CTL_USBAEP) == 0U) return Resp::TIMEOUT;
    if (num == 0U) Ep0Token(false);
    const Type type = TypeOf(out.ctl);
    const bool iso = type == Type::ISOCHRONOUS;
    if (!iso && (out.ctl & CTL_STALL) != 0U) return Resp::STALL;
    // EP0 OUT has the packet size programmed for EP0 IN.
    const size_t packet = PacketSize(num == 0U ? in_[0].ctl : out.ctl, num);
    if (length > packet)
    {
      Violation("host sent %zu bytes to OUT endpoint %u with packet size %zu", length,
                num, packet);
      return Resp::TIMEOUT;
    }
    const bool open = (out.ctl & CTL_EPENA) != 0U && (out.ctl & CTL_NAKSTS) == 0U &&
                      !gonak_ &&
                      (!iso || ((out.ctl & CTL_DPID) != 0U) == ((frame_ & 1U) != 0U));
    if (!open)
    {
      if (num == 0U && (out.ctl & CTL_EPENA) == 0U) out.intr |= INT_OTEPDIS;
      return iso ? Resp::TIMEOUT : Resp::NAK;
    }
    const uint32_t words = static_cast<uint32_t>((length + 3U) / 4U);
    if (!DmaEnabled() && rx_.size() + 2U + words > (grxfsiz_ & 0xFFFFU))
      return iso ? Resp::TIMEOUT : Resp::NAK;
    if (!iso)
    {
      // A packet with the PID of the previous one is a resend: acknowledged and dropped.
      if (((out.ctl & CTL_DPID) != 0U) != (pid != 0U)) return Resp::ACK;
      out.ctl ^= CTL_DPID;
    }
    uint32_t count = PktCnt(out.tsiz, num);
    if (count != 0U) --count;
    SetPktCnt(out.tsiz, num, count);
    const bool done = count == 0U || length < packet;
    if (DmaEnabled())
    {
      if (length != 0U) DmaStore(out.dma, data, length, "OUT data");
      out.dma += static_cast<uint32_t>(length);
      SubXfrSiz(out.tsiz, num, length);
      if (done)
      {
        out.ctl |= CTL_NAKSTS;
        out.ctl &= ~CTL_EPENA;
        out.intr |= INT_XFRC;
      }
    }
    else
    {
      rx_.push_back({Status(PKT_OUT_DATA, num, length, pid), true});
      for (uint32_t i = 0; i < words; ++i)
      {
        uint32_t word = 0;
        std::memcpy(&word, data + i * 4U, std::min<size_t>(4U, length - i * 4U));
        rx_.push_back({word, false});
      }
      if (done)
      {
        out.ctl |= CTL_NAKSTS;
        rx_.push_back({Status(PKT_OUT_DONE, num, 0, 0), true});
      }
    }
    if (num == 0U && control_write_) control_write_data_ = true;
    return iso ? Resp::TIMEOUT : Resp::ACK;
  }

  /// 主机发 IN 令牌并确认收到的数据包 / The host sends an IN token and acknowledges the
  /// data packet it gets
  InResult HostIn(uint8_t address, uint8_t num)
  {
    Tick("IN", num);
    InResult result;
    // The status stage of SET_ADDRESS is still answered at the old address.
    const bool old_address = num == 0U && set_address_stage_ && address == old_address_;
    if ((!Listening(address) && !(old_address && Connected() && !suspended_)) ||
        num >= config_.endpoints)
      return result;
    CheckResetSteps();
    InEp& in = in_[num];
    if ((in.ctl & CTL_USBAEP) == 0U && num != 0U) return result;
    if (num == 0U) Ep0Token(true);
    const Type type = TypeOf(in.ctl);
    const bool iso = type == Type::ISOCHRONOUS;
    if (!iso && (in.ctl & CTL_STALL) != 0U)
    {
      result.resp = Resp::STALL;
      return result;
    }
    const bool ready = (in.ctl & CTL_EPENA) != 0U && (in.ctl & CTL_NAKSTS) == 0U &&
                       !in.packets.empty() && (iso || !ginak_) &&
                       (!iso || ((in.ctl & CTL_DPID) != 0U) == ((frame_ & 1U) != 0U));
    if (!ready)
    {
      if ((in.ctl & CTL_EPENA) != 0U && (in.ctl & CTL_NAKSTS) == 0U && in.packets.empty())
        in.intr |= INT_ITTXFE;
      // An isochronous endpoint without data answers with a zero-length packet.
      // The manual has that for an enabled endpoint whose FIFO lacks the data of the
      // frame (RM0468 "IN data transfers", isochronous: "a zero data length packet would
      // be transmitted") and for NAKSTS set (DIEPCTL.NAKSTS).
      // ASSUMPTION: the manual does not say what an active isochronous IN endpoint does
      // when EPENA is clear or when the token comes in a frame of the other parity than
      // EONUM. The model answers a zero-length packet there as well, so that the host
      // sees one packet per frame; TestIsochronousFrames and TestUacStreams rely on it
      // for the frame before the scheduled one.
      result.resp = iso ? Resp::DATA : Resp::NAK;
      return result;
    }
    result.resp = Resp::DATA;
    result.pid = (in.ctl & CTL_DPID) != 0U ? 1U : 0U;
    result.data = std::move(in.packets.front());
    in.packets.pop_front();
    if (!iso) in.ctl ^= CTL_DPID;
    uint32_t count = PktCnt(in.tsiz, num);
    if (count != 0U) --count;
    SetPktCnt(in.tsiz, num, count);
    if (DmaEnabled()) DmaFetch(in, num);
    if (count == 0U && XfrSiz(in.tsiz, num) == 0U)
    {
      in.ctl &= ~CTL_EPENA;
      in.intr |= INT_XFRC;
    }
    if (num == 0U && set_address_stage_ && result.data.empty())
      set_address_stage_ = false;  // The new address applies from here on.
    return result;
  }

  // -------------------------------------------------------------------------
  // Register access (MmioTrap::Device)
  // -------------------------------------------------------------------------

  uint32_t MmioPeek(uint32_t offset) override
  {
    if (before_read) before_read(offset);
    return Value(offset);
  }

  void MmioAfterRead(uint32_t offset) override
  {
    Elapse(false);
    AfterRead(offset);
  }

 private:
  uint32_t Value(uint32_t offset)
  {
    if (offset >= FIFO)
    {
      if (rx_.empty()) return 0U;
      return rx_.front().value;
    }
    switch (offset)
    {
      case GOTGCTL:
        return gotgctl_;
      case GOTGINT:
        return gotgint_;
      case GAHBCFG:
        return gahbcfg_;
      case GUSBCFG:
        return gusbcfg_;
      case GRSTCTL:
        return (AhbIdle() ? RST_AHBIDL : 0U) | (grstctl_ & 0x7C0U) | busy_bits_;
      case GINTSTS:
        return Gintsts();
      case GINTMSK:
        return gintmsk_;
      case GRXSTSR:
      case GRXSTSP:
        return rx_.empty() ? 0U : rx_.front().value;
      case GRXFSIZ:
        return grxfsiz_;
      case DIEPTXF0:
        return txf_[0];
      case GSNPSID:
        return config_.snpsid;
      case DCFG:
        return dcfg_;
      case DCTL:
        return dctl_ | (gonak_ ? DCTL_GONSTS : 0U) | (ginak_ ? DCTL_GINSTS : 0U);
      case DSTS:
        return (suspended_ ? 1U : 0U) | (enumspd_ << 1U) |
               (static_cast<uint32_t>(VisibleFrame()) << 8U);
      case DIEPMSK:
        return diepmsk_;
      case DOEPMSK:
        return doepmsk_;
      case DAINT:
        return Daint();
      case DAINTMSK:
        return daintmsk_;
      case DIEPEMPMSK:
        return diepempmsk_;
      default:
        break;
    }
    if (offset >= GHWCFG1 && offset < GHWCFG1 + 16U)
      return config_.ghwcfg[(offset - GHWCFG1) / 4U];
    if (offset >= DIEPTXF1 && offset < DIEPTXF1 + 15U * 4U)
      return txf_[1U + (offset - DIEPTXF1) / 4U];
    if (offset >= DIEP && offset < DIEP + 16U * 0x20U)
    {
      const uint8_t num = static_cast<uint8_t>((offset - DIEP) / 0x20U);
      if (num >= config_.endpoints) return 0U;
      const InEp& in = in_[num];
      switch ((offset - DIEP) % 0x20U)
      {
        case EP_CTL:
          return in.ctl | (in.stuck_disable || in.disable_due != 0U ? CTL_EPDIS : 0U);
        case EP_INT:
          return InInt(in);
        case EP_TSIZ:
          return in.tsiz;
        case EP_DMA:
          return in.dma;
        case EP_TXFSTS:
          return TxDepth(in) > TxUsed(in) ? TxDepth(in) - TxUsed(in) : 0U;
        default:
          return 0U;
      }
    }
    if (offset >= DOEP && offset < DOEP + 16U * 0x20U)
    {
      const uint8_t num = static_cast<uint8_t>((offset - DOEP) / 0x20U);
      if (num >= config_.endpoints) return 0U;
      const OutEp& out = out_[num];
      switch ((offset - DOEP) % 0x20U)
      {
        case EP_CTL:
          return out.ctl | (out.stuck_disable || out.disable_due != 0U ? CTL_EPDIS : 0U);
        case EP_INT:
          return out.intr;
        case EP_TSIZ:
          return out.tsiz;
        case EP_DMA:
          return out.dma;
        default:
          return 0U;
      }
    }
    return other_[offset / 4U];
  }

  void AfterRead(uint32_t offset)
  {
    if (offset >= FIFO)
    {
      if (rx_.empty() || rx_.front().status)
      {
        Violation("receive FIFO data read at 0x%X with no data word on top", offset);
        return;
      }
      rx_.pop_front();
      return;
    }
    if (offset == GRSTCTL)
    {
      if (busy_reads_ != 0U)
      {
        if (--busy_reads_ == 0U) EndBusy();
      }
      else if (ahb_hold_reads_ != 0U)
      {
        --ahb_hold_reads_;
      }
      return;
    }
    if (offset != GRXSTSP) return;
    if (rx_.empty())
    {
      Violation("GRXSTSP popped with an empty receive FIFO");
      return;
    }
    if (!rx_.front().status)
    {
      Violation("GRXSTSP popped while packet data is on top of the receive FIFO");
      return;
    }
    const uint32_t status = rx_.front().value;
    rx_.pop_front();
    const uint8_t num = static_cast<uint8_t>(status & 0xFU);
    const size_t bytes = (status >> 4U) & 0x7FFU;
    switch ((status >> 17U) & 0xFU)
    {
      case PKT_GONAK:
        gonak_effective_ = true;
        break;
      case PKT_OUT_DATA:
        SubXfrSiz(out_[num].tsiz, num, bytes);
        break;
      case PKT_OUT_DONE:
        out_[num].ctl &= ~CTL_EPENA;
        out_[num].intr |= INT_XFRC;
        break;
      case PKT_SETUP_DONE:
        out_[0].ctl &= ~CTL_EPENA;
        out_[0].intr |= INT_STUP;
        break;
      case PKT_SETUP_DATA:
        if (slave_setup_event) out_[0].intr |= INT_XFRC | INT_STPKTRX;
        break;
      default:
        break;
    }
  }

  void MmioWrite(uint32_t offset, uint32_t value) override
  {
    if (before_write)
    {
      auto hook = std::move(before_write);
      before_write = nullptr;
      hook(offset, value);
    }
    if (log != nullptr) Log("write %03X = %08X", offset, value);
    Write(offset, value);
    Elapse(false);
    if (after_write)
    {
      auto hook = std::move(after_write);
      after_write = nullptr;
      hook(offset, value);
    }
  }

 private:
  struct InEp
  {
    uint32_t ctl = 0, intr = 0, tsiz = 0, dma = 0;
    std::deque<std::vector<uint8_t>> packets;  // Complete packets in the transmit FIFO.
    std::vector<uint32_t> partial;             // Words of the packet being written.
    bool reprogrammed = false;  // DIEPTSIZ was written while the endpoint was enabled.
    bool stuck_disable = false;
    unsigned disable_due = 0;  // Register accesses left until a written EPDIS acts.
  };
  struct OutEp
  {
    uint32_t ctl = 0, intr = 0, tsiz = 0, dma = 0;
    bool stuck_disable = false;
    unsigned disable_due = 0;
  };
  struct RxWord
  {
    uint32_t value;
    bool status;
  };

  void Violation(const char* format, ...) __attribute__((format(printf, 2, 3)))
  {
    char text[200];
    va_list arguments;
    va_start(arguments, format);
    std::vsnprintf(text, sizeof(text), format, arguments);
    va_end(arguments);
    for (const auto& known : tolerated)
    {
      if (std::strstr(text, known.c_str()) != nullptr)
      {
        notes.emplace_back(text);
        return;
      }
    }
    violations.emplace_back(text);
  }
  void Log(const char* format, ...) __attribute__((format(printf, 2, 3)))
  {
    va_list arguments;
    va_start(arguments, format);
    std::vfprintf(log, format, arguments);
    va_end(arguments);
    std::fputc(10, log);
  }
  void Note(const char* format, ...) __attribute__((format(printf, 2, 3)))
  {
    char text[200];
    va_list arguments;
    va_start(arguments, format);
    std::vsnprintf(text, sizeof(text), format, arguments);
    va_end(arguments);
    notes.emplace_back(text);
  }

  static Type TypeOf(uint32_t ctl) { return static_cast<Type>((ctl >> 18U) & 3U); }
  static size_t PacketSize(uint32_t ctl, uint8_t num)
  {
    return num == 0U ? (64U >> (ctl & 3U)) : (ctl & CTL_MPSIZ);
  }
  static uint32_t PktCnt(uint32_t tsiz, uint8_t num)
  {
    return (tsiz >> 19U) & (num == 0U ? 3U : 0x3FFU);
  }
  static void SetPktCnt(uint32_t& tsiz, uint8_t num, uint32_t count)
  {
    const uint32_t mask = (num == 0U ? 3U : 0x3FFU) << 19U;
    tsiz = (tsiz & ~mask) | ((count << 19U) & mask);
  }
  static uint32_t XfrSiz(uint32_t tsiz, uint8_t num)
  {
    return tsiz & (num == 0U ? 0x7FU : 0x7FFFFU);
  }
  static void SubXfrSiz(uint32_t& tsiz, uint8_t num, size_t bytes)
  {
    const uint32_t mask = num == 0U ? 0x7FU : 0x7FFFFU;
    const uint32_t size = tsiz & mask;
    tsiz = (tsiz & ~mask) | (size > bytes ? size - static_cast<uint32_t>(bytes) : 0U);
  }
  static uint32_t Status(uint32_t kind, uint8_t num, size_t bytes, uint8_t pid)
  {
    return num | (static_cast<uint32_t>(bytes) << 4U) | ((pid != 0U ? 2U : 0U) << 15U) |
           (kind << 17U);
  }

  bool VbusSensed() const { return (gotgctl_ & OTGCTL_BVALOEN) == 0U; }
  bool Listening(uint8_t address) const
  {
    return Connected() && !suspended_ && !in_reset_ && address == Address();
  }

  uint32_t TxDepth(const InEp& in) const
  {
    return txf_[(in.ctl & CTL_TXFNUM) >> 22U] >> 16U;
  }
  static uint32_t TxUsed(const InEp& in)
  {
    uint32_t words = static_cast<uint32_t>(in.partial.size());
    for (const auto& packet : in.packets)
      words += static_cast<uint32_t>((packet.size() + 3U) / 4U);
    return words;
  }
  uint32_t InInt(const InEp& in) const
  {
    const uint32_t used = TxUsed(in);
    const bool empty =
        (gahbcfg_ & AHB_TXFELVL) != 0U ? used == 0U : used <= TxDepth(in) / 2U;
    return in.intr | (empty ? INT_TXFE : 0U);
  }
  uint32_t Daint() const
  {
    uint32_t bits = 0;
    for (uint8_t i = 0; i < config_.endpoints; ++i)
    {
      const uint32_t mask = diepmsk_ | (((diepempmsk_ >> i) & 1U) != 0U ? INT_TXFE : 0U);
      if ((InInt(in_[i]) & mask) != 0U) bits |= 1U << i;
      if ((out_[i].intr & doepmsk_) != 0U) bits |= 1U << (16U + i);
    }
    return bits;
  }
  uint32_t Gintsts() const
  {
    uint32_t value = latched_ | GINT_NPTXFE | GINT_PTXFE;
    if (!rx_.empty()) value |= GINT_RXFLVL;
    const uint32_t daint = Daint() & daintmsk_;
    if ((daint & 0xFFFFU) != 0U) value |= GINT_IEPINT;
    if ((daint >> 16U) != 0U) value |= GINT_OEPINT;
    if (gotgint_ != 0U) value |= GINT_OTG;
    if (gonak_effective_) value |= GINT_GONAKEFF;
    if (ginak_) value |= GINT_GINAKEFF;
    return value;
  }

  /// 总线上发生了别的事：自清位早已清零 / Something else happened on the bus: the
  /// self-clearing bits are long clear
  /// 一次寄存器访问过去了：到期的 EPDIS 生效 / One register access went by: a due
  /// EPDIS takes effect
  void Elapse(bool all)
  {
    if (gonak_due_ != 0U && (all || --gonak_due_ == 0U))
    {
      gonak_due_ = 0;
      gonak_effective_ = gonak_;
    }
    for (uint8_t i = 0; i < config_.endpoints; ++i)
    {
      InEp& in = in_[i];
      if (in.disable_due != 0U && (all || --in.disable_due == 0U))
      {
        in.disable_due = 0;
        in.ctl &= ~CTL_EPENA;
        in.intr |= INT_EPDISD;
      }
      OutEp& out = out_[i];
      if (out.disable_due != 0U && (all || --out.disable_due == 0U))
      {
        out.disable_due = 0;
        out.ctl &= ~CTL_EPENA;
        out.intr |= INT_EPDISD;
      }
    }
  }

  void Tick(const char* event, unsigned number = 0)
  {
    EndBusy();
    Elapse(true);
    if (log != nullptr) Log("bus: %s %u (frame %u)", event, number, frame_);
  }

  bool AhbIdle() const
  {
    return !ahbidl_follows_flush || (busy_bits_ == 0U && ahb_hold_reads_ == 0U);
  }

  /// 自清位清零；发送 FIFO 的冲刷在这一刻生效 / The self-clearing bits clear; the flush
  /// of the transmit FIFO takes effect now
  void EndBusy()
  {
    if ((busy_bits_ & RST_TXFFLSH) != 0U)
    {
      for (uint8_t i = 0; i < config_.endpoints; ++i)
      {
        InEp& in = in_[i];
        if (flush_fifo_ != 0x10U && FifoOf(in) != flush_fifo_) continue;
        in.packets.clear();
        in.partial.clear();
      }
    }
    if ((busy_bits_ & RST_TXFFLSH) != 0U && ahbidl_follows_flush) ahb_hold_reads_ = 2U;
    busy_bits_ = 0U;
    busy_reads_ = 0U;
  }

  static uint32_t FifoOf(const InEp& in) { return (in.ctl & CTL_TXFNUM) >> 22U; }

  /// 发送 FIFO 正在冲刷 / The transmit FIFO is being flushed
  bool Flushing(uint32_t fifo) const
  {
    return (busy_bits_ & RST_TXFFLSH) != 0U &&
           (flush_fifo_ == 0x10U || flush_fifo_ == fifo);
  }

  void SetInNak(InEp& in)
  {
    in.ctl |= CTL_NAKSTS;
    in.intr |= INT_INEPNE;
  }

  /// 总线复位之后主机的第一个令牌到达时，检查驱动是否做完了手册的"USB 复位时的端点初始化"
  /// （RM0468 62.15.6 Device programming model, Endpoint initialization on USB reset）：
  /// 1. 所有 OUT 端点 SNAK = 1；2. 打开 DAINTMSK 的 INEP0 和 OUTEP0、DOEPMSK 的 STUPM 和
  /// XFRCM、DIEPMSK 的 XFRCM 和 TOM；3. 接收 FIFO 至少 EP0 一个最大包加 2 个字（OUT
  /// 数据的 状态）加 10 个字（SETUP 包），EP0 的发送 FIFO 至少一个最大包；4.
  /// DOEPTSIZ0.STUPCNT = 3；
  /// 5. DMA 模式下 DOEPDMA0 是有效的内存地址。
  /// When the first token after a bus reset arrives, check that the driver completed
  /// "Endpoint initialization on USB reset" of the manual (RM0468 62.15.6 Device
  /// programming model): 1. SNAK = 1 on all OUT endpoints; 2. INEP0 and OUTEP0 unmasked
  /// in DAINTMSK, STUPM and XFRCM in DOEPMSK, XFRCM and TOM in DIEPMSK; 3. a receive FIFO
  /// of at least one EP0 maximum packet plus 2 words (status of the OUT data) plus 10
  /// words (SETUP packets) and a transmit FIFO 0 of at least one maximum packet; 4.
  /// STUPCNT = 3 in DOEPTSIZ0; 5. in DMA mode DOEPDMA0 holds a valid memory address.
  void CheckResetSteps()
  {
    if (!reset_steps_due_) return;
    reset_steps_due_ = false;
    static constexpr char RULE[] = "RM0468 62.15.6 Endpoint initialization on USB reset";
    for (uint8_t i = 0; i < config_.endpoints; ++i)
    {
      if (!snak_since_reset_[i])
        Violation("OUT endpoint %u not NAKed after the USB reset (%s, step 1)", i, RULE);
    }
    constexpr uint32_t DAINT_EP0 = 0x10001U;
    if ((daintmsk_ & DAINT_EP0) != DAINT_EP0)
      Violation(
          "EP0 interrupts not unmasked in DAINTMSK after the USB reset (%s, step 2)",
          RULE);
    if ((doepmsk_ & (INT_STUP | INT_XFRC)) != (INT_STUP | INT_XFRC))
      Violation("STUPM/XFRCM not set in DOEPMSK after the USB reset (%s, step 2)", RULE);
    if ((diepmsk_ & (INT_TOC | INT_XFRC)) != (INT_TOC | INT_XFRC))
      Violation("TOM/XFRCM not set in DIEPMSK after the USB reset (%s, step 2)", RULE);
    if ((grxfsiz_ & 0xFFFFU) < 16U + 2U + 10U)
      Violation("receive FIFO smaller than 28 words after the USB reset (%s, step 3)",
                RULE);
    if ((txf_[0] >> 16U) < 16U)
      Violation(
          "transmit FIFO 0 smaller than one packet after the USB reset (%s, step 3)",
          RULE);
    if (((out_[0].tsiz >> 29U) & 3U) != 3U)
      Violation("STUPCNT is not 3 in DOEPTSIZ0 after the USB reset (%s, step 4)", RULE);
    if (DmaEnabled() && !DmaReachable(out_[0].dma, 8U))
      Violation("DOEPDMA0 holds no valid address after the USB reset (%s, step 5)", RULE);
  }

  /// SETUP 之后 EP0 上的第一个 IN/OUT 令牌结束 SETUP 阶段 / The first IN or OUT token on
  /// EP0 after a SETUP ends the SETUP stage
  void Ep0Token(bool in_token)
  {
    if (setup_done_due_)
    {
      setup_done_due_ = false;
      if (DmaEnabled())
      {
        setup_done_held_ = true;
        FinishDmaSetup();
      }
      else
      {
        rx_.push_back({Status(PKT_SETUP_DONE, 0, 0, 0), true});
      }
    }
    if (in_token && control_write_ && control_write_data_ && DmaEnabled() &&
        config_.snpsid > SNPSID_300A)
    {
      // The host moved from the data stage of a control write to its status stage.
      control_write_data_ = false;
      OutEp& out = out_[0];
      out.intr |= INT_STSPHSRX;
      if ((out.ctl & CTL_EPENA) != 0U)
      {
        out.ctl &= ~CTL_EPENA;
        out.intr |= INT_XFRC;
      }
    }
  }

  void DeliverDmaSetups()
  {
    OutEp& out = out_[0];
    while (!dma_setups_.empty() && (out.ctl & CTL_EPENA) != 0U)
    {
      DmaStore(out.dma, dma_setups_.front().data(), 8U, "SETUP");
      out.dma += 8U;
      SubXfrSiz(out.tsiz, 0, 8U);
      dma_setups_.pop_front();
      if (config_.snpsid > SNPSID_300A && setup_packet_event_early)
        out.intr |= INT_XFRC | INT_STPKTRX;
    }
    FinishDmaSetup();
  }

  void FinishDmaSetup()
  {
    if (!setup_done_held_ || !dma_setups_.empty()) return;
    setup_done_held_ = false;
    OutEp& out = out_[0];
    out.ctl &= ~CTL_EPENA;
    out.intr |= INT_STUP;
    if (config_.snpsid > SNPSID_300A && !setup_packet_event_early)
      out.intr |= INT_XFRC | INT_STPKTRX;
  }

  bool DmaReachable(uint32_t address, size_t bytes) const
  {
    return config_.dma_memory != nullptr &&
           config_.dma_memory->Contains(address, (bytes + 3U) & ~size_t{3});
  }
  void DmaStore(uint32_t address, const uint8_t* data, size_t bytes, const char* what)
  {
    if (!DmaReachable(address, bytes))
    {
      Violation("DMA store of %s to 0x%08X outside DMA memory", what, address);
      return;
    }
    std::memcpy(reinterpret_cast<void*>(static_cast<uintptr_t>(address)), data, bytes);
  }
  void DmaFetch(InEp& in, uint8_t num)
  {
    if ((in.ctl & CTL_EPENA) == 0U) return;
    while (XfrSiz(in.tsiz, num) != 0U)
    {
      const size_t packet = PacketSize(in.ctl, num);
      const size_t length = std::min<size_t>(packet, XfrSiz(in.tsiz, num));
      if (TxUsed(in) + (length + 3U) / 4U > TxDepth(in)) break;
      std::vector<uint8_t> bytes(length);
      if (DmaReachable(in.dma, length))
        std::memcpy(bytes.data(), reinterpret_cast<void*>(static_cast<uintptr_t>(in.dma)),
                    length);
      else
        Violation("DMA fetch from 0x%08X outside DMA memory", in.dma);
      in.packets.push_back(std::move(bytes));
      in.dma += static_cast<uint32_t>(length);
      SubXfrSiz(in.tsiz, num, length);
    }
  }

  void FifoWrite(uint8_t num, uint32_t value)
  {
    if (num >= config_.endpoints)
    {
      Violation("write to the FIFO of endpoint %u, which does not exist", num);
      return;
    }
    InEp& in = in_[num];
    if (Flushing(FifoOf(in)))
      Violation(
          "word written to transmit FIFO %u while its flush is running (RM0468 "
          "OTG_GRSTCTL TXFFLSH: wait until the core clears the bit)",
          num);
    if (DmaEnabled() || (in.ctl & CTL_EPENA) == 0U || XfrSiz(in.tsiz, num) == 0U)
    {
      Violation("word written to transmit FIFO %u with no transfer data left", num);
      return;
    }
    in.partial.push_back(value);
    if (TxUsed(in) > TxDepth(in))
      Violation("transmit FIFO %u overflow (%u of %u words)", num, TxUsed(in),
                TxDepth(in));
    const size_t packet = PacketSize(in.ctl, num);
    const size_t length = std::min<size_t>(packet, XfrSiz(in.tsiz, num));
    if (in.partial.size() < (length + 3U) / 4U) return;
    std::vector<uint8_t> bytes(in.partial.size() * 4U);
    std::memcpy(bytes.data(), in.partial.data(), bytes.size());
    bytes.resize(length);
    in.partial.clear();
    in.packets.push_back(std::move(bytes));
    SubXfrSiz(in.tsiz, num, length);
  }

  void WriteInCtl(uint8_t num, uint32_t value)
  {
    InEp& in = in_[num];
    const uint32_t plain =
        num == 0U ? (3U | CTL_TXFNUM) : (CTL_MPSIZ | CTL_USBAEP | CTL_EPTYP | CTL_TXFNUM);
    in.ctl = (in.ctl & ~plain) | (value & plain);
    if (num == 0U)
    {
      in.ctl |= value & CTL_STALL;  // Set by software, cleared by a SETUP.
    }
    else
    {
      in.ctl = (in.ctl & ~CTL_STALL) | (value & CTL_STALL);
      if ((value & CTL_SD0PID) != 0U) in.ctl &= ~CTL_DPID;
      if ((value & CTL_SD1PID) != 0U) in.ctl |= CTL_DPID;
    }
    if ((value & CTL_CNAK) != 0U)
    {
      in.ctl &= ~CTL_NAKSTS;
      in.intr &= ~INT_INEPNE;
    }
    if ((value & CTL_SNAK) != 0U) SetInNak(in);
    if ((value & CTL_EPDIS) != 0U)
    {
      if ((in.ctl & CTL_EPENA) != 0U)
      {
        in.disable_due = disable_latency + 1U;
        in.stuck_disable = false;
      }
      else if (!in.stuck_disable)
      {
        Violation("EPDIS set on IN endpoint %u, which is not enabled", num);
        in.stuck_disable = true;
      }
    }
    else if ((value & CTL_EPENA) != 0U && ((in.ctl & CTL_EPENA) == 0U || in.reprogrammed))
    {
      // An endpoint whose size register was rewritten while it was enabled goes on with
      // the new values.
      in.reprogrammed = false;
      if (Flushing(FifoOf(in)))
        Violation(
            "IN endpoint %u enabled while its transmit FIFO is being flushed (RM0468 "
            "OTG_GRSTCTL TXFFLSH: wait until the core clears the bit)",
            num);
      if (TypeOf(in.ctl) == Type::ISOCHRONOUS)
      {
        in.packets.clear();
        in.partial.clear();
      }
      else if (TxUsed(in) != 0U)
      {
        Violation(
            "IN endpoint %u enabled with %u words of an earlier transfer in its "
            "FIFO",
            num, TxUsed(in));
      }
      in.ctl |= CTL_EPENA;
      if (XfrSiz(in.tsiz, num) == 0U && PktCnt(in.tsiz, num) != 0U)
        in.packets.emplace_back();
      if (DmaEnabled()) DmaFetch(in, num);
    }
  }

  void WriteOutCtl(uint8_t num, uint32_t value)
  {
    OutEp& out = out_[num];
    if (num != 0U)
    {
      const uint32_t plain = CTL_MPSIZ | CTL_USBAEP | CTL_EPTYP | CTL_SNPM | CTL_STALL;
      out.ctl = (out.ctl & ~plain) | (value & plain);
      if ((value & CTL_SD0PID) != 0U) out.ctl &= ~CTL_DPID;
      if ((value & CTL_SD1PID) != 0U) out.ctl |= CTL_DPID;
    }
    else
    {
      out.ctl = (out.ctl & ~CTL_SNPM) | (value & CTL_SNPM);
      out.ctl |= value & CTL_STALL;
    }
    if ((value & CTL_CNAK) != 0U) out.ctl &= ~CTL_NAKSTS;
    if ((value & CTL_SNAK) != 0U)
    {
      out.ctl |= CTL_NAKSTS;
      snak_since_reset_[num] = true;
    }
    if ((value & CTL_EPDIS) != 0U && num != 0U)
    {
      if ((out.ctl & CTL_EPENA) != 0U)
      {
        if (!gonak_effective_ && TypeOf(out.ctl) != Type::ISOCHRONOUS)
          Violation("OUT endpoint %u disabled without global OUT NAK in effect", num);
        out.disable_due = disable_latency + 1U;
        out.stuck_disable = false;
      }
      else if (!out.stuck_disable)
      {
        Violation("EPDIS set on OUT endpoint %u, which is not enabled", num);
        out.stuck_disable = true;
      }
    }
    else if ((value & CTL_EPENA) != 0U && (out.ctl & CTL_EPENA) == 0U)
    {
      out.ctl |= CTL_EPENA;
      if (num == 0U && DmaEnabled()) DeliverDmaSetups();
    }
  }

  void Flush(uint32_t value)
  {
    static constexpr char RULE[] = "RM0468 OTG_GRSTCTL";
    if ((busy_bits_ & RST_TXFFLSH) != 0U)
    {
      if (((value ^ grstctl_) & 0x7C0U) != 0U)
        Violation(
            "TXFNUM changed while TXFFLSH is still set (%s: not before the core clears "
            "it)",
            RULE);
      if ((value & RST_TXFFLSH) != 0U)
        Violation(
            "TXFFLSH written while the previous flush is still running (%s: wait until "
            "the core clears the bit)",
            RULE);
    }
    else if ((value & RST_TXFFLSH) != 0U && !AhbIdle())
    {
      Violation("TXFFLSH written while AHBIDL is 0 (%s: check AHBIDL first)", RULE);
    }
    grstctl_ = value & 0x7C0U;
    if ((value & RST_CSRST) != 0U)
    {
      CoreReset(true);
      busy_bits_ |= RST_CSRST;
    }
    if ((value & RST_RXFFLSH) != 0U)
    {
      rx_.clear();
      busy_bits_ |= RST_RXFFLSH;
    }
    if ((value & RST_TXFFLSH) != 0U)
    {
      const uint32_t fifo = (value >> 6U) & 0x1FU;
      for (uint8_t i = 0; i < config_.endpoints; ++i)
      {
        const InEp& in = in_[i];
        if (fifo != 0x10U && FifoOf(in) != fifo) continue;
        if ((in.ctl & CTL_EPENA) != 0U && (in.ctl & CTL_NAKSTS) == 0U && !in_reset_)
          Note("transmit FIFO %u flushed while IN endpoint %u is enabled without NAK",
               fifo, i);
      }
      flush_fifo_ = fifo;
      busy_bits_ |= RST_TXFFLSH;
    }
    if (busy_bits_ == 0U)
      busy_reads_ = 0U;
    else if (self_clear_reads == 0U)
      EndBusy();
    else
      busy_reads_ = self_clear_reads;
  }

  void Write(uint32_t offset, uint32_t value)
  {
    if (offset >= FIFO)
    {
      FifoWrite(static_cast<uint8_t>((offset - FIFO) / FIFO_STRIDE), value);
      return;
    }
    switch (offset)
    {
      case GOTGCTL:
        gotgctl_ = value;
        return;
      case GOTGINT:
        gotgint_ &= ~value;
        return;
      case GAHBCFG:
        gahbcfg_ = value;
        return;
      case GUSBCFG:
        gusbcfg_ = value;
        return;
      case GRSTCTL:
        Flush(value);
        return;
      case GINTSTS:
        latched_ &= ~(value & GINT_W1C);
        if ((value & GINT_SOF) != 0U && sof_again_)
        {
          sof_again_ = false;
          latched_ |= GINT_SOF;
        }
        return;
      case GINTMSK:
        gintmsk_ = value;
        return;
      case GRXFSIZ:
        grxfsiz_ = value;
        return;
      case DIEPTXF0:
        txf_[0] = value;
        return;
      case DCFG:
        dcfg_ = value;
        return;
      case DCTL:
        dctl_ = value & (DCTL_RWUSIG | DCTL_SDIS | 0x00078870U);
        if ((value & DCTL_SGINAK) != 0U) ginak_ = true;
        if ((value & DCTL_CGINAK) != 0U) ginak_ = false;
        if ((value & DCTL_SGONAK) != 0U)
        {
          if (gonak_effective_)
            Violation("SGONAK set while global OUT NAK is already in effect");
          if (!gonak_)
          {
            gonak_ = true;
            if (DmaEnabled() && gonak_latency == 0U)
              gonak_effective_ = true;
            else if (DmaEnabled())
              gonak_due_ = gonak_latency;
            else
              rx_.push_back({Status(PKT_GONAK, 0, 0, 0), true});
          }
        }
        if ((value & DCTL_CGONAK) != 0U)
        {
          gonak_ = false;
          gonak_effective_ = false;
          gonak_due_ = 0;
        }
        return;
      case DIEPMSK:
        diepmsk_ = value;
        return;
      case DOEPMSK:
        doepmsk_ = value;
        return;
      case DAINTMSK:
        daintmsk_ = value;
        return;
      case DIEPEMPMSK:
        diepempmsk_ = value;
        return;
      default:
        break;
    }
    if (offset >= DIEPTXF1 && offset < DIEPTXF1 + 15U * 4U)
    {
      txf_[1U + (offset - DIEPTXF1) / 4U] = value;
      return;
    }
    if (offset >= DIEP && offset < DIEP + 16U * 0x20U)
    {
      const uint8_t num = static_cast<uint8_t>((offset - DIEP) / 0x20U);
      if (num >= config_.endpoints) return;
      InEp& in = in_[num];
      switch ((offset - DIEP) % 0x20U)
      {
        case EP_CTL:
          WriteInCtl(num, value);
          break;
        case EP_INT:
          in.intr &= ~(value & ~INT_TXFE);
          break;
        case EP_TSIZ:
          if ((in.ctl & CTL_EPENA) != 0U)
          {
            Violation("DIEPTSIZ%u written while the endpoint is enabled", num);
            in.reprogrammed = true;
          }
          in.tsiz = value & (num == 0U ? 0x0018007FU : 0x7FFFFFFFU);
          break;
        case EP_DMA:
          in.dma = value;
          break;
        default:
          break;
      }
      return;
    }
    if (offset >= DOEP && offset < DOEP + 16U * 0x20U)
    {
      const uint8_t num = static_cast<uint8_t>((offset - DOEP) / 0x20U);
      if (num >= config_.endpoints) return;
      OutEp& out = out_[num];
      switch ((offset - DOEP) % 0x20U)
      {
        case EP_CTL:
          WriteOutCtl(num, value);
          break;
        case EP_INT:
          out.intr &= ~value;
          break;
        case EP_TSIZ:
          if ((out.ctl & CTL_EPENA) != 0U)
          {
            // EP0 OUT stays enabled for SETUP packets in DMA mode; drivers reprogram it.
            if (num == 0U)
              Note("DOEPTSIZ0 written while EP0 OUT is enabled");
            else
              Violation("DOEPTSIZ%u written while the endpoint is enabled", num);
          }
          out.tsiz = value & (num == 0U ? 0x6008007FU : 0x7FFFFFFFU);
          break;
        case EP_DMA:
          out.dma = value;
          break;
        default:
          break;
      }
      return;
    }
    other_[offset / 4U] = value;
  }

  /// 核心软复位保留 DCFG.DSPD、DCTL.SDIS、GCCFG 和 PCGCCTL / A core soft reset keeps
  /// DCFG.DSPD, DCTL.SDIS, GCCFG and PCGCCTL
  void CoreReset(bool soft)
  {
    const uint32_t keep_dcfg = soft ? (dcfg_ & 3U) : 0U;
    const uint32_t keep_dctl = soft ? (dctl_ & DCTL_SDIS) : DCTL_SDIS;
    const uint32_t gccfg = soft ? other_[GCCFG / 4U] : 0U;
    const uint32_t pcgcctl = soft ? other_[PCGCCTL / 4U] : 0U;
    std::memset(other_, 0, sizeof(other_));
    other_[GCCFG / 4U] = gccfg;
    other_[PCGCCTL / 4U] = pcgcctl;
    gotgctl_ = gotgint_ = gahbcfg_ = 0;
    if (!soft) gusbcfg_ = 0x1440U;
    grstctl_ = 0;
    busy_bits_ = 0;
    busy_reads_ = 0;
    ahb_hold_reads_ = 0;
    latched_ = 0;
    sof_again_ = false;
    gintmsk_ = 0;
    grxfsiz_ = config_.fifo_depth;
    for (auto& value : txf_)
      value = (static_cast<uint32_t>(config_.fifo_depth) << 16U) | config_.fifo_depth;
    dcfg_ = 0x02200000U | keep_dcfg;
    dctl_ = keep_dctl;
    diepmsk_ = doepmsk_ = daintmsk_ = diepempmsk_ = 0;
    enumspd_ = 1;
    gonak_ = gonak_effective_ = ginak_ = false;
    gonak_due_ = 0;
    for (auto& in : in_) in = InEp{};
    for (auto& out : out_) out = OutEp{};
    out_[0].ctl = CTL_USBAEP;
    in_[0].ctl = CTL_USBAEP;
    rx_.clear();
    dma_setups_.clear();
    in_reset_ = false;
    reset_steps_due_ = false;
    setup_done_due_ = setup_done_held_ = false;
    set_address_stage_ = false;
    control_write_ = control_write_data_ = false;
    old_address_ = 0;
  }

  Config config_;
  uint32_t other_[FIFO / 4U] = {};  // Registers without modelled behaviour: plain memory.
  uint32_t gotgctl_ = 0, gotgint_ = 0, gahbcfg_ = 0, gusbcfg_ = 0, grstctl_ = 0;
  uint32_t busy_bits_ = 0;
  unsigned busy_reads_ = 0;
  unsigned ahb_hold_reads_ = 0;  // Reads left that still see AHBIDL = 0 after a flush.
  uint32_t flush_fifo_ = 0;      // TXFNUM of the transmit FIFO flush that is running.
  uint32_t latched_ = 0;         // Write-one-to-clear bits of GINTSTS.
  bool sof_again_ = false;       // The repeated SOF of the last token is still due.
  uint32_t gintmsk_ = 0, grxfsiz_ = 0, txf_[16] = {};
  uint32_t dcfg_ = 0, dctl_ = 0, diepmsk_ = 0, doepmsk_ = 0, daintmsk_ = 0;
  uint32_t diepempmsk_ = 0, enumspd_ = 1;
  bool gonak_ = false, gonak_effective_ = false, ginak_ = false;
  unsigned gonak_due_ = 0;  // Register accesses left until a written SGONAK acts (DMA).
  InEp in_[16];
  OutEp out_[16];
  std::deque<RxWord> rx_;
  std::deque<std::vector<uint8_t>> dma_setups_;  // SETUP packets not yet in memory.

  // Bus state.
  bool vbus_ = true, suspended_ = false, in_reset_ = false;
  bool reset_steps_due_ = false;  // The first token after a reset checks the steps.
  bool snak_since_reset_[16] =
      {};  // SNAK was written to the OUT endpoint after the reset.
  uint16_t frame_ = 0;
  bool setup_done_due_ = false, setup_done_held_ = false;
  bool set_address_stage_ = false;
  uint8_t old_address_ = 0;
  bool control_write_ = false, control_write_data_ = false;
};

}  // namespace Dwc2
