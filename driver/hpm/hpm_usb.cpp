#include "hpm_usb.hpp"

#if defined(HPM_USB0_BASE)

#include <cstring>

#include "hpm_interrupt.h"
#include "hpm_misc.h"
#include "hpm_usb_drv.h"
#include "timebase.hpp"

static_assert(LibXR::HPMUSBController::MAX_ENDPOINTS <=
                  sizeof(USB_Type::ENDPTCTRL) / sizeof(USB_Type::ENDPTCTRL[0]),
              "MAX_ENDPOINTS exceeds ENDPTCTRL register count");

using namespace LibXR;
using LibXR::USB::Endpoint;

namespace
{

// dTD 令牌（SDK hpm_usb_device.h dcd_qtd_t）。
// dTD token (dcd_qtd_t in the SDK's hpm_usb_device.h).
constexpr uint32_t TOKEN_ACTIVE = 1UL << 7U;
constexpr uint32_t TOKEN_HALTED = 1UL << 6U;
constexpr uint32_t TOKEN_BUFFER_ERROR = 1UL << 5U;
constexpr uint32_t TOKEN_TRANSACTION_ERROR = 1UL << 3U;
constexpr uint32_t TOKEN_ERRORS =
    TOKEN_HALTED | TOKEN_BUFFER_ERROR | TOKEN_TRANSACTION_ERROR;
constexpr uint32_t TOKEN_IOC = 1UL << 15U;
constexpr uint32_t TOKEN_BYTES_SHIFT = 16U;
constexpr uint32_t TOKEN_BYTES_MASK = 0x7FFFU;
// next 字段的结束位 / Terminate bit of a next field
constexpr uint32_t NEXT_TERMINATE = 1U;

// dQH 能力字（SDK dcd_qhd_t）：IOS 位 15、包长 16-26、ZLT 位 29、MULT 30-31。
// dQH capabilities (SDK dcd_qhd_t): IOS bit 15, size 16-26, ZLT bit 29, MULT 30-31.
constexpr uint32_t CAP_IOS = 1UL << 15U;
constexpr uint32_t CAP_SIZE_SHIFT = 16U;
constexpr uint32_t CAP_ZLT_OFF = 1UL << 29U;
constexpr uint32_t CAP_MULT_SHIFT = 30U;
// dTD 令牌的 MultO（位 11:10），只用于发送方向的等时 dTD。
// MultO of a dTD token (bits 11:10), for transmit isochronous dTDs only.
constexpr uint32_t TOKEN_MULTO_SHIFT = 10U;

// ENDPTCTRL 半字内的位（OUT 在位 0 起，IN 在位 16 起）。
// Bits within a half of ENDPTCTRL (OUT from bit 0, IN from bit 16).
constexpr uint32_t CTRL_STALL = 1UL << 0U;
constexpr uint32_t CTRL_TYPE_SHIFT = 2U;
constexpr uint32_t CTRL_TYPE_MASK = 3UL << CTRL_TYPE_SHIFT;
constexpr uint32_t CTRL_TOGGLE_RESET = 1UL << 6U;
constexpr uint32_t CTRL_ENABLE = 1UL << 7U;
constexpr uint32_t TYPE_BULK = 2U;

constexpr uint32_t EP0_BITS = 0x00010001UL;
constexpr size_t EP0_PACKET = 64U;
constexpr size_t PAGE = 4096U;
constexpr uint32_t PSPD_HIGH = 2U;
// FlushAll 的上限：只在总线上没有包时调用，实测复位期间 8.4 us。
// Bound of FlushAll: called only with no packet on the bus, measured 8.4 us in a reset.
constexpr uint64_t HANDSHAKE_US = 50U;
// 读 SETUP 的 tripwire 循环和 STALL 重写的次数上限 / Bound of the tripwire loop that
// reads a SETUP and of STALL rewrites
constexpr uint32_t RETRY_LIMIT = 16U;

constexpr uint32_t INTERRUPTS = USB_USBINTR_UE_MASK | USB_USBINTR_UEE_MASK |
                                USB_USBINTR_PCE_MASK | USB_USBINTR_URE_MASK |
                                USB_USBINTR_SLE_MASK | USB_USBINTR_SRE_MASK;

uint64_t NowUs() { return static_cast<uint64_t>(Timebase::GetMicroseconds()); }

/// DMA 看到的地址 / Address as the DMA sees it
uint32_t DmaAddress(const volatile void* pointer)
{
  return core_local_mem_to_sys_address(
      0U, static_cast<uint32_t>(reinterpret_cast<uintptr_t>(pointer)));
}

/// 内存和寄存器写入的先后对 DMA 可见 / Make memory and register writes ordered for DMA
void Fence() { __asm volatile("fence iorw, iorw" ::: "memory"); }

/// 令牌里的剩余字节 / Bytes left in a token
size_t BytesLeft(uint32_t token)
{
  return (token >> TOKEN_BYTES_SHIFT) & TOKEN_BYTES_MASK;
}

/// 读寄存器直到 done 成立，最多 HANDSHAKE_SPINS 次 / Read registers until done holds,
/// HANDSHAKE_SPINS times at most
template <typename Done>
bool Spin(Done done)
{
  for (uint32_t i = 0; i < HPMUSBEndpoint::HANDSHAKE_SPINS; ++i)
  {
    if (done())
    {
      return true;
    }
  }
  return false;
}

/// 端点类型在 ENDPTCTRL 中的编码 / Endpoint type code in ENDPTCTRL
uint32_t TypeCode(Endpoint::Type type) { return static_cast<uint32_t>(type); }
}  // namespace

// ---------------------------------------------------------------------------
// Controller
// ---------------------------------------------------------------------------

HPMUSBController::HPMUSBController(
    USB_Type* regs, uint32_t irq, bool high_speed,
    const std::initializer_list<HPMUSBEndpointMemory>& rx_ep_cfgs,
    const std::initializer_list<HPMUSBEndpointMemory>& tx_ep_cfgs)
    : regs_(regs), irq_(irq)
{
  REQUIRE(rx_ep_cfgs.size() > 0U && rx_ep_cfgs.size() <= MAX_ENDPOINTS);
  REQUIRE(tx_ep_cfgs.size() > 0U && tx_ep_cfgs.size() <= MAX_ENDPOINTS);

  // SDK 的初始化：PHY、控制器复位、设备模式、SLOM（SETUP 用 tripwire 读）、不用中断
  // 门限。之后保持断开（RS=0），到 StartController 才接上。
  // The SDK's initialization: PHY, controller reset, device mode, SLOM (SETUPs read with
  // the tripwire), no interrupt threshold. It stays detached (RS=0) until
  // StartController.
  usb_dcd_init(regs_);
  if (high_speed)
  {
    regs_->PORTSC1 &= ~USB_PORTSC1_PFSC_MASK;
  }
  else
  {
    regs_->PORTSC1 |= USB_PORTSC1_PFSC_MASK;
  }
  regs_->USBINTR = 0U;
  for (auto& qh : qh_)
  {
    qh.next = NEXT_TERMINATE;
  }
  // EP0 OUT 收 SETUP 只看 dQH 的能力字，接上总线之前就写好。
  // EP0 OUT receives SETUPs from the dQH capabilities alone; written before attaching.
  qh_[QueueIndex(0U, false)].capabilities =
      (EP0_PACKET << CAP_SIZE_SHIFT) | CAP_ZLT_OFF | CAP_IOS;
  qh_[QueueIndex(0U, true)].capabilities = (EP0_PACKET << CAP_SIZE_SHIFT) | CAP_ZLT_OFF;
  usb_dcd_set_edpt_list_addr(regs_, DmaAddress(qh_));
  // 非控制端点两个方向的类型都不能是控制（复位值）。
  // Neither direction of a non-control endpoint may keep the control type (the reset
  // value).
  for (uint8_t i = 1U; i < MAX_ENDPOINTS; ++i)
  {
    regs_->ENDPTCTRL[i] =
        (TYPE_BULK << CTRL_TYPE_SHIFT) | ((TYPE_BULK << CTRL_TYPE_SHIFT) << IN_SHIFT);
  }

  auto rx = rx_ep_cfgs.begin();
  auto tx = tx_ep_cfgs.begin();
  REQUIRE(rx->memory.addr_ != nullptr && tx->memory.addr_ != nullptr);
  auto* ep0_out =
      new HPMUSBEndpoint(*this, Endpoint::EPNumber::EP0, Endpoint::Direction::OUT, *rx);
  auto* ep0_in =
      new HPMUSBEndpoint(*this, Endpoint::EPNumber::EP0, Endpoint::Direction::IN, *tx);
  SetEndpoint0(ep0_in, ep0_out);
  auto number = Endpoint::EPNumber::EP1;
  for (++rx; rx != rx_ep_cfgs.end(); ++rx)
  {
    if (rx->memory.addr_ != nullptr)
    {
      REQUIRE(Put(new HPMUSBEndpoint(*this, number, Endpoint::Direction::OUT, *rx)) ==
              ErrorCode::OK);
    }
    number = Endpoint::NextEPNumber(number);
  }
  number = Endpoint::EPNumber::EP1;
  for (++tx; tx != tx_ep_cfgs.end(); ++tx)
  {
    if (tx->memory.addr_ != nullptr)
    {
      REQUIRE(Put(new HPMUSBEndpoint(*this, number, Endpoint::Direction::IN, *tx)) ==
              ErrorCode::OK);
    }
    number = Endpoint::NextEPNumber(number);
  }
}

bool HPMUSBController::FlushAll()
{
  // SDK usb_dcd_bus_reset 的握手：等装入结束，全部 FLUSH，等 FLUSH 和 STAT 清零；FLUSH
  // 之后 STAT 仍为 1 的再 FLUSH。实测复位期间 8.36~8.48 us、挂起中 0.69~0.73 us。
  // The handshake of the SDK's usb_dcd_bus_reset: wait for primes, flush everything,
  // wait for FLUSH and STAT to clear; flush again what STAT still shows after a FLUSH.
  // Measured 8.36-8.48 us during a reset and 0.69-0.73 us while suspended.
  const uint64_t start = NowUs();
  while (regs_->ENDPTPRIME != 0U)
  {
    if (NowUs() - start > HANDSHAKE_US)
    {
      return false;
    }
  }
  regs_->ENDPTFLUSH = 0xFFFFFFFFUL;
  while (true)
  {
    const uint32_t flushing = regs_->ENDPTFLUSH;
    const uint32_t ready = regs_->ENDPTSTAT;
    if (flushing == 0U && ready == 0U)
    {
      return true;
    }
    if (flushing == 0U)
    {
      regs_->ENDPTFLUSH = ready;
    }
    if (NowUs() - start > HANDSHAKE_US)
    {
      return false;
    }
  }
}

void HPMUSBController::StartPollTimer()
{
  // 控制器自带的 GPTIMER0，重复模式；它的中断就是本控制器的中断，owner 在里面查未结束的
  // 停止。清 TI0 只写这一位。
  // The controller's own GPTIMER0 in repeat mode; its interrupt is this controller's,
  // in which the owner checks the pending stops. Clearing TI0 writes that bit only.
  regs_->GPTIMER0LD = POLL_US - 1U;
  regs_->GPTIMER0CTRL = USB_GPTIMER0CTRL_GPTRUN_MASK | USB_GPTIMER0CTRL_GPTRST_MASK |
                        USB_GPTIMER0CTRL_GPTMODE_MASK;
  regs_->USBINTR |= USB_USBINTR_TIE0_MASK;
}

void HPMUSBController::StopPollTimer()
{
  regs_->USBINTR &= ~USB_USBINTR_TIE0_MASK;
  regs_->GPTIMER0CTRL = 0U;
  regs_->USBSTS = USB_USBSTS_TI0_MASK;
}

// ---------------------------------------------------------------------------
// Device
// ---------------------------------------------------------------------------

HPMUSBDevice::HPMUSBDevice(USB_Type* regs, uint32_t irq, bool high_speed,
                           const std::initializer_list<HPMUSBEndpointMemory>& rx_ep_cfgs,
                           const std::initializer_list<HPMUSBEndpointMemory>& tx_ep_cfgs,
                           uint16_t vid, uint16_t pid, uint16_t bcd,
                           const Languages& languages, const Configs& configs,
                           ConstRawData uid)
    : HPMUSBController(regs, irq, high_speed, rx_ep_cfgs, tx_ep_cfgs),
      USB::DeviceCore(*this, *this, USB::USBSpec::USB_2_1,
                      high_speed ? USB::Speed::HIGH : USB::Speed::FULL,
                      USB::DeviceDescriptor::PacketSize0::SIZE_64, vid, pid, bcd,
                      languages, configs, uid)
{
  REQUIRE(device_ == nullptr);
  for (uint8_t i = 0; i < MAX_ENDPOINTS; ++i)
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
  intc_m_disable_irq(irq_);
  intc_set_irq_priority(irq_, 1U);
  // 调试器在中断中途停住的旧程序会把本源留在“已 claim 未 complete”，之后不再报告；补一次
  // complete（实测）。
  // An old program halted by the debugger in the middle of the interrupt leaves this
  // source claimed and not completed, after which it never fires; one complete fixes it
  // (measured).
  intc_m_complete_irq(irq_);
  device_ = this;
}

void HPMUSBDevice::Dispatch(USB_Type* regs)
{
  HPMUSBDevice* device = device_;
  if (device == nullptr || device->regs_ != regs)
  {
    if (regs != nullptr)
    {
      regs->USBINTR = 0U;
    }
    return;
  }
  device->OnControllerInterrupt();
}

void HPMUSBDevice::StartController(bool in_isr)
{
  UNUSED(in_isr);
  // 断开时间未到时只记下"要接上"并使能中断线，到点后 owner 接上。
  // While the detach time has not passed only "attach wanted" is recorded and the line
  // enabled; the owner attaches once the time has passed.
  if (detach_.WantAttach())
  {
    MaskInterrupt();
    reset_pending_ = false;
    suspended_ = false;
    Attach();
  }
  UnmaskInterrupt();
}

void HPMUSBDevice::Attach()
{
  regs_->USBSTS = regs_->USBSTS;
  regs_->USBINTR = INTERRUPTS;
  usb_dcd_connect(regs_);
}

void HPMUSBDevice::StopController(bool in_isr)
{
  UNUSED(in_isr);
  HaltController();
  regs_->USBINTR = 0U;
  (void)detach_.OnHalted(false);
}

void HPMUSBDevice::MaskInterrupt()
{
  intc_m_disable_irq(irq_);
  Fence();
}

void HPMUSBDevice::UnmaskInterrupt() { intc_m_enable_irq(irq_); }

bool HPMUSBDevice::PendInterrupt()
{
  __plic_set_irq_pending(HPM_PLIC_BASE, irq_);
  return true;
}

void HPMUSBDevice::SetAddress(uint8_t address, AddressStage stage)
{
  // USBADRA：地址先暂存，EP0 的状态 IN 被 ACK 时才生效。
  // USBADRA: the address is staged and takes effect when EP0's status IN is ACKed.
  if (stage == AddressStage::BEFORE_STATUS)
  {
    regs_->DEVICEADDR = USB_DEVICEADDR_USBADR_SET(address) | USB_DEVICEADDR_USBADRA_MASK;
  }
}

void HPMUSBDevice::HaltController()
{
  // 断开之后总线上没有包，FlushAll 只取决于芯片。硬件上不再有请求：未结束的停止作废、不再
  // 报告，之后每个 StopHardware 立即返回 ABORTED。
  // Detached, no packet is on the bus and FlushAll depends on the chip alone. The
  // hardware holds no request any more: pending stops are void and reported no more, and
  // every later StopHardware returns ABORTED at once.
  regs_->USBCMD &= ~USB_USBCMD_RS_MASK;
  (void)FlushAll();
  regs_->ENDPTSETUPSTAT = regs_->ENDPTSETUPSTAT;
  regs_->ENDPTCOMPLETE = regs_->ENDPTCOMPLETE;
  regs_->ENDPTNAK = regs_->ENDPTNAK;
  for (uint8_t i = 0; i < MAX_ENDPOINTS; ++i)
  {
    for (HPMUSBEndpoint* ep : {in_[i], out_[i]})
    {
      if (ep != nullptr)
      {
        ep->queued_ = 0U;
        ep->further_count_ = 0U;
        ep->further_next_ = 0U;
        ep->stopping_ = false;
        ep->self_stop_ = false;
        ep->stop_failed_ = false;
        ep->ResetOverlay();
      }
    }
  }
  pending_stops_ = 0U;
  StopPollTimer();
  reset_pending_ = false;
  suspended_ = false;
}

void HPMUSBDevice::ServiceHardware(bool in_isr)
{
  // 断开时间已到、要接上：先接上，再读这一轮的快照。
  // The detach time has passed and an attach is wanted: attach first, then take this
  // pass's snapshot.
  if (detach_.TakeAttach())
  {
    Attach();
  }
  // 一次快照。先清读到的 USBSTS 位，再读 ENDPTCOMPLETE 和 ENDPTSETUPSTAT：之后才到的
  // 完成会再置 UI，不会丢。顺序：总线复位、复位结束、端点完成（同号先 IN 后 OUT）、
  // 未结束的停止、SETUP、恢复、挂起、帧起始。
  // One snapshot. The USBSTS bits read are cleared before ENDPTCOMPLETE and
  // ENDPTSETUPSTAT are read: a completion arriving later sets UI again and is not lost.
  // Order: bus reset, end of the reset, endpoint completions (IN before OUT per number),
  // pending stops, SETUP, resume, suspend, start of frame.
  const uint32_t status = regs_->USBSTS & regs_->USBINTR;
  regs_->USBSTS = status;
  if ((status & USB_USBSTS_URI_MASK) != 0U)
  {
    OnUsbReset(in_isr);
  }
  if (reset_pending_ && (regs_->PORTSC1 & USB_PORTSC1_PR_MASK) == 0U)
  {
    ReportReset(in_isr);
  }
  if (reset_pending_)
  {
    return;
  }
  regs_->ENDPTCOMPLETE = regs_->ENDPTCOMPLETE;
  HandleCompletions(in_isr);
  if (pending_stops_ != 0U)
  {
    AdvanceStops(in_isr);
  }
  HandleSetup(in_isr);
  if ((status & (USB_USBSTS_PCI_MASK | USB_USBSTS_SRI_MASK)) != 0U && suspended_ &&
      (regs_->PORTSC1 & USB_PORTSC1_SUSP_MASK) == 0U)
  {
    suspended_ = false;
    OnResume(in_isr);
  }
  if ((status & USB_USBSTS_SLI_MASK) != 0U && !suspended_ &&
      (regs_->PORTSC1 & USB_PORTSC1_SUSP_MASK) != 0U)
  {
    // 挂起期间停止照常结束：没有 SOF，GPTIMER0 的中断照样进来。
    // While suspended stops end as usual: without SOFs the GPTIMER0 interrupt still
    // comes.
    suspended_ = true;
    OnSuspend(in_isr);
  }
  if ((status & USB_USBSTS_SRI_MASK) != 0U && !suspended_)
  {
    ReportFrame(in_isr);
  }
}

void HPMUSBDevice::OnUsbReset(bool in_isr)
{
  // 控制器在复位时已清掉非 EP0 端点的使能，但不结束传输（实测 URI 后 20 us ENDPTSTAT 仍
  // 为 1）。这里在复位持续期内做 SDK usb_dcd_bus_reset 的握手（实测 8.4 us，结束时 PR 仍
  // 为 1），结束未结束的停止，再清地址和事件。速度到端口回到工作状态时才确定，复位在那时
  // 报告。
  // At the reset the controller has cleared the enables of every endpoint but EP0, but
  // ends no transfer (measured: ENDPTSTAT still 1 20 us after URI). The handshake of the
  // SDK's usb_dcd_bus_reset runs here within the reset (measured 8.4 us, PR still 1 at
  // its end), pending stops end, then the address and the events are cleared. The speed
  // is settled only when the port is operational again, and the reset is reported then.
  for (uint8_t i = 1U; i < MAX_ENDPOINTS; ++i)
  {
    regs_->ENDPTCTRL[i] =
        (TYPE_BULK << CTRL_TYPE_SHIFT) | ((TYPE_BULK << CTRL_TYPE_SHIFT) << IN_SHIFT);
  }
  regs_->ENDPTNAK = regs_->ENDPTNAK;
  regs_->ENDPTNAKEN = 0U;
  regs_->ENDPTSETUPSTAT = regs_->ENDPTSETUPSTAT;
  regs_->ENDPTCOMPLETE = regs_->ENDPTCOMPLETE;
  FinishPendingStops(in_isr);
  regs_->DEVICEADDR = 0U;
  reset_pending_ = true;
  suspended_ = false;
}

void HPMUSBDevice::ReportReset(bool in_isr)
{
  reset_pending_ = false;
  const uint32_t speed = USB_PORTSC1_PSPD_GET(regs_->PORTSC1);
  OnBusReset(in_isr, speed == PSPD_HIGH ? USB::Speed::HIGH : USB::Speed::FULL);
}

void HPMUSBDevice::ReportFrame(bool in_isr)
{
  // FRINDEX：位 13:3 帧号，位 2:0 微帧（全速下不用）。
  // FRINDEX: bits 13:3 the frame number, bits 2:0 the microframe (unused at full speed).
  const uint32_t index = regs_->FRINDEX & USB_FRINDEX_FRINDEX_MASK;
  const auto frame = static_cast<uint16_t>((index >> 3U) & 0x7FFU);
  if (GetSpeed() == USB::Speed::HIGH)
  {
    OnStartOfFrame(in_isr, frame, static_cast<uint8_t>(index & 7U));
  }
  else
  {
    OnStartOfFrame(in_isr, frame);
  }
}

void HPMUSBDevice::HandleCompletions(bool in_isr)
{
  // ENDPTCOMPLETE 每端点只有一位，同一端点上几个 dTD 先后退役会并成一次，所以不看它：
  // 每个硬件上有请求的端点都按 dTD 令牌查一遍（它的位由 ServiceHardware 清掉）。同号先
  // IN 后 OUT。
  // ENDPTCOMPLETE has one bit per endpoint, and several dTDs of one endpoint retiring in
  // turn merge into one, so it is not read: every endpoint with requests on the hardware
  // is checked by its dTD tokens (ServiceHardware clears its bits). IN before OUT for
  // each number.
  for (uint8_t num = 0; num < MAX_ENDPOINTS; ++num)
  {
    for (HPMUSBEndpoint* ep : {in_[num], out_[num]})
    {
      if (ep != nullptr && ep->queued_ != 0U)
      {
        ep->Complete(in_isr);
      }
    }
  }
}

void HPMUSBDevice::HandleSetup(bool in_isr)
{
  if ((regs_->ENDPTSETUPSTAT & 1U) == 0U)
  {
    return;
  }
  // 收到 SETUP 时控制器自己清掉 EP0 两个方向的 ENDPTSTAT（实测 <=0.47 us），不回写 dTD。
  // 等它清零，再按顺序约定先报告 EP0 上已退役的请求、结束 EP0 上未结束的停止。
  // At a SETUP the controller clears ENDPTSTAT of both EP0 directions itself (measured
  // at most 0.47 us) and writes no dTD back. Wait for that, then, by the ordering
  // contract, report EP0 requests that retired and end EP0's pending stops first.
  (void)Spin([this]() { return (regs_->ENDPTSTAT & EP0_BITS) == 0U; });
  for (HPMUSBEndpoint* ep : {in_[0], out_[0]})
  {
    if (ep == nullptr || ep->queued_ == 0U || ep->stop_failed_)
    {
      continue;
    }
    if (ep->stopping_)
    {
      if (!ep->FinishStop(in_isr))
      {
        // 这个停止还在等 FLUSH：SETUP 留到之后的检查再报告（GPTIMER0 在运行）。
        // This stop still waits for its FLUSH: the SETUP is left to a later check
        // (GPTIMER0 is running).
        return;
      }
      continue;
    }
    ep->Complete(in_isr);
  }
  // Setup tripwire（USBMODE.SLOM=1）：清 SETUPSTAT，置 SUTW，拷出 SETUP，SUTW 仍为 1 才算
  // 拷完整；期间又来了 SETUP 就再拷一次（SDK hpm_usb_regs.h USBCMD.SUTW）。
  // Setup tripwire (USBMODE.SLOM=1): clear SETUPSTAT, set SUTW, copy the SETUP; the copy
  // is whole only if SUTW is still 1; a SETUP arriving meanwhile is copied again (SDK
  // hpm_usb_regs.h USBCMD.SUTW).
  uint32_t words[2] = {};
  const QueueHead& qh = qh_[QueueIndex(0U, false)];
  for (uint32_t round = 0; round < RETRY_LIMIT; ++round)
  {
    regs_->ENDPTSETUPSTAT = 1U;
    for (uint32_t copy = 0; copy < RETRY_LIMIT; ++copy)
    {
      regs_->USBCMD |= USB_USBCMD_SUTW_MASK;
      Fence();
      words[0] = qh.setup[0];
      words[1] = qh.setup[1];
      Fence();
      if ((regs_->USBCMD & USB_USBCMD_SUTW_MASK) != 0U)
      {
        break;
      }
    }
    regs_->USBCMD &= ~USB_USBCMD_SUTW_MASK;
    if ((regs_->ENDPTSETUPSTAT & 1U) == 0U)
    {
      break;
    }
  }
  USB::SetupPacket packet = {};
  static_assert(sizeof(packet) == sizeof(words), "SETUP is 8 bytes");
  std::memcpy(&packet, words, sizeof(packet));
  OnSetup(in_isr, packet);
}

void HPMUSBDevice::FinishPendingStops(bool in_isr)
{
  // 在途但没有被停的请求也一并清掉装入：它们随后由复位报告引起的 Close 当场结束。
  // Requests in flight but not stopped lose their primes too: the Close caused by the
  // reset report ends them at once afterwards.
  const bool flushed = FlushAll();
  if (pending_stops_ == 0U)
  {
    return;
  }
  for (uint8_t i = 0; i < MAX_ENDPOINTS; ++i)
  {
    for (HPMUSBEndpoint* ep : {in_[i], out_[i]})
    {
      if (ep != nullptr && ep->stopping_ && !ep->stop_failed_ &&
          !(flushed && ep->FinishStop(in_isr)))
      {
        ep->stop_failed_ = true;
        ep->OnHardwareComplete(in_isr, 0U, ErrorCode::TIMEOUT);
      }
    }
  }
}

void HPMUSBDevice::AdvanceStops(bool in_isr)
{
  const uint64_t now = NowUs();
  for (uint8_t i = 0; i < MAX_ENDPOINTS; ++i)
  {
    for (HPMUSBEndpoint* ep : {in_[i], out_[i]})
    {
      if (ep == nullptr || !ep->stopping_ || ep->stop_failed_ || ep->FinishStop(in_isr))
      {
        continue;
      }
      const uint32_t bit = ep->Bit();
      const uint64_t age = now - ep->stop_at_us_;
      if ((regs_->ENDPTFLUSH & bit) == 0U && (regs_->ENDPTSTAT & bit) != 0U)
      {
        // FLUSH 已清而 STAT 仍为 1：装入与 FLUSH 相撞，按手册再 FLUSH（SDK
        // usb_dcd_edpt_close 的循环）。
        // FLUSH cleared but STAT still 1: a prime met the FLUSH; flush again as the
        // manual says (the loop of the SDK's usb_dcd_edpt_close).
        regs_->ENDPTFLUSH = bit;
      }
      else if (age >= HPMUSBEndpoint::STOP_REFLUSH_US && !ep->reflushed_)
      {
        ep->reflushed_ = true;
        regs_->ENDPTFLUSH = bit;
      }
      else if (age >= HPMUSBEndpoint::STOP_TIMEOUT_US && !ep->self_stop_)
      {
        ep->stop_failed_ = true;
        ep->OnHardwareComplete(in_isr, 0U, ErrorCode::TIMEOUT);
      }
    }
  }
}

// ---------------------------------------------------------------------------
// Endpoint
// ---------------------------------------------------------------------------

uint8_t HPMUSBEndpoint::SlotsFor(EPNumber number, uint8_t slots)
{
  if (slots != 0U)
  {
    return slots;
  }
  return number == EPNumber::EP0 ? 1U : DEFAULT_SLOTS;
}

HPMUSBEndpoint::HPMUSBEndpoint(HPMUSBController& controller, EPNumber number,
                               Direction direction, const HPMUSBEndpointMemory& memory)
    : Endpoint(number, direction, memory.memory, SlotsFor(number, memory.slots)),
      controller_(controller),
      slots_(SlotsFor(number, memory.slots))
{
  const uint8_t num = EPNumberToInt8(number);
  REQUIRE(num < HPMUSBController::MAX_ENDPOINTS);
  REQUIRE(slots_ <= MAX_SLOTS);
  // 每个槽一个 dTD，从池里连续取。 / One dTD per slot, taken as a run from the pool.
  REQUIRE(controller.td_used_ + slots_ <= HPMUSBController::TD_POOL);
  td_ = &controller.td_[controller.td_used_];
  controller.td_used_ = static_cast<uint8_t>(controller.td_used_ + slots_);
  if (direction == Direction::IN)
  {
    controller.in_[num] = this;
  }
  else
  {
    controller.out_[num] = this;
  }
}

Endpoint::HwLimits HPMUSBEndpoint::HardwareLimits(Direction direction, Type type) const
{
  UNUSED(direction);
  HwLimits limits;
  limits.direct_memory = true;
  switch (type)
  {
    case Type::CONTROL:
      limits.max_packet_size = EP0_PACKET;
      limits.max_packets = 1U;
      limits.max_bytes = EP0_PACKET;
      break;
    case Type::ISOCHRONOUS:
      limits.max_packet_size = 1024U;
      limits.max_packets = ISO_PACKETS_PER_INTERVAL;
      limits.max_bytes = ISO_PACKETS_PER_INTERVAL * 1024U;
      limits.max_requests = slots_;
      limits.max_packets_per_interval = ISO_PACKETS_PER_INTERVAL;
      break;
    case Type::BULK:
      limits.max_packet_size = 1024U;
      limits.max_bytes = MAX_BYTES;
      limits.max_requests = slots_;
      break;
    case Type::INTR:
      limits.max_packet_size = 1024U;
      limits.max_bytes = MAX_BYTES;
      limits.max_requests = slots_;
      limits.max_packets_per_interval = INTR_PACKETS_PER_INTERVAL;
      break;
  }
  return limits;
}

ErrorCode HPMUSBEndpoint::CheckHardware(const Config& cfg) const
{
  if ((cfg.type == Type::CONTROL) != (Num() == 0U))
  {
    return ErrorCode::NOT_SUPPORT;
  }
  return ErrorCode::OK;
}

void HPMUSBEndpoint::ResetOverlay()
{
  auto& qh = Qh();
  qh.next = NEXT_TERMINATE;
  qh.token = 0U;
  for (uint8_t i = 0U; i < slots_; ++i)
  {
    Td(i).token = 0U;
  }
}

void HPMUSBEndpoint::ConfigureHardware(const Config& cfg)
{
  const uint8_t num = Num();
  uint32_t caps =
      (static_cast<uint32_t>(cfg.max_packet_size) << CAP_SIZE_SHIFT) | CAP_ZLT_OFF;
  if (cfg.type == Type::ISOCHRONOUS)
  {
    caps |= static_cast<uint32_t>(cfg.max_packets_per_interval) << CAP_MULT_SHIFT;
  }
  if (num == 0U && !IsIn())
  {
    caps |= CAP_IOS;
  }
  Qh().capabilities = caps;
  ResetOverlay();
  if (num == 0U)
  {
    // EP0 总是使能的控制端点。 / EP0 is always an enabled control endpoint.
    return;
  }
  const uint32_t shift = CtrlShift();
  const uint32_t other = IsIn() ? 0U : HPMUSBController::IN_SHIFT;
  uint32_t value = Regs()->ENDPTCTRL[num];
  value &= ~((CTRL_TYPE_MASK | CTRL_STALL) << shift);
  value |= ((TypeCode(cfg.type) << CTRL_TYPE_SHIFT) | CTRL_ENABLE | CTRL_TOGGLE_RESET)
           << shift;
  if (((value >> other) & CTRL_TYPE_MASK) == 0U)
  {
    value |= (TYPE_BULK << CTRL_TYPE_SHIFT) << other;
  }
  Regs()->ENDPTCTRL[num] = value;
}

void HPMUSBEndpoint::CloseHardware(bool in_isr)
{
  UNUSED(in_isr);
  const uint8_t num = Num();
  if (num == 0U)
  {
    return;
  }
  Regs()->ENDPTCTRL[num] &= ~(CTRL_STALL << CtrlShift());
}

bool HPMUSBEndpoint::Quiescent() const
{
  const uint32_t bit = Bit();
  auto* regs = Regs();
  return ((regs->ENDPTFLUSH | regs->ENDPTSTAT | regs->ENDPTPRIME) & bit) == 0U;
}

bool HPMUSBEndpoint::SpinQuiescent() const
{
  return Spin([this]() { return Quiescent(); });
}

ErrorCode HPMUSBEndpoint::StartHardware(const HwRequest& request, bool in_isr)
{
  UNUSED(in_isr);
  ASSERT(queued_ < slots_ && !stopping_);
  ASSERT(request.length <= MAX_BYTES);
  const uint8_t num = Num();
  auto* regs = Regs();
  const uint32_t bit = Bit();
  if (num != 0U && (regs->ENDPTCTRL[num] & (CTRL_ENABLE << CtrlShift())) == 0U)
  {
    // 总线复位清掉了使能；复位后、重新配置之前不启动。
    // A bus reset cleared the enable; nothing starts between it and the reconfiguration.
    return ErrorCode::STATE_ERR;
  }
  if (num == 0U && (regs->ENDPTSETUPSTAT & 1U) != 0U)
  {
    // 有还没有报告的 SETUP：这个应答属于被取代的传输，不装入。
    // A SETUP not reported yet: this reply belongs to the superseded transfer.
    return ErrorCode::FAILED;
  }
  const uint8_t which = RingAt(head_, queued_);
  auto& td = Td(which);
  const uint32_t address = DmaAddress(request.data);
  td.next = NEXT_TERMINATE;
  td.length = static_cast<uint32_t>(request.length);
  td.buffer[0] = address;
  const uint32_t page = address & ~static_cast<uint32_t>(PAGE - 1U);
  for (uint32_t i = 1U; i < 5U; ++i)
  {
    td.buffer[i] = page + i * PAGE;
  }
  td.token = (static_cast<uint32_t>(request.length) << TOKEN_BYTES_SHIFT) | TOKEN_IOC |
             TOKEN_ACTIVE;
  if (IsIn() && GetConfig().type == Type::ISOCHRONOUS)
  {
    // 等时 IN 的 dTD 写 MultO = 这个区间实际的包数（长度 0 为一个零长度包）：只靠 dQH 的
    // MULT 时，包数少于 MULT 的请求主机一个也收不到，端点随后停住（实测 MultO 为 0 时长度
    // 0、1、1024、1025、2048 都是这样）。有了 MultO，PID 按实际包数排，各长度主机都收对。
    // An isochronous IN dTD gets MultO = the actual packets of this interval (one
    // zero-length packet for length 0): with the dQH MULT alone the host receives
    // nothing of a request with fewer packets than MULT and the endpoint sends no more
    // (measured with MultO 0 for lengths 0, 1, 1024, 1025 and 2048). With MultO the PIDs
    // follow the actual count and the host receives every length right.
    const size_t packet = GetConfig().max_packet_size;
    const size_t packets =
        request.length == 0U ? 1U : (request.length + packet - 1U) / packet;
    td.token |= static_cast<uint32_t>(packets) << TOKEN_MULTO_SHIFT;
  }
  if (queued_ != 0U)
  {
    Append(td);
    return ErrorCode::OK;
  }
  head_ = which;
  Prime(td);
  if (num != 0U || (regs->ENDPTSETUPSTAT & 1U) == 0U)
  {
    return ErrorCode::OK;
  }
  // 装入前后到了 SETUP：撤回。控制器收到 SETUP 时自己清 EP0 的装入（实测不超过
  // 0.47 us），先等它；装入晚于 SETUP 被处理时再 FLUSH。都没有在芯片的上限内完成就按
  // 后端自己的停止处理，在报告那个 SETUP 之前结束，报 ABORTED。
  // A SETUP arrived around the load: retract it. The controller clears EP0's prime
  // itself at a SETUP (measured at most 0.47 us), so that is awaited first; a prime
  // handled after the SETUP is flushed. If neither finishes within the chip's bound
  // this becomes a stop issued by the backend, ended before that SETUP is reported and
  // reported as ABORTED.
  bool gone = SpinQuiescent();
  if (!gone)
  {
    regs->ENDPTFLUSH = bit;
    gone = SpinQuiescent();
  }
  if (gone && (td.token & TOKEN_ACTIVE) != 0U)
  {
    ClearRequests();
    return ErrorCode::FAILED;
  }
  if (!gone)
  {
    self_stop_ = true;
    BeginPending();
  }
  // dTD 已退役：旧应答的包在 SETUP 之前已上线，按普通完成在 ServiceHardware 里报告。
  // The dTD retired: the old reply went out before the SETUP and is reported as a plain
  // completion in ServiceHardware.
  return ErrorCode::OK;
}

void HPMUSBEndpoint::Prime(HPMUSBController::TransferDescriptor& td)
{
  // 覆盖区清 Active/Halted 并指向新 dTD：不清的话控制器接着跑旧覆盖区（实测缺 121 处、
  // 重 42 处）；剩余字节写成请求长度、当前 dTD 清零，供停止时计数（还没装入时算出 0）。
  // 本端点此刻没有装入，写覆盖区不会与硬件冲突。上一个请求在报告之后才置的完成位在这里
  // 清掉。
  // The overlay gets Active and Halted cleared and points at the new dTD: otherwise the
  // controller runs on the old overlay (measured 121 gaps and 42 repeats); its bytes left
  // are set to the request length and the current dTD to 0 for counting at a stop (0
  // comes out before the load). Nothing is primed on this endpoint now, so the overlay
  // write meets no hardware update. A completion bit of the previous request set after
  // its report is cleared here.
  auto* regs = Regs();
  const uint32_t bit = Bit();
  auto& qh = Qh();
  qh.current = 0U;
  qh.token = td.length << TOKEN_BYTES_SHIFT;
  qh.next = DmaAddress(&td);
  regs->ENDPTCOMPLETE = bit;
  Fence();
  queued_ = 1U;
  regs->ENDPTPRIME = bit;
}

void HPMUSBEndpoint::Append(HPMUSBController::TransferDescriptor& td)
{
  // 往已装入的端点追加（SDK hpm_usb_regs.h USBCMD.ATDTW，即手册的 add dTD tripwire
  // 步骤）：先把新 dTD 接到最后一个后面；装入还在进行（PRIME 为 1）就结束；否则置
  // ATDTW、读 ENDPTSTAT、确认 ATDTW 仍为 1（控制器在危险区会清它，清了就重来），再清
  // ATDTW。读到的 STAT 为 1 说明控制器还在这条链上，会接着做新 dTD；为 0 说明更早的都
  // 已退役、链已经停了，新 dTD 按空端点重新装入。本端点的完成位不清：更早的请求的完成
  // 可能刚刚置位。
  // Appending to a primed endpoint (USBCMD.ATDTW in the SDK's hpm_usb_regs.h, the
  // manual's add-dTD-tripwire procedure): link the new dTD behind the last one; done if
  // the prime is still in progress (PRIME 1); otherwise set ATDTW, read ENDPTSTAT and
  // check that ATDTW is still 1 (the controller clears it in its hazard region, and then
  // this repeats), then clear ATDTW. STAT read as 1 means the controller is still on the
  // list and goes on to the new dTD; 0 means the earlier ones retired and the list
  // stopped, so the new dTD is primed as on an empty endpoint. This endpoint's completion
  // bit is not cleared: an earlier request's completion may have just set it.
  auto* regs = Regs();
  const uint32_t bit = Bit();
  auto& previous = Td(RingAt(head_, static_cast<uint8_t>(queued_ - 1U)));
  Fence();
  previous.next = DmaAddress(&td);
  Fence();
  if ((regs->ENDPTPRIME & bit) != 0U)
  {
    ++queued_;
    return;
  }
  bool ready = false;
  for (uint32_t round = 0U; round < RETRY_LIMIT * 4U; ++round)
  {
    regs->USBCMD |= USB_USBCMD_ATDTW_MASK;
    ready = (regs->ENDPTSTAT & bit) != 0U;
    if ((regs->USBCMD & USB_USBCMD_ATDTW_MASK) != 0U)
    {
      break;
    }
  }
  regs->USBCMD &= ~USB_USBCMD_ATDTW_MASK;
  if (ready)
  {
    ++queued_;
    return;
  }
  // 链已经停了：更早的都已退役（控制器按链的顺序做，它们的完成按先后另行报告），新 dTD
  // 按空端点装入。覆盖区写给新 dTD，更早的结果只看它们自己的令牌。
  // The list has stopped: the earlier ones have all retired (the controller works down
  // the list in order; their completions are reported in order on their own), and the
  // new dTD is primed as on an empty endpoint. The overlay is written for the new dTD;
  // the earlier ones' results come from their own tokens only.
  auto& qh = Qh();
  qh.current = 0U;
  qh.token = td.length << TOKEN_BYTES_SHIFT;
  qh.next = DmaAddress(&td);
  Fence();
  ++queued_;
  regs->ENDPTPRIME = bit;
}

Endpoint::StopOutcome HPMUSBEndpoint::OutcomeOf(uint8_t which) const
{
  const auto& td = Td(which);
  const uint32_t token = td.token;
  const size_t length = td.length;
  if ((token & TOKEN_ACTIVE) == 0U)
  {
    const size_t left = BytesLeft(token);
    const size_t moved = length > left ? length - left : 0U;
    return {(token & TOKEN_ERRORS) != 0U ? ErrorCode::FAILED : ErrorCode::OK, moved};
  }
  // dTD 没有退役：它在内存里不变，已传的字节只在覆盖区（实测），而且只在覆盖区正装着它
  // （当前 dTD 是它）时才算；启动时覆盖区的剩余字节写成请求长度、当前 dTD 清零，所以
  // 还没装入时算出 0。最后一包已 ACK、退役写回之前被 FLUSH 停住时覆盖区剩余为 0、Active
  // 可能已清：照覆盖区算，整个做完就报 OK。
  // The dTD has not retired: it is unchanged in memory and the bytes moved are in the
  // overlay only (measured), and only while the overlay holds it (it is the current
  // dTD); the overlay's bytes left are set to the request length and the current dTD to
  // 0 at the start, so 0 comes out before the load. When the FLUSH caught it after its
  // last packet was ACKed and before the retirement was written back, the overlay shows
  // 0 left and may have Active cleared: it is taken as is, and a whole transfer is
  // reported as OK.
  const auto& qh = Qh();
  if (qh.current != DmaAddress(&td))
  {
    return {ErrorCode::ABORTED, 0U};
  }
  const size_t left = BytesLeft(qh.token);
  const size_t moved = length > left ? length - left : 0U;
  if (length != 0U && moved == length)
  {
    return {ErrorCode::OK, moved};
  }
  return {ErrorCode::ABORTED, moved};
}

void HPMUSBEndpoint::ClearRequests()
{
  queued_ = 0U;
  further_count_ = 0U;
  further_next_ = 0U;
  ResetOverlay();
  EndPending();
}

void HPMUSBEndpoint::Complete(bool in_isr)
{
  // 按先后：最早那个 dTD 退役了才报告它，再看下一个。报告的回调里可以再提交、停止或
  // 关闭本端点，所以每次都重新看状态。
  // In order: the earliest dTD is reported once it retired, then the next is looked at.
  // The callback of a report may submit, stop or close on this endpoint, so the state is
  // read again each time.
  while (queued_ != 0U && !stop_failed_)
  {
    if ((Td(head_).token & TOKEN_ACTIVE) != 0U)
    {
      return;
    }
    if (stopping_)
    {
      // 请求在停止未结束时自己退役：FLUSH 也清了才结束停止，
      // 免得留着的 FLUSH 清掉下一次装入。
      // The request retired by itself while its stop is pending: the stop ends only
      // once the FLUSH has cleared too, so that a lingering FLUSH cannot clear the next
      // prime.
      (void)FinishStop(in_isr);
      return;
    }
    const StopOutcome outcome = OutcomeOf(head_);
    Td(head_).token = 0U;
    head_ = RingAt(head_, 1U);
    --queued_;
    OnHardwareComplete(in_isr, outcome.actual, outcome.result);
  }
}

void HPMUSBEndpoint::BeginPending()
{
  if (stopping_)
  {
    return;
  }
  stopping_ = true;
  reflushed_ = false;
  stop_at_us_ = NowUs();
  if (controller_.pending_stops_++ == 0U)
  {
    controller_.StartPollTimer();
  }
}

void HPMUSBEndpoint::EndPending()
{
  self_stop_ = false;
  stop_failed_ = false;
  if (!stopping_)
  {
    return;
  }
  stopping_ = false;
  if (--controller_.pending_stops_ == 0U)
  {
    controller_.StopPollTimer();
  }
}

uint8_t HPMUSBEndpoint::TakeOutcomes(StopOutcome (&outcomes)[MAX_SLOTS]) const
{
  const uint8_t count = queued_;
  for (uint8_t i = 0U; i < count; ++i)
  {
    outcomes[i] = OutcomeOf(RingAt(head_, i));
  }
  return count;
}

bool HPMUSBEndpoint::FinishStop(bool in_isr)
{
  ASSERT(stopping_);
  if (!Quiescent())
  {
    return false;
  }
  StopOutcome outcomes[MAX_SLOTS];
  const uint8_t count = TakeOutcomes(outcomes);
  const bool self = self_stop_;
  ClearRequests();
  // 按先后为硬件上的每个请求报告一次；后端自己发起的停止（EP0 撤回）没有核心在等，没做
  // 完时报 ABORTED。
  // One report per request that was on the hardware, in order; a stop the backend issued
  // itself (EP0 retract) has no core waiting, ABORTED if unfinished.
  for (uint8_t i = 0U; i < count; ++i)
  {
    const ErrorCode result = (self && outcomes[i].result != ErrorCode::OK)
                                 ? ErrorCode::ABORTED
                                 : outcomes[i].result;
    OnHardwareComplete(in_isr, outcomes[i].actual, result);
  }
  return true;
}

Endpoint::StopOutcome HPMUSBEndpoint::StopHardware(bool in_isr)
{
  UNUSED(in_isr);
  if (further_next_ < further_count_)
  {
    // 前面的调用已经停掉了硬件上的全部请求，这次给下一个的结果。
    // An earlier call stopped every request on the hardware; this one gives the next
    // outcome.
    const StopOutcome outcome = further_[further_next_++];
    if (further_next_ == further_count_)
    {
      further_count_ = 0U;
      further_next_ = 0U;
    }
    return outcome;
  }
  // 控制器停止或复位之后，硬件上已没有请求。
  // After the controller was halted or reset no request is on the hardware.
  if (queued_ == 0U)
  {
    return {ErrorCode::ABORTED, 0U};
  }
  if (stopping_)
  {
    // 后端自己发起的停止（EP0 撤回）还没结束：从这里起它算核心的停止。
    // A stop the backend issued (EP0 retract) has not ended: from here on it is the
    // core's.
    self_stop_ = false;
    return {ErrorCode::PENDING, 0U};
  }
  auto* regs = Regs();
  // 已经停稳：硬件上的 dTD 都退役了且没有装入；或 EP0 在 SETUP 时被控制器清掉了装入。
  // 数据端点还有 dTD 在活动时一律写 FLUSH：一个 dTD 退役、控制器转去下一个时 STAT 会
  // 短暂为 0（SDK 寄存器说明），不能只看 STAT。EP0 有未报告的 SETUP 时先等控制器清
  // （芯片定上限），不写 FLUSH。
  // Quiescent already: every dTD on the hardware retired with nothing primed, or the
  // controller cleared EP0's prime at a SETUP. A data endpoint with a dTD still active
  // always gets a FLUSH: STAT reads 0 for a moment while the controller moves from a
  // retired dTD to the next (SDK register notes), so STAT alone proves nothing. With a
  // SETUP not reported yet EP0 waits for the controller to clear it (chip-bounded) and
  // writes no FLUSH.
  bool active = false;
  for (uint8_t i = 0U; i < queued_; ++i)
  {
    active = active || (Td(RingAt(head_, i)).token & TOKEN_ACTIVE) != 0U;
  }
  bool quiescent = (!active || Num() == 0U) && Quiescent();
  if (!quiescent && Num() == 0U && (regs->ENDPTSETUPSTAT & 1U) != 0U)
  {
    quiescent = SpinQuiescent();
  }
  if (!quiescent)
  {
    // FLUSH 停掉本端点硬件上的全部请求，只与本端点有关；总线空闲时在芯片的时间内完成
    // （实测不超过 1.1 us），否则要等线上的包，这里不等，返回 PENDING。
    // The FLUSH stops every request of this endpoint on the hardware and concerns this
    // endpoint only; on an idle bus it finishes in chip time (measured at most 1.1 us);
    // otherwise it waits for packets on the wire, which is not waited for: PENDING.
    regs->ENDPTFLUSH = Bit();
    quiescent = SpinQuiescent();
  }
  if (!quiescent)
  {
    BeginPending();
    return {ErrorCode::PENDING, 0U};
  }
  StopOutcome outcomes[MAX_SLOTS];
  const uint8_t count = TakeOutcomes(outcomes);
  ClearRequests();
  for (uint8_t i = 1U; i < count; ++i)
  {
    further_[i - 1U] = outcomes[i];
  }
  further_count_ = static_cast<uint8_t>(count - 1U);
  return outcomes[0];
}

void HPMUSBEndpoint::WriteStall()
{
  const uint8_t num = Num();
  auto* regs = Regs();
  const uint32_t stall = CTRL_STALL << CtrlShift();
  // SETUPSTAT 置位期间和清零后约 50 个时钟内，控制器清 EP0 的 STALL：读回，没生效就重写，
  // 直到生效或又来了 SETUP（SDK hpm_usb_regs.h ENDPTCTRL TXS/RXS）。数据端点一次就生效。
  // While SETUPSTAT is set and for about 50 clocks after it is cleared the controller
  // clears EP0's STALL: read back and write again until it holds or a new SETUP has come
  // (ENDPTCTRL TXS/RXS in the SDK's hpm_usb_regs.h). Data endpoints take it at once.
  for (uint32_t i = 0; i < RETRY_LIMIT; ++i)
  {
    regs->ENDPTCTRL[num] |= stall;
    if ((regs->ENDPTCTRL[num] & stall) != 0U ||
        (num == 0U && (regs->ENDPTSETUPSTAT & 1U) != 0U))
    {
      return;
    }
  }
}

ErrorCode HPMUSBEndpoint::StallHardware(bool in_isr)
{
  UNUSED(in_isr);
  WriteStall();
  return ErrorCode::OK;
}

ErrorCode HPMUSBEndpoint::ClearStallHardware(bool in_isr)
{
  UNUSED(in_isr);
  const uint8_t num = Num();
  const uint32_t shift = CtrlShift();
  uint32_t value = Regs()->ENDPTCTRL[num] & ~(CTRL_STALL << shift);
  if (num != 0U)
  {
    // 批量、中断回到 DATA0（TXR/RXR 写 1）。 / Bulk and interrupt restart at DATA0.
    value |= CTRL_TOGGLE_RESET << shift;
  }
  Regs()->ENDPTCTRL[num] = value;
  return ErrorCode::OK;
}

// ---------------------------------------------------------------------------
// 中断向量 / Interrupt vector
// ---------------------------------------------------------------------------

#if defined(IRQn_USB0)
SDK_DECLARE_EXT_ISR_M(IRQn_USB0, libxr_hpm_usb0_isr)
void libxr_hpm_usb0_isr(void) { HPMUSBDevice::Dispatch(HPM_USB0); }
#endif
#if defined(IRQn_USB1) && defined(HPM_USB1)
SDK_DECLARE_EXT_ISR_M(IRQn_USB1, libxr_hpm_usb1_isr)
void libxr_hpm_usb1_isr(void) { HPMUSBDevice::Dispatch(HPM_USB1); }
#endif

void HPMUSBDevice::ResetController(bool in_isr)
{
  UNUSED(in_isr);
  // 断开要保持到主机看得见，否则主机不复位总线。这里只断开，不等：detach_ 按节拍计
  // RESET_DETACH_MS，到点后 owner 在 ServiceHardware 开头接上。
  // The detach has to last until the host sees it, or the host does not reset the bus.
  // Only the detach happens here, no wait: detach_ counts RESET_DETACH_MS on the tick
  // and the owner attaches at the start of ServiceHardware once it has passed.
  HaltController();
  if (detach_.OnHalted(true) == USB::DetachTimer::HaltResult::ATTACH_NOW)
  {
    Attach();
  }
}

#endif
