// NOLINTBEGIN(cppcoreguidelines-pro-type-cstyle-cast,performance-no-int-to-ptr)
#include <algorithm>
#include <cstring>

#include "ch32_usb_dev.hpp"
#include "ch32_usb_rcc.hpp"
#include "timebase.hpp"

using namespace LibXR;

#if defined(USBHSD)

// 中断属性只有 WCH 的 RISC-V 编译器认识；主机测试按普通函数编译。
// Only the WCH RISC-V compiler knows the interrupt attribute; host tests build a plain
// function.
#if defined(__riscv)
#define CH32_USB_IRQ __attribute__((interrupt("WCH-Interrupt-fast")))
#else
#define CH32_USB_IRQ
#endif

namespace
{

using Ep = CH32EndpointOtgHs;
using Endpoint = USB::Endpoint;

constexpr uint8_t OUT_IDX = static_cast<uint8_t>(Endpoint::Direction::OUT);
constexpr uint8_t IN_IDX = static_cast<uint8_t>(Endpoint::Direction::IN);

/// 没有对象时只清掉已锁存的标志 / Without an object, only clear the latched flags
void ClearPendingOtgHsInterrupts()
{
  while (true)
  {
    const uint8_t flags = USBHSD->INT_FG;
    if (flags == 0U)
    {
      break;
    }
    USBHSD->INT_FG = flags;
  }
}

}  // namespace

// NOLINTNEXTLINE(readability-identifier-naming)
extern "C" CH32_USB_IRQ void USBHS_IRQHandler(void)
{
  CH32USBOtgHS* usb = CH32USBOtgHS::self_;
  if (usb == nullptr)
  {
    ClearPendingOtgHsInterrupts();
    return;
  }
  usb->OnControllerInterrupt();
}

CH32USBOtgHS::CH32USBOtgHS(const std::initializer_list<RawData>& rx_ep_cfgs,
                           const std::initializer_list<RawData>& tx_ep_cfgs, uint16_t vid,
                           uint16_t pid, uint16_t bcd, const Languages& languages,
                           const Configs& configs, ConstRawData uid)
    : USB::DeviceCore(*this, AddEndpoints(*this, rx_ep_cfgs, tx_ep_cfgs),
                      USB::USBSpec::USB_2_1, USB::Speed::HIGH,
                      USB::DeviceDescriptor::PacketSize0::SIZE_64, vid, pid, bcd,
                      languages, configs, uid),
      tick_(TickCheck, TickDone, this)
{
  Ep::tick_ = &tick_;
}

bool CH32USBOtgHS::TickCheck(void* arg)
{
  UNUSED(arg);
  return true;
}

void CH32USBOtgHS::TickDone(void* arg, ErrorCode result)
{
  UNUSED(result);
  // 只发布事件、挂起 USB 中断，不碰 USB 寄存器；中断那一轮 ServiceHardware 末尾由
  // AgeStops 结束到期的停止，由 ReportFrame 报告帧起始。
  // Only publishes an event and pends the USB interrupt, touching no USB register; at
  // the end of that interrupt's ServiceHardware pass AgeStops ends the stops that are
  // due and ReportFrame reports the start of frame.
  static_cast<CH32USBOtgHS*>(arg)->RequestPoll(true);
}

USB::EndpointPool& CH32USBOtgHS::AddEndpoints(CH32USBOtgHS& self,
                                              const std::initializer_list<RawData>& rx,
                                              const std::initializer_list<RawData>& tx)
{
  const size_t count = rx.size() > tx.size() ? rx.size() : tx.size();
  REQUIRE(count > 0U && count <= Ep::EP_OTG_HS_MAX_SIZE);
  RawData rx_blocks[Ep::EP_OTG_HS_MAX_SIZE] = {};
  RawData tx_blocks[Ep::EP_OTG_HS_MAX_SIZE] = {};
  std::copy(rx.begin(), rx.end(), rx_blocks);
  std::copy(tx.begin(), tx.end(), tx_blocks);

  // EP0 两个方向共用 UEP0_DMA，所以是同一块。 / EP0's two directions share UEP0_DMA,
  // hence one block.
  REQUIRE(rx_blocks[0].addr_ != nullptr && rx_blocks[0].addr_ == tx_blocks[0].addr_ &&
          rx_blocks[0].size_ >= 64U);
  USB::EndpointPool& pool = self;
  auto* out0 = new Ep(Endpoint::EPNumber::EP0, Endpoint::Direction::OUT, rx_blocks[0]);
  auto* in0 = new Ep(Endpoint::EPNumber::EP0, Endpoint::Direction::IN, rx_blocks[0]);
  pool.SetEndpoint0(in0, out0);
  REQUIRE(self.Register(*out0) == ErrorCode::OK);
  REQUIRE(self.Register(*in0) == ErrorCode::OK);

  Endpoint::EPNumber number = Endpoint::EPNumber::EP1;
  for (size_t i = 1U; i < count; ++i, number = Endpoint::NextEPNumber(number))
  {
    const RawData blocks[2] = {rx_blocks[i], tx_blocks[i]};
    const Endpoint::Direction dirs[2] = {Endpoint::Direction::OUT,
                                         Endpoint::Direction::IN};
    for (size_t d = 0U; d < 2U; ++d)
    {
      if (blocks[d].addr_ == nullptr)
      {
        continue;
      }
      REQUIRE(blocks[d].size_ >= 64U);
      auto* ep = new Ep(number, dirs[d], blocks[d]);
      REQUIRE(pool.Put(ep) == ErrorCode::OK);
      REQUIRE(self.Register(*ep) == ErrorCode::OK);
    }
  }
  return pool;
}

USB::Speed CH32USBOtgHS::ReadBusSpeed()
{
  return (USBHSD->SPEED_TYPE & USBHS_USB_SPEED_TYPE) == USBHS_USB_SPEED_HIGH
             ? USB::Speed::HIGH
             : USB::Speed::FULL;
}

void CH32USBOtgHS::ResetEp0(uint8_t rx_res)
{
  Ep::map_[0][OUT_IDX]->tog0_ = true;
  Ep::map_[0][IN_IDX]->tog0_ = true;
  USBHSD->UEP0_TX_CTRL = USBHS_UEP_T_TOG_DATA1 | USBHS_UEP_T_RES_NAK;
  USBHSD->UEP0_RX_CTRL = static_cast<uint8_t>(USBHS_UEP_R_TOG_DATA1 | rx_res);
}

void CH32USBOtgHS::ServiceHardware(bool in_isr)
{
  ++Ep::pass_;
  // 断开时间已到、要接上：先接上，再照常处理这一轮。
  // The detach time has passed and an attach is wanted: attach first, then handle the
  // pass as usual.
  if (detach_.TakeAttach())
  {
    SetUpController();
  }
  while (true)
  {
    const uint16_t fgst = *reinterpret_cast<volatile uint16_t*>(
        reinterpret_cast<uintptr_t>(&USBHSD->INT_FG));
    const uint8_t flags = static_cast<uint8_t>(fgst & 0x00FFU);
    const uint8_t int_st = static_cast<uint8_t>(fgst >> 8U);
    if (flags == 0U)
    {
      break;
    }
    // 本轮看到的 UIF_TRANSFER 归事件循环，到本轮末尾才清；期间控制器对其他事务回忙
    // NAK，寄存器写入和完成报告都不会撞上新事务。
    // A UIF_TRANSFER seen in this round belongs to the event loop and is cleared at the
    // end of the round; meanwhile the controller busy-NAKs other transactions, so
    // register writes and reports cannot collide with a new one.
    const bool transfer = (flags & USBHS_UIF_TRANSFER) != 0U;
    Ep::transfer_held_ = transfer;
    const uint8_t token = int_st & USBHS_UIS_TOKEN_MASK;
    const uint8_t ep_num = int_st & USBHS_UIS_ENDP_MASK;
    const bool ep0_event = transfer && ep_num == 0U &&
                           (token == USBHS_UIS_TOKEN_IN || token == USBHS_UIS_TOKEN_OUT);

    // 1) 总线复位：核心先关闸门、结束全部请求，再把 EP0 置回等待 SETUP。
    // 1) Bus reset: the core closes the gate and ends every request first, then EP0
    //    goes back to waiting for a SETUP.
    if ((flags & USBHS_UIF_BUS_RST) != 0U)
    {
      USBHSD->DEV_AD = 0U;
      // 复位期间总线上没有事务：未结束的停止在报告复位之前结束。
      // No transaction is on the bus during a reset: pending stops end before the reset
      // is reported.
      Ep::FinishQuietStops(in_isr, true, false);
      OnBusReset(in_isr, ReadBusSpeed());
      ResetEp0(USBHS_UEP_R_RES_ACK);
      setup_since_reset_ = true;
      suspended_ = false;
    }

    // 2) EP0 的完成在 SETUP 之前交付（顺序契约，Endpoint::OnHardwareComplete）；SETUP
    //    之后再处理还会再翻一次 EP0 OUT 的 toggle。
    // 2) EP0 completions are delivered before the SETUP (ordering contract,
    //    Endpoint::OnHardwareComplete); handled after the SETUP they would also flip
    //    the EP0 OUT toggle a second time.
    if (ep0_event)
    {
      // INT_ST 的 IS_NAK 不作判据：没开 NAK 中断，UIF_TRANSFER 只为完成的事务置位，而
      // 有别的流量时这个标记出现在真正完成的事务上（之后被忙 NAK 的令牌置的）。CH32V307
      // 实测：批量流量下 151 631 个 EP0 零长度状态包的完成全带着它，473 万个批量 IN 完成
      // 也全带着；只有控制流量时状态包的完成从不带。
      // IS_NAK of INT_ST is no criterion: the NAK interrupt is off, so UIF_TRANSFER is
      // set for completed transactions only, and with other traffic the mark sits on
      // genuine completions (set by a token that was busy-NAKed afterwards). Measured on
      // a CH32V307: under bulk traffic all 151,631 completions of zero-length EP0 status
      // packets carried it, as did all 4.73 million bulk IN completions; with control
      // traffic alone a status completion never carried it.
      if (token == USBHS_UIS_TOKEN_OUT)
      {
        // 同一轮还有 SETUP 时，数据包的字节已被 SETUP 覆盖（两者共用 UEP0_DMA 那一块），
        // 报 FAILED。
        // With a SETUP in the same round the data packet's bytes were overwritten by the
        // SETUP (both use the UEP0_DMA block), so it is reported as FAILED.
        Ep* out0 = Ep::map_[0][OUT_IDX];
        const uint16_t rx_len = USBHSD->RX_LEN;
        if (out0->transfer_active_ && rx_len > 0U && (flags & USBHS_UIF_SETUP_ACT) != 0U)
        {
          // 停止未结束的请求已经回调过，这里只结束停止。 / A request whose stop is
          // pending was called back already; only the stop ends here.
          const bool stopping = out0->stopping_;
          out0->EndPending();
          out0->transfer_active_ = false;
          out0->OnHardwareComplete(in_isr, 0U,
                                   stopping ? ErrorCode::ABORTED : ErrorCode::FAILED);
        }
        else if (out0->transfer_active_)
        {
          out0->TransferComplete(in_isr, rx_len, true);
        }
      }
      else
      {
        Ep::map_[0][IN_IDX]->TransferComplete(in_isr, 0U, true);
      }
    }

    // 3) SETUP：8 字节先拷出 EP0 的 DMA 块（EP0 两个方向和下一个 SETUP 都写这块）。
    // 3) SETUP: the 8 bytes are copied out of EP0's DMA block first (both EP0
    //    directions and the next SETUP write that block).
    if ((flags & USBHS_UIF_SETUP_ACT) != 0U)
    {
      USB::SetupPacket setup{};
      std::memcpy(&setup, Ep::map_[0][OUT_IDX]->SlotMemory().addr_, sizeof(setup));
      if (setup_since_reset_)
      {
        setup_since_reset_ = false;
        const USB::Speed speed = ReadBusSpeed();
        if (speed != GetSpeed())
        {
          OnBusReset(in_isr, speed);
        }
      }
      // 先写 NAK，SETUP_ACT 到本轮末尾才清：其间控制器对所有令牌回忙 NAK（CH32V307
      // 实测，手册只写了 UIF_TRANSFER），控制端点上未结束的停止在交付 SETUP 之前结束，
      // 交付期间对它的停止当场完成。
      // NAK is written first and SETUP_ACT is cleared at the end of the round: meanwhile
      // the controller busy-NAKs every token (measured on a CH32V307; the manual states
      // it for UIF_TRANSFER only), a pending stop on the control endpoint ends before
      // the SETUP is handed over, and a stop of it during the hand-over is complete at
      // once.
      ResetEp0(USBHS_UEP_R_RES_NAK);
      Ep::setup_held_ = true;
      Ep::FinishQuietStops(in_isr, true, true);
      OnSetup(in_isr, setup);
      Ep::setup_held_ = false;
    }

    // 4) 其余令牌：数据端点的完成。SETUP 令牌只是信息位（SETUP 经 SETUP_ACT 上报）；
    //    SOF 中断不开，帧起始由 ReportFrame 上报。
    // 4) Other tokens: data endpoint completions. A SETUP token is information only
    //    (SETUP is reported through SETUP_ACT); the SOF interrupt stays off and
    //    ReportFrame reports starts of frame.
    if (transfer && !ep0_event)
    {
      if (token == USBHS_UIS_TOKEN_OUT && Ep::map_[ep_num][OUT_IDX] != nullptr)
      {
        // 每个非 EP0 OUT 事件都是一个已应答、已写进槽的包。带 IS_NAK 的事件的 TOG_OK
        // 不可信（后一个被忙 NAK 的令牌改写了 INT_ST）：板上实测这些事件带着新数据、
        // 主机随后发下一包，TOG_OK 却多为 0。所以带 IS_NAK 的包按 toggle 正确处理，只有
        // 不带 IS_NAK 的 TOG_OK=0 才是主机重发的旧包。
        // Every non-EP0 OUT event is a packet acknowledged and written to the slot. The
        // TOG_OK of an event with IS_NAK cannot be trusted (a later busy-NAKed token
        // rewrote INT_ST): measured on the board, these events carry new data and the
        // host sends the next packet afterwards, yet TOG_OK is mostly 0. So a packet
        // with IS_NAK is taken as in step, and only TOG_OK=0 without IS_NAK is a resend
        // of an old packet.
        // 紧跟着到来的 SETUP 也会清掉挂着的 OUT 事件的 TOG_OK，而且不置 IS_NAK（SETUP
        // 不会被忙 NAK）：这样的新包被当成重发丢掉，相位随之错开，下一包也丢（CH32V307
        // 实测，5 次现场一致）。所以 SETUP 已锁存时这个事件的 TOG_OK 同样不作判据。
        // A SETUP arriving right after also clears the TOG_OK of a pending OUT event,
        // and sets no IS_NAK (a SETUP is never busy-NAKed): such a new packet was dropped
        // as a resend, the phase went out of step and the next packet was dropped too
        // (measured on a CH32V307, the same in 5 captures). So with a SETUP latched the
        // TOG_OK of this event is no criterion either.
        const bool setup_latched = ((flags | USBHSD->INT_FG) & USBHS_UIF_SETUP_ACT) != 0U;
        Ep::map_[ep_num][OUT_IDX]->TransferComplete(
            in_isr, USBHSD->RX_LEN,
            setup_latched || (int_st & (USBHS_UIS_TOG_OK | USBHS_UIS_IS_NAK)) != 0U);
      }
      else if (token == USBHS_UIS_TOKEN_IN && Ep::map_[ep_num][IN_IDX] != nullptr &&
               (int_st & USBHS_UIS_IS_NAK) == 0U)
      {
        // 等时 IN 不按 INT_ST 分发（见下面的 ServiceIsoIn），这个调用对它什么也不做。
        // An isochronous IN is not dispatched by INT_ST (see ServiceIsoIn below); this
        // call does nothing for it.
        Ep::map_[ep_num][IN_IDX]->TransferComplete(in_isr, 0U, true);
      }
    }
    if (transfer)
    {
      Ep::CompleteAckedInPackets(in_isr);
      // 等时 IN：每个端点按 toggle 位认出它刚发出的一包（数据或零长度包），并在这一刻
      // 改写它的寄存器。有别的端点的流量时 INT_ST 多半已指向别处（CH32V307 实测：120 s
      // 里 81 万轮“INT_ST 指向别的端点而本端点的 toggle 已变”，对 10 万个指向本端点的
      // 事件），所以不看 INT_ST。
      // Isochronous IN: each endpoint recognises by its toggle bits the packet it has
      // just sent (data or zero-length) and has its registers rewritten at this moment.
      // With traffic on other endpoints INT_ST has mostly moved on (measured on a
      // CH32V307: in 120 s 810,000 rounds with INT_ST at another endpoint and this
      // endpoint's toggle changed, against 100,000 events pointing at it), so INT_ST is
      // not looked at.
      Ep::ServiceIsoIn(in_isr);
    }

    // 本轮清 HALT 的 toggle 复位放在本轮的完成都交付之后。 / The toggle resets of halts
    // cleared in this round come after this round's completions were delivered.
    Ep::FinishClearToggles();

    // 5) 写 1 清掉本轮看到的标志。 / 5) Write 1 to clear the flags seen in this round.
    USBHSD->INT_FG = flags;
    Ep::transfer_held_ = false;
  }
  // 挂起和恢复按 MIS_ST 此刻的状态报告，不按标志出现的次数：标志只有一个，读状态和清
  // 标志之间又变了一次的话，那次变化会随清除一起消失，后端就一直停在相反的状态上。事件
  // 循环已把标志清掉，这里读到的是最新状态；此后再变会重新置标志，由下一轮处理。排在
  // 本轮的完成和 SETUP 之后（设备核心的事件顺序）。
  // Suspend and resume are reported from the state MIS_ST shows now, not from how often
  // the flag appeared: there is one flag only, and a second change between reading the
  // state and clearing the flag would vanish with the clear and leave the backend in the
  // opposite state for good. The event loop has cleared the flag, so what is read here
  // is the latest state; a later change sets the flag again and is handled in the next
  // pass. This comes after the pass's completions and SETUP (the device core's event
  // order).
  const bool suspended = (USBHSD->MIS_ST & USBHS_UMS_SUSPEND) != 0U;
  if (!detach_.Detached() && suspended != suspended_)
  {
    suspended_ = suspended;
    if (suspended)
    {
      // 总线已空闲：未结束的停止在报告挂起之前结束。 / The bus is idle: pending stops
      // end before the suspend is reported.
      Ep::FinishQuietStops(in_isr, true, false);
      OnSuspend(in_isr);
    }
    else
    {
      OnResume(in_isr);
    }
  }
  // 没有待处理的事件了：SIE 空闲的话，未结束的停止都已停稳；否则按时间结束到期的。
  // No event is waiting any more: with a free SIE every pending stop is quiescent;
  // otherwise those that are due end by time.
  Ep::FinishQuietStops(in_isr, false, false);
  Ep::AgeStops(in_isr);
  if (!detach_.Detached() && !suspended_)
  {
    ReportFrame(in_isr);
    // 下限为 0：下一个节拍就挂起中断，约每毫秒一轮（下限为 1 时要等满一个完整的节拍，
    // CH32V307 实测是每 2 ms 一轮）。等待已在运行时返回 BUSY。
    // A lower bound of 0: the next tick pends the interrupt, about one pass per
    // millisecond (with 1 a whole tick has to pass first, which measured as one pass per
    // 2 ms on a CH32V307).
    // BUSY while the wait already runs.
    (void)tick_.Start(0U, 1U);
  }
}

void CH32USBOtgHS::ReportFrame(bool in_isr)
{
  const uint16_t raw = USBHSD->FRAME_NO & FRAME_NO_MASK;
  if (raw == last_frame_no_)
  {
    return;
  }
  last_frame_no_ = raw;
  // 全速下高 3 位没有意义，只报整帧。 / At full speed the high 3 bits mean nothing and
  // whole frames are reported.
  const bool high = GetSpeed() == USB::Speed::HIGH;
  OnStartOfFrame(in_isr, static_cast<uint16_t>(raw & 0x07FFU),
                 high ? static_cast<uint8_t>(raw >> 13U) : uint8_t{0xFFU});
}

void CH32USBOtgHS::MaskInterrupt() { NVIC_DisableIRQ(USBHS_IRQn); }

void CH32USBOtgHS::UnmaskInterrupt() { NVIC_EnableIRQ(USBHS_IRQn); }

bool CH32USBOtgHS::PendInterrupt()
{
  NVIC_SetPendingIRQ(USBHS_IRQn);
  return true;
}

void CH32USBOtgHS::SetAddress(uint8_t address, AddressStage stage)
{
  if (stage == AddressStage::STATUS_COMPLETE)
  {
    USBHSD->DEV_AD = address;
  }
}

void CH32USBOtgHS::OnControlReady() { USBHSD->UEP0_RX_CTRL = USBHS_UEP_R_RES_ACK; }

void CH32USBOtgHS::SetUpController()
{
  USBHSD->CONTROL = USBHS_UC_CLR_ALL | USBHS_UC_RESET_SIE;
  USBHSD->CONTROL &= ~USBHS_UC_RESET_SIE;
  USBHSD->HOST_CTRL = USBHS_UH_PHY_SUSPENDM;
  USBHSD->CONTROL = USBHS_UC_DMA_EN | USBHS_UC_INT_BUSY | USBHS_UC_SPEED_HIGH;
  // SOF 中断不开：SOF 以 UIF_TRANSFER 上报并改写 INT_ST，一个还没读出的 OUT 完成就此
  // 丢失。进中断先抄事件也只在中断延迟不超过几微秒时不丢（CH32V307 实测：别的中断占
  // 5 us 时 47 万个 OUT 包 0 丢，占 20 us 时丢 0.73%）。帧起始改由读 FRAME_NO 上报，见
  // ReportFrame。
  // The SOF interrupt stays off: a SOF is reported as UIF_TRANSFER and rewrites INT_ST,
  // which loses an OUT completion not yet read. Copying the event first thing in the
  // interrupt loses nothing only while the interrupt latency stays within a few
  // microseconds (measured on a CH32V307: 0 of 470,000 OUT packets lost with another
  // interrupt taking 5 us, 0.73 % with 20 us). Starts of frame are reported from
  // FRAME_NO instead, see ReportFrame.
  USBHSD->INT_EN =
      USBHS_UIE_SETUP_ACT | USBHS_UIE_TRANSFER | USBHS_UIE_DETECT | USBHS_UIE_SUSPEND;
  USBHSD->DEV_AD = 0U;
  USBHSD->CONTROL |= USBHS_UC_DEV_PU_EN;
}

void CH32USBOtgHS::StartController(bool in_isr)
{
  UNUSED(in_isr);
  // 先选共享的 48 MHz 时钟源，再开 USBHS 总线时钟。 / Select the shared 48 MHz source
  // first, then enable the USBHS bus clock.
  CH32UsbRcc::ConfigureUsb48M();
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_USBHS, ENABLE);
  // 断开时间未到时只记下"要接上"，到点后 owner 接上；中断线照样使能。
  // While the detach time has not passed only "attach wanted" is recorded and the owner
  // attaches once it has; the line is enabled all the same.
  if (detach_.WantAttach())
  {
    SetUpController();
  }
  // 先登记对象再开中断线：没有对象时中断只清标志，会丢掉这期间的总线复位。
  // Register the object before the line is enabled: without an object the interrupt
  // only clears the flags and would drop a bus reset arriving in between.
  self_ = this;
  NVIC_EnableIRQ(USBHS_IRQn);
}

void CH32USBOtgHS::HaltController()
{
  // SIE 复位后硬件不再为任何端点收发；端点上的请求和未结束的停止作废，之后的
  // StopHardware 立即返回 ABORTED。上拉和中断源都关掉。
  // After the SIE reset the hardware moves no data for any endpoint; the requests and
  // the pending stops on the endpoints are void and every later StopHardware returns
  // ABORTED at once. The pull-up and the interrupt sources are off.
  USBHSD->CONTROL = USBHS_UC_CLR_ALL | USBHS_UC_RESET_SIE;
  USBHSD->CONTROL = 0U;
  USBHSD->INT_EN = 0U;
  for (auto& pair : Ep::map_)
  {
    for (Ep* ep : pair)
    {
      if (ep != nullptr)
      {
        ep->transfer_active_ = false;
        ep->stopping_ = false;
      }
    }
  }
  Ep::pending_stops_ = 0U;
}

void CH32USBOtgHS::StopController(bool in_isr)
{
  UNUSED(in_isr);
  // 中断线保持使能、对象保持登记：owner 靠这条线运行。
  // The line stays enabled and the object registered: the owner runs on this line.
  HaltController();
  (void)detach_.OnHalted(false);
}

void CH32USBOtgHS::ResetController(bool in_isr)
{
  UNUSED(in_isr);
  // 拔出一段时间再接上，主机看到断开后重新枚举。断开要保持到主机看得见：全速下只断开
  // 几十微秒，主机 5 次里 4 次看不见（CH32V208 实测），5 ms 以上都看得见。这里只断开，
  // 不等：detach_ 按节拍计时，到点后 owner 在 ServiceHardware 开头接上。
  // Detached for a while and attached again, the host sees a disconnect and enumerates
  // again. The detach has to last until the host sees it: at full speed a detach of some
  // tens of microseconds went unseen 4 times out of 5 (measured on a CH32V208); 5 ms and
  // more were always seen. Only the detach happens here, no wait: detach_ counts on the
  // tick and the owner attaches at the start of ServiceHardware once the time has passed.
  HaltController();
  if (detach_.OnHalted(true) == USB::DetachTimer::HaltResult::ATTACH_NOW)
  {
    SetUpController();
  }
}

#endif  // defined(USBHSD)

// NOLINTEND(cppcoreguidelines-pro-type-cstyle-cast,performance-no-int-to-ptr)
