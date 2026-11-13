#include "esp_usb_ep.hpp"

#include "esp_memory_utils.h"

#if SOC_USB_OTG_SUPPORTED && defined(CONFIG_IDF_TARGET_ESP32S3) && \
    CONFIG_IDF_TARGET_ESP32S3

#include <algorithm>
#include <cstring>

#include "esp_usb.hpp"
#include "esp_usb_dev.hpp"

namespace LibXR
{

ESP32USBEndpoint::ESP32USBEndpoint(ESP32USBDevice& device, EPNumber number,
                                   Direction direction, RawData buffer)
    : USB::Endpoint(number, direction, buffer), device_(device)
{
  // 槽内存就是 DMA 目标（本后端不做中间拷贝）：构造期就要求它存在、DMA 可用且
  // 对齐，否则运行期会静默丢数据或写坏内存。
  // The slot memory is the DMA target itself (this backend does not copy through an
  // intermediate buffer): require at construction that it exists, is DMA-capable
  // and aligned, or the run time would silently drop or corrupt data.
  REQUIRE(buffer.addr_ != nullptr);
  REQUIRE(esp_ptr_dma_capable(buffer.addr_));
  REQUIRE(reinterpret_cast<uintptr_t>(buffer.addr_) % ESPUSBDetail::USB_DMA_ALIGNMENT ==
          0U);
}

ErrorCode ESP32USBEndpoint::ConfigureHardware(const Config& cfg)
{
  // 包长已由核心按 MaxPacketSizeLimit 收敛，这里不再改 Config（基类返回 const）；
  // 状态由基类管理，后端只返回结果码。
  // The packet size is already clamped by the core against MaxPacketSizeLimit, so
  // the Config is not touched here (the base hands it over const); the base owns
  // the state and the backend only reports a result code.
  const uint16_t packet = cfg.max_packet_size;
  ASSERT(cfg.direction == Direction::IN || cfg.direction == Direction::OUT);
  ASSERT(packet > 0U);

  auto buffer = SlotMemory();
  // 槽内存就是 DMA 目标，没有备用缓冲：槽不满足 DMA 对齐/可达性时拒绝配置。
  // The slot memory is the DMA target and there is no fallback buffer: refuse the
  // configuration when a slot does not satisfy DMA alignment and reachability.
  if (device_.DmaEnabled() && (buffer.size_ > 0U) &&
      !(cfg.direction == Direction::IN
            ? ESPUSBDetail::CanUseDirectInDmaBuffer(buffer.addr_, buffer.size_)
            : ESPUSBDetail::CanUseDirectOutDmaBuffer(buffer.addr_, buffer.size_)))
  {
    return ErrorCode::NO_BUFF;
  }

  if (IsPlanning())
  {
    return ErrorCode::OK;
  }

  if (cfg.direction == Direction::IN)
  {
    const bool is_bulk = (cfg.type == Type::BULK);
    if (!fifo_allocated_ && !device_.AllocateTxFifo(EPNumberToInt8(GetNumber()), packet,
                                                    is_bulk, fifo_words_))
    {
      return ErrorCode::NO_BUFF;
    }
    fifo_allocated_ = true;
  }
  else
  {
    if (!device_.EnsureRxFifo(packet))
    {
      return ErrorCode::NO_BUFF;
    }
  }

  ActivateHardwareEndpoint();
  ResetTransferState();
  return ErrorCode::OK;
}

void ESP32USBEndpoint::CloseHardware(bool /*in_isr*/)
{
  // 核心保证先停止、确认后才关闭，所以到这里端点已经是禁用的；这里收掉中断掩码并清
  // USBAEP 停用端点（EP0 始终激活），之后核心不再应答发给它的令牌。万一硬件仍是使能，
  // 发出禁用请求但不等待（等待由停止状态机和 SOF 超时负责）。
  // The core stops an endpoint and waits for the confirmation before closing it, so the
  // endpoint is already disabled here; the interrupt masks are dropped and the endpoint
  // is deactivated by clearing USBAEP (EP0 is always active), after which the controller
  // no longer answers tokens sent to it. If the hardware is still enabled, a disable
  // request goes out without waiting (waiting belongs to the stop state machine and the
  // SOF timeout).
  auto* dev = reinterpret_cast<usb_dwc_dev_t*>(ESPUSBDetail::DWC2_FS_REG_BASE);
  const uint8_t ep_num = EPNumberToInt8(GetNumber());

  if (GetDirection() == Direction::IN)
  {
    if (ep_num == 0U)
    {
      if (dev->diepctl0_reg.epena)
      {
        dev->diepctl0_reg.snak = 1;
        dev->diepctl0_reg.epdis = 1;
      }
    }
    else
    {
      dev->diepempmsk_reg.val &= ~(1UL << ep_num);
      if (dev->in_eps[ep_num - 1U].diepctl_reg.epena)
      {
        dev->in_eps[ep_num - 1U].diepctl_reg.snak = 1;
        dev->in_eps[ep_num - 1U].diepctl_reg.epdis = 1;
      }
      dev->in_eps[ep_num - 1U].diepctl_reg.usbactep = 0;
    }
    dev->daintmsk_reg.val &= ~(1UL << ep_num);
  }
  else
  {
    if (ep_num == 0U)
    {
      ESPUSBDetail::RequestOutDisable(dev->doepctl0_reg, false);
    }
    else
    {
      ESPUSBDetail::RequestOutDisable(dev->out_eps[ep_num - 1U].doepctl_reg, true);
      dev->out_eps[ep_num - 1U].doepctl_reg.usbactep = 0;
    }
    dev->daintmsk_reg.val &= ~(1UL << (16U + ep_num));
  }

  ResetTransferState();
}

ErrorCode ESP32USBEndpoint::StallHardware(bool /*in_isr*/)
{
  const bool is_in = (GetDirection() == Direction::IN);
  auto* dev = reinterpret_cast<usb_dwc_dev_t*>(ESPUSBDetail::DWC2_FS_REG_BASE);
  const uint8_t ep_num = EPNumberToInt8(GetNumber());

  if (is_in)
  {
    if (ep_num == 0U)
    {
      dev->diepctl0_reg.stall = 1;
    }
    else
    {
      dev->in_eps[ep_num - 1U].diepctl_reg.stall = 1;
    }
  }
  else
  {
    if (ep_num == 0U)
    {
      dev->doepctl0_reg.stall = 1;
    }
    else
    {
      dev->out_eps[ep_num - 1U].doepctl_reg.stall = 1;
    }
  }

  return ErrorCode::OK;
}

ErrorCode ESP32USBEndpoint::ClearStallHardware(bool /*in_isr*/)
{
  auto* dev = reinterpret_cast<usb_dwc_dev_t*>(ESPUSBDetail::DWC2_FS_REG_BASE);
  const uint8_t ep_num = EPNumberToInt8(GetNumber());

  if (GetDirection() == Direction::IN)
  {
    if (ep_num == 0U)
    {
      dev->diepctl0_reg.stall = 0;
    }
    else
    {
      auto& reg = dev->in_eps[ep_num - 1U].diepctl_reg;
      reg.stall = 0;
      reg.setd0pid = 1;
    }
  }
  else
  {
    if (ep_num == 0U)
    {
      dev->doepctl0_reg.stall = 0;
    }
    else
    {
      auto& reg = dev->out_eps[ep_num - 1U].doepctl_reg;
      reg.stall = 0;
      reg.setd0pid = 1;
    }
  }

  return ErrorCode::OK;
}

USB::Endpoint::StopResult ESP32USBEndpoint::StopHardware(bool in_isr)
{
  auto* dev = reinterpret_cast<usb_dwc_dev_t*>(ESPUSBDetail::DWC2_FS_REG_BASE);
  const uint8_t ep_num = EPNumberToInt8(GetNumber());
  const bool is_in = (GetDirection() == Direction::IN);

  if (!transfer_active_)
  {
    // 没有在途请求：硬件侧无需停止，直接确认。
    // Nothing in flight: nothing to stop on the hardware side, confirm at once.
    stop_phase_ = StopPhase::NONE;
    return StopResult::DONE;
  }

  // 停止是 SNAK -> 中断确认 INEPNE -> EPDIS -> 中断确认
  // EPDISD -> 刷 FIFO，期间不能忙等（核心在临界区里调用本函数，主机不轮询时
  // 忙等会卡到看门狗）。这里只发起第一步，其余由中断推进，完成后调
  // OnHardwareStopped。
  // Stopping is SNAK -> INEPNE interrupt -> EPDIS ->
  // EPDISD interrupt -> flush FIFO, and it must not busy-wait (the core calls
  // this inside a critical section, and a host that stops polling would hang the
  // device until the watchdog fires). Start the first step here, let the
  // interrupt advance the rest and call OnHardwareStopped when it is done.
  if (is_in)
  {
    if (iso_abort_)
    {
      // 错过帧的禁用序列已经在跑：它改为以这次停止结束。
      // A missed-frame disable is already running; it now ends as this stop.
      iso_abort_ = false;
      // 端点已经自己禁用（传输先完成了），没有 EPDISD 会来：这里直接结束，请求已全部
      // 发出。
      // The endpoint has disabled itself (the transfer completed first) and no EPDISD
      // will come: end the stop here; the request has been sent completely.
      if (dev->in_eps[ep_num - 1U].diepctl_reg.epena == 0U)
      {
        stopped_sent_all_ = true;
        DropCompletionFlag();
        ESPUSBDetail::FlushTxFifo(dev, ep_num);
        stop_phase_ = StopPhase::NONE;
        transfer_active_ = false;
        return StopResult::DONE;
      }
      return StopResult::PENDING;
    }
    bool enabled = false;
    if (ep_num == 0U)
    {
      enabled = dev->diepctl0_reg.epena != 0U;
    }
    else
    {
      enabled = dev->in_eps[ep_num - 1U].diepctl_reg.epena != 0U;
    }
    if (!enabled)
    {
      // 端点已不在使能状态：请求已经在硬件里发完，在这里记下“已全部发出”。
      // The endpoint is no longer enabled: the request has been sent in the hardware, and
      // "everything was sent" is recorded here.
      stopped_sent_all_ = true;
      DropCompletionFlag();
      ESPUSBDetail::FlushTxFifo(dev, ep_num);
      stop_phase_ = StopPhase::NONE;
      transfer_active_ = false;
      return StopResult::DONE;
    }
    // 打开 EPDISD 与 INEPNE 的中断，才能在中断里推进。
    // Unmask EPDISD and INEPNE so the interrupt can drive the sequence.
    dev->diepmsk_reg.epdisbldmsk = 1;
    dev->diepmsk_reg.inepnakeffmsk = 1;
    // NAK 已经生效时（SETUP 让核心自己给 EP0 IN 置了 NAK，那次 INEPNE 早已处理过）不再
    // 等 INEPNE，直接写 EPDIS。
    // With the NAK already in effect (a SETUP made the core set it on EP0 IN, and that
    // INEPNE was handled long ago) there is no INEPNE to wait for: write EPDIS directly.
    bool nak_effective = false;
    if (ep_num == 0U)
    {
      nak_effective = dev->diepctl0_reg.naksts != 0U;
      dev->diepctl0_reg.snak = 1;
      if (nak_effective) dev->diepctl0_reg.epdis = 1;
    }
    else
    {
      nak_effective = dev->in_eps[ep_num - 1U].diepctl_reg.naksts != 0U;
      dev->in_eps[ep_num - 1U].diepctl_reg.snak = 1;
      if (nak_effective) dev->in_eps[ep_num - 1U].diepctl_reg.epdis = 1;
    }
    stop_phase_ = nak_effective ? StopPhase::DIS_WAIT : StopPhase::SNAK_WAIT;
    return StopResult::PENDING;
  }

  if (ep_num == 0U)
  {
    // EP0 OUT：手册不允许写 EPDIS。SNAK 在事务边界立即生效（INT_BUSY 自动 NAK），
    // 契约要求此时就报 DONE；把 DOEPDMA0 指向驱动自己的暂存，任何迟到的一包只
    // 会落进暂存，被借出的槽可以安全回收。
    // EP0 OUT: the manual forbids EPDIS. SNAK takes effect at the transaction
    // boundary (INT_BUSY auto-NAKs), and the contract wants DONE right here; point
    // DOEPDMA0 at the driver's own scratch so a late packet lands in the scratch
    // and the lent slot can be reclaimed safely.
    dev->doepctl0_reg.snak = 1;
    dev->doepdma0_reg.val =
        static_cast<uint32_t>(reinterpret_cast<uintptr_t>(device_.ep0_out_scratch_));
    stop_phase_ = StopPhase::NONE;
    transfer_active_ = false;
    return StopResult::DONE;
  }

  if (dev->out_eps[ep_num - 1U].doepctl_reg.epena == 0U)
  {
    DropCompletionFlag();
    stop_phase_ = StopPhase::NONE;
    transfer_active_ = false;
    return StopResult::DONE;
  }
  // 非 EP0 OUT：禁用 OUT 端点之前要先让全局 OUT NAK 生效（DCTL.SGOUTNAK，等
  // GINTSTS.GOUTNAKEFF），再写 SNAK 和 EPDIS，中断确认 EPDISD 后撤销全局 OUT NAK。
  // 全局 OUT NAK 由所有正在停止的 OUT 端点共用；已经生效时这里直接走到 EPDIS。
  // Non-EP0 OUT: the global OUT NAK has to be in effect before an OUT endpoint is
  // disabled (DCTL.SGOUTNAK, then GINTSTS.GOUTNAKEFF); SNAK and EPDIS follow, and once
  // the EPDISD interrupt confirms the stop the global OUT NAK is released. It is shared
  // by every OUT endpoint that is stopping; when it is already in effect this goes on to
  // EPDIS at once.
  stop_phase_ = StopPhase::SNAK_WAIT;
  device_.AcquireGlobalOutNak();
  (void)AdvanceStop(in_isr, 0U);
  return StopResult::PENDING;
}

bool ESP32USBEndpoint::AdvanceStop(bool in_isr, uint32_t status)
{
  auto* dev = reinterpret_cast<usb_dwc_dev_t*>(ESPUSBDetail::DWC2_FS_REG_BASE);
  const uint8_t ep_num = EPNumberToInt8(GetNumber());
  const bool is_in = (GetDirection() == Direction::IN);

  if (stop_phase_ == StopPhase::NONE)
  {
    return false;
  }

  // 超时不在这里判断：本函数只在端点自己的中断里跑，没有中断就永远不会超时。
  // 核心按 1 ms 帧计数，超时后调设备层的 OnStopTimeout（控制器复位升级），由它
  // 调 ForceStop 收尾（与 STM32 OTG 同思路）。
  // The timeout is not decided here: this runs only in the endpoint's own
  // interrupt, so without an interrupt it would never fire. The core counts 1 ms
  // frames and calls the device-level OnStopTimeout (the controller-reset
  // escalation), which finishes the stop through ForceStop (same approach as the
  // STM32 OTG backend).
  if (is_in)
  {
    usb_dwc_diepint_reg_t intr = {};
    intr.val = status;
    if (stop_phase_ == StopPhase::SNAK_WAIT)
    {
      // NAK 生效之前核心仍可能把最后一个包发出去：传输已经完成、端点已经禁用时不能再写
      // EPDIS（只能对使能的端点写，也不会再有 EPDISD）。FIFO 已空，停止到此结束，请求
      // 已全部发出。
      // The core may still send the last packet before the NAK is in effect: once the
      // transfer has completed and the endpoint is disabled, EPDIS must not be written
      // (it is only for an enabled endpoint, and no EPDISD would follow). The FIFO is
      // empty, the stop ends here and the request has been sent completely.
      const bool enabled = (ep_num == 0U)
                               ? (dev->diepctl0_reg.epena != 0U)
                               : (dev->in_eps[ep_num - 1U].diepctl_reg.epena != 0U);
      if (!enabled)
      {
        stopped_sent_all_ = true;
        stop_phase_ = StopPhase::NONE;
        transfer_active_ = false;
        OnHardwareStopped(in_isr);
        return true;
      }
      if (intr.inepnakeff == 0U) return false;
      if (ep_num == 0U)
      {
        dev->diepctl0_reg.epdis = 1;
      }
      else
      {
        dev->in_eps[ep_num - 1U].diepctl_reg.epdis = 1;
      }
      stop_phase_ = StopPhase::DIS_WAIT;
      return false;
    }
    if (intr.epdisbld == 0U)
    {
      // 只有传输完成标志能说明传输先结束了：普通的禁用里 EPENA 可能比 EPDISD 早一点清掉。
      // Only a transfer-complete flag shows that the transfer ended first: in an ordinary
      // disable EPENA may clear a moment before the EPDISD flag is raised.
      const bool enabled = (ep_num == 0U)
                               ? (dev->diepctl0_reg.epena != 0U)
                               : (dev->in_eps[ep_num - 1U].diepctl_reg.epena != 0U);
      if (enabled || intr.xfercompl == 0U) return false;
      // 端点没有等到 EPDIS 生效就先把传输发完了（主机的令牌排在 EPDIS 前面）：核心在
      // 传输完成时自己清 EPENA，EPDIS 写到一个已禁用的端点上，EPDISD 不会再来，等它只会
      // 一直等到超时。等待中的错过帧中止作废（数据其实发出去了），完成由调用者照常报告；
      // 等待中的停止在这里结束，请求已全部发出。
      // The endpoint finished its transfer before the EPDIS took effect (the host's token
      // came ahead of it): the core clears EPENA itself at the end of a transfer, EPDIS
      // lands on an endpoint that is already disabled, and no EPDISD follows, so waiting
      // for it would only end in the timeout. A missed-frame abort that was waiting is
      // void (the data went out) and the caller reports the completion as usual; a
      // waiting stop ends here with the request sent completely.
      stop_phase_ = StopPhase::NONE;
      if (iso_abort_)
      {
        iso_abort_ = false;
        return false;
      }
      ESPUSBDetail::FlushTxFifo(dev, ep_num);
      stopped_sent_all_ = true;
      transfer_active_ = false;
      OnHardwareStopped(in_isr);
      return true;
    }
    ESPUSBDetail::FlushTxFifo(dev, ep_num);
    stop_phase_ = StopPhase::NONE;
    transfer_active_ = false;
    if (iso_abort_)
    {
      // 错过帧的请求以 FAILED 结束，设备类据此提交下一帧。
      // The request that missed its frame ends with FAILED, on which the class submits
      // the next frame.
      const uint32_t tag = transfer_tag_;
      ResetTransferState();
      OnHardwareComplete(in_isr, tag, 0U, ErrorCode::FAILED);
      return true;
    }
    // 禁用生效的同时主机取走了最后一个包：请求已全部发出。
    // The host took the last packet as the disable took effect: the request has been
    // sent completely.
    if (intr.xfercompl != 0U) stopped_sent_all_ = true;
    OnHardwareStopped(in_isr);
    return true;
  }

  if (stop_phase_ == StopPhase::SNAK_WAIT)
  {
    if (dev->gintsts_reg.goutnakeff == 0U) return false;
    auto& ctl = dev->out_eps[ep_num - 1U].doepctl_reg;
    if (ctl.epena == 0U)
    {
      // 等全局 OUT NAK 期间传输已经收完，端点已禁用：不能再写 EPDIS。
      // The transfer completed while the global OUT NAK was awaited and the endpoint is
      // disabled: EPDIS must not be written.
      device_.ReleaseGlobalOutNak();
      stop_phase_ = StopPhase::NONE;
      transfer_active_ = false;
      OnHardwareStopped(in_isr);
      return true;
    }
    dev->doepmsk_reg.epdisbldmsk = 1;
    ESPUSBDetail::RequestOutDisable(ctl, true);
    stop_phase_ = StopPhase::DIS_WAIT;
    return false;
  }

  usb_dwc_doepint_reg_t intr = {};
  intr.val = status;
  if (intr.epdisbld == 0U) return false;
  device_.ReleaseGlobalOutNak();
  stop_phase_ = StopPhase::NONE;
  transfer_active_ = false;
  OnHardwareStopped(in_isr);
  return true;
}

void ESP32USBEndpoint::DropCompletionFlag()
{
  // 停止在这里同步结束了一个已经在硬件里完成的请求，核心从停止这边拿结果。它的 XFRC 还
  // 挂着，要等停止返回之后才进中断；那时核心可能已经启动了下一个请求，这次完成会报到
  // 那个请求上。EP0 的完成在处理 SETUP 之前已经收掉（FinishPendingEp0InStatus），不在
  // 这里处理。
  // The stop has just ended, synchronously, a request that had already completed in the
  // hardware, and the core takes the result from the stop. Its XFRC is still pending and
  // would only be handled after the stop returns; by then the core may have started the
  // next request, and the completion would be reported for that one. EP0 completions are
  // consumed before a SETUP is handled (FinishPendingEp0InStatus) and are left alone.
  const uint8_t ep_num = EPNumberToInt8(GetNumber());
  if (ep_num == 0U) return;
  auto* dev = reinterpret_cast<usb_dwc_dev_t*>(ESPUSBDetail::DWC2_FS_REG_BASE);
  if (GetDirection() == Direction::IN)
  {
    usb_dwc_diepint_reg_t flag = {};
    flag.xfercompl = 1;
    dev->in_eps[ep_num - 1U].diepint_reg.val = flag.val;
  }
  else
  {
    usb_dwc_doepint_reg_t flag = {};
    flag.xfercompl = 1;
    dev->out_eps[ep_num - 1U].doepint_reg.val = flag.val;
  }
}

void ESP32USBEndpoint::ConfirmStopAfterReset(bool in_isr)
{
  // 复位清掉了端点中断掩码，等待中的停止不会再有 EPDISD 中断来推进；复位已经确认
  // 端点被禁用，这里直接完成它。
  // The reset wipes the endpoint interrupt masks, so a waiting stop would never get its
  // EPDISD interrupt; the reset has confirmed the endpoint disabled, so finish it here.
  if (stop_phase_ == StopPhase::NONE)
  {
    return;
  }
  stop_phase_ = StopPhase::NONE;
  transfer_active_ = false;
  if (iso_abort_)
  {
    // 没有停止在等确认；请求由复位的收尾流程结束。
    // No stop is waiting for a confirmation; the reset teardown ends the request.
    iso_abort_ = false;
    return;
  }
  OnHardwareStopped(in_isr);
}

// 等时 IN 传输没有在安排的那一帧发出去（主机没有取，或令牌出错）：核心只报告“至少有
// 一个等时 IN 端点没发”，应用要逐个读等时 IN 端点的控制寄存器找出是哪个。端点仍使能、
// 且奇偶帧位（EONUM）等于刚结束那一帧的奇偶（ended_odd）时，它就是为那一帧安排的；为下
// 一帧安排的端点奇偶相反，请求保持。对未完成的端点置 NAK 并禁用，等端点禁用中断，再冲刷
// 发送 FIFO。请求以 FAILED 结束，设备类据此提交下一帧。
// An isochronous IN transfer was not sent in the frame it was scheduled for (the host
// did not take it, or the token was corrupted): the core only reports that at least one
// isochronous IN endpoint did not send, and the application reads the control register of
// every isochronous IN endpoint to find which. An endpoint that is still enabled and
// whose even/odd frame bit (EONUM) equals the parity of the frame that ended (ended_odd)
// was armed for that frame; one armed for the next frame has the other parity and keeps
// its request. The incomplete endpoint gets the NAK and is disabled, the application
// waits for the endpoint-disabled interrupt and flushes the transmit FIFO. The request
// ends with FAILED, on which the class submits the next frame.
void ESP32USBEndpoint::OnIsoIncomplete(bool ended_odd)
{
  if (GetDirection() != Direction::IN || GetConfig().type != Type::ISOCHRONOUS) return;
  if (!transfer_active_ || stop_phase_ != StopPhase::NONE) return;
  auto* dev = reinterpret_cast<usb_dwc_dev_t*>(ESPUSBDetail::DWC2_FS_REG_BASE);
  auto& ctl = dev->in_eps[EPNumberToInt8(GetNumber()) - 1U].diepctl_reg;
  if (ctl.epena == 0U) return;
  if ((ctl.dpid != 0U) != ended_odd) return;
  iso_abort_ = true;
  dev->diepmsk_reg.epdisbldmsk = 1;
  ctl.snak = 1;
  ctl.epdis = 1;
  stop_phase_ = StopPhase::DIS_WAIT;
}

void ESP32USBEndpoint::ForceStop(bool in_isr)
{
  // 控制器复位升级路径：停止没等到硬件确认，寄存器已经不可信（DMA 仍可能写槽，
  // 但复位会中止一切）。这里只收掉软件状态、不再碰寄存器；核心复位后由设备层
  // 重建寄存器并让主机重新枚举。
  // Controller-reset escalation path: the stop never confirmed and the registers
  // can no longer be trusted (the DMA may still be writing the slot, but the reset
  // aborts everything). Only the software state is dropped here; the device layer
  // rebuilds the registers after the reset and the host enumerates again.
  // 只有核心在等的停止才确认：错过帧的禁用序列没有停止在等，没有停止的端点更没有。核心
  // 不会在下一次停止开始时清掉确认标志，多报的确认会让那次停止在硬件停下之前就结束。
  // Only a stop the core is waiting for is confirmed: a missed-frame disable has no stop
  // waiting, and an endpoint without a stop even less. The core does not clear the
  // confirmation when the next stop starts, so a confirmation too many would end that
  // stop before the hardware has stopped.
  const bool stop_waiting = (stop_phase_ != StopPhase::NONE) && !iso_abort_;
  stop_phase_ = StopPhase::NONE;
  // 复位后 FIFO 记账和传输状态都要重建（端点会在下一次 Configure 重新分配）。
  // The FIFO bookkeeping and transfer state are rebuilt after the reset (endpoints
  // re-allocate on the next Configure).
  ResetHardwareState();
  if (stop_waiting) OnHardwareStopped(in_isr);
}

size_t ESP32USBEndpoint::ReapProgress(const HwRequest& request, bool& retired)
{
  // 只有硬件报过传输完成才能认为主机真的取走了数据，停止时序把它记在
  // stopped_sent_all_ 里；DMA 模式下 xfersize 在数据进 FIFO 时就清零，不能拿它当成功
  // 凭据（与 STM32 OTG 同口径）。
  // Only a transfer completion proves the host really took the data, and the stop
  // sequence records it in stopped_sent_all_: in DMA mode xfersize drops as soon as a
  // packet enters the FIFO, so it is no evidence of success (same criterion as the STM32
  // OTG backend).
  const uint8_t ep_num = EPNumberToInt8(GetNumber());
  retired = false;
  if (GetDirection() == Direction::IN && ep_num != 0U)
  {
    if (stopped_sent_all_)
    {
      retired = true;
      return request.length;
    }
    return GetCompletedTransferSize();
  }
  // OUT 侧：已经落进槽的字节数是确定的。
  // OUT side: the bytes already in the slot are known for certain.
  return GetCompletedTransferSize();
}

ErrorCode ESP32USBEndpoint::StartHardware(const HwRequest& request, bool /*in_isr*/)
{
  const size_t size = request.length;

  if (request.slot.size_ < size)
  {
    return ErrorCode::NO_BUFF;
  }

  transfer_buffer_ = static_cast<uint8_t*>(request.slot.addr_);
  transfer_tag_ = request.tag;
  transfer_active_ = true;
  stopped_sent_all_ = false;
  transfer_request_size_ = size;
  transfer_actual_size_ = 0U;
  transfer_queued_size_ = 0U;

  const uint8_t ep_num = EPNumberToInt8(GetNumber());
  if ((ep_num == 0U) && (GetDirection() == Direction::OUT))
  {
    ep0_out_phase_ = (size == 0U) ? Ep0OutPhase::STATUS : Ep0OutPhase::DATA;
  }

  if (!PrepareTransferBuffer(size))
  {
    ResetTransferState();
    return ErrorCode::NO_MEM;
  }

  ProgramTransfer(size);
  return ErrorCode::OK;
}

size_t ESP32USBEndpoint::MaxPacketSizeLimit(Type type) const
{
  // 与 STM32 OTG 同口径：CONTROL 固定 64，其余受端点存储上限约束。
  // Same convention as STM32 OTG: CONTROL is fixed at 64, the rest is bounded by
  // the endpoint storage.
  if (type == Type::CONTROL)
  {
    return 64U;
  }
  return SlotMemory().size_;
}

void ESP32USBEndpoint::ResetHardwareState()
{
  fifo_allocated_ = false;
  fifo_words_ = 0U;
  ResetTransferState();
}

void ESP32USBEndpoint::HandleInInterrupt(bool in_isr)
{
  // 先读出并清掉这个端点的中断状态，之后的处理都用这一份：分两次读的话，两次之间
  // 才置位的停止标志会在整体清除时被清掉而没人看到。
  // Read and clear the interrupt status of this endpoint first and let everything below
  // use that one copy: with two reads, a stop flag that is set between them is wiped by
  // the blanket clear without having been seen.
  const uint8_t ep_num = EPNumberToInt8(GetNumber());
  usb_dwc_diepint_reg_t intr = {};
  auto* dev = reinterpret_cast<usb_dwc_dev_t*>(ESPUSBDetail::DWC2_FS_REG_BASE);

  if (ep_num == 0U)
  {
    intr.val = dev->diepint0_reg.val;
    dev->diepint0_reg.val = intr.val;
  }
  else
  {
    intr.val = dev->in_eps[ep_num - 1U].diepint_reg.val;
    dev->in_eps[ep_num - 1U].diepint_reg.val = intr.val;
  }

  // 停止时序未完成时先推进它（PENDING 由中断确认后调 OnHardwareStopped）。请求在这里
  // 结束时，这一份状态里的其余标志都属于它，不再处理：核心可能已经在 OnHardwareStopped
  // 里启动了下一个请求，再报一次完成会挂到那个请求上。
  // Advance the stop sequence first while it is still pending (PENDING becomes a
  // confirmed stop through OnHardwareStopped). When the request ends here, the other
  // flags of this copy belong to it and are not handled: the core may have started the
  // next request inside OnHardwareStopped, and a completion reported now would be
  // attached to that request.
  if (AdvanceStop(in_isr, intr.val)) return;

  // 在这里捕获标签：之后现读 transfer_tag_ 会被同一条路径上的 StartHardware 改写，
  // 报出去的完成可能挂到下一个请求上。
  // Capture the tag here: reading transfer_tag_ live later can be overwritten by a
  // StartHardware on the same path, attaching the completion to the next request.
  const uint32_t tag = transfer_tag_;

  if (!device_.DmaEnabled() && intr.txfemp)
  {
    WriteMoreTxData();
    if (transfer_queued_size_ >= transfer_request_size_)
    {
      dev->diepempmsk_reg.val &= ~(1UL << ep_num);
    }
  }

  if (intr.xfercompl)
  {
    const bool rearm_ep0_setup = (ep_num == 0U) && device_.DmaEnabled() &&
                                 (transfer_request_size_ == 0U) &&
                                 device_.LastSetupDirectionOut();
    dev->diepempmsk_reg.val &= ~(1UL << ep_num);
    const size_t actual =
        device_.DmaEnabled() ? GetCompletedTransferSize() : transfer_request_size_;
    OnHardwareComplete(in_isr, tag, actual);
    if (rearm_ep0_setup)
    {
      device_.ReloadSetupPacketCount();
    }
    if (!transfer_active_)
    {
      ResetTransferState();
    }
  }
}

void ESP32USBEndpoint::FinishPendingEp0InStatus(bool in_isr)
{
  if (device_.endpoint_map_.in[0] == nullptr) return;
  auto* dev = reinterpret_cast<usb_dwc_dev_t*>(ESPUSBDetail::DWC2_FS_REG_BASE);
  if (dev->diepint0_reg.xfercompl)
  {
    auto* input = static_cast<ESP32USBEndpoint*>(device_.endpoint_map_.in[0]);
    input->HandleInInterrupt(in_isr);
  }
}

void ESP32USBEndpoint::HandleOutInterrupt(bool in_isr)
{
  // 与 IN 相同：先读出并清掉中断状态，停止时序和后面的处理都用这一份。
  // As for IN: read and clear the interrupt status first; the stop sequence and the
  // handling below use that one copy.
  const uint8_t ep_num = EPNumberToInt8(GetNumber());
  usb_dwc_doepint_reg_t intr = {};
  auto* dev = reinterpret_cast<usb_dwc_dev_t*>(ESPUSBDetail::DWC2_FS_REG_BASE);

  if (ep_num == 0U)
  {
    intr.val = dev->doepint0_reg.val;
    dev->doepint0_reg.val = intr.val;
  }
  else
  {
    intr.val = dev->out_eps[ep_num - 1U].doepint_reg.val;
    dev->out_eps[ep_num - 1U].doepint_reg.val = intr.val;
  }

  if (AdvanceStop(in_isr, intr.val)) return;
  const uint32_t tag = transfer_tag_;

  const bool ep0_setup_related =
      (ep_num == 0U) && (intr.setup || intr.stuppktrcvd || intr.back2backsetup);
  const bool has_active_ep0_transfer =
      (ep_num == 0U) && transfer_active_ &&
      ((transfer_buffer_ != nullptr) || (transfer_request_size_ > 0U));

  if (intr.xfercompl &&
      ((ep_num != 0U) || ((ep0_out_phase_ != Ep0OutPhase::SETUP) && !ep0_setup_related &&
                          has_active_ep0_transfer)))
  {
    const size_t actual =
        device_.DmaEnabled() ? GetCompletedTransferSize() : transfer_actual_size_;
    const auto finish_out_transfer_result = FinishOutTransfer(actual);
    REQUIRE_FROM_CALLBACK(finish_out_transfer_result, in_isr);
    if (ep_num == 0U)
    {
      FinishPendingEp0InStatus(in_isr);
    }
    OnHardwareComplete(in_isr, tag, actual);
    if ((ep_num == 0U) && device_.DmaEnabled() && (transfer_request_size_ == 0U))
    {
      device_.ReloadSetupPacketCount();
    }
    if (!transfer_active_)
    {
      ResetTransferState();
    }
  }

  // EP0 OUT 收完一个数据包、或主机进入控制写的状态阶段之后被核心关掉：重新使能它来接收
  // SETUP，保持 NAK。SETUP 阶段结束的情况在下面处理。
  // EP0 OUT was disabled by the core after a data packet or when the host entered the
  // status stage of a control write: enable it again for SETUP packets, with the NAK
  // kept. The end of a SETUP stage is handled below.
  if ((ep_num == 0U) && device_.DmaEnabled() && !intr.setup &&
      (intr.xfercompl || intr.stsphsercvd) && (dev->doepctl0_reg.epena == 0U))
  {
    device_.ReloadSetupPacketCount(false);
  }

  if (intr.setup)
  {
    if (device_.DmaEnabled())
    {
      const auto cache_sync_dma_buffer_result = ESPUSBDetail::CacheSyncDmaBuffer(
          device_.setup_packet_, ESP32USBDevice::SETUP_DMA_BUFFER_BYTES, false);
      REQUIRE_FROM_CALLBACK(cache_sync_dma_buffer_result, in_isr);
    }
    // 核心把每个 SETUP 写到 DOEPDMA0 当前的地址并前移 8 字节，要处理的是最后收到的那个，
    // 它就在这个地址之前：背靠背的 SETUP 依次排在 SETUP 缓冲区里；OUT 数据阶段已经挂上时
    // 到来的 SETUP 写在数据槽的开头。其余情况（刚重新装填过）取 SETUP 缓冲区的开头。
    // The core stores each SETUP at the current address of DOEPDMA0 and advances it by 8
    // bytes, and the one to process is the last received, right below that address:
    // back-to-back SETUP packets follow one another in the SETUP buffer, and a SETUP that
    // arrives with an OUT data stage armed lands at the start of the data slot. Anything
    // else (the buffer was just reloaded) means the start of the SETUP buffer.
    const uint8_t* packet = device_.setup_packet_;
    if (device_.DmaEnabled())
    {
      const uintptr_t start = reinterpret_cast<uintptr_t>(device_.setup_packet_);
      const uintptr_t last = static_cast<uintptr_t>(dev->doepdma0_reg.val) -
                             ESP32USBDevice::SETUP_PACKET_BYTES;
      if (last > start && last + ESP32USBDevice::SETUP_PACKET_BYTES <=
                              start + ESP32USBDevice::SETUP_DMA_BUFFER_BYTES)
      {
        packet = reinterpret_cast<const uint8_t*>(last);
      }
      else if (transfer_hw_buffer_ != nullptr &&
               last == reinterpret_cast<uintptr_t>(transfer_hw_buffer_))
      {
        const auto slot_sync_result = ESPUSBDetail::CacheSyncDmaBuffer(
            transfer_hw_buffer_, ESPUSBDetail::USB_DMA_ALIGNMENT, false);
        REQUIRE_FROM_CALLBACK(slot_sync_result, in_isr);
        packet = transfer_hw_buffer_;
      }
    }
    device_.UpdateSetupState(packet);
    const auto* setup = reinterpret_cast<const USB::SetupPacket*>(packet);
    const bool expect_out_data_stage =
        ((setup->bmRequestType & 0x80U) == 0U) && (setup->wLength > 0U);

    FinishPendingEp0InStatus(in_isr);
    ep0_out_phase_ = Ep0OutPhase::SETUP;
    ResetTransferState();

    // EP0 IN 上被这个 SETUP 打断的传输不在这里清软件状态：它在硬件里还使能着、FIFO 里
    // 还有包，核心处理 SETUP 时会停止它（StopHardware 走禁用端点、冲刷 FIFO 的序列）。
    // The EP0 IN transfer this SETUP interrupts keeps its software state here: it is
    // still enabled in the hardware with a packet in its FIFO, and the core stops it when
    // it handles the SETUP (StopHardware runs the disable and FIFO flush sequence).

    if (device_.DmaEnabled())
    {
      device_.ReloadSetupPacketCount(!expect_out_data_stage);
    }
    device_.OnSetupPacket(in_isr, setup);
  }
}

void ESP32USBEndpoint::HandleRxData(size_t size)
{
  uint8_t* const dst = (transfer_buffer_ != nullptr)
                           ? (transfer_buffer_ + transfer_actual_size_)
                           : nullptr;
  const size_t remain = (transfer_request_size_ > transfer_actual_size_)
                            ? (transfer_request_size_ - transfer_actual_size_)
                            : 0U;
  const size_t copy_size = std::min(size, remain);
  const size_t drop_size = size - copy_size;

  if (copy_size > 0U)
  {
    ESPUSBDetail::ReadFifoPacket(
        ESPUSBDetail::GetEndpointFifo(
            reinterpret_cast<usb_dwc_dev_t*>(ESPUSBDetail::DWC2_FS_REG_BASE), 0),
        dst, copy_size);
    transfer_actual_size_ += copy_size;
  }

  if (drop_size == 0U)
  {
    return;
  }

  uint8_t sink[64] = {};
  size_t remain_drop = drop_size;
  auto* fifo = ESPUSBDetail::GetEndpointFifo(
      reinterpret_cast<usb_dwc_dev_t*>(ESPUSBDetail::DWC2_FS_REG_BASE), 0);
  while (remain_drop > 0U)
  {
    const size_t chunk = std::min(remain_drop, sizeof(sink));
    ESPUSBDetail::ReadFifoPacket(fifo, sink, chunk);
    remain_drop -= chunk;
  }
}

void ESP32USBEndpoint::ObserveDmaRxStatus(uint8_t pktsts, size_t size)
{
  if (!device_.DmaEnabled() || (GetDirection() != Direction::OUT))
  {
    return;
  }

  if (pktsts != ESPUSBDetail::RX_STATUS_DATA)
  {
    return;
  }

  dma_rx_status_seen_ = true;
  dma_rx_status_bytes_ = std::min(transfer_request_size_, dma_rx_status_bytes_ + size);
}

void ESP32USBEndpoint::ResetTransferState()
{
  transfer_tag_ = 0U;
  transfer_active_ = false;
  transfer_buffer_ = nullptr;
  transfer_hw_buffer_ = nullptr;
  transfer_request_size_ = 0U;
  transfer_actual_size_ = 0U;
  transfer_queued_size_ = 0U;
  dma_rx_status_bytes_ = 0U;
  dma_rx_status_seen_ = false;
  iso_abort_ = false;
}

bool ESP32USBEndpoint::PrepareTransferBuffer(size_t size)
{
  transfer_hw_buffer_ = transfer_buffer_;

  if (!device_.DmaEnabled() || (size == 0U))
  {
    return true;
  }

  // 槽在 ConfigureHardware 里已验证满足 DMA 要求，这里只做 IN 方向的 cache 回写。
  // The slot was verified for DMA in ConfigureHardware; only the IN-direction cache
  // write-back is left to do here.
  if (GetDirection() == Direction::IN)
  {
    return ESPUSBDetail::CacheSyncDmaBuffer(transfer_hw_buffer_, size, true, true);
  }
  return true;
}

bool ESP32USBEndpoint::FinishOutTransfer(size_t actual_size)
{
  transfer_actual_size_ = actual_size;

  if (!device_.DmaEnabled() || (actual_size == 0U))
  {
    return true;
  }

  DEV_ASSERT(transfer_hw_buffer_ != nullptr);

  // 只同步 DMA 实际写入的范围：同步整块槽内存会越过最后一个槽的末尾。
  // Sync only the range the DMA actually wrote: syncing the whole slot memory runs past
  // the end of the last slot.
  const size_t sync = ESPUSBDetail::AlignUp(actual_size, ESPUSBDetail::USB_DMA_ALIGNMENT);
  return ESPUSBDetail::CacheSyncDmaBuffer(transfer_hw_buffer_, sync, false);
}

size_t ESP32USBEndpoint::GetRemainingTransferSize() const
{
  auto* dev = reinterpret_cast<usb_dwc_dev_t*>(ESPUSBDetail::DWC2_FS_REG_BASE);
  const uint8_t ep_num = EPNumberToInt8(GetNumber());

  if (GetDirection() == Direction::IN)
  {
    if (ep_num == 0U)
    {
      return dev->dieptsiz0_reg.xfersize;
    }
    return dev->in_eps[ep_num - 1U].dieptsiz_reg.xfersize;
  }

  if (ep_num == 0U)
  {
    return dev->doeptsiz0_reg.xfersize;
  }
  return dev->out_eps[ep_num - 1U].doeptsiz_reg.xfersize;
}

size_t ESP32USBEndpoint::GetCompletedTransferSize() const
{
  if ((GetDirection() == Direction::OUT) && device_.DmaEnabled() && dma_rx_status_seen_)
  {
    return dma_rx_status_bytes_;
  }

  const size_t remaining = GetRemainingTransferSize();
  return (remaining >= transfer_request_size_) ? 0U
                                               : (transfer_request_size_ - remaining);
}

void ESP32USBEndpoint::ActivateHardwareEndpoint()
{
  auto* dev = reinterpret_cast<usb_dwc_dev_t*>(ESPUSBDetail::DWC2_FS_REG_BASE);
  const uint8_t ep_num = EPNumberToInt8(GetNumber());
  auto& cfg = GetConfig();

  if (GetDirection() == Direction::IN)
  {
    if (ep_num == 0U)
    {
      usb_dwc_diepctl0_reg_t ctl = {};
      ctl.mps = ESPUSBDetail::EncodeEp0Mps(cfg.max_packet_size);
      ctl.usbactep = 1;
      ctl.eptype = static_cast<uint32_t>(cfg.type);
      ctl.txfnum = 0;
      dev->diepctl0_reg.val = ctl.val;
    }
    else
    {
      usb_dwc_diepctl_reg_t ctl = {};
      ctl.mps = cfg.max_packet_size;
      ctl.usbactep = 1;
      ctl.eptype = static_cast<uint32_t>(cfg.type);
      ctl.txfnum = ep_num;
      ctl.setd0pid = 1;
      dev->in_eps[ep_num - 1U].diepctl_reg.val = ctl.val;
    }
    dev->daintmsk_reg.val |= (1UL << ep_num);
  }
  else
  {
    if (ep_num == 0U)
    {
      usb_dwc_doepctl0_reg_t ctl = {};
      ctl.mps = ESPUSBDetail::EncodeEp0Mps(cfg.max_packet_size);
      ctl.usbactep = 1;
      ctl.eptype = static_cast<uint32_t>(cfg.type);
      dev->doepctl0_reg.val = ctl.val;
    }
    else
    {
      usb_dwc_doepctl_reg_t ctl = {};
      ctl.mps = cfg.max_packet_size;
      ctl.usbactep = 1;
      ctl.eptype = static_cast<uint32_t>(cfg.type);
      ctl.setd0pid = 1;
      dev->out_eps[ep_num - 1U].doepctl_reg.val = ctl.val;
    }
    dev->daintmsk_reg.val |= (1UL << (16U + ep_num));
  }
}

void ESP32USBEndpoint::ProgramTransfer(size_t size)
{
  auto* dev = reinterpret_cast<usb_dwc_dev_t*>(ESPUSBDetail::DWC2_FS_REG_BASE);
  const uint8_t ep_num = EPNumberToInt8(GetNumber());
  const uint16_t pkt_count = ESPUSBDetail::PacketCount(size, MaxPacketSize());
  // 等时端点只在帧号奇偶与端点的奇偶位相同的帧里传输，每次传输都要设置：安排在下一帧。
  // An isochronous endpoint transfers only in a frame whose parity equals the parity bit
  // of the endpoint, which is programmed for each transfer: schedule it for the next
  // frame.
  const bool iso = GetConfig().type == Type::ISOCHRONOUS;
  const bool odd_frame = (dev->dsts_reg.soffn & 1U) != 0U;

  if (GetDirection() == Direction::IN)
  {
    if (ep_num == 0U)
    {
      dev->dieptsiz0_reg.xfersize = static_cast<uint32_t>(size);
      dev->dieptsiz0_reg.pktcnt = pkt_count;
      if (device_.DmaEnabled() && (size > 0U))
      {
        dev->diepdma0_reg.dmaaddr =
            static_cast<uint32_t>(reinterpret_cast<uintptr_t>(transfer_hw_buffer_));
      }
      ESPUSBDetail::StartEndpointTransfer(dev->diepctl0_reg);
    }
    else
    {
      auto& tsiz = dev->in_eps[ep_num - 1U].dieptsiz_reg;
      tsiz.xfersize = static_cast<uint32_t>(size);
      tsiz.pktcnt = pkt_count;
      tsiz.mc = 1U;
      if (device_.DmaEnabled() && (size > 0U))
      {
        dev->in_eps[ep_num - 1U].diepdma_reg.dmaddr =
            static_cast<uint32_t>(reinterpret_cast<uintptr_t>(transfer_hw_buffer_));
      }
      if (iso)
      {
        if (odd_frame)
        {
          dev->in_eps[ep_num - 1U].diepctl_reg.setd0pid = 1;
        }
        else
        {
          dev->in_eps[ep_num - 1U].diepctl_reg.setd1pid = 1;
        }
      }
      ESPUSBDetail::StartEndpointTransfer(dev->in_eps[ep_num - 1U].diepctl_reg);
    }

    if (!device_.DmaEnabled() && (size > 0U))
    {
      WriteMoreTxData();
      if ((ep_num != 0U) && (transfer_queued_size_ < transfer_request_size_))
      {
        dev->diepempmsk_reg.val |= (1UL << ep_num);
      }
    }
  }
  else
  {
    if (ep_num == 0U)
    {
      // 整字一次写入：EP0 OUT 此时可能正使能着等 SETUP，中间值不能落到寄存器里。
      // Written as one word: EP0 OUT may be enabled for SETUP packets at this point, and
      // no intermediate value may reach the register.
      usb_dwc_doeptsiz0_reg_t tsiz = {};
      tsiz.xfersize = static_cast<uint32_t>((size == 0U) ? MaxPacketSize() : size);
      tsiz.pktcnt = pkt_count;
      dev->doeptsiz0_reg.val = tsiz.val;
      if (device_.DmaEnabled() && (size > 0U))
      {
        dev->doepdma0_reg.dmaaddr =
            static_cast<uint32_t>(reinterpret_cast<uintptr_t>(transfer_hw_buffer_));
      }
      ESPUSBDetail::StartEndpointTransfer(dev->doepctl0_reg);
    }
    else
    {
      auto& tsiz = dev->out_eps[ep_num - 1U].doeptsiz_reg;
      tsiz.xfersize = static_cast<uint32_t>(size);
      tsiz.pktcnt = pkt_count;
      tsiz.rxdpid = 0U;
      if (device_.DmaEnabled() && (size > 0U))
      {
        dev->out_eps[ep_num - 1U].doepdma_reg.dmaaddr =
            static_cast<uint32_t>(reinterpret_cast<uintptr_t>(transfer_hw_buffer_));
      }
      if (iso)
      {
        if (odd_frame)
        {
          dev->out_eps[ep_num - 1U].doepctl_reg.setd0pid = 1;
        }
        else
        {
          dev->out_eps[ep_num - 1U].doepctl_reg.setd1pid = 1;
        }
      }
      ESPUSBDetail::StartEndpointTransfer(dev->out_eps[ep_num - 1U].doepctl_reg);
    }
  }
}

void ESP32USBEndpoint::WriteMoreTxData()
{
  if (transfer_buffer_ == nullptr || transfer_queued_size_ >= transfer_request_size_)
  {
    return;
  }

  auto* dev = reinterpret_cast<usb_dwc_dev_t*>(ESPUSBDetail::DWC2_FS_REG_BASE);
  const uint8_t ep_num = EPNumberToInt8(GetNumber());
  const volatile uint32_t* fifo = ESPUSBDetail::GetEndpointFifo(dev, ep_num);

  while (transfer_queued_size_ < transfer_request_size_)
  {
    uint16_t fifo_words = 0U;
    if (ep_num == 0U)
    {
      fifo_words = dev->dtxfsts0_reg.ineptxfspcavail;
    }
    else
    {
      fifo_words = dev->in_eps[ep_num - 1U].dtxfsts_reg.ineptxfspcavail;
    }

    const size_t remaining = transfer_request_size_ - transfer_queued_size_;
    const size_t packet_size = std::min(remaining, static_cast<size_t>(MaxPacketSize()));
    if (packet_size > static_cast<size_t>(fifo_words) * 4U)
    {
      break;
    }

    ESPUSBDetail::WriteFifoPacket(const_cast<volatile uint32_t*>(fifo),
                                  transfer_buffer_ + transfer_queued_size_, packet_size);
    transfer_queued_size_ += packet_size;
  }
}

}  // namespace LibXR

#endif
