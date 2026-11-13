/**
 * @file fsdev.tpp
 * @brief fsdev.hpp 里两个类模板的实现，由 fsdev.hpp 在末尾包含 / Implementation of
 *        the two class templates of fsdev.hpp, included at the end of fsdev.hpp.
 */

#pragma once

namespace LibXR::USB
{
// ---------------------------------------------------------------------------
// 端点 / Endpoint
// ---------------------------------------------------------------------------

template <typename HW>
void FSDevEndpoint<HW>::SetStat(bool in, uint32_t target)
{
  const uint8_t num = Num();
  const uint32_t stat_mask = in ? HW::EP_STAT_TX : HW::EP_STAT_RX;
  const uint32_t ctr = in ? HW::EP_CTR_TX : HW::EP_CTR_RX;
  const uint32_t valid = in ? FSDEV_STAT_TX_VALID : FSDEV_STAT_RX_VALID;
  const bool arming = (target == valid);
  // 写 1 翻转：只写到达目标的增量。读与写（以及写与读回）
  // 之间硬件可能自己完成一次事务（结束 IN/OUT 并自行把 STAT 改回 NAK、锁存 CTR），
  // 按旧值算增量会把 STAT 翻错。所以写后回读核对，不对就按新值重算重写（有界）。
  // CTR/ERR 是 rc_w0：这些写永远写 1 保持，窗口里锁存的 CTR 不能被抹掉。
  // 装填（目标 VALID）时若发现本方向 CTR 已锁存，说明写入已生效且这笔事务已经完成——STAT
  // 是硬件自动回的 NAK，绝不能重试：重试会把端点再次装填（IN 用同一份 PMA 数据重发一包、
  // OUT 多收一包）。完成的事务交给 CTR 处理推进。停止类目标（NAK/STALL/DISABLED）
  // 没有二次装填问题，窗口里完成的事务只会把 STAT 推回 NAK，按新值重算重写即可收敛，
  // 不能提前放弃—— 否则停止路径等于没停住。
  // Write-1-to-toggle: write only the delta that reaches the target. Between the read and
  // the write (and between the write and the read-back) the hardware may finish a
  // transaction by itself (end the IN/OUT, set STAT back to NAK, latch CTR), and a delta
  // from the old value would toggle STAT wrongly. So read back after the write and, if it
  // does not match, recompute from the new value and write again (bounded).
  // CTR/ERR are rc_w0: these writes always write 1 to keep them, so a CTR latched in the
  // window is never wiped. When arming (target VALID) and the CTR of this direction is
  // already latched, the write took effect and the transaction has completed: STAT is the
  // NAK the hardware set itself and must never be retried, since a retry would arm the
  // endpoint again (IN resends a packet from the same PMA data, OUT receives one packet
  // more). The completed transaction is advanced by the CTR handling. Stop targets
  // (NAK/STALL/DISABLED) cannot arm twice: a transaction completed in the window only
  // moves STAT back to NAK, so recomputing from the new value and writing again
  // converges and must not give up early, or the stop path would not stop.
  constexpr uint32_t NO_CLEAR =
      HW::EP_CTR_RX | HW::EP_CTR_TX | HW::EP_ERR_RX | HW::EP_ERR_TX;
  for (unsigned attempt = 0U; attempt < 3U; ++attempt)
  {
    const uint32_t cur = HW::ReadEp(num);
    if (arming && (cur & ctr) != 0U)
    {
      // 本方向事务已完成，由 CTR 处理推进。
      // The transaction of this direction has completed; the CTR handling goes on.
      return;
    }
    const uint32_t value =
        (cur & HW::EP_KEEP_MASK) | ((cur ^ target) & stat_mask) | NO_CLEAR;
    HW::WriteEp(num, value);
    const uint32_t after = HW::ReadEp(num);
    if ((after & stat_mask) == target)
    {
      return;
    }
    if (arming && (after & ctr) != 0U)
    {
      // 装填后事务在窗口内完成：STAT 是硬件自动回的 NAK，不重试。
      // The transaction completed within the window after the load: STAT is the NAK
      // the hardware set by itself, so there is no retry.
      return;
    }
  }
  // 三次重算都没到达目标且（装填时）CTR 未锁存：读回路径异常，
  // NAK/装填语义已不可靠（停止路径等于没停住），不能静默继续。
  // Three recomputations did not reach the target and (when arming) no CTR is latched:
  // the read-back path is broken and the NAK/arm semantics no longer hold (the stop path
  // would not stop), so it must not continue silently.
  REQUIRE(false);
}

template <typename HW>
void FSDevEndpoint<HW>::WriteType(uint32_t bits)
{
  const uint8_t num = Num();
  const uint32_t cur = HW::ReadEp(num);
  // TYPE/EA 在保持掩码内：先抹掉旧值再写新的。CTR/ERR 写 1 保持（同 SetStat）。
  // TYPE/EA are inside the keep mask: clear the old value before writing the new one.
  // CTR/ERR keep on a written 1 (as in SetStat).
  constexpr uint32_t NO_CLEAR =
      HW::EP_CTR_RX | HW::EP_CTR_TX | HW::EP_ERR_RX | HW::EP_ERR_TX;
  const uint32_t value =
      (cur & HW::EP_KEEP_MASK & ~(HW::EP_TYPE | HW::EP_KIND | HW::EP_ADDR)) | bits |
      Num() | NO_CLEAR;
  HW::WriteEp(num, value);
}

template <typename HW>
void FSDevEndpoint<HW>::ClearDataToggle(bool in)
{
  const uint8_t num = Num();
  const uint32_t dtog = in ? HW::EP_DTOG_TX : HW::EP_DTOG_RX;
  // CTR/ERR 写 1 保持（同 SetStat）。
  // CTR/ERR keep on a written 1 (as in SetStat).
  constexpr uint32_t NO_CLEAR =
      HW::EP_CTR_RX | HW::EP_CTR_TX | HW::EP_ERR_RX | HW::EP_ERR_TX;
  const uint32_t cur = HW::ReadEp(num);
  if ((cur & dtog) != 0U)
  {
    HW::WriteEp(num, (cur & HW::EP_KEEP_MASK) | dtog | NO_CLEAR);
  }
}

template <typename HW>
void FSDevEndpoint<HW>::ArmTx(size_t bytes)
{
  HW::SetTxCount(Num(), static_cast<uint16_t>(bytes));
  SetStat(true, FSDEV_STAT_TX_VALID);
}

template <typename HW>
void FSDevEndpoint<HW>::ArmRx()
{
  HW::SetRxCountEncoded(Num(), MaxPacketSize());
  SetStat(false, FSDEV_STAT_RX_VALID);
}

template <typename HW>
size_t FSDevEndpoint<HW>::MaxPacketSizeLimit(Type type) const
{
  size_t cap = pma_size_;
  // OUT 的 COUNT_RX 编码按 2 B / 32 B 块取整。
  // The COUNT_RX encoding of OUT rounds to 2 B / 32 B blocks.
  if (!IsIn() && cap > 62U)
  {
    cap = (cap / 32U) * 32U;
  }
  return type == Type::CONTROL ? LibXR::min<size_t>(cap, 64U) : cap;
}

template <typename HW>
ErrorCode FSDevEndpoint<HW>::ConfigureHardware(const Config& cfg)
{
  if (cfg.type == Type::ISOCHRONOUS)
  {
    return ErrorCode::NOT_SUPPORT;
  }

  // 同一端点号 IN、OUT 共用 EP_TYPE。
  // IN and OUT of one endpoint number share EP_TYPE.
  FSDevEndpoint* other = IsIn() ? device_.out_[Num()] : device_.in_[Num()];
  if (other != nullptr)
  {
    const State state = other->GetState();
    if ((state == State::IDLE || state == State::STALLED) && other->GetType() != cfg.type)
    {
      return ErrorCode::FAILED;
    }
  }

  hw_tag_ = 0U;
  hw_data_ = nullptr;
  hw_length_ = 0U;
  hw_progress_ = 0U;
  stopped_actual_ = 0U;

  const uint8_t num = Num();
  WriteType(FsDevTypeBits(cfg.type));
  if (IsIn())
  {
    HW::SetTxBuf(num, pma_addr_);
  }
  else
  {
    HW::SetRxBuf(num, pma_addr_);
  }
  // 重新绑定从 DATA0 开始；EP0 的 DATA0/1 由硬件在 SETUP 后自动设置。
  // A new binding starts at DATA0; the hardware sets the EP0 DATA0/1 after SETUP.
  ClearDataToggle(true);
  ClearDataToggle(false);
  SetStat(IsIn(), IsIn() ? FSDEV_STAT_TX_NAK : FSDEV_STAT_RX_NAK);
  return ErrorCode::OK;
}

template <typename HW>
void FSDevEndpoint<HW>::CloseHardware(bool in_isr)
{
  UNUSED(in_isr);
  hw_tag_ = 0U;
  if (Num() == 0U)
  {
    return;  // EP0 常驻。
  }
  SetStat(IsIn(), IsIn() ? FSDEV_STAT_TX_DIS : FSDEV_STAT_RX_DIS);
}

template <typename HW>
ErrorCode FSDevEndpoint<HW>::StartHardware(const HwRequest& request, bool in_isr)
{
  UNUSED(in_isr);
  hw_tag_ = request.tag;
  hw_data_ = static_cast<const uint8_t*>(request.slot.addr_);
  hw_length_ = request.length;
  hw_progress_ = 0U;
  stopped_actual_ = 0U;
  if (IsIn())
  {
    // 第一个包现在就进 PMA；多包的其余部分在 CTR_TX 中推进。
    // The first packet goes to the PMA now; CTR_TX advances the rest of a multi-packet
    // transfer.
    const size_t first = LibXR::min(request.length, static_cast<size_t>(MaxPacketSize()));
    if (first != 0U)
    {
      HW::PmaWrite(pma_addr_, hw_data_, first);
    }
    ArmTx(first);
  }
  else
  {
    ArmRx();
  }
  return ErrorCode::OK;
}

template <typename HW>
Endpoint::StopResult FSDevEndpoint<HW>::StopHardware(bool in_isr)
{
  UNUSED(in_isr);
  const uint8_t num = Num();
  const uint32_t cur = HW::ReadEp(num);
  if (IsIn())
  {
    // 最后一个包可能刚被 ACK 而中断还没处理：计入进度。
    // The last packet may have just been ACKed while the interrupt has not run yet: count
    // it as progress.
    if ((cur & HW::EP_CTR_TX) != 0U)
    {
      hw_progress_ +=
          LibXR::min(static_cast<size_t>(MaxPacketSize()), hw_length_ - hw_progress_);
      FsDevClearCtrTx<HW>(num);
    }
    SetStat(true, FSDEV_STAT_TX_NAK);
    // SetStat 的读改写窗口里硬件可能又 ACK 完一包（CTR 已锁存）：补计一次。
    // In the read-modify-write window of SetStat the hardware may ACK one more packet
    // (CTR latched): count it once more.
    if ((HW::ReadEp(num) & HW::EP_CTR_TX) != 0U)
    {
      hw_progress_ +=
          LibXR::min(static_cast<size_t>(MaxPacketSize()), hw_length_ - hw_progress_);
      FsDevClearCtrTx<HW>(num);
    }
    stopped_actual_ = hw_progress_;
  }
  else
  {
    // 收到但未处理的包读进槽并计入。
    // A packet received but not handled yet is read into the slot and counted.
    if ((cur & HW::EP_CTR_RX) != 0U)
    {
      const size_t bytes = HW::GetRxCount(num);
      const size_t room = hw_length_ > hw_progress_ ? hw_length_ - hw_progress_ : 0U;
      const size_t take = LibXR::min(bytes, room);
      if (take != 0U)
      {
        HW::PmaRead(const_cast<uint8_t*>(hw_data_) + hw_progress_, pma_addr_, take);
      }
      hw_progress_ += take;
      FsDevClearCtrRx<HW>(num);
    }
    SetStat(false, FSDEV_STAT_RX_NAK);
    // SetStat 的读改写窗口里硬件可能又收完一包（CTR 已锁存）：补收补计一次。
    // In the read-modify-write window of SetStat the hardware may receive one more packet
    // (CTR latched): receive and count it once more.
    if ((HW::ReadEp(num) & HW::EP_CTR_RX) != 0U)
    {
      const size_t bytes = HW::GetRxCount(num);
      const size_t room = hw_length_ > hw_progress_ ? hw_length_ - hw_progress_ : 0U;
      const size_t take = LibXR::min(bytes, room);
      if (take != 0U)
      {
        HW::PmaRead(const_cast<uint8_t*>(hw_data_) + hw_progress_, pma_addr_, take);
      }
      hw_progress_ += take;
      FsDevClearCtrRx<HW>(num);
    }
    stopped_actual_ = hw_progress_;
  }
  hw_tag_ = 0U;
  return StopResult::DONE;
}

template <typename HW>
size_t FSDevEndpoint<HW>::ReapProgress(const HwRequest& request, bool& retired)
{
  UNUSED(request);
  retired = false;
  return stopped_actual_;
}

template <typename HW>
ErrorCode FSDevEndpoint<HW>::StallHardware(bool in_isr)
{
  UNUSED(in_isr);
  SetStat(IsIn(), IsIn() ? FSDEV_STAT_TX_STALL : FSDEV_STAT_RX_STALL);
  return ErrorCode::OK;
}

template <typename HW>
ErrorCode FSDevEndpoint<HW>::ClearStallHardware(bool in_isr)
{
  UNUSED(in_isr);
  if (Num() == 0U)
  {
    // 下一个 SETUP 清 EP0 的 STALL 与 DATA0/1。
    // The next SETUP clears the STALL and the DATA0/1 state of EP0.
    return ErrorCode::OK;
  }
  ClearDataToggle(IsIn());  // 恢复 DATA0
  SetStat(IsIn(), IsIn() ? FSDEV_STAT_TX_NAK : FSDEV_STAT_RX_NAK);
  return ErrorCode::OK;
}

template <typename HW>
void FSDevEndpoint<HW>::OnTxPacket()
{
  const uint32_t tag = hw_tag_;
  if (tag == 0U)
  {
    // 已停止，或重复事件 / Already stopped, or a repeated event.
    return;
  }
  const size_t packet = MaxPacketSize();
  const size_t sent = LibXR::min(packet, hw_length_ - hw_progress_);
  hw_progress_ += sent;
  if (hw_progress_ < hw_length_)
  {
    const size_t next = LibXR::min(packet, hw_length_ - hw_progress_);
    HW::PmaWrite(pma_addr_, hw_data_ + hw_progress_, next);
    ArmTx(next);
  }
  else
  {
    hw_tag_ = 0U;
    OnHardwareComplete(true, tag, hw_length_);
  }
}

template <typename HW>
void FSDevEndpoint<HW>::OnRxPacket(size_t bytes)
{
  const uint32_t tag = hw_tag_;
  if (tag == 0U)
  {
    return;
  }
  const size_t packet = MaxPacketSize();
  const size_t room = hw_length_ > hw_progress_ ? hw_length_ - hw_progress_ : 0U;
  const size_t take = LibXR::min(bytes, room);
  if (take != 0U)
  {
    HW::PmaRead(const_cast<uint8_t*>(hw_data_) + hw_progress_, pma_addr_, take);
  }
  hw_progress_ += take;
  // 短包或收满请求长度即完成（bulk OUT 语义；核心保证请求长度是包长整数倍）。
  // A short packet or a full request length completes it (bulk OUT semantics; the core
  // keeps the request length a multiple of the packet size).
  if (bytes < packet || hw_progress_ >= hw_length_)
  {
    hw_tag_ = 0U;
    OnHardwareComplete(true, tag, hw_progress_);
  }
  else
  {
    ArmRx();
  }
}

// ---------------------------------------------------------------------------
// 设备 / Device
// ---------------------------------------------------------------------------

template <typename HW>
FSDevDevice<HW>::FSDevDevice(
    const std::initializer_list<EPConfig>& rx_ep_cfgs,
    const std::initializer_list<EPConfig>& tx_ep_cfgs,
    DeviceDescriptor::PacketSize0 packet_size, uint16_t vid, uint16_t pid, uint16_t bcd,
    const std::initializer_list<const DescriptorStrings::LanguagePack*>& lang_list,
    const std::initializer_list<const std::initializer_list<ConfigDescriptorItem*>>&
        configs,
    ConstRawData uid, size_t pma_limit, uint8_t slot_count)
    : EndpointPool(rx_ep_cfgs.size() + tx_ep_cfgs.size()),
      DeviceCore<FullSpeedCapabilities>(*this, USBSpec::USB_2_1, Speed::FULL, packet_size,
                                        vid, pid, bcd, lang_list, configs, uid)
{
  REQUIRE(rx_ep_cfgs.size() > 0U && rx_ep_cfgs.size() <= HW::ENDPOINT_COUNT);
  REQUIRE(tx_ep_cfgs.size() > 0U && tx_ep_cfgs.size() <= HW::ENDPOINT_COUNT);
  const size_t limit = pma_limit != 0U ? pma_limit : HW::PMA_SIZE;

  SetHardwareGuard(
      this, [](void*) -> uintptr_t { return HW::EnterGuard(); },
      [](void*, uintptr_t saved) { HW::ExitGuard(saved); });

  // 同一 HW 类型只允许一个设备实例：self_ 是 Dispatch 的静态入口。
  // Only one device instance per HW type: self_ is the static entry of Dispatch.
  REQUIRE(self_ == nullptr);
  self_ = this;

  HW::MspInit();
  HW::InitRegs();

  // BTABLE 之后按端点号依次分配：EP0 OUT、EP0 IN、EP1 OUT、EP1 IN、...
  // Allocated after the BTABLE in endpoint order: EP0 OUT, EP0 IN, EP1 OUT, EP1 IN, ...
  EPConfig rx[MAX_EPS] = {};
  EPConfig tx[MAX_EPS] = {};
  uint16_t addr_out[MAX_EPS] = {};
  uint16_t addr_in[MAX_EPS] = {};
  for (size_t i = 0; i < rx_ep_cfgs.size(); ++i)
  {
    rx[i] = rx_ep_cfgs.begin()[i];
  }
  for (size_t i = 0; i < tx_ep_cfgs.size(); ++i)
  {
    tx[i] = tx_ep_cfgs.begin()[i];
  }
  endpoint_count_ = static_cast<uint8_t>(
      rx_ep_cfgs.size() > tx_ep_cfgs.size() ? rx_ep_cfgs.size() : tx_ep_cfgs.size());

  size_t offset = static_cast<size_t>(endpoint_count_) * 8U;
  // BTABLE 段每端点 8 B 天然对齐；各缓冲起点必须落在对齐边界上（经典型
  // RX/TX 地址写 ADDR 字段按偶数截断，DRD 按 4 字节截断），否则会静默截断、
  // 后一个缓冲与前面重叠。
  // The BTABLE takes 8 B per endpoint and stays aligned; every buffer must start on an
  // aligned boundary (the classic type truncates the RX/TX ADDR field to even, DRD to 4
  // bytes), otherwise the address is silently truncated and a buffer overlaps the one
  // before it.
  REQUIRE((offset % HW::PMA_ALIGN) == 0U);
  for (size_t i = 0; i < rx_ep_cfgs.size(); ++i)
  {
    REQUIRE((rx[i].pma_size % HW::PMA_ALIGN) == 0U);
    addr_out[i] = static_cast<uint16_t>(offset);
    offset += rx[i].pma_size;
  }
  for (size_t i = 0; i < tx_ep_cfgs.size(); ++i)
  {
    REQUIRE((tx[i].pma_size % HW::PMA_ALIGN) == 0U);
    addr_in[i] = static_cast<uint16_t>(offset);
    offset += tx[i].pma_size;
  }
  REQUIRE(offset <= limit);

  auto* ep0_out =
      new FSDevEndpoint<HW>(*this, Endpoint::EPNumber::EP0, Endpoint::Direction::OUT,
                            rx[0].buffer, addr_out[0], rx[0].pma_size, slot_count);
  auto* ep0_in =
      new FSDevEndpoint<HW>(*this, Endpoint::EPNumber::EP0, Endpoint::Direction::IN,
                            tx[0].buffer, addr_in[0], tx[0].pma_size, slot_count);
  SetEndpoint0(ep0_in, ep0_out);

  for (size_t i = 1; i < rx_ep_cfgs.size(); ++i)
  {
    Put(new FSDevEndpoint<HW>(*this, static_cast<Endpoint::EPNumber>(i),
                              Endpoint::Direction::OUT, rx[i].buffer, addr_out[i],
                              rx[i].pma_size, slot_count));
  }
  for (size_t i = 1; i < tx_ep_cfgs.size(); ++i)
  {
    Put(new FSDevEndpoint<HW>(*this, static_cast<Endpoint::EPNumber>(i),
                              Endpoint::Direction::IN, tx[i].buffer, addr_in[i],
                              tx[i].pma_size, slot_count));
  }

  for (size_t i = 0; i < rx_ep_cfgs.size(); ++i)
  {
    HW::SetRxBuf(static_cast<uint8_t>(i), addr_out[i]);
  }
  for (size_t i = 0; i < tx_ep_cfgs.size(); ++i)
  {
    HW::SetTxBuf(static_cast<uint8_t>(i), addr_in[i]);
  }

  // 中断源保持关闭，Start() 才打开；上拉在 Start() 最后接通。
  // The interrupt sources stay off until Start(); the pull-up comes on last in Start().
  HW::WriteCntr(
      static_cast<uint16_t>(HW::ReadCntr() & ~(HW::CNTR_RUN_MASK | HW::CNTR_SOFM)));
}

template <typename HW>
void FSDevDevice<HW>::HandleInterrupt()
{
  // 噪声位只清不处理，并且是按进入时的快照清的：掩码里混进一个有处理分支的标志时，
  // 快照之后才锁存的那次事件会被直接清掉。
  // The noise flags are only cleared, and cleared from the snapshot taken on entry: a
  // handled flag inside the mask would wipe an event that latched after the snapshot.
  static_assert((HW::ISTR_NOISE & (HW::ISTR_CTR | HW::ISTR_RESET | HW::ISTR_SOF |
                                   HW::ISTR_SUSP | HW::ISTR_WKUP)) == 0U,
                "ISTR_NOISE overlaps an interrupt flag that has a handler");
  const uint16_t istr = HW::ReadIstr();
  if ((istr & HW::ISTR_RESET) != 0U)
  {
    HW::ClearIstr(HW::ISTR_RESET);
    OnBusResetRegs();
  }
  if ((istr & HW::ISTR_CTR) != 0U)
  {
    HandleCtr();
  }
  if ((istr & HW::ISTR_SOF) != 0U)
  {
    HW::ClearIstr(HW::ISTR_SOF);
    HandleSof();
  }
  if ((istr & HW::ISTR_SUSP) != 0U)
  {
    // 先进挂起再清标志、最后接时钟门控（DRD 变体要求的顺序）；经典型的
    // SuspendClocked 为空。
    // Enter suspend first, then clear the flags, then gate the clocks (the order the
    // DRD variant requires); SuspendClocked is empty on the classic type.
    HW::SuspendRegs();
    HW::ClearIstr(HW::ISTR_SUSP);
    HW::SuspendClocked();
    OnSuspend(true);
  }
  if ((istr & HW::ISTR_WKUP) != 0U)
  {
    HW::ResumeRegs();
    HW::ClearIstr(HW::ISTR_WKUP);
    OnResume(true);
  }
  if ((istr & HW::ISTR_NOISE) != 0U)
  {
    HW::ClearIstr(HW::ISTR_NOISE);
  }
}

template <typename HW>
void FSDevDevice<HW>::HandleCtr()
{
  // 一次中断只报一个端点；带上界循环，剩余事件由再次进入的中断处理。
  // One interrupt reports one endpoint; loop with a bound, the next interrupt entry
  // handles the remaining events.
  for (unsigned pass = 0; pass < 8U; ++pass)
  {
    const uint16_t istr = HW::ReadIstr();
    if ((istr & HW::ISTR_CTR) == 0U)
    {
      break;
    }
    HandleEp(static_cast<uint8_t>(istr & 0x000FU));
    HW::ClearIstr(HW::ISTR_CTR);
  }
}

template <typename HW>
void FSDevDevice<HW>::HandleEp(uint8_t num)
{
  const uint32_t epr = HW::ReadEp(num);
  const bool ctr_tx = (epr & HW::EP_CTR_TX) != 0U;
  const bool ctr_rx = (epr & HW::EP_CTR_RX) != 0U;
  const bool setup = (epr & HW::EP_SETUP) != 0U;

  // 先 TX 后 RX：SETUP 会打断当前控制传输，之前的完成事件先走完。
  // TX before RX: a SETUP aborts the current control transfer, so earlier completions go
  // first.
  if (ctr_tx)
  {
    FsDevClearCtrTx<HW>(num);
    FSDevEndpoint<HW>* ep = num < MAX_EPS ? in_[num] : nullptr;
    if (ep != nullptr && !discard_)
    {
      ep->OnTxPacket();
    }
  }
  if (ctr_rx)
  {
    FsDevClearCtrRx<HW>(num);
    if (num == 0U && setup)
    {
      // 新 SETUP 打断在途控制传输；在途请求由核心的结束流程回收（ABORTED）。
      // A new SETUP aborts the control transfer in flight; the core's end path reclaims
      // the request in flight (ABORTED).
      HW::PmaRead(setup_, out_[0]->pma_addr_, 8U);
      SetupPacket packet;
      std::memcpy(&packet, setup_, sizeof(packet));
      OnSetupPacket(true, &packet);
    }
    else
    {
      FSDevEndpoint<HW>* ep = num < MAX_EPS ? out_[num] : nullptr;
      if (ep != nullptr && !discard_)
      {
        ep->OnRxPacket(HW::GetRxCount(num));
      }
      // discard_：旧会话的包读掉丢弃（count 由下一次挂接收重写）。
      // discard_: a packet of the old session is read and dropped (the next receive
      // arm rewrites the count).
    }
  }
}

template <typename HW>
void FSDevDevice<HW>::OnBusResetRegs()
{
  EndpointPool::HardwareScope hardware(this);
  discard_ = true;
  // 硬件已清全部 EPnR（STAT = DISABLED）；地址归零，EP0 准备收 SETUP。
  // The hardware has cleared every EPnR (STAT = DISABLED); the address goes to zero and
  // EP0 gets ready for SETUP.
  HW::WriteDaddr(0x0080U);  // EF | 0
  if (out_[0] != nullptr)
  {
    // EP0 OUT NAK：SETUP 照常接收 / EP0 OUT answers NAK; a SETUP is still received.
    out_[0]->SetStat(false, FSDEV_STAT_RX_NAK);
  }
  OnBusReset(true);
}

template <typename HW>
void FSDevDevice<HW>::HandleSof()
{
  OnSof(true, static_cast<uint16_t>(HW::ReadFnr() & 0x07FFU));
}

}  // namespace LibXR::USB
