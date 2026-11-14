#include "fsdev.hpp"

namespace LibXR::USB
{
// ---------------------------------------------------------------------------
// 端点 / Endpoint
// ---------------------------------------------------------------------------

FSDevEndpoint::FSDevEndpoint(FSDevDevice& device, EPNumber number, Direction direction,
                             RawData buffer, uint16_t pma_addr, size_t pma_size,
                             uint8_t slot_count)
    : Endpoint(number, direction, buffer, slot_count),
      device_(device),
      pma_addr_(pma_addr),
      pma_size_(pma_size)
{
  REQUIRE(EPNumberToInt8(GetNumber()) < FSDevDevice::MAX_EPS);
  if (direction == Direction::IN)
  {
    device_.in_[EPNumberToInt8(GetNumber())] = this;
  }
  else
  {
    device_.out_[EPNumberToInt8(GetNumber())] = this;
  }
}

void FSDevEndpoint::SetStat(bool in, uint32_t target)
{
  const uint8_t num = Num();
  const uint32_t stat_mask = in ? FSDEV_EP_STAT_TX : FSDEV_EP_STAT_RX;
  const uint32_t ctr = in ? FSDEV_EP_CTR_TX : FSDEV_EP_CTR_RX;
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
  const uint32_t no_clear =
      FSDEV_EP_CTR_RX | FSDEV_EP_CTR_TX | device_.ep_err_rx_ | device_.ep_err_tx_;
  for (unsigned attempt = 0U; attempt < 3U; ++attempt)
  {
    const uint32_t cur = device_.ReadEp(num);
    if (arming && (cur & ctr) != 0U)
    {
      // 本方向事务已完成，由 CTR 处理推进。
      // The transaction of this direction has completed; the CTR handling goes on.
      return;
    }
    const uint32_t value =
        (cur & device_.ep_keep_mask_) | ((cur ^ target) & stat_mask) | no_clear;
    device_.WriteEp(num, value);
    const uint32_t after = device_.ReadEp(num);
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

void FSDevEndpoint::WriteType(uint32_t bits)
{
  const uint8_t num = Num();
  const uint32_t cur = device_.ReadEp(num);
  // TYPE/EA 在保持掩码内：先抹掉旧值再写新的。CTR/ERR 写 1 保持（同 SetStat）。
  // TYPE/EA are inside the keep mask: clear the old value before writing the new one.
  // CTR/ERR keep on a written 1 (as in SetStat).
  const uint32_t no_clear =
      FSDEV_EP_CTR_RX | FSDEV_EP_CTR_TX | device_.ep_err_rx_ | device_.ep_err_tx_;
  const uint32_t value =
      (cur & device_.ep_keep_mask_ & ~(FSDEV_EP_TYPE | FSDEV_EP_KIND | FSDEV_EP_ADDR)) |
      bits | Num() | no_clear;
  device_.WriteEp(num, value);
}

void FSDevEndpoint::ClearDataToggle(bool in)
{
  const uint8_t num = Num();
  const uint32_t dtog = in ? FSDEV_EP_DTOG_TX : FSDEV_EP_DTOG_RX;
  // CTR/ERR 写 1 保持（同 SetStat）。
  // CTR/ERR keep on a written 1 (as in SetStat).
  const uint32_t no_clear =
      FSDEV_EP_CTR_RX | FSDEV_EP_CTR_TX | device_.ep_err_rx_ | device_.ep_err_tx_;
  const uint32_t cur = device_.ReadEp(num);
  if ((cur & dtog) != 0U)
  {
    device_.WriteEp(num, (cur & device_.ep_keep_mask_) | dtog | no_clear);
  }
}

void FSDevEndpoint::ArmTx(size_t bytes)
{
  device_.SetTxCount(Num(), static_cast<uint16_t>(bytes));
  SetStat(true, FSDEV_STAT_TX_VALID);
}

void FSDevEndpoint::ArmRx()
{
  device_.SetRxCountEncoded(Num(), MaxPacketSize());
  SetStat(false, FSDEV_STAT_RX_VALID);
}

size_t FSDevEndpoint::MaxPacketSizeLimit(Type type) const
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

ErrorCode FSDevEndpoint::ConfigureHardware(const Config& cfg)
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
    device_.SetTxBuf(num, pma_addr_);
  }
  else
  {
    device_.SetRxBuf(num, pma_addr_);
  }
  // 重新绑定从 DATA0 开始；EP0 的 DATA0/1 由硬件在 SETUP 后自动设置。
  // A new binding starts at DATA0; the hardware sets the EP0 DATA0/1 after SETUP.
  ClearDataToggle(true);
  ClearDataToggle(false);
  SetStat(IsIn(), IsIn() ? FSDEV_STAT_TX_NAK : FSDEV_STAT_RX_NAK);
  return ErrorCode::OK;
}

void FSDevEndpoint::CloseHardware(bool in_isr)
{
  UNUSED(in_isr);
  hw_tag_ = 0U;
  if (Num() == 0U)
  {
    return;  // EP0 常驻。
  }
  SetStat(IsIn(), IsIn() ? FSDEV_STAT_TX_DIS : FSDEV_STAT_RX_DIS);
}

ErrorCode FSDevEndpoint::StartHardware(const HwRequest& request, bool in_isr)
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
      device_.PmaWrite(pma_addr_, hw_data_, first);
    }
    ArmTx(first);
  }
  else
  {
    ArmRx();
  }
  return ErrorCode::OK;
}

Endpoint::StopResult FSDevEndpoint::StopHardware(bool in_isr)
{
  UNUSED(in_isr);
  const uint8_t num = Num();
  const uint32_t cur = device_.ReadEp(num);
  if (IsIn())
  {
    // 最后一个包可能刚被 ACK 而中断还没处理：计入进度。
    // The last packet may have just been ACKed while the interrupt has not run yet: count
    // it as progress.
    if ((cur & FSDEV_EP_CTR_TX) != 0U)
    {
      hw_progress_ +=
          LibXR::min(static_cast<size_t>(MaxPacketSize()), hw_length_ - hw_progress_);
      device_.ClearCtrTx(num);
    }
    SetStat(true, FSDEV_STAT_TX_NAK);
    // SetStat 的读改写窗口里硬件可能又 ACK 完一包（CTR 已锁存）：补计一次。
    // In the read-modify-write window of SetStat the hardware may ACK one more packet
    // (CTR latched): count it once more.
    if ((device_.ReadEp(num) & FSDEV_EP_CTR_TX) != 0U)
    {
      hw_progress_ +=
          LibXR::min(static_cast<size_t>(MaxPacketSize()), hw_length_ - hw_progress_);
      device_.ClearCtrTx(num);
    }
    stopped_actual_ = hw_progress_;
  }
  else
  {
    // 收到但未处理的包读进槽并计入。
    // A packet received but not handled yet is read into the slot and counted.
    if ((cur & FSDEV_EP_CTR_RX) != 0U)
    {
      const size_t bytes = device_.GetRxCount(num);
      const size_t room = hw_length_ > hw_progress_ ? hw_length_ - hw_progress_ : 0U;
      const size_t take = LibXR::min(bytes, room);
      if (take != 0U)
      {
        device_.PmaRead(const_cast<uint8_t*>(hw_data_) + hw_progress_, pma_addr_, take);
      }
      hw_progress_ += take;
      device_.ClearCtrRx(num);
    }
    SetStat(false, FSDEV_STAT_RX_NAK);
    // SetStat 的读改写窗口里硬件可能又收完一包（CTR 已锁存）：补收补计一次。
    // In the read-modify-write window of SetStat the hardware may receive one more packet
    // (CTR latched): receive and count it once more.
    if ((device_.ReadEp(num) & FSDEV_EP_CTR_RX) != 0U)
    {
      const size_t bytes = device_.GetRxCount(num);
      const size_t room = hw_length_ > hw_progress_ ? hw_length_ - hw_progress_ : 0U;
      const size_t take = LibXR::min(bytes, room);
      if (take != 0U)
      {
        device_.PmaRead(const_cast<uint8_t*>(hw_data_) + hw_progress_, pma_addr_, take);
      }
      hw_progress_ += take;
      device_.ClearCtrRx(num);
    }
    stopped_actual_ = hw_progress_;
  }
  hw_tag_ = 0U;
  return StopResult::DONE;
}

size_t FSDevEndpoint::ReapProgress(const HwRequest& request, bool& retired)
{
  UNUSED(request);
  retired = false;
  return stopped_actual_;
}

ErrorCode FSDevEndpoint::StallHardware(bool in_isr)
{
  UNUSED(in_isr);
  SetStat(IsIn(), IsIn() ? FSDEV_STAT_TX_STALL : FSDEV_STAT_RX_STALL);
  return ErrorCode::OK;
}

ErrorCode FSDevEndpoint::ClearStallHardware(bool in_isr)
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

void FSDevEndpoint::OnTxPacket()
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
    device_.PmaWrite(pma_addr_, hw_data_ + hw_progress_, next);
    ArmTx(next);
  }
  else
  {
    hw_tag_ = 0U;
    OnHardwareComplete(true, tag, hw_length_);
  }
}

void FSDevEndpoint::OnRxPacket(size_t bytes)
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
    device_.PmaRead(const_cast<uint8_t*>(hw_data_) + hw_progress_, pma_addr_, take);
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

FSDevDevice::FSDevDevice(
    const MemoryMap& memory_map, const std::initializer_list<EPConfig>& rx_ep_cfgs,
    const std::initializer_list<EPConfig>& tx_ep_cfgs,
    DeviceDescriptor::PacketSize0 packet_size, uint16_t vid, uint16_t pid, uint16_t bcd,
    const std::initializer_list<const DescriptorStrings::LanguagePack*>& lang_list,
    const std::initializer_list<const std::initializer_list<ConfigDescriptorItem*>>&
        configs,
    ConstRawData uid, size_t pma_limit, uint8_t slot_count)
    : EndpointPool(rx_ep_cfgs.size() + tx_ep_cfgs.size()),
      DeviceCore<FullSpeedCapabilities>(*this, USBSpec::USB_2_1, Speed::FULL, packet_size,
                                        vid, pid, bcd, lang_list, configs, uid),
      memory_map_(memory_map)
{
  REQUIRE(rx_ep_cfgs.size() > 0U && rx_ep_cfgs.size() <= MAX_EPS);
  REQUIRE(tx_ep_cfgs.size() > 0U && tx_ep_cfgs.size() <= MAX_EPS);
  REQUIRE(memory_map_.alignment != 0U);
  const size_t limit = pma_limit != 0U ? pma_limit : memory_map_.packet_memory_size;

  // 一个程序只允许一个 FSDEV 设备对象：self_ 是 Dispatch 的静态入口。
  // A program may have one FSDEV device object only: self_ is the static entry of
  // Dispatch.
  REQUIRE(self_ == nullptr);
  self_ = this;

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
  REQUIRE((offset % memory_map_.alignment) == 0U);
  for (size_t i = 0; i < rx_ep_cfgs.size(); ++i)
  {
    REQUIRE((rx[i].pma_size % memory_map_.alignment) == 0U);
    addr_out[i] = static_cast<uint16_t>(offset);
    offset += rx[i].pma_size;
  }
  for (size_t i = 0; i < tx_ep_cfgs.size(); ++i)
  {
    REQUIRE((tx[i].pma_size % memory_map_.alignment) == 0U);
    addr_in[i] = static_cast<uint16_t>(offset);
    offset += tx[i].pma_size;
  }
  REQUIRE(offset <= limit);

  auto* ep0_out =
      new FSDevEndpoint(*this, Endpoint::EPNumber::EP0, Endpoint::Direction::OUT,
                        rx[0].buffer, addr_out[0], rx[0].pma_size, slot_count);
  auto* ep0_in =
      new FSDevEndpoint(*this, Endpoint::EPNumber::EP0, Endpoint::Direction::IN,
                        tx[0].buffer, addr_in[0], tx[0].pma_size, slot_count);
  SetEndpoint0(ep0_in, ep0_out);

  for (size_t i = 1; i < rx_ep_cfgs.size(); ++i)
  {
    Put(new FSDevEndpoint(*this, static_cast<Endpoint::EPNumber>(i),
                          Endpoint::Direction::OUT, rx[i].buffer, addr_out[i],
                          rx[i].pma_size, slot_count));
  }
  for (size_t i = 1; i < tx_ep_cfgs.size(); ++i)
  {
    Put(new FSDevEndpoint(*this, static_cast<Endpoint::EPNumber>(i),
                          Endpoint::Direction::IN, tx[i].buffer, addr_in[i],
                          tx[i].pma_size, slot_count));
  }
}

FSDevDevice::~FSDevDevice()
{
  if (self_ == this)
  {
    self_ = nullptr;
  }
}

void FSDevDevice::InitController()
{
  InitRegs();
  for (uint8_t i = 0; i < MAX_EPS; ++i)
  {
    if (out_[i] != nullptr)
    {
      SetRxBuf(i, out_[i]->pma_addr_);
    }
  }
  for (uint8_t i = 0; i < MAX_EPS; ++i)
  {
    if (in_[i] != nullptr)
    {
      SetTxBuf(i, in_[i]->pma_addr_);
    }
  }

  // 中断源保持关闭，Start() 才打开；上拉在 Start() 最后接通。
  // The interrupt sources stay off until Start(); the pull-up comes on last in Start().
  WriteCntr(static_cast<uint16_t>(ReadCntr() & ~(FSDEV_CNTR_RUN_MASK | FSDEV_CNTR_SOFM)));
  controller_ready_ = true;
}

void FSDevDevice::Start(bool in_isr)
{
  UNUSED(in_isr);
  // 派生类的构造函数没有调用 InitController()。
  // The constructor of the derived class did not call InitController().
  REQUIRE(controller_ready_);
  {
    EndpointPool::HardwareScope hardware(this);
    WriteCntr(static_cast<uint16_t>(ReadCntr() | FSDEV_CNTR_RUN_MASK | FSDEV_CNTR_SOFM));
    // 上拉最后接通：前面的初始化不产生主机可见的变化。
    // The pull-up comes on last: the initialization before it changes nothing the host
    // can see.
    Connect(true);
  }
}

void FSDevDevice::Stop(bool in_isr)
{
  UNUSED(in_isr);
  {
    EndpointPool::HardwareScope hardware(this);
    Connect(false);
    WriteCntr(
        static_cast<uint16_t>(ReadCntr() & ~(FSDEV_CNTR_RUN_MASK | FSDEV_CNTR_SOFM)));
  }
  // 下拉之后主机不再通信；在途请求的完成事件不会再来，由下一次解绑或复位流程结束。
  // After the pull-up is off the host no longer talks; completions of requests in
  // flight never arrive, and the next unbind or reset ends them.
}

ErrorCode FSDevDevice::SetAddress(uint8_t address, ControlContext context)
{
  // 经典型在状态阶段 IN 完成后写 DADDR（RM0008 §23.4.5）。
  // The classic type writes DADDR after the status-stage IN completes (RM0008
  // §23.4.5).
  if (context == ControlContext::STATUS_IN_COMPLETE)
  {
    EndpointPool::HardwareScope hardware(this);
    WriteDaddr(static_cast<uint16_t>(0x0080U | address));
  }
  return ErrorCode::OK;
}

void FSDevDevice::Dispatch()
{
  FSDevDevice* device = self_;
  if (device == nullptr || !device->IsInited())
  {
    return;
  }
  EndpointPool::InterruptScope batch(*device);
  device->HandleInterrupt();
}

void FSDevDevice::ClearCtrTx(uint8_t num)
{
  const uint32_t self = FSDEV_EP_CTR_TX | ep_err_tx_;
  const uint32_t other = FSDEV_EP_CTR_RX | ep_err_rx_;
  const uint32_t cur = ReadEp(num);
  WriteEp(num, (cur & ep_keep_mask_ & ~self) | other);
}

void FSDevDevice::ClearCtrRx(uint8_t num)
{
  const uint32_t self = FSDEV_EP_CTR_RX | ep_err_rx_;
  const uint32_t other = FSDEV_EP_CTR_TX | ep_err_tx_;
  const uint32_t cur = ReadEp(num);
  WriteEp(num, (cur & ep_keep_mask_ & ~self) | other);
}

// ---------------------------------------------------------------------------
// 经典型控制器的寄存器、BTABLE 与 PMA / Registers, BTABLE and PMA of the classic
// controllers
// ---------------------------------------------------------------------------

uint32_t FSDevDevice::ReadRegister(uint32_t offset)
{
  // NOLINTNEXTLINE(performance-no-int-to-ptr)
  return *reinterpret_cast<volatile uint16_t*>(memory_map_.registers + offset);
}

void FSDevDevice::WriteRegister(uint32_t offset, uint32_t value)
{
  // NOLINTNEXTLINE(performance-no-int-to-ptr)
  *reinterpret_cast<volatile uint16_t*>(memory_map_.registers + offset) =
      static_cast<uint16_t>(value);
}

volatile uint16_t* FSDevDevice::BtField(uint8_t num, unsigned field) const
{
  // BTABLE = 0：字段 m 是第 8n+2m 个字节处的 16 位字，它的地址再乘 word_stride。
  // BTABLE = 0: field m is the 16-bit word at byte 8n+2m, and its address is further
  // multiplied by word_stride.
  // NOLINTNEXTLINE(performance-no-int-to-ptr)
  return reinterpret_cast<volatile uint16_t*>(
      memory_map_.packet_memory +
      (static_cast<uintptr_t>(num) * 8U + 2U * field) * memory_map_.word_stride);
}

void FSDevDevice::SetTxBuf(uint8_t num, uint16_t addr)
{
  *BtField(num, 0U) = static_cast<uint16_t>(addr & 0xFFFEU);
}

void FSDevDevice::SetTxCount(uint8_t num, uint16_t bytes) { *BtField(num, 1U) = bytes; }

void FSDevDevice::SetRxBuf(uint8_t num, uint16_t addr)
{
  *BtField(num, 2U) = static_cast<uint16_t>(addr & 0xFFFEU);
}

void FSDevDevice::SetRxCountEncoded(uint8_t num, uint16_t mps)
{
  // COUNT_RX 的块编码：不超过 62 B 按 2 B 的块，超过按 32 B 的块（BL_SIZE = 1）。
  // Block encoding of COUNT_RX: blocks of 2 B up to 62 B, blocks of 32 B above that
  // (BL_SIZE = 1).
  uint16_t encoded = 0U;
  if (mps == 0U)
  {
    encoded = 0x8000U;  // BL_SIZE，0 块 / BL_SIZE with no block
  }
  else if (mps <= 62U)
  {
    encoded = static_cast<uint16_t>(((mps + 1U) / 2U) << 10U);
  }
  else
  {
    encoded = static_cast<uint16_t>(0x8000U | ((((mps + 31U) / 32U) - 1U) << 10U));
  }
  *BtField(num, 3U) = encoded;
}

uint16_t FSDevDevice::GetRxCount(uint8_t num)
{
  return static_cast<uint16_t>(*BtField(num, 3U) & 0x03FFU);
}

void FSDevDevice::PmaWrite(uint16_t addr, const uint8_t* src, size_t len)
{
  // 间隔取到局部变量里：循环里每次写 PMA 之后编译器都得重新读成员。
  // The stride is taken into a local: inside the loop the compiler would have to read the
  // member again after every write to the PMA.
  const size_t stride = memory_map_.word_stride;
  // NOLINTNEXTLINE(performance-no-int-to-ptr)
  volatile uint16_t* p = reinterpret_cast<volatile uint16_t*>(
      memory_map_.packet_memory + static_cast<uintptr_t>(addr) * stride);
  for (size_t i = 0; i < len; i += 2)
  {
    uint16_t word = src[i];
    if (i + 1 < len)
    {
      word |= static_cast<uint16_t>(src[i + 1]) << 8U;
    }
    *p = word;
    p += stride;
  }
}

void FSDevDevice::PmaRead(uint8_t* dst, uint16_t addr, size_t len)
{
  // 间隔取到局部变量里，原因同 PmaWrite / The stride is taken into a local, as in
  // PmaWrite
  const size_t stride = memory_map_.word_stride;
  // NOLINTNEXTLINE(performance-no-int-to-ptr)
  const volatile uint16_t* p = reinterpret_cast<const volatile uint16_t*>(
      memory_map_.packet_memory + static_cast<uintptr_t>(addr) * stride);
  for (size_t i = 0; i < len; i += 2)
  {
    const uint16_t word = *p;
    p += stride;
    dst[i] = static_cast<uint8_t>(word & 0xFFU);
    if (i + 1 < len)
    {
      dst[i + 1] = static_cast<uint8_t>(word >> 8U);
    }
  }
}

void FSDevDevice::InitRegs()
{
  WriteCntr(FSDEV_CNTR_FRES);  // 强复位并顺带清 PDWN / Force reset, clears PDWN too
  WriteCntr(0U);
  // 直接写 0 清除全部挂起标志；ClearIstr(0) 写的是全 1（保持），含义不同。
  // Write 0 directly to clear every pending flag; ClearIstr(0) writes all ones (keep),
  // which means something else.
  WriteRegister(FSDEV_REG_ISTR, 0U);
  WriteRegister(FSDEV_REG_BTABLE, 0U);
}

void FSDevDevice::SuspendRegs()
{
  WriteCntr(static_cast<uint16_t>(ReadCntr() | FSDEV_CNTR_FSUSP));
}

void FSDevDevice::SuspendClocked() {}

void FSDevDevice::ResumeRegs()
{
  WriteCntr(static_cast<uint16_t>(ReadCntr() & ~(FSDEV_CNTR_LPMODE | FSDEV_CNTR_FSUSP)));
}

void FSDevDevice::HandleInterrupt()
{
  // 噪声位只清不处理，并且是按进入时的快照清的：掩码里混进一个有处理分支的标志时，
  // 快照之后才锁存的那次事件会被直接清掉。
  // The noise flags are only cleared, and cleared from the snapshot taken on entry: a
  // handled flag inside the mask would wipe an event that latched after the snapshot.
  static_assert((FSDEV_ISTR_NOISE & (FSDEV_ISTR_CTR | FSDEV_ISTR_RESET | FSDEV_ISTR_SOF |
                                     FSDEV_ISTR_SUSP | FSDEV_ISTR_WKUP)) == 0U,
                "ISTR_NOISE overlaps an interrupt flag that has a handler");
  const uint16_t istr = ReadIstr();
  if ((istr & FSDEV_ISTR_RESET) != 0U)
  {
    ClearIstr(FSDEV_ISTR_RESET);
    OnBusResetRegs();
  }
  if ((istr & FSDEV_ISTR_CTR) != 0U)
  {
    HandleCtr();
  }
  if ((istr & FSDEV_ISTR_SOF) != 0U)
  {
    ClearIstr(FSDEV_ISTR_SOF);
    HandleSof();
  }
  if ((istr & FSDEV_ISTR_SUSP) != 0U)
  {
    // 先进挂起再清标志、最后接时钟门控（DRD 变体要求的顺序）；经典型的
    // SuspendClocked 为空。
    // Enter suspend first, then clear the flags, then gate the clocks (the order the
    // DRD variant requires); SuspendClocked is empty on the classic type.
    WriteCntr(static_cast<uint16_t>(ReadCntr() & ~FSDEV_CNTR_SOFM));
    SuspendRegs();
    ClearIstr(FSDEV_ISTR_SUSP);
    SuspendClocked();
    OnSuspend(true);
  }
  if ((istr & FSDEV_ISTR_WKUP) != 0U)
  {
    ResumeRegs();
    WriteCntr(static_cast<uint16_t>(ReadCntr() | FSDEV_CNTR_SOFM));
    ClearIstr(FSDEV_ISTR_WKUP);
    OnResume(true);
  }
  if ((istr & FSDEV_ISTR_NOISE) != 0U)
  {
    ClearIstr(FSDEV_ISTR_NOISE);
  }
}

void FSDevDevice::HandleCtr()
{
  // 一次中断只报一个端点；带上界循环，剩余事件由再次进入的中断处理。
  // One interrupt reports one endpoint; loop with a bound, the next interrupt entry
  // handles the remaining events.
  for (unsigned pass = 0; pass < 8U; ++pass)
  {
    const uint16_t istr = ReadIstr();
    if ((istr & FSDEV_ISTR_CTR) == 0U)
    {
      break;
    }
    HandleEp(static_cast<uint8_t>(istr & 0x000FU));
    ClearIstr(FSDEV_ISTR_CTR);
  }
}

void FSDevDevice::HandleEp(uint8_t num)
{
  const uint32_t epr = ReadEp(num);
  const bool ctr_tx = (epr & FSDEV_EP_CTR_TX) != 0U;
  const bool ctr_rx = (epr & FSDEV_EP_CTR_RX) != 0U;
  const bool setup = (epr & FSDEV_EP_SETUP) != 0U;

  // 先 TX 后 RX：SETUP 会打断当前控制传输，之前的完成事件先走完。
  // TX before RX: a SETUP aborts the current control transfer, so earlier completions go
  // first.
  if (ctr_tx)
  {
    ClearCtrTx(num);
    FSDevEndpoint* ep = num < MAX_EPS ? in_[num] : nullptr;
    if (ep != nullptr && !discard_)
    {
      ep->OnTxPacket();
    }
  }
  if (ctr_rx)
  {
    ClearCtrRx(num);
    if (num == 0U && setup)
    {
      // 新 SETUP 打断在途控制传输；在途请求由核心的结束流程回收（ABORTED）。
      // A new SETUP aborts the control transfer in flight; the core's end path reclaims
      // the request in flight (ABORTED).
      PmaRead(setup_, out_[0]->pma_addr_, 8U);
      SetupPacket packet;
      std::memcpy(&packet, setup_, sizeof(packet));
      OnSetupPacket(true, &packet);
    }
    else
    {
      FSDevEndpoint* ep = num < MAX_EPS ? out_[num] : nullptr;
      if (ep != nullptr && !discard_)
      {
        ep->OnRxPacket(GetRxCount(num));
      }
      // discard_：旧会话的包读掉丢弃（count 由下一次挂接收重写）。
      // discard_: a packet of the old session is read and dropped (the next receive
      // arm rewrites the count).
    }
  }
}

void FSDevDevice::OnBusResetRegs()
{
  EndpointPool::HardwareScope hardware(this);
  discard_ = true;
  // 硬件已清全部 EPnR（STAT = DISABLED）；地址归零，EP0 准备收 SETUP。
  // The hardware has cleared every EPnR (STAT = DISABLED); the address goes to zero and
  // EP0 gets ready for SETUP.
  WriteDaddr(0x0080U);  // EF | 0
  if (out_[0] != nullptr)
  {
    // EP0 OUT NAK：SETUP 照常接收 / EP0 OUT answers NAK; a SETUP is still received.
    out_[0]->SetStat(false, FSDEV_STAT_RX_NAK);
  }
  // 复位把总线从挂起中唤醒，帧中断随之打开。
  // A reset wakes the bus from suspend, so the frame interrupt is on again.
  WriteCntr(static_cast<uint16_t>(ReadCntr() | FSDEV_CNTR_SOFM));
  OnBusReset(true);
}

void FSDevDevice::HandleSof() { OnSof(true, static_cast<uint16_t>(ReadFnr() & 0x07FFU)); }

}  // namespace LibXR::USB
