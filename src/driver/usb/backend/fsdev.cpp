#include "fsdev.hpp"

#include <cstring>

#include "timebase.hpp"

namespace LibXR::USB
{

namespace
{

/// EP_TYPE 编码：CONTROL=01、ISO=10、BULK=00、INT=11 / EP_TYPE encoding
constexpr uint32_t TypeBits(Endpoint::Type type)
{
  switch (type)
  {
    case Endpoint::Type::CONTROL:
      return 0x0200U;
    case Endpoint::Type::ISOCHRONOUS:
      return 0x0400U;
    case Endpoint::Type::INTR:
      return 0x0600U;
    case Endpoint::Type::BULK:
    default:
      return 0x0000U;
  }
}

/// 换掉 EPnR 写入值里的类型字段 / Replace the type field of an EPnR value to be written
constexpr uint32_t WithType(uint32_t value, Endpoint::Type type)
{
  return (value & ~FSDEV_EP_TYPE) | TypeBits(type);
}

/// EP0 上有未处理的 SETUP：CTR_RX 与 SETUP 同时置位 / EP0 has an unhandled SETUP:
/// CTR_RX and SETUP both set
bool SetupPending(uint32_t epr)
{
  return (epr & (FSDEV_EP_CTR_RX | FSDEV_EP_SETUP)) == (FSDEV_EP_CTR_RX | FSDEV_EP_SETUP);
}

/**
 * @brief COUNT_RX 的块编码 / Block encoding of COUNT_RX
 * @note 不超过 62 字节按 2 字节块，否则按 32 字节块（BL_SIZE = 1）。
 *       2-byte blocks up to 62 bytes, 32-byte blocks above (BL_SIZE = 1).
 */
uint16_t EncodeRxSize(uint16_t mps)
{
  if (mps == 0U)
  {
    return 0x8000U;
  }
  if (mps <= 62U)
  {
    return static_cast<uint16_t>(((mps + 1U) / 2U) << 10U);
  }
  return static_cast<uint16_t>(0x8000U | ((((mps + 31U) / 32U) - 1U) << 10U));
}

/// 等时：缓冲 b 在 BTABLE 里的计数字段（0 发送位置，1 接收位置）/ Isochronous: the count
/// field of buffer b in the BTABLE (0 the transmit location, 1 the receive location)
constexpr unsigned IsoCountField(uint8_t buffer) { return buffer == 0U ? 1U : 3U; }

}  // namespace

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
  ASSERT(direction == Direction::IN || direction == Direction::OUT);
  ASSERT(EPNumberToInt8(number) < FSDevPool::MAX_EPS);
}

bool FSDevEndpoint::SetStat(uint32_t target)
{
  const uint8_t num = Num();
  const bool in = IsIn();
  const uint32_t stat_mask = in ? FSDEV_EP_STAT_TX : FSDEV_EP_STAT_RX;
  const uint32_t ctr = in ? FSDEV_EP_CTR_TX : FSDEV_EP_CTR_RX;
  const uint32_t nak = in ? FSDEV_STAT_TX_NAK : FSDEV_STAT_RX_NAK;
  bool arming = target == (in ? FSDEV_STAT_TX_VALID : FSDEV_STAT_RX_VALID);
  // STAT 写 1 翻转：按读到的值算增量。读与写之间硬件可能自己结束一次事务（STAT 回
  // NAK、锁存 CTR），所以写后读回，不对就按新值重算（有界）。CTR 和错误位总写 1，窗口
  // 里锁存的不会被抹掉。装填时本方向 CTR 已锁存说明这次装填已被消耗，不能重试，否则
  // 同一份 PMA 数据再发一次或多收一包；完成由 CTR 处理。EP0 有未处理的 SETUP 时硬件已把
  // 两方向置 NAK，装填会撤掉它，所以改写 NAK。
  // STAT toggles on a written 1: the delta is computed from the value read. The
  // hardware may end a transaction between the read and the write (STAT back to NAK,
  // CTR latched), so the value is read back and recomputed from the new one (bounded).
  // CTR and the error bits are always written 1, so one latched in the window survives.
  // When arming, a latched CTR of this direction means the arm was consumed and must
  // not be retried, or the same PMA data would go out again or one packet more would
  // be taken; the CTR handling goes on. With an unhandled SETUP on EP0 the hardware has
  // set both directions to NAK and arming would undo it, so NAK is written instead.
  const uint32_t no_clear =
      FSDEV_EP_CTR_RX | FSDEV_EP_CTR_TX | device_.ep_err_rx_ | device_.ep_err_tx_;
  const uint32_t valid = in ? FSDEV_STAT_TX_VALID : FSDEV_STAT_RX_VALID;
  const uint32_t stall = in ? FSDEV_STAT_TX_STALL : FSDEV_STAT_RX_STALL;
  for (unsigned attempt = 0U; attempt < 3U; ++attempt)
  {
    uint32_t cur = device_.ReadEp(num);
    if (arming && (cur & ctr) != 0U)
    {
      return true;
    }
    if (arming && num == 0U && SetupPending(cur))
    {
      arming = false;
      target = nak;
    }
    // 把一个 VALID 的数据端点改成别的状态时，先把它的地址字段换成没有端点使用的号，
    // 写完再换回来。读与写之间硬件可能结束一个事务、自己回了 NAK，下面的翻转就会把这
    // 个 NAK 翻回 VALID；改回去之前主机的下一个令牌若被接受，IN 会把同一包再发一遍，
    // OUT 会多收一包（数字见 FSDevEndpoint 的类说明）。地址不匹配时控制器不理会令牌，
    // 主机当作一次传输错误重试。EP0 不这样做：它要一直能收 SETUP。
    // Before a VALID data endpoint is written to another state, its address field is
    // set to a number no endpoint uses, and set back afterwards. The hardware may end a
    // transaction between the read and the write and set NAK itself, and the toggle
    // below then turns that NAK back into VALID; if the host's next token is accepted
    // before the correction, an IN sends the same packet again and an OUT takes one
    // packet more (figures in the class notes of FSDevEndpoint). With an address that
    // does not match the controller ignores the token and the host retries it as a
    // transfer error. Not done on EP0, which must take a SETUP at any time.
    const bool parked = !arming && num != 0U && (cur & stat_mask) == valid;
    if (parked)
    {
      device_.WriteEp(num, (cur & device_.ep_keep_mask_ & ~FSDEV_EP_ADDR) |
                               FSDEV_EP_PARK_ADDR | no_clear);
      cur = device_.ReadEp(num);
    }
    // BULK 闸（见 EnableEp0BulkGate）：控制写的数据包按 BULK 装填；其余装填和 STALL 在
    // 同一次写里把 EP0 的类型写回 CONTROL。
    // BULK gate (see EnableEp0BulkGate): a data packet of a control write is armed typed
    // BULK; any other arm and a STALL type EP0 CONTROL again in the same write.
    const bool gate = num == 0U && device_.ep0_bulk_gate_ && (arming || target == stall);
    const bool bulk = gate && arming && !in && device_.ep0_out_data_;
    const bool control = gate && !bulk;
    uint32_t keep = cur & device_.ep_keep_mask_;
    if (gate)
    {
      keep = WithType(keep, bulk ? Type::BULK : Type::CONTROL);
    }
    device_.WriteEp(num, keep | ((cur ^ target) & stat_mask) | no_clear);
    uint32_t after = device_.ReadEp(num);
    if (!arming && (after & stat_mask) != target)
    {
      // 读与写之间硬件结束了一个事务、自己回了 NAK，上面那次翻转就把 NAK 又翻成了别的值
      // （停止时是 VALID：端点被重新放行）。按刚读回的值立刻再写一次，中间不再读。EP0
      // 没有换地址，全靠这次补写够快。
      // The hardware ended a transaction between the read and the write and set NAK
      // itself, so the toggle above turned that NAK into something else (VALID for a
      // stop: the endpoint was released again). The value just read back is used for an
      // immediate second write with no read in between. EP0 has no address change and
      // relies on this second write being quick.
      uint32_t again = after & device_.ep_keep_mask_;
      if (control)
      {
        again = WithType(again, Type::CONTROL);
      }
      device_.WriteEp(num, again | ((after ^ target) & stat_mask) | no_clear);
      after = device_.ReadEp(num);
    }
    if (parked)
    {
      device_.WriteEp(num,
                      (after & device_.ep_keep_mask_ & ~FSDEV_EP_ADDR) | num | no_clear);
      after = device_.ReadEp(num);
    }
    if (arming && num == 0U && SetupPending(after))
    {
      arming = false;
      target = nak;
      continue;
    }
    if ((after & stat_mask) == target || (arming && (after & ctr) != 0U))
    {
      return true;
    }
  }
  return false;
}

void FSDevEndpoint::WriteType(uint32_t bits)
{
  const uint8_t num = Num();
  const uint32_t cur = device_.ReadEp(num);
  // 类型、KIND、地址在保持掩码内：先抹掉旧值再写；CTR 和错误位写 1 保持。
  // Type, KIND and address are inside the keep mask: clear the old value, then write;
  // CTR and the error bits keep on a written 1.
  const uint32_t no_clear =
      FSDEV_EP_CTR_RX | FSDEV_EP_CTR_TX | device_.ep_err_rx_ | device_.ep_err_tx_;
  device_.WriteEp(num, (cur & device_.ep_keep_mask_ &
                        ~(FSDEV_EP_TYPE | FSDEV_EP_KIND | FSDEV_EP_ADDR)) |
                           bits | num | no_clear);
}

void FSDevEndpoint::ClearDataToggle()
{
  const uint8_t num = Num();
  const uint32_t dtog = IsIn() ? FSDEV_EP_DTOG_TX : FSDEV_EP_DTOG_RX;
  const uint32_t no_clear =
      FSDEV_EP_CTR_RX | FSDEV_EP_CTR_TX | device_.ep_err_rx_ | device_.ep_err_tx_;
  const uint32_t cur = device_.ReadEp(num);
  if ((cur & dtog) != 0U)
  {
    device_.WriteEp(num, (cur & device_.ep_keep_mask_) | dtog | no_clear);
  }
}

bool FSDevEndpoint::Arm()
{
  const uint8_t num = Num();
  if (IsIn())
  {
    const size_t bytes = LibXR::min(length_ - progress_, size_t{MaxPacketSize()});
    if (bytes != 0U)
    {
      device_.PmaWrite(pma_addr_, data_ + progress_, bytes);
    }
    device_.SetTxCount(num, static_cast<uint16_t>(bytes));
    return SetStat(FSDEV_STAT_TX_VALID);
  }
  // 接收区大小在配置时已写好，每次装填都是同一个值。EP0 的包可能早到时不再重写：重写
  // 会抹掉已锁存的那个包的长度。
  // The receive size was written at configuration and is the same for every arm. Where
  // packets of EP0 may come early it is not written again: that would wipe the length
  // of a packet already latched.
  if (!EarlyOut())
  {
    device_.SetRxCountEncoded(num, MaxPacketSize());
  }
  return SetStat(FSDEV_STAT_RX_VALID);
}

bool FSDevEndpoint::Completed(uint32_t epr) const
{
  if (IsIn())
  {
    return (epr & FSDEV_EP_CTR_TX) != 0U;
  }
  return (epr & FSDEV_EP_CTR_RX) != 0U && !(Num() == 0U && SetupPending(epr));
}

bool FSDevEndpoint::TakePacket(ErrorCode& result)
{
  const uint8_t num = Num();
  const size_t packet = MaxPacketSize();
  result = ErrorCode::OK;
  if (IsIn())
  {
    progress_ += LibXR::min(packet, length_ - progress_);
    device_.ClearCtrTx(num);
    return progress_ >= length_;
  }
  // 先读 PMA 再清 CTR_RX：CTR_RX 置位期间硬件不收新的 SETUP（RM0008 p.631），EP0 的
  // 缓冲不会在读之前被覆盖。
  // Read the PMA before clearing CTR_RX: while CTR_RX is set the hardware takes no new
  // SETUP (RM0008 p.631), so EP0's buffer is not overwritten before the read.
  const size_t bytes = device_.GetRxCount(num);
  const size_t room = length_ - progress_;
  const size_t take = LibXR::min(bytes, room);
  if (take != 0U)
  {
    device_.PmaRead(data_ + progress_, pma_addr_, take);
  }
  progress_ += take;
  device_.ClearCtrRx(num);
  if (bytes > room)
  {
    result = ErrorCode::OUT_OF_RANGE;
    return true;
  }
  return bytes < packet || progress_ >= length_;
}

void FSDevEndpoint::OnPacket(bool in_isr)
{
  ErrorCode result = ErrorCode::OK;
  if (TakePacket(result))
  {
    busy_ = false;
    OnHardwareComplete(in_isr, progress_, result);
    return;
  }
  if (!Arm() && !HeldInReset())
  {
    busy_ = false;
    OnHardwareComplete(in_isr, progress_, ErrorCode::FAILED);
  }
}

bool FSDevEndpoint::EarlyOut() const
{
  return device_.ep0_early_out_ && Num() == 0U && !IsIn();
}

bool FSDevEndpoint::HeldInReset() const
{
  // 配置过的端点 EPnR 不会是 0（EP0 有类型位，其余有地址位）。读到 0 就是总线复位的
  // SE0 期间控制器把寄存器按在复位值上：这时写不进，RESET 中断要到约 10 ms 之后才来
  // （STM32F302 实测）。这不是硬件错误，请求留在途中，由随后的总线复位结束。
  // A configured endpoint's EPnR is never 0 (EP0 has type bits, the others address
  // bits). Reading 0 means the controller holds its registers at their reset value
  // during the SE0 of a bus reset: writes do not take, and the RESET interrupt comes
  // only about 10 ms later (measured on an STM32F302). This is no hardware error; the
  // request stays in flight and the bus reset that follows ends it.
  return device_.ReadEp(Num()) == 0U;
}

void FSDevEndpoint::OnShadowPacket(bool in_isr, const uint8_t* data, size_t bytes)
{
  if (stopping_ && !stop_failed_)
  {
    // 被停请求的事务完成了：停止结束，这个包不进槽。 / The stopped request's
    // transaction completed: the stop ends and the packet stays out of the slot.
    AdvanceStop(in_isr, 0U, true);
    return;
  }
  if (!busy_)
  {
    return;
  }
  const size_t packet = MaxPacketSize();
  const size_t room = length_ - progress_;
  const size_t take = LibXR::min(bytes, room);
  if (take != 0U)
  {
    std::memcpy(data_ + progress_, data, take);
  }
  progress_ += take;
  if (bytes > room || bytes < packet || progress_ >= length_)
  {
    busy_ = false;
    OnHardwareComplete(in_isr, progress_,
                       bytes > room ? ErrorCode::OUT_OF_RANGE : ErrorCode::OK);
    return;
  }
  if (!Arm() && !HeldInReset())
  {
    busy_ = false;
    OnHardwareComplete(in_isr, progress_, ErrorCode::FAILED);
  }
}

Endpoint::HwLimits FSDevEndpoint::HardwareLimits(Direction direction, Type type) const
{
  size_t size = pma_size_;
  if (direction == Direction::OUT && size > 62U)
  {
    size = size / 32U * 32U;
  }
  size = LibXR::min(size, size_t{1023U});
  if (type == Type::CONTROL)
  {
    size = LibXR::min(size, size_t{64U});
  }
  HwLimits limits;
  limits.max_packet_size = size;
  if (type == Type::ISOCHRONOUS)
  {
    // 缓冲 1 借同号另一方向的 PMA，两块必须一样大；不一样（或没有）时包长上限报 0，
    // Configure 和写描述符都当场拒绝。一个请求一包，一块缓冲一个请求。
    // Buffer 1 borrows the PMA of the other direction of the number and both must be
    // the same size; otherwise (or without one) a packet limit of 0 makes Configure and
    // the descriptor writing refuse at once. One packet per request, one request per
    // buffer.
    const FSDevEndpoint* other = Partner();
    if (other == nullptr || other->pma_size_ != pma_size_)
    {
      limits.max_packet_size = 0U;
    }
    limits.max_packets = 1U;
    limits.max_bytes = limits.max_packet_size;
    limits.max_requests = ISO_REQUESTS;
  }
  // CPU 在槽和 PMA 之间拷贝，硬件不访问槽。
  // The CPU copies between the slot and the PMA; the hardware never accesses the slot.
  limits.direct_memory = false;
  return limits;
}

ErrorCode FSDevEndpoint::CheckHardware(const Config& cfg) const
{
  // 同一端点号的 IN、OUT 共用 EP_TYPE；对方已启用时类型必须相同。等时端点占用整个
  // EPnR 和两个方向的 PMA：对方已启用时不能配置等时，对方是等时时不能配置本方向。
  // IN and OUT of one number share EP_TYPE; the type must match an enabled other half.
  // An isochronous endpoint takes the whole EPnR and the PMA of both directions: no
  // isochronous endpoint while the other half is enabled, and nothing while the other
  // half is isochronous.
  const FSDevEndpoint* other = Partner();
  const bool other_enabled = other != nullptr && other->GetState() != State::DISABLED;
  if ((cfg.type == Type::ISOCHRONOUS && other_enabled) || PartnerIsochronous())
  {
    return ErrorCode::STATE_ERR;
  }
  if (other_enabled && other->GetType() != cfg.type)
  {
    return ErrorCode::FAILED;
  }
  return ErrorCode::OK;
}

void FSDevEndpoint::ConfigureHardware(const Config& cfg)
{
  const uint8_t num = Num();
  WriteType(TypeBits(cfg.type));
  if (cfg.type == Type::ISOCHRONOUS)
  {
    // 缓冲 0 在发送位置，用自己的 PMA；缓冲 1 在接收位置，用另一方向的 PMA（RM0008
    // p.634 表 171）。两块清空，DTOG 指向缓冲 0，STAT 置 11 后一直有效。
    // Buffer 0 at the transmit location with this endpoint's PMA, buffer 1 at the
    // receive location with the other direction's (RM0008 p.634, table 171). Both are
    // emptied, DTOG points at buffer 0, and STAT is set to 11 and stays so.
    device_.SetTxBuf(num, pma_addr_);
    device_.SetRxBuf(num, Partner()->pma_addr_);
    for (uint8_t buffer = 0U; buffer < ISO_REQUESTS; ++buffer)
    {
      if (IsIn())
      {
        device_.SetIsoTxCount(num, buffer, 0U);
      }
      else
      {
        device_.SetIsoRxSize(num, buffer, cfg.max_packet_size);
      }
    }
    iso_ = true;
    iso_head_ = 0U;
    iso_count_ = 0U;
    ClearDataToggle();
    IsoWriteStat(IsIn() ? FSDEV_STAT_TX_VALID : FSDEV_STAT_RX_VALID);
    return;
  }
  if (IsIn())
  {
    device_.SetTxBuf(num, pma_addr_);
  }
  else
  {
    device_.SetRxBuf(num, pma_addr_);
    // 接收区大小在配置时就写好，不等第一次装填：SETUP 在任何装填之前就会到，而描述表在
    // PMA 里，上电后内容不定。
    // The receive size is written at configuration and not left to the first arm: a
    // SETUP arrives before any arm, and the description table lives in the PMA, whose
    // content is undefined after power-on.
    device_.SetRxCountEncoded(num, cfg.max_packet_size);
  }
  // 新的绑定从 DATA0 开始，只动本方向：另一方向可能正在传输。EP0 的 DATA0/1 由硬件在
  // SETUP 时设定（RM0008 p.630），不动。
  // A new binding starts at DATA0, this direction only: the other may be transferring.
  // EP0's DATA0/1 is set by the hardware on each SETUP (RM0008 p.630) and left alone.
  if (num != 0U)
  {
    ClearDataToggle();
  }
  // 写不进只发生在总线复位把 EPnR 按在 0 上的那段时间（见 HeldInReset）；随后的复位
  // 处理会重新配置端点，不算失败。
  // The write can fail to take only while a bus reset holds EPnR at 0 (see
  // HeldInReset); the reset handling that follows configures the endpoint again, so
  // this is no failure.
  const bool written = SetStat(IsIn() ? FSDEV_STAT_TX_NAK : FSDEV_STAT_RX_NAK);
  ASSERT(written || HeldInReset());
  UNUSED(written);
}

void FSDevEndpoint::CloseHardware(bool in_isr)
{
  UNUSED(in_isr);
  if (Num() == 0U)
  {
    return;
  }
  if (iso_)
  {
    // 停止已在 Close 之前做完；同号另一方向从此可以配置。 / The stop was done before
    // Close; the other direction of the number may be configured from now on.
    IsoWriteStat(IsIn() ? FSDEV_STAT_TX_DIS : FSDEV_STAT_RX_DIS);
    iso_ = false;
    iso_count_ = 0U;
    return;
  }
  (void)SetStat(IsIn() ? FSDEV_STAT_TX_DIS : FSDEV_STAT_RX_DIS);
}

ErrorCode FSDevEndpoint::StartHardware(const HwRequest& request, bool in_isr)
{
  UNUSED(in_isr);
  if (iso_)
  {
    // 排在已有的请求之后；IN 有空出的缓冲就拷进去。 / Queued behind the requests already
    // there; an IN is copied into a free buffer if there is one.
    ASSERT(iso_count_ < ISO_REQUESTS);
    IsoRequest& slot = iso_req_[(iso_head_ + iso_count_) % ISO_REQUESTS];
    slot.data = request.data;
    slot.length = request.length;
    slot.buffer = -1;
    ++iso_count_;
    if (IsIn())
    {
      IsoLoad();
    }
    return ErrorCode::OK;
  }
  data_ = request.data;
  length_ = request.length;
  progress_ = 0U;
  // 控制读的状态包：受理请求，STAT_RX 留在 NAK，由 ReleaseEp0Status 放行。
  // The status packet of a control read: the request is accepted, STAT_RX stays NAK and
  // ReleaseEp0Status releases it.
  status_held_ =
      device_.ep0_late_status_ && Num() == 0U && !IsIn() && request.length == 0U;
  if (Completed(device_.ReadEp(Num())))
  {
    if (EarlyOut())
    {
      // 早到的包是这次请求的第一包：不清、不装填，由 CTR 处理收下。
      // The early packet is the first packet of this request: it is neither cleared nor
      // armed over; the CTR handling takes it.
      busy_ = true;
      return ErrorCode::OK;
    }
    // 没有请求时锁存的本方向 CTR 不属于任何请求，清掉，免得装填把它当作这次装填已被
    // 消耗。
    // A CTR of this direction latched while no request was on the hardware belongs to
    // no request; clear it so that arming does not take it for this arm being consumed.
    if (IsIn())
    {
      device_.ClearCtrTx(Num());
    }
    else
    {
      device_.ClearCtrRx(Num());
    }
  }
  if (!status_held_ && !Arm() && !HeldInReset())
  {
    return ErrorCode::FAILED;
  }
  busy_ = true;
  return ErrorCode::OK;
}

Endpoint::StopOutcome FSDevEndpoint::StopHardware(bool in_isr)
{
  UNUSED(in_isr);
  if (iso_)
  {
    return IsoStop();
  }
  const uint8_t num = Num();
  const bool in = IsIn();
  const uint32_t valid = in ? FSDEV_STAT_TX_VALID : FSDEV_STAT_RX_VALID;
  const uint32_t nak = in ? FSDEV_STAT_TX_NAK : FSDEV_STAT_RX_NAK;
  const uint32_t stat_mask = in ? FSDEV_EP_STAT_TX : FSDEV_EP_STAT_RX;
  ErrorCode result = ErrorCode::ABORTED;
  bool ended = false;

  // 1. 已锁存的完成：硬件已自动置 NAK，这一包已经传完。控制器复位之后没有请求，不收。
  //    EP0 的包可能早到时也不收：总线在运行时核心只在 SETUP 之后停 EP0 的请求，而
  //    SETUP 只在 CTR_RX 已清时被收下，所以这时锁存的普通包在 SETUP 之后才到，属于
  //    下一个请求。
  // 1. A latched completion: the hardware has set NAK itself and the packet is done.
  //    After a controller reset there is no request and nothing is collected. Nor where
  //    packets of EP0 may come early: while the bus runs the core stops a request of EP0
  //    only after a SETUP, and a SETUP is taken only with CTR_RX clear, so an ordinary
  //    packet latched now came after the SETUP and belongs to the next request.
  uint32_t epr = device_.ReadEp(num);
  if (busy_ && Completed(epr) && !EarlyOut())
  {
    ended = TakePacket(result);
    epr = device_.ReadEp(num);
  }
  busy_ = false;

  // 2. STAT 不是 VALID：硬件不会为本端点开始事务（RM0008 p.630，推断），已停稳。包完成
  //    后硬件自己回 NAK，SETUP 把 EP0 两方向置 NAK，总线复位和控制器复位把 EPnR 清零。
  // 2. STAT is not VALID: the hardware starts no transaction for this endpoint (RM0008
  //    p.630, inferred), so it is quiescent. The hardware sets NAK itself after a
  //    packet, a SETUP sets both EP0 directions to NAK, and a bus reset or a controller
  //    reset clears EPnR.
  if ((epr & stat_mask) != valid)
  {
    return {ended ? result : ErrorCode::ABORTED, progress_};
  }

  // 3. 写 NAK。写的过程中事务完成的话 CTR 已锁存、硬件已回 NAK：收下，已停稳。写的过程
  //    中 EP0 收到 SETUP 同理。
  // 3. Write NAK. A transaction completing during the write has latched its CTR and the
  //    hardware answers NAK: collect it, quiescent. Likewise a SETUP arriving on EP0
  //    during the write.
  const bool written = SetStat(nak);
  epr = device_.ReadEp(num);
  if (Completed(epr))
  {
    ended = TakePacket(result);
    return {ended ? result : ErrorCode::ABORTED, progress_};
  }
  if (written && ((num == 0U && SetupPending(epr)) || device_.suspended_))
  {
    // 总线挂起时线上没有事务。 / No transaction is on the wire while the bus is
    // suspended.
    return {ErrorCode::ABORTED, progress_};
  }

  // 4. 写 NAK 时 STAT 为 VALID：已开始的事务仍会以 ACK 完成，CTR 之后才出现（F302 实测
  //    3.5-55.1 us），而控制器没有标志说明线上有没有这样的事务。不等：停止未结束，帧号
  //    在写 NAK 之后读，见 AdvanceStop。
  // 4. STAT was VALID when NAK was written: a transaction already started still ends
  //    with ACK and its CTR shows up afterwards (3.5-55.1 us measured on an F302), and
  //    the controller has no flag telling whether such a transaction is on the wire. No
  //    wait: the stop is pending, with the frame number read after the NAK, see
  //    AdvanceStop.
  stopping_ = true;
  nak_written_ = written;
  stop_failed_ = false;
  stop_frame_ = device_.FrameNumber();
  ++device_.pending_stops_;
  return {ErrorCode::PENDING, 0U};
}

void FSDevEndpoint::DropPacket()
{
  if (IsIn())
  {
    device_.ClearCtrTx(Num());
  }
  else
  {
    device_.ClearCtrRx(Num());
  }
}

void FSDevEndpoint::AdvanceStop(bool in_isr, uint16_t frame, bool force)
{
  if (!stopping_ || stop_failed_)
  {
    return;
  }
  const bool in = IsIn();
  if (!nak_written_)
  {
    // 停止时 NAK 没写进。STAT 仍为 VALID 就重写一次，再写不进报 TIMEOUT；硬件已自己
    // 离开 VALID（包完成、SETUP、总线复位）就不用写了。
    // The NAK was not written at the stop. With STAT still VALID it is written once
    // more and TIMEOUT is reported if that fails too; if the hardware has left VALID by
    // itself (a packet completed, a SETUP, a bus reset) nothing needs writing.
    const uint32_t valid = in ? FSDEV_STAT_TX_VALID : FSDEV_STAT_RX_VALID;
    const uint32_t stat_mask = in ? FSDEV_EP_STAT_TX : FSDEV_EP_STAT_RX;
    if ((device_.ReadEp(Num()) & stat_mask) == valid)
    {
      if (!SetStat(in ? FSDEV_STAT_TX_NAK : FSDEV_STAT_RX_NAK))
      {
        stop_failed_ = true;
        OnHardwareComplete(in_isr, 0U, ErrorCode::TIMEOUT);
        return;
      }
      nak_written_ = true;
      stop_frame_ = device_.FrameNumber();
      if (!force)
      {
        // NAK 现在才生效，从这一帧重新算。 / The NAK applies only now; the frame
        // rule restarts from this frame.
        return;
      }
    }
    nak_written_ = true;
  }
  // 事务不跨帧：写 NAK 之后收到过帧起始，写 NAK 时在线上的事务就已经结束；它的 CTR 在
  // 调用者读帧号之前已锁存，下面读得到。
  // Transactions never cross a frame: once a start of frame came after the NAK, the
  // transaction on the wire at the NAK is over; its CTR was latched before the caller
  // read the frame number and is seen by the read below.
  if (!force && frame == stop_frame_)
  {
    return;
  }
  // 期间完成的包只清 CTR：槽已在停止时放回，EP0 OUT 上的 SETUP 留给 HandleEp。
  // A packet completed meanwhile only has its CTR cleared: the slot was given back at
  // the stop, and a SETUP on EP0 OUT is left to HandleEp.
  if (Completed(device_.ReadEp(Num())))
  {
    DropPacket();
  }
  stopping_ = false;
  --device_.pending_stops_;
  OnHardwareComplete(in_isr, progress_, ErrorCode::ABORTED);
}

ErrorCode FSDevEndpoint::StallHardware(bool in_isr)
{
  UNUSED(in_isr);
  if (iso_)
  {
    // 等时没有 STALL：禁用，主机的令牌得不到应答。 / Isochronous has no STALL: disabled,
    // the host's tokens get no answer.
    IsoWriteStat(IsIn() ? FSDEV_STAT_TX_DIS : FSDEV_STAT_RX_DIS);
    return ErrorCode::OK;
  }
  if (Num() == 0U)
  {
    // STALL 结束控制传输：之后清掉的 EP0 包不再按控制写的数据包写 BULK（BULK 闸）。
    // A STALL ends the control transfer: an EP0 packet cleared afterwards no longer
    // types EP0 BULK as a data packet of a control write (BULK gate).
    device_.ep0_out_data_ = false;
  }
  return SetStat(IsIn() ? FSDEV_STAT_TX_STALL : FSDEV_STAT_RX_STALL) ? ErrorCode::OK
                                                                     : ErrorCode::FAILED;
}

ErrorCode FSDevEndpoint::ClearStallHardware(bool in_isr)
{
  UNUSED(in_isr);
  if (Num() == 0U)
  {
    // 下一个 SETUP 把 EP0 两方向置 NAK 并设好 DATA0/1（RM0008 p.630）。
    // The next SETUP sets both EP0 directions to NAK and sets DATA0/1 (RM0008 p.630).
    return ErrorCode::OK;
  }
  const uint8_t num = Num();
  const bool in = IsIn();
  const uint32_t stat_mask = in ? FSDEV_EP_STAT_TX : FSDEV_EP_STAT_RX;
  if (iso_)
  {
    // 只从 00（Stall 之后）恢复：两块清空、DTOG 回到缓冲 0、写 11。已是 11 时不动：
    // 等时没有数据翻转，DTOG 只选缓冲，改它会打乱硬件上两个请求的顺序。
    // Only from 00 (after a Stall): both buffers emptied, DTOG back to buffer 0, 11
    // written. At 11 nothing changes: isochronous has no data toggle, DTOG only selects
    // the buffer, and changing it would reorder the two requests on the hardware.
    if ((device_.ReadEp(num) & stat_mask) == 0U)
    {
      IsoEmpty(0U);
      IsoEmpty(1U);
      ClearDataToggle();
      IsoWriteStat(in ? FSDEV_STAT_TX_VALID : FSDEV_STAT_RX_VALID);
    }
    return ErrorCode::OK;
  }
  const uint32_t valid = in ? FSDEV_STAT_TX_VALID : FSDEV_STAT_RX_VALID;
  const uint32_t dtog = in ? FSDEV_EP_DTOG_TX : FSDEV_EP_DTOG_RX;
  const uint32_t no_clear =
      FSDEV_EP_CTR_RX | FSDEV_EP_CTR_TX | device_.ep_err_rx_ | device_.ep_err_tx_;
  const uint32_t keep = device_.ep_keep_mask_ & ~FSDEV_EP_ADDR;
  uint32_t cur = device_.ReadEp(num);
  if ((cur & stat_mask) != valid)
  {
    // STALL 或 NAK：硬件不会自己结束事务，照旧回到 NAK、DATA0。
    // STALL or NAK: the hardware ends no transaction by itself; back to NAK and DATA0.
    ClearDataToggle();
    return SetStat(in ? FSDEV_STAT_TX_NAK : FSDEV_STAT_RX_NAK) ? ErrorCode::OK
                                                               : ErrorCode::FAILED;
  }
  // 没有 STALL、请求在硬件上（VALID）：只把翻转位改回 DATA0，STAT 不写（写 0 不翻转），
  // 请求留在硬件上。改翻转位期间地址换成停车号，主机的新令牌没有应答、由主机重试；能在
  // 读与写之间结束的只有停车时已在线上的那个事务：它让硬件翻了 DTOG（并回 NAK、锁存
  // CTR，留给完成处理），读回看到就再改一次。
  // Not halted, the request on the hardware (VALID): only the toggle returns to DATA0;
  // STAT is not written (a written 0 does not toggle) and the request stays on the
  // hardware. While the toggle is changed the address is parked, so a new host token
  // gets no answer and the host retries it; only the transaction already on the wire
  // when parking can end between a read and a write: it toggles DTOG (and sets NAK and
  // latches CTR, left to the completion handling), which the read-back sees and fixes.
  device_.WriteEp(num, (cur & keep) | FSDEV_EP_PARK_ADDR | no_clear);
  bool data0 = false;
  for (unsigned attempt = 0U; attempt < 3U; ++attempt)
  {
    cur = device_.ReadEp(num);
    if ((cur & dtog) == 0U)
    {
      data0 = true;
      break;
    }
    device_.WriteEp(num, (cur & keep) | FSDEV_EP_PARK_ADDR | dtog | no_clear);
  }
  device_.WriteEp(num, (cur & keep) | num | no_clear);
  return data0 ? ErrorCode::OK : ErrorCode::FAILED;
}

// ---------------------------------------------------------------------------
// 等时端点 / Isochronous endpoints
// ---------------------------------------------------------------------------

FSDevEndpoint* FSDevEndpoint::Partner() const
{
  const uint8_t num = Num();
  return IsIn() ? device_.out_[num] : device_.in_[num];
}

bool FSDevEndpoint::PartnerIsochronous() const
{
  const FSDevEndpoint* other = Partner();
  return other != nullptr && other->iso_;
}

void FSDevEndpoint::IsoWriteStat(uint32_t target)
{
  const uint8_t num = Num();
  const uint32_t stat_mask = IsIn() ? FSDEV_EP_STAT_TX : FSDEV_EP_STAT_RX;
  const uint32_t no_clear =
      FSDEV_EP_CTR_RX | FSDEV_EP_CTR_TX | device_.ep_err_rx_ | device_.ep_err_tx_;
  const uint32_t cur = device_.ReadEp(num);
  device_.WriteEp(
      num, (cur & device_.ep_keep_mask_) | ((cur ^ target) & stat_mask) | no_clear);
}

void FSDevEndpoint::IsoEmpty(uint8_t buffer)
{
  if (IsIn())
  {
    device_.SetIsoTxCount(Num(), buffer, 0U);
  }
  else
  {
    device_.SetIsoRxSize(Num(), buffer, MaxPacketSize());
  }
}

void FSDevEndpoint::IsoLoad()
{
  const uint8_t num = Num();
  for (uint8_t round = 0U; round < ISO_REQUESTS; ++round)
  {
    // 第一个还没进缓冲的请求，以及最后一个已进缓冲的请求所在的那块。 / The first
    // request not yet in a buffer, and the buffer of the last one already in one.
    IsoRequest* waiting = nullptr;
    int8_t last = -1;
    bool occupied[ISO_REQUESTS] = {};
    for (uint8_t k = 0U; k < iso_count_; ++k)
    {
      IsoRequest& request = iso_req_[(iso_head_ + k) % ISO_REQUESTS];
      if (request.buffer >= 0)
      {
        last = request.buffer;
        occupied[request.buffer] = true;
      }
      else if (waiting == nullptr)
      {
        waiting = &request;
      }
    }
    if (waiting == nullptr)
    {
      return;
    }
    const uint32_t epr = device_.ReadEp(num);
    const uint8_t hardware = (epr & FSDEV_EP_DTOG_TX) != 0U ? 1U : 0U;
    uint8_t buffer = 0U;
    if ((epr & FSDEV_EP_CTR_TX) != 0U)
    {
      // 这一帧的事务已经结束、CTR 还没处理：刚用过的那块（DTOG 的另一块）要先由
      // IsoOnPacket 判定，不能装；下一次事务要到下一帧，它用的那块空着就装进去，否则等。
      // This frame's transaction is over and its CTR not yet handled: the buffer just
      // used (the one DTOG does not point at) has to be judged by IsoOnPacket first and
      // is not loaded; the next transaction comes only in the next frame, so the buffer
      // it uses is loaded if empty, otherwise the request waits.
      if (occupied[hardware])
      {
        return;
      }
      buffer = hardware;
    }
    else
    {
      // 按顺序：紧跟最后一个已装请求的那块；都没装时是硬件下一次不用的那块。硬件下一次
      // 要发的那块不装：它的令牌可能就在眼前。
      // In order: the buffer after the last loaded request's; with none loaded, the one
      // the next transaction does not use. The buffer the next transaction sends is not
      // loaded: its token may be imminent.
      buffer = last >= 0 ? static_cast<uint8_t>(last ^ 1)
                         : static_cast<uint8_t>(hardware ^ 1U);
      if (buffer == hardware || occupied[buffer])
      {
        return;
      }
    }
    const uint16_t addr = buffer == 0U ? pma_addr_ : Partner()->pma_addr_;
    if (waiting->length != 0U)
    {
      device_.PmaWrite(addr, waiting->data, waiting->length);
    }
    device_.SetIsoTxCount(num, buffer, static_cast<uint16_t>(waiting->length));
    waiting->buffer = static_cast<int8_t>(buffer);
  }
}

void FSDevEndpoint::IsoOnPacket(bool in_isr)
{
  const uint8_t num = Num();
  const uint32_t epr = device_.ReadEp(num);
  if (IsIn())
  {
    // 刚用过的那块 = DTOG_TX 的另一块。 / The buffer just used is the one DTOG_TX does
    // not point at.
    const uint8_t used = (epr & FSDEV_EP_DTOG_TX) != 0U ? 0U : 1U;
    device_.ClearCtrTx(num);
    if (iso_count_ != 0U && iso_req_[iso_head_].buffer == static_cast<int8_t>(used))
    {
      IsoRequest& head = iso_req_[iso_head_];
      const size_t length = head.length;
      head.buffer = -1;
      iso_head_ = static_cast<uint8_t>((iso_head_ + 1U) % ISO_REQUESTS);
      --iso_count_;
      // 写回零长度，不再发旧数据。 / Back to zero length, never stale data.
      device_.SetIsoTxCount(num, used, 0U);
      IsoLoad();
      OnHardwareComplete(in_isr, length, ErrorCode::OK);
      return;
    }
    // 发出的是零长度包。 / A zero-length packet went out.
    IsoLoad();
    return;
  }
  const uint8_t used = (epr & FSDEV_EP_DTOG_RX) != 0U ? 0U : 1U;
  if (iso_count_ == 0U)
  {
    // 没有请求时的包丢弃。 / A packet without a request is dropped.
    device_.ClearCtrRx(num);
    return;
  }
  // CRC 错的包手册算作正确事务（RM0008 p.635），照常交付。 / A packet with a CRC error
  // is a correct transaction in the manual (RM0008 p.635) and is delivered as usual.
  IsoRequest& head = iso_req_[iso_head_];
  const size_t bytes = device_.GetIsoRxCount(num, used);
  const size_t take = LibXR::min(bytes, head.length);
  if (take != 0U)
  {
    device_.PmaRead(head.data, used == 0U ? pma_addr_ : Partner()->pma_addr_, take);
  }
  device_.ClearCtrRx(num);
  iso_head_ = static_cast<uint8_t>((iso_head_ + 1U) % ISO_REQUESTS);
  --iso_count_;
  OnHardwareComplete(in_isr, take,
                     bytes > take ? ErrorCode::OUT_OF_RANGE : ErrorCode::OK);
}

Endpoint::StopOutcome FSDevEndpoint::IsoStop()
{
  if (iso_count_ == 0U)
  {
    return {ErrorCode::ABORTED, 0U};
  }
  IsoRequest& head = iso_req_[iso_head_];
  if (head.buffer == ISO_STOPPED)
  {
    // 核心随后的调用：同一次停止里的其余请求。 / The core's following calls: the other
    // requests of the same stop.
    head.buffer = -1;
    iso_head_ = static_cast<uint8_t>((iso_head_ + 1U) % ISO_REQUESTS);
    --iso_count_;
    return {ErrorCode::ABORTED, 0U};
  }
  const uint8_t num = Num();
  const bool in = IsIn();
  const uint32_t ctr = in ? FSDEV_EP_CTR_TX : FSDEV_EP_CTR_RX;
  const uint32_t dtog = in ? FSDEV_EP_DTOG_TX : FSDEV_EP_DTOG_RX;
  const uint32_t no_clear =
      FSDEV_EP_CTR_RX | FSDEV_EP_CTR_TX | device_.ep_err_rx_ | device_.ep_err_tx_;
  const uint32_t keep = device_.ep_keep_mask_ & ~FSDEV_EP_ADDR;
  // 停车：此后的令牌没有应答（主机少一帧）；只有停车时已在线上的事务还能结束。不等。
  // Park: later tokens get no answer (the host misses one frame); only a transaction
  // already on the wire at the park can still end. No wait.
  uint32_t cur = device_.ReadEp(num);
  device_.WriteEp(num, (cur & keep) | FSDEV_EP_PARK_ADDR | no_clear);
  const uint32_t epr = device_.ReadEp(num);
  StopOutcome outcome{ErrorCode::ABORTED, 0U};
  if ((epr & ctr) != 0U)
  {
    // 有一个事务结束了：刚用过的那块 = DTOG 的另一块。IN 那块装着最早的请求就是它
    // 发完了；OUT 的包交给最早的请求。
    // A transaction has ended: the buffer just used is the one DTOG does not point at.
    // An IN whose earliest request sits in it has gone out; an OUT packet goes to the
    // earliest request.
    const uint8_t used = (epr & dtog) != 0U ? 0U : 1U;
    if (in)
    {
      if (head.buffer == static_cast<int8_t>(used))
      {
        outcome = {ErrorCode::OK, head.length};
      }
      device_.ClearCtrTx(num);
    }
    else
    {
      const size_t bytes = device_.GetIsoRxCount(num, used);
      const size_t take = LibXR::min(bytes, head.length);
      if (take != 0U)
      {
        device_.PmaRead(head.data, used == 0U ? pma_addr_ : Partner()->pma_addr_, take);
      }
      outcome = {bytes > take ? ErrorCode::OUT_OF_RANGE : ErrorCode::OK, take};
      device_.ClearCtrRx(num);
    }
  }
  if (in)
  {
    IsoEmpty(0U);
    IsoEmpty(1U);
  }
  head.buffer = -1;
  iso_head_ = static_cast<uint8_t>((iso_head_ + 1U) % ISO_REQUESTS);
  --iso_count_;
  for (uint8_t k = 0U; k < iso_count_; ++k)
  {
    iso_req_[(iso_head_ + k) % ISO_REQUESTS].buffer = ISO_STOPPED;
  }
  cur = device_.ReadEp(num);
  device_.WriteEp(num, (cur & keep) | num | no_clear);
  return outcome;
}

// ---------------------------------------------------------------------------
// 端点池 / Endpoint pool
// ---------------------------------------------------------------------------

FSDevPool::FSDevPool(FSDevDevice& device, const MemoryMap& memory_map,
                     const std::initializer_list<EPConfig>& rx_ep_cfgs,
                     const std::initializer_list<EPConfig>& tx_ep_cfgs, size_t pma_limit,
                     uint8_t slot_count)
{
  REQUIRE(rx_ep_cfgs.size() > 0U && rx_ep_cfgs.size() <= MAX_EPS);
  REQUIRE(tx_ep_cfgs.size() > 0U && tx_ep_cfgs.size() <= MAX_EPS);
  REQUIRE(memory_map.alignment != 0U);
  const size_t limit = pma_limit != 0U ? pma_limit : memory_map.packet_memory_size;
  endpoint_count_ = static_cast<uint8_t>(
      rx_ep_cfgs.size() > tx_ep_cfgs.size() ? rx_ep_cfgs.size() : tx_ep_cfgs.size());

  // 缓冲起点必须落在对齐边界上：地址字段按对齐截断，否则缓冲静默重叠。
  // Every buffer must start on an aligned boundary: the address field is truncated to
  // the alignment, or buffers would overlap silently.
  size_t offset = static_cast<size_t>(endpoint_count_) * 8U;
  REQUIRE((offset % memory_map.alignment) == 0U);
  for (size_t pass = 0U; pass < 2U; ++pass)
  {
    const bool in = pass == 1U;
    const std::initializer_list<EPConfig>& cfgs = in ? tx_ep_cfgs : rx_ep_cfgs;
    // 上面的 REQUIRE 已保证 cfgs.size() <= MAX_EPS；关掉断言的构建里编译器看不到这一点，
    // 会报 -Warray-bounds，所以循环条件里再写一次。
    // The REQUIRE above already bounds cfgs.size() by MAX_EPS; a build with assertions
    // off cannot see that and warns with -Warray-bounds, hence the bound in the loop.
    for (size_t i = 0U; i < cfgs.size() && i < MAX_EPS; ++i)
    {
      const EPConfig& cfg = cfgs.begin()[i];
      REQUIRE((cfg.pma_size % memory_map.alignment) == 0U);
      auto* ep = new FSDevEndpoint(
          device, static_cast<Endpoint::EPNumber>(i),
          in ? Endpoint::Direction::IN : Endpoint::Direction::OUT, cfg.buffer,
          static_cast<uint16_t>(offset), cfg.pma_size, slot_count);
      offset += cfg.pma_size;
      (in ? in_ : out_)[i] = ep;
      if (i != 0U)
      {
        REQUIRE(Put(ep) == ErrorCode::OK);
      }
    }
  }
  REQUIRE(offset <= limit);
  SetEndpoint0(in_[0], out_[0]);
}

// ---------------------------------------------------------------------------
// 设备 / Device
// ---------------------------------------------------------------------------

FSDevDevice::FSDevDevice(
    const MemoryMap& memory_map, const std::initializer_list<EPConfig>& rx_ep_cfgs,
    const std::initializer_list<EPConfig>& tx_ep_cfgs,
    DeviceDescriptor::PacketSize0 packet_size, uint16_t vid, uint16_t pid, uint16_t bcd,
    const std::initializer_list<const DescriptorStrings::LanguagePack*>& languages,
    const std::initializer_list<const std::initializer_list<DeviceClass*>>& configs,
    ConstRawData uid, size_t pma_limit, uint8_t slot_count)
    : FSDevPool(*this, memory_map, rx_ep_cfgs, tx_ep_cfgs, pma_limit, slot_count),
      DeviceCore(*this, *this, USBSpec::USB_2_1, Speed::FULL, packet_size, vid, pid, bcd,
                 languages, configs, uid),
      memory_map_(memory_map)
{
  // 一个程序只有一个 FSDEV 设备对象：self_ 是 Dispatch 的静态入口。
  // One FSDEV device object per program: self_ is the static entry of Dispatch.
  REQUIRE(self_ == nullptr);
  self_ = this;
  for (FSDevEndpoint* const* table : {in_, out_})
  {
    for (uint8_t i = 0U; i < MAX_EPS; ++i)
    {
      if (table[i] != nullptr)
      {
        REQUIRE(Register(*table[i]) == ErrorCode::OK);
      }
    }
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
  // 早到的包没有请求来取时靠快速确认清掉。 / An early packet that no request takes is
  // cleared by the fast acknowledgement.
  ASSERT(!ep0_early_out_ || ep0_fast_ack_);
  ASSERT(!ep0_bulk_gate_ || ep0_fast_ack_);
  InitRegs();
  for (uint8_t i = 0U; i < MAX_EPS; ++i)
  {
    if (out_[i] != nullptr)
    {
      SetRxBuf(i, out_[i]->pma_addr_);
    }
    if (in_[i] != nullptr)
    {
      SetTxBuf(i, in_[i]->pma_addr_);
    }
  }
  // 中断源保持关闭，Start 才打开；上拉在 Start 最后接通。
  // The interrupt sources stay off until Start; the pull-up comes on last in Start.
  WriteCntr(static_cast<uint16_t>(ReadCntr() & ~(FSDEV_CNTR_RUN_MASK | FSDEV_CNTR_SOFM)));
  controller_ready_ = true;
}

void FSDevDevice::StartController(bool in_isr)
{
  UNUSED(in_isr);
  WaitDetached();
  REQUIRE(controller_ready_);
  WriteCntr(static_cast<uint16_t>(ReadCntr() | FSDEV_CNTR_RUN_MASK | FSDEV_CNTR_SOFM));
  Connect(true);
}

void FSDevDevice::WaitDetached()
{
  // 断开要保持到主机看得见，否则主机不复位总线：全速下只断开几十微秒，主机多半看不见
  // （CH32V208、STM32H723 实测），5 ms 以上都看得见。本控制器上没有单独测过。
  // The detach has to last until the host sees it, or the host does not reset the bus:
  // at full speed a detach of some tens of microseconds mostly goes unseen (measured on
  // a CH32V208 and an STM32H723), and 5 ms or more was always seen. Not measured on
  // this controller itself.
  if (!halted_)
  {
    return;
  }
  while (static_cast<uint64_t>(Timebase::GetMicroseconds()) - halted_at_us_ < DETACH_US)
  {
  }
  if (detach_held_)
  {
    // 平台放开 D+ 并恢复控制器（时钟）；控制器重新初始化，与 HaltController 里的一样。
    // The platform releases D+ and restores the controller (clock); the controller is
    // initialized again, as in HaltController.
    ReleaseDetached();
    detach_held_ = false;
    InitController();
  }
  halted_ = false;
}

void FSDevDevice::HaltController()
{
  Connect(false);
  halted_at_us_ = static_cast<uint64_t>(Timebase::GetMicroseconds());
  halted_ = true;
  for (FSDevEndpoint* const* table : {in_, out_})
  {
    for (uint8_t i = 0U; i < MAX_EPS; ++i)
    {
      if (table[i] != nullptr)
      {
        table[i]->busy_ = false;
        table[i]->stopping_ = false;
        table[i]->stop_failed_ = false;
      }
    }
  }
  pending_stops_ = 0U;
  suspended_ = false;
  setup_shadow_pending_ = false;
  ep0_out_data_ = false;
  // 强制复位清掉全部 EPnR，并关掉中断源。 / The forced reset clears every EPnR and
  // switches the interrupt sources off.
  InitController();
  // 平台可在断开期间把 D+ 拉低（外部上拉的板子）。 / The platform may hold D+ low while
  // detached (boards with an external pull-up).
  detach_held_ = HoldDetached();
}

void FSDevDevice::StopController(bool in_isr)
{
  UNUSED(in_isr);
  HaltController();
}

void FSDevDevice::SetAddress(uint8_t address, AddressStage stage)
{
  // 经典型在状态 IN 完成后写 DADDR（RM0008 §23.4.5）。
  // The classic controller writes DADDR after the status IN completed (RM0008 §23.4.5).
  if (stage == AddressStage::STATUS_COMPLETE)
  {
    WriteDaddr(static_cast<uint16_t>(0x0080U | address));
  }
}

void FSDevDevice::ResetController(bool in_isr)
{
  UNUSED(in_isr);
  HaltController();
  WaitDetached();
  WriteCntr(static_cast<uint16_t>(ReadCntr() | FSDEV_CNTR_RUN_MASK | FSDEV_CNTR_SOFM));
  Connect(true);
}

void FSDevDevice::Dispatch()
{
  FSDevDevice* device = self_;
  if (device == nullptr || !device->controller_ready_ || device->detach_held_)
  {
    return;
  }
  if (device->ep0_bulk_gate_)
  {
    device->CaptureSetup();
  }
  if (device->ep0_fast_ack_)
  {
    device->FastAckEp0();
  }
  if (device->status_window_ && device->HoldStatusWindow())
  {
    return;
  }
  device->OnControllerInterrupt();
  if (device->ep0_late_status_)
  {
    device->ReleaseEp0Status();
  }
}

void FSDevDevice::ReleaseEp0Status()
{
  FSDevEndpoint* out = out_[0];
  const FSDevEndpoint* in = in_[0];
  if (!out->status_held_)
  {
    return;
  }
  // EP0 IN 上还有包在途（数据阶段没结束）：继续留住。
  // A packet is still in flight on EP0 IN (the data stage is not over): keep holding.
  if (in->busy_)
  {
    return;
  }
  // 留住之后请求已被停掉（新的 SETUP、总线复位、Stop）：没有东西要放行。
  // The request was stopped after it was held (a new SETUP, a bus reset, Stop): nothing
  // is left to release.
  if (!out->busy_)
  {
    out->status_held_ = false;
    return;
  }
  // 有未处理的 SETUP 时 SetStat 写回 NAK，算已放行，由随后的 SETUP 处理结束请求。VALID
  // 没写进（包括总线复位的 SE0 期间）时继续留住，下一次中断再写。
  // With an unhandled SETUP SetStat writes NAK back; that counts as released, and the
  // SETUP handling that follows ends the request. If VALID does not take (the SE0 of a
  // bus reset included) the hold stays and the next interrupt writes again.
  const bool armed = out->Arm();
  out->status_held_ = !armed;
  if (armed)
  {
    status_window_ = true;
    window_sofs_ = 0U;
  }
}

bool FSDevDevice::HoldStatusWindow()
{
  // 状态包已被快速确认收下：等待结束。 / The fast acknowledgement took the status
  // packet: the wait is over.
  if (ep0_shadow_pending_)
  {
    status_window_ = false;
    return false;
  }
  // 有界：每个端点方向至多一个完成（之后硬件回 NAK），外加帧起始。
  // Bounded: at most one completion per endpoint and direction (the hardware answers
  // NAK afterwards), plus the starts of frame.
  for (unsigned i = 0U; i < 2U * MAX_EPS + STATUS_WINDOW_SOFS; ++i)
  {
    const uint16_t istr = ReadIstr();
    if ((istr & (FSDEV_ISTR_RESET | FSDEV_ISTR_SUSP | FSDEV_ISTR_WKUP)) != 0U)
    {
      status_window_ = false;
      return false;
    }
    if ((istr & FSDEV_ISTR_SOF) != 0U)
    {
      ClearIstr(FSDEV_ISTR_SOF);
      sof_latched_ = true;
      if (++window_sofs_ >= STATUS_WINDOW_SOFS)
      {
        status_window_ = false;
        return false;
      }
      continue;
    }
    if ((istr & FSDEV_ISTR_CTR) == 0U)
    {
      return true;
    }
    const uint8_t num = static_cast<uint8_t>(istr & FSDEV_ISTR_EP_ID);
    if (num == 0U)
    {
      // EP0 上有事件：SETUP，或快速确认之后才到的包（再确认一次，不让它等 owner）。
      // An event on EP0: a SETUP, or a packet that arrived after the fast
      // acknowledgement (acknowledged once more so that it does not wait for the owner).
      if (ep0_bulk_gate_)
      {
        CaptureSetup();
      }
      if (ep0_fast_ack_)
      {
        FastAckEp0();
      }
      status_window_ = false;
      return false;
    }
    // 等时端点没有 NAK，双缓冲只容得下一帧的延迟：结束等待，owner 进来处理。
    // An isochronous endpoint has no NAK and its double buffer tolerates one frame of
    // delay only: the wait ends and the owner comes in.
    if (num < MAX_EPS && ((in_[num] != nullptr && in_[num]->iso_) ||
                          (out_[num] != nullptr && out_[num]->iso_)))
    {
      status_window_ = false;
      return false;
    }
    // 其他端点的完成：清 CTR、记下。硬件已把这个方向置 NAK，PMA 里的数据留着。
    // A completion of another endpoint: clear its CTR and latch it. The hardware has set
    // this direction to NAK and the data in the PMA stay.
    const uint32_t epr = ReadEp(num);
    if ((epr & FSDEV_EP_CTR_TX) != 0U)
    {
      ClearCtrTx(num);
      latched_ = static_cast<uint16_t>(latched_ | (0x0100U << num));
    }
    if ((epr & FSDEV_EP_CTR_RX) != 0U)
    {
      ClearCtrRx(num);
      latched_ = static_cast<uint16_t>(latched_ | (0x0001U << num));
    }
  }
  return true;
}

void FSDevDevice::ServiceLatched(bool in_isr)
{
  const uint16_t bits = latched_;
  latched_ = 0U;
  for (uint8_t num = 1U; num < MAX_EPS; ++num)
  {
    if ((bits & (0x0100U << num)) != 0U)
    {
      HandleCtrTx(in_isr, num);
    }
    if ((bits & (0x0001U << num)) != 0U)
    {
      HandleCtrRx(in_isr, num);
    }
  }
}

void FSDevDevice::FastAckEp0()
{
  if (ep0_shadow_pending_)
  {
    return;
  }
  const uint32_t epr = ReadEp(0U);
  if ((epr & FSDEV_EP_CTR_RX) == 0U || (epr & FSDEV_EP_SETUP) != 0U)
  {
    return;
  }
  // 先读 PMA 再清 CTR_RX：清掉之后硬件就可以把下一个 SETUP 写进同一块接收区。
  // Read the PMA before clearing CTR_RX: once it is clear the hardware may write the
  // next SETUP into the same receive area.
  const uint16_t count = GetRxCount(0U);
  const size_t take = LibXR::min(size_t{count}, sizeof(ep0_shadow_));
  if (take != 0U)
  {
    PmaRead(ep0_shadow_, out_[0]->pma_addr_, take);
  }
  ep0_shadow_count_ = count;
  ep0_shadow_pending_ = true;
  ClearCtrRx(0U);
}

void FSDevDevice::CaptureSetup()
{
  if (setup_shadow_pending_)
  {
    return;
  }
  const uint32_t epr = ReadEp(0U);
  if (!SetupPending(epr))
  {
    return;
  }
  // 先读 PMA 再清 CTR_RX，同一次写把类型按控制传输的方向写好（见 ClearCtrRx）。
  // The PMA is read before CTR_RX is cleared; the same write types EP0 for the direction
  // of the control transfer (see ClearCtrRx).
  PmaRead(setup_shadow_, out_[0]->pma_addr_, sizeof(setup_shadow_));
  const bool has_data = (setup_shadow_[6] | setup_shadow_[7]) != 0U;
  ep0_out_data_ = has_data && (setup_shadow_[0] & 0x80U) == 0U;
  setup_shadow_pending_ = true;
  ClearCtrRx(0U);
  // 再读一遍：控制器边收边写 PMA，正在收的数据包（之后可能被 NAK）会在两次读之间改写
  // SETUP 的字节，COUNT 却要等包收完才变。 / Read again: the controller writes the PMA
  // while it receives, so a data packet on the wire (possibly NAKed later) changes the
  // SETUP's bytes between the two reads, while COUNT changes only once the packet is in.
  uint8_t again[sizeof(setup_shadow_)];
  PmaRead(again, out_[0]->pma_addr_, sizeof(again));
  if (GetRxCount(0U) != sizeof(setup_shadow_) ||
      std::memcmp(again, setup_shadow_, sizeof(again)) != 0)
  {
    // COUNT 不是 8：类型为 CONTROL 时控制器照收数据包，一个数据包在 SETUP 读走之前覆盖
    // 了它（读到的是数据），或在读 SETUP 与清 CTR_RX 之间到达、它的 CTR_RX 被这次清掉
    // （主机以为送到，设备没有它）。两种情形都不交付 SETUP，主机的这次控制传输以超时
    // 结束，而不是把数据当 SETUP、或把少了第一包的数据当完整的。类型写回 CONTROL，等
    // 主机的下一个 SETUP。
    // COUNT is not 8: typed CONTROL the controller takes data packets, and a data packet
    // either overwrote the SETUP before it was read (what was read is data) or came
    // between reading the SETUP and clearing CTR_RX, whose clear wiped its CTR_RX (the
    // host believes it delivered, the device does not have it). Either way the SETUP is
    // not handed over and the host's control transfer ends with a timeout, rather than
    // data taken for a SETUP or data without its first packet taken as complete. EP0 is
    // typed CONTROL again for the host's next SETUP.
    ++setups_overwritten_;
    setup_shadow_pending_ = false;
    ep0_out_data_ = false;
    ClearCtrRx(0U);
  }
}

void FSDevDevice::ServiceSetupShadow(bool in_isr)
{
  // 与 HandleEp 的 SETUP 分支相同：同时锁存的 EP0 IN 完成先交付，控制端点上未结束的停止
  // 在交付 SETUP 之前结束。
  // As in the SETUP branch of HandleEp: an EP0 IN completion latched as well is handed
  // over first, and pending stops on the control endpoint end before the SETUP is.
  if ((ReadEp(0U) & FSDEV_EP_CTR_TX) != 0U)
  {
    HandleCtrTx(in_isr, 0U);
  }
  setup_shadow_pending_ = false;
  SetupPacket setup;
  std::memcpy(&setup, setup_shadow_, sizeof(setup));
  in_[0]->AdvanceStop(in_isr, 0U, true);
  out_[0]->AdvanceStop(in_isr, 0U, true);
  OnSetup(in_isr, setup);
}

void FSDevDevice::ServiceEp0Shadow(bool in_isr)
{
  // 这个包之前完成的 EP0 IN 先交付（它的 CTR_TX 还锁存着）。
  // An EP0 IN that completed before this packet is delivered first (its CTR_TX is still
  // latched).
  if ((ReadEp(0U) & FSDEV_EP_CTR_TX) != 0U)
  {
    HandleCtrTx(in_isr, 0U);
  }
  ep0_shadow_pending_ = false;
  const size_t bytes = LibXR::min(size_t{ep0_shadow_count_}, sizeof(ep0_shadow_));
  out_[0]->OnShadowPacket(in_isr, ep0_shadow_, bytes);
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
  uint32_t keep = cur & ep_keep_mask_ & ~self;
  // BULK 闸：控制写期间 EP0 OUT 的 CTR_RX 在写 BULK 的同一次写里清（SETUP 的和每个数据
  // 包的），清掉之后硬件按 STAT_RX 回 NAK，直到装填把类型写回 CONTROL。别的 EP0 包（例如
  // 控制读的状态包，它可能在写 BULK 之前就被收下、不经装填）清的同时写回 CONTROL：传输
  // 到此结束，接下来是 SETUP。
  // BULK gate: during a control write the CTR_RX of EP0 OUT is cleared in the same write
  // that types it BULK (the SETUP's and each data packet's), so the hardware answers NAK
  // by STAT_RX from then on until an arm types it CONTROL again. Any other EP0 packet
  // (the status packet of a control read, say, which may be taken before BULK is
  // written and without an arm) is cleared in a write that types EP0 CONTROL again: the
  // transfer ends there and a SETUP comes next.
  if (num == 0U && ep0_bulk_gate_)
  {
    keep = WithType(keep, ep0_out_data_ ? Endpoint::Type::BULK : Endpoint::Type::CONTROL);
  }
  WriteEp(num, keep | other);
}

void FSDevDevice::ServiceHardware(bool in_isr)
{
  // 噪声位按进入时的快照清：掩码里混进有处理分支的标志时，快照之后锁存的那次事件会被
  // 直接清掉。
  // The noise flags are cleared from the snapshot taken on entry: a handled flag inside
  // the mask would wipe an event latched after the snapshot.
  static_assert((FSDEV_ISTR_NOISE & (FSDEV_ISTR_CTR | FSDEV_ISTR_RESET | FSDEV_ISTR_SOF |
                                     FSDEV_ISTR_SUSP | FSDEV_ISTR_WKUP)) == 0U,
                "ISTR_NOISE overlaps an interrupt flag that has a handler");
  const uint16_t istr = ReadIstr();
  if ((istr & FSDEV_ISTR_RESET) != 0U)
  {
    // 复位时硬件已清地址和全部 EPnR（RM0008 p.640），之前锁存的 CTR 随之消失；设备核心
    // 在 OnBusReset 里关闭各端点（StopHardware 不等待）并重新配置 EP0。
    // The reset has cleared the address and every EPnR (RM0008 p.640) with any CTR
    // latched before it; in OnBusReset the device core closes the endpoints
    // (StopHardware does not wait) and configures EP0 again.
    ClearIstr(FSDEV_ISTR_RESET);
    WriteDaddr(0x0080U);
    WriteCntr(static_cast<uint16_t>(ReadCntr() | FSDEV_CNTR_SOFM));
    suspended_ = false;
    // 复位之前收下的 EP0 包作废。 / An EP0 packet taken before the reset is void.
    ep0_shadow_pending_ = false;
    setup_shadow_pending_ = false;
    ep0_out_data_ = false;
    // 未结束的停止在报告复位之前结束。 / Pending stops end before the reset is
    // reported.
    AdvanceStops(in_isr, true);
    OnBusReset(in_isr, Speed::FULL);
  }
  if (ep0_shadow_pending_)
  {
    // 快速确认收下的 EP0 包早于现在还锁存着的 SETUP，先处理。
    // The EP0 packet the fast acknowledgement took precedes any SETUP still latched
    // and is handled first.
    ServiceEp0Shadow(in_isr);
  }
  if (setup_shadow_pending_)
  {
    // 入口读走的 SETUP（BULK 闸）。 / The SETUP read at the entry (BULK gate).
    ServiceSetupShadow(in_isr);
  }
  if (latched_ != 0U)
  {
    // 等状态包期间记下的完成早于现在锁存的。总线复位之后各端点已没有请求，记下的完成
    // 只被清掉。
    // Completions latched while the status packet was awaited precede those latched now.
    // After a bus reset no endpoint has a request and a latched completion is only
    // cleared.
    ServiceLatched(in_isr);
  }
  if ((istr & FSDEV_ISTR_CTR) != 0U)
  {
    HandleCtr(in_isr);
  }
  AdvanceStops(in_isr, false);
  if ((istr & FSDEV_ISTR_SOF) != 0U || sof_latched_)
  {
    // 等状态包期间清掉的帧起始在这里报告一次。 / Starts of frame cleared while the
    // status packet was awaited are reported once here.
    sof_latched_ = false;
    if ((istr & FSDEV_ISTR_SOF) != 0U)
    {
      ClearIstr(FSDEV_ISTR_SOF);
    }
    OnStartOfFrame(in_isr, static_cast<uint16_t>(ReadRegister(FSDEV_REG_FNR) & 0x07FFU));
  }
  if ((istr & FSDEV_ISTR_SUSP) != 0U)
  {
    // 先进挂起再清标志，最后接时钟门控（DRD 要求的顺序）；经典型 SuspendClocked 为空。
    // Enter suspend, clear the flag, then gate the clocks (the order DRD requires);
    // SuspendClocked is empty on classic.
    WriteCntr(static_cast<uint16_t>(ReadCntr() & ~FSDEV_CNTR_SOFM));
    SuspendRegs();
    ClearIstr(FSDEV_ISTR_SUSP);
    SuspendClocked();
    // 总线已空闲 3 ms：线上没有事务，未结束的停止就此结束；SOF 中断已关，之后的停止
    // 当场完成。
    // The bus has been idle for 3 ms: no transaction is on the wire and pending stops
    // end here; the SOF interrupt is off, and later stops complete at once.
    suspended_ = true;
    noise_wakeups_ = 0U;
    AdvanceStops(in_isr, true);
    OnSuspend(in_isr);
  }
  if ((istr & FSDEV_ISTR_WKUP) != 0U)
  {
    ResumeRegs();
    // 手册的恢复步骤（RM0365 p.1024，表 176）：清 FSUSP 之后看 FNR 的 RXDP、RXDM。恢复
    // 信号是 "01"，复位是 "00"；RXDP 为 1（空闲电平 "10" 或不允许的 "11"）是线上的噪声，
    // 回到挂起。STM32F302 实测：主机还有批量请求在途时挂起端口，35-62% 的挂起之后
    // 60 us 来一次线路为 "10" 的唤醒，真正的恢复在约 124 ms 之后。
    // 中断来得比恢复信号（至少 20 ms）还晚时，读到的已是恢复之后的空闲电平，这之后总线
    // 上的每个包都会再触发一次唤醒：一次挂起里第 MAX_NOISE_WAKEUPS + 1 次这样的唤醒按
    // 真的恢复处理，设备不会停在挂起里。
    // The manual's resume procedure (RM0365 p.1024, table 176): after clearing FSUSP,
    // look at RXDP and RXDM of FNR. Resume signalling is "01" and a reset "00"; RXDP at
    // 1 (the idle level "10" or the illegal "11") is noise on the line and goes back to
    // suspend. Measured on an STM32F302: when a port is suspended with bulk requests in
    // flight, 35-62 % of the suspends are followed 60 us later by a wake-up with the
    // line at "10", and the real resume comes about 124 ms later.
    // If the interrupt comes later than the resume signalling lasts (at least 20 ms),
    // what is read is already the idle level after it, and every packet on the bus then
    // raises another wake-up: the (MAX_NOISE_WAKEUPS + 1)-th such wake-up of one suspend
    // is handled as a real resume, so the device cannot stay in suspend.
    if (suspended_ && noise_wakeups_ < MAX_NOISE_WAKEUPS &&
        (ReadRegister(FSDEV_REG_FNR) & FSDEV_FNR_RXDP) != 0U)
    {
      ++noise_wakeups_;
      SuspendRegs();
      ClearIstr(FSDEV_ISTR_WKUP);
      SuspendClocked();
    }
    else
    {
      WriteCntr(static_cast<uint16_t>(ReadCntr() | FSDEV_CNTR_SOFM));
      ClearIstr(FSDEV_ISTR_WKUP);
      suspended_ = false;
      OnResume(in_isr);
    }
  }
  if ((istr & FSDEV_ISTR_NOISE) != 0U)
  {
    ClearIstr(FSDEV_ISTR_NOISE);
  }
}

void FSDevDevice::AdvanceStops(bool in_isr, bool force)
{
  if (pending_stops_ == 0U)
  {
    return;
  }
  const uint16_t frame = FrameNumber();
  for (FSDevEndpoint* const* table : {in_, out_})
  {
    for (uint8_t i = 0U; i < MAX_EPS; ++i)
    {
      if (table[i] != nullptr)
      {
        table[i]->AdvanceStop(in_isr, frame, force);
      }
    }
  }
}

void FSDevDevice::HandleCtr(bool in_isr)
{
  // ISTR.CTR 只读，是各端点 CTR 的或（RM0008 p.640）；每轮按 EP_ID 处理一个端点。剩余
  // 的事件在开线后由仍为高的 ISTR.CTR 再次触发中断。
  // ISTR.CTR is read-only and the OR of the endpoints' CTRs (RM0008 p.640); each pass
  // handles the endpoint EP_ID names. Remaining events raise the interrupt again after
  // the unmask, since ISTR.CTR stays high.
  for (unsigned pass = 0U; pass < 8U; ++pass)
  {
    const uint16_t istr = ReadIstr();
    if ((istr & FSDEV_ISTR_CTR) == 0U)
    {
      return;
    }
    HandleEp(in_isr, static_cast<uint8_t>(istr & FSDEV_ISTR_EP_ID));
  }
}

void FSDevDevice::HandleCtrTx(bool in_isr, uint8_t num)
{
  FSDevEndpoint* in = num < MAX_EPS ? in_[num] : nullptr;
  if (in != nullptr && in->iso_)
  {
    in->IsoOnPacket(in_isr);
  }
  else if (in != nullptr && in->stopping_ && !in->stop_failed_)
  {
    // 被停请求的事务完成了：停止结束。 / The stopped request's transaction completed:
    // the stop ends.
    in->AdvanceStop(in_isr, 0U, true);
  }
  else if (in != nullptr && in->busy_)
  {
    in->OnPacket(in_isr);
  }
  else
  {
    ClearCtrTx(num);
  }
}

void FSDevDevice::HandleEp(bool in_isr, uint8_t num)
{
  // 一次读取，先 TX 后 RX：同一次读到的 EP0 IN 完成先于 SETUP 交付。SETUP 之后硬件
  // 已把 STAT_TX 置 NAK（RM0008 p.630），读取之后不会再有属于旧传输的 IN 完成。
  // One read, TX before RX: an EP0 IN completion read together with a SETUP is
  // delivered first. After a SETUP the hardware has set STAT_TX to NAK (RM0008 p.630),
  // so no IN completion of the old transfer can follow the read.
  const uint32_t epr = ReadEp(num);
  if ((epr & FSDEV_EP_CTR_TX) != 0U)
  {
    HandleCtrTx(in_isr, num);
  }
  if ((epr & FSDEV_EP_CTR_RX) == 0U)
  {
    return;
  }
  if (num == 0U && (epr & FSDEV_EP_SETUP) != 0U)
  {
    // 先读 8 字节再清 CTR_RX：CTR_RX 置位期间硬件不收新 SETUP。
    // Read the 8 bytes before clearing CTR_RX: no new SETUP is taken while it is set.
    uint8_t raw[8];
    PmaRead(raw, out_[0]->pma_addr_, sizeof(raw));
    // 方向是 bmRequestType 的位 7，wLength 在字节 6、7。 / The direction is bit 7 of
    // bmRequestType, wLength sits in bytes 6 and 7.
    const bool has_data = (raw[6] | raw[7]) != 0U;
    ep0_out_data_ = has_data && (raw[0] & 0x80U) == 0U;
    ClearCtrRx(0U);
    SetupPacket setup;
    std::memcpy(&setup, raw, sizeof(setup));
    // SETUP 已把 EP0 两方向置 NAK：控制端点上未结束的停止在交付 SETUP 之前结束。
    // The SETUP has set both EP0 directions to NAK: pending stops on the control
    // endpoint end before the SETUP is handed over.
    in_[0]->AdvanceStop(in_isr, 0U, true);
    out_[0]->AdvanceStop(in_isr, 0U, true);
    OnSetup(in_isr, setup);
    return;
  }
  HandleCtrRx(in_isr, num);
}

void FSDevDevice::HandleCtrRx(bool in_isr, uint8_t num)
{
  FSDevEndpoint* out = num < MAX_EPS ? out_[num] : nullptr;
  if (out != nullptr && out->iso_)
  {
    out->IsoOnPacket(in_isr);
  }
  else if (out != nullptr && out->stopping_ && !out->stop_failed_)
  {
    out->AdvanceStop(in_isr, 0U, true);
  }
  else if (out != nullptr && out->busy_)
  {
    out->OnPacket(in_isr);
  }
  else if (num == 0U && ep0_early_out_)
  {
    // 早到的包：接收请求要到 owner 这一轮稍后才启动。留着不清：请求启动时把它算作
    // 第一包；一直没有请求时，下一次中断入口的快速确认把它拷走，随后被丢弃。EP_ID 这一
    // 轮会一直指着 EP0，其余端点的完成由仍为高的 ISTR.CTR 触发的下一次中断处理。
    // An early packet: the receive request starts only later in this owner pass. It
    // stays latched: the request counts it as its first packet when it starts; if none
    // starts, the fast acknowledgement at the next interrupt entry copies it out and it
    // is dropped then. EP_ID keeps naming EP0 for the rest of this pass; the other
    // endpoints' completions are handled in the next interrupt, which ISTR.CTR, still
    // high, raises.
  }
  else
  {
    // 没有请求时到达的包，读掉丢弃。 / A packet with no request is dropped.
    ClearCtrRx(num);
  }
}

// ---------------------------------------------------------------------------
// 经典型寄存器、BTABLE 与 PMA / Classic registers, BTABLE and PMA
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
  // BTABLE = 0：字段 m 是第 8n+2m 字节处的 16 位字，地址再乘 word_stride。
  // BTABLE = 0: field m is the 16-bit word at byte 8n+2m, times word_stride.
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
  *BtField(num, 3U) = EncodeRxSize(mps);
}

uint16_t FSDevDevice::GetRxCount(uint8_t num)
{
  return static_cast<uint16_t>(*BtField(num, 3U) & 0x03FFU);
}

void FSDevDevice::SetIsoTxCount(uint8_t num, uint8_t buffer, uint16_t bytes)
{
  // COUNTn_TX_1 的位 15..10 不用，写 0（RM0008 p.649）。 / Bits 15..10 of COUNTn_TX_1
  // are unused and written 0 (RM0008 p.649).
  *BtField(num, IsoCountField(buffer)) = static_cast<uint16_t>(bytes & 0x03FFU);
}

void FSDevDevice::SetIsoRxSize(uint8_t num, uint8_t buffer, uint16_t mps)
{
  *BtField(num, IsoCountField(buffer)) = EncodeRxSize(mps);
}

uint16_t FSDevDevice::GetIsoRxCount(uint8_t num, uint8_t buffer)
{
  return static_cast<uint16_t>(*BtField(num, IsoCountField(buffer)) & 0x03FFU);
}

void FSDevDevice::PmaWrite(uint16_t addr, const uint8_t* src, size_t len)
{
  const size_t stride = memory_map_.word_stride;
  // NOLINTNEXTLINE(performance-no-int-to-ptr)
  volatile uint16_t* p = reinterpret_cast<volatile uint16_t*>(
      memory_map_.packet_memory + static_cast<uintptr_t>(addr) * stride);
  for (size_t i = 0U; i < len; i += 2U)
  {
    uint16_t word = src[i];
    if (i + 1U < len)
    {
      word |= static_cast<uint16_t>(src[i + 1U]) << 8U;
    }
    *p = word;
    p += stride;
  }
}

void FSDevDevice::PmaRead(uint8_t* dst, uint16_t addr, size_t len)
{
  const size_t stride = memory_map_.word_stride;
  // NOLINTNEXTLINE(performance-no-int-to-ptr)
  const volatile uint16_t* p = reinterpret_cast<const volatile uint16_t*>(
      memory_map_.packet_memory + static_cast<uintptr_t>(addr) * stride);
  for (size_t i = 0U; i < len; i += 2U)
  {
    const uint16_t word = *p;
    p += stride;
    dst[i] = static_cast<uint8_t>(word & 0xFFU);
    if (i + 1U < len)
    {
      dst[i + 1U] = static_cast<uint8_t>(word >> 8U);
    }
  }
}

void FSDevDevice::InitRegs()
{
  WriteCntr(FSDEV_CNTR_FRES);  // 强制复位，顺带清 PDWN / Force reset, clears PDWN too
  WriteCntr(0U);
  // 直接写 0 清全部标志；ClearIstr(0) 写的是全 1（保持）。
  // Write 0 to clear every flag; ClearIstr(0) writes all ones (keep).
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

}  // namespace LibXR::USB
