// NOLINTBEGIN(cppcoreguidelines-pro-type-cstyle-cast,performance-no-int-to-ptr)
#include <algorithm>
#include <cstring>

#include "ch32_usb_dev.hpp"
#include "ch32_usb_rcc.hpp"
#include "timebase.hpp"

using namespace LibXR;

#if defined(USBFSD)

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

using Ep = CH32EndpointOtgFs;
using Endpoint = USB::Endpoint;

constexpr uint8_t OUT_IDX = static_cast<uint8_t>(Endpoint::Direction::OUT);
constexpr uint8_t IN_IDX = static_cast<uint8_t>(Endpoint::Direction::IN);

/// 一轮里最多补交付这么多个并进来的等时完成（一个端点号的两个方向）/ Merged isochronous
/// completions delivered per round at most (both directions of one endpoint number)
constexpr uint8_t MERGED_EVENTS_MAX = 2U;

/// 设备模式下会被置位、可写 1 清的标志 / Flags set in device mode and cleared by writing
/// 1
constexpr uint8_t OTG_FS_CLEARABLE_MASK = USBFS_UIF_FIFO_OV | USBFS_UIF_HST_SOF |
                                          USBFS_UIF_SUSPEND | USBFS_UIF_TRANSFER |
                                          USBFS_UIF_DETECT | USBFS_UIF_BUS_RST;

/// 控制器拔出、主机看到断开的时间（µs）/ Time the controller stays detached so that the
/// host sees a disconnect (µs)
constexpr uint32_t DETACH_US = 5000U;

/// 共享端点每个方向的槽字节数 / Slot bytes per direction of a shared endpoint
constexpr size_t SHARED_SLOT_SIZE = 64U;

/// 初始化时分配一段按缓存行对齐的槽内存，不释放 / Allocate cache-line aligned slot
/// memory at initialisation; never freed
uint8_t* AllocateSlot(size_t size)
{
  constexpr uintptr_t ALIGN = HW_CACHE_LINE_SIZE;
  auto* raw = new uint8_t[size + ALIGN]();
  return reinterpret_cast<uint8_t*>((reinterpret_cast<uintptr_t>(raw) + ALIGN - 1U) &
                                    ~(ALIGN - 1U));
}

/// 没有对象时只清掉已锁存的标志 / Without an object, only clear the latched flags
void ClearPendingOtgFsInterrupts()
{
  while (true)
  {
    const uint8_t pending = USBFSD->INT_FG & OTG_FS_CLEARABLE_MASK;
    if (pending == 0U)
    {
      break;
    }
    USBFSD->INT_FG = pending;
  }
}

}  // namespace

// NOLINTNEXTLINE(readability-identifier-naming)
extern "C" CH32_USB_IRQ void USBFS_IRQHandler(void)
{
  CH32USBOtgFS* usb = CH32USBOtgFS::self_;
  if (usb == nullptr)
  {
    ClearPendingOtgFsInterrupts();
    return;
  }
  usb->OnControllerInterrupt();
}

CH32USBOtgFS::CH32USBOtgFS(const std::initializer_list<RawData>& rx_ep_cfgs,
                           const std::initializer_list<RawData>& tx_ep_cfgs,
                           USB::DeviceDescriptor::PacketSize0 packet_size, uint16_t vid,
                           uint16_t pid, uint16_t bcd, const Languages& languages,
                           const Configs& configs, ConstRawData uid)
    : USB::DeviceCore(*this, AddEndpoints(*this, rx_ep_cfgs, tx_ep_cfgs),
                      USB::USBSpec::USB_2_1, USB::Speed::FULL, packet_size, vid, pid, bcd,
                      languages, configs, uid)
{
}

USB::EndpointPool& CH32USBOtgFS::AddEndpoints(CH32USBOtgFS& self,
                                              const std::initializer_list<RawData>& rx,
                                              const std::initializer_list<RawData>& tx)
{
  const size_t count = rx.size() > tx.size() ? rx.size() : tx.size();
  REQUIRE(count > 0U && count <= Ep::EP_OTG_FS_MAX_SIZE);
  RawData rx_blocks[Ep::EP_OTG_FS_MAX_SIZE] = {};
  RawData tx_blocks[Ep::EP_OTG_FS_MAX_SIZE] = {};
  std::copy(rx.begin(), rx.end(), rx_blocks);
  std::copy(tx.begin(), tx.end(), tx_blocks);

  // EP0 两个方向共用一块（手册 p.355）。 / EP0's two directions share one block (manual
  // p.355).
  REQUIRE(rx_blocks[0].addr_ != nullptr && rx_blocks[0].addr_ == tx_blocks[0].addr_ &&
          rx_blocks[0].size_ >= 64U);
  USB::EndpointPool& pool = self;
  auto* out0 =
      new Ep(Endpoint::EPNumber::EP0, Endpoint::Direction::OUT, rx_blocks[0], nullptr);
  auto* in0 =
      new Ep(Endpoint::EPNumber::EP0, Endpoint::Direction::IN, rx_blocks[0], nullptr);
  pool.SetEndpoint0(in0, out0);
  REQUIRE(self.Register(*out0) == ErrorCode::OK);
  REQUIRE(self.Register(*in0) == ErrorCode::OK);

  Endpoint::EPNumber number = Endpoint::EPNumber::EP1;
  for (size_t i = 1U; i < count; ++i, number = Endpoint::NextEPNumber(number))
  {
    const bool has_rx = rx_blocks[i].addr_ != nullptr;
    const bool has_tx = tx_blocks[i].addr_ != nullptr;
    if (has_rx && has_tx)
    {
      // 共享（单 bank）：硬件的接收区 64 字节，发送区固定在其后 +64（手册表 23-4
      // p.359），所以配置给出的两块必须相邻，合起来就是硬件读写的 128 字节。槽另外
      // 分配一次，每个方向 64 字节，数据由 CPU 在槽和这 128 字节之间逐包拷贝。
      // Shared (single bank): the hardware has a 64-byte receive area with the transmit
      // area fixed at +64 behind it (manual table 23-4 p.359), so the two configured
      // blocks must be adjacent and together are the 128 bytes the hardware accesses.
      // The slots are allocated once here, 64 bytes per direction, and the CPU copies
      // packet by packet between a slot and those 128 bytes.
      REQUIRE(tx_blocks[i].addr_ == static_cast<uint8_t*>(rx_blocks[i].addr_) + 64U);
      REQUIRE(rx_blocks[i].size_ >= 64U && tx_blocks[i].size_ >= 64U);
      auto* block = static_cast<uint8_t*>(rx_blocks[i].addr_);
      auto* out =
          new Ep(number, Endpoint::Direction::OUT,
                 RawData(AllocateSlot(SHARED_SLOT_SIZE), SHARED_SLOT_SIZE), block);
      auto* in = new Ep(number, Endpoint::Direction::IN,
                        RawData(AllocateSlot(SHARED_SLOT_SIZE), SHARED_SLOT_SIZE), block);
      REQUIRE(pool.Put(out) == ErrorCode::OK && pool.Put(in) == ErrorCode::OK);
      REQUIRE(self.Register(*out) == ErrorCode::OK &&
              self.Register(*in) == ErrorCode::OK);
    }
    else if (has_rx || has_tx)
    {
      // 单向：两个槽各占块的一半。 / Single direction: two slots of half the block each.
      const RawData block = has_rx ? rx_blocks[i] : tx_blocks[i];
      REQUIRE(block.size_ >= 128U);
      auto* ep =
          new Ep(number, has_rx ? Endpoint::Direction::OUT : Endpoint::Direction::IN,
                 block, nullptr);
      REQUIRE(pool.Put(ep) == ErrorCode::OK);
      REQUIRE(self.Register(*ep) == ErrorCode::OK);
    }
  }
  return pool;
}

void CH32USBOtgFS::ResetEp0()
{
  Ep::map_[0][OUT_IDX]->tog_ = true;
  Ep::map_[0][IN_IDX]->tog_ = true;
  USBFSD->UEP0_TX_CTRL = USBFS_UEP_T_RES_NAK;
  USBFSD->UEP0_RX_CTRL = USBFS_UEP_R_RES_NAK;
}

void CH32USBOtgFS::ServiceHardware(bool in_isr)
{
  while (true)
  {
    const uint16_t fgst = *reinterpret_cast<volatile uint16_t*>(
        reinterpret_cast<uintptr_t>(&USBFSD->INT_FG));
    const uint8_t pending = static_cast<uint8_t>(fgst & OTG_FS_CLEARABLE_MASK);
    const uint8_t int_st = static_cast<uint8_t>(fgst >> 8U);
    if (pending == 0U)
    {
      break;
    }
    // 本轮看到的 UIF_TRANSFER 归事件循环，到本轮末尾才清。全速控制器每个 UIF_TRANSFER
    // 只有一个令牌，SETUP 也以令牌出现，所以 EP0 的完成天然在 SETUP 之前交付。
    // A UIF_TRANSFER seen in this round belongs to the event loop and is cleared at the
    // end of the round. The full-speed controller has one token per UIF_TRANSFER and a
    // SETUP comes as a token too, so EP0 completions are delivered before a SETUP by
    // construction.
    const bool transfer = (pending & USBFS_UIF_TRANSFER) != 0U;
    Ep::transfer_held_ = transfer;

    if ((pending & USBFS_UIF_BUS_RST) != 0U)
    {
      USBFSD->DEV_ADDR = 0U;
      // 复位期间总线上没有事务：未结束的停止在报告复位之前结束。
      // No transaction is on the bus during a reset: pending stops end before the reset
      // is reported.
      Ep::FinishQuietStops(in_isr, true, false);
      OnBusReset(in_isr, USB::Speed::FULL);
      ResetEp0();
      suspended_ = false;
    }

    if (transfer)
    {
      const uint8_t token = int_st & USBFS_UIS_TOKEN_MASK;
      const uint8_t ep_num = int_st & USBFS_UIS_ENDP_MASK;
      // 端点表只有 EP0-EP7；ENDP 字段有 4 位。 / The table holds EP0-EP7 only; the ENDP
      // field has 4 bits.
      Ep* const* eps = ep_num < Ep::EP_OTG_FS_MAX_SIZE ? Ep::map_[ep_num] : nullptr;
      switch (token)
      {
        case USBFS_UIS_TOKEN_SETUP:
        {
          // 8 字节先拷出 EP0 的块（EP0 两个方向和下一个 SETUP 都写这块）。
          // The 8 bytes are copied out of EP0's block first (both EP0 directions and the
          // next SETUP write that block).
          USB::SetupPacket setup{};
          std::memcpy(&setup, Ep::map_[0][OUT_IDX]->SlotMemory().addr_, sizeof(setup));
          // 先写 NAK，标志到本轮末尾才清：其间控制器对所有令牌回 NAK，控制端点上未
          // 结束的停止在交付 SETUP 之前结束，交付期间对它的停止当场完成。
          // NAK is written first and the flag is cleared at the end of the round:
          // meanwhile the controller answers every token with NAK, a pending stop on
          // the control endpoint ends before the SETUP is handed over, and a stop of it
          // during the hand-over is complete at once.
          ResetEp0();
          Ep::setup_held_ = true;
          Ep::FinishQuietStops(in_isr, true, true);
          OnSetup(in_isr, setup);
          Ep::setup_held_ = false;
          break;
        }
        case USBFS_UIS_TOKEN_OUT:
          if (eps != nullptr && eps[OUT_IDX] != nullptr)
          {
            eps[OUT_IDX]->TransferComplete(in_isr, USBFSD->RX_LEN,
                                           (int_st & USBFS_UIS_TOG_OK) != 0U);
          }
          break;
        case USBFS_UIS_TOKEN_IN:
          if (eps != nullptr && eps[IN_IDX] != nullptr)
          {
            eps[IN_IDX]->TransferComplete(in_isr, 0U, true);
          }
          break;
        default:
          break;
      }
    }

    // SOF 不改写还没读出的完成的 INT_ST 和 RX_LEN（CH32V208 实测），所以放在本轮的传输
    // 事件之后处理：先结束等它的停止，再报告帧起始。这个控制器没有帧号寄存器，帧号报
    // “不知道”。
    // A SOF does not rewrite the INT_ST and RX_LEN of a completion not yet read
    // (measured on a CH32V208), so it is handled after the transfer event of the round:
    // the stops waiting for it end first, then the start of frame is reported. This
    // controller has no frame number register and the number is reported as unknown.
    if ((pending & USBFS_UIF_HST_SOF) != 0U)
    {
      if (Ep::pending_stops_ != 0U)
      {
        Ep::FinishStopsAtSof(in_isr);
      }
      // 总线挂起期间这个标志仍然每毫秒来一次（CH32V208 实测：挂起的 3149 ms 里 3150
      // 次），那不是主机的帧起始，不报告。
      // While the bus is suspended the flag still comes every millisecond (measured on a
      // CH32V208: 3150 times in 3149 ms of suspend); that is no start of frame from the
      // host and is not reported.
      if ((USBFSD->MIS_ST & USBFS_UMS_SUSPEND) == 0U)
      {
        OnStartOfFrame(in_isr, 0xFFFFU);
      }
    }

    // 等时事务不受 INT_BUSY 约束（CH32V208 实测：把 UIF_TRANSFER 挂住 1.5 ms，297 次里
    // 每次都照样收进了下一个等时 OUT 包，并改写 INT_ST 和 RX_LEN）。本轮处理期间完成的
    // 等时事务并进了已经置位的标志，会随下面的清除一起消失，包留在槽里没人交付（实测
    // 180 s 漏 111 包，都落在帧开头）。所以清标志之前再看 INT_ST：令牌或端点已不是刚
    // 处理的那个、不带 IS_NAK、指向一个有请求在途的等时端点，就先交付它的完成。清除
    // 之后令牌字段不再表示方向（实测读成 11）。这次检查时数据已进槽、事务还没结束的那
    // 一个要到清除之后才看得出来，见下面。
    // Isochronous transactions are not held off by INT_BUSY (measured on a CH32V208:
    // with UIF_TRANSFER held for 1.5 ms the next isochronous OUT packet was taken in all
    // 297 cases and rewrote INT_ST and RX_LEN). One that completes while the round is
    // handled merges into the flag already set and would vanish with the clear below,
    // its packet left in the slot undelivered (measured: 111 packets missed in 180 s,
    // all at the start of a frame). So INT_ST is looked at again before the clear: a
    // token or endpoint other than the one just handled, without IS_NAK and pointing at
    // an isochronous endpoint with a request in flight, has its completion delivered
    // first. After the clear the token field no longer tells the direction (it measured
    // as 11). A transaction whose data is in the slot but which is not over at this
    // check shows only after the clear, see below.
    uint8_t handled = int_st;
    if (transfer)
    {
      for (uint8_t i = 0U; i < MERGED_EVENTS_MAX; ++i)
      {
        const uint8_t now = USBFSD->INT_ST;
        const uint16_t now_len = USBFSD->RX_LEN;
        const uint8_t now_token = now & USBFS_UIS_TOKEN_MASK;
        const uint8_t now_ep = now & USBFS_UIS_ENDP_MASK;
        const bool is_out = now_token == USBFS_UIS_TOKEN_OUT;
        if ((now & USBFS_UIS_IS_NAK) != 0U ||
            ((now ^ handled) & (USBFS_UIS_TOKEN_MASK | USBFS_UIS_ENDP_MASK)) == 0U ||
            !(is_out || now_token == USBFS_UIS_TOKEN_IN) || now_ep == 0U ||
            now_ep >= Ep::EP_OTG_FS_MAX_SIZE)
        {
          break;
        }
        Ep* ep = Ep::map_[now_ep][is_out ? OUT_IDX : IN_IDX];
        if (ep == nullptr || !ep->transfer_active_ ||
            ep->GetType() != USB::Endpoint::Type::ISOCHRONOUS)
        {
          break;
        }
        ep->TransferComplete(in_isr, is_out ? now_len : 0U,
                             (now & USBFS_UIS_TOG_OK) != 0U);
        handled = now;
      }
    }

    USBFSD->INT_FG = pending;
    Ep::transfer_held_ = false;

    // 上面那次检查时还没结束、随后并进标志又被清掉的等时事务（CH32V208 实测：检查时
    // INT_ST 还是刚处理的 0x23、RX_LEN 1022，清除之后是 0x76、RX_LEN 8）。清除之后
    // INT_ST 的端点号和 RX_LEN 仍然有效，令牌位无效：端点号不是刚处理的那个、不带
    // IS_NAK、这个端点号恰好只有一个方向有等时请求在途，就交付它。先读 INT_ST 和 RX_LEN
    // 再读标志：读完之后才完成的事务会把标志置位，留给下一轮。同一个端点号两个方向都有
    // 等时请求在途时分不出方向，不交付。
    // An isochronous transaction that was not over at the check above, then merged into
    // the flag and was cleared (measured on a CH32V208: at the check INT_ST was still
    // the 0x23 just handled with RX_LEN 1022, after the clear 0x76 with RX_LEN 8). After
    // the clear the endpoint number of INT_ST and RX_LEN are still valid and the token
    // bits are not: an endpoint number other than the one just handled, without IS_NAK,
    // on which exactly one direction has an isochronous request in flight, gets that
    // request completed. INT_ST and RX_LEN are read before the flag: a transaction
    // completing after these reads sets the flag and is left to the next round. With
    // isochronous requests in flight in both directions of one number the direction
    // cannot be told and nothing is delivered.
    if (transfer)
    {
      const uint8_t late = USBFSD->INT_ST;
      const uint16_t late_len = USBFSD->RX_LEN;
      const uint8_t late_ep = late & USBFS_UIS_ENDP_MASK;
      if ((USBFSD->INT_FG & USBFS_UIF_TRANSFER) == 0U &&
          (late & USBFS_UIS_IS_NAK) == 0U && late_ep != 0U &&
          late_ep < Ep::EP_OTG_FS_MAX_SIZE && late_ep != (handled & USBFS_UIS_ENDP_MASK))
      {
        Ep* out = Ep::map_[late_ep][OUT_IDX];
        Ep* in = Ep::map_[late_ep][IN_IDX];
        const bool out_waits = out != nullptr && out->transfer_active_ &&
                               out->GetType() == USB::Endpoint::Type::ISOCHRONOUS;
        const bool in_waits = in != nullptr && in->transfer_active_ &&
                              in->GetType() == USB::Endpoint::Type::ISOCHRONOUS;
        if (out_waits && !in_waits)
        {
          out->TransferComplete(in_isr, late_len, (late & USBFS_UIS_TOG_OK) != 0U);
        }
        else if (in_waits && !out_waits)
        {
          in->TransferComplete(in_isr, 0U, true);
        }
      }
    }
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
  const bool suspended = (USBFSD->MIS_ST & USBFS_UMS_SUSPEND) != 0U;
  if (!halted_ && suspended != suspended_)
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
  // 没有待处理的事件了：SIE 空闲的话，未结束的停止都已停稳。
  // No event is waiting any more: with a free SIE every pending stop is quiescent.
  Ep::FinishQuietStops(in_isr, false, false);
}

void CH32USBOtgFS::MaskInterrupt() { NVIC_DisableIRQ(USBFS_IRQn); }

void CH32USBOtgFS::UnmaskInterrupt() { NVIC_EnableIRQ(USBFS_IRQn); }

bool CH32USBOtgFS::PendInterrupt()
{
  NVIC_SetPendingIRQ(USBFS_IRQn);
  return true;
}

void CH32USBOtgFS::SetAddress(uint8_t address, AddressStage stage)
{
  if (stage == AddressStage::STATUS_COMPLETE)
  {
    USBFSD->DEV_ADDR =
        static_cast<uint8_t>((USBFSD->DEV_ADDR & USBFS_UDA_GP_BIT) | address);
    USBFSD->UEP0_TX_CTRL = USBFS_UEP_T_RES_NAK;
    USBFSD->UEP0_RX_CTRL = USBFS_UEP_R_RES_ACK;
  }
}

void CH32USBOtgFS::OnControlReady() { USBFSD->UEP0_RX_CTRL = USBFS_UEP_R_RES_ACK; }

void CH32USBOtgFS::SetUpController()
{
  USBFSH->BASE_CTRL = USBFS_UC_RESET_SIE | USBFS_UC_CLR_ALL;
  USBFSH->BASE_CTRL = 0x00U;
  // SOF 中断常开：每个帧起始报告给设备类，未结束的停止也靠它结束。
  // The SOF interrupt is always on: every start of frame is reported to the device
  // classes, and pending stops end by it as well.
  USBFSD->INT_EN =
      USBFS_UIE_SUSPEND | USBFS_UIE_BUS_RST | USBFS_UIE_TRANSFER | USBFS_UIE_HST_SOF;
  USBFSD->DEV_ADDR = 0U;
  USBFSD->BASE_CTRL = USBFS_UC_DEV_PU_EN | USBFS_UC_INT_BUSY | USBFS_UC_DMA_EN;
  USBFSD->UDEV_CTRL = USBFS_UD_PD_DIS | USBFS_UD_PORT_EN;
}

void CH32USBOtgFS::StartController(bool in_isr)
{
  UNUSED(in_isr);
  // 与 FSDEV 共用 48 MHz 时钟选择规则。 / Same 48 MHz clock rule as FSDEV.
  CH32UsbRcc::ConfigureUsb48M();
#if defined(RCC_USBCLK48MCLKSource_USBPHY) && defined(RCC_AHBPeriph_USBHS)
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_USBHS, ENABLE);
#endif
#if defined(RCC_AHBPeriph_USBFS)
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_USBFS, ENABLE);
#elif defined(RCC_AHBPeriph_USBOTGFS)
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_USBOTGFS, ENABLE);
#else
#error "The SDK header of this part names no bus clock for the USBFS controller."
#endif
  WaitDetached();
  SetUpController();
  // 先登记对象再开中断线：没有对象时中断只清标志，会丢掉这期间的总线复位。
  // Register the object before the line is enabled: without an object the interrupt
  // only clears the flags and would drop a bus reset arriving in between.
  self_ = this;
  NVIC_EnableIRQ(USBFS_IRQn);
}

void CH32USBOtgFS::HaltController()
{
  // SIE 复位后硬件不再为任何端点收发；端点上的请求和未结束的停止作废，之后的
  // StopHardware 立即返回 ABORTED。上拉和中断源都关掉。
  // After the SIE reset the hardware moves no data for any endpoint; the requests and
  // the pending stops on the endpoints are void and every later StopHardware returns
  // ABORTED at once. The pull-up and the interrupt sources are off.
  USBFSH->BASE_CTRL = USBFS_UC_RESET_SIE | USBFS_UC_CLR_ALL;
  USBFSD->BASE_CTRL = 0x00U;
  USBFSD->INT_EN = 0x00U;
  halted_at_us_ = static_cast<uint64_t>(Timebase::GetMicroseconds());
  halted_ = true;
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

void CH32USBOtgFS::StopController(bool in_isr)
{
  UNUSED(in_isr);
  // 中断线保持使能、对象保持登记：owner 靠这条线运行。
  // The line stays enabled and the object registered: the owner runs on this line.
  HaltController();
}

void CH32USBOtgFS::ResetController(bool in_isr)
{
  UNUSED(in_isr);
  // 拔出一段时间再接上，主机看到断开后重新枚举。
  // Detached for a while and attached again, the host sees a disconnect and enumerates
  // again.
  HaltController();
  WaitDetached();
  SetUpController();
}

void CH32USBOtgFS::WaitDetached()
{
  // 断开要保持到主机看得见：全速下只断开几十微秒，主机 5 次里 4 次看不见（CH32V208
  // 实测），5 ms 以上都看得见。
  // The detach has to last until the host sees it: at full speed a detach of some tens
  // of microseconds went unseen 4 times out of 5 (measured on a CH32V208); 5 ms and
  // more were always seen.
  if (!halted_)
  {
    return;
  }
  while (static_cast<uint64_t>(Timebase::GetMicroseconds()) - halted_at_us_ < DETACH_US)
  {
  }
  halted_ = false;
}

#endif  // defined(USBFSD)

// NOLINTEND(cppcoreguidelines-pro-type-cstyle-cast,performance-no-int-to-ptr)
