#pragma once

#include <cstring>

#include "usb/core/ep.hpp"
#include "usb/core/ep_pool.hpp"
#include "usb/core/usb_macro_guard.hpp"
#include "usb/device/dev_core.hpp"

/**
 * @file fsdev.hpp
 * @brief STM32 FSDEV 与 CH32 DevFS 共用的 PMA 型寄存器运行期 / Shared register runtime
 *        for the PMA-based FSDEV peripherals (STM32 FSDEV and CH32 DevFS)
 *
 * 硬件只有每方向一个 PMA 包缓冲；槽模型的两槽完全由软件实现（CPU 把数据在槽与
 * PMA 之间逐包拷贝）。停止 = STAT 写 NAK，立即生效并返回 `StopResult::DONE`；
 * 迟到的 CTR 在停止时读掉并计入进度。等时端点不支持。
 *
 * The hardware offers one PMA packet buffer per direction; the two slots of the slot
 * model live entirely in software (the CPU copies packet by packet between the slot and
 * the PMA). A stop writes STAT = NAK, takes effect at once and returns
 * `StopResult::DONE`; a late CTR is read and counted when the stop runs. Isochronous
 * endpoints are not supported.
 *
 * 芯片差异收敛在模板参数 `HW`（硬件访问 traits）上：寄存器位与掩码常量、
 * BTABLE/PMA 访问、上电复位与挂起/唤醒序列、上拉。经典型（F1/F3/F0/L0/G4/WB）与
 * DRD 变体（G0/C0/H5/U5/U0/U3 的 USB_DRD_FS）各有一份 traits。
 *
 * Chip differences are contained in the template parameter `HW`, the hardware-access
 * traits: register bits and masks, BTABLE/PMA access, the power-on reset and
 * suspend/resume sequences and the pull-up. The classic controllers (F1/F3/F0/L0/G4/WB)
 * and the DRD variant (USB_DRD_FS on G0/C0/H5/U5/U0/U3) each have one traits type.
 *
 * Traits 约定 / Traits contract:
 * - 常量（uint32_t，经典 16 位寄存器零扩展）/ Constants (uint32_t, the classic 16-bit
 *   registers zero-extended): `kEpCtrRx kEpDtogRx kEpStatRx kEpSetup kEpType kEpKind
 *   kEpCtrTx kEpDtogTx kEpStatTx kEpAddr`；经典型的 `kEpErrRx = kEpErrTx = 0`；
 *   `kEpKeepMask` = 写 EPnR 时写 1 保持的全部位（CTR、SETUP、类型、KIND、地址，
 *   DRD 再加 ERR/NAK/DEVADDR/LSEP）。
 * - `kIstrCtr kIstrReset kIstrSof kIstrSusp kIstrWkup kIstrNoise`（噪声位 = ERR|
 *   PMAOVR|ESOF，只清标志）；`kCntrRunMask` = CTRM|RESETM|SUSPM|WKUPM；`kCntrSofm`。
 * - 函数 / Functions: `ReadEp WriteEp`（EPnR，32 位视图）、`ReadCntr WriteCntr
 *   ReadIstr ClearIstr ReadFnr WriteDaddr`（CNTR/ISTR/FNR/DADDR；`ClearIstr` 用写 0
 *   清位、写 1 保持其余位的 ISTR 写法）、`SetTxBuf SetTxCount SetRxBuf
 *   SetRxCountEncoded GetRxCount`（BTABLE，地址与计数以存储字节为单位）、
 *   `PmaWrite PmaRead`（PMA 拷贝）、`InitRegs SuspendRegs SuspendClocked ResumeRegs
 *   Connect MspInit`、`EnterGuard ExitGuard`（硬件守卫：屏蔽本地中断，返回保存值）。
 *   CTR 的清除由运行期按 `kEpKeepMask` 公式统一完成，traits 不需要单独的函数。
 * - `kPmaSize`：PMA 存储字节；`kEndpoints`：硬件端点数。
 */

namespace LibXR::USB
{

/// 经典型与 DRD 一致的 STAT/DTOG 目标值 / STAT and DTOG targets shared by both
/// variants (classic 16-bit layout)
constexpr uint32_t kFsDevStatTxDis = 0x0000U;
constexpr uint32_t kFsDevStatTxStall = 0x0010U;
constexpr uint32_t kFsDevStatTxNak = 0x0020U;
constexpr uint32_t kFsDevStatTxValid = 0x0030U;
constexpr uint32_t kFsDevStatRxDis = 0x0000U;
constexpr uint32_t kFsDevStatRxStall = 0x1000U;
constexpr uint32_t kFsDevStatRxNak = 0x2000U;
constexpr uint32_t kFsDevStatRxValid = 0x3000U;

/// 清除本方向 CTR（写 0），另一方向 CTR 与 ERR 写 1 保持；STAT/DTOG 写 0 不翻转。
/// Clears the direction's CTR (write 0) while keeping the other direction's CTR and
/// ERR (write 1); STAT/DTOG are written 0 and stay unchanged.
template <typename HW>
inline void FsDevClearCtrTx(uint8_t num)
{
  constexpr uint32_t kSelf = HW::kEpCtrTx | HW::kEpErrTx;
  constexpr uint32_t kOther = HW::kEpCtrRx | HW::kEpErrRx;
  const uint32_t cur = HW::ReadEp(num);
  HW::WriteEp(num, (cur & HW::kEpKeepMask & ~kSelf) | kOther);
}

template <typename HW>
inline void FsDevClearCtrRx(uint8_t num)
{
  constexpr uint32_t kSelf = HW::kEpCtrRx | HW::kEpErrRx;
  constexpr uint32_t kOther = HW::kEpCtrTx | HW::kEpErrTx;
  const uint32_t cur = HW::ReadEp(num);
  HW::WriteEp(num, (cur & HW::kEpKeepMask & ~kSelf) | kOther);
}

/// EP_TYPE 编码（bits[9:8]）：CONTROL=01、ISO=10、BULK=00、INT=11 / EP_TYPE encoding
/// (bits [9:8]): CONTROL=01, ISO=10, BULK=00, INT=11
constexpr uint32_t FsDevTypeBits(Endpoint::Type type)
{
  switch (type)
  {
    case Endpoint::Type::CONTROL:
      return 0x0200U;
    case Endpoint::Type::ISOCHRONOUS:
      return 0x0400U;
    case Endpoint::Type::INTERRUPT:
      return 0x0600U;
    case Endpoint::Type::BULK:
    default:
      return 0x0000U;
  }
}

template <typename HW>
class FSDevDevice;

/**
 * @brief FSDEV 端点 / FSDEV endpoint
 *
 * IN：启动时把第一个包拷进 PMA 并置 VALID，每个 CTR_TX（包被主机 ACK）在 ISR 里
 * 推进下一包；OUT：启动时挂接收并置 VALID，每个 CTR_RX 把包从 PMA 读进槽，短包或
 * 收满请求长度时完成。SETUP 到达时设备读走 8 字节交给核心。
 *
 * IN: the first packet is copied into the PMA at start and STAT set VALID; every
 * CTR_TX (the packet was ACKed) advances the next packet in the interrupt. OUT: the
 * receive is armed at start; every CTR_RX reads the packet from the PMA into the slot,
 * and the request completes on a short packet or when the requested length is reached.
 * On SETUP the device reads the 8 bytes and hands them to the core.
 */
template <typename HW>
class FSDevEndpoint : public Endpoint
{
 public:
  FSDevEndpoint(FSDevDevice<HW>& device, EPNumber number, Direction direction,
                RawData buffer, uint16_t pma_addr, size_t pma_size,
                uint8_t slot_count = 0)
      : Endpoint(number, direction, buffer, slot_count),
        device_(device),
        pma_addr_(pma_addr),
        pma_size_(pma_size)
  {
    REQUIRE(EPNumberToInt8(GetNumber()) < HW::kEndpoints);
    if (direction == Direction::IN)
    {
      device_.in_[EPNumberToInt8(GetNumber())] = this;
    }
    else
    {
      device_.out_[EPNumberToInt8(GetNumber())] = this;
    }
  }

 protected:
  ErrorCode ConfigureHardware(const Config& cfg) override;
  void CloseHardware(bool in_isr) override;
  ErrorCode StartHardware(const HwRequest& request, bool in_isr) override;
  StopResult StopHardware(bool in_isr) override;
  size_t ReapProgress(const HwRequest& request, bool& retired) override;
  ErrorCode StallHardware(bool in_isr) override;
  ErrorCode ClearStallHardware(bool in_isr) override;
  size_t MaxPacketSizeLimit(Type type) const override;

 private:
  friend class FSDevDevice<HW>;

  bool IsIn() const { return GetConfig().direction == Direction::IN; }
  uint8_t Num() const { return EPNumberToInt8(GetNumber()); }

  // EPnR 的 STAT/DTOG 写 1 翻转、其余位写 1 保持。只在 owner（HardwareScope 内）或
  // ISR 中调用：同一控制器的 IRQ 不嵌套。
  // STAT/DTOG of EPnR toggle on a written 1, the other bits keep on a written 1. Call it
  // only in the owner (inside a HardwareScope) or in the ISR: the IRQs of one controller
  // do not nest.
  void SetStat(bool in, uint32_t target);
  void WriteType(uint32_t bits);
  void ClearDataToggle(bool in);
  void ArmTx(size_t bytes);
  void ArmRx();

  // IRQ 侧：每个 CTR_TX / CTR_RX 一个包。
  void OnTxPacket();
  void OnRxPacket(size_t bytes);

  FSDevDevice<HW>& device_;
  uint16_t pma_addr_;  ///< PMA 存储字节偏移，构造期固定 / PMA byte offset, fixed at
                       ///< construction
  size_t pma_size_;

  // 在途请求状态：owner 在硬件守卫内写，ISR 读 / The owner writes it inside the
  // hardware guard, the ISR reads it.
  uint32_t hw_tag_ = 0;
  const uint8_t* hw_data_ = nullptr;
  size_t hw_length_ = 0;
  size_t hw_progress_ = 0;  ///< IN：已 ACK 字节；OUT：已收字节 / IN ACKed, OUT received
  size_t stopped_actual_ = 0;
};

/**
 * @brief FSDEV 设备 / FSDEV device
 *
 * PMA 布局构造期固定：BTABLE 每端点 8 字节放在 0，之后按端点号依次分配 OUT、IN
 * 缓冲。端点号按方向从 EP0 依次分配：RX 列表第 i 项是 EP i 的 OUT，TX 列表第 i 项
 * 是 EP i 的 IN；EP0 的两个方向必须提供。同一端点号 IN、OUT 共用类型字段，两方向
 * 配置不同类型时 Configure 报错。
 *
 * The PMA layout is fixed at construction: the BTABLE takes 8 bytes per endpoint at 0,
 * followed by the OUT and IN buffers in endpoint order. Endpoint numbers are assigned
 * per direction from EP0: entry i of RX is EP i OUT, entry i of TX is EP i IN; both
 * directions of EP0 are mandatory. IN and OUT of one endpoint number share the type
 * field, and Configure fails when the two directions request different types.
 */
template <typename HW>
class FSDevDevice : public EndpointPool, public DeviceCore<FullSpeedCapabilities>
{
 public:
  /// 端点配置：槽内存与 PMA 缓冲字节 / Endpoint configuration: slot memory and PMA
  /// bytes
  struct EPConfig
  {
    RawData buffer;
    size_t pma_size;
  };

  /**
   * @param pma_limit PMA 可用字节（构造期断言）；0 = 全部 / Usable PMA bytes
   *                  (asserted at construction); 0 = all of it
   */
  FSDevDevice(
      const std::initializer_list<EPConfig>& rx_ep_cfgs,
      const std::initializer_list<EPConfig>& tx_ep_cfgs,
      DeviceDescriptor::PacketSize0 packet_size, uint16_t vid, uint16_t pid, uint16_t bcd,
      const std::initializer_list<const DescriptorStrings::LanguagePack*>& lang_list,
      const std::initializer_list<const std::initializer_list<ConfigDescriptorItem*>>&
          configs,
      ConstRawData uid = {nullptr, 0}, size_t pma_limit = 0, uint8_t slot_count = 0)
      : EndpointPool(rx_ep_cfgs.size() + tx_ep_cfgs.size()),
        DeviceCore<FullSpeedCapabilities>(*this, USBSpec::USB_2_1, Speed::FULL,
                                          packet_size, vid, pid, bcd, lang_list, configs,
                                          uid)
  {
    REQUIRE(rx_ep_cfgs.size() > 0U && rx_ep_cfgs.size() <= HW::kEndpoints);
    REQUIRE(tx_ep_cfgs.size() > 0U && tx_ep_cfgs.size() <= HW::kEndpoints);
    const size_t limit = pma_limit != 0U ? pma_limit : HW::kPmaSize;

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
    EPConfig rx[kMaxEps] = {};
    EPConfig tx[kMaxEps] = {};
    uint16_t addr_out[kMaxEps] = {};
    uint16_t addr_in[kMaxEps] = {};
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
    REQUIRE((offset % HW::kPmaAlign) == 0U);
    for (size_t i = 0; i < rx_ep_cfgs.size(); ++i)
    {
      REQUIRE((rx[i].pma_size % HW::kPmaAlign) == 0U);
      addr_out[i] = static_cast<uint16_t>(offset);
      offset += rx[i].pma_size;
    }
    for (size_t i = 0; i < tx_ep_cfgs.size(); ++i)
    {
      REQUIRE((tx[i].pma_size % HW::kPmaAlign) == 0U);
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
        static_cast<uint16_t>(HW::ReadCntr() & ~(HW::kCntrRunMask | HW::kCntrSofm)));
  }

  void Init(bool in_isr) override { DeviceCore<FullSpeedCapabilities>::Init(in_isr); }
  void Deinit(bool in_isr) override { DeviceCore<FullSpeedCapabilities>::Deinit(in_isr); }

  ~FSDevDevice() override
  {
    if (self_ == this)
    {
      self_ = nullptr;
    }
  }

  void Start(bool in_isr) override
  {
    UNUSED(in_isr);
    {
      EndpointPool::HardwareScope hardware(this);
      connected_ = true;
      uint16_t cntr = HW::ReadCntr();
      cntr |= HW::kCntrRunMask;
      if (WantsBusTime())
      {
        cntr |= HW::kCntrSofm;
      }
      else
      {
        cntr &= static_cast<uint16_t>(~HW::kCntrSofm);
      }
      HW::WriteCntr(cntr);
      // 上拉最后接通：前面的初始化不产生主机可见的变化。
      // The pull-up comes on last: the initialization before it changes nothing the host
      // can see.
      HW::Connect(true);
    }
  }

  void Stop(bool in_isr) override
  {
    UNUSED(in_isr);
    {
      EndpointPool::HardwareScope hardware(this);
      connected_ = false;
      HW::Connect(false);
      HW::WriteCntr(
          static_cast<uint16_t>(HW::ReadCntr() & ~(HW::kCntrRunMask | HW::kCntrSofm)));
    }
    // 下拉之后主机不再通信；在途请求的完成事件不会再来，由下一次解绑或复位流程结束。
    // After the pull-up is off the host no longer talks; completions of requests in
    // flight never arrive, and the next unbind or reset ends them.
  }

  ErrorCode SetAddress(uint8_t address, ControlContext context) override
  {
    // 经典型在状态阶段 IN 完成后写 DADDR（RM0008 §23.4.5）。
    // The classic type writes DADDR after the status-stage IN completes (RM0008
    // §23.4.5).
    if (context == ControlContext::STATUS_IN_COMPLETE)
    {
      EndpointPool::HardwareScope hardware(this);
      HW::WriteDaddr(static_cast<uint16_t>(0x0080U | address));
    }
    return ErrorCode::OK;
  }

  void OnControllerReset() override { discard_ = false; }

  void OnStopsSettled() override
  {
    if (!connected_)
    {
      // 已 Stop（上拉拉低）之后不会再有 SOF；不重开 SOFM，避免未连接状态下
      // 出现"中断使能开着却没有事务"的困惑。
      // After Stop() (pull-up off) no SOF comes; SOFM is not unmasked again, so a
      // disconnected device never shows an enabled interrupt without traffic.
      return;
    }
    EndpointPool::HardwareScope hardware(this);
    // 配置切换后重估 SOFM（与 OTG 的 UpdateTick 对齐）：WantsBusTime() 为真时开，
    // 否则关。SOFM 只是中断使能，设备未 Start 时写它无副作用。
    // Re-evaluate SOFM after a configuration change (as UpdateTick in OTG): on while
    // WantsBusTime() is true, off otherwise. SOFM is only an interrupt enable; writing it
    // before Start() has no side effect.
    uint16_t cntr = HW::ReadCntr();
    if (WantsBusTime())
    {
      cntr |= HW::kCntrSofm;
    }
    else
    {
      cntr &= static_cast<uint16_t>(~HW::kCntrSofm);
    }
    HW::WriteCntr(cntr);
  }

  Speed ReadBusSpeed() const override { return Speed::FULL; }

  /// 中断入口（STM32 wrap 或 CH32 共享中断） / Interrupt entry (STM32 wrap or the
  /// shared CH32 vector)
  static void Dispatch()
  {
    FSDevDevice* device = self_;
    if (device == nullptr || !device->IsInited())
    {
      return;
    }
    EndpointPool::InterruptScope batch(*device);
    device->HandleInterrupt();
  }

 protected:
  static constexpr uint8_t kMaxEps = 8;

  template <typename U>
  friend class FSDevEndpoint;

  void HandleInterrupt();
  void HandleCtr();
  void HandleEp(uint8_t num);
  void OnBusResetRegs();
  void HandleSof();

  FSDevEndpoint<HW>* in_[kMaxEps] = {};
  FSDevEndpoint<HW>* out_[kMaxEps] = {};
  uint8_t endpoint_count_ = 1;
  bool connected_ = false;  ///< Start/Stop：设备是否属于总线 / Whether the device
                            ///< belongs on the bus
  bool discard_ = false;    ///< 复位已见、owner 未应用 / Reset seen, not yet applied
  alignas(4) uint8_t setup_[8] = {};

  static inline FSDevDevice* self_ = nullptr;
};

// ---------------------------------------------------------------------------
// Endpoint
// ---------------------------------------------------------------------------

template <typename HW>
void FSDevEndpoint<HW>::SetStat(bool in, uint32_t target)
{
  const uint8_t num = Num();
  const uint32_t stat_mask = in ? HW::kEpStatTx : HW::kEpStatRx;
  const uint32_t ctr = in ? HW::kEpCtrTx : HW::kEpCtrRx;
  const uint32_t valid = in ? kFsDevStatTxValid : kFsDevStatRxValid;
  const bool arming = (target == valid);
  // 写 1 翻转：只写到达目标的增量。读与写（以及写与读回）之间硬件可能自己
  // 完成一次事务（结束 IN/OUT 并自行把 STAT 改回 NAK、锁存 CTR），按旧值算
  // 增量会把 STAT 翻错。所以写后回读核对，不对就按新值重算重写（有界）。
  // CTR/ERR 是 rc_w0：这些写永远写 1 保持，窗口里锁存的 CTR 不能被抹掉。
  // 装填（目标 VALID）时若发现本方向 CTR 已锁存，说明写入已生效且这笔事务
  // 已经完成——STAT 是硬件自动回的 NAK，绝不能重试：重试会把端点再次装填
  // （IN 用同一份 PMA 数据重发一包、OUT 多收一包）。完成的事务交给 CTR 处
  // 理推进。停止类目标（NAK/STALL/DISABLED）没有二次装填问题，窗口里完成
  // 的事务只会把 STAT 推回 NAK，按新值重算重写即可收敛，不能提前放弃——
  // 否则停止路径等于没停住。
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
  constexpr uint32_t kNoClear = HW::kEpCtrRx | HW::kEpCtrTx | HW::kEpErrRx | HW::kEpErrTx;
  for (unsigned attempt = 0U; attempt < 3U; ++attempt)
  {
    const uint32_t cur = HW::ReadEp(num);
    if (arming && (cur & ctr) != 0U)
    {
      return;  // 本方向事务已完成，由 CTR 处理推进
    }
    const uint32_t value =
        (cur & HW::kEpKeepMask) | ((cur ^ target) & stat_mask) | kNoClear;
    HW::WriteEp(num, value);
    const uint32_t after = HW::ReadEp(num);
    if ((after & stat_mask) == target)
    {
      return;
    }
    if (arming && (after & ctr) != 0U)
    {
      return;  // 装填后事务在窗口内完成：STAT 是硬件自动回的 NAK，不重试
    }
  }
  // 三次重算都没到达目标且（装填时）CTR 未锁存：读回路径异常，NAK/装填语义
  // 已不可靠（停止路径等于没停住），不能静默继续。
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
  constexpr uint32_t kNoClear = HW::kEpCtrRx | HW::kEpCtrTx | HW::kEpErrRx | HW::kEpErrTx;
  const uint32_t value =
      (cur & HW::kEpKeepMask & ~(HW::kEpType | HW::kEpKind | HW::kEpAddr)) | bits |
      Num() | kNoClear;
  HW::WriteEp(num, value);
}

template <typename HW>
void FSDevEndpoint<HW>::ClearDataToggle(bool in)
{
  const uint8_t num = Num();
  const uint32_t dtog = in ? HW::kEpDtogTx : HW::kEpDtogRx;
  // CTR/ERR 写 1 保持（同 SetStat）。
  // CTR/ERR keep on a written 1 (as in SetStat).
  constexpr uint32_t kNoClear = HW::kEpCtrRx | HW::kEpCtrTx | HW::kEpErrRx | HW::kEpErrTx;
  const uint32_t cur = HW::ReadEp(num);
  if ((cur & dtog) != 0U)
  {
    HW::WriteEp(num, (cur & HW::kEpKeepMask) | dtog | kNoClear);
  }
}

template <typename HW>
void FSDevEndpoint<HW>::ArmTx(size_t bytes)
{
  HW::SetTxCount(Num(), static_cast<uint16_t>(bytes));
  SetStat(true, kFsDevStatTxValid);
}

template <typename HW>
void FSDevEndpoint<HW>::ArmRx()
{
  HW::SetRxCountEncoded(Num(), MaxPacketSize());
  SetStat(false, kFsDevStatRxValid);
}

template <typename HW>
size_t FSDevEndpoint<HW>::MaxPacketSizeLimit(Type type) const
{
  size_t cap = pma_size_;
  // OUT 的 COUNT_RX 编码按 2 B / 32 B 块取整。
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
  SetStat(IsIn(), IsIn() ? kFsDevStatTxNak : kFsDevStatRxNak);
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
  SetStat(IsIn(), IsIn() ? kFsDevStatTxDis : kFsDevStatRxDis);
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
    if ((cur & HW::kEpCtrTx) != 0U)
    {
      hw_progress_ +=
          LibXR::min(static_cast<size_t>(MaxPacketSize()), hw_length_ - hw_progress_);
      FsDevClearCtrTx<HW>(num);
    }
    SetStat(true, kFsDevStatTxNak);
    // SetStat 的读改写窗口里硬件可能又 ACK 完一包（CTR 已锁存）：补计一次。
    // In the read-modify-write window of SetStat the hardware may ACK one more packet
    // (CTR latched): count it once more.
    if ((HW::ReadEp(num) & HW::kEpCtrTx) != 0U)
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
    if ((cur & HW::kEpCtrRx) != 0U)
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
    SetStat(false, kFsDevStatRxNak);
    // SetStat 的读改写窗口里硬件可能又收完一包（CTR 已锁存）：补收补计一次。
    // In the read-modify-write window of SetStat the hardware may receive one more packet
    // (CTR latched): receive and count it once more.
    if ((HW::ReadEp(num) & HW::kEpCtrRx) != 0U)
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
  SetStat(IsIn(), IsIn() ? kFsDevStatTxStall : kFsDevStatRxStall);
  return ErrorCode::OK;
}

template <typename HW>
ErrorCode FSDevEndpoint<HW>::ClearStallHardware(bool in_isr)
{
  UNUSED(in_isr);
  if (Num() == 0U)
  {
    return ErrorCode::OK;  // 下一个 SETUP 清 EP0 的 STALL 与 DATA0/1。
  }
  ClearDataToggle(IsIn());  // 恢复 DATA0
  SetStat(IsIn(), IsIn() ? kFsDevStatTxNak : kFsDevStatRxNak);
  return ErrorCode::OK;
}

template <typename HW>
void FSDevEndpoint<HW>::OnTxPacket()
{
  const uint32_t tag = hw_tag_;
  if (tag == 0U)
  {
    return;  // 已停止，或重复事件。
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
// Device
// ---------------------------------------------------------------------------

template <typename HW>
void FSDevDevice<HW>::HandleInterrupt()
{
  const uint16_t istr = HW::ReadIstr();
  if ((istr & HW::kIstrReset) != 0U)
  {
    HW::ClearIstr(HW::kIstrReset);
    OnBusResetRegs();
  }
  if ((istr & HW::kIstrCtr) != 0U)
  {
    HandleCtr();
  }
  if ((istr & HW::kIstrSof) != 0U)
  {
    HW::ClearIstr(HW::kIstrSof);
    HandleSof();
  }
  if ((istr & HW::kIstrSusp) != 0U)
  {
    // 先进挂起再清标志、最后接时钟门控（HAL DRD 的顺序）；经典型的
    // SuspendClocked 为空。
    // Enter suspend first, then clear the flags, then gate the clocks (the HAL DRD
    // order); SuspendClocked is empty on the classic type.
    HW::SuspendRegs();
    HW::ClearIstr(HW::kIstrSusp);
    HW::SuspendClocked();
    OnSuspend(true);
  }
  if ((istr & HW::kIstrWkup) != 0U)
  {
    HW::ResumeRegs();
    HW::ClearIstr(HW::kIstrWkup);
    OnResume(true);
  }
  if ((istr & HW::kIstrNoise) != 0U)
  {
    HW::ClearIstr(HW::kIstrNoise);
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
    if ((istr & HW::kIstrCtr) == 0U)
    {
      break;
    }
    HandleEp(static_cast<uint8_t>(istr & 0x000FU));
    HW::ClearIstr(HW::kIstrCtr);
  }
}

template <typename HW>
void FSDevDevice<HW>::HandleEp(uint8_t num)
{
  const uint32_t epr = HW::ReadEp(num);
  const bool ctr_tx = (epr & HW::kEpCtrTx) != 0U;
  const bool ctr_rx = (epr & HW::kEpCtrRx) != 0U;
  const bool setup = (epr & HW::kEpSetup) != 0U;

  // 先 TX 后 RX：SETUP 会打断当前控制传输，之前的完成事件先走完。
  // TX before RX: a SETUP aborts the current control transfer, so earlier completions go
  // first.
  if (ctr_tx)
  {
    FsDevClearCtrTx<HW>(num);
    FSDevEndpoint<HW>* ep = num < kMaxEps ? in_[num] : nullptr;
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
      FSDevEndpoint<HW>* ep = num < kMaxEps ? out_[num] : nullptr;
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
    out_[0]->SetStat(false, kFsDevStatRxNak);  // EP0 OUT NAK：SETUP 照常接收
  }
  OnBusReset(true);
}

template <typename HW>
void FSDevDevice<HW>::HandleSof()
{
  OnSof(true, static_cast<uint16_t>(HW::ReadFnr() & 0x07FFU));
}

}  // namespace LibXR::USB
