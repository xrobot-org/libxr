#pragma once

#include <cstring>

#include "usb/core/ep.hpp"
#include "usb/core/ep_pool.hpp"
#include "usb/device/dev_core.hpp"

/**
 * @file fsdev.hpp
 * @brief PMA 型全速设备控制器（FSDEV）的寄存器运行期，由多个平台共用
 *        Register runtime of the PMA-based full-speed device controller (FSDEV), shared
 *        by several platforms
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
 * BTABLE/PMA 访问、上电复位与挂起/唤醒序列、上拉。经典型（16 位寄存器）与 DRD 变体
 * （32 位寄存器）各有一份 traits，由使用它的平台提供。
 *
 * Chip differences are contained in the template parameter `HW`, the hardware-access
 * traits: register bits and masks, BTABLE/PMA access, the power-on reset and
 * suspend/resume sequences and the pull-up. The classic controllers (16-bit registers)
 * and the DRD variant (32-bit registers) each have one traits type, provided by the
 * platform that uses it.
 *
 * Traits 约定 / Traits contract:
 * - 常量（uint32_t，经典 16 位寄存器零扩展）/ Constants (uint32_t, the classic 16-bit
 *   registers zero-extended): `EP_CTR_RX EP_DTOG_RX EP_STAT_RX EP_SETUP EP_TYPE EP_KIND
 *   EP_CTR_TX EP_DTOG_TX EP_STAT_TX EP_ADDR`；经典型的 `EP_ERR_RX = EP_ERR_TX = 0`；
 *   `EP_KEEP_MASK` = 写 EPnR 时写 1 保持的全部位（CTR、SETUP、类型、KIND、地址，
 *   DRD 再加 ERR/NAK/DEVADDR/LSEP）。
 * - `ISTR_CTR ISTR_RESET ISTR_SOF ISTR_SUSP ISTR_WKUP ISTR_NOISE`（噪声位 = ERR|
 *   PMAOVR|ESOF，只清标志）；`CNTR_RUN_MASK` = CTRM|RESETM|SUSPM|WKUPM；`CNTR_SOFM`。
 * - 函数 / Functions: `ReadEp WriteEp`（EPnR，32 位视图）、`ReadCntr WriteCntr
 *   ReadIstr ClearIstr ReadFnr WriteDaddr`（CNTR/ISTR/FNR/DADDR；`ClearIstr` 用写 0
 *   清位、写 1 保持其余位的 ISTR 写法）、`SetTxBuf SetTxCount SetRxBuf
 *   SetRxCountEncoded GetRxCount`（BTABLE，地址与计数以存储字节为单位）、
 *   `PmaWrite PmaRead`（PMA 拷贝）、`InitRegs SuspendRegs SuspendClocked ResumeRegs
 *   Connect MspInit`、`EnterGuard ExitGuard`（硬件守卫：屏蔽本地中断，返回保存值）。
 *   CTR 的清除由运行期按 `EP_KEEP_MASK` 公式统一完成，traits 不需要单独的函数。
 * - `PMA_SIZE`：PMA 存储字节；`ENDPOINT_COUNT`：硬件端点数。
 */

namespace LibXR::USB
{

/// 经典型与 DRD 一致的 STAT/DTOG 目标值 / STAT and DTOG targets shared by both
/// variants (classic 16-bit layout)
constexpr uint32_t FSDEV_STAT_TX_DIS = 0x0000U;
constexpr uint32_t FSDEV_STAT_TX_STALL =
    0x0010U;  ///< STAT_TX = STALL，回复 STALL 握手 / Reply STALL
constexpr uint32_t FSDEV_STAT_TX_NAK = 0x0020U;  ///< STAT_TX = NAK，回复 NAK / Reply NAK
constexpr uint32_t FSDEV_STAT_TX_VALID =
    0x0030U;  ///< STAT_TX = VALID，发送数据 / Send data
constexpr uint32_t FSDEV_STAT_RX_DIS =
    0x0000U;  ///< STAT_RX = DISABLED，不应答 / Ignore tokens
constexpr uint32_t FSDEV_STAT_RX_STALL =
    0x1000U;  ///< STAT_RX = STALL，回复 STALL 握手 / Reply STALL
constexpr uint32_t FSDEV_STAT_RX_NAK = 0x2000U;  ///< STAT_RX = NAK，回复 NAK / Reply NAK
constexpr uint32_t FSDEV_STAT_RX_VALID =
    0x3000U;  ///< STAT_RX = VALID，接收数据 / Accept data

/// 清除本方向 CTR（写 0），另一方向 CTR 与 ERR 写 1 保持；STAT/DTOG 写 0 不翻转。
/// Clears the direction's CTR (write 0) while keeping the other direction's CTR and
/// ERR (write 1); STAT/DTOG are written 0 and stay unchanged.
template <typename HW>
inline void FsDevClearCtrTx(uint8_t num)
{
  constexpr uint32_t SELF = HW::EP_CTR_TX | HW::EP_ERR_TX;
  constexpr uint32_t OTHER = HW::EP_CTR_RX | HW::EP_ERR_RX;
  const uint32_t cur = HW::ReadEp(num);
  HW::WriteEp(num, (cur & HW::EP_KEEP_MASK & ~SELF) | OTHER);
}

/// 清除接收方向 CTR（写 0），发送方向 CTR 与 ERR 写 1 保持；规则同 FsDevClearCtrTx。
/// Clears the receive direction's CTR (write 0) while keeping the transmit direction's
/// CTR and ERR (write 1); same rules as FsDevClearCtrTx.
template <typename HW>
inline void FsDevClearCtrRx(uint8_t num)
{
  constexpr uint32_t SELF = HW::EP_CTR_RX | HW::EP_ERR_RX;
  constexpr uint32_t OTHER = HW::EP_CTR_TX | HW::EP_ERR_TX;
  const uint32_t cur = HW::ReadEp(num);
  HW::WriteEp(num, (cur & HW::EP_KEEP_MASK & ~SELF) | OTHER);
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
    case Endpoint::Type::INTR:
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
  /**
   * @brief 构造端点并登记到所属设备的 IN/OUT 表 / Construct the endpoint and register it
   * in the owning device's IN/OUT table
   * @param device 所属设备 / Owning device
   * @param number 端点号，须小于 HW::ENDPOINT_COUNT / Endpoint number, must be below
   * HW::ENDPOINT_COUNT
   * @param direction 端点方向 / Endpoint direction
   * @param buffer 槽使用的内存 / Memory used by the slots
   * @param pma_addr 该端点缓冲在 PMA 中的字节偏移 / Byte offset of the endpoint's buffer
   * in the PMA
   * @param pma_size 该端点 PMA 缓冲的字节数 / Size of the endpoint's PMA buffer in bytes
   * @param slot_count 槽数量，0 为基类默认值 / Number of slots, 0 selects the base-class
   * default
   */
  FSDevEndpoint(FSDevDevice<HW>& device, EPNumber number, Direction direction,
                RawData buffer, uint16_t pma_addr, size_t pma_size,
                uint8_t slot_count = 0)
      : Endpoint(number, direction, buffer, slot_count),
        device_(device),
        pma_addr_(pma_addr),
        pma_size_(pma_size)
  {
    REQUIRE(EPNumberToInt8(GetNumber()) < HW::ENDPOINT_COUNT);
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
  /// 把本方向 STAT 置为 STALL / Set this direction's STAT to STALL
  ErrorCode StallHardware(bool in_isr) override;
  /// 清 STALL：EP0 无操作（下一个 SETUP 清除），其余端点复位 DATA0 并置 NAK / Clear
  /// STALL: a no-op on EP0 (the next SETUP clears it); other endpoints reset DATA0 and
  /// go to NAK
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
  // IRQ side: one packet per CTR_TX / CTR_RX.
  void OnTxPacket();
  void OnRxPacket(size_t bytes);

  FSDevDevice<HW>& device_;
  uint16_t pma_addr_ = 0;  ///< PMA 存储字节偏移，构造期固定 / PMA byte offset, fixed at
                           ///< construction
  size_t pma_size_ = 0;

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
    RawData buffer;   ///< 槽使用的内存 / Memory used by the slots
    size_t pma_size;  ///< 该端点 PMA 缓冲字节数，须按 HW::PMA_ALIGN 对齐 / PMA buffer
                      ///< size in bytes, must be a multiple of HW::PMA_ALIGN
  };

  /**
   * @brief 构造设备、分配 PMA 并创建全部端点 / Construct the device, lay out the PMA and
   * create every endpoint
   * @param rx_ep_cfgs OUT 端点配置，第 i 项对应 EP i，EP0 必须提供，不超过
   * HW::ENDPOINT_COUNT / OUT endpoint configs, entry i is EP i, EP0 is mandatory, at most
   * HW::ENDPOINT_COUNT
   * @param tx_ep_cfgs IN 端点配置，第 i 项对应 EP i，EP0 必须提供，不超过
   * HW::ENDPOINT_COUNT / IN endpoint configs, entry i is EP i, EP0 is mandatory, at most
   * HW::ENDPOINT_COUNT
   * @param packet_size 端点 0 最大包长 / EP0 max packet size
   * @param vid 厂商 ID / Vendor ID
   * @param pid 产品 ID / Product ID
   * @param bcd 设备版本号（bcdDevice）/ Device release number (bcdDevice)
   * @param lang_list 字符串描述符语言包列表 / String descriptor language packs
   * @param configs 每个 configuration 的 class/描述符项列表 / Class and descriptor items
   * of each configuration
   * @param uid 可选的唯一标识字节，作为序列号字符串的后缀 / Optional unique ID bytes
   * used as the suffix of the serial-number string
   * @param pma_limit PMA 可用字节（构造期断言）；0 = 全部 / Usable PMA bytes
   *                  (asserted at construction); 0 = all of it
   * @param slot_count 每个端点的槽数量，0 为基类默认值 / Number of slots per endpoint, 0
   * selects the base-class default
   */
  FSDevDevice(
      const std::initializer_list<EPConfig>& rx_ep_cfgs,
      const std::initializer_list<EPConfig>& tx_ep_cfgs,
      DeviceDescriptor::PacketSize0 packet_size, uint16_t vid, uint16_t pid, uint16_t bcd,
      const std::initializer_list<const DescriptorStrings::LanguagePack*>& lang_list,
      const std::initializer_list<const std::initializer_list<ConfigDescriptorItem*>>&
          configs,
      ConstRawData uid = {nullptr, 0}, size_t pma_limit = 0, uint8_t slot_count = 0);

  /// 请求初始化设备（转给设备核心）/ Request the initialization of the device (forwarded
  /// to the device core)
  void Init(bool in_isr) override { DeviceCore<FullSpeedCapabilities>::Init(in_isr); }
  /// 请求反初始化设备（转给设备核心）/ Request the deinitialization of the device
  /// (forwarded to the device core)
  void Deinit(bool in_isr) override { DeviceCore<FullSpeedCapabilities>::Deinit(in_isr); }

  ~FSDevDevice() override
  {
    if (self_ == this)
    {
      self_ = nullptr;
    }
  }

  /// 打开 CTR/RESET/SUSP/WKUP 中断，有类需要总线时间时同时打开 SOF 中断，最后接通上拉
  /// Enable the CTR, RESET, SUSP and WKUP interrupts, and the SOF interrupt when a class
  /// wants the bus time, then connect the pull-up last
  void Start(bool in_isr) override
  {
    UNUSED(in_isr);
    {
      EndpointPool::HardwareScope hardware(this);
      connected_ = true;
      uint16_t cntr = HW::ReadCntr();
      cntr |= HW::CNTR_RUN_MASK;
      if (WantsBusTime())
      {
        cntr |= HW::CNTR_SOFM;
      }
      else
      {
        cntr &= static_cast<uint16_t>(~HW::CNTR_SOFM);
      }
      HW::WriteCntr(cntr);
      // 上拉最后接通：前面的初始化不产生主机可见的变化。
      // The pull-up comes on last: the initialization before it changes nothing the host
      // can see.
      HW::Connect(true);
    }
  }

  /// 先断开上拉再关闭上述中断 / Disconnect the pull-up first, then disable those
  /// interrupts
  void Stop(bool in_isr) override
  {
    UNUSED(in_isr);
    {
      EndpointPool::HardwareScope hardware(this);
      connected_ = false;
      HW::Connect(false);
      HW::WriteCntr(
          static_cast<uint16_t>(HW::ReadCntr() & ~(HW::CNTR_RUN_MASK | HW::CNTR_SOFM)));
    }
    // 下拉之后主机不再通信；在途请求的完成事件不会再来，由下一次解绑或复位流程结束。
    // After the pull-up is off the host no longer talks; completions of requests in
    // flight never arrive, and the next unbind or reset ends them.
  }

  /// 仅在状态阶段 IN 完成时写 DADDR（EF | address）/ Write DADDR (EF | address) only
  /// when the status-stage IN has completed
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

  /// 复位已被 owner 应用，不再丢弃端点事件 / The reset has been applied by the owner, so
  /// endpoint events are no longer discarded
  void OnControllerReset() override { discard_ = false; }

  /// 所有端点停止都已确认后，按是否有类需要总线时间重新决定 SOF 中断的开关
  /// Once every endpoint stop is confirmed, decide again whether the SOF interrupt is on,
  /// depending on whether a class wants the bus time
  void OnStopsSettled() override
  {
    if (!connected_)
    {
      // 已 Stop（上拉拉低）之后不会再有 SOF；不重开 SOFM，
      // 避免未连接状态下出现"中断使能开着却没有事务"的困惑。
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
      cntr |= HW::CNTR_SOFM;
    }
    else
    {
      cntr &= static_cast<uint16_t>(~HW::CNTR_SOFM);
    }
    HW::WriteCntr(cntr);
  }

  /// 本控制器只支持全速 / This controller is full-speed only
  Speed ReadBusSpeed() const override { return Speed::FULL; }

  /// 中断入口，由平台的中断处理函数调用 / Interrupt entry, called from the interrupt
  /// handler of the platform
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
  /// 支持的最大端点号数量 / Maximum number of endpoint numbers supported
  static constexpr uint8_t MAX_EPS = 8;

  template <typename U>
  friend class FSDevEndpoint;

  /// 读 ISTR 并按 RESET、CTR、SOF、SUSP、WKUP、噪声的顺序处理 / Read ISTR and handle
  /// RESET, CTR, SOF, SUSP, WKUP and noise in that order
  void HandleInterrupt();
  /// 循环（最多 8 轮）处理 CTR 事件 / Handle CTR events in a loop (at most 8 passes)
  void HandleCtr();
  /// 处理端点 num 的 CTR_TX/CTR_RX；EP0 的 SETUP 交给核心 / Handle CTR_TX/CTR_RX of
  /// endpoint num; a SETUP on EP0 goes to the core
  void HandleEp(uint8_t num);
  /// 总线复位：地址归零、恢复 EP0 接收并通知核心 / Bus reset: clear the address, restore
  /// EP0 reception and notify the core
  void OnBusResetRegs();
  /// 把 FNR 的 11 位帧号交给核心 / Pass the 11-bit frame number from FNR to the core
  void HandleSof();

  /// 按端点号索引的 IN 端点 / IN endpoints by number
  FSDevEndpoint<HW>* in_[MAX_EPS] = {};
  /// 按端点号索引的 OUT 端点 / OUT endpoints by number
  FSDevEndpoint<HW>* out_[MAX_EPS] = {};
  /// 使用的端点号数量（RX/TX 列表较长者）/ Endpoint numbers in use (the longer of the
  /// RX/TX lists)
  uint8_t endpoint_count_ = 1;
  bool connected_ = false;  ///< Start/Stop：设备是否属于总线 / Whether the device
                            ///< belongs on the bus
  bool discard_ = false;    ///< 复位已见、owner 未应用 / Reset seen, not yet applied
  /// SETUP 包暂存 / SETUP packet staging
  alignas(4) uint8_t setup_[8] = {};

  /// 唯一实例，Dispatch 的入口 / The single instance, entry of Dispatch
  static inline FSDevDevice* self_ = nullptr;
};

}  // namespace LibXR::USB

#include "fsdev.tpp"
