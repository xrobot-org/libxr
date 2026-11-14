#pragma once

#include <cstddef>
#include <cstdint>
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
 * `FSDevDevice` 是基类，里面是经典型控制器（16 位寄存器）的完整寄存器模型：寄存器
 * 偏移、各寄存器的位、BTABLE 和 PMA 的访问、复位与挂起/唤醒序列。平台的设备类从它
 * 派生，在构造时给出寄存器块和 PMA 的地址（`FSDevDevice::MemoryMap`），并实现接上拉
 * 的 `Connect()`；时钟、引脚和中断由平台的设备类自己处理。寄存器布局不同的变体
 * （32 位寄存器的 DRD）覆盖 `FSDevDevice` 里对应的虚函数。
 *
 * `FSDevDevice` is a base class that holds the whole register model of the classic
 * controller (16-bit registers): the register offsets, the bits of every register, the
 * access to BTABLE and PMA, and the reset and suspend/resume sequences. The device
 * class of a platform derives from it, gives the addresses of the register block and of
 * the PMA at construction (`FSDevDevice::MemoryMap`) and implements `Connect()` for the
 * pull-up; clocks, pins and interrupts are the business of the platform's device class.
 * A variant with a different register layout (DRD with 32-bit registers) overrides the
 * corresponding virtual functions of `FSDevDevice`.
 */

namespace LibXR::USB
{

/// 寄存器相对寄存器块基址的偏移；端点寄存器 EPnR 在 4n / Offsets of the registers from
/// the base of the register block; the endpoint register EPnR is at 4n
constexpr uint32_t FSDEV_REG_CNTR = 0x40U;
constexpr uint32_t FSDEV_REG_ISTR = 0x44U;    ///< 中断状态 / Interrupt status
constexpr uint32_t FSDEV_REG_FNR = 0x48U;     ///< 帧号 / Frame number
constexpr uint32_t FSDEV_REG_DADDR = 0x4CU;   ///< 设备地址 / Device address
constexpr uint32_t FSDEV_REG_BTABLE = 0x50U;  ///< BTABLE 在 PMA 中的偏移（经典型）/
                                              ///< Offset of the BTABLE in the PMA
                                              ///< (classic controllers)

/// EPnR 的位。CTR 写 0 清除、写 1 保持；STAT 与 DTOG 写 1 翻转；SETUP 只读 / Bits of
/// EPnR. CTR is cleared by writing 0 and kept by writing 1; STAT and DTOG toggle on a
/// written 1; SETUP is read-only
constexpr uint32_t FSDEV_EP_CTR_RX = 0x8000U;
constexpr uint32_t FSDEV_EP_DTOG_RX = 0x4000U;  ///< 接收数据翻转位 / Receive data toggle
constexpr uint32_t FSDEV_EP_STAT_RX = 0x3000U;  ///< 接收状态 / Receive status
constexpr uint32_t FSDEV_EP_SETUP = 0x0800U;  ///< 收到的是 SETUP / The packet was a SETUP
constexpr uint32_t FSDEV_EP_TYPE = 0x0600U;   ///< 端点类型 / Endpoint type
constexpr uint32_t FSDEV_EP_KIND = 0x0100U;   ///< 端点类型的附加位 / Endpoint kind
constexpr uint32_t FSDEV_EP_CTR_TX = 0x0080U;   ///< 发送完成 / Transmission complete
constexpr uint32_t FSDEV_EP_DTOG_TX = 0x0040U;  ///< 发送数据翻转位 / Transmit data toggle
constexpr uint32_t FSDEV_EP_STAT_TX = 0x0030U;  ///< 发送状态 / Transmit status
constexpr uint32_t FSDEV_EP_ADDR = 0x000FU;     ///< 端点地址 / Endpoint address
/// 经典型写 EPnR 时写 1 保持的位：CTR、SETUP、类型、KIND、地址 / Bits a write to EPnR
/// keeps by writing 1 on the classic controllers: CTR, SETUP, type, KIND and address
constexpr uint32_t FSDEV_EP_KEEP_MASK = FSDEV_EP_CTR_RX | FSDEV_EP_SETUP | FSDEV_EP_TYPE |
                                        FSDEV_EP_KIND | FSDEV_EP_CTR_TX | FSDEV_EP_ADDR;

/// ISTR 的标志 / Flags of ISTR
constexpr uint16_t FSDEV_ISTR_CTR = 0x8000U;
constexpr uint16_t FSDEV_ISTR_WKUP = 0x1000U;   ///< 唤醒 / Wake-up
constexpr uint16_t FSDEV_ISTR_SUSP = 0x0800U;   ///< 挂起 / Suspend
constexpr uint16_t FSDEV_ISTR_RESET = 0x0400U;  ///< 总线复位 / Bus reset
constexpr uint16_t FSDEV_ISTR_SOF = 0x0200U;    ///< 帧起始 / Start of frame
/// PMAOVR | ERR | ESOF：运行期只清除，不处理 / PMAOVR | ERR | ESOF: the runtime only
/// clears them
constexpr uint16_t FSDEV_ISTR_NOISE = 0x6100U;

/// CNTR 的位 / Bits of CNTR
constexpr uint16_t FSDEV_CNTR_SOFM = 0x0200U;
/// CTRM | RESETM | SUSPM | WKUPM：设备运行时打开的中断 / CTRM | RESETM | SUSPM | WKUPM:
/// the interrupts enabled while the device runs
constexpr uint16_t FSDEV_CNTR_RUN_MASK = 0x8000U | 0x0400U | 0x0800U | 0x1000U;
constexpr uint16_t FSDEV_CNTR_FSUSP = 0x0008U;   ///< 强制挂起 / Force suspend
constexpr uint16_t FSDEV_CNTR_LPMODE = 0x0004U;  ///< 低功耗模式 / Low-power mode
constexpr uint16_t FSDEV_CNTR_FRES = 0x0001U;    ///< 强制复位 / Force reset

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
class FSDevEndpoint : public Endpoint
{
 public:
  /**
   * @brief 构造端点并登记到所属设备的 IN/OUT 表 / Construct the endpoint and register it
   * in the owning device's IN/OUT table
   * @param device 所属设备 / Owning device
   * @param number 端点号，须小于 FSDevDevice::MAX_EPS / Endpoint number, must be below
   * FSDevDevice::MAX_EPS
   * @param direction 端点方向 / Endpoint direction
   * @param buffer 槽使用的内存 / Memory used by the slots
   * @param pma_addr 该端点缓冲在 PMA 中的字节偏移 / Byte offset of the endpoint's buffer
   * in the PMA
   * @param pma_size 该端点 PMA 缓冲的字节数 / Size of the endpoint's PMA buffer in bytes
   * @param slot_count 槽数量，0 为基类默认值 / Number of slots, 0 selects the base-class
   * default
   */
  FSDevEndpoint(FSDevDevice& device, EPNumber number, Direction direction, RawData buffer,
                uint16_t pma_addr, size_t pma_size, uint8_t slot_count = 0);

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
  friend class FSDevDevice;

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

  FSDevDevice& device_;
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
 * @brief FSDEV 设备：各平台 FSDEV 设备类的基类 / FSDEV device: base class of the FSDEV
 * device classes of the platforms
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
 *
 * 派生类要做的事：构造函数里把控制器的地址交给本类的构造函数，打开控制器的时钟，
 * 安装硬件守卫（`SetHardwareGuard`），然后调用 `InitController()`；实现 `Connect()`；
 * 在控制器的中断里调用 `Dispatch()`。一个程序里只能有一个 FSDEV 设备对象。本类不能
 * 直接使用：构造函数是 protected 的，应用用的是平台的设备类。
 *
 * What a derived class does: its constructor passes the addresses of the controller to
 * the constructor of this class, enables the clock of the controller, installs the
 * hardware guard (`SetHardwareGuard`) and then calls `InitController()`; it implements
 * `Connect()`; it calls `Dispatch()` from the interrupt of the controller. A program
 * can have one FSDEV device object only. This class is not used directly: its
 * constructor is protected, and applications use the device class of the platform.
 */
class FSDevDevice : public EndpointPool, public DeviceCore<FullSpeedCapabilities>
{
 public:
  /// 端点配置：槽内存与 PMA 缓冲字节 / Endpoint configuration: slot memory and PMA
  /// bytes
  struct EPConfig
  {
    RawData buffer;   ///< 槽使用的内存 / Memory used by the slots
    size_t pma_size;  ///< 该端点 PMA 缓冲字节数，须按 MemoryMap::alignment 对齐 / PMA
                      ///< buffer size in bytes, must be a multiple of
                      ///< MemoryMap::alignment
  };

  /// 控制器在这颗芯片上的位置，以及包缓冲区（PMA）的形状 / Where the controller is on
  /// this chip, and the shape of its packet memory (PMA)
  struct MemoryMap
  {
    uintptr_t registers;        ///< 寄存器块的基址，即 EP0R 的地址 / Base address of the
                                ///< register block, which is the address of EP0R
    uintptr_t packet_memory;    ///< PMA 的基址 / Base address of the PMA
    size_t packet_memory_size;  ///< PMA 的字节数 / Size of the PMA in bytes
    /// 经典型 PMA 里每个 16 位数据字占几个 16 位地址：2 表示每个 32 位字只用低 16 位，
    /// 1 表示连续存放 / How many 16-bit addresses one 16-bit data word takes in the PMA
    /// of a classic controller: 2 when only the low half of every 32-bit word is used,
    /// 1 when the words are contiguous
    uint8_t word_stride;
    /// PMA 里缓冲地址与长度的对齐字节数：经典型为 2 / Alignment in bytes of buffer
    /// addresses and sizes in the PMA: 2 on the classic controllers
    uint8_t alignment;
  };

  /// 请求初始化设备（转给设备核心）/ Request the initialization of the device (forwarded
  /// to the device core)
  void Init(bool in_isr) override { DeviceCore<FullSpeedCapabilities>::Init(in_isr); }
  /// 请求反初始化设备（转给设备核心）/ Request the deinitialization of the device
  /// (forwarded to the device core)
  void Deinit(bool in_isr) override { DeviceCore<FullSpeedCapabilities>::Deinit(in_isr); }

  ~FSDevDevice() override;

  /// 打开 CTR/RESET/SUSP/WKUP 和 SOF 中断，最后接通上拉。SOF 中断在设备上总线且没有挂起
  /// 时保持打开：挂起时关，恢复和总线复位时开，Stop() 时关
  /// Enable the CTR, RESET, SUSP, WKUP and SOF interrupts, then connect the pull-up last.
  /// The SOF interrupt stays on while the device is on the bus and not suspended: off at
  /// a suspend, on at a resume and at a bus reset, off in Stop()
  void Start(bool in_isr) override;

  /// 先断开上拉再关闭上述中断 / Disconnect the pull-up first, then disable those
  /// interrupts
  void Stop(bool in_isr) override;

  /// 仅在状态阶段 IN 完成时写 DADDR（EF | address）/ Write DADDR (EF | address) only
  /// when the status-stage IN has completed
  ErrorCode SetAddress(uint8_t address, ControlContext context) override;

  /// 复位已被 owner 应用，不再丢弃端点事件 / The reset has been applied by the owner, so
  /// endpoint events are no longer discarded
  void OnControllerReset() override { discard_ = false; }

  /// 本控制器只支持全速 / This controller is full-speed only
  Speed ReadBusSpeed() const override { return Speed::FULL; }

  /// 中断入口，由平台的中断处理函数调用 / Interrupt entry, called from the interrupt
  /// handler of the platform
  static void Dispatch();

 protected:
  /// 硬件的端点号数量 / Number of endpoint numbers of the hardware
  static constexpr uint8_t MAX_EPS = 8;

  /**
   * @brief 构造设备、分配 PMA 并创建全部端点；不访问硬件 / Construct the device, lay out
   * the PMA and create every endpoint; the hardware is not touched
   * @param memory_map 控制器的地址与 PMA 的形状 / Addresses of the controller and shape
   * of the PMA
   * @param rx_ep_cfgs OUT 端点配置，第 i 项对应 EP i，EP0 必须提供，不超过 MAX_EPS /
   * OUT endpoint configs, entry i is EP i, EP0 is mandatory, at most MAX_EPS
   * @param tx_ep_cfgs IN 端点配置，第 i 项对应 EP i，EP0 必须提供，不超过 MAX_EPS /
   * IN endpoint configs, entry i is EP i, EP0 is mandatory, at most MAX_EPS
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
      const MemoryMap& memory_map, const std::initializer_list<EPConfig>& rx_ep_cfgs,
      const std::initializer_list<EPConfig>& tx_ep_cfgs,
      DeviceDescriptor::PacketSize0 packet_size, uint16_t vid, uint16_t pid, uint16_t bcd,
      const std::initializer_list<const DescriptorStrings::LanguagePack*>& lang_list,
      const std::initializer_list<const std::initializer_list<ConfigDescriptorItem*>>&
          configs,
      ConstRawData uid = {nullptr, 0}, size_t pma_limit = 0, uint8_t slot_count = 0);

  /**
   * @brief 把控制器初始化到“已复位、中断关闭、未接上拉”的状态 / Bring the controller to
   * the state "reset, interrupts off, pull-up not connected"
   *
   * 派生类在自己的构造函数里、打开控制器时钟之后调用一次：复位控制器（`InitRegs`），
   * 把各端点的 PMA 缓冲地址写进 BTABLE。之后 `Start()` 才打开中断并接上拉。
   *
   * A derived class calls it once in its own constructor, after it has enabled the
   * clock of the controller: it resets the controller (`InitRegs`) and writes the PMA
   * buffer address of every endpoint into the BTABLE. `Start()` then enables the
   * interrupts and connects the pull-up.
   */
  void InitController();

  /// 接通或断开 D+ 上拉，由平台实现 / Connect or disconnect the D+ pull-up; implemented
  /// by the platform
  virtual void Connect(bool on) = 0;

  /// 读偏移 offset 处的寄存器。经典型是 16 位访问；寄存器宽度不同的变体覆盖它 / Read the
  /// register at the offset. A 16-bit access on the classic controllers; a variant with
  /// another register width overrides it
  virtual uint32_t ReadRegister(uint32_t offset);
  /// 写偏移 offset 处的寄存器，其余同 ReadRegister / Write the register at the offset;
  /// otherwise as ReadRegister
  virtual void WriteRegister(uint32_t offset, uint32_t value);

  /// 把端点 num 发送缓冲在 PMA 中的字节偏移写进 BTABLE / Write the byte offset of the
  /// transmit buffer of endpoint num in the PMA into the BTABLE
  virtual void SetTxBuf(uint8_t num, uint16_t addr);
  /// 写端点 num 下一个 IN 包的字节数 / Write the byte count of the next IN packet of
  /// endpoint num
  virtual void SetTxCount(uint8_t num, uint16_t bytes);
  /// 把端点 num 接收缓冲在 PMA 中的字节偏移写进 BTABLE / Write the byte offset of the
  /// receive buffer of endpoint num in the PMA into the BTABLE
  virtual void SetRxBuf(uint8_t num, uint16_t addr);
  /// 按最大包长 mps 写端点 num 接收区的大小（BTABLE 的块编码）/ Write the size of the
  /// receive area of endpoint num for the maximum packet size mps (block encoding of
  /// the BTABLE)
  virtual void SetRxCountEncoded(uint8_t num, uint16_t mps);
  /// 端点 num 收到的包的字节数 / Byte count of the packet endpoint num received
  virtual uint16_t GetRxCount(uint8_t num);
  /// 把 len 字节从 src 拷进 PMA 的字节偏移 addr 处 / Copy len bytes from src into the PMA
  /// at byte offset addr
  virtual void PmaWrite(uint16_t addr, const uint8_t* src, size_t len);
  /// 把 PMA 字节偏移 addr 处的 len 字节拷到 dst / Copy len bytes from the PMA at byte
  /// offset addr to dst
  virtual void PmaRead(uint8_t* dst, uint16_t addr, size_t len);

  /// 复位控制器并清除全部中断标志，由 InitController() 调用 / Reset the controller and
  /// clear every interrupt flag; called by InitController()
  virtual void InitRegs();
  /// 总线挂起时让控制器进入挂起 / Put the controller into suspend when the bus is
  /// suspended
  virtual void SuspendRegs();
  /// 挂起标志清除之后的动作，经典型没有 / Action after the suspend flag has been
  /// cleared; none on the classic controllers
  virtual void SuspendClocked();
  /// 总线唤醒时让控制器离开挂起 / Take the controller out of suspend when the bus wakes
  /// up
  virtual void ResumeRegs();

  /// 读端点寄存器 EPnR / Read the endpoint register EPnR
  uint32_t ReadEp(uint8_t num) { return ReadRegister(static_cast<uint32_t>(num) * 4U); }
  /// 写端点寄存器 EPnR / Write the endpoint register EPnR
  void WriteEp(uint8_t num, uint32_t value)
  {
    WriteRegister(static_cast<uint32_t>(num) * 4U, value);
  }
  /// 读 CNTR 的低 16 位 / Read the low 16 bits of CNTR
  uint16_t ReadCntr() { return static_cast<uint16_t>(ReadRegister(FSDEV_REG_CNTR)); }
  /// 写 CNTR，高位写 0 / Write CNTR; the high bits are written 0
  void WriteCntr(uint16_t value) { WriteRegister(FSDEV_REG_CNTR, value); }
  /// 读 ISTR 的低 16 位 / Read the low 16 bits of ISTR
  uint16_t ReadIstr() { return static_cast<uint16_t>(ReadRegister(FSDEV_REG_ISTR)); }
  /// 清除给出的 ISTR 标志：这些位写 0，其余位（含高位）写 1 保持 / Clear the given flags
  /// of ISTR: these bits are written 0, every other bit, the high ones included, is
  /// written 1 and keeps
  void ClearIstr(uint16_t flags)
  {
    WriteRegister(FSDEV_REG_ISTR, ~static_cast<uint32_t>(flags));
  }
  /// 读 FNR 的低 16 位 / Read the low 16 bits of FNR
  uint16_t ReadFnr() { return static_cast<uint16_t>(ReadRegister(FSDEV_REG_FNR)); }
  /// 写 DADDR / Write DADDR
  void WriteDaddr(uint16_t value) { WriteRegister(FSDEV_REG_DADDR, value); }

  /// 清除发送方向 CTR（写 0），接收方向 CTR 与 ERR 写 1 保持；STAT/DTOG 写 0 不翻转 /
  /// Clear the transmit direction's CTR (write 0) while keeping the receive direction's
  /// CTR and ERR (write 1); STAT/DTOG are written 0 and stay unchanged
  void ClearCtrTx(uint8_t num);
  /// 清除接收方向 CTR，规则同 ClearCtrTx / Clear the receive direction's CTR; same
  /// rules as ClearCtrTx
  void ClearCtrRx(uint8_t num);

  /// 控制器的地址与 PMA 的形状 / Addresses of the controller and shape of the PMA
  const MemoryMap memory_map_;
  /// EPnR 里接收、发送方向的错误位；经典型没有，为 0 / Error bits of the receive and the
  /// transmit direction in EPnR; the classic controllers have none, so 0
  uint32_t ep_err_rx_ = 0U;
  uint32_t ep_err_tx_ = 0U;  ///< 见 ep_err_rx_ / See ep_err_rx_
  /// 写 EPnR 时写 1 保持的全部位；有更多这类位的变体在调用 InitController() 之前改写 /
  /// Every bit a write to EPnR keeps by writing 1; a variant with more such bits sets it
  /// before it calls InitController()
  uint32_t ep_keep_mask_ = FSDEV_EP_KEEP_MASK;

 private:
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
  /// 经典型 BTABLE 里端点 num 的第 field 个 16 位字段（0 = ADDR_TX，1 = COUNT_TX，
  /// 2 = ADDR_RX，3 = COUNT_RX）/ Field number field of endpoint num in the BTABLE of a
  /// classic controller (0 = ADDR_TX, 1 = COUNT_TX, 2 = ADDR_RX, 3 = COUNT_RX)
  volatile uint16_t* BtField(uint8_t num, unsigned field) const;

  /// 按端点号索引的 IN 端点 / IN endpoints by number
  FSDevEndpoint* in_[MAX_EPS] = {};
  /// 按端点号索引的 OUT 端点 / OUT endpoints by number
  FSDevEndpoint* out_[MAX_EPS] = {};
  /// 使用的端点号数量（RX/TX 列表较长者）/ Endpoint numbers in use (the longer of the
  /// RX/TX lists)
  uint8_t endpoint_count_ = 1;
  bool discard_ = false;  ///< 复位已见、owner 未应用 / Reset seen, not yet applied
  /// InitController() 已经调用 / InitController() has been called
  bool controller_ready_ = false;
  /// SETUP 包暂存 / SETUP packet staging
  alignas(4) uint8_t setup_[8] = {};

  /// 唯一实例，Dispatch 的入口 / The single instance, entry of Dispatch
  static inline FSDevDevice* self_ = nullptr;
};

}  // namespace LibXR::USB
