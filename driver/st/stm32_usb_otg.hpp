#pragma once

#include "main.h"

#undef USB

#include "usb/core/ep.hpp"
#include "usb/core/ep_pool.hpp"
#include "usb/core/usb_detach_timer.hpp"
#include "usb/core/usb_service.hpp"
#include "usb/device/dev_core.hpp"

#if defined(HAL_PCD_MODULE_ENABLED) && (defined(USB_OTG_FS) || defined(USB_OTG_HS) || \
                                        defined(USB1_OTG_HS) || defined(USB2_OTG_HS))

namespace LibXR
{

class STM32OtgEndpoint;

/**
 * @class STM32OtgController
 * @brief STM32 OTG（DWC2）控制器：寄存器、FIFO 和全部端点对象 / STM32 OTG (DWC2)
 *        controller: registers, FIFOs and every endpoint object
 *
 * 设备核心在构造时就要用到 EP0，所以端点必须在它之前建好；本类作为设备的第一个基类，
 * 负责 core 初始化和建端点。不使用 PCD 句柄：构造时用临时句柄调用 CubeMX 生成的
 * HAL_PCD_MspInit()（时钟、引脚、NVIC），再用 LL 函数初始化 core。CubeMX 保持默认
 * 设置：链接时包装 HAL_PCD_Init() 和 HAL_PCD_IRQHandler()（stm32_usb_wrap.cpp）。
 * The device core needs EP0 at construction, so the endpoints are built before it; this
 * class, the device's first base, initializes the core and builds the endpoints. No PCD
 * handle is used: the constructor calls the CubeMX-generated HAL_PCD_MspInit() (clocks,
 * pins, NVIC) through a temporary handle and initializes the core with the LL functions.
 * CubeMX keeps its default settings: the link wraps HAL_PCD_Init() and
 * HAL_PCD_IRQHandler() (stm32_usb_wrap.cpp).
 *
 * @note 构造之后寄存器只在设备的 owner 中访问（STM32USBOtgDevice 的 Start、Stop 例外，
 *       见其说明）。
 *       After construction the registers are touched only in the device's owner (except
 *       Start and Stop of STM32USBOtgDevice, see there).
 */
class STM32OtgController : public USB::EndpointPool
{
 public:
  /// PHY 接口 / PHY interface
  enum class Phy : uint8_t
  {
    EMBEDDED_FS,  ///< 片内全速 PHY / Embedded full-speed PHY
    ULPI,         ///< 外部 ULPI 高速 PHY / External ULPI high-speed PHY
    EMBEDDED_HS   ///< 片内高速 PHY（F723/F733 等）/ Embedded high-speed PHY (F723/F733)
  };

  /// 控制器配置 / Controller configuration
  struct HardwareConfig
  {
    Phy phy = Phy::EMBEDDED_FS;  ///< PHY 接口 / PHY interface
    bool dma = false;            ///< 使用 core 内部 DMA / Use the core's internal DMA
    bool vbus_sensing = false;   ///< VBUS 检测 / VBUS sensing
    bool high_speed = false;     ///< 以高速枚举（需高速 PHY）/ Enumerate at high speed
    /// DMA 时 EP0 OUT 的缓冲：≥ 64 B，32 字节对齐，DMA 可达 / EP0 OUT buffer with DMA:
    /// ≥ 64 B, 32-byte aligned, reachable by DMA
    RawData ep0_dma_buffer{nullptr, 0};
  };

  /// IN 端点：槽内存与 TX FIFO 字节数 / IN endpoint: slot memory and TX FIFO bytes
  struct EPInConfig
  {
    RawData buffer;  ///< 该端点的槽内存 / Slot memory of the endpoint
    /// TX FIFO 字节数，至少 8 且为 4 的倍数 / TX FIFO bytes, at least 8, a multiple of 4
    size_t fifo_size;
  };

 protected:
  /**
   * @brief 初始化 core、建全部端点并划分 FIFO / Initialize the core, build every
   *        endpoint and lay out the FIFOs
   * @param core OTG core 寄存器 / OTG core registers
   * @param irqn 本控制器的中断线 / This controller's interrupt line
   * @param endpoint_count core 的端点数（不超过 16）/ Endpoint count (at most 16)
   * @param total_fifo core 的 FIFO RAM 字节数 / FIFO RAM of the core in bytes
   * @param hw 控制器配置 / Controller configuration
   * @param rx_fifo_size 共享 RX FIFO 字节数，4 的倍数且不小于 64 × OUT 端点数 / Shared
   *        RX FIFO bytes, a multiple of 4 and at least 64 × the OUT endpoint count
   * @param rx_ep_cfgs 各 OUT 端点的槽内存，第一项属于 EP0 / Slot memory of each OUT
   *        endpoint, the first one EP0's
   * @param tx_ep_cfgs 各 IN 端点的配置，第一项属于 EP0；RX 与全部 TX FIFO 之和不得超过
   *        total_fifo / Each IN endpoint, the first one EP0's; RX and all TX FIFOs
   *        together fit in total_fifo
   * @note 调用者：STM32USBOtgDevice 的构造，在控制器中断使能之前。
   *       Caller: the constructor of STM32USBOtgDevice, before the controller
   *       interrupt is enabled.
   */
  STM32OtgController(USB_OTG_GlobalTypeDef* core, IRQn_Type irqn, uint8_t endpoint_count,
                     size_t total_fifo, const HardwareConfig& hw, size_t rx_fifo_size,
                     const std::initializer_list<RawData>& rx_ep_cfgs,
                     const std::initializer_list<EPInConfig>& tx_ep_cfgs);

  /// @name 寄存器块（仅 owner）/ Register blocks (owner only)
  /// @{

  /// 设备模式寄存器块（DCFG、DCTL、DSTS 等）/ Device-mode register block (DCFG, DCTL,
  /// DSTS and so on)
  USB_OTG_DeviceTypeDef* Device() const;
  /// IN 端点 num 的寄存器块 / Register block of IN endpoint num
  USB_OTG_INEndpointTypeDef* InEp(uint8_t num) const;
  /// OUT 端点 num 的寄存器块 / Register block of OUT endpoint num
  USB_OTG_OUTEndpointTypeDef* OutEp(uint8_t num) const;
  /// 端点 num 的 FIFO 窗口；写入填 TX FIFO，读 Fifo(0) 取 RX 数据 / FIFO window of
  /// endpoint num; writes fill the TX FIFO, reads of Fifo(0) take RX data
  volatile uint32_t& Fifo(uint8_t num) const;
  /// @}

  /**
   * @brief 让 EP0 OUT 能收 SETUP / Make EP0 OUT accept SETUPs
   * @note 调用者：复位、OnControlReady、EP0 的 STALL 和完成（owner）。
   *       Callers: reset, OnControlReady, EP0 stall and completion (owner).
   */
  void ArmSetup();

  /**
   * @brief 弹出 RX FIFO 状态项并把数据搬进各 OUT 端点 / Pop RX FIFO status entries and
   *        move the data into the OUT endpoints
   * @note 只在 FIFO 模式、owner 中调用：中断处理（RXFLVL）和 OUT 停止的等待循环。每次最多
   *       16 项；SETUP 拷进 rx_setup_，不交出；不报告完成。
   *       FIFO mode only, in the owner: interrupt handling (RXFLVL) and the wait loops of
   *       an OUT stop. At most 16 entries per call; a SETUP is copied into rx_setup_ and
   *       not handed over; no completion is reported.
   */
  void DrainRxFifo();

  /**
   * @brief 请求全局 OUT NAK / Request the global OUT NAK
   * @note 调用者：第一个需要它的 OUT 停止（已有停止占着时共用，不再调用）和
   *       HaltController。GONAKEFF 清零之后才允许写 SGONAK（RM0468 p.2717）：仍置位时先撤
   *       销一次并等它清零，只取决于芯片，以 HANDSHAKE_SPINS 次读为界。
   *       Callers: the first OUT stop that needs it (a stop that finds another one
   *       holding it shares it and does not call this) and HaltController. SGONAK may be
   *       written only once GONAKEFF is clear (RM0468 p.2717): while it is still set the
   *       NAK is withdrawn once more and the flag awaited, which depends on the chip
   *       alone and is bounded by HANDSHAKE_SPINS reads.
   */
  void RequestGlobalOutNak();

  /// 清中断状态、打开中断、接上总线；调用者：Start、ResetController / Clear status,
  /// enable interrupts and attach; callers: Start, ResetController
  void Connect();

  USB_OTG_GlobalTypeDef* const core_;  ///< core 寄存器；构造时写 / Core registers
  const IRQn_Type irqn_;               ///< 中断线；Mask/Unmask/Pend 读 / Interrupt line
  const uint8_t endpoint_count_;       ///< core 的端点数 / Endpoint count of the core
  const HardwareConfig hw_;            ///< 控制器配置 / Controller configuration
  /// GSNPSID，core 复位后读一次 / GSNPSID, read once after the core reset
  uint32_t core_id_ = 0U;
  /// RX FIFO 扣除固定开销后一个 OUT 包能用的字节；构造时写 / Bytes one OUT packet can
  /// use in the RX FIFO after its fixed overhead; written at construction
  size_t rx_packet_limit_ = 0U;
  /// IN 端点对象，下标为端点号；构造时写，之后只读 / IN endpoint objects by number;
  /// written at construction, read-only afterwards
  STM32OtgEndpoint* in_[16] = {};
  /// OUT 端点对象，下标为端点号；构造时写，之后只读 / OUT endpoint objects by number;
  /// written at construction, read-only afterwards
  STM32OtgEndpoint* out_[16] = {};
  /// FIFO 模式下最近一个 SETUP；DrainRxFifo 写，SETUP 交付读 / FIFO mode: the last
  /// SETUP; written by DrainRxFifo, read when the SETUP is handed over
  alignas(4) uint8_t rx_setup_[8] = {};
  /**
   * @brief 已看到一个 SETUP（FIFO 模式弹出了它，或读到 STPKTRX）、还没有交付 / A SETUP
   *        was seen (popped in FIFO mode, or STPKTRX was read) and not handed over yet
   * @note 交付时清零。
   *       Cleared at the hand-over.
   */
  bool setup_pending_ = false;
  /// 停止未结束的 IN 端点数；仅 owner / IN endpoints whose stop has not ended; owner
  /// only
  uint8_t in_stops_ = 0U;
  /// 停止未结束的 OUT 端点数；仅 owner / OUT endpoints whose stop has not ended; owner
  /// only
  uint8_t out_stops_ = 0U;

  /**
   * @brief 硬件已收到一个还没有交付的 SETUP / The hardware holds a SETUP not handed
   *        over yet
   * @note 调用者：EP0 的 StartHardware 装入之后（owner）。依据：DOEPINT0 的 STUP 或
   *       STPKTRX；FIFO 模式另看 setup_pending_ 和 RX FIFO 队首的 SETUP 状态项。
   *       Caller: EP0's StartHardware after the load (owner). Evidence: STUP or STPKTRX
   *       in DOEPINT0; in FIFO mode also setup_pending_ and a SETUP entry at the head of
   *       the RX FIFO.
   */
  bool SetupLatched() const;

 private:
  friend class STM32OtgEndpoint;

  /// 按 in_ 的顺序在 RX FIFO 之后排各 TX FIFO / Place the TX FIFOs after the RX FIFO
  void ProgramFifos(size_t rx_fifo_size);
};

/**
 * @class STM32USBOtgDevice
 * @brief STM32 OTG（DWC2）USB 设备 / STM32 OTG (DWC2) USB device
 *
 * 实现设备服务的四个入口：ServiceHardware、MaskInterrupt、UnmaskInterrupt、
 * PendInterrupt。PendInterrupt 置本控制器的 NVIC 挂起位，所以 owner 总在 OTG 中断里
 * 运行；屏蔽只用 NVIC 的本线，不关全部中断。端点停止是同步的，见 STM32OtgEndpoint。
 * Implements the four entries of the device service: ServiceHardware, MaskInterrupt,
 * UnmaskInterrupt, PendInterrupt. PendInterrupt sets this controller's NVIC pending bit,
 * so the owner always runs in the OTG interrupt; masking uses only this line at the
 * NVIC and never disables all interrupts. Endpoint stops are synchronous, see
 * STM32OtgEndpoint.
 */
class STM32USBOtgDevice : public STM32OtgController,
                          public USB::DeviceService,
                          public USB::DeviceCore
{
 public:
  /**
   * @brief 中断入口 / Interrupt entry
   * @note 调用者：libxr_stm32_otg_dispatch（OTG 中断里）。设备已构造时进入
   *       OnControllerInterrupt，否则关 GINT。
   *       Caller: libxr_stm32_otg_dispatch (in the OTG interrupt). Enters
   *       OnControllerInterrupt once the device is constructed, otherwise clears GINT.
   */
  static void Dispatch(USB_OTG_GlobalTypeDef* core);

  /**
   * @brief 打开中断并接上总线 / Enable interrupts and attach to the bus
   * @note 调用者：应用，线程中。在 owner 之外写 GINTSTS、GINTMSK、GAHBCFG、DCTL，期间屏蔽
   *       本中断线；不与持有 owner 的线程互斥（没有 PendInterrupt 时）。
   *       Caller: the application, in a thread. Writes GINTSTS, GINTMSK, GAHBCFG and
   *       DCTL outside the owner with this line masked; not exclusive against a thread
   *       that owns (without PendInterrupt).
   */
  void StartController(bool in_isr) override;

  /**
   * @brief 断开总线并让控制器静止 / Detach from the bus and make the controller
   *        quiescent
   * @note 仅 owner，处理 Stop 的请求时调用，见 DeviceCore::StopController。与
   *       ResetController 的前半相同，只是不再接上总线，并关掉控制器的中断源。
   *       Owner only, called when the Stop request is served, see
   *       DeviceCore::StopController. The same as the first half of ResetController,
   *       except that the bus is not attached again and the controller's interrupt
   *       sources are switched off.
   */
  void StopController(bool in_isr) override;

 protected:
  /**
   * @brief 构造 / Construct
   * @param index core 编号：0 为 FS，1 为 HS / Core index: 0 FS, 1 HS
   * @param core OTG core 寄存器 / OTG core registers
   * @param irqn 本控制器的中断线 / This controller's interrupt line
   * @param endpoint_count core 的端点数（不超过 16）/ Endpoint count (at most 16)
   * @param total_fifo core 的 FIFO RAM 字节数 / FIFO RAM of the core in bytes
   * @param hw 控制器配置；hw.high_speed 决定最高速度 / Controller configuration;
   *        hw.high_speed sets the highest speed
   * @param rx_fifo_size 共享 RX FIFO 字节数，4 的倍数且不小于 64 × OUT 端点数 / Shared
   *        RX FIFO bytes, a multiple of 4 and at least 64 × the OUT endpoint count
   * @param rx_ep_cfgs 各 OUT 端点的槽内存，第一项属于 EP0 / Slot memory of each OUT
   *        endpoint, the first one EP0's
   * @param tx_ep_cfgs 各 IN 端点的配置，第一项属于 EP0；RX 与全部 TX FIFO 之和不得超过
   *        total_fifo / Each IN endpoint, the first one EP0's; RX and all TX FIFOs
   *        together fit in total_fifo
   * @param packet_size EP0 包长 / EP0 packet size
   * @param vid 厂商 ID / Vendor ID
   * @param pid 产品 ID / Product ID
   * @param bcd bcdDevice
   * @param lang_list 字符串语言包 / String language packs
   * @param configs 每个配置的类列表 / Class list of each configuration
   * @param uid 序列号字符串后缀，可为空 / Serial-number suffix, may be empty
   * @note 登记全部端点，并在这里（而不是 Start）把自己写进 devices_，Start 之前挂起的
   *       中断也能找到设备。
   *       Registers every endpoint and enters itself into devices_ here rather than in
   *       Start, so an interrupt pended before Start finds the device.
   */
  STM32USBOtgDevice(
      uint8_t index, USB_OTG_GlobalTypeDef* core, IRQn_Type irqn, uint8_t endpoint_count,
      size_t total_fifo, const HardwareConfig& hw, size_t rx_fifo_size,
      const std::initializer_list<RawData>& rx_ep_cfgs,
      const std::initializer_list<EPInConfig>& tx_ep_cfgs,
      USB::DeviceDescriptor::PacketSize0 packet_size, uint16_t vid, uint16_t pid,
      uint16_t bcd,
      const std::initializer_list<const USB::DescriptorStrings::LanguagePack*>& lang_list,
      const std::initializer_list<const std::initializer_list<USB::DeviceClass*>>&
          configs,
      ConstRawData uid);

  /// @name 设备服务的入口 / Entries of the device service
  /// @{
  /// 按固定顺序处理一次状态快照，见 cpp / Handle one status snapshot in a fixed order,
  /// see the cpp
  void ServiceHardware(bool in_isr) override;
  /// NVIC_DisableIRQ（带 DSB/ISB）/ NVIC_DisableIRQ (with DSB/ISB)
  void MaskInterrupt() override;
  /// NVIC_EnableIRQ
  void UnmaskInterrupt() override;
  /// NVIC_SetPendingIRQ，返回 true / NVIC_SetPendingIRQ, returns true
  bool PendInterrupt() override;
  /// @}

  /// @name 设备核心的后端钩子（owner）/ Backend hooks of the device core (owner)
  /// @{
  /// core 在状态阶段之前采用新地址：只在 BEFORE_STATUS 写 DCFG.DAD / The core takes the
  /// address before the status stage: DCFG.DAD is written at BEFORE_STATUS only
  void SetAddress(uint8_t address, AddressStage stage) override;
  /// ArmSetup
  void OnControlReady() override;
  /**
   * @brief 断开、禁用全部端点、清空 FIFO，再接上总线 / Detach, disable every
   *        endpoint, flush the FIFOs and attach again
   * @note 返回后端点寄存器的 EPENA 和 XFRC 都为 0，所以之后每个 StopHardware 立即返回
   *       ABORTED。只做前半段（HaltController、detach_.OnHalted(true)），不等待；断开
   *       保持 RESET_DETACH_MS 由 detach_ 按节拍计时，到点后 owner 接上（Connect），主机
   *       才会复位总线。
   *       Afterwards EPENA and XFRC of every endpoint are 0, so every later StopHardware
   *       returns ABORTED at once. First half only (HaltController,
   *       detach_.OnHalted(true)), no wait; the detach lasts RESET_DETACH_MS, counted on
   *       the tick by detach_, and the owner attaches once it has passed (Connect), so
   *       that the host resets the bus.
   */
  void ResetController(bool in_isr) override;
  /// @}

 private:
  static constexpr uint8_t MAX_DEVICES = 2;
  /**
   * @brief 断开总线，让硬件不再为任何端点工作，作废未结束的停止 / Detach, make the
   *        hardware stop working for every endpoint and void the pending stops
   * @note StopController 和 ResetController 共用。
   *       Shared by StopController and ResetController.
   */
  void HaltController();
  /// 最短断开时长（毫秒，节拍计数）/ Minimum detach time (ms, counted in ticks)
  static constexpr uint32_t RESET_DETACH_MS = 10U;
  /**
   * @brief 最短断开时间 / Minimum detach time
   * @note HaltController 之后调用 OnHalted：ResetController 传 true，StopController 传
   *       false，都立即返回。StartController 用 WantAttach：true 时就地接上，false 时只
   *       使能中断线。ServiceHardware 开头 TakeAttach 为 true 时做后半段（接上）。
   *       OnHalted after HaltController: true from ResetController, false from
   *       StopController; both return at once. StartController uses WantAttach: on true
   *       it attaches in place, on false it only enables the interrupt line. At the start
   *       of ServiceHardware, when TakeAttach is true, the second half (the attach)
   *       runs.
   * @note HaltController 里的 Spin 循环（等 GINTSTS.GINAKEFF / BOUTNAKEFF、EPDISD、
   *       FIFO 刷新、core 软复位）是芯片定上限的握手，不属于断开等待。
   *       The Spin loops in HaltController (GINAKEFF / BOUTNAKEFF, EPDISD, FIFO flush,
   *       core soft reset) are handshakes bounded by the chip, not the detach wait.
   */
  USB::DetachTimer detach_{*this, RESET_DETACH_MS};
  /// DSTS.FNSOF 是 14 位，用作“没有”的值 / DSTS.FNSOF has 14 bits; the "none" value
  static constexpr uint32_t NO_SOF_POSITION = 0xFFFFFFFFU;

  void OnUsbReset(bool in_isr);
  void OnEnumerationDone(bool in_isr);
  void HandleInEndpoints(bool in_isr);
  void HandleOutEndpoints(bool in_isr);
  void HandleSetup(bool in_isr, USB_OTG_OUTEndpointTypeDef* out0, uint32_t events);
  void HandleIsoIncomplete(bool in_isr);
  void HandleSof(bool in_isr);
  /// 全局 OUT NAK 已生效：结束各 OUT 端点未结束的停止 / The global OUT NAK took effect:
  /// end the pending stops of the OUT endpoints
  void HandleGlobalOutNak(bool in_isr);
  /**
   * @brief 当场结束所有未结束的停止 / End every pending stop now
   * @note 调用者：总线复位、挂起、断开的处理，在报告该事件之前。此时总线上没有包，握手
   *       由芯片定上限；握手仍不成功的报 TIMEOUT。
   *       Callers: the handling of a bus reset, suspend and disconnect, before the event
   *       is reported. No packet is on the bus then, so the handshakes are bounded by the
   *       chip; one that still fails is reported as TIMEOUT.
   */
  void FinishPendingStops(bool in_isr);
  /**
   * @brief 每个 SOF 给未结束的停止计数，满 STOP_TIMEOUT_SOFS 再查一次，仍未停稳报 TIMEOUT
   *        / Count every SOF for the pending stops; at STOP_TIMEOUT_SOFS check once more
   *        and report TIMEOUT if still not quiescent
   */
  void CountStopTimeouts(bool in_isr);

  /// 本设备在 devices_ 中的位置 / Index of this device in devices_
  const uint8_t index_;
  /**
   * @brief 最近处理的 SOF 的 DSTS.FNSOF / DSTS.FNSOF of the SOF handled last
   * @note 写：HandleSof、复位、Connect；读：HandleSof。含义：同一个 SOF 令牌 core 可能约
   *       1.4 µs 后再置一次 SOF（STM32H723 实测），帧号相同的第二次被忽略。
   *       Written by HandleSof, the reset and Connect; read by HandleSof. Meaning: the
   *       core may flag one SOF token again about 1.4 µs later (measured on an
   *       STM32H723); the second one with the same number is ignored.
   */
  uint32_t last_sof_position_ = NO_SOF_POSITION;

  /// 中断号到设备；构造时写，中断入口读 / Devices by core; written at construction,
  /// read by the interrupt entry
  static inline STM32USBOtgDevice* devices_[MAX_DEVICES] = {};
};

/**
 * @class STM32OtgEndpoint
 * @brief STM32 OTG 端点，停止不等总线 / STM32 OTG endpoint whose stop never waits for
 *        the bus
 *
 * FIFO 模式：IN 由 CPU 写 TX FIFO（启动时写放得下的整包，其余在 TXFE 中断里写），OUT
 * 由 RXFLVL 读进槽。DMA 模式：core 直接读写槽；EP0 OUT 例外，总是收进设备的 EP0 缓冲再
 * 拷进槽。
 * FIFO mode: the CPU writes IN data to the TX FIFO (the whole packets that fit at start,
 * the rest on TXFE) and reads OUT data into the slot on RXFLVL. DMA mode: the core reads
 * and writes the slot itself, except EP0 OUT, which always receives into the device's
 * EP0 buffer and is copied into the slot.
 *
 * 停止按 RM0468 p.2807（IN）和 p.2799-2800（OUT）的步骤，但不等“NAK 生效”：NAK 已生效
 * 就当场禁用端点并返回结果，否则写下 NAK 请求返回 PENDING，由 INEPNE（IN）或全局 OUT
 * NAK 生效（OUT）的中断结束；请求自己的 XFRC 同样结束它。禁用、刷 FIFO 这些与总线无关
 * 的握手在调用内做，以 HANDSHAKE_SPINS 次寄存器读为界（STM32H723 实测：EPDIS 到
 * EPDISD 不超过 0.33 µs，刷 TX FIFO 不超过 0.45 µs）。
 * A stop follows RM0468 p.2807 (IN) and p.2799-2800 (OUT) without waiting for the NAK
 * to take effect: with the NAK in effect the endpoint is disabled at once and the
 * outcome returned; otherwise the NAK request is written, PENDING is returned, and the
 * INEPNE interrupt (IN) or the global OUT NAK taking effect (OUT) ends the stop, as does
 * the request's own XFRC. The handshakes that do not depend on the bus (disable, FIFO
 * flush) run inside the call, bounded by HANDSHAKE_SPINS register reads (measured on an
 * STM32H723: EPDIS to EPDISD at most 0.33 µs, TX FIFO flush at most 0.45 µs).
 *
 * @note 停止，对应 Endpoint::StopHardware 的规则。
 *       停稳的证据：硬件的停稳事件（IN 是 INEPNE，OUT 是全局 OUT NAK 生效）和请求自己的
 *       XFRC。协议上界用帧起始计数：满 STOP_TIMEOUT_SOFS 个仍未确认才报 TIMEOUT。
 *       缺陷一，停稳的事件会丢：全局 OUT NAK 的“已生效”偶尔不来（STM32H723，FIFO 模式
 *       134 219 次请求里 4 次，DMA 模式也见到）。请求过了一帧仍未生效就撤销后重新
 *       请求；实测 FIFO 模式 8 次、DMA 模式 3 次，全部恢复。
 *       缺陷二，停止的手段影响别的端点：全局 OUT NAK 期间所有 OUT 端点回 NAK，主机会
 *       重发，不丢数据；但控制器丢掉这期间别的等时 OUT 端点的包（RM0468 p.2798），
 *       反复停止批量 OUT 的压力下实测丢 1.2%-3.6%。IN 的停止只影响本端点。
 *       另外两类在实测中没有出现。
 *       Stops, against the rules of Endpoint::StopHardware.
 *       Evidence of quiescence: the hardware's quiescence events (INEPNE for IN, global
 *       OUT NAK effective for OUT) and the request's own XFRC. The protocol bound is
 *       counted in starts of frame: TIMEOUT only when STOP_TIMEOUT_SOFS of them passed
 *       without confirmation.
 *       Flaw one, the quiescence event can be lost: the "effective" of global OUT NAK
 *       sometimes never comes (STM32H723: 4 of 134,219 requests in FIFO mode, seen in
 *       DMA mode as well). A request still not effective one frame later is withdrawn
 *       and made again; measured 8 times in FIFO mode and 3 times in DMA mode, all
 *       recovered.
 *       Flaw two, the means of stopping affects other endpoints: during global OUT NAK
 *       every OUT endpoint answers NAK, which the host retries without loss; but the
 *       controller drops the packets of other isochronous OUT endpoints meanwhile
 *       (RM0468 p.2798), measured as 1.2-3.6 % under repeated stops of a bulk OUT. A
 *       stop of an IN affects that endpoint only.
 *       The other two kinds did not show in the measurements.
 */
class STM32OtgEndpoint : public USB::Endpoint
{
 public:
  /// 与总线无关的寄存器握手最多读这么多次 / Register reads a bus-independent handshake
  /// may take at most
  static constexpr uint32_t HANDSHAKE_SPINS = 256U;
  /// 未结束的停止经过这么多个 SOF 仍未停稳就报 TIMEOUT / SOFs after which a pending
  /// stop that is still not quiescent is reported as TIMEOUT
  static constexpr uint8_t STOP_TIMEOUT_SOFS = 16U;

  /**
   * @brief 构造并登记到控制器 / Construct and enter into the controller
   * @param controller 所属控制器，登记进它的端点表 / Owning controller; the endpoint is
   *        entered into its endpoint table
   * @param number 端点号，小于控制器的端点数 / Endpoint number, below the
   *        controller's endpoint count
   * @param direction 端点方向 / Endpoint direction
   * @param fifo_size IN：本端点的 TX FIFO 字节数；OUT：不用 / IN: TX FIFO bytes of this
   *        endpoint; OUT: unused
   * @param buffer 本端点的槽内存 / Slot memory of this endpoint
   * @note 调用者：STM32OtgController 构造，每个端点一次。
   *       Caller: the STM32OtgController constructor, once per endpoint.
   */
  STM32OtgEndpoint(STM32OtgController& controller, EPNumber number, Direction direction,
                   size_t fifo_size, RawData buffer);

 protected:
  void ConfigureHardware(const Config& cfg) override;
  void CloseHardware(bool in_isr) override;
  ErrorCode StartHardware(const HwRequest& request, bool in_isr) override;
  StopOutcome StopHardware(bool in_isr) override;
  ErrorCode StallHardware(bool in_isr) override;
  /// 批量/中断端点复位为 DATA0；EP0 的 STALL 由下一个 SETUP 清除 / Bulk and interrupt
  /// endpoints restart at DATA0; the next SETUP clears an EP0 STALL
  ErrorCode ClearStallHardware(bool in_isr) override;
  /**
   * @brief 硬件上限，只看参数和构造时的 FIFO 大小 / Hardware limits from the
   *        arguments and the FIFO sizes fixed at construction
   * @note 包必须整个放进 FIFO：IN 为本端点的 TX FIFO，OUT 为 rx_packet_limit_。批量和
   *       中断一次最多 1023 包、524284 字节（PKTCNT 10 位、XFRSIZ 19 位且 OUT 按字对齐，
   *       RM0468 p.2735、p.2744、p.2801）；控制和等时一次一个包。
   *       A packet must fit its FIFO whole: the endpoint's TX FIFO for IN,
   *       rx_packet_limit_ for OUT. Bulk and interrupt take up to 1023 packets and 524284
   *       bytes per request (10-bit PKTCNT, 19-bit XFRSIZ, OUT word aligned, RM0468
   *       p.2735, p.2744, p.2801); control and isochronous one packet.
   */
  HwLimits HardwareLimits(Direction direction, Type type) const override;

 private:
  friend class STM32USBOtgDevice;
  friend class STM32OtgController;

  bool IsIn() const { return AvailableDirection() == Direction::IN; }
  uint8_t Num() const { return EPNumberToInt8(GetNumber()); }
  bool Dma() const { return controller_.hw_.dma; }

  /// IN、FIFO 模式：按 FIFO 空间写整包 / IN, FIFO mode: write whole packets that fit
  void FillFifo();
  /// OUT、FIFO 模式：从 RX FIFO 读 bytes 字节，超出请求的丢弃 / OUT, FIFO mode: read
  /// bytes from the RX FIFO, dropping what exceeds the request
  void ReadFifo(size_t bytes);
  /// 硬件标记完成：算 actual，清 active_，OnHardwareComplete(OK) / The hardware flagged
  /// completion: compute actual, clear active_, OnHardwareComplete(OK)
  void Complete(bool in_isr);
  /// OUT 已收到的字节；DMA 时做 invalidate 并（EP0）拷进槽 / Bytes an OUT request
  /// received; with DMA invalidates and (EP0) copies into the slot
  size_t OutBytes() const;
  /// IN 被停时主机已取走的字节 / Bytes the host took from a stopped IN request
  size_t InBytesSent() const;

  /// 写下 IN 的停止；NAK 已生效则禁用并给结果，否则 PENDING / Issue the stop of an IN
  /// request: disable and give the outcome if the NAK is in effect, else PENDING
  StopOutcome StopIn();
  /**
   * @brief 写下 OUT 的停止（全局 OUT NAK） / Issue the stop of an OUT request (global OUT
   *        NAK)
   * @note 已生效则禁用并给结果，否则 PENDING。
   *       Disable and give the outcome if it is in effect, else PENDING.
   */
  StopOutcome StopOut();
  StopOutcome StopOutEp0();
  /**
   * @brief NAK 已生效时禁用 IN 端点并刷 TX FIFO / With the NAK in effect, disable the IN
   *        endpoint and flush its TX FIFO
   * @note 握手未在界内完成返回 false。
   *       False if a handshake did not finish within the bound.
   */
  bool DisableIn();
  /**
   * @brief 全局 OUT NAK 已生效时禁用 OUT 端点 / With the global OUT NAK in effect,
   *        disable the OUT endpoint
   * @note 握手未在界内完成返回 false。
   *       False if the handshake did not finish within the bound.
   */
  bool DisableOut();
  /// 端点已不使能时的结果：XFRC 为 OK，否则 ABORTED / Outcome once the endpoint is
  /// disabled: OK with XFRC, else ABORTED
  StopOutcome DisabledOutcome();
  /// 停止转为未结束：置 stopping_，计数，打开结束它的中断 / The stop becomes pending:
  /// set stopping_, count it and enable the interrupt that ends it
  void BeginPending();
  /**
   * @brief 停止不再未结束 / The stop is no longer pending
   * @note 清 stopping_，计数减一，最后一个时关掉中断并撤销全局 OUT NAK。
   *       Clear stopping_, uncount it, and with the last one mask the interrupt and
   *       withdraw the global OUT NAK.
   */
  void EndPending();
  /**
   * @brief 结束未结束的停止并报告 / End a pending stop and report it
   * @param quiescent 调用者已知 NAK 生效（中断标志、总线复位、挂起、刚收到的 SETUP）
   *        The caller knows the NAK is in effect (interrupt flag, bus reset, suspend, a
   *        SETUP just received)
   * @return false：还没有停稳，或禁用握手未完成，停止仍未结束。
   *         false: not quiescent yet, or the disable handshake did not finish; the stop
   *         stays pending.
   */
  bool FinishStop(bool in_isr, bool quiescent);

  STM32OtgController& controller_;  ///< 所属控制器 / Owning controller
  const size_t fifo_size_;          ///< IN 的 TX FIFO 字节 / TX FIFO bytes of IN

  /// @name 在途请求：仅 owner / Request in flight: owner only
  /// @{
  /**
   * @brief 硬件里有本端点的请求 / The hardware holds a request of this endpoint
   * @note 写：StartHardware 置位，Complete、给出结果的 StopHardware 和 FinishStop 清零；
   *       读：事件处理（忽略没有请求时的完成和 TXFE）、FillFifo、ReadFifo。
   *       Set by StartHardware, cleared by Complete, by a StopHardware that gives the
   *       outcome and by FinishStop; read by the event handling (ignores completions and
   *       TXFE with nothing in flight), FillFifo and ReadFifo.
   */
  bool active_ = false;
  /**
   * @brief 停止已写下、硬件还没有停稳 / The stop was issued and the hardware is not
   *        quiescent yet
   * @note 置位之后本端点不再读写槽：FillFifo 不再写、ReadFifo 只丢弃。
   *       While set the endpoint no longer touches the slot: FillFifo writes nothing and
   *       ReadFifo only drops.
   */
  bool stopping_ = false;
  /// 这次停止由后端自己发起，核心没有在等；结束时按 stop_report_ 报告 / The backend
  /// issued this stop itself and the core is not waiting; it ends with stop_report_
  bool self_stop_ = false;
  /// 已为这次停止报告过 TIMEOUT / TIMEOUT was reported for this stop
  bool stop_failed_ = false;
  /// 停止未结束期间经过的 SOF 数 / SOFs seen while the stop is pending
  uint8_t stop_sofs_ = 0U;
  /// 后端自己发起的停止结束时报告的结果 / Result reported when a backend-issued stop
  /// ends
  ErrorCode stop_report_ = ErrorCode::ABORTED;
  uint8_t* hw_data_ = nullptr;  ///< 请求的数据 / Data of the request
  size_t hw_length_ = 0U;       ///< 请求长度 / Length of the request
  /// OUT：写进 DOEPTSIZ.XFRSIZ 的值 / OUT: value written to DOEPTSIZ.XFRSIZ
  size_t hw_programmed_ = 0U;
  /// FIFO 模式：IN 已写进 FIFO、OUT 已读出的字节 / FIFO mode: bytes written to the
  /// FIFO (IN) or read from it (OUT)
  size_t hw_done_ = 0U;
  /// @}
};

#if defined(USB_OTG_FS)
/// STM32 OTG FS 设备 / STM32 OTG FS device
class STM32USBDeviceOtgFS : public STM32USBOtgDevice
{
 public:
  /// 参数见 STM32USBOtgDevice / Parameters as for STM32USBOtgDevice
  STM32USBDeviceOtgFS(
      const HardwareConfig& hw, size_t rx_fifo_size,
      const std::initializer_list<RawData> RX_EP_CFGS,
      const std::initializer_list<EPInConfig> TX_EP_CFGS,
      USB::DeviceDescriptor::PacketSize0 packet_size, uint16_t vid, uint16_t pid,
      uint16_t bcd,
      const std::initializer_list<const USB::DescriptorStrings::LanguagePack*> LANG_LIST,
      const std::initializer_list<const std::initializer_list<USB::DeviceClass*>> CONFIGS,
      ConstRawData uid = {nullptr, 0});
};
#endif

#if defined(USB_OTG_HS)
/// STM32 OTG HS 设备 / STM32 OTG HS device
class STM32USBDeviceOtgHS : public STM32USBOtgDevice
{
 public:
  /// 参数见 STM32USBOtgDevice / Parameters as for STM32USBOtgDevice
  STM32USBDeviceOtgHS(
      const HardwareConfig& hw, size_t rx_fifo_size,
      const std::initializer_list<RawData> RX_EP_CFGS,
      const std::initializer_list<EPInConfig> TX_EP_CFGS,
      USB::DeviceDescriptor::PacketSize0 packet_size, uint16_t vid, uint16_t pid,
      uint16_t bcd,
      const std::initializer_list<const USB::DescriptorStrings::LanguagePack*> LANG_LIST,
      const std::initializer_list<const std::initializer_list<USB::DeviceClass*>> CONFIGS,
      ConstRawData uid = {nullptr, 0});
};
#endif

#if !defined(USB_OTG_HS) && defined(USB1_OTG_HS)
/// STM32 OTG HS1 设备 / STM32 OTG HS1 device
class STM32USBDeviceOtgHS1 : public STM32USBOtgDevice
{
 public:
  /// 参数见 STM32USBOtgDevice / Parameters as for STM32USBOtgDevice
  STM32USBDeviceOtgHS1(
      const HardwareConfig& hw, size_t rx_fifo_size,
      const std::initializer_list<RawData> RX_EP_CFGS,
      const std::initializer_list<EPInConfig> TX_EP_CFGS,
      USB::DeviceDescriptor::PacketSize0 packet_size, uint16_t vid, uint16_t pid,
      uint16_t bcd,
      const std::initializer_list<const USB::DescriptorStrings::LanguagePack*> LANG_LIST,
      const std::initializer_list<const std::initializer_list<USB::DeviceClass*>> CONFIGS,
      ConstRawData uid = {nullptr, 0});
};
#endif

#if !defined(USB_OTG_HS) && defined(USB2_OTG_HS)
/// STM32 OTG HS2 设备 / STM32 OTG HS2 device
class STM32USBDeviceOtgHS2 : public STM32USBOtgDevice
{
 public:
  /// 参数见 STM32USBOtgDevice / Parameters as for STM32USBOtgDevice
  STM32USBDeviceOtgHS2(
      const HardwareConfig& hw, size_t rx_fifo_size,
      const std::initializer_list<RawData> RX_EP_CFGS,
      const std::initializer_list<EPInConfig> TX_EP_CFGS,
      USB::DeviceDescriptor::PacketSize0 packet_size, uint16_t vid, uint16_t pid,
      uint16_t bcd,
      const std::initializer_list<const USB::DescriptorStrings::LanguagePack*> LANG_LIST,
      const std::initializer_list<const std::initializer_list<USB::DeviceClass*>> CONFIGS,
      ConstRawData uid = {nullptr, 0});
};
#endif

}  // namespace LibXR

#endif
