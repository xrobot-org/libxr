#pragma once

#include <cstddef>
#include <cstdint>
#include <initializer_list>

#include "esp_def.hpp"
#include "esp_intr_alloc.h"
#include "libxr_type.hpp"
#include "soc/soc_caps.h"
#include "usb/core/ep.hpp"
#include "usb/core/ep_pool.hpp"
#include "usb/core/usb_service.hpp"
#include "usb/device/dev_core.hpp"

#if (SOC_USB_OTG_SUPPORTED && defined(CONFIG_IDF_TARGET_ESP32S3) && \
     CONFIG_IDF_TARGET_ESP32S3) ||                                  \
    defined(__DOXYGEN__)

namespace LibXR
{

class ESP32USBEndpoint;

/**
 * @class ESP32USBController
 * @brief ESP32-S3 USB OTG（DWC2，全速，内部 DMA）控制器：寄存器、FIFO 和全部端点 /
 *        ESP32-S3 USB OTG (DWC2, full speed, internal DMA) controller: registers,
 *        FIFOs and every endpoint
 *
 * 设备核心在构造时就要用到 EP0，所以端点必须在它之前建好；本类作为设备的第一个基类，
 * 负责建端点、清理 ROM 的 USB 栈、打开内部 PHY、复位 core 和划分 FIFO。构造后总线保持
 * 断开，中断线保持关闭，直到 Start。
 * The device core needs EP0 at construction, so the endpoints are built before it; this
 * class, the device's first base, builds the endpoints, tears down the ROM USB stack,
 * opens the internal PHY, resets the core and lays out the FIFOs. After construction
 * the bus stays detached and the interrupt line off until Start.
 *
 * @note FIFO 划分在构造时固定：EP0 的 TX FIFO 16 字，每个 IN 端点按 EPConfig 的
 *       in_fifo_bytes，GHWCFG3.DFIFODEPTH 里其余的全部给共享的 RX FIFO。之后不再改。
 *       The FIFO layout is fixed at construction: 16 words of TX FIFO for EP0, each IN
 *       endpoint gets in_fifo_bytes of its EPConfig, and the rest of
 *       GHWCFG3.DFIFODEPTH is the shared RX FIFO. It never changes afterwards.
 * @note 构造之后寄存器只在设备的 owner 中访问（ESP32USBDevice 的 Start、Stop 例外，
 *       见其说明）。
 *       After construction the registers are touched only in the device's owner (except
 *       Start and Stop of ESP32USBDevice, see there).
 */
class ESP32USBController : public USB::EndpointPool
{
 public:
  /// 端点配置项；第一项属于 EP0 / Endpoint configuration entry; the first is EP0's
  struct EPConfig
  {
    /// 本项生成哪些方向 / Which directions this entry creates
    enum class DirectionHint : int8_t
    {
      BOTH_DIRECTIONS = -1,  ///< 内存平分给 OUT 和 IN / Memory split for OUT and IN
      OUT_ONLY = 0,          ///< 只有 OUT / OUT only
      IN_ONLY = 1            ///< 只有 IN / IN only
    };

    /**
     * @brief 端点内存 / Endpoint memory
     * @note 内部 DMA 可达 RAM，起点和长度按 HW_CACHE_LINE_SIZE 对齐；两个方向时平分。
     *       Internal DMA-capable RAM, start and length aligned to HW_CACHE_LINE_SIZE;
     *       split in halves for two directions.
     */
    RawData buffer;
    /// 方向 / Directions
    DirectionHint direction_hint = DirectionHint::BOTH_DIRECTIONS;
    /// IN 端点的 TX FIFO 字节数，4 的倍数且不小于 64；EP0 不用 / TX FIFO bytes of the
    /// IN endpoint, a multiple of 4 and at least 64; unused for EP0
    size_t in_fifo_bytes = 64U;

    EPConfig() = delete;
    /// 两个方向共用一块内存 / Both directions from one memory block
    EPConfig(RawData buffer) : buffer(buffer) {}
    /// 单方向 / One direction
    EPConfig(RawData buffer, bool is_in)
        : buffer(buffer),
          direction_hint(is_in ? DirectionHint::IN_ONLY : DirectionHint::OUT_ONLY)
    {
    }
  };

 protected:
  /**
   * @brief 建端点、初始化硬件 / Build the endpoints and initialize the hardware
   * @param ep_cfgs 端点配置，第一项为 EP0，其余依次为 EP1、EP2… / Endpoint
   *        configurations: EP0 first, then EP1, EP2 and so on
   * @param packet_size EP0 包长 / EP0 packet size
   * @note 调用者：ESP32USBDevice 的构造，在任务中、控制器中断分配之前。
   *       Caller: the constructor of ESP32USBDevice, in a task, before the
   *       controller interrupt is allocated.
   */
  ESP32USBController(const std::initializer_list<EPConfig>& ep_cfgs, size_t packet_size);

  /**
   * @brief 复位 core 并重建寄存器基线 / Reset the core and rebuild the register
   *        baseline
   * @return true 成功；false 某个自清位在 CORE_RESET_BOUND_US 内没有清。
   *         true on success; false when a self-clearing bit did not clear within
   *         CORE_RESET_BOUND_US.
   * @note 调用者：构造（任务）、ResetController（owner）。之后总线断开、全局中断
   *       屏蔽、所有端点不使能、FIFO 按构造时的划分。
   *       Callers: the constructor (task) and ResetController (owner). Afterwards the
   *       bus is detached, the global interrupt masked, every endpoint disabled and
   *       the FIFOs laid out as at construction.
   */
  bool ResetCore();

  /// 清中断状态、打开中断、接上总线；调用者：Start、ResetController / Clear status,
  /// enable interrupts and attach; callers: Start, ResetController
  void Connect();

  /**
   * @brief 让 EP0 OUT 只接收 SETUP，OUT 数据保持 NAK / Make EP0 OUT receive SETUP
   *        packets only, OUT data stays NAKed
   * @note 仅 owner：OnControlReady、处理 SETUP 前，以及 EP0 OUT 被 core 关掉而没有
   *       请求在途时。DMA 时 core 只在 EP0 OUT 使能时存 SETUP。保持 NAK：清掉 NAK 时，
   *       控制写的数据包会在核心为它挂接收之前被收进 SETUP 缓冲（ESP32-S3 实测，主机
   *       5 s 超时）。
   *       Owner only: in OnControlReady, before a SETUP is handled, and when the core
   *       disabled EP0 OUT with no request in flight. With DMA the core stores a SETUP
   *       only while EP0 OUT is enabled. The NAK stays: with it cleared, the data packet
   *       of a control write lands in the SETUP buffer before the core arms a receive
   *       for it (measured on an ESP32-S3; the host times out after 5 s).
   */
  void ArmSetup();

  /**
   * @brief 硬件已收到、还没有交给设备核心的 SETUP / A SETUP the hardware received and
   *        that is not handed to the device core yet
   * @note 仅 owner。STUP 要到主机的下一个令牌才置位（STM32H723 上同一个 IP 实测晚
   *       0.7~1.1 ms），所以也看更早出现的 STPKTRX，以及事件处理清掉 STPKTRX 时记下的
   *       setup_pending_。S3 上 STPKTRX 是否置位没有测过。
   *       Owner only. STUP is set only at the host's next token (0.7 to 1.1 ms later,
   *       measured on the same IP in an STM32H723), so the earlier STPKTRX counts too,
   *       and so does setup_pending_, noted when the event handling clears STPKTRX.
   *       Whether the S3 sets STPKTRX has not been measured.
   */
  bool SetupLatched() const;

  /// EP0 的 TX FIFO 字数 / TX FIFO words of EP0
  static constexpr uint32_t EP0_FIFO_WORDS = 16U;
  /// core 的 OUT 端点数（EP0..EP6）/ OUT endpoints of the core (EP0..EP6)
  static constexpr uint8_t ENDPOINT_COUNT = 7U;
  /// 有专用 TX FIFO 的 IN 端点号上限 / Highest IN endpoint number with a TX FIFO
  static constexpr uint8_t MAX_IN_NUMBER = 4U;
  /// SETUP 包长 / SETUP packet size
  static constexpr size_t SETUP_SIZE = 8U;
  /// EP0 的 DMA 缓冲大小 / Size of the EP0 DMA buffers
  static constexpr size_t EP0_BUFFER_SIZE = 64U;
  /// 自清位等待的上限（µs）/ Bound of a wait on a self-clearing bit (µs)
  static constexpr int64_t CORE_RESET_BOUND_US = 20000;

  /// IN 端点，下标为端点号，没有时为空；构造时写，之后只读 / IN endpoints by number,
  /// null when absent; written at construction, read-only afterwards
  ESP32USBEndpoint* in_[ENDPOINT_COUNT] = {};
  /// OUT 端点，下标为端点号，没有时为空；构造时写，之后只读 / OUT endpoints by number,
  /// null when absent; written at construction, read-only afterwards
  ESP32USBEndpoint* out_[ENDPOINT_COUNT] = {};

  /**
   * @brief SETUP 的 DMA 缓冲 / DMA buffer for SETUP packets
   * @note core 把每个 SETUP 写到 DOEPDMA0 并前移 8 字节；ArmSetup 把 DOEPDMA0 指回开头。
   *       The core writes each SETUP at DOEPDMA0 and advances it by 8 bytes; ArmSetup
   *       points DOEPDMA0 back at the start.
   */
  alignas(4) uint8_t setup_buffer_[EP0_BUFFER_SIZE] = {};
  /**
   * @brief EP0 OUT 数据和状态阶段的 DMA 缓冲 / DMA buffer of the EP0 OUT data and
   *        status stages
   * @note 硬件从不写 EP0 OUT 的槽：数据收进这里，完成时拷进槽。数据阶段挂着时到来的
   *       SETUP 也写在这里的开头。
   *       The hardware never writes the EP0 OUT slot: data lands here and is copied
   *       into the slot on completion. A SETUP that arrives while a data stage is armed
   *       is written at the start of this buffer too.
   */
  alignas(4) uint8_t ep0_out_buffer_[EP0_BUFFER_SIZE] = {};

 private:
  friend class ESP32USBEndpoint;

  /// 按构造时的划分写 FIFO 尺寸寄存器 / Write the FIFO size registers from the layout
  void ProgramFifos();

  /// 共享 RX FIFO 的字数；构造时写 / Words of the shared RX FIFO; set at construction
  uint32_t rx_fifo_words_ = 0U;
  /// 各 IN 端点的 TX FIFO 字数，下标为端点号；构造时写 / TX FIFO words per IN endpoint
  /// number; set at construction
  uint32_t tx_fifo_words_[MAX_IN_NUMBER + 1U] = {};
  /// RX FIFO 扣除固定开销后一个 OUT 包能用的字节；构造时写 / Bytes one OUT packet can
  /// use in the RX FIFO after its fixed overhead; set at construction
  size_t rx_packet_limit_ = 0U;

 protected:
  /// @name 未结束的停止：仅 owner / Pending stops: owner only
  /// @{

  /// STPKTRX 已见到并清掉、SETUP 还没有交付 / STPKTRX was seen and cleared and the
  /// SETUP is not handed over yet
  bool setup_pending_ = false;
  /// 停止未结束的 IN 端点数；非 0 时 DIEPMSK.INEPNE 打开 / IN endpoints whose stop is
  /// pending; DIEPMSK.INEPNE is enabled while non-zero
  uint8_t in_stops_ = 0U;
  /**
   * @brief 停止未结束的 OUT 端点数 / OUT endpoints whose stop is pending
   * @note 非 0 时全局 OUT NAK 保持、GOUTNAKEFF 中断打开。
   *       While non-zero the global OUT NAK is held and the GOUTNAKEFF interrupt enabled.
   */
  uint8_t out_stops_ = 0U;
  /// @}
};

/**
 * @class ESP32USBDevice
 * @brief ESP32-S3 USB 设备 / ESP32-S3 USB device
 *
 * 实现设备服务的三个入口：ServiceHardware、MaskInterrupt、UnmaskInterrupt。没有
 * PendInterrupt：USB 中断源是电平触发，软件不能挂起它，所以线程就地成为 owner，可在
 * 任一核上。屏蔽只把本中断源在中断矩阵上改接到不用的 6 号 CPU 中断，不关全部中断，
 * 也不取 IDF 的中断分配锁。端点停止不等总线，见 ESP32USBEndpoint。
 * Implements three entries of the device service: ServiceHardware, MaskInterrupt and
 * UnmaskInterrupt. There is no PendInterrupt: the USB source is level-triggered and
 * software cannot pend it, so a thread owns in place, on either core. Masking only
 * routes this source to the unused CPU interrupt 6 at the interrupt matrix; it never
 * disables all interrupts and does not take the IDF interrupt-allocation lock.
 * Endpoint stops never wait for the bus, see ESP32USBEndpoint.
 *
 * @note 构造必须在任务中进行（分配中断、打开 PHY）；中断分配在调用构造的那个核上。
 *       Construct in a task (it allocates the interrupt and opens the PHY); the
 *       interrupt is allocated on the core that runs the constructor.
 */
class ESP32USBDevice : public ESP32USBController,
                       public USB::DeviceService,
                       public USB::DeviceCore
{
 public:
  /**
   * @brief 构造 / Construct
   * @param ep_cfgs 端点配置，见 ESP32USBController / Endpoint configurations, see
   *        ESP32USBController
   * @param packet_size EP0 包长 / EP0 packet size
   * @param vid 厂商 ID / Vendor ID
   * @param pid 产品 ID / Product ID
   * @param bcd bcdDevice
   * @param lang_list 字符串语言包 / String language packs
   * @param configs 每个配置的类列表 / Class list of each configuration
   * @param uid 序列号字符串后缀，可为空 / Serial-number suffix, may be empty
   * @note 登记全部端点，分配中断（保持关闭）。
   *       Registers every endpoint and allocates the interrupt (left off).
   */
  ESP32USBDevice(
      const std::initializer_list<EPConfig> ep_cfgs,
      USB::DeviceDescriptor::PacketSize0 packet_size, uint16_t vid, uint16_t pid,
      uint16_t bcd,
      const std::initializer_list<const USB::DescriptorStrings::LanguagePack*> lang_list,
      const std::initializer_list<const std::initializer_list<USB::DeviceClass*>> configs,
      ConstRawData uid = {nullptr, 0});

  /**
   * @brief 打开中断并接上总线 / Enable interrupts and attach to the bus
   * @note 调用者：应用，任务中。在 owner 之外写 GINTSTS、GINTMSK、GAHBCFG、DCTL，期间
   *       屏蔽本中断线；不与另一个核上持有 owner 的线程或中断互斥。
   *       Caller: the application, in a task. Writes GINTSTS, GINTMSK, GAHBCFG and
   *       DCTL outside the owner with this line masked; not exclusive against a thread
   *       or an interrupt that owns on the other core.
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
  /// @name 设备服务的入口 / Entries of the device service
  /// @{
  /// 按固定顺序处理一次状态快照，见 cpp / Handle one status snapshot in a fixed order,
  /// see the cpp
  void ServiceHardware(bool in_isr) override;
  /// 把中断源改接到 6 号 CPU 中断 / Route the source to CPU interrupt 6
  void MaskInterrupt() override;
  /// 把中断源接回分配时的 CPU 中断 / Route the source back to its allocated interrupt
  void UnmaskInterrupt() override;
  /// @}

  /// @name 设备核心的后端钩子（owner）/ Backend hooks of the device core (owner)
  /// @{
  /// 在 BEFORE_STATUS 写 DCFG.DEVADDR / DCFG.DEVADDR is written at BEFORE_STATUS
  void SetAddress(uint8_t address, AddressStage stage) override;
  /// ArmSetup
  void OnControlReady() override;
  /**
   * @brief 断开、复位 core、重建基线，再接上总线 / Detach, reset the core, rebuild the
   *        baseline and attach again
   * @note 返回后所有端点寄存器的 EPENA 和 XFRC 为 0，未结束的停止作废、不再报告，
   *       之后每个 StopHardware 立即返回 ABORTED。断开保持 RESET_DETACH_US，在 owner
   *       里忙等，主机才会复位总线。core 复位失败时保持断开。
   *       Afterwards EPENA and XFRC of every endpoint register are 0, pending stops are
   *       void and reported no more, and every later StopHardware returns ABORTED at
   *       once. The detach lasts RESET_DETACH_US, busy-waiting in the owner, so that the
   *       host resets the bus. If the core reset fails the device stays detached.
   */
  void ResetController(bool in_isr) override;
  /// @}

 private:
  /// ResetController 断开总线的时长（µs）/ How long ResetController stays detached (µs)
  static constexpr int64_t RESET_DETACH_US = 10000;
  /**
   * @brief HaltController 断开之后等够 RESET_DETACH_US / Wait until RESET_DETACH_US have
   *        passed since HaltController detached
   * @note StartController 和 ResetController 在重新接上之前调用。
   *       Called by StartController and ResetController before they attach.
   */
  void WaitDetached();
  /// HaltController 断开总线的时刻（µs）和“断开后还没有重新接上” / Time HaltController
  /// detached (us) and "not attached again since"
  int64_t halted_at_us_ = 0;
  bool halted_ = false;
  /// 6 号 CPU 中断：接到它的中断源被屏蔽 / CPU interrupt 6: a source routed to it is
  /// masked
  static constexpr uint32_t DISABLED_CPU_INTERRUPT = 6U;

  /// 中断入口：OnControllerInterrupt / Interrupt entry: OnControllerInterrupt
  static void IsrEntry(void* arg);

  /**
   * @brief 断开总线、作废未结束的停止并复位 core / Detach, void the pending stops and
   *        reset the core
   * @note StopController 和 ResetController 共用。返回 ResetCore 的结果。
   *       Shared by StopController and ResetController. Returns what ResetCore returned.
   */
  bool HaltController();
  void OnUsbReset(bool in_isr);
  void HandleInEndpoints(bool in_isr);
  void HandleOutEndpoints(bool in_isr);
  void HandleEp0Out(bool in_isr, uint32_t events);
  void HandleIsoIncomplete(bool in_isr);
  /// 全局 OUT NAK 生效：结束各 OUT 端点未结束的停止 / The global OUT NAK took effect:
  /// end the pending stops of the OUT endpoints
  void HandleGlobalOutNak(bool in_isr);
  /**
   * @brief 总线上没有包时结束所有未结束的停止 / End every pending stop while no packet
   *        is on the bus
   * @note 调用者：总线复位、挂起的处理，在向设备核心报告之前。NAK 生效只取决于芯片，
   *       用有上限的寄存器握手完成；握手没有完成的报 TIMEOUT。
   *       Callers: the bus reset and suspend handling, before the event is reported to
   *       the device core. The NAK taking effect depends on the chip alone and is
   *       completed with bounded register handshakes; a stop whose handshake does not
   *       finish is reported as TIMEOUT.
   */
  void FinishPendingStops(bool in_isr);
  /// 每个 SOF 给未结束的停止计数，到上限再查一次，仍未停稳报 TIMEOUT / Count a SOF
  /// for every pending stop; at the bound check once more and report TIMEOUT
  void CountStopTimeouts(bool in_isr);
  /// 复位期间的假挂起：软断开 20 µs 再连接 / A suspend inside a bus reset: soft
  /// detach for 20 µs, then attach
  void RecoverFalseSuspend();

  intr_handle_t intr_handle_ = nullptr;  ///< 中断句柄；构造时写 / Handle, constructor
  int intr_cpu_ = 0;  ///< 中断所在的核；构造时写 / Core of the interrupt, constructor
  /// 分配到的 CPU 中断号；构造时写 / Allocated CPU interrupt number, constructor
  uint32_t intr_number_ = 0U;
  /**
   * @brief 总线复位之后、ENUMDONE 之前 / Between a bus reset and its ENUMDONE
   * @note 写：OnUsbReset 置位，ENUMDONE、假挂起恢复和 ResetController 清零；读：
   *       USBSUSP 的处理。都在 owner 中。
   *       Set by OnUsbReset, cleared by ENUMDONE, the false-suspend recovery and
   *       ResetController; read when handling USBSUSP. All in the owner.
   */
  bool reset_enum_pending_ = false;
};

/**
 * @class ESP32USBEndpoint
 * @brief ESP32-S3 USB 端点，内部 DMA，停止不等总线 / ESP32-S3 USB endpoint with
 *        internal DMA and a stop that never waits for the bus
 *
 * core 直接读写槽（EP0 OUT 例外，收进控制器的 EP0 缓冲再拷进槽）。停止按 RM0468
 * p.2806-2807（IN）和 p.2798-2799（OUT）的步骤（同一个 DWC2 IP，用在 S3 上属推断，
 * 步骤本身 S3 实测过）：NAK 是否生效要等线上的包，StopHardware 不等，没有生效就返回
 * PENDING，由 INEPNE、GOUTNAKEFF 中断或请求自己的完成结束；禁用和刷新 FIFO 是只取决于
 * 芯片的握手（S3 实测 EPDISD 最长 1.45 µs、TXFFLSH 最长 1.82 µs），读 HANDSHAKE_SPINS
 * 次寄存器为限。停止未结束满 STOP_TIMEOUT_SOFS 个帧起始报 TIMEOUT。
 * The core reads and writes the slot itself (except EP0 OUT, which lands in the
 * controller's EP0 buffer and is copied into the slot). A stop follows RM0468
 * p.2806-2807 (IN) and p.2798-2799 (OUT) (the same DWC2 IP; inferred for the S3, the
 * steps themselves measured on it): whether the NAK took effect depends on the packet on
 * the wire, which StopHardware does not wait for; without it PENDING is returned and
 * the INEPNE or GOUTNAKEFF interrupt, or the request's own completion, ends the stop.
 * Disabling and flushing the FIFO are handshakes that depend on the chip alone (EPDISD
 * at most 1.45 µs and TXFFLSH at most 1.82 µs measured on the S3), bounded by
 * HANDSHAKE_SPINS register reads. A stop still pending after STOP_TIMEOUT_SOFS starts
 * of frame is reported as TIMEOUT.
 *
 * @note 停止，对应 Endpoint::StopHardware 的规则。
 *       停稳的证据：硬件的停稳事件（IN 是 INEPNE，OUT 是全局 OUT NAK 生效）和请求自己的
 *       完成。协议上界用帧起始计数，见 STOP_TIMEOUT_SOFS。
 *       缺陷，停止的手段影响别的端点：全局 OUT NAK 期间所有 OUT 端点回 NAK，主机会
 *       重发；控制器丢掉这期间别的等时 OUT 端点的包。
 *       “停稳的事件会丢”在 S3 上没有见到，本后端没有重新请求。另外两类在实测中没有出现。
 *       Stops, against the rules of Endpoint::StopHardware.
 *       Evidence of quiescence: the hardware's quiescence events (INEPNE for IN, global
 *       OUT NAK effective for OUT) and the request's own completion. The protocol bound
 *       is counted in starts of frame, see STOP_TIMEOUT_SOFS.
 *       Flaw, the means of stopping affects other endpoints: during global OUT NAK every
 *       OUT endpoint answers NAK, which the host retries; the controller drops the
 *       packets of other isochronous OUT endpoints meanwhile.
 *       A lost quiescence event was not seen on the S3 and this backend makes no second
 *       request. The other two kinds did not show in the measurements.
 */
class ESP32USBEndpoint : public USB::Endpoint
{
 public:
  /// 与总线无关的寄存器握手最多读这么多次 / Register reads a bus-independent
  /// handshake may take at most
  static constexpr uint32_t HANDSHAKE_SPINS = 256U;
  /// 未结束的停止经过这么多个 SOF 仍未停稳就报 TIMEOUT / SOFs after which a pending
  /// stop that is still not quiescent is reported as TIMEOUT
  static constexpr uint8_t STOP_TIMEOUT_SOFS = 16U;

  /**
   * @brief 构造并登记到控制器 / Construct and enter into the controller
   * @note 调用者：ESP32USBController 构造，每个端点一次。槽内存必须在内部 DMA 可达
   *       RAM 中。
   *       Caller: the ESP32USBController constructor, once per endpoint. The slot
   *       memory must be in internal DMA-capable RAM.
   */
  ESP32USBEndpoint(ESP32USBController& controller, EPNumber number, Direction direction,
                   RawData buffer);

 protected:
  void ConfigureHardware(const Config& cfg) override;
  /// 保持 USBACTEP，只置 NAK / Keeps USBACTEP and sets NAK only
  void CloseHardware(bool in_isr) override;
  ErrorCode StartHardware(const HwRequest& request, bool in_isr) override;
  StopOutcome StopHardware(bool in_isr) override;
  ErrorCode StallHardware(bool in_isr) override;
  /// 批量/中断端点复位为 DATA0；EP0 的 STALL 由下一个 SETUP 清除 / Bulk and interrupt
  /// endpoints restart at DATA0; the next SETUP clears an EP0 STALL
  ErrorCode ClearStallHardware(bool in_isr) override;
  /**
   * @brief 硬件上限，只看参数和构造时的 FIFO 划分 / Hardware limits from the
   *        arguments and the FIFO layout fixed at construction
   * @note 包必须整个放进 FIFO：IN 为本端点的 TX FIFO，OUT 为 rx_packet_limit_。批量和
   *       中断一次最多 127 包、IN 65535 字节、OUT 65532 字节（PKTCNT 7 位、XFERSIZE
   *       16 位、OUT 按字对齐；ESP-IDF usb_dwc_cfg.h，TRM 未写）；控制和等时一次一个包。
   *       A packet must fit its FIFO whole: the endpoint's TX FIFO for IN,
   *       rx_packet_limit_ for OUT. Bulk and interrupt take up to 127 packets and 65535
   *       bytes IN or 65532 bytes OUT per request (7-bit PKTCNT, 16-bit XFERSIZE, OUT
   *       word aligned; ESP-IDF usb_dwc_cfg.h, not in the TRM); control and
   *       isochronous one packet.
   */
  HwLimits HardwareLimits(Direction direction, Type type) const override;

 private:
  friend class ESP32USBDevice;
  friend class ESP32USBController;

  /// EP0 OUT 当前装的是什么 / What EP0 OUT is armed for
  enum class Ep0Armed : uint8_t
  {
    SETUP,  ///< ArmSetup：只收 SETUP / ArmSetup: SETUP packets only
    DATA    ///< StartHardware：数据或状态阶段 / StartHardware: data or status stage
  };

  bool IsIn() const { return AvailableDirection() == Direction::IN; }
  uint8_t Num() const { return EPNumberToInt8(GetNumber()); }

  /// 硬件标记完成：算 actual，清 active_，OnHardwareComplete(OK) / The hardware flagged
  /// completion: compute actual, clear active_, OnHardwareComplete(OK)
  void Complete(bool in_isr);
  /// OUT 已收到的字节；EP0 拷进槽 / Bytes an OUT request received; EP0 copies them
  /// into the slot
  size_t OutBytes() const;
  /// IN 被停时主机已取走的字节 / Bytes the host took from a stopped IN request
  size_t InBytesSent() const;

  /// IN：写 SNAK；NAK 已生效就禁用并给结果，否则 PENDING / IN: write SNAK; with the
  /// NAK in effect disable and give the outcome, else PENDING
  StopOutcome StopIn();
  /// OUT：请求全局 OUT NAK；已生效就禁用并给结果，否则 PENDING / OUT: request the
  /// global OUT NAK; with it in effect disable and give the outcome, else PENDING
  StopOutcome StopOut();
  /// EP0 OUT：只写 SNAK，当场给结果 / EP0 OUT: SNAK only, outcome at once
  StopOutcome StopOutEp0();
  /// 禁用 IN 端点并刷新它的 TX FIFO；握手超限返回 false / Disable the IN endpoint and
  /// flush its TX FIFO; false when a handshake exceeds its bound
  bool DisableIn();
  /// 禁用 OUT 端点；握手超限返回 false / Disable the OUT endpoint; false when the
  /// handshake exceeds its bound
  bool DisableOut();
  /// 端点已不使能时被停请求的结果 / Outcome of the stopped request once the endpoint
  /// is disabled
  StopOutcome DisabledOutcome();
  /// 停止转为未结束：计数，打开对应的 NAK 生效中断 / The stop becomes pending: count
  /// it and enable the matching NAK-effective interrupt
  void BeginPending();
  /**
   * @brief 停止不再未结束 / The stop is no longer pending
   * @note 计数减一，最后一个时关中断（OUT 另撤销全局 OUT NAK）。
   *       Uncount it and, with the last, mask the interrupt (OUT also withdraws the
   *       global OUT NAK).
   */
  void EndPending();
  /**
   * @brief 尝试结束未结束的停止并报告 / Try to end the pending stop and report it
   * @param quiescent 调用者已知 NAK 已生效或总线上没有包 / The caller knows the NAK
   *        took effect or no packet is on the bus
   * @return 是否已结束并报告 / Whether it ended and was reported
   */
  bool FinishStop(bool in_isr, bool quiescent);

  ESP32USBController& controller_;  ///< 所属控制器 / Owning controller

  /// @name 在途请求：仅 owner / Request in flight: owner only
  /// @{
  /**
   * @brief 硬件里有本端点的请求 / The hardware holds a request of this endpoint
   * @note 写：StartHardware 置位，Complete、StopHardware（非 PENDING）和 FinishStop
   *       清零；读：事件处理（忽略没有请求时的完成）。
   *       Set by StartHardware, cleared by Complete, StopHardware (unless PENDING) and
   *       FinishStop; read by the event handling (ignores completions with nothing in
   *       flight).
   */
  bool active_ = false;
  /// 停止已写下、硬件还没有停稳 / The stop was issued and the hardware is not
  /// quiescent yet
  bool stopping_ = false;
  /// 这次停止是后端自己发起的，核心没有在等 / The backend issued this stop itself; the
  /// core is not waiting for it
  bool self_stop_ = false;
  /// 已为这次停止报告过 TIMEOUT / TIMEOUT was reported for this stop
  bool stop_failed_ = false;
  /// 停止未结束期间经过的 SOF 数 / SOFs seen while the stop is pending
  uint8_t stop_sofs_ = 0U;
  /// 后端自己发起的停止结束时报告的结果 / Result reported when a backend-issued stop
  /// ends
  ErrorCode stop_report_ = ErrorCode::FAILED;
  uint8_t* hw_data_ = nullptr;  ///< 请求的数据 / Data of the request
  size_t hw_length_ = 0U;       ///< 请求长度 / Length of the request
  /// 写进 XFERSIZE 的值 / Value written to XFERSIZE
  size_t hw_programmed_ = 0U;
  /**
   * @brief EP0 OUT 装的是什么 / What EP0 OUT is armed for
   * @note 写：ArmSetup 写 SETUP，StartHardware 写 DATA；读：EP0 OUT 的完成判断和停止。
   *       Written by ArmSetup (SETUP) and StartHardware (DATA); read by the EP0 OUT
   *       completion check and stop.
   */
  Ep0Armed ep0_armed_ = Ep0Armed::SETUP;
  /// @}
};

}  // namespace LibXR

#endif
