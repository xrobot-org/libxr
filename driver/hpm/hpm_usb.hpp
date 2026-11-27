#pragma once

#include <cstddef>
#include <cstdint>
#include <initializer_list>

#include "hpm_soc.h"
#include "libxr_def.hpp"
#include "libxr_type.hpp"
#include "usb/core/ep.hpp"
#include "usb/core/ep_pool.hpp"
#include "usb/core/usb_detach_timer.hpp"
#include "usb/core/usb_service.hpp"
#include "usb/device/dev_core.hpp"

#if defined(HPM_USB0_BASE) || defined(__DOXYGEN__)

namespace LibXR
{

class HPMUSBEndpoint;

/**
 * @brief 一个端点方向的槽内存和槽数 / Slot memory and slot count of one endpoint
 *        direction
 * @note 只给内存（可隐式转换）时槽数为 0，由后端取默认：EP0 一个，其余
 *       HPMUSBEndpoint::DEFAULT_SLOTS 个。槽数 1 到 Endpoint::MAX_SLOTS，内存由核心平分
 *       给各槽；非控制端点硬件上最多同时挂这么多个请求，每个槽一个 dTD（32 字节，取自
 *       控制器的 dTD 池）。
 *       Given the memory alone (implicitly) the slot count is 0 and the backend takes its
 *       default: one for EP0, HPMUSBEndpoint::DEFAULT_SLOTS for the others. 1 to
 *       Endpoint::MAX_SLOTS slots, the memory split evenly by the core; a non-control
 *       endpoint holds that many requests on the hardware at once, one dTD per slot
 *       (32 bytes, from the controller's dTD pool).
 */
struct HPMUSBEndpointMemory
{
  /// 只给内存，默认槽数 / Memory only, default slot count
  HPMUSBEndpointMemory(RawData memory, uint8_t slots = 0U) : memory(memory), slots(slots)
  {
  }
  /// 地址、长度和槽数 / Address, length and slot count
  HPMUSBEndpointMemory(void* addr, size_t size, uint8_t slots = 0U)
      : memory(addr, size), slots(slots)
  {
  }
  RawData memory;  ///< 槽内存；地址为 nullptr 表示没有这个方向 / Slot memory; null: none
  uint8_t slots;   ///< 槽数，0 为默认 / Slot count, 0 for the default
};

/**
 * @class HPMUSBController
 * @brief HPM USB 设备控制器：寄存器、队列头、传输描述符和全部端点对象 / HPM USB device
 *        controller: registers, queue heads, transfer descriptors and every endpoint
 *
 * 控制器按端点队列头（dQH）和传输描述符（dTD）工作：每个端点方向一个 dQH，表头地址写入
 * ENDPTLISTADDR；本后端每个请求一个 dTD，挂在 dQH 的覆盖区后写 ENDPTPRIME 装入。设备核心
 * 在构造时就要用到 EP0，所以端点必须在它之前建好；本类作为设备的第一个基类，负责 PHY 和
 * 控制器初始化、建端点。时钟和引脚由 BSP 在构造之前设好（init_usb0_clock 等）。
 * The controller works from endpoint queue heads (dQH) and transfer descriptors (dTD):
 * one dQH per endpoint direction, the list address written to ENDPTLISTADDR; this
 * backend gives each request one dTD, linked into the dQH overlay and loaded by writing
 * ENDPTPRIME. The device core needs EP0 at construction, so the endpoints are built
 * before it; this class, the device's first base, initializes the PHY and the controller
 * and builds the endpoints. The BSP sets the clock and the pins before construction
 * (init_usb0_clock and the like).
 *
 * @note 构造之后寄存器只在设备的 owner 中访问（HPMUSBDevice 的 StartController 例外，
 *       见其说明）。队列头和描述符放在本对象里，对象必须放在 USB DMA 能访问、没有数据
 *       缓存的内存里（HPM5301 的 DLM）。
 *       After construction the registers are touched only in the device's owner
 *       (except StartController of HPMUSBDevice, see there). The queue heads and
 *       descriptors live in this object, which must sit in memory the USB DMA reaches
 *       and no data cache covers (the DLM on an HPM5301).
 */
class HPMUSBController : public USB::EndpointPool
{
 public:
  /// 端点号上限（ENDPTCTRL 的个数）/ Endpoint numbers (count of ENDPTCTRL registers)
  static constexpr uint8_t MAX_ENDPOINTS = 16U;
  /// ENDPTCOMPLETE 等寄存器中 IN 方向的位偏移 / Bit offset of the IN direction in
  /// ENDPTCOMPLETE and the like
  static constexpr uint32_t IN_SHIFT = 16U;

  /**
   * @brief 端点队列头（dQH），64 字节 / Endpoint queue head (dQH), 64 bytes
   * @note 字 0 能力、字 1 当前 dTD、字 2-8 覆盖区（下一 dTD、令牌、5 个页指针）、字 10-11
   *       SETUP 包（SDK hpm_usb_device.h 的 dcd_qhd_t）。
   *       Word 0 capabilities, word 1 current dTD, words 2-8 the overlay (next dTD,
   *       token, 5 page pointers), words 10-11 the SETUP packet (dcd_qhd_t in the SDK's
   *       hpm_usb_device.h).
   */
  struct alignas(64) QueueHead
  {
    volatile uint32_t capabilities;  ///< 包长、ZLT、IOS、MULT / Size, ZLT, IOS, MULT
    volatile uint32_t current;       ///< 当前 dTD / Current dTD
    volatile uint32_t next;          ///< 覆盖区：下一 dTD / Overlay: next dTD
    volatile uint32_t token;         ///< 覆盖区：令牌 / Overlay: token
    volatile uint32_t buffer[5];     ///< 覆盖区：页指针 / Overlay: page pointers
    volatile uint32_t reserved;      ///< 保留 / Reserved
    volatile uint32_t setup[2];      ///< SETUP 包（仅 EP0 OUT）/ SETUP packet (EP0 OUT)
    uint32_t padding[4];             ///< 对齐到 64 字节 / Up to 64 bytes
  };
  static_assert(sizeof(QueueHead) == 64U, "dQH is 64 bytes");

  /// 传输描述符（dTD），32 字节对齐 / Transfer descriptor (dTD), 32-byte aligned
  struct alignas(32) TransferDescriptor
  {
    volatile uint32_t next;       ///< 下一 dTD，位 0 为结束 / Next dTD, bit 0 terminates
    volatile uint32_t token;      ///< 状态、IOC、剩余字节 / Status, IOC, bytes left
    volatile uint32_t buffer[5];  ///< 页指针 / Page pointers
    uint32_t length;              ///< 软件：请求长度 / Software: request length
  };
  static_assert(sizeof(TransferDescriptor) == 32U, "dTD is 32 bytes");

 protected:
  /**
   * @brief 初始化 PHY 和控制器，建全部端点 / Initialize the PHY and the controller and
   *        build every endpoint
   * @param regs USB 寄存器块 / USB register block
   * @param irq 本控制器的中断号 / Interrupt number of this controller
   * @param high_speed false 时强制全速（PORTSC1.PFSC）/ false forces full speed
   *        (PORTSC1.PFSC)
   * @param rx_ep_cfgs 各 OUT 端点的槽内存和槽数，第一项属于 EP0，下标为端点号
   *        Slot memory and slot count of each OUT endpoint, the first one EP0's,
   *        indexed by number
   * @param tx_ep_cfgs 各 IN 端点的槽内存和槽数，同上 / Slot memory and slot count of
   *        each IN endpoint, likewise
   * @note 调用者：HPMUSBDevice 的构造，在控制器中断使能之前。地址为 nullptr 的项表示
   *       该端点号没有这个方向。
   *       Caller: the constructor of HPMUSBDevice, before the controller interrupt is
   *       enabled. An entry with a null address means the number lacks that direction.
   */
  HPMUSBController(USB_Type* regs, uint32_t irq, bool high_speed,
                   const std::initializer_list<HPMUSBEndpointMemory>& rx_ep_cfgs,
                   const std::initializer_list<HPMUSBEndpointMemory>& tx_ep_cfgs);

  /// 端点号和方向在队列头表中的下标 / Index of a number and direction in the dQH list
  static uint8_t QueueIndex(uint8_t num, bool in) { return num * 2U + (in ? 1U : 0U); }
  /// 端点号和方向在 ENDPTPRIME 等寄存器中的位 / Bit of a number and direction in
  /// ENDPTPRIME and the like
  static uint32_t EndpointBit(uint8_t num, bool in)
  {
    return 1UL << (num + (in ? IN_SHIFT : 0U));
  }

  /**
   * @brief 所有端点同时握手：清事件、等装入结束、全部 FLUSH / Handshake on every
   *        endpoint: clear events, wait for primes, flush everything
   * @return 握手在 HANDSHAKE_US 内完成 / The handshake finished within HANDSHAKE_US
   * @note 只在总线上没有包时调用（总线复位期间、断开之后），此时上限由芯片决定：实测复位
   *       期间 8.4 us、挂起中 0.73 us（见 cpp）。
   *       Called only with no packet on the bus (during a bus reset, after a detach),
   *       when the bound is the chip's: measured 8.4 us during a reset and 0.73 us while
   *       suspended (see the cpp).
   */
  bool FlushAll();

  /// 未结束的停止的检查周期（us）/ Check period of pending stops (us)
  static constexpr uint32_t POLL_US = 100U;
  /**
   * @brief 启动控制器自带的 GPTIMER0（重复，POLL_US）/ Start the controller's own
   *        GPTIMER0 (repeating, POLL_US)
   * @note 第一个停止转为未结束时调用；它的中断就是本控制器的中断，owner 在里面推进停止。
   *       挂起时没有 SOF，只有它的中断。
   *       Called when the first stop becomes pending; its interrupt is this
   *       controller's, in which the owner advances the stops. While suspended there is
   *       no SOF and its interrupt is the only one.
   */
  void StartPollTimer();
  /// 停止 GPTIMER0 并清 TI0；最后一个停止结束时调用 / Stop GPTIMER0 and clear TI0;
  /// called when the last stop ends
  void StopPollTimer();

  /// 本对象的队列头表；2048 字节对齐（ENDPTLISTADDR）/ This object's dQH list; 2048-byte
  /// aligned (ENDPTLISTADDR)
  alignas(2048) QueueHead qh_[MAX_ENDPOINTS * 2U] = {};
  /// dTD 池的大小：32 个端点方向各两个，共 2 KiB / Size of the dTD pool: two for each
  /// of the 32 endpoint directions, 2 KiB
  static constexpr uint8_t TD_POOL = MAX_ENDPOINTS * 2U * 2U;
  /**
   * @brief dTD 池；每个端点构造时按槽数连续取一段 / dTD pool; every endpoint takes a
   *        run of its slot count at construction
   * @note 只为存在的端点方向取，所以各端点槽数之和可到 TD_POOL（超出时构造失败）。
   *       Taken only for the endpoint directions that exist, so the slot counts may add
   *       up to TD_POOL (construction fails beyond it).
   */
  TransferDescriptor td_[TD_POOL] = {};
  uint8_t td_used_ = 0U;  ///< 已取走的 dTD 个数；仅构造 / dTDs taken; construction only
  USB_Type* const regs_;  ///< 寄存器块；构造时写 / Register block
  const uint32_t irq_;    ///< 中断号；Mask/Unmask/Pend 读 / Interrupt number
  /// IN 端点对象，下标为端点号；构造时写，之后只读 / IN endpoint objects by number;
  /// written at construction, read-only afterwards
  HPMUSBEndpoint* in_[MAX_ENDPOINTS] = {};
  /// OUT 端点对象，下标为端点号；构造时写，之后只读 / OUT endpoint objects by number;
  /// written at construction, read-only afterwards
  HPMUSBEndpoint* out_[MAX_ENDPOINTS] = {};
  /// 停止未结束的端点数；仅 owner / Endpoints whose stop is pending; owner only
  uint8_t pending_stops_ = 0U;

 private:
  friend class HPMUSBEndpoint;
};

/**
 * @class HPMUSBDevice
 * @brief HPM USB 设备 / HPM USB device
 *
 * 实现设备服务的四个入口：ServiceHardware、MaskInterrupt、UnmaskInterrupt、
 * PendInterrupt。PendInterrupt 置 PLIC 中本控制器的挂起位，所以 owner 总在 USB 中断里
 * 运行；屏蔽只用 PLIC 中本线的使能位，不关全部中断。停止见 HPMUSBEndpoint。
 * Implements the four entries of the device service: ServiceHardware, MaskInterrupt,
 * UnmaskInterrupt, PendInterrupt. PendInterrupt sets this controller's pending bit in
 * the PLIC, so the owner always runs in the USB interrupt; masking uses only this line's
 * enable bit in the PLIC and never disables all interrupts. For stops see
 * HPMUSBEndpoint.
 *
 * @note 总线复位在 URI 时收尾（结束未结束的停止、清端点），在端口回到工作状态（PCI，
 *       PORTSC1.PR 为 0）时按 PORTSC1.PSPD 报告速度；挂起之后的 PCI 报告为恢复。帧起始用
 *       SOF 中断（USBSTS.SRI），在总线运行时一直打开，帧号读 FRINDEX；清 SRI 只写这一位，
 *       不影响别的未读事件。选 SOF 中断而不是只读 FRINDEX，是因为等时类（UAC）按
 *       微帧调度；它也给未结束的停止一个约每微帧一次的检查机会。没有 VBUS 检测，断开
 *       表现为挂起。
 *       A bus reset is wound up at URI (pending stops ended, endpoints cleared) and the
 *       speed from PORTSC1.PSPD is reported when the port returns to operation (PCI
 *       with PORTSC1.PR at 0); a PCI after a suspend is reported as a resume. Starts of
 *       frame come from the SOF interrupt (USBSTS.SRI), kept on while the bus runs, the
 *       number from FRINDEX; clearing SRI writes that bit only and leaves every other
 *       unread event alone. The SOF interrupt is chosen over reading FRINDEX alone
 *       because isochronous classes (UAC) pace by microframe, and it gives pending stops
 *       a check about once per microframe too. There is no VBUS detection; a disconnect
 *       shows as a suspend.
 */
class HPMUSBDevice : public HPMUSBController,
                     public USB::DeviceService,
                     public USB::DeviceCore
{
 public:
  /// 设备类列表 / Class lists of the configurations
  using Configs = std::initializer_list<const std::initializer_list<USB::DeviceClass*>>;
  /// 字符串语言包 / String language packs
  using Languages = std::initializer_list<const USB::DescriptorStrings::LanguagePack*>;

  /**
   * @brief 构造设备、端点和设备核心 / Construct the device, its endpoints and the core
   * @param regs USB 寄存器块（HPM_USB0）/ USB register block (HPM_USB0)
   * @param irq 中断号（IRQn_USB0）/ Interrupt number (IRQn_USB0)
   * @param high_speed 以高速枚举；false 强制全速 / Enumerate at high speed; false forces
   *        full speed
   * @param rx_ep_cfgs OUT 槽内存和槽数，下标为端点号（EP0 在首位）；只写内存时为默认
   *        槽数 / OUT slot memory and slot count indexed by endpoint number (EP0 first);
   *        the memory alone gives the default count
   * @param tx_ep_cfgs IN 槽内存和槽数，同上 / IN slot memory and slot count, likewise
   * @param vid 厂商 ID / Vendor ID
   * @param pid 产品 ID / Product ID
   * @param bcd bcdDevice
   * @param languages 字符串语言包 / String language packs
   * @param configs 每个配置的类列表 / Class list of each configuration
   * @param uid 序列号字符串后缀，可为空 / Serial-number suffix, may be empty
   * @note EP0 包长固定为 64。调用者：应用，初始化时一次；对象放在 DLM。
   *       The EP0 packet size is 64. Caller: the application, once at init; the object
   *       lives in the DLM.
   */
  HPMUSBDevice(USB_Type* regs, uint32_t irq, bool high_speed,
               const std::initializer_list<HPMUSBEndpointMemory>& rx_ep_cfgs,
               const std::initializer_list<HPMUSBEndpointMemory>& tx_ep_cfgs,
               uint16_t vid, uint16_t pid, uint16_t bcd, const Languages& languages,
               const Configs& configs, ConstRawData uid = {nullptr, 0});

  /**
   * @brief 中断入口 / Interrupt entry
   * @param regs 触发中断的 USB 寄存器块 / USB register block whose IRQ fired
   * @note 调用者：USB 控制器的中断函数（本驱动定义）。设备已构造且寄存器块匹配时进入
   *       OnControllerInterrupt，否则关掉该控制器的中断源。
   *       Caller: the USB interrupt function (defined by this driver). Enters
   *       OnControllerInterrupt when matched, otherwise switches USBINTR off.
   */
  static void Dispatch(USB_Type* regs = HPM_USB0);

  /**
   * @brief 打开中断并接上总线 / Enable interrupts and attach to the bus
   * @note 调用者：应用，线程中。在 owner 之外写 USBSTS、USBINTR 和 USBCMD，
   *       期间屏蔽本中断线；owner 总在本中断里运行，所以这期间没有 owner 访问寄存器。
   *       Caller: the application, in a thread. Writes USBSTS, USBINTR and USBCMD
   *       outside the owner with this line masked; the owner always runs in this
   *       interrupt, so no owner touches the registers meanwhile.
   */
  void StartController(bool in_isr) override;

  /**
   * @brief 断开总线并让控制器静止 / Detach from the bus and make the controller
   *        quiescent
   * @note 仅 owner，见 DeviceCore::StopController。与 ResetController 的前半相同，只是不
   *       再接上总线，并关掉控制器的中断源。
   *       Owner only, see DeviceCore::StopController. The same as the first half of
   *       ResetController, except that the bus is not attached again and the
   *       controller's interrupt sources are switched off.
   */
  void StopController(bool in_isr) override;

 protected:
  /// @name 设备服务的入口 / Entries of the device service
  /// @{
  /// 按固定顺序处理一次状态快照，见 cpp / Handle one status snapshot in a fixed order,
  /// see the cpp
  void ServiceHardware(bool in_isr) override;
  /// 清 PLIC 中本线的使能位 / Clear this line's enable bit in the PLIC
  void MaskInterrupt() override;
  /// 置 PLIC 中本线的使能位 / Set this line's enable bit in the PLIC
  void UnmaskInterrupt() override;
  /// 置 PLIC 中本线的挂起位，返回 true / Set this line's pending bit in the PLIC; true
  bool PendInterrupt() override;
  /// @}

  /// @name 设备核心的后端钩子（owner）/ Backend hooks of the device core (owner)
  /// @{
  /**
   * @brief 在 BEFORE_STATUS 以 USBADRA 写地址 / Write the address with USBADRA at
   *        BEFORE_STATUS
   * @note USBADRA 让控制器在状态 IN 被 ACK 之后才采用新地址（SDK hpm_usb_regs.h
   *       DEVICEADDR）。
   *       USBADRA makes the controller take the new address only after the status IN is
   *       ACKed (DEVICEADDR in the SDK's hpm_usb_regs.h).
   */
  void SetAddress(uint8_t address, AddressStage stage) override;
  /**
   * @brief 断开、静止，再接上总线 / Detach, quiesce and attach again
   * @note 返回后硬件上没有任何请求，之后每个 StopHardware 立即返回 ABORTED。只做前半段
   *       （HaltController、detach_.OnHalted(true)），不等待；断开保持 RESET_DETACH_MS
   *       由 detach_ 按节拍计时，到点后 owner 接上，主机才会复位总线。
   *       Afterwards no request is on the hardware, and every later StopHardware returns
   *       ABORTED at once. First half only (HaltController, detach_.OnHalted(true)), no
   *       wait; the detach lasts RESET_DETACH_MS, counted on the tick by detach_, and the
   *       owner attaches once it has passed, so that the host resets the bus.
   */
  void ResetController(bool in_isr) override;
  /// @}

 private:
  /// 最短断开时长（毫秒，节拍计数）/ Minimum detach time (ms, counted in ticks)
  static constexpr uint32_t RESET_DETACH_MS = 10U;

  /**
   * @brief 断开总线，让硬件不再为任何端点工作，作废未结束的停止 / Detach, make the
   *        hardware stop working for every endpoint and void the pending stops
   * @note StopController 和 ResetController 共用。
   *       Shared by StopController and ResetController.
   */
  void HaltController();
  /// 清状态、打开中断源、接上总线；调用者：StartController、ResetController、
  /// ServiceHardware（到点的接上）/ Clear the status, enable the interrupt sources and
  /// attach; callers: StartController, ResetController, ServiceHardware (a due attach)
  void Attach();
  /// 总线复位的收尾（URI）/ Wind-up of a bus reset (URI)
  void OnUsbReset(bool in_isr);
  /**
   * @brief 按 dTD 令牌报告硬件上有请求的各端点的完成，同号先 IN 后 OUT / Report the
   *        completions of every endpoint with requests on the hardware by the dTD tokens,
   *        IN before OUT for each number
   */
  void HandleCompletions(bool in_isr);
  /// 报告复位：按 PORTSC1.PSPD 给速度 / Report the reset with the speed of PORTSC1.PSPD
  void ReportReset(bool in_isr);
  /// 报告帧起始（FRINDEX）/ Report the start of frame (FRINDEX)
  void ReportFrame(bool in_isr);
  /**
   * @brief 报告已锁存的 SETUP / Report a latched SETUP
   * @note EP0 上未结束的停止要先结束；它还在等 FLUSH 时本次不报告，留到之后的检查（SETUP
   *       因此晚一些报告），见 HPMUSBEndpoint。
   *       Pending stops on EP0 end first; while one still waits for its FLUSH the SETUP
   *       is not reported in this pass but left to a later check (so it is reported
   *       somewhat later), see HPMUSBEndpoint.
   */
  void HandleSetup(bool in_isr);
  /**
   * @brief 总线复位时当场结束所有未结束的停止 / End every pending stop now at a bus
   *        reset
   * @note 复位期间总线上没有包，FlushAll 由芯片定上限；仍不成功的报 TIMEOUT。
   *       No packet is on the bus during a reset, so FlushAll is bounded by the chip; a
   *       stop that still fails is reported as TIMEOUT.
   */
  void FinishPendingStops(bool in_isr);
  /// 推进未结束的停止：查 FLUSH、按时间重发或报 TIMEOUT / Advance pending stops: check
  /// the FLUSH, flush again or report TIMEOUT by time
  void AdvanceStops(bool in_isr);

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
   *       节拍来自系统层（FreeRTOS、ThreadX 的包装）；裸机时板级工程每毫秒调用
   *       DriverAsyncCondition::OnTick，见 HPMTimebase。
   *       The tick comes from the system layer (the FreeRTOS or ThreadX wrap);
   *       bare-metal, the board project calls DriverAsyncCondition::OnTick every
   *       millisecond, see HPMTimebase.
   */
  USB::DetachTimer detach_{*this, RESET_DETACH_MS};
  /// URI 之后还没有报告复位 / A URI was seen and the reset is not reported yet
  bool reset_pending_ = false;
  /// 已向核心报告挂起、还没有报告恢复 / Suspend reported to the core, resume not yet
  bool suspended_ = false;

  /// 已构造的设备，中断入口读 / The constructed device, read by the interrupt entry
  static inline HPMUSBDevice* device_ = nullptr;
};

/**
 * @class HPMUSBEndpoint
 * @brief HPM USB 端点，停止不等总线 / HPM USB endpoint whose stop never waits for the
 *        bus
 *
 * 一个请求一个 dTD（最多 MAX_BYTES 字节），控制器 DMA 直接读写槽（direct_memory）。
 * 非控制端点硬件上最多挂槽数个请求（HwLimits::max_requests 等于槽数，槽数由
 * HPMUSBEndpointMemory 给出，默认 DEFAULT_SLOTS），每个槽一个 dTD，按环轮流用。
 * 空端点装入：写 dTD 和 dQH 覆盖区（清 Active/Halted，next 指向 dTD），清本端点的
 * ENDPTCOMPLETE 位，再写 ENDPTPRIME。更早的请求还在硬件上时追加：新 dTD 接在最后一个
 * 后面，按 ATDTW 步骤确认控制器还在这条链上；链已经停了（更早的都已退役）就按空端点
 * 重新装入。完成：不看 ENDPTCOMPLETE，从最早的起按先后逐个看内存里 dTD 令牌的
 * Active 位和剩余字节；OUT 的短包让它所在的 dTD 退役，下一包进后面那个 dTD。
 * One dTD per request (at most MAX_BYTES bytes); the controller's DMA reads and writes
 * the slot itself (direct_memory). A non-control endpoint holds up to as many requests
 * on the hardware as it has slots (HwLimits::max_requests is the slot count, given by
 * HPMUSBEndpointMemory, DEFAULT_SLOTS by default), one dTD per slot used as a ring.
 * Prime on an empty endpoint: write the dTD and the dQH overlay (Active and Halted
 * cleared, next pointing at the dTD), clear this endpoint's ENDPTCOMPLETE bit, then
 * write ENDPTPRIME.
 * Append while earlier requests are on the hardware: link the new dTD behind the last
 * one and confirm with the ATDTW procedure that the controller is still on the list; if
 * the list has stopped (the earlier ones all retired) prime it as on an empty endpoint.
 * Completion: ENDPTCOMPLETE is not read; the dTD tokens in memory are read in order
 * from the oldest for the Active bit and the bytes left; a short OUT packet retires the
 * dTD it landed in and the next packet goes to the dTD behind it.
 *
 * @note 停止，对应 Endpoint::StopHardware 的规则。
 *       手段：数据端点写 ENDPTFLUSH。EP0 在 SETUP 之后不写 FLUSH：控制器收到 SETUP 时
 *       自己清掉 EP0 的 ENDPTSTAT（实测 SETUPSTAT 可见后不超过 0.47 us，22 200 次），
 *       不回写 dTD；而 FLUSH 要等别的端点线上的包，实测 HS 批量下最长 102 us、FS 等时下
 *       682 us。
 *       停稳的证据：本端点的 ENDPTFLUSH 位自清且 ENDPTSTAT 位为 0（手册语义：FLUSH 让
 *       在途的那一包做完后才清位；实测之后 10 800 次停止没有任何内存写入和完成）；被停
 *       请求自己的 dTD 退役（Active 为 0）；EP0 的 SETUP（ENDPTSTAT 被硬件清零）。总线
 *       空闲时 FLUSH 在调用内完成（实测不超过 1.1 us），所以调用里读 HANDSHAKE_SPINS
 *       次；否则 PENDING，每个 SOF、控制器 GPTIMER0（每 POLL_US）和其他中断进来时
 *       再查。
 *       协议上界在这颗芯片上不能单独作证据：FLUSH 写下之后控制器继续回应 IN 令牌，HS 下
 *       又发出最多 12 包、118 us（实测），即写下停止不等于不再开始新包；FLUSH 自清才是
 *       停稳。超过 STOP_REFLUSH_US 仍未自清就再写一次 FLUSH，超过 STOP_TIMEOUT_US 再查
 *       一次仍不行才报 TIMEOUT；两个上限都远大于实测最长 682 us。
 *       多个请求同时在硬件上时，FLUSH 一次停掉全部。各自的结果：dTD 已退役按它的令牌；
 *       覆盖区正装着它（当前 dTD 是它）按覆盖区的剩余字节；都不是为 ABORTED、0。调用内
 *       停稳时返回第一个的结果，核心随后为其余每个请求各调一次，依次拿后面的；PENDING
 *       时停稳后按先后每个请求报告一次（TIMEOUT 只报一次）。
 *       缺陷一，完成事件被合并或改写：有。ENDPTCOMPLETE 每端点一位，同一端点上几个 dTD
 *       先后退役会并成一次。所以不靠它：每轮把硬件上有请求的端点都按 dTD 令牌（按
 *       请求保存在内存里的状态）查一遍，逐个报告。
 *       缺陷二，软件写寄存器抹掉硬件的更新：ENDPTPRIME/ENDPTFLUSH 写 1 置位、
 *       ENDPTCOMPLETE/ENDPTSETUPSTAT 写 1 清零，只影响写的那一位，不会抹掉别的端点；
 *       dQH 覆盖区只在本端点没有装入（ENDPTSTAT 为 0，停稳之后）时写。会出问题的是 EP0
 *       的 ENDPTCTRL0：TXS/RXS 在 SETUPSTAT 置位期间和清零后约 50 个时钟内由硬件清零，
 *       读-改-写或刚清 SETUPSTAT 后写的 STALL 会丢或被写回。做法：置 STALL 后读回，没
 *       生效就重写，直到生效或又来了 SETUP（SDK 寄存器说明给的步骤）；清 STALL 时显式
 *       清两位，被写回的 STALL 由核心在下一个 SETUP 的 ClearStall 清掉。
 *       缺陷三，停稳的事件会丢：FLUSH 自清之后 ENDPTSTAT 仍为 1（装入与 FLUSH 相撞）时
 *       重写 FLUSH（手册步骤）；FLUSH 一直不清时按时间重写一次，之后才 TIMEOUT。实测
 *       FLUSH 从不需要重写（10 800 次）。
 *       缺陷四，停止影响别的端点：FLUSH 只清本端点；别的端点不受影响，只是 FLUSH 自清要
 *       等别的端点线上的包。OUT 的 FLUSH 会丢掉已经 ACK、还在控制器接收缓冲里的包（实测
 *       约 1% 的停止各丢 1 包），这一包属于被停的请求，被停的 OUT 请求按契约整体以
 *       ABORTED 结束，所以不另算。
 *       实测（HPM5301EVKLite，高速，主机 Intel xHCI，两个请求）：EP1 IN、OUT 批量交替
 *       停止，每方向约 3.2 万次，99% 的停止时硬件上有两个请求；IN 丢 0、OUT 越出被停
 *       请求的丢失 0、相位错 0；PENDING 约 1/5，最长 471 us，TIMEOUT 0，FLUSH 重写 0。
 *       PENDING 的 IN 停止由核心当场按 actual 0 交付，停稳前硬件已发出的数据（8 358 次、
 *       102 MB）不再报告，按序续发的类会重复这些数据。OUT 短包结束的请求连续 7.3 万次
 *       长度和内容都对。停止压力下约 1/6 的追加碰上链已停、重新装入，ATDTW 最多
 *       重来 4 次。等时 OUT 每微帧 1024 B：owner 一轮最长 195 us 时 0 丢（一个请求
 *       时同样负载丢 4.9%）。
 *       Stops, against the rules of Endpoint::StopHardware.
 *       Means: data endpoints write ENDPTFLUSH. EP0 after a SETUP writes no FLUSH: the
 *       controller clears EP0's ENDPTSTAT itself when a SETUP arrives (measured at most
 *       0.47 us after SETUPSTAT shows, 22,200 times) and does not write the dTD back,
 *       while a FLUSH waits for other endpoints' packets on the wire, measured up to
 *       102 us under HS bulk and 682 us under FS isochronous traffic.
 *       Evidence of quiescence: this endpoint's ENDPTFLUSH bit cleared by itself with its
 *       ENDPTSTAT bit at 0 (a FLUSH lets the packet in progress finish before the bit
 *       clears; measured, no memory write and no completion followed in 10,800 stops);
 *       the stopped request's own dTD retiring (Active 0); a SETUP on EP0 (ENDPTSTAT
 *       cleared by the hardware). On an idle bus the FLUSH finishes within the call
 *       (measured at most 1.1 us), so the call reads HANDSHAKE_SPINS times; otherwise
 *       PENDING, and it is checked again in every SOF, the controller's GPTIMER0 (every
 *       POLL_US) and any other interrupt.
 *       The protocol bound alone is no evidence on this chip: after the FLUSH is written
 *       the controller keeps answering IN tokens, at HS up to 12 more packets in 118 us
 *       (measured), so a written stop does not mean no new packet starts; the FLUSH
 *       clearing itself is the quiescence. A FLUSH not cleared after STOP_REFLUSH_US is
 *       written once more, and TIMEOUT is reported only after STOP_TIMEOUT_US and one
 *       more check; both bounds are far above the longest measured 682 us.
 *       With several requests on the hardware one FLUSH stops them all. Each one's
 *       outcome: from its token if its dTD retired; from the overlay's bytes left if the
 *       overlay holds it (it is the current dTD); else ABORTED with 0. When quiescent
 *       within the call the first outcome is returned and the core's further calls, one
 *       per further request, take the others in turn; with PENDING each request is
 *       reported once, in order, once quiescent (TIMEOUT only once).
 *       Flaw one, completion events merge or are rewritten: present. ENDPTCOMPLETE has
 *       one bit per endpoint, and several dTDs of one endpoint retiring in turn merge
 *       into one. It is therefore not relied on: every pass checks each endpoint with
 *       requests on the hardware by its dTD tokens (state kept per request in memory)
 *       and reports them one by one.
 *       Flaw two, a software write undoes a hardware update: ENDPTPRIME and ENDPTFLUSH
 *       set and ENDPTCOMPLETE and ENDPTSETUPSTAT clear on writing 1, touching the
 *       written bit only, so no other endpoint is undone; the dQH overlay is written
 *       only while this endpoint holds nothing (ENDPTSTAT 0, after quiescence). What can
 *       go wrong is EP0's ENDPTCTRL0: the hardware clears TXS and RXS while SETUPSTAT is
 *       set and for about 50 clocks after it is cleared, so a STALL written by
 *       read-modify-write or right after SETUPSTAT is cleared can be lost or written
 *       back. A STALL is read back and written again until it holds or a new SETUP has
 *       come (the procedure in the SDK register notes); clearing writes both bits
 *       explicitly, and a STALL written back is cleared by the core's ClearStall at the
 *       next SETUP.
 *       Flaw three, the quiescence event can be lost: when ENDPTSTAT is still 1 after
 *       the FLUSH cleared (a prime met the FLUSH) the FLUSH is written again (the manual
 *       procedure); a FLUSH that never clears is written once more by time, and only
 *       then TIMEOUT. Measured, the FLUSH never needed a second write (10,800 times).
 *       Flaw four, a stop affects other endpoints: a FLUSH clears this endpoint only;
 *       others are not affected, the FLUSH only waits for their packets on the wire to
 *       clear itself. An OUT FLUSH drops a packet already ACKed and still in the
 *       controller's receive buffer (measured in about 1 % of stops, one packet each);
 *       that packet belongs to the stopped request, which by contract ends as a whole
 *       with ABORTED, so it is not counted apart.
 *       Measured (HPM5301EVKLite, high speed, Intel xHCI host, two requests): EP1 IN and
 *       OUT bulk stopped in turn, about 32,000 times per direction, two requests on the
 *       hardware at 99 % of the stops: 0 IN lost, 0 OUT lost beyond the stopped
 *       requests, 0 toggle mismatches; about 1 in 5 PENDING, longest 471 us, 0 TIMEOUT,
 *       0 FLUSH rewrites. The core delivers a PENDING IN stop at once with actual 0, and
 *       the data the hardware sent before quiescence (8,358 stops, 102 MB) is not
 *       reported any more, so a class that resends by sequence repeats it. OUT requests
 *       ended by a short packet: 73,000 in a row with the right length and bytes.
 *       Under the stop load about 1 in 6 appends found the list stopped and primed
 *       again; ATDTW took at most 4 retries. Isochronous OUT at 1024 B per microframe:
 *       0 lost with owner passes of up to 195 us (4.9 % lost under the same load with
 *       one request).
 */
class HPMUSBEndpoint : public USB::Endpoint
{
 public:
  /// 与总线无关的寄存器握手最多读这么多次 / Register reads a bus-independent handshake
  /// may take at most
  static constexpr uint32_t HANDSHAKE_SPINS = 64U;
  /// 一次请求的字节上限：5 个页指针从任意偏移起至少覆盖 16 KiB / Bytes of one request:
  /// 5 page pointers cover at least 16 KiB from any offset
  static constexpr size_t MAX_BYTES = 16384U;
  /// 等时端点一个服务区间最多几个包（dQH MULT）/ Packets per service interval of an
  /// isochronous endpoint (dQH MULT)
  static constexpr uint8_t ISO_PACKETS_PER_INTERVAL = 3U;
  /// 中断端点一个服务区间最多几个包 / Packets per service interval of an interrupt
  /// endpoint
  static constexpr uint8_t INTR_PACKETS_PER_INTERVAL = 3U;
  /// 没给槽数时非控制端点的槽数（EP0 总是一个）/ Slots of a non-control endpoint when
  /// none is given (EP0 always has one)
  static constexpr uint8_t DEFAULT_SLOTS = 2U;
  /// FLUSH 未自清时再写一次的时间（us）/ Time after which a FLUSH that has not cleared
  /// is written again (us)
  static constexpr uint64_t STOP_REFLUSH_US = 2000U;
  /// 未结束的停止报 TIMEOUT 的时间（us）/ Time after which a pending stop is reported
  /// as TIMEOUT (us)
  static constexpr uint64_t STOP_TIMEOUT_US = 20000U;

  /**
   * @brief 构造并登记到控制器 / Construct and enter into the controller
   * @note 调用者：HPMUSBController 构造，每个端点一次。槽数为 0 时 EP0 取一个、其余取
   *       DEFAULT_SLOTS，换算后的槽数交给核心；从控制器的 dTD 池取槽数个 dTD。
   *       Caller: the HPMUSBController constructor, once per endpoint. A slot count of
   *       0 gives EP0 one and the others DEFAULT_SLOTS, and the resolved count is passed
   *       to the core; takes that many dTDs from the controller's pool.
   */
  HPMUSBEndpoint(HPMUSBController& controller, EPNumber number, Direction direction,
                 const HPMUSBEndpointMemory& memory);

 protected:
  /**
   * @brief 控制类型只在 EP0 / The control type on EP0 only
   * @note 同号两个方向的类型字段各自独立（ENDPTCTRL 的 TXT/RXT），无需比较。
   *       The two directions of a number have separate type fields (TXT and RXT of
   *       ENDPTCTRL), so nothing is compared.
   */
  ErrorCode CheckHardware(const Config& cfg) const override;
  /**
   * @brief 写 dQH 能力字、类型、使能和翻转位复位 / Write the dQH capabilities, the type,
   *        the enable and the toggle reset
   * @note 同号另一方向还没配置时把它的类型写成批量：手册要求非控制端点两个方向的类型都不
   *       是控制（SDK hpm_usb_regs.h ENDPTCTRL TXT/RXT）。
   *       While the other direction of the number is unconfigured its type is written as
   *       bulk: both types of a non-control endpoint must be non-control (ENDPTCTRL
   *       TXT/RXT in the SDK's hpm_usb_regs.h).
   */
  void ConfigureHardware(const Config& cfg) override;
  /**
   * @brief 清 STALL，端点保持使能、没有装入，所以回 NAK / Clear the STALL; the endpoint
   *        stays enabled with nothing primed, so it NAKs
   * @note 不清使能：使能清掉的端点对令牌不应答，主机记事务错误（实测）。
   *       The enable is kept: an endpoint with its enable cleared does not answer tokens
   *       and the host counts transaction errors (measured).
   */
  void CloseHardware(bool in_isr) override;
  ErrorCode StartHardware(const HwRequest& request, bool in_isr) override;
  StopOutcome StopHardware(bool in_isr) override;
  ErrorCode StallHardware(bool in_isr) override;
  /// 清 STALL；数据端点同时复位翻转位到 DATA0 / Clear the STALL; data endpoints also
  /// reset the toggle to DATA0
  ErrorCode ClearStallHardware(bool in_isr) override;
  /**
   * @brief 硬件上限 / Hardware limits
   * @note 控制一次一个包、64 字节。等时一个请求就是一个服务区间：高速最多 3 个
   *       1024 字节的包，dQH MULT 写选定的包数；IN 的 dTD 另写 MultO = 实际包数；OUT
   *       每个（微）帧结束一个 dTD，不论收到几个包（实测）。批量和中断一次最多
   *       MAX_BYTES；高速中断一个区间最多 3 个包，靠主机按描述符轮询，dQH MULT 保持
   *       0（SDK 要求非等时为 0），实测 3 个包时约 24.4 MB/s。
   *       Control: one packet of 64 bytes. An isochronous request is one service
   *       interval: at high speed up to 3 packets of 1024 bytes, the dQH MULT holds the
   *       chosen count, and an IN dTD also gets MultO = the actual packets; OUT retires
   *       one dTD per (micro)frame whatever number of packets arrived (measured). Bulk
   *       and interrupt take up to MAX_BYTES per request; a high-speed interrupt
   *       endpoint takes up to 3 packets per interval because the host polls by the
   *       descriptor, with the dQH MULT left 0 (the SDK requires 0 for non-isochronous);
   *       about 24.4 MB/s measured with 3 packets.
   */
  HwLimits HardwareLimits(Direction direction, Type type) const override;

 private:
  friend class HPMUSBDevice;
  friend class HPMUSBController;

  bool IsIn() const { return AvailableDirection() == Direction::IN; }
  uint8_t Num() const { return EPNumberToInt8(GetNumber()); }
  uint8_t Index() const { return HPMUSBController::QueueIndex(Num(), IsIn()); }
  uint32_t Bit() const { return HPMUSBController::EndpointBit(Num(), IsIn()); }
  HPMUSBController::QueueHead& Qh() const { return controller_.qh_[Index()]; }
  HPMUSBController::TransferDescriptor& Td(uint8_t which) const { return td_[which]; }
  /// 环上 which 之后第 step 个 dTD / The dTD step places after which on the ring
  uint8_t RingAt(uint8_t which, uint8_t step) const
  {
    return static_cast<uint8_t>((which + step) % slots_);
  }
  USB_Type* Regs() const { return controller_.regs_; }

  /// 本方向在 ENDPTCTRL 中的位偏移（IN 为 16）/ Bit offset of this direction in
  /// ENDPTCTRL (16 for IN)
  uint32_t CtrlShift() const { return IsIn() ? 16U : 0U; }
  /// 写 STALL 直到生效；EP0 遇到新 SETUP 时放弃 / Write the STALL until it holds; EP0
  /// gives up when a new SETUP arrives
  void WriteStall();
  /// 覆盖区复位：清 Active/Halted，next 结束 / Reset the overlay: Active and Halted
  /// cleared, next terminated
  void ResetOverlay();
  /// 本端点是否已停稳：FLUSH 位和 STAT 位都为 0 / Whether this endpoint is quiescent:
  /// both its FLUSH and its STAT bits are 0
  bool Quiescent() const;
  /// 读 HANDSHAKE_SPINS 次等停稳 / Read HANDSHAKE_SPINS times for quiescence
  bool SpinQuiescent() const;
  /**
   * @brief 硬件上的一个请求停稳之后的结果 / Outcome of one request on the hardware once
   *        quiescent
   * @param which 它的 dTD（环上的下标）/ Its dTD (index on the ring)
   * @note dTD 已退役按它的令牌；否则只在覆盖区正装着它时按覆盖区的剩余字节：全部传完为
   *       OK，否则 ABORTED；覆盖区没装着它为 ABORTED、0。
   *       From the dTD token if it retired; otherwise from the overlay's bytes left only
   *       while the overlay holds it: OK if everything moved, else ABORTED; ABORTED with
   *       0 if the overlay does not hold it.
   */
  StopOutcome OutcomeOf(uint8_t which) const;
  /// 按先后取硬件上每个请求的结果，返回个数 / The outcomes of the requests on the
  /// hardware in order; returns how many
  uint8_t TakeOutcomes(StopOutcome (&outcomes)[MAX_SLOTS]) const;
  /// 硬件上没有请求了：清计数、覆盖区复位、结束未结束的停止 / No request on the hardware
  /// any more: clear the count, reset the overlay, end a pending stop
  void ClearRequests();
  /// 空端点装入：写覆盖区、清完成位、写 ENDPTPRIME / Prime an empty endpoint: write the
  /// overlay, clear the completion bit, write ENDPTPRIME
  void Prime(HPMUSBController::TransferDescriptor& td);
  /// 更早的请求还在硬件上时按 ATDTW 步骤追加到最后一个之后 / Append behind the last one
  /// with the ATDTW procedure while earlier requests are on the hardware
  void Append(HPMUSBController::TransferDescriptor& td);
  /// 按先后报告已退役的 dTD，遇到还在活动的就停 / Report the retired dTDs in order,
  /// stopping at one still active
  void Complete(bool in_isr);
  /// 停止转为未结束 / The stop becomes pending
  void BeginPending();
  /// 停止不再未结束 / The stop is no longer pending
  void EndPending();
  /// 停稳了就结束未结束的停止并报告；返回是否结束 / End and report a pending stop if
  /// quiescent; returns whether it ended
  bool FinishStop(bool in_isr);

  HPMUSBController& controller_;  ///< 所属控制器 / Owning controller
  /// 本端点的 dTD，槽数个，用作环；构造时从池里取 / This endpoint's dTDs, one per slot,
  /// used as a ring; taken from the pool at construction
  HPMUSBController::TransferDescriptor* td_ = nullptr;
  uint8_t slots_ = 0U;  ///< 槽数（默认已换算）/ Slot count (default resolved)
  /// 槽数：给了就用，没给时 EP0 一个、其余 DEFAULT_SLOTS / The slot count: as given, or
  /// one for EP0 and DEFAULT_SLOTS for the others
  static uint8_t SlotsFor(EPNumber number, uint8_t slots);

  /// @name 在途请求：仅 owner / Request in flight: owner only
  /// @{
  /**
   * @brief 硬件上本端点的请求数，0 到槽数 / Requests of this endpoint on the hardware, 0
   *        to the slot count
   * @note 写：StartHardware 加一，Complete 逐个减一，ClearRequests 和控制器停止或复位
   *       清零。
   *       Raised by StartHardware, lowered one by one by Complete, cleared by
   *       ClearRequests and by a controller halt or reset.
   */
  uint8_t queued_ = 0U;
  /// 最早那个请求的 dTD（环上的下标）/ The dTD of the earliest request (ring index)
  uint8_t head_ = 0U;
  /**
   * @brief 调用内停掉多个请求时，第一个之后各请求的结果，按先后 / Outcomes of the
   *        requests after the first when one call stopped several, in order
   * @note 那次 StopHardware 返回第一个的结果；核心随后为硬件上的每个后续请求各调一次，
   *       依次从这里取（这些调用不返回 PENDING）。
   *       That StopHardware returned the first outcome; the core then calls once more
   *       for each further request on the hardware, taking these in turn (those calls
   *       never return PENDING).
   */
  StopOutcome further_[MAX_SLOTS - 1U];
  uint8_t further_count_ = 0U;  ///< further_ 里的个数 / Entries in further_
  uint8_t further_next_ = 0U;   ///< 下一次给出的那个 / The one given next
  /// 停止已写下、还没有停稳 / The stop was issued and is not quiescent yet
  bool stopping_ = false;
  /// 这次停止由后端发起（EP0 撤回装入），结束时报 ABORTED / The backend issued this stop
  /// itself (EP0 retracting a load) and reports ABORTED when it ends
  bool self_stop_ = false;
  /// 已为这次停止报告过 TIMEOUT / TIMEOUT was reported for this stop
  bool stop_failed_ = false;
  /// 已按时间重写过 FLUSH / The FLUSH was written again by time
  bool reflushed_ = false;
  /// 停止写下的时刻（us）/ Time the stop was issued (us)
  uint64_t stop_at_us_ = 0U;
  /// @}
};

}  // namespace LibXR

#endif
