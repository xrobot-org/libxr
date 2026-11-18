#pragma once

#include <cstdint>
#include <initializer_list>

#include "ch32_usb.hpp"
#include "ch32_usb_endpoint.hpp"
#include "libxr_def.hpp"
#include "libxr_type.hpp"
#include "usb/core/ep_pool.hpp"
#include "usb/core/usb_service.hpp"
#include "usb/device/dev_core.hpp"

namespace LibXR
{

#if defined(USBFSD) || defined(__DOXYGEN__)

/**
 * @class CH32USBOtgFS
 * @brief CH32 OTG FS 设备/ CH32 OTG FS device
 *
 * 一个控制器一个对象。控制器中断只调用 OnControllerInterrupt；硬件事件在
 * ServiceHardware 里处理，线程的请求挂起本中断，所以 owner 总在 USBFS 中断里。
 * One object per controller. The controller interrupt only calls
 * OnControllerInterrupt; hardware events are handled in ServiceHardware, and a thread's
 * request pends this interrupt, so the owner always runs in the USBFS interrupt.
 */
class CH32USBOtgFS : public USB::DeviceService,
                     private USB::EndpointPool,
                     public USB::DeviceCore
{
 public:
  /// 设备类列表 / Class lists of the configurations
  using Configs = std::initializer_list<const std::initializer_list<USB::DeviceClass*>>;
  /// 字符串语言包 / String language packs
  using Languages = std::initializer_list<const USB::DescriptorStrings::LanguagePack*>;

  /**
   * @brief 构造设备、端点和设备核心 / Construct the device, its endpoints and the core
   * @param rx_ep_cfgs OUT 块，下标为端点号（EP0 在首位），缺席传 {nullptr, 0}
   *        OUT blocks indexed by endpoint number (EP0 first), {nullptr, 0} when absent
   * @param tx_ep_cfgs IN 块，同上 / IN blocks, likewise
   * @param packet_size EP0 包长 / EP0 packet size
   * @param vid 厂商 ID / Vendor ID
   * @param pid 产品 ID / Product ID
   * @param bcd bcdDevice
   * @param languages 字符串语言包 / String language packs
   * @param configs 每个配置的类列表 / Class list of each configuration
   * @param uid 序列号字符串后缀，可为空 / Serial-number suffix, may be empty
   * @note 端点号两个方向都有 = 共享（IN 块必须紧跟在 OUT 块 +64 处，两块各 ≥64 B）；只有
   *       一个方向 = 单向（块 ≥128 B，两个槽各占一半）；EP0 两个方向传同一块（≥64 B）。
   *       调用者：应用，初始化时一次。
   *       A number with both directions is shared (the IN block must sit at OUT +64, each
   *       at least 64 B); with one direction it is single (block at least 128 B, two
   *       slots of half each); both EP0 directions pass the same block (at least 64 B).
   *       Caller: the application, once at init.
   */
  CH32USBOtgFS(const std::initializer_list<RawData>& rx_ep_cfgs,
               const std::initializer_list<RawData>& tx_ep_cfgs,
               USB::DeviceDescriptor::PacketSize0 packet_size, uint16_t vid, uint16_t pid,
               uint16_t bcd, const Languages& languages, const Configs& configs,
               ConstRawData uid = {nullptr, 0});

  /**
   * @brief 开时钟、复位 SIE、使能中断和上拉，最后使能 USBFS 中断线 / Enable the clock,
   *        reset the SIE, enable interrupts and the pull-up, then enable the USBFS line
   * @note 调用者：应用，控制器中断之外。 / Caller: the application, outside the
   *       controller interrupt.
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

  /**
   * @brief 已启动的实例，供中断函数使用 / Started instance used by the interrupt handler
   * @note 写：Start、Stop；读：USBFS_IRQHandler。 / Written by Start and Stop; read by
   *       USBFS_IRQHandler.
   */
  static inline CH32USBOtgFS* self_ = nullptr;

 protected:
  void ServiceHardware(bool in_isr) override;
  /// NVIC_DisableIRQ(USBFS_IRQn)
  void MaskInterrupt() override;
  /// NVIC_EnableIRQ(USBFS_IRQn)
  void UnmaskInterrupt() override;
  /// NVIC_SetPendingIRQ(USBFS_IRQn)，返回 true / NVIC_SetPendingIRQ(USBFS_IRQn), true
  bool PendInterrupt() override;
  /// 状态 IN 完成时写 DEV_ADDR / Writes DEV_ADDR when the status IN has completed
  void SetAddress(uint8_t address, AddressStage stage) override;
  /// EP0 RX 置 ACK / EP0 RX to ACK
  void OnControlReady() override;
  /// 复位 SIE 并重新接上总线 / Reset the SIE and attach again
  void ResetController(bool in_isr) override;

 private:
  /// 建端点、登记到端点池和设备服务 / Build the endpoints, register them with the pool
  /// and the device service
  static USB::EndpointPool& AddEndpoints(CH32USBOtgFS& self,
                                         const std::initializer_list<RawData>& rx,
                                         const std::initializer_list<RawData>& tx);
  /**
   * @brief 断开总线，让硬件不再为任何端点工作，作废未结束的停止 / Detach, make the
   *        hardware stop working for every endpoint and void the pending stops
   * @note StopController 和 ResetController 共用。
   *       Shared by StopController and ResetController.
   */
  static void HaltController();
  /**
   * @brief HaltController 断开之后等够最短断开时间 / Wait until the minimum detach time
   *        has passed since HaltController detached
   * @note StartController 和 ResetController 在重新接上之前调用。
   *       Called by StartController and ResetController before they attach.
   */
  static void WaitDetached();
  /// HaltController 断开总线的时刻（µs）和“断开后还没有重新接上” / Time HaltController
  /// detached (us) and "not attached again since"
  static inline uint64_t halted_at_us_ = 0U;
  static inline bool halted_ = false;
  /// 已向设备核心报告挂起（之后还没有报告恢复或复位）；仅 owner / A suspend was
  /// reported to the device core (no resume or reset since); owner only
  bool suspended_ = false;
  /// 控制器寄存器初始化，Start 和 ResetController 共用 / Controller register set-up
  /// shared by Start and ResetController
  static void SetUpController();
  /// EP0 两个方向 NAK，下一包 DATA1；总线复位和 SETUP 时 / Both EP0 directions NAK,
  /// DATA1 next; on bus reset and SETUP
  static void ResetEp0();
};

#endif  // defined(USBFSD) || defined(__DOXYGEN__)

#if defined(USBHSD) || defined(__DOXYGEN__)

/**
 * @class CH32USBOtgHS
 * @brief CH32 OTG HS 设备/ CH32 OTG HS device
 *
 * 一个控制器一个对象。控制器中断只调用 OnControllerInterrupt；硬件事件在
 * ServiceHardware 里处理，线程的请求挂起本中断，所以 owner 总在 USBHS 中断里。
 * One object per controller. The controller interrupt only calls
 * OnControllerInterrupt; hardware events are handled in ServiceHardware, and a thread's
 * request pends this interrupt, so the owner always runs in the USBHS interrupt.
 */
class CH32USBOtgHS : public USB::DeviceService,
                     private USB::EndpointPool,
                     public USB::DeviceCore
{
 public:
  /// 设备类列表 / Class lists of the configurations
  using Configs = std::initializer_list<const std::initializer_list<USB::DeviceClass*>>;
  /// 字符串语言包 / String language packs
  using Languages = std::initializer_list<const USB::DescriptorStrings::LanguagePack*>;

  /**
   * @brief 构造设备、端点和设备核心 / Construct the device, its endpoints and the core
   * @param rx_ep_cfgs OUT 块，下标为端点号（EP0 在首位），缺席传 {nullptr, 0}
   *        OUT blocks indexed by endpoint number (EP0 first), {nullptr, 0} when absent
   * @param tx_ep_cfgs IN 块，同上 / IN blocks, likewise
   * @param vid 厂商 ID / Vendor ID
   * @param pid 产品 ID / Product ID
   * @param bcd bcdDevice
   * @param languages 字符串语言包 / String language packs
   * @param configs 每个配置的类列表 / Class list of each configuration
   * @param uid 序列号字符串后缀，可为空 / Serial-number suffix, may be empty
   * @note EP1-15 两个方向的块互不相关（各 ≥64 B）；EP0 两个方向传同一块（≥64 B）。
   *       调用者：应用，初始化时一次。
   *       The two blocks of EP1-15 are unrelated (each at least 64 B); both EP0
   *       directions pass the same block (at least 64 B). Caller: the application,
   *       once at init.
   */
  CH32USBOtgHS(const std::initializer_list<RawData>& rx_ep_cfgs,
               const std::initializer_list<RawData>& tx_ep_cfgs, uint16_t vid,
               uint16_t pid, uint16_t bcd, const Languages& languages,
               const Configs& configs, ConstRawData uid = {nullptr, 0});

  /**
   * @brief 开时钟、复位 SIE、以高速使能 DMA、中断和上拉，最后使能 USBHS 中断线
   *        Enable the clock, reset the SIE, enable DMA, interrupts and the pull-up at
   *        high speed, then enable the USBHS line
   * @note 调用者：应用，控制器中断之外。 / Caller: the application, outside the
   *       controller interrupt.
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

  /**
   * @brief 已启动的实例，供中断函数使用 / Started instance used by the interrupt handler
   * @note 写：Start、Stop；读：USBHS_IRQHandler。 / Written by Start and Stop; read by
   *       USBHS_IRQHandler.
   */
  static inline CH32USBOtgHS* self_ = nullptr;

 protected:
  void ServiceHardware(bool in_isr) override;
  /// NVIC_DisableIRQ(USBHS_IRQn)
  void MaskInterrupt() override;
  /// NVIC_EnableIRQ(USBHS_IRQn)
  void UnmaskInterrupt() override;
  /// NVIC_SetPendingIRQ(USBHS_IRQn)，返回 true / NVIC_SetPendingIRQ(USBHS_IRQn), true
  bool PendInterrupt() override;
  /// 状态 IN 完成时写 DEV_AD / Writes DEV_AD when the status IN has completed
  void SetAddress(uint8_t address, AddressStage stage) override;
  /// EP0 RX 置 ACK / EP0 RX to ACK
  void OnControlReady() override;
  /// 复位 SIE 并重新接上总线 / Reset the SIE and attach again
  void ResetController(bool in_isr) override;

 private:
  /// 建端点、登记到端点池和设备服务 / Build the endpoints, register them with the pool
  /// and the device service
  static USB::EndpointPool& AddEndpoints(CH32USBOtgHS& self,
                                         const std::initializer_list<RawData>& rx,
                                         const std::initializer_list<RawData>& tx);
  /**
   * @brief 断开总线，让硬件不再为任何端点工作，作废未结束的停止 / Detach, make the
   *        hardware stop working for every endpoint and void the pending stops
   * @note StopController 和 ResetController 共用。
   *       Shared by StopController and ResetController.
   */
  static void HaltController();
  /**
   * @brief HaltController 断开之后等够最短断开时间 / Wait until the minimum detach time
   *        has passed since HaltController detached
   * @note StartController 和 ResetController 在重新接上之前调用。
   *       Called by StartController and ResetController before they attach.
   */
  static void WaitDetached();
  /// HaltController 断开总线的时刻（µs）和“断开后还没有重新接上” / Time HaltController
  /// detached (us) and "not attached again since"
  static inline uint64_t halted_at_us_ = 0U;
  static inline bool halted_ = false;
  /// 控制器寄存器初始化，Start 和 ResetController 共用 / Controller register set-up
  /// shared by Start and ResetController
  static void SetUpController();
  /// SPEED_TYPE 给出的当前速度 / The current speed from SPEED_TYPE
  static USB::Speed ReadBusSpeed();
  /**
   * @brief EP0 回到等待 SETUP：两个方向 DATA1，IN NAK / EP0 back to waiting for a SETUP:
   *        both directions DATA1, IN NAK
   * @param rx_res EP0 RX 的 RES：总线复位后 ACK，SETUP 后 NAK / RES of EP0 RX: ACK after
   *        a bus reset, NAK after a SETUP
   */
  static void ResetEp0(uint8_t rx_res);
  /// 节拍等待的条件：恒成立，只用它的延时 / Condition of the tick wait: always true,
  /// only its delay is used
  static bool TickCheck(void* arg);
  /// 节拍等待结束（节拍上下文）：叫醒 owner / The tick wait ended (tick context): wake
  /// the owner
  static void TickDone(void* arg, ErrorCode result);

  /**
   * @brief 约 1 ms 后叫醒 owner 的等待 / Wait that wakes the owner about 1 ms later
   * @note 在 owner 里启动：总线没有挂起时每一轮末尾启动（没有别的中断时也能看到帧号
   *       前进），有停止未结束时由端点启动。板级工程须每毫秒调用一次
   *       DriverAsyncCondition::OnTick；没有节拍时，帧起始只在控制器有别的中断时上报，
   *       线上事务不产生事件的那部分停止也要等到控制器的下一次中断才结束。
   *       Started in the owner: at the end of every pass while the bus is not suspended
   *       (so that the frame number is seen to advance with no other interrupt coming),
   *       and by the endpoints while a stop is pending. The board project must call
   *       DriverAsyncCondition::OnTick once per millisecond; without the tick, starts of
   *       frame are reported only when the controller has another interrupt, and the
   *       stops whose transaction on the wire raises no event end only at the
   *       controller's next interrupt.
   */
  DriverAsyncCondition tick_;

  /**
   * @brief 帧号变了就向设备核心报告一次帧起始 / Report one start of frame to the device
   *        core if the frame number changed
   * @note 仅 owner，每一轮末尾调用。帧号取自 FRAME_NO（设备模式下是最近收到的 SOF 的
   *       帧号，低 11 位帧号、高 3 位微帧号，手册 p.340），不开 SOF 中断。两次调用之间
   *       过了不止一个（微）帧时只报告最新的一个，间隔由设备类按帧号自己算。
   *       Owner only, called at the end of every pass. The number comes from FRAME_NO
   *       (in device mode the frame number of the latest SOF received: low 11 bits the
   *       frame, high 3 bits the microframe; manual p.340) and the SOF interrupt stays
   *       off. If more than one (micro)frame passed between two calls only the latest
   *       is reported; a device class derives the interval from the numbers.
   */
  void ReportFrame(bool in_isr);
  /// FRAME_NO 的有效位：帧号和微帧号 / Valid bits of FRAME_NO: frame and microframe
  static constexpr uint16_t FRAME_NO_MASK = 0xE7FFU;
  /// 上次报告的 FRAME_NO；大于 16 位表示还没有 / FRAME_NO reported last; above 16 bits
  /// when there is none yet
  uint32_t last_frame_no_ = UINT32_MAX;
  /// 已向设备核心报告挂起（之后还没有报告恢复或复位）；仅 owner / A suspend was
  /// reported to the device core (no resume or reset since); owner only
  bool suspended_ = false;

  /**
   * @brief 总线复位之后还没收到 SETUP / No SETUP since the last bus reset
   * @note 写、读：ServiceHardware（owner）。控制器在高速 chirp 结束前就报复位，此时
   *       SPEED_TYPE 还是全速；第一个 SETUP 时速度已定，与报告的不同就再报一次复位。
   *       Written and read by ServiceHardware (owner). The controller reports the reset
   *       before the high-speed chirp ends, while SPEED_TYPE still reads full speed; at
   *       the first SETUP the speed is settled, and a different one is reported as a
   *       second reset.
   */
  bool setup_since_reset_ = false;
};

#endif  // defined(USBHSD) || defined(__DOXYGEN__)

}  // namespace LibXR
