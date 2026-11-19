#pragma once

#include <atomic>
#include <cstdint>

#include "core.hpp"
#include "ep.hpp"
#include "libxr_cb.hpp"
#include "libxr_def.hpp"
#include "serialized_service.hpp"

namespace LibXR::USB
{

/**
 * @class DeviceService
 * @brief USB 设备的串行执行入口 / Serialized execution entry of a USB device
 *
 * 一个控制器一个对象。全部进度（硬件事件、设备事件、端点启动）只在赢得本对象
 * SerializedService 的调用者（owner）中进行，所以不需要锁；寄存器访问通常也都在 owner
 * 中，后端在 owner 之外访问寄存器的条件见 Endpoint 的类说明。控制器中断经
 * OnControllerInterrupt 进入；其他上下文经 Request 或端点的请求进入。核心不进临界区，
 * 也不关全部中断。
 * One object per controller. All progress (hardware events, device events, endpoint
 * starts) happens only in the caller that wins this object's SerializedService (the
 * owner), so no lock is needed; register access normally happens in the owner too, and
 * the class note of Endpoint states when a backend may touch registers outside it. The
 * controller interrupt enters through OnControllerInterrupt; other contexts enter
 * through Request or an endpoint's request. The core takes no critical section and never
 * disables all interrupts.
 *
 * @note 后端设备类派生本类并实现四个函数：ServiceHardware、MaskInterrupt、
 *       UnmaskInterrupt，可选 PendInterrupt。
 *       The backend device class derives from this class and implements four functions:
 *       ServiceHardware, MaskInterrupt, UnmaskInterrupt and, optionally, PendInterrupt.
 * @note owner 的一轮按固定顺序：先硬件事件（随后开中断线），再设备事件，最后启动有
 *       待处理位的端点。一轮中新到的请求由同一个 owner 在下一轮处理。
 *       One owner pass runs in a fixed order: hardware events first (then the line is
 *       unmasked), then device events, then the endpoints whose pending bit is set.
 *       Requests arriving during a pass are handled by the same owner in the next pass.
 */
class DeviceService
{
 public:
  /**
   * @brief 设备核心可用的事件位 / Event bits available to the device core
   * @note 位 26 是 STOPS_ENDED_EVENT，27 是 POLL_EVENT，28-30 是本类的其他事件，31 属于
   *       SerializedService。
   *       Bit 26 is STOPS_ENDED_EVENT, 27 is POLL_EVENT, 28-30 are this class's other
   *       events and 31 belongs to SerializedService.
   */
  static constexpr uint32_t DEVICE_EVENTS = (1U << 26U) - 1U;
  /**
   * @brief 某个端点的停止失败 / An endpoint stop failed
   * @note 后端把某个未结束的停止报告为 TIMEOUT：它在自己的上限内没有确认停稳。设备核心
   *       收到后复位控制器（主机重新枚举），再调用 CloseAllEndpoints，被卡住的槽在那里
   *       变空闲。停止的正常结束平时不通知设备核心，见 STOPS_ENDED_EVENT。
   *       The backend reported a pending stop as TIMEOUT: it could not confirm
   *       quiescence within its own bound. The device core then resets the controller
   *       (the host enumerates again) and calls CloseAllEndpoints, where the stuck slot
   *       becomes free. The normal end of a stop is not signalled to the device core,
   *       except through STOPS_ENDED_EVENT.
   */
  static constexpr uint32_t STOP_FAILED_EVENT = 1U << 28U;
  /**
   * @brief 有一个未结束的停止结束了 / A pending stop has ended
   * @note 只在设备核心用 WatchStops 要求时发布：它在断开总线之前等所有端点停稳。
   *       Published only while the device core asked for it with WatchStops: it waits
   *       for every endpoint to be quiescent before it detaches from the bus.
   */
  static constexpr uint32_t STOPS_ENDED_EVENT = 1U << 26U;

  /**
   * @brief 端点启动闸门 / Endpoint start gate
   * @note 只有三种取值，数据端点打开时 EP0 一定打开。
   *       Three values only; whenever data endpoints may start, EP0 may too.
   */
  enum class StartGate : uint8_t
  {
    CLOSED,  ///< 任何端点都不启动硬件 / No endpoint starts hardware
    EP0,     ///< 只有 EP0 可以启动 / Only EP0 may start
    ALL      ///< 所有端点都可以启动 / Every endpoint may start
  };

  /// 构造；闸门关闭，速度为 FULL / Construct; gate closed, speed FULL
  DeviceService() = default;
  /// 经基类指针销毁后端设备 / Destroy a backend device through the base class
  virtual ~DeviceService() = default;
  /// 不可复制：一个控制器只有一个 owner / Not copyable: one controller has one owner
  DeviceService(const DeviceService&) = delete;
  /// 不可复制赋值 / Not copy-assignable
  DeviceService& operator=(const DeviceService&) = delete;

  /**
   * @brief 控制器中断入口 / Controller interrupt entry
   * @note 调用者：后端的控制器中断函数，只在该中断里调用。先屏蔽本控制器的中断线，再
   *       发布硬件事件并尝试成为 owner。成为 owner 时处理硬件事件、开线、推进有工作的
   *       端点；否则直接返回，由当前 owner 在同一轮处理硬件事件并开线。
   *       Caller: the backend's controller interrupt handler, only there. Masks this
   *       controller's line first, then publishes the hardware event and tries to own.
   *       If it owns, it handles the hardware events, unmasks the line and advances the
   *       endpoints with work; otherwise it returns at once and the current owner
   *       handles the hardware events in its pass and unmasks.
   */
  void OnControllerInterrupt();

  /**
   * @brief 以设备事件位请求 owner / Request the owner with device event bits
   * @param events DEVICE_EVENTS 内的非零位，含义由设备核心定义。
   *        Nonzero bits within DEVICE_EVENTS; their meaning is the device core's.
   * @param in_isr 是否在中断中 / Whether in an ISR
   * @note 调用者：设备核心（生命周期、挂起等请求），设备类经设备核心定义的位；任何上下
   *       文，包括 owner 内。有 PendInterrupt 时先发布再挂起控制器中断，owner 在中断里
   *       运行；否则就地尝试成为 owner。处理程序可能在返回前运行。
   *       Callers: the device core (lifecycle, suspend and similar requests) and device
   *       classes through bits the device core defines; any context, the owner included.
   *       With PendInterrupt it publishes, then pends the controller interrupt and the
   *       owner runs there; otherwise it tries to own in place. The handler may run
   *       before this returns.
   * @note 一次请求的位可能在两轮里各出现一次（另一个 owner 在发布与本调用尝试成为 owner
   *       之间先取走了它）。位只表示需要重新检查，不计次数。
   *       The bits of one call may show up in two passes (another owner took them
   *       between the publish and this call's attempt to own). A bit means "check
   *       again", not a count.
   */
  void Request(uint32_t events, bool in_isr);

  /**
   * @brief 设置设备事件处理程序 / Set the device event handler
   * @param handler 在 owner 中以本轮的设备事件位调用（DEVICE_EVENTS 与
   *        STOP_FAILED_EVENT 的子集，非零）。
   *        Called in the owner with this pass's device event bits (a nonzero subset of
   *        DEVICE_EVENTS and STOP_FAILED_EVENT).
   * @note 调用者：设备核心，构造时一次，在控制器中断使能之前。处理程序按位重新检查
   *       状态，不把一个位当作一次事件（见 Request）。
   *       Caller: the device core, once at construction, before the controller
   *       interrupt is enabled. The handler re-checks state per bit and never treats a
   *       bit as one occurrence (see Request).
   */
  void SetDeviceHandler(Callback<uint32_t> handler) { device_handler_ = handler; }

  /**
   * @brief 登记端点，分配其待处理位 / Register an endpoint and give it its pending bit
   * @param ep 端点（端点号已由构造检查）；BOTH 端点用 OUT 位。
   *        The endpoint (its number was checked at construction); a BOTH endpoint gets
   *        the OUT bit.
   * @return OK；FULL：该位已被占用。
   *         OK; FULL: the bit is taken.
   * @note 调用者：后端设备构造时，EP0 两个和每个数据端点各一次；在控制器中断使能和第一
   *       次 Request 之前，不可并发。
   *       Caller: the backend device at construction, once for each EP0 half and each
   *       data endpoint; before the controller interrupt is enabled and the first
   *       Request, never concurrently.
   */
  ErrorCode Register(Endpoint& ep);

  /**
   * @brief 设置端点启动闸门 / Set the endpoint start gate
   * @note 调用者：设备核心，仅 owner（复位、地址或配置变化、挂起、恢复时）。每个已登记
   *       端点都重新检查队列，新闸门不允许的端点什么也不做；关闭不停止在途请求。
   *       Caller: the device core, owner only (on reset, address or configuration
   *       change, suspend and resume). Every registered endpoint re-checks its queue,
   *       and one the new gate does not allow does nothing; closing does not stop
   *       requests in flight.
   */
  void SetStartGate(StartGate gate);

  /**
   * @brief 控制器复位后关闭所有已登记端点 / Close every registered endpoint after a
   *        controller reset
   * @note 调用者：设备核心，仅 owner，收到 STOP_FAILED_EVENT 并 ResetController 之后。
   *       每个端点：未结束或失败的停止经 StopHardware（复位后立即返回）结束，被停的槽
   *       变空闲，不回调；然后 Close，CloseHardware 当场调用。包括已被设备类交回端点池
   *       的端点和 EP0。
   *       Caller: the device core, owner only, after STOP_FAILED_EVENT and
   *       ResetController. For each endpoint a pending or failed stop ends through
   *       StopHardware (which returns at once after the reset) and the stopped slot
   *       becomes free without a callback; then Close, with CloseHardware in place.
   *       Endpoints a device class has handed back to the pool, and EP0, are included.
   */
  void CloseAllEndpoints(bool in_isr);

  /// 是否还有端点的停止未结束（含已报告失败的）；仅 owner / Whether any endpoint still
  /// has a stop that is pending (failed ones included); owner only
  bool AnyStopPending() const;

  /// 之后每个停止结束时是否发布 STOPS_ENDED_EVENT；仅 owner / Whether every stop that
  /// ends from now on publishes STOPS_ENDED_EVENT; owner only
  void WatchStops(bool on) { watch_stops_ = on; }

  /// 总线速度；调用者：端点（Configure）、设备核心，仅 owner / Bus speed; callers: the
  /// endpoint (Configure) and the device core, owner only
  Speed GetSpeed() const { return speed_; }
  /// 记录总线速度；调用者：设备核心在复位后，仅 owner / Record the bus speed; caller: the
  /// device core after a reset, owner only
  void SetSpeed(Speed speed) { speed_ = speed; }

 protected:
  /// @name 后端提供 / Supplied by the backend
  /// @{

  /**
   * @brief 处理硬件事件 / Handle hardware events
   * @param in_isr owner 是否在中断中 / Whether the owner is in an ISR
   * @note 只在 owner 中调用，原因是控制器中断（HARDWARE_EVENT）或 RequestPoll
   *       （POLL_EVENT）。读状态、搬数据，并向核心报告完成和停止的结束
   *       （OnHardwareComplete）以及 SETUP；推进挂起的停止（查一次是否停稳、按节拍数判
   *       超时）。处理完返回；只有控制器中断引起时核心随后开中断线。不等总线。
   *       Called in the owner only, because of the controller interrupt
   *       (HARDWARE_EVENT) or RequestPoll (POLL_EVENT). Reads status, moves data and
   *       reports completions and the ends of stops (OnHardwareComplete) and SETUP to
   *       the core; advances pending stops (checks quiescence once, times them out by
   *       counted ticks). Returns when done; the core unmasks the line afterwards only
   *       when the controller interrupt caused the call. Never waits for the bus.
   */
  virtual void ServiceHardware(bool in_isr) = 0;

  /**
   * @brief 节拍请求一轮 ServiceHardware（发布事件、挂起中断）/ The tick asks for one
   *        pass of ServiceHardware (publish an event, pend the interrupt)
   * @param in_isr 是否在中断中 / Whether in an ISR
   * @note 用途：控制器没有中断可等的事由后端按节拍来查，即未结束的停止按时间结束，以及
   *       只能读寄存器得到的状态（例如没有开帧起始中断时的帧号）。
   *       Purpose: what the controller raises no interrupt for is checked by the backend
   *       on the tick, that is pending stops that end by time, and state that can only
   *       be read from a register (the frame number where the start-of-frame interrupt
   *       is not enabled, for instance).
   * @note 新增用途：最短断开时间到点且要接上时（DetachTimer 的结果函数），owner 在这一轮
   *       ServiceHardware 开头接上总线。断开期间控制器没有中断，这是唯一的唤醒。
   *       Added purpose: when the minimum detach time has passed and an attach is wanted
   *       (DetachTimer's result function), the owner attaches at the start of that
   *       pass's ServiceHardware. A detached controller raises no interrupt, so this is
   *       the only wake-up.
   * @note 调用者：后端自己持有的毫秒节拍对象（DriverAsyncCondition）的结果函数，在时基
   *       上下文里，后端需要时每个节拍一次。只发布 POLL_EVENT 并挂起控制器中断，不碰
   *       USB 寄存器；当前的 owner（控制器中断，或持有 owner 的线程）在它的下一轮以
   *       POLL_EVENT 调用 ServiceHardware。只有 PendInterrupt 返回 true 的后端可以用
   *       它，核心断言这一点：否则节拍中断会就地成为 owner，类的回调会在节拍上下文里
   *       运行。没有 pend 的后端没有节拍兜底，只能靠停稳中断结束停止。
   *       Caller: the result function of the backend's own millisecond tick object
   *       (DriverAsyncCondition), in the timebase context, once per tick while the
   *       backend needs it. Only publishes POLL_EVENT and pends the controller interrupt,
   *       touching no USB register; the current owner (the controller interrupt, or the
   *       thread that owns) calls ServiceHardware for POLL_EVENT in its next pass. Only
   *       a backend whose PendInterrupt returns true may use it, as the core asserts:
   *       otherwise the tick interrupt would own in place and class callbacks would run
   *       in the tick context. A backend without pend has no tick fallback and ends
   *       stops only on quiescence interrupts.
   * @note 例外：没有 pend 的后端可以在线程里调用（in_isr 为 false），调用者就地成为
   *       owner 做这一轮；ESP32 的定时器服务任务这样接上总线，见
   *       ESP32USBDevice::detach_。在中断里调用仍然断言。
   *       Exception: a backend without pend may call it in a thread (in_isr false), and
   *       the caller owns in place for that pass; ESP32's timer service task attaches
   *       the bus this way, see ESP32USBDevice::detach_. A call in an interrupt still
   *       asserts.
   */
  void RequestPoll(bool in_isr);

  /**
   * @brief 在中断控制器屏蔽本控制器的中断线 / Mask this controller's line at the
   *        interrupt controller
   * @note 只在 OnControllerInterrupt 开头调用。返回时屏蔽对任何核都已生效。不得改控制
   *       器自己的中断使能寄存器。
   *       Called only at the start of OnControllerInterrupt. Effective for every core
   *       on return. Must not touch the controller's own interrupt-enable registers.
   * @note 例外（FSDEV）：D+ 被拉住、控制器时钟关着时，FSDev 后端在 Dispatch 里屏蔽自己
   *       的中断线，断开时间到点时由节拍（DetachTimer 的结果函数，FSDevDevice::WakeHeld）
   *       打开并挂起。原因：时钟关着时控制器的中断请求停在关时钟那一刻，可能一直有效；
   *       STM32F103 实测，不屏蔽时中断线反复进入、优先级更低的节拍得不到运行，第 4 次
   *       强制复位就再也没有接上；屏蔽后 70/70 次复位、30/30 次停止再启动都接上了。
   *       Exception (FSDEV): while D+ is held with the controller clock off, the FSDev
   *       backend masks its own line in Dispatch, and the tick (DetachTimer's result
   *       function, FSDevDevice::WakeHeld) unmasks and pends it when the detach time has
   *       passed. Reason: with the clock off the controller's interrupt request stays as
   *       it was when the clock went off and may stay active; measured on an STM32F103,
   *       without the mask the line kept entering, the lower-priority tick never ran and
   *       the device never attached again from the 4th forced reset on; with the mask
   *       70/70 resets and 30/30 stop-then-start cycles attached again.
   */
  virtual void MaskInterrupt() = 0;

  /**
   * @brief 解除屏蔽 / Unmask
   * @note 只在 owner 处理完硬件事件后调用；FSDEV 拉住 D+ 期间的例外见 MaskInterrupt。
   *       Called only by the owner after handling the hardware events; for the FSDEV
   *       exception while D+ is held see MaskInterrupt.
   */
  virtual void UnmaskInterrupt() = 0;

  /**
   * @brief 把本控制器的中断置为挂起 / Set this controller's interrupt pending
   * @return true：已挂起，owner 将在该中断里运行；false：不支持（默认），调用者就地尝试
   *         成为 owner。
   *         true: pended, the owner will run in that interrupt; false: not supported
   *         (the default), the caller tries to own in place.
   * @note 任何上下文。同一后端必须总是返回同一个值。
   *       Any context. A backend must always return the same value.
   */
  virtual bool PendInterrupt() { return false; }

  /// @}

 private:
  friend class Endpoint;
  friend class DetachTimer;

  /// SerializedService 中的硬件事件位 / Hardware event bit in the SerializedService
  static constexpr uint32_t HARDWARE_EVENT = 1U << 30U;
  /// 节拍请求一轮 ServiceHardware（RequestPoll）/ The tick asks for one pass of
  /// ServiceHardware (RequestPoll)
  static constexpr uint32_t POLL_EVENT = 1U << 27U;
  /// 至少一个端点的待处理位已置 / At least one endpoint's pending bit is set
  static constexpr uint32_t ENDPOINT_EVENT = 1U << 29U;
  /// 待处理位的个数：每个端点号两个方向 / Number of pending bits: two per endpoint number
  static constexpr uint32_t ENDPOINT_BITS =
      2U * static_cast<uint32_t>(Endpoint::EPNumber::EP_MAX_NUM);
  static_assert(ENDPOINT_BITS <= 32U, "pending bits must fit one word");

  /**
   * @brief 非中断入口：发布，然后挂起中断或就地尝试成为 owner / Entry from outside the
   *        controller interrupt: publish, then pend or try to own in place
   * @param events 要发布的 SerializedService 事件位 / SerializedService bits to publish
   * @note Request、RequestEndpoint 和 RequestPoll 共用。发布一定先于 PendInterrupt。
   *       Shared by Request, RequestEndpoint and RequestPoll. Publishing always
   *       precedes PendInterrupt.
   */
  void Enter(uint32_t events, bool in_isr);

  /**
   * @brief owner 的一轮 / One owner pass
   * @param events 本轮取到的事件位 / Event bits taken for this pass
   * @note SerializedService 的处理程序。HARDWARE_EVENT 或 POLL_EVENT：ServiceHardware
   *       一次，有 HARDWARE_EVENT 时再 UnmaskInterrupt；设备事件位：设备处理程序；
   *       ENDPOINT_EVENT：取走全部待处理位（包括本轮前面刚置的），按位调用端点的
   *       StartNext。
   *       The SerializedService handler. HARDWARE_EVENT or POLL_EVENT: ServiceHardware
   *       once, then UnmaskInterrupt if HARDWARE_EVENT was set; device bits: the device
   *       handler; ENDPOINT_EVENT: take all pending bits (those set earlier in this pass
   *       included) and call StartNext of each endpoint.
   */
  void Serve(uint32_t events, bool in_isr);

  /// @name 端点调用 / Called by Endpoint
  /// @{

  /**
   * @brief 为一个端点请求 owner / Request the owner for one endpoint
   * @param bit 端点的待处理位 / The endpoint's pending bit
   * @note 任何上下文，包括 owner 内。先置待处理位，再经 Enter 发布 ENDPOINT_EVENT。
   *       Any context, the owner included. Sets the pending bit, then publishes
   *       ENDPOINT_EVENT through Enter.
   */
  void RequestEndpoint(uint32_t bit, bool in_isr);

  /// 端点现在能否启动硬件；仅 owner / Whether an endpoint may start hardware now; owner
  /// only
  bool CanStart(bool ep0) const
  {
    return gate_ == StartGate::ALL || (ep0 && gate_ == StartGate::EP0);
  }

  /**
   * @brief 端点的停止报告为 TIMEOUT / An endpoint's stop was reported as TIMEOUT
   * @note 仅 owner，在硬件事件处理中。发布 STOP_FAILED_EVENT，同一个 owner 的下一轮
   *       交给设备处理程序。
   *       Owner only, from the hardware event handling. Publishes STOP_FAILED_EVENT
   *       for the device handler in the same owner's next pass.
   */
  void StopFailed() { service_.Publish(STOP_FAILED_EVENT); }

  /// 端点的停止正常结束；仅 owner / An endpoint's stop ended normally; owner only
  void StopEnded()
  {
    if (watch_stops_)
    {
      service_.Publish(STOPS_ENDED_EVENT);
    }
  }

  /// @}

  /// owner 选举与事件位 / Owner election and event bits
  SerializedService service_;
  /// 设备核心在等所有停止结束；仅 owner / The device core is waiting for every stop to
  /// end; owner only
  bool watch_stops_ = false;
  /// 端点待处理位：任何上下文置位，owner 取走 / Endpoint pending bits: set anywhere,
  /// taken by the owner
  std::atomic<uint32_t> endpoint_pending_{0U};
  /// 待处理位到端点；登记时写 / Pending bit to endpoint; written at registration
  Endpoint* endpoints_[ENDPOINT_BITS] = {};
  /// 设备事件处理程序；构造时写 / Device event handler; written at construction
  Callback<uint32_t> device_handler_;
  StartGate gate_ = StartGate::CLOSED;  ///< 启动闸门：仅 owner / Start gate: owner only
  Speed speed_ = Speed::FULL;           ///< 总线速度：仅 owner / Bus speed: owner only
};

}  // namespace LibXR::USB
