#pragma once

#include <atomic>
#include <cstdint>

#include "driver_async_condition.hpp"
#include "libxr_def.hpp"
#include "usb_service.hpp"

namespace LibXR::USB
{

/**
 * @class DetachTimer
 * @brief 后端的最短断开时间：断开后按毫秒节拍计时，到点后由 owner 重新接上
 *        A backend's minimum detach time: counted on the millisecond tick after a
 *        detach; the owner attaches again once it has passed
 *
 * 断开总线后，主机要看到足够长的断开才认为设备离开（复位、停止、构造时的断开都一样）。
 * 断开的一方（HaltController）
 * 调用 OnHalted 后立即返回；时间由一个 DriverAsyncCondition 按节拍计数，不读 Timebase
 * 的时间戳，所以工程选错时基类、时间戳回退时这段时间也不会变短；到点时若要接上，结果
 * 函数调用 RequestPoll，owner 在下一轮 ServiceHardware 开头用 TakeAttach 取到这件事，
 * 做后半段（平台放开 D+、重新初始化控制器、接上拉）。StartController 不等：时间未到时
 * 只记下"要接上"。
 * After a detach the host has to see the bus detached long enough before it takes the
 * device as gone (a reset, a stop and the detach at construction alike). The side that
 * detaches (HaltController) calls OnHalted and returns at once; the time is
 * counted on the tick by a DriverAsyncCondition, without Timebase timestamps, so a
 * project that picked the wrong timebase class or a timestamp that steps back cannot
 * shorten it; when it has passed and an attach is wanted, the result function calls
 * RequestPoll and the owner picks the attach up with TakeAttach at the start of its next
 * ServiceHardware and does the second half (the platform releases D+, the controller is
 * initialized again, the pull-up is connected). StartController does not wait: while the
 * time has not passed it only records "attach wanted".
 *
 * 状态（一个原子字，CAS 推进）/ States (one atomic word, advanced by CAS):
 * - ATTACHED：没有断开，或已经重新接上；计时对象空闲。/ Not detached, or attached again;
 *   the condition is idle.
 * - DETACHING：已断开，计时中，不要接上。/ Detached, counting, no attach wanted.
 * - DETACHING_WANTED：已断开，计时中，到点后接上。/ Detached, counting, attach when the
 *   time has passed.
 * - DETACHED：已断开，时间已够，不要接上（停止之后、构造之后 Start 之前）。/ Detached,
 *   the time has passed, no attach wanted (after a stop; after construction before
 *   Start).
 * - ATTACH_DUE：时间已够且要接上，POLL_EVENT 已发布，等 owner 接上。/ The time has passed
 *   and an attach is wanted; POLL_EVENT is published and the owner is to attach.
 *
 * @note 计时只从 ATTACHED 开始。已经断开时再次断开（等断开期间又复位或又停止）不重新
 *       计时：总线从第一次断开起一直断着，主机看到的断开就是从那时算起。所以计时对象的
 *       Start() 只在它空闲时调用，BUSY 不会出现，更不会传到应用。
 *       Counting starts only from ATTACHED. A detach while already detached (another
 *       reset or stop while the detach runs) does not restart it: the bus has stayed
 *       detached since the first detach, and that is when the host's view of it began.
 *       So the condition's Start() is called only while it is idle; BUSY cannot occur,
 *       let alone reach the application.
 * @note 有 PendInterrupt 的后端：结果函数调用 RequestPoll，只发布并挂起控制器中断，
 *       在任何节拍上下文都合法（FreeRTOS 的节拍中断不高于
 *       configMAX_SYSCALL_INTERRUPT_PRIORITY，见 system/freertos/libxr_system.hpp）。
 *       唤醒不止 RequestPoll 的后端在构造时给出 defer，结果函数改调用它：没有 pend 的
 *       后端（ESP32）把这一轮交给一个任务（见 ESP32USBDevice::detach_）；FSDEV 先打开
 *       拉住期间屏蔽的中断线再挂起（见 FSDevDevice::WakeHeld）。defer 返回 false 时结果
 *       函数 Start() 1 ms 后再试，不丢。
 *       Backends with PendInterrupt: the result function calls RequestPoll, which only
 *       publishes and pends the controller interrupt and is legal in any tick context
 *       (FreeRTOS's tick interrupt is never above configMAX_SYSCALL_INTERRUPT_PRIORITY,
 *       see system/freertos/libxr_system.hpp). A backend whose wake-up needs more than
 *       RequestPoll gives defer at construction and the result function calls it
 *       instead: a backend without pend (ESP32) hands that pass to a task (see
 *       ESP32USBDevice::detach_); FSDEV first opens the line it masked while D+ was held,
 *       then pends it (see FSDevDevice::WakeHeld). When defer returns false the result
 *       function Start()s again 1 ms later, nothing is lost.
 * @note 没有节拍时计时永远不结束。检测：构造时 REQUIRE(HasTickSource())；
 *       StartController 遇到计时中时，若 Timebase 显示自断开起已过 3 × detach_ms 而
 *       TickCount() 没动就 ASSERT（时间戳回退时只会漏报，不会误报）。都不等待。
 *       Without the tick the count never ends. Detection: REQUIRE(HasTickSource()) at
 *       construction; StartController, finding the count running, ASSERTs when Timebase
 *       shows 3 x detach_ms since the detach while TickCount() did not move (a timestamp
 *       stepping back can only make it miss, never report falsely). Neither waits.
 */
class DetachTimer
{
 public:
  /// OnHalted 的结果 / Result of OnHalted
  enum class HaltResult : uint8_t
  {
    COUNTING,    ///< 计时中（新开始或沿用）/ Counting (newly started or continued)
    ATTACH_NOW,  ///< 时间早已够且要接上：调用者（owner）就地做后半段 / The time passed
                 ///< long ago and an attach is wanted: the caller (owner) does the
                 ///< second half in place
    DETACHED,    ///< 时间早已够，不要接上 / The time passed long ago, no attach wanted
  };

  /// 唤醒接上的一轮（代替 RequestPoll）；节拍上下文调用，成功返回 true / Wakes the attach
  /// pass (instead of RequestPoll); called in the tick context, true on success
  using DeferFun = bool (*)(void* arg);

  /**
   * @param service 结果函数在其上调用 RequestPoll / The service the result function
   *                calls RequestPoll on
   * @param detach_ms 最短断开时间（毫秒）；实际不短于它，最多多一个节拍 / Minimum
   *                  detach time in milliseconds; the actual time is not shorter and at
   *                  most one tick longer
   * @note 构造时 REQUIRE(DriverAsyncCondition::HasTickSource())。
   *       REQUIREs DriverAsyncCondition::HasTickSource() at construction.
   */
  DetachTimer(DeviceService& service, uint32_t detach_ms);
  /// 唤醒不止 RequestPoll 的后端用 / For a backend whose wake-up needs more than
  /// RequestPoll
  DetachTimer(DeviceService& service, uint32_t detach_ms, DeferFun defer, void* arg);

  /**
   * @brief 后端刚断开了总线 / The backend has just detached the bus
   * @param attach_wanted 到点后是否接上（ResetController：true；StopController 和构造：
   *                      false）/ Whether to attach when the time has passed
   *                      (ResetController: true; StopController and construction: false)
   * @return 见 HaltResult / See HaltResult
   * @note 调用者：HaltController 之后，owner 中，或构造中（中断线使能之前）。不等待。
   *       ATTACHED 时开始计时；正在计时时只更新"要接上"；DETACHED 时：要接上返回
   *       ATTACH_NOW，否则保持。
   *       Caller: after HaltController, in the owner, or in the constructor (before the
   *       line is enabled). Never waits. From ATTACHED it starts counting; while counting
   *       it only updates "attach wanted"; from DETACHED it returns ATTACH_NOW if an
   *       attach is wanted and stays otherwise.
   * @note 计时开始时记下 TickCount() 和 Timebase 的时刻，供 WantAttach 的检测用。
   *       When counting starts, TickCount() and the Timebase time are recorded for the
   *       check in WantAttach.
   */
  HaltResult OnHalted(bool attach_wanted);

  /**
   * @brief StartController 要接上 / StartController wants to attach
   * @return true：现在接上（ATTACHED，或时间已够的 DETACHED），调用者就地接上；
   *         false：还在计时，已记下"要接上"，调用者不接上、不等待，只使能中断线
   *         true: attach now (ATTACHED, or DETACHED with the time passed); the caller
   *         attaches in place. false: still counting; "attach wanted" is
   *         recorded and the caller neither attaches nor waits, it only enables the
   *         interrupt line
   * @note 调用者：StartController，应用上下文。与结果函数（节拍）和 owner 并发，CAS。
   *       计时中时做没有节拍的检测（见类说明）。
   *       Caller: StartController, application context. Concurrent with the result
   *       function (tick) and the owner; CAS. While counting it runs the missing-tick
   *       check (see the class notes).
   */
  bool WantAttach();

  /**
   * @brief owner 取走到点的接上 / The owner takes a due attach
   * @return ATTACH_DUE 时转为 ATTACHED 并返回 true，调用者随即做后半段 / From
   *         ATTACH_DUE: switches to ATTACHED and returns true; the caller does the second
   *         half right away
   * @note 仅 owner，ServiceHardware 开头（POLL_EVENT 的一轮）。
   *       Owner only, at the start of ServiceHardware (the POLL_EVENT pass).
   */
  bool TakeAttach();

  /// 断开中（ATTACHED 以外的任何状态）；任何上下文 / Detached (any state other than
  /// ATTACHED); any context
  bool Detached() const;

 private:
  enum class State : uint32_t
  {
    ATTACHED,
    DETACHING,
    DETACHING_WANTED,
    DETACHED,
    ATTACH_DUE,
  };

  /// 恒成立：只用计时对象的延时 / Always true: only the condition's delay is used
  static bool Check(void* arg);
  /**
   * @brief 时间已够（节拍上下文）/ The time has passed (tick context)
   * @note DETACHING → DETACHED；DETACHING_WANTED → ATTACH_DUE 并 RequestPoll(true)
   *       （给了 defer 时调用 defer）。不碰 USB 寄存器。
   *       DETACHING -> DETACHED; DETACHING_WANTED -> ATTACH_DUE and RequestPoll(true)
   *       (defer instead when given). Touches no USB register.
   */
  static void Done(void* arg, ErrorCode result);

  DeviceService& service_;
  const uint32_t detach_ms_;
  std::atomic<State> state_{State::ATTACHED};
  /// 代替 RequestPoll 的唤醒 / Wake-up instead of RequestPoll
  DeferFun defer_ = nullptr;
  void* defer_arg_ = nullptr;
  /// 计时对象正在等 defer 的重试（只在节拍上下文读写）/ The condition is waiting for a
  /// retry of defer (read and written in the tick context only)
  bool retry_ = false;
  /// 计时开始时的 TickCount() 和 Timebase 毫秒，检测用 / TickCount() and Timebase ms at
  /// the start of the count, for the check
  uint32_t tick_at_halt_ = 0U;
  uint32_t ms_at_halt_ = 0U;
  /// 计时对象，只在 ATTACHED → DETACHING 时 Start / The counting condition, started
  /// only on ATTACHED -> DETACHING
  DriverAsyncCondition condition_;
};

}  // namespace LibXR::USB
