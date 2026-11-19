#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <initializer_list>

#include "device_class.hpp"
#include "device_composition.hpp"
#include "libxr_def.hpp"
#include "libxr_type.hpp"
#include "usb/core/core.hpp"
#include "usb/core/desc_dev.hpp"
#include "usb/core/desc_str.hpp"
#include "usb/core/ep.hpp"
#include "usb/core/ep_pool.hpp"
#include "usb/core/usb_service.hpp"

namespace LibXR::USB
{

/**
 * @class DeviceCore
 * @brief USB 设备核心：EP0 控制传输和设备生命周期 / USB device core: EP0 control
 *        transfers and the device lifecycle
 *
 * 后端设备类派生本类和 DeviceService。核心不存事件、不暂缓、不接续：后端在
 * ServiceHardware 中报告的总线事件和 SETUP 当场处理；Init、Deinit、类的服务请求和停止
 * 失败经设备服务的事件位，在同一个 owner 中处理。端点停止不等待：每个处理函数返回时，
 * 它调用的端点函数都已生效（状态已变、请求都已回调），推迟到停止结束的只有端点自己的
 * 硬件动作和请求启动；核心从不等停止，也不因停止推迟应答。
 * The backend device class derives from this class and from DeviceService. The core
 * stores no event, defers nothing and has no continuation: bus events and SETUPs the
 * backend reports from ServiceHardware are handled at once; Init, Deinit, class service
 * requests and stop failures come as the device service's event bits and are handled in
 * the same owner. Endpoint stops do not wait: when a handler returns, the endpoint
 * functions it called have taken effect (state changed, every request called back), and
 * only the endpoints' own hardware steps and request starts wait for the end of a
 * stop; the core never waits for a stop and never delays an answer because of one.
 *
 * @note 因此 SET_CONFIGURATION、SET_INTERFACE 的状态阶段可能早于旧绑定端点停稳。总线
 *       是串行的，停止写入后仍发出的旧包都排在随后的状态事务之前；主机按什么顺序调度
 *       端点由主机决定，换主机是否仍是这样不保证。
 *       Hence the status stage of SET_CONFIGURATION or SET_INTERFACE may come before
 *       the old binding's endpoints are quiescent. The bus is serial, so old packets
 *       still sent after the stop is written come before the later status
 *       transaction; the order in which a host schedules endpoints is the host's, and
 *       is not guaranteed to stay so on another host.
 *
 * @note owner 一轮中的顺序（DeviceService 定的）：先 ServiceHardware，其中后端按硬件
 *       顺序报告端点完成、总线事件和 SETUP，核心逐个当场处理；再设备事件位，按
 *       STOP_FAILED_EVENT、ENABLE_EVENT、SERVICE_EVENT 的顺序；最后启动端点。
 *       Order within one owner pass (set by DeviceService): first ServiceHardware, in
 *       which the backend reports endpoint completions, bus events and SETUPs in
 *       hardware order and the core handles each at once; then the device event bits,
 *       in the order STOP_FAILED_EVENT, ENABLE_EVENT, SERVICE_EVENT; then endpoint
 *       starts.
 * @note 后端在一次状态读取中同时看到的事件按此顺序报告：总线复位、端点完成（EP0 的在
 *       SETUP 之前，见 Endpoint::OnHardwareComplete）、SETUP、挂起和恢复、断开。挂起和
 *       恢复同时出现时，最后报告的那个与硬件此刻的状态一致。
 *       Events the backend sees in one status read are reported in this order: bus
 *       reset, endpoint completions (EP0 ones before the SETUP, see
 *       Endpoint::OnHardwareComplete), SETUP, suspend and resume, disconnect. When
 *       suspend and resume appear together, the one reported last agrees with the state
 *       of the hardware at that moment.
 * @note 合法状态：未初始化时闸门 CLOSED、未配置、EP0 关闭、没有控制传输；已配置蕴含
 *       已初始化；每个处理函数返回时闸门等于 (已初始化, 挂起, 已配置) 决定的值。
 *       Legal states: while not initialized the gate is CLOSED, the device is
 *       unconfigured, EP0 is closed and no control transfer is open; configured implies
 *       initialized; whenever a handler returns, the gate equals the value given by
 *       (initialized, suspended, configured).
 */
class DeviceCore
{
 public:
  /// SET_ADDRESS 的地址写在哪个阶段 / Stage at which the SET_ADDRESS address is written
  enum class AddressStage : uint8_t
  {
    BEFORE_STATUS,   ///< 收到 SETUP，状态 IN 之前 / SETUP received, before status IN
    STATUS_COMPLETE  ///< 状态 IN 已完成 / Status IN completed
  };

  /**
   * @brief 构造 / Construct
   * @param service 本控制器的设备服务（后端自身）/ This controller's device service
   *        (the backend itself)
   * @param pool 端点池，EP0 已经 SetEndpoint0 / Endpoint pool with EP0 already set
   * @param spec USB 规范版本 / USB specification release
   * @param speed 控制器能跑的最高速度，FULL 或 HIGH / Highest speed of the
   *        controller, FULL or HIGH
   * @param packet_size EP0 包长；HIGH 时必须为 64 / EP0 packet size; 64 for HIGH
   * @param vid 厂商 ID / Vendor ID
   * @param pid 产品 ID / Product ID
   * @param bcd bcdDevice
   * @param languages 字符串语言包 / String language packs
   * @param configs 每个配置的类列表 / Class list of each configuration
   * @param uid 序列号字符串后缀，可为空 / Serial-number suffix, may be empty
   * @note 调用者：后端设备构造，在控制器中断使能之前。建组合层、分配控制缓冲、写设备
   *       描述符，并向 service 设置设备事件处理程序。只在这里分配内存。
   *       Caller: the backend device's constructor, before the controller interrupt is
   *       enabled. Builds the composition, allocates the control buffer, writes the
   *       device descriptor and sets the device event handler on service. Memory is
   *       allocated here only.
   */
  DeviceCore(
      DeviceService& service, EndpointPool& pool, USBSpec spec, Speed speed,
      DeviceDescriptor::PacketSize0 packet_size, uint16_t vid, uint16_t pid, uint16_t bcd,
      const std::initializer_list<const DescriptorStrings::LanguagePack*>& languages,
      const std::initializer_list<const std::initializer_list<DeviceClass*>>& configs,
      ConstRawData uid = {nullptr, 0});

  /// 经基类指针销毁后端设备 / Destroy a backend device through the base class
  virtual ~DeviceCore() = default;
  /// 不可复制 / Not copyable
  DeviceCore(const DeviceCore&) = delete;
  /// 不可复制赋值 / Not copy-assignable
  DeviceCore& operator=(const DeviceCore&) = delete;

  /**
   * @brief 请求初始化 / Request initialization
   * @note 任何上下文。记下"要启用"，发布 ENABLE_EVENT；owner 中未初始化时打开 EP0、闸门
   *       改为 EP0、调用 OnControlReady。处理程序可能在返回前运行。后端的中断线使能晚于
   *       本调用时，请求在使能后才被处理（见 Start）。
   *       Any context. Records "enabled" and publishes ENABLE_EVENT; in the owner, if not
   *       initialized, EP0 is opened, the gate becomes EP0 and OnControlReady runs. The
   *       handler may run before this returns. If the backend enables its interrupt line
   *       after this call, the request is served once it does (see Start).
   */
  void Init(bool in_isr);

  /**
   * @brief 请求反初始化 / Request deinitialization
   * @note 任何上下文。记下"不启用"，发布 ENABLE_EVENT；owner 中已初始化时按 Deinitialize
   *       收尾。
   *       Any context. Records "not enabled" and publishes ENABLE_EVENT; in the owner,
   *       if initialized, it winds down as Deinitialize does.
   */
  void Deinit(bool in_isr);

  /**
   * @brief 启动控制器并接上总线 / Start the controller and attach to the bus
   * @note 应用调用，在调用者的上下文里执行 StartController。撤销还没有开始处理的
   *       Stop；已经开始处理的 Stop 要等它断开总线之后才启动（在调用者的上下文里忙等，
   *       通常一个包或一帧的时间）。所以不要在 owner 里（类的回调里）或能打断 owner 的
   *       中断里调用。后端在断开之后的最短断开时间内不重新接上；时间未到时这里不等，
   *       只记下"要接上"并使能中断线，到点后 owner 接上（见 USB::DetachTimer）。所以
   *       返回时总线可能还没有接上。
   *       Called by the application; runs StartController in the caller's context. It
   *       cancels a Stop whose handling has not begun; one that is being handled is
   *       waited for until it has detached (a busy wait in the caller's context,
   *       normally a packet or a frame). So do not call it in the owner (in a class
   *       callback) or from an interrupt that can preempt the owner. A backend does not
   *       attach again within its minimum detach time after a detach; while that time
   *       has not passed this does not wait, it records "attach wanted" and enables the
   *       interrupt line, and the owner attaches once it has passed (see
   *       USB::DetachTimer). So the bus may not be attached yet on return.
   */
  void Start(bool in_isr);

  /**
   * @brief 请求断开总线并停止控制器 / Request to detach from the bus and stop the
   *        controller
   * @note 任何上下文，不等待。记下"要停止"，发布 STOP_EVENT；owner 中先中止控制传输并
   *       Deinitialize：各端点按平常的路径关闭，在途请求以 ABORTED 回调，没有停稳的
   *       停止照常由后端之后结束，被停的槽照常到那时才归还。所有端点的停止都结束之后
   *       （通常在一个包或一帧之内），owner 才调用 StopController 断开总线。所以本函数
   *       返回时总线可能还没有断开；这段时间设备不应答控制传输。处理程序可能在返回前
   *       运行；owner 靠控制器中断运行的后端在中断线使能之前（Start 之前）不会处理它。
   *       之后再 Start，主机的总线复位按记下的启用重新初始化。
   *       Any context, never waits. Records "stop" and publishes STOP_EVENT; in the
   *       owner the control transfer is aborted and Deinitialize runs first: the
   *       endpoints are closed the ordinary way, requests in flight are called back
   *       with ABORTED, a stop that is not quiescent ends later through the backend as
   *       usual and its slot is given back only then. Once the stop of every endpoint
   *       has ended (normally within a packet or a frame) the owner calls StopController
   *       to detach. So the bus may still be attached when this function returns; the
   *       device answers no control transfer in that time. The handler may run before
   *       this returns; a backend whose owner runs in the controller interrupt does not
   *       serve it before its line is enabled (before Start). After a later Start the
   *       host's bus reset initializes again according to the recorded enable.
   */
  void Stop(bool in_isr);

 protected:
  /// @name 后端在 ServiceHardware 中调用（owner）/ Called by the backend from
  ///       ServiceHardware (owner)
  /// @{

  /**
   * @brief 收到 SETUP，当场处理 / A SETUP arrived; handle it at once
   * @param in_isr owner 是否在中断中，同 ServiceHardware / Whether the owner is in an
   *        ISR, as for ServiceHardware
   * @param setup 8 字节 SETUP，只在本调用内读 / The 8-byte SETUP, read during this call
   *        only
   * @note 未初始化时忽略。否则依次：旧传输被类接受过就调用其 OnControlAbort，控制阶段
   *       回到空闲；EP0 IN、OUT 各 Flush（上一个传输以协议 STALL 结束时改为
   *       ClearStall），旧请求在此以 ABORTED 回调，核心不处理它们；然后分发新请求。可能
   *       调用 EP0 和数据端点的后端函数（嵌套在 ServiceHardware 里）。
   *       Ignored when not initialized. Otherwise, in order: if a class accepted the old
   *       transfer, its OnControlAbort is called and the control phase returns to idle;
   *       EP0 IN and OUT are each flushed (cleared with ClearStall instead when the
   *       previous transfer ended in a protocol stall), so old requests are called back
   *       with ABORTED and the core ignores them; then the new request is dispatched.
   *       May call backend functions of EP0 and data endpoints (nested in
   *       ServiceHardware).
   * @note 本核心让 EP0 IN、OUT 各用一个槽、一次只提交一个请求。这里的 Flush 依赖
   *       Endpoint::StopHardware 的交接规则 5：控制端点上的停止最多挂起到下一个 SETUP，
   *       SETUP 报告之后当场给出结果。停止仍未结束、而被停的槽又没有放回时（硬件直接
   *       读写槽），新应答借不到槽（BUSY），按分发失败处理：拒绝这个传输
   *       （StallTransfer），主机重试；核心不等待。
   *       This core uses one slot each on EP0 IN and OUT and submits one request at a
   *       time. The flushes here rely on handover rule 5 of Endpoint::StopHardware: a
   *       stop on a control endpoint stays pending at most until the next SETUP and
   *       gives the outcome at once after a reported SETUP. If the
   *       stop is still pending and the stopped slot was not given back (the hardware
   *       reads or writes the slot directly), the new reply gets no slot (BUSY) and this
   *       counts as a failed dispatch: the transfer is refused (StallTransfer) and the
   *       host retries; the core does not wait.
   * @note 旧传输排队的应答不会发给新传输：同一次读取里的 SETUP 在任何启动之前处理
   *       （报告只置待处理位，见 Endpoint::OnHardwareComplete）；之后才锁存的 SETUP 由
   *       控制端点的 StartHardware 挡住（锁存未报告的 SETUP 时撤回，见
   *       Endpoint::StartHardware）。被挡住的旧应答以 FAILED 或 ABORTED 结束，核心
   *       忽略它。
   *       A queued reply of the old transfer never goes to the new one: a SETUP of
   *       the same status read is handled before any start (a report only sets the
   *       pending bit, see Endpoint::OnHardwareComplete); a SETUP latched later is held
   *       off by the control endpoint's StartHardware (the load is retracted while an
   *       unreported SETUP is latched, see Endpoint::StartHardware). The old reply so
   *       held off ends with FAILED or ABORTED, which the core ignores.
   */
  void OnSetup(bool in_isr, const SetupPacket& setup);

  /**
   * @brief 总线复位 / Bus reset
   * @param in_isr owner 是否在中断中，同 ServiceHardware / Whether the owner is in an
   *        ISR, as for ServiceHardware
   * @param speed 复位后协商出的速度，不得高于构造时的速度 / Speed negotiated by the
   *        reset; not above the construction speed
   * @note 依次：Deinitialize；把 speed 存进设备服务（速度只存这一份）；清挂起；记下要
   *       启用时 Initialize。后端在报告前完成自己的复位记账，并已报告所有未结束的停止
   *       （见 Endpoint::StopHardware）。
   *       In order: Deinitialize; store speed in the device service (the only copy of
   *       the speed); clear suspended; Initialize if enabled is recorded. The backend
   *       does its own reset bookkeeping before reporting and has reported every
   *       pending stop by then (see Endpoint::StopHardware).
   */
  void OnBusReset(bool in_isr, Speed speed);

  /// 总线挂起：记下挂起，闸门关闭，在途请求不停止 / Bus suspend: record suspended,
  /// close the gate; requests in flight are not stopped
  void OnSuspend(bool in_isr);

  /// 总线恢复：清挂起，闸门按状态重开，排队请求启动 / Bus resume: clear suspended,
  /// reopen the gate from the state; queued requests start
  void OnResume(bool in_isr);

  /**
   * @brief 断开（VBUS 消失等） / Disconnect (VBUS lost and the like)
   * @note Deinitialize，清挂起；之后的复位按记下的启用决定。
   *       Deinitialize, clear suspended; a later reset follows the recorded enable.
   */
  void OnDisconnect(bool in_isr);

  /**
   * @brief 帧起始 / Start of frame
   * @param in_isr owner 是否在中断中，同 ServiceHardware / Whether the owner is in an
   *        ISR, as for ServiceHardware
   * @param frame 帧号（11 位），0xffff 表示不知道 / Frame number (11 bits), 0xffff
   *        when unknown
   * @param microframe 微帧 0-7，0xff 表示只报告整帧 / Microframe 0-7, 0xff for whole
   *        frames only
   * @note 已配置时转给当前配置的类，否则什么也不做；核心不存帧号。
   *       Forwarded to the classes of the current configuration when configured,
   *       otherwise nothing; the core keeps no frame number.
   */
  void OnStartOfFrame(bool in_isr, uint16_t frame, uint8_t microframe = 0xffU);

  /// 是否已初始化（EP0 已打开）；仅 owner / Whether initialized (EP0 open); owner only
  bool IsInited() const { return inited_; }

  /// @}
  /// @name 后端实现 / Implemented by the backend
  /// @{

  /**
   * @brief 写设备地址 / Write the device address
   * @param address 主机给的地址 0-127 / Address from the host, 0-127
   * @param stage BEFORE_STATUS 在发状态 IN 之前调用，STATUS_COMPLETE 在状态 IN 完成后
   *        调用；后端在控制器要求的那一次写入。
   *        BEFORE_STATUS is called before the status IN is sent, STATUS_COMPLETE after
   *        it completed; the backend writes at the one its controller needs.
   * @note 仅 owner / Owner only
   */
  virtual void SetAddress(uint8_t address, AddressStage stage) = 0;

  /**
   * @brief 让 EP0 准备接收下一个 SETUP / Make EP0 ready for the next SETUP
   * @note 仅 owner；Initialize 末尾和每个控制传输的状态阶段以 OK 完成时调用，被取代、
   *       STALL 或中止的传输之后不调用。默认什么也不做。Initialize 末尾调用时，上一次
   *       Deinitialize 关 EP0 的停止可能未结束，EP0 的 CloseHardware、ConfigureHardware
   *       会在之后到来；它们不得撤销这里做的准备（见 Endpoint::CloseHardware）。
   *       Owner only; called at the end of Initialize and when a control transfer's
   *       status stage completes with OK, never after a superseded, stalled or aborted
   *       transfer. Does nothing by default. When called at the end of Initialize, the
   *       stop of EP0 from the last Deinitialize may still be pending and EP0's
   *       CloseHardware and ConfigureHardware come afterwards; they must not undo what
   *       this prepared (see Endpoint::CloseHardware).
   */
  virtual void OnControlReady() {}

  /**
   * @brief 端点停止失败后复位控制器 / Reset the controller after an endpoint stop
   *        failed
   * @note 仅 owner，收到 STOP_FAILED_EVENT 时调用。返回时硬件不再为任何端点收发或读写
   *       内存，所有未结束的停止就此结束且不再报告，之后每个 StopHardware 立即返回
   *       ABORTED；总线已断开。只做前半段，不等待：最短断开时间由 DetachTimer 计时，
   *       到点后 owner 在之后的一轮里重新接上，主机重新枚举，复位照常报告。断开期间
   *       （中断线使能）owner 照常处理请求：端点可以打开、关闭，停止立即结束；硬件在
   *       接上时重新初始化，之后的总线复位让核心重新 Initialize。已经断开时（等断开期间
   *       又复位）不重新计时。
   *       Owner only, called on STOP_FAILED_EVENT. On return the hardware no longer
   *       moves data or reads or writes memory for any endpoint, every pending stop has
   *       ended and is not reported any more, and every later StopHardware returns
   *       ABORTED at once; the bus is detached. Only the first half, no wait: the
   *       minimum detach time is counted by DetachTimer, and once it has passed the owner
   *       attaches in a later pass, the host enumerates again and the reset is reported
   *       as usual. While detached (line enabled) the owner serves requests as usual:
   *       endpoints may be opened and closed and their stops end at once; the hardware
   *       is initialized again when it attaches, and the following bus reset makes the
   *       core Initialize again. Already detached (another reset while the detach runs),
   *       the count is not restarted.
   */
  virtual void ResetController(bool in_isr) = 0;

  /**
   * @brief 启动控制器并接上总线 / Start the controller and attach to the bus
   * @note 后端实现；由 Start 在应用的上下文里调用，不经 owner。最迟在这里使能控制器
   *       中断线，之后总线复位才会报告上来。不等待：最短断开时间已过（或从未断开）时
   *       就地接上；未过时只记下"要接上"（DetachTimer::WantAttach），到点后 owner 接上。
   *       Implemented by the backend; called by Start in the application's context, not
   *       through the owner. Enables the controller's interrupt line here at the latest;
   *       bus resets are reported only afterwards. Never waits: when the minimum detach
   *       time has passed (or there was no detach) it attaches in place; otherwise it
   *       only records "attach wanted" (DetachTimer::WantAttach) and the owner attaches
   *       once the time has passed.
   */
  virtual void StartController(bool in_isr) = 0;

  /**
   * @brief 断开总线并让控制器静止 / Detach from the bus and make the controller
   *        quiescent
   * @note 仅 owner。平常在 Stop 的收尾里调用，那时所有端点都已关闭、没有停止未结束，
   *       硬件上没有任何请求，这里只需断开总线并关掉控制器自己的中断源，不需要等任何
   *       握手。停止失败时也会调用（代替 ResetController），那时按 ResetController 的
   *       要求让硬件静止：不再为任何端点收发或读写内存，未结束的停止就此作废、不再
   *       报告，之后每个 StopHardware 立即返回 ABORTED。两种情形都不重新接上总线。中断
   *       线本身保持使能：owner 一轮结束时总会开线，之后的请求也靠它运行。
   *       Owner only. Normally called at the end of a Stop, when every endpoint is closed
   *       and no stop is pending: nothing is on the hardware, so it only detaches and
   *       switches the controller's own interrupt sources off, with no handshake to wait
   *       for. It is also called after a failed stop (instead of ResetController), and
   *       then makes the hardware quiescent as ResetController must: no data moved and
   *       no memory touched for any endpoint, pending stops void and reported no more,
   *       every later StopHardware returning ABORTED at once. Neither case attaches
   *       again. The interrupt line itself stays enabled: an owner pass always unmasks
   *       it at its end, and later requests need it to run.
   * @note 不等待。断开之后开始最短断开时间的计时（DetachTimer::OnHalted(false)）；
   *       正在计时时（复位的断开还没结束就停止）沿用原来的计时，并撤销"要接上"。到点
   *       与否都不再接上，直到下一次 StartController。
   *       Never waits. After the detach the minimum detach time starts counting
   *       (DetachTimer::OnHalted(false)); while it already counts (a stop before a
   *       reset's detach has ended) the running count is kept and "attach wanted" is
   *       withdrawn. Nothing attaches again before the next StartController, whether
   *       the time has passed or not.
   */
  virtual void StopController(bool in_isr) = 0;

  /// @}

 private:
  /// Init、Deinit 发布：重新比较"要启用"与"已初始化" / Published by Init and Deinit:
  /// compare "enabled" with "initialized" again
  static constexpr uint32_t ENABLE_EVENT = 1U << 0U;
  /// Stop 发布：按"要停止"决定是否停止控制器 / Published by Stop: stop the controller
  /// if "stop" is still recorded
  static constexpr uint32_t STOP_EVENT = 1U << 2U;
  static_assert(DeviceClass::SERVICE_EVENT != ENABLE_EVENT &&
                    DeviceClass::SERVICE_EVENT != STOP_EVENT,
                "event bits must differ");

  /// 控制传输阶段 / Control transfer phase
  enum class Phase : uint8_t
  {
    IDLE,        ///< 没有控制传输 / No control transfer
    DATA_IN,     ///< IN 数据阶段，状态 OUT 已挂 / IN data stage, status OUT armed
    DATA_OUT,    ///< OUT 数据阶段 / OUT data stage
    STATUS_IN,   ///< 状态 IN 在途 / Status IN in flight
    STATUS_OUT,  ///< IN 数据发完，等状态 OUT / IN data sent, awaiting status OUT
  };

  /**
   * @brief 设备事件处理程序 / Device event handler
   * @note 设备服务在 owner 中调用。按顺序：STOP_FAILED_EVENT：中止控制传输，
   *       ResetController（正在等断开时改用 StopController 并结束这次 Stop），设备服务
   *       CloseAllEndpoints，Deinitialize；STOP_EVENT：仍"要停止"时中止控制传输、
   *       Deinitialize，开始等断开；等断开期间每次进来都查一遍，所有停止都结束了就
   *       FinishDetach；ENABLE_EVENT：要
   *       启用且未初始化则 Initialize，不启用且已初始化则 Deinitialize；SERVICE_EVENT：
   *       组合层 ServiceClasses。
   *       Called by the device service in the owner. In order: STOP_FAILED_EVENT: abort
   *       the control transfer, ResetController (StopController instead while a
   *       detach is awaited, which ends that Stop), the device service's
   *       CloseAllEndpoints, Deinitialize; STOP_EVENT: if "stop" is still recorded,
   *       abort the control transfer, Deinitialize and start awaiting the detach;
   *       while a detach is awaited every call checks and runs FinishDetach once all
   *       stops have ended; ENABLE_EVENT: Initialize if enabled and not
   *       initialized, Deinitialize if not enabled and initialized; SERVICE_EVENT: the
   *       composition's ServiceClasses.
   */
  void HandleDeviceEvents(bool in_isr, uint32_t events);
  /**
   * @brief Stop 的最后一步 / Last step of a Stop
   * @note StopController，关闭复位中途又打开的端点，结束这次 Stop。
   *       StopController, close endpoints a reset reopened meanwhile, end the Stop.
   */
  void FinishDetach(bool in_isr);

  /// 打开 EP0、置已初始化、更新闸门、OnControlReady / Open EP0, set initialized,
  /// update the gate, OnControlReady
  void Initialize(bool in_isr);

  /**
   * @brief 回到未初始化 / Return to uninitialized
   * @note 依次：中止控制传输；组合层 SetConfiguration(0)（类关闭端点）；关闭 EP0 IN、
   *       OUT；清已初始化；闸门 CLOSED。未初始化时什么也不做。这些 Close 的停止可以未
   *       结束，之后的 Initialize 照样 Configure EP0。
   *       In order: abort the control transfer; the composition's SetConfiguration(0)
   *       (classes close their endpoints); close EP0 IN and OUT; clear initialized; gate
   *       CLOSED. Does nothing when not initialized. The stops of these Close calls may
   *       still be pending, and a later Initialize configures EP0 all the same.
   */
  void Deinitialize(bool in_isr);

  /// 按 (已初始化, 挂起, 已配置) 设置设备服务的闸门 / Set the device service's gate
  /// from (initialized, suspended, configured)
  void UpdateGate();

  /// 分发 setup_：标准请求或类、厂商请求；非 OK 时调用者 STALL / Dispatch setup_ as a
  /// standard or a class/vendor request; on non-OK the caller stalls
  ErrorCode Dispatch(bool in_isr);

  /// 标准请求；SYNCH_FRAME 总是 NOT_SUPPORT / Standard requests; SYNCH_FRAME is always
  /// NOT_SUPPORT
  ErrorCode HandleStandard(bool in_isr);

  /// GET_DESCRIPTOR / GET_DESCRIPTOR
  ErrorCode SendDescriptor(bool in_isr);

  /**
   * @brief 类和厂商请求 / Class and vendor requests
   * @note 设备为接收者的厂商请求先给 BOS；接口、端点为接收者的交给拥有它的类的
   *       OnControlRequest。
   *       Vendor requests to the device go to the BOS first; requests to an interface or
   *       an endpoint go to the owning class's OnControlRequest.
   */
  ErrorCode HandleClassOrVendor(bool in_isr);

  /// 开始 IN 数据阶段：按 wLength 截短，先挂状态 OUT，再发第一包 / Start the IN data
  /// stage: truncate to wLength, arm the status OUT, then send the first packet
  void StartIn(bool in_isr, ConstRawData data);

  /// 把下一包 IN 数据拷进 EP0 的槽并提交；最后一包带 zlp / Copy the next IN packet into
  /// EP0's slot and submit it; the last one carries zlp
  void SendNextIn(bool in_isr);

  /// 为 OUT 数据阶段挂下一包接收 / Arm the next packet of the OUT data stage
  void ReceiveNextOut(bool in_isr);

  /// 发状态 IN（零长度包）/ Send the status IN (a zero-length packet)
  void StatusIn(bool in_isr);

  /// 传输正常结束：阶段回到空闲，类 OnControlComplete，OnControlReady / Normal end:
  /// phase back to idle, class OnControlComplete, OnControlReady
  void Finish(bool in_isr);

  /// 拒绝传输：中止它，再 Stall EP0 IN 和 OUT / Refuse the transfer: abort it, then
  /// Stall EP0 IN and OUT
  void StallTransfer(bool in_isr);

  /// 阶段回到空闲；类接受过的传输调用其 OnControlAbort / Phase back to idle; a transfer a
  /// class accepted gets its OnControlAbort
  void AbortTransfer(bool in_isr);

  /**
   * @brief EP0 IN 完成 / EP0 IN completion
   * @note 只在 DATA_IN、STATUS_IN 且 result 为 OK 时动作；其他（包括 Flush 交付的旧
   *       请求）忽略，阶段已不匹配。
   *       Acts only in DATA_IN or STATUS_IN with OK; anything else (old requests a
   *       flush delivers included) is ignored, as the phase no longer matches.
   */
  void InComplete(bool in_isr, const Endpoint::Completion& completion);

  /**
   * @brief EP0 OUT 完成 / EP0 OUT completion
   * @note DATA_OUT：拷贝数据，收齐交 OnControlData；DATA_IN（主机提前发状态 OUT）或
   *       STATUS_OUT：Flush EP0 IN 后 Finish，不再发数据。其他忽略。交回槽。
   *       DATA_OUT: copy the data and hand it to OnControlData when complete; DATA_IN
   *       (the host sent the status OUT early) or STATUS_OUT: flush EP0 IN, then Finish
   *       and send no more data. Anything else is ignored. The slot is handed back.
   */
  void OutComplete(bool in_isr, const Endpoint::Completion& completion);

  /// @name 构造时固定 / Fixed at construction
  /// @{
  DeviceService& service_;  ///< 设备服务（含速度和闸门）/ Device service (speed, gate)
  /// 端点池；EP0 每次经 GetEndpoint0In/Out 取 / Endpoint pool; EP0 is fetched through
  /// GetEndpoint0In/Out each time
  EndpointPool& pool_;
  const Speed max_speed_;  ///< 控制器最高速度 / Highest controller speed
  /// 设备描述符，类的改写已在构造时完成 / Device descriptor, class rewrite done at
  /// construction
  DeviceDescriptor device_desc_;
  DeviceComposition composition_;  ///< 类与配置 / Classes and configurations
  /// 控制缓冲：OUT 数据阶段和核心自己的小应答 / Control buffer: OUT data stage and
  /// the core's own short answers
  RawData control_buffer_{nullptr, 0};
  /// @}

  /**
   * @brief 应用是否要求启用 / Whether the application asked for the device enabled
   * @note Init、Deinit 在任何上下文写；设备事件处理程序和 OnBusReset 在 owner 中读。
   *       Written by Init and Deinit in any context; read in the owner by the device
   *       event handler and OnBusReset.
   */
  std::atomic<bool> enable_requested_{false};
  /**
   * @brief 应用要求停止控制器、还没有处理 / The application asked to stop the
   *        controller and it is not served yet
   * @note Stop 置位，Start 清零（任何上下文）；设备事件处理程序在 owner 中取走。
   *       Set by Stop and cleared by Start (any context); taken by the device event
   *       handler in the owner.
   */
  std::atomic<bool> stop_requested_{false};
  /// 一次 Stop 从开始处理到断开总线之间为真；Start 等它清零 / True from the start of
  /// a Stop's handling until it has detached; Start waits until it clears
  std::atomic<bool> stop_busy_{false};

  /// @name 仅 owner / Owner only
  /// @{
  /// 已初始化（EP0 打开）：Initialize、Deinitialize 写 / Initialized (EP0 open):
  /// written by Initialize and Deinitialize
  bool inited_ = false;
  /// Stop 已收尾、等所有端点停稳后断开；设备事件处理程序写 / A Stop has wound up and
  /// awaits every endpoint's quiescence to detach; written by the device event handler
  bool detach_pending_ = false;
  /**
   * @brief 总线挂起 / Bus suspended
   * @note OnSuspend、OnResume、OnBusReset、OnDisconnect 写，UpdateGate 读。
   *       Written by OnSuspend, OnResume, OnBusReset, OnDisconnect; read by UpdateGate.
   */
  bool suspended_ = false;
  /// 当前控制传输的阶段 / Phase of the current control transfer
  Phase phase_ = Phase::IDLE;
  /// 当前 SETUP；phase_ 不为 IDLE 时有效 / Current SETUP; valid while phase_ is not IDLE
  SetupPacket setup_{};
  /// 接受了当前传输的类；标准请求为 nullptr / Class that accepted the current
  /// transfer; nullptr for standard requests
  DeviceClass* control_class_ = nullptr;
  /// IN 数据阶段要发的全部数据（已截短）/ All data of the IN data stage (truncated)
  ConstRawData in_data_{nullptr, 0};
  /// 数据阶段已传输的字节数（IN 已提交、OUT 已收到）/ Bytes of the data stage moved
  /// so far (submitted for IN, received for OUT)
  size_t transferred_ = 0;
  /// @}
};

}  // namespace LibXR::USB
