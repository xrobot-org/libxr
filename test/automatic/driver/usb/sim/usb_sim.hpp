/**
 * @file usb_sim.hpp
 * @brief USB 仿真后端 / USB simulation backend
 *
 * 只实现核心声明的后端契约：Endpoint 的后端函数（XxxHardware、HardwareLimits）、
 * DeviceService 的四个函数和 DeviceCore 对后端的接口；其余全部是真实的端点核心、设备
 * 核心、组合层和设备类。违反契约的调用直接断言。
 * Implements only the backend contract the core declares: the backend functions of
 * Endpoint (XxxHardware, HardwareLimits), the four functions of DeviceService and the
 * backend interface of DeviceCore; everything above it is the real endpoint core, device
 * core, composition and device classes. A call that breaks the contract asserts.
 *
 * 硬件和 CPU 分开建模。主机的每个动作（SETUP、OUT、IN 令牌、SOF、总线复位、挂起、
 * 恢复、断开）先在硬件模型里当场完成握手，再锁存事件、拉起中断线；中断线屏蔽时事件
 * 留着，开线时再进中断。硬件模型自己的锁只模拟寄存器访问的原子性，任何核心函数都不在
 * 它里面调用。
 * Hardware and CPU are modelled apart. Every host action (SETUP, OUT and IN tokens, SOF,
 * bus reset, suspend, resume, disconnect) gets its handshake from the hardware model on
 * the spot, then latches its event and raises the interrupt line; while the line is
 * masked the event waits, and the interrupt runs once the line is unmasked. The
 * hardware model's own lock stands only for the atomicity of register access; no core
 * function is ever called while it is held.
 *
 * 时间只有帧：SOF 由主机推进，停稳和停止超时都按帧计。
 * Time is frames only: the host advances SOF, and quiescence and the stop timeout count
 * frames.
 */

#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <deque>
#include <functional>
#include <initializer_list>
#include <memory>
#include <mutex>
#include <new>
#include <thread>
#include <vector>

#include "dev_core.hpp"
#include "ep.hpp"
#include "ep_pool.hpp"
#include "test_assert.hpp"
#include "usb_service.hpp"

#ifndef USB_SIM_ABLATION
#define USB_SIM_ABLATION 0
#endif

namespace LibXR::USB::Sim
{

/**
 * @brief 消融编号：非 0 时把仿真改错一处，用来证明用例能失败 / Ablation number: when
 *        nonzero one place of the simulation is broken, to show that a case can fail
 * @note 编号见各处 Ablated(n)；只用于手动的消融构建（-DUSB_SIM_ABLATION=n）。
 *       The numbers are at the Ablated(n) sites; for manual ablation builds only
 *       (-DUSB_SIM_ABLATION=n).
 */
constexpr int ABLATION = USB_SIM_ABLATION;

/// 是否在做第 n 号消融 / Whether ablation n is active
constexpr bool Ablated(int n) { return ABLATION == n; }

/// 停止怎样停稳 / How a stop becomes quiescent
enum class StopMode : uint8_t
{
  IMMEDIATE,  ///< StopHardware 当场给出结果 / StopHardware gives the outcome at once
  FRAMES,     ///< PENDING，若干帧后停稳 / PENDING, quiescent after some frames
  /// PENDING，只有复位、断开、挂起或停止超时才结束 / PENDING until a reset, a
  /// disconnect, a suspend or the stop timeout
  NEVER
};

/**
 * @brief 控制端点装入时有未报告的 SETUP / A SETUP not yet reported when a control
 *        endpoint is loaded
 * @note 对应 Endpoint::StartHardware 的控制端点规则 1 和 2。
 *       Rules 1 and 2 of the control endpoint rule of Endpoint::StartHardware.
 */
enum class SetupRule : uint8_t
{
  NAK,     ///< SETUP 报告之前硬件回 NAK / The hardware NAKs until the SETUP is reported
  RETRACT  ///< 硬件会执行，后端撤回并返回失败 / It would run; the backend retracts
};

/// 主机看到的握手 / Handshake seen by the host
enum class Handshake : uint8_t
{
  ACK,
  NAK,
  STALL,
  NONE  ///< 等时事务没有握手 / Isochronous transactions have no handshake
};

/// 数据包 PID / Data packet PID
enum class Pid : uint8_t
{
  DATA0,
  DATA1,
  DATA2,
  MDATA
};

/// IN 令牌的结果 / Result of an IN token
struct InResult
{
  Handshake handshake = Handshake::NAK;
  std::vector<uint8_t> data;
  Pid pid = Pid::DATA0;
};

/// 一个数据端点号的硬件规格 / Hardware specification of one data endpoint number
struct EndpointSpec
{
  uint8_t number = 1;
  Endpoint::Direction direction = Endpoint::Direction::BOTH;
  size_t buffer = 1024;    ///< 每个方向的槽内存字节 / Slot memory bytes per direction
  uint8_t slot_count = 0;  ///< 0：核心默认 / 0: the core default
  /// FIFO 或 PMA 的包长上限，0 只受控制器上限约束 / FIFO or PMA packet limit; 0 leaves
  /// the controller limit only
  size_t packet_limit = 0;
  uint8_t max_requests = 1;  ///< 硬件上同时的请求数 / Requests on the hardware at once
};

/// 控制器的硬件限制和行为 / Hardware limits and behaviour of the controller
struct Profile
{
  std::vector<EndpointSpec> endpoints = {{1, Endpoint::Direction::BOTH, 1024, 0, 0, 1},
                                         {2, Endpoint::Direction::BOTH, 1024, 0, 0, 1},
                                         {3, Endpoint::Direction::BOTH, 1024, 0, 0, 1}};
  size_t ep0_buffer = 64;  ///< EP0 每个方向的内存 / EP0 memory per direction
  size_t max_packet_limit = UINT16_MAX;  ///< 控制器包长上限 / Controller packet limit
  size_t control_packet_limit = 64;      ///< 控制端点包长上限 / Control packet limit
  size_t max_packets = SIZE_MAX;         ///< 一次请求的包数上限 / Packets per request
  size_t max_bytes = SIZE_MAX;           ///< 一次请求的字节上限 / Bytes per request
  bool direct_memory = true;             ///< 硬件直接读写槽 / Hardware accesses slots
  bool isochronous = true;               ///< 支持等时端点 / Isochronous supported
  /// 高速等时和中断端点一个服务区间的包数上限 / Packets per service interval of
  /// high-speed isochronous and interrupt endpoints
  uint8_t max_packets_per_interval = 1;
  /// 同号 IN、OUT 共用端点类型（FSDEV 的 EP_TYPE）/ IN and OUT of one number share the
  /// endpoint type (EP_TYPE on FSDEV)
  bool shared_type_per_number = false;
  /// 等时端点占用整个端点号（FSDEV 的双缓冲）/ An isochronous endpoint takes the
  /// whole number (FSDEV double buffering)
  bool iso_takes_number = false;
  /**
   * @brief 占用 TX FIFO 的 IN 端点号数上限（含 EP0），0 不限 / Limit on IN endpoint
   *        numbers holding a TX FIFO (EP0 included), 0 for none
   * @note 分配在 ConfigureHardware，保持到总线复位、断开或控制器复位。超出时
   *       CheckHardware 返回 NO_BUFF。
   *       Allocated in ConfigureHardware and held until a bus reset, a disconnect or a
   *       controller reset. Beyond it CheckHardware returns NO_BUFF.
   */
  uint8_t in_endpoint_limit = 0;
  StopMode stop_mode = StopMode::IMMEDIATE;
  uint32_t stop_frames = 2;  ///< FRAMES 模式下的帧数 / Frames for StopMode::FRAMES
  /// 停止发出之后硬件还会做完的令牌数（线上的包）/ Tokens the hardware still serves
  /// after the stop is issued (packets on the wire)
  uint32_t stop_grace_tokens = 0;
  /// 停止超时的帧数，报告 TIMEOUT / Frames until a stop times out and reports TIMEOUT
  uint32_t stop_timeout_frames = 64;
  /// 没有被主机取走的等时 IN 区间以 FAILED 结束（否则留在硬件上）/ An isochronous IN
  /// interval the host did not take ends with FAILED (otherwise it stays on the hardware)
  bool iso_in_expire = true;
  SetupRule setup_rule = SetupRule::NAK;
  bool pend_interrupt = false;  ///< PendInterrupt 是否可用 / Whether PendInterrupt works
  /// SET_ADDRESS 的地址在哪个阶段写进硬件 / Stage at which the address reaches the
  /// hardware
  DeviceCore::AddressStage address_stage = DeviceCore::AddressStage::STATUS_COMPLETE;
  /**
   * @brief 调度注入：每次 StartHardware 返回前调用，不持硬件锁 / Schedule injection:
   *        called before every StartHardware returns, without the hardware lock
   * @note 可在这里做主机动作；中断线这时被 owner 屏蔽或会被它屏蔽，事件等 owner 处理。
   *       A host action may be done here; the line is masked by the owner or will be,
   *       and the event waits for the owner.
   */
  std::function<void(uint8_t number, bool in)> on_start;
  /// 同上，在 StartHardware 读硬件状态之前调用 / As above, called before StartHardware
  /// reads the hardware state
  std::function<void(uint8_t number, bool in)> before_start;
};

class Controller;

/**
 * @class SimEndpoint
 * @brief 仿真端点：硬件上最多 max_requests 个请求，主机按包推进 / Simulated endpoint:
 *        up to max_requests requests on the hardware, advanced by the host per packet
 */
class SimEndpoint : public Endpoint
{
 public:
  SimEndpoint(Controller& controller, EPNumber number, Direction direction,
              RawData memory, const EndpointSpec& spec);

  /// @name 后端函数被调用的次数 / Calls of the backend functions
  /// @{
  std::atomic<unsigned> starts{0};
  std::atomic<unsigned> stops{0};
  std::atomic<unsigned> pending_stops{0};
  std::atomic<unsigned> closes{0};
  std::atomic<unsigned> configures{0};
  std::atomic<unsigned> stalls{0};
  std::atomic<unsigned> clears{0};
  std::atomic<unsigned> retracts{0};
  /// @}

  /// 硬件上的请求数 / Requests on the hardware
  size_t OnHardware() const;
  /// 硬件是否置了 STALL / Whether the hardware halts the endpoint
  bool HardwareStalled() const;
  /// 是否有发出、未停稳的停止 / Whether a stop is issued and not quiescent
  bool StopPending() const;
  /// 硬件的数据翻转位 / The hardware's data toggle
  uint8_t Toggle() const;
  /// 硬件当前的包长；未配置时为 0 / Packet size in the hardware; 0 when unconfigured
  size_t HardwarePacket() const;

  /// 向核心报告；只在 ServiceHardware 中 / Report to the core; ServiceHardware only
  void Report(bool in_isr, size_t actual, ErrorCode result)
  {
    OnHardwareComplete(in_isr, actual, result);
  }

 protected:
  ErrorCode CheckHardware(const Config& cfg) const override;
  void ConfigureHardware(const Config& cfg) override;
  void CloseHardware(bool in_isr) override;
  ErrorCode StartHardware(const HwRequest& request, bool in_isr) override;
  StopOutcome StopHardware(bool in_isr) override;
  ErrorCode StallHardware(bool in_isr) override;
  ErrorCode ClearStallHardware(bool in_isr) override;
  HwLimits HardwareLimits(Direction direction, Type type) const override;

 private:
  friend class Controller;

  /// 硬件上的一个请求 / One request on the hardware
  struct Request
  {
    uint8_t* data = nullptr;
    size_t length = 0;
    size_t progress = 0;  ///< 已收发字节 / Bytes moved
    size_t packets = 0;   ///< 本区间已收发的包 / Packets moved in this interval
    bool done = false;    ///< 硬件已做完、尚未报告 / Finished, not reported yet
    bool touched = false;
    ErrorCode result = ErrorCode::OK;
    uint32_t frames = 0;  ///< 装入之后经过的 SOF / SOFs since the load
  };

  bool IsIn() const { return in_; }
  /// 第一个没做完的请求；没有时为空 / First unfinished request; null if none
  Request* Active();
  /// 硬件锁内：停稳，结果留给 ServiceHardware / Under the hardware lock: quiescent, the
  /// outcome left to ServiceHardware
  void EndStopLocked();
  /// 硬件锁内：总线复位、断开冻结硬件 / Under the hardware lock: a bus reset or a
  /// disconnect freezes the hardware
  void QuiesceLocked();
  /// 硬件锁内：控制器复位 / Under the hardware lock: controller reset
  void ResetLocked();
  /// 硬件锁内：一帧 / Under the hardware lock: one frame
  bool FrameLocked();
  Handshake HostOutLocked(const std::vector<uint8_t>& data, Pid pid, bool& latched);
  InResult HostInLocked(bool& latched);
  HwLimits LimitsFor(Type type) const;

  Controller& controller_;
  const EndpointSpec spec_;
  const bool in_;

  /// @name 硬件状态，受 Controller 的硬件锁保护 / Hardware state, under the
  ///       controller's lock
  /// @{
  std::deque<Request> hw_;
  bool enabled_ = false;
  bool stalled_ = false;
  bool quiesced_ = false;     ///< 复位或断开之后，停止当场完成 / After a reset or detach
  bool after_reset_ = false;  ///< 控制器复位之后 / After a controller reset
  bool stop_pending_ = false;
  bool stop_ended_ = false;     ///< 停稳，待按请求逐个报告 / Quiescent, reports due
  bool stop_timeout_ = false;   ///< 待报告 TIMEOUT / TIMEOUT due
  bool stop_failed_ = false;    ///< 已报告 TIMEOUT / TIMEOUT reported
  bool stop_followup_ = false;  ///< 当场停止后还有请求待取结果 / More outcomes due
  uint32_t stop_frames_left_ = 0;
  uint32_t stop_age_ = 0;
  uint32_t grace_left_ = 0;
  uint8_t toggle_ = 0;
  Type type_ = Type::BULK;
  size_t packet_ = 0;
  uint8_t interval_packets_ = 1;
  std::vector<Pid> iso_pids_;  ///< 本区间收到的 PID / PIDs received in this interval
  /// @}
};

/**
 * @class Controller
 * @brief 仿真控制器的硬件部分：端点、主机动作和锁存的事件 / Hardware part of the
 *        simulated controller: endpoints, host actions and latched events
 */
class Controller
{
 public:
  explicit Controller(const Profile& profile);
  virtual ~Controller() = default;
  Controller(const Controller&) = delete;
  Controller& operator=(const Controller&) = delete;

  /// @name 主机动作：任何线程，不得在 owner 中 / Host actions: any thread, never in
  ///       the owner
  /// @{
  void Plug();
  void Unplug();
  void BusReset(Speed speed = Speed::FULL);
  void Suspend();
  void Resume();
  /// 一个（微）帧 / One (micro)frame
  void Sof();
  void Setup(const SetupPacket& setup);
  Handshake OutToken(uint8_t number, const std::vector<uint8_t>& data,
                     Pid pid = Pid::DATA0);
  InResult InToken(uint8_t number);
  /// @}

  /// @name 观察 / Observation
  /// @{
  bool Visible() const;
  bool Suspended() const;
  uint8_t Address() const;
  uint32_t Frame() const;
  Speed BusSpeed() const;
  unsigned ControllerResets() const { return controller_resets_.load(); }
  unsigned ControllerStops() const { return controller_stops_.load(); }
  /// 等时 OUT 没有请求时丢掉的包 / Isochronous OUT packets dropped for want of a request
  unsigned IsoDropped() const { return iso_dropped_.load(); }
  SimEndpoint* In(uint8_t number) const { return ep_in_[number]; }
  SimEndpoint* Out(uint8_t number) const { return ep_out_[number]; }
  const Profile& GetProfile() const { return profile_; }
  /// @}

 protected:
  /// 后端锁存了事件，拉起中断线 / The backend latched an event; raise the line
  virtual void RaiseLine() = 0;

  /// 一个待报告的事件 / One event due for reporting
  struct Event
  {
    enum class Kind : uint8_t
    {
      NONE,
      ENDPOINT,
      RESET,
      SETUP,
      SUSPEND,
      RESUME,
      DISCONNECT,
      SOF
    } kind = Kind::NONE;
    SimEndpoint* ep = nullptr;
    size_t actual = 0;
    ErrorCode result = ErrorCode::OK;
    SetupPacket setup{};
    Speed speed = Speed::FULL;
    uint32_t frame = 0;
  };

  /**
   * @brief 按一次状态读取的顺序取下一个事件 / Take the next event in the order of one
   *        status read
   * @note 顺序：停止的结束（含 TIMEOUT）、总线复位、端点完成（EP0 在前）、SETUP、挂起和
   *       恢复、断开、SOF。锁存的 SETUP 先结束 EP0 未结束的停止（交接规则 5）。
   *       Order: ends of stops (TIMEOUT included), bus reset, endpoint completions (EP0
   *       first), SETUP, suspend and resume, disconnect, SOF. A latched SETUP first ends
   *       the pending stops of EP0 (handover rule 5).
   */
  Event NextEvent();

  /// 控制器复位：硬件静止，之后每个停止立即结束 / Controller reset: the hardware is
  /// quiescent and every later stop ends at once
  void HardwareReset(bool reattach);
  void SetPullup(bool on);
  void WriteAddress(uint8_t address);
  /// 所有端点都空闲、没有停止 / Every endpoint idle with no stop
  bool HardwareIdle() const;

  EndpointPool sim_pool_;
  std::atomic<unsigned> controller_resets_{0};
  std::atomic<unsigned> controller_stops_{0};

 private:
  friend class SimEndpoint;

  /// 端点内存的释放 / Release of endpoint memory
  struct AlignedDelete
  {
    void operator()(uint8_t* p) const { ::operator delete[](p, std::align_val_t(64)); }
  };

  SimEndpoint* Make(uint8_t number, Endpoint::Direction direction,
                    const EndpointSpec& spec);
  SimEndpoint* Find(uint8_t number, bool in) const
  {
    return in ? ep_in_[number] : ep_out_[number];
  }
  bool SetupWindow() const { return setup_window_; }
  void Latch() { latched_ = true; }
  /// 释放硬件锁后按需拉线 / Raise the line, if needed, once the lock is released
  void Flush();

  const Profile profile_;
  mutable std::mutex mutex_;
  std::vector<std::unique_ptr<uint8_t, AlignedDelete>> memory_;
  std::vector<std::unique_ptr<SimEndpoint>> all_;  ///< EP0 OUT、EP0 IN 在前 / EP0 first
  SimEndpoint* ep_in_[16] = {};
  SimEndpoint* ep_out_[16] = {};

  /// @name 硬件状态，受 mutex_ 保护 / Hardware state, under mutex_
  /// @{
  bool vbus_ = true;
  bool pullup_ = false;
  bool sources_on_ = false;  ///< 控制器的中断源 / The controller's interrupt sources
  bool suspended_ = false;
  bool setup_flag_ = false;
  bool setup_window_ = false;
  SetupPacket setup_{};
  bool reset_latched_ = false;
  Speed reset_speed_ = Speed::FULL;
  Speed speed_ = Speed::FULL;
  bool suspend_latched_ = false;
  bool resume_latched_ = false;
  bool disconnect_latched_ = false;
  /// 断开之后又接上并复位：断开先报告 / Detached, then attached and reset: the
  /// disconnect is reported first
  bool disconnect_first_ = false;
  bool sof_latched_ = false;
  uint32_t sof_count_ = 0;
  uint8_t address_ = 0;
  uint32_t tx_fifos_ = 0;  ///< 占用 TX FIFO 的端点号 / Numbers holding a TX FIFO
  bool latched_ = false;   ///< 本次动作锁存了事件 / This action latched an event
  std::atomic<unsigned> iso_dropped_{0};
  /// @}
};

/**
 * @class OwnerCheck
 * @brief 后端函数只由 owner 调用：不同线程不得同时在里面 / Backend functions are called
 *        by the owner only: no two threads may be inside at once
 */
class OwnerCheck
{
 public:
  void Enter()
  {
    const std::thread::id me = std::this_thread::get_id();
    if (thread_.load() == me)
    {
      ++depth_;
      return;
    }
    std::thread::id none{};
    // 两个线程同时在后端函数里：owner 不是串行的 / Two threads inside the backend at
    // once: the owner is not serialized.
    TEST_ASSERT(thread_.compare_exchange_strong(none, me));
    depth_ = 1;
  }
  void Leave()
  {
    if (--depth_ == 0)
    {
      thread_.store(std::thread::id{});
    }
  }

 private:
  std::atomic<std::thread::id> thread_{};
  int depth_ = 0;
};

/**
 * @class SimDevice
 * @brief 仿真控制器加设备服务和设备核心 / Simulated controller with the device service
 *        and the device core
 * @note 中断线：硬件锁存事件后拉起；屏蔽时等着，开线时补进。同一时刻只有一个中断在运行，
 *       发起它的线程就是“中断上下文”。
 *       The interrupt line: raised once the hardware latched an event; while masked it
 *       waits and is taken when unmasked. One interrupt runs at a time, on the thread
 *       that raised it, which is then the "interrupt context".
 */
class SimDevice : public Controller, public DeviceService, public DeviceCore
{
 public:
  SimDevice(
      const Profile& profile, DeviceDescriptor::PacketSize0 packet_size, uint16_t vid,
      uint16_t pid, uint16_t bcd,
      const std::initializer_list<const DescriptorStrings::LanguagePack*>& languages,
      const std::initializer_list<const std::initializer_list<DeviceClass*>>& configs,
      Speed speed = Speed::FULL, ConstRawData uid = {nullptr, 0});

  /// 是否已初始化；只在没有并发的 owner 时读 / Whether initialized; read only with no
  /// concurrent owner
  bool Inited() const { return IsInited(); }
  EndpointPool& Pool() { return sim_pool_; }

  /// @name 报告给设备核心的事件数 / Events reported to the device core
  /// @{
  std::atomic<unsigned> resets{0};
  std::atomic<unsigned> setups{0};
  std::atomic<unsigned> sofs{0};
  std::atomic<unsigned> disconnects{0};
  std::atomic<unsigned> suspends{0};
  std::atomic<unsigned> resumes{0};
  std::atomic<unsigned> interrupts{0};      ///< 进入中断的次数 / Interrupt entries
  std::atomic<unsigned> masked_entries{0};  ///< 屏蔽期间到来的拉线 / Raises while masked
  /// @}

  /// 后端函数的串行检查 / Serialization check of the backend functions
  OwnerCheck owner_check;

  /// 中断线是否被屏蔽 / Whether the interrupt line is masked
  bool LineMasked() const { return line_masked_.load(); }

 protected:
  void ServiceHardware(bool in_isr) override;
  void MaskInterrupt() override;
  void UnmaskInterrupt() override;
  bool PendInterrupt() override;
  void SetAddress(uint8_t address, AddressStage stage) override;
  void ResetController(bool in_isr) override;
  void StartController(bool in_isr) override;
  void StopController(bool in_isr) override;
  void RaiseLine() override;

 private:
  /// 线未屏蔽、有挂起、没有中断在运行时进入中断 / Enter the interrupt if the line is
  /// unmasked, pending and no interrupt runs
  void TryFire();

  std::atomic<bool> line_pending_{false};
  std::atomic<bool> line_masked_{false};
  std::atomic<bool> line_enabled_{false};
  std::atomic<bool> isr_active_{false};
};

/// 后端函数的作用域：串行检查 / Scope of a backend function: serialization check
class BackendScope
{
 public:
  explicit BackendScope(OwnerCheck& check) : check_(check) { check_.Enter(); }
  ~BackendScope() { check_.Leave(); }
  BackendScope(const BackendScope&) = delete;
  BackendScope& operator=(const BackendScope&) = delete;

 private:
  OwnerCheck& check_;
};

inline SimEndpoint::SimEndpoint(Controller& controller, EPNumber number,
                                Direction direction, RawData memory,
                                const EndpointSpec& spec)
    : Endpoint(number, direction, memory, spec.slot_count),
      controller_(controller),
      spec_(spec),
      in_(direction == Direction::IN)
{
}

inline size_t SimEndpoint::OnHardware() const
{
  std::lock_guard<std::mutex> lock(controller_.mutex_);
  return hw_.size();
}

inline bool SimEndpoint::HardwareStalled() const
{
  std::lock_guard<std::mutex> lock(controller_.mutex_);
  return stalled_;
}

inline bool SimEndpoint::StopPending() const
{
  std::lock_guard<std::mutex> lock(controller_.mutex_);
  return stop_pending_ || stop_ended_ || stop_timeout_ || stop_failed_;
}

inline uint8_t SimEndpoint::Toggle() const
{
  std::lock_guard<std::mutex> lock(controller_.mutex_);
  return toggle_;
}

inline size_t SimEndpoint::HardwarePacket() const
{
  std::lock_guard<std::mutex> lock(controller_.mutex_);
  return enabled_ ? packet_ : 0U;
}

inline Endpoint::HwLimits SimEndpoint::LimitsFor(Type type) const
{
  const Profile& profile = controller_.profile_;
  HwLimits limits;
  size_t packet = profile.max_packet_limit;
  if (spec_.packet_limit != 0U && !Ablated(22))
  {
    packet = std::min(packet, spec_.packet_limit);
  }
  limits.direct_memory = profile.direct_memory && !Ablated(9);
  if (type == Type::CONTROL)
  {
    limits.max_packet_size = std::min(packet, profile.control_packet_limit);
    limits.max_packets = 1U;
    limits.max_bytes = limits.max_packet_size;
    return limits;
  }
  limits.max_packet_size = packet;
  limits.max_packets = profile.max_packets;
  limits.max_bytes = std::max(profile.max_bytes, packet);
  limits.max_requests = Ablated(12) || Ablated(24) ? uint8_t{1} : spec_.max_requests;
  if (type == Type::ISOCHRONOUS || type == Type::INTR)
  {
    limits.max_packets_per_interval = profile.max_packets_per_interval;
  }
  if (type == Type::ISOCHRONOUS)
  {
    limits.max_packets = std::max<size_t>(limits.max_packets, 1U);
  }
  return limits;
}

inline Endpoint::HwLimits SimEndpoint::HardwareLimits(Direction direction,
                                                      Type type) const
{
  UNUSED(direction);
  return LimitsFor(type);
}

inline ErrorCode SimEndpoint::CheckHardware(const Config& cfg) const
{
  auto& device = static_cast<SimDevice&>(controller_);
  BackendScope scope(device.owner_check);
  const Profile& profile = controller_.profile_;
  if (cfg.type == Type::ISOCHRONOUS && !profile.isochronous)
  {
    return ErrorCode::NOT_SUPPORT;
  }
  const uint8_t number = EPNumberToInt8(GetNumber());
  if (profile.iso_takes_number && number != 0U && !Ablated(21))
  {
    const SimEndpoint* other = controller_.Find(number, cfg.direction != Direction::IN);
    const bool other_on = other != nullptr && other->GetState() != State::DISABLED;
    if (other_on &&
        (cfg.type == Type::ISOCHRONOUS || other->GetType() == Type::ISOCHRONOUS))
    {
      return ErrorCode::STATE_ERR;
    }
  }
  if (profile.shared_type_per_number && number != 0U && !Ablated(20))
  {
    const SimEndpoint* other = controller_.Find(number, cfg.direction != Direction::IN);
    if (other != nullptr && other->GetState() != State::DISABLED &&
        other->GetType() != cfg.type)
    {
      return ErrorCode::FAILED;
    }
  }
  if (cfg.direction == Direction::IN && profile.in_endpoint_limit != 0U)
  {
    // 已占用的，加上已配置、硬件配置可能推迟到停止结束的 IN 端点：它们之后都会占用。
    // Those held, plus the IN endpoints configured whose hardware configuration may
    // wait for the end of a stop: all of them will hold one.
    std::lock_guard<std::mutex> lock(controller_.mutex_);
    uint32_t held = controller_.tx_fifos_ | 1U;
    for (uint8_t n = 1; n < 16; ++n)
    {
      const SimEndpoint* ep = controller_.ep_in_[n];
      if (ep != nullptr && ep != this && ep->GetState() != State::DISABLED)
      {
        held |= 1U << n;
      }
    }
    if ((held & (1U << number)) == 0U &&
        static_cast<unsigned>(__builtin_popcount(held)) >= profile.in_endpoint_limit)
    {
      return ErrorCode::NO_BUFF;
    }
  }
  return ErrorCode::OK;
}

inline void SimEndpoint::ConfigureHardware(const Config& cfg)
{
  auto& device = static_cast<SimDevice&>(controller_);
  BackendScope scope(device.owner_check);
  std::lock_guard<std::mutex> lock(controller_.mutex_);
  // 契约：只在没有在途请求、没有停止时 / Contract: nothing in flight, no stop.
  TEST_ASSERT(hw_.empty() && !stop_pending_ && !stop_ended_ && !stop_failed_);
  TEST_ASSERT(cfg.direction == (in_ ? Direction::IN : Direction::OUT));
  const uint8_t number = EPNumberToInt8(GetNumber());
  const Profile& profile = controller_.profile_;
  if (profile.shared_type_per_number && number != 0U)
  {
    // 共用的类型字段按 cfg 写：另一方向的硬件若还在用别的类型，就被改坏了。
    // The shared type field is written from cfg: had the other direction's hardware
    // still used another type, it would be clobbered now.
    const SimEndpoint* other = controller_.Find(number, !in_);
    TEST_ASSERT(other == nullptr || !other->enabled_ || other->type_ == cfg.type);
  }
  if (in_ && profile.in_endpoint_limit != 0U)
  {
    controller_.tx_fifos_ |= 1U << number;
    TEST_ASSERT(static_cast<unsigned>(__builtin_popcount(controller_.tx_fifos_ | 1U)) <=
                profile.in_endpoint_limit);
  }
  enabled_ = true;
  stalled_ = false;
  quiesced_ = false;
  after_reset_ = false;
  toggle_ = 0;
  type_ = cfg.type;
  packet_ = cfg.max_packet_size;
  interval_packets_ = cfg.max_packets_per_interval;
  iso_pids_.clear();
  ++configures;
}

inline void SimEndpoint::CloseHardware(bool in_isr)
{
  UNUSED(in_isr);
  auto& device = static_cast<SimDevice&>(controller_);
  BackendScope scope(device.owner_check);
  std::lock_guard<std::mutex> lock(controller_.mutex_);
  // 契约：只在没有在途请求、没有未结束的停止时 / Contract: nothing in flight and no
  // stop pending.
  TEST_ASSERT(hw_.empty() && !stop_pending_ && !stop_ended_);
  enabled_ = false;
  stalled_ = false;
  if (Ablated(23) && in_)
  {
    controller_.tx_fifos_ &= ~(1U << EPNumberToInt8(GetNumber()));
  }
  ++closes;
}

inline ErrorCode SimEndpoint::StartHardware(const HwRequest& request, bool in_isr)
{
  UNUSED(in_isr);
  auto& device = static_cast<SimDevice&>(controller_);
  BackendScope scope(device.owner_check);
  if (controller_.profile_.before_start)
  {
    controller_.profile_.before_start(EPNumberToInt8(GetNumber()), in_);
  }
  {
    std::lock_guard<std::mutex> lock(controller_.mutex_);
    // 契约：已配置、没有停止、没有 STALL、硬件上的请求数在上限之下。
    // Contract: configured, no stop, not halted, fewer requests than the limit.
    // 总线复位或断开已让硬件停用端点、核心还没收到报告时也可能来：请求留在硬件上不动，
    // 之后的停止当场完成。
    // It may also come after a bus reset or a disconnect has deactivated the endpoint
    // in hardware and before the core got the report: the request stays on the
    // hardware without moving, and the later stop completes at once.
    TEST_ASSERT((enabled_ || quiesced_) && !after_reset_);
    TEST_ASSERT(!stop_pending_ && !stop_ended_ && !stop_failed_ && !stop_followup_);
    TEST_ASSERT(!stalled_);
    const HwLimits limits = LimitsFor(type_);
    const size_t max_requests = type_ == Type::CONTROL ? 1U : limits.max_requests;
    TEST_ASSERT(hw_.size() < max_requests);
    TEST_ASSERT(request.data != nullptr);
    TEST_ASSERT(request.length <= limits.max_bytes);
    if (!in_)
    {
      // 契约：OUT 请求是包长的整数倍，0 只用于控制端点。
      // Contract: an OUT request is a packet multiple, 0 for control endpoints only.
      TEST_ASSERT(packet_ != 0U && request.length % packet_ == 0U);
      TEST_ASSERT(request.length != 0U || type_ == Type::CONTROL);
    }
    if (type_ == Type::ISOCHRONOUS)
    {
      TEST_ASSERT(request.length <= packet_ * interval_packets_);
    }
    if (GetNumber() == EPNumber::EP0 && controller_.setup_flag_ &&
        controller_.profile_.setup_rule == SetupRule::RETRACT && !Ablated(11))
    {
      // 规则 2：有未报告的 SETUP，撤回这次装入。
      // Rule 2: an unreported SETUP is latched, so the load is retracted.
      ++retracts;
      return ErrorCode::FAILED;
    }
    Request r;
    r.data = request.data;
    r.length = request.length;
    hw_.push_back(r);
    ++starts;
  }
  if (controller_.profile_.on_start)
  {
    controller_.profile_.on_start(EPNumberToInt8(GetNumber()), in_);
  }
  return ErrorCode::OK;
}

inline Endpoint::StopOutcome SimEndpoint::StopHardware(bool in_isr)
{
  UNUSED(in_isr);
  auto& device = static_cast<SimDevice&>(controller_);
  BackendScope scope(device.owner_check);
  std::lock_guard<std::mutex> lock(controller_.mutex_);
  ++stops;
  if (after_reset_)
  {
    // 交接规则 3：控制器复位之后立即返回 / Handover rule 3: at once after the reset.
    hw_.clear();
    stop_pending_ = stop_ended_ = stop_timeout_ = stop_failed_ = false;
    stop_followup_ = false;
    return {ErrorCode::ABORTED, 0U};
  }
  // 契约：硬件上有请求，没有未结束的停止 / Contract: a request on the hardware and no
  // stop pending.
  TEST_ASSERT(!hw_.empty());
  if (!stop_followup_)
  {
    TEST_ASSERT(!stop_pending_ && !stop_ended_ && !stop_timeout_ && !stop_failed_);
    bool all_done = true;
    for (const Request& r : hw_)
    {
      all_done = all_done && r.done;
    }
    const bool ep0 = GetNumber() == EPNumber::EP0;
    const bool now = controller_.profile_.stop_mode == StopMode::IMMEDIATE || quiesced_ ||
                     all_done || (ep0 && controller_.SetupWindow() && !Ablated(3));
    if (!now)
    {
      stop_pending_ = true;
      stop_frames_left_ = controller_.profile_.stop_frames;
      stop_age_ = 0;
      grace_left_ = controller_.profile_.stop_grace_tokens;
      ++pending_stops;
      return {ErrorCode::PENDING, 0U};
    }
  }
  const Request r = hw_.front();
  hw_.pop_front();
  stop_followup_ = !hw_.empty();
  iso_pids_.clear();
  if (r.done && !Ablated(6))
  {
    return {r.result, r.progress};
  }
  return {ErrorCode::ABORTED, r.progress};
}

inline ErrorCode SimEndpoint::StallHardware(bool in_isr)
{
  UNUSED(in_isr);
  auto& device = static_cast<SimDevice&>(controller_);
  BackendScope scope(device.owner_check);
  std::lock_guard<std::mutex> lock(controller_.mutex_);
  // 契约：Stall 先结束全部请求 / Contract: Stall ends every request first.
  TEST_ASSERT(hw_.empty() && !stop_pending_ && !stop_ended_);
  stalled_ = true;
  ++stalls;
  return ErrorCode::OK;
}

inline ErrorCode SimEndpoint::ClearStallHardware(bool in_isr)
{
  UNUSED(in_isr);
  auto& device = static_cast<SimDevice&>(controller_);
  BackendScope scope(device.owner_check);
  std::lock_guard<std::mutex> lock(controller_.mutex_);
  // 契约：没有停止未结束；硬件上的请求留着 / Contract: no stop pending; requests on
  // the hardware stay.
  TEST_ASSERT(!stop_pending_ && !stop_ended_ && !stop_failed_);
  stalled_ = false;
  if (!Ablated(1))
  {
    toggle_ = 0;
  }
  ++clears;
  return ErrorCode::OK;
}

inline SimEndpoint::Request* SimEndpoint::Active()
{
  for (Request& r : hw_)
  {
    if (!r.done)
    {
      return &r;
    }
  }
  return nullptr;
}

inline void SimEndpoint::EndStopLocked()
{
  if (!stop_pending_)
  {
    return;
  }
  stop_pending_ = false;
  stop_ended_ = !hw_.empty();
  iso_pids_.clear();
  controller_.Latch();
}

inline void SimEndpoint::QuiesceLocked()
{
  if (!Ablated(2))
  {
    EndStopLocked();
  }
  quiesced_ = true;
  enabled_ = false;
  stalled_ = false;
  iso_pids_.clear();
}

inline void SimEndpoint::ResetLocked()
{
  hw_.clear();
  stop_pending_ = stop_ended_ = stop_timeout_ = stop_failed_ = false;
  stop_followup_ = false;
  after_reset_ = true;
  quiesced_ = false;
  enabled_ = false;
  stalled_ = false;
  iso_pids_.clear();
}

inline bool SimEndpoint::FrameLocked()
{
  bool latched = false;
  const Profile& profile = controller_.profile_;
  if (stop_pending_)
  {
    ++stop_age_;
    if (profile.stop_mode == StopMode::FRAMES && stop_frames_left_ != 0U &&
        --stop_frames_left_ == 0U)
    {
      EndStopLocked();
      return true;
    }
    if (stop_age_ >= profile.stop_timeout_frames && !Ablated(5))
    {
      // 超过上界仍未停稳：报告一次 TIMEOUT / Past the bound and not quiescent: one
      // TIMEOUT.
      stop_pending_ = false;
      stop_timeout_ = true;
      return true;
    }
    return false;
  }
  if (!enabled_ || type_ != Type::ISOCHRONOUS || stop_ended_ || stop_failed_)
  {
    return false;
  }
  Request* r = Active();
  if (r == nullptr)
  {
    return false;
  }
  ++r->frames;
  if (in_)
  {
    // 区间结束时没有发完：FAILED；从未取过的要等它整整一个区间。
    // An interval ending unfinished: FAILED; one never taken gets one whole interval.
    if (r->touched || (profile.iso_in_expire && !Ablated(15) && r->frames >= 2U))
    {
      r->done = true;
      r->result = ErrorCode::FAILED;
      latched = true;
    }
  }
  else if (!iso_pids_.empty())
  {
    // 区间里只收到了 MDATA：PID 序列不对 / Only MDATA in the interval: a wrong PID
    // sequence.
    r->done = true;
    r->result = ErrorCode::FAILED;
    iso_pids_.clear();
    latched = true;
  }
  return latched;
}

inline Handshake SimEndpoint::HostOutLocked(const std::vector<uint8_t>& data, Pid pid,
                                            bool& latched)
{
  latched = false;
  const bool control = GetNumber() == EPNumber::EP0;
  if (!enabled_)
  {
    return Handshake::NAK;
  }
  if (type_ == Type::ISOCHRONOUS)
  {
    Request* r = Active();
    if (r == nullptr || stop_pending_ || stop_ended_ || stop_failed_)
    {
      ++controller_.iso_dropped_;
      return Handshake::NONE;
    }
    TEST_ASSERT(data.size() <= packet_);
    const size_t room = r->length - r->progress;
    const size_t take = std::min(room, data.size());
    if (take != 0U)
    {
      std::memcpy(r->data + r->progress, data.data(), take);
    }
    r->progress += take;
    r->touched = true;
    iso_pids_.push_back(pid);
    if (data.size() > room)
    {
      r->done = true;
      r->result = ErrorCode::OUT_OF_RANGE;
      iso_pids_.clear();
      latched = true;
      return Handshake::NONE;
    }
    if (pid != Pid::MDATA)
    {
      // 区间的最后一个包：核对 PID 序列（USB 2.0 5.9.2）/ The last packet of the
      // interval: check the PID sequence (USB 2.0 5.9.2).
      const size_t n = iso_pids_.size();
      const Pid expect = n == 1U ? Pid::DATA0 : (n == 2U ? Pid::DATA1 : Pid::DATA2);
      bool ok = Ablated(8) || (n <= interval_packets_ && pid == expect);
      for (size_t i = 0; i + 1U < n; ++i)
      {
        ok = ok && iso_pids_[i] == Pid::MDATA;
      }
      r->done = true;
      r->result = ok ? ErrorCode::OK : ErrorCode::FAILED;
      iso_pids_.clear();
      latched = true;
    }
    return Handshake::NONE;
  }
  if (stalled_)
  {
    return Handshake::STALL;
  }
  if (control && controller_.setup_flag_ &&
      controller_.profile_.setup_rule == SetupRule::NAK)
  {
    return Handshake::NAK;
  }
  if (quiesced_ || stop_ended_ || stop_failed_)
  {
    return Handshake::NAK;
  }
  if (stop_pending_)
  {
    if (grace_left_ == 0U && !Ablated(10))
    {
      return Handshake::NAK;
    }
    --grace_left_;
  }
  Request* r = Active();
  if (r == nullptr)
  {
    return Handshake::NAK;
  }
  TEST_ASSERT(data.size() <= packet_);
  if (!control)
  {
    if (static_cast<uint8_t>(pid) != toggle_ && !Ablated(7))
    {
      // 翻转位不符：重发的包，确认但丢弃（USB 2.0 8.6.4）/ Toggle mismatch: a
      // retransmission, acknowledged and dropped (USB 2.0 8.6.4).
      return Handshake::ACK;
    }
    toggle_ ^= 1U;
  }
  const size_t room = r->length - r->progress;
  const size_t take = std::min(room, data.size());
  if (take != 0U)
  {
    std::memcpy(r->data + r->progress, data.data(), take);
  }
  r->progress += take;
  if (data.size() > room)
  {
    r->done = true;
    r->result = ErrorCode::OUT_OF_RANGE;
  }
  else if (data.size() < packet_ || r->progress >= r->length)
  {
    r->done = true;
  }
  latched = r->done && !stop_pending_;
  return Handshake::ACK;
}

inline InResult SimEndpoint::HostInLocked(bool& latched)
{
  latched = false;
  InResult result;
  const bool control = GetNumber() == EPNumber::EP0;
  if (!enabled_)
  {
    return result;
  }
  if (type_ == Type::ISOCHRONOUS)
  {
    result.handshake = Handshake::ACK;
    Request* r = Active();
    if (r == nullptr || stop_pending_ || stop_ended_ || stop_failed_)
    {
      // 没有数据：零长度包 / No data: a zero-length packet.
      return result;
    }
    const size_t rest = r->length - r->progress;
    const size_t give = std::min(rest, packet_);
    const size_t total = r->length == 0U ? 1U : (r->length + packet_ - 1U) / packet_;
    const size_t left = total - r->packets;
    result.pid = left == 1U ? Pid::DATA0 : (left == 2U ? Pid::DATA1 : Pid::DATA2);
    result.data.assign(r->data + r->progress, r->data + r->progress + give);
    r->progress += give;
    ++r->packets;
    r->touched = true;
    if (r->packets >= total)
    {
      r->done = true;
      latched = true;
    }
    return result;
  }
  if (stalled_)
  {
    result.handshake = Handshake::STALL;
    return result;
  }
  if (control && controller_.setup_flag_ &&
      controller_.profile_.setup_rule == SetupRule::NAK)
  {
    return result;
  }
  if (quiesced_ || stop_ended_ || stop_failed_)
  {
    return result;
  }
  if (stop_pending_)
  {
    if (grace_left_ == 0U && !Ablated(10))
    {
      return result;
    }
    --grace_left_;
  }
  Request* r = Active();
  if (r == nullptr)
  {
    return result;
  }
  const size_t rest = r->length - r->progress;
  const size_t give = std::min(rest, packet_);
  result.data.assign(r->data + r->progress, r->data + r->progress + give);
  result.pid = toggle_ == 0U ? Pid::DATA0 : Pid::DATA1;
  result.handshake = Handshake::ACK;
  if (!control)
  {
    toggle_ ^= 1U;
  }
  r->progress += give;
  // 最后一个包（含零长度包）被主机取走即完成 / Done once the host took the last
  // packet, a zero-length one included.
  if (r->progress >= r->length)
  {
    r->done = true;
    latched = !stop_pending_;
  }
  return result;
}

inline Controller::Controller(const Profile& profile) : profile_(profile)
{
  EndpointSpec ep0;
  ep0.number = 0;
  ep0.buffer = profile_.ep0_buffer;
  ep0.slot_count = 0;
  SimEndpoint* out0 = Make(0, Endpoint::Direction::OUT, ep0);
  SimEndpoint* in0 = Make(0, Endpoint::Direction::IN, ep0);
  sim_pool_.SetEndpoint0(in0, out0);
  for (const EndpointSpec& spec : profile_.endpoints)
  {
    TEST_ASSERT(spec.number >= 1U && spec.number < 16U);
    if (spec.direction != Endpoint::Direction::IN)
    {
      TEST_ASSERT(sim_pool_.Put(Make(spec.number, Endpoint::Direction::OUT, spec)) ==
                  ErrorCode::OK);
    }
    if (spec.direction != Endpoint::Direction::OUT)
    {
      TEST_ASSERT(sim_pool_.Put(Make(spec.number, Endpoint::Direction::IN, spec)) ==
                  ErrorCode::OK);
    }
  }
}

inline SimEndpoint* Controller::Make(uint8_t number, Endpoint::Direction direction,
                                     const EndpointSpec& spec)
{
  const size_t size = (spec.buffer + 63U) / 64U * 64U;
  auto* memory = static_cast<uint8_t*>(::operator new[](size, std::align_val_t(64)));
  std::memset(memory, 0, size);
  memory_.emplace_back(memory);
  all_.emplace_back(new SimEndpoint(*this, static_cast<Endpoint::EPNumber>(number),
                                    direction, RawData{memory, size}, spec));
  SimEndpoint* ep = all_.back().get();
  (direction == Endpoint::Direction::IN ? ep_in_ : ep_out_)[number] = ep;
  return ep;
}

inline void Controller::Flush()
{
  bool raise = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    raise = latched_ && sources_on_;
    latched_ = false;
  }
  if (raise)
  {
    RaiseLine();
  }
}

inline void Controller::Plug()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    vbus_ = true;
  }
}

inline void Controller::Unplug()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    vbus_ = false;
    suspended_ = false;
    setup_flag_ = false;
    tx_fifos_ = 0;
    address_ = 0;
    // 交接规则 4：断开之前结束所有未结束的停止 / Handover rule 4: every pending stop
    // ends before the disconnect is reported.
    for (auto& ep : all_)
    {
      ep->QuiesceLocked();
    }
    disconnect_latched_ = true;
    latched_ = true;
  }
  Flush();
}

inline void Controller::BusReset(Speed speed)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    TEST_ASSERT(vbus_ && pullup_);
    suspended_ = false;
    suspend_latched_ = resume_latched_ = false;
    setup_flag_ = false;
    tx_fifos_ = 0;
    address_ = 0;
    speed_ = speed;
    for (auto& ep : all_)
    {
      ep->QuiesceLocked();
    }
    disconnect_first_ = disconnect_latched_;
    reset_latched_ = true;
    reset_speed_ = speed;
    latched_ = true;
  }
  Flush();
}

inline void Controller::Suspend()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (suspended_)
    {
      return;
    }
    suspended_ = true;
    // 总线空闲是所有端点的停稳证据 / An idle bus is evidence of quiescence for every
    // endpoint.
    for (auto& ep : all_)
    {
      if (!Ablated(4))
      {
        ep->EndStopLocked();
      }
    }
    suspend_latched_ = true;
    latched_ = true;
  }
  Flush();
}

inline void Controller::Resume()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!suspended_)
    {
      return;
    }
    suspended_ = false;
    resume_latched_ = true;
    latched_ = true;
  }
  Flush();
}

inline void Controller::Sof()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    // 挂起期间、看不见设备时主机不发 SOF / No SOF while suspended or detached.
    TEST_ASSERT(!suspended_ && vbus_ && pullup_);
    ++sof_count_;
    for (auto& ep : all_)
    {
      if (ep->FrameLocked())
      {
        latched_ = true;
      }
    }
    sof_latched_ = true;
    latched_ = true;
  }
  Flush();
}

inline void Controller::Setup(const SetupPacket& setup)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    TEST_ASSERT(vbus_ && pullup_ && !suspended_);
    setup_ = setup;
    setup_flag_ = true;
    // SETUP 总是被接收，并清除控制端点的 STALL / A SETUP is always taken and clears
    // the halt of the control endpoint.
    ep_in_[0]->stalled_ = false;
    ep_out_[0]->stalled_ = false;
    latched_ = true;
  }
  Flush();
}

inline Handshake Controller::OutToken(uint8_t number, const std::vector<uint8_t>& data,
                                      Pid pid)
{
  Handshake handshake = Handshake::NAK;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    TEST_ASSERT(vbus_ && pullup_ && !suspended_);
    SimEndpoint* ep = ep_out_[number];
    TEST_ASSERT(ep != nullptr);
    bool latched = false;
    handshake = ep->HostOutLocked(data, pid, latched);
    latched_ = latched_ || latched;
  }
  Flush();
  return handshake;
}

inline InResult Controller::InToken(uint8_t number)
{
  InResult result;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    TEST_ASSERT(vbus_ && pullup_ && !suspended_);
    SimEndpoint* ep = ep_in_[number];
    TEST_ASSERT(ep != nullptr);
    bool latched = false;
    result = ep->HostInLocked(latched);
    latched_ = latched_ || latched;
  }
  Flush();
  return result;
}

inline bool Controller::Visible() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return vbus_ && pullup_;
}

inline bool Controller::Suspended() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return suspended_;
}

inline uint8_t Controller::Address() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return address_;
}

inline uint32_t Controller::Frame() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return sof_count_;
}

inline Speed Controller::BusSpeed() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return speed_;
}

inline Controller::Event Controller::NextEvent()
{
  std::lock_guard<std::mutex> lock(mutex_);
  setup_window_ = false;
  Event event;
  if (setup_flag_)
  {
    ep_out_[0]->EndStopLocked();
    ep_in_[0]->EndStopLocked();
  }
  for (auto& ep : all_)
  {
    if (ep->stop_timeout_)
    {
      ep->stop_timeout_ = false;
      ep->stop_failed_ = true;
      event.kind = Event::Kind::ENDPOINT;
      event.ep = ep.get();
      event.result = ErrorCode::TIMEOUT;
      return event;
    }
    if (ep->stop_ended_)
    {
      TEST_ASSERT(!ep->hw_.empty());
      const SimEndpoint::Request r = ep->hw_.front();
      ep->hw_.pop_front();
      ep->stop_ended_ = !ep->hw_.empty();
      event.kind = Event::Kind::ENDPOINT;
      event.ep = ep.get();
      event.actual = r.progress;
      event.result = r.done ? r.result : ErrorCode::ABORTED;
      return event;
    }
  }
  if (disconnect_latched_ && disconnect_first_ && !Ablated(25))
  {
    // 一次读取里既有断开又有之后的复位：按发生的先后报告，否则复位之后的断开会让刚
    // 复位的设备回到未初始化。
    // A disconnect and a later reset in one read: reported in the order they happened,
    // else a disconnect after the reset would undo the device just reset.
    disconnect_latched_ = disconnect_first_ = false;
    event.kind = Event::Kind::DISCONNECT;
    return event;
  }
  if (reset_latched_)
  {
    reset_latched_ = false;
    event.kind = Event::Kind::RESET;
    event.speed = reset_speed_;
    return event;
  }
  for (auto& ep : all_)
  {
    if (!ep->stop_pending_ && !ep->stop_failed_ && !ep->stop_followup_ &&
        !ep->hw_.empty() && ep->hw_.front().done)
    {
      const SimEndpoint::Request r = ep->hw_.front();
      ep->hw_.pop_front();
      event.kind = Event::Kind::ENDPOINT;
      event.ep = ep.get();
      event.actual = r.progress;
      event.result = r.result;
      return event;
    }
  }
  if (setup_flag_)
  {
    setup_flag_ = false;
    setup_window_ = true;
    event.kind = Event::Kind::SETUP;
    event.setup = setup_;
    return event;
  }
  if (suspend_latched_ || resume_latched_)
  {
    // 两个都在时，后报告的与此刻的状态一致 / With both, the one reported last agrees
    // with the state now.
    const bool both = suspend_latched_ && resume_latched_;
    const bool report_suspend = both ? !suspended_ : suspend_latched_;
    (report_suspend ? suspend_latched_ : resume_latched_) = false;
    event.kind = report_suspend ? Event::Kind::SUSPEND : Event::Kind::RESUME;
    return event;
  }
  if (disconnect_latched_)
  {
    disconnect_latched_ = false;
    event.kind = Event::Kind::DISCONNECT;
    return event;
  }
  if (sof_latched_)
  {
    sof_latched_ = false;
    event.kind = Event::Kind::SOF;
    event.frame = sof_count_;
    return event;
  }
  return event;
}

inline void Controller::HardwareReset(bool reattach)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto& ep : all_)
    {
      ep->ResetLocked();
    }
    setup_flag_ = setup_window_ = false;
    reset_latched_ = suspend_latched_ = resume_latched_ = false;
    disconnect_first_ = false;
    disconnect_latched_ = sof_latched_ = false;
    tx_fifos_ = 0;
    address_ = 0;
    suspended_ = false;
    pullup_ = reattach;
    sources_on_ = reattach;
    latched_ = false;
  }
}

inline void Controller::SetPullup(bool on)
{
  std::lock_guard<std::mutex> lock(mutex_);
  pullup_ = on;
  sources_on_ = on;
}

inline void Controller::WriteAddress(uint8_t address)
{
  std::lock_guard<std::mutex> lock(mutex_);
  address_ = address;
}

inline bool Controller::HardwareIdle() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto& ep : all_)
  {
    if (!ep->hw_.empty() || ep->stop_pending_ || ep->stop_ended_ || ep->stop_failed_)
    {
      return false;
    }
  }
  return true;
}

inline SimDevice::SimDevice(
    const Profile& profile, DeviceDescriptor::PacketSize0 packet_size, uint16_t vid,
    uint16_t pid, uint16_t bcd,
    const std::initializer_list<const DescriptorStrings::LanguagePack*>& languages,
    const std::initializer_list<const std::initializer_list<DeviceClass*>>& configs,
    Speed speed, ConstRawData uid)
    : Controller(profile),
      DeviceService(),
      DeviceCore(*this, sim_pool_, USBSpec::USB_2_1, speed, packet_size, vid, pid, bcd,
                 languages, configs, uid)
{
  for (uint8_t n = 0; n < 16; ++n)
  {
    if (Out(n) != nullptr)
    {
      TEST_ASSERT(Register(*Out(n)) == ErrorCode::OK);
    }
    if (In(n) != nullptr)
    {
      TEST_ASSERT(Register(*In(n)) == ErrorCode::OK);
    }
  }
}

inline void SimDevice::RaiseLine()
{
  if (line_masked_.load())
  {
    ++masked_entries;
  }
  line_pending_.store(true);
  TryFire();
}

inline void SimDevice::TryFire()
{
  while (line_pending_.load() && !line_masked_.load() && line_enabled_.load())
  {
    bool idle = false;
    if (!isr_active_.compare_exchange_strong(idle, true))
    {
      // 别的中断在运行，它返回前会再看一次 / Another interrupt runs and looks again
      // before it leaves.
      return;
    }
    ++interrupts;
    OnControllerInterrupt();
    isr_active_.store(false);
  }
}

inline void SimDevice::MaskInterrupt() { line_masked_.store(!Ablated(16)); }

inline void SimDevice::UnmaskInterrupt()
{
  line_masked_.store(false);
  TryFire();
}

inline bool SimDevice::PendInterrupt()
{
  if (!GetProfile().pend_interrupt)
  {
    return false;
  }
  line_pending_.store(true);
  TryFire();
  return true;
}

inline void SimDevice::ServiceHardware(bool in_isr)
{
  BackendScope scope(owner_check);
  line_pending_.store(false);
  while (true)
  {
    const Event event = NextEvent();
    switch (event.kind)
    {
      case Event::Kind::NONE:
        return;
      case Event::Kind::ENDPOINT:
        event.ep->Report(in_isr, event.actual, event.result);
        break;
      case Event::Kind::RESET:
        ++resets;
        OnBusReset(in_isr, event.speed);
        break;
      case Event::Kind::SETUP:
        ++setups;
        OnSetup(in_isr, event.setup);
        break;
      case Event::Kind::SUSPEND:
        ++suspends;
        OnSuspend(in_isr);
        break;
      case Event::Kind::RESUME:
        ++resumes;
        OnResume(in_isr);
        break;
      case Event::Kind::DISCONNECT:
        ++disconnects;
        OnDisconnect(in_isr);
        break;
      case Event::Kind::SOF:
      {
        ++sofs;
        const bool high = BusSpeed() == Speed::HIGH;
        const uint32_t frame = high ? event.frame / 8U : event.frame;
        OnStartOfFrame(in_isr, static_cast<uint16_t>(frame & 0x7FFU),
                       high ? static_cast<uint8_t>(event.frame % 8U) : 0xFFU);
        break;
      }
    }
  }
}

inline void SimDevice::SetAddress(uint8_t address, AddressStage stage)
{
  BackendScope scope(owner_check);
  if (stage == GetProfile().address_stage)
  {
    WriteAddress(address);
  }
}

inline void SimDevice::ResetController(bool in_isr)
{
  UNUSED(in_isr);
  BackendScope scope(owner_check);
  ++controller_resets_;
  // 复位后重新接上；主机看到断开再接上，重新枚举。
  // Attached again after the reset; the host sees a detach and an attach and
  // enumerates again.
  HardwareReset(true);
}

inline void SimDevice::StartController(bool in_isr)
{
  UNUSED(in_isr);
  SetPullup(true);
  line_enabled_.store(true);
  TryFire();
}

inline void SimDevice::StopController(bool in_isr)
{
  UNUSED(in_isr);
  BackendScope scope(owner_check);
  ++controller_stops_;
  // 平常的 Stop：所有端点已关闭、没有停止。停止失败时代替复位：硬件静止。
  // An ordinary Stop: every endpoint closed and no stop. After a failed stop it stands
  // for the reset: the hardware is made quiescent.
  bool failed = false;
  for (uint8_t n = 0; n < 16; ++n)
  {
    for (SimEndpoint* ep : {In(n), Out(n)})
    {
      failed = failed || (ep != nullptr && ep->StopPending());
    }
  }
  if (!failed)
  {
    TEST_ASSERT(HardwareIdle());
  }
  HardwareReset(Ablated(17));
}

inline SetupPacket MakeSetup(uint8_t type, uint8_t request, uint16_t value,
                             uint16_t index, uint16_t length)
{
  SetupPacket packet{};
  packet.bmRequestType = type;
  packet.bRequest = request;
  packet.wValue = value;
  packet.wIndex = index;
  packet.wLength = length;
  return packet;
}

/**
 * @class Host
 * @brief 主机的事务层 / The host transaction layer
 * @note 控制传输、按包长拆分的批量传输、按端点的数据翻转位；设备 NAK 时重试，至少
 *       RETRIES 次并持续 RETRY_TIME（owner 可能正被别的线程占着）。auto_frames 时每
 *       NAKS_PER_FRAME 次 NAK 过一帧，如同真实主机的重试跨过帧。
 *       Control transfers, bulk transfers split by the packet size, a data toggle per
 *       endpoint; a NAK is retried at least RETRIES times and for RETRY_TIME (another
 *       thread may hold the owner). With auto_frames one frame passes every
 *       NAKS_PER_FRAME NAKs, as a real host's retries cross frames.
 */
class Host
{
 public:
  static constexpr unsigned RETRIES = 16;
  static constexpr unsigned NAKS_PER_FRAME = 4;

  explicit Host(Controller& device, uint16_t ep0_packet = 64)
      : device_(device), ep0_packet_(ep0_packet)
  {
  }

  bool auto_frames = true;             ///< 重试时推进帧 / Advance frames on retries
  const char* last_failure = nullptr;  ///< 最近失败的步骤 / Step that failed last
  unsigned duplicates = 0;             ///< 翻转位不符丢弃的 IN 包 / IN dropped
  Handshake last_handshake = Handshake::ACK;  ///< 最近一次握手 / The latest handshake

  /// 控制读；STALL 时 ok 为 false / Control read; ok is false on STALL
  std::vector<uint8_t> ControlIn(const SetupPacket& setup, bool& ok)
  {
    device_.Setup(setup);
    return ControlInData(setup.wLength, ok);
  }

  /// 控制读的数据和状态阶段，SETUP 已发 / Data and status stages of a control read
  /// whose SETUP was sent
  std::vector<uint8_t> ControlInData(uint16_t length, bool& ok)
  {
    ok = false;
    std::vector<uint8_t> data;
    while (data.size() < length)
    {
      InResult in = In(0);
      if (in.handshake != Handshake::ACK)
      {
        return data;
      }
      data.insert(data.end(), in.data.begin(), in.data.end());
      if (in.data.size() < ep0_packet_)
      {
        break;
      }
    }
    ok = Out(0, {}) == Handshake::ACK;
    return data;
  }

  /// 控制写（含无数据阶段）/ Control write (with or without a data stage)
  bool ControlOut(const SetupPacket& setup, const std::vector<uint8_t>& data = {})
  {
    device_.Setup(setup);
    for (size_t off = 0; off < data.size(); off += ep0_packet_)
    {
      const size_t n = std::min<size_t>(ep0_packet_, data.size() - off);
      if (Out(0, std::vector<uint8_t>(data.begin() + off, data.begin() + off + n)) !=
          Handshake::ACK)
      {
        return false;
      }
    }
    InResult in = In(0);
    return in.handshake == Handshake::ACK && in.data.empty();
  }

  /// 总线复位、设备描述符、地址、配置描述符、SET_CONFIGURATION / Bus reset, device
  /// descriptor, address, configuration descriptor, SET_CONFIGURATION
  bool Enumerate(uint8_t address = 1, uint8_t configuration = 1,
                 Speed speed = Speed::FULL)
  {
    device_.BusReset(speed);
    ResetToggles();
    bool ok = false;
    device_desc_ = ControlIn(MakeSetup(0x80, 6, 0x0100, 0, 18), ok);
    if (!ok || device_desc_.size() != 18)
    {
      return Fail("GET_DESCRIPTOR(device)");
    }
    if (!ControlOut(MakeSetup(0x00, 5, address, 0, 0)))
    {
      return Fail("SET_ADDRESS");
    }
    // 恢复时间（USB 2.0 9.2.6.3 的 2 ms）：地址在状态阶段完成后由 owner 写入，可能晚于
    // 主机看到状态阶段的握手。
    // The recovery interval (2 ms in USB 2.0 9.2.6.3): the owner writes the address once
    // the status stage completes, which may be later than the host saw its handshake.
    const auto set_at = std::chrono::steady_clock::now();
    while (device_.Address() != address &&
           std::chrono::steady_clock::now() - set_at < RETRY_TIME)
    {
      std::this_thread::yield();
    }
    if (device_.Address() != address)
    {
      return Fail("address not applied");
    }
    auto head = ControlIn(MakeSetup(0x80, 6, 0x0200, 0, 9), ok);
    if (!ok || head.size() != 9)
    {
      return Fail("GET_DESCRIPTOR(configuration, 9)");
    }
    const uint16_t total = static_cast<uint16_t>(head[2] | (head[3] << 8));
    config_ = ControlIn(MakeSetup(0x80, 6, 0x0200, 0, total), ok);
    if (!ok || config_.size() != total)
    {
      return Fail("GET_DESCRIPTOR(configuration)");
    }
    if (configuration != 0U && !SetConfiguration(configuration))
    {
      return Fail("SET_CONFIGURATION");
    }
    last_failure = nullptr;
    return true;
  }

  /// SET_CONFIGURATION；成功后翻转位回到 DATA0 / SET_CONFIGURATION; the toggles return
  /// to DATA0 on success
  bool SetConfiguration(uint8_t value)
  {
    const bool ok = ControlOut(MakeSetup(0x00, 9, value, 0, 0));
    if (ok)
    {
      ResetToggles();
    }
    return ok;
  }

  /// SET_INTERFACE；成功后该接口端点的翻转位回到 DATA0 / SET_INTERFACE; the toggles of
  /// the interface's endpoints return to DATA0 on success
  bool SetInterface(uint8_t itf, uint8_t alt)
  {
    const bool ok = ControlOut(MakeSetup(0x01, 11, alt, itf, 0));
    if (ok)
    {
      for (uint8_t addr : InterfaceEndpoints(itf))
      {
        Toggle(addr) = 0;
      }
    }
    return ok;
  }

  /// CLEAR_FEATURE(ENDPOINT_HALT)；成功后翻转位回到 DATA0 / CLEAR_FEATURE
  /// (ENDPOINT_HALT); the toggle returns to DATA0 on success
  bool ClearHalt(uint8_t ep_addr)
  {
    const bool ok = ControlOut(MakeSetup(0x02, 1, 0, ep_addr, 0));
    if (ok)
    {
      Toggle(ep_addr) = 0;
    }
    return ok;
  }

  /// SET_FEATURE(ENDPOINT_HALT) / SET_FEATURE(ENDPOINT_HALT)
  bool SetHalt(uint8_t ep_addr) { return ControlOut(MakeSetup(0x02, 3, 0, ep_addr, 0)); }

  /// 一个 OUT 包，只试一次，带翻转位 / One OUT packet, one attempt, with the toggle
  Handshake TryOut(uint8_t number, const std::vector<uint8_t>& data)
  {
    const uint8_t pid = number == 0U ? 0U : Toggle(number);
    const Handshake hs = device_.OutToken(number, data, static_cast<Pid>(pid));
    if (hs == Handshake::ACK && number != 0U)
    {
      Toggle(number) ^= 1U;
    }
    return hs;
  }

  /// 一个 IN 包，只试一次；翻转位不符的包丢弃，算 NAK / One IN packet, one attempt; a
  /// packet with a mismatched toggle is dropped and counts as NAK
  InResult TryIn(uint8_t number)
  {
    InResult in = device_.InToken(number);
    if (in.handshake == Handshake::ACK && number != 0U)
    {
      const uint8_t addr = static_cast<uint8_t>(number | 0x80U);
      if (static_cast<uint8_t>(in.pid) != Toggle(addr))
      {
        ++duplicates;
        in.handshake = Handshake::NAK;
        in.data.clear();
      }
      else
      {
        Toggle(addr) ^= 1U;
      }
    }
    return in;
  }

  /// 一个 OUT 包，NAK 时重试 / One OUT packet, retried on NAK
  Handshake Out(uint8_t number, const std::vector<uint8_t>& data)
  {
    Handshake hs = Handshake::NAK;
    const auto start = std::chrono::steady_clock::now();
    for (unsigned i = 0;; ++i)
    {
      const uint8_t pid = number == 0U ? 0U : Toggle(number);
      hs = device_.OutToken(number, data, static_cast<Pid>(pid));
      if (hs == Handshake::ACK && number != 0U)
      {
        Toggle(number) ^= 1U;
      }
      if (hs != Handshake::NAK || !KeepRetrying(i, start))
      {
        break;
      }
    }
    last_handshake = hs;
    return hs;
  }

  /// 一个 IN 包，NAK 时重试；翻转位不符的包丢弃后再取 / One IN packet, retried on
  /// NAK; a packet with a mismatched toggle is dropped and the next one taken
  InResult In(uint8_t number)
  {
    InResult in;
    const auto start = std::chrono::steady_clock::now();
    for (unsigned i = 0;; ++i)
    {
      in = device_.InToken(number);
      if (in.handshake == Handshake::ACK && number != 0U)
      {
        const uint8_t addr = static_cast<uint8_t>(number | 0x80U);
        if (static_cast<uint8_t>(in.pid) != Toggle(addr))
        {
          ++duplicates;
          in.handshake = Handshake::NAK;
        }
        else
        {
          Toggle(addr) ^= 1U;
        }
      }
      if (in.handshake != Handshake::NAK || !KeepRetrying(i, start))
      {
        break;
      }
    }
    last_handshake = in.handshake;
    return in;
  }

  /**
   * @brief 批量写，按包长拆分 / Bulk write split by the packet size
   * @param zlp 整倍数长度之后补零长度包 / Close a packet-multiple length with a
   *        zero-length packet
   */
  bool BulkOut(uint8_t number, uint16_t packet, const std::vector<uint8_t>& data,
               bool zlp = true)
  {
    size_t off = 0;
    do
    {
      const size_t n = std::min<size_t>(packet, data.size() - off);
      if (Out(number, std::vector<uint8_t>(data.begin() + off, data.begin() + off + n)) !=
          Handshake::ACK)
      {
        return false;
      }
      off += n;
      if (n < packet)
      {
        return true;
      }
    } while (off < data.size());
    if (zlp && !data.empty())
    {
      return Out(number, {}) == Handshake::ACK;
    }
    return true;
  }

  /// 批量读：读到短包或 max 字节 / Bulk read up to a short packet or max bytes
  std::vector<uint8_t> BulkIn(uint8_t number, uint16_t packet, size_t max)
  {
    std::vector<uint8_t> data;
    while (data.size() < max)
    {
      InResult in = In(number);
      if (in.handshake != Handshake::ACK)
      {
        break;
      }
      data.insert(data.end(), in.data.begin(), in.data.end());
      if (in.data.size() < packet)
      {
        break;
      }
    }
    return data;
  }

  const std::vector<uint8_t>& ConfigDescriptor() const { return config_; }
  const std::vector<uint8_t>& DeviceDescriptorBytes() const { return device_desc_; }

  /// 主机一侧的翻转位 / The host's data toggle
  uint8_t& Toggle(uint8_t ep_addr)
  {
    return toggles_[(ep_addr & 0x80U) != 0U ? 1 : 0][ep_addr & 0x0FU];
  }

  void ResetToggles()
  {
    for (auto& dir : toggles_)
    {
      for (uint8_t& t : dir)
      {
        t = 0;
      }
    }
  }

  /// 配置描述符里某个接口的端点地址 / Endpoint addresses of one interface in the
  /// configuration descriptor
  std::vector<uint8_t> InterfaceEndpoints(uint8_t itf) const
  {
    std::vector<uint8_t> addrs;
    bool inside = false;
    for (size_t off = 0; off + 2U <= config_.size() && config_[off] != 0U;
         off += config_[off])
    {
      if (config_[off + 1U] == 4U && off + 2U < config_.size())
      {
        inside = config_[off + 2U] == itf;
      }
      else if (config_[off + 1U] == 5U && inside && off + 2U < config_.size())
      {
        addrs.push_back(config_[off + 2U]);
      }
    }
    return addrs;
  }

  static constexpr std::chrono::milliseconds RETRY_TIME{200};

 private:
  bool Fail(const char* step)
  {
    last_failure = step;
    return false;
  }

  bool KeepRetrying(unsigned attempt, std::chrono::steady_clock::time_point start)
  {
    if (auto_frames && (attempt + 1U) % NAKS_PER_FRAME == 0U && device_.Visible() &&
        !device_.Suspended())
    {
      device_.Sof();
    }
    if (attempt + 1U < RETRIES)
    {
      return true;
    }
    if (std::chrono::steady_clock::now() - start >= RETRY_TIME)
    {
      return false;
    }
    std::this_thread::yield();
    return true;
  }

  Controller& device_;
  uint16_t ep0_packet_;
  std::vector<uint8_t> device_desc_;
  std::vector<uint8_t> config_;
  uint8_t toggles_[2][16] = {};
};

}  // namespace LibXR::USB::Sim
