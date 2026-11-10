/**
 * @file usb_sim.hpp
 * @brief USB 仿真后端 / USB simulation backend.
 *
 * 只实现基类声明的后端契约（`Endpoint` 的后端钩子、`DeviceCore` 对后端的接口、
 * `EndpointPool` 的硬件守卫），其余全部是真实的端点核心、DeviceCore 和设备类。仿真是
 * 契约的参考实现：违反契约直接断言；与它不一致的真实后端是后端的 bug。
 * Implements only the backend contract declared by the base classes (the backend hooks
 * of `Endpoint`, the backend interface of `DeviceCore` and the hardware guard of
 * `EndpointPool`); everything above it is the real endpoint core, DeviceCore and device
 * classes. The simulation is the reference implementation of the contract: a violation
 * asserts, and a real backend that differs from it has a bug.
 *
 * 主机侧按包操作（SETUP、OUT、IN 令牌、SOF、总线复位、挂起、恢复、断开），每个动作是
 * 一次仿真中断。硬件守卫是一把锁：线程侧持有它相当于屏蔽中断，此时到来的中断挂起，
 * 守卫释放时再执行，和单核上屏蔽中断的效果一致。
 * The host side works per packet (SETUP, OUT, IN tokens, SOF, bus reset, suspend,
 * resume, disconnect), and every action is one simulated interrupt. The hardware guard
 * is a lock: holding it on the thread side is masking the interrupt, an interrupt raised
 * meanwhile stays pending and runs when the guard is released, as masking does on one
 * core.
 *
 * 时间只有帧：SOF 由测试推进，停止确认和停止超时都按帧计。
 * Time is frames only: the test advances SOF, and stop confirmation and the stop timeout
 * count frames.
 *
 * 帧中断和真实控制器一样只在后端打开它时才送到 DeviceCore：有待决停止时，或者后端按
 * `SofPolicy` 在自己的重估点（Start、OnStopsSettled、DeviceCore 的
 * `OnBusTimeNeedChanged()`，CH32 还有 OnControlReady）看到 `WantsBusTime()` 为真。这是
 * 按读代码得到的模型，只有这几个重估点：真实的 STM32 OTG 后端还会在总线复位的 TX FIFO
 * 清空完成时重估（PollTxFlush 里的 UpdateTick），仿真没有建模；真实板子上哪个重估点在
 * 枚举之后让帧中断保持打开没有确定，所以在没有 `OnBusTimeNeedChanged()` 的情况下，仿真
 * 的 STM32 OTG、ESP32 档位在枚举之后帧中断是关的，这一点和 STM32H723 上的测量不符。
 * As on a real controller, the frame interrupt reaches DeviceCore only while the backend
 * has it on: while a stop is pending, or after the backend saw `WantsBusTime()` true at
 * one of its own re-evaluation points (Start, OnStopsSettled, the DeviceCore hook
 * `OnBusTimeNeedChanged()`, and OnControlReady on CH32) as `SofPolicy` selects. This is a
 * model made from reading the code and has only these points: the real STM32 OTG backend
 * also re-evaluates when the TX FIFO flush after a bus reset completes (UpdateTick in
 * PollTxFlush), which is not modelled; which re-evaluation point keeps the frame
 * interrupt on after enumeration on a real board was not determined, so without
 * `OnBusTimeNeedChanged()` the simulated STM32 OTG and ESP32 profiles have the frame
 * interrupt off after enumeration, which differs from what was measured on an STM32H723.
 */

#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <mutex>
#include <new>
#include <thread>
#include <vector>

#include "dev_core.hpp"
#include "ep.hpp"
#include "ep_pool.hpp"
#include "test_assert.hpp"

#if defined(__SANITIZE_ADDRESS__)
/// 库按约定只分配、不释放（见 AGENTS.md）：DeviceCore 的控制接收缓冲、Callback、队列和
/// 读写端口都是这样，每个仿真设备和设备类都会"泄漏"几块，泄漏检查在这里没有意义。
/// ASan 的越界和释放后使用检查照常开启，UBSan 不受影响。
/// The library allocates and never frees by convention (see AGENTS.md): the control
/// receive buffer of DeviceCore, callbacks, queues and the read and write ports all do,
/// so every simulated device and device class "leaks" a few blocks and the leak check
/// means nothing here. ASan's overflow and use-after-free checks stay on, and UBSan is
/// unaffected.
extern "C" [[gnu::used]] inline const char* __asan_default_options()
{
  return "detect_leaks=0";
}
#endif

namespace LibXR::USB::Sim
{

/// 待决停止怎样确认 / How a pending stop is confirmed
enum class StopMode : uint8_t
{
  IMMEDIATE,  ///< StopHardware 直接返回 DONE / StopHardware returns DONE
  FRAMES,     ///< 返回 PENDING，若干帧后确认 / PENDING, confirmed after some frames
  NEVER       ///< 返回 PENDING，只有复位、断开或停止超时才确认 / PENDING until a reset,
              ///< a disconnect or the stop timeout
};

/// 后端何时重估帧中断开关 / When the backend re-evaluates its frame interrupt enable
enum class SofPolicy : uint8_t
{
  /// STM32 OTG、ESP32 DWC2、FSDEV：Start、OnStopsSettled 和 OnBusTimeNeedChanged
  /// （直接调用 OnStopsSettled）里重估，开关随 `WantsBusTime()` 开或关 / STM32 OTG,
  /// ESP32 DWC2, FSDEV: re-evaluated in Start, OnStopsSettled and OnBusTimeNeedChanged
  /// (which calls OnStopsSettled), on or off with `WantsBusTime()`
  AT_STOPS,
  /// CH32 OTG：OnControlReady 里 `WantsBusTime()` 为真就打开，OnBusTimeNeedChanged 里
  /// 随 `WantsBusTime()` 开或关 / CH32 OTG: OnControlReady turns it on when
  /// `WantsBusTime()` is true, OnBusTimeNeedChanged sets it on or off with
  /// `WantsBusTime()`
  AT_CONTROL_READY,
  /// 理想控制器：每帧都送（隔离测试用，不对应任何真实后端）/ Ideal controller, every
  /// frame is delivered (to isolate a class from the policy; no real backend)
  ALWAYS
};

/// 一个数据端点的硬件规格 / Hardware specification of one data endpoint
struct EndpointSpec
{
  uint8_t number = 1;
  Endpoint::Direction direction = Endpoint::Direction::BOTH;
  size_t buffer = 512;     ///< 槽内存字节（两个槽时平分）/ Slot memory bytes
  uint8_t slot_count = 0;  ///< 0：EP0 一个，其余两个 / 0: one for EP0, two otherwise
  /// 该端点的 FIFO/PMA 包长上限，0 表示只受控制器上限约束 / This endpoint's FIFO or PMA
  /// packet limit; 0 leaves only the controller limit
  size_t packet_limit = 0;
};

/// 控制器的硬件限制 / Hardware limits of the controller
struct Profile
{
  std::vector<EndpointSpec> endpoints = {{1, Endpoint::Direction::BOTH, 512, 0},
                                         {2, Endpoint::Direction::BOTH, 512, 0},
                                         {3, Endpoint::Direction::BOTH, 512, 0}};
  size_t ep0_buffer = 64;
  size_t max_packet_limit = UINT16_MAX;  ///< FIFO/PMA 上限 / FIFO or PMA limit
  size_t control_packet_limit = 64;      ///< 控制端点包长上限 / Control packet limit
  bool isochronous = true;               ///< 是否支持等时端点 / Isochronous supported
  /// 同号 IN、OUT 共用端点类型（FSDEV 的 EP_TYPE）：另一方向已配置成别的类型时配置失败
  /// IN and OUT of one number share the endpoint type (EP_TYPE on FSDEV): configuring a
  /// different type than the other, configured direction fails
  bool shared_type_per_number = false;
  /// 占用 TX FIFO 的 IN 端点数上限（含 EP0），0 不限；占用一直保持到总线复位或断开
  /// （DWC2 的 TX FIFO 分配）。超出时配置返回 NO_BUFF。
  /// Limit on IN endpoints holding a TX FIFO (EP0 included), 0 for none; a FIFO stays
  /// held until a bus reset or disconnect (DWC2 TX FIFO allocation). Beyond it a
  /// configuration returns NO_BUFF.
  uint8_t in_endpoint_limit = 0;
  StopMode stop_mode = StopMode::IMMEDIATE;
  uint32_t stop_frames = 2;  ///< FRAMES 模式下的帧数 / Frames for StopMode::FRAMES
  SofPolicy sof_policy = SofPolicy::AT_STOPS;
  /// 调度注入：每次 StartHardware（守卫内）调用，可在此发起仿真中断，中断挂起到守卫
  /// 释放时执行 / Schedule injection: called on every StartHardware (inside the guard);
  /// an interrupt raised here is pended until the guard is released
  std::function<void(uint8_t number, bool in)> on_start;
};

/// 主机看到的握手 / Handshake seen by the host
enum class Handshake : uint8_t
{
  ACK,
  NAK,
  STALL
};

struct InResult
{
  Handshake handshake = Handshake::NAK;
  std::vector<uint8_t> data;
};

// ---------------------------------------------------------------------------
// Hardware guard
// ---------------------------------------------------------------------------

/// 硬件守卫：可重入的锁。持有者所在线程发起的中断挂起到最外层释放时执行。
/// Hardware guard: a re-entrant lock. An interrupt raised by the thread that holds it is
/// pended until the outermost release.
class Guard
{
 public:
  void Enter()
  {
    if (owner_.load(std::memory_order_acquire) == std::this_thread::get_id())
    {
      ++depth_;
      return;
    }
    mutex_.lock();
    owner_.store(std::this_thread::get_id(), std::memory_order_release);
    depth_ = 1;
  }

  /// 最外层释放时返回挂起的中断，由调用方执行 / The outermost release hands the pending
  /// interrupts back for the caller to run
  std::vector<std::function<void()>> Leave()
  {
    TEST_ASSERT(HeldByMe() && depth_ > 0);
    if (--depth_ != 0)
    {
      return {};
    }
    std::vector<std::function<void()>> pending;
    pending.swap(pending_);
    owner_.store(std::thread::id(), std::memory_order_release);
    mutex_.unlock();
    return pending;
  }

  bool HeldByMe() const
  {
    return owner_.load(std::memory_order_acquire) == std::this_thread::get_id();
  }

  void Pend(std::function<void()> irq) { pending_.push_back(std::move(irq)); }

 private:
  std::mutex mutex_;
  std::atomic<std::thread::id> owner_{};
  int depth_ = 0;
  std::vector<std::function<void()>> pending_;
};

template <typename Caps>
class Device;

// ---------------------------------------------------------------------------
// Endpoint
// ---------------------------------------------------------------------------

/// 仿真端点：同一时刻最多一个在途请求，主机按包推进 / Simulated endpoint: one request
/// in flight at most, advanced by the host packet by packet
template <typename Caps>
class SimEndpoint : public Endpoint
{
 public:
  SimEndpoint(Device<Caps>& device, EPNumber number, Direction direction, RawData buffer,
              uint8_t slot_count, size_t packet_limit = 0)
      : Endpoint(number, direction, buffer, slot_count),
        device_(device),
        packet_limit_(packet_limit)
  {
  }

  /// 后端钩子被调用的次数，测试用 / Backend hook counts for tests
  unsigned starts = 0, stops = 0, closes = 0;

  bool Active() const { return active_; }
  bool StopPending() const { return stop_pending_; }

 protected:
  ErrorCode ConfigureHardware(const Config& cfg) override
  {
    TEST_ASSERT(device_.guard_.HeldByMe());
    TEST_ASSERT(!active_ && !stop_pending_);
    if (cfg.type == Type::ISOCHRONOUS && !device_.profile_.isochronous)
    {
      return ErrorCode::NOT_SUPPORT;
    }
    const uint8_t number = EPNumberToInt8(GetNumber());
    if (device_.profile_.shared_type_per_number && number != 0U)
    {
      const SimEndpoint* other =
          IsIn() ? device_.ep_out_[number] : device_.ep_in_[number];
      if (other != nullptr &&
          (other->GetState() == State::IDLE || other->GetState() == State::STALLED) &&
          other->GetConfig().type != cfg.type)
      {
        return ErrorCode::FAILED;
      }
    }
    if (IsIn() && device_.profile_.in_endpoint_limit != 0U &&
        (device_.tx_fifos_ & (1U << number)) == 0U)
    {
      if (static_cast<unsigned>(__builtin_popcount(device_.tx_fifos_)) >=
          device_.profile_.in_endpoint_limit)
      {
        return ErrorCode::NO_BUFF;
      }
      device_.tx_fifos_ |= 1U << number;
    }
    stalled_ = false;
    progress_ = 0;
    stopped_actual_ = 0;
    return ErrorCode::OK;
  }

  void CloseHardware(bool in_isr) override
  {
    UNUSED(in_isr);
    TEST_ASSERT(device_.guard_.HeldByMe());
    // 契约：只在没有在途请求时关闭 / Contract: only without an in-flight request.
    TEST_ASSERT(!active_ && !stop_pending_);
    stalled_ = false;
    ++closes;
  }

  ErrorCode StartHardware(const HwRequest& request, bool in_isr) override
  {
    UNUSED(in_isr);
    TEST_ASSERT(device_.guard_.HeldByMe());
    // 契约：同一时刻最多一个请求 / Contract: one request at a time.
    TEST_ASSERT(!active_ && !stop_pending_);
    TEST_ASSERT(request.length <= request.slot.size_ || request.length == 0U);
    if (!IsIn())
    {
      // 契约：OUT 请求长度是包长的整数倍 / Contract: an OUT request is a packet multiple.
      TEST_ASSERT(MaxPacketSize() != 0U && request.length % MaxPacketSize() == 0U);
    }
    tag_ = request.tag;
    data_ = static_cast<uint8_t*>(request.slot.addr_);
    length_ = request.length;
    progress_ = 0;
    active_ = true;
    ++starts;
    if (device_.profile_.on_start)
    {
      device_.profile_.on_start(EPNumberToInt8(GetNumber()), IsIn());
    }
    return ErrorCode::OK;
  }

  StopResult StopHardware(bool in_isr) override
  {
    UNUSED(in_isr);
    TEST_ASSERT(device_.guard_.HeldByMe());
    TEST_ASSERT(!stop_pending_);
    ++stops;
    // EP0 的传输被下一个 SETUP 取消，停止总是立即完成 / An EP0 transfer is cancelled by
    // the next SETUP, so its stop is always immediate.
    if (!active_ || device_.profile_.stop_mode == StopMode::IMMEDIATE ||
        GetNumber() == EPNumber::EP0)
    {
      active_ = false;
      stopped_actual_ = progress_;
      return StopResult::DONE;
    }
    // 停止过程中端点不再接受事务 / No transaction is taken while the stop runs.
    stop_pending_ = true;
    frames_left_ = device_.profile_.stop_frames;
    // 待决停止需要帧来计超时：真实后端在启动停止时打开帧中断（STM32 OTG 的 UpdateTick）。
    // A pending stop needs frames for its timeout: real backends turn the frame interrupt
    // on when a stop starts (UpdateTick on STM32 OTG).
    device_.sof_on_ = true;
    return StopResult::PENDING;
  }

  size_t ReapProgress(const HwRequest& request, bool& retired) override
  {
    UNUSED(request);
    TEST_ASSERT(device_.guard_.HeldByMe());
    TEST_ASSERT(!active_ && !stop_pending_);
    retired = false;
    return stopped_actual_;
  }

  ErrorCode StallHardware(bool in_isr) override
  {
    UNUSED(in_isr);
    TEST_ASSERT(device_.guard_.HeldByMe());
    stalled_ = true;
    return ErrorCode::OK;
  }

  ErrorCode ClearStallHardware(bool in_isr) override
  {
    UNUSED(in_isr);
    TEST_ASSERT(device_.guard_.HeldByMe());
    stalled_ = false;
    return ErrorCode::OK;
  }

  size_t MaxPacketSizeLimit(Type type) const override
  {
    size_t limit = device_.profile_.max_packet_limit;
    if (packet_limit_ != 0U) limit = std::min(limit, packet_limit_);
    if (type == Type::CONTROL)
      limit = std::min(limit, device_.profile_.control_packet_limit);
    return limit;
  }

 private:
  friend class Device<Caps>;

  bool IsIn() const { return GetConfig().direction == Direction::IN; }

  // ---- 中断侧（仿真中断里调用）/ Interrupt side (inside a simulated interrupt) ----

  Handshake HostOut(const uint8_t* data, size_t size)
  {
    if (stalled_) return Handshake::STALL;
    if (!active_ || stop_pending_) return Handshake::NAK;
    const size_t packet = MaxPacketSize();
    TEST_ASSERT(size <= packet);  // 主机不会发超过包长的包 / the host never does
    const size_t room = length_ - progress_;
    const size_t take = size < room ? size : room;
    if (take != 0U) std::memcpy(data_ + progress_, data, take);
    progress_ += take;
    if (size > room)
    {
      // 主机发来的比请求剩下的多 / The host sent more than the request has room for.
      active_ = false;
      OnHardwareComplete(true, tag_, progress_, ErrorCode::OUT_OF_RANGE);
      return Handshake::ACK;
    }
    if (size < packet || progress_ >= length_)
    {
      active_ = false;
      OnHardwareComplete(true, tag_, progress_);
    }
    return Handshake::ACK;
  }

  InResult HostIn()
  {
    InResult result;
    if (stalled_)
    {
      result.handshake = Handshake::STALL;
      return result;
    }
    if (!active_ || stop_pending_) return result;
    const size_t packet = MaxPacketSize();
    const size_t rest = length_ - progress_;
    const size_t give = rest < packet ? rest : packet;
    result.data.assign(data_ + progress_, data_ + progress_ + give);
    progress_ += give;
    result.handshake = Handshake::ACK;
    // 最后一个包（含零长包）被主机取走即完成 / Done once the host took the last packet.
    if (progress_ >= length_)
    {
      active_ = false;
      OnHardwareComplete(true, tag_, length_);
    }
    return result;
  }

  /// SETUP 清除 EP0 的 STALL / A SETUP clears the EP0 halt
  void ClearStallBySetup() { stalled_ = false; }

  void OnFrame()
  {
    if (stop_pending_ && device_.profile_.stop_mode == StopMode::FRAMES &&
        frames_left_ != 0U && --frames_left_ == 0U)
    {
      ConfirmStop();
    }
  }

  /// 确认待决停止（帧到期、总线复位、断开、停止超时）/ Confirm a pending stop (frames
  /// elapsed, bus reset, disconnect, stop timeout)
  void ConfirmStop()
  {
    if (!stop_pending_) return;
    stop_pending_ = false;
    active_ = false;
    stopped_actual_ = progress_;
    OnHardwareStopped(true);
  }

  Device<Caps>& device_;
  size_t packet_limit_ = 0;
  bool active_ = false;
  bool stop_pending_ = false;
  bool stalled_ = false;
  uint32_t tag_ = 0;
  uint8_t* data_ = nullptr;
  size_t length_ = 0;
  size_t progress_ = 0;
  size_t stopped_actual_ = 0;
  uint32_t frames_left_ = 0;
};

// ---------------------------------------------------------------------------
// Device
// ---------------------------------------------------------------------------

/// 仿真控制器加设备核心 / Simulated controller with the device core
template <typename Caps = FullSpeedCapabilities>
class Device : public EndpointPool, public DeviceCore<Caps>
{
 public:
  using Core = DeviceCore<Caps>;
  using Ep = SimEndpoint<Caps>;

  Device(const Profile& profile, DeviceDescriptor::PacketSize0 packet_size, uint16_t vid,
         uint16_t pid, uint16_t bcd,
         const std::initializer_list<const DescriptorStrings::LanguagePack*>& languages,
         const std::initializer_list<const std::initializer_list<ConfigDescriptorItem*>>&
             configs,
         Speed speed = Speed::FULL)
      : EndpointPool(2U + CountEndpoints(profile)),
        Core(*this, USBSpec::USB_2_1, speed, packet_size, vid, pid, bcd, languages,
             configs),
        profile_(profile)
  {
    SetHardwareGuard(
        this,
        [](void* self) -> uintptr_t
        {
          static_cast<Device*>(self)->guard_.Enter();
          return 0U;
        },
        [](void* self, uintptr_t) { static_cast<Device*>(self)->LeaveGuard(); });

    auto* ep0_out = MakeEndpoint(0, Endpoint::Direction::OUT, profile_.ep0_buffer, 0);
    auto* ep0_in = MakeEndpoint(0, Endpoint::Direction::IN, profile_.ep0_buffer, 0);
    SetEndpoint0(ep0_in, ep0_out);
    for (const auto& spec : profile_.endpoints)
    {
      if (spec.direction == Endpoint::Direction::BOTH)
      {
        TEST_ASSERT(Put(MakeEndpoint(spec.number, Endpoint::Direction::OUT, spec.buffer,
                                     spec.slot_count, spec.packet_limit)) ==
                    ErrorCode::OK);
        TEST_ASSERT(Put(MakeEndpoint(spec.number, Endpoint::Direction::IN, spec.buffer,
                                     spec.slot_count, spec.packet_limit)) ==
                    ErrorCode::OK);
      }
      else
      {
        TEST_ASSERT(Put(MakeEndpoint(spec.number, spec.direction, spec.buffer,
                                     spec.slot_count, spec.packet_limit)) ==
                    ErrorCode::OK);
      }
    }
  }

  ~Device() override = default;

  // ---- DeviceCore 对后端的接口 / Backend interface of DeviceCore ----

  void Start(bool in_isr) override
  {
    UNUSED(in_isr);
    EndpointPool::HardwareScope hardware(this);
    pulled_up_ = true;
    // STM32 OTG（Connect）、ESP32、FSDEV 在 Start 里按 WantsBusTime() 设置帧中断。
    // STM32 OTG (Connect), ESP32 and FSDEV set the frame interrupt from WantsBusTime()
    // in Start.
    if (profile_.sof_policy == SofPolicy::AT_STOPS) sof_on_ = this->WantsBusTime();
  }

  void Stop(bool in_isr) override
  {
    UNUSED(in_isr);
    EndpointPool::HardwareScope hardware(this);
    pulled_up_ = false;
    sof_on_ = false;
  }

  /// 设备在线时主机能看见它 / The host sees the device while it is attached and pulled up
  bool Visible() const { return pulled_up_ && attached_; }
  uint8_t Address() const { return address_.load(); }
  uint32_t Frame() const { return frame_; }
  unsigned StopTimeouts() const { return stop_timeouts_; }
  Ep* In(uint8_t number) { return ep_in_[number]; }
  Ep* Out(uint8_t number) { return ep_out_[number]; }

  // ---- 主机动作：每个是一次仿真中断 / Host actions: one simulated interrupt each ----

  void BusReset()
  {
    Irq(
        [this]
        {
          address_ = 0;
          discard_ = true;
          tx_fifos_ = 0;
          // 契约（ep.hpp / dev_core.hpp）：复位时确认所有待决停止 / Contract: a bus reset
          // confirms every pending stop.
          ConfirmAllStops();
          this->OnBusReset(true);
        });
  }

  void Disconnect()
  {
    Irq(
        [this]
        {
          attached_ = false;
          tx_fifos_ = 0;
          // 契约（ep.hpp / dev_core.hpp）：断开时确认所有待决停止 / Contract: a
          // disconnect confirms every pending stop.
          ConfirmAllStops();
          this->OnDisconnect(true);
        });
  }

  void Attach()
  {
    Irq([this] { attached_ = true; });
  }

  void Suspend()
  {
    Irq(
        [this]
        {
          suspended_ = true;
          this->OnSuspend(true);
        });
  }

  void Resume()
  {
    Irq(
        [this]
        {
          suspended_ = false;
          this->OnResume(true);
        });
  }

  /// 一帧：先推进各端点的停止，再交给核心计帧 / One frame: advance the endpoint stops,
  /// then let the core count it
  void Sof()
  {
    // 挂起期间主机不发 SOF / The host sends no SOF while suspended.
    TEST_ASSERT(!suspended_ && Visible());
    Irq(
        [this]
        {
          ++frame_;
          ForEachEndpoint([](Ep& ep) { ep.OnFrame(); });
          // 和真实控制器一样，帧中断只在后端打开它（见 SofPolicy）或有待决停止时才送到
          // DeviceCore。
          // As on a real controller, the frame interrupt reaches DeviceCore only while
          // the backend has it on (see SofPolicy) or a stop is pending.
          bool stopping = false;
          ForEachEndpoint([&stopping](Ep& ep)
                          { stopping = stopping || ep.StopPending(); });
          if (sof_on_ || stopping || profile_.sof_policy == SofPolicy::ALWAYS)
          {
            ++sof_delivered_;
            this->OnSof(true, static_cast<uint16_t>(frame_ & 0x7FFU));
          }
        });
  }

  void Setup(const SetupPacket& packet)
  {
    TEST_ASSERT(Visible());
    Irq(
        [this, packet]
        {
          ep0_in()->ClearStallBySetup();
          ep0_out()->ClearStallBySetup();
          SetupPacket copy = packet;
          this->OnSetupPacket(true, &copy);
        });
  }

  Handshake OutPacket(uint8_t number, const std::vector<uint8_t>& data)
  {
    // 主机令牌来自总线，不会在设备自己屏蔽中断时同步发起 / Host tokens come from the bus,
    // never synchronously from inside the device's own masked region.
    TEST_ASSERT(Visible() && !guard_.HeldByMe());
    Handshake result = Handshake::NAK;
    Irq(
        [this, number, &data, &result]
        {
          Ep* ep = number == 0U ? ep0_out() : ep_out_[number];
          TEST_ASSERT(ep != nullptr);
          if (number != 0U && discard_) return;
          result = ep->HostOut(data.data(), data.size());
        });
    return result;
  }

  InResult InToken(uint8_t number)
  {
    TEST_ASSERT(Visible() && !guard_.HeldByMe());
    InResult result;
    Irq(
        [this, number, &result]
        {
          Ep* ep = number == 0U ? ep0_in() : ep_in_[number];
          TEST_ASSERT(ep != nullptr);
          if (number != 0U && discard_) return;
          result = ep->HostIn();
        });
    return result;
  }

  /// 调度注入用：主机取走一个 IN 包，结果交给 sink。守卫被持有时挂起执行。
  /// For schedule injection: the host takes one IN packet and hands the result to sink;
  /// pended while the guard is held.
  void InjectIn(uint8_t number, std::function<void(InResult)> sink)
  {
    Irq(
        [this, number, sink]
        {
          Ep* ep = number == 0U ? ep0_in() : ep_in_[number];
          TEST_ASSERT(ep != nullptr);
          sink(ep->HostIn());
        });
  }

  /// 运行一次仿真中断。守卫被本线程持有时挂起到释放时执行。
  /// Run one simulated interrupt. While this thread holds the guard it is pended until
  /// the release.
  void Irq(std::function<void()> body)
  {
    if (guard_.HeldByMe())
    {
      guard_.Pend(std::move(body));
      return;
    }
    guard_.Enter();
    {
      EndpointPool::InterruptScope batch(*this);
      body();
    }
    LeaveGuard();
  }

 protected:
  ErrorCode SetAddress(uint8_t address, ControlContext context) override
  {
    // 地址在状态阶段完成后生效 / The address takes effect after the status stage.
    if (context == ControlContext::STATUS_IN_COMPLETE)
    {
      EndpointPool::HardwareScope hardware(this);
      address_ = address;
    }
    return ErrorCode::OK;
  }

  void OnControllerReset() override { discard_ = false; }

  /// 所有待决停止确认后重估帧中断（STM32 OTG 的 UpdateTick，ESP32，FSDEV）；设备已 Stop
  /// 时不重开（FSDEV）。
  /// Re-evaluate the frame interrupt once every stop is confirmed (UpdateTick on STM32
  /// OTG, ESP32, FSDEV); not turned on again after Stop (FSDEV).
  void OnStopsSettled() override
  {
    if (profile_.sof_policy != SofPolicy::AT_STOPS || !pulled_up_) return;
    EndpointPool::HardwareScope hardware(this);
    sof_on_ = this->WantsBusTime();
  }

  /// CH32 在控制传输就绪时（每次控制传输结束，含
  /// SET_CONFIGURATION）打开帧中断，不再关闭。 CH32 turns the frame interrupt on when the
  /// control endpoint is ready (after every control transfer, SET_CONFIGURATION included)
  /// and never turns it off.
  void OnControlReady() override
  {
    if (profile_.sof_policy != SofPolicy::AT_CONTROL_READY) return;
    EndpointPool::HardwareScope hardware(this);
    if (this->WantsBusTime()) sof_on_ = true;
  }

  /// WantsBusTime() 变化：和真实后端一样重估帧中断。STM32 OTG、ESP32、FSDEV 直接调用
  /// OnStopsSettled；CH32 按需求开或关。
  /// WantsBusTime() changed: re-evaluate the frame interrupt as the real backends do.
  /// STM32 OTG, ESP32 and FSDEV call OnStopsSettled; CH32 sets it on or off with the
  /// need.
  void OnBusTimeNeedChanged() override
  {
    if (profile_.sof_policy == SofPolicy::AT_STOPS)
    {
      OnStopsSettled();
    }
    else if (profile_.sof_policy == SofPolicy::AT_CONTROL_READY)
    {
      EndpointPool::HardwareScope hardware(this);
      sof_on_ = this->WantsBusTime();
    }
  }

 public:
  /// 后端的帧中断是否打开、送到 DeviceCore 的 SOF 数 / Whether the backend has its frame
  /// interrupt on, and the SOFs delivered to DeviceCore
  bool FrameInterruptOn() const { return sof_on_; }
  unsigned SofDelivered() const { return sof_delivered_; }

 protected:
  /// 停止超时：复位控制器、确认所有待决停止，主机看到设备断开后重新枚举。
  /// Stop timeout: reset the controller and confirm every pending stop; the host sees
  /// the device disconnect and enumerates again.
  void OnStopTimeout(bool in_isr) override
  {
    ++stop_timeouts_;
    {
      EndpointPool::HardwareScope hardware(this);
      attached_ = false;
      ConfirmAllStops();
    }
    this->OnDisconnect(in_isr);
  }

 private:
  friend class SimEndpoint<Caps>;

  static size_t CountEndpoints(const Profile& profile)
  {
    size_t count = 0;
    for (const auto& spec : profile.endpoints)
    {
      count += spec.direction == Endpoint::Direction::BOTH ? 2U : 1U;
    }
    return count;
  }

  Ep* MakeEndpoint(uint8_t number, Endpoint::Direction direction, size_t size,
                   uint8_t slot_count, size_t packet_limit = 0)
  {
    auto* memory = static_cast<uint8_t*>(::operator new[](size, std::align_val_t(64)));
    std::memset(memory, 0, size);
    storage_.emplace_back(memory);
    endpoints_.emplace_back(new Ep(*this, static_cast<Endpoint::EPNumber>(number),
                                   direction, RawData{memory, size}, slot_count,
                                   packet_limit));
    Ep* ep = endpoints_.back().get();
    if (number != 0U)
    {
      (direction == Endpoint::Direction::IN ? ep_in_ : ep_out_)[number] = ep;
    }
    return ep;
  }

  Ep* ep0_in() { return static_cast<Ep*>(this->GetEndpoint0In()); }
  Ep* ep0_out() { return static_cast<Ep*>(this->GetEndpoint0Out()); }

  template <typename F>
  void ForEachEndpoint(F f)
  {
    for (auto& ep : endpoints_) f(*ep);
  }

  void ConfirmAllStops()
  {
    ForEachEndpoint([](Ep& ep) { ep.ConfirmStop(); });
  }

  void LeaveGuard()
  {
    for (auto& irq : guard_.Leave())
    {
      Irq(std::move(irq));
    }
  }

  struct AlignedDelete
  {
    void operator()(uint8_t* p) const { ::operator delete[](p, std::align_val_t(64)); }
  };

  Profile profile_;
  Guard guard_;
  std::vector<std::unique_ptr<uint8_t, AlignedDelete>> storage_;
  std::vector<std::unique_ptr<Ep>> endpoints_;
  Ep* ep_in_[16] = {};
  Ep* ep_out_[16] = {};
  bool pulled_up_ = false;
  bool attached_ = true;
  bool suspended_ = false;
  bool discard_ = false;
  std::atomic<uint8_t> address_{0};
  uint32_t frame_ = 0;
  unsigned stop_timeouts_ = 0;
  uint32_t tx_fifos_ =
      0;                 ///< 占用 TX FIFO 的 IN 端点号位图 / IN numbers holding a TX FIFO
  bool sof_on_ = false;  ///< 后端帧中断开关 / Backend frame interrupt on
  unsigned sof_delivered_ = 0;  ///< 送到 DeviceCore 的 SOF / SOFs given to DeviceCore
};

// ---------------------------------------------------------------------------
// Host helpers
// ---------------------------------------------------------------------------

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

/// 主机的事务层：控制传输与按包长拆分的批量传输；设备 NAK 时重试有限次。
/// The host transaction layer: control transfers and bulk transfers split by the packet
/// size; a NAK is retried a bounded number of times.
template <typename Caps>
class Host
{
 public:
  static constexpr unsigned RETRIES = 16;

  explicit Host(Device<Caps>& device, uint16_t ep0_packet = 64)
      : device_(device), ep0_packet_(ep0_packet)
  {
  }

  /// 控制读：返回数据阶段的字节；STALL 时 ok 为 false / Control read; ok is false on
  /// STALL
  std::vector<uint8_t> ControlIn(const SetupPacket& setup, bool& ok)
  {
    ok = false;
    device_.Setup(setup);
    std::vector<uint8_t> data;
    while (data.size() < setup.wLength)
    {
      InResult in = Retry([&] { return device_.InToken(0); });
      if (in.handshake != Handshake::ACK) return data;
      data.insert(data.end(), in.data.begin(), in.data.end());
      if (in.data.size() < ep0_packet_) break;
    }
    // 状态阶段：OUT 零长包 / Status stage: a zero-length OUT.
    ok = RetryOut(0, {}) == Handshake::ACK;
    return data;
  }

  /// 控制写（含无数据阶段）/ Control write (with or without a data stage)
  bool ControlOut(const SetupPacket& setup, const std::vector<uint8_t>& data = {})
  {
    device_.Setup(setup);
    for (size_t off = 0; off < data.size(); off += ep0_packet_)
    {
      const size_t n = std::min<size_t>(ep0_packet_, data.size() - off);
      if (RetryOut(0, std::vector<uint8_t>(data.begin() + off, data.begin() + off + n)) !=
          Handshake::ACK)
        return false;
    }
    // 状态阶段：IN 零长包 / Status stage: a zero-length IN.
    InResult in = Retry([&] { return device_.InToken(0); });
    return in.handshake == Handshake::ACK && in.data.empty();
  }

  /// 标准枚举：设备描述符、地址、配置描述符、SET_CONFIGURATION / Standard enumeration
  bool Enumerate(uint8_t address = 1, uint8_t configuration = 1)
  {
    bool ok = false;
    auto device_desc = ControlIn(MakeSetup(0x80, 6, 0x0100, 0, 18), ok);
    if (!ok || device_desc.size() != 18) return Fail("GET_DESCRIPTOR(device)");
    if (!ControlOut(MakeSetup(0x00, 5, address, 0, 0))) return Fail("SET_ADDRESS");
    // 地址在状态阶段的完成回调里生效，回调由谁抢到 owner 谁执行（可能是另一个线程，例如
    // DAP 的 Step 线程）；真实主机在 SET_ADDRESS 之后也要等恢复时间。所以这里等它生效。
    // The address takes effect in the status-stage completion, which runs in whichever
    // caller wins the owner (possibly another thread, such as the DAP Step thread); a
    // real host also waits a recovery time after SET_ADDRESS. So wait for it here.
    for (const auto start = std::chrono::steady_clock::now();
         device_.Address() != address; std::this_thread::yield())
    {
      if (std::chrono::steady_clock::now() - start >= RETRY_TIME)
        return Fail("address not applied");
    }
    auto head = ControlIn(MakeSetup(0x80, 6, 0x0200, 0, 9), ok);
    if (!ok || head.size() != 9) return Fail("GET_DESCRIPTOR(configuration, 9)");
    const uint16_t total = static_cast<uint16_t>(head[2] | (head[3] << 8));
    config_ = ControlIn(MakeSetup(0x80, 6, 0x0200, 0, total), ok);
    if (!ok || config_.size() != total) return Fail("GET_DESCRIPTOR(configuration)");
    if (!ControlOut(MakeSetup(0x00, 9, configuration, 0, 0)))
      return Fail("SET_CONFIGURATION");
    last_failure = nullptr;
    return true;
  }

  /// 最近一次失败的步骤，成功时为空 / The step that failed last, null on success
  const char* last_failure = nullptr;

  const std::vector<uint8_t>& ConfigDescriptor() const { return config_; }

  /// 批量写：按包长拆分；exact 为 true 时整倍数长度不补零长包（OpenOCD 的写法）。
  /// Bulk write split by the packet size; with exact the transfer does not end a
  /// packet-multiple length with a zero-length packet (as OpenOCD writes).
  bool BulkOut(uint8_t number, uint16_t packet, const std::vector<uint8_t>& data,
               bool exact = false)
  {
    size_t off = 0;
    do
    {
      const size_t n = std::min<size_t>(packet, data.size() - off);
      if (RetryOut(number,
                   std::vector<uint8_t>(data.begin() + off, data.begin() + off + n)) !=
          Handshake::ACK)
        return false;
      off += n;
      if (n < packet) return true;
    } while (off < data.size());
    if (!exact && !data.empty())
    {
      return RetryOut(number, {}) == Handshake::ACK;
    }
    return true;
  }

  /// 批量读：读到短包或 max 字节 / Bulk read up to a short packet or max bytes
  std::vector<uint8_t> BulkIn(uint8_t number, uint16_t packet, size_t max)
  {
    std::vector<uint8_t> data;
    while (data.size() < max)
    {
      InResult in = Retry([&] { return device_.InToken(number); });
      if (in.handshake != Handshake::ACK) break;
      data.insert(data.end(), in.data.begin(), in.data.end());
      if (in.data.size() < packet) break;
    }
    return data;
  }

 private:
  bool Fail(const char* step)
  {
    last_failure = step;
    return false;
  }

  template <typename F>
  InResult Retry(F token)
  {
    InResult in;
    const auto start = std::chrono::steady_clock::now();
    for (unsigned i = 0;; ++i)
    {
      in = token();
      if (in.handshake != Handshake::NAK || !KeepRetrying(i, start)) break;
    }
    return in;
  }

  Handshake RetryOut(uint8_t number, const std::vector<uint8_t>& data)
  {
    Handshake hs = Handshake::NAK;
    const auto start = std::chrono::steady_clock::now();
    for (unsigned i = 0;; ++i)
    {
      hs = device_.OutPacket(number, data);
      if (hs != Handshake::NAK || !KeepRetrying(i, start)) break;
    }
    return hs;
  }

  /// NAK 重试：至少 RETRIES 次，并持续到 RETRY_TIME。设备的 owner 可能正被别的线程
  /// 占用（例如 DAP 的 Step 线程），处理要等它让出；真实主机对 NAK 也会持续重试。
  /// NAK retries: at least RETRIES attempts and until RETRY_TIME has passed. The
  /// device's owner may be held by another thread (the DAP Step thread, say) and the
  /// work waits for it; real hosts keep retrying NAKs too.
  static bool KeepRetrying(unsigned attempt, std::chrono::steady_clock::time_point start)
  {
    if (attempt + 1 < RETRIES) return true;
    if (std::chrono::steady_clock::now() - start >= RETRY_TIME) return false;
    std::this_thread::yield();
    return true;
  }

  static constexpr std::chrono::milliseconds RETRY_TIME{200};

  Device<Caps>& device_;
  uint16_t ep0_packet_;
  std::vector<uint8_t> config_;
};

}  // namespace LibXR::USB::Sim
