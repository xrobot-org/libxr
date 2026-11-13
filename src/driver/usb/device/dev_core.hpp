#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <initializer_list>

#include "device_capabilities.hpp"
#include "device_features.hpp"
#include "timebase.hpp"

namespace LibXR::USB
{
/**
 * 事件驱动的 USB 设备协议引擎。能力是显式的编译期策略；不支持的功能状态与
 * 分发不会被实例化。类的组合仍是普通的运行期组合，而不是模板树。
 * Event-driven USB device protocol engine. Capabilities are an explicit compile-
 * time policy; unsupported feature state and dispatch are not instantiated.
 * Class composition remains ordinary runtime composition, not a template tree.
 */
template <typename Capabilities = FullSpeedCapabilities>
class DeviceCore : private Detail::BosFeature<Capabilities::BOS>,
                   private Detail::BusTimeFeature<Capabilities::BUS_TIME>,
                   private Detail::HighSpeedFeature<(Capabilities::SPEEDS &
                                                     SpeedBit(Speed::HIGH)) != 0U>
{
 public:
  /// 地址写入所处的控制传输阶段 / Control-transfer phase in which an address is written
  using Context = ControlContext;

  /**
   * @brief 构造设备核心 / Construct the device core
   * @param pool 端点池（由后端持有，须比本对象长寿）/ Endpoint pool (owned by the
   * backend, must outlive this object)
   * @param spec USB 规范版本 / USB specification release
   * @param speed 初始总线速度；HIGH 同时允许 FULL 与 HIGH，须属于 Capabilities::SPEEDS
   * / Initial bus speed; HIGH also allows FULL and HIGH, must be in Capabilities::SPEEDS
   * @param packet_size 端点 0 最大包长（8/16/32/64；HIGH 必须 64，LOW 必须 8）/ EP0 max
   * packet size (8/16/32/64; HIGH requires 64, LOW requires 8)
   * @param vid 厂商 ID / Vendor ID
   * @param pid 产品 ID / Product ID
   * @param bcd 设备版本号（bcdDevice）/ Device release number (bcdDevice)
   * @param languages 字符串描述符语言包列表 / String descriptor language packs
   * @param configs 每个 configuration 的 class/描述符项列表 / Class and descriptor items
   * of each configuration
   * @param uid 可选的唯一标识字节，作为序列号字符串的后缀；默认无 / Optional unique ID
   * bytes used as the suffix of the serial-number string; none by default
   */
  DeviceCore(
      EndpointPool& pool, USBSpec spec, Speed speed,
      DeviceDescriptor::PacketSize0 packet_size, uint16_t vid, uint16_t pid, uint16_t bcd,
      const std::initializer_list<const DescriptorStrings::LanguagePack*>& languages,
      const std::initializer_list<const std::initializer_list<ConfigDescriptorItem*>>&
          configs,
      ConstRawData uid = {nullptr, 0});
  virtual ~DeviceCore() = default;
  DeviceCore(const DeviceCore&) = delete;
  DeviceCore& operator=(const DeviceCore&) = delete;

  /// 请求初始化设备；由 owner 执行 / Request the initialization of the device; the owner
  /// carries it out
  /// @param in_isr 是否在 ISR / Whether in ISR context
  virtual void Init(bool in_isr) { RequestLifecycle(in_isr, INIT_EVENT, true); }
  /// 请求反初始化设备；由 owner 执行 / Request the deinitialization of the device; the
  /// owner carries it out
  /// @param in_isr 是否在 ISR / Whether in ISR context
  virtual void Deinit(bool in_isr) { RequestLifecycle(in_isr, DEINIT_EVENT, false); }
  /**
   * @brief 启动控制器并接通总线（例如接通 D+ 上拉），由后端实现 / Start the controller
   * and attach to the bus (for example enable the D+ pull-up); implemented by the backend
   * @param in_isr 是否在 ISR / Whether in ISR context
   */
  virtual void Start(bool in_isr) = 0;

  /**
   * @brief 断开总线并停止控制器，由后端实现 / Detach from the bus and stop the
   * controller; implemented by the backend
   * @param in_isr 是否在 ISR / Whether in ISR context
   */
  virtual void Stop(bool in_isr) = 0;

  /// 初始化是否已完成（EP0 已打开、描述符已准备好）；可在任意上下文读取 / Whether
  /// initialization has completed (EP0 open, descriptors prepared); readable from any
  /// context
  bool IsInited() const { return inited_.load(std::memory_order_acquire) != 0U; }
  /**
   * @brief 主机是否已选中非 0 配置（SET_CONFIGURATION），复位、取消配置、Deinit 后为
   *        false；可在任意上下文读取
   *        Whether the host has selected a non-zero configuration (SET_CONFIGURATION);
   *        false after a reset, a deconfiguration or Deinit; readable from any context
   */
  bool IsConfigured() const { return composition_.IsConfigured(); }
  /// 后端在收到 SETUP 包时调用：把 8 字节包存入单槽邮箱并请求 owner 处理 / Called by the
  /// backend on a received SETUP: stores the 8-byte packet in the one-slot mailbox and
  /// requests owner service
  /// @param in_isr 是否在 ISR / Whether in ISR context
  /// @param setup 收到的 SETUP 包 / The received SETUP packet
  void OnSetupPacket(bool in_isr, const SetupPacket* setup);

  /// 后端检测到总线复位时调用，随后在 owner 中重新初始化 / Called by the backend on a
  /// bus reset; the device is then reinitialized in the owner
  /// @param in_isr 是否在 ISR / Whether in ISR context
  void OnBusReset(bool in_isr) { RequestLifecycle(in_isr, RESET_EVENT, true); }

  /// 后端检测到断开连接时调用，随后在 owner 中反初始化 / Called by the
  /// backend on disconnect; the device is then deinitialized in the owner
  /// @param in_isr 是否在 ISR / Whether in ISR context
  void OnDisconnect(bool in_isr) { RequestLifecycle(in_isr, DISCONNECT_EVENT, false); }

  /// 后端检测到总线挂起时调用 / Called by the backend when the bus is suspended
  /// @param in_isr 是否在 ISR / Whether in ISR context
  void OnSuspend(bool in_isr) { RequestPower(in_isr, true); }

  /// 后端检测到总线恢复时调用 / Called by the backend when the bus resumes
  /// @param in_isr 是否在 ISR / Whether in ISR context
  void OnResume(bool in_isr) { RequestPower(in_isr, false); }

  /// 后端在每个帧起始（SOF）调用，用于端点停止超时计数和总线时间 / Called by the backend
  /// on every start of frame; drives endpoint-stop timeout counting and the bus time
  /// @param in_isr 是否在 ISR / Whether in ISR context
  /// @param frame 帧号（11 位，0xffff 表示未知）/ Frame number (11 bits, 0xffff when
  /// unknown)
  /// @param microframe 微帧号 0-7；0xff 表示控制器只报告完整帧 / Microframe 0-7; 0xff
  /// when the controller only reports full frames
  void OnSof(bool in_isr, uint16_t frame, uint8_t microframe = 0xff);
  /// 当前配置中是否有类需要总线时间；后端据此决定是否打开 SOF 中断 / Whether a class of
  /// the current configuration wants the bus time; the backend uses it to decide whether
  /// to enable the SOF interrupt
  bool WantsBusTime() const
  {
    if constexpr (Capabilities::BUS_TIME) return composition_.WantsBusTime();
    return false;
  }

 protected:
  /// 写入设备地址；由后端按控制器要求在 context 指明的阶段落实 / Program the device
  /// address; the backend applies it in the phase given by context, as its controller
  /// requires
  /// @param address 主机分配的地址（0-127）/ Address assigned by the host (0-127)
  /// @param context 当前所处的控制传输阶段 / Current control-transfer phase
  /// @return 错误码；状态阶段完成之前返回非 OK 时 SET_ADDRESS 被 STALL，
  /// STATUS_IN_COMPLETE 阶段的返回值不使用 / Error code; a non-OK result before the
  /// status stage completes stalls SET_ADDRESS, the result of the STATUS_IN_COMPLETE
  /// phase is not used
  virtual ErrorCode SetAddress(uint8_t address, Context context) = 0;

  /// 主机 SET_FEATURE(DEVICE_REMOTE_WAKEUP) 时调用，后端据此允许远程唤醒 / Called on
  /// SET_FEATURE(DEVICE_REMOTE_WAKEUP) so the backend can allow remote wakeup
  virtual void EnableRemoteWakeup() {}

  /// 主机 CLEAR_FEATURE(DEVICE_REMOTE_WAKEUP) 时调用 / Called on
  /// CLEAR_FEATURE(DEVICE_REMOTE_WAKEUP)
  virtual void DisableRemoteWakeup() {}

  /// 远程唤醒当前是否被允许，用于 GET_STATUS 的应答；默认 false / Whether remote wakeup
  /// is currently enabled, reported in the GET_STATUS answer; false by default
  virtual bool IsRemoteWakeupEnabled() const { return false; }
  /// 物理复位之后、EP0 打开之前，与后端相关的软件记账。
  /// Backend-specific software bookkeeping after physical reset, before EP0 open.
  virtual void OnControllerReset() {}
  /// 让 EP0 准备好接收下一个 SETUP：初始化完成和每次控制传输结束时在硬件保护内调用 /
  /// Make EP0 ready for the next SETUP; called inside the hardware guard after
  /// initialization and after every finished control transfer
  virtual void OnControlReady() {}
  /// 端点停止已连续 STOP_FRAME_LIMIT 帧未被确认。后端复位控制器并确认所有待处理的停止；
  /// 主机会重新枚举。
  /// An endpoint stop has not been confirmed for STOP_FRAME_LIMIT frames. The backend
  /// resets the controller and confirms every pending stop; the host enumerates again.
  virtual void OnStopTimeout(bool in_isr) { UNUSED(in_isr); }
  /// 所有端点停止都已确认（帧计数可以结束） / Every endpoint stop is confirmed (frame
  /// counting may end)
  virtual void OnStopsSettled() {}
  /// 总线复位后读取协商出的速度；默认返回当前速度 / Read the speed negotiated after a bus
  /// reset; returns the current speed by default
  virtual Speed ReadBusSpeed() const { return speed_; }

  /// 当前总线速度 / Current bus speed
  Speed GetSpeed() const { return speed_; }

 private:
  static constexpr uint32_t INIT_EVENT = EndpointPool::INIT_EVENT;
  static constexpr uint32_t DEINIT_EVENT = EndpointPool::DEINIT_EVENT;
  static constexpr uint32_t RESET_EVENT = EndpointPool::RESET_EVENT;
  static constexpr uint32_t DISCONNECT_EVENT = EndpointPool::DISCONNECT_EVENT;
  static constexpr uint32_t SETUP_EVENT = EndpointPool::SETUP_EVENT;
  static constexpr uint32_t POWER_EVENT = EndpointPool::POWER_EVENT;
  static constexpr uint32_t TIME_EVENT = EndpointPool::TIME_EVENT;
  static constexpr uint32_t CLASS_EVENT = EndpointPool::CLASS_EVENT;
  static constexpr uint32_t STOP_DONE_EVENT = EndpointPool::STOP_DONE_EVENT;
  static constexpr uint32_t STOP_TIMEOUT_EVENT = EndpointPool::STOP_TIMEOUT_EVENT;
  static constexpr uint32_t LIFECYCLE_EVENTS = EndpointPool::LIFECYCLE_EVENTS;

  void RequestLifecycle(bool in_isr, uint32_t event, bool enabled)
  {
    {
      EndpointPool::HardwareScope hardware(pool_);
      // 只使在此生命周期边界之前捕获的 SETUP 失效。之后捕获的 SETUP，
      // 即使两个事件位合并，也保持待处理。
      // Invalidate only SETUP captured before this lifecycle boundary. A SETUP
      // captured afterwards remains pending even when both event bits coalesce.
      captured_setup_pending_ = false;
      desired_enabled_.store(enabled ? 1U : 0U, std::memory_order_release);
      requested_suspend_.store(0U, std::memory_order_release);
      pool_.PublishControl(event);
    }
    pool_.RequestService(in_isr);
  }

  void RequestPower(bool in_isr, bool suspended)
  {
    {
      EndpointPool::HardwareScope hardware(pool_);
      requested_suspend_.store(suspended ? 1U : 0U, std::memory_order_release);
      pool_.PublishControl(POWER_EVENT);
    }
    pool_.RequestService(in_isr);
  }

  enum class Phase : uint8_t
  {
    IDLE,
    DATA_IN,
    DATA_OUT,
    STATUS_IN,
    STATUS_OUT,
    STALLED
  };
  /// 一旦所有端点停止都被确认就继续的一个步骤。
  /// A step that resumes once every endpoint stop is confirmed.
  enum class Continuation : uint8_t
  {
    NONE,
    /// 旧控制传输停止后分发 setup_。
    /// Dispatch setup_ after the old control transfer stopped.
    SETUP,
    /// 绑定 pending_config_ 并发送状态阶段。
    /// Bind pending_config_ and send the status stage.
    CONFIG,
    /// 发送 SET_INTERFACE 的状态阶段 / Send the status stage of SET_INTERFACE.
    STATUS
  };
  void ProcessEvents(bool in_isr, uint32_t events);
  void Continue(bool in_isr);
  void Initialize(bool in_isr);
  void Deinitialize(bool in_isr);
  void HandleSetup(bool in_isr, const SetupPacket& setup);
  void DispatchSetup(bool in_isr);
  ErrorCode HandleStandardRequest(bool in_isr);
  ErrorCode ClassRequest(bool in_isr);
  ErrorCode SendDescriptor(bool in_isr);
  void WriteControl(bool in_isr, ConstRawData data);
  void SendNextIn(bool in_isr);
  void ReceiveNextOut(bool in_isr);
  void StatusIn(bool in_isr);
  void ArmStatusOut(bool in_isr);
  void InComplete(bool in_isr, const Endpoint::Completion& completion);
  void OutComplete(bool in_isr, const Endpoint::Completion& completion);
  void Finish(bool in_isr);
  void Abort(bool in_isr);
  void Stall(bool in_isr);
  void DeliverBusTime(bool in_isr);
  bool TakeCapturedSetup(SetupPacket& setup);

  EndpointPool& pool_;
  DeviceComposition composition_;
  DeviceDescriptor device_desc_;
  Speed speed_;
  const uint16_t ep0_packet_size_;
  const uint32_t profile_speeds_;
  std::atomic<uint32_t> desired_enabled_{0};
  std::atomic<uint32_t> inited_{0};
  std::atomic<uint32_t> requested_suspend_{0};
  // 捕获、失效和消费使用后端的同一个短保护。
  // 这个八字节邮箱内部不执行任何回调或包处理。
  // Capture, invalidation and consumption use the backend's same short guard.
  // No callback or packet processing executes inside this eight-byte mailbox.
  SetupPacket captured_setup_{};
  bool captured_setup_pending_ = false;
  SetupPacket setup_{};
  DeviceClass* control_class_ = nullptr;
  Phase phase_ = Phase::IDLE;
  RawData control_out_;
  ConstRawData control_in_{nullptr, 0};
  size_t transferred_ = 0;
  size_t in_segment_size_ = 0;
  bool in_tail_zero_ = false;
  bool status_out_armed_ = false;
  bool status_out_seen_ = false;
  bool replacing_setup_ = false;
  uint8_t pending_address_ = 0xff;
  /// 端点停止期间被暂缓的事件 / Events held back while an endpoint stops.
  uint32_t deferred_events_ = 0;
  Continuation continuation_ = Continuation::NONE;
  uint16_t pending_config_ = 0;
  Callback<const Endpoint::Completion&> in_callback_ =
      Callback<const Endpoint::Completion&>::Create(
          [](bool context, DeviceCore* self, const Endpoint::Completion& completion)
          { self->InComplete(context, completion); }, this);
  Callback<const Endpoint::Completion&> out_callback_ =
      Callback<const Endpoint::Completion&>::Create(
          [](bool context, DeviceCore* self, const Endpoint::Completion& completion)
          { self->OutComplete(context, completion); }, this);
  Endpoint* in0_ = nullptr;
  Endpoint* out0_ = nullptr;
};
}  // namespace LibXR::USB

#include "dev_core.tpp"
