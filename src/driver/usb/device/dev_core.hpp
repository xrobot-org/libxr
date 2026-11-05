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
  using Context = ControlContext;
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

  virtual void Init(bool in_isr) { RequestLifecycle(in_isr, INIT_EVENT, true); }
  virtual void Deinit(bool in_isr) { RequestLifecycle(in_isr, DEINIT_EVENT, false); }
  virtual void Start(bool in_isr) = 0;
  virtual void Stop(bool in_isr) = 0;
  bool IsInited() const { return inited_.load(std::memory_order_acquire) != 0U; }
  void OnSetupPacket(bool in_isr, const SetupPacket* setup);
  void OnBusReset(bool in_isr) { RequestLifecycle(in_isr, RESET_EVENT, true); }
  void OnDisconnect(bool in_isr) { RequestLifecycle(in_isr, DISCONNECT_EVENT, false); }
  void OnSuspend(bool in_isr) { RequestPower(in_isr, true); }
  void OnResume(bool in_isr) { RequestPower(in_isr, false); }
  void OnSof(bool in_isr, uint16_t frame, uint8_t microframe = 0xff);
  bool WantsBusTime() const
  {
    if constexpr (Capabilities::BUS_TIME) return composition_.WantsBusTime();
    return false;
  }

 protected:
  virtual ErrorCode SetAddress(uint8_t address, Context context) = 0;
  virtual void EnableRemoteWakeup() {}
  virtual void DisableRemoteWakeup() {}
  virtual bool IsRemoteWakeupEnabled() const { return false; }
  /// Backend-specific software bookkeeping after physical reset, before EP0 open.
  virtual void OnControllerReset() {}
  virtual void OnControlReady() {}
  /// An endpoint stop has not been confirmed for STOP_FRAME_LIMIT frames. The backend
  /// resets the controller and confirms every pending stop; the host enumerates again.
  virtual void OnStopTimeout(bool in_isr) { UNUSED(in_isr); }
  /// Every endpoint stop is confirmed (frame counting may end).
  virtual void OnStopsSettled() {}
  virtual Speed ReadBusSpeed() const { return speed_; }
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
      EndpointPool::HardwareScope hardware(&pool_);
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
      EndpointPool::HardwareScope hardware(&pool_);
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
  /// A step that resumes once every endpoint stop is confirmed.
  enum class Continuation : uint8_t
  {
    NONE,
    SETUP,   ///< Dispatch setup_ after the old control transfer stopped.
    CONFIG,  ///< Bind pending_config_ and send the status stage.
    STATUS   ///< Send the status stage of SET_INTERFACE.
  };
  void ProcessEvents(bool in_isr, uint32_t events);
  void Continue(bool in_isr);
  void Initialize(bool in_isr);
  void Deinitialize(bool in_isr);
  void HandleSetup(bool in_isr, const SetupPacket& setup);
  void RefreshUnconfiguredSpeed();
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
  uint32_t deferred_events_ = 0;  ///< Events held back while an endpoint stops.
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
