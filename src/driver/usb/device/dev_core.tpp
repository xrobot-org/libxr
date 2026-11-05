#pragma once

namespace LibXR::USB
{
template <typename C>
DeviceCore<C>::DeviceCore(
    EndpointPool& pool, USBSpec spec, Speed speed,
    DeviceDescriptor::PacketSize0 packet_size, uint16_t vid, uint16_t pid, uint16_t bcd,
    const std::initializer_list<const DescriptorStrings::LanguagePack*>& languages,
    const std::initializer_list<const std::initializer_list<ConfigDescriptorItem*>>&
        configs,
    ConstRawData uid)
    : pool_(pool),
      composition_(pool, languages, configs, uid),
      device_desc_(spec, packet_size, vid, pid, bcd, composition_.GetConfigNum()),
      speed_(speed),
      ep0_packet_size_(static_cast<uint8_t>(packet_size)),
      profile_speeds_(speed == Speed::HIGH
                          ? (SpeedBit(Speed::FULL) | SpeedBit(Speed::HIGH))
                          : SpeedBit(speed)),
      control_out_(new uint8_t[composition_.ControlReceiveCapacity()],
                   composition_.ControlReceiveCapacity())
{
  static_assert(
      (C::SPEEDS &
       ~(SpeedBit(Speed::LOW) | SpeedBit(Speed::FULL) | SpeedBit(Speed::HIGH))) == 0U,
      "A SuperSpeed policy requires its descriptor/link backend implementation");
  REQUIRE((C::SPEEDS & SpeedBit(speed)) != 0U);
  REQUIRE(ep0_packet_size_ == 8U || ep0_packet_size_ == 16U || ep0_packet_size_ == 32U ||
          ep0_packet_size_ == 64U);
  REQUIRE(speed != Speed::HIGH || ep0_packet_size_ == 64U);
  REQUIRE(speed != Speed::LOW || ep0_packet_size_ == 8U);
  if constexpr (C::BOS) this->InitializeBos(composition_);
  pool_.SetSpeed(speed);
  pool_.SetControlHandler(
      Callback<uint32_t>::Create([](bool context, DeviceCore* self, uint32_t events)
                                 { self->ProcessEvents(context, events); }, this));
}

template <typename C>
void DeviceCore<C>::Initialize(bool in_isr)
{
  in0_ = pool_.GetEndpoint0In();
  out0_ = pool_.GetEndpoint0Out();
  REQUIRE(in0_ != nullptr && out0_ != nullptr);
  in0_->SetOnComplete(in_callback_);
  out0_->SetOnComplete(out_callback_);
  if (in0_->Configure(
          {Endpoint::Direction::IN, Endpoint::Type::CONTROL, ep0_packet_size_}, in_isr) !=
          ErrorCode::OK ||
      out0_->Configure(
          {Endpoint::Direction::OUT, Endpoint::Type::CONTROL, ep0_packet_size_},
          in_isr) != ErrorCode::OK)
  {
    inited_.store(0U, std::memory_order_release);
    return;
  }
  composition_.Init(in_isr);
  if (composition_.PrepareDescriptors(profile_speeds_, in_isr) != ErrorCode::OK)
  {
    inited_.store(0U, std::memory_order_release);
    return;
  }
  {
    EndpointPool::HardwareScope hardware(&pool_);
    OnControlReady();
    // An IRQ can run as soon as the hardware guard releases. Publish readiness
    // before then, or a valid first SETUP can be discarded as uninitialized.
    inited_.store(1U, std::memory_order_release);
  }
  pool_.KickEndpoints(in_isr);
}

template <typename C>
void DeviceCore<C>::Deinitialize(bool in_isr)
{
  Abort(in_isr);
  inited_.store(0U, std::memory_order_release);
  composition_.Deinit(in_isr);
  if (in0_) in0_->Close(in_isr);
  if (out0_) out0_->Close(in_isr);
}

template <typename C>
void DeviceCore<C>::ProcessEvents(bool in_isr, uint32_t events)
{
  // While an endpoint stop is unconfirmed nothing else proceeds; the confirmation
  // posts STOP_DONE_EVENT and the held events run then.
  if ((events & STOP_TIMEOUT_EVENT) && pool_.AnyStopping()) OnStopTimeout(in_isr);
  if ((events & STOP_DONE_EVENT) && !pool_.AnyStopping()) OnStopsSettled();
  events = (events | deferred_events_) & ~(STOP_DONE_EVENT | STOP_TIMEOUT_EVENT);
  deferred_events_ = 0;
  // A held-back lifecycle event also holds every endpoint start.
  pool_.SetLifecycleHold((events & LIFECYCLE_EVENTS) != 0U);
  if (pool_.AnyStopping())
  {
    deferred_events_ = events;
    return;
  }
  if (events & LIFECYCLE_EVENTS)
  {
    // First retire the old session; then reconcile the last published intent.
    // DEINIT|INIT is not equivalent to DEINIT, and INIT must not get lost in an
    // else-if branch merely because teardown was also pending.
    continuation_ = Continuation::NONE;
    Deinitialize(in_isr);
    if (pool_.AnyStopping())
    {
      deferred_events_ = events;  // Deinitialize is idempotent; it runs again then.
      return;
    }
    if (desired_enabled_.load(std::memory_order_acquire))
    {
      bool speed_ok = true;
      if (events & RESET_EVENT)
      {
        EndpointPool::HardwareScope hardware(&pool_);
        OnControllerReset();
        const Speed observed_speed = ReadBusSpeed();
        speed_ok = (profile_speeds_ & SpeedBit(observed_speed)) != 0U;
        if (speed_ok)
        {
          speed_ = observed_speed;
          pool_.SetSpeed(speed_);
        }
      }
      if (speed_ok)
      {
        pool_.SetSuspended(false, in_isr);
        if constexpr (C::BUS_TIME) this->previous = 0xffffffffU;
        Initialize(in_isr);
        if ((events & RESET_EVENT) && IsInited())
          composition_.Notify(in_isr, DeviceEvent::RESET);
      }
    }
    events &= ~LIFECYCLE_EVENTS;
    pool_.SetLifecycleHold(false);
    pool_.KickEndpoints(in_isr);
  }
  if (continuation_ != Continuation::NONE)
  {
    Continue(in_isr);
    if (pool_.AnyStopping())
    {
      deferred_events_ = events;
      return;
    }
  }
  if (events & CLASS_EVENT)
    for (size_t i = 0; i < composition_.ClassCount(); ++i)
    {
      DeviceClass* item = composition_.ClassAt(i);
      if (item->service_pending_.exchange(0U, std::memory_order_acq_rel) != 0U)
        item->OnService(in_isr);
    }
  if (!IsInited()) return;
  if (events & POWER_EVENT)
  {
    const bool suspend = requested_suspend_.load(std::memory_order_acquire) != 0U;
    if (pool_.IsSuspended() != suspend) pool_.SetSuspended(suspend, in_isr);
  }
  if (events & SETUP_EVENT)
  {
    SetupPacket setup;
    if (TakeCapturedSetup(setup)) HandleSetup(in_isr, setup);
  }
  if constexpr (C::BUS_TIME)
    if (events & TIME_EVENT) DeliverBusTime(in_isr);
}

template <typename C>
void DeviceCore<C>::Continue(bool in_isr)
{
  const Continuation step = continuation_;
  continuation_ = Continuation::NONE;
  switch (step)
  {
    case Continuation::SETUP:
      DispatchSetup(in_isr);
      break;
    case Continuation::CONFIG:
      if (composition_.FinishSwitchConfig(pending_config_, in_isr) != ErrorCode::OK)
        Stall(in_isr);
      else
        StatusIn(in_isr);
      break;
    case Continuation::STATUS:
      if (!pool_.ConfigurationValid())
        Stall(in_isr);
      else
        StatusIn(in_isr);
      break;
    case Continuation::NONE:
      break;
  }
}

template <typename C>
void DeviceCore<C>::OnSetupPacket(bool in_isr, const SetupPacket* setup)
{
  if (setup == nullptr) return;
  {
    EndpointPool::HardwareScope hardware(&pool_);
    captured_setup_ = *setup;
    captured_setup_pending_ = true;
    pool_.PublishControl(SETUP_EVENT);
  }
  pool_.RequestService(in_isr);
}

template <typename C>
bool DeviceCore<C>::TakeCapturedSetup(SetupPacket& setup)
{
  EndpointPool::HardwareScope hardware(&pool_);
  // A newer lifecycle request must be applied before a post-boundary SETUP.
  // Its publication is inside the same guard as mailbox invalidation.
  if (!captured_setup_pending_ || pool_.HasPendingControl(LIFECYCLE_EVENTS)) return false;
  setup = captured_setup_;
  captured_setup_pending_ = false;
  return true;
}

template <typename C>
void DeviceCore<C>::HandleSetup(bool in_isr, const SetupPacket& setup)
{
  // Consume a genuinely captured old completion first (a STATUS IN may still have to
  // apply its address); nonfinal old data must not be continued.
  replacing_setup_ = true;
  in0_->RetireCompletion(in_isr);
  out0_->RetireCompletion(in_isr);
  replacing_setup_ = false;
  Abort(in_isr);
  in0_->Flush(in_isr);
  out0_->Flush(in_isr);
  if (in0_->IsStalled()) (void)in0_->ClearStall(in_isr);
  if (out0_->IsStalled()) (void)out0_->ClearStall(in_isr);
  setup_ = setup;
  RefreshUnconfiguredSpeed();
  if (pool_.AnyStopping())
  {
    continuation_ = Continuation::SETUP;
    return;
  }
  DispatchSetup(in_isr);
}

// 有的控制器在报告复位时还没完成速度协商（CH32 USBHS 在高速 chirp 结束前读到全速），
// 所以在 SET_CONFIGURATION 之前的每个 SETUP 上重读一次：此时协商早已结束，描述符和端点
// 都还没按速度定下来。
// Some controllers report the reset before speed negotiation has finished (CH32 USBHS
// reads full speed until the high-speed chirp ends), so the speed is read again on every
// setup packet before SET_CONFIGURATION: negotiation is long over by then, and neither
// the descriptors nor the endpoints have been committed to a speed yet.
template <typename C>
void DeviceCore<C>::RefreshUnconfiguredSpeed()
{
  if (pool_.DataEnabled()) return;
  const Speed observed = ReadBusSpeed();
  if (observed == speed_ || (profile_speeds_ & SpeedBit(observed)) == 0U) return;
  speed_ = observed;
  EndpointPool::HardwareScope hardware(&pool_);
  pool_.SetSpeed(speed_);
}

template <typename C>
void DeviceCore<C>::DispatchSetup(bool in_isr)
{
  if constexpr ((C::SPEEDS & SpeedBit(Speed::HIGH)) != 0U)
    this->other_speed_descriptor = false;
  transferred_ = 0;
  status_out_armed_ = false;
  status_out_seen_ = false;
  const auto type = static_cast<RequestType>(setup_.bmRequestType & REQ_TYPE_MASK);
  ErrorCode result = type == RequestType::STANDARD ? HandleStandardRequest(in_isr)
                                                   : ClassRequest(in_isr);
  if (result != ErrorCode::OK) Stall(in_isr);
}

template <typename C>
void DeviceCore<C>::Abort(bool in_isr)
{
  DeviceClass* previous = control_class_;
  const SetupPacket old = setup_;
  control_class_ = nullptr;
  phase_ = Phase::IDLE;
  control_in_ = {nullptr, 0};
  transferred_ = 0;
  in_segment_size_ = 0;
  in_tail_zero_ = false;
  status_out_armed_ = false;
  status_out_seen_ = false;
  pending_address_ = 0xff;
  if (previous) previous->OnControlAbort(in_isr, old);
}

template <typename C>
void DeviceCore<C>::Finish(bool in_isr)
{
  DeviceClass* previous = control_class_;
  const SetupPacket completed = setup_;
  control_class_ = nullptr;
  phase_ = Phase::IDLE;
  status_out_armed_ = false;
  status_out_seen_ = false;
  {
    EndpointPool::HardwareScope hardware(&pool_);
    OnControlReady();
  }
  if (previous) previous->OnControlComplete(in_isr, completed);
}

template <typename C>
void DeviceCore<C>::Stall(bool in_isr)
{
  Abort(in_isr);
  phase_ = Phase::STALLED;
  if (in0_) (void)in0_->Stall(in_isr);
  if (out0_) (void)out0_->Stall(in_isr);
}

template <typename C>
void DeviceCore<C>::StatusIn(bool in_isr)
{
  phase_ = Phase::STATUS_IN;
  if (in0_->SubmitZeroLength(in_isr) != ErrorCode::OK) Stall(in_isr);
}

template <typename C>
void DeviceCore<C>::ArmStatusOut(bool in_isr)
{
  if (status_out_armed_) return;
  status_out_armed_ = true;
  if (out0_->Receive(0U, in_isr) != ErrorCode::OK) Stall(in_isr);
}

template <typename C>
void DeviceCore<C>::WriteControl(bool in_isr, ConstRawData data)
{
  control_in_ = {data.addr_, LibXR::min(data.size_, static_cast<size_t>(setup_.wLength))};
  if (control_class_ && control_in_.size_ > ep0_packet_size_ &&
      control_in_.size_ <= control_out_.size_)
  {
    Memory::FastCopy(control_out_.addr_, control_in_.addr_, control_in_.size_);
    control_in_.addr_ = control_out_.addr_;
  }
  transferred_ = 0;
  in_tail_zero_ = control_in_.size_ != 0U && control_in_.size_ < setup_.wLength &&
                  control_in_.size_ % ep0_packet_size_ == 0U;
  phase_ = Phase::DATA_IN;
  SendNextIn(in_isr);
}

template <typename C>
void DeviceCore<C>::SendNextIn(bool in_isr)
{
  const size_t remaining = control_in_.size_ - transferred_;
  const size_t amount = LibXR::min(remaining, static_cast<size_t>(ep0_packet_size_));
  // Early STATUS OUT arming is a hardware compatibility measure only. Completion
  // is still driven by the actual status packet, never by this arming call.
  ArmStatusOut(in_isr);
  if (phase_ != Phase::DATA_IN) return;
  Endpoint::Slot slot;
  if (in0_->AcquireSlot(slot, in_isr) != ErrorCode::OK)
  {
    Stall(in_isr);
    return;
  }
  REQUIRE(amount <= slot.data.size_);
  if (amount)
    Memory::FastCopy(slot.data.addr_,
                     static_cast<const uint8_t*>(control_in_.addr_) + transferred_,
                     amount);
  if constexpr ((C::SPEEDS & SpeedBit(Speed::HIGH)) != 0U)
    if (this->other_speed_descriptor && transferred_ <= 1U && transferred_ + amount > 1U)
      static_cast<uint8_t*>(slot.data.addr_)[1U - transferred_] = 7U;
  in_segment_size_ = amount;
  // A reply shorter than wLength that ends on a packet boundary is terminated by a
  // ZLP within the same request.
  const bool last = transferred_ + amount == control_in_.size_;
  if (in0_->Submit(slot, amount, last && in_tail_zero_, in_isr) != ErrorCode::OK)
  {
    in0_->ReleaseSlot(slot, in_isr);
    Stall(in_isr);
  }
}

template <typename C>
void DeviceCore<C>::ReceiveNextOut(bool in_isr)
{
  if (out0_->Receive(ep0_packet_size_, in_isr) != ErrorCode::OK) Stall(in_isr);
}

template <typename C>
void DeviceCore<C>::InComplete(bool in_isr, const Endpoint::Completion& completion)
{
  if (!IsInited() || completion.result == ErrorCode::ABORTED) return;
  if (completion.result != ErrorCode::OK)
  {
    if (!replacing_setup_) Stall(in_isr);
    return;
  }
  if (phase_ == Phase::STATUS_IN)
  {
    if (pending_address_ != 0xff)
    {
      (void)SetAddress(pending_address_, Context::STATUS_IN_COMPLETE);
      pending_address_ = 0xff;
    }
    Finish(in_isr);
    return;
  }
  if (phase_ != Phase::DATA_IN || completion.actual != in_segment_size_)
  {
    if (!replacing_setup_) Stall(in_isr);
    return;
  }
  transferred_ += completion.actual;
  if (transferred_ < control_in_.size_)
  {
    if (!replacing_setup_ && !status_out_seen_) SendNextIn(in_isr);
    return;
  }
  if (control_class_)
  {
    ConstRawData complete = control_in_;
    if (control_class_->OnControlData(in_isr, setup_, complete) != ErrorCode::OK)
    {
      Stall(in_isr);
      return;
    }
  }
  phase_ = Phase::STATUS_OUT;
  if (status_out_seen_) Finish(in_isr);
}

template <typename C>
void DeviceCore<C>::OutComplete(bool in_isr, const Endpoint::Completion& completion)
{
  const Endpoint::Slot lease = completion.slot;
  const bool data_stage = IsInited() && phase_ == Phase::DATA_OUT && !replacing_setup_ &&
                          completion.result == ErrorCode::OK;
  const bool fits = completion.actual <= setup_.wLength - transferred_ &&
                    transferred_ + completion.actual <= control_out_.size_;
  // The core copies what it needs and returns the lease before acting on it.
  if (lease.Valid())
  {
    if (data_stage && fits && completion.actual != 0U)
      Memory::FastCopy(static_cast<uint8_t*>(control_out_.addr_) + transferred_,
                       lease.data.addr_, completion.actual);
    out0_->ReleaseSlot(lease, in_isr);
  }
  if (!IsInited() || completion.result == ErrorCode::ABORTED) return;
  if (status_out_armed_ && (phase_ == Phase::DATA_IN || phase_ == Phase::STATUS_OUT))
  {
    if (completion.result != ErrorCode::OK || completion.actual != 0U)
    {
      Stall(in_isr);
      return;
    }
    status_out_seen_ = true;
    if (phase_ == Phase::STATUS_OUT) Finish(in_isr);
    return;
  }
  if (phase_ != Phase::DATA_OUT || replacing_setup_) return;
  if (!data_stage || !fits)
  {
    Stall(in_isr);
    return;
  }
  transferred_ += completion.actual;
  if (transferred_ < setup_.wLength && completion.actual == ep0_packet_size_)
  {
    ReceiveNextOut(in_isr);
    return;
  }
  ConstRawData full{control_out_.addr_, transferred_};
  if (control_class_ == nullptr ||
      control_class_->OnControlData(in_isr, setup_, full) != ErrorCode::OK)
  {
    Stall(in_isr);
    return;
  }
  StatusIn(in_isr);
}

template <typename C>
ErrorCode DeviceCore<C>::ClassRequest(bool in_isr)
{
  const auto type = static_cast<RequestType>(setup_.bmRequestType & REQ_TYPE_MASK);
  if (type != RequestType::CLASS && type != RequestType::VENDOR)
    return ErrorCode::NOT_SUPPORT;
  if constexpr (C::BOS)
  {
    if (type == RequestType::VENDOR)
    {
      this->SelectBos(composition_);
      BosVendorResult bos{};
      const auto result = this->manager->ProcessVendorRequest(in_isr, &setup_, bos);
      if (result == ErrorCode::OK && bos.handled)
      {
        if (setup_.wLength && (setup_.bmRequestType & REQ_DIRECTION_MASK))
          WriteControl(in_isr, bos.in_data);
        else if (setup_.wLength == 0U)
          StatusIn(in_isr);
        else
          return ErrorCode::ARG_ERR;
        return ErrorCode::OK;
      }
      if (result != ErrorCode::OK && result != ErrorCode::NOT_SUPPORT) return result;
    }
  }
  const auto recipient =
      static_cast<Recipient>(setup_.bmRequestType & REQ_RECIPIENT_MASK);
  DeviceClass* item =
      recipient == Recipient::INTERFACE
          ? composition_.FindClassByInterfaceNumber(setup_.wIndex & 0xffU)
      : recipient == Recipient::ENDPOINT
          ? composition_.FindClassByEndpointAddress(static_cast<uint8_t>(setup_.wIndex))
          : nullptr;
  if (!item) return ErrorCode::NOT_FOUND;
  typename DeviceClass::ControlTransferResult result{};
  const ErrorCode answer = item->OnControlRequest(in_isr, setup_, result);
  if (answer != ErrorCode::OK) return answer;
  control_class_ = item;
  if (setup_.wLength == 0U)
  {
    StatusIn(in_isr);
    return ErrorCode::OK;
  }
  if (setup_.bmRequestType & REQ_DIRECTION_MASK)
  {
    if (result.write_data.size_ && result.write_data.addr_ == nullptr)
      return ErrorCode::ARG_ERR;
    WriteControl(in_isr,
                 result.write_data);  // An empty IN response is a valid data stage.
    return ErrorCode::OK;
  }
  // The core receives the whole OUT data stage and hands it over by OnControlData.
  if (result.write_data.size_ || setup_.wLength > control_out_.size_)
    return ErrorCode::ARG_ERR;
  phase_ = Phase::DATA_OUT;
  transferred_ = 0;
  ReceiveNextOut(in_isr);
  return ErrorCode::OK;
}

template <typename C>
ErrorCode DeviceCore<C>::SendDescriptor(bool in_isr)
{
  const uint8_t type = static_cast<uint8_t>(setup_.wValue >> 8U);
  const uint8_t index = static_cast<uint8_t>(setup_.wValue);
  ConstRawData data{nullptr, 0};
  switch (type)
  {
    case 1:
      if (index != 0U) return ErrorCode::ARG_ERR;
      (void)composition_.TryOverrideDeviceDescriptor(device_desc_);
      data = device_desc_.GetData();
      break;
    case 2:
      data = composition_.GetConfigDescriptor(index, speed_);
      if (data.size_ == 0U) return ErrorCode::NOT_FOUND;
      break;
    case 6:
      if constexpr ((C::SPEEDS & SpeedBit(Speed::HIGH)) != 0U)
      {
        if ((profile_speeds_ & SpeedBit(Speed::HIGH)) == 0U || index != 0U)
          return ErrorCode::NOT_SUPPORT;
        (void)composition_.TryOverrideDeviceDescriptor(device_desc_);
        this->qualifier[0] = 10U;
        this->qualifier[1] = 6U;
        const auto spec = static_cast<uint16_t>(device_desc_.GetUSBSpec());
        this->qualifier[2] = static_cast<uint8_t>(spec);
        this->qualifier[3] = static_cast<uint8_t>(spec >> 8U);
        this->qualifier[4] = static_cast<uint8_t>(device_desc_.data_.bDeviceClass);
        this->qualifier[5] = device_desc_.data_.bDeviceSubClass;
        this->qualifier[6] = device_desc_.data_.bDeviceProtocol;
        this->qualifier[7] = static_cast<uint8_t>(ep0_packet_size_);
        this->qualifier[8] = static_cast<uint8_t>(composition_.GetConfigNum());
        this->qualifier[9] = 0;
        data = {this->qualifier, sizeof(this->qualifier)};
      }
      else
        return ErrorCode::NOT_SUPPORT;
      break;
    case 7:
      if constexpr ((C::SPEEDS & SpeedBit(Speed::HIGH)) != 0U)
      {
        if ((profile_speeds_ & SpeedBit(Speed::HIGH)) == 0U)
          return ErrorCode::NOT_SUPPORT;
        const Speed other = speed_ == Speed::HIGH ? Speed::FULL : Speed::HIGH;
        data = composition_.GetConfigDescriptor(index, other);
        if (data.size_ == 0U) return ErrorCode::NOT_FOUND;
        this->other_speed_descriptor = true;
      }
      else
        return ErrorCode::NOT_SUPPORT;
      break;
    case 3:
      if (const auto result =
              composition_.GetStringDescriptor(index, setup_.wIndex, data);
          result != ErrorCode::OK)
        return result;
      break;
    case 15:
      if constexpr (C::BOS)
      {
        if (index != 0U || device_desc_.GetUSBSpec() < USBSpec::USB_2_1)
          return ErrorCode::NOT_SUPPORT;
        this->SelectBos(composition_);
        data = this->manager->GetBosDescriptor();
      }
      else
        return ErrorCode::NOT_SUPPORT;
      break;
    default:
    {
      if ((setup_.bmRequestType & REQ_RECIPIENT_MASK) !=
          static_cast<uint8_t>(Recipient::INTERFACE))
        return ErrorCode::NOT_SUPPORT;
      auto* item = composition_.FindClassByInterfaceNumber(setup_.wIndex & 0xffU);
      if (!item) return ErrorCode::NOT_FOUND;
      const auto result = item->OnGetDescriptor(in_isr, setup_.bRequest, setup_.wValue,
                                                setup_.wLength, data);
      if (result != ErrorCode::OK) return result;
    }
  }
  WriteControl(in_isr, data);
  return ErrorCode::OK;
}

template <typename C>
ErrorCode DeviceCore<C>::HandleStandardRequest(bool in_isr)
{
  const bool input = (setup_.bmRequestType & REQ_DIRECTION_MASK) != 0U;
  const auto recipient =
      static_cast<Recipient>(setup_.bmRequestType & REQ_RECIPIENT_MASK);
  const auto request = static_cast<StandardRequest>(setup_.bRequest);
  switch (request)
  {
    case StandardRequest::GET_DESCRIPTOR:
      if (!input || setup_.wLength == 0U) return ErrorCode::ARG_ERR;
      return SendDescriptor(in_isr);
    case StandardRequest::SET_ADDRESS:
      if (input || recipient != Recipient::DEVICE || setup_.wIndex || setup_.wLength ||
          setup_.wValue > 127U || composition_.GetCurrentConfig())
        return ErrorCode::ARG_ERR;
      pending_address_ = static_cast<uint8_t>(setup_.wValue);
      if (const auto result = SetAddress(pending_address_, Context::SETUP_BEFORE_STATUS);
          result != ErrorCode::OK)
        return result;
      StatusIn(in_isr);
      return SetAddress(pending_address_, Context::STATUS_IN_ARMED);
    case StandardRequest::GET_CONFIGURATION:
    {
      if (!input || recipient != Recipient::DEVICE || setup_.wValue || setup_.wIndex ||
          setup_.wLength != 1U)
        return ErrorCode::ARG_ERR;
      const uint8_t config = static_cast<uint8_t>(composition_.GetCurrentConfig());
      WriteControl(in_isr, {&config, 1});
      return ErrorCode::OK;
    }
    case StandardRequest::SET_CONFIGURATION:
    {
      if (input || recipient != Recipient::DEVICE || setup_.wIndex || setup_.wLength ||
          setup_.wValue > 255U)
        return ErrorCode::ARG_ERR;
      auto result = composition_.BeginSwitchConfig(setup_.wValue, in_isr);
      if (result != ErrorCode::OK) return result;
      if (pool_.AnyStopping())
      {
        // The status stage follows the binding, after the old endpoints stopped.
        pending_config_ = setup_.wValue;
        continuation_ = Continuation::CONFIG;
        return ErrorCode::OK;
      }
      result = composition_.FinishSwitchConfig(setup_.wValue, in_isr);
      if (result != ErrorCode::OK) return result;
      StatusIn(in_isr);
      return ErrorCode::OK;
    }
    case StandardRequest::GET_INTERFACE:
    case StandardRequest::SET_INTERFACE:
    {
      const bool get = request == StandardRequest::GET_INTERFACE;
      if (recipient != Recipient::INTERFACE || input != get || setup_.wIndex > 255U ||
          (get ? (setup_.wValue != 0U || setup_.wLength != 1U)
               : (setup_.wValue > 255U || setup_.wLength != 0U)))
        return ErrorCode::ARG_ERR;
      auto* item = composition_.FindClassByInterfaceNumber(setup_.wIndex);
      if (!item) return ErrorCode::NOT_FOUND;
      if (get)
      {
        uint8_t alt = 0;
        const auto result = item->GetAltSetting(static_cast<uint8_t>(setup_.wIndex), alt);
        if (result != ErrorCode::OK && result != ErrorCode::NOT_SUPPORT) return result;
        WriteControl(in_isr, {&alt, 1});
      }
      else
      {
        const auto result = item->SetAltSetting(static_cast<uint8_t>(setup_.wIndex),
                                                static_cast<uint8_t>(setup_.wValue));
        if (result != ErrorCode::OK &&
            !(result == ErrorCode::NOT_SUPPORT && setup_.wValue == 0U))
          return ErrorCode::ARG_ERR;
        if (pool_.AnyStopping())
        {
          continuation_ = Continuation::STATUS;
          return ErrorCode::OK;
        }
        if (!pool_.ConfigurationValid()) return ErrorCode::ARG_ERR;
        StatusIn(in_isr);
      }
      return ErrorCode::OK;
    }
    case StandardRequest::GET_STATUS:
    {
      if (!input || setup_.wLength != 2U || setup_.wValue) return ErrorCode::ARG_ERR;
      uint16_t status = 0;
      if (recipient == Recipient::DEVICE)
      {
        if (setup_.wIndex) return ErrorCode::ARG_ERR;
        status =
            (composition_.GetDeviceStatus() & 1U) | (IsRemoteWakeupEnabled() ? 2U : 0U);
      }
      else if (recipient == Recipient::INTERFACE)
      {
        if (setup_.wIndex > 255U ||
            !composition_.FindClassByInterfaceNumber(setup_.wIndex))
          return ErrorCode::NOT_FOUND;
      }
      else if (recipient == Recipient::ENDPOINT)
      {
        if ((setup_.wIndex & 0xff70U) != 0U) return ErrorCode::ARG_ERR;
        Endpoint* ep = nullptr;
        if (pool_.FindEndpoint(static_cast<uint8_t>(setup_.wIndex), ep) != ErrorCode::OK)
          return ErrorCode::NOT_FOUND;
        status = ep->IsStalled() ? 1U : 0U;
      }
      else
        return ErrorCode::ARG_ERR;
      const uint8_t bytes[] = {static_cast<uint8_t>(status),
                               static_cast<uint8_t>(status >> 8U)};
      WriteControl(in_isr, {bytes, 2});
      return ErrorCode::OK;
    }
    case StandardRequest::SET_FEATURE:
    case StandardRequest::CLEAR_FEATURE:
    {
      if (input || setup_.wLength) return ErrorCode::ARG_ERR;
      const bool set = request == StandardRequest::SET_FEATURE;
      if (recipient == Recipient::ENDPOINT)
      {
        if (setup_.wValue || (setup_.wIndex & 0xff70U) || !(setup_.wIndex & 0x0fU))
          return ErrorCode::ARG_ERR;
        Endpoint* ep = nullptr;
        if (pool_.FindEndpoint(static_cast<uint8_t>(setup_.wIndex), ep) !=
                ErrorCode::OK ||
            ep->GetType() == Endpoint::Type::ISOCHRONOUS)
          return ErrorCode::NOT_SUPPORT;
        const auto result = set ? ep->Stall(in_isr) : ep->ClearStall(in_isr);
        if (result != ErrorCode::OK) return result;
        if (!set)
          composition_.Notify(in_isr, DeviceEvent::ENDPOINT_RESUMED,
                              static_cast<uint8_t>(setup_.wIndex));
      }
      else if (recipient == Recipient::DEVICE && setup_.wValue == 1U &&
               setup_.wIndex == 0U && composition_.GetCurrentConfig() &&
               (composition_.GetDeviceStatus() & 2U))
      {
        if (set)
          EnableRemoteWakeup();
        else
          DisableRemoteWakeup();
      }
      else
        return ErrorCode::NOT_SUPPORT;
      StatusIn(in_isr);
      return ErrorCode::OK;
    }
    case StandardRequest::SYNCH_FRAME:
      if constexpr (C::BUS_TIME)
      {
        if (!input || recipient != Recipient::ENDPOINT || setup_.wValue ||
            setup_.wLength != 2U || (setup_.wIndex & 0xff70U))
          return ErrorCode::ARG_ERR;
        Endpoint* ep = nullptr;
        if (pool_.FindEndpoint(static_cast<uint8_t>(setup_.wIndex), ep) !=
                ErrorCode::OK ||
            ep->GetType() != Endpoint::Type::ISOCHRONOUS)
          return ErrorCode::NOT_SUPPORT;
        const uint16_t frame = static_cast<uint16_t>(
            this->observed.load(std::memory_order_acquire) & 0x7ffU);
        const uint8_t bytes[] = {static_cast<uint8_t>(frame),
                                 static_cast<uint8_t>(frame >> 8U)};
        WriteControl(in_isr, {bytes, 2});
        return ErrorCode::OK;
      }
      return ErrorCode::NOT_SUPPORT;
    default:
      return ErrorCode::NOT_SUPPORT;
  }
}

template <typename C>
void DeviceCore<C>::OnSof(bool in_isr, uint16_t frame, uint8_t microframe)
{
  if (pool_.CountStopFrame(frame)) pool_.PostControl(STOP_TIMEOUT_EVENT, in_isr);
  if constexpr (C::BUS_TIME)
  {
    this->observed_ms.store(static_cast<uint32_t>(Timebase::GetMilliseconds()),
                            std::memory_order_relaxed);
    this->observed.store((frame == 0xffffU ? 0xffffU : (frame & 0x7ffU)) |
                             (static_cast<uint32_t>(microframe) << 16U),
                         std::memory_order_release);
    if (composition_.WantsBusTime()) pool_.PostControl(TIME_EVENT, in_isr);
  }
  else
  {
    UNUSED(in_isr);
    UNUSED(frame);
    UNUSED(microframe);
  }
}

template <typename C>
void DeviceCore<C>::DeliverBusTime(bool in_isr)
{
  if constexpr (C::BUS_TIME)
  {
    const uint32_t now = this->observed.load(std::memory_order_acquire);
    const uint32_t ms = this->observed_ms.load(std::memory_order_relaxed);
    BusTime time;
    const uint16_t frame = static_cast<uint16_t>(now & 0xffffU);
    time.microframe = static_cast<uint8_t>(now >> 16U);
    if (this->previous != 0xffffffffU && frame != 0xffffU &&
        (this->previous & 0xffffU) != 0xffffU && ms - this->previous_ms < 2048U)
    {
      const uint8_t previous_micro = static_cast<uint8_t>(this->previous >> 16U);
      if ((previous_micro == 0xffU) == (time.microframe == 0xffU))
      {
        if (time.microframe == 0xffU)
          time.elapsed_intervals = (frame - (this->previous & 0x7ffU)) & 0x7ffU;
        else
        {
          const uint32_t current = frame * 8U + time.microframe;
          const uint32_t previous = (this->previous & 0x7ffU) * 8U + previous_micro;
          time.elapsed_intervals = (current - previous) & 0x3fffU;
        }
      }
    }
    this->previous = now;
    this->previous_ms = ms;
    composition_.NotifyBusTime(in_isr, time);
  }
}
}  // namespace LibXR::USB
