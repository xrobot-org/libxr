#include "dev_core.hpp"

#include "libxr_mem.hpp"

using namespace LibXR::USB;
using LibXR::ConstRawData;
using LibXR::ErrorCode;
using LibXR::RawData;
using LibXR::USB::DescriptorType;
using LibXR::USB::Recipient;
using LibXR::USB::RequestType;
using LibXR::USB::Speed;
using LibXR::USB::StandardRequest;
using LibXR::USB::USBSpec;

namespace
{

/// 方向位置位即 IN 请求 / The direction bit set means an IN request
bool IsIn(const LibXR::USB::SetupPacket& setup)
{
  return (setup.bmRequestType & LibXR::USB::REQ_DIRECTION_MASK) != 0U;
}

/// 请求的接收者 / Recipient of the request
Recipient RecipientOf(const LibXR::USB::SetupPacket& setup)
{
  return static_cast<Recipient>(setup.bmRequestType & LibXR::USB::REQ_RECIPIENT_MASK);
}

/// 端点地址的 wIndex 是否合法（只有方向位和端点号）/ Whether wIndex is a valid
/// endpoint address (direction bit and number only)
bool IsEndpointIndex(uint16_t index) { return (index & 0xff70U) == 0U; }

/// 标准特性选择子 / Standard feature selectors
constexpr uint16_t FEATURE_ENDPOINT_HALT = 0U;

}  // namespace

DeviceCore::DeviceCore(
    DeviceService& service, EndpointPool& pool, USBSpec spec, Speed speed,
    DeviceDescriptor::PacketSize0 packet_size, uint16_t vid, uint16_t pid, uint16_t bcd,
    const std::initializer_list<const DescriptorStrings::LanguagePack*>& languages,
    const std::initializer_list<const std::initializer_list<DeviceClass*>>& configs,
    ConstRawData uid)
    : service_(service),
      pool_(pool),
      max_speed_(speed),
      device_desc_(spec, packet_size, vid, pid, bcd,
                   static_cast<uint8_t>(configs.size())),
      composition_(pool, service, languages, configs, uid)
{
  const auto packet = static_cast<uint8_t>(packet_size);
  ASSERT(speed == Speed::FULL || speed == Speed::HIGH);
  ASSERT(packet == 8U || packet == 16U || packet == 32U || packet == 64U);
  ASSERT(speed != Speed::HIGH || packet == 64U);
  ASSERT(pool_.GetEndpoint0In() != nullptr && pool_.GetEndpoint0Out() != nullptr);

  const size_t capacity = composition_.ControlReceiveCapacity();
  control_buffer_ = {new uint8_t[capacity], capacity};
  (void)composition_.WriteDeviceDescriptor(device_desc_);

  // EP0 的回调在这里设一次，之后每次 Configure 都沿用。
  // EP0's callbacks are set here once; every later Configure keeps them.
  pool_.GetEndpoint0In()->SetOnComplete(Callback<const Endpoint::Completion&>::Create(
      [](bool in_isr, DeviceCore* self, const Endpoint::Completion& completion)
      { self->InComplete(in_isr, completion); }, this));
  pool_.GetEndpoint0Out()->SetOnComplete(Callback<const Endpoint::Completion&>::Create(
      [](bool in_isr, DeviceCore* self, const Endpoint::Completion& completion)
      { self->OutComplete(in_isr, completion); }, this));
  service_.SetDeviceHandler(
      Callback<uint32_t>::Create([](bool in_isr, DeviceCore* self, uint32_t events)
                                 { self->HandleDeviceEvents(in_isr, events); }, this));
}

void DeviceCore::Init(bool in_isr)
{
  enable_requested_.store(1U, std::memory_order_release);
  service_.Request(ENABLE_EVENT, in_isr);
}

void DeviceCore::Deinit(bool in_isr)
{
  enable_requested_.store(0U, std::memory_order_release);
  service_.Request(ENABLE_EVENT, in_isr);
}

void DeviceCore::OnSetup(bool in_isr, const SetupPacket& setup)
{
  if (!inited_)
  {
    return;
  }
  // 旧传输到此为止：先回到空闲，Flush 交回的旧请求因此不会被当作当前传输处理。
  // The old transfer ends here: back to idle first, so the old requests a flush hands
  // back are not taken for the current transfer.
  AbortTransfer(in_isr);
  Endpoint* in0 = pool_.GetEndpoint0In();
  Endpoint* out0 = pool_.GetEndpoint0Out();
  // Flush 不离开 STALLED；上一个传输被拒绝时，清除 STALL 才能收发新传输。
  // Flush does not leave STALLED; after a refused transfer, clearing the halt is what
  // lets the new one move data.
  if (in0->IsStalled())
  {
    (void)in0->ClearStall(in_isr);
  }
  else
  {
    in0->Flush(in_isr);
  }
  if (out0->IsStalled())
  {
    (void)out0->ClearStall(in_isr);
  }
  else
  {
    out0->Flush(in_isr);
  }
  setup_ = setup;
  if (Dispatch(in_isr) != ErrorCode::OK)
  {
    StallTransfer(in_isr);
  }
}

void DeviceCore::OnBusReset(bool in_isr, Speed speed)
{
  ASSERT(static_cast<uint8_t>(speed) <= static_cast<uint8_t>(max_speed_));
  Deinitialize(in_isr);
  service_.SetSpeed(speed);
  suspended_ = false;
  if (enable_requested_.load(std::memory_order_acquire) != 0U && !detach_pending_)
  {
    Initialize(in_isr);
  }
}

void DeviceCore::OnSuspend(bool in_isr)
{
  UNUSED(in_isr);
  suspended_ = true;
  UpdateGate();
}

void DeviceCore::OnResume(bool in_isr)
{
  UNUSED(in_isr);
  suspended_ = false;
  UpdateGate();
}

void DeviceCore::OnDisconnect(bool in_isr)
{
  Deinitialize(in_isr);
  suspended_ = false;
}

void DeviceCore::Start(bool in_isr)
{
  stop_requested_.store(0U);
  // 这次停止可能已经开始处理、还在等端点停稳后断开：等它断开再启动，不与它同时写
  // 控制器。这里先写"不停止"再读"正在停止"，处理程序先写"正在停止"再读"要停止"，两边
  // 至少有一边看得到对方。
  // That stop may already be under way and awaiting the endpoints' quiescence before
  // it detaches: wait for the detach before starting, so the two never write the
  // controller at the same time. This side writes "no stop" and then reads "stopping";
  // the handler writes "stopping" and then reads "stop": at least one of them sees the
  // other.
  while (stop_busy_.load() != 0U)
  {
  }
  StartController(in_isr);
}

void DeviceCore::Stop(bool in_isr)
{
  stop_requested_.store(1U);
  service_.Request(STOP_EVENT, in_isr);
}

void DeviceCore::OnStartOfFrame(bool in_isr, uint16_t frame, uint8_t microframe)
{
  composition_.StartOfFrame(in_isr, frame, microframe);
}

void DeviceCore::HandleDeviceEvents(bool in_isr, uint32_t events)
{
  if ((events & DeviceService::STOP_FAILED_EVENT) != 0U)
  {
    AbortTransfer(in_isr);
    if (detach_pending_)
    {
      // 等断开时有停止失败：不重新接上，直接让控制器静止并断开。
      // A stop failed while a detach is awaited: no re-attach, the controller is made
      // quiescent and detached right away.
      FinishDetach(in_isr);
    }
    else
    {
      ResetController(in_isr);
      // 复位后每个停止立即结束：被卡住的槽在这里变空闲，所有端点关闭。
      // After the reset every stop ends at once: the stuck slot becomes free here and
      // every endpoint is closed.
      service_.CloseAllEndpoints(in_isr);
      Deinitialize(in_isr);
    }
  }
  if ((events & STOP_EVENT) != 0U)
  {
    stop_busy_.store(1U);
    if (stop_requested_.exchange(0U) != 0U && !detach_pending_)
    {
      // 按平常的路径收尾，不等总线：没有停稳的停止之后由后端结束。
      // Wind up the ordinary way without waiting for the bus: a stop that is not
      // quiescent is ended by the backend later.
      AbortTransfer(in_isr);
      Deinitialize(in_isr);
      detach_pending_ = true;
      service_.WatchStops(true);
    }
    if (!detach_pending_)
    {
      stop_busy_.store(0U);
    }
  }
  if (detach_pending_ && !service_.AnyStopPending())
  {
    FinishDetach(in_isr);
  }
  if ((events & ENABLE_EVENT) != 0U)
  {
    const bool enable = enable_requested_.load(std::memory_order_acquire) != 0U;
    if (enable && !inited_ && !detach_pending_)
    {
      Initialize(in_isr);
    }
    else if (!enable && inited_)
    {
      Deinitialize(in_isr);
    }
  }
  if ((events & DeviceClass::SERVICE_EVENT) != 0U)
  {
    composition_.ServiceClasses(in_isr);
  }
}

void DeviceCore::FinishDetach(bool in_isr)
{
  service_.WatchStops(false);
  StopController(in_isr);
  // 等断开期间的总线复位可能又打开了 EP0；控制器静止之后每个停止立即结束。
  // A bus reset while the detach was awaited may have reopened EP0; with the controller
  // quiescent every stop ends at once.
  service_.CloseAllEndpoints(in_isr);
  Deinitialize(in_isr);
  detach_pending_ = false;
  stop_busy_.store(0U);
}

void DeviceCore::Initialize(bool in_isr)
{
  Endpoint* in0 = pool_.GetEndpoint0In();
  Endpoint* out0 = pool_.GetEndpoint0Out();
  const auto packet = static_cast<uint16_t>(device_desc_.data_.bMaxPacketSize0);
  // EP0 的包长必须正好是设备描述符里写的，否则主机和硬件对包边界的看法不同。
  // EP0's packet size must be exactly the one in the device descriptor, or the host
  // and the hardware disagree on packet boundaries.
  const bool ok =
      in0->Configure({Endpoint::Direction::IN, Endpoint::Type::CONTROL, packet},
                     in_isr) == ErrorCode::OK &&
      out0->Configure({Endpoint::Direction::OUT, Endpoint::Type::CONTROL, packet},
                      in_isr) == ErrorCode::OK &&
      in0->MaxPacketSize() == packet && out0->MaxPacketSize() == packet;
  if (!ok)
  {
    in0->Close(in_isr);
    out0->Close(in_isr);
    return;
  }
  inited_ = true;
  UpdateGate();
  OnControlReady();
}

void DeviceCore::Deinitialize(bool in_isr)
{
  if (!inited_)
  {
    return;
  }
  AbortTransfer(in_isr);
  (void)composition_.SetConfiguration(0U, service_.GetSpeed(), in_isr);
  pool_.GetEndpoint0In()->Close(in_isr);
  pool_.GetEndpoint0Out()->Close(in_isr);
  inited_ = false;
  UpdateGate();
}

void DeviceCore::UpdateGate()
{
  using Gate = DeviceService::StartGate;
  Gate gate = Gate::CLOSED;
  if (inited_ && !suspended_)
  {
    gate = (composition_.GetConfiguration() != 0U) ? Gate::ALL : Gate::EP0;
  }
  service_.SetStartGate(gate);
}

ErrorCode DeviceCore::Dispatch(bool in_isr)
{
  const auto type = static_cast<RequestType>(setup_.bmRequestType & REQ_TYPE_MASK);
  return (type == RequestType::STANDARD) ? HandleStandard(in_isr)
                                         : HandleClassOrVendor(in_isr);
}

ErrorCode DeviceCore::HandleStandard(bool in_isr)
{
  const bool input = IsIn(setup_);
  const Recipient recipient = RecipientOf(setup_);
  auto* answer = static_cast<uint8_t*>(control_buffer_.addr_);
  switch (static_cast<StandardRequest>(setup_.bRequest))
  {
    case StandardRequest::GET_DESCRIPTOR:
      if (!input || setup_.wLength == 0U)
      {
        return ErrorCode::ARG_ERR;
      }
      return SendDescriptor(in_isr);

    case StandardRequest::SET_ADDRESS:
      if (input || recipient != Recipient::DEVICE || setup_.wIndex != 0U ||
          setup_.wLength != 0U || setup_.wValue > 127U ||
          composition_.GetConfiguration() != 0U)
      {
        return ErrorCode::ARG_ERR;
      }
      // 状态 IN 完成后的那一次写入在 InComplete 里，地址从 setup_ 取。
      // The write after the status IN is in InComplete, with the address from setup_.
      SetAddress(static_cast<uint8_t>(setup_.wValue), AddressStage::BEFORE_STATUS);
      StatusIn(in_isr);
      return ErrorCode::OK;

    case StandardRequest::GET_CONFIGURATION:
      if (!input || recipient != Recipient::DEVICE || setup_.wValue != 0U ||
          setup_.wIndex != 0U || setup_.wLength != 1U)
      {
        return ErrorCode::ARG_ERR;
      }
      answer[0] = composition_.GetConfiguration();
      StartIn(in_isr, {answer, 1U});
      return ErrorCode::OK;

    case StandardRequest::SET_CONFIGURATION:
    {
      if (input || recipient != Recipient::DEVICE || setup_.wIndex != 0U ||
          setup_.wLength != 0U || setup_.wValue > composition_.ConfigCount())
      {
        return ErrorCode::ARG_ERR;
      }
      const ErrorCode result = composition_.SetConfiguration(
          static_cast<uint8_t>(setup_.wValue), service_.GetSpeed(), in_isr);
      // 失败时组合层已回到未配置，闸门也跟着改。
      // On failure the composition is back to unconfigured; the gate follows.
      UpdateGate();
      if (result != ErrorCode::OK)
      {
        return result;
      }
      StatusIn(in_isr);
      return ErrorCode::OK;
    }

    case StandardRequest::GET_INTERFACE:
    case StandardRequest::SET_INTERFACE:
    {
      const bool get =
          setup_.bRequest == static_cast<uint8_t>(StandardRequest::GET_INTERFACE);
      if (recipient != Recipient::INTERFACE || input != get || setup_.wIndex > 255U ||
          (get ? (setup_.wValue != 0U || setup_.wLength != 1U)
               : (setup_.wValue > 255U || setup_.wLength != 0U)))
      {
        return ErrorCode::ARG_ERR;
      }
      const auto itf = static_cast<uint8_t>(setup_.wIndex);
      DeviceClass* item = composition_.FindClassByInterface(itf);
      if (item == nullptr)
      {
        return ErrorCode::NOT_FOUND;
      }
      if (!get)
      {
        const ErrorCode result =
            item->SetAltSetting(in_isr, itf, static_cast<uint8_t>(setup_.wValue));
        if (result != ErrorCode::OK)
        {
          return result;
        }
        StatusIn(in_isr);
        return ErrorCode::OK;
      }
      uint8_t alt = 0U;
      if (item->GetAltSetting(itf, alt) != ErrorCode::OK)
      {
        alt = 0U;
      }
      answer[0] = alt;
      StartIn(in_isr, {answer, 1U});
      return ErrorCode::OK;
    }

    case StandardRequest::GET_STATUS:
    {
      if (!input || setup_.wLength != 2U || setup_.wValue != 0U)
      {
        return ErrorCode::ARG_ERR;
      }
      uint16_t status = 0U;
      if (recipient == Recipient::DEVICE)
      {
        if (setup_.wIndex != 0U)
        {
          return ErrorCode::ARG_ERR;
        }
        status = composition_.DeviceStatus();
      }
      else if (recipient == Recipient::INTERFACE)
      {
        if (setup_.wIndex > 255U || composition_.FindClassByInterface(
                                        static_cast<uint8_t>(setup_.wIndex)) == nullptr)
        {
          return ErrorCode::NOT_FOUND;
        }
      }
      else if (recipient == Recipient::ENDPOINT)
      {
        Endpoint* ep = nullptr;
        if (!IsEndpointIndex(setup_.wIndex) ||
            pool_.FindEndpoint(static_cast<uint8_t>(setup_.wIndex), ep) != ErrorCode::OK)
        {
          return ErrorCode::NOT_FOUND;
        }
        status = ep->IsStalled() ? 1U : 0U;
      }
      else
      {
        return ErrorCode::ARG_ERR;
      }
      answer[0] = static_cast<uint8_t>(status);
      answer[1] = static_cast<uint8_t>(status >> 8U);
      StartIn(in_isr, {answer, 2U});
      return ErrorCode::OK;
    }

    case StandardRequest::SET_FEATURE:
    case StandardRequest::CLEAR_FEATURE:
    {
      // 只支持数据端点的 HALT；远程唤醒和测试模式一律拒绝。
      // Only the halt of a data endpoint is supported; remote wakeup and test mode are
      // refused.
      if (input || setup_.wLength != 0U || recipient != Recipient::ENDPOINT ||
          setup_.wValue != FEATURE_ENDPOINT_HALT || !IsEndpointIndex(setup_.wIndex) ||
          (setup_.wIndex & 0x0fU) == 0U)
      {
        return ErrorCode::NOT_SUPPORT;
      }
      const auto address = static_cast<uint8_t>(setup_.wIndex);
      Endpoint* ep = nullptr;
      if (pool_.FindEndpoint(address, ep) != ErrorCode::OK ||
          ep->GetType() == Endpoint::Type::ISOCHRONOUS)
      {
        return ErrorCode::NOT_SUPPORT;
      }
      const bool set =
          setup_.bRequest == static_cast<uint8_t>(StandardRequest::SET_FEATURE);
      const ErrorCode result = set ? ep->Stall(in_isr) : ep->ClearStall(in_isr);
      if (result != ErrorCode::OK)
      {
        return result;
      }
      if (!set)
      {
        if (DeviceClass* item = composition_.FindClassByEndpoint(address))
        {
          item->OnHaltCleared(in_isr, address);
        }
      }
      StatusIn(in_isr);
      return ErrorCode::OK;
    }

    case StandardRequest::SYNCH_FRAME:
    default:
      return ErrorCode::NOT_SUPPORT;
  }
}

ErrorCode DeviceCore::SendDescriptor(bool in_isr)
{
  const auto type = static_cast<DescriptorType>(setup_.wValue >> 8U);
  const auto index = static_cast<uint8_t>(setup_.wValue);
  const bool high_capable = max_speed_ == Speed::HIGH;
  ConstRawData data{nullptr, 0};
  switch (static_cast<uint8_t>(type))
  {
    case static_cast<uint8_t>(DescriptorType::DEVICE):
      if (index != 0U)
      {
        return ErrorCode::ARG_ERR;
      }
      data = device_desc_.GetData();
      break;

    case static_cast<uint8_t>(DescriptorType::CONFIGURATION):
    {
      RawData config{nullptr, 0};
      const ErrorCode result =
          composition_.WriteConfigDescriptor(index, service_.GetSpeed(), config);
      if (result != ErrorCode::OK)
      {
        return result;
      }
      data = config;
      break;
    }

    case static_cast<uint8_t>(DescriptorType::STRING):
    {
      const ErrorCode result =
          composition_.GetStringDescriptor(index, setup_.wIndex, data);
      if (result != ErrorCode::OK)
      {
        return result;
      }
      break;
    }

    case 6U:  // DEVICE_QUALIFIER
    {
      if (!high_capable || index != 0U)
      {
        return ErrorCode::NOT_SUPPORT;
      }
      const auto& d = device_desc_.data_;
      const auto spec = static_cast<uint16_t>(d.bcdUSB);
      auto* q = static_cast<uint8_t*>(control_buffer_.addr_);
      q[0] = 10U;
      q[1] = 6U;
      q[2] = static_cast<uint8_t>(spec);
      q[3] = static_cast<uint8_t>(spec >> 8U);
      q[4] = static_cast<uint8_t>(d.bDeviceClass);
      q[5] = d.bDeviceSubClass;
      q[6] = d.bDeviceProtocol;
      q[7] = static_cast<uint8_t>(d.bMaxPacketSize0);
      q[8] = composition_.ConfigCount();
      q[9] = 0U;
      data = {q, 10U};
      break;
    }

    case 7U:  // OTHER_SPEED_CONFIGURATION
    {
      if (!high_capable)
      {
        return ErrorCode::NOT_SUPPORT;
      }
      const Speed other =
          (service_.GetSpeed() == Speed::HIGH) ? Speed::FULL : Speed::HIGH;
      RawData config{nullptr, 0};
      const ErrorCode result = composition_.WriteConfigDescriptor(index, other, config);
      if (result != ErrorCode::OK)
      {
        return result;
      }
      static_cast<uint8_t*>(config.addr_)[1] = 7U;
      data = config;
      break;
    }

    case static_cast<uint8_t>(DescriptorType::BOS):
      if (index != 0U || device_desc_.GetUSBSpec() < USBSpec::USB_2_1)
      {
        return ErrorCode::NOT_SUPPORT;
      }
      data = composition_.Bos().GetBosDescriptor();
      break;

    default:
    {
      // 其他类型只可能是接口的类特定描述符。
      // Any other type can only be an interface's class-specific descriptor.
      if (RecipientOf(setup_) != Recipient::INTERFACE)
      {
        return ErrorCode::NOT_SUPPORT;
      }
      DeviceClass* item =
          composition_.FindClassByInterface(static_cast<uint8_t>(setup_.wIndex));
      if (item == nullptr)
      {
        return ErrorCode::NOT_FOUND;
      }
      const ErrorCode result = item->OnGetDescriptor(in_isr, setup_, data);
      if (result != ErrorCode::OK)
      {
        return result;
      }
      break;
    }
  }
  StartIn(in_isr, data);
  return ErrorCode::OK;
}

ErrorCode DeviceCore::HandleClassOrVendor(bool in_isr)
{
  const auto type = static_cast<RequestType>(setup_.bmRequestType & REQ_TYPE_MASK);
  const Recipient recipient = RecipientOf(setup_);
  if (type != RequestType::CLASS && type != RequestType::VENDOR)
  {
    return ErrorCode::NOT_SUPPORT;
  }

  if (recipient == Recipient::DEVICE)
  {
    if (type != RequestType::VENDOR)
    {
      return ErrorCode::NOT_SUPPORT;
    }
    BosVendorResult bos{};
    const ErrorCode result =
        composition_.Bos().ProcessVendorRequest(in_isr, &setup_, bos);
    if (result != ErrorCode::OK)
    {
      return result;
    }
    if (setup_.wLength == 0U)
    {
      StatusIn(in_isr);
      return ErrorCode::OK;
    }
    if (!IsIn(setup_))
    {
      return ErrorCode::ARG_ERR;
    }
    StartIn(in_isr, bos.in_data);
    return ErrorCode::OK;
  }

  DeviceClass* item = nullptr;
  if (recipient == Recipient::INTERFACE)
  {
    item = composition_.FindClassByInterface(static_cast<uint8_t>(setup_.wIndex));
  }
  else if (recipient == Recipient::ENDPOINT)
  {
    item = composition_.FindClassByEndpoint(static_cast<uint8_t>(setup_.wIndex));
  }
  if (item == nullptr)
  {
    return ErrorCode::NOT_FOUND;
  }
  ConstRawData in_data{nullptr, 0};
  const ErrorCode result = item->OnControlRequest(in_isr, setup_, in_data);
  if (result != ErrorCode::OK)
  {
    return result;
  }
  // 从这里起类接受了传输；之后的任何错误都经 StallTransfer 给它 OnControlAbort。
  // From here the class has accepted the transfer; any later error gives it
  // OnControlAbort through StallTransfer.
  control_class_ = item;
  if (setup_.wLength == 0U)
  {
    StatusIn(in_isr);
    return ErrorCode::OK;
  }
  if (IsIn(setup_))
  {
    if (in_data.size_ != 0U && in_data.addr_ == nullptr)
    {
      return ErrorCode::ARG_ERR;
    }
    StartIn(in_isr, in_data);
    return ErrorCode::OK;
  }
  if (setup_.wLength > control_buffer_.size_)
  {
    return ErrorCode::ARG_ERR;
  }
  transferred_ = 0U;
  ReceiveNextOut(in_isr);
  return ErrorCode::OK;
}

void DeviceCore::StartIn(bool in_isr, ConstRawData data)
{
  const size_t length = (data.size_ < setup_.wLength) ? data.size_ : setup_.wLength;
  in_data_ = {data.addr_, length};
  transferred_ = 0U;
  phase_ = Phase::DATA_IN;
  // 主机可以在任何一个 IN 包之后进入状态阶段，所以状态 OUT 先于第一包挂好。
  // The host may enter the status stage after any IN packet, so the status OUT is
  // armed before the first packet.
  if (pool_.GetEndpoint0Out()->Receive(0U, in_isr) != ErrorCode::OK)
  {
    StallTransfer(in_isr);
    return;
  }
  SendNextIn(in_isr);
}

void DeviceCore::SendNextIn(bool in_isr)
{
  Endpoint* in0 = pool_.GetEndpoint0In();
  const size_t remaining = in_data_.size_ - transferred_;
  const size_t packet = in0->MaxPacketSize();
  const size_t amount = (remaining < packet) ? remaining : packet;
  Endpoint::Slot slot;
  if (in0->AcquireSlot(slot) != ErrorCode::OK)
  {
    StallTransfer(in_isr);
    return;
  }
  if (amount != 0U)
  {
    LibXR::Memory::FastCopy(slot.data.addr_,
                            static_cast<const uint8_t*>(in_data_.addr_) + transferred_,
                            amount);
  }
  // 比 wLength 短的应答以短包结束；最后一包正好满包时由 Submit 补结尾零长度包。
  // A reply shorter than wLength must end with a short packet; when the last packet is
  // full, Submit adds the closing zero-length packet.
  const bool last = transferred_ + amount == in_data_.size_;
  const bool zlp = last && in_data_.size_ < setup_.wLength;
  if (in0->Submit(slot, amount, zlp, in_isr) != ErrorCode::OK)
  {
    (void)in0->ReleaseSlot(slot);
    StallTransfer(in_isr);
    return;
  }
  transferred_ += amount;
}

void DeviceCore::ReceiveNextOut(bool in_isr)
{
  phase_ = Phase::DATA_OUT;
  Endpoint* out0 = pool_.GetEndpoint0Out();
  if (out0->Receive(out0->MaxPacketSize(), in_isr) != ErrorCode::OK)
  {
    StallTransfer(in_isr);
  }
}

void DeviceCore::StatusIn(bool in_isr)
{
  phase_ = Phase::STATUS_IN;
  Endpoint* in0 = pool_.GetEndpoint0In();
  Endpoint::Slot slot;
  if (in0->AcquireSlot(slot) != ErrorCode::OK)
  {
    StallTransfer(in_isr);
    return;
  }
  if (in0->Submit(slot, 0U, false, in_isr) != ErrorCode::OK)
  {
    (void)in0->ReleaseSlot(slot);
    StallTransfer(in_isr);
  }
}

void DeviceCore::Finish(bool in_isr)
{
  DeviceClass* item = control_class_;
  control_class_ = nullptr;
  phase_ = Phase::IDLE;
  in_data_ = {nullptr, 0};
  transferred_ = 0U;
  if (item != nullptr)
  {
    item->OnControlComplete(in_isr, setup_);
  }
  OnControlReady();
}

void DeviceCore::StallTransfer(bool in_isr)
{
  AbortTransfer(in_isr);
  (void)pool_.GetEndpoint0In()->Stall(in_isr);
  (void)pool_.GetEndpoint0Out()->Stall(in_isr);
}

void DeviceCore::AbortTransfer(bool in_isr)
{
  DeviceClass* item = control_class_;
  control_class_ = nullptr;
  phase_ = Phase::IDLE;
  in_data_ = {nullptr, 0};
  transferred_ = 0U;
  if (item != nullptr)
  {
    item->OnControlAbort(in_isr, setup_);
  }
}

void DeviceCore::InComplete(bool in_isr, const Endpoint::Completion& completion)
{
  if (completion.result != ErrorCode::OK)
  {
    return;
  }
  if (phase_ == Phase::DATA_IN)
  {
    if (transferred_ < in_data_.size_)
    {
      SendNextIn(in_isr);
    }
    else
    {
      phase_ = Phase::STATUS_OUT;
    }
    return;
  }
  if (phase_ != Phase::STATUS_IN)
  {
    return;
  }
  if (setup_.bmRequestType == 0U &&
      setup_.bRequest == static_cast<uint8_t>(StandardRequest::SET_ADDRESS))
  {
    SetAddress(static_cast<uint8_t>(setup_.wValue), AddressStage::STATUS_COMPLETE);
  }
  Finish(in_isr);
}

void DeviceCore::OutComplete(bool in_isr, const Endpoint::Completion& completion)
{
  Endpoint* out0 = pool_.GetEndpoint0Out();
  const bool ok = completion.result == ErrorCode::OK;
  // 先拷走要的数据，再交回槽，然后才据此动作（动作里可能再挂接收）。
  // Copy what is needed, hand the slot back, and only then act (acting may arm the
  // next receive).
  const bool fits = transferred_ + completion.actual <= setup_.wLength;
  if (ok && phase_ == Phase::DATA_OUT && fits && completion.actual != 0U)
  {
    LibXR::Memory::FastCopy(static_cast<uint8_t*>(control_buffer_.addr_) + transferred_,
                            completion.slot.data.addr_, completion.actual);
  }
  if (completion.slot.Valid())
  {
    (void)out0->ReleaseSlot(completion.slot);
  }
  if (!ok)
  {
    return;
  }

  if (phase_ == Phase::DATA_OUT)
  {
    if (!fits)
    {
      StallTransfer(in_isr);
      return;
    }
    transferred_ += completion.actual;
    if (transferred_ < setup_.wLength && completion.actual == out0->MaxPacketSize())
    {
      ReceiveNextOut(in_isr);
      return;
    }
    ASSERT(control_class_ != nullptr);
    if (control_class_->OnControlData(
            in_isr, setup_, {control_buffer_.addr_, transferred_}) != ErrorCode::OK)
    {
      StallTransfer(in_isr);
      return;
    }
    StatusIn(in_isr);
    return;
  }

  if (phase_ == Phase::DATA_IN || phase_ == Phase::STATUS_OUT)
  {
    // 状态 OUT 到了：传输结束。IN 数据没发完也不再发（主机已进入状态阶段）。阶段先改，
    // Flush 交回的 IN 完成就会被忽略。
    // The status OUT arrived: the transfer is over. IN data not yet sent is not sent
    // any more (the host is in the status stage). The phase changes first, so the IN
    // completion the flush hands back is ignored.
    phase_ = Phase::STATUS_OUT;
    pool_.GetEndpoint0In()->Flush(in_isr);
    Finish(in_isr);
  }
}
