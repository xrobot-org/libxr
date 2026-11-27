#include "hid.hpp"

#include <cstddef>

#include "libxr_mem.hpp"

using namespace LibXR::USB;
using LibXR::ConstRawData;
using LibXR::ErrorCode;
using LibXR::USB::DescriptorType;
using LibXR::USB::Recipient;
using LibXR::USB::REQ_DIRECTION_MASK;
using LibXR::USB::REQ_RECIPIENT_MASK;
using LibXR::USB::REQ_TYPE_MASK;
using LibXR::USB::RequestType;
using LibXR::USB::SetupPacket;
using LibXR::USB::Speed;

namespace
{
/// HID 接口类 / HID interface class
constexpr uint8_t CLASS_HID = 0x03U;
/// 启动接口子类（HID 1.11 4.2）/ Boot interface subclass (HID 1.11 4.2)
constexpr uint8_t SUBCLASS_BOOT = 0x01U;
/// HID 1.11 / HID 1.11
constexpr uint16_t BCD_HID = 0x0111U;
}  // namespace

HID::HID(Endpoint::EPNumber in_ep_num, Endpoint::EPNumber out_ep_num,
         size_t in_report_size, size_t out_report_size, uint8_t in_interval,
         uint8_t out_interval, BootProtocol boot, const char* interface_string)
    : in_num_(in_ep_num),
      out_num_(out_ep_num),
      in_report_size_(in_report_size),
      out_report_size_(out_report_size),
      in_interval_(in_interval),
      out_interval_(out_interval),
      boot_(boot),
      interface_string_(interface_string),
      out_cb_(
          Callback<const Endpoint::Completion&>::Create(OnDataOutCompleteStatic, this)),
      in_cb_(Callback<const Endpoint::Completion&>::Create(OnDataInCompleteStatic, this))
{
  ASSERT(in_report_size >= 1U && in_report_size <= MAX_PACKET);
  ASSERT(!HasOut() || (out_report_size >= 1U && out_report_size <= MAX_PACKET));
}

ErrorCode HID::SendInputReport(ConstRawData report, bool in_isr)
{
  if (report.addr_ == nullptr || report.size_ == 0U || report.size_ > in_report_size_)
  {
    return ErrorCode::ARG_ERR;
  }
  Endpoint* ep = sender_ep_.load(std::memory_order_acquire);
  if (ep == nullptr)
  {
    return ErrorCode::FAILED;
  }
  Endpoint::Slot slot;
  const ErrorCode acquired = ep->AcquireSlot(slot);
  if (acquired != ErrorCode::OK)
  {
    // BUSY：槽都在用；STATE_ERR：端点刚被关闭。
    // BUSY: every slot in use; STATE_ERR: the endpoint has just been closed.
    return acquired == ErrorCode::BUSY ? ErrorCode::BUSY : ErrorCode::FAILED;
  }
  // 借到槽之后再看一次：读指针之后本类可能已解绑，端点可能已交给别的类配置，这个槽就属于
  // 别的类。指针仍是这个端点时，槽要么属于本类当前的绑定，要么在之后的 Close 中作废，
  // 作废的槽 Submit 返回 ABORTED。
  // Look again after the slot is borrowed: since the pointer was read this class may
  // have been unbound and the endpoint configured for another class, so the slot would
  // be that class's. While the pointer is still this endpoint, the slot belongs to this
  // class's current binding or is made stale by a later Close, and Submit of a stale
  // slot returns ABORTED.
  if (sender_ep_.load(std::memory_order_acquire) != ep)
  {
    (void)ep->ReleaseSlot(slot);
    return ErrorCode::FAILED;
  }
  if (report.size_ > slot.data.size_)
  {
    (void)ep->ReleaseSlot(slot);
    return ErrorCode::FAILED;
  }
  LibXR::Memory::FastCopy(slot.data.addr_, report.addr_, report.size_);
  // 主机按报告描述符知道报告长度，不需要结尾零长度包；ABORTED 时槽已空闲。
  // The host knows the report length from the report descriptor, so no closing packet;
  // on ABORTED the slot is free.
  const ErrorCode result = ep->Submit(slot, report.size_, false, in_isr);
  ASSERT(result == ErrorCode::OK || result == ErrorCode::ABORTED);
  return result == ErrorCode::OK ? ErrorCode::OK : ErrorCode::FAILED;
}

uint8_t HID::GetIdleRate() const
{
  return static_cast<uint8_t>(idle_rate_.load(std::memory_order_relaxed));
}

size_t HID::GetMaxConfigSize() const
{
  return HasOut() ? sizeof(DescriptorBlock) : offsetof(DescriptorBlock, ep_out);
}

const char* HID::GetInterfaceString(size_t local_interface_index) const
{
  return local_interface_index == 0U ? interface_string_ : nullptr;
}

size_t HID::GetControlReceiveCapacity() const
{
  size_t capacity = 64U;
  capacity = in_report_size_ > capacity ? in_report_size_ : capacity;
  capacity = out_report_size_ > capacity ? out_report_size_ : capacity;
  return capacity;
}

Endpoint::Config HID::InConfig() const
{
  return {Endpoint::Direction::IN, Endpoint::Type::INTR,
          static_cast<uint16_t>(in_report_size_)};
}

Endpoint::Config HID::OutConfig() const
{
  return {Endpoint::Direction::OUT, Endpoint::Type::INTR,
          static_cast<uint16_t>(out_report_size_)};
}

ErrorCode HID::WriteDescriptors(const EndpointPool& pool, Speed speed,
                                uint8_t first_interface, ConstRawData& block)
{
  DescriptorBlock& d = desc_block_;
  bool ok =
      WriteEndpointDescriptor(pool, speed, in_num_, InConfig(), in_interval_, d.ep_in);
  if (HasOut())
  {
    ok = WriteEndpointDescriptor(pool, speed, out_num_, OutConfig(), out_interval_,
                                 d.ep_out) &&
         ok;
  }
  if (!ok)
  {
    return ErrorCode::NOT_SUPPORT;
  }

  const bool boot = boot_ != BootProtocol::NONE;
  d.intf = {9U,
            static_cast<uint8_t>(DescriptorType::INTERFACE),
            first_interface,
            0U,
            static_cast<uint8_t>(HasOut() ? 2U : 1U),
            CLASS_HID,
            boot ? SUBCLASS_BOOT : static_cast<uint8_t>(0U),
            static_cast<uint8_t>(boot_),
            GetInterfaceStringIndex(0U)};
  d.hid = {9U,
           static_cast<uint8_t>(HidDescriptorType::HID),
           BCD_HID,
           0U,
           1U,
           static_cast<uint8_t>(HidDescriptorType::REPORT),
           static_cast<uint16_t>(GetReportDesc().size_)};
  block = {&desc_block_, GetMaxConfigSize()};
  return ErrorCode::OK;
}

ErrorCode HID::BindOne(EndpointPool& pool, Endpoint::EPNumber number,
                       const Endpoint::Config& cfg, uint16_t descriptor_field,
                       size_t report_size, bool in_isr, Endpoint*& ep)
{
  const ErrorCode result = BindEndpoint(pool, number, cfg, descriptor_field, in_isr, ep);
  if (result != ErrorCode::OK)
  {
    return result;
  }
  // 最长报告必须装进一次请求（向上取整到包长）。
  // The longest report must fit one request (rounded up to the packet size).
  const size_t packet = ep->MaxPacketSize();
  const size_t needed = (report_size + packet - 1U) / packet * packet;
  return ep->MaxTransferSize() >= needed ? ErrorCode::OK : ErrorCode::NO_BUFF;
}

ErrorCode HID::BindEndpoints(EndpointPool& pool, uint8_t first_interface, bool in_isr)
{
  UNUSED(first_interface);
  ErrorCode result = BindOne(pool, in_num_, InConfig(), desc_block_.ep_in.wMaxPacketSize,
                             in_report_size_, in_isr, in_ep_);
  if (result == ErrorCode::OK && HasOut())
  {
    result = BindOne(pool, out_num_, OutConfig(), desc_block_.ep_out.wMaxPacketSize,
                     out_report_size_, in_isr, out_ep_);
  }
  if (result != ErrorCode::OK)
  {
    return result;
  }
  protocol_ = 1U;
  idle_rate_.store(0U, std::memory_order_relaxed);
  in_ep_->SetOnComplete(in_cb_);
  sender_ep_.store(in_ep_, std::memory_order_release);
  if (out_ep_ == nullptr)
  {
    return ErrorCode::OK;
  }
  out_ep_->SetOnComplete(out_cb_);
  return out_ep_->Receive(OutReceiveSize(), in_isr);
}

void HID::UnbindEndpoints(EndpointPool& pool, bool in_isr)
{
  // 撤下发送者用的指针必须在本函数返回前、别的类配置这个端点之前：此后借到槽的发送者在
  // 复查中失败。放在 Close 之前或之后都可以：Close 之后借槽得 STATE_ERR，之前借到的槽被
  // Close 作废。
  // The senders' pointer must be withdrawn before this returns, i.e. before another
  // class configures the endpoint: a sender that borrows a slot after that fails the
  // check. Before or after Close both work: a lend after Close gets STATE_ERR, and a
  // slot lent before is made stale by Close.
  sender_ep_.store(nullptr, std::memory_order_release);
  UnbindEndpoint(pool, in_ep_, in_isr);
  UnbindEndpoint(pool, out_ep_, in_isr);
}

ErrorCode HID::WriteDeviceDescriptor(DeviceDescriptor& descriptor)
{
  descriptor.data_.bDeviceClass = DeviceDescriptor::ClassID::PER_INTERFACE;
  descriptor.data_.bDeviceSubClass = 0U;
  descriptor.data_.bDeviceProtocol = 0U;
  return ErrorCode::OK;
}

bool HID::OwnsEndpoint(uint8_t ep_addr) const
{
  return ep_addr == Endpoint::EPNumberToAddr(in_num_, Endpoint::Direction::IN) ||
         (HasOut() &&
          ep_addr == Endpoint::EPNumberToAddr(out_num_, Endpoint::Direction::OUT));
}

ErrorCode HID::OnGetDescriptor(bool in_isr, const SetupPacket& setup, ConstRawData& data)
{
  UNUSED(in_isr);
  const auto type = static_cast<uint8_t>(setup.wValue >> 8U);
  const auto index = static_cast<uint8_t>(setup.wValue);
  if (index != 0U)
  {
    return ErrorCode::NOT_SUPPORT;
  }
  if (type == static_cast<uint8_t>(HidDescriptorType::HID))
  {
    data = {&desc_block_.hid, sizeof(desc_block_.hid)};
    return ErrorCode::OK;
  }
  if (type == static_cast<uint8_t>(HidDescriptorType::REPORT))
  {
    data = GetReportDesc();
    return ErrorCode::OK;
  }
  return ErrorCode::NOT_SUPPORT;
}

ErrorCode HID::OnControlRequest(bool in_isr, const SetupPacket& setup,
                                ConstRawData& in_data)
{
  const auto type = static_cast<RequestType>(setup.bmRequestType & REQ_TYPE_MASK);
  const auto recipient = static_cast<Recipient>(setup.bmRequestType & REQ_RECIPIENT_MASK);
  // HID 只有一个接口，按接口号找到本类就说明 wIndex 是它。
  // HID has a single interface, so being found by interface number means wIndex is it.
  if (type != RequestType::CLASS || recipient != Recipient::INTERFACE)
  {
    return ErrorCode::NOT_SUPPORT;
  }
  const bool input = (setup.bmRequestType & REQ_DIRECTION_MASK) != 0U;
  const auto report_type = static_cast<uint8_t>(setup.wValue >> 8U);
  const auto report_id = static_cast<uint8_t>(setup.wValue);
  const bool legal_type = report_type >= static_cast<uint8_t>(ReportType::INPUT) &&
                          report_type <= static_cast<uint8_t>(ReportType::FEATURE);
  switch (static_cast<ClassRequest>(setup.bRequest))
  {
    case ClassRequest::GET_REPORT:
      if (!input || setup.wLength == 0U || !legal_type)
      {
        return ErrorCode::ARG_ERR;
      }
      return OnGetReport(in_isr, static_cast<ReportType>(report_type), report_id,
                         in_data);

    case ClassRequest::SET_REPORT:
      // 核心收下数据阶段后调用 OnControlData。
      // The core receives the data stage, then calls OnControlData.
      return (!input && setup.wLength != 0U && legal_type) ? ErrorCode::OK
                                                           : ErrorCode::ARG_ERR;

    case ClassRequest::GET_IDLE:
      // 只支持对全部报告的一个空闲速率（报告号 0）。
      // Only one idle rate for all reports (report ID 0) is supported.
      if (!input || setup.wLength != 1U || report_id != 0U)
      {
        return ErrorCode::ARG_ERR;
      }
      idle_reply_ = static_cast<uint8_t>(idle_rate_.load(std::memory_order_relaxed));
      in_data = {&idle_reply_, 1U};
      return ErrorCode::OK;

    case ClassRequest::SET_IDLE:
      if (input || setup.wLength != 0U || report_id != 0U)
      {
        return ErrorCode::ARG_ERR;
      }
      idle_rate_.store(static_cast<uint8_t>(setup.wValue >> 8U),
                       std::memory_order_relaxed);
      return ErrorCode::OK;

    case ClassRequest::GET_PROTOCOL:
      if (!input || setup.wLength != 1U || setup.wValue != 0U)
      {
        return ErrorCode::ARG_ERR;
      }
      // protocol_ 只被 SET_PROTOCOL 改，新的 SETUP 先结束本次传输，所以直接指向它。
      // protocol_ changes only by SET_PROTOCOL, and a new SETUP ends this transfer first,
      // so the reply points at it directly.
      in_data = {&protocol_, 1U};
      return ErrorCode::OK;

    case ClassRequest::SET_PROTOCOL:
      if (input || setup.wLength != 0U || setup.wValue > 1U)
      {
        return ErrorCode::ARG_ERR;
      }
      protocol_ = static_cast<uint8_t>(setup.wValue);
      return ErrorCode::OK;

    default:
      return ErrorCode::NOT_SUPPORT;
  }
}

ErrorCode HID::OnControlData(bool in_isr, const SetupPacket& setup, ConstRawData data)
{
  // 只有 SET_REPORT 有 OUT 数据阶段。
  // Only SET_REPORT has an OUT data stage.
  if (setup.bRequest != static_cast<uint8_t>(ClassRequest::SET_REPORT))
  {
    return ErrorCode::NOT_SUPPORT;
  }
  return OnSetReport(in_isr, static_cast<ReportType>(setup.wValue >> 8U),
                     static_cast<uint8_t>(setup.wValue), data);
}

ErrorCode HID::OnGetReport(bool in_isr, ReportType type, uint8_t report_id,
                           ConstRawData& data)
{
  UNUSED(in_isr);
  UNUSED(type);
  UNUSED(report_id);
  data = {nullptr, 0U};
  return ErrorCode::OK;
}

ErrorCode HID::OnSetReport(bool in_isr, ReportType type, uint8_t report_id,
                           ConstRawData data)
{
  UNUSED(report_id);
  return type == ReportType::OUTPUT ? OnOutReport(in_isr, data) : ErrorCode::NOT_SUPPORT;
}

ErrorCode HID::OnOutReport(bool in_isr, ConstRawData report)
{
  UNUSED(in_isr);
  UNUSED(report);
  return ErrorCode::NOT_SUPPORT;
}

void HID::OnDataOutComplete(bool in_isr, const Endpoint::Completion& completion)
{
  if (completion.result == ErrorCode::OK)
  {
    (void)OnOutReport(in_isr, completion.slot.data);
    (void)out_ep_->ReleaseSlot(completion.slot);
  }
  // 不论结果都重挂：STALLED 时排队到 ClearStall，因此不需要 OnHaltCleared。
  // Re-arm whatever the result: while STALLED it waits for ClearStall, so
  // OnHaltCleared is not needed.
  (void)out_ep_->Receive(OutReceiveSize(), in_isr);
}

void HID::OnDataInComplete(bool in_isr, const Endpoint::Completion& completion)
{
  UNUSED(in_isr);
  UNUSED(completion);
}

size_t HID::OutReceiveSize() const
{
  ASSERT(out_ep_ != nullptr);
  const size_t packet = out_ep_->MaxPacketSize();
  return (out_report_size_ + packet - 1U) / packet * packet;
}

void HID::OnDataOutCompleteStatic(bool in_isr, HID* self,
                                  const Endpoint::Completion& completion)
{
  if (self->out_ep_ == nullptr)
  {
    // 未绑定时 OUT 只会以 ABORTED 结束，不借出槽（见 Endpoint::Close）。
    // While unbound an OUT only ends with ABORTED and lends no slot (see
    // Endpoint::Close).
    ASSERT(!completion.slot.Valid());
    return;
  }
  self->OnDataOutComplete(in_isr, completion);
}

void HID::OnDataInCompleteStatic(bool in_isr, HID* self,
                                 const Endpoint::Completion& completion)
{
  if (self->in_ep_ != nullptr)
  {
    self->OnDataInComplete(in_isr, completion);
  }
}
