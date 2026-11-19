#include "cdc_base.hpp"

#include <cstring>

using namespace LibXR::USB;
using LibXR::ConstRawData;
using LibXR::ErrorCode;
using LibXR::UART;
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
/// CDC 类代码 / CDC class codes
constexpr uint8_t CLASS_COMM = 0x02U;
constexpr uint8_t CLASS_DATA = 0x0AU;
/// ACM 子类 / ACM subclass
constexpr uint8_t SUBCLASS_ACM = 0x02U;
/// SERIAL_STATE 通知码 / SERIAL_STATE notification code
constexpr uint8_t NOTIFICATION_SERIAL_STATE = 0x20U;
/// 通知端点的轮询间隔：全速 4 ms，高速 2^(4-1) 微帧 = 1 ms / Polling interval of the
/// notification endpoint: 4 ms at full speed, 2^(4-1) microframes = 1 ms at high speed
constexpr uint8_t COMM_INTERVAL = 0x04U;
}  // namespace

CDCBase::CDCBase(Endpoint::EPNumber data_in_ep_num, Endpoint::EPNumber data_out_ep_num,
                 Endpoint::EPNumber comm_ep_num, const char* control_interface_string,
                 const char* data_interface_string)
    : data_in_num_(data_in_ep_num),
      data_out_num_(data_out_ep_num),
      comm_num_(comm_ep_num),
      control_interface_string_(control_interface_string),
      data_interface_string_(data_interface_string),
      data_out_cb_(
          Callback<const Endpoint::Completion&>::Create(OnDataOutCompleteStatic, this)),
      data_in_cb_(
          Callback<const Endpoint::Completion&>::Create(OnDataInCompleteStatic, this)),
      comm_cb_(Callback<const Endpoint::Completion&>::Create(OnCommCompleteStatic, this))
{
}

bool CDCBase::IsDtrSet() const
{
  return (control_line_state_.load(std::memory_order_acquire) & CONTROL_LINE_DTR) != 0U;
}

bool CDCBase::IsRtsSet() const
{
  return (control_line_state_.load(std::memory_order_acquire) & CONTROL_LINE_RTS) != 0U;
}

ErrorCode CDCBase::SendSerialState(bool in_isr)
{
  serial_state_pending_.store(1U, std::memory_order_release);
  RequestService(in_isr);
  return ErrorCode::OK;
}

void CDCBase::SetOnSetControlLineStateCallback(Callback<bool, bool> cb)
{
  on_set_control_line_state_ = cb;
}

void CDCBase::SetOnSetLineCodingCallback(Callback<UART::Configuration> cb)
{
  on_set_line_coding_ = cb;
}

const char* CDCBase::GetInterfaceString(size_t local_interface_index) const
{
  switch (local_interface_index)
  {
    case 0:
      return control_interface_string_;
    case 1:
      return data_interface_string_;
    default:
      return nullptr;
  }
}

ErrorCode CDCBase::WriteDescriptors(const EndpointPool& pool, Speed speed,
                                    uint8_t first_interface, ConstRawData& block)
{
  DescriptorBlock& d = desc_block_;
  bool ok = WriteEndpointDescriptor(pool, speed, data_in_num_, DATA_IN_CONFIG, 0U,
                                    d.data_in_ep);
  ok = WriteEndpointDescriptor(pool, speed, data_out_num_, DATA_OUT_CONFIG, 0U,
                               d.data_out_ep) &&
       ok;
  ok = WriteEndpointDescriptor(pool, speed, comm_num_, COMM_CONFIG, COMM_INTERVAL,
                               d.comm_ep) &&
       ok;
  if (!ok)
  {
    return ErrorCode::NOT_SUPPORT;
  }

  const auto comm_itf = first_interface;
  const auto data_itf = static_cast<uint8_t>(first_interface + 1U);
  const auto cs_interface = static_cast<uint8_t>(DescriptorType::CS_INTERFACE);
  const auto interface = static_cast<uint8_t>(DescriptorType::INTERFACE);
  d.iad = {8U,         static_cast<uint8_t>(DescriptorType::IAD),
           comm_itf,   2U,
           CLASS_COMM, SUBCLASS_ACM,
           0U,         0U};
  d.comm_intf = {9U,           interface, comm_itf,
                 0U,           1U,        CLASS_COMM,
                 SUBCLASS_ACM, 0U,        GetInterfaceStringIndex(0U)};
  d.header = {5U, cs_interface, static_cast<uint8_t>(DescriptorSubtype::HEADER), 0x0110U};
  // 不处理呼叫管理 / No call management
  d.call_management = {5U, cs_interface,
                       static_cast<uint8_t>(DescriptorSubtype::CALL_MANAGEMENT), 0x00U,
                       data_itf};
  // 支持 SET/GET_LINE_CODING 和 SET_CONTROL_LINE_STATE / Supports SET/GET_LINE_CODING
  // and SET_CONTROL_LINE_STATE
  d.acm = {4U, cs_interface, static_cast<uint8_t>(DescriptorSubtype::ACM), 0x02U};
  d.union_desc = {5U, cs_interface, static_cast<uint8_t>(DescriptorSubtype::UNION),
                  comm_itf, data_itf};
  d.data_intf = {
      9U, interface, data_itf, 0U, 2U, CLASS_DATA, 0U, 0U, GetInterfaceStringIndex(1U)};
  block = {&desc_block_, sizeof(desc_block_)};
  return ErrorCode::OK;
}

ErrorCode CDCBase::BindEndpoints(EndpointPool& pool, uint8_t first_interface, bool in_isr)
{
  ErrorCode result =
      BindEndpoint(pool, data_in_num_, DATA_IN_CONFIG,
                   desc_block_.data_in_ep.wMaxPacketSize, in_isr, data_in_);
  if (result == ErrorCode::OK)
  {
    result = BindEndpoint(pool, data_out_num_, DATA_OUT_CONFIG,
                          desc_block_.data_out_ep.wMaxPacketSize, in_isr, data_out_);
  }
  if (result == ErrorCode::OK)
  {
    result = BindEndpoint(pool, comm_num_, COMM_CONFIG,
                          desc_block_.comm_ep.wMaxPacketSize, in_isr, comm_in_);
  }
  if (result != ErrorCode::OK)
  {
    return result;
  }
  comm_interface_ = first_interface;
  control_line_state_.store(0U, std::memory_order_release);
  data_out_->SetOnComplete(data_out_cb_);
  data_in_->SetOnComplete(data_in_cb_);
  comm_in_->SetOnComplete(comm_cb_);
  return ErrorCode::OK;
}

void CDCBase::UnbindEndpoints(EndpointPool& pool, bool in_isr)
{
  control_line_state_.store(0U, std::memory_order_release);
  UnbindEndpoint(pool, data_in_, in_isr);
  UnbindEndpoint(pool, data_out_, in_isr);
  UnbindEndpoint(pool, comm_in_, in_isr);
}

bool CDCBase::OwnsEndpoint(uint8_t ep_addr) const
{
  return ep_addr == Endpoint::EPNumberToAddr(data_in_num_, Endpoint::Direction::IN) ||
         ep_addr == Endpoint::EPNumberToAddr(data_out_num_, Endpoint::Direction::OUT) ||
         ep_addr == Endpoint::EPNumberToAddr(comm_num_, Endpoint::Direction::IN);
}

ErrorCode CDCBase::OnControlRequest(bool in_isr, const SetupPacket& setup,
                                    ConstRawData& in_data)
{
  const auto type = static_cast<RequestType>(setup.bmRequestType & REQ_TYPE_MASK);
  const auto recipient = static_cast<Recipient>(setup.bmRequestType & REQ_RECIPIENT_MASK);
  if (type != RequestType::CLASS || recipient != Recipient::INTERFACE ||
      setup.wIndex != comm_interface_)
  {
    return ErrorCode::NOT_SUPPORT;
  }
  const auto request = static_cast<ClassRequest>(setup.bRequest);
  const bool input = (setup.bmRequestType & REQ_DIRECTION_MASK) != 0U;
  switch (request)
  {
    case ClassRequest::SET_LINE_CODING:
      // 核心收下 7 字节的数据阶段后调用 OnControlData。
      // The core receives the 7-byte data stage, then calls OnControlData.
      return (!input && setup.wLength == sizeof(CDCLineCoding)) ? ErrorCode::OK
                                                                : ErrorCode::ARG_ERR;

    case ClassRequest::GET_LINE_CODING:
      if (!input || setup.wLength != sizeof(CDCLineCoding))
      {
        return ErrorCode::ARG_ERR;
      }
      line_coding_reply_ = line_coding_;
      in_data = {&line_coding_reply_, sizeof(line_coding_reply_)};
      return ErrorCode::OK;

    case ClassRequest::SET_CONTROL_LINE_STATE:
      if (input || setup.wLength != 0U)
      {
        return ErrorCode::ARG_ERR;
      }
      control_line_state_.store(setup.wValue, std::memory_order_release);
      (void)SendSerialState(in_isr);
      on_set_control_line_state_.Run(in_isr, IsDtrSet(), IsRtsSet());
      return ErrorCode::OK;

    case ClassRequest::SEND_BREAK:
      return (!input && setup.wLength == 0U) ? ErrorCode::OK : ErrorCode::ARG_ERR;

    default:
      return ErrorCode::NOT_SUPPORT;
  }
}

ErrorCode CDCBase::OnControlData(bool in_isr, const SetupPacket& setup, ConstRawData data)
{
  // 只有 SET_LINE_CODING 有 OUT 数据阶段。
  // Only SET_LINE_CODING has an OUT data stage.
  if (setup.bRequest != static_cast<uint8_t>(ClassRequest::SET_LINE_CODING))
  {
    return ErrorCode::NOT_SUPPORT;
  }
  if (data.size_ != sizeof(CDCLineCoding))
  {
    return ErrorCode::ARG_ERR;
  }
  CDCLineCoding next{};
  std::memcpy(&next, data.addr_, sizeof(next));
  UART::Configuration cfg = ToUartConfiguration(next);
  ApplyLineCoding(in_isr, next, cfg);
  line_coding_ = next;
  on_set_line_coding_.Run(in_isr, cfg);
  return ErrorCode::OK;
}

void CDCBase::OnService(bool in_isr) { TrySendSerialState(in_isr); }

void CDCBase::ApplyLineCoding(bool in_isr, CDCLineCoding& coding,
                              UART::Configuration& config)
{
  UNUSED(in_isr);
  UNUSED(coding);
  UNUSED(config);
}

UART::Configuration CDCBase::ToUartConfiguration(const CDCLineCoding& coding)
{
  UART::Configuration cfg{};
  cfg.baudrate = coding.dwDTERate;
  cfg.stop_bits = coding.bCharFormat == 2U ? 2U : 1U;
  cfg.parity = coding.bParityType == 1U   ? UART::Parity::ODD
               : coding.bParityType == 2U ? UART::Parity::EVEN
                                          : UART::Parity::NO_PARITY;
  cfg.data_bits = coding.bDataBits;
  return cfg;
}

CDCBase::CDCLineCoding CDCBase::ToLineCoding(const UART::Configuration& cfg)
{
  CDCLineCoding coding{};
  coding.dwDTERate = cfg.baudrate;
  coding.bCharFormat = cfg.stop_bits == 2U ? 2U : 0U;
  coding.bParityType = cfg.parity == UART::Parity::ODD    ? 1U
                       : cfg.parity == UART::Parity::EVEN ? 2U
                                                          : 0U;
  coding.bDataBits = cfg.data_bits;
  return coding;
}

void CDCBase::TrySendSerialState(bool in_isr)
{
  if (comm_in_ == nullptr || serial_state_pending_.load(std::memory_order_acquire) == 0U)
  {
    return;
  }
  Endpoint::Slot slot;
  if (comm_in_->AcquireSlot(slot) != ErrorCode::OK)
  {
    // 没有空闲槽：通知端点的完成回调会再试。
    // No free slot: the notification endpoint's completion tries again.
    return;
  }
  // 先清再读状态：之后的请求会再置位，不会丢。
  // Clear before reading the state: a later request sets it again and is not lost.
  serial_state_pending_.store(0U, std::memory_order_release);
  SerialStateNotification message{};
  message.bmRequestType = 0xA1U;
  message.bNotification = NOTIFICATION_SERIAL_STATE;
  message.wValue = 0U;
  message.wIndex = comm_interface_;
  message.wLength = 2U;
  // bRxCarrier 和 bTxCarrier 随 DTR / bRxCarrier and bTxCarrier follow DTR
  message.serialState = IsDtrSet() ? 0x03U : 0U;
  ASSERT(slot.data.size_ >= sizeof(message));
  std::memcpy(slot.data.addr_, &message, sizeof(message));
  // 10 字节不是 16 的倍数，不需要结尾零长度包；ABORTED 时槽已空闲。
  // 10 bytes is no multiple of 16, so no closing packet; on ABORTED the slot is free.
  const ErrorCode result = comm_in_->Submit(slot, sizeof(message), false, in_isr);
  ASSERT(result == ErrorCode::OK || result == ErrorCode::ABORTED);
  UNUSED(result);
}

void CDCBase::OnDataOutCompleteStatic(bool in_isr, CDCBase* self,
                                      const Endpoint::Completion& completion)
{
  if (self->data_out_ == nullptr)
  {
    // 未绑定时 OUT 只会以 ABORTED 结束，不借出槽（见 Endpoint::Close）。
    // While unbound an OUT only ends with ABORTED and lends no slot (see
    // Endpoint::Close).
    ASSERT(!completion.slot.Valid());
    return;
  }
  self->OnDataOutComplete(in_isr, completion);
}

void CDCBase::OnDataInCompleteStatic(bool in_isr, CDCBase* self,
                                     const Endpoint::Completion& completion)
{
  if (self->data_in_ != nullptr)
  {
    self->OnDataInComplete(in_isr, completion);
  }
}

void CDCBase::OnCommCompleteStatic(bool in_isr, CDCBase* self,
                                   const Endpoint::Completion& completion)
{
  UNUSED(completion);
  self->TrySendSerialState(in_isr);
}
