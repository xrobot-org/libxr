#include "dfu_def.hpp"

using namespace LibXR::USB;
using LibXR::ConstRawData;
using LibXR::ErrorCode;
using LibXR::USB::BosCapability;
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
/// DFU 接口类与子类 / DFU interface class and subclass
constexpr uint8_t INTERFACE_CLASS = 0xFEU;
constexpr uint8_t INTERFACE_SUB_CLASS = 0x01U;
}  // namespace

DfuInterfaceClassBase::DfuInterfaceClassBase(const char* interface_string,
                                             uint8_t protocol,
                                             const char* webusb_landing_page_url,
                                             uint8_t webusb_vendor_code,
                                             const char* winusb_device_interface_guid)
    : interface_string_(interface_string),
      winusb_device_interface_guid_(winusb_device_interface_guid),
      protocol_(protocol),
      webusb_cap_(webusb_landing_page_url, webusb_vendor_code)
{
}

const char* DfuInterfaceClassBase::GetInterfaceString(size_t local_interface_index) const
{
  return (local_interface_index == 0U) ? interface_string_ : nullptr;
}

const char* DfuInterfaceClassBase::GetWinUsbInterfaceGuid() const
{
  return (winusb_device_interface_guid_ != nullptr &&
          winusb_device_interface_guid_[0] != '\0')
             ? winusb_device_interface_guid_
             : nullptr;
}

size_t DfuInterfaceClassBase::GetBosCapabilityCount()
{
  return webusb_cap_.Enabled() ? 1U : 0U;
}

BosCapability* DfuInterfaceClassBase::GetBosCapability(size_t index)
{
  return (index == 0U && webusb_cap_.Enabled()) ? &webusb_cap_ : nullptr;
}

ErrorCode DfuInterfaceClassBase::WriteDescriptors(const EndpointPool& pool, Speed speed,
                                                  uint8_t first_interface,
                                                  ConstRawData& block)
{
  UNUSED(pool);
  UNUSED(speed);
  desc_block_.interface_desc = {9U,
                                static_cast<uint8_t>(DescriptorType::INTERFACE),
                                first_interface,
                                0U,
                                0U,
                                INTERFACE_CLASS,
                                INTERFACE_SUB_CLASS,
                                protocol_,
                                GetInterfaceStringIndex(0U)};
  desc_block_.functional = functional_;
  block = {&desc_block_, sizeof(desc_block_)};
  return ErrorCode::OK;
}

ErrorCode DfuInterfaceClassBase::OnGetDescriptor(bool in_isr, const SetupPacket& setup,
                                                 ConstRawData& data)
{
  UNUSED(in_isr);
  if ((setup.wValue >> 8U) != functional_.bDescriptorType)
  {
    return ErrorCode::NOT_SUPPORT;
  }
  // functional_ 构造后不变，满足"有效到传输结束"。
  // functional_ never changes after construction, so it stays valid to the end.
  data = {&functional_, sizeof(functional_)};
  return ErrorCode::OK;
}

ErrorCode DfuInterfaceClassBase::CheckClassRequest(const SetupPacket& setup) const
{
  const auto type = static_cast<RequestType>(setup.bmRequestType & REQ_TYPE_MASK);
  const auto recipient = static_cast<Recipient>(setup.bmRequestType & REQ_RECIPIENT_MASK);
  if (type != RequestType::CLASS || recipient != Recipient::INTERFACE ||
      setup.wIndex != interface_)
  {
    return ErrorCode::NOT_SUPPORT;
  }
  const auto request = static_cast<DFURequest>(setup.bRequest);
  const bool input = request == DFURequest::UPLOAD || request == DFURequest::GETSTATUS ||
                     request == DFURequest::GETSTATE;
  const bool is_input = (setup.bmRequestType & REQ_DIRECTION_MASK) != 0U;
  if (input != is_input ||
      (request == DFURequest::GETSTATUS && setup.wLength != sizeof(DFUStatusResponse)) ||
      (request == DFURequest::GETSTATE && setup.wLength != 1U) ||
      (request == DFURequest::DETACH && setup.wLength != 0U))
  {
    return ErrorCode::ARG_ERR;
  }
  return ErrorCode::OK;
}
