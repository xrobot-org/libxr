#include "hid_mouse.hpp"

using namespace LibXR::USB;
using LibXR::ConstRawData;
using LibXR::ErrorCode;

namespace
{
/// 启动鼠标报告描述符，另加滚轮 / Boot mouse report descriptor plus a wheel
constexpr uint8_t MOUSE_REPORT_DESC[] = {
    0x05, 0x01,  // Usage Page (Generic Desktop)
    0x09, 0x02,  // Usage (Mouse)
    0xA1, 0x01,  // Collection (Application)
    0x09, 0x01,  //   Usage (Pointer)
    0xA1, 0x00,  //   Collection (Physical)
    0x05, 0x09,  //     Usage Page (Buttons)
    0x19, 0x01,  //     Usage Minimum (1)
    0x29, 0x03,  //     Usage Maximum (3)
    0x15, 0x00,  //     Logical Minimum (0)
    0x25, 0x01,  //     Logical Maximum (1)
    0x95, 0x03,  //     Report Count (3)
    0x75, 0x01,  //     Report Size (1)
    0x81, 0x02,  //     Input (Data, Variable, Absolute): buttons
    0x95, 0x01,  //     Report Count (1)
    0x75, 0x05,  //     Report Size (5)
    0x81, 0x03,  //     Input (Constant, Variable, Absolute): padding
    0x05, 0x01,  //     Usage Page (Generic Desktop)
    0x09, 0x30,  //     Usage (X)
    0x09, 0x31,  //     Usage (Y)
    0x09, 0x38,  //     Usage (Wheel)
    0x15, 0x81,  //     Logical Minimum (-127)
    0x25, 0x7F,  //     Logical Maximum (127)
    0x75, 0x08,  //     Report Size (8)
    0x95, 0x03,  //     Report Count (3)
    0x81, 0x06,  //     Input (Data, Variable, Relative): X, Y, wheel
    0xC0,        //   End Collection
    0xC0         // End Collection
};
}  // namespace

HIDMouse::HIDMouse(Endpoint::EPNumber in_ep_num, uint8_t in_ep_interval,
                   const char* interface_string)
    : HID(in_ep_num, Endpoint::EPNumber::EP_INVALID, sizeof(Report), 0U, in_ep_interval,
          1U, BootProtocol::MOUSE, interface_string)
{
}

ErrorCode HIDMouse::Move(uint8_t buttons, int8_t x, int8_t y, int8_t wheel, bool in_isr)
{
  const Report report{buttons, x, y, wheel};
  return SendInputReport({&report, sizeof(report)}, in_isr);
}

ErrorCode HIDMouse::Release(bool in_isr) { return Move(0U, 0, 0, 0, in_isr); }

ConstRawData HIDMouse::GetReportDesc() const
{
  return {MOUSE_REPORT_DESC, sizeof(MOUSE_REPORT_DESC)};
}
