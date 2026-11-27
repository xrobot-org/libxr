#include "hid_keyboard.hpp"

using namespace LibXR::USB;
using LibXR::ConstRawData;
using LibXR::ErrorCode;

namespace
{
/// 标准启动键盘报告描述符（HID 1.11 附录 B.1）/ Standard boot keyboard report
/// descriptor (HID 1.11 appendix B.1)
constexpr uint8_t KEYBOARD_REPORT_DESC[] = {
    0x05, 0x01,  // Usage Page (Generic Desktop)
    0x09, 0x06,  // Usage (Keyboard)
    0xA1, 0x01,  // Collection (Application)
    0x05, 0x07,  //   Usage Page (Keyboard/Keypad)
    0x19, 0xE0,  //   Usage Minimum (LeftControl)
    0x29, 0xE7,  //   Usage Maximum (Right GUI)
    0x15, 0x00,  //   Logical Minimum (0)
    0x25, 0x01,  //   Logical Maximum (1)
    0x75, 0x01,  //   Report Size (1)
    0x95, 0x08,  //   Report Count (8)
    0x81, 0x02,  //   Input (Data, Variable, Absolute): modifier keys
    0x95, 0x01,  //   Report Count (1)
    0x75, 0x08,  //   Report Size (8)
    0x81, 0x03,  //   Input (Constant, Variable, Absolute): reserved
    0x95, 0x05,  //   Report Count (5)
    0x75, 0x01,  //   Report Size (1)
    0x05, 0x08,  //   Usage Page (LEDs)
    0x19, 0x01,  //   Usage Minimum (Num Lock)
    0x29, 0x05,  //   Usage Maximum (Kana)
    0x91, 0x02,  //   Output (Data, Variable, Absolute): LED report
    0x95, 0x01,  //   Report Count (1)
    0x75, 0x03,  //   Report Size (3)
    0x91, 0x03,  //   Output (Constant, Variable, Absolute): padding
    0x95, 0x06,  //   Report Count (6)
    0x75, 0x08,  //   Report Size (8)
    0x15, 0x00,  //   Logical Minimum (0)
    0x25, 0x65,  //   Logical Maximum (101)
    0x05, 0x07,  //   Usage Page (Keyboard/Keypad)
    0x19, 0x00,  //   Usage Minimum (Reserved (no event))
    0x29, 0x65,  //   Usage Maximum (Keyboard Application)
    0x81, 0x00,  //   Input (Data, Array): 6 key codes
    0xC0         // End Collection
};

constexpr uint8_t LED_NUM_LOCK = 0x01U;
constexpr uint8_t LED_CAPS_LOCK = 0x02U;
constexpr uint8_t LED_SCROLL_LOCK = 0x04U;
}  // namespace

HIDKeyboard::HIDKeyboard(Endpoint::EPNumber in_ep_num, Endpoint::EPNumber out_ep_num,
                         bool enable_out_endpoint, uint8_t in_ep_interval,
                         uint8_t out_ep_interval, const char* interface_string)
    : HID(in_ep_num, enable_out_endpoint ? out_ep_num : Endpoint::EPNumber::EP_INVALID,
          sizeof(Report), 1U, in_ep_interval, out_ep_interval, BootProtocol::KEYBOARD,
          interface_string)
{
}

ErrorCode HIDKeyboard::PressKey(std::initializer_list<KeyCode> keys, uint8_t mods,
                                bool in_isr)
{
  Report report{mods, 0U, {0U}};
  const bool phantom = keys.size() > sizeof(report.keys);
  size_t i = 0;
  for (KeyCode key : keys)
  {
    if (i >= sizeof(report.keys))
    {
      break;
    }
    report.keys[i++] = static_cast<uint8_t>(phantom ? KeyCode::ERROR_ROLLOVER : key);
  }
  return Send(report, in_isr);
}

ErrorCode HIDKeyboard::ReleaseAll(bool in_isr)
{
  return Send(Report{0U, 0U, {0U}}, in_isr);
}

bool HIDKeyboard::GetNumLock() const
{
  return (led_state_.load(std::memory_order_relaxed) & LED_NUM_LOCK) != 0U;
}

bool HIDKeyboard::GetCapsLock() const
{
  return (led_state_.load(std::memory_order_relaxed) & LED_CAPS_LOCK) != 0U;
}

bool HIDKeyboard::GetScrollLock() const
{
  return (led_state_.load(std::memory_order_relaxed) & LED_SCROLL_LOCK) != 0U;
}

void HIDKeyboard::SetOnLedChangeCallback(Callback<bool, bool, bool> cb)
{
  on_led_change_ = cb;
}

ConstRawData HIDKeyboard::GetReportDesc() const
{
  return {KEYBOARD_REPORT_DESC, sizeof(KEYBOARD_REPORT_DESC)};
}

ErrorCode HIDKeyboard::OnOutReport(bool in_isr, ConstRawData report)
{
  if (report.size_ == 0U)
  {
    return ErrorCode::ARG_ERR;
  }
  led_state_.store(static_cast<const uint8_t*>(report.addr_)[0],
                   std::memory_order_relaxed);
  on_led_change_.Run(in_isr, GetNumLock(), GetCapsLock(), GetScrollLock());
  return ErrorCode::OK;
}

ErrorCode HIDKeyboard::Send(const Report& report, bool in_isr)
{
  return SendInputReport({&report, sizeof(report)}, in_isr);
}
