#pragma once
#include <cstdint>
#include <cstring>

#include "hid.hpp"

namespace LibXR::USB
{

// 标准 HID Boot 鼠标报告描述符 / Standard HID Boot Mouse Report Descriptor
static const constexpr uint8_t HID_MOUSE_REPORT_DESC[] = {
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
    0x81, 0x02,  //     Input (Data, Variable, Absolute) ; Buttons
    0x95, 0x01,  //     Report Count (1)
    0x75, 0x05,  //     Report Size (5)
    0x81, 0x03,  //     Input (Constant, Variable, Absolute) ; Padding
    0x05, 0x01,  //     Usage Page (Generic Desktop)
    0x09, 0x30,  //     Usage (X)
    0x09, 0x31,  //     Usage (Y)
    0x09, 0x38,  //     Usage (Wheel)
    0x15, 0x81,  //     Logical Minimum (-127)
    0x25, 0x7F,  //     Logical Maximum (127)
    0x75, 0x08,  //     Report Size (8)
    0x95, 0x03,  //     Report Count (3)
    0x81, 0x06,  //     Input (Data, Variable, Relative) ; X, Y, Wheel
    0xC0,        //   End Collection
    0xC0         // End Collection
};

/**
 * @class HIDMouse
 * @brief 标准 USB HID 鼠标派生类 / Standard USB HID Mouse derived class
 */
class HIDMouse : public HID<sizeof(HID_MOUSE_REPORT_DESC), 4, 0>
{
 public:
  /**
   * @brief 构造函数 / Constructor
   * @param in_ep_num IN 端点号（必填）/ IN endpoint number (required)
   * @param in_ep_interval IN 端点间隔 / IN endpoint interval
   * @param interface_string 接口字符串 / Interface string
   */
  HIDMouse(Endpoint::EPNumber in_ep_num, uint8_t in_ep_interval = 1,
           const char* interface_string =
               HID<sizeof(HID_MOUSE_REPORT_DESC), 4, 0>::DEFAULT_INTERFACE_STRING)
      : HID(in_ep_num, Endpoint::EPNumber::EP_INVALID, false, in_ep_interval, 1,
            interface_string)
  {
  }

  /**
   * @brief 鼠标按钮定义 / Mouse button definitions
   */
  enum Button : uint8_t
  {
    LEFT = 0x01,
    RIGHT = 0x02,
    MIDDLE = 0x04
  };

  /**
   * @brief 鼠标报告结构体 / Mouse input report structure
   */
  struct Report
  {
    uint8_t buttons;  ///< 按键状态 / Button state
    int8_t x;         ///< X 轴相对移动 / X movement
    int8_t y;         ///< Y 轴相对移动 / Y movement
    int8_t wheel;     ///< 滚轮 / Wheel
  };

  /**
   * @brief 发送鼠标移动与按钮状态 / Send mouse movement and button state
   * @param buttons 按键位图 / Button bitmap
   * @param x X 轴移动量 / X-axis movement
   * @param y Y 轴移动量 / Y-axis movement
   * @param wheel 滚轮移动量 / Wheel movement
   */
  void Move(uint8_t buttons, int8_t x, int8_t y, int8_t wheel = 0, bool in_isr = false)
  {
    Report report = {buttons, x, y, wheel};
    SendInputReport(ConstRawData{&report, sizeof(report)}, in_isr);
  }

  /**
   * @brief 释放所有按钮 / Release all buttons
   */
  void Release(bool in_isr = false)
  {
    Report report = {0, 0, 0, 0};
    SendInputReport(ConstRawData{&report, sizeof(report)}, in_isr);
  }

 protected:
  ErrorCode WriteDeviceDescriptor(DeviceDescriptor& header) override
  {
    header.data_.bDeviceClass = DeviceDescriptor::ClassID::HID;
    header.data_.bDeviceSubClass = 1;
    header.data_.bDeviceProtocol = 2;  // Mouse
    return ErrorCode::OK;
  }

  ConstRawData GetReportDesc() override
  {
    return ConstRawData{HID_MOUSE_REPORT_DESC, sizeof(HID_MOUSE_REPORT_DESC)};
  }
};

}  // namespace LibXR::USB
