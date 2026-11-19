#pragma once

#include <cstdint>

#include "hid.hpp"

namespace LibXR::USB
{

/**
 * @class HIDMouse
 * @brief 启动鼠标：三个按键、X、Y、滚轮，4 字节输入报告 / Boot mouse: three buttons,
 *        X, Y and wheel, 4-byte input report
 */
class HIDMouse : public HID
{
 public:
  /**
   * @brief 构造 / Construct
   * @param in_ep_num IN 端点号 / IN endpoint number
   * @param in_ep_interval IN 的 bInterval / bInterval of IN
   * @param interface_string 接口字符串 / Interface string
   */
  explicit HIDMouse(Endpoint::EPNumber in_ep_num, uint8_t in_ep_interval = 1,
                    const char* interface_string = DEFAULT_INTERFACE_STRING);

  /// 按键位 / Button bits
  enum Button : uint8_t
  {
    LEFT = 0x01,   ///< 左键 / Left
    RIGHT = 0x02,  ///< 右键 / Right
    MIDDLE = 0x04  ///< 中键 / Middle
  };

  /// 输入报告 / Input report
  struct Report
  {
    uint8_t buttons;  ///< 按键位 / Button bits
    int8_t x;         ///< X 相对移动 / Relative X
    int8_t y;         ///< Y 相对移动 / Relative Y
    int8_t wheel;     ///< 滚轮 / Wheel
  };

  /// 发送移动和按键；返回值同 SendInputReport；任何上下文 / Send movement and buttons;
  /// returns as SendInputReport; any context
  ErrorCode Move(uint8_t buttons, int8_t x, int8_t y, int8_t wheel = 0,
                 bool in_isr = false);

  /// 松开全部按键；返回值同 SendInputReport；任何上下文 / Release every button; returns
  /// as SendInputReport; any context
  ErrorCode Release(bool in_isr = false);

 protected:
  ConstRawData GetReportDesc() const override;
};

}  // namespace LibXR::USB
