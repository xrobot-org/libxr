#pragma once

#include <array>
#include <cstdint>

#include "hid.hpp"

namespace LibXR::USB
{

/**
 * @class HIDGamepadT
 * @brief 4 轴 8 键手柄，9 字节输入报告 / Gamepad with 4 axes and 8 buttons, 9-byte
 *        input report
 * @tparam LOG_MIN 轴的逻辑最小值 / Logical minimum of the axes
 * @tparam LOG_MAX 轴的逻辑最大值 / Logical maximum of the axes
 * @tparam IN_EP_INTERVAL IN 的 bInterval / bInterval of IN
 * @note 轴是 16 位，报告描述符在编译期按 LOG_MIN、LOG_MAX 生成。Send、SendButtons、
 *       SendAxes 读改上一次的报告，只能在一个上下文中调用。
 *       Axes are 16 bits; the report descriptor is built at compile time from LOG_MIN
 *       and LOG_MAX. Send, SendButtons and SendAxes read and change the last report and
 *       must be called from one context only.
 */
template <int16_t LOG_MIN = 0, int16_t LOG_MAX = 2047, uint8_t IN_EP_INTERVAL = 1>
class HIDGamepadT : public HID
{
  static_assert(LOG_MIN <= LOG_MAX, "LOG_MIN must not exceed LOG_MAX");

 public:
  LIBXR_PACKED_BEGIN
  /// 输入报告，9 字节 / Input report, 9 bytes
  struct Report
  {
    int16_t x;        ///< X 轴 / X axis
    int16_t y;        ///< Y 轴 / Y axis
    int16_t z;        ///< Z 轴 / Z axis
    int16_t rx;       ///< Rx 轴 / Rx axis
    uint8_t buttons;  ///< 按键位 / Button bits
  };
  LIBXR_PACKED_END
  static_assert(sizeof(Report) == 9U, "Report is 9 bytes");

  /// 按键位 / Button bits
  enum Button : uint8_t
  {
    BTN1 = 0x01,
    BTN2 = 0x02,
    BTN3 = 0x04,
    BTN4 = 0x08,
    BTN5 = 0x10,
    BTN6 = 0x20,
    BTN7 = 0x40,
    BTN8 = 0x80
  };

  /// 构造；轴置中，按键清零 / Construct; axes centred, buttons cleared
  explicit HIDGamepadT(Endpoint::EPNumber in_ep_num,
                       const char* interface_string = DEFAULT_INTERFACE_STRING)
      : HID(in_ep_num, Endpoint::EPNumber::EP_INVALID, sizeof(Report), 0U, IN_EP_INTERVAL,
            1U, BootProtocol::NONE, interface_string),
        last_{Mid(), Mid(), Mid(), Mid(), 0U}
  {
  }

  /// 发送轴和按键，轴夹到 [LOG_MIN, LOG_MAX]；返回值同 SendInputReport / Send axes and
  /// buttons, axes clamped to [LOG_MIN, LOG_MAX]; returns as SendInputReport
  ErrorCode Send(int x, int y, int z, int rx, uint8_t buttons, bool in_isr = false)
  {
    last_ = {Clamp(x), Clamp(y), Clamp(z), Clamp(rx), buttons};
    return SendInputReport({&last_, sizeof(last_)}, in_isr);
  }

  /// 只改按键 / Change the buttons only
  ErrorCode SendButtons(uint8_t buttons, bool in_isr = false)
  {
    last_.buttons = buttons;
    return SendInputReport({&last_, sizeof(last_)}, in_isr);
  }

  /// 只改轴 / Change the axes only
  ErrorCode SendAxes(int x, int y, int z, int rx, bool in_isr = false)
  {
    return Send(x, y, z, rx, last_.buttons, in_isr);
  }

 protected:
  ConstRawData GetReportDesc() const override
  {
    static constexpr std::array<uint8_t, 50> DESC = MakeDesc();
    return {DESC.data(), DESC.size()};
  }

 private:
  /// 低字节 / Low byte
  static constexpr uint8_t Low(int16_t v)
  {
    return static_cast<uint8_t>(static_cast<uint16_t>(v) & 0xFFU);
  }
  /// 高字节 / High byte
  static constexpr uint8_t High(int16_t v)
  {
    return static_cast<uint8_t>(static_cast<uint16_t>(v) >> 8U);
  }

  /// 生成报告描述符，50 字节 / Build the report descriptor, 50 bytes
  static constexpr std::array<uint8_t, 50> MakeDesc()
  {
    return {{
        0x05, 0x01,                         // Usage Page (Generic Desktop)
        0x09, 0x05,                         // Usage (Game Pad)
        0xA1, 0x01,                         // Collection (Application)
        0x09, 0x01,                         //   Usage (Pointer)
        0xA1, 0x00,                         //   Collection (Physical)
        0x05, 0x01,                         //     Usage Page (Generic Desktop)
        0x09, 0x30,                         //     Usage (X)
        0x09, 0x31,                         //     Usage (Y)
        0x09, 0x32,                         //     Usage (Z)
        0x09, 0x33,                         //     Usage (Rx)
        0x16, Low(LOG_MIN), High(LOG_MIN),  // Logical Minimum (16 bits)
        0x26, Low(LOG_MAX), High(LOG_MAX),  // Logical Maximum (16 bits)
        0x75, 0x10,                         //     Report Size (16)
        0x95, 0x04,                         //     Report Count (4)
        0x81, 0x02,                         //     Input (Data, Variable, Absolute)
        0xC0,                               //   End Collection
        0x05, 0x09,                         //   Usage Page (Button)
        0x19, 0x01,                         //   Usage Minimum (1)
        0x29, 0x08,                         //   Usage Maximum (8)
        0x15, 0x00,                         //   Logical Minimum (0)
        0x25, 0x01,                         //   Logical Maximum (1)
        0x95, 0x08,                         //   Report Count (8)
        0x75, 0x01,                         //   Report Size (1)
        0x81, 0x02,                         //   Input (Data, Variable, Absolute)
        0xC0                                // End Collection
    }};
  }

  /// 夹到逻辑范围 / Clamp to the logical range
  static constexpr int16_t Clamp(int v)
  {
    return (v < LOG_MIN) ? LOG_MIN : ((v > LOG_MAX) ? LOG_MAX : static_cast<int16_t>(v));
  }
  /// 逻辑中点 / Logical midpoint
  static constexpr int16_t Mid()
  {
    return static_cast<int16_t>((static_cast<int32_t>(LOG_MIN) + LOG_MAX) / 2);
  }

  /// 上一次的报告；调用者的上下文 / Last report; the caller's context
  Report last_;
};

/// 单极 0..2047 / Unipolar 0..2047
using HIDGamepad = HIDGamepadT<0, 2047, 1>;
/// 双极 -2048..2047 / Bipolar -2048..2047
using HIDGamepadBipolar = HIDGamepadT<-2048, 2047, 1>;

}  // namespace LibXR::USB
