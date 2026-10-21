#pragma once
#include <cmath>
#include <cstdint>
#include <cstring>

#include "hid.hpp"

namespace LibXR::USB
{

// 模板化 4 轴 + 8 按钮 HID 手柄报告描述符（见类内 GetReportDesc）
// Templated 4-axes + 8-buttons HID gamepad report descriptor (see GetReportDesc in class)

/**
 * @class HIDGamepadT
 * @brief 模板化 4 轴 + 8 按钮 HID 手柄 / Templated 4-axis + 8-button HID gamepad
 *
 * @tparam LOG_MIN 轴逻辑最小值（默认 0）/ Axis logical minimum (default 0)
 * @tparam LOG_MAX 轴逻辑最大值（默认 2047）/ Axis logical maximum (default 2047)
 * @tparam IN_EP_INTERVAL_MS IN 端点轮询间隔（ms，默认 1）/ IN endpoint polling interval
 * in ms (default 1)
 *
 * @details 轴使用 16 位容器（Report Size = 16），Logical Minimum 用 0x16（有符号 16
 * 位）编码， Logical Maximum 用 0x26（16 位）编码；输入报告长度固定为 9 字节（4×2 + 1） /
 *          Axes use 16-bit containers (Report Size = 16); Logical Minimum uses 0x16
 * (signed 16-bit), Logical Maximum uses 0x26 (16-bit); input report length is fixed to 9
 * bytes (4×2 + 1).
 */
template <int16_t LOG_MIN = 0, int16_t LOG_MAX = 2047, uint8_t IN_EP_INTERVAL_MS = 1>
class HIDGamepadT
    : public HID<50 /* desc size */, 9 /* report len */, 0 /* feature len */>
{
  static_assert(LOG_MIN <= LOG_MAX, "LOG_MIN must be <= LOG_MAX");
  static_assert(LOG_MIN >= -32768 && LOG_MAX <= 32767,
                "Axis logical range must fit in int16_t");

 public:
  /**
   * @brief 构造函数 / Constructor
   * @param in_ep_num IN 端点号（必填） / IN endpoint number (required)
   * @param interface_string 接口字符串 / Interface string
   * @note 仅启用 IN 端点，默认 1ms 轮询 / IN-only, 1 ms polling by default
   */
  explicit HIDGamepadT(
      Endpoint::EPNumber in_ep_num,
      const char* interface_string = HID<50, 9, 0>::DEFAULT_INTERFACE_STRING)
      : HID<50, 9, 0>(in_ep_num, Endpoint::EPNumber::EP_INVALID, false, IN_EP_INTERVAL_MS,
                      /*out_ep_interval*/ 1, interface_string)
  {
    // 初始化上一帧：轴置中，按钮清零 / Initialize last report: axes to mid, buttons
    // cleared
    last_.x = Mid();
    last_.y = Mid();
    last_.z = Mid();
    last_.rx = Mid();
    last_.buttons = 0;
  }

  /**
   * @brief 按钮位掩码（8 个） / Button bit masks (8 buttons)
   */
  enum Button : uint8_t
  {
    BTN1 = 0x01,
    BTN2 = 0x02,
    BTN3 = 0x04,
    BTN4 = 0x08,
    BTN5 = 0x10,
    BTN6 = 0x20,
    BTN7 = 0x40,
    BTN8 = 0x80,
  };

  LIBXR_PACKED_BEGIN
  /**
   * @brief 输入报告结构（9 字节） / Input report structure (9 bytes)
   * @details 4 个 16 位轴（LOG_MIN..LOG_MAX）+ 8 位按钮 / Four 16-bit axes
   * (LOG_MIN..LOG_MAX) + 8 button bits
   */
  struct Report
  {
    int16_t x;        ///< X 轴 / X axis (LOG_MIN..LOG_MAX)
    int16_t y;        ///< Y 轴 / Y axis (LOG_MIN..LOG_MAX)
    int16_t z;        ///< Z 轴 / Z axis (LOG_MIN..LOG_MAX)
    int16_t rx;       ///< Rx 轴 / Rx axis (LOG_MIN..LOG_MAX)
    uint8_t buttons;  ///< 按钮位 / Button bits (8)
  };
  LIBXR_PACKED_END

  static_assert(sizeof(Report) == 9, "Report size must be 9 bytes");

  /**
   * @brief 发送完整输入报告（轴 + 按钮） / Send a full input report (axes + buttons)
   * @param x X 轴 / X axis
   * @param y Y 轴 / Y axis
   * @param z Z 轴 / Z axis
   * @param rx Rx 轴 / Rx axis
   * @param buttons 按钮位掩码 / Button bit mask
   * @return 错误码 / Error code
   * @note 轴值会被夹到 [LOG_MIN, LOG_MAX] / Axis values are clamped to [LOG_MIN, LOG_MAX]
   */
  ErrorCode Send(int x, int y, int z, int rx, uint8_t buttons, bool in_isr = false)
  {
    last_.x = Clamp(x);
    last_.y = Clamp(y);
    last_.z = Clamp(z);
    last_.rx = Clamp(rx);
    last_.buttons = buttons;
    return SendInputReport(ConstRawData{&last_, sizeof(last_)}, in_isr);
  }

  /**
   * @brief 仅更新按钮（保持轴不变） / Update buttons only (axes unchanged)
   * @param buttons 按钮位掩码 / Button bit mask
   * @param in_isr 是否在中断中 / Whether in ISR
   * @return 错误码 / Error code
   */
  ErrorCode SendButtons(uint8_t buttons, bool in_isr = false)
  {
    last_.buttons = buttons;
    return SendInputReport(ConstRawData{&last_, sizeof(last_)}, in_isr);
  }

  /**
   * @brief 仅更新轴（保持按钮不变） / Update axes only (buttons unchanged)
   * @param x X 轴 / X axis
   * @param y Y 轴 / Y axis
   * @param z Z 轴 / Z axis
   * @param rx Rx 轴 / Rx axis
   * @param in_isr 是否在中断中 / Whether in ISR
   * @return 错误码 / Error code
   */
  ErrorCode SendAxes(int x, int y, int z, int rx, bool in_isr = false)
  {
    last_.x = Clamp(x);
    last_.y = Clamp(y);
    last_.z = Clamp(z);
    last_.rx = Clamp(rx);
    return SendInputReport(ConstRawData{&last_, sizeof(last_)}, in_isr);
  }

 protected:
  /**
   * @brief 写入设备描述符（Per-Interface） / Write device descriptor (Per-Interface)
   * @param header 设备描述符头 / Device descriptor header
   * @return 始终返回 OK / Always returns OK
   */
  ErrorCode WriteDeviceDescriptor(DeviceDescriptor& header) override
  {
    header.data_.bDeviceClass = DeviceDescriptor::ClassID::PER_INTERFACE;  // 0x00
    header.data_.bDeviceSubClass = 0;
    header.data_.bDeviceProtocol = 0;
    return ErrorCode::OK;
  }

  /**
   * @brief 获取报告描述符 / Get report descriptor
   * @return 报告描述符及长度 / Report descriptor data and length
   */
  ConstRawData GetReportDesc() override { return ConstRawData{desc_, sizeof(desc_)}; }

 private:
  // 报告描述符（编译期常量 50 字节） / HID report descriptor (compile-time 50 bytes)
  // 轴：X/Y/Z/Rx，16 位容器；按钮：8 个 / Axes: X/Y/Z/Rx, 16-bit containers; Buttons: 8
  static constexpr uint8_t U8(uint16_t v) { return static_cast<uint8_t>(v & 0xFF); }
  static constexpr uint8_t U8H(uint16_t v)
  {
    return static_cast<uint8_t>((v >> 8) & 0xFF);
  }

  static constexpr uint8_t desc_[/*50*/] = {
      0x05, 0x01,  // Usage Page (Generic Desktop)
      0x09, 0x05,  // Usage (Game Pad)
      0xA1, 0x01,  // Collection (Application)

      // Axes collection
      0x09, 0x01,  //   Usage (Pointer)
      0xA1, 0x00,  //   Collection (Physical)
      0x05, 0x01,  //     Usage Page (Generic Desktop)
      0x09, 0x30,  //     Usage (X)
      0x09, 0x31,  //     Usage (Y)
      0x09, 0x32,  //     Usage (Z)
      0x09, 0x33,  //     Usage (Rx)

      // Logical Min/Max (16-bit)
      0x16, U8(static_cast<uint16_t>(LOG_MIN)),
      U8H(static_cast<uint16_t>(LOG_MIN)),  // Logical Minimum
      0x26, U8(static_cast<uint16_t>(LOG_MAX)),
      U8H(static_cast<uint16_t>(LOG_MAX)),  // Logical Maximum

      0x75, 0x10,  //     Report Size (16)
      0x95, 0x04,  //     Report Count (4)
      0x81, 0x02,  //     Input (Data, Variable, Absolute)
      0xC0,        //   End Collection (Physical)

      // Buttons
      0x05, 0x09,  //   Usage Page (Button)
      0x19, 0x01,  //   Usage Minimum (Button 1)
      0x29, 0x08,  //   Usage Maximum (Button 8)
      0x15, 0x00,  //   Logical Minimum (0)
      0x25, 0x01,  //   Logical Maximum (1)
      0x95, 0x08,  //   Report Count (8)
      0x75, 0x01,  //   Report Size (1)
      0x81, 0x02,  //   Input (Data, Variable, Absolute)

      0xC0  // End Collection (Application)
  };

  static_assert(sizeof(desc_) == 50,
                "HID report descriptor size changed; update base template arg!");

  /**
   * @brief 将值夹到 [LOG_MIN, LOG_MAX] / Clamp a value to [LOG_MIN, LOG_MAX]
   */
  static constexpr int16_t Clamp(int v)
  {
    return (v < LOG_MIN) ? LOG_MIN : (v > LOG_MAX ? LOG_MAX : static_cast<int16_t>(v));
  }

  /**
   * @brief 轴逻辑中点 / Logical midpoint of axes
   */
  static constexpr int16_t Mid()
  {
    return static_cast<int16_t>(
        (static_cast<int32_t>(LOG_MIN) + static_cast<int32_t>(LOG_MAX)) / 2);
  }

  Report last_{};  ///< 最近一次发送的输入报告 / Last-sent input report
};

/**
 * @brief 单极范围别名（0..2047） / Unipolar alias (0..2047)
 */
using HIDGamepad = HIDGamepadT<0, 2047, 1>;

/**
 * @brief 双极范围别名（-2048..2047） / Bipolar alias (-2048..2047)
 */
using HIDGamepadBipolar = HIDGamepadT<-2048, 2047, 1>;

}  // namespace LibXR::USB
