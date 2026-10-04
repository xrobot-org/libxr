#pragma once
#include <cstdint>
#include <cstring>

#include "hid.hpp"

namespace LibXR::USB
{

// 标准 HID Boot 键盘报告描述符
// Standard HID Boot keyboard report descriptor.
static const constexpr uint8_t HID_KEYBOARD_REPORT_DESC[] = {
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
    0x81, 0x02,  //   Input (Data, Variable, Absolute) ; 8 bits: Modifier keys
    0x95, 0x01,  //   Report Count (1)
    0x75, 0x08,  //   Report Size (8)
    0x81, 0x03,  //   Input (Constant, Variable, Absolute) ; 8 bits: Reserved
    0x95, 0x05,  //   Report Count (5)
    0x75, 0x01,  //   Report Size (1)
    0x05, 0x08,  //   Usage Page (LEDs)
    0x19, 0x01,  //   Usage Minimum (Num Lock)
    0x29, 0x05,  //   Usage Maximum (Kana)
    0x91, 0x02,  //   Output (Data, Variable, Absolute) ; 5 bits: LED report
    0x95, 0x01,  //   Report Count (1)
    0x75, 0x03,  //   Report Size (3)
    0x91, 0x03,  //   Output (Constant, Variable, Absolute) ; 3 bits: Padding
    0x95, 0x06,  //   Report Count (6)
    0x75, 0x08,  //   Report Size (8)
    0x15, 0x00,  //   Logical Minimum (0)
    0x25, 0x65,  //   Logical Maximum (101)
    0x05, 0x07,  //   Usage Page (Keyboard/Keypad)
    0x19, 0x00,  //   Usage Minimum (Reserved (no event))
    0x29, 0x65,  //   Usage Maximum (Keyboard Application)
    0x81, 0x00,  //   Input (Data, Array) ; 6 x KeyCode
    0xC0         // End Collection
};

/**
 * @class HIDKeyboard
 * @brief 标准 USB HID 键盘派生类
 *        Standard USB HID Keyboard derived class
 *
 */
class HIDKeyboard : public HID<sizeof(HID_KEYBOARD_REPORT_DESC), 8, 1>
{
 public:
  /**
   * @brief 构造函数 / Constructor
   * @param in_ep_num IN 端点号（必填）/ IN endpoint number (required)
   * @param out_ep_num OUT 端点号（必填，未启用 OUT 时传 EP_INVALID）/ OUT endpoint number
   * (required; pass EP_INVALID when OUT is disabled)
   * @param enable_out_endpoint 是否启用 OUT 端点 / Enable OUT endpoint
   * @param in_ep_interval IN 端点间隔 / IN endpoint interval
   * @param out_ep_interval OUT 端点间隔 / OUT endpoint interval
   * @param interface_string 接口字符串 / Interface string
   */
  HIDKeyboard(Endpoint::EPNumber in_ep_num, Endpoint::EPNumber out_ep_num,
              bool enable_out_endpoint = false, uint8_t in_ep_interval = 1,
              uint8_t out_ep_interval = 1,
              const char* interface_string =
                  HID<sizeof(HID_KEYBOARD_REPORT_DESC), 8, 1>::DEFAULT_INTERFACE_STRING)
      : HID(in_ep_num, out_ep_num, enable_out_endpoint, in_ep_interval, out_ep_interval,
            interface_string)
  {
  }

  /**
   * @brief 修饰键枚举 / Modifier enum
   */
  enum Modifier : uint8_t
  {
    NONE = 0x00,         ///< 无修饰键 / No modifier
    LEFT_CTRL = 0x01,    ///< 左 Ctrl / Left Control
    LEFT_SHIFT = 0x02,   ///< 左 Shift / Left Shift
    LEFT_ALT = 0x04,     ///< 左 Alt / Left Alt
    LEFT_GUI = 0x08,     ///< 左 GUI / Left GUI (Win/Command)
    RIGHT_CTRL = 0x10,   ///< 右 Ctrl / Right Control
    RIGHT_SHIFT = 0x20,  ///< 右 Shift / Right Shift
    RIGHT_ALT = 0x40,    ///< 右 Alt / Right Alt
    RIGHT_GUI = 0x80     ///< 右 GUI / Right GUI
  };

  /**
   * @brief 按键代码枚举 / KeyCode enum
   */
  enum class KeyCode : uint8_t
  {
    NONE = 0x00,             ///< 无事件 / No event
    ERROR_ROLLOVER = 0x01,   ///< 错误溢出 / ErrorRollOver
    POST_FAIL = 0x02,        ///< POST 失败 / POSTFail
    ERROR_UNDEFINED = 0x03,  ///< 未定义 / ErrorUndefined

    // 字母
    A = 0x04,
    B = 0x05,
    C = 0x06,
    D = 0x07,
    E = 0x08,
    F = 0x09,
    G = 0x0A,
    H = 0x0B,
    I = 0x0C,
    J = 0x0D,
    K = 0x0E,
    L = 0x0F,
    M = 0x10,
    N = 0x11,
    O = 0x12,
    P = 0x13,
    Q = 0x14,
    R = 0x15,
    S = 0x16,
    T = 0x17,
    U = 0x18,
    V = 0x19,
    W = 0x1A,
    X = 0x1B,
    Y = 0x1C,
    Z = 0x1D,

    // 数字上排
    NUM_1 = 0x1E,
    NUM_2 = 0x1F,
    NUM_3 = 0x20,
    NUM_4 = 0x21,
    NUM_5 = 0x22,
    NUM_6 = 0x23,
    NUM_7 = 0x24,
    NUM_8 = 0x25,
    NUM_9 = 0x26,
    NUM_0 = 0x27,

    ENTER = 0x28,
    ESCAPE = 0x29,
    BACKSPACE = 0x2A,
    TAB = 0x2B,
    SPACE = 0x2C,
    MINUS = 0x2D,          ///< `-`
    EQUAL = 0x2E,          ///< `=`
    LEFT_BRACKET = 0x2F,   ///< `[`
    RIGHT_BRACKET = 0x30,  ///< `]`
    BACKSLASH = 0x31,      ///< `\`
    NON_US_HASH = 0x32,    ///< Non-US # and ~
    SEMICOLON = 0x33,      ///< `;`
    APOSTROPHE = 0x34,     ///< `'`
    GRAVE = 0x35,          ///< `` ` ``
    COMMA = 0x36,          ///< `,`
    PERIOD = 0x37,         ///< `.`
    SLASH = 0x38,          ///< `/`
    CAPS_LOCK = 0x39,

    // F1-F12
    F1 = 0x3A,
    F2 = 0x3B,
    F3 = 0x3C,
    F4 = 0x3D,
    F5 = 0x3E,
    F6 = 0x3F,
    F7 = 0x40,
    F8 = 0x41,
    F9 = 0x42,
    F10 = 0x43,
    F11 = 0x44,
    F12 = 0x45,

    PRINT_SCREEN = 0x46,
    SCROLL_LOCK = 0x47,
    PAUSE = 0x48,
    INSERT = 0x49,
    HOME = 0x4A,
    PAGE_UP = 0x4B,
    DELETE = 0x4C,
    END = 0x4D,
    PAGE_DOWN = 0x4E,
    RIGHT_ARROW = 0x4F,
    LEFT_ARROW = 0x50,
    DOWN_ARROW = 0x51,
    UP_ARROW = 0x52,

    // 小键盘
    NUM_LOCK = 0x53,
    KEYPAD_SLASH = 0x54,     ///< Keypad /
    KEYPAD_ASTERISK = 0x55,  ///< Keypad *
    KEYPAD_MINUS = 0x56,     ///< Keypad -
    KEYPAD_PLUS = 0x57,      ///< Keypad +
    KEYPAD_ENTER = 0x58,     ///< Keypad Enter
    KEYPAD_1 = 0x59,
    KEYPAD_2 = 0x5A,
    KEYPAD_3 = 0x5B,
    KEYPAD_4 = 0x5C,
    KEYPAD_5 = 0x5D,
    KEYPAD_6 = 0x5E,
    KEYPAD_7 = 0x5F,
    KEYPAD_8 = 0x60,
    KEYPAD_9 = 0x61,
    KEYPAD_0 = 0x62,
    KEYPAD_DOT = 0x63,  ///< Keypad .

    NON_US_BACKSLASH = 0x64,  ///< Non-US \ and |
    APPLICATION = 0x65,       ///< Application (Menu)
    POWER = 0x66,             ///< Power
    KEYPAD_EQUAL = 0x67,      ///< Keypad =

    F13 = 0x68,
    F14 = 0x69,
    F15 = 0x6A,
    F16 = 0x6B,
    F17 = 0x6C,
    F18 = 0x6D,
    F19 = 0x6E,
    F20 = 0x6F,
    F21 = 0x70,
    F22 = 0x71,
    F23 = 0x72,
    F24 = 0x73,

    EXECUTE = 0x74,
    HELP = 0x75,
    MENU = 0x76,
    SELECT = 0x77,
    STOP = 0x78,
    AGAIN = 0x79,  ///< Redo
    UNDO = 0x7A,
    CUT = 0x7B,
    COPY = 0x7C,
    PASTE = 0x7D,
    FIND = 0x7E,
    MUTE = 0x7F,
    VOLUME_UP = 0x80,
    VOLUME_DOWN = 0x81,
    LOCKING_CAPS_LOCK = 0x82,
    LOCKING_NUM_LOCK = 0x83,
    LOCKING_SCROLL_LOCK = 0x84,
    KEYPAD_COMMA = 0x85,       ///< Keypad ,
    KEYPAD_EQUAL_SIGN = 0x86,  ///< Keypad =

    INTERNATIONAL1 = 0x87,
    INTERNATIONAL2 = 0x88,
    INTERNATIONAL3 = 0x89,
    INTERNATIONAL4 = 0x8A,
    INTERNATIONAL5 = 0x8B,
    INTERNATIONAL6 = 0x8C,
    INTERNATIONAL7 = 0x8D,
    INTERNATIONAL8 = 0x8E,
    INTERNATIONAL9 = 0x8F,

    LANG1 = 0x90,
    LANG2 = 0x91,
    LANG3 = 0x92,
    LANG4 = 0x93,
    LANG5 = 0x94,
    LANG6 = 0x95,
    LANG7 = 0x96,
    LANG8 = 0x97,
    LANG9 = 0x98,

    ALTERNATE_ERASE = 0x99,
    SYSREQ_ATTENTION = 0x9A,
    CANCEL = 0x9B,
    CLEAR = 0x9C,
    PRIOR = 0x9D,
    RETURN = 0x9E,
    SEPARATOR = 0x9F,
    OUT = 0xA0,
    OPER = 0xA1,
    CLEAR_AGAIN = 0xA2,
    CRSEL_PROPS = 0xA3,
    EXSEL = 0xA4,

    // 0xA5~0xDF RESERVED, skip explicit
    RESERVED_A5 = 0xA5,
    RESERVED_A6 = 0xA6,
    RESERVED_A7 = 0xA7,
    RESERVED_A8 = 0xA8,
    RESERVED_A9 = 0xA9,
    RESERVED_AA = 0xAA,
    RESERVED_AB = 0xAB,
    RESERVED_AC = 0xAC,
    RESERVED_AD = 0xAD,
    RESERVED_AE = 0xAE,
    RESERVED_AF = 0xAF,
    RESERVED_B0 = 0xB0,
    RESERVED_B1 = 0xB1,
    RESERVED_B2 = 0xB2,
    RESERVED_B3 = 0xB3,
    RESERVED_B4 = 0xB4,
    RESERVED_B5 = 0xB5,
    RESERVED_B6 = 0xB6,
    RESERVED_B7 = 0xB7,
    RESERVED_B8 = 0xB8,
    RESERVED_B9 = 0xB9,
    RESERVED_BA = 0xBA,
    RESERVED_BB = 0xBB,
    RESERVED_BC = 0xBC,
    RESERVED_BD = 0xBD,
    RESERVED_BE = 0xBE,
    RESERVED_BF = 0xBF,
    RESERVED_C0 = 0xC0,
    RESERVED_C1 = 0xC1,
    RESERVED_C2 = 0xC2,
    RESERVED_C3 = 0xC3,
    RESERVED_C4 = 0xC4,
    RESERVED_C5 = 0xC5,
    RESERVED_C6 = 0xC6,
    RESERVED_C7 = 0xC7,
    RESERVED_C8 = 0xC8,
    RESERVED_C9 = 0xC9,
    RESERVED_CA = 0xCA,
    RESERVED_CB = 0xCB,
    RESERVED_CC = 0xCC,
    RESERVED_CD = 0xCD,
    RESERVED_CE = 0xCE,
    RESERVED_CF = 0xCF,
    RESERVED_D0 = 0xD0,
    RESERVED_D1 = 0xD1,
    RESERVED_D2 = 0xD2,
    RESERVED_D3 = 0xD3,
    RESERVED_D4 = 0xD4,
    RESERVED_D5 = 0xD5,
    RESERVED_D6 = 0xD6,
    RESERVED_D7 = 0xD7,
    RESERVED_D8 = 0xD8,
    RESERVED_D9 = 0xD9,
    RESERVED_DA = 0xDA,
    RESERVED_DB = 0xDB,
    RESERVED_DC = 0xDC,
    RESERVED_DD = 0xDD,
    RESERVED_DE = 0xDE,
    RESERVED_DF = 0xDF,

    // 修饰键
    LEFT_CONTROL = 0xE0,
    LEFT_SHIFT = 0xE1,
    LEFT_ALT = 0xE2,
    LEFT_GUI = 0xE3,
    RIGHT_CONTROL = 0xE4,
    RIGHT_SHIFT = 0xE5,
    RIGHT_ALT = 0xE6,
    RIGHT_GUI = 0xE7
    // 0xE8~0xFF 保留
  };

  /**
   * @brief 输入报告结构体 / Keyboard input report struct
   */
  struct Report
  {
    uint8_t modifiers;  ///< 修饰键 / Modifier
    uint8_t reserved;   ///< 保留 / Reserved
    uint8_t keys[6];    ///< 最多 6 个按键 / Up to 6 keys
  };

  /**
   * @brief 按下指定按键 / Press the specified key(s)
   * @param keys 按键列表（最多 6 个）/ List of keys (max 6)
   * @param mods 修饰键 / Modifiers
   * @note 标准 HID Boot 协议只支持最多 6 键 / Boot protocol supports up to 6 keys
   */
  void PressKey(std::initializer_list<KeyCode> keys, uint8_t mods = Modifier::NONE)
  {
    ASSERT(keys.size() <= 6);
    report_ = {static_cast<uint8_t>(mods), 0, {0}};
    size_t i = 0;
    for (auto k : keys)
    {
      report_.keys[i++] = static_cast<uint8_t>(k);
    }
    SendInputReport(ConstRawData{&report_, sizeof(report_)});
  }

  /**
   * @brief 释放所有按键 / Release all keys
   */
  void ReleaseAll()
  {
    Report report = {0, 0, {0}};
    SendInputReport(ConstRawData{&report, sizeof(report)});
  }

 protected:
  /**
   * @brief 写入设备描述符 / Write device descriptor
   * @param header 设备描述符头指针 / Device descriptor header pointer
   * @return ErrorCode 返回 OK / Always returns OK
   */
  ErrorCode WriteDeviceDescriptor(DeviceDescriptor& header) override
  {
    header.data_.bDeviceClass = DeviceDescriptor::ClassID::HID;
    header.data_.bDeviceSubClass = 1;
    header.data_.bDeviceProtocol = 1;
    return ErrorCode::OK;
  }

  /**
   * @brief 获取报告描述符 / Get Report Descriptor
   * @return ConstRawData 报告描述符及长度 / Report descriptor data and length
   */
  ConstRawData GetReportDesc() override
  {
    return ConstRawData{HID_KEYBOARD_REPORT_DESC, sizeof(HID_KEYBOARD_REPORT_DESC)};
  }

  /**
   * @brief OUT 端点回调，处理 LED 状态 / OUT endpoint callback, handle LED status
   * @param in_isr 是否在中断中 / In ISR
   * @param data OUT 报告数据 / OUT report data
   */
  void OnDataOutComplete(bool in_isr, ConstRawData& data) override
  {
    if (data.size_ >= 1)
    {
      led_state_ = *(static_cast<const uint8_t*>(data.addr_));
      on_led_change_cb_.Run(in_isr, GetNumLock(), GetCapsLock(), GetScrollLock());
    }
  }

  /**
   * @brief 处理 SET_REPORT 请求 / Handle SET_REPORT request
   * @param report_id 报告 ID / Report ID
   * @param result 输出结果 / Output result
   * @return ErrorCode 错误码 / Error code
   */
  ErrorCode OnSetReport(uint8_t report_id,
                        DeviceClass::ControlTransferResult& result) override
  {
    UNUSED(report_id);
    UNUSED(result);

    result.read_data = {&led_state_, 1};
    return ErrorCode::OK;
  }

  /**
   * @brief 处理 SET_REPORT 数据阶段 / Handle SET_REPORT data stage
   * @param in_isr 是否在中断中 / In ISR
   * @param data 数据 / Data
   * @return ErrorCode 错误码 / Error code
   */
  ErrorCode OnSetReportData(bool in_isr, ConstRawData& data) override
  {
    if (data.size_ >= 1)
    {
      led_state_ = *(static_cast<const uint8_t*>(data.addr_));
      on_led_change_cb_.Run(in_isr, GetNumLock(), GetCapsLock(), GetScrollLock());
      return ErrorCode::OK;
    }

    return ErrorCode::NOT_SUPPORT;
  }

 public:
  /**
   * @brief 获取 NumLock 状态 / Get NumLock status
   * @return true 开启 / Enabled, false 关闭 / Disabled
   */
  bool GetNumLock() { return (led_state_ & 0x01) != 0; }

  /**
   * @brief 获取 CapsLock 状态 / Get CapsLock status
   * @return true 开启 / Enabled, false 关闭 / Disabled
   */
  bool GetCapsLock() { return (led_state_ & 0x02) != 0; }

  /**
   * @brief 获取 ScrollLock 状态 / Get ScrollLock status
   * @return true 开启 / Enabled, false 关闭 / Disabled
   */
  bool GetScrollLock() { return (led_state_ & 0x04) != 0; }

  /**
   * @brief 设置 LED 状态变化回调 / Set LED state change callback
   * @param cb 回调函数，参数为 NumLock、CapsLock、ScrollLock 状态 / Callback with
   * NumLock, CapsLock, ScrollLock status
   */
  void SetOnLedChangeCallback(LibXR::Callback<bool, bool, bool> cb)
  {
    on_led_change_cb_ = cb;
  }

 private:
  uint8_t led_state_ = 0;  ///< LED 状态/ LED state
  Report report_;          ///< 当前输入报告 / Current input report
  LibXR::Callback<bool, bool, bool>
      on_led_change_cb_;  ///< LED 状态变化回调 / LED state change callback
};

}  // namespace LibXR::USB
