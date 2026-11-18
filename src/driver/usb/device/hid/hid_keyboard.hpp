#pragma once

#include <atomic>
#include <cstdint>
#include <initializer_list>

#include "hid.hpp"
#include "libxr_cb.hpp"

namespace LibXR::USB
{

/**
 * @class HIDKeyboard
 * @brief 标准启动键盘：8 字节输入报告，1 字节 LED 输出报告 / Standard boot keyboard:
 *        8-byte input report, 1-byte LED output report
 *
 * LED 报告可以经可选的中断 OUT 端点到来，也可以经 SET_REPORT(Output) 到来，两条路径都进
 * OnOutReport。
 * The LED report arrives on the optional interrupt OUT endpoint or through
 * SET_REPORT(Output); both paths end in OnOutReport.
 */
class HIDKeyboard : public HID
{
 public:
  /**
   * @brief 构造 / Construct
   * @param in_ep_num IN 端点号 / IN endpoint number
   * @param out_ep_num OUT 端点号；enable_out_endpoint 为 false 时不用 / OUT endpoint
   *        number; unused when enable_out_endpoint is false
   * @param enable_out_endpoint 是否使用中断 OUT 端点接收 LED 报告 / Whether the LED
   *        report uses an interrupt OUT endpoint
   * @param in_ep_interval IN 的 bInterval / bInterval of IN
   * @param out_ep_interval OUT 的 bInterval / bInterval of OUT
   * @param interface_string 接口字符串 / Interface string
   */
  HIDKeyboard(Endpoint::EPNumber in_ep_num, Endpoint::EPNumber out_ep_num,
              bool enable_out_endpoint = false, uint8_t in_ep_interval = 1,
              uint8_t out_ep_interval = 1,
              const char* interface_string = DEFAULT_INTERFACE_STRING);

  /// 修饰键位 / Modifier bits
  enum Modifier : uint8_t
  {
    NONE = 0x00,         ///< 无 / None
    LEFT_CTRL = 0x01,    ///< 左 Ctrl / Left Control
    LEFT_SHIFT = 0x02,   ///< 左 Shift / Left Shift
    LEFT_ALT = 0x04,     ///< 左 Alt / Left Alt
    LEFT_GUI = 0x08,     ///< 左 GUI / Left GUI
    RIGHT_CTRL = 0x10,   ///< 右 Ctrl / Right Control
    RIGHT_SHIFT = 0x20,  ///< 右 Shift / Right Shift
    RIGHT_ALT = 0x40,    ///< 右 Alt / Right Alt
    RIGHT_GUI = 0x80     ///< 右 GUI / Right GUI
  };

  /// 键码（HID Usage Tables，键盘页 0x07）/ Key codes (HID Usage Tables, keyboard
  /// page 0x07)
  enum class KeyCode : uint8_t
  {
    NONE = 0x00,             ///< 无事件 / No event
    ERROR_ROLLOVER = 0x01,   ///< 按键过多 / ErrorRollOver
    POST_FAIL = 0x02,        ///< POSTFail
    ERROR_UNDEFINED = 0x03,  ///< ErrorUndefined
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
    BACKSLASH = 0x31,      ///< 反斜杠 / Backslash
    NON_US_HASH = 0x32,    ///< 非美式 # 和 ~ / Non-US # and ~
    SEMICOLON = 0x33,      ///< `;`
    APOSTROPHE = 0x34,     ///< `'`
    GRAVE = 0x35,          ///< 重音符 / Grave accent
    COMMA = 0x36,          ///< `,`
    PERIOD = 0x37,         ///< `.`
    SLASH = 0x38,          ///< `/`
    CAPS_LOCK = 0x39,
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
    NUM_LOCK = 0x53,
    KEYPAD_SLASH = 0x54,
    KEYPAD_ASTERISK = 0x55,
    KEYPAD_MINUS = 0x56,
    KEYPAD_PLUS = 0x57,
    KEYPAD_ENTER = 0x58,
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
    KEYPAD_DOT = 0x63,
    NON_US_BACKSLASH = 0x64,
    APPLICATION = 0x65,
    POWER = 0x66,
    KEYPAD_EQUAL = 0x67,
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
    AGAIN = 0x79,
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
    KEYPAD_COMMA = 0x85,
    KEYPAD_EQUAL_SIGN = 0x86,
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
    LEFT_CONTROL = 0xE0,
    LEFT_SHIFT = 0xE1,
    LEFT_ALT = 0xE2,
    LEFT_GUI = 0xE3,
    RIGHT_CONTROL = 0xE4,
    RIGHT_SHIFT = 0xE5,
    RIGHT_ALT = 0xE6,
    RIGHT_GUI = 0xE7
  };

  /// 输入报告 / Input report
  struct Report
  {
    uint8_t modifiers;  ///< 修饰键位 / Modifier bits
    uint8_t reserved;   ///< 保留 / Reserved
    uint8_t keys[6];    ///< 最多 6 个键 / Up to 6 keys
  };

  /**
   * @brief 发送按下的键 / Send the keys held down
   * @param keys 键列表；超过 6 个时六个键位都填 ErrorRollOver（HID 规范的幻影状态），
   *        修饰键照常。
   *        Key list; with more than 6 keys every key slot holds ErrorRollOver (the
   *        phantom state of the HID specification) and the modifiers are reported as
   *        usual.
   * @param mods 修饰键位 / Modifier bits
   * @param in_isr 是否在中断中 / Whether in an ISR
   * @return 同 SendInputReport / As SendInputReport
   * @note 任何上下文 / Any context
   */
  ErrorCode PressKey(std::initializer_list<KeyCode> keys, uint8_t mods = Modifier::NONE,
                     bool in_isr = false);

  /// 松开全部键；返回值同 SendInputReport；任何上下文 / Release every key; returns as
  /// SendInputReport; any context
  ErrorCode ReleaseAll(bool in_isr = false);

  /// NumLock 灯；任何上下文 / NumLock LED; any context
  bool GetNumLock() const;
  /// CapsLock 灯；任何上下文 / CapsLock LED; any context
  bool GetCapsLock() const;
  /// ScrollLock 灯；任何上下文 / ScrollLock LED; any context
  bool GetScrollLock() const;

  /**
   * @brief 设置 LED 变化回调（NumLock、CapsLock、ScrollLock）/ Set the LED change
   *        callback (NumLock, CapsLock, ScrollLock)
   * @note 在设备启动前设置；回调在 owner 中运行。
   *       Set before the device starts; the callback runs in the owner.
   */
  void SetOnLedChangeCallback(Callback<bool, bool, bool> cb);

 protected:
  ConstRawData GetReportDesc() const override;

  /// LED 报告：取第一个字节，调用回调；空报告 ARG_ERR / LED report: takes the first
  /// byte and runs the callback; an empty report gets ARG_ERR
  ErrorCode OnOutReport(bool in_isr, ConstRawData report) override;

 private:
  /// 发送一条报告 / Send one report
  ErrorCode Send(const Report& report, bool in_isr);

  /// LED 位：owner 写，任何上下文读 / LED bits: written by the owner, read anywhere
  std::atomic<uint8_t> led_state_{0U};
  /// LED 变化回调；应用在启动前设置，owner 读 / LED change callback; set by the
  /// application before start, read by the owner
  Callback<bool, bool, bool> on_led_change_;
};

}  // namespace LibXR::USB
