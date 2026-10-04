#ifndef LIBXR_COLOR_HPP
#define LIBXR_COLOR_HPP

#include <cstdint>

namespace LibXR
{

/**
 * @brief 终端文本样式 / Terminal text style
 * @details 定义 ANSI 文本样式，如 BOLD（加粗）、DIM（弱化）、UNDERLINE（下划线）等。
 *          Defines ANSI text styles such as BOLD, DIM, and UNDERLINE.
 */
enum class TextStyle : uint8_t
{
  NONE = 0,
  BOLD,
  DIM,
  UNDERLINE,
  BLINK,
  REVERSE,
  CONCEALED,
  COUNT
};

/**
 * @brief 终端控制序列 / Terminal control sequence
 * @details 定义 ANSI 终端控制命令，如 RESET（重置）和 ERASE_LINE（清除当前行）。
 *          Defines ANSI terminal control commands such as RESET and ERASE_LINE.
 */
enum class TerminalControl : uint8_t
{
  NONE = 0,
  RESET,
  ERASE_LINE,
  COUNT
};

/**
 * @brief 终端前景色 / Terminal foreground color
 */
enum class Foreground : uint8_t
{
  NONE = 0,
  BLACK,
  RED,
  GREEN,
  YELLOW,
  BLUE,
  MAGENTA,
  CYAN,
  WHITE,
  COUNT
};

/**
 * @brief 终端背景色 / Terminal background color
 */
enum class Background : uint8_t
{
  NONE = 0,
  BLACK,
  RED,
  GREEN,
  YELLOW,
  BLUE,
  MAGENTA,
  CYAN,
  WHITE,
  COUNT
};

/**
 * @brief 终端文本预设 / Terminal text preset
 * @details 预组合的 ANSI 文本预设，例如黄色粗体、红色粗体、红底粗体等。
 *          Precomposed ANSI presets such as yellow bold, red bold, and bold on red.
 */
enum class Preset : uint8_t
{
  NONE = 0,
  YELLOW_BOLD,
  RED_BOLD,
  BOLD_ON_RED,
  COUNT
};

/**
 * @brief ANSI转义序列 - 文本样式 / ANSI escape sequences for text styles
 */
inline constexpr const char* LIBXR_TEXT_STYLE_STR[] = {
    "", "\033[1m", "\033[2m", "\033[4m", "\033[5m", "\033[7m", "\033[8m"};

/**
 * @brief ANSI转义序列 - 终端控制 / ANSI escape sequences for terminal controls
 */
inline constexpr const char* LIBXR_TERMINAL_CONTROL_STR[] = {"", "\033[m", "\033[K"};

/**
 * @brief ANSI转义序列 - 前景色 / ANSI escape sequences for foreground colors
 */
inline constexpr const char* LIBXR_FOREGROUND_STR[] = {
    "",         "\033[30m", "\033[31m", "\033[32m", "\033[33m",
    "\033[34m", "\033[35m", "\033[36m", "\033[37m"};

/**
 * @brief ANSI转义序列 - 背景颜色 / ANSI escape sequences for background color
 */
inline constexpr const char* LIBXR_BACKGROUND_STR[] = {
    "",         "\033[40m", "\033[41m", "\033[42m", "\033[43m",
    "\033[44m", "\033[45m", "\033[46m", "\033[47m"};

/**
 * @brief ANSI转义序列 - 文本预设 / ANSI escape sequences for text presets
 */
inline constexpr const char* LIBXR_PRESET_STR[] = {"", "\033[33m\033[1m",
                                                   "\033[31m\033[1m", "\033[1m\033[41m"};

}  // namespace LibXR

#endif  // LIBXR_COLOR_HPP
