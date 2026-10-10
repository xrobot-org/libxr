#include "logger.hpp"

#include <cstdint>

#include "libxr_def.hpp"
#include "libxr_rw.hpp"
#include "message.hpp"

using namespace LibXR;

namespace
{
static Topic log_topic;  ///< 日志发布主题 / Log publish topic

/**
 * @brief 将日志级别映射到终端颜色前缀
 *        Map one log level to the terminal color prefix
 * @param level 日志级别 / Log level
 * @return 颜色转义串；未知级别返回空串
 *         Color escape sequence, or an empty string for unknown levels
 */
const char* GetLogColor(LogLevel level)
{
  switch (level)
  {
    case LogLevel::XR_LOG_LEVEL_DEBUG:
      return LIBXR_FOREGROUND_STR[static_cast<uint8_t>(Foreground::MAGENTA)];
    case LogLevel::XR_LOG_LEVEL_INFO:
      return LIBXR_FOREGROUND_STR[static_cast<uint8_t>(Foreground::CYAN)];
    case LogLevel::XR_LOG_LEVEL_PASS:
      return LIBXR_FOREGROUND_STR[static_cast<uint8_t>(Foreground::GREEN)];
    case LogLevel::XR_LOG_LEVEL_WARN:
      return LIBXR_FOREGROUND_STR[static_cast<uint8_t>(Foreground::YELLOW)];
    case LogLevel::XR_LOG_LEVEL_ERROR:
      return LIBXR_FOREGROUND_STR[static_cast<uint8_t>(Foreground::RED)];
    default:
      return "";
  }
}

/**
 * @brief 将日志级别映射为单字符标签
 *        Map one log level to its single-character label
 * @param level 日志级别 / Log level
 * @return 单字符日志标签；未知级别返回 `"UNKNOWN"`
 *         Single-character log label, or `"UNKNOWN"` for unknown levels
 */
const char* LogLevelToString(LogLevel level)
{
  switch (level)
  {
    case LogLevel::XR_LOG_LEVEL_DEBUG:
      return "D";
    case LogLevel::XR_LOG_LEVEL_INFO:
      return "I";
    case LogLevel::XR_LOG_LEVEL_PASS:
      return "P";
    case LogLevel::XR_LOG_LEVEL_WARN:
      return "W";
    case LogLevel::XR_LOG_LEVEL_ERROR:
      return "E";
    default:
      return "UNKNOWN";
  }
}

/**
 * @brief 把一条日志渲染到当前终端输出
 *        Render one log record into the current terminal output
 * @param data 日志数据 / Log record
 * @param timestamp 日志时间戳 / Log timestamp
 *
 * @note 当前格式固定为“颜色 + 级别 + 时间戳毫秒 + 文件:行号 + 消息 + reset”。
 *       The current format is fixed as "color + level + timestamp(ms) +
 *       file:line + message + reset".
 */
void PrintLogToTerminal(const LogData& data, MicrosecondTimestamp timestamp)
{
  const char* color = GetLogColor(data.level);
  const uint32_t timestamp_ms =
      static_cast<uint32_t>(static_cast<uint64_t>(timestamp) / 1000U);
  STDIO::Print<"{}{} [{}]({}:{}) {}{}\r\n">(
      color, LogLevelToString(data.level), timestamp_ms, data.file, data.line,
      data.message,
      LIBXR_TERMINAL_CONTROL_STR[static_cast<uint8_t>(TerminalControl::RESET)]);
}

/**
 * @brief 订阅内部日志 topic，并在满足输出级别时把日志打印到终端
 *        Subscribe to the internal log topic and print logs to the terminal
 *        when the output level allows them
 * @param in_isr 当前是否在中断上下文 / Whether currently in ISR context
 * @param tp 日志 topic 句柄；当前实现未直接使用
 *           Log topic handle; unused directly by the current implementation
 * @param log_message 收到的日志消息视图 / Received log-message view
 */
void OnLogMessage([[maybe_unused]] bool in_isr, Topic tp,
                  const Topic::MessageView<LogData>& log_message)
{
  UNUSED(tp);

  DEV_ASSERT_FROM_CALLBACK(log_message.data != nullptr, in_isr);

  if (LIBXR_LOG_OUTPUT_LEVEL >= static_cast<uint8_t>(log_message.data->level) &&
      STDIO::write_ && STDIO::write_->Writable())
  {
    PrintLogToTerminal(*log_message.data, log_message.timestamp);
  }
}
}  // namespace

/**
 * @brief 初始化 logger 的内部 topic 与终端订阅路径
 *        Initialize the logger's internal topic and terminal subscriber path
 *
 * @note 这里只做一次性初始化：创建 `/xr/log` topic，并注册一个把日志打印到
 *       `STDIO` 的 callback。
 *       This performs one-time initialization only: create the `/xr/log`
 *       topic and register one callback that prints logs to `STDIO`.
 */
void Logger::Init()
{
  log_topic = Topic::CreateTopic<LogData>("/xr/log", nullptr, true);

  auto log_cb = LibXR::Topic::Callback::Create(OnLogMessage, log_topic);
  log_topic.RegisterCallback(log_cb);

  initialized_ = true;
}

/**
 * @brief 把一条日志发布到内部日志 topic
 *        Publish one log record into the internal log topic
 * @param data 待发布日志 / Log record to publish
 */
void Logger::PublishToTopic(LogData& data) { log_topic.Publish(data); }
