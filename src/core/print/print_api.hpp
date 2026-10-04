#pragma once

#include <cstddef>
#include <cstring>
#include <limits>
#include <string_view>
#include <utility>

#include "format_surface.hpp"
#include "printf.hpp"
#include "writer.hpp"

namespace LibXR::Print
{
/**
 * @brief print 便捷接口的公开返回值约定 / Public return-value contract for the print
 * convenience surface
 *
 * 像 Write()、FormatTo()、PrintfTo() 这样的 sink 写入接口只返回
 * ErrorCode，只表达成败，不暴露长度。
 * Sink-writing helpers such as Write(), FormatTo(), and PrintfTo() return
 * ErrorCode only and report success or failure without exposing a length.
 *
 * 像 FormatIntoBuffer()、PrintfIntoBuffer() 这样的有界缓冲区接口返回 int；
 * 成功时采用 snprintf 风格的完整长度语义：返回未截断时本应产生的完整字符数，
 * 不含结尾零字节，即使发生截断也如此。运行期错误（包括格式化失败，以及完整长度
 * 已无法放入 int）返回 -1。
 * Bounded-buffer helpers such as FormatIntoBuffer() and PrintfIntoBuffer()
 * return int and follow snprintf-style length semantics on success: the full
 * formatted size excluding the trailing zero byte, even when truncation
 * occurs. Runtime errors, including formatting failures and full sizes that no
 * longer fit in int, are reported as -1.
 *
 * SNPrintf() 保持同样的契约 / SNPrintf() keeps the same contract.
 */

/**
 * @brief 将编译后的格式写入输出端 / Write one compiled format into a sink
 * @tparam Sink 输出端类型，需满足 `OutputSink` / Sink type satisfying `OutputSink`
 * @tparam Format 编译格式类型，需满足 `CompiledFormat` / Compiled format type satisfying
 * `CompiledFormat`
 * @tparam Args 调用点实参类型列表 / Call-site argument types
 * @param sink 输出端 / Destination sink
 * @param format 编译后的格式对象 / Compiled format object
 * @param args 参与本次格式化的调用点实参 / Call-site arguments used for this formatting
 * pass
 * @return 仅返回 sink 写入状态 / Returns only the sink write status
 */
template <typename Sink, typename Format, typename... Args>
  requires OutputSink<Sink> && CompiledFormat<std::remove_cvref_t<Format>>
[[nodiscard]] inline ErrorCode Write(Sink& sink, const Format& format, Args&&... args)
{
  using Built = std::remove_cvref_t<Format>;
  return Writer::template RunArgumentOrder<Sink, Built, Built::ArgumentOrder()>(
      sink, format, std::forward<Args>(args)...);
}

/**
 * @brief 面向用户的便捷包装：将一份编译格式写入一个输出端 / Public convenience wrapper
 * that writes one compiled format into one sink
 * @tparam Sink 输出端类型，需满足 `OutputSink` / Sink type satisfying `OutputSink`
 * @tparam Format 编译格式类型 / Compiled format type
 * @tparam Args 调用点实参类型列表 / Call-site argument types
 * @param sink 输出端 / Destination sink
 * @param format 编译后的格式对象 / Compiled format object
 * @param args 参与本次格式化的调用点实参 / Call-site arguments used for this formatting
 * pass
 * @return 仅返回 sink 写入状态 / Returns only the sink write status
 */
template <OutputSink Sink, typename Format, typename... Args>
[[nodiscard]] inline ErrorCode FormatTo(Sink& sink, const Format& format, Args&&... args)
{
  return Write(sink, format, std::forward<Args>(args)...);
}

/**
 * @brief 将一条 brace 风格字面量直接写入一个输出端 / Write one brace-style literal
 * directly into one sink
 * @tparam Source brace 风格格式串字面量 / Brace-style format literal
 * @tparam Sink 输出端类型，需满足 `OutputSink` / Sink type satisfying `OutputSink`
 * @tparam Args 调用点实参类型列表 / Call-site argument types
 * @param sink 输出端 / Destination sink
 * @param args 参与本次格式化的调用点实参 / Call-site arguments used for this formatting
 * pass
 * @return 仅返回 sink 写入状态 / Returns only the sink write status
 */
template <Text Source, OutputSink Sink, typename... Args>
[[nodiscard]] inline ErrorCode FormatTo(Sink& sink, Args&&... args)
{
  constexpr LibXR::Format<Source> format{};
  return FormatTo(sink, format, std::forward<Args>(args)...);
}

/**
 * @brief 将一条 printf 风格字面量直接写入一个输出端 / Write one printf-style literal
 * directly into one sink
 * @tparam Source printf 风格格式串字面量 / Printf-style format literal
 * @tparam Sink 输出端类型，需满足 `OutputSink` / Sink type satisfying `OutputSink`
 * @tparam Args 调用点实参类型列表 / Call-site argument types
 * @param sink 输出端 / Destination sink
 * @param args 参与本次格式化的调用点实参 / Call-site arguments used for this formatting
 * pass
 * @return 仅返回 sink 写入状态 / Returns only the sink write status
 */
template <Text Source, OutputSink Sink, typename... Args>
[[nodiscard]] inline ErrorCode PrintfTo(Sink& sink, Args&&... args)
{
  constexpr auto format = Printf::Build<Source>();
  return FormatTo(sink, format, std::forward<Args>(args)...);
}

/**
 * @brief 将一份编译格式写入一个有界 char 缓冲区；当容量非零时结果始终带结尾零字节 /
 * Format one compiled format into one bounded char buffer and keep the result
 * NUL-terminated when capacity is nonzero
 * @param buffer 目标缓冲区；当 `capacity == 0` 时允许为空 / Destination buffer; may be
 * null when `capacity == 0`
 * @param capacity 目标缓冲区总容量（字节） / Total destination capacity in bytes
 * @param format 编译后的格式对象 / Compiled format object
 * @param args 参与本次格式化的调用点实参 / Call-site arguments used for this formatting
 * pass
 *
 * 返回值采用 snprintf 语义：它表示未截断时本应产生的完整字符数，
 * 不含结尾零字节。截断不算错误；真正保留到 buffer 里的长度可由
 * capacity 推导。
 * The returned size follows snprintf semantics: it is the full character count
 * that would have been produced without truncation, excluding the trailing zero
 * byte. Truncation is not an error; the retained payload length can be derived
 * from capacity.
 *
 * 当 capacity 非零时，目标缓冲区始终保持 NUL 结尾。
 * When capacity is nonzero, the destination buffer is always kept
 * NUL-terminated.
 *
 * @note `buffer` 真正最多保留 `capacity - 1` 个可见字符；最后一个字节保留给结尾零 / The
 * buffer retains at most `capacity - 1` visible characters; the last byte is reserved for
 * the trailing zero
 * @note 当 `capacity == 0` 时不会写入结尾零，返回值仍然表示完整未截断长度 / When
 * `capacity == 0`, no trailing zero is written and the return value still reports the
 * full untruncated size
 * @return 成功时返回完整格式化长度（不含结尾零字节）；运行期错误或长度超出 `int`
 * 可表示范围时返回 `-1` / Returns the full formatted size excluding the trailing zero
 * byte on success, or `-1` on runtime error or `int` overflow
 */
[[nodiscard]] inline int FormatIntoBuffer(char* buffer, size_t capacity,
                                          const auto& format, auto&&... args)
{
  struct BufferSink
  {
    char* buffer = nullptr;
    size_t retain_limit = 0;
    size_t retained_size = 0;
    size_t total_size = 0;

    [[nodiscard]] ErrorCode Write(std::string_view chunk)
    {
      total_size += chunk.size();

      if (buffer == nullptr || retained_size >= retain_limit)
      {
        return ErrorCode::OK;
      }

      size_t writable = retain_limit - retained_size;
      size_t copy_size = chunk.size() < writable ? chunk.size() : writable;
      if (copy_size > 0)
      {
        std::memcpy(buffer + retained_size, chunk.data(), copy_size);
        retained_size += copy_size;
      }
      return ErrorCode::OK;
    }
  };

  BufferSink sink{
      .buffer = buffer,
      .retain_limit = capacity > 0 ? capacity - 1 : 0,
  };
  auto ec = Write(sink, format, std::forward<decltype(args)>(args)...);
  if (ec != ErrorCode::OK)
  {
    if (capacity > 0 && buffer != nullptr)
    {
      buffer[0] = '\0';
    }
    return -1;
  }

  if (capacity > 0 && buffer != nullptr)
  {
    buffer[sink.retained_size] = '\0';
  }

  if (sink.total_size > static_cast<size_t>(std::numeric_limits<int>::max()))
  {
    return -1;
  }

  return static_cast<int>(sink.total_size);
}

/**
 * @brief 将一条 brace 风格字面量直接写入一个有界 char 缓冲区 / Format one brace-style
 * literal directly into one bounded char buffer
 * @tparam Source brace 风格格式串字面量 / Brace-style format literal
 * @tparam Args 调用点实参类型列表 / Call-site argument types
 * @param buffer 目标缓冲区；当 `capacity == 0` 时允许为空 / Destination buffer; may be
 * null when `capacity == 0`
 * @param capacity 目标缓冲区总容量（字节） / Total destination capacity in bytes
 * @param args 参与本次格式化的调用点实参 / Call-site arguments used for this formatting
 * pass
 * @return 返回完整格式化长度，不含结尾零字节；运行期错误（包括尺寸溢出）返回 `-1` /
 * Returns the full formatted size excluding the trailing zero byte, or `-1` on runtime
 * error including size overflow
 */
template <Text Source, typename... Args>
[[nodiscard]] inline int FormatIntoBuffer(char* buffer, size_t capacity, Args&&... args)
{
  constexpr LibXR::Format<Source> format{};
  return FormatIntoBuffer(buffer, capacity, format, std::forward<Args>(args)...);
}

/**
 * @brief 将一条 printf 风格字面量直接写入一个有界 char 缓冲区 / Format one printf-style
 * literal directly into one bounded char buffer
 * @tparam Source printf 风格格式串字面量 / Printf-style format literal
 * @tparam Args 调用点实参类型列表 / Call-site argument types
 * @param buffer 目标缓冲区；当 `capacity == 0` 时允许为空 / Destination buffer; may be
 * null when `capacity == 0`
 * @param capacity 目标缓冲区总容量（字节） / Total destination capacity in bytes
 * @param args 参与本次格式化的调用点实参 / Call-site arguments used for this formatting
 * pass
 * @return 返回完整格式化长度，不含结尾零字节；运行期错误（包括尺寸溢出）返回 `-1` /
 * Returns the full formatted size excluding the trailing zero byte, or `-1` on runtime
 * error including size overflow
 */
template <Text Source, typename... Args>
[[nodiscard]] inline int PrintfIntoBuffer(char* buffer, size_t capacity, Args&&... args)
{
  constexpr auto format = Printf::Build<Source>();
  return FormatIntoBuffer(buffer, capacity, format, std::forward<Args>(args)...);
}

/**
 * @brief 基于编译格式路径实现的 snprintf 风格包装 / snprintf-style wrapper built on top
 * of the compiled-format path
 * @tparam Args 调用点实参类型列表 / Call-site argument types
 * @param buffer 目标缓冲区；当 `capacity == 0` 时允许为空 / Destination buffer; may be
 * null when `capacity == 0`
 * @param capacity 目标缓冲区总容量（字节） / Total destination capacity in bytes
 * @param format 编译后的格式对象 / Compiled format object
 * @param args 参与本次格式化的调用点实参 / Call-site arguments used for this formatting
 * pass
 *
 * 返回完整格式化长度（不含结尾零字节）；运行期错误（包括尺寸溢出）返回 -1。
 * Returns the full formatted size excluding the trailing zero byte, or -1 on
 * runtime error (including size overflow).
 * 截断不算错误，返回值仍然是完整长度。
 * Truncation is not an error and still returns the full size.
 *
 * @return 成功时返回完整格式化长度；运行期错误或长度超出 `int` 可表示范围时返回 `-1` /
 * Returns the full formatted size on success, or `-1` on runtime error or `int` overflow
 */
[[nodiscard]] inline int SNPrintf(char* buffer, size_t capacity, const auto& format,
                                  auto&&... args)
{
  return FormatIntoBuffer(buffer, capacity, format,
                          std::forward<decltype(args)>(args)...);
}

/**
 * @brief 面向一条 printf 风格字面量的 snprintf 风格包装 / snprintf-style wrapper for one
 * printf-style literal
 * @tparam Source printf 风格格式串字面量 / Printf-style format literal
 * @tparam Args 调用点实参类型列表 / Call-site argument types
 * @param buffer 目标缓冲区；当 `capacity == 0` 时允许为空 / Destination buffer; may be
 * null when `capacity == 0`
 * @param capacity 目标缓冲区总容量（字节） / Total destination capacity in bytes
 * @param args 参与本次格式化的调用点实参 / Call-site arguments used for this formatting
 * pass
 * @return 返回完整格式化长度，不含结尾零字节；运行期错误（包括尺寸溢出）返回 `-1` /
 * Returns the full formatted size excluding the trailing zero byte, or `-1` on runtime
 * error including size overflow
 */
template <Text Source, typename... Args>
[[nodiscard]] inline int SNPrintf(char* buffer, size_t capacity, Args&&... args)
{
  constexpr auto format = Printf::Build<Source>();
  return SNPrintf(buffer, capacity, format, std::forward<Args>(args)...);
}

}  // namespace LibXR::Print
