/**
 * @file test_logger.cpp
 * @brief Logger 格式选择与输出测试 / Logger format selection and output tests.
 *
 * 检查花括号格式、printf 格式和纯文本的输出，以及位置、颜色和换行字符。
 * Check brace, printf and plain-text output, including source locations, colors and line
 * endings.
 */

#include <cstddef>
#include <string>
#include <string_view>

#include "libxr.hpp"
#include "libxr_def.hpp"
#include "test.hpp"
#include "test_assert.hpp"

static_assert(LibXR::Detail::LoggerLiteral::SelectFrontend<
                  LibXR::Detail::LoggerLiteral::Frontend::Auto, "brace {}", int>() ==
              LibXR::Detail::LoggerLiteral::Frontend::Format);
static_assert(LibXR::Detail::LoggerLiteral::SelectFrontend<
                  LibXR::Detail::LoggerLiteral::Frontend::Auto, "printf %d", int>() ==
              LibXR::Detail::LoggerLiteral::Frontend::Printf);
static_assert(LibXR::Detail::LoggerLiteral::SelectFrontend<
                  LibXR::Detail::LoggerLiteral::Frontend::Printf, "forced %d", int>() ==
              LibXR::Detail::LoggerLiteral::Frontend::Printf);

namespace
{

size_t CountSubstring(std::string_view text, std::string_view needle)
{
  size_t count = 0;
  size_t pos = 0;
  while ((pos = text.find(needle, pos)) != std::string_view::npos)
  {
    ++count;
    pos += needle.size();
  }
  return count;
}

std::string ReadPipeText(LibXR::Pipe& pipe)
{
  const size_t output_size = pipe.GetReadPort().Size();
  TEST_ASSERT(output_size > 0);

  std::string text(output_size, '\0');
  LibXR::ReadOperation read_op;
  TEST_ASSERT(pipe.GetReadPort()(LibXR::RawData{text.data(), output_size}, read_op) ==
              LibXR::ErrorCode::OK);
  return text;
}

}  // namespace

void test_logger()
{
  LibXR::Pipe output(2048);
  auto* old_write = LibXR::STDIO::write_;
  auto* old_stream = LibXR::STDIO::write_stream_;

  // 临时把日志输出接到 Pipe，检查实际生成的文本；读取后恢复 STDIO 的原指针。
  // 本用例不执行 PlatformInit，没有 STDIO 线程会同时写日志。
  // Redirect logging to a Pipe to inspect generated text, then restore the original STDIO
  // pointers. The case runs without PlatformInit, so no STDIO worker logs meanwhile.
  LibXR::STDIO::write_ = &output.GetWritePort();
  LibXR::STDIO::write_stream_ = nullptr;

  LibXR::Logger::Publish<"brace {}">(LibXR::LogLevel::XR_LOG_LEVEL_ERROR,
                                     "logger_test.cpp", 123, 7);
  LibXR::Logger::Publish<XR_PRINTF("printf %d")>(LibXR::LogLevel::XR_LOG_LEVEL_ERROR,
                                                 "logger_test.cpp", 124, 9);
  LibXR::Logger::Publish<"plain literal">(LibXR::LogLevel::XR_LOG_LEVEL_ERROR,
                                          "logger_test.cpp", 125);

  const auto text = ReadPipeText(output);

  LibXR::STDIO::write_ = old_write;
  LibXR::STDIO::write_stream_ = old_stream;

  TEST_ASSERT(
      text.find(
          LibXR::LIBXR_FOREGROUND_STR[static_cast<size_t>(LibXR::Foreground::RED)]) !=
      std::string::npos);
  TEST_ASSERT(text.find("(logger_test.cpp:123) brace 7") != std::string::npos);
  TEST_ASSERT(text.find("(logger_test.cpp:124) printf 9") != std::string::npos);
  TEST_ASSERT(text.find("(logger_test.cpp:125) plain literal") != std::string::npos);
  TEST_ASSERT(text.find(LibXR::LIBXR_TERMINAL_CONTROL_STR[static_cast<size_t>(
                  LibXR::TerminalControl::RESET)]) != std::string::npos);
  TEST_ASSERT(CountSubstring(text, "\r\n") == 3);
}
