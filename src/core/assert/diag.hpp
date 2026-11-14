#pragma once

#include <cstdint>
#include <utility>

#include "libxr_cb.hpp"

namespace LibXR::Assert
{
/**
 * @brief Fatal 错误回调类型，参数为文件名与行号 / Fatal error callback type taking file
 * name and line number
 */
using FatalCallback = LibXR::Callback<const char*, uint32_t>;

namespace Detail
{
// 进程级 fatal 回调存根 / Process-global fatal callback storage.
inline FatalCallback fatal_error_callback_;  // NOLINT

/// 致命错误循环里每一圈先执行的动作，参数是文件名和行号 / Action run first in every round
/// of the fatal error loop, given the file name and the line number
using FatalReportFun = void (*)(const char* file, uint32_t line);

/**
 * @brief libxr_fatal_error() 与 libxr_fatal_error_and_print() 共用的循环 / The loop
 * shared by libxr_fatal_error() and libxr_fatal_error_and_print()
 * @param file 出错的源文件，可以为空 / Source file where the error occurred, may be null
 * @param line 出错的行号 / Line number where the error occurred
 * @param in_isr 是否发生在中断服务例程（ISR） / Whether it occurred in an ISR
 * @param report 每一圈在调用回调之前执行的动作，可以为空 / Action run in every round
 * before the callback is called, may be null
 */
void FatalErrorLoop(const char* file, uint32_t line, bool in_isr, FatalReportFun report);
}  // namespace Detail

/**
 * @brief 注册 fatal 错误回调 / Register the fatal error callback
 * @param cb 用户提供的 fatal 错误回调 / User-provided fatal error callback
 */
inline void RegisterFatalErrorCallback(FatalCallback cb)
{
  Detail::fatal_error_callback_ = std::move(cb);
}

/**
 * @brief 返回当前注册的 fatal 错误回调对象 / Return the currently registered fatal error
 * callback object
 * @return 当前注册的 fatal 错误回调对象 / Returns the currently registered fatal error
 * callback object
 */
[[nodiscard]] inline FatalCallback FatalErrorCallback()
{
  return Detail::fatal_error_callback_;
}

/**
 * @brief 执行当前 fatal 错误回调 / Run the currently registered fatal error callback
 * @param in_isr 当前调用是否来自中断上下文 / Whether the current call is from ISR context
 * @param file fatal 错误文件名 / Fatal error file name
 * @param line fatal 错误行号 / Fatal error line number
 */
inline void RunFatalErrorCallback(bool in_isr, const char* file, uint32_t line)
{
  if (!Detail::fatal_error_callback_.Empty())
  {
    Detail::fatal_error_callback_.Run(in_isr, file, line);
  }
}
}  // namespace LibXR::Assert
