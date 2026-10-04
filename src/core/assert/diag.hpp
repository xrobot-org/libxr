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
