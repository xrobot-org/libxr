#include "libxr.hpp"

// libxr_fatal_error_and_print() 单独放在这个文件里：只有它引用打印的代码。没有 ASSERT 和
// DEV_ASSERT 的发布固件不引用它，链接时就不会带上这个文件和打印的代码。
// libxr_fatal_error_and_print() has this file to itself: it is the only reference to the
// printing code. Release firmware, which has no ASSERT and no DEV_ASSERT, does not refer
// to it, so the link leaves this file and the printing code out.

namespace
{
void PrintFatalError(const char* file, uint32_t line)
{
  if (file != nullptr && LibXR::STDIO::write_ && LibXR::STDIO::write_->Writable())
  {
    LibXR::STDIO::Print<"Fatal error at {}:{}\r\n">(file, static_cast<int>(line));
  }
}
}  // namespace

extern "C" void libxr_fatal_error_and_print(const char* file, uint32_t line, bool in_isr)
{
  LibXR::Assert::Detail::FatalErrorLoop(file, line, in_isr, &PrintFatalError);
}
