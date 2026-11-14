#include "libxr.hpp"
#include "thread.hpp"

namespace
{
/// 循环里每一圈先执行的动作，参数是文件名和行号 / Action run first in every round of the
/// loop, given the file name and the line number
using ReportFun = void (*)(const char* file, uint32_t line);

void PrintFatalError(const char* file, uint32_t line)
{
  if (file != nullptr && LibXR::STDIO::write_ && LibXR::STDIO::write_->Writable())
  {
    LibXR::STDIO::Print<"Fatal error at {}:{}\r\n">(file, static_cast<int>(line));
  }
}

// 两个入口共用的循环；report 为空时不打印。
// The loop shared by the two entries; nothing is printed when report is null.
void FatalErrorLoop(const char* file, uint32_t line, bool in_isr, ReportFun report)
{
  volatile bool stop = false;
  while (!stop)
  {
    if (in_isr)
    {
      *(volatile int*)0 = 0;  // NOLINT
    }
    else
    {
      if (report != nullptr)
      {
        report(file, line);
      }

      // The fatal callback is executed only on the non-ISR fatal path here.
      // Normalize the user callback to thread context instead of forwarding the
      // original fault source flag.
      LibXR::Assert::RunFatalErrorCallback(false, file, line);

      LibXR::Thread::Sleep(500);
    }
  }
}
}  // namespace

extern "C" void libxr_fatal_error(const char* file, uint32_t line, bool in_isr)
{
  FatalErrorLoop(file, line, in_isr, nullptr);
}

// 只有这个函数引用打印的代码。发布固件里没有 ASSERT 和 DEV_ASSERT，不引用它；按函数分段
// 并回收未引用段的链接会把它和打印的代码一起丢掉。
// This function is the only reference to the printing code. Release firmware has no
// ASSERT and no DEV_ASSERT and does not refer to it; a link that puts every function in
// its own section and collects unreferenced sections drops it together with the printing
// code.
extern "C" void libxr_fatal_error_and_print(const char* file, uint32_t line, bool in_isr)
{
  FatalErrorLoop(file, line, in_isr, &PrintFatalError);
}
