#include "libxr.hpp"
#include "thread.hpp"

namespace
{
// 循环里每一圈先执行的动作 / Action run first in every round of the loop.
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

      // 致命回调只在非中断的路径上执行，传给它的 in_isr 恒为 false。
      // The fatal callback runs on the non-ISR path only and always gets in_isr = false.
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

// 只有这个函数引用打印的代码：没有调用者时，回收未引用段的链接会把两者一起丢掉。
// Only this function refers to the printing code: with no caller, a link that collects
// unreferenced sections drops both.
extern "C" void libxr_fatal_error_and_print(const char* file, uint32_t line, bool in_isr)
{
  FatalErrorLoop(file, line, in_isr, &PrintFatalError);
}
