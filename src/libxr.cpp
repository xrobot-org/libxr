#include "libxr.hpp"

#include "thread.hpp"

void LibXR::Assert::Detail::FatalErrorLoop(const char* file, uint32_t line, bool in_isr,
                                           FatalReportFun report)
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

extern "C" void libxr_fatal_error(const char* file, uint32_t line, bool in_isr)
{
  LibXR::Assert::Detail::FatalErrorLoop(file, line, in_isr, nullptr);
}
