#include "libxr_system.hpp"

#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cstddef>

#include "libxr_def.hpp"
#include "libxr_rw.hpp"
#include "libxr_type.hpp"
#include "linux_timebase.hpp"
#include "logger.hpp"
#include "thread.hpp"
#include "timer.hpp"

struct timespec libxr_linux_start_time_spec;  // NOLINT

static LibXR::LinuxTimebase libxr_linux_timebase;

// STDIO 线程不会退出，信号量须在进程退出期间保持有效。
// The STDIO threads never exit, so the semaphores stay valid during process exit.
static LibXR::Semaphore& stdo_sem = *new LibXR::Semaphore();
static LibXR::Semaphore& stdi_space_sem = *new LibXR::Semaphore();
static constexpr size_t host_stdio_queue_bytes = 4096;

namespace
{
class StdioReadPort final : public LibXR::ReadPort
{
 public:
  StdioReadPort(size_t size, LibXR::Semaphore& space_sem)
      : ReadPort(size), space_sem_(space_sem)
  {
  }

 protected:
  void OnReadQueueSpaceAvailable(bool in_isr) override
  {
    space_sem_.PostFromCallback(in_isr);
  }

 private:
  LibXR::Semaphore& space_sem_;
};
}  // namespace

void StdiThread(LibXR::ReadPort* read_port)
{
  static uint8_t read_buff[host_stdio_queue_bytes];

  if (!isatty(STDIN_FILENO))
  {
    XR_LOG_WARN("STDIO.read_: stdin is not a TTY, parking thread forever");
    while (true)
    {
      LibXR::Thread::Sleep(UINT32_MAX);
    }
  }

  while (true)
  {
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(STDIN_FILENO, &rfds);

    int ret = select(STDIN_FILENO + 1, &rfds, NULL, NULL, NULL);

    if (ret > 0 && FD_ISSET(STDIN_FILENO, &rfds))
    {
      int ready = 0;
      if (ioctl(STDIN_FILENO, FIONREAD, &ready) != -1 && ready > 0)
      {
        auto queue = read_port->GetReadQueue(false);
        const size_t read_size = std::min(static_cast<size_t>(ready), queue.EmptySize());
        if (read_size == 0U)
        {
          queue.Publish();
          LibXR::ErrorCode wait_result = LibXR::ErrorCode::TIMEOUT;
          do
          {
            wait_result = stdi_space_sem.Wait(UINT32_MAX);
          } while (wait_result == LibXR::ErrorCode::TIMEOUT);
          REQUIRE(wait_result == LibXR::ErrorCode::OK);
          continue;
        }

        auto size = fread(read_buff, sizeof(char), read_size, stdin);
        if (size < 1)
        {
          queue.Publish();
          continue;
        }
        [[maybe_unused]] const auto push_batch_result = queue.PushBatch(read_buff, size);
        DEV_ASSERT(push_batch_result == LibXR::ErrorCode::OK);
        queue.Publish();
      }
    }
  }
}

void StdoThread(LibXR::WritePort* write_port)
{
  while (true)
  {
    if (stdo_sem.Wait() == LibXR::ErrorCode::OK)
    {
      for (;;)
      {
        bool failed = false;
        {
          auto queue = write_port->GetWriteQueue(false);
          if (queue.Empty())
          {
            break;
          }

          const size_t accepted = queue.PopWithWriter(
              queue.AvailableSize(),
              [&failed](const uint8_t* first, size_t first_size, const uint8_t* second,
                        size_t second_size) -> size_t
              {
                const size_t first_written =
                    fwrite(first, sizeof(char), first_size, stdout);
                if (first_written != first_size)
                {
                  failed = true;
                  return first_written;
                }

                const size_t second_written =
                    fwrite(second, sizeof(char), second_size, stdout);
                if (second_written != second_size)
                {
                  failed = true;
                }
                return first_written + second_written;
              });
          UNUSED(accepted);
          UNUSED(fflush(stdout));
        }

        if (failed)
        {
          auto failed_queue = write_port->GetWriteQueue(false);
          if (!failed_queue.Empty())
          {
            failed_queue.FailFront(LibXR::ErrorCode::FAILED);
          }
        }
      }
    }
  }
}

void LibXR::PlatformInit(uint32_t timer_pri, uint32_t timer_stack_depth)
{
  LibXR::Timer::priority_ = static_cast<LibXR::Thread::Priority>(timer_pri);
  LibXR::Timer::stack_depth_ = timer_stack_depth;
  auto write_fun = [](WritePort& port, bool)
  {
    UNUSED(port);
    stdo_sem.Post();
  };

  LibXR::STDIO::write_ = new LibXR::WritePort(32, host_stdio_queue_bytes);

  *LibXR::STDIO::write_ = write_fun;

  LibXR::STDIO::read_ = new StdioReadPort(host_stdio_queue_bytes, stdi_space_sem);

  struct termios tty = {};
  if (isatty(STDIN_FILENO) && tcgetattr(STDIN_FILENO, &tty) == 0)
  {
    // 关闭规范输入模式和回显 / Disable canonical input and echo.
    tty.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &tty);  // 立即生效 / Apply immediately.
  }

  LibXR::Thread stdi_thread, stdo_thread;
  stdi_thread.Create<LibXR::ReadPort*>(LibXR::STDIO::read_, StdiThread, "STDIO.read_",
                                       1024, LibXR::Thread::Priority::MEDIUM);

  stdo_thread.Create<LibXR::WritePort*>(LibXR::STDIO::write_, StdoThread, "STDIO.write_",
                                        1024, LibXR::Thread::Priority::MEDIUM);
}
