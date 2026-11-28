#include "libxr_system.hpp"

#include <emscripten.h>

#include <algorithm>
#include <cstring>

#include "libxr_assert.hpp"
#include "libxr_def.hpp"
#include "libxr_rw.hpp"
#include "libxr_type.hpp"
#include "queue.hpp"
#include "semaphore.hpp"
#include "serialized_service.hpp"
#include "thread.hpp"
#include "timebase.hpp"
#include "timer.hpp"
#include "webasm_timebase.hpp"

static constexpr size_t webasm_stdio_queue_bytes = 4096;

namespace
{
constexpr uint32_t EVENT_WRITE = 1U;
LibXR::SerializedService webasm_write_service;
}  // namespace

extern "C"
{
  // 接收 JavaScript 提供的输入字符串 / Receive an input string from JavaScript.
  void receive_input(const char* js_input)
  {
    if (LibXR::STDIO::read_ && LibXR::STDIO::read_->Readable())
    {
      auto queue = LibXR::STDIO::read_->GetReadQueue(false);
      const size_t size = strlen(js_input);
      const size_t accepted = std::min(size, queue.EmptySize());
      if (accepted != 0U)
      {
        [[maybe_unused]] const auto push_batch_result =
            queue.PushBatch(reinterpret_cast<const uint8_t*>(js_input), accepted);
        DEV_ASSERT(push_batch_result == LibXR::ErrorCode::OK);
      }
      queue.Publish();
    }
  }
}

void LibXR::PlatformInit()
{
  static LibXR::WebAsmTimebase libxr_webasm_timebase;

  auto write_fun = [](WritePort& port, bool in_isr)
  {
    webasm_write_service.Invoke(
        EVENT_WRITE, in_isr,
        [&port](uint32_t, bool owner_in_isr)
        {
          for (;;)
          {
            auto queue = port.GetWriteQueue(owner_in_isr);
            if (queue.Empty())
            {
              return;
            }

            const size_t offered = queue.AvailableSize();
            const size_t accepted = queue.PopWithWriter(
                offered,
                [](const uint8_t* first, size_t first_size, const uint8_t* second,
                   size_t second_size) -> size_t
                {
                  auto emit = [](const uint8_t* data, size_t size)
                  {
                    if (size == 0U)
                    {
                      return;
                    }
                    EM_ASM(
                        {
                          var ptr = $0;
                          var len = $1;
                          for (var i = 0; i < len; i++)
                          {
                            Module.put_char(String.fromCharCode(HEAPU8[ptr + i]));
                          }
                        },
                        reinterpret_cast<uintptr_t>(data), size);
                  };

                  emit(first, first_size);
                  emit(second, second_size);
                  return first_size + second_size;
                });
            DEV_ASSERT(accepted == offered);
          }
        });
  };

  LibXR::STDIO::write_ = new LibXR::WritePort(32, webasm_stdio_queue_bytes);

  *LibXR::STDIO::write_ = write_fun;

  LibXR::STDIO::read_ = new LibXR::ReadPort(webasm_stdio_queue_bytes);
}

void LibXR::Timer::RefreshTimerInIdle()
{
  static bool in_timer = false;
  if (in_timer)
  {
    return;
  }

  static auto last_refresh_time = Timebase::GetMilliseconds();

  while (static_cast<uint32_t>(Timebase::GetMilliseconds() - last_refresh_time) > 0u)
  {
    in_timer = true;
    last_refresh_time = (last_refresh_time + 1);
    Timer::Refresh();
    in_timer = false;
  }
}
