#include "libxr_system.hpp"

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <new>

#include "libxr_assert.hpp"
#include "libxr_def.hpp"
#include "libxr_rw.hpp"
#include "libxr_type.hpp"
#include "list.hpp"
#include "queue.hpp"
#include "semaphore.hpp"
#include "thread.hpp"
#include "timebase.hpp"
#include "timer.hpp"

void LibXR::PlatformInit() {}

void LibXR::Timer::RefreshTimerInIdle()
{
  static bool in_timer = false;
  if (in_timer)
  {
    return;
  }

  static auto last_refresh_time = Timebase::GetMilliseconds();

  if (last_refresh_time == Timebase::GetMilliseconds())
  {
    return;
  }

  in_timer = true;
  last_refresh_time = (last_refresh_time + 1);
  Timer::Refresh();
  in_timer = false;
}

/*
 * 裸机系统的全局 operator new/delete：用 C 库堆分配，失败时走 REQUIRE，
 * 与 FreeRTOS 系统层一致。libstdc++ 自带的版本在失败时抛 std::bad_alloc，
 * 即使以 -fno-exceptions 编译也会把异常支持链接进固件。
 * Global operator new/delete of the bare-metal system: they allocate from the C library
 * heap and fail through REQUIRE, as the FreeRTOS system does. The libstdc++ versions
 * throw std::bad_alloc on failure, which links the exception support into firmware even
 * with -fno-exceptions.
 */
void* operator new(std::size_t size)
{
  if (size == 0)
  {
    size = sizeof(std::size_t);
  }

  auto ans = std::malloc(size);
  REQUIRE(ans != nullptr);
  return ans;
}

void operator delete(void* ptr) noexcept { std::free(ptr); }

void operator delete(void* ptr, std::size_t size) noexcept
{
  UNUSED(size);
  std::free(ptr);
}

void* operator new[](std::size_t size) { return ::operator new(size); }

void operator delete[](void* ptr) noexcept { ::operator delete(ptr); }

void operator delete[](void* ptr, std::size_t size) noexcept
{
  ::operator delete(ptr, size);
}

void* operator new(std::size_t size, std::align_val_t align)
{
  std::size_t a = static_cast<std::size_t>(align);
  std::size_t space = size + a + sizeof(void*);
  void* raw = std::malloc(space);
  REQUIRE(raw != nullptr);

  uintptr_t raw_addr = reinterpret_cast<uintptr_t>(raw) + sizeof(void*);
  uintptr_t aligned_addr = (raw_addr + a - 1) & ~(a - 1);
  void* aligned_ptr = reinterpret_cast<void*>(aligned_addr);  // NOLINT
  reinterpret_cast<void**>(aligned_ptr)[-1] = raw;

  return aligned_ptr;
}

void operator delete(void* ptr, std::align_val_t) noexcept
{
  if (ptr)
  {
    std::free((static_cast<void**>(ptr))[-1]);
  }
}

void operator delete(void* ptr, std::size_t, std::align_val_t align) noexcept
{
  operator delete(ptr, align);
}

void* operator new[](std::size_t size, std::align_val_t align)
{
  return ::operator new(size, align);
}

void operator delete[](void* ptr, std::align_val_t align) noexcept
{
  ::operator delete(ptr, align);
}

void operator delete[](void* ptr, std::size_t, std::align_val_t align) noexcept
{
  ::operator delete(ptr, align);
}
