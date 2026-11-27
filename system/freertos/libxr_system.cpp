#include "libxr.hpp"
#include "timer.hpp"

static_assert(configTICK_RATE_HZ == 1000, "configTICK_RATE_HZ must be 1000");

#if defined(configUSE_TIMERS) && (configUSE_TIMERS + 0)
#warning \
    "It is recommended to set configUSE_TIMERS to 0 to reduce resource usage unless software timers are strictly required."
#endif

// NOLINTNEXTLINE
extern "C" __attribute__((weak)) void vApplicationStackOverflowHook(TaskHandle_t xTask,
                                                                    char* pcTaskName)
{
  static volatile const char* task_name = pcTaskName;
  UNUSED(task_name);
  UNUSED(xTask);
  REQUIRE(false);
}

uint32_t LibXR::libxr_freertos_timebase_tick_offset = 0;

#if defined(ESP_PLATFORM)
namespace LibXR::Detail
{
/// IDF 的 tick 钩子登记为节拍源（driver/esp/esp_tick.cpp）/ Registers IDF's tick hook as
/// the tick source (driver/esp/esp_tick.cpp)
void RegisterEspTickHook();
}  // namespace LibXR::Detail
#endif

void LibXR::PlatformInit(uint32_t timer_pri, uint32_t timer_stack_depth)
{
  if (!Timebase::IsReady())
  {
    /* You should initialize Timebase first */
    ASSERT(false);
  }

  LibXR::Timer::priority_ = static_cast<LibXR::Thread::Priority>(timer_pri);
  LibXR::Timer::stack_depth_ = timer_stack_depth;

  uint32_t rtos_tick = static_cast<uint32_t>(xTaskGetTickCount());
  uint32_t timebase_tick = static_cast<uint32_t>(Timebase::GetMilliseconds());

  libxr_freertos_timebase_tick_offset = rtos_tick - timebase_tick;

#if defined(ESP_PLATFORM)
  // DriverAsyncCondition 的节拍源，见 libxr_system.hpp。
  // Tick source of DriverAsyncCondition, see libxr_system.hpp.
  Detail::RegisterEspTickHook();
#endif
}

#ifndef ESP_PLATFORM
void* operator new(std::size_t size)
{
  if (size == 0)
  {
    size = sizeof(std::size_t);
  }

#ifdef LIBXR_DEBUG_BUILD
  static volatile uint32_t free_size = 0;
  UNUSED(free_size);
  free_size = xPortGetFreeHeapSize();
#endif

  auto ans = pvPortMalloc(size);
  REQUIRE(ans != nullptr);
  return ans;
}

void operator delete(void* ptr) noexcept { vPortFree(ptr); }

void operator delete(void* ptr, std::size_t size) noexcept
{
  UNUSED(size);
  vPortFree(ptr);
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
  void* raw = pvPortMalloc(space);
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
    vPortFree((static_cast<void**>(ptr))[-1]);
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

#endif
