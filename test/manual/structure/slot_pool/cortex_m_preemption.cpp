// Cortex-M SlotPool/MPMC preemption harness compiled with arm-none-eabi-g++ and
// driven by run_slot_pool_preemption.py under Unicorn.
//
// 低优先级上下文停在 SlotPool 每个操作（Acquire、写入、Publish、Take、读取、
// Release）的中间点，或停在 Put/Get 的 FastCopy 边界时，高优先级上下文的
// Put/Get 必须不被挡住：有空闲/就绪槽一定成功，FULL/EMPTY 只反映那一刻的
// 真实状态。同一场景下 MPMC（origin/dev，无临界区）会返回假 FULL/EMPTY，
// 作为对照。
//
// While the low-priority context stops at an intermediate point of every
// SlotPool operation (Acquire, writing, Publish, Take, reading, Release) or at
// the FastCopy boundary inside Put/Get, a Put/Get from the high-priority
// context must never be blocked: it succeeds whenever a free/ready slot exists,
// and FULL/EMPTY reflect the true state at that moment. The same scenario on
// the MPMC queue (origin/dev, no critical section) returns false FULL/EMPTY and
// serves as the comparison.

#include <cstddef>
#include <cstdint>
#include <new>

#include "libxr_mem.hpp"
#include "mpmc_queue_base.hpp"
#include "slot_pool.hpp"

alignas(16) static unsigned char arena[2048];
static size_t arena_used;
void* operator new[](size_t size, std::align_val_t alignment)
{
  auto align = static_cast<size_t>(alignment);
  arena_used = (arena_used + align - 1) & ~(align - 1);
  void* p = arena + arena_used;
  arena_used += size;
  return p;
}
void operator delete[](void*) noexcept {}
void operator delete[](void*, std::align_val_t) noexcept {}
extern "C" void* memset(void* ptr, int value, size_t length)
{
  auto* bytes = static_cast<unsigned char*>(ptr);
  while (length--) *bytes++ = static_cast<unsigned char>(value);
  return ptr;
}
extern "C" void __aeabi_memclr4(void* ptr, size_t length) { memset(ptr, 0, length); }
extern "C" void libxr_fatal_error(const char*, uint32_t, bool)
{
  __asm volatile("bkpt 0");
  while (true)
  {
  }
}
extern "C" void libxr_fatal_error_and_print(const char* file, uint32_t line, bool in_isr)
{
  libxr_fatal_error(file, line, in_isr);
}

// A separate out-of-line byte copy exposes the writing/reading boundary of
// Put/Get. Same byte-copy contract as Memory::FastCopy without any OS. noinline
// keeps the symbol call stable at -O2: the runner injects the interrupt at this
// boundary, and the templates that call it live in this same translation unit.
void __attribute__((noinline)) LibXR::Memory::FastCopy(void* dst, const void* src,
                                                       size_t size)
{
  auto* d = static_cast<unsigned char*>(dst);
  auto* s = static_cast<const unsigned char*>(src);
  while (size--) *d++ = *s++;
}

// SlotPool with an injectable publish sequence for the wraparound scenario.
class WrapSlotPool : public LibXR::SlotPool<uint32_t>
{
 public:
  using LibXR::SlotPool<uint32_t>::SlotPool;

  void PrimePublishSeq(uint32_t value)
  {
    publish_seq_.store(value, std::memory_order_relaxed);
  }
};

alignas(8) static unsigned char pool_storage[sizeof(WrapSlotPool)];
static WrapSlotPool* pool;
static size_t low_index;

alignas(8) static unsigned char queue_storage[sizeof(LibXR::MPMCQueueBase)];
static LibXR::MPMCQueueBase* queue;

extern "C"
{
  volatile uint32_t sp_low_value;
  volatile int32_t sp_low_status;
  volatile int32_t sp_high_put_status;
  volatile uint32_t sp_high_put_value;
  volatile int32_t sp_high_get_status;
  volatile uint32_t sp_high_get_value;
  volatile int32_t sp_drain_status[3];
  volatile uint32_t sp_drain_value[3];

  volatile uint32_t mpmc_low_value;
  volatile int32_t mpmc_low_status;
  volatile int32_t mpmc_high_status[5];
  volatile uint32_t mpmc_high_value;
  volatile uint32_t mpmc_drain_value[3];
}

// ---------------------------------------------------------------- SlotPool //

extern "C" void SpInit2Empty()
{
  arena_used = 0;
  pool = new (pool_storage) WrapSlotPool(2);
  low_index = 0;
}

extern "C" void SpInit2One()
{
  SpInit2Empty();
  pool->Put(1000);
}

extern "C" void SpInit2Full()
{
  SpInit2Empty();
  pool->Put(1000);
  pool->Put(1001);
}

extern "C" void SpInit4TwoPublished()
{
  arena_used = 0;
  pool = new (pool_storage) WrapSlotPool(4);
  low_index = 0;
  pool->Put(1000);
  pool->Put(1001);
}

extern "C" void SpInit2FullWrap()
{
  arena_used = 0;
  pool = new (pool_storage) WrapSlotPool(2);
  pool->PrimePublishSeq(0xFFFFFFFEU);
  low_index = 0;
  pool->Put(1000);
  pool->Put(1001);
}

// Low-context steps: park the task between any two of them.
extern "C" void SpLowAcquire()
{
  size_t index = 0;
  sp_low_status = static_cast<int32_t>(pool->Acquire(index));
  low_index = index;
}
extern "C" void SpLowWrite()
{
  uint32_t value = sp_low_value;
  LibXR::Memory::FastCopy(&pool->At(low_index), &value, sizeof(value));
}
extern "C" void SpLowPublish() { pool->Publish(low_index); }
extern "C" void SpLowTake()
{
  size_t index = 0;
  sp_low_status = static_cast<int32_t>(pool->Take(index));
  low_index = index;
}
extern "C" void SpLowRead()
{
  uint32_t value = 0;
  LibXR::Memory::FastCopy(&value, &pool->At(low_index), sizeof(value));
  sp_low_value = value;
}
extern "C" void SpLowRelease() { pool->Release(low_index); }
extern "C" void SpLowPut()
{
  uint32_t value = sp_low_value;
  sp_low_status = static_cast<int32_t>(pool->Put(value));
}
extern "C" void SpLowGet()
{
  uint32_t value = 0;
  sp_low_status = static_cast<int32_t>(pool->Get(value));
  sp_low_value = value;
}

// High-context operations, run as the "interrupt" between low steps.
extern "C" void SpHighPut()
{
  uint32_t value = sp_high_put_value;
  sp_high_put_status = static_cast<int32_t>(pool->Put(value));
}
extern "C" void SpHighGet()
{
  uint32_t value = 0;
  sp_high_get_status = static_cast<int32_t>(pool->Get(value));
  sp_high_get_value = value;
}
extern "C" void SpHighGetThenPut()
{
  SpHighGet();
  SpHighPut();
}
extern "C" void SpHighPutThenDrain3()
{
  SpHighPut();
  for (int i = 0; i < 3; ++i)
  {
    uint32_t value = 0;
    sp_drain_status[i] = static_cast<int32_t>(pool->Get(value));
    sp_drain_value[i] = value;
  }
}
extern "C" void SpHighDrain2()
{
  for (int i = 0; i < 2; ++i)
  {
    uint32_t value = 0;
    sp_drain_status[i] = static_cast<int32_t>(pool->Get(value));
    sp_drain_value[i] = value;
  }
}

// ------------------------------------------------------------------- MPMC //

extern "C" void MpmcInit2Full()
{
  arena_used = 0;
  queue = new (queue_storage) LibXR::MPMCQueueBase(sizeof(uint32_t), 2);
  uint32_t value = 1000;
  queue->PushBytes(&value);
  value = 1001;
  queue->PushBytes(&value);
}

// Capacity 4 with two published items: an interrupted push claims cell 2, a
// later push publishes cell 3, and the drain must still see all three items.
extern "C" void MpmcInit4TwoPublished()
{
  arena_used = 0;
  queue = new (queue_storage) LibXR::MPMCQueueBase(sizeof(uint32_t), 4);
  uint32_t value = 1000;
  queue->PushBytes(&value);
  value = 1001;
  queue->PushBytes(&value);
}

extern "C" void MpmcLowPop()
{
  uint32_t value = 0;
  mpmc_low_status = static_cast<int32_t>(queue->PopBytes(&value));
  mpmc_low_value = value;
}
extern "C" void MpmcLowPush()
{
  uint32_t value = mpmc_low_value;
  mpmc_low_status = static_cast<int32_t>(queue->PushBytes(&value));
}
extern "C" void MpmcHighPop()
{
  uint32_t value = 0;
  mpmc_high_status[0] = static_cast<int32_t>(queue->PopBytes(&value));
  mpmc_high_value = value;
}
extern "C" void MpmcHighPush()
{
  uint32_t value = mpmc_high_value;
  mpmc_high_status[1] = static_cast<int32_t>(queue->PushBytes(&value));
}
extern "C" void MpmcHighPopThenPush()
{
  MpmcHighPop();
  MpmcHighPush();
}
extern "C" void MpmcHighPushThenDrain3()
{
  MpmcHighPush();
  for (int i = 0; i < 3; ++i)
  {
    uint32_t value = 0;
    mpmc_high_status[2 + i] = static_cast<int32_t>(queue->PopBytes(&value));
    mpmc_drain_value[i] = value;
  }
}
