/**
 * @file test_object_pool.cpp
 * @brief 检查共享对象池的所有权、只读转换、外部存储和并发归还。 /
 * Tests shared ownership, const conversion, external slots and concurrent returns.
 *
 * 检查三种空闲队列，以及外部队列、外部存储和析构自动回收。
 * Checks three free-slot queues, external storage and automatic handle release.
 */

#include <array>
#include <barrier>
#include <chrono>
#include <thread>
#include <type_traits>
#include <utility>

#include "libxr.hpp"
#include "test.hpp"
#include "test_assert.hpp"

namespace
{
struct Payload
{
  int value = 0;
};

template <typename QueueType>
void RunExternalQueueChecks()
{
  QueueType free_queue(3);
  LibXR::BasicObjectPool<Payload, QueueType> pool(free_queue, 3);

  typename LibXR::BasicObjectPool<Payload, QueueType>::Handle a;
  typename LibXR::BasicObjectPool<Payload, QueueType>::Handle b;
  typename LibXR::BasicObjectPool<Payload, QueueType>::Handle c;

  TEST_ASSERT(pool.Acquire(a) == LibXR::ErrorCode::OK);
  TEST_ASSERT(pool.Acquire(b) == LibXR::ErrorCode::OK);
  TEST_ASSERT(pool.Acquire(c) == LibXR::ErrorCode::OK);
  TEST_ASSERT(pool.EmptySize() == 0);

  b.Reset();
  TEST_ASSERT(pool.EmptySize() == 1);
}

template <typename QueueType>
void RunExternalSlotChecks()
{
  using Pool = LibXR::BasicObjectPool<Payload, QueueType>;
  typename Pool::Slot slots[3] = {};
  Pool pool(3, slots);

  typename LibXR::BasicObjectPool<Payload, QueueType>::Handle handle;
  TEST_ASSERT(pool.Acquire(handle) == LibXR::ErrorCode::OK);
  handle->value = 123;
  TEST_ASSERT(pool.UnsafeAt(handle.Index()).value == 123);
  const auto address = reinterpret_cast<uintptr_t>(&handle.Get());
  const auto slot_address = reinterpret_cast<uintptr_t>(&slots[handle.Index()]);
  TEST_ASSERT(address >= slot_address && address < slot_address + sizeof(slots[0]));
}

template <typename QueueType>
void RunExternalQueueAndSlotChecks()
{
  QueueType free_queue(3);
  using Pool = LibXR::BasicObjectPool<Payload, QueueType>;
  typename Pool::Slot slots[3] = {};
  Pool pool(free_queue, 3, slots);

  typename LibXR::BasicObjectPool<Payload, QueueType>::Handle handle;
  TEST_ASSERT(pool.Acquire(handle) == LibXR::ErrorCode::OK);
  handle->value = 456;
  TEST_ASSERT(pool.UnsafeAt(handle.Index()).value == 456);
  const auto address = reinterpret_cast<uintptr_t>(&handle.Get());
  const auto slot_address = reinterpret_cast<uintptr_t>(&slots[handle.Index()]);
  TEST_ASSERT(address >= slot_address && address < slot_address + sizeof(slots[0]));
}

template <typename Pool>
void RunSharedHandleChecks()
{
  using Handle = typename Pool::Handle;
  using ConstHandle = typename Pool::ConstHandle;
  static_assert(std::is_copy_constructible_v<Handle>);
  static_assert(std::is_copy_assignable_v<ConstHandle>);
  static_assert(std::is_convertible_v<Handle, ConstHandle>);
  static_assert(!std::is_constructible_v<Handle, ConstHandle>);
  static_assert(!std::is_assignable_v<Handle&, const ConstHandle&>);
  static_assert(!std::is_assignable_v<Handle&, ConstHandle&&>);
  static_assert(!std::is_constructible_v<Handle, Pool*, typename Pool::IndexType>);
  static_assert(!std::is_constructible_v<ConstHandle, Pool*, typename Pool::IndexType>);
  static_assert(!std::is_copy_constructible_v<typename Pool::Slot>);
  static_assert(!std::is_move_constructible_v<typename Pool::Slot>);
  static_assert(std::is_same_v<decltype(std::declval<Handle&>().Get()), Payload&>);
  static_assert(
      std::is_same_v<decltype(std::declval<const Handle&>().Get()), const Payload&>);
  static_assert(
      std::is_same_v<decltype(std::declval<ConstHandle&>().Get()), const Payload&>);
  static_assert(std::is_same_v<decltype(std::declval<ConstHandle&>().operator->()),
                               const Payload*>);

  Pool pool(2);
  Handle owner;
  TEST_ASSERT(pool.Acquire(owner) == LibXR::ErrorCode::OK);
  owner->value = 123;
  const auto index = owner.Index();
  const Payload* address = &owner.Get();
  Handle copy = owner;
  ConstHandle reader = owner;
  owner.Reset();
  copy.Reset();
  TEST_ASSERT(pool.EmptySize() == 1U);
  TEST_ASSERT(reader.Index() == index && &reader.Get() == address);
  TEST_ASSERT(reader->value == 123);

  ConstHandle reader_copy(reader);
  reader.Reset();
  Handle other;
  TEST_ASSERT(pool.Acquire(other) == LibXR::ErrorCode::OK);
  TEST_ASSERT(other.Index() != index);
  Handle exhausted;
  TEST_ASSERT(pool.Acquire(exhausted) == LibXR::ErrorCode::EMPTY);
  TEST_ASSERT(!exhausted.Valid());
  ConstHandle moved(std::move(reader_copy));
  TEST_ASSERT(!reader_copy.Valid() && moved.Valid());
  reader_copy.Reset();
  TEST_ASSERT(pool.EmptySize() == 0U);
  moved.Reset();
  other.Reset();
  TEST_ASSERT(pool.EmptySize() == 2U);

  // Self-assignment and same-slot aliases must not lose the final reference.
  TEST_ASSERT(pool.Acquire(owner) == LibXR::ErrorCode::OK);
  auto& self = owner;
  owner = self;
  owner = std::move(self);
  copy = owner;
  owner = copy;
  copy = std::move(owner);
  TEST_ASSERT(!owner.Valid() && copy.Valid());
  reader = copy;
  reader = copy;
  reader = std::move(copy);
  TEST_ASSERT(!copy.Valid() && reader.Valid());
  auto& reader_self = reader;
  reader = reader_self;
  reader = std::move(reader_self);
  TEST_ASSERT(pool.EmptySize() == 1U);
  reader.Reset();
  TEST_ASSERT(pool.EmptySize() == 2U);

  // Cross-pool assignment releases the old destination, not the source's slot.
  Pool second_pool(2);
  TEST_ASSERT(pool.Acquire(owner) == LibXR::ErrorCode::OK);
  TEST_ASSERT(second_pool.Acquire(copy) == LibXR::ErrorCode::OK);
  owner = copy;
  TEST_ASSERT(pool.EmptySize() == 2U && second_pool.EmptySize() == 1U);
  copy.Reset();
  TEST_ASSERT(second_pool.EmptySize() == 1U);
  owner = Handle{};
  TEST_ASSERT(second_pool.EmptySize() == 2U);

  TEST_ASSERT(pool.Acquire(owner) == LibXR::ErrorCode::OK);
  TEST_ASSERT(second_pool.Acquire(copy) == LibXR::ErrorCode::OK);
  reader = owner;
  reader = std::move(copy);
  TEST_ASSERT(!copy.Valid());
  owner.Reset();
  TEST_ASSERT(pool.EmptySize() == 2U && second_pool.EmptySize() == 1U);
  ConstHandle empty;
  reader = empty;
  TEST_ASSERT(second_pool.EmptySize() == 2U);
  Handle empty_owner;
  ConstHandle empty_copy(empty_owner);
  ConstHandle empty_move(std::move(empty_owner));
  TEST_ASSERT(!empty_copy.Valid() && !empty_move.Valid());
}

struct LifetimePayload
{
  inline static int constructions = 0;
  inline static int destructions = 0;
  int value = 0;
  LifetimePayload() { ++constructions; }
  ~LifetimePayload() { ++destructions; }
};

struct alignas(64) InPlacePayload
{
  int value;
  int* destructions;
  InPlacePayload(int initial, int& destroyed) : value(initial), destructions(&destroyed)
  {
  }
  ~InPlacePayload() { ++*destructions; }
  InPlacePayload(const InPlacePayload&) = delete;
  InPlacePayload& operator=(const InPlacePayload&) = delete;
};

void RunResidentStorageChecks()
{
  LifetimePayload::constructions = 0;
  LifetimePayload::destructions = 0;
  {
    LibXR::ObjectPool<LifetimePayload> pool(1);
    TEST_ASSERT(LifetimePayload::constructions == 1);
    decltype(pool)::Handle owner;
    TEST_ASSERT(pool.Acquire(owner) == LibXR::ErrorCode::OK);
    owner->value = 321;
    {
      decltype(pool)::ConstHandle reader = std::move(owner);
      auto second_reader = reader;
      TEST_ASSERT(second_reader->value == 321);
    }
    TEST_ASSERT(LifetimePayload::destructions == 0);
    TEST_ASSERT(pool.Acquire(owner) == LibXR::ErrorCode::OK);
    TEST_ASSERT(owner->value == 321);
    owner.Reset();
    TEST_ASSERT(LifetimePayload::constructions == 1);
  }
  TEST_ASSERT(LifetimePayload::destructions == 1);

  using Pool = LibXR::MPMCObjectPool<InPlacePayload>;
  static_assert(!std::is_default_constructible_v<Pool::Slot>);
  static_assert(!std::is_constructible_v<Pool, size_t>);
  static_assert(alignof(Pool::Slot) >= alignof(InPlacePayload));
  int destructions = 0;
  {
    Pool::Slot slots[] = {Pool::Slot(std::in_place, 17, destructions),
                          Pool::Slot(std::in_place, 29, destructions)};
    LibXR::MPMCQueue<uint32_t> queue(2);
    {
      Pool pool(queue, 2, slots);
      Pool::Handle owner;
      TEST_ASSERT(pool.Acquire(owner) == LibXR::ErrorCode::OK);
      TEST_ASSERT(owner->value == 17);
      TEST_ASSERT(reinterpret_cast<uintptr_t>(&owner.Get()) % 64U == 0U);
      Pool::ConstHandle reader = std::move(owner);
      reader.Reset();
      TEST_ASSERT(destructions == 0);
    }
    TEST_ASSERT(destructions == 0);
  }
  TEST_ASSERT(destructions == 2);
}

template <size_t WorkerCount>
void RunSameSlotReleaseChecks()
{
  using Pool = LibXR::MPMCObjectPool<Payload>;
  constexpr size_t rounds = 64;
  Pool pool(2);
  Pool::Handle held_slot;
  TEST_ASSERT(pool.Acquire(held_slot) == LibXR::ErrorCode::OK);
  std::array<Pool::ConstHandle, WorkerCount> readers;
  std::array<std::thread, WorkerCount> workers;
  std::barrier start(static_cast<std::ptrdiff_t>(WorkerCount + 1U));
  std::barrier done(static_cast<std::ptrdiff_t>(WorkerCount + 1U));

  for (size_t i = 0; i < WorkerCount; ++i)
  {
    workers[i] = std::thread(
        [&, i]()
        {
          for (size_t round = 0; round < rounds; ++round)
          {
            start.arrive_and_wait();
            readers[i].Reset();
            done.arrive_and_wait();
          }
        });
  }

  for (size_t round = 0; round < rounds; ++round)
  {
    Pool::Handle owner;
    TEST_ASSERT(pool.Acquire(owner) == LibXR::ErrorCode::OK);
    const auto index = owner.Index();
    TEST_ASSERT(index != held_slot.Index());
    owner->value = static_cast<int>(round);
    for (auto& reader : readers) reader = owner;
    owner.Reset();
    TEST_ASSERT(pool.EmptySize() == 0U);

    // All workers now decrement the same counter, without ordering their releases.
    start.arrive_and_wait();
    done.arrive_and_wait();

    for (const auto& reader : readers) TEST_ASSERT(!reader.Valid());
    TEST_ASSERT(pool.EmptySize() == 1U);
    Pool::Handle returned;
    TEST_ASSERT(pool.Acquire(returned) == LibXR::ErrorCode::OK);
    TEST_ASSERT(returned.Index() == index);
    TEST_ASSERT(returned->value == static_cast<int>(round));
    Pool::Handle duplicate;
    TEST_ASSERT(pool.Acquire(duplicate) == LibXR::ErrorCode::EMPTY);
    TEST_ASSERT(!duplicate.Valid());
    returned.Reset();
  }

  for (auto& worker : workers) worker.join();
  TEST_ASSERT(pool.EmptySize() == 1U);
  held_slot.Reset();
  TEST_ASSERT(pool.EmptySize() == 2U);
}

void RunConcurrentReturnChecks()
{
  using Pool = LibXR::MPMCObjectPool<Payload>;
  constexpr size_t count = 8;
  Pool pool(count);
  std::array<Pool::ConstHandle, count> first;
  std::array<Pool::ConstHandle, count> second;
  std::array<Pool::Handle, count> reacquired;
  for (size_t i = 0; i < count; ++i)
  {
    Pool::Handle owner;
    TEST_ASSERT(pool.Acquire(owner) == LibXR::ErrorCode::OK);
    owner->value = static_cast<int>(owner.Index());
    first[i] = owner;
    second[i] = std::move(owner);
  }

  // Phase one leaves even-slot final returns to worker1 and odd ones to worker2.
  std::barrier phase(2);
  std::thread worker1(
      [&]()
      {
        for (size_t i = 1; i < count; i += 2) first[i].Reset();
        phase.arrive_and_wait();
        for (size_t i = 0; i < count; i += 2) first[i].Reset();
      });
  std::thread worker2(
      [&]()
      {
        for (size_t i = 0; i < count; i += 2) second[i].Reset();
        phase.arrive_and_wait();
        for (size_t i = 1; i < count; i += 2) second[i].Reset();
      });
  std::array<bool, count> seen{};
  size_t acquired = 0;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (acquired < count && std::chrono::steady_clock::now() < deadline)
  {
    const auto result = pool.Acquire(reacquired[acquired]);
    if (result == LibXR::ErrorCode::EMPTY)
    {
      std::this_thread::yield();
      continue;
    }
    TEST_ASSERT(result == LibXR::ErrorCode::OK);
    const auto index = reacquired[acquired].Index();
    TEST_ASSERT(index < count && !seen[index]);
    TEST_ASSERT(reacquired[acquired]->value == static_cast<int>(index));
    seen[index] = true;
    reacquired[acquired]->value = static_cast<int>(index + 1000U);
    ++acquired;
  }
  TEST_ASSERT(acquired == count);
  worker1.join();
  worker2.join();
  TEST_ASSERT(pool.EmptySize() == 0U);
  for (auto& handle : reacquired)
  {
    TEST_ASSERT(handle->value == static_cast<int>(handle.Index() + 1000U));
    handle.Reset();
  }
  TEST_ASSERT(pool.EmptySize() == count);
}
}  // namespace

void test_object_pool()
{
  RunSharedHandleChecks<LibXR::ObjectPool<Payload>>();
  RunSharedHandleChecks<LibXR::SPSCObjectPool<Payload>>();
  RunSharedHandleChecks<LibXR::MPMCObjectPool<Payload>>();
  RunResidentStorageChecks();
  RunSameSlotReleaseChecks<8>();
  RunSameSlotReleaseChecks<32>();
  RunConcurrentReturnChecks();
  // Basic acquire/release using the ordinary FIFO queue as free-index storage.
  {
    LibXR::ObjectPool<Payload> pool(3);

    LibXR::ObjectPool<Payload>::Handle a;
    LibXR::ObjectPool<Payload>::Handle b;
    LibXR::ObjectPool<Payload>::Handle c;
    LibXR::ObjectPool<Payload>::Handle d;

    TEST_ASSERT(pool.Acquire(a) == LibXR::ErrorCode::OK);
    TEST_ASSERT(pool.Acquire(b) == LibXR::ErrorCode::OK);
    TEST_ASSERT(pool.Acquire(c) == LibXR::ErrorCode::OK);
    TEST_ASSERT(pool.EmptySize() == 0);
    TEST_ASSERT(pool.Acquire(d) == LibXR::ErrorCode::EMPTY);

    a->value = 11;
    b->value = 22;
    c->value = 33;
    TEST_ASSERT((*a).value == 11);
    TEST_ASSERT((*b).value == 22);
    TEST_ASSERT((*c).value == 33);

    const auto a_index = a.Index();
    a.Reset();
    TEST_ASSERT(pool.EmptySize() == 1);
    TEST_ASSERT(pool.Acquire(d) == LibXR::ErrorCode::OK);
    TEST_ASSERT(d.Index() == a_index);
    d->value = 44;
    TEST_ASSERT(pool.UnsafeAt(d.Index()).value == 44);
  }

  // The same pool contract should work with the typed SPSC free-index queue.
  {
    LibXR::SPSCObjectPool<Payload> pool(3);

    LibXR::SPSCObjectPool<Payload>::Handle a;
    LibXR::SPSCObjectPool<Payload>::Handle b;
    LibXR::SPSCObjectPool<Payload>::Handle c;
    LibXR::SPSCObjectPool<Payload>::Handle d;

    TEST_ASSERT(pool.Acquire(a) == LibXR::ErrorCode::OK);
    TEST_ASSERT(pool.Acquire(b) == LibXR::ErrorCode::OK);
    TEST_ASSERT(pool.Acquire(c) == LibXR::ErrorCode::OK);
    TEST_ASSERT(pool.EmptySize() == 0);
    TEST_ASSERT(pool.Acquire(d) == LibXR::ErrorCode::EMPTY);

    a->value = 11;
    b->value = 22;
    c->value = 33;
    TEST_ASSERT((*a).value == 11);
    TEST_ASSERT((*b).value == 22);
    TEST_ASSERT((*c).value == 33);

    const auto a_index = a.Index();
    a.Reset();
    TEST_ASSERT(pool.EmptySize() == 1);
    TEST_ASSERT(pool.Acquire(d) == LibXR::ErrorCode::OK);
    TEST_ASSERT(d.Index() == a_index);
    d->value = 44;
    TEST_ASSERT(pool.UnsafeAt(d.Index()).value == 44);
  }

  // The same pool contract should work with the typed MPMC free-index queue.
  {
    LibXR::MPMCObjectPool<Payload> pool(3);

    LibXR::MPMCObjectPool<Payload>::Handle a;
    LibXR::MPMCObjectPool<Payload>::Handle b;
    LibXR::MPMCObjectPool<Payload>::Handle c;
    LibXR::MPMCObjectPool<Payload>::Handle d;

    TEST_ASSERT(pool.Acquire(a) == LibXR::ErrorCode::OK);
    TEST_ASSERT(pool.Acquire(b) == LibXR::ErrorCode::OK);
    TEST_ASSERT(pool.Acquire(c) == LibXR::ErrorCode::OK);
    TEST_ASSERT(pool.EmptySize() == 0);
    TEST_ASSERT(pool.Acquire(d) == LibXR::ErrorCode::EMPTY);

    const auto a_index = a.Index();
    a.Reset();
    TEST_ASSERT(pool.EmptySize() == 1);
    TEST_ASSERT(pool.Acquire(d) == LibXR::ErrorCode::OK);
    TEST_ASSERT(d.Index() == a_index);
  }

  // External queue should also be supported.
  {
    using FreeQueue = LibXR::Queue<uint32_t>;
    RunExternalQueueChecks<FreeQueue>();
  }

  // External slot storage should also be supported.
  {
    using FreeQueue = LibXR::Queue<uint32_t>;
    RunExternalSlotChecks<FreeQueue>();
  }

  // External queue + external slot storage should both work together.
  {
    using FreeQueue = LibXR::Queue<uint32_t>;
    RunExternalQueueAndSlotChecks<FreeQueue>();
  }

  // RAII release through destructor should return the slot automatically.
  {
    LibXR::ObjectPool<Payload> pool(2);

    {
      LibXR::ObjectPool<Payload>::Handle handle;
      TEST_ASSERT(pool.Acquire(handle) == LibXR::ErrorCode::OK);
      handle->value = 77;
      TEST_ASSERT(pool.EmptySize() == 1);
    }

    TEST_ASSERT(pool.EmptySize() == 2);
  }

  // Moving the only handle transfers the final-return responsibility.
  {
    LibXR::ObjectPool<Payload> pool(1);

    LibXR::ObjectPool<Payload>::Handle first;
    TEST_ASSERT(pool.Acquire(first) == LibXR::ErrorCode::OK);
    TEST_ASSERT(pool.EmptySize() == 0);

    auto second = std::move(first);
    TEST_ASSERT(!first.Valid());
    TEST_ASSERT(second.Valid());
    second->value = 99;
    TEST_ASSERT(pool.UnsafeAt(second.Index()).value == 99);

    second.Reset();
    TEST_ASSERT(pool.EmptySize() == 1);
  }
}
