/**
 * @file test_latest_snapshot.cpp
 * @brief 检查 LatestSnapshot 的发布、覆盖和单生产者单消费者并发边界。 /
 * Tests LatestSnapshot publication, overwrite and single-producer single-consumer
 * concurrency boundaries.
 *
 * 并发测试检查消费者读到的值完整且序号不回退，阻塞拷贝测试检查正在读的槽不被覆盖。
 * The concurrent case checks that values read are complete and never go backwards; the
 * blocking-copy case checks that the slot being read is not overwritten.
 */

#include <atomic>
#include <chrono>
#include <cstdint>
#include <thread>

#include "latest_snapshot.hpp"
#include "test.hpp"
#include "test_assert.hpp"

namespace
{

constexpr uint32_t WAIT_TIMEOUT_MS = 2000U;

template <typename Predicate>
bool WaitUntil(Predicate&& predicate)
{
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(WAIT_TIMEOUT_MS);
  while (!predicate())
  {
    if (std::chrono::steady_clock::now() >= deadline)
    {
      return false;
    }
    std::this_thread::yield();
  }
  return true;
}

struct Snapshot
{
  uint32_t sequence = 0U;
  uint32_t inverse = ~0U;
  uint32_t payload[16]{};
};

Snapshot MakeSnapshot(uint32_t sequence)
{
  Snapshot value{};
  value.sequence = sequence;
  value.inverse = ~sequence;
  for (uint32_t& word : value.payload)
  {
    word = sequence;
  }
  return value;
}

bool IsValid(const Snapshot& value)
{
  if (value.inverse != ~value.sequence)
  {
    return false;
  }
  for (const uint32_t word : value.payload)
  {
    if (word != value.sequence)
    {
      return false;
    }
  }
  return true;
}

void TestInitialAndLatestOnlySemantics()
{
  LibXR::LatestSnapshot<Snapshot> snapshot(MakeSnapshot(0U));
  Snapshot output{};

  TEST_ASSERT(!snapshot.LoadLatest(output));
  TEST_ASSERT(IsValid(output));
  TEST_ASSERT(output.sequence == 0U);

  snapshot.Store(MakeSnapshot(1U));
  snapshot.Store(MakeSnapshot(2U));
  snapshot.Store(MakeSnapshot(3U));

  TEST_ASSERT(snapshot.LoadLatest(output));
  TEST_ASSERT(IsValid(output));
  TEST_ASSERT(output.sequence == 3U);
  TEST_ASSERT(!snapshot.LoadLatest(output));
  TEST_ASSERT(output.sequence == 3U);

  snapshot.Store(MakeSnapshot(4U));
  TEST_ASSERT(snapshot.LoadLatest(output));
  TEST_ASSERT(output.sequence == 4U);
}

struct BlockingSnapshot
{
  uint32_t sequence = 0U;
  uint32_t inverse = ~0U;

  BlockingSnapshot() = default;
  explicit BlockingSnapshot(uint32_t value) : sequence(value), inverse(~value) {}
  BlockingSnapshot(const BlockingSnapshot&) = default;

  BlockingSnapshot& operator=(const BlockingSnapshot& other)
  {
    if (this == blocked_destination &&
        block_enabled.load(std::memory_order_acquire) != 0U)
    {
      copy_entered.store(1U, std::memory_order_release);
      TEST_ASSERT(
          WaitUntil([] { return release_copy.load(std::memory_order_acquire) != 0U; }));
    }
    sequence = other.sequence;
    inverse = other.inverse;
    return *this;
  }

  static BlockingSnapshot* blocked_destination;
  static std::atomic<uint32_t> block_enabled;
  static std::atomic<uint32_t> copy_entered;
  static std::atomic<uint32_t> release_copy;
};

BlockingSnapshot* BlockingSnapshot::blocked_destination = nullptr;
std::atomic<uint32_t> BlockingSnapshot::block_enabled{0U};
std::atomic<uint32_t> BlockingSnapshot::copy_entered{0U};
std::atomic<uint32_t> BlockingSnapshot::release_copy{0U};

void TestReaderSlotSurvivesTwoFurtherStores()
{
  LibXR::LatestSnapshot<BlockingSnapshot> snapshot(BlockingSnapshot(0U));
  BlockingSnapshot output{};
  BlockingSnapshot::blocked_destination = &output;
  BlockingSnapshot::copy_entered.store(0U, std::memory_order_relaxed);
  BlockingSnapshot::release_copy.store(0U, std::memory_order_relaxed);
  BlockingSnapshot::block_enabled.store(1U, std::memory_order_release);

  snapshot.Store(BlockingSnapshot(1U));
  std::thread reader([&] { TEST_ASSERT(snapshot.LoadLatest(output)); });

  TEST_ASSERT(WaitUntil(
      []
      { return BlockingSnapshot::copy_entered.load(std::memory_order_acquire) != 0U; }));

  snapshot.Store(BlockingSnapshot(2U));
  snapshot.Store(BlockingSnapshot(3U));

  BlockingSnapshot::release_copy.store(1U, std::memory_order_release);
  reader.join();
  BlockingSnapshot::block_enabled.store(0U, std::memory_order_release);

  TEST_ASSERT(output.sequence == 1U);
  TEST_ASSERT(output.inverse == ~1U);
  TEST_ASSERT(snapshot.LoadLatest(output));
  TEST_ASSERT(output.sequence == 3U);
  TEST_ASSERT(output.inverse == ~3U);
  BlockingSnapshot::blocked_destination = nullptr;
}

void TestConcurrentPublicationStress()
{
  constexpr uint32_t publication_count = 100000U;
  LibXR::LatestSnapshot<Snapshot> snapshot(MakeSnapshot(0U));
  std::atomic<uint32_t> start{0U};
  std::atomic<uint32_t> producer_done{0U};
  std::atomic<uint32_t> failures{0U};
  std::atomic<uint32_t> last_seen{0U};

  std::thread producer(
      [&]
      {
        TEST_ASSERT(WaitUntil([&] { return start.load(std::memory_order_acquire) != 0U; }));
        for (uint32_t sequence = 1U; sequence <= publication_count; ++sequence)
        {
          snapshot.Store(MakeSnapshot(sequence));
        }
        producer_done.store(1U, std::memory_order_release);
      });

  std::thread consumer(
      [&]
      {
        Snapshot output{};
        uint32_t previous = 0U;
        start.store(1U, std::memory_order_release);

        const bool completed = WaitUntil(
            [&]
            {
              if (snapshot.LoadLatest(output))
              {
                if (!IsValid(output) || output.sequence < previous)
                {
                  failures.fetch_add(1U, std::memory_order_relaxed);
                }
                previous = output.sequence;
                last_seen.store(previous, std::memory_order_release);
              }
              return producer_done.load(std::memory_order_acquire) != 0U &&
                     previous == publication_count;
            });
        if (!completed)
        {
          failures.fetch_add(1U, std::memory_order_relaxed);
        }
      });

  producer.join();
  consumer.join();

  TEST_ASSERT(failures.load(std::memory_order_acquire) == 0U);
  TEST_ASSERT(last_seen.load(std::memory_order_acquire) == publication_count);
}

}  // namespace

void test_latest_snapshot()
{
  TestInitialAndLatestOnlySemantics();
  TestReaderSlotSurvivesTwoFurtherStores();
  TestConcurrentPublicationStress();
}
