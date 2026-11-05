/**
 * @file test_spsc_queue.cpp
 * @brief 检查 SPSC 队列操作、批量拷贝、回调失败和生产消费顺序。 /
 * Tests SPSC queue operations, batch copies, callback failures and producer-consumer
 * order.
 *
 * 暂停尚未发布的写入后执行 Reset，检查旧数据被丢弃且新数据仍可读。
 * Pauses an unpublished write across Reset; old data is discarded and new data survives.
 */

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <type_traits>

#include "libxr.hpp"
#include "libxr_def.hpp"
#include "test.hpp"
#include "test_assert.hpp"

namespace
{
using Queue = LibXR::SPSCQueue<uint32_t>;

struct NoDefaultPayload
{
  explicit NoDefaultPayload(uint32_t value_in) : value(value_in) {}

  uint32_t value;
};

struct ProducerArg
{
  Queue* queue;
  uint32_t total_items;
  std::atomic<bool>* producer_done;
};

struct ResetProducerArg
{
  Queue* queue;
  std::atomic<bool>* ready;
  std::atomic<bool>* resume;
};

void ProducerTask(ProducerArg arg)
{
  for (uint32_t value = 0; value < arg.total_items; ++value)
  {
    while (arg.queue->Push(value) != LibXR::ErrorCode::OK)
    {
      LibXR::Thread::Yield();
    }
  }
  arg.producer_done->store(true, std::memory_order_release);
}

void ResetProducerTask(ResetProducerArg arg)
{
  const size_t produced = arg.queue->ProduceWithWriter(
      1U,
      [&](uint32_t* first, size_t first_size, uint32_t*, size_t second_size)
      {
        TEST_ASSERT(first_size == 1U);
        TEST_ASSERT(second_size == 0U);
        first[0] = 99U;
        arg.ready->store(true, std::memory_order_release);
        while (!arg.resume->load(std::memory_order_acquire))
        {
          LibXR::Thread::Yield();
        }
        return 1U;
      });
  TEST_ASSERT(produced == 1U);
}

// Moves a full queue through the ring from every start index, so the batch copy splits
// at every possible point. Each half pairs a batch call with single-payload calls, which
// do not use the batch copy path.
void CheckWrappedBatch(size_t element_size, size_t element_align)
{
  constexpr size_t CAPACITY = 5;
  uint8_t input[CAPACITY * 4] = {};
  for (size_t index = 0; index < sizeof(input); ++index)
  {
    input[index] = static_cast<uint8_t>(index + 1U);
  }
  const size_t bytes = CAPACITY * element_size;

  for (size_t start = 0; start <= CAPACITY; ++start)
  {
    LibXR::SPSCQueueBase queue(element_size, element_align, CAPACITY);
    uint8_t output[sizeof(input)] = {};
    for (size_t index = 0; index < start; ++index)
    {
      TEST_ASSERT(queue.PushBytes(input) == LibXR::ErrorCode::OK);
      TEST_ASSERT(queue.PopBytes(output) == LibXR::ErrorCode::OK);
    }

    // Batch in, single out.
    TEST_ASSERT(queue.PushBatchBytes(input, CAPACITY) == LibXR::ErrorCode::OK);
    TEST_ASSERT(queue.PushBatchBytes(input, 1) == LibXR::ErrorCode::FULL);
    std::memset(output, 0, sizeof(output));
    TEST_ASSERT(queue.PeekBatchBytes(output, CAPACITY) == LibXR::ErrorCode::OK);
    TEST_ASSERT(std::memcmp(output, input, bytes) == 0);
    std::memset(output, 0, sizeof(output));
    for (size_t index = 0; index < CAPACITY; ++index)
    {
      TEST_ASSERT(queue.PopBytes(output + index * element_size) == LibXR::ErrorCode::OK);
    }
    TEST_ASSERT(std::memcmp(output, input, bytes) == 0);

    // Single in, batch out.
    for (size_t index = 0; index < CAPACITY; ++index)
    {
      TEST_ASSERT(queue.PushBytes(input + index * element_size) == LibXR::ErrorCode::OK);
    }
    std::memset(output, 0, sizeof(output));
    TEST_ASSERT(queue.PopBatchBytes(output, CAPACITY) == LibXR::ErrorCode::OK);
    TEST_ASSERT(queue.PopBatchBytes(output, 1) == LibXR::ErrorCode::EMPTY);
    TEST_ASSERT(std::memcmp(output, input, bytes) == 0);
  }
}

struct BatchProducerArg
{
  LibXR::SPSCQueue<uint8_t>* queue;
  uint32_t total;
};

// Pushes 0, 1, 2, ... (mod 256) in batches of 1 to 7 bytes.
void BatchProducerTask(BatchProducerArg arg)
{
  uint8_t batch[7] = {};
  for (uint32_t sent = 0, size = 1; sent < arg.total; sent += size, size = size % 7U + 1U)
  {
    size = LibXR::min(size, arg.total - sent);
    for (uint32_t index = 0; index < size; ++index)
    {
      batch[index] = static_cast<uint8_t>(sent + index);
    }
    while (arg.queue->PushBatch(batch, size) != LibXR::ErrorCode::OK)
    {
      LibXR::Thread::Yield();
    }
  }
}
}  // namespace

static_assert(!std::is_default_constructible_v<NoDefaultPayload>);
static_assert(std::is_trivially_copyable_v<NoDefaultPayload>);

void test_spsc_queue()
{
  // Batch copies that wrap at every split point: bytes, 4-byte payloads, and padded
  // slots (payload size not a multiple of its alignment).
  CheckWrappedBatch(1, 1);
  CheckWrappedBatch(4, 4);
  CheckWrappedBatch(3, 4);

  // A batch popped into a null buffer is discarded.
  {
    LibXR::SPSCQueue<uint8_t> queue(4);
    const uint8_t data[3] = {1, 2, 3};
    TEST_ASSERT(queue.PushBatch(data, 3) == LibXR::ErrorCode::OK);
    TEST_ASSERT(queue.PopBatch(nullptr, 3) == LibXR::ErrorCode::OK);
    TEST_ASSERT(queue.Size() == 0);
  }

  // Batches pushed and popped by two threads keep their order across the ring end.
  {
    constexpr uint32_t TOTAL = 100000;
    LibXR::SPSCQueue<uint8_t> queue(13);
    LibXR::Thread producer;
    producer.Create<BatchProducerArg>(BatchProducerArg{&queue, TOTAL}, BatchProducerTask,
                                      "spsc_batch_prod", 1024,
                                      LibXR::Thread::Priority::REALTIME);

    uint8_t batch[5] = {};
    for (uint32_t received = 0, size = 1; received < TOTAL;)
    {
      size = LibXR::min(size, TOTAL - received);
      if (queue.PopBatch(batch, size) != LibXR::ErrorCode::OK)
      {
        LibXR::Thread::Yield();
        continue;
      }
      for (uint32_t index = 0; index < size; ++index)
      {
        TEST_ASSERT(batch[index] == static_cast<uint8_t>(received + index));
      }
      received += size;
      size = size % 5U + 1U;
    }
    TEST_ASSERT(producer.Join() == LibXR::ErrorCode::OK);
  }

  // Basic push/pop/peek behavior on a tiny queue.
  {
    Queue queue(4);
    uint32_t value = 0;

    TEST_ASSERT(queue.MaxSize() == 4);
    TEST_ASSERT(queue.Size() == 0);
    TEST_ASSERT(queue.EmptySize() == 4);
    TEST_ASSERT(queue.Pop(value) == LibXR::ErrorCode::EMPTY);

    TEST_ASSERT(queue.Push(11) == LibXR::ErrorCode::OK);
    TEST_ASSERT(queue.Push(22) == LibXR::ErrorCode::OK);
    TEST_ASSERT(queue.Peek(value) == LibXR::ErrorCode::OK);
    TEST_ASSERT(value == 11);
    TEST_ASSERT(queue.Pop(value) == LibXR::ErrorCode::OK);
    TEST_ASSERT(value == 11);
    TEST_ASSERT(queue.Pop(value) == LibXR::ErrorCode::OK);
    TEST_ASSERT(value == 22);
    TEST_ASSERT(queue.Pop(value) == LibXR::ErrorCode::EMPTY);
  }

  // Capacity 1 is valid and zero-length batches are no-ops.
  {
    Queue queue(1);
    uint32_t value = 0;
    uint32_t batch[1] = {33};

    TEST_ASSERT(queue.PushBatch(nullptr, 0) == LibXR::ErrorCode::OK);
    TEST_ASSERT(queue.PopBatch(nullptr, 0) == LibXR::ErrorCode::OK);
    TEST_ASSERT(queue.PeekBatch(nullptr, 0) == LibXR::ErrorCode::OK);

    TEST_ASSERT(queue.Push(11) == LibXR::ErrorCode::OK);
    TEST_ASSERT(queue.Push(22) == LibXR::ErrorCode::FULL);
    TEST_ASSERT(queue.Peek(value) == LibXR::ErrorCode::OK);
    TEST_ASSERT(value == 11);
    TEST_ASSERT(queue.Pop(value) == LibXR::ErrorCode::OK);
    TEST_ASSERT(value == 11);
    TEST_ASSERT(queue.Pop(value) == LibXR::ErrorCode::EMPTY);

    TEST_ASSERT(queue.PushBatch(batch, 1) == LibXR::ErrorCode::OK);
    TEST_ASSERT(queue.PopBatch(&value, 1) == LibXR::ErrorCode::OK);
    TEST_ASSERT(value == 33);
  }

  // Batch APIs, wraparound, writer callback, and reset behavior.
  {
    Queue queue(5);
    uint32_t initial[5] = {1, 2, 3, 4, 5};
    uint32_t readback[5] = {};

    TEST_ASSERT(queue.PushBatch(initial, 5) == LibXR::ErrorCode::OK);
    TEST_ASSERT(queue.PushBatch(initial, 1) == LibXR::ErrorCode::FULL);

    TEST_ASSERT(queue.PeekBatch(readback, 5) == LibXR::ErrorCode::OK);
    for (size_t index = 0; index < 5; ++index)
    {
      TEST_ASSERT(readback[index] == initial[index]);
    }

    TEST_ASSERT(queue.PopBatch(readback, 2) == LibXR::ErrorCode::OK);
    TEST_ASSERT(readback[0] == 1);
    TEST_ASSERT(readback[1] == 2);

    uint32_t wrap[2] = {6, 7};
    TEST_ASSERT(queue.PushBatch(wrap, 2) == LibXR::ErrorCode::OK);

    uint32_t expected[5] = {3, 4, 5, 6, 7};
    TEST_ASSERT(queue.PeekBatch(readback, 5) == LibXR::ErrorCode::OK);
    for (size_t index = 0; index < 5; ++index)
    {
      TEST_ASSERT(readback[index] == expected[index]);
    }

    size_t write_cursor = 100;
    queue.Reset();
    TEST_ASSERT(queue.Size() == 0);
    for (size_t index = 0; index < 3; ++index)
    {
      TEST_ASSERT(queue.PushWithWriter(
                      [&](uint32_t* slot, size_t count)
                      {
                        TEST_ASSERT(count == 1);
                        slot[0] = static_cast<uint32_t>(write_cursor++);
                        return LibXR::ErrorCode::OK;
                      }) == LibXR::ErrorCode::OK);
    }

    uint32_t expected_value = 100;
    for (size_t index = 0; index < 3; ++index)
    {
      TEST_ASSERT(queue.PopWithReader(
                      [&](const uint32_t* slot, size_t count)
                      {
                        TEST_ASSERT(count == 1);
                        TEST_ASSERT(slot[0] == expected_value++);
                        return LibXR::ErrorCode::OK;
                      }) == LibXR::ErrorCode::OK);
    }
    uint32_t value = 0;
    TEST_ASSERT(queue.Pop(value) == LibXR::ErrorCode::EMPTY);
  }

  // Normal Push/Pop remain byte-payload operations and do not require Data{}.
  {
    LibXR::SPSCQueue<NoDefaultPayload> queue(2);
    NoDefaultPayload pushed(88);
    NoDefaultPayload popped(0);

    TEST_ASSERT(queue.Push(pushed) == LibXR::ErrorCode::OK);
    TEST_ASSERT(queue.Pop(popped) == LibXR::ErrorCode::OK);
    TEST_ASSERT(popped.value == 88);
  }

  // Hold a producer after it writes an uncommitted slot, then reset from the consumer.
  {
    Queue queue(4);
    TEST_ASSERT(queue.Push(10U) == LibXR::ErrorCode::OK);
    TEST_ASSERT(queue.Push(11U) == LibXR::ErrorCode::OK);

    uint32_t value = 0U;
    TEST_ASSERT(queue.Pop(value) == LibXR::ErrorCode::OK);
    TEST_ASSERT(value == 10U);

    std::atomic<bool> producer_ready = false;
    std::atomic<bool> resume_producer = false;
    LibXR::Thread producer;
    producer.Create<ResetProducerArg>(
        ResetProducerArg{&queue, &producer_ready, &resume_producer}, ResetProducerTask,
        "spsc_reset", 1024, LibXR::Thread::Priority::REALTIME);

    while (!producer_ready.load(std::memory_order_acquire))
    {
      LibXR::Thread::Yield();
    }
    queue.Reset();
    resume_producer.store(true, std::memory_order_release);

    TEST_ASSERT(producer.Join() == LibXR::ErrorCode::OK);
    TEST_ASSERT(queue.Size() == 1U);
    TEST_ASSERT(queue.Pop(value) == LibXR::ErrorCode::OK);
    TEST_ASSERT(value == 99U);
    TEST_ASSERT(queue.Pop(value) == LibXR::ErrorCode::EMPTY);
  }

  // Callback failures must not commit partially produced or consumed elements.
  {
    Queue queue(2);
    uint32_t value = 0;
    bool writer_called = false;

    TEST_ASSERT(queue.PushWithWriter(
                    [](uint32_t* slot, size_t count)
                    {
                      TEST_ASSERT(count == 1);
                      slot[0] = 42;
                      return LibXR::ErrorCode::FAILED;
                    }) == LibXR::ErrorCode::FAILED);
    TEST_ASSERT(queue.Size() == 0);

    TEST_ASSERT(queue.Push(1) == LibXR::ErrorCode::OK);
    TEST_ASSERT(queue.Push(2) == LibXR::ErrorCode::OK);
    TEST_ASSERT(queue.PushWithWriter(
                    [&](uint32_t* slot, size_t count)
                    {
                      UNUSED(slot);
                      UNUSED(count);
                      writer_called = true;
                      return LibXR::ErrorCode::OK;
                    }) == LibXR::ErrorCode::FULL);
    TEST_ASSERT(!writer_called);
    TEST_ASSERT(queue.Pop() == LibXR::ErrorCode::OK);
    TEST_ASSERT(queue.Pop() == LibXR::ErrorCode::OK);

    TEST_ASSERT(queue.Push(55) == LibXR::ErrorCode::OK);
    TEST_ASSERT(queue.PopWithReader(
                    [](const uint32_t* slot, size_t count)
                    {
                      TEST_ASSERT(count == 1);
                      TEST_ASSERT(slot[0] == 55);
                      return LibXR::ErrorCode::FAILED;
                    }) == LibXR::ErrorCode::FAILED);
    TEST_ASSERT(queue.Pop(value) == LibXR::ErrorCode::OK);
    TEST_ASSERT(value == 55);
  }

  // Batched byte callbacks receive contiguous chunks even when the ring wraps.
  {
    LibXR::SPSCQueue<uint8_t> byte_queue(6);
    uint8_t value = 0;
    const uint8_t initial[5] = {10, 11, 12, 13, 14};

    TEST_ASSERT(byte_queue.PushBatch(initial, 5) == LibXR::ErrorCode::OK);
    for (uint8_t expected = 10; expected < 13; ++expected)
    {
      TEST_ASSERT(byte_queue.Pop(value) == LibXR::ErrorCode::OK);
      TEST_ASSERT(value == expected);
    }

    uint8_t write_next = 20;
    size_t write_chunks = 0;
    TEST_ASSERT(byte_queue.PushWithWriter(4,
                                          [&](uint8_t* chunk, size_t count)
                                          {
                                            TEST_ASSERT(count == 2);
                                            ++write_chunks;
                                            for (size_t index = 0; index < count; ++index)
                                            {
                                              chunk[index] = write_next++;
                                            }
                                            return LibXR::ErrorCode::OK;
                                          }) == LibXR::ErrorCode::OK);
    TEST_ASSERT(write_chunks == 2);

    const uint8_t expected_after_write[6] = {13, 14, 20, 21, 22, 23};
    uint8_t readback[6] = {};
    TEST_ASSERT(byte_queue.PopBatch(readback, 6) == LibXR::ErrorCode::OK);
    for (size_t index = 0; index < 6; ++index)
    {
      TEST_ASSERT(readback[index] == expected_after_write[index]);
    }

    const uint8_t full[6] = {1, 2, 3, 4, 5, 6};
    const uint8_t wrap[4] = {7, 8, 9, 10};
    TEST_ASSERT(byte_queue.PushBatch(full, 6) == LibXR::ErrorCode::OK);
    for (size_t index = 0; index < 4; ++index)
    {
      TEST_ASSERT(byte_queue.Pop(value) == LibXR::ErrorCode::OK);
      TEST_ASSERT(value == static_cast<uint8_t>(index + 1));
    }
    TEST_ASSERT(byte_queue.PushBatch(wrap, 4) == LibXR::ErrorCode::OK);

    const uint8_t expected_read_chunks[5] = {5, 6, 7, 8, 9};
    size_t read_cursor = 0;
    size_t read_chunks = 0;
    TEST_ASSERT(byte_queue.PopWithReader(
                    5,
                    [&](const uint8_t* chunk, size_t count)
                    {
                      TEST_ASSERT((read_chunks == 0 && count == 1) ||
                                  (read_chunks == 1 && count == 4));
                      for (size_t index = 0; index < count; ++index)
                      {
                        TEST_ASSERT(chunk[index] == expected_read_chunks[read_cursor++]);
                      }
                      ++read_chunks;
                      return LibXR::ErrorCode::OK;
                    }) == LibXR::ErrorCode::OK);
    TEST_ASSERT(read_chunks == 2);
    TEST_ASSERT(read_cursor == 5);
    TEST_ASSERT(byte_queue.Pop(value) == LibXR::ErrorCode::OK);
    TEST_ASSERT(value == 10);
    TEST_ASSERT(byte_queue.Pop(value) == LibXR::ErrorCode::EMPTY);
  }

  // End-to-end producer/consumer handoff under sustained contention.
  {
    constexpr uint32_t TOTAL_ITEMS = 50000;
    Queue queue(8);
    std::atomic<bool> producer_done = false;
    LibXR::Thread producer;

    producer.Create<ProducerArg>(ProducerArg{&queue, TOTAL_ITEMS, &producer_done},
                                 ProducerTask, "spsc_prod", 1024,
                                 LibXR::Thread::Priority::REALTIME);

    for (uint32_t expected = 0; expected < TOTAL_ITEMS; ++expected)
    {
      uint32_t value = UINT32_MAX;
      while (queue.Pop(value) != LibXR::ErrorCode::OK)
      {
        LibXR::Thread::Yield();
      }
      TEST_ASSERT(value == expected);
    }

    while (!producer_done.load(std::memory_order_acquire))
    {
      LibXR::Thread::Yield();
    }
    TEST_ASSERT(producer.Join() == LibXR::ErrorCode::OK);
    uint32_t value = 0;
    TEST_ASSERT(queue.Pop(value) == LibXR::ErrorCode::EMPTY);
    TEST_ASSERT(queue.Size() == 0);
  }
}
