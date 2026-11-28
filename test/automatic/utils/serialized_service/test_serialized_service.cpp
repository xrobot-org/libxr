/**
 * @file test_serialized_service.cpp
 * @brief 检查事件合并、递归请求和并发调用的串行处理。 /
 * Tests event coalescing, recursive requests and serialized concurrent calls.
 *
 * 检查竞争调用的临时回调不会被保留，以及事件发布后的数据可见性。
 * Checks transient callable lifetime and payload visibility after event publication.
 */

#include <array>
#include <atomic>
#include <cstdint>
#include <semaphore>
#include <thread>
#include <type_traits>
#include <vector>

#include "serialized_service.hpp"
#include "slot_pool.hpp"
#include "test.hpp"
#include "test_assert.hpp"

namespace
{
constexpr uint32_t FIRST = 1U;
constexpr uint32_t SECOND = 2U;
constexpr uint32_t THIRD = 4U;

static_assert(sizeof(LibXR::SerializedService) == sizeof(std::atomic<uint32_t>));
static_assert(!std::is_copy_constructible_v<LibXR::SerializedService>);

void TestDeferredAndRecursiveEvents()
{
  // 先发布的事件由下一次 Invoke 处理；回调中再次请求执行也不能递归进入回调。
  // The next Invoke handles queued events; invoking again inside the callback must not
  // recurse.
  LibXR::SerializedService service;
  service.Publish(0U);
  service.Publish(SECOND);
  service.Publish(SECOND);
  unsigned calls = 0U;
  unsigned depth = 0U;
  TEST_ASSERT(service.Invoke(FIRST, true,
                             [&](uint32_t events, bool in_isr)
                             {
                               TEST_ASSERT(++depth == 1U);
                               TEST_ASSERT(in_isr);
                               if (calls++ == 0U)
                               {
                                 TEST_ASSERT(events == (FIRST | SECOND));
                                 TEST_ASSERT(!service.Invoke(THIRD, false,
                                                             [](uint32_t, bool)
                                                             { TEST_ASSERT(false); }));
                                 service.Publish(THIRD);
                               }
                               else
                               {
                                 TEST_ASSERT(events == THIRD);
                               }
                               --depth;
                             }));
  TEST_ASSERT(calls == 2U && depth == 0U);
  TEST_ASSERT(service.Invoke(FIRST, false, [](uint32_t events, bool in_isr)
                             { TEST_ASSERT(events == FIRST && !in_isr); }));
}

void TestCompetingCallableLifetime()
{
  // 暂停正在执行的回调，另一线程提交事件后销毁临时捕获对象；后续仍由原回调处理。
  // Pause the active callback, submit from a temporary callable, then let the original
  // callback handle it.
  LibXR::SerializedService service;
  std::binary_semaphore entered(0);
  std::binary_semaphore resume(0);
  unsigned owner_calls = 0U;
  int payload = 0;
  std::thread owner(
      [&]
      {
        TEST_ASSERT(service.Invoke(FIRST, false,
                                   [&](uint32_t events, bool in_isr)
                                   {
                                     TEST_ASSERT(events != 0U);
                                     TEST_ASSERT(!in_isr);
                                     if (owner_calls++ == 0U)
                                     {
                                       TEST_ASSERT(events == FIRST);
                                       entered.release();
                                       resume.acquire();
                                     }
                                     else
                                     {
                                       TEST_ASSERT(events == (SECOND | THIRD));
                                       TEST_ASSERT(payload == 42);
                                     }
                                   }));
      });
  // 确认第一个回调已停住，再提交竞争调用，使临时对象的销毁早于后续处理。
  // Wait until the first callback is paused, then submit so the temporary dies before
  // later handling.
  entered.acquire();
  payload = 42;
  {
    int transient_capture = 7;
    TEST_ASSERT(
        !service.Invoke(SECOND, true, [&](uint32_t, bool) { transient_capture = 8; }));
    TEST_ASSERT(transient_capture == 7);
  }
  service.Publish(THIRD);
  resume.release();
  owner.join();
  TEST_ASSERT(owner_calls == 2U);
}

void TestConcurrentReleaseAndPayloadPublication()
{
  // 每个生产者等上一条数据被处理后再写下一条；处理回调之间不能重叠。
  // Each producer waits for its prior value to be handled; handler executions must not
  // overlap.
  constexpr unsigned PRODUCERS = 4U;
  constexpr uint32_t ITERATIONS = 5000U;
  LibXR::SerializedService service;
  std::array<uint32_t, PRODUCERS> payload{};
  std::array<uint32_t, PRODUCERS> consumed{};
  std::array<std::binary_semaphore, PRODUCERS> acknowledged{
      std::binary_semaphore{0}, std::binary_semaphore{0}, std::binary_semaphore{0},
      std::binary_semaphore{0}};
  std::array<std::thread, PRODUCERS> threads;
  std::atomic<unsigned> active{0U};
  auto handler = [&](uint32_t events, bool)
  {
    TEST_ASSERT(events != 0U);
    TEST_ASSERT(active.fetch_add(1U, std::memory_order_relaxed) == 0U);
    for (unsigned i = 0U; i < PRODUCERS; ++i)
    {
      if ((events & (1U << i)) != 0U)
      {
        // 普通数据通过 service 事件发布；这里没有额外的锁替它保证可见性。
        // The service event publishes this plain payload; no extra lock supplies
        // visibility.
        TEST_ASSERT(payload[i] == consumed[i] + 1U);
        consumed[i] = payload[i];
        acknowledged[i].release();
      }
    }
    TEST_ASSERT(active.fetch_sub(1U, std::memory_order_relaxed) == 1U);
  };
  for (unsigned i = 0U; i < PRODUCERS; ++i)
  {
    threads[i] = std::thread(
        [&, i]
        {
          for (uint32_t sequence = 1U; sequence <= ITERATIONS; ++sequence)
          {
            payload[i] = sequence;
            service.Invoke(1U << i, false, handler);
            acknowledged[i].acquire();
          }
        });
  }
  for (auto& thread : threads)
  {
    thread.join();
  }
  for (uint32_t count : consumed)
  {
    TEST_ASSERT(count == ITERATIONS);
  }
  TEST_ASSERT(active.load(std::memory_order_relaxed) == 0U);
}

void TestCanTxSerializedServicePattern()
{
  // 模拟 CAN/FDCAN 发送服务模型：验证 FD 优先、邮箱满重试槽、完成中断排空以及
  // bus-off/重配置后的恢复 kick 语义。
  // Model the CAN/FDCAN TX service pattern: FD priority, full-mailbox retry slots,
  // completion-IRQ drain, and bus-off / reconfig restart kicks.
  constexpr uint32_t TX_EVENT = 1U;
  struct MockCanTx
  {
    LibXR::SerializedService tx_service;
    LibXR::SlotPool<uint32_t> tx_fd_queue{8};
    LibXR::SlotPool<uint32_t> tx_classic_queue{8};
    uint32_t tx_fd_retry_pack = 0U;
    bool tx_fd_retry_valid = false;
    uint32_t tx_classic_retry_pack = 0U;
    bool tx_classic_retry_valid = false;
    std::atomic<uint32_t> hw_free_slots{0U};
    std::atomic<uint32_t> bus_off{0U};
    std::atomic<uint32_t> active_owners{0U};
    std::vector<uint32_t> sent_order;

    bool TryHwSubmit(uint32_t pack)
    {
      if (bus_off.load(std::memory_order_acquire) != 0U)
      {
        return false;
      }
      uint32_t free_slots = hw_free_slots.load(std::memory_order_acquire);
      while (free_slots > 0U)
      {
        if (hw_free_slots.compare_exchange_weak(free_slots, free_slots - 1U,
                                                std::memory_order_acq_rel,
                                                std::memory_order_acquire))
        {
          sent_order.push_back(pack);
          return true;
        }
      }
      return false;
    }

    void TxService()
    {
      tx_service.Invoke(
          TX_EVENT, false,
          [this](uint32_t, bool)
          {
            TEST_ASSERT(active_owners.fetch_add(1U, std::memory_order_relaxed) == 0U);
            while (hw_free_slots.load(std::memory_order_acquire) != 0U)
            {
              uint32_t fd_pack = 0U;
              bool has_fd = tx_fd_retry_valid;
              if (has_fd)
              {
                fd_pack = tx_fd_retry_pack;
              }
              else
              {
                has_fd = (tx_fd_queue.Get(fd_pack) == LibXR::ErrorCode::OK);
              }
              if (has_fd)
              {
                if (!TryHwSubmit(fd_pack))
                {
                  tx_fd_retry_pack = fd_pack;
                  tx_fd_retry_valid = true;
                  break;
                }
                tx_fd_retry_valid = false;
                continue;
              }

              uint32_t classic_pack = 0U;
              bool has_classic = tx_classic_retry_valid;
              if (has_classic)
              {
                classic_pack = tx_classic_retry_pack;
              }
              else
              {
                has_classic =
                    (tx_classic_queue.Get(classic_pack) == LibXR::ErrorCode::OK);
              }
              if (has_classic)
              {
                if (!TryHwSubmit(classic_pack))
                {
                  tx_classic_retry_pack = classic_pack;
                  tx_classic_retry_valid = true;
                  break;
                }
                tx_classic_retry_valid = false;
                continue;
              }
              break;
            }
            TEST_ASSERT(active_owners.fetch_sub(1U, std::memory_order_relaxed) == 1U);
          });
    }
  };

  // 1. FD 优先 + 邮箱满转入重试槽 + 完成中断续发 + bus-off 恢复。
  // FD priority, full-mailbox retry slot, completion-IRQ drain, and bus-off recovery.
  {
    MockCanTx can;
    TEST_ASSERT(can.tx_classic_queue.Put(100U) == LibXR::ErrorCode::OK);
    TEST_ASSERT(can.tx_classic_queue.Put(101U) == LibXR::ErrorCode::OK);
    TEST_ASSERT(can.tx_fd_queue.Put(200U) == LibXR::ErrorCode::OK);
    TEST_ASSERT(can.tx_fd_queue.Put(201U) == LibXR::ErrorCode::OK);

    // 硬件仅 1 槽且处于 bus-off：首帧 FD 进入重试槽，不丢帧。
    // Hardware reports 1 free slot while bus-off: first FD frame parks in retry slot.
    can.hw_free_slots.store(1U, std::memory_order_release);
    can.bus_off.store(1U, std::memory_order_release);
    can.TxService();
    TEST_ASSERT(can.sent_order.empty());
    TEST_ASSERT(can.tx_fd_retry_valid && can.tx_fd_retry_pack == 200U);

    // 恢复 bus-off 并重触发：优先发 200（仅 1 槽）。
    // Clear bus-off and restart kick: drains 200 into the single free slot.
    can.bus_off.store(0U, std::memory_order_release);
    can.TxService();
    TEST_ASSERT(can.sent_order.size() == 1U && can.sent_order[0] == 200U);
    TEST_ASSERT(!can.tx_fd_retry_valid);

    // 完成中断释放 3 槽：先发剩余 FD 201，再按序发 classic 100、101。
    // Completion IRQ frees 3 slots: sends FD 201 first, then classic 100 and 101.
    can.hw_free_slots.fetch_add(3U, std::memory_order_acq_rel);
    can.TxService();
    TEST_ASSERT(can.sent_order.size() == 4U);
    TEST_ASSERT(can.sent_order[0] == 200U && can.sent_order[1] == 201U &&
                can.sent_order[2] == 100U && can.sent_order[3] == 101U);
  }

  // 2. 并发入队 kick 与完成中断交错：全部帧必须发出且不重叠。
  // Concurrent producer kicks interleaved with completion-IRQ kicks drain every frame.
  {
    constexpr uint32_t TOTAL_FRAMES = 500U;
    MockCanTx can;
    std::atomic<uint32_t> producer_done{0U};
    std::thread irq_sim(
        [&]
        {
          while (producer_done.load(std::memory_order_acquire) == 0U ||
                 can.sent_order.size() < TOTAL_FRAMES)
          {
            if (can.hw_free_slots.load(std::memory_order_acquire) < 3U)
            {
              can.hw_free_slots.fetch_add(1U, std::memory_order_acq_rel);
            }
            can.TxService();
            std::this_thread::yield();
          }
        });

    for (uint32_t seq = 1U; seq <= TOTAL_FRAMES; ++seq)
    {
      while (can.tx_classic_queue.Put(seq) != LibXR::ErrorCode::OK)
      {
        can.TxService();
        std::this_thread::yield();
      }
      can.TxService();
    }
    producer_done.store(1U, std::memory_order_release);
    irq_sim.join();
    TEST_ASSERT(can.sent_order.size() == TOTAL_FRAMES);
    for (uint32_t i = 0U; i < TOTAL_FRAMES; ++i)
    {
      TEST_ASSERT(can.sent_order[i] == i + 1U);
    }
  }
}
}  // namespace

void test_serialized_service()
{
  TestDeferredAndRecursiveEvents();
  TestCompetingCallableLifetime();
  TestConcurrentReleaseAndPayloadPublication();
  TestCanTxSerializedServicePattern();
}
