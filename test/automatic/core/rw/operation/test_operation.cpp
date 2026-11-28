/**
 * @file test_operation.cpp
 * @brief AsyncBlockWait 超时分离与已认领通知回收测试 /
 *        Tests for AsyncBlockWait timeout detachment and claimed-notification draining.
 */

#include <atomic>
#include <thread>

#include "libxr.hpp"
#include "test.hpp"
#include "test_assert.hpp"

void test_operation()
{
  using namespace LibXR;

  Semaphore sem(0U);
  AsyncBlockWait wait;

  // 1. 正常完成：TryPost 先于超时返回结果，且重复复用保持干净。
  // Normal completion: TryPost wakes Wait before timeout and state returns to IDLE.
  for (int i = 0; i < 4; ++i)
  {
    const ErrorCode expected = (i % 2 == 0) ? ErrorCode::OK : ErrorCode::FAILED;
    wait.Start(sem);
    std::thread completer(
        [&]
        {
          Thread::Sleep(1U);
          TEST_ASSERT(wait.TryPost(false, expected));
        });
    TEST_ASSERT(wait.Wait(500U) == expected);
    completer.join();
    TEST_ASSERT(!wait.TryPost(false, ErrorCode::OK));
    TEST_ASSERT(sem.Wait(0U) == ErrorCode::TIMEOUT);
  }

  // 2. 超时分离：Wait(0) 先转入 DETACHED，迟到的 TryPost 返回 false 且不泄漏信号量。
  // Timeout detachment: Wait(0) detaches, and a late TryPost returns false without
  // leaking a semaphore token.
  wait.Start(sem);
  TEST_ASSERT(wait.Wait(0U) == ErrorCode::TIMEOUT);
  TEST_ASSERT(!wait.TryPost(false, ErrorCode::OK));
  TEST_ASSERT(sem.Wait(0U) == ErrorCode::TIMEOUT);

  // 3. 边界竞争：Wait(0) 与 TryPost 并发竞争；若 TryPost 认领成功，Wait 必须通过
  // WaitClaimedNotification 回收通知并返回真实错误码；若先分离，TryPost 返回 false。
  // Boundary race between Wait(0) and TryPost: either TryPost claims and Wait drains
  // and returns the completion result, or Wait detaches first and TryPost returns false.
  for (int round = 0; round < 200; ++round)
  {
    const ErrorCode expected = ((round & 1) != 0) ? ErrorCode::FAILED : ErrorCode::OK;
    wait.Start(sem);
    std::atomic<uint32_t> start_wait{0U};
    std::atomic<uint32_t> posted{0U};
    std::thread completer(
        [&]
        {
          while (start_wait.load(std::memory_order_acquire) == 0U)
          {
            std::this_thread::yield();
          }
          posted.store(wait.TryPost(false, expected) ? 1U : 0U,
                       std::memory_order_release);
        });

    start_wait.store(1U, std::memory_order_release);
    const ErrorCode ans = wait.Wait(0U);
    completer.join();
    if (posted.load(std::memory_order_acquire) != 0U)
    {
      TEST_ASSERT(ans == expected);
    }
    else
    {
      TEST_ASSERT(ans == ErrorCode::TIMEOUT);
    }
    TEST_ASSERT(sem.Wait(0U) == ErrorCode::TIMEOUT);
  }

  // 4. 确定性已认领回收：TryPost 完成前 sem 已计数，随后下一次 Start/Wait 仍可复用。
  // Deterministic post-then-wait and reuse after claimed completion.
  wait.Start(sem);
  TEST_ASSERT(wait.TryPost(true, ErrorCode::FAILED));
  TEST_ASSERT(wait.Wait(0U) == ErrorCode::FAILED);
  TEST_ASSERT(sem.Wait(0U) == ErrorCode::TIMEOUT);

  wait.Start(sem);
  TEST_ASSERT(wait.TryPost(false, ErrorCode::OK));
  TEST_ASSERT(wait.Wait(10U) == ErrorCode::OK);
  TEST_ASSERT(sem.Wait(0U) == ErrorCode::TIMEOUT);
}
