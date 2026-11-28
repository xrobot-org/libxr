/**
 * @file test_mutex.hpp
 * @brief 实际系统的互斥锁测试 / Mutex tests on the target system.
 *
 * 由另一任务检查持锁时 TryLock 返回 BUSY，解锁后可继续；不假设同一任务的递归锁行为。
 * Check BUSY from another task and progress after unlock, without assuming same-task
 * recursion rules.
 */
#pragma once

#include <atomic>

#include "../../test_worker.hpp"
#include "mutex.hpp"

#if !defined(LIBXR_NOT_SUPPORT_MUTI_THREAD)
namespace LibXR::Test
{
/**
 * @brief 检查互斥与解锁后的任务进展 / Check exclusion and progress after unlock.
 * @param worker 独占借用的空闲 ManualWorker / Exclusively borrowed idle ManualWorker.
 * @param timeout_ms 每次协作等待的上限 / Deadline for each cooperating wait.
 * @param iterations 争用轮数，每轮两次更新 / Contention rounds, with two updates per
 * round.
 */
inline void TestMutex(ManualWorker& worker, uint32_t timeout_ms = 1000,
                      uint32_t iterations = 1000)
{
  TEST_ASSERT(worker.IsIdle());
  TEST_ASSERT(iterations > 0 && iterations <= UINT32_MAX / 2U);
  struct Context
  {
    Mutex mutex;
    Semaphore release;
    Semaphore tried;
    Semaphore finished;
    std::atomic<uint32_t> inside{0};
    std::atomic<uint32_t> writes{0};
    uint32_t timeout_ms;
    uint32_t iterations;
  } context{{}, {0}, {0}, {0}, 0, 0, timeout_ms, iterations};
  auto job = [](void* arg)
  {
    auto* context = static_cast<Context*>(arg);
    for (uint32_t round = 0; round < context->iterations; ++round)
    {
      TEST_ASSERT(context->release.Wait(context->timeout_ms) == ErrorCode::OK);
      TEST_ASSERT(context->mutex.TryLock() == ErrorCode::BUSY);
      context->tried.Post();
      TEST_ASSERT(context->mutex.Lock() == ErrorCode::OK);
      TEST_ASSERT(context->inside.fetch_add(1, std::memory_order_relaxed) == 0);
      const uint32_t value = context->writes.load(std::memory_order_relaxed);
      Thread::Yield();
      context->writes.store(value + 1U, std::memory_order_relaxed);
      TEST_ASSERT(context->inside.fetch_sub(1, std::memory_order_relaxed) == 1);
      context->mutex.Unlock();
      context->finished.Post();
    }
  };
  worker.Submit(job, &context);
  for (uint32_t round = 0; round < iterations; ++round)
  {
    // 此时协作任务在等 release，TryLock 可立即检查上一轮是否真正释放了锁。
    // The worker is waiting for release; TryLock checks that the prior round unlocked.
    TEST_ASSERT(context.mutex.TryLock() == ErrorCode::OK);
    TEST_ASSERT(context.inside.fetch_add(1, std::memory_order_relaxed) == 0);
    const uint32_t value = context.writes.load(std::memory_order_relaxed);
    context.writes.store(value + 1U, std::memory_order_relaxed);
    context.release.Post();
    TEST_ASSERT(context.tried.Wait(timeout_ms) == ErrorCode::OK);
    TEST_ASSERT(context.writes.load(std::memory_order_relaxed) == 2U * round + 1U);
    TEST_ASSERT(context.inside.fetch_sub(1, std::memory_order_relaxed) == 1);
    context.mutex.Unlock();
    TEST_ASSERT(context.finished.Wait(timeout_ms) == ErrorCode::OK);
    TEST_ASSERT(context.writes.load(std::memory_order_relaxed) == 2U * (round + 1U));
  }
  worker.WaitDone(timeout_ms);
  TEST_ASSERT(context.inside.load(std::memory_order_relaxed) == 0);
  TEST_ASSERT(context.mutex.TryLock() == ErrorCode::OK);
  context.mutex.Unlock();
}
}  // namespace LibXR::Test
#endif
