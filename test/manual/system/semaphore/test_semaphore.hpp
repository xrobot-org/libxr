/**
 * @file test_semaphore.hpp
 * @brief 实际系统的信号量测试 / Semaphore tests on the target system.
 *
 * 检查计数、空等待超时、线程上下文的回调接口及跨任务通知；协作任务使用传入的
 * ManualWorker。 Check counts, empty-wait timeout, thread-context callback posting and
 * cross-task notification.
 */
#pragma once

#include "../../test_worker.hpp"

#if !defined(LIBXR_NOT_SUPPORT_MUTI_THREAD)
namespace LibXR::Test
{
/**
 * @brief 检查信号量计数与任务间通知 / Check counting and task-to-task notification.
 * @param worker 独占借用的空闲 ManualWorker / Exclusively borrowed idle ManualWorker.
 * @param tick_ms 调度 tick 的毫秒数，向上取整 / Scheduler tick duration rounded up to
 * milliseconds.
 * @param timeout_ms 每次协作等待的上限 / Deadline for each cooperating wait.
 * @param iterations 通知批次数 / Number of notification batches.
 */
inline void TestSemaphore(ManualWorker& worker, uint32_t tick_ms,
                          uint32_t timeout_ms = 1000, uint32_t iterations = 1000)
{
  TEST_ASSERT(worker.IsIdle());
  TEST_ASSERT(iterations > 0);
  TEST_ASSERT(tick_ms > 0 && tick_ms <= timeout_ms / 16U);
  Semaphore count(0);
  TEST_ASSERT(count.Value() == 0);
  TEST_ASSERT(count.Wait(0) == ErrorCode::TIMEOUT);
  count.Post();
  count.Post();
  TEST_ASSERT(count.Value() == 2);
  TEST_ASSERT(count.Wait(0) == ErrorCode::OK);
  TEST_ASSERT(count.Wait(0) == ErrorCode::OK);
  TEST_ASSERT(count.Wait(0) == ErrorCode::TIMEOUT);
  count.PostFromCallback(false);
  TEST_ASSERT(count.Wait(0) == ErrorCode::OK);

  const uint32_t wait_ms = tick_ms * 4U;
  const uint32_t start = Thread::GetTime();
  TEST_ASSERT(count.Wait(wait_ms) == ErrorCode::TIMEOUT);
  const uint32_t elapsed = Thread::GetTime() - start;
  TEST_ASSERT(elapsed < timeout_ms);
  TEST_ASSERT(elapsed + tick_ms + 1U >= wait_ms);

  struct Context
  {
    Semaphore release;
    Semaphore notified;
    Semaphore batch_ready;
    uint32_t timeout_ms;
    uint32_t iterations;
  } context{{0}, {0}, {0}, timeout_ms, iterations};
  auto job = [](void* arg)
  {
    auto* context = static_cast<Context*>(arg);
    for (uint32_t round = 0; round < context->iterations; ++round)
    {
      TEST_ASSERT(context->release.Wait(context->timeout_ms) == ErrorCode::OK);
      const uint32_t batch = 1U + round % 8U;
      for (uint32_t item = 0; item < batch; ++item)
      {
        if (item % 2U == 0)
        {
          context->notified.Post();
        }
        else
        {
          context->notified.PostFromCallback(false);
        }
        Thread::Yield();
      }
      context->batch_ready.Post();
    }
  };
  worker.Submit(job, &context);
  for (uint32_t round = 0; round < iterations; ++round)
  {
    const uint32_t batch = 1U + round % 8U;
    TEST_ASSERT(context.notified.Wait(0) == ErrorCode::TIMEOUT);
    context.release.Post();
    // 偶数轮先等整批入队；奇数轮直接接收，让发送与等待交错。
    // Buffer a whole batch on even rounds; consume during publication on odd rounds.
    if (round % 2U == 0)
    {
      TEST_ASSERT(context.batch_ready.Wait(timeout_ms) == ErrorCode::OK);
      TEST_ASSERT(context.notified.Value() == batch);
    }
    for (uint32_t item = 0; item < batch; ++item)
    {
      TEST_ASSERT(context.notified.Wait(timeout_ms) == ErrorCode::OK);
    }
    if (round % 2U != 0)
    {
      TEST_ASSERT(context.batch_ready.Wait(timeout_ms) == ErrorCode::OK);
    }
    TEST_ASSERT(context.notified.Wait(0) == ErrorCode::TIMEOUT);
  }
  worker.WaitDone(timeout_ms);
  TEST_ASSERT(context.batch_ready.Wait(0) == ErrorCode::TIMEOUT);
}
}  // namespace LibXR::Test
#endif
