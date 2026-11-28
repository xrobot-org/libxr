/**
 * @file test_thread.hpp
 * @brief 在实际系统上检查 Thread 的延时 / Check Thread delays on the target system.
 *
 * 在调用方提供的 ManualWorker 中检查 Sleep、SleepUntil 及计划唤醒时间；不创建或回收线程。
 * Use a supplied ManualWorker to check Sleep, SleepUntil and scheduled wake times; no
 * thread management.
 */
#pragma once

#include "../../test_worker.hpp"

#if !defined(LIBXR_NOT_SUPPORT_MUTI_THREAD)
namespace LibXR::Test
{
/**
 * @brief 检查延时和周期等待 / Check delays and periodic waits.
 * @param worker 独占借用的空闲 ManualWorker / Exclusively borrowed idle ManualWorker.
 * @param tick_ms 调度 tick 的毫秒数，向上取整 / Scheduler tick duration rounded up to
 * milliseconds.
 * @param timeout_ms 每次进度等待的上限 / Deadline for each progress wait.
 * @param iterations 重复轮数 / Number of rounds.
 */
inline void TestThread(ManualWorker& worker, uint32_t tick_ms, uint32_t timeout_ms = 1000,
                       uint32_t iterations = 100)
{
  TEST_ASSERT(worker.IsIdle());
  TEST_ASSERT(iterations > 0);
  TEST_ASSERT(tick_ms > 0 && tick_ms <= timeout_ms / 16U);
  struct Context
  {
    uint32_t tick_ms;
    uint32_t timeout_ms;
    uint32_t iterations;
    Semaphore progressed;
  } context{tick_ms, timeout_ms, iterations, {0}};
  auto job = [](void* arg)
  {
    auto* context = static_cast<Context*>(arg);
    MillisecondTimestamp wakeup = Thread::GetTime();
    for (uint32_t round = 0; round < context->iterations; ++round)
    {
      const uint32_t delay = (4U + round % 4U) * context->tick_ms;
      const uint32_t start = Thread::GetTime();
      Thread::Sleep(delay);
      const uint32_t elapsed = Thread::GetTime() - start;
      TEST_ASSERT(elapsed < context->timeout_ms);
      TEST_ASSERT(elapsed + context->tick_ms + 1U >= delay);
      context->progressed.Post();

      // 沿用上一轮的计划时间，兼顾已过期的等待和下一次真正阻塞的等待。
      // Keep the schedule across rounds, exercising both overdue and future waits.
      for (unsigned step = 0; step < 2; ++step)
      {
        const uint32_t target = static_cast<uint32_t>(wakeup) + delay;
        const uint32_t wait_start = Thread::GetTime();
        Thread::SleepUntil(wakeup, delay);
        const uint32_t now = Thread::GetTime();
        TEST_ASSERT(static_cast<uint32_t>(wakeup) == target);
        TEST_ASSERT(now - wait_start < context->timeout_ms);
        // 允许一个 tick 的边界误差和不足1ms的取整误差，不要求准点唤醒。
        // Allow one tick plus millisecond truncation; do not require exact wakeup
        // time.
        TEST_ASSERT(now - target < UINT32_MAX / 2U ||
                    target - now <= context->tick_ms + 1U);
        context->progressed.Post();
      }
    }
  };
  worker.Submit(job, &context);
  for (uint32_t round = 0; round < iterations; ++round)
  {
    for (unsigned step = 0; step < 3; ++step)
    {
      TEST_ASSERT(context.progressed.Wait(timeout_ms) == ErrorCode::OK);
    }
  }
  worker.WaitDone(timeout_ms);
  TEST_ASSERT(context.progressed.Wait(0) == ErrorCode::TIMEOUT);
}
}  // namespace LibXR::Test
#endif
