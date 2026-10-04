/**
 * @file test_timer.cpp
 * @brief Timer 周期与调度测试 / Timer period and scheduling tests.
 *
 * 手动 Refresh 检查停止、恢复和周期修改；另一个进程检查真实调度线程的回调。
 * Use manual Refresh for stop/resume and period changes; a separate process checks real
 * scheduling.
 */

#include <atomic>
#include <cstdio>

#include "lockfree_list.hpp"
#include "semaphore.hpp"
#include "test_assert.hpp"
#include "timer.hpp"

void test_timer_semantics()
{
  TEST_ASSERT(LibXR::Timer::list_ == nullptr);
  // 第一次 Add 之前的 Refresh 不创建任务列表和调度线程。
  // A Refresh before the first Add creates neither the task list nor a scheduler thread.
  LibXR::Timer::Refresh();
  TEST_ASSERT(LibXR::Timer::list_ == nullptr);
  // 预先设置列表，让 Add 不创建调度线程；后面的 Refresh 全由本测试逐步调用。
  // Provide the list so Add does not start a scheduler; this test drives every Refresh.
  LibXR::LockFreeList list;
  LibXR::Timer::list_ = &list;
  unsigned callbacks = 0;
  auto handle = LibXR::Timer::CreateTask<unsigned*>([](unsigned* count) { ++*count; },
                                                    &callbacks, 3);
  LibXR::Timer::Add(handle);

  auto advance = [](unsigned ticks)
  {
    for (unsigned tick = 0; tick < ticks; ++tick)
    {
      LibXR::Timer::Refresh();
    }
  };

  // 未启动时不会累计到期；下面的步数都是 Refresh 次数，不是墙钟时间。
  // A disabled timer does not advance toward firing; these steps count Refresh calls, not
  // wall time.
  advance(6);
  TEST_ASSERT(callbacks == 0);
  LibXR::Timer::Start(handle);
  advance(2);
  TEST_ASSERT(callbacks == 0);
  // 已经走过两步，停止期间保留进度；恢复后再走一步就满三步。
  // Two steps have elapsed. Stop preserves that progress, so one step after restart
  // completes the period.
  LibXR::Timer::Stop(handle);
  advance(5);
  TEST_ASSERT(callbacks == 0);
  LibXR::Timer::Start(handle);
  advance(1);
  TEST_ASSERT(callbacks == 1);
  advance(2);
  TEST_ASSERT(callbacks == 1);
  advance(1);
  TEST_ASSERT(callbacks == 2);

  LibXR::Timer::SetCycle(handle, 5);
  advance(4);
  TEST_ASSERT(callbacks == 2);
  // 当前已累计四步；缩短周期不清零计数，因此下一次 Refresh 就会触发。
  // Four steps are already counted; shortening the period keeps that count, so the next
  // Refresh fires.
  LibXR::Timer::SetCycle(handle, 2);
  advance(1);
  TEST_ASSERT(callbacks == 3);
  advance(1);
  TEST_ASSERT(callbacks == 3);
  advance(1);
  TEST_ASSERT(callbacks == 4);
  LibXR::Timer::Stop(handle);
  advance(6);
  TEST_ASSERT(callbacks == 4);
  LibXR::Timer::list_ = nullptr;
  std::fprintf(stderr,
               "timer_semantics: disabled, exact cycles, stop/resume phase, SetCycle "
               "passed; callbacks=%u\n",
               callbacks);
}

void test_timer_scheduler()
{
  struct State
  {
    LibXR::Timer::TimerHandle handle = nullptr;
    LibXR::Semaphore completed;
    std::atomic<unsigned> callbacks{0};
  };

  TEST_ASSERT(LibXR::Timer::list_ == nullptr);
  // 调度线程常驻；上下文保留到隔离进程退出，不能在回调可能使用它时释放。
  // The scheduler persists; keep its context until the isolated process exits.
  auto* state = new State;
  state->handle = LibXR::Timer::CreateTask<State*>(
      [](State* context)
      {
        if (context->callbacks.fetch_add(1, std::memory_order_relaxed) + 1 == 3)
        {
          LibXR::Timer::Stop(context->handle);
          context->completed.Post();
        }
      },
      state, 2);
  // 先启用再 Add，避免新线程在定时器启用前开始调度。
  // Enable before Add so the new scheduler cannot run before the timer is enabled.
  LibXR::Timer::Start(state->handle);
  LibXR::Timer::Add(state->handle);
  TEST_ASSERT(state->completed.Wait(1000) == LibXR::ErrorCode::OK);
  TEST_ASSERT(state->callbacks.load(std::memory_order_relaxed) == 3);
  std::fprintf(
      stderr,
      "timer_scheduler: native scheduler delivered 3 callbacks within a 1000ms wait\n");
}
