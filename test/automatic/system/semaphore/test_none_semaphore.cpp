/**
 * @file test_none_semaphore.cpp
 * @brief 裸机后端 Semaphore 原子计数与空闲定时刷新测试 /
 *        Bare-metal Semaphore atomic count and idle-timer refresh tests.
 */
#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <thread>

#include "semaphore.hpp"
#include "test_assert.hpp"
#include "timebase.hpp"
#include "timer.hpp"

namespace
{
std::atomic<uint32_t> now_ms{UINT32_MAX - 2U};
// 确定性推进步长：为 1 时，每次通过 GetMilliseconds() 读取时钟在首次基准后加 1 ms。
// Deterministic clock stepping: when enabled, advances now_ms after the baseline read.
std::atomic<uint32_t> auto_step_enabled{0U};
std::atomic<uint32_t> auto_step_skip{0U};
}  // namespace

LibXR::MillisecondTimestamp LibXR::Timebase::GetMilliseconds()
{
  const uint32_t cur = now_ms.load(std::memory_order_acquire);
  if (auto_step_enabled.load(std::memory_order_acquire) != 0U)
  {
    const uint32_t skip = auto_step_skip.load(std::memory_order_relaxed);
    if (skip > 0U)
    {
      auto_step_skip.store(skip - 1U, std::memory_order_relaxed);
    }
    else
    {
      now_ms.store(cur + 1U, std::memory_order_release);
    }
  }
  return cur;
}

LibXR::MicrosecondTimestamp LibXR::Timebase::GetMicroseconds()
{
  return static_cast<uint64_t>(now_ms.load(std::memory_order_acquire)) * 1000U;
}

extern "C" void libxr_fatal_error(const char* file, uint32_t line, bool)
{
  std::fprintf(stderr, "%s:%u: product assertion failed\n", file ? file : "(no location)",
               line);
  std::abort();
}

extern "C" void libxr_fatal_error_and_print(const char* file, uint32_t line, bool in_isr)
{
  libxr_fatal_error(file, line, in_isr);
}

int main()
{
  using namespace LibXR;

  Semaphore sem(0U);
  TEST_ASSERT(sem.Value() == 0U);
  TEST_ASSERT(sem.Wait(0U) == ErrorCode::TIMEOUT);

  sem.Post();
  sem.PostFromCallback(true);
  TEST_ASSERT(sem.Value() == 2U);
  TEST_ASSERT(sem.Wait(0U) == ErrorCode::OK);
  TEST_ASSERT(sem.Value() == 1U);
  TEST_ASSERT(sem.Wait(0U) == ErrorCode::OK);
  TEST_ASSERT(sem.Value() == 0U);
  TEST_ASSERT(sem.Wait(0U) == ErrorCode::TIMEOUT);

  // 先建立 Timer::RefreshTimerInIdle() 静态基准为 UINT32_MAX - 2U，再确定性验证
  // 跨 UINT32_MAX
  // 回绕的定时唤醒、回绕后的确定性超时到期以及常规区间定时唤醒（无线程竞态）。 Establish
  // RefreshTimerInIdle() baseline at UINT32_MAX - 2U, then deterministically verify timer
  // wakeup across UINT32_MAX wraparound, post-wraparound timed-wait expiration, and
  // normal post wakeup without any clock-thread race.
  Timer::RefreshTimerInIdle();

  struct TimerContext
  {
    Semaphore* sem;
    uint32_t target_tick = 0U;
    uint32_t ticks = 0U;
  } timer_ctx{&sem, 4U, 0U};

  auto handle = Timer::CreateTask(
      +[](TimerContext* ctx)
      {
        ++ctx->ticks;
        if (ctx->target_tick != 0U && ctx->ticks == ctx->target_tick)
        {
          ctx->sem->PostFromCallback(true);
        }
      },
      &timer_ctx, 1U);
  Timer::Add(handle);
  Timer::Start(handle);

  // 1. 跨 UINT32_MAX 回绕的定时唤醒：时钟从 UINT32_MAX - 2 推进 4 ms 到 1，第 4 个 tick
  // 触发 Post。 Wraparound timed wakeup: now_ms crosses UINT32_MAX -> 0 -> 1 and fires on
  // tick 4.
  auto_step_skip.store(1U, std::memory_order_relaxed);
  auto_step_enabled.store(1U, std::memory_order_release);
  TEST_ASSERT(sem.Wait(10U) == ErrorCode::OK);
  auto_step_enabled.store(0U, std::memory_order_release);
  TEST_ASSERT(timer_ctx.ticks >= 4U);
  TEST_ASSERT(static_cast<uint32_t>(Timebase::GetMilliseconds()) < 16U);

  // 2. 确定性超时到期：无任务 Post 时，Wait(5) 推进 5 ms 后精确返回 TIMEOUT。
  // Deterministic timeout expiration: without Post, Wait(5) expires cleanly.
  timer_ctx.target_tick = 0U;
  const uint32_t before_timeout = Timebase::GetMilliseconds();
  auto_step_skip.store(1U, std::memory_order_relaxed);
  auto_step_enabled.store(1U, std::memory_order_release);
  TEST_ASSERT(sem.Wait(5U) == ErrorCode::TIMEOUT);
  auto_step_enabled.store(0U, std::memory_order_release);
  const uint32_t after_timeout = Timebase::GetMilliseconds();
  TEST_ASSERT(static_cast<uint32_t>(after_timeout - before_timeout) >= 5U);
  TEST_ASSERT(sem.Value() == 0U);

  // 3. 非回绕常规区间定时唤醒。
  // Normal (non-wraparound) timed wakeup.
  timer_ctx.ticks = 0U;
  timer_ctx.target_tick = 3U;
  auto_step_skip.store(1U, std::memory_order_relaxed);
  auto_step_enabled.store(1U, std::memory_order_release);
  TEST_ASSERT(sem.Wait(20U) == ErrorCode::OK);
  auto_step_enabled.store(0U, std::memory_order_release);
  Timer::Stop(handle);
  TEST_ASSERT(timer_ctx.ticks >= 3U);

  // 4. 并发 Post / Wait(0) 交错：验证不丢计数、不发生下溢且建立 acquire/release 同步。
  // Concurrent Post / Wait(0) stress: verify no lost tokens, no underflow, and
  // acquire/release publication.
  constexpr uint32_t TOTAL_POSTS = 20000U;
  Semaphore stress_sem(0U);
  uint32_t published_value = 0U;
  std::atomic<uint32_t> produced{0U};

  std::thread producer(
      [&]
      {
        for (uint32_t i = 1U; i <= TOTAL_POSTS; ++i)
        {
          if (i == 1U)
          {
            published_value = 0x5A5AU;
          }
          stress_sem.PostFromCallback((i & 1U) != 0U);
          produced.fetch_add(1U, std::memory_order_release);
        }
      });

  uint32_t consumed = 0U;
  bool observed_first = false;
  while (consumed < TOTAL_POSTS)
  {
    if (stress_sem.Wait(0U) == ErrorCode::OK)
    {
      if (!observed_first)
      {
        TEST_ASSERT(published_value == 0x5A5AU);
        observed_first = true;
      }
      ++consumed;
    }
    else
    {
      std::this_thread::yield();
    }
  }
  producer.join();
  TEST_ASSERT(produced.load(std::memory_order_acquire) == TOTAL_POSTS);
  TEST_ASSERT(stress_sem.Value() == 0U);
  TEST_ASSERT(stress_sem.Wait(0U) == ErrorCode::TIMEOUT);

  std::puts("none semaphore: atomic_ref Post/Wait(0)/wraparound/timed-wait/Value passed");
  return 0;
}
