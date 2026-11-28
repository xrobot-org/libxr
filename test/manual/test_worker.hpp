/**
 * @file test_worker.hpp
 * @brief 手动测试复用的协作工作上下文 / Reusable cooperating worker for manual tests.
 *
 * 在多线程系统上复用一个持久 `Thread`；在裸机系统上通过 1 ms `Timer` 任务协同执行。
 * Reuses one persistent `Thread` on threaded systems, or a 1 ms `Timer` task on
 * bare-metal systems.
 *
 * @pre 实例须具有静态或进程级存活期（底层常驻线程或不可卸载的 `Timer` 任务会长期持有
 *      `this` 指针），不得在调用后析构。
 *      Instances must have static or process lifetime (the backing thread or
 *      non-removable `Timer` task retains `this` permanently); never destroy an active
 *      instance.
 * @pre 裸机（`LIBXR_NOT_SUPPORT_MUTI_THREAD`）模式下，提交的 `job` 必须能独立返回而
 *      不阻塞等待外层调用方，避免与空闲定时刷新形成死锁；需要真正并发阻塞交错的系统
 *      压测（如 `TestMutex`/`TestSemaphore`/`TestThread`）须在多线程后端运行。
 *      On bare-metal backends (`LIBXR_NOT_SUPPORT_MUTI_THREAD`), submitted jobs must
 *      return cooperatively without blocking on the caller; blocking multi-task tests
 *      (`TestMutex`/`TestSemaphore`/`TestThread`) require a threaded backend.
 */
#pragma once

#include <atomic>
#include <cstdint>

#include "semaphore.hpp"
#include "test_assert.hpp"
#include "thread.hpp"

#if defined(LIBXR_NOT_SUPPORT_MUTI_THREAD)
#include "timer.hpp"
#endif

namespace LibXR::Test
{
class ManualWorker
{
 public:
  using JobFn = void (*)(void*);

  ManualWorker(size_t stack_depth = 1024,
               Thread::Priority priority = Thread::Priority::MEDIUM)
  {
#if defined(LIBXR_NOT_SUPPORT_MUTI_THREAD)
    UNUSED(stack_depth);
    UNUSED(priority);
    auto handle = Timer::CreateTask(TimerTick, this, 1U);
    Timer::Start(handle);
    Timer::Add(handle);
#else
    thread_.Create<ManualWorker*>(this, ThreadLoop, "xr_manual_worker", stack_depth,
                                  priority);
#endif
  }

  ManualWorker(const ManualWorker&) = delete;
  ManualWorker& operator=(const ManualWorker&) = delete;

  /// @brief 仅当上一次任务完成且 `WaitDone()` 已回收完成令牌后返回 `true` /
  ///        Returns `true` only after `WaitDone()` has consumed the prior completion
  ///        token.
  bool IsIdle() const { return state_.load(std::memory_order_acquire) == STATE_IDLE; }

  void Submit(JobFn fn, void* arg)
  {
    TEST_ASSERT(fn != nullptr);
    uint32_t expected = STATE_IDLE;
    TEST_ASSERT(state_.compare_exchange_strong(
        expected, STATE_CLAIMING, std::memory_order_acq_rel, std::memory_order_acquire));
    fn_ = fn;
    arg_ = arg;
    state_.store(STATE_RUNNING, std::memory_order_release);
#if !defined(LIBXR_NOT_SUPPORT_MUTI_THREAD)
    start_.Post();
#endif
  }

  void WaitDone(uint32_t timeout_ms)
  {
    TEST_ASSERT(timeout_ms > 0U && timeout_ms < UINT32_MAX / 2U);
    TEST_ASSERT(done_.Wait(timeout_ms) == ErrorCode::OK);
    TEST_ASSERT(state_.load(std::memory_order_acquire) == STATE_COMPLETED);
    state_.store(STATE_IDLE, std::memory_order_release);
  }

 private:
  static constexpr uint32_t STATE_IDLE = 0U;
  static constexpr uint32_t STATE_CLAIMING = 1U;
  static constexpr uint32_t STATE_RUNNING = 2U;
  static constexpr uint32_t STATE_COMPLETED = 3U;

#if defined(LIBXR_NOT_SUPPORT_MUTI_THREAD)
  static void TimerTick(ManualWorker* self)
  {
    if (self->state_.load(std::memory_order_acquire) != STATE_RUNNING)
    {
      return;
    }
    JobFn fn = self->fn_;
    void* arg = self->arg_;
    fn(arg);
    self->state_.store(STATE_COMPLETED, std::memory_order_release);
    self->done_.Post();
  }
#else
  static void ThreadLoop(ManualWorker* self)
  {
    for (;;)
    {
      TEST_ASSERT(self->start_.Wait(UINT32_MAX) == ErrorCode::OK);
      TEST_ASSERT(self->state_.load(std::memory_order_acquire) == STATE_RUNNING);
      JobFn fn = self->fn_;
      void* arg = self->arg_;
      fn(arg);
      self->state_.store(STATE_COMPLETED, std::memory_order_release);
      self->done_.Post();
    }
  }

  Thread thread_;
  Semaphore start_{0U};
#endif

  Semaphore done_{0U};
  std::atomic<uint32_t> state_{STATE_IDLE};
  JobFn fn_ = nullptr;
  void* arg_ = nullptr;
};
}  // namespace LibXR::Test
