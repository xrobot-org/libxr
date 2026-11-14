/**
 * @file test_async_cooperative.cpp
 * @brief 软件 Timer 驱动 ASync 的测试 / Tests for software-Timer-driven ASync.
 *
 * 使用实际裸机后端和可控时钟，检查延后执行、完整发布、忙时拒绝和重复使用。
 * Use the real bare-metal backend with a controlled clock; check deferred execution,
 * complete publication, busy rejection and reuse. This is not a hardware ISR test.
 */
#include <cstdio>
#include <cstdlib>

#include "async.hpp"
#include "test_assert.hpp"
#include "timebase.hpp"
#include "timer.hpp"

namespace
{
uint32_t now_ms = 0;
void Tick()
{
  ++now_ms;
  LibXR::Timer::RefreshTimerInIdle();
}
}  // namespace

LibXR::MillisecondTimestamp LibXR::Timebase::GetMilliseconds() { return now_ms; }
LibXR::MicrosecondTimestamp LibXR::Timebase::GetMicroseconds()
{
  return static_cast<uint64_t>(now_ms) * 1000U;
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
  auto* async = new ASync(0, Thread::Priority::MEDIUM);
  Timer::RefreshTimerInIdle();
  TEST_ASSERT(async->GetStatus() == ASync::Status::READY);

  // 模拟提交者已取得 BUSY、但尚未发布任务的窗口，Timer 此时不能执行任务。
  // Model the reachable window after claiming BUSY but before publishing the job.
  async->status_.store(ASync::Status::BUSY, std::memory_order_relaxed);
  Tick();
  TEST_ASSERT(async->GetStatus() == ASync::Status::BUSY);
  async->status_.store(ASync::Status::READY, std::memory_order_release);

  unsigned calls = 0;
  auto job = ASync::Job::Create(
      [](bool in_isr, unsigned* calls, ASync* self)
      {
        TEST_ASSERT(!in_isr);
        TEST_ASSERT(self->GetStatus() == ASync::Status::BUSY);
        TEST_ASSERT(self->AssignJob(ASync::Job{}) == ErrorCode::BUSY);
        TEST_ASSERT(self->AssignJobFromCallback(ASync::Job{}, true) == ErrorCode::BUSY);
        ++*calls;
      },
      &calls);
  auto rejected = ASync::Job::Create([](bool, void*, ASync*) { TEST_ASSERT(false); },
                                     static_cast<void*>(nullptr));
  for (unsigned iteration = 0; iteration < 3; ++iteration)
  {
    const auto result = iteration == 0
                            ? async->AssignJob(job)
                            : async->AssignJobFromCallback(job, iteration == 2);
    TEST_ASSERT(result == ErrorCode::OK);
    TEST_ASSERT(calls == iteration);
    TEST_ASSERT(async->GetStatus() == ASync::Status::BUSY);
    TEST_ASSERT(async->AssignJob(rejected) == ErrorCode::BUSY);
    TEST_ASSERT(async->AssignJobFromCallback(rejected, true) == ErrorCode::BUSY);
    Timer::RefreshTimerInIdle();
    TEST_ASSERT(calls == iteration);
    Tick();
    TEST_ASSERT(calls == iteration + 1);
    TEST_ASSERT(async->AssignJob(rejected) == ErrorCode::BUSY);
    TEST_ASSERT(async->AssignJobFromCallback(rejected, false) == ErrorCode::BUSY);
    TEST_ASSERT(async->GetStatus() == ASync::Status::DONE);
    TEST_ASSERT(async->GetStatus() == ASync::Status::READY);
    Tick();
    TEST_ASSERT(calls == iteration + 1);
  }
  std::puts("cooperative ASync: publication, deferred execution and reuse passed");
}
