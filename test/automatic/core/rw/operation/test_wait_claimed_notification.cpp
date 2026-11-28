/**
 * @file test_wait_claimed_notification.cpp
 * @brief 确定性验证全部五处 Detail::WaitClaimedNotification 调用点 /
 *        Deterministic tests for all five Detail::WaitClaimedNotification call sites.
 *
 * 覆盖 / Covers:
 * 1. AsyncBlockWait::Wait (src/core/rw/operation.cpp)
 * 2. ReadPort::WaitForBlock (src/core/rw/read_port.cpp)
 * 3. WritePort::WaitForRetirement (src/core/rw/write_port.cpp)
 * 4. WritePort::WaitForBlock (src/core/rw/write_port.cpp)
 * 5. Topic::SyncSubscriber::Wait (src/middleware/message/subscriber/sync.hpp)
 *
 * 除单线程脚本化测试（断言回收路径严格以 UINT32_MAX 调用 Wait、覆盖初始 TIMEOUT/FAILED
 * 及回收阶段 TIMEOUT->OK 与 FAILED->REQUIRE）外，另用双线程确定性屏障覆盖「完成方已认领
 * 原子状态但尚未 Post 信号量」的真实竞争窗口。
 */

#include <condition_variable>
#include <csetjmp>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>

#include "operation.hpp"
#include "read_port.hpp"
#include "semaphore.hpp"
#include "test_assert.hpp"
#include "timebase.hpp"
#include "topic.hpp"
#include "write_port.hpp"

namespace
{
struct WaitStep
{
  LibXR::ErrorCode forced_return = LibXR::ErrorCode::OK;
  bool consume_token_if_ok = true;
  bool check_timeout = false;
  uint32_t expected_timeout = UINT32_MAX;
  bool wait_for_post_entered = false;
  bool block_until_token = false;
  std::function<void()> before_return{};
};

struct ScriptedSemState
{
  std::mutex mtx;
  std::condition_variable cv;
  uint32_t tokens = 0U;
  uint32_t post_calls = 0U;
  uint32_t wait_calls = 0U;
  uint32_t hold_post_until_wait_calls = 0U;
  uint32_t post_entered = 0U;
  std::deque<WaitStep> script{};
};

std::mutex g_table_mtx;

std::unordered_map<const LibXR::Semaphore*, std::shared_ptr<ScriptedSemState>>& SemTable()
{
  static std::unordered_map<const LibXR::Semaphore*, std::shared_ptr<ScriptedSemState>>
      table;
  return table;
}

std::shared_ptr<ScriptedSemState> StatePtr(const LibXR::Semaphore& sem)
{
  std::lock_guard<std::mutex> lk(g_table_mtx);
  auto& slot = SemTable()[&sem];
  if (!slot)
  {
    slot = std::make_shared<ScriptedSemState>();
  }
  return slot;
}

bool g_expect_fatal = false;
std::jmp_buf g_fatal_jmp;
uint32_t g_fatal_hits = 0U;

void ResetFatalTrap()
{
  g_expect_fatal = false;
  g_fatal_hits = 0U;
}

// 在无非平凡局部对象的包装函数里捕获 REQUIRE 触发的 libxr_fatal_error。
// Catch REQUIRE-triggered libxr_fatal_error without active non-trivial stack frames.
void ExpectFatal(void (*fn)(void*), void* ctx)
{
  ResetFatalTrap();
  g_expect_fatal = true;
  if (setjmp(g_fatal_jmp) == 0)
  {
    fn(ctx);
    g_expect_fatal = false;
    TEST_ASSERT(false);
  }
  g_expect_fatal = false;
  TEST_ASSERT(g_fatal_hits == 1U);
}

void NoopWriteFun(LibXR::WritePort&, bool) {}

void PerformPost(const LibXR::Semaphore& sem)
{
  auto st = StatePtr(sem);
  std::unique_lock<std::mutex> lk(st->mtx);
  st->post_entered = 1U;
  st->cv.notify_all();
  if (st->hold_post_until_wait_calls > 0U)
  {
    st->cv.wait(lk, [&] { return st->wait_calls >= st->hold_post_until_wait_calls; });
  }
  ++st->tokens;
  ++st->post_calls;
  st->cv.notify_all();
}
}  // namespace

extern "C" void libxr_fatal_error(const char* file, uint32_t line, bool)
{
  if (g_expect_fatal)
  {
    ++g_fatal_hits;
    std::longjmp(g_fatal_jmp, 1);
  }
  std::fprintf(stderr, "%s:%u: unexpected product assertion failure\n",
               file ? file : "(no location)", line);
  std::abort();
}

extern "C" void libxr_fatal_error_and_print(const char* file, uint32_t line, bool in_isr)
{
  libxr_fatal_error(file, line, in_isr);
}

LibXR::MillisecondTimestamp LibXR::Timebase::GetMilliseconds() { return 0U; }

LibXR::MicrosecondTimestamp LibXR::Timebase::GetMicroseconds() { return 12345U; }

LibXR::Semaphore::Semaphore(uint32_t init_count) : semaphore_handle_(init_count)
{
  std::lock_guard<std::mutex> lk(g_table_mtx);
  auto st = std::make_shared<ScriptedSemState>();
  st->tokens = init_count;
  SemTable()[this] = std::move(st);
}

LibXR::Semaphore::~Semaphore()
{
  std::lock_guard<std::mutex> lk(g_table_mtx);
  SemTable().erase(this);
}

void LibXR::Semaphore::Post() { PerformPost(*this); }

void LibXR::Semaphore::PostFromCallback(bool) { PerformPost(*this); }

LibXR::ErrorCode LibXR::Semaphore::Wait(uint32_t timeout)
{
  auto st = StatePtr(*this);
  WaitStep step{};
  bool has_step = false;
  {
    std::unique_lock<std::mutex> lk(st->mtx);
    ++st->wait_calls;
    st->cv.notify_all();
    if (!st->script.empty())
    {
      step = std::move(st->script.front());
      st->script.pop_front();
      has_step = true;
    }
    else
    {
      if (st->tokens > 0U)
      {
        --st->tokens;
        return ErrorCode::OK;
      }
      return ErrorCode::TIMEOUT;
    }
  }

  if (step.check_timeout)
  {
    TEST_ASSERT(timeout == step.expected_timeout);
  }
  if (step.wait_for_post_entered)
  {
    std::unique_lock<std::mutex> lk(st->mtx);
    st->cv.wait(lk, [&] { return st->post_entered != 0U; });
    TEST_ASSERT(st->tokens == 0U);
  }
  if (step.before_return)
  {
    step.before_return();
  }
  if (step.block_until_token)
  {
    std::unique_lock<std::mutex> lk(st->mtx);
    st->cv.wait(lk, [&] { return st->tokens > 0U; });
  }
  if (step.forced_return == ErrorCode::OK && step.consume_token_if_ok)
  {
    std::lock_guard<std::mutex> lk(st->mtx);
    TEST_ASSERT(st->tokens > 0U);
    --st->tokens;
  }
  (void)has_step;
  return step.forced_return;
}

size_t LibXR::Semaphore::Value()
{
  auto st = StatePtr(*this);
  std::lock_guard<std::mutex> lk(st->mtx);
  return st->tokens;
}

namespace
{
using namespace LibXR;

WaitStep DrainTimeoutStep()
{
  WaitStep s{};
  s.forced_return = ErrorCode::TIMEOUT;
  s.consume_token_if_ok = false;
  s.check_timeout = true;
  s.expected_timeout = UINT32_MAX;
  return s;
}

WaitStep DrainOkStep(bool block_until_token = false)
{
  WaitStep s{};
  s.forced_return = ErrorCode::OK;
  s.consume_token_if_ok = true;
  s.check_timeout = true;
  s.expected_timeout = UINT32_MAX;
  s.block_until_token = block_until_token;
  return s;
}

WaitStep DrainFailedStep()
{
  WaitStep s{};
  s.forced_return = ErrorCode::FAILED;
  s.consume_token_if_ok = false;
  s.check_timeout = true;
  s.expected_timeout = UINT32_MAX;
  return s;
}

void TestAsyncBlockWait()
{
  // 1a. 初始 Wait 返回 TIMEOUT / FAILED 前由 TryPost 认领，回收阶段严格以 UINT32_MAX
  // 经历 TIMEOUT -> OK。
  for (ErrorCode init_err : {ErrorCode::TIMEOUT, ErrorCode::FAILED})
  {
    for (ErrorCode completion_ec : {ErrorCode::OK, ErrorCode::FAILED})
    {
      Semaphore sem(0U);
      AsyncBlockWait wait;
      wait.Start(sem);
      auto st = StatePtr(sem);
      st->script.push_back({init_err, false, true, 10U, false, false,
                            [&] { TEST_ASSERT(wait.TryPost(true, completion_ec)); }});
      st->script.push_back(DrainTimeoutStep());
      st->script.push_back(DrainOkStep());

      TEST_ASSERT(wait.Wait(10U) == completion_ec);
      TEST_ASSERT(st->wait_calls == 3U);
      TEST_ASSERT(st->post_calls == 1U);
      TEST_ASSERT(st->tokens == 0U);
      TEST_ASSERT(!wait.TryPost(false, ErrorCode::OK));

      wait.Start(sem);
      TEST_ASSERT(wait.TryPost(false, ErrorCode::OK));
      TEST_ASSERT(wait.Wait(0U) == ErrorCode::OK);
      TEST_ASSERT(st->tokens == 0U);
    }
  }

  // 1b. 双线程确定性 Post 前窗口：completer 先 CAS 认领 CLAIMED 并卡在 Post 增加计数前，
  // waiter 初始 Wait(10) 在 tokens==0 时返回 TIMEOUT，进入 WaitClaimedNotification 经历
  // 首次 Wait(UINT32_MAX)->TIMEOUT（仍 tokens==0），第二次 Wait(UINT32_MAX) 放行 Post
  // 并消费。
  {
    Semaphore sem(0U);
    AsyncBlockWait wait;
    wait.Start(sem);
    auto st = StatePtr(sem);
    st->hold_post_until_wait_calls = 3U;  // wait #1=init, #2=drain TO, #3=drain OK
    WaitStep init_step{};
    init_step.forced_return = ErrorCode::TIMEOUT;
    init_step.consume_token_if_ok = false;
    init_step.check_timeout = true;
    init_step.expected_timeout = 10U;
    init_step.wait_for_post_entered = true;
    st->script.push_back(init_step);
    st->script.push_back(DrainTimeoutStep());
    st->script.push_back(DrainOkStep(true));

    std::thread completer([&] { TEST_ASSERT(wait.TryPost(true, ErrorCode::FAILED)); });
    TEST_ASSERT(wait.Wait(10U) == ErrorCode::FAILED);
    completer.join();
    TEST_ASSERT(st->wait_calls == 3U);
    TEST_ASSERT(st->post_calls == 1U);
    TEST_ASSERT(st->tokens == 0U);
  }

  // 1c. 未认领时初始 ThreadX FAILED 分离为 TIMEOUT 且不泄漏 token。
  {
    Semaphore sem(0U);
    AsyncBlockWait wait;
    wait.Start(sem);
    auto st = StatePtr(sem);
    st->script.push_back({ErrorCode::FAILED, false, true, 10U, false, false, {}});

    TEST_ASSERT(wait.Wait(10U) == ErrorCode::TIMEOUT);
    TEST_ASSERT(st->wait_calls == 1U);
    TEST_ASSERT(st->tokens == 0U);
    TEST_ASSERT(!wait.TryPost(false, ErrorCode::OK));
    TEST_ASSERT(st->tokens == 0U);
  }

  // 1d. 回收阶段返回 FAILED 必须触发 REQUIRE（且参数须为 UINT32_MAX）。
  {
    struct Ctx
    {
      Semaphore sem{0U};
      AsyncBlockWait wait{};
    } ctx;
    ctx.wait.Start(ctx.sem);
    auto st = StatePtr(ctx.sem);
    st->script.push_back({ErrorCode::TIMEOUT, false, true, 10U, false, false,
                          [&] { TEST_ASSERT(ctx.wait.TryPost(true, ErrorCode::OK)); }});
    st->script.push_back(DrainFailedStep());

    ExpectFatal(
        +[](void* p)
        {
          auto* c = static_cast<Ctx*>(p);
          (void)c->wait.Wait(10U);
        },
        &ctx);
    TEST_ASSERT(st->wait_calls == 2U);
    TEST_ASSERT(st->tokens == 1U);
  }
}

void TestReadPortWaitForBlock()
{
  const uint8_t payload[4] = {0x11U, 0x22U, 0x33U, 0x44U};

  // 2a. 未认领时初始 FAILED 取消挂起读并返回 TIMEOUT，端口恢复 IDLE。
  {
    ReadPort port(32U);
    Semaphore sem(0U);
    auto st = StatePtr(sem);
    st->script.push_back({ErrorCode::FAILED, false, true, 10U, false, false, {}});
    uint8_t dst[4] = {};
    ReadOperation op(sem, 10U);

    TEST_ASSERT(port(RawData(dst, sizeof(dst)), op) == ErrorCode::TIMEOUT);
    TEST_ASSERT(st->wait_calls == 1U);
    TEST_ASSERT(st->tokens == 0U);

    {
      auto rq = port.GetReadQueue(false);
      TEST_ASSERT(rq.PushBatch(payload, sizeof(payload)) == ErrorCode::OK);
      rq.Publish();
    }
    ReadOperation op2(sem, 10U);
    TEST_ASSERT(port(RawData(dst, sizeof(dst)), op2) == ErrorCode::OK);
    TEST_ASSERT(std::memcmp(dst, payload, sizeof(payload)) == 0);
    TEST_ASSERT(st->tokens == 0U);
  }

  // 2b. 初始 TIMEOUT / FAILED + 严格 UINT32_MAX 回收 TIMEOUT -> OK。
  for (ErrorCode init_err : {ErrorCode::TIMEOUT, ErrorCode::FAILED})
  {
    ReadPort port(32U);
    Semaphore sem(0U);
    auto st = StatePtr(sem);
    uint8_t dst[4] = {};
    ReadOperation op(sem, 10U);

    st->script.push_back({init_err, false, true, 10U, false, false, [&]
                          {
                            auto rq = port.GetReadQueue(true);
                            TEST_ASSERT(rq.PushBatch(payload, sizeof(payload)) ==
                                        ErrorCode::OK);
                            rq.Publish();
                          }});
    st->script.push_back(DrainTimeoutStep());
    st->script.push_back(DrainOkStep());

    TEST_ASSERT(port(RawData(dst, sizeof(dst)), op) == ErrorCode::OK);
    TEST_ASSERT(std::memcmp(dst, payload, sizeof(payload)) == 0);
    TEST_ASSERT(st->wait_calls == 3U);
    TEST_ASSERT(st->post_calls == 1U);
    TEST_ASSERT(st->tokens == 0U);
    TEST_ASSERT(port.ClearQueuedData(false) == ErrorCode::OK);
  }

  // 2c. 双线程确定性 Post 前窗口：Publish 认领 BLOCK_CLAIMED 并阻塞在 Post 前。
  {
    ReadPort port(32U);
    Semaphore sem(0U);
    auto st = StatePtr(sem);
    st->hold_post_until_wait_calls = 3U;
    uint8_t dst[4] = {};
    ReadOperation op(sem, 10U);
    std::thread producer;

    WaitStep init_step{};
    init_step.forced_return = ErrorCode::TIMEOUT;
    init_step.consume_token_if_ok = false;
    init_step.check_timeout = true;
    init_step.expected_timeout = 10U;
    init_step.before_return = [&]
    {
      producer = std::thread(
          [&]
          {
            auto rq = port.GetReadQueue(true);
            TEST_ASSERT(rq.PushBatch(payload, sizeof(payload)) == ErrorCode::OK);
            rq.Publish();
          });
      std::unique_lock<std::mutex> lk(st->mtx);
      st->cv.wait(lk, [&] { return st->post_entered != 0U; });
      TEST_ASSERT(st->tokens == 0U);
    };
    st->script.push_back(init_step);
    st->script.push_back(DrainTimeoutStep());
    st->script.push_back(DrainOkStep(true));

    TEST_ASSERT(port(RawData(dst, sizeof(dst)), op) == ErrorCode::OK);
    producer.join();
    TEST_ASSERT(std::memcmp(dst, payload, sizeof(payload)) == 0);
    TEST_ASSERT(st->wait_calls == 3U);
    TEST_ASSERT(st->tokens == 0U);
  }

  // 2d. 回收阶段 FAILED 触发 REQUIRE。
  {
    struct Ctx
    {
      ReadPort port{32U};
      Semaphore sem{0U};
      uint8_t dst[4] = {};
      ReadOperation op{sem, 10U};
    } ctx;
    auto st = StatePtr(ctx.sem);
    st->script.push_back({ErrorCode::TIMEOUT, false, true, 10U, false, false, [&]
                          {
                            auto rq = ctx.port.GetReadQueue(true);
                            TEST_ASSERT(rq.PushBatch(payload, sizeof(payload)) ==
                                        ErrorCode::OK);
                            rq.Publish();
                          }});
    st->script.push_back(DrainFailedStep());

    ExpectFatal(
        +[](void* p)
        {
          auto* c = static_cast<Ctx*>(p);
          (void)c->port(RawData(c->dst, sizeof(c->dst)), c->op);
        },
        &ctx);
    TEST_ASSERT(st->wait_calls == 2U);
    TEST_ASSERT(st->tokens == 1U);
  }
}

void TestWritePortWaitForRetirement()
{
  const uint8_t req1[4] = {1U, 2U, 3U, 4U};
  const uint8_t req2[4] = {5U, 6U, 7U, 8U};

  // 3a. 未认领初始 FAILED 退回 BLOCK_DETACHED 并返回 TIMEOUT。
  {
    WritePort port(4U, 32U);
    port = NoopWriteFun;

    Semaphore sem1(0U);
    StatePtr(sem1)->script.push_back(
        {ErrorCode::TIMEOUT, false, true, 1U, false, false, {}});
    WriteOperation op1(sem1, 1U);
    TEST_ASSERT(port(ConstRawData(req1, sizeof(req1)), op1) == ErrorCode::TIMEOUT);

    Semaphore sem2(0U);
    auto st2 = StatePtr(sem2);
    st2->script.push_back({ErrorCode::FAILED, false, true, 1U, false, false, {}});
    WriteOperation op2(sem2, 1U);
    TEST_ASSERT(port(ConstRawData(req2, sizeof(req2)), op2) == ErrorCode::TIMEOUT);
    TEST_ASSERT(st2->wait_calls == 1U);
    TEST_ASSERT(st2->tokens == 0U);
    TEST_ASSERT(port.Size() == sizeof(req1));

    uint8_t sink[4] = {};
    {
      auto wq = port.GetWriteQueue(false);
      TEST_ASSERT(wq.AvailableSize() == sizeof(req1));
      wq.PopAll(sink);
    }
    TEST_ASSERT(std::memcmp(sink, req1, sizeof(req1)) == 0);
    TEST_ASSERT(port.Size() == 0U);
  }

  // 3b. 初始 TIMEOUT / FAILED + 严格 UINT32_MAX 回收 TIMEOUT -> OK。
  for (ErrorCode init_err : {ErrorCode::TIMEOUT, ErrorCode::FAILED})
  {
    WritePort port(4U, 32U);
    port = NoopWriteFun;

    Semaphore sem1(0U);
    StatePtr(sem1)->script.push_back(
        {ErrorCode::TIMEOUT, false, true, 1U, false, false, {}});
    WriteOperation op1(sem1, 1U);
    TEST_ASSERT(port(ConstRawData(req1, sizeof(req1)), op1) == ErrorCode::TIMEOUT);

    Semaphore sem2(0U);
    auto st2 = StatePtr(sem2);
    uint8_t sink[4] = {};
    st2->script.push_back({init_err, false, true, 1U, false, false, [&]
                           {
                             auto wq = port.GetWriteQueue(true);
                             TEST_ASSERT(wq.AvailableSize() == sizeof(req1));
                             wq.PopAll(sink);
                           }});
    st2->script.push_back(DrainTimeoutStep());
    st2->script.push_back(DrainOkStep());

    WriteOperation op2(sem2, 1U);
    TEST_ASSERT(port(ConstRawData(req2, sizeof(req2)), op2) == ErrorCode::TIMEOUT);
    TEST_ASSERT(std::memcmp(sink, req1, sizeof(req1)) == 0);
    TEST_ASSERT(st2->wait_calls == 3U);
    TEST_ASSERT(st2->post_calls == 1U);
    TEST_ASSERT(st2->tokens == 0U);
    TEST_ASSERT(port.Size() == 0U);

    WriteOperation none_op;
    TEST_ASSERT(port(ConstRawData(req2, sizeof(req2)), none_op) == ErrorCode::OK);
    TEST_ASSERT(port.Size() == sizeof(req2));
  }

  // 3c. 双线程确定性 Post 前窗口（WaitForRetirement 认领 LOCKED 但尚未 Post）。
  {
    WritePort port(4U, 32U);
    port = NoopWriteFun;
    Semaphore sem1(0U);
    StatePtr(sem1)->script.push_back(
        {ErrorCode::TIMEOUT, false, true, 1U, false, false, {}});
    WriteOperation op1(sem1, 1U);
    TEST_ASSERT(port(ConstRawData(req1, sizeof(req1)), op1) == ErrorCode::TIMEOUT);

    Semaphore sem2(0U);
    auto st2 = StatePtr(sem2);
    st2->hold_post_until_wait_calls = 3U;
    uint8_t sink[4] = {};
    std::thread drainer;
    WaitStep init_step{};
    init_step.forced_return = ErrorCode::TIMEOUT;
    init_step.consume_token_if_ok = false;
    init_step.check_timeout = true;
    init_step.expected_timeout = 1U;
    init_step.before_return = [&]
    {
      drainer = std::thread(
          [&]
          {
            auto wq = port.GetWriteQueue(true);
            wq.PopAll(sink);
          });
      std::unique_lock<std::mutex> lk(st2->mtx);
      st2->cv.wait(lk, [&] { return st2->post_entered != 0U; });
      TEST_ASSERT(st2->tokens == 0U);
    };
    st2->script.push_back(init_step);
    st2->script.push_back(DrainTimeoutStep());
    st2->script.push_back(DrainOkStep(true));

    WriteOperation op2(sem2, 1U);
    TEST_ASSERT(port(ConstRawData(req2, sizeof(req2)), op2) == ErrorCode::TIMEOUT);
    drainer.join();
    TEST_ASSERT(std::memcmp(sink, req1, sizeof(req1)) == 0);
    TEST_ASSERT(st2->wait_calls == 3U);
    TEST_ASSERT(st2->tokens == 0U);
  }

  // 3d. 回收阶段返回 FAILED 触发 REQUIRE。
  {
    struct Ctx
    {
      WritePort port{4U, 32U};
      Semaphore sem1{0U};
      Semaphore sem2{0U};
      WriteOperation op2{sem2, 1U};
      uint8_t sink[4] = {};
    } ctx;
    ctx.port = NoopWriteFun;
    StatePtr(ctx.sem1)->script.push_back(
        {ErrorCode::TIMEOUT, false, true, 1U, false, false, {}});
    WriteOperation op1(ctx.sem1, 1U);
    TEST_ASSERT(ctx.port(ConstRawData(req1, sizeof(req1)), op1) == ErrorCode::TIMEOUT);

    auto st2 = StatePtr(ctx.sem2);
    st2->script.push_back({ErrorCode::TIMEOUT, false, true, 1U, false, false, [&]
                           {
                             auto wq = ctx.port.GetWriteQueue(true);
                             wq.PopAll(ctx.sink);
                           }});
    st2->script.push_back(DrainFailedStep());

    ExpectFatal(
        +[](void* p)
        {
          auto* c = static_cast<Ctx*>(p);
          const uint8_t d[4] = {9U, 9U, 9U, 9U};
          (void)c->port(ConstRawData(d, sizeof(d)), c->op2);
        },
        &ctx);
    TEST_ASSERT(st2->wait_calls == 2U);
    TEST_ASSERT(st2->tokens == 1U);
  }
}

void TestWritePortWaitForBlock()
{
  const uint8_t payload[4] = {0xA1U, 0xB2U, 0xC3U, 0xD4U};

  // 4a. 未认领初始 FAILED 转入 BLOCK_DETACHED 并返回 TIMEOUT。
  {
    WritePort port(4U, 32U);
    port = NoopWriteFun;
    Semaphore sem(0U);
    auto st = StatePtr(sem);
    st->script.push_back({ErrorCode::FAILED, false, true, 10U, false, false, {}});
    WriteOperation op(sem, 10U);

    TEST_ASSERT(port(ConstRawData(payload, sizeof(payload)), op) == ErrorCode::TIMEOUT);
    TEST_ASSERT(st->wait_calls == 1U);
    TEST_ASSERT(st->tokens == 0U);

    uint8_t sink[4] = {};
    {
      auto wq = port.GetWriteQueue(false);
      wq.PopAll(sink);
    }
    TEST_ASSERT(st->post_calls == 0U);
    TEST_ASSERT(st->tokens == 0U);
  }

  // 4b. 初始 TIMEOUT / FAILED + 严格 UINT32_MAX 回收 TIMEOUT -> OK（PopAll &
  // FailFront）。
  for (ErrorCode init_err : {ErrorCode::TIMEOUT, ErrorCode::FAILED})
  {
    for (ErrorCode settle_ec : {ErrorCode::OK, ErrorCode::FAILED})
    {
      WritePort port(4U, 32U);
      port = NoopWriteFun;
      Semaphore sem(0U);
      auto st = StatePtr(sem);
      uint8_t sink[4] = {};
      st->script.push_back({init_err, false, true, 10U, false, false, [&]
                            {
                              auto wq = port.GetWriteQueue(true);
                              TEST_ASSERT(wq.AvailableSize() == sizeof(payload));
                              if (settle_ec == ErrorCode::OK)
                              {
                                wq.PopAll(sink);
                              }
                              else
                              {
                                wq.FailFront(ErrorCode::FAILED);
                              }
                            }});
      st->script.push_back(DrainTimeoutStep());
      st->script.push_back(DrainOkStep());

      WriteOperation op(sem, 10U);
      TEST_ASSERT(port(ConstRawData(payload, sizeof(payload)), op) == settle_ec);
      if (settle_ec == ErrorCode::OK)
      {
        TEST_ASSERT(std::memcmp(sink, payload, sizeof(payload)) == 0);
      }
      TEST_ASSERT(st->wait_calls == 3U);
      TEST_ASSERT(st->post_calls == 1U);
      TEST_ASSERT(st->tokens == 0U);

      WriteOperation none_op;
      TEST_ASSERT(port(ConstRawData(payload, sizeof(payload)), none_op) == ErrorCode::OK);
    }
  }

  // 4c. 双线程确定性 Post 前窗口（WaitForBlock 认领 BLOCK_CLAIMED 但尚未 Post）。
  {
    WritePort port(4U, 32U);
    port = NoopWriteFun;
    Semaphore sem(0U);
    auto st = StatePtr(sem);
    st->hold_post_until_wait_calls = 3U;
    uint8_t sink[4] = {};
    std::thread drainer;
    WaitStep init_step{};
    init_step.forced_return = ErrorCode::TIMEOUT;
    init_step.consume_token_if_ok = false;
    init_step.check_timeout = true;
    init_step.expected_timeout = 10U;
    init_step.before_return = [&]
    {
      drainer = std::thread(
          [&]
          {
            auto wq = port.GetWriteQueue(true);
            wq.PopAll(sink);
          });
      std::unique_lock<std::mutex> lk(st->mtx);
      st->cv.wait(lk, [&] { return st->post_entered != 0U; });
      TEST_ASSERT(st->tokens == 0U);
    };
    st->script.push_back(init_step);
    st->script.push_back(DrainTimeoutStep());
    st->script.push_back(DrainOkStep(true));

    WriteOperation op(sem, 10U);
    TEST_ASSERT(port(ConstRawData(payload, sizeof(payload)), op) == ErrorCode::OK);
    drainer.join();
    TEST_ASSERT(std::memcmp(sink, payload, sizeof(payload)) == 0);
    TEST_ASSERT(st->wait_calls == 3U);
    TEST_ASSERT(st->tokens == 0U);
  }

  // 4d. 回收阶段 FAILED 触发 REQUIRE。
  {
    struct Ctx
    {
      WritePort port{4U, 32U};
      Semaphore sem{0U};
      WriteOperation op{sem, 10U};
      uint8_t sink[4] = {};
    } ctx;
    ctx.port = NoopWriteFun;
    auto st = StatePtr(ctx.sem);
    st->script.push_back({ErrorCode::TIMEOUT, false, true, 10U, false, false, [&]
                          {
                            auto wq = ctx.port.GetWriteQueue(true);
                            wq.PopAll(ctx.sink);
                          }});
    st->script.push_back(DrainFailedStep());

    ExpectFatal(
        +[](void* p)
        {
          auto* c = static_cast<Ctx*>(p);
          const uint8_t d[4] = {1U, 2U, 3U, 4U};
          (void)c->port(ConstRawData(d, sizeof(d)), c->op);
        },
        &ctx);
    TEST_ASSERT(st->wait_calls == 2U);
    TEST_ASSERT(st->tokens == 1U);
  }
}

void TestTopicSyncSubscriberWait()
{
  auto topic = Topic::CreateTopic<uint32_t>("test_wait_claimed_sync_topic");

  // 5a. 未认领时初始 ThreadX FAILED 退回 WAIT_IDLE 并保留原始 ErrorCode::FAILED。
  {
    uint32_t rx = 0U;
    Topic::SyncSubscriber<uint32_t> sub(topic, rx);
    auto st = StatePtr(sub.block_->data_.sem);
    st->script.push_back({ErrorCode::FAILED, false, true, 10U, false, false, {}});

    TEST_ASSERT(sub.Wait(10U) == ErrorCode::FAILED);
    TEST_ASSERT(st->wait_calls == 1U);
    TEST_ASSERT(st->tokens == 0U);
    TEST_ASSERT(sub.block_->data_.wait_state.load(std::memory_order_acquire) ==
                Topic::SyncBlock::WAIT_IDLE);

    // 5b. 初始 TIMEOUT / FAILED + 严格 UINT32_MAX 回收 TIMEOUT -> OK。
    uint32_t seq = 0x1000U;
    for (ErrorCode init_err : {ErrorCode::TIMEOUT, ErrorCode::FAILED})
    {
      ++seq;
      uint32_t expected_val = seq;
      st->wait_calls = 0U;
      st->post_calls = 0U;
      st->script.push_back({init_err, false, true, 10U, false, false,
                            [&] { topic.Publish(expected_val); }});
      st->script.push_back(DrainTimeoutStep());
      st->script.push_back(DrainOkStep());

      TEST_ASSERT(sub.Wait(10U) == ErrorCode::OK);
      TEST_ASSERT(rx == expected_val);
      TEST_ASSERT(st->wait_calls == 3U);
      TEST_ASSERT(st->post_calls == 1U);
      TEST_ASSERT(st->tokens == 0U);
      TEST_ASSERT(sub.block_->data_.wait_state.load(std::memory_order_acquire) ==
                  Topic::SyncBlock::WAIT_IDLE);
      TEST_ASSERT(sub.Wait(0U) == ErrorCode::TIMEOUT);
    }

    // 5c. 双线程确定性 Post 前窗口：Publish 认领 WAIT_CLAIMED 并阻塞在 Post 前。
    {
      uint32_t expected_val = 0xCAFEBABEU;
      st->wait_calls = 0U;
      st->post_calls = 0U;
      st->post_entered = 0U;
      st->hold_post_until_wait_calls = 3U;
      std::thread publisher;
      WaitStep init_step{};
      init_step.forced_return = ErrorCode::TIMEOUT;
      init_step.consume_token_if_ok = false;
      init_step.check_timeout = true;
      init_step.expected_timeout = 10U;
      init_step.before_return = [&]
      {
        publisher = std::thread([&] { topic.Publish(expected_val); });
        std::unique_lock<std::mutex> lk(st->mtx);
        st->cv.wait(lk, [&] { return st->post_entered != 0U; });
        TEST_ASSERT(st->tokens == 0U);
      };
      st->script.push_back(init_step);
      st->script.push_back(DrainTimeoutStep());
      st->script.push_back(DrainOkStep(true));

      TEST_ASSERT(sub.Wait(10U) == ErrorCode::OK);
      publisher.join();
      TEST_ASSERT(rx == expected_val);
      TEST_ASSERT(st->wait_calls == 3U);
      TEST_ASSERT(st->tokens == 0U);
      st->hold_post_until_wait_calls = 0U;
    }
  }

  // 5d. 已认领回收阶段返回 FAILED 必须触发 REQUIRE。
  {
    struct Ctx
    {
      Topic topic;
      uint32_t rx = 0U;
      uint32_t payload = 0xDEADBEEFU;
      Topic::SyncSubscriber<uint32_t> sub;
      Ctx(Topic t) : topic(t), sub(t, rx) {}
    } ctx(topic);
    auto st = StatePtr(ctx.sub.block_->data_.sem);
    st->script.push_back({ErrorCode::TIMEOUT, false, true, 10U, false, false,
                          [&] { ctx.topic.Publish(ctx.payload); }});
    st->script.push_back(DrainFailedStep());

    ExpectFatal(
        +[](void* p)
        {
          auto* c = static_cast<Ctx*>(p);
          (void)c->sub.Wait(10U);
        },
        &ctx);
    TEST_ASSERT(st->wait_calls == 2U);
    TEST_ASSERT(st->tokens == 1U);
    TEST_ASSERT(ctx.rx == 0xDEADBEEFU);
  }
}
}  // namespace

int main()
{
  TestAsyncBlockWait();
  TestReadPortWaitForBlock();
  TestWritePortWaitForRetirement();
  TestWritePortWaitForBlock();
  TestTopicSyncSubscriberWait();
  std::puts(
      "wait_claimed_notification: all 5 call sites (initial FAILED, pre-Post claim "
      "window, Wait(UINT32_MAX) drain TIMEOUT->OK, drain FAILED->REQUIRE) passed");
  return 0;
}
