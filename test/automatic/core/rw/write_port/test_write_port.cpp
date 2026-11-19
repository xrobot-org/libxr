/**
 * @file test_write_port.cpp
 * @brief 检查 WritePort 的容量限制、完成结果和阻塞等待。 /
 * Tests WritePort capacity limits, completion results and blocking waits.
 *
 * 区分入队与后端完成，并检查超时后迟到完成不会通知旧等待者。
 * Distinguishes admission from completion and checks detached timeout waiters.
 */

#include "core/rw/rw_port_test_common.hpp"
#include "test_assert.hpp"

namespace
{
void VerifyPendingWriteMode(TestMode mode, LibXR::ErrorCode result)
{
  // 入队成功不等于后端完成；取走或拒绝队首请求后，才检查最终结果。
  // Admission is not completion; consume or fail the front request before checking its
  // result.
  using namespace LibXR;

  WritePort w(2, 16);
  w = PendingWriteFun;

  std::vector<uint8_t> tx = {0x31, 0x41, 0x59, 0x26};
  WriteHarness write(mode);

  if (mode == TestMode::BLOCK)
  {
    Semaphore done;
    Thread finisher;
    StartWriteFinisher(finisher, w, done, result, "wr_finish");

    auto block_result = w(ConstRawData{tx.data(), tx.size()}, write.op);
    TEST_ASSERT(block_result == result);

    ExpectWaitOk(done);
    JoinThreadIfNeeded(finisher);
    TEST_ASSERT(w.Size() == 0);
    return;
  }

  auto call_result = w(ConstRawData{tx.data(), tx.size()}, write.op);
  TEST_ASSERT(call_result == ErrorCode::OK);

  {
    auto queue = w.GetWriteQueue(false);
    TEST_ASSERT(!queue.Empty());
    if (result == ErrorCode::OK)
    {
      static uint8_t sink[16];
      TEST_ASSERT(queue.AvailableSize() <= sizeof(sink));
      queue.PopAll(sink);
    }
    else
    {
      queue.FailFront(result);
    }
  }
  if (mode != TestMode::NONE)
  {
    write.ExpectFinal(result);
  }
  TEST_ASSERT(w.Size() == 0);
}

void test_rw_block_write_timeout_detaches_waiter()
{
  // 每次超时后才让后端处理请求；迟到完成不应留下信号量通知。
  // Consume each request after its timeout; late completion must leave no semaphore
  // token.
  using namespace LibXR;

  WritePort w(2, 64);
  w = PendingWriteFun;

  static const uint8_t TX1[] = {1, 2, 3};
  static const uint8_t TX2[] = {4, 5, 6};

  Semaphore sem1;
  WriteOperation op1(sem1, 0);
  auto ec = w(ConstRawData{TX1, sizeof(TX1)}, op1);
  TEST_ASSERT(ec == ErrorCode::TIMEOUT);
  TEST_ASSERT(sem1.Value() == 0);

  static uint8_t sink[sizeof(TX1)] = {};
  {
    auto queue = w.GetWriteQueue(false);
    TEST_ASSERT(!queue.Empty());
    queue.PopAll(sink);
  }

  Semaphore sem2;
  WriteOperation op2(sem2, 0);
  ec = w(ConstRawData{TX2, sizeof(TX2)}, op2);
  TEST_ASSERT(ec == ErrorCode::TIMEOUT);
  TEST_ASSERT(sem2.Value() == 0);

  static uint8_t sink2[sizeof(TX2)] = {};
  {
    auto queue = w.GetWriteQueue(false);
    TEST_ASSERT(!queue.Empty());
    queue.PopAll(sink2);
  }
  TEST_ASSERT(sem1.Value() == 0);
  TEST_ASSERT(sem2.Value() == 0);
}

void test_rw_write_port_block_pending_result_propagates()
{
  // 后端完成线程报错时，阻塞写应返回同一个错误。
  // A blocking write must return the error supplied by the backend completion worker.
  using namespace LibXR;

  WritePort w(2, 16);
  w = PendingWriteFun;

  static const uint8_t TX[] = {0x5A};
  Semaphore sem;
  WriteOperation op(sem, 100);
  Semaphore done;
  Thread finisher;
  StartWriteFinisher(finisher, w, done, ErrorCode::FAILED, "wr_finish");

  auto ec = w(ConstRawData{TX, sizeof(TX)}, op);
  TEST_ASSERT(ec == ErrorCode::FAILED);
  ExpectWaitOk(done, SHORT_WAIT_MS);
  JoinThreadIfNeeded(finisher);
}

void test_rw_write_port_block_reused_waiter_discards_stale_signal()
{
  // 同一信号量先等一次失败、再等一次成功，第二次不能误用第一次的通知。
  // Reuse one semaphore for failure then success; the second wait must not consume an old
  // token.
  using namespace LibXR;

  WritePort w(2, 16);
  w = PendingWriteFun;

  static const uint8_t TX1[] = {0x6B};
  static const uint8_t TX2[] = {0x7C};
  Semaphore sem;
  WriteOperation op(sem, 100);
  Semaphore done1;
  Thread finisher1;
  StartWriteFinisher(finisher1, w, done1, ErrorCode::FAILED, "wr_stale1");

  auto ec = w(ConstRawData{TX1, sizeof(TX1)}, op);
  TEST_ASSERT(ec == ErrorCode::FAILED);
  ExpectWaitOk(done1, SHORT_WAIT_MS);
  JoinThreadIfNeeded(finisher1);
  TEST_ASSERT(sem.Value() == 0);

  Semaphore done2;
  Thread finisher2;
  StartWriteFinisher(finisher2, w, done2, ErrorCode::OK, "wr_stale2");

  ec = w(ConstRawData{TX2, sizeof(TX2)}, op);
  TEST_ASSERT(ec == ErrorCode::OK);
  ExpectWaitOk(done2, SHORT_WAIT_MS);
  JoinThreadIfNeeded(finisher2);
  TEST_ASSERT(sem.Value() == 0);
}

struct RetirementRaceContext
{
  LibXR::WritePort* port;
  std::atomic<uint32_t>* stage;
  LibXR::Semaphore* round_done;
  uint32_t iterations;
};

void RunRetirementRaceConsumer(RetirementRaceContext ctx)
{
  using namespace LibXR;

  uint8_t sink[16];
  for (uint32_t iter = 0; iter < ctx.iterations; ++iter)
  {
    const uint32_t target = iter + 1U;
    while (ctx.stage->load(std::memory_order_acquire) != target)
    {
      Thread::Yield();
    }

    if ((iter & 1U) != 0U)
    {
      Thread::Yield();
    }

    for (uint8_t step = 0; step < 2U; ++step)
    {
      for (;;)
      {
        auto queue = ctx.port->GetWriteQueue(false);
        if (queue.Empty())
        {
          Thread::Yield();
          continue;
        }
        TEST_ASSERT(queue.AvailableSize() <= sizeof(sink));
        queue.PopAll(sink);
        break;
      }
    }

    ctx.round_done->Post();
  }
}

void test_rw_write_port_block_detached_retirement_race()
{
  // 旧 BLOCK 超时进入 BLOCK_DETACHED 后，后端释放到 IDLE 与后续 BLOCK 并发时不能返回
  // BUSY，且超过 Capacity() 的后续 BLOCK 在未退队前仍优先返回 BUSY。
  // Racing backend retirement from BLOCK_DETACHED to IDLE against a follow-up BLOCK
  // write within Capacity() must never return BUSY; oversized BLOCK still returns BUSY
  // while detached.
  using namespace LibXR;

  WritePort w(2, 16);
  w = PendingWriteFun;

  static const uint8_t DETACHED_TX[] = {0x11};
  static const uint8_t OVERSIZED_TX[17] = {0x22};
  static const uint8_t FOLLOW_UP_TX[16] = {
      0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
      0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10,
  };

  {
    Semaphore sem_detached;
    WriteOperation detached_op(sem_detached, 0);
    TEST_ASSERT(w(ConstRawData{DETACHED_TX, sizeof(DETACHED_TX)}, detached_op) ==
                ErrorCode::TIMEOUT);

    Semaphore sem_oversized;
    WriteOperation oversized_op(sem_oversized, ASYNC_TIMEOUT_MS);
    TEST_ASSERT(w(ConstRawData{OVERSIZED_TX, sizeof(OVERSIZED_TX)}, oversized_op) ==
                ErrorCode::BUSY);
    TEST_ASSERT(sem_oversized.Value() == 0);

    WriteOperation async_op;
    TEST_ASSERT(w(ConstRawData{DETACHED_TX, sizeof(DETACHED_TX)}, async_op) ==
                ErrorCode::BUSY);

    uint8_t sink[16];
    auto queue = w.GetWriteQueue(false);
    TEST_ASSERT(!queue.Empty());
    queue.PopAll(sink);
    TEST_ASSERT(sem_detached.Value() == 0);
  }

  constexpr uint32_t ITERATIONS = 128U;
  std::atomic<uint32_t> stage{0U};
  Semaphore round_done;
  Thread consumer;
  consumer.Create(RetirementRaceContext{&w, &stage, &round_done, ITERATIONS},
                  RunRetirementRaceConsumer, "wr_retire_race", 1024,
                  Thread::Priority::MEDIUM);

  Semaphore sem1;
  Semaphore sem2;
  for (uint32_t iter = 0; iter < ITERATIONS; ++iter)
  {
    WriteOperation op1(sem1, 0);
    TEST_ASSERT(w(ConstRawData{DETACHED_TX, sizeof(DETACHED_TX)}, op1) ==
                ErrorCode::TIMEOUT);
    TEST_ASSERT(sem1.Value() == 0);

    stage.store(iter + 1U, std::memory_order_release);
    if ((iter % 3U) == 1U)
    {
      Thread::Yield();
    }

    const size_t follow_up_size = (iter % 4U == 0U) ? w.Capacity() : 4U;
    WriteOperation op2(sem2, ASYNC_TIMEOUT_MS);
    const ErrorCode ec = w(ConstRawData{FOLLOW_UP_TX, follow_up_size}, op2);
    TEST_ASSERT(ec != ErrorCode::BUSY);
    TEST_ASSERT(ec == ErrorCode::OK);

    ExpectWaitOk(round_done, ASYNC_TIMEOUT_MS);
    TEST_ASSERT(sem1.Value() == 0);
    TEST_ASSERT(sem2.Value() == 0);
    TEST_ASSERT(w.Size() == 0);
  }

  JoinThreadIfNeeded(consumer);
}
}  // namespace

void test_write_port()
{
  using namespace LibXR;

  for (auto mode : ASYNC_MODES)
  {
    VerifyPendingWriteMode(mode, ErrorCode::FAILED);
  }

  WritePort pending(1, 4);
  pending = PendingWriteFun;
  const uint8_t second[] = {5};
  WriteOperation first_op;
  WriteOperation second_op;
  std::vector<uint8_t> first(pending.EmptySize(), 0x3C);
  TEST_ASSERT(!first.empty());
  TEST_ASSERT(pending(ConstRawData{first.data(), first.size()}, first_op) ==
              ErrorCode::OK);
  TEST_ASSERT(pending(ConstRawData{second, sizeof(second)}, second_op) ==
              ErrorCode::FULL);
  {
    auto queue = pending.GetWriteQueue(false);
    TEST_ASSERT(!queue.Empty());
    static uint8_t sink[4];
    queue.PopAll(sink);
  }

  uint8_t byte = 0;
  WritePort unconfigured(2, 1);
  WriteOperation write_op;
  TEST_ASSERT(unconfigured(ConstRawData{&byte, 1}, write_op) == ErrorCode::NOT_SUPPORT);
  WritePort full(2, 1);
  full = PendingWriteFun;
  const uint8_t pair[2] = {0x55, 0x66};
  TEST_ASSERT(full(ConstRawData{pair, sizeof(pair)}, write_op) == ErrorCode::FULL);
  TEST_ASSERT(full.Size() == 0);

  test_rw_block_write_timeout_detaches_waiter();
  test_rw_write_port_block_pending_result_propagates();
  test_rw_write_port_block_reused_waiter_discards_stale_signal();
  test_rw_write_port_block_detached_retirement_race();
}
