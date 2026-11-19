/**
 * @file test_write_stream.cpp
 * @brief 检查 Stream 阻塞提交、析构提交和超时后的完成处理。 /
 * Tests blocking Stream commit, destructor submission and completion after timeout.
 *
 * 检查错误返回和信号量中没有遗留通知。
 * Checks returned errors and the absence of leftover semaphore tokens.
 */

#include "core/rw/rw_port_test_common.hpp"
#include "test_assert.hpp"

namespace
{
enum class StreamSubmitMode : uint8_t
{
  COMMIT,
  DESTRUCT
};

void VerifyStreamBlockPendingCompletion(LibXR::ErrorCode finish_result,
                                        LibXR::ErrorCode expected_result,
                                        StreamSubmitMode submit_mode)
{
  // 完成线程模拟后端取走请求；Commit 检查返回值，析构提交检查通知已被消费。
  // A worker consumes the request; check Commit results or a consumed token after
  // destruction.
  using namespace LibXR;

  WritePort w(2, 16);
  w = PendingWriteFun;

  static const uint8_t TX[] = {0x41, 0x42, 0x43, 0x44};
  Semaphore sem;
  WriteOperation op(sem, SHORT_WAIT_MS);
  Semaphore done;
  Thread finisher;
  StartWriteFinisher(
      finisher, w, done, finish_result,
      (submit_mode == StreamSubmitMode::COMMIT) ? "wr_stream_commit" : "wr_stream_dtor");

  {
    WritePort::Stream ws(&w, op);
    ws << ConstRawData{TX, sizeof(TX)};
    if (submit_mode == StreamSubmitMode::COMMIT)
    {
      auto ec = ws.Commit();
      TEST_ASSERT(ec == expected_result);
    }
  }

  ExpectWaitOk(done, SHORT_WAIT_MS);
  JoinThreadIfNeeded(finisher);
  TEST_ASSERT(sem.Value() == 0);
}

void VerifyStreamBlockTimeout()
{
  // 超时只结束等待，已经复制的请求仍由后端处理；迟到完成不能再通知旧等待者。
  // Timeout ends the wait, not the copied request; late completion must not notify the
  // old waiter.
  using namespace LibXR;

  WritePort w(2, 16);
  w = PendingWriteFun;

  static const uint8_t TX[] = {0x51, 0x52, 0x53};
  Semaphore sem;
  WriteOperation op(sem, 0);
  WritePort::Stream ws(&w, op);
  ws << ConstRawData{TX, sizeof(TX)};

  auto ec = ws.Commit();
  TEST_ASSERT(ec == ErrorCode::TIMEOUT);
  TEST_ASSERT(sem.Value() == 0);

  {
    auto queue = w.GetWriteQueue(false);
    TEST_ASSERT(!queue.Empty());
    static uint8_t sink[16];
    queue.PopAll(sink);
  }

  TEST_ASSERT(sem.Value() == 0);
}

void VerifyStreamZeroShiftWithoutOwnership(TestMode mode)
{
  // 未持有写入权时，operator<< 写入零长度数据不能占用 LOCKED，后续 Commit 和析构
  // 也不得触发额外完成通知或轮询 DONE 跳变。
  // Zero-length operator<< without ownership must not claim LOCKED or trigger extra
  // callbacks or POLLING DONE transitions on later Commit or destruction.
  using namespace LibXR;

  WritePort w(2, 16);
  w = PendingWriteFun;
  static const uint8_t TX[] = {0x61, 0x62};
  static const uint8_t OTHER[] = {0x73};

  // 1) After Commit(), stream no longer owns the port.
  {
    WriteHarness harness(mode);
    WritePort::Stream ws(&w, harness.op);
    TEST_ASSERT(ws.Write(ConstRawData{TX, sizeof(TX)}) == ErrorCode::OK);
    TEST_ASSERT(ws.Commit() == ErrorCode::OK);
    {
      auto queue = w.GetWriteQueue(false);
      TEST_ASSERT(!queue.Empty());
      uint8_t sink[16];
      queue.PopAll(sink);
    }
    harness.ExpectFinal(ErrorCode::OK);
    harness.Reset();

    ws << ConstRawData{nullptr, 0} << ConstRawData{TX, 0};
    TEST_ASSERT(ws.EmptySize() == 0U);

    WriteOperation other_op;
    TEST_ASSERT(w(ConstRawData{OTHER, sizeof(OTHER)}, other_op) == ErrorCode::OK);
    {
      auto queue = w.GetWriteQueue(false);
      TEST_ASSERT(!queue.Empty());
      uint8_t sink[16];
      queue.PopAll(sink);
    }

    TEST_ASSERT(ws.Commit() == ErrorCode::OK);
    if (mode == TestMode::POLLING)
    {
      TEST_ASSERT(harness.polling_status.load(std::memory_order_acquire) ==
                  WriteOperation::OperationPollingStatus::READY);
    }
    else if (mode == TestMode::CALLBACK)
    {
      TEST_ASSERT(harness.probe.count.load(std::memory_order_acquire) == 0U);
    }
  }

  // 2) Construction fails Acquire() while the port is busy; after holder releases,
  //    zero-length operator<< still must not claim LOCKED or notify on Commit/dtor.
  WriteHarness check_harness(mode);
  {
    WriteOperation holder_op;
    WritePort::Stream holder(&w, holder_op);
    WritePort::Stream ws(&w, check_harness.op);
    TEST_ASSERT(ws.EmptySize() == 0U);
    TEST_ASSERT(holder.Commit() == ErrorCode::OK);

    ws << ConstRawData{nullptr, 0} << ConstRawData{TX, 0};
    TEST_ASSERT(ws.EmptySize() == 0U);

    WriteOperation other_op;
    TEST_ASSERT(w(ConstRawData{OTHER, sizeof(OTHER)}, other_op) == ErrorCode::OK);
    {
      auto queue = w.GetWriteQueue(false);
      TEST_ASSERT(!queue.Empty());
      uint8_t sink[16];
      queue.PopAll(sink);
    }

    TEST_ASSERT(ws.Commit() == ErrorCode::OK);
  }

  if (mode == TestMode::POLLING)
  {
    TEST_ASSERT(check_harness.polling_status.load(std::memory_order_acquire) ==
                WriteOperation::OperationPollingStatus::READY);
  }
  else if (mode == TestMode::CALLBACK)
  {
    TEST_ASSERT(check_harness.probe.count.load(std::memory_order_acquire) == 0U);
  }
}
}  // namespace

void test_write_stream()
{
  VerifyStreamBlockPendingCompletion(LibXR::ErrorCode::FAILED, LibXR::ErrorCode::FAILED,
                                     StreamSubmitMode::COMMIT);
  VerifyStreamBlockTimeout();
  VerifyStreamBlockPendingCompletion(LibXR::ErrorCode::OK, LibXR::ErrorCode::OK,
                                     StreamSubmitMode::DESTRUCT);
  VerifyStreamZeroShiftWithoutOwnership(TestMode::POLLING);
  VerifyStreamZeroShiftWithoutOwnership(TestMode::CALLBACK);
}
