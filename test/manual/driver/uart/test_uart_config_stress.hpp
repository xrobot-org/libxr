/**
 * @file test_uart_config_stress.hpp
 * @brief UART 并发配置压力测试 / UART concurrent configuration stress test.
 *
 * 收发期间借用 ASync 反复提交配置，检查请求完成及停止干扰后的恢复。
 * Use a borrowed ASync to submit configurations during I/O; check completion and
 * recovery afterward. Payload loss or corruption during switching is allowed.
 */
#pragma once

#include "async.hpp"
#include "test_uart.hpp"

namespace LibXR::Test
{
/** @brief 压力阶段的观测计数 / Observed counts from the stress phase. */
struct UARTConfigStressResult
{
  uint32_t config_ok = 0;    ///< 已接纳配置 / Accepted configurations.
  uint32_t config_busy = 0;  ///< 配置忙时拒绝 / Busy configuration submissions.
  uint32_t during_read =
      0;  ///< 读取调用期间的配置尝试 / Config attempts during a read call.
  uint32_t write_ok = 0;        ///< 已接纳发送 / Accepted writes.
  uint32_t write_rejected = 0;  ///< BUSY 或 FULL 拒绝 / Writes rejected as BUSY or FULL.
  uint32_t write_failed = 0;    ///< 已完成但失败的发送 / Writes completed with failure.
  uint32_t read_ok = 0;  ///< 完成的读取，不校验内容 / Completed reads, payload unchecked.
  uint32_t read_timeout = 0;  ///< 读取超时 / Read timeouts.
};

/**
 * @brief 检查并发配置后的 UART 恢复 / Check UART recovery after concurrent configuration.
 * @pre 后端支持配置与收发重叠；TX 接 RX，独占端口，初始无遗留请求或接收数据。
 *      The backend must support configuration overlapping I/O. Wire TX to RX and
 *      reserve both ports, initially with no pending requests or received bytes.
 * @pre tx/rx 为独立等长工作 RAM，满足后端要求且各自不超过两个端口容量。
 *      tx/rx are separate equal-size work RAM meeting backend requirements and both
 *      port capacities. They must not overlap active backend buffers.
 * @param uart 已初始化的串口 / Initialized UART.
 * @param worker 独占借用的 READY 状态 ASync / Exclusively borrowed READY ASync.
 * @param tx 发送工作区，其大小为每包长度 / TX work buffer; its size is the packet length.
 * @param rx 接收工作区 / RX work buffer.
 * @param configs 每轮轮换起点的合法配置列表，尝试间隔 1 ms / Legal configurations tried
 * 1 ms apart, rotating the starting entry each round.
 * @param stable_config 开始及结束时的固定配置，须为 8 位数据 / Initial and final
 * configuration, with 8 data bits.
 * @param quiet_ms 停止干扰后留给线路排空和配置生效的时间，由调用方确定 /
 *        Caller-chosen time for wire drain and configuration application after
 * interference.
 * @param timeout_ms 每次读及各收敛等待的上限，不强制中断同步后端调用 /
 *        Timeout for each read and convergence wait; cannot preempt synchronous backend
 * calls.
 * @param iterations 压力轮数 / Stress rounds.
 * @return 压力阶段统计；返回前已通过固定配置回环 / Stress counts; fixed-config loopback
 * passes before return.
 */
inline UARTConfigStressResult TestUARTConfigStress(
    UART& uart, ASync& worker, RawData tx, RawData rx,
    std::initializer_list<UART::Configuration> configs, UART::Configuration stable_config,
    uint32_t quiet_ms, uint32_t timeout_ms = 1000, uint32_t iterations = 100)
{
  TEST_ASSERT(worker.GetStatus() == ASync::Status::READY);
  TEST_ASSERT(iterations > 0 && configs.size() > 0 &&
              configs.size() <= UINT32_MAX / iterations);
  TEST_ASSERT(timeout_ms > 0 && timeout_ms < UINT32_MAX / 2U);
  TEST_ASSERT(quiet_ms > 0 && quiet_ms < UINT32_MAX / 2U && stable_config.data_bits == 8);
  TEST_ASSERT(uart.read_port_ && uart.write_port_);
  TEST_ASSERT(uart.read_port_->Readable() && uart.write_port_->Writable());
  TEST_ASSERT(tx.addr_ && rx.addr_ && tx.size_ > 0 && tx.size_ == rx.size_);
  TEST_ASSERT(tx.size_ <= uart.read_port_->Capacity() &&
              tx.size_ <= uart.write_port_->Capacity());
  const auto tx_begin = reinterpret_cast<uintptr_t>(tx.addr_);
  const auto rx_begin = reinterpret_cast<uintptr_t>(rx.addr_);
  TEST_ASSERT(tx_begin >= rx_begin ? tx_begin - rx_begin >= rx.size_
                                   : rx_begin - tx_begin >= tx.size_);
  TEST_ASSERT(uart.read_port_->Size() == 0);
  TEST_ASSERT(uart.SetConfig(stable_config) == ErrorCode::OK);
  Thread::Sleep(quiet_ms);

  struct Context
  {
    UART& uart;
    std::initializer_list<UART::Configuration> configs;
    UARTConfigStressResult result;
    size_t first_config = 0;
    std::atomic<uint32_t> reading{0}, tx_calls{0}, tx_failed{0}, bad_result{0};
  } context{uart, configs, {}};
  auto config_job = ASync::Job::Create(
      [](bool, Context* self, ASync*)
      {
        // 有限任务独立结束；裸机 Timer 中不能等调用线程发出停止信号。
        // Finish independently; a cooperative Timer job cannot wait for its caller.
        size_t index = self->first_config;
        for (size_t i = 0; i < self->configs.size(); ++i)
        {
          const bool reading = self->reading.load(std::memory_order_acquire) != 0;
          const auto ec = self->uart.SetConfig(self->configs.begin()[index]);
          TEST_ASSERT(ec == ErrorCode::OK || ec == ErrorCode::BUSY);
          if (ec == ErrorCode::OK)
            ++self->result.config_ok;
          else
            ++self->result.config_busy;
          if (reading && self->reading.load(std::memory_order_acquire) != 0)
            ++self->result.during_read;
          if (++index == self->configs.size()) index = 0;
          Thread::Sleep(1);
        }
      },
      &context);
  auto callback = WriteOperation::Callback::Create(
      [](bool, Context* self, ErrorCode ec)
      {
        if (ec == ErrorCode::FAILED)
          self->tx_failed.fetch_add(1, std::memory_order_relaxed);
        else if (ec != ErrorCode::OK)
          self->bad_result.store(1, std::memory_order_relaxed);
        // 最后一次访问上下文才发布计数，ISR 不打印或断言。
        // Publish on the last context access; no logging or assertions in ISR.
        self->tx_calls.fetch_add(1, std::memory_order_release);
      },
      &context);
  WriteOperation write(callback);
  Semaphore read_sem;
  ReadOperation read(read_sem, timeout_ms);
  auto wait_until = [&](auto finished)
  {
    const uint32_t start = Thread::GetTime();
    while (!finished())
    {
      TEST_ASSERT(Thread::GetTime() - start < timeout_ms);
      Thread::Sleep(1);
    }
  };
  auto check_tx = [&]()
  {
    const uint32_t calls = context.tx_calls.load(std::memory_order_acquire);
    TEST_ASSERT(calls <= context.result.write_ok);
    TEST_ASSERT(context.bad_result.load(std::memory_order_relaxed) == 0);
    return calls == context.result.write_ok;
  };
  for (uint32_t round = 0; round < iterations; ++round)
  {
    auto* sent = static_cast<uint8_t*>(tx.addr_);
    for (size_t i = 0; i < tx.size_; ++i) sent[i] = (i * 37U + round) & 0x7FU;
    const auto written = uart.Write({tx.addr_, tx.size_}, write);
    TEST_ASSERT(written == ErrorCode::OK || written == ErrorCode::BUSY ||
                written == ErrorCode::FULL);
    if (written == ErrorCode::OK)
      ++context.result.write_ok;
    else
      ++context.result.write_rejected;
    // 轮换起点，避免总由同一配置占住唯一待处理槽。
    // Rotate the first attempt so the same configuration does not always claim the slot.
    context.first_config = round % configs.size();
    TEST_ASSERT(worker.AssignJob(config_job) == ErrorCode::OK);
    context.reading.store(1, std::memory_order_release);
    const auto received = uart.Read(rx, read);
    context.reading.store(0, std::memory_order_release);
    TEST_ASSERT(received == ErrorCode::OK || received == ErrorCode::TIMEOUT);
    if (received == ErrorCode::OK)
      ++context.result.read_ok;
    else
      ++context.result.read_timeout;
    wait_until(
        [&]()
        {
          const auto status = worker.GetStatus();
          TEST_ASSERT(status == ASync::Status::BUSY || status == ASync::Status::DONE);
          return status == ASync::Status::DONE;
        });
    // 发送用回调跟踪到终态；不把 BLOCK Write 超时误当作请求退出。
    // Track TX to terminal completion instead of treating BLOCK timeout as retirement.
    wait_until(check_tx);
  }
  TEST_ASSERT(context.result.config_ok > 0 && context.result.during_read > 0 &&
              context.result.write_ok > 0);
  wait_until(
      [&]()
      {
        const auto ec = uart.SetConfig(stable_config);
        TEST_ASSERT(ec == ErrorCode::OK || ec == ErrorCode::BUSY);
        return ec == ErrorCode::OK;
      });
  // 所有请求已结束，再等线路排空，最后丢弃切换期间的接收残留。
  // After request completion, wait for wire drain before discarding switching residue.
  Thread::Sleep(quiet_ms);
  TEST_ASSERT(uart.read_port_->ClearQueuedData() == ErrorCode::OK);
  TestUARTLoopback(uart, tx, rx, {tx.size_}, timeout_ms, 1);
  TEST_ASSERT(check_tx());
  context.result.write_failed = context.tx_failed.load(std::memory_order_relaxed);
  return context.result;
}
}  // namespace LibXR::Test
