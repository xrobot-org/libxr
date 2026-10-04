/**
 * @file test_uart.hpp
 * @brief UART 接线回环测试 / Wired UART loopback test.
 *
 * 反复收发不同长度的数据，检查阻塞、轮询、回调完成、挂起读取和发送源复用。
 * Repeat different packet lengths; check blocking, polling and callback completion,
 * pending reads, and reuse of the caller's transmit buffer after Write returns.
 * 配置测试另用阻塞收发检查数据及总耗时 / A separate configuration test checks
 * data and total transfer time using blocking operations.
 */
#pragma once

#include <initializer_list>

#include "test_assert.hpp"
#include "timebase.hpp"
#include "transfer_wait.hpp"
#include "uart.hpp"

namespace LibXR::Test
{
/**
 * @brief 检查不同长度下的串口收发回环 / Check UART loopback at selected lengths.
 * @pre 配置为 8 位数据，TX 接 RX，RX 不接其他输出；独占串口且初始接收队列为空。
 *      Configure 8-bit data, connect TX to RX without other outputs, reserve the UART,
 *      and start with an empty receive queue.
 * @pre tx/rx 是独立工作 RAM，不能与彼此或后端正在使用的缓冲区重叠。
 *      tx/rx must be separate work RAM, disjoint from each other and active backend
 * buffers.
 * @param uart 已初始化且可读写的串口 / Initialized UART with both directions enabled.
 * @param semaphore 调用方保留的 BLOCK 完成信号量，收发共用 / Caller-retained semaphore
 * for BLOCK completion, shared by TX and RX.
 * @param tx 发送工作缓冲区，内容会被覆盖 / Transmit work buffer; contents are
 * overwritten.
 * @param rx 接收工作缓冲区，内容会被覆盖 / Receive work buffer; contents are overwritten.
 * @param lengths 每轮的正字节数列表，须同时适合工作缓冲区及收发端口容量 /
 *        Positive byte lengths per round, within both work buffers and port capacities.
 * @param timeout_ms 每次 BLOCK 调用及各操作返回后的完成等待上限 /
 *        Timeout for each BLOCK call and completion wait measured from each submission's
 * return.
 * @param iterations 重复轮数，每个长度测试三种完成方式 /
 *        Rounds; each length uses all three completion modes.
 */
inline void TestUARTLoopback(UART& uart, Semaphore& semaphore, RawData tx, RawData rx,
                             std::initializer_list<size_t> lengths,
                             uint32_t timeout_ms = 1000, uint32_t iterations = 100)
{
  TEST_ASSERT(uart.read_port_ != nullptr && uart.write_port_ != nullptr);
  TEST_ASSERT(uart.read_port_->Readable() && uart.write_port_->Writable());
  TEST_ASSERT(tx.addr_ != nullptr && rx.addr_ != nullptr);
  TEST_ASSERT(iterations > 0 && lengths.size() > 0 &&
              lengths.size() <= UINT32_MAX / iterations);
  const auto tx_begin = reinterpret_cast<uintptr_t>(tx.addr_);
  const auto rx_begin = reinterpret_cast<uintptr_t>(rx.addr_);
  TEST_ASSERT(tx_begin >= rx_begin ? tx_begin - rx_begin >= rx.size_
                                   : rx_begin - tx_begin >= tx.size_);
  for (size_t length : lengths)
  {
    TEST_ASSERT(length > 0 && length <= tx.size_ && length <= rx.size_);
    TEST_ASSERT(length <= uart.read_port_->Capacity() &&
                length <= uart.write_port_->Capacity());
  }
  auto* sent = static_cast<uint8_t*>(tx.addr_);
  auto* received = static_cast<uint8_t*>(rx.addr_);
  // BLOCK 发送和接收在本线程依次完成，可共用一个信号量。
  // BLOCK TX and RX complete in turn on this thread and can share the semaphore.
  Detail::TransferTestCompletion tx_completion(semaphore, timeout_ms),
      rx_completion(semaphore, timeout_ms);
  for (uint32_t round = 0; round < iterations; ++round)
  {
    for (size_t length : lengths)
    {
      for (unsigned mode = 0; mode < 3; ++mode)
      {
        TEST_ASSERT(uart.read_port_->Size() == 0);
        const auto seed = static_cast<uint8_t>(round + length + mode * 85U);
        auto expected = [&](size_t i) { return static_cast<uint8_t>((i % 251U) ^ seed); };
        for (size_t i = 0; i < length; ++i)
        {
          sent[i] = expected(i);
          received[i] = static_cast<uint8_t>(~expected(i));
        }
        auto read = [&](ReadOperation& op) { return uart.Read({received, length}, op); };
        // 非阻塞方式先挂读取；BLOCK 先发再读，避免同一线程等待尚未发出的数据。
        // Arm nonblocking reads first; BLOCK writes first to avoid waiting on itself.
        if (mode != 0)
        {
          rx_completion.Start(mode, read);
        }
        tx_completion.Start(
            mode, [&](WriteOperation& op) { return uart.Write({sent, length}, op); });
        // Write 返回后立即复用源缓冲区，回读仍应是提交时的数据。
        // Reuse the source immediately after Write returns; RX must retain the original.
        for (size_t i = 0; i < length; ++i)
        {
          sent[i] = static_cast<uint8_t>(~expected(i));
        }
        if (mode == 0)
        {
          rx_completion.Start(mode, read);
        }
        tx_completion.Wait();
        rx_completion.Wait();
        for (size_t i = 0; i < length; ++i)
        {
          TEST_ASSERT(received[i] == expected(i));
        }
        TEST_ASSERT(uart.read_port_->Size() == 0);
        tx_completion.Check();
        rx_completion.Check();
      }
    }
  }
}
/**
 * @brief 检查配置后的回环数据和传输耗时 / Check loopback data and timing after
 * configuration.
 * @pre 沿用回环接线及独立缓冲区要求；从普通任务调用，串口空闲，微秒时间基已初始化。
 *      Use the same loopback wiring and separate buffers; call from a normal task with
 *      an idle UART and an initialized microsecond timebase.
 * @pre 后端须在空闲配置请求之后，按新配置执行后续收发。
 *      The backend must use the accepted idle configuration for subsequent transfers.
 * @param uart 已初始化的串口 / Initialized UART.
 * @param semaphore 调用方保留的 BLOCK 完成信号量，收发共用 / Caller-retained semaphore
 * for BLOCK completion, shared by TX and RX.
 * @param config 后端支持的配置，数据位数不超过 8 / Supported configuration with at most 8
 * data bits.
 * @param tx 发送工作区，其大小为每包长度 / Transmit work buffer; its size is the packet
 * length.
 * @param rx 等长的接收工作区 / Receive work buffer of equal size.
 * @param min_elapsed_us 所有包传输耗时之和的下限，单位微秒 / Lower bound on summed
 * transfer time in microseconds.
 * @param max_elapsed_us 总耗时上限，包含调用、调度及协议间隙 / Upper bound including
 * call, scheduling and protocol gaps.
 * @param iterations 数据包数量 / Number of packets.
 * @param timeout_ms 每次阻塞读写的超时 / Timeout for each blocking read or write.
 * @return 实测的传输耗时之和，单位微秒 / Measured sum of transfer times in microseconds.
 * @note 正常返回后保留新配置及工作区内容；重复调用可检查同配置重设和 A-B-A 切换。
 *       Retains the new configuration and buffer contents. Repeat calls for same-config
 *       setup and A-B-A switching. Timing alone cannot identify every framing setting.
 * @note 接收数据只比较配置指定的有效低位，缓冲区中的原始高位保持不变。
 *       Compare only the configured low data bits; retain the raw upper bits in RX.
 */
inline uint64_t TestUARTConfig(UART& uart, Semaphore& semaphore,
                               UART::Configuration config, RawData tx, RawData rx,
                               uint64_t min_elapsed_us, uint64_t max_elapsed_us,
                               uint32_t iterations = 100, uint32_t timeout_ms = 1000)
{
  TEST_ASSERT(Timebase::IsReady() && iterations > 0);
  TEST_ASSERT(min_elapsed_us > 0 && max_elapsed_us >= min_elapsed_us);
  TEST_ASSERT(timeout_ms > 0 && timeout_ms < UINT32_MAX / 2U);
  TEST_ASSERT(config.data_bits > 0 && config.data_bits <= 8);
  TEST_ASSERT(uart.read_port_ != nullptr && uart.write_port_ != nullptr);
  TEST_ASSERT(uart.read_port_->Readable() && uart.write_port_->Writable());
  TEST_ASSERT(tx.addr_ != nullptr && rx.addr_ != nullptr && tx.size_ > 0 &&
              tx.size_ == rx.size_);
  TEST_ASSERT(tx.size_ <= uart.read_port_->Capacity() &&
              tx.size_ <= uart.write_port_->Capacity());
  const auto tx_begin = reinterpret_cast<uintptr_t>(tx.addr_);
  const auto rx_begin = reinterpret_cast<uintptr_t>(rx.addr_);
  TEST_ASSERT(tx_begin >= rx_begin ? tx_begin - rx_begin >= rx.size_
                                   : rx_begin - tx_begin >= tx.size_);
  TEST_ASSERT(uart.read_port_->Size() == 0);
  WriteOperation write(semaphore, timeout_ms);
  ReadOperation read(semaphore, timeout_ms);
  TEST_ASSERT(uart.SetConfig(config) == ErrorCode::OK);
  auto* sent = static_cast<uint8_t*>(tx.addr_);
  auto* received = static_cast<uint8_t*>(rx.addr_);
  const auto mask = static_cast<uint8_t>((1U << config.data_bits) - 1U);
  uint64_t elapsed_us = 0;
  for (uint32_t round = 0; round < iterations; ++round)
  {
    auto expected = [&](size_t i)
    { return static_cast<uint8_t>(((i % 251U) * 37U ^ round) & mask); };
    for (size_t i = 0; i < tx.size_; ++i)
    {
      sent[i] = expected(i);
      received[i] = static_cast<uint8_t>(~expected(i));
    }
    // 计到完整回读，不把发送入队当作线路完成；准备和检查留在计时区间外。
    // Time through full readback, not TX admission; prepare and check outside the
    // interval.
    const auto start = Timebase::GetMicroseconds();
    TEST_ASSERT(uart.Write({sent, tx.size_}, write) == ErrorCode::OK);
    TEST_ASSERT(uart.Read(rx, read) == ErrorCode::OK);
    elapsed_us += (Timebase::GetMicroseconds() - start).ToMicrosecond();
    TEST_ASSERT(elapsed_us <= max_elapsed_us);
    // 高位不属于有效数据，调用方按配置屏蔽；8 位模式仍比较全部位。
    // Mask unspecified upper bits in the caller; 8-bit mode still checks every bit.
    for (size_t i = 0; i < tx.size_; ++i)
    {
      TEST_ASSERT((received[i] & mask) == expected(i));
    }
    TEST_ASSERT(uart.read_port_->Size() == 0);
  }
  TEST_ASSERT(elapsed_us >= min_elapsed_us);
  return elapsed_us;
}
}  // namespace LibXR::Test
