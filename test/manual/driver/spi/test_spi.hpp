/**
 * @file test_spi.hpp
 * @brief SPI 全双工接线回环测试 / Wired full-duplex SPI loopback test.
 *
 * 在指定长度下反复同时收发，检查三种完成方式、接收内容和发送缓冲区保持不变。
 * Repeat equal-length transfers in three completion modes; check received bytes
 * and that the transmitted bytes remain unchanged. Requires MOSI wired to MISO.
 * 配置测试另检查指定配置下的回环数据和传输耗时。
 * A separate configuration test checks loopback data and transfer time after setup.
 */
#pragma once

#include <initializer_list>

#include "spi.hpp"
#include "test_assert.hpp"
#include "timebase.hpp"
#include "transfer_wait.hpp"

namespace LibXR::Test
{
/**
 * @brief 检查不同长度下的全双工回环 / Check full-duplex loopback at selected lengths.
 * @pre 初始化为 8 位帧全双工主机，MOSI 接 MISO，MISO 不连接其他输出。
 *      Configure an 8-bit full-duplex master; wire MOSI to MISO without other outputs on
 * MISO.
 * @pre 独占总线；tx/rx 是独立工作 RAM，不能与彼此或后端正在使用的缓冲区重叠。
 *      Reserve the bus; tx/rx must be separate work RAM, disjoint from each other and
 * active backend buffers.
 * @param spi 已初始化的 SPI / Initialized SPI controller.
 * @param semaphore 调用方保留的 BLOCK 完成信号量 / Caller-retained semaphore for BLOCK
 * completion.
 * @param tx 发送工作缓冲区，内容会被覆盖 / Transmit work buffer; contents are
 * overwritten.
 * @param rx 接收工作缓冲区，内容会被覆盖 / Receive work buffer; contents are overwritten.
 * @param lengths 每轮测试的字节数列表，须满足后端容量和对齐要求 /
 *        Byte lengths tested each round, within backend capacity/alignment requirements.
 * @param timeout_ms BLOCK 超时及返回后的完成等待上限 / BLOCK timeout and post-return wait
 * limit.
 * @param iterations 重复轮数，每个长度均测试三种完成方式 /
 *        Rounds; every length uses all three completion modes.
 */
inline void TestSPILoopback(SPI& spi, Semaphore& semaphore, RawData tx, RawData rx,
                            std::initializer_list<size_t> lengths,
                            uint32_t timeout_ms = 1000, uint32_t iterations = 1000)
{
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
  }
  auto* sent = static_cast<uint8_t*>(tx.addr_);
  auto* received = static_cast<uint8_t*>(rx.addr_);
  Detail::TransferTestCompletion completion(semaphore, timeout_ms);
  for (uint32_t round = 0; round < iterations; ++round)
  {
    for (size_t length : lengths)
    {
      for (unsigned mode = 0; mode < 3; ++mode)
      {
        const auto seed = static_cast<uint8_t>(round + length + mode * 85U);
        auto expected = [&](size_t i) { return static_cast<uint8_t>((i % 251U) ^ seed); };
        for (size_t i = 0; i < length; ++i)
        {
          sent[i] = expected(i);
          received[i] = static_cast<uint8_t>(~expected(i));
        }
        completion.Run(mode,
                       [&](SPI::OperationRW& operation)
                       {
                         return spi.ReadAndWrite({received, length}, {sent, length},
                                                 operation, false);
                       });
        // 分别对照生成规则，避免收发缓冲区被同时改坏后仍相等。
        // Check both against the pattern, not just each other, to catch shared
        // corruption.
        for (size_t i = 0; i < length; ++i)
        {
          TEST_ASSERT(sent[i] == expected(i));
          TEST_ASSERT(received[i] == expected(i));
        }
        completion.Check();
      }
    }
  }
}
/**
 * @brief 检查配置后的 SPI 回环数据和耗时 / Check SPI loopback data and timing after
 * setup.
 * @pre 沿用 8 位全双工主机接线及独立缓冲区要求，关闭双缓冲。从普通任务调用，
 *      总线空闲，微秒时间基已初始化。
 *      Use the same 8-bit full-duplex master wiring and separate buffers, with double
 *      buffering disabled. Call from a normal task with an idle bus and a ready
 *      microsecond timebase.
 * @param spi 已初始化的 SPI / Initialized SPI controller.
 * @param semaphore 调用方保留的 BLOCK 完成信号量 / Caller-retained semaphore for BLOCK
 * completion.
 * @param config 后端支持的配置 / Configuration supported by the backend.
 * @param tx 发送工作区，其大小为每次传输长度 / Transmit work buffer; its size is the
 * transfer length.
 * @param rx 等长的接收工作区 / Receive work buffer of equal size.
 * @param min_elapsed_us 所有传输耗时之和的下限，单位微秒 / Lower bound on summed transfer
 * time in microseconds.
 * @param max_elapsed_us 总耗时上限，包含调用、调度及传输间隙 / Upper bound including
 * calls, scheduling and transfer gaps.
 * @param iterations 传输次数 / Number of transfers.
 * @param timeout_ms 每次阻塞调用的超时 / Timeout for each blocking call.
 * @return 实测的传输耗时之和，单位微秒 / Measured sum of transfer times in microseconds.
 * @note 正常返回后保留新配置及工作区内容；重复调用可检查同配置重设和 A-B-A 切换。
 *       回环和耗时不能独立确认线路上的时钟极性和相位。
 *       Retains the new configuration and buffer contents. Repeat calls for same-config
 *       setup and A-B-A switching. Loopback and timing do not independently verify
 *       the electrical clock polarity or phase.
 */
inline uint64_t TestSPIConfig(SPI& spi, Semaphore& semaphore, SPI::Configuration config,
                              RawData tx, RawData rx, uint64_t min_elapsed_us,
                              uint64_t max_elapsed_us, uint32_t iterations = 100,
                              uint32_t timeout_ms = 1000)
{
  TEST_ASSERT(Timebase::IsReady() && iterations > 0);
  TEST_ASSERT(min_elapsed_us > 0 && max_elapsed_us >= min_elapsed_us);
  TEST_ASSERT(timeout_ms > 0 && timeout_ms < UINT32_MAX / 2U);
  TEST_ASSERT(!config.double_buffer);
  TEST_ASSERT(tx.addr_ != nullptr && rx.addr_ != nullptr && tx.size_ > 0 &&
              tx.size_ == rx.size_);
  const auto tx_begin = reinterpret_cast<uintptr_t>(tx.addr_);
  const auto rx_begin = reinterpret_cast<uintptr_t>(rx.addr_);
  TEST_ASSERT(tx_begin >= rx_begin ? tx_begin - rx_begin >= rx.size_
                                   : rx_begin - tx_begin >= tx.size_);
  SPI::OperationRW operation(semaphore, timeout_ms);
  TEST_ASSERT(spi.SetConfig(config) == ErrorCode::OK);
  auto* sent = static_cast<uint8_t*>(tx.addr_);
  auto* received = static_cast<uint8_t*>(rx.addr_);
  uint64_t elapsed_us = 0;
  for (uint32_t round = 0; round < iterations; ++round)
  {
    auto expected = [&](size_t i)
    { return static_cast<uint8_t>((i % 251U) * 37U ^ round); };
    for (size_t i = 0; i < tx.size_; ++i)
    {
      sent[i] = expected(i);
      received[i] = static_cast<uint8_t>(~expected(i));
    }
    // BLOCK 返回时传输已完成；准备和检查放在计时区间外。
    // BLOCK returns after completion; prepare and check outside the measured interval.
    const auto start = Timebase::GetMicroseconds();
    TEST_ASSERT(spi.ReadAndWrite(rx, {sent, tx.size_}, operation) == ErrorCode::OK);
    elapsed_us += (Timebase::GetMicroseconds() - start).ToMicrosecond();
    TEST_ASSERT(elapsed_us <= max_elapsed_us);
    for (size_t i = 0; i < tx.size_; ++i)
    {
      TEST_ASSERT(sent[i] == expected(i));
      TEST_ASSERT(received[i] == expected(i));
    }
  }
  TEST_ASSERT(elapsed_us >= min_elapsed_us);
  return elapsed_us;
}
}  // namespace LibXR::Test
