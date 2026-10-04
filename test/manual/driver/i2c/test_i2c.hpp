/**
 * @file test_i2c.hpp
 * @brief I2C 寄存器读写测试 / I2C register read and write tests.
 *
 * 用阻塞、轮询和回调方式检查固定寄存器，以及两组调用方数据的写入回读。
 * Check fixed registers and write/readback of two caller-supplied patterns in
 * blocking, polling and callback modes. Device-specific values belong to the caller.
 * 配置测试另检查固定寄存器读数及传输耗时。
 * A separate configuration test checks fixed-register data and transfer time.
 */
#pragma once

#include <cstring>

#include "i2c.hpp"
#include "test_assert.hpp"
#include "thread.hpp"
#include "timebase.hpp"
#include "transfer_wait.hpp"

namespace LibXR::Test
{
namespace Detail
{
inline void CheckI2CRegister(uint16_t slave_addr, uint16_t mem_addr,
                             I2C::MemAddrLength addr_length)
{
  TEST_ASSERT(slave_addr <= 0x3FFU);
  TEST_ASSERT(addr_length == I2C::MemAddrLength::BYTE_8 ||
              addr_length == I2C::MemAddrLength::BYTE_16);
  TEST_ASSERT(addr_length != I2C::MemAddrLength::BYTE_8 || mem_addr <= 0xFFU);
}

inline void CheckI2CBuffers(ConstRawData expected, RawData buffer)
{
  TEST_ASSERT(expected.addr_ != nullptr && buffer.addr_ != nullptr);
  TEST_ASSERT(expected.size_ > 0 && expected.size_ <= buffer.size_);
  const auto source = reinterpret_cast<uintptr_t>(expected.addr_);
  const auto destination = reinterpret_cast<uintptr_t>(buffer.addr_);
  TEST_ASSERT(source >= destination ? source - destination >= expected.size_
                                    : destination - source >= expected.size_);
}

inline void PoisonI2CBuffer(ConstRawData expected, RawData buffer)
{
  const auto* wanted = static_cast<const uint8_t*>(expected.addr_);
  auto* received = static_cast<uint8_t*>(buffer.addr_);
  for (size_t i = 0; i < expected.size_; ++i)
  {
    received[i] = static_cast<uint8_t>(~wanted[i]);
  }
}

inline void CheckI2CData(ConstRawData expected, RawData buffer)
{
  const auto* wanted = static_cast<const uint8_t*>(expected.addr_);
  const auto* received = static_cast<const uint8_t*>(buffer.addr_);
  for (size_t i = 0; i < expected.size_; ++i)
  {
    TEST_ASSERT(received[i] == wanted[i]);
  }
}

}  // namespace Detail

/**
 * @brief 检查三种完成方式下的固定寄存器读数 / Check fixed reads in three completion
 * modes.
 * @pre 总线和设备已初始化并由测试独占，寄存器允许重复读取且内容固定。
 *      Initialize and reserve the bus/device; registers must support stable repeated
 * reads.
 * @param i2c 已初始化的 I2C / Initialized I2C controller.
 * @param slave_addr 不带 R/W 位的从机地址 / Slave address without the R/W bit.
 * @param mem_addr 起始寄存器地址 / Starting register address.
 * @param addr_length 寄存器地址长度 / Register address width.
 * @param expected 预期字节，决定读取长度，不能与接收区重叠 / Expected bytes defining the
 *        read length; must not overlap the receive area.
 * @param buffer 接收区，容量至少为 expected 大小 / Receive storage, at least expected
 * size.
 * @param timeout_ms BLOCK 超时及返回后的完成等待上限 / BLOCK timeout and post-return wait
 * limit.
 * @param iterations 轮数，每轮三次读取 / Rounds, with three reads per round.
 */
inline void TestI2CMemRead(I2C& i2c, uint16_t slave_addr, uint16_t mem_addr,
                           I2C::MemAddrLength addr_length, ConstRawData expected,
                           RawData buffer, uint32_t timeout_ms = 1000,
                           uint32_t iterations = 1000)
{
  Detail::CheckI2CRegister(slave_addr, mem_addr, addr_length);
  Detail::CheckI2CBuffers(expected, buffer);
  TEST_ASSERT(iterations > 0);
  Detail::TransferTestCompletion completion(timeout_ms);
  const RawData read_buffer{buffer.addr_, expected.size_};
  for (uint32_t round = 0; round < iterations; ++round)
  {
    for (unsigned mode = 0; mode < 3; ++mode)
    {
      Detail::PoisonI2CBuffer(expected, read_buffer);
      completion.Run(mode,
                     [&](ReadOperation& operation)
                     {
                       return i2c.MemRead(slave_addr, mem_addr, read_buffer, operation,
                                          addr_length, false);
                     });
      Detail::CheckI2CData(expected, read_buffer);
      completion.Check();
    }
  }
}

/**
 * @brief 交替写入两组合法数据并回读 / Alternate two legal patterns and check readback.
 * @pre 目标区域须专供测试，支持原样回读；两组数据均合法且长度相同。
 *      Reserve a region that reads back written bytes; both patterns must be legal and
 * equal-sized.
 * @pre 调用方负责分页、写入寿命、保存与恢复原值；成功后保留第二组数据。
 *      The caller handles paging, endurance and saving/restoring old values. Leaves the
 * second pattern.
 * @param i2c 已初始化并独占的总线 / Initialized, exclusively reserved bus.
 * @param slave_addr 不带 R/W 位的从机地址 / Slave address without the R/W bit.
 * @param mem_addr 可读写区域起始地址 / Start of the read/write region.
 * @param addr_length 寄存器地址长度 / Register address width.
 * @param first 第一组合法写入数据 / First legal write pattern.
 * @param second 不同的第二组数据 / Distinct second pattern.
 * @param buffer 独立回读缓冲区，不能与任一写入数据重叠 / Separate receive buffer,
 * disjoint from both patterns.
 * @param settle_ms 写完成到回读之间的等待时间 / Delay between write completion and
 * readback.
 * @param timeout_ms BLOCK 超时及返回后的完成等待上限 / BLOCK timeout and post-return wait
 * limit.
 * @param iterations 轮数，每轮各六次写入和读取 / Rounds, each with six writes and six
 * reads.
 */
inline void TestI2CMemWriteRead(I2C& i2c, uint16_t slave_addr, uint16_t mem_addr,
                                I2C::MemAddrLength addr_length, ConstRawData first,
                                ConstRawData second, RawData buffer,
                                uint32_t settle_ms = 0, uint32_t timeout_ms = 1000,
                                uint32_t iterations = 100)
{
  Detail::CheckI2CRegister(slave_addr, mem_addr, addr_length);
  Detail::CheckI2CBuffers(first, buffer);
  Detail::CheckI2CBuffers(second, buffer);
  TEST_ASSERT(first.size_ == second.size_);
  TEST_ASSERT(std::memcmp(first.addr_, second.addr_, first.size_) != 0);
  TEST_ASSERT(settle_ms < UINT32_MAX / 2U && iterations > 0 &&
              iterations <= UINT32_MAX / 4U);
  Detail::TransferTestCompletion completion(timeout_ms);
  const ConstRawData patterns[] = {first, second};
  const RawData read_buffer{buffer.addr_, first.size_};
  for (uint32_t round = 0; round < iterations; ++round)
  {
    for (unsigned mode = 0; mode < 3; ++mode)
    {
      for (const auto& pattern : patterns)
      {
        completion.Run(mode,
                       [&](WriteOperation& operation)
                       {
                         return i2c.MemWrite(slave_addr, mem_addr, pattern, operation,
                                             addr_length, false);
                       });
        Thread::Sleep(settle_ms);
        Detail::PoisonI2CBuffer(pattern, read_buffer);
        completion.Run(mode,
                       [&](ReadOperation& operation)
                       {
                         return i2c.MemRead(slave_addr, mem_addr, read_buffer, operation,
                                            addr_length, false);
                       });
        Detail::CheckI2CData(pattern, read_buffer);
        completion.Check();
      }
    }
  }
}
/**
 * @brief 检查配置后的寄存器读数和耗时 / Check register data and timing after
 * configuration.
 * @pre 沿用固定寄存器读取的设备、缓冲区和地址要求。从普通任务调用，总线空闲，
 *      微秒时间基已初始化，后端支持所选配置。
 *      Use the same device, buffer and address requirements as the fixed-read test.
 *      Call from a normal task with an idle bus, a ready microsecond timebase and a
 *      configuration supported by the backend.
 * @param i2c 已初始化并独占的总线 / Initialized, exclusively reserved bus.
 * @param config 后端和从机均支持的时钟配置 / Clock configuration supported by backend and
 * slave.
 * @param slave_addr 不带 R/W 位的从机地址 / Slave address without the R/W bit.
 * @param mem_addr 起始寄存器地址 / Starting register address.
 * @param addr_length 寄存器地址长度 / Register address width.
 * @param expected 固定预期字节，决定单次读取长度 / Fixed expected bytes defining each
 * read length.
 * @param buffer 独立接收区，容量至少为 expected 大小 / Separate receive storage, at least
 * expected size.
 * @param min_elapsed_us 所有读取耗时之和的下限，单位微秒 / Lower bound on summed read
 * time in microseconds.
 * @param max_elapsed_us 总耗时上限，包含协议、调用及调度开销 / Upper bound including
 * protocol, call and scheduling overhead.
 * @param iterations 读取次数 / Number of reads.
 * @param timeout_ms 传给每次 BLOCK 操作的超时参数 / Timeout passed to each BLOCK
 * operation.
 * @return 实测读取耗时之和，单位微秒 / Measured sum of read times in microseconds.
 * @note 正常返回后保留新配置；重复调用可检查同配置重设和 A-B-A 切换。
 *       时钟拉伸和软件间隙也计入测量时间。
 *       Retains the new configuration. Repeat calls for same-config setup and A-B-A
 *       switching. Clock stretching and software gaps are part of the measured time.
 */
inline uint64_t TestI2CConfig(I2C& i2c, I2C::Configuration config, uint16_t slave_addr,
                              uint16_t mem_addr, I2C::MemAddrLength addr_length,
                              ConstRawData expected, RawData buffer,
                              uint64_t min_elapsed_us, uint64_t max_elapsed_us,
                              uint32_t iterations = 100, uint32_t timeout_ms = 1000)
{
  Detail::CheckI2CRegister(slave_addr, mem_addr, addr_length);
  Detail::CheckI2CBuffers(expected, buffer);
  TEST_ASSERT(Timebase::IsReady() && iterations > 0);
  TEST_ASSERT(min_elapsed_us > 0 && max_elapsed_us >= min_elapsed_us);
  TEST_ASSERT(timeout_ms > 0 && timeout_ms < UINT32_MAX / 2U);
  Semaphore semaphore;
  ReadOperation operation(semaphore, timeout_ms);
  const RawData read_buffer{buffer.addr_, expected.size_};
  TEST_ASSERT(i2c.SetConfig(config) == ErrorCode::OK);
  uint64_t elapsed_us = 0;
  for (uint32_t round = 0; round < iterations; ++round)
  {
    Detail::PoisonI2CBuffer(expected, read_buffer);
    // 地址阶段、数据传输和完成等待都计时；预填和核对放在区间外。
    // Include addressing, data transfer and completion wait; prepare and check outside.
    const auto start = Timebase::GetMicroseconds();
    TEST_ASSERT(i2c.MemRead(slave_addr, mem_addr, read_buffer, operation, addr_length,
                            false) == ErrorCode::OK);
    elapsed_us += (Timebase::GetMicroseconds() - start).ToMicrosecond();
    TEST_ASSERT(elapsed_us <= max_elapsed_us);
    Detail::CheckI2CData(expected, read_buffer);
  }
  TEST_ASSERT(elapsed_us >= min_elapsed_us);
  return elapsed_us;
}
}  // namespace LibXR::Test
