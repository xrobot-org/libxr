/**
 * @file dap_test_common.hpp
 * @brief DAP v1 与 v2 主机测试共用的 SWD 替身 / SWD double shared by the DAP v1 and v2
 * host tests.
 */

#pragma once

#include <atomic>
#include <cstdint>
#include <cstring>
#include <functional>
#include <thread>
#include <vector>

#include "debug/swd.hpp"

namespace LibXR::USB::Test
{
/// 最小的 SWD 探针：每次传输都成功，读到固定的字；记录每次调用所在的线程
/// Minimal SWD probe: every transfer succeeds and reads a fixed word. It records the
/// thread of every call.
class FakeSwd : public LibXR::Debug::Swd
{
 public:
  unsigned enters = 0;       ///< EnterSwd() 次数 / Calls of EnterSwd().
  unsigned closes = 0;       ///< Close() 次数 / Calls of Close().
  unsigned line_resets = 0;  ///< LineReset() 次数 / Calls of LineReset().
  unsigned sequences = 0;    ///< SeqWriteBits() 次数 / Calls of SeqWriteBits().
  /// 写传输与带数据的写序列的次数 / Write transfers and write sequences with data.
  unsigned writes = 0;
  unsigned reads = 0;     ///< 读传输次数 / Read transfers.
  uint32_t clock_hz = 0;  ///< 最近一次 SetClockHz() 的值 / Last SetClockHz() value.
  /// SetClockHz() 的次数，可以从另一个线程读。Step 线程开始一个会话时最后调用它，所以等到
  /// 这次调用就知道 Step 已经开始执行手里的命令。
  /// Calls of SetClockHz(), readable from another thread. The Step thread ends the start
  /// of a session with one, so a test that waits for it knows that Step has begun the
  /// command it holds.
  std::atomic<unsigned> clock_calls{0};
  std::vector<std::thread::id> callers;  ///< 每次调用的线程 / Thread of every call.
  /// SeqWriteBits() 执行期间调用一次，随后清空；测试用它在命令执行中途插入动作
  /// Called once inside SeqWriteBits() and then cleared; a test uses it to act while a
  /// command is executing.
  std::function<void()> on_sequence;

  ErrorCode SetClockHz(uint32_t hz) override
  {
    Note();
    clock_hz = hz;
    clock_calls.fetch_add(1);
    return ErrorCode::OK;
  }
  void Close() override
  {
    Note();
    ++closes;
  }
  ErrorCode LineReset() override
  {
    Note();
    ++line_resets;
    return ErrorCode::OK;
  }
  ErrorCode EnterSwd() override
  {
    Note();
    ++enters;
    return ErrorCode::OK;
  }
  ErrorCode Transfer(const LibXR::Debug::SwdProtocol::Request& req,
                     LibXR::Debug::SwdProtocol::Response& resp) override
  {
    Note();
    if (req.rnw)
    {
      ++reads;
      resp.rdata = 0x12345678u;
    }
    else
    {
      ++writes;
    }
    resp.ack = LibXR::Debug::SwdProtocol::Ack::OK;
    resp.parity_ok = true;
    return ErrorCode::OK;
  }
  void IdleClocks(uint32_t) override { Note(); }
  ErrorCode SeqWriteBits(uint32_t cycles, const uint8_t* data) override
  {
    Note();
    ++sequences;
    if (on_sequence)
    {
      const std::function<void()> hook = std::move(on_sequence);
      on_sequence = nullptr;
      hook();
    }
    if (cycles != 0U && data != nullptr)
    {
      ++writes;
    }
    return ErrorCode::OK;
  }
  ErrorCode SeqReadBits(uint32_t cycles, uint8_t* out) override
  {
    Note();
    if (out != nullptr)
    {
      std::memset(out, 0, (cycles + 7U) / 8U);
    }
    return ErrorCode::OK;
  }

 private:
  void Note() { callers.push_back(std::this_thread::get_id()); }
};

}  // namespace LibXR::USB::Test
