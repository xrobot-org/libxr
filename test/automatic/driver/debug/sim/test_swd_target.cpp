/**
 * @file test_swd_target.cpp
 * @brief SWD 目标仿真的单元测试 / Unit tests for the simulated SWD target.
 *
 * 同一套 ADIv5 操作分别经 `SimSwd`（接口级、按位生成波形）和 `SwdGeneralGPIO` +
 * 仿真引脚（真实位操作引擎）跑一遍：选择 SWD、读 DPIDR、上电、MEM-AP 读写（字、半字、
 * 字节通道，地址自增与 4 KiB 回绕）、粘性错误与清除、WAIT 重试、溢出检测的数据阶段、
 * 2..4 个转向周期、写数据校验错、线复位后必须先读 DPIDR、掉线、ROM 表、停核与内核
 * 寄存器、系统复位。
 * The same ADIv5 operations run through `SimSwd` (interface level, waveform generated
 * bit by bit) and through `SwdGeneralGPIO` with simulated pins (the real bit-banging
 * engine): selecting SWD, reading DPIDR, power-up, MEM-AP access (word, halfword and
 * byte lanes, auto-increment and the 4 KiB wrap), sticky errors and clearing, WAIT
 * retries, the overrun-detection data phase, 2..4 turnaround cycles, write-data
 * parity errors, the DPIDR read required after a line reset, a vanished target, the
 * ROM table, halting and core registers, and system reset.
 *
 * `Swd::TransferPolicy` 的转向周期数和数据阶段（DAP_SWD_Configure 设置）两条路径都要
 * 生效：`SimSwd` 读策略生成波形，`SwdGeneralGPIO` 的无延时和带延时路径按策略出时钟。
 * The turnaround cycles and the data phase of `Swd::TransferPolicy` (set by
 * DAP_SWD_Configure) must take effect on both connections: `SimSwd` reads the policy to
 * generate the waveform, and the no-delay and delayed paths of `SwdGeneralGPIO` clock
 * according to it.
 */

#include <cstdio>
#include <functional>

#include "debug/swd_general_gpio.hpp"
#include "swd_target.hpp"
#include "test_assert.hpp"

using namespace LibXR;
using namespace LibXR::Debug;
using namespace LibXR::Debug::Sim;
using SwdProtocol::Ack;

namespace
{
/// 主机侧的 ADIv5 操作 / ADIv5 operations on the host side
struct Adi
{
  Swd& swd;

  Ack Dp(bool rnw, uint8_t a32, uint32_t& data)
  {
    SwdProtocol::Response r;
    TEST_ASSERT(swd.Transfer({SwdProtocol::Port::DP, rnw, a32, data}, r) ==
                ErrorCode::OK);
    if (rnw) data = r.rdata;
    if (r.ack == Ack::OK && rnw) TEST_ASSERT(r.parity_ok);
    return r.ack;
  }
  Ack Ap(bool rnw, uint8_t a32, uint32_t& data)
  {
    SwdProtocol::Response r;
    TEST_ASSERT(swd.Transfer({SwdProtocol::Port::AP, rnw, a32, data}, r) ==
                ErrorCode::OK);
    if (rnw) data = r.rdata;
    return r.ack;
  }
  uint32_t DpRead(uint8_t a32)
  {
    uint32_t v = 0;
    TEST_ASSERT(Dp(true, a32, v) == Ack::OK);
    return v;
  }
  void DpWrite(uint8_t a32, uint32_t v) { TEST_ASSERT(Dp(false, a32, v) == Ack::OK); }
  void ApWrite(uint8_t reg, uint32_t v)
  {
    DpWrite(2, reg & 0xF0u);  // SELECT: APSEL 0, APBANKSEL
    TEST_ASSERT(Ap(false, (reg >> 2) & 3u, v) == Ack::OK);
  }
  uint32_t ApRead(uint8_t reg)
  {
    DpWrite(2, reg & 0xF0u);
    uint32_t v = 0;
    TEST_ASSERT(Ap(true, (reg >> 2) & 3u, v) == Ack::OK);  // posted
    return DpRead(3);                                      // RDBUFF
  }

  void Connect()
  {
    TEST_ASSERT(swd.EnterSwd() == ErrorCode::OK);
    TEST_ASSERT(DpRead(0) == DebugPort::DPIDR);
    DpWrite(0, 0x1E);                                       // ABORT: clear everything
    DpWrite(1, 0x50000000u);                                // CDBGPWRUPREQ | CSYSPWRUPREQ
    TEST_ASSERT((DpRead(1) & 0xA0000000u) == 0xA0000000u);  // both ACKs
  }

  void Write32(uint32_t addr, uint32_t v)
  {
    ApWrite(0x00, 0x23000012u);  // CSW: word, single increment
    ApWrite(0x04, addr);
    ApWrite(0x0C, v);
  }
  uint32_t Read32(uint32_t addr)
  {
    ApWrite(0x00, 0x23000012u);
    ApWrite(0x04, addr);
    return ApRead(0x0C);
  }
};

/// 只在接口级的 SimSwd 上跑 / Run on the interface-level SimSwd only
void ForSimSwd(const std::function<void(Target&, Swd&)>& body)
{
  Target t;
  SimSwd swd(t.wire);
  body(t, swd);
  TEST_ASSERT(t.wire.contention == 0);
}

/// 只在真实位操作引擎上跑：SwdGeneralGPIO 分无延时（cpu_hz 0）和带延时（16 MHz 的 CPU 在
/// 500 kHz 的 SWCLK 下每个半周期要循环若干次）两条传输路径
/// Run on the real bit-banging engine only: SwdGeneralGPIO has a no-delay path (cpu_hz 0)
/// and a delayed one (a 16 MHz CPU at 500 kHz SWCLK loops several times per half period)
void ForEngine(const std::function<void(Target&, Swd&)>& body)
{
  for (const uint32_t cpu_hz : {0u, 16'000'000u})
  {
    Target t;
    SimSwclkPin swclk(t.wire);
    SimSwdioPin swdio(t.wire);
    SwdGeneralGPIO<SimSwclkPin, SimSwdioPin> swd(swclk, swdio, cpu_hz, SwdGpioTiming{});
    body(t, swd);
    TEST_ASSERT(t.wire.contention == 0);
  }
}

/// 每种接法各跑一遍 / Run once per connection
void ForEachConnection(const std::function<void(Target&, Swd&)>& body)
{
  ForSimSwd(body);
  ForEngine(body);
}

// ---------------------------------------------------------------------------

/// SWJ-DP 上电是 JTAG：切换序列之前什么都不应答 / The SWJ-DP powers up in JTAG mode:
/// nothing answers before the switch sequence
void TestSwitchToSwd()
{
  ForEachConnection(
      [](Target& t, Swd& swd)
      {
        Adi adi{swd};
        TEST_ASSERT(swd.LineReset() == ErrorCode::OK);
        uint32_t v = 0;
        TEST_ASSERT(adi.Dp(true, 0, v) != Ack::OK);
        TEST_ASSERT(!t.wire.SwdSelected());
        adi.Connect();
        TEST_ASSERT(t.wire.SwdSelected());
        TEST_ASSERT(t.wire.protocol_errors == 0);
        // 已经在 SWD 时再发一次切换序列也能连上 / Sending the sequence again while in
        // SWD still connects.
        adi.Connect();
        TEST_ASSERT(t.wire.protocol_errors == 0);
      });
}

/// 线复位后第一条必须读 DPIDR，否则不应答 / After a line reset the first access must
/// read DPIDR, anything else gets no response
void TestDpidrFirstAfterLineReset()
{
  ForEachConnection(
      [](Target&, Swd& swd)
      {
        Adi adi{swd};
        adi.Connect();
        TEST_ASSERT(swd.LineReset() == ErrorCode::OK);
        swd.IdleClocks(2);
        uint32_t v = 0;
        TEST_ASSERT(adi.Dp(true, 1, v) == Ack::PROTOCOL);  // CTRL/STAT first: silence
        TEST_ASSERT(adi.DpRead(0) == DebugPort::DPIDR);
        TEST_ASSERT(adi.Dp(true, 1, v) == Ack::OK);
      });
}

/// MEM-AP：字读写、地址自增、4 KiB 回绕，空闲周期夹在传输之间 / MEM-AP: word access,
/// auto-increment, the 4 KiB wrap, idle cycles between transfers
void TestMemApWords()
{
  ForEachConnection(
      [](Target& t, Swd& swd)
      {
        Adi adi{swd};
        adi.Connect();
        adi.Write32(0x20000000u, 0x12345678u);
        swd.IdleClocks(4);
        TEST_ASSERT(adi.Read32(0x20000000u) == 0x12345678u);
        // 自增：连续写四个字 / auto-increment: four words in a row
        adi.ApWrite(0x00, 0x23000012u);
        adi.ApWrite(0x04, 0x20000FF8u);
        for (uint32_t i = 0; i < 4; ++i)
        {
          uint32_t v = 0xA0000000u + i;
          TEST_ASSERT(adi.Ap(false, 3, v) == Ack::OK);
          swd.IdleClocks(1);
        }
        // TAR 在 4 KiB 内回绕：第三个字写到 0x20000000 / TAR wraps inside 4 KiB: the
        // third word lands at 0x20000000
        TEST_ASSERT(t.dp.Tar() == 0x20000008u);
        TEST_ASSERT(adi.Read32(0x20000FF8u) == 0xA0000000u);
        TEST_ASSERT(adi.Read32(0x20000FFCu) == 0xA0000001u);
        TEST_ASSERT(adi.Read32(0x20000000u) == 0xA0000002u);
        // 延迟读：连续读，第一次返回旧值，RDBUFF 给最后一个 / posted reads
        adi.ApWrite(0x04, 0x20000FF8u);
        uint32_t first = 0, second = 0;
        TEST_ASSERT(adi.Ap(true, 3, first) == Ack::OK);
        TEST_ASSERT(adi.Ap(true, 3, second) == Ack::OK);
        TEST_ASSERT(second == 0xA0000000u);
        TEST_ASSERT(adi.DpRead(3) == 0xA0000001u);
      });
}

/// 半字与字节在对应的字节通道上 / Halfwords and bytes sit on their byte lanes
void TestByteLanes()
{
  ForEachConnection(
      [](Target&, Swd& swd)
      {
        Adi adi{swd};
        adi.Connect();
        adi.Write32(0x20000010u, 0);
        adi.ApWrite(0x00, 0x23000010u);  // byte, single increment
        adi.ApWrite(0x04, 0x20000011u);
        adi.ApWrite(0x0C, 0x0000AB00u);  // lane 1
        adi.ApWrite(0x00, 0x23000011u);  // halfword
        adi.ApWrite(0x04, 0x20000012u);
        adi.ApWrite(0x0C, 0xCDEF0000u);  // lanes 2-3
        TEST_ASSERT(adi.Read32(0x20000010u) == 0xCDEFAB00u);
        adi.ApWrite(0x00, 0x23000010u);
        adi.ApWrite(0x04, 0x20000013u);
        TEST_ASSERT(adi.ApRead(0x0C) == 0xCD000000u);
      });
}

/// 总线错误置 STICKYERR，之后 AP 访问回 FAULT，ABORT 清除后恢复 / A bus error sets
/// STICKYERR, AP accesses then get FAULT, and ABORT clears it
void TestStickyErrorAndAbort()
{
  ForEachConnection(
      [](Target&, Swd& swd)
      {
        Adi adi{swd};
        adi.Connect();
        adi.ApWrite(0x00, 0x23000012u);
        adi.ApWrite(0x04, 0x40000000u);  // 未映射 / unmapped
        uint32_t v = 0;
        TEST_ASSERT(adi.Ap(true, 3, v) == Ack::OK);  // 延迟读本身应答 OK / posted: OK
        TEST_ASSERT(adi.Dp(true, 3, v) == Ack::FAULT);
        TEST_ASSERT((adi.DpRead(1) & DebugPort::STICKYERR) != 0u);
        TEST_ASSERT(adi.Dp(false, 2, v) == Ack::FAULT);  // SELECT 写也被拒 / rejected
        adi.DpWrite(0, SwdProtocol::DP_ABORT_STKERRCLR);
        TEST_ASSERT((adi.DpRead(1) & DebugPort::STICKYERR) == 0u);
        TEST_ASSERT(adi.Read32(0x20000000u) == 0u);
        // 闪存可读不可写 / flash reads but does not take writes
        adi.ApWrite(0x04, 0x08000000u);
        v = 1;
        TEST_ASSERT(adi.Ap(false, 3, v) == Ack::OK);
        TEST_ASSERT((adi.DpRead(1) & DebugPort::STICKYERR) != 0u);
        adi.DpWrite(0, SwdProtocol::DP_ABORT_STKERRCLR);
      });
}

/// 注入的 WAIT：主机重试后数据照常 / Injected WAITs: data intact after the retries
void TestWaitRetry()
{
  ForEachConnection(
      [](Target& t, Swd& swd)
      {
        Adi adi{swd};
        adi.Connect();
        adi.Write32(0x20000020u, 0xCAFEF00Du);
        adi.ApWrite(0x04, 0x20000020u);
        t.dp.inject_wait = 3;
        uint32_t v = 0;
        int waits = 0;
        Ack ack;
        while ((ack = adi.Ap(true, 3, v)) == Ack::WAIT) ++waits;
        TEST_ASSERT(ack == Ack::OK && waits == 3);
        TEST_ASSERT(adi.DpRead(3) == 0xCAFEF00Du);
      });
}

/// WAIT 重试次数：wait_retry 为 N 时第一次尝试之后最多再重试 N 次（CMSIS-DAP 参考实现的
/// do { 传输 } while (WAIT && retry--)），N 为 0 时第一个 WAIT 就报告。
/// TransferWithRetry 和 TransferRetryAfterWait（调用方自己做第一次尝试）都按此计数。
/// WAIT retries: with wait_retry N at most N retries follow the first attempt (the
/// CMSIS-DAP reference does { transfer } while (WAIT && retry--)), and with 0 the first
/// WAIT is reported. TransferWithRetry and TransferRetryAfterWait (the caller makes the
/// first attempt itself) both count that way.
void TestWaitRetryCount()
{
  ForEachConnection(
      [](Target& t, Swd& swd)
      {
        Adi adi{swd};
        adi.Connect();
        adi.ApWrite(0x04, 0x20000020u);
        const auto read_drw = SwdProtocol::make_ap_read_req(3);
        struct Case
        {
          uint16_t retry;
          unsigned waits;
        };
        // Retries N and WAITs injected: the transfer succeeds when the WAITs run out
        // within the N + 1 attempts; otherwise the target keeps the unused WAITs.
        for (const Case c : {Case{0, 1}, Case{0, 2}, Case{2, 2}, Case{2, 3}, Case{2, 4}})
        {
          const bool absorbed = c.waits <= c.retry;
          const unsigned left = absorbed ? 0u : c.waits - (c.retry + 1u);
          for (const bool caller_first : {false, true})
          {
            Swd::TransferPolicy policy;
            policy.wait_retry = c.retry;
            swd.SetTransferPolicy(policy);
            t.dp.inject_wait = c.waits;
            SwdProtocol::Response r;
            if (caller_first)
            {
              TEST_ASSERT(swd.Transfer(read_drw, r) == ErrorCode::OK);
              if (r.ack == Ack::WAIT)
              {
                TEST_ASSERT(swd.TransferRetryAfterWait(read_drw, r) == ErrorCode::OK);
              }
            }
            else
            {
              TEST_ASSERT(swd.TransferWithRetry(read_drw, r) == ErrorCode::OK);
            }
            TEST_ASSERT(r.ack == (absorbed ? Ack::OK : Ack::WAIT));
            TEST_ASSERT(t.dp.inject_wait == left);
            t.dp.inject_wait = 0;
          }
        }
      });
}

/// 溢出检测：打开 ORUNDETECT 后 WAIT 之后有数据阶段，主机走完它就能继续；不走则错位。
/// Overrun detection: with ORUNDETECT a data phase follows a WAIT; a host that clocks it
/// carries on, one that skips it is misaligned.
void OverrunDetectCase(bool data_phase, Target& t, Swd& swd);

void TestOverrunDetectDataPhase()
{
  // 没有数据阶段（默认）和有数据阶段，每种接法各一遍
  // Without the data phase (the default) and with it, once per connection
  ForEachConnection([](Target& t, Swd& swd) { OverrunDetectCase(false, t, swd); });
  ForEachConnection([](Target& t, Swd& swd) { OverrunDetectCase(true, t, swd); });
}

void OverrunDetectCase(bool data_phase, Target& t, Swd& swd)
{
  Adi adi{swd};
  adi.Connect();
  adi.Write32(0x20000040u, 0x5A5A5A5Au);
  adi.DpWrite(1, 0x50000001u);  // ORUNDETECT
  if (data_phase) SetWireConfig(swd, 1, true);
  adi.ApWrite(0x04, 0x20000040u);
  t.dp.inject_wait = 1;
  uint32_t v = 0;
  TEST_ASSERT(adi.Ap(true, 3, v) == Ack::WAIT);
  if (!data_phase)
  {
    // 主机跳过数据阶段：下一条请求的头被当成数据吞掉，得不到应答
    // The host skipped the data phase: the next header is swallowed as data
    // and gets no response.
    SwdProtocol::Response r;
    TEST_ASSERT(swd.Transfer({SwdProtocol::Port::DP, true, 1, 0}, r) == ErrorCode::OK);
    TEST_ASSERT(r.ack != Ack::OK);
    return;
  }
  // WAIT 置 STICKYORUN，清掉后重试 / the WAIT set STICKYORUN; clear it and retry
  TEST_ASSERT((adi.DpRead(1) & DebugPort::STICKYORUN) != 0u);
  adi.DpWrite(0, SwdProtocol::DP_ABORT_ORUNERRCLR);
  TEST_ASSERT(adi.Ap(true, 3, v) == Ack::OK);
  TEST_ASSERT(adi.DpRead(3) == 0x5A5A5A5Au);
  // 写也一样：WAIT 后的空写不被目标采用 / The same for a write: the dummy
  // write after a WAIT is not taken by the target.
  t.dp.inject_wait = 1;
  v = 0x12345678u;
  TEST_ASSERT(adi.Ap(false, 3, v) == Ack::WAIT);
  adi.DpWrite(0, SwdProtocol::DP_ABORT_ORUNERRCLR);
  TEST_ASSERT(adi.Ap(false, 3, v) == Ack::OK);
  // 前一次读让 TAR 自增到下一个字 / The read before advanced TAR by one word.
  TEST_ASSERT(adi.Read32(0x20000044u) == 0x12345678u);
  TEST_ASSERT(t.wire.protocol_errors == 0);
}

void TurnaroundCase(uint8_t turn, Target& t, Swd& swd);

/// 转向周期：目标 WCR.TURNROUND 和主机配置一致时 2..4 个周期都能正常访问，数据阶段
/// 同样按转向周期走；不一致时错位。
/// Turnaround: with the target's WCR.TURNROUND and the host configuration in step,
/// 2..4 cycles all work, and so does the data phase; out of step, the link misaligns.
void TestTurnaround()
{
  for (const uint8_t turn : {2, 3, 4})
    ForEachConnection([&](Target& t, Swd& swd) { TurnaroundCase(turn, t, swd); });
}

void TurnaroundCase(uint8_t turn, Target& t, Swd& swd)
{
  Adi adi{swd};
  adi.Connect();
  adi.DpWrite(2, 1u);  // SELECT.DPBANKSEL = 1：WCR
  adi.DpWrite(1, static_cast<uint32_t>(turn - 1u) << 8);
  SetWireConfig(swd, turn, true);
  TEST_ASSERT(adi.DpRead(1) == ((static_cast<uint32_t>(turn - 1u) << 8) | 0x40u));
  adi.DpWrite(2, 0u);
  adi.Write32(0x20000050u, 0xA5A5C3C3u);
  TEST_ASSERT(adi.Read32(0x20000050u) == 0xA5A5C3C3u);

  adi.DpWrite(1, 0x50000001u);  // ORUNDETECT
  t.dp.inject_wait = 1;
  uint32_t v = 0;
  TEST_ASSERT(adi.Ap(true, 3, v) == Ack::WAIT);
  adi.DpWrite(0, SwdProtocol::DP_ABORT_ORUNERRCLR);
  t.dp.inject_wait = 1;
  v = 0x0F0F0F0Fu;
  TEST_ASSERT(adi.Ap(false, 3, v) == Ack::WAIT);
  adi.DpWrite(0, SwdProtocol::DP_ABORT_ORUNERRCLR);
  TEST_ASSERT(adi.Read32(0x20000050u) == 0xA5A5C3C3u);
  TEST_ASSERT(t.wire.protocol_errors == 0);

  // 主机回到 1 个周期，目标仍是 turn 个：读不出 DPIDR
  // The host back at one cycle while the target keeps turn: DPIDR is not read.
  SetWireConfig(swd, 1, true);
  uint32_t id = 0;
  const Ack ack = adi.Dp(true, 0, id);
  TEST_ASSERT(ack != Ack::OK || id != DebugPort::DPIDR);
}

/// 写数据校验错置 WDATAERR，写被丢弃 / A write-data parity error sets WDATAERR and the
/// write is dropped
void TestWriteParityError()
{
  Target t;
  SimSwd swd(t.wire);
  Adi adi{swd};
  adi.Connect();
  adi.Write32(0x20000030u, 0x11111111u);
  adi.ApWrite(0x04, 0x20000030u);
  swd.corrupt_write_parity = true;
  uint32_t v = 0x22222222u;
  TEST_ASSERT(adi.Ap(false, 3, v) == Ack::OK);
  swd.corrupt_write_parity = false;
  TEST_ASSERT((adi.DpRead(1) & DebugPort::WDATAERR) != 0u);
  TEST_ASSERT(adi.Ap(true, 3, v) == Ack::FAULT);
  adi.DpWrite(0, SwdProtocol::DP_ABORT_WDERRCLR);
  TEST_ASSERT(adi.Read32(0x20000030u) == 0x11111111u);
}

/// 读数据校验错和掉线 / Bad read parity and a vanished target
void TestBadReadParityAndDisconnect()
{
  ForEachConnection(
      [](Target& t, Swd& swd)
      {
        Adi adi{swd};
        adi.Connect();
        t.dp.inject_bad_parity = 1;
        SwdProtocol::Response r;
        TEST_ASSERT(swd.Transfer({SwdProtocol::Port::DP, true, 0, 0}, r) ==
                    ErrorCode::OK);
        TEST_ASSERT(r.ack == Ack::OK && !r.parity_ok && r.rdata == DebugPort::DPIDR);
        t.wire.disconnected = true;
        TEST_ASSERT(swd.Transfer({SwdProtocol::Port::DP, true, 0, 0}, r) ==
                    ErrorCode::OK);
        TEST_ASSERT(r.ack == Ack::PROTOCOL);  // 线上拉，读到 0b111 / pulled up: 0b111
        t.wire.disconnected = false;
        adi.Connect();
      });
}

/// ROM 表能找到 SCS，组件 ID 正确 / The ROM table leads to the SCS with valid IDs
void TestRomTable()
{
  Target t;
  SimSwd swd(t.wire);
  Adi adi{swd};
  adi.Connect();
  TEST_ASSERT(adi.ApRead(0xFC) == DebugPort::AP_IDR);
  const uint32_t base = adi.ApRead(0xF8) & ~0xFFFu;
  TEST_ASSERT(base == TargetMemory::ROM_TABLE);
  const uint32_t entry = adi.Read32(base);
  TEST_ASSERT((entry & 3u) == 3u);
  const uint32_t scs = base + (entry & ~0xFFFu);
  TEST_ASSERT(scs == TargetMemory::SCS_BASE);
  TEST_ASSERT(adi.Read32(scs + 0xFF0u) == 0x0Du && adi.Read32(scs + 0xFF4u) == 0xE0u &&
              adi.Read32(scs + 0xFFCu) == 0xB1u);
  TEST_ASSERT(adi.Read32(base + 0xFF4u) == 0x10u);  // ROM 表类 / ROM table class
}

/// 停核、读写内核寄存器、复位向量捕获 / Halt, core register access, reset vector catch
void TestCoreDebug()
{
  Target t;
  SimSwd swd(t.wire);
  Adi adi{swd};
  adi.Connect();
  TEST_ASSERT(adi.Read32(TargetMemory::CPUID) == 0x410FC241u);
  adi.Write32(TargetMemory::DHCSR, 0xA05F0003u);  // DEBUGEN | HALT
  const uint32_t dhcsr = adi.Read32(TargetMemory::DHCSR);
  TEST_ASSERT((dhcsr & (1u << 17)) != 0u && (dhcsr & (1u << 16)) != 0u);
  // R3 = 0x55AA55AA，再读回 / write R3, read it back
  adi.Write32(TargetMemory::DCRDR, 0x55AA55AAu);
  adi.Write32(TargetMemory::DCRSR, (1u << 16) | 3u);
  adi.Write32(TargetMemory::DCRDR, 0);
  adi.Write32(TargetMemory::DCRSR, 3u);
  TEST_ASSERT(adi.Read32(TargetMemory::DCRDR) == 0x55AA55AAu);
  // 复位向量捕获：系统复位后仍停核，S_RESET_ST 置位一次 / reset vector catch: halted
  // after a system reset, S_RESET_ST set once
  adi.Write32(TargetMemory::DEMCR, 1u);
  adi.Write32(TargetMemory::AIRCR, 0x05FA0004u);
  TEST_ASSERT(t.memory.resets == 2 && t.memory.Halted());
  TEST_ASSERT((adi.Read32(TargetMemory::DHCSR) & (1u << 25)) != 0u);
  TEST_ASSERT((adi.Read32(TargetMemory::DHCSR) & (1u << 25)) == 0u);
  TEST_ASSERT(t.memory.Reg(3) == 0u);
  // 恢复运行 / resume
  adi.Write32(TargetMemory::DHCSR, 0xA05F0001u);
  TEST_ASSERT((adi.Read32(TargetMemory::DHCSR) & (1u << 17)) == 0u);
}

void RegularCases()
{
  TestSwitchToSwd();
  TestDpidrFirstAfterLineReset();
  TestMemApWords();
  TestByteLanes();
  TestStickyErrorAndAbort();
  TestWaitRetry();
  TestWaitRetryCount();
  TestWriteParityError();
  TestOverrunDetectDataPhase();
  TestTurnaround();
  TestBadReadParityAndDisconnect();
  TestRomTable();
  TestCoreDebug();
  std::puts("swd target: ADIv5 SW-DP and MEM-AP simulation");
}

}  // namespace

int main()
{
  RegularCases();
  return 0;
}
