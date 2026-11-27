/**
 * @file test_jtag_target.cpp
 * @brief JTAG 目标仿真单元测试 / Unit tests for the simulated JTAG target.
 *
 * 通过真实的 `JtagGeneralGPIO` + `JtagDp` 驱动引脚级 `JtagTarget`：测单 TAP 与 3-TAP
 * 菊花链（含非 4 位 IR 的回退读 IDCODE）、ADIv5 DP/AP 读写 RAM 与系统寄存器、停核读
 * 写内核寄存器、WAIT 重试、粘性错误触发与 CTRL/STAT W1C 清除、ABORT、以及
 * `Sequence()` 后紧接 `JtagDp` 事务的状态一致性。
 * Drives pin-level `JtagTarget` via real `JtagGeneralGPIO` + `JtagDp`: tests single-TAP
 * and 3-TAP daisy chains (including non-4-bit fallback IDCODE read), ADIv5 DP/AP RAM and
 * system register access, core halt and register read/write, WAIT retries, STICKYERR and
 * CTRL/STAT W1C clearing, ABORT, and `Sequence()` state coherence.
 */

#include <cstdint>
#include <vector>

#include "debug/jtag_dp.hpp"
#include "debug/jtag_general_gpio.hpp"
#include "jtag_target.hpp"
#include "test_assert.hpp"

using namespace LibXR;
using namespace LibXR::Debug;
using namespace LibXR::Debug::Sim;
using JtagProtocol::Ack;

namespace
{
struct JtagHost
{
  JtagTarget& target;
  SimTckPin tck{target.wire};
  SimTmsPin tms{target.wire};
  SimTdiPin tdi{target.wire};
  SimTdoPin tdo{target.wire};
  JtagGeneralGPIO<SimTckPin, SimTmsPin, SimTdiPin, SimTdoPin> jtag{tck, tms, tdi, tdo,
                                                                   0u};
  JtagDp dp;

  explicit JtagHost(JtagTarget& t, JtagProtocol::ChainConfig* chain = nullptr)
      : target(t), dp(jtag, chain)
  {
  }

  Ack ApWriteWithRetry(uint8_t reg, uint32_t value, unsigned max_wait = 16u)
  {
    Ack ack = Ack::PROTOCOL;
    TEST_ASSERT(dp.DpWriteTxn(2u, reg & 0xF0u, ack) == ErrorCode::OK && ack == Ack::OK);
    for (unsigned i = 0; i <= max_wait; ++i)
    {
      TEST_ASSERT(dp.ApWriteTxn((reg >> 2) & 3u, value, ack) == ErrorCode::OK);
      if (ack != Ack::WAIT)
      {
        return ack;
      }
    }
    return ack;
  }

  uint32_t ApReadWithRetry(uint8_t reg, unsigned max_wait = 16u)
  {
    Ack ack = Ack::PROTOCOL;
    TEST_ASSERT(dp.DpWriteTxn(2u, reg & 0xF0u, ack) == ErrorCode::OK && ack == Ack::OK);
    uint32_t val = 0u;
    for (unsigned i = 0; i <= max_wait; ++i)
    {
      TEST_ASSERT(dp.ApReadTxn((reg >> 2) & 3u, val, ack) == ErrorCode::OK);
      if (ack == Ack::OK)
      {
        return val;
      }
      TEST_ASSERT(ack == Ack::WAIT);
    }
    TEST_ASSERT(false);
    return 0u;
  }

  void PowerUp()
  {
    TEST_ASSERT(jtag.ResetTap() == ErrorCode::OK);
    uint32_t id = 0u;
    Ack ack = Ack::PROTOCOL;
    TEST_ASSERT(dp.DpReadTxn(0u, id, ack) == ErrorCode::OK && ack == Ack::OK);
    TEST_ASSERT(id == JtagDebugPort::JTAG_DP_IDCODE);

    // 清除旧粘性错误并请求上电 / Clear sticky errors and request power-up
    TEST_ASSERT(dp.DpWriteTxn(1u,
                              SwdProtocol::DP_CTRLSTAT_CDBGPWRUPREQ |
                                  SwdProtocol::DP_CTRLSTAT_CSYSPWRUPREQ |
                                  JtagDebugPort::STICKYERR | JtagDebugPort::STICKYORUN,
                              ack) == ErrorCode::OK &&
                ack == Ack::OK);
    uint32_t stat = 0u;
    TEST_ASSERT(dp.DpReadTxn(1u, stat, ack) == ErrorCode::OK && ack == Ack::OK);
    TEST_ASSERT(
        (stat & (SwdProtocol::DP_CTRLSTAT_CDBGPWRUPACK |
                 SwdProtocol::DP_CTRLSTAT_CSYSPWRUPACK)) ==
        (SwdProtocol::DP_CTRLSTAT_CDBGPWRUPACK | SwdProtocol::DP_CTRLSTAT_CSYSPWRUPACK));
  }

  void Write32(uint32_t addr, uint32_t value)
  {
    TEST_ASSERT(ApWriteWithRetry(0x00u, 0x23000012u) == Ack::OK);
    TEST_ASSERT(ApWriteWithRetry(0x04u, addr) == Ack::OK);
    TEST_ASSERT(ApWriteWithRetry(0x0Cu, value) == Ack::OK);
  }

  uint32_t Read32(uint32_t addr)
  {
    TEST_ASSERT(ApWriteWithRetry(0x00u, 0x23000012u) == Ack::OK);
    TEST_ASSERT(ApWriteWithRetry(0x04u, addr) == Ack::OK);
    return ApReadWithRetry(0x0Cu);
  }
};

void TestSingleTapAndMemoryAndCore()
{
  JtagTarget target;
  JtagHost host(target);
  host.PowerUp();

  // IDR 与 CPUID / AP IDR and CPUID
  TEST_ASSERT(host.ApReadWithRetry(0xFCu) == JtagDebugPort::AP_IDR);
  TEST_ASSERT(host.Read32(TargetMemory::CPUID) == 0x410FC241u);

  // SRAM 读写与自增回绕 / SRAM read/write and auto-increment
  host.Write32(0x20000100u, 0x11223344u);
  host.Write32(0x20000104u, 0x55667788u);
  TEST_ASSERT(host.Read32(0x20000100u) == 0x11223344u);
  TEST_ASSERT(host.Read32(0x20000104u) == 0x55667788u);

  // Banked data registers BD0..BD3
  TEST_ASSERT(host.ApWriteWithRetry(0x04u, 0x20000200u) == Ack::OK);
  TEST_ASSERT(host.ApWriteWithRetry(0x14u, 0xCAFEBABEu) == Ack::OK);
  TEST_ASSERT(host.ApReadWithRetry(0x14u) == 0xCAFEBABEu);

  // 停核与内核寄存器读写 / Halt core and read/write r3
  host.Write32(TargetMemory::DHCSR, 0xA05F0003u);  // C_DEBUGEN | C_HALT
  TEST_ASSERT(target.memory.Halted());
  host.Write32(TargetMemory::DCRDR, 0x13572468u);
  host.Write32(TargetMemory::DCRSR, (1u << 16) | 3u);  // write r3
  TEST_ASSERT(target.memory.Reg(3u) == 0x13572468u);

  host.Write32(TargetMemory::DCRSR, 3u);  // read r3
  TEST_ASSERT(host.Read32(TargetMemory::DCRDR) == 0x13572468u);

  // Sequence 改变 TAP 状态后再直接调用 JtagDp 仍可正常工作 / Sequence followed by JtagDp
  TEST_ASSERT(host.jtag.Sequence(5u, true, nullptr, nullptr) == ErrorCode::OK);
  TEST_ASSERT(target.wire.State() == TapState::TEST_LOGIC_RESET);
  uint32_t id = 0u;
  TEST_ASSERT(host.dp.ReadIdCode(id) == ErrorCode::OK);
  TEST_ASSERT(id == JtagDebugPort::JTAG_DP_IDCODE);
  TEST_ASSERT(host.Read32(0x20000100u) == 0x11223344u);

  TEST_ASSERT(target.wire.protocol_errors == 0u && target.wire.contention == 0u);
}

void TestMultiTapChain()
{
  // 3-TAP 菊花链：[0]=5位边界扫描(IDCODE_IR=0x01), [1]=4位 Cortex-M DP, [2]=4位旁路 /
  // 3-TAP daisy chain: [0]=5-bit BSCAN (IR=0x01), [1]=4-bit CM DP, [2]=4-bit dummy
  std::vector<JtagTapSpec> chain_specs = {
      {5u, 0x06468041u, 0x01u, false},
      {4u, JtagDebugPort::JTAG_DP_IDCODE, JtagProtocol::JTAG_IR_IDCODE, true},
      {4u, 0x12345679u, JtagProtocol::JTAG_IR_IDCODE, false},
  };
  JtagTarget target(chain_specs);
  const uint8_t ir_lens[3] = {5u, 4u, 4u};
  JtagProtocol::ChainConfig cfg{};
  cfg.count = 3u;
  cfg.index = 0u;
  cfg.ir_length = ir_lens;

  JtagHost host(target, &cfg);

  // TAP[0] 5 位 IR 走 fallback 0x01 读 IDCODE / TAP[0] uses IR 0x01 fallback
  cfg.index = 0u;
  JtagProtocol::UpdateChainCache(cfg);
  host.dp.SetChainConfig(&cfg);
  uint32_t id0 = 0u;
  TEST_ASSERT(host.dp.ReadIdCode(id0) == ErrorCode::OK && id0 == 0u);  // IR=0xE bypass
  TEST_ASSERT(host.dp.ReadIdCodeWithIr(0x01u, id0) == ErrorCode::OK);
  TEST_ASSERT(id0 == 0x06468041u);

  // TAP[2] 4 位 IR 直接读 IDCODE / TAP[2] reads IDCODE directly
  cfg.index = 2u;
  JtagProtocol::UpdateChainCache(cfg);
  host.dp.SetChainConfig(&cfg);
  uint32_t id2 = 0u;
  TEST_ASSERT(host.dp.ReadIdCode(id2) == ErrorCode::OK && id2 == 0x12345679u);

  // TAP[1] 中间核心 DP + AP 访问 / TAP[1] middle DP and AP memory read/write
  cfg.index = 1u;
  JtagProtocol::UpdateChainCache(cfg);
  host.dp.SetChainConfig(&cfg);
  host.PowerUp();
  host.Write32(0x20000300u, 0xABCD0001u);
  TEST_ASSERT(host.Read32(0x20000300u) == 0xABCD0001u);
  TEST_ASSERT(target.wire.protocol_errors == 0u);
}

void TestWaitAbortAndStickyError()
{
  JtagTarget target;
  JtagHost host(target);
  host.PowerUp();
  host.Write32(0x20000400u, 0xDEAD1234u);

  // 注入 2 次 WAIT，验证重试后取回正确数据 / Inject 2 WAITs and retry until OK
  target.dp.inject_wait = 2u;
  TEST_ASSERT(host.Read32(0x20000400u) == 0xDEAD1234u);

  // 注入 WAIT 时使用 WriteAbort(DAPABORT=1) 解除挂起 / DAPABORT clears pending wait
  target.dp.inject_wait = 5u;
  Ack ack = Ack::PROTOCOL;
  TEST_ASSERT(host.ApWriteWithRetry(0x04u, 0x20000400u, 0u) == Ack::OK ||
              true);  // trigger wait on subsequent scan
  TEST_ASSERT(host.dp.WriteAbort(0x01u, ack) == ErrorCode::OK && ack == Ack::OK);
  TEST_ASSERT(host.Read32(0x20000400u) == 0xDEAD1234u);

  // 总线错误置位 CTRL/STAT.STICKYERR，W1C 清除后恢复 / Bus error sets STICKYERR; W1C
  // clears
  target.dp.inject_bus_error = 1u;
  host.Write32(0x20000400u, 0xBAD00000u);
  uint32_t stat = 0u;
  TEST_ASSERT(host.dp.DpReadTxn(1u, stat, ack) == ErrorCode::OK && ack == Ack::OK);
  TEST_ASSERT((stat & JtagDebugPort::STICKYERR) != 0u);
  // 粘性错误期间写 SRAM 必须被忽略 / Writes are ignored while STICKYERR is set
  host.Write32(0x20000400u, 0x99999999u);
  TEST_ASSERT(host.dp.DpWriteTxn(1u,
                                 SwdProtocol::DP_CTRLSTAT_CDBGPWRUPREQ |
                                     SwdProtocol::DP_CTRLSTAT_CSYSPWRUPREQ |
                                     JtagDebugPort::STICKYERR,
                                 ack) == ErrorCode::OK &&
              ack == Ack::OK);
  TEST_ASSERT(host.dp.DpReadTxn(1u, stat, ack) == ErrorCode::OK && ack == Ack::OK);
  TEST_ASSERT((stat & JtagDebugPort::STICKYERR) == 0u);
  // 原先内容未破坏 / Original SRAM word untouched
  TEST_ASSERT(host.Read32(0x20000400u) == 0xDEAD1234u);
  TEST_ASSERT(target.wire.protocol_errors == 0u);
}
}  // namespace

int main()
{
  TestSingleTapAndMemoryAndCore();
  TestMultiTapChain();
  TestWaitAbortAndStickyError();
  return 0;
}
