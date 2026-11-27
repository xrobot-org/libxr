/**
 * @file test_jtag_general_gpio.cpp
 * @brief JtagGeneralGPIO 主机测试 / Host tests for JtagGeneralGPIO.
 */

#include <cstdint>
#include <vector>

#include "debug/jtag_general_gpio.hpp"
#include "gpio.hpp"
#include "test_assert.hpp"

using namespace LibXR;

namespace
{
/// 记录每次写入与上升沿采样的引脚 / Recording GPIO capturing writes and rising edges
class RecordingPin : public GPIO
{
 public:
  bool Read() override { return level; }
  void Write(bool value) override
  {
    level = value;
    writes.push_back(value);
  }
  ErrorCode EnableInterrupt() override { return ErrorCode::OK; }
  ErrorCode DisableInterrupt() override { return ErrorCode::OK; }
  ErrorCode SetConfig(Configuration cfg) override
  {
    last_cfg = cfg;
    ++cfg_calls;
    return ErrorCode::OK;
  }

  bool level = false;
  Configuration last_cfg{};
  unsigned cfg_calls = 0u;
  std::vector<bool> writes;
};

struct SampledEdge
{
  bool tms = false;
  bool tdi = false;
};

struct AnsweringTdoPin : public RecordingPin
{
  bool Read() override
  {
    return next < answers.size() ? static_cast<bool>(answers[next++]) : false;
  }
  std::vector<bool> answers;
  size_t next = 0u;
};

struct TckMonitorPin : public RecordingPin
{
  RecordingPin* tms = nullptr;
  RecordingPin* tdi = nullptr;
  std::vector<SampledEdge> edges;
  std::function<void(bool rising)> on_edge;

  void Write(bool value) override
  {
    const bool rising = value && !level;
    const bool falling = !value && level;
    RecordingPin::Write(value);
    if (rising && tms != nullptr && tdi != nullptr)
    {
      edges.push_back({tms->level, tdi->level});
      if (on_edge)
      {
        on_edge(true);
      }
    }
    else if (falling && on_edge)
    {
      on_edge(false);
    }
  }
};

void TestInitAndCloseAndClock()
{
  TckMonitorPin tck;
  RecordingPin tms;
  RecordingPin tdi;
  AnsweringTdoPin tdo;
  tck.tms = &tms;
  tck.tdi = &tdi;

  Debug::JtagGeneralGPIO<TckMonitorPin, RecordingPin, RecordingPin, AnsweringTdoPin> jtag(
      tck, tms, tdi, tdo, 0u);

  // 初始化配置方向并在构造结束时复位 TAP（5 个 TMS=1）/ Init configures directions and
  // clocks 5 TMS=1 resets
  TEST_ASSERT(tck.cfg_calls == 1u &&
              tck.last_cfg.direction == GPIO::Direction::OUTPUT_PUSH_PULL);
  TEST_ASSERT(tms.cfg_calls == 1u && tms.last_cfg.pull == GPIO::Pull::UP);
  TEST_ASSERT(tdi.cfg_calls == 1u);
  TEST_ASSERT(tdo.cfg_calls == 1u && tdo.last_cfg.direction == GPIO::Direction::INPUT);
  TEST_ASSERT(tck.edges.size() == 5u);
  for (const auto& edge : tck.edges)
  {
    TEST_ASSERT(edge.tms);
  }

  // 频率夹逼与 0 Hz 均返回 OK / Clock bounds and 0 Hz succeed
  TEST_ASSERT(jtag.SetClockHz(0u) == ErrorCode::OK);
  TEST_ASSERT(jtag.SetClockHz(100u) == ErrorCode::OK);
  TEST_ASSERT(jtag.SetClockHz(200'000'000u) == ErrorCode::OK);

  // 不支持直接跳到中间态 / Unsupported target states return NOT_SUPPORT
  TEST_ASSERT(jtag.GotoState(Debug::TapState::PAUSE_DR) == ErrorCode::NOT_SUPPORT);
  TEST_ASSERT(jtag.GotoState(Debug::TapState::EXIT1_IR) == ErrorCode::NOT_SUPPORT);

  jtag.Close();
  TEST_ASSERT(!tck.level && tms.level && !tdi.level);
}

void TestShiftIrDrAndTiming()
{
  TckMonitorPin tck;
  RecordingPin tms;
  RecordingPin tdi;
  AnsweringTdoPin tdo;
  tck.tms = &tms;
  tck.tdi = &tdi;

  Debug::JtagGeneralGPIO<TckMonitorPin, RecordingPin, RecordingPin, AnsweringTdoPin> jtag(
      tck, tms, tdi, tdo, 0u);
  tck.edges.clear();

  // 4-bit ShiftIR 发送 0xA（LSB-first: 0,1,0,1），读回 0x5（1,0,1,0） /
  // ShiftIR 4 bits 0xA (LSB-first: 0,1,0,1), reading back 0x5 (1,0,1,0)
  tdo.answers = {true, false, true, false};
  const uint8_t tx_ir = 0x0Au;
  uint8_t rx_ir = 0xFFu;
  TEST_ASSERT(jtag.ShiftIR(4u, &tx_ir, &rx_ir) == ErrorCode::OK);
  TEST_ASSERT(rx_ir == 0x05u);
  TEST_ASSERT(tdo.next == 4u);

  // 构造结束在 TLR：EnsureIdle 先 ResetTap(5 拍 TMS=1) + 1 拍 Idle(TMS=0) +
  // Shift-IR(4 拍: 1,1,0,0) + 4 拍移位(前 3 拍 TMS=0, 末拍 TMS=1) + ExitToIdle(2 拍:
  // 1,0) = 16 拍 / Starts at TLR: 5(reset) + 1(RTI) + 4(Shift-IR) + 4(data) + 2(RTI)
  TEST_ASSERT(tck.edges.size() == 16u);
  // 验证 TMS 序列 / Check TMS sequence
  const bool expected_tms[16] = {true,  true,  true,  true,  true,  false, true, true,
                                 false, false, false, false, false, true,  true, false};
  for (size_t i = 0; i < 16u; ++i)
  {
    TEST_ASSERT(tck.edges[i].tms == expected_tms[i]);
  }
  // 验证 4 位 IR TDI 数据位（下标 10..13）：0xA LSB first -> 0, 1, 0, 1
  TEST_ASSERT(!tck.edges[10].tdi && tck.edges[11].tdi && !tck.edges[12].tdi &&
              tck.edges[13].tdi);

  // 紧接着 ShiftDR（此时已在 RUN_TEST_IDLE，只需要 3+bits+2 拍），且 0 bit 为空操作 /
  // Consecutive ShiftDR from RUN_TEST_IDLE takes 3 + bits + 2 clocks
  tck.edges.clear();
  TEST_ASSERT(jtag.ShiftDR(0u, &tx_ir, &rx_ir) == ErrorCode::OK);
  TEST_ASSERT(tck.edges.empty());

  tdo.answers = {false, true, true, false, true, false, false, true};
  tdo.next = 0u;
  const uint8_t tx_dr[1] = {0x3Cu};
  uint8_t rx_dr[1] = {0u};
  TEST_ASSERT(jtag.ShiftDR(8u, tx_dr, rx_dr) == ErrorCode::OK);
  TEST_ASSERT(rx_dr[0] == 0x96u);  // 0,1,1,0,1,0,0,1 LSB first = 0b10010110 = 0x96
  TEST_ASSERT(tck.edges.size() == 3u + 8u + 2u);
  uint8_t observed_tdi = 0u;
  for (size_t i = 0; i < 8u; ++i)
  {
    if (tck.edges[3u + i].tdi)
    {
      observed_tdi = static_cast<uint8_t>(observed_tdi | (1u << i));
    }
  }
  TEST_ASSERT(observed_tdi == 0x3Cu);
  // 前 7 位 TMS=0，第 8 位 TMS=1 进入 Exit1-DR / First 7 TMS=0, last TMS=1
  for (size_t i = 0; i < 7u; ++i)
  {
    TEST_ASSERT(!tck.edges[3u + i].tms);
  }
  TEST_ASSERT(tck.edges[3u + 7u].tms);
}

void TestSequenceUpdatesTapStateAndIdleClocks()
{
  // 使用真实 JTAG 状态机跟踪器验证 Sequence 后再调 ShiftIR/ShiftDR 不会状态脱节 /
  // Verify Sequence followed by ShiftIR/ShiftDR stays in sync with real IEEE 1149.1 FSM
  auto step_fsm = [](Debug::TapState s, bool tms) -> Debug::TapState
  {
    using S = Debug::TapState;
    switch (s)
    {
      case S::TEST_LOGIC_RESET:
        return tms ? S::TEST_LOGIC_RESET : S::RUN_TEST_IDLE;
      case S::RUN_TEST_IDLE:
        return tms ? S::SELECT_DR_SCAN : S::RUN_TEST_IDLE;
      case S::SELECT_DR_SCAN:
        return tms ? S::SELECT_IR_SCAN : S::CAPTURE_DR;
      case S::CAPTURE_DR:
        return tms ? S::EXIT1_DR : S::SHIFT_DR;
      case S::SHIFT_DR:
        return tms ? S::EXIT1_DR : S::SHIFT_DR;
      case S::EXIT1_DR:
        return tms ? S::UPDATE_DR : S::PAUSE_DR;
      case S::PAUSE_DR:
        return tms ? S::EXIT2_DR : S::PAUSE_DR;
      case S::EXIT2_DR:
        return tms ? S::UPDATE_DR : S::SHIFT_DR;
      case S::UPDATE_DR:
        return tms ? S::SELECT_DR_SCAN : S::RUN_TEST_IDLE;
      case S::SELECT_IR_SCAN:
        return tms ? S::TEST_LOGIC_RESET : S::CAPTURE_IR;
      case S::CAPTURE_IR:
        return tms ? S::EXIT1_IR : S::SHIFT_IR;
      case S::SHIFT_IR:
        return tms ? S::EXIT1_IR : S::SHIFT_IR;
      case S::EXIT1_IR:
        return tms ? S::UPDATE_IR : S::PAUSE_IR;
      case S::PAUSE_IR:
        return tms ? S::EXIT2_IR : S::PAUSE_IR;
      case S::EXIT2_IR:
        return tms ? S::UPDATE_IR : S::SHIFT_IR;
      case S::UPDATE_IR:
        return tms ? S::SELECT_DR_SCAN : S::RUN_TEST_IDLE;
    }
    return S::TEST_LOGIC_RESET;
  };

  TckMonitorPin tck;
  RecordingPin tms;
  RecordingPin tdi;
  AnsweringTdoPin tdo;
  tck.tms = &tms;
  tck.tdi = &tdi;

  Debug::TapState hw_state = Debug::TapState::TEST_LOGIC_RESET;
  std::vector<Debug::TapState> states_in_shift;
  tck.on_edge = [&](bool rising)
  {
    if (!rising)
    {
      return;
    }
    if (hw_state == Debug::TapState::SHIFT_IR || hw_state == Debug::TapState::SHIFT_DR)
    {
      states_in_shift.push_back(hw_state);
    }
    hw_state = step_fsm(hw_state, tms.level);
  };

  Debug::JtagGeneralGPIO<TckMonitorPin, RecordingPin, RecordingPin, AnsweringTdoPin> jtag(
      tck, tms, tdi, tdo, 0u);
  TEST_ASSERT(hw_state == Debug::TapState::TEST_LOGIC_RESET);

  // 先用 ShiftDR 使主机内部处于 RUN_TEST_IDLE / Enter RUN_TEST_IDLE via ShiftDR
  uint8_t dummy = 0u;
  TEST_ASSERT(jtag.ShiftDR(1u, &dummy, nullptr) == ErrorCode::OK);
  TEST_ASSERT(hw_state == Debug::TapState::RUN_TEST_IDLE);

  // 然后调用 Sequence(5, true, ...) 把硬件 TAP 打回 TEST_LOGIC_RESET（OpenOCD TLR 做法）
  // / Clock 5 TMS=1 via Sequence() to reach TEST_LOGIC_RESET without calling ResetTap()
  tdo.answers = {true, false, true, false, true};
  tdo.next = 0u;
  uint8_t cap = 0u;
  TEST_ASSERT(jtag.Sequence(5u, true, nullptr, &cap) == ErrorCode::OK);
  TEST_ASSERT(cap == 0x15u);
  TEST_ASSERT(hw_state == Debug::TapState::TEST_LOGIC_RESET);

  // 紧接着直接调用 ShiftIR：硬件必须精确在 SHIFT_IR 移位这 4 位 /
  // Immediate ShiftIR must clock all 4 IR bits inside hardware SHIFT_IR
  states_in_shift.clear();
  const uint8_t ir = 0x0Eu;
  TEST_ASSERT(jtag.ShiftIR(4u, &ir, nullptr) == ErrorCode::OK);
  TEST_ASSERT(states_in_shift.size() == 4u);
  for (auto s : states_in_shift)
  {
    TEST_ASSERT(s == Debug::TapState::SHIFT_IR);
  }
  TEST_ASSERT(hw_state == Debug::TapState::RUN_TEST_IDLE);

  // 用 Sequence 从 RUN_TEST_IDLE 进入 SELECT_DR_SCAN(tms=1) 后再进 UPDATE_DR 经 pause
  // 等： 打 1 拍 TMS=1 停留在 SELECT_DR_SCAN，之后立刻 ShiftDR 仍须正确进 SHIFT_DR / 1
  // clock with TMS=1 leaves TAP at SELECT_DR_SCAN; next ShiftDR still works
  TEST_ASSERT(jtag.Sequence(1u, true, nullptr, nullptr) == ErrorCode::OK);
  TEST_ASSERT(hw_state == Debug::TapState::SELECT_DR_SCAN);
  states_in_shift.clear();
  const uint32_t dr = 0x12345678u;
  TEST_ASSERT(jtag.ShiftDR(32u, reinterpret_cast<const uint8_t*>(&dr), nullptr) ==
              ErrorCode::OK);
  TEST_ASSERT(states_in_shift.size() == 32u);
  for (auto s : states_in_shift)
  {
    TEST_ASSERT(s == Debug::TapState::SHIFT_DR);
  }
  TEST_ASSERT(hw_state == Debug::TapState::RUN_TEST_IDLE);

  // IdleClocks 插入 TMS=0 时钟且保持在 RUN_TEST_IDLE / IdleClocks stays in RUN_TEST_IDLE
  tck.edges.clear();
  jtag.IdleClocks(3u);
  TEST_ASSERT(tck.edges.size() == 3u);
  for (const auto& edge : tck.edges)
  {
    TEST_ASSERT(!edge.tms && !edge.tdi);
  }
  TEST_ASSERT(hw_state == Debug::TapState::RUN_TEST_IDLE);
}
}  // namespace

void test_jtag_general_gpio()
{
  TestInitAndCloseAndClock();
  TestShiftIrDrAndTiming();
  TestSequenceUpdatesTapStateAndIdleClocks();
}
