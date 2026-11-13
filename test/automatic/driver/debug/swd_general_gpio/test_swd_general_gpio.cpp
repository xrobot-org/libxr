/**
 * @file test_swd_general_gpio.cpp
 * @brief SwdGeneralGPIO 主机测试 / Host tests for SwdGeneralGPIO.
 */

#include <cstddef>
#include <vector>

#include "debug/swd_general_gpio.hpp"
#include "gpio.hpp"
#include "test_assert.hpp"

using namespace LibXR;

namespace
{
/// 记录每次写入电平的引脚 / A pin that records every level written to it.
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
  ErrorCode SetConfig(Configuration) override { return ErrorCode::OK; }

  bool level = true;         ///< 当前电平 / Current level.
  std::vector<bool> writes;  ///< 写入过的电平 / Levels written.
};

/// SWCLK 引脚：在每个上升沿记下 SWDIO 的电平，与目标采样的时刻相同
/// SWCLK pin: records the SWDIO level at every rising edge, as a target samples it.
struct ClockedPin : public RecordingPin
{
  RecordingPin* data = nullptr;  ///< SWDIO 引脚 / The SWDIO pin.
  std::vector<bool> sampled;     ///< 各上升沿的 SWDIO 电平 / SWDIO at each rising edge.
  void Write(bool value) override
  {
    const bool RISING = value && !level;
    RecordingPin::Write(value);
    if (RISING && data != nullptr)
    {
      sampled.push_back(data->level);
    }
  }
};

/// 空闲周期里 SWDIO 保持低电平：高电平会被目标当成起始位
/// Idle cycles keep SWDIO low: a high level would be read by the target as a start bit.
void TestIdleClocksDriveSwdioLow()
{
  ClockedPin swclk;
  RecordingPin swdio;
  swclk.data = &swdio;
  Debug::SwdGeneralGPIO<ClockedPin, RecordingPin> swd(swclk, swdio, 0u,
                                                      Debug::SwdGpioTiming{});
  swclk.sampled.clear();

  swd.IdleClocks(4);
  TEST_ASSERT(swclk.sampled.size() == 4);
  for (const bool level : swclk.sampled)
  {
    TEST_ASSERT(!level);
  }
  // 之后也保持低电平 / It stays low afterwards.
  TEST_ASSERT(!swdio.level);
}

/// 传输的请求按 ADIv5 的包请求格式发出，LSB 在先：DP 读 IDCODE 在线上是 0xA5
/// The request of a transfer goes out as the ADIv5 packet request, LSB first: a DP read
/// of IDCODE is 0xA5 on the wire.
void TestRequestOnTheWire()
{
  ClockedPin swclk;
  RecordingPin swdio;
  swclk.data = &swdio;
  Debug::SwdGeneralGPIO<ClockedPin, RecordingPin> swd(swclk, swdio, 0u,
                                                      Debug::SwdGpioTiming{});
  swclk.sampled.clear();

  Debug::SwdProtocol::Response resp{};
  (void)swd.Transfer(
      Debug::SwdProtocol::MakeDpReadRequest(Debug::SwdProtocol::DpReadReg::IDCODE), resp);
  TEST_ASSERT(swclk.sampled.size() >= 8);
  uint8_t request = 0;
  for (unsigned i = 0; i < 8; ++i)
  {
    if (swclk.sampled[i])
    {
      request = static_cast<uint8_t>(request | (1u << i));
    }
  }
  TEST_ASSERT(request == 0xA5u);
}

/// SWDIO 引脚：读的时候先依次返回预设的电平，代替目标的应答
/// SWDIO pin whose reads return the scripted levels first, in place of a target's answer.
struct AnsweringPin : public RecordingPin
{
  bool Read() override
  {
    return next < answers.size() ? static_cast<bool>(answers[next++]) : level;
  }

  std::vector<bool> answers;  ///< 目标要返回的电平 / Levels the target returns.
  size_t next = 0;            ///< 下一个要返回的位置 / Index of the next level.
};

/// 写传输在校验位之后把 SWDIO 拉低，再打 3 个时钟：目标用它们完成写入
/// After the parity bit a write transfer holds SWDIO low for 3 more clocks, which the
/// target uses to complete the write.
void TestWriteIsFollowedByIdleClocks()
{
  ClockedPin swclk;
  AnsweringPin swdio;
  swclk.data = &swdio;
  Debug::SwdGeneralGPIO<ClockedPin, AnsweringPin> swd(swclk, swdio, 0u,
                                                      Debug::SwdGpioTiming{});
  swclk.sampled.clear();

  // ACK 为 OK：三位依次是 1、0、0。
  // The ACK is OK: its three bits are 1, 0, 0 in this order.
  swdio.answers = {true, false, false};
  // 最高两位和校验位都是 1，后面的低电平才看得出来。
  // The top two data bits and the parity bit are 1, so that the low level after them
  // shows.
  constexpr uint32_t WDATA = 0xC000'0001u;
  Debug::SwdProtocol::Response resp{};
  TEST_ASSERT(swd.Transfer(Debug::SwdProtocol::MakeApWriteRequest(1u, WDATA), resp) ==
              ErrorCode::OK);
  TEST_ASSERT(resp.ack == Debug::SwdProtocol::Ack::OK);

  // 上升沿依次是：请求 8 个，转向 1 个，ACK 3 个，转向 1 个，数据 32 个，校验 1 个，
  // 然后是写入后的 3 个。
  // Rising edges in order: 8 of the request, 1 turnaround, 3 of the ACK, 1 turnaround,
  // 32 data bits, 1 parity bit, then the 3 after the write.
  constexpr size_t DATA_AT = 8u + 1u + 3u + 1u;
  constexpr size_t PARITY_AT = DATA_AT + 32u;
  TEST_ASSERT(swclk.sampled.size() == PARITY_AT + 1u + 3u);
  uint32_t data = 0;
  for (unsigned i = 0; i < 32u; ++i)
  {
    if (swclk.sampled[DATA_AT + i])
    {
      data |= (1u << i);
    }
  }
  TEST_ASSERT(data == WDATA);
  TEST_ASSERT(swclk.sampled[PARITY_AT]);
  for (size_t i = PARITY_AT + 1u; i < swclk.sampled.size(); ++i)
  {
    TEST_ASSERT(!swclk.sampled[i]);
  }
}

/// SetClockHz 只在无延时路径（每个时钟 c 个周期）不超速时（f >= f_cpu / c）用它；否则在
/// N >= 1（a + b·N）中取最接近请求周期的一档，一样近时取较慢的。这里用一组假想的参数：
/// CPU 100 MHz，b = 10 周期，a = 15 周期，c = 12 周期。行尾注释是请求的每时钟周期数。
/// SetClockHz takes the path without delay (c cycles per clock) only when it does not
/// overshoot (f >= f_cpu / c); otherwise it picks the N >= 1 (a + b·N) closest to the
/// requested period, the slower one on a tie. Synthetic port: 100 MHz CPU, b = 10
/// cycles, a = 15 cycles, c = 12 cycles. The trailing comments give the requested cycles
/// per clock.
void TestDelayLoopsPickTheNearestStep()
{
  constexpr uint32_t CPU = 100'000'000u;
  constexpr Debug::SwdGpioTiming T{10.0f, 15.0f, 12.0f};
  TEST_ASSERT(T.DelayLoops(CPU, 3'000'000u) == 2u);  // 33.3: 35 beats 25
  TEST_ASSERT(T.DelayLoops(CPU, 4'000'000u) == 1u);  // 25: exact
  TEST_ASSERT(T.DelayLoops(CPU, 5'000'000u) == 1u);  // 20: 25 beats 12
  // 16.7 周期：12 比 25 更近，但快于请求值，所以 N = 1。
  // 16.7 cycles: 12 would be nearer than 25 but faster than requested, so N = 1.
  TEST_ASSERT(T.DelayLoops(CPU, 6'000'000u) == 1u);
  TEST_ASSERT(T.DelayLoops(CPU, 10'000'000u) == 0u);  // 10: no delay
  TEST_ASSERT(T.DelayLoops(CPU, 1'000'000u) == 9u);   // 100: 95 / 105 tie -> 105
  TEST_ASSERT(T.DelayLoops(CPU, 2'000'000u) == 4u);   // 50: 45 / 55 tie -> 55
  TEST_ASSERT(T.DelayLoops(CPU, 50'000u) == 199u);    // 2000: 1995 / 2005 tie
  // 18.5 周期，在 12 和 25 正中间：无延时路径会超速。
  // 18.5 cycles, halfway between 12 and 25: the path without delay would overshoot.
  TEST_ASSERT(T.DelayLoops(37'000'000u, 2'000'000u) == 1u);
  // 正好 12 周期（f == f_cpu / c）：无延时路径不超速。
  // 12 cycles exactly (f == f_cpu / c): the path without delay does not overshoot.
  TEST_ASSERT(T.DelayLoops(96'000'000u, 8'000'000u) == 0u);
  // 12.5 周期：略大于 c，所以 N = 1，尽管 12 比 25 近得多。
  // 12.5 cycles: just above c, so N = 1 although 12 is far nearer than 25.
  TEST_ASSERT(T.DelayLoops(CPU, 8'000'000u) == 1u);
  // CPU 主频未知时保持无延时路径 / An unknown CPU clock keeps the path without delay.
  TEST_ASSERT(T.DelayLoops(0u, 1'000'000u) == 0u);
  // 默认值：每档 2 周期，截距 0，无延时路径 1 周期。
  // Defaults: 2 cycles per step, no intercept, 1 cycle without delay.
  TEST_ASSERT(Debug::SwdGpioTiming{}.DelayLoops(CPU, 1'000'000u) == 50u);
}

/// 扫一遍频率：无延时路径当且仅当不超速时被选中；否则选中的 N >= 1 不比相邻两档离请求
/// 值更远，一样近时是较慢的一档，并且 N >= 2 时误差不超过半档。
/// Over a sweep: the path without delay is chosen exactly when it does not overshoot;
/// otherwise the chosen N >= 1 is at least as close to the request as its neighbours,
/// the slower on a tie, and within half a step when N >= 2.
void TestDelayLoopsNearestStepSweep()
{
  constexpr uint32_t CPU = 64'000'000u;
  // 判据用自己的定点数（1/10 周期）计算，不读被测类的内部表示。
  // The oracle computes with its own fixed point (tenths of a cycle) and does not read
  // the representation inside the class under test.
  constexpr uint32_t B10 = 88u;
  constexpr uint32_t A10 = 160u;
  constexpr uint32_t C10 = 110u;
  constexpr Debug::SwdGpioTiming T{8.8f, 16.0f, 11.0f};
  const uint64_t TARGET = static_cast<uint64_t>(CPU) * 10u;
  auto cost = [&](uint32_t n) -> uint64_t { return A10 + uint64_t{B10} * n; };
  auto err = [&](uint64_t c, uint32_t hz) -> uint64_t
  {
    const uint64_t X = c * hz;
    return X > TARGET ? X - TARGET : TARGET - X;
  };
  for (uint32_t hz = 50'000u; hz <= 20'000'000u; hz += 25'000u)
  {
    const uint32_t N = T.DelayLoops(CPU, hz);
    const bool FAST_OK = uint64_t{C10} * hz >= TARGET;
    TEST_ASSERT((N == 0u) == FAST_OK);
    if (N == 0u)
    {
      continue;
    }
    const uint64_t E = err(cost(N), hz);
    for (const uint32_t OTHER : {N - 1u, N + 1u})
    {
      if (OTHER == 0u)
      {
        continue;
      }
      const uint64_t OE = err(cost(OTHER), hz);
      TEST_ASSERT(E <= OE);
      if (E == OE)
      {
        TEST_ASSERT(cost(N) > cost(OTHER));
      }
    }
    if (N >= 2u)
    {
      // |a + b·N − 目标| <= b / 2，两边都乘以 hz 和 10。
      // |a + b·N - target| <= b / 2, all scaled by hz and by 10.
      TEST_ASSERT(2u * E <= uint64_t{B10} * hz);
    }
  }
}
/// 周期数可以带一位小数，float 和 double 都接受，结果在编译期可用
/// Cycle counts may carry one decimal, float and double are both accepted, and the
/// result is usable at compile time.
void TestTimingFromFractionalCycles()
{
  constexpr uint32_t CPU = 72'000'000u;
  constexpr Debug::SwdGpioTiming T{9.0f, 16.9f, 11.4f};
  // c = 11.4：无延时路径从 72 MHz / 11.4 = 6315789.47 Hz 起才不超速。
  // c = 11.4: the path without delay stops overshooting at 72 MHz / 11.4 =
  // 6315789.47 Hz.
  static_assert(T.DelayLoops(CPU, 6'315'790u) == 0u);
  static_assert(T.DelayLoops(CPU, 6'315'789u) == 1u);
  // 1 MHz 是每周期 72 个周期：16.9 + 9.0 × 6 = 70.9 比 79.9 近。
  // 1 MHz is 72 cycles per period: 16.9 + 9.0 x 6 = 70.9 is nearer than 79.9.
  static_assert(T.DelayLoops(CPU, 1'000'000u) == 6u);
  // 100 kHz 是 720 个周期：16.9 + 9.0 × 78 = 718.9 比 727.9 近；300 kHz 是 240 个周期：
  // 16.9 + 9.0 × 25 = 241.9 比 232.9 近。
  // 100 kHz is 720 cycles: 16.9 + 9.0 x 78 = 718.9 is nearer than 727.9; 300 kHz is 240
  // cycles: 16.9 + 9.0 x 25 = 241.9 is nearer than 232.9.
  static_assert(T.DelayLoops(CPU, 100'000u) == 78u);
  static_assert(T.DelayLoops(CPU, 300'000u) == 25u);

  constexpr Debug::SwdGpioTiming FROM_DOUBLE{9.0, 16.9, 11.4};
  static_assert(FROM_DOUBLE.DelayLoops(CPU, 6'315'790u) == 0u);
  static_assert(FROM_DOUBLE.DelayLoops(CPU, 6'315'789u) == 1u);
  TEST_ASSERT(T.DelayLoops(CPU, 1'000'000u) == FROM_DOUBLE.DelayLoops(CPU, 1'000'000u));
}

/// 记录 BusyLoop() 调用的时基类，代替平台的时基类作为类型参数
/// Timebase class that records the BusyLoop() calls, used as the type parameter in place
/// of the timebase class of a platform.
class RecordingTimebase : public Timebase
{
 public:
  static void BusyLoop(uint32_t loops)
  {
    ++calls;
    last_loops = loops;
  }

  static inline uint32_t calls = 0;  ///< 调用次数 / Number of calls.
  static inline uint32_t last_loops =
      0;  ///< 最近一次的循环次数 / Loops of the last call.
};

/// 带延时的路径每个半周期调用一次所选时基类的 BusyLoop()，循环次数是 DelayLoops()
/// 的结果； 无延时路径不调用它 The delayed path calls BusyLoop() of the chosen timebase
/// class once per half period with the count from DelayLoops(); the path without delay
/// does not call it.
void TestDelayedPathUsesTheTimebaseClass()
{
  using DelayedGpio =
      Debug::SwdGeneralGPIO<ClockedPin, RecordingPin, Debug::SwdIoDriveMode::PUSH_PULL,
                            RecordingTimebase>;
  constexpr uint32_t CPU = 100'000'000u;
  constexpr Debug::SwdGpioTiming T{10.0f, 15.0f, 12.0f};
  ClockedPin swclk;
  RecordingPin swdio;
  swclk.data = &swdio;
  DelayedGpio swd(swclk, swdio, CPU, T);

  TEST_ASSERT(swd.SetClockHz(1'000'000u) == ErrorCode::OK);
  const uint32_t LOOPS = T.DelayLoops(CPU, 1'000'000u);
  TEST_ASSERT(LOOPS >= 1u);
  RecordingTimebase::calls = 0;
  swclk.sampled.clear();
  swd.IdleClocks(4);
  TEST_ASSERT(swclk.sampled.size() == 4);
  TEST_ASSERT(RecordingTimebase::calls == 8u);
  TEST_ASSERT(RecordingTimebase::last_loops == LOOPS);

  // 10 MHz 的请求走无延时路径 / A 10 MHz request takes the path without delay.
  TEST_ASSERT(swd.SetClockHz(10'000'000u) == ErrorCode::OK);
  RecordingTimebase::calls = 0;
  swd.IdleClocks(4);
  TEST_ASSERT(RecordingTimebase::calls == 0u);
}
}  // namespace

void test_swd_general_gpio()
{
  TestIdleClocksDriveSwdioLow();
  TestRequestOnTheWire();
  TestWriteIsFollowedByIdleClocks();
  TestDelayLoopsPickTheNearestStep();
  TestDelayLoopsNearestStepSweep();
  TestTimingFromFractionalCycles();
  TestDelayedPathUsesTheTimebaseClass();
}
