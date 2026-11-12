#pragma once

#include <cstdint>

#include "gpio.hpp"
#include "jtag.hpp"
#include "libxr_def.hpp"
#include "libxr_mem.hpp"
#include "timebase.hpp"

namespace LibXR::Debug
{
/**
 * @brief 基于 GpioType 轮询 bit-bang 的 JTAG 探针。
 *        JTAG probe based on polling bit-bang using GpioType.
 *
 * @tparam TckGpioType TCK GPIO 类型 / TCK GPIO type
 * @tparam TmsGpioType TMS GPIO 类型 / TMS GPIO type
 * @tparam TdiGpioType TDI GPIO 类型 / TDI GPIO type
 * @tparam TdoGpioType TDO GPIO 类型 / TDO GPIO type
 *
 * @note 推荐外围电路：TCK/TMS/TDI 串联 33Ω，TMS 上拉 10k；TDO 由目标驱动。
 *       Recommended circuit: 33Ω series on TCK/TMS/TDI, 10k pull-up on TMS; TDO
 *       driven by target.
 */
template <typename TckGpioType, typename TmsGpioType, typename TdiGpioType,
          typename TdoGpioType>
class JtagGeneralGPIO final : public Jtag
{
  /// 最低 TCK 频率（Hz） / Lowest TCK frequency (Hz).
  static constexpr uint32_t MIN_HZ = 10'000u;
  /// 最高 TCK 频率（Hz） / Highest TCK frequency (Hz).
  static constexpr uint32_t MAX_HZ = 100'000'000u;

  /// 每秒的纳秒数 / Nanoseconds per second.
  static constexpr uint32_t NS_PER_SEC = 1'000'000'000u;
  /// loops_per_us 乘以半周期纳秒数，再除以它，得到每个半周期的循环次数
  /// loops_per_us times the half period in ns, divided by this, is the loop count per
  /// half period.
  static constexpr uint32_t LOOPS_SCALE = 1000u;
  /// 上述除法向上取整用的偏置 / Bias that rounds this division up.
  static constexpr uint32_t CEIL_BIAS = LOOPS_SCALE - 1u;

  /// 默认 TCK 频率（Hz） / Default TCK frequency (Hz).
  static constexpr uint32_t DEFAULT_CLOCK_HZ = 500'000u;
  /// TMS 为 1 的时钟数：5 个时钟可从任何状态回到 Test-Logic-Reset
  /// Clocks with TMS high: five reach Test-Logic-Reset from any state.
  static constexpr uint32_t RESET_CYCLES = 5u;

  /// 最低频率下的半周期（ns，向上取整）
  /// Half period at the lowest frequency (ns, rounded up).
  static constexpr uint32_t HALF_PERIOD_NS_MAX =
      (NS_PER_SEC + (2u * MIN_HZ) - 1u) / (2u * MIN_HZ);
  /// loops_per_us 的上限：最低频率下每个半周期的循环次数仍在 uint32_t 范围内
  /// Upper limit of loops_per_us: the loop count per half period at the lowest frequency
  /// still fits uint32_t.
  static constexpr uint32_t MAX_LOOPS_PER_US =
      (UINT32_MAX - CEIL_BIAS) / HALF_PERIOD_NS_MAX;

 public:
  /**
   * @brief 配置四个引脚，设置默认 TCK 频率并复位 TAP
   *        Configures the four pins, sets the default TCK frequency and resets the TAP
   * @param tck 用作 TCK 的 GPIO / GPIO used as TCK.
   * @param tms 用作 TMS 的 GPIO / GPIO used as TMS.
   * @param tdi 用作 TDI 的 GPIO / GPIO used as TDI.
   * @param tdo 用作 TDO 的 GPIO / GPIO used as TDO.
   * @param loops_per_us 每微秒的延时循环次数，在目标板上按 Timebase::BusyLoop() 标定；
   *        0 表示不加延时 / Delay-loop iterations per microsecond, calibrated on the
   *        board against Timebase::BusyLoop(); 0 adds no delay.
   * @param default_hz 默认 TCK 频率（Hz） / Default TCK frequency (Hz).
   */
  explicit JtagGeneralGPIO(TckGpioType& tck, TmsGpioType& tms, TdiGpioType& tdi,
                           TdoGpioType& tdo, uint32_t loops_per_us,
                           uint32_t default_hz = DEFAULT_CLOCK_HZ)
      : tck_(tck), tms_(tms), tdi_(tdi), tdo_(tdo), loops_per_us_(loops_per_us)
  {
    if (loops_per_us_ > MAX_LOOPS_PER_US)
    {
      loops_per_us_ = MAX_LOOPS_PER_US;
    }

    tck_.SetConfig({TckGpioType::Direction::OUTPUT_PUSH_PULL, TckGpioType::Pull::NONE});
    tms_.SetConfig({TmsGpioType::Direction::OUTPUT_PUSH_PULL, TmsGpioType::Pull::UP});
    tdi_.SetConfig({TdiGpioType::Direction::OUTPUT_PUSH_PULL, TdiGpioType::Pull::NONE});
    tdo_.SetConfig({TdoGpioType::Direction::INPUT, TdoGpioType::Pull::NONE});

    tck_.Write(false);
    tms_.Write(true);
    tdi_.Write(false);

    (void)SetClockHz(default_hz);
    (void)ResetTap();
  }

  ~JtagGeneralGPIO() override = default;

  JtagGeneralGPIO(const JtagGeneralGPIO&) = delete;
  JtagGeneralGPIO& operator=(const JtagGeneralGPIO&) = delete;

  /**
   * @brief 设置 TCK 频率 / Set the TCK frequency
   *
   * 每个半周期的循环次数是 loops_per_us 乘以半周期（us），向上取整；不足一次时不加延时。
   * The loop count per half period is loops_per_us times the half period (us), rounded
   * up; below one iteration no delay is added.
   *
   * @param hz 目标频率（Hz），限制在 MIN_HZ 到 MAX_HZ 之间；0 表示不加延时 / Target
   *        frequency (Hz), clamped to MIN_HZ..MAX_HZ; 0 adds no delay.
   * @return 总是 OK / Always OK.
   */
  ErrorCode SetClockHz(uint32_t hz) override
  {
    if (hz == 0u)
    {
      clock_hz_ = 0u;
      half_period_ns_ = 0u;
      half_period_loops_ = 0u;
      return ErrorCode::OK;
    }

    if (hz < MIN_HZ)
    {
      hz = MIN_HZ;
    }
    if (hz > MAX_HZ)
    {
      hz = MAX_HZ;
    }

    clock_hz_ = hz;

    half_period_ns_ = (NS_PER_SEC + (2u * hz) - 1u) / (2u * hz);

    if (loops_per_us_ == 0u)
    {
      half_period_loops_ = 0u;
      return ErrorCode::OK;
    }

    const uint64_t LOOPS_SCALED =
        static_cast<uint64_t>(loops_per_us_) * static_cast<uint64_t>(half_period_ns_);
    if (LOOPS_SCALED < LOOPS_SCALE)
    {
      half_period_loops_ = 0u;
    }
    else
    {
      const uint64_t LOOPS_CEIL = (LOOPS_SCALED + CEIL_BIAS) / LOOPS_SCALE;
      half_period_loops_ =
          (LOOPS_CEIL > UINT32_MAX) ? UINT32_MAX : static_cast<uint32_t>(LOOPS_CEIL);
    }

    return ErrorCode::OK;
  }

  /// 引脚回到 TCK 低、TMS 高、TDI 低，仍保持输出
  /// The pins return to TCK low, TMS high and TDI low and stay outputs.
  void Close() override
  {
    tck_.Write(false);
    tms_.Write(true);
    tdi_.Write(false);
  }

  /// TMS 保持为 1 打 RESET_CYCLES 个时钟，TAP 回到 Test-Logic-Reset
  /// RESET_CYCLES clocks with TMS high take the TAP to Test-Logic-Reset.
  ErrorCode ResetTap() override
  {
    tms_.Write(true);
    for (uint32_t i = 0; i < RESET_CYCLES; ++i)
    {
      ClockCycle(true, false, nullptr);
    }
    current_state_ = TapState::TEST_LOGIC_RESET;
    return ErrorCode::OK;
  }

  /**
   * @brief 转到目标 TAP 状态 / Move to the target TAP state
   * @param target 只支持 TEST_LOGIC_RESET、RUN_TEST_IDLE、SHIFT_IR 和 SHIFT_DR / Only
   *        TEST_LOGIC_RESET, RUN_TEST_IDLE, SHIFT_IR and SHIFT_DR are supported.
   * @return OK；其他目标返回 NOT_SUPPORT / OK; NOT_SUPPORT for any other target.
   */
  ErrorCode GotoState(TapState target) override
  {
    if (target == current_state_)
    {
      return ErrorCode::OK;
    }

    switch (target)
    {
      case TapState::TEST_LOGIC_RESET:
        return ResetTap();
      case TapState::RUN_TEST_IDLE:
        EnsureIdle();
        return ErrorCode::OK;
      case TapState::SHIFT_IR:
        EnsureIdle();
        ApplyTmsSequence(SEQ_TO_SHIFT_IR, SEQ_TO_SHIFT_IR_LEN);
        current_state_ = TapState::SHIFT_IR;
        return ErrorCode::OK;
      case TapState::SHIFT_DR:
        EnsureIdle();
        ApplyTmsSequence(SEQ_TO_SHIFT_DR, SEQ_TO_SHIFT_DR_LEN);
        current_state_ = TapState::SHIFT_DR;
        return ErrorCode::OK;
      default:
        return ErrorCode::NOT_SUPPORT;
    }
  }

  /// 进入 Shift-IR 移位 bits 位，然后回到 Run-Test/Idle；bits 为 0 时什么都不做
  /// Enters Shift-IR, shifts the bits and returns to Run-Test/Idle; nothing for 0 bits.
  ErrorCode ShiftIR(uint32_t bits, const uint8_t* in_lsb_first,
                    uint8_t* out_lsb_first) override
  {
    if (bits == 0u)
    {
      return ErrorCode::OK;
    }

    const ErrorCode EC = GotoState(TapState::SHIFT_IR);
    if (EC != ErrorCode::OK)
    {
      return EC;
    }

    ShiftBits(bits, in_lsb_first, out_lsb_first);
    ExitToIdle();
    return ErrorCode::OK;
  }

  /// 进入 Shift-DR 移位 bits 位，然后回到 Run-Test/Idle；bits 为 0 时什么都不做
  /// Enters Shift-DR, shifts the bits and returns to Run-Test/Idle; nothing for 0 bits.
  ErrorCode ShiftDR(uint32_t bits, const uint8_t* in_lsb_first,
                    uint8_t* out_lsb_first) override
  {
    if (bits == 0u)
    {
      return ErrorCode::OK;
    }

    const ErrorCode EC = GotoState(TapState::SHIFT_DR);
    if (EC != ErrorCode::OK)
    {
      return EC;
    }

    ShiftBits(bits, in_lsb_first, out_lsb_first);
    ExitToIdle();
    return ErrorCode::OK;
  }

  /**
   * @brief TMS 固定，打 cycles 个时钟 / Clock cycles cycles with a fixed TMS
   *
   * 不更新本类记录的 TAP 状态：调用方用它改变了状态时，之后要自己调用 ResetTap()。
   * The TAP state tracked by this class is not updated: a caller that changes the state
   * with it has to call ResetTap() afterwards.
   *
   * @param cycles 时钟数 / Number of clocks.
   * @param tms 这些时钟里的 TMS 电平 / TMS level during these clocks.
   * @param tdi_lsb_first 要输出的 TDI 位，LSB 在先；为空时输出 0 / TDI bits to send, LSB
   *        first; null sends zeros.
   * @param tdo_lsb_first 不为空时存放采到的 TDO 位，LSB 在先 / When not null, receives
   *        the sampled TDO bits, LSB first.
   * @return 总是 OK / Always OK.
   */
  ErrorCode Sequence(uint32_t cycles, bool tms, const uint8_t* tdi_lsb_first,
                     uint8_t* tdo_lsb_first) override
  {
    if (cycles == 0u)
    {
      return ErrorCode::OK;
    }

    if (tdo_lsb_first != nullptr)
    {
      const uint32_t BYTES = (cycles + 7u) / 8u;
      Memory::FastSet(tdo_lsb_first, 0, BYTES);
    }

    for (uint32_t i = 0; i < cycles; ++i)
    {
      const bool TDI_VAL = (tdi_lsb_first == nullptr)
                               ? false
                               : (((tdi_lsb_first[i / 8u] >> (i & 7u)) & 0x1u) != 0u);
      bool tdo_bit = false;
      ClockCycle(tms, TDI_VAL, tdo_lsb_first ? &tdo_bit : nullptr);

      if (tdo_lsb_first != nullptr && tdo_bit)
      {
        tdo_lsb_first[i / 8u] =
            static_cast<uint8_t>(tdo_lsb_first[i / 8u] | (1u << (i & 7u)));
      }
    }

    return ErrorCode::OK;
  }

  /// 先回到 Run-Test/Idle，再以 TMS 为 0 打 cycles 个时钟
  /// Returns to Run-Test/Idle first, then clocks cycles cycles with TMS low.
  void IdleClocks(uint32_t cycles) override
  {
    if (cycles == 0u)
    {
      return;
    }

    (void)GotoState(TapState::RUN_TEST_IDLE);
    for (uint32_t i = 0; i < cycles; ++i)
    {
      ClockCycle(false, false, nullptr);
    }
  }

 private:
  /// 从 Run-Test/Idle 到 Shift-DR 的 TMS 序列：Select-DR-Scan、Capture-DR、Shift-DR
  /// TMS sequence from Run-Test/Idle to Shift-DR: Select-DR-Scan, Capture-DR, Shift-DR.
  static constexpr uint8_t SEQ_TO_SHIFT_DR[] = {1u, 0u, 0u};
  /// SEQ_TO_SHIFT_DR 的长度 / Length of SEQ_TO_SHIFT_DR.
  static constexpr uint32_t SEQ_TO_SHIFT_DR_LEN = 3u;
  /// 从 Run-Test/Idle 到 Shift-IR 的 TMS 序列：Select-DR-Scan、Select-IR-Scan、
  /// Capture-IR、Shift-IR
  /// TMS sequence from Run-Test/Idle to Shift-IR: Select-DR-Scan, Select-IR-Scan,
  /// Capture-IR, Shift-IR.
  static constexpr uint8_t SEQ_TO_SHIFT_IR[] = {1u, 1u, 0u, 0u};
  /// SEQ_TO_SHIFT_IR 的长度 / Length of SEQ_TO_SHIFT_IR.
  static constexpr uint32_t SEQ_TO_SHIFT_IR_LEN = 4u;

  /// 按序列每个时钟输出一个 TMS 值，TDI 为 0
  /// Clocks out one TMS value of the sequence per clock, with TDI low.
  void ApplyTmsSequence(const uint8_t* seq, uint32_t len)
  {
    for (uint32_t i = 0; i < len; ++i)
    {
      const bool TMS_VAL = (seq[i] != 0u);
      ClockCycle(TMS_VAL, false, nullptr);
    }
  }

  /// 不在 Run-Test/Idle 时，先复位 TAP，再以 TMS 为 0 打一个时钟进入它
  /// When not in Run-Test/Idle: resets the TAP, then one clock with TMS low enters it.
  void EnsureIdle()
  {
    if (current_state_ == TapState::RUN_TEST_IDLE)
    {
      return;
    }
    (void)ResetTap();
    ClockCycle(false, false, nullptr);
    current_state_ = TapState::RUN_TEST_IDLE;
  }

  /// 移位结束后从 Exit1 经 Update 回到 Run-Test/Idle
  /// After a shift, from Exit1 through Update back to Run-Test/Idle.
  void ExitToIdle()
  {
    // EXIT1 -> UPDATE (TMS=1), UPDATE -> IDLE (TMS=0)
    ClockCycle(true, false, nullptr);
    ClockCycle(false, false, nullptr);
    current_state_ = TapState::RUN_TEST_IDLE;
  }

  /**
   * @brief 在 Shift-IR 或 Shift-DR 状态移位 bits 位
   *        Shifts bits bits in Shift-IR or Shift-DR
   *
   * 前 bits - 1 位 TMS 为 0，最后一位 TMS 为 1，TAP 随最后一位进入 Exit1。
   * The first bits - 1 bits go out with TMS low and the last one with TMS high, which
   * takes the TAP to Exit1.
   *
   * @param bits 位数 / Number of bits.
   * @param in_lsb_first 要移入的位，LSB 在先；为空时移入 0 / Bits to shift in, LSB first;
   *        null shifts in zeros.
   * @param out_lsb_first 不为空时先清零，再存放移出的位 / When not null, cleared first
   * and then filled with the bits shifted out.
   */
  void ShiftBits(uint32_t bits, const uint8_t* in_lsb_first, uint8_t* out_lsb_first)
  {
    if (out_lsb_first != nullptr && bits != 0u)
    {
      const uint32_t BYTES = (bits + 7u) / 8u;
      Memory::FastSet(out_lsb_first, 0, BYTES);
    }

    if (bits == 0u)
    {
      return;
    }

    if (bits > 1u)
    {
      (void)Sequence(bits - 1u, false, in_lsb_first, out_lsb_first);
    }

    const uint32_t LAST = bits - 1u;
    const bool LAST_TDI = (in_lsb_first == nullptr)
                              ? false
                              : (((in_lsb_first[LAST / 8u] >> (LAST & 7u)) & 0x1u) != 0u);
    bool last_tdo = false;
    ClockCycle(true, LAST_TDI, out_lsb_first ? &last_tdo : nullptr);

    if (out_lsb_first != nullptr && last_tdo)
    {
      out_lsb_first[LAST / 8u] =
          static_cast<uint8_t>(out_lsb_first[LAST / 8u] | (1u << (LAST & 7u)));
    }
  }

  /**
   * @brief 一个 TCK 周期 / One TCK period
   *
   * 先设置 TMS 和 TDI，TCK 先低后高；TDO 在 TCK 回到低电平之前采样，目标在下降沿才更新
   * TDO。周期结束时 TCK 为低。
   * TMS and TDI are set first, then TCK goes low and high; TDO is sampled before TCK
   * returns low, since the target updates TDO on the falling edge. TCK ends low.
   *
   * @param tms 本周期的 TMS 电平 / TMS level of this period.
   * @param tdi 本周期的 TDI 电平 / TDI level of this period.
   * @param tdo 不为空时存放采到的 TDO / When not null, receives the sampled TDO.
   */
  void ClockCycle(bool tms, bool tdi, bool* tdo)
  {
    tms_.Write(tms);
    tdi_.Write(tdi);

    if (half_period_loops_ == 0u)
    {
      tck_.Write(false);
      tck_.Write(true);
      if (tdo != nullptr)
      {
        *tdo = tdo_.Read();
      }
      tck_.Write(false);
    }
    else
    {
      tck_.Write(false);
      DelayHalf();
      tck_.Write(true);
      DelayHalf();
      if (tdo != nullptr)
      {
        *tdo = tdo_.Read();
      }
      tck_.Write(false);
    }
  }

  /// 忙等半个 TCK 周期 / Busy-wait for half a TCK period.
  inline void DelayHalf() { Timebase::BusyLoop(half_period_loops_); }

 private:
  TckGpioType& tck_;  ///< TCK GPIO / GPIO for TCK.
  TmsGpioType& tms_;  ///< TMS GPIO / GPIO for TMS.
  TdiGpioType& tdi_;  ///< TDI GPIO / GPIO for TDI.
  TdoGpioType& tdo_;  ///< TDO GPIO / GPIO for TDO.

  uint32_t clock_hz_ = 0u;  ///< 当前 TCK 频率（Hz） / Current TCK frequency (Hz).
  /// 每微秒的延时循环次数 / Delay-loop iterations per microsecond.
  uint32_t loops_per_us_ = 0u;
  uint32_t half_period_ns_ = 0u;  ///< 半周期（ns） / Half period (ns).
  /// 每个半周期的延时循环次数，0 表示不加延时
  /// Delay loops per half period; 0 adds no delay.
  uint32_t half_period_loops_ = 0u;

  /// 本类记录的 TAP 状态 / TAP state tracked by this class.
  TapState current_state_ = TapState::TEST_LOGIC_RESET;
};

}  // namespace LibXR::Debug
