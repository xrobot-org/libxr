#pragma once

#include <cstdint>

#include "gpio.hpp"
#include "jtag.hpp"
#include "libxr_def.hpp"
#include "libxr_mem.hpp"

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
  static constexpr uint32_t MIN_HZ = 10'000u;
  static constexpr uint32_t MAX_HZ = 100'000'000u;

  static constexpr uint32_t NS_PER_SEC = 1'000'000'000u;
  static constexpr uint32_t LOOPS_SCALE = 1000u;
  static constexpr uint32_t CEIL_BIAS = LOOPS_SCALE - 1u;

  static constexpr uint32_t DEFAULT_CLOCK_HZ = 500'000u;
  static constexpr uint32_t RESET_CYCLES = 5u;

  static constexpr uint32_t HALF_PERIOD_NS_MAX =
      (NS_PER_SEC + (2u * MIN_HZ) - 1u) / (2u * MIN_HZ);
  static constexpr uint32_t MAX_LOOPS_PER_US =
      (UINT32_MAX - CEIL_BIAS) / HALF_PERIOD_NS_MAX;

 public:
  /**
   * @brief 构造函数 / Constructor.
   * @param tck 用作 TCK 的 GPIO / GPIO used as TCK.
   * @param tms 用作 TMS 的 GPIO / GPIO used as TMS.
   * @param tdi 用作 TDI 的 GPIO / GPIO used as TDI.
   * @param tdo 用作 TDO 的 GPIO / GPIO used as TDO.
   * @param loops_per_us 每个 us 的循延时环次数 / Loops per us of delay.
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

  void Close() override
  {
    tck_.Write(false);
    tms_.Write(true);
    tdi_.Write(false);
  }

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
  static constexpr uint8_t SEQ_TO_SHIFT_DR[] = {1u, 0u, 0u};
  static constexpr uint32_t SEQ_TO_SHIFT_DR_LEN = 3u;
  static constexpr uint8_t SEQ_TO_SHIFT_IR[] = {1u, 1u, 0u, 0u};
  static constexpr uint32_t SEQ_TO_SHIFT_IR_LEN = 4u;

  void ApplyTmsSequence(const uint8_t* seq, uint32_t len)
  {
    for (uint32_t i = 0; i < len; ++i)
    {
      const bool TMS_VAL = (seq[i] != 0u);
      ClockCycle(TMS_VAL, false, nullptr);
    }
  }

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
  void ExitToIdle()
  {
    // EXIT1 -> UPDATE (TMS=1), UPDATE -> IDLE (TMS=0)
    ClockCycle(true, false, nullptr);
    ClockCycle(false, false, nullptr);
    current_state_ = TapState::RUN_TEST_IDLE;
  }

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

  inline void DelayHalf() { BusyLoop(half_period_loops_); }

  static void BusyLoop(uint32_t loops)
  {
    volatile uint32_t sink = loops;
    while (sink--)
    {
    }
  }

 private:
  TckGpioType& tck_;
  TmsGpioType& tms_;
  TdiGpioType& tdi_;
  TdoGpioType& tdo_;

  uint32_t clock_hz_ = 0u;
  uint32_t loops_per_us_ = 0u;
  uint32_t half_period_ns_ = 0u;
  uint32_t half_period_loops_ = 0u;

  TapState current_state_ = TapState::TEST_LOGIC_RESET;
};

}  // namespace LibXR::Debug
