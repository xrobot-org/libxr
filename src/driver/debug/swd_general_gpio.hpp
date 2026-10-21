#pragma once

#include <atomic>
#include <cmath>
#include <cstdint>

#include "gpio.hpp"
#include "libxr_def.hpp"
#include "libxr_mem.hpp"
#include "swd.hpp"
#include "timebase.hpp"

namespace LibXR::Debug
{
enum class SwdIoDriveMode : uint8_t
{
  PUSH_PULL = 0,  ///< Drive phase uses push-pull output.
  OPEN_DRAIN = 1  ///< Drive phase uses open-drain output.
};

/**
 * @brief SwdGeneralGPIO 的时序参数，单位是 CPU 周期的 1/10，与 CPU 主频无关。
 *        Timing parameters of SwdGeneralGPIO, in tenths of a CPU cycle, independent
 *        of the CPU clock.
 *
 * 带延时的路径上，一个 SWCLK 周期耗时 a + b·N 个 CPU 周期，N 是每个半周期的延时循环
 * 次数（一个周期有两个半周期，所以 b 是两次延时循环的耗时）；无延时路径每个 SWCLK
 * 周期耗时 c 个 CPU 周期。
 * On the delayed path one SWCLK period costs a + b·N CPU cycles, where N is the
 * delay-loop count per half period (a period has two half periods, so b is the cost
 * of two delay iterations); the path without delay costs c CPU cycles per period.
 *
 * 标定（DWT 周期计数）：固定几个 N，用 DWT_CYCCNT 分别测读数据相、写数据相和请求
 * 字节每个 SWCLK 周期的 CPU 周期数，按 a + b·N 对各路径一起拟合，得到 a 和 b 的典型
 * 值；c 是 N = 0 时各路径每周期周期数的典型值。SetClockHz 只在无延时路径不超速时
 * 用它，否则在 N >= 1 中取周期数最接近请求周期的档位，所以实际时钟与请求值相差不超过
 * 半档（b / 2），可能略快也可能略慢。
 * Calibration (DWT cycle counter): fix a few N and measure the CPU cycles per SWCLK
 * period of the read data phase, the write data phase and the request byte with
 * DWT_CYCCNT; fit a + b·N over all paths together for the typical a and b, and take c
 * as the typical cycles per period of the paths at N = 0. SetClockHz takes the path
 * without delay only when it does not overshoot; otherwise it picks, among N >= 1, the
 * step whose cycles per period are closest to the requested period, so the clock is
 * within half a step (b / 2) of the request, slightly faster or slightly slower.
 *
 * 默认值是保守值（每次延时循环和每个周期 1 个 CPU 周期，截距 0）：未标定的平台上
 * 实际时钟通常明显慢于请求值。
 * The defaults are conservative (one CPU cycle per delay iteration and per period,
 * zero intercept): on an uncalibrated platform the clock is usually much slower than
 * requested.
 */
struct SwdGpioTiming
{
  uint32_t step_cycles_x10 =
      20u;  ///< b：每个 N 的周期数 ×10 / b: cycles per step of N x10.
  uint32_t intercept_cycles_x10 = 0u;  ///< a：截距 ×10 / a: intercept x10.
  uint32_t fast_cycles_x10 =
      10u;  ///< c：无延时路径每周期周期数 ×10 / c: cycles per period without delay x10.
};

/**
 * @brief 基于 GpioType 轮询 bit-bang 的 SWD 探针。
 *        SWD probe based on polling bit-bang using GpioType.
 *
 * @tparam SwclkGpioType SWCLK GPIO 类型 / SWCLK GPIO type
 * @tparam SwdioGpioType SWDIO GPIO 类型 / SWDIO GPIO type
 *
 * @note 推荐外围电路：SWCLK/SWDIO 均串联 33Ω 限流电阻，SWDIO 端接 10k 上拉电阻。
 *       Recommended circuit: 33Ω series resistors on SWCLK/SWDIO, 10k pull-up on SWDIO.
 * @note 裸线（没有串联电阻）上默认的 GPIO 驱动强度会产生振铃，
 *       引起 SWD 错误；驱动强度由 BSP 设置，本类不改。
 *       快速 bit-bang 时还要注意：有些芯片上 SWDIO 改成输出后
 *       要等若干个 CPU 周期才生效（外设寄存器经较慢的总线写入），
 *       GpioType 的 SetConfig 需要把这段时间算进去。
 *       On bare wires (no series resistors) the default GPIO drive strength
 *       rings and causes SWD errors; the drive strength is set by the BSP and
 *       not changed here. For fast bit-banging also note that on some chips
 *       SWDIO takes a number of CPU cycles to become an output after the change
 *       (the peripheral register is written over a slower bus), and the
 *       GpioType's SetConfig has to account for it.
 */
template <typename SwclkGpioType, typename SwdioGpioType,
          SwdIoDriveMode IO_DRIVE_MODE = SwdIoDriveMode::PUSH_PULL>
class SwdGeneralGPIO final : public Swd
{
  static constexpr uint32_t MIN_HZ = 50'000u;
  static constexpr uint32_t MAX_HZ = 100'000'000u;

  static_assert(MIN_HZ > 0u);
  static_assert(MAX_HZ >= MIN_HZ);

 public:
  /**
   * @brief 构造函数 / Constructor.
   * @param swclk 用作 SWCLK 的 GPIO / GPIO used as SWCLK.
   * @param swdio 用作 SWDIO 的 GPIO / GPIO used as SWDIO.
   * @param cpu_hz 执行本类的 CPU 主频（Hz）；0 表示始终走无延时路径 /
   *        Clock of the CPU that runs this class (Hz); 0 always takes the path
   *        without delay.
   * @param timing 本平台标定的时序参数（见 SwdGpioTiming） / Timing parameters
   *        calibrated for this platform (see SwdGpioTiming).
   * @param default_hz 默认 SWCLK 频率（Hz） / Default SWCLK frequency (Hz).
   */
  explicit SwdGeneralGPIO(SwclkGpioType& swclk, SwdioGpioType& swdio, uint32_t cpu_hz,
                          const SwdGpioTiming& timing,
                          uint32_t default_hz = DEFAULT_CLOCK_HZ)
      : swclk_(swclk), swdio_(swdio), cpu_hz_(cpu_hz), timing_(timing)
  {
    // SWCLK baseline
    swclk_.SetConfig(
        {SwclkGpioType::Direction::OUTPUT_PUSH_PULL, SwclkGpioType::Pull::NONE});
    swclk_.Write(true);

    // SWDIO baseline
    (void)SetSwdioDriveMode();
    swdio_.Write(true);

    (void)SetClockHz(default_hz);
  }

  ~SwdGeneralGPIO() override = default;

  SwdGeneralGPIO(const SwdGeneralGPIO&) = delete;
  SwdGeneralGPIO& operator=(const SwdGeneralGPIO&) = delete;

  ErrorCode SetClockHz(uint32_t hz) override
  {
    if (hz == 0u)
    {
      clock_hz_ = 0u;
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
    half_period_loops_ = DelayLoops(cpu_hz_, hz, timing_);
    return ErrorCode::OK;
  }

  /**
   * @brief 每个半周期的延时循环次数 / Delay-loop count per half period.
   *
   * f >= f_cpu / c 时返回 0：无延时路径不会快于请求值。否则在 N >= 1（每周期 a + b·N）
   * 中取每周期周期数最接近 f_cpu / f 的一档，两档一样近时取较慢的一档。无延时路径与
   * N = 1 之间的间隔可能大于一档，所以只在它不超速时才用它。只用整数运算，不引入浮点库。
   * Returns 0 when f >= f_cpu / c: the path without delay is then never faster than
   * requested. Otherwise, among N >= 1 (a + b·N per period), the one whose cycles per
   * period are closest to f_cpu / f wins, and of two equally close ones the slower. The
   * gap between the path without delay and N = 1 can be wider than a step, so that path
   * is used only when it does not overshoot. Integer-only, so no floating-point helpers
   * are pulled in.
   *
   * @param cpu_hz CPU 主频（Hz），0 时返回 0 / CPU clock (Hz); 0 returns 0.
   * @param hz 请求的 SWCLK 频率（Hz） / Requested SWCLK frequency (Hz).
   * @param timing 时序参数 / Timing parameters.
   * @return 延时循环次数 / Delay-loop count.
   */
  static constexpr uint32_t DelayLoops(uint32_t cpu_hz, uint32_t hz,
                                       const SwdGpioTiming& timing)
  {
    if (cpu_hz == 0u || hz == 0u)
    {
      return 0u;
    }
    // 都乘以 hz 比较，避免周期取整。误差 = |每周期周期数 × hz − f_cpu|，单位 1/10 周期。
    // Compare after multiplying by hz, so the period is never rounded. The error is
    // |cycles per period x hz - f_cpu|, in tenths of a cycle.
    const uint64_t TARGET = static_cast<uint64_t>(cpu_hz) * 10u;
    const uint64_t REQ_HZ = hz;
    if (static_cast<uint64_t>(timing.fast_cycles_x10) * REQ_HZ >= TARGET)
    {
      return 0u;
    }

    const uint64_t A = timing.intercept_cycles_x10;
    const uint64_t B = (timing.step_cycles_x10 == 0u) ? 1u : timing.step_cycles_x10;

    // 最接近的 N 是 floor((目标 − a) / b) 或它加 1（至少为 1）。
    // The closest N is floor((target - a) / b) or one more (at least 1).
    uint64_t lo = 1u;
    if (TARGET > A * REQ_HZ)
    {
      lo = (TARGET - A * REQ_HZ) / (B * REQ_HZ);
      if (lo < 1u)
      {
        lo = 1u;
      }
    }

    uint64_t best_n = lo;
    const uint64_t LO_ERR = AbsDiff((A + B * lo) * REQ_HZ, TARGET);
    const uint64_t NEXT_ERR = AbsDiff((A + B * (lo + 1u)) * REQ_HZ, TARGET);
    // 一样近时取较慢的一档 / On a tie take the slower step.
    if (NEXT_ERR <= LO_ERR)
    {
      best_n = lo + 1u;
    }
    return (best_n >= UINT32_MAX) ? UINT32_MAX : static_cast<uint32_t>(best_n);
  }

  void Close() override
  {
    InvalidateSelectCache();

    // 安全状态：Safe state:
    // - SWCLK 高电平（历史行为） / SWCLK high (legacy).
    // - SWDIO 上拉输入（不驱动） / SWDIO input with pull-up (no drive).
    swclk_.Write(true);
    (void)SetSwdioSampleMode();
  }

  ErrorCode LineReset() override
  {
    InvalidateSelectCache();
    if (Delayed())
    {
      LineResetImpl<true>();
    }
    else
    {
      LineResetImpl<false>();
    }
    return ErrorCode::OK;
  }

  ErrorCode EnterSwd() override
  {
    InvalidateSelectCache();
    if (Delayed())
    {
      EnterSwdImpl<true>();
    }
    else
    {
      EnterSwdImpl<false>();
    }
    return ErrorCode::OK;
  }

  ErrorCode Transfer(const SwdProtocol::Request& req,
                     SwdProtocol::Response& resp) override
  {
    // 每个入口只选一次路径，位级代码里不再判断延时次数是否为 0。
    // Each entry point picks the path once; the bit-level code no longer checks whether
    // the delay count is zero.
    return Delayed() ? TransferImpl<true>(req, resp) : TransferImpl<false>(req, resp);
  }

  void IdleClocks(uint32_t cycles) override
  {
    if (Delayed())
    {
      IdleClocksImpl<true>(cycles);
    }
    else
    {
      IdleClocksImpl<false>(cycles);
    }
  }

  ErrorCode SeqWriteBits(uint32_t cycles, const uint8_t* data_lsb_first) override
  {
    if (cycles == 0u)
    {
      swclk_.Write(false);
      return ErrorCode::OK;
    }
    if (data_lsb_first == nullptr)
    {
      return ErrorCode::ARG_ERR;
    }

    if (Delayed())
    {
      SeqWriteBitsImpl<true>(cycles, data_lsb_first);
    }
    else
    {
      SeqWriteBitsImpl<false>(cycles, data_lsb_first);
    }
    return ErrorCode::OK;
  }

  ErrorCode SeqReadBits(uint32_t cycles, uint8_t* out_lsb_first) override
  {
    if (cycles == 0u)
    {
      swclk_.Write(false);
      return ErrorCode::OK;
    }
    if (out_lsb_first == nullptr)
    {
      return ErrorCode::ARG_ERR;
    }

    const uint32_t BYTES = (cycles + 7u) / 8u;
    Memory::FastSet(out_lsb_first, 0, BYTES);

    if (Delayed())
    {
      SeqReadBitsImpl<true>(cycles, out_lsb_first);
    }
    else
    {
      SeqReadBitsImpl<false>(cycles, out_lsb_first);
    }
    return ErrorCode::OK;
  }

 private:
  static constexpr uint64_t AbsDiff(uint64_t x, uint64_t y)
  {
    return x > y ? x - y : y - x;
  }

  static inline uint8_t Parity32(uint32_t x)
  {
    x ^= x >> 16;
    x ^= x >> 8;
    x ^= x >> 4;
    x &= 0xFu;
    static constexpr uint8_t LUT[16] = {
        0, 1, 1, 0, 1, 0, 0, 1,
        1, 0, 0, 1, 0, 1, 1, 0};  ///< 奇偶校验查找表（4-bit 折叠） / Parity lookup table
                                  ///< (4-bit fold).
    return LUT[x];
  }

 private:
  /**
   * @enum SwdioMode
   * @brief SWDIO 管脚当前模式 / Current SWDIO pin mode.
   */
  enum class SwdioMode : uint8_t
  {
    UNKNOWN = 0,  ///< 未知/未初始化 / Unknown / uninitialized.
    DRIVE,        ///< 输出驱动阶段 / Drive phase.
    SAMPLE_IN,    ///< 输入采样阶段 / Sample phase.
  };

  ErrorCode SetSwdioDriveMode()
  {
    if (swdio_mode_ != SwdioMode::DRIVE)
    {
      const ErrorCode EC =
          swdio_.SetConfig({(IO_DRIVE_MODE == SwdIoDriveMode::OPEN_DRAIN)
                                ? SwdioGpioType::Direction::OUTPUT_OPEN_DRAIN
                                : SwdioGpioType::Direction::OUTPUT_PUSH_PULL,
                            SwdioGpioType::Pull::NONE});
      if (EC != ErrorCode::OK)
      {
        return EC;
      }
    }

    swdio_mode_ = SwdioMode::DRIVE;
    return ErrorCode::OK;
  }

  ErrorCode SetSwdioSampleMode()
  {
    if (swdio_mode_ != SwdioMode::SAMPLE_IN)
    {
      // 采样阶段统一把 SWDIO 置为上拉输入，这样目标开始驱动 ACK/数据之前，
      // 总线空闲电平始终有明确定义。
      // Sampling should always leave SWDIO as a pulled-up input so the line has a
      // defined idle level before the target actively drives ACK/data.
      const ErrorCode EC =
          swdio_.SetConfig({SwdioGpioType::Direction::INPUT, SwdioGpioType::Pull::UP});
      if (EC != ErrorCode::OK)
      {
        return EC;
      }
    }

    swdio_mode_ = SwdioMode::SAMPLE_IN;
    return ErrorCode::OK;
  }

  /// SetClockHz 只在无延时路径时把延时次数设为 0 / SetClockHz sets the delay count to
  /// 0 only for the path without delay.
  bool Delayed() const { return half_period_loops_ != 0u; }

  // 位级辅助函数以 kDelay 区分有无延时，同一份源码生成两条路径；强制内联：-Os 下编译器
  // 不内联它们，每一位都变成一次函数调用，在带 Flash 等待周期的 MCU 上 SWD 位速率下降
  // 一半以上。
  // The bit-level helpers take kDelay, so one source yields both paths. They are forced
  // inline: under -Os the compiler keeps them out of line, every bit becomes a call, and
  // on MCUs with flash wait states the SWD bit rate drops by more than half.
  template <bool kDelay>
  [[gnu::always_inline]] inline void DelayHalf()
  {
    if constexpr (kDelay)
    {
      BusyLoop(half_period_loops_);
    }
  }

  template <bool kDelay>
  [[gnu::always_inline]] inline void GenOneClk()
  {
    swclk_.Write(false);
    DelayHalf<kDelay>();
    swclk_.Write(true);
    DelayHalf<kDelay>();
  }

  template <bool kDelay>
  [[gnu::always_inline]] inline void WriteBit(bool bit)
  {
    swdio_.Write(bit);
    GenOneClk<kDelay>();
  }

  // 按字节收发的函数都展开成 8 位，不留逐位循环：循环的跳转目标落在哪里随代码布局变化，
  // 在取指要等待的 Flash 上（例如 STM32F1 的 64 位预取行），跨行的跳转目标会让每一位多等
  // 几个周期，同一份源码换个位置就快慢不同。
  // The byte functions are unrolled into 8 bits with no per-bit loop: where a loop's
  // branch target lands moves with the code layout, and on flash with fetch wait states
  // (for example the 64-bit prefetch lines of the STM32F1) a target straddling a line
  // costs every bit a few cycles, so the same source runs at different speeds depending
  // on where it is placed.
  template <bool kDelay>
  inline void WriteByteLSB(uint8_t b)
  {
    WriteBit<kDelay>((b & 0x01u) != 0u);
    WriteBit<kDelay>((b & 0x02u) != 0u);
    WriteBit<kDelay>((b & 0x04u) != 0u);
    WriteBit<kDelay>((b & 0x08u) != 0u);
    WriteBit<kDelay>((b & 0x10u) != 0u);
    WriteBit<kDelay>((b & 0x20u) != 0u);
    WriteBit<kDelay>((b & 0x40u) != 0u);
    WriteBit<kDelay>((b & 0x80u) != 0u);
  }

  template <bool kDelay>
  [[gnu::always_inline]] inline bool ReadBitAndClock()
  {
    swclk_.Write(false);
    DelayHalf<kDelay>();
    const bool BIT = swdio_.Read();
    swclk_.Write(true);
    DelayHalf<kDelay>();
    return BIT;
  }

  template <bool kDelay>
  inline uint8_t ReadByteLSB()
  {
    uint8_t v = 0u;
    v |= static_cast<uint8_t>(ReadBitAndClock<kDelay>() ? 0x01u : 0u);
    v |= static_cast<uint8_t>(ReadBitAndClock<kDelay>() ? 0x02u : 0u);
    v |= static_cast<uint8_t>(ReadBitAndClock<kDelay>() ? 0x04u : 0u);
    v |= static_cast<uint8_t>(ReadBitAndClock<kDelay>() ? 0x08u : 0u);
    v |= static_cast<uint8_t>(ReadBitAndClock<kDelay>() ? 0x10u : 0u);
    v |= static_cast<uint8_t>(ReadBitAndClock<kDelay>() ? 0x20u : 0u);
    v |= static_cast<uint8_t>(ReadBitAndClock<kDelay>() ? 0x40u : 0u);
    v |= static_cast<uint8_t>(ReadBitAndClock<kDelay>() ? 0x80u : 0u);
    return v;
  }

  /**
   * @brief 忙等 loops 次延时循环 / Busy-wait for loops delay iterations.
   * @param loops 循环次数，必须 >= 1：只有带延时的路径调用它，SetClockHz 只在无延时
   *        路径时产生 0 / Iteration count, must be >= 1: only the delayed path calls it,
   *        and SetClockHz produces 0 only for the path without delay.
   */
  [[gnu::always_inline]] static inline void BusyLoop(uint32_t loops)
  {
    DEV_ASSERT(loops >= 1u);
#if defined(__thumb2__)
    // Thumb-2：两条 16 位指令的循环，.balign 8 让它落在同一个 64 位取指行里，含它的函数段
    // 也因此按 8 字节对齐，每次迭代的周期数不再随代码布局变化（编译器生成的循环可能跨行，
    // 每次迭代多等取指周期）。对齐填充的 nop 只在进入循环前执行一次。
    // Thumb-2: a loop of two 16-bit instructions; .balign 8 keeps it inside one 64-bit
    // fetch line and makes the section of the containing function 8-byte aligned, so
    // the cycles per iteration no longer depend on code layout (a compiler-generated loop
    // may straddle a line and wait for an extra fetch on every iteration). The alignment
    // nops run once, before the loop.
    asm volatile(
        ".balign 8\n"
        "1:\n\t"
        "subs %0, %0, #1\n\t"
        "bne 1b\n"
        : "+l"(loops)
        :
        : "cc");
#else
    // 计数器放在寄存器里；atomic_signal_fence 只约束编译器、不产生指令，使空循环不会被
    // 优化掉。volatile 计数器会在每次迭代读写栈并带内存屏障，每次迭代更慢。
    // The counter lives in a register; atomic_signal_fence constrains only the compiler
    // and emits no instruction, so the empty loop is not optimised away. A volatile
    // counter would read and write the stack with a memory barrier on every iteration,
    // which makes each iteration slower.
    while (loops--)
    {
      std::atomic_signal_fence(std::memory_order_seq_cst);
    }
#endif
  }

  template <bool kDelay>
  void LineResetImpl()
  {
    // SWD 线复位：SWDIO = 1 持续 >= 50 个周期；此处使用 64 个周期。
    // SWD line reset: SWDIO = 1 for >= 50 cycles; here use 64 cycles.
    (void)SetSwdioDriveMode();
    swdio_.Write(true);
    for (uint32_t i = 0; i < LINE_RESET_CYCLES; ++i)
    {
      GenOneClk<kDelay>();
    }
  }

  template <bool kDelay>
  void EnterSwdImpl()
  {
    LineResetImpl<kDelay>();
    WriteByteLSB<kDelay>(JTAG_TO_SWD_SEQ0);
    WriteByteLSB<kDelay>(JTAG_TO_SWD_SEQ1);
    LineResetImpl<kDelay>();
    swdio_.Write(false);
    WriteByteLSB<kDelay>(0x00);
  }

  template <bool kDelay>
  void IdleClocksImpl(uint32_t cycles)
  {
    // CMSIS-DAP 空闲周期插入 / CMSIS-DAP idle cycles insertion.
    // 空闲周期里 SWDIO 驱动为低：高电平会被目标当成起始位，下一个请求就对不齐。
    // SWDIO is driven low during the idle cycles: a high level is taken by the target as
    // a start bit and the next request would be misaligned.
    (void)SetSwdioDriveMode();
    swdio_.Write(false);

    for (uint32_t i = 0; i < cycles; ++i)
    {
      GenOneClk<kDelay>();
    }

    swdio_.Write(false);
  }

  template <bool kDelay>
  void SeqWriteBitsImpl(uint32_t cycles, const uint8_t* data_lsb_first)
  {
    (void)SetSwdioDriveMode();

    // Keep legacy: start from SWCLK low and end low
    swclk_.Write(false);

    for (uint32_t i = 0; i < cycles; ++i)
    {
      const bool BIT = (((data_lsb_first[i / 8u] >> (i & 7u)) & 0x01u) != 0u);
      swdio_.Write(BIT);

      // 匹配 DAP_Transfer 请求路径：每个 bit 只产生一个完整低-高时钟周期，
      // 不在每 bit 末尾额外插入无定时低脉冲。
      // Match the DAP_Transfer request path: one bit owns one complete
      // low-high clock cycle. Do not add an un-timed low pulse after every bit.
      GenOneClk<kDelay>();
    }
    swclk_.Write(false);
  }

  template <bool kDelay>
  void SeqReadBitsImpl(uint32_t cycles, uint8_t* out_lsb_first)
  {
    (void)SetSwdioSampleMode();

    // start from low, end low
    swclk_.Write(false);

    for (uint32_t i = 0; i < cycles; ++i)
    {
      // Use the updated read phase (CMSIS-style)
      if (ReadBitAndClock<kDelay>())
      {
        out_lsb_first[i / 8u] =
            static_cast<uint8_t>(out_lsb_first[i / 8u] | (1u << (i & 7u)));
      }

      // keep legacy end-low for next bit
      swclk_.Write(false);
    }
  }

  template <bool kDelay>
  ErrorCode TransferImpl(const SwdProtocol::Request& req, SwdProtocol::Response& resp)
  {
    resp.ack = SwdProtocol::Ack::PROTOCOL;
    resp.rdata = 0u;
    resp.parity_ok = true;

    const uint8_t REQUEST_BYTE = SwdProtocol::MakeRequest(req);

    (void)SetSwdioDriveMode();
    WriteByteLSB<kDelay>(REQUEST_BYTE);

    (void)SetSwdioSampleMode();
    GenOneClk<kDelay>();  // turnaround Host -> Target

    // ACK: CMSIS SW_READ_BIT phase (sample in low phase)
    uint8_t ack_raw = 0u;
    for (uint32_t i = 0; i < ACK_BITS; ++i)
    {
      if (ReadBitAndClock<kDelay>())
      {
        ack_raw |= static_cast<uint8_t>(1u << i);
      }
    }
    resp.ack = SwdProtocol::DecodeAck(static_cast<uint8_t>(ack_raw & 0x7u));

    if (resp.ack != SwdProtocol::Ack::OK)
    {
      GenOneClk<kDelay>();  // turnaround Target -> Host (skip data)
      (void)SetSwdioDriveMode();
      swdio_.Write(true);
      swclk_.Write(false);
      return ErrorCode::OK;
    }

    if (req.rnw)
    {
      uint32_t data = 0u;
      for (uint32_t byte = 0; byte < 4u; ++byte)
      {
        const uint32_t B = ReadByteLSB<kDelay>();
        data |= (B << (8u * byte));
      }

      const bool PARITY_BIT = ReadBitAndClock<kDelay>();
      resp.rdata = data;
      resp.parity_ok = (static_cast<uint8_t>(PARITY_BIT) == Parity32(data));

      (void)SetSwdioDriveMode();
      swdio_.Write(true);
      GenOneClk<kDelay>();

      swclk_.Write(false);
    }
    else
    {
      (void)SetSwdioDriveMode();
      GenOneClk<kDelay>();

      const uint32_t DATA = req.wdata;
      for (uint32_t byte = 0; byte < 4u; ++byte)
      {
        const uint8_t B = static_cast<uint8_t>((DATA >> (8u * byte)) & 0xFFu);
        WriteByteLSB<kDelay>(B);
      }

      const bool PARITY_BIT = (Parity32(DATA) & 0x1u) != 0u;
      WriteBit<kDelay>(PARITY_BIT);

      // 写数据相之后 SWDIO 保持低电平再补几个时钟，让目标完成（可能是 posted 的）写入，
      // 否则同一批次里紧随 AP 写的下一个请求会在错误的相位采到 ACK。
      // After a write data phase SWDIO stays low for a few more clocks so the target can
      // complete a (possibly posted) write; otherwise the request that follows an AP
      // write in the same batch samples its ACK at the wrong phase.
      swdio_.Write(false);
      for (uint32_t k = 0; k < WRITE_TRAILING_IDLE_CLOCKS; ++k)
      {
        GenOneClk<kDelay>();
      }

      swdio_.Write(true);
      swclk_.Write(false);
    }

    return ErrorCode::OK;
  }

 private:
  static constexpr uint32_t DEFAULT_CLOCK_HZ =
      500'000u;  ///< 默认 SWCLK 频率（Hz） / Default SWCLK frequency (Hz).
  static constexpr uint32_t LINE_RESET_CYCLES =
      64u;  ///< 线复位时钟周期数 / Line reset clock cycles.
  static constexpr uint32_t BYTE_BITS = 8u;  ///< 每字节比特数 / Bits per byte.
  static constexpr uint32_t ACK_BITS = 3u;   ///< ACK 比特数 / ACK bits.
  static constexpr uint32_t WRITE_TRAILING_IDLE_CLOCKS =
      3u;  ///< 写数据相后的空闲时钟数 / Idle clocks after a write data phase.

  static constexpr uint8_t JTAG_TO_SWD_SEQ0 =
      0x9Eu;  ///< JTAG->SWD 序列字节 0 / JTAG-to-SWD sequence byte 0.
  static constexpr uint8_t JTAG_TO_SWD_SEQ1 =
      0xE7u;  ///< JTAG->SWD 序列字节 1 / JTAG-to-SWD sequence byte 1.

  SwclkGpioType& swclk_;  ///< SWCLK GPIO / GPIO for SWCLK.
  SwdioGpioType& swdio_;  ///< SWDIO GPIO / GPIO for SWDIO.

  uint32_t clock_hz_ = 0u;  ///< 当前 SWCLK 频率（Hz） / Current SWCLK frequency (Hz).

  uint32_t cpu_hz_ = 0u;  ///< CPU 主频（Hz） / CPU clock (Hz).
  SwdGpioTiming timing_;  ///< 时序参数 / Timing parameters.
  uint32_t half_period_loops_ =
      0u;  ///< 每个半周期的延时循环次数 / Delay loops per half period.

  SwdioMode swdio_mode_ =
      SwdioMode::UNKNOWN;  ///< SWDIO 当前模式缓存 / Cached current SWDIO mode.
};

}  // namespace LibXR::Debug
