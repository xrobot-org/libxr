#pragma once

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
 * @brief 基于 GpioType 轮询 bit-bang 的 SWD 探针。
 *        SWD probe based on polling bit-bang using GpioType.
 *
 * @tparam SwclkGpioType SWCLK GPIO 类型 / SWCLK GPIO type
 * @tparam SwdioGpioType SWDIO GPIO 类型 / SWDIO GPIO type
 *
 * @note 推荐外围电路：SWCLK/SWDIO 均串联 33Ω 限流电阻，SWDIO 端接 10k 上拉电阻。
 *       Recommended circuit: 33Ω series resistors on SWCLK/SWDIO, 10k pull-up on SWDIO.
 */
template <typename SwclkGpioType, typename SwdioGpioType,
          SwdIoDriveMode IO_DRIVE_MODE = SwdIoDriveMode::PUSH_PULL>
class SwdGeneralGPIO final : public Swd
{
  static constexpr uint32_t MIN_HZ = 50'000u;
  static constexpr uint32_t MAX_HZ = 100'000'000u;

  static constexpr uint32_t NS_PER_SEC = 1'000'000'000u;
  static constexpr uint32_t LOOPS_SCALE = 1000u;           // ns -> us 的缩放分母
  static constexpr uint32_t CEIL_BIAS = LOOPS_SCALE - 1u;  // ceil(x/LOOPS_SCALE) 的偏置

  static constexpr uint32_t HalfPeriodNsFromHz(uint32_t hz)
  {
    // ceil(1e9 / (2*hz))
    return (NS_PER_SEC + (2u * hz) - 1u) / (2u * hz);
  }

  static constexpr uint32_t HALF_PERIOD_NS_MAX = HalfPeriodNsFromHz(MIN_HZ);
  static constexpr uint32_t MAX_LOOPS_PER_US =
      (UINT32_MAX - CEIL_BIAS) / HALF_PERIOD_NS_MAX;

  static_assert(MIN_HZ > 0u);
  static_assert(MAX_HZ >= MIN_HZ);
  static_assert(HALF_PERIOD_NS_MAX > 0u);

 public:
  /**
   * @brief 构造函数。Constructor.
   * @param swclk 用作 SWCLK 的 GPIO。GPIO used as SWCLK.
   * @param swdio 用作 SWDIO 的 GPIO。GPIO used as SWDIO.
   * @param loops_per_us 每个 us 的循延时环次数。Loops per us of delay.
   * @param default_hz 默认 SWCLK 频率（Hz）。Default SWCLK frequency (Hz).
   */
  explicit SwdGeneralGPIO(SwclkGpioType& swclk, SwdioGpioType& swdio,
                          uint32_t loops_per_us, uint32_t default_hz = DEFAULT_CLOCK_HZ)
      : swclk_(swclk), swdio_(swdio), loops_per_us_(loops_per_us)
  {
    if (loops_per_us_ > MAX_LOOPS_PER_US)
    {
      loops_per_us_ = MAX_LOOPS_PER_US;
    }

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

    // Keep MCU timing setup integer-only so simple SWD clock setup does not
    // pull in floating-point runtime helpers.
    const uint64_t DEN = static_cast<uint64_t>(2u) * static_cast<uint64_t>(hz);
    half_period_ns_ =
        static_cast<uint32_t>((static_cast<uint64_t>(NS_PER_SEC) + DEN - 1u) / DEN);

    if (loops_per_us_ == 0u)
    {
      half_period_loops_ = 0u;
      return ErrorCode::OK;
    }

    const uint64_t loops_num =
        static_cast<uint64_t>(loops_per_us_) * static_cast<uint64_t>(half_period_ns_);

    if (loops_num < static_cast<uint64_t>(LOOPS_SCALE))
    {
      half_period_loops_ = 0u;
    }
    else
    {
      const uint64_t loops_ceil = (loops_num + static_cast<uint64_t>(LOOPS_SCALE) - 1u) /
                                  static_cast<uint64_t>(LOOPS_SCALE);
      half_period_loops_ = (loops_ceil >= static_cast<uint64_t>(UINT32_MAX))
                               ? UINT32_MAX
                               : static_cast<uint32_t>(loops_ceil);
    }

    return ErrorCode::OK;
  }

  void Close() override
  {
    InvalidateSelectCache();

    // 安全状态：Safe state:
    // - SWCLK 高电平（历史行为）。SWCLK high (legacy).
    // - SWDIO 上拉输入（不驱动）。SWDIO input with pull-up (no drive).
    swclk_.Write(true);
    (void)SetSwdioSampleMode();
  }

  ErrorCode LineReset() override
  {
    InvalidateSelectCache();
    // SWD 线复位：SWDIO = 1 持续 >= 50 个周期；此处使用 64 个周期。SWD line reset: SWDIO
    // = 1 for >= 50 cycles; here use 64 cycles.
    (void)SetSwdioDriveMode();
    swdio_.Write(true);
    for (uint32_t i = 0; i < LINE_RESET_CYCLES; ++i)
    {
      GenOneClk();
    }
    return ErrorCode::OK;
  }

  ErrorCode EnterSwd() override
  {
    ErrorCode ec = LineReset();
    if (ec != ErrorCode::OK)
    {
      return ec;
    }

    WriteByteLSB(JTAG_TO_SWD_SEQ0);
    WriteByteLSB(JTAG_TO_SWD_SEQ1);

    ec = LineReset();
    if (ec != ErrorCode::OK)
    {
      return ec;
    }

    swdio_.Write(false);
    WriteByteLSB(0x00);

    return ErrorCode::OK;
  }

  ErrorCode Transfer(const SwdProtocol::Request& req,
                     SwdProtocol::Response& resp) override
  {
    // 目标：当 half_period_loops_ == 0 时，整次 Transfer 走无延时路径，
    // 避免每半周期 BusyLoop(0) 的空转判断开销。
    if (half_period_loops_ == 0u)
    {
      return TransferWithoutDelay(req, resp);
    }
    else
    {
      return TransferWithDelay(req, resp);
    }
  }

  void IdleClocks(uint32_t cycles) override
  {
    // CMSIS-DAP 空闲周期插入。CMSIS-DAP idle cycles insertion.
    // 保留历史序列：周期内驱动 SWDIO 高电平，结束后拉低。Keep the legacy sequence: drive
    // SWDIO high during cycles, then pull low.
    (void)SetSwdioDriveMode();
    swdio_.Write(true);

    for (uint32_t i = 0; i < cycles; ++i)
    {
      GenOneClk();
    }

    swdio_.Write(false);
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

    (void)SetSwdioDriveMode();

    // Keep legacy: start from SWCLK low and end low
    swclk_.Write(false);

    for (uint32_t i = 0; i < cycles; ++i)
    {
      const bool BIT = (((data_lsb_first[i / 8u] >> (i & 7u)) & 0x01u) != 0u);
      swdio_.Write(BIT);

      // Match the DAP_Transfer request path: one bit owns one complete
      // low-high clock cycle. Do not add an un-timed low pulse after every bit.
      // 匹配 DAP_Transfer 请求路径：每个 bit 只产生一个完整低-高时钟周期，
      // 不在每 bit 末尾额外插入无定时低脉冲。
      GenOneClk();
    }
    swclk_.Write(false);

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

    (void)SetSwdioSampleMode();

    // start from low, end low
    swclk_.Write(false);

    for (uint32_t i = 0; i < cycles; ++i)
    {
      // Use the updated read phase (CMSIS-style)
      bool bit = false;
      if (half_period_loops_ == 0u)
      {
        swclk_.Write(false);
        bit = swdio_.Read();
        swclk_.Write(true);
      }
      else
      {
        swclk_.Write(false);
        DelayHalf();
        bit = swdio_.Read();
        swclk_.Write(true);
        DelayHalf();
      }

      if (bit)
      {
        out_lsb_first[i / 8u] =
            static_cast<uint8_t>(out_lsb_first[i / 8u] | (1u << (i & 7u)));
      }

      // keep legacy end-low for next bit
      swclk_.Write(false);
    }

    return ErrorCode::OK;
  }

 private:
  static inline uint8_t MakeReq(bool apndp, bool rnw, uint8_t addr2b)
  {
    const uint8_t A2 = addr2b & 0x1u;
    const uint8_t A3 = (addr2b >> 1) & 0x1u;
    const uint8_t PAR = static_cast<uint8_t>((apndp ^ rnw ^ A2 ^ A3) & 0x1u);

    // 请求字节位域：start(1), APnDP, RnW, A2, A3, PAR, stop(0), park(1)。Request bit
    // fields: start(1), APnDP, RnW, A2, A3, PAR, stop(0), park(1).
    return static_cast<uint8_t>((1u << 0) | (static_cast<uint8_t>(apndp) << 1) |
                                (static_cast<uint8_t>(rnw) << 2) |
                                (static_cast<uint8_t>(A2) << 3) |
                                (static_cast<uint8_t>(A3) << 4) |
                                (static_cast<uint8_t>(PAR) << 5) | (0u << 6) | (1u << 7));
  }

  static inline uint8_t Parity32(uint32_t x)
  {
    x ^= x >> 16;
    x ^= x >> 8;
    x ^= x >> 4;
    x &= 0xFu;
    static constexpr uint8_t LUT[16] = {
        0, 1, 1, 0, 1, 0, 0, 1,
        1, 0, 0, 1, 0, 1, 1, 0};  ///< 奇偶校验查找表（4-bit 折叠）。Parity lookup table
                                  ///< (4-bit fold).
    return LUT[x];
  }

  static inline SwdProtocol::Ack DecodeAck(uint8_t ack_bits)
  {
    switch (ack_bits)
    {
      case 0x1:
        return SwdProtocol::Ack::OK;
      case 0x2:
        return SwdProtocol::Ack::WAIT;
      case 0x4:
        return SwdProtocol::Ack::FAULT;
      case 0x0:
        return SwdProtocol::Ack::NO_ACK;
      default:
        return SwdProtocol::Ack::PROTOCOL;
    }
  }

 private:
  /**
   * @enum SwdioMode
   * @brief SWDIO 管脚当前模式。Current SWDIO pin mode.
   */
  enum class SwdioMode : uint8_t
  {
    UNKNOWN = 0,  ///< 未知/未初始化。Unknown / uninitialized.
    DRIVE,        ///< 输出驱动阶段。Drive phase.
    SAMPLE_IN,    ///< 输入采样阶段。Sample phase.
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
      // Sampling should always leave SWDIO as a pulled-up input so the line has a
      // defined idle level before the target actively drives ACK/data.
      // 采样阶段统一把 SWDIO 置为上拉输入，这样目标开始驱动 ACK/数据之前，
      // 总线空闲电平始终有明确定义。
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

  inline void DelayHalf() { BusyLoop(half_period_loops_); }

  inline void GenOneClk()
  {
    swclk_.Write(false);
    DelayHalf();
    swclk_.Write(true);
    DelayHalf();
  }

  inline void GenOneClkWithoutDelay()
  {
    swclk_.Write(false);
    swclk_.Write(true);
  }

  inline void WriteBit(bool bit)
  {
    swdio_.Write(bit);
    GenOneClk();
  }

  inline void WriteBitWithoutDelay(bool bit)
  {
    swdio_.Write(bit);
    GenOneClkWithoutDelay();
  }

  inline void WriteByteLSB(uint8_t b)
  {
    for (uint32_t i = 0; i < BYTE_BITS; ++i)
    {
      WriteBit(((b >> i) & 0x1u) != 0u);
    }
  }

  inline void WriteByteLSBWithoutDelay(uint8_t b)
  {
    for (uint32_t i = 0; i < BYTE_BITS; ++i)
    {
      WriteBitWithoutDelay(((b >> i) & 0x1u) != 0u);
    }
  }

  inline bool ReadBitAndClock()
  {
    swclk_.Write(false);
    DelayHalf();
    const bool BIT = swdio_.Read();
    swclk_.Write(true);
    DelayHalf();
    return BIT;
  }

  inline bool ReadBitAndClockWithoutDelay()
  {
    swclk_.Write(false);
    const bool BIT = swdio_.Read();
    swclk_.Write(true);
    return BIT;
  }

  inline uint8_t ReadByteLSB()
  {
    uint8_t v = 0u;
    for (uint32_t i = 0; i < BYTE_BITS; ++i)
    {
      if (ReadBitAndClock())
      {
        v = static_cast<uint8_t>(v | (1u << i));
      }
    }
    return v;
  }

  inline uint8_t ReadByteLSBWithoutDelay()
  {
    uint8_t v = 0u;
    v |= static_cast<uint8_t>(ReadBitAndClockWithoutDelay() ? 0x01u : 0u);
    v |= static_cast<uint8_t>(ReadBitAndClockWithoutDelay() ? 0x02u : 0u);
    v |= static_cast<uint8_t>(ReadBitAndClockWithoutDelay() ? 0x04u : 0u);
    v |= static_cast<uint8_t>(ReadBitAndClockWithoutDelay() ? 0x08u : 0u);
    v |= static_cast<uint8_t>(ReadBitAndClockWithoutDelay() ? 0x10u : 0u);
    v |= static_cast<uint8_t>(ReadBitAndClockWithoutDelay() ? 0x20u : 0u);
    v |= static_cast<uint8_t>(ReadBitAndClockWithoutDelay() ? 0x40u : 0u);
    v |= static_cast<uint8_t>(ReadBitAndClockWithoutDelay() ? 0x80u : 0u);
    return v;
  }

  static void BusyLoop(uint32_t loops)
  {
    volatile uint32_t sink = loops;
    while (sink--)
    {
    }
  }

 private:
  ErrorCode TransferWithDelay(const SwdProtocol::Request& req,
                              SwdProtocol::Response& resp)
  {
    resp.ack = SwdProtocol::Ack::PROTOCOL;
    resp.rdata = 0u;
    resp.parity_ok = true;

    const bool APNDP = (req.port == SwdProtocol::Port::AP);
    const uint8_t REQUEST_BYTE = MakeReq(APNDP, req.rnw, req.addr2b);

    (void)SetSwdioDriveMode();
    WriteByteLSB(REQUEST_BYTE);

    (void)SetSwdioSampleMode();
    GenOneClk();  // turnaround Host -> Target

    // ACK: CMSIS SW_READ_BIT phase (sample in low phase)
    uint8_t ack_raw = 0u;
    for (uint32_t i = 0; i < ACK_BITS; ++i)
    {
      if (ReadBitAndClock())
      {
        ack_raw |= static_cast<uint8_t>(1u << i);
      }
    }
    resp.ack = DecodeAck(static_cast<uint8_t>(ack_raw & 0x7u));

    if (resp.ack != SwdProtocol::Ack::OK)
    {
      GenOneClk();  // turnaround Target -> Host (skip data)
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
        const uint32_t B = ReadByteLSB();
        data |= (B << (8u * byte));
      }

      const bool PARITY_BIT = ReadBitAndClock();
      resp.rdata = data;
      resp.parity_ok = (static_cast<uint8_t>(PARITY_BIT) == Parity32(data));

      (void)SetSwdioDriveMode();
      swdio_.Write(true);
      GenOneClk();

      swclk_.Write(false);
    }
    else
    {
      (void)SetSwdioDriveMode();
      GenOneClk();

      const uint32_t DATA = req.wdata;
      for (uint32_t byte = 0; byte < 4u; ++byte)
      {
        const uint8_t B = static_cast<uint8_t>((DATA >> (8u * byte)) & 0xFFu);
        WriteByteLSB(B);
      }

      const bool PARITY_BIT = (Parity32(DATA) & 0x1u) != 0u;
      WriteBit(PARITY_BIT);

      // Trailing idle clocks after a write data phase: keep SWDIO driven LOW and
      // clock the target so it completes/settles the (possibly posted) write before
      // the next request start bit. Without this, a transfer immediately following an
      // AP write in the same DAP_Transfer batch samples ACK at the wrong phase (JUNK).
      swdio_.Write(false);
      for (uint32_t k = 0; k < WRITE_TRAILING_IDLE_CLOCKS; ++k)
      {
        GenOneClk();
      }

      swdio_.Write(true);
      swclk_.Write(false);
    }

    return ErrorCode::OK;
  }

  ErrorCode TransferWithoutDelay(const SwdProtocol::Request& req,
                                 SwdProtocol::Response& resp)
  {
    resp.ack = SwdProtocol::Ack::PROTOCOL;
    resp.rdata = 0u;
    resp.parity_ok = true;

    const bool APNDP = (req.port == SwdProtocol::Port::AP);
    const uint8_t REQUEST_BYTE = MakeReq(APNDP, req.rnw, req.addr2b);

    (void)SetSwdioDriveMode();
    WriteByteLSBWithoutDelay(REQUEST_BYTE);

    (void)SetSwdioSampleMode();
    GenOneClkWithoutDelay();  // turnaround Host -> Target

    // ACK: CMSIS SW_READ_BIT phase (sample in low phase)
    uint8_t ack_raw = 0u;
    for (uint32_t i = 0; i < ACK_BITS; ++i)
    {
      if (ReadBitAndClockWithoutDelay())
      {
        ack_raw |= static_cast<uint8_t>(1u << i);
      }
    }
    resp.ack = DecodeAck(static_cast<uint8_t>(ack_raw & 0x7u));

    if (resp.ack != SwdProtocol::Ack::OK)
    {
      GenOneClkWithoutDelay();
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
        const uint32_t B = ReadByteLSBWithoutDelay();
        data |= (B << (8u * byte));
      }

      const bool PARITY_BIT = ReadBitAndClockWithoutDelay();
      resp.rdata = data;
      resp.parity_ok = (static_cast<uint8_t>(PARITY_BIT) == Parity32(data));

      (void)SetSwdioDriveMode();
      swdio_.Write(true);
      GenOneClkWithoutDelay();

      swclk_.Write(false);
    }
    else
    {
      (void)SetSwdioDriveMode();
      GenOneClkWithoutDelay();

      const uint32_t DATA = req.wdata;
      for (uint32_t byte = 0; byte < 4u; ++byte)
      {
        const uint8_t B = static_cast<uint8_t>((DATA >> (8u * byte)) & 0xFFu);
        WriteByteLSBWithoutDelay(B);
      }

      const bool PARITY_BIT = (Parity32(DATA) & 0x1u) != 0u;
      WriteBitWithoutDelay(PARITY_BIT);

      // Trailing idle clocks after a write data phase (see delay-path note).
      swdio_.Write(false);
      for (uint32_t k = 0; k < WRITE_TRAILING_IDLE_CLOCKS; ++k)
      {
        GenOneClkWithoutDelay();
      }

      swdio_.Write(true);
      swclk_.Write(false);
    }

    return ErrorCode::OK;
  }

 private:
  static constexpr uint32_t DEFAULT_CLOCK_HZ =
      500'000u;  ///< 默认 SWCLK 频率（Hz）。Default SWCLK frequency (Hz).
  static constexpr uint32_t LINE_RESET_CYCLES =
      64u;  ///< 线复位时钟周期数。Line reset clock cycles.
  static constexpr uint32_t BYTE_BITS = 8u;  ///< 每字节比特数。Bits per byte.
  static constexpr uint32_t ACK_BITS = 3u;   ///< ACK 比特数。ACK bits.
  static constexpr uint32_t WRITE_TRAILING_IDLE_CLOCKS =
      3u;  ///< Idle clocks (SWDIO low) after a write data phase to settle posted writes.

  static constexpr uint8_t JTAG_TO_SWD_SEQ0 =
      0x9Eu;  ///< JTAG->SWD 序列字节 0。JTAG-to-SWD sequence byte 0.
  static constexpr uint8_t JTAG_TO_SWD_SEQ1 =
      0xE7u;  ///< JTAG->SWD 序列字节 1。JTAG-to-SWD sequence byte 1.

  SwclkGpioType& swclk_;  ///< SWCLK GPIO。GPIO for SWCLK.
  SwdioGpioType& swdio_;  ///< SWDIO GPIO。GPIO for SWDIO.

  uint32_t clock_hz_ = 0u;  ///< 当前 SWCLK 频率（Hz）。Current SWCLK frequency (Hz).

  uint32_t loops_per_us_ = 0u;       // 手调系数：BusyLoop 每微秒大约需要的迭代数
  uint32_t half_period_ns_ = 0u;     // 当前半周期（ns）
  uint32_t half_period_loops_ = 0u;  // 当前半周期对应的 BusyLoop 迭代数

  SwdioMode swdio_mode_ =
      SwdioMode::UNKNOWN;  ///< SWDIO 当前模式缓存。Cached current SWDIO mode.
};

}  // namespace LibXR::Debug
