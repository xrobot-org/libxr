#pragma once

#include <array>
#include <cstdint>

#include "gpio.hpp"
#include "rvswd.hpp"
#include "rvswd_protocol.hpp"

namespace LibXR::Debug {
enum class RvSwdioDriveMode : uint8_t
{
  PUSH_PULL = 0,
  OPEN_DRAIN = 1,
};

/**
 * @brief RVSWD protocol over LibXR GPIO bit-banging.
 */
template <typename RvSwclkGpioType, typename RvSwdioGpioType,
          RvSwdioDriveMode IO_DRIVE_MODE = RvSwdioDriveMode::PUSH_PULL>
class RvSwdGeneralGPIO final : public RvSwd
{
  static constexpr uint32_t MIN_HZ = 10'000u;
  static constexpr uint32_t MAX_HZ = 100'000'000u;
  static constexpr uint32_t DEFAULT_CLOCK_HZ = 1'000'000u;
  static constexpr uint32_t NS_PER_SEC = 1'000'000'000u;
  static constexpr uint32_t LOOPS_SCALE = 1000u;
  static constexpr uint32_t CEIL_BIAS = LOOPS_SCALE - 1u;

  static constexpr uint32_t HalfPeriodNsFromHz(uint32_t hz)
  {
    return (NS_PER_SEC + (2u * hz) - 1u) / (2u * hz);
  }

  static constexpr uint32_t HALF_PERIOD_NS_MAX = HalfPeriodNsFromHz(MIN_HZ);
  static constexpr uint32_t MAX_LOOPS_PER_US =
      (UINT32_MAX - CEIL_BIAS) / HALF_PERIOD_NS_MAX;

 public:
  static constexpr uint32_t kLegacyReadTurnaroundBits = 1u;
  static constexpr uint32_t kDataBits = 32u;
  static constexpr uint32_t kDmiAddrBits = 7u;
  static constexpr uint32_t kDmiOpBits = 2u;
  static constexpr uint32_t kDmiParityBits = 1u;
  static constexpr uint32_t kDmiFrameBits =
      kDmiAddrBits + kDataBits + kDmiOpBits + kDmiParityBits;
  static constexpr uint32_t kRvSwdOpBits = 1u;
  static constexpr uint32_t kRvSwdHeaderParityBits = 1u;
  static constexpr uint32_t kRvSwdHostPadBits = 5u;
  static constexpr uint32_t kRvSwdDataParityBits = 1u;
  static constexpr uint32_t kRvSwdHostTailBits = 5u;
  static constexpr uint32_t kReadPrefixBits =
      kDmiAddrBits + kRvSwdOpBits + kRvSwdHeaderParityBits + kRvSwdHostPadBits;
  static constexpr uint32_t kWriteFrameBits =
      kReadPrefixBits + kDataBits + kRvSwdDataParityBits + kRvSwdHostTailBits;
  static constexpr uint32_t kDmiCaptureExtraBits = 8u;
  static constexpr uint32_t kDmiCaptureBits = kDmiFrameBits + kDmiCaptureExtraBits;
  static constexpr uint32_t kWakeupBits = 100u;
  static constexpr uint8_t kRvSwdPadPrefix = 0x15u;  // 10101
  static constexpr uint8_t kRvSwdTail = 0x17u;       // 10111
  static constexpr uint8_t kDmiAddrMask = 0x7Fu;
  static constexpr uint32_t kOnlineCaprExpected = 0x00010403u;
  static constexpr uint8_t kBridgeMaintenanceCmd = 0x0Eu;
  static constexpr uint8_t kBridgeAckOk = 0x01u;
  static constexpr uint8_t kBridgeAckWait = 0x02u;
  static constexpr uint8_t kBridgeAckFault = 0x04u;
  // Firmware-side 0x40A0 uses a 16-bit retry pair, so keep host-side retries
  // generous to avoid premature FAIL on transient WAIT windows.
  static constexpr uint8_t kBridgeWaitRetryLimit = 0xFFu;

  explicit RvSwdGeneralGPIO(RvSwclkGpioType& rvswclk, RvSwdioGpioType& rvswdio,
                            uint32_t loops_per_us,
                            uint32_t default_hz = DEFAULT_CLOCK_HZ)
      : rvswclk_(rvswclk), rvswdio_(rvswdio), loops_per_us_(loops_per_us)
  {
    if (loops_per_us_ > MAX_LOOPS_PER_US)
    {
      loops_per_us_ = MAX_LOOPS_PER_US;
    }

    rvswclk_.SetConfig(
        {RvSwclkGpioType::Direction::OUTPUT_PUSH_PULL, RvSwclkGpioType::Pull::NONE});
    rvswclk_.Write(true);

    (void)SetRvSwdioDriveMode();
    rvswdio_.Write(true);

    (void)SetClockHz(default_hz);
  }
  ~RvSwdGeneralGPIO() override = default;

  RvSwdGeneralGPIO(const RvSwdGeneralGPIO&) = delete;
  RvSwdGeneralGPIO& operator=(const RvSwdGeneralGPIO&) = delete;

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
    half_period_ns_ = HalfPeriodNsFromHz(hz);

    if (loops_per_us_ == 0u)
    {
      half_period_loops_ = 0u;
      return ErrorCode::OK;
    }

    const uint64_t loops_scaled =
        static_cast<uint64_t>(loops_per_us_) * static_cast<uint64_t>(half_period_ns_);
    half_period_loops_ =
        (loops_scaled < LOOPS_SCALE)
            ? 0u
            : static_cast<uint32_t>((loops_scaled + CEIL_BIAS) / LOOPS_SCALE);

    return ErrorCode::OK;
  }

  uint64_t GetLastHostFrame() const { return last_host_frame_; }
  uint64_t GetLastTargetFrame() const { return last_target_frame_; }
  bool HasLastTargetFrame() const { return last_target_frame_valid_; }
  uint64_t GetLastTargetRaw43() const { return last_target_raw43_; }
  bool HasLastTargetRaw43() const { return last_target_raw43_valid_; }
  uint8_t GetLastTargetFrameInfo() const { return last_target_frame_info_; }
  uint32_t GetLastOnlineCapr() const { return last_online_capr_; }
  bool HasLastOnlineCapr() const { return last_online_capr_valid_; }
  uint8_t GetLastEnterState() const { return last_enter_state_; }
  uint8_t GetLastOnlineShadAddr() const { return last_online_shad_addr_; }
  uint8_t GetLastOnlineMaskAddr() const { return last_online_mask_addr_; }
  uint8_t GetLastOnlineCaprAddr() const { return last_online_capr_addr_; }
  uint8_t GetLastOnlineCaprAttemptMask() const { return last_online_capr_attempt_mask_; }
  uint8_t GetLastOnlineCaprTurnBit() const { return last_online_capr_turn_bit_; }
  uint32_t GetLastOnlineCaprTurnValue() const { return last_online_capr_turn_value_; }
  uint32_t GetLastOnlineCaprNoTurnValue() const { return last_online_capr_noturn_value_; }

  struct TransferDebugSnapshot
  {
    uint8_t stage = 0u;
    int8_t last_ec = 0;
    uint8_t last_req_addr = 0u;
    uint8_t last_req_tail = 0u;
    uint8_t last_resp_addr = 0u;
    uint8_t last_resp_tail = 0u;
    uint8_t last_parity_rx = 0u;
    uint8_t last_parity_calc = 0u;
    uint32_t last_req_data = 0u;
    uint32_t last_resp_data = 0u;
    uint32_t last_tx_lo = 0u;
    uint32_t last_tx_hi = 0u;
    uint32_t last_rx_lo = 0u;
    uint32_t last_rx_hi = 0u;
  };

  struct RawReadCapture
  {
    uint8_t dmi_addr = 0u;
    uint8_t bit_count = 0u;
    int8_t last_ec = static_cast<int8_t>(ErrorCode::OK);
    std::array<uint8_t, 12u> raw_bits = {};
  };

  static inline volatile TransferDebugSnapshot debug_snapshot_ = {};

  ErrorCode DebugCaptureReadBits(uint8_t dmi_addr, uint8_t bit_count,
                                 RawReadCapture& capture)
  {
    capture = {};
    capture.dmi_addr = dmi_addr;
    capture.bit_count = bit_count;
    debug_snapshot_.stage = 0x30u;
    debug_snapshot_.last_req_addr = dmi_addr;
    debug_snapshot_.last_req_tail = bit_count;
    debug_snapshot_.last_ec = static_cast<int8_t>(ErrorCode::OK);

    if (bit_count == 0u || bit_count > static_cast<uint8_t>(capture.raw_bits.size() * 8u) ||
        bit_count > 64u)
    {
      capture.last_ec = static_cast<int8_t>(ErrorCode::ARG_ERR);
      debug_snapshot_.last_ec = capture.last_ec;
      return ErrorCode::ARG_ERR;
    }

    uint64_t raw = 0u;
    const ErrorCode ec = CaptureReadWindow(dmi_addr, bit_count, raw);
    capture.last_ec = static_cast<int8_t>(ec);
    debug_snapshot_.last_ec = capture.last_ec;
    if (ec != ErrorCode::OK)
    {
      return ec;
    }

    for (uint32_t i = 0u; i < bit_count; ++i)
    {
      if (((raw >> i) & 0x01u) != 0u)
      {
        capture.raw_bits[i / 8u] = static_cast<uint8_t>(capture.raw_bits[i / 8u] |
                                                         (1u << (i & 7u)));
      }
    }

    debug_snapshot_.last_rx_lo = static_cast<uint32_t>(raw & 0xFFFF'FFFFu);
    debug_snapshot_.last_rx_hi = static_cast<uint32_t>(raw >> 32u);
    debug_snapshot_.stage = 0x33u;
    return ErrorCode::OK;
  }

  ErrorCode CaptureReadWindow(uint8_t cmd, uint32_t bits, uint64_t& capture)
  {
    if (bits == 0u || bits > 64u)
    {
      return ErrorCode::ARG_ERR;
    }

    const uint8_t addr = static_cast<uint8_t>(cmd & 0x7Fu);
    uint8_t target_addr = 0u;
    uint8_t target_status = 0u;
    uint32_t target_data = 0u;
    const ErrorCode ec = RunDmiTransfer(addr, 0u, Op::READ, target_addr, target_data, target_status);
    if (ec != ErrorCode::OK)
    {
      return ec;
    }

    const uint8_t target_parity = CalcOddParity(target_addr, target_data, target_status);
    const uint64_t raw_frame = (static_cast<uint64_t>(target_addr & 0x7Fu) << 35u) |
                               (static_cast<uint64_t>(target_data) << 3u) |
                               (static_cast<uint64_t>(target_status & 0x03u) << 1u) |
                               static_cast<uint64_t>(target_parity & 0x01u);

    if (bits >= kDmiFrameBits)
    {
      capture = raw_frame;
      return ErrorCode::OK;
    }

    capture = raw_frame >> (kDmiFrameBits - bits);
    return ErrorCode::OK;
  }

  void Close() override
  {
    online_ready_ = false;
    bridge_stage_43_ = 0u;
    bridge_scratch_payload_ = 0u;
    rvswclk_.Write(true);
    (void)SetRvSwdioSampleMode();
  }

  ErrorCode LineReset() override
  {
    online_ready_ = false;
    bridge_stage_43_ = 0u;
    bridge_scratch_payload_ = 0u;
    const ErrorCode ec = SetRvSwdioDriveMode();
    if (ec != ErrorCode::OK)
    {
      return ec;
    }

    rvswdio_.Write(true);
    for (uint32_t i = 0u; i < 64u; ++i)
    {
      GenOneClk();
    }
    return ErrorCode::OK;
  }

  ErrorCode EnterRvSwd() override
  {
    online_ready_ = false;
    bridge_stage_43_ = 0u;
    bridge_scratch_payload_ = 0u;
    last_online_capr_ = 0u;
    last_online_capr_valid_ = false;
    last_enter_state_ = 0u;
    last_online_shad_addr_ = 0u;
    last_online_mask_addr_ = 0u;
    last_online_capr_addr_ = 0u;
    last_online_capr_attempt_mask_ = 0u;
    last_online_capr_turn_bit_ = 0xFFu;
    last_online_capr_turn_value_ = 0u;
    last_online_capr_noturn_value_ = 0u;

    last_enter_state_ = 0x01u;

    const ErrorCode ec = SendWakeupSequence();
    if (ec != ErrorCode::OK)
    {
      return ec;
    }
    last_enter_state_ = 0x06u;
    online_ready_ = true;
    last_enter_state_ = 0x07u;
    return ErrorCode::OK;
  }

  ErrorCode Transfer(const Request& req, Response& resp) override
  {
    const uint8_t addr = static_cast<uint8_t>(req.addr & 0x7Fu);
    resp.addr = addr;
    resp.data = 0u;
    resp.ack = Ack::PROTOCOL;

    if (req.op != Op::NOP && req.op != Op::READ && req.op != Op::WRITE)
    {
      return ErrorCode::ARG_ERR;
    }

    if (!online_ready_)
    {
      const ErrorCode ec = EnterRvSwd();
      if (ec != ErrorCode::OK)
      {
        resp.ack = Ack::PROTOCOL;
        return ec;
      }
    }

    if (req.op == Op::READ)
    {
      uint32_t data = 0u;
      const ErrorCode ec = ReadWordRaw(addr, data);
      if (ec != ErrorCode::OK)
      {
        resp.ack = Ack::FAILED;
        return ec;
      }
      resp.data = data;
      resp.ack = Ack::OK;
      return ErrorCode::OK;
    }

    if (req.op == Op::WRITE)
    {
      const ErrorCode ec = WriteWordRaw(addr, req.data);
      if (ec != ErrorCode::OK)
      {
        resp.ack = Ack::FAILED;
        return ec;
      }
      resp.data = req.data;
      resp.ack = Ack::OK;
      return ErrorCode::OK;
    }

    uint8_t target_addr = 0u;
    uint8_t target_status = 0u;
    uint32_t target_data = 0u;
    const ErrorCode ec = RunDmiTransfer(addr, req.data, req.op, target_addr, target_data,
                                        target_status);
    if (ec != ErrorCode::OK)
    {
      online_ready_ = false;
      resp.ack = Ack::PROTOCOL;
      return ec;
    }

    resp.addr = target_addr;
    resp.data = target_data;
    resp.ack = RvSwdProtocol::DecodeDmiStatus(target_status);
    if (resp.ack == Ack::PROTOCOL)
    {
      return ErrorCode::FAILED;
    }
    return ErrorCode::OK;
  }

  void IdleClocks(uint32_t cycles) override
  {
    (void)SetRvSwdioDriveMode();
    rvswdio_.Write(true);
    for (uint32_t i = 0u; i < cycles; ++i)
    {
      GenOneClk();
    }
    rvswdio_.Write(false);
  }

 private:
  static uint8_t Parity8(uint8_t value)
  {
    value = static_cast<uint8_t>(value ^ (value >> 4u));
    value = static_cast<uint8_t>(value ^ (value >> 2u));
    value = static_cast<uint8_t>(value ^ (value >> 1u));
    return static_cast<uint8_t>(value & 0x01u);
  }

  static uint8_t Parity32(uint32_t value)
  {
    value ^= (value >> 16u);
    value ^= (value >> 8u);
    value ^= (value >> 4u);
    value ^= (value >> 2u);
    value ^= (value >> 1u);
    return static_cast<uint8_t>(value & 0x01u);
  }

  static uint8_t Parity7PlusOp(uint8_t dmi_addr, bool write)
  {
    uint8_t parity = 0u;
    const uint8_t addr = static_cast<uint8_t>(dmi_addr & kDmiAddrMask);
    for (uint8_t bit = 0u; bit < kDmiAddrBits; ++bit)
    {
      parity ^= static_cast<uint8_t>((addr >> bit) & 0x01u);
    }
    if (write)
    {
      parity ^= 0x01u;
    }
    return static_cast<uint8_t>(parity & 0x01u);
  }

  static uint16_t BuildReadPrefix(uint8_t dmi_addr)
  {
    const uint16_t addr = static_cast<uint16_t>(dmi_addr & kDmiAddrMask);
    const uint16_t parity = static_cast<uint16_t>(Parity7PlusOp(dmi_addr, false));
    return static_cast<uint16_t>(
        (addr << (kRvSwdOpBits + kRvSwdHeaderParityBits + kRvSwdHostPadBits)) |
        (parity << kRvSwdHostPadBits) | kRvSwdPadPrefix);
  }

  static uint64_t BuildWriteFrame(uint8_t dmi_addr, uint32_t data)
  {
    const uint64_t addr = static_cast<uint64_t>(dmi_addr & kDmiAddrMask);
    const uint64_t prefix_parity = static_cast<uint64_t>(Parity7PlusOp(dmi_addr, true));
    const uint64_t prefix =
        (addr << (kRvSwdOpBits + kRvSwdHeaderParityBits + kRvSwdHostPadBits)) |
        (static_cast<uint64_t>(1u) << (kRvSwdHeaderParityBits + kRvSwdHostPadBits)) |
        (prefix_parity << kRvSwdHostPadBits) | kRvSwdPadPrefix;
    const uint64_t data_parity = static_cast<uint64_t>(Parity32(data));
    return (prefix << (kDataBits + kRvSwdDataParityBits + kRvSwdHostTailBits)) |
           (static_cast<uint64_t>(data) << (kRvSwdDataParityBits + kRvSwdHostTailBits)) |
           (data_parity << kRvSwdHostTailBits) | kRvSwdTail;
  }

  static uint8_t CalcOddParity(uint8_t addr7, uint32_t data, uint8_t op2)
  {
    const uint8_t parity = static_cast<uint8_t>((Parity8(addr7) ^ Parity32(data) ^ Parity8(op2)) & 0x01u);
    return static_cast<uint8_t>(parity ^ 0x01u);
  }

  static uint8_t CalcEvenParity(uint8_t addr7, uint32_t data, uint8_t status2)
  {
    return static_cast<uint8_t>((Parity8(addr7) ^ Parity32(data) ^ Parity8(status2)) & 0x01u);
  }

  static void SetBitLsb(uint8_t* buf, uint32_t bit_index, bool bit)
  {
    const uint32_t byte_index = bit_index / 8u;
    const uint32_t bit_offset = bit_index & 7u;
    if (bit)
    {
      buf[byte_index] = static_cast<uint8_t>(buf[byte_index] | (1u << bit_offset));
    }
  }

  static bool GetBitLsb(const uint8_t* buf, uint32_t bit_index)
  {
    const uint32_t byte_index = bit_index / 8u;
    const uint32_t bit_offset = bit_index & 7u;
    return ((buf[byte_index] >> bit_offset) & 0x01u) != 0u;
  }

  ErrorCode WriteBits(uint32_t bits, const uint8_t* data_lsb_first)
  {
    return RvSwdSeqWriteBits(bits, data_lsb_first);
  }

  ErrorCode EncodeAndWriteMsbFirst(uint64_t payload, uint32_t bits)
  {
    if (bits == 0u || bits > 64u)
    {
      return ErrorCode::ARG_ERR;
    }

    std::array<uint8_t, 8> packed = {};
    for (uint32_t i = 0u; i < bits; ++i)
    {
      const bool bit = ((payload >> (bits - 1u - i)) & 0x01u) != 0u;
      SetBitLsb(packed.data(), i, bit);
    }
    return WriteBits(bits, packed.data());
  }

  ErrorCode ReadMsbFirstU32RvSwd(uint32_t bits, uint32_t& out)
  {
    if (bits == 0u || bits > 32u)
    {
      return ErrorCode::ARG_ERR;
    }

    std::array<uint8_t, 4> packed = {};
    const ErrorCode ec = RvSwdReadBits(bits, packed.data());
    if (ec != ErrorCode::OK)
    {
      return ec;
    }

    uint32_t value = 0u;
    for (uint32_t i = 0u; i < bits; ++i)
    {
      value = (value << 1u) | static_cast<uint32_t>(GetBitLsb(packed.data(), i));
    }
    out = value;
    return ErrorCode::OK;
  }

  ErrorCode ReadMsbFirstU64RvSwd(uint32_t bits, uint64_t& out)
  {
    if (bits == 0u || bits > 64u)
    {
      return ErrorCode::ARG_ERR;
    }

    std::array<uint8_t, 8> packed = {};
    const ErrorCode ec = RvSwdReadBits(bits, packed.data());
    if (ec != ErrorCode::OK)
    {
      return ec;
    }

    uint64_t value = 0u;
    for (uint32_t i = 0u; i < bits; ++i)
    {
      value = (value << 1u) | static_cast<uint64_t>(GetBitLsb(packed.data(), i));
    }
    out = value;
    return ErrorCode::OK;
  }

  ErrorCode ReadMsbFirstU32(uint32_t bits, uint32_t& out)
  {
    if (bits == 0u || bits > 32u)
    {
      return ErrorCode::ARG_ERR;
    }

    std::array<uint8_t, 4> packed = {};
    const ErrorCode ec = SeqReadBits(bits, packed.data());
    if (ec != ErrorCode::OK)
    {
      return ec;
    }

    uint32_t value = 0u;
    for (uint32_t i = 0u; i < bits; ++i)
    {
      value = (value << 1u) | static_cast<uint32_t>(GetBitLsb(packed.data(), i));
    }
    out = value;
    return ErrorCode::OK;
  }

  ErrorCode SendLegacyStopBit()
  {
    const uint8_t one_lsb = 0x01u;
    const ErrorCode ec = WriteBits(1u, &one_lsb);
    if (ec != ErrorCode::OK)
    {
      return ec;
    }
    IdleClocks(2u);
    return ErrorCode::OK;
  }

  ErrorCode LegacyWriteWordRaw(uint8_t wire_addr_with_rw, uint32_t data)
  {
    wire_addr_with_rw = RvSwdProtocol::EncodeLegacyWriteAddr(wire_addr_with_rw);
    const uint64_t frame =
        (static_cast<uint64_t>(1u) << 40u) | (static_cast<uint64_t>(wire_addr_with_rw) << 32u) |
        static_cast<uint64_t>(data);

    ErrorCode ec = EncodeAndWriteMsbFirst(frame, 41u);
    if (ec != ErrorCode::OK)
    {
      return ec;
    }
    return SendLegacyStopBit();
  }

  ErrorCode LegacyReadWordRaw(uint8_t wire_addr_with_rw, uint32_t& data)
  {
    wire_addr_with_rw = RvSwdProtocol::EncodeLegacyReadAddr(wire_addr_with_rw);
    const uint16_t frame = static_cast<uint16_t>((1u << 8u) | wire_addr_with_rw);

    ErrorCode ec = EncodeAndWriteMsbFirst(frame, 9u);
    if (ec != ErrorCode::OK)
    {
      return ec;
    }

    ec = RvSwdPrepareReadTurnaround();
    if (ec != ErrorCode::OK)
    {
      return ec;
    }

    uint32_t turnaround = 0u;
    ec = ReadMsbFirstU32(kLegacyReadTurnaroundBits, turnaround);
    if (ec != ErrorCode::OK)
    {
      return ec;
    }

    ec = ReadMsbFirstU32(kDataBits, data);
    if (ec != ErrorCode::OK)
    {
      return ec;
    }

    return SendLegacyStopBit();
  }

  ErrorCode LegacyReadWordRawCapture(uint8_t wire_addr_with_rw, uint32_t& data,
                                     uint8_t& turnaround_bit)
  {
    wire_addr_with_rw = RvSwdProtocol::EncodeLegacyReadAddr(wire_addr_with_rw);
    const uint16_t frame = static_cast<uint16_t>((1u << 8u) | wire_addr_with_rw);

    ErrorCode ec = EncodeAndWriteMsbFirst(frame, 9u);
    if (ec != ErrorCode::OK)
    {
      return ec;
    }

    ec = RvSwdPrepareReadTurnaround();
    if (ec != ErrorCode::OK)
    {
      return ec;
    }

    uint32_t turnaround = 0u;
    ec = ReadMsbFirstU32(kLegacyReadTurnaroundBits, turnaround);
    if (ec != ErrorCode::OK)
    {
      return ec;
    }
    turnaround_bit = static_cast<uint8_t>(turnaround & 0x01u);

    ec = ReadMsbFirstU32(kDataBits, data);
    if (ec != ErrorCode::OK)
    {
      return ec;
    }

    return SendLegacyStopBit();
  }

  ErrorCode LegacyReadWordRawNoTurn(uint8_t wire_addr_with_rw, uint32_t& data)
  {
    wire_addr_with_rw = RvSwdProtocol::EncodeLegacyReadAddr(wire_addr_with_rw);
    const uint16_t frame = static_cast<uint16_t>((1u << 8u) | wire_addr_with_rw);

    ErrorCode ec = EncodeAndWriteMsbFirst(frame, 9u);
    if (ec != ErrorCode::OK)
    {
      return ec;
    }

    ec = ReadMsbFirstU32(kDataBits, data);
    if (ec != ErrorCode::OK)
    {
      return ec;
    }

    return SendLegacyStopBit();
  }

  ErrorCode SendWakeupSequence()
  {
    return RvSwdWakePulse();
  }

  ErrorCode EnterOnlineAndReadCapr(bool use_turnaround, uint32_t& online_capr)
  {
    ErrorCode ec = SendWakeupSequence();
    if (ec != ErrorCode::OK)
    {
      return ec;
    }
    last_enter_state_ = 0x02u;

    last_online_shad_addr_ =
        RvSwdProtocol::EncodeLegacyWriteAddr(RvSwdProtocol::ONLINE_CFGR_SHAD);
    ec = LegacyWriteWordRaw(RvSwdProtocol::ONLINE_CFGR_SHAD,
                            RvSwdProtocol::ONLINE_ENABLE_OUTPUT);
    if (ec != ErrorCode::OK)
    {
      return ec;
    }
    last_enter_state_ = 0x03u;

    last_online_mask_addr_ =
        RvSwdProtocol::EncodeLegacyWriteAddr(RvSwdProtocol::ONLINE_CFGR_MASK);
    ec = LegacyWriteWordRaw(RvSwdProtocol::ONLINE_CFGR_MASK,
                            RvSwdProtocol::ONLINE_ENABLE_OUTPUT);
    if (ec != ErrorCode::OK)
    {
      return ec;
    }
    last_enter_state_ = 0x04u;

    last_online_capr_addr_ =
        RvSwdProtocol::EncodeLegacyReadAddr(RvSwdProtocol::ONLINE_CAPR_STA);
    if (use_turnaround)
    {
      last_online_capr_attempt_mask_ |= 0x01u;
      ec = LegacyReadWordRawCapture(RvSwdProtocol::ONLINE_CAPR_STA, online_capr,
                                    last_online_capr_turn_bit_);
      if (ec == ErrorCode::OK)
      {
        last_online_capr_turn_value_ = online_capr;
        last_online_capr_attempt_mask_ |= 0x10u;
      }
      return ec;
    }

    last_online_capr_attempt_mask_ |= 0x02u;
    ec = LegacyReadWordRawNoTurn(RvSwdProtocol::ONLINE_CAPR_STA, online_capr);
    if (ec == ErrorCode::OK)
    {
      last_online_capr_noturn_value_ = online_capr;
      last_online_capr_attempt_mask_ |= 0x20u;
    }
    return ec;
  }

  static std::array<uint8_t, 4> ToBeBytes(uint32_t value)
  {
    return {static_cast<uint8_t>(value >> 24), static_cast<uint8_t>(value >> 16),
            static_cast<uint8_t>(value >> 8), static_cast<uint8_t>(value)};
  }

  static uint32_t FromBeBytes(const std::array<uint8_t, 4>& bytes)
  {
    return (static_cast<uint32_t>(bytes[0]) << 24) | (static_cast<uint32_t>(bytes[1]) << 16) |
           (static_cast<uint32_t>(bytes[2]) << 8) | static_cast<uint32_t>(bytes[3]);
  }

  static uint8_t DecodeBridgeAckBits(uint8_t ack_bits_lsb)
  {
    const uint8_t bits = static_cast<uint8_t>(ack_bits_lsb & 0x07u);
    if (bits == 0x01u)
    {
      return kBridgeAckOk;
    }
    if (bits == 0x02u)
    {
      return kBridgeAckWait;
    }
    if (bits == 0x04u)
    {
      return kBridgeAckFault;
    }
    return kBridgeAckFault;
  }

  static uint8_t BridgeAckToTargetStatus(uint8_t bridge_ack)
  {
    switch (bridge_ack)
    {
      case kBridgeAckOk:
        return 0x00u;
      case kBridgeAckWait:
        return 0x03u;
      case kBridgeAckFault:
      default:
        return 0x02u;
    }
  }

  enum class BridgePayloadMode : uint8_t
  {
    None = 0u,
    Primary = 1u,
    Scratch = 2u,
  };

  ErrorCode RunRvswdCommand(uint8_t cmd, std::array<uint8_t, 4>& payload_be,
                            BridgePayloadMode payload_mode,
                            uint8_t& bridge_ack, uint8_t& ack_bits_lsb,
                            bool& payload_updated)
  {
    bridge_ack = kBridgeAckFault;
    ack_bits_lsb = 0u;
    payload_updated = false;

    ErrorCode ec = RvSwdStart();
    if (ec != ErrorCode::OK)
    {
      return ec;
    }

    uint8_t cmd_lsb = cmd;
    ec = WriteBits(8u, &cmd_lsb);
    if (ec != ErrorCode::OK)
    {
      (void)RvSwdStop();
      return ec;
    }

    ec = RvSwdPrepareReadTurnaround();
    if (ec != ErrorCode::OK)
    {
      (void)RvSwdStop();
      return ec;
    }

    std::array<uint8_t, 1> ack_buf = {};
    ec = RvSwdReadBits(3u, ack_buf.data());
    if (ec != ErrorCode::OK)
    {
      (void)RvSwdStop();
      return ec;
    }
    ack_bits_lsb = static_cast<uint8_t>(ack_buf[0] & 0x07u);
    bridge_ack = DecodeBridgeAckBits(ack_bits_lsb);

    std::array<uint8_t, 4> active_payload = payload_be;
    if (payload_mode == BridgePayloadMode::Scratch)
    {
      active_payload = ToBeBytes(bridge_scratch_payload_);
    }

    if (bridge_ack == kBridgeAckOk && payload_mode != BridgePayloadMode::None)
    {
      const bool read_path = (cmd & 0x02u) != 0u;
      if (read_path)
      {
        std::array<uint8_t, 4> read_bytes = {};
        ec = RvSwdReadBits(32u, read_bytes.data());
        if (ec != ErrorCode::OK)
        {
          (void)RvSwdStop();
          return ec;
        }
        active_payload = read_bytes;
      }
      else
      {
        ec = WriteBits(32u, active_payload.data());
        if (ec != ErrorCode::OK)
        {
          (void)RvSwdStop();
          return ec;
        }
      }

      if (payload_mode == BridgePayloadMode::Primary)
      {
        if (read_path)
        {
          payload_be = active_payload;
          payload_updated = true;
        }
      }
      else if (payload_mode == BridgePayloadMode::Scratch)
      {
        bridge_scratch_payload_ = FromBeBytes(active_payload);
      }
    }

    if (bridge_ack == kBridgeAckOk)
    {
      IdleClocks(2u);
    }

    return RvSwdStop();
  }

  ErrorCode RunRvswdWithWaitRetry(uint8_t cmd, std::array<uint8_t, 4>& payload_be,
                                  BridgePayloadMode payload_mode, uint8_t& bridge_ack,
                                  uint8_t& ack_bits_lsb, bool& payload_updated,
                                  uint8_t& wait_retries)
  {
    bridge_ack = kBridgeAckFault;
    ack_bits_lsb = 0u;
    payload_updated = false;
    wait_retries = 0u;

    for (uint8_t attempt = 0u; attempt <= kBridgeWaitRetryLimit; ++attempt)
    {
      bool updated = false;
      uint8_t bits = 0u;
      uint8_t ack = kBridgeAckFault;
      const ErrorCode ec = RunRvswdCommand(cmd, payload_be, payload_mode, ack, bits, updated);
      if (ec != ErrorCode::OK)
      {
        return ec;
      }

      bridge_ack = ack;
      ack_bits_lsb = bits;
      payload_updated = payload_updated || updated;
      if (bridge_ack != kBridgeAckWait)
      {
        return ErrorCode::OK;
      }

      if (wait_retries != 0xFFu)
      {
        ++wait_retries;
      }
    }

    // 0x40A0 bridge semantics: WAIT budget exhausted is treated as
    // transaction failure so upper layers can recover, not as perpetual BUSY.
    bridge_ack = kBridgeAckFault;
    ack_bits_lsb = 0x04u;
    return ErrorCode::OK;
  }

  ErrorCode RunBridgeStage(uint8_t stage_cmd, BridgePayloadMode payload_mode, uint8_t stage_tag,
                           std::array<uint8_t, 4>& payload_be, uint8_t& bridge_ack,
                           uint8_t& ack_bits_lsb, bool& payload_updated,
                           uint8_t& wait_retries, uint8_t& path_tag)
  {
    bool stage_payload_updated = false;
    uint8_t stage_wait_retries = 0u;
    const ErrorCode ec = RunRvswdWithWaitRetry(
        stage_cmd, payload_be, payload_mode, bridge_ack, ack_bits_lsb,
        stage_payload_updated, stage_wait_retries);
    if (ec != ErrorCode::OK)
    {
      return ec;
    }

    payload_updated = payload_updated || stage_payload_updated;
    if (stage_wait_retries > wait_retries)
    {
      wait_retries = stage_wait_retries;
    }
    path_tag = stage_tag;
    return ErrorCode::OK;
  }

  ErrorCode WriteWordRaw(uint8_t dmi_addr, uint32_t data)
  {
    const uint64_t frame = BuildWriteFrame(dmi_addr, data);

    debug_snapshot_.stage = 0x10u;
    debug_snapshot_.last_ec = static_cast<int8_t>(ErrorCode::OK);
    debug_snapshot_.last_req_addr = static_cast<uint8_t>(dmi_addr & kDmiAddrMask);
    debug_snapshot_.last_req_tail = 0x02u;
    debug_snapshot_.last_req_data = data;
    debug_snapshot_.last_resp_addr = 0u;
    debug_snapshot_.last_resp_tail = 0u;
    debug_snapshot_.last_resp_data = 0u;
    debug_snapshot_.last_parity_rx = 0u;
    debug_snapshot_.last_parity_calc = 0u;
    debug_snapshot_.last_tx_lo = static_cast<uint32_t>(frame & 0xFFFF'FFFFu);
    debug_snapshot_.last_tx_hi = static_cast<uint32_t>(frame >> 32u);
    debug_snapshot_.last_rx_lo = 0u;
    debug_snapshot_.last_rx_hi = 0u;

    ErrorCode ec = BeginRvSwdFrame();
    if (ec != ErrorCode::OK)
    {
      return ec;
    }

    ec = EncodeAndWriteMsbFirst(frame, kWriteFrameBits);
    if (ec != ErrorCode::OK)
    {
      (void)EndRvSwdFrame();
      return ec;
    }

    debug_snapshot_.stage = 0x11u;
    ec = EndRvSwdFrame();
    if (ec == ErrorCode::OK)
    {
      debug_snapshot_.stage = 0x12u;
    }
    return ec;
  }

  ErrorCode ReadWordRaw(uint8_t dmi_addr, uint32_t& data)
  {
    const uint16_t header = BuildReadPrefix(dmi_addr);

    debug_snapshot_.stage = 0x20u;
    debug_snapshot_.last_ec = static_cast<int8_t>(ErrorCode::OK);
    debug_snapshot_.last_req_addr = static_cast<uint8_t>(dmi_addr & kDmiAddrMask);
    debug_snapshot_.last_req_tail = 0x01u;
    debug_snapshot_.last_req_data = 0u;
    debug_snapshot_.last_resp_addr = static_cast<uint8_t>(dmi_addr & kDmiAddrMask);
    debug_snapshot_.last_resp_tail = 0x00u;
    debug_snapshot_.last_resp_data = 0u;
    debug_snapshot_.last_parity_rx = 0u;
    debug_snapshot_.last_parity_calc = 0u;
    debug_snapshot_.last_tx_lo = header;
    debug_snapshot_.last_tx_hi = 0u;
    debug_snapshot_.last_rx_lo = 0u;
    debug_snapshot_.last_rx_hi = 0u;

    ErrorCode ec = BeginRvSwdFrame();
    if (ec != ErrorCode::OK)
    {
      return ec;
    }

    ec = EncodeAndWriteMsbFirst(header, kReadPrefixBits);
    if (ec != ErrorCode::OK)
    {
      (void)EndRvSwdFrame();
      return ec;
    }

    debug_snapshot_.stage = 0x21u;
    ec = ReadMsbFirstU32RvSwd(kDataBits, data);
    if (ec != ErrorCode::OK)
    {
      (void)EndRvSwdFrame();
      return ec;
    }

    std::array<uint8_t, 1u> parity_buf = {};
    ec = RvSwdReadBits(1u, parity_buf.data());
    if (ec != ErrorCode::OK)
    {
      (void)EndRvSwdFrame();
      return ec;
    }

    const uint8_t parity_rx = GetBitLsb(parity_buf.data(), 0u) ? 1u : 0u;
    const uint8_t parity_calc = Parity32(data);
    debug_snapshot_.last_parity_rx = parity_rx;
    debug_snapshot_.last_parity_calc = parity_calc;
    debug_snapshot_.last_resp_data = data;
    debug_snapshot_.last_rx_lo = data;
    debug_snapshot_.stage = 0x22u;

    ec = EncodeAndWriteMsbFirst(kRvSwdTail, kRvSwdHostTailBits);
    if (ec != ErrorCode::OK)
    {
      (void)EndRvSwdFrame();
      return ec;
    }

    debug_snapshot_.stage = 0x23u;
    ec = EndRvSwdFrame();
    if (ec != ErrorCode::OK)
    {
      return ec;
    }

    debug_snapshot_.stage = 0x24u;
    if (parity_rx != parity_calc)
    {
      return ErrorCode::FAILED;
    }
    return ErrorCode::OK;
  }

  ErrorCode RunDmiTransfer(uint8_t addr, uint32_t data, Op op, uint8_t& target_addr,
                           uint32_t& target_data, uint8_t& target_status)
  {
    uint8_t op2 = 0u;
    switch (op)
    {
      case Op::NOP:
        op2 = 0u;
        break;
      case Op::READ:
        op2 = 1u;
        break;
      case Op::WRITE:
        op2 = 2u;
        break;
      default:
        return ErrorCode::ARG_ERR;
    }

    const uint8_t host_parity = CalcOddParity(addr, data, op2);
    last_host_frame_ = (static_cast<uint64_t>(addr & 0x7Fu) << 35u) |
                       (static_cast<uint64_t>(data) << 3u) |
                       (static_cast<uint64_t>(op2 & 0x03u) << 1u) |
                       static_cast<uint64_t>(host_parity & 0x01u);
    debug_snapshot_.stage = 0x70u;
    debug_snapshot_.last_ec = static_cast<int8_t>(ErrorCode::OK);
    debug_snapshot_.last_req_addr = static_cast<uint8_t>(addr & 0x7Fu);
    debug_snapshot_.last_req_tail = static_cast<uint8_t>(op2 & 0x03u);
    debug_snapshot_.last_req_data = data;
    debug_snapshot_.last_resp_addr = 0u;
    debug_snapshot_.last_resp_tail = 0u;
    debug_snapshot_.last_resp_data = 0u;
    debug_snapshot_.last_parity_rx = 0u;
    debug_snapshot_.last_parity_calc = host_parity;
    debug_snapshot_.last_tx_lo = static_cast<uint32_t>(last_host_frame_ & 0xFFFF'FFFFu);
    debug_snapshot_.last_tx_hi = static_cast<uint32_t>(last_host_frame_ >> 32u);
    debug_snapshot_.last_rx_lo = 0u;
    debug_snapshot_.last_rx_hi = 0u;
    last_target_frame_ = 0u;
    last_target_frame_valid_ = false;
    last_target_raw43_ = 0u;
    last_target_raw43_valid_ = false;
    last_target_frame_info_ = 0u;
    bridge_stage_43_ = 0u;
    ErrorCode ec = RvSwdStart();
    if (ec != ErrorCode::OK)
    {
      debug_snapshot_.last_ec = static_cast<int8_t>(ec);
      return ec;
    }

    ec = EncodeAndWriteMsbFirst(last_host_frame_, kDmiFrameBits);
    if (ec != ErrorCode::OK)
    {
      (void)RvSwdStop();
      debug_snapshot_.last_ec = static_cast<int8_t>(ec);
      return ec;
    }
    debug_snapshot_.stage = 0x71u;

    ec = RvSwdPrepareReadTurnaround();
    if (ec != ErrorCode::OK)
    {
      (void)RvSwdStop();
      debug_snapshot_.last_ec = static_cast<int8_t>(ec);
      return ec;
    }

    uint64_t raw_target_frame = 0u;
    ec = ReadMsbFirstU64RvSwd(kDmiFrameBits, raw_target_frame);
    if (ec != ErrorCode::OK)
    {
      (void)RvSwdStop();
      debug_snapshot_.last_ec = static_cast<int8_t>(ec);
      return ec;
    }
    debug_snapshot_.stage = 0x72u;
    debug_snapshot_.last_rx_lo = static_cast<uint32_t>(raw_target_frame & 0xFFFF'FFFFu);
    debug_snapshot_.last_rx_hi = static_cast<uint32_t>(raw_target_frame >> 32u);

    ec = RvSwdStop();
    if (ec != ErrorCode::OK)
    {
      debug_snapshot_.last_ec = static_cast<int8_t>(ec);
      return ec;
    }

    target_addr = static_cast<uint8_t>((raw_target_frame >> 35u) & 0x7Fu);
    target_data = static_cast<uint32_t>((raw_target_frame >> 3u) & 0xFFFFFFFFu);
    target_status = static_cast<uint8_t>((raw_target_frame >> 1u) & 0x03u);
    const uint8_t target_parity = static_cast<uint8_t>(raw_target_frame & 0x01u);
    const uint8_t expected_parity = CalcEvenParity(target_addr, target_data, target_status);
    debug_snapshot_.last_resp_addr = target_addr;
    debug_snapshot_.last_resp_tail = target_status;
    debug_snapshot_.last_resp_data = target_data;
    debug_snapshot_.last_parity_rx = target_parity;
    debug_snapshot_.last_parity_calc = expected_parity;

    last_target_frame_ = raw_target_frame;
    last_target_frame_valid_ = true;
    last_target_raw43_ = 0u;
    last_target_raw43_valid_ = false;
    last_target_frame_info_ =
        static_cast<uint8_t>((target_parity == expected_parity) ? 0x01u : 0x00u);

    if (target_parity != expected_parity)
    {
      debug_snapshot_.last_ec = static_cast<int8_t>(ErrorCode::FAILED);
      return ErrorCode::FAILED;
    }

    debug_snapshot_.stage = 0x73u;
    debug_snapshot_.last_ec = static_cast<int8_t>(ErrorCode::OK);
    return ErrorCode::OK;
  }

 private:
  enum class RvSwdioMode : uint8_t
  {
    UNKNOWN = 0u,
    DRIVE,
    SAMPLE_IN,
  };

  ErrorCode SeqWriteBits(uint32_t cycles, const uint8_t* data_lsb_first)
  {
    return RvSwdSeqWriteBits(cycles, data_lsb_first);
  }

  ErrorCode SeqReadBits(uint32_t cycles, uint8_t* out_lsb_first)
  {
    if (cycles == 0u)
    {
      rvswclk_.Write(false);
      return ErrorCode::OK;
    }
    if (out_lsb_first == nullptr)
    {
      return ErrorCode::ARG_ERR;
    }

    ClearBits(out_lsb_first, cycles);

    const ErrorCode ec = SetRvSwdioSampleMode();
    if (ec != ErrorCode::OK)
    {
      return ec;
    }

    rvswclk_.Write(false);
    for (uint32_t i = 0u; i < cycles; ++i)
    {
      bool bit = false;
      if (half_period_loops_ == 0u)
      {
        rvswclk_.Write(false);
        bit = rvswdio_.Read();
        rvswclk_.Write(true);
      }
      else
      {
        rvswclk_.Write(false);
        DelayHalf();
        bit = rvswdio_.Read();
        rvswclk_.Write(true);
        DelayHalf();
      }

      if (bit)
      {
        SetBit(out_lsb_first, i);
      }

      rvswclk_.Write(false);
    }

    return ErrorCode::OK;
  }

  ErrorCode RvSwdReadBits(uint32_t cycles, uint8_t* out_lsb_first)
  {
    if (cycles == 0u)
    {
      rvswclk_.Write(false);
      return ErrorCode::OK;
    }
    if (out_lsb_first == nullptr)
    {
      return ErrorCode::ARG_ERR;
    }

    ClearBits(out_lsb_first, cycles);

    const ErrorCode ec = SetRvSwdioSampleMode();
    if (ec != ErrorCode::OK)
    {
      return ec;
    }

    rvswclk_.Write(false);
    for (uint32_t i = 0u; i < cycles; ++i)
    {
      bool bit = false;
      if (half_period_loops_ == 0u)
      {
        rvswclk_.Write(true);
        bit = rvswdio_.Read();
        rvswclk_.Write(false);
      }
      else
      {
        rvswclk_.Write(true);
        DelayHalf();
        bit = rvswdio_.Read();
        rvswclk_.Write(false);
        DelayHalf();
      }

      if (bit)
      {
        SetBit(out_lsb_first, i);
      }
    }

    return ErrorCode::OK;
  }

  ErrorCode RvSwdSeqWriteBits(uint32_t cycles, const uint8_t* data_lsb_first)
  {
    if (cycles == 0u)
    {
      rvswclk_.Write(false);
      return ErrorCode::OK;
    }
    if (data_lsb_first == nullptr)
    {
      return ErrorCode::ARG_ERR;
    }

    const ErrorCode ec = SetRvSwdioDriveMode();
    if (ec != ErrorCode::OK)
    {
      return ec;
    }

    rvswclk_.Write(false);
    for (uint32_t i = 0u; i < cycles; ++i)
    {
      const bool bit = (((data_lsb_first[i / 8u] >> (i & 7u)) & 0x01u) != 0u);
      rvswdio_.Write(bit);
      GenOneClk();
      rvswclk_.Write(false);
    }

    return ErrorCode::OK;
  }

  ErrorCode RvSwdWakePulse()
  {
    const ErrorCode ec = SetRvSwdioDriveMode();
    if (ec != ErrorCode::OK)
    {
      return ec;
    }

    rvswclk_.Write(true);
    rvswdio_.Write(true);
    DelayHalf();
    for (uint32_t i = 0u; i < 100u; ++i)
    {
      rvswclk_.Write(false);
      DelayHalf();
      rvswclk_.Write(true);
      DelayHalf();
    }

    rvswdio_.Write(false);
    DelayHalf();
    rvswdio_.Write(true);
    DelayHalf();
    return ErrorCode::OK;
  }

  ErrorCode RvSwdPrepareReadTurnaround()
  {
    const ErrorCode ec = SetRvSwdioSampleMode();
    if (ec != ErrorCode::OK)
    {
      return ec;
    }

    rvswclk_.Write(false);
    return ErrorCode::OK;
  }

  ErrorCode RvSwdStart()
  {
    const ErrorCode ec = SetRvSwdioDriveMode();
    if (ec != ErrorCode::OK)
    {
      return ec;
    }

    rvswdio_.Write(true);
    rvswclk_.Write(true);
    DelayHalf();
    rvswdio_.Write(false);
    DelayHalf();
    rvswclk_.Write(false);
    return ErrorCode::OK;
  }

  ErrorCode RvSwdStop()
  {
    const ErrorCode ec = SetRvSwdioDriveMode();
    if (ec != ErrorCode::OK)
    {
      return ec;
    }

    rvswdio_.Write(false);
    rvswclk_.Write(false);
    DelayHalf();
    rvswclk_.Write(true);
    DelayHalf();
    rvswdio_.Write(true);
    DelayHalf();
    return ErrorCode::OK;
  }

  ErrorCode BeginRvSwdFrame() { return RvSwdStart(); }
  ErrorCode EndRvSwdFrame() { return RvSwdStop(); }

  ErrorCode SetRvSwdioDriveMode()
  {
    if (rvswdio_mode_ != RvSwdioMode::DRIVE)
    {
      const ErrorCode ec =
          rvswdio_.SetConfig({(IO_DRIVE_MODE == RvSwdioDriveMode::OPEN_DRAIN)
                                   ? RvSwdioGpioType::Direction::OUTPUT_OPEN_DRAIN
                                   : RvSwdioGpioType::Direction::OUTPUT_PUSH_PULL,
                               RvSwdioGpioType::Pull::NONE});
      if (ec != ErrorCode::OK)
      {
        return ec;
      }
    }

    rvswdio_mode_ = RvSwdioMode::DRIVE;
    return ErrorCode::OK;
  }

  ErrorCode SetRvSwdioSampleMode()
  {
    if (rvswdio_mode_ != RvSwdioMode::SAMPLE_IN)
    {
      const ErrorCode ec =
          rvswdio_.SetConfig({RvSwdioGpioType::Direction::INPUT, RvSwdioGpioType::Pull::UP});
      if (ec != ErrorCode::OK)
      {
        return ec;
      }
    }

    rvswdio_mode_ = RvSwdioMode::SAMPLE_IN;
    return ErrorCode::OK;
  }

  void DelayHalf() { BusyLoop(half_period_loops_); }

  void GenOneClk()
  {
    rvswclk_.Write(false);
    DelayHalf();
    rvswclk_.Write(true);
    DelayHalf();
  }

  static void BusyLoop(uint32_t loops)
  {
    volatile uint32_t sink = loops;
    while (sink-- > 0u)
    {
      __asm volatile("nop");
    }
  }

  static void ClearBits(uint8_t* data, uint32_t bits)
  {
    const uint32_t bytes = (bits + 7u) / 8u;
    for (uint32_t i = 0u; i < bytes; ++i)
    {
      data[i] = 0u;
    }
  }

  static void SetBit(uint8_t* data, uint32_t bit)
  {
    data[bit / 8u] = static_cast<uint8_t>(data[bit / 8u] | (1u << (bit & 7u)));
  }

  RvSwclkGpioType& rvswclk_;
  RvSwdioGpioType& rvswdio_;
  uint32_t loops_per_us_ = 0u;
  uint32_t clock_hz_ = 0u;
  uint32_t half_period_ns_ = 0u;
  uint32_t half_period_loops_ = 0u;
  RvSwdioMode rvswdio_mode_ = RvSwdioMode::UNKNOWN;
  bool online_ready_ = false;
  uint64_t last_host_frame_ = 0u;
  uint64_t last_target_frame_ = 0u;
  bool last_target_frame_valid_ = false;
  uint64_t last_target_raw43_ = 0u;
  bool last_target_raw43_valid_ = false;
  uint8_t last_target_frame_info_ = 0u;
  uint32_t last_online_capr_ = 0u;
  bool last_online_capr_valid_ = false;
  uint8_t last_enter_state_ = 0u;
  uint8_t last_online_shad_addr_ = 0u;
  uint8_t last_online_mask_addr_ = 0u;
  uint8_t last_online_capr_addr_ = 0u;
  uint8_t last_online_capr_attempt_mask_ = 0u;
  uint8_t last_online_capr_turn_bit_ = 0xFFu;
  uint32_t last_online_capr_turn_value_ = 0u;
  uint32_t last_online_capr_noturn_value_ = 0u;
  uint8_t bridge_stage_43_ = 0u;
  uint32_t bridge_scratch_payload_ = 0u;
};

}  // namespace LibXR::Debug

