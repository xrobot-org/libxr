/**
 * @file jtag_target.hpp
 * @brief 引脚级 JTAG 目标仿真：IEEE 1149.1 TAP + 多 TAP 菊花链 + ADIv5 JTAG-DP + AHB-AP
 *        Pin-level JTAG target simulation: IEEE 1149.1 TAP, multi-TAP daisy chain,
 *        ADIv5 JTAG-DP and AHB-AP.
 *
 * 复用 swd_target.hpp 的 `TargetMemory`：每个 TCK 上升沿采样 TMS 与 TDI、在
 * SHIFT_IR/SHIFT_DR 沿菊花链右移一位并按 IEEE 1149.1 推进 TAP 状态机；每个 TCK
 * 下降沿更新 TDO。支持单 TAP 与多 TAP 链、ADIv5 JTAG-DP（IDCODE、ABORT、DPACC、
 * APACC、CTRL/STAT W1C 粘性错误、WAIT 注入与总线错误注入）。
 * Reuses `TargetMemory` from swd_target.hpp: every TCK rising edge samples TMS and TDI,
 * shifts one bit along the daisy chain in SHIFT_IR/SHIFT_DR, and advances the IEEE
 * 1149.1 TAP FSM; every TCK falling edge updates TDO. Supports single and multi-TAP
 * chains and ADIv5 JTAG-DP (IDCODE, ABORT, DPACC, APACC, CTRL/STAT W1C sticky errors,
 * WAIT injection, and bus error injection).
 */

#pragma once

#include <cstdint>
#include <vector>

#include "debug/jtag.hpp"
#include "debug/jtag_protocol.hpp"
#include "debug/swd_protocol.hpp"
#include "gpio.hpp"
#include "swd_target.hpp"

namespace LibXR::Debug::Sim
{

/// ADIv5 JTAG-DP 与一个 AHB-AP / ADIv5 JTAG-DP with one AHB-AP
class JtagDebugPort
{
 public:
  static constexpr uint32_t JTAG_DP_IDCODE = 0x4BA00477u;
  static constexpr uint32_t AP_IDR = 0x24770011u;
  static constexpr uint32_t STICKYORUN = 1u << 1;
  static constexpr uint32_t STICKYCMP = 1u << 4;
  static constexpr uint32_t STICKYERR = 1u << 5;
  static constexpr uint32_t READOK = 1u << 6;

  static constexpr uint8_t RAW_ACK_WAIT = 0x1u;
  static constexpr uint8_t RAW_ACK_OK_FAULT = 0x2u;

  explicit JtagDebugPort(TargetMemory& memory) : memory_(memory) {}

  // ---- 故障注入 / Fault injection ----
  unsigned inject_wait = 0;       ///< 之后 N 次 AP 扫描回 WAIT / N WAITs
  unsigned inject_bus_error = 0;  ///< 之后 N 次 AP 传输出总线错误 / N bus errors
  /// 每 wait_every 次 AP 访问注入 wait_burst 个 WAIT（0 关闭）/ Every
  /// wait_every AP accesses inject wait_burst WAITs (0 turns it off)
  unsigned wait_every = 0;
  unsigned wait_burst = 1;

  uint32_t CtrlStat() const
  {
    uint32_t v = ctrl_ & (SwdProtocol::DP_CTRLSTAT_CDBGPWRUPREQ |
                          SwdProtocol::DP_CTRLSTAT_CSYSPWRUPREQ | 1u);
    if (v & SwdProtocol::DP_CTRLSTAT_CDBGPWRUPREQ)
    {
      v |= SwdProtocol::DP_CTRLSTAT_CDBGPWRUPACK;
    }
    if (v & SwdProtocol::DP_CTRLSTAT_CSYSPWRUPREQ)
    {
      v |= SwdProtocol::DP_CTRLSTAT_CSYSPWRUPACK;
    }
    return v | sticky_ | (read_ok_ ? READOK : 0u);
  }

  /// CTRL/STAT.ORUNDETECT：WAIT 期间置 STICKYORUN / Set STICKYORUN on WAIT
  bool OverrunDetect() const { return (ctrl_ & 1u) != 0u; }

  /// TEST_LOGIC_RESET 时复位 DP 挂起状态 / Reset transient DP state on TLR
  void ResetTapState()
  {
    captured_wait_ = false;
    next_raw_ack_ = RAW_ACK_OK_FAULT;
    pending_rdata_ = 0u;
    queued_rdata_ = 0u;
    read_ok_ = false;
  }

  /// CAPTURE_DR（DPACC/APACC）：载入上一笔请求的 35 位响应 / Load 35-bit response
  uint64_t CaptureAcc()
  {
    captured_wait_ = (next_raw_ack_ == RAW_ACK_WAIT);
    if (captured_wait_ && OverrunDetect())
    {
      sticky_ |= STICKYORUN;
    }
    return (static_cast<uint64_t>(pending_rdata_) << 3) |
           static_cast<uint64_t>(next_raw_ack_ & 0x7u);
  }

  /// UPDATE_DR（ABORT）：bit0=DAPABORT 中止当前等待中的请求 / Abort pending request
  void UpdateAbort(uint64_t dr_bits)
  {
    const uint32_t wdata = static_cast<uint32_t>(dr_bits >> 3);
    if ((wdata & 0x1u) != 0u)
    {
      inject_wait = 0u;
      captured_wait_ = false;
      next_raw_ack_ = RAW_ACK_OK_FAULT;
      pending_rdata_ = queued_rdata_;
    }
  }

  /// UPDATE_DR（DPACC/APACC）：若本轮未回 WAIT，执行新请求 / Execute request unless WAIT
  void UpdateAcc(bool ap, uint64_t dr_bits)
  {
    const bool rnw = (dr_bits & 1u) != 0u;
    const uint8_t a32 = static_cast<uint8_t>((dr_bits >> 1) & 3u);
    const uint32_t wdata = static_cast<uint32_t>(dr_bits >> 3);
    const bool rdbuff = !ap && rnw && (a32 == 3u);

    if (captured_wait_)
    {
      captured_wait_ = false;
      if (!ap && !rnw && a32 == 1u)
      {
        const uint32_t clear_mask = wdata & (STICKYORUN | STICKYCMP | STICKYERR);
        sticky_ &= ~clear_mask;
        ctrl_ = wdata & ~(STICKYORUN | STICKYCMP | STICKYERR | READOK |
                          SwdProtocol::DP_CTRLSTAT_CDBGPWRUPACK |
                          SwdProtocol::DP_CTRLSTAT_CSYSPWRUPACK);
        read_ok_ = true;
      }
      if (inject_wait > 0u)
      {
        --inject_wait;
      }
      if (inject_wait == 0u)
      {
        next_raw_ack_ = RAW_ACK_OK_FAULT;
        pending_rdata_ = queued_rdata_;
      }
      return;
    }

    uint32_t result_data = pending_rdata_;
    if (!ap)
    {
      if (!rnw)
      {
        switch (a32)
        {
          case 1:
          {
            const uint32_t clear_mask = wdata & (STICKYORUN | STICKYCMP | STICKYERR);
            sticky_ &= ~clear_mask;
            ctrl_ = wdata & ~(STICKYORUN | STICKYCMP | STICKYERR | READOK |
                              SwdProtocol::DP_CTRLSTAT_CDBGPWRUPACK |
                              SwdProtocol::DP_CTRLSTAT_CSYSPWRUPACK);
            read_ok_ = true;
            break;
          }
          case 2:
            select_ = wdata;
            read_ok_ = true;
            break;
          default:
            break;
        }
      }
      else
      {
        switch (a32)
        {
          case 0:
            result_data = 0u;
            read_ok_ = true;
            break;
          case 1:
            result_data = CtrlStat();
            read_ok_ = true;
            break;
          case 2:
            result_data = select_;
            read_ok_ = true;
            break;
          case 3:
          default:
            // RDBUFF 返回上一次缓冲值，不发起新的 AP 访问 / Returns last value
            result_data = pending_rdata_;
            break;
        }
      }
    }
    else if ((sticky_ & (STICKYERR | STICKYORUN | STICKYCMP)) == 0u)
    {
      if (wait_every != 0u && ((++ap_accesses_ % wait_every) == 0u))
      {
        inject_wait += wait_burst;
      }
      if (rnw)
      {
        read_ok_ = ApRead(a32, result_data);
      }
      else
      {
        ApWrite(a32, wdata);
        result_data = 0u;
      }
    }
    else
    {
      // 粘性错误期间忽略 AP 访问 / AP accesses ignored while sticky error is set
      read_ok_ = false;
      result_data = 0u;
    }

    if (inject_wait != 0u)
    {
      queued_rdata_ = result_data;
      pending_rdata_ = 0u;
      next_raw_ack_ = RAW_ACK_WAIT;
    }
    else
    {
      queued_rdata_ = result_data;
      pending_rdata_ = result_data;
      next_raw_ack_ = RAW_ACK_OK_FAULT;
    }
  }

  uint32_t Select() const { return select_; }
  uint32_t Csw() const { return csw_; }
  uint32_t Tar() const { return tar_; }

 private:
  bool ApRead(uint8_t a32, uint32_t& value)
  {
    if ((select_ >> 24) != 0u)
    {
      value = 0u;
      return true;
    }
    const uint32_t reg = (select_ & 0xF0u) | (static_cast<uint32_t>(a32) << 2);
    switch (reg)
    {
      case 0x00:
        value = (csw_ & ~0x30u) | ((csw_ & 0x30u) == 0x10u ? 0x10u : 0u) | 0x40u;
        return true;
      case 0x04:
        value = tar_;
        return true;
      case 0x0C:
        return Transfer(true, tar_, value);
      case 0x10:
      case 0x14:
      case 0x18:
      case 0x1C:
      {
        const uint32_t addr = (tar_ & ~0xFu) | (reg & 0xCu);
        return Transfer(true, addr, value, false);
      }
      case 0xF4:
        value = 0u;
        return true;
      case 0xF8:
        value = TargetMemory::ROM_TABLE | 3u;
        return true;
      case 0xFC:
        value = AP_IDR;
        return true;
      default:
        value = 0u;
        return true;
    }
  }

  void ApWrite(uint8_t a32, uint32_t value)
  {
    if ((select_ >> 24) != 0u)
    {
      return;
    }
    const uint32_t reg = (select_ & 0xF0u) | (static_cast<uint32_t>(a32) << 2);
    uint32_t dummy = value;
    switch (reg)
    {
      case 0x00:
        csw_ = value;
        break;
      case 0x04:
        tar_ = value;
        break;
      case 0x0C:
        (void)Transfer(false, tar_, dummy);
        break;
      case 0x10:
      case 0x14:
      case 0x18:
      case 0x1C:
        (void)Transfer(false, (tar_ & ~0xFu) | (reg & 0xCu), dummy, false);
        break;
      default:
        break;
    }
  }

  bool Transfer(bool read, uint32_t addr, uint32_t& value, bool use_tar = true)
  {
    const uint8_t size = use_tar ? static_cast<uint8_t>(csw_ & 7u) : 2u;
    bool ok = false;
    if (inject_bus_error != 0u)
    {
      --inject_bus_error;
      ok = false;
    }
    else
    {
      ok = read ? memory_.Read(addr, size, value) : memory_.Write(addr, size, value);
    }
    if (!ok)
    {
      sticky_ |= STICKYERR;
      return false;
    }
    if (use_tar && (csw_ & 0x30u) == 0x10u)
    {
      tar_ = (tar_ & ~0xFFFu) | ((tar_ + (1u << size)) & 0xFFFu);
    }
    return true;
  }

  TargetMemory& memory_;
  uint32_t ctrl_ = 0u;
  uint32_t select_ = 0u;
  uint32_t sticky_ = 0u;
  uint32_t csw_ = 0x00000002u;
  uint32_t tar_ = 0u;
  uint32_t pending_rdata_ = 0u;
  uint32_t queued_rdata_ = 0u;
  uint8_t next_raw_ack_ = RAW_ACK_OK_FAULT;
  unsigned ap_accesses_ = 0u;
  bool read_ok_ = false;
  bool captured_wait_ = false;
};

/// 链上一个 TAP 的配置与运行状态 / Specification and state of one TAP on the chain
struct JtagTapSpec
{
  uint8_t ir_len = 4u;
  uint32_t idcode = JtagDebugPort::JTAG_DP_IDCODE;
  uint32_t idcode_ir = JtagProtocol::JTAG_IR_IDCODE;
  bool has_dp = true;
};

/// IEEE 1149.1 线协议目标端（支持多 TAP 菊花链）/ Pin-level JTAG chain target
class JtagWireTarget
{
 public:
  JtagWireTarget(JtagDebugPort& dp, std::vector<JtagTapSpec> specs) : dp_(dp)
  {
    if (specs.empty())
    {
      specs.push_back(JtagTapSpec{});
    }
    taps_.reserve(specs.size());
    for (const JtagTapSpec& s : specs)
    {
      TapNode node{};
      node.spec = s;
      node.bypass_ir = (s.ir_len >= 32u) ? UINT32_MAX : ((1u << s.ir_len) - 1u);
      node.ir = s.idcode_ir;
      node.dp = s.has_dp ? &dp_ : nullptr;
      taps_.push_back(node);
    }
  }

  bool disconnected = false;
  unsigned protocol_errors = 0u;
  unsigned line_resets = 0u;
  unsigned tck_rising_edges = 0u;
  const unsigned contention = 0u;  ///< JTAG TDO 独立，无竞争 / Dedicated TDO: zero

  void SetTms(bool v) { tms_ = v; }
  void SetTdi(bool v) { tdi_ = v; }
  void SetTck(bool v)
  {
    if (v && !tck_)
    {
      tck_ = true;
      OnRisingEdge();
    }
    else if (!v && tck_)
    {
      tck_ = false;
      OnFallingEdge();
    }
  }
  bool Tdo() const { return disconnected ? true : tdo_; }
  TapState State() const { return state_; }

 private:
  struct TapNode
  {
    JtagTapSpec spec{};
    uint32_t bypass_ir = 0xFu;
    uint32_t ir = 0xEu;
    uint32_t active_len = 32u;
    uint64_t shift_reg = 0u;
    JtagDebugPort* dp = nullptr;
  };

  static TapState NextState(TapState s, bool tms)
  {
    switch (s)
    {
      case TapState::TEST_LOGIC_RESET:
        return tms ? TapState::TEST_LOGIC_RESET : TapState::RUN_TEST_IDLE;
      case TapState::RUN_TEST_IDLE:
        return tms ? TapState::SELECT_DR_SCAN : TapState::RUN_TEST_IDLE;
      case TapState::SELECT_DR_SCAN:
        return tms ? TapState::SELECT_IR_SCAN : TapState::CAPTURE_DR;
      case TapState::CAPTURE_DR:
        return tms ? TapState::EXIT1_DR : TapState::SHIFT_DR;
      case TapState::SHIFT_DR:
        return tms ? TapState::EXIT1_DR : TapState::SHIFT_DR;
      case TapState::EXIT1_DR:
        return tms ? TapState::UPDATE_DR : TapState::PAUSE_DR;
      case TapState::PAUSE_DR:
        return tms ? TapState::EXIT2_DR : TapState::PAUSE_DR;
      case TapState::EXIT2_DR:
        return tms ? TapState::UPDATE_DR : TapState::SHIFT_DR;
      case TapState::UPDATE_DR:
        return tms ? TapState::SELECT_DR_SCAN : TapState::RUN_TEST_IDLE;
      case TapState::SELECT_IR_SCAN:
        return tms ? TapState::TEST_LOGIC_RESET : TapState::CAPTURE_IR;
      case TapState::CAPTURE_IR:
        return tms ? TapState::EXIT1_IR : TapState::SHIFT_IR;
      case TapState::SHIFT_IR:
        return tms ? TapState::EXIT1_IR : TapState::SHIFT_IR;
      case TapState::EXIT1_IR:
        return tms ? TapState::UPDATE_IR : TapState::PAUSE_IR;
      case TapState::PAUSE_IR:
        return tms ? TapState::EXIT2_IR : TapState::PAUSE_IR;
      case TapState::EXIT2_IR:
        return tms ? TapState::UPDATE_IR : TapState::SHIFT_IR;
      case TapState::UPDATE_IR:
        return tms ? TapState::SELECT_DR_SCAN : TapState::RUN_TEST_IDLE;
    }
    return TapState::TEST_LOGIC_RESET;
  }

  static uint64_t MaskBits(uint32_t bits)
  {
    return (bits >= 64u) ? UINT64_MAX : ((uint64_t{1} << bits) - 1u);
  }

  void ShiftChain()
  {
    const size_t n = taps_.size();
    bool feed = tdi_;
    // 从靠近 TDI 的 taps_[n-1] 依次移入靠近 TDO 的 taps_[0] / Shift from TDI end to TDO
    for (size_t idx = n; idx > 0u; --idx)
    {
      TapNode& tap = taps_[idx - 1u];
      const bool out_bit = (tap.shift_reg & 1u) != 0u;
      const uint32_t len = (tap.active_len == 0u) ? 1u : tap.active_len;
      tap.shift_reg = (tap.shift_reg >> 1) | (feed ? (uint64_t{1} << (len - 1u)) : 0u);
      tap.shift_reg &= MaskBits(len);
      feed = out_bit;
    }
  }

  void OnCaptureIr()
  {
    shift_bits_ = 0u;
    for (TapNode& tap : taps_)
    {
      tap.active_len = tap.spec.ir_len;
      // IEEE 1149.1: CAPTURE-IR 固定装入 bit0=1, bit1=0 / CAPTURE-IR loads b...01
      tap.shift_reg = 0x1u & MaskBits(tap.active_len);
    }
  }

  void OnCaptureDr()
  {
    shift_bits_ = 0u;
    chain_dr_bits_ = 0u;
    for (TapNode& tap : taps_)
    {
      const uint32_t ir = tap.ir;
      if (ir == tap.spec.idcode_ir)
      {
        tap.active_len = 32u;
        tap.shift_reg = tap.spec.idcode;
      }
      else if (tap.dp != nullptr && ir == JtagProtocol::JTAG_IR_ABORT)
      {
        tap.active_len = JtagProtocol::JTAG_DP_DR_LEN;
        tap.shift_reg = 0u;
      }
      else if (tap.dp != nullptr &&
               (ir == JtagProtocol::JTAG_IR_DPACC || ir == JtagProtocol::JTAG_IR_APACC))
      {
        tap.active_len = JtagProtocol::JTAG_DP_DR_LEN;
        tap.shift_reg = tap.dp->CaptureAcc();
      }
      else
      {
        // BYPASS 或未实现指令 / BYPASS or unimplemented instruction
        tap.active_len = 1u;
        tap.shift_reg = 0u;
      }
      chain_dr_bits_ += tap.active_len;
    }
  }

  void OnUpdateIr()
  {
    for (TapNode& tap : taps_)
    {
      tap.ir = static_cast<uint32_t>(tap.shift_reg & MaskBits(tap.spec.ir_len));
    }
  }

  void OnUpdateDr()
  {
    for (TapNode& tap : taps_)
    {
      if (tap.dp == nullptr)
      {
        continue;
      }
      if (tap.ir == JtagProtocol::JTAG_IR_DPACC || tap.ir == JtagProtocol::JTAG_IR_APACC)
      {
        if (shift_bits_ != chain_dr_bits_)
        {
          ++protocol_errors;
          continue;
        }
        tap.dp->UpdateAcc(tap.ir == JtagProtocol::JTAG_IR_APACC, tap.shift_reg);
      }
      else if (tap.ir == JtagProtocol::JTAG_IR_ABORT)
      {
        if (shift_bits_ != chain_dr_bits_)
        {
          ++protocol_errors;
          continue;
        }
        tap.dp->UpdateAbort(tap.shift_reg);
      }
    }
  }

  void OnResetTap()
  {
    ++line_resets;
    for (TapNode& tap : taps_)
    {
      tap.ir = tap.spec.idcode_ir;
      if (tap.dp != nullptr)
      {
        tap.dp->ResetTapState();
      }
    }
  }

  void OnRisingEdge()
  {
    ++tck_rising_edges;
    if (state_ == TapState::SHIFT_IR || state_ == TapState::SHIFT_DR)
    {
      ShiftChain();
      ++shift_bits_;
    }

    const TapState next = NextState(state_, tms_);
    const bool entered_tlr =
        (next == TapState::TEST_LOGIC_RESET && state_ != TapState::TEST_LOGIC_RESET);
    state_ = next;

    switch (state_)
    {
      case TapState::TEST_LOGIC_RESET:
        if (entered_tlr)
        {
          OnResetTap();
        }
        break;
      case TapState::CAPTURE_IR:
        OnCaptureIr();
        break;
      case TapState::CAPTURE_DR:
        OnCaptureDr();
        break;
      case TapState::UPDATE_IR:
        OnUpdateIr();
        break;
      case TapState::UPDATE_DR:
        OnUpdateDr();
        break;
      default:
        break;
    }
  }

  void OnFallingEdge()
  {
    if (state_ == TapState::SHIFT_IR || state_ == TapState::SHIFT_DR)
    {
      tdo_ = (taps_[0].shift_reg & 1u) != 0u;
    }
    else
    {
      tdo_ = false;
    }
  }

  JtagDebugPort& dp_;
  std::vector<TapNode> taps_;
  TapState state_ = TapState::TEST_LOGIC_RESET;
  bool tck_ = false;
  bool tms_ = true;
  bool tdi_ = false;
  bool tdo_ = false;
  uint32_t shift_bits_ = 0u;
  uint32_t chain_dr_bits_ = 32u;
};

/// 完整 JTAG 目标（内存 + JTAG-DP/AHB-AP + 线协议链路）/ Complete JTAG target
struct JtagTarget
{
  explicit JtagTarget(std::vector<JtagTapSpec> chain = {JtagTapSpec{}})
      : dp(memory), wire(dp, std::move(chain))
  {
  }

  TargetMemory memory;
  JtagDebugPort dp;
  JtagWireTarget wire;
};

/// TCK 引脚（驱动 JtagWireTarget）/ TCK pin driving JtagWireTarget
class SimTckPin final : public LibXR::GPIO
{
 public:
  explicit SimTckPin(JtagWireTarget& wire) : wire_(wire) {}
  bool Read() override { return level_; }
  void Write(bool value) override
  {
    level_ = value;
    wire_.SetTck(value);
  }
  ErrorCode EnableInterrupt() override { return ErrorCode::OK; }
  ErrorCode DisableInterrupt() override { return ErrorCode::OK; }
  ErrorCode SetConfig(Configuration) override { return ErrorCode::OK; }

 private:
  JtagWireTarget& wire_;
  bool level_ = false;
};

/// TMS 引脚（驱动 JtagWireTarget）/ TMS pin driving JtagWireTarget
class SimTmsPin final : public LibXR::GPIO
{
 public:
  explicit SimTmsPin(JtagWireTarget& wire) : wire_(wire) {}
  bool Read() override { return level_; }
  void Write(bool value) override
  {
    level_ = value;
    wire_.SetTms(value);
  }
  ErrorCode EnableInterrupt() override { return ErrorCode::OK; }
  ErrorCode DisableInterrupt() override { return ErrorCode::OK; }
  ErrorCode SetConfig(Configuration) override { return ErrorCode::OK; }

 private:
  JtagWireTarget& wire_;
  bool level_ = true;
};

/// TDI 引脚（驱动 JtagWireTarget）/ TDI pin driving JtagWireTarget
class SimTdiPin final : public LibXR::GPIO
{
 public:
  explicit SimTdiPin(JtagWireTarget& wire) : wire_(wire) {}
  bool Read() override { return level_; }
  void Write(bool value) override
  {
    level_ = value;
    wire_.SetTdi(value);
  }
  ErrorCode EnableInterrupt() override { return ErrorCode::OK; }
  ErrorCode DisableInterrupt() override { return ErrorCode::OK; }
  ErrorCode SetConfig(Configuration) override { return ErrorCode::OK; }

 private:
  JtagWireTarget& wire_;
  bool level_ = false;
};

/// TDO 引脚（读取 JtagWireTarget）/ TDO pin reading JtagWireTarget
class SimTdoPin final : public LibXR::GPIO
{
 public:
  explicit SimTdoPin(JtagWireTarget& wire) : wire_(wire) {}
  bool Read() override { return wire_.Tdo(); }
  void Write(bool) override {}
  ErrorCode EnableInterrupt() override { return ErrorCode::OK; }
  ErrorCode DisableInterrupt() override { return ErrorCode::OK; }
  ErrorCode SetConfig(Configuration) override { return ErrorCode::OK; }

 private:
  JtagWireTarget& wire_;
};

}  // namespace LibXR::Debug::Sim
