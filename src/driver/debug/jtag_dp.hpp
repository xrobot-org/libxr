// jtag_dp.hpp
#pragma once

#include <cstdint>

#include "jtag.hpp"
#include "jtag_protocol.hpp"
#include "libxr_def.hpp"

namespace LibXR::Debug
{
/**
 * @brief JTAG-DP 事务层封装（DP/AP 访问与 IDCODE/ABORT）。
 *
 * 说明：
 * - JTAG-DP 读为管线化：读请求返回上一笔结果。
 * - 本类提供 DpReadTxn/ApReadTxn 等“带取数”的事务方法。
 */
class JtagDp final
{
 public:
  explicit JtagDp(Jtag& jtag, JtagProtocol::ChainConfig* chain = nullptr)
      : jtag_(jtag), chain_(chain)
  {
  }

  void SetChainConfig(JtagProtocol::ChainConfig* chain)
  {
    chain_ = chain;
    current_ir_valid_ = false;
  }

  ErrorCode DpRead(uint8_t addr2b, uint32_t& val, JtagProtocol::Ack& ack)
  {
    JtagProtocol::Response resp;
    const ErrorCode EC = Transfer(JtagProtocol::MakeDpRequest(true, addr2b, 0u), resp);
    ack = resp.ack;
    val = resp.rdata;
    return EC;
  }

  ErrorCode DpWrite(uint8_t addr2b, uint32_t val, JtagProtocol::Ack& ack)
  {
    JtagProtocol::Response resp;
    const ErrorCode EC = Transfer(JtagProtocol::MakeDpRequest(false, addr2b, val), resp);
    ack = resp.ack;
    return EC;
  }

  ErrorCode DpReadTxn(uint8_t addr2b, uint32_t& val, JtagProtocol::Ack& ack)
  {
    JtagProtocol::Response resp;
    ErrorCode EC = Transfer(JtagProtocol::MakeDpRequest(true, addr2b, 0u), resp);
    ack = resp.ack;
    if (EC != ErrorCode::OK)
    {
      return EC;
    }
    if (resp.ack != JtagProtocol::Ack::OK)
    {
      return ErrorCode::OK;
    }
    // 读为管线化：第二次读取 RDBUFF 拿到结果
    EC = Transfer(JtagProtocol::MakeDpRequest(true, 3u, 0u), resp);
    ack = resp.ack;
    if (EC != ErrorCode::OK || resp.ack != JtagProtocol::Ack::OK)
    {
      return EC;
    }
    val = resp.rdata;

    if ((addr2b & 0x03u) == 0u && val == 0u)
    {
      uint32_t idcode = 0u;
      const ErrorCode id_ec = ReadIdCode(idcode);
      if (id_ec == ErrorCode::OK && idcode != 0u && idcode != 0xFFFF'FFFFu)
      {
        val = idcode;
        ack = JtagProtocol::Ack::OK;
      }
    }
    return ErrorCode::OK;
  }

  ErrorCode DpWriteTxn(uint8_t addr2b, uint32_t val, JtagProtocol::Ack& ack)
  {
    if ((addr2b & 0x03u) == 0u)
    {
      return WriteAbort(val, ack);
    }

    return DpWrite(addr2b, val, ack);
  }

  ErrorCode ApReadTxn(uint8_t addr2b, uint32_t& val, JtagProtocol::Ack& ack)
  {
    JtagProtocol::Response resp;
    ErrorCode EC = Transfer(JtagProtocol::MakeApRequest(true, addr2b, 0u), resp);
    ack = resp.ack;
    if (EC != ErrorCode::OK)
    {
      return EC;
    }
    if (resp.ack != JtagProtocol::Ack::OK)
    {
      return ErrorCode::OK;
    }
    // AP 读同样是 posted：读 RDBUFF 获取实际数据
    EC = Transfer(JtagProtocol::MakeDpRequest(true, 3u, 0u), resp);
    ack = resp.ack;
    if (EC != ErrorCode::OK || resp.ack != JtagProtocol::Ack::OK)
    {
      return EC;
    }
    val = resp.rdata;
    return ErrorCode::OK;
  }

  ErrorCode ApWriteTxn(uint8_t addr2b, uint32_t val, JtagProtocol::Ack& ack)
  {
    JtagProtocol::Response resp;
    const ErrorCode EC = Transfer(JtagProtocol::MakeApRequest(false, addr2b, val), resp);
    ack = resp.ack;
    return EC;
  }

  ErrorCode ReadIdCode(uint32_t& idcode)
  {
    return ReadIdCodeWithIr(JtagProtocol::JTAG_IR_IDCODE, idcode);
  }

  ErrorCode ReadIdCodeWithIr(uint32_t ir, uint32_t& idcode)
  {
    const ErrorCode EC = SelectIr(ir);
    if (EC != ErrorCode::OK)
    {
      return EC;
    }

    uint64_t out_dr = 0u;
    const ErrorCode EC2 = ShiftValueThroughChain(0u, 32u, out_dr);
    if (EC2 != ErrorCode::OK)
    {
      return EC2;
    }
    idcode = static_cast<uint32_t>(out_dr & 0xFFFF'FFFFu);
    return ErrorCode::OK;
  }

  ErrorCode WriteAbort(uint32_t data, JtagProtocol::Ack& ack)
  {
    const ErrorCode EC = SelectIr(JtagProtocol::JTAG_IR_ABORT);
    if (EC != ErrorCode::OK)
    {
      ack = JtagProtocol::Ack::PROTOCOL;
      return EC;
    }

    uint64_t out_dr = 0u;
    const ErrorCode EC2 = ShiftValueThroughChain(
        JtagProtocol::PackDpDr(JtagProtocol::MakeDpRequest(false, 0u, data)),
        JtagProtocol::JTAG_DP_DR_LEN, out_dr);
    if (EC2 != ErrorCode::OK)
    {
      ack = JtagProtocol::Ack::PROTOCOL;
      return EC2;
    }

    (void)out_dr;
    // JTAG ABORT uses its own scan chain; captured TDO bits are not a
    // DPACC/APACC ACK field. A completed shift means the ABORT was issued.
    ack = JtagProtocol::Ack::OK;
    return ErrorCode::OK;
  }

  ErrorCode Transfer(const JtagProtocol::Request& req, JtagProtocol::Response& resp)
  {
    const ErrorCode EC =
        SelectIr(req.port == JtagProtocol::Port::AP ? JtagProtocol::JTAG_IR_APACC
                                                    : JtagProtocol::JTAG_IR_DPACC);
    if (EC != ErrorCode::OK)
    {
      resp.ack = JtagProtocol::Ack::PROTOCOL;
      return EC;
    }

    uint64_t out_dr = 0u;
    const ErrorCode EC2 = ShiftValueThroughChain(JtagProtocol::PackDpDr(req),
                                                 JtagProtocol::JTAG_DP_DR_LEN, out_dr);
    if (EC2 != ErrorCode::OK)
    {
      resp.ack = JtagProtocol::Ack::PROTOCOL;
      return EC2;
    }

    JtagProtocol::UnpackDpDr(out_dr, resp);
    return ErrorCode::OK;
  }

 private:
  static constexpr uint32_t DEFAULT_IR_BITS = 4u;
  static constexpr uint32_t MAX_BITS = 512u;
  static constexpr uint32_t MAX_BYTES = (MAX_BITS + 7u) / 8u;

  uint32_t IrBits() const
  {
    if (chain_ != nullptr && chain_->ir_length != nullptr && chain_->count > 0 &&
        chain_->index < chain_->count)
    {
      const uint8_t bits = chain_->ir_length[chain_->index];
      return (bits == 0u) ? DEFAULT_IR_BITS : bits;
    }
    return DEFAULT_IR_BITS;
  }

  uint32_t IrBeforeBits()
  {
    if (chain_ == nullptr)
    {
      return 0u;
    }
    if (chain_->ir_before_bits_len == 0u && chain_->ir_after_bits_len == 0u)
    {
      JtagProtocol::UpdateChainCache(*chain_);
    }
    return chain_->ir_before_bits_len;
  }

  uint32_t IrAfterBits()
  {
    if (chain_ == nullptr)
    {
      return 0u;
    }
    if (chain_->ir_before_bits_len == 0u && chain_->ir_after_bits_len == 0u)
    {
      JtagProtocol::UpdateChainCache(*chain_);
    }
    return chain_->ir_after_bits_len;
  }

  uint32_t DrBeforeBits()
  {
    if (chain_ == nullptr)
    {
      return 0u;
    }
    return (chain_->count == 0 || chain_->index >= chain_->count)
               ? 0u
               : static_cast<uint32_t>(chain_->index);
  }

  uint32_t DrAfterBits()
  {
    if (chain_ == nullptr)
    {
      return 0u;
    }
    return (chain_->count == 0 || chain_->index >= chain_->count)
               ? 0u
               : static_cast<uint32_t>(chain_->count - chain_->index - 1u);
  }

  ErrorCode SelectIr(uint32_t ir)
  {
    if (current_ir_valid_ && current_ir_ == ir)
    {
      return ErrorCode::OK;
    }

    const uint32_t ir_bits = IrBits();
    const uint32_t pre = IrBeforeBits();
    const uint32_t post = IrAfterBits();
    const uint32_t total_bits = pre + ir_bits + post;

    if (total_bits > MAX_BITS)
    {
      return ErrorCode::SIZE_ERR;
    }

    uint8_t buf[MAX_BYTES] = {};
    PutOnes(buf, 0u, pre);
    PutBits(buf, pre, ir, ir_bits);
    PutOnes(buf, pre + ir_bits, post);
    const ErrorCode EC = jtag_.ShiftIR(total_bits, buf, nullptr);
    if (EC == ErrorCode::OK)
    {
      current_ir_ = ir;
      current_ir_valid_ = true;
    }
    return EC;
  }

  ErrorCode ShiftValueThroughChain(uint64_t in_value, uint32_t value_bits,
                                   uint64_t& out_value)
  {
    const uint32_t pre = DrBeforeBits();
    const uint32_t post = DrAfterBits();
    const uint32_t total_bits = pre + value_bits + post;
    if (total_bits > MAX_BITS)
    {
      return ErrorCode::SIZE_ERR;
    }

    uint8_t tx[MAX_BYTES] = {};
    uint8_t rx[MAX_BYTES] = {};
    PutOnes(tx, 0u, pre);
    PutBits64(tx, pre, in_value, value_bits);
    PutOnes(tx, pre + value_bits, post);

    const ErrorCode EC = jtag_.ShiftDR(total_bits, tx, rx);
    if (EC != ErrorCode::OK)
    {
      return EC;
    }

    out_value = GetBits64(rx, pre, value_bits);
    return ErrorCode::OK;
  }

  static void PutOnes(uint8_t* buf, uint32_t bit_off, uint32_t bits)
  {
    for (uint32_t i = 0; i < bits; ++i)
    {
      const uint32_t idx = bit_off + i;
      const uint32_t byte = idx / 8u;
      const uint32_t shift = idx & 7u;
      buf[byte] = static_cast<uint8_t>(buf[byte] | (1u << shift));
    }
  }

  static void PutBits(uint8_t* buf, uint32_t bit_off, uint32_t value, uint32_t bits)
  {
    for (uint32_t i = 0; i < bits; ++i)
    {
      const bool bit = ((value >> i) & 0x1u) != 0u;
      const uint32_t idx = bit_off + i;
      const uint32_t byte = idx / 8u;
      const uint32_t shift = idx & 7u;
      if (bit)
      {
        buf[byte] = static_cast<uint8_t>(buf[byte] | (1u << shift));
      }
      else
      {
        buf[byte] = static_cast<uint8_t>(buf[byte] & ~(1u << shift));
      }
    }
  }

  static void PutBits64(uint8_t* buf, uint32_t bit_off, uint64_t value, uint32_t bits)
  {
    for (uint32_t i = 0; i < bits; ++i)
    {
      const bool bit = ((value >> i) & 0x1u) != 0u;
      const uint32_t idx = bit_off + i;
      const uint32_t byte = idx / 8u;
      const uint32_t shift = idx & 7u;
      if (bit)
      {
        buf[byte] = static_cast<uint8_t>(buf[byte] | (1u << shift));
      }
      else
      {
        buf[byte] = static_cast<uint8_t>(buf[byte] & ~(1u << shift));
      }
    }
  }

  static uint64_t GetBits64(const uint8_t* buf, uint32_t bit_off, uint32_t bits)
  {
    uint64_t v = 0u;
    for (uint32_t i = 0; i < bits; ++i)
    {
      const uint32_t idx = bit_off + i;
      const uint32_t byte = idx / 8u;
      const uint32_t shift = idx & 7u;
      const uint64_t bit = (buf[byte] >> shift) & 0x1u;
      v |= (bit << i);
    }
    return v;
  }

 private:
  Jtag& jtag_;
  JtagProtocol::ChainConfig* chain_ = nullptr;
  uint32_t current_ir_ = 0u;
  bool current_ir_valid_ = false;
};

}  // namespace LibXR::Debug
