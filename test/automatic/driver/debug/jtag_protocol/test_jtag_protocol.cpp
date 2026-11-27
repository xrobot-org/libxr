/**
 * @file test_jtag_protocol.cpp
 * @brief JTAG 协议打包、ACK 映射与链缓存主机测试 / Host tests for JTAG protocol packing,
 *        ACK mapping and chain cache.
 */

#include <cstdint>
#include <initializer_list>

#include "debug/jtag_protocol.hpp"
#include "test_assert.hpp"

using namespace LibXR::Debug::JtagProtocol;

namespace
{
void TestAckMapping()
{
  // JTAG-DP 原始 3 位编码：0x0=NO_ACK、0x1=WAIT、0x2=OK、0x4=FAULT、其余为 PROTOCOL /
  // JTAG-DP raw 3-bit encoding: 0x0=NO_ACK, 0x1=WAIT, 0x2=OK, 0x4=FAULT, others PROTOCOL.
  TEST_ASSERT(MapJtagAck(0x0u) == Ack::NO_ACK);
  TEST_ASSERT(MapJtagAck(0x1u) == Ack::WAIT);
  TEST_ASSERT(MapJtagAck(0x2u) == Ack::OK);
  TEST_ASSERT(MapJtagAck(0x4u) == Ack::FAULT);
  for (uint8_t raw : {0x3u, 0x5u, 0x6u, 0x7u})
  {
    TEST_ASSERT(MapJtagAck(raw) == Ack::PROTOCOL);
  }
  // 只看低 3 位 / Only the low 3 bits are used.
  TEST_ASSERT(MapJtagAck(0xFAu) == Ack::OK);

  static_assert(MapJtagAck(0x2u) == Ack::OK, "MapJtagAck works in constant expressions");
  static_assert(MapJtagAck(0x1u) == Ack::WAIT,
                "MapJtagAck WAIT works in constant expressions");
}

void TestPackAndUnpackDpDr()
{
  const Request wr = MakeApRequest(false, 2u, 0xA55A1234u);
  TEST_ASSERT(wr.port == Port::AP && !wr.rnw && wr.addr2b == 2u &&
              wr.wdata == 0xA55A1234u);
  const uint64_t packed_wr = PackDpDr(wr);
  // bit0=RnW(0), bit1=A2(0), bit2=A3(1), bits3..34=wdata
  TEST_ASSERT((packed_wr & 0x7u) == 0x4u);
  TEST_ASSERT((packed_wr >> 3) == 0xA55A1234u);

  const Request rd = MakeDpRequest(true, 3u, 0xDEADBEEFu);
  TEST_ASSERT(rd.port == Port::DP && rd.rnw && rd.addr2b == 3u);
  const uint64_t packed_rd = PackDpDr(rd);
  // bit0=RnW(1), bit1=A2(1), bit2=A3(1)
  TEST_ASSERT((packed_rd & 0x7u) == 0x7u);
  TEST_ASSERT((packed_rd >> 3) == 0xDEADBEEFu);

  // 高位由调用方截断为 35 位，addr2b 夹到 2 位 / addr2b masks to 2 bits.
  const Request masked = MakeApRequest(true, 0xFFu, 0x11223344u);
  TEST_ASSERT(masked.addr2b == 0x3u);

  Response resp{};
  const uint64_t out_ok = (uint64_t{0xCAFEBABEu} << 3) | 0x2u;
  UnpackDpDr(out_ok, resp);
  TEST_ASSERT(resp.ack == Ack::OK && resp.rdata == 0xCAFEBABEu);

  const uint64_t out_wait = (uint64_t{0x12345678u} << 3) | 0x1u;
  UnpackDpDr(out_wait, resp);
  TEST_ASSERT(resp.ack == Ack::WAIT && resp.rdata == 0x12345678u);

  const uint64_t out_bad = (uint64_t{0x00000001u} << 3) | 0x7u;
  UnpackDpDr(out_bad, resp);
  TEST_ASSERT(resp.ack == Ack::PROTOCOL && resp.rdata == 0x00000001u);
}

void TestChainCache()
{
  const uint8_t ir_lens[3] = {5u, 4u, 6u};
  ChainConfig cfg{};
  cfg.count = 3u;
  cfg.ir_length = ir_lens;

  // index=0（最靠近 TDO）/ index=0 (nearest TDO)
  cfg.index = 0u;
  UpdateChainCache(cfg);
  TEST_ASSERT(cfg.ir_before_bits_len == 0u && cfg.ir_after_bits_len == 10u);
  TEST_ASSERT(cfg.dr_before_bits_len == 0u && cfg.dr_after_bits_len == 2u);

  // index=1（中间设备）/ index=1 (middle TAP)
  cfg.index = 1u;
  UpdateChainCache(cfg);
  TEST_ASSERT(cfg.ir_before_bits_len == 5u && cfg.ir_after_bits_len == 6u);
  TEST_ASSERT(cfg.dr_before_bits_len == 1u && cfg.dr_after_bits_len == 1u);

  // index=2（最靠近 TDI）/ index=2 (nearest TDI)
  cfg.index = 2u;
  UpdateChainCache(cfg);
  TEST_ASSERT(cfg.ir_before_bits_len == 9u && cfg.ir_after_bits_len == 0u);
  TEST_ASSERT(cfg.dr_before_bits_len == 2u && cfg.dr_after_bits_len == 0u);

  // 单设备链 / Single-TAP chain
  const uint8_t single_ir[1] = {4u};
  cfg.count = 1u;
  cfg.index = 0u;
  cfg.ir_length = single_ir;
  UpdateChainCache(cfg);
  TEST_ASSERT(cfg.ir_before_bits_len == 0u && cfg.ir_after_bits_len == 0u);
  TEST_ASSERT(cfg.dr_before_bits_len == 0u && cfg.dr_after_bits_len == 0u);

  // 越界或空链应清零缓存 / Out-of-bounds or empty chain clears cache
  cfg.count = 2u;
  cfg.index = 2u;
  UpdateChainCache(cfg);
  TEST_ASSERT(cfg.ir_before_bits_len == 0u && cfg.ir_after_bits_len == 0u);
  TEST_ASSERT(cfg.dr_before_bits_len == 0u && cfg.dr_after_bits_len == 0u);

  cfg.count = 0u;
  cfg.index = 0u;
  UpdateChainCache(cfg);
  TEST_ASSERT(cfg.ir_before_bits_len == 0u && cfg.ir_after_bits_len == 0u);
  TEST_ASSERT(cfg.dr_before_bits_len == 0u && cfg.dr_after_bits_len == 0u);
}
}  // namespace

void test_jtag_protocol()
{
  TestAckMapping();
  TestPackAndUnpackDpDr();
  TestChainCache();
}
