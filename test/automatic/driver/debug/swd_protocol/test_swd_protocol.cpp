/**
 * @file test_swd_protocol.cpp
 * @brief SWD 请求与 ACK 编码主机测试 / Host tests for the SWD request and ACK encoding.
 *
 * 判据来自 ADIv5.2 B4.2 的包格式：请求按线上顺序为 Start=1、APnDP、RnW、A[2]、A[3]、
 * Parity（这四位的偶校验）、Stop=0、Park=1，LSB 先发；ACK 3 位 LSB 先收，OK=0b001、
 * WAIT=0b010、FAULT=0b100。
 * The oracle is the packet format of ADIv5.2 B4.2: the request in wire order is
 * Start=1, APnDP, RnW, A[2], A[3], Parity (even parity of those four), Stop=0, Park=1,
 * LSB first; the 3-bit ACK is received LSB first, OK=0b001, WAIT=0b010, FAULT=0b100.
 */

#include <initializer_list>

#include "debug/swd_protocol.hpp"
#include "test_assert.hpp"

using namespace LibXR::Debug::SwdProtocol;

namespace
{
/// 按包格式逐位写出的请求字节，bit0 最先上线
/// Request bits written out from the packet definition, bit 0 first on the wire.
constexpr uint8_t Packet(bool apndp, bool rnw, bool a2, bool a3)
{
  const bool parity = apndp ^ rnw ^ a2 ^ a3;
  return static_cast<uint8_t>(1u | (apndp ? 0x02u : 0u) | (rnw ? 0x04u : 0u) |
                              (a2 ? 0x08u : 0u) | (a3 ? 0x10u : 0u) |
                              (parity ? 0x20u : 0u) | 0x80u);
}

void TestRequestEncoding()
{
  // 注释列出各请求中为 1 的位 / The comments list the bits that are set.
  // DP read IDCODE (A = 0x0): Start, RnW, Parity, Park.
  TEST_ASSERT(MakeRequest(make_dp_read_req(DpReadReg::IDCODE)) == 0xA5u);
  // DP read RDBUFF (A = 0xC): Start, RnW, A2, A3, Parity, Park.
  TEST_ASSERT(MakeRequest(make_dp_read_req(DpReadReg::RDBUFF)) == 0xBDu);
  // DP write ABORT (A = 0x0): Start and Park only.
  TEST_ASSERT(MakeRequest(make_dp_write_req(DpWriteReg::ABORT, 0u)) == 0x81u);
  // DP write SELECT (A = 0x8): Start, A3, Parity, Park.
  TEST_ASSERT(MakeRequest(make_dp_write_req(DpWriteReg::SELECT, 0u)) == 0xB1u);
  // AP read, A = 0x0: Start, APnDP, RnW, Park (even parity of two ones is zero).
  TEST_ASSERT(MakeRequest(make_ap_read_req(0u)) == 0x87u);
  // AP write, A = 0xC: Start, APnDP, A2, A3, Parity, Park.
  TEST_ASSERT(MakeRequest(make_ap_write_req(3u, 0u)) == 0xBBu);

  // 所有组合都与包格式比较；写数据不影响请求字节。
  // Every combination against the packet definition; the write data does not matter.
  for (unsigned v = 0; v < 16u; ++v)
  {
    const bool apndp = (v & 1u) != 0u;
    const bool rnw = (v & 2u) != 0u;
    const uint8_t addr2b = static_cast<uint8_t>(v >> 2);
    const Request req{apndp ? Port::AP : Port::DP, rnw, addr2b, 0xFFFFFFFFu};
    TEST_ASSERT(MakeRequest(req) ==
                Packet(apndp, rnw, (addr2b & 1u) != 0u, (addr2b & 2u) != 0u));
  }
  static_assert(MakeRequest(make_dp_read_req(DpReadReg::IDCODE)) == 0xA5u,
                "MakeRequest is usable in constant expressions");
}

void TestAckDecoding()
{
  TEST_ASSERT(DecodeAck(0x1u) == Ack::OK);
  TEST_ASSERT(DecodeAck(0x2u) == Ack::WAIT);
  TEST_ASSERT(DecodeAck(0x4u) == Ack::FAULT);
  // 没有目标驱动数据线 / No target driving the line.
  TEST_ASSERT(DecodeAck(0x0u) == Ack::NO_ACK);
  // 其余组合都不是合法的 ACK，包括悬空的线被上拉时读到的全 1。
  // Any other pattern is not a valid ACK, including the all-ones of a floating line
  // pulled high.
  for (unsigned bits : {0x3u, 0x5u, 0x6u, 0x7u})
  {
    TEST_ASSERT(DecodeAck(static_cast<uint8_t>(bits)) == Ack::PROTOCOL);
  }
  static_assert(DecodeAck(0x1u) == Ack::OK,
                "DecodeAck is usable in constant expressions");
}
}  // namespace

void test_swd_protocol()
{
  TestRequestEncoding();
  TestAckDecoding();
}
