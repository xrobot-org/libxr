/**
 * @file test_packet.cpp
 * @brief Topic 报文打包与解析测试 / Topic packet packing and parsing tests.
 *
 * 检查帧头、分段输入、时间戳、对齐与长度兼容，以及非法帧和打包错误。
 * Check headers, fragmented input, timestamps, alignment, length compatibility and
 * invalid packets.
 */

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

#include "crc.hpp"
#include "libxr.hpp"
#include "libxr_def.hpp"
#include "middleware/message/message_test_payloads.hpp"
#include "test.hpp"
#include "test_assert.hpp"

inline uint64_t TimestampUs(const LibXR::MicrosecondTimestamp& timestamp)
{
  return static_cast<uint64_t>(timestamp);
}

template <typename Packet>
inline void RewritePacketPayloadLengthForTest(Packet& packet, size_t payload_len)
{
  // 构造长度改变但 CRC 正确的报文，让测试进入长度兼容处理，而不是先被 CRC 拒绝。
  // Keep CRCs valid after changing length so the test reaches length handling rather than
  // CRC rejection.
  auto* packet_bytes = reinterpret_cast<uint8_t*>(&packet);
  const auto crc_offset = sizeof(LibXR::Topic::PackedDataHeader) + payload_len;

  packet.raw.header_.SetDataLen(static_cast<uint32_t>(payload_len));
  packet.raw.header_.pack_header_crc8 = LibXR::CRC8::Calculate(
      &packet.raw, sizeof(LibXR::Topic::PackedDataHeader) - sizeof(uint8_t));
  packet_bytes[crc_offset] = LibXR::CRC8::Calculate(packet_bytes, crc_offset);
}

namespace
{
void TestPacketHeaderAndServerParse()
{
  constexpr size_t PACKET_SIZE = LibXR::Topic::PACK_BASE_SIZE + sizeof(double);

  auto domain = LibXR::Topic::Domain("message_packet_domain");
  auto topic = LibXR::Topic::CreateTopic<double>("message_packet_tp", &domain);

  static bool cb_in_isr = false;
  static LibXR::MicrosecondTimestamp cb_timestamp;
  static double rx_value = 0.0;

  auto msg_cb = LibXR::Topic::Callback::Create(
      [](bool in_isr, void*, LibXR::MicrosecondTimestamp timestamp, double& data)
      {
        cb_in_isr = in_isr;
        cb_timestamp = timestamp;
        rx_value = data;
      },
      reinterpret_cast<void*>(0));
  topic.RegisterCallback(msg_cb);

  LibXR::Topic::PackedData<double> packed_data;
  LibXR::Topic::Server topic_server(512);
  topic_server.Register(topic);

  const double value0 = 48.48;
  const LibXR::MicrosecondTimestamp timestamp0(4004);
  TEST_ASSERT(topic.PackData(value0, packed_data, timestamp0) == LibXR::ErrorCode::OK);
  rx_value = -1.0;
  cb_in_isr = false;
  TEST_ASSERT(
      topic_server.ParseDataFromCallback(LibXR::ConstRawData(packed_data), true) == 1);
  TEST_ASSERT(rx_value == value0);
  TEST_ASSERT(TimestampUs(cb_timestamp) == TimestampUs(timestamp0));
  TEST_ASSERT(cb_in_isr);

  const double value1 = 56.56;
  const LibXR::MicrosecondTimestamp timestamp1(5005);
  TEST_ASSERT(topic.PackData(value1, packed_data, timestamp1) == LibXR::ErrorCode::OK);
  rx_value = -1.0;
  cb_in_isr = true;
  TEST_ASSERT(
      topic_server.ParseDataFromCallback(LibXR::ConstRawData(packed_data), false) == 1);
  TEST_ASSERT(rx_value == value1);
  TEST_ASSERT(TimestampUs(cb_timestamp) == TimestampUs(timestamp1));
  TEST_ASSERT(!cb_in_isr);

  const double value2 = 64.64;
  const LibXR::MicrosecondTimestamp timestamp2(0x010203040506ULL);
  TEST_ASSERT(topic.PackData(value2, packed_data, timestamp2) == LibXR::ErrorCode::OK);
  TEST_ASSERT(packed_data.raw.header_.prefix == LibXR::Topic::PACKET_PREFIX);
  TEST_ASSERT(packed_data.raw.header_.version == LibXR::Topic::PACKET_VERSION);
  TEST_ASSERT(packed_data.raw.header_.data_len_raw[0] == sizeof(double));
  TEST_ASSERT(packed_data.raw.header_.data_len_raw[1] == 0);
  TEST_ASSERT(packed_data.raw.header_.data_len_raw[2] == 0);
  TEST_ASSERT(packed_data.raw.header_.timestamp_us_raw[0] == 0x06);
  TEST_ASSERT(packed_data.raw.header_.timestamp_us_raw[1] == 0x05);
  TEST_ASSERT(packed_data.raw.header_.timestamp_us_raw[2] == 0x04);
  TEST_ASSERT(packed_data.raw.header_.timestamp_us_raw[3] == 0x03);
  TEST_ASSERT(packed_data.raw.header_.timestamp_us_raw[4] == 0x02);
  TEST_ASSERT(packed_data.raw.header_.timestamp_us_raw[5] == 0x01);
  TEST_ASSERT(TimestampUs(packed_data.GetTimestamp()) == TimestampUs(timestamp2));

  // 拆成两段输入，第一段不足一帧时不能发布，收到余下部分后才完成解析。
  // Feed two fragments: the incomplete first part must not publish; the remainder
  // completes the frame.
  auto* packet = reinterpret_cast<uint8_t*>(&packed_data);
  TEST_ASSERT(topic_server.ParseData(LibXR::ConstRawData(packet, 3)) == 0);
  TEST_ASSERT(topic_server.ParseData(LibXR::ConstRawData(packet + 3, PACKET_SIZE - 3)) ==
              1);
  TEST_ASSERT(rx_value == value2);
  TEST_ASSERT(TimestampUs(cb_timestamp) == TimestampUs(timestamp2));

  LibXR::Topic::Server exact_size_server(PACKET_SIZE);
  exact_size_server.Register(topic);
  TEST_ASSERT(exact_size_server.ParseData(LibXR::ConstRawData(packed_data)) == 1);
  TEST_ASSERT(rx_value == value2);
  TEST_ASSERT(TimestampUs(cb_timestamp) == TimestampUs(timestamp2));

  const double raw_value = 72.72;
  const LibXR::MicrosecondTimestamp raw_timestamp(6006);
  uint8_t raw_packet[PACKET_SIZE] = {};
  TEST_ASSERT(topic.PackRaw(LibXR::ConstRawData(raw_value),
                            LibXR::RawData(raw_packet, sizeof(raw_packet)),
                            raw_timestamp) == LibXR::ErrorCode::OK);
  rx_value = -1.0;
  TEST_ASSERT(
      topic_server.ParseData(LibXR::ConstRawData(raw_packet, sizeof(raw_packet))) == 1);
  TEST_ASSERT(rx_value == raw_value);
  TEST_ASSERT(TimestampUs(cb_timestamp) == TimestampUs(raw_timestamp));

  uint8_t small_packet[PACKET_SIZE - 1] = {};
  uint32_t wrong_size_payload = 0;
  TEST_ASSERT(topic.PackRaw(LibXR::ConstRawData(wrong_size_payload),
                            LibXR::RawData(raw_packet, sizeof(raw_packet)),
                            raw_timestamp) == LibXR::ErrorCode::SIZE_ERR);
  TEST_ASSERT(topic.PackRaw(LibXR::ConstRawData(raw_value),
                            LibXR::RawData(small_packet, sizeof(small_packet)),
                            raw_timestamp) == LibXR::ErrorCode::NO_BUFF);
  TEST_ASSERT(topic.PackRaw(LibXR::ConstRawData(raw_value), LibXR::RawData(),
                            raw_timestamp) == LibXR::ErrorCode::PTR_NULL);
}

void TestPacketAlignmentAndLengthCompatibility()
{
  // 检查强对齐数据的回调访问，以及旧报文只带结构体前缀时仍能读取已有字段。
  // Check callback access to aligned data and reading existing fields from a shorter
  // prefix packet.
  auto domain = LibXR::Topic::Domain("message_packet_alignment_domain");

  auto aligned_topic =
      LibXR::Topic::CreateTopic<WideAlignedPayload>("aligned_packet_tp", &domain);
  static uint64_t aligned_view_value = 0;
  auto aligned_cb = LibXR::Topic::Callback::Create(
      [](bool, void*, const LibXR::Topic::MessageView<WideAlignedPayload>& message)
      {
        TEST_ASSERT(message.data != nullptr);
        aligned_view_value = message.data->right;
      },
      reinterpret_cast<void*>(0));
  aligned_topic.RegisterCallback(aligned_cb);
  LibXR::Topic::Server aligned_server(512);
  aligned_server.Register(aligned_topic);
  WideAlignedPayload aligned_tx{0x1122334455667788ULL, 0x8877665544332211ULL};
  LibXR::Topic::PackedData<WideAlignedPayload> aligned_packet;
  TEST_ASSERT(aligned_topic.PackData(aligned_tx, aligned_packet,
                                     LibXR::MicrosecondTimestamp(6106)) ==
              LibXR::ErrorCode::OK);
  aligned_view_value = 0;
  TEST_ASSERT(aligned_server.ParseData(LibXR::ConstRawData(aligned_packet)) == 1);
  TEST_ASSERT(aligned_view_value == aligned_tx.right);

  auto prefix_topic =
      LibXR::Topic::CreateTopic<PrefixIntPayload>("prefix_int_tp", &domain);
  static PrefixIntPayload prefix_rx{};
  auto prefix_cb =
      LibXR::Topic::Callback::Create([](bool, void*, PrefixIntPayload& data)
                                     { prefix_rx = data; }, reinterpret_cast<void*>(0));
  prefix_topic.RegisterCallback(prefix_cb);
  LibXR::Topic::Server prefix_server(512);
  prefix_server.Register(prefix_topic);
  PrefixIntPayload prefix_tx{0x11223344, 0x55667788};
  LibXR::Topic::PackedData<PrefixIntPayload> prefix_packet;
  TEST_ASSERT(prefix_topic.PackData(prefix_tx, prefix_packet,
                                    LibXR::MicrosecondTimestamp(6116)) ==
              LibXR::ErrorCode::OK);
  RewritePacketPayloadLengthForTest(prefix_packet, sizeof(int32_t));
  prefix_rx = PrefixIntPayload{-1, -1};
  TEST_ASSERT(prefix_server.ParseData(LibXR::ConstRawData(
                  &prefix_packet, LibXR::Topic::PACK_BASE_SIZE + sizeof(int32_t))) == 1);
  TEST_ASSERT(prefix_rx.value == prefix_tx.value);
}

void TestPacketValidationFailures()
{
  constexpr size_t PACKET_SIZE = LibXR::Topic::PACK_BASE_SIZE + sizeof(double);

  auto domain = LibXR::Topic::Domain("message_packet_validation_domain");
  auto topic = LibXR::Topic::CreateTopic<double>("message_packet_validation_tp", &domain);

  static double rx_value = 0.0;
  auto msg_cb = LibXR::Topic::Callback::Create(
      [](bool, void*, LibXR::MicrosecondTimestamp, double& data) { rx_value = data; },
      reinterpret_cast<void*>(0));
  topic.RegisterCallback(msg_cb);

  LibXR::Topic::PackedData<double> packed_data;
  TEST_ASSERT(topic.PackData(77.77, packed_data, LibXR::MicrosecondTimestamp(123456)) ==
              LibXR::ErrorCode::OK);

  LibXR::Topic::Server topic_server(512);
  topic_server.Register(topic);

  // 未知主题报文仍使用正确 CRC，避免把校验失败误当成主题查找失败。
  // Keep valid CRCs for the unknown topic so a checksum failure cannot masquerade as a
  // lookup failure.
  auto unknown_topic_packet = packed_data;
  unknown_topic_packet.raw.header_.topic_name_crc32 ^= 0x13572468;
  unknown_topic_packet.raw.header_.pack_header_crc8 =
      LibXR::CRC8::Calculate(&unknown_topic_packet.raw,
                             sizeof(LibXR::Topic::PackedDataHeader) - sizeof(uint8_t));
  unknown_topic_packet.crc8_ =
      LibXR::CRC8::Calculate(&unknown_topic_packet, PACKET_SIZE - sizeof(uint8_t));
  TEST_ASSERT(topic_server.ParseData(LibXR::ConstRawData(unknown_topic_packet)) == 0);

  auto unknown_version_packet = packed_data;
  unknown_version_packet.raw.header_.version ^= 0x5A;
  unknown_version_packet.raw.header_.pack_header_crc8 =
      LibXR::CRC8::Calculate(&unknown_version_packet.raw,
                             sizeof(LibXR::Topic::PackedDataHeader) - sizeof(uint8_t));
  unknown_version_packet.crc8_ =
      LibXR::CRC8::Calculate(&unknown_version_packet, PACKET_SIZE - sizeof(uint8_t));
  TEST_ASSERT(topic_server.ParseData(LibXR::ConstRawData(unknown_version_packet)) == 0);

  auto truncated_packet = packed_data;
  RewritePacketPayloadLengthForTest(truncated_packet, sizeof(double) - 1);
  rx_value = -1.0;
  TEST_ASSERT(topic_server.ParseData(
                  LibXR::ConstRawData(&truncated_packet, PACKET_SIZE - 1)) == 1);
  TEST_ASSERT(rx_value != 77.77);

  auto legacy_prefix_packet = packed_data;
  legacy_prefix_packet.raw.header_.prefix = 0xA5;
  legacy_prefix_packet.raw.header_.pack_header_crc8 =
      LibXR::CRC8::Calculate(&legacy_prefix_packet.raw,
                             sizeof(LibXR::Topic::PackedDataHeader) - sizeof(uint8_t));
  legacy_prefix_packet.crc8_ =
      LibXR::CRC8::Calculate(&legacy_prefix_packet, PACKET_SIZE - sizeof(uint8_t));
  TEST_ASSERT(topic_server.ParseData(LibXR::ConstRawData(legacy_prefix_packet)) == 0);

  auto bad_header_packet = packed_data;
  bad_header_packet.raw.header_.pack_header_crc8 ^= 0x5A;
  TEST_ASSERT(topic_server.ParseData(LibXR::ConstRawData(bad_header_packet)) == 0);

  auto bad_payload_packet = packed_data;
  bad_payload_packet.crc8_ ^= 0xA5;
  TEST_ASSERT(topic_server.ParseData(LibXR::ConstRawData(bad_payload_packet)) == 0);

  TEST_ASSERT(topic_server.ParseData(LibXR::ConstRawData(packed_data)) == 1);
  TEST_ASSERT(rx_value == 77.77);
}

void AppendPacket(std::vector<uint8_t>& stream, const void* packet, size_t size)
{
  const auto* bytes = static_cast<const uint8_t*>(packet);
  stream.insert(stream.end(), bytes, bytes + size);
}

void TestPacketResync()
{
  // 检查假前缀、未注册 topic、超长声明与超过队列的输入都不会连带丢掉后面的真包。
  // Check that false prefixes, unregistered topics, oversized declarations and input
  // longer than the queue never take a following real packet with them.
  constexpr size_t PACKET_SIZE = LibXR::Topic::PACK_BASE_SIZE + sizeof(double);

  auto domain = LibXR::Topic::Domain("message_packet_resync_domain");
  auto topic = LibXR::Topic::CreateTopic<double>("message_packet_resync_tp", &domain);
  auto unknown =
      LibXR::Topic::CreateTopic<double>("message_packet_resync_other", &domain);

  static size_t rx_count = 0;
  static double rx_value = 0.0;
  auto msg_cb = LibXR::Topic::Callback::Create(
      [](bool, void*, double& data)
      {
        rx_count++;
        rx_value = data;
      },
      reinterpret_cast<void*>(0));
  topic.RegisterCallback(msg_cb);

  LibXR::Topic::Server server(512);
  server.Register(topic);

  LibXR::Topic::PackedData<double> real;
  TEST_ASSERT(topic.PackData(1.5, real, LibXR::MicrosecondTimestamp(1000)) ==
              LibXR::ErrorCode::OK);

  // 未注册 topic 的包，payload 最后一个字节是前缀
  // 0x5A：不能把它当成包头，吞掉后面的真包。 A packet of an unregistered topic whose last
  // payload byte is the 0x5A prefix: it must not be taken as a header that swallows the
  // following real packet.
  const uint8_t prefix_tail[sizeof(double)] = {1, 2, 3, 4,
                                               5, 6, 7, LibXR::Topic::PACKET_PREFIX};
  double unknown_value = 0.0;
  std::memcpy(&unknown_value, prefix_tail, sizeof(double));
  LibXR::Topic::PackedData<double> other;
  TEST_ASSERT(unknown.PackData(unknown_value, other, LibXR::MicrosecondTimestamp(1001)) ==
              LibXR::ErrorCode::OK);
  std::vector<uint8_t> stream;
  AppendPacket(stream, &other, PACKET_SIZE);
  AppendPacket(stream, &real, PACKET_SIZE);
  rx_count = 0;
  TEST_ASSERT(server.ParseData(LibXR::ConstRawData(stream.data(), stream.size())) == 1);
  TEST_ASSERT(rx_count == 1 && rx_value == 1.5);

  // 噪声里的 0x5A 紧挨着真包 / A 0x5A in noise right before a real packet.
  stream = {LibXR::Topic::PACKET_PREFIX, 0x01, 0x02};
  AppendPacket(stream, &real, PACKET_SIZE);
  rx_count = 0;
  TEST_ASSERT(server.ParseData(LibXR::ConstRawData(stream.data(), stream.size())) == 1);
  TEST_ASSERT(rx_count == 1);

  // 头部 CRC 正确但声明的长度超过队列：按假前缀处理，后面的真包照收。
  // A valid header CRC declaring more than the queue holds: handled as a false prefix,
  // and the following real packet is still received.
  auto oversized = real;
  oversized.raw.header_.SetDataLen(100000);
  oversized.raw.header_.pack_header_crc8 = LibXR::CRC8::Calculate(
      &oversized.raw, sizeof(LibXR::Topic::PackedDataHeader) - sizeof(uint8_t));
  stream.clear();
  AppendPacket(stream, &oversized, sizeof(LibXR::Topic::PackedDataHeader));
  AppendPacket(stream, &real, PACKET_SIZE);
  rx_count = 0;
  TEST_ASSERT(server.ParseData(LibXR::ConstRawData(stream.data(), stream.size())) == 1);
  TEST_ASSERT(rx_count == 1);

  // payload CRC 坏的包后面紧跟真包 / A packet with a bad payload CRC right before a real
  // packet.
  auto bad_payload = real;
  bad_payload.crc8_ ^= 0xA5;
  stream.clear();
  AppendPacket(stream, &bad_payload, PACKET_SIZE);
  AppendPacket(stream, &real, PACKET_SIZE);
  rx_count = 0;
  TEST_ASSERT(server.ParseData(LibXR::ConstRawData(stream.data(), stream.size())) == 1);
  TEST_ASSERT(rx_count == 1);

  // 混合流逐字节喂入，结果相同 / The mixed stream fed one byte at a time gives the same
  // result.
  stream.clear();
  AppendPacket(stream, &other, PACKET_SIZE);
  stream.push_back(LibXR::Topic::PACKET_PREFIX);
  AppendPacket(stream, &real, PACKET_SIZE);
  AppendPacket(stream, &bad_payload, PACKET_SIZE);
  AppendPacket(stream, &real, PACKET_SIZE);
  rx_count = 0;
  size_t delivered = 0;
  for (uint8_t byte : stream)
  {
    delivered += server.ParseData(LibXR::ConstRawData(&byte, 1));
  }
  TEST_ASSERT(delivered == 2 && rx_count == 2);

  // 一次喂入的字节比队列长：分段处理，一个包都不丢。
  // One input longer than the queue is handled in pieces and loses no packet.
  LibXR::Topic::Server small_server(PACKET_SIZE * 2);
  small_server.Register(topic);
  stream.clear();
  for (int i = 0; i < 9; ++i)
  {
    AppendPacket(stream, &real, PACKET_SIZE);
  }
  rx_count = 0;
  TEST_ASSERT(small_server.ParseData(LibXR::ConstRawData(stream.data(), stream.size())) ==
              9);
  TEST_ASSERT(rx_count == 9);
}

}  // namespace

void test_message_packet()
{
  TestPacketHeaderAndServerParse();
  TestPacketValidationFailures();
  TestPacketAlignmentAndLengthCompatibility();
  TestPacketResync();
}
