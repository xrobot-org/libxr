#pragma once

#include "../topic.hpp"

namespace LibXR
{
#ifndef __DOXYGEN__
LIBXR_PACKED_BEGIN
/**
 * @struct Topic::PackedDataHeader
 * @brief 打包消息的固定 16 字节头 / Fixed 16-byte header of one packed message
 *
 * @note 当前头布局固定为：
 *       `prefix(1) + data_len(3) + topic_crc32(4) + timestamp_us(6) + version(1) +
 *       crc8(1)`。
 *       The current header layout is fixed as:
 *       `prefix(1) + data_len(3) + topic_crc32(4) + timestamp_us(6) + version(1) +
 *       crc8(1)`.
 */
struct Topic::PackedDataHeader
{
  uint8_t prefix;             ///< 包前缀字节 / Packet prefix byte.
  uint8_t data_len_raw[3];    ///< 小端 24 位 payload 长度 / Little-endian 24-bit payload
                              ///< length.
  uint32_t topic_name_crc32;  ///< 目标 topic 名称 CRC32 键 / CRC32 key of the target
                              ///< topic name.
  uint8_t timestamp_us_raw[6];  ///< 小端 48 位微秒时间戳 / Little-endian 48-bit
                                ///< microsecond timestamp.
  uint8_t version;              ///< 包协议版本 / Packet protocol version.
  uint8_t pack_header_crc8;     ///< 头部 CRC8 / Header CRC8.

  /**
   * @brief 设置数据区长度 / Set the payload length
   * @param len 数据区长度 / Payload length
   */
  void SetDataLen(uint32_t len);

  /**
   * @brief 获取数据区长度 / Get the payload length
   * @return 数据区长度 / Returns the payload length
   */
  uint32_t GetDataLen() const;

  /**
   * @brief 设置报文时间戳 / Set the packet timestamp
   * @param timestamp 报文时间戳 / Packet timestamp
   */
  void SetTimestamp(MicrosecondTimestamp timestamp);

  /**
   * @brief 获取报文时间戳 / Get the packet timestamp
   * @return 报文时间戳 / Returns the packet timestamp
   */
  MicrosecondTimestamp GetTimestamp() const;
};

static_assert(sizeof(Topic::PackedDataHeader) == 16);
static_assert(offsetof(Topic::PackedDataHeader, prefix) == 0);
static_assert(offsetof(Topic::PackedDataHeader, data_len_raw) == 1);
static_assert(offsetof(Topic::PackedDataHeader, topic_name_crc32) == 4);
static_assert(offsetof(Topic::PackedDataHeader, timestamp_us_raw) == 8);
static_assert(offsetof(Topic::PackedDataHeader, version) == 14);
static_assert(offsetof(Topic::PackedDataHeader, pack_header_crc8) == 15);

/**
 * @class Topic::PackedData
 * @brief 一条完整打包消息的原始字节对象 / Raw-byte object of one fully packed message
 * @tparam Data payload 类型 / Payload type
 *
 * @note 这里暴露的是 packet 字节布局，不承诺 `raw.data_` 自身具备 `Data` 的自然对齐。
 *       This exposes packet byte layout only; it does not promise that
 *       `raw.data_` itself satisfies the natural alignment of `Data`.
 */
template <typename Data>
class Topic::PackedData
{
  static_assert(TopicPayload<Data>);

 public:
  /**
   * @brief `PackedDataHeader + payload` 的原始字节布局 / Raw byte layout consisting of
   *        `PackedDataHeader + payload`
   */
  struct
  {
    PackedDataHeader header_;     ///< 固定头 / Fixed packet header.
    uint8_t data_[sizeof(Data)];  ///< payload 原始字节区 / Raw payload byte area.
  } raw;  ///< 固定头和 payload 组成的原始字节布局 / Raw byte layout made of the fixed
          ///< header and payload.

  uint8_t crc8_;  ///< 尾部 CRC8 / Trailing CRC8.

  /**
   * @brief 读取 packet 里的时间戳 / Read the timestamp stored in this packet
   * @return packet 时间戳 / Packet timestamp
   */
  MicrosecondTimestamp GetTimestamp() const { return raw.header_.GetTimestamp(); }
};
LIBXR_PACKED_END
#endif

/**
 * @brief 按当前 topic 的名字和类型契约把一条消息打包成 packet / Pack one message into
 *        one packet using the current topic name and type contract
 * @tparam Data payload 类型 / Payload type
 * @param data 待打包 payload / Payload to pack
 * @param packet 输出 packet / Output packed message
 * @param timestamp 要写入包头的时间戳 / Timestamp to store into the packet header
 * @return 操作结果错误码 / Error code
 */
template <typename Data>
ErrorCode Topic::PackData(const Data& data, PackedData<Data>& packet,
                          MicrosecondTimestamp timestamp)
{
  CheckTopicPayload<Data>();
  ASSERT(block_ != nullptr);
  ASSERT(block_->data_.payload_type_id == TypeID::GetID<Data>());
  ASSERT(block_->data_.payload_size == sizeof(Data));

  PackBytes(block_->data_.crc32, RawData(packet), timestamp, ConstRawData(data));
  return ErrorCode::OK;
}
}  // namespace LibXR
