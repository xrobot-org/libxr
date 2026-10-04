#pragma once

#include "../topic.hpp"
#include "queue.hpp"

namespace LibXR
{
/**
 * @class Topic::Server
 * @brief 将字节流解析成 packet 并发布到已注册 topic 的状态机 / State machine that
 *        parses byte streams into packets and publishes them into registered topics
 *
 * @note 当前 `Server` 自己不维护 raw topic/cache 语义；它只依赖注册 topic 的：
 *       `payload_size`、`payload_alignment` 和名字 CRC 键。
 *       The current `Server` does not restore old raw topic/cache semantics by
 *       itself; it relies only on each registered topic's `payload_size`,
 *       `payload_alignment`, and name CRC key.
 * @note 当前兼容规则：
 *       收到的 packet payload 短于 topic
 * 固定大小时，仅保证前缀部分有效，后半段保持未定义； 长于 topic
 * 固定大小时，仅保留前缀部分，其余字节直接截断。
 *       Current compatibility rule: when an incoming packet payload is shorter than the
 *       topic's fixed size, only the leading prefix is guaranteed valid and the remaining
 *       tail stays unspecified; when it is longer, only the prefix matching the topic
 *       size is kept and the rest is truncated.
 */
class Topic::Server
{
 public:
  /**
   * @enum Status
   * @brief parser 当前所在阶段 / Current stage of the parser
   */
  enum class Status : uint8_t
  {
    WAIT_START,     ///< 正在找下一包前缀 / Searching for the next packet prefix.
    WAIT_TOPIC,     ///< 已读到前缀，正在等完整头 / Prefix received; waiting for the full
                    ///< header.
    WAIT_DATA_CRC,  ///< 头已接受，正在等 payload 和尾 CRC / Header accepted; waiting for
                    ///< payload plus trailing CRC.
  };

  /**
   * @brief 构造 parser 并分配内部暂存队列 / Construct the parser and allocate its
   *        internal staging queue
   * @param buffer_length 内部字节队列和暂存缓冲区大小 / Size of the internal byte queue
   *        and staging buffer
   */
  Server(size_t buffer_length);

  /**
   * @brief 注册一个可接收 packet 的 topic / Register one topic that may receive parsed
   *        packets
   * @param topic 目标 topic 句柄 / Target topic handle
   *
   * @note 注册时会断言该 topic 的 `payload_size + PACK_BASE_SIZE` 能放进本 server 的
   *       暂存缓冲区。
   *       Registration asserts that the topic's `payload_size + PACK_BASE_SIZE`
   *       fits in this server's staging buffer.
   * @note server 的内部暂存缓冲区在构造时固定按 `CACHE_LINE_SIZE` 对齐分配；注册时还会
   *       断言该 topic 的 `payload_alignment <= CACHE_LINE_SIZE`。
   *       The server's internal staging buffer is allocated once at
   *       `CACHE_LINE_SIZE` alignment during construction; registration also
   *       asserts that the topic's `payload_alignment <= CACHE_LINE_SIZE`.
   */
  void Register(TopicHandle topic);

  /**
   * @brief 在普通上下文里喂入一批新字节 / Feed one new byte batch in normal context
   * @param data 新收到的原始字节 / Newly received raw bytes
   * @return 成功解析并发布的包数量 / Number of packets parsed and published
   */
  size_t ParseData(ConstRawData data);

  /**
   * @brief 在回调/ISR 路径里喂入一批新字节 / Feed one new byte batch in callback/ISR path
   * @param data 新收到的原始字节 / Newly received raw bytes
   * @param in_isr 当前是否位于 ISR / Whether the current path is in ISR context
   * @return 成功解析并发布的包数量 / Number of packets parsed and published
   */
  size_t ParseDataFromCallback(ConstRawData data, bool in_isr);

 private:
  /**
   * @enum ParseResult
   * @brief 一次 payload 阶段处理结果 / Result of one payload-stage handling step
   */
  enum class ParseResult : uint8_t
  {
    NEED_MORE,  ///< 当前包还没收全 / Current packet is still incomplete.
    DROPPED,    ///< 当前包被丢弃 / Current packet is dropped.
    DELIVERED   ///< 当前包已发布 / Current packet is delivered.
  };

  /**
   * @brief `ParseData*()` 的共享实现 / Shared implementation behind `ParseData*()`
   * @param data 新收到的原始字节 / Newly received raw bytes
   * @param from_callback 是否来自回调路径 / Whether the current parse comes from
   *        callback path
   * @param in_isr 当前是否位于 ISR / Whether the current path is in ISR context
   * @return 成功解析并发布的包数量 / Number of packets parsed and published
   */
  size_t ParseDataRaw(ConstRawData data, bool from_callback, bool in_isr);

  /**
   * @brief 把输入流同步到下一条 packet 起点 / Synchronize the input stream to the next
   *        packet start
   * @return 若已找到起始字节则返回 `true`，否则返回 `false` / Returns `true` when a
   *         packet start byte is found, otherwise `false`
   */
  bool SyncToPacketStart();

  /**
   * @brief 在已对齐前缀后继续读取并校验完整头部 / Read and validate the full header
   *        after the prefix is aligned
   * @return 头部有效返回 `true`，否则返回 `false` / Returns `true` when the header is
   *         valid, otherwise `false`
   */
  bool ReadHeader();

  /**
   * @brief 读取当前包的 payload 和尾 CRC，并在成功时发布 / Read the payload and
   *        trailing CRC of the current packet and publish it on success
   * @param from_callback 是否来自回调路径 / Whether the current parse comes from
   *        callback path
   * @param in_isr 当前是否位于 ISR / Whether the current path is in ISR context
   * @return 当前 payload 阶段的处理结果 / Result of the current payload stage
   */
  ParseResult ReadPayload(bool from_callback, bool in_isr);

  /**
   * @brief 清空当前包的解析上下文并回到找起点状态 / Clear the current packet parsing
   *        context and return to the start-search state
   */
  void ResetParser();

  Status status_ = Status::WAIT_START;  ///< 当前 parser 阶段 / Current parser stage.
  uint32_t data_len_ = 0;  ///< 当前包头声明的 payload 长度 / Payload length declared by
                           ///< the current header.
  RBTree<uint32_t> topic_map_;  ///< 从 topic 名称 CRC32 到 topic 句柄的映射 / Map from
                                ///< topic-name CRC32 to topic handle.
  QueueBase queue_;             ///< 输入字节 FIFO / Input byte FIFO.
  RawData parse_buff_;  ///< 当前包头和 payload 的暂存缓冲区 / Staging buffer holding the
                        ///< current header and payload.
  TopicHandle current_topic_ =
      nullptr;  ///< 当前包命中的目标 topic / Target topic matched by the current packet.
  MicrosecondTimestamp current_timestamp_;  ///< 当前包头里的时间戳 / Timestamp carried by
                                            ///< the current packet header.
};
}  // namespace LibXR
