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
 * @note 重新同步规则：包头和整包两个 CRC 都通过之前，字节只看不取；任一 CRC 不对，说明
 *       队首的前缀只是数据里碰巧出现的 0x5A，只丢这一个字节再接着找。两个 CRC 都通过的包
 *       是真包，版本不对或 topic 未注册时整包跳过。所以噪声、未注册的 topic 都不会连带
 *       丢掉后面的真包。
 *       Resync rule: bytes are only peeked until both the header CRC and the packet CRC
 *       pass; if either fails, the prefix at the front is a 0x5A that merely occurs in
 *       the data, and only that one byte is dropped before searching on. A packet that
 *       passes both CRCs is real and is skipped whole when its version differs or its
 *       topic is not registered. Noise and unregistered topics therefore never take a
 *       following real packet with them.
 */
class Topic::Server
{
 public:
  /**
   * @brief 构造 parser 并分配内部暂存队列 / Construct the parser and allocate its
   *        internal staging queue
   * @param buffer_length 内部字节队列和暂存缓冲区大小，也是能接收的最大包长 / Size of
   *        the internal byte queue and staging buffer, which is also the largest packet
   *        accepted
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
   * @param data 新收到的原始字节，长度不限 / Newly received raw bytes, of any length
   * @return 成功解析并发布的包数量 / Number of packets parsed and published
   */
  size_t ParseData(ConstRawData data);

  /**
   * @brief 在回调/ISR 路径里喂入一批新字节 / Feed one new byte batch in callback/ISR path
   * @param data 新收到的原始字节，长度不限 / Newly received raw bytes, of any length
   * @param in_isr 当前是否位于 ISR / Whether the current path is in ISR context
   * @return 成功解析并发布的包数量 / Number of packets parsed and published
   */
  size_t ParseDataFromCallback(ConstRawData data, bool in_isr);

 private:
  /**
   * @enum Step
   * @brief 队首一次处理的结果 / Result of handling the queue front once
   */
  enum class Step : uint8_t
  {
    NEED_MORE,  ///< 队首的包还没收全 / The packet at the front is incomplete.
    DISCARDED,  ///< 丢掉了一个假前缀字节或一个不接收的完整包 / Dropped one false prefix
                ///< byte or one whole packet that is not accepted.
    DELIVERED   ///< 队首的包已发布 / The packet at the front was published.
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
   * @brief 处理队列里已有的字节，直到需要更多输入 / Handle the queued bytes until more
   *        input is needed
   * @param from_callback 是否来自回调路径 / Whether the current parse comes from
   *        callback path
   * @param in_isr 当前是否位于 ISR / Whether the current path is in ISR context
   * @return 发布的包数量 / Number of packets published
   */
  size_t ParseQueued(bool from_callback, bool in_isr);

  /**
   * @brief 丢掉前缀之前的字节，让队首是前缀字节 / Drop the bytes before the next prefix
   *        so the queue starts with a prefix byte
   * @return 找到前缀返回 `true` / Returns `true` when a prefix byte is at the front
   */
  bool SyncToPacketStart();

  /**
   * @brief 处理队首以前缀开头的一个包：先只看不取，整包 CRC 都通过后才从队列取走。
   *        Handle one packet that starts at the queue front: peek first, and remove
   *        it from the queue only after both CRCs pass.
   * @param from_callback 是否来自回调路径 / Whether the current parse comes from
   *        callback path
   * @param in_isr 当前是否位于 ISR / Whether the current path is in ISR context
   * @return 这一次处理的结果 / Result of this step
   */
  Step HandleFrontPacket(bool from_callback, bool in_isr);

  RBTree<uint32_t> topic_map_;  ///< 从 topic 名称 CRC32 到 topic 句柄的映射 / Map from
                                ///< topic-name CRC32 to topic handle.
  QueueBase queue_;             ///< 输入字节 FIFO / Input byte FIFO.
  RawData parse_buff_;  ///< 当前包头和 payload 的暂存缓冲区 / Staging buffer holding the
                        ///< current header and payload.
};
}  // namespace LibXR
