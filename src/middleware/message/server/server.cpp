#include "server.hpp"

#include <cstddef>
#include <cstdint>

#include "../packet/packet.hpp"
#include "crc.hpp"
#include "libxr_mem.hpp"

using namespace LibXR;

Topic::Server::Server(size_t buffer_length)
    : topic_map_([](const uint32_t& a, const uint32_t& b) { return (a > b) - (a < b); }),
      queue_(1, buffer_length)
{
  ASSERT(buffer_length > PACK_BASE_SIZE);
  parse_buff_.size_ = buffer_length;
  parse_buff_.addr_ =
      new (std::align_val_t(LibXR::CACHE_LINE_SIZE)) uint8_t[parse_buff_.size_];
}

void Topic::Server::Register(TopicHandle topic)
{
  ASSERT(topic != nullptr);
  ASSERT(topic->data_.payload_size != 0);
  ASSERT(topic->data_.payload_alignment != 0);
  ASSERT(topic->data_.payload_alignment <= LibXR::CACHE_LINE_SIZE);

  ASSERT(topic->data_.payload_size + PACK_BASE_SIZE <= parse_buff_.size_);

  auto* node = new RBTree<uint32_t>::Node<TopicHandle>(topic);
  topic_map_.Insert(*node, topic->key);
}

size_t Topic::Server::ParseData(ConstRawData data)
{
  return ParseDataRaw(data, false, false);
}

size_t Topic::Server::ParseDataFromCallback(ConstRawData data, bool in_isr)
{
  return ParseDataRaw(data, true, in_isr);
}

size_t Topic::Server::ParseDataRaw(ConstRawData data, bool from_callback, bool in_isr)
{
  size_t count = 0;
  const auto* bytes = static_cast<const uint8_t*>(data.addr_);
  size_t remaining = data.size_;

  // 输入可能比队列剩余空间大：按剩余空间分段压入，每段之后先解析腾出空间。解析完队里最多
  // 剩一个没收全的包，它比队列短，所以下一段总有空位。
  // Input may exceed the free queue space: push it in pieces that fit and parse after
  // each piece to make room. After parsing at most one incomplete packet stays queued,
  // and it is shorter than the queue, so the next piece always has room.
  while (remaining > 0)
  {
    const size_t piece = LibXR::min(remaining, queue_.EmptySize());
    ASSERT(piece > 0);
    (void)queue_.PushBatchBytes(bytes, piece);
    bytes += piece;
    remaining -= piece;
    count += ParseQueued(from_callback, in_isr);
  }
  return count;
}

size_t Topic::Server::ParseQueued(bool from_callback, bool in_isr)
{
  size_t count = 0;
  while (SyncToPacketStart())
  {
    const Step step = HandleFrontPacket(from_callback, in_isr);
    if (step == Step::NEED_MORE)
    {
      break;
    }
    if (step == Step::DELIVERED)
    {
      count++;
    }
  }
  return count;
}

bool Topic::Server::SyncToPacketStart()
{
  while (queue_.Size() > 0)
  {
    uint8_t prefix = 0;
    queue_.PeekBytes(&prefix);
    if (prefix == PACKET_PREFIX)
    {
      return true;
    }
    queue_.PopBytes();
  }
  return false;
}

Topic::Server::Step Topic::Server::HandleFrontPacket(bool from_callback, bool in_isr)
{
  constexpr size_t HEADER_SIZE = sizeof(PackedDataHeader);
  auto* header = reinterpret_cast<PackedDataHeader*>(parse_buff_.addr_);

  // 1. 头部收全后先只看不取。头部 CRC 不对，说明队首的前缀字节只是数据里碰巧出现的
  //    0x5A：只丢这一个字节，从下一个字节接着找，后面的真包一个字节都不少。
  //    Peek the header once it is complete. A bad header CRC means the prefix at the
  //    front is a 0x5A that merely occurs in the data: drop that one byte and search
  //    on from the next, so no byte of a following real packet is lost.
  if (queue_.Size() < HEADER_SIZE)
  {
    return Step::NEED_MORE;
  }
  queue_.PeekBatchBytes(parse_buff_.addr_, HEADER_SIZE);
  if (!CRC8::Verify(parse_buff_.addr_, HEADER_SIZE))
  {
    queue_.PopBytes();
    return Step::DISCARDED;
  }

  // 2. 按头部声明的长度等整包到齐，再校验整包 CRC。比队列还长的包收不下，按假前缀处理；
  //    整包 CRC 不对同样只丢一个字节。
  //    Wait for the whole packet by the declared length, then check the packet CRC. A
  //    packet longer than the queue cannot be held and is treated as a false prefix; a
  //    bad packet CRC likewise drops only one byte.
  const uint32_t data_len = header->GetDataLen();
  const size_t packet_size = data_len + PACK_BASE_SIZE;
  if (packet_size > queue_.MaxSize())
  {
    queue_.PopBytes();
    return Step::DISCARDED;
  }
  if (queue_.Size() < packet_size)
  {
    return Step::NEED_MORE;
  }
  queue_.PeekBatchBytes(parse_buff_.addr_, packet_size);
  if (!CRC8::Verify(parse_buff_.addr_, packet_size))
  {
    queue_.PopBytes();
    return Step::DISCARDED;
  }

  // 3. 整包校验通过，是一个真包，从队列取走；后面的字节从下一个包开始。版本不对或 topic
  //    没注册的真包整包跳过，不在它的 payload 里重新找前缀。
  //    Both CRCs pass, so this is a real packet: remove it from the queue, and the
  //    following bytes start at the next packet. A real packet with another version or
  //    an unregistered topic is skipped whole, without searching its payload for a
  //    prefix.
  queue_.PopBatchBytes(nullptr, packet_size);
  if (header->version != PACKET_VERSION)
  {
    return Step::DISCARDED;
  }
  auto* node = topic_map_.Search<TopicHandle>(header->topic_name_crc32);
  if (node == nullptr)
  {
    return Step::DISCARDED;
  }

  TopicHandle topic_handle = *node;
  const MicrosecondTimestamp timestamp = header->GetTimestamp();
  const auto target_size = topic_handle->data_.payload_size;
  auto* payload_addr = reinterpret_cast<uint8_t*>(parse_buff_.addr_) + HEADER_SIZE;
  void* publish_addr = payload_addr;
  if (reinterpret_cast<uintptr_t>(payload_addr) % topic_handle->data_.payload_alignment !=
      0)
  {
    // 暂存缓冲区按缓存行对齐，挪到开头即满足对齐 / The staging buffer is cache-line
    // aligned, so moving to its start satisfies the alignment.
    publish_addr = parse_buff_.addr_;
    LibXR::Memory::FastMove(publish_addr, payload_addr,
                            data_len >= target_size ? target_size : data_len);
  }

  auto topic = Topic(topic_handle);
  if (from_callback)
  {
    topic.PublishBytesFromServerCallback(publish_addr, target_size, timestamp, in_isr);
  }
  else
  {
    topic.PublishBytesFromServer(publish_addr, target_size, timestamp);
  }
  return Step::DELIVERED;
}
