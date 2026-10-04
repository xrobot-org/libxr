#pragma once

#include <atomic>
#include <concepts>
#include <cstddef>
#include <cstdint>

#include "libxr_cb.hpp"
#include "libxr_def.hpp"
#include "libxr_time.hpp"
#include "libxr_type.hpp"
#include "lockfree_list.hpp"
#include "mutex.hpp"
#include "queue.hpp"
#include "rbt.hpp"
#include "semaphore.hpp"
#include "thread.hpp"

namespace LibXR
{
/**
 * @brief topic 可承载 payload 的类型约束 / Type constraint for payloads carried by one
 *        topic
 * @tparam Data 待检查的 payload 类型 / Payload type to validate
 */
template <typename Data>
concept TopicPayload =
    !std::is_reference_v<Data> && !std::is_const_v<Data> && !std::is_volatile_v<Data> &&
    std::is_object_v<Data> && std::is_default_constructible_v<Data> &&
    std::is_copy_assignable_v<Data> && std::is_trivially_destructible_v<Data>;

/**
 * @brief 在模板上下文里断言 payload 类型满足 topic 契约 / Assert in template
 *        context that one payload type satisfies the topic contract
 * @tparam Data 待检查的 payload 类型 / Payload type to validate
 */
template <typename Data>
constexpr void CheckTopicPayload()
{
  static_assert(TopicPayload<Data>,
                "LibXR::Topic typed payload must be a non-cv/ref object type that is "
                "default-constructible, copy-assignable, and trivially destructible.");
}

/**
 * @class Topic
 * @brief 发布订阅主题 / Publish-subscribe topic
 *
 * 一个 `Topic` 表示一种精确类型的消息通道：它在每次发布时同步通知挂在上面的同步、
 * 异步、队列或回调订阅者，但自身不缓存最近一次消息。
 * One `Topic` represents one exact-typed message channel: it synchronously notifies
 * the attached synchronous, asynchronous, queued, or callback subscribers on each
 * publish, but does not cache the latest message itself.
 */
class Topic
{
  /**
   * @enum LockState
   * @brief topic 发布路径的内部锁状态 / Internal lock state of the topic publish path
   */
  enum class LockState : uint32_t
  {
    UNLOCKED = 0,  ///< 当前未持有发布锁 / The publish path is currently unlocked.
    LOCKED = 1,    ///< 当前通过原子快路径持有发布权 / The publish path is locked through
                   ///< the atomic fast path.
    USE_MUTEX = UINT32_MAX  ///< 当前主题改走互斥量串行化 / This topic currently
                            ///< serializes publishers through a mutex.
  };

 public:
  /**
   * @struct Block
   * @brief topic 运行时状态块 / Runtime state block of one topic
   *
   * @note 当前 topic 自身不缓存最近一次消息值；这里只保存类型契约、名称键值、发布串行化
   *       状态以及订阅链表。
   *       The topic no longer caches the latest payload value itself; this block
   *       keeps only the type contract, topic key, publish-serialization state,
   *       and subscriber list.
   */
  struct Block
  {
    std::atomic<LockState>
        busy;             ///< 发布路径串行化状态 / Publish-path serialization state.
    LockFreeList subers;  ///< 已挂接订阅者链表 / List of attached subscribers.
    TypeID::ID
        payload_type_id;    ///< 精确 payload 类型标识 / Exact payload type identifier.
    uint32_t payload_size;  ///< 该 topic 固定 payload 字节数 / Fixed payload size in
                            ///< bytes of this topic.
    uint32_t payload_alignment;  ///< 该 topic payload 所需对齐 / Required payload
                                 ///< alignment of this topic.
    uint32_t crc32;              ///< 主题名 CRC32 键 / CRC32 key of the topic name.
    Mutex* mutex;  ///< 多发布者主题使用的互斥量 / Mutex used by multi-publisher topics.
  };

#ifndef __DOXYGEN__
  /**
   * @struct PackedDataHeader
   * @brief 打包消息固定头 / Fixed header of one packed message
   */
  struct PackedDataHeader;

  /**
   * @class PackedData
   * @brief 带固定头和尾 CRC 的打包消息对象 / Packed message object with fixed header
   *        and trailing CRC
   * @tparam Data 负载类型 / Payload type
   */
  template <typename Data>
  class PackedData;
  static constexpr uint8_t PACKET_PREFIX =
      0x5A;  ///< 打包消息前缀字节 / Packed-message prefix byte.
  static constexpr uint8_t PACKET_VERSION =
      0x01;  ///< 打包消息协议版本 / Packed-message protocol version.
  static constexpr size_t PACK_BASE_SIZE =
      17;  ///< 固定非 payload 开销：16 字节头 + 1 字节尾 CRC8 / Fixed non-payload
           ///< overhead: 16-byte header plus 1-byte trailing CRC8.
#endif

  /**
   * @typedef TopicHandle
   * @brief 指向一个 topic 运行时状态块的句柄 / Handle pointing to one topic runtime
   *        state block
   */
  typedef RBTree<uint32_t>::Node<Block>* TopicHandle;

  /**
   * @struct MessageView
   * @brief 带时间戳和 payload 指针的只读消息视图 / Read-only message view carrying a
   *        timestamp and a payload pointer
   * @tparam Data payload 类型 / Payload type
   */
  template <typename Data>
  struct MessageView
  {
    static_assert(TopicPayload<Data>);
    MicrosecondTimestamp timestamp;  ///< 消息时间戳 / Message timestamp.
    Data* data;  ///< 指向本次发布 payload 对象的指针 / Pointer to the payload object of
                 ///< this publish.
  };

  /**
   * @struct RawMessageView
   * @brief 带时间戳和只读原始 payload 视图的消息 / Message carrying a timestamp and
   *        a read-only raw payload view
   *
   * @note 该视图只在当前回调调用期间有效；它不拥有 payload 字节，也不绕过 topic 的
   *       发布时类型契约。
   *       This view is valid only for the current callback invocation; it does not
   *       own the payload bytes and does not bypass the topic's publish-time type
   *       contract.
   */
  struct RawMessageView
  {
    MicrosecondTimestamp timestamp;  ///< 消息时间戳 / Message timestamp.
    ConstRawData payload;  ///< 本次发布 payload 的只读字节视图 / Read-only byte view of
                           ///< this publish payload.
  };

  /**
   * @struct Message
   * @brief 带时间戳和 payload 副本的消息对象 / Message object carrying a timestamp and
   *        a payload copy
   * @tparam Data payload 类型 / Payload type
   */
  template <typename Data>
  struct Message
  {
    static_assert(TopicPayload<Data>);
    MicrosecondTimestamp timestamp;  ///< 消息时间戳 / Message timestamp.
    Data data;                       ///< payload 对象副本 / Copied payload object.
  };

  /**
   * @brief 在普通上下文里锁住一个 topic 发布路径 / Lock one topic publish path in normal
   *        context
   * @param topic 目标 topic 句柄 / Target topic handle
   */
  static void Lock(TopicHandle topic);

  /**
   * @brief 在普通上下文里释放一个 topic 发布路径 / Unlock one topic publish path in
   *        normal context
   * @param topic 目标 topic 句柄 / Target topic handle
   */
  static void Unlock(TopicHandle topic);

  /**
   * @brief 在回调或 ISR 路径里锁住一个 topic 发布路径 / Lock one topic publish path from
   *        callback or ISR context
   * @param topic 目标 topic 句柄 / Target topic handle
   */
  static void LockFromCallback(TopicHandle topic);

  /**
   * @brief 在回调或 ISR 路径里释放一个 topic 发布路径 / Unlock one topic publish path
   *        from callback or ISR context
   * @param topic 目标 topic 句柄 / Target topic handle
   */
  static void UnlockFromCallback(TopicHandle topic);

  /**
   * @class Domain
   * @brief topic 所属的命名域 / Naming domain that groups topics
   */
  class Domain
  {
   public:
    /**
     * @brief 构造一个 topic 域 / Construct one topic domain
     * @param name 域名称 / Domain name
     */
    Domain(const char* name);

    RBTree<uint32_t>::Node<RBTree<uint32_t>>*
        node_;  ///< 该域在全局域表里的节点 / This domain's node inside the global domain
                ///< tree.
  };

  /**
   * @enum SuberType
   * @brief topic 支持的订阅者种类 / Subscriber kinds supported by a topic
   */
  enum class SuberType : uint8_t
  {
    SYNC,      ///< 同步等待型订阅者 / Synchronous wait-based subscriber.
    ASYNC,     ///< 异步本地缓冲型订阅者 / Asynchronous local-buffer subscriber.
    QUEUE,     ///< 队列转发型订阅者 / Queue-forwarding subscriber.
    CALLBACK,  ///< 回调执行型订阅者 / Callback-executing subscriber.
  };

  /**
   * @struct SuberBlock
   * @brief 所有订阅块共用的公共头 / Common header shared by all subscriber blocks
   */
  struct SuberBlock
  {
    SuberType type;  ///< 订阅块的具体种类 / Concrete kind of this subscriber block.
  };

  /**
   * @struct SyncBlock
   * @brief 同步订阅者挂在 topic 链表里的数据块 / Subscriber block used by one
   *        synchronous subscriber inside the topic list
   */
  struct SyncBlock;

  /**
   * @class SyncSubscriber
   * @brief 通过 `Wait()` 接收消息的同步订阅者 / Synchronous subscriber receiving
   *        messages via `Wait()`
   * @tparam Data 订阅的数据类型 / Subscribed data type
   */
  template <typename Data>
  class SyncSubscriber;

  /**
   * @enum ASyncSubscriberState
   * @brief 异步订阅者本地缓冲区状态 / State of one asynchronous subscriber local buffer
   */
  enum class ASyncSubscriberState : uint32_t;

  /**
   * @struct ASyncBlock
   * @brief 异步订阅者挂在 topic 链表里的数据块 / Subscriber block used by one
   *        asynchronous subscriber inside the topic list
   */
  struct ASyncBlock;

  /**
   * @class ASyncSubscriber
   * @brief 先等待再主动取数据的异步订阅者 / Asynchronous subscriber that waits first
   *        and then pulls data explicitly
   * @tparam Data 订阅的数据类型 / Subscribed data type
   */
  template <typename Data>
  class ASyncSubscriber;

  /**
   * @struct QueueBlock
   * @brief 队列订阅者挂在 topic 链表里的数据块 / Subscriber block used by one queued
   *        subscriber inside the topic list
   */
  struct QueueBlock;

  /**
   * @class QueuedSubscriber
   * @brief 把每次发布推入队列的订阅者 / Subscriber that pushes each publish into a queue
   */
  class QueuedSubscriber;

  /**
   * @class Callback
   * @brief 每次发布时直接执行函数的回调订阅句柄 / Callback subscription handle that
   *        runs a function on each publish
   */
  class Callback;

  /**
   * @struct CallbackBlock
   * @brief 回调订阅者挂在 topic 链表里的数据块 / Subscriber block used by one callback
   *        subscriber inside the topic list
   */
  struct CallbackBlock;

  /**
   * @class Server
   * @brief 把字节流解析成 packet 并投递到 topic 的 parser / Parser that turns byte
   *        streams into packets and delivers them into topics
   */
  class Server;

  /**
   * @brief 注册一个回调订阅者 / Register one callback subscriber
   * @param cb 要注册的回调句柄 / Callback handle to register
   */
  void RegisterCallback(Callback& cb);

  /**
   * @brief 读取当前时间戳 / Read the current timestamp
   * @return 当前时间戳 / Current timestamp
   */
  static MicrosecondTimestamp NowTimestamp();

  /**
   * @brief 构造一个空 topic 视图 / Construct one empty topic view
   */
  Topic();

  /**
   * @brief 用显式类型契约构造或查找一个 topic / Construct or look up one topic with
   *        an explicit runtime type contract
   * @param name topic 名称 / Topic name
   * @param payload_type_id payload 类型标识 / Payload type identifier
   * @param payload_size 该 topic 固定 payload 字节数 / Fixed payload size of this topic
   * @param payload_alignment 该 topic payload 所需对齐，必须是非零 2 次幂 /
   *        Required payload alignment of this topic; must be a non-zero power of
   *        two
   * @param domain 可选主题域 / Optional topic domain
   * @param multi_publisher 是否允许多发布者串行化 / Whether to allow serialized
   * multi-publisher use
   * @note 同名 topic 已存在但类型契约不同，或它以单发布者创建而本次要求多发布者时，
   *       在所有构建类型中进入致命错误。
   *       If a topic with this name exists with a different type contract, or was
   *       created for a single publisher while multi_publisher is requested, a fatal
   *       error is raised in every build type.
   */
  Topic(const char* name, TypeID::ID payload_type_id, size_t payload_size,
        size_t payload_alignment, Domain* domain = nullptr, bool multi_publisher = false);

  /**
   * @brief 用精确类型创建或查找一个 topic / Create or look up one topic using one
   *        exact payload type
   * @tparam Data payload 类型 / Payload type
   * @param name topic 名称 / Topic name
   * @param domain 可选主题域 / Optional topic domain
   * @param multi_publisher 是否允许多发布者串行化 / Whether to allow serialized
   *        multi-publisher use
   * @return 对应的 topic 视图 / Topic view of the requested topic
   */
  template <typename Data>
  static Topic CreateTopic(const char* name, Domain* domain = nullptr,
                           bool multi_publisher = false)
  {
    CheckTopicPayload<Data>();
    return Topic(name, TypeID::GetID<Data>(), sizeof(Data), alignof(Data), domain,
                 multi_publisher);
  }

  /**
   * @brief 从已有句柄构造一个 topic 视图 / Construct one topic view from an existing
   *        handle
   * @param topic 既有 topic 句柄 / Existing topic handle
   */
  Topic(TopicHandle topic);

  /**
   * @brief 按名称查找一个已存在 topic / Find one existing topic by name
   * @param name topic 名称 / Topic name
   * @param domain 可选主题域 / Optional topic domain
   * @return 找到则返回句柄，否则返回空 / Returns the handle when found, otherwise null
   */
  static TopicHandle Find(const char* name, Domain* domain = nullptr);

  /**
   * @brief 按精确类型查找或创建一个 topic / Find or create one topic with an exact
   *        payload type
   * @tparam Data payload 类型 / Payload type
   * @param name topic 名称 / Topic name
   * @param domain 可选主题域 / Optional topic domain
   * @param multi_publisher 是否允许多发布者串行化 / Whether to allow serialized
   *        multi-publisher use
   * @return topic 句柄 / Topic handle
   * @note 已有 topic 的冲突检查与构造函数相同，在所有构建类型中生效。
   *       Conflicts with an existing topic are checked as in the constructor, in every
   *       build type.
   */
  template <typename Data>
  static TopicHandle FindOrCreate(const char* name, Domain* domain = nullptr,
                                  bool multi_publisher = false)
  {
    CheckTopicPayload<Data>();
    auto topic = Find(name, domain);
    if (topic != nullptr)
    {
      RequireTypeContract(topic, TypeID::GetID<Data>(), sizeof(Data), alignof(Data));
      if (multi_publisher)
      {
        RequireMultiPublisher(topic);
      }
    }
    else
    {
      topic = CreateTopic<Data>(name, domain, multi_publisher).block_;
    }
    return topic;
  }

  /**
   * @brief 获取该 topic 固定 payload 字节数 / Get the fixed payload size of this topic
   * @return payload 字节数 / Payload size in bytes
   */
  [[nodiscard]] size_t PayloadSize() const
  {
    ASSERT(block_ != nullptr);
    return block_->data_.payload_size;
  }

  /**
   * @brief 获取该 topic payload 的对齐要求 / Get the payload alignment requirement of
   *        this topic
   * @return payload 对齐要求 / Payload alignment requirement
   */
  [[nodiscard]] size_t PayloadAlignment() const
  {
    ASSERT(block_ != nullptr);
    return block_->data_.payload_alignment;
  }

  /**
   * @brief 在普通上下文里发布一条消息，并自动取当前时间戳 / Publish one message in
   *        normal context and stamp it with the current time
   * @tparam Data payload 类型 / Payload type
   * @param data 待发布的 payload 对象 / Payload object to publish
   */
  template <typename Data>
  void Publish(Data& data)
  {
    PublishTyped(data, NowTimestamp(), false, false);
  }

  /**
   * @brief 在普通上下文里按指定时间戳发布一条消息 / Publish one message in normal
   *        context with an explicit timestamp
   * @tparam Data payload 类型 / Payload type
   * @param data 待发布的 payload 对象 / Payload object to publish
   * @param timestamp 消息时间戳 / Message timestamp
   */
  template <typename Data>
  void Publish(Data& data, MicrosecondTimestamp timestamp)
  {
    PublishTyped(data, timestamp, false, false);
  }

  /**
   * @brief 在回调或 ISR 路径里发布一条消息，并自动取当前时间戳 / Publish one message
   *        from callback or ISR context and stamp it with the current time
   * @tparam Data payload 类型 / Payload type
   * @param data 待发布的 payload 对象 / Payload object to publish
   * @param in_isr 当前是否位于 ISR / Whether the current path is in ISR context
   */
  template <typename Data>
  void PublishFromCallback(Data& data, bool in_isr)
  {
    PublishTyped(data, NowTimestamp(), true, in_isr);
  }

  /**
   * @brief 在回调或 ISR 路径里按指定时间戳发布一条消息 / Publish one message from
   *        callback or ISR context with an explicit timestamp
   * @tparam Data payload 类型 / Payload type
   * @param data 待发布的 payload 对象 / Payload object to publish
   * @param timestamp 消息时间戳 / Message timestamp
   * @param in_isr 当前是否位于 ISR / Whether the current path is in ISR context
   */
  template <typename Data>
  void PublishFromCallback(Data& data, MicrosecondTimestamp timestamp, bool in_isr)
  {
    PublishTyped(data, timestamp, true, in_isr);
  }

  /**
   * @brief 供 packet/server 路径按字节发布一条消息 / Publish one message from the
   *        packet/server path using bytes already arranged as the exact payload object
   * @param payload_addr 已满足对齐的 payload 起始地址 / Start address of the
   * already-aligned payload object
   * @param payload_size payload 字节数，必须等于 topic 固定大小 / Payload size, must
   * equal the fixed topic size
   * @param timestamp 这条消息的时间戳 / Timestamp of this message
   */
  void PublishBytesFromServer(void* payload_addr, size_t payload_size,
                              MicrosecondTimestamp timestamp)
  {
    PublishServerBytes(payload_addr, payload_size, timestamp, false, false);
  }

  /**
   * @brief 供回调/ISR 上下文里的 packet/server 路径按字节发布一条消息 / Publish one
   *        packet/server message from callback/ISR context using bytes already arranged
   * as the exact payload object
   * @param payload_addr 已满足对齐的 payload 起始地址 / Start address of the
   * already-aligned payload object
   * @param payload_size payload 字节数，必须等于 topic 固定大小 / Payload size, must
   * equal the fixed topic size
   * @param timestamp 这条消息的时间戳 / Timestamp of this message
   * @param in_isr 当前是否位于 ISR / Whether the current path is in ISR context
   */
  void PublishBytesFromServerCallback(void* payload_addr, size_t payload_size,
                                      MicrosecondTimestamp timestamp, bool in_isr)
  {
    PublishServerBytes(payload_addr, payload_size, timestamp, true, in_isr);
  }

  /**
   * @brief 将一个精确类型消息打包成 packet / Pack one exact-typed message into one
   *        packet using the topic's runtime contract and current timestamp
   * @tparam Data 负载类型 / Payload type
   * @param data 待打包 payload / Payload to pack
   * @param packet 输出 packet 对象 / Output packed message object
   * @return 操作结果错误码 / Error code
   */
  template <typename Data>
  ErrorCode PackData(const Data& data, PackedData<Data>& packet)
  {
    return PackData(data, packet, NowTimestamp());
  }

  /**
   * @brief 将一个精确类型消息按指定时间戳打包成 packet / Pack one exact-typed message
   *        into one packet with the given timestamp
   * @tparam Data 负载类型 / Payload type
   * @param data 待打包 payload / Payload to pack
   * @param packet 输出 packet 对象 / Output packed message object
   * @param timestamp 指定时间戳 / Explicit timestamp
   * @return 操作结果错误码 / Error code
   */
  template <typename Data>
  ErrorCode PackData(const Data& data, PackedData<Data>& packet,
                     MicrosecondTimestamp timestamp);

  /**
   * @brief 将一段 raw payload 按当前 topic 元数据打包成 packet / Pack a raw payload
   *        byte view into one packet using the current topic metadata and current
   * timestamp
   * @param data 待打包 payload 字节 / Payload bytes to pack
   * @param packet 输出 packet 缓冲区 / Output packet buffer
   * @return 操作结果错误码 / Error code
   */
  ErrorCode PackRaw(ConstRawData data, RawData packet)
  {
    return PackRaw(data, packet, NowTimestamp());
  }

  /**
   * @brief 将一段 raw payload 按当前 topic 元数据和指定时间戳打包成 packet / Pack a raw
   *        payload byte view into one packet using the current topic metadata and
   * timestamp
   * @param data 待打包 payload 字节 / Payload bytes to pack
   * @param packet 输出 packet 缓冲区 / Output packet buffer
   * @param timestamp 指定时间戳 / Explicit timestamp
   * @return 操作结果错误码 / Error code
   */
  ErrorCode PackRaw(ConstRawData data, RawData packet, MicrosecondTimestamp timestamp)
  {
    if (block_ == nullptr || data.addr_ == nullptr || packet.addr_ == nullptr)
    {
      return ErrorCode::PTR_NULL;
    }

    if (data.size_ != block_->data_.payload_size)
    {
      return ErrorCode::SIZE_ERR;
    }

    if (packet.size_ < PACK_BASE_SIZE + data.size_)
    {
      return ErrorCode::NO_BUFF;
    }

    PackBytes(block_->data_.crc32, packet, timestamp, data);
    return ErrorCode::OK;
  }

  /**
   * @brief 等待指定名称的 topic 出现 / Wait until a topic with the given name exists
   * @param name topic 名称 / Topic name
   * @param timeout 等待超时，默认永久等待 / Timeout in ticks, default wait forever
   * @param domain 可选主题域 / Optional topic domain
   * @return 找到则返回句柄，否则返回空 / Returns the handle when found, otherwise null
   */
  static TopicHandle WaitTopic(const char* name, uint32_t timeout = UINT32_MAX,
                               Domain* domain = nullptr);

  /**
   * @brief 把 topic 视图转换为底层句柄 / Convert this topic view to the underlying
   *        handle
   * @return topic 句柄 / Topic handle
   */
  operator TopicHandle() { return block_; }

  /**
   * @brief 读取 topic 键值 / Read the key value of this topic
   * @return topic 键值 / Topic key value
   */
  uint32_t GetKey() const;

 private:
  TopicHandle block_ = nullptr;  ///< 当前 topic 视图绑定的状态块 / Runtime state block
                                 ///< bound to the current topic view.

  static inline RBTree<uint32_t>* domain_ =
      nullptr;  ///< 全局 topic 域注册表 / Global registry of topic domains.
  static inline Domain* def_domain_ = nullptr;  ///< 缺省 topic 域 / Default topic domain.

  /**
   * @brief 确保全局域注册表已创建 / Ensure the global domain registry exists
   */
  static void EnsureDomainRegistry();

  /**
   * @brief 确保默认域已创建 / Ensure the default domain exists
   * @return 默认域指针 / Default domain pointer
   */
  static Domain* EnsureDefaultDomain();

  /**
   * @brief 校验 server 侧字节发布前提 / Check the preconditions of one server-side byte
   *        publish
   * @param topic 目标 topic / Target topic
   * @param payload_addr 已满足对齐的 payload 地址 / Already-aligned payload address
   * @param payload_size payload 字节数 / Payload size in bytes
   */
  static void CheckServerPublishContract(TopicHandle topic, void* payload_addr,
                                         size_t payload_size)
  {
    ASSERT(topic != nullptr);
    ASSERT(payload_addr != nullptr);
    ASSERT(payload_size == topic->data_.payload_size);
    ASSERT(topic->data_.payload_alignment != 0);
    ASSERT(reinterpret_cast<uintptr_t>(payload_addr) % topic->data_.payload_alignment ==
           0);
  }

  /**
   * @brief 检查订阅者看到的精确 payload 类型与 topic 契约一致 / Check that the exact
   *        payload type seen by a subscriber matches the topic contract
   * @tparam Data 订阅类型 / Subscriber payload type
   * @param topic 目标 topic 视图 / Target topic view
   * @note 类型检查在所有构建类型中生效，见 `RequireTypeContract()`。
   *       The type check is active in every build type; see `RequireTypeContract()`.
   */
  template <typename Data>
  static void CheckSubscriberType(Topic topic)
  {
    CheckTopicPayload<Data>();
    ASSERT(topic.block_ != nullptr);
    RequireTypeContract(topic.block_, TypeID::GetID<Data>(), sizeof(Data), alignof(Data));
  }

  /**
   * @brief 初始化期检查 topic 类型契约，在所有构建类型中生效 / Check one topic type
   *        contract at initialization time, active in every build type
   * @param topic 目标 topic 句柄 / Target topic handle
   * @param payload_type_id 调用方要求的精确 payload 类型标识 / Exact payload type ID
   *        required by the caller
   * @param payload_size 调用方要求的 payload 字节数 / Payload size required by the
   *        caller
   * @param payload_alignment 调用方要求的 payload 对齐 / Payload alignment required by
   *        the caller
   * @note 不一致时断言失败并进入致命错误；topic 句柄和两边的类型标识可在栈帧中查看。
   *       On mismatch, the assertion fails and raises a fatal error; the topic handle
   *       and both type IDs can be inspected in the stack frames.
   */
  static void RequireTypeContract(TopicHandle topic, TypeID::ID payload_type_id,
                                  size_t payload_size, size_t payload_alignment);

  /**
   * @brief 初始化期检查回调 payload 类型，在所有构建类型中生效 / Check one callback
   *        payload type at initialization time, active in every build type
   * @param topic 目标 topic 句柄 / Target topic handle
   * @param payload_type_id 回调要求的精确 payload 类型标识 / Exact payload type ID
   *        required by the callback
   * @note 不一致时断言失败并进入致命错误；topic 句柄和两边的类型标识可在栈帧中查看。
   *       On mismatch, the assertion fails and raises a fatal error; the topic handle
   *       and both type IDs can be inspected in the stack frames.
   */
  static void RequireCallbackType(TopicHandle topic, TypeID::ID payload_type_id);

  /**
   * @brief 初始化期检查已有 topic 能否多发布者使用，在所有构建类型中生效 / Check at
   *        initialization time that one existing topic allows multiple publishers,
   *        active in every build type
   * @param topic 目标 topic 句柄 / Target topic handle
   * @note 该 topic 以单发布者创建时断言失败并进入致命错误。
   *       If the topic was created for a single publisher, the assertion fails and
   *       raises a fatal error.
   */
  static void RequireMultiPublisher(TopicHandle topic);

  /**
   * @brief 为订阅者分配一个长期存在的本地接收对象 / Allocate one long-lived local
   *        receive object for a subscriber
   * @tparam Data 对象类型 / Object type
   * @return 新分配对象的地址 / Address of the newly allocated object
   */
  template <typename Data>
  static void* AllocateSubscriberBuffer()
  {
    CheckTopicPayload<Data>();
    return new Data;
  }

  /**
   * @brief 按精确类型把一份 payload 拷到订阅者缓冲区 / Copy one payload into a
   *        subscriber buffer using the exact type
   * @tparam Data payload 类型 / Payload type
   * @param dst 订阅者缓冲区地址 / Subscriber buffer address
   * @param payload_addr 本次发布 payload 地址 / Address of the payload object of the
   *        current publish
   */
  template <typename Data>
  static void CopyPayload(void* dst, void* payload_addr)
  {
    CheckTopicPayload<Data>();
    ASSERT(dst != nullptr);
    ASSERT(payload_addr != nullptr);
    *reinterpret_cast<Data*>(dst) = *reinterpret_cast<Data*>(payload_addr);
  }

  /**
   * @brief 强类型发布入口的共享实现 / Shared implementation of typed publish entry
   *        points
   * @tparam Data payload 类型 / Payload type
   * @param data 待发布 payload / Payload to publish
   * @param timestamp 消息时间戳 / Message timestamp
   * @param from_callback 是否来自回调路径 / Whether this publish comes from callback
   *        path
   * @param in_isr 当前是否位于 ISR / Whether the current path is in ISR context
   */
  template <typename Data>
  void PublishTyped(Data& data, MicrosecondTimestamp timestamp, bool from_callback,
                    bool in_isr)
  {
    CheckTopicPayload<Data>();

    if (from_callback)
    {
      LockFromCallback(block_);
    }
    else
    {
      Lock(block_);
    }

    CheckPublishContract(block_, TypeID::GetID<Data>(), sizeof(Data), alignof(Data));
    DispatchSubscribers(block_, timestamp, &data, from_callback, in_isr);

    if (from_callback)
    {
      UnlockFromCallback(block_);
    }
    else
    {
      Unlock(block_);
    }
  }

  /**
   * @brief 校验一次强类型发布的运行时契约 / Check the runtime contract of one typed
   *        publish
   * @param topic 目标 topic 句柄 / Target topic handle
   * @param payload_type_id 本次发布的精确 payload 类型标识 / Exact payload type ID of
   *        this publish
   * @param payload_size 本次发布的 payload 固定字节数 / Fixed payload size of this
   *        publish
   * @param payload_alignment 本次发布的 payload 对齐要求 / Payload alignment
   *        requirement of this publish
   */
  static void CheckPublishContract(TopicHandle topic, TypeID::ID payload_type_id,
                                   size_t payload_size, size_t payload_alignment);

  /**
   * @brief 将一段 payload 字节和 topic 元数据拼成 packet / Pack one payload byte range
   *        together with topic metadata into one packet
   * @param topic_name_crc32 目标 topic 的 CRC32 键 / CRC32 key of the target topic
   * @param buffer 输出原始缓冲区 / Output raw buffer
   * @param timestamp 消息时间戳 / Message timestamp
   * @param data 待打包的 payload 字节 / Payload bytes to pack
   */
  static void PackBytes(uint32_t topic_name_crc32, RawData buffer,
                        MicrosecondTimestamp timestamp, ConstRawData data);

  /**
   * @brief 将一条消息分发给一个订阅块 / Dispatch one message to one subscriber block
   * @param block 目标订阅块 / Target subscriber block
   * @param timestamp 消息时间戳 / Message timestamp
   * @param payload_addr 本次发布 payload 地址 / Address of the payload object of the
   *        current publish
   * @param from_callback 是否来自回调路径 / Whether this publish comes from callback
   *        path
   * @param in_isr 当前是否位于 ISR / Whether the current path is in ISR context
   */
  static void DispatchSubscriber(SuberBlock& block, MicrosecondTimestamp timestamp,
                                 void* payload_addr, bool from_callback, bool in_isr);

  /**
   * @brief 将一条消息分发给一个 topic 上的全部订阅者 / Dispatch one message to all
   *        subscribers attached to one topic
   * @param topic 目标 topic 句柄 / Target topic handle
   * @param timestamp 消息时间戳 / Message timestamp
   * @param payload_addr 本次发布 payload 地址 / Address of the payload object of the
   *        current publish
   * @param from_callback 是否来自回调路径 / Whether this publish comes from callback
   *        path
   * @param in_isr 当前是否位于 ISR / Whether the current path is in ISR context
   */
  static void DispatchSubscribers(TopicHandle topic, MicrosecondTimestamp timestamp,
                                  void* payload_addr, bool from_callback, bool in_isr);

  /**
   * @brief `PublishBytesFromServer*()` 的共享实现 / Shared implementation behind
   *        `PublishBytesFromServer*()`
   * @param payload_addr 已满足对齐的 payload 地址 / Already-aligned payload address
   * @param payload_size payload 字节数 / Payload size in bytes
   * @param timestamp 消息时间戳 / Message timestamp
   * @param from_callback 是否来自回调发布路径 / Whether this publish comes from callback
   * path
   * @param in_isr 当前是否位于 ISR / Whether the current path is in ISR context
   */
  void PublishServerBytes(void* payload_addr, size_t payload_size,
                          MicrosecondTimestamp timestamp, bool from_callback, bool in_isr)
  {
    CheckServerPublishContract(block_, payload_addr, payload_size);

    if (from_callback)
    {
      LockFromCallback(block_);
    }
    else
    {
      Lock(block_);
    }

    DispatchSubscribers(block_, timestamp, payload_addr, from_callback, in_isr);

    if (from_callback)
    {
      UnlockFromCallback(block_);
    }
    else
    {
      Unlock(block_);
    }
  }
};
}  // namespace LibXR

#include "packet/packet.hpp"
#include "server/server.hpp"
#include "subscriber/async.hpp"
#include "subscriber/callback.hpp"
#include "subscriber/queue.hpp"
#include "subscriber/sync.hpp"
