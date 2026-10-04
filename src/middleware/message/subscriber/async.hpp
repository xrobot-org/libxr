#pragma once

#include "../topic.hpp"

namespace LibXR
{
/**
 * @enum Topic::ASyncSubscriberState
 * @brief 异步订阅者本地缓冲区的状态 / State of the async subscriber's local buffer
 */
enum class Topic::ASyncSubscriberState : uint32_t
{
  IDLE = 0,     ///< 当前没有等待，也没有待消费的新数据 / No wait is pending and no unread
                ///< data is buffered.
  WAITING = 1,  ///< 等待下一次发布填充本地缓冲区 / Waiting for the next publish to fill
                ///< the local buffer.
  DATA_READY = UINT32_MAX  ///< 本地缓冲区已有一份待消费的新数据 / One unread fresh sample
                           ///< is buffered locally.
};

/**
 * @struct Topic::ASyncBlock
 * @brief 异步订阅者自己挂的数据块 / Data block owned by one asynchronous subscriber
 */
struct Topic::ASyncBlock : public Topic::SuberBlock
{
  void* buff_addr;  ///< 长期存在的本地接收对象地址 / Long-lived local receive object
                    ///< address.
  void (*copy_payload)(
      void* dst,
      void* payload_addr);  ///< 按订阅精确类型执行负载拷贝的适配函数 / Adapter that
                            ///< copies one payload using the subscriber's exact type.
  MicrosecondTimestamp
      timestamp;  ///< 最近接收的消息时间戳 / Latest received message timestamp.
  std::atomic<ASyncSubscriberState> state =
      ASyncSubscriberState::IDLE;  ///< 当前异步订阅状态 / Current async subscriber state.
};

/**
 * @class Topic::ASyncSubscriber
 * @brief 先 `StartWaiting()`，再自己来取数据的订阅者 / Subscriber that first calls
 *        `StartWaiting()` and later pulls the data itself
 * @tparam Data 订阅的数据类型 / Subscribed data type
 */
template <typename Data>
class Topic::ASyncSubscriber
{
 public:
  /**
   * @brief 通过主题名称构造异步订阅者 / Construct an asynchronous subscriber by topic
   * name
   * @param name 订阅的主题名称 / Name of the subscribed topic
   * @param domain 可选的域指针 / Optional domain pointer
   * @note 包含初始化期动态内存分配，订阅者应长期存在 / Contains initialization-time
   * dynamic allocation; subscribers are expected to be long-lived
   */
  ASyncSubscriber(const char* name, Domain* domain = nullptr)
      : ASyncSubscriber(Topic(WaitTopic(name, UINT32_MAX, domain)))
  {
  }

  /**
   * @brief 通过 `Topic` 句柄构造异步订阅者 / Construct an asynchronous subscriber from a
   *        `Topic` handle
   * @param topic 订阅的主题 / Subscribed topic
   * @note 包含初始化期动态内存分配，订阅者应长期存在 / Contains initialization-time
   * dynamic allocation; subscribers are expected to be long-lived
   */
  ASyncSubscriber(Topic topic)
  {
    Topic::CheckSubscriberType<Data>(topic);

    block_ = new LockFreeList::Node<ASyncBlock>;
    block_->data_.type = SuberType::ASYNC;
    block_->data_.timestamp = MicrosecondTimestamp();
    block_->data_.buff_addr = Topic::AllocateSubscriberBuffer<Data>();
    block_->data_.copy_payload = &Topic::CopyPayload<Data>;
    topic.block_->data_.subers.Add(*block_);
  }

  /**
   * @brief 禁止拷贝异步订阅者 / Copy construction is disabled for asynchronous
   *        subscribers
   * @param other 待拷贝的异步订阅者 / Asynchronous subscriber to copy from
   */
  ASyncSubscriber(const ASyncSubscriber& other) = delete;

  /**
   * @brief 禁止拷贝赋值异步订阅者 / Copy assignment is disabled for asynchronous
   *        subscribers
   * @param other 待拷贝的异步订阅者 / Asynchronous subscriber to copy from
   * @return 当前异步订阅者 / Returns the current asynchronous subscriber
   */
  ASyncSubscriber& operator=(const ASyncSubscriber& other) = delete;

  /**
   * @brief 移动构造异步订阅者 / Move-construct one asynchronous subscriber
   * @param other 被转移的异步订阅者 / Asynchronous subscriber to move from
   * @note 这里只移动本地句柄指针；底层订阅块仍留在 topic 的订阅链表里，`other`
   *       会被清成空句柄 /
   *       This moves only the local handle pointer; the underlying subscriber
   *       block stays registered in the topic list and `other` becomes empty
   */
  ASyncSubscriber(ASyncSubscriber&& other) noexcept : block_(other.block_)
  {
    other.block_ = nullptr;
  }

  /**
   * @brief 移动赋值异步订阅者 / Move-assign one asynchronous subscriber
   * @param other 被转移的异步订阅者 / Asynchronous subscriber to move from
   * @return 当前异步订阅者 / Returns the current asynchronous subscriber
   * @note 这里只改当前包装对象指向的订阅块，不会注销旧块；底层订阅块仍留在 topic
   *       的订阅链表里，`other` 会被清成空句柄 /
   *       This only changes which subscriber block the current wrapper points
   *       to and does not unregister the old block; the underlying subscriber
   *       blocks stay in the topic list and `other` becomes empty
   */
  ASyncSubscriber& operator=(ASyncSubscriber&& other) noexcept
  {
    if (this != &other)
    {
      block_ = other.block_;
      other.block_ = nullptr;
    }
    return *this;
  }

  /**
   * @brief 检查数据是否可用 / Check whether data is available
   * @return 数据已准备好返回 `true`，否则返回 `false` / Returns `true` if data is ready,
   * otherwise `false`
   */
  bool Available()
  {
    return block_->data_.state.load(std::memory_order_acquire) ==
           ASyncSubscriberState::DATA_READY;
  }

  /**
   * @brief 获取当前数据 / Retrieve the current data
   * @return 当前数据引用 / Returns a reference to the current data
   *
   * @note 若当前缓冲区处于 `DATA_READY`，本次读取会把状态清回 `IDLE`；之后新的发布需要
   *       重新调用 `StartWaiting()` 才会继续接收 /
   *       If the local buffer is currently `DATA_READY`, this read clears the
   *       state back to `IDLE`; later publishes are ignored again until
   *       `StartWaiting()` is called
   */
  Data& GetData()
  {
    if (block_->data_.state.load(std::memory_order_acquire) ==
        ASyncSubscriberState::DATA_READY)
    {
      block_->data_.state.store(ASyncSubscriberState::IDLE, std::memory_order_release);
    }
    return *reinterpret_cast<Data*>(block_->data_.buff_addr);
  }

  /**
   * @brief 获取最近一次接收的消息时间戳 / Get the latest received message timestamp
   * @return 最近一次接收的消息时间戳 / Returns the latest received message timestamp
   */
  MicrosecondTimestamp GetTimestamp() const { return block_->data_.timestamp; }

  /**
   * @brief 开始等待数据更新 / Start waiting for a data update
   * @note 异步订阅者只接收 `WAITING` 状态下的下一次发布；`IDLE` 或 `DATA_READY`
   * 状态下的新发布会被忽略 / Async subscribers only capture the next publish while in
   * `WAITING` state; new publishes in `IDLE` or `DATA_READY` state are ignored
   */
  void StartWaiting()
  {
    if (block_->data_.state.load(std::memory_order_acquire) == ASyncSubscriberState::IDLE)
    {
      block_->data_.state.store(ASyncSubscriberState::WAITING, std::memory_order_release);
    }
  }

  LockFreeList::Node<ASyncBlock>* block_ =
      nullptr;  ///< 订阅者数据块 / Subscriber data block.
};
}  // namespace LibXR
