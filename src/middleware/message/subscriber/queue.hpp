#pragma once

#include "../topic.hpp"

namespace LibXR
{
/**
 * @struct Topic::QueueBlock
 * @brief 队列订阅者自己挂的数据块 / Data block owned by one queued subscriber
 */
struct Topic::QueueBlock : public Topic::SuberBlock
{
  SPSCQueueBase* queue;  ///< 指向订阅队列基类 / Pointer to the subscribed queue base.
  void (*fun)(MicrosecondTimestamp, void*,
              QueueBlock&);  ///< 把一条发布转发进队列 / Adapter that forwards one publish
                             ///< into the queue.
};

/**
 * @class Topic::QueuedSubscriber
 * @brief 每次发布都往队列里塞一份数据的订阅者 / Subscriber that pushes one entry into
 *        a queue on each publish
 */
class Topic::QueuedSubscriber
{
 public:
  /**
   * @brief 通过主题名称构造队列订阅者 / Construct a queue subscriber by topic name
   * @tparam Data 队列存储的数据类型 / Data type stored in the queue
   * @param name 订阅的主题名称 / Name of the subscribed topic
   * @param queue 订阅的数据队列 / Subscribed data queue
   * @param domain 可选的域指针，默认为 `nullptr` / Optional domain pointer, default
   * `nullptr`
   * @note 包含初始化期动态内存分配，订阅者应长期存在 / Contains initialization-time
   * dynamic allocation; subscribers are expected to be long-lived
   * @note 队列订阅者只保存 `queue` 的指针；队列对象本身必须至少活到订阅者不再使用
   *       为止 /
   *       Queued subscribers keep only a pointer to `queue`; the queue object
   *       itself must outlive the subscriber's use of it
   * @note 每次发布都会直接调一次底层 `SPSCQueueBase::PushBytes()`；如果 push 不进去，
   *       这次发布就直接丢掉 /
   *       Each publish directly calls one underlying `SPSCQueueBase::PushBytes()`;
   *       if that push cannot fit, this publish is dropped immediately
   */
  template <typename Data>
  QueuedSubscriber(const char* name, SPSCQueue<Data>& queue, Domain* domain = nullptr)
      : QueuedSubscriber(Topic(WaitTopic(name, UINT32_MAX, domain)), queue)
  {
  }

  /**
   * @brief 通过主题名称构造带时间戳消息队列订阅者 / Construct a queue subscriber for
   * timestamped messages by topic name
   * @tparam Data 队列消息的数据类型 / Data type stored in the queue message
   * @param name 订阅的主题名称 / Name of the subscribed topic
   * @param queue 订阅的消息队列 / Subscribed message queue
   * @param domain 可选的域指针，默认为 `nullptr` / Optional domain pointer, default
   * `nullptr`
   * @note 队列订阅者只保存 `queue` 的指针；队列对象本身必须至少活到订阅者不再使用
   *       为止 /
   *       Queued subscribers keep only a pointer to `queue`; the queue object
   *       itself must outlive the subscriber's use of it
   * @note 每次发布都会直接调一次底层 `SPSCQueueBase::PushBytes()`；如果 push 不进去，
   *       这次发布就直接丢掉 /
   *       Each publish directly calls one underlying `SPSCQueueBase::PushBytes()`;
   *       if that push cannot fit, this publish is dropped immediately
   */
  template <typename Data>
  QueuedSubscriber(const char* name, SPSCQueue<Message<Data>>& queue,
                   Domain* domain = nullptr)
      : QueuedSubscriber(Topic(WaitTopic(name, UINT32_MAX, domain)), queue)
  {
  }

  /**
   * @brief 使用 `Topic` 和无锁队列构造订阅者 / Construct a subscriber from a `Topic` and
   * a lock-free queue
   * @tparam Data 队列存储的数据类型 / Data type stored in the queue
   * @param topic 订阅的主题 / Subscribed topic
   * @param queue 订阅的数据队列 / Subscribed data queue
   * @note 包含初始化期动态内存分配，订阅者应长期存在 / Contains initialization-time
   * dynamic allocation; subscribers are expected to be long-lived
   * @note 队列订阅者只保存 `queue` 的指针；队列对象本身必须至少活到订阅者不再使用
   *       为止 /
   *       Queued subscribers keep only a pointer to `queue`; the queue object
   *       itself must outlive the subscriber's use of it
   * @note 每次发布都会直接调一次底层 `SPSCQueueBase::PushBytes()`；如果 push 不进去，
   *       这次发布就直接丢掉 /
   *       Each publish directly calls one underlying `SPSCQueueBase::PushBytes()`;
   *       if that push cannot fit, this publish is dropped immediately
   */
  template <typename Data>
  QueuedSubscriber(Topic topic, SPSCQueue<Data>& queue)
  {
    Topic::CheckSubscriberType<Data>(topic);

    block_ = new LockFreeList::Node<QueueBlock>;
    block_->data_.type = SuberType::QUEUE;
    block_->data_.queue = &queue;
    block_->data_.fun = [](MicrosecondTimestamp, void* payload_addr, QueueBlock& block)
    { (void)block.queue->PushBytes(payload_addr); };

    topic.block_->data_.subers.Add(*block_);
  }

  /**
   * @brief 使用 `Topic` 和带时间戳消息队列构造订阅者 / Construct a subscriber from a
   * `Topic` and a timestamped message queue
   * @tparam Data 队列消息的数据类型 / Data type stored in the queue message
   * @param topic 订阅的主题 / Subscribed topic
   * @param queue 订阅的消息队列 / Subscribed message queue
   * @note 队列订阅者只保存 `queue` 的指针；队列对象本身必须至少活到订阅者不再使用
   *       为止 /
   *       Queued subscribers keep only a pointer to `queue`; the queue object
   *       itself must outlive the subscriber's use of it
   * @note 每次发布都会直接调一次底层 `SPSCQueueBase::PushBytes()`；如果 push 不进去，
   *       这次发布就直接丢掉 /
   *       Each publish directly calls one underlying `SPSCQueueBase::PushBytes()`;
   *       if that push cannot fit, this publish is dropped immediately
   */
  template <typename Data>
  QueuedSubscriber(Topic topic, SPSCQueue<Message<Data>>& queue)
  {
    Topic::CheckSubscriberType<Data>(topic);

    block_ = new LockFreeList::Node<QueueBlock>;
    block_->data_.type = SuberType::QUEUE;
    block_->data_.queue = &queue;
    block_->data_.fun =
        [](MicrosecondTimestamp timestamp, void* payload_addr, QueueBlock& block)
    {
      Message<Data> message{timestamp, *reinterpret_cast<Data*>(payload_addr)};
      (void)block.queue->PushBytes(&message);
    };

    topic.block_->data_.subers.Add(*block_);
  }

  /**
   * @brief 禁止拷贝队列订阅者 / Copy construction is disabled for queued subscribers
   * @param other 待拷贝的队列订阅者 / Queued subscriber to copy from
   */
  QueuedSubscriber(const QueuedSubscriber& other) = delete;

  /**
   * @brief 禁止拷贝赋值队列订阅者 / Copy assignment is disabled for queued subscribers
   * @param other 待拷贝的队列订阅者 / Queued subscriber to copy from
   * @return 当前队列订阅者 / Returns the current queued subscriber
   */
  QueuedSubscriber& operator=(const QueuedSubscriber& other) = delete;

  /**
   * @brief 移动构造队列订阅者 / Move-construct one queued subscriber
   * @param other 被转移的队列订阅者 / Queued subscriber to move from
   * @note 这里只移动本地句柄指针；底层订阅块仍留在 topic 的订阅链表里，`other`
   *       会被清成空句柄 /
   *       This moves only the local handle pointer; the underlying subscriber
   *       block stays registered in the topic list and `other` becomes empty
   */
  QueuedSubscriber(QueuedSubscriber&& other) noexcept : block_(other.block_)
  {
    other.block_ = nullptr;
  }

  /**
   * @brief 移动赋值队列订阅者 / Move-assign one queued subscriber
   * @param other 被转移的队列订阅者 / Queued subscriber to move from
   * @return 当前队列订阅者 / Returns the current queued subscriber
   * @note 这里只改当前包装对象指向的订阅块，不会注销旧块；底层订阅块仍留在 topic
   *       的订阅链表里，`other` 会被清成空句柄 /
   *       This only changes which subscriber block the current wrapper points
   *       to and does not unregister the old block; the underlying subscriber
   *       blocks stay in the topic list and `other` becomes empty
   */
  QueuedSubscriber& operator=(QueuedSubscriber&& other) noexcept
  {
    if (this != &other)
    {
      block_ = other.block_;
      other.block_ = nullptr;
    }
    return *this;
  }

 private:
  LockFreeList::Node<QueueBlock>* block_ =
      nullptr;  ///< 订阅者数据块 / Subscriber data block.
};
}  // namespace LibXR
