#pragma once

#include "../topic.hpp"

namespace LibXR
{
/**
 * @struct Topic::SyncBlock
 * @brief 同步订阅者自己挂的数据块 / Data block owned by one synchronous subscriber
 */
struct Topic::SyncBlock : public SuberBlock
{
  /**
   * @enum WaitState
   * @brief 同步等待状态 / Synchronous wait state
   *
   * `WAIT_CLAIMED` 表示这次唤醒已经归超时前那一次 `Wait()` 所有，直到它把 semaphore post
   * 消费掉为止；否则下一次 `Wait()` 可能会错误抢走旧唤醒。
   * `WAIT_CLAIMED` means the wakeup already belongs to the `Wait()` that timed out
   * just before the post and must stay reserved until that waiter consumes the
   * semaphore post; otherwise a later `Wait()` could steal the old wakeup.
   */
  enum WaitState : uint32_t
  {
    WAIT_IDLE = 0,    ///< 当前没有挂起等待 / No wait is currently pending.
    WAITING = 1,      ///< 当前有一个挂起的等待者 / One waiter is currently pending.
    WAIT_CLAIMED = 2  ///< 某次发布已归一个刚超时的等待者所有 / One publish wakeup is
                      ///< reserved for a waiter that just timed out.
  };

  void* buff_addr;  ///< 收到消息后要拷到这里 / Received payloads are copied here.
  void (*copy_payload)(
      void* dst,
      void* payload_addr);  ///< 按订阅精确类型执行负载拷贝的适配函数 / Adapter that
                            ///< copies one payload using the subscriber's exact type.
  MicrosecondTimestamp
      timestamp;  ///< 这里对应那份数据的时间戳 / Timestamp paired with the buffered data.
  std::atomic<uint32_t> wait_state =
      WAIT_IDLE;  ///< 当前 `Wait()` 的挂起状态 / Current pending state of `Wait()`.
  Semaphore sem;  ///< 用来唤醒 `Wait()` 的信号量 / Semaphore used to wake `Wait()`.
};

/**
 * @class Topic::SyncSubscriber
 * @brief 调用 `Wait()` 收消息的订阅者 / Subscriber that receives messages by calling
 *        `Wait()`
 * @tparam Data 订阅的数据类型 / Type of data being subscribed to
 */
template <typename Data>
class Topic::SyncSubscriber
{
 public:
  /**
   * @brief 通过主题名称构造同步订阅者 / Construct a synchronous subscriber by topic name
   * @param name 主题名称 / Topic name
   * @param data 用来接收消息的对象 / Destination object receiving subscribed data
   * @param domain 可选的主题域 / Optional topic domain
   * @note 包含初始化期动态内存分配，订阅者应长期存在 / Contains initialization-time
   * dynamic allocation; subscribers are expected to be long-lived
   * @note 同步订阅者不会自建接收缓冲区，而是直接把收到的数据写进 `data`；`data`
   *       必须至少活到订阅者不再使用为止 /
   *       Synchronous subscribers do not allocate their own receive buffer;
   *       they write incoming data directly into `data`, which must outlive the
   *       subscriber's use of it
   */
  SyncSubscriber(const char* name, Data& data, Domain* domain = nullptr)
      : SyncSubscriber(Topic(WaitTopic(name, UINT32_MAX, domain)), data)
  {
  }

  /**
   * @brief 通过 `Topic` 句柄构造同步订阅者 / Construct a synchronous subscriber using a
   * `Topic` handle
   * @param topic 订阅的主题 / Topic being subscribed to
   * @param data 用来接收消息的对象 / Destination object receiving subscribed data
   * @note 包含初始化期动态内存分配，订阅者应长期存在 / Contains initialization-time
   * dynamic allocation; subscribers are expected to be long-lived
   * @note 同步订阅者不会自建接收缓冲区，而是直接把收到的数据写进 `data`；`data`
   *       必须至少活到订阅者不再使用为止 /
   *       Synchronous subscribers do not allocate their own receive buffer;
   *       they write incoming data directly into `data`, which must outlive the
   *       subscriber's use of it
   */
  SyncSubscriber(Topic topic, Data& data)
  {
    Topic::CheckSubscriberType<Data>(topic);

    block_ = new LockFreeList::Node<SyncBlock>;
    block_->data_.type = SuberType::SYNC;
    block_->data_.timestamp = MicrosecondTimestamp();
    block_->data_.wait_state.store(SyncBlock::WAIT_IDLE, std::memory_order_relaxed);
    block_->data_.buff_addr = &data;
    block_->data_.copy_payload = &Topic::CopyPayload<Data>;
    topic.block_->data_.subers.Add(*block_);
  }

  /**
   * @brief 禁止拷贝同步订阅者 / Copy construction is disabled for synchronous
   *        subscribers
   * @param other 待拷贝的同步订阅者 / Synchronous subscriber to copy from
   */
  SyncSubscriber(const SyncSubscriber& other) = delete;

  /**
   * @brief 禁止拷贝赋值同步订阅者 / Copy assignment is disabled for synchronous
   *        subscribers
   * @param other 待拷贝的同步订阅者 / Synchronous subscriber to copy from
   * @return 当前同步订阅者 / Returns the current synchronous subscriber
   */
  SyncSubscriber& operator=(const SyncSubscriber& other) = delete;

  /**
   * @brief 移动构造同步订阅者 / Move-construct one synchronous subscriber
   * @param other 被转移的同步订阅者 / Synchronous subscriber to move from
   * @note 这里只移动本地句柄指针；底层订阅块仍留在 topic 的订阅链表里，`other`
   *       会被清成空句柄 /
   *       This moves only the local handle pointer; the underlying subscriber
   *       block stays registered in the topic list and `other` becomes empty
   */
  SyncSubscriber(SyncSubscriber&& other) noexcept : block_(other.block_)
  {
    other.block_ = nullptr;
  }

  /**
   * @brief 移动赋值同步订阅者 / Move-assign one synchronous subscriber
   * @param other 被转移的同步订阅者 / Synchronous subscriber to move from
   * @return 当前同步订阅者 / Returns the current synchronous subscriber
   * @note 这里只改当前包装对象指向的订阅块，不会注销旧块；底层订阅块仍留在 topic
   *       的订阅链表里，`other` 会被清成空句柄 /
   *       This only changes which subscriber block the current wrapper points
   *       to and does not unregister the old block; the underlying subscriber
   *       blocks stay in the topic list and `other` becomes empty
   */
  SyncSubscriber& operator=(SyncSubscriber&& other) noexcept
  {
    if (this != &other)
    {
      block_ = other.block_;
      other.block_ = nullptr;
    }
    return *this;
  }

  /**
   * @brief 等待接收数据 / Wait for data reception
   * @param timeout 超时时间，默认为 `UINT32_MAX` / Timeout period, default `UINT32_MAX`
   * @return 操作结果错误码 / Error code indicating the operation result
   *
   * @note 同一时刻只允许一个等待者；若已有挂起等待，会返回 `ErrorCode::BUSY` /
   *       Only one waiter is allowed at a time; if another wait is already
   *       pending, this returns `ErrorCode::BUSY`
   * @note 若发布恰好发生在超时边界，当前等待者仍会保留那次唤醒并最终返回 `OK`，不会把
   *       这次唤醒漏给下一次 `Wait()` /
   *       If a publish arrives right at the timeout boundary, the current waiter
   *       still keeps that wakeup and eventually returns `OK` instead of leaking
   *       the wakeup to a later `Wait()`
   */
  ErrorCode Wait(uint32_t timeout = UINT32_MAX)
  {
    ASSERT(block_ != nullptr);

    auto& data = block_->data_;
    uint32_t expected = SyncBlock::WAIT_IDLE;
    if (!data.wait_state.compare_exchange_strong(expected, SyncBlock::WAITING,
                                                 std::memory_order_acq_rel,
                                                 std::memory_order_acquire))
    {
      return ErrorCode::BUSY;
    }

    auto wait_ans = data.sem.Wait(timeout);
    if (wait_ans == ErrorCode::OK)
    {
      data.wait_state.store(SyncBlock::WAIT_IDLE, std::memory_order_release);
      return ErrorCode::OK;
    }

    expected = SyncBlock::WAITING;
    if (data.wait_state.compare_exchange_strong(expected, SyncBlock::WAIT_IDLE,
                                                std::memory_order_acq_rel,
                                                std::memory_order_acquire))
    {
      return wait_ans;
    }

    DEV_ASSERT(data.wait_state.load(std::memory_order_acquire) ==
               SyncBlock::WAIT_CLAIMED);

    ErrorCode finish_wait_ans;
    do
    {
      finish_wait_ans = data.sem.Wait(UINT32_MAX);
    } while (finish_wait_ans == ErrorCode::TIMEOUT);
    REQUIRE(finish_wait_ans == ErrorCode::OK);
    data.wait_state.store(SyncBlock::WAIT_IDLE, std::memory_order_release);
    return ErrorCode::OK;
  }

  /**
   * @brief 获取最近一次接收的消息时间戳 / Get the latest received message timestamp
   * @return 最近一次接收的消息时间戳 / Returns the latest received message timestamp
   */
  MicrosecondTimestamp GetTimestamp() const { return block_->data_.timestamp; }

  LockFreeList::Node<SyncBlock>* block_ =
      nullptr;  ///< 订阅者数据块 / Subscriber data block.
};
}  // namespace LibXR
