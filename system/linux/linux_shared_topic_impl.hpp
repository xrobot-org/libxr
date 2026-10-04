#pragma once

#if defined(LIBXR_SYSTEM_POSIX_HOST)

#include <fcntl.h>
#include <linux/futex.h>
#include <signal.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <time.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cinttypes>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <type_traits>

#include "crc.hpp"
#include "libxr_def.hpp"
#include "message.hpp"

namespace LibXR
{
/**
 * @enum LinuxSharedSubscriberMode
 * @brief 订阅者接收模式 / Receive mode of a Linux shared subscriber.
 */
enum class LinuxSharedSubscriberMode : uint8_t
{
  BROADCAST_FULL = 0,      ///< 广播到该订阅者，满队列时报错 / Broadcast and fail on full.
  BROADCAST_DROP_OLD = 1,  ///< 广播到该订阅者，满队列时丢最旧 / Broadcast and drop oldest
                           ///< on full.
  BALANCE_RR = 2,          ///< 参与 RR 负载均衡组 / Participate in the RR balanced group.
};

/**
 * @struct LinuxSharedTopicConfig
 * @brief Linux 共享 Topic 的创建配置 / Creation config for Linux shared topics.
 */
struct LinuxSharedTopicConfig
{
  uint32_t slot_num = 64;       ///< 共享 payload 槽位数 / Number of shared payload slots.
  uint32_t subscriber_num = 8;  ///< 最大订阅者数量 / Maximum number of subscribers.
  uint32_t queue_num =
      64;  ///< 每订阅者描述符队列长度 / Descriptor queue length per subscriber.
};

/**
 * @class LinuxSharedTopic
 * @brief Linux 主机进程间共享 Topic / Linux-host shared-memory topic.
 *
 * 该类提供 Linux 主机上的固定大小进程间消息通道，使用共享内存保存 payload，
 * 使用原子变量和 futex 完成发布/接收同步。在 `Webots` 系统配置下仍复用这套实现，
 * 但超时语义遵循 `Webots` 的系统时间。
 * This class provides a fixed-size host-side IPC topic. Payloads live in shared memory,
 * while publish/receive synchronization is handled with atomics and futex. Under the
 * `Webots` system configuration it still reuses this implementation, but timeout behavior
 * follows the `Webots` system time model.
 *
 * @tparam TopicData 话题数据类型，必须为平凡可拷贝类型。
 * Topic data type, must be trivially copyable.
 */
template <typename TopicData>
class LinuxSharedTopic : public Topic
{
  static_assert(std::is_trivially_copyable<TopicData>::value,
                "LinuxSharedTopic requires trivially copyable data");
  static_assert(std::atomic<uint32_t>::is_always_lock_free,
                "LinuxSharedTopic requires lock-free 32-bit atomics");
  static_assert(std::atomic<uint64_t>::is_always_lock_free,
                "LinuxSharedTopic requires lock-free 64-bit atomics");

  enum class SharedDataState : uint8_t
  {
    EMPTY = 0,
    PUBLISHER = 1,
    SUBSCRIBER = 2,
  };

 public:
  class SharedData;
  using Data = SharedData;
  static constexpr const char* DEFAULT_DOMAIN_NAME = "libxr_def_domain";

  /**
   * @class Subscriber
   * @brief 同步订阅者，用于从共享 Topic 中等待并读取消息。
   *        Synchronous subscriber for waiting on and reading messages from a shared
   * topic.
   */
  class Subscriber
  {
   public:
    using Data = SharedData;

    /**
     * @brief 默认构造函数，创建空订阅者。
     * Default constructor creating an empty subscriber.
     */
    Subscriber() = default;

    /**
     * @brief 通过主题名附着并创建订阅者 / Attach and create a subscriber by topic name.
     * @param name 主题名称 / Topic name.
     * @param mode 订阅模式 / Subscriber mode.
     *
     * @note 包含动态内存分配。
     *       Contains dynamic memory allocation.
     */
    explicit Subscriber(const char* name, LinuxSharedSubscriberMode mode =
                                              LinuxSharedSubscriberMode::BROADCAST_FULL)
        : owned_topic_(new LinuxSharedTopic(name))
    {
      if (Attach(*owned_topic_, mode) != ErrorCode::OK)
      {
        delete owned_topic_;
        owned_topic_ = nullptr;
      }
    }

    Subscriber(const char* name, const char* domain_name,
               LinuxSharedSubscriberMode mode = LinuxSharedSubscriberMode::BROADCAST_FULL)
        : owned_topic_(new LinuxSharedTopic(name, domain_name))
    {
      if (Attach(*owned_topic_, mode) != ErrorCode::OK)
      {
        delete owned_topic_;
        owned_topic_ = nullptr;
      }
    }

    Subscriber(const char* name, Topic::Domain& domain,
               LinuxSharedSubscriberMode mode = LinuxSharedSubscriberMode::BROADCAST_FULL)
        : owned_topic_(new LinuxSharedTopic(name, domain))
    {
      if (Attach(*owned_topic_, mode) != ErrorCode::OK)
      {
        delete owned_topic_;
        owned_topic_ = nullptr;
      }
    }

    /**
     * @brief 基于现有共享 Topic 创建订阅者。
     * Create a subscriber from an existing shared topic.
     * @param topic 已打开的共享 Topic / Opened shared topic.
     * @param mode 订阅模式 / Subscriber mode.
     */
    explicit Subscriber(
        LinuxSharedTopic& topic,
        LinuxSharedSubscriberMode mode = LinuxSharedSubscriberMode::BROADCAST_FULL)
    {
      (void)Attach(topic, mode);
    }

    /**
     * @brief 析构函数，自动释放订阅者占用的状态 / Destructor releasing subscriber state.
     */
    ~Subscriber() { Reset(); }

    Subscriber(const Subscriber&) = delete;
    Subscriber& operator=(const Subscriber&) = delete;

    Subscriber(Subscriber&& other) noexcept { *this = std::move(other); }

    Subscriber& operator=(Subscriber&& other) noexcept
    {
      if (this == &other)
      {
        return *this;
      }

      Reset();

      topic_ = other.topic_;
      owned_topic_ = other.owned_topic_;
      subscriber_index_ = other.subscriber_index_;
      current_slot_index_ = other.current_slot_index_;
      current_sequence_ = other.current_sequence_;
      current_timestamp_ = other.current_timestamp_;

      other.topic_ = nullptr;
      other.owned_topic_ = nullptr;
      other.subscriber_index_ = INVALID_INDEX;
      other.current_slot_index_ = INVALID_INDEX;
      other.current_sequence_ = 0;
      other.current_timestamp_ = MicrosecondTimestamp();
      return *this;
    }

    /**
     * @brief 检查订阅者是否有效 / Checks whether the subscriber is valid.
     * @return true 订阅者有效 / Subscriber is valid.
     * @return false 订阅者无效 / Subscriber is invalid.
     */
    bool Valid() const { return topic_ != nullptr && subscriber_index_ != INVALID_INDEX; }

    /**
     * @brief 等待一条消息，并把当前订阅者切到该 payload。
     *        Wait for one message and bind the subscriber to that payload.
     * @param timeout_ms 超时时间，默认无限等待。
     * Timeout in milliseconds, default is wait forever.
     * @return 错误码 / Error code indicating the wait result.
     */
    ErrorCode Wait(uint32_t timeout_ms = UINT32_MAX)
    {
      if (!Valid())
      {
        return ErrorCode::STATE_ERR;
      }

      const uint64_t deadline_ms =
          (timeout_ms == UINT32_MAX) ? 0 : (NowMonotonicMs() + timeout_ms);

      Descriptor desc = {};
      while (true)
      {
        ErrorCode pop_ans = topic_->TryPopDescriptor(subscriber_index_, desc);
        if (pop_ans == ErrorCode::OK)
        {
          Release();
          topic_->HoldSlot(subscriber_index_, desc.slot_index);
          current_slot_index_ = desc.slot_index;
          current_sequence_ = desc.sequence;
          current_timestamp_ = topic_->SlotTimestamp(desc.slot_index);
          return ErrorCode::OK;
        }

        uint32_t wait_ms = UINT32_MAX;
        if (timeout_ms != UINT32_MAX)
        {
          const uint64_t now_ms = NowMonotonicMs();
          if (now_ms >= deadline_ms)
          {
            return ErrorCode::TIMEOUT;
          }
          wait_ms = static_cast<uint32_t>(deadline_ms - now_ms);
        }

        const ErrorCode wait_ans =
            topic_->WaitReady(topic_->subscribers_[subscriber_index_], wait_ms);
        if (wait_ans == ErrorCode::OK)
        {
          continue;
        }
        return wait_ans;
      }
    }

    /**
     * @brief 等待一条消息，并通过句柄返回该 payload。
     *        Wait for one message and return the payload through a handle.
     * @param data 输出句柄 / Output data handle.
     * @param timeout_ms 超时时间，默认无限等待。
     * Timeout in milliseconds, default is wait forever.
     * @return 错误码 / Error code indicating the wait result.
     */
    ErrorCode Wait(SharedData& data, uint32_t timeout_ms = UINT32_MAX)
    {
      data.Reset();

      if (!Valid())
      {
        return ErrorCode::STATE_ERR;
      }

      const uint64_t deadline_ms =
          (timeout_ms == UINT32_MAX) ? 0 : (NowMonotonicMs() + timeout_ms);

      Descriptor desc = {};
      while (true)
      {
        ErrorCode pop_ans = topic_->TryPopDescriptor(subscriber_index_, desc);
        if (pop_ans == ErrorCode::OK)
        {
          data.topic_ = topic_;
          data.slot_index_ = desc.slot_index;
          data.sequence_ = desc.sequence;
          data.state_ = SharedDataState::SUBSCRIBER;
          data.subscriber_index_ = subscriber_index_;
          topic_->HoldSlot(subscriber_index_, desc.slot_index);
          return ErrorCode::OK;
        }

        uint32_t wait_ms = UINT32_MAX;
        if (timeout_ms != UINT32_MAX)
        {
          const uint64_t now_ms = NowMonotonicMs();
          if (now_ms >= deadline_ms)
          {
            return ErrorCode::TIMEOUT;
          }
          wait_ms = static_cast<uint32_t>(deadline_ms - now_ms);
        }

        const ErrorCode wait_ans =
            topic_->WaitReady(topic_->subscribers_[subscriber_index_], wait_ms);
        if (wait_ans == ErrorCode::OK)
        {
          continue;
        }
        return wait_ans;
      }
    }

    /**
     * @brief 获取当前持有的消息数据指针 / Get the pointer to the currently held payload.
     * @return 当前消息指针；若未持有消息则返回 nullptr。
     *         Pointer to current payload, or nullptr if none is held.
     */
    TopicData* GetData() const
    {
      if (!Valid() || current_slot_index_ == INVALID_INDEX)
      {
        return nullptr;
      }

      return &topic_->payloads_[current_slot_index_];
    }

    /**
     * @brief 获取当前消息序号 / Get the sequence number of the current message.
     */
    uint64_t GetSequence() const { return current_sequence_; }

    /**
     * @brief 获取当前消息时间戳 / Get the timestamp of the current message.
     */
    MicrosecondTimestamp GetTimestamp() const { return current_timestamp_; }

    /**
     * @brief 获取当前待消费描述符数量 / Get the number of queued pending descriptors.
     */
    uint32_t GetPendingNum() const
    {
      if (!Valid())
      {
        return 0;
      }

      const SubscriberControl& control = topic_->subscribers_[subscriber_index_];
      const uint32_t head = control.queue_head.load(std::memory_order_acquire);
      const uint32_t tail = control.queue_tail.load(std::memory_order_acquire);
      if (tail >= head)
      {
        return tail - head;
      }
      return topic_->queue_capacity_ - (head - tail);
    }

    /**
     * @brief 获取该订阅者累计丢弃消息数。
     * Get the accumulated drop count of this subscriber.
     */
    uint64_t GetDropNum() const
    {
      if (!Valid())
      {
        return 0;
      }

      return topic_->subscribers_[subscriber_index_].dropped_messages.load(
          std::memory_order_acquire);
    }

    /**
     * @brief 释放当前持有的消息槽位 / Release the currently held payload slot.
     */
    void Release()
    {
      if (!Valid() || current_slot_index_ == INVALID_INDEX)
      {
        return;
      }

      topic_->ClearHeldSlot(subscriber_index_, current_slot_index_);
      topic_->ReleaseSlot(current_slot_index_);
      current_slot_index_ = INVALID_INDEX;
      current_sequence_ = 0;
      current_timestamp_ = MicrosecondTimestamp();
    }

    /**
     * @brief 注销订阅者并释放所有持有资源。
     * Unregister the subscriber and release owned resources.
     */
    void Reset()
    {
      if (!Valid())
      {
        return;
      }

      topic_->UnregisterBalancedSubscriber(subscriber_index_);
      topic_->subscribers_[subscriber_index_].active.store(0, std::memory_order_release);
      topic_->subscribers_[subscriber_index_].owner_pid.store(0,
                                                              std::memory_order_release);
      topic_->subscribers_[subscriber_index_].owner_starttime.store(
          0, std::memory_order_release);

      Descriptor desc = {};
      while (topic_->TryPopDescriptor(subscriber_index_, desc) == ErrorCode::OK)
      {
        topic_->ReleaseSlot(desc.slot_index);
      }

      Release();

      topic_ = nullptr;
      delete owned_topic_;
      owned_topic_ = nullptr;
      subscriber_index_ = INVALID_INDEX;
      current_slot_index_ = INVALID_INDEX;
      current_sequence_ = 0;
      current_timestamp_ = MicrosecondTimestamp();
    }

   private:
    ErrorCode Attach(LinuxSharedTopic& topic, LinuxSharedSubscriberMode mode)
    {
      Reset();

      if (!topic.Valid())
      {
        return ErrorCode::STATE_ERR;
      }

      if (topic.self_identity_.starttime == 0)
      {
        return ErrorCode::STATE_ERR;
      }

      for (uint32_t i = 0; i < topic.subscriber_capacity_; ++i)
      {
        uint32_t expected = 0;
        auto& active = topic.subscribers_[i].active;
        if (active.compare_exchange_strong(expected, 1, std::memory_order_acq_rel,
                                           std::memory_order_relaxed))
        {
          topic.subscribers_[i].queue_head.store(0, std::memory_order_release);
          topic.subscribers_[i].queue_tail.store(0, std::memory_order_release);
          topic.subscribers_[i].ready_signal.store(0, std::memory_order_release);
          topic.subscribers_[i].dropped_messages.store(0, std::memory_order_release);
          topic.subscribers_[i].owner_pid.store(topic.self_identity_.pid,
                                                std::memory_order_release);
          topic.subscribers_[i].owner_starttime.store(topic.self_identity_.starttime,
                                                      std::memory_order_release);
          topic.subscribers_[i].held_slot.store(INVALID_INDEX, std::memory_order_release);
          topic.subscribers_[i].mode.store(static_cast<uint32_t>(mode),
                                           std::memory_order_release);
          if (mode == LinuxSharedSubscriberMode::BALANCE_RR)
          {
            const ErrorCode join_ans = topic.RegisterBalancedSubscriber(i);
            if (join_ans != ErrorCode::OK)
            {
              topic.subscribers_[i].active.store(0, std::memory_order_release);
              topic.subscribers_[i].owner_pid.store(0, std::memory_order_release);
              topic.subscribers_[i].owner_starttime.store(0, std::memory_order_release);
              topic.subscribers_[i].mode.store(
                  static_cast<uint32_t>(LinuxSharedSubscriberMode::BROADCAST_FULL),
                  std::memory_order_release);
              return join_ans;
            }
          }
          topic_ = &topic;
          subscriber_index_ = i;
          current_slot_index_ = INVALID_INDEX;
          current_sequence_ = 0;
          current_timestamp_ = MicrosecondTimestamp();
          return ErrorCode::OK;
        }
      }

      return ErrorCode::FULL;
    }

    LinuxSharedTopic* topic_ = nullptr;
    LinuxSharedTopic* owned_topic_ = nullptr;
    uint32_t subscriber_index_ = INVALID_INDEX;
    uint32_t current_slot_index_ = INVALID_INDEX;
    uint64_t current_sequence_ = 0;
    MicrosecondTimestamp current_timestamp_;
  };

  /**
   * @class SharedData
   * @brief 共享 Topic payload 句柄 / Payload handle for Linux shared topics.
   *
   * 该句柄可由发布者通过 `CreateData()` 获得，也可由订阅者通过 `Wait(data)` 获得。
   * 句柄析构或 `Reset()` 时会自动回收对应槽位。
   * This handle can be acquired by publishers via `CreateData()` or by subscribers via
   * `Wait(data)`. The slot is released automatically on destruction or `Reset()`.
   */
  class SharedData
  {
   public:
    /**
     * @brief 默认构造函数，创建空句柄 / Default constructor creating an empty handle.
     */
    SharedData() = default;

    /**
     * @brief 析构函数，自动回收句柄持有的槽位。
     * Destructor automatically releasing the held slot.
     */
    ~SharedData() { Reset(); }

    SharedData(const SharedData&) = delete;
    SharedData& operator=(const SharedData&) = delete;

    SharedData(SharedData&& other) noexcept { *this = std::move(other); }

    /**
     * @brief 移动赋值，转移槽位所有权 / Move assignment transferring slot ownership.
     */
    SharedData& operator=(SharedData&& other) noexcept
    {
      if (this == &other)
      {
        return *this;
      }

      Reset();

      topic_ = other.topic_;
      slot_index_ = other.slot_index_;
      sequence_ = other.sequence_;
      state_ = other.state_;
      subscriber_index_ = other.subscriber_index_;

      other.topic_ = nullptr;
      other.slot_index_ = INVALID_INDEX;
      other.sequence_ = 0;
      other.state_ = SharedDataState::EMPTY;
      other.subscriber_index_ = INVALID_INDEX;
      return *this;
    }

    /**
     * @brief 检查句柄是否有效 / Checks whether the handle is valid.
     */
    bool Valid() const { return topic_ != nullptr && slot_index_ != INVALID_INDEX; }

    /**
     * @brief 检查句柄是否为空 / Checks whether the handle is empty.
     */
    bool Empty() const { return !Valid(); }

    /**
     * @brief 获取消息序号 / Get the sequence number of the held message.
     */
    uint64_t GetSequence() const { return sequence_; }

    /**
     * @brief 获取消息时间戳 / Get the held message timestamp.
     */
    MicrosecondTimestamp GetTimestamp() const
    {
      if (!Valid() || state_ != SharedDataState::SUBSCRIBER)
      {
        return MicrosecondTimestamp();
      }
      return topic_->SlotTimestamp(slot_index_);
    }

    /**
     * @brief 获取可写数据指针 / Get a writable pointer to the payload.
     * @return 数据指针；若句柄无效则返回 nullptr。
     *         Payload pointer, or nullptr if the handle is invalid.
     */
    TopicData* GetData()
    {
      if (!Valid())
      {
        return nullptr;
      }
      return &topic_->payloads_[slot_index_];
    }

    /**
     * @brief 获取数据指针 / Get a pointer to the payload.
     * @return 数据指针；若句柄无效则返回 nullptr。
     *         Payload pointer, or nullptr if the handle is invalid.
     */
    TopicData* GetData() const
    {
      if (!Valid())
      {
        return nullptr;
      }
      return &topic_->payloads_[slot_index_];
    }

    /**
     * @brief 释放句柄持有的槽位 / Release the slot held by this handle.
     */
    void Reset()
    {
      if (!Valid())
      {
        return;
      }

      if (state_ == SharedDataState::PUBLISHER)
      {
        topic_->RecycleSlot(slot_index_);
      }
      else if (state_ == SharedDataState::SUBSCRIBER)
      {
        topic_->ClearHeldSlot(subscriber_index_, slot_index_);
        topic_->ReleaseSlot(slot_index_);
      }
      topic_ = nullptr;
      slot_index_ = INVALID_INDEX;
      sequence_ = 0;
      state_ = SharedDataState::EMPTY;
      subscriber_index_ = INVALID_INDEX;
    }

   private:
    friend class LinuxSharedTopic<TopicData>;
    friend class Subscriber;

    LinuxSharedTopic* topic_ = nullptr;
    uint32_t slot_index_ = INVALID_INDEX;
    uint64_t sequence_ = 0;
    SharedDataState state_ = SharedDataState::EMPTY;
    uint32_t subscriber_index_ = INVALID_INDEX;
  };

  /**
   * @brief 以附着模式打开已有共享 Topic / Open an existing shared topic in attach mode.
   * @param topic_name 主题名称 / Topic name.
   */
  explicit LinuxSharedTopic(const char* topic_name)
      : LinuxSharedTopic(topic_name, DEFAULT_DOMAIN_NAME)
  {
  }

  LinuxSharedTopic(const char* topic_name, const char* domain_name)
      : create_(false),
        publisher_(false),
        config_(),
        domain_crc32_(ResolveDomainKey(domain_name)),
        topic_name_(ResolveTopicName(topic_name)),
        name_key_(BuildNameKey(domain_crc32_, topic_name_)),
        shm_name_(BuildShmName(name_key_))
  {
    (void)ReadProcessIdentity(static_cast<uint32_t>(getpid()), self_identity_);
    Open();
  }

  LinuxSharedTopic(const char* topic_name, Topic::Domain& domain)
      : create_(false),
        publisher_(false),
        config_(),
        domain_crc32_(domain.node_ != nullptr ? domain.node_->key : 0),
        topic_name_(ResolveTopicName(topic_name)),
        name_key_(BuildNameKey(domain_crc32_, topic_name_)),
        shm_name_(BuildShmName(name_key_))
  {
    (void)ReadProcessIdentity(static_cast<uint32_t>(getpid()), self_identity_);
    Open();
  }

  /**
   * @brief 以发布者模式创建或接管共享 Topic。
   * Create or take over a shared topic as publisher.
   * @param topic_name 主题名称 / Topic name.
   * @param config 创建配置 / Creation config.
   */
  LinuxSharedTopic(const char* topic_name, const LinuxSharedTopicConfig& config)
      : LinuxSharedTopic(topic_name, DEFAULT_DOMAIN_NAME, config)
  {
  }

  LinuxSharedTopic(const char* topic_name, const char* domain_name,
                   const LinuxSharedTopicConfig& config)
      : create_(true),
        publisher_(true),
        config_(config),
        domain_crc32_(ResolveDomainKey(domain_name)),
        topic_name_(ResolveTopicName(topic_name)),
        name_key_(BuildNameKey(domain_crc32_, topic_name_)),
        shm_name_(BuildShmName(name_key_))
  {
    (void)ReadProcessIdentity(static_cast<uint32_t>(getpid()), self_identity_);
    Open();
  }

  LinuxSharedTopic(const char* topic_name, Topic::Domain& domain,
                   const LinuxSharedTopicConfig& config)
      : create_(true),
        publisher_(true),
        config_(config),
        domain_crc32_(domain.node_ != nullptr ? domain.node_->key : 0),
        topic_name_(ResolveTopicName(topic_name)),
        name_key_(BuildNameKey(domain_crc32_, topic_name_)),
        shm_name_(BuildShmName(name_key_))
  {
    (void)ReadProcessIdentity(static_cast<uint32_t>(getpid()), self_identity_);
    Open();
  }

  /**
   * @typedef SyncSubscriber
   * @brief 同步订阅者别名 / Alias of the synchronous shared subscriber.
   */
  using SyncSubscriber = Subscriber;

  /**
   * @brief 析构函数，关闭共享 Topic / Destructor closing the shared topic.
   */
  ~LinuxSharedTopic() { Close(); }

  LinuxSharedTopic(const LinuxSharedTopic&) = delete;
  LinuxSharedTopic& operator=(const LinuxSharedTopic&) = delete;

  LinuxSharedTopic(LinuxSharedTopic&&) = delete;
  LinuxSharedTopic& operator=(LinuxSharedTopic&&) = delete;

  /**
   * @brief 检查共享 Topic 是否已成功打开。
   * Checks whether the shared topic is opened successfully.
   */
  bool Valid() const { return open_ok_; }

  /**
   * @brief 获取打开阶段的错误码 / Get the error code from the open stage.
   */
  ErrorCode GetError() const { return open_status_; }

  /**
   * @brief 获取当前活跃订阅者数量 / Get the current number of active subscribers.
   */
  uint32_t GetSubscriberNum() const
  {
    if (!Valid())
    {
      return 0;
    }

    uint32_t count = 0;
    for (uint32_t i = 0; i < subscriber_capacity_; ++i)
    {
      if (subscribers_[i].active.load(std::memory_order_acquire) != 0)
      {
        ++count;
      }
    }
    return count;
  }

  /**
   * @brief 为发布者申请一个可写 payload 槽位。
   * Acquire a writable payload slot for the publisher.
   * @param data 输出句柄 / Output payload handle.
   * @return 错误码 / Error code indicating the acquisition result.
   */
  ErrorCode CreateData(SharedData& data)
  {
    if (!Valid())
    {
      return ErrorCode::STATE_ERR;
    }

    if (!PublisherValid())
    {
      return ErrorCode::STATE_ERR;
    }

    data.Reset();

    uint32_t slot_index = INVALID_INDEX;
    ErrorCode pop_ans = PopFreeSlot(slot_index);
    if (pop_ans != ErrorCode::OK)
    {
      ScavengeDeadSubscribers();
      pop_ans = PopFreeSlot(slot_index);
      if (pop_ans != ErrorCode::OK)
      {
        return pop_ans;
      }
    }

    slots_[slot_index].refcount.store(0, std::memory_order_release);
    slots_[slot_index].sequence.store(0, std::memory_order_release);
    slots_[slot_index].timestamp_us = 0;

    data.topic_ = this;
    data.slot_index_ = slot_index;
    data.sequence_ = 0;
    data.state_ = SharedDataState::PUBLISHER;
    data.subscriber_index_ = INVALID_INDEX;
    return ErrorCode::OK;
  }

  /**
   * @brief 复制一份数据并发布 / Copy one payload and publish it.
   * @param data 待发布的数据 / Payload to publish.
   * @return 错误码 / Error code indicating the publish result.
   */
  ErrorCode Publish(const TopicData& data)
  {
    SharedData topic_data;
    const ErrorCode acquire_ans = CreateData(topic_data);
    if (acquire_ans != ErrorCode::OK)
    {
      return acquire_ans;
    }

    *topic_data.GetData() = data;
    return Publish(topic_data);
  }

  ErrorCode Publish(const TopicData& data, MicrosecondTimestamp timestamp)
  {
    SharedData topic_data;
    const ErrorCode acquire_ans = CreateData(topic_data);
    if (acquire_ans != ErrorCode::OK)
    {
      return acquire_ans;
    }

    *topic_data.GetData() = data;
    return Publish(topic_data, timestamp);
  }

  /**
   * @brief 发布一个已申请好的 payload 句柄 / Publish a pre-acquired payload handle.
   */
  ErrorCode Publish(SharedData&& data) { return PublishData<false>(data); }

  ErrorCode Publish(SharedData&& data, MicrosecondTimestamp timestamp)
  {
    return PublishData<true>(data, timestamp);
  }

  /**
   * @brief 发布一个已申请好的 payload 句柄 / Publish a pre-acquired payload handle.
   * @param data 由本 Topic 的 CreateData() 申请的句柄 / Handle acquired by CreateData()
   *        of this Topic.
   * @return OK：已发布（没有订阅者时直接丢弃）；FULL：订阅者队列已满或没有可选的
   *         BALANCE_RR 订阅者；STATE_ERR：句柄无效或不属于本 Topic。
   *         OK: published (dropped when there is no subscriber); FULL: a subscriber
   *         queue is full or no BALANCE_RR subscriber can be selected; STATE_ERR: the
   *         handle is invalid or belongs to another Topic.
   *
   * @note 除 STATE_ERR 外，每次返回时句柄都已交出或释放槽位，包括返回 FULL 时，此后
   *       `data.Valid()` 为 false。返回 FULL 后重试须重新调用 CreateData() 并重新填写
   *       数据；对同一句柄再次调用 Publish() 只会返回 STATE_ERR。
   *       Except for STATE_ERR, the handle has handed over or released its slot on
   *       every return, FULL included, and `data.Valid()` is false afterwards. A retry
   *       after FULL calls CreateData() again and refills the payload; calling Publish()
   *       again on the same handle only returns STATE_ERR.
   */
  ErrorCode Publish(SharedData& data) { return PublishData<false>(data); }

  ErrorCode Publish(SharedData& data, MicrosecondTimestamp timestamp)
  {
    return PublishData<true>(data, timestamp);
  }

  /**
   * @brief 获取累计发布失败次数 / Get the accumulated publish failure count.
   */
  uint64_t GetPublishFailedNum() const
  {
    if (!Valid())
    {
      return 0;
    }
    return header_->publish_failures.load(std::memory_order_acquire);
  }

  /**
   * @brief 删除对应的共享内存对象 / Remove the backing shared-memory object.
   * @param topic_name 主题名称 / Topic name.
   * @return 错误码 / Error code indicating the removal result.
   */
  static ErrorCode Remove(const char* topic_name)
  {
    return Remove(topic_name, DEFAULT_DOMAIN_NAME);
  }

  static ErrorCode Remove(const char* topic_name, const char* domain_name)
  {
    const std::string shm_name = BuildShmName(
        BuildNameKey(ResolveDomainKey(domain_name), ResolveTopicName(topic_name)));
    if (shm_unlink(shm_name.c_str()) == 0 || errno == ENOENT)
    {
      return ErrorCode::OK;
    }
    return ErrorCode::FAILED;
  }

  static ErrorCode Remove(const char* topic_name, Topic::Domain& domain)
  {
    const uint32_t domain_crc32 = (domain.node_ != nullptr) ? domain.node_->key : 0;
    const std::string shm_name =
        BuildShmName(BuildNameKey(domain_crc32, ResolveTopicName(topic_name)));
    if (shm_unlink(shm_name.c_str()) == 0 || errno == ENOENT)
    {
      return ErrorCode::OK;
    }
    return ErrorCode::FAILED;
  }

 private:
  struct alignas(LibXR::CONCURRENCY_ALIGNMENT) SharedHeader
  {
    uint64_t magic = 0;
    uint64_t name_key = 0;
    uint32_t domain_crc32 = 0;
    uint32_t version = 0;
    uint32_t data_size = 0;
    uint32_t slot_count = 0;
    uint32_t subscriber_capacity = 0;
    uint32_t queue_capacity = 0;
    uint32_t topic_name_len = 0;
    std::atomic<uint32_t> init_state;
    std::atomic<uint32_t> publisher_pid;
    std::atomic<uint64_t> publisher_starttime;
    std::atomic<uint64_t> free_queue_head;
    std::atomic<uint64_t> free_queue_tail;
    std::atomic<uint64_t> next_sequence;
    std::atomic<uint64_t> publish_failures;
  };

  struct alignas(LibXR::CONCURRENCY_ALIGNMENT) SlotControl
  {
    std::atomic<uint32_t> refcount;
    std::atomic<uint64_t> sequence;
    uint64_t timestamp_us;
  };

  struct alignas(16) FreeSlotCell
  {
    std::atomic<uint64_t> sequence;
    uint32_t slot_index = 0;
    uint32_t reserved = 0;
  };

  struct Descriptor
  {
    uint32_t slot_index = INVALID_INDEX;
    uint32_t reserved = 0;
    uint64_t sequence = 0;
  };

  struct alignas(LibXR::CONCURRENCY_ALIGNMENT) SubscriberControl
  {
    std::atomic<uint32_t> active;
    std::atomic<uint32_t> mode;
    std::atomic<uint32_t> queue_head;
    std::atomic<uint32_t> queue_tail;
    std::atomic<uint32_t>
        ready_signal;  ///< 唤醒提示，不表示队列长度 / Wake hint, not size.
    std::atomic<uint64_t> dropped_messages;
    std::atomic<uint32_t> owner_pid;
    std::atomic<uint64_t> owner_starttime;
    std::atomic<uint32_t> held_slot;
  };

  struct alignas(LibXR::CONCURRENCY_ALIGNMENT) BalancedGroupControl
  {
    std::atomic<uint64_t> rr_cursor;
  };

  struct ProcessIdentity
  {
    uint32_t pid = 0;
    uint64_t starttime = 0;
  };

  static constexpr uint64_t MAGIC = 0x4c58524950435348ULL;
  static constexpr uint32_t VERSION = 3;
  static constexpr uint32_t INIT_READY = 1;
  static constexpr uint32_t INVALID_INDEX = UINT32_MAX;

  static uint32_t ResolveDomainKey(const char* domain_name)
  {
    const std::string resolved = (domain_name == nullptr || domain_name[0] == '\0')
                                     ? std::string(DEFAULT_DOMAIN_NAME)
                                     : std::string(domain_name);
    return CRC32::Calculate(resolved.data(), resolved.size());
  }

  static std::string ResolveTopicName(const char* topic_name)
  {
    return (topic_name != nullptr) ? std::string(topic_name) : std::string();
  }

  static uint64_t BuildNameKey(uint32_t domain_crc32, const std::string& topic_name)
  {
    const uint32_t topic_len = static_cast<uint32_t>(topic_name.size());
    std::string key_material;
    key_material.reserve(sizeof(domain_crc32) + sizeof(topic_len) + topic_len);
    key_material.append(reinterpret_cast<const char*>(&domain_crc32),
                        sizeof(domain_crc32));
    key_material.append(reinterpret_cast<const char*>(&topic_len), sizeof(topic_len));
    key_material.append(topic_name.data(), topic_name.size());
    return CRC64::Calculate(key_material.data(), key_material.size());
  }

  static std::string BuildShmName(uint64_t name_key)
  {
    char buffer[64] = {};
    std::snprintf(buffer, sizeof(buffer), "/libxr_ipc_%016" PRIx64, name_key);
    return std::string(buffer);
  }

  static size_t AlignUp(size_t value, size_t alignment)
  {
    return (value + alignment - 1U) & ~(alignment - 1U);
  }

  static uint64_t NowMonotonicMs() { return MonotonicTime::NowMilliseconds(); }

  static MicrosecondTimestamp NowMessageTimestamp() { return Topic::NowTimestamp(); }

  static uint64_t ToSharedTimestamp(MicrosecondTimestamp timestamp)
  {
    return MonotonicTime::XrToSharedMicroseconds(static_cast<uint64_t>(timestamp));
  }

  static MicrosecondTimestamp FromSharedTimestamp(uint64_t timestamp_us)
  {
    return MicrosecondTimestamp(MonotonicTime::SharedToXrMicroseconds(timestamp_us));
  }

  static bool ReadProcessIdentity(uint32_t pid, ProcessIdentity& identity)
  {
    identity = {};
    if (pid == 0)
    {
      return false;
    }

    char path[64] = {};
    std::snprintf(path, sizeof(path), "/proc/%u/stat", pid);

    std::ifstream file(path);
    if (!file.is_open())
    {
      return false;
    }

    std::string line;
    std::getline(file, line);
    if (line.empty())
    {
      return false;
    }

    const size_t rparen = line.rfind(')');
    if (rparen == std::string::npos || rparen + 2U >= line.size())
    {
      return false;
    }

    std::istringstream iss(line.substr(rparen + 2U));
    std::string token;
    for (int field = 3; field <= 22; ++field)
    {
      if (!(iss >> token))
      {
        return false;
      }

      if (field == 22)
      {
        identity.pid = pid;
        identity.starttime = std::strtoull(token.c_str(), nullptr, 10);
        return identity.starttime != 0;
      }
    }

    return false;
  }

  static int FutexWait(std::atomic<uint32_t>* word, uint32_t expected,
                       uint32_t timeout_ms)
  {
    struct timespec timeout = {};
    struct timespec* timeout_ptr = nullptr;
    if (timeout_ms != UINT32_MAX)
    {
      timeout.tv_sec = static_cast<time_t>(timeout_ms / 1000U);
      timeout.tv_nsec = static_cast<long>(timeout_ms % 1000U) * 1000000L;
      timeout_ptr = &timeout;
    }

    return static_cast<int>(syscall(SYS_futex, reinterpret_cast<uint32_t*>(word),
                                    FUTEX_WAIT, expected, timeout_ptr, nullptr, 0));
  }

  static int FutexWake(std::atomic<uint32_t>* word)
  {
    return static_cast<int>(syscall(SYS_futex, reinterpret_cast<uint32_t*>(word),
                                    FUTEX_WAKE, INT32_MAX, nullptr, nullptr, 0));
  }

  static size_t ComputeSharedBytes(uint32_t slot_count, uint32_t subscriber_capacity,
                                   uint32_t queue_capacity, uint32_t topic_name_len)
  {
    size_t offset = 0;
    offset = AlignUp(offset, alignof(SharedHeader));
    offset += sizeof(SharedHeader);

    offset += static_cast<size_t>(topic_name_len) + 1U;

    offset = AlignUp(offset, alignof(SlotControl));
    offset += sizeof(SlotControl) * slot_count;

    offset = AlignUp(offset, alignof(SubscriberControl));
    offset += sizeof(SubscriberControl) * subscriber_capacity;

    offset = AlignUp(offset, alignof(BalancedGroupControl));
    offset += sizeof(BalancedGroupControl);

    offset = AlignUp(offset, alignof(std::atomic<uint32_t>));
    offset += sizeof(std::atomic<uint32_t>) * subscriber_capacity;

    offset = AlignUp(offset, alignof(FreeSlotCell));
    offset += sizeof(FreeSlotCell) * slot_count;

    offset = AlignUp(offset, alignof(Descriptor));
    offset += sizeof(Descriptor) * subscriber_capacity * queue_capacity;

    offset = AlignUp(offset, alignof(TopicData));
    offset += sizeof(TopicData) * slot_count;
    return offset;
  }

  void SetupPointers()
  {
    size_t offset = 0;

    offset = AlignUp(offset, alignof(SharedHeader));
    header_ = reinterpret_cast<SharedHeader*>(base_ + offset);
    offset += sizeof(SharedHeader);

    topic_name_ptr_ = reinterpret_cast<char*>(base_ + offset);
    offset += static_cast<size_t>(header_->topic_name_len) + 1U;

    offset = AlignUp(offset, alignof(SlotControl));
    slots_ = reinterpret_cast<SlotControl*>(base_ + offset);
    offset += sizeof(SlotControl) * slot_count_;

    offset = AlignUp(offset, alignof(SubscriberControl));
    subscribers_ = reinterpret_cast<SubscriberControl*>(base_ + offset);
    offset += sizeof(SubscriberControl) * subscriber_capacity_;

    offset = AlignUp(offset, alignof(BalancedGroupControl));
    balanced_group_ = reinterpret_cast<BalancedGroupControl*>(base_ + offset);
    offset += sizeof(BalancedGroupControl);

    offset = AlignUp(offset, alignof(std::atomic<uint32_t>));
    balanced_members_ = reinterpret_cast<std::atomic<uint32_t>*>(base_ + offset);
    offset += sizeof(std::atomic<uint32_t>) * subscriber_capacity_;

    offset = AlignUp(offset, alignof(FreeSlotCell));
    free_slots_ = reinterpret_cast<FreeSlotCell*>(base_ + offset);
    offset += sizeof(FreeSlotCell) * slot_count_;

    offset = AlignUp(offset, alignof(Descriptor));
    descriptors_ = reinterpret_cast<Descriptor*>(base_ + offset);
    offset += sizeof(Descriptor) * subscriber_capacity_ * queue_capacity_;

    offset = AlignUp(offset, alignof(TopicData));
    payloads_ = reinterpret_cast<TopicData*>(base_ + offset);
  }

  bool HeaderMatchesIdentity() const
  {
    if (header_->name_key != name_key_)
    {
      return false;
    }
    if (header_->domain_crc32 != domain_crc32_)
    {
      return false;
    }
    if (header_->topic_name_len != topic_name_.size())
    {
      return false;
    }
    if (std::memcmp(topic_name_ptr_, topic_name_.c_str(), topic_name_.size() + 1U) != 0)
    {
      return false;
    }
    return true;
  }

  ErrorCode InitializeLayout()
  {
    if (config_.slot_num == 0 || config_.subscriber_num == 0 || config_.queue_num < 2)
    {
      return ErrorCode::ARG_ERR;
    }

    const size_t bytes =
        ComputeSharedBytes(config_.slot_num, config_.subscriber_num, config_.queue_num,
                           static_cast<uint32_t>(topic_name_.size()));

    if (ftruncate(fd_, static_cast<off_t>(bytes)) != 0)
    {
      return ErrorCode::INIT_ERR;
    }

    const struct stat st = GetStat();
    if (st.st_size <= 0)
    {
      return ErrorCode::INIT_ERR;
    }

    mapping_size_ = static_cast<size_t>(st.st_size);
    mapping_ = mmap(nullptr, mapping_size_, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
    if (mapping_ == MAP_FAILED)
    {
      mapping_ = nullptr;
      return ErrorCode::INIT_ERR;
    }

    base_ = static_cast<uint8_t*>(mapping_);
    slot_count_ = config_.slot_num;
    subscriber_capacity_ = config_.subscriber_num;
    queue_capacity_ = config_.queue_num;
    header_ = reinterpret_cast<SharedHeader*>(base_ + AlignUp(0, alignof(SharedHeader)));
    header_->topic_name_len = static_cast<uint32_t>(topic_name_.size());
    SetupPointers();

    header_->magic = MAGIC;
    header_->name_key = name_key_;
    header_->domain_crc32 = domain_crc32_;
    header_->version = VERSION;
    header_->data_size = sizeof(TopicData);
    header_->slot_count = slot_count_;
    header_->subscriber_capacity = subscriber_capacity_;
    header_->queue_capacity = queue_capacity_;
    std::memcpy(topic_name_ptr_, topic_name_.c_str(), topic_name_.size() + 1U);
    header_->publisher_pid.store(self_identity_.pid, std::memory_order_release);
    header_->publisher_starttime.store(self_identity_.starttime,
                                       std::memory_order_release);
    header_->free_queue_head.store(0, std::memory_order_release);
    header_->free_queue_tail.store(slot_count_, std::memory_order_release);
    header_->next_sequence.store(0, std::memory_order_release);
    header_->publish_failures.store(0, std::memory_order_release);

    for (uint32_t i = 0; i < slot_count_; ++i)
    {
      slots_[i].refcount.store(0, std::memory_order_release);
      slots_[i].sequence.store(0, std::memory_order_release);
      slots_[i].timestamp_us = 0;
      std::construct_at(&payloads_[i], TopicData{});
      free_slots_[i].slot_index = i;
      free_slots_[i].sequence.store(static_cast<uint64_t>(i) + 1U,
                                    std::memory_order_release);
    }

    for (uint32_t i = 0; i < subscriber_capacity_; ++i)
    {
      subscribers_[i].active.store(0, std::memory_order_release);
      subscribers_[i].mode.store(
          static_cast<uint32_t>(LinuxSharedSubscriberMode::BROADCAST_FULL),
          std::memory_order_release);
      subscribers_[i].queue_head.store(0, std::memory_order_release);
      subscribers_[i].queue_tail.store(0, std::memory_order_release);
      subscribers_[i].ready_signal.store(0, std::memory_order_release);
      subscribers_[i].dropped_messages.store(0, std::memory_order_release);
      subscribers_[i].owner_pid.store(0, std::memory_order_release);
      subscribers_[i].owner_starttime.store(0, std::memory_order_release);
      subscribers_[i].held_slot.store(INVALID_INDEX, std::memory_order_release);
      balanced_members_[i].store(INVALID_INDEX, std::memory_order_release);
    }

    balanced_group_->rr_cursor.store(0, std::memory_order_release);

    for (size_t i = 0; i < static_cast<size_t>(subscriber_capacity_) * queue_capacity_;
         ++i)
    {
      descriptors_[i] = Descriptor{};
    }

    header_->init_state.store(INIT_READY, std::memory_order_release);
    return ErrorCode::OK;
  }

  ErrorCode AttachLayout()
  {
    const struct stat st = GetStat();
    if (st.st_size <= 0)
    {
      return ErrorCode::NOT_FOUND;
    }

    mapping_size_ = static_cast<size_t>(st.st_size);
    mapping_ = mmap(nullptr, mapping_size_, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
    if (mapping_ == MAP_FAILED)
    {
      mapping_ = nullptr;
      return ErrorCode::INIT_ERR;
    }

    base_ = static_cast<uint8_t*>(mapping_);
    header_ = reinterpret_cast<SharedHeader*>(base_);

    while (header_->init_state.load(std::memory_order_acquire) != INIT_READY)
    {
      usleep(1000);
    }

    if (header_->magic != MAGIC || header_->version != VERSION ||
        header_->data_size != sizeof(TopicData))
    {
      return ErrorCode::CHECK_ERR;
    }

    slot_count_ = header_->slot_count;
    subscriber_capacity_ = header_->subscriber_capacity;
    queue_capacity_ = header_->queue_capacity;
    SetupPointers();
    if (!HeaderMatchesIdentity())
    {
      return ErrorCode::CHECK_ERR;
    }
    return ErrorCode::OK;
  }

  bool TryReclaimStaleSegment()
  {
    int stale_fd = shm_open(shm_name_.c_str(), O_RDWR, 0600);
    if (stale_fd < 0)
    {
      return errno == ENOENT;
    }

    struct stat st = {};
    if (fstat(stale_fd, &st) != 0)
    {
      close(stale_fd);
      return false;
    }

    bool reclaim = false;
    if (st.st_size < static_cast<off_t>(sizeof(SharedHeader)))
    {
      reclaim = true;
    }
    else
    {
      void* mapping = mmap(nullptr, static_cast<size_t>(st.st_size),
                           PROT_READ | PROT_WRITE, MAP_SHARED, stale_fd, 0);
      if (mapping != MAP_FAILED)
      {
        uint8_t* base = static_cast<uint8_t*>(mapping);
        auto* header = reinterpret_cast<SharedHeader*>(mapping);
        const uint32_t init_state = header->init_state.load(std::memory_order_acquire);
        bool identity_match = false;
        const size_t mapping_size = static_cast<size_t>(st.st_size);
        const size_t topic_name_bytes = topic_name_.size() + 1U;
        if (header->magic == MAGIC && header->version == VERSION &&
            header->domain_crc32 == domain_crc32_ &&
            header->topic_name_len == topic_name_.size())
        {
          size_t offset = AlignUp(0, alignof(SharedHeader));
          offset += sizeof(SharedHeader);
          if (offset <= mapping_size && topic_name_bytes <= (mapping_size - offset))
          {
            const char* mapped_topic = reinterpret_cast<const char*>(base + offset);
            identity_match =
                (header->name_key == name_key_) &&
                (std::memcmp(mapped_topic, topic_name_.c_str(), topic_name_bytes) == 0);
          }
        }
        const ProcessIdentity publisher_identity = {
            header->publisher_pid.load(std::memory_order_acquire),
            header->publisher_starttime.load(std::memory_order_acquire),
        };

        if (!identity_match)
        {
          reclaim = false;
        }
        else if (init_state != INIT_READY)
        {
          reclaim = !ProcessAlive(publisher_identity);
        }
        else if (!ProcessAlive(publisher_identity))
        {
          reclaim = true;
        }

        munmap(mapping, static_cast<size_t>(st.st_size));
      }
    }

    close(stale_fd);

    if (!reclaim)
    {
      return false;
    }

    return shm_unlink(shm_name_.c_str()) == 0 || errno == ENOENT;
  }

  void Open()
  {
    open_status_ = ErrorCode::STATE_ERR;
    open_ok_ = false;

    if (create_)
    {
      for (int attempt = 0; attempt < 2; ++attempt)
      {
        fd_ = shm_open(shm_name_.c_str(), O_CREAT | O_EXCL | O_RDWR, 0600);
        if (fd_ >= 0)
        {
          break;
        }

        if (errno != EEXIST || !TryReclaimStaleSegment())
        {
          break;
        }
      }

      if (fd_ < 0)
      {
        open_status_ = (errno == EEXIST) ? ErrorCode::BUSY : ErrorCode::INIT_ERR;
        return;
      }

      open_status_ = InitializeLayout();
    }
    else
    {
      fd_ = shm_open(shm_name_.c_str(), O_RDWR, 0600);
      if (fd_ < 0)
      {
        open_status_ = (errno == ENOENT) ? ErrorCode::NOT_FOUND : ErrorCode::INIT_ERR;
        return;
      }

      open_status_ = AttachLayout();
    }

    if (fd_ >= 0)
    {
      close(fd_);
      fd_ = -1;
    }

    open_ok_ = (open_status_ == ErrorCode::OK);
  }

  void Close()
  {
    if (mapping_ != nullptr)
    {
      munmap(mapping_, mapping_size_);
    }

    mapping_ = nullptr;
    base_ = nullptr;
    header_ = nullptr;
    slots_ = nullptr;
    subscribers_ = nullptr;
    free_slots_ = nullptr;
    descriptors_ = nullptr;
    payloads_ = nullptr;
    mapping_size_ = 0;
    open_ok_ = false;
    open_status_ = ErrorCode::STATE_ERR;
  }

  struct stat GetStat() const
  {
    struct stat st = {};
    fstat(fd_, &st);
    return st;
  }

  Descriptor* DescriptorRing(uint32_t subscriber_index) const
  {
    return descriptors_ + static_cast<size_t>(subscriber_index) * queue_capacity_;
  }

  static bool ProcessAlive(const ProcessIdentity& identity)
  {
    ProcessIdentity current = {};
    if (!ReadProcessIdentity(identity.pid, current))
    {
      return false;
    }

    return current.starttime == identity.starttime;
  }

  bool PublisherValid() const
  {
    if (!publisher_ || header_ == nullptr)
    {
      return false;
    }

    const ProcessIdentity owner = {
        header_->publisher_pid.load(std::memory_order_acquire),
        header_->publisher_starttime.load(std::memory_order_acquire),
    };
    return owner.pid == self_identity_.pid && owner.starttime == self_identity_.starttime;
  }

  void HoldSlot(uint32_t subscriber_index, uint32_t slot_index)
  {
    subscribers_[subscriber_index].held_slot.store(slot_index, std::memory_order_release);
  }

  void ClearHeldSlot(uint32_t subscriber_index, uint32_t slot_index)
  {
    uint32_t expected = slot_index;
    subscribers_[subscriber_index].held_slot.compare_exchange_strong(
        expected, INVALID_INDEX, std::memory_order_acq_rel, std::memory_order_relaxed);
  }

  MicrosecondTimestamp SlotTimestamp(uint32_t slot_index) const
  {
    return FromSharedTimestamp(slots_[slot_index].timestamp_us);
  }

  ErrorCode RegisterBalancedSubscriber(uint32_t subscriber_index)
  {
    for (uint32_t i = 0; i < subscriber_capacity_; ++i)
    {
      uint32_t expected = INVALID_INDEX;
      if (balanced_members_[i].compare_exchange_strong(expected, subscriber_index,
                                                       std::memory_order_acq_rel,
                                                       std::memory_order_relaxed))
      {
        return ErrorCode::OK;
      }
    }
    return ErrorCode::FULL;
  }

  void UnregisterBalancedSubscriber(uint32_t subscriber_index)
  {
    for (uint32_t i = 0; i < subscriber_capacity_; ++i)
    {
      uint32_t expected = subscriber_index;
      if (balanced_members_[i].compare_exchange_strong(expected, INVALID_INDEX,
                                                       std::memory_order_acq_rel,
                                                       std::memory_order_relaxed))
      {
        return;
      }
    }
  }

  bool SelectBalancedSubscriber(uint32_t& subscriber_index)
  {
    const uint64_t base =
        balanced_group_->rr_cursor.fetch_add(1, std::memory_order_acq_rel);
    for (uint32_t offset = 0; offset < subscriber_capacity_; ++offset)
    {
      const uint32_t member_index =
          balanced_members_[(base + offset) % subscriber_capacity_].load(
              std::memory_order_acquire);
      if (member_index == INVALID_INDEX)
      {
        continue;
      }
      if (subscribers_[member_index].active.load(std::memory_order_acquire) == 0)
      {
        continue;
      }
      if (subscribers_[member_index].mode.load(std::memory_order_acquire) !=
          static_cast<uint32_t>(LinuxSharedSubscriberMode::BALANCE_RR))
      {
        continue;
      }
      const ProcessIdentity owner_identity = {
          subscribers_[member_index].owner_pid.load(std::memory_order_acquire),
          subscribers_[member_index].owner_starttime.load(std::memory_order_acquire),
      };
      if (owner_identity.pid == 0 || owner_identity.starttime == 0)
      {
        continue;
      }
      if (!ProcessAlive(owner_identity))
      {
        ReclaimSubscriber(member_index);
        continue;
      }
      if (!QueueHasSpace(member_index))
      {
        continue;
      }
      subscriber_index = member_index;
      return true;
    }
    return false;
  }

  bool ReclaimSubscriber(uint32_t subscriber_index)
  {
    uint32_t expected = 1;
    if (!subscribers_[subscriber_index].active.compare_exchange_strong(
            expected, 0, std::memory_order_acq_rel, std::memory_order_relaxed))
    {
      return false;
    }

    if (subscribers_[subscriber_index].mode.load(std::memory_order_acquire) ==
        static_cast<uint32_t>(LinuxSharedSubscriberMode::BALANCE_RR))
    {
      UnregisterBalancedSubscriber(subscriber_index);
    }

    subscribers_[subscriber_index].owner_pid.store(0, std::memory_order_release);
    subscribers_[subscriber_index].owner_starttime.store(0, std::memory_order_release);
    subscribers_[subscriber_index].mode.store(
        static_cast<uint32_t>(LinuxSharedSubscriberMode::BROADCAST_FULL),
        std::memory_order_release);

    const uint32_t held_slot = subscribers_[subscriber_index].held_slot.exchange(
        INVALID_INDEX, std::memory_order_acq_rel);
    if (held_slot != INVALID_INDEX)
    {
      ReleaseSlot(held_slot);
    }

    Descriptor desc = {};
    while (TryPopDescriptor(subscriber_index, desc) == ErrorCode::OK)
    {
      ReleaseSlot(desc.slot_index);
    }

    return true;
  }

  static void PostReady(SubscriberControl& control)
  {
    control.ready_signal.store(1, std::memory_order_release);
    FutexWake(&control.ready_signal);
  }

  static bool HasQueuedData(const SubscriberControl& control)
  {
    return control.queue_head.load(std::memory_order_acquire) !=
           control.queue_tail.load(std::memory_order_acquire);
  }

  static ErrorCode WaitReady(SubscriberControl& control, uint32_t timeout_ms)
  {
    if (HasQueuedData(control))
    {
      return ErrorCode::OK;
    }

    const bool infinite_wait = (timeout_ms == UINT32_MAX);
    const uint64_t deadline_ms = infinite_wait ? 0 : (NowMonotonicMs() + timeout_ms);

    while (true)
    {
      // 先清提示再检查队列，覆盖检查与进入 futex 等待之间的发布。
      // Clear the hint before checking the queue so a later publication prevents sleep.
      control.ready_signal.exchange(0, std::memory_order_acq_rel);
      if (HasQueuedData(control))
      {
        return ErrorCode::OK;
      }

      uint32_t wait_ms = UINT32_MAX;
      if (!infinite_wait)
      {
        wait_ms = MonotonicTime::RemainingMilliseconds(deadline_ms);
        if (wait_ms == 0)
        {
          return ErrorCode::TIMEOUT;
        }
      }

      wait_ms = MonotonicTime::WaitSliceMilliseconds(wait_ms);

      const int futex_ans = FutexWait(&control.ready_signal, 0, wait_ms);
      if (futex_ans == 0 || errno == EAGAIN || errno == EINTR)
      {
        continue;
      }

      if (errno == ETIMEDOUT)
      {
        if (infinite_wait)
        {
          continue;
        }
        if (MonotonicTime::RemainingMilliseconds(deadline_ms) == 0 &&
            !HasQueuedData(control))
        {
          return ErrorCode::TIMEOUT;
        }
        continue;
      }

      return ErrorCode::FAILED;
    }
  }

  void ScavengeDeadSubscribers()
  {
    for (uint32_t i = 0; i < subscriber_capacity_; ++i)
    {
      if (subscribers_[i].active.load(std::memory_order_acquire) == 0)
      {
        continue;
      }

      const ProcessIdentity owner_identity = {
          subscribers_[i].owner_pid.load(std::memory_order_acquire),
          subscribers_[i].owner_starttime.load(std::memory_order_acquire),
      };
      if (ProcessAlive(owner_identity))
      {
        continue;
      }

      ReclaimSubscriber(i);
    }
  }

  bool QueueHasSpace(uint32_t subscriber_index) const
  {
    const SubscriberControl& control = subscribers_[subscriber_index];
    const uint32_t head = control.queue_head.load(std::memory_order_acquire);
    const uint32_t tail = control.queue_tail.load(std::memory_order_relaxed);
    const uint32_t next_tail = (tail + 1U) % queue_capacity_;
    return next_tail != head;
  }

  void PushDescriptor(uint32_t subscriber_index, const Descriptor& descriptor)
  {
    SubscriberControl& control = subscribers_[subscriber_index];
    Descriptor* ring = DescriptorRing(subscriber_index);

    const uint32_t tail = control.queue_tail.load(std::memory_order_relaxed);
    ring[tail] = descriptor;
    const uint32_t next_tail = (tail + 1U) % queue_capacity_;
    control.queue_tail.store(next_tail, std::memory_order_release);
    PostReady(control);
  }

  ErrorCode TryPopDescriptor(uint32_t subscriber_index, Descriptor& descriptor)
  {
    SubscriberControl& control = subscribers_[subscriber_index];
    Descriptor* ring = DescriptorRing(subscriber_index);

    while (true)
    {
      uint32_t head = control.queue_head.load(std::memory_order_relaxed);
      const uint32_t tail = control.queue_tail.load(std::memory_order_acquire);
      if (head == tail)
      {
        return ErrorCode::EMPTY;
      }

      descriptor = ring[head];
      const uint32_t next_head = (head + 1U) % queue_capacity_;
      if (control.queue_head.compare_exchange_weak(
              head, next_head, std::memory_order_acq_rel, std::memory_order_relaxed))
      {
        return ErrorCode::OK;
      }
    }
  }

  ErrorCode DropDescriptor(uint32_t subscriber_index)
  {
    SubscriberControl& control = subscribers_[subscriber_index];
    Descriptor* ring = DescriptorRing(subscriber_index);

    while (true)
    {
      uint32_t head = control.queue_head.load(std::memory_order_relaxed);
      const uint32_t tail = control.queue_tail.load(std::memory_order_acquire);
      if (head == tail)
      {
        return ErrorCode::EMPTY;
      }

      const Descriptor descriptor = ring[head];
      const uint32_t next_head = (head + 1U) % queue_capacity_;
      if (control.queue_head.compare_exchange_weak(
              head, next_head, std::memory_order_acq_rel, std::memory_order_relaxed))
      {
        control.dropped_messages.fetch_add(1, std::memory_order_relaxed);
        ReleaseSlot(descriptor.slot_index);
        return ErrorCode::OK;
      }
    }
  }

  ErrorCode PopFreeSlot(uint32_t& slot_index)
  {
    while (true)
    {
      uint64_t head = header_->free_queue_head.load(std::memory_order_relaxed);
      FreeSlotCell& cell = free_slots_[head % slot_count_];
      const uint64_t seq = cell.sequence.load(std::memory_order_acquire);
      const intptr_t diff = static_cast<intptr_t>(seq) - static_cast<intptr_t>(head + 1U);

      if (diff == 0)
      {
        if (header_->free_queue_head.compare_exchange_weak(
                head, head + 1U, std::memory_order_acq_rel, std::memory_order_relaxed))
        {
          slot_index = cell.slot_index;
          cell.sequence.store(head + slot_count_, std::memory_order_release);
          return ErrorCode::OK;
        }
      }
      else if (diff < 0)
      {
        return ErrorCode::FULL;
      }
    }
  }

  void RecycleSlot(uint32_t slot_index)
  {
    slots_[slot_index].sequence.store(0, std::memory_order_release);
    slots_[slot_index].timestamp_us = 0;

    while (true)
    {
      uint64_t tail = header_->free_queue_tail.load(std::memory_order_relaxed);
      FreeSlotCell& cell = free_slots_[tail % slot_count_];
      const uint64_t seq = cell.sequence.load(std::memory_order_acquire);
      const intptr_t diff = static_cast<intptr_t>(seq) - static_cast<intptr_t>(tail);

      if (diff == 0)
      {
        if (header_->free_queue_tail.compare_exchange_weak(
                tail, tail + 1U, std::memory_order_acq_rel, std::memory_order_relaxed))
        {
          cell.slot_index = slot_index;
          cell.sequence.store(tail + 1U, std::memory_order_release);
          return;
        }
      }
    }
  }

  void ReleaseSlot(uint32_t slot_index)
  {
    const uint32_t prev =
        slots_[slot_index].refcount.fetch_sub(1, std::memory_order_acq_rel);
    DEV_ASSERT(prev > 0);
    if (prev == 1)
    {
      RecycleSlot(slot_index);
    }
  }

  template <bool HAS_TIMESTAMP>
  ErrorCode PublishData(SharedData& data,
                        MicrosecondTimestamp timestamp = MicrosecondTimestamp())
  {
    if (!data.Valid() || data.topic_ != this)
    {
      return ErrorCode::STATE_ERR;
    }

    uint32_t active_count = 0;
    uint32_t balanced_target = INVALID_INDEX;
    bool has_balanced_subscriber = false;
    for (uint32_t i = 0; i < subscriber_capacity_; ++i)
    {
      if (subscribers_[i].active.load(std::memory_order_acquire) == 0)
      {
        continue;
      }

      const LinuxSharedSubscriberMode mode = static_cast<LinuxSharedSubscriberMode>(
          subscribers_[i].mode.load(std::memory_order_acquire));
      if (mode == LinuxSharedSubscriberMode::BALANCE_RR)
      {
        has_balanced_subscriber = true;
        continue;
      }

      if (!QueueHasSpace(i))
      {
        ScavengeDeadSubscribers();
        if (subscribers_[i].active.load(std::memory_order_acquire) == 0)
        {
          continue;
        }

        if (mode == LinuxSharedSubscriberMode::BROADCAST_DROP_OLD)
        {
          const ErrorCode drop_ans = DropDescriptor(i);
          if (drop_ans == ErrorCode::EMPTY && QueueHasSpace(i))
          {
            // 消费者可能在 QueueHasSpace() 和 DropDescriptor() 之间弹走旧描述符。
            // 此时队列已经有空位，发布者继续写入即可，不能把竞态误报成 FULL。
          }
          else if (drop_ans != ErrorCode::OK)
          {
            header_->publish_failures.fetch_add(1, std::memory_order_relaxed);
            data.Reset();
            return ErrorCode::FULL;
          }
        }
        else
        {
          subscribers_[i].dropped_messages.fetch_add(1, std::memory_order_relaxed);
          header_->publish_failures.fetch_add(1, std::memory_order_relaxed);
          data.Reset();
          return ErrorCode::FULL;
        }
      }

      ++active_count;
    }

    if (has_balanced_subscriber)
    {
      if (!SelectBalancedSubscriber(balanced_target))
      {
        ScavengeDeadSubscribers();
        if (!SelectBalancedSubscriber(balanced_target))
        {
          header_->publish_failures.fetch_add(1, std::memory_order_relaxed);
          data.Reset();
          return ErrorCode::FULL;
        }
      }
      ++active_count;
    }

    if (active_count == 0)
    {
      data.Reset();
      return ErrorCode::OK;
    }

    const uint64_t sequence =
        header_->next_sequence.fetch_add(1, std::memory_order_acq_rel) + 1ULL;
    if constexpr (!HAS_TIMESTAMP)
    {
      timestamp = NowMessageTimestamp();
    }
    SlotControl& slot = slots_[data.slot_index_];
    slot.refcount.store(active_count, std::memory_order_release);
    slot.timestamp_us = ToSharedTimestamp(timestamp);
    slot.sequence.store(sequence, std::memory_order_release);

    const Descriptor descriptor = {data.slot_index_, 0U, sequence};
    for (uint32_t i = 0; i < subscriber_capacity_; ++i)
    {
      if (subscribers_[i].active.load(std::memory_order_acquire) == 0)
      {
        continue;
      }
      const LinuxSharedSubscriberMode mode = static_cast<LinuxSharedSubscriberMode>(
          subscribers_[i].mode.load(std::memory_order_acquire));
      if (mode == LinuxSharedSubscriberMode::BALANCE_RR)
      {
        continue;
      }
      PushDescriptor(i, descriptor);
    }

    if (balanced_target != INVALID_INDEX)
    {
      PushDescriptor(balanced_target, descriptor);
    }

    data.topic_ = nullptr;
    data.slot_index_ = INVALID_INDEX;
    return ErrorCode::OK;
  }

  bool create_ = false;
  bool publisher_ = false;
  LinuxSharedTopicConfig config_;
  uint32_t domain_crc32_ = 0;
  std::string topic_name_;
  uint64_t name_key_ = 0;
  std::string shm_name_;
  ProcessIdentity self_identity_ = {};

  int fd_ = -1;
  void* mapping_ = nullptr;
  uint8_t* base_ = nullptr;
  size_t mapping_size_ = 0;

  SharedHeader* header_ = nullptr;
  char* topic_name_ptr_ = nullptr;
  SlotControl* slots_ = nullptr;
  SubscriberControl* subscribers_ = nullptr;
  BalancedGroupControl* balanced_group_ = nullptr;
  std::atomic<uint32_t>* balanced_members_ = nullptr;
  FreeSlotCell* free_slots_ = nullptr;
  Descriptor* descriptors_ = nullptr;
  TopicData* payloads_ = nullptr;

  uint32_t slot_count_ = 0;
  uint32_t subscriber_capacity_ = 0;
  uint32_t queue_capacity_ = 0;

  bool open_ok_ = false;
  ErrorCode open_status_ = ErrorCode::STATE_ERR;
};

}  // namespace LibXR

#endif
