#pragma once

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <limits>
#include <new>

#include "libxr_def.hpp"
#include "libxr_mem.hpp"

namespace LibXR
{
/**
 * @class SPSCQueueBase
 * @brief 单生产者单消费者字节队列内核 / Single-producer single-consumer byte-queue core
 *
 * 这个内核只负责 SPSC ring 的索引推进、容量判断与原始字节存储，不包含多消费者
 * 抢占逻辑，也不暴露任何 CAS 路径。上层 `SPSCQueue<Data>` 只负责把具体类型
 * `Data` 映射到这些字节槽位。
 *
 * This core contains only the SPSC ring progression, capacity checks, and raw
 * byte storage. It does not include any multi-consumer claiming logic or expose
 * any CAS-based path. The upper `SPSCQueue<Data>` wrapper is responsible only
 * for mapping the concrete `Data` type onto these byte slots.
 */
class alignas(LibXR::CONCURRENCY_ALIGNMENT) SPSCQueueBase
{
 public:
  using IndexType = size_t;  ///< 环形缓冲区索引类型 / Ring-buffer index type.

  /**
   * @brief 构造 SPSC 字节队列内核 / Construct the SPSC byte-queue core
   * @param element_size 单个 payload 的字节数 / Byte size of one payload
   * @param capacity 队列容量 / Queue capacity
   */
  SPSCQueueBase(size_t element_size, size_t capacity)
      : element_size_(element_size),
        capacity_(capacity),
        payload_alloc_align_(alignof(std::max_align_t)),
        payload_stride_(ComputeStride(element_size, alignof(std::byte))),
        payloads_(nullptr),
        head_(0),
        tail_(0)
  {
    InitStorage();
  }

  /**
   * @brief 构造 SPSC 字节队列内核并指定元素对齐 / Construct the SPSC byte-queue core
   *        with explicit element alignment
   * @param element_size 单个 payload 的字节数 / Byte size of one payload
   * @param element_align 单个 payload 的对齐要求 / Alignment requirement of one payload
   * @param capacity 队列容量 / Queue capacity
   */
  SPSCQueueBase(size_t element_size, size_t element_align, size_t capacity)
      : element_size_(element_size),
        capacity_(capacity),
        payload_alloc_align_(alignof(std::max_align_t)),
        payload_stride_(ComputeStride(element_size, element_align)),
        payloads_(nullptr),
        head_(0),
        tail_(0)
  {
    InitStorage();
  }

 private:
  void InitStorage()
  {
    ASSERT(element_size_ > 0);
    ASSERT(payload_alloc_align_ > 0);
    ASSERT(capacity_ > 0);
    ASSERT(capacity_ <= std::numeric_limits<size_t>::max() - 1);
    ASSERT((payload_alloc_align_ & (payload_alloc_align_ - 1)) == 0);

    const size_t payload_bytes = MultiplyChecked(payload_stride_, RingCapacity());
    payloads_ = static_cast<std::byte*>(
        ::operator new[](payload_bytes, std::align_val_t(payload_alloc_align_)));
  }

 public:
  /**
   * @brief 析构 SPSC 字节队列内核 / Destroy the SPSC byte-queue core
   */
  ~SPSCQueueBase()
  {
    ::operator delete[](payloads_, std::align_val_t(payload_alloc_align_));
  }

  /**
   * @brief 按字节入队一个 payload / Enqueue one payload by bytes
   * @param value 指向待入队 payload 的指针 / Pointer to the payload to enqueue
   * @return 成功返回 `ErrorCode::OK`；队列满返回 `ErrorCode::FULL`；空指针返回
   *         `ErrorCode::PTR_NULL`
   *         Returns `ErrorCode::OK` on success; returns `ErrorCode::FULL` when
   *         the queue is full; returns `ErrorCode::PTR_NULL` when `value` is null
   */
  ErrorCode PushBytes(const void* value)
  {
    if (value == nullptr)
    {
      return ErrorCode::PTR_NULL;
    }

    const auto current_tail = tail_.load(std::memory_order_relaxed);
    const auto next_tail = Increment(current_tail);

    if (next_tail == head_.load(std::memory_order_acquire))
    {
      return ErrorCode::FULL;
    }

    LibXR::Memory::FastCopy(PayloadPtr(current_tail), value, element_size_);
    tail_.store(next_tail, std::memory_order_release);
    return ErrorCode::OK;
  }

  /**
   * @brief 按字节出队一个 payload；传空指针时仅丢弃队头元素
   *        / Dequeue one payload by bytes; pass null to discard the front item only
   * @param value 用于接收 payload 的缓冲区；传 `nullptr` 时仅丢弃
   *        / Buffer that receives the payload; pass `nullptr` to discard only
   * @return 成功返回 `ErrorCode::OK`；队列空返回 `ErrorCode::EMPTY`
   *         Returns `ErrorCode::OK` on success; returns `ErrorCode::EMPTY` when
   *         the queue is empty
   */
  ErrorCode PopBytes(void* value = nullptr)
  {
    const auto current_head = head_.load(std::memory_order_relaxed);

    if (current_head == tail_.load(std::memory_order_acquire))
    {
      return ErrorCode::EMPTY;
    }

    if (value != nullptr)
    {
      LibXR::Memory::FastCopy(value, PayloadPtr(current_head), element_size_);
    }

    head_.store(Increment(current_head), std::memory_order_release);
    return ErrorCode::OK;
  }

  /**
   * @brief 按字节查看一个队头 payload 但不出队 / Peek one front payload by bytes without
   * dequeuing it
   * @param value 用于接收 payload 的缓冲区 / Buffer that receives the payload
   * @return 成功返回 `ErrorCode::OK`；队列空返回 `ErrorCode::EMPTY`；空指针返回
   *         `ErrorCode::PTR_NULL`
   *         Returns `ErrorCode::OK` on success; returns `ErrorCode::EMPTY` when
   *         the queue is empty; returns `ErrorCode::PTR_NULL` when `value` is null
   */
  ErrorCode PeekBytes(void* value)
  {
    if (value == nullptr)
    {
      return ErrorCode::PTR_NULL;
    }

    const auto current_head = head_.load(std::memory_order_relaxed);
    if (current_head == tail_.load(std::memory_order_acquire))
    {
      return ErrorCode::EMPTY;
    }

    LibXR::Memory::FastCopy(value, PayloadPtr(current_head), element_size_);
    return ErrorCode::OK;
  }

  /**
   * @brief 按字节批量入队多个 payload / Enqueue multiple payloads by bytes
   * @param data 指向 payload 数组的字节指针 / Byte pointer to the payload array
   * @param count payload 个数 / Number of payloads
   * @return 成功返回 `ErrorCode::OK`；队列满返回 `ErrorCode::FULL`；
   *         `count` 不为 0 且 `data` 为空时返回 `ErrorCode::PTR_NULL`
   *         Returns `ErrorCode::OK` on success; returns `ErrorCode::FULL` when the queue
   *         is full; returns `ErrorCode::PTR_NULL` when `data` is null and `count`
   *         is not 0
   */
  ErrorCode PushBatchBytes(const void* data, size_t count)
  {
    if (count == 0U)
    {
      return ErrorCode::OK;
    }
    if (data == nullptr)
    {
      return ErrorCode::PTR_NULL;
    }

    const auto current_tail = tail_.load(std::memory_order_relaxed);
    const auto current_head = head_.load(std::memory_order_acquire);
    const size_t capacity = RingCapacity();
    const size_t free_space = (current_tail >= current_head)
                                  ? (capacity - (current_tail - current_head) - 1)
                                  : (current_head - current_tail - 1);

    if (free_space < count)
    {
      return ErrorCode::FULL;
    }

    const auto* src = static_cast<const std::byte*>(data);
    for (size_t index = 0; index < count; ++index)
    {
      LibXR::Memory::FastCopy(PayloadPtr((current_tail + index) % capacity),
                              src + index * element_size_, element_size_);
    }

    tail_.store((current_tail + count) % capacity, std::memory_order_release);
    return ErrorCode::OK;
  }

  /**
   * @brief 通过写入器回调批量入队 payload / Enqueue payloads through a writer callback
   * @tparam Writer 写入器类型 / Writer callback type
   * @param count payload 个数 / Number of payloads
   * @param writer 写入器，签名为 `ErrorCode(void* buffer, size_t chunk_count)`
   *               / Writer with signature `ErrorCode(void* buffer, size_t chunk_count)`
   * @return 成功返回 `ErrorCode::OK`；空间不足返回
   * `ErrorCode::FULL`；否则返回写入器错误码 / Returns `ErrorCode::OK` on success,
   * `ErrorCode::FULL` when free space is insufficient, otherwise returns the writer error
   * code
   * @note 写入器每次收到一段连续 payload 存储区，字节长度为
   *       `chunk_count * element_size` / The writer receives one contiguous
   *       payload storage chunk each time, with byte length
   *       `chunk_count * element_size`
   */
  template <typename Writer>
  ErrorCode PushBytesWithWriter(size_t count, Writer&& writer)
  {
    if (count == 0U)
    {
      return ErrorCode::OK;
    }

    const auto current_tail = tail_.load(std::memory_order_relaxed);
    const auto current_head = head_.load(std::memory_order_acquire);
    const size_t capacity = RingCapacity();
    const size_t free_space = (current_tail >= current_head)
                                  ? (capacity - (current_tail - current_head) - 1)
                                  : (current_head - current_tail - 1);

    if (free_space < count)
    {
      return ErrorCode::FULL;
    }

    const size_t first_chunk = std::min(count, capacity - current_tail);
    Writer& writer_ref = writer;
    const ErrorCode first_ec = writer_ref(PayloadPtr(current_tail), first_chunk);
    if (first_ec != ErrorCode::OK)
    {
      return first_ec;
    }

    if (count > first_chunk)
    {
      const ErrorCode second_ec = writer_ref(PayloadPtr(0), count - first_chunk);
      if (second_ec != ErrorCode::OK)
      {
        return second_ec;
      }
    }

    tail_.store((current_tail + count) % capacity, std::memory_order_release);
    return ErrorCode::OK;
  }

  /**
   * @brief 通过一次双 span 回调生产 payload 前缀 / Produce a payload prefix through one
   *        two-span writer callback.
   * @tparam Writer 写入器类型 / Writer callback type.
   * @param limit 最多提供的 payload 个数 / Maximum number of offered payloads.
   * @param writer 接收两段存储和元素数，返回已写入的前缀个数 /
   *        Receives two storage pointers and element counts; returns the written prefix
   *        count.
   * @return 实际发布到队列的 payload 个数 / Number of payloads published to the queue.
   * @pre 队列只有一个 producer；回调期间不得重入同侧操作或 Reset /
   *      The queue has one producer; no same-side operation or Reset during the callback.
   * @note 可提供数量为零时不调用回调；第二段为空时指针为 `nullptr`，所有指针只在回调期间
   *       有效 / The callback is skipped when the offered count is zero; an empty second
   *       span is `nullptr`, and all pointers are valid only during the callback.
   */
  template <typename Writer>
  size_t ProduceWithWriter(size_t limit, Writer&& writer)
  {
    if (limit == 0U)
    {
      return 0U;
    }

    const auto current_tail = tail_.load(std::memory_order_relaxed);
    const auto current_head = head_.load(std::memory_order_acquire);
    const size_t capacity = RingCapacity();
    const size_t free_space = (current_tail >= current_head)
                                  ? (capacity - (current_tail - current_head) - 1U)
                                  : (current_head - current_tail - 1U);
    const size_t offered = std::min(limit, free_space);
    if (offered == 0U)
    {
      return 0U;
    }

    const size_t first_count = std::min(offered, capacity - current_tail);
    const size_t second_count = offered - first_count;
    void* const second = second_count == 0U ? nullptr : PayloadPtr(0U);
    const size_t produced =
        writer(PayloadPtr(current_tail), first_count, second, second_count);
    ASSERT(produced <= offered);
    if (produced != 0U)
    {
      tail_.store((current_tail + produced) % capacity, std::memory_order_release);
    }
    return produced;
  }

  /**
   * @brief 按字节批量出队多个 payload / Dequeue multiple payloads by bytes
   * @param data 用于接收 payload 的字节缓冲区；传 `nullptr` 时仅丢弃
   *        / Byte buffer receiving payloads; pass `nullptr` to discard only
   * @param count payload 个数 / Number of payloads
   * @return 成功返回 `ErrorCode::OK`；元素不足返回 `ErrorCode::EMPTY`
   *         Returns `ErrorCode::OK` on success; returns `ErrorCode::EMPTY` when
   *         there are not enough payloads available
   */
  ErrorCode PopBatchBytes(void* data, size_t count)
  {
    if (count == 0U)
    {
      return ErrorCode::OK;
    }

    const auto current_head = head_.load(std::memory_order_relaxed);
    const auto current_tail = tail_.load(std::memory_order_acquire);
    const size_t capacity = RingCapacity();
    const size_t available = (current_tail >= current_head)
                                 ? (current_tail - current_head)
                                 : (capacity - current_head + current_tail);

    if (available < count)
    {
      return ErrorCode::EMPTY;
    }

    auto* dst = static_cast<std::byte*>(data);
    if (dst != nullptr)
    {
      for (size_t index = 0; index < count; ++index)
      {
        LibXR::Memory::FastCopy(dst + index * element_size_,
                                PayloadPtr((current_head + index) % capacity),
                                element_size_);
      }
    }

    head_.store((current_head + count) % capacity, std::memory_order_release);
    return ErrorCode::OK;
  }

  /**
   * @brief 通过读取器回调批量出队 payload / Dequeue payloads through a reader callback
   * @tparam Reader 读取器类型 / Reader callback type
   * @param count payload 个数 / Number of payloads
   * @param reader 读取器，签名为 `ErrorCode(const void* buffer, size_t chunk_count)`
   *               / Reader with signature `ErrorCode(const void* buffer, size_t
   * chunk_count)`
   * @return 成功返回 `ErrorCode::OK`；元素不足返回
   * `ErrorCode::EMPTY`；否则返回读取器错误码 / Returns `ErrorCode::OK` on success,
   * `ErrorCode::EMPTY` when there are not enough payloads, otherwise returns the reader
   * error code
   * @note 读取器每次收到一段连续 payload 存储区，字节长度为
   *       `chunk_count * element_size` / The reader receives one contiguous
   *       payload storage chunk each time, with byte length
   *       `chunk_count * element_size`
   */
  template <typename Reader>
  ErrorCode PopBytesWithReader(size_t count, Reader&& reader)
  {
    if (count == 0U)
    {
      return ErrorCode::OK;
    }

    const auto current_head = head_.load(std::memory_order_relaxed);
    const auto current_tail = tail_.load(std::memory_order_acquire);
    const size_t capacity = RingCapacity();
    const size_t available = (current_tail >= current_head)
                                 ? (current_tail - current_head)
                                 : (capacity - current_head + current_tail);

    if (available < count)
    {
      return ErrorCode::EMPTY;
    }

    const size_t first_chunk = std::min(count, capacity - current_head);
    Reader& reader_ref = reader;
    const ErrorCode first_ec = reader_ref(PayloadPtr(current_head), first_chunk);
    if (first_ec != ErrorCode::OK)
    {
      return first_ec;
    }

    if (count > first_chunk)
    {
      const ErrorCode second_ec = reader_ref(PayloadPtr(0), count - first_chunk);
      if (second_ec != ErrorCode::OK)
      {
        return second_ec;
      }
    }

    head_.store((current_head + count) % capacity, std::memory_order_release);
    return ErrorCode::OK;
  }

  /**
   * @brief 通过一次双 span 回调消费 payload 前缀 / Consume a payload prefix through one
   *        two-span reader callback.
   * @tparam Reader 读取器类型 / Reader callback type.
   * @param limit 最多提供的 payload 个数 / Maximum number of offered payloads.
   * @param reader 接收两段数据和元素数，返回已接受的前缀个数 /
   *        Receives two data pointers and element counts; returns the accepted prefix
   * count.
   * @return 实际从队列移除的 payload 个数 / Number of payloads removed from the queue.
   * @pre 队列只有一个 consumer；回调期间不得重入同侧操作或 Reset /
   *      The queue has one consumer; no same-side operation or Reset during the callback.
   * @note 可提供数量为零时不调用回调；返回值不得超过可提供数量；所有指针只在回调期间有效
   * / The callback is skipped when the offered count is zero and must return no more than
   * the offered count; all pointers are valid only during the callback.
   */
  template <typename Reader>
  size_t ConsumeWithReader(size_t limit, Reader&& reader)
  {
    if (limit == 0U)
    {
      return 0U;
    }

    const auto current_head = head_.load(std::memory_order_relaxed);
    const auto current_tail = tail_.load(std::memory_order_acquire);
    const size_t capacity = RingCapacity();
    const size_t available = (current_tail >= current_head)
                                 ? (current_tail - current_head)
                                 : (capacity - current_head + current_tail);
    const size_t offered = std::min(limit, available);
    if (offered == 0U)
    {
      return 0U;
    }

    const size_t first_count = std::min(offered, capacity - current_head);
    const size_t second_count = offered - first_count;
    const void* const second = second_count == 0U ? nullptr : PayloadPtr(0U);
    const size_t accepted =
        reader(PayloadPtr(current_head), first_count, second, second_count);
    ASSERT(accepted <= offered);
    if (accepted != 0U)
    {
      head_.store((current_head + accepted) % capacity, std::memory_order_release);
    }
    return accepted;
  }

  /**
   * @brief 按字节批量查看多个 payload 但不出队
   *        / Peek multiple payloads by bytes without dequeuing them
   * @param data 用于接收 payload 的字节缓冲区 / Byte buffer receiving payloads
   * @param count payload 个数 / Number of payloads
   * @return 成功返回 `ErrorCode::OK`；元素不足返回 `ErrorCode::EMPTY`；
   *         `count` 不为 0 且 `data` 为空时返回 `ErrorCode::PTR_NULL`
   *         Returns `ErrorCode::OK` on success; returns `ErrorCode::EMPTY` when there are
   *         not enough payloads available; returns `ErrorCode::PTR_NULL` when `data` is
   *         null and `count` is not 0
   */
  ErrorCode PeekBatchBytes(void* data, size_t count)
  {
    if (count == 0U)
    {
      return ErrorCode::OK;
    }
    if (data == nullptr)
    {
      return ErrorCode::PTR_NULL;
    }

    const auto current_head = head_.load(std::memory_order_relaxed);
    const auto current_tail = tail_.load(std::memory_order_acquire);
    const size_t capacity = RingCapacity();
    const size_t available = (current_tail >= current_head)
                                 ? (current_tail - current_head)
                                 : (capacity - current_head + current_tail);

    if (available < count)
    {
      return ErrorCode::EMPTY;
    }

    auto* dst = static_cast<std::byte*>(data);
    for (size_t index = 0; index < count; ++index)
    {
      LibXR::Memory::FastCopy(dst + index * element_size_,
                              PayloadPtr((current_head + index) % capacity),
                              element_size_);
    }
    return ErrorCode::OK;
  }

  /**
   * @brief 从 consumer 侧丢弃当前可见的全部 payload / Discard all currently available
   *        payloads from the consumer side
   *
   * 本方法读取 producer 已发布的 tail 快照，并将 consumer head 推进到该位置；不会
   * 写入 tail，因此单个 producer 可以继续运行。与该快照并发发布的 payload 可能被
   * 丢弃，也可能保留。
   *
   * This method snapshots the producer-published tail and advances the consumer
   * head to it. It never writes tail, so the single producer may continue.
   * Payloads published concurrently with the snapshot may be discarded or retained.
   */
  void Reset()
  {
    const auto current_tail = tail_.load(std::memory_order_acquire);
    head_.store(current_tail, std::memory_order_release);
  }

  /**
   * @brief 获取当前已用元素数 / Get the current element count
   * @return 当前元素个数 / Current number of stored payloads
   */
  size_t Size() const
  {
    const auto current_head = head_.load(std::memory_order_acquire);
    const auto current_tail = tail_.load(std::memory_order_acquire);
    return (current_tail >= current_head)
               ? (current_tail - current_head)
               : (RingCapacity() - current_head + current_tail);
  }

  /**
   * @brief 获取剩余空槽数 / Get the current free-slot count
   * @return 当前空槽个数 / Current number of free slots
   */
  size_t EmptySize() const { return capacity_ - Size(); }

  /**
   * @brief 获取队列最大容量 / Get the maximum queue capacity
   * @return 队列容量 / Queue capacity
   */
  size_t MaxSize() const { return capacity_; }

 private:
  /**
   * @brief 获取指定槽位 payload 起始地址 / Get the payload base address of one slot
   * @param index 目标槽位下标 / Target slot index
   * @return 指向目标槽位 payload 起始地址的指针 / Pointer to the slot payload base
   * address
   */
  std::byte* PayloadPtr(IndexType index) { return payloads_ + index * payload_stride_; }

  /**
   * @brief 获取指定槽位 payload 起始地址（只读）
   *        / Get the payload base address of one slot (const)
   * @param index 目标槽位下标 / Target slot index
   * @return 指向目标槽位 payload 起始地址的只读指针
   *         / Read-only pointer to the slot payload base address
   */
  const std::byte* PayloadPtr(IndexType index) const
  {
    return payloads_ + index * payload_stride_;
  }

  /**
   * @brief 获取环形缓冲区的物理槽位总数 / Get the physical ring-slot count
   * @return 物理环形槽位总数（含一个保留空槽）
   *         / Physical ring-slot count including one reserved empty slot
   */
  size_t RingCapacity() const { return capacity_ + 1; }

  /**
   * @brief 沿环形缓冲区推进一个槽位 / Advance one slot along the ring
   * @param index 当前槽位下标 / Current slot index
   * @return 推进后的槽位下标 / Advanced slot index
   */
  IndexType Increment(IndexType index) const { return (index + 1) % RingCapacity(); }

  /// @brief 禁止拷贝构造 / Non-copyable.
  SPSCQueueBase(const SPSCQueueBase&);
  /// @brief 禁止拷贝赋值 / Non-copy-assignable.
  SPSCQueueBase& operator=(const SPSCQueueBase&);
  /// @brief 禁止移动构造 / Non-movable.
  SPSCQueueBase(SPSCQueueBase&&);
  /// @brief 禁止移动赋值 / Non-move-assignable.
  SPSCQueueBase& operator=(SPSCQueueBase&&);

  /**
   * @brief 安全地向上对齐字节数 / Safely align one byte count upward
   * @param size 待对齐字节数 / Byte count to align
   * @param align 目标对齐粒度 / Target alignment granularity
   * @return 对齐后的字节数 / Aligned byte count
   */
  static size_t AlignUpChecked(size_t size, size_t align)
  {
    ASSERT(align > 0);
    ASSERT((align & (align - 1)) == 0);
    ASSERT(size <= std::numeric_limits<size_t>::max() - (align - 1));
    return ((size + align - 1) / align) * align;
  }

  static size_t ComputeStride(size_t element_size, size_t element_align)
  {
    ASSERT(element_size > 0);
    ASSERT(element_align > 0);
    ASSERT((element_align & (element_align - 1)) == 0);
    return AlignUpChecked(element_size, element_align);
  }

  /**
   * @brief 安全地计算两个字节数的乘积 / Safely multiply two byte counts
   * @param lhs 左操作数 / Left operand
   * @param rhs 右操作数 / Right operand
   * @return 乘积结果 / Product result
   */
  static size_t MultiplyChecked(size_t lhs, size_t rhs)
  {
    if (lhs == 0 || rhs == 0)
    {
      return 0;
    }

    ASSERT(lhs <= std::numeric_limits<size_t>::max() / rhs);
    return lhs * rhs;
  }

  const size_t element_size_;  ///< 单个 payload 的字节数 / Byte size of one payload.
  const size_t capacity_;      ///< 队列容量 / Queue capacity.
  const size_t payload_alloc_align_;  ///< 整体分配对齐 / Allocation alignment for the
                                      ///< payload buffer.
  const size_t payload_stride_;  ///< 相邻 payload 槽位之间的步长 / Byte stride between
                                 ///< adjacent payload slots.
  std::byte* payloads_;          ///< payload 字节缓冲区 / Byte buffer storing payloads.

  alignas(LibXR::CONCURRENCY_ALIGNMENT) std::atomic<
      IndexType> head_;  ///< 下一个待出队的环形下标 / Next ring index to dequeue.
  alignas(LibXR::CONCURRENCY_ALIGNMENT) std::atomic<
      IndexType> tail_;  ///< 下一个待入队的环形下标 / Next ring index to enqueue.
};
}  // namespace LibXR
