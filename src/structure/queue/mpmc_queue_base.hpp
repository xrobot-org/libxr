#pragma once

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <new>
#include <type_traits>

#include "libxr_def.hpp"

namespace LibXR
{
/**
 * @class MPMCQueueBase
 * @brief 有界 MPMC 字节队列内核 / Bounded MPMC byte-queue core
 *
 * 这个内核把并发协议和字节搬运集中在一个非模板实现里，以减少不同 payload
 * 类型各自实例化一整套无锁协议所带来的 flash 膨胀。它只负责搬运固定大小、
 * 默认字宽对齐的字节 payload；类型语义由上层薄包装负责。
 *
 * This core keeps the concurrency protocol and byte-copying logic in one
 * non-template implementation so different payload types do not each instantiate
 * a full copy of the lock-free protocol. It only moves fixed-size, word-aligned
 * byte payloads; type semantics are handled by thin wrappers above it.
 */
class MPMCQueueBase
{
 public:
  using SequenceType =
      size_t;  ///< 单调递增的逻辑序号类型 / Monotonic logical sequence type.
  using SequenceDiffType =
      std::make_signed_t<SequenceType>;  ///< 序号差值判定类型 / Signed type used for
                                         ///< sequence-delta checks.

  /**
   * @brief 构造一个字节队列内核 / Construct one byte-queue core
   * @param element_size 单个 payload 的字节数 / Byte size of one payload
   * @param capacity 队列容量 / Queue capacity
   */
  MPMCQueueBase(size_t element_size, size_t capacity);
  /**
   * @brief 析构字节队列内核 / Destroy the byte-queue core
   */
  ~MPMCQueueBase();

  /**
   * @brief 按字节入队一个 payload / Enqueue one payload by bytes
   * @param value 指向待入队 payload 的指针 / Pointer to the payload to enqueue
   * @return 成功返回 `ErrorCode::OK`；队列满返回 `ErrorCode::FULL`；空指针返回
   *         `ErrorCode::PTR_NULL`
   *         Returns `ErrorCode::OK` on success; `ErrorCode::FULL` when the
   *         queue is full; `ErrorCode::PTR_NULL` when `value` is null
   */
  ErrorCode PushBytes(const void* value);
  /**
   * @brief 按字节出队一个 payload；传空指针时只丢弃队头元素
   *        / Dequeue one payload by bytes; pass null to discard the front item only
   * @param value 用于接收 payload 的缓冲区；传 `nullptr` 时仅丢弃
   *        / Buffer that receives the payload; pass `nullptr` to discard only
   * @return 成功返回 `ErrorCode::OK`；队列空返回 `ErrorCode::EMPTY`
   *         Returns `ErrorCode::OK` on success; returns `ErrorCode::EMPTY` when
   *         the queue is empty
   */
  ErrorCode PopBytes(void* value = nullptr);

  /**
   * @brief 获取队列最大容量 / Get the maximum queue capacity
   * @return 队列容量 / Queue capacity
   */
  [[nodiscard]] size_t MaxSize() const { return capacity_; }
  /**
   * @brief 获取并发快照下的当前元素数 / Get the current approximate element count
   *
   * @note 该值是近似快照：
   *       - 在并发入队/出队时，该值可能已经过期；
   *       - 这里按 `SequenceType` 的模差值估算已用槽数，再钳到 `[0, MaxSize()]`，
   *         因此即使极端长寿命下序号回绕后，它仍然只是近似值，而不是精确值。
   *       This value is an approximate snapshot:
   *       - it may already be stale while producers/consumers are progressing;
   *       - it is computed from modular `SequenceType` differences and then
   *         clamped to `[0, MaxSize()]`, so it remains approximate rather than
   *         exact even after very long-lived sequence wraparound.
   */
  [[nodiscard]] size_t Size() const;
  /**
   * @brief 获取剩余空槽数 / Get the current free-slot count
   * @return 当前空槽个数 / Current number of free slots
   */
  [[nodiscard]] size_t EmptySize() const { return capacity_ - Size(); }
  /**
   * @brief 获取单个 payload 的字节数 / Get the byte size of one payload
   * @return 单个 payload 的字节数 / Byte size of one payload
   */
  [[nodiscard]] size_t ElementSize() const { return element_size_; }

 private:
  /// @brief 每个逻辑槽对应的序号单元 / Sequence cell for one logical slot.
  struct alignas(LibXR::CONCURRENCY_ALIGNMENT) SequenceCell
  {
    std::atomic<SequenceType>
        value;  ///< 当前槽的逻辑序号 / Current logical sequence of the slot.
  };

  /// @brief 获取指定槽位 payload 起始地址 / Get the payload base address of one slot.
  [[nodiscard]] void* PayloadPtr(size_t index);
  /// @brief 获取指定槽位 payload 起始地址（只读）。
  /// Get the payload base address of one slot (const).
  [[nodiscard]] const void* PayloadPtr(size_t index) const;
  /// @brief 安全地向上对齐字节数 / Safely align one byte count upward.
  [[nodiscard]] static size_t AlignUpChecked(size_t value, size_t align);
  /// @brief 安全地计算乘积 / Safely multiply two size values.
  [[nodiscard]] static size_t MultiplyChecked(size_t lhs, size_t rhs);
  /**
   * @brief payload 缓冲区整体分配对齐 / Allocation alignment used for the whole payload
   * buffer
   */
  static constexpr size_t PAYLOAD_ALLOC_ALIGN =
      std::max(alignof(size_t), alignof(std::max_align_t));

  /// @brief 禁止拷贝构造 / Non-copyable.
  MPMCQueueBase(const MPMCQueueBase&);
  /// @brief 禁止拷贝赋值 / Non-copy-assignable.
  MPMCQueueBase& operator=(const MPMCQueueBase&);
  /// @brief 禁止移动构造 / Non-movable.
  MPMCQueueBase(MPMCQueueBase&&);
  /// @brief 禁止移动赋值 / Non-move-assignable.
  MPMCQueueBase& operator=(MPMCQueueBase&&);

  const size_t element_size_;    ///< 单个 payload 的字节数 / Byte size of one payload.
  const size_t capacity_;        ///< 队列容量 / Queue capacity.
  const size_t payload_stride_;  ///< 相邻 payload 槽位之间的步长 / Byte stride between
                                 ///< adjacent payload slots.
  SequenceCell* sequences_;      ///< 槽序号数组 / Array of per-slot sequence cells.
  std::byte* payloads_;          ///< payload 字节缓冲区 / Byte buffer storing payloads.

  alignas(LibXR::CONCURRENCY_ALIGNMENT) std::atomic<
      SequenceType> head_;  ///< 下一个待出队的逻辑位置 / Next logical dequeue position.
  alignas(LibXR::CONCURRENCY_ALIGNMENT) std::atomic<
      SequenceType> tail_;  ///< 下一个待入队的逻辑位置 / Next logical enqueue position.
};
}  // namespace LibXR
