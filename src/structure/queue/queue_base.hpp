#pragma once

#include <cstddef>
#include <cstdint>

#include "libxr_def.hpp"

namespace LibXR
{
/**
 * @class QueueBase
 * @brief 提供固定大小循环缓冲区的字节 FIFO 队列。
 *        Byte FIFO queue providing a fixed-size circular buffer.
 *
 * 该类只按固定元素大小搬运字节，不管理强类型对象的构造、析构或所有权。
 * This class moves fixed-size byte elements only and does not manage typed
 * object construction, destruction, or ownership.
 */
class QueueBase
{
 public:
  /**
   * @brief 使用外部缓冲区构造队列 / Construct the queue with an external buffer.
   * @param element_size 队列中每个元素的字节数 / Byte size of each queue element.
   * @param length 队列最大容量 / Maximum queue capacity.
   * @param buffer 外部缓冲区指针 / Pointer to the external buffer.
   */
  QueueBase(uint16_t element_size, size_t length, uint8_t* buffer);

  /**
   * @brief 由队列内部申请缓冲区并构造队列。
   *        Construct the queue with an internally allocated buffer.
   * @param element_size 队列中每个元素的字节数 / Byte size of each queue element.
   * @param length 队列最大容量 / Maximum queue capacity.
   *
   * @note 包含动态内存分配 / Contains dynamic memory allocation.
   */
  QueueBase(uint16_t element_size, size_t length);

  /**
   * @brief 析构队列 / Destroy the queue.
   */
  ~QueueBase();

  /**
   * @brief 访问指定物理槽位的原始元素地址。
   *        Access the raw element address of one physical slot.
   * @param index 目标物理槽位下标 / Target physical slot index.
   * @return 指向该槽位元素起始地址的指针。
   * Pointer to the element base address of the slot.
   */
  [[nodiscard]] void* operator[](uint32_t index);

  /**
   * @brief 按字节入队一个元素 / Enqueue one element by bytes.
   * @param data 指向待入队元素的指针 / Pointer to the element to enqueue.
   * @return 成功返回 `ErrorCode::OK`，队列满返回 `ErrorCode::FULL`。
   *         Returns `ErrorCode::OK` on success and `ErrorCode::FULL` when the queue is
   * full.
   */
  ErrorCode PushBytes(const void* data);

  /**
   * @brief 按字节查看队头元素但不出队。
   *        Peek the front element by bytes without dequeuing it.
   * @param data 接收队头元素的缓冲区 / Buffer receiving the front element.
   * @return 成功返回 `ErrorCode::OK`，队列空返回 `ErrorCode::EMPTY`。
   *         Returns `ErrorCode::OK` on success and `ErrorCode::EMPTY` when the queue is
   * empty.
   */
  ErrorCode PeekBytes(void* data);

  /**
   * @brief 按字节出队一个元素；传空指针时仅丢弃队头。
   *        Dequeue one element by bytes; pass null to discard the front item only.
   * @param data 接收出队元素的缓冲区；传 `nullptr` 时仅丢弃。
   *             Buffer receiving the dequeued element; pass `nullptr` to discard only.
   * @return 成功返回 `ErrorCode::OK`，队列空返回 `ErrorCode::EMPTY`。
   *         Returns `ErrorCode::OK` on success and `ErrorCode::EMPTY` when the queue is
   * empty.
   */
  ErrorCode PopBytes(void* data = nullptr);

  /**
   * @brief 获取当前最后一个已入队元素的物理槽位下标。
   *        Get the physical slot index of the current last queued element.
   * @return 队列非空时返回最后一个元素的物理槽位下标，否则返回 `-1`。
   *         Returns the physical slot index of the last element, or `-1` when the queue
   * is empty.
   */
  int GetLastElementIndex() const;

  /**
   * @brief 获取当前第一个已入队元素的物理槽位下标。
   *        Get the physical slot index of the current first queued element.
   * @return 队列非空时返回第一个元素的物理槽位下标，否则返回 `-1`。
   *         Returns the physical slot index of the first element, or `-1` when the queue
   * is empty.
   */
  int GetFirstElementIndex() const;

  /**
   * @brief 按字节批量入队多个元素 / Enqueue multiple elements by bytes.
   * @param data 指向元素数组的缓冲区 / Buffer pointing to the element array.
   * @param size 要入队的元素个数 / Number of elements to enqueue.
   * @return 成功返回 `ErrorCode::OK`，空间不足返回 `ErrorCode::FULL`。
   *         Returns `ErrorCode::OK` on success and `ErrorCode::FULL` when free space is
   * insufficient.
   */
  ErrorCode PushBatchBytes(const void* data, size_t size);

  /**
   * @brief 按字节批量出队多个元素 / Dequeue multiple elements by bytes.
   * @param data 接收出队元素的缓冲区；传 `nullptr` 时仅丢弃。
   *             Buffer receiving dequeued elements; pass `nullptr` to discard only.
   * @param size 要出队的元素个数 / Number of elements to dequeue.
   * @return 成功返回 `ErrorCode::OK`，元素不足返回 `ErrorCode::EMPTY`。
   *         Returns `ErrorCode::OK` on success and `ErrorCode::EMPTY` when stored
   * elements are insufficient.
   */
  ErrorCode PopBatchBytes(void* data, size_t size);

  /**
   * @brief 按字节批量查看多个元素但不出队。
   *        Peek multiple elements by bytes without dequeuing them.
   * @param data 接收查看结果的缓冲区 / Buffer receiving the peeked elements.
   * @param size 要查看的元素个数 / Number of elements to peek.
   * @return 成功返回 `ErrorCode::OK`，元素不足返回 `ErrorCode::EMPTY`。
   *         Returns `ErrorCode::OK` on success and `ErrorCode::EMPTY` when stored
   * elements are insufficient.
   */
  ErrorCode PeekBatchBytes(void* data, size_t size);

  /**
   * @brief 清空当前状态后，用一个新元素覆盖队列内容。
   *        Reset the queue state and overwrite it with one new element.
   * @param data 指向新元素的指针 / Pointer to the new element.
   * @return 成功返回 `ErrorCode::OK` / Returns `ErrorCode::OK` on success.
   */
  ErrorCode OverwriteBytes(const void* data);

  /**
   * @brief 重置队列状态 / Reset the queue state.
   */
  void Reset();

  /**
   * @brief 获取当前已存储元素个数 / Get the current stored element count.
   * @return 当前已存储元素个数 / Current number of stored elements.
   */
  [[nodiscard]] size_t Size() const;

  /**
   * @brief 获取当前剩余空槽数 / Get the current free-slot count.
   * @return 当前剩余空槽数 / Current number of free slots.
   */
  [[nodiscard]] size_t EmptySize() const;

  /**
   * @brief 获取队列最大容量 / Get the maximum queue capacity.
   * @return 队列最大容量 / Maximum queue capacity.
   */
  [[nodiscard]] size_t MaxSize() const { return length_; }

 private:
  /// @brief 禁止拷贝构造 / Non-copyable.
  QueueBase(const QueueBase&);
  /// @brief 禁止拷贝赋值 / Non-copy-assignable.
  QueueBase& operator=(const QueueBase&);
  /// @brief 禁止同类型左值赋值重载 / Non-copy-assignable overload for non-const lvalues.
  QueueBase& operator=(QueueBase&);
  /// @brief 禁止移动赋值（const rvalue 形式） / Non-move-assignable (const rvalue form).
  QueueBase& operator=(const QueueBase&&);
  /// @brief 禁止移动赋值 / Non-move-assignable.
  QueueBase& operator=(QueueBase&&);

 public:
  uint8_t* queue_array_;        ///< 队列数据缓冲区 / Queue data buffer.
  const uint16_t ELEMENT_SIZE;  ///< 单个元素的字节数 / Byte size of one element.
  size_t head_ = 0;  ///< 当前队头物理槽位下标 / Physical slot index of the current head.
  size_t tail_ = 0;  ///< 下一个待写入物理槽位下标 / Physical slot index of the next
                     ///< enqueue position.
  bool is_full_ = false;  ///< 当前队列是否已满 / Whether the queue is currently full.
  size_t length_;         ///< 队列最大容量 / Maximum queue capacity.
  bool own_buffer_ =
      false;  ///< 是否由当前队列拥有缓冲区 / Whether this queue owns the buffer.
};
}  // namespace LibXR
