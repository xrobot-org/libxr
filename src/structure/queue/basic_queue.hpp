#pragma once

#include <cstddef>
#include <cstdint>

#include "queue_base.hpp"
#include "queue_typed_base.hpp"

namespace LibXR
{
/**
 * @class Queue
 * @brief 基于 QueueBase 的泛型队列模板类。
 *        Generic queue template class based on QueueBase.
 *
 * 该类提供一个类型安全的队列，用于存储 `Data` 类型的元素。
 * 它支持标准的队列操作，如推入（Push）、弹出（Pop）、查看（Peek）以及批量操作。
 * This class provides a type-safe queue for storing elements of type `Data`.
 * It supports standard queue operations such as push, pop, peek, and batch operations.
 *
 * @tparam Data 队列存储的数据类型 / Type of data stored in the queue.
 */
template <typename Data>
class Queue final : public QueueTypedBase<Queue<Data>, Data>, public QueueBase
{
 public:
  using ValueType = Data;  ///< 队列元素类型 / Queue element type.
  /// @brief 重新公开强类型出队接口 / Re-expose the typed pop interface.
  using QueueTypedBase<Queue<Data>, Data>::Pop;
  /// @brief 重新公开强类型入队接口 / Re-expose the typed push interface.
  using QueueTypedBase<Queue<Data>, Data>::Push;

  /**
   * @brief 构造一个由内部缓冲区支撑的强类型队列。
   *        Construct one typed queue backed by an internal buffer.
   * @param length 队列最大容量 / Maximum queue capacity.
   *
   * @note 包含动态内存分配 / Contains dynamic memory allocation.
   */
  explicit Queue(size_t length) : QueueBase(sizeof(Data), length) {}

  /**
   * @brief 使用外部缓冲区构造强类型队列。
   *        Construct one typed queue with an external buffer.
   * @param length 队列最大容量 / Maximum queue capacity.
   * @param buffer 外部缓冲区指针 / Pointer to the external buffer.
   *
   * @note 调用方负责保证外部缓冲区至少能容纳 `length * sizeof(Data)` 字节。
   *       The caller must ensure that the external buffer can hold at least
   *       `length * sizeof(Data)` bytes.
   */
  Queue(size_t length, uint8_t* buffer) : QueueBase(sizeof(Data), length, buffer) {}

  /**
   * @brief 按逻辑索引访问队列中的元素 / Access an element in the queue by logical index.
   *
   * 该函数允许使用索引访问队列中的元素。
   * 如果索引为正，则表示从 `head_`（队列头部）开始的偏移量。
   * 如果索引为负，则表示从 `tail_`（队列尾部）开始的偏移量。
   * This function provides indexed access to elements in the queue.
   * If the index is positive, it is relative to `head_` (front of the queue).
   * If negative, it is relative to `tail_` (end of the queue).
   *
   * @param index 访问的逻辑索引值 / Logical index to access.
   * @return 索引对应的元素引用 / Reference to the element at the specified index.
   */
  Data& operator[](int32_t index)
  {
    if (index >= 0)
    {
      index = (head_ + index) % length_;
    }
    else
    {
      index = (tail_ + index + length_) % length_;
    }

    return *reinterpret_cast<Data*>(&queue_array_[index * ELEMENT_SIZE]);
  }

  /**
   * @brief 查看队头元素但不出队 / Peek the front element without removing it.
   * @param data 用于接收查看结果的引用 / Reference receiving the peeked element.
   * @return 成功返回 `ErrorCode::OK`，队列空返回 `ErrorCode::EMPTY`。
   *         Returns `ErrorCode::OK` on success and `ErrorCode::EMPTY` when the queue is
   * empty.
   */
  ErrorCode Peek(Data& data) { return QueueBase::PeekBytes(&data); }

  /**
   * @brief 批量推入多个元素 / Push multiple elements into the queue.
   * @param data 指向元素数组的指针 / Pointer to the array of elements to push.
   * @param size 要推入的元素个数 / Number of elements to push.
   * @return 成功返回 `ErrorCode::OK`，空间不足返回 `ErrorCode::FULL`。
   *         Returns `ErrorCode::OK` on success and `ErrorCode::FULL` when free space is
   * insufficient.
   */
  ErrorCode PushBatch(const Data* data, size_t size)
  {
    return QueueBase::PushBatchBytes(data, size);
  }

  /**
   * @brief 批量移除多个元素并复制到输出数组。
   *        Pop multiple elements and copy them into the output array.
   * @param data 指向输出数组的指针 / Pointer to the array receiving popped elements.
   * @param size 要移除的元素个数 / Number of elements to remove.
   * @return 成功返回 `ErrorCode::OK`，元素不足返回 `ErrorCode::EMPTY`。
   *         Returns `ErrorCode::OK` on success and `ErrorCode::EMPTY` when stored
   * elements are insufficient.
   */
  ErrorCode PopBatch(Data* data, size_t size)
  {
    return QueueBase::PopBatchBytes(data, size);
  }

  /**
   * @brief 批量查看多个元素但不移除 / Peek multiple elements without removing them.
   * @param data 指向输出数组的指针 / Pointer to the array receiving peeked elements.
   * @param size 要查看的元素个数 / Number of elements to retrieve.
   * @return 成功返回 `ErrorCode::OK`，元素不足返回 `ErrorCode::EMPTY`。
   *         Returns `ErrorCode::OK` on success and `ErrorCode::EMPTY` when stored
   * elements are insufficient.
   */
  ErrorCode PeekBatch(Data* data, size_t size)
  {
    return QueueBase::PeekBatchBytes(data, size);
  }

  /**
   * @brief 清空当前状态后，用一个新元素覆盖队列内容。
   *        Reset the queue state and overwrite it with one new element.
   * @param data 用于覆盖队列的新元素 / New element used to overwrite the queue.
   * @return 成功返回 `ErrorCode::OK` / Returns `ErrorCode::OK` on success.
   */
  ErrorCode Overwrite(const Data& data) { return QueueBase::OverwriteBytes(&data); }

  /**
   * @brief 获取队列最大容量 / Get the maximum queue capacity.
   * @return 队列最大容量 / Maximum queue capacity.
   */
  [[nodiscard]] size_t MaxSize() const { return QueueBase::MaxSize(); }
};
}  // namespace LibXR
