#pragma once

#include <cstring>
#include <type_traits>

#include "libxr_def.hpp"
#include "mutex.hpp"

namespace LibXR
{
/**
 * @class Stack
 * @brief 线程安全的栈数据结构 / Thread-safe stack data structure
 *
 * 该类实现了一个基于数组的线程安全栈，支持基本的 `Push`、`Pop`、`Peek`
 * 等操作，并使用互斥锁 (`Mutex`) 保护数据安全。
 * This class implements a thread-safe stack based on an array, supporting basic
 * operations such as `Push`, `Pop`, and `Peek`, with mutex (`Mutex`) protection to ensure
 * data safety.
 *
 * @tparam Data 栈中存储的数据类型 / The type of data stored in the stack
 */
template <typename Data>
class Stack
{
 private:
  Data* stack_;         ///< 栈存储数组 / Stack storage array
  uint32_t top_ = 0;    ///< 当前栈顶索引 / Current top index of the stack
  uint32_t depth_;      ///< 栈的最大容量 / Maximum capacity of the stack
  LibXR::Mutex mutex_;  ///< 互斥锁，确保线程安全 / Mutex to ensure thread safety

 public:
  /**
   * @brief 栈的构造函数 / Stack constructor
   * @param depth 栈的最大容量 / Maximum capacity of the stack
   *
   * @note 包含动态内存分配。
   *       Contains dynamic memory allocation.
   */
  Stack(uint32_t depth) : stack_(new Data[depth]), depth_(depth) {}

  /**
   * @brief 析构栈并释放存储数组 / Destroys the stack and frees the storage array
   */
  ~Stack() { delete[] stack_; }

  /// @brief 禁止复制构造 / Not copy-constructible
  Stack(const Stack&) = delete;
  /// @brief 禁止复制赋值 / Not copy-assignable
  Stack& operator=(const Stack&) = delete;

  /**
   * @brief 获取指定索引的元素 / Retrieves the element at a specified index
   * @param index 元素索引，支持负索引（从栈顶向下索引） / Element index, supports
   * negative indexing (relative to the top)
   * @return 该索引位置的元素 / The element at the given index
   */
  Data& operator[](int32_t index)
  {
    if (index >= 0)
    {
      ASSERT(static_cast<uint32_t>(index) < depth_);
      return stack_[index];
    }
    else
    {
      ASSERT(static_cast<int32_t>(top_) + index >= 0);
      return stack_[top_ + index];
    }
  }

  /**
   * @brief 获取栈中当前元素数量 / Returns the number of elements currently in the stack
   */
  [[nodiscard]] uint32_t Size() const { return top_; }

  /**
   * @brief 获取栈的剩余可用空间 / Returns the remaining available space in the stack
   */
  [[nodiscard]] uint32_t EmptySize() const { return depth_ - top_; }

  /**
   * @brief 向栈中推入数据 / Pushes data onto the stack
   * @param data 要推入的元素 / The element to be pushed
   * @return 操作结果，成功返回 `ErrorCode::OK`，栈满返回 `ErrorCode::FULL` /
   *         Operation result: returns `ErrorCode::OK` on success, `ErrorCode::FULL` if
   * the stack is full
   */
  ErrorCode Push(const Data& data)
  {
    mutex_.Lock();

    if (top_ >= depth_)
    {
      mutex_.Unlock();
      return ErrorCode::FULL;
    }
    stack_[top_++] = data;
    mutex_.Unlock();
    return ErrorCode::OK;
  }

  /**
   * @brief 从栈中弹出数据 / Pops data from the stack
   * @param data 用于存储弹出的数据 / Variable to store the popped data
   * @return 操作结果，成功返回 `ErrorCode::OK`，栈为空返回 `ErrorCode::EMPTY` /
   *         Operation result: returns `ErrorCode::OK` on success, `ErrorCode::EMPTY` if
   * the stack is empty
   */
  ErrorCode Pop(Data& data)
  {
    mutex_.Lock();

    if (top_ == 0)
    {
      mutex_.Unlock();
      return ErrorCode::EMPTY;
    }
    data = stack_[--top_];
    mutex_.Unlock();
    return ErrorCode::OK;
  }

  /**
   * @brief 从栈中弹出数据（不返回数据） / Pops data from the stack (without returning
   * data)
   * @return 操作结果，成功返回 `ErrorCode::OK`，栈为空返回 `ErrorCode::EMPTY` /
   *         Operation result: returns `ErrorCode::OK` on success, `ErrorCode::EMPTY` if
   * the stack is empty
   */
  ErrorCode Pop()
  {
    mutex_.Lock();

    if (top_ == 0)
    {
      mutex_.Unlock();
      return ErrorCode::EMPTY;
    }
    --top_;
    mutex_.Unlock();
    return ErrorCode::OK;
  }

  /**
   * @brief 获取栈顶数据但不弹出 / Retrieves the top data of the stack without popping
   * @param data 用于存储栈顶数据 / Variable to store the retrieved data
   * @return 操作结果，成功返回 `ErrorCode::OK`，栈为空返回 `ErrorCode::EMPTY` /
   *         Operation result: returns `ErrorCode::OK` on success, `ErrorCode::EMPTY` if
   * the stack is empty
   */
  ErrorCode Peek(Data& data)
  {
    mutex_.Lock();

    if (top_ == 0)
    {
      mutex_.Unlock();

      return ErrorCode::EMPTY;
    }
    data = stack_[top_ - 1];
    mutex_.Unlock();

    return ErrorCode::OK;
  }

  /**
   * @brief 在指定位置插入数据 / Inserts data at a specified position
   * @param data 要插入的数据 / The data to be inserted
   * @param index 插入位置索引 / Index at which the data is inserted
   * @return 操作结果，成功返回 `ErrorCode::OK`，栈满返回
   * `ErrorCode::FULL`，索引超出范围返回 `ErrorCode::OUT_OF_RANGE` / Operation result:
   * returns `ErrorCode::OK` on success, `ErrorCode::FULL` if the stack is full,
   * `ErrorCode::OUT_OF_RANGE` if the index is out of range
   *
   * @note 插入位置之后的元素按字节搬移，因此 `Data` 必须可平凡复制，否则编译失败。
   *       The elements after the position are moved byte by byte, so `Data` must be
   *       trivially copyable; otherwise the call does not compile.
   */
  ErrorCode Insert(const Data& data, uint32_t index)
  {
    static_assert(std::is_trivially_copyable_v<Data>,
                  "Stack::Insert moves elements with memmove and requires a trivially "
                  "copyable element type");

    mutex_.Lock();
    if (top_ >= depth_)
    {
      mutex_.Unlock();
      return ErrorCode::FULL;
    }

    if (index > top_)
    {
      mutex_.Unlock();
      return ErrorCode::OUT_OF_RANGE;
    }

    memmove(&stack_[index + 1], &stack_[index], (top_ - index) * sizeof(Data));

    stack_[index] = data;
    top_++;

    mutex_.Unlock();

    return ErrorCode::OK;
  }

  /**
   * @brief 删除指定位置的数据 / Deletes data at a specified position
   * @param index 要删除的索引位置 / Index of the data to be deleted
   * @return 操作结果，成功返回 `ErrorCode::OK`，索引超出范围返回
   * `ErrorCode::OUT_OF_RANGE` / Operation result: returns `ErrorCode::OK` on success,
   * `ErrorCode::OUT_OF_RANGE` if the index is out of range
   */
  ErrorCode Delete(uint32_t index)
  {
    mutex_.Lock();
    if (index >= top_)
    {
      mutex_.Unlock();
      return ErrorCode::OUT_OF_RANGE;
    }

    for (uint32_t i = index; i < top_ - 1; i++)
    {
      stack_[i] = stack_[i + 1];
    }
    top_--;
    mutex_.Unlock();
    return ErrorCode::OK;
  }

  /**
   * @brief 重置栈 / Resets the stack
   *
   * 该方法清空栈，并将 `top_` 归零。
   * This method clears the stack and resets `top_` to zero.
   */
  void Reset()
  {
    mutex_.Lock();
    top_ = 0;
    mutex_.Unlock();
  }
};
}  // namespace LibXR
