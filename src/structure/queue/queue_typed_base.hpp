#pragma once

#include "libxr_def.hpp"

namespace LibXR
{
/**
 * @class QueueTypedBase
 * @brief 强类型队列的公共薄包装 / Common thin wrapper for typed queues.
 *
 * 该基类只把固定类型 `Data` 的 `Push` / `Pop` 映射到派生类自己的字节队列
 * `PushBytes` / `PopBytes` 接口。它没有数据成员、虚函数或 RTTI 依赖。
 *
 * This base maps fixed-type `Data` `Push` / `Pop` operations to the derived
 * byte queue's `PushBytes` / `PopBytes` interface. It has no data members,
 * virtual functions, or RTTI dependency.
 *
 * @tparam Derived 最终强类型队列 / Final typed queue.
 * @tparam Data 队列元素类型 / Queue element type.
 */
template <typename Derived, typename Data>
class QueueTypedBase
{
 public:
  using ValueType = Data;  ///< 队列元素类型 / Queue element type.

  /**
   * @brief 推入一个强类型元素 / Push one typed element.
   * @param item 待入队元素 / Element to enqueue.
   * @return 底层字节队列返回的操作结果 / Operation result returned by the byte queue.
   */
  ErrorCode Push(const Data& item)
  {
    return static_cast<Derived*>(this)->PushBytes(&item);
  }

  /**
   * @brief 弹出一个强类型元素 / Pop one typed element.
   * @param item 用于接收出队元素的引用 / Reference receiving the dequeued element.
   * @return 底层字节队列返回的操作结果 / Operation result returned by the byte queue.
   */
  ErrorCode Pop(Data& item) { return static_cast<Derived*>(this)->PopBytes(&item); }

  /**
   * @brief 丢弃一个队头元素 / Discard one front element.
   * @return 底层字节队列返回的操作结果 / Operation result returned by the byte queue.
   */
  ErrorCode Pop() { return static_cast<Derived*>(this)->PopBytes(nullptr); }
};
}  // namespace LibXR
