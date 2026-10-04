#pragma once

#include <cstddef>
#include <type_traits>

#include "mpmc_queue_base.hpp"
#include "queue_typed_base.hpp"

namespace LibXR
{
/**
 * @class MPMCQueue
 * @brief 带固定 payload 类型的有界 MPMC 队列。
 *        Bounded MPMC queue with a fixed payload type.
 *
 * 队列内部只把 payload 当成原始字节块搬运，不会在内部形成 `Payload*` 指针，
 * 因此 `Payload` 的对齐要求不会额外约束队列内部字节缓冲区的语义边界。
 *
 * The queue moves payloads only as raw byte blocks and never forms internal
 * `Payload*` pointers, so `Payload` alignment does not add an extra semantic
 * constraint on the internal byte-buffer contract.
 *
 * @tparam Payload 队列存储的数据类型 / Queue element type.
 */
template <typename Payload>
class MPMCQueue final : public QueueTypedBase<MPMCQueue<Payload>, Payload>,
                        public MPMCQueueBase
{
  /// @brief 仅接受平凡可拷贝 payload / Accepts only trivially copyable payloads.
  static_assert(std::is_trivially_copyable_v<Payload>,
                "MPMCQueue requires trivially copyable payloads");
  /// @brief 仅接受平凡可析构 payload / Accepts only trivially destructible payloads.
  static_assert(std::is_trivially_destructible_v<Payload>,
                "MPMCQueue requires trivially destructible payloads");

 public:
  using ValueType = Payload;  ///< 队列元素类型 / Queue element type.
  /// @brief 重新公开强类型出队接口 / Re-expose the typed pop interface.
  using QueueTypedBase<MPMCQueue<Payload>, Payload>::Pop;
  /// @brief 重新公开强类型入队接口 / Re-expose the typed push interface.
  using QueueTypedBase<MPMCQueue<Payload>, Payload>::Push;

  /**
   * @brief 构造一个 MPMC 队列 / Construct one MPMC queue.
   * @param capacity 队列容量 / Queue capacity.
   *
   * @note 包含动态内存分配 / Contains dynamic memory allocation.
   */
  explicit MPMCQueue(size_t capacity) : MPMCQueueBase(sizeof(Payload), capacity) {}
};
}  // namespace LibXR
