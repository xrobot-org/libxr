#pragma once

#include <atomic>
#include <cstdint>
#include <type_traits>

namespace LibXR
{

/**
 * @class LatestSnapshot
 * @brief 只保留最新一份完整值的单生产者、单消费者邮箱 /
 *        Single-producer, single-consumer mailbox that keeps only the latest complete
 *        value
 *
 * 内部有三个槽：生产者独占后槽，消费者独占前槽，中间槽由一个原子状态
 * 交给对方，承载最新一次完成的发布。连续的 Store() 可以覆盖消费者尚未
 * 取走的中间值，但不会覆盖消费者正在拷贝的值。适用于只关心最新数据的
 * 场合，例如在接收中断里保存最新的反馈帧，由控制线程读取。
 *
 * The three slots are a back slot owned by the producer, a front slot owned by the
 * consumer, and a middle slot handed over through one atomic state, which carries the
 * latest completed publication. Repeated Store() calls may overwrite a middle value the
 * consumer has not taken, but never the value the consumer is copying. It suits data of
 * which only the latest value matters, such as the latest feedback frame stored in a
 * receive interrupt and read by a control thread.
 *
 * @tparam T 可拷贝构造、可拷贝赋值的值类型 / Copy-constructible, copy-assignable value
 *           type
 * @warning 只能有一个生产者调用 Store()，一个串行的消费者调用
 *          LoadLatest()；生产者与消费者的调用可以在不同核、线程或中断中
 *          重叠。
 *          Exactly one producer may call Store(), and exactly one serialized consumer may
 *          call LoadLatest(); producer and consumer calls may overlap on different cores
 *          or in thread and interrupt contexts.
 */
template <typename T>
class LatestSnapshot
{
  static_assert(std::is_copy_constructible_v<T>,
                "LatestSnapshot requires a copy-constructible value type");
  static_assert(std::is_copy_assignable_v<T>,
                "LatestSnapshot requires a copy-assignable value type");

 public:
  /**
   * @brief 用同一个初始值构造三个槽 / Construct all three slots with the same initial
   *        value
   *
   * @param initial 初始值，LoadLatest() 在第一次 Store() 之前输出它 / Initial value,
   *                output by LoadLatest() before the first Store()
   */
  explicit LatestSnapshot(const T& initial) noexcept(
      std::is_nothrow_copy_constructible_v<T>)
      : slots_{initial, initial, initial}
  {
  }

  /**
   * @brief 发布一个完整的值 / Publish one complete value
   *
   * 先把值拷入生产者独占的后槽，再把该槽作为最新的中间槽交给消费者。
   * 只能由唯一的生产者调用。
   *
   * The value is copied into the producer-owned back slot before that slot is released
   * to the consumer as the newest middle slot. Only the single producer may call it.
   *
   * @param value 要发布的值 / Value to publish
   */
  void Store(const T& value) noexcept(std::is_nothrow_copy_assignable_v<T>)
  {
    slots_[back_] = value;

    const uint32_t previous =
        state_.exchange(Pack(back_, true), std::memory_order_acq_rel);
    back_ = Index(previous);
  }

  /**
   * @brief 把最新的完整值拷入 output / Copy the latest complete value into output
   *
   * 有新的发布时先取得它；没有时 output 得到消费者上一次取得的值。只能
   * 由唯一的消费者串行调用。
   *
   * A newer publication is acquired first; without one, output receives the value the
   * consumer acquired last. Only the single consumer may call it, one call at a time.
   *
   * @param output 接收值的对象 / Object receiving the value
   * @return 本次取得了新的发布时为 true，否则为 false / true when this call acquired a
   *         newer publication, otherwise false
   */
  bool LoadLatest(T& output) noexcept(std::is_nothrow_copy_assignable_v<T>)
  {
    bool updated = false;
    uint32_t observed = state_.load(std::memory_order_acquire);

    while (HasNew(observed))
    {
      const uint32_t desired = Pack(front_, false);
      if (state_.compare_exchange_weak(observed, desired, std::memory_order_acq_rel,
                                       std::memory_order_acquire))
      {
        front_ = Index(observed);
        updated = true;
        break;
      }
    }

    output = slots_[front_];
    return updated;
  }

  /// 禁止拷贝构造 / Non-copyable
  LatestSnapshot(const LatestSnapshot&) = delete;
  /// 禁止拷贝赋值 / Non-copy-assignable
  LatestSnapshot& operator=(const LatestSnapshot&) = delete;
  /// 禁止移动构造 / Non-movable
  LatestSnapshot(LatestSnapshot&&) = delete;
  /// 禁止移动赋值 / Non-move-assignable
  LatestSnapshot& operator=(LatestSnapshot&&) = delete;

 private:
  static constexpr uint32_t INDEX_MASK = 0x3U;
  static constexpr uint32_t HAS_NEW_BIT = 1U << 2U;

  static constexpr uint32_t Pack(uint32_t index, bool has_new)
  {
    return index | (has_new ? HAS_NEW_BIT : 0U);
  }

  static constexpr uint32_t Index(uint32_t state) { return state & INDEX_MASK; }

  static constexpr bool HasNew(uint32_t state) { return (state & HAS_NEW_BIT) != 0U; }

  T slots_[3];
  // 低两位是中间槽的下标，HAS_NEW_BIT 表示中间槽中有消费者尚未取走的发布。
  // The low two bits index the middle slot; HAS_NEW_BIT marks a publication the consumer
  // has not taken yet.
  std::atomic<uint32_t> state_{Pack(1U, false)};
  uint32_t front_ = 0U;
  uint32_t back_ = 2U;
};

}  // namespace LibXR
