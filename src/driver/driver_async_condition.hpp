#pragma once

#include <atomic>
#include <cstdint>

#include "libxr_def.hpp"

namespace LibXR
{

/**
 * @brief 驱动专用的非阻塞等待：在毫秒节拍上检查一个条件，成立或超时时通知驱动
 *        Driver-only non-blocking wait: a condition checked on the millisecond tick, with
 *        a notification when it holds or times out
 *
 * 代替驱动中"一边查标志一边看超时"的轮询。Wait() 立即返回；min_ms 之前不检查，之后每个
 * 节拍调用一次 check，成立时 done(OK)，过了 max_ms 仍不成立时 done(TIMEOUT)。节拍的
 * 间隔固定为 T 毫秒；Wait() 之后可能紧接着来一个节拍，所以检查不早于 min_ms、不晚于
 * min_ms 向上取整到 T 的整数倍再加 T 开始，超时同样不早于 max_ms、不晚于 max_ms 向上
 * 取整到 T 的整数倍再加 T。min_ms 和 max_ms 须小于 2^29（约 6 天）。
 *
 * check 和 done 在节拍上下文执行，可能是高于 RTOS 系统调用上限的中断，所以两者都必须是
 * 有界的极短操作：几条寄存器读写，不阻塞、不循环等待、不调用 RTOS 接口。check 只读寄存器
 * 和 arg 中不变的内容。done 可以做一步短的硬件操作并 Wait 下一个条件，把长流程拆成几段；
 * 链的最后一步唤醒驱动自己的上下文。驱动须保证链独占它操作的硬件和数据。
 *
 * 节拍由使用它的平台的时基驱动提供（见 OnTick()）。对象由驱动作为成员持有，
 * 构造时挂入全局链表，运行期不分配内存。
 *
 * Replaces the driver loops that poll a flag while watching a timeout. Wait() returns at
 * once; check is not called before min_ms, then once per tick: done(OK) when it holds,
 * done(TIMEOUT) when it still does not after max_ms. Ticks come every T milliseconds, and
 * one may come right after Wait(), so checking starts no earlier than min_ms and no later
 * than min_ms rounded up to a multiple of T plus T; the timeout likewise comes no earlier
 * than max_ms and no later than max_ms rounded up to a multiple of T plus T. min_ms and
 * max_ms are below 2^29 (about six days).
 *
 * check and done run in the tick context, which may be an interrupt above the RTOS system
 * call ceiling, so both must be short bounded operations: a few register accesses, no
 * blocking, no wait loops, no RTOS calls. check only reads registers and what arg holds
 * unchanged. done may take one short hardware step and Wait for the next condition,
 * splitting a long procedure into stages; the last stage wakes the driver's own context.
 * The driver must keep the hardware and data of the chain to the chain.
 *
 * The timebase driver of a platform that uses it provides the tick (see OnTick()). The
 * driver holds the object as a member; the constructor links it into a global list and
 * nothing is allocated at run time.
 */
class DriverAsyncCondition
{
 public:
  /// 条件：成立时返回 true / Condition: true when it holds
  using Check = bool (*)(void* arg);
  /// 结果：ErrorCode::OK 或 ErrorCode::TIMEOUT / Result: ErrorCode::OK or
  /// ErrorCode::TIMEOUT
  using Done = void (*)(void* arg, ErrorCode result);

  /**
   * @param check 条件 / Condition
   * @param done 结果回调 / Result callback
   * @param arg 两个回调的参数 / Argument of both callbacks
   */
  DriverAsyncCondition(Check check, Done done, void* arg);

  /// 只能在没有 OnTick() 并发运行时销毁（例如主机测试）/ Destroy only while no OnTick()
  /// can run (host tests, for example)
  ~DriverAsyncCondition();

  DriverAsyncCondition(const DriverAsyncCondition&) = delete;
  DriverAsyncCondition& operator=(const DriverAsyncCondition&) = delete;

  /**
   * @brief 开始等待；立即返回 / Start waiting; returns at once
   * @param min_ms 开始检查前至少等待的毫秒数 / Milliseconds to wait at least before
   * checking
   * @param max_ms 超时的毫秒数，不小于 min_ms / Timeout in milliseconds, at least min_ms
   * @return OK；上一次等待未结束时 BUSY；max_ms 小于 min_ms 时 ARG_ERR / OK; BUSY while
   * the previous wait runs; ARG_ERR when max_ms is below min_ms
   */
  ErrorCode Wait(uint32_t min_ms, uint32_t max_ms);

  /**
   * @brief 节拍入口：时基驱动每过 elapsed_ms 毫秒调用一次 / Tick entry: the timebase
   * driver calls it every elapsed_ms milliseconds
   *
   * 每次调用的 elapsed_ms 相同 / elapsed_ms is the same on every call.
   * 不可重入；每个平台至多一个节拍源，没有使用者的平台不提供节拍。 / Not reentrant;
   * every platform has at most one tick source, and a platform without a user provides
   * none.
   */
  static void OnTick(uint32_t elapsed_ms = 1);

 private:
  // State word: phase [31:30], milliseconds counted since Wait() [29:0].
  static constexpr uint32_t PHASE_SHIFT = 30U;
  static constexpr uint32_t PHASE_MASK = 3U << PHASE_SHIFT;
  static constexpr uint32_t COUNT_MASK = ~PHASE_MASK;
  static constexpr uint32_t IDLE = 0U;
  static constexpr uint32_t SETUP = 1U << PHASE_SHIFT;  ///< Wait() is writing the limits.
  static constexpr uint32_t WAITING = 2U << PHASE_SHIFT;

  void Tick(uint32_t elapsed_ms);

  Check check_;
  Done done_;
  void* arg_;
  // Written by Wait() in SETUP, read by the tick in WAITING (published by the state
  // word).
  uint32_t min_ms_ = 0;
  uint32_t max_ms_ = 0;
  std::atomic<uint32_t> state_{IDLE};
  DriverAsyncCondition* next_ = nullptr;  ///< Fixed once linked.

  static inline std::atomic<DriverAsyncCondition*> head_{nullptr};
};

}  // namespace LibXR
