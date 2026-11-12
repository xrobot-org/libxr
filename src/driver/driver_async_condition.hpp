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
 * 代替驱动中"一边查标志一边看超时"的轮询。Start() 立即返回；min_ms 之前不检查，之后每个
 * 节拍调用一次 check，成立时 done(OK)，过了 max_ms 仍不成立时 done(TIMEOUT)。节拍的
 * 间隔固定为 T 毫秒；Start() 之后可能紧接着来一个节拍，所以检查不早于 min_ms、不晚于
 * min_ms 向上取整到 T 的整数倍再加 T 开始，超时同样不早于 max_ms、不晚于 max_ms 向上
 * 取整到 T 的整数倍再加 T。
 *
 * check 和 done 在节拍上下文执行，可能是高于 RTOS 系统调用上限的中断，所以两者都必须是
 * 有界的极短操作：几条寄存器读写，不阻塞、不循环等待、不调用 RTOS 接口。check 只读寄存器
 * 和 arg 中不变的内容。done 可以做一步短的硬件操作，再对下一个条件调用 Start()，
 * 把长流程拆成几段；链的最后一步唤醒驱动自己的上下文。驱动须保证链独占它操作的硬件和
 * 数据。
 *
 * 节拍由使用它的平台的时基驱动提供（见 OnTick()）。对象由驱动作为成员持有，
 * 构造时挂入全局链表，运行期不分配内存。
 *
 * Replaces the driver loops that poll a flag while watching a timeout. Start() returns at
 * once; check is not called before min_ms, then once per tick: done(OK) when it holds,
 * done(TIMEOUT) when it still does not after max_ms. Ticks come every T milliseconds, and
 * one may come right after Start(), so checking starts no earlier than min_ms and no
 * later than min_ms rounded up to a multiple of T plus T; the timeout likewise comes no
 * earlier than max_ms and no later than max_ms rounded up to a multiple of T plus T.
 *
 * check and done run in the tick context, which may be an interrupt above the RTOS system
 * call ceiling, so both must be short bounded operations: a few register accesses, no
 * blocking, no wait loops, no RTOS calls. check only reads registers and what arg holds
 * unchanged. done may take one short hardware step and Start() the next condition,
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
  /// 条件函数：成立时返回 true / Condition function: returns true when it holds.
  using CheckFun = bool (*)(void* arg);
  /// 结果函数：result 为 OK 或 TIMEOUT / Result function: result is OK or TIMEOUT.
  using DoneFun = void (*)(void* arg, ErrorCode result);

  /// Start() 接受的最大毫秒数 / Largest millisecond value Start() accepts.
  static constexpr uint32_t MAX_WAIT_MS = (1U << 29U) - 1U;

  /**
   * @brief 绑定条件函数和结果函数，并挂入全局链表
   *        Binds the condition and result functions and links into the global list
   * @param check 条件函数 / Condition function
   * @param done 结果函数 / Result function
   * @param arg 传给两个函数的参数 / Argument passed to both functions
   */
  DriverAsyncCondition(CheckFun check, DoneFun done, void* arg);

  /**
   * @brief 从全局链表摘下；只供主机测试
   *        Unlinks from the global list; for host tests only
   *
   * 驱动持有的对象不销毁。OnTick() 不加锁遍历链表，所以析构时不能有 OnTick() 在运行。
   * Objects held by drivers are never destroyed. OnTick() walks the list without a lock,
   * so no OnTick() may run during destruction.
   */
  ~DriverAsyncCondition();

  DriverAsyncCondition(const DriverAsyncCondition&) = delete;
  DriverAsyncCondition& operator=(const DriverAsyncCondition&) = delete;

  /**
   * @brief 开始一次等待；立即返回 / Starts one wait; returns at once
   * @param min_ms 开始检查前至少经过的毫秒数 / Milliseconds that pass at least before
   *               the first check
   * @param max_ms 超时的毫秒数，不小于 min_ms，不大于 MAX_WAIT_MS / Timeout in
   *               milliseconds, at least min_ms and at most MAX_WAIT_MS
   * @return OK；上一次等待未结束时 BUSY；参数超出上述范围时 ARG_ERR / OK; BUSY while
   *         the previous wait runs; ARG_ERR when the arguments are outside these ranges
   */
  ErrorCode Start(uint32_t min_ms, uint32_t max_ms);

  /**
   * @brief 节拍入口：时基驱动每过 elapsed_ms 毫秒调用一次
   *        Tick entry: the timebase driver calls it every elapsed_ms milliseconds
   * @param elapsed_ms 节拍间隔的毫秒数，每次调用都相同 / Tick interval in
   *                   milliseconds, the same on every call
   *
   * 不可重入；每个平台至多一个节拍源，没有使用者的平台不提供节拍。
   * Not reentrant; every platform has at most one tick source, and a platform without a
   * user provides none.
   */
  static void OnTick(uint32_t elapsed_ms = 1);

 private:
  /// 一次等待所处的阶段 / Phase of one wait.
  enum class Phase : uint32_t
  {
    IDLE = 0U,     ///< 可以开始新的等待 / Available for a new wait.
    SETUP = 1U,    ///< Start() 正在写上下限 / Start() is writing the limits.
    WAITING = 2U,  ///< 节拍正在计时和检查 / The tick is counting and checking.
  };

  /// 低两位存阶段，其余位存已计的毫秒数 / Low two bits: phase; rest: counted ms.
  static constexpr uint32_t PHASE_BITS = 2U;
  static constexpr uint32_t PHASE_MASK = (1U << PHASE_BITS) - 1U;
  /// 状态字可表示的最大毫秒数，计数到此为止 / Largest encodable count; counting
  /// saturates here.
  static constexpr uint32_t MAX_COUNT = UINT32_MAX >> PHASE_BITS;

  static_assert(static_cast<uint32_t>(Phase::WAITING) <= PHASE_MASK);
  // 超时判断用 max_ms 加一个节拍间隔，两者之和须在计数范围内。
  // The timeout compares against max_ms plus one tick interval; the sum must stay
  // countable.
  static_assert(MAX_WAIT_MS <= MAX_COUNT / 2U);

  /// 合并阶段与已计毫秒数 / Encode the phase and the counted milliseconds.
  static constexpr uint32_t MakeState(Phase phase, uint32_t count)
  {
    return (count << PHASE_BITS) | static_cast<uint32_t>(phase);
  }

  /// 提取阶段 / Extract the phase.
  static Phase GetPhase(uint32_t state) { return static_cast<Phase>(state & PHASE_MASK); }

  /// 提取已计毫秒数 / Extract the counted milliseconds.
  static uint32_t GetCount(uint32_t state) { return state >> PHASE_BITS; }

  /// 推进本对象的等待 / Advance the wait of this object.
  void Tick(uint32_t elapsed_ms);

  CheckFun check_;  ///< 条件函数 / Condition function.
  DoneFun done_;    ///< 结果函数 / Result function.
  void* arg_;       ///< 传给两个函数的参数 / Argument passed to both functions.
  /// Start() 在 SETUP 阶段写入，节拍在 WAITING 阶段读取 / Written by Start() in SETUP,
  /// read by the tick in WAITING.
  uint32_t min_ms_ = 0;
  uint32_t max_ms_ = 0;
  /// 阶段与从 Start() 起已计的毫秒数 / Phase and milliseconds counted since Start().
  std::atomic<uint32_t> state_{MakeState(Phase::IDLE, 0U)};
  /// 链表中的下一个对象，挂入后不变 / Next object in the list; fixed once linked.
  DriverAsyncCondition* next_ = nullptr;

  /// 全局链表头 / Head of the global list.
  static inline std::atomic<DriverAsyncCondition*> head_{nullptr};
};

}  // namespace LibXR
