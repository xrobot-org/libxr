#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

#include "libxr_def.hpp"

namespace LibXR::USB
{

/**
 * @class EndpointSlots
 * @brief 端点槽的无锁记账 / Lock-free bookkeeping of an endpoint's slots
 *
 * 记录一个端点的每个槽各处于什么状态（空闲、借出、作废、排队、在途）、排队的槽谁先
 * 谁后、端点是否启用以及方向、每个槽的借出戳和排队请求的参数（长度与零长度包标志）。
 * 槽数在构造时给定，1 到 MAX_SLOTS。不访问硬件，不调用回调，不知道包长和槽容量，也不
 * 解释零长度包标志；这些由 Endpoint 负责。
 * Records for every slot of an endpoint which state it is in (free, lent, stale, queued,
 * in flight), the order of the queued slots, whether the endpoint is enabled and in
 * which direction, each slot's lend stamp and the parameters of a queued request (length
 * and zero-length flag). The slot count is given at construction, 1 to MAX_SLOTS. It
 * touches no hardware, runs no callback, knows neither the packet size nor the slot
 * capacity and does not interpret the zero-length flag; Endpoint owns those.
 *
 * @note 每个操作对槽状态的改变是内部状态字的一次原子比较交换（失败时重试），所有槽的
 *       状态、启用位和方向都在这一个字里，操作之间不加锁。标为“任何上下文”的操作可与
 *       其他任何操作并发；标为“仅 owner”的操作由同一个串行执行者调用，彼此不并发，但
 *       可与任何上下文的操作并发。
 *       Each operation changes slot states with one atomic compare-and-swap on the
 *       internal state word, retried on failure; the states of all slots, the enable bit
 *       and the direction live in that one word and no lock is taken. Operations marked
 *       "any context" may run concurrently with any other; operations marked "owner
 *       only" are called by one serialized executor, never concurrently with each other,
 *       but concurrently with any-context operations.
 * @note 借出戳：每个槽一个 32 位计数。一次借出结束时（持有者在 Queue 或 Release 中交回）
 *       戳加一，写在槽状态改变之前；句柄只在戳等于该槽当前的戳时被接受，所以每次借出
 *       的句柄互不相同，交回之后立即失效。同一个句柄不得在两个上下文中同时使用：Queue
 *       和 Release 先推进戳、写请求参数，再做比较交换。
 *       Lend stamp: a 32-bit count per slot. It advances by one when a lend ends (the
 *       holder hands the slot back in Queue or Release), written before the slot state
 *       changes; a handle is accepted only while its stamp equals the slot's current
 *       one, so the handles of different lends differ and a handle stops being accepted
 *       the moment it is handed back. One handle must not be used from two contexts at
 *       the same time: Queue and Release advance the stamp and write the request
 *       parameters before their compare-and-swap.
 * @note 状态字不记录绑定的代数。随绑定变化的量（包长、一次请求的上限）由调用者在借出
 *       或认领成功之后读取：此后端点若被停用，该槽作废，Queue 返回 ABORTED。
 *       The state word keeps no binding generation. The caller reads what changes with
 *       a binding (packet size, request limit) after the lend or claim succeeded: if
 *       the endpoint is disabled afterwards the slot is stale and Queue returns
 *       ABORTED.
 */
class EndpointSlots
{
 public:
  /// 不指向任何槽的槽号 / Slot index that refers to no slot
  static constexpr uint8_t NO_SLOT = 0xFFU;
  /// 一个端点最多几个槽：全部槽的状态要放进一个状态字 / Most slots of one endpoint:
  /// the states of all slots have to fit one state word
  static constexpr uint8_t MAX_SLOTS = 8U;

  /// 一次借出：槽号与借出戳 / One lend: slot index and lend stamp
  struct Lend
  {
    uint8_t index = NO_SLOT;  ///< 槽号 / Slot index
    uint32_t stamp = 0;       ///< 借出戳 / Lend stamp
  };

  /// 排队请求的参数，原样存取 / Parameters of a queued request, stored as given
  struct Request
  {
    size_t length = 0;  ///< 请求长度 / Request length
    bool zlp = false;   ///< 提交者的零长度包标志 / The submitter's zero-length flag
  };

  /// 取出的排队请求 / A queued request taken for starting
  struct Queued
  {
    uint8_t index = NO_SLOT;  ///< 槽号；NO_SLOT 表示没有排队请求 / NO_SLOT: none
    Request request;          ///< 排队时给出的参数 / Parameters given when queued
  };

  /**
   * @brief 构造记账 / Construct the bookkeeping
   * @param slot_count 槽数，1 到 MAX_SLOTS / Number of slots, 1 to MAX_SLOTS
   * @note 构造后所有槽空闲，端点未启用。每槽的记录在这里分配一次。
   *       All slots start free and the endpoint starts disabled. The per-slot records
   *       are allocated here, once.
   */
  explicit EndpointSlots(uint8_t slot_count);
  /// 释放每槽的记录 / Free the per-slot records
  ~EndpointSlots();

  /// 不可复制：状态字只属于一个端点 / Not copyable: the state word has one owner
  EndpointSlots(const EndpointSlots&) = delete;
  /// 不可复制赋值 / Not copy-assignable
  EndpointSlots& operator=(const EndpointSlots&) = delete;

  /**
   * @brief 启用并开始新的绑定 / Enable and start a new binding
   * @param sending true 为 IN（发送），false 为 OUT（接收）/ true for IN, false for OUT
   * @pre 仅 owner，端点未启用。 / Owner only, endpoint disabled.
   * @note 作废的槽保持作废及其戳，直到持有者交回；它只挡住自己。在途的槽属于上一次
   *       绑定被停的请求，保持在途，不能借出，直到 owner 以 FinishFree 结束它；在此之前
   *       owner 不调用 TakeNext。
   *       A stale slot keeps its state and stamp until its holder hands it back; it
   *       blocks only itself. An in-flight slot belongs to a stopped request of the
   *       previous binding: it stays in flight and cannot be lent until the owner ends
   *       it with FinishFree, and the owner does not call TakeNext before that.
   */
  void Enable(bool sending);

  /**
   * @brief 停用：排队的槽变空闲，借出的槽作废 / Disable: free queued, mark lent stale
   * @return 由排队变为空闲的槽数 / Number of queued slots freed
   * @pre 仅 owner。 / Owner only.
   * @note 在途的槽不受影响，由 owner 之后以 FinishFree 结束。之后借出和认领返回
   *       STATE_ERR；作废的槽交回时 Queue 返回 ABORTED，Release 返回 OK。
   *       An in-flight slot is left alone; the owner ends it later with FinishFree.
   *       Afterwards lends and claims return STATE_ERR; a stale slot handed back gets
   *       ABORTED from Queue and OK from Release.
   */
  uint8_t Disable();

  /**
   * @brief 排队的槽变空闲，端点保持启用 / Free queued slots; stay enabled
   * @return 由排队变为空闲的槽数 / Number of queued slots freed
   * @pre 仅 owner。 / Owner only.
   */
  uint8_t DropQueued();

  /**
   * @brief 为发送借一个空闲槽 / Lend a free slot for sending
   * @param lend 成功时写入槽号和这次借出的戳 / Receives the index and the stamp of
   *        this lend on success
   * @return OK；BUSY：没有空闲槽；STATE_ERR：未启用或方向是 OUT。
   *         OK; BUSY: no free slot; STATE_ERR: disabled or the direction is OUT.
   * @note 任何上下文。槽 0 优先。 / Any context. Slot 0 is preferred.
   */
  [[nodiscard]] ErrorCode LendForSending(Lend& lend);

  /**
   * @brief 为接收认领一个空闲槽 / Claim a free slot for receiving
   * @param lend 成功时写入槽号和这次借出的戳 / Receives the index and the stamp of
   *        this lend on success
   * @return OK；BUSY：没有空闲槽；STATE_ERR：未启用或方向是 IN。
   *         OK; BUSY: no free slot; STATE_ERR: disabled or the direction is IN.
   * @note 任何上下文。认领和借出一样得到只属于这一次的戳，该槽此前的句柄都不再被接受；
   *       认领的槽在 Queue 之前只属于认领者。
   *       Any context. A claim gets a stamp of its own like a lend, and no earlier
   *       handle of the slot is accepted; the claimed slot belongs to the claimer alone
   *       until it is queued.
   */
  [[nodiscard]] ErrorCode ClaimForReceiving(Lend& lend);

  /**
   * @brief 把借出的槽排入队列 / Queue a lent slot
   * @param lend 当前这次借出 / The current lend
   * @param request 请求参数，原样交给 TakeNext / Request parameters, handed to TakeNext
   * @param sending 调用者的方向，须与绑定的方向一致 / Caller's direction; must match
   * @return OK；ABORTED：槽在停用时被作废，现已空闲；STATE_ERR：lend 不是该槽当前这次
   *         借出，或方向不符，槽不受影响。作废先于方向检查：作废的槽方向不符时也返回
   *         ABORTED 并变空闲。
   *         OK; ABORTED: the slot was marked stale by Disable and is now free;
   *         STATE_ERR: lend is not the slot's current lend, or the direction differs,
   *         and the slot is left alone. Staleness is checked before the direction: a
   *         stale slot with the wrong direction also gets ABORTED and becomes free.
   * @note 任何上下文。参数在确认句柄之后、排队的比较交换之前写入；此时只有持有者能
   *       改变该槽，所以写入不与他人冲突。返回 OK 或 ABORTED 之后这个句柄不再被接受。
   *       Any context. The parameters are written after the handle is confirmed and
   *       before the queueing compare-and-swap; only the holder can change the slot
   *       then, so the write races with nobody. After OK or ABORTED the handle is no
   *       longer accepted.
   */
  [[nodiscard]] ErrorCode Queue(const Lend& lend, const Request& request, bool sending);

  /**
   * @brief 交回借出或作废的槽，不排队 / Hand back a lent or stale slot without queuing
   * @param lend 当前这次借出 / The current lend
   * @return OK（包括作废的槽）；STATE_ERR：lend 不是当前这次借出，槽不受影响。
   *         OK (a stale slot included); STATE_ERR: lend is not the current lend, and
   *         the slot is left alone.
   * @note 任何上下文。 / Any context.
   */
  ErrorCode Release(const Lend& lend);

  /**
   * @brief 取最早排队的槽转为在途 / Take the earliest queued slot and mark it in flight
   * @return 槽号与请求参数；没有排队请求时 index 为 NO_SLOT。
   *         Index and request parameters; index is NO_SLOT when nothing is queued.
   * @pre 仅 owner。 / Owner only.
   * @note 排队的先后就是各次 Queue 的比较交换成功的先后。
   *       The order of the queue is the order in which the compare-and-swaps of the
   *       Queue calls succeeded.
   */
  Queued TakeNext();

  /**
   * @brief 在途的槽结束并变空闲 / End the in-flight slot as free
   * @param index 在途的槽 / The in-flight slot
   * @pre 仅 owner。端点可以已停用或已重新启用。
   *      Owner only. The endpoint may be disabled or enabled again.
   */
  void FinishFree(uint8_t index);

  /**
   * @brief 在途的槽结束并借出 / End the in-flight slot as lent
   * @param index 在途的槽 / The in-flight slot
   * @return 这次借出的戳 / Stamp of this lend
   * @pre 仅 owner，端点已启用。 / Owner only, endpoint enabled.
   * @note 用于收到数据的 OUT 槽：槽借给完成回调的接收者，由其 Release。
   *       For an OUT slot holding received bytes: the slot is lent to the receiver of
   *       the completion callback, who releases it.
   */
  uint32_t FinishLent(uint8_t index);

  /**
   * @brief 在途的槽排队时给出的参数 / Parameters an in-flight slot was queued with
   * @param index 在途的槽 / The in-flight slot
   * @pre 仅 owner。 / Owner only.
   */
  Request RequestOf(uint8_t index) const;

 private:
  /// 槽状态，每个槽 3 位 / Slot state, three bits per slot
  enum SlotState : uint32_t
  {
    FREE = 0U,       ///< 空闲 / Free
    LENT = 1U,       ///< 借出或认领 / Lent or claimed
    STALE = 2U,      ///< 借出后端点被停用 / Lent, then the endpoint was disabled
    QUEUED = 3U,     ///< 排队等待启动 / Queued for starting
    IN_FLIGHT = 4U,  ///< 已交给硬件 / Handed to the hardware
  };

  /**
   * @brief 状态字布局 / State word layout
   * @note 槽 i 的状态在 [3i+2:3i]（i 为 0 到 7），启用 [24]，发送 [25]，排队序号
   *       [31:26]。排队序号每次 Queue 成功加一，按 64 回绕。
   *       The state of slot i is in [3i+2:3i] (i from 0 to 7), enabled [24], sending
   *       [25], queueing sequence [31:26]. The sequence advances by one with every
   *       successful Queue and wraps at 64.
   */
  static constexpr uint32_t SLOT_BITS = 3U;
  static constexpr uint32_t SLOT_MASK = (1U << SLOT_BITS) - 1U;  ///< 一个槽 / One slot
  static constexpr uint32_t ENABLED_BIT = 1U << 24U;             ///< 已启用 / Enabled
  static constexpr uint32_t SENDING_BIT = 1U << 25U;  ///< 方向为 IN / Direction is IN
  /// 排队序号的最低位 / LSB of the queueing sequence
  static constexpr uint32_t SEQUENCE_SHIFT = 26U;
  /// 排队序号 / The queueing sequence
  static constexpr uint32_t SEQUENCE_MASK = (1U << 6U) - 1U;

  /// 每个槽在状态字之外的记录 / What is kept per slot outside the state word
  struct Record
  {
    /// 当前这次借出的戳；结束借出的一方在改变槽状态之前加一 / Stamp of the current
    /// lend; advanced by whoever ends the lend, before the slot state changes
    std::atomic<uint32_t> stamp{0};
    /// 排队请求的参数；持有者在 Queue 中写，owner 在 TakeNext 中读 / Queued request
    /// parameters; written by the holder in Queue, read by the owner in TakeNext
    Request request;
    /// 排队时的排队序号；读写同 request / Queueing sequence at the time of queueing;
    /// written and read like request
    uint8_t order = 0;
  };

  /// 读槽状态 / Read a slot's state
  static uint32_t StateOf(uint32_t word, uint8_t index);
  /// 写槽状态 / Write a slot's state
  static uint32_t WithState(uint32_t word, uint8_t index, uint32_t state);
  /// 槽转为 QUEUED 并把排队序号加一 / Mark the slot QUEUED and advance the queueing
  /// sequence by one
  static uint32_t MarkQueued(uint32_t word, uint8_t index);
  /// lend 的戳是否是该槽当前的戳 / Whether lend carries the slot's current stamp
  bool Holds(const Lend& lend) const;
  /// LendForSending 与 ClaimForReceiving 的共同部分 / Shared body of the two lends
  ErrorCode LendFree(bool sending, Lend& lend);

  const uint8_t slot_count_;       ///< 槽数，构造时固定 / Slot count, fixed
  std::atomic<uint32_t> word_{0};  ///< 状态字 / State word
  Record* const records_;          ///< 每槽的记录，slot_count_ 个 / Per-slot records
};

}  // namespace LibXR::USB
