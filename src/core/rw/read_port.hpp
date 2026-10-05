#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

#include "operation.hpp"
#include "queue.hpp"

namespace LibXR
{

class Pipe;

/**
 * @brief 基于 SPSC 字节队列的读端口 / Read endpoint backed by an SPSC byte queue.
 *
 * 后端持续向队列写入数据并通知端口；端口最多保存一个尚未满足的读请求。
 * The backend feeds the queue and notifies the port, which retains at most one
 * pending read request.
 *
 * @pre 后端负责串行化接收生产者。同一端口的 BLOCK 与非 BLOCK 操作不得重叠或嵌套。
 *      The backend serializes RX production. BLOCK and non-BLOCK operations on the
 *      same port must not overlap or nest.
 * @note 端口、所属后端和通知对象须在所有可能访问它们的调用与回调返回前保持有效。
 *       The port, its backend, and notification targets must outlive all calls and
 *       callbacks that may access them.
 */
class ReadPort
{
 private:
  /// 请求处理阶段 / Request processing phase.
  enum class Phase : uint32_t
  {
    IDLE = 0U,     ///< 可接纳请求 / Available for a request.
    CLAIMED = 1U,  ///< 独占请求处理或出队 / Exclusive request or dequeue access.
    PENDING = 2U,  ///< 请求等待数据 / Published request waiting for data.
    CLAIMED_WITH_WAITER = 3U,  ///< 超时方等待安全交接 / Timeout awaits a safe handoff.
    BLOCK_CLAIMED = 4U,        ///< BLOCK 完成已认领 / BLOCK completion has been claimed.
  };

  /// 数据发布通知，可与任一阶段共存；不计数字节 / Publication hint, not a byte count.
  static constexpr uint32_t EVENT_BIT = 1U << 31U;
  /// 请求阶段所占的位 / Bits containing the request phase.
  static constexpr uint32_t PHASE_MASK = EVENT_BIT - 1U;

  /// 挂起读请求 / Pending read request.
  struct Request
  {
    RawData data;      ///< 借用的接收缓冲区 / Borrowed destination buffer.
    ReadOperation op;  ///< 完成通知方式 / Completion notification.
  };

  /// 提取请求阶段 / Extract the request phase.
  static Phase GetPhase(uint32_t state) { return static_cast<Phase>(state & PHASE_MASK); }

  /// 替换阶段并保留数据通知 / Replace the phase while preserving the publication hint.
  static uint32_t WithPhase(uint32_t state, Phase phase)
  {
    return (state & EVENT_BIT) | static_cast<uint32_t>(phase);
  }

  /// 检查是否有待处理的数据通知 / Check for a publication hint.
  static bool HasEvent(uint32_t state) { return (state & EVENT_BIT) != 0U; }

  /// 尝试独占空闲端口并清除已观察到的通知 / Claim IDLE and clear the observed hint.
  [[nodiscard]] bool TryClaimIdle();
  /// 判断数据是否足够；零长度请求等待非空 / Test availability, or nonempty for zero size.
  [[nodiscard]] bool HasEnough(size_t available, size_t requested) const;
  /// 释放 CLAIMED 并保留数据通知 / Release CLAIMED while preserving publication hints.
  void ReleaseClaimed(bool in_isr);
  /// 在写入 BLOCK 接收缓冲区前认领完成 / Claim BLOCK completion before copying data.
  [[nodiscard]] bool ClaimBlockCompletion();
  /// 等待方取走结果后释放端口 / Release the port after the waiter takes its result.
  void ReleaseBlockCompletion(bool in_isr);
  /// 发布接收入队通知并处理挂起读 / Signal RX production and process a pending read.
  void PublishProduced(bool in_isr);
  /// 处理 Pipe 共享队列的数据通知 / Handle data notification from the shared Pipe queue.
  void NotifyDataAvailable(bool in_isr);
  /// 认领挂起请求，数据足够时完成 / Claim a pending request and complete it if possible.
  void ProcessPendingReads(bool in_isr);
  /// 复制数据，释放处理权，再通知非 BLOCK 操作 / Copy, release, then notify non-BLOCK.
  void CompleteClaimedRead(bool in_isr);
  /// 复制数据并唤醒 BLOCK 等待者，由等待者释放端口 / Copy and wake; the waiter releases.
  void CompleteClaimedBlock(bool in_isr);
  /// 等待完成，超时时与处理方安全交接 / Wait for completion or a safe timeout handoff.
  [[nodiscard]] ErrorCode WaitForBlock(ReadOperation& op);
  /// 在 Pipe 构造时绑定借用队列 / Bind the borrowed queue during Pipe construction.
  void BindQueue(SPSCQueue<uint8_t>* queue);

  friend class Pipe;

 public:
  /**
   * @brief 接收队列的短期写入接口 / Short-lived producer interface for the RX queue.
   *
   * 一次使用可多次调用同一种入队方法，最后显式调用一次 Publish，包括未写入数据的情况。
   * 不可复制或移动；析构不会自动发布通知。
   * Use one production method repeatedly, then call Publish exactly once, even if no
   * bytes were written. This object is noncopyable and nonmovable. Destruction does
   * not publish notifications.
   *
   * @note 入队即通过 SPSC 发布字节；Publish 推进挂起读，可能同步执行完成回调。
   *       Enqueueing makes bytes visible through the SPSC; Publish drives pending reads
   *       and may invoke completion callbacks inline.
   */
  class ReadQueue
  {
   public:
    ReadQueue(const ReadQueue&) = delete;
    ReadQueue& operator=(const ReadQueue&) = delete;
    ReadQueue(ReadQueue&&) = delete;
    ReadQueue& operator=(ReadQueue&&) = delete;
    /// 开发期检查是否已 Publish / Check for Publish in development builds.
    ~ReadQueue();

    /**
     * @brief 批量复制字节到接收队列 / Copy a batch of bytes into the RX queue.
     * @param data 数据源；正长度时不可为空 / Source, non-null for a positive size.
     * @param size 字节数 / Number of bytes.
     * @return 全部入队返回 OK；空间不足返回 FULL，不部分写入。零长度返回 OK。
     *         OK if all bytes are queued; FULL without partial writes if space is
     *         insufficient. A zero size returns OK.
     * @pre 尚未调用 Publish / Publish has not been called.
     */
    [[nodiscard]] ErrorCode PushBatch(const uint8_t* data, size_t size);

    /**
     * @brief 通过回调直接写入接收队列 / Fill the RX queue through a writer callback.
     * @tparam Writer 写入器类型 / Writer callback type.
     * @param limit 最多提供的字节数 / Maximum number of bytes to offer.
     * @param writer 签名为 size_t(uint8_t*, size_t, uint8_t*, size_t)，接收按 FIFO
     *        顺序排列的最多两段空间，返回已写入的连续前缀长度。
     *        Callable as size_t(uint8_t*, size_t, uint8_t*, size_t); receives up to two
     *        FIFO-ordered spans and returns the length of the written prefix.
     * @return 实际入队字节数 / Number of bytes actually queued.
     * @pre 尚未 Publish；回调不可重入同一队列的生产操作，返回值不得超过提供的空间。
     *      Before Publish; the callback must not reenter production on this queue or
     *      return more bytes than offered.
     * @note 提供长度不超过 limit 和空闲空间。非空时只调用一次回调，指针仅在回调内有效。
     *       The offer is limited by limit and free space. The callback runs once for a
     *       nonempty offer; its pointers are valid only during that call.
     */
    template <typename Writer>
    [[nodiscard]] size_t PushWithWriter(size_t limit, Writer&& writer)
    {
      ASSERT_FROM_CALLBACK(!finished_, in_isr_);

      const size_t produced = port_.queue_data_->ProduceWithWriter(
          limit,
          [&](void* first, size_t first_size, void* second, size_t second_size) -> size_t
          {
            const size_t produced = writer(static_cast<uint8_t*>(first), first_size,
                                           static_cast<uint8_t*>(second), second_size);
            ASSERT_FROM_CALLBACK(produced <= first_size + second_size, in_isr_);
            return produced;
          });
      dirty_ = dirty_ || (produced != 0U);
      return produced;
    }

    /**
     * @brief 获取接收队列当前空闲空间 / Get current RX queue free space.
     * @return 空闲字节数；查询不预留空间 / Free bytes; the query reserves no space.
     */
    [[nodiscard]] size_t EmptySize() const;

    /**
     * @brief 获取接收队列容量 / Get the RX queue capacity.
     * @return 队列总容量，单位为字节 / Total queue capacity in bytes.
     */
    [[nodiscard]] size_t Capacity() const;

    /**
     * @brief 结束本次写入并通知挂起读 / End production and notify a pending read.
     * @pre 每个 ReadQueue 仅调用一次，之后不再使用该写入接口。
     *      Call once per ReadQueue and do not use the producer interface afterward.
     * @note 未写入字节时只结束本次使用；有数据时使用获取接口时的 in_isr 上下文
     *       推进读请求，可能同步复制数据并调用完成回调。
     *       With no produced bytes, only ends this use. Otherwise drives reads using
     *       the captured in_isr context and may copy data and invoke callbacks inline.
     */
    void Publish();

   private:
    friend class ReadPort;

    ReadQueue(ReadPort& port, bool in_isr) : port_(port), in_isr_(in_isr) {}

    ReadPort& port_;         ///< 所属读端口 / Associated read port.
    const bool in_isr_;      ///< 本次生产的调用上下文 / Context of this production call.
    bool dirty_ = false;     ///< 是否成功写入过字节 / Whether any bytes were queued.
    bool finished_ = false;  ///< 是否已调用 Publish / Whether Publish was called.
  };

  /**
   * @brief 构造读端口并分配接收队列 / Construct a read port and allocate its RX queue.
   * @param buffer_size 队列容量，单位为字节；为零时不分配，端口保持未绑定。
   *        Queue capacity in bytes; zero leaves the port unbound without allocating.
   * @note Pipe 在构造时将未绑定的读端口绑定到共享队列。
   *       Pipe binds its initially unbound read port to the shared queue at construction.
   */
  explicit ReadPort(size_t buffer_size = 128U);

  /**
   * @brief 析构读端口 / Destroy the read port.
   * @pre 相关请求、调用和回调已结束 / All related requests, calls, and callbacks ended.
   * @note 不取消请求，也不释放队列存储 / Does not cancel requests or free queue storage.
   */
  virtual ~ReadPort() = default;

  /**
   * @brief 获取后端接收入队接口 / Obtain the backend RX producer interface.
   * @param in_isr 是否在中断中调用；该值用于本次 Publish 引起的通知。
   *        Whether called in an ISR; used for notifications caused by this Publish.
   * @return 本次接收入队接口 / Producer interface for this RX operation.
   * @pre 队列已绑定，由唯一接收生产者调用；Pipe 的借用读端口不可使用此接口。
   *      A bound queue and the sole RX producer are required; not for a Pipe's borrowed
   *      read endpoint.
   */
  [[nodiscard]] ReadQueue GetReadQueue(bool in_isr = false);

  /**
   * @brief 获取当前空闲空间 / Get current free space.
   * @return 空闲字节数，未绑定时为零；并发消费或生产可使结果变化。
   *         Free bytes, or zero when unbound; concurrent progress may change the value.
   */
  [[nodiscard]] size_t EmptySize() const;

  /**
   * @brief 获取当前排队的数据量 / Get the current queued data size.
   * @return 已排队字节数，未绑定时为零；查询不预留数据，并发进展可使结果变化。
   *         Queued bytes, or zero when unbound; the query reserves no data and concurrent
   *         progress may change the value.
   */
  [[nodiscard]] size_t Size() const;

  /**
   * @brief 获取接收队列容量 / Get the RX queue capacity.
   * @return 队列总字节数，未绑定时为零 / Total capacity in bytes, or zero when unbound.
   */
  [[nodiscard]] size_t Capacity() const;

  /**
   * @brief 检查端口是否支持读取 / Check whether the port supports reads.
   * @return 已绑定队列时为 true，不表示队列非空或端口空闲。
   *         True when a queue is bound; does not indicate available data or an idle port.
   */
  [[nodiscard]] bool Readable() const;

  /**
   * @brief 提交读请求 / Submit a read request.
   * @param data 接收地址和字节数。正长度请求须有有效地址，数据足够时才一次性复制；
   *        零长度请求等待队列非空，不消费字节，地址可为空。
   *        Destination and byte count. Positive reads require a valid address and copy
   *        only when the full request is available. Zero size waits for a nonempty queue
   *        without consuming bytes and permits a null address.
   * @param op 完成通知方式；其回调、轮询状态或信号量在操作结束前须保持有效。
   *        Completion descriptor; its callback, polling state, or semaphore must remain
   *        valid until the operation ends.
   * @param in_isr 当前调用是否位于中断 / Whether this call is in an ISR.
   * @return 非 BLOCK 的 OK 表示请求已接纳，可能同步完成，也可能等待后续数据；
   *         BLOCK 的 OK 表示读取完成，超时取消返回 TIMEOUT。未绑定返回 NOT_SUPPORT，
   *         请求处理状态被占用返回 BUSY，正长度超过队列容量返回 SIZE_ERR。
   *         Non-BLOCK OK means admitted, with inline or deferred completion. BLOCK OK
   *         means completed; timeout cancellation returns TIMEOUT. NOT_SUPPORT when
   *         unbound, BUSY when request processing is occupied, SIZE_ERR above capacity.
   * @pre BLOCK 只能在线程中调用；返回前不可在同一端口再 Read 或 ClearQueuedData，
   *      也不可与尚未结束的非 BLOCK 操作重叠。每个信号量只服务一个活动 BLOCK 调用。
   *      BLOCK is task-only; no same-port Read or ClearQueuedData before it returns,
   *      and no overlap with a live non-BLOCK operation. Use a dedicated semaphore.
   * @note 接收缓冲区须保持有效直到完成或取消；BLOCK 缓冲区须保持到调用返回。
   *       超时与处理方竞争时，须等处理方停止访问缓冲区后才返回，可能超过指定时间；
   *       若完成方赢得交接则返回完成结果。
   *       Keep the destination valid until completion or cancellation, and through return
   *       for BLOCK. A timeout racing progress waits until buffer access ends, possibly
   *       exceeding the requested duration; completion wins if already claimed.
   * @note 非 BLOCK 回调可提交后续非 BLOCK 读；跨线程回调顺序不保证。立即满足的
   *       BLOCK 直接返回，不释放信号量；拒绝接纳的请求不发送完成通知。
   *       Non-BLOCK callbacks may submit another non-BLOCK read; cross-thread callback
   *       order is not guaranteed. Immediate BLOCK completion does not post a semaphore;
   *       rejected requests emit no completion notification.
   * @note 回调在实际完成读取的上下文执行，可能位于本次调用或后端 Publish 内；
   *       轮询状态应以 acquire 读取。不可阻塞等待依赖当前调用返回才能推进的操作。
   *       Callbacks run in the context completing the read, possibly inside this call or
   *       backend Publish. Load polling status with acquire ordering. Do not block on
   *       progress that requires the current call to return.
   */
  ErrorCode operator()(RawData data, ReadOperation& op, bool in_isr = false);

 protected:
  /**
   * @brief 接收队列空间可用通知 / RX queue space-available notification.
   *
   * 正长度出队后、完成通知前调用；ClearQueuedData 成功时也调用，包括空队列。
   * bool 参数表示本次调用是否在中断中；默认实现为空。
   * Called after positive dequeue and before completion notification, or after a
   * successful ClearQueuedData even for an empty queue. The bool argument indicates
   * ISR context; the default implementation does nothing.
   *
   * @note 派生实现负责恢复接收，或保留通知供当前生产者再次处理；需与其他接收入口串行化。
   *       Overrides resume RX or retain a hint for the producer to recheck,
   *       serialized with other RX entry points.
   */
  virtual void OnReadQueueSpaceAvailable(bool) {}

 public:
  /**
   * @brief 丢弃当前已排队的接收字节 / Discard currently queued RX bytes.
   * @param in_isr 当前调用是否位于中断 / Whether this call is in an ISR.
   * @return 成功返回 OK，未绑定返回 NOT_SUPPORT，存在活动请求或出队处理时返回 BUSY。
   *         OK on success, NOT_SUPPORT when unbound, BUSY during an active request or
   *         dequeue operation.
   * @pre 同一端口的 BLOCK 调用已返回 / Any same-port BLOCK call has returned.
   * @note 只推进消费者位置，可与唯一生产者并发。并发到达的数据可能保留或被丢弃。
   *       不取消挂起请求；成功后释放处理权并通知队列空间可用。
   *       Advances only the consumer index and may overlap the sole producer. Racing
   *       data may survive or be discarded. Does not cancel pending reads; releases
   *       request processing and notifies available space on success.
   */
  [[nodiscard]] ErrorCode ClearQueuedData(bool in_isr = false);

 private:
  /// 普通端口分配、Pipe 端口借用的字节队列 / Allocated RX queue, or borrowed Pipe queue.
  SPSCQueue<uint8_t>* queue_data_ = nullptr;
  /// 请求阶段与可合并的数据通知 / Request phase and coalescible publication hint.
  std::atomic<uint32_t> state_{static_cast<uint32_t>(Phase::IDLE)};
  /// 通过请求阶段保护的唯一挂起请求 / Single pending request protected by the phase.
  Request info_{};
  /// Post 前写入，Wait 成功后读取 / Written before Post, read after a successful Wait.
  ErrorCode block_result_ = ErrorCode::OK;
};

}  // namespace LibXR
