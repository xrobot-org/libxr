#pragma once

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string_view>
#include <type_traits>
#include <utility>

#include "operation.hpp"
#include "queue.hpp"

namespace LibXR
{

class Pipe;

/**
 * @brief 基于 SPSC 字节队列的写端口 / Write endpoint backed by an SPSC byte queue.
 *
 * 普通后端按请求顺序取走数据，整个请求被后端接收后完成；Pipe 共用读写字节队列，
 * 写入队列后即可完成。调用者的数据会复制到队列，完成通知对象仍由调用者持有。
 * Queued backends consume requests in FIFO order and complete each after accepting
 * all its bytes. Pipe shares the byte queue with its reader and completes writes on
 * admission. Payload is copied; completion targets remain caller-owned.
 *
 * @pre 后端负责串行化出队及完成通知。同一端口的 BLOCK 与非 BLOCK 操作不得重叠或
 *      嵌套；同时只能有一个尚未返回的 BLOCK 调用。超时请求退出队列前仍属于 BLOCK。
 *      The backend serializes consumption and completion. BLOCK and non-BLOCK work
 *      on the same port must not overlap or nest. Only one BLOCK caller may be live;
 *      a timed-out queued request remains BLOCK work until it retires.
 * @note 端口、所属后端和通知对象须在所有相关调用、回调及持有写入权的 Stream
 *       结束前保持有效。完成表示后端已接收数据，具体接收位置由后端约定。
 *       The port, backend, and notification targets must outlive related calls,
 *       callbacks, and owning Streams. Completion means acceptance at the storage
 *       boundary defined by the backend.
 */
class WritePort
{
 private:
  /// 生产者占用与 BLOCK 等待阶段 / Producer admission and BLOCK waiting phase.
  enum class Phase : uint32_t
  {
    IDLE = 0U,            ///< 可接纳新的写入 / Available for a new writer.
    LOCKED = 1U,          ///< 生产者正在准备请求 / Producer preparing a request.
    BLOCK_WAITING = 2U,   ///< 调用者等待请求完成 / Caller waiting for completion.
    BLOCK_CLAIMED = 3U,   ///< 后端已认领 BLOCK 完成 / Backend claimed BLOCK completion.
    BLOCK_DETACHED = 4U,  ///< 调用已超时，请求仍在队列中 / Timed out, still queued.
    /// 后续 BLOCK 等待旧请求退出 / Later BLOCK awaits retirement.
    BLOCK_RETIRE_WAITING = 5U,
  };

  /// 低三位存阶段，其余位存已发布请求数 / Low three bits: phase; rest: released count.
  static constexpr uint32_t PHASE_BITS = 3U;
  static constexpr uint32_t PHASE_MASK = (1U << PHASE_BITS) - 1U;
  /// 发布一个请求的计数增量 / Encoded increment for one released request.
  static constexpr uint32_t RELEASED_INCREMENT = 1U << PHASE_BITS;
  /// 状态字可表示的最大已发布请求数 / Maximum encodable released-request count.
  static constexpr uint32_t MAX_RELEASED_REQUESTS = UINT32_MAX >> PHASE_BITS;

  static_assert(static_cast<uint32_t>(Phase::BLOCK_RETIRE_WAITING) <= PHASE_MASK);

  /// 写请求的长度与完成通知 / Write request length and completion descriptor.
  struct Request
  {
    size_t size;        ///< 整个请求的字节数 / Original request size in bytes.
    WriteOperation op;  ///< 完成通知方式 / Completion notification.
  };

  static_assert(std::is_trivially_copyable_v<Request>);
  static_assert(std::is_trivially_destructible_v<Request>);

  /// 合并阶段与已发布请求数 / Encode the phase and released-request count.
  static constexpr uint32_t MakeState(Phase phase, uint32_t released)
  {
    return (released << PHASE_BITS) | static_cast<uint32_t>(phase);
  }

  /// 提取生产者与 BLOCK 阶段 / Extract the producer and BLOCK phase.
  static Phase GetPhase(uint32_t state) { return static_cast<Phase>(state & PHASE_MASK); }

  /// 提取后端可消费的 FIFO 前缀请求数 / Extract the consumable FIFO prefix count.
  static uint32_t GetReleasedCount(uint32_t state) { return state >> PHASE_BITS; }

  /// 替换阶段并保留已发布请求数 / Replace the phase while preserving the released count.
  static uint32_t WithPhase(uint32_t state, Phase phase)
  {
    return (state & ~PHASE_MASK) | static_cast<uint32_t>(phase);
  }

  /// 检查请求容量是否可由状态字表示 / Check that request capacity fits the state word.
  static size_t ValidateQueueSize(size_t queue_size)
  {
    ASSERT(queue_size <= MAX_RELEASED_REQUESTS);
    return queue_size;
  }

  /// 无请求队列时采用入队即完成模式 / No metadata queue means completion on admission.
  [[nodiscard]] bool IsAdmissionMode() const { return queue_requests_ == nullptr; }
  /// 以 acquire 读取当前阶段 / Load the current phase with acquire ordering.
  [[nodiscard]] Phase LoadPhase() const
  {
    return GetPhase(state_.load(std::memory_order_acquire));
  }
  /// 尝试从 IDLE 取得生产者独占权，失败时可选记录当时阶段 /
  /// Try to claim exclusive producer access from IDLE; optionally record observed phase.
  [[nodiscard]] bool TryClaimProducer(Phase* failed_phase = nullptr);
  /// 登记后续 BLOCK 对旧超时请求的等待 / Register a later BLOCK retirement waiter.
  [[nodiscard]] bool TryRegisterRetirementWait(WriteOperation& op);
  /// 等旧请求退出，超时不提交新数据 / Await retirement; timeout submits no data.
  [[nodiscard]] ErrorCode WaitForRetirement(WriteOperation& op);
  /// 发布请求并通知后端，BLOCK 等待完成 / Publish, notify, and wait if BLOCK.
  [[nodiscard]] ErrorCode CommitQueued(size_t size, WriteOperation& op, bool in_isr);
  /// 通知读端，释放写入权，再完成写操作 / Notify the reader, release, then complete.
  [[nodiscard]] ErrorCode CommitAdmission(size_t size, WriteOperation& op, bool in_isr);
  /// 等待完成，超时不撤销数据 / Wait for completion; timeout leaves data queued.
  [[nodiscard]] ErrorCode WaitForBlock(WriteOperation& op);
  /// 释放 LOCKED，保留已发布请求数 / Release LOCKED while preserving the released count.
  void ReleaseProducer(Phase next, bool in_isr);
  /// 一次 CAS 同时释放生产者并发布一个请求 / Release the producer and publish in one CAS.
  void PublishQueuedRequest(Phase next, bool in_isr);
  /// 减少已发布请求数，保留阶段 / Decrement the released count, preserving the phase.
  void DecrementReleasedRequest(bool in_isr);
  /// 在当前调用上下文通知后端推进写入 / Notify backend progress in the current context.
  void NotifyBackend(bool in_isr);
  /// 更新剩余量，队头结束时移除并完成 / Account progress; retire a finished front.
  void SettleWriteQueue(size_t accepted, ErrorCode result, bool in_isr) noexcept;
  /// 完成请求或清退超时 BLOCK / Complete a request or retire a detached BLOCK.
  void CompleteRequest(Request& request, ErrorCode result, bool in_isr);

  friend class Pipe;

 public:
  /**
   * @brief 已提交队头的出队接口 / Consumer interface for one released front.
   *
   * 每个对象只对应一个请求的剩余部分，最多调用一次 PopAll、PopWithWriter 或 FailFront。
   * 析构时记录本次进展；整个请求结束后才移除请求并发送完成通知。继续处理须重新获取接口。
   * Each object covers one request remainder and permits at most one PopAll,
   * PopWithWriter, or FailFront call. Destruction accounts progress and retires and
   * completes the request only when its entire remainder is consumed. Obtain a new
   * interface for further progress.
   *
   * @pre 后端须串行化从获取接口到析构及完成回调返回的全过程，不可嵌套出队。
   *      The backend serializes acquisition through destruction and completion
   *      callbacks; consumer scopes must not overlap or nest.
   * @note 接收的数据须在出队方法返回前进入后端可持续持有的存储。析构可能同步执行
   *       用户回调；后端须保留并再次处理回调中新增写入的通知。
   *       Accepted bytes must reach stable backend storage before the consuming
   *       method returns. Destruction may invoke user callbacks inline; the backend
   *       must retain and recheck progress notifications from callback-generated writes.
   */
  class WriteQueue
  {
   public:
    WriteQueue(const WriteQueue&) = delete;
    WriteQueue& operator=(const WriteQueue&) = delete;
    WriteQueue(WriteQueue&&) = delete;
    WriteQueue& operator=(WriteQueue&&) = delete;
    /// 结算出队，使用获取时的上下文 / Settle progress using the captured context.
    ~WriteQueue() noexcept;

    /**
     * @brief 获取当前队头的剩余长度 / Get the current front remainder.
     * @return 剩余字节数，不含后续请求 / Remaining bytes, excluding later requests.
     */
    [[nodiscard]] size_t AvailableSize() const
    {
      DEV_ASSERT_FROM_CALLBACK(popped_size_ <= front_size_, in_isr_);
      return front_size_ - popped_size_;
    }

    /**
     * @brief 检查本接口是否还有数据 / Check whether this interface has remaining data.
     * @return 当前剩余长度为零时返回 true，不代表整个端口队列为空。
     *         True for a zero remainder; does not describe the entire port queue.
     */
    [[nodiscard]] bool Empty() const { return AvailableSize() == 0U; }

    /**
     * @brief 复制并取走队头剩余数据 / Copy and consume the entire front remainder.
     * @param destination 后端接收地址，至少容纳 AvailableSize() 字节。
     *        Backend destination with room for at least AvailableSize() bytes.
     * @pre 当前队头非空，地址有效，本接口尚未执行出队操作。
     *      A nonempty front, valid destination, and no previous action are required.
     * @note 返回时字节已出队；请求在本接口析构时完成。参数和队列一致性由后端保证。
     *       Bytes leave the queue before return; request completion occurs at interface
     *       destruction. Arguments and queue consistency are backend invariants.
     */
    void PopAll(uint8_t* destination);

    /**
     * @brief 通过回调取走队头前缀 / Consume a front prefix through a callback.
     * @tparam Writer 后端接收函数类型 / Backend writer callback type.
     * @param limit 本次最多提供的字节数 / Maximum number of bytes to offer.
     * @param writer 签名为 size_t(const uint8_t*, size_t, const uint8_t*, size_t)，
     *        接收按 FIFO 顺序排列的最多两段数据，返回已接收的连续前缀长度。
     *        Callable as size_t(const uint8_t*, size_t, const uint8_t*, size_t); receives
     *        up to two FIFO-ordered spans and returns the accepted prefix length.
     * @return 实际出队字节数；零表示本次无进展，保留原队头。
     *         Bytes actually consumed; zero leaves the front unchanged.
     * @pre 队头非空，本接口尚未执行出队操作；回调不可重入出队路径，返回值不得超过
     *      提供的字节数。只有已复制或转移到稳定后端存储的字节才可计为接收。
     *      Requires a nonempty front and no previous action. The callback must not
     *      reenter consumption on this port or return more than offered. Count only
     *      bytes copied or transferred to stable backend storage.
     * @note 提供长度不超过 limit 和当前队头剩余长度。非空时调用一次回调，数据指针
     *       仅在回调内有效。部分出队不完成请求；暂时无法接收时由后端安排后续推进。
     *       The offer is bounded by limit and the front remainder. The callback runs
     *       once for a positive offer; pointers expire on return. Partial consumption
     *       does not complete the request; the backend arranges later progress.
     */
    template <typename Writer>
    size_t PopWithWriter(size_t limit, Writer&& writer)
    {
      BeginAction();
      const size_t offered = std::min(limit, AvailableSize());
      if (offered == 0U)
      {
        return 0U;
      }

      const size_t accepted = port_.queue_data_->ConsumeWithReader(
          offered,
          [&](const uint8_t* first, size_t first_size, const uint8_t* second,
              size_t second_size) -> size_t
          {
            const size_t accepted = writer(first, first_size, second, second_size);
            ASSERT_FROM_CALLBACK(accepted <= first_size + second_size, in_isr_);
            return accepted;
          });
      popped_size_ += accepted;
      return accepted;
    }

    /**
     * @brief 丢弃剩余数据并结束队头 / Discard the remainder and fail the front.
     * @param reason 请求的完成错误，不可为 OK / Completion error, which must not be OK.
     * @pre 队头非空，本接口尚未执行出队操作；后端已确认该请求无法安全继续或重发。
     *      A nonempty front and no previous action; the backend has established that
     *      this request cannot safely continue or be replayed.
     * @note 用于部分传输后发生不可恢复错误的请求。后续请求保留，析构时完成当前请求。
     *       Applies to unrecoverable partial transfers. Later requests remain queued;
     *       the current request completes on destruction.
     */
    void FailFront(ErrorCode reason);

   private:
    friend class WritePort;

    /// 检查并记录本接口唯一一次出队操作 / Check and record the single permitted action.
    void BeginAction()
    {
      ASSERT_FROM_CALLBACK(!action_used_, in_isr_);
      action_used_ = true;
    }

    /// 保存当前队头剩余量与出队上下文 / Capture the front remainder and consumer context.
    WriteQueue(WritePort& port, bool in_isr, size_t front_size)
        : port_(port), in_isr_(in_isr), front_size_(front_size)
    {
    }

    WritePort& port_;          ///< 所属写端口 / Associated write port.
    const bool in_isr_;        ///< 获取接口时的中断上下文 / Captured ISR context.
    const size_t front_size_;  ///< 获取时的队头剩余字节数 / Initial front remainder.
    /// 本次接收或丢弃的字节数 / Bytes accepted or discarded in this scope.
    size_t popped_size_ = 0U;
    ErrorCode settlement_result_ = ErrorCode::OK;  ///< 本次结束结果 / Settlement result.
    /// 是否已调用出队方法，包括零进展 / Whether an action was used, even if zero.
    bool action_used_ = false;
  };

  /**
   * @brief 合并多次写入的流接口 / Stream interface for batched writes.
   *
   * 取得写入权后，各次 Write 直接复制到端口队列；Commit 或析构提交整个批次。
   * 普通后端在提交后才可消费本批次，Pipe 读端可在提交前读到已追加的字节。
   * Once acquired, each Write copies into the port queue. Commit or destruction
   * submits the batch. Queued backends consume it only after submission; Pipe readers
   * may consume appended bytes before submission.
   *
   * @pre 仅在线程中使用。同一 Stream 的成员调用及析构不得并发或重入；BLOCK 与
   *      非 BLOCK 的混用限制同 WritePort。持有写入权期间端口和后端须保持有效。
   *      Task-only. Calls and destruction on one Stream must not overlap or reenter.
   *      WritePort's BLOCK/non-BLOCK rules apply. Keep the port and backend alive
   *      while the Stream owns producer access, including between member calls.
   * @note 提交后可再次 Acquire 或 Write 开始新批次；各批次沿用构造时的通知对象，
   *       调用者须保证它在相关操作结束前有效。所有通知使用 in_isr = false。
   *       After submission, Acquire or Write can start a new batch using the same
   *       completion targets; keep them valid through the related operations.
   *       Notifications use in_isr = false.
   */
  class Stream
  {
   public:
    /**
     * @brief 构造流并尝试取得写入权 / Construct a stream and try to acquire the port.
     * @param port 目标写端口 / Target write port.
     * @param op 每个批次使用的完成通知描述符 / Completion descriptor used for each batch.
     * @note 构造不报告 Acquire 失败；可显式调用 Acquire 检查或重试。
     *       Construction hides acquisition failure; call Acquire to check or retry.
     */
    Stream(WritePort* port, WriteOperation op);

    /**
     * @brief 提交剩余批次并结束流 / Submit the remaining batch and destroy the stream.
     * @note 持有写入权时行为同 Commit，包括空批次通知和 BLOCK 等待，但无法返回结果。
     *       尚未取得写入权时不执行操作。
     *       While owning the port, behaves like Commit, including empty completion and
     *       BLOCK waits, but discards the result. Does nothing without ownership.
     */
    ~Stream();

    Stream(const Stream&) = delete;
    Stream& operator=(const Stream&) = delete;
    Stream(Stream&&) = delete;
    Stream& operator=(Stream&&) = delete;

    /**
     * @brief 向当前批次追加数据 / Append data to the current batch.
     * @param data 数据源和字节数；正长度时地址须有效，返回后即可复用源缓冲区。
     *        Source and byte count; a positive size requires a valid address. The source
     *        buffer may be reused after return.
     * @return 全部追加返回 OK；空间不足返回 FULL，本次不部分追加，保留已有批次。
     *         无可用端口返回 NOT_SUPPORT；需要取得写入权时也可能返回 BUSY 或 FULL。
     *         OK if fully appended; FULL leaves this input unwritten and prior appends
     *         intact. NOT_SUPPORT without a usable port; Acquire may return BUSY or FULL.
     * @note 必要时先 Acquire。零长度在能力检查后直接返回 OK，不取得写入权或完成操作。
     *       Acquires if needed. Zero size returns OK after the capability check without
     *       acquiring producer access or completing an operation.
     */
    [[nodiscard]] ErrorCode Write(ConstRawData data);

    /**
     * @brief 向当前批次追加文本 / Append text to the current batch.
     * @param text 待复制的文本，不附加结束符 / Text to copy without an added terminator.
     * @return 同 Write(ConstRawData) / Same results as Write(ConstRawData).
     */
    [[nodiscard]] ErrorCode Write(std::string_view text)
    {
      return Write(ConstRawData{text.data(), text.size()});
    }

    /**
     * @brief 尝试追加数据并返回流引用 / Try to append data and return the stream.
     * @param data 待追加的数据 / Data to append.
     * @return 当前流，可用于连续追加 / This stream for chained appends.
     * @note 取得写入权失败或空间不足时不追加，也不报告错误；需要结果时使用 Write。
     *       Skips the input without reporting acquisition or capacity failures; use
     *       Write when the result is needed.
     */
    Stream& operator<<(const ConstRawData& data);

    /**
     * @brief 提交批次并释放写入权 / Submit the batch and release producer access.
     * @return 非 BLOCK 提交返回 OK；普通后端的 BLOCK 返回完成结果或 TIMEOUT。
     *         Pipe 入队后返回 OK。未持有写入权时返回 OK，不执行操作。
     *         Non-BLOCK submission returns OK; queued BLOCK returns completion or
     *         TIMEOUT. Pipe returns OK on admission. Without ownership, returns OK
     *         without taking action.
     * @note 空批次先释放写入权，再完成非 BLOCK 通知；不通知后端，BLOCK 不等待。
     *       非空批次的完成与超时语义同 WritePort::operator()，可能同步调用用户回调。
     *       An empty batch releases ownership before non-BLOCK completion; it does
     *       not notify the backend or wait for BLOCK. Nonempty batches follow
     *       WritePort::operator() rules and may invoke callbacks inline.
     */
    [[nodiscard]] ErrorCode Commit();

    /**
     * @brief 尝试取得端口写入权 / Try to acquire the port's producer access.
     * @return 成功或已持有返回 OK；端口为空返回 PTR_NULL，不支持写入返回 NOT_SUPPORT，
     *         生产者状态被占用返回 BUSY，请求队列无空位返回 FULL。
     *         OK when acquired or already owned; PTR_NULL for a null port, NOT_SUPPORT
     *         when unavailable, BUSY for an occupied phase, FULL for no request slot.
     * @note 取得后保持独占，直到 Commit 或析构。只检查请求槽，不预留字节空间；
     *       不等待旧超时 BLOCK 请求退出，即使本 Stream 使用 BLOCK 也立即返回。
     *       Holds exclusive access until Commit or destruction. Checks a request slot
     *       without reserving byte capacity. Never waits for detached BLOCK retirement,
     *       even when this Stream uses a BLOCK operation.
     */
    [[nodiscard]] ErrorCode Acquire();

    /**
     * @brief 获取当前批次可追加的空间 / Get current free space for appending.
     * @return 持有写入权时返回字节队列当前空闲量，否则为零。
     *         Current free bytes while owning the port, or zero otherwise.
     * @note 查询不预留空间；后端或 Pipe 读端消费数据后可释放更多空间。
     *       Reserves no space; backend or Pipe reader consumption can free more capacity.
     */
    [[nodiscard]] size_t EmptySize() const
    {
      return owns_port_ ? port_->queue_data_->EmptySize() : 0U;
    }

   private:
    /// 清空批次记账后提交 / Clear batch bookkeeping, then submit via the port.
    [[nodiscard]] ErrorCode SubmitBuffered();
    /// 释放无待提交数据时的写入权 / Release producer access when no batch bytes remain.
    void Release();
    /// 完成空批次的非 BLOCK 通知 / Complete an empty non-BLOCK batch.
    void CompleteEmpty();

    WritePort* port_ = nullptr;  ///< 借用的写端口 / Borrowed write port.
    WriteOperation op_;  ///< 每批次使用的通知描述符 / Per-batch completion descriptor.
    /// 本批次累计追加字节数 / Total bytes appended in this batch.
    size_t buffered_size_ = 0U;
    /// 是否持有 LOCKED 写入权 / Whether this stream owns LOCKED.
    bool owns_port_ = false;
  };

  /**
   * @brief 构造写端口并分配队列 / Construct a write port and allocate its queues.
   * @param queue_size 请求队列容量；正数采用后端逐请求完成模式，零采用 Pipe 使用的
   *        入队即完成模式，不分配请求队列。最大值为 2^29 - 1。
   *        Request capacity; positive selects queued completion, zero selects the
   *        admission completion used by Pipe, with no metadata. Maximum: 2^29 - 1.
   * @param buffer_size 字节队列容量；零表示不分配字节存储。
   *        Byte queue capacity; zero allocates no payload storage.
   * @note 使用前须绑定 WriteFun。模式由请求队列是否存在决定，构造后保持不变。
   *       Bind WriteFun before use. Metadata presence fixes the mode at construction.
   */
  WritePort(size_t queue_size = 3U, size_t buffer_size = 128U);

  /**
   * @brief 获取字节队列当前空闲空间 / Get current byte queue free space.
   * @return 空闲字节数，无字节队列时为零；不反映请求槽或生产者是否可用。
   *         Free bytes, or zero without a queue; no metadata or phase guarantee.
   * @note 查询不预留空间，并发进展可使结果变化。
   *       Reserves no space; concurrent progress may change the value.
   */
  [[nodiscard]] size_t EmptySize() const;

  /**
   * @brief 获取字节队列当前数据量 / Get current byte queue occupancy.
   * @return 队列字节数，包含 Stream 已追加但尚未提交的字节；无队列时为零。
   *         Queued bytes, including uncommitted Stream appends, or zero without a queue.
   * @note 已交给后端存储的字节不计入此值；并发进展可使结果变化。
   *       Excludes backend-owned bytes; concurrent progress may change the value.
   */
  [[nodiscard]] size_t Size() const;

  /**
   * @brief 获取字节队列总容量 / Get total byte queue capacity.
   * @return 总字节数，无队列时为零 / Capacity in bytes, or zero without a queue.
   */
  [[nodiscard]] size_t Capacity() const;

  /**
   * @brief 检查端口是否支持写入 / Check whether the port supports writes.
   * @return 存在字节队列且已绑定 WriteFun 时为 true，不保证下一次写入可接纳。
   *         True with a byte queue and bound WriteFun; does not guarantee admission.
   */
  [[nodiscard]] bool Writable() const;

  /**
   * @brief 绑定写入进度通知函数 / Bind the write progress notification function.
   * @param fun 接收端口和当前中断上下文的通知函数，不返回请求完成结果。
   *        Notification receiving the port and ISR context, without a completion result.
   * @return 当前端口 / This port.
   * @pre 仅在端口或后端构造时绑定一次，运行前完成；Pipe 由自身构造函数绑定。
   *      Bind once at construction, before use. Pipe binds its own callback.
   * @note 普通后端通过 GetWriteQueue 消费已提交请求，并负责保留、重新检查进度通知。
   *       Queued backends consume released requests through GetWriteQueue and retain
   *       and recheck progress notifications.
   * @note Pipe 在持有写入权时通知读端，通知返回后释放写入权，再完成写操作。
   *       Pipe notifies its reader while holding producer access, then releases
   *       access before completing the write.
   */
  WritePort& operator=(WriteFun fun);

  /**
   * @brief 获取已提交队头的出队接口
   *        / Obtain a consumer interface for the released front.
   * @param in_isr 当前是否在中断中；本次析构结算和完成通知使用该上下文。
   *        ISR context, captured for settlement and completion notifications.
   * @return 当前队头剩余部分的接口，无已发布请求时返回空接口。
   *         Interface for the current front remainder, or empty when none is released.
   * @pre 仅供具有请求队列的后端使用，由唯一消费者串行调用；Pipe 不使用此接口。
   *      Requires a metadata queue and a serialized sole consumer; not used by Pipe.
   * @note 后端可在生产者准备新请求时继续消费较早的已发布请求。
   *       Older released requests remain consumable while a producer prepares a new one.
   */
  [[nodiscard]] WriteQueue GetWriteQueue(bool in_isr = false);

  /**
   * @brief 提交写请求 / Submit a write request.
   * @param data 数据源和字节数；正长度地址须有效，接纳时在本次调用内完整复制到队列，
   *        返回后即可复用源缓冲区。
   *        零长度允许空地址，在能力检查后立即成功，不通知后端，非 BLOCK 同步完成。
   *        Source and byte count; admitted data is fully copied during this call.
   *        A positive size requires a valid address; the source is reusable after return.
   *        Zero size permits null and succeeds after the capability check, without
   *        backend notification, completing non-BLOCK inline.
   * @param op 完成通知描述符；回调和轮询状态须保持到完成，BLOCK 信号量须保持到返回。
   *        Completion descriptor; keep callback and polling targets through completion
   *        and the BLOCK semaphore through return.
   * @param in_isr 当前调用是否位于中断 / Whether this call is in an ISR.
   * @return 非 BLOCK 的 OK 表示请求已接纳；普通后端的 BLOCK 返回完成结果或 TIMEOUT。
   *         Pipe 入队后返回 OK，无需等待读端消费。无写入能力返回 NOT_SUPPORT，
   *         生产者状态冲突返回 BUSY，字节空间或请求槽不足返回 FULL，不部分接纳。
   *         Non-BLOCK OK means admission; queued BLOCK returns completion or TIMEOUT.
   *         Pipe returns OK on admission without waiting for reads. NOT_SUPPORT when
   *         unavailable, BUSY for a phase conflict, FULL for insufficient byte space or
   *         request slots, without partial admission.
   * @pre BLOCK 仅在线程中调用，每个信号量只服务一个活动 BLOCK 调用；遵守类说明中的
   *      BLOCK 与非 BLOCK 互斥约定。通知对象和端口须在仍可能访问它们的调用结束前有效。
   *      BLOCK is task-only with a dedicated semaphore for one live call. Follow the
   *      class-level BLOCK/non-BLOCK exclusion and keep targets and the port alive
   *      through calls that may still access them.
   * @note BLOCK 不等待普通队列空间。若旧 BLOCK 已超时且本次长度不超过 Capacity()，
   *       可先等旧请求退出，再提交并等待本次完成；两次等待分别使用原 timeout。
   *       第一段等待超时不提交本次数据；第二段超时不撤销已排队数据，后端仍可发送。
   *       BLOCK does not wait for ordinary queue space. A later request no larger than
   *       Capacity() may first await a timed-out BLOCK's retirement, then submit and
   *       await its own completion; each wait uses the original timeout. A retirement
   *       timeout submits no new data; a completion timeout leaves queued data intact.
   * @note 完成方已认领本次 BLOCK 时，超时方须等信号量交接并返回实际完成结果；
   *       因此调用可能超过指定时间。超时返回后，旧请求不会再访问该调用的信号量。
   *       If completion is already claimed, the caller waits for the semaphore handoff
   *       and returns the actual result, possibly exceeding the timeout. A detached
   *       request no longer accesses the timed-out caller's semaphore.
   * @note 非 BLOCK 回调在实际完成的上下文执行，可能发生在本次调用内；可提交后续
   *       非 BLOCK 写入，其结果仍受生产者占用与容量限制。轮询状态以 acquire 读取。
   *       拒绝接纳的请求不发送完成通知；零长度和 Pipe 的 BLOCK 不等待或释放信号量。
   *       Non-BLOCK callbacks run in the completion context, possibly inline, and may
   *       submit further non-BLOCK writes subject to admission and capacity. Load
   *       polling status with acquire ordering. Rejected requests emit no completion;
   *       zero-size and Pipe BLOCK writes neither wait nor post a semaphore.
   * @note 回调不可阻塞等待依赖当前调用返回才能推进的操作。
   *       Callbacks must not block on progress requiring the current call to return.
   */
  ErrorCode operator()(ConstRawData data, WriteOperation& op, bool in_isr = false);

 private:
  /// 普通后端的请求队列；Pipe 入队即完成时为空 / Metadata queue; null for Pipe admission.
  SPSCQueue<Request>* queue_requests_;
  /// 写端口分配、Pipe 读端借用的字节队列 / Allocated queue, borrowed by a Pipe reader.
  SPSCQueue<uint8_t>* queue_data_;
  /// 构造时绑定的后端进度通知 / Backend progress notification bound during construction.
  WriteFun write_fun_ = nullptr;

  /// 阶段与已发布 FIFO 前缀请求数 / Phase and released FIFO prefix count.
  /// Pipe 的请求计数恒为零 / Pipe keeps the request count at zero.
  std::atomic<uint32_t> state_{MakeState(Phase::IDLE, 0U)};
  /// Post 前写入，Wait 成功后读取 / Written before Post, read after a successful Wait.
  ErrorCode block_result_ = ErrorCode::OK;
  /// 唯一后端消费者维护的剩余量 / Remainder maintained by the sole backend consumer.
  size_t front_remaining_ = 0U;
  /// 后续 BLOCK 的信号量 / Later BLOCK waiter's semaphore.
  /// 通过 BLOCK_RETIRE_WAITING 阶段发布 / Published through BLOCK_RETIRE_WAITING.
  Semaphore* admission_waiter_ = nullptr;
};

}  // namespace LibXR
