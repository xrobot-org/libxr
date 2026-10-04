#pragma once

#include <cstddef>
#include <type_traits>

#include "queue_typed_base.hpp"
#include "spsc_queue_base.hpp"

namespace LibXR
{
/**
 * @class SPSCQueue
 * @brief 单生产者单消费者无锁队列 / Single-producer single-consumer lock-free queue.
 *
 * 模板壳只把 `Data` 映射为固定大小的字节 payload 并复用 `SPSCQueueBase`，
 * 不在队列内部管理 `Data` 对象生命周期。调用方必须保证 payload 可以按该
 * 项目的队列契约进行字节搬运。
 *
 * This template wrapper maps `Data` to a fixed-size byte payload and reuses
 * `SPSCQueueBase`. It does not manage `Data` object lifetime inside the queue.
 * The caller must ensure that the payload is valid for this project's
 * byte-moving queue contract.
 *
 * @tparam Data 队列存储的数据类型 / Queue element type.
 */
template <typename Data>
class SPSCQueue final : public QueueTypedBase<SPSCQueue<Data>, Data>, public SPSCQueueBase
{
 public:
  static_assert(alignof(Data) <= alignof(std::max_align_t),
                "SPSCQueue does not support over-aligned payload types");

  using ValueType = Data;  ///< 队列元素类型 / Queue element type.
  /// @brief 重新公开强类型出队接口 / Re-expose the typed pop interface.
  using QueueTypedBase<SPSCQueue<Data>, Data>::Pop;
  /// @brief 重新公开强类型入队接口 / Re-expose the typed push interface.
  using QueueTypedBase<SPSCQueue<Data>, Data>::Push;

  /**
   * @brief 构造一个 SPSC 队列 / Construct one SPSC queue.
   * @param length 队列容量 / Queue capacity.
   *
   * @note 包含动态内存分配 / Contains dynamic memory allocation.
   */
  explicit SPSCQueue(size_t length) : SPSCQueueBase(sizeof(Data), alignof(Data), length)
  {
  }

  /**
   * @brief 析构一个 SPSC 队列 / Destroy one SPSC queue.
   */
  ~SPSCQueue() = default;

  /**
   * @brief 查看一个队头 payload 但不出队 / Peek one front payload without dequeuing it.
   * @param item 用于接收 payload / Receives the peeked payload.
   * @return 成功返回 `ErrorCode::OK`；队列空返回 `ErrorCode::EMPTY`
   *         Returns `ErrorCode::OK` on success; returns `ErrorCode::EMPTY` when
   *         the queue is empty
   */
  ErrorCode Peek(Data& item) { return SPSCQueueBase::PeekBytes(&item); }

  /**
   * @brief 批量推入多个 payload / Push multiple payloads into the queue.
   * @param data payload 数组指针 / Pointer to the payload array.
   * @param size payload 个数 / Number of payloads.
   * @return 成功返回 `ErrorCode::OK`；队列满返回 `ErrorCode::FULL`
   *         Returns `ErrorCode::OK` on success; returns `ErrorCode::FULL` when
   *         the queue is full
   */
  ErrorCode PushBatch(const Data* data, size_t size)
  {
    return SPSCQueueBase::PushBatchBytes(data, size);
  }

  /**
   * @brief 通过写入器回调推入一个 payload / Push one payload via a writer callback.
   * @tparam Writer 写入器类型 / Writer callback type.
   * @param writer 写入器回调，签名为 `ErrorCode(Data* buffer, size_t count)`
   *        Writer callback with signature `ErrorCode(Data* buffer, size_t count)`.
   * @return 成功返回 `ErrorCode::OK`；队列满返回 `ErrorCode::FULL`；否则返回写入器错误码
   *         Returns `ErrorCode::OK` on success; returns `ErrorCode::FULL` when the
   *         queue is full; otherwise returns the writer error code
   *
   * @note 回调中的 `count` 固定为 1，成功写入后才提交到队列。
   *       The callback always receives `count == 1`; the payload is committed
   *       only after the writer succeeds.
   */
  template <typename Writer>
  ErrorCode PushWithWriter(Writer&& writer)
  {
    return PushWithWriter(1, std::forward<Writer>(writer));
  }

  /**
   * @brief 通过写入器回调批量推入 payload / Push multiple payloads via a writer callback.
   * @tparam Writer 写入器类型 / Writer callback type.
   * @param size payload 个数 / Number of payloads.
   * @param writer 写入器回调，签名为 `ErrorCode(Data* buffer, size_t count)`。
   *        Writer callback with signature `ErrorCode(Data* buffer, size_t count)`.
   * @return 成功返回 `ErrorCode::OK`；队列满返回
   * `ErrorCode::FULL`；否则返回写入器错误码。
   * Returns `ErrorCode::OK` on success; returns `ErrorCode::FULL` when the queue is full;
   * otherwise returns the writer error code.
   *
   * @note 仅当写入器处理完整批次后才提交入队。
   *       The enqueue is committed only after the writer accepts the full batch.
   */
  template <typename Writer>
  ErrorCode PushWithWriter(size_t size, Writer&& writer)
  {
    static_assert(
        std::is_trivially_copyable_v<Data>,
        "batched SPSCQueue::PushWithWriter requires trivially copyable payloads");
    static_assert(
        std::is_trivially_destructible_v<Data>,
        "batched SPSCQueue::PushWithWriter requires trivially destructible payloads");
    static_assert(std::is_invocable_v<Writer&, Data*, size_t>,
                  "PushWithWriter writer must be callable as "
                  "ErrorCode(Data* buffer, size_t count)");
    using WriterRet = std::invoke_result_t<Writer&, Data*, size_t>;
    static_assert(std::is_convertible_v<WriterRet, ErrorCode>,
                  "PushWithWriter writer return type must be convertible to ErrorCode");

    Writer& writer_ref = writer;
    return SPSCQueueBase::PushBytesWithWriter(
        size, [&](void* buffer, size_t count) -> ErrorCode
        { return writer_ref(static_cast<Data*>(buffer), count); });
  }

  /**
   * @brief 通过一次双 span 回调生产 payload 前缀 / Produce a payload prefix through one
   *        two-span writer callback.
   * @tparam Writer 写入器类型 / Writer callback type.
   * @param limit 最多提供的 payload 个数 / Maximum number of offered payloads.
   * @param writer 签名为 `size_t(Data*, size_t, Data*, size_t)`，返回已写入的前缀个数 /
   *        Callback with signature `size_t(Data*, size_t, Data*, size_t)`; returns the
   *        written prefix count.
   * @return 实际发布到队列的 payload 个数 / Number of payloads published to the queue.
   * @pre 队列只有一个 producer；回调期间不得重入同侧操作或 Reset /
   *      The queue has one producer; no same-side operation or Reset during the callback.
   * @note 没有可用空间或 `limit == 0` 时不调用回调；回调指针只在调用期间有效 /
   *       The callback is skipped when no space is available or `limit == 0`; pointers
   *       are valid only during the callback.
   */
  template <typename Writer>
  size_t ProduceWithWriter(size_t limit, Writer&& writer)
  {
    static_assert(std::is_trivially_copyable_v<Data>);
    static_assert(std::is_trivially_destructible_v<Data>);
    return SPSCQueueBase::ProduceWithWriter(
        limit,
        [&](void* first, size_t first_size, void* second, size_t second_size) -> size_t
        {
          return writer(static_cast<Data*>(first), first_size, static_cast<Data*>(second),
                        second_size);
        });
  }

  /**
   * @brief 通过读取器回调弹出一个 payload / Pop one payload via a reader callback.
   * @tparam Reader 读取器类型 / Reader callback type.
   * @param reader 读取器回调，签名为 `ErrorCode(const Data* buffer, size_t count)`
   *        Reader callback with signature `ErrorCode(const Data* buffer, size_t count)`.
   * @return 成功返回 `ErrorCode::OK`；队列空返回 `ErrorCode::EMPTY`；否则返回读取器错误码
   *         Returns `ErrorCode::OK` on success; returns `ErrorCode::EMPTY` when the
   *         queue is empty; otherwise returns the reader error code
   *
   * @note 回调中的 `count` 固定为 1，读取器成功后才提交出队。
   *       The callback always receives `count == 1`; the pop is committed only
   *       after the reader succeeds.
   */
  template <typename Reader>
  ErrorCode PopWithReader(Reader&& reader)
  {
    return PopWithReader(1, std::forward<Reader>(reader));
  }

  /**
   * @brief 通过读取器回调批量弹出 payload / Pop multiple payloads via a reader callback.
   * @tparam Reader 读取器类型 / Reader callback type.
   * @param size payload 个数 / Number of payloads.
   * @param reader 读取器回调，签名为 `ErrorCode(const Data* buffer, size_t count)`。
   *        Reader callback with signature `ErrorCode(const Data* buffer, size_t count)`.
   * @return 成功返回 `ErrorCode::OK`；队列空返回
   * `ErrorCode::EMPTY`；否则返回读取器错误码。
   * Returns `ErrorCode::OK` on success; returns `ErrorCode::EMPTY` when the queue is
   * empty; otherwise returns the reader error code.
   *
   * @note 仅当读取器处理完整批次后才提交出队。
   *       The pop is committed only after the reader accepts the full batch.
   */
  template <typename Reader>
  ErrorCode PopWithReader(size_t size, Reader&& reader)
  {
    static_assert(
        std::is_trivially_copyable_v<Data>,
        "batched SPSCQueue::PopWithReader requires trivially copyable payloads");
    static_assert(
        std::is_trivially_destructible_v<Data>,
        "batched SPSCQueue::PopWithReader requires trivially destructible payloads");
    static_assert(std::is_invocable_v<Reader&, const Data*, size_t>,
                  "PopWithReader reader must be callable as "
                  "ErrorCode(const Data* buffer, size_t count)");
    using ReaderRet = std::invoke_result_t<Reader&, const Data*, size_t>;
    static_assert(std::is_convertible_v<ReaderRet, ErrorCode>,
                  "PopWithReader reader return type must be convertible to ErrorCode");

    Reader& reader_ref = reader;
    return SPSCQueueBase::PopBytesWithReader(
        size, [&](const void* buffer, size_t count) -> ErrorCode
        { return reader_ref(static_cast<const Data*>(buffer), count); });
  }

  /**
   * @brief 通过一次双 span 回调消费 payload 前缀 / Consume a payload prefix through one
   *        two-span reader callback.
   * @tparam Reader 读取器类型 / Reader callback type.
   * @param limit 最多提供的 payload 个数 / Maximum number of offered payloads.
   * @param reader 签名为 `size_t(const Data*, size_t, const Data*, size_t)`，返回已接受的
   *        前缀个数 / Callback with signature `size_t(const Data*, size_t, const Data*,
   *        size_t)`; returns the accepted prefix count.
   * @return 实际从队列移除的 payload 个数 / Number of payloads removed from the queue.
   * @pre 队列只有一个 consumer；回调期间不得重入同侧操作或 Reset /
   *      The queue has one consumer; no same-side operation or Reset during the callback.
   * @note 没有可用数据或 `limit == 0` 时不调用回调；回调指针只在调用期间有效 /
   *       The callback is skipped when no data is available or `limit == 0`; pointers are
   *       valid only during the callback.
   */
  template <typename Reader>
  size_t ConsumeWithReader(size_t limit, Reader&& reader)
  {
    static_assert(std::is_trivially_copyable_v<Data>);
    static_assert(std::is_trivially_destructible_v<Data>);
    return SPSCQueueBase::ConsumeWithReader(
        limit,
        [&](const void* first, size_t first_size, const void* second,
            size_t second_size) -> size_t
        {
          return reader(static_cast<const Data*>(first), first_size,
                        static_cast<const Data*>(second), second_size);
        });
  }

  /**
   * @brief 批量弹出多个 payload / Pop multiple payloads from the queue.
   * @param data 用于接收 payload 的数组 / Array receiving dequeued payloads.
   * @param size payload 个数 / Number of payloads.
   * @return 成功返回 `ErrorCode::OK`；队列空返回 `ErrorCode::EMPTY`
   *         Returns `ErrorCode::OK` on success; returns `ErrorCode::EMPTY` when
   *         the queue does not contain enough payloads
   */
  ErrorCode PopBatch(Data* data, size_t size)
  {
    return SPSCQueueBase::PopBatchBytes(data, size);
  }

  /**
   * @brief 批量查看多个 payload 但不出队 / Peek multiple payloads without dequeuing them.
   * @param data 用于接收 payload 的数组 / Array receiving peeked payloads.
   * @param size payload 个数 / Number of payloads.
   * @return 成功返回 `ErrorCode::OK`；队列空返回 `ErrorCode::EMPTY`
   *         Returns `ErrorCode::OK` on success; returns `ErrorCode::EMPTY` when
   *         the queue does not contain enough payloads
   */
  ErrorCode PeekBatch(Data* data, size_t size)
  {
    return SPSCQueueBase::PeekBatchBytes(data, size);
  }

  /**
   * @brief 从 consumer 侧丢弃当前可见的全部 payload / Discard currently available
   *        payloads from the consumer side
   */
  void Reset() { SPSCQueueBase::Reset(); }
};
}  // namespace LibXR
