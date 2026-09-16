#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <new>
#include <type_traits>
#include <utility>

#include "libxr_def.hpp"
#include "queue.hpp"

namespace LibXR
{
/**
 * @brief 可作为对象池空闲索引队列的类型约束。
 * @brief Type constraint for free-index queues used by object pools.
 *
 * 该约束只要求队列提供最小的强类型接口：
 * - `ValueType`
 * - `Push(const ValueType&)`
 * - `Pop(ValueType&)`
 * - `Size()`
 *
 * This constraint requires only the minimal typed queue interface:
 * - `ValueType`
 * - `Push(const ValueType&)`
 * - `Pop(ValueType&)`
 * - `Size()`
 */
template <typename QueueType>
concept PoolIndexQueue =
    requires(QueueType queue, const typename QueueType::ValueType& index_const,
             typename QueueType::ValueType& index_mut) {
      typename QueueType::ValueType;
      { queue.Push(index_const) } -> std::same_as<ErrorCode>;
      { queue.Pop(index_mut) } -> std::same_as<ErrorCode>;
      { queue.Size() } -> std::convertible_to<size_t>;
    };

/**
 * @class BasicObjectPool
 * @brief 基于空闲索引队列的 RAII 槽池。
 * @brief RAII slot pool backed by a free-index queue.
 *
 * 成功申请建立一个共享引用；最后一个引用释放时归还索引，不析构或清空负载。
 * Handle 可写，ConstHandle 只读；计数不提供负载读写同步。运行操作不分配内存。
 * 每槽引用数不得超过 UINT32_MAX。池及外部槽存储必须比全部句柄活得更久。
 * 构造和析构需要静止状态，不能在 ISR 执行。同一句柄对象的并发访问须由调用方同步。
 *
 * Acquire creates one shared reference; final release returns the index without
 * destroying or clearing the payload. Handle permits writes, ConstHandle is
 * read-only; counting does not synchronize payload access. Runtime operations
 * allocate no memory. Each slot must have at most UINT32_MAX references. Pool and
 * external storage outlive all handles. Construction/destruction require quiescence
 * outside ISR. Concurrent operations on the same handle require synchronization.
 *
 * 同一池的申请不得并发或重入。普通队列需外部串行化队列访问；SPSC 只有一个最终归还方；
 * MPMC 允许并发归还。ISR 支持还取决于平台原子实现，不保证单次操作的固定耗时。
 * Acquisitions on a pool must not overlap or reenter. Ordinary queues require
 * externally serialized queue access; SPSC permits one final-return producer;
 * MPMC permits concurrent returns. ISR use requires suitable target atomics and
 * does not imply a fixed per-operation latency bound.
 *
 * @tparam Data 槽内对象类型。 Slot object type.
 * @tparam FreeQueue 空闲索引队列类型。 Free-index queue type.
 */
template <typename Data, PoolIndexQueue FreeQueue>
class BasicObjectPool
{
 public:
  using ValueType = Data;       ///< 槽内对象类型。 Slot object type.
  using QueueType = FreeQueue;  ///< 空闲索引队列类型。 Free-index queue type.
  using IndexType = typename FreeQueue::ValueType;  ///< 槽索引类型。 Slot index type.

  /// @brief 槽索引必须是整数类型。 Slot indices must use an integral type.
  static_assert(std::is_integral_v<IndexType>,
                "BasicObjectPool requires an integral index queue");
  /// @brief 槽索引必须是无符号整数类型。 Slot indices must use an unsigned integral type.
  static_assert(std::is_unsigned_v<IndexType>,
                "BasicObjectPool requires an unsigned index queue");

  /**
   * @brief 对象及其引用计数的常驻存储。 Resident payload and reference-count storage.
   * @note 外部存储须提供 Slot 数组，并保持到池及所有句柄使用结束。
   *       External Slot arrays must outlive pool use and all handles.
   */
  class Slot
  {
   public:
    /// @brief 默认构造负载。 Default-construct the payload.
    Slot()
      requires std::is_default_constructible_v<Data>
    = default;

    /// @brief 用参数原地构造负载。 Construct the payload in place from arguments.
    template <typename... Args>
      requires std::is_constructible_v<Data, Args...>
    explicit Slot(std::in_place_t, Args&&... args) : data_(std::forward<Args>(args)...)
    {
    }

    Slot(const Slot&) = delete;
    Slot& operator=(const Slot&) = delete;
    Slot(Slot&&) = delete;
    Slot& operator=(Slot&&) = delete;

   private:
    friend class BasicObjectPool;
    std::atomic<uint32_t> references_{0};
    Data data_;
  };

 private:
  /// @brief 同一共享所有权的访问限定实现。 Access-qualified shared ownership.
  template <bool IsConst>
  class BasicHandle
  {
    using AccessType = std::conditional_t<IsConst, const Data, Data>;

   public:
    /// @brief 构造空句柄。 Construct an empty handle.
    BasicHandle() = default;

    /// @brief 复制现有引用。 Retain an existing reference.
    BasicHandle(const BasicHandle& other) noexcept
        : pool_(other.pool_), index_(other.index_)
    {
      Retain();
    }

    /// @brief 复制为只读引用。 Copy a mutable reference into a read-only handle.
    template <bool OtherConst>
      requires(IsConst && !OtherConst)
    BasicHandle(const BasicHandle<OtherConst>& other) noexcept
        : pool_(other.pool_), index_(other.index_)
    {
      Retain();
    }

    /// @brief 转移引用并清空源句柄。 Transfer ownership and empty the source.
    BasicHandle(BasicHandle&& other) noexcept
        : pool_(std::exchange(other.pool_, nullptr)),
          index_(std::exchange(other.index_, IndexType{}))
    {
    }

    /// @brief 转移为只读引用。 Transfer mutable ownership into a read-only handle.
    template <bool OtherConst>
      requires(IsConst && !OtherConst)
    BasicHandle(BasicHandle<OtherConst>&& other) noexcept
        : pool_(std::exchange(other.pool_, nullptr)),
          index_(std::exchange(other.index_, IndexType{}))
    {
    }

    /// @brief 替换引用，先保留源再释放旧引用。 Retain source before releasing old
    /// ownership.
    BasicHandle& operator=(const BasicHandle& other) noexcept
    {
      if (pool_ != other.pool_ || index_ != other.index_)
      {
        BasicHandle copy(other);
        Swap(copy);
      }
      return *this;
    }

    /// @brief 从可写句柄复制赋值。 Copy-assign from a mutable handle.
    template <bool OtherConst>
      requires(IsConst && !OtherConst)
    BasicHandle& operator=(const BasicHandle<OtherConst>& other) noexcept
    {
      if (pool_ != other.pool_ || index_ != other.index_)
      {
        BasicHandle copy(other);
        Swap(copy);
      }
      return *this;
    }

    /// @brief 转移赋值并释放旧引用。 Move-assign and release previous ownership.
    BasicHandle& operator=(BasicHandle&& other) noexcept
    {
      if (this != &other)
      {
        BasicHandle moved(std::move(other));
        Swap(moved);
      }
      return *this;
    }

    /// @brief 从可写句柄转移赋值。 Move-assign from a mutable handle.
    template <bool OtherConst>
      requires(IsConst && !OtherConst)
    BasicHandle& operator=(BasicHandle<OtherConst>&& other) noexcept
    {
      BasicHandle moved(std::move(other));
      Swap(moved);
      return *this;
    }

    /// @brief 释放本引用；最后一个引用归还槽位。 Release; the final reference returns the
    /// slot.
    ~BasicHandle() { Reset(); }

    /// @brief 是否持有引用。 Whether this handle owns a reference.
    [[nodiscard]] bool Valid() const { return pool_ != nullptr; }

    /// @brief 按句柄权限访问负载，句柄必须有效。 Access payload; requires a valid handle.
    [[nodiscard]] AccessType& Get()
    {
      ASSERT(Valid());
      return pool_->UnsafeAt(index_);
    }

    /// @brief 保留旧接口的 const 只读访问。 Preserve const-qualified read-only access.
    [[nodiscard]] const Data& Get() const
    {
      ASSERT(Valid());
      return pool_->UnsafeAt(index_);
    }

    /// @brief 按句柄权限返回负载指针。 Return an access-qualified payload pointer.
    [[nodiscard]] AccessType* operator->() { return &Get(); }
    /// @brief 返回只读负载指针。 Return a read-only payload pointer.
    [[nodiscard]] const Data* operator->() const { return &Get(); }
    /// @brief 按句柄权限解引用。 Dereference with this handle's access qualification.
    [[nodiscard]] AccessType& operator*() { return Get(); }
    /// @brief 只读解引用。 Dereference read-only.
    [[nodiscard]] const Data& operator*() const { return Get(); }

    /// @brief 返回有效句柄的槽索引。 Return the slot index of a valid handle.
    [[nodiscard]] IndexType Index() const
    {
      ASSERT(Valid());
      return index_;
    }

    /// @brief 清空句柄并释放引用；空句柄无操作。 Empty and release; empty handles are a
    /// no-op.
    void Reset()
    {
      BasicObjectPool* pool = std::exchange(pool_, nullptr);
      const IndexType index = std::exchange(index_, IndexType{});
      if (pool != nullptr)
      {
        pool->Release(index);
      }
    }

   private:
    friend class BasicObjectPool;
    template <bool>
    friend class BasicHandle;

    BasicHandle(BasicObjectPool* pool, IndexType index) : pool_(pool), index_(index) {}

    void Retain() noexcept
    {
      if (pool_ != nullptr)
      {
        pool_->Retain(index_);
      }
    }

    void Swap(BasicHandle& other) noexcept
    {
      std::swap(pool_, other.pool_);
      std::swap(index_, other.index_);
    }

    BasicObjectPool* pool_ = nullptr;
    IndexType index_ = {};
  };

 public:
  /// @brief 可复制的可写共享句柄。 Copyable mutable shared handle.
  using Handle = BasicHandle<false>;
  /// @brief 可复制的只读共享句柄，只接受可写到只读转换。
  ///        Copyable read-only shared handle; conversion is mutable-to-const only.
  using ConstHandle = BasicHandle<true>;

  /**
   * @brief 用内部 queue 和内部 slots 构造 pool。
   * @brief Construct the pool with an internal queue and internal slots.
   * @param slot_count 槽位数量。 Number of slots.
   */
  template <typename T = Data>
    requires std::is_default_constructible_v<T>
  explicit BasicObjectPool(size_t slot_count) : slot_count_(slot_count)
  {
    ASSERT(slot_count_ > 0);
    ConstructOwnedQueue(slot_count_);
    AllocateOwnedSlots();
    InitializeFreeQueue();
  }

  /**
   * @brief 用内部 queue 和外部 slots 构造 pool。
   * @brief Construct the pool with an internal queue and external slots.
   * @param slot_count 槽位数量。 Number of slots.
   * @param slots 外部槽数组。 Caller-provided slot storage.
   *
   * @note 调用方必须保证 `slots` 指向至少 `slot_count` 个 `Slot` 槽位。
   *       The caller must ensure that `slots` points to at least `slot_count`
   *       `Slot` slots.
   */
  BasicObjectPool(size_t slot_count, Slot* slots) : slot_count_(slot_count), slots_(slots)
  {
    ASSERT(slot_count_ > 0);
    ASSERT(slots_ != nullptr);
    ConstructOwnedQueue(slot_count_);
    InitializeFreeQueue();
  }

  /**
   * @brief 用外部 queue 和内部 slots 构造 pool。
   * @brief Construct the pool with an external queue and internal slots.
   * @param free_queue 外部空闲索引队列。 Caller-provided free-index queue.
   * @param slot_count 槽位数量。 Number of slots.
   *
   * @note 调用方传入的 `free_queue` 必须是空队列，并且只供当前 pool 独占使用。
   *       The caller-provided `free_queue` must be empty and dedicated to this pool.
   * @note 调用方必须保证 `free_queue` 的容量至少为 `slot_count`。
   *       The caller must ensure that `free_queue` capacity is at least `slot_count`.
   */
  template <typename T = Data>
    requires std::is_default_constructible_v<T>
  BasicObjectPool(FreeQueue& free_queue, size_t slot_count)
      : free_queue_(&free_queue), slot_count_(slot_count)
  {
    ASSERT(slot_count_ > 0);
    ASSERT(free_queue.Size() == 0);
    AllocateOwnedSlots();
    InitializeFreeQueue();
  }

  /**
   * @brief 用外部 queue 和外部 slots 构造 pool。
   * @brief Construct the pool with an external queue and external slots.
   * @param free_queue 外部空闲索引队列。 Caller-provided free-index queue.
   * @param slot_count 槽位数量。 Number of slots.
   * @param slots 外部槽数组。 Caller-provided slot storage.
   *
   * @note 调用方传入的 `free_queue` 必须是空队列，并且只供当前 pool 独占使用。
   *       The caller-provided `free_queue` must be empty and dedicated to this pool.
   * @note 调用方必须保证 `free_queue` 的容量至少为 `slot_count`。
   *       The caller must ensure that `free_queue` capacity is at least `slot_count`.
   * @note 调用方必须保证 `slots` 指向至少 `slot_count` 个 `Slot` 槽位。
   *       The caller must ensure that `slots` points to at least `slot_count`
   *       `Slot` slots.
   */
  BasicObjectPool(FreeQueue& free_queue, size_t slot_count, Slot* slots)
      : free_queue_(&free_queue), slot_count_(slot_count), slots_(slots)
  {
    ASSERT(slot_count_ > 0);
    ASSERT(slots_ != nullptr);
    ASSERT(free_queue.Size() == 0);
    InitializeFreeQueue();
  }

  /**
   * @brief 析构对象池，并释放其拥有的内部资源。
   * @brief Destroy the object pool and release owned internal resources.
   */
  ~BasicObjectPool()
  {
    ASSERT(EmptySize() == slot_count_);

    if (owns_slots_)
    {
      delete[] slots_;
    }

    if (owns_free_queue_)
    {
      free_queue_->~FreeQueue();
    }
  }

  /// @brief 禁止拷贝构造。 Non-copyable.
  BasicObjectPool(const BasicObjectPool&) = delete;
  /// @brief 禁止拷贝赋值。 Non-copy-assignable.
  BasicObjectPool& operator=(const BasicObjectPool&) = delete;
  /// @brief 禁止移动构造。 Non-movable.
  BasicObjectPool(BasicObjectPool&&) = delete;
  /// @brief 禁止移动赋值。 Non-move-assignable.
  BasicObjectPool& operator=(BasicObjectPool&&) = delete;

  /**
   * @brief 获取一个槽位 handle。
   * @brief Acquire one slot handle.
   * @param handle 用于接收成功获取的 handle。 Handle receiving the acquired slot.
   * @return 成功返回 `ErrorCode::OK`，池空返回 `ErrorCode::EMPTY`。
   *         Returns `ErrorCode::OK` on success and `ErrorCode::EMPTY` when the pool is
   * empty.
   */
  [[nodiscard]] ErrorCode Acquire(Handle& handle)
  {
    ASSERT(!handle.Valid());

    IndexType index = 0;
    const ErrorCode ec = free_queue_->Pop(index);
    if (ec != ErrorCode::OK)
    {
      return ec;
    }

    DEV_ASSERT(static_cast<size_t>(index) < slot_count_);
    DEV_ASSERT(slots_[index].references_.load(std::memory_order_relaxed) == 0U);
    slots_[index].references_.store(1U, std::memory_order_relaxed);
    handle = Handle(this, index);
    return ErrorCode::OK;
  }

  /**
   * @brief 返回当前仍可获取的空闲槽位数。
   * @brief Return the number of currently acquirable free slots.
   * @return 当前空闲槽位数。 Current number of free slots.
   */
  [[nodiscard]] size_t EmptySize() const { return free_queue_->Size(); }

  /**
   * @brief 返回对象池总槽位数。
   * @brief Return the total number of slots in the pool.
   * @return 槽位总数。 Total slot count.
   */
  [[nodiscard]] size_t Size() const { return slot_count_; }

  /**
   * @brief 通过槽位索引直接访问对象，不参与所有权检查。
   * @brief Access an object by slot index without ownership checks.
   * @param index 槽位索引。 Slot index.
   * @return 对应槽位对象的引用。 Reference to the object stored in the slot.
   *
   * @note 该接口会绕过 `Acquire()` / `Handle` 的所有权语义，只适合调试、检查
   *       外部存储区或其他明确知道槽位状态的场景。
   *       This API bypasses the ownership semantics of `Acquire()` / `Handle`,
   *       and is intended only for debugging, external-storage inspection, or
   *       other cases where the caller already knows the slot state.
   */
  [[nodiscard]] Data& UnsafeAt(size_t index)
  {
    ASSERT(index < slot_count_);
    return slots_[index].data_;
  }

  /**
   * @brief 通过槽位索引只读直接访问对象，不参与所有权检查。
   * @brief Access an object by slot index as const without ownership checks.
   * @param index 槽位索引。 Slot index.
   * @return 对应槽位对象的常量引用。 Const reference to the object stored in the slot.
   *
   * @note 该接口会绕过 `Acquire()` / `Handle` 的所有权语义，只适合调试、检查
   *       外部存储区或其他明确知道槽位状态的场景。
   *       This API bypasses the ownership semantics of `Acquire()` / `Handle`,
   *       and is intended only for debugging, external-storage inspection, or
   *       other cases where the caller already knows the slot state.
   */
  [[nodiscard]] const Data& UnsafeAt(size_t index) const
  {
    ASSERT(index < slot_count_);
    return slots_[index].data_;
  }

 private:
  /// @brief 保留已存活的引用。 Retain an existing live reference.
  /// @pre 调用方须保证新增引用不会使计数溢出；断言不是运行时溢出防护。
  ///      The caller must prevent count overflow; assertions are not a runtime guard.
  void Retain(IndexType index) noexcept
  {
    DEV_ASSERT(static_cast<size_t>(index) < slot_count_);
    [[maybe_unused]] const uint32_t previous =
        slots_[index].references_.fetch_add(1U, std::memory_order_relaxed);
    DEV_ASSERT(previous > 0U);
    ASSERT(previous < std::numeric_limits<uint32_t>::max());
  }

  /// @brief 释放引用，只有最后一个引用归还索引。 Only the final release returns the
  /// index.
  void Release(IndexType index)
  {
    DEV_ASSERT(static_cast<size_t>(index) < slot_count_);
    const uint32_t previous =
        slots_[index].references_.fetch_sub(1U, std::memory_order_acq_rel);
    DEV_ASSERT(previous > 0U);
    if (previous == 1U)
    {
      // 归还发布后可能立即复用；此后不能再访问槽位。
      // Publication permits immediate reuse; do not access the slot afterward.
      [[maybe_unused]] const ErrorCode ec = free_queue_->Push(index);
      DEV_ASSERT(ec == ErrorCode::OK);
    }
  }

  /**
   * @brief 在内部存储区里构造一个自拥有空闲队列。
   * @brief Construct an owned free queue inside internal storage.
   * @param slot_count 槽位数量，同时也是初始化时要压入的索引数量。
   *                   Slot count and thus the number of indices to preload.
   */
  void ConstructOwnedQueue(size_t slot_count)
  {
    free_queue_ = new (free_queue_storage_) FreeQueue(slot_count);
    owns_free_queue_ = true;
  }

  /**
   * @brief 申请并拥有内部槽数组。
   * @brief Allocate and own the internal slot storage.
   */
  void AllocateOwnedSlots()
  {
    slots_ = new Slot[slot_count_];
    owns_slots_ = true;
  }

  /**
   * @brief 用 `0 .. slot_count - 1` 初始化空闲索引队列。
   * @brief Initialize the free-index queue with `0 .. slot_count - 1`.
   */
  void InitializeFreeQueue()
  {
    ASSERT(slot_count_ > 0);
    ASSERT(slot_count_ - 1 <= static_cast<size_t>(std::numeric_limits<IndexType>::max()));
    for (size_t index = 0; index < slot_count_; ++index)
    {
      [[maybe_unused]] const ErrorCode ec =
          free_queue_->Push(static_cast<IndexType>(index));
      DEV_ASSERT(ec == ErrorCode::OK);
    }
  }

  /// @brief 内部自拥有 queue 的原地构造存储区。 In-place storage for an internally owned
  /// queue.
  alignas(FreeQueue) std::byte free_queue_storage_[sizeof(FreeQueue)] = {};
  FreeQueue* free_queue_ =
      nullptr;               ///< 空闲索引队列指针。 Pointer to the free-index queue.
  const size_t slot_count_;  ///< 槽位总数。 Total slot count.
  Slot* slots_ = nullptr;    ///< 槽数组指针。 Pointer to slot storage.
  bool owns_free_queue_ =
      false;  ///< 是否拥有内部 queue。 Whether this pool owns the free queue.
  bool owns_slots_ =
      false;  ///< 是否拥有内部 slots。 Whether this pool owns the slot storage.
};

/**
 * @brief 基于普通 FIFO 空闲索引队列的 RAII pool 别名。
 * @brief RAII pool alias backed by the ordinary FIFO free-index queue.
 * @tparam Data 槽内对象类型。 Slot object type.
 * @tparam IndexType 槽索引类型，默认 `uint32_t`。 Slot index type, default `uint32_t`.
 */
template <typename Data, typename IndexType = uint32_t>
using ObjectPool = BasicObjectPool<Data, Queue<IndexType>>;

/**
 * @brief 基于 SPSC 空闲索引队列的 RAII pool 别名。
 * @brief RAII pool alias backed by the SPSC free-index queue.
 * @tparam Data 槽内对象类型。 Slot object type.
 * @tparam IndexType 槽索引类型，默认 `uint32_t`。 Slot index type, default `uint32_t`.
 */
template <typename Data, typename IndexType = uint32_t>
using SPSCObjectPool = BasicObjectPool<Data, SPSCQueue<IndexType>>;

/**
 * @brief 基于 MPMC 空闲索引队列的 RAII pool 别名。
 * @brief RAII pool alias backed by the MPMC free-index queue.
 * @tparam Data 槽内对象类型。 Slot object type.
 * @tparam IndexType 槽索引类型，默认 `uint32_t`。 Slot index type, default `uint32_t`.
 */
template <typename Data, typename IndexType = uint32_t>
using MPMCObjectPool = BasicObjectPool<Data, MPMCQueue<IndexType>>;
}  // namespace LibXR
