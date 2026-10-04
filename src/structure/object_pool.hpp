#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <type_traits>
#include <utility>

#include "libxr_def.hpp"

namespace LibXR
{
/**
 * @class ObjectPool
 * @brief 带引用计数的固定槽对象池 / Fixed-slot object pool with reference counting.
 *
 * 成功申请建立一个引用；句柄可以复制，最后一个引用释放时槽位回到空闲栈。
 * 槽内对象随池构造、随池析构，释放时不析构也不清空，下一个申请方自行写入。
 * `Handle` 可写，`ConstHandle` 只读，只能从可写转换为只读；计数不同步负载读写。
 * 运行期操作不分配内存。每槽引用数不得超过 UINT32_MAX。
 *
 * Acquire creates one reference. Handles are copyable, and the final release
 * returns the slot to the free stack. Payloads are constructed and destroyed with
 * the pool; release neither destroys nor clears them, so the next acquirer writes
 * its own data. `Handle` permits writes and `ConstHandle` is read-only; conversion
 * is mutable-to-const only. Counting does not synchronize payload access. Runtime
 * operations allocate no memory. Each slot must have at most UINT32_MAX references.
 *
 * 同一池同一时刻只能有一个申请方：申请不得重叠或重入，调试构建会检查。释放可以在
 * 任意线程或 ISR 中并发进行，且不会失败。不同句柄对象可以并发使用，同一句柄对象的
 * 并发访问须由调用方同步。构造和析构须在静止状态下进行，不能在 ISR 中执行；池及
 * 外部槽存储必须比全部句柄活得更久。ISR 使用要求目标平台提供 32 位原子 CAS。
 *
 * Only one acquirer may use a pool at a time: acquisitions must not overlap or
 * reenter, and debug builds check this. Releases may run concurrently from any
 * thread or ISR and cannot fail. Distinct handle objects may be used concurrently;
 * concurrent access to one handle object requires caller synchronization.
 * Construction and destruction require quiescence outside ISR. The pool and any
 * external slot storage must outlive all handles. ISR use requires 32-bit atomic CAS
 * on the target.
 *
 * @tparam Data 槽内对象类型 / Slot object type.
 */
template <typename Data>
class ObjectPool
{
 public:
  using ValueType = Data;      ///< 槽内对象类型 / Slot object type.
  using IndexType = uint32_t;  ///< 槽索引类型 / Slot index type.

  /**
   * @brief 槽位：引用计数、空闲链接和常驻负载。
   *        Slot: reference count, free-stack link and resident payload.
   * @note 外部存储须提供 Slot 数组，并保持到池及所有句柄使用结束。
   *       External Slot arrays must outlive pool use and all handles.
   */
  class Slot
  {
   public:
    /// @brief 默认构造负载 / Default-construct the payload.
    Slot()
      requires std::is_default_constructible_v<Data>
    = default;

    /**
     * @brief 用参数原地构造负载 / Construct the payload in place from arguments.
     * @param args 负载构造参数 / Payload constructor arguments.
     */
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
    friend class ObjectPool;
    std::atomic<uint32_t> references_{0};
    IndexType next_ = 0;
    Data data_;
  };

 private:
  /// @brief 同一共享所有权的访问限定实现 / Access-qualified shared ownership.
  template <bool IsConst>
  class BasicHandle
  {
    using AccessType = std::conditional_t<IsConst, const Data, Data>;

   public:
    /// @brief 构造空句柄 / Construct an empty handle.
    BasicHandle() = default;

    /// @brief 复制现有引用 / Retain an existing reference.
    BasicHandle(const BasicHandle& other) noexcept
        : pool_(other.pool_), index_(other.index_)
    {
      Retain();
    }

    /// @brief 复制为只读引用 / Copy a mutable reference into a read-only handle.
    template <bool OtherConst>
      requires(IsConst && !OtherConst)
    BasicHandle(const BasicHandle<OtherConst>& other) noexcept
        : pool_(other.pool_), index_(other.index_)
    {
      Retain();
    }

    /// @brief 转移引用并清空源句柄 / Transfer ownership and empty the source.
    BasicHandle(BasicHandle&& other) noexcept
        : pool_(std::exchange(other.pool_, nullptr)),
          index_(std::exchange(other.index_, IndexType{}))
    {
    }

    /// @brief 转移为只读引用 / Transfer mutable ownership into a read-only handle.
    template <bool OtherConst>
      requires(IsConst && !OtherConst)
    BasicHandle(BasicHandle<OtherConst>&& other) noexcept
        : pool_(std::exchange(other.pool_, nullptr)),
          index_(std::exchange(other.index_, IndexType{}))
    {
    }

    /// @brief 先保留源引用再释放旧引用 / Retain the source before releasing the old
    /// reference.
    BasicHandle& operator=(const BasicHandle& other) noexcept
    {
      if (pool_ != other.pool_ || index_ != other.index_)
      {
        BasicHandle copy(other);
        Swap(copy);
      }
      return *this;
    }

    /// @brief 从可写句柄复制赋值 / Copy-assign from a mutable handle.
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

    /// @brief 转移赋值并释放旧引用 / Move-assign and release the old reference.
    BasicHandle& operator=(BasicHandle&& other) noexcept
    {
      if (this != &other)
      {
        BasicHandle moved(std::move(other));
        Swap(moved);
      }
      return *this;
    }

    /// @brief 从可写句柄转移赋值 / Move-assign from a mutable handle.
    template <bool OtherConst>
      requires(IsConst && !OtherConst)
    BasicHandle& operator=(BasicHandle<OtherConst>&& other) noexcept
    {
      BasicHandle moved(std::move(other));
      Swap(moved);
      return *this;
    }

    /// @brief 释放本引用；最后一个引用归还槽位 / Release; the final reference returns
    /// the slot.
    ~BasicHandle() { Reset(); }

    /// @brief 是否持有引用 / Whether this handle owns a reference.
    [[nodiscard]] bool Valid() const { return pool_ != nullptr; }

    /// @brief 按句柄权限访问负载，句柄必须有效 / Access the payload; requires a valid
    /// handle.
    [[nodiscard]] AccessType& Get()
    {
      ASSERT(Valid());
      return pool_->slots_[index_].data_;
    }

    /// @brief 只读访问负载，句柄必须有效 / Read-only payload access; requires a valid
    /// handle.
    [[nodiscard]] const Data& Get() const
    {
      ASSERT(Valid());
      return pool_->slots_[index_].data_;
    }

    /// @brief 按句柄权限返回负载指针 / Return an access-qualified payload pointer.
    [[nodiscard]] AccessType* operator->() { return &Get(); }
    /// @brief 返回只读负载指针 / Return a read-only payload pointer.
    [[nodiscard]] const Data* operator->() const { return &Get(); }
    /// @brief 按句柄权限解引用 / Dereference with this handle's access qualification.
    [[nodiscard]] AccessType& operator*() { return Get(); }
    /// @brief 只读解引用 / Dereference read-only.
    [[nodiscard]] const Data& operator*() const { return Get(); }

    /// @brief 返回有效句柄的槽索引 / Return the slot index of a valid handle.
    [[nodiscard]] IndexType Index() const
    {
      ASSERT(Valid());
      return index_;
    }

    /// @brief 清空句柄并释放引用；空句柄无操作 / Empty and release; no-op on an empty
    /// handle.
    void Reset()
    {
      ObjectPool* pool = std::exchange(pool_, nullptr);
      const IndexType index = std::exchange(index_, IndexType{});
      if (pool != nullptr)
      {
        pool->Release(index);
      }
    }

   private:
    friend class ObjectPool;
    template <bool>
    friend class BasicHandle;

    BasicHandle(ObjectPool* pool, IndexType index) : pool_(pool), index_(index) {}

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

    ObjectPool* pool_ = nullptr;
    IndexType index_ = {};
  };

 public:
  /// @brief 可复制的可写共享句柄 / Copyable mutable shared handle.
  using Handle = BasicHandle<false>;
  /// @brief 可复制的只读共享句柄，只接受可写到只读转换。
  ///        Copyable read-only shared handle; conversion is mutable-to-const only.
  using ConstHandle = BasicHandle<true>;

  /**
   * @brief 用内部槽数组构造池 / Construct the pool with internal slots.
   * @param slot_count 槽位数量 / Number of slots.
   */
  template <typename T = Data>
    requires std::is_default_constructible_v<T>
  explicit ObjectPool(size_t slot_count)
      : slots_(new Slot[slot_count]), slot_count_(slot_count), owns_slots_(true)
  {
    InitializeFreeStack();
  }

  /**
   * @brief 用外部槽数组构造池 / Construct the pool with caller-provided slots.
   * @param slot_count 槽位数量 / Number of slots.
   * @param slots 外部槽数组 / Caller-provided slot storage.
   *
   * @note `slots` 必须指向至少 `slot_count` 个未被其他池使用的 `Slot`；池不构造也不
   *       析构其中的负载。
   *       `slots` must point to at least `slot_count` slots not used by another pool;
   *       the pool neither constructs nor destroys their payloads.
   */
  ObjectPool(size_t slot_count, Slot* slots)
      : slots_(slots), slot_count_(slot_count), owns_slots_(false)
  {
    ASSERT(slots_ != nullptr);
    InitializeFreeStack();
  }

  /**
   * @brief 析构池；所有句柄必须已释放 / Destroy the pool; all handles must be released.
   */
  ~ObjectPool()
  {
    ASSERT(EmptySize() == slot_count_);

    if (owns_slots_)
    {
      delete[] slots_;
    }
  }

  ObjectPool(const ObjectPool&) = delete;
  ObjectPool& operator=(const ObjectPool&) = delete;
  ObjectPool(ObjectPool&&) = delete;
  ObjectPool& operator=(ObjectPool&&) = delete;

  /**
   * @brief 获取一个空闲槽位 / Acquire one free slot.
   * @param handle 接收槽位的空句柄 / Empty handle receiving the slot.
   * @return 成功返回 `ErrorCode::OK`，没有空槽返回 `ErrorCode::EMPTY`，不等待。
   *         `ErrorCode::OK` on success, `ErrorCode::EMPTY` without waiting when no
   *         slot is free.
   * @note 同一池的申请不得重叠或重入 / Acquisitions on one pool must not overlap or
   *       reenter.
   */
  [[nodiscard]] ErrorCode Acquire(Handle& handle)
  {
    ASSERT(!handle.Valid());
#ifdef LIBXR_DEBUG_BUILD
    // 重叠申请会让空闲栈出现 ABA，同一槽位被两个申请方取得。
    // Overlapping acquisitions cause ABA on the free stack and hand one slot to two
    // acquirers.
    uint32_t idle = 0U;
    ASSERT(acquiring_.compare_exchange_strong(idle, 1U, std::memory_order_acquire,
                                              std::memory_order_relaxed));
#endif

    IndexType index = 0;
    const ErrorCode ec = PopFree(index);
    if (ec == ErrorCode::OK)
    {
      free_count_.fetch_sub(1U, std::memory_order_relaxed);
      DEV_ASSERT(slots_[index].references_.load(std::memory_order_relaxed) == 0U);
      slots_[index].references_.store(1U, std::memory_order_relaxed);
      handle = Handle(this, index);
    }

#ifdef LIBXR_DEBUG_BUILD
    acquiring_.store(0U, std::memory_order_release);
#endif
    return ec;
  }

  /**
   * @brief 返回空闲槽位数 / Return the number of free slots.
   * @return 空闲槽位数；有并发申请或释放时为近似值，只用于监控。
   *         Free slot count; approximate during concurrent acquire or release, for
   *         monitoring only.
   */
  [[nodiscard]] size_t EmptySize() const
  {
    return free_count_.load(std::memory_order_relaxed);
  }

  /**
   * @brief 返回槽位总数 / Return the total number of slots.
   * @return 槽位总数 / Total slot count.
   */
  [[nodiscard]] size_t Size() const { return slot_count_; }

  /**
   * @brief 按槽位索引直接访问对象，不检查所有权。
   *        Access an object by slot index without ownership checks.
   * @param index 槽位索引 / Slot index.
   * @return 槽内对象的引用 / Reference to the slot object.
   * @note 只用于调试或调用方已确知槽位状态的场景。
   *       Intended for debugging or when the caller already knows the slot state.
   */
  [[nodiscard]] Data& UnsafeAt(size_t index)
  {
    ASSERT(index < slot_count_);
    return slots_[index].data_;
  }

  /**
   * @brief 按槽位索引只读访问对象，不检查所有权。
   *        Read-only access by slot index without ownership checks.
   * @param index 槽位索引 / Slot index.
   * @return 槽内对象的常量引用 / Const reference to the slot object.
   * @note 只用于调试或调用方已确知槽位状态的场景。
   *       Intended for debugging or when the caller already knows the slot state.
   */
  [[nodiscard]] const Data& UnsafeAt(size_t index) const
  {
    ASSERT(index < slot_count_);
    return slots_[index].data_;
  }

 private:
  static constexpr IndexType EMPTY_INDEX = std::numeric_limits<IndexType>::max();

  /// @pre 调用方须保证计数不溢出 / The caller must prevent count overflow.
  void Retain(IndexType index) noexcept
  {
    DEV_ASSERT(static_cast<size_t>(index) < slot_count_);
    [[maybe_unused]] const uint32_t previous =
        slots_[index].references_.fetch_add(1U, std::memory_order_relaxed);
    DEV_ASSERT(previous > 0U);
    ASSERT(previous < std::numeric_limits<uint32_t>::max());
  }

  void Release(IndexType index)
  {
    DEV_ASSERT(static_cast<size_t>(index) < slot_count_);
    // acq_rel：其他持有者的负载访问先于最终归还。
    // acq_rel: other holders' payload accesses happen before the final return.
    const uint32_t previous =
        slots_[index].references_.fetch_sub(1U, std::memory_order_acq_rel);
    DEV_ASSERT(previous > 0U);
    if (previous == 1U)
    {
      free_count_.fetch_add(1U, std::memory_order_relaxed);
      PushFree(index);
    }
  }

  void InitializeFreeStack()
  {
    ASSERT(slot_count_ > 0);
    ASSERT(slot_count_ < static_cast<size_t>(EMPTY_INDEX));
    for (size_t index = 0; index < slot_count_; ++index)
    {
      slots_[index].next_ = static_cast<IndexType>(index + 1U);
    }
    slots_[slot_count_ - 1U].next_ = EMPTY_INDEX;
    free_head_.store(0U, std::memory_order_relaxed);
    free_count_.store(static_cast<uint32_t>(slot_count_), std::memory_order_relaxed);
  }

  /// @brief 弹出栈顶空槽；只由唯一申请方调用 / Pop the top free slot; called only by
  /// the single acquirer.
  ErrorCode PopFree(IndexType& index)
  {
    IndexType head = free_head_.load(std::memory_order_acquire);
    IndexType next = EMPTY_INDEX;
    do
    {
      if (head == EMPTY_INDEX)
      {
        return ErrorCode::EMPTY;
      }
      DEV_ASSERT(static_cast<size_t>(head) < slot_count_);
      // 只有申请方会取出槽位，head 在栈上时其 next_ 不会被改写，所以没有 ABA。
      // Only the acquirer removes slots, so next_ of a stacked head cannot change and
      // ABA cannot occur.
      next = slots_[head].next_;
    } while (!free_head_.compare_exchange_weak(head, next, std::memory_order_acquire,
                                               std::memory_order_acquire));
    index = head;
    return ErrorCode::OK;
  }

  /// @brief 把槽位压回栈顶；可并发、可在 ISR 调用 / Push a slot back; concurrent and
  /// ISR-safe.
  void PushFree(IndexType index)
  {
    IndexType head = free_head_.load(std::memory_order_relaxed);
    do
    {
      // 槽位已不在栈上且只属于本次归还，可以直接写 next_。
      // The slot is off the stack and owned by this return, so next_ is private here.
      slots_[index].next_ = head;
    } while (!free_head_.compare_exchange_weak(head, index, std::memory_order_release,
                                               std::memory_order_relaxed));
  }

  Slot* slots_;              ///< 槽数组 / Slot storage.
  const size_t slot_count_;  ///< 槽位总数 / Total slot count.
  const bool owns_slots_;    ///< 是否由池分配槽数组 / Whether the pool owns the slots.
  std::atomic<IndexType> free_head_{EMPTY_INDEX};  ///< 空闲栈顶 / Free-stack top.
  std::atomic<uint32_t> free_count_{0};            ///< 空闲槽位数 / Free slot count.
  /// 调试构建的重叠申请检查；各构建都保留，布局不随编译开关变化。
  /// Overlap check for debug builds; kept in every build so the layout does not
  /// depend on build flags.
  std::atomic<uint32_t> acquiring_{0};
};
}  // namespace LibXR
