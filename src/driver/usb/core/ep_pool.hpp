#pragma once

#include <cstddef>
#include <cstdint>

#include "core.hpp"
#include "ep.hpp"
#include "serialized_service.hpp"

namespace LibXR::USB
{

/**
 * @brief USB端点池类 / USB endpoint pool class
 *
 * 按「端点号 × 方向」二维索引管理固定的 Endpoint 指针。所有 Endpoint 对象由后端在
 * 初始化时通过 Put() 填入，池只负责登记、分配、查找与回收，不负责对象生命周期。
 * 端点 0 的 IN/OUT 不进入索引数组，由 SetEndpoint0/GetEndpoint0* 单独管理。
 *
 * Manages fixed Endpoint pointers indexed by (endpoint-number, direction). All Endpoint
 * objects are populated by the backend at init time via Put(); the pool only registers,
 * allocates, looks up and recycles them, and does not own their lifetime. Endpoint 0's
 * IN/OUT are kept out of the index array and managed by SetEndpoint0/GetEndpoint0*.
 *
 * @note 分配/回收发生在配置切换（SetConfiguration/Reset）这类低频、非并发路径，
 *       因此使用普通数组与非原子状态，无需无锁结构。
 *       Allocation/recycling happens on low-frequency, non-concurrent paths
 *       (SetConfiguration/Reset), so a plain array with non-atomic state is used.
 */
class EndpointPool
{
 public:
  /**
   * @brief 构造函数 / Constructor
   * @param endpoint_num 端点总数（包含端点0，必须 >=2）/ Total number of endpoints
   * (including EP0, must >=2)
   *
   * @note 端点对象本身由后端通过 Put() 填入；该参数仅用于合法性校验。
   *       Endpoint objects are populated by the backend via Put(); this argument is only
   *       used for a validity check.
   */
  explicit EndpointPool(size_t endpoint_num);

  EndpointPool(const EndpointPool&) = delete;
  EndpointPool& operator=(const EndpointPool&) = delete;

  /**
   * @brief 登记一个端点到池中 / Register an endpoint into the pool
   *
   * 按端点自身的号（GetNumber）与可用方向（AvailableDirection）入位。端点 0 不应
   * 通过此接口登记（应使用 SetEndpoint0）。
   * Registered by the endpoint's own number (GetNumber) and available direction
   * (AvailableDirection). Endpoint 0 must not be registered here (use SetEndpoint0).
   *
   * @param ep 待登记的端点对象指针 / Endpoint pointer to register
   * @retval ErrorCode 操作结果（OK/ARG_ERR/FULL）/ Operation result
   */
  ErrorCode Put(Endpoint* ep);

  /**
   * @brief 分配端点 / Allocate endpoint
   *
   * 按指定端点号与方向精确匹配一个已登记且空闲的端点。调用方必须显式指定端点号，
   * 不再支持自动分配。
   * Precisely matches a registered, available endpoint by the given number and direction.
   * The caller must specify the endpoint number explicitly; auto-allocation is no longer
   * supported.
   *
   * @param[out] ep_info 分配得到的端点对象指针 / Allocated endpoint pointer
   * @param direction 端点方向（IN/OUT）/ Endpoint direction
   * @param ep_num 指定端点号（必填）/ Endpoint number (required)
   * @retval ErrorCode 操作结果（OK/NOT_FOUND）/ Operation result
   */
  ErrorCode Get(Endpoint*& ep_info, Endpoint::Direction direction,
                Endpoint::EPNumber ep_num);

  /**
   * @brief 回收端点 / Release endpoint
   * @param ep_info 待回收的端点对象指针 / Endpoint pointer to release
   * @retval ErrorCode 操作结果（OK/NOT_FOUND）/ Operation result
   */
  ErrorCode Release(Endpoint* ep_info);

  /**
   * @brief 查找端点/ Lookup endpoint
   *
   * 端点 0 走 ep0_in_/ep0_out_ 旁路；其余端点仅在「已分配（in-use）」状态下可被查到，
   * 未分配的端点返回 NOT_FOUND（收到端点级请求视为异常）。
   * Endpoint 0 goes through the ep0_in_/ep0_out_ bypass; other endpoints are visible only
   * while allocated (in-use); an unallocated endpoint returns NOT_FOUND (an
   * endpoint-level request for it is treated as an error).
   *
   * @param ep_addr 端点地址，IN端点高位需加0x80 / Endpoint address (0x80 for IN
   * endpoints)
   * @param[out] ans 查找到的端点对象指针 / Found endpoint pointer
   * @retval ErrorCode 操作结果（OK/NOT_FOUND）/ Operation result
   */
  ErrorCode FindEndpoint(uint8_t ep_addr, Endpoint*& ans);

  /**
   * @brief 获取端点0的OUT对象 / Get Endpoint 0's OUT object
   * @return Endpoint* 端点0 OUT对象指针 / Endpoint 0 OUT pointer
   */
  Endpoint* GetEndpoint0Out();

  /**
   * @brief 获取端点0的IN对象 / Get Endpoint 0's IN object
   * @return Endpoint* 端点0 IN对象指针 / Endpoint 0 IN pointer
   */
  Endpoint* GetEndpoint0In();

  /**
   * @brief 设置端点0的IN/OUT对象 / Set Endpoint 0 IN/OUT objects
   * @param ep0_in 端点0 IN对象指针 / Endpoint 0 IN pointer
   * @param ep0_out 端点0 OUT对象指针 / Endpoint 0 OUT pointer
   */
  void SetEndpoint0(Endpoint* ep0_in, Endpoint* ep0_out);

  using HardwareEnter = uintptr_t (*)(void*);
  using HardwareLeave = void (*)(void*, uintptr_t);
  void SetHardwareGuard(void* context, HardwareEnter enter, HardwareLeave leave)
  {
    hardware_context_ = context;
    hardware_enter_ = enter;
    hardware_leave_ = leave;
  }

  // Short hardware/control-mailbox critical region; never class callbacks or blocking
  // work. Backends mask local IRQs in every context (the owner may run inside an
  // unrelated ISR that the controller IRQ can preempt) and, on SMP, also serialize
  // register access.
  class HardwareScope
  {
   public:
    [[gnu::always_inline]] explicit HardwareScope(EndpointPool* pool) : pool_(pool)
    {
      if (pool_ && pool_->hardware_enter_)
        token_ = pool_->hardware_enter_(pool_->hardware_context_);
    }
    [[gnu::always_inline]] ~HardwareScope() { Release(); }
    HardwareScope(const HardwareScope&) = delete;
    HardwareScope& operator=(const HardwareScope&) = delete;
    [[gnu::always_inline]] void Release()
    {
      if (pool_ && pool_->hardware_leave_)
        pool_->hardware_leave_(pool_->hardware_context_, token_);
      pool_ = nullptr;
    }

   private:
    EndpointPool* pool_;
    uintptr_t token_ = 0;
  };

  // Marks one controller IRQ batch. Callbacks inside it only record facts; the owner
  // is requested once the outermost batch ends. It does not mask interrupts.
  class InterruptScope
  {
   public:
    explicit InterruptScope(EndpointPool& pool) : pool_(pool)
    {
      pool_.interrupt_depth_.fetch_add(1U, std::memory_order_acq_rel);
    }
    ~InterruptScope()
    {
      // A batch that recorded nothing (a FIFO refill, say) does not run the owner.
      if (pool_.interrupt_depth_.fetch_sub(1U, std::memory_order_acq_rel) == 1U &&
          pool_.irq_deferred_.exchange(0U, std::memory_order_acq_rel) != 0U)
        pool_.RequestService(true);
    }
    InterruptScope(const InterruptScope&) = delete;
    InterruptScope& operator=(const InterruptScope&) = delete;

   private:
    EndpointPool& pool_;
  };

  /// Owner event bits. The device core owns the low byte; the pool owns the rest.
  static constexpr uint32_t INIT_EVENT = 1U;
  static constexpr uint32_t DEINIT_EVENT = 2U;
  static constexpr uint32_t RESET_EVENT = 4U;
  static constexpr uint32_t DISCONNECT_EVENT = 8U;
  static constexpr uint32_t SETUP_EVENT = 16U;
  static constexpr uint32_t POWER_EVENT = 32U;
  static constexpr uint32_t TIME_EVENT = 64U;
  static constexpr uint32_t CLASS_EVENT = 128U;
  static constexpr uint32_t ENDPOINT_EVENT = 256U;
  static constexpr uint32_t STOP_DONE_EVENT = 512U;
  static constexpr uint32_t STOP_TIMEOUT_EVENT = 1024U;
  /// Frames an unconfirmed stop may wait before the controller is reset.
  static constexpr uint32_t STOP_FRAME_LIMIT = 8U;
  static constexpr uint32_t LIFECYCLE_EVENTS =
      INIT_EVENT | DEINIT_EVENT | RESET_EVENT | DISCONNECT_EVENT;

  /**
   * @brief 请求 owner：没有 owner 时调用者就地执行，否则把工作留给当前 owner /
   *        Request the owner: run it in place when there is none, otherwise leave the
   *        work to the current owner
   *
   * owner 是赢得 `SerializedService::Invoke` 的调用者：控制器中断，或者任何经由
   * Submit/Receive/ReturnSlot/RequestClassService/Init 等走到这里的应用线程。owner
   * 中运行的所有回调（端点完成、类回调、控制请求）因此可能在中断里（in_isr=true），也可能
   * 在任意应用线程里（in_isr=false），都不得阻塞。线程持有 owner 时，中断只发布工作，由
   * 这个线程在当前一轮里处理；在控制器中断批次（InterruptScope）里只发布，批次结束时再请求。
   * The owner is whichever caller wins `SerializedService::Invoke`: the controller
   * interrupt, or any application thread that gets here through
   * Submit/Receive/ReturnSlot/RequestClassService/Init and the like. Every callback run
   * in the owner (endpoint completions, class callbacks, control requests) can therefore
   * run in an interrupt (in_isr=true) or in an arbitrary application thread
   * (in_isr=false), and none may block. While a thread owns, interrupt work is only
   * published and that thread handles it in its current pass; inside a controller
   * interrupt batch (InterruptScope) work is only published and requested when the batch
   * ends.
   */
  void RequestService(bool in_isr);
  void SetControlHandler(Callback<uint32_t> callback) { control_handler_ = callback; }
  // Publish atomically with a hardware/mailbox change, then RequestService after
  // releasing the short guard. Publication itself never invokes callbacks.
  void PublishControl(uint32_t events)
  {
    control_pending_.fetch_or(events, std::memory_order_release);
  }
  void PostControl(uint32_t events, bool in_isr);
  [[gnu::always_inline]] bool HasPendingControl(uint32_t mask = UINT32_MAX) const
  {
    return (control_pending_.load(std::memory_order_acquire) & mask) != 0U;
  }
  /// An endpoint has work: a submit, a completion or a stop confirmation.
  /// Owner: clear an endpoint's pending bit; true if it was set.
  bool TakeEndpointBit(uint32_t bit)
  {
    return (endpoint_pending_.fetch_and(~bit, std::memory_order_acq_rel) & bit) != 0U;
  }
  [[gnu::always_inline]] void NotifyEndpoint(uint32_t bit, bool in_isr)
  {
    endpoint_pending_.fetch_or(bit, std::memory_order_release);
    PostControl(ENDPOINT_EVENT, in_isr);
  }
  /// Owner: make every endpoint recheck its queue after a start gate opened.
  void KickEndpoints(bool in_isr);
  /// Owner: whether an endpoint may start hardware now.
  [[gnu::always_inline]] bool CanStart(bool ep0) const
  {
    return !planning_ && !suspended_ && !lifecycle_hold_ &&
           !HasPendingControl(LIFECYCLE_EVENTS) && (ep0 || data_enabled_);
  }
  /// Owner: count of endpoints whose hardware stop is not yet confirmed.
  bool AnyStopping() const
  {
    return stopping_count_.load(std::memory_order_relaxed) != 0U;
  }
  void AddStopping()
  {
    if (stopping_count_.fetch_add(1U, std::memory_order_relaxed) == 0U)
    {
      stop_frames_.store(0U, std::memory_order_relaxed);
      stop_last_frame_.store(NO_FRAME, std::memory_order_relaxed);
    }
  }
  /// IRQ (bus tick): true once when a stop has waited STOP_FRAME_LIMIT 1 ms frames. A
  /// high-speed bus ticks every microframe; only a new frame number counts. 0xFFFF is an
  /// unknown frame number (full-speed controllers without a frame counter): every tick
  /// is one frame.
  bool CountStopFrame(uint16_t frame)
  {
    if (stopping_count_.load(std::memory_order_relaxed) == 0U) return false;
    if (frame != 0xFFFFU &&
        stop_last_frame_.exchange(frame, std::memory_order_relaxed) == frame)
      return false;
    return stop_frames_.fetch_add(1U, std::memory_order_relaxed) + 1U == STOP_FRAME_LIMIT;
  }
  /// Owner: a lifecycle event is held back; no endpoint may start meanwhile.
  void SetLifecycleHold(bool hold) { lifecycle_hold_ = hold; }
  void StopFinished(bool in_isr);

  bool DataEnabled() const { return data_enabled_; }
  void SetDataEnabled(bool enabled, bool in_isr);
  bool IsPlanning() const { return planning_; }
  void SetPlanning(bool planning) { planning_ = planning; }
  Speed GetSpeed() const { return speed_; }
  void SetSpeed(Speed speed) { speed_ = speed; }
  bool ConfigurationValid() const;
  bool IsSuspended() const { return suspended_; }
  void SetSuspended(bool suspended, bool in_isr);

 private:
  /// 方向数量（OUT=0, IN=1）/ Number of direction slots (OUT=0, IN=1)
  static constexpr size_t DIR_COUNT = 2;
  /// 索引数组容量（端点号 0..EP_MAX_NUM-1）/ Index array size (endpoint numbers)
  static constexpr size_t SLOT_COUNT =
      static_cast<size_t>(Endpoint::EPNumber::EP_MAX_NUM);

  /// 槽占用状态 / Slot usage state
  enum class SlotUse : uint8_t
  {
    AVAILABLE,  ///< 已登记、空闲可分配 / Registered and free to allocate
    IN_USE      ///< 已分配、正在使用 / Allocated and in use
  };

  /**
   * @brief 方向枚举转索引下标 / Convert direction enum to array index
   * @param dir 端点方向（IN/OUT）/ Endpoint direction
   * @return size_t 下标（OUT=0, IN=1）/ Index (OUT=0, IN=1)
   */
  static size_t DirIndex(Endpoint::Direction dir)
  {
    return (dir == Endpoint::Direction::IN) ? 1U : 0U;
  }

  Endpoint* slots_[SLOT_COUNT][DIR_COUNT] = {};  ///< [端点号][方向] 端点指针 / [num][dir]
  SlotUse use_[SLOT_COUNT][DIR_COUNT] = {};      ///< 对应槽占用状态 / Per-slot usage

  void* hardware_context_ = nullptr;
  HardwareEnter hardware_enter_ = nullptr;
  HardwareLeave hardware_leave_ = nullptr;
  std::atomic<uint32_t> interrupt_depth_{0};
  std::atomic<uint32_t> irq_deferred_{0};  ///< Work was recorded during an IRQ batch.
  SerializedService service_;
  std::atomic<uint32_t> control_pending_{0};
  // Endpoints with work: bit 2n is EP n OUT (or a bidirectional EP n), 2n + 1 is EP n IN.
  std::atomic<uint32_t> endpoint_pending_{0};
  Endpoint* by_bit_[2 * SLOT_COUNT] = {};
  void Register(Endpoint* ep, size_t number, size_t dir);
  Callback<uint32_t> control_handler_;
  std::atomic<uint32_t> stopping_count_{0};  ///< Changed by the owner only.
  std::atomic<uint32_t> stop_frames_{0};
  static constexpr uint32_t NO_FRAME = UINT32_MAX;
  std::atomic<uint32_t> stop_last_frame_{NO_FRAME};  ///< Frame number of the last count.
  bool lifecycle_hold_ = false;                      ///< Controller-owner state.
  bool data_enabled_ = true;
  bool planning_ = false;
  Speed speed_ = Speed::FULL;
  bool suspended_ = false;       ///< Controller-owner state.
  Endpoint* ep0_in_ = nullptr;   ///< 端点0 IN对象 / Endpoint 0 IN pointer
  Endpoint* ep0_out_ = nullptr;  ///< 端点0 OUT对象 / Endpoint 0 OUT pointer
};

// Endpoint helpers on the request path that need the pool; ep.cpp inlines them.
inline bool Endpoint::CanStart() const
{
  return pool_ == nullptr || pool_->CanStart(number_ == EPNumber::EP0);
}

inline void Endpoint::NotifyOwner(bool in_isr)
{
  if (pool_ != nullptr) pool_->NotifyEndpoint(pending_bit_, in_isr);
}

inline void Endpoint::NotifyQueued(bool in_isr)
{
  // Pairs with the fence in Service: either this sees servicing_ cleared and notifies,
  // or the owner's StartNext sees the request just queued.
  std::atomic_thread_fence(std::memory_order_seq_cst);
  if (servicing_.load(std::memory_order_relaxed) != 0U) return;
  NotifyOwner(in_isr);
}

}  // namespace LibXR::USB
