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

  /// 进入硬件临界区的回调，返回的令牌原样交给 HardwareLeave / Callback that enters the
  /// hardware critical region; the returned token is passed back to HardwareLeave
  using HardwareEnter = uintptr_t (*)(void*);
  /// 离开硬件临界区的回调，参数为 context 与 HardwareEnter 返回的令牌 / Callback that
  /// leaves the critical region, given the context and the token from HardwareEnter
  using HardwareLeave = void (*)(void*, uintptr_t);
  /**
   * @brief 设置后端的硬件临界区回调 / Set the backend's hardware critical-region hooks
   * @param context 回调的用户指针 / User pointer passed to both callbacks
   * @param enter 进入回调，可为空（不加保护）/ Enter callback, may be null (no guard)
   * @param leave 离开回调，可为空 / Leave callback, may be null
   */
  void SetHardwareGuard(void* context, HardwareEnter enter, HardwareLeave leave)
  {
    hardware_context_ = context;
    hardware_enter_ = enter;
    hardware_leave_ = leave;
  }

  /**
   * @brief 硬件/控制邮箱临界区（RAII）/ Hardware and control-mailbox critical region
   *
   * 临界区必须很短，不得运行类回调或阻塞。后端在任何上下文都屏蔽本地中断（owner 可能
   * 运行在无关的中断里，而控制器中断可抢占它），SMP 下还要串行化寄存器访问。
   * Keep it short; never run class callbacks or blocking work. Backends mask local IRQs
   * in every context (the owner may run inside an unrelated ISR that the controller IRQ
   * can preempt) and, on SMP, also serialize register access.
   */
  class HardwareScope
  {
   public:
    /// 进入 pool 的硬件临界区；pool 不会为空，编译器不必再沿空指针路径分析
    /// Enter the hardware critical region of pool; pool is never null, so the compiler
    /// does not follow a null-pointer path
    LIBXR_ALWAYS_INLINE explicit HardwareScope(EndpointPool& pool) : pool_(&pool)
    {
      if (pool.hardware_enter_) token_ = pool.hardware_enter_(pool.hardware_context_);
    }
    /// 指针版本；pool 为空或未设置回调时不做任何事 / Pointer overload; does nothing when
    /// pool is null or no hook is set
    LIBXR_ALWAYS_INLINE explicit HardwareScope(EndpointPool* pool) : pool_(pool)
    {
      if (pool_ && pool_->hardware_enter_)
        token_ = pool_->hardware_enter_(pool_->hardware_context_);
    }
    LIBXR_ALWAYS_INLINE ~HardwareScope() { Release(); }
    HardwareScope(const HardwareScope&) = delete;
    HardwareScope& operator=(const HardwareScope&) = delete;
    /// 提前离开临界区；重复调用无效，析构时不会再离开一次 / Leave early; repeated calls
    /// are no-ops and the destructor will not leave again
    LIBXR_ALWAYS_INLINE void Release()
    {
      if (pool_ && pool_->hardware_leave_)
        pool_->hardware_leave_(pool_->hardware_context_, token_);
      pool_ = nullptr;
    }

   private:
    EndpointPool* pool_;
    uintptr_t token_ = 0;
  };

  /**
   * @brief 标记一次控制器中断批次（RAII）/ Marks one controller IRQ batch (RAII)
   *
   * 批次内的回调只记录事实，最外层批次结束时才请求 owner；不屏蔽中断。
   * Callbacks inside it only record facts; the owner is requested once the outermost
   * batch ends. It does not mask interrupts.
   */
  class InterruptScope
  {
   public:
    /// 进入批次，嵌套深度加一 / Enter a batch; increments the nesting depth
    explicit InterruptScope(EndpointPool& pool) : pool_(pool)
    {
      pool_.interrupt_depth_.fetch_add(1U, std::memory_order_acq_rel);
    }
    ~InterruptScope()
    {
      // 没有记录任何工作的批次（例如 FIFO 补充）不会运行 owner。
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

  // Owner 事件位：设备核心使用低字节，其余归 pool / Owner event bits. The device core
  // owns the low byte; the pool owns the rest.
  /// 请求初始化（使能）/ Init (enable) requested
  static constexpr uint32_t INIT_EVENT = 1U;
  /// 请求反初始化（禁用）/ Deinit (disable) requested
  static constexpr uint32_t DEINIT_EVENT = 2U;
  /// 总线复位 / Bus reset
  static constexpr uint32_t RESET_EVENT = 4U;
  /// 总线断开 / Bus disconnect
  static constexpr uint32_t DISCONNECT_EVENT = 8U;
  /// 端点 0 收到 Setup 包 / A setup packet arrived on endpoint 0
  static constexpr uint32_t SETUP_EVENT = 16U;
  /// 挂起/恢复状态变化 / Suspend or resume state changed
  static constexpr uint32_t POWER_EVENT = 32U;
  /// 需要向类投递总线时间 / The bus time must be delivered to the classes
  static constexpr uint32_t TIME_EVENT = 64U;
  /// 某个类请求了 owner 服务 / A class requested owner service
  static constexpr uint32_t CLASS_EVENT = 128U;
  /// 至少一个端点有待处理的工作 / At least one endpoint has pending work
  static constexpr uint32_t ENDPOINT_EVENT = 256U;
  /// 最后一个未确认的端点停止已确认 / The last unconfirmed endpoint stop was confirmed
  static constexpr uint32_t STOP_DONE_EVENT = 512U;
  /// 端点停止等待超过 STOP_FRAME_LIMIT 帧 / An endpoint stop waited STOP_FRAME_LIMIT
  /// frames without confirmation
  static constexpr uint32_t STOP_TIMEOUT_EVENT = 1024U;
  /// 控制器复位前，一次未确认的停止最多可等待的帧数。
  /// Frames an unconfirmed stop may wait before the controller is reset.
  static constexpr uint32_t STOP_FRAME_LIMIT = 8U;
  /// 生命周期事件掩码（Init/Deinit/Reset/Disconnect）；挂起时端点不得启动硬件 /
  /// Lifecycle event mask (Init/Deinit/Reset/Disconnect); while one is pending no
  /// endpoint may start hardware
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
  /// 设置 owner 处理控制事件（除 ENDPOINT_EVENT 外的位）的回调，参数为事件位掩码 /
  /// Set the owner's handler for control events (every bit except ENDPOINT_EVENT); the
  /// argument is the event mask
  void SetControlHandler(Callback<uint32_t> callback) { control_handler_ = callback; }
  // Publish atomically with a hardware/mailbox change, then RequestService after
  // releasing the short guard. Publication itself never invokes callbacks.
  /// 只发布事件位，不请求 owner，不调用回调 / Publish event bits only; neither requests
  /// the owner nor runs callbacks
  void PublishControl(uint32_t events)
  {
    control_pending_.fetch_or(events, std::memory_order_release);
  }
  /// 发布事件位并请求 owner（PublishControl + RequestService）/ Publish event bits and
  /// request the owner (PublishControl + RequestService)
  void PostControl(uint32_t events, bool in_isr);
  /// 是否有 mask 内的控制事件待处理 / Whether any control event within mask is pending
  LIBXR_ALWAYS_INLINE bool HasPendingControl(uint32_t mask = UINT32_MAX) const
  {
    return (control_pending_.load(std::memory_order_acquire) & mask) != 0U;
  }
  /// Owner：清除端点的待处理位；该位原先已置位时返回 true。
  /// Owner: clear an endpoint's pending bit; true if it was set.
  bool TakeEndpointBit(uint32_t bit)
  {
    return (endpoint_pending_.fetch_and(~bit, std::memory_order_acq_rel) & bit) != 0U;
  }
  /// 端点有工作（提交、完成或停止确认）：置位其待处理位并投递 ENDPOINT_EVENT /
  /// An endpoint has work (a submit, a completion or a stop confirmation): set its
  /// pending bit and post ENDPOINT_EVENT
  LIBXR_ALWAYS_INLINE void NotifyEndpoint(uint32_t bit, bool in_isr)
  {
    endpoint_pending_.fetch_or(bit, std::memory_order_release);
    PostControl(ENDPOINT_EVENT, in_isr);
  }
  /// Owner：启动门打开后，让每个端点重新检查其队列。
  /// Owner: make every endpoint recheck its queue after a start gate opened.
  void KickEndpoints(bool in_isr);
  /// Owner：端点现在是否可以启动硬件 / Owner: whether an endpoint may start hardware now.
  LIBXR_ALWAYS_INLINE bool CanStart(bool ep0) const
  {
    return !planning_ && !suspended_ && !lifecycle_hold_ &&
           !HasPendingControl(LIFECYCLE_EVENTS) && (ep0 || data_enabled_);
  }
  /// Owner：是否有端点的硬件停止尚未确认。
  /// Owner: whether any endpoint's hardware stop is not yet confirmed.
  bool AnyStopping() const
  {
    return stopping_count_.load(std::memory_order_relaxed) != 0U;
  }
  /// 登记一个等待硬件确认的端点停止；第一个登记时重置帧计数 / Register one endpoint
  /// stop awaiting hardware confirmation; the first one resets the frame counter
  void AddStopping()
  {
    if (stopping_count_.fetch_add(1U, std::memory_order_relaxed) == 0U)
    {
      stop_frames_.store(0U, std::memory_order_relaxed);
      stop_last_frame_.store(NO_FRAME, std::memory_order_relaxed);
    }
  }
  /// IRQ（总线节拍）：当一次停止已等待 STOP_FRAME_LIMIT 个 1 ms 帧时返回一次 true。
  /// 高速总线每个微帧产生一次节拍，只有新的帧号才计数。0xFFFF 表示帧号未知（没有帧计数器
  /// 的全速控制器）：每次节拍算一帧。
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
  /// Owner：生命周期事件被暂缓；期间任何端点都不得启动。
  /// Owner: a lifecycle event is held back; no endpoint may start meanwhile.
  void SetLifecycleHold(bool hold) { lifecycle_hold_ = hold; }
  /// 一个端点的停止已确认；最后一个确认时投递 STOP_DONE_EVENT / One endpoint's stop was
  /// confirmed; the last one posts STOP_DONE_EVENT
  void StopFinished(bool in_isr);

  /// 数据端点（非端点 0）是否允许启动 / Whether data endpoints (not EP0) may start
  bool DataEnabled() const { return data_enabled_; }
  /// 设置数据端点允许启动；置 true 时唤醒所有端点重新检查队列 / Allow or forbid data
  /// endpoints to start; true makes every endpoint recheck its queue
  void SetDataEnabled(bool enabled, bool in_isr);
  /// 是否正在规划配置（此时端点不得启动）/ Whether a configuration is being planned
  /// (endpoints may not start meanwhile)
  bool IsPlanning() const { return planning_; }
  /// 设置配置规划标志 / Set the configuration-planning flag
  void SetPlanning(bool planning) { planning_ = planning; }
  /// 当前总线速度 / Current bus speed
  Speed GetSpeed() const { return speed_; }
  /// 记录当前总线速度 / Record the current bus speed
  void SetSpeed(Speed speed) { speed_ = speed; }
  /// 已分配的数据端点中是否没有处于 ERROR 状态者 / Whether no allocated data endpoint
  /// is in the ERROR state
  bool ConfigurationValid() const;
  /// 总线是否处于挂起 / Whether the bus is suspended
  bool IsSuspended() const { return suspended_; }
  /// 设置挂起标志；退出挂起时唤醒所有端点重新检查队列 / Set the suspend flag; leaving
  /// suspend makes every endpoint recheck its queue
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
  /// IRQ 批处理期间记录了工作 / Work was recorded during an IRQ batch.
  std::atomic<uint32_t> irq_deferred_{0};
  SerializedService service_;
  std::atomic<uint32_t> control_pending_{0};
  // 有待处理工作的端点：位 2n 为 EP n OUT（或双向 EP n），位 2n + 1 为 EP n IN。
  // Endpoints with work: bit 2n is EP n OUT (or a bidirectional EP n), 2n + 1 is EP n IN.
  std::atomic<uint32_t> endpoint_pending_{0};
  Endpoint* by_bit_[2 * SLOT_COUNT] = {};
  void Register(Endpoint* ep, size_t number, size_t dir);
  Callback<uint32_t> control_handler_;
  /// 仅由 owner 修改 / Changed by the owner only.
  std::atomic<uint32_t> stopping_count_{0};
  std::atomic<uint32_t> stop_frames_{0};
  static constexpr uint32_t NO_FRAME = UINT32_MAX;
  /// 上一次计数时的帧号 / Frame number of the last count.
  std::atomic<uint32_t> stop_last_frame_{NO_FRAME};
  /// 控制器 owner 状态 / Controller-owner state.
  bool lifecycle_hold_ = false;
  bool data_enabled_ = true;
  bool planning_ = false;
  Speed speed_ = Speed::FULL;
  /// 控制器 owner 状态 / Controller-owner state.
  bool suspended_ = false;
  Endpoint* ep0_in_ = nullptr;   ///< 端点0 IN对象 / Endpoint 0 IN pointer
  Endpoint* ep0_out_ = nullptr;  ///< 端点0 OUT对象 / Endpoint 0 OUT pointer
};

// 请求路径上需要池的端点辅助函数；ep.cpp 将其内联。
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
  // 与 Service 中的栅栏配对：要么这里看到 servicing_ 已清除并发出通知，
  // 要么 owner 的 StartNext 看到刚入队的请求。
  // Pairs with the fence in Service: either this sees servicing_ cleared and notifies,
  // or the owner's StartNext sees the request just queued.
  std::atomic_thread_fence(std::memory_order_seq_cst);
  if (servicing_.load(std::memory_order_relaxed) != 0U) return;
  NotifyOwner(in_isr);
}

}  // namespace LibXR::USB
