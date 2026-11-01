#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

#include "core.hpp"
#include "libxr_cb.hpp"
#include "libxr_def.hpp"
#include "libxr_type.hpp"
#include "usb_macro_guard.hpp"

namespace LibXR::USB
{

class EndpointPool;

/**
 * @class Endpoint
 * @brief USB 端点基类 / USB endpoint base class
 *
 * 端点的内存由 BSP
 * 提供，构造时分成固定的一个或两个槽，之后不随包长和速度变化。取槽、往槽里
 * 写数据、提交、归还可以在任何上下文进行：取槽和提交各是一次对端点状态字的原子修改，中间的
 * 拷贝不加锁。借出的槽只能由持有者归还；端点解绑时借出的槽作废，持有者之后提交得到
 * `ABORTED`。启动硬件、回收完成、解绑与停止只在设备的串行化执行者（owner）中进行。
 *
 * Endpoint memory is supplied by the BSP and split into one or two slots at construction;
 * the split does not follow the packet size or the bus speed. Acquiring, filling,
 * submitting and returning a slot work in any context: acquire and submit are one atomic
 * update of the endpoint state word each, and the copy in between takes no lock. A lent
 * slot is only ever returned by its holder; unbinding marks it stale and a later submit
 * returns `ABORTED`. Hardware start, completion retirement, unbinding and stopping run
 * only in the device's serialized executor (owner).
 */
class Endpoint
{
 public:
  /// 端点方向 / Endpoint direction
  enum class Direction : uint8_t
  {
    OUT = 0,  ///< 输出方向 / OUT direction
    IN = 1,   ///< 输入方向 / IN direction
    BOTH = 2  ///< 双向（可配置为 IN/OUT） / Both (configurable as IN/OUT)
  };

  /// 端点号 / Endpoint number
  enum class EPNumber : uint8_t
  {
    EP0 = 0,
    EP1 = 1,
    EP2 = 2,
    EP3 = 3,
    EP4 = 4,
    EP5 = 5,
    EP6 = 6,
    EP7 = 7,
    EP8 = 8,
    EP9 = 9,
    EP10 = 10,
    EP11 = 11,
    EP12 = 12,
    EP13 = 13,
    EP14 = 14,
    EP15 = 15,
    EP_MAX_NUM = 16,   ///< 端点数量上限 / Maximum number of endpoints
    EP_INVALID = 0xFF  ///< 非法端点号 / Invalid endpoint
  };

  /// 端点类型 / Endpoint type
  enum class Type : uint8_t
  {
    CONTROL = 0,      ///< 控制端点 / Control
    ISOCHRONOUS = 1,  ///< 等时端点 / Isochronous
    BULK = 2,         ///< 批量端点 / Bulk
    INTERRUPT = 3     ///< 中断端点 / Interrupt
  };

  /// 端点状态；ERROR 只表示 Configure 失败 / Endpoint state; ERROR only reports a failed
  /// Configure
  enum class State : uint8_t
  {
    DISABLED,  ///< 未配置 / Not configured
    IDLE,      ///< 已配置 / Configured
    STALLED,   ///< 已置 STALL / Halted
    ERROR      ///< 配置失败 / Configuration failed
  };

  /// 后端停止结果 / Backend stop result
  enum class StopResult : uint8_t
  {
    DONE,    ///< 硬件已经停止 / Hardware already stopped
    PENDING  ///< 等硬件事件后调用 OnHardwareStopped / OnHardwareStopped follows later
  };

  static constexpr uint8_t EPNumberToInt8(EPNumber ep)
  {
    return static_cast<uint8_t>(ep);
  }

  static constexpr uint8_t EPNumberToAddr(EPNumber ep, Direction dir)
  {
    ASSERT(dir == Direction::IN || dir == Direction::OUT);
    return static_cast<uint8_t>(ep) | (dir == Direction::IN ? 0x80 : 0x00);
  }

  static constexpr EPNumber AddrToEPNumber(uint8_t addr, Direction& dir)
  {
    dir = addr & 0x80 ? Direction::IN : Direction::OUT;
    return static_cast<EPNumber>(addr & 0x7F);
  }

  static constexpr EPNumber NextEPNumber(EPNumber ep)
  {
    ASSERT(ep <= EPNumber::EP15);
    return static_cast<EPNumber>(EPNumberToInt8(ep) + 1);
  }

  /// 端点配置 / Endpoint configuration
  struct Config
  {
    Direction direction = Direction::OUT;
    Type type = Type::BULK;
    /// 最大包长，必须是该类型在当前速度下的合法值；0 无效 / Max packet size; must be
    /// legal for the type at the current speed; 0 is invalid
    uint16_t max_packet_size = 0;
    size_t receive_size = 0;  ///< OUT：ReturnSlot 重新挂接收的长度；0 表示槽容量
  };

  /// Configure 之后读回，到下一次 Configure/Close 有效 / Valid until the next
  /// Configure/Close
  struct Granted
  {
    size_t slot_capacity = 0;  ///< 包长整数倍 / Multiple of the packet size
  };

  /// 槽句柄，只有槽号 / Slot handle; the index is the whole identity
  struct Slot
  {
    RawData data{nullptr, 0};  ///< IN：可写区域；OUT 租约：收到的字节
    uint8_t index = 0xFF;
    bool Valid() const { return index != 0xFF; }
  };

  /// 请求完成 / Request completion
  struct Completion
  {
    ErrorCode result = ErrorCode::OK;  ///< OK/ABORTED/FAILED/OUT_OF_RANGE/SIZE_ERR
    size_t actual = 0;                 ///< IN：主机已取走；OUT：收到
    Slot slot;                         ///< 仅 OUT 且 result == OK：借给持有者的槽
  };

  /**
   * @param number 端点号 / Endpoint number
   * @param avail 可用方向 / Available direction
   * @param slots 槽内存，起点与长度按 HW_CACHE_LINE_SIZE 对齐 / Slot memory, start and
   * length aligned to HW_CACHE_LINE_SIZE
   * @param slot_count 槽数：1 或 2；0 表示 EP0 一个、其余两个。两个槽时内存平分，每个槽的
   * 起点与长度按 HW_CACHE_LINE_SIZE 对齐 / Slot count, 1 or 2; 0 means one for EP0 and
   * two for the others. Two slots split the memory in halves aligned to
   * HW_CACHE_LINE_SIZE
   */
  Endpoint(EPNumber number, Direction avail, RawData slots, uint8_t slot_count = 0);
  virtual ~Endpoint() = default;
  Endpoint(const Endpoint&) = delete;
  Endpoint& operator=(const Endpoint&) = delete;

  EPNumber GetNumber() const { return number_; }
  Direction AvailableDirection() const { return avail_direction_; }
  Direction GetDirection() const
  {
    return GetState() == State::DISABLED ? avail_direction_ : config_.direction;
  }
  uint8_t GetAddress() const
  {
    return GetState() == State::DISABLED ? EPNumberToInt8(number_)
                                         : EPNumberToAddr(number_, config_.direction);
  }
  [[gnu::always_inline]] State GetState() const
  {
    return state_.load(std::memory_order_acquire);
  }
  Type GetType() const { return config_.type; }
  uint16_t MaxPacketSize() const { return config_.max_packet_size; }
  /// 当前速度下槽和后端都放得下的最大合法包长；没有时为 0。只读查询，Configure 仍须显式
  /// 指定包长 / Largest packet size legal at the current speed that the slot and the
  /// backend hold, 0 when there is none. A query only: Configure still takes an explicit
  /// size
  uint16_t LargestPacketSize(Type type) const;
  bool IsStalled() const { return GetState() == State::STALLED; }
  const Granted& GetGranted() const { return granted_; }

  // ---- 配置与生命周期：只在 owner 中调用 / Owner only ----

  /// OK / ARG_ERR（包长非法）/ NO_BUFF（槽装不下一个包）/ STATE_ERR / FAILED
  [[nodiscard]] ErrorCode Configure(const Config& cfg, bool in_isr);
  /// 解绑：停用端点，借出的槽作废，排队与在途请求恰好结束一次 / Unbind
  void Close(bool in_isr);
  /// 结束排队与在途请求，端点保持启用 / End queued and in-flight requests
  void Flush(bool in_isr);
  /// 先结束待决请求再置 STALL / End pending requests, then halt
  [[nodiscard]] ErrorCode Stall(bool in_isr);
  [[nodiscard]] ErrorCode ClearStall(bool in_isr);
  /// 停止确认之前为 true / True until the hardware stop is confirmed
  bool IsStopping() const { return stopping_; }
  /// 立即回收已经记录的硬件完成 / Retire an already captured hardware completion now
  void RetireCompletion(bool in_isr);
  /// 完成回调在 owner 中执行；回调必须浅 / Runs in the owner; must stay shallow
  void SetOnComplete(Callback<const Completion&> callback) { on_complete_ = callback; }

  // ---- IN 借用：任何上下文 / IN borrowing, any context ----

  /// OK / BUSY（无空闲槽）/ STATE_ERR（未启用或方向不符）
  [[nodiscard]] ErrorCode AcquireSlot(Slot& slot, bool in_isr);
  /// OK / ABORTED（槽在解绑前借出，已回到空闲）/ SIZE_ERR / STATE_ERR
  [[nodiscard]] ErrorCode Submit(const Slot& slot, size_t length, bool zlp, bool in_isr);
  /// 不占槽的零长请求 / Zero-length request without a slot
  [[nodiscard]] ErrorCode SubmitZeroLength(bool in_isr);
  /// 归还未提交的 IN 槽或 OUT 租约，不重新挂接收 / Return without rearming
  void ReleaseSlot(const Slot& slot, bool in_isr);

  // ---- OUT 借用 / OUT borrowing ----

  /// 挂一个空闲槽接收；bytes 为包长整数倍 / Arm one free slot; bytes is a packet
  /// multiple
  [[nodiscard]] ErrorCode Receive(size_t bytes, bool in_isr);
  /// 归还租约并按 receive_size 重新挂接收 / Return a lease and rearm it
  [[nodiscard]] ErrorCode ReturnSlot(const Slot& slot, bool in_isr);

 protected:
  /// 一个已提交请求在硬件上的描述 / One submitted request as seen by the backend
  struct HwRequest
  {
    RawData slot{nullptr, 0};  ///< 整个槽；OUT 可收满它 / Whole slot
    size_t length = 0;         ///< IN 发送长度；OUT 请求长度 / Request length
    uint32_t tag = 0;          ///< 完成时原样交回 / Returned with the completion
  };

  // ---- 后端接口：只在 owner 中、硬件守卫内调用 / Backend, owner only, in the guard ----

  /// 应用已校验的配置；planning 模式下不调用 / Apply a validated configuration
  virtual ErrorCode ConfigureHardware(const Config& cfg) = 0;
  /// 关闭端点；只在没有在途请求时调用 / Close; only without an in-flight request
  virtual void CloseHardware(bool in_isr) = 0;
  /// 启动一个请求；同一时刻最多一个 / Start one request; at most one at a time
  virtual ErrorCode StartHardware(const HwRequest& request, bool in_isr) = 0;
  /// 停止在途请求；PENDING 时之后调用 OnHardwareStopped / Stop the in-flight request
  virtual StopResult StopHardware(bool in_isr) = 0;
  /// 停止确认后读硬件事实 / Read the hardware facts after a confirmed stop
  virtual size_t ReapProgress(const HwRequest& request, bool& retired) = 0;
  virtual ErrorCode StallHardware(bool in_isr) = 0;
  virtual ErrorCode ClearStallHardware(bool in_isr) = 0;
  /// 后端能接受的最大包长（FIFO/PMA）/ Largest packet the backend accepts
  virtual size_t MaxPacketSizeLimit(Type type) const
  {
    UNUSED(type);
    return UINT16_MAX;
  }

  // ---- IRQ 侧：只记录 / IRQ side, record only ----

  /// 只接受当前在途请求的标签 / Accepts only the current in-flight tag
  void OnHardwareComplete(bool in_isr, uint32_t tag, size_t actual,
                          ErrorCode result = ErrorCode::OK);
  void OnHardwareStopped(bool in_isr);

  bool IsPlanning() const;
  const Config& GetConfig() const { return config_; }
  RawData SlotMemory() const { return memory_; }

 private:
  friend class EndpointPool;

  enum SlotState : uint32_t
  {
    FREE = 0,
    LENT = 1,
    STALE = 2,
    QUEUED = 3,
    INFLIGHT = 4
  };

  // State word: slot 0 [2:0], slot 1 [5:3], head [6] (slot 1 was queued first), enabled
  // [7], zero-length queued [8], reserved [9], zero-length in flight [10], reserved
  // [13:11], IN [14], reserved [15], binding generation [31:16]. Everything a
  // non-owner caller decides on is in this word; the generation makes a CAS fail across a
  // rebinding. A slot keeps its place across bindings, so a stale slot only ever blocks
  // itself.
  static constexpr uint32_t SLOT_BITS = 3U;
  static constexpr uint32_t SLOT_MASK = 7U;
  static constexpr uint32_t SLOT_FIELDS = (SLOT_MASK << SLOT_BITS) | SLOT_MASK;
  static constexpr uint32_t HEAD_BIT = 1U << 6U;
  static constexpr uint32_t ENABLED_BIT = 1U << 7U;
  static constexpr uint32_t ZLP_QUEUED_BIT = 1U << 8U;
  static constexpr uint32_t ZLP_INFLIGHT_BIT = 1U << 10U;
  static constexpr uint32_t DIR_IN_BIT = 1U << 14U;
  static constexpr uint32_t GEN_ONE = 1U << 16U;
  static constexpr uint8_t ZLP_INDEX = 2U;
  static constexpr uint8_t NO_REQUEST = 0xFFU;

  // The small helpers on the request path are forced inline: under -Os the compiler keeps
  // them out of line, and on flash with wait states each call costs more than its body.
  [[gnu::always_inline]] static uint32_t SlotOf(uint32_t word, uint8_t index)
  {
    return (word >> (index * SLOT_BITS)) & SLOT_MASK;
  }
  [[gnu::always_inline]] static uint32_t WithSlot(uint32_t word, uint8_t index,
                                                  uint32_t state)
  {
    return (word & ~(SLOT_MASK << (index * SLOT_BITS))) | (state << (index * SLOT_BITS));
  }
  [[gnu::always_inline]] static uint32_t Enqueue(uint32_t word, uint8_t index)
  {
    const uint8_t other = index ^ 1U;
    const uint8_t head = SlotOf(word, other) == QUEUED ? other : index;
    word = WithSlot(word, index, QUEUED);
    return head == 1U ? (word | HEAD_BIT) : (word & ~HEAD_BIT);
  }

  struct SlotMeta
  {
    size_t length = 0;
    bool zlp = false;
  };

  void Attach(EndpointPool& pool) { pool_ = &pool; }
  void Service(bool in_isr);
  // Defined in ep_pool.hpp, which ep.cpp includes.
  [[gnu::always_inline]] inline void NotifyOwner(bool in_isr);
  [[gnu::always_inline]] inline void NotifyQueued(bool in_isr);
  [[gnu::always_inline]] inline bool CanStart() const;
  [[gnu::always_inline]] bool Usable(uint32_t word, uint8_t index) const
  {
    return index < slot_count_ && SlotOf(word, index) == FREE;
  }
  [[gnu::always_inline]] RawData SlotData(uint8_t index, size_t capacity) const
  {
    return {static_cast<uint8_t*>(memory_.addr_) + index * slot_size_, capacity};
  }
  void StartNext(bool in_isr);
  bool StartOne(bool in_isr, bool defer_failure);
  bool StartEarly(bool in_isr);
  void FinishFailedStart(bool in_isr);
  void StartRequest(bool in_isr, uint8_t index, size_t length, bool defer_failure);
  void RetireMailbox(bool in_isr);
  void FinishInflight(bool in_isr, ErrorCode result, size_t actual);
  void EndRequests(bool in_isr, bool disable);
  void FinishStop(bool in_isr);
  void ApplyAfterStop(bool in_isr);
  void DeliverAborts(bool in_isr);
  void Deliver(bool in_isr, ErrorCode result, size_t actual);
  void Deliver(bool in_isr, ErrorCode result, size_t actual, const Slot& slot);
  [[gnu::always_inline]] uint32_t NextTag(uint8_t index)
  {
    sequence_ = (sequence_ + 1U) & 0x3FFFFFFFU;
    if (sequence_ == 0U) sequence_ = 1U;
    return (sequence_ << 2U) | index;
  }
  void SetState(State state) { state_.store(state, std::memory_order_release); }
  // Requests are being ended (stop pending, halted or closed); nothing continues.
  [[gnu::always_inline]] bool Ending() const
  {
    return stopping_ || GetState() != State::IDLE;
  }

  EPNumber number_;
  Direction avail_direction_;
  RawData memory_;
  uint8_t slot_count_ = 1;  // Fixed at construction.
  size_t slot_size_ = 0;  // Bytes of each slot; slot 1 starts here. Cache-line multiple.
  Config config_;
  Granted granted_;
  std::atomic<State> state_{State::DISABLED};
  EndpointPool* pool_ = nullptr;

  // Written only while disabled; read by other contexts between loading the state word
  // and the CAS that validates it.
  std::atomic<uint32_t> word_{0};
  std::atomic<uint32_t> capacity_{0};
  std::atomic<uint32_t> packet_{0};
  std::atomic<uint32_t> receive_size_{0};
  SlotMeta meta_[2];

  // Owner-only request state.
  uint8_t inflight_ = NO_REQUEST;
  bool inflight_tail_ =
      false;  // The ZLP tail of the in-flight slot request is on the wire.
  HwRequest inflight_request_;
  uint32_t sequence_ = 0;
  uint8_t pending_aborts_ = 0;  // Queued requests ended behind the in-flight one.
  bool stopping_ = false;
  bool close_after_stop_ = false;
  bool stall_after_stop_ = false;
  bool start_before_deliver_ = false;
  bool start_failed_ = false;  // An early start failed; its FAILED completion is pending.

  // IRQ producer, owner consumer. Word-sized for Cortex-M0 atomics.
  std::atomic<uint32_t> inflight_tag_{0};
  std::atomic<uint32_t> completion_ready_{0};
  uint32_t completion_tag_ = 0;
  size_t completion_actual_ = 0;
  ErrorCode completion_result_ = ErrorCode::OK;
  std::atomic<uint32_t> stop_confirmed_{0};
  uint32_t pending_bit_ = 0;  // This endpoint's bit in the pool's pending mask.
  // Set by the owner while it retires this endpoint's completion (callback included);
  // a request queued meanwhile is started by the StartNext that follows.
  std::atomic<uint32_t> servicing_{0};

  Callback<const Completion&> on_complete_;
};

}  // namespace LibXR::USB
