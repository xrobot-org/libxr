#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

#include "core.hpp"
#include "ep_slots.hpp"
#include "libxr_cb.hpp"
#include "libxr_def.hpp"
#include "libxr_type.hpp"

namespace LibXR::USB
{

class DeviceService;

/**
 * @class Endpoint
 * @brief USB 端点基类 / USB endpoint base class
 *
 * 端点内存由 BSP 提供，构造时分成固定的若干个槽，个数由后端给定。设备类借槽、写入、
 * 提交（IN），
 * 或挂接收、在完成回调里拿到装着数据的槽、用完交回（OUT）；这些调用可在任何上下文进行，
 * 槽的记账由 EndpointSlots 无锁完成。访问寄存器、启动和停止硬件、处理完成、结束请求只在
 * 设备的串行执行者（owner）中进行：owner 是赢得设备串行服务的调用者，即控制器中断，或
 * 经由 Submit、Receive 等请求服务的应用线程。
 * Endpoint memory comes from the BSP and is split at construction into a fixed number
 * of slots, which the backend chooses. Device classes borrow, fill and submit a slot
 * (IN), or arm a receive,
 * get the slot holding the bytes in the completion callback and hand it back (OUT);
 * these calls work in any context and EndpointSlots keeps the slot bookkeeping without
 * locks. Register access, starting and stopping the hardware, handling completions and
 * ending requests happen only in the device's serialized executor (owner): whichever
 * caller wins the device's serialized service, i.e. the controller interrupt or an
 * application thread that requests the service through Submit, Receive and the like.
 *
 * @note 每个被接受的请求恰好得到一次完成回调。回调在 owner 中运行，可能在中断里
 *       （in_isr=true）也可能在应用线程里（in_isr=false），不得阻塞。
 *       Every accepted request gets exactly one completion callback. The callback runs
 *       in the owner, in an interrupt (in_isr=true) or in an application thread
 *       (in_isr=false), and must not block.
 * @note 后端函数（XxxHardware）只由 owner 调用。寄存器访问不加锁，靠串行保证并发安全：
 *       通常都在 owner 中进行；后端在 owner 之外访问寄存器时（例如在控制器中断入口先
 *       收下事件），由后端保证它与 owner 不会同时访问同一个寄存器。线程持有 owner 时，
 *       控制器中断只屏蔽自己的中断线、发布事件后返回，由该线程处理硬件事件；这一步由
 *       后端和设备完成，核心不为后端加锁。
 *       The backend functions (XxxHardware) are called by the owner only. Register
 *       access takes no lock and is safe under concurrency by being serialized: it
 *       normally happens in the owner; where a backend touches registers outside the
 *       owner (such as collecting an event at the entry of the controller interrupt),
 *       the backend makes sure that this and the owner never access the same register
 *       at the same time. While a thread owns, the controller interrupt only masks its
 *       own line, publishes an event and returns, and that thread handles the hardware
 *       events; the backend and the device do this, and the core takes no lock on the
 *       backend's behalf.
 * @note 每次借出都给句柄一个新戳，核心只接受当前这次借出的句柄；重复交回或用更早的
 *       句柄交回得到 STATE_ERR，不影响槽的当前持有者。Close 使借出的槽作废：持有者之后
 *       Submit 得到 ABORTED，ReleaseSlot 得到 OK。
 *       Each lend gives the handle a new stamp and the core accepts only the handle of
 *       the current lend; a double hand-back or one through an older handle gets
 *       STATE_ERR and leaves the slot's current holder alone. Close marks lent slots
 *       stale: the holder's later Submit returns ABORTED and ReleaseSlot returns OK.
 * @note 停止不等待：Close、Flush、Stall 遇到在途请求时发出停止后就返回，每个请求在返回
 *       前都已回调。硬件直接读写槽的端点，被停请求的槽在硬件停稳之前不再借出，其余端点
 *       在停止时就放回。硬件动作（CloseHardware、ConfigureHardware、StallHardware、
 *       ClearStallHardware）和下一个请求的启动推迟到后端报告停止结束；此期间不再装入
 *       新的包。核心和设备核心都不等停止。
 *       Stops do not wait: Close, Flush and Stall issue the stop of an in-flight
 *       request and return, and every request has had its callback by then. On an
 *       endpoint whose hardware reads or writes the slot directly the stopped
 *       request's slot is not lent again until the hardware is quiescent; other
 *       endpoints give it back at the stop. The hardware steps (CloseHardware,
 *       ConfigureHardware, StallHardware, ClearStallHardware) and the start of the next
 *       request wait for the backend's report that the stop ended; no new packet is
 *       loaded meanwhile. Neither this core nor the device core waits for a stop.
 * @note 等时端点没有握手和重传，一个请求是一个服务区间的数据，最多
 *       PacketsPerInterval() 个包（默认一个）：
 *       - IN 的 OK 表示包在它的服务区间里发上了线，不表示主机收对了。
 *       - 服务区间过去而主机没有取走这个包：后端可以以 FAILED 结束这个请求，也可以让它
 *         留在硬件上，到主机下次来取时发出。设备类不能假设是哪一种。
 *       - OUT 的某个服务区间主机没有发包：不报告，请求继续等下一个包；主机发的零长度包
 *         以 OK、actual 为 0 完成。设备类靠数据内容发现缺口。
 *       - OUT 的一个服务区间里主机发几个包就收几个，不要求发满；请求在这个区间的最后
 *         一个包收下时完成，actual 是它们的总字节数。
 *       An isochronous endpoint has no handshake and no retry; one request is the data
 *       of one service interval, at most PacketsPerInterval() packets (one by default):
 *       - OK of an IN means the packet went onto the wire in its service interval, not
 *         that the host received it correctly.
 *       - If the service interval passes and the host has not taken the packet, the
 *         backend may end the request with FAILED, or may leave it on the hardware to be
 *         sent when the host next asks. A device class must not assume either.
 *       - If the host sends no packet in a service interval of an OUT, nothing is
 *         reported and the request keeps waiting for the next packet; a zero-length
 *         packet from the host completes with OK and actual 0. A device class finds
 *         gaps from the data.
 *       - In a service interval of an OUT the packets the host sends are received,
 *         however few; the request completes when the last packet of that interval has
 *         been taken, actual being their total number of bytes.
 */
class Endpoint
{
 public:
  /// @name 标识与配置 / Identity and configuration
  /// @{

  /// 端点方向 / Endpoint direction
  enum class Direction : uint8_t
  {
    OUT = 0,  ///< 输出方向 / OUT direction
    IN = 1,   ///< 输入方向 / IN direction
    BOTH = 2  ///< 双向（可配置为 IN/OUT）/ Both (configurable as IN or OUT)
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
    INTR = 3          ///< 中断端点 / Interrupt
  };

  /// 端点状态 / Endpoint state
  enum class State : uint8_t
  {
    DISABLED,  ///< 未配置或已关闭 / Not configured, or closed
    IDLE,      ///< 已配置 / Configured
    STALLED    ///< 已置 STALL / Halted
  };

  /// 端点号转为整数 / Endpoint number as an integer
  static constexpr uint8_t EPNumberToInt8(EPNumber ep)
  {
    return static_cast<uint8_t>(ep);
  }

  /// 端点号和方向合成端点地址（IN 置 bit7）/ Endpoint address (bit 7 set for IN)
  static constexpr uint8_t EPNumberToAddr(EPNumber ep, Direction dir)
  {
    ASSERT(dir == Direction::IN || dir == Direction::OUT);
    return static_cast<uint8_t>(ep) | (dir == Direction::IN ? 0x80 : 0x00);
  }

  /// 下一个端点号；ep 不得超过 EP15 / The next endpoint number; ep must not exceed EP15
  static constexpr EPNumber NextEPNumber(EPNumber ep)
  {
    ASSERT(ep <= EPNumber::EP15);
    return static_cast<EPNumber>(EPNumberToInt8(ep) + 1);
  }

  /// 端点配置 / Endpoint configuration
  struct Config
  {
    Direction direction = Direction::OUT;  ///< IN 或 OUT / IN or OUT
    Type type = Type::BULK;                ///< 传输类型 / Transfer type
    /// 包长上限，见 Configure / Upper bound of the packet size, see Configure
    uint16_t max_packet_size = 0;
    /// 一个服务区间最多几个包的上限，至少为 1，见 Configure / Upper bound of the
    /// packets in one service interval, at least 1, see Configure
    uint8_t max_packets_per_interval = 1U;
  };

  /**
   * @brief USB 2.0 端点描述符 wMaxPacketSize 字段的值 / Value of wMaxPacketSize in a
   *        USB 2.0 endpoint descriptor
   * @param packet 包长，1..1024 / Packet size, 1..1024
   * @param packets 一个服务区间最多几个包，1..3 / Packets in one service interval,
   *        1..3
   * @note 位 10..0 是包长，位 12..11 是包数减一（USB 2.0 9.6.6）。
   *       Bits 10..0 are the packet size and bits 12..11 the count minus one (USB 2.0
   *       9.6.6).
   */
  static constexpr uint16_t PacketSizeField(uint16_t packet, uint8_t packets)
  {
    ASSERT(packet >= 1U && packet <= 1024U && packets >= 1U && packets <= 3U);
    return static_cast<uint16_t>(packet | ((packets - 1U) << 11U));
  }

  /**
   * @brief 构造端点 / Construct an endpoint
   * @param number 端点号 / Endpoint number
   * @param avail 硬件支持的方向 / Direction the hardware supports
   * @param slots 槽内存，起点与长度按 HW_CACHE_LINE_SIZE 对齐。
   *        Slot memory; start and length aligned to HW_CACHE_LINE_SIZE.
   * @param slot_count 槽数：1 到 MAX_SLOTS；0 表示端点号 0（默认控制管道）一个、其余
   *        两个。内存平分给各个槽，每个槽按 HW_CACHE_LINE_SIZE 对齐。硬件直接读写槽的
   *        端点（见 HwLimits::direct_memory），被停的槽到停止结束才能再借，只有一个槽时
   *        在此期间借槽和 Receive 得到 BUSY；其余端点的被停槽在停止时就放回。
   *        Slot count, 1 to MAX_SLOTS; 0 means one for endpoint number 0 (the default
   *        control pipe) and two for the others. The memory is split evenly, each slot
   *        aligned to HW_CACHE_LINE_SIZE. On an endpoint whose hardware reads or writes
   *        the slot directly (see HwLimits::direct_memory) a stopped slot can be lent
   *        again only when the stop ends, so with one slot AcquireSlot and Receive get
   *        BUSY meanwhile; other endpoints get the stopped slots back at the stop.
   */
  Endpoint(EPNumber number, Direction avail, RawData slots, uint8_t slot_count = 0);
  /// 经基类指针销毁后端端点 / Destroy a backend endpoint through the base class
  virtual ~Endpoint() = default;
  /// 一个端点最多几个槽 / Most slots of one endpoint
  static constexpr uint8_t MAX_SLOTS = EndpointSlots::MAX_SLOTS;
  /// 不可复制：槽内存与请求只属于一个端点 / Not copyable: slots and requests have one
  /// owner
  Endpoint(const Endpoint&) = delete;
  /// 不可复制赋值 / Not copy-assignable
  Endpoint& operator=(const Endpoint&) = delete;

  /// 端点号 / Endpoint number
  EPNumber GetNumber() const { return number_; }
  /// 硬件支持的方向（可为 BOTH）/ Directions the hardware supports (may be BOTH)
  Direction AvailableDirection() const { return avail_direction_; }
  /**
   * @brief 当前方向；DISABLED 时为可用方向 / Current direction; the available one if
   *        DISABLED
   * @note 读 Configure 写入的配置：在 owner 中，或确知没有并发的 Configure 时调用。
   *       Reads the configuration written by Configure: call it in the owner, or when
   *       no Configure can run concurrently.
   */
  Direction GetDirection() const
  {
    return GetState() == State::DISABLED ? avail_direction_ : config_.direction;
  }
  /**
   * @brief 当前端点地址；DISABLED 时只有端点号 / Current address; only the number if
   *        DISABLED
   * @note 调用上下文同 GetDirection / Calling context as for GetDirection
   */
  uint8_t GetAddress() const
  {
    return GetState() == State::DISABLED ? EPNumberToInt8(number_)
                                         : EPNumberToAddr(number_, config_.direction);
  }
  /// 当前状态（acquire 读取，任何上下文）/ Current state (acquire load, any context)
  State GetState() const { return state_.load(std::memory_order_acquire); }
  /// 是否处于 STALLED / Whether the state is STALLED
  bool IsStalled() const { return GetState() == State::STALLED; }
  /// 配置的传输类型；调用上下文同 GetDirection / Configured transfer type; calling
  /// context as for GetDirection
  Type GetType() const { return config_.type; }
  /// Configure 选定的包长（任何上下文）/ Packet size chosen by Configure (any context)
  uint16_t MaxPacketSize() const
  {
    return static_cast<uint16_t>(packet_.load(std::memory_order_relaxed));
  }
  /// Configure 选定的一个服务区间最多几个包（任何上下文）/ Packets in one service
  /// interval chosen by Configure (any context)
  uint8_t PacketsPerInterval() const
  {
    return static_cast<uint8_t>(packet_.load(std::memory_order_relaxed) >>
                                INTERVAL_PACKETS_SHIFT);
  }
  /**
   * @brief 一次请求最多能带的字节数 / Largest number of bytes one request can carry
   * @note 包长的整数倍：槽的大小和后端一次请求的上限中较小的一个，向下取整；控制类型
   *       为一个包；等时类型为一个服务区间，即 PacketsPerInterval() 个包。到下一次
   *       Configure 前有效。
   *       A packet multiple: the smaller of the slot size and the backend's limit for
   *       one request, rounded down; one packet for the control type; one service
   *       interval, that is PacketsPerInterval() packets, for the isochronous type.
   *       Valid until the next Configure.
   */
  size_t MaxTransferSize() const { return capacity_.load(std::memory_order_relaxed); }

  /// @}
  /// @name 槽与完成 / Slot and completion
  /// @{

  /**
   * @brief 槽句柄 / Slot handle
   * @note 槽号和借出戳只由核心写入和读取。默认构造的句柄不指向任何槽。
   *       The slot index and the lend stamp are written and read by the core only. A
   *       default-constructed handle refers to no slot.
   */
  struct Slot
  {
   public:
    /// IN：可写区域；OUT：收到的字节 / IN: writable region; OUT: the received bytes
    RawData data{nullptr, 0};
    /// 是否指向某个槽 / Whether the handle refers to a slot
    bool Valid() const { return index_ != EndpointSlots::NO_SLOT; }

   private:
    friend class Endpoint;
    uint8_t index_ = EndpointSlots::NO_SLOT;  ///< 槽号 / Slot index
    uint32_t stamp_ = 0;                      ///< 借出戳 / Lend stamp
  };

  /**
   * @brief 请求完成 / Request completion
   * @note 等时端点的 OK 和 FAILED 另有含义，见类说明。
   *       For an isochronous endpoint OK and FAILED mean something else, see the class
   *       note.
   * @note result 为 OK、ABORTED、FAILED 或 OUT_OF_RANGE。OUT 且 OK 时 slot 借给回调的
   *       接收者，actual 等于 slot.data.size_，接收者用完后 ReleaseSlot；其余情况 slot
   *       无效，actual 是结束前已传输的字节数。
   *       result is OK, ABORTED, FAILED or OUT_OF_RANGE. For OUT with OK, slot is lent
   *       to the receiver of the callback, actual equals slot.data.size_, and the
   *       receiver calls ReleaseSlot when done; otherwise slot is invalid and actual is
   *       the number of bytes moved before the request ended.
   */
  struct Completion
  {
    ErrorCode result = ErrorCode::OK;  ///< 请求结果 / Request result
    size_t actual = 0;  ///< IN：主机取走；OUT：收到 / IN: taken by host; OUT: received
    Slot slot;          ///< 仅 OUT 且 OK 时有效 / Valid for OUT with OK only
  };

  /// @}
  /// @name 设备类的数据接口：IN / Data interface for device classes: IN
  /// @{

  /**
   * @brief 借一个空闲槽用于发送 / Borrow a free slot for sending
   * @param slot 成功时写入句柄，data 覆盖整个槽容量。
   *        Receives the handle on success; data spans the slot capacity.
   * @return OK；BUSY：没有空闲槽；STATE_ERR：未启用或不是 IN。
   *         OK; BUSY: no free slot; STATE_ERR: not enabled, or not IN.
   * @note 任何上下文；不运行 owner，不触发回调。之后以一次成功的 Submit 或 ReleaseSlot
   *       交回。STALLED 时也可借。
   *       Any context; does not run the owner and triggers no callback. Hand the slot
   *       back with one successful Submit or ReleaseSlot. Works while STALLED too.
   */
  [[nodiscard]] ErrorCode AcquireSlot(Slot& slot);

  /**
   * @brief 提交借到的槽 / Submit a borrowed slot
   * @param slot 当前这次借出的句柄 / Handle of the current lend
   * @param length 发送槽内前 length 字节；0 发送一个零长度包。
   *        Sends the first length bytes of the slot; 0 sends one zero-length packet.
   * @param zlp true 且 length 是 MaxPacketSize() 的非零整数倍时，数据之后再发一个结尾
   *        零长度包，两者是同一个请求；其他情况不起作用。
   *        When true and length is a non-zero multiple of MaxPacketSize(), a closing
   *        zero-length packet follows the data within the same request; no effect
   *        otherwise.
   * @param in_isr 是否在中断中 / Whether in an ISR
   * @return OK；ABORTED：槽在 Close 之前借出，现已空闲；SIZE_ERR：length 超过
   *         MaxTransferSize()，槽仍归持有者；STATE_ERR：句柄不是当前这次借出（已交回、
   *         已提交或来自更早的借出）或端点不是 IN，槽不受影响。
   *         OK; ABORTED: the slot was lent before a Close and is free now; SIZE_ERR:
   *         length exceeds MaxTransferSize() and the holder keeps the slot; STATE_ERR:
   *         the handle is not the current lend (handed back, submitted or from an
   *         earlier lend) or the endpoint is not IN, and the slot is left alone.
   * @note 任何上下文；可能就地运行 owner，或挂起控制器中断让 owner 在中断里运行，完成
   *       回调可能在返回前运行。STALLED 时请求排队，ClearStall 后启动。
   *       Any context; may run the owner in place, or pend the controller interrupt so
   *       that the owner runs there, so the completion callback can run before this
   *       returns. While STALLED the request waits in the queue until ClearStall.
   * @note 带结尾零长度包的请求只完成一次，OK 时 actual 为 length。数据发完、零长度包
   *       还没发时被 Close、Flush 或 Stall 停止，则不再发零长度包，请求以 ABORTED
   *       完成，actual 为 length。
   *       A request with a closing zero-length packet completes once, with actual equal
   *       to length on OK. If Close, Flush or Stall stops it after the data and before
   *       the zero-length packet, that packet is not sent and the request completes
   *       with ABORTED and actual equal to length.
   */
  [[nodiscard]] ErrorCode Submit(const Slot& slot, size_t length, bool zlp, bool in_isr);

  /**
   * @brief 交回未提交的 IN 槽或 OUT 收到数据的槽 / Hand back an unsubmitted IN slot or
   *        a received OUT slot
   * @param slot 当前这次借出的句柄 / Handle of the current lend
   * @return OK（包括 Close 后作废的槽）；STATE_ERR：句柄不是当前这次借出，槽不受影响。
   *         OK (a slot made stale by Close included); STATE_ERR: the handle is not the
   *         current lend, and the slot is left alone.
   * @note 任何上下文；不运行 owner，不触发回调。OUT 交回后以 Receive 重新挂接收。
   *       Any context; does not run the owner and triggers no callback. For OUT, arm
   *       again with Receive.
   */
  ErrorCode ReleaseSlot(const Slot& slot);

  /// @}
  /// @name 设备类的数据接口：OUT / Data interface for device classes: OUT
  /// @{

  /**
   * @brief 挂一个空闲槽接收 / Arm a free slot for receiving
   * @param bytes 包长整数倍，不超过 MaxTransferSize()；0 只用于控制类型（状态阶段）。
   *        A packet multiple not above MaxTransferSize(); 0 only for the control type
   *        (the status stage).
   * @param in_isr 是否在中断中 / Whether in an ISR
   * @return OK；BUSY：没有空闲槽；SIZE_ERR：bytes 不合法；STATE_ERR：未启用、不是 OUT，
   *         或挂接收期间被 Close。
   *         OK; BUSY: no free slot; SIZE_ERR: illegal bytes; STATE_ERR: not enabled,
   *         not OUT, or closed while arming.
   * @note 任何上下文；与 Submit 一样请求 owner。完成时 Completion::slot 借给回调的
   *       接收者，用完后 ReleaseSlot，再 Receive。
   *       Any context; requests the owner as Submit does. On completion
   *       Completion::slot is lent to the receiver of the callback, which calls
   *       ReleaseSlot when done and then Receive again.
   */
  [[nodiscard]] ErrorCode Receive(size_t bytes, bool in_isr);

  /// @}
  /// @name 生命周期：仅 owner / Lifecycle: owner only
  /// @{

  /**
   * @brief 配置并启用端点 / Configure and enable the endpoint
   * @param cfg 方向、类型、包长上限和一个服务区间的包数上限 / Direction, type and the
   *        upper bounds of the packet size and of the packets in one service interval
   * @param in_isr 是否在中断中 / Whether in an ISR
   * @return OK；STATE_ERR：不是 DISABLED，或停止失败、等待控制器复位（见 Close）；
   *         ARG_ERR：方向非法，包数上限为 0，或不超过上限的包长对该类型和速度都不合法；
   *         NO_BUFF：合法包长都放不进槽或超过后端上限，或硬件直接读写槽、只有一个槽而
   *         类型不是控制；其他：后端的 CheckHardware 拒绝了这个配置，返回它的错误。
   *         OK; STATE_ERR: not DISABLED, or a stop failed and the controller reset is
   *         due (see Close); ARG_ERR: illegal direction, a packet-count bound of 0, or
   *         no size up to the bound is legal for the type and speed; NO_BUFF: no legal
   *         size fits the slot and the backend limit, or the hardware reads or writes
   *         the slot directly, there is one slot and the type is not control; otherwise
   *         the backend's CheckHardware refused the configuration and this is its error.
   * @note 包长取不超过 cfg.max_packet_size、对类型和当前速度合法、放得进一个槽且不超过
   *       后端最大包长（HardwareLimits）的最大值，用 MaxPacketSize() 读回。失败时端点
   *       保持 DISABLED，之前的配置不受影响。
   *       The packet size is the largest one not above cfg.max_packet_size that is
   *       legal for the type and the current speed, fits one slot and does not exceed
   *       the backend's largest packet (HardwareLimits); read it back with
   *       MaxPacketSize(). On failure the endpoint stays DISABLED and is otherwise
   *       unchanged.
   * @note 一个服务区间的包数在包长选定之后选：取不超过 cfg.max_packets_per_interval、
   *       对类型、速度和选定的包长合法、且不超过后端上限（HardwareLimits 的
   *       max_packets_per_interval）的最大值，用 PacketsPerInterval() 读回。只有高速的
   *       等时和中断端点可以大于 1（最多 3；2 个包要求包长至少 513，3 个包至少 683，
   *       USB 2.0 表 9-14）。等时类型的一个请求就是一个服务区间，所以它的包数还要让整个
   *       区间放得进一个槽，并且不超过后端一次请求的上限。
   *       The packets in one service interval are chosen after the packet size: the
   *       largest count not above cfg.max_packets_per_interval that is legal for the
   *       type, the speed and the chosen packet size and does not exceed the backend
   *       limit (max_packets_per_interval of HardwareLimits); read it back with
   *       PacketsPerInterval(). Only high-speed isochronous and interrupt endpoints may
   *       have more than 1 (at most 3; 2 packets need a packet size of at least 513 and
   *       3 packets of at least 683, USB 2.0 Table 9-14). One request of the isochronous
   *       type is one service interval, so its count also lets the whole interval fit
   *       one slot and stay within the backend's limit for one request.
   * @note 上一次 Close 的停止未结束时也返回 OK，不等待：状态、包长、MaxTransferSize()
   *       和借槽当场生效，硬件直接读写槽的端点的被停槽在停止结束前不借出。CloseHardware
   *       和 ConfigureHardware 在停止结束时依次调用，此前提交的请求排队，之后启动。
   *       返回值在任何情况下都是最终结果：能否配置在本函数内全部判定完，
   *       ConfigureHardware 不会失败。
   *       Returns OK without waiting also while the stop of the last Close has not
   *       ended: the state, the packet size, MaxTransferSize() and lending take effect
   *       at once, and on an endpoint whose hardware reads or writes the slot directly
   *       the stopped slot is not lent until the stop ends; CloseHardware
   *       and ConfigureHardware run in this order when it ends, and requests submitted
   *       before then are queued and start afterwards. The return value is final in
   *       every case: whether the configuration is possible is decided in full inside
   *       this function, and ConfigureHardware cannot fail.
   */
  [[nodiscard]] ErrorCode Configure(const Config& cfg, bool in_isr);

  /**
   * @brief 关闭（解绑）端点 / Close (unbind) the endpoint
   * @param in_isr 是否在中断中 / Whether in an ISR
   * @note 返回时：状态为 DISABLED，借出的槽已作废，每个请求都已回调。顺序：
   *       StopHardware；它当场给出结果时 CloseHardware；在途请求的回调；排队请求各一次
   *       ABORTED。已是 DISABLED 时不做任何事。
   *       On return the state is DISABLED, lent slots are stale and every request has
   *       had its callback. Order: StopHardware; CloseHardware if it gave the outcome at
   *       once; the callback of the in-flight request; one ABORTED per queued request.
   *       Does nothing when already DISABLED.
   * @note 在途请求的回调：StopHardware 当场给出结果时按结果交付（硬件已做完的 IN 为
   *       OK）；返回 PENDING 时当场以 ABORTED 交付，actual 为此刻已知的字节数（结尾
   *       零长度包在途时为数据长度，否则 0）；停止结束时不再回调，硬件在停止期间做完了
   *       也丢弃结果。所以被停 IN 请求的 actual 是下界：停止结束之前硬件还可能把更多
   *       数据发给主机，类不能靠它从断点精确续发。被停的 OUT 请求和还欠结尾零长度包的
   *       IN 请求总是以 ABORTED 完成，不借出槽。
   *       The in-flight request's callback: from the outcome when StopHardware gives it
   *       at once (OK for an IN the hardware had finished); with ABORTED at once when it
   *       returns PENDING, actual being the bytes known now (the data length while the
   *       closing zero-length packet is on the wire, else 0); no callback when the stop
   *       ends, the result being dropped even if the hardware finished the request
   *       during the stop. The actual of a stopped IN request is therefore a lower
   *       bound: until the stop ends the hardware may still send more data to the host,
   *       and a class cannot resume exactly from it. A stopped OUT request, and an IN
   *       request still owing its closing zero-length packet, always completes with
   *       ABORTED and lends no slot.
   * @note 停止挂起时：硬件直接读写槽的端点，被停的槽保持在途、不借出；其余端点的被停
   *       槽当场放回。CloseHardware 在后端报告停止结束时调用，
   *       之前推迟的 ClearStallHardware 作废。报告为 TIMEOUT 时停止失败：设备收到
   *       STOP_FAILED_EVENT，控制器复位后由设备服务结束这个停止（见
   *       DeviceService::CloseAllEndpoints）。
   *       While the stop is pending, on an endpoint whose hardware reads or writes the
   *       slot directly the stopped slot stays in flight and is not lent; other
   *       endpoints get it back at once. CloseHardware runs when the backend reports the
   *       end of the stop, and a
   *       ClearStallHardware deferred earlier is dropped. A report of TIMEOUT means the
   *       stop failed: the device gets STOP_FAILED_EVENT and the device service ends
   *       the stop after the controller reset (see DeviceService::CloseAllEndpoints).
   * @note 回调里可以重新 Configure 本端点并 SetOnComplete；之后交付的排队请求的
   *       ABORTED 仍属于旧的绑定，进入调用时设置的回调。所有回调都在本函数返回前进行，
   *       不会进入之后的绑定。
   *       A callback may Configure this endpoint again and SetOnComplete; the ABORTED
   *       of queued requests delivered after that still belongs to the old binding and
   *       goes to the callback set at that time. Every callback runs before this
   *       returns and never reaches a later binding.
   */
  void Close(bool in_isr);

  /**
   * @brief 结束全部请求，端点保持启用 / End all requests; the endpoint stays enabled
   * @param in_isr 是否在中断中 / Whether in an ISR
   * @note 返回时：状态不变，每个请求都已回调，顺序和在途请求的结果同 Close（没有
   *       CloseHardware）。之后提交的请求（包括回调里提交的）排队，停止结束后由 owner 在
   *       端点阶段启动。已有停止未结束时只结束排队请求。DISABLED 时不做任何事。
   *       On return the state is unchanged and every request has had its callback, in
   *       the order and with the in-flight result of Close (without CloseHardware).
   *       Requests submitted afterwards, from those callbacks included, are queued and
   *       the owner starts them in its endpoint step once the stop has ended. With a
   *       stop already pending only the queued requests end. Does nothing while
   *       DISABLED.
   */
  void Flush(bool in_isr);

  /**
   * @brief 结束全部请求并置 STALL / End all requests, then halt
   * @param in_isr 是否在中断中 / Whether in an ISR
   * @return OK（已是 STALLED 也为 OK）；STATE_ERR：DISABLED。
   *         OK (also when already STALLED); STATE_ERR: DISABLED.
   * @note 返回时：状态为 STALLED，每个请求都已回调，顺序同 Close，CloseHardware 换成
   *       StallHardware。没有停止未结束时 StallHardware 已调用；否则它在停止结束时调用
   *       （其间 ClearStall 或 Close 了则不调用），此前硬件还没有置 STALL。STALLED 期间
   *       提交的请求排队，ClearStall 后启动。
   *       On return the state is STALLED and every request has had its callback, in
   *       the order of Close with StallHardware in place of CloseHardware.
   *       StallHardware has run unless a stop is pending; then it runs when the stop
   *       ends (not at all if ClearStall or Close came in between) and the hardware is
   *       not halted until then. Requests submitted while STALLED wait for ClearStall.
   */
  [[nodiscard]] ErrorCode Stall(bool in_isr);

  /**
   * @brief 清除 STALL 回到 IDLE / Clear the halt and return to IDLE
   * @param in_isr 是否在中断中 / Whether in an ISR
   * @return OK；STATE_ERR：DISABLED；其他：当场调用的 ClearStallHardware 的错误。
   *         OK; STATE_ERR: DISABLED; otherwise the error of a ClearStallHardware called
   *         in place.
   * @note 没有停止未结束时：返回 OK 时 ClearStallHardware 已调用，状态为 IDLE，排队
   *       请求已交给 owner 启动（可能在返回前）；ClearStallHardware 失败时状态不变，排队
   *       请求不启动。
   *       With no stop pending: on OK, ClearStallHardware has run, the state is IDLE
   *       and queued requests are handed to the owner to start (possibly before this
   *       returns); if ClearStallHardware fails the state is unchanged and nothing
   *       starts.
   * @note 停止未结束时：状态当场变为 IDLE 并返回 OK；ClearStallHardware 在停止结束时
   *       调用（数据翻转位回到 DATA0），排队请求随后启动。那时它的失败只能忽略。
   *       With a stop pending: the state becomes IDLE at once and OK is returned;
   *       ClearStallHardware runs when the stop ends (the data toggle returns to DATA0)
   *       and queued requests start afterwards. Its failure then can only be ignored.
   */
  [[nodiscard]] ErrorCode ClearStall(bool in_isr);

  /**
   * @brief 设置完成回调 / Set the completion callback
   * @param callback 每个请求调用一次，见 Completion / Called once per request
   * @note 绑定时设置，回调保留到下一次设置。
   *       Set at binding; the callback stays until the next one is set.
   */
  void SetOnComplete(Callback<const Completion&> callback) { on_complete_ = callback; }

  /// @}

 protected:
  /// @name 后端接口 / Backend interface
  /// @{

  /**
   * @brief 一个请求在硬件上的描述 / One request as seen by the backend
   * @note 从 data 起，length 按缓存行大小向上取整的字节都可写，后端可以对这个范围做缓存
   *       维护，DMA 按字写入超出 length 的几个字节也落在这个范围里。
   *       From data on, length rounded up to the cache line size is writable; the
   *       backend may do cache maintenance on that range, and the few bytes a DMA
   *       writing whole words puts beyond length fall inside it too.
   */
  struct HwRequest
  {
    uint8_t* data = nullptr;  ///< 数据起始地址 / Start of the data
    size_t length = 0;        ///< IN：发送长度；OUT：接收长度 / Length to send/receive
  };

  /**
   * @brief 后端硬件的上限 / Limits of the backend hardware
   * @note 默认值表示没有限制。一次请求的长度同时受 max_packets 和 max_bytes 限制。
   *       max_packets 至少为 1，max_bytes 不小于 max_packet_size（核心断言）。
   *       The defaults mean no limit. The length of one request is bound by both
   *       max_packets and max_bytes. max_packets is at least 1 and max_bytes is not
   *       below max_packet_size (asserted by the core).
   */
  struct HwLimits
  {
    size_t max_packet_size = UINT16_MAX;  ///< 最大包长 / Largest packet
    /// 一次请求最多几个包；等时类型的一次请求是一个服务区间 / Packets in one request;
    /// one request of the isochronous type is one service interval
    size_t max_packets = SIZE_MAX;
    size_t max_bytes = SIZE_MAX;  ///< 一次请求最多多少字节 / Bytes in one request
    /**
     * @brief 硬件是否直接读写槽内存 / Whether the hardware reads or writes the slot
     *        memory directly
     * @note true（默认）：硬件（DMA）直接读写槽，被停的槽留到停止结束。false：数据只由
     *       CPU 在后端的事件处理里在槽与硬件缓冲之间拷贝，被停的槽在停止时就放回，后端
     *       在 StopHardware 返回后不再读写这个槽。
     *       true (default): the hardware (DMA) reads or writes the slot directly and the
     *       stopped slot is kept until the stop ends. false: only the CPU copies between
     *       the slot and the hardware buffer, in the backend's event handling; the
     *       stopped slot is given back at the stop, and the backend no longer reads or
     *       writes it once StopHardware has returned.
     */
    bool direct_memory = true;
    /**
     * @brief 硬件上同时最多挂几个请求，至少为 1 / Requests the hardware holds at once,
     *        at least 1
     * @note 大于 1 时，核心在更早的请求还在硬件上时就把后面已提交的请求交下来（见
     *       StartHardware）。每个请求占一个槽，所以硬件上的请求数也不会超过槽数；控制
     *       端点总是 1。
     *       When above 1 the core hands later submitted requests down while earlier ones
     *       are still on the hardware (see StartHardware). Every request takes a slot,
     *       so the requests on the hardware never exceed the slot count either; a
     *       control endpoint always has 1.
     */
    uint8_t max_requests = 1U;
    /**
     * @brief 一个服务区间里硬件最多收发几个包，至少为 1 / Packets the hardware moves in
     *        one service interval, at least 1
     * @note 只对等时和中断类型有意义（USB 2.0 高速的高带宽端点）。等时类型要大于 1，
     *       max_packets 和 max_bytes 也得容得下整个区间。
     *       Meaningful for the isochronous and interrupt types only (high-bandwidth
     *       endpoints of USB 2.0 high speed). For the isochronous type to exceed 1,
     *       max_packets and max_bytes must hold the whole interval as well.
     */
    uint8_t max_packets_per_interval = 1U;
  };

  /**
   * @brief 后端是否接受这个配置 / Whether the backend accepts this configuration
   * @param cfg 其中 max_packet_size 和 max_packets_per_interval 是选定的包长和包数。
   *        max_packet_size and max_packets_per_interval are the chosen size and count.
   * @return OK；其他：Configure 以这个错误返回，端点保持 DISABLED。
   *         OK; otherwise Configure returns this error and the endpoint stays DISABLED.
   * @note 仅 owner，由 Configure 当场调用，上一次停止未结束时也是。HardwareLimits 表达
   *       不了的限制在这里检查，可以读别的端点当前的状态和配置（例如同号两个方向共用
   *       一个硬件类型字段时两边要一致）。不改变任何状态，不写寄存器。默认接受。
   *       Owner only, called in place by Configure, also while the last stop has not
   *       ended. Restrictions HardwareLimits cannot express are checked here, and the
   *       current state and configuration of other endpoints may be read (such as both
   *       directions of one number having to agree where they share one hardware type
   *       field). It changes no state and writes no register. Accepts by default.
   */
  virtual ErrorCode CheckHardware(const Config& cfg) const
  {
    UNUSED(cfg);
    return ErrorCode::OK;
  }

  /**
   * @brief 应用已校验的配置 / Apply a validated configuration
   * @param cfg 其中 max_packet_size 和 max_packets_per_interval 是选定的包长和包数。
   *        max_packet_size and max_packets_per_interval are the chosen size and count.
   * @note 仅 owner。可能在 Configure 里当场调用，也可能在停止结束的报告里、紧接
   *       CloseHardware 调用。不会失败：能否配置已由 HardwareLimits 和 CheckHardware
   *       判定，这里只把 cfg 写进硬件。两个方向共用的硬件设置（例如端点类型）照 cfg 写。
   *       Owner only. Called in place by Configure, or from the report that ends a stop
   *       right after CloseHardware. It cannot fail: whether the configuration is
   *       possible has been decided by HardwareLimits and CheckHardware, and this only
   *       writes cfg to the hardware. A hardware setting both directions share (such as
   *       the endpoint type) is written from cfg.
   */
  virtual void ConfigureHardware(const Config& cfg) = 0;

  /**
   * @brief 关闭端点 / Close the endpoint
   * @note 只在没有在途请求、也没有未结束的停止时调用。返回后硬件对该端点回 NAK，GetConfig
   *       仍是被关闭的配置。控制端点的 CloseHardware 和随后的 ConfigureHardware 可能晚于
   *       设备核心的 OnControlReady 到来，二者都不得撤销它接收 SETUP 的准备。
   *       Called only with nothing in flight and no stop pending. Afterwards the
   *       hardware NAKs on this endpoint; GetConfig is still the closed configuration.
   *       A control endpoint's CloseHardware and the ConfigureHardware after it may
   *       come later than the device core's OnControlReady, and neither may undo its
   *       readiness to receive a SETUP.
   */
  virtual void CloseHardware(bool in_isr) = 0;

  /**
   * @brief 启动一个请求 / Start one request
   * @param request 请求；IN 且 length 为 0 时发送零长度包。
   *        The request; IN with length 0 sends a zero-length packet.
   * @param in_isr 是否在中断中 / Whether in an ISR
   * @return OK，或失败（该请求以 FAILED 完成）/ OK, or failure (completes with FAILED)
   * @note 调用条件：仅 owner；只在硬件上的请求数小于 HardwareLimits 报的 max_requests
   *       时调用。硬件按调用的先后执行这些请求，完成也按这个先后报告。结尾零长度包在
   *       数据部分完成后作为第二次调用到来：同一个 data，length 为 0；还欠结尾零长度包的
   *       请求结束之前，核心不交下一个请求。本函数不关中断、不挂起调度、不等总线。
   *       Calling conditions: owner only; called only while fewer requests are on the
   *       hardware than the max_requests HardwareLimits gives. The hardware executes
   *       these requests in the order of the calls and their completions are reported
   *       in that order. A closing zero-length packet comes as a second call after the
   *       data part completed: same data, length 0; until a request that still owes its
   *       closing zero-length packet has ended the core hands no further request down.
   *       This function disables no interrupt, suspends no scheduling and never waits
   *       for the bus.
   * @note 等时端点的一个请求是一个服务区间的数据。IN：这个区间发 ⌈length / 包长⌉ 个
   *       包（length 为 0 时一个零长度包），数据 PID 按实际包数排（USB 2.0 5.9.2）。
   *       OUT：主机在这个区间发来的包都收下后完成，不论几个；收到的 PID 序列不对时以
   *       FAILED 完成。
   *       One request of an isochronous endpoint is the data of one service interval.
   *       IN: ⌈length / packet size⌉ packets go out in that interval (one zero-length
   *       packet when length is 0), with the data PIDs of that actual count (USB 2.0
   *       5.9.2). OUT: it completes once the packets the host sends in that interval
   *       have been taken, however few; a wrong sequence of received PIDs completes it
   *       with FAILED.
   * @note 可以在本函数内报告完成（同步完成），见 OnHardwareComplete；报告过时必须返回
   *       OK。硬件上还有更早的请求时不得在本函数内报告。这时返回失败的话，这个请求以
   *       FAILED 结束，它的回调排在更早那个请求的回调之后。
   *       The completion may be reported from inside this function (a synchronous
   *       completion), see OnHardwareComplete; after such a report it must return OK.
   *       With an earlier request still on the hardware nothing may be reported from
   *       inside this function. If it returns a failure then, the request ends with
   *       FAILED and its callback comes after the callback of the earlier request.
   * @note 控制端点上的规则，与硬件无关：SETUP 取代正在进行的阶段，所以有尚未向设备核心
   *       报告的 SETUP 时，硬件上不能留着这次装入的请求。
   *       The rule on a control endpoint, independent of the hardware: a SETUP supersedes
   *       the stage in progress, so while a SETUP not yet reported to the device core is
   *       latched, the request loaded here must not stay on the hardware.
   * @note 怎样满足这条规则，按硬件的行为分：
   *       1. SETUP 标志清掉之前硬件自己不执行装入的包（回 NAK）：什么都不用做。
   *       2. 硬件会执行：装入之后立即复读“已收到 SETUP 包”的指示（包到达时就置位的那
   *          一个，不是 SETUP 阶段完成标志），有未报告的 SETUP 就撤回这次装入。装入到
   *          复读之间插进来的执行是剩下的窗口。
   *          - 撤回当场完成（上限由芯片决定的握手）：返回失败，请求以 FAILED 结束，设备
   *            核心忽略它。
   *          - 撤回要等线上的包发完（旧应答的一包已在发）：不等，返回 OK；这次装入当作
   *            未结束的停止，后端在之后的硬件事件处理里结束它、以 ABORTED 报告，并按
   *            StopHardware 的交接规则 5 先于那个 SETUP 报告。已在线上的那一包是接受
   *            的残余。
   *       3. SETUP 包与控制端点的数据共用一块硬件缓冲：装入之前先查一次，已锁存的
   *          SETUP 先拷出来再装入；装入过程中才到的 SETUP 已被覆盖，后端清掉它、不
   *          报告，主机超时后重发。
   *       How the rule is met, by the behaviour of the hardware:
   *       1. The hardware itself does not execute a loaded packet (it answers NAK) until
   *          the SETUP flag is cleared: nothing has to be done.
   *       2. The hardware would execute it: right after the load, re-read the "SETUP
   *          packet received" indication (the one set when the packet arrives, not the
   *          SETUP phase done flag) and retract the load if an unreported SETUP is
   *          latched. Whatever runs between the load and the re-read is the remaining
   *          window.
   *          - The retract completes at once (a handshake bounded by the chip): return a
   *            failure; the request ends with FAILED and the device core ignores it.
   *          - The retract has to wait for a packet on the wire (a packet of the old
   *            reply is already being sent): do not wait, return OK; the load counts as
   *            a pending stop, which the backend ends in its later hardware event
   *            handling and reports as ABORTED, before that SETUP as handover rule 5 of
   *            StopHardware requires. The packet already on the wire is the accepted
   *            residual.
   *       3. The SETUP packet shares one hardware buffer with the control endpoint's
   *          data: check before the load and copy a latched SETUP out before loading; a
   *          SETUP arriving during the load has been overwritten, and the backend clears
   *          it without reporting it, the host retrying after its timeout.
   */
  virtual ErrorCode StartHardware(const HwRequest& request, bool in_isr) = 0;

  /**
   * @brief 停止的结果 / Outcome of a stop
   * @note result 与 OnHardwareComplete 的 result 含义相同，另加 PENDING：硬件还没有
   *       停稳，结果之后经 OnHardwareComplete 报告。PENDING 时 actual 不用。
   *       result means what it means for OnHardwareComplete, plus PENDING: the hardware
   *       is not quiescent yet and the outcome is reported later through
   *       OnHardwareComplete. actual is unused with PENDING.
   */
  struct StopOutcome
  {
    ErrorCode result = ErrorCode::ABORTED;  ///< 请求结果 / Request result
    size_t actual = 0;                      ///< 已传输字节数 / Bytes moved
  };

  /**
   * @brief 发出停止，不等待 / Issue the stop of the in-flight request without waiting
   * @param in_isr 是否在中断中 / Whether in an ISR
   * @return 硬件此刻已停稳时为被停请求的结果：OK，硬件已做完整个请求；ABORTED，没有
   *         做完；FAILED 或 OUT_OF_RANGE，硬件错误。否则 PENDING。核心断言只有这五种。
   *         The outcome of the stopped request if the hardware is quiescent now: OK, the
   *         hardware finished the whole request; ABORTED, it did not; FAILED or
   *         OUT_OF_RANGE, a hardware error. PENDING otherwise. The core asserts that it
   *         is one of these five.
   * @note 调用条件：仅 owner，只在硬件上有请求且没有停止未结束时调用（控制器复位之后的
   *       那一次例外，见交接规则 3），也会在中断里调用。一次调用停掉硬件上的全部请求，
   *       返回其中最早那个的结果；硬件上还有别的请求时核心接着再调用，每次得到下一个的
   *       结果，这些调用里硬件已经停稳，不得返回 PENDING。“停稳”指硬件不再为这些请求
   *       收发数据，也不再读写它们的槽。
   *       Calling conditions: owner only, called only with a request on the hardware
   *       and no stop pending (except the one call after a controller reset, see
   *       handover rule 3), interrupts included. One call stops every request on the
   *       hardware and returns the outcome of the earliest; while further requests are
   *       on the hardware the core calls again and gets the outcome of the next each
   *       time, the hardware being quiescent in those calls, which must not return
   *       PENDING. Quiescent means the hardware no longer moves data for these requests
   *       and no longer reads or writes their slots.
   * @note 与核心的交接，与硬件无关：
   *       1. 不等总线。本函数不等线上的包；上限由芯片而不是主机决定的寄存器握手可以
   *          做完，上限要有实测或手册出处。停稳还要等总线时返回 PENDING。本函数内不
   *          调用 OnHardwareComplete。
   *       2. 返回 PENDING 之后，后端不再为硬件上的任何请求装入新的包，端点保持使能
   *          （主机仍得到握手）。确认停稳后，在硬件事件处理中按先后为硬件上的每个请求
   *          各调用一次 OnHardwareComplete：真实结果（期间完成的包计入 actual，不再
   *          单独报告），或 TIMEOUT（见停稳的证据，只报一次）。别的端点的事件和 SETUP
   *          照常报告。报告之后才出现的、属于这些请求的完成标志，在下一次
   *          StartHardware 之前清掉。
   *       3. 报告 TIMEOUT 之后不再报告这个请求。控制器复位结束所有尚未报告的停止；
   *          复位之后再调用本函数，立即返回 ABORTED。
   *       4. 总线复位和断开时，后端在向设备核心报告该事件之前结束每个未结束的停止并
   *          报告，不等之后的硬件事件。挂起期间停止照常结束。
   *       5. 控制端点：SETUP 取代正在进行的阶段。向设备核心报告 SETUP 之前，这个控制
   *          端点上未结束的停止先结束并报告；从报告 SETUP 到下一次报告硬件事件之间，对
   *          它的停止当场给出结果，不返回 PENDING。SETUP 之前发出的停止和别的停止一样
   *          结束，主机空闲不得使它超时。
   *       6. HwLimits::direct_memory 为 false 的端点：本函数返回后后端不再读写这次
   *          请求的槽，之后才到的 OUT 包丢弃。
   *       Handover with the core, independent of the hardware:
   *       1. Never wait for the bus. This function does not wait for a packet on the
   *          wire; it may complete a register handshake whose bound is set by the chip,
   *          not by the host, the bound backed by a measurement or a manual reference.
   *          It returns PENDING when quiescence still depends on the bus. It never calls
   *          OnHardwareComplete.
   *       2. After PENDING the backend loads no new packet for any request on the
   *          hardware and the endpoint stays enabled (the host still gets a handshake).
   *          Once quiescence is confirmed it calls OnHardwareComplete from its hardware
   *          event handling once for each request on the hardware, in order: the real
   *          result (packets completed meanwhile count in actual and are not reported
   *          on their own), or TIMEOUT (see the evidence of quiescence; reported once).
   *          Events of other endpoints and a SETUP are reported as usual. A completion
   *          flag of these requests that shows up after the report is cleared before
   *          the next StartHardware.
   *       3. After reporting TIMEOUT nothing more is reported for the request. A
   *          controller reset ends every stop not reported yet; called again after the
   *          reset, this function returns ABORTED at once.
   *       4. On a bus reset or a disconnect the backend ends and reports every pending
   *          stop before it reports that event to the device core, without waiting for
   *          a later hardware event. While suspended, stops end as usual.
   *       5. Control endpoints: a SETUP supersedes the stage in progress. Before a SETUP
   *          is reported to the device core, the pending stops on that control endpoint
   *          end and are reported; from the report of the SETUP to the next report of a
   *          hardware event a stop of it gives its outcome at once and never returns
   *          PENDING. A stop issued before a SETUP ends like any other stop, and an idle
   *          host must not make it time out.
   *       6. On an endpoint whose HwLimits::direct_memory is false the backend no longer
   *          reads or writes the request's slot once this function has returned, and an
   *          OUT packet arriving later is dropped.
   * @note 停稳的证据。证据必须晚于停止请求。
   *       - 协议给的上界：全速和低速下事务不跨帧，停止之后的第一个帧起始就是证据；高速
   *         下停止之后经过的时间超过一个包在线上的最长时间就是证据。前提是停止写下之后
   *         硬件不再为这个端点开始新的事务；写下之后还会继续应答的控制器（例如停止要等
   *         硬件自己确认的队列型控制器）不能用它，只能用硬件自己的证据，时间只用来判
   *         TIMEOUT。
   *       - 停止总要在有界时间内结束。TIMEOUT 只用于硬件过了上界仍不确认的情况，报告
   *         之前在同一次处理里再读一次状态，已停稳就报真实结果。
   *       - 硬件能更早给出的证据可以用：端点自己的停稳事件；被停请求自己的完成；端点
   *         已自行禁用。
   *       - 所有端点共用的状态（例如“总线空闲”）：停止之后读到它成立，是充分证据；
   *         不能只等它，总线一直忙时它可以长时间不成立，必须同时有协议上界。
   *       - 没有中断能叫醒 owner 时，后端用自己的节拍经 DeviceService::RequestPoll
   *         叫醒；节拍上下文不碰 USB 寄存器。用来叫醒的中断不得破坏还没读出的事件。
   *       Evidence of quiescence. Evidence must be later than the stop request.
   *       - The bound the protocol gives: at full and low speed a transaction never
   *         crosses a frame, so the first start of frame after the stop is evidence; at
   *         high speed a time since the stop longer than the longest packet on the wire
   *         is evidence. The premise is that the hardware starts no new transaction for
   *         the endpoint once the stop is written; a controller that goes on answering
   *         after it (a queue-based controller whose stop awaits the hardware's own
   *         confirmation, for instance) cannot use the bound, only its own evidence, and
   *         time serves it for TIMEOUT alone.
   *       - A stop always has to end within a bounded time. TIMEOUT is for hardware
   *         that does not confirm even past the bound; before it is reported the status
   *         is read once more in the same handling and the real result reported if
   *         quiescent by then.
   *       - Earlier evidence from the hardware may be used: a quiescence event of the
   *         endpoint; the stopped request's own completion; the endpoint having
   *         disabled itself.
   *       - A status shared by all endpoints (such as "bus idle"): read as holding after
   *         the stop it is sufficient evidence; it must not be the only thing waited
   *         for, since it can stay absent for long while the bus is busy, and the
   *         protocol bound has to stand beside it.
   *       - Where no interrupt wakes the owner, the backend wakes it from its own tick
   *         through DeviceService::RequestPoll; the tick context touches no USB
   *         register. An interrupt used for the wake must not destroy an event not yet
   *         read.
   * @note 结果的证据。期间完成的包必须不多不少地计入；硬件有下列缺陷时，后端按对应的
   *       规则保证这一点，并在自己的头文件里写明它属于哪几类、怎么做、实测依据。
   *       - 完成事件会被之后的事务改写或合并：不能只靠事件；用硬件按端点保存的状态
   *         作证据，或在清事件之前确认没有别的完成并进来。
   *       - 软件写寄存器会抹掉硬件同时做的更新：这一位要么只由软件维护，要么后端能发现
   *         更新丢了并补回。
   *       - 停稳的事件会丢：过了协议上界仍没有等到，重新请求一次，之后才报 TIMEOUT。
   *       - 停止的手段影响别的端点：只允许主机会重试的那种影响（回 NAK）；会丢的数据
   *         （例如别的等时端点的包）要在后端的说明里写明。
   *       Evidence of the outcome. The packets completed meanwhile must be counted, none
   *       missing and none extra; where the hardware has one of the flaws below the
   *       backend keeps to the rule beside it and states in its own header which flaws
   *       it has, what it does and the measurement behind it.
   *       - A completion event can be rewritten by, or merge with, a later transaction:
   *         the event alone is not enough; state the hardware keeps per endpoint serves
   *         as evidence, or the backend makes sure before clearing the event that no
   *         other completion has merged into it.
   *       - A software write to a register undoes an update the hardware makes at the
   *         same time: the bit is either maintained by software alone, or the backend
   *         can detect the lost update and redo it.
   *       - The quiescence event can be lost: if it has not come once the protocol bound
   *         has passed, the request is made once more, and only then TIMEOUT.
   *       - The means of stopping affects other endpoints: only an effect the host
   *         retries (a NAK) is allowed; data that is lost (such as packets of other
   *         isochronous endpoints) is stated in the backend's documentation.
   */
  virtual StopOutcome StopHardware(bool in_isr) = 0;

  /// 在硬件上置 STALL / Halt the endpoint in hardware
  virtual ErrorCode StallHardware(bool in_isr) = 0;
  /**
   * @brief 在硬件上清除 STALL，数据翻转位回到 DATA0 / Clear the halt in hardware and
   *        return the data toggle to DATA0
   * @note 仅 owner，没有停止未结束时调用。主机可以对没有 STALL 的端点发
   *       CLEAR_FEATURE(ENDPOINT_HALT)，这时状态是 IDLE，硬件上可能有请求：翻转位照样回到
   *       DATA0（USB 2.0 9.4.5），硬件上的请求原样留着继续等主机，不得撤掉，也不报告。
   *       Owner only, called with no stop pending. A host may send
   *       CLEAR_FEATURE(ENDPOINT_HALT) to an endpoint that is not halted; the state is
   *       IDLE then and requests may be on the hardware: the toggle returns to DATA0 all
   *       the same (USB 2.0 9.4.5), and the requests on the hardware stay as they are,
   *       waiting for the host; they are neither withdrawn nor reported.
   */
  virtual ErrorCode ClearStallHardware(bool in_isr) = 0;
  /**
   * @brief 后端硬件的上限 / Limits of the backend hardware
   * @param direction 要配置的方向（IN 或 OUT）/ Direction to configure (IN or OUT)
   * @param type 要配置的传输类型 / Transfer type to configure
   * @return 最大包长，以及一次请求的包数和字节数上限。
   *         The largest packet, and the packet and byte limits of one request.
   * @note 只依赖参数和构造时确定的量，不读当前配置；端点未配置时也可调用。Configure
   *       据此选包长并确定 MaxTransferSize()。
   *       Depends only on the arguments and on what is fixed at construction, never on
   *       the current configuration; callable on an unconfigured endpoint. Configure
   *       chooses the packet size and sets MaxTransferSize() from it.
   */
  virtual HwLimits HardwareLimits(Direction direction, Type type) const
  {
    UNUSED(direction);
    UNUSED(type);
    return {};
  }

  /**
   * @brief 报告硬件上最早那个请求的完成，一次调用结束一个请求 / Report the hardware
   *        completion of the earliest request on the hardware; one call ends one request
   * @param in_isr 是否在中断中 / Whether in an ISR
   * @param actual 已传输字节数 / Bytes moved
   * @param result 普通完成：OK；FAILED 或 OUT_OF_RANGE：硬件错误，等时 IN 的 FAILED
   *        也用于服务区间已过、主机没有取走（见类说明）；ABORTED：控制端点撤回的装入
   *        （见 StartHardware）。停止结束的报告另可为 TIMEOUT（停止失败），ABORTED 表示
   *        没有做完。核心断言只有这些。
   *        A plain completion: OK; FAILED or OUT_OF_RANGE: a hardware error, FAILED of
   *        an isochronous IN also standing for a service interval that passed without
   *        the host taking the packet (see the class note); ABORTED: a control
   *        endpoint's retracted load (see StartHardware). The report that ends a stop
   *        may also be TIMEOUT (the stop failed), ABORTED meaning not finished. The core
   *        asserts one of these.
   * @note 仅 owner：在后端的硬件事件处理中（控制器中断里，或由持有 owner 的线程执行），
   *       或在本端点的 StartHardware 之内；StopHardware 和其他后端函数内不调用。每次
   *       StartHardware 启动的请求恰好结束一次：经本函数报告，或作为 StopHardware 的
   *       返回值（PENDING 除外）；没有停止未结束、也没有在途请求时报告被忽略。
   *       Owner only: from the backend's hardware event handling (in the controller
   *       interrupt, or run by the thread that owns), or from inside this endpoint's
   *       StartHardware; never from StopHardware or another backend function. Each
   *       request started by StartHardware ends exactly once: reported through this
   *       function or returned by StopHardware (PENDING excepted); a report with no
   *       stop pending and nothing in flight is ignored. A report after TIMEOUT breaks
   *       this contract: the core asserts and drops it.
   * @note 停止未结束时（以 stop_ 判断），报告就是停止结束，即使被停的槽已在停止时放回：
   *       请求已在停止时回调过，这里不再回调；仍留着的被停槽变空闲，推迟的硬件动作按
   *       CloseHardware、ConfigureHardware、StallHardware 或 ClearStallHardware 的顺序
   *       执行（见 ApplyDeferred）。TIMEOUT 时什么也不执行，经设备服务发布
   *       STOP_FAILED_EVENT。
   *       With a stop pending (judged by stop_) the report ends the stop, even if the
   *       stopped slot was already given back at the stop: the request was called back
   *       when it was stopped and gets no callback here; a stopped slot still held
   *       becomes free and the deferred hardware steps run in the order CloseHardware,
   *       ConfigureHardware, StallHardware or ClearStallHardware (see ApplyDeferred).
   *       With TIMEOUT nothing runs and STOP_FAILED_EVENT is published through the
   *       device service.
   * @note 在硬件事件处理中调用时：交付完成（停止结束则不交付），然后只置本端点的待处理
   *       位。下一个排队请求和结尾零长度包都不在这里启动，而在本轮或之后一轮的端点阶段
   *       启动，总在本次 ServiceHardware 返回之后，那时本次状态读取里的 SETUP 都已报告、
   *       已由设备核心处理；之后才锁存的 SETUP 由 StartHardware 的控制端点规则挡住。在
   *       StartHardware 之内调用时只留存并立即返回，不调用后端函数和回调；StartHardware
   *       返回后核心处理它，处理完才从引起这次调用的核心函数返回。
   *       Called from the hardware event handling: the completion is delivered (none
   *       when a stop ends), then only this endpoint's pending bit is set. Neither the
   *       next queued request nor a closing zero-length packet starts here; both start
   *       in the endpoint step of this or a later owner pass, always after this
   *       ServiceHardware returns, when the SETUPs of this status read have been
   *       reported and handled by the device core; a SETUP latched later is held off
   *       by the control endpoint rule of StartHardware. Called from inside
   *       StartHardware, it is
   *       only held and returns at once, calling no backend function and no callback;
   *       the core handles it after StartHardware returns and before the core function
   *       that made the call returns.
   * @note 顺序约定：后端向设备核心报告 SETUP 之前，必须先为这个控制端点上硬件已标记完成
   *       的请求调用本函数，这些请求因此在 SETUP 之前交付。
   *       Ordering contract: before a backend reports a SETUP to the device core, it
   *       calls this for every request on that control endpoint the hardware has
   *       flagged complete, so those are delivered before the SETUP.
   */
  void OnHardwareComplete(bool in_isr, size_t actual, ErrorCode result = ErrorCode::OK);

  /**
   * @brief 硬件当前的配置 / Configuration the hardware has now
   * @note 最近一次 ConfigureHardware 的参数。停止未结束时可能与 GetDirection、GetType
   *       给出的新绑定不同；后端只读这一份。
   *       The argument of the latest ConfigureHardware. While a stop is pending it may
   *       differ from the new binding that GetDirection and GetType give; the backend
   *       reads this copy only.
   */
  const Config& GetConfig() const { return hw_config_; }
  /// 构造时提供的整块槽内存 / The whole slot memory given at construction
  RawData SlotMemory() const { return memory_; }

  /// @}

 private:
  friend class DeviceService;
  friend class EndpointPool;

  /// 没有在途请求 / No request in flight
  static constexpr uint8_t NO_REQUEST = EndpointSlots::NO_SLOT;
  /// packet_ 中“一个服务区间的包数”所在的位置 / Position of the packets per service
  /// interval inside packet_
  static constexpr uint32_t INTERVAL_PACKETS_SHIFT = 24U;

  /// 在途请求的结尾零长度包 / Closing zero-length packet of the in-flight request
  enum class Closing : uint8_t
  {
    NONE,  ///< 不需要，或已不再需要 / Not needed, or no longer needed
    OWED,  ///< 数据部分在途，之后要发 / Data part in flight; the packet is owed
    /// 数据部分已完成，等端点阶段启动 / Data part done; awaiting the endpoint step
    DUE,
    ON_WIRE  ///< 结尾零长度包在途 / The closing packet is in flight
  };

  /// 结束请求的是哪个函数，决定停止之后的硬件操作 / Which function ends the requests;
  /// picks the hardware step after the stop
  enum class Ending : uint8_t
  {
    FLUSH,  ///< Flush：没有硬件操作 / Flush: no hardware step
    CLOSE,  ///< Close：停用记账，CloseHardware / Close: disable, CloseHardware
    STALL   ///< Stall：StallHardware
  };

  /// 在途请求的停止 / Stop of the in-flight request
  enum class Stop : uint8_t
  {
    NONE,     ///< 没有停止未结束 / No stop pending
    PENDING,  ///< 已发出，等后端报告 / Issued, awaiting the backend's report
    FAILED    ///< 报告为 TIMEOUT，等控制器复位 / Reported TIMEOUT; awaiting the reset
  };

  /// 推迟到停止结束的 CloseHardware（及新绑定的 ConfigureHardware）/ CloseHardware
  /// deferred to the end of the stop (and ConfigureHardware of a new binding)
  static constexpr uint8_t DEFER_CLOSE = 1U << 0U;
  /// 推迟到停止结束的 ClearStallHardware / ClearStallHardware deferred to the end of
  /// the stop
  static constexpr uint8_t DEFER_CLEAR_STALL = 1U << 1U;

  /**
   * @brief 启动排队的请求 / Start queued requests
   * @note 只由设备服务在端点阶段、取走本端点的待处理位后调用。可以启动时（IDLE、设备
   *       服务允许）：closing_ 为 DUE 时变为 ON_WIRE，以长度 0 发结尾零长度包；没有在途
   *       请求时取下一个排队请求，据其 zlp 与长度设定 closing_ 后启动；都经
   *       StartInflight，循环到不能再启动。stop_ 不为 NONE 时什么也不启动（被停的槽可能
   *       已放回，所以不能只看 inflight_）。
   *       Called only by the device service in its endpoint step, after it took this
   *       endpoint's pending bit. When starting is allowed (IDLE, the device service
   *       permits it): with closing_ DUE, closing_ becomes ON_WIRE and the closing
   *       packet starts with length 0; with nothing in flight, the next queued request
   *       starts after closing_ is set from its zlp and length; both go through
   *       StartInflight, looping until nothing more can start. Nothing starts while
   *       stop_ is not NONE (the stopped slot may have been given back already, so
   *       inflight_ alone does not tell).
   */
  void StartNext(bool in_isr);

  /**
   * @brief 把在途的槽交给硬件 / Hand the in-flight slot to the hardware
   * @param length 本次启动的长度；结尾零长度包为 0 / Length of this start; 0 for the
   *        closing zero-length packet
   * @note 在 in_backend_ 置位期间调用 StartHardware。启动失败时请求以 FAILED 完成；
   *       期间留存的报告在返回前交给 HandleReport。
   *       Calls StartHardware with in_backend_ set. A failed start completes the request
   *       with FAILED; a report held meanwhile goes to HandleReport before this returns.
   */
  void StartInflight(bool in_isr, size_t length);

  /**
   * @brief 把 ahead_ 里最后记下的请求提前交给硬件 / Hand the request recorded last in
   *        ahead_ to the hardware ahead of its turn
   * @note 只由 StartNext 调用，这时硬件上已有在途请求。启动失败时只记下，FAILED 的回调
   *       等它前面的请求都结束之后由 FinishInflight 交付。
   *       Called by StartNext only, with a request already in flight on the hardware. A
   *       failed start is only recorded; FinishInflight delivers the FAILED callback
   *       once the requests before it have ended.
   */
  void StartAhead(bool in_isr);

  /**
   * @brief 处理在途请求的一次报告 / Handle one report of the in-flight request
   * @note closing_ 为 OWED 且数据部分以 OK 完成时，closing_ 变为 DUE，请求留在途中，
   *       零长度包由 StartNext 发；否则 FinishInflight，ON_WIRE 时以 data_length_ 作
   *       actual。不调用 StartHardware；最后置本端点的待处理位。
   *       When closing_ is OWED and the data part completed with OK, closing_ becomes
   *       DUE and the request stays in flight, StartNext sending the closing packet;
   *       otherwise FinishInflight, with data_length_ as the actual while ON_WIRE.
   *       Calls no StartHardware; sets this endpoint's pending bit at the end.
   */
  void HandleReport(bool in_isr, ErrorCode result, size_t actual);

  /**
   * @brief 结束在途请求并交付完成 / End the in-flight request and deliver it
   * @note OUT 且 OK 时槽借给接收者，否则变空闲。
   *       For OUT with OK the slot is lent to the receiver, otherwise it becomes free.
   */
  void FinishInflight(bool in_isr, ErrorCode result, size_t actual);

  /**
   * @brief Close、Flush、Stall 的共同部分 / Shared part of Close, Flush and Stall
   * @param ending 调用者；CLOSE 时停用记账并使借出的槽作废（已停用时再做一次无影响）。
   *        The caller; CLOSE disables and marks lent slots stale (doing it again when
   *        already disabled changes nothing).
   * @note 依次：(1) 排队请求移出队列；CLOSE 清掉 DEFER_CLEAR_STALL；STALL 置
   *       STALLED。(2) 有请求在途、stop_ 为 NONE 且 closing_ 不是 DUE 时调用
   *       StopHardware；DUE 时硬件上没有东西，按 ABORTED、actual 为 data_length_ 结束。
   *       (3) 结果为 PENDING 时 stop_ 变 PENDING，CLOSE 记下 DEFER_CLOSE，direct_memory
   *       为 false 的端点当场放回被停的槽、inflight_ 置空；已有停止未结束（stop_ 不为
   *       NONE）时同样只记下；否则 CLOSE 调 CloseHardware，STALL 调 StallHardware，在途
   *       请求的槽按结果结束。(4) 在途请求交付一次：按结果，或 PENDING 时为 ABORTED；
   *       OUT 的 OK 和还欠结尾零长度包的 OK 改为 ABORTED，结尾零长度包在途时 actual 取
   *       data_length_。(5) 排队请求各交付一次 ABORTED。回调都在端点自己的状态改完之后。
   *       In order: (1) queued requests leave the queue; CLOSE clears
   *       DEFER_CLEAR_STALL; STALL sets STALLED. (2) With a request in flight, stop_ NONE
   *       and closing_ not DUE, StopHardware; with DUE nothing is on the hardware and
   *       the request ends with ABORTED and data_length_ as actual. (3) On PENDING stop_
   *       becomes PENDING, CLOSE records DEFER_CLOSE, and an endpoint whose
   *       direct_memory is false gives the stopped slot back at once and clears
   *       inflight_; with a stop already pending (stop_ not NONE) it only records too;
   *       otherwise CLOSE calls CloseHardware, STALL calls
   *       StallHardware and the in-flight slot ends from the outcome. (4) The in-flight
   *       request is delivered once: from the outcome, or with ABORTED on PENDING; OK
   *       becomes ABORTED for OUT and for a request still owing its closing zero-length
   *       packet, and actual is data_length_ while that packet is on the wire. (5) One
   *       ABORTED per queued request. All callbacks come after the endpoint's own state
   *       is settled.
   */
  void EndRequests(bool in_isr, Ending ending);

  /**
   * @brief 停止结束后执行推迟的硬件动作 / Run the deferred hardware steps after a stop
   * @note 调用前 stop_ 已回到 NONE、仍留着的被停槽已空闲（只在 inflight_ 不为空时放，
   *       已在停止时放回的不再放）。先把 deferred_ 取到局部并清零，
   *       再依次：DEFER_CLOSE 时 CloseHardware，状态不是 DISABLED 时再以 config_ 调
   *       ConfigureHardware；状态为 STALLED 时 StallHardware；状态为 IDLE 且有
   *       DEFER_CLEAR_STALL 时 ClearStallHardware；最后置本端点的待处理位。
   *       ConfigureHardware 失败（后端违约，断言）时：状态改为 DISABLED、记账停用、不再
   *       做后面的动作，之后才为排队请求各交付一次 ABORTED；回调里看到的是 stop_ NONE、
   *       deferred_ 0、DISABLED，可以再 Configure。
   *       On entry stop_ is NONE again and a stopped slot still held is free (freed
   *       only if inflight_ is set; one given back at the stop is not freed twice).
   *       deferred_ is
   *       copied to a local and cleared first, then in order: with DEFER_CLOSE,
   *       CloseHardware, then ConfigureHardware with config_ unless the state is
   *       DISABLED; with the state STALLED, StallHardware; with the state IDLE and
   *       DEFER_CLEAR_STALL, ClearStallHardware; finally this endpoint's pending bit is
   *       set. If ConfigureHardware fails (a backend fault, asserted): the state becomes
   *       DISABLED, the bookkeeping is disabled, no later step runs, and only then one
   *       ABORTED per queued request is delivered; those callbacks see stop_ NONE,
   *       deferred_ 0 and DISABLED, and may Configure again.
   */
  void ApplyDeferred(bool in_isr);

  /**
   * @brief 控制器复位后结束停止并关闭 / End the stop and close after a controller reset
   * @note 只由 DeviceService::CloseAllEndpoints 调用。stop_ 不为 NONE 时再调用一次
   *       StopHardware（复位后立即返回，即使被停的槽已放回），仍留着的被停槽变空闲，
   *       不回调，stop_ 回到 NONE，deferred_ 清零（推迟的动作都不再做）；然后同 Close，
   *       CloseHardware 当场调用。
   *       Called by DeviceService::CloseAllEndpoints only. With stop_ not NONE, calls
   *       StopHardware once more (it returns at once after the reset, even if the
   *       stopped slot was already given back), frees a stopped slot still held without
   *       a callback, sets stop_ back to NONE and clears deferred_ (no deferred step
   *       runs any more); then as Close, with CloseHardware in place.
   */
  void CloseAfterReset(bool in_isr);

  /// 调用完成回调；没有借出的槽时 slot 为 Slot{} / Run the completion callback; slot
  /// is Slot{} when no slot is lent
  void Deliver(bool in_isr, ErrorCode result, size_t actual, const Slot& slot);

  /**
   * @brief 某个配置在某个速度下会得到的包长 / Packet size a configuration gets at a speed
   * @param cfg 方向、类型和包长上限 / Direction, type and packet-size upper bound
   * @param speed 总线速度 / Bus speed
   * @return 包长；0 表示本端点在该速度下不能这样配置，包括硬件直接读写槽、只有一个槽
   *         而类型不是控制。
   *         The packet size; 0 when this endpoint cannot be configured so at that
   *         speed, including the hardware accessing the slot directly with one slot and
   *         a type other than control.
   * @note 只读，任何上下文，不要求端点已配置，也不改变端点。Configure 用同一个计算；
   *       端点池在绑定之前用它回答设备类写描述符时的包长查询。
   *       Read-only, any context; the endpoint need not be configured and is not changed.
   *       Configure uses the same computation; the pool answers the packet-size query of
   *       a class writing its descriptors with it, before any bind.
   */
  uint16_t PacketSizeFor(const Config& cfg, Speed speed) const;
  /**
   * @brief 某个配置在某个速度下会得到的“一个服务区间最多几个包”/ Packets in one
   *        service interval a configuration gets at a speed
   * @return 包数；0 的条件同 PacketSizeFor，另加 cfg.max_packets_per_interval 为 0。
   *         The count; 0 under the conditions of PacketSizeFor, and when
   *         cfg.max_packets_per_interval is 0.
   * @note 调用条件和用途同 PacketSizeFor；Configure 用同一个计算。
   *       Called and used as PacketSizeFor; Configure uses the same computation.
   */
  uint8_t PacketsPerIntervalFor(const Config& cfg, Speed speed) const;
  /// 请求 owner 为本端点启动排队请求 / Ask the owner to start this endpoint's queue
  void NotifyOwner(bool in_isr);
  /// 槽 index 的内存，长度为 size / Memory of slot index, size bytes long
  RawData SlotData(uint8_t index, size_t size) const;
  /// 以 release 写状态 / Store the state with release ordering
  void SetState(State state) { state_.store(state, std::memory_order_release); }

  /// @name 构造与登记时固定 / Fixed at construction and registration
  /// @{
  EPNumber number_ = EPNumber::EP_INVALID;       ///< 端点号 / Endpoint number
  Direction avail_direction_ = Direction::BOTH;  ///< 可用方向 / Available direction
  RawData memory_;                               ///< 槽内存 / Slot memory
  size_t slot_size_ = 0;  ///< 每个槽的字节数，缓存行整数倍 / Bytes per slot
  /// 所属设备服务，Register 时写入 / Owning device service, set by Register
  DeviceService* pool_ = nullptr;
  /// 本端点在设备服务待处理掩码中的位 / This endpoint's bit in the service's pending mask
  uint32_t pending_bit_ = 0;
  /// @}

  /// @name 配置：仅 owner 写 / Configuration: written by the owner only
  /// @{
  /// 当前绑定的配置，包长为选定值 / Configuration of the current binding, chosen size
  Config config_;
  /// 硬件当前的配置：ConfigureHardware 成功后写，GetConfig 读 / Configuration the
  /// hardware has: written after a successful ConfigureHardware, read by GetConfig
  Config hw_config_;
  /// 槽容量；任何上下文读 / Slot capacity; read in any context
  std::atomic<uint32_t> capacity_{0};
  /**
   * @brief 包长在低 16 位，位 16 标记控制类型，位 31..24 是一个服务区间的包数 / Packet
   *        size in the low 16 bits, bit 16 marking the control type, bits 31..24 the
   *        packets per service interval
   * @note MaxPacketSize 和 Receive 在任何上下文读；Receive 据此判断 0 字节是否合法，
   *       不读 owner 写的 config_。
   *       Read in any context by MaxPacketSize and Receive; Receive decides from it
   *       whether 0 bytes are legal, without reading config_, which the owner writes.
   */
  std::atomic<uint32_t> packet_{0};
  /// @}

  /// 状态：owner 写，任何上下文读 / State: written by the owner, read anywhere
  std::atomic<State> state_{State::DISABLED};
  /// 槽的无锁记账：任何上下文 / Lock-free slot bookkeeping: any context
  EndpointSlots slot_states_;

  /// @name 请求：仅 owner / Requests: owner only
  /// @{
  /**
   * @brief 在途的槽 / Slot in flight
   * @note 只由 StartNext 写成非空，其他地方只会置空。stop_ 不为 NONE 时它是留着的被停
   *       槽（硬件直接读写槽的端点），或已经为空（被停槽在停止时放回）；状态可以是任何
   *       一种。
   *       Set to a slot only by StartNext; elsewhere it is only cleared. With stop_ not
   *       NONE it is the stopped slot still held (an endpoint whose hardware reads or
   *       writes the slot directly), or already empty (the stopped slot was given back
   *       at the stop); the state may be any.
   */
  uint8_t inflight_ = NO_REQUEST;
  /// 一个提前交给硬件的请求 / One request handed to the hardware ahead of its turn
  struct Ahead
  {
    uint8_t index = EndpointSlots::NO_SLOT;  ///< 槽；没有时为 NO_SLOT / Slot, or none
    size_t length = 0;                       ///< 请求长度 / Request length
    bool owed = false;    ///< 之后还欠结尾零长度包 / A closing packet is owed after it
    bool failed = false;  ///< StartHardware 返回了失败 / StartHardware returned a failure
  };
  /**
   * @brief 排在在途请求之后、已提前交给硬件的请求的槽，按先后 / Slots of the requests
   *        behind the one in flight that have been handed to the hardware ahead, in order
   * @note StartNext 在末尾写入；在途请求结束时 FinishInflight 把最前面的一个顶上去成为
   *       inflight_；EndRequests、停止结束的报告和 CloseAfterReset 清空。只在 inflight_
   *       不为空时非空。欠着结尾零长度包的请求和没能进硬件的请求后面不再提前交，所以
   *       这两种只会是最后一个。
   *       StartNext appends; FinishInflight moves the first one up to inflight_ when the
   *       request in flight ends; cleared by EndRequests, by the report that ends a stop
   *       and by CloseAfterReset. Non-empty only while inflight_ is. Nothing is handed
   *       ahead behind a request that owes its closing zero-length packet or one that
   *       did not reach the hardware, so only the last entry can be either.
   */
  uint8_t ahead_[MAX_SLOTS - 1U] = {};
  uint8_t ahead_count_ = 0U;  ///< ahead_ 里的个数 / Entries in ahead_
  /// ahead_ 的最后一个没能进硬件（StartHardware 失败）/ The last entry of ahead_ did not
  /// reach the hardware (StartHardware failed)
  bool ahead_failed_ = false;
  /// ahead_ 的第 position 个请求 / The request at position of ahead_
  Ahead AheadAt(uint8_t position) const;
  /// 取走 ahead_ 的第一个，返回它的槽；没有时返回 NO_REQUEST / Remove the first entry
  /// of ahead_ and return its slot, or NO_REQUEST when there is none
  uint8_t PopAhead();
  /// 放回并清空 ahead_ 里的槽；keep_started 时只放回没能进硬件的那个 / Free and clear
  /// the slots in ahead_; with keep_started only the one that did not reach the hardware
  void FreeAhead(bool keep_started);
  /**
   * @brief inflight_ 是顶上来的、当初没能进硬件的请求，等 FinishInflight 以 FAILED 结束 /
   *        inflight_ is a request that moved up and never reached the hardware;
   *        FinishInflight ends it with FAILED
   */
  bool inflight_failed_ = false;
  /// 硬件上同时最多几个请求：后端报的上限，控制端点为 1；Configure 写 / Requests on the
  /// hardware at once: the backend's limit, 1 on a control endpoint; written by Configure
  uint8_t max_requests_ = 1U;
  /// 未结束的停止还欠几次报告；stop_ 为 PENDING 时非零 / Reports the pending stop still
  /// owes; nonzero while stop_ is PENDING
  uint8_t stop_reports_ = 0U;
  /**
   * @brief 在途请求的停止 / Stop of the in-flight request
   * @note “停止是否未结束”的判断一律看本成员，不看 inflight_：Configure、Close、Flush、
   *       Stall、ClearStall、EndRequests、StartNext、OnHardwareComplete、
   *       CloseAfterReset。
   *       Every "is a stop pending" test reads this member, never inflight_:
   *       Configure, Close, Flush, Stall, ClearStall, EndRequests, StartNext,
   *       OnHardwareComplete, CloseAfterReset.
   */
  Stop stop_ = Stop::NONE;
  /**
   * @brief 推迟到停止结束的硬件动作，DEFER_* 位 / Hardware steps deferred to the end of
   *        the stop, DEFER_* bits
   * @note 不变式：不为 0 只在 stop_ 不为 NONE 时。只在停止结束的报告（ApplyDeferred）和
   *       CloseAfterReset 里清零，这两处也是 stop_ 回到 NONE 的仅有地方；TIMEOUT 时保留，
   *       由 CloseAfterReset 清掉。
   *       Invariant: nonzero only while stop_ is not NONE. Cleared only by the report
   *       that ends a stop (ApplyDeferred) and by CloseAfterReset, the only places where
   *       stop_ returns to NONE; kept on TIMEOUT and cleared by CloseAfterReset.
   */
  uint8_t deferred_ = 0U;
  /**
   * @brief 在途请求是否还欠结尾零长度包 / Whether the in-flight request still owes its
   *        closing zero-length packet
   * @note StartNext 启动请求时写入 OWED 或 NONE、发零长度包时写入 ON_WIRE，HandleReport
   *       写入 DUE，都在 owner 中；只在有请求在途时读。
   *       Written by StartNext when it starts a request (OWED or NONE) or sends the
   *       closing packet (ON_WIRE) and by HandleReport (DUE), all in the owner; read
   *       only with a request in flight.
   */
  Closing closing_ = Closing::NONE;
  /// 数据部分的字节数，ON_WIRE 时作完成的 actual / Bytes of the data part; the
  /// completion's actual while ON_WIRE
  size_t data_length_ = 0;
  /// 正在调用本端点的 StartHardware / Inside this endpoint's StartHardware
  bool in_backend_ = false;
  /// in_backend_ 期间收到的报告尚未处理 / A report received while in_backend_ is set
  /// awaits handling
  bool held_ = false;
  size_t held_actual_ = 0;  ///< 留存报告的字节数 / Bytes of the held report
  ErrorCode held_result_ = ErrorCode::OK;  ///< 留存报告的结果 / Result of the held report
  /// @}

  Callback<const Completion&> on_complete_;  ///< 完成回调 / Completion callback
};

}  // namespace LibXR::USB
