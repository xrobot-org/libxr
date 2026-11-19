#pragma once

#include <cstddef>
#include <cstdint>

#include "ch32_usb.hpp"
#include "driver_async_condition.hpp"
#include "libxr_def.hpp"
#include "libxr_type.hpp"
#include "usb/core/ep.hpp"

namespace LibXR
{

class CH32USBOtgFS;
class CH32USBOtgHS;

#if defined(USBFSD) || defined(__DOXYGEN__)

/**
 * @class CH32EndpointOtgFs
 * @brief CH32 OTG FS 端点/ CH32 OTG FS endpoint
 *
 * 一个对象管一个端点号的一个方向。端点号只有一个方向时（单向）UEPn_DMA 随请求指向槽，
 * 硬件直接读写槽，零拷贝，两个槽。两个方向都有时（共享）硬件把 OUT 放在一块 128 字节
 * 缓冲的基址、IN 放在基址 +64，UEPn_DMA 始终指向基址；槽是另外的内存，数据由 CPU 在
 * 这块缓冲和槽之间逐包拷贝（最多 64 字节），槽不被硬件访问，每个方向一个槽。EP0 两个
 * 方向共用一块，硬件直接读写。
 *
 * 批量和中断端点的 DATA0/DATA1 由软件管，不开硬件的自动翻转：TX_CTRL、RX_CTRL 因此只有
 * owner 在写，停止时读改写 NAK 不会和硬件抢同一个寄存器。每个发送完成事件翻一次 IN，每个
 * TOG_OK 的接收事件翻一次 OUT，没有请求在途时也翻（厂商例程的做法，手册 p.359 两种都
 * 允许）。开自动翻转时，停止的那次读改写会把硬件刚翻过的位写回去（CH32V208 实测 5000 次
 * 停止丢 3 个包）。
 * One object serves one direction of one endpoint number. With one direction per number
 * (single) UEPn_DMA follows the request to the slot, the hardware reads and writes the
 * slot, nothing is copied, and there are two slots. With both directions (shared) the
 * hardware puts OUT at the base of a 128-byte buffer and IN at base +64, and UEPn_DMA
 * always points at the base; the slots are separate memory, the CPU copies each packet
 * (64 bytes at most) between that buffer and the slot, the hardware never accesses the
 * slot, and each direction has one slot. EP0's two directions share one block that the
 * hardware accesses directly.
 *
 * DATA0/DATA1 of bulk and interrupt endpoints is kept by software and the hardware's
 * automatic toggle stays off: TX_CTRL and RX_CTRL are then written by the owner alone,
 * and the read-modify-write of NAK at a stop cannot race the hardware for the register.
 * Every transmit completion event flips IN once and every receive event with TOG_OK
 * flips OUT once, also with no request in flight (the vendor examples do the same; the
 * manual, p.359, allows either). With the automatic toggle on, the read-modify-write of
 * a stop writes back the bit the hardware has just flipped (measured on a CH32V208: 3
 * packets lost in 5,000 stops).
 *
 * @note 寄存器只在设备的 owner 中访问（后端函数和 CH32USBOtgFS::ServiceHardware）。
 *       Registers are touched only in the device's owner (the backend functions and
 *       CH32USBOtgFS::ServiceHardware).
 *
 * @note 停止，对应 Endpoint::StopHardware 的规则。
 *       停稳的证据：被停请求自己的完成事件；所有端点共用的 SIE 空闲状态（只作充分
 *       证据）；协议上界是写 NAK 之后的第一个帧起始，停止时已经锁存的那个不算。挂起和
 *       总线复位也结束它。本后端不报 TIMEOUT。
 *       缺陷一，完成事件会被合并：INT_BUSY 不约束等时事务，它的完成会并进已经置位的
 *       UIF_TRANSFER。每轮清标志之前和之后各查一次 INT_ST 并补交付，见
 *       CH32USBOtgFS::ServiceHardware。实测等时 IN、OUT 同时跑 3 x 180 s 漏 1 个包。
 *       缺陷二，软件写寄存器会抹掉硬件的更新：停止对 TX_CTRL、RX_CTRL 的读-改-写会
 *       写回硬件刚自动翻转的位，所以翻转位只由软件维护（见上）。
 *       另外两类没有：没有会丢的停稳事件；停止只写本端点的寄存器。
 *       实测（CH32V208）：两个接口各约 6.5 万次停止，丢包和重复都是 0，未结束的停止
 *       最长 855 us。
 *       Stops, against the rules of Endpoint::StopHardware.
 *       Evidence of quiescence: the stopped request's own completion event; the SIE-free
 *       status shared by all endpoints (as sufficient evidence only); the protocol
 *       bound is the first start of frame after NAK was written, not one already
 *       latched at the stop. Suspend and a bus reset end it too. This backend reports
 *       no TIMEOUT.
 *       Flaw one, completion events merge: INT_BUSY does not hold off isochronous
 *       transactions, and their completion merges into a UIF_TRANSFER already set.
 *       Every round looks at INT_ST once before and once after clearing the flag and
 *       delivers what merged, see CH32USBOtgFS::ServiceHardware. Measured with
 *       isochronous IN and OUT together: 1 packet missed in 3 x 180 s.
 *       Flaw two, a software write undoes an update of the hardware: the
 *       read-modify-write of TX_CTRL and RX_CTRL at a stop writes back the bit the
 *       hardware has just toggled, so the toggle is kept by software alone (see above).
 *       The other two kinds are absent: there is no quiescence event to lose, and a
 *       stop writes this endpoint's registers only.
 *       Measured (CH32V208): about 65,000 stops on each of two interfaces, 0 lost and 0
 *       repeated, longest pending stop 855 us.
 */
class CH32EndpointOtgFs : public USB::Endpoint
{
 public:
  /**
   * @brief 构造端点，登记到端点表，硬件置 NAK / Construct, enter the endpoint table,
   *        NAK in hardware
   * @param ep_num 端点号 / Endpoint number
   * @param dir 方向（IN 或 OUT）/ Direction (IN or OUT)
   * @param buffer 槽内存。单向：硬件直接读写它；EP0：两个方向共用的块，硬件直接读写；
   *        共享：只由 CPU 读写。
   *        Slot memory. Single: the hardware reads and writes it; EP0: the block both
   *        directions share, accessed by the hardware; shared: touched by the CPU only.
   * @param shared_block 该端点号两个方向都用时，硬件的 128 字节块（OUT 在基址，IN 在
   *        基址 +64），两个方向的对象拿同一块；否则为 nullptr。
   *        When the number is used in both directions, the hardware's 128-byte block (OUT
   *        at its base, IN at base +64), the same for both objects; nullptr otherwise.
   * @note 调用者：CH32USBOtgFS 构造，控制器中断使能之前。
   *       Caller: the CH32USBOtgFS constructor, before the interrupt is enabled.
   */
  CH32EndpointOtgFs(EPNumber ep_num, Direction dir, RawData buffer,
                    uint8_t* shared_block);

  /// 停止未结束的端点数，诊断用；在 owner 之外读到的是瞬时值 / Number of endpoints whose
  /// stop is pending, for diagnostics; outside the owner the value is a momentary one
  static uint8_t PendingStops() { return pending_stops_; }

 protected:
  void ConfigureHardware(const Config& cfg) override;
  void CloseHardware(bool in_isr) override;
  ErrorCode StartHardware(const HwRequest& request, bool in_isr) override;
  /**
   * @brief 写 NAK；SIE 此刻空闲就收下本端点已锁存的完成并给结果，否则 PENDING
   *        Write NAK; with the SIE free now, collect this endpoint's latched completion
   *        and give the outcome, else PENDING
   * @note 不等线上的事务。未结束的停止由本端点的完成事件、之后读到的 SIE 空闲、写 NAK
   *       之后的第一个帧起始、总线复位或挂起结束（SOF 中断常开）。帧起始是上界：事务
   *       不跨帧，所以这个后端不报 TIMEOUT。处理 SETUP 期间对控制端点的
   *       停止当场完成：SETUP 标志未清时控制器对所有令牌回 NAK（CH32V208 实测）。
   *       The transaction on the wire is not waited for. A pending stop ends on this
   *       endpoint's completion event, on the SIE read free later, on the first start of
   *       frame after the NAK, on a bus reset or on suspend (the SOF interrupt is always
   *       on). The start of frame is the bound: transactions never cross a frame, so
   *       this backend reports no TIMEOUT. A stop of the control
   *       endpoint during SETUP handling is complete at once: with the SETUP flag
   *       pending the controller answers every token with NAK (measured on a CH32V208).
   */
  StopOutcome StopHardware(bool in_isr) override;
  ErrorCode StallHardware(bool in_isr) override;
  ErrorCode ClearStallHardware(bool in_isr) override;
  HwLimits HardwareLimits(Direction direction, Type type) const override;

 private:
  friend class CH32USBOtgFS;

  /// OTG FS 端点号个数 / Number of OTG FS endpoint numbers
  static constexpr uint8_t EP_OTG_FS_MAX_SIZE = 8;
  /// 共享块中每个方向的字节数 / Bytes per direction in a shared block
  static constexpr size_t SHARED_BLOCK_HALF = 64U;

  /// 停止时要计入的本端点完成事件 / The completion event of this endpoint to count at
  /// the stop
  struct StopEvent
  {
    /// 有一个本端点、本方向的完成事件 / There is a completion event of this endpoint
    /// and direction
    bool mine = false;
    size_t size = 0U;     ///< OUT：该事件的 RX_LEN / OUT: RX_LEN of that event
    bool tog_ok = false;  ///< OUT：该事件的 TOG_OK / OUT: TOG_OK of that event
  };

  /**
   * @brief 结算被停的请求：计入最后一个完成事件，给出结果和字节数 / Settle the stopped
   *        request: count the last completion event, give the result and byte count
   * @param event 停止时的完成事件；没有时为 {} / The completion event at the stop; {}
   *        when there is none
   * @note 计入一个 OUT 包时：EP0 翻软件 toggle，累加已收字节。数据不拷：被停的 OUT 请求
   *       以 ABORTED 交付、不借出槽。
   *       Counting an OUT packet flips EP0's software toggle and adds to the bytes
   *       received. No data is copied: a stopped OUT request is delivered ABORTED and
   *       lends no slot.
   */
  StopOutcome SettleStop(const StopEvent& event);
  /// 停止转为未结束：计数，记下此刻是否已锁存着 SOF / The stop becomes pending: count
  /// it and note whether a SOF is latched already
  void BeginPending();
  /// 停止不再未结束：计数减一 / The stop is no longer pending: uncount it
  void EndPending();
  /// 结束未结束的停止并报告 / End a pending stop and report it
  void FinishStop(bool in_isr, const StopEvent& event);
  /**
   * @brief 结束所有已停稳的未结束停止 / End every pending stop that is quiescent
   * @param force 调用者已知总线上没有事务（总线复位、挂起、控制端点的 SETUP）
   *        The caller knows no transaction is on the bus (bus reset, suspend, a SETUP on
   *        the control endpoint)
   * @param only_ep0 只处理控制端点 / Control endpoint only
   * @note 调用者：ServiceHardware。不带 force 时要求 SIE 空闲且没有待处理的传输事件。
   *       Caller: ServiceHardware. Without force it requires a free SIE and no transfer
   *       event waiting.
   */
  static void FinishQuietStops(bool in_isr, bool force, bool only_ep0);
  /**
   * @brief 帧起始结束未结束的停止 / A start of frame ends the pending stops
   * @note 调用者：ServiceHardware，本轮的传输事件处理之后。事务不跨帧：写 NAK 之后收到
   *       帧起始，写 NAK 时在线上的那个事务就已经结束，它的完成事件在本轮已经处理过。
   *       不看 SIE 是否空闲：别的端点的流量可以让它一直忙（CH32V208 实测，只按 SIE 空闲
   *       判断时空闲端点的停止 16 帧都结束不了）。本轮快照之后才发出的停止不算。
   *       Caller: ServiceHardware, after the transfer event of the round. Transactions
   *       never cross a frame: with a start of frame after the NAK, the transaction that
   *       was on the wire at the NAK is over and its completion event was handled in this
   *       round. The free SIE is not consulted: other endpoints' traffic can keep it busy
   *       (measured on a CH32V208: judged by the free SIE alone, the stop of an idle
   *       endpoint did not end within 16 frames). A stop issued after this round's
   *       snapshot does not count.
   */
  static void FinishStopsAtSof(bool in_isr);

  /**
   * @brief 一次本端点的硬件完成 / One hardware completion of this endpoint
   * @param size OUT：RX_LEN；IN：不用 / OUT: RX_LEN; IN: unused
   * @param tog_ok OUT：INT_ST.TOG_OK / OUT: INT_ST.TOG_OK
   * @note 调用者：ServiceHardware（owner）。先推进软件 toggle，再推进多包请求，或结束
   *       请求并报告。没有请求在途时只推进 toggle。
   *       Caller: ServiceHardware (owner). Advances the software toggle first, then
   *       advances a multi-packet request, or ends it and reports. With no request in
   *       flight only the toggle advances.
   */
  void TransferComplete(bool in_isr, size_t size, bool tog_ok);
  /// 主机 ACK 了一个包：翻本方向的软件 toggle（批量和中断）/ The host ACKed a packet:
  /// flip this direction's software toggle (bulk and interrupt)
  void AdvanceToggle();

  /// 写入本方向 RES 字段，保留其他位 / Write this direction's RES field, keep the rest
  void SetResponse(uint8_t res);

  /**
   * @brief 端点表 / Endpoint table
   * @note 写：端点构造；读：ServiceHardware、StopHardware。
   *       Written by the endpoint constructor; read by ServiceHardware, StopHardware.
   */
  static inline CH32EndpointOtgFs* map_[EP_OTG_FS_MAX_SIZE][2] = {};

  /**
   * @brief ServiceHardware 已读到 UIF_TRANSFER，要到本轮结束才清 / ServiceHardware has
   *        read UIF_TRANSFER and clears it at the end of its round
   * @note 写：ServiceHardware；读：StopHardware。置位时锁存的事件已由事件循环分发，
   *       INT_BUSY 又让控制器对其他事务回 NAK，所以停止不收任何锁存事件。
   *       Written by ServiceHardware; read by StopHardware. While set, the latched event
   *       belongs to the event loop and INT_BUSY makes the controller NAK every other
   *       transaction, so a stop collects no latched event.
   */
  static inline bool transfer_held_ = false;
  /// ServiceHardware 正在处理 SETUP 令牌，标志还没有清 / ServiceHardware is handling a
  /// SETUP token whose flag is not cleared yet
  static inline bool setup_held_ = false;
  /// 停止未结束的端点数；仅 owner / Endpoints whose stop is pending; owner only
  static inline uint8_t pending_stops_ = 0U;

  /// 该端点号只有一个方向；构造时固定 / Only this direction exists; fixed at
  /// construction
  const bool single_direction_;
  /// 构造时的块基址；共享时 UEPn_DMA 固定在这里 / Block base given at construction;
  /// UEPn_DMA stays here when shared
  uint8_t* const block_dma_base_;

  /// 停止已写下、硬件还没有停稳 / The stop was issued and the hardware is not
  /// quiescent yet
  bool stopping_ = false;
  /**
   * @brief 停止发出时已经锁存着一个 SOF / A SOF was already latched when the stop was
   *        issued
   * @note 它早于 NAK，处理到它时跳过一次。
   *       It precedes the NAK and is skipped once when it is handled.
   */
  bool sof_before_stop_ = false;

  /**
   * @brief EP0 软件 DATA0/1：true 表示下一包用 DATA1 / EP0 software DATA0/1: true
   *        means DATA1 next
   * @note 写：总线复位、SETUP（ServiceHardware）、EP0 的 StartHardware（IN）和完成
   *       （OUT）、ClearStallHardware；读：StartHardware。均在 owner。
   *       Written by bus reset and SETUP (ServiceHardware), EP0 StartHardware (IN) and
   *       completion (OUT), ClearStallHardware; read by StartHardware. All in the owner.
   */
  bool tog_ = false;

  /// @name 在途请求：仅 owner / Request in flight: owner only
  /// @{
  /// 有请求挂在硬件上且尚未结束 / A request is on the hardware and not ended
  bool transfer_active_ = false;
  size_t transfer_requested_ = 0;       ///< 请求字节数 / Bytes requested
  size_t transfer_accumulated_ = 0;     ///< OUT 已收字节 / OUT bytes received
  uint8_t* transfer_cursor_ = nullptr;  ///< 下一包在槽中的位置 / Next packet in the slot
  size_t transfer_remaining_ = 0;       ///< IN 尚未挂出的字节 / IN bytes not armed yet
  /// @}
};

#endif  // defined(USBFSD) || defined(__DOXYGEN__)

#if defined(USBHSD) || defined(__DOXYGEN__)

/**
 * @class CH32EndpointOtgHs
 * @brief CH32 OTG HS 端点/ CH32 OTG HS endpoint
 *
 * 一个对象管一个端点号的一个方向。EP1-15 两个方向的 DMA 寄存器分立；EP0 两个方向共用
 * UEP0_DMA 和一块内存。批量和中断 IN 用硬件自动 toggle，OUT 由软件在每个 TOG_OK 包后
 * 翻转。
 * One object serves one direction of one endpoint number. EP1-15 have separate DMA
 * registers per direction; EP0's two directions share UEP0_DMA and one block. Bulk and
 * interrupt IN use the hardware auto toggle; software flips the OUT toggle after every
 * TOG_OK packet.
 *
 * @note 寄存器只在设备的 owner 中访问（后端函数和 CH32USBOtgHS::ServiceHardware）。
 *       Registers are touched only in the device's owner (the backend functions and
 *       CH32USBOtgHS::ServiceHardware).
 *
 * @note 停止，对应 Endpoint::StopHardware 的规则。
 *       停稳的证据：被停请求自己的完成；所有端点共用的 SIE 空闲状态（只作充分证据）；
 *       协议上界是写 NAK 之后经过 STOP_AGE_US（长于任何一包在线上的时间），毫秒节拍挂起
 *       USB 中断，由中断再处理一轮时检查。总线复位和挂起也结束它。本后端不报 TIMEOUT。
 *       缺陷一，完成事件会被改写：INT_ST 只有一份，之后被忙 NAK 的令牌和紧跟着到来的
 *       SETUP 都会改写它（置 IS_NAK、清 TOG_OK）；SOF 中断打开时 SOF 也改写它，所以
 *       SOF 中断不开。IN 包是否发出按端点自己的 toggle 位判断（每轮扫描）；OUT 包在带
 *       IS_NAK 或 SETUP 已锁存时不看 TOG_OK。
 *       缺陷二，软件写寄存器会抹掉硬件的更新：写 TX_LEN、TX_CTRL、RX_CTRL 所在的那个
 *       32 位字时，IN 的自动翻转会丢。完成事件到了而 toggle 没动就补翻
 *       （RepairLostToggle，以及 StopHardware 写 NAK 之后的检查）。OUT 的翻转位由软件
 *       维护。
 *       另外两类没有：没有会丢的停稳事件；停止只写本端点的寄存器。
 *       实测（CH32V307）：约 25 万次停止，两个方向丢包和重复都是 0。
 *
 * @note 等时 IN。等时事务不受 INT_BUSY 约束，主机随时来取包；在它取包的那一刻改
 *       ENDP_CONFIG 里本端点的发送使能，SIE 会停在一次发不完的发送上（SIE_FREE 一直
 *       为 0），此后设备对任何令牌都不应答，总线复位也清不掉（CH32V307 实测：逐包开关
 *       发送使能时 6 次 60 s 的流里 3 次出现；发送使能常开时 11 次里 0 次）。所以：
 *       - 发送使能从配置开到关闭，其间不动。没有包可发时长度为 0，主机取到零长度包。
 *       - 本端点的 TX_LEN、TX_CTRL、TX_DMA 只在它刚发完一包（数据或零长度包）之后写：
 *         那时离主机下一次取包最远。发出一包按 toggle 位的自动变化认出（手册 p.347），
 *         每个有 UIF_TRANSFER 的轮次查一次。
 *       - 请求启动时，这一轮里本端点刚发完一包就当场装入，否则等它下一次发出再装入，
 *         所以空闲之后的第一包晚一个服务区间。连续提交的流不受影响：上一包的完成和
 *         下一包的启动在同一轮里。
 *       - 停止：包还没装上硬件时当场结束；装上了且 SIE 空闲时撤回，这是唯一不在“刚发完
 *         一包”时刻的写入，只改长度和 TX_CTRL。
 *       Isochronous IN. Isochronous transactions are not held off by INT_BUSY and the
 *       host takes a packet whenever it likes; changing this endpoint's transmit enable
 *       in ENDP_CONFIG at that moment leaves the SIE in a transmission that never ends
 *       (SIE_FREE stays 0), after which the device answers no token at all, and a bus
 *       reset does not clear it (measured on a CH32V307: with the enable switched per
 *       packet, 3 of 6 streams of 60 s; with the enable left on, 0 of 11). Hence:
 *       - Transmit stays enabled from configuration to close. With no packet to send
 *         the length is 0 and the host gets a zero-length packet.
 *       - TX_LEN, TX_CTRL and TX_DMA of the endpoint are written only right after it
 *         has sent a packet (data or zero-length): that is farthest from the host's
 *         next take. A sent packet is recognised by the automatic change of the toggle
 *         bits (manual p.347), checked in every round with UIF_TRANSFER.
 *       - When a request starts, it is loaded at once if the endpoint sent a packet in
 *         this pass, otherwise at its next transmission, so the first packet after an
 *         idle time is one service interval late. A stream submitted continuously is
 *         not affected: the completion of one packet and the start of the next are in
 *         one pass.
 *       - Stop: a packet not on the hardware yet ends at once; one that is loaded is
 *         retracted while the SIE is free, the only write that is not at "a packet has
 *         just gone out", and it changes the length and TX_CTRL only.
 *       Stops, against the rules of Endpoint::StopHardware.
 *       Evidence of quiescence: the stopped request's own completion; the SIE-free
 *       status shared by all endpoints (as sufficient evidence only); the protocol
 *       bound is STOP_AGE_US since NAK was written (longer than any packet on the
 *       wire), checked in the pass the USB interrupt runs when the millisecond tick
 *       pends it. A bus reset and suspend end it too. This backend reports no TIMEOUT.
 *       Flaw one, completion events are rewritten: there is one INT_ST, and a token
 *       busy-NAKed afterwards as well as a SETUP arriving right after rewrite it
 *       (IS_NAK set, TOG_OK cleared); with the SOF interrupt enabled a SOF rewrites it
 *       too, so that interrupt stays off. Whether an IN packet went out is told by the
 *       endpoint's own toggle bits (scanned every round); for an OUT packet TOG_OK is
 *       not looked at when IS_NAK is set or a SETUP is latched.
 *       Flaw two, a software write undoes an update of the hardware: a write to the
 *       32-bit word that holds TX_LEN, TX_CTRL and RX_CTRL loses the automatic toggle
 *       of the IN. A completion event with an unmoved toggle is repaired
 *       (RepairLostToggle, and the check after the NAK write in StopHardware). The OUT
 *       toggle is kept by software.
 *       The other two kinds are absent: there is no quiescence event to lose, and a
 *       stop writes this endpoint's registers only.
 *       Measured (CH32V307): about 250,000 stops, 0 lost and 0 repeated in both
 *       directions.
 */
class CH32EndpointOtgHs : public USB::Endpoint
{
 public:
  /**
   * @brief 构造端点，登记到端点表，设 DMA 地址，硬件置 NAK / Construct, enter the
   *        endpoint table, set the DMA address, NAK in hardware
   * @param ep_num 端点号 / Endpoint number
   * @param dir 方向（IN 或 OUT）/ Direction (IN or OUT)
   * @param buffer 槽内存；EP0 两个方向传同一块 / Slot memory; both EP0 directions pass
   *        the same block
   * @note 调用者：CH32USBOtgHS 构造，控制器中断使能之前。
   *       Caller: the CH32USBOtgHS constructor, before the interrupt is enabled.
   */
  CH32EndpointOtgHs(EPNumber ep_num, Direction dir, RawData buffer);

 protected:
  void ConfigureHardware(const Config& cfg) override;
  void CloseHardware(bool in_isr) override;
  ErrorCode StartHardware(const HwRequest& request, bool in_isr) override;
  /**
   * @brief 写 NAK；SIE 此刻空闲就收下本端点已锁存的完成并给结果，否则 PENDING
   *        Write NAK; with the SIE free now, collect this endpoint's latched completion
   *        and give the outcome, else PENDING
   * @note 不等线上的事务。未结束的停止由本端点的完成事件、之后读到的 SIE 空闲、总线
   *       复位或挂起结束；都没有时由毫秒节拍挂起 USB 中断、在那一轮里按时间结束
   *       （这个控制器的 SOF 中断会改写未读的 OUT 事件，不能代替节拍）。节拍由板级
   *       工程每毫秒调用 DriverAsyncCondition::OnTick 提供。处理 SETUP 期间对控制端点
   *       的停止当场完成。
   *       The transaction on the wire is not waited for. A pending stop ends on this
   *       endpoint's completion event, on the SIE read free later, on a bus reset or on
   *       suspend; failing all of these the millisecond tick pends the USB interrupt and
   *       that pass ends it by time (this controller's SOF interrupt rewrites an unread
   *       OUT event and cannot stand in for the tick). The board project supplies the
   *       tick by calling DriverAsyncCondition::OnTick every millisecond. A stop of the
   *       control endpoint during SETUP handling is complete at once.
   */
  StopOutcome StopHardware(bool in_isr) override;
  ErrorCode StallHardware(bool in_isr) override;
  ErrorCode ClearStallHardware(bool in_isr) override;
  HwLimits HardwareLimits(Direction direction, Type type) const override;

 private:
  friend class CH32USBOtgHS;

  /// OTG HS 端点号个数 / Number of OTG HS endpoint numbers
  static constexpr uint8_t EP_OTG_HS_MAX_SIZE = 16;
  /// tog_window_ 的有效位 / Valid bit of tog_window_
  static constexpr uint8_t TOG_WINDOW = 0x80;

  /// 停止时要计入的本端点完成事件 / The completion event of this endpoint to count at
  /// the stop
  struct StopEvent
  {
    /// 有一个本端点、本方向的完成事件 / There is a completion event of this endpoint
    /// and direction
    bool mine = false;
    bool in_acked = false;  ///< IN：该事件不带 IS_NAK / IN: that event carries no IS_NAK
    bool out_tog_ok = false;  ///< OUT：该事件的包 toggle 正确 / OUT: packet in step
    size_t rx_len = 0U;       ///< OUT：该事件的 RX_LEN / OUT: RX_LEN of that event
  };

  /**
   * @brief 结算被停的请求：计入最后一个完成事件，给出结果和字节数 / Settle the stopped
   *        request: count the last completion event, give the result and byte count
   * @param event 停止时的完成事件；没有时为 {} / The completion event at the stop; {}
   *        when there is none
   * @param flip_out 批量和中断 OUT 的软件 toggle 还没有为这个包翻过 / The software
   *        toggle of a bulk or interrupt OUT has not been flipped for this packet yet
   * @note 没有事件时，批量和中断 IN 仍按自动 toggle 是否翻过判断这一包发出没有
   *       （CH32V307 实测：带 IS_NAK 上报的 IN 包里有已经发出的，toggle 才可信）。
   *       Without an event a bulk or interrupt IN still decides by the auto toggle
   *       whether the packet went out (measured on a CH32V307: some IN packets reported
   *       with IS_NAK had been delivered; the toggle is what can be trusted).
   */
  StopOutcome SettleStop(const StopEvent& event, bool flip_out);
  /// 停止转为未结束：计数、记下时刻、启动节拍等待 / The stop becomes pending: count it,
  /// note the time and start the tick wait
  void BeginPending();
  /// 停止不再未结束：计数减一 / The stop is no longer pending: uncount it
  void EndPending();
  /// 结束未结束的停止并报告 / End a pending stop and report it
  void FinishStop(bool in_isr, const StopOutcome& outcome);
  /**
   * @brief 结束所有已停稳的未结束停止 / End every pending stop that is quiescent
   * @param force 调用者已知总线上没有事务（总线复位、挂起、控制端点的 SETUP）
   *        The caller knows no transaction is on the bus (bus reset, suspend, a SETUP on
   *        the control endpoint)
   * @param only_ep0 只处理控制端点 / Control endpoint only
   * @note 不带 force 时要求 SIE 空闲且没有待处理的传输事件。SIE 空闲是充分条件：满负荷
   *       时它可能长时间不出现（CH32V307 实测 1200 次里 41 次超过 2 ms），那时由
   *       AgeStops 按时间结束。
   *       Without force it requires a free SIE and no transfer event waiting. A free SIE
   *       is sufficient only: under saturated traffic it may not show for a long time
   *       (more than 2 ms in 41 of 1200 on a CH32V307), and AgeStops then ends the stop
   *       by time.
   */
  static void FinishQuietStops(bool in_isr, bool force, bool only_ep0);
  /**
   * @brief 按时间结束未结束的停止 / End pending stops by time
   * @note 调用者：ServiceHardware，每次调用的末尾、还有停止未结束时。写下 NAK 已满
   *       STOP_AGE_US 的停止结束：这段时间长于任何一包在线上的时间，写 NAK 时在线上的
   *       事务已经结束。还有未结束的就再启动节拍等待，保证之后有一轮来查。
   *       Caller: ServiceHardware, at the end of every call while a stop is pending. A
   *       stop whose NAK was written STOP_AGE_US ago ends: that is longer than any packet
   *       stays on the wire, so the transaction on the wire at the NAK is over. With
   *       stops still pending the tick wait is started again, so that a later pass
   *       checks them.
   */
  static void AgeStops(bool in_isr);

  /**
   * @brief 一次本端点的硬件完成 / One hardware completion of this endpoint
   * @param size OUT：RX_LEN；IN：不用 / OUT: RX_LEN; IN: unused
   * @param tog_ok OUT：INT_ST.TOG_OK / OUT: INT_ST.TOG_OK
   * @note 调用者：ServiceHardware（owner）。批量和中断 OUT 先按 TOG_OK 翻软件 toggle；
   *       推进多包请求，或结束请求并报告。没有请求在途时不报告。
   *       Caller: ServiceHardware (owner). Bulk and interrupt OUT flip the software
   *       toggle on TOG_OK first; advances a multi-packet request, or ends it and
   *       reports. Reports nothing with no request in flight.
   */
  void TransferComplete(bool in_isr, size_t size, bool tog_ok);

  /// 挂着的批量或中断 IN 包是否已被主机 ACK（自动 toggle 已翻）/ Whether the armed
  /// bulk or interrupt IN packet was ACKed by the host (the auto toggle moved)
  bool InPacketAcked() const;

  /**
   * @brief 等时 IN 在上次写 TX_CTRL 之后发出过一包 / An isochronous IN packet has gone
   *        out since TX_CTRL was last written
   * @note 依据是 toggle 位已变。
   *       Told by the toggle bits having changed.
   */
  bool IsoInSent() const;
  /// 等时 IN：换成没有包的状态（长度 0，DATA0）/ Isochronous IN: load the no-packet
  /// state (length 0, DATA0)
  void IdleIsoIn();
  /// 等时 IN：把在途请求的包装上硬件 / Isochronous IN: load the packet of the request
  /// in flight
  void LoadIsoIn();
  /**
   * @brief 等时 IN：本端点刚发出一包 / Isochronous IN: the endpoint has just sent a
   *        packet
   * @note 装着的包发出了就完成请求（停止未结束时结束停止），然后把寄存器换成没有包的
   *       状态；有请求在等时把它装上。
   *       If the loaded packet went out the request completes (a pending stop ends),
   *       then the registers go to the no-packet state; a waiting request is loaded.
   */
  void OnIsoInSent(bool in_isr);
  /**
   * @brief 对每个刚发出一包的等时 IN 端点调用 OnIsoInSent / Call OnIsoInSent for every
   *        isochronous IN endpoint that has just sent a packet
   * @note 调用者：ServiceHardware，每轮有 UIF_TRANSFER 时一次。
   *       Caller: ServiceHardware, once per round with UIF_TRANSFER.
   */
  static void ServiceIsoIn(bool in_isr);

  /// 好包之后补上被同号 OUT 挂包吞掉的 IN toggle 推进 / After a good packet, redo an IN
  /// toggle advance lost while the OUT of the same number was armed
  void RepairLostToggle();

  /**
   * @brief 完成所有已被 ACK 而 INT_ST 没报的 IN 包 / Complete every IN packet that was
   *        ACKed although INT_ST did not report it
   * @note 调用者：ServiceHardware，每轮有 UIF_TRANSFER 时一次。
   *       Caller: ServiceHardware, once per round with UIF_TRANSFER.
   */
  static void CompleteAckedInPackets(bool in_isr);

  /**
   * @brief 清 HALT 的 toggle 复位：翻转位回到 DATA0，RES 只有 STALL 时改成 NAK / The
   *        toggle reset of a cleared halt: the toggle returns to DATA0, RES becomes NAK
   *        only when it is STALL
   * @note 调用者：ClearStallHardware（不在事件轮里时），FinishClearToggles。
   *       Caller: ClearStallHardware (outside an event round), FinishClearToggles.
   */
  void ApplyClearToggle();

  /**
   * @brief 执行本轮推迟的清 HALT toggle 复位 / Apply the toggle resets of the halts
   *        cleared in this round
   * @note 调用者：ServiceHardware，每轮清标志之前，在本轮的完成都已交付之后。高速控制器
   *       一轮可以锁存多个事务：SETUP 之前已完成的本端点事务要到 SETUP 之后才交付，它要按
   *       旧相位处理（OUT 的软件 toggle 翻一次，IN 按挂包时的 toggle 认出已发），所以复位
   *       推迟到这里。标志清掉之前 INT_BUSY 和 SETUP_ACT 让控制器对其他事务回 NAK。
   *       Caller: ServiceHardware, every round before the flags are cleared and after the
   *       round's completions were delivered. The high-speed controller can latch several
   *       transactions per round: one of this endpoint that completed before the SETUP
   *       is delivered after the SETUP and must be handled at the old phase (an OUT's
   *       software toggle flips once, an IN is recognised as sent by the toggle at
   *       arming), hence the reset waits until here. Until the flags are cleared INT_BUSY
   *       and SETUP_ACT make the controller NAK other transactions.
   */
  static void FinishClearToggles();

  /// 推迟到本轮末尾的清 HALT，位 2*n+方向 / Halts cleared and deferred to the end of
  /// the round, bit 2*n+direction
  static inline uint32_t clear_tog_pending_ = 0U;

  /// 写入本方向 RES 字段，保留其他位 / Write this direction's RES field, keep the rest
  void SetResponse(uint8_t res);

  /**
   * @brief 端点表 / Endpoint table
   * @note 写：端点构造；读：ServiceHardware、StopHardware、OUT 的 StartHardware（取同号
   *       IN）、CompleteAckedInPackets。
   *       Written by the endpoint constructor; read by ServiceHardware, StopHardware,
   *       an OUT's StartHardware (the IN of the same number), CompleteAckedInPackets.
   */
  static inline CH32EndpointOtgHs* map_[EP_OTG_HS_MAX_SIZE][2] = {};

  /**
   * @brief ServiceHardware 已读到 UIF_TRANSFER，要到本轮结束才清 / ServiceHardware has
   *        read UIF_TRANSFER and clears it at the end of its round
   * @note 写：ServiceHardware；读：StopHardware。置位时锁存的事件已由事件循环分发，
   *       INT_BUSY 又让控制器对其他事务回 NAK，所以停止不收任何锁存事件。
   *       Written by ServiceHardware; read by StopHardware. While set, the latched event
   *       belongs to the event loop and INT_BUSY makes the controller NAK every other
   *       transaction, so a stop collects no latched event.
   */
  static inline bool transfer_held_ = false;

  /**
   * @brief EP0 软件 DATA0/1：true 表示下一包用 DATA1 / EP0 software DATA0/1: true
   *        means DATA1 next
   * @note 写：总线复位、SETUP（ServiceHardware）、EP0 IN 的 StartHardware、status OUT
   *       的 StartHardware；读：EP0 IN 的 StartHardware。均在 owner。
   *       Written by bus reset and SETUP (ServiceHardware), EP0 IN StartHardware and
   *       status OUT StartHardware; read by EP0 IN StartHardware. All in the owner.
   */
  bool tog0_ = false;

  /// ServiceHardware 正在处理 SETUP，标志还没有清 / ServiceHardware is handling a SETUP
  /// whose flag is not cleared yet
  static inline bool setup_held_ = false;
  /// 停止未结束的端点数；仅 owner / Endpoints whose stop is pending; owner only
  static inline uint8_t pending_stops_ = 0U;
  /**
   * @brief 设备的节拍等待对象 / The device's tick wait
   * @note 写：设备构造；用：owner（BeginPending、AgeStops）。它只负责约 1 ms 后挂起 USB
   *       中断，让中断再处理一轮；停止是否结束由时刻决定。
   *       Written at the device's construction; used in the owner (BeginPending,
   *       AgeStops). It only pends the USB interrupt about 1 ms later, so that the
   *       interrupt runs another pass; whether a stop is over is decided by its time.
   */
  static inline DriverAsyncCondition* tick_ = nullptr;
  /// ServiceHardware 的轮次，从 1 起；ServiceHardware 开头加一 / Pass number of
  /// ServiceHardware, from 1; incremented at the start of ServiceHardware
  static inline uint32_t pass_ = 0U;

  /// 停止已写下、硬件还没有停稳 / The stop was issued and the hardware is not
  /// quiescent yet
  bool stopping_ = false;
  /// 写下 NAK 的时刻（µs），stopping_ 时有效 / Time the NAK was written (µs), valid
  /// while stopping_
  uint64_t stop_us_ = 0;

  /// @name 在途请求：仅 owner / Request in flight: owner only
  /// @{
  /// 有请求挂在硬件上且尚未结束 / A request is on the hardware and not ended
  bool transfer_active_ = false;
  size_t transfer_requested_ = 0;       ///< 请求字节数 / Bytes requested
  size_t transfer_accumulated_ = 0;     ///< OUT 已收字节 / OUT bytes received
  uint8_t* transfer_cursor_ = nullptr;  ///< 下一包在槽中的位置 / Next packet in the slot
  size_t transfer_remaining_ = 0;       ///< IN 尚未挂出的字节 / IN bytes not armed yet
  /// IN：挂包时 TX_CTRL 的 toggle 位 / IN: the TX_CTRL toggle bits when armed
  uint8_t tx_tog_armed_ = 0;
  /// 已配置为等时 IN；ConfigureHardware 写，CloseHardware 清 / Configured as an
  /// isochronous IN; written by ConfigureHardware, cleared by CloseHardware
  bool iso_in_ = false;
  /// 等时 IN：在途请求的包已装上硬件 / Isochronous IN: the packet of the request in
  /// flight is on the hardware
  bool iso_loaded_ = false;
  /**
   * @brief 等时 IN 上一次发出一包所在的那一轮 ServiceHardware / The ServiceHardware
   *        pass in which the isochronous IN last sent a packet
   * @note 0 表示没有。
   *       0 for none.
   */
  uint32_t iso_free_pass_ = 0U;
  /**
   * @brief 仅 IN：同号 OUT 挂包期间完成了事务时，挂包前的 IN toggle 位（| TOG_WINDOW）；
   *        否则 0 / IN only: the IN toggle bits from before an arm of the OUT of the same
   *        number during which a transaction completed (| TOG_WINDOW); 0 otherwise
   * @note 写：同号 OUT 的 StartHardware；读并清：RepairLostToggle（每个被 ACK 的包）；
   *       清：ClearStallHardware、ConfigureHardware（它们把 toggle 复位为 DATA0，留下的
   *       窗口会在下一个包后误翻）。
   *       Written by the OUT's StartHardware; read and cleared by RepairLostToggle (every
   *       ACKed packet); cleared by ClearStallHardware and ConfigureHardware (they reset
   *       the toggle to DATA0, and a window left behind would flip it wrongly after the
   *       next packet).
   */
  uint8_t tog_window_ = 0;
  /// @}
};

#endif  // defined(USBHSD) || defined(__DOXYGEN__)

}  // namespace LibXR
