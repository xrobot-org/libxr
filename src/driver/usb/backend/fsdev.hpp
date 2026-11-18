#pragma once

#include <cstddef>
#include <cstdint>
#include <initializer_list>

#include "usb/core/ep.hpp"
#include "usb/core/ep_pool.hpp"
#include "usb/core/usb_service.hpp"
#include "usb/device/dev_core.hpp"

/**
 * @file fsdev.hpp
 * @brief PMA 型全速设备控制器（FSDEV）的后端，由多个平台共用 / Backend of the PMA-based
 *        full-speed device controller (FSDEV), shared by several platforms
 *
 * 硬件每个端点每个方向只有一个 PMA 包缓冲，CPU 在槽与 PMA 之间逐包拷贝；硬件从不读写
 * 槽内存。寄存器只在设备服务的 owner 中访问（ServiceHardware 和端点的后端函数），例外
 * 只有 Dispatch 里、owner 之外的几步：EP0 的快速确认，状态 OUT 的晚放行，以及等状态包
 * 期间把其他端点的完成先记下（见 FSDevDevice::EnableEp0FastAck、EnableEp0LateStatus）。
 * 本后端不进临界区，也不关全部中断。等时端点不支持。
 * The hardware has one PMA packet buffer per endpoint and direction, and the CPU copies
 * packet by packet between the slot and the PMA; the hardware never touches slot
 * memory. Registers are accessed only in the device service's owner (ServiceHardware
 * and the endpoints' backend functions). The only exceptions are steps of Dispatch
 * outside the owner: the fast acknowledgement of EP0, the late release of the status
 * OUT, and latching the other endpoints' completions while the status packet is awaited
 * (see FSDevDevice::EnableEp0FastAck and EnableEp0LateStatus). This backend takes no
 * critical section and never disables all interrupts. Isochronous endpoints are not
 * supported.
 *
 * 平台的设备类派生 FSDevDevice：构造时给出寄存器块和 PMA 的地址，打开时钟后调用
 * InitController；实现 Connect、MaskInterrupt、UnmaskInterrupt，可选
 * PendInterrupt；在控制器 中断里调用 Dispatch。寄存器布局不同的变体（32 位寄存器的
 * DRD）覆盖对应的虚函数。 The platform's device class derives from FSDevDevice: it gives
 * the addresses of the register block and the PMA at construction and calls
 * InitController after enabling the clocks; it implements Connect, MaskInterrupt,
 * UnmaskInterrupt and optionally PendInterrupt, and calls Dispatch from the controller
 * interrupt. A variant with another register layout (DRD with 32-bit registers) overrides
 * the corresponding virtuals.
 */

namespace LibXR::USB
{

/// 寄存器相对寄存器块基址的偏移；EPnR 在 4n / Register offsets from the base of the
/// register block; EPnR is at 4n
constexpr uint32_t FSDEV_REG_CNTR = 0x40U;
constexpr uint32_t FSDEV_REG_ISTR = 0x44U;    ///< 中断状态 / Interrupt status
constexpr uint32_t FSDEV_REG_FNR = 0x48U;     ///< 帧号 / Frame number
constexpr uint32_t FSDEV_REG_DADDR = 0x4CU;   ///< 设备地址 / Device address
constexpr uint32_t FSDEV_REG_BTABLE = 0x50U;  ///< BTABLE 偏移 / BTABLE offset

/**
 * @brief EPnR 的位 / Bits of EPnR
 * @note CTR 写 0 清除、写 1 保持；STAT 与 DTOG 写 1 翻转；SETUP 只读。
 *       CTR is cleared by writing 0 and kept by writing 1; STAT and DTOG toggle on a
 *       written 1; SETUP is read-only.
 */
constexpr uint32_t FSDEV_EP_CTR_RX = 0x8000U;
constexpr uint32_t FSDEV_EP_DTOG_RX = 0x4000U;  ///< 接收数据翻转位 / Receive data toggle
constexpr uint32_t FSDEV_EP_STAT_RX = 0x3000U;  ///< 接收状态 / Receive status
constexpr uint32_t FSDEV_EP_SETUP = 0x0800U;  ///< 收到的是 SETUP / The packet was a SETUP
constexpr uint32_t FSDEV_EP_TYPE = 0x0600U;   ///< 端点类型 / Endpoint type
constexpr uint32_t FSDEV_EP_KIND = 0x0100U;   ///< 类型附加位 / Endpoint kind
constexpr uint32_t FSDEV_EP_CTR_TX = 0x0080U;   ///< 发送完成 / Transmission complete
constexpr uint32_t FSDEV_EP_DTOG_TX = 0x0040U;  ///< 发送数据翻转位 / Transmit data toggle
constexpr uint32_t FSDEV_EP_STAT_TX = 0x0030U;  ///< 发送状态 / Transmit status
constexpr uint32_t FSDEV_EP_ADDR = 0x000FU;     ///< 端点地址 / Endpoint address
/**
 * @brief 没有端点使用的地址 / An address no endpoint uses
 * @note 改写 STAT 的片刻里端点停在这里，见 FSDevEndpoint::SetStat。
 *       An endpoint is parked here for the moment its STAT is rewritten, see
 *       FSDevEndpoint::SetStat.
 */
constexpr uint32_t FSDEV_EP_PARK_ADDR = 0x000FU;
/// 经典型写 EPnR 时写 1 保持的位 / Bits a write to EPnR keeps by writing 1 (classic)
constexpr uint32_t FSDEV_EP_KEEP_MASK = FSDEV_EP_CTR_RX | FSDEV_EP_SETUP | FSDEV_EP_TYPE |
                                        FSDEV_EP_KIND | FSDEV_EP_CTR_TX | FSDEV_EP_ADDR;

/// ISTR 的标志 / Flags of ISTR
constexpr uint16_t FSDEV_ISTR_CTR = 0x8000U;
constexpr uint16_t FSDEV_ISTR_WKUP = 0x1000U;   ///< 唤醒 / Wake-up
constexpr uint16_t FSDEV_ISTR_SUSP = 0x0800U;   ///< 挂起 / Suspend
constexpr uint16_t FSDEV_ISTR_RESET = 0x0400U;  ///< 总线复位 / Bus reset
constexpr uint16_t FSDEV_ISTR_SOF = 0x0200U;    ///< 帧起始 / Start of frame
/// PMAOVR | ERR | ESOF：只清除，不处理 / Only cleared, never handled
constexpr uint16_t FSDEV_ISTR_NOISE = 0x6100U;
/// ISTR 的 EP_ID 字段 / The EP_ID field of ISTR
constexpr uint16_t FSDEV_ISTR_EP_ID = 0x000FU;

/// CNTR 的位 / Bits of CNTR
constexpr uint16_t FSDEV_CNTR_SOFM = 0x0200U;
/// CTRM | RESETM | SUSPM | WKUPM：运行时打开的中断 / The interrupts enabled while running
constexpr uint16_t FSDEV_CNTR_RUN_MASK = 0x8000U | 0x0400U | 0x0800U | 0x1000U;
constexpr uint16_t FSDEV_CNTR_FSUSP = 0x0008U;   ///< 强制挂起 / Force suspend
constexpr uint16_t FSDEV_CNTR_LPMODE = 0x0004U;  ///< 低功耗模式 / Low-power mode

/// FNR 的 RXDP：D+ 线的电平 / RXDP of FNR: level of the D+ line
constexpr uint16_t FSDEV_FNR_RXDP = 0x8000U;
constexpr uint16_t FSDEV_CNTR_FRES = 0x0001U;  ///< 强制复位 / Force reset

/// STAT 的目标值（低 16 位，两种变体相同）/ STAT targets (low half, both variants)
constexpr uint32_t FSDEV_STAT_TX_DIS = 0x0000U;
constexpr uint32_t FSDEV_STAT_TX_STALL = 0x0010U;  ///< 回复 STALL / Reply STALL
constexpr uint32_t FSDEV_STAT_TX_NAK = 0x0020U;    ///< 回复 NAK / Reply NAK
constexpr uint32_t FSDEV_STAT_TX_VALID = 0x0030U;  ///< 发送数据 / Send data
constexpr uint32_t FSDEV_STAT_RX_DIS = 0x0000U;    ///< 不应答 / Ignore tokens
constexpr uint32_t FSDEV_STAT_RX_STALL = 0x1000U;  ///< 回复 STALL / Reply STALL
constexpr uint32_t FSDEV_STAT_RX_NAK = 0x2000U;    ///< 回复 NAK / Reply NAK
constexpr uint32_t FSDEV_STAT_RX_VALID = 0x3000U;  ///< 接收数据 / Accept data

class FSDevDevice;

/**
 * @class FSDevEndpoint
 * @brief FSDEV 端点，固定一个方向 / FSDEV endpoint with one fixed direction
 *
 * IN：启动时把第一个包拷进 PMA 并置 VALID，每个 CTR_TX（主机已 ACK）推进下一包。OUT：
 * 启动时置 VALID，每个 CTR_RX 把包从 PMA 拷进槽，短包或收满时完成。完成、推进都在
 * ServiceHardware 里；停止不等线上的事务，见 StopHardware。
 * IN: the first packet is copied into the PMA at start and STAT set VALID; every CTR_TX
 * (the host ACKed) advances the next packet. OUT: STAT is set VALID at start; every
 * CTR_RX copies the packet from the PMA into the slot, and a short packet or a full
 * request completes it. Completion and progress happen in ServiceHardware; a stop does
 * not wait for the transaction on the wire, see StopHardware.
 *
 * @note 停止，对应 Endpoint::StopHardware 的规则。
 *       停稳的证据：STAT 不是 VALID 时当场算停稳（硬件发完一包自己回 NAK，此后不会为
 *       本端点开始事务）；被停请求自己的 CTR；否则靠协议上界，写 NAK 之后帧号变了就
 *       结束。挂起、总线复位和控制端点上的 SETUP 也结束它。这个控制器没有停稳事件，
 *       也读不出线上有没有事务。
 *       缺陷：软件写寄存器会抹掉硬件的更新。EPnR 的 STAT 位写 1 翻转，写入值由读到的
 *       值算出；其间硬件结束了一个事务、自己回了 NAK 的话，停止的那次写就把 NAK 翻回
 *       VALID，改回去之前主机的下一个令牌若被接受，IN 把同一包再发一遍、OUT 多收一包。
 *       所以数据端点在改写期间把地址字段换成没有端点使用的号（控制器不理会发给它的
 *       令牌，主机当作一次传输错误重试），写后读回、不对就立刻再写，然后换回地址。
 *       另外三类没有：CTR 按端点、按方向各一位，不会被别的事务改写；停止只写本端点的
 *       寄存器。
 *       实测（STM32F302）：30 s 停止压力，停止 IN 7522 次、OUT 8436 次，结束于自己的
 *       CTR 506 次、帧号变化 14718 次（都只差一帧），TIMEOUT 0；总线挂起时未结束的
 *       停止 330 个全部在挂起处理里结束。
 *       实测（APM32F103 直接接根口，每轮 30 s 停止压力约 1 万次改写）：不换地址时每轮
 *       约 2 次“同一包发了两遍或多收一包”，读回后立刻再写仍有约 0.26 次；换地址后 18
 *       轮都是 0，主机没有收到传输错误。换地址的时间 3.2~4.2 us。
 *       Stops, against the rules of Endpoint::StopHardware.
 *       Evidence of quiescence: STAT other than VALID counts as quiescent at once (the
 *       hardware sets NAK itself after a packet and starts no transaction for the
 *       endpoint afterwards); the stopped request's own CTR; otherwise the protocol
 *       bound, a frame number that changed after NAK was written. Suspend, a bus reset
 *       and a SETUP on the control endpoint end it too. This controller has no
 *       quiescence event and cannot tell whether a transaction is on the wire.
 *       Flaw: a software write undoes an update of the hardware. The STAT bits of EPnR
 *       toggle on writing 1 and the value written is computed from a value read; if the
 *       hardware ended a transaction in between and set NAK itself, the write of a stop
 *       turns that NAK back into VALID, and if the host's next token is accepted before
 *       the correction an IN sends the same packet again and an OUT takes one packet
 *       more. So a data endpoint has its address field set to a number no endpoint uses
 *       while it is rewritten (the controller ignores a token for it and the host
 *       retries it as a transfer error); the value is read back, written again at once
 *       if wrong, and the address is set back.
 *       The other three kinds are absent: CTR is one bit per endpoint and direction and
 *       no other transaction rewrites it; a stop writes this endpoint's register only.
 *       Measured (STM32F302): 30 s of stop stress, 7,522 IN and 8,436 OUT stops, ended
 *       by the own CTR 506 times and by a frame change 14,718 times (always one frame
 *       later), 0 TIMEOUT; 330 stops pending at a bus suspend all ended in the suspend
 *       handling.
 *       Measured (APM32F103 on a root port, about 10,000 rewrites per 30 s round of stop
 *       stress): without the address change about 2 cases per round of "the same packet
 *       sent twice or one packet more taken", and about 0.26 with the immediate second
 *       write alone; with the address change 0 in 18 rounds and no transfer error at
 *       the host. The address is changed for 3.2 to 4.2 us.
 */
class FSDevEndpoint : public Endpoint
{
 public:
  /**
   * @brief 构造端点 / Construct the endpoint
   * @param device 所属设备 / Owning device
   * @param number 端点号，小于 FSDevPool::MAX_EPS / Endpoint number, below
   *        FSDevPool::MAX_EPS
   * @param direction IN 或 OUT / IN or OUT
   * @param buffer 槽内存 / Slot memory
   * @param pma_addr 本端点缓冲在 PMA 中的字节偏移 / Byte offset of the buffer in the PMA
   * @param pma_size 本端点 PMA 缓冲的字节数 / Size of the PMA buffer in bytes
   * @param slot_count 槽数，0 为核心默认值 / Slot count, 0 for the core default
   * @note 调用者：FSDevPool 的构造函数 / Caller: the constructor of FSDevPool
   */
  FSDevEndpoint(FSDevDevice& device, EPNumber number, Direction direction, RawData buffer,
                uint16_t pma_addr, size_t pma_size, uint8_t slot_count = 0);

 protected:
  /// 同号另一方向已启用时类型必须相同：两个方向共用 EP_TYPE / The other direction
  /// of the number, if enabled, must have the same type: both share EP_TYPE
  ErrorCode CheckHardware(const Config& cfg) const override;
  /// 写类型、缓冲地址和 OUT 的接收区大小，清本方向 DTOG，置 NAK / Write the type, the
  /// buffer address and the receive size of OUT, clear this direction's DTOG, set NAK
  void ConfigureHardware(const Config& cfg) override;
  /// 非 EP0 置 DISABLED；EP0 常驻不动 / DISABLED for non-EP0; EP0 stays as it is
  void CloseHardware(bool in_isr) override;
  /**
   * @brief 记下请求并装填第一包 / Record the request and arm the first packet
   * @note 装填写不进返回 FAILED。总线复位的 SE0 期间寄存器写不进不算失败：请求留在途中，
   *       由随后的总线复位结束。
   *       FAILED when the arm cannot be written. Registers that ignore writes during the
   *       SE0 of a bus reset are no failure: the request stays in flight and the bus
   *       reset that follows ends it.
   */
  ErrorCode StartHardware(const HwRequest& request, bool in_isr) override;
  /**
   * @brief 收下已锁存的完成，写 NAK；硬件此刻已停稳就给结果，否则 PENDING / Collect a
   *        latched completion and write NAK; the outcome if the hardware is quiescent
   *        now, PENDING otherwise
   * @note STAT 不是 VALID（包已完成硬件自己回了 NAK、SETUP 把两方向置 NAK、总线复位清了
   *       EPnR）时没有事务会开始，当场给结果；总线挂起时也当场给结果。STAT 为 VALID 时
   *       控制器没有标志能说明线上有没有本端点的事务，写 NAK 后返回 PENDING，由下面
   *       之一结束：本方向的 CTR（那个事务完成了）；帧号变了（写 NAK 之后收到过帧起始，
   *       事务不跨帧，所以写 NAK 时在线上的事务已经结束）；挂起；总线复位；控制端点的
   *       SETUP。EP0 OUT 上带 SETUP 的 CTR_RX 不收、不清，留给 ServiceHardware。未结束
   *       的时间通常在一帧以内（STM32F302 实测最长 1.28 ms）；恢复信号期间发出的停止要
   *       等到恢复后的第一个帧起始（实测最长 20 ms），总线复位的 SE0 期间要等到 RESET
   *       中断（实测最长 10.6 ms）。
   *       While STAT is not VALID (the packet completed and the hardware set NAK itself,
   *       a SETUP set both directions to NAK, a bus reset cleared EPnR) no transaction
   *       can start and the outcome is given at once; likewise while the bus is
   *       suspended. With STAT VALID the controller has no flag telling whether a
   *       transaction of this endpoint is on the wire, so NAK is written and PENDING
   *       returned, ended by one of: this direction's CTR (that transaction completed);
   *       a changed frame number (a start of frame came after the NAK, and transactions
   *       never cross a frame, so one on the wire at the NAK is over); suspend; bus
   *       reset; a SETUP on the control endpoint. A CTR_RX carrying a SETUP on EP0 OUT
   *       is neither collected nor cleared; ServiceHardware handles it. The pending
   *       time is normally within one frame (1.28 ms at most measured on an
   *       STM32F302); a stop issued during resume signalling waits for the first start
   *       of frame after the resume (20 ms at most measured), and one during the SE0 of
   *       a bus reset waits for the RESET interrupt (10.6 ms at most measured).
   * @note 返回 PENDING 之后才完成的包只清 CTR，不拷进槽（槽已放回）。NAK 写不进时在下
   *       一次 ServiceHardware 里重写一次，仍写不进就报 TIMEOUT。
   *       A packet completing after PENDING only has its CTR cleared and is not copied
   *       into the slot (which was given back). If NAK cannot be written it is written
   *       once more in the next ServiceHardware, and TIMEOUT is reported if that fails
   *       too.
   */
  StopOutcome StopHardware(bool in_isr) override;
  /// 置 STALL / Set STALL
  ErrorCode StallHardware(bool in_isr) override;
  /// 非 EP0 复位 DATA0 并置 NAK；EP0 无操作（下一个 SETUP 置两方向 NAK）/ Non-EP0: reset
  /// DATA0 and set NAK; no-op on EP0 (the next SETUP sets both directions to NAK)
  ErrorCode ClearStallHardware(bool in_isr) override;
  /**
   * @brief 最大包长由 PMA 缓冲决定 / The largest packet follows from the PMA buffer
   * @note IN 为缓冲字节数；OUT 超过 62 字节时按 32 字节块向下取整（COUNT_RX 的块编码）；
   *       两者再受 10 位计数限制（1023）；CONTROL 不超过 64。一次请求不限包数：后端
   *       逐包推进。
   *       IN: the buffer size; OUT: rounded down to 32-byte blocks above 62 bytes (block
   *       encoding of COUNT_RX); both bound by the 10-bit count (1023); CONTROL at most
   *       64. No packet limit per request: the backend advances packet by packet.
   */
  HwLimits HardwareLimits(Direction direction, Type type) const override;

 private:
  friend class FSDevDevice;

  /// 是否为 IN 端点 / Whether this is an IN endpoint
  bool IsIn() const { return AvailableDirection() == Direction::IN; }
  /// 端点号 / Endpoint number
  uint8_t Num() const { return EPNumberToInt8(GetNumber()); }

  /**
   * @brief 把本方向 STAT 写成 target，写后读回 / Write this direction's STAT to target
   *        and read it back
   * @return true：已到达，或装填时事务已完成（STAT 是硬件自动回的 NAK）或 EP0 有未处理
   *         的 SETUP（不装填）；false：三轮重算仍未到达。
   *         true: reached, or when arming the transaction has already completed (STAT is
   *         the NAK the hardware set) or EP0 has an unhandled SETUP (not armed); false:
   *         not reached after three recomputations.
   * @note EP0 装填时读到未处理的 SETUP 就不装填；写后才看到的 SETUP 改写回 NAK。
   *       When arming EP0 and an unhandled SETUP is read, it does not arm; a SETUP seen
   *       only after the write makes it write NAK back.
   */
  bool SetStat(uint32_t target);
  /// 写类型和地址字段 / Write the type and address fields
  void WriteType(uint32_t bits);
  /// 把本方向 DTOG 清为 DATA0 / Reset this direction's DTOG to DATA0
  void ClearDataToggle();
  /**
   * @brief 装填当前包 / Arm the current packet
   * @note IN 拷入 PMA 并写计数，OUT 写接收区大小；然后置 VALID。
   *       IN copies into the PMA and writes the count, OUT writes the receive size; then
   *       sets VALID.
   */
  bool Arm();
  /// 读值里有本方向的完成可收：CTR 置位，且不是 EP0 OUT 上的 SETUP / The value holds a
  /// completion of this direction to collect: CTR set and not a SETUP on EP0 OUT
  bool Completed(uint32_t epr) const;
  /**
   * @brief 收下一个已完成的包并清本方向 CTR / Collect one completed packet and clear
   *        this direction's CTR
   * @param result 请求结束时写入：OK，或 OUT 包超过剩余空间时 OUT_OF_RANGE / Written when
   *        the request ends: OK, or OUT_OF_RANGE when an OUT packet exceeds the room left
   * @return 请求是否结束 / Whether the request has ended
   */
  bool TakePacket(ErrorCode& result);
  /// ServiceHardware 中本方向 CTR：收包，结束则报告，否则装填下一包 / This direction's
  /// CTR in ServiceHardware: collect, report if ended, otherwise arm the next packet
  void OnPacket(bool in_isr);
  /// EP0 OUT：处理快速确认时拷出的那个包（CTR_RX 已清）/ EP0 OUT: handle the packet
  /// the fast acknowledgement copied out (CTR_RX already cleared)
  void OnShadowPacket(bool in_isr, const uint8_t* data, size_t bytes);
  /// 清本方向 CTR，不收包 / Clear this direction's CTR without collecting the packet
  void DropPacket();
  /**
   * @brief 本端点是 EP0 OUT，且它的包可能早于请求被收下（FSDevDevice::EnableEp0EarlyOut）
   *        / This is EP0 OUT and its packets may be taken before their request
   *        (FSDevDevice::EnableEp0EarlyOut)
   */
  bool EarlyOut() const;
  /// 总线复位的 SE0 期间控制器把 EPnR 按在 0 上、写不进 / The controller holds EPnR at 0
  /// and ignores writes during the SE0 of a bus reset
  bool HeldInReset() const;
  /**
   * @brief 推进未结束的停止 / Advance the pending stop
   * @param frame 调用者在读 EPnR 之前读到的帧号 / Frame number the caller read before
   *        any EPnR
   * @param force 调用者已知线上没有本端点的事务（本方向的 CTR、总线复位、挂起、控制
   *        端点的 SETUP）/ The caller knows no transaction of this endpoint is on the
   *        wire (this direction's CTR, bus reset, suspend, a SETUP on the control
   *        endpoint)
   * @note 调用者：ServiceHardware。停稳时清掉期间锁存的 CTR 并报告一次；NAK 还没写进时
   *       重写，仍写不进报 TIMEOUT，之后不再报告。
   *       Caller: ServiceHardware. When quiescent it clears a CTR latched meanwhile and
   *       reports once; if NAK is not written yet it writes it again and reports
   *       TIMEOUT if that fails, after which it reports nothing more.
   */
  void AdvanceStop(bool in_isr, uint16_t frame, bool force);

  FSDevDevice& device_;      ///< 所属设备；构造时固定 / Owning device; fixed
  const uint16_t pma_addr_;  ///< PMA 字节偏移；构造时固定 / PMA byte offset; fixed
  const size_t pma_size_;    ///< PMA 字节数；构造时固定 / PMA bytes; fixed

  /// @name 在途请求：仅 owner / Request in flight: owner only
  /// @{
  /**
   * @brief 硬件上有请求 / A request is on the hardware
   * @note StartHardware 置位；请求结束时清除（OnPacket 报告，或 StopHardware 返回）。
   *       ServiceHardware 读：没有请求时到达的 CTR 只清除，不计入。
   *       Set by StartHardware; cleared when the request ends (OnPacket reports, or
   *       StopHardware returns). Read by ServiceHardware: a CTR arriving with no request
   *       is only cleared, never counted.
   */
  bool busy_ = false;
  /// EP0 OUT：状态包的请求已受理，STAT_RX 还留在 NAK，等 ReleaseEp0Status 放行 / EP0
  /// OUT: the status request is accepted, STAT_RX is still NAK until ReleaseEp0Status
  bool status_held_ = false;
  /// StopHardware 返回了 PENDING，停止还没有报告 / StopHardware returned PENDING and
  /// the stop is not reported yet
  bool stopping_ = false;
  /// 停止的 NAK 已写进 / The NAK of the stop is written
  bool nak_written_ = false;
  /// 已为这次停止报告 TIMEOUT，不再报告 / TIMEOUT was reported for this stop; nothing
  /// more is reported
  bool stop_failed_ = false;
  /// 写下 NAK 之后读到的帧号 / Frame number read after the NAK was written
  uint16_t stop_frame_ = 0;
  uint8_t* data_ = nullptr;  ///< 请求数据；StartHardware 写 / Request data; by Start
  size_t length_ = 0;        ///< 请求长度；StartHardware 写 / Request length; by Start
  /// IN：主机已 ACK 的字节；OUT：已收的字节。StartHardware 清零，收包时增加 / IN: bytes
  /// the host ACKed; OUT: bytes received. Zeroed by Start, raised per collected packet
  size_t progress_ = 0;
  /// @}
};

/**
 * @class FSDevPool
 * @brief 本控制器的端点池：布 PMA、建端点 / Endpoint pool of this controller: lays out
 *        the PMA and creates the endpoints
 * @note 设备核心构造时要求池里已有 EP0，所以端点在设备核心之前建好；本类作为
 *       FSDevDevice 排在 DeviceCore 之前的基类。不访问硬件。
 *       The device core requires EP0 in the pool at its construction, so the endpoints
 *       are built before it; this class is the base of FSDevDevice placed before
 *       DeviceCore. Does not touch the hardware.
 */
class FSDevPool : public EndpointPool
{
 public:
  /// 硬件的端点号数量 / Number of endpoint numbers of the hardware
  static constexpr uint8_t MAX_EPS = 8;
  static_assert(MAX_EPS <= FSDEV_EP_PARK_ADDR, "the park address is an endpoint number");

  /// 端点配置：槽内存与 PMA 缓冲字节 / Endpoint configuration: slot memory and PMA bytes
  struct EPConfig
  {
    RawData buffer;   ///< 槽内存 / Slot memory
    size_t pma_size;  ///< PMA 字节数，MemoryMap::alignment 的倍数 / PMA bytes, a multiple
                      ///< of MemoryMap::alignment
  };

  /// 控制器的地址与 PMA 的形状 / Addresses of the controller and shape of the PMA
  struct MemoryMap
  {
    uintptr_t registers;        ///< 寄存器块基址（EP0R）/ Register block base (EP0R)
    uintptr_t packet_memory;    ///< PMA 基址 / PMA base
    size_t packet_memory_size;  ///< PMA 字节数 / PMA bytes
    /// 经典型 PMA 里每个 16 位数据字占几个 16 位地址（2 或 1）/ How many 16-bit
    /// addresses one 16-bit data word takes in a classic PMA (2 or 1)
    uint8_t word_stride;
    uint8_t alignment;  ///< 缓冲地址与长度的对齐字节数 / Alignment of buffers in bytes
  };

 protected:
  /**
   * @brief 布 PMA 并建全部端点 / Lay out the PMA and create every endpoint
   * @param device 端点所属设备（构造中）/ Device of the endpoints (under construction)
   * @param memory_map PMA 的形状 / Shape of the PMA
   * @param rx_ep_cfgs OUT 端点，第 i 项是 EP i，EP0 必须有 / OUT endpoints, entry i is
   *        EP i, EP0 mandatory
   * @param tx_ep_cfgs IN 端点，规则同上 / IN endpoints, same rules
   * @param pma_limit PMA 可用字节，0 为全部 / Usable PMA bytes, 0 for all
   * @param slot_count 每个端点的槽数，0 为核心默认 / Slots per endpoint, 0 for default
   * @note 布局：BTABLE 每端点 8 字节在 0，之后依次是各 OUT、各 IN 缓冲。
   *       Layout: the BTABLE takes 8 bytes per endpoint at 0, followed by every OUT,
   *       then every IN buffer.
   */
  FSDevPool(FSDevDevice& device, const MemoryMap& memory_map,
            const std::initializer_list<EPConfig>& rx_ep_cfgs,
            const std::initializer_list<EPConfig>& tx_ep_cfgs, size_t pma_limit,
            uint8_t slot_count);

  /// 按端点号的 IN 端点；构造时写 / IN endpoints by number; written at construction
  FSDevEndpoint* in_[MAX_EPS] = {};
  /// 按端点号的 OUT 端点；构造时写 / OUT endpoints by number; written at construction
  FSDevEndpoint* out_[MAX_EPS] = {};
  /// 用到的端点号数量；构造时写 / Endpoint numbers in use; written at construction
  uint8_t endpoint_count_ = 1;
};

/**
 * @class FSDevDevice
 * @brief FSDEV 设备：各平台 FSDEV 设备类的基类 / FSDEV device: base of the platforms'
 *        FSDEV device classes
 *
 * 实现设备服务的 ServiceHardware 和设备核心的后端函数；MaskInterrupt、UnmaskInterrupt、
 * PendInterrupt 和 Connect 由平台实现。一个程序只能有一个对象（Dispatch 是静态入口）。
 * Implements the device service's ServiceHardware and the device core's backend
 * functions; MaskInterrupt, UnmaskInterrupt, PendInterrupt and Connect are the
 * platform's. A program has one object only (Dispatch is a static entry).
 *
 * @note ServiceHardware 的顺序：读 ISTR 快照；RESET；CTR 循环（每轮一个端点，最多 8
 *       轮）；按帧号推进未结束的停止；SOF；SUSP；WKUP；噪声位。一个端点先处理 CTR_TX
 *       再处理 CTR_RX，EP0 的 SETUP 只在读到 CTR_TX 已清时交给设备核心，满足 SETUP 顺序
 *       约定。
 *       Order in ServiceHardware: ISTR snapshot; RESET; CTR loop (one endpoint per pass,
 *       at most 8 passes); pending stops advanced by the frame number; SOF; SUSP; WKUP;
 *       noise flags. Per endpoint CTR_TX comes before CTR_RX, and an EP0 SETUP goes to
 *       the device core only when CTR_TX reads clear, which meets the SETUP ordering
 *       contract.
 * @note 未结束的停止靠帧起始中断得到下一轮 ServiceHardware：运行时 SOF 中断一直开着；
 *       总线挂起时关掉，那时停止当场完成。不用毫秒节拍，也不要求 PendInterrupt。
 *       A pending stop gets its next ServiceHardware pass from the start-of-frame
 *       interrupt: it stays enabled while running and is off while the bus is suspended,
 *       when stops complete at once. No millisecond tick is used and PendInterrupt is
 *       not required.
 */
class FSDevDevice : public DeviceService, public FSDevPool, public DeviceCore
{
 public:
  ~FSDevDevice() override;

  /**
   * @brief 打开 CTR、RESET、SUSP、WKUP、SOF 中断，最后接上拉 / Enable the CTR, RESET,
   *        SUSP, WKUP and SOF interrupts, then connect the pull-up
   * @note 调用者：应用，任何线程，在 InitController 之后。不经 owner：此前控制器中断源
   *       全关，写 CNTR 不与 ServiceHardware 竞争。
   *       Caller: the application, any thread, after InitController. Not through the
   *       owner: until here every interrupt source is off, so the CNTR write does not
   *       race ServiceHardware.
   */
  void StartController(bool in_isr) override;

  /**
   * @brief 断开总线并让控制器静止 / Detach from the bus and make the controller
   *        quiescent
   * @note 仅 owner，处理 Stop 的请求时调用，见 DeviceCore::StopController。与
   *       ResetController 的前半相同，只是不再接上总线，并关掉控制器的中断源。
   *       Owner only, called when the Stop request is served, see
   *       DeviceCore::StopController. The same as the first half of ResetController,
   *       except that the bus is not attached again and the controller's interrupt
   *       sources are switched off.
   */
  void StopController(bool in_isr) override;

  /**
   * @brief 中断入口 / Interrupt entry
   * @note 调用者：平台的控制器中断函数。InitController 之前什么也不做（共享 CAN 向量时
   *       CAN 中断也会进来）。
   *       Caller: the platform's controller interrupt handler. Does nothing before
   *       InitController (with a vector shared with CAN, CAN interrupts come here too).
   */
  static void Dispatch();

 protected:
  /**
   * @brief 构造设备、布 PMA、建端点并登记到设备服务；不访问硬件 / Construct the device,
   *        lay out the PMA, create the endpoints and register them with the device
   *        service; the hardware is not touched
   * @note 参数见 FSDevPool 和 DeviceCore / Parameters as for FSDevPool and DeviceCore
   */
  FSDevDevice(
      const MemoryMap& memory_map, const std::initializer_list<EPConfig>& rx_ep_cfgs,
      const std::initializer_list<EPConfig>& tx_ep_cfgs,
      DeviceDescriptor::PacketSize0 packet_size, uint16_t vid, uint16_t pid, uint16_t bcd,
      const std::initializer_list<const DescriptorStrings::LanguagePack*>& languages,
      const std::initializer_list<const std::initializer_list<DeviceClass*>>& configs,
      ConstRawData uid = {nullptr, 0}, size_t pma_limit = 0, uint8_t slot_count = 0);

  /**
   * @brief 把控制器置为“已复位、中断源关、未接上拉” / Bring the controller to "reset,
   *        interrupt sources off, pull-up off"
   * @note 调用者：平台构造函数在打开时钟后一次；ResetController。
   *       Callers: the platform constructor once after enabling the clocks;
   *       ResetController.
   */
  void InitController();

  /**
   * @brief 打开 EP0 的快速确认 / Enable the fast acknowledgement of EP0
   * @note 本控制器在 EP0 的 CTR_RX 没清之前不收新的 SETUP；主机把控制传输排着队发时，
   *       下一个 SETUP 在状态包之后几十微秒就到，等 owner 来清就晚了（STM32F302 实测：
   *       空闲时进中断到清除 23~40 µs，紧跟控制读之后的传输失败 0.02%~0.7%）。打开后，
   *       Dispatch 在把事件交给 owner 之前先看 EP0：收到的是普通包（不是 SETUP）就把它
   *       拷进一块影子缓冲、清 CTR_RX；owner 那一轮先处理影子里的包，顺序不变。
   *       This controller takes no new SETUP while CTR_RX of EP0 is set; a host that
   *       queues control transfers sends the next SETUP some tens of microseconds after
   *       the status packet, too early for the owner to have cleared it (measured on an
   *       STM32F302: 23 to 40 us from interrupt entry to the clear when idle, 0.02 to
   *       0.7 % of the transfers following a control read failing). When enabled,
   *       Dispatch looks at EP0 before handing the events to the owner: an ordinary
   *       packet (no SETUP) is copied to a shadow buffer and CTR_RX cleared; the owner
   *       handles the packet in the shadow first in its pass, so the order is unchanged.
   * @note 只有 owner 总在控制器中断里运行的平台（PendInterrupt 返回 true）可以打开：
   *       那时快速确认和 owner 在同一个中断里先后执行，不会同时碰 EP0R。平台构造时
   *       调用。owner 正在跑一轮时来的包仍要等这一轮结束，快速确认缩短不了那一段。
   *       Only a platform whose owner always runs in the controller interrupt
   *       (PendInterrupt returns true) may enable it: the fast acknowledgement and the
   *       owner then run one after the other in the same interrupt and never touch EP0R
   *       at the same time. Called at the platform's construction. A packet arriving
   *       while an owner pass is running still waits for the end of that pass; the fast
   *       acknowledgement cannot shorten that part.
   */
  void EnableEp0FastAck() { ep0_fast_ack_ = true; }

  /**
   * @brief 打开状态 OUT 的晚放行 / Enable the late release of the status OUT
   * @note 控制读里设备核心在第一包数据之前就请求接收状态包。打开后，这个请求照常受理，
   *       但 STAT_RX 留在 NAK；等 EP0 IN 上没有包在途（最后一包数据已完成，owner 也已
   *       处理完），Dispatch 在 owner 那一轮之后才写 VALID。这是手册规定的顺序（RM0008
   *       p.631、RM0365 p.1019）：放行最后一个数据阶段时相反方向保持 NAK，控制操作完成
   *       后再改成 VALID。状态包因此不会在 owner 处理“数据发完”的那一轮中间被收下，
   *       它的 CTR_RX 不会留到那一轮结束而挡住主机的下一个 SETUP。
   *       In a control read the device core requests the status packet before the first
   *       data packet. When enabled, the request is accepted as usual but STAT_RX stays
   *       NAK; once no packet is in flight on EP0 IN (the last data packet completed and
   *       the owner handled it), Dispatch writes VALID after the owner's pass. This is
   *       the order the manual prescribes (RM0008 p.631, RM0365 p.1019): while the last
   *       data stage is enabled the opposite direction stays NAK, and it becomes VALID
   *       when the control operation has completed. The status packet is therefore never
   *       taken in the middle of the pass that handles the sent data, and its CTR_RX
   *       does not stay until the end of that pass, blocking the host's next SETUP.
   * @note 放行之后到状态包被收下之前，Dispatch 不让 owner 开始新的一轮：其他端点的完成
   *       只清 CTR、记下，帧起始只清标志、计数；状态包（或 SETUP、总线复位、挂起、唤醒）
   *       一到，owner 才进来，先处理记下的完成。硬件在每个包之后已把那个端点置 NAK，数据
   *       留在 PMA 里（RM0008 p.630），所以晚处理不丢包。这样状态包到达时控制器中断里
   *       没有长的一轮在跑，它的 CTR_RX 由下一次中断入口清掉。主机迟迟不发状态包时，数到
   *       STATUS_WINDOW_SOFS 个帧起始就恢复正常处理。这段时间里其他上下文发给 owner 的
   *       请求也一并推迟。
   *       Between the release and the taking of the status packet Dispatch keeps the
   *       owner from starting a pass: completions of other endpoints only have their CTR
   *       cleared and are latched, a start of frame only has its flag cleared and is
   *       counted; when the status packet (or a SETUP, a bus reset, a suspend, a wake-up)
   *       arrives the owner comes in and handles the latched completions first. The
   *       hardware has set such an endpoint to NAK after its packet and the data stay in
   *       the PMA (RM0008 p.630), so handling them later loses nothing. No long pass is
   *       then running in the controller interrupt when the status packet arrives, and
   *       the next interrupt entry clears its CTR_RX. If the host does not send the
   *       status packet, normal handling resumes after STATUS_WINDOW_SOFS starts of
   *       frame. Requests other contexts make to the owner are postponed for the same
   *       time.
   * @note 数据阶段还没结束时主机提前发来的状态包得到 NAK。最后一包数据的 ACK 在总线上
   *       丢失时，设备看不到“数据发完”，状态包一直得到 NAK，到主机放弃后的下一个 SETUP
   *       为止。
   *       A status packet the host sends before the data stage ended is answered NAK.
   *       When the ACK of the last data packet is lost on the bus, the device never sees
   *       the data as sent and the status packet is answered NAK until the next SETUP
   *       after the host gave up.
   * @note 平台条件与 EnableEp0FastAck 相同：owner 总在控制器中断里运行，放行和 owner 在
   *       同一个中断里先后执行。平台构造时调用。
   *       The platform condition is that of EnableEp0FastAck: the owner always runs in
   *       the controller interrupt, so the release and the owner run one after the other
   *       in the same interrupt. Called at the platform's construction.
   */
  void EnableEp0LateStatus() { ep0_late_status_ = true; }

  /**
   * @brief 声明 EP0 的 OUT 包可能早于它的请求被收下 / Declare that an OUT packet of EP0
   *        may be taken before its request
   * @note 有的控制器在控制端点上不按 STAT_RX 回 NAK：SETUP 之后、接收请求启动之前主机
   *       发来的数据包或状态包也被 ACK，并锁存 CTR_RX。打开后，EP0 OUT 的请求启动时已
   *       锁存的普通包算作它的第一包：不清掉，由 CTR 处理收下。请求一直没有启动时，这个
   *       包在下一次中断里被丢弃（主机已收到 ACK）。要求同时打开 EnableEp0FastAck。平台
   *       构造时调用。
   *       Some controllers do not answer NAK from STAT_RX on a control endpoint: a data
   *       or status packet the host sends after the SETUP and before the receive request
   *       starts is ACKed too and latches CTR_RX. When enabled, an ordinary packet
   *       latched when a request of EP0 OUT starts counts as its first packet: it is not
   *       cleared and the CTR handling takes it. If no request starts, the packet is
   *       dropped in the next interrupt (the host has seen the ACK). Requires
   *       EnableEp0FastAck as well. Called at the platform's construction.
   */
  void EnableEp0EarlyOut() { ep0_early_out_ = true; }

  /// 接通或断开 D+ 上拉；平台实现 / Connect or disconnect the D+ pull-up; platform
  virtual void Connect(bool on) = 0;

  /// 读偏移处的寄存器，经典型 16 位访问 / Read a register, 16-bit access on classic
  virtual uint32_t ReadRegister(uint32_t offset);
  /// 写偏移处的寄存器 / Write a register
  virtual void WriteRegister(uint32_t offset, uint32_t value);
  /// 写端点 num 发送缓冲地址 / Write endpoint num's transmit buffer address
  virtual void SetTxBuf(uint8_t num, uint16_t addr);
  /// 写端点 num 下一个 IN 包的字节数 / Write the byte count of endpoint num's next IN
  virtual void SetTxCount(uint8_t num, uint16_t bytes);
  /// 写端点 num 接收缓冲地址 / Write endpoint num's receive buffer address
  virtual void SetRxBuf(uint8_t num, uint16_t addr);
  /// 按包长 mps 写端点 num 接收区大小（块编码）/ Write the receive size of endpoint num
  /// for packet size mps (block encoding)
  virtual void SetRxCountEncoded(uint8_t num, uint16_t mps);
  /// 端点 num 收到的字节数 / Bytes endpoint num received
  virtual uint16_t GetRxCount(uint8_t num);
  /// 把 len 字节拷进 PMA 偏移 addr / Copy len bytes into the PMA at offset addr
  virtual void PmaWrite(uint16_t addr, const uint8_t* src, size_t len);
  /// 从 PMA 偏移 addr 拷出 len 字节 / Copy len bytes out of the PMA at offset addr
  virtual void PmaRead(uint8_t* dst, uint16_t addr, size_t len);
  /// 复位控制器并清全部中断标志 / Reset the controller and clear every interrupt flag
  virtual void InitRegs();
  /// 进入挂起 / Enter suspend
  virtual void SuspendRegs();
  /// 挂起标志清除后的动作，经典型没有 / Step after the suspend flag is cleared; none on
  /// classic
  virtual void SuspendClocked();
  /// 离开挂起 / Leave suspend
  virtual void ResumeRegs();

  /// 处理硬件事件，见类说明 / Handle hardware events, see the class notes
  void ServiceHardware(bool in_isr) override;
  /// 状态 IN 完成后写 DADDR / Write DADDR once the status IN has completed
  void SetAddress(uint8_t address, AddressStage stage) override;
  /**
   * @brief 停止失败后复位控制器 / Reset the controller after a failed stop
   * @note 断开上拉、强制复位、重写 BTABLE、等够最短断开时间、打开中断源、接上拉。复位清掉
   *       全部 EPnR（STAT 为 DISABLED、没有 CTR），未结束的停止就此作废、不再报告，之后
   *       的 StopHardware 返回 ABORTED。
   *       Disconnects the pull-up, forces a reset, writes the BTABLE again, waits out the
   *       minimum detach time, enables the interrupt sources and connects the pull-up.
   *       The reset clears every EPnR (STAT DISABLED, no CTR); pending stops are void and
   *       reported no more, and a later StopHardware returns ABORTED.
   */
  void ResetController(bool in_isr) override;

  /// 读 EPnR / Read EPnR
  uint32_t ReadEp(uint8_t num) { return ReadRegister(static_cast<uint32_t>(num) * 4U); }
  /// 写 EPnR / Write EPnR
  void WriteEp(uint8_t num, uint32_t value)
  {
    WriteRegister(static_cast<uint32_t>(num) * 4U, value);
  }
  /// 读 CNTR / Read CNTR
  uint16_t ReadCntr() { return static_cast<uint16_t>(ReadRegister(FSDEV_REG_CNTR)); }
  /// 写 CNTR / Write CNTR
  void WriteCntr(uint16_t value) { WriteRegister(FSDEV_REG_CNTR, value); }
  /// 读 ISTR / Read ISTR
  uint16_t ReadIstr() { return static_cast<uint16_t>(ReadRegister(FSDEV_REG_ISTR)); }
  /// 清给出的 ISTR 标志（写 0），其余写 1 保持 / Clear the given ISTR flags (write 0),
  /// keep the others (write 1)
  void ClearIstr(uint16_t flags)
  {
    WriteRegister(FSDEV_REG_ISTR, ~static_cast<uint32_t>(flags));
  }
  /// 写 DADDR / Write DADDR
  void WriteDaddr(uint16_t value) { WriteRegister(FSDEV_REG_DADDR, value); }
  /// 清发送方向 CTR，保持接收方向 / Clear the transmit CTR, keep the receive one
  void ClearCtrTx(uint8_t num);
  /// 清接收方向 CTR，保持发送方向 / Clear the receive CTR, keep the transmit one
  void ClearCtrRx(uint8_t num);

  /// 控制器地址与 PMA 形状；构造时固定 / Controller addresses and PMA shape; fixed
  const MemoryMap memory_map_;
  /// EPnR 里接收、发送方向的错误位；经典型为 0 / Error bits of EPnR; 0 on classic
  uint32_t ep_err_rx_ = 0U;
  uint32_t ep_err_tx_ = 0U;  ///< 见 ep_err_rx_ / See ep_err_rx_
  /// 写 EPnR 时写 1 保持的位；变体在 InitController 之前改写 / Bits a write to EPnR keeps
  /// by writing 1; a variant sets it before InitController
  uint32_t ep_keep_mask_ = FSDEV_EP_KEEP_MASK;

 private:
  friend class FSDevEndpoint;

  /**
   * @brief 断开总线，让硬件不再为任何端点工作，作废未结束的停止 / Detach, make the
   *        hardware stop working for every endpoint and void the pending stops
   * @note StopController 和 ResetController 共用。
   *       Shared by StopController and ResetController.
   */
  void HaltController();
  /**
   * @brief HaltController 断开之后等够 DETACH_US / Wait until DETACH_US have passed since
   *        HaltController detached
   * @note StartController 和 ResetController 在重新接上之前调用。
   *       Called by StartController and ResetController before they attach.
   */
  void WaitDetached();
  /// 断开至少保持这么久主机才看得见（µs）/ Time a detach must last for the host to
  /// see it (µs)
  static constexpr uint64_t DETACH_US = 5000U;
  /// HaltController 断开总线的时刻（µs）和“断开后还没有重新接上” / Time HaltController
  /// detached (us) and "not attached again since"
  uint64_t halted_at_us_ = 0U;
  bool halted_ = false;
  /// CTR 循环，最多 8 轮 / CTR loop, at most 8 passes
  void HandleCtr(bool in_isr);
  /// 处理端点 num 的 CTR_TX、CTR_RX / Handle CTR_TX and CTR_RX of endpoint num
  void HandleEp(bool in_isr, uint8_t num);
  /// 处理端点 num 的 CTR_TX / Handle CTR_TX of endpoint num
  void HandleCtrTx(bool in_isr, uint8_t num);
  /// 处理端点 num 的 CTR_RX（不是 SETUP）/ Handle CTR_RX of endpoint num (no SETUP)
  void HandleCtrRx(bool in_isr, uint8_t num);
  /**
   * @brief 快速确认：EP0 的普通包拷进影子缓冲并清 CTR_RX / Fast acknowledgement: copy
   *        an ordinary EP0 packet to the shadow and clear CTR_RX
   * @note 在 Dispatch 里、owner 之前调用。
   *       Called in Dispatch, before the owner.
   */
  void FastAckEp0();
  /// owner 处理影子里的包：先处理 EP0 已锁存的 CTR_TX，再交付这个包 / The owner
  /// handles the packet in the shadow: a latched CTR_TX of EP0 first, then the packet
  void ServiceEp0Shadow(bool in_isr);
  /**
   * @brief 放行被留住的状态 OUT / Release the held status OUT
   * @note 调用者：Dispatch，在 owner 那一轮之后。EP0 IN 上还有包在途就不动；否则把
   *       STAT_RX 写成 VALID，写不进就留到下一次中断再写。不报告任何结果。
   *       Caller: Dispatch, after the owner's pass. Nothing happens while a packet is in
   *       flight on EP0 IN; otherwise STAT_RX is written VALID, and a write that does
   *       not take is repeated at the next interrupt. Nothing is reported.
   */
  void ReleaseEp0Status();
  /**
   * @brief 等状态包期间的中断入口 / Interrupt entry while the status packet is awaited
   * @return true：这次只有其他端点的完成和帧起始，已记下，owner 不进来；false：等待已结
   *         束，owner 照常进来 / true: only completions of other endpoints and starts of
   *         frame were pending and are latched, the owner stays out; false: the wait is
   *         over and the owner comes in as usual
   * @note 调用者：Dispatch，在快速确认之后、owner 之前。
   *       Caller: Dispatch, after the fast acknowledgement and before the owner.
   */
  bool HoldStatusWindow();
  /// owner 处理等状态包期间记下的完成 / The owner handles the completions latched
  /// while the status packet was awaited
  void ServiceLatched(bool in_isr);
  /// 等状态包最多数这么多个帧起始 / Starts of frame counted at most while the status
  /// packet is awaited
  static constexpr uint8_t STATUS_WINDOW_SOFS = 4U;

  /// 快速确认已打开；平台构造时写 / Fast acknowledgement enabled; set at the
  /// platform's construction
  bool ep0_fast_ack_ = false;
  /// 状态 OUT 的晚放行已打开；平台构造时写 / Late release of the status OUT enabled;
  /// set at the platform's construction
  bool ep0_late_status_ = false;
  /// EP0 的 OUT 包可能早于请求被收下；平台构造时写 / An OUT packet of EP0 may be taken
  /// before its request; set at the platform's construction
  bool ep0_early_out_ = false;
  /**
   * @brief 状态 OUT 已放行、状态包还没到 / The status OUT is released and the status
   *        packet not yet taken
   * @note ReleaseEp0Status 置位，HoldStatusWindow 清零。owner 只经 Dispatch 进来，而
   *       Dispatch 在它为真时先调 HoldStatusWindow，所以 owner 运行期间它总是假，记下的完
   *       成也已在那一轮的 ServiceHardware 开头处理掉。
   *       Set by ReleaseEp0Status, cleared by HoldStatusWindow. The owner comes in
   *       through Dispatch only, and Dispatch calls HoldStatusWindow first while this is
   *       true, so it is always false while the owner runs, and the latched completions
   *       were handled at the start of that pass's ServiceHardware.
   */
  bool status_window_ = false;
  /// 这次等待里已数到的帧起始 / Starts of frame counted in this wait
  uint8_t window_sofs_ = 0U;
  /// 等待期间清掉过帧起始标志，owner 还没有报告 / A start-of-frame flag was cleared
  /// during the wait and the owner has not reported it yet
  bool sof_latched_ = false;
  /// 等待期间记下的完成：位 n 是端点 n 的 OUT，位 8+n 是端点 n 的 IN / Completions
  /// latched during the wait: bit n is OUT of endpoint n, bit 8+n is IN of endpoint n
  uint16_t latched_ = 0U;
  /// 影子里有一个包等 owner 处理；FastAckEp0 置位，owner 清零 / A packet in the shadow
  /// awaits the owner; set by FastAckEp0, cleared by the owner
  bool ep0_shadow_pending_ = false;
  /// 影子里那个包的 COUNT_RX / COUNT_RX of the packet in the shadow
  uint16_t ep0_shadow_count_ = 0;
  /// EP0 OUT 包的影子缓冲 / Shadow buffer of an EP0 OUT packet
  uint8_t ep0_shadow_[64] = {};
  /// 当前帧号 / The current frame number
  uint16_t FrameNumber()
  {
    return static_cast<uint16_t>(ReadRegister(FSDEV_REG_FNR) & 0x07FFU);
  }
  /**
   * @brief 推进所有未结束的停止 / Advance every pending stop
   * @param force 总线上没有事务（总线复位、挂起）/ No transaction is on the bus (bus
   *        reset, suspend)
   * @note 先读帧号再读各 EPnR：帧号变了之后读到的 EPnR 里，写 NAK 时在线上的那个事务
   *       的 CTR 已经锁存。
   *       The frame number is read before any EPnR: an EPnR read after the frame number
   *       changed already holds the CTR of the transaction that was on the wire at the
   *       NAK.
   */
  void AdvanceStops(bool in_isr, bool force);

  /// 停止未结束的端点数；仅 owner / Endpoints whose stop is pending; owner only
  uint8_t pending_stops_ = 0;
  /// 总线已挂起（SUSP 之后，WKUP 或 RESET 之前）；仅 owner / The bus is suspended (after
  /// SUSP, before WKUP or RESET); owner only
  bool suspended_ = false;
  /// 本次挂起里被判为噪声的唤醒次数；仅 owner / Wake-ups of this suspend that were
  /// taken for noise; owner only
  uint8_t noise_wakeups_ = 0;
  /// 一次挂起里最多把这么多次唤醒判为噪声 / Wake-ups of one suspend taken for noise at
  /// most
  static constexpr uint8_t MAX_NOISE_WAKEUPS = 2U;
  /// 经典型 BTABLE 字段（0 ADDR_TX，1 COUNT_TX，2 ADDR_RX，3 COUNT_RX）/ Classic BTABLE
  /// field (0 ADDR_TX, 1 COUNT_TX, 2 ADDR_RX, 3 COUNT_RX)
  volatile uint16_t* BtField(uint8_t num, unsigned field) const;

  /// InitController 已调用；InitController 写，Start 和 Dispatch 读 / InitController
  /// has run; written by InitController, read by Start and Dispatch
  bool controller_ready_ = false;

  /// 唯一实例，Dispatch 的入口 / The single instance, entry of Dispatch
  static inline FSDevDevice* self_ = nullptr;
};

}  // namespace LibXR::USB
