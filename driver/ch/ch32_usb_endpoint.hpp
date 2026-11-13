#pragma once

#include "ch32_usb.hpp"
#include "libxr_def.hpp"
#include "libxr_type.hpp"
#include "usb/core/ep.hpp"

namespace LibXR
{

// CH32 OTG FS/HS 端点仍使用槽模型之前的后端接口，将在第 5 步迁移。在此之前，
// 旧类只在定义了 LIBXR_CH32_USB_OTG_LEGACY 时编译，这样同时定义 USBFSD 的器件（V203
// 同时具有 DevFS 设备和 OTG FS 外设）可以构建已迁移的 DevFS 路径。
// 该开关将随第 5 步迁移一并移除。
// The CH32 OTG FS/HS endpoints still speak the pre-slot-model backend interface; they
// are migrated in step 5. Until then the legacy classes are only compiled when
// LIBXR_CH32_USB_OTG_LEGACY is set, so a part that also defines USBFSD (V203 has both
// the DevFS device and the OTG FS peripheral) can build the migrated DevFS path.
// Remove this switch with the step-5 migration.
#if defined(USBFSD) || defined(__DOXYGEN__)

/**
 * @brief CH32 OTG FS 端点实现 / CH32 OTG FS endpoint implementation
 */
class CH32EndpointOtgFs : public USB::Endpoint
{
 public:
  /**
   * @brief 构造端点并登记到 map_otg_fs_，初始为 NAK / Construct the endpoint, register it
   * in map_otg_fs_ and leave it NAKing
   * @param ep_num 端点号 / Endpoint number
   * @param dir 方向 / Direction
   * @param buffer 端点缓冲；共享双向端点两个方向传同一整块 / Endpoint buffer; both
   * directions of a shared bidirectional endpoint pass the same whole block
   * @param single_direction 该端点号只有一个方向（槽占块的一半，UEPn_DMA 随请求更新）/
   * Only this direction exists for the endpoint number (slots use half the block and
   * UEPn_DMA follows the request)
   */
  CH32EndpointOtgFs(EPNumber ep_num, Direction dir, LibXR::RawData buffer,
                    bool single_direction);

  ErrorCode ConfigureHardware(const Config& cfg) override;
  void CloseHardware(bool in_isr) override;
  ErrorCode StartHardware(const HwRequest& request, bool in_isr) override;
  StopResult StopHardware(bool in_isr) override;
  size_t ReapProgress(const HwRequest& request, bool& retired) override;

  /**
   * @brief 传输完成中断入口：推进 IN 分包或结束请求 / Transfer-complete interrupt
   * entry: continues an IN split or finishes the request
   * @param size 本次硬件完成的字节数（OUT 为收到的长度）/ Bytes completed by the
   * hardware this time (the received length for OUT)
   * @note 没有活动请求时直接丢弃（迟到的完成）/ Dropped when no request is active
   * (a late completion)
   */
  void TransferComplete(size_t size);

  /// 将该方向置为 STALL / Put this direction into STALL
  ErrorCode StallHardware(bool in_isr) override;

  /// 清除 STALL：恢复为 NAK 并开启自动 toggle；EP0 不处理（SETUP 已复位）/ Clear STALL:
  /// back to NAK with auto toggle; EP0 is left alone (SETUP already reset it)
  ErrorCode ClearStallHardware(bool in_isr) override;
  size_t MaxPacketSizeLimit(Type type) const override;

  /// EP0 数据阶段的软件 DATA0/DATA1 翻转状态，true 表示下一包用 DATA1 / Software
  /// DATA0/DATA1 state of the EP0 data stage; true means the next packet uses DATA1
  bool tog_ = false;
  /// 端点为等时传输（不使用 ACK 握手与自动 toggle）/ The endpoint is isochronous (no ACK
  /// handshake or auto toggle)
  bool is_isochronous_ = false;
  /// 该端点号只有一个方向 / Only one direction exists for this endpoint number
  bool single_direction_ = false;

  /// 构造时传入的整块 DMA 缓冲 / The whole DMA buffer given at construction
  RawData dma_buffer_;
  /// 当前请求使用的槽内存 / Slot memory of the current request
  RawData transfer_dma_{nullptr, 0};

  /// 当前请求的核心标签，完成时原样回报 / Core tag of the current request, reported back
  /// on completion
  uint32_t transfer_tag_ = 0;
  /// 有请求已挂到硬件上且尚未完成 / A request is armed on the hardware and not finished
  bool transfer_active_ = false;
  /// 硬件是否为本请求报过完成（ReapProgress 据此判断能否算成功）。
  /// Whether the hardware reported a completion for this request (ReapProgress
  /// uses it to decide whether the request can count as successful).
  bool hw_completed_ = false;
  /// 当前请求的总字节数 / Total bytes of the current request
  size_t transfer_requested_ = 0;
  /// 当前请求已完成的字节数 / Bytes of the current request completed so far
  size_t transfer_accumulated_ = 0;
  /// 下一包在槽内存中的位置 / Position of the next packet in the slot memory
  uint8_t* transfer_cursor_ = nullptr;
  /// IN 分包：剩余未发出的字节数（0 表示本请求只剩最后一包）。
  /// IN splitting: bytes of the request not sent yet (0 = last packet in flight).
  size_t transfer_remaining_ = 0;
  /// 共享双向端点的块基址：硬件 OUT 用 [base, base+64)，IN 固定用 [base+64,
  /// base+128)，UEPn_DMA 必须一直是 base。IN 端点对象从构造参数拿到的是
  /// base+64（TX 列表项），故这里回退 64 得到块基址。
  /// Block base of a shared bidirectional endpoint: the hardware uses [base,
  /// base+64) for OUT and always [base+64, base+128) for IN, so UEPn_DMA must
  /// stay at base. The IN endpoint object is given base+64 (its TX list entry),
  /// so step back to the block base.
  uint8_t* block_dma_base_ = nullptr;

  // IRQ 分发（自由函数）需要经端点对象读取槽内存（EP0 setup 包）。
  // The IRQ dispatch (free functions) reads slot memory through the endpoint
  // objects (the EP0 setup packet).
  using USB::Endpoint::SlotMemory;

  /// OTG FS 端点号个数 / Number of OTG FS endpoint numbers
  static constexpr uint8_t EP_OTG_FS_MAX_SIZE = 8;
  /// 共享双向端点中每个方向占用的字节数（OUT 在块基址，IN 在 +64）/ Bytes per
  /// direction in a shared bidirectional endpoint (OUT at the block base, IN at +64)
  static constexpr size_t SHARED_BLOCK_HALF = 64U;
  /// 按 [端点号][方向] 索引的端点表，供中断分发使用 / Endpoints indexed by
  /// [number][direction], used by interrupt dispatch
  static inline CH32EndpointOtgFs* map_otg_fs_[EP_OTG_FS_MAX_SIZE][2] = {};
};

#endif  // defined(USBFSD) || defined(__DOXYGEN__)

#if defined(USBHSD) || defined(__DOXYGEN__)

/**
 * @brief CH32 OTG HS 端点实现 / CH32 OTG HS endpoint implementation
 */
class CH32EndpointOtgHs : public USB::Endpoint
{
 public:
  /**
   * @brief 构造端点、登记到 map_otg_hs_ 并设置 DMA 地址，初始为 NAK / Construct the
   * endpoint, register it in map_otg_hs_, set its DMA address and leave it NAKing
   * @param ep_num 端点号 / Endpoint number
   * @param dir 方向 / Direction
   * @param buffer 端点缓冲 / Endpoint buffer
   */
  CH32EndpointOtgHs(EPNumber ep_num, Direction dir, LibXR::RawData buffer);

  ErrorCode ConfigureHardware(const Config& cfg) override;
  void CloseHardware(bool in_isr) override;
  ErrorCode StartHardware(const HwRequest& request, bool in_isr) override;
  StopResult StopHardware(bool in_isr) override;
  size_t ReapProgress(const HwRequest& request, bool& retired) override;

  /**
   * @brief 传输完成中断入口：推进 IN 分包或结束请求 / Transfer-complete interrupt
   * entry: continues an IN split or finishes the request
   * @param size 本次硬件完成的字节数（OUT 为收到的长度）/ Bytes completed by the
   * hardware this time (the received length for OUT)
   * @note 没有活动请求时直接丢弃（迟到的完成）/ Dropped when no request is active
   * (a late completion)
   */
  void TransferComplete(size_t size);
  /// 非 EP0 BULK/INTERRUPT IN：挂起的包是否已被主机 ACK（TOG_AUTO 下 toggle 已翻转）。
  /// Non-EP0 bulk/interrupt IN: whether the host has ACKed the armed packet (the
  /// TOG_AUTO toggle moved since it was armed).
  bool InPacketAcked() const;
  /// 每次 UIF_TRANSFER 调用：完成所有已被 ACK 但 INT_ST 没报出来的 IN 包。
  /// Called on every UIF_TRANSFER: completes each IN packet that was ACKed although
  /// INT_ST did not report it.
  static void CompleteAckedInPackets();
  /// 将该方向置为 STALL / Put this direction into STALL
  ErrorCode StallHardware(bool in_isr) override;

  /// 清除 STALL：恢复为 NAK 并开启自动 toggle、复位 toggle 状态；EP0 不处理（SETUP 已
  /// 复位）/ Clear STALL: back to NAK with auto toggle and reset toggle state; EP0 is
  /// left alone (SETUP already reset it)
  ErrorCode ClearStallHardware(bool in_isr) override;
  size_t MaxPacketSizeLimit(Type type) const override;

  /// 预留的设备编号，当前代码既不赋值也不读取 / Reserved device id; the current code
  /// neither assigns nor reads it
  uint8_t dev_id_;
  /// EP0 数据阶段的软件 DATA0/DATA1 状态，true 表示下一包用 DATA1 / Software
  /// DATA0/DATA1 state of the EP0 data stage; true means the next packet uses DATA1
  bool tog0_ = false;
  /// 与 tog0_ 一同被复位的第二个 toggle 状态，当前代码只写不读 / Second toggle state
  /// reset together with tog0_; the current code only writes it
  bool tog1_ = false;

  /// 当前请求的字节数，供 IRQ 窄恢复路径判断 EP0 IN 是否有数据在途 / Byte count of the
  /// current request, read by the narrow IRQ recovery path to tell whether EP0 IN has
  /// data in flight
  size_t last_transfer_size_ = 0;
  /// 构造时传入的 DMA 缓冲 / The DMA buffer given at construction
  RawData dma_buffer_;
  /// 当前请求使用的槽内存 / Slot memory of the current request
  RawData transfer_dma_{nullptr, 0};

  /// 当前请求的核心标签，完成时原样回报 / Core tag of the current request, reported back
  /// on completion
  uint32_t transfer_tag_ = 0;
  /// 有请求已挂到硬件上且尚未完成 / A request is armed on the hardware and not finished
  bool transfer_active_ = false;
  /// 硬件是否为本请求报过完成（ReapProgress 据此判断能否算成功）。
  /// Whether the hardware reported a completion for this request (ReapProgress
  /// uses it to decide whether the request can count as successful).
  bool hw_completed_ = false;
  /// 当前请求的总字节数 / Total bytes of the current request
  size_t transfer_requested_ = 0;
  /// 当前请求已完成的字节数 / Bytes of the current request completed so far
  size_t transfer_accumulated_ = 0;
  /// 下一包在槽内存中的位置 / Position of the next packet in the slot memory
  uint8_t* transfer_cursor_ = nullptr;
  /// IN 分包：剩余未发出的字节数（0 表示本请求只剩最后一包）。
  /// IN splitting: bytes of the request not sent yet (0 = last packet in flight).
  size_t transfer_remaining_ = 0;
  /// IN：挂包时 TX_CTRL 的 toggle 位（USBHS_UEP_T_TOG_MASK）。
  /// IN: the TX_CTRL toggle bits (USBHS_UEP_T_TOG_MASK) when the packet was armed.
  uint8_t tx_tog_armed_ = 0;

  // 同 OtgFs：IRQ 分发（自由函数）经端点对象读取槽内存。
  // Same as OtgFs: the IRQ dispatch (free functions) reads slot memory through
  // the endpoint objects.
  using USB::Endpoint::SlotMemory;

  /// OTG HS 端点号个数 / Number of OTG HS endpoint numbers
  static constexpr uint8_t EP_OTG_HS_MAX_SIZE = 16;
  /// 按 [端点号][方向] 索引的端点表，供中断分发使用 / Endpoints indexed by
  /// [number][direction], used by interrupt dispatch
  static inline CH32EndpointOtgHs* map_otg_hs_[EP_OTG_HS_MAX_SIZE][2] = {};
};

#endif  // defined(USBHSD) || defined(__DOXYGEN__)

}  // namespace LibXR
