#pragma once

#include "ch32_usb.hpp"
#include "libxr_def.hpp"
#include "libxr_type.hpp"
#include "usb/core/ep.hpp"

namespace LibXR
{

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
  CH32EndpointOtgFs(EPNumber ep_num, Direction dir, LibXR::RawData buffer,
                    bool single_direction);

  ErrorCode ConfigureHardware(const Config& cfg) override;
  void CloseHardware(bool in_isr) override;
  ErrorCode StartHardware(const HwRequest& request, bool in_isr) override;
  StopResult StopHardware(bool in_isr) override;
  size_t ReapProgress(const HwRequest& request, bool& retired) override;

  void TransferComplete(size_t size);
  ErrorCode StallHardware(bool in_isr) override;
  ErrorCode ClearStallHardware(bool in_isr) override;
  size_t MaxPacketSizeLimit(Type type) const override;

  bool tog_ = false;
  bool is_isochronous_ = false;
  bool single_direction_ = false;

  RawData dma_buffer_;
  RawData transfer_dma_{nullptr, 0};

  uint32_t transfer_tag_ = 0;
  bool transfer_active_ = false;
  // 硬件是否为本请求报过完成（ReapProgress 据此判断能否算成功）。
  // Whether the hardware reported a completion for this request (ReapProgress
  // uses it to decide whether the request can count as successful).
  bool hw_completed_ = false;
  size_t transfer_requested_ = 0;
  size_t transfer_accumulated_ = 0;
  uint8_t* transfer_cursor_ = nullptr;
  // IN 分包：剩余未发出的字节数（0 表示本请求只剩最后一包）。
  // IN splitting: bytes of the request not sent yet (0 = last packet in flight).
  size_t transfer_remaining_ = 0;
  // 共享双向端点的块基址：硬件 OUT 用 [base, base+64)，IN 固定用 [base+64,
  // base+128)，UEPn_DMA 必须一直是 base。IN 端点对象从构造参数拿到的是
  // base+64（TX 列表项），故这里回退 64 得到块基址。
  // Block base of a shared bidirectional endpoint: the hardware uses [base,
  // base+64) for OUT and always [base+64, base+128) for IN, so UEPn_DMA must
  // stay at base. The IN endpoint object is given base+64 (its TX list entry),
  // so step back to the block base.
  uint8_t* block_dma_base_ = nullptr;

  // IRQ 分发（自由函数）需要经端点对象读取槽内存（EP0 setup 包）。
  // The IRQ dispatch (free functions) reads slot memory through the endpoint
  // objects (the EP0 setup packet).
  using USB::Endpoint::SlotMemory;

  static constexpr uint8_t EP_OTG_FS_MAX_SIZE = 8;
  static constexpr size_t SHARED_BLOCK_HALF = 64U;
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
  CH32EndpointOtgHs(EPNumber ep_num, Direction dir, LibXR::RawData buffer);

  ErrorCode ConfigureHardware(const Config& cfg) override;
  void CloseHardware(bool in_isr) override;
  ErrorCode StartHardware(const HwRequest& request, bool in_isr) override;
  StopResult StopHardware(bool in_isr) override;
  size_t ReapProgress(const HwRequest& request, bool& retired) override;

  void TransferComplete(size_t size);
  /// 一个好包（OUT 的 TOG_OK、IN 的 ACK）已收发：若另一方向挂包时控制器丢了这次自动推进，
  /// 由软件补推一次。
  /// A good packet (OUT with TOG_OK, IN ACKed) was transferred: if the controller lost
  /// its automatic advance while the other direction was being armed, advance it once in
  /// software.
  void RepairLostToggle();
  /// 非 EP0 BULK/INTERRUPT IN：挂起的包是否已被主机 ACK（TOG_AUTO 下 toggle 已翻转）。
  /// Non-EP0 bulk/interrupt IN: whether the host has ACKed the armed packet (the
  /// TOG_AUTO toggle moved since it was armed).
  bool InPacketAcked() const;
  /// 每次 UIF_TRANSFER 调用：完成所有已被 ACK 但 INT_ST 没报出来的 IN 包。
  /// Called on every UIF_TRANSFER: completes each IN packet that was ACKed although
  /// INT_ST did not report it.
  static void CompleteAckedInPackets();
  ErrorCode StallHardware(bool in_isr) override;
  ErrorCode ClearStallHardware(bool in_isr) override;
  size_t MaxPacketSizeLimit(Type type) const override;

  uint8_t dev_id_;
  bool tog0_ = false;
  bool tog1_ = false;

  size_t last_transfer_size_ = 0;
  RawData dma_buffer_;
  RawData transfer_dma_{nullptr, 0};

  uint32_t transfer_tag_ = 0;
  bool transfer_active_ = false;
  // 硬件是否为本请求报过完成（ReapProgress 据此判断能否算成功）。
  // Whether the hardware reported a completion for this request (ReapProgress
  // uses it to decide whether the request can count as successful).
  bool hw_completed_ = false;
  size_t transfer_requested_ = 0;
  size_t transfer_accumulated_ = 0;
  uint8_t* transfer_cursor_ = nullptr;
  // IN 分包：剩余未发出的字节数（0 表示本请求只剩最后一包）。
  // IN splitting: bytes of the request not sent yet (0 = last packet in flight).
  size_t transfer_remaining_ = 0;
  // IN：挂包时 TX_CTRL 的 toggle 位（USBHS_UEP_T_TOG_MASK）。
  // IN: the TX_CTRL toggle bits (USBHS_UEP_T_TOG_MASK) when the packet was armed.
  uint8_t tx_tog_armed_ = 0;
  // 另一方向挂包期间完成了一个事务时，本方向在那之前的 toggle 位（| TOG_WINDOW），否则 0。
  // The toggle bits of this direction before the other direction's arm, during which a
  // transaction completed (| TOG_WINDOW); 0 otherwise.
  uint8_t tog_window_ = 0;
  static constexpr uint8_t TOG_WINDOW = 0x80;

  // 同 OtgFs：IRQ 分发（自由函数）经端点对象读取槽内存。
  // Same as OtgFs: the IRQ dispatch (free functions) reads slot memory through
  // the endpoint objects.
  using USB::Endpoint::SlotMemory;

  static constexpr uint8_t EP_OTG_HS_MAX_SIZE = 16;
  static inline CH32EndpointOtgHs* map_otg_hs_[EP_OTG_HS_MAX_SIZE][2] = {};
};

#endif  // defined(USBHSD) || defined(__DOXYGEN__)

}  // namespace LibXR
