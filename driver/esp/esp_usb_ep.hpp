#pragma once

#include "esp_def.hpp"
#include "soc/soc_caps.h"
#include "usb/core/ep.hpp"

#if (SOC_USB_OTG_SUPPORTED && defined(CONFIG_IDF_TARGET_ESP32S3) && \
     CONFIG_IDF_TARGET_ESP32S3) ||                                  \
    defined(__DOXYGEN__)

namespace LibXR
{

class ESP32USBDevice;

/**
 * @brief ESP32-S3 USB 端点实现 / ESP32-S3 USB endpoint implementation
 */
class ESP32USBEndpoint : public USB::Endpoint
{
 public:
  /// @param device 所属 USB 设备 / Owning USB device
  /// @param number 端点号 / Endpoint number
  /// @param direction 端点方向 / Endpoint direction
  /// @param buffer 该端点的数据缓冲区 / Data buffer of this endpoint
  ESP32USBEndpoint(ESP32USBDevice& device, EPNumber number, Direction direction,
                   RawData buffer);

  friend class ESP32USBDevice;

  ErrorCode ConfigureHardware(const Config& cfg) override;
  void CloseHardware(bool in_isr) override;
  /// 置位该端点的 STALL / Set STALL on this endpoint
  ErrorCode StallHardware(bool in_isr) override;
  /// 清除该端点的 STALL / Clear STALL on this endpoint
  ErrorCode ClearStallHardware(bool in_isr) override;
  ErrorCode StartHardware(const HwRequest& request, bool in_isr) override;
  StopResult StopHardware(bool in_isr) override;
  size_t ReapProgress(const HwRequest& request, bool& retired) override;

  /// 停止时序的阶段（由中断推进，不忙等）/ Stop sequence phase (advanced by
  /// interrupts, never busy-waiting)
  enum class StopPhase : uint8_t
  {
    NONE = 0,
    SNAK_WAIT,
    DIS_WAIT,
  };

  /// 由 IN/OUT 中断调用，推进停止时序 / Called from the IN/OUT interrupts to advance the
  /// stop sequence
  /// @param in_isr 是否在中断上下文 / Whether the call is in interrupt context
  /// @param status 中断入口处读出并清掉的那一份端点中断状态（IN 为 DIEPINT，OUT 为
  ///        DOEPINT）；不在端点自己的中断里调用时为 0（全局 OUT NAK 生效时）/ The
  ///        endpoint interrupt status that the handler read and cleared on entry (DIEPINT
  ///        for IN, DOEPINT for OUT); 0 when called outside the endpoint's own interrupt
  ///        (when the global OUT NAK takes effect)
  /// @return 在途请求在这里结束时为 true，这一份状态里的其余标志都属于它 / True when
  ///         the request in flight ended here; the other flags of that status belong to
  ///         it
  bool AdvanceStop(bool in_isr, uint32_t status);

  /// 控制器复位升级路径：收掉软件状态并确认停止（不再碰寄存器）/ Controller-reset
  /// escalation: drop the software state and confirm the stop (no register access)
  void ForceStop(bool in_isr);

  size_t MaxPacketSizeLimit(Type type) const override;

 private:
  enum class Ep0OutPhase : uint8_t
  {
    SETUP = 0U,
    DATA = 1U,
    STATUS = 2U,
  };

  void ResetHardwareState();
  /// 总线复位里硬件已被禁用：确认还在等待的停止 / The hardware is disabled by a bus
  /// reset: confirm a stop that is still waiting
  void ConfirmStopAfterReset(bool in_isr);
  /// 丢掉一个已由停止同步收尾的请求还挂着的传输完成标志 / Drop the pending
  /// transfer-complete flag of a request that a stop has just ended synchronously
  void DropCompletionFlag();
  void HandleInInterrupt(bool in_isr);
  /// 等时 IN 错过了它的帧：禁用端点，请求以 FAILED 结束；只处理为刚结束的那一帧（奇偶位
  /// 等于 ended_odd）安排的端点 / An isochronous IN transfer missed its frame: disable
  /// the endpoint and end the request with FAILED; only an endpoint armed for the frame
  /// that ended (its even/odd bit equals ended_odd) is handled
  void OnIsoIncomplete(bool ended_odd);
  void FinishPendingEp0InStatus(bool in_isr);
  void HandleOutInterrupt(bool in_isr);
  void HandleRxData(size_t size);
  void ObserveDmaRxStatus(uint8_t pktsts, size_t size);
  void ResetTransferState();
  bool PrepareTransferBuffer(size_t size);
  bool FinishOutTransfer(size_t actual_size);
  size_t GetRemainingTransferSize() const;
  size_t GetCompletedTransferSize() const;
  void ActivateHardwareEndpoint();
  void ProgramTransfer(size_t size);
  void WriteMoreTxData();

  ESP32USBDevice& device_;
  bool fifo_allocated_ = false;
  uint16_t fifo_words_ = 0U;
  uint8_t* transfer_buffer_ = nullptr;
  uint8_t* transfer_hw_buffer_ = nullptr;
  size_t transfer_request_size_ = 0U;
  uint32_t transfer_tag_ = 0U;
  bool transfer_active_ = false;
  StopPhase stop_phase_ = StopPhase::NONE;
  /// IN：停止时请求已在硬件里发完 / IN: the request had been sent completely in the
  /// hardware when the stop came
  bool stopped_sent_all_ = false;
  /// 正在跑的禁用序列是为错过帧的等时 IN 请求发起的：结束时报 FAILED，而不是确认一次
  /// 停止 / The running disable sequence was started for an isochronous IN request that
  /// missed its frame: it ends the request with FAILED instead of confirming a stop
  bool iso_abort_ = false;
  size_t transfer_actual_size_ = 0U;
  size_t transfer_queued_size_ = 0U;
  size_t dma_rx_status_bytes_ = 0U;
  bool dma_rx_status_seen_ = false;
  Ep0OutPhase ep0_out_phase_ = Ep0OutPhase::SETUP;
};

}  // namespace LibXR

#endif
