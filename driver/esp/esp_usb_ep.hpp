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

  /// 由 IN/OUT 中断调用，推进停止时序。两个标志取自中断入口处读出并清掉的那一份端点
  /// 中断状态 / Called from the IN/OUT interrupts to advance the stop sequence. The two
  /// flags come from the endpoint interrupt status that the handler read and cleared on
  /// entry
  /// @param nak_effective INEPNE：IN 端点的 NAK 已生效 / INEPNE: the NAK of the IN
  ///        endpoint is in effect
  /// @param disabled EPDISD：端点已禁用 / EPDISD: the endpoint is disabled
  void AdvanceStop(bool in_isr, bool nak_effective, bool disabled);
  /// 有停止在等硬件确认（设备据此打开 SOF，核心靠帧数判停止超时）/ A stop is waiting for
  /// the hardware (the device keeps SOF on for it; the core counts frames to time it out)
  bool StopPending() const { return stop_phase_ != StopPhase::NONE; }

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
  void HandleInInterrupt(bool in_isr);
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
  size_t transfer_actual_size_ = 0U;
  size_t transfer_queued_size_ = 0U;
  size_t dma_rx_status_bytes_ = 0U;
  bool dma_rx_status_seen_ = false;
  Ep0OutPhase ep0_out_phase_ = Ep0OutPhase::SETUP;
};

}  // namespace LibXR

#endif
