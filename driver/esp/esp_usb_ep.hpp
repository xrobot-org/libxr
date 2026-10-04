#pragma once

#include "usb/core/ep.hpp"

#if (SOC_USB_OTG_SUPPORTED && defined(CONFIG_IDF_TARGET_ESP32S3) && \
     CONFIG_IDF_TARGET_ESP32S3) ||                                  \
    defined(__DOXYGEN__)

namespace LibXR
{

class ESP32USBDevice;

/**
 * @brief ESP32-S3 USB endpoint implementation
 */
class ESP32USBEndpoint : public USB::Endpoint
{
 public:
  ESP32USBEndpoint(ESP32USBDevice& device, EPNumber number, Direction direction,
                   RawData buffer);

  friend class ESP32USBDevice;

  void Configure(const Config& cfg) override;
  void Close() override;
  ErrorCode Stall() override;
  ErrorCode ClearStall() override;
  ErrorCode Transfer(size_t size) override;

  size_t MaxTransferSize() const override;

 private:
  enum class Ep0OutPhase : uint8_t
  {
    SETUP = 0U,
    DATA = 1U,
    STATUS = 2U,
  };

  void ResetHardwareState();
  void HandleInInterrupt(bool in_isr);
  void FinishPendingEp0InStatus(bool in_isr);
  void HandleOutInterrupt(bool in_isr);
  void HandleRxData(size_t size);
  void ObserveDmaRxStatus(uint8_t pktsts, size_t size);
  void ResetTransferState();
  bool EnsureDmaShadow(size_t size);
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
  uint8_t* dma_shadow_buffer_ = nullptr;
  size_t dma_shadow_size_ = 0U;
  uint8_t* transfer_buffer_ = nullptr;
  uint8_t* transfer_hw_buffer_ = nullptr;
  size_t transfer_request_size_ = 0U;
  size_t transfer_actual_size_ = 0U;
  size_t transfer_queued_size_ = 0U;
  bool transfer_uses_shadow_ = false;
  size_t transfer_direct_sync_size_ = 0U;
  size_t dma_rx_status_bytes_ = 0U;
  bool dma_rx_status_seen_ = false;
  Ep0OutPhase ep0_out_phase_ = Ep0OutPhase::SETUP;
};

}  // namespace LibXR

#endif
