#pragma once

#include "ch32_usb.hpp"
#include "libxr_def.hpp"
#include "libxr_type.hpp"
#include "usb/core/ep.hpp"

namespace LibXR
{

#if defined(USBFSD) || defined(__DOXYGEN__)

/**
 * @brief CH32 OTG FS 端点实现 / CH32 OTG FS endpoint implementation
 */
class CH32EndpointOtgFs : public USB::Endpoint
{
 public:
  CH32EndpointOtgFs(EPNumber ep_num, Direction dir, LibXR::RawData buffer,
                    bool single_direction);

  void Configure(const Config& cfg) override;
  void Close() override;
  ErrorCode Transfer(size_t size) override;

  void TransferComplete(size_t size);
  ErrorCode Stall() override;
  ErrorCode ClearStall() override;

  void SwitchBuffer() override;

  bool tog_ = false;
  bool is_isochronous_ = false;
  bool single_direction_ = false;

  size_t last_transfer_size_ = 0;
  RawData dma_buffer_;

  static constexpr uint8_t EP_OTG_FS_MAX_SIZE = 8;
  static inline CH32EndpointOtgFs* map_otg_fs_[EP_OTG_FS_MAX_SIZE][2] = {};
};

#endif  // defined(USBFSD) || defined(__DOXYGEN__)

#if defined(RCC_APB1Periph_USB) || defined(__DOXYGEN__)

/**
 * @brief CH32 FSDEV 端点实现 / CH32 FSDEV endpoint implementation
 */
class CH32EndpointDevFs : public USB::Endpoint
{
 public:
  CH32EndpointDevFs(EPNumber ep_num, Direction dir, LibXR::RawData buffer,
                    bool is_isochronous);

  void Configure(const Config& cfg) override;
  void Close() override;
  ErrorCode Transfer(size_t size) override;

  void TransferComplete(size_t size);

  void CopyRxDataToBuffer(size_t size);

  ErrorCode Stall() override;
  ErrorCode ClearStall() override;

  void SwitchBuffer() override;

  static void ResetPMAAllocator();
  static void SetEpTxStatus(uint8_t ep, uint16_t status);
  static void SetEpRxStatus(uint8_t ep, uint16_t status);
  static void ClearEpCtrTx(uint8_t ep);
  static void ClearEpCtrRx(uint8_t ep);
  static uint16_t GetRxCount(uint8_t ep);

  bool is_isochronous_ = false;

  size_t last_transfer_size_ = 0;

  uint16_t pma_addr_ = 0;

  static constexpr uint8_t EP_DEV_FS_MAX_SIZE = 8;
  static inline CH32EndpointDevFs* map_dev_fs_[EP_DEV_FS_MAX_SIZE][2] = {};
};

#endif  // defined(RCC_APB1Periph_USB) || defined(__DOXYGEN__)

#if defined(USBHSD) || defined(__DOXYGEN__)

/**
 * @brief CH32 OTG HS 端点实现 / CH32 OTG HS endpoint implementation
 */
class CH32EndpointOtgHs : public USB::Endpoint
{
 public:
  CH32EndpointOtgHs(EPNumber ep_num, Direction dir, LibXR::RawData buffer,
                    bool double_buffer);

  void Configure(const Config& cfg) override;
  void Close() override;
  ErrorCode Transfer(size_t size) override;

  void TransferComplete(size_t size);
  ErrorCode Stall() override;
  ErrorCode ClearStall() override;

  void SwitchBuffer() override;

  uint8_t dev_id_;
  bool tog0_ = false;
  bool tog1_ = false;
  bool hw_double_buffer_ = false;

  size_t last_transfer_size_ = 0;
  RawData dma_buffer_;

  static constexpr uint8_t EP_OTG_HS_MAX_SIZE = 16;
  static inline CH32EndpointOtgHs* map_otg_hs_[EP_OTG_HS_MAX_SIZE][2] = {};
};

#endif  // defined(USBHSD) || defined(__DOXYGEN__)

}  // namespace LibXR
