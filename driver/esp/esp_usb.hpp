#pragma once

#include <cstddef>
#include <cstdint>

#include "esp_def.hpp"
#include "esp_dma_utils.h"
#include "esp_heap_caps.h"
#include "usb/core/ep.hpp"

#if (SOC_USB_OTG_SUPPORTED && defined(CONFIG_IDF_TARGET_ESP32S3) && \
     CONFIG_IDF_TARGET_ESP32S3) ||                                  \
    defined(__DOXYGEN__)

#include "soc/usb_dwc_struct.h"

namespace LibXR::ESPUSBDetail
{

inline constexpr uint32_t DWC2_FS_REG_BASE = 0x60080000UL;
inline constexpr size_t FIFO_BASE_OFFSET = 0x1000U;
inline constexpr size_t FIFO_STRIDE = 0x1000U;

inline constexpr uint8_t RX_STATUS_GLOBAL_OUT_NAK = 1U;
inline constexpr uint8_t RX_STATUS_DATA = 2U;
inline constexpr uint8_t RX_STATUS_TRANSFER_COMPLETE = 3U;
inline constexpr uint8_t RX_STATUS_SETUP_DONE = 4U;
inline constexpr uint8_t RX_STATUS_SETUP_DATA = 6U;

inline constexpr uint8_t ENUM_SPEED_FULL_30_TO_60_MHZ = 1U;
inline constexpr uint8_t ENUM_SPEED_FULL_48_MHZ = 3U;

inline constexpr size_t WORD_SIZE = sizeof(uint32_t);
inline constexpr uint8_t FLUSH_ALL_TX_FIFO = 0x10U;
inline constexpr uint32_t DMA_BURST_INCR4 = 4U;
inline constexpr uint32_t DMA_MEMORY_CAPS =
    MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA | MALLOC_CAP_8BIT;
inline constexpr uint16_t ESP32_SX_FS_DMA_MIN_RX_FIFO_WORDS = 88U;
inline constexpr uint16_t ESP32_SX_FS_MIN_TX_FIFO_WORDS = 16U;
inline constexpr uint32_t DISABLE_OUT_WAIT_GUARD = 100000U;

#if defined(CONFIG_USB_ALIGN_SIZE)
inline constexpr size_t USB_DMA_ALIGNMENT = CONFIG_USB_ALIGN_SIZE;
#elif defined(CONFIG_CACHE_L1_CACHE_LINE_SIZE)
inline constexpr size_t USB_DMA_ALIGNMENT = CONFIG_CACHE_L1_CACHE_LINE_SIZE;
#else
inline constexpr size_t USB_DMA_ALIGNMENT = 64U;
#endif

constexpr uint32_t PackTxFifoSizeReg(uint16_t start, uint16_t words)
{
  return (static_cast<uint32_t>(words) << 16U) | static_cast<uint32_t>(start);
}

bool CacheSyncDmaBuffer(const void* addr, size_t size, bool cache_to_mem,
                        bool allow_unaligned = false);
size_t AlignUp(size_t value, size_t align);
esp_dma_mem_info_t UsbDmaMemInfo();
bool CanUseDirectInDmaBuffer(const void* ptr, size_t size);
bool CanUseDirectOutDmaBuffer(const void* ptr, size_t size);
uint16_t CalcRxFifoWords(uint16_t largest_packet_size, uint8_t ep_count);
uint16_t CalcConfiguredRxFifoWords(uint16_t largest_packet_size, uint8_t ep_count,
                                   bool dma_enabled);
uint16_t GetHardwareFifoDepthWords();
uint8_t EncodeEp0Mps(uint16_t packet_size);
uint16_t ClampPacketSize(LibXR::USB::Endpoint::Type type, uint16_t requested);
uint16_t CalcTxFifoWords(uint16_t packet_size, bool dma_enabled);
volatile uint32_t* GetEndpointFifo(usb_dwc_dev_t* dev, uint8_t ep_num);
void WriteFifoPacket(volatile uint32_t* fifo, const uint8_t* src, size_t size);
void ReadFifoPacket(const volatile uint32_t* fifo, uint8_t* dst, size_t size);
uint16_t PacketCount(size_t size, uint16_t max_packet_size);
void FlushTxFifo(usb_dwc_dev_t* dev, uint8_t fifo_num);
void DisableInEndpointAndWait(usb_dwc_dev_t* dev);
void DisableInEndpointAndWait(usb_dwc_dev_t* dev, uint8_t ep_num);

template <typename EpCtl>
void StartEndpointTransfer(volatile EpCtl& reg)
{
  EpCtl ctl = {};
  ctl.val = reg.val;
  ctl.snak = 0;
  ctl.cnak = 1;
  ctl.epdis = 0;
  ctl.epena = 1;
  reg.val = ctl.val;
}

template <typename DoepCtl>
void DisableOutEndpointAndWait(volatile DoepCtl& ctl)
{
  DoepCtl disable = {};
  disable.val = ctl.val;
  disable.snak = 1;
  const bool was_enabled = disable.epena;
  if (was_enabled)
  {
    disable.epdis = 1;
  }
  ctl.val = disable.val;

  if (!was_enabled)
  {
    return;
  }

  uint32_t guard = DISABLE_OUT_WAIT_GUARD;
  while (ctl.epena && guard > 0U)
  {
    --guard;
  }
}

}  // namespace LibXR::ESPUSBDetail

#endif
