#include "esp_usb.hpp"

#if SOC_USB_OTG_SUPPORTED && defined(CONFIG_IDF_TARGET_ESP32S3) && \
    CONFIG_IDF_TARGET_ESP32S3

#include <algorithm>
#include <cstring>

#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_memory_utils.h"
#include "hal/cache_hal.h"

namespace LibXR::ESPUSBDetail
{

#if SOC_CACHE_INTERNAL_MEM_VIA_L1CACHE || SOC_PSRAM_DMA_CAPABLE
extern "C" esp_err_t esp_cache_msync(void* addr, size_t size, int flags);

constexpr int CACHE_SYNC_FLAG_UNALIGNED = (1 << 1);
constexpr int CACHE_SYNC_FLAG_DIR_C2M = (1 << 2);
constexpr int CACHE_SYNC_FLAG_DIR_M2C = (1 << 3);

bool CacheSyncDmaBuffer(const void* addr, size_t size, bool cache_to_mem,
                        bool allow_unaligned)
{
  if ((addr == nullptr) || (size == 0U))
  {
    return true;
  }

  uint32_t cache_level = 0;
  uint32_t cache_id = 0;
  const bool cache_supported = cache_hal_vaddr_to_cache_level_id(
      static_cast<uint32_t>(reinterpret_cast<uintptr_t>(addr)), size, &cache_level,
      &cache_id);
  if (!cache_supported)
  {
    return true;
  }

  int flags = cache_to_mem ? CACHE_SYNC_FLAG_DIR_C2M : CACHE_SYNC_FLAG_DIR_M2C;
  if (allow_unaligned && cache_to_mem)
  {
    flags |= CACHE_SYNC_FLAG_UNALIGNED;
  }
  return esp_cache_msync(const_cast<void*>(addr), size, flags) == ESP_OK;
}
#else
bool CacheSyncDmaBuffer(const void*, size_t, bool, bool) { return true; }
#endif

size_t AlignUp(size_t value, size_t align)
{
  if (align <= 1U)
  {
    return value;
  }
  return ((value + align - 1U) / align) * align;
}

esp_dma_mem_info_t UsbDmaMemInfo()
{
  esp_dma_mem_info_t info = {};
  info.extra_heap_caps = MALLOC_CAP_INTERNAL;
  info.dma_alignment_bytes = WORD_SIZE;
  return info;
}

bool CanUseDirectInDmaBuffer(const void* ptr, size_t size)
{
  if ((ptr == nullptr) || (size == 0U))
  {
    return size == 0U;
  }

  auto* start = static_cast<const uint8_t*>(ptr);
  auto* end = start + size - 1U;
  return esp_ptr_dma_capable(start) && esp_ptr_dma_capable(end) &&
         esp_ptr_word_aligned(ptr);
}

bool CanUseDirectOutDmaBuffer(const void* ptr, size_t size)
{
  if ((ptr == nullptr) || (size == 0U))
  {
    return false;
  }

  auto* start = static_cast<const uint8_t*>(ptr);
  auto* end = start + size - 1U;
  if (!esp_ptr_dma_capable(start) || !esp_ptr_dma_capable(end) ||
      !esp_ptr_word_aligned(ptr) ||
      !esp_dma_is_buffer_alignment_satisfied(ptr, size, UsbDmaMemInfo()))
  {
    return false;
  }

  uint32_t cache_level = 0;
  uint32_t cache_id = 0;
  const bool cache_supported = cache_hal_vaddr_to_cache_level_id(
      static_cast<uint32_t>(reinterpret_cast<uintptr_t>(ptr)), size, &cache_level,
      &cache_id);
  if (!cache_supported)
  {
    return true;
  }

  const uintptr_t addr = reinterpret_cast<uintptr_t>(ptr);
  return ((addr % USB_DMA_ALIGNMENT) == 0U) && (AlignUp(size, USB_DMA_ALIGNMENT) == size);
}

uint16_t CalcRxFifoWords(uint16_t largest_packet_size, uint8_t ep_count)
{
  return static_cast<uint16_t>(13U + 1U + 2U * (((largest_packet_size + 3U) / 4U) + 1U) +
                               2U * ep_count);
}

uint16_t CalcConfiguredRxFifoWords(uint16_t largest_packet_size, uint8_t ep_count,
                                   bool dma_enabled)
{
  uint16_t words = CalcRxFifoWords(largest_packet_size, ep_count);
  if (dma_enabled)
  {
    words = std::max<uint16_t>(words, ESP32_SX_FS_DMA_MIN_RX_FIFO_WORDS);
  }
  return words;
}

uint16_t GetHardwareFifoDepthWords()
{
  auto* dev = reinterpret_cast<usb_dwc_dev_t*>(DWC2_FS_REG_BASE);
  return static_cast<uint16_t>(dev->ghwcfg3_reg.dfifodepth);
}

uint8_t EncodeEp0Mps(uint16_t packet_size)
{
  switch (packet_size)
  {
    case 8:
      return 3U;
    case 16:
      return 2U;
    case 32:
      return 1U;
    case 64:
    default:
      return 0U;
  }
}

uint16_t ClampPacketSize(LibXR::USB::Endpoint::Type type, uint16_t requested)
{
  switch (type)
  {
    case LibXR::USB::Endpoint::Type::CONTROL:
      return static_cast<uint16_t>(std::min<uint16_t>(requested, 64U));
    case LibXR::USB::Endpoint::Type::BULK:
      return static_cast<uint16_t>(std::min<uint16_t>(requested, 64U));
    case LibXR::USB::Endpoint::Type::INTERRUPT:
      return static_cast<uint16_t>(std::min<uint16_t>(requested, 64U));
    case LibXR::USB::Endpoint::Type::ISOCHRONOUS:
      return static_cast<uint16_t>(std::min<uint16_t>(requested, 1023U));
    default:
      return requested;
  }
}

uint16_t CalcTxFifoWords(uint16_t packet_size, bool dma_enabled)
{
  uint16_t words = static_cast<uint16_t>((packet_size + 3U) / 4U);
  if (dma_enabled)
  {
    words = std::max<uint16_t>(words, ESP32_SX_FS_MIN_TX_FIFO_WORDS);
  }
  return words;
}

volatile uint32_t* GetEndpointFifo(usb_dwc_dev_t* dev, uint8_t ep_num)
{
  uintptr_t base = reinterpret_cast<uintptr_t>(dev);
  return reinterpret_cast<volatile uint32_t*>(
      base + FIFO_BASE_OFFSET + static_cast<uintptr_t>(ep_num) * FIFO_STRIDE);
}

void WriteFifoPacket(volatile uint32_t* fifo, const uint8_t* src, size_t size)
{
  for (size_t offset = 0; offset < size; offset += WORD_SIZE)
  {
    uint32_t word = 0U;
    const size_t chunk = std::min(WORD_SIZE, size - offset);
    std::memcpy(&word, src + offset, chunk);
    fifo[0] = word;
  }
}

void ReadFifoPacket(const volatile uint32_t* fifo, uint8_t* dst, size_t size)
{
  for (size_t offset = 0; offset < size; offset += WORD_SIZE)
  {
    const uint32_t word = fifo[0];
    const size_t chunk = std::min(WORD_SIZE, size - offset);
    std::memcpy(dst + offset, &word, chunk);
  }
}

uint16_t PacketCount(size_t size, uint16_t max_packet_size)
{
  if (size == 0U)
  {
    return 1U;
  }
  return static_cast<uint16_t>((size + max_packet_size - 1U) / max_packet_size);
}

bool FlushTxFifo(usb_dwc_dev_t* dev, uint8_t fifo_num)
{
  dev->grstctl_reg.txfnum = fifo_num;
  dev->grstctl_reg.txfflsh = 1;
  return SpinUntil([dev]() { return !dev->grstctl_reg.txfflsh; });
}

}  // namespace LibXR::ESPUSBDetail

#endif
