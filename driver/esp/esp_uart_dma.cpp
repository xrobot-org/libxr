#include "esp_uart.hpp"

#if SOC_GDMA_SUPPORTED && SOC_UHCI_SUPPORTED

#include <algorithm>
#include <array>

#include "esp_attr.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_memory_utils.h"
#include "esp_private/periph_ctrl.h"
#include "hal/uhci_ll.h"
#include "soc/ext_mem_defs.h"

namespace
{
// RX 使用循环 DMA 描述符环，语义接近 STM/CH 的循环 RX DMA：持续接收，
// 软件侧维护消费索引。
// RX uses a circular DMA descriptor ring, similar to STM/CH circular RX DMA
// behavior (continuous receive + software consumer index).
constexpr uint32_t DMA_RX_NODE_COUNT = 8;

// 当前 ESP GDMA link item 单节点最多只能描述 4095 字节。
// Current ESP GDMA link items cannot describe more than 4095 bytes in one node.
constexpr size_t DMA_MAX_BUFFER_SIZE_PER_LINK_ITEM = 4095U;

// 为就地修改描述符长度而保留的 GDMA link descriptor 最小本地视图。
// Minimal local view of the GDMA link descriptor layout used for in-place patching.
struct GdmaLinkItem
{
  struct
  {
    uint32_t size : 12;
    uint32_t length : 12;
    uint32_t reserved24 : 4;
    uint32_t err_eof : 1;
    uint32_t reserved29 : 1;
    uint32_t suc_eof : 1;
    uint32_t owner : 1;
  } dw0;
  void* buffer;
  GdmaLinkItem* next;
};

constexpr uint32_t GDMA_OWNER_CPU = 0U;
constexpr uint32_t GDMA_OWNER_DMA = 1U;

// 用于 DMA storage 和 node 大小对齐计算的辅助函数。
// Helper used for DMA storage and node-size alignment calculations.
size_t AlignUp(size_t value, size_t align)
{
  if (align <= 1)
  {
    return value;
  }
  return ((value + align - 1) / align) * align;
}

// 当目标 SoC 提供 non-cache alias 时，把 ESP-IDF 返回的 cache 地址转换成
// GDMA 描述符使用的 non-cache 地址。
// Convert the cached address returned by ESP-IDF into the non-cache alias used
// by the GDMA descriptors when the target SoC exposes one.
uintptr_t CacheAddrToNonCache(uintptr_t addr)
{
#if SOC_NON_CACHEABLE_OFFSET
  return addr + SOC_NON_CACHEABLE_OFFSET;
#else
  return addr;
#endif
}

// 从 GDMA list head 地址恢复首个 link item。
// Recover the first link item from a GDMA list head address.
GdmaLinkItem* LinkItemFromHeadAddr(uintptr_t head_addr)
{
  return reinterpret_cast<GdmaLinkItem*>(CacheAddrToNonCache(head_addr));
}

#if SOC_CACHE_INTERNAL_MEM_VIA_L1CACHE || SOC_PSRAM_DMA_CAPABLE
extern "C" esp_err_t esp_cache_msync(void* addr, size_t size, int flags);
#if SOC_CACHE_INTERNAL_MEM_VIA_L1CACHE
extern "C" esp_err_t esp_cache_get_alignment(uint32_t heap_caps, size_t* out_alignment);
#endif

constexpr int CACHE_SYNC_FLAG_UNALIGNED = (1 << 1);
constexpr int CACHE_SYNC_FLAG_DIR_C2M = (1 << 2);
constexpr int CACHE_SYNC_FLAG_DIR_M2C = (1 << 3);

// 当当前内存区域可缓存时，同步一个 DMA 窗口。
// Synchronize one DMA window when the active memory region is cacheable.
bool CacheSyncDmaBuffer(const void* addr, size_t size, bool cache_to_mem)
{
  if ((addr == nullptr) || (size == 0U))
  {
    return true;
  }

#if SOC_PSRAM_DMA_CAPABLE && !SOC_CACHE_INTERNAL_MEM_VIA_L1CACHE
  if (!esp_ptr_external_ram(addr))
  {
    return true;
  }
#endif

  // M2C 不接受 UNALIGNED；RX 调用方传入完整且按 cache line 对齐的 descriptor。
  // M2C does not accept UNALIGNED; RX callers pass a complete cache-line-aligned
  // descriptor.
  int flags = cache_to_mem ? (CACHE_SYNC_FLAG_DIR_C2M | CACHE_SYNC_FLAG_UNALIGNED)
                           : CACHE_SYNC_FLAG_DIR_M2C;

  const esp_err_t ret = esp_cache_msync(const_cast<void*>(addr), size, flags);
  // Non-cacheable regions can return ESP_ERR_INVALID_ARG; treat as no-op success.
  return (ret == ESP_OK) || (ret == ESP_ERR_INVALID_ARG);
}
#endif
}  // namespace

namespace LibXR
{

// TX EOF 表示当前暂存的 active payload 已经完整离开 DMA 引擎。
// TX EOF means the current staged active payload has fully left the DMA engine.
bool IRAM_ATTR ESP32UART::DmaTxEofCallback(gdma_channel_handle_t, gdma_event_data_t*,
                                           void* user_data)
{
  auto* uart = static_cast<ESP32UART*>(user_data);
  if (uart != nullptr)
  {
    uart->service_.Invoke(ESP32UART::EVENT_TX_DONE, true,
                          [uart](uint32_t events, bool in_isr)
                          { uart->ServiceEvents(events, in_isr); });
  }
  return false;
}

// TX 描述符错误会被上报为后端 TX 失败。
// TX descriptor error is surfaced as a backend TX failure.
bool IRAM_ATTR ESP32UART::DmaTxDescrErrCallback(gdma_channel_handle_t, gdma_event_data_t*,
                                                void* user_data)
{
  auto* uart = static_cast<ESP32UART*>(user_data);
  if (uart != nullptr)
  {
    uart->service_.Invoke(ESP32UART::EVENT_TX_ERROR, true,
                          [uart](uint32_t events, bool in_isr)
                          { uart->ServiceEvents(events, in_isr); });
  }
  return false;
}

// RX 完成回调只负责把事件转发到 UART 对象状态机。
// RX done callback only forwards the event into the UART object state machine.
bool IRAM_ATTR ESP32UART::DmaRxDoneCallback(gdma_channel_handle_t,
                                            gdma_event_data_t* event_data,
                                            void* user_data)
{
  auto* uart = static_cast<ESP32UART*>(user_data);
  if (uart != nullptr)
  {
    const uint32_t event = (event_data != nullptr && event_data->flags.abnormal_eof)
                               ? ESP32UART::EVENT_RX_ERROR
                               : ESP32UART::EVENT_RX_WORK;
    uart->service_.Invoke(event, true, [uart](uint32_t events, bool in_isr)
                          { uart->ServiceEvents(events, in_isr); });
  }
  return false;
}

// RX 描述符错误要求完整恢复 RX 环。
// RX descriptor error requests a full RX ring recovery.
bool IRAM_ATTR ESP32UART::DmaRxDescrErrCallback(gdma_channel_handle_t, gdma_event_data_t*,
                                                void* user_data)
{
  auto* uart = static_cast<ESP32UART*>(user_data);
  if (uart != nullptr)
  {
    uart->service_.Invoke(ESP32UART::EVENT_RX_ERROR, true,
                          [uart](uint32_t events, bool in_isr)
                          { uart->ServiceEvents(events, in_isr); });
  }
  return false;
}

// DMA 后端初始化做三件事：
// 1. 把 UHCI 绑定到选定 UART。
// 2. 为双缓冲两半各准备一条 TX 描述符链。
// 3. 准备一条循环 RX 描述符环。
// DMA backend bring-up does three things:
// 1. Bind UHCI to the selected UART.
// 2. Prepare two TX descriptor lists, one per double-buffer half.
// 3. Prepare one circular RX descriptor ring.
ErrorCode ESP32UART::InitDmaBackend()
{
  if (dma_backend_enabled_)
  {
    return ErrorCode::OK;
  }

  periph_module_enable(PERIPH_UHCI0_MODULE);
  periph_module_reset(PERIPH_UHCI0_MODULE);

  uhci_hal_init(&uhci_hal_, 0);
  uhci_ll_attach_uart_port(uhci_hal_.dev, uart_num_);

  uhci_seper_chr_t sep_chr = {};
  sep_chr.sub_chr_en = 0;
  uhci_ll_set_seper_chr(uhci_hal_.dev, &sep_chr);
  uhci_ll_rx_set_eof_mode(uhci_hal_.dev, UHCI_RX_IDLE_EOF);

  gdma_channel_alloc_config_t tx_cfg = {
      .sibling_chan = nullptr,
      .direction = GDMA_CHANNEL_DIRECTION_TX,
      .flags = {},
  };
  if (gdma_new_ahb_channel(&tx_cfg, &tx_dma_channel_) != ESP_OK)
  {
    return ErrorCode::INIT_ERR;
  }

  if (gdma_connect(tx_dma_channel_, GDMA_MAKE_TRIGGER(GDMA_TRIG_PERIPH_UHCI, 0)) !=
      ESP_OK)
  {
    return ErrorCode::INIT_ERR;
  }

  gdma_transfer_config_t transfer_cfg = {
      .max_data_burst_size = 0,
      .access_ext_mem = true,
  };
  if (gdma_config_transfer(tx_dma_channel_, &transfer_cfg) != ESP_OK)
  {
    return ErrorCode::INIT_ERR;
  }

  size_t tx_int_alignment = 1;
  size_t tx_ext_alignment = 1;
  if (gdma_get_alignment_constraints(tx_dma_channel_, &tx_int_alignment,
                                     &tx_ext_alignment) != ESP_OK)
  {
    return ErrorCode::INIT_ERR;
  }
  const size_t tx_dma_alignment =
      std::max<size_t>(1, std::max(tx_int_alignment, tx_ext_alignment));

  gdma_strategy_config_t tx_strategy = {
      .owner_check = true,
      .auto_update_desc = true,
      .eof_till_data_popped = true,
  };
  if (gdma_apply_strategy(tx_dma_channel_, &tx_strategy) != ESP_OK)
  {
    return ErrorCode::INIT_ERR;
  }

  gdma_link_list_config_t tx_link_cfg = {
      .num_items = 1,
      .item_alignment = 4,
      .flags = {},
  };
  gdma_link_list_handle_t tx_dma_links[2] = {nullptr, nullptr};

  for (int i = 0; i < 2; ++i)
  {
    if (gdma_new_link_list(&tx_link_cfg, &tx_dma_links[i]) != ESP_OK)
    {
      return ErrorCode::INIT_ERR;
    }

    gdma_buffer_mount_config_t tx_mount = {
        .buffer = tx_dma_buffer_.Buffer(i),
        .buffer_alignment = tx_dma_alignment,
        .length = 1,
        .flags =
            {
                .mark_eof = 1,
                .mark_final = 1,
                .bypass_buffer_align_check = 0,
            },
    };

    if (gdma_link_mount_buffers(tx_dma_links[i], 0, &tx_mount, 1, nullptr) != ESP_OK)
    {
      return ErrorCode::INIT_ERR;
    }

    tx_dma_head_addr_[i] = gdma_link_get_head_addr(tx_dma_links[i]);
    if (tx_dma_head_addr_[i] == 0U)
    {
      return ErrorCode::INIT_ERR;
    }
  }

  gdma_tx_event_callbacks_t tx_callbacks = {
      .on_trans_eof = DmaTxEofCallback,
      .on_descr_err = DmaTxDescrErrCallback,
  };
  if (gdma_register_tx_event_callbacks(tx_dma_channel_, &tx_callbacks, this) != ESP_OK)
  {
    return ErrorCode::INIT_ERR;
  }

  gdma_channel_alloc_config_t rx_cfg = {
      .sibling_chan = nullptr,
      .direction = GDMA_CHANNEL_DIRECTION_RX,
      .flags = {},
  };
  if (gdma_new_ahb_channel(&rx_cfg, &rx_dma_channel_) != ESP_OK)
  {
    return ErrorCode::INIT_ERR;
  }

  if (gdma_connect(rx_dma_channel_, GDMA_MAKE_TRIGGER(GDMA_TRIG_PERIPH_UHCI, 0)) !=
      ESP_OK)
  {
    return ErrorCode::INIT_ERR;
  }

  if (gdma_config_transfer(rx_dma_channel_, &transfer_cfg) != ESP_OK)
  {
    return ErrorCode::INIT_ERR;
  }

  gdma_strategy_config_t rx_strategy = {
      .owner_check = true,
      .auto_update_desc = false,
      .eof_till_data_popped = false,
  };
  if (gdma_apply_strategy(rx_dma_channel_, &rx_strategy) != ESP_OK)
  {
    return ErrorCode::INIT_ERR;
  }

  size_t rx_int_alignment = 1;
  size_t rx_ext_alignment = 1;
  if (gdma_get_alignment_constraints(rx_dma_channel_, &rx_int_alignment,
                                     &rx_ext_alignment) != ESP_OK)
  {
    return ErrorCode::INIT_ERR;
  }
  const size_t rx_dma_alignment =
      std::max<size_t>(1, std::max(rx_int_alignment, rx_ext_alignment));

  size_t rx_cache_alignment = 1U;
#if SOC_CACHE_INTERNAL_MEM_VIA_L1CACHE
  if ((esp_cache_get_alignment(MALLOC_CAP_INTERNAL, &rx_cache_alignment) != ESP_OK) ||
      (rx_cache_alignment == 0U))
  {
    return ErrorCode::INIT_ERR;
  }
#endif

  gdma_link_list_config_t rx_link_cfg = {
      .num_items = DMA_RX_NODE_COUNT,
      .item_alignment = 4,
      .flags = {},
  };
  if (gdma_new_link_list(&rx_link_cfg, &rx_dma_link_) != ESP_OK)
  {
    return ErrorCode::INIT_ERR;
  }

  // 保持单个环窗口适度偏大，以降低高波特率下的 ISR 压力。
  // Keep one ring window reasonably large to lower ISR pressure at high baud.
  const size_t rx_chunk_target = std::min<size_t>(
      std::max<size_t>(32, rx_isr_buffer_size_ / DMA_RX_NODE_COUNT), 512);
  const size_t rx_storage_alignment =
      std::max<size_t>(4, std::max(rx_dma_alignment, rx_cache_alignment));
  rx_dma_chunk_size_ =
      std::max<size_t>(AlignUp(rx_chunk_target, rx_storage_alignment), 32);
  const size_t rx_storage_bytes =
      AlignUp(rx_dma_chunk_size_ * DMA_RX_NODE_COUNT, rx_storage_alignment);

  rx_dma_storage_ = static_cast<uint8_t*>(
      heap_caps_aligned_alloc(rx_storage_alignment, rx_storage_bytes,
                              MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA | MALLOC_CAP_8BIT));
  if (rx_dma_storage_ == nullptr)
  {
    return ErrorCode::NO_MEM;
  }

  std::array<gdma_buffer_mount_config_t, DMA_RX_NODE_COUNT> rx_mount = {};
  for (uint32_t i = 0; i < DMA_RX_NODE_COUNT; ++i)
  {
    rx_mount[i] = gdma_buffer_mount_config_t{
        .buffer = rx_dma_storage_ + (static_cast<size_t>(i) * rx_dma_chunk_size_),
        .buffer_alignment = rx_dma_alignment,
        .length = rx_dma_chunk_size_,
        .flags =
            {
                .mark_eof = 0,
                .mark_final = 0,
                .bypass_buffer_align_check = 0,
            },
    };
  }

  if (gdma_link_mount_buffers(rx_dma_link_, 0, rx_mount.data(), DMA_RX_NODE_COUNT,
                              nullptr) != ESP_OK)
  {
    return ErrorCode::INIT_ERR;
  }

  gdma_rx_event_callbacks_t rx_callbacks = {
      .on_recv_eof = nullptr,
      .on_descr_err = DmaRxDescrErrCallback,
      .on_recv_done = DmaRxDoneCallback,
  };
  if (gdma_register_rx_event_callbacks(rx_dma_channel_, &rx_callbacks, this) != ESP_OK)
  {
    return ErrorCode::INIT_ERR;
  }

  if (gdma_reset(rx_dma_channel_) != ESP_OK)
  {
    return ErrorCode::INIT_ERR;
  }

  rx_dma_node_index_ = 0;
  dma_backend_enabled_ = true;
  if (gdma_start(rx_dma_channel_, gdma_link_get_head_addr(rx_dma_link_)) != ESP_OK)
  {
    return ErrorCode::INIT_ERR;
  }
  return ErrorCode::OK;
}

// TX DMA 启动时只修改预挂载描述符链的动态字段，避免每次请求都重建描述符。
// TX DMA start only patches the dynamic fields of the pre-mounted descriptor
// list, so the hot path avoids rebuilding descriptors for every request.
bool IRAM_ATTR ESP32UART::StartDmaTx()
{
  if ((tx_dma_channel_ == nullptr) || !tx_active_valid_)
  {
    return false;
  }

  uint8_t* const active_buffer = tx_dma_buffer_.ActiveBuffer();
  const size_t active_len = tx_active_length_;
  if ((active_buffer == nullptr) || (active_len == 0) ||
      (active_len > DMA_MAX_BUFFER_SIZE_PER_LINK_ITEM))
  {
    return false;
  }

  const int link_index = tx_dma_buffer_.ActiveBlock();
  if ((link_index != 0) && (link_index != 1))
  {
    return false;
  }

  if (tx_dma_head_addr_[link_index] == 0U)
  {
    return false;
  }

  auto* desc = LinkItemFromHeadAddr(tx_dma_head_addr_[link_index]);
  if (desc == nullptr)
  {
    return false;
  }

  // 描述符链保持预挂载，只就地更新本次传输长度。
  // Keep descriptor list pre-mounted and only patch the dynamic transfer length in-place.
  desc->buffer = active_buffer;
  desc->dw0.size = static_cast<uint32_t>(active_len);
  desc->dw0.length = static_cast<uint32_t>(active_len);
  desc->dw0.err_eof = 0U;
  desc->dw0.suc_eof = 1U;
  desc->dw0.owner = GDMA_OWNER_DMA;
  desc->next = nullptr;
  std::atomic_thread_fence(std::memory_order_release);

#if SOC_CACHE_INTERNAL_MEM_VIA_L1CACHE || SOC_PSRAM_DMA_CAPABLE
  if (!CacheSyncDmaBuffer(active_buffer, active_len, true))
  {
    return false;
  }
#endif

  return gdma_start(tx_dma_channel_, tx_dma_head_addr_[link_index]) == ESP_OK;
}

// RX DMA 完成以描述符 owner 位为准，而不是依赖短命回调指针。已完成描述符按
// 最老顺序消费，按 ReadQueue 实时容量复制，然后在同一串行 owner 中交还 DMA。
// RX DMA completion is derived from descriptor ownership, not from a transient
// callback pointer. Completed descriptors are consumed oldest-first, copied up
// to live ReadQueue capacity, and returned to DMA in the same serialized owner.
void IRAM_ATTR ESP32UART::HandleDmaRxDone(gdma_event_data_t* event_data, bool in_isr)
{
  UNUSED(event_data);
  if ((rx_dma_storage_ == nullptr) || (rx_dma_chunk_size_ == 0))
  {
    return;
  }

  auto* first = LinkItemFromHeadAddr(gdma_link_get_head_addr(rx_dma_link_));
  DEV_ASSERT_FROM_CALLBACK(first != nullptr, in_isr);
  auto* item = first;
  for (uint32_t i = 0U; i < rx_dma_node_index_; ++i)
  {
    DEV_ASSERT_FROM_CALLBACK(item != nullptr, in_isr);
    item = item->next;
  }

  auto queue = _read_port.GetReadQueue(in_isr);
  for (uint32_t processed = 0U; processed < DMA_RX_NODE_COUNT; ++processed)
  {
    DEV_ASSERT_FROM_CALLBACK(item != nullptr, in_isr);
    if (item->dw0.owner != GDMA_OWNER_CPU)
    {
      break;
    }

    std::atomic_thread_fence(std::memory_order_acquire);
    const size_t received = std::min<size_t>(item->dw0.length, rx_dma_chunk_size_);
#if SOC_CACHE_INTERNAL_MEM_VIA_L1CACHE || SOC_PSRAM_DMA_CAPABLE
    [[maybe_unused]] const auto cache_sync_dma_buffer_result =
        CacheSyncDmaBuffer(item->buffer, rx_dma_chunk_size_, false);
    REQUIRE_FROM_CALLBACK(cache_sync_dma_buffer_result, in_isr);
#endif
    if (received != 0U && queue.EmptySize() != 0U)
    {
      const size_t fitting = std::min(received, queue.EmptySize());
      [[maybe_unused]] const auto push_batch_result =
          queue.PushBatch(static_cast<const uint8_t*>(item->buffer), fitting);
      DEV_ASSERT_FROM_CALLBACK(push_batch_result == ErrorCode::OK, in_isr);
    }

    std::atomic_thread_fence(std::memory_order_release);
    item->dw0.owner = GDMA_OWNER_DMA;
    rx_dma_node_index_ = (rx_dma_node_index_ + 1U) % DMA_RX_NODE_COUNT;
    item = item->next;
  }
  queue.Publish();
}

void IRAM_ATTR ESP32UART::StopDmaRx(bool in_isr)
{
  DEV_ASSERT_FROM_CALLBACK(rx_dma_channel_ != nullptr, in_isr);
  [[maybe_unused]] const auto gdma_stop_result = gdma_stop(rx_dma_channel_);
  REQUIRE_FROM_CALLBACK(gdma_stop_result == ESP_OK, in_isr);
  [[maybe_unused]] const auto gdma_reset_result = gdma_reset(rx_dma_channel_);
  REQUIRE_FROM_CALLBACK(gdma_reset_result == ESP_OK, in_isr);
}

void IRAM_ATTR ESP32UART::ResetDmaRxDescriptors(bool in_isr)
{
  DEV_ASSERT_FROM_CALLBACK(rx_dma_link_ != nullptr, in_isr);
  auto* item = LinkItemFromHeadAddr(gdma_link_get_head_addr(rx_dma_link_));
  DEV_ASSERT_FROM_CALLBACK(item != nullptr, in_isr);
  for (uint32_t i = 0U; i < DMA_RX_NODE_COUNT; ++i)
  {
    DEV_ASSERT_FROM_CALLBACK(item != nullptr, in_isr);
    item->dw0.size = static_cast<uint32_t>(rx_dma_chunk_size_);
    item->dw0.length = static_cast<uint32_t>(rx_dma_chunk_size_);
    item->dw0.err_eof = 0U;
    item->dw0.suc_eof = 0U;
    std::atomic_thread_fence(std::memory_order_release);
    item->dw0.owner = GDMA_OWNER_DMA;
    item = item->next;
  }

  rx_dma_node_index_ = 0;
}

void IRAM_ATTR ESP32UART::StartDmaRx(bool in_isr)
{
  DEV_ASSERT_FROM_CALLBACK(rx_dma_channel_ != nullptr, in_isr);
  DEV_ASSERT_FROM_CALLBACK(rx_dma_link_ != nullptr, in_isr);
  [[maybe_unused]] const auto gdma_start_result =
      gdma_start(rx_dma_channel_, gdma_link_get_head_addr(rx_dma_link_));
  REQUIRE_FROM_CALLBACK(gdma_start_result == ESP_OK, in_isr);
}

// RX DMA 恢复会从节点零重新启动整个环。
// RX DMA recovery restarts the circular ring from node zero.
void IRAM_ATTR ESP32UART::HandleDmaRxError(bool in_isr)
{
  if ((rx_dma_channel_ == nullptr) || (rx_dma_link_ == nullptr))
  {
    return;
  }

  StopDmaRx(in_isr);
  ResetDmaRxDescriptors(in_isr);
  StartDmaRx(in_isr);
}

// TX DMA 恢复会中止当前硬件传输，再交给公共 TX 完成路径清理软件状态。
// TX DMA recovery aborts the current hardware transfer and lets the common TX
// completion path clean up the software state.
void IRAM_ATTR ESP32UART::HandleDmaTxError(bool in_isr)
{
  if (tx_dma_channel_ != nullptr)
  {
    gdma_stop(tx_dma_channel_);
    gdma_reset(tx_dma_channel_);
  }
  OnTxTransferDone(in_isr, ErrorCode::FAILED);
}

}  // namespace LibXR

#endif
