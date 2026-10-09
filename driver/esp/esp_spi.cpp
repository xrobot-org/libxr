#include "esp_spi.hpp"

#include <algorithm>
#include <array>
#include <cstdlib>
#include <cstring>

#include "driver/gpio.h"
#include "driver/spi_common.h"
#include "esp_attr.h"
#include "esp_clk_tree.h"
#include "esp_err.h"
#include "esp_memory_utils.h"
#include "esp_private/periph_ctrl.h"
#include "esp_private/spi_common_internal.h"
#include "esp_timer.h"
#if SOC_GDMA_SUPPORTED
#include "esp_private/gdma.h"
#endif
#include "esp_rom_gpio.h"
#include "hal/spi_hal.h"
#include "libxr_def.hpp"
#include "soc/spi_periph.h"
#include "timebase.hpp"

namespace LibXR
{
namespace
{

uint8_t ResolveSpiMode(SPI::ClockPolarity polarity, SPI::ClockPhase phase)
{
  if (polarity == SPI::ClockPolarity::LOW)
  {
    return (phase == SPI::ClockPhase::EDGE_1) ? 0U : 1U;
  }
  return (phase == SPI::ClockPhase::EDGE_1) ? 2U : 3U;
}

spi_dma_ctx_t* ToDmaCtx(void* ctx) { return reinterpret_cast<spi_dma_ctx_t*>(ctx); }

uint64_t GetNowUs() { return static_cast<uint64_t>(Timebase::GetMicroseconds()); }

uint64_t CalcPollingTimeoutUs(size_t size, uint32_t bus_hz)
{
  const uint32_t safe_hz = (bus_hz == 0U) ? 1U : bus_hz;
  const uint64_t bits = static_cast<uint64_t>(size) * 8ULL;
  const uint64_t wire_time_us = (bits * 1000000ULL + safe_hz - 1ULL) / safe_hz;
  // Keep a generous margin for bus contention / APB jitter while staying
  // time-based.
  return std::max<uint64_t>(100ULL, wire_time_us * 8ULL + 50ULL);
}

inline void SpiEnableClockAtomic(spi_host_device_t host_id, bool enable)
{
#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#endif
  PERIPH_RCC_ATOMIC() { spi_ll_enable_clock(host_id, enable); }
#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif
}

inline void SpiSetClockSourceAtomic(spi_dev_t* hw, spi_clock_source_t clk_source)
{
#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#endif
  PERIPH_RCC_ATOMIC() { spi_ll_set_clk_source(hw, clk_source); }
#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif
}

#if SOC_GDMA_SUPPORTED
esp_err_t DmaReset(gdma_channel_handle_t chan) { return gdma_reset(chan); }

esp_err_t DmaStart(gdma_channel_handle_t chan, const void* desc)
{
  return gdma_start(chan, reinterpret_cast<intptr_t>(desc));
}
#else
esp_err_t DmaReset(spi_dma_chan_handle_t chan)
{
  spi_dma_reset(chan);
  return ESP_OK;
}

esp_err_t DmaStart(spi_dma_chan_handle_t chan, const void* desc)
{
  spi_dma_start(chan, const_cast<void*>(desc));
  return ESP_OK;
}
#endif

#if SOC_CACHE_INTERNAL_MEM_VIA_L1CACHE || SOC_PSRAM_DMA_CAPABLE
extern "C" esp_err_t esp_cache_msync(void* addr, size_t size, int flags);

constexpr int CACHE_SYNC_FLAG_UNALIGNED = (1 << 1);
constexpr int CACHE_SYNC_FLAG_DIR_C2M = (1 << 2);
constexpr int CACHE_SYNC_FLAG_DIR_M2C = (1 << 3);

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

  int flags = cache_to_mem ? CACHE_SYNC_FLAG_DIR_C2M : CACHE_SYNC_FLAG_DIR_M2C;
  flags |= CACHE_SYNC_FLAG_UNALIGNED;

  const esp_err_t ret = esp_cache_msync(const_cast<void*>(addr), size, flags);
  // Non-cacheable regions can return ESP_ERR_INVALID_ARG; treat as no-op
  // success.
  return (ret == ESP_OK) || (ret == ESP_ERR_INVALID_ARG);
}
#endif

}  // namespace

ESP32SPI::ESP32SPI(spi_host_device_t host, int sclk_pin, int miso_pin, int mosi_pin,
                   RawData dma_rx, RawData dma_tx, SPI::Configuration config,
                   uint32_t dma_enable_min_size, bool enable_dma)
    : SPI(dma_rx, dma_tx),
      host_(host),
      sclk_pin_(sclk_pin),
      miso_pin_(miso_pin),
      mosi_pin_(mosi_pin),
      dma_enable_min_size_(dma_enable_min_size),
      dma_requested_(enable_dma),
      dma_rx_raw_(dma_rx),
      dma_tx_raw_(dma_tx),
      dbuf_rx_block_size_(dma_rx.size_ / 2U),
      dbuf_tx_block_size_(dma_tx.size_ / 2U)
{
  ASSERT(host_ != SPI1_HOST);
  ASSERT(host_ < SPI_HOST_MAX);
  ASSERT(static_cast<int>(host_) < SOC_SPI_PERIPH_NUM);
  ASSERT(dma_rx_raw_.addr_ != nullptr);
  ASSERT(dma_tx_raw_.addr_ != nullptr);
  ASSERT(dma_rx_raw_.size_ > 0U);
  ASSERT(dma_tx_raw_.size_ > 0U);

  if (InitializeHardware() != ErrorCode::OK)
  {
    REQUIRE(false);
    return;
  }

  if (ConfigurePins() != ErrorCode::OK)
  {
    REQUIRE(false);
    return;
  }

  if (InstallInterrupt() != ErrorCode::OK)
  {
    REQUIRE(false);
    return;
  }

  if (dma_requested_)
  {
    const ErrorCode dma_ans = InitDmaBackend();
    REQUIRE(dma_ans == ErrorCode::OK);
    if (dma_ans != ErrorCode::OK)
    {
      return;
    }
  }

  if (SetConfig(config) != ErrorCode::OK)
  {
    REQUIRE(false);
    return;
  }
}

bool ESP32SPI::Acquire() { return !busy_.TestAndSet(); }

void ESP32SPI::Release() { busy_.Clear(); }

ErrorCode ESP32SPI::InitializeHardware()
{
  if (initialized_)
  {
    return ErrorCode::OK;
  }

  if ((host_ <= SPI1_HOST) || (host_ >= SPI_HOST_MAX) ||
      (static_cast<int>(host_) >= SOC_SPI_PERIPH_NUM))
  {
    return ErrorCode::ARG_ERR;
  }

  const auto& signal = spi_periph_signal[host_];
  hw_ = signal.hw;
  if (hw_ == nullptr)
  {
    return ErrorCode::NOT_SUPPORT;
  }

  PERIPH_RCC_ATOMIC()
  {
    (void)__DECLARE_RCC_ATOMIC_ENV;
    spi_ll_enable_bus_clock(host_, true);
    spi_ll_reset_register(host_);
  }
  SpiEnableClockAtomic(host_, true);
  spi_ll_master_init(hw_);

  const spi_line_mode_t line_mode = {
      .cmd_lines = 1,
      .addr_lines = 1,
      .data_lines = 1,
  };
  spi_ll_master_set_line_mode(hw_, line_mode);
  spi_ll_set_half_duplex(hw_, false);
  spi_ll_set_tx_lsbfirst(hw_, false);
  spi_ll_set_rx_lsbfirst(hw_, false);
  spi_ll_set_mosi_delay(hw_, 0, 0);
  spi_ll_enable_mosi(hw_, 1);
  spi_ll_enable_miso(hw_, 1);
  spi_ll_set_dummy(hw_, 0);
  spi_ll_set_command_bitlen(hw_, 0);
  spi_ll_set_addr_bitlen(hw_, 0);
  spi_ll_set_command(hw_, 0, 0, false);
  spi_ll_set_address(hw_, 0, 0, false);
  hw_->user.usr_command = 0;
  hw_->user.usr_addr = 0;

  SpiSetClockSourceAtomic(hw_, SPI_CLK_SRC_DEFAULT);
  if (ResolveClockSource(source_clock_hz_) != ErrorCode::OK)
  {
    return ErrorCode::INIT_ERR;
  }

  spi_ll_disable_int(hw_);
  spi_ll_clear_int_stat(hw_);
  initialized_ = true;
  return ErrorCode::OK;
}

ErrorCode ESP32SPI::ConfigurePins()
{
  if (!initialized_ || (hw_ == nullptr))
  {
    return ErrorCode::STATE_ERR;
  }

  const auto& signal = spi_periph_signal[host_];

  if (sclk_pin_ >= 0)
  {
    if (!GPIO_IS_VALID_OUTPUT_GPIO(sclk_pin_))
    {
      return ErrorCode::ARG_ERR;
    }
    esp_rom_gpio_pad_select_gpio(static_cast<uint32_t>(sclk_pin_));
    esp_rom_gpio_connect_out_signal(sclk_pin_, signal.spiclk_out, false, false);
    gpio_input_enable(static_cast<gpio_num_t>(sclk_pin_));
    esp_rom_gpio_connect_in_signal(sclk_pin_, signal.spiclk_in, false);
  }

  if (mosi_pin_ >= 0)
  {
    if (!GPIO_IS_VALID_OUTPUT_GPIO(mosi_pin_))
    {
      return ErrorCode::ARG_ERR;
    }
    esp_rom_gpio_pad_select_gpio(static_cast<uint32_t>(mosi_pin_));
    esp_rom_gpio_connect_out_signal(mosi_pin_, signal.spid_out, false, false);
    gpio_input_enable(static_cast<gpio_num_t>(mosi_pin_));
    esp_rom_gpio_connect_in_signal(mosi_pin_, signal.spid_in, false);
  }

  if (miso_pin_ >= 0)
  {
    if (!GPIO_IS_VALID_GPIO(miso_pin_))
    {
      return ErrorCode::ARG_ERR;
    }
    gpio_input_enable(static_cast<gpio_num_t>(miso_pin_));
    esp_rom_gpio_connect_in_signal(miso_pin_, signal.spiq_in, false);
  }

  return ErrorCode::OK;
}

ErrorCode ESP32SPI::ResolveClockSource(uint32_t& source_hz)
{
  source_hz = 0;
  const esp_err_t err =
      esp_clk_tree_src_get_freq_hz(static_cast<soc_module_clk_t>(SPI_CLK_SRC_DEFAULT),
                                   ESP_CLK_TREE_SRC_FREQ_PRECISION_CACHED, &source_hz);
  if ((err != ESP_OK) || (source_hz == 0))
  {
    return ErrorCode::INIT_ERR;
  }
  return ErrorCode::OK;
}

ErrorCode ESP32SPI::InstallInterrupt()
{
  if (intr_installed_)
  {
    return ErrorCode::OK;
  }

  const int irq_source = static_cast<int>(spi_periph_signal[host_].irq);
  if (esp_intr_alloc(irq_source, ESP_INTR_FLAG_IRAM, &ESP32SPI::SpiIsrEntry, this,
                     &intr_handle_) != ESP_OK)
  {
    intr_handle_ = nullptr;
    return ErrorCode::INIT_ERR;
  }

  intr_installed_ = true;
  return ErrorCode::OK;
}

ErrorCode ESP32SPI::InitDmaBackend()
{
  if (dma_enabled_)
  {
    return ErrorCode::OK;
  }

  const size_t cfg_max_size = std::max(dma_rx_raw_.size_, dma_tx_raw_.size_);
  int actual_max_size = 0;
  if (spicommon_dma_chan_alloc(host_, SPI_DMA_CH_AUTO, 0) != ESP_OK)
  {
    return ErrorCode::INIT_ERR;
  }

  if (spicommon_dma_desc_alloc(host_, static_cast<int>(cfg_max_size), &actual_max_size) !=
      ESP_OK)
  {
    (void)spicommon_dma_chan_free(host_);
    return ErrorCode::INIT_ERR;
  }

  dma_ctx_ = spi_bus_get_dma_ctx(host_);
  dma_enabled_ = true;
  dma_max_transfer_bytes_ =
      std::min<size_t>({static_cast<size_t>(actual_max_size), dma_rx_raw_.size_,
                        dma_tx_raw_.size_, MAX_DMA_TRANSFER_BYTES});

  if (dma_max_transfer_bytes_ == 0U)
  {
    return ErrorCode::INIT_ERR;
  }

  return ErrorCode::OK;
}

void IRAM_ATTR ESP32SPI::SpiIsrEntry(void* arg)
{
  auto* spi = static_cast<ESP32SPI*>(arg);
  if (spi != nullptr)
  {
    spi->HandleInterrupt();
  }
}

void IRAM_ATTR ESP32SPI::HandleInterrupt()
{
  if ((hw_ == nullptr) || !initialized_)
  {
    return;
  }

  if (!async_running_)
  {
    spi_ll_clear_int_stat(hw_);
    return;
  }

  if (!spi_ll_usr_is_done(hw_))
  {
    return;
  }

  FinishAsync(true, ErrorCode::OK);
}

void IRAM_ATTR ESP32SPI::FinishAsync(bool in_isr, ErrorCode ec)
{
  if (!async_running_)
  {
    return;
  }

#if CONFIG_IDF_TARGET_ESP32
  // Keep ESP32 SPI DMA workaround state in sync with transfer lifecycle.
  if (dma_enabled_)
  {
    spi_dma_ctx_t* ctx = ToDmaCtx(dma_ctx_);
    if (ctx != nullptr)
    {
      spicommon_dmaworkaround_idle(ctx->tx_dma_chan.chan_id);
    }
  }
#endif

  spi_ll_disable_int(hw_);
  spi_ll_clear_int_stat(hw_);

  RawData rx = {nullptr, 0U};
  const RawData read_back = read_back_;
  const bool mem_read = mem_read_;
  if ((ec == ErrorCode::OK) && async_dma_rx_enabled_)
  {
    rx = GetRxBuffer();
#if SOC_CACHE_INTERNAL_MEM_VIA_L1CACHE || SOC_PSRAM_DMA_CAPABLE
    if (!CacheSyncDmaBuffer(rx.addr_, async_dma_size_, false))
    {
      ec = ErrorCode::FAILED;
    }
#endif
  }

  if ((ec == ErrorCode::OK) && (read_back.size_ > 0U))
  {
    if (rx.addr_ == nullptr)
    {
      rx = GetRxBuffer();
    }
    uint8_t* src = static_cast<uint8_t*>(rx.addr_);
    if (mem_read)
    {
      ASSERT(rx.size_ >= (read_back.size_ + 1U));
      src += 1;
    }
    else
    {
      ASSERT(rx.size_ >= read_back.size_);
    }
    Memory::FastCopy(read_back.addr_, src, read_back.size_);
  }

  if (ec == ErrorCode::OK)
  {
    SwitchBufferLocal();
  }

  async_running_ = false;
  async_dma_size_ = 0U;
  async_dma_rx_enabled_ = false;
  mem_read_ = false;
  read_back_ = {nullptr, 0};
  Release();
  if (rw_op_.type == OperationRW::OperationType::BLOCK)
  {
    (void)block_wait_.TryPost(in_isr, ec);
  }
  else
  {
    rw_op_.UpdateStatus(in_isr, ec);
  }
}

ErrorCode ESP32SPI::SetConfig(SPI::Configuration config)
{
  if (!initialized_ || (hw_ == nullptr))
  {
    return ErrorCode::STATE_ERR;
  }
  if (busy_.IsSet())
  {
    return ErrorCode::BUSY;
  }

  if (config.prescaler == Prescaler::UNKNOWN)
  {
    return ErrorCode::ARG_ERR;
  }
  if (config.double_buffer &&
      ((dbuf_rx_block_size_ == 0U) || (dbuf_tx_block_size_ == 0U)))
  {
    return ErrorCode::ARG_ERR;
  }

  const uint32_t div = PrescalerToDiv(config.prescaler);
  if ((div == 0) || (source_clock_hz_ == 0))
  {
    return ErrorCode::ARG_ERR;
  }

  const uint32_t target_hz = source_clock_hz_ / div;
  if (target_hz == 0)
  {
    return ErrorCode::ARG_ERR;
  }

  const uint8_t mode = ResolveSpiMode(config.clock_polarity, config.clock_phase);
  spi_ll_master_set_mode(hw_, mode);

  const int applied_hz = spi_ll_master_set_clock(hw_, static_cast<int>(source_clock_hz_),
                                                 static_cast<int>(target_hz), 128);
  if (applied_hz <= 0)
  {
    return ErrorCode::INIT_ERR;
  }

  spi_ll_apply_config(hw_);
  GetConfig() = config;
  dbuf_enabled_ = config.double_buffer;
  dbuf_active_block_ = 0U;

  return ErrorCode::OK;
}

bool ESP32SPI::UseLocalDoubleBuffer() const
{
  return dbuf_enabled_ && (dbuf_rx_block_size_ > 0U) && (dbuf_tx_block_size_ > 0U);
}

RawData ESP32SPI::GetRxBuffer()
{
  if (UseLocalDoubleBuffer())
  {
    auto* base = static_cast<uint8_t*>(dma_rx_raw_.addr_);
    return {base + static_cast<size_t>(dbuf_active_block_) * dbuf_rx_block_size_,
            dbuf_rx_block_size_};
  }
  return dma_rx_raw_;
}

RawData ESP32SPI::GetTxBuffer()
{
  if (UseLocalDoubleBuffer())
  {
    auto* base = static_cast<uint8_t*>(dma_tx_raw_.addr_);
    return {base + static_cast<size_t>(dbuf_active_block_) * dbuf_tx_block_size_,
            dbuf_tx_block_size_};
  }
  return dma_tx_raw_;
}

void ESP32SPI::SwitchBufferLocal()
{
  if (UseLocalDoubleBuffer())
  {
    dbuf_active_block_ ^= 1U;
  }
}

bool ESP32SPI::CanUseDma(size_t size) const
{
  return dma_requested_ && dma_enabled_ && (dma_ctx_ != nullptr) &&
         (size > dma_enable_min_size_) && (size <= dma_max_transfer_bytes_);
}

ErrorCode ESP32SPI::EnsureReadyAndAcquire()
{
  if (!initialized_)
  {
    return ErrorCode::INIT_ERR;
  }
  if (!Acquire())
  {
    return ErrorCode::BUSY;
  }
  return ErrorCode::OK;
}

ErrorCode ESP32SPI::FinalizeSyncResult(OperationRW& op, bool in_isr, ErrorCode ec)
{
  if (op.type != OperationRW::OperationType::BLOCK)
  {
    op.UpdateStatus(in_isr, ec);
  }
  return ec;
}

ErrorCode ESP32SPI::CompleteZeroSize(OperationRW& op, bool in_isr)
{
  return FinalizeSyncResult(op, in_isr, ErrorCode::OK);
}

ErrorCode ESP32SPI::ReturnAsyncStartResult(ErrorCode ec, bool started)
{
  if (!started)
  {
    Release();
  }
  return ec;
}

void ESP32SPI::ConfigureTransferRegisters(size_t size)
{
  static constexpr spi_line_mode_t LINE_MODE = {
      .cmd_lines = 1,
      .addr_lines = 1,
      .data_lines = 1,
  };
  const size_t bitlen = size * 8U;

  spi_ll_master_set_line_mode(hw_, LINE_MODE);
  spi_ll_set_mosi_bitlen(hw_, bitlen);
  spi_ll_set_miso_bitlen(hw_, bitlen);
  spi_ll_set_command_bitlen(hw_, 0);
  spi_ll_set_addr_bitlen(hw_, 0);
  spi_ll_set_command(hw_, 0, 0, false);
  spi_ll_set_address(hw_, 0, 0, false);
  hw_->user.usr_command = 0;
  hw_->user.usr_addr = 0;
}

ErrorCode ESP32SPI::StartAsyncTransfer(const uint8_t* tx, uint8_t* rx, size_t size,
                                       bool enable_rx, RawData read_back, bool mem_read,
                                       OperationRW& op, bool& started)
{
  started = false;

  if (!CanUseDma(size))
  {
    return ErrorCode::NOT_SUPPORT;
  }
  if ((tx == nullptr) || (size == 0U))
  {
    return ErrorCode::ARG_ERR;
  }

  spi_dma_ctx_t* ctx = ToDmaCtx(dma_ctx_);
  if (ctx == nullptr)
  {
    return ErrorCode::INIT_ERR;
  }

  const bool rx_enabled = enable_rx && (rx != nullptr);
  ConfigureTransferRegisters(size);

#if SOC_CACHE_INTERNAL_MEM_VIA_L1CACHE || SOC_PSRAM_DMA_CAPABLE
  if (!CacheSyncDmaBuffer(tx, size, true))
  {
    return ErrorCode::FAILED;
  }
#endif

  if (enable_rx && (rx != nullptr))
  {
    spicommon_dma_desc_setup_link(ctx->dmadesc_rx, rx, static_cast<int>(size), true);
    if (DmaReset(ctx->rx_dma_chan) != ESP_OK)
    {
      return ErrorCode::INIT_ERR;
    }
    spi_hal_hw_prepare_rx(hw_);
    if (DmaStart(ctx->rx_dma_chan, ctx->dmadesc_rx) != ESP_OK)
    {
      return ErrorCode::INIT_ERR;
    }
  }
#if CONFIG_IDF_TARGET_ESP32
  else
  {
    // Keep ESP32 full-duplex TX-only DMA behavior aligned with IDF workaround.
    spi_ll_dma_rx_enable(hw_, true);
    (void)DmaStart(ctx->rx_dma_chan, nullptr);
  }
#endif

  spicommon_dma_desc_setup_link(ctx->dmadesc_tx, tx, static_cast<int>(size), false);
  if (DmaReset(ctx->tx_dma_chan) != ESP_OK)
  {
    return ErrorCode::INIT_ERR;
  }
  spi_hal_hw_prepare_tx(hw_);
  if (DmaStart(ctx->tx_dma_chan, ctx->dmadesc_tx) != ESP_OK)
  {
    return ErrorCode::INIT_ERR;
  }

#if CONFIG_IDF_TARGET_ESP32
  spicommon_dmaworkaround_transfer_active(ctx->tx_dma_chan.chan_id);
#endif

  rw_op_ = op;
  read_back_ = read_back;
  mem_read_ = mem_read;
  async_dma_size_ = size;
  async_dma_rx_enabled_ = rx_enabled;
  if (op.type == OperationRW::OperationType::BLOCK)
  {
    block_wait_.Start(*op.data.sem_info.sem);
  }
  started = true;

  // On ESP32 classic, enable data lines only after DMA descriptors/channels are
  // ready.
  spi_ll_enable_mosi(hw_, 1);
  spi_ll_enable_miso(hw_, rx_enabled ? 1 : 0);
  spi_ll_clear_int_stat(hw_);
  spi_ll_enable_int(hw_);
  async_running_ = true;
  spi_ll_apply_config(hw_);
  spi_ll_user_start(hw_);

  op.MarkAsRunning();
  if (op.type == OperationRW::OperationType::BLOCK)
  {
    return block_wait_.Wait(op.data.sem_info.timeout);
  }
  return ErrorCode::OK;
}

ErrorCode ESP32SPI::ExecuteChunk(const uint8_t* tx, uint8_t* rx, size_t size,
                                 bool enable_rx)
{
  if ((size == 0U) || (size > MAX_POLLING_TRANSFER_BYTES))
  {
    return ErrorCode::SIZE_ERR;
  }

  static constexpr std::array<uint8_t, MAX_POLLING_TRANSFER_BYTES> ZERO = {};
  const uint8_t* tx_data = (tx != nullptr) ? tx : ZERO.data();

  ConfigureTransferRegisters(size);
  spi_ll_enable_mosi(hw_, 1);
  spi_ll_enable_miso(hw_, enable_rx ? 1 : 0);
  spi_ll_write_buffer(hw_, tx_data, size * 8U);
  spi_ll_clear_int_stat(hw_);
  spi_ll_apply_config(hw_);
  spi_ll_user_start(hw_);

  const uint64_t timeout_us = CalcPollingTimeoutUs(size, GetBusSpeed());
  const uint64_t start_us = GetNowUs();
  while (!spi_ll_usr_is_done(hw_))
  {
    if ((GetNowUs() - start_us) > timeout_us)
    {
      return ErrorCode::TIMEOUT;
    }
  }

  if (enable_rx && (rx != nullptr))
  {
    spi_ll_read_buffer(hw_, rx, size * 8U);
  }
  spi_ll_clear_int_stat(hw_);
  return ErrorCode::OK;
}

ErrorCode ESP32SPI::ExecuteTransfer(const uint8_t* tx, uint8_t* rx, size_t size,
                                    bool enable_rx)
{
  size_t offset = 0U;
  while (offset < size)
  {
    const size_t remain = size - offset;
    const size_t chunk = std::min(remain, MAX_POLLING_TRANSFER_BYTES);
    const uint8_t* tx_chunk = (tx != nullptr) ? (tx + offset) : nullptr;
    uint8_t* rx_chunk = (enable_rx && (rx != nullptr)) ? (rx + offset) : nullptr;

    const ErrorCode ec = ExecuteChunk(tx_chunk, rx_chunk, chunk, enable_rx);
    if (ec != ErrorCode::OK)
    {
      return ec;
    }
    offset += chunk;
  }

  return ErrorCode::OK;
}

ErrorCode ESP32SPI::ReadAndWrite(RawData read_data, ConstRawData write_data,
                                 OperationRW& op, bool in_isr)
{
  const size_t need = std::max(read_data.size_, write_data.size_);
  if (need == 0U)
  {
    return CompleteZeroSize(op, in_isr);
  }

  const ErrorCode lock_ec = EnsureReadyAndAcquire();
  if (lock_ec != ErrorCode::OK)
  {
    return lock_ec;
  }

  RawData rx = GetRxBuffer();
  RawData tx = GetTxBuffer();
  ASSERT(rx.size_ >= need);
  ASSERT(tx.size_ >= need);

  auto* tx_ptr = static_cast<uint8_t*>(tx.addr_);
  if (write_data.size_ > 0U)
  {
    Memory::FastCopy(tx_ptr, write_data.addr_, write_data.size_);
  }
  if (need > write_data.size_)
  {
    Memory::FastSet(tx_ptr + write_data.size_, 0x00, need - write_data.size_);
  }

  if (CanUseDma(need))
  {
    bool started = false;
    const ErrorCode ec = StartAsyncTransfer(tx_ptr, static_cast<uint8_t*>(rx.addr_), need,
                                            true, read_data, false, op, started);
    return ReturnAsyncStartResult(ec, started);
  }

  const ErrorCode ec =
      ExecuteTransfer(tx_ptr, static_cast<uint8_t*>(rx.addr_), need, true);
  if (ec == ErrorCode::OK)
  {
    if (read_data.size_ > 0U)
    {
      Memory::FastCopy(read_data.addr_, rx.addr_, read_data.size_);
    }
    SwitchBufferLocal();
  }

  Release();
  return FinalizeSyncResult(op, in_isr, ec);
}

ErrorCode ESP32SPI::MemRead(uint16_t reg, RawData read_data, OperationRW& op, bool in_isr)
{
  if (read_data.size_ == 0U)
  {
    return CompleteZeroSize(op, in_isr);
  }

  const ErrorCode lock_ec = EnsureReadyAndAcquire();
  if (lock_ec != ErrorCode::OK)
  {
    return lock_ec;
  }

  RawData rx = GetRxBuffer();
  RawData tx = GetTxBuffer();
  const size_t total = read_data.size_ + 1U;

  ASSERT(rx.size_ >= total);
  ASSERT(tx.size_ >= total);

  auto* tx_ptr = static_cast<uint8_t*>(tx.addr_);
  tx_ptr[0] = static_cast<uint8_t>(reg | 0x80U);
  Memory::FastSet(tx_ptr + 1, 0x00, read_data.size_);

  if (CanUseDma(total))
  {
    bool started = false;
    const ErrorCode ec = StartAsyncTransfer(tx_ptr, static_cast<uint8_t*>(rx.addr_),
                                            total, true, read_data, true, op, started);
    return ReturnAsyncStartResult(ec, started);
  }

  const ErrorCode ec =
      ExecuteTransfer(tx_ptr, static_cast<uint8_t*>(rx.addr_), total, true);
  if (ec == ErrorCode::OK)
  {
    auto* rx_ptr = static_cast<uint8_t*>(rx.addr_);
    Memory::FastCopy(read_data.addr_, rx_ptr + 1, read_data.size_);
    SwitchBufferLocal();
  }

  Release();
  return FinalizeSyncResult(op, in_isr, ec);
}

ErrorCode ESP32SPI::MemWrite(uint16_t reg, ConstRawData write_data, OperationRW& op,
                             bool in_isr)
{
  if (write_data.size_ == 0U)
  {
    return CompleteZeroSize(op, in_isr);
  }

  const ErrorCode lock_ec = EnsureReadyAndAcquire();
  if (lock_ec != ErrorCode::OK)
  {
    return lock_ec;
  }

  RawData tx = GetTxBuffer();
  const size_t total = write_data.size_ + 1U;
  ASSERT(tx.size_ >= total);

  auto* tx_ptr = static_cast<uint8_t*>(tx.addr_);
  tx_ptr[0] = static_cast<uint8_t>(reg & 0x7FU);
  Memory::FastCopy(tx_ptr + 1, write_data.addr_, write_data.size_);

  if (CanUseDma(total))
  {
    bool started = false;
    const ErrorCode ec = StartAsyncTransfer(tx_ptr, nullptr, total, false, {nullptr, 0},
                                            false, op, started);
    return ReturnAsyncStartResult(ec, started);
  }

  const ErrorCode ec = ExecuteTransfer(tx_ptr, nullptr, total, false);
  if (ec == ErrorCode::OK)
  {
    SwitchBufferLocal();
  }

  Release();
  return FinalizeSyncResult(op, in_isr, ec);
}

ErrorCode ESP32SPI::Transfer(size_t size, OperationRW& op, bool in_isr)
{
  if (size == 0U)
  {
    return CompleteZeroSize(op, in_isr);
  }

  const ErrorCode lock_ec = EnsureReadyAndAcquire();
  if (lock_ec != ErrorCode::OK)
  {
    return lock_ec;
  }

  RawData rx = GetRxBuffer();
  RawData tx = GetTxBuffer();
  ASSERT(rx.size_ >= size);
  ASSERT(tx.size_ >= size);

  if (CanUseDma(size))
  {
    bool started = false;
    const ErrorCode ec = StartAsyncTransfer(static_cast<const uint8_t*>(tx.addr_),
                                            static_cast<uint8_t*>(rx.addr_), size, true,
                                            {nullptr, 0}, false, op, started);
    return ReturnAsyncStartResult(ec, started);
  }

  const ErrorCode ec = ExecuteTransfer(static_cast<const uint8_t*>(tx.addr_),
                                       static_cast<uint8_t*>(rx.addr_), size, true);
  if (ec == ErrorCode::OK)
  {
    SwitchBufferLocal();
  }

  Release();
  return FinalizeSyncResult(op, in_isr, ec);
}

uint32_t ESP32SPI::GetMaxBusSpeed() const { return source_clock_hz_; }

SPI::Prescaler ESP32SPI::GetMaxPrescaler() const { return Prescaler::DIV_65536; }

}  // namespace LibXR
