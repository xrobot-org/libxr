#pragma once

#include <cstddef>
#include <cstdint>

#include "driver/gpio.h"
#include "esp_def.hpp"
#include "esp_intr_alloc.h"
#include "flag.hpp"
#include "hal/spi_ll.h"
#include "hal/spi_types.h"
#include "soc/soc_caps.h"
#include "spi.hpp"

namespace LibXR
{

/**
 * @class ESP32SPI
 * @brief ESP32 SPI 主机驱动实现，支持 DMA 与轮询传输
 *        ESP32 SPI master driver supporting DMA and polling transfers.
 */
class ESP32SPI : public SPI
{
 public:
  /**
   * @brief 构造 ESP32SPI 主机驱动实例 / Construct an ESP32SPI master driver instance.
   * @param host ESP-IDF SPI 主机设备 / ESP-IDF SPI host device.
   * @param sclk_pin SCLK 引脚号 / SCLK pin number.
   * @param miso_pin MISO 引脚号 / MISO pin number.
   * @param mosi_pin MOSI 引脚号 / MOSI pin number.
   * @param dma_rx RX DMA 缓冲区 / RX DMA buffer.
   * @param dma_tx TX DMA 缓冲区 / TX DMA buffer.
   * @param config 初始 SPI 总线配置 / Initial SPI bus configuration.
   * @param dma_enable_min_size 启用 DMA 的最小传输字节数 / Minimum transfer size for DMA.
   * @param enable_dma 是否允许使用 DMA / Whether DMA mode is enabled.
   */
  ESP32SPI(spi_host_device_t host, int sclk_pin, int miso_pin, int mosi_pin,
           RawData dma_rx, RawData dma_tx,
           SPI::Configuration config =
               {
                   SPI::ClockPolarity::LOW,
                   SPI::ClockPhase::EDGE_1,
                   SPI::Prescaler::DIV_8,
                   false,
               },
           uint32_t dma_enable_min_size = 3U, bool enable_dma = true);

  ErrorCode ReadAndWrite(RawData read_data, ConstRawData write_data, OperationRW& op,
                         bool in_isr = false) override;

  ErrorCode SetConfig(SPI::Configuration config) override;

  ErrorCode MemRead(uint16_t reg, RawData read_data, OperationRW& op,
                    bool in_isr = false) override;

  ErrorCode MemWrite(uint16_t reg, ConstRawData write_data, OperationRW& op,
                     bool in_isr = false) override;

  ErrorCode Transfer(size_t size, OperationRW& op, bool in_isr = false) override;

  uint32_t GetMaxBusSpeed() const override;

  Prescaler GetMaxPrescaler() const override;

  /**
   * @brief 获取 RX DMA 缓冲区视图 / Get the RX DMA buffer view.
   * @return RX DMA 缓冲区 / RX DMA buffer.
   */
  RawData GetRxBuffer();

  /**
   * @brief 获取 TX DMA 缓冲区视图 / Get the TX DMA buffer view.
   * @return TX DMA 缓冲区 / TX DMA buffer.
   */
  RawData GetTxBuffer();

 private:
  static constexpr size_t MAX_POLLING_TRANSFER_BYTES = SOC_SPI_MAXIMUM_BUFFER_SIZE;
  static constexpr size_t MAX_DMA_TRANSFER_BYTES = SPI_LL_DMA_MAX_BIT_LEN / 8U;

  bool Acquire();

  void Release();

  ErrorCode InitializeHardware();

  ErrorCode ConfigurePins();

  ErrorCode ResolveClockSource(uint32_t& source_hz);

  ErrorCode InstallInterrupt();

  ErrorCode InitDmaBackend();

  static void SpiIsrEntry(void* arg);

  void HandleInterrupt();

  void FinishAsync(bool in_isr, ErrorCode ec);

  bool CanUseDma(size_t size) const;

  ErrorCode EnsureReadyAndAcquire();

  static ErrorCode FinalizeSyncResult(OperationRW& op, bool in_isr, ErrorCode ec);

  static ErrorCode CompleteZeroSize(OperationRW& op, bool in_isr);

  ErrorCode ReturnAsyncStartResult(ErrorCode ec, bool started);

  ErrorCode StartAsyncTransfer(const uint8_t* tx, uint8_t* rx, size_t size,
                               bool enable_rx, RawData read_back, bool mem_read,
                               OperationRW& op, bool& started);

  void ConfigureTransferRegisters(size_t size);

  ErrorCode ExecuteChunk(const uint8_t* tx, uint8_t* rx, size_t size, bool enable_rx);

  ErrorCode ExecuteTransfer(const uint8_t* tx, uint8_t* rx, size_t size, bool enable_rx);

  bool UseLocalDoubleBuffer() const;

  void SwitchBufferLocal();

  spi_host_device_t host_;
  spi_dev_t* hw_ = nullptr;
  int sclk_pin_;
  int miso_pin_;
  int mosi_pin_;
  uint32_t source_clock_hz_ = 0;
  Flag::Atomic busy_{};
  bool initialized_ = false;
  uint32_t dma_enable_min_size_ = 3U;
  bool dma_requested_ = true;
  RawData dma_rx_raw_ = {nullptr, 0};
  RawData dma_tx_raw_ = {nullptr, 0};
  size_t dbuf_rx_block_size_ = 0;
  size_t dbuf_tx_block_size_ = 0;
  uint8_t dbuf_active_block_ = 0;
  AsyncBlockWait block_wait_{};
  bool dbuf_enabled_ = false;
  intr_handle_t intr_handle_ = nullptr;
  bool intr_installed_ = false;
  bool dma_enabled_ = false;
  void* dma_ctx_ = nullptr;
  size_t dma_max_transfer_bytes_ = 0;
  bool async_running_ = false;
  size_t async_dma_size_ = 0;
  bool async_dma_rx_enabled_ = false;
  bool mem_read_ = false;
  RawData read_back_ = {nullptr, 0};
  OperationRW rw_op_;
};

}  // namespace LibXR
