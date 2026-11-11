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
 * @brief ESP32 SPI 主机驱动 / ESP32 SPI master driver
 */
class ESP32SPI : public SPI
{
 public:
  /**
   * @brief 构造并初始化 SPI 主机 / Construct and initialize the SPI master
   * @param host SPI 主机，不能是 SPI1_HOST / SPI host, must not be SPI1_HOST
   * @param sclk_pin SCLK 引脚 / SCLK pin
   * @param miso_pin MISO 引脚 / MISO pin
   * @param mosi_pin MOSI 引脚 / MOSI pin
   * @param dma_rx 接收缓冲区，不能为空；同时用作 DMA 缓冲；启用 double_buffer
   * 时分为两半轮流使用 / RX buffer and DMA buffer, must not be empty; split in two halves
   * used alternately when double_buffer is enabled
   * @param dma_tx 发送缓冲区，不能为空，用法同 dma_rx / TX buffer, must not be empty,
   *        used like dma_rx
   * @param config 初始 SPI 配置 / Initial SPI configuration
   * @param dma_enable_min_size 传输长度大于该值时使用 DMA（字节）/ Transfers longer than
   *        this many bytes use DMA
   * @param enable_dma 是否允许使用 DMA / Whether DMA may be used
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

  /// 当前接收缓冲区；启用双缓冲时为当前活动的那一半 / Current RX buffer; the active half
  /// when double buffering is enabled
  RawData GetRxBuffer();

  /// 当前发送缓冲区；启用双缓冲时为当前活动的那一半 / Current TX buffer; the active half
  /// when double buffering is enabled
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
