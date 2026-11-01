#pragma once

#include "libxr.hpp"
#include DEF2STR(LIBXR_CH32_CONFIG_FILE)

#include <cstring>

#include "ch32_spi_def.hpp"
#include "libxr_def.hpp"
#include "libxr_rw.hpp"
#include "spi.hpp"

namespace LibXR
{

/**
 * @brief CH32 SPI 驱动实现 / CH32 SPI driver implementation
 */
class CH32SPI : public SPI
{
 public:
  /**
   * @brief 构造 SPI 对象 / Construct SPI object
   */
  CH32SPI(ch32_spi_id_t id, RawData dma_rx, RawData dma_tx, GPIO_TypeDef* sck_port,
          uint16_t sck_pin, GPIO_TypeDef* miso_port, uint16_t miso_pin,
          GPIO_TypeDef* mosi_port, uint16_t mosi_pin, uint32_t pin_remap = 0,
          bool master_mode = true, bool firstbit_msb = true,
          uint16_t prescaler = SPI_BaudRatePrescaler_64, uint32_t dma_enable_min_size = 3,
          SPI::Configuration config = {SPI::ClockPolarity::LOW, SPI::ClockPhase::EDGE_1});

  /// SPI 接口实现 / SPI interface overrides
  ErrorCode ReadAndWrite(RawData read_data, ConstRawData write_data, OperationRW& op,
                         bool in_isr) override;
  ErrorCode SetConfig(SPI::Configuration config) override;
  ErrorCode MemRead(uint16_t reg, RawData read_data, OperationRW& op,
                    bool in_isr) override;
  ErrorCode MemWrite(uint16_t reg, ConstRawData write_data, OperationRW& op,
                     bool in_isr) override;

  ErrorCode Transfer(size_t size, OperationRW& op, bool in_isr) override;
  uint32_t GetMaxBusSpeed() const override;
  SPI::Prescaler GetMaxPrescaler() const override;

  /// DMA 中断回调 / DMA interrupt callbacks
  void RxDmaIRQHandler();
  void TxDmaIRQHandler();

  /// 轮询传输辅助函数 / Polling transfer helper
  ErrorCode PollingTransfer(uint8_t* rx, const uint8_t* tx, uint32_t len);

  static CH32SPI* map_[CH32_SPI_NUMBER];

 private:
  inline bool DmaBusy() const
  {
    return (dma_rx_channel_->CNTR != 0) || (dma_tx_channel_->CNTR != 0) || busy_;
  }

  void StartDmaDuplex(uint32_t count);
  void StopDma();
  void AbortDma(uint32_t transfer);
  ErrorCode WaitBlock(uint32_t timeout);

  void PrepareTxBuffer(ConstRawData write_data, uint32_t need_len, uint32_t prefix = 0,
                       uint8_t dummy = 0x00);

  static SPI::Prescaler MapCH32PrescalerToEnum(uint16_t p);
  static bool MapEnumToCH32Prescaler(SPI::Prescaler p, uint16_t& out);

 public:
  SPI_TypeDef* instance_;
  DMA_Channel_TypeDef* dma_rx_channel_;
  DMA_Channel_TypeDef* dma_tx_channel_;
  ch32_spi_id_t id_;
  uint32_t dma_enable_min_size_;

  /// 运行时状态 / Runtime state
  OperationRW rw_op_;
  RawData read_buff_;
  bool mem_read_ = false;
  bool busy_ = false;
  uint32_t dma_transfer_ = 0;  ///< 已开始的 DMA 传输数 / DMA transfers started

  /// SPI 寄存器配置缓存 / Cached SPI register configuration
  uint16_t mode_;       ///< SPI_Mode_Master / SPI_Mode_Slave
  uint16_t datasize_;   ///< SPI_DataSize_8b by default
  uint16_t firstbit_;   ///< SPI_FirstBit_MSB / SPI_FirstBit_LSB
  uint16_t prescaler_;  ///< SPI_BaudRatePrescaler_x
  uint16_t nss_ = SPI_NSS_Soft;

  /// GPIO 配置 / GPIO configuration
  GPIO_TypeDef* sck_port_;
  uint16_t sck_pin_;
  GPIO_TypeDef* miso_port_;
  uint16_t miso_pin_;
  GPIO_TypeDef* mosi_port_;
  uint16_t mosi_pin_;

  AsyncBlockWait block_wait_{};
};

}  // namespace LibXR
