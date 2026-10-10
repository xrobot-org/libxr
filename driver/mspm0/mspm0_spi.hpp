#pragma once

#include <ti/driverlib/dl_dma.h>
#include <ti/driverlib/dl_spi.h>

#include "spi.hpp"
#include "ti_msp_dl_config.h"

namespace LibXR
{

/**
 * @brief MSPM0 SPI 控制器 / MSPM0 SPI controller.
 *
 * @note 构造时开启 DMA 收发完成中断，并把 SPI 中断设为优先级 1；SysConfig 中该实例的
 *       中断和优先级设置会被覆盖。
 *       The constructor enables the DMA RX/TX done interrupts and sets the SPI interrupt
 *       to priority 1, replacing the interrupt and priority settings SysConfig made for
 *       the instance.
 */
class MSPM0SPI : public SPI
{
 public:
  struct Resources
  {
    SPI_Regs* instance;
    IRQn_Type irqn;
    uint32_t clock_freq;
    uint8_t index;
    uint8_t dma_rx_channel;
    uint8_t dma_tx_channel;
  };

  MSPM0SPI(Resources res, RawData dma_rx_buffer, RawData dma_tx_buffer,
           uint32_t dma_enable_min_size = 3,
           SPI::Configuration config = {SPI::ClockPolarity::LOW, SPI::ClockPhase::EDGE_1,
                                        SPI::Prescaler::DIV_4, false});

  ErrorCode ReadAndWrite(RawData read_data, ConstRawData write_data, OperationRW& op,
                         bool in_isr = false) override;

  ErrorCode SetConfig(SPI::Configuration config) override;

  ErrorCode MemRead(uint16_t reg, RawData read_data, OperationRW& op,
                    bool in_isr = false) override;

  ErrorCode MemWrite(uint16_t reg, ConstRawData write_data, OperationRW& op,
                     bool in_isr = false) override;

  uint32_t GetMaxBusSpeed() const override;

  Prescaler GetMaxPrescaler() const override;

  ErrorCode Transfer(size_t size, OperationRW& op, bool in_isr = false) override;

  static void OnInterrupt(uint8_t index);

  static constexpr uint8_t ResolveIndex(IRQn_Type irqn)
  {
    switch (irqn)
    {
#if defined(SPI0_BASE)
      case SPI0_INT_IRQn:
        return 0;
#endif
#if defined(SPI1_BASE)
      case SPI1_INT_IRQn:
        return 1;
#endif
#if defined(SPI2_BASE)
      case SPI2_INT_IRQn:
        return 2;
#endif
      default:
        return INVALID_INSTANCE_INDEX;
    }
  }

 private:
  enum class DmaMode : uint8_t
  {
    DUPLEX,
    RX_ONLY,
    TX_ONLY
  };

#if defined(SPI2_BASE)
  static constexpr uint8_t MAX_SPI_INSTANCES = 3;
#elif defined(SPI1_BASE)
  static constexpr uint8_t MAX_SPI_INSTANCES = 2;
#else
  static constexpr uint8_t MAX_SPI_INSTANCES = 1;
#endif
  static constexpr uint8_t INVALID_INSTANCE_INDEX = 0xFF;
  static constexpr uint32_t RX_ONLY_REPEAT_TX_MAX_FRAMES = 256U;

  ErrorCode PollingTransfer(uint8_t* rx, const uint8_t* tx, uint32_t len);

  ErrorCode CompleteDmaOperation(OperationRW& op, bool in_isr);

  void StartDmaDuplex(uint32_t count);

  void StartDmaRxOnly(uint32_t offset, uint32_t count);

  void StartDmaTxOnly(uint32_t count);

  void StopDma();

  void HandleInterrupt();

  bool DmaBusy() const;

  Resources res_;
  uint32_t dma_enable_min_size_;
  OperationRW rw_op_;
  volatile ErrorCode dma_result_ = ErrorCode::OK;
  RawData read_buff_;
  bool mem_read_ = false;
  DmaMode dma_mode_ = DmaMode::DUPLEX;
  uint32_t masked_interrupts_for_tx_only_ = 0;
  uint32_t rx_only_offset_ = 0;
  uint32_t rx_only_remaining_ = 0;
  volatile bool busy_ = false;

  static MSPM0SPI* instance_map_[MAX_SPI_INSTANCES];
};

/**
 * @brief 由 SysConfig 宏生成 MSPM0SPI 构造参数 / MSPM0SPI constructor arguments from
 *        SysConfig macros.
 * @param clock_hz SPI 输入时钟，即该实例所在电源域的 BUSCLK（PD0 为 ULPCLK，PD1 为
 *        MCLK）；SysConfig 在 SYSCFG_DL_NAME_init 的注释中给出该值 / SPI input clock,
 *        the BUSCLK of the instance's power domain (ULPCLK for PD0, MCLK for PD1);
 *        SysConfig states it in a comment of SYSCFG_DL_NAME_init.
 */
#define MSPM0_SPI_INIT(name, clock_hz, dma_rx_name, dma_tx_name, rx_buffer_addr,      \
                       rx_buffer_size, tx_buffer_addr, tx_buffer_size, dma_min_size)  \
  ::LibXR::MSPM0SPI::Resources{name##_INST,                                           \
                               name##_INST_INT_IRQN,                                  \
                               static_cast<uint32_t>(clock_hz),                       \
                               ::LibXR::MSPM0SPI::ResolveIndex(name##_INST_INT_IRQN), \
                               static_cast<uint8_t>(dma_rx_name##_CHAN_ID),           \
                               static_cast<uint8_t>(dma_tx_name##_CHAN_ID)},          \
      ::LibXR::RawData{(rx_buffer_addr), (rx_buffer_size)},                           \
      ::LibXR::RawData{(tx_buffer_addr), (tx_buffer_size)}, (dma_min_size)

}  // namespace LibXR
