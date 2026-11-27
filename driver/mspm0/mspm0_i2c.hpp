#pragma once

#include <ti/driverlib/dl_i2c.h>

#include "i2c.hpp"
#include "ti_msp_dl_config.h"

namespace LibXR
{

/**
 * @class MSPM0I2C
 * @brief TI MSPM0 I2C 主机驱动实现，支持 DMA 与中断传输
 *        TI MSPM0 I2C master driver supporting DMA and interrupt transfers.
 */
class MSPM0I2C : public I2C
{
 public:
  /**
   * @brief MSPM0 I2C 硬件资源描述 / MSPM0 I2C hardware resource descriptor.
   */
  struct Resources
  {
    I2C_Regs* instance;   ///< I2C 外设寄存器指针 / I2C peripheral register pointer.
    IRQn_Type irqn;       ///< I2C 中断号 / I2C interrupt number.
    uint32_t clock_freq;  ///< 外设输入时钟频率（Hz） / Peripheral clock frequency (Hz).
    uint32_t default_bus_speed_hz;  ///< 默认 I2C 总线速率（Hz） / Default I2C bus
                                    ///< frequency (Hz).
    uint8_t index;                  ///< 内部实例表索引 / Internal instance table index.
  };

  /**
   * @brief 构造 MSPM0I2C 主机驱动实例 / Construct an `MSPM0I2C` master driver instance.
   * @param res 硬件资源描述 / Hardware resource descriptor.
   * @param stage_buffer DMA 暂存缓冲区 / DMA staging buffer.
   * @param dma_enable_min_size 启用 DMA 的最小传输字节数 / Minimum transfer size for DMA.
   * @param config 初始 I2C 总线配置 / Initial I2C configuration.
   */
  MSPM0I2C(Resources res, RawData stage_buffer, uint32_t dma_enable_min_size = 8,
           I2C::Configuration config = {100000});

  ErrorCode Read(uint16_t slave_addr, RawData read_data, ReadOperation& op,
                 bool in_isr = false) override;

  ErrorCode Write(uint16_t slave_addr, ConstRawData write_data, WriteOperation& op,
                  bool in_isr = false) override;

  ErrorCode MemRead(uint16_t slave_addr, uint16_t mem_addr, RawData read_data,
                    ReadOperation& op,
                    MemAddrLength mem_addr_size = MemAddrLength::BYTE_8,
                    bool in_isr = false) override;

  ErrorCode MemWrite(uint16_t slave_addr, uint16_t mem_addr, ConstRawData write_data,
                     WriteOperation& op,
                     MemAddrLength mem_addr_size = MemAddrLength::BYTE_8,
                     bool in_isr = false) override;

  ErrorCode SetConfig(Configuration config) override;

  /**
   * @brief 根据 I2C 中断号解析内部实例索引 / Resolve the internal instance index from an
   *        I2C IRQn.
   * @param irqn MSPM0 I2C 中断号 / MSPM0 I2C interrupt number.
   * @return 对应实例索引 / Resolved instance index.
   */
  static constexpr uint8_t ResolveIndex(IRQn_Type irqn)
  {
    switch (irqn)
    {
#if defined(I2C0_BASE)
      case I2C0_INT_IRQn:
        return 0;
#endif
#if defined(I2C1_BASE)
      case I2C1_INT_IRQn:
        return 1;
#endif
#if defined(I2C2_BASE)
      case I2C2_INT_IRQn:
        return 2;
#endif
#if defined(I2C3_BASE)
      case I2C3_INT_IRQn:
        return 3;
#endif
      default:
        return INVALID_INSTANCE_INDEX;
    }
  }

 private:
  static constexpr uint8_t MAX_I2C_INSTANCES = 4;
  static constexpr uint8_t INVALID_INSTANCE_INDEX = 0xFF;

  ErrorCode WaitControllerIdle() const;

  ErrorCode WaitTransactionDone() const;

  ErrorCode WaitBusIdle() const;

  ErrorCode CheckControllerError() const;

  ErrorCode PollingWrite7(uint16_t addr7, const uint8_t* data, size_t size);

  ErrorCode PollingRead7(uint16_t addr7, uint8_t* data, size_t size);

  ErrorCode DmaWrite7(uint16_t addr7, ConstRawData write_data);

  ErrorCode DmaRead7(uint16_t addr7, RawData read_data);

  ErrorCode WaitDmaTransferDone(uint8_t channel_id) const;

  Resources res_;
  RawData stage_buffer_;
  uint32_t dma_enable_min_size_;
  bool dma_enabled_ = false;
};

/**
 * @brief 由 SysConfig 宏生成 MSPM0I2C 构造参数 / MSPM0I2C constructor arguments from
 *        SysConfig macros.
 * @param clock_hz I2C 输入时钟，即该实例所在电源域的 BUSCLK（PD0 为 ULPCLK，PD1 为
 *        MCLK）；SysConfig 不导出该频率 / I2C input clock, the BUSCLK of the instance's
 *        power domain (ULPCLK for PD0, MCLK for PD1); SysConfig does not export it.
 */
#define MSPM0_I2C_INIT(name, clock_hz, stage_addr, stage_size, dma_min_size)           \
  ::LibXR::MSPM0I2C::Resources{name##_INST, name##_INST_INT_IRQN,                      \
                               static_cast<uint32_t>(clock_hz),                        \
                               static_cast<uint32_t>(name##_BUS_SPEED_HZ),             \
                               ::LibXR::MSPM0I2C::ResolveIndex(name##_INST_INT_IRQN)}, \
      ::LibXR::RawData{(stage_addr), (stage_size)}, (dma_min_size)

}  // namespace LibXR
