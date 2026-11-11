#pragma once

#include <ti/driverlib/dl_i2c.h>

#include "i2c.hpp"
#include "ti_msp_dl_config.h"

namespace LibXR
{

/**
 * @brief MSPM0 I2C 控制器（主机模式）/ MSPM0 I2C controller (controller role)
 */
class MSPM0I2C : public I2C
{
 public:
  /// 一个 I2C 实例的硬件资源描述 / Hardware resources of one I2C instance
  struct Resources
  {
    I2C_Regs* instance;             ///< I2C 外设寄存器 / I2C peripheral registers
    IRQn_Type irqn;                 ///< 该实例的中断号 / Interrupt number of the instance
    uint32_t clock_freq;            ///< 外设时钟（Hz，>0）/ Peripheral clock (Hz, > 0)
    uint32_t default_bus_speed_hz;  ///< 配置速率为 0 时使用的总线速率（Hz）/ Bus speed
                                    ///< (Hz) used when the configured speed is 0
    uint8_t index;                  ///< 实例编号，小于 4 / Instance index, below 4
  };

  /**
   * @brief 构造并按给定配置初始化 I2C / Construct and configure the I2C controller
   * @param res 硬件资源 / Hardware resources
   * @param stage_buffer DMA 搬运用的暂存缓冲区，不能为空 / Staging buffer used for DMA
   *        transfers, must not be empty
   * @param dma_enable_min_size 传输长度大于该值时使用 DMA（字节）/ Transfers longer than
   *        this many bytes use DMA
   * @param config 初始配置；clock_speed 为 0 时取 res.default_bus_speed_hz / Initial
   *        configuration; a clock_speed of 0 falls back to res.default_bus_speed_hz
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

  /// 由 I2C 中断号取得实例编号，未知中断号返回 0xFF / Resolve the instance index from
  /// the I2C IRQ number; 0xFF for an unknown IRQ
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
