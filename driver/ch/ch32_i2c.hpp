#pragma once

#include "libxr.hpp"
#include DEF2STR(LIBXR_CH32_CONFIG_FILE)

#ifdef I2C
#undef I2C
#endif

#include "ch32_i2c_def.hpp"
#include "i2c.hpp"
#include "libxr_def.hpp"
#include "libxr_rw.hpp"
#include "timebase.hpp"

namespace LibXR
{

class CH32I2C : public I2C
{
 public:
  /**
   * @brief 构造 I2C 驱动对象 / Construct I2C driver object
   * @details `slave_addr` 使用不带 R/W 位的原始 7 位或 10 位地址。
   *          `slave_addr` uses raw 7-bit or 10-bit addresses without the R/W bit.
   */
  CH32I2C(ch32_i2c_id_t id, RawData dma_buff, GPIO_TypeDef* scl_port, uint16_t scl_pin,
          GPIO_TypeDef* sda_port, uint16_t sda_pin, uint32_t pin_remap = 0,
          uint32_t dma_enable_min_size = 3, uint32_t default_clock_hz = 400000,
          bool ten_bit_addr = false);

  ErrorCode Read(uint16_t slave_addr, RawData read_data, ReadOperation& op,
                 bool in_isr) override;

  ErrorCode Write(uint16_t slave_addr, ConstRawData write_data, WriteOperation& op,
                  bool in_isr) override;

  ErrorCode MemRead(uint16_t slave_addr, uint16_t mem_addr, RawData read_data,
                    ReadOperation& op, MemAddrLength mem_addr_size, bool in_isr) override;

  ErrorCode MemWrite(uint16_t slave_addr, uint16_t mem_addr, ConstRawData write_data,
                     WriteOperation& op, MemAddrLength mem_addr_size,
                     bool in_isr) override;

  ErrorCode SetConfig(Configuration config) override;

  /// DMA 回调入口 / DMA callbacks from CH32 DMA driver
  void RxDmaIRQHandler();
  void TxDmaIRQHandler();

  /// 错误中断回调 / I2C error IRQ callback
  void ErrorIRQHandler();

  static CH32I2C* map_[CH32_I2C_NUMBER];

 private:
  /// 默认超时（微秒） / Default timeout in microseconds
  static constexpr uint32_t K_DEFAULT_TIMEOUT_US = 20000;  // 20ms

  inline bool DmaBusy() const
  {
    return (dma_rx_channel_->CNTR != 0) || (dma_tx_channel_->CNTR != 0) || busy_;
  }

  static inline uint8_t Addr7ToAddr8(uint16_t addr7)
  {
    // WCH SPL 在 I2C_Send7bitAddress 中仅更新 bit0(R/W)。
    // WCH SPL updates only bit0 (R/W) in I2C_Send7bitAddress.
    ASSERT(addr7 <= 0x7F);
    return static_cast<uint8_t>(((addr7 & 0x7F) << 1) & 0xFE);
  }

  static inline uint16_t Addr10Clamp(uint16_t addr10)
  {
    ASSERT(addr10 <= 0x3FF);
    return static_cast<uint16_t>(addr10 & 0x3FF);
  }

  bool WaitEvent(uint32_t evt, uint32_t timeout_us = K_DEFAULT_TIMEOUT_US);
  bool WaitFlag(uint32_t flag, FlagStatus st, uint32_t timeout_us = K_DEFAULT_TIMEOUT_US);
  ErrorCode WaitEventOrRecover(uint32_t evt, uint32_t timeout_us = K_DEFAULT_TIMEOUT_US);
  ErrorCode WaitFlagOrRecover(uint32_t flag, FlagStatus st,
                              uint32_t timeout_us = K_DEFAULT_TIMEOUT_US);

  void ClearAddrFlag();

  ErrorCode MasterStartAndAddress(uint16_t slave_addr, uint8_t dir);

  // 10 位地址流程：先以写方向发送地址阶段，读流程再发重复起始切到读方向。
  // 10-bit address flow: use write direction for address phase, then repeated start
  // with read direction when needed.
  ErrorCode MasterStartAndAddress10Bit(uint16_t addr10, uint8_t final_dir);

  ErrorCode SendMemAddr(uint16_t mem_addr, MemAddrLength len);

  ErrorCode PollingWriteBytes(const uint8_t* data, uint32_t len);
  ErrorCode PollingReadBytes(uint8_t* data, uint32_t len);

  void StartTxDma(uint32_t len);
  void StartRxDma(uint32_t len);

  // Caller has either checked idle state or stopped DMA for recovery.
  void ApplyConfig(Configuration config);

  void AbortTransfer(ErrorCode ec);
  void RecoverAfterImmediateFailure();

 public:
  I2C_TypeDef* instance_;
  DMA_Channel_TypeDef* dma_rx_channel_;
  DMA_Channel_TypeDef* dma_tx_channel_;
  ch32_i2c_id_t id_;
  uint32_t dma_enable_min_size_;

  RawData dma_buff_;

  ReadOperation read_op_;
  WriteOperation write_op_;
  RawData read_buff_;
  bool read_ = false;
  bool busy_ = false;
  bool recovering_ = false;

  GPIO_TypeDef* scl_port_;
  uint16_t scl_pin_;
  GPIO_TypeDef* sda_port_;
  uint16_t sda_pin_;

  Configuration cfg_{400000};

  bool ten_bit_addr_ = false;
  AsyncBlockWait block_wait_{};
};

}  // namespace LibXR
