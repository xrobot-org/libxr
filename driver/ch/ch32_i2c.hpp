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

/**
 * @class CH32I2C
 * @brief CH32 I2C 主机驱动实现，支持 7/10 位寻址与 DMA/轮询传输
 *        CH32 I2C master driver supporting 7/10-bit addressing and DMA/polling transfers.
 */
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

  /// DMA 接收中断回调入口 / DMA RX interrupt handler entry.
  void RxDmaIRQHandler();
  /// DMA 发送中断回调入口 / DMA TX interrupt handler entry.
  void TxDmaIRQHandler();

  /// 错误中断回调 / I2C error IRQ callback
  void ErrorIRQHandler();

  static CH32I2C* map_[CH32_I2C_NUMBER];  ///< 按 I2C 外设 ID 索引的驱动实例表 / Driver
                                          ///< instance lookup table indexed by I2C ID.

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
  I2C_TypeDef* instance_;  ///< CH32 I2C 外设寄存器基址 / I2C peripheral registers.
  DMA_Channel_TypeDef* dma_rx_channel_;  ///< RX DMA 通道 / RX DMA channel.
  DMA_Channel_TypeDef* dma_tx_channel_;  ///< TX DMA 通道 / TX DMA channel.
  ch32_i2c_id_t id_;                     ///< I2C 实例编号 / I2C instance ID.
  uint32_t dma_enable_min_size_;  ///< 启用 DMA 的最小传输字节数 / Minimum transfer size
                                  ///< for DMA.

  RawData dma_buff_;  ///< 内部 DMA 暂存缓冲区 / Internal DMA staging buffer.

  ReadOperation read_op_;    ///< 当前读操作完成句柄 / Pending read completion operation.
  WriteOperation write_op_;  ///< 当前写操作完成句柄 / Pending write completion operation.
  RawData read_buff_;        ///< 当前读请求目标缓冲区 / Active read destination buffer.
  bool read_ =
      false;  ///< 当前传输方向标志（true 为读） / Current direction flag (true for read).
  bool busy_ = false;  ///< 传输忙碌标志 / Active transfer busy flag.
  bool recovering_ =
      false;  ///< 总线错误恢复中标志 / Bus error recovery in progress flag.

  GPIO_TypeDef* scl_port_;  ///< SCL 引脚端口 / SCL GPIO port.
  uint16_t scl_pin_;        ///< SCL 引脚掩码 / SCL GPIO pin mask.
  GPIO_TypeDef* sda_port_;  ///< SDA 引脚端口 / SDA GPIO port.
  uint16_t sda_pin_;        ///< SDA 引脚掩码 / SDA GPIO pin mask.

  Configuration cfg_{400000};  ///< 当前 I2C 时钟配置 / Current I2C clock configuration.

  bool ten_bit_addr_ =
      false;  ///< 是否启用 10 位从机地址 / Whether 10-bit slave addressing is enabled.
  AsyncBlockWait block_wait_{};  ///< 阻塞等待辅助对象 / Blocking wait helper.
};

}  // namespace LibXR
