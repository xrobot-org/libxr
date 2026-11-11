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
 * @brief CH32 I2C 主机驱动 / CH32 I2C master driver
 * @details 长度不超过 dma_enable_min_size 的传输使用轮询，更长的传输使用 DMA 并由 DMA
 *          完成中断收尾。
 *          Transfers no longer than dma_enable_min_size are polled; longer ones use DMA
 *          and are finished from the DMA completion interrupt.
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

  /// DMA 回调入口 / DMA callbacks from CH32 DMA driver
  void RxDmaIRQHandler();

  /// TX DMA 传输完成回调 / TX DMA transfer-complete callback
  void TxDmaIRQHandler();

  /// 错误中断回调 / I2C error IRQ callback
  void ErrorIRQHandler();

  /// 按 I2C 编号索引的实例表，供中断服务函数查找 / Instances indexed by I2C id, used by
  /// the ISRs to find the driver
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
  I2C_TypeDef* instance_;                ///< I2C 外设寄存器 / I2C peripheral registers
  DMA_Channel_TypeDef* dma_rx_channel_;  ///< RX DMA 通道 / RX DMA channel
  DMA_Channel_TypeDef* dma_tx_channel_;  ///< TX DMA 通道 / TX DMA channel
  ch32_i2c_id_t id_;                     ///< I2C 编号 / I2C id
  /// 使用 DMA 的最小传输字节数，不超过该值走轮询 / Transfers of at most this many bytes
  /// are polled instead of using DMA
  uint32_t dma_enable_min_size_;

  /// DMA 缓冲区，单次 DMA 传输不得超过其大小 / DMA buffer; a DMA transfer must fit in it
  RawData dma_buff_;

  ReadOperation read_op_;    ///< 进行中的读操作 / Pending read operation
  WriteOperation write_op_;  ///< 进行中的写操作 / Pending write operation
  /// DMA 读完成后数据要拷回的用户缓冲区 / User buffer the DMA-read data is copied to on
  /// completion
  RawData read_buff_;
  bool read_ = false;  ///< 当前传输为读方向 / Current transfer is a read
  /// 有 DMA 传输在进行 / A DMA transfer is in flight
  bool busy_ = false;
  /// 正在执行故障恢复，此时中断回调直接返回 / Fault recovery is running; the interrupt
  /// callbacks return early
  bool recovering_ = false;

  GPIO_TypeDef* scl_port_;  ///< SCL 所在 GPIO 端口 / GPIO port of SCL
  uint16_t scl_pin_;        ///< SCL 引脚掩码 / SCL pin mask
  GPIO_TypeDef* sda_port_;  ///< SDA 所在 GPIO 端口 / GPIO port of SDA
  uint16_t sda_pin_;        ///< SDA 引脚掩码 / SDA pin mask

  /// 当前配置（时钟频率，Hz）/ Current configuration (clock rate in Hz)
  Configuration cfg_{400000};

  bool ten_bit_addr_ = false;  ///< 使用 10 位从机地址 / Use 10-bit slave addresses
  /// 阻塞模式下等待 DMA 完成的同步对象 / Waits for DMA completion in blocking mode
  AsyncBlockWait block_wait_{};
};

}  // namespace LibXR
