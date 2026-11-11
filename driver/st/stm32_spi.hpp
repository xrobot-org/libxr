#pragma once

#include <cstring>

#include "main.h"

#ifdef HAL_SPI_MODULE_ENABLED

#ifdef SPI
#undef SPI
#endif

#include "libxr_def.hpp"
#include "libxr_rw.hpp"
#include "spi.hpp"

typedef enum
{
#ifdef SPI1
  STM32_SPI1,
#endif
#ifdef SPI2
  STM32_SPI2,
#endif
#ifdef SPI3
  STM32_SPI3,
#endif
#ifdef SPI4
  STM32_SPI4,
#endif
#ifdef SPI5
  STM32_SPI5,
#endif
#ifdef SPI6
  STM32_SPI6,
#endif
#ifdef SPI7
  STM32_SPI7,
#endif
#ifdef SPI8
  STM32_SPI8,
#endif
  STM32_SPI_NUMBER,
  STM32_SPI_ID_ERROR
} stm32_spi_id_t;

stm32_spi_id_t STM32_SPI_GetID(SPI_TypeDef* addr);  // NOLINT

namespace LibXR
{
/**
 * @brief STM32 SPI 驱动实现 / STM32 SPI driver implementation
 */
class STM32SPI : public SPI
{
 public:
  /**
   * @brief 构造 SPI 对象 / Construct SPI object
   */
  STM32SPI(SPI_HandleTypeDef* spi_handle, RawData dma_buff_rx, RawData dma_buff_tx,
           uint32_t dma_enable_min_size = 3);

  ErrorCode ReadAndWrite(RawData read_data, ConstRawData write_data, OperationRW& op,
                         bool in_isr) override;

  ErrorCode SetConfig(SPI::Configuration config) override;

  ErrorCode MemRead(uint16_t reg, RawData read_data, OperationRW& op,
                    bool in_isr) override;

  ErrorCode MemWrite(uint16_t reg, ConstRawData write_data, OperationRW& op,
                     bool in_isr) override;

  uint32_t GetMaxBusSpeed() const override;

  Prescaler GetMaxPrescaler() const override;

  ErrorCode Transfer(size_t size, OperationRW& op, bool in_isr) override;

  /// HAL SPI 句柄 / HAL SPI handle
  SPI_HandleTypeDef* spi_handle_;

  /// 外设编号，同时是 map 的下标 / Peripheral ID, also the index into map
  stm32_spi_id_t id_ = STM32_SPI_ID_ERROR;

  /// 传输长度大于该值时使用 DMA（字节）/ Transfers longer than this many bytes use DMA
  uint32_t dma_enable_min_size_ = 3;

  /// 进行中传输的完成通知 / Completion operation of the transfer in flight
  OperationRW rw_op_;

  /// 调用者的读目标缓冲区，DMA 完成后从接收缓冲区拷入；无读目标时为空 / Caller's read
  /// destination, filled from the RX buffer when the DMA completes; empty without one
  RawData read_buff_;

  /// 进行中的传输是寄存器读（接收数据前带一个地址字节）/ The transfer in flight is a
  /// register read (one address byte precedes the received data)
  bool mem_read_ = false;
  /// 阻塞模式下等待完成的同步对象 / Synchronization for waiting in blocking mode
  AsyncBlockWait block_wait_{};

  /// 外设编号到实例的映射，供 HAL 回调找回对象 / Instance table by peripheral ID, used
  /// by HAL callbacks to find the object
  static STM32SPI* map[STM32_SPI_NUMBER];  // NOLINT

 private:
  /**
   * @brief DMA 启动失败后恢复 HAL 句柄
   *        Recover the HAL handle after a failed DMA start
   * @param st DMA 启动函数的返回值 / Return value of the DMA start call
   * @note 多个 STM32 HAL 系列（其中有 F1、F3、F4、F7、G0）在 DMA
   *       通道启动失败时返回 HAL_ERROR，但不把句柄状态改回 READY，
   *       之后每次传输都会返回 BUSY。这里中止该次传输，让下一次传输可以启动。
   *       Several STM32 HAL families (F1, F3, F4, F7 and G0 among them) return
   *       HAL_ERROR when a DMA channel fails to start but leave the handle state
   *       busy, so every later transfer returns BUSY. This aborts that transfer so
   *       the next one can start.
   */
  void RecoverFailedDmaStart(HAL_StatusTypeDef st);
};

}  // namespace LibXR

#endif
