#pragma once

#include <type_traits>
#include <utility>

#include "main.h"

#ifdef HAL_I2C_MODULE_ENABLED

#ifdef I2C
#undef I2C
#endif

#include "i2c.hpp"
#include "libxr_assert.hpp"

typedef enum
{
#ifdef I2C1
  STM32_I2C1,
#endif
#ifdef I2C2
  STM32_I2C2,
#endif
#ifdef I2C3
  STM32_I2C3,
#endif
#ifdef I2C4
  STM32_I2C4,
#endif
#ifdef I2C5
  STM32_I2C5,
#endif
#ifdef I2C6
  STM32_I2C6,
#endif
#ifdef I2C7
  STM32_I2C7,
#endif
#ifdef I2C8
  STM32_I2C8,
#endif
  STM32_I2C_NUMBER,
  STM32_I2C_ID_ERROR
} stm32_i2c_id_t;

stm32_i2c_id_t STM32_I2C_GetID(I2C_TypeDef* hi2c);  // NOLINT

namespace LibXR
{
/**
 * @brief STM32 I2C 驱动实现 / STM32 I2C driver implementation
 */
class STM32I2C : public I2C
{
 public:
  /**
   * @brief 构造 I2C 对象 / Construct I2C object
   */
  STM32I2C(I2C_HandleTypeDef* hi2c, RawData dma_buff, uint32_t dma_enable_min_size = 3);

  ErrorCode Read(uint16_t slave_addr, RawData read_data, ReadOperation& op,
                 bool in_isr) override;

  ErrorCode Write(uint16_t slave_addr, ConstRawData write_data, WriteOperation& op,
                  bool in_isr) override;

  ErrorCode MemRead(uint16_t slave_addr, uint16_t mem_addr, RawData read_data,
                    ReadOperation& op, MemAddrLength mem_addr_size, bool in_isr) override;

  ErrorCode MemWrite(uint16_t slave_addr, uint16_t mem_addr, ConstRawData write_data,
                     WriteOperation& op, MemAddrLength mem_addr_size,
                     bool in_isr) override;

  template <typename, typename = void>
  struct HasTiming : std::false_type
  {
  };

  template <typename T>
  struct HasTiming<T, std::void_t<decltype(std::declval<T>()->Init.Timing)>>
      : std::true_type
  {
  };

  template <typename, typename = void>
  struct HasClockSpeed : std::false_type
  {
  };

  template <typename T>
  struct HasClockSpeed<T, std::void_t<decltype(std::declval<T>()->Init.ClockSpeed)>>
      : std::true_type
  {
  };

  /// 句柄没有 Init.ClockSpeed 字段时不做任何事 / No-op when the handle has no
  /// Init.ClockSpeed field
  template <typename T>
  typename std::enable_if<!HasClockSpeed<T>::value>::type SetClockSpeed(
      T&, const Configuration&)
  {
  }

  /// 把配置中的时钟速度写入 Init.ClockSpeed（Hz）/ Write the configured clock speed into
  /// Init.ClockSpeed (Hz)
  template <typename T>
  typename std::enable_if<HasClockSpeed<T>::value>::type SetClockSpeed(
      T& i2c_handle, const Configuration& config)
  {
    i2c_handle->Init.ClockSpeed = config.clock_speed;
  }

  /// 把 timing 写入 Init.Timing / Write timing into Init.Timing
  template <typename T>
  typename std::enable_if<HasTiming<T>::value>::type SetTiming(T& i2c_handle,
                                                               uint32_t timing)
  {
    i2c_handle->Init.Timing = timing;
  }

  /// 句柄没有 Init.Timing 字段时不做任何事 / No-op when the handle has no Init.Timing
  /// field
  template <typename T>
  typename std::enable_if<!HasTiming<T>::value>::type SetTiming(T&, uint32_t)
  {
  }

  ErrorCode SetConfig(Configuration config) override;

  /// 外设编号，同时是 map 的下标 / Peripheral ID, also the index into map
  stm32_i2c_id_t id_;
  /// HAL I2C 句柄 / HAL I2C handle
  I2C_HandleTypeDef* i2c_handle_;
  /// 传输长度大于该值时使用 DMA（字节）/ Transfers longer than this many bytes use DMA
  uint32_t dma_enable_min_size_;

  /// DMA 传输用的暂存缓冲区 / Staging buffer for DMA transfers
  RawData dma_buff_;

  /// 进行中读事务的完成通知 / Completion operation of the read in flight
  ReadOperation read_op_;
  /// 进行中写事务的完成通知 / Completion operation of the write in flight
  WriteOperation write_op_;
  /// 调用者的读目标缓冲区，DMA 完成后从 dma_buff_ 拷入 / Caller's read destination,
  /// filled from dma_buff_ when the DMA completes
  RawData read_buff_;

  /// 当前事务是读 / Whether the current transaction is a read
  bool read_ = false;
  /// 阻塞超时后正在恢复外设，期间拒绝新事务 / Peripheral recovery after a blocking
  /// timeout is running; new transactions are refused meanwhile
  bool recovering_ = false;
  /// 阻塞模式下等待完成的同步对象 / Synchronization for waiting in blocking mode
  AsyncBlockWait block_wait_{};

  /// 外设编号到实例的映射，供 HAL 回调找回对象 / Instance table by peripheral ID, used
  /// by HAL callbacks to find the object
  static STM32I2C* map[STM32_I2C_NUMBER];  // NOLINT
};
}  // namespace LibXR

#endif
