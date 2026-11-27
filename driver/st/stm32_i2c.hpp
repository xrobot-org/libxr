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

  /**
   * @brief 在不具备 `Init.ClockSpeed` 字段的 HAL 句柄上空操作回退 / No-op fallback when
   *        `Init.ClockSpeed` is absent.
   * @tparam T HAL I2C 句柄指针类型 / HAL I2C handle pointer type.
   */
  template <typename T>
  typename std::enable_if<!HasClockSpeed<T>::value>::type SetClockSpeed(
      T&, const Configuration&)
  {
  }

  /**
   * @brief 写入 HAL 句柄的 `Init.ClockSpeed` 字段 / Write `Init.ClockSpeed` on the HAL
   *        I2C handle.
   * @tparam T HAL I2C 句柄指针类型 / HAL I2C handle pointer type.
   * @param i2c_handle HAL I2C 句柄 / HAL I2C handle.
   * @param config 目标 I2C 总线配置 / Target I2C configuration.
   */
  template <typename T>
  typename std::enable_if<HasClockSpeed<T>::value>::type SetClockSpeed(
      T& i2c_handle, const Configuration& config)
  {
    i2c_handle->Init.ClockSpeed = config.clock_speed;
  }

  /**
   * @brief 写入 HAL 句柄的 `Init.Timing` 字段 / Write `Init.Timing` on the HAL I2C
   * handle.
   * @tparam T HAL I2C 句柄指针类型 / HAL I2C handle pointer type.
   * @param i2c_handle HAL I2C 句柄 / HAL I2C handle.
   * @param timing 编码后的时序寄存器值 / Encoded timing register value.
   */
  template <typename T>
  typename std::enable_if<HasTiming<T>::value>::type SetTiming(T& i2c_handle,
                                                               uint32_t timing)
  {
    i2c_handle->Init.Timing = timing;
  }

  /**
   * @brief 在不具备 `Init.Timing` 字段的 HAL 句柄上空操作回退 / No-op fallback when
   *        `Init.Timing` is absent.
   * @tparam T HAL I2C 句柄指针类型 / HAL I2C handle pointer type.
   */
  template <typename T>
  typename std::enable_if<!HasTiming<T>::value>::type SetTiming(T&, uint32_t)
  {
  }

  ErrorCode SetConfig(Configuration config) override;

  stm32_i2c_id_t id_;              ///< STM32 I2C 实例编号 / STM32 I2C instance ID.
  I2C_HandleTypeDef* i2c_handle_;  ///< HAL I2C 句柄 / HAL I2C handle.
  uint32_t dma_enable_min_size_;   ///< 启用 DMA 的最小传输字节数 / Minimum transfer size
                                   ///< for DMA.

  RawData dma_buff_;  ///< 内部 DMA 暂存缓冲区 / Internal DMA staging buffer.

  ReadOperation read_op_;    ///< 当前读操作完成句柄 / Active read completion handle.
  WriteOperation write_op_;  ///< 当前写操作完成句柄 / Active write completion handle.
  RawData read_buff_;        ///< 当前读目标缓冲区 / Active read destination buffer.

  bool read_ =
      false;  ///< 当前方向标志（true 为读） / Current direction flag (true for read).
  bool recovering_ =
      false;  ///< 总线错误恢复进行中标志 / Bus error recovery in progress flag.
  AsyncBlockWait block_wait_{};  ///< 阻塞等待辅助对象 / Blocking wait helper.

  static STM32I2C* map[STM32_I2C_NUMBER];  ///< 按实例 ID 索引的驱动映射表 / Driver map
                                           ///< indexed by I2C ID. NOLINT
};
}  // namespace LibXR

#endif
