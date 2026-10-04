#pragma once

#include "libxr_rw.hpp"

namespace LibXR
{

/**
 * @brief I2C（Inter-Integrated Circuit）接口类。
 *        I2C (Inter-Integrated Circuit) interface class.
 *
 * 该类提供 I2C 通信的基础接口，包括读、写以及配置 I2C 设备的功能。
 * This class provides a fundamental interface for I2C communication,
 * including read, write, and configuration functionalities.
 */
class I2C
{
 public:
  /**
   * @brief I2C 设备的配置信息结构体。
   *        Configuration structure for an I2C device.
   */
  struct Configuration
  {
    uint32_t
        clock_speed;  ///< I2C 通信时钟速率（单位：Hz） / The I2C clock speed (in Hz).
  };

  enum class MemAddrLength : uint8_t
  {
    BYTE_8,
    BYTE_16
  };

  /**
   * @brief 默认构造函数。
   *        Default constructor.
   */
  I2C() {}

  /**
   * @brief 读取 I2C 设备的数据。
   *        Reads data from an I2C device.
   *
   * 该函数从指定的 I2C 从设备地址读取数据，并存储到 `read_data` 中。
   * This function reads data from the specified I2C slave address
   * and stores it in `read_data`.
   *
   * @param slave_addr 目标 I2C 从设备地址，不带 R/W 位。
   *                   Target I2C slave address, no R/W bit included.
   * @param read_data 存储读取数据的 `RawData` 对象。
   *                  A `RawData` object to store the read data.
   * @param op 读取操作对象，包含同步或异步操作模式。
   *           Read operation object containing synchronous or asynchronous operation
   * mode.
   * @param in_isr 是否在中断中进行操作 / Whether the operation is performed in an ISR.
   * @return 返回 `ErrorCode`，指示操作是否成功。
   *         Returns an `ErrorCode` indicating whether the operation was successful.
   */
  virtual ErrorCode Read(uint16_t slave_addr, RawData read_data, ReadOperation& op,
                         bool in_isr = false) = 0;

  /**
   * @brief 向 I2C 设备写入数据。
   *        Writes data to an I2C device.
   *
   * 该函数将 `write_data` 写入指定的 I2C 从设备地址。
   * This function writes `write_data` to the specified I2C slave address.
   *
   * @param slave_addr 目标 I2C 从设备地址，不带 R/W 位。
   *                   Target I2C slave address, no R/W bit included.
   * @param write_data 需要写入的数据，`ConstRawData` 类型。
   *                   The data to be written, of type `ConstRawData`.
   * @param op 写入操作对象，包含同步或异步操作模式。
   *           Write operation object containing synchronous or asynchronous operation
   * mode.
   * @param in_isr 是否在中断中进行操作 / Whether the operation is performed in an ISR.
   * @return 返回 `ErrorCode`，指示操作是否成功。
   *         Returns an `ErrorCode` indicating whether the operation was successful.
   */
  virtual ErrorCode Write(uint16_t slave_addr, ConstRawData write_data,
                          WriteOperation& op, bool in_isr = false) = 0;

  /**
   * @brief 配置 I2C 设备参数。
   *        Configures the I2C device settings.
   *
   * 该函数用于设置 I2C 设备的参数，例如通信速率等。
   * This function sets the parameters of the I2C device, such as the communication speed.
   *
   * @param config 包含 I2C 设置信息的 `Configuration` 结构体。
   *               A `Configuration` structure containing I2C settings.
   * @return 返回 `ErrorCode`，指示配置是否成功。
   *         Returns an `ErrorCode` indicating whether the configuration was successful.
   */
  virtual ErrorCode SetConfig(Configuration config) = 0;

  /**
   * @brief 从 I2C 设备指定寄存器读取数据。
   *        Reads data from a specific register of an I2C device.
   *
   * 该函数从指定 I2C 从设备的寄存器地址读取数据，并存储到 `read_data` 中。
   * This function reads data from the specified register of the I2C slave
   * and stores it in `read_data`.
   *
   * @param slave_addr 目标 I2C 从设备地址，不带 R/W 位。
   *                   Target I2C slave address, no R/W bit included.
   * @param mem_addr 寄存器地址（通常为 8 位或 16 位）。
   *                 Register address (typically 8-bit or 16-bit).
   * @param read_data 用于存储读取数据的 `RawData` 对象。
   *                  `RawData` object to store read data.
   * @param op 异步或同步的读取操作对象。
   *          Read operation object (sync or async).
   * @param mem_addr_size 寄存器地址长度。
   *                      Size of register address in bytes.
   * @param in_isr 是否在中断中进行操作 / Whether the operation is performed in an ISR.
   * @return 返回 `ErrorCode`，表示是否读取成功。
   *         Returns `ErrorCode` indicating success or failure.
   */
  virtual ErrorCode MemRead(uint16_t slave_addr, uint16_t mem_addr, RawData read_data,
                            ReadOperation& op,
                            MemAddrLength mem_addr_size = MemAddrLength::BYTE_8,
                            bool in_isr = false) = 0;

  /**
   * @brief 向 I2C 设备指定寄存器写入数据。
   *        Writes data to a specific register of an I2C device.
   *
   * 该函数将 `write_data` 写入指定 I2C 从设备的寄存器地址。
   * This function writes `write_data` to the specified register of the I2C slave.
   *
   * @param slave_addr 目标 I2C 从设备地址，不带 R/W 位。
   *                   Target I2C slave address, no R/W bit included.
   * @param mem_addr 寄存器地址（通常为 8 位或 16 位）。
   *                 Register address (typically 8-bit or 16-bit).
   * @param write_data 要写入的数据，`ConstRawData` 类型。
   *                   Data to be written, of type `ConstRawData`.
   * @param op 异步或同步的写入操作对象。
   *          Write operation object (sync or async).
   * @param mem_addr_size 寄存器地址长度。
   *                      Size of register address in bytes.
   * @param in_isr 是否在中断中进行操作 / Whether the operation is performed in an ISR.
   * @return 返回 `ErrorCode`，表示是否写入成功。
   *         Returns `ErrorCode` indicating success or failure.
   */
  virtual ErrorCode MemWrite(uint16_t slave_addr, uint16_t mem_addr,
                             ConstRawData write_data, WriteOperation& op,
                             MemAddrLength mem_addr_size = MemAddrLength::BYTE_8,
                             bool in_isr = false) = 0;
};

}  // namespace LibXR
