#pragma once

#include "double_buffer.hpp"
#include "libxr.hpp"

namespace LibXR
{

/**
 * @class SPI
 * @brief 串行外设接口（SPI）抽象类。
 *        Abstract class for Serial Peripheral Interface (SPI).
 */
class SPI
{
 public:
  /**
   * @enum ClockPolarity
   * @brief 定义 SPI 时钟极性 / Defines the SPI clock polarity.
   */
  enum class ClockPolarity : uint8_t
  {
    LOW = 0,  ///< 时钟空闲时为低电平 / Clock idle low.
    HIGH = 1  ///< 时钟空闲时为高电平 / Clock idle high.
  };

  /**
   * @enum ClockPhase
   * @brief 定义 SPI 时钟相位 / Defines the SPI clock phase.
   */
  enum class ClockPhase : uint8_t
  {
    EDGE_1 = 0,  ///< 在第一个时钟边沿采样数据 / Data sampled on the first clock edge.
    EDGE_2 = 1   ///< 在第二个时钟边沿采样数据 / Data sampled on the second clock edge.
  };

  enum class Prescaler : uint8_t
  {
    DIV_1 = 0,       ///< 分频系数为 1 / Division factor is 1.
    DIV_2 = 1,       ///< 分频系数为 2 / Division factor is 2.
    DIV_4 = 2,       ///< 分频系数为 4 / Division factor is 4.
    DIV_8 = 3,       ///< 分频系数为 8 / Division factor is 8.
    DIV_16 = 4,      ///< 分频系数为 16 / Division factor is 16.
    DIV_32 = 5,      ///< 分频系数为 32 / Division factor is 32.
    DIV_64 = 6,      ///< 分频系数为 64 / Division factor is 64.
    DIV_128 = 7,     ///< 分频系数为 128 / Division factor is 128.
    DIV_256 = 8,     ///< 分频系数为 256 / Division factor is 256.
    DIV_512 = 9,     ///< 分频系数为 512 / Division factor is 512.
    DIV_1024 = 10,   ///< 分频系数为 1024 / Division factor is 1024.
    DIV_2048 = 11,   ///< 分频系数为 2048 / Division factor is 2048.
    DIV_4096 = 12,   ///< 分频系数为 4096 / Division factor is 4096.
    DIV_8192 = 13,   ///< 分频系数为 8192 / Division factor is 8192.
    DIV_16384 = 14,  ///< 分频系数为 16384 / Division factor is 16384.
    DIV_32768 = 15,  ///< 分频系数为 32768 / Division factor is 32768.
    DIV_65536 = 16,  ///< 分频系数为 65536 / Division factor is 65536.
    UNKNOWN = 0xFF   ///< 未知分频系数 / Unknown prescaler.
  };

  /**
   * @typedef OperationRW
   * @brief 定义读写操作类型的别名 / Defines an alias for the read/write operation type.
   */
  using OperationRW = WriteOperation;

  /**
   * @brief 将分频系数转换为除数 / Converts a prescaler to a divisor.
   * @param prescaler 分频系数 / Prescaler.
   * @return 除数 / Divisor.
   */
  static constexpr uint32_t PrescalerToDiv(Prescaler prescaler)
  {
    if (prescaler == Prescaler::UNKNOWN)
    {
      return 0u;
    }
    const uint8_t ORD = static_cast<uint8_t>(prescaler);
    return (ORD <= 30) ? (1u << ORD) : 0u;
  }

  /**
   * @struct Configuration
   * @brief 存储 SPI 配置参数的结构体。
   *        Structure for storing SPI configuration parameters.
   */
  struct Configuration
  {
    ClockPolarity clock_polarity =
        ClockPolarity::LOW;                       ///< SPI 时钟极性 / SPI clock polarity.
    ClockPhase clock_phase = ClockPhase::EDGE_1;  ///< SPI 时钟相位 / SPI clock phase.
    Prescaler prescaler = Prescaler::UNKNOWN;     ///< SPI 分频系数 / SPI prescaler.
    bool double_buffer = false;  ///< 是否使用双缓冲区 / Whether to use double buffer.
  };

  /**
   * @struct ReadWriteInfo
   * @brief 存储 SPI 读写操作信息的结构体。
   * Structure for storing SPI read/write operation information.
   */
  struct ReadWriteInfo
  {
    RawData read_data;        ///< 读取的数据缓冲区 / Buffer for storing read data.
    ConstRawData write_data;  ///< 待写入的数据缓冲区 / Buffer for data to be written.
    OperationRW op;           ///< 读写操作类型 / Type of read/write operation.
  };

  /**
   * @brief 构造函数 / Constructor.
   * @param rx_buffer 存储接收数据的缓冲区 / Buffer to store received data.
   * @param tx_buffer 存储发送数据的缓冲区 / Buffer to store data to be sent.
   */
  SPI(RawData rx_buffer, RawData tx_buffer)
      : rx_buffer_(rx_buffer),
        tx_buffer_(tx_buffer),
        double_buffer_rx_(rx_buffer),
        double_buffer_tx_(tx_buffer)
  {
  }

  /**
   * @brief 进行 SPI 读写操作 / Performs SPI read and write operations.
   * @param read_data 存储读取数据的缓冲区 / Buffer to store the read data.
   * @param write_data 需要写入的数据缓冲区 / Buffer containing the data to be written.
   * @param op 读写操作类型 / Type of read/write operation.
   * @param in_isr 是否在中断中进行操作 / Whether the operation is performed in an ISR.
   * @return 操作结果的错误码 / Error code indicating the result of the operation.
   */
  virtual ErrorCode ReadAndWrite(RawData read_data, ConstRawData write_data,
                                 OperationRW& op, bool in_isr = false) = 0;

  /**
   * @brief 进行 SPI 读取操作 / Performs SPI read operation.
   * @param read_data 存储读取数据的缓冲区 / Buffer to store the read data.
   * @param op 读写操作类型 / Type of read/write operation.
   * @param in_isr 是否在中断中进行操作 / Whether the operation is performed in an ISR.
   * @return 操作结果的错误码 / Error code indicating the result of the operation.
   */
  virtual ErrorCode Read(RawData read_data, OperationRW& op, bool in_isr = false)
  {
    return ReadAndWrite(read_data, ConstRawData(nullptr, 0), op, in_isr);
  }

  /**
   * @brief 进行 SPI 写入操作 / Performs SPI write operation.
   * @param write_data 需要写入的数据缓冲区 / Buffer containing the data to be written.
   * @param op 读写操作类型 / Type of read/write operation.
   * @param in_isr 是否在中断中进行操作 / Whether the operation is performed in an ISR.
   * @return 操作结果的错误码 / Error code indicating the result of the operation.
   */
  virtual ErrorCode Write(ConstRawData write_data, OperationRW& op, bool in_isr = false)
  {
    return ReadAndWrite(RawData(nullptr, 0), write_data, op, in_isr);
  }

  /**
   * @brief 设置 SPI 配置参数 / Sets SPI configuration parameters.
   * @param config 需要应用的 SPI 配置 / The SPI configuration to apply.
   * @return 操作结果的错误码 / Error code indicating the result of the operation.
   */
  virtual ErrorCode SetConfig(Configuration config) = 0;

  /**
   * @brief 获取 SPI 设备的最大时钟速度 / Gets the maximum clock speed of the SPI device.
   * @return SPI 设备的最大时钟速度（单位：Hz）。
   * The maximum clock speed of the SPI device (in Hz).
   */
  virtual uint32_t GetMaxBusSpeed() const = 0;

  /**
   * @brief 获取 SPI 设备的最大分频系数 / Gets the maximum prescaler of the SPI device.
   * @return SPI 设备的最大分频系数 / The maximum prescaler of the SPI device.
   */
  virtual Prescaler GetMaxPrescaler() const = 0;

  /**
   * @brief 获取 SPI 设备的当前总线速度 / Gets the current bus speed of the SPI device.
   * @return SPI 设备的当前总线速度（单位：Hz）。
   * The current bus speed of the SPI device (in Hz).
   */
  uint32_t GetBusSpeed() const
  {
    const uint32_t DIV = PrescalerToDiv(config_.prescaler);
    const uint32_t SRC = GetMaxBusSpeed();
    if (DIV == 0u || SRC == 0u)
    {
      return 0u;
    }
    return SRC / DIV;
  }

  /**
   * @brief 计算 SPI 分频系数 / Calculates the SPI prescaler.
   * @param target_max_bus_speed 目标最大总线速度（单位：Hz）。
   *                            Target maximum bus speed (in Hz).
   * @param target_min_bus_speed 目标最小总线速度（单位：Hz）。
   *                            Target minimum bus speed (in Hz).
   * @param increase 是否从最小分频系数开始 / Whether to start from the minimum prescaler.
   * @return 计算得到的分频系数 / The calculated prescaler.
   */
  Prescaler CalcPrescaler(uint32_t target_max_bus_speed, uint32_t target_min_bus_speed,
                          bool increase)
  {
    const uint32_t SRC = GetMaxBusSpeed();
    if (SRC == 0u)
    {
      return Prescaler::UNKNOWN;
    }

    if (target_max_bus_speed && target_min_bus_speed &&
        target_min_bus_speed > target_max_bus_speed)
    {
      uint32_t t = target_min_bus_speed;
      target_min_bus_speed = target_max_bus_speed;
      target_max_bus_speed = t;
    }

    const uint8_t MAX_IDX = static_cast<uint8_t>(GetMaxPrescaler());

    ASSERT(MAX_IDX != static_cast<uint8_t>(Prescaler::UNKNOWN));

    auto fits = [&](Prescaler p) -> bool
    {
      const uint32_t DIV = PrescalerToDiv(p);
      if (DIV == 0u)
      {
        return false;
      }
      const uint32_t F = SRC / DIV;
      if (target_max_bus_speed && F > target_max_bus_speed)
      {
        return false;
      }
      if (target_min_bus_speed && F < target_min_bus_speed)
      {
        return false;
      }
      return true;
    };

    if (increase)
    {
      for (uint8_t i = 0; i <= MAX_IDX; ++i)
      {
        Prescaler p = static_cast<Prescaler>(i);
        if (fits(p))
        {
          return p;
        }
      }
    }
    else
    {
      for (int i = static_cast<int>(MAX_IDX); i >= 0; --i)
      {
        Prescaler p = static_cast<Prescaler>(i);
        if (fits(p))
        {
          return p;
        }
      }
    }

    const uint32_t F_FASTEST = SRC / PrescalerToDiv(Prescaler::DIV_1);
    const Prescaler P_SLOWEST = static_cast<Prescaler>(MAX_IDX);
    const uint32_t F_SLOWEST = SRC / PrescalerToDiv(P_SLOWEST);

    if (target_min_bus_speed && F_FASTEST < target_min_bus_speed)
    {
      return Prescaler::DIV_1;
    }
    if (target_max_bus_speed && F_SLOWEST > target_max_bus_speed)
    {
      return P_SLOWEST;
    }

    if (increase)
    {
      for (uint8_t i = 0; i <= MAX_IDX; ++i)
      {
        Prescaler p = static_cast<Prescaler>(i);
        const uint32_t F = SRC / PrescalerToDiv(p);
        if (!target_max_bus_speed || F <= target_max_bus_speed)
        {
          return p;
        }
      }
      return P_SLOWEST;
    }
    else
    {
      for (int i = static_cast<int>(MAX_IDX); i >= 0; --i)
      {
        Prescaler p = static_cast<Prescaler>(i);
        const uint32_t F = SRC / PrescalerToDiv(p);
        if (!target_min_bus_speed || F >= target_min_bus_speed)
        {
          return p;
        }
      }
      return Prescaler::DIV_1;
    }
  }

  /**
   * @brief 获取接收数据的缓冲区 / Gets the buffer for storing received data.
   * @return 接收数据的缓冲区 / The buffer for storing received data.
   */
  RawData GetRxBuffer()
  {
    if (IsDoubleBuffer())
    {
      return {double_buffer_rx_.ActiveBuffer(), double_buffer_rx_.Size()};
    }
    else
    {
      return rx_buffer_;
    }
  }

  /**
   * @brief 获取发送数据的缓冲区 / Gets the buffer for storing data to be sent.
   * @return 发送数据的缓冲区 / The buffer for storing data to be sent.
   */
  RawData GetTxBuffer()
  {
    if (IsDoubleBuffer())
    {
      return {double_buffer_tx_.ActiveBuffer(), double_buffer_tx_.Size()};
    }
    else
    {
      return tx_buffer_;
    }
  }

  /**
   * @brief 切换缓冲区 / Switches the buffer.
   */
  void SwitchBuffer()
  {
    if (IsDoubleBuffer())
    {
      double_buffer_rx_.Switch();
      double_buffer_tx_.Switch();
    }
  }

  /**
   * @brief 设置缓冲区的有效数据长度 / Sets the length of valid data in the buffer.
   */
  void SetActiveLength(size_t len) { double_buffer_tx_.SetActiveLength(len); }

  /**
   * @brief 获取缓冲区的有效数据长度 / Gets the length of valid data in the buffer.
   */
  size_t GetActiveLength() const { return double_buffer_tx_.GetActiveLength(); }

  /**
   * @brief 进行一次SPI传输（使用当前缓冲区数据，零拷贝，支持双缓冲）。
   *        Performs a SPI transfer (zero-copy, supports double buffering).
   * @param size 需要传输的数据大小 / The size of the data to be transferred.
   * @param op 读写操作类型 / Type of read/write operation.
   * @param in_isr 是否在中断中进行操作 / Whether the operation is performed in an ISR.
   * @return 操作结果的错误码 / Error code indicating the result of the operation.
   */
  virtual ErrorCode Transfer(size_t size, OperationRW& op, bool in_isr = false) = 0;

  /**
   * @brief 向 SPI 设备的寄存器写入数据。
   *        Writes data to a specific register of the SPI device.
   *
   * @param reg 寄存器地址 / Register address.
   * @param write_data 写入的数据缓冲区 / Buffer containing data to write.
   * @param op 操作类型（同步/异步） / Operation mode (sync/async).
   * @param in_isr 是否在中断中进行操作 / Whether the operation is performed in an ISR.
   * @return 操作结果的错误码 / Error code indicating success or failure.
   */
  virtual ErrorCode MemWrite(uint16_t reg, ConstRawData write_data, OperationRW& op,
                             bool in_isr = false) = 0;

  /**
   * @brief 从 SPI 设备的寄存器读取数据。
   *        Reads data from a specific register of the SPI device.
   *
   * @param reg 寄存器地址 / Register address.
   * @param read_data 读取的数据缓冲区 / Buffer to store read data.
   * @param op 操作类型（同步/异步） / Operation mode (sync/async).
   * @param in_isr 是否在中断中进行操作 / Whether the operation is performed in an ISR.
   * @return 操作结果的错误码 / Error code indicating success or failure.
   */
  virtual ErrorCode MemRead(uint16_t reg, RawData read_data, OperationRW& op,
                            bool in_isr = false) = 0;

  /**
   * @brief 获取 SPI 配置参数 / Gets the SPI configuration parameters.
   * @return SPI 配置参数 / The SPI configuration parameters.
   */
  inline Configuration& GetConfig() { return config_; }

  /**
   * @brief 检查是否使用双缓冲区 / Checks if double buffering is enabled.
   *
   * @return true
   * @return false
   */
  inline bool IsDoubleBuffer() const { return config_.double_buffer; }

 private:
  Configuration config_;
  RawData rx_buffer_, tx_buffer_;
  DoubleBuffer double_buffer_rx_, double_buffer_tx_;
};

}  // namespace LibXR
