#pragma once

#include "libxr.hpp"
#include "libxr_rw.hpp"

namespace LibXR
{

/**
 * @class UART
 * @brief 通用异步收发传输（UART）基类 / Abstract base class for Universal Asynchronous
 * Receiver-Transmitter (UART)
 *
 * 该类定义了 UART 设备的基本接口，包括配置和数据传输端口。
 * This class defines the basic interface for a UART device, including configuration and
 * data transmission ports.
 */
class UART
{
 public:
  /**
   * @enum Parity
   * @brief 奇偶校验模式 / Parity mode
   *
   * 指定 UART 传输时的奇偶校验模式。
   * Specifies the parity mode used in UART transmission.
   */
  enum class Parity : uint8_t
  {
    NO_PARITY = 0,  ///< 无校验 / No parity
    EVEN = 1,       ///< 偶校验 / Even parity
    ODD = 2         ///< 奇校验 / Odd parity
  };

  /**
   * @struct Configuration
   * @brief UART 配置结构体 / UART configuration structure
   *
   * 该结构体包含 UART 端口的基本配置参数，如波特率、数据位、停止位等。
   * This structure contains basic configuration parameters for the UART port, such as
   * baud rate, data bits, and stop bits.
   */
  struct Configuration
  {
    uint32_t baudrate;  ///< 波特率 / Baud rate
    // TODO: Mark, Space
    Parity parity;  ///< 校验模式 / Parity mode
    /// 有效数据位数。按字节接收时仅低 data_bits 位有效，高位由调用方按需屏蔽。
    /// Number of valid data bits. For byte-wise RX, only the low data_bits bits are
    /// valid; callers mask the unspecified upper bits when needed.
    uint8_t data_bits;
    // TODO: 0.5 1.5
    uint8_t stop_bits;  ///< 停止位长度 / Number of stop bits
  };

  ReadPort* read_port_;    ///< 读取端口 / Read port
  WritePort* write_port_;  ///< 写入端口 / Write port

  /**
   * @brief UART 构造函数 / UART constructor
   * @param read_port 接收端口 / Read port
   * @param write_port 发送端口 / Write port
   *
   * 该构造函数初始化 UART 的读取和写入端口。
   * This constructor initializes the read and write ports of the UART.
   */
  template <typename ReadPortType = ReadPort, typename WritePortType = WritePort>
  UART(ReadPortType* read_port, WritePortType* write_port)
      : read_port_(read_port), write_port_(write_port)
  {
  }

  /**
   * @brief 设置 UART 配置 / Sets the UART configuration
   * @param config UART 配置信息 / UART configuration settings
   * @param in_isr 调用者是否位于中断上下文 / Whether the caller is in an ISR.
   * @note 配置应用时机和允许的调用环境由具体后端规定。
   *       Application timing and supported call contexts depend on the backend.
   * @return 返回操作状态，成功时返回 `ErrorCode::OK`，否则返回相应错误码 / Returns the
   * operation status, `ErrorCode::OK` if successful, otherwise an error code
   *
   * 该方法为纯虚函数，子类必须实现具体的 UART 配置逻辑。
   * This is a pure virtual function. Subclasses must implement the specific UART
   * configuration logic.
   */
  virtual ErrorCode SetConfig(Configuration config, bool in_isr = false) = 0;

  /**
   * @brief 向 UART 写入数据 / Write data to the UART interface.
   * @tparam OperationType 写操作类型，需继承自 `WriteOperation` / Write operation type
   *         derived from `WriteOperation`.
   * @param data 待发送的只读数据缓冲区 / Read-only buffer containing data to write.
   * @param op 异步或同步完成通知操作对象 / Completion operation descriptor.
   * @param in_isr 是否在中断上下文中调用 / Whether invoked from an ISR.
   * @return 操作结果错误码 / Error code indicating status.
   */
  template <typename OperationType, typename = std::enable_if_t<std::is_base_of_v<
                                        WriteOperation, std::decay_t<OperationType>>>>
  ErrorCode Write(ConstRawData data, OperationType&& op, bool in_isr = false)
  {
    return (*write_port_)(data, std::forward<OperationType>(op), in_isr);
  }

  /**
   * @brief 从 UART 读取数据 / Read data from the UART interface.
   * @tparam OperationType 读操作类型，需继承自 `ReadOperation` / Read operation type
   *         derived from `ReadOperation`.
   * @param data 用于接收数据的缓冲区 / Destination buffer for received data.
   * @param op 异步或同步完成通知操作对象 / Completion operation descriptor.
   * @param in_isr 是否在中断上下文中调用 / Whether invoked from an ISR.
   * @return 操作结果错误码 / Error code indicating status.
   */
  template <typename OperationType, typename = std::enable_if_t<std::is_base_of_v<
                                        ReadOperation, std::decay_t<OperationType>>>>
  ErrorCode Read(RawData data, OperationType&& op, bool in_isr = false)
  {
    return (*read_port_)(data, std::forward<OperationType>(op), in_isr);
  }
};

}  // namespace LibXR
