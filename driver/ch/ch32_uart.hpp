#pragma once

#include <atomic>

#include "libxr.hpp"
#include DEF2STR(LIBXR_CH32_CONFIG_FILE)

#include "ch32_uart_def.hpp"
#include "double_buffer.hpp"
#include "libxr_def.hpp"
#include "libxr_rw.hpp"
#include "serialized_service.hpp"
#include "uart.hpp"

namespace LibXR
{

/**
 * @brief 使用循环接收 DMA 和双缓冲发送的 CH32 串口
 *        / CH32 UART with circular RX DMA and double-buffered TX.
 *
 * 写请求完整复制到发送缓冲半区后完成，DMA 完成仅释放半区。接收持续运行，
 * 按 DMA 位置变化向 ReadPort 发布数据；软件队列放不下的部分作为接收溢出丢弃。
 * Writes complete once copied into a TX half; DMA completion only releases that half.
 * RX runs continuously and publishes observed DMA bytes to ReadPort, discarding any
 * overflow beyond available software capacity.
 *
 * @pre 调用遵守 ReadPort/WritePort 约定；DMA 缓冲区满足平台访问和对齐要求，运行期间有效。
 *      Follow the port contracts. DMA buffers must meet platform access/alignment
 *      requirements and remain valid throughout operation.
 */
class CH32UART : public UART
{
 public:
  /**
   * @brief 构造串口并关联 DMA 缓冲区 / Construct a UART with DMA buffers.
   * @param id 串口编号 / UART identifier.
   * @param dma_rx 接收 DMA 缓冲区 / RX DMA buffer.
   * @param dma_tx 双缓冲发送存储，每半区容纳一个请求 / TX storage, one request per half.
   * @param tx_gpio_port 发送引脚端口 / TX GPIO port.
   * @param tx_gpio_pin 发送引脚 / TX GPIO pin.
   * @param rx_gpio_port 接收引脚端口 / RX GPIO port.
   * @param rx_gpio_pin 接收引脚 / RX GPIO pin.
   * @param pin_remap 引脚重映射配置，默认零 / Pin remapping, zero by default.
   * @param tx_queue_size 写请求队列容量，启用发送时须大于零 / Positive TX request
   * capacity.
   * @param config 初始串口配置 / Initial UART configuration.
   */
  CH32UART(ch32_uart_id_t id, RawData dma_rx, RawData dma_tx, GPIO_TypeDef* tx_gpio_port,
           uint16_t tx_gpio_pin, GPIO_TypeDef* rx_gpio_port, uint16_t rx_gpio_pin,
           uint32_t pin_remap = 0, uint32_t tx_queue_size = 5,
           UART::Configuration config = {115200, UART::Parity::NO_PARITY, 8, 1});

  /**
   * @brief 提交串口配置请求 / Submit a UART configuration request.
   * @param config 目标配置 / Requested configuration.
   * @param in_isr 当前调用是否在中断中 / Whether this call is in an ISR.
   * @return 接纳返回 OK，配置槽被占用返回 BUSY，参数不支持返回 ARG_ERR 或 NOT_SUPPORT。
   *         OK on admission, BUSY for an occupied slot, ARG_ERR or NOT_SUPPORT for
   *         unsupported settings.
   * @note 接纳后在发送空闲时应用，期间保留已准备的 DMA 数据；不保证返回前已经生效。
   *       Applies when TX is idle while preserving prepared DMA data. May apply after
   * return.
   */
  ErrorCode SetConfig(UART::Configuration config, bool in_isr = false) override;

  /// 通知后端处理已提交的写请求 / Notify the backend of released writes.
  static void WriteFun(WritePort& port, bool in_isr);

  /// 处理发送 DMA 完成通知 / Handle TX DMA completion.
  void TxDmaIRQHandler();
  /// 处理接收 DMA 进展通知 / Handle RX DMA progress.
  void RxDmaIRQHandler();
  /// 处理接收空闲和发送完成中断 / Handle RX idle and TX complete interrupts.
  void UartIRQHandler();

  ch32_uart_id_t id_;  ///< UART 编号 / UART id
  /// 使能的方向，USART_Mode_Rx / USART_Mode_Tx 的组合 / Enabled directions, a
  /// combination of USART_Mode_Rx / USART_Mode_Tx
  uint16_t uart_mode_;

  /// 接收字节队列与读请求 / RX byte queue and read requests.
  ReadPort _read_port;
  /// 写请求及发送字节队列 / Write requests and TX byte queue.
  WritePort _write_port;

  /// 调用者提供的接收 DMA 缓冲区 / Caller-provided RX DMA buffer.
  RawData dma_buff_rx_;
  /// 当前与待发送的 DMA 半区 / Active and pending TX DMA halves.
  DoubleBuffer dma_buff_tx_;

  /// 上次已处理的 DMA 接收位置 / Last processed RX DMA position.
  size_t last_rx_pos_ = 0;

  USART_TypeDef* instance_;  ///< USART 外设寄存器 / USART peripheral registers
  DMA_Channel_TypeDef* dma_rx_channel_;  ///< RX DMA 通道 / RX DMA channel
  DMA_Channel_TypeDef* dma_tx_channel_;  ///< TX DMA 通道 / TX DMA channel

  /// 发送半区是否在使用，仅后端处理器访问 / TX half in use; service-only access.
  Flag::Plain tx_busy_;

  /// 按 UART 编号索引的实例表，供中断服务函数查找 / Instances indexed by UART id, used
  /// by the ISRs to find the driver
  static CH32UART* map_[CH32_UART_NUMBER];

 private:
  /// 待应用配置的发布阶段 / Pending configuration publication phase.
  enum class ConfigState : uint32_t
  {
    EMPTY = 0U,
    RESERVED = 1U,
    PUBLISHED = 2U,
  };

  static constexpr uint32_t TX_EVENT_WRITE = 1U << 0U;
  static constexpr uint32_t TX_EVENT_DMA_DONE = 1U << 1U;
  static constexpr uint32_t TX_EVENT_CONFIG = 1U << 2U;
  static constexpr uint32_t TX_EVENT_TC = 1U << 3U;
  static constexpr uint32_t TX_EVENT_RX_WORK = 1U << 4U;

  /// 串行处理收发和配置通知 / Serialize RX, TX, and configuration progress.
  void HandleTxService(uint32_t events, bool in_isr);
  /// 填充空闲发送半区并启动 DMA / Fill available TX halves and start DMA.
  void FillTx(bool in_isr);
  /// 释放已发送半区，不重复完成写请求 / Release the sent half without completing again.
  void HandleTxDone(bool in_isr);
  /// 启动当前半区发送 / Start transmitting the active half.
  void StartTxDma(bool in_isr);
  /// 应用已验证配置 / Apply validated configuration.
  void ApplyConfig(UART::Configuration config);
  /// 在发送空闲边界尝试应用配置 / Try applying configuration at the TX idle boundary.
  void TryApplyConfig(bool in_isr);
  /// 复制可容纳的接收前缀，更新游标后发布 / Copy fitting RX bytes, advance, then publish.
  void HandleRxData(bool in_isr);

  /// 共享的收发与配置处理器 / Shared RX, TX, and configuration service.
  SerializedService tx_service_;
  /// 配置槽的并发发布状态 / Concurrent configuration slot state.
  std::atomic<ConfigState> config_state_{ConfigState::EMPTY};
  /// 由调用者发布、后端应用的配置 / Caller-published configuration for backend
  /// application.
  UART::Configuration pending_config_{};
};

}  // namespace LibXR
