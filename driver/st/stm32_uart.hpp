#pragma once

#include <atomic>

#include "main.h"
#include "stm32_uart_gpdma.hpp"

#ifdef HAL_UART_MODULE_ENABLED

#ifdef UART
#undef UART
#endif

#include "double_buffer.hpp"
#include "flag.hpp"
#include "libxr_def.hpp"
#include "libxr_rw.hpp"
#include "serialized_service.hpp"
#include "uart.hpp"

typedef enum : uint8_t
{
#ifdef USART1
  STM32_USART1,
#endif
#ifdef USART2
  STM32_USART2,
#endif
#ifdef USART3
  STM32_USART3,
#endif
#ifdef USART4
  STM32_USART4,
#endif
#ifdef USART5
  STM32_USART5,
#endif
#ifdef USART6
  STM32_USART6,
#endif
#ifdef USART7
  STM32_USART7,
#endif
#ifdef USART8
  STM32_USART8,
#endif
#ifdef USART9
  STM32_USART9,
#endif
#ifdef USART10
  STM32_USART10,
#endif
#ifdef USART11
  STM32_USART11,
#endif
#ifdef USART12
  STM32_USART12,
#endif
#ifdef USART13
  STM32_USART13,
#endif
#ifdef UART1
  STM32_UART1,
#endif
#ifdef UART2
  STM32_UART2,
#endif
#ifdef UART3
  STM32_UART3,
#endif
#ifdef UART4
  STM32_UART4,
#endif
#ifdef UART5
  STM32_UART5,
#endif
#ifdef UART6
  STM32_UART6,
#endif
#ifdef UART7
  STM32_UART7,
#endif
#ifdef UART8
  STM32_UART8,
#endif
#ifdef UART9
  STM32_UART9,
#endif
#ifdef UART10
  STM32_UART10,
#endif
#ifdef UART11
  STM32_UART11,
#endif
#ifdef UART12
  STM32_UART12,
#endif
#ifdef UART13
  STM32_UART13,
#endif
#ifdef LPUART1
  STM32_LPUART1,
#endif
#ifdef LPUART2
  STM32_LPUART2,
#endif
#ifdef LPUART3
  STM32_LPUART3,
#endif
  STM32_UART_NUMBER,
  STM32_UART_ID_ERROR
} stm32_uart_id_t;

stm32_uart_id_t stm32_uart_get_id(USART_TypeDef* addr);

namespace LibXR
{
/**
 * @brief 使用循环接收 DMA 和双缓冲发送的 STM32 串口
 *        / STM32 UART with circular RX DMA and double-buffered TX.
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
 * @pre H5 的 RX 句柄已挂接静止的循环链表，缓冲区长度是 4 的倍数，每段不超过
 *      65535 字节；TX 使用普通单次 DMA。相关 UART/DMA 中断具有相同抢占优先级。
 *      On H5, RX starts with a stopped circular seed list. The buffer size is a multiple
 *      of four, with at most 65535 bytes per node; TX uses normal one-shot DMA.
 *      Related UART/DMA IRQs share a preemption priority.
 */
class STM32UART : public UART
{
 public:
  /// 通知后端处理已提交的写请求 / Notify the backend of released writes.
  static void WriteFun(WritePort& port, bool in_isr);

  /**
   * @brief 构造串口并关联 DMA 缓冲区 / Construct a UART with DMA buffers.
   * @param uart_handle 已初始化的 HAL 句柄 / Initialized HAL handle.
   * @param dma_buff_rx 接收 DMA 缓冲区 / RX DMA buffer.
   * @param dma_buff_tx 双缓冲发送存储，每半区容纳一个请求 / TX storage, one request per
   * half.
   * @param tx_queue_size 写请求队列容量，启用发送时须大于零 / Positive TX request
   * capacity.
   */
  STM32UART(UART_HandleTypeDef* uart_handle, RawData dma_buff_rx, RawData dma_buff_tx,
            uint32_t tx_queue_size = 5);

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

  /// 初始化循环接收 DMA 并检查启动结果 / Initialize circular RX DMA and check startup.
  void SetRxDMA(bool in_isr = false);
  /// 通知发送完成，交给后端释放半区 / Notify TX completion to release the active half.
  void TxCompleteIRQHandler();
  /// 通知接收位置变化 / Notify RX position changes.
  void RxEventIRQHandler();
  /// 通知错误，由后端处理器启动中止 / Report an error; the service initiates the abort.
  void ErrorIRQHandler();
  /// 通知 HAL 中止完成，由后端恢复收发 / Notify HAL abort completion for backend
  /// recovery.
  void AbortCompleteIRQHandler();

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

  /// HAL UART 句柄 / HAL UART handle
  UART_HandleTypeDef* uart_handle_;

  /// 发送半区是否在使用，仅后端处理器访问 / TX half in use; service-only access.
  Flag::Plain tx_busy_;

  /// 外设编号，同时是 map 的下标 / Peripheral ID, also the index into map
  stm32_uart_id_t id_ = STM32_UART_ID_ERROR;

  /// 外设编号到实例的映射，供 HAL 回调找回对象 / Instance table by peripheral ID, used
  /// by HAL callbacks to find the object
  static STM32UART* map[STM32_UART_NUMBER];  // NOLINT

 private:
  /// 待应用配置的发布阶段 / Pending configuration publication phase.
  enum class ConfigState : uint32_t
  {
    EMPTY = 0U,
    RESERVED = 1U,
    PUBLISHED = 2U,
  };

  static constexpr uint32_t TX_EVENT_WRITE = 1U << 0U;
  static constexpr uint32_t TX_EVENT_DONE = 1U << 1U;
  static constexpr uint32_t TX_EVENT_CONFIG = 1U << 2U;
  static constexpr uint32_t TX_EVENT_RX_WORK = 1U << 3U;
  static constexpr uint32_t TX_EVENT_ABORT = 1U << 4U;
  static constexpr uint32_t TX_EVENT_ERROR = 1U << 5U;
#if defined(LIBXR_STM32_UART_GPDMA)
  static constexpr uint32_t TX_EVENT_START_RX = 1U << 6U;
  static void DmaAbortCallback(DMA_HandleTypeDef* dma_handle);
#endif

  /// 启动或接续硬件中止，由串行处理器调用 / Start or join a hardware abort from the
  /// service.
  void BeginAbort(bool in_isr);

  /// 串行处理收发和配置通知 / Serialize RX, TX, and configuration progress.
  void HandleTxService(uint32_t events, bool in_isr);
  /// 填充空闲发送半区并启动 DMA / Fill available TX halves and start DMA.
  void FillTx(bool in_isr);
  /// 释放已发送半区，不重复完成写请求 / Release the sent half without completing again.
  void HandleTxDone(bool in_isr);
  /// 启动当前半区发送 / Start transmitting the active half.
  void StartTxDma(bool in_isr);
  /// 应用已验证配置并检查 HAL 结果 / Apply validated settings and check HAL results.
  void ApplyConfig(UART::Configuration config, bool in_isr);
  /// 在发送空闲边界尝试应用配置 / Try applying configuration at the TX idle boundary.
  void TryApplyConfig(bool in_isr);
  /// 复制可容纳的接收前缀，更新游标后发布 / Copy fitting RX bytes, advance, then publish.
  void HandleRxData(bool in_isr);

  /// 共享的收发与配置处理器 / Shared RX, TX, and configuration service.
  SerializedService tx_service_;
  /// 中止完成前保留发送半区并暂停硬件操作，仅处理器访问。
  /// Retain the TX half and defer hardware operations until abort completion;
  /// service-only.
  bool abort_pending_ = false;
  /// 配置槽的并发发布状态 / Concurrent configuration slot state.
  std::atomic<ConfigState> config_state_{ConfigState::EMPTY};
  /// 由调用者发布、后端应用的配置 / Caller-published configuration for backend
  /// application.
  UART::Configuration pending_config_{};
#if defined(LIBXR_STM32_UART_GPDMA)
  STM32GpdmaUartAdapter gpdma_adapter_;
#endif
};

}  // namespace LibXR

#endif
