#pragma once

#include <cstddef>
#include <cstdint>

#include "main.h"

#if defined(STM32H5)
#define LIBXR_STM32_UART_GPDMA 1
#endif

#if (defined(LIBXR_STM32_UART_GPDMA) && defined(HAL_UART_MODULE_ENABLED)) || \
    defined(__DOXYGEN__)

#if !defined(HAL_DMA_MODULE_ENABLED) || !defined(DMA_LINKEDLIST_CIRCULAR) || \
    !defined(DMA_GPDMA_LINEAR_NODE) || !defined(IS_GPDMA_INSTANCE)
#error "STM32H5 UART requires the HAL GPDMA linked-list API"
#endif

#if (defined(STM32H503xx) || defined(STM32H523xx) || defined(STM32H533xx) || \
     defined(STM32H543xx) || defined(STM32H553xx) || defined(STM32H562xx) || \
     defined(STM32H563xx) || defined(STM32H573xx)) &&                        \
    !defined(USART_DMAREQUESTS_SW_WA)
#error "STM32H5 UART requires the HAL USART DMA-request workaround"
#endif

namespace LibXR
{
/**
 * @brief STM32 GPDMA 串口适配器 / STM32 GPDMA UART adapter
 *
 * 持有四个线性接收节点与循环链表，数据缓冲区由调用者提供。
 * Owns four linear RX nodes and their circular list; the caller supplies the payload.
 * @pre UART、DMA 句柄及缓冲区在传输期间有效且由本后端独占；相关 UART/DMA
 *      中断具有相同抢占优先级，运行中的 DMA 中断必须使能。
 *      UART/DMA handles and buffers remain valid and exclusively owned by this backend.
 *      Related UART/DMA IRQs share a preemption priority; active DMA IRQs must be
 * enabled.
 */
class STM32GpdmaUartAdapter
{
 public:
  static constexpr size_t RX_NODE_COUNT = 4U;
  using AbortCallback = void (*)(DMA_HandleTypeDef*);

  /// 绑定已初始化的 UART 句柄 / Bind an initialized UART handle.
  explicit STM32GpdmaUartAdapter(UART_HandleTypeDef* uart_handle);

  /// 构建或复用四节点循环链表并启动 RX / Build or reuse the four-node ring and start RX.
  HAL_StatusTypeDef StartLinkedListDmaRx(uint8_t* buffer, size_t total_size, bool in_isr);
  /// 读取一次当前 DMA 写入地址 / Read the current DMA destination address once.
  [[nodiscard]] uint8_t* GetLinkedListDmaRxProducer() const;
  /// 关闭 TX 完成中断，保留 DMA 请求位 / Disable TX completion IRQ, retaining DMA
  /// requests.
  void CloseTxTerminalSource() const;
  /// 启动异步中止或接续已有中止 / Start an asynchronous abort or join an existing abort.
  [[nodiscard]] bool LaunchStop(DMA_HandleTypeDef* dma_handle, AbortCallback callback,
                                bool in_isr);
  /// 检查通道及 HAL 链表均已停止 / Check that the channel and HAL list have stopped.
  [[nodiscard]] static bool StopComplete(DMA_HandleTypeDef* dma_handle);
  /// 检查所有已配置的 DMA 通道均已停止 / Check all configured DMA channels have stopped.
  [[nodiscard]] bool AllStopsComplete() const;
  /// 在停止状态清除旧标志和链表计数 / Clear stale flags and list count while stopped.
  static void FinalizeStopped(DMA_HandleTypeDef* dma_handle, bool in_isr);

 private:
  // 节点保持在同一 64 KiB 链表地址窗口内。
  // Keep all nodes within one 64 KiB linked-list address window.
  struct alignas(256) StateBlock
  {
    DMA_NodeTypeDef nodes_[RX_NODE_COUNT];
    DMA_QListTypeDef rx_queue_{};
    UART_HandleTypeDef* uart_handle_ = nullptr;
    uint8_t* rx_buffer_ = nullptr;
    size_t rx_total_size_ = 0U;
    size_t rx_node_size_ = 0U;
    bool rx_queue_built_ = false;
  };
  static_assert(offsetof(StateBlock, nodes_) == 0U);
  static_assert(sizeof(StateBlock) == 256U);

  [[nodiscard]] static bool IsStopped(DMA_HandleTypeDef* dma_handle);
  static void DisableInterrupts(DMA_HandleTypeDef* dma_handle);
  static void ClearFlags(DMA_HandleTypeDef* dma_handle);
  void BuildRxQueue(uint8_t* buffer, size_t total_size, bool in_isr);
  StateBlock state_{};
};
static_assert(sizeof(STM32GpdmaUartAdapter) == 256U);
}  // namespace LibXR
#endif
