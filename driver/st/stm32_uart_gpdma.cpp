#include "stm32_uart_gpdma.hpp"

#if defined(LIBXR_STM32_UART_GPDMA) && defined(HAL_UART_MODULE_ENABLED)

#include "libxr_assert.hpp"
#include "stm32_dcache.hpp"

namespace LibXR
{
namespace
{

IRQn_Type GetGpdmaIrq(DMA_Channel_TypeDef* instance)
{
#define LIBXR_GPDMA_IRQ_CASE(DMA, CHANNEL) \
  if (instance == DMA##_Channel##CHANNEL)  \
  {                                        \
    return DMA##_Channel##CHANNEL##_IRQn;  \
  }

// 按首通道宏判断通道组是否存在；IRQn 是枚举，不能用 defined 检查。
// Detect channel groups by their first channel macro; IRQn names are enum values.
#if defined(GPDMA1_Channel0)
  LIBXR_GPDMA_IRQ_CASE(GPDMA1, 0)
  LIBXR_GPDMA_IRQ_CASE(GPDMA1, 1)
  LIBXR_GPDMA_IRQ_CASE(GPDMA1, 2)
  LIBXR_GPDMA_IRQ_CASE(GPDMA1, 3)
  LIBXR_GPDMA_IRQ_CASE(GPDMA1, 4)
  LIBXR_GPDMA_IRQ_CASE(GPDMA1, 5)
  LIBXR_GPDMA_IRQ_CASE(GPDMA1, 6)
  LIBXR_GPDMA_IRQ_CASE(GPDMA1, 7)
#endif
#if defined(GPDMA1_Channel8)
  LIBXR_GPDMA_IRQ_CASE(GPDMA1, 8)
  LIBXR_GPDMA_IRQ_CASE(GPDMA1, 9)
  LIBXR_GPDMA_IRQ_CASE(GPDMA1, 10)
  LIBXR_GPDMA_IRQ_CASE(GPDMA1, 11)
#endif
#if defined(GPDMA1_Channel12)
  LIBXR_GPDMA_IRQ_CASE(GPDMA1, 12)
  LIBXR_GPDMA_IRQ_CASE(GPDMA1, 13)
  LIBXR_GPDMA_IRQ_CASE(GPDMA1, 14)
  LIBXR_GPDMA_IRQ_CASE(GPDMA1, 15)
#endif
#if defined(GPDMA2_Channel0)
  LIBXR_GPDMA_IRQ_CASE(GPDMA2, 0)
  LIBXR_GPDMA_IRQ_CASE(GPDMA2, 1)
  LIBXR_GPDMA_IRQ_CASE(GPDMA2, 2)
  LIBXR_GPDMA_IRQ_CASE(GPDMA2, 3)
  LIBXR_GPDMA_IRQ_CASE(GPDMA2, 4)
  LIBXR_GPDMA_IRQ_CASE(GPDMA2, 5)
  LIBXR_GPDMA_IRQ_CASE(GPDMA2, 6)
  LIBXR_GPDMA_IRQ_CASE(GPDMA2, 7)
#endif
#if defined(GPDMA2_Channel8)
  LIBXR_GPDMA_IRQ_CASE(GPDMA2, 8)
  LIBXR_GPDMA_IRQ_CASE(GPDMA2, 9)
  LIBXR_GPDMA_IRQ_CASE(GPDMA2, 10)
  LIBXR_GPDMA_IRQ_CASE(GPDMA2, 11)
#endif
#if defined(GPDMA2_Channel12)
  LIBXR_GPDMA_IRQ_CASE(GPDMA2, 12)
  LIBXR_GPDMA_IRQ_CASE(GPDMA2, 13)
  LIBXR_GPDMA_IRQ_CASE(GPDMA2, 14)
  LIBXR_GPDMA_IRQ_CASE(GPDMA2, 15)
#endif

#undef LIBXR_GPDMA_IRQ_CASE

  return NonMaskableInt_IRQn;
}

class GpdmaNvicMaskGuard
{
 public:
  explicit GpdmaNvicMaskGuard(DMA_HandleTypeDef* dma_handle)
      : irq_(GetGpdmaIrq(dma_handle->Instance)),
        valid_(static_cast<int32_t>(irq_) >= 0),
        was_enabled_(valid_ && (NVIC_GetEnableIRQ(irq_) != 0U))
  {
    if (!valid_)
    {
      return;
    }

    if (was_enabled_)
    {
      NVIC_DisableIRQ(irq_);
      __DSB();
      __ISB();
    }
  }

  ~GpdmaNvicMaskGuard()
  {
    if (valid_ && was_enabled_)
    {
      NVIC_EnableIRQ(irq_);
      __DSB();
      __ISB();
    }
  }

  GpdmaNvicMaskGuard(const GpdmaNvicMaskGuard&) = delete;
  GpdmaNvicMaskGuard& operator=(const GpdmaNvicMaskGuard&) = delete;

  [[nodiscard]] bool Valid() const { return valid_; }
  [[nodiscard]] bool WasEnabled() const { return was_enabled_; }

 private:
  IRQn_Type irq_;
  bool valid_;
  bool was_enabled_;
};

}  // namespace

STM32GpdmaUartAdapter::STM32GpdmaUartAdapter(UART_HandleTypeDef* uart_handle)
{
  ASSERT(uart_handle != nullptr);
  ASSERT((uart_handle->hdmatx == nullptr) ||
         (IS_GPDMA_INSTANCE(uart_handle->hdmatx->Instance) != 0U));
  ASSERT((uart_handle->hdmarx == nullptr) ||
         (IS_GPDMA_INSTANCE(uart_handle->hdmarx->Instance) != 0U));
  state_.uart_handle_ = uart_handle;
}

HAL_StatusTypeDef STM32GpdmaUartAdapter::StartLinkedListDmaRx(uint8_t* buffer,
                                                              size_t total_size,
                                                              bool in_isr)
{
  DMA_HandleTypeDef* const dma_handle = state_.uart_handle_->hdmarx;
  ASSERT_FROM_CALLBACK(dma_handle != nullptr, in_isr);
  ASSERT_FROM_CALLBACK(buffer != nullptr, in_isr);
  ASSERT_FROM_CALLBACK(total_size >= RX_NODE_COUNT, in_isr);
  ASSERT_FROM_CALLBACK((total_size % RX_NODE_COUNT) == 0U, in_isr);
  ASSERT_FROM_CALLBACK((total_size / RX_NODE_COUNT) <= UINT16_MAX, in_isr);
  ASSERT_FROM_CALLBACK(IS_DMA_BLOCK_SIZE(total_size / RX_NODE_COUNT) != 0U, in_isr);
  ASSERT_FROM_CALLBACK(dma_handle->Parent == state_.uart_handle_ &&
                           IS_GPDMA_INSTANCE(dma_handle->Instance) != 0U,
                       in_isr);
  ASSERT_FROM_CALLBACK(
      dma_handle->InitLinkedList.LinkStepMode == DMA_LSM_FULL_EXECUTION &&
          dma_handle->InitLinkedList.LinkedListMode == DMA_LINKEDLIST_CIRCULAR &&
          dma_handle->Mode == DMA_LINKEDLIST_CIRCULAR,
      in_isr);

  if (!state_.rx_queue_built_)
  {
    BuildRxQueue(buffer, total_size, in_isr);
  }
  else
  {
    ASSERT_FROM_CALLBACK(buffer == state_.rx_buffer_, in_isr);
    ASSERT_FROM_CALLBACK(total_size == state_.rx_total_size_, in_isr);
    DEV_ASSERT_FROM_CALLBACK(dma_handle->LinkedListQueue == &state_.rx_queue_, in_isr);
  }

  DEV_ASSERT_FROM_CALLBACK(StopComplete(dma_handle), in_isr);
  FinalizeStopped(dma_handle, in_isr);
  // 启动前将描述符与接收缓冲区同步到 DMA 可见的存储。
  // Make descriptors and RX storage visible to DMA before starting the channel.
  STM32_CleanDCacheByAddr(&state_, sizeof(state_));
  STM32_CleanDCacheByAddr(buffer, total_size);
  STM32_InvalidateDCacheByAddr(buffer, total_size);

  return HAL_UARTEx_ReceiveToIdle_DMA(state_.uart_handle_, buffer,
                                      static_cast<uint16_t>(state_.rx_node_size_));
}

uint8_t* STM32GpdmaUartAdapter::GetLinkedListDmaRxProducer() const
{
  DEV_ASSERT(state_.uart_handle_->hdmarx != nullptr);
  const uintptr_t destination = state_.uart_handle_->hdmarx->Instance->CDAR;
  return reinterpret_cast<uint8_t*>(destination);
}

void STM32GpdmaUartAdapter::CloseTxTerminalSource() const
{
  ATOMIC_CLEAR_BIT(state_.uart_handle_->Instance->CR1, USART_CR1_TCIE);
  const volatile uint32_t cr1 = state_.uart_handle_->Instance->CR1;
  UNUSED(cr1);
  __DSB();
}

bool STM32GpdmaUartAdapter::LaunchStop(DMA_HandleTypeDef* dma_handle,
                                       AbortCallback callback,
                                       [[maybe_unused]] bool in_isr)
{
  ASSERT(dma_handle != nullptr);
  ASSERT(dma_handle->Parent == state_.uart_handle_);
  ASSERT(callback != nullptr);

  const auto abort_is_joinable = [&]()
  {
    if (dma_handle->XferAbortCallback == callback)
    {
      return true;
    }

    // 保留 HAL 因线路错误安装的 RX 中止回调，它会向同一处理器报告错误。
    // Preserve HAL's RX line-error abort callback, which notifies the same service.
    return (dma_handle == state_.uart_handle_->hdmarx) &&
           (state_.uart_handle_->ErrorCode != HAL_UART_ERROR_NONE) &&
           (dma_handle->XferAbortCallback != nullptr);
  };

  if (StopComplete(dma_handle))
  {
    return true;
  }

  GpdmaNvicMaskGuard irq_guard(dma_handle);
  if (!irq_guard.Valid() || !irq_guard.WasEnabled())
  {
    return false;
  }

  // 屏蔽该 DMA 中断后重新检查，覆盖首次检查后已完成的中止。
  // Recheck after masking the DMA IRQ in case it completed after the first check.
  if (StopComplete(dma_handle))
  {
    return true;
  }
  if (dma_handle->State == HAL_DMA_STATE_ABORT)
  {
    return abort_is_joinable();
  }
  if (dma_handle->State == HAL_DMA_STATE_READY && IsStopped(dma_handle) &&
      dma_handle->Lock == HAL_LOCKED)
  {
    // HAL IRQ 已停止通道，尚未解锁并回调；由该回调继续推进。
    // The HAL IRQ has stopped the channel but has not unlocked or notified yet.
    return true;
  }
  if (dma_handle->State != HAL_DMA_STATE_BUSY)
  {
    return false;
  }

  AbortCallback const previous_callback = dma_handle->XferAbortCallback;
  const HAL_StatusTypeDef result = HAL_DMA_Abort_IT(dma_handle);
  if (result == HAL_OK)
  {
    // DMA 中断仍被屏蔽；只有 HAL 未替换回调时才安装本后端回调。
    // With the DMA IRQ masked, install our callback only if HAL did not replace it.
    if (dma_handle->XferAbortCallback == previous_callback)
    {
      dma_handle->XferAbortCallback = callback;
      return true;
    }

    return abort_is_joinable();
  }

  return ((dma_handle->State == HAL_DMA_STATE_ABORT) && abort_is_joinable()) ||
         StopComplete(dma_handle);
}

bool STM32GpdmaUartAdapter::StopComplete(DMA_HandleTypeDef* dma_handle)
{
  if ((dma_handle == nullptr) || !IsStopped(dma_handle) ||
      (dma_handle->State != HAL_DMA_STATE_READY) || (dma_handle->Lock != HAL_UNLOCKED))
  {
    return false;
  }
  if ((dma_handle->Mode & DMA_LINKEDLIST) == DMA_LINKEDLIST)
  {
    return (dma_handle->LinkedListQueue != nullptr) &&
           (dma_handle->LinkedListQueue->State == HAL_DMA_QUEUE_STATE_READY);
  }
  return true;
}

bool STM32GpdmaUartAdapter::AllStopsComplete() const
{
  return ((state_.uart_handle_->hdmatx == nullptr) ||
          StopComplete(state_.uart_handle_->hdmatx)) &&
         ((state_.uart_handle_->hdmarx == nullptr) ||
          StopComplete(state_.uart_handle_->hdmarx));
}

void STM32GpdmaUartAdapter::FinalizeStopped(DMA_HandleTypeDef* dma_handle, bool in_isr)
{
  DEV_ASSERT_FROM_CALLBACK(StopComplete(dma_handle), in_isr);
  DEV_ASSERT_FROM_CALLBACK(dma_handle->Lock == HAL_UNLOCKED, in_isr);

  DisableInterrupts(dma_handle);
  if ((dma_handle->Mode & DMA_LINKEDLIST) == DMA_LINKEDLIST)
  {
    // 错误中断可能保留 CBR1；清零后下次启动才能从链表头装载。
    // Error IRQs may leave CBR1 set; clear it so the next start reloads the list head.
    dma_handle->Instance->CBR1 = 0U;
  }
  const volatile uint32_t ccr = dma_handle->Instance->CCR;
  UNUSED(ccr);
  __DSB();
  ClearFlags(dma_handle);
  __DSB();
}

bool STM32GpdmaUartAdapter::IsStopped(DMA_HandleTypeDef* dma_handle)
{
  const uint32_t ccr = dma_handle->Instance->CCR;
  return (ccr & DMA_CCR_EN) == 0U;
}

void STM32GpdmaUartAdapter::DisableInterrupts(DMA_HandleTypeDef* dma_handle)
{
  __HAL_DMA_DISABLE_IT(dma_handle, DMA_IT_TC | DMA_IT_HT | DMA_IT_DTE | DMA_IT_ULE |
                                       DMA_IT_USE | DMA_IT_SUSP | DMA_IT_TO);
}

void STM32GpdmaUartAdapter::ClearFlags(DMA_HandleTypeDef* dma_handle)
{
  __HAL_DMA_CLEAR_FLAG(dma_handle, DMA_FLAG_TC | DMA_FLAG_HT | DMA_FLAG_DTE |
                                       DMA_FLAG_ULE | DMA_FLAG_USE | DMA_FLAG_SUSP |
                                       DMA_FLAG_TO);
}

void STM32GpdmaUartAdapter::BuildRxQueue(uint8_t* buffer, size_t total_size, bool in_isr)
{
  DMA_HandleTypeDef* const dma_handle = state_.uart_handle_->hdmarx;
  DEV_ASSERT_FROM_CALLBACK(StopComplete(dma_handle), in_isr);
  ASSERT_FROM_CALLBACK(dma_handle->LinkedListQueue != nullptr, in_isr);
  ASSERT_FROM_CALLBACK(dma_handle->LinkedListQueue->Head != nullptr, in_isr);

  DMA_NodeConfTypeDef node_config{};
  [[maybe_unused]] const auto dma_ex_list_get_node_config_result =
      HAL_DMAEx_List_GetNodeConfig(&node_config, dma_handle->LinkedListQueue->Head);
  DEV_ASSERT_FROM_CALLBACK(dma_ex_list_get_node_config_result == HAL_OK, in_isr);

  // 保留 BSP 的请求与端口等属性，明确设置字节接收环的布局。
  // Preserve BSP request and port attributes while defining the byte RX ring layout.
  node_config.NodeType = DMA_GPDMA_LINEAR_NODE;
  node_config.Init.BlkHWRequest = DMA_BREQ_SINGLE_BURST;
  node_config.Init.Direction = DMA_PERIPH_TO_MEMORY;
  node_config.Init.SrcInc = DMA_SINC_FIXED;
  node_config.Init.DestInc = DMA_DINC_INCREMENTED;
  node_config.Init.SrcDataWidth = DMA_SRC_DATAWIDTH_BYTE;
  node_config.Init.DestDataWidth = DMA_DEST_DATAWIDTH_BYTE;
  node_config.Init.SrcBurstLength = 1U;
  node_config.Init.DestBurstLength = 1U;
  node_config.Init.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
  node_config.Init.Mode = DMA_NORMAL;
  node_config.DataHandlingConfig.DataExchange = DMA_EXCHANGE_NONE;
  node_config.DataHandlingConfig.DataAlignment = DMA_DATA_RIGHTALIGN_ZEROPADDED;
  node_config.TriggerConfig.TriggerMode = DMA_TRIGM_BLOCK_TRANSFER;
  node_config.TriggerConfig.TriggerPolarity = DMA_TRIG_POLARITY_MASKED;
  node_config.TriggerConfig.TriggerSelection = 0U;
  node_config.RepeatBlockConfig = {};

  const size_t node_size = total_size / RX_NODE_COUNT;
  node_config.SrcAddress = static_cast<uint32_t>(
      reinterpret_cast<uintptr_t>(&state_.uart_handle_->Instance->RDR));
  node_config.DataSize = static_cast<uint32_t>(node_size);

  for (size_t i = 0U; i < RX_NODE_COUNT; ++i)
  {
    node_config.DstAddress =
        static_cast<uint32_t>(reinterpret_cast<uintptr_t>(&buffer[i * node_size]));
    [[maybe_unused]] const auto dma_ex_list_build_node_result =
        HAL_DMAEx_List_BuildNode(&node_config, &state_.nodes_[i]);
    DEV_ASSERT_FROM_CALLBACK(dma_ex_list_build_node_result == HAL_OK, in_isr);
    [[maybe_unused]] const auto dma_ex_list_insert_node_tail_result =
        HAL_DMAEx_List_InsertNode_Tail(&state_.rx_queue_, &state_.nodes_[i]);
    DEV_ASSERT_FROM_CALLBACK(dma_ex_list_insert_node_tail_result == HAL_OK, in_isr);
  }

  [[maybe_unused]] const auto dma_ex_list_set_circular_mode_result =
      HAL_DMAEx_List_SetCircularMode(&state_.rx_queue_);
  DEV_ASSERT_FROM_CALLBACK(dma_ex_list_set_circular_mode_result == HAL_OK, in_isr);
  [[maybe_unused]] const auto dma_ex_list_un_linkq_result =
      HAL_DMAEx_List_UnLinkQ(dma_handle);
  DEV_ASSERT_FROM_CALLBACK(dma_ex_list_un_linkq_result == HAL_OK, in_isr);
  [[maybe_unused]] const auto dma_ex_list_linkq_result =
      HAL_DMAEx_List_LinkQ(dma_handle, &state_.rx_queue_);
  DEV_ASSERT_FROM_CALLBACK(dma_ex_list_linkq_result == HAL_OK, in_isr);
  DEV_ASSERT_FROM_CALLBACK(dma_handle->LinkedListQueue == &state_.rx_queue_, in_isr);

  state_.rx_buffer_ = buffer;
  state_.rx_total_size_ = total_size;
  state_.rx_node_size_ = node_size;
  state_.rx_queue_built_ = true;
}

}  // namespace LibXR

#endif
