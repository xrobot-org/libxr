#include "stm32_adc_gpdma.hpp"

#if defined(LIBXR_STM32_ADC_GPDMA) && defined(HAL_ADC_MODULE_ENABLED)

#include <limits>

#include "libxr_def.hpp"
#include "stm32_dcache.hpp"

namespace LibXR
{

HAL_StatusTypeDef STM32GpdmaAdcAdapter::Start(ADC_HandleTypeDef* adc_handle,
                                              uint32_t* buffer, uint32_t sample_count,
                                              size_t buffer_size)
{
  ASSERT(adc_handle != nullptr);
  ASSERT(adc_handle->DMA_Handle != nullptr);
  ASSERT(buffer != nullptr);
  ASSERT(sample_count > 0U);
  ASSERT(sample_count <= std::numeric_limits<size_t>::max() / sizeof(uint16_t));
  ASSERT(buffer_size >= static_cast<size_t>(sample_count) * sizeof(uint16_t));
  DMA_HandleTypeDef* const dma_handle = adc_handle->DMA_Handle;
  ASSERT(dma_handle->Parent == adc_handle);
  ASSERT(IS_GPDMA_INSTANCE(dma_handle->Instance) != 0U);
  ASSERT(dma_handle->Mode == DMA_LINKEDLIST_CIRCULAR);
  ASSERT(dma_handle->InitLinkedList.LinkedListMode == DMA_LINKEDLIST_CIRCULAR);
  ASSERT(dma_handle->InitLinkedList.LinkStepMode == DMA_LSM_FULL_EXECUTION);
  ASSERT((dma_handle->Instance->CCR & DMA_CCR_LSM) == DMA_LSM_FULL_EXECUTION);

  if (state_.original_queue == nullptr)
  {
    BuildAndAttach(adc_handle, buffer, sample_count, buffer_size);
  }
  else
  {
    DEV_ASSERT(dma_handle->LinkedListQueue == &state_.queue);
    DEV_ASSERT(state_.queue.Head == &state_.node);
    DEV_ASSERT(state_.queue.FirstCircularNode == &state_.node);
    DEV_ASSERT(state_.queue.NodeNumber == 1U);
  }

  FinalizeStopped(dma_handle);
  STM32_CleanDCacheByAddr(&state_, sizeof(state_));
  STM32_CleanDCacheByAddr(buffer, buffer_size);
  STM32_InvalidateDCacheByAddr(buffer, buffer_size);

  return HAL_ADC_Start_DMA(adc_handle, buffer, sample_count);
}

HAL_StatusTypeDef STM32GpdmaAdcAdapter::Stop(ADC_HandleTypeDef* adc_handle)
{
  ASSERT(adc_handle != nullptr);
  ASSERT(adc_handle->DMA_Handle != nullptr);
  DMA_HandleTypeDef* const dma_handle = adc_handle->DMA_Handle;
  ASSERT(dma_handle->Parent == adc_handle);

  const HAL_StatusTypeDef stop_status = HAL_ADC_Stop_DMA(adc_handle);
  if (stop_status != HAL_OK)
  {
    return stop_status;
  }

  if (state_.original_queue == nullptr)
  {
    return HAL_OK;
  }

  FinalizeStopped(dma_handle);
  DEV_ASSERT(dma_handle->LinkedListQueue == &state_.queue);
  DEV_ASSERT(state_.queue.State == HAL_DMA_QUEUE_STATE_READY);
  DMA_QListTypeDef* const original_queue = state_.original_queue;
  [[maybe_unused]] const auto dma_ex_list_un_linkq_result =
      HAL_DMAEx_List_UnLinkQ(dma_handle);
  DEV_ASSERT(dma_ex_list_un_linkq_result == HAL_OK);
  [[maybe_unused]] const auto dma_ex_list_linkq_result =
      HAL_DMAEx_List_LinkQ(dma_handle, original_queue);
  DEV_ASSERT(dma_ex_list_linkq_result == HAL_OK);
  DEV_ASSERT(dma_handle->LinkedListQueue == original_queue);
  state_.original_queue = nullptr;
  return HAL_OK;
}

void STM32GpdmaAdcAdapter::FinalizeStopped(DMA_HandleTypeDef* dma_handle)
{
  DEV_ASSERT(dma_handle->State == HAL_DMA_STATE_READY);
  DEV_ASSERT(dma_handle->Lock == HAL_UNLOCKED);
  DEV_ASSERT((dma_handle->Instance->CCR & DMA_CCR_EN) == 0U);
  DEV_ASSERT(dma_handle->LinkedListQueue != nullptr);
  DEV_ASSERT(dma_handle->LinkedListQueue->State == HAL_DMA_QUEUE_STATE_READY);

  // 错误中断可能保留块计数，清零后下次启动才会装载链表头。
  // Error IRQs may retain the block count; clear it so the next start loads the list
  // head.
  __HAL_DMA_DISABLE_IT(dma_handle, DMA_IT_TC | DMA_IT_HT | DMA_IT_DTE | DMA_IT_ULE |
                                       DMA_IT_USE | DMA_IT_SUSP | DMA_IT_TO);
  dma_handle->Instance->CBR1 = 0U;
  __HAL_DMA_CLEAR_FLAG(dma_handle, DMA_FLAG_TC | DMA_FLAG_HT | DMA_FLAG_DTE |
                                       DMA_FLAG_ULE | DMA_FLAG_USE | DMA_FLAG_SUSP |
                                       DMA_FLAG_TO);
  __DSB();
}

void STM32GpdmaAdcAdapter::BuildAndAttach(ADC_HandleTypeDef* adc_handle, uint32_t* buffer,
                                          uint32_t sample_count, size_t buffer_size)
{
  DMA_HandleTypeDef* const dma_handle = adc_handle->DMA_Handle;
  ASSERT(dma_handle->State == HAL_DMA_STATE_READY);
  ASSERT((dma_handle->Instance->CCR & DMA_CCR_EN) == 0U);
  ASSERT(dma_handle->LinkedListQueue != nullptr);
  ASSERT(dma_handle->LinkedListQueue->Head != nullptr);
  ASSERT(dma_handle->LinkedListQueue->FirstCircularNode != nullptr);
  ASSERT(dma_handle->LinkedListQueue->State == HAL_DMA_QUEUE_STATE_READY);

  DMA_NodeConfTypeDef node_config{};
  [[maybe_unused]] const auto dma_ex_list_get_node_config_result =
      HAL_DMAEx_List_GetNodeConfig(&node_config, dma_handle->LinkedListQueue->Head);
  DEV_ASSERT(dma_ex_list_get_node_config_result == HAL_OK);
  ASSERT(node_config.Init.Direction == DMA_PERIPH_TO_MEMORY);
  ASSERT(node_config.Init.SrcInc == DMA_SINC_FIXED);
  ASSERT(node_config.Init.DestInc == DMA_DINC_INCREMENTED);
  ASSERT(node_config.Init.SrcDataWidth == DMA_SRC_DATAWIDTH_HALFWORD);
  ASSERT(node_config.Init.DestDataWidth == DMA_DEST_DATAWIDTH_HALFWORD);
  ASSERT(node_config.Init.SrcBurstLength == 1U);
  ASSERT(node_config.Init.DestBurstLength == 1U);
  ASSERT(node_config.Init.BlkHWRequest == DMA_BREQ_SINGLE_BURST);
  ASSERT(node_config.Init.Mode == DMA_NORMAL);
  ASSERT(node_config.TriggerConfig.TriggerPolarity == DMA_TRIG_POLARITY_MASKED);
  ASSERT(node_config.DataHandlingConfig.DataExchange == DMA_EXCHANGE_NONE);
  ASSERT(sample_count <= std::numeric_limits<uint32_t>::max() / sizeof(uint16_t));

  const uint32_t transfer_size = sample_count * sizeof(uint16_t);
  ASSERT(buffer_size >= transfer_size);
  ASSERT(IS_DMA_BLOCK_SIZE(transfer_size) != 0U);

  node_config.NodeType = DMA_GPDMA_LINEAR_NODE;
  node_config.SrcAddress =
      static_cast<uint32_t>(reinterpret_cast<uintptr_t>(&adc_handle->Instance->DR));
  node_config.DstAddress = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(buffer));
  node_config.DataSize = transfer_size;

  DEV_ASSERT(state_.queue.Head == nullptr);
  DEV_ASSERT(state_.queue.NodeNumber == 0U);
  [[maybe_unused]] const auto dma_ex_list_build_node_result =
      HAL_DMAEx_List_BuildNode(&node_config, &state_.node);
  DEV_ASSERT(dma_ex_list_build_node_result == HAL_OK);
  [[maybe_unused]] const auto dma_ex_list_insert_node_tail_result =
      HAL_DMAEx_List_InsertNode_Tail(&state_.queue, &state_.node);
  DEV_ASSERT(dma_ex_list_insert_node_tail_result == HAL_OK);
  [[maybe_unused]] const auto dma_ex_list_set_circular_mode_result =
      HAL_DMAEx_List_SetCircularMode(&state_.queue);
  DEV_ASSERT(dma_ex_list_set_circular_mode_result == HAL_OK);
  DEV_ASSERT(state_.queue.Head == &state_.node);
  DEV_ASSERT(state_.queue.FirstCircularNode == &state_.node);
  DEV_ASSERT(state_.queue.NodeNumber == 1U);
  DEV_ASSERT(state_.queue.State == HAL_DMA_QUEUE_STATE_READY);

  state_.original_queue = dma_handle->LinkedListQueue;
  [[maybe_unused]] const auto dma_ex_list_un_linkq_result =
      HAL_DMAEx_List_UnLinkQ(dma_handle);
  DEV_ASSERT(dma_ex_list_un_linkq_result == HAL_OK);
  [[maybe_unused]] const auto dma_ex_list_linkq_result =
      HAL_DMAEx_List_LinkQ(dma_handle, &state_.queue);
  DEV_ASSERT(dma_ex_list_linkq_result == HAL_OK);
  DEV_ASSERT(dma_handle->LinkedListQueue == &state_.queue);
}

}  // namespace LibXR

#endif
