#pragma once

#include <cstddef>
#include <cstdint>

#include "main.h"

#if defined(STM32H5) && defined(HAL_ADC_MODULE_ENABLED) && defined(HAL_DMA_MODULE_ENABLED)
#if !defined(DMA_LINKEDLIST_CIRCULAR) || !defined(DMA_GPDMA_LINEAR_NODE)
#error "STM32H5 ADC requires the HAL GPDMA linked-list API"
#endif
#define LIBXR_STM32_ADC_GPDMA 1
#endif

#if defined(LIBXR_STM32_ADC_GPDMA) || defined(__DOXYGEN__)
namespace LibXR
{
/**
 * @brief STM32 ADC 循环采样的 GPDMA 适配器 / GPDMA adapter for circular STM32 ADC
 * sampling
 *
 * 持有一个线性循环节点，采样缓冲区由调用者提供；启动时替换 BSP 链表，停止后恢复。
 * Owns one linear circular node while borrowing sample storage; replaces the BSP list
 * on startup and restores it after stopping.
 * @note 由所属 STM32ADC 在构造时启动、析构时停止，期间独占对应 ADC/DMA 句柄。
 *       Started by the owning STM32ADC constructor and stopped by its destructor,
 *       with exclusive ADC/DMA handle ownership throughout that lifetime.
 */
class STM32GpdmaAdcAdapter
{
 public:
  STM32GpdmaAdcAdapter() = default;

  /**
   * @brief 挂接循环链表并启动 ADC DMA / Attach the circular list and start ADC DMA.
   * @param adc_handle 已关联静止循环链表的 ADC 句柄 / ADC handle with a stopped circular
   * list.
   * @param buffer DMA 可访问的半字采样缓冲区 / DMA-accessible halfword sample storage.
   * @param sample_count 每轮采样数，单位为半字 / Samples per cycle, in halfwords.
   * @param buffer_size 缓冲区字节数 / Available buffer size in bytes.
   * @return HAL 启动结果 / HAL startup result.
   */
  HAL_StatusTypeDef Start(ADC_HandleTypeDef* adc_handle, uint32_t* buffer,
                          uint32_t sample_count, size_t buffer_size);

  /**
   * @brief 停止 ADC DMA 并恢复 BSP 链表 / Stop ADC DMA and restore the BSP list.
   * @param adc_handle 启动时使用的 ADC 句柄 / ADC handle used at startup.
   * @return HAL 停止结果；失败时保留私有链表 / HAL stop result; retain the private list
   * on failure.
   */
  HAL_StatusTypeDef Stop(ADC_HandleTypeDef* adc_handle);

 private:
  // DMA 读取的节点寄存器按 32 字节对齐，避免跨越链表地址窗口。
  // Align the DMA node registers to 32 bytes to avoid crossing the list address window.
  struct alignas(32) StateBlock
  {
    DMA_NodeTypeDef node{};
    DMA_QListTypeDef queue{};
    DMA_QListTypeDef* original_queue = nullptr;
  };
  static_assert(offsetof(StateBlock, node) == 0U);

  void BuildAndAttach(ADC_HandleTypeDef* adc_handle, uint32_t* buffer,
                      uint32_t sample_count, size_t buffer_size);
  static void FinalizeStopped(DMA_HandleTypeDef* dma_handle);
  StateBlock state_{};
};
}  // namespace LibXR
#endif
