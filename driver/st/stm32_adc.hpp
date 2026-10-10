#pragma once

#include <initializer_list>

#include "main.h"

#ifdef HAL_ADC_MODULE_ENABLED

#ifdef ADC
#undef ADC
#endif

#include "adc.hpp"
#include "libxr.hpp"
#include "libxr_def.hpp"
#include "stm32_adc_gpdma.hpp"

namespace LibXR
{

/**
 * @brief STM32 ADC 驱动实现 / STM32 ADC driver implementation
 */
class STM32ADC
{
  template <typename, typename = void>
  struct GetADCResolution
  {
    float Get(ADC_HandleTypeDef* hadc)
    {
      UNUSED(hadc);
      return 4095.0f;
    }
  };

  template <typename T>
  struct GetADCResolution<T, std::void_t<decltype(std::declval<T>().Init.Resolution)>>
  {
    float Get(T* hadc)
    {
      switch (hadc->Init.Resolution)
      {
#ifdef ADC_RESOLUTION_16B
        case ADC_RESOLUTION_16B:
          return 65535.0f;
#endif
#ifdef ADC_RESOLUTION_12B
        case ADC_RESOLUTION_12B:
          return 4095.0f;
#endif
#ifdef ADC_RESOLUTION_10B
        case ADC_RESOLUTION_10B:
          return 1023.0f;
#endif
#ifdef ADC_RESOLUTION_8B
        case ADC_RESOLUTION_8B:
          return 255.0f;
#endif
#ifdef ADC_RESOLUTION_6B
        case ADC_RESOLUTION_6B:
          return 63.0f;
#endif
        default:
          return 4095.0f;
      }
    }
  };

  template <typename T, typename = void>
  struct HasContinuousConvMode : std::false_type
  {
  };

  template <typename T>
  struct HasContinuousConvMode<
      T, std::void_t<decltype(std::declval<T>()->Init.ContinuousConvMode)>>
      : std::true_type
  {
  };

  template <typename T, typename = void>
  struct HasDMAContinuousRequests : std::false_type
  {
  };

  template <typename T>
  struct HasDMAContinuousRequests<
      T, std::void_t<decltype(std::declval<T>()->Init.DMAContinuousRequests)>>
      : std::true_type
  {
  };

  template <typename T, typename = void>
  struct HasNbrOfConversion : std::false_type
  {
  };

  template <typename T>
  struct HasNbrOfConversion<
      T, std::void_t<decltype(std::declval<T>()->Init.NbrOfConversion)>> : std::true_type
  {
  };

  template <typename T, typename = void>
  struct HasDMACircularMode : std::false_type
  {
  };

  template <typename T>
  struct HasDMACircularMode<
      T, std::void_t<decltype(std::declval<T>()->DMA_Handle->Init.Mode)>> : std::true_type
  {
  };

  template <typename T, typename = void>
  struct HasConversionDataManagement : std::false_type
  {
  };

  template <typename T>
  struct HasConversionDataManagement<
      T, std::void_t<decltype(std::declval<T>()->Init.ConversionDataManagement)>>
      : std::true_type
  {
  };

  template <typename T>
  static typename std::enable_if<HasContinuousConvMode<T>::value>::type
  AssertContinuousConvModeEnabled(T hadc)
  {
    ASSERT(hadc->Init.ContinuousConvMode == ENABLE);
  }

  template <typename T>
  static typename std::enable_if<!HasContinuousConvMode<T>::value>::type
  AssertContinuousConvModeEnabled(T)
  {
  }

  template <typename T>
  static typename std::enable_if<HasContinuousConvMode<T>::value>::type
  AssertContinuousConvModeDisabled(T hadc)
  {
    ASSERT(hadc->Init.ContinuousConvMode == DISABLE);
  }

  template <typename T>
  static typename std::enable_if<!HasContinuousConvMode<T>::value>::type
  AssertContinuousConvModeDisabled(T)
  {
  }

  // 有 ConversionDataManagement 的时候，只看 ConversionDataManagement ==
  // ADC_CONVERSIONDATA_DMA_CIRCULAR
  template <typename T>
  static typename std::enable_if<HasConversionDataManagement<T>::value>::type
  AssertDMAContReqEnabled(T hadc)
  {
#ifdef ADC_CONVERSIONDATA_DMA_CIRCULAR
    ASSERT(hadc->Init.ConversionDataManagement == ADC_CONVERSIONDATA_DMA_CIRCULAR);
#else
    UNUSED(hadc);
#endif
  }

  // 没有 ConversionDataManagement，但有 DMAContinuousRequests => 维持原有检查
  template <typename T>
  static typename std::enable_if<!HasConversionDataManagement<T>::value &&
                                 HasDMAContinuousRequests<T>::value>::type
  AssertDMAContReqEnabled(T hadc)
  {
    ASSERT(hadc->Init.DMAContinuousRequests == ENABLE);
  }

  template <typename T>
  static typename std::enable_if<!HasConversionDataManagement<T>::value &&
                                 !HasDMAContinuousRequests<T>::value>::type
  AssertDMAContReqEnabled(T)
  {
  }

  template <typename T>
  static typename std::enable_if<HasNbrOfConversion<T>::value>::type AssertNbrOfConvEq(
      T hadc, uint32_t n)
  {
    ASSERT(hadc->Init.NbrOfConversion == n);
  }

  template <typename T>
  static typename std::enable_if<!HasNbrOfConversion<T>::value>::type AssertNbrOfConvEq(
      T, uint32_t)
  {
  }

  template <typename T>
  static typename std::enable_if<HasDMACircularMode<T>::value>::type AssertDMACircular(
      T hadc)
  {
    ASSERT(hadc->DMA_Handle != nullptr);
#if defined(DMA_CIRCULAR)
    ASSERT(hadc->DMA_Handle->Init.Mode == DMA_CIRCULAR);
#else
    static_assert(sizeof(T) == 0, "DMA_CIRCULAR is not defined by the HAL");
#endif
  }

  template <typename T>
  static typename std::enable_if<!HasDMACircularMode<T>::value>::type AssertDMACircular(T)
  {
  }

 public:
  /**
   * @brief ADC 通道对象 / ADC channel object
   */
  class Channel : public ADC
  {
   public:
    /**
     * @brief 构造通道对象 / Construct channel object
     *
     * @param adc 所属 ADC 对象 / Parent ADC object
     * @param index 通道索引 / Channel index
     * @param ch 通道号 / Channel number
     */
    Channel(STM32ADC* adc, uint8_t index, uint32_t ch);

    /**
     * @brief 读取 ADC 电压值 / Read ADC voltage
     *
     * @return float 电压值 / Voltage value
     */
    float Read() override;

   private:
    Channel();
    STM32ADC* adc_;
    uint8_t index_;
    uint32_t ch_;

    friend class STM32ADC;
  };

  /**
   * @brief 构造 ADC 驱动对象 / Construct ADC driver object
   *
   * @param hadc HAL ADC 句柄 / HAL ADC handle
   * @param dma_buff DMA 缓冲区 / DMA buffer
   * @param channels ADC 通道列表 / ADC channel list
   * @param vref 参考电压 / Reference voltage
   * @note H5 DMA 采样使用半字传输和循环链表，缓冲区、句柄及原链表在对象生命周期内有效。
   *       H5 DMA sampling uses halfword transfers and a circular list; buffers, handles
   *       and the original list remain valid for the object's lifetime.
   */
  STM32ADC(ADC_HandleTypeDef* hadc, RawData dma_buff,
           std::initializer_list<uint32_t> channels, float vref);

  /**
   * @brief 析构函数 / Destructor
   */
  ~STM32ADC();

  /**
   * @brief 获取 ADC 通道对象 / Get ADC channel object
   *
   * @param index 通道索引 / Channel index
   * @return Channel& 通道对象引用 / Channel object reference
   */
  Channel& GetChannel(uint8_t index);

  /**
   * @brief 读取指定通道电压 / Read channel voltage
   *
   * @param channel 通道号 / Channel number
   * @return float 电压值 / Voltage value
   */
  float ReadChannel(uint8_t channel);

 private:
  alignas(LibXR::CONCURRENCY_ALIGNMENT) std::atomic<uint32_t> locked_ = 0U;
  ADC_HandleTypeDef* hadc_;
  const uint8_t NUM_CHANNELS;
  uint8_t filter_size_;
  bool use_dma_;
  RawData dma_buffer_;
  float resolution_;
  Channel** channels_;
  float vref_;
#if defined(LIBXR_STM32_ADC_GPDMA)
  STM32GpdmaAdcAdapter gpdma_adapter_;
#endif

  float ConvertToVoltage(float adc_value);
};

}  // namespace LibXR

#endif
