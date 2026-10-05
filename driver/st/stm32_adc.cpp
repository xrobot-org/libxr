#include "stm32_adc.hpp"

#include <cstddef>
#include <limits>

#include "libxr_def.hpp"
#include "stm32_dcache.hpp"

#ifdef HAL_ADC_MODULE_ENABLED

using namespace LibXR;

using H = ADC_HandleTypeDef*;

#if defined(HAL_ADC_MODULE_ENABLED) && !defined(ADC_CALIB_OFFSET_AND_LINEARITY) && \
    !defined(ADC_CALIB_OFFSET_LINEARITY) && !defined(ADC_CALIB_OFFSET) &&          \
    !defined(ADC_SINGLE_ENDED)
extern "C" HAL_StatusTypeDef
    __attribute__((weak)) HAL_ADCEx_Calibration_Start(ADC_HandleTypeDef* hadc)
{
  (void)hadc;
  return HAL_OK;
}
#endif

namespace
{

uint8_t CalculateFilterSize(RawData dma_buff, std::size_t channel_count)
{
  ASSERT(channel_count > 0U);

  const std::size_t filter_size = dma_buff.size_ / channel_count / sizeof(uint16_t);
  ASSERT(filter_size > 0U);
  ASSERT(filter_size <= std::numeric_limits<uint8_t>::max());
  return static_cast<uint8_t>(filter_size);
}

}  // namespace

STM32ADC::Channel::Channel(STM32ADC* adc, uint8_t index, uint32_t ch)
    : adc_(adc), index_(index), ch_(ch)
{
}

float STM32ADC::Channel::Read() { return adc_->ReadChannel(index_); }

STM32ADC::STM32ADC(ADC_HandleTypeDef* hadc, RawData dma_buff,
                   std::initializer_list<uint32_t> channels, float vref)
    : hadc_(hadc),
      NUM_CHANNELS(channels.size()),
      filter_size_(CalculateFilterSize(dma_buff, channels.size())),
      use_dma_(hadc_->DMA_Handle != nullptr),
      dma_buffer_(dma_buff),
      resolution_(GetADCResolution<ADC_HandleTypeDef>{}.Get(hadc)),
      channels_(new Channel*[NUM_CHANNELS]),
      vref_(vref)
{
  auto it = channels.begin();
  for (uint8_t i = 0; i < NUM_CHANNELS; ++i)
  {
    channels_[i] = new Channel(this, i, *it++);
  }

#if defined(ADC_CALIB_OFFSET_AND_LINEARITY) && defined(ADC_SINGLE_ENDED)
  HAL_ADCEx_Calibration_Start(hadc, ADC_CALIB_OFFSET_AND_LINEARITY, ADC_SINGLE_ENDED);
#elif defined(ADC_CALIB_OFFSET_LINEARITY) && defined(ADC_SINGLE_ENDED)
  HAL_ADCEx_Calibration_Start(hadc, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED);
#elif defined(ADC_CALIB_OFFSET) && defined(ADC_SINGLE_ENDED)
  HAL_ADCEx_Calibration_Start(hadc, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
#elif defined(ADC_SINGLE_ENDED)
  HAL_ADCEx_Calibration_Start(hadc, ADC_SINGLE_ENDED);
#else
  HAL_ADCEx_Calibration_Start(hadc);
#endif

  if (use_dma_)
  {
    /* DMA must be in circular mode */
    AssertContinuousConvModeEnabled<H>(hadc_);
    AssertDMAContReqEnabled<H>(hadc_);
#if !defined(LIBXR_STM32_ADC_GPDMA)
    AssertDMACircular<H>(hadc_);
#endif
    AssertNbrOfConvEq<H>(hadc_, NUM_CHANNELS);
#if defined(LIBXR_STM32_ADC_GPDMA)
    [[maybe_unused]] const auto adc_dma_result =
        gpdma_adapter_.Start(hadc_, reinterpret_cast<uint32_t*>(dma_buffer_.addr_),
                             NUM_CHANNELS * filter_size_, dma_buffer_.size_);
    REQUIRE(adc_dma_result == HAL_OK);
#else
    HAL_ADC_Start_DMA(hadc_, reinterpret_cast<uint32_t*>(dma_buffer_.addr_),
                      NUM_CHANNELS * filter_size_);
#endif
  }
  else
  {
    AssertNbrOfConvEq<H>(hadc_, 1);
    AssertContinuousConvModeDisabled<H>(hadc_);
    HAL_ADC_Start(hadc_);
  }
}

STM32ADC::~STM32ADC()
{
#if defined(LIBXR_STM32_ADC_GPDMA)
  if (use_dma_)
  {
    [[maybe_unused]] const auto adc_dma_result = gpdma_adapter_.Stop(hadc_);
    REQUIRE(adc_dma_result == HAL_OK);
  }
  else
  {
    HAL_ADC_Stop(hadc_);
  }
#else
  use_dma_ ? HAL_ADC_Stop_DMA(hadc_) : HAL_ADC_Stop(hadc_);
#endif
  for (uint8_t i = 0; i < NUM_CHANNELS; ++i)
  {
    delete channels_[i];
  }
  delete[] channels_;
}

STM32ADC::Channel& STM32ADC::GetChannel(uint8_t index) { return *channels_[index]; }

float STM32ADC::ReadChannel(uint8_t channel)
{
  ASSERT(channel < NUM_CHANNELS);

  uint16_t* buffer = reinterpret_cast<uint16_t*>(dma_buffer_.addr_);
  if (use_dma_)
  {
    STM32_InvalidateDCacheByAddr(buffer, filter_size_ * NUM_CHANNELS * 2);
    uint32_t sum = 0;
    for (uint8_t i = 0; i < filter_size_; ++i)
    {
      sum += buffer[channel + i * NUM_CHANNELS];
    }
    return ConvertToVoltage(static_cast<float>(sum) / static_cast<float>(filter_size_));
  }

  ADC_ChannelConfTypeDef config = {};
  uint32_t time = 0;
  UNUSED(time);
#if defined(ADC_SAMPLETIME_16CYCLES)
  time = ADC_SAMPLETIME_16CYCLES;
#elif defined(ADC_SAMPLETIME_16CYCLES_5)
  time = ADC_SAMPLETIME_16CYCLES_5;
#elif defined(ADC_SAMPLETIME_17CYCLES)
  time = ADC_SAMPLETIME_17CYCLES;
#elif defined(ADC_SAMPLETIME_17CYCLES_5)
  time = ADC_SAMPLETIME_17CYCLES_5;
#elif defined(ADC_SAMPLETIME_18CYCLES)
  time = ADC_SAMPLETIME_18CYCLES;
#elif defined(ADC_SAMPLETIME_18CYCLES_5)
  time = ADC_SAMPLETIME_18CYCLES_5;
#elif defined(ADC_SAMPLETIME_19CYCLES)
  time = ADC_SAMPLETIME_19CYCLES;
#elif defined(ADC_SAMPLETIME_19CYCLES_5)
  time = ADC_SAMPLETIME_19CYCLES_5;
#elif defined(ADC_SAMPLETIME_20CYCLES)
  time = ADC_SAMPLETIME_20CYCLES;
#elif defined(ADC_SAMPLETIME_20CYCLES_5)
  time = ADC_SAMPLETIME_20CYCLES_5;
#elif defined(ADC_SAMPLETIME_21CYCLES)
  time = ADC_SAMPLETIME_21CYCLES;
#elif defined(ADC_SAMPLETIME_21CYCLES_5)
  time = ADC_SAMPLETIME_21CYCLES_5;
#elif defined(ADC_SAMPLETIME_22CYCLES)
  time = ADC_SAMPLETIME_22CYCLES;
#elif defined(ADC_SAMPLETIME_22CYCLES_5)
  time = ADC_SAMPLETIME_22CYCLES_5;
#elif defined(ADC_SAMPLETIME_23CYCLES)
  time = ADC_SAMPLETIME_23CYCLES;
#elif defined(ADC_SAMPLETIME_23CYCLES_5)
  time = ADC_SAMPLETIME_23CYCLES_5;
#elif defined(ADC_SAMPLETIME_24CYCLES)
  time = ADC_SAMPLETIME_24CYCLES;
#elif defined(ADC_SAMPLETIME_24CYCLES_5)
  time = ADC_SAMPLETIME_24CYCLES_5;
#elif defined(ADC_SAMPLETIME_25CYCLES)
  time = ADC_SAMPLETIME_25CYCLES;
#elif defined(ADC_SAMPLETIME_25CYCLES_5)
  time = ADC_SAMPLETIME_25CYCLES_5;
#elif defined(ADC_SAMPLETIME_26CYCLES)
  time = ADC_SAMPLETIME_26CYCLES;
#elif defined(ADC_SAMPLETIME_26CYCLES_5)
  time = ADC_SAMPLETIME_26CYCLES_5;
#elif defined(ADC_SAMPLETIME_27CYCLES)
  time = ADC_SAMPLETIME_27CYCLES;
#elif defined(ADC_SAMPLETIME_27CYCLES_5)
  time = ADC_SAMPLETIME_27CYCLES_5;
#elif defined(ADC_SAMPLETIME_28CYCLES)
  time = ADC_SAMPLETIME_28CYCLES;
#elif defined(ADC_SAMPLETIME_28CYCLES_5)
  time = ADC_SAMPLETIME_28CYCLES_5;
#elif defined(ADC_SAMPLETIME_29CYCLES)
  time = ADC_SAMPLETIME_29CYCLES;
#elif defined(ADC_SAMPLETIME_29CYCLES_5)
  time = ADC_SAMPLETIME_29CYCLES_5;
#elif defined(ADC_SAMPLETIME_30CYCLES)
  time = ADC_SAMPLETIME_30CYCLES;
#elif defined(ADC_SAMPLETIME_30CYCLES_5)
  time = ADC_SAMPLETIME_30CYCLES_5;
#elif defined(ADC_SAMPLETIME_31CYCLES)
  time = ADC_SAMPLETIME_31CYCLES;
#elif defined(ADC_SAMPLETIME_31CYCLES_5)
  time = ADC_SAMPLETIME_31CYCLES_5;
#elif defined(ADC_SAMPLETIME_32CYCLES)
  time = ADC_SAMPLETIME_32CYCLES;
#elif defined(ADC_SAMPLETIME_32CYCLES_5)
  time = ADC_SAMPLETIME_32CYCLES_5;
#error "Unsupported sample time"
#endif

  config.Channel = channels_[channel]->ch_;
#if defined(ADC_REGULAR_RANK_1)
  config.Rank = ADC_REGULAR_RANK_1;
#else
  config.Rank = 1;
#endif
#if defined(ADC_SINGLE_ENDED) && !defined(STM32L0)
  config.SingleDiff = ADC_SINGLE_ENDED;
#endif
#if defined(ADC_OFFSET_NONE)
  config.OffsetNumber = ADC_OFFSET_NONE;
  config.Offset = 0;
#endif
#if !defined(STM32L0)
  config.SamplingTime = time;
#endif

  uint32_t expected = 0U;

  // Multiple threads are working on the same adc peripheral
  // Please use dma mode
  const bool locked = locked_.compare_exchange_strong(
      expected, 0xF0F0F0F0U, std::memory_order_acquire, std::memory_order_relaxed);
  REQUIRE(locked);

  HAL_ADC_ConfigChannel(hadc_, &config);

  uint32_t sum = 0;
  for (uint8_t i = 0; i < filter_size_; ++i)
  {
    HAL_ADC_Start(hadc_);
    HAL_ADC_PollForConversion(hadc_, 20);
    buffer[channel + i * NUM_CHANNELS] = HAL_ADC_GetValue(hadc_);
    sum += buffer[channel + i * NUM_CHANNELS];
  }

  locked_.store(0, std::memory_order_release);

  return ConvertToVoltage(static_cast<float>(sum) / static_cast<float>(filter_size_));
}

float STM32ADC::ConvertToVoltage(float adc_value)
{
  return adc_value * vref_ / resolution_;
}

#endif
