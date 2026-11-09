#pragma once

#include <atomic>
#include <initializer_list>

#include "adc.hpp"
#include "hpm_adc_v2.h"
#include "hpm_clock_drv.h"
#include "hpm_soc.h"
#include "libxr.hpp"
#include "libxr_def.hpp"

#if __has_include("hpm_adc_v2.h")
#define LIBXR_HPM_ADC_SUPPORTED 1
#else
#define LIBXR_HPM_ADC_SUPPORTED 0
#endif

#if LIBXR_HPM_ADC_SUPPORTED

namespace LibXR
{

class HPMADC
{
 public:
  class Channel : public ADC
  {
   public:
    Channel(HPMADC* adc, uint8_t index, uint8_t ch);

    float Read() override;

    uint8_t ChannelNumber() const { return ch_; }

   private:
    Channel();

    HPMADC* adc_;
    uint8_t index_;
    uint8_t ch_;

    friend class HPMADC;
  };

  HPMADC(adc_v2_handle_t adc, clock_name_t clock, std::initializer_list<uint8_t> channels,
         float vref, uint32_t sample_cycle = 20U,
         adc_v2_resolution_bits_t resolution = adc_v2_resolution_12bit,
         uint8_t clock_divider = 4U, bool auto_board_init = true,
         uint8_t filter_size = 1U);

  Channel& GetChannel(uint8_t index);

  float ReadChannel(uint8_t channel);

  uint16_t GetLastRaw(uint8_t channel) const;

  bool IsValid() const { return valid_; }

 private:
  static float ResolutionMax(adc_v2_resolution_bits_t resolution);

  bool Init(uint32_t sample_cycle, adc_v2_resolution_bits_t resolution,
            uint8_t clock_divider, bool auto_board_init);

  float ConvertToVoltage(float adc_value) const;

  static constexpr uint8_t MAX_CHANNELS =
#ifdef ADC_SOC_MAX_CHANNEL_NUM
      static_cast<uint8_t>(ADC_SOC_MAX_CHANNEL_NUM);
#else
      32U;
#endif

  alignas(LibXR::CONCURRENCY_ALIGNMENT) std::atomic<uint32_t> locked_ = 0U;
  adc_v2_handle_t adc_;
  clock_name_t clock_;
  const uint8_t NUM_CHANNELS;
  Channel channels_[MAX_CHANNELS];
  uint16_t last_raw_[MAX_CHANNELS];
  const uint8_t filter_size_;
  float resolution_;
  float vref_;
  bool valid_;
};

}  // namespace LibXR

#endif
