#include "esp_adc.hpp"

#include "esp_adc/adc_cali_scheme.h"
#include "esp_clk_tree.h"
#include "esp_private/adc_private.h"
#include "esp_private/adc_share_hw_ctrl.h"
#include "esp_private/esp_clk_tree_common.h"
#include "esp_private/gpio.h"
#include "esp_private/sar_periph_ctrl.h"
#include "hal/adc_hal_common.h"
#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3
#define sens_dev_t sens_dev_s
#include "hal/adc_oneshot_hal.h"
#undef sens_dev_t
#else
#include "hal/adc_oneshot_hal.h"
#endif

#if defined(CONFIG_IDF_TARGET_ESP32) && CONFIG_IDF_TARGET_ESP32 && \
    defined(CONFIG_ESP_WIFI_ENABLED) && CONFIG_ESP_WIFI_ENABLED
#include "esp_wifi.h"
#endif

#ifndef ANALOG_CLOCK_ENABLE
#define ANALOG_CLOCK_ENABLE() ((void)0)
#endif
#ifndef ANALOG_CLOCK_DISABLE
#define ANALOG_CLOCK_DISABLE() ((void)0)
#endif

extern portMUX_TYPE rtc_spinlock;

namespace LibXR
{

namespace
{

constexpr uint32_t DEFAULT_LINE_FITTING_VREF_MV = 1100U;

}  // namespace

ESP32ADC::Channel::Channel() : parent_(nullptr), idx_(0), channel_num_(0) {}

ESP32ADC::Channel::Channel(ESP32ADC* parent, uint8_t idx, uint8_t channel_num)
    : parent_(parent), idx_(idx), channel_num_(channel_num)
{
}

float ESP32ADC::Channel::Read() { return parent_ ? parent_->ReadChannel(idx_) : 0.f; }

ESP32ADC::ESP32ADC(adc_unit_t unit, const adc_channel_t* channels, uint8_t num_channels,
                   uint32_t freq, adc_atten_t attenuation, adc_bitwidth_t bitwidth,
                   float reference_voltage, size_t dma_buf_size)
    : unit_(unit),
      num_channels_(num_channels),
      attenuation_(attenuation),
      bitwidth_(ResolveBitwidth(bitwidth)),
      reference_voltage_(reference_voltage),
      max_raw_((1U << static_cast<uint8_t>(bitwidth_)) - 1U)
{
  ASSERT(channels != nullptr);
  ASSERT(num_channels_ > 0U);
  ASSERT(reference_voltage_ > 0.0f);
  ASSERT(max_raw_ != 0U);
  ASSERT(num_channels_ <= SOC_ADC_MAX_CHANNEL_NUM);
  if ((channels == nullptr) || (num_channels_ == 0U) || (reference_voltage_ <= 0.0f) ||
      (max_raw_ == 0U) || (num_channels_ > SOC_ADC_MAX_CHANNEL_NUM))
  {
    return;
  }

  channels_ = new Channel[num_channels_];
  channel_ids_ = new adc_channel_t[num_channels_];
  channel_ready_ = new bool[num_channels_];
  latest_values_ = new float[num_channels_];
  latest_raw_ = new uint16_t[num_channels_];

  for (uint8_t i = 0; i < SOC_ADC_MAX_CHANNEL_NUM; ++i)
  {
    channel_idx_map_[i] = INVALID_CHANNEL_IDX;
    cali_handles_[i] = nullptr;
  }

  for (uint8_t i = 0; i < num_channels_; ++i)
  {
    ASSERT(IsValidChannel(channels[i]));
    const uint8_t ch = static_cast<uint8_t>(channels[i]);
    ASSERT(channel_idx_map_[ch] == INVALID_CHANNEL_IDX);
    if (!IsValidChannel(channels[i]) || (channel_idx_map_[ch] != INVALID_CHANNEL_IDX))
    {
      return;
    }

    channels_[i] = Channel(this, i, ch);
    channel_ids_[i] = channels[i];
    channel_idx_map_[ch] = i;
    channel_ready_[i] = false;
    latest_values_[i] = 0.0f;
    latest_raw_[i] = 0U;
  }

#if defined(CONFIG_IDF_TARGET_ESP32) && CONFIG_IDF_TARGET_ESP32 && \
    defined(CONFIG_ESP_WIFI_ENABLED) && CONFIG_ESP_WIFI_ENABLED
  if (unit_ == ADC_UNIT_2)
  {
    wifi_mode_t wifi_mode = WIFI_MODE_NULL;
    const esp_err_t wifi_mode_err = esp_wifi_get_mode(&wifi_mode);
    const bool wifi_active = (wifi_mode_err == ESP_OK) && (wifi_mode != WIFI_MODE_NULL);
    ASSERT(!wifi_active);
    if (wifi_active)
    {
      return;
    }
  }
#endif

  (void)InitCalibration();

  const ContinuousInitResult cont_ans = InitContinuous(freq, dma_buf_size);
  if (cont_ans == ContinuousInitResult::STARTED)
  {
    valid_ = true;
    return;
  }

  if (cont_ans == ContinuousInitResult::FAILED)
  {
    REQUIRE(false);
    return;
  }

  const bool oneshot_ok = InitOneshot();
  REQUIRE(oneshot_ok);
  if (!oneshot_ok)
  {
    return;
  }

  valid_ = true;
}

ESP32ADC::Channel& ESP32ADC::GetChannel(uint8_t idx)
{
  ASSERT(idx < num_channels_);
  return channels_[idx];
}

float ESP32ADC::ReadChannel(uint8_t idx)
{
  ASSERT(valid_);
  ASSERT(idx < num_channels_);
  if (!valid_ || (idx >= num_channels_))
  {
    return 0.f;
  }

  if (backend_ == Backend::CONTINUOUS_DMA)
  {
#if SOC_ADC_DIG_CTRL_SUPPORTED && SOC_ADC_DMA_SUPPORTED
    if (!channel_ready_[idx])
    {
      DrainContinuousFrames(continuous_prime_timeout_ms_);
    }
    else
    {
      DrainContinuousFrames(0U);
    }
#endif
    return latest_values_[idx];
  }

  DEV_ASSERT(backend_ == Backend::ONESHOT);
  DEV_ASSERT(oneshot_inited_ && (oneshot_hal_ != nullptr));
  if (!oneshot_inited_ || (oneshot_hal_ == nullptr))
  {
    return 0.f;
  }

  const esp_err_t lock_err = adc_lock_try_acquire(unit_);

  if (lock_err != ESP_OK)
  {
    return 0.f;
  }

  int raw = 0;
  bool converted = false;
  bool clk_src_enabled = false;

  portENTER_CRITICAL(&rtc_spinlock);

#if SOC_ADC_DIG_CTRL_SUPPORTED && !SOC_ADC_RTC_CTRL_SUPPORTED
  const esp_err_t clk_on =
      esp_clk_tree_enable_src(static_cast<soc_module_clk_t>(oneshot_hal_->clk_src), true);

  clk_src_enabled = (clk_on == ESP_OK);
#else
  clk_src_enabled = true;
#endif

  if (clk_src_enabled)
  {
    ANALOG_CLOCK_ENABLE();
    adc_oneshot_hal_setup(oneshot_hal_, channel_ids_[idx]);
#if SOC_ADC_CALIBRATION_V1_SUPPORTED
    adc_set_hw_calibration_code(unit_, attenuation_);
#endif
    converted = adc_oneshot_hal_convert(oneshot_hal_, &raw);
    ANALOG_CLOCK_DISABLE();
  }

#if SOC_ADC_DIG_CTRL_SUPPORTED && !SOC_ADC_RTC_CTRL_SUPPORTED
  if (clk_src_enabled)
  {
    const esp_err_t clk_off = esp_clk_tree_enable_src(
        static_cast<soc_module_clk_t>(oneshot_hal_->clk_src), false);

    if (clk_off != ESP_OK)
    {
      converted = false;
    }
  }
#endif

  portEXIT_CRITICAL(&rtc_spinlock);

  const esp_err_t unlock_err = adc_lock_release(unit_);

  if (unlock_err != ESP_OK)
  {
    return 0.f;
  }

  if (!converted)
  {
    return 0.f;
  }

  latest_raw_[idx] = static_cast<uint16_t>(raw);
  latest_values_[idx] = RawToVoltage(idx, latest_raw_[idx]);
  channel_ready_[idx] = true;
  return latest_values_[idx];
}

adc_bitwidth_t ESP32ADC::ResolveBitwidth(adc_bitwidth_t bitwidth)
{
  if (bitwidth == ADC_BITWIDTH_DEFAULT)
  {
    return static_cast<adc_bitwidth_t>(SOC_ADC_RTC_MAX_BITWIDTH);
  }
  return bitwidth;
}

bool ESP32ADC::InitCalibration()
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
  bool calibrated = false;

  for (uint8_t i = 0; i < num_channels_; ++i)
  {
    adc_cali_curve_fitting_config_t config = {};
    config.unit_id = unit_;
    config.chan = channel_ids_[i];
    config.atten = attenuation_;
    config.bitwidth = bitwidth_;

    adc_cali_handle_t handle = nullptr;
    const esp_err_t err = adc_cali_create_scheme_curve_fitting(&config, &handle);
    if (err != ESP_OK)
    {
      continue;
    }

    cali_handles_[i] = handle;
    calibrated = true;
  }

  return calibrated;
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
  adc_cali_line_fitting_config_t config = {};
  config.unit_id = unit_;
  config.atten = attenuation_;
  config.bitwidth = bitwidth_;
#if CONFIG_IDF_TARGET_ESP32
  config.default_vref = DEFAULT_LINE_FITTING_VREF_MV;
#endif

  adc_cali_handle_t handle = nullptr;
  const esp_err_t err = adc_cali_create_scheme_line_fitting(&config, &handle);
  if (err != ESP_OK)
  {
    return false;
  }

  for (uint8_t i = 0; i < num_channels_; ++i)
  {
    cali_handles_[i] = handle;
  }

  return true;
#else
  return false;
#endif
}

float ESP32ADC::RawToVoltage(uint8_t idx, uint16_t raw) const
{
  ASSERT(idx < num_channels_);
  if (idx >= num_channels_)
  {
    return 0.f;
  }

  if (cali_handles_[idx] != nullptr)
  {
    int voltage_mv = 0;
    const esp_err_t err =
        adc_cali_raw_to_voltage(cali_handles_[idx], static_cast<int>(raw), &voltage_mv);

    if (err == ESP_OK)
    {
      return static_cast<float>(voltage_mv) / 1000.0f;
    }
  }

  return Normalize(static_cast<float>(raw));
}

float ESP32ADC::Normalize(float raw) const
{
  return (raw / static_cast<float>(max_raw_)) * reference_voltage_;
}

uint32_t ESP32ADC::ClampSampleFreq(uint32_t freq)
{
  if (freq < SOC_ADC_SAMPLE_FREQ_THRES_LOW)
  {
    return SOC_ADC_SAMPLE_FREQ_THRES_LOW;
  }
  if (freq > SOC_ADC_SAMPLE_FREQ_THRES_HIGH)
  {
    return SOC_ADC_SAMPLE_FREQ_THRES_HIGH;
  }
  return freq;
}

uint32_t ESP32ADC::AlignUp(uint32_t value, uint32_t align)
{
  if (align == 0U)
  {
    return value;
  }
  return (value + align - 1U) / align * align;
}

bool ESP32ADC::IsValidChannel(adc_channel_t channel) const
{
  const int channel_count = SOC_ADC_CHANNEL_NUM(static_cast<int>(unit_));
  const int channel_num = static_cast<int>(channel);
  return (channel_count > 0) && (channel_num >= 0) && (channel_num < channel_count);
}

void ESP32ADC::ConfigureAnalogPad(adc_channel_t channel) const
{
  int io_num = -1;
  if ((adc_channel_to_io(unit_, channel, &io_num) == ESP_OK) && (io_num >= 0))
  {
    (void)gpio_config_as_analog(static_cast<gpio_num_t>(io_num));
  }
}

}  // namespace LibXR
