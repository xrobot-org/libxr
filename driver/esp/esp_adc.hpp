#pragma once

#include <cstddef>
#include <cstdint>

#include "adc.hpp"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_def.hpp"
#include "hal/adc_types.h"
#include "soc/soc_caps.h"

#if SOC_ADC_DIG_CTRL_SUPPORTED && SOC_ADC_DMA_SUPPORTED
#include "esp_adc/adc_continuous.h"
#endif

struct adc_oneshot_hal_ctx_t;

namespace LibXR
{

/**
 * @class ESP32ADC
 * @brief ESP32 ADC 驱动包装类，支持连续 DMA 与单次转换回退
 *        ESP32 ADC driver wrapper supporting continuous DMA with oneshot fallback.
 */
class ESP32ADC
{
 public:
  /**
   * @class Channel
   * @brief 绑定到单个 ESP32 ADC 通道的 LibXR ADC 接口实现
   *        LibXR ADC channel adapter bound to one ESP32 ADC channel.
   */
  class Channel : public ADC
  {
   public:
    /// @brief 构造未绑定通道占位对象 / Construct an unbound channel placeholder.
    Channel();
    /**
     * @brief 构造绑定到指定 ESP32ADC 通道的适配器 / Construct an adapter bound to an
     *        `ESP32ADC` channel.
     * @param parent 所属 `ESP32ADC` 实例 / Owning `ESP32ADC` instance.
     * @param idx 通道在父实例中的索引 / Channel index within `parent`.
     * @param channel_num ESP-IDF ADC 通道号 / ESP-IDF ADC channel number.
     */
    Channel(ESP32ADC* parent, uint8_t idx, uint8_t channel_num);

    float Read() override;

    /**
     * @brief 获取对应的硬件 ADC 通道号 / Get the hardware ADC channel number.
     * @return ESP-IDF ADC 通道号 / ESP-IDF ADC channel number.
     */
    uint8_t ChannelNumber() const { return channel_num_; }

   private:
    ESP32ADC* parent_;
    uint8_t idx_;
    uint8_t channel_num_;
  };

  /**
   * @brief 连续 DMA 采样初始化结果 / Continuous DMA initialization result.
   */
  enum class ContinuousInitResult : uint8_t
  {
    STARTED = 0,  ///< 已启动连续 DMA 模式 / Continuous DMA started.
    UNSUPPORTED,  ///< 当前单元/参数不支持连续模式 / Continuous mode unsupported.
    FAILED,       ///< 初始化失败 / Initialization failed.
  };

  /**
   * @brief 构造 ESP32ADC 管理器并配置采样通道 / Construct ESP32ADC manager and configure
   *        channels.
   * @param unit ADC 硬件单元 / ADC hardware unit.
   * @param channels ADC 通道号数组 / Array of ADC channel numbers.
   * @param num_channels 通道数量 / Number of channels.
   * @param freq 采样频率（Hz） / Sample frequency in Hz.
   * @param attenuation 输入衰减配置 / Input attenuation configuration.
   * @param bitwidth 采样位宽 / Sample resolution bitwidth.
   * @param reference_voltage 参考电压（V） / Reference voltage in volts.
   * @param dma_buf_size DMA 缓冲区大小 / DMA buffer size.
   */
  ESP32ADC(
      adc_unit_t unit, const adc_channel_t* channels, uint8_t num_channels,
      uint32_t freq = SOC_ADC_SAMPLE_FREQ_THRES_LOW,
      adc_atten_t attenuation = ADC_ATTEN_DB_12,
      adc_bitwidth_t bitwidth = static_cast<adc_bitwidth_t>(SOC_ADC_DIGI_MAX_BITWIDTH),
      float reference_voltage = 3.3f, size_t dma_buf_size = 256);

  /**
   * @brief 按索引获取通道适配器引用 / Get the channel adapter reference by index.
   * @param idx 通道索引 / Channel index.
   * @return 通道对象引用 / Reference to the `Channel` object.
   */
  Channel& GetChannel(uint8_t idx);

  /**
   * @brief 读取指定索引通道的电压值 / Read voltage of the channel at the given index.
   * @param idx 通道索引 / Channel index.
   * @return 换算后的电压值（V） / Converted voltage in volts.
   */
  float ReadChannel(uint8_t idx);

 private:
  enum class Backend : uint8_t
  {
    NONE = 0,
    CONTINUOUS_DMA,
    ONESHOT,
  };

  static adc_bitwidth_t ResolveBitwidth(adc_bitwidth_t bitwidth);
  float Normalize(float raw) const;
  static uint32_t ClampSampleFreq(uint32_t freq);
  static uint32_t AlignUp(uint32_t value, uint32_t align);
  bool IsValidChannel(adc_channel_t channel) const;
  void ConfigureAnalogPad(adc_channel_t channel) const;
  bool InitCalibration();
  bool InitOneshot();
  ContinuousInitResult InitContinuous(uint32_t freq, size_t dma_buf_size);
  float RawToVoltage(uint8_t idx, uint16_t raw) const;

#if SOC_ADC_DIG_CTRL_SUPPORTED && SOC_ADC_DMA_SUPPORTED
  static bool IsDigiUnitSupported(adc_unit_t unit);
  static adc_digi_output_format_t ResolveContinuousFormat();
  void DrainContinuousFrames(uint32_t timeout_ms);
#endif

  Channel* channels_ = nullptr;
  adc_channel_t* channel_ids_ = nullptr;
  uint8_t channel_idx_map_[SOC_ADC_MAX_CHANNEL_NUM] = {};
  bool* channel_ready_ = nullptr;
  float* latest_values_ = nullptr;
  uint16_t* latest_raw_ = nullptr;

  adc_cali_handle_t cali_handles_[SOC_ADC_MAX_CHANNEL_NUM] = {};

  adc_unit_t unit_;
  uint8_t num_channels_;
  adc_atten_t attenuation_;
  adc_bitwidth_t bitwidth_;
  float reference_voltage_;
  uint16_t max_raw_;
  static constexpr uint8_t INVALID_CHANNEL_IDX = 0xFFU;

  Backend backend_ = Backend::NONE;
  adc_oneshot_hal_ctx_t* oneshot_hal_ = nullptr;
  bool oneshot_inited_ = false;
  bool valid_ = false;

#if SOC_ADC_DIG_CTRL_SUPPORTED && SOC_ADC_DMA_SUPPORTED
  adc_continuous_handle_t continuous_handle_ = nullptr;
  uint8_t* continuous_read_buf_ = nullptr;
  adc_continuous_data_t* continuous_parsed_buf_ = nullptr;
  uint32_t continuous_frame_size_ = 0U;
  uint32_t continuous_prime_timeout_ms_ = 0U;
#endif
};

}  // namespace LibXR
