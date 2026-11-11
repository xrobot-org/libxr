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
 * @brief ESP32 ADC 单元驱动 / ESP32 ADC unit driver
 *
 * 优先使用连续 DMA 采样；该单元不支持连续模式时退回单次采样。每个通道通过 GetChannel()
 * 取得一个 ADC 接口对象。
 * Prefers continuous DMA sampling and falls back to one-shot sampling when the unit has
 * no continuous mode. Each channel is exposed as an ADC object through GetChannel().
 */
class ESP32ADC
{
 public:
  /// 单个 ADC 通道，读数委托给所属的 ESP32ADC / One ADC channel; reads are delegated to
  /// the owning ESP32ADC
  class Channel : public ADC
  {
   public:
    Channel();
    /**
     * @brief 构造绑定到某个 ESP32ADC 的通道 / Construct a channel bound to an ESP32ADC
     * @param parent 所属 ADC 单元 / Owning ADC unit
     * @param idx 通道在 channels 数组中的下标 / Index in the constructor's channel array
     * @param channel_num 硬件通道号（adc_channel_t）/ Hardware channel number
     *        (adc_channel_t)
     */
    Channel(ESP32ADC* parent, uint8_t idx, uint8_t channel_num);

    float Read() override;

    /// 硬件通道号 / Hardware channel number
    uint8_t ChannelNumber() const { return channel_num_; }

   private:
    ESP32ADC* parent_;
    uint8_t idx_;
    uint8_t channel_num_;
  };

  /// 连续 DMA 模式的初始化结果 / Result of initializing continuous DMA mode
  enum class ContinuousInitResult : uint8_t
  {
    STARTED = 0,  ///< 连续采样已启动 / Continuous sampling started
    UNSUPPORTED,  ///< 该单元不支持，改用单次采样 / Unsupported, use one-shot instead
    FAILED,       ///< 初始化失败 / Initialization failed
  };

  /**
   * @brief 构造 ADC 单元并启动采样 / Construct the ADC unit and start sampling
   * @param unit ADC 单元 / ADC unit
   * @param channels 通道号数组，元素互不相同 / Channel array, entries must be distinct
   * @param num_channels channels 的元素个数，大于 0 / Number of entries in channels, > 0
   * @param freq 连续模式采样率（Hz），超出芯片范围时被限幅 / Continuous-mode sample rate
   *        (Hz), clamped to the chip range
   * @param attenuation 输入衰减 / Input attenuation
   * @param bitwidth 采样位宽，ADC_BITWIDTH_DEFAULT 取芯片最大位宽 / Sample bit width;
   *        ADC_BITWIDTH_DEFAULT selects the chip maximum
   * @param reference_voltage 满量程对应的参考电压（V），大于 0 / Reference voltage (V) at
   *        full scale, > 0
   * @param dma_buf_size 连续模式 DMA 缓冲区大小（字节）/ Continuous-mode DMA buffer size
   *        (bytes)
   */
  ESP32ADC(
      adc_unit_t unit, const adc_channel_t* channels, uint8_t num_channels,
      uint32_t freq = SOC_ADC_SAMPLE_FREQ_THRES_LOW,
      adc_atten_t attenuation = ADC_ATTEN_DB_12,
      adc_bitwidth_t bitwidth = static_cast<adc_bitwidth_t>(SOC_ADC_DIGI_MAX_BITWIDTH),
      float reference_voltage = 3.3f, size_t dma_buf_size = 256);

  /**
   * @brief 取得第 idx 个通道对象 / Get the channel object at index idx
   * @param idx 通道下标，小于 num_channels / Channel index, below num_channels
   * @return 通道引用 / Channel reference
   */
  Channel& GetChannel(uint8_t idx);

  /**
   * @brief 读取第 idx 个通道的电压 / Read the voltage of channel idx
   * @param idx 通道下标，小于 num_channels / Channel index, below num_channels
   * @return 电压（V）；单元未就绪或读取失败时为 0 / Voltage (V); 0 when the unit is not
   *         ready or the read fails
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
