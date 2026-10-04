#include "esp_i2c.hpp"

#include <algorithm>
#include <array>

#include "esp_clk_tree.h"
#include "esp_private/periph_ctrl.h"
#include "esp_rom_gpio.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "libxr_def.hpp"
#include "timebase.hpp"

namespace LibXR
{
namespace
{

constexpr uint8_t ACK_VALUE = I2C_MASTER_ACK;
constexpr uint8_t NACK_VALUE = I2C_MASTER_NACK;
constexpr uint8_t CHECK_ACK = 1U;
constexpr uint8_t NO_CHECK_ACK = 0U;

uint64_t ToTimeoutUs(uint32_t timeout_ms)
{
  return (timeout_ms == UINT32_MAX) ? UINT64_MAX
                                    : static_cast<uint64_t>(timeout_ms) * 1000ULL;
}

uint64_t GetNowUs() { return static_cast<uint64_t>(Timebase::GetMicroseconds()); }

inline void SetBusClockAtomic(i2c_port_t port, bool enable)
{
#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#endif
  PERIPH_RCC_ATOMIC() { i2c_ll_enable_bus_clock(port, enable); }
#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif
}

inline void ResetBusRegisterAtomic(i2c_port_t port)
{
#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#endif
  PERIPH_RCC_ATOMIC() { i2c_ll_reset_register(port); }
#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif
}

void WriteCommand(i2c_dev_t* dev, int cmd_idx, uint8_t op_code, uint8_t ack_val,
                  uint8_t ack_exp, uint8_t ack_en, uint8_t byte_num)
{
  i2c_ll_hw_cmd_t cmd = {};
  cmd.op_code = op_code;
  cmd.ack_val = ack_val;
  cmd.ack_exp = ack_exp;
  cmd.ack_en = ack_en;
  cmd.byte_num = byte_num;
  i2c_ll_master_write_cmd_reg(dev, cmd, cmd_idx);
}

ErrorCode WaitSegmentDone(i2c_hal_context_t& hal, int done_cmd_idx, uint64_t timeout_us)
{
  const uint64_t start_us = GetNowUs();

  auto recover_after_error = [&]()
  {
    i2c_ll_clear_intr_mask(hal.dev, I2C_LL_INTR_MASK);
    i2c_hal_master_fsm_rst(&hal);
    i2c_ll_update(hal.dev);
  };

  while (true)
  {
    const uint32_t intr = hal.dev->int_raw.val;

    if ((intr & I2C_LL_INTR_NACK) != 0U)
    {
      recover_after_error();
      return ErrorCode::NO_RESPONSE;
    }
    if ((intr & I2C_LL_INTR_TIMEOUT) != 0U)
    {
      recover_after_error();
      return ErrorCode::TIMEOUT;
    }
    if ((intr & I2C_LL_INTR_ARBITRATION) != 0U)
    {
      recover_after_error();
      return ErrorCode::FAILED;
    }

    if (i2c_ll_master_is_cmd_done(hal.dev, done_cmd_idx) ||
        ((intr & (I2C_LL_INTR_MST_COMPLETE | I2C_LL_INTR_END_DETECT)) != 0U))
    {
      return ErrorCode::OK;
    }

    if ((timeout_us != UINT64_MAX) && ((GetNowUs() - start_us) > timeout_us))
    {
      recover_after_error();
      return ErrorCode::TIMEOUT;
    }
  }
}

ErrorCode StartAndWaitSegment(i2c_hal_context_t& hal, int done_cmd_idx,
                              uint64_t timeout_us)
{
  i2c_ll_clear_intr_mask(hal.dev, I2C_LL_INTR_MASK);
  i2c_hal_master_trans_start(&hal);
  return WaitSegmentDone(hal, done_cmd_idx, timeout_us);
}

template <typename OperationType>
ErrorCode Complete(OperationType& op, bool in_isr, ErrorCode result)
{
  // Synchronous fast path: BLOCK ops return directly without post+wait
  // round-trip.
  if (op.type != OperationType::OperationType::BLOCK)
  {
    op.UpdateStatus(in_isr, result);
  }
  return result;
}

}  // namespace

ESP32I2C::ESP32I2C(i2c_port_t port_num, int scl_pin, int sda_pin, uint32_t clock_speed,
                   bool enable_internal_pullup, uint32_t timeout_ms,
                   uint32_t isr_enable_min_size)
    : port_num_(port_num),
      scl_pin_(scl_pin),
      sda_pin_(sda_pin),
      enable_internal_pullup_(enable_internal_pullup),
      timeout_ms_(timeout_ms),
      isr_enable_min_size_(isr_enable_min_size),
      config_{clock_speed}
{
  ASSERT(port_num_ >= 0);
  ASSERT(port_num_ < SOC_I2C_NUM);
  ASSERT(GPIO_IS_VALID_OUTPUT_GPIO(static_cast<gpio_num_t>(scl_pin_)));
  ASSERT(GPIO_IS_VALID_OUTPUT_GPIO(static_cast<gpio_num_t>(sda_pin_)));
  ASSERT(config_.clock_speed > 0U);
  ASSERT(FIFO_LEN > 2U);

  if (InitHardware() != ErrorCode::OK)
  {
    REQUIRE(false);
    return;
  }
}

bool ESP32I2C::Acquire() { return !busy_.TestAndSet(); }

void ESP32I2C::Release() { busy_.Clear(); }

bool ESP32I2C::IsValid7BitAddr(uint16_t addr) { return addr <= 0x7FU; }

ErrorCode ESP32I2C::EnsureInitialized(bool in_isr)
{
  if (initialized_)
  {
    return ErrorCode::OK;
  }
  if (in_isr)
  {
    return ErrorCode::INIT_ERR;
  }
  return InitHardware();
}

size_t ESP32I2C::MemAddrBytes(MemAddrLength mem_addr_size)
{
  return (mem_addr_size == MemAddrLength::BYTE_16) ? 2U : 1U;
}

void ESP32I2C::EncodeMemAddr(uint16_t mem_addr, size_t mem_len, uint8_t* out)
{
  DEV_ASSERT(out != nullptr);
  if (mem_len == 2U)
  {
    out[0] = static_cast<uint8_t>((mem_addr >> 8) & 0xFFU);
    out[1] = static_cast<uint8_t>(mem_addr & 0xFFU);
    return;
  }

  out[0] = static_cast<uint8_t>(mem_addr & 0xFFU);
}

ErrorCode ESP32I2C::ResolveClockSource(uint32_t& source_hz)
{
  source_hz = 0U;
  const esp_err_t err =
      esp_clk_tree_src_get_freq_hz(static_cast<soc_module_clk_t>(I2C_CLK_SRC_DEFAULT),
                                   ESP_CLK_TREE_SRC_FREQ_PRECISION_CACHED, &source_hz);
  if ((err != ESP_OK) || (source_hz == 0U))
  {
    return ErrorCode::INIT_ERR;
  }
  return ErrorCode::OK;
}

ErrorCode ESP32I2C::ApplyConfig()
{
  if ((hal_.dev == nullptr) || (config_.clock_speed == 0U))
  {
    return ErrorCode::ARG_ERR;
  }

  uint32_t source_clock_hz = 0U;
  if (ResolveClockSource(source_clock_hz) != ErrorCode::OK)
  {
    return ErrorCode::INIT_ERR;
  }

  // ESP-IDF supports Standard/Fast mode. Reject values before entering the
  // LL timing calculation, whose preconditions are enforced with assertions.
  if (config_.clock_speed > 400000U || config_.clock_speed > source_clock_hz / 28U)
  {
    return ErrorCode::NOT_SUPPORT;
  }
#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2
  // These targets have no module-clock divider and 10-bit START/STOP counters.
  if (source_clock_hz / config_.clock_speed / 2U > I2C_SCL_START_HOLD_TIME_V)
  {
    return ErrorCode::NOT_SUPPORT;
  }
#else
  // The LL implementation uses an 8-bit module-clock divider (encoded minus one).
  if (source_clock_hz / (config_.clock_speed * 1024U) + 1U > 256U)
  {
    return ErrorCode::NOT_SUPPORT;
  }
#endif

  source_clock_hz_ = source_clock_hz;
  PERIPH_RCC_ATOMIC() { i2c_ll_set_source_clk(hal_.dev, I2C_CLK_SRC_DEFAULT); }
  // ESP32's timing calculation compensates for the currently selected filter.
  i2c_ll_master_set_filter(hal_.dev, 7U);
  _i2c_hal_set_bus_timing(&hal_, static_cast<int>(config_.clock_speed),
                          I2C_CLK_SRC_DEFAULT, static_cast<int>(source_clock_hz_));
  i2c_ll_update(hal_.dev);

  return ErrorCode::OK;
}

ErrorCode ESP32I2C::InitHardware()
{
  if (initialized_)
  {
    return ErrorCode::OK;
  }

  if ((port_num_ < 0) || (port_num_ >= SOC_I2C_NUM) ||
      (static_cast<size_t>(port_num_) >= SOC_I2C_NUM))
  {
    return ErrorCode::OUT_OF_RANGE;
  }

  if (!GPIO_IS_VALID_OUTPUT_GPIO(static_cast<gpio_num_t>(scl_pin_)) ||
      !GPIO_IS_VALID_OUTPUT_GPIO(static_cast<gpio_num_t>(sda_pin_)))
  {
    return ErrorCode::ARG_ERR;
  }

  SetBusClockAtomic(port_num_, true);
  ResetBusRegisterAtomic(port_num_);

  _i2c_hal_init(&hal_, static_cast<int>(port_num_));
  if (hal_.dev == nullptr)
  {
    return ErrorCode::INIT_ERR;
  }

  i2c_hal_master_init(&hal_);
  i2c_ll_disable_intr_mask(hal_.dev, I2C_LL_MASTER_EVENT_INTR);
  i2c_ll_clear_intr_mask(hal_.dev, I2C_LL_MASTER_EVENT_INTR);

  ErrorCode err = ConfigurePins();
  if (err != ErrorCode::OK)
  {
    return err;
  }

  err = InstallInterrupt();
  if (err != ErrorCode::OK)
  {
    return err;
  }

  err = ApplyConfig();
  if (err != ErrorCode::OK)
  {
    return err;
  }

  initialized_ = true;
  return ErrorCode::OK;
}

ErrorCode ESP32I2C::ConfigurePins()
{
  if (hal_.dev == nullptr)
  {
    return ErrorCode::STATE_ERR;
  }

  const auto& sig = i2c_periph_signal[port_num_];
  const gpio_num_t sda_gpio = static_cast<gpio_num_t>(sda_pin_);
  const gpio_num_t scl_gpio = static_cast<gpio_num_t>(scl_pin_);

  gpio_set_level(sda_gpio, 1);
  esp_rom_gpio_pad_select_gpio(static_cast<uint32_t>(sda_pin_));
  gpio_set_direction(sda_gpio, GPIO_MODE_INPUT_OUTPUT_OD);
  gpio_set_pull_mode(sda_gpio,
                     enable_internal_pullup_ ? GPIO_PULLUP_ONLY : GPIO_FLOATING);
  esp_rom_gpio_connect_out_signal(sda_pin_, sig.sda_out_sig, false, false);
  esp_rom_gpio_connect_in_signal(sda_pin_, sig.sda_in_sig, false);

  gpio_set_level(scl_gpio, 1);
  esp_rom_gpio_pad_select_gpio(static_cast<uint32_t>(scl_pin_));
  gpio_set_direction(scl_gpio, GPIO_MODE_INPUT_OUTPUT_OD);
  gpio_set_pull_mode(scl_gpio,
                     enable_internal_pullup_ ? GPIO_PULLUP_ONLY : GPIO_FLOATING);
  esp_rom_gpio_connect_out_signal(scl_pin_, sig.scl_out_sig, false, false);
  esp_rom_gpio_connect_in_signal(scl_pin_, sig.scl_in_sig, false);

  return ErrorCode::OK;
}

ErrorCode ESP32I2C::RecoverController()
{
  if (hal_.dev == nullptr)
  {
    return ErrorCode::INIT_ERR;
  }

#if SOC_I2C_SUPPORT_HW_FSM_RST
  i2c_hal_master_fsm_rst(&hal_);
  i2c_ll_update(hal_.dev);
#else
  // Reset the controller before clearing the external bus. Restore source
  // clock and timing first because the hardware clear FSM needs a live I2C
  // clock after the register reset.
  ResetBusRegisterAtomic(port_num_);
  i2c_hal_master_init(&hal_);
  i2c_ll_disable_intr_mask(hal_.dev, I2C_LL_MASTER_EVENT_INTR);
  i2c_ll_clear_intr_mask(hal_.dev, I2C_LL_INTR_MASK);
  const ErrorCode config_ec = ApplyConfig();
  if (config_ec != ErrorCode::OK)
  {
    return config_ec;
  }
#endif

#if SOC_I2C_SUPPORT_HW_CLR_BUS
  constexpr uint64_t BUS_CLEAR_TIMEOUT_US = 50000ULL;
  const uint64_t start_us = GetNowUs();
  i2c_ll_master_clr_bus(hal_.dev, I2C_LL_RESET_SLV_SCL_PULSE_NUM_DEFAULT, true);
  while (i2c_ll_master_is_bus_clear_done(hal_.dev))
  {
    if ((GetNowUs() - start_us) > BUS_CLEAR_TIMEOUT_US)
    {
      i2c_ll_master_clr_bus(hal_.dev, 0, false);
      i2c_ll_update(hal_.dev);
      return ErrorCode::TIMEOUT;
    }
  }
  i2c_ll_update(hal_.dev);
#else
  // Software fallback for targets without the hardware bus-clear state
  // machine. A slave interrupted mid-byte may keep SDA low across an MCU
  // reset, so clock it through at most one byte and then generate STOP.
  constexpr int BUS_CLEAR_SCL_PULSES = 9;
  constexpr uint32_t BUS_CLEAR_HALF_PERIOD_US = 5U;

  const gpio_num_t scl_gpio = static_cast<gpio_num_t>(scl_pin_);
  const gpio_num_t sda_gpio = static_cast<gpio_num_t>(sda_pin_);
  esp_rom_gpio_pad_select_gpio(static_cast<uint32_t>(scl_pin_));
  esp_rom_gpio_pad_select_gpio(static_cast<uint32_t>(sda_pin_));
  gpio_set_level(scl_gpio, 0);
  gpio_set_level(sda_gpio, 1);
  gpio_set_direction(scl_gpio, GPIO_MODE_OUTPUT_OD);
  gpio_set_direction(sda_gpio, GPIO_MODE_INPUT_OUTPUT_OD);
  gpio_set_pull_mode(scl_gpio,
                     enable_internal_pullup_ ? GPIO_PULLUP_ONLY : GPIO_FLOATING);
  gpio_set_pull_mode(sda_gpio,
                     enable_internal_pullup_ ? GPIO_PULLUP_ONLY : GPIO_FLOATING);
  esp_rom_delay_us(BUS_CLEAR_HALF_PERIOD_US);

  int pulse_count = 0;
  while ((gpio_get_level(sda_gpio) == 0) && (pulse_count++ < BUS_CLEAR_SCL_PULSES))
  {
    gpio_set_level(scl_gpio, 1);
    esp_rom_delay_us(BUS_CLEAR_HALF_PERIOD_US);
    gpio_set_level(scl_gpio, 0);
    esp_rom_delay_us(BUS_CLEAR_HALF_PERIOD_US);
  }

  // Generate STOP: SDA low -> high while SCL is high.
  gpio_set_level(sda_gpio, 0);
  gpio_set_level(scl_gpio, 1);
  esp_rom_delay_us(BUS_CLEAR_HALF_PERIOD_US);
  gpio_set_level(sda_gpio, 1);
  esp_rom_delay_us(BUS_CLEAR_HALF_PERIOD_US);
#endif

  // Software clear temporarily selects GPIO mode; re-applying the matrix
  // routing is harmless on hardware-clear targets and restores it here.
  const ErrorCode pin_ec = ConfigurePins();
  if (pin_ec != ErrorCode::OK)
  {
    return pin_ec;
  }

  return ErrorCode::OK;
}

bool ESP32I2C::ShouldUseInterruptAsync(size_t total_size) const
{
  if (!intr_installed_)
  {
    return false;
  }
  return (isr_enable_min_size_ > 0U) &&
         (total_size >= static_cast<size_t>(isr_enable_min_size_));
}

ErrorCode ESP32I2C::StartAsyncTransaction(uint16_t slave_addr,
                                          const uint8_t* write_prefix_payload,
                                          size_t write_prefix_size,
                                          const uint8_t* write_payload, size_t write_size,
                                          uint8_t* read_payload, size_t read_size,
                                          ReadOperation& op, bool in_isr)
{
  if (!initialized_ || (hal_.dev == nullptr))
  {
    return ErrorCode::INIT_ERR;
  }
  if (!IsValid7BitAddr(slave_addr))
  {
    return ErrorCode::ARG_ERR;
  }
  if ((write_prefix_size > 0U) && (write_prefix_payload == nullptr))
  {
    return ErrorCode::PTR_NULL;
  }
  if (write_prefix_size > async_write_prefix_.size())
  {
    return ErrorCode::SIZE_ERR;
  }
  if ((write_size > 0U) && (write_payload == nullptr))
  {
    return ErrorCode::PTR_NULL;
  }
  if ((read_size > 0U) && (read_payload == nullptr))
  {
    return ErrorCode::PTR_NULL;
  }
  if (async_running_)
  {
    return ErrorCode::BUSY;
  }

  if (i2c_ll_is_bus_busy(hal_.dev))
  {
    if (in_isr)
    {
      return ErrorCode::BUSY;
    }
    const ErrorCode ec = RecoverController();
    if (ec != ErrorCode::OK)
    {
      return ec;
    }
  }

  i2c_ll_txfifo_rst(hal_.dev);
  i2c_ll_rxfifo_rst(hal_.dev);
  i2c_ll_disable_intr_mask(hal_.dev, I2C_LL_MASTER_EVENT_INTR);
  i2c_ll_clear_intr_mask(hal_.dev, I2C_LL_INTR_MASK);

  async_op_ = op;
  op.MarkAsRunning();
  async_running_ = true;
  async_slave_addr_ = slave_addr;
  async_write_prefix_size_ = write_prefix_size;
  async_write_prefix_offset_ = 0U;
  if (write_prefix_size > 0U)
  {
    Memory::FastCopy(async_write_prefix_.data(), write_prefix_payload, write_prefix_size);
  }
  async_write_payload_ = write_payload;
  async_write_size_ = write_size;
  async_write_offset_ = 0U;
  async_read_payload_ = read_payload;
  async_read_size_ = read_size;
  async_read_offset_ = 0U;
  async_pending_read_chunk_ = 0U;
  async_write_phase_done_ =
      !((write_prefix_size > 0U) || (write_size > 0U) || (read_size == 0U));
  async_write_addr_sent_ = false;
  async_write_stop_sent_ = false;
  async_read_addr_sent_ = false;

  const ErrorCode kick = KickAsyncTransaction();
  if ((kick == ErrorCode::PENDING) || (kick == ErrorCode::OK))
  {
    if (kick == ErrorCode::OK)
    {
      FinishAsync(false, ErrorCode::OK);
    }
    return ErrorCode::OK;
  }

  async_running_ = false;
  async_write_prefix_size_ = 0U;
  async_write_prefix_offset_ = 0U;
  async_write_payload_ = nullptr;
  async_write_size_ = 0U;
  async_write_offset_ = 0U;
  async_read_payload_ = nullptr;
  async_read_size_ = 0U;
  async_read_offset_ = 0U;
  async_pending_read_chunk_ = 0U;
  async_write_phase_done_ = true;
  async_write_addr_sent_ = false;
  async_write_stop_sent_ = false;
  async_read_addr_sent_ = false;
  return kick;
}

ErrorCode ESP32I2C::KickAsyncTransaction()
{
  if (!async_running_ || (hal_.dev == nullptr))
  {
    return ErrorCode::STATE_ERR;
  }

  const uint8_t write_addr =
      static_cast<uint8_t>((async_slave_addr_ << 1U) | I2C_MASTER_WRITE);
  const uint8_t read_addr =
      static_cast<uint8_t>((async_slave_addr_ << 1U) | I2C_MASTER_READ);
  const size_t fifo_len = FIFO_LEN;
  const size_t write_chunk_cap = (fifo_len > 1U) ? (fifo_len - 1U) : 0U;
  DEV_ASSERT(write_chunk_cap > 0U);

  while (true)
  {
    if (async_pending_read_chunk_ > 0U)
    {
      i2c_ll_read_rxfifo(hal_.dev, async_read_payload_ + async_read_offset_,
                         static_cast<uint8_t>(async_pending_read_chunk_));
      async_read_offset_ += async_pending_read_chunk_;
      async_pending_read_chunk_ = 0U;
    }

    int cmd_idx = 0;

    if (!async_write_phase_done_)
    {
      if (!async_write_addr_sent_)
      {
        WriteCommand(hal_.dev, cmd_idx++, I2C_LL_CMD_RESTART, ACK_VALUE, ACK_VALUE,
                     NO_CHECK_ACK, 0U);
        i2c_ll_write_txfifo(hal_.dev, &write_addr, 1U);
        WriteCommand(hal_.dev, cmd_idx++, I2C_LL_CMD_WRITE, ACK_VALUE, ACK_VALUE,
                     CHECK_ACK, 1U);
        async_write_addr_sent_ = true;
      }

      if (async_write_prefix_offset_ < async_write_prefix_size_)
      {
        const size_t chunk = std::min(
            async_write_prefix_size_ - async_write_prefix_offset_, write_chunk_cap);
        i2c_ll_write_txfifo(hal_.dev,
                            async_write_prefix_.data() + async_write_prefix_offset_,
                            static_cast<uint8_t>(chunk));
        async_write_prefix_offset_ += chunk;

        WriteCommand(hal_.dev, cmd_idx++, I2C_LL_CMD_WRITE, ACK_VALUE, ACK_VALUE,
                     CHECK_ACK, static_cast<uint8_t>(chunk));
        WriteCommand(hal_.dev, cmd_idx++, I2C_LL_CMD_END, ACK_VALUE, ACK_VALUE,
                     NO_CHECK_ACK, 0U);
      }
      else if (async_write_offset_ < async_write_size_)
      {
        const size_t chunk =
            std::min(async_write_size_ - async_write_offset_, write_chunk_cap);
        i2c_ll_write_txfifo(hal_.dev, async_write_payload_ + async_write_offset_,
                            static_cast<uint8_t>(chunk));
        async_write_offset_ += chunk;

        WriteCommand(hal_.dev, cmd_idx++, I2C_LL_CMD_WRITE, ACK_VALUE, ACK_VALUE,
                     CHECK_ACK, static_cast<uint8_t>(chunk));
        WriteCommand(hal_.dev, cmd_idx++, I2C_LL_CMD_END, ACK_VALUE, ACK_VALUE,
                     NO_CHECK_ACK, 0U);
      }
      else if (async_read_size_ == 0U)
      {
        if (!async_write_stop_sent_)
        {
          WriteCommand(hal_.dev, cmd_idx++, I2C_LL_CMD_STOP, ACK_VALUE, ACK_VALUE,
                       NO_CHECK_ACK, 0U);
#if SOC_I2C_STOP_INDEPENDENT
          WriteCommand(hal_.dev, cmd_idx++, I2C_LL_CMD_END, ACK_VALUE, ACK_VALUE,
                       NO_CHECK_ACK, 0U);
#endif
          async_write_stop_sent_ = true;
        }
        async_write_phase_done_ = async_write_stop_sent_;
      }
      else
      {
        async_write_phase_done_ = true;
      }

      if (cmd_idx > 0)
      {
        i2c_ll_clear_intr_mask(hal_.dev, I2C_LL_INTR_MASK);
        i2c_ll_enable_intr_mask(hal_.dev, I2C_LL_MASTER_EVENT_INTR);
        i2c_hal_master_trans_start(&hal_);
        return ErrorCode::PENDING;
      }

      continue;
    }

    if (async_read_size_ > 0U)
    {
      if (!async_read_addr_sent_)
      {
        WriteCommand(hal_.dev, cmd_idx++, I2C_LL_CMD_RESTART, ACK_VALUE, ACK_VALUE,
                     NO_CHECK_ACK, 0U);
        i2c_ll_write_txfifo(hal_.dev, &read_addr, 1U);
        WriteCommand(hal_.dev, cmd_idx++, I2C_LL_CMD_WRITE, ACK_VALUE, ACK_VALUE,
                     CHECK_ACK, 1U);
        async_read_addr_sent_ = true;
      }

      if (async_read_offset_ < async_read_size_)
      {
        const size_t chunk = std::min(async_read_size_ - async_read_offset_, fifo_len);
        const bool is_last = (async_read_offset_ + chunk) >= async_read_size_;

        if (!is_last)
        {
          WriteCommand(hal_.dev, cmd_idx++, I2C_LL_CMD_READ, ACK_VALUE, ACK_VALUE,
                       NO_CHECK_ACK, static_cast<uint8_t>(chunk));
          WriteCommand(hal_.dev, cmd_idx++, I2C_LL_CMD_END, ACK_VALUE, ACK_VALUE,
                       NO_CHECK_ACK, 0U);
        }
        else if (chunk == 1U)
        {
          WriteCommand(hal_.dev, cmd_idx++, I2C_LL_CMD_READ, NACK_VALUE, ACK_VALUE,
                       NO_CHECK_ACK, 1U);
        }
        else
        {
          WriteCommand(hal_.dev, cmd_idx++, I2C_LL_CMD_READ, ACK_VALUE, ACK_VALUE,
                       NO_CHECK_ACK, static_cast<uint8_t>(chunk - 1U));
          WriteCommand(hal_.dev, cmd_idx++, I2C_LL_CMD_READ, NACK_VALUE, ACK_VALUE,
                       NO_CHECK_ACK, 1U);
        }

        if (is_last)
        {
          WriteCommand(hal_.dev, cmd_idx++, I2C_LL_CMD_STOP, ACK_VALUE, ACK_VALUE,
                       NO_CHECK_ACK, 0U);
#if SOC_I2C_STOP_INDEPENDENT
          WriteCommand(hal_.dev, cmd_idx++, I2C_LL_CMD_END, ACK_VALUE, ACK_VALUE,
                       NO_CHECK_ACK, 0U);
#endif
        }

        async_pending_read_chunk_ = chunk;
      }

      if (cmd_idx > 0)
      {
        i2c_ll_clear_intr_mask(hal_.dev, I2C_LL_INTR_MASK);
        i2c_ll_enable_intr_mask(hal_.dev, I2C_LL_MASTER_EVENT_INTR);
        i2c_hal_master_trans_start(&hal_);
        return ErrorCode::PENDING;
      }
    }

    return ErrorCode::OK;
  }
}

void ESP32I2C::FinishAsync(bool in_isr, ErrorCode ec)
{
  if (!async_running_)
  {
    return;
  }

  if (hal_.dev != nullptr)
  {
    i2c_ll_disable_intr_mask(hal_.dev, I2C_LL_MASTER_EVENT_INTR);
    i2c_ll_clear_intr_mask(hal_.dev, I2C_LL_INTR_MASK);
  }

  ReadOperation op = async_op_;
  async_op_ = {};
  async_running_ = false;
  async_write_prefix_size_ = 0U;
  async_write_prefix_offset_ = 0U;
  async_write_payload_ = nullptr;
  async_write_size_ = 0U;
  async_write_offset_ = 0U;
  async_read_payload_ = nullptr;
  async_read_size_ = 0U;
  async_read_offset_ = 0U;
  async_pending_read_chunk_ = 0U;
  async_write_phase_done_ = true;
  async_write_addr_sent_ = false;
  async_write_stop_sent_ = false;
  async_read_addr_sent_ = false;

  Release();
  if (op.type == ReadOperation::OperationType::BLOCK)
  {
    (void)block_wait_.TryPost(in_isr, ec);
  }
  else
  {
    op.UpdateStatus(in_isr, ec);
  }
}

ErrorCode ESP32I2C::InstallInterrupt()
{
  if (intr_installed_)
  {
    return ErrorCode::OK;
  }

  const int irq = i2c_periph_signal[port_num_].irq;
  if (irq <= 0)
  {
    return ErrorCode::NOT_SUPPORT;
  }

  if (esp_intr_alloc(irq, 0, I2cIsrEntry, this, &intr_handle_) != ESP_OK)
  {
    intr_handle_ = nullptr;
    return ErrorCode::INIT_ERR;
  }

  intr_installed_ = true;
  return ErrorCode::OK;
}

void ESP32I2C::I2cIsrEntry(void* arg)
{
  auto* self = static_cast<ESP32I2C*>(arg);
  if (self != nullptr)
  {
    self->HandleInterrupt();
  }
}

void ESP32I2C::HandleInterrupt()
{
  if (hal_.dev == nullptr)
  {
    return;
  }

  const uint32_t intr = hal_.dev->int_raw.val;
  if ((intr & I2C_LL_MASTER_EVENT_INTR) == 0U)
  {
    return;
  }

  if (!async_running_)
  {
    i2c_ll_disable_intr_mask(hal_.dev, I2C_LL_MASTER_EVENT_INTR);
    i2c_ll_clear_intr_mask(hal_.dev, I2C_LL_INTR_MASK);
    return;
  }

  i2c_ll_disable_intr_mask(hal_.dev, I2C_LL_MASTER_EVENT_INTR);
  i2c_ll_clear_intr_mask(hal_.dev, I2C_LL_INTR_MASK);

  if ((intr & I2C_LL_INTR_NACK) != 0U)
  {
    i2c_hal_master_fsm_rst(&hal_);
    i2c_ll_update(hal_.dev);
    FinishAsync(true, ErrorCode::NO_RESPONSE);
    return;
  }
  if ((intr & I2C_LL_INTR_TIMEOUT) != 0U)
  {
    i2c_hal_master_fsm_rst(&hal_);
    i2c_ll_update(hal_.dev);
    FinishAsync(true, ErrorCode::TIMEOUT);
    return;
  }
  if ((intr & I2C_LL_INTR_ARBITRATION) != 0U)
  {
    i2c_hal_master_fsm_rst(&hal_);
    i2c_ll_update(hal_.dev);
    FinishAsync(true, ErrorCode::FAILED);
    return;
  }

  if ((intr & (I2C_LL_INTR_MST_COMPLETE | I2C_LL_INTR_END_DETECT)) == 0U)
  {
    return;
  }

  const ErrorCode kick = KickAsyncTransaction();
  if (kick == ErrorCode::PENDING)
  {
    return;
  }

  FinishAsync(true, kick);
}

ErrorCode ESP32I2C::ExecuteTransaction(uint16_t slave_addr, const uint8_t* write_payload,
                                       size_t write_size, uint8_t* read_payload,
                                       size_t read_size, bool in_isr)
{
  if (!initialized_ || (hal_.dev == nullptr))
  {
    return ErrorCode::INIT_ERR;
  }
  if (!IsValid7BitAddr(slave_addr))
  {
    return ErrorCode::ARG_ERR;
  }
  if ((write_size > 0U) && (write_payload == nullptr))
  {
    return ErrorCode::PTR_NULL;
  }
  if ((read_size > 0U) && (read_payload == nullptr))
  {
    return ErrorCode::PTR_NULL;
  }

  if (i2c_ll_is_bus_busy(hal_.dev))
  {
    if (in_isr)
    {
      return ErrorCode::BUSY;
    }
    const ErrorCode ec = RecoverController();
    if (ec != ErrorCode::OK)
    {
      return ec;
    }
  }

  i2c_ll_txfifo_rst(hal_.dev);
  i2c_ll_rxfifo_rst(hal_.dev);
  i2c_ll_clear_intr_mask(hal_.dev, I2C_LL_INTR_MASK);

  const uint64_t timeout_us = ToTimeoutUs(timeout_ms_);
  const uint8_t write_addr = static_cast<uint8_t>((slave_addr << 1U) | I2C_MASTER_WRITE);
  const uint8_t read_addr = static_cast<uint8_t>((slave_addr << 1U) | I2C_MASTER_READ);
  const size_t fifo_len = FIFO_LEN;
  const size_t write_chunk_cap = (fifo_len > 1U) ? (fifo_len - 1U) : 0U;
  DEV_ASSERT(write_chunk_cap > 0U);

  int cmd_idx = 0;

  auto start_and_wait = [&](int done_cmd) -> ErrorCode
  { return StartAndWaitSegment(hal_, done_cmd, timeout_us); };

  if ((write_size > 0U) || (read_size == 0U))
  {
    WriteCommand(hal_.dev, cmd_idx++, I2C_LL_CMD_RESTART, ACK_VALUE, ACK_VALUE,
                 NO_CHECK_ACK, 0U);

    i2c_ll_write_txfifo(hal_.dev, &write_addr, 1U);
    WriteCommand(hal_.dev, cmd_idx++, I2C_LL_CMD_WRITE, ACK_VALUE, ACK_VALUE, CHECK_ACK,
                 1U);

    size_t write_offset = 0U;
    while (write_offset < write_size)
    {
      const size_t chunk = std::min(write_size - write_offset, write_chunk_cap);
      i2c_ll_write_txfifo(hal_.dev,
                          static_cast<const uint8_t*>(write_payload) + write_offset,
                          static_cast<uint8_t>(chunk));
      write_offset += chunk;

      WriteCommand(hal_.dev, cmd_idx++, I2C_LL_CMD_WRITE, ACK_VALUE, ACK_VALUE, CHECK_ACK,
                   static_cast<uint8_t>(chunk));
      WriteCommand(hal_.dev, cmd_idx++, I2C_LL_CMD_END, ACK_VALUE, ACK_VALUE,
                   NO_CHECK_ACK, 0U);

      const ErrorCode ec = start_and_wait(cmd_idx - 1);
      if (ec != ErrorCode::OK)
      {
        if (!in_isr)
        {
          (void)RecoverController();
        }
        return ec;
      }
      cmd_idx = 0;
    }

    if (write_size == 0U)
    {
      if (read_size == 0U)
      {
        WriteCommand(hal_.dev, cmd_idx++, I2C_LL_CMD_STOP, ACK_VALUE, ACK_VALUE,
                     NO_CHECK_ACK, 0U);
#if SOC_I2C_STOP_INDEPENDENT
        WriteCommand(hal_.dev, cmd_idx++, I2C_LL_CMD_END, ACK_VALUE, ACK_VALUE,
                     NO_CHECK_ACK, 0U);
#endif
      }
      const ErrorCode ec = start_and_wait(cmd_idx - 1);
      if (ec != ErrorCode::OK)
      {
        if (!in_isr)
        {
          (void)RecoverController();
        }
        return ec;
      }
      cmd_idx = 0;
    }
    else if (read_size == 0U)
    {
      WriteCommand(hal_.dev, cmd_idx++, I2C_LL_CMD_STOP, ACK_VALUE, ACK_VALUE,
                   NO_CHECK_ACK, 0U);
#if SOC_I2C_STOP_INDEPENDENT
      WriteCommand(hal_.dev, cmd_idx++, I2C_LL_CMD_END, ACK_VALUE, ACK_VALUE,
                   NO_CHECK_ACK, 0U);
#endif
      const ErrorCode ec = start_and_wait(cmd_idx - 1);
      if (ec != ErrorCode::OK)
      {
        if (!in_isr)
        {
          (void)RecoverController();
        }
        return ec;
      }
      cmd_idx = 0;
    }
  }

  if (read_size > 0U)
  {
    WriteCommand(hal_.dev, cmd_idx++, I2C_LL_CMD_RESTART, ACK_VALUE, ACK_VALUE,
                 NO_CHECK_ACK, 0U);
    i2c_ll_write_txfifo(hal_.dev, &read_addr, 1U);
    WriteCommand(hal_.dev, cmd_idx++, I2C_LL_CMD_WRITE, ACK_VALUE, ACK_VALUE, CHECK_ACK,
                 1U);

    size_t read_offset = 0U;
    while (read_offset < read_size)
    {
      const size_t chunk = std::min(read_size - read_offset, fifo_len);
      const bool is_last = (read_offset + chunk) >= read_size;

      if (!is_last)
      {
        WriteCommand(hal_.dev, cmd_idx++, I2C_LL_CMD_READ, ACK_VALUE, ACK_VALUE,
                     NO_CHECK_ACK, static_cast<uint8_t>(chunk));
        WriteCommand(hal_.dev, cmd_idx++, I2C_LL_CMD_END, ACK_VALUE, ACK_VALUE,
                     NO_CHECK_ACK, 0U);
      }
      else if (chunk == 1U)
      {
        WriteCommand(hal_.dev, cmd_idx++, I2C_LL_CMD_READ, NACK_VALUE, ACK_VALUE,
                     NO_CHECK_ACK, 1U);
      }
      else
      {
        WriteCommand(hal_.dev, cmd_idx++, I2C_LL_CMD_READ, ACK_VALUE, ACK_VALUE,
                     NO_CHECK_ACK, static_cast<uint8_t>(chunk - 1U));
        WriteCommand(hal_.dev, cmd_idx++, I2C_LL_CMD_READ, NACK_VALUE, ACK_VALUE,
                     NO_CHECK_ACK, 1U);
      }

      if (is_last)
      {
        WriteCommand(hal_.dev, cmd_idx++, I2C_LL_CMD_STOP, ACK_VALUE, ACK_VALUE,
                     NO_CHECK_ACK, 0U);
#if SOC_I2C_STOP_INDEPENDENT
        WriteCommand(hal_.dev, cmd_idx++, I2C_LL_CMD_END, ACK_VALUE, ACK_VALUE,
                     NO_CHECK_ACK, 0U);
#endif
      }

      const ErrorCode ec = start_and_wait(cmd_idx - 1);
      if (ec != ErrorCode::OK)
      {
        if (!in_isr)
        {
          (void)RecoverController();
        }
        return ec;
      }

      i2c_ll_read_rxfifo(hal_.dev, read_payload + read_offset,
                         static_cast<uint8_t>(chunk));
      read_offset += chunk;
      cmd_idx = 0;
    }
  }

  i2c_ll_clear_intr_mask(hal_.dev, I2C_LL_INTR_MASK);
  return ErrorCode::OK;
}

ErrorCode ESP32I2C::SetConfig(Configuration config)
{
  if (config.clock_speed == 0U)
  {
    return ErrorCode::ARG_ERR;
  }

  if (!initialized_)
  {
    const ErrorCode init_err = InitHardware();
    if (init_err != ErrorCode::OK)
    {
      return init_err;
    }
  }

  if (!Acquire())
  {
    return ErrorCode::BUSY;
  }

  if (i2c_ll_is_bus_busy(hal_.dev))
  {
    Release();
    return ErrorCode::BUSY;
  }

  const Configuration previous_config = config_;
  config_ = config;
  const ErrorCode ans = ApplyConfig();
  if (ans != ErrorCode::OK)
  {
    config_ = previous_config;
  }
  Release();
  return ans;
}

ErrorCode ESP32I2C::Write(uint16_t slave_addr, ConstRawData write_data,
                          WriteOperation& op, bool in_isr)
{
  const ErrorCode init_ec = EnsureInitialized(in_isr);
  if (init_ec != ErrorCode::OK)
  {
    return Complete(op, in_isr, init_ec);
  }

  if (!IsValid7BitAddr(slave_addr))
  {
    return Complete(op, in_isr, ErrorCode::ARG_ERR);
  }

  if ((write_data.size_ > 0U) && (write_data.addr_ == nullptr))
  {
    return Complete(op, in_isr, ErrorCode::PTR_NULL);
  }

  if (!Acquire())
  {
    return Complete(op, in_isr, ErrorCode::BUSY);
  }

  const size_t total_size = write_data.size_;
  if (ShouldUseInterruptAsync(total_size))
  {
    if (op.type == WriteOperation::OperationType::BLOCK)
    {
      block_wait_.Start(*op.data.sem_info.sem);
    }
    const ErrorCode ans = StartAsyncTransaction(
        slave_addr, nullptr, 0U, static_cast<const uint8_t*>(write_data.addr_),
        write_data.size_, nullptr, 0U, op, in_isr);
    if (ans != ErrorCode::OK)
    {
      if (op.type == WriteOperation::OperationType::BLOCK)
      {
        block_wait_.Cancel();
      }
      Release();
      return Complete(op, in_isr, ans);
    }
    if (op.type == WriteOperation::OperationType::BLOCK)
    {
      ASSERT(!in_isr);
      return block_wait_.Wait(op.data.sem_info.timeout);
    }
    return ErrorCode::OK;
  }

  const ErrorCode ans =
      ExecuteTransaction(slave_addr, static_cast<const uint8_t*>(write_data.addr_),
                         write_data.size_, nullptr, 0U, in_isr);
  Release();
  return Complete(op, in_isr, ans);
}

ErrorCode ESP32I2C::Read(uint16_t slave_addr, RawData read_data, ReadOperation& op,
                         bool in_isr)
{
  const ErrorCode init_ec = EnsureInitialized(in_isr);
  if (init_ec != ErrorCode::OK)
  {
    return Complete(op, in_isr, init_ec);
  }

  if (!IsValid7BitAddr(slave_addr))
  {
    return Complete(op, in_isr, ErrorCode::ARG_ERR);
  }

  if ((read_data.size_ > 0U) && (read_data.addr_ == nullptr))
  {
    return Complete(op, in_isr, ErrorCode::PTR_NULL);
  }

  if (!Acquire())
  {
    return Complete(op, in_isr, ErrorCode::BUSY);
  }

  const size_t total_size = read_data.size_;
  if (ShouldUseInterruptAsync(total_size))
  {
    if (op.type == ReadOperation::OperationType::BLOCK)
    {
      block_wait_.Start(*op.data.sem_info.sem);
    }
    const ErrorCode ans = StartAsyncTransaction(slave_addr, nullptr, 0U, nullptr, 0U,
                                                static_cast<uint8_t*>(read_data.addr_),
                                                read_data.size_, op, in_isr);
    if (ans != ErrorCode::OK)
    {
      if (op.type == ReadOperation::OperationType::BLOCK)
      {
        block_wait_.Cancel();
      }
      Release();
      return Complete(op, in_isr, ans);
    }
    if (op.type == ReadOperation::OperationType::BLOCK)
    {
      ASSERT(!in_isr);
      return block_wait_.Wait(op.data.sem_info.timeout);
    }
    return ErrorCode::OK;
  }

  const ErrorCode ans =
      ExecuteTransaction(slave_addr, nullptr, 0U, static_cast<uint8_t*>(read_data.addr_),
                         read_data.size_, in_isr);
  Release();
  return Complete(op, in_isr, ans);
}

ErrorCode ESP32I2C::MemWrite(uint16_t slave_addr, uint16_t mem_addr,
                             ConstRawData write_data, WriteOperation& op,
                             MemAddrLength mem_addr_size, bool in_isr)
{
  const ErrorCode init_ec = EnsureInitialized(in_isr);
  if (init_ec != ErrorCode::OK)
  {
    return Complete(op, in_isr, init_ec);
  }

  if (!IsValid7BitAddr(slave_addr))
  {
    return Complete(op, in_isr, ErrorCode::ARG_ERR);
  }

  if ((write_data.size_ > 0U) && (write_data.addr_ == nullptr))
  {
    return Complete(op, in_isr, ErrorCode::PTR_NULL);
  }

  const size_t mem_len = MemAddrBytes(mem_addr_size);
  if (mem_len > MAX_WRITE_PAYLOAD)
  {
    return Complete(op, in_isr, ErrorCode::SIZE_ERR);
  }

  if (!Acquire())
  {
    return Complete(op, in_isr, ErrorCode::BUSY);
  }

  std::array<uint8_t, 2> mem_raw = {};
  EncodeMemAddr(mem_addr, mem_len, mem_raw.data());

  const size_t total_size = mem_len + write_data.size_;
  if (ShouldUseInterruptAsync(total_size))
  {
    if (op.type == WriteOperation::OperationType::BLOCK)
    {
      block_wait_.Start(*op.data.sem_info.sem);
    }
    const ErrorCode ans =
        StartAsyncTransaction(slave_addr, mem_raw.data(), mem_len,
                              static_cast<const uint8_t*>(write_data.addr_),
                              write_data.size_, nullptr, 0U, op, in_isr);
    if (ans != ErrorCode::OK)
    {
      if (op.type == WriteOperation::OperationType::BLOCK)
      {
        block_wait_.Cancel();
      }
      Release();
      return Complete(op, in_isr, ans);
    }
    if (op.type == WriteOperation::OperationType::BLOCK)
    {
      ASSERT(!in_isr);
      return block_wait_.Wait(op.data.sem_info.timeout);
    }
    return ErrorCode::OK;
  }

  std::array<uint8_t, FIFO_LEN> staging = {};
  const size_t max_chunk = MAX_WRITE_PAYLOAD - mem_len;
  auto* src = static_cast<const uint8_t*>(write_data.addr_);
  size_t offset = 0U;
  ErrorCode ans = ErrorCode::OK;

  if (write_data.size_ == 0U)
  {
    EncodeMemAddr(mem_addr, mem_len, staging.data());
    ans = ExecuteTransaction(slave_addr, staging.data(), mem_len, nullptr, 0U, in_isr);
  }
  else
  {
    while (offset < write_data.size_)
    {
      const size_t chunk = std::min(write_data.size_ - offset, max_chunk);
      const uint16_t cur_mem = static_cast<uint16_t>(mem_addr + offset);
      EncodeMemAddr(cur_mem, mem_len, staging.data());
      Memory::FastCopy(staging.data() + mem_len, src + offset, chunk);
      ans = ExecuteTransaction(slave_addr, staging.data(), mem_len + chunk, nullptr, 0U,
                               in_isr);
      if (ans != ErrorCode::OK)
      {
        break;
      }
      offset += chunk;
    }
  }

  Release();
  return Complete(op, in_isr, ans);
}

ErrorCode ESP32I2C::MemRead(uint16_t slave_addr, uint16_t mem_addr, RawData read_data,
                            ReadOperation& op, MemAddrLength mem_addr_size, bool in_isr)
{
  const ErrorCode init_ec = EnsureInitialized(in_isr);
  if (init_ec != ErrorCode::OK)
  {
    return Complete(op, in_isr, init_ec);
  }

  if (!IsValid7BitAddr(slave_addr))
  {
    return Complete(op, in_isr, ErrorCode::ARG_ERR);
  }

  if ((read_data.size_ > 0U) && (read_data.addr_ == nullptr))
  {
    return Complete(op, in_isr, ErrorCode::PTR_NULL);
  }

  const size_t mem_len = MemAddrBytes(mem_addr_size);
  if (mem_len > MAX_WRITE_READ_PREFIX)
  {
    return Complete(op, in_isr, ErrorCode::SIZE_ERR);
  }

  if (!Acquire())
  {
    return Complete(op, in_isr, ErrorCode::BUSY);
  }

  std::array<uint8_t, 2> mem_raw = {};
  EncodeMemAddr(mem_addr, mem_len, mem_raw.data());

  auto* dst = static_cast<uint8_t*>(read_data.addr_);
  const size_t total_size = mem_len + read_data.size_;
  if ((read_data.size_ > 0U) && ShouldUseInterruptAsync(total_size))
  {
    if (op.type == ReadOperation::OperationType::BLOCK)
    {
      block_wait_.Start(*op.data.sem_info.sem);
    }
    const ErrorCode ans =
        StartAsyncTransaction(slave_addr, mem_raw.data(), mem_len, nullptr, 0U, dst,
                              read_data.size_, op, in_isr);
    if (ans != ErrorCode::OK)
    {
      if (op.type == ReadOperation::OperationType::BLOCK)
      {
        block_wait_.Cancel();
      }
      Release();
      return Complete(op, in_isr, ans);
    }
    if (op.type == ReadOperation::OperationType::BLOCK)
    {
      ASSERT(!in_isr);
      return block_wait_.Wait(op.data.sem_info.timeout);
    }
    return ErrorCode::OK;
  }

  size_t offset = 0U;
  ErrorCode ans = ErrorCode::OK;

  while (offset < read_data.size_)
  {
    const size_t chunk = std::min(read_data.size_ - offset, MAX_READ_PAYLOAD);
    const uint16_t cur_mem = static_cast<uint16_t>(mem_addr + offset);
    EncodeMemAddr(cur_mem, mem_len, mem_raw.data());

    ans = ExecuteTransaction(slave_addr, mem_raw.data(), mem_len, dst + offset, chunk,
                             in_isr);
    if (ans != ErrorCode::OK)
    {
      break;
    }
    offset += chunk;
  }

  Release();
  return Complete(op, in_isr, ans);
}

}  // namespace LibXR
