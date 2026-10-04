#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "driver/gpio.h"
#include "esp_def.hpp"
#include "esp_intr_alloc.h"
#include "flag.hpp"
#include "hal/i2c_hal.h"
#include "hal/i2c_types.h"
#include "i2c.hpp"
#include "soc/i2c_periph.h"
#include "soc/soc_caps.h"

namespace LibXR
{

class ESP32I2C : public I2C
{
 public:
  static constexpr int PIN_NO_CHANGE = -1;

  ESP32I2C(i2c_port_t port_num, int scl_pin, int sda_pin, uint32_t clock_speed = 400000U,
           bool enable_internal_pullup = true, uint32_t timeout_ms = 100U,
           uint32_t isr_enable_min_size = 32U);

  ErrorCode Read(uint16_t slave_addr, RawData read_data, ReadOperation& op,
                 bool in_isr = false) override;

  ErrorCode Write(uint16_t slave_addr, ConstRawData write_data, WriteOperation& op,
                  bool in_isr = false) override;

  ErrorCode SetConfig(Configuration config) override;

  ErrorCode MemRead(uint16_t slave_addr, uint16_t mem_addr, RawData read_data,
                    ReadOperation& op,
                    MemAddrLength mem_addr_size = MemAddrLength::BYTE_8,
                    bool in_isr = false) override;

  ErrorCode MemWrite(uint16_t slave_addr, uint16_t mem_addr, ConstRawData write_data,
                     WriteOperation& op,
                     MemAddrLength mem_addr_size = MemAddrLength::BYTE_8,
                     bool in_isr = false) override;

  i2c_port_t Port() const { return port_num_; }

 private:
  static constexpr size_t FIFO_LEN = SOC_I2C_FIFO_LEN;
  static constexpr size_t MAX_WRITE_PAYLOAD = (FIFO_LEN > 4U) ? (FIFO_LEN - 4U) : 0U;
  static constexpr size_t MAX_WRITE_READ_PREFIX = (FIFO_LEN > 5U) ? (FIFO_LEN - 5U) : 0U;
  static constexpr size_t MAX_READ_PAYLOAD = (FIFO_LEN > 4U) ? (FIFO_LEN - 4U) : FIFO_LEN;

  bool Acquire();
  void Release();

  ErrorCode InitHardware();
  ErrorCode ConfigurePins();
  ErrorCode ApplyConfig();
  ErrorCode ResolveClockSource(uint32_t& source_hz);
  ErrorCode RecoverController();
  ErrorCode EnsureInitialized(bool in_isr);
  bool ShouldUseInterruptAsync(size_t total_size) const;
  static size_t MemAddrBytes(MemAddrLength mem_addr_size);
  static void EncodeMemAddr(uint16_t mem_addr, size_t mem_len, uint8_t* out);
  ErrorCode ExecuteTransaction(uint16_t slave_addr, const uint8_t* write_payload,
                               size_t write_size, uint8_t* read_payload, size_t read_size,
                               bool in_isr);
  ErrorCode StartAsyncTransaction(uint16_t slave_addr,
                                  const uint8_t* write_prefix_payload,
                                  size_t write_prefix_size, const uint8_t* write_payload,
                                  size_t write_size, uint8_t* read_payload,
                                  size_t read_size, ReadOperation& op, bool in_isr);
  ErrorCode KickAsyncTransaction();
  void FinishAsync(bool in_isr, ErrorCode ec);
  static bool IsValid7BitAddr(uint16_t addr);
  ErrorCode InstallInterrupt();
  static void I2cIsrEntry(void* arg);
  void HandleInterrupt();

  i2c_port_t port_num_;
  int scl_pin_;
  int sda_pin_;
  bool enable_internal_pullup_;
  uint32_t timeout_ms_;
  uint32_t isr_enable_min_size_;
  bool initialized_ = false;
  Configuration config_{};
  i2c_hal_context_t hal_ = {};
  uint32_t source_clock_hz_ = 0U;
  Flag::Atomic busy_{};
  intr_handle_t intr_handle_ = nullptr;
  bool intr_installed_ = false;

  bool async_running_ = false;
  ReadOperation async_op_{};
  uint16_t async_slave_addr_ = 0U;
  std::array<uint8_t, 2> async_write_prefix_ = {};
  size_t async_write_prefix_size_ = 0U;
  size_t async_write_prefix_offset_ = 0U;
  const uint8_t* async_write_payload_ = nullptr;
  size_t async_write_size_ = 0U;
  size_t async_write_offset_ = 0U;
  uint8_t* async_read_payload_ = nullptr;
  size_t async_read_size_ = 0U;
  size_t async_read_offset_ = 0U;
  size_t async_pending_read_chunk_ = 0U;
  bool async_write_phase_done_ = true;
  bool async_write_addr_sent_ = false;
  bool async_write_stop_sent_ = false;
  bool async_read_addr_sent_ = false;
  AsyncBlockWait block_wait_{};
};

}  // namespace LibXR
