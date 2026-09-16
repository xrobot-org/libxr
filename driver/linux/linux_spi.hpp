#pragma once

#include <cstdint>
#include <string>

#include "libxr_def.hpp"
#include "spi.hpp"

namespace LibXR
{

/** Synchronous LibXR SPI backend backed by a Linux spidev node. */
class LinuxSPI final : public SPI
{
 public:
  LinuxSPI(const char *device, RawData rx_buffer, RawData tx_buffer, Configuration config);
  ~LinuxSPI();

  LinuxSPI(const LinuxSPI &) = delete;
  LinuxSPI &operator=(const LinuxSPI &) = delete;

  ErrorCode ReadAndWrite(RawData read_data, ConstRawData write_data, OperationRW &op,
                         bool in_isr = false) override;
  ErrorCode Transfer(size_t size, OperationRW &op, bool in_isr = false) override;
  ErrorCode MemWrite(uint16_t reg, ConstRawData write_data, OperationRW &op,
                     bool in_isr = false) override;
  ErrorCode MemRead(uint16_t reg, RawData read_data, OperationRW &op,
                    bool in_isr = false) override;
  ErrorCode SetConfig(Configuration config) override;

  uint32_t GetMaxBusSpeed() const override { return max_bus_speed_; }
  Prescaler GetMaxPrescaler() const override { return Prescaler::DIV_16384; }
  [[nodiscard]] bool IsValid() const { return fd_ >= 0; }
  [[nodiscard]] uint32_t GetBusSpeed() const { return bus_speed_; }
  [[nodiscard]] const char *GetDevice() const { return device_.c_str(); }

 private:
  std::string device_;
  int fd_{-1};
  uint32_t max_bus_speed_{};
  uint32_t bus_speed_{};
  uint8_t mode_{};
};

}  // namespace LibXR
