#include "linux_spi.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace LibXR
{

LinuxSPI::LinuxSPI(const char *device, RawData rx_buffer, RawData tx_buffer,
                   Configuration config)
    : SPI(rx_buffer, tx_buffer), device_(device == nullptr ? "" : device)
{
  if (device_.empty()) return;
  fd_ = ::open(device_.c_str(), O_RDWR | O_CLOEXEC);
  if (fd_ < 0) return;

  if (::ioctl(fd_, SPI_IOC_RD_MAX_SPEED_HZ, &max_bus_speed_) < 0 || max_bus_speed_ == 0u)
    max_bus_speed_ = 0u;
  uint8_t bits = 8u;
  if (::ioctl(fd_, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0 ||
      SetConfig(config) != ErrorCode::OK)
  {
    ::close(fd_);
    fd_ = -1;
  }
}

LinuxSPI::~LinuxSPI()
{
  if (fd_ >= 0) ::close(fd_);
}

ErrorCode LinuxSPI::SetConfig(Configuration config)
{
  if (fd_ < 0) return ErrorCode::INIT_ERR;
  mode_ = static_cast<uint8_t>((static_cast<uint8_t>(config.clock_polarity) << 1u) |
                               static_cast<uint8_t>(config.clock_phase));
  if (::ioctl(fd_, SPI_IOC_WR_MODE, &mode_) < 0) return ErrorCode::FAILED;

  uint32_t speed = max_bus_speed_;
  if (config.prescaler != Prescaler::UNKNOWN)
  {
    const uint32_t divisor = PrescalerToDiv(config.prescaler);
    if (divisor == 0u) return ErrorCode::ARG_ERR;
    speed = max_bus_speed_ / divisor;
  }
  if (speed == 0u) speed = max_bus_speed_;
  bus_speed_ = speed;
  if (::ioctl(fd_, SPI_IOC_WR_MAX_SPEED_HZ, &bus_speed_) < 0) return ErrorCode::FAILED;
  GetConfig() = config;
  return ErrorCode::OK;
}

ErrorCode LinuxSPI::ReadAndWrite(RawData read_data, ConstRawData write_data, OperationRW &op,
                                 bool in_isr)
{
  ErrorCode result = ErrorCode::OK;
  const bool has_rx = read_data.addr_ != nullptr && read_data.size_ != 0u;
  const bool has_tx = write_data.addr_ != nullptr && write_data.size_ != 0u;
  if (fd_ < 0)
    result = ErrorCode::INIT_ERR;
  else if (in_isr)
    result = ErrorCode::NOT_SUPPORT;
  else if ((read_data.size_ != 0u && read_data.addr_ == nullptr) ||
           (write_data.size_ != 0u && write_data.addr_ == nullptr))
    result = ErrorCode::ARG_ERR;
  else if (!has_rx && !has_tx)
    result = ErrorCode::ARG_ERR;
  else if (has_rx && has_tx && read_data.size_ != write_data.size_)
    result = ErrorCode::SIZE_ERR;
  else
  {
    spi_ioc_transfer transfer{};
    transfer.tx_buf = has_tx ? reinterpret_cast<uintptr_t>(write_data.addr_) : 0u;
    transfer.rx_buf = has_rx ? reinterpret_cast<uintptr_t>(read_data.addr_) : 0u;
    transfer.len = static_cast<uint32_t>(has_tx ? write_data.size_ : read_data.size_);
    transfer.speed_hz = bus_speed_;
    transfer.bits_per_word = 8u;
    result = ::ioctl(fd_, SPI_IOC_MESSAGE(1), &transfer) >= 1 ? ErrorCode::OK
                                                              : ErrorCode::FAILED;
  }
  op.UpdateStatus(in_isr, result);
  return result;
}

ErrorCode LinuxSPI::Transfer(size_t size, OperationRW &op, bool in_isr)
{
  RawData rx = GetRxBuffer();
  RawData tx = GetTxBuffer();
  if (size > rx.size_ || size > tx.size_)
  {
    op.UpdateStatus(in_isr, ErrorCode::SIZE_ERR);
    return ErrorCode::SIZE_ERR;
  }
  return ReadAndWrite(RawData(rx.addr_, size), ConstRawData(tx.addr_, size), op, in_isr);
}

ErrorCode LinuxSPI::MemWrite(uint16_t reg, ConstRawData write_data, OperationRW &op,
                             bool in_isr)
{
  if (reg > 0xffu)
  {
    op.UpdateStatus(in_isr, ErrorCode::SIZE_ERR);
    return ErrorCode::SIZE_ERR;
  }
  if (write_data.size_ != 0u && write_data.addr_ == nullptr)
  {
    op.UpdateStatus(in_isr, ErrorCode::ARG_ERR);
    return ErrorCode::ARG_ERR;
  }
  if (write_data.size_ + 1u > GetTxBuffer().size_)
  {
    op.UpdateStatus(in_isr, ErrorCode::SIZE_ERR);
    return ErrorCode::SIZE_ERR;
  }
  auto tx = static_cast<uint8_t *>(GetTxBuffer().addr_);
  tx[0] = static_cast<uint8_t>(reg & 0x7fu);
  if (write_data.size_ != 0u) std::memcpy(tx + 1u, write_data.addr_, write_data.size_);
  return ReadAndWrite(RawData(nullptr, 0u), ConstRawData(tx, write_data.size_ + 1u), op,
                      in_isr);
}

ErrorCode LinuxSPI::MemRead(uint16_t reg, RawData read_data, OperationRW &op, bool in_isr)
{
  if (reg > 0xffu)
  {
    op.UpdateStatus(in_isr, ErrorCode::SIZE_ERR);
    return ErrorCode::SIZE_ERR;
  }
  if (read_data.size_ != 0u && read_data.addr_ == nullptr)
  {
    op.UpdateStatus(in_isr, ErrorCode::ARG_ERR);
    return ErrorCode::ARG_ERR;
  }
  if (read_data.size_ == 0u)
  {
    op.UpdateStatus(in_isr, ErrorCode::OK);
    return ErrorCode::OK;
  }
  if (read_data.size_ + 1u > GetRxBuffer().size_ ||
      read_data.size_ + 1u > GetTxBuffer().size_)
  {
    op.UpdateStatus(in_isr, ErrorCode::SIZE_ERR);
    return ErrorCode::SIZE_ERR;
  }
  auto tx = static_cast<uint8_t *>(GetTxBuffer().addr_);
  auto rx = static_cast<uint8_t *>(GetRxBuffer().addr_);
  tx[0] = static_cast<uint8_t>(reg | 0x80u);
  std::memset(tx + 1u, 0, read_data.size_);
  const ErrorCode result = ReadAndWrite(RawData(rx, read_data.size_ + 1u),
                                        ConstRawData(tx, read_data.size_ + 1u), op, in_isr);
  if (result == ErrorCode::OK) std::memcpy(read_data.addr_, rx + 1u, read_data.size_);
  return result;
}

}  // namespace LibXR
