#pragma once

#include <ti/driverlib/dl_flashctl.h>

#include <cstddef>
#include <cstdint>

#include "flash.hpp"

namespace LibXR
{

/**
 * @brief Flash sector descriptor.
 */
struct FlashSector
{
  std::uint32_t address;
  std::uint32_t size;
};

class MSPM0Flash : public Flash
{
 public:
  static constexpr std::size_t MIN_ERASE_SIZE_BYTES = DL_FLASHCTL_SECTOR_SIZE;
  static constexpr std::size_t MIN_WRITE_SIZE_BYTES = 8U;

  MSPM0Flash(std::uint32_t base_address, std::size_t size);
  MSPM0Flash(const FlashSector* sectors, std::size_t sector_count,
             std::size_t start_sector);

  MSPM0Flash(const FlashSector* sectors, std::size_t sector_count)
      : MSPM0Flash(sectors, sector_count, sector_count - 1)
  {
  }

  ErrorCode Erase(std::size_t offset, std::size_t size) override;
  ErrorCode Write(std::size_t offset, ConstRawData data) override;

 private:
  bool IsRangeValid(std::size_t offset, std::size_t size) const;
  static bool IsAligned(std::size_t value, std::size_t alignment);

  std::uint32_t base_address_ = 0;
  std::size_t size_ = 0;
};

}  // namespace LibXR
