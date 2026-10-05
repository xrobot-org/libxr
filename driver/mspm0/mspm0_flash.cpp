#include "mspm0_flash.hpp"

#include <algorithm>
#include <cstring>

#include "core_cm0plus.h"
#include "libxr_mem.hpp"

using namespace LibXR;

namespace
{

class IrqGuard
{
 public:
  IrqGuard() : primask_(__get_PRIMASK()) { __disable_irq(); }

  ~IrqGuard()
  {
    if ((primask_ & 1U) == 0U)
    {
      __enable_irq();
    }
  }

 private:
  std::uint32_t primask_;
};

bool FlashCommandPassed(DL_FLASHCTL_COMMAND_STATUS status)
{
  return status == DL_FLASHCTL_COMMAND_STATUS_PASSED;
}

std::size_t AlignDown(std::size_t value, std::size_t alignment)
{
  return value - (value % alignment);
}

std::size_t AlignUp(std::size_t value, std::size_t alignment)
{
  return AlignDown(value + alignment - 1U, alignment);
}

const FlashSector& CheckedStartSector(const FlashSector* sectors,
                                      std::size_t sector_count, std::size_t start_sector)
{
  ASSERT(sectors != nullptr);
  ASSERT(sector_count > 0U);
  ASSERT(start_sector > 0U);
  ASSERT(start_sector <= sector_count);
  return sectors[start_sector - 1U];
}

std::size_t CheckedFlashWindowSize(const FlashSector* sectors, std::size_t sector_count,
                                   std::size_t start_sector)
{
  const auto& start = CheckedStartSector(sectors, sector_count, start_sector);
  const auto& end = sectors[sector_count - 1U];
  ASSERT(end.address >= start.address);
  ASSERT(end.size > 0U);
  return static_cast<std::size_t>(end.address - start.address) + end.size;
}

}  // namespace

MSPM0Flash::MSPM0Flash(std::uint32_t base_address, std::size_t size)
    : Flash(MIN_ERASE_SIZE_BYTES, MIN_WRITE_SIZE_BYTES,
            RawData(reinterpret_cast<void*>(base_address), size)),
      base_address_(base_address),
      size_(size)
{
  ASSERT(IsAligned(base_address_, MIN_ERASE_SIZE_BYTES));
  ASSERT(IsAligned(size_, MIN_ERASE_SIZE_BYTES));
}

MSPM0Flash::MSPM0Flash(const FlashSector* sectors, std::size_t sector_count,
                       std::size_t start_sector)
    : MSPM0Flash(CheckedStartSector(sectors, sector_count, start_sector).address,
                 CheckedFlashWindowSize(sectors, sector_count, start_sector))
{
  ASSERT(CheckedStartSector(sectors, sector_count, start_sector).size ==
         MIN_ERASE_SIZE_BYTES);
}

ErrorCode MSPM0Flash::Erase(std::size_t offset, std::size_t size)
{
  if (size == 0U)
  {
    return ErrorCode::ARG_ERR;
  }

  if (!IsRangeValid(offset, size))
  {
    return ErrorCode::OUT_OF_RANGE;
  }

  const std::size_t erase_begin = AlignDown(offset, MIN_ERASE_SIZE_BYTES);
  const std::size_t erase_end = AlignUp(offset + size, MIN_ERASE_SIZE_BYTES);

  for (std::size_t sector_offset = erase_begin; sector_offset < erase_end;
       sector_offset += MIN_ERASE_SIZE_BYTES)
  {
    IrqGuard guard;
    const std::uint32_t address =
        base_address_ + static_cast<std::uint32_t>(sector_offset);
    DL_FlashCTL_executeClearStatus(FLASHCTL);
    DL_FlashCTL_unprotectSector(FLASHCTL, address, DL_FLASHCTL_REGION_SELECT_MAIN);
    const auto status = DL_FlashCTL_eraseMemoryFromRAM(FLASHCTL, address,
                                                       DL_FLASHCTL_COMMAND_SIZE_SECTOR);
    DL_FlashCTL_protectMainMemory(FLASHCTL);
    if (!FlashCommandPassed(status))
    {
      return ErrorCode::FAILED;
    }
  }

  return ErrorCode::OK;
}

ErrorCode MSPM0Flash::Write(std::size_t offset, ConstRawData data)
{
  if (data.addr_ == nullptr || data.size_ == 0U ||
      !IsAligned(offset, MIN_WRITE_SIZE_BYTES))
  {
    return ErrorCode::ARG_ERR;
  }

  if (!IsRangeValid(offset, data.size_))
  {
    return ErrorCode::OUT_OF_RANGE;
  }

  const auto* source = static_cast<const std::uint8_t*>(data.addr_);
  std::size_t written = 0U;
  while (written < data.size_)
  {
    const std::size_t chunk_size =
        std::min<std::size_t>(MIN_WRITE_SIZE_BYTES, data.size_ - written);
    std::uint32_t words[2] = {0xFFFFFFFFU, 0xFFFFFFFFU};
    std::memcpy(words, source + written, chunk_size);

    const std::uint32_t address =
        base_address_ + static_cast<std::uint32_t>(offset + written);
    if (Memory::FastCmp(reinterpret_cast<const std::uint8_t*>(address), source + written,
                        chunk_size) == 0)
    {
      written += chunk_size;
      continue;
    }

    DL_FLASHCTL_COMMAND_STATUS status;
    {
      IrqGuard guard;
      DL_FlashCTL_executeClearStatus(FLASHCTL);
      DL_FlashCTL_disableOverrideHardwareGeneratedECC(FLASHCTL);
      DL_FlashCTL_unprotectSector(FLASHCTL, address, DL_FLASHCTL_REGION_SELECT_MAIN);
      status =
          DL_FlashCTL_programMemoryFromRAM64WithECCGenerated(FLASHCTL, address, words);
      DL_FlashCTL_protectMainMemory(FLASHCTL);
    }
    if (!FlashCommandPassed(status))
    {
      return ErrorCode::FAILED;
    }

    written += chunk_size;
  }

  return ErrorCode::OK;
}

bool MSPM0Flash::IsRangeValid(std::size_t offset, std::size_t size) const
{
  return offset <= size_ && size <= (size_ - offset);
}

bool MSPM0Flash::IsAligned(std::size_t value, std::size_t alignment)
{
  return alignment != 0U && (value % alignment) == 0U;
}
