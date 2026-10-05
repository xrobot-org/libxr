#include "stm32_flash.hpp"

#ifdef HAL_FLASH_MODULE_ENABLED

using namespace LibXR;

namespace
{
/** @brief Flash 擦写期间的缓存状态与写锁管理 / Cache state and write lock for Flash
 * operations. */
class FlashOperationGuard
{
 public:
  FlashOperationGuard()
  {
#if defined(ICACHE) && defined(ICACHE_CR_EN) && defined(ICACHE_SR_BUSYF)
    standalone_enabled_ = (ICACHE->CR & ICACHE_CR_EN) != 0U;
    if (standalone_enabled_)
    {
      // 关闭独立 ICACHE 会自动失效；等硬件空闲，不消费共享的完成标志。
      // Disabling standalone ICACHE invalidates it; wait for idle without consuming
      // the shared completion flag.
      CLEAR_BIT(ICACHE->CR, ICACHE_CR_EN);
      __DSB();
      __ISB();
      const uint32_t start = HAL_GetTick();
      while ((ICACHE->CR & ICACHE_CR_EN) != 0U || (ICACHE->SR & ICACHE_SR_BUSYF) != 0U)
      {
        if (static_cast<uint32_t>(HAL_GetTick() - start) > 1U)
        {
          // 抢占期间可能已完成，超时报错前重新确认。
          // Completion may occur during preemption; recheck before failing.
          REQUIRE((ICACHE->CR & ICACHE_CR_EN) == 0U &&
                  (ICACHE->SR & ICACHE_SR_BUSYF) == 0U);
        }
      }
    }
#endif
#if defined(__ICACHE_PRESENT) && (__ICACHE_PRESENT == 1U)
    i_cache_enabled_ = (SCB->CCR & SCB_CCR_IC_Msk) != 0U;
    if (i_cache_enabled_) SCB_DisableICache();
#endif
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    d_cache_enabled_ = (SCB->CCR & SCB_CCR_DC_Msk) != 0U;
    if (d_cache_enabled_) SCB_DisableDCache();
#endif
    HAL_FLASH_Unlock();
  }

  ~FlashOperationGuard()
  {
    HAL_FLASH_Lock();
#if defined(__ICACHE_PRESENT) && (__ICACHE_PRESENT == 1U)
    if (i_cache_enabled_) SCB_EnableICache();
#endif
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    if (d_cache_enabled_) SCB_EnableDCache();
#endif
#if defined(ICACHE) && defined(ICACHE_CR_EN) && defined(ICACHE_SR_BUSYF)
    if (standalone_enabled_)
    {
      __DSB();
      SET_BIT(ICACHE->CR, ICACHE_CR_EN);
      __DSB();
      __ISB();
    }
#endif
  }

  FlashOperationGuard(const FlashOperationGuard&) = delete;
  FlashOperationGuard& operator=(const FlashOperationGuard&) = delete;

 private:
#if defined(ICACHE) && defined(ICACHE_CR_EN) && defined(ICACHE_SR_BUSYF)
  bool standalone_enabled_ = false;
#endif
#if defined(__ICACHE_PRESENT) && (__ICACHE_PRESENT == 1U)
  bool i_cache_enabled_ = false;
#endif
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
  bool d_cache_enabled_ = false;
#endif
};

/** @brief 扇区表的结束地址 / End address of the sector table */
uint32_t TableEnd(const FlashRegion* regions, size_t region_count)
{
  const auto& last = regions[region_count - 1];
  return last.address + last.sector_size * last.sector_count;
}

/** @brief 从 address 开始的扇区的字节数 / Size of the sector starting at address */
uint32_t SectorSizeAt(const FlashRegion* regions, size_t region_count, uint32_t address)
{
  for (size_t i = 0; i < region_count; ++i)
  {
    const auto& region = regions[i];
    if (address >= region.address &&
        address - region.address < region.sector_size * region.sector_count)
    {
      ASSERT((address - region.address) % region.sector_size == 0U);
      return region.sector_size;
    }
  }
  ASSERT(false);
  return 0U;
}

/** @brief 倒数第二个扇区的地址 / Address of the second-to-last sector */
uint32_t SecondToLastSector(const FlashRegion* regions, size_t region_count)
{
  const auto& last = regions[region_count - 1];
  if (last.sector_count >= 2U)
  {
    return last.address + last.sector_size * (last.sector_count - 2U);
  }
  ASSERT(region_count >= 2U);
  const auto& previous = regions[region_count - 2];
  return previous.address + previous.sector_size * (previous.sector_count - 1U);
}
}  // namespace

STM32Flash::STM32Flash(const FlashRegion* regions, size_t region_count,
                       uint32_t start_address)
    : Flash(SectorSizeAt(regions, region_count, start_address), MIN_WRITE_SIZE,
            {reinterpret_cast<void*>(start_address),
             TableEnd(regions, region_count) - start_address}),
      regions_(regions),
      base_address_(start_address),
      program_type_(DetermineProgramType()),
      region_count_(region_count)
{
}

STM32Flash::STM32Flash(const FlashRegion* regions, size_t region_count)
    : STM32Flash(regions, region_count, SecondToLastSector(regions, region_count))
{
}

ErrorCode STM32Flash::Erase(size_t offset, size_t size)
{
  if (size == 0)
  {
    return ErrorCode::ARG_ERR;
  }

  uint32_t start_addr = base_address_ + offset;
  uint32_t end_addr = start_addr + size;

  FlashOperationGuard operation;

  // 表按地址排列；index 是跨段的扇区序号，bank_entries 记录每个 bank 中已经走过的扇区
  // 数，即下一个扇区在该 bank 内的序号。bank 2 的起点由表给出，小容量型号上它与 bank 1
  // 之间可能有空洞（如 H743xG 在 0x08100000，G474xC 在 0x08040000）。
  // The table is ordered by address; index is the sector number across runs, and
  // bank_entries counts the sectors already passed in each bank, which is the next
  // sector's number within that bank. The table gives where bank 2 starts, after a gap on
  // smaller parts (H743xG at 0x08100000, G474xC at 0x08040000).
  uint32_t bank_entries[2] = {0U, 0U};
  [[maybe_unused]] uint32_t index = 0U;
  for (size_t r = 0; r < region_count_; ++r)
  {
    const auto& region = regions_[r];
    for (uint32_t k = 0; k < region.sector_count; ++k, ++index)
    {
      const uint32_t address = region.address + region.sector_size * k;
#if defined(FLASH_BANK_2)
      const size_t bank_slot = (STM32FlashBankOf(address) == FLASH_BANK_2) ? 1U : 0U;
#else
      const size_t bank_slot = 0U;
#endif
      [[maybe_unused]] const uint32_t number_in_bank = bank_entries[bank_slot]++;
      if (address + region.sector_size <= start_addr)
      {
        continue;
      }
      if (address >= end_addr)
      {
        return ErrorCode::OK;
      }
      FLASH_EraseInitTypeDef erase_init = {};

#if defined(FLASH_TYPEERASE_PAGES) && defined(FLASH_PAGE_SIZE)  // STM32F1/G4... series
      // 有 Page 字段的系列（G0、G4、L4、L5、U5 等）按 bank 内页号编号；其余系列用页地址。
      // Families with a Page field (G0, G4, L4, L5, U5, ...) number pages within each
      // bank; the others take the page address.
      erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
      SetNbPages(erase_init, address, number_in_bank);
      erase_init.NbPages = 1;
      SetBanks(erase_init, address);
#elif defined(FLASH_TYPEERASE_SECTORS)  // STM32F4/F7/H7... series
      erase_init.TypeErase = FLASH_TYPEERASE_SECTORS;
#if defined(FLASH_SECTOR_SIZE)
      // H5、H7 的扇区大小一致，按 bank 内扇区号编号。
      // H5 and H7 have uniform sectors numbered within each bank.
      erase_init.Sector = number_in_bank;
#elif defined(FLASH_SECTOR_TOTAL)
      // F2、F4、F7 的扇区号跨两个 bank 连续编号，等于跨段的扇区序号。
      // F2, F4 and F7 number sectors across both banks, as the index across runs does.
      erase_init.Sector = index;
#else
#error "No supported Flash sector numbering defined"
#endif
      erase_init.NbSectors = 1;
#if defined(FLASH_BANK_1)
      erase_init.Banks = STM32FlashBankOf(address);
#endif
#if defined(FLASH_CR_PSIZE)
      erase_init.VoltageRange = FLASH_VOLTAGE_RANGE_1;
#endif
#else
      return ErrorCode::NOT_SUPPORT;
#endif

      uint32_t error = 0;
      HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase_init, &error);
      if (status != HAL_OK || error != 0xFFFFFFFFU)
      {
        return ErrorCode::FAILED;
      }
    }
  }

  return ErrorCode::OK;
}

ErrorCode STM32Flash::Write(size_t offset, ConstRawData data)
{
  if (!data.addr_ || data.size_ == 0)
  {
    return ErrorCode::ARG_ERR;
  }

  uint32_t addr = base_address_ + offset;
  if (!IsInRange(addr, data.size_))
  {
    return ErrorCode::OUT_OF_RANGE;
  }

  FlashOperationGuard operation;

  const uint8_t* src = reinterpret_cast<const uint8_t*>(data.addr_);
  size_t written = 0;

#if defined(FLASH_TYPEPROGRAM_FLASHWORD) || defined(FLASH_TYPEPROGRAM_QUADWORD)
  alignas(LibXR::HW_CACHE_LINE_SIZE)
      uint32_t flash_word_buffer[MIN_WRITE_SIZE / sizeof(uint32_t)];
  while (written < data.size_)
  {
    size_t chunk_size = LibXR::min<size_t>(MinWriteSize(), data.size_ - written);

    Memory::FastSet(flash_word_buffer, 0xFF, sizeof(flash_word_buffer));
    Memory::FastCopy(flash_word_buffer, src + written, chunk_size);

    if (Memory::FastCmp(reinterpret_cast<const uint8_t*>(addr + written), src + written,
                        chunk_size) == 0)
    {
      written += chunk_size;
      continue;
    }

    if (HAL_FLASH_Program(program_type_, addr + written,
                          reinterpret_cast<uint32_t>(flash_word_buffer)) != HAL_OK)
    {
      return ErrorCode::FAILED;
    }

    written += chunk_size;
  }

#else
  while (written < data.size_)
  {
    size_t chunk_size = LibXR::min<size_t>(MinWriteSize(), data.size_ - written);

    if (Memory::FastCmp(reinterpret_cast<const uint8_t*>(addr + written), src + written,
                        chunk_size) == 0)
    {
      written += chunk_size;
      continue;
    }

    uint64_t word = 0xFFFFFFFFFFFFFFFF;
    Memory::FastCopy(&word, src + written, chunk_size);

    if (HAL_FLASH_Program(program_type_, addr + written, word) != HAL_OK)
    {
      return ErrorCode::FAILED;
    }

    written += chunk_size;
  }
#endif

  return ErrorCode::OK;
}

bool STM32Flash::IsInRange(uint32_t addr, size_t size) const
{
  const uint32_t BEGIN = base_address_;
  const uint32_t LIMIT = TableEnd(regions_, region_count_);
  const uint32_t END = addr + size;
  return (addr >= BEGIN) && (END <= LIMIT) && (END >= addr);  // 最后一项防溢出
}

#endif
