#pragma once

#include <algorithm>
#include <cstdint>
#include <cstring>

#include "flash.hpp"
#include "libxr_def.hpp"
#include "libxr_type.hpp"
#include "main.h"

#ifdef HAL_FLASH_MODULE_ENABLED

namespace LibXR
{

#if defined(FLASH_BANK_2) && defined(FLASH_BANK_1)
// NOLINTNEXTLINE
inline uint32_t STM32FlashBankOf(uint32_t addr)
{
#if !defined(FLASH_BANK2_BASE)
// NOLINTNEXTLINE
#if defined(FLASH_BANK_SIZE)
  const auto FLASH_BANK2_BASE = FLASH_BANK_SIZE + FLASH_BASE;
#else
  const auto FLASH_BANK2_BASE = 0x100000 + FLASH_BASE;
#endif
#endif
  return (addr >= FLASH_BANK2_BASE) ? FLASH_BANK_2 : FLASH_BANK_1;
}
#elif defined(FLASH_BANK_1)
// NOLINTNEXTLINE
inline uint32_t STM32FlashBankOf(uint32_t) { return FLASH_BANK_1; }
#else
// NOLINTNEXTLINE
inline uint32_t STM32FlashBankOf(uint32_t) { return 1; }
#endif

/**
 * @brief 由 HAL 的 `FLASH_TYPEPROGRAM_*` 宏确定的最小写入单位（字节） / Minimum write
 *        unit in bytes, determined from the HAL `FLASH_TYPEPROGRAM_*` macros
 */
// NOLINTNEXTLINE
constexpr size_t STM32FlashMinWriteSize()
{
#ifdef FLASH_TYPEPROGRAM_BYTE
  return 1;
#elif defined(FLASH_TYPEPROGRAM_HALFWORD)
  return 2;
#elif defined(FLASH_TYPEPROGRAM_WORD)
  return 4;
#elif defined(FLASH_TYPEPROGRAM_DOUBLEWORD)
  return 8;
#elif defined(FLASH_TYPEPROGRAM_FLASHWORD)
  return FLASH_NB_32BITWORD_IN_FLASHWORD * 4;
#elif defined(FLASH_TYPEPROGRAM_QUADWORD)
  return 16;
#else
#error "No supported FLASH_TYPEPROGRAM_xxx defined"
#endif
}

#ifndef __DOXYGEN__

template <typename, typename = void>
struct HasFlashPage : std::false_type
{
};

template <typename, typename = void>
struct HasFlashBank : std::false_type
{
};

template <typename T>
struct HasFlashPage<T, std::void_t<decltype(std::declval<T>().Page)>> : std::true_type
{
};

template <typename T>
struct HasFlashBank<T, std::void_t<decltype(std::declval<T>().Banks)>> : std::true_type
{
};

template <typename T>
// NOLINTNEXTLINE
typename std::enable_if<!HasFlashPage<T>::value>::type SetNbPages(T& init, uint32_t addr,
                                                                  uint32_t page)
{
  UNUSED(page);
  init.PageAddress = addr;
}

template <typename T>
// NOLINTNEXTLINE
typename std::enable_if<HasFlashPage<T>::value>::type SetNbPages(T& init, uint32_t addr,
                                                                 uint32_t page)
{
  UNUSED(addr);
  init.Page = page;
}

template <typename T>
// NOLINTNEXTLINE
typename std::enable_if<!HasFlashBank<T>::value>::type SetBanks(T&, uint32_t)
{
}

template <typename T>
// NOLINTNEXTLINE
typename std::enable_if<HasFlashBank<T>::value>::type SetBanks(T& init, uint32_t addr)
{
  init.Banks = STM32FlashBankOf(addr);
}

#endif

/**
 * @brief STM32 闪存驱动实现 / STM32 flash driver implementation
 * @pre 擦写调用及相关缓存控制由调用方串行化；等待期间 HAL tick 必须能推进。
 *      The caller serializes erase/program calls and related cache control; the HAL
 *      tick must advance during waits.
 * @note 擦写期间暂时关闭相关缓存；进入擦写流程后，成功或失败返回均恢复缓存开关并锁定
 * Flash。
 * Related caches are temporarily disabled during erase/program operations. Once the
 * operation starts, success and failure returns restore the original cache enable state
 * and lock Flash.
 * @note 扇区表须从 Flash 起始地址开始，按地址用若干段列出整片 Flash 的扇区或页；一段可以
 * 跨 bank。擦除时按地址判断所在 bank，H5、H7 和使用 Page 字段的系列按 bank 内序号擦除，
 * F2/F4/F7 按跨 bank 的扇区号擦除。不支持 bank 交换（SWAP_BANK、BFB2 等选项字节）。
 * The sector table must list the sectors or pages of the whole Flash in address order as
 * runs, starting at the Flash base; a run may cross a bank boundary. Erase takes the bank
 * from the address; H5, H7 and the families with a Page field erase by the number within
 * the bank, F2/F4/F7 by the sector number across both banks. Bank swap (SWAP_BANK, BFB2
 * and similar option bytes) is not supported.
 */
class STM32Flash : public Flash
{
 public:
  /**
   * @brief 最小写入单位（字节），由 HAL 的 `FLASH_TYPEPROGRAM_*` 宏确定；可直接作为
   *        `DatabaseRaw` 的模板参数 / Minimum write unit in bytes, determined from the
   *        HAL `FLASH_TYPEPROGRAM_*` macros; usable directly as the `DatabaseRaw`
   *        template argument
   */
  static constexpr size_t MIN_WRITE_SIZE = STM32FlashMinWriteSize();

  /**
   * @brief 构造闪存对象 / Construct flash object
   * @param regions 扇区表 / Sector table
   * @param region_count 扇区表的段数 / Number of runs in the sector table
   * @param start_address 存储区的起始地址，须正好是某个扇区的起点；存储区一直到 Flash
   *        末尾 / Start address of the storage area; must be exactly the start of a
   *        sector; the area extends to the end of the Flash
   */
  STM32Flash(const FlashRegion* regions, size_t region_count, uint32_t start_address);

  /**
   * @brief 用末尾两个扇区构造，供 DatabaseRaw 的主块和备份块使用 / Construct with the
   *        last two sectors, for the main and backup blocks of DatabaseRaw
   * @param regions 扇区表 / Sector table
   * @param region_count 扇区表的段数 / Number of runs in the sector table
   */
  STM32Flash(const FlashRegion* regions, size_t region_count);

  ErrorCode Erase(size_t offset, size_t size) override;

  ErrorCode Write(size_t offset, ConstRawData data) override;

 private:
  const FlashRegion* regions_;
  uint32_t base_address_;
  uint32_t program_type_;
  size_t region_count_;

  static constexpr uint32_t DetermineProgramType()
  {
#ifdef FLASH_TYPEPROGRAM_BYTE
    return FLASH_TYPEPROGRAM_BYTE;
#elif defined(FLASH_TYPEPROGRAM_HALFWORD)
    return FLASH_TYPEPROGRAM_HALFWORD;
#elif defined(FLASH_TYPEPROGRAM_WORD)
    return FLASH_TYPEPROGRAM_WORD;
#elif defined(FLASH_TYPEPROGRAM_DOUBLEWORD)
    return FLASH_TYPEPROGRAM_DOUBLEWORD;
#elif defined(FLASH_TYPEPROGRAM_FLASHWORD)
    return FLASH_TYPEPROGRAM_FLASHWORD;
#elif defined(FLASH_TYPEPROGRAM_QUADWORD)
    return FLASH_TYPEPROGRAM_QUADWORD;
#else
#error "No supported FLASH_TYPEPROGRAM_xxx defined"
#endif
  }

  bool IsInRange(uint32_t addr, size_t size) const;
};

}  // namespace LibXR

#endif
