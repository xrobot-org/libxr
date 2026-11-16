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
#if defined(FLASH_TYPEPROGRAM_BYTE) && defined(FLASH_TYPEPROGRAM_QUADWORD)
  // H7R/S：小于四字的编程类型是对整个 128 位 Flash 字的强制写入，而一个 Flash 字连同
  // ECC 只能写一次，所以按四字编程。
  // H7R/S: the program types smaller than a quad-word are forced writes of the whole
  // 128-bit flash word, and a flash word with its ECC can be written only once, so the
  // driver programs quad-words.
  return 16;
#elif defined(FLASH_TYPEPROGRAM_BYTE)
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

// H7：读到双位 ECC 错误的 Flash 字时产生总线错误，并置位 FLASH_SR 的 DBECCERR。
// H7R/S 相同，标志是 FLASH_ISR 的 DBECCERRF，由 FLASH_ICR 清除。
// H7: reading a flash word with a double ECC error raises a bus error and sets DBECCERR
// in FLASH_SR. H7R/S does the same; its flag is DBECCERRF in FLASH_ISR, cleared through
// FLASH_ICR.
#if defined(SCB_CCR_BFHFNMIGN_Msk) &&                                  \
    ((defined(FLASH_SR_DBECCERR) && defined(FLASH_CCR_CLR_DBECCERR) && \
      defined(FLASH_CCR_CLR_SNECCERR)) ||                              \
     (defined(FLASH_ISR_DBECCERRF) && defined(FLASH_ICR_DBECCERRF) &&  \
      defined(FLASH_ICR_SNECCERRF)))
#define LIBXR_STM32_FLASH_ECC_BUS_ERROR 1
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
 * @note H7 和 H7R/S 上编程或擦除被掉电打断后，被打断的 Flash 字 ECC 校验不过。`Read` 按
 * Flash 字读取，读每个字时屏蔽全部中断并忽略总线错误，读完检查双位错误标志（H7 的
 * DBECCERR，H7R/S 的 DBECCERRF）；出错的字在缓冲区里填 0，
 * `Read` 仍返回 `ErrorCode::OK`。读到的 0 既不是擦除态也不是完整数据，调用方按写了一半的
 * 内容处理；真实损坏造成的双位错误读起来相同，不作区分。`Write` 编程前的比较同样读取，
 * 目标字读不出来时不编程，返回 `ErrorCode::CHECK_ERR`，擦除后才能再写。其他系列的 `Read`
 * 是基类的直接拷贝。
 * On H7 and H7R/S a flash word whose program or erase was cut by a power loss fails its
 * ECC check. `Read` reads flash word by flash word, masks all interrupts and ignores the
 * bus error for each word, and tests the double error flag afterwards (DBECCERR on H7,
 * DBECCERRF on H7R/S); a failing word is filled with 0 in the buffer and `Read` still
 * returns `ErrorCode::OK`. The zeros are neither erased flash nor
 * complete data, so the caller handles them as half-written contents; a double error
 * caused by real damage reads the same and is not told apart. The comparison before
 * programming in `Write` reads the same way and returns `ErrorCode::CHECK_ERR` without
 * programming when the target word cannot be read; the word can be written again after
 * an erase. On other families `Read` is the plain copy of the base class.
 * @note 这两个系列的 ECC 标志（单位错误已纠正、双位错误）置位后一直保持，H7 的 HAL 把
 * 它们当作编程或擦除失败。驱动在每次读完一个 Flash 字之后、每次擦写开始之前清除这两个
 * 标志。
 * The ECC flags of both families (single error corrected, double error) stay set once
 * raised, and the H7 HAL takes them for a failed program or erase. The driver clears both
 * after each flash word it reads and before each erase or program.
 * @note G0、G4、L4、L5、WB、WL、U0、U5、WBA、H5、U3 读到这样的单元时产生 NMI，读到的是
 * 写了一半的内容；`NMI_Handler` 需要调用 `libxr_stm32_flash_ecc_nmi()`
 * （stm32_flash_nmi.h），否则停在 CubeMX 生成的死循环里。
 * On G0, G4, L4, L5, WB, WL, U0, U5, WBA, H5 and U3 reading such a unit raises an NMI and
 * delivers the half-written contents; `NMI_Handler` has to call
 * `libxr_stm32_flash_ecc_nmi()` (stm32_flash_nmi.h), otherwise it stays in the endless
 * loop CubeMX generates.
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

#if defined(LIBXR_STM32_FLASH_ECC_BUS_ERROR)
  ErrorCode Read(size_t offset, RawData data) override;
#endif

 private:
  const FlashRegion* regions_;
  uint32_t base_address_;
  uint32_t program_type_;
  size_t region_count_;

  static constexpr uint32_t DetermineProgramType()
  {
#if defined(FLASH_TYPEPROGRAM_BYTE) && defined(FLASH_TYPEPROGRAM_QUADWORD)
    return FLASH_TYPEPROGRAM_QUADWORD;
#elif defined(FLASH_TYPEPROGRAM_BYTE)
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
