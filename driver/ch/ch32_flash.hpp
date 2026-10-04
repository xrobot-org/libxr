#pragma once

#include <cstdint>
#include <cstring>

#include "flash.hpp"
#include "libxr_def.hpp"
#include "libxr_type.hpp"
#include DEF2STR(LIBXR_CH32_CONFIG_FILE)

namespace LibXR
{

/**
 * @brief CH32 闪存驱动实现 / CH32 flash driver implementation
 * @note 扇区表须从 Flash 起始地址开始，按地址用若干段列出整片 Flash 的扇区。
 *       The sector table must list the sectors of the whole Flash in address order as
 *       runs, starting at the Flash base.
 */
class CH32Flash : public Flash
{
 public:
  /**
   * @brief 最小写入单位（字节，半字编程）；可直接作为 `DatabaseRaw` 的模板参数 /
   *        Minimum write unit in bytes (half-word programming); usable directly as the
   *        `DatabaseRaw` template argument
   */
  static constexpr size_t MIN_WRITE_SIZE = 2;

  /**
   * @brief 构造闪存对象 / Construct flash object
   * @param regions 扇区表 / Sector table
   * @param region_count 扇区表的段数 / Number of runs in the sector table
   * @param start_address 存储区的起始地址，须正好是某个扇区的起点；存储区一直到 Flash
   *        末尾 / Start address of the storage area; must be exactly the start of a
   *        sector; the area extends to the end of the Flash
   */
  CH32Flash(const FlashRegion* regions, size_t region_count, uint32_t start_address);

  /**
   * @brief 用末尾两个扇区构造，供 DatabaseRaw 的主块和备份块使用 / Construct with the
   *        last two sectors, for the main and backup blocks of DatabaseRaw
   * @param regions 扇区表 / Sector table
   * @param region_count 扇区表的段数 / Number of runs in the sector table
   */
  CH32Flash(const FlashRegion* regions, size_t region_count);

  ErrorCode Erase(size_t offset, size_t size) override;
  ErrorCode Write(size_t offset, ConstRawData data) override;

  static constexpr size_t MinWriteSize()
  {
    return MIN_WRITE_SIZE;
  }  ///< 最小写入粒度（半字） / Minimum write size (half-word)

  static constexpr uint32_t PageSize()
  {
    return 256;
  }  ///< 快速擦除页大小 / Page erase size in fast erase mode

 private:
  const FlashRegion* regions_;
  uint32_t base_address_;
  size_t region_count_;

  bool IsInRange(uint32_t addr, size_t size) const;
  static inline void ClearFlashFlagsOnce();
};

}  // namespace LibXR
