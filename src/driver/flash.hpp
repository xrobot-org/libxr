#pragma once

#include <cstdint>
#include <cstring>

#include "libxr_def.hpp"
#include "libxr_rw.hpp"
#include "libxr_type.hpp"

namespace LibXR
{

/**
 * @brief 一段连续、大小相同的扇区 / A run of contiguous sectors of equal size
 * @note 驱动的扇区表由若干段组成，按地址顺序列出整片 Flash。
 *       A driver's sector table is made of such runs and lists the whole Flash in address
 *       order.
 */
struct FlashRegion
{
  uint32_t address;       ///< 第一个扇区的地址 / Address of the first sector
  uint32_t sector_size;   ///< 每个扇区的字节数 / Bytes per sector
  uint32_t sector_count;  ///< 扇区个数 / Number of sectors
};

/**
 * @class Flash
 * @brief 抽象基类，表示闪存接口。
 * Abstract base class representing a flash memory interface.
 */
class Flash
{
 public:
  /**
   * @brief 构造函数，初始化闪存属性。
   * Constructs a Flash object with specified properties.
   * @param min_erase_size 最小可擦除块大小（字节）。
   * Minimum erasable block size in bytes.
   * @param min_write_size 最小可写块大小（字节） / Minimum writable block size in bytes.
   * @param flash_area 用于闪存操作的存储区域。
   * Memory area allocated for flash operations.
   */
  Flash(size_t min_erase_size, size_t min_write_size, RawData flash_area);

  /**
   * @brief 擦除闪存的指定区域 / Erases a section of the flash memory.
   * @param offset 要擦除的起始偏移地址 / The starting offset of the section to erase.
   * @param size 要擦除的区域大小 / The size of the section to erase.
   * @return 返回操作结果的错误码 / ErrorCode indicating success or failure.
   */
  virtual ErrorCode Erase(size_t offset, size_t size) = 0;

  /**
   * @brief 向闪存写入数据 / Writes data to the flash memory.
   * @param offset 数据写入的起始偏移地址 / The starting offset to write data.
   * @param data 需要写入的数据 / The data to be written.
   * @return 返回操作结果的错误码 / ErrorCode indicating success or failure.
   */
  virtual ErrorCode Write(size_t offset, ConstRawData data) = 0;

  /**
   * @brief 从闪存中读取数据 / Reads data from the flash memory.
   *
   * @param offset 数据读取的起始偏移地址 / The starting offset to read data.
   * @param data 存储读取数据的缓冲区 / Data buffer to store the read data.
   * @return 返回操作结果的错误码 / ErrorCode indicating success or failure.
   */
  virtual ErrorCode Read(size_t offset, RawData data);

  /**
   * @brief 获取最小可擦除块大小（字节）。
   * Returns the minimum erasable block size in bytes.
   *
   * @return 最小可擦除块大小（字节） / size_t Minimum erasable block size in bytes.
   */
  size_t MinEraseSize() const { return min_erase_size_; }

  /**
   * @brief 获取最小可写块大小（字节） / Returns the minimum writable block size in bytes.
   *
   * @return 最小可写块大小（字节） / size_t Minimum writable block size in bytes.
   */
  size_t MinWriteSize() const { return min_write_size_; }

  /**
   * @brief 获取闪存存储区域的大小 / Returns the size of the flash memory area.
   *
   * @return 闪存存储区域的大小 / size_t Size of the flash memory area.
   */
  size_t Size() const { return flash_area_.size_; }

 private:
  size_t min_erase_size_ =
      0;  ///< 最小可擦除块大小（字节） / Minimum erasable block size in bytes.
  size_t min_write_size_ =
      0;  ///< 最小可写块大小（字节） / Minimum writable block size in bytes.
  RawData flash_area_;  ///< 用于闪存操作的存储区域。
                        ///< Memory area allocated for flash operations.
};

}  // namespace LibXR
