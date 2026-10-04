#pragma once

#include <cstdint>
#include <cstdio>
#include <cstring>

#include "libxr_def.hpp"

namespace LibXR
{
/**
 * @class CRC8
 * @brief 8 位循环冗余校验（CRC-8）计算类 / CRC-8 checksum computation class
 *
 * 该类实现了 CRC-8 校验算法，支持计算和验证数据的 CRC8 校验码。
 * This class implements the CRC-8 checksum algorithm, supporting computation and
 * verification.
 */
class CRC8
{
 private:
  static const uint8_t INIT = 0xFF;  ///< CRC8 初始值 / CRC8 initial value

 public:
  static inline uint8_t tab_[256];  ///< CRC8 查找表 / CRC8 lookup table
  static inline bool inited_ =
      false;  ///< 查找表是否已初始化 / Whether the lookup table is initialized

  CRC8() {}

  /**
   * @brief 生成 CRC8 查找表 / Generates the CRC8 lookup table
   */
  static void GenerateTable()
  {
    uint8_t crc = 0;

    for (int i = 0; i < 256; i++)
    {
      tab_[i] = i;
    }

    for (int i = 0; i < 256; i++)
    {
      for (int j = 7; j >= 0; j--)
      {
        crc = tab_[i] & 0x01;

        if (crc)
        {
          tab_[i] = tab_[i] >> 1;
          tab_[i] ^= 0x8c;
        }
        else
        {
          tab_[i] = tab_[i] >> 1;
        }
      }
    }
    inited_ = true;
  }

  /**
   * @brief 计算数据的 CRC8 校验码 / Computes the CRC8 checksum for the given data
   * @param raw 输入数据指针 / Pointer to input data
   * @param len 数据长度 / Length of the data
   * @return 计算得到的 CRC8 值 / Computed CRC8 value
   */
  static uint8_t Calculate(const void* raw, size_t len);

  /**
   * @brief 验证数据的 CRC8 校验码 / Verifies the CRC8 checksum of the given data
   * @param raw 输入数据指针 / Pointer to input data
   * @param len 数据长度 / Length of the data
   * @return 校验成功返回 `true`，否则返回 `false` /
   *         Returns `true` if the checksum is valid, otherwise returns `false`
   */
  static bool Verify(const void* raw, size_t len)
  {
    const uint8_t* buf = reinterpret_cast<const uint8_t*>(raw);
    if (!inited_)
    {
      GenerateTable();
    }

    if (len < 2)
    {
      return false;
    }
    uint8_t expected = Calculate(buf, len - sizeof(uint8_t));
    return expected == buf[len - sizeof(uint8_t)];
  }
};

/**
 * @class CRC16
 * @brief 16 位循环冗余校验（CRC-16）计算类 / CRC-16 checksum computation class
 *
 * 该类实现了 CRC-16 校验算法，支持计算和验证数据的 CRC16 校验码。
 * This class implements the CRC-16 checksum algorithm, supporting computation and
 * verification.
 */
class CRC16
{
 private:
  static const uint16_t INIT = 0xFFFF;  ///< CRC16 初始值 / CRC16 initial value

 public:
  static inline uint16_t tab_[256];  ///< CRC16 查找表 / CRC16 lookup table
  static inline bool inited_ =
      false;  ///< 查找表是否已初始化 / Whether the lookup table is initialized
  CRC16() {}

  /**
   * @brief 生成 CRC16 查找表 / Generates the CRC16 lookup table
   */
  static void GenerateTable()
  {
    uint16_t crc = 0;
    for (int i = 0; i < 256; ++i)
    {
      crc = i;
      for (int j = 0; j < 8; ++j)
      {
        if (crc & 1)
        {
          crc = (crc >> 1) ^ 0x8408;
        }
        else
        {
          crc >>= 1;
        }
      }
      tab_[i] = crc;
    }
    inited_ = true;
  }

  /**
   * @brief 计算数据的 CRC16 校验码 / Computes the CRC16 checksum for the given data
   * @param raw 输入数据指针 / Pointer to input data
   * @param len 数据长度 / Length of the data
   * @return 计算得到的 CRC16 值 / Computed CRC16 value
   */
  static uint16_t Calculate(const void* raw, size_t len);

  /**
   * @brief 验证数据的 CRC16 校验码 / Verifies the CRC16 checksum of the given data
   * @param raw 输入数据指针 / Pointer to input data
   * @param len 数据长度 / Length of the data
   * @return 校验成功返回 `true`，否则返回 `false` /
   *         Returns `true` if the checksum is valid, otherwise returns `false`
   */
  static bool Verify(const void* raw, size_t len)
  {
    const uint8_t* buf = reinterpret_cast<const uint8_t*>(raw);
    if (!inited_)
    {
      GenerateTable();
    }

    if (len < 2)
    {
      return false;
    }

    uint16_t actual = 0;
    std::memcpy(&actual, buf + len - sizeof(actual), sizeof(actual));

    uint16_t expected = Calculate(buf, len - sizeof(uint16_t));
    return expected == actual;
  }
};

/**
 * @class CRC32
 * @brief 32 位循环冗余校验（CRC-32）计算类 / CRC-32 checksum computation class
 *
 * 该类实现了 CRC-32 校验算法，支持计算和验证数据的 CRC32 校验码。
 * This class implements the CRC-32 checksum algorithm, supporting computation and
 * verification.
 */
class CRC32
{
 private:
  static const uint32_t INIT = 0xFFFFFFFF;  ///< CRC32 初始值 / CRC32 initial value

 public:
  static inline uint32_t tab_[256];  ///< CRC32 查找表 / CRC32 lookup table
  static inline bool inited_ =
      false;  ///< 查找表是否已初始化 / Whether the lookup table is initialized

  CRC32() {}

  /**
   * @brief 生成 CRC32 查找表 / Generates the CRC32 lookup table
   */
  static void GenerateTable()
  {
    uint32_t crc = 0;
    for (int i = 0; i < 256; ++i)
    {
      crc = i;
      for (int j = 0; j < 8; ++j)
      {
        if (crc & 1)
        {
          crc = (crc >> 1) ^ 0xEDB88320;
        }
        else
        {
          crc >>= 1;
        }
      }
      tab_[i] = crc;
    }
    inited_ = true;
  }

  /**
   * @brief 计算数据的 CRC32 校验码 / Computes the CRC32 checksum for the given data
   * @param raw 输入数据指针 / Pointer to input data
   * @param len 数据长度 / Length of the data
   * @return 计算得到的 CRC32 值 / Computed CRC32 value
   */
  static uint32_t Calculate(const void* raw, size_t len);

  /**
   * @brief 验证数据的 CRC32 校验码 / Verifies the CRC32 checksum of the given data
   * @param raw 输入数据指针 / Pointer to input data
   * @param len 数据长度 / Length of the data
   * @return 校验成功返回 `true`，否则返回 `false` /
   *         Returns `true` if the checksum is valid, otherwise returns `false`
   */
  static bool Verify(const void* raw, size_t len)
  {
    const uint8_t* buf = reinterpret_cast<const uint8_t*>(raw);
    if (!inited_)
    {
      GenerateTable();
    }

    if (len < sizeof(uint32_t))
    {
      return false;
    }

    uint32_t actual = 0;
    std::memcpy(&actual, buf + len - sizeof(actual), sizeof(actual));

    uint32_t expected = Calculate(buf, len - sizeof(uint32_t));
    return expected == actual;
  }
};

/**
 * @class CRC64
 * @brief 64 位循环冗余校验（CRC-64）计算类 / CRC-64 checksum computation class
 *
 * 该类实现了 CRC-64 校验算法，支持计算数据的 64 位校验值。
 * This class implements the CRC-64 checksum algorithm and supports computing
 * 64-bit checksums for input data.
 */
class CRC64
{
 private:
  static const uint64_t INIT =
      0xFFFFFFFFFFFFFFFFULL;  ///< CRC64 初始值 / CRC64 initial value

 public:
  static inline uint64_t tab_[256];  ///< CRC64 查找表 / CRC64 lookup table
  static inline bool inited_ =
      false;  ///< 查找表是否已初始化 / Whether the lookup table is initialized

  CRC64() {}

  /**
   * @brief 生成 CRC64 查找表 / Generates the CRC64 lookup table
   */
  static void GenerateTable()
  {
    uint64_t crc = 0;
    for (int i = 0; i < 256; ++i)
    {
      crc = static_cast<uint64_t>(i);
      for (int j = 0; j < 8; ++j)
      {
        if (crc & 1ULL)
        {
          crc = (crc >> 1U) ^ 0xC96C5795D7870F42ULL;
        }
        else
        {
          crc >>= 1U;
        }
      }
      tab_[i] = crc;
    }
    inited_ = true;
  }

  /**
   * @brief 计算数据的 CRC64 校验码 / Computes the CRC64 checksum for the given data
   * @param raw 输入数据指针 / Pointer to input data
   * @param len 数据长度 / Length of the data
   * @return 计算得到的 CRC64 值 / Computed CRC64 value
   */
  static uint64_t Calculate(const void* raw, size_t len);
};
}  // namespace LibXR
