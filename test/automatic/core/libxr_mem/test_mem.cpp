/**
 * @file test_mem.cpp
 * @brief 检查内存填充、复制和比较。 /
 * Tests memory fill, copy and comparison.
 *
 * 包含零长度、非对齐地址和复制范围边界；比较结果与 memcmp 的正负号一致。
 * Includes zero lengths, unaligned addresses, copy bounds and memcmp sign checks.
 */

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "libxr.hpp"
#include "libxr_def.hpp"
#include "libxr_mem.hpp"
#include "test.hpp"
#include "test_assert.hpp"

static int sign(int v) { return (v > 0) - (v < 0); }

void test_memory()
{
  // FastSet: 基础填充与 size=0
  {
    uint8_t buf[64] = {};
    LibXR::Memory::FastSet(buf, 0xAA, sizeof(buf));
    for (size_t i = 0; i < sizeof(buf); ++i)
    {
      TEST_ASSERT(buf[i] == 0xAA);
    }

    // size = 0: 不应修改
    uint8_t guard = 0x5A;
    LibXR::Memory::FastSet(&guard, 0x00, 0);
    TEST_ASSERT(guard == 0x5A);
  }
  // FastCopy: 基础拷贝、size=0、自拷贝、非对齐地址、不同长度
  {
    uint8_t src[128];
    uint8_t dst[128];

    for (size_t i = 0; i < sizeof(src); ++i)
    {
      src[i] = static_cast<uint8_t>(i ^ 0x5C);
      dst[i] = 0x00;
    }

    // 基础整段拷贝
    LibXR::Memory::FastCopy(dst, src, sizeof(src));
    TEST_ASSERT(std::memcmp(dst, src, sizeof(src)) == 0);

    // size = 0: 不应修改 dst，且支持空指针零长度调用
    dst[17] = 0x11;
    LibXR::Memory::FastCopy(dst, src, 0);
    LibXR::Memory::FastCopy(nullptr, nullptr, 0);
    TEST_ASSERT(dst[17] == 0x11);

    // dst == src: 自拷贝不应破坏数据
    uint8_t backup[128];
    std::memcpy(backup, src, sizeof(src));
    LibXR::Memory::FastCopy(src, src, sizeof(src));
    TEST_ASSERT(std::memcmp(src, backup, sizeof(src)) == 0);

    // 非重叠 FastMove 应与 FastCopy 行为一致
    LibXR::Memory::FastSet(dst, 0x00, sizeof(dst));
    LibXR::Memory::FastMove(dst + 3, src + 1, 61);
    TEST_ASSERT(std::memcmp(dst + 3, src + 1, 61) == 0);
    TEST_ASSERT(dst[2] == 0x00 && dst[64] == 0x00);

    // 非对齐地址拷贝：dst+1 <- src+3
    uint8_t src2[200];
    uint8_t dst2[200];
    for (size_t i = 0; i < sizeof(src2); ++i)
    {
      src2[i] = static_cast<uint8_t>(0xA5 ^ (i * 3));
      dst2[i] = 0xCC;
    }

    const size_t OFF_DST = 1;
    const size_t OFF_SRC = 3;
    const size_t LEN = 73;  // 非 4/8 对齐长度
    LibXR::Memory::FastCopy(dst2 + OFF_DST, src2 + OFF_SRC, LEN);
    TEST_ASSERT(std::memcmp(dst2 + OFF_DST, src2 + OFF_SRC, LEN) == 0);

    // 小尺寸/多种长度覆盖（包含 1..65）
    for (size_t n = 1; n <= 65; ++n)
    {
      for (size_t i = 0; i < sizeof(src); ++i)
      {
        src[i] = static_cast<uint8_t>(i + n);
        dst[i] = 0xEE;
      }
      LibXR::Memory::FastCopy(dst + 7, src + 5, n);  // 故意偏移
      TEST_ASSERT(std::memcmp(dst + 7, src + 5, n) == 0);
      // 确保范围外不被写（简单哨兵检查）
      TEST_ASSERT(dst[6] == 0xEE);
      TEST_ASSERT(dst[7 + n] == 0xEE);
    }

    // 错位相位全覆盖与关键边界长度检查 / Phase and boundary-length regression
    {
      struct OffsetPair
      {
        size_t dst_off;
        size_t src_off;
      };
      static constexpr OffsetPair OFFSET_PAIRS[] = {
          {0, 1}, {0, 2}, {0, 3}, {0, 4}, {0, 5}, {0, 6},
          {0, 7}, {1, 0}, {2, 5}, {3, 1}, {5, 2}, {7, 4},
      };
      static constexpr size_t BOUNDARY_LENS[] = {
          0, 1, 3, 7, 8, 9, 15, 16, 17, 23, 24, 31, 32, 33, 63, 64, 65, 97,
      };

      alignas(8) uint8_t src_buf[16 + 128];
      alignas(8) uint8_t dst_buf[16 + 128];
      uint8_t* const src_base = src_buf + 8;
      uint8_t* const dst_base = dst_buf + 8;

      for (size_t i = 0; i < sizeof(src_buf); ++i)
      {
        src_buf[i] = static_cast<uint8_t>(0x5A ^ (i * 13));
      }

      for (const auto& pair : OFFSET_PAIRS)
      {
        for (const size_t len : BOUNDARY_LENS)
        {
          std::memset(dst_buf, 0xEE, sizeof(dst_buf));
          LibXR::Memory::FastCopy(dst_base + pair.dst_off, src_base + pair.src_off, len);
          TEST_ASSERT(dst_base[pair.dst_off - 1] == 0xEE);
          TEST_ASSERT(dst_base[pair.dst_off + len] == 0xEE);
          if (len > 0)
          {
            TEST_ASSERT(
                std::memcmp(dst_base + pair.dst_off, src_base + pair.src_off, len) == 0);
          }
        }
      }
    }
  }
  // FastCmp: 等于/不等，差异位置覆盖，并与 std::memcmp 符号对齐
  {
    uint8_t a[96];
    uint8_t b[96];

    for (size_t i = 0; i < sizeof(a); ++i)
    {
      a[i] = static_cast<uint8_t>(0x3C ^ (i * 7));
      b[i] = a[i];
    }

    // 完全相等
    TEST_ASSERT(LibXR::Memory::FastCmp(a, b, sizeof(a)) == 0);

    // 首字节差异
    b[0] = static_cast<uint8_t>(b[0] + 1);
    int r1 = LibXR::Memory::FastCmp(a, b, sizeof(a));
    int m1 = std::memcmp(a, b, sizeof(a));
    TEST_ASSERT(sign(r1) == sign(m1));
    b[0] = a[0];

    // 中间差异
    b[37] = static_cast<uint8_t>(b[37] - 1);
    int r2 = LibXR::Memory::FastCmp(a, b, sizeof(a));
    int m2 = std::memcmp(a, b, sizeof(a));
    TEST_ASSERT(sign(r2) == sign(m2));
    b[37] = a[37];

    // 末尾差异
    b[95] = static_cast<uint8_t>(b[95] + 5);
    int r3 = LibXR::Memory::FastCmp(a, b, sizeof(a));
    int m3 = std::memcmp(a, b, sizeof(a));
    TEST_ASSERT(sign(r3) == sign(m3));
    b[95] = a[95];

    // size = 0：应视为相等（对齐 memcmp 语义）
    TEST_ASSERT(LibXR::Memory::FastCmp(a, b, 0) == 0);

    // 非对齐指针比较
    uint8_t c[128];
    uint8_t d[128];
    for (size_t i = 0; i < sizeof(c); ++i)
    {
      c[i] = static_cast<uint8_t>(i * 11);
      d[i] = c[i];
    }
    // 制造一个差异
    d[19] ^= 0x01;

    const size_t OFF1 = 1;
    const size_t OFF2 = 3;
    const size_t N = 64;
    int r4 = LibXR::Memory::FastCmp(c + OFF1, d + OFF2, N);
    int m4 = std::memcmp(c + OFF1, d + OFF2, N);
    TEST_ASSERT(sign(r4) == sign(m4));
  }
}
