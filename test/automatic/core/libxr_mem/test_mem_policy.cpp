/**
 * @file test_mem_policy.cpp
 * @brief 按编译期策略矩阵穷举验证 Memory::Fast*。 /
 * Exhaustively verifies Memory::Fast* across compile-time policies.
 */

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "libxr_mem.hpp"
#include "test_assert.hpp"

namespace
{
constexpr size_t GUARD_BYTES = 16U;
constexpr size_t MAX_ALIGN_OFF = 8U;
constexpr size_t MAX_TEST_LEN = 257U;
constexpr uint8_t CANARY = 0xEEU;

int Sign(int v) { return (v > 0) - (v < 0); }

void FillPattern(uint8_t* buf, size_t len, uint8_t seed)
{
  for (size_t i = 0U; i < len; ++i)
  {
    buf[i] = static_cast<uint8_t>((seed + i * 17U) ^ (i >> 2U));
  }
}

void VerifyCanaryBuffer(const uint8_t* buf, size_t total_len, size_t region_start,
                        const uint8_t* expected, size_t region_len)
{
  for (size_t i = 0U; i < region_start; ++i)
  {
    TEST_ASSERT(buf[i] == CANARY);
  }
  if (region_len > 0U)
  {
    TEST_ASSERT(std::memcmp(buf + region_start, expected, region_len) == 0);
  }
  for (size_t i = region_start + region_len; i < total_len; ++i)
  {
    TEST_ASSERT(buf[i] == CANARY);
  }
}

void TestZeroLengthAndSelfArgs()
{
  std::array<uint8_t, 128U> buf{};
  std::array<uint8_t, 128U> expected{};

  LibXR::Memory::FastCopy(nullptr, nullptr, 0U);
  LibXR::Memory::FastMove(nullptr, nullptr, 0U);
  LibXR::Memory::FastSet(nullptr, 0xAAU, 0U);
  TEST_ASSERT(LibXR::Memory::FastCmp(nullptr, nullptr, 0U) == 0);

  FillPattern(buf.data(), buf.size(), 0x3AU);
  expected = buf;

  LibXR::Memory::FastCopy(buf.data() + 3U, nullptr, 0U);
  LibXR::Memory::FastCopy(nullptr, buf.data() + 5U, 0U);
  LibXR::Memory::FastMove(buf.data() + 7U, nullptr, 0U);
  LibXR::Memory::FastMove(nullptr, buf.data() + 1U, 0U);
  LibXR::Memory::FastSet(buf.data() + 2U, 0x00U, 0U);
  TEST_ASSERT(LibXR::Memory::FastCmp(buf.data() + 3U, nullptr, 0U) == 0);
  TEST_ASSERT(LibXR::Memory::FastCmp(nullptr, buf.data() + 5U, 0U) == 0);
  TEST_ASSERT(std::memcmp(buf.data(), expected.data(), buf.size()) == 0);

  for (size_t off = 0U; off < MAX_ALIGN_OFF; ++off)
  {
    for (size_t len = 0U; len <= 65U; ++len)
    {
      FillPattern(buf.data(), buf.size(), static_cast<uint8_t>(off * 13U + len));
      expected = buf;

      LibXR::Memory::FastCopy(buf.data() + GUARD_BYTES + off,
                              buf.data() + GUARD_BYTES + off, len);
      TEST_ASSERT(std::memcmp(buf.data(), expected.data(), buf.size()) == 0);

      LibXR::Memory::FastMove(buf.data() + GUARD_BYTES + off,
                              buf.data() + GUARD_BYTES + off, len);
      TEST_ASSERT(std::memcmp(buf.data(), expected.data(), buf.size()) == 0);

      TEST_ASSERT(LibXR::Memory::FastCmp(buf.data() + GUARD_BYTES + off,
                                         buf.data() + GUARD_BYTES + off, len) == 0);
    }
  }
}

void TestCopyAndNonOverlapMoveMatrix()
{
  static constexpr size_t EXTRA_LENS[] = {127U, 128U, 129U, 255U, 256U, 257U};

  constexpr size_t BUF_LEN = GUARD_BYTES + MAX_ALIGN_OFF + MAX_TEST_LEN + GUARD_BYTES;
  alignas(16) std::array<uint8_t, BUF_LEN> src{};
  alignas(16) std::array<uint8_t, BUF_LEN> dst{};

  FillPattern(src.data(), src.size(), 0x5CU);

  auto verify_len = [&](size_t dst_off, size_t src_off, size_t len, bool use_move)
  {
    const size_t dst_start = GUARD_BYTES + dst_off;
    const size_t src_start = GUARD_BYTES + src_off;
    dst.fill(CANARY);
    if (use_move)
    {
      LibXR::Memory::FastMove(dst.data() + dst_start, src.data() + src_start, len);
    }
    else
    {
      LibXR::Memory::FastCopy(dst.data() + dst_start, src.data() + src_start, len);
    }
    VerifyCanaryBuffer(dst.data(), dst.size(), dst_start, src.data() + src_start, len);
  };

  for (size_t dst_off = 0U; dst_off < MAX_ALIGN_OFF; ++dst_off)
  {
    for (size_t src_off = 0U; src_off < MAX_ALIGN_OFF; ++src_off)
    {
      for (size_t len = 0U; len <= 96U; ++len)
      {
        verify_len(dst_off, src_off, len, false);
        if ((len <= 33U) || (len == 64U) || (len == 96U))
        {
          verify_len(dst_off, src_off, len, true);
        }
      }
      for (const size_t len : EXTRA_LENS)
      {
        verify_len(dst_off, src_off, len, false);
        verify_len(dst_off, src_off, len, true);
      }
    }
  }
}

void TestOverlapFastMove()
{
  alignas(16) std::array<uint8_t, 160U> buf{};
  alignas(16) std::array<uint8_t, 160U> ref{};

  for (size_t base_off = 0U; base_off < MAX_ALIGN_OFF; ++base_off)
  {
    for (size_t shift = 1U; shift <= 17U; ++shift)
    {
      for (size_t len = 0U; len <= 80U; ++len)
      {
        // Forward overlap: dst < src
        FillPattern(buf.data(), buf.size(), static_cast<uint8_t>(shift + len));
        ref = buf;
        const size_t fwd_dst = GUARD_BYTES + base_off;
        const size_t fwd_src = fwd_dst + shift;
        LibXR::Memory::FastMove(buf.data() + fwd_dst, buf.data() + fwd_src, len);
        std::memmove(ref.data() + fwd_dst, ref.data() + fwd_src, len);
        TEST_ASSERT(std::memcmp(buf.data(), ref.data(), buf.size()) == 0);

        // Backward overlap: dst > src
        FillPattern(buf.data(), buf.size(), static_cast<uint8_t>(shift * 3U + len));
        ref = buf;
        const size_t bwd_src = GUARD_BYTES + base_off;
        const size_t bwd_dst = bwd_src + shift;
        LibXR::Memory::FastMove(buf.data() + bwd_dst, buf.data() + bwd_src, len);
        std::memmove(ref.data() + bwd_dst, ref.data() + bwd_src, len);
        TEST_ASSERT(std::memcmp(buf.data(), ref.data(), buf.size()) == 0);
      }
    }
  }
}

void TestFastSetMatrix()
{
  static constexpr size_t EXTRA_LENS[] = {127U, 128U, 129U, 255U, 256U, 257U};
  static constexpr uint8_t VALUES[] = {0x00U, 0xA5U, 0xFFU};

  constexpr size_t BUF_LEN = GUARD_BYTES + MAX_ALIGN_OFF + MAX_TEST_LEN + GUARD_BYTES;
  alignas(16) std::array<uint8_t, BUF_LEN> buf{};
  std::array<uint8_t, MAX_TEST_LEN> expected{};

  auto verify_set = [&](size_t off, size_t len, uint8_t val)
  {
    buf.fill(CANARY);
    expected.fill(val);
    LibXR::Memory::FastSet(buf.data() + GUARD_BYTES + off, val, len);
    VerifyCanaryBuffer(buf.data(), buf.size(), GUARD_BYTES + off, expected.data(), len);
  };

  for (size_t off = 0U; off < MAX_ALIGN_OFF; ++off)
  {
    for (size_t len = 0U; len <= 96U; ++len)
    {
      verify_set(off, len, VALUES[(off + len) % 3U]);
    }
    for (const size_t len : EXTRA_LENS)
    {
      for (const uint8_t val : VALUES)
      {
        verify_set(off, len, val);
      }
    }
  }
}

void TestFastCmpMatrix()
{
  static constexpr size_t LENS[] = {0U,  1U,  3U,  7U,  8U,  9U,  15U,  16U, 17U,
                                    31U, 32U, 33U, 64U, 65U, 96U, 128U, 257U};
  constexpr size_t BUF_LEN = GUARD_BYTES + MAX_ALIGN_OFF + MAX_TEST_LEN + GUARD_BYTES;
  alignas(16) std::array<uint8_t, BUF_LEN> a{};
  alignas(16) std::array<uint8_t, BUF_LEN> b{};

  for (size_t a_off = 0U; a_off < MAX_ALIGN_OFF; ++a_off)
  {
    for (size_t b_off = 0U; b_off < MAX_ALIGN_OFF; ++b_off)
    {
      for (const size_t len : LENS)
      {
        const size_t a_start = GUARD_BYTES + a_off;
        const size_t b_start = GUARD_BYTES + b_off;

        FillPattern(a.data(), a.size(), 0x2DU);
        b.fill(0x55U);
        if (len > 0U)
        {
          std::memcpy(b.data() + b_start, a.data() + a_start, len);
        }

        TEST_ASSERT(LibXR::Memory::FastCmp(a.data() + a_start, b.data() + b_start, len) ==
                    0);

        if (len > 0U)
        {
          const size_t positions[] = {0U, len / 2U, len - 1U};
          for (const size_t pos : positions)
          {
            const uint8_t orig = b[b_start + pos];
            b[b_start + pos] = static_cast<uint8_t>(orig ^ 0x01U);
            const int got =
                LibXR::Memory::FastCmp(a.data() + a_start, b.data() + b_start, len);
            const int want = std::memcmp(a.data() + a_start, b.data() + b_start, len);
            TEST_ASSERT(Sign(got) == Sign(want));
            b[b_start + pos] = orig;
          }
        }
      }
    }
  }
}
}  // namespace

int main()
{
  TestZeroLengthAndSelfArgs();
  TestCopyAndNonOverlapMoveMatrix();
  TestOverlapFastMove();
  TestFastSetMatrix();
  TestFastCmpMatrix();
  return 0;
}
