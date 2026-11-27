#include <bit>
#include <cstring>

#include "libxr_def.hpp"
#include "libxr_mem.hpp"
#include "libxr_mem_policy.hpp"

namespace
{
using LibXR::MemoryDetail::ACTIVE_FASTCOPY_POLICY;
using LibXR::MemoryDetail::ACTIVE_FASTMOVE_POLICY;
using LibXR::MemoryDetail::ACTIVE_FASTSET_POLICY;
using LibXR::MemoryDetail::BasicPolicy;
using LibXR::MemoryDetail::FastCopyPolicy;
using LibXR::MemoryDetail::HAS_FAST_UNALIGNED_WORD;
using LibXR::MemoryDetail::WORD_SIZE;

// 按小端字节序前向预读源端错位字节并移位拼字后对齐写入 32 位目标字。
// Forward-preload lead bytes from an unaligned source and emit aligned 32-bit
// words by shifting and combining in little-endian order.
template <unsigned LEAD_BYTES>
[[maybe_unused]] LIBXR_ALWAYS_INLINE void CopyShiftedLittleEndian(uint8_t*& d,
                                                                  const uint8_t*& s,
                                                                  size_t& size)
{
  static_assert(LEAD_BYTES > 0U && LEAD_BYTES < sizeof(uint32_t));
  constexpr unsigned SHIFT_L = LEAD_BYTES * 8U;
  constexpr unsigned SHIFT_R = 32U - SHIFT_L;

  uint32_t carry = 0U;
  for (unsigned i = 0U; i < LEAD_BYTES; ++i)
  {
    carry |= static_cast<uint32_t>(s[i]) << (i * 8U);
  }
  s += LEAD_BYTES;
  size -= LEAD_BYTES;

  auto* dw = reinterpret_cast<uint32_t*>(d);
  const auto* sw = reinterpret_cast<const uint32_t*>(s);

  while (size >= 16U)
  {
    uint32_t w = sw[0];
    dw[0] = carry | (w << SHIFT_L);
    carry = w >> SHIFT_R;
    w = sw[1];
    dw[1] = carry | (w << SHIFT_L);
    carry = w >> SHIFT_R;
    w = sw[2];
    dw[2] = carry | (w << SHIFT_L);
    carry = w >> SHIFT_R;
    w = sw[3];
    dw[3] = carry | (w << SHIFT_L);
    carry = w >> SHIFT_R;
    sw += 4;
    dw += 4;
    size -= 16U;
  }

  while (size >= 4U)
  {
    const uint32_t w = *sw++;
    *dw++ = carry | (w << SHIFT_L);
    carry = w >> SHIFT_R;
    size -= 4U;
  }

  // carry 中仍保留未写出的 LEAD_BYTES 字节，退回指针和长度由尾部循环收尾。
  // Return the unwritten LEAD_BYTES bytes still held in carry to the byte tail.
  d = reinterpret_cast<uint8_t*>(dw);
  s = reinterpret_cast<const uint8_t*>(sw) - LEAD_BYTES;
  size += LEAD_BYTES;
}

// 按给定字宽先批量读入 8 个字再写回，避免编译器生成运行期别名降级分支并保持突发访存。
// Copy aligned words by loading 8 words before storing them, preventing
// runtime alias-check fallback loops while preserving burst transfers.
template <typename WordT>
[[maybe_unused]] LIBXR_ALWAYS_INLINE void CopyAlignedWords(uint8_t*& d, const uint8_t*& s,
                                                           size_t& size)
{
  constexpr size_t WORD_BYTES = sizeof(WordT);
  constexpr size_t BURST_BYTES = WORD_BYTES * 8U;

  auto* dw = reinterpret_cast<WordT*>(d);
  const auto* sw = reinterpret_cast<const WordT*>(s);

  while (size >= BURST_BYTES)
  {
    const WordT a0 = sw[0];
    const WordT a1 = sw[1];
    const WordT a2 = sw[2];
    const WordT a3 = sw[3];
    const WordT a4 = sw[4];
    const WordT a5 = sw[5];
    const WordT a6 = sw[6];
    const WordT a7 = sw[7];
    dw[0] = a0;
    dw[1] = a1;
    dw[2] = a2;
    dw[3] = a3;
    dw[4] = a4;
    dw[5] = a5;
    dw[6] = a6;
    dw[7] = a7;
    dw += 8;
    sw += 8;
    size -= BURST_BYTES;
  }

  while (size >= WORD_BYTES)
  {
    *dw++ = *sw++;
    size -= WORD_BYTES;
  }

  d = reinterpret_cast<uint8_t*>(dw);
  s = reinterpret_cast<const uint8_t*>(sw);
}

// 偶地址差场景下的 16 位对齐拷贝辅助：先处理奇起始字节，再以 uint16_t 展开拷贝。
// 16-bit aligned copy helper for even address differences: peel one odd byte if
// needed, then burst-copy in uint16_t units.
[[maybe_unused]] LIBXR_ALWAYS_INLINE void CopyAligned16(uint8_t*& d, const uint8_t*& s,
                                                        size_t& size)
{
  if ((reinterpret_cast<uintptr_t>(d) & 1U) && (size > 0U))
  {
    *d++ = *s++;
    --size;
  }
  CopyAlignedWords<uint16_t>(d, s, size);
}

// 32 位小端错位前向对齐目标指针后，按源端相位差分发移位拼字模板。
// Align destination to 4 bytes on 32-bit little-endian targets, then dispatch the
// shift-and-combine template for the remaining source phase.
template <FastCopyPolicy POLICY>
[[maybe_unused]] LIBXR_ALWAYS_INLINE void CopyMisalignedShifted32(uint8_t*& d,
                                                                  const uint8_t*& s,
                                                                  uintptr_t d_offset,
                                                                  size_t& size)
{
  if (size < 8U)
  {
    return;
  }

  if (d_offset != 0U)
  {
    size_t head = 4U - d_offset;
    while (head--)
    {
      *d++ = *s++;
      --size;
    }
  }

  const unsigned lead_bytes =
      static_cast<unsigned>(4U - (reinterpret_cast<uintptr_t>(s) & 3U));
  if (size >= 4U + lead_bytes)
  {
    if (lead_bytes == 1U)
    {
      CopyShiftedLittleEndian<1U>(d, s, size);
    }
    else if constexpr (POLICY == FastCopyPolicy::SHIFTED_ODD)
    {
      CopyShiftedLittleEndian<3U>(d, s, size);
    }
    else if (lead_bytes == 2U)
    {
      CopyShiftedLittleEndian<2U>(d, s, size);
    }
    else
    {
      CopyShiftedLittleEndian<3U>(d, s, size);
    }
  }
}

// 按编译期策略和字宽分发 FastCopy 实现；策略切换不引入运行时分支。
// Compile-time FastCopy dispatch across word size and misalignment policy.
template <size_t WORD_BYTES, FastCopyPolicy POLICY>
LIBXR_ALWAYS_INLINE void FastCopyImpl(void* dst, const void* src, size_t size)
{
  static_assert(WORD_BYTES == 4U || WORD_BYTES == 8U);

  // 所有策略统一保留零长度与自拷贝早退契约，避免 std::memcpy UB 或只读区原地写回。
  // Enforce zero-length and self-copy early return across all policies to avoid
  // std::memcpy UB and prevent read-modify-write faults on read-only memory.
  if ((size == 0U) || (dst == src))
  {
    return;
  }

  if constexpr (POLICY == FastCopyPolicy::STD_LIBC)
  {
    std::memcpy(dst, src, size);
  }
  else if constexpr (POLICY == FastCopyPolicy::UNALIGNED_WORD)
  {
    static_assert((POLICY != FastCopyPolicy::UNALIGNED_WORD) || HAS_FAST_UNALIGNED_WORD,
                  "LIBXR_FASTCOPY_POLICY_UNALIGNED_WORD requires hardware unaligned word "
                  "access");

    // 仅在硬件支持非对齐字访存的架构下直接执行 4x 非对齐 32 位拷贝。
    // Emit unaligned 32-bit loads/stores directly on targets with hardware support.
    auto* d = static_cast<uint8_t*>(dst);
    const auto* s = static_cast<const uint8_t*>(src);

    while (size >= 16U)
    {
      uint32_t w0 = 0U;
      uint32_t w1 = 0U;
      uint32_t w2 = 0U;
      uint32_t w3 = 0U;
      std::memcpy(&w0, s, 4U);
      std::memcpy(&w1, s + 4U, 4U);
      std::memcpy(&w2, s + 8U, 4U);
      std::memcpy(&w3, s + 12U, 4U);
      std::memcpy(d, &w0, 4U);
      std::memcpy(d + 4U, &w1, 4U);
      std::memcpy(d + 8U, &w2, 4U);
      std::memcpy(d + 12U, &w3, 4U);
      d += 16U;
      s += 16U;
      size -= 16U;
    }

    while (size >= 4U)
    {
      uint32_t w = 0U;
      std::memcpy(&w, s, 4U);
      std::memcpy(d, &w, 4U);
      d += 4U;
      s += 4U;
      size -= 4U;
    }

    while (size--)
    {
      *d++ = *s++;
    }
  }
  else
  {
    uint8_t* d = static_cast<uint8_t*>(dst);
    const uint8_t* s = static_cast<const uint8_t*>(src);

    const uintptr_t d_offset = reinterpret_cast<uintptr_t>(d) & (WORD_BYTES - 1U);
    const uintptr_t s_offset = reinterpret_cast<uintptr_t>(s) & (WORD_BYTES - 1U);

    // 源与目的对齐偏移相同时，先补齐前导字节再做字宽突发拷贝。
    // When source and destination share alignment offset, align then burst-copy.
    if (d_offset == s_offset)
    {
      if (d_offset != 0U)
      {
        size_t head = WORD_BYTES - d_offset;
        if (head > size)
        {
          head = size;
        }
        while (head--)
        {
          *d++ = *s++;
          --size;
        }
      }

      if constexpr (WORD_BYTES == 8U)
      {
        CopyAlignedWords<uint64_t>(d, s, size);
      }
      else
      {
        CopyAlignedWords<uint32_t>(d, s, size);
      }
    }
    // 对齐偏移不同：64 位目标在 4 字节对齐差保留 32 位拷，其余小端错位场景按策略
    // 选择 16 位对齐拷或 32 位移位拼字，避免奇相位差退化为逐字节循环；32 位小端同样
    // 按策略选择仅奇相位或全相位 32 位移位拼字。
    // Misaligned offsets: on 64-bit targets keep 32-bit word copy for 4-byte phase
    // differences and apply 16-bit aligned or 32-bit shift-and-combine on little-endian
    // targets so odd phase diffs avoid byte loops; 32-bit little-endian uses the same
    // policy-selected shifted path.
    else
    {
      if constexpr (WORD_BYTES == 8U)
      {
        const uintptr_t addr_diff =
            reinterpret_cast<uintptr_t>(s) - reinterpret_cast<uintptr_t>(d);

        // 地址差为 4 的倍数时使用 32 位拷贝。
        // If address difference is a multiple of 4, use 4-byte copying.
        if ((addr_diff & 3U) == 0U)
        {
          while ((reinterpret_cast<uintptr_t>(d) & 3U) && (size > 0U))
          {
            *d++ = *s++;
            --size;
          }
          CopyAlignedWords<uint32_t>(d, s, size);
        }
        else if constexpr (POLICY == FastCopyPolicy::SHIFTED_ALL &&
                           std::endian::native == std::endian::little)
        {
          CopyMisalignedShifted32<POLICY>(d, s, reinterpret_cast<uintptr_t>(d) & 3U,
                                          size);
        }
        // 地址差为偶数时使用 16 位拷贝。
        // If address difference is even, use 2-byte copying.
        else if ((addr_diff & 1U) == 0U)
        {
          CopyAligned16(d, s, size);
        }
        else if constexpr (std::endian::native == std::endian::little)
        {
          CopyMisalignedShifted32<FastCopyPolicy::SHIFTED_ODD>(
              d, s, reinterpret_cast<uintptr_t>(d) & 3U, size);
        }
      }
      else if constexpr (WORD_BYTES == 4U && std::endian::native == std::endian::little)
      {
        if constexpr (POLICY == FastCopyPolicy::SHIFTED_ODD)
        {
          const uintptr_t addr_diff =
              reinterpret_cast<uintptr_t>(s) - reinterpret_cast<uintptr_t>(d);
          // 偶相位差保留 16 位对齐路径，仅奇相位差走移位拼字模板。
          // Keep the 16-bit aligned path on even differences; shift-combine odd diffs.
          if ((addr_diff & 1U) == 0U && size > 0U)
          {
            CopyAligned16(d, s, size);
          }
          else
          {
            CopyMisalignedShifted32<POLICY>(d, s, d_offset, size);
          }
        }
        else
        {
          CopyMisalignedShifted32<POLICY>(d, s, d_offset, size);
        }
      }
      else
      {
        const uintptr_t addr_diff =
            reinterpret_cast<uintptr_t>(s) - reinterpret_cast<uintptr_t>(d);
        if ((addr_diff & 1U) == 0U && size > 0U)
        {
          CopyAligned16(d, s, size);
        }
      }
    }

    // 拷贝剩余尾部字节。
    // Copy any remaining tail bytes.
    while (size--)
    {
      *d++ = *s++;
    }
  }
}

// 无需循环地填充不足一个字的零散字节，避免 GCC -O3 把短循环替换成 memset 调用。
// Fill strictly fewer than WORD_BYTES bytes without a byte loop so GCC -O3 does
// not rewrite short head/tail byte peels into external memset calls.
template <size_t WORD_BYTES>
[[maybe_unused]] LIBXR_ALWAYS_INLINE void FillSmallBytes(uint8_t*& d, uint8_t value,
                                                         size_t count)
{
  static_assert(WORD_BYTES == 4U || WORD_BYTES == 8U);
  if constexpr (WORD_BYTES == 8U)
  {
    if (count >= 4U)
    {
      d[0] = value;
      d[1] = value;
      d[2] = value;
      d[3] = value;
      d += 4U;
      count -= 4U;
    }
  }
  if (count >= 2U)
  {
    d[0] = value;
    d[1] = value;
    d += 2U;
    count -= 2U;
  }
  if (count != 0U)
  {
    *d++ = value;
  }
}
}  // namespace

void LibXR::Memory::FastCopy(void* dst, const void* src, size_t size)
{
  FastCopyImpl<WORD_SIZE, ACTIVE_FASTCOPY_POLICY>(dst, src, size);
}

void LibXR::Memory::FastMove(void* dst, const void* src, size_t size)
{
  if ((size == 0U) || (dst == src))
  {
    return;
  }

  if constexpr (ACTIVE_FASTMOVE_POLICY == BasicPolicy::STD_LIBC)
  {
    std::memmove(dst, src, size);
  }
  else
  {
    auto* d = static_cast<uint8_t*>(dst);
    const auto* s = static_cast<const uint8_t*>(src);

    if (!(d < s + size && s < d + size))
    {
      FastCopy(dst, src, size);
      return;
    }

    if (d > s)
    {
      // Backward-overlap move: consume from the tail first so we never overwrite
      // bytes that still need to be read from the source window.
      uintptr_t d_end_offset = reinterpret_cast<uintptr_t>(d + size) & (WORD_SIZE - 1U);
      uintptr_t s_end_offset = reinterpret_cast<uintptr_t>(s + size) & (WORD_SIZE - 1U);

      d += size;
      s += size;

      if (d_end_offset == s_end_offset)
      {
        // Once both ends share the same alignment, we can peel the unaligned tail
        // bytes and then switch to wide backward copies safely.
        if (d_end_offset != 0U)
        {
          size_t tail = d_end_offset;
          if (tail > size)
          {
            tail = size;
          }
          while (tail--)
          {
            *--d = *--s;
            --size;
          }
        }

        if constexpr (WORD_SIZE == 8U)
        {
          auto* dw = reinterpret_cast<uint64_t*>(d);
          auto* sw = reinterpret_cast<const uint64_t*>(s);

          while (size >= 64U)
          {
            uint64_t a0 = sw[-1];
            uint64_t a1 = sw[-2];
            uint64_t a2 = sw[-3];
            uint64_t a3 = sw[-4];
            uint64_t a4 = sw[-5];
            uint64_t a5 = sw[-6];
            uint64_t a6 = sw[-7];
            uint64_t a7 = sw[-8];
            dw[-1] = a0;
            dw[-2] = a1;
            dw[-3] = a2;
            dw[-4] = a3;
            dw[-5] = a4;
            dw[-6] = a5;
            dw[-7] = a6;
            dw[-8] = a7;
            dw -= 8;
            sw -= 8;
            size -= 64U;
          }
          while (size >= 8U)
          {
            uint64_t a = *--sw;
            *--dw = a;
            size -= 8U;
          }

          d = reinterpret_cast<uint8_t*>(dw);
          s = reinterpret_cast<const uint8_t*>(sw);
        }
        else
        {
          auto* dw = reinterpret_cast<uint32_t*>(d);
          auto* sw = reinterpret_cast<const uint32_t*>(s);

          while (size >= 32U)
          {
            uint32_t a0 = sw[-1];
            uint32_t a1 = sw[-2];
            uint32_t a2 = sw[-3];
            uint32_t a3 = sw[-4];
            uint32_t a4 = sw[-5];
            uint32_t a5 = sw[-6];
            uint32_t a6 = sw[-7];
            uint32_t a7 = sw[-8];
            dw[-1] = a0;
            dw[-2] = a1;
            dw[-3] = a2;
            dw[-4] = a3;
            dw[-5] = a4;
            dw[-6] = a5;
            dw[-7] = a6;
            dw[-8] = a7;
            dw -= 8;
            sw -= 8;
            size -= 32U;
          }
          while (size >= 4U)
          {
            uint32_t a = *--sw;
            *--dw = a;
            size -= 4U;
          }

          d = reinterpret_cast<uint8_t*>(dw);
          s = reinterpret_cast<const uint8_t*>(sw);
        }
      }

      while (size--)
      {
        *--d = *--s;
      }
      return;
    }

    // Forward-overlap move: valid only when destination starts before source, so
    // consuming from the head cannot destroy unread source bytes.
    uintptr_t d_offset = reinterpret_cast<uintptr_t>(d) & (WORD_SIZE - 1U);
    uintptr_t s_offset = reinterpret_cast<uintptr_t>(s) & (WORD_SIZE - 1U);

    if (d_offset == s_offset)
    {
      // Same-alignment forward overlap can reuse the same "align head, then burst"
      // strategy as FastCopy because source bytes are always read before they are
      // overwritten.
      if (d_offset != 0U)
      {
        size_t head = WORD_SIZE - d_offset;
        if (head > size)
        {
          head = size;
        }
        while (head--)
        {
          *d++ = *s++;
          --size;
        }
      }

      if constexpr (WORD_SIZE == 8U)
      {
        CopyAlignedWords<uint64_t>(d, s, size);
      }
      else
      {
        CopyAlignedWords<uint32_t>(d, s, size);
      }
    }

    while (size--)
    {
      *d++ = *s++;
    }
  }
}

void LibXR::Memory::FastSet(void* dst, uint8_t value, size_t size)
{
  if (size == 0U)
  {
    return;
  }

  if constexpr (ACTIVE_FASTSET_POLICY == BasicPolicy::STD_LIBC)
  {
    std::memset(dst, value, size);
  }
  else
  {
    uint8_t* d = static_cast<uint8_t*>(dst);
    uintptr_t d_offset = reinterpret_cast<uintptr_t>(d) & (WORD_SIZE - 1U);

    // 先处理头部到字对齐；head 严格小于 WORD_SIZE。
    // Peel unaligned head bytes (< WORD_SIZE) before entering the word burst loop.
    if (d_offset != 0U)
    {
      size_t head = WORD_SIZE - d_offset;
      if (head > size)
      {
        head = size;
      }
      FillSmallBytes<WORD_SIZE>(d, value, head);
      size -= head;
    }

    if constexpr (WORD_SIZE == 8U)
    {
      // 8-byte pattern
      uint64_t pat = value;
      pat |= pat << 8;
      pat |= pat << 16;
      pat |= pat << 32;

      auto* dw = reinterpret_cast<uint64_t*>(d);

      while (size >= 64U)
      {
        dw[0] = pat;
        dw[1] = pat;
        dw[2] = pat;
        dw[3] = pat;
        dw[4] = pat;
        dw[5] = pat;
        dw[6] = pat;
        dw[7] = pat;
        dw += 8;
        size -= 64U;
      }
      while (size >= 8U)
      {
        *dw++ = pat;
        size -= 8U;
      }

      d = reinterpret_cast<uint8_t*>(dw);
    }
    else
    {
      // 4-byte pattern
      uint32_t pat = value;
      pat |= pat << 8;
      pat |= pat << 16;

      auto* dw = reinterpret_cast<uint32_t*>(d);

      while (size >= 32U)
      {
        dw[0] = pat;
        dw[1] = pat;
        dw[2] = pat;
        dw[3] = pat;
        dw[4] = pat;
        dw[5] = pat;
        dw[6] = pat;
        dw[7] = pat;
        dw += 8;
        size -= 32U;
      }
      while (size >= 4U)
      {
        *dw++ = pat;
        size -= 4U;
      }

      d = reinterpret_cast<uint8_t*>(dw);
    }

    // 处理尾部剩余字节；经过字循环后 size 严格小于 WORD_SIZE。
    // Fill remaining tail bytes (< WORD_SIZE).
    FillSmallBytes<WORD_SIZE>(d, value, size);
  }
}
