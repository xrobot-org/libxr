#include <cstring>

#include "libxr_def.hpp"
#include "libxr_mem.hpp"
#include "libxr_mem_policy.hpp"

int LibXR::Memory::FastCmp(const void* a, const void* b, size_t size)
{
  if ((size == 0U) || (a == b))
  {
    return 0;
  }

  if constexpr (LibXR::MemoryDetail::ACTIVE_FASTCMP_POLICY ==
                LibXR::MemoryDetail::BasicPolicy::STD_LIBC)
  {
    return std::memcmp(a, b, size);
  }
  else
  {
    constexpr size_t WORD_SIZE = LibXR::MemoryDetail::WORD_SIZE;
    const uint8_t* p = static_cast<const uint8_t*>(a);
    const uint8_t* q = static_cast<const uint8_t*>(b);
    const uint8_t* mismatch_p = nullptr;
    const uint8_t* mismatch_q = nullptr;
    size_t mismatch_size = 0;

    uintptr_t p_off = reinterpret_cast<uintptr_t>(p) & (WORD_SIZE - 1U);
    uintptr_t q_off = reinterpret_cast<uintptr_t>(q) & (WORD_SIZE - 1U);

    // 若同相位：先补齐到 WORD_SIZE 对齐再做宽比较
    if ((p_off == q_off) && (p_off != 0U))
    {
      size_t head = WORD_SIZE - p_off;
      if (head > size)
      {
        head = size;
      }

      while (head--)
      {
        int diff = static_cast<int>(*p++) - static_cast<int>(*q++);
        if (diff != 0)
        {
          return diff;
        }
        --size;
      }
    }

    if constexpr (WORD_SIZE == 8U)
    {
      // 8-byte compare（仅在两者均 8 对齐时才安全/快）
      if ((((reinterpret_cast<uintptr_t>(p) | reinterpret_cast<uintptr_t>(q)) & 7u) ==
           0u))
      {
        auto* pw = reinterpret_cast<const uint64_t*>(p);
        auto* qw = reinterpret_cast<const uint64_t*>(q);

        while (size >= 64)
        {
          if (pw[0] != qw[0])
          {
            mismatch_p = reinterpret_cast<const uint8_t*>(pw + 0);
            mismatch_q = reinterpret_cast<const uint8_t*>(qw + 0);
            mismatch_size = 8;
            goto compare_fixed_bytes;
          }
          if (pw[1] != qw[1])
          {
            mismatch_p = reinterpret_cast<const uint8_t*>(pw + 1);
            mismatch_q = reinterpret_cast<const uint8_t*>(qw + 1);
            mismatch_size = 8;
            goto compare_fixed_bytes;
          }
          if (pw[2] != qw[2])
          {
            mismatch_p = reinterpret_cast<const uint8_t*>(pw + 2);
            mismatch_q = reinterpret_cast<const uint8_t*>(qw + 2);
            mismatch_size = 8;
            goto compare_fixed_bytes;
          }
          if (pw[3] != qw[3])
          {
            mismatch_p = reinterpret_cast<const uint8_t*>(pw + 3);
            mismatch_q = reinterpret_cast<const uint8_t*>(qw + 3);
            mismatch_size = 8;
            goto compare_fixed_bytes;
          }
          if (pw[4] != qw[4])
          {
            mismatch_p = reinterpret_cast<const uint8_t*>(pw + 4);
            mismatch_q = reinterpret_cast<const uint8_t*>(qw + 4);
            mismatch_size = 8;
            goto compare_fixed_bytes;
          }
          if (pw[5] != qw[5])
          {
            mismatch_p = reinterpret_cast<const uint8_t*>(pw + 5);
            mismatch_q = reinterpret_cast<const uint8_t*>(qw + 5);
            mismatch_size = 8;
            goto compare_fixed_bytes;
          }
          if (pw[6] != qw[6])
          {
            mismatch_p = reinterpret_cast<const uint8_t*>(pw + 6);
            mismatch_q = reinterpret_cast<const uint8_t*>(qw + 6);
            mismatch_size = 8;
            goto compare_fixed_bytes;
          }
          if (pw[7] != qw[7])
          {
            mismatch_p = reinterpret_cast<const uint8_t*>(pw + 7);
            mismatch_q = reinterpret_cast<const uint8_t*>(qw + 7);
            mismatch_size = 8;
            goto compare_fixed_bytes;
          }

          pw += 8;
          qw += 8;
          size -= 64;
        }

        while (size >= 8)
        {
          if (*pw != *qw)
          {
            mismatch_p = reinterpret_cast<const uint8_t*>(pw);
            mismatch_q = reinterpret_cast<const uint8_t*>(qw);
            mismatch_size = 8;
            goto compare_fixed_bytes;
          }
          ++pw;
          ++qw;
          size -= 8;
        }

        p = reinterpret_cast<const uint8_t*>(pw);
        q = reinterpret_cast<const uint8_t*>(qw);
      }
    }
    else
    {
      // 4-byte compare（两者均 4 对齐）
      if ((((reinterpret_cast<uintptr_t>(p) | reinterpret_cast<uintptr_t>(q)) & 3u) ==
           0u))
      {
        auto* pw = reinterpret_cast<const uint32_t*>(p);
        auto* qw = reinterpret_cast<const uint32_t*>(q);

        while (size >= 32)
        {
          if (pw[0] != qw[0])
          {
            mismatch_p = reinterpret_cast<const uint8_t*>(pw + 0);
            mismatch_q = reinterpret_cast<const uint8_t*>(qw + 0);
            mismatch_size = 4;
            goto compare_fixed_bytes;
          }
          if (pw[1] != qw[1])
          {
            mismatch_p = reinterpret_cast<const uint8_t*>(pw + 1);
            mismatch_q = reinterpret_cast<const uint8_t*>(qw + 1);
            mismatch_size = 4;
            goto compare_fixed_bytes;
          }
          if (pw[2] != qw[2])
          {
            mismatch_p = reinterpret_cast<const uint8_t*>(pw + 2);
            mismatch_q = reinterpret_cast<const uint8_t*>(qw + 2);
            mismatch_size = 4;
            goto compare_fixed_bytes;
          }
          if (pw[3] != qw[3])
          {
            mismatch_p = reinterpret_cast<const uint8_t*>(pw + 3);
            mismatch_q = reinterpret_cast<const uint8_t*>(qw + 3);
            mismatch_size = 4;
            goto compare_fixed_bytes;
          }
          if (pw[4] != qw[4])
          {
            mismatch_p = reinterpret_cast<const uint8_t*>(pw + 4);
            mismatch_q = reinterpret_cast<const uint8_t*>(qw + 4);
            mismatch_size = 4;
            goto compare_fixed_bytes;
          }
          if (pw[5] != qw[5])
          {
            mismatch_p = reinterpret_cast<const uint8_t*>(pw + 5);
            mismatch_q = reinterpret_cast<const uint8_t*>(qw + 5);
            mismatch_size = 4;
            goto compare_fixed_bytes;
          }
          if (pw[6] != qw[6])
          {
            mismatch_p = reinterpret_cast<const uint8_t*>(pw + 6);
            mismatch_q = reinterpret_cast<const uint8_t*>(qw + 6);
            mismatch_size = 4;
            goto compare_fixed_bytes;
          }
          if (pw[7] != qw[7])
          {
            mismatch_p = reinterpret_cast<const uint8_t*>(pw + 7);
            mismatch_q = reinterpret_cast<const uint8_t*>(qw + 7);
            mismatch_size = 4;
            goto compare_fixed_bytes;
          }

          pw += 8;
          qw += 8;
          size -= 32;
        }

        while (size >= 4)
        {
          if (*pw != *qw)
          {
            mismatch_p = reinterpret_cast<const uint8_t*>(pw);
            mismatch_q = reinterpret_cast<const uint8_t*>(qw);
            mismatch_size = 4;
            goto compare_fixed_bytes;
          }
          ++pw;
          ++qw;
          size -= 4;
        }

        p = reinterpret_cast<const uint8_t*>(pw);
        q = reinterpret_cast<const uint8_t*>(qw);
      }
    }

  compare_fixed_bytes:
    if (mismatch_size == 8)
    {
      int diff = static_cast<int>(mismatch_p[0]) - static_cast<int>(mismatch_q[0]);
      if (diff != 0)
      {
        return diff;
      }
      diff = static_cast<int>(mismatch_p[1]) - static_cast<int>(mismatch_q[1]);
      if (diff != 0)
      {
        return diff;
      }
      diff = static_cast<int>(mismatch_p[2]) - static_cast<int>(mismatch_q[2]);
      if (diff != 0)
      {
        return diff;
      }
      diff = static_cast<int>(mismatch_p[3]) - static_cast<int>(mismatch_q[3]);
      if (diff != 0)
      {
        return diff;
      }
      diff = static_cast<int>(mismatch_p[4]) - static_cast<int>(mismatch_q[4]);
      if (diff != 0)
      {
        return diff;
      }
      diff = static_cast<int>(mismatch_p[5]) - static_cast<int>(mismatch_q[5]);
      if (diff != 0)
      {
        return diff;
      }
      diff = static_cast<int>(mismatch_p[6]) - static_cast<int>(mismatch_q[6]);
      if (diff != 0)
      {
        return diff;
      }
      return static_cast<int>(mismatch_p[7]) - static_cast<int>(mismatch_q[7]);
    }

    if (mismatch_size == 4)
    {
      int diff = static_cast<int>(mismatch_p[0]) - static_cast<int>(mismatch_q[0]);
      if (diff != 0)
      {
        return diff;
      }
      diff = static_cast<int>(mismatch_p[1]) - static_cast<int>(mismatch_q[1]);
      if (diff != 0)
      {
        return diff;
      }
      diff = static_cast<int>(mismatch_p[2]) - static_cast<int>(mismatch_q[2]);
      if (diff != 0)
      {
        return diff;
      }
      return static_cast<int>(mismatch_p[3]) - static_cast<int>(mismatch_q[3]);
    }

    // tail byte compare（包括：未满足宽比较对齐条件的情况）
    while (size--)
    {
      int diff = static_cast<int>(*p++) - static_cast<int>(*q++);
      if (diff != 0)
      {
        return diff;
      }
    }

    return 0;
  }
}
