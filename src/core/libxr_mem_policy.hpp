#pragma once

#include <cstddef>
#include <cstdint>

#include "libxr_def.hpp"

/// \cond
#define LIBXR_FASTCOPY_POLICY_SHIFTED_ODD 0
#define LIBXR_FASTCOPY_POLICY_SHIFTED_ALL 1
#define LIBXR_FASTCOPY_POLICY_STD_LIBC 2
#define LIBXR_FASTCOPY_POLICY_UNALIGNED_WORD 3

#ifndef LIBXR_FASTCOPY_POLICY
#define LIBXR_FASTCOPY_POLICY LIBXR_FASTCOPY_POLICY_SHIFTED_ODD
#endif

#define LIBXR_MEM_POLICY_BUILTIN 0
#define LIBXR_MEM_POLICY_STD_LIBC 1

#ifndef LIBXR_FASTMOVE_POLICY
#define LIBXR_FASTMOVE_POLICY LIBXR_MEM_POLICY_BUILTIN
#endif

#ifndef LIBXR_FASTSET_POLICY
#define LIBXR_FASTSET_POLICY LIBXR_MEM_POLICY_BUILTIN
#endif

#ifndef LIBXR_FASTCMP_POLICY
#define LIBXR_FASTCMP_POLICY LIBXR_MEM_POLICY_BUILTIN
#endif

#ifndef LIBXR_MEM_WORD_SIZE
#define LIBXR_MEM_WORD_SIZE LibXR::ALIGN_SIZE
#endif

namespace LibXR::MemoryDetail
{
enum class FastCopyPolicy : uint8_t
{
  SHIFTED_ODD = LIBXR_FASTCOPY_POLICY_SHIFTED_ODD,
  SHIFTED_ALL = LIBXR_FASTCOPY_POLICY_SHIFTED_ALL,
  STD_LIBC = LIBXR_FASTCOPY_POLICY_STD_LIBC,
  UNALIGNED_WORD = LIBXR_FASTCOPY_POLICY_UNALIGNED_WORD,
};

enum class BasicPolicy : uint8_t
{
  BUILTIN = LIBXR_MEM_POLICY_BUILTIN,
  STD_LIBC = LIBXR_MEM_POLICY_STD_LIBC,
};

inline constexpr size_t WORD_SIZE = (LIBXR_MEM_WORD_SIZE);
static_assert(WORD_SIZE == 4U || WORD_SIZE == 8U, "Unsupported LIBXR_MEM_WORD_SIZE");

inline constexpr int ACTIVE_FASTCOPY_POLICY_RAW = (LIBXR_FASTCOPY_POLICY);
static_assert(ACTIVE_FASTCOPY_POLICY_RAW == LIBXR_FASTCOPY_POLICY_SHIFTED_ODD ||
                  ACTIVE_FASTCOPY_POLICY_RAW == LIBXR_FASTCOPY_POLICY_SHIFTED_ALL ||
                  ACTIVE_FASTCOPY_POLICY_RAW == LIBXR_FASTCOPY_POLICY_STD_LIBC ||
                  ACTIVE_FASTCOPY_POLICY_RAW == LIBXR_FASTCOPY_POLICY_UNALIGNED_WORD,
              "Unsupported LIBXR_FASTCOPY_POLICY");
[[maybe_unused]] inline constexpr FastCopyPolicy ACTIVE_FASTCOPY_POLICY =
    static_cast<FastCopyPolicy>(ACTIVE_FASTCOPY_POLICY_RAW);

inline constexpr int ACTIVE_FASTMOVE_POLICY_RAW = (LIBXR_FASTMOVE_POLICY);
static_assert(ACTIVE_FASTMOVE_POLICY_RAW == LIBXR_MEM_POLICY_BUILTIN ||
                  ACTIVE_FASTMOVE_POLICY_RAW == LIBXR_MEM_POLICY_STD_LIBC,
              "Unsupported LIBXR_FASTMOVE_POLICY");
[[maybe_unused]] inline constexpr BasicPolicy ACTIVE_FASTMOVE_POLICY =
    static_cast<BasicPolicy>(ACTIVE_FASTMOVE_POLICY_RAW);

inline constexpr int ACTIVE_FASTSET_POLICY_RAW = (LIBXR_FASTSET_POLICY);
static_assert(ACTIVE_FASTSET_POLICY_RAW == LIBXR_MEM_POLICY_BUILTIN ||
                  ACTIVE_FASTSET_POLICY_RAW == LIBXR_MEM_POLICY_STD_LIBC,
              "Unsupported LIBXR_FASTSET_POLICY");
[[maybe_unused]] inline constexpr BasicPolicy ACTIVE_FASTSET_POLICY =
    static_cast<BasicPolicy>(ACTIVE_FASTSET_POLICY_RAW);

inline constexpr int ACTIVE_FASTCMP_POLICY_RAW = (LIBXR_FASTCMP_POLICY);
static_assert(ACTIVE_FASTCMP_POLICY_RAW == LIBXR_MEM_POLICY_BUILTIN ||
                  ACTIVE_FASTCMP_POLICY_RAW == LIBXR_MEM_POLICY_STD_LIBC,
              "Unsupported LIBXR_FASTCMP_POLICY");
[[maybe_unused]] inline constexpr BasicPolicy ACTIVE_FASTCMP_POLICY =
    static_cast<BasicPolicy>(ACTIVE_FASTCMP_POLICY_RAW);

#if defined(__ARM_FEATURE_UNALIGNED) || defined(__x86_64__) || defined(__i386__) || \
    defined(_M_X64) || defined(_M_IX86) || defined(__aarch64__) || defined(_M_ARM64)
[[maybe_unused]] inline constexpr bool HAS_FAST_UNALIGNED_WORD = true;
#else
[[maybe_unused]] inline constexpr bool HAS_FAST_UNALIGNED_WORD = false;
#endif
}  // namespace LibXR::MemoryDetail
/// \endcond
