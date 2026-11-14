/**
 * @file ch32_test_common.hpp
 * @brief CH32 USB 端点主机测试共用的部分 / Parts shared by the CH32 USB endpoint host
 *        tests.
 *
 * 每个测试程序用自己的假寄存器头文件编译一份驱动源文件，所以它们是独立的可执行文件，
 * 各自提供断言失败时的出口。
 * Each test program compiles a driver source with its own fake register header, so they
 * are separate executables, and each provides the exit taken when an assertion fails.
 */

#pragma once

#include <cstdint>
#include <cstdio>
#include <cstdlib>

/// 断言失败：打印位置并终止；失败的 REQUIRE 不带位置。
/// Assertion failure: print the place and abort; a failed REQUIRE carries no location.
extern "C" inline void libxr_fatal_error(const char* file, uint32_t line, bool)
{
  std::fprintf(stderr, "%s:%u: CH32 USB assertion failed\n",
               file != nullptr ? file : "(no location)", line);
  std::abort();
}

/// ASSERT 与 DEV_ASSERT 的入口，这里与 libxr_fatal_error 相同。
/// Entry of ASSERT and DEV_ASSERT, the same as libxr_fatal_error here.
extern "C" inline void libxr_fatal_error_and_print(const char* file, uint32_t line,
                                                   bool in_isr)
{
  libxr_fatal_error(file, line, in_isr);
}

namespace LibXR::USB::Test
{
/// 指针在 32 位 DMA 寄存器里的值 / The value a pointer has in a 32-bit DMA register.
inline uint32_t Address(const void* pointer)
{
  return static_cast<uint32_t>(reinterpret_cast<uintptr_t>(pointer));
}
}  // namespace LibXR::USB::Test
