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

/// 断言失败：打印位置并终止 / Assertion failure: print the place and abort.
extern "C" inline void libxr_fatal_error(const char* file, uint32_t line, bool)
{
  std::fprintf(stderr, "%s:%u: CH32 USB assertion failed\n", file, line);
  std::abort();
}

namespace LibXR::USB::Test
{
/// 指针在 32 位 DMA 寄存器里的值 / The value a pointer has in a 32-bit DMA register.
inline uint32_t Address(const void* pointer)
{
  return static_cast<uint32_t>(reinterpret_cast<uintptr_t>(pointer));
}
}  // namespace LibXR::USB::Test
