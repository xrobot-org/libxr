/**
 * @file test_assert.cpp
 * @brief 检查致命错误回调的注册、调用次数、位置和 ISR 标记，以及库的两个致命入口和
 * REQUIRE 交给回调的位置。 /
 * Tests fatal callback registration, call count, source location and ISR flag, and the
 * location that the two fatal entries of the library and REQUIRE hand to the callback.
 */

#include "test_assert.hpp"

#include <sys/wait.h>
#include <unistd.h>

#include <cstdint>
#include <cstring>
#include <string_view>

#include "libxr.hpp"
#include "libxr_def.hpp"
#include "test.hpp"

namespace
{

struct FatalProbe
{
  int hit_count = 0;
  bool in_isr = true;
  std::string_view file;
  uint32_t line = 0;
};

// 子进程用退出码报告回调收到的位置。
// The child reports the location the callback got through its exit code.
constexpr int LOCATION_EXIT = 42;     // 调用者给出的位置 / The location the caller gave.
constexpr int NO_LOCATION_EXIT = 43;  // 位置为空 / No location.
constexpr int UNEXPECTED_EXIT = 44;
constexpr const char* DEMO_FILE = "demo.cpp";
constexpr uint32_t DEMO_LINE = 7;

void FailingRequire() { REQUIRE(false); }
void FailingFatalError() { libxr_fatal_error(DEMO_FILE, DEMO_LINE, false); }
void FailingFatalErrorAndPrint()
{
  libxr_fatal_error_and_print(DEMO_FILE, DEMO_LINE, false);
}

// 在子进程里调用 fail，经由库真正的致命入口；回调用退出码报告它收到的位置。
// Call fail in a child, through the real fatal entries of the library; the callback
// reports the location it got through the exit code.
int FatalLocationSeenByCallback(void (*fail)())
{
  std::fflush(nullptr);
  const pid_t child = fork();
  TEST_ASSERT(child >= 0);
  if (child == 0)
  {
    auto callback = LibXR::Assert::FatalCallback::Create(
        [](bool in_isr, int, const char* file, uint32_t line)
        {
          if (in_isr)
          {
            _exit(UNEXPECTED_EXIT);
          }
          if (file == nullptr && line == 0U)
          {
            _exit(NO_LOCATION_EXIT);
          }
          const bool demo =
              file != nullptr && std::strcmp(file, DEMO_FILE) == 0 && line == DEMO_LINE;
          _exit(demo ? LOCATION_EXIT : UNEXPECTED_EXIT);
        },
        0);
    LibXR::Assert::RegisterFatalErrorCallback(callback);
    fail();
    _exit(0);  // 致命入口返回了 / The fatal entry returned.
  }
  int status = 0;
  TEST_ASSERT(waitpid(child, &status, 0) == child);
  TEST_ASSERT(WIFEXITED(status));
  return WEXITSTATUS(status);
}

}  // namespace

void test_assert()
{
  // 临时替换 fatal 回调，测试后恢复，避免后续测试继续引用栈上的 probe。
  // Temporarily replace the fatal callback and restore it before the stack probe expires.
  auto old_callback = LibXR::Assert::FatalErrorCallback();

  FatalProbe probe;
  auto callback = LibXR::Assert::FatalCallback::Create(
      [](bool in_isr, FatalProbe* probe, const char* file, uint32_t line)
      {
        probe->hit_count++;
        probe->in_isr = in_isr;
        probe->file = file;
        probe->line = line;
      },
      &probe);

  LibXR::Assert::RegisterFatalErrorCallback(callback);
  TEST_ASSERT(!LibXR::Assert::FatalErrorCallback().Empty());

  // 通过公开入口调用，检查只分发一次，并原样传递位置和 ISR 标记。
  // Use the public entry; check one dispatch with the supplied location and ISR flag.
  LibXR::Assert::RunFatalErrorCallback(false, "test_assert.cpp", 42);

  TEST_ASSERT(probe.hit_count == 1);
  TEST_ASSERT(!probe.in_isr);
  TEST_ASSERT(probe.file == "test_assert.cpp");
  TEST_ASSERT(probe.line == 42);

  LibXR::Assert::RegisterFatalErrorCallback(old_callback);

  // 失败的 REQUIRE 不带位置；两个致命入口都把调用者给的位置原样交给回调，并且不返回。
  // A failed REQUIRE carries no location; both fatal entries hand the location their
  // caller gives to the callback unchanged, and neither returns.
  TEST_ASSERT(FatalLocationSeenByCallback(&FailingRequire) == NO_LOCATION_EXIT);
  TEST_ASSERT(FatalLocationSeenByCallback(&FailingFatalError) == LOCATION_EXIT);
  TEST_ASSERT(FatalLocationSeenByCallback(&FailingFatalErrorAndPrint) == LOCATION_EXIT);
}
