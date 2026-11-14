/**
 * @file assert_modes.cpp
 * @brief 检查四种断言开关组合下的求值、失败分发和测试断言。 /
 * Tests evaluation, failure dispatch and TEST_ASSERT in four assertion modes.
 *
 * ASSERT 和 DEV_ASSERT 带文件名和行号进入 libxr_fatal_error_and_print；REQUIRE 进入
 * libxr_fatal_error，不带位置。记录入口、位置和 ISR 标记；子进程确认 TEST_ASSERT 失败会
 * 触发 SIGABRT。
 * ASSERT and DEV_ASSERT enter libxr_fatal_error_and_print with a file name and a line
 * number; REQUIRE enters libxr_fatal_error without a location. Checks the entry, the
 * location and the ISR flag; a child verifies that TEST_ASSERT raises SIGABRT.
 */

#include <sys/wait.h>
#include <unistd.h>

#include <csignal>
#include <cstring>

#include "libxr_def.hpp"
#include "test_assert.hpp"

namespace
{
int fatal_count = 0;
bool fatal_in_isr = false;
bool fatal_print = false;  // 进入的是带打印的入口 / The printing entry was entered.
const char* fatal_file = nullptr;
uint32_t fatal_line = 0;
}  // namespace

// 仅记录宏的分发，不调用平台致命错误处理 / Record dispatch without platform shutdown.
extern "C" void libxr_fatal_error(const char* file, uint32_t line, bool in_isr)
{
  ++fatal_count;
  fatal_print = false;
  fatal_file = file;
  fatal_line = line;
  fatal_in_isr = in_isr;
}

extern "C" void libxr_fatal_error_and_print(const char* file, uint32_t line, bool in_isr)
{
  ++fatal_count;
  fatal_print = true;
  fatal_file = file;
  fatal_line = line;
  fatal_in_isr = in_isr;
}

int main()
{
  int evaluations = 0;
  ASSERT(++evaluations == 1);
  TEST_ASSERT(evaluations == 1);
  const uint32_t assert_line = __LINE__ + 1;
  ASSERT(false);
#ifdef LIBXR_DEBUG_BUILD
  TEST_ASSERT(fatal_count == 1 && fatal_print && !fatal_in_isr);
  TEST_ASSERT(fatal_line == assert_line && std::strcmp(fatal_file, __FILE__) == 0);
#else
  UNUSED(assert_line);
  TEST_ASSERT(fatal_count == 0);
#endif

  fatal_count = 0;
  evaluations = 0;
  DEV_ASSERT(++evaluations == 1);
  const uint32_t dev_assert_line = __LINE__ + 1;
  DEV_ASSERT(false);
#ifdef LIBXR_DEV_ASSERT_BUILD
  TEST_ASSERT(evaluations == 1 && fatal_count == 1 && fatal_print && !fatal_in_isr);
  TEST_ASSERT(fatal_line == dev_assert_line && std::strcmp(fatal_file, __FILE__) == 0);
#else
  UNUSED(dev_assert_line);
  TEST_ASSERT(evaluations == 0 && fatal_count == 0);
  DEV_ASSERT(no_such_identifier);
  DEV_ASSERT_FROM_CALLBACK(no_such_condition, no_such_context);
#endif

  // REQUIRE 在每种开关组合下都生效，不带位置，进入不打印的入口。
  // REQUIRE is active in every mode, carries no location and enters the entry that does
  // not print.
  fatal_count = 0;
  evaluations = 0;
  REQUIRE(++evaluations == 1);
  REQUIRE_FROM_CALLBACK(false, true);
  TEST_ASSERT(evaluations == 1 && fatal_count == 1);
  TEST_ASSERT(fatal_in_isr && !fatal_print && fatal_file == nullptr && fatal_line == 0U);
  REQUIRE(false);
  TEST_ASSERT(fatal_count == 2);
  TEST_ASSERT(!fatal_in_isr && !fatal_print && fatal_file == nullptr && fatal_line == 0U);

  fatal_count = 0;
  ASSERT_FROM_CALLBACK(false, true);
#ifdef LIBXR_DEBUG_BUILD
  TEST_ASSERT(fatal_count == 1 && fatal_in_isr && fatal_print);
#else
  TEST_ASSERT(fatal_count == 0);
#endif
  fatal_count = 0;
  DEV_ASSERT_FROM_CALLBACK(false, true);
#ifdef LIBXR_DEV_ASSERT_BUILD
  TEST_ASSERT(fatal_count == 1 && fatal_in_isr && fatal_print);
#else
  TEST_ASSERT(fatal_count == 0);
#endif

  // 测试检查自身在所有开关组合下都必须失败 / Test checks must fail in every mode.
  const pid_t child = fork();
  TEST_ASSERT(child >= 0);
  if (child == 0)
  {
    TEST_ASSERT(false);
    _exit(0);
  }
  int status = 0;
  TEST_ASSERT(waitpid(child, &status, 0) == child);
  TEST_ASSERT(WIFSIGNALED(status) && WTERMSIG(status) == SIGABRT);
  return 0;
}
