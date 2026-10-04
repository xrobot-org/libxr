/**
 * @file main.cpp
 * @brief automatic 测试执行器 / Automatic test runner.
 *
 * 列出运行入口，支持按名称选择和进程隔离，并把测试失败传回 CTest。
 * List case entries, select by name, isolate cases and propagate failures to CTest.
 */

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "libxr.hpp"
#include "test.hpp"
#include "test_assert.hpp"

#if defined(LIBXR_SYSTEM_POSIX_HOST)
#include <sys/wait.h>
#include <unistd.h>
#endif

void test_async();
void test_assert();
void test_linux_database_raw();
void test_linux_database_sequential();
void test_def();
void test_color();
void test_crc();
void test_float_encoder();
void test_event();
void test_flag();
void test_inertia();
void test_list();
void test_lockfree_list();
void test_kinematic();
void test_mpmc_queue();
void test_object_pool();
void test_linux_stdio_print();
void test_message_packet();
void test_message_topic();
void test_message_topic_contract();
void test_queue();
void test_spsc_queue();
void test_spsc_prefix();
void test_serialized_service();
void test_rbt();
void test_ramfs();
void test_semaphore();
void test_mutex();
void test_stack();
void test_terminal_command();
void test_terminal_display();
void test_terminal();
void test_thread();
void test_timebase();
void test_timer_semantics();
void test_timer_scheduler();
void test_read_port();
void test_write_port();
void test_write_stream();
void test_message_runtime();
void test_database();
void test_logger();
void test_terminal_input();
void test_time();
void test_transform();
void test_double_buffer();
void test_type();
void test_string();
void test_cycle_value();
void test_pid();
void test_pipe();
void test_print();
void test_cb();
void test_memory();
void test_linux_shm_topic();
void test_linux_uart_rx_backpressure();
void test_linux_uart_tx_backpressure();
void test_linux_uart_block_timeout();
void test_linux_uart_callback_write();
void test_linux_uart_stream_config();
void test_linux_uart_reconnect();
void test_linux_uart_partial_disconnect();
void test_linux_uart_initial_open_recovery();

namespace LinuxSharedTopicBench
{
int RunStandardBenchmarksSmoke();
int RunLatencyBenchmarksSmoke();
int RunOverloadBenchmarksSmoke();
int RunModeBenchmarksSmoke();
}  // namespace LinuxSharedTopicBench

namespace
{
const char* test_name = nullptr;

struct TestCase
{
  const char* name;
  int (*function)();
  // 常驻线程或全局状态会影响后续用例时，设为 true，在 exec 后的新进程中运行。
  // Use a fresh process for cases whose permanent workers or global state affect later
  // cases.
  bool isolated;
  // 内部要 fork 的用例须同时隔离并跳过 PlatformInit，且自行确保 fork 时没有其他线程。
  // Cases that fork internally need isolation, no PlatformInit, and no other threads at
  // fork.
  bool platform_init = true;
};

template <void (*Fn)()>
int RunVoidEntry()
{
  Fn();
  return 0;
}

void RunTestCase(const TestCase& test_case, bool direct)
{
  test_name = test_case.name;

  if (direct || !test_case.isolated)
  {
    // 普通用例共用一次平台初始化；只负责启动隔离用例的进程不需要启动 STDIO 线程。
    // Initialize once for ordinary cases; a process only launching isolated cases needs
    // no STDIO workers.
    static bool initialized = false;
    if (test_case.platform_init && !initialized)
    {
      LibXR::PlatformInit();
      initialized = true;
    }
    TEST_ASSERT(test_case.function() == 0);
    return;
  }

#if defined(LIBXR_SYSTEM_POSIX_HOST)
  std::fflush(nullptr);
  pid_t child = fork();
  TEST_ASSERT(child >= 0);

  if (child == 0)
  {
    // 多线程 fork 后只执行 exec 或退出 / Only exec or exit after a threaded fork.
    execl("/proc/self/exe", "test", "--direct-case", test_case.name,
          static_cast<char*>(nullptr));
    _exit(127);
  }

  int status = 0;
  pid_t waited;
  do
  {
    waited = waitpid(child, &status, 0);
  } while (waited == -1 && errno == EINTR);
  TEST_ASSERT(waited == child);
  TEST_ASSERT(WIFEXITED(status));
  TEST_ASSERT(WEXITSTATUS(status) == 0);
#else
  TEST_ASSERT(false);
#endif
}

int RunBenchLinuxSharedTopicSet()
{
  int status = 0;
  status |= LinuxSharedTopicBench::RunStandardBenchmarksSmoke();
  status |= LinuxSharedTopicBench::RunLatencyBenchmarksSmoke();
  status |= LinuxSharedTopicBench::RunOverloadBenchmarksSmoke();
  status |= LinuxSharedTopicBench::RunModeBenchmarksSmoke();
  return status;
}

struct GroupedTestCase
{
  const char* group;
  TestCase test_case;
};

constexpr GroupedTestCase kMainTestCases[] = {
    {"timer_tests", {"timer_semantics", &RunVoidEntry<test_timer_semantics>, true}},
    {"timer_tests", {"timer_scheduler", &RunVoidEntry<test_timer_scheduler>, true}},
    {"linux_uart_tests",
     {"uart_rx_space", &RunVoidEntry<test_linux_uart_rx_backpressure>, true}},
    {"linux_uart_tests",
     {"uart_tx_partial", &RunVoidEntry<test_linux_uart_tx_backpressure>, true}},
    {"linux_uart_tests",
     {"uart_block_timeout", &RunVoidEntry<test_linux_uart_block_timeout>, true}},
    {"linux_uart_tests",
     {"uart_callback_write", &RunVoidEntry<test_linux_uart_callback_write>, true}},
    {"linux_uart_tests",
     {"uart_stream_config", &RunVoidEntry<test_linux_uart_stream_config>, true}},
    {"linux_uart_tests",
     {"uart_reconnect", &RunVoidEntry<test_linux_uart_reconnect>, true}},
    {"linux_uart_tests",
     {"uart_partial_disconnect", &RunVoidEntry<test_linux_uart_partial_disconnect>,
      true}},
    {"linux_uart_tests",
     {"uart_initial_recovery", &RunVoidEntry<test_linux_uart_initial_open_recovery>,
      true}},
    {"core_tests", {"assert", &RunVoidEntry<test_assert>, false}},
    {"core_tests", {"def", &RunVoidEntry<test_def>, false}},
    {"core_tests", {"callback", &RunVoidEntry<test_cb>, false}},
    {"core_tests", {"pipe", &RunVoidEntry<test_pipe>, false}},
    {"core_tests", {"read_port", &RunVoidEntry<test_read_port>, false}},
    {"core_tests", {"write_port", &RunVoidEntry<test_write_port>, false}},
    {"core_tests", {"write_stream", &RunVoidEntry<test_write_stream>, false}},
    {"core_tests", {"memory", &RunVoidEntry<test_memory>, false}},
    {"core_tests", {"color", &RunVoidEntry<test_color>, false}},
    {"core_tests", {"time", &RunVoidEntry<test_time>, false}},

    {"synchronization_tests", {"semaphore", &RunVoidEntry<test_semaphore>, false}},
    {"synchronization_tests", {"mutex", &RunVoidEntry<test_mutex>, false}},
    {"synchronization_tests", {"async", &RunVoidEntry<test_async>, false}},
    {"synchronization_tests",
     {"serialized_service", &RunVoidEntry<test_serialized_service>, false}},

    {"utility_tests", {"crc", &RunVoidEntry<test_crc>, false}},
    {"utility_tests", {"encoder", &RunVoidEntry<test_float_encoder>, false}},
    {"utility_tests", {"cycle_value", &RunVoidEntry<test_cycle_value>, false}},
    {"utility_tests", {"print", &RunVoidEntry<test_print>, false}},
    {"utility_tests", {"flag", &RunVoidEntry<test_flag>, false}},

    {"data_structure_tests", {"rbt", &RunVoidEntry<test_rbt>, false}},
    {"data_structure_tests", {"queue", &RunVoidEntry<test_queue>, false}},
    {"data_structure_tests", {"spsc_queue", &RunVoidEntry<test_spsc_queue>, false}},
    {"data_structure_tests", {"spsc_prefix", &RunVoidEntry<test_spsc_prefix>, false}},
    {"data_structure_tests", {"mpmc_queue", &RunVoidEntry<test_mpmc_queue>, false}},
    {"data_structure_tests", {"object_pool", &RunVoidEntry<test_object_pool>, false}},
    {"data_structure_tests", {"stack", &RunVoidEntry<test_stack>, false}},
    {"data_structure_tests", {"list", &RunVoidEntry<test_list>, false}},
    {"data_structure_tests", {"lockfree_list", &RunVoidEntry<test_lockfree_list>, false}},
    {"data_structure_tests", {"double_buffer", &RunVoidEntry<test_double_buffer>, false}},
    {"data_structure_tests", {"type", &RunVoidEntry<test_type>, false}},
    {"data_structure_tests", {"string", &RunVoidEntry<test_string>, false}},

    {"threading_tests", {"thread", &RunVoidEntry<test_thread>, false}},
    {"threading_tests", {"timebase", &RunVoidEntry<test_timebase>, true}},

    {"runtime_tests", {"message_runtime", &RunVoidEntry<test_message_runtime>, false}},

    {"motion_tests", {"inertia", &RunVoidEntry<test_inertia>, false}},
    {"motion_tests", {"kinematic", &RunVoidEntry<test_kinematic>, false}},
    {"motion_tests", {"transform", &RunVoidEntry<test_transform>, false}},

    {"control_tests", {"pid", &RunVoidEntry<test_pid>, false}},

    {"system_tests", {"ramfs", &RunVoidEntry<test_ramfs>, false}},
    {"system_tests", {"event", &RunVoidEntry<test_event>, false}},
    {"system_tests", {"message_topic", &RunVoidEntry<test_message_topic>, false}},
    {"system_tests",
     {"message_topic_contract", &RunVoidEntry<test_message_topic_contract>, true, false}},
    {"system_tests", {"message_packet", &RunVoidEntry<test_message_packet>, false}},
    {"system_tests", {"database", &RunVoidEntry<test_database>, false}},
    {"linux_host_tests",
     {"linux_stdio_print", &RunVoidEntry<test_linux_stdio_print>, false}},
    {"linux_host_tests",
     {"linux_database_sequential", &RunVoidEntry<test_linux_database_sequential>, true,
      false}},
    {"linux_host_tests",
     {"linux_database_raw", &RunVoidEntry<test_linux_database_raw>, true, false}},
    {"system_tests", {"logger", &RunVoidEntry<test_logger>, true}},
    {"system_tests",
     {"linux_shm_topic", &RunVoidEntry<test_linux_shm_topic>, true, false}},
    {"system_tests", {"linux_shm_bench", &RunBenchLinuxSharedTopicSet, true, false}},
    {"system_tests", {"terminal_command", &RunVoidEntry<test_terminal_command>, true}},
    {"system_tests", {"terminal_display", &RunVoidEntry<test_terminal_display>, false}},
    {"system_tests", {"terminal_input", &RunVoidEntry<test_terminal_input>, true}},
    {"system_tests", {"terminal", &RunVoidEntry<test_terminal>, true}},
};
}  // namespace

int main(int argc, char** argv)
{
  const bool direct = argc == 3 && std::strcmp(argv[1], "--direct-case") == 0;
  if (argc != 1 && (argc != 3 || (!direct && std::strcmp(argv[1], "--case") != 0)))
  {
    std::fprintf(stderr, "Usage: %s [--case NAME]\n", argv[0]);
    return 2;
  }
  const char* selected = argc == 3 ? argv[2] : nullptr;
  unsigned executed = 0;

  auto err_cb = LibXR::Assert::FatalCallback::Create(
      [](bool in_isr, void* arg, const char* file, uint32_t line)
      {
        UNUSED(in_isr);
        UNUSED(arg);
        std::fprintf(stderr, "%s:%u: product assertion failed in [%s].\n", file,
                     static_cast<unsigned>(line), test_name ? test_name : "startup");
        std::fflush(stderr);
        std::abort();
      },
      reinterpret_cast<void*>(0));

  LibXR::Assert::RegisterFatalErrorCallback(err_cb);

  if (!direct)
  {
    std::fprintf(stderr, "Running LibXR Tests...\n");
    std::fflush(stderr);
  }

  const char* current_group = nullptr;
  for (const auto& entry : kMainTestCases)
  {
    if (selected && std::strcmp(selected, entry.test_case.name) != 0)
    {
      continue;
    }
    if (!direct &&
        (current_group == nullptr || std::strcmp(current_group, entry.group) != 0))
    {
      current_group = entry.group;
      std::fprintf(stderr, "Test Group [%s]\n", current_group);
      std::fflush(stderr);
    }

    RunTestCase(entry.test_case, direct);
    if (direct)
    {
      std::exit(0);
    }
    ++executed;
    std::fprintf(stderr, "\tTest [%s] Passed.\n", entry.test_case.name);
    std::fflush(stderr);
  }

  if (executed == 0)
  {
    std::fprintf(stderr, "Unknown test case: %s\n", selected);
    return 2;
  }
  std::fprintf(stderr, "All %u tests completed.\n", executed);
  std::fflush(stderr);
  std::exit(0);
}
