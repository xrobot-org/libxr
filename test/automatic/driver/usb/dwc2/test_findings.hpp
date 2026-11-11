/**
 * @file test_findings.hpp
 * @brief 已知缺陷用例的运行入口 / Runner for the known-defect cases.
 *
 * 有些用例检查的行为后端目前做不到（见 README.md 的"发现"一节）。它们保持完整的断言，
 * 不放进默认的 ctest：测试程序不带参数时只跑常规用例；`--list-findings` 列出这些用例，
 * `--finding <名字>` 单独跑一个。断言失败时进程终止并输出位置，后端修好后同一条命令
 * 返回 0，此时应把用例移进常规用例。
 * Some cases check behaviour the backend does not have yet (see the findings section of
 * README.md). They keep their full assertions and are left out of the default ctest:
 * without arguments a test program runs the regular cases only; `--list-findings` lists
 * these cases and `--finding <name>` runs one. A failed assertion aborts with its
 * location; once the backend is fixed the same command returns 0, and the case then
 * belongs among the regular ones.
 *
 * `--check-findings` runs every listed case in a child process and requires that it still
 * aborts at a failed assertion whose text contains the `fails_with` of the case. A
 * finding that starts to pass, or that fails for another reason (a crash, another
 * assertion), makes the command return 1, so the change shows up instead of going
 * unnoticed. The ctests `stm32_usb_otg_findings` and `esp_usb_findings` run it; they
 * carry the label `findings`.
 * `--check-findings`
 * 在子进程里逐个运行列出的用例，要求每个仍然在断言失败处中止，且断言文字 含有该用例的
 * `fails_with`。某个发现开始通过，或因别的原因失败（崩溃、别的断言），命令 返回
 * 1，变化就不会被忽略。ctest `stm32_usb_otg_findings` 和 `esp_usb_findings` 运行它，
 * 带标签 `findings`。
 */

#pragma once

#include <fcntl.h>
#include <signal.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cstdio>
#include <cstring>
#include <initializer_list>
#include <string>

extern char** environ;

namespace TestFindings
{

struct Case
{
  const char* name;  ///< 用例名 / Case name
  const char* item;  ///< 缺陷 / The defect
  void (*run)();
  /// 现在失败时断言文字里应含有的片段（`--check-findings` 用）/ Text the failing
  /// assertion contains today (used by `--check-findings`)
  const char* fails_with;
};

/// 在子进程里运行一个发现，返回它的输出和等待状态 / Run one finding in a child process,
/// return its output and wait status
inline bool RunInChild(const char* self, const char* name, std::string& output,
                       int& status)
{
  int fds[2];
  if (pipe(fds) != 0) return false;
  posix_spawn_file_actions_t actions;
  posix_spawn_file_actions_init(&actions);
  posix_spawn_file_actions_adddup2(&actions, fds[1], 1);
  posix_spawn_file_actions_adddup2(&actions, fds[1], 2);
  posix_spawn_file_actions_addclose(&actions, fds[0]);
  posix_spawn_file_actions_addclose(&actions, fds[1]);
  char* const arguments[] = {const_cast<char*>(self), const_cast<char*>("--finding"),
                             const_cast<char*>(name), nullptr};
  pid_t pid = 0;
  const int spawned = posix_spawn(&pid, self, &actions, nullptr, arguments, environ);
  posix_spawn_file_actions_destroy(&actions);
  close(fds[1]);
  if (spawned != 0)
  {
    close(fds[0]);
    return false;
  }
  char buffer[4096];
  ssize_t count;
  while ((count = read(fds[0], buffer, sizeof(buffer))) > 0)
    output.append(buffer, static_cast<size_t>(count));
  close(fds[0]);
  return waitpid(pid, &status, 0) == pid;
}

/// 每个发现都必须仍然失败 / Every finding must still fail
inline int CheckAll(const char* self, std::initializer_list<Case> findings)
{
  int unexpected = 0;
  for (const auto& item : findings)
  {
    std::string output;
    int status = 0;
    if (!RunInChild(self, item.name, output, status))
    {
      std::printf("finding %s: could not run it\n", item.name);
      ++unexpected;
      continue;
    }
    if (WIFEXITED(status) && WEXITSTATUS(status) == 0)
    {
      std::printf("finding %s (%s): NOW PASSES, move the case among the regular ones\n",
                  item.name, item.item);
      ++unexpected;
    }
    else if (!WIFSIGNALED(status) || WTERMSIG(status) != SIGABRT ||
             output.find("test failed:") == std::string::npos ||
             output.find(item.fails_with) == std::string::npos)
    {
      std::printf(
          "finding %s (%s): FAILS FOR ANOTHER REASON (expected an assertion with "
          "\"%s\")\n%s",
          item.name, item.item, item.fails_with, output.c_str());
      ++unexpected;
    }
    else
    {
      std::printf("finding %s: still fails as documented\n", item.name);
    }
  }
  std::fflush(stdout);
  return unexpected == 0 ? 0 : 1;
}

/// 处理 `--list-findings` 和 `--finding <名字>`；不是这两种参数时返回 -1 / Handles
/// `--list-findings` and `--finding <name>`; returns -1 for any other arguments
inline int Run(int argc, char** argv, std::initializer_list<Case> findings)
{
  if (argc >= 2 && std::strcmp(argv[1], "--list-findings") == 0)
  {
    for (const auto& item : findings) std::printf("%s\t%s\n", item.name, item.item);
    return 0;
  }
  if (argc >= 2 && std::strcmp(argv[1], "--check-findings") == 0)
    return CheckAll(argv[0], findings);
  if (argc >= 3 && std::strcmp(argv[1], "--finding") == 0)
  {
    for (const auto& item : findings)
    {
      if (std::strcmp(item.name, argv[2]) == 0)
      {
        item.run();
        std::printf("finding %s (%s): passes\n", item.name, item.item);
        return 0;
      }
    }
    std::fprintf(stderr, "unknown finding: %s\n", argv[2]);
    return 2;
  }
  return -1;
}

}  // namespace TestFindings
