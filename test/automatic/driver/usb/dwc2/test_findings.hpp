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
 */

#pragma once

#include <cstdio>
#include <cstring>
#include <initializer_list>

namespace TestFindings
{

struct Case
{
  const char* name;  ///< 用例名 / Case name
  const char* item;  ///< 缺陷 / The defect
  void (*run)();
};

/// 处理 `--list-findings` 和 `--finding <名字>`；不是这两种参数时返回 -1 / Handles
/// `--list-findings` and `--finding <name>`; returns -1 for any other arguments
inline int Run(int argc, char** argv, std::initializer_list<Case> findings)
{
  if (argc >= 2 && std::strcmp(argv[1], "--list-findings") == 0)
  {
    for (const auto& item : findings) std::printf("%s\t%s\n", item.name, item.item);
    return 0;
  }
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
