/**
 * @file sim_findings.hpp
 * @brief 已知缺陷用例的运行入口 / Runner for the known-defect cases.
 *
 * 仿真测试里有一类用例检查的行为候选线的库目前做不到（见 README.md 的"发现"一节）。
 * 它们保持完整的断言，不放进默认的 ctest：测试程序不带参数时只跑常规用例；
 * `--list-findings` 列出这些用例和对应的缺陷条目，`--finding <名字>` 单独跑一个。
 * 断言失败时进程终止并输出位置，库修好后同一条命令返回 0，此时应把用例移进常规用例。
 * Some simulation cases check behaviour the candidate library does not have yet (see the
 * findings section of README.md). They keep their full assertions and are left out of the
 * default ctest: without arguments a test program runs the regular cases only;
 * `--list-findings` lists these cases with the defect item, `--finding <name>` runs one.
 * A failed assertion aborts with its location; once the library is fixed the same
 * command returns 0, and the case then belongs among the regular ones.
 */

#pragma once

#include <cstdio>
#include <cstring>
#include <initializer_list>

namespace LibXR::USB::Sim
{

struct FindingCase
{
  const char* name;  ///< 用例名 / Case name
  const char* item;  ///< 缺陷条目 / Defect item
  void (*run)();
};

/// 测试程序的 main：regular 是常规用例，findings 是已知缺陷用例
/// Main of a test program: regular holds the regular cases, findings the known-defect
/// cases
inline int RunTestProgram(int argc, char** argv, void (*regular)(),
                          std::initializer_list<FindingCase> findings)
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
  regular();
  return 0;
}

}  // namespace LibXR::USB::Sim
