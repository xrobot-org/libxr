# LibXR 测试 / LibXR tests

测试分为两类：

There are two kinds of tests:

- **automatic**：在 Linux 上运行，由 CI 自动执行。测试核心功能和 Linux 能验证的系统、驱动行为。
  Runs on Linux in CI. Checks core functionality and system or driver behavior that can be tested on Linux.
- **manual**：需要具体系统或外设，由用户准备环境后调用。[system](manual/system/README.md) 已提供可调用的系统测试；[driver](manual/driver/README.md) 提供 GPIO、PWM、ADC、DAC、Flash、I2C、SPI、CAN / CAN FD 和 UART 测试。
  Needs a particular system or peripheral and is called after the user sets it up. [system](manual/system/README.md) provides callable system tests; [driver](manual/driver/README.md) provides GPIO, PWM, ADC, DAC, Flash, I2C, SPI, CAN / CAN FD and UART tests.

## 构建并运行 / Build and run

需要支持 C++20 的编译器、CMake，以及 util-linux 提供的 `script`。Linux 开发包可参考 [CI 的安装步骤](../.github/workflows/check.yml)。在仓库根目录执行：

You need a C++20 compiler, CMake and `script` from util-linux. See the [CI installation steps](../.github/workflows/check.yml) for Linux development packages. Run from the repository root:

```sh
cmake -S . -B build -DLIBXR_TEST_BUILD=ON -DLIBXR_DEV_ASSERT_BUILD=ON -DCMAKE_BUILD_TYPE=Debug
cmake --build build --parallel 8
ctest --test-dir build --output-on-failure --no-tests=error
```

CTest 会运行已登记的自动测试。终端相关测试需要伪终端，CTest 已通过 `script` 处理，无需另外打开串口。任何测试失败都会使 CTest 返回失败；完整程序的超时为 300 秒。

CTest runs the registered automatic tests. It uses `script` to provide the pseudo-terminal needed by terminal tests; no serial device is required. Any failed test makes CTest fail. The full program has a 300-second timeout.

只运行一组测试时，用 `--case` 指定名称。例如：

Use `--case` to run one group. For example:

```sh
script -q -e -c './build/test/libxr_test --case lockfree_list' /dev/null
```

名称列在 [automatic/main.cpp](automatic/main.cpp) 中。可执行文件为 `build/test/libxr_test`，对应的 CMake 目标名为 `libxr_test`。需要查看完整输出时使用：

Case names are listed in [automatic/main.cpp](automatic/main.cpp). The executable is `build/test/libxr_test`, and its CMake target is `libxr_test`. To see the full output, run:

```sh
ctest --test-dir build -V -R '^libxr_automatic$' --no-tests=error
```

## 构建选项 / Build options

日常修改可以使用上面的 Debug 配置。CI 还会运行两种 Release 配置，检查优化后的代码，以及关闭开发断言后是否仍能正常运行。

Use the Debug configuration above for everyday development. CI also runs two Release configurations to check optimized code, both with and without developer assertions.

| `CMAKE_BUILD_TYPE` | `LIBXR_DEV_ASSERT_BUILD` | 用途 / Purpose |
| --- | --- | --- |
| Debug | ON | 调试时运行全部检查 / Run all checks while debugging |
| Release | ON | 优化后保留开发断言 / Keep developer assertions in optimized code |
| Release | OFF | 使用默认关闭开发断言的配置 / Test with developer assertions disabled by default |

断言宏本身另有四个小测试，分别检查 `ASSERT` 和 `DEV_ASSERT` 开关的四种组合；这四项不是完整测试程序的运行模式。构建时还会检查关闭断言的公共头文件，以及不同打印配置能否编译。测试中的 `TEST_ASSERT` 始终生效。

Four separate small tests check all on/off combinations of `ASSERT` and `DEV_ASSERT`; they are not four modes of the full test program. The build also checks public headers with assertions disabled and compiles several print configurations. `TEST_ASSERT` in test code is always active.

只构建产品库时设置 `LIBXR_TEST_BUILD=OFF`，此时不构建测试，也不需要 `script`。

Set `LIBXR_TEST_BUILD=OFF` to build only the library. Tests and the requirement for `script` are then omitted.

## 测试放在哪里 / Where tests belong

`automatic` 按被测接口组织，源文件名去掉扩展名后作为测试目录。同名 `.hpp` 和 `.cpp` 共用一个目录。`driver` 和 `system` 按统一抽象分类，使用哪个平台的实现不改变测试归属。例如：

`automatic` follows the interface being tested. Remove the source file extension to get its directory; matching `.hpp` and `.cpp` share one. Group driver and system tests by the common abstraction, regardless of the platform implementation used. For example:

| 源码 / Source | 测试目录 / Tests |
| --- | --- |
| `src/core/libxr_pipe.hpp` | `automatic/core/libxr_pipe/` |
| `src/core/rw/read_port.*` | `automatic/core/rw/read_port/` |
| `src/structure/lockfree_list.*` | `automatic/structure/lockfree_list/` |
| `src/middleware/terminal/command.hpp` | `automatic/middleware/terminal/command/` |
| `src/driver/uart.hpp` | `automatic/driver/uart/` |
| `src/system/semaphore.hpp` | `automatic/system/semaphore/` |
| `src/middleware/message/topic.*` | `automatic/middleware/message/topic/` |

只用一次的辅助代码留在测试文件里。多个文件共用的辅助放在最近的公共目录。具体测试的运行前提或特殊检查方法，写在所属目录的 README 中。

Keep single-use helpers in the test file. Put shared helpers in the nearest common directory. Explain any special setup or checking method in a README beside the relevant tests.

## 添加测试 / Add a test

1. 将文件放到对应目录，在 [automatic/CMakeLists.txt](automatic/CMakeLists.txt) 中列出源码，并接入现有入口或 `main.cpp`。
   Put the file in the matching directory, list it in [automatic/CMakeLists.txt](automatic/CMakeLists.txt), and call it from an existing entry or `main.cpp`.
2. 每个测试文件在头部说明测什么，有特殊运行前提时一并注明；正文注释解释关键步骤的原因和预期结果。用 `TEST_ASSERT` 检查结果，简单赋值和直观断言无需逐行解释。
   Start each test file with a comment describing its checks and any special setup requirements. Explain important steps and expected results beside the code. Use `TEST_ASSERT`; simple assignments and obvious assertions need no line-by-line commentary.
3. 使用全局状态、常驻线程或子进程时，在相关代码旁说明生命周期。执行器的隔离选项见 `main.cpp` 中的 `TestCase`。
   When using global state, permanent threads or child processes, explain their lifetime beside the code. See `TestCase` in `main.cpp` for isolation options.
