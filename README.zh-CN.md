# LibXR

<div align="center">

<img src="https://github.com/xrobot-org/LibXR_CppCodeGenerator/raw/main/imgs/XRobot.jpeg" width="300">

致力于成为最佳的嵌入式开发框架

![License](https://img.shields.io/badge/license-Apache--2.0-blue)
[![Documentation](https://img.shields.io/badge/docs-online-brightgreen)](https://xrobot.work/libxr/)
[![GitHub Issues](https://img.shields.io/github/issues/Jiu-xiao/libxr)](https://github.com/xrobot-org/libxr/issues)
[![C/C++ CI](https://github.com/xrobot-org/libxr/actions/workflows/check.yml/badge.svg)](https://github.com/xrobot-org/libxr/actions/workflows/check.yml)
[![Generate and Deploy Doxygen Docs](https://github.com/xrobot-org/libxr/actions/workflows/doxygen.yml/badge.svg)](https://github.com/xrobot-org/libxr/actions/workflows/doxygen.yml)
[![CI/CD - Python Package](https://github.com/xrobot-org/LibXR_CppCodeGenerator/actions/workflows/python-publish.yml/badge.svg)](https://github.com/xrobot-org/LibXR_CppCodeGenerator/actions/workflows/python-publish.yml)
[![FOSSA Status](https://app.fossa.com/api/projects/git%2Bgithub.com%2FJiu-xiao%2Flibxr.svg?type=shield)](https://app.fossa.com/projects/git%2Bgithub.com%2FJiu-xiao%2Flibxr?ref=badge_shield)

</div>

[English](https://github.com/xrobot-org/libxr/blob/main/README.md) | [中文](https://github.com/xrobot-org/libxr/blob/main/README.zh-CN.md)

## 本库适用于哪些人

* 不需要桌面或网页应用。
* 需要实时性与高性能。
* 希望快速且可靠地完成整个项目。
* 不想关心 API 差异。

## 硬件支持

[平台外设支持列表](./doc/support.md)

## USB 协议栈支持

请参阅 [XRUSB](https://github.com/Jiu-xiao/XRUSB)

## 系统层支持

1. 应用程序除非重启或进入低功耗模式，否则永不退出。
2. 所有内存仅在初始化时分配，不会动态释放。
3. 最小非阻塞延迟为 1 微秒，最小阻塞延迟为 1 毫秒。
4. 所有未使用函数不会被链接进最终程序。

| `System`                  | Thread | Timer | Semaphore | Mutex | Queue | ASync |
| ------------------------- | ------ | ----- | --------- | ----- | ----- | ----- |
| None                      | ✅      | ✅     | ✅         | ✅     | ✅     | ✅     |
| FreeRTOS                  | ✅      | ✅     | ✅         | ✅     | ✅     | ✅     |
| ThreadX                   | ✅      | ✅     | ✅         | ✅     | ✅     | ✅     |
| Linux                     | ✅      | ✅     | ✅         | ✅     | ✅     | ✅     |
| Webots(Linux)             | ✅      | ✅     | ✅         | ✅     | ✅     | ✅     |
| WebAssembly(SingleThread) | ✅      | ✅     | ✅         | ✅     | ✅     | ✅     |

## 数据结构支持

| `Structure` | List | Stack | RBTree | Queue | LockFreeList |
| ----------- | ---- | ----- | ------ | ------------- | ------------ |
|             | ✅    | ✅     | ✅      | ✅             | ✅            |

## 中间件支持

| `Middleware` | Event | Message | Ramfs | Terminal | Database | Log |
| ------------ | ----- | ------- | ----- | -------- | -------- | --- |
|              | ✅     | ✅       | ✅     | ✅        | ✅        | ✅   |

## 实用工具

用于调试、机器人学和通信的一些有用工具。

| Kinematics | Forward-Kinematics | Inverse-Kinematics | Coordinate | Pose and Position |
| ---------- | ------------------ | ------------------ | ---------- | ----------------- |
|            | ✅                  | ✅                  | ❌          | ✅                 |

| Dynamics | Inertia | Torque | G-Compensation |
| -------- | ------- | ------ | -------------- |
|          | ✅       | ❌      | ❌              |

| Control | PID | LQR | MPC |
| ------- | --- | --- | --- |
|         | ✅   | ❌   | ❌   |

| Signal | LP Filter | Kalman Filter | FFT | FunctionGen |
| ------ | --------- | ------------- | --- | ----------- |
|        | ❌         | ❌             | ❌   | ❌           |

| Math | CycleValue | CRC8/16/32 | Triangle |
| ---- | ---------- | ---------- | -------- |
|      | ✅          | ✅          | ❌        |

## 使用方法示例

```sh
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 配置 LibXR
set(LIBXR_SYSTEM FreeRTOS) # None/Linux/FreeRTOS...
set(LIBXR_DRIVER st)       # st/Linux/...
add_subdirectory(path_to_libxr)

target_link_libraries(${CMAKE_PROJECT_NAME}
    xr
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    PUBLIC $<TARGET_PROPERTY:xr,INTERFACE_INCLUDE_DIRECTORIES>
)
```

## 通用CMake配置

### 系统和驱动平台选择

LibXR 当前以 C++20 为目标标准，请使用已经支持 `CMAKE_CXX_STANDARD 20` 的编译器/工具链。

默认会自动识别宿主系统（Linux、Windows），自动设置 LIBXR_SYSTEM 和 LIBXR_DRIVER。你也可以在 CMake 命令行或外部 CMakeLists.txt 中预先指定，分别对应[system](./system)和[driver](./driver)下的不同文件夹。

```cmake
# 手动指定系统和驱动
set(LIBXR_SYSTEM Linux)
set(LIBXR_DRIVER Linux)
```

### 编译为共享/静态库

默认编译为静态库，可以在 CMake 命令行或外部 CMakeLists.txt 中预先指定：

```cmake
# 编译为共享库
set(LIBXR_SHARED_BUILD True)

# 编译为静态库（默认）
set(LIBXR_STATIC_BUILD True)

# 编译为 object 库
set(LIBXR_OBJECT_BUILD True)
```

### 禁用Eigen

此选项禁用 Eigen 库的编译，同时会禁用所有依赖Eigen的代码。适用于某些对C++标准实现不完全的平台。

```cmake
set(LIBXR_NO_EIGEN True)
```

### 默认标量类型

默认标量类型为 double，此选项只影响默认构造参数。

```cmake
set(LIBXR_DEFAULT_SCALAR float)
```

### 日志消息最大长度

裸机/RTOS默认为64，Linux下默认为256。

```cmake
set(XR_LOG_MESSAGE_MAX_LEN 256)
```

### 日志等级

4-0分别对应DEBUG、INFO、PASS、WARNING、ERROR，默认为4。此选项决定了允许发布到话题的最大日志级别。

```cmake
set(LIBXR_LOG_LEVEL 4)
```

### 日志打印等级

4-0分别对应DEBUG、INFO、PASS、WARNING、ERROR，默认为4。此选项决定了允许打印到STDIO::write_的最大日志级别。

### 单元测试

开启此选项为Linux平台构建单元测试。

```cmake
set(LIBXR_TEST_BUILD True)
```

自动测试在 `test/automatic/` 中镜像源码目录和源文件归属。在仓库根目录构建并执行完整测试：

```sh
cmake -S . -B build -DLIBXR_TEST_BUILD=ON -DLIBXR_DEV_ASSERT_BUILD=ON -DCMAKE_BUILD_TYPE=Debug
cmake --build build --parallel 8
ctest --test-dir build --output-on-failure --no-tests=error
```

CTest 通过 util-linux 的 `script` 程序为运行测试提供所需的伪终端。
构建矩阵、编译探针及手动测试预留目录说明见 [test/README.md](test/README.md)。

## 其他工具

### STM32 C++ 代码自动生成器

[libxr-python-package (PyPI)](https://pypi.org/project/libxr/)

## 视频教程

[Bilibili 视频教程](https://www.bilibili.com/video/BV1c8XVYLERR/)

## 开源许可证

[![FOSSA Status](https://app.fossa.com/api/projects/git%2Bgithub.com%2FJiu-xiao%2Flibxr.svg?type=large)](https://app.fossa.com/projects/git%2Bgithub.com%2FJiu-xiao%2Flibxr?ref=badge_large)
