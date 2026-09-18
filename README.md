# LibXR

<div align="center">

<img src="https://github.com/xrobot-org/LibXR_CppCodeGenerator/raw/main/imgs/XRobot.jpeg" width="300">

Want to be the best embedded framework

![License](https://img.shields.io/badge/license-Apache--2.0-blue)
[![Documentation](https://img.shields.io/badge/docs-online-brightgreen)](https://xrobot.work/libxr/)
[![GitHub Issues](https://img.shields.io/github/issues/Jiu-xiao/libxr)](https://github.com/xrobot-org/libxr/issues)
[![C/C++ CI](https://github.com/xrobot-org/libxr/actions/workflows/check.yml/badge.svg)](https://github.com/xrobot-org/libxr/actions/workflows/check.yml)
[![Generate and Deploy Doxygen Docs](https://github.com/xrobot-org/libxr/actions/workflows/doxygen.yml/badge.svg)](https://github.com/xrobot-org/libxr/actions/workflows/doxygen.yml)
[![CI/CD - Python Package](https://github.com/xrobot-org/LibXR_CppCodeGenerator/actions/workflows/python-publish.yml/badge.svg)](https://github.com/xrobot-org/LibXR_CppCodeGenerator/actions/workflows/python-publish.yml)
[![FOSSA Status](https://app.fossa.com/api/projects/git%2Bgithub.com%2FJiu-xiao%2Flibxr.svg?type=shield)](https://app.fossa.com/projects/git%2Bgithub.com%2FJiu-xiao%2Flibxr?ref=badge_shield)

</div>

[English](https://github.com/xrobot-org/libxr/blob/main/README.md) | [中文](https://github.com/xrobot-org/libxr/blob/main/README.zh-CN.md)

## Who is this library for

* People who do not require desktop and web applications.
* Realtime and high performance.
* Finish the entire project quickly and reliably.
* Don't want to care about the differences in APIs.

## Hardware Support

[Platform Peripheral Support List](./doc/support.md)

## USB Stack Support

See [XRUSB](https://github.com/Jiu-xiao/XRUSB)

## System Layer

1. The application will never exit unless it reboot or enter into low-power mode.
2. All memory is only allocated during initializtion and will never be released.
3. The minimum Non-blocking delay is 1us, minimum blocking delay is 1ms.
4. All unused functions will not be linked.

| `System`                  | Thread | Timer | Semaphore | Mutex | Queue | ASync |
| ------------------------- | ------ | ----- | --------- | ----- | ----- | ----- |
| None                      | ✅      | ✅     | ✅         | ✅     | ✅     | ✅     |
| FreeRTOS                  | ✅      | ✅     | ✅         | ✅     | ✅     | ✅     |
| ThreadX                   | ✅      | ✅     | ✅         | ✅     | ✅     | ✅     |
| Linux                     | ✅      | ✅     | ✅         | ✅     | ✅     | ✅     |
| Webots(Linux)             | ✅      | ✅     | ✅         | ✅     | ✅     | ✅     |
| WebAssembly(SingleThread) | ✅      | ✅     | ✅         | ✅     | ✅     | ✅     |

## Data structure

| `Structure` | List | Stack | RBTree | Queue | LockFreeList |
| ----------- | ---- | ----- | ------ | ------------- | ------------ |
|             | ✅    | ✅     | ✅      | ✅             | ✅            |

## Middleware

| `Middleware` | Event | Message | Ramfs | Terminal | Database | Log |
| ------------ | ----- | ------- | ----- | -------- | -------- | --- |
|              | ✅     | ✅       | ✅     | ✅        | ✅        | ✅   |

## Utils

Some useful tools for debugging, robotics, and communication.

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

## Usage

```sh
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# LibXR
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

## General CMake Configuration

### System and Driver Platform Selection

LibXR currently targets C++20. Use a compiler/toolchain that already supports `CMAKE_CXX_STANDARD 20`.

By default, the host system (Linux, Windows) is automatically detected, and `LIBXR_SYSTEM` and `LIBXR_DRIVER` are set accordingly. You can also manually specify them via the CMake command line or in an external `CMakeLists.txt`, corresponding to the different folders under [system](./system) and [driver](./driver).

```cmake
# Manually specify system and driver
set(LIBXR_SYSTEM Linux)
set(LIBXR_DRIVER Linux)
```

### Build as Shared/Static Library

By default, the library is built as a static target. You can explicitly set the build type in the CMake command line or your own CMakeLists.txt:

```cmake
# Build as a shared library
set(LIBXR_SHARED_BUILD True)

# Build as a static library (default)
set(LIBXR_STATIC_BUILD True)

# Build as an object library
set(LIBXR_OBJECT_BUILD True)
```

### Disable Eigen

This option disables building the Eigen library and will also disable any code depending on Eigen. This is useful for platforms with incomplete C++ standard library support.

```cmake
set(LIBXR_NO_EIGEN True)
```

### Default Scalar Type

The default scalar type is `double`. This option only affects the default value for constructors.

```cmake
set(LIBXR_DEFAULT_SCALAR float)
```

### Maximum Log Message Length

Defaults to 64 for bare-metal/RTOS, and 256 for Linux.

```cmake
set(XR_LOG_MESSAGE_MAX_LEN 256)
```

### Log Level

Levels 4-0 correspond to DEBUG, INFO, PASS, WARNING, and ERROR, with the default set to 4. This option determines the maximum log level allowed to be published to topics.

```cmake
set(LIBXR_LOG_LEVEL 4)
```

### Log Print Level

Levels 4-0 correspond to DEBUG, INFO, PASS, WARNING, and ERROR, with the default set to 4. This option determines the maximum log level allowed to be printed to `STDIO::write_`.

### Unit Testing

Enable this option to build unit tests on the Linux platform.

```cmake
set(LIBXR_TEST_BUILD True)
```

Automatic tests mirror source directories and file ownership in `test/automatic/`. Build and run the
complete suite from the repository root:

```sh
cmake -S . -B build -DLIBXR_TEST_BUILD=ON -DLIBXR_DEV_ASSERT_BUILD=ON -DCMAKE_BUILD_TYPE=Debug
cmake --build build --parallel 8
ctest --test-dir build --output-on-failure --no-tests=error
```

CTest provides the pseudo-terminal required by the runtime runner through the
util-linux `script` program. See [test/README.md](test/README.md) for the build
matrix, compile probes, and reserved manual-test directories.

## Others

### STM32 C++ Code Generator

[libxr-python-package](https://pypi.org/project/libxr/)

## Video Tutorial

[Bilibili](https://www.bilibili.com/video/BV1c8XVYLERR/)

## License

[![FOSSA Status](https://app.fossa.com/api/projects/git%2Bgithub.com%2FJiu-xiao%2Flibxr.svg?type=large)](https://app.fossa.com/projects/git%2Bgithub.com%2FJiu-xiao%2Flibxr?ref=badge_large)
