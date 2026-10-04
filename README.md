# LibXR

面向裸机、RTOS 与 Linux 的嵌入式 C++ 框架 / Embedded C++ framework for bare-metal, RTOS and Linux

<h1 align="center">
<img src="https://github.com/xrobot-org/LibXR_CppCodeGenerator/raw/master/imgs/XRobot.jpeg" width="300">
</h1><br>

[![License](https://img.shields.io/badge/license-Apache--2.0-blue)](https://github.com/xrobot-org/libxr/blob/master/LICENSE)
[![Documentation](https://img.shields.io/badge/docs-online-brightgreen)](https://xrobot.work/libxr/)
[![GitHub Issues](https://img.shields.io/github/issues/xrobot-org/libxr)](https://github.com/xrobot-org/libxr/issues)
[![C/C++ CI](https://github.com/xrobot-org/libxr/actions/workflows/check.yml/badge.svg)](https://github.com/xrobot-org/libxr/actions/workflows/check.yml)
[![Generate and Deploy Doxygen Docs](https://github.com/xrobot-org/libxr/actions/workflows/doxygen.yml/badge.svg)](https://github.com/xrobot-org/libxr/actions/workflows/doxygen.yml)
[![CI/CD - Python Package](https://github.com/xrobot-org/LibXR_CppCodeGenerator/actions/workflows/python-publish.yml/badge.svg)](https://github.com/xrobot-org/LibXR_CppCodeGenerator/actions/workflows/python-publish.yml)
[![FOSSA Status](https://app.fossa.com/api/projects/git%2Bgithub.com%2FJiu-xiao%2Flibxr.svg?type=shield)](https://app.fossa.com/projects/git%2Bgithub.com%2FJiu-xiao%2Flibxr?ref=badge_shield)

LibXR 是用 C++20 编写的嵌入式框架，同一份应用代码可以在裸机、FreeRTOS、ThreadX 和 Linux 等系统上运行。
它把操作系统封装为统一的线程、互斥锁、信号量和软件定时器接口，把各平台的外设封装为 `LibXR::UART`、
`LibXR::SPI` 这样的抽象类，并提供消息中间件、USB 设备协议栈 XRUSB、数据结构和机器人学工具。

LibXR is an embedded framework written in C++20; the same application code runs on bare metal,
FreeRTOS, ThreadX, Linux and other systems. It wraps the operating system in common thread, mutex,
semaphore and software timer interfaces, wraps the peripherals of each platform in abstract
classes such as `LibXR::UART` and `LibXR::SPI`, and provides a message middleware, the USB device
stack XRUSB, data structures and robotics utilities.

---

## 🔧 安装 / Installation

LibXR 以源码形式加入 CMake 工程，需要支持 C++20 的编译器和 CMake 3.12 或更高版本。工程通常以 Git
子模块引入 LibXR，设置系统层和驱动层后用 `add_subdirectory` 加入构建，再链接目标 `xr`：

LibXR is added to a CMake project as source code and needs a C++20 compiler and CMake 3.12 or
later. A project usually adds LibXR as a Git submodule, sets the system and driver layers,
adds it with `add_subdirectory` and links the target `xr`:

```bash
git submodule add https://github.com/xrobot-org/libxr.git libxr
```

```cmake
set(LIBXR_SYSTEM FreeRTOS)
set(LIBXR_DRIVER st)
add_subdirectory(libxr)
target_link_libraries(${CMAKE_PROJECT_NAME} xr)
```

`LIBXR_SYSTEM` 和 `LIBXR_DRIVER` 分别对应 [`system/`](https://github.com/xrobot-org/libxr/tree/master/system)
和 [`driver/`](https://github.com/xrobot-org/libxr/tree/master/driver) 下的目录名。在 Linux 宿主上本地编译且
没有设置这两个变量时，二者均为 `linux`，设置了 `WEBOTS_HOME` 时为 `webots`。Windows 上的主机程序在 WSL
或 `ghcr.io/xrobot-org/docker-image-linux` 镜像中构建。

`LIBXR_SYSTEM` and `LIBXR_DRIVER` name directories under
[`system/`](https://github.com/xrobot-org/libxr/tree/master/system) and
[`driver/`](https://github.com/xrobot-org/libxr/tree/master/driver). For a native build on a
Linux host without these variables, both are `linux`, or `webots` when `WEBOTS_HOME` is set.
Host programs on Windows are built under WSL or in the `ghcr.io/xrobot-org/docker-image-linux`
image.

STM32CubeMX 工程可以由 [LibXR_CppCodeGenerator](https://github.com/xrobot-org/LibXR_CppCodeGenerator)
加入 LibXR 并生成外设对象；使用可复用模块的工程由 [XRobot](https://github.com/xrobot-org/XRobot) 管理。

STM32CubeMX projects can get LibXR and their peripheral objects from
[LibXR_CppCodeGenerator](https://github.com/xrobot-org/LibXR_CppCodeGenerator); projects built
from reusable Modules are managed by [XRobot](https://github.com/xrobot-org/XRobot).

---

## 📚 基本概念 / Concepts

以一块运行 FreeRTOS 的 STM32F407 板子为例：USART1 用作命令行终端，SPI1 上接一颗 IMU，一个线程周期
读取 IMU 并把数据交给其他线程。各个概念的对应关系如下：

Take an STM32F407 board running FreeRTOS: USART1 serves as the command-line terminal, an IMU is on
SPI1, and a thread reads the IMU periodically and hands the data to other threads. The concepts
correspond as follows:

| 概念 Concept | 在这个例子里 | In this example |
| --- | --- | --- |
| 系统层 System layer | `system/freertos`：`LibXR::Thread`、`Mutex`、`Semaphore` 和 `Timer` 由 FreeRTOS 实现；同一份应用代码在 Linux 上使用 `system/linux` | `system/freertos`: `LibXR::Thread`, `Mutex`, `Semaphore` and `Timer` are implemented on FreeRTOS; the same application code uses `system/linux` on Linux |
| 驱动层 Driver layer | `driver/st`：`STM32UART`、`STM32SPI` 实现抽象类 `LibXR::UART`、`LibXR::SPI`，应用代码只使用抽象类 | `driver/st`: `STM32UART` and `STM32SPI` implement the abstract classes `LibXR::UART` and `LibXR::SPI`; application code uses only the abstract classes |
| 时基与平台初始化 Timebase and platform init | `STM32TimerTimebase` 提供毫秒与微秒时间，随后 `PlatformInit()` 完成系统层的初始化，并设置软件定时器线程的优先级和栈大小 | `STM32TimerTimebase` provides millisecond and microsecond time, then `PlatformInit()` initializes the system layer and sets the priority and stack size of the software timer thread |
| 读写操作 Read/write operation | 串口读写时传入 `ReadOperation` 或 `WriteOperation`，决定调用阻塞等待、轮询状态还是完成时回调 | UART reads and writes take a `ReadOperation` or `WriteOperation`, which selects blocking, polling or a completion callback |
| Topic | IMU 线程把数据发布到名为 `imu` 的 Topic，订阅者以回调、队列或等待的方式接收 | The IMU thread publishes to a Topic named `imu`; subscribers receive by callback, queue or waiting |
| 终端 Terminal | `LibXR::Terminal` 在 USART1 上提供命令行，命令是 `RamFS` 中的可执行文件 | `LibXR::Terminal` provides a command line on USART1; the commands are executable files in `RamFS` |

---

## 🧵 系统层 / System Layer

系统层提供线程、互斥锁、信号量、软件定时器和异步任务（`ASync`），已有 None（裸机）、FreeRTOS、ThreadX、
Linux、Webots 和 WebAssembly 六种实现。软件定时器的任务在一个管理线程中按毫秒周期运行；None 和
WebAssembly 没有多线程，定时器在空闲等待时刷新。以下程序在 Linux 上创建一个 10 ms 周期的定时器任务：

The system layer provides threads, mutexes, semaphores, software timers and asynchronous jobs
(`ASync`), with six implementations: None (bare metal), FreeRTOS, ThreadX, Linux, Webots and
WebAssembly. Software timer tasks run with millisecond periods in one management thread; None
and WebAssembly have no threads, and the timer is refreshed while waiting. The program below
creates a timer task with a 10 ms period on Linux:

```cpp
LibXR::PlatformInit();

static int ticks = 0;
auto task = LibXR::Timer::CreateTask<int*>([](int* count) { ++*count; }, &ticks, 10);
LibXR::Timer::Add(task);
LibXR::Timer::Start(task);

LibXR::Thread::Sleep(500);
std::printf("timer ticks after 500 ms: %d\n", ticks);
```

```text
timer ticks after 500 ms: 50
```

各系统的实现与限制见 [操作系统](https://xrobot.work/docs/basic_coding/system)。

The implementations and limits of each system are described in
[Operating System](https://xrobot.work/en/docs/basic_coding/system).

---

## 🔌 外设驱动 / Peripheral Drivers

抽象类包括 `GPIO`、`UART`、`SPI`、`I2C`、`CAN`/`FDCAN`、`ADC`、`DAC`、`PWM`、`Flash`、`Watchdog`、
`PowerManager` 和 `Timebase`，平台驱动位于 `driver/` 下的 `st`（STM32）、`ch`（CH32）、`esp`（ESP32）、
`hpm`（HPMicro）、`mspm0`（TI MSPM0）、`linux`、`webots` 和 `webasm`。应用代码以抽象类的引用使用外设，换
平台时只替换创建对象的那几行。读写接口带一个操作对象，决定这次调用如何完成，下例传入信号量，`Write`
阻塞到发送完成或 100 ms 超时：

The abstract classes are `GPIO`, `UART`, `SPI`, `I2C`, `CAN`/`FDCAN`, `ADC`, `DAC`, `PWM`, `Flash`,
`Watchdog`, `PowerManager` and `Timebase`, and the platform drivers are under `driver/`: `st`
(STM32), `ch` (CH32), `esp` (ESP32), `hpm` (HPMicro), `mspm0` (TI MSPM0), `linux`, `webots` and
`webasm`. Application code uses peripherals through references to the abstract classes, so
moving to another platform replaces only the lines that create the objects. Read and write
calls take an operation object that selects how the call completes; below it is a semaphore,
and `Write` blocks until the data is sent or 100 ms pass:

```cpp
void Report(LibXR::UART& uart)
{
  static LibXR::Semaphore sem;
  static LibXR::WriteOperation op(sem, 100);
  uart.Write(LibXR::ConstRawData("ready\r\n"), op);
}

// STM32 工程的 app_main() 中（节选） / In app_main() of an STM32 project (excerpt)
static STM32UART usart1(&huart1, usart1_rx_buf, usart1_tx_buf, 5);
Report(usart1);
```

各平台已支持的外设见 [平台外设支持列表](doc/support.md)，接口说明见
[外设驱动](https://xrobot.work/docs/basic_coding/driver)。

The peripherals supported on each platform are listed in the
[platform peripheral support list](doc/support.md), and the interfaces are described in
[Device Drivers](https://xrobot.work/en/docs/basic_coding/driver).

---

## 🔗 USB 设备协议栈 / USB Device Stack

XRUSB 是 LibXR 的 USB 设备协议栈，位于 `src/driver/usb`。设备类有 CDC-ACM（作为 `LibXR::UART` 使用）、
HID 键盘、鼠标与手柄、UAC1 麦克风、GS USB（Linux SocketCAN 使用的 CAN/CAN FD 适配器）、DAPLink V1 与 V2
（CMSIS-DAP 调试器）以及 DFU 运行时与 bootloader，并支持 WebUSB 和 WinUSB MS OS 2.0 描述符。设备控制器
驱动覆盖 STM32 的 FSDEV、OTG FS 与 OTG HS，ESP32-S3 的 OTG FS，以及 CH32 的 FSDEV、USBFS 与 USBHS。

XRUSB is the USB device stack of LibXR, in `src/driver/usb`. The device classes are CDC-ACM
(used as a `LibXR::UART`), HID keyboard, mouse and gamepad, a UAC1 microphone, GS USB (a CAN/CAN FD
adapter for Linux SocketCAN), DAPLink V1 and V2 (CMSIS-DAP debug probes) and DFU runtime and
bootloader, with WebUSB and WinUSB MS OS 2.0 descriptors. The device controller drivers cover
FSDEV, OTG FS and OTG HS on STM32, OTG FS on ESP32-S3, and FSDEV, USBFS and USBHS on CH32.

设备类与平台的支持情况见 [XRUSB](src/driver/usb/README.md)，用法见
[XRUSB协议栈](https://xrobot.work/docs/xrusb)。

The support matrix is in [XRUSB](src/driver/usb/README.md), and usage is described in
[XRUSB Stack](https://xrobot.work/en/docs/xrusb).

---

## 📨 中间件 / Middleware

`Topic` 是进程内按类型检查的发布订阅通道，订阅者以回调、队列、异步或同步等待的方式接收；Linux 上另有
跨进程的 `LinuxSharedTopic`。其余中间件包括事件（`Event`）、内存文件系统 `RamFS` 与命令行终端 `Terminal`、
保存在 Flash 上的键值数据库（`DatabaseRaw`、`DatabaseRawSequential`）以及日志（`XR_LOG_INFO` 等宏）。下例
由一个线程发布三次温度，回调订阅者逐次打印：

`Topic` is an in-process publish-subscribe channel with type checking, and subscribers receive by
callback, queue, asynchronous or synchronous waiting; Linux also has the inter-process
`LinuxSharedTopic`. The other middleware components are events (`Event`), the in-memory file
system `RamFS` with the command-line `Terminal`, key-value databases stored in Flash
(`DatabaseRaw`, `DatabaseRawSequential`) and logging (`XR_LOG_INFO` and related macros). In the
example below a thread publishes a temperature three times and a callback subscriber prints
each value:

```cpp
void Producer(LibXR::Topic* topic)
{
  for (int i = 1; i <= 3; ++i)
  {
    float temperature = 20.0f + 0.5f * static_cast<float>(i);
    topic->Publish(temperature);
    LibXR::Thread::Sleep(100);
  }
}

// main() 中 / in main()
auto topic = LibXR::Topic::CreateTopic<float>("temperature");
auto on_data = LibXR::Topic::Callback::Create(
    [](bool, void*, float& value) { std::printf("temperature = %.1f\n", value); },
    static_cast<void*>(nullptr));
topic.RegisterCallback(on_data);

LibXR::Thread producer;
producer.Create<LibXR::Topic*>(&topic, Producer, "producer", 4096,
                               LibXR::Thread::Priority::MEDIUM);
```

```text
temperature = 20.5
temperature = 21.0
temperature = 21.5
```

各组件的说明见 [中间件](https://xrobot.work/docs/basic_coding/middleware)。

Each component is described in [Middleware](https://xrobot.work/en/docs/basic_coding/middleware).

---

## 🧮 数据结构与工具 / Data Structures and Utilities

数据结构包括链表（`List`、`LockFreeList`）、栈、红黑树、队列（`Queue`、`SPSCQueue`、`MPMCQueue`）、对象池和
双缓冲。工具包括 CRC8/16/32/64、周期值、PID 控制器、正逆运动学、位姿与姿态变换（`Position`、`EulerAngle`、
`RotationMatrix`、`Quaternion`）、惯量、浮点编码和标志位。核心组件提供 `RawData`、回调、`Pipe`、格式化输出以及
`ReadPort`/`WritePort` 读写端口。

The data structures are linked lists (`List`, `LockFreeList`), a stack, a red-black tree, queues
(`Queue`, `SPSCQueue`, `MPMCQueue`), object pools and double buffers. The utilities are CRC8/16/32/64,
cyclic values, a PID controller, forward and inverse kinematics, pose and attitude transforms
(`Position`, `EulerAngle`, `RotationMatrix`, `Quaternion`), inertia, float encoding and flags. The
core provides `RawData`, callbacks, `Pipe`, formatted output and the `ReadPort`/`WritePort` I/O
ports.

说明见 [数据结构](https://xrobot.work/docs/basic_coding/structure)、
[数学与工具](https://xrobot.work/docs/basic_coding/utils) 和 [核心组件](https://xrobot.work/docs/basic_coding/core)。

They are described in [Data Structures](https://xrobot.work/en/docs/basic_coding/structure),
[Utilities and Math](https://xrobot.work/en/docs/basic_coding/utils) and
[Core Components](https://xrobot.work/en/docs/basic_coding/core).

---

## ⚙️ CMake 选项 / CMake Options

| 变量 Variable | 默认值 Default | 说明 | Description |
| --- | --- | --- | --- |
| `LIBXR_SYSTEM` | Linux 宿主为 `linux` / `linux` on a Linux host | 系统层，`system/` 下的目录名 | System layer, a directory name under `system/` |
| `LIBXR_DRIVER` | Linux 宿主为 `linux` / `linux` on a Linux host | 驱动层，`driver/` 下的目录名 | Driver layer, a directory name under `driver/` |
| `LIBXR_SHARED_BUILD`、`LIBXR_STATIC_BUILD`、`LIBXR_OBJECT_BUILD` | 静态库 / static library | 编译为共享库、静态库或 object 库 | Build a shared, static or object library |
| `LIBXR_NO_EIGEN` | 未设置 / unset | 不使用 Eigen，依赖 Eigen 的代码一并关闭 | Leave out Eigen and the code that depends on it |
| `LIBXR_DEFAULT_SCALAR` | `double` | 数学工具类的默认标量类型 | Default scalar type of the math utility classes |
| `XR_LOG_MESSAGE_MAX_LEN` | Linux、Webots、WebAssembly 为 256，其余为 64 / 256 on Linux, Webots and WebAssembly, otherwise 64 | 单条日志的最大长度 | Maximum length of one log message |
| `LIBXR_LOG_LEVEL` | `4` | 发布到日志 Topic 的最高等级，4 到 0 依次为 DEBUG、INFO、PASS、WARNING、ERROR | Highest level published to the log Topic; 4 to 0 are DEBUG, INFO, PASS, WARNING and ERROR |
| `LIBXR_LOG_OUTPUT_LEVEL` | `4` | 打印到 `STDIO::write_` 的最高等级 | Highest level printed to `STDIO::write_` |
| `LIBXR_PRINT_*` | 见文档 / see the docs | 格式化输出支持的转换和功能 | Conversions and features supported by formatted output |
| `LIBXR_TEST_BUILD` | 未设置 / unset | 在 Linux 上构建自动测试 | Build the automatic tests on Linux |
| `XROBOT_MODULES_DIR` | 未设置 / unset | XRobot 工程的 `Modules/` 目录，设置后构建其中的模块，并检查生成的主函数是否为最新 | `Modules/` of an XRobot project; when set, its Modules are built and the generated main function is checked for being up to date |

`LIBXR_PRINT_*` 的各项见 [编译期格式化输出](https://xrobot.work/docs/basic_coding/core/core-print)，完整说明见
[CMake 配置](https://xrobot.work/docs/basic_coding/basic-cmake)。

The `LIBXR_PRINT_*` settings are listed in
[Compile-Time Formatting](https://xrobot.work/en/docs/basic_coding/core/core-print), and all options are
described in [CMake Configuration](https://xrobot.work/en/docs/basic_coding/basic-cmake).

---

## 🧪 测试 / Tests

```bash
cmake -S . -B build -DLIBXR_TEST_BUILD=ON -DLIBXR_DEV_ASSERT_BUILD=ON -DCMAKE_BUILD_TYPE=Debug
cmake --build build --parallel 8
ctest --test-dir build --output-on-failure --no-tests=error
```

测试需要 util-linux 的 `script` 提供伪终端，构建矩阵与目录约定见
[test/README.md](https://github.com/xrobot-org/libxr/blob/master/test/README.md)。

The tests need `script` from util-linux for a pseudo-terminal; the build matrix and the directory
layout are described in [test/README.md](https://github.com/xrobot-org/libxr/blob/master/test/README.md).

---

## 📖 更多信息 / More Information

- [文档 / Documentation](https://xrobot.work/docs/basic_coding)
- [API 文档 / API Reference](https://xrobot.work/libxr/)
- [平台外设支持列表 / Platform Peripheral Support](doc/support.md)
- [XRUSB](src/driver/usb/README.md)
- [LibXR_CppCodeGenerator](https://github.com/xrobot-org/LibXR_CppCodeGenerator)
- [XRobot](https://github.com/xrobot-org/XRobot)
- [视频教程 / Video Tutorial (Bilibili)](https://www.bilibili.com/video/BV1c8XVYLERR/)
- [Issue Tracker](https://github.com/xrobot-org/libxr/issues)

[![FOSSA Status](https://app.fossa.com/api/projects/git%2Bgithub.com%2FJiu-xiao%2Flibxr.svg?type=large)](https://app.fossa.com/projects/git%2Bgithub.com%2FJiu-xiao%2Flibxr?ref=badge_large)
