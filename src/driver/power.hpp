#pragma once

#include "gpio.hpp"
#include "libxr.hpp"
#include "ramfs.hpp"

namespace LibXR
{

/**
 * @class PowerManager
 * @brief 电源管理器基类 / Abstract base class for Power Manager
 *
 * 该类定义了电源管理的基本接口，所有电源管理模块应继承此类并实现具体的 `Reset` 和
 * `Shutdown` 方法。
 * This class defines the basic interface for power management. All power management
 * modules should inherit from this class and implement the `Reset` and `Shutdown`
 * methods.
 */
class PowerManager
{
 public:
  /**
   * @brief 默认构造函数 / Default constructor
   */
  PowerManager() = default;

  /**
   * @brief 默认析构函数 / Default destructor
   */
  virtual ~PowerManager() = default;

  /**
   * @brief 复位电源管理模块 / Resets the power management module
   *
   * 该方法应由子类实现，用于执行特定的复位操作，例如重启电源控制器或恢复默认设置。
   * This method should be implemented by subclasses to perform specific reset operations,
   * such as restarting the power controller or restoring default settings.
   */
  virtual void Reset() = 0;

  /**
   * @brief 关闭系统电源 / Shuts down the system power
   *
   * 该方法应由子类实现，用于执行系统关机操作，例如断开电源或进入低功耗模式。
   * This method should be implemented by subclasses to perform system shutdown
   * operations, such as cutting off power or entering a low-power mode.
   */
  virtual void Shutdown() = 0;

  /**
   * @brief 跳转到启动加载器 / Jumps to the bootloader
   *
   * 平台实现进入芯片的启动加载器（例如 ROM BSL 或 ISP）；未实现的平台退化为 `Reset()`。
   * Platform implementations enter the bootloader of the chip (such as a ROM BSL or ISP);
   * platforms without one fall back to `Reset()`.
   */
  virtual void JumpToBootloader() { Reset(); }

  /**
   * @brief 在 RamFS 根目录注册 power 命令 / Registers the power command in the RamFS root
   *
   * 终端中 `power reset`、`power shutdown`、`power bootloader` 分别调用 `Reset()`、
   * `Shutdown()`、`JumpToBootloader()`；没有参数或参数不认识时打印用法并返回 -1。命令文件
   * 在第一次注册时分配，之后不释放；同一个对象只应注册一次。
   * In the terminal, `power reset`, `power shutdown` and `power bootloader` call
   * `Reset()`, `Shutdown()` and `JumpToBootloader()`; without an argument or with an
   * unknown one the command prints its usage and returns -1. The command file is
   * allocated on registration and never freed; register one object only once.
   *
   * @param ramfs 终端使用的文件系统 / The file system the terminal uses
   */
  void RegisterCommand(RamFS& ramfs);

  /**
   * @brief 启动时按引脚电平进入启动加载器 / Pin-level bootloader entry at startup
   *
   * 读一次 `pin`，电平等于 `level` 时调用 `JumpToBootloader()`，否则直接返回。
   * 在初始化早期调用一次，按住该引脚对应的按键再复位即可进入启动加载器；
   * 运行中该引脚仍归程序使用。引脚的输入方向和上下拉由工程配置决定，
   * 这里不修改；悬空的引脚可能误触发。
   * Reads `pin` once and calls `JumpToBootloader()` when it reads `level`, otherwise
   * returns. Called once early during initialization, so holding the key on that pin
   * through a reset enters the bootloader, while the pin stays with the application at
   * run time. The input direction and pull of the pin come from the project
   * configuration and are left unchanged; a floating pin may trigger by accident.
   *
   * @param pin 检测的引脚 / The pin to check
   * @param level 进入启动加载器的电平 / The level that enters the bootloader
   */
  void CheckBootloaderPin(GPIO& pin, bool level);
};

}  // namespace LibXR
