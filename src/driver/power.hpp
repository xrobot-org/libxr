#pragma once

#include "libxr.hpp"

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
   *
   */
  virtual void JumpToBootloader() { Reset(); }
};

}  // namespace LibXR
