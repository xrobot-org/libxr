#pragma once

#include <ti/driverlib/m0p/dl_systick.h>

#include "timebase.hpp"

namespace LibXR
{

class MSPM0Timebase : public Timebase
{
 public:
  /**
   * @brief 构造函数 / Constructor
   *
   * 配置 MSPM0 SysTick 时间基的回绕范围，并标记时间基已就绪。
   * Configures the MSPM0 SysTick wrap range and marks the timebase ready.
   */
  MSPM0Timebase();

  /**
   * @brief SysTick 中断入口辅助函数。
   *        Helper called from the SysTick interrupt path.
   */
  static void OnSysTickInterrupt();

  /**
   * @brief 同步毫秒计数器。
   *        Synchronize the millisecond counter.
   * @param ticks 新的毫秒计数值 / New millisecond tick value.
   */
  static void Sync(uint32_t ticks);

  /**
   * @brief SysTick 毫秒计数器。
   *        SysTick millisecond counter.
   */
  inline static volatile uint32_t sys_tick_ms;  // NOLINT
};

}  // namespace LibXR
