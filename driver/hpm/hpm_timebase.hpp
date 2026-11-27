#pragma once

#include "hpm_clock_drv.h"
#include "hpm_mchtmr_drv.h"
#include "hpm_soc.h"
#include "timebase.hpp"

namespace LibXR
{

/**
 * @class HPMTimebase
 * @brief HPM MCHTMR 时间基准实现 / HPM MCHTMR based timebase implementation.
 *
 * 基于 MCHTMR 计数器提供微秒与毫秒时间戳。
 * Provides microsecond and millisecond timestamps using MCHTMR counter.
 *
 * @note 裸机（system/none）时 DriverAsyncCondition 的节拍由板级工程给：启动时调用一次
 *       DriverAsyncCondition::RegisterTickSource()，之后每毫秒调用一次
 *       DriverAsyncCondition::OnTick()（间隔固定，elapsed_ms 不变）。本类不提供节拍。
 *       FreeRTOS、ThreadX 下节拍来自系统层，板级工程不调用。
 * @note Bare-metal (system/none), the board project provides DriverAsyncCondition's tick:
 *       it calls DriverAsyncCondition::RegisterTickSource() once at start and then
 *       DriverAsyncCondition::OnTick() every millisecond (a fixed interval, elapsed_ms
 *       unchanged). This class provides no tick. Under FreeRTOS or ThreadX the tick
 *       comes from the system layer and the board project does not call it.
 */
class HPMTimebase : public Timebase
{
 public:
  /**
   * @brief 构造 HPM 时间基对象 / Construct an HPM timebase object.
   * @param timer MCHTMR 外设基地址 / MCHTMR base address.
   * @param clock 定时器时钟源 / Clock source used by the timer.
   *
   * 构造时选择计数器实例并缓存输入时钟频率。
   * Construction selects the counter instance and caches the input clock rate.
   */
  HPMTimebase(MCHTMR_Type* timer = HPM_MCHTMR, clock_name_t clock = clock_mchtmr0);
};

}  // namespace LibXR
