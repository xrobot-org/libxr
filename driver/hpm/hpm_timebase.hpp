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
 * @note 毫秒节拍（DriverAsyncCondition）：使用 FreeRTOS 时机器定时器比较中断归 FreeRTOS
 *       移植层，节拍来自 vApplicationTickHook（configUSE_TICK_HOOK 须为 1）；不使用 RTOS
 *       时本类在构造时打开 MTIMECMP 比较中断作为 1 kHz 节拍并定义 mchtmr_isr()，定时器
 *       时钟须为 1 kHz 的整数倍。
 *       Millisecond tick (DriverAsyncCondition): with FreeRTOS the machine timer compare
 *       interrupt belongs to the FreeRTOS port and the tick comes from
 *       vApplicationTickHook (configUSE_TICK_HOOK must be 1); without an RTOS this class
 *       enables the MTIMECMP compare interrupt as the 1 kHz tick at construction and
 *       defines mchtmr_isr(); the timer clock must be a multiple of 1 kHz.
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
