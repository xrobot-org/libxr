#include "hpm_power.hpp"

#include "hpm_ppor_drv.h"
#include "hpm_romapi.h"
#ifdef HPM_PDGO_BASE
#include "hpm_pdgo_drv.h"
#endif

using namespace LibXR;

HPMPowerManager::HPMPowerManager() {}

/**
 * @brief PPOR 软件复位整机 / Resets the whole device through the PPOR
 *
 * 做法 SDK 的 DFU 触发例程一致：使能软件复位源后按 24 个计数（约 1 微秒）触发复位。
 * The same sequence as the DFU trigger sample of the SDK: enable the software reset
 * source, then issue the reset after 24 counters (about one microsecond).
 */
void HPMPowerManager::Reset()
{
  ppor_reset_mask_set_source_enable(HPM_PPOR, ppor_reset_software);
  ppor_sw_reset(HPM_PPOR, 24);
  while (true)
  {
  }
}

/**
 * @brief 进入 power down 模式 / Enters the power down mode
 *
 * 有 PDGO 的 SoC（HPM5300 系列等）按 SDK 电源模式例程的做法设置关断计数后等待断电，
 * 由 RESETN 或 WAKEUP 引脚唤醒；没有 PDGO 的 SoC 退化为关中断后执行 WFI 等待。
 * SoCs with a PDGO (the HPM5300 series among others) set the turn-off counter as the
 * power mode sample of the SDK does and wait for the power down, woken up by the RESETN
 * or WAKEUP pin; SoCs without a PDGO fall back to disabling interrupts and executing
 * WFI.
 */
void HPMPowerManager::Shutdown()
{
#ifdef HPM_PDGO_BASE
  pdgo_set_turnoff_counter(HPM_PDGO, 0x100);
  while (true)
  {
  }
#else
  disable_global_irq(CSR_MSTATUS_MIE_MASK);
  while (true)
  {
    __asm volatile("wfi");
  }
#endif
}

/**
 * @brief 进入 ROM ISP / Enters the ROM ISP
 *
 * 关中断后调用 ROM API 进入 ISP，外设由 ROM 自动探测；正常不返回，返回时复位整机。
 * Disables the interrupts and calls the ROM API to enter the ISP, with the peripheral
 * auto-detected by the ROM; the call does not return normally, and a return resets the
 * device.
 */
void HPMPowerManager::JumpToBootloader()
{
  disable_global_irq(CSR_MSTATUS_MIE_MASK);
  api_boot_arg_t arg;
  arg.index = 0;
  arg.peripheral = API_BOOT_PERIPH_AUTO;
  arg.src = API_BOOT_SRC_ISP;
  arg.tag = API_BOOT_TAG;
  rom_enter_bootloader(&arg);
  Reset();
}
