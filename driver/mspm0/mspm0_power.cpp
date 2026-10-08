#include "mspm0_power.hpp"

using namespace LibXR;

MSPM0PowerManager::MSPM0PowerManager() {}

/**
 * @brief 复位整机 / Resets the whole device
 *
 * `DL_SYSCTL_RESET_SYSRST` 按头文件注释只复位 CPU 和外设（"CPU plus peripherals
 * only"），因此选用 `DL_SYSCTL_RESET_BOOT`：它触发 boot 配置流程，复位绝大部分核内逻辑
 * 并给 SRAM 重新上电，效果等同整机重新从 Flash 启动。
 * Per the DriverLib comment, `DL_SYSCTL_RESET_SYSRST` resets the CPU plus the
 * peripherals only. `DL_SYSCTL_RESET_BOOT` is used instead: it triggers the device boot
 * configuration routine, resets the majority of the core logic and power-cycles the
 * SRAM, so the device restarts from Flash like a fresh power-up.
 */
void MSPM0PowerManager::Reset() { DL_SYSCTL_resetDevice(DL_SYSCTL_RESET_BOOT); }

/**
 * @brief 进入 SHUTDOWN 低功耗模式 / Enters the SHUTDOWN low-power mode
 *
 * 进入功耗最低的 SHUTDOWN 模式，由 NRST 或配置了唤醒功能的 IO 唤醒；退出 SHUTDOWN 触发
 * BOR，唤醒后等同一次复位重启。
 * Enters the SHUTDOWN mode with the lowest current consumption, woken up by NRST or a
 * wake-up capable IO; leaving SHUTDOWN triggers a BOR, so waking up is equivalent to a
 * reset.
 */
void MSPM0PowerManager::Shutdown()
{
  DL_SYSCTL_setPowerPolicySHUTDOWN();
  __WFI();
  while (true)
  {
  }
}

/**
 * @brief 复位并进入 ROM BSL / Resets into the ROM BSL
 */
void MSPM0PowerManager::JumpToBootloader()
{
  DL_SYSCTL_resetDevice(DL_SYSCTL_RESET_BOOTLOADER_ENTRY);
}
