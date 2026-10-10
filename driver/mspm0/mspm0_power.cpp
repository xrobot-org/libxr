#include "mspm0_power.hpp"

// 应用使用的 SRAM（SRAM_BANK0）的末尾，来自 SysConfig 的链接脚本；
// 启动文件也从它取初始栈指针。
// End of the SRAM the application uses (SRAM_BANK0), from the SysConfig linker
// script; the startup file takes its initial stack pointer from it too.
extern "C" uint32_t __StackTop;

using namespace LibXR;

/**
 * @brief 构造时释放 SHUTDOWN 锁存的 IO / Releases the IO latched by SHUTDOWN
 *
 * 退出 SHUTDOWN 后 IO（包括 SWD 引脚）保持进入时的状态，直到软件写 SHDNIOREL 释放
 * （SLAU846B 2.4.7）；在此之前串口不能收发，调试器也连不上。TRM 要求先重新配置 IO
 * 再释放，构造函数在 SysConfig 初始化引脚之后运行，满足这一顺序。判断用 SYSSTATUS 的
 * SHDNIOLOCK 位，不读会被清除的 RSTCAUSE。
 * After leaving SHUTDOWN, the IO (the SWD pins included) keep the state they had on
 * entry until software releases them through SHDNIOREL (SLAU846B 2.4.7); until then the
 * UART cannot transfer and a debugger cannot connect. The TRM requires the IO to be
 * reconfigured before the release; the constructor runs after SysConfig has initialized
 * the pins, which meets that order. The check uses the SHDNIOLOCK bit of SYSSTATUS
 * rather than RSTCAUSE, which is cleared on read.
 */
MSPM0PowerManager::MSPM0PowerManager()
{
  if (DL_SYSCTL_getStatus() & DL_SYSCTL_STATUS_SHUTDOWN_IO_LOCK_TRUE)
  {
    DL_SYSCTL_releaseShutdownIO();
  }
}

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
 * 进入功耗最低的 SHUTDOWN 模式，由 NRST、SWD 活动或配置了唤醒功能的 IO 唤醒；退出
 * SHUTDOWN 触发 BOR，唤醒后等同一次复位重启，锁存的 IO 由构造函数释放。MSPM0Gx51x 上
 * LFCLK_IN 引脚配成输入且上拉时，退出 SHUTDOWN 后 LFCLK 会卡住（勘误 SYSCTL_ERR_05，
 * SLAZ758E），该引脚应下拉或不配成输入。
 * Enters the SHUTDOWN mode with the lowest current consumption, woken up by NRST, SWD
 * activity or a wake-up capable IO; leaving SHUTDOWN triggers a BOR, so waking up is
 * equivalent to a reset, and the constructor releases the latched IO. On the
 * MSPM0Gx51x, an LFCLK_IN pin configured as an input with a pull-up leaves the LFCLK
 * stuck after SHUTDOWN (erratum SYSCTL_ERR_05, SLAZ758E); pull that pin down or do not
 * configure it as an input.
 */
void MSPM0PowerManager::Shutdown()
{
  DL_SYSCTL_setPowerPolicySHUTDOWN();
  // 有中断挂起时 WFI 立即返回；处理完中断再执行 WFI，直到进入 SHUTDOWN。
  // WFI returns at once while an interrupt is pending; after it is served, WFI runs
  // again until the device enters SHUTDOWN.
  while (true)
  {
    __WFI();
  }
}

/**
 * @brief 复位并进入 ROM BSL / Resets into the ROM BSL
 *
 * 照 SDK 示例 bsl_software_invoke_app_demo_uart（其中注明为规避 BSL_ERR_01）在复位前清零
 * SRAM 的数据和 ECC 码，否则 BSL 读到 ECC 不一致的 SRAM，触发 SRAMDED NMI 后停止响应。
 * 示例从 FACTORY 区的 SRAMFLASH 读 SRAM 大小，而 Flash 等待周期为 2（MCLK 高于 32 MHz）时
 * 访问 FACTORY 区会 HardFault（勘误 FLASH_ERR_01，SLAZ758E），所以这里改为清零
 * 0x20200000 至 __StackTop（SRAM_BANK0，BSL 的缓冲区在其中）以及同样大小的 ECC 码区
 * 0x20300000。清零覆盖栈，因此关中断并且只用寄存器；操作数限定为低位寄存器，因为
 * Thumb-1 的 str 只接受 r0-r7。
 * Following the SDK example bsl_software_invoke_app_demo_uart (which names it a
 * workaround for BSL_ERR_01), the SRAM data and ECC codes are cleared before the reset;
 * otherwise the BSL reads SRAM with inconsistent ECC, takes an SRAMDED NMI and stops
 * responding. The example reads the SRAM size from FACTORY SRAMFLASH, but a FACTORY
 * access with flash wait state 2 (MCLK above 32 MHz) HardFaults (erratum FLASH_ERR_01,
 * SLAZ758E), so 0x20200000 up to __StackTop (SRAM_BANK0, which holds the BSL buffer) and
 * the ECC code region of the same size at 0x20300000 are cleared instead. The clear
 * covers the stack, so interrupts are off and only registers are used; the operands are
 * low registers because the Thumb-1 str takes r0-r7 only.
 */
void MSPM0PowerManager::JumpToBootloader()
{
  __disable_irq();
  const uint32_t size = reinterpret_cast<uint32_t>(&__StackTop) - 0x20200000U;
  __asm volatile(
      ".syntax unified\n"
      "ldr     r1, =0x20300000\n"
      "adds    r2, %[size], r1\n"
      "movs    r3, #0\n"
      "1:\n"
      "str     r3, [r1]\n"
      "adds    r1, r1, #4\n"
      "cmp     r1, r2\n"
      "blo     1b\n"
      "ldr     r1, =0x20200000\n"
      "adds    r2, %[size], r1\n"
      "2:\n"
      "str     r3, [r1]\n"
      "adds    r1, r1, #4\n"
      "cmp     r1, r2\n"
      "blo     2b\n"
      "str     %[lvl_val], [%[lvl_addr]]\n"
      "str     %[cmd_val], [%[cmd_addr]]\n"
      :
      : [size] "l"(size), [lvl_addr] "l"(&SYSCTL->SOCLOCK.RESETLEVEL),
        [lvl_val] "l"(DL_SYSCTL_RESET_BOOTLOADER_ENTRY),
        [cmd_addr] "l"(&SYSCTL->SOCLOCK.RESETCMD),
        [cmd_val] "l"(SYSCTL_RESETCMD_KEY_VALUE | SYSCTL_RESETCMD_GO_TRUE)
      : "r1", "r2", "r3", "memory");
  while (true)
  {
  }
}
