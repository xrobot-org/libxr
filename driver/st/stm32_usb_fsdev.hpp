#pragma once

#include "main.h"

/*
 * 设备头文件的 USB 宏（寄存器指针）与命名空间 LibXR::USB 冲突，这里取消定义。控制器是否
 * 存在用 USB_BASE（经典型）和 USB_DRD_FS（DRD）判断，不受影响。
 * The device header's USB macro (the register pointer) collides with the LibXR::USB
 * namespace and is undefined here. Whether the controller exists is told by USB_BASE
 * (classic) and USB_DRD_FS (DRD), which are not affected.
 */
#undef USB

#include "usb/backend/fsdev.hpp"

#if defined(HAL_PCD_MODULE_ENABLED) && (defined(USB_BASE) || defined(USB_DRD_FS))

/*
 * PMA 的字节数：设备头文件给出 PMA_END_ADDR 时用它，否则按系列取；应用可以预先定义
 * LIBXR_STM32_USB_PMA_SIZE 覆盖。
 * Size of the PMA in bytes: PMA_END_ADDR when the device header gives it, otherwise
 * by family; an application may define LIBXR_STM32_USB_PMA_SIZE beforehand to
 * override it.
 */
#if !defined(LIBXR_STM32_USB_PMA_SIZE)
#if defined(PMA_END_ADDR)
#define LIBXR_STM32_USB_PMA_SIZE PMA_END_ADDR
// --- F0: USB FS Device, PMA = 1024B (dedicated) -------------------------
#elif defined(STM32F0)
#define LIBXR_STM32_USB_PMA_SIZE 1024u

// --- F1: F102/F103 USB FS Device, PMA = 512B，与 CAN 共用，不能并行使用 ---
#elif defined(STM32F102xB) || defined(STM32F102xC) || defined(STM32F103x6) || \
    defined(STM32F103xB) || defined(STM32F103xE) || defined(STM32F103xG)
#define LIBXR_STM32_USB_PMA_SIZE 512u

#if defined(HAL_CAN_MODULE_ENABLED)
#error "STM32F102/F103 cannot use CAN at the same time with USB."
#endif

// --- F3 小容量带 USB ：PMA = 512B，专用给 USB --------------------
#elif defined(STM32F302xB) || defined(STM32F302xC) || defined(STM32F303xB) || \
    defined(STM32F303xC) || defined(STM32F373xC)
#define LIBXR_STM32_USB_PMA_SIZE 512u

// --- F3 大容量（和 CAN 共享 1KB）的几种：F302x6/8,D/E & F303xD/E -------
// - 总共 1KB USB+CAN SRAM
// - 若 CAN 在用：USB 只能用前 768B，最后 256B 给 CAN
#elif defined(STM32F302x6) || defined(STM32F302x8) || defined(STM32F302xD) || \
    defined(STM32F302xE) || defined(STM32F303xD) || defined(STM32F303xE)

#if defined(HAL_CAN_MODULE_ENABLED)
// 给 USB 768B
#define LIBXR_STM32_USB_PMA_SIZE 768u
#else
// 整 1KB 给 USB
#define LIBXR_STM32_USB_PMA_SIZE 1024u
#endif

// --- L0: USB FS Device, PMA = 1024B (dedicated) -------------------------
#elif defined(STM32L0)
#define LIBXR_STM32_USB_PMA_SIZE 1024u

// --- L1: USB FS Device, PMA = 512B (dedicated) --------------------------
#elif defined(STM32L1)
#define LIBXR_STM32_USB_PMA_SIZE 512u

// --- G4: USB FS Device (V1), PMA = 1024B (dedicated) --------------------
#elif defined(STM32G4)
#define LIBXR_STM32_USB_PMA_SIZE 1024u

// --- G0: USB_DRD_FS (V2), PMA = 2048B (dedicated) -----------------------
#elif defined(STM32G0)
#define LIBXR_STM32_USB_PMA_SIZE 2048u

// --- C0: USB_DRD_FS (V2), PMA = 2048B (dedicated) -----------------------
#elif defined(STM32C0)
#define LIBXR_STM32_USB_PMA_SIZE 2048u

// --- H5（H503/563/573/562 DRD_FS），PMA = 2048B -------------------------
#elif defined(STM32H503xx) || defined(STM32H563xx) || defined(STM32H573xx) || \
    defined(STM32H562xx)
#define LIBXR_STM32_USB_PMA_SIZE 2048u

// --- WB55/35: USB FS Device, PMA = 1024B (专门给 USB) ------------------
#elif defined(STM32WB)
#define LIBXR_STM32_USB_PMA_SIZE 1024u

// --- L4: USB FS Device (非 OTG)，PMA = 1024B ----------------------------
#elif defined(STM32L4)
#define LIBXR_STM32_USB_PMA_SIZE 1024u

// --- L5: USB FS Device/DRD，PMA = 1024B -------------------------------
#elif defined(STM32L5)
#define LIBXR_STM32_USB_PMA_SIZE 1024u

// --- U5: USB_DRD_FS (V2)，PMA = 2048B -----------------------------------
#elif defined(STM32U5)
#define LIBXR_STM32_USB_PMA_SIZE 2048u

// --- U0: USB_DRD_FS (V2，1KB 专用) --------------------------------------
#elif defined(STM32U0)
#define LIBXR_STM32_USB_PMA_SIZE 1024u

// --- U3: USB_DRD_FS (V2，2KB 专用) --------------------------------------
#elif defined(STM32U3)
#define LIBXR_STM32_USB_PMA_SIZE 2048u

#else
#error \
    "Unknown STM32 USB FS/DRD with PMA. Please define LIBXR_STM32_USB_PMA_SIZE manually."
#endif
#endif

/*
 * 断开时把 D+（PA12）当 GPIO 推挽输出低，让带固定外部 D+ 上拉的板子也真正断开（见
 * STM32USBDeviceDevFs 的说明）。默认 1；PA12 不能被驱动的板子定义为 0。
 * While detached, D+ (PA12) is driven low as a GPIO push-pull output so that boards with
 * a fixed external D+ pull-up detach as well (see the STM32USBDeviceDevFs notes). Default
 * 1; a board whose PA12 must not be driven defines it as 0.
 */
#if !defined(LIBXR_STM32_FSDEV_DP_DETACH)
#define LIBXR_STM32_FSDEV_DP_DETACH 1
#endif

namespace LibXR
{

/**
 * @class STM32USBDeviceDevFs
 * @brief STM32 FSDEV / DRD 设备 / STM32 FSDEV or DRD device
 *
 * 寄存器模型在 USB::FSDevDevice。本类给出控制器地址与 PMA 形状、接上拉，在 DRD 上
 * 覆盖 32 位寄存器布局的函数，并提供设备服务的屏蔽、开线和挂起：对控制器处理函数所挂的
 * NVIC 线逐条 NVIC_DisableIRQ / NVIC_EnableIRQ，挂起置第一条线（LP）。不关全部中断；
 * 屏蔽、开线和挂起不写 CNTR。
 * The register model is USB::FSDevDevice. This class gives the controller addresses
 * and the PMA shape, connects the pull-up, overrides the 32-bit register layout on DRD,
 * and supplies the device service's mask, unmask and pend: NVIC_DisableIRQ /
 * NVIC_EnableIRQ on each NVIC line the controller handler hangs on, and pend sets the
 * first line (LP). Never disables all interrupts; mask, unmask and pend never write
 * CNTR.
 *
 * @note 断开时驱动 D+（LIBXR_STM32_FSDEV_DP_DETACH 为 1，默认）。有的板子 D+ 上焊了固定的
 *       1.5 kΩ 上拉（F103 的 Bluepill 一类；本库测过的 STM32H503 测试板也是：软件关掉
 *       DPPU 50 ms，FNR.RXDP 仍为 1、帧号照走），只关内部上拉断不开，设备复位之后主机还
 *       留着旧地址。于是构造、Stop、ResetController 的每次断开都按下面的顺序做：
 *       Connect(false) → CNTR = FRES|PDWN → 关 USB 外设时钟（HAL 的 RCC 宏；寄存器和 PMA
 *       保留，不复位 RCC）→ 保存 PA12 的 GPIO 配置 → ODR12 = 0（BSRR 的复位半边）、推挽
 *       输出 → 返回，DetachTimer 按节拍计 DETACH_MS（5 ms），不等待 →（到点后 owner 的
 *       Dispatch，或时间已过时 StartController 就地：AttachAgain）恢复 PA12 的配置 → 开
 *       USB 时钟 → CNTR = FRES 再 0（InitController）→ 打开中断源 → Connect(true)。
 *       - 引脚：所有支持的系列、所有带这个 USB 的封装，D+ 都是 GPIOA 的第 12 位（ST 的
 *         STM32_open_pin_data，887 个型号）。F04x/F070x6 小封装（PA11_PA12_RMP）和
 *         G0B/C071/U073 的共用焊盘（PA12_RMP）把 PA12 换到别的焊盘时，GPIOA 第 12 位跟着
 *         换过去，仍然驱动 D+ 焊盘。D-（PA11）不动。
 *       - 与手册的出入（实测，手册没写）：各系列手册只说 USB 引脚在 USB 使能时自动接到收
 *         发器（RM0008 表 29），或作为“附加功能”优先于 GPIO（各 RM 的 GPIO 章），没有说
 *         USB 断电并关时钟时 GPIO 输出能接管焊盘。这里只在断电、关时钟之后驱动。实测在
 *         F103（PA12 是附加功能）和 H503（PA12 为模拟模式）上：拉低期间 IDR12 读 0，主
 *         机记到断开并重新枚举，关掉驱动的对照镜像主机什么也没看到。PA12 设为 AF10 的系
 *         列没有测过。
 *       - 拉低时内部上拉已关、收发器已断电，引脚只对外部上拉灌电流（1.5 kΩ 约 2.2 mA）。
 *       - 关时钟期间不访问控制器的寄存器：Dispatch 只查 TakeAttach，不进；
 *         ResetController 只做前半段就返回，后半段在之后的 owner 一轮里；Stop 之后到
 *         Start 之前控制器保持断电、关时钟；拉住时 owner 不运行，不再复位或停止，D+ 一
 *         直拉住。
 *       - 关时钟期间控制器的中断请求停在关时钟那一刻，可能一直有效（实测 F103：中断线
 *         反复进入，节拍中断得不到运行，第 4 次强制复位起就不再接上；屏蔽后 70/70 次复
 *         位、30/30 次停止再启动都接上）。Dispatch 因此在拉住期间屏蔽中断线，到点时节拍
 *         打开并挂起（FSDevDevice::WakeHeld）；这是 DeviceService::MaskInterrupt 写明的
 *         例外。
 *       - 找不到本系列 USB 时钟的 HAL 宏时不关时钟，只置 PDWN。
 * @note D+ is driven while detached (LIBXR_STM32_FSDEV_DP_DETACH 1, the default). Some
 *       boards have a fixed 1.5 kOhm pull-up on D+ (F103 boards of the Bluepill kind;
 *       so does the STM32H503 test board this library was measured on: with DPPU
 *       switched off for 50 ms, FNR.RXDP stayed 1 and the frame number went on), so
 *       switching the internal pull-up off does not detach, and after a device reset the
 *       host keeps the old address. Every detach (construction, Stop, ResetController)
 *       therefore runs: Connect(false) -> CNTR = FRES|PDWN -> USB peripheral clock off
 *       (HAL RCC macro; registers and PMA are kept, no RCC reset) -> save PA12's GPIO
 *       configuration -> ODR12 = 0 (reset half of BSRR) and push-pull output ->
 *       return; DetachTimer counts DETACH_MS (5 ms) on the tick, no wait -> (once it has
 *       passed, Dispatch in the owner, or StartController in place when it already has:
 *       AttachAgain) restore PA12's configuration -> USB clock on -> CNTR = FRES
 *       then 0 (InitController) -> enable the interrupt sources -> Connect(true).
 *       - Pin: on every supported family and every package with this USB, D+ is bit 12
 *         of GPIOA (ST's STM32_open_pin_data, 887 parts). Where the small F04x/F070x6
 *         packages (PA11_PA12_RMP) or the shared pads of G0B/C071/U073 (PA12_RMP) move
 *         PA12 onto another pad, bit 12 of GPIOA moves with it and still drives the D+
 *         pad. D- (PA11) is left alone.
 *       - Deviation from the manuals (measured, not written there): the manuals only say
 *         that the USB pins connect to the transceiver as soon as the USB is enabled
 *         (RM0008 table 29) or, as "additional functions", take priority over the GPIO
 *         (the GPIO chapters); none says that a GPIO output takes the pad over while the
 *         USB is powered down with its clock off. The drive happens only after power-down
 *         and clock off. Measured on an F103 (PA12 an additional function) and an H503
 *         (PA12 in analog mode): IDR12 reads 0 while driven, the host logs a disconnect
 *         and enumerates again, and a control image with the drive off is not seen by
 *         the host. Families with PA12 on AF10 are not measured.
 *       - While low, the internal pull-up is off and the transceiver powered down, so the
 *         pin only sinks the external pull-up (about 2.2 mA through 1.5 kOhm).
 *       - With the clock off the controller's registers are not accessed: Dispatch only
 *         asks TakeAttach and does not enter; ResetController does the first half and
 *         returns, the second half follows in a later owner pass; between Stop and Start
 *         the controller stays powered down with its clock off; while D+ is held the
 *         owner does not run, so no further reset or stop happens and D+ stays held.
 *       - With the clock off the controller's interrupt request stays as it was when the
 *         clock went off and may stay active (measured on an F103: the line kept
 *         entering, the tick interrupt never ran and from the 4th forced reset on the
 *         device did not attach again; with the mask 70/70 resets and 30/30
 *         stop-then-start cycles attached again). Dispatch therefore masks the line while
 *         held, and the tick opens and pends it when the time has passed
 *         (FSDevDevice::WakeHeld); this is the exception stated at
 *         DeviceService::MaskInterrupt.
 *       - Where no HAL macro for the family's USB clock is found, the clock stays on and
 *         only PDWN is set.
 *
 * @note 线的选择：F1 为 LP/HP（与 CAN 共用 20/19）；F3 按 SYSCFG_CFGR1.USB_IT_RMP 选
 *       74/75 或 19/20（构造时读，在 HAL_PCD_MspInit 之后）；其他系列取设备头的线名，
 *       未逐个核对；可用 LIBXR_STM32_FSDEV_IRQS 覆盖。唤醒线不在内（是否进 Dispatch
 *       未核对）。
 *       Line choice: LP/HP on F1 (shared with CAN as 20/19); on F3 by
 *       SYSCFG_CFGR1.USB_IT_RMP, 74/75 or 19/20 (read at construction, after
 *       HAL_PCD_MspInit); other families use the device header's line names, not
 *       checked one by one; LIBXR_STM32_FSDEV_IRQS overrides. The wake-up line is not
 *       included (whether it enters Dispatch is unchecked).
 * @note 上拉（F1/F3，没有内部上拉）：Connect 调用用户的 HAL_PCDEx_SetConnectionState，
 *       传的是 HAL_PCD_DevConnect / DevDisconnect 也会传的那个句柄，即 CubeMX 交给
 *       HAL_PCD_Init 的句柄（STM32USBInitHandle）：Instance 和 Init 有效；HAL 自己的
 *       初始化没有运行，其余字段保持调用前的值。工程没有调用 HAL_PCD_Init 时，构造时
 *       分配一个句柄，只填 Instance。Connect 不构造、不清零句柄（在 F103 上清零一个句柄
 *       要 122.8 µs）。
 *       Pull-up (F1/F3, no internal pull-up): Connect calls the user's
 *       HAL_PCDEx_SetConnectionState with the handle HAL_PCD_DevConnect / DevDisconnect
 *       would pass, i.e. the one CubeMX passed to HAL_PCD_Init (STM32USBInitHandle):
 *       Instance and Init are valid; HAL's own initialization did not run and the other
 *       fields keep their values from before the call. A project that never calls
 *       HAL_PCD_Init gets a handle allocated at construction with only Instance set.
 *       Connect builds and zeroes no handle (zeroing one takes 122.8 us on an F103).
 */
class STM32USBDeviceDevFs : public USB::FSDevDevice
{
 public:
  /// 构造并初始化控制器；参数见 FSDevDevice / Construct and initialize the controller;
  /// parameters as for FSDevDevice
  STM32USBDeviceDevFs(
      const std::initializer_list<EPConfig>& rx_ep_cfgs,
      const std::initializer_list<EPConfig>& tx_ep_cfgs,
      USB::DeviceDescriptor::PacketSize0 packet_size, uint16_t vid, uint16_t pid,
      uint16_t bcd,
      const std::initializer_list<const USB::DescriptorStrings::LanguagePack*>& languages,
      const std::initializer_list<const std::initializer_list<USB::DeviceClass*>>&
          configs,
      ConstRawData uid = {nullptr, 0}, size_t pma_limit = 0, uint8_t slot_count = 0);

 protected:
  void Connect(bool on) override;
  /// 逐条 NVIC_DisableIRQ（CMSIS 自带 DSB、ISB）/ NVIC_DisableIRQ per line (CMSIS adds
  /// DSB and ISB)
  void MaskInterrupt() override;
  /// 逐条 NVIC_EnableIRQ / NVIC_EnableIRQ per line
  void UnmaskInterrupt() override;
  /// 置 LP 线挂起，返回 true / Set the LP line pending, return true
  bool PendInterrupt() override;

#if defined(USB_DRD_FS) || defined(__DOXYGEN__)
  uint32_t ReadRegister(uint32_t offset) override;
  void WriteRegister(uint32_t offset, uint32_t value) override;
  void SetTxBuf(uint8_t num, uint16_t addr) override;
  void SetTxCount(uint8_t num, uint16_t bytes) override;
  void SetRxBuf(uint8_t num, uint16_t addr) override;
  void SetRxCountEncoded(uint8_t num, uint16_t mps) override;
  uint16_t GetRxCount(uint8_t num) override;
  void SetIsoTxCount(uint8_t num, uint8_t buffer, uint16_t bytes) override;
  void SetIsoRxSize(uint8_t num, uint8_t buffer, uint16_t mps) override;
  uint16_t GetIsoRxCount(uint8_t num, uint8_t buffer) override;
  void PmaWrite(uint16_t addr, const uint8_t* src, size_t len) override;
  void PmaRead(uint8_t* dst, uint16_t addr, size_t len) override;
  void InitRegs() override;
  void SuspendRegs() override;
  void SuspendClocked() override;
  void ResumeRegs() override;
#endif
  /// 断开期间拉低 D+，见类说明 / Hold D+ low while detached, see the class notes
  bool HoldDetached() override;
  /// 放开 D+、恢复时钟 / Release D+ and restore the clock
  void ReleaseDetached() override;

 private:
  /// 控制器处理函数所挂的 NVIC 线；构造时写 / NVIC lines of the controller handler;
  /// written at construction
  IRQn_Type lines_[2] = {};
  /// 有效线数；构造时写 / Lines in use; written at construction
  uint8_t line_count_ = 0;
  /// HoldDetached 保存的 PA12 配置，ReleaseDetached 恢复 / PA12's configuration saved
  /// by HoldDetached, restored by ReleaseDetached
  uint32_t dp_saved_[5] = {};
  /// F1/F3：HAL_PCDEx_SetConnectionState 的句柄，构造时定，见类说明 / F1/F3: handle for
  /// HAL_PCDEx_SetConnectionState, set at construction, see the class notes
  PCD_HandleTypeDef* connection_handle_ = nullptr;
};

}  // namespace LibXR

#endif
