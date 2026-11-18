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

namespace LibXR
{

/**
 * @class STM32USBDeviceDevFs
 * @brief STM32 FSDEV / DRD 设备 / STM32 FSDEV or DRD device
 *
 * 寄存器模型在 USB::FSDevDevice。本类给出控制器地址与 PMA 形状、接上拉，在 DRD 上
 * 覆盖 32 位寄存器布局的函数，并提供设备服务的屏蔽、开线和挂起：对控制器处理函数所挂的
 * NVIC 线逐条 NVIC_DisableIRQ / NVIC_EnableIRQ，挂起置第一条线（LP）。不关全部中断，
 * 不写 CNTR。
 * The register model is USB::FSDevDevice. This class gives the controller addresses
 * and the PMA shape, connects the pull-up, overrides the 32-bit register layout on DRD,
 * and supplies the device service's mask, unmask and pend: NVIC_DisableIRQ /
 * NVIC_EnableIRQ on each NVIC line the controller handler hangs on, and pend sets the
 * first line (LP). Never disables all interrupts and never writes CNTR.
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
  void PmaWrite(uint16_t addr, const uint8_t* src, size_t len) override;
  void PmaRead(uint8_t* dst, uint16_t addr, size_t len) override;
  void InitRegs() override;
  void SuspendRegs() override;
  void SuspendClocked() override;
  void ResumeRegs() override;
#endif

 private:
  /// 控制器处理函数所挂的 NVIC 线；构造时写 / NVIC lines of the controller handler;
  /// written at construction
  IRQn_Type lines_[2] = {};
  /// 有效线数；构造时写 / Lines in use; written at construction
  uint8_t line_count_ = 0;
};

}  // namespace LibXR

#endif
