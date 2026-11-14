#pragma once

#include "main.h"

// 设备头文件的 `USB` 宏（寄存器指针）与命名空间 `LibXR::USB` 冲突，这里取消定义。
// 控制器是否存在用 `USB_BASE`（经典型）和 `USB_DRD_FS`（DRD）判断：它们不受这个
// #undef 影响，判断结果与头文件的包含顺序无关。
// The `USB` macro of the device header (the register pointer) collides with the
// `LibXR::USB` namespace and is undefined here. Whether the controller exists is told by
// `USB_BASE` (classic) and `USB_DRD_FS` (DRD): the #undef does not touch them, so the
// answer does not depend on the order in which headers are included.
#undef USB

#include "usb/backend/fsdev.hpp"

#if defined(HAL_PCD_MODULE_ENABLED) && (defined(USB_BASE) || defined(USB_DRD_FS))

namespace LibXR
{

// 经典型 FSDEV 控制器与 USB_DRD_FS 的 PMA 大小。带 CAN 的 F1/F3 共享包内存，
// 共享比例在构建期决定，方式与旧驱动相同。未知料号请手动定义 LIBXR_STM32_USB_PMA_SIZE。
// PMA sizes of the classic FSDEV controllers and of USB_DRD_FS. F1/F3 with CAN share
// the packet memory; the share is decided at build time the same way the old driver
// did. Define LIBXR_STM32_USB_PMA_SIZE manually for an unknown part number.
#if defined(PMA_END_ADDR)
#define LIBXR_STM32_USB_PMA_SIZE PMA_END_ADDR
// --- F0: USB FS Device, PMA = 1024B (dedicated) -------------------------
#elif defined(STM32F0)
#define LIBXR_STM32_USB_PMA_SIZE 1024u

// --- F1: F102/F103 USB FS Device, PMA = 512B，与 CAN 共用，不能并行使用 ---
// --- F1: F102/F103 USB FS Device, PMA = 512B, shared with CAN, no parallel use ---
#elif defined(STM32F102xB) || defined(STM32F102xC) || defined(STM32F103x6) || \
    defined(STM32F103xB) || defined(STM32F103xE) || defined(STM32F103xG)
#define LIBXR_STM32_USB_PMA_SIZE 512u

#if defined(HAL_CAN_MODULE_ENABLED)
#error "STM32F102/F103 cannot use CAN at the same time with USB."
#endif

// --- F3 xB/xC 与 F373xC：PMA = 512B，全部给 USB，不与 CAN 共享
//     （RM0365 表 172、RM0316 表 178）
// --- F3 xB/xC and F373xC: PMA = 512B, all of it for USB, nothing shared with CAN
//     (RM0365 table 172, RM0316 table 178)
#elif defined(STM32F302xB) || defined(STM32F302xC) || defined(STM32F303xB) || \
    defined(STM32F303xC) || defined(STM32F373xC)
#define LIBXR_STM32_USB_PMA_SIZE 512u

// --- F302x6/8/D/E 与 F303xD/E：1KB，与 CAN 共享（RM0365 表 172、RM0316 表 178）
// - CAN 时钟打开后 USB 只能用前 768B，最后 256B 给 CAN；这里以是否启用 HAL 的 CAN
//   模块为准。
// --- F302x6/8/D/E and F303xD/E: 1 KB shared with CAN (RM0365 table 172, RM0316 table
//     178)
// - With the CAN clock enabled USB can only use the first 768 B and the last 256 B go
//   to CAN; the enabled HAL CAN module is taken as the sign of that.
#elif defined(STM32F302x6) || defined(STM32F302x8) || defined(STM32F302xD) || \
    defined(STM32F302xE) || defined(STM32F303xD) || defined(STM32F303xE)

#if defined(HAL_CAN_MODULE_ENABLED)
// 给 USB 768B / 768 B for USB.
#define LIBXR_STM32_USB_PMA_SIZE 768u
#else
// 整 1KB 给 USB / The whole 1 KB for USB.
#define LIBXR_STM32_USB_PMA_SIZE 1024u
#endif

// --- L0: USB FS Device, PMA = 1024B (dedicated) -------------------------
#elif defined(STM32L0)
#define LIBXR_STM32_USB_PMA_SIZE 1024u

// --- L1: USB FS Device, PMA = 512B (dedicated) --------------------------
//     上拉是 SYSCFG_PMC 的 USB_PU 位（RM0038 第 24.4.2 节）/ The pull-up is the USB_PU
//     bit of SYSCFG_PMC (RM0038 section 24.4.2)
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

/**
 * @brief STM32 FSDEV / DRD USB 设备 / STM32 FSDEV or DRD USB device
 *
 * 寄存器模型在基类 `USB::FSDevDevice` 里（usb/backend/fsdev.hpp）。本类给出控制器的
 * 地址和 PMA 的形状，接上拉，并在带 `USB_DRD_FS` 的芯片上覆盖 32 位寄存器布局的那几个
 * 函数。
 *
 * 不使用 PCD 句柄：构造时借临时句柄调用 CubeMX 的 HAL_PCD_MspInit()（时钟、引脚、
 * NVIC），之后寄存器全部由基类直接读写。CubeMX 保持默认设置：链接时包装
 * HAL_PCD_Init() 与 HAL_PCD_IRQHandler()，main() 里的 MX_USB_PCD_Init() 不再初始化
 * 控制器，生成的中断函数进入 Dispatch()。
 *
 * The register model is in the base class `USB::FSDevDevice` (usb/backend/fsdev.hpp).
 * This class gives the addresses of the controller and the shape of the PMA, connects
 * the pull-up and, on parts with `USB_DRD_FS`, overrides the functions of the 32-bit
 * register layout.
 *
 * No PCD handle is used: the constructor calls the CubeMX HAL_PCD_MspInit() (clocks,
 * pins, NVIC) through a temporary handle, and from then on the base class programs the
 * registers directly. CubeMX keeps its default settings: the link wraps HAL_PCD_Init()
 * and HAL_PCD_IRQHandler(), so MX_USB_PCD_Init() no longer initializes the controller
 * and the generated interrupt handler enters Dispatch().
 */
class STM32USBDeviceDevFs : public USB::FSDevDevice
{
 public:
  /**
   * @brief 构造设备并初始化控制器；参数同 USB::FSDevDevice 的构造函数（不含地址）/
   * Construct the device and initialize the controller; the parameters are those of the
   * USB::FSDevDevice constructor without the addresses
   * @param rx_ep_cfgs OUT 端点配置，第 i 项对应 EP i / OUT endpoint configs, entry i is
   * EP i
   * @param tx_ep_cfgs IN 端点配置，第 i 项对应 EP i / IN endpoint configs, entry i is
   * EP i
   * @param packet_size 端点 0 最大包长 / EP0 max packet size
   * @param vid 厂商 ID / Vendor ID
   * @param pid 产品 ID / Product ID
   * @param bcd 设备版本号（bcdDevice）/ Device release number (bcdDevice)
   * @param lang_list 字符串描述符语言包列表 / String descriptor language packs
   * @param configs 每个 configuration 的 class/描述符项列表 / Class and descriptor items
   * of each configuration
   * @param uid 可选的唯一标识字节，作为序列号字符串的后缀 / Optional unique ID bytes
   * used as the suffix of the serial-number string
   * @param pma_limit PMA 可用字节（构造期断言）；0 = 全部 / Usable PMA bytes (asserted
   * at construction); 0 = all of it
   * @param slot_count 每个端点的槽数量，0 为基类默认值 / Number of slots per endpoint, 0
   * selects the base-class default
   */
  STM32USBDeviceDevFs(
      const std::initializer_list<EPConfig>& rx_ep_cfgs,
      const std::initializer_list<EPConfig>& tx_ep_cfgs,
      USB::DeviceDescriptor::PacketSize0 packet_size, uint16_t vid, uint16_t pid,
      uint16_t bcd,
      const std::initializer_list<const USB::DescriptorStrings::LanguagePack*>& lang_list,
      const std::initializer_list<
          const std::initializer_list<USB::ConfigDescriptorItem*>>& configs,
      ConstRawData uid = {nullptr, 0}, size_t pma_limit = 0, uint8_t slot_count = 0);

 protected:
  /// 接通或断开 D+ 上拉：BCDR 的 DPPU、L1 的 SYSCFG_PMC，F1/F3 交给
  /// HAL_PCDEx_SetConnectionState / Connect or disconnect the D+ pull-up: DPPU of BCDR,
  /// SYSCFG_PMC on L1, HAL_PCDEx_SetConnectionState on F1/F3
  void Connect(bool on) override;

#if defined(USB_DRD_FS) || defined(__DOXYGEN__)
  // DRD（USB_DRD_FS）的寄存器布局：寄存器 32 位，低 16 位与经典型一致；BTABLE 是固定在
  // PMA 起点的 32 位 TXBD/RXBD；PMA 按 32 位字访问，字节偏移 1:1；挂起位是
  // SUSPEN/SUSPRDY。
  // Register layout of the DRD variant (USB_DRD_FS): 32-bit registers whose low half is
  // the classic layout; the BTABLE is the 32-bit TXBD/RXBD table fixed at the start of
  // the PMA; the PMA is accessed as 32-bit words with a 1:1 byte offset; the suspend
  // bits are SUSPEN/SUSPRDY.
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
};

}  // namespace LibXR

#endif
