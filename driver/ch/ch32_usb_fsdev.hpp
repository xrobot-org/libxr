#pragma once

#include "libxr_def.hpp"
#include DEF2STR(LIBXR_CH32_CONFIG_FILE)

#include "usb/backend/fsdev.hpp"

#if defined(RCC_APB1Periph_USB) || defined(__DOXYGEN__)

namespace LibXR
{

/**
 * @class CH32USBDeviceFS
 * @brief CH32 的 USBD（PMA 型全速设备控制器）/ CH32 USBD (PMA-based full-speed device
 *        controller)
 *
 * 寄存器模型在 USB::FSDevDevice：USBD 的寄存器偏移、位和 PMA 的访问方式与它的经典型
 * 相同（CH32FV2x_V3xRM 21.3，p.326-337）。本类给出控制器地址与 PMA 形状，打开时钟、
 * 配置引脚、接内部上拉，并提供设备服务的屏蔽、开线和挂起。
 * The register model is USB::FSDevDevice: the register offsets, the bits and the PMA
 * access of USBD are those of its classic variant (CH32FV2x_V3xRM 21.3, p.326-337). This
 * class gives the controller addresses and the PMA shape, enables the clocks, sets up
 * the pins, connects the internal pull-up, and supplies the device service's mask,
 * unmask and pend.
 *
 * @note 中断线：只用低优先级线 USB_LP_CAN1_RX0。高优先级线只报等时和双缓冲批量端点的
 *       完成（手册 p.321），本后端两者都不用，所以不打开它，也不从它进入；唤醒线只在
 *       内核停止时钟时才需要，不打开。低优先级线与 CAN1 的 RX0 共用向量，入口在
 *       ch32_usbcan_shared.cpp。
 *       Interrupt lines: only the low-priority line USB_LP_CAN1_RX0 is used. The
 *       high-priority line reports completions of isochronous and double-buffered bulk
 *       endpoints only (manual p.321); this backend uses neither, so the line is not
 *       enabled and is no entry. The wake-up line is needed only when the core stops
 *       its clocks and is not enabled. The low-priority line shares its vector with
 *       RX0 of CAN1; the entry is in ch32_usbcan_shared.cpp.
 * @note PMA 与 CAN 共用 512 字节：CAN1 占高 128 字节，CAN1 和 CAN2 都用时占高 256 字节
 *       （手册 p.322）。可用字节数在构造时按已构造的 CAN 对象确定，之后不能再构造 CAN
 *       对象（CH32CAN 的构造函数断言）。
 *       The PMA shares its 512 bytes with CAN: CAN1 takes the upper 128 bytes, CAN1 and
 *       CAN2 together the upper 256 (manual p.322). The usable size is fixed at
 *       construction from the CAN objects that exist; no CAN object may be constructed
 *       afterwards (the constructor of CH32CAN asserts it).
 * @note 与手册初始化步骤（p.321 21.2.2 2)）的差别：手册在撤销强制复位之前写描述表、接
 *       上拉；这里先撤销强制复位、清标志、写描述表，中断源和上拉留到 StartController
 *       最后才开。这样主机的总线复位总是落在中断已打开的控制器上。
 *       Difference from the manual's initialization (p.321 21.2.2 2)): the manual writes
 *       the description table and connects the pull-up before releasing the forced
 *       reset; here the forced reset is released, the flags are cleared and the table
 *       written first, and the interrupt sources and the pull-up come on last in
 *       StartController. The host's bus reset then always meets a controller whose
 *       interrupts are enabled.
 */
class CH32USBDeviceFS : public USB::FSDevDevice
{
 public:
  /**
   * @brief 构造并初始化控制器 / Construct and initialize the controller
   * @param rx_ep_cfgs OUT 端点，第 i 项是 EP i，EP0 必须有 / OUT endpoints, entry i is
   *        EP i, EP0 mandatory
   * @param tx_ep_cfgs IN 端点，规则同上 / IN endpoints, same rules
   * @param packet_size EP0 包长 / EP0 packet size
   * @param vid 厂商 ID / Vendor ID
   * @param pid 产品 ID / Product ID
   * @param bcd bcdDevice
   * @param languages 字符串语言包 / String language packs
   * @param configs 每个配置的类列表 / Class list of each configuration
   * @param uid 序列号字符串后缀，可为空 / Serial-number suffix, may be empty
   * @param pma_limit PMA 可用字节，0 为 CAN 留下的全部；不得超过它 / Usable PMA bytes,
   *        0 for all that CAN leaves; must not exceed it
   * @param slot_count 每个端点的槽数，0 为核心默认 / Slots per endpoint, 0 for default
   * @note 打开 48 MHz 和 USBD 的时钟，PA11、PA12 先输出低再改为浮空输入，选全速，然后
   *       复位控制器。中断源和上拉在 Start 时才打开。
   *       Enables the 48 MHz clock and the clock of USBD, drives PA11 and PA12 low and
   *       then makes them floating inputs, selects full speed and resets the
   *       controller. The interrupt sources and the pull-up come on at Start.
   */
  CH32USBDeviceFS(
      const std::initializer_list<EPConfig>& rx_ep_cfgs,
      const std::initializer_list<EPConfig>& tx_ep_cfgs,
      USB::DeviceDescriptor::PacketSize0 packet_size, uint16_t vid, uint16_t pid,
      uint16_t bcd,
      const std::initializer_list<const USB::DescriptorStrings::LanguagePack*>& languages,
      const std::initializer_list<const std::initializer_list<USB::DeviceClass*>>&
          configs,
      ConstRawData uid = {nullptr, 0}, size_t pma_limit = 0, uint8_t slot_count = 0);

 protected:
  /// 内部 1.5 kΩ 上拉：EXTEN_CTR 的 USBD_PU_EN / The internal 1.5 kOhm pull-up:
  /// USBD_PU_EN of EXTEN_CTR
  void Connect(bool on) override;
  /// NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn)
  void MaskInterrupt() override;
  /// NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn)
  void UnmaskInterrupt() override;
  /// NVIC_SetPendingIRQ(USB_LP_CAN1_RX0_IRQn)，返回 true / NVIC_SetPendingIRQ, true
  bool PendInterrupt() override;
};

}  // namespace LibXR

#endif  // defined(RCC_APB1Periph_USB) || defined(__DOXYGEN__)
