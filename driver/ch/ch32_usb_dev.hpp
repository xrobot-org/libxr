#pragma once

#include "ch32_usb.hpp"
#include "ch32_usb_endpoint.hpp"
#include "ch32_usbcan_shared.hpp"
#include "libxr_def.hpp"
#include "libxr_type.hpp"
#include "usb/backend/fsdev.hpp"
#include "usb/core/ep_pool.hpp"
#include "usb/device/dev_core.hpp"

namespace LibXR
{

#if defined(RCC_APB1Periph_USB) || defined(__DOXYGEN__)

/**
 * @brief CH32 FSDEV 设备驱动 / CH32 FSDEV device driver
 *
 * 寄存器模型在基类 `USB::FSDevDevice` 里（usb/backend/fsdev.hpp），与 STM32 经典型
 * FSDEV 共用：16 位寄存器，PMA 里每个 16 位数据字占两个 16 位地址。本类给出控制器的
 * 地址、打开时钟、用 EXTEN_USBD_PU_EN 接上拉，中断经 `ch32_usbcan_shared` 注册到
 * USB/CAN 共享向量。
 *
 * PMA 布局构造期固定，之后不能再增加 CAN 的占用；PMA 可用量取构造时刻的
 * `usb_pma_limit_bytes()`。
 *
 * The register model is in the base class `USB::FSDevDevice` (usb/backend/fsdev.hpp)
 * and is shared with the STM32 classic FSDEV: 16-bit registers, and every 16-bit data
 * word of the PMA takes two 16-bit addresses. This class gives the addresses of the
 * controller, enables its clock and connects the pull-up through EXTEN_USBD_PU_EN; the
 * interrupt is registered through `ch32_usbcan_shared` on the shared USB/CAN vectors.
 *
 * The PMA layout is fixed at construction and CAN usage cannot grow afterwards; the
 * usable PMA is `usb_pma_limit_bytes()` at construction time.
 */
class CH32USBDeviceFS : public USB::FSDevDevice
{
 public:
  /**
   * @brief 构造 FSDEV 设备对象并初始化控制器 / Construct the FSDEV device object and
   * initialize the controller
   */
  CH32USBDeviceFS(
      const std::initializer_list<EPConfig>& rx_ep_cfgs,
      const std::initializer_list<EPConfig>& tx_ep_cfgs,
      USB::DeviceDescriptor::PacketSize0 packet_size, uint16_t vid, uint16_t pid,
      uint16_t bcd,
      const std::initializer_list<const USB::DescriptorStrings::LanguagePack*> LANG_LIST,
      const std::initializer_list<const std::initializer_list<USB::ConfigDescriptorItem*>>
          CONFIGS,
      ConstRawData uid = {nullptr, 0});

  /// 注册共享 USB/CAN 中断并使能向量，再启动 FSDEV 运行期 / Register the shared USB/CAN
  /// interrupt and enable the vectors, then start the FSDEV runtime
  void Start(bool in_isr) override;

  /// 先停止 FSDEV 运行期，再注销共享中断；CAN1 仍在使用时保留其向量 / Stop the FSDEV
  /// runtime, then unregister the shared interrupt; the CAN1 vectors stay enabled while
  /// CAN1 is active
  void Stop(bool in_isr) override;

 protected:
  /// 用 EXTEN_USBD_PU_EN 接通或断开 D+ 上拉 / Connect or disconnect the D+ pull-up
  /// through EXTEN_USBD_PU_EN
  void Connect(bool on) override;
};

#endif  // defined(RCC_APB1Periph_USB) || defined(__DOXYGEN__)

#if defined(USBFSD) || defined(__DOXYGEN__)

/**
 * @brief CH32 OTG FS 设备驱动 / CH32 OTG FS device driver
 */
class CH32USBOtgFS : public USB::EndpointPool,
                     public USB::DeviceCore<USB::FullSpeedCapabilities>
{
 public:
  /**
   * @brief 构造 OTG FS 设备对象 / Construct OTG FS device object
   *
   * RX/TX 列表按下标对应端点号（EP0 在首位）：端点号同时出现在两个列表 = 双向
   * （硬件共享模式，IN 缓冲必须落在 OUT 缓冲 +64 处且两块各 ≥64 B，构造期
   * REQUIRE）；只出现在一个列表 = 单向（两个槽各占块的一半，块 ≥2×MPS）。
   * EP0 的 RX[0] 与 TX[0] 必须是同一块（≥64 B，收发共用）。缺席方向传
   * {nullptr, 0}。
   * RX/TX lists are indexed by endpoint number (EP0 first): a number present in
   * both lists is bidirectional (hardware shared mode, the IN buffer must sit at
   * OUT+64 with both blocks >=64 B, REQUIRED at construction); a number in only
   * one list is single-direction (two slots, half a block each, block >=2xMPS).
   * EP0's RX[0] and TX[0] must be the same block (>=64 B, shared). Absent
   * directions pass {nullptr, 0}.
   */
  CH32USBOtgFS(
      const std::initializer_list<RawData> RX_EP_CFGS,
      const std::initializer_list<RawData> TX_EP_CFGS,
      USB::DeviceDescriptor::PacketSize0 packet_size, uint16_t vid, uint16_t pid,
      uint16_t bcd,
      const std::initializer_list<const USB::DescriptorStrings::LanguagePack*> LANG_LIST,
      const std::initializer_list<const std::initializer_list<USB::ConfigDescriptorItem*>>
          CONFIGS,
      ConstRawData uid = {nullptr, 0});

  /// 在状态阶段 IN 完成时写入 DEV_ADDR（保留 UDA_GP 位）并让 EP0 回到 RX ACK / TX NAK；
  /// 其余阶段不做事 / At STATUS_IN_COMPLETE writes DEV_ADDR (keeping the UDA_GP bits) and
  /// sets EP0 to RX ACK / TX NAK; does nothing in other phases
  ErrorCode SetAddress(uint8_t address, USB::ControlContext context) override;

  /// 将 EP0 RX 置为 ACK，准备接收下一个 SETUP / Set EP0 RX to ACK to receive the next
  /// SETUP
  void OnControlReady() override;

  /// 配置 48 MHz 时钟、复位 SIE、使能中断与上拉并打开 USBFS_IRQn / Configure the 48 MHz
  /// clock, reset the SIE, enable interrupts and the pull-up, and enable USBFS_IRQn
  void Start(bool in_isr) override;

  /// 复位 SIE、关闭控制器与 USBFS_IRQn / Reset the SIE, switch the controller off and
  /// disable USBFS_IRQn
  void Stop(bool in_isr) override;

  /// 当前已启动的实例，供中断服务函数使用；Stop() 后为空 / Started instance used by the
  /// ISR; null after Stop()
  static inline CH32USBOtgFS* self_ = nullptr;
};

#endif  // defined(USBFSD) || defined(__DOXYGEN__)

#if defined(USBHSD) || defined(__DOXYGEN__)

/**
 * @brief CH32 OTG HS 设备驱动 / CH32 OTG HS device driver
 */
class CH32USBOtgHS : public USB::EndpointPool,
                     public USB::DeviceCore<USB::HighSpeedCapabilities>
{
 public:
  /**
   * @brief 构造 OTG HS 设备对象 / Construct OTG HS device object
   *
   * RX/TX 列表按下标对应端点号（EP0 在首位）。HS 的 EP1-15 收发 DMA 寄存器
   * 分立，双向端点两列表的块无相邻要求（各 ≥64 B，默认两槽由软件切换 DMA
   * 指针实现，硬件 BUF_MOD 双缓冲不再使用）；EP0 收发共用
   * 一个 DMA 地址，RX[0] 与 TX[0] 必须是同一块。缺席方向传 {nullptr, 0}。
   * RX/TX lists are indexed by endpoint number (EP0 first). HS EP1-15 have
   * separate RX/TX DMA registers, so a bidirectional endpoint's two blocks have
   * no adjacency requirement (each >=64 B; the two slots are switched by software
   * DMA pointers, the hardware BUF_MOD double buffer is no longer used). EP0 shares one
   * DMA address, so RX[0] and TX[0] must be the same block. Absent directions pass
   * {nullptr, 0}.
   */
  CH32USBOtgHS(
      const std::initializer_list<RawData> RX_EP_CFGS,
      const std::initializer_list<RawData> TX_EP_CFGS, uint16_t vid, uint16_t pid,
      uint16_t bcd,
      const std::initializer_list<const USB::DescriptorStrings::LanguagePack*> LANG_LIST,
      const std::initializer_list<const std::initializer_list<USB::ConfigDescriptorItem*>>
          CONFIGS,
      ConstRawData uid = {nullptr, 0});

  /// 在状态阶段 IN 完成时写入 DEV_AD；其余阶段不做事 / Writes DEV_AD at
  /// STATUS_IN_COMPLETE; does nothing in other phases
  ErrorCode SetAddress(uint8_t address, USB::ControlContext context) override;

  /// 读取 SPEED_TYPE 得到总线复位后协商的速度 / Read SPEED_TYPE for the speed negotiated
  /// after bus reset
  USB::Speed ReadBusSpeed() const override
  {
    return (USBHSD->SPEED_TYPE & USBHS_USB_SPEED_TYPE) == USBHS_USB_SPEED_HIGH
               ? USB::Speed::HIGH
               : USB::Speed::FULL;
  }

  /// 将 EP0 RX 置为 ACK，准备接收下一个 SETUP / Set EP0 RX to ACK to receive the next
  /// SETUP
  void OnControlReady() override;

  /// 配置 48 MHz 时钟、复位 SIE、以高速使能 DMA/中断/上拉并打开 USBHS_IRQn / Configure
  /// the 48 MHz clock, reset the SIE, enable DMA, interrupts and the pull-up at high
  /// speed, and enable USBHS_IRQn
  void Start(bool in_isr) override;

  /// 复位 SIE、关闭控制器与 USBHS_IRQn / Reset the SIE, switch the controller off and
  /// disable USBHS_IRQn
  void Stop(bool in_isr) override;

  /// 当前已启动的实例，供中断服务函数使用；Stop() 后为空 / Started instance used by the
  /// ISR; null after Stop()
  static inline CH32USBOtgHS* self_ = nullptr;
};

#endif  // defined(USBHSD) || defined(__DOXYGEN__)

}  // namespace LibXR
