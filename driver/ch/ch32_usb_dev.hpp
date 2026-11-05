#pragma once

#include "ch32_usb.hpp"
#include "ch32_usb_endpoint.hpp"
#include "ch32_usb_fsdev_regs.hpp"
#include "ch32_usbcan_shared.hpp"
#include "libxr_def.hpp"
#include "libxr_type.hpp"
#include "usb/core/ep_pool.hpp"
#include "usb/device/dev_core.hpp"

namespace LibXR
{

class CH32USBDevice : public USB::EndpointPool,
                      public USB::DeviceCore<USB::FullSpeedCapabilities>
{
 public:
  /**
   * @brief 构造 CH32 USB 设备核心对象 / Construct CH32 USB device core
   */
  CH32USBDevice(
      ch32_usb_dev_id_t id, size_t max_ep_num,
      USB::DeviceDescriptor::PacketSize0 packet_size, uint16_t vid, uint16_t pid,
      uint16_t bcd,
      const std::initializer_list<const USB::DescriptorStrings::LanguagePack*> LANG_LIST,
      const std::initializer_list<const std::initializer_list<USB::ConfigDescriptorItem*>>
          CONFIGS,
      USB::Speed speed = USB::Speed::FULL, USB::USBSpec spec = USB::USBSpec::USB_2_1)
      : USB::EndpointPool(max_ep_num),
        USB::DeviceCore<USB::FullSpeedCapabilities>(*this, spec, speed, packet_size, vid,
                                                    pid, bcd, LANG_LIST, CONFIGS),
        id_(id)
  {
  }

  void Init(bool in_isr) override
  {
    USB::DeviceCore<USB::FullSpeedCapabilities>::Init(in_isr);
  }
  void Deinit(bool in_isr) override
  {
    USB::DeviceCore<USB::FullSpeedCapabilities>::Deinit(in_isr);
  }

  uint8_t id_;
};

#if defined(RCC_APB1Periph_USB) || defined(__DOXYGEN__)

/**
 * @brief CH32 FSDEV 设备驱动 / CH32 FSDEV device driver
 *
 * PMA 布局构造期固定，之后不能再增加 CAN 的占用；PMA 可用量取构造时刻的
 * `usb_pma_limit_bytes()`。寄存器运行期与 STM32 FSDEV 共用（usb/backend/fsdev.hpp），
 * 中断经 `ch32_usbcan_shared` 注册到 USB/CAN 共享向量。
 *
 * The PMA layout is fixed at construction and CAN usage cannot grow afterwards; the
 * usable PMA is `usb_pma_limit_bytes()` at construction time. The register runtime is
 * shared with STM32 FSDEV (usb/backend/fsdev.hpp); the interrupt is registered through
 * `ch32_usbcan_shared` on the shared USB/CAN vectors.
 */
class CH32USBDeviceFS : public USB::FSDevDevice<Ch32DevFsHw>
{
 public:
  using EPConfig = typename USB::FSDevDevice<Ch32DevFsHw>::EPConfig;

  /**
   * @brief 构造 FSDEV 设备对象 / Construct FSDEV device object
   */
  CH32USBDeviceFS(
      const std::initializer_list<EPConfig>& rx_ep_cfgs,
      const std::initializer_list<EPConfig>& tx_ep_cfgs,
      USB::DeviceDescriptor::PacketSize0 packet_size, uint16_t vid, uint16_t pid,
      uint16_t bcd,
      const std::initializer_list<const USB::DescriptorStrings::LanguagePack*> LANG_LIST,
      const std::initializer_list<const std::initializer_list<USB::ConfigDescriptorItem*>>
          CONFIGS,
      ConstRawData uid = {nullptr, 0})
      : USB::FSDevDevice<Ch32DevFsHw>(rx_ep_cfgs, tx_ep_cfgs, packet_size, vid, pid, bcd,
                                      LANG_LIST, CONFIGS, uid,
                                      CH32UsbCanShared::usb_pma_limit_bytes())
  {
    CH32UsbCanShared::usb_pma_configured.store(true, std::memory_order_release);
  }

  void Start(bool in_isr) override;
  void Stop(bool in_isr) override;
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

  ErrorCode SetAddress(uint8_t address, USB::ControlContext context) override;

  void OnControlReady() override;

  void Start(bool in_isr) override;
  void Stop(bool in_isr) override;

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
   * 指针实现，硬件 BUF_MOD 双缓冲不再使用——MODEL_USB §2.2）；EP0 收发共用
   * 一个 DMA 地址，RX[0] 与 TX[0] 必须是同一块。缺席方向传 {nullptr, 0}。
   * RX/TX lists are indexed by endpoint number (EP0 first). HS EP1-15 have
   * separate RX/TX DMA registers, so a bidirectional endpoint's two blocks have
   * no adjacency requirement (each >=64 B; the two slots are switched by software
   * DMA pointers, the hardware BUF_MOD double buffer is no longer used -
   * MODEL_USB §2.2). EP0 shares one DMA address, so RX[0] and TX[0] must be the
   * same block. Absent directions pass {nullptr, 0}.
   */
  CH32USBOtgHS(
      const std::initializer_list<RawData> RX_EP_CFGS,
      const std::initializer_list<RawData> TX_EP_CFGS, uint16_t vid, uint16_t pid,
      uint16_t bcd,
      const std::initializer_list<const USB::DescriptorStrings::LanguagePack*> LANG_LIST,
      const std::initializer_list<const std::initializer_list<USB::ConfigDescriptorItem*>>
          CONFIGS,
      ConstRawData uid = {nullptr, 0});

  ErrorCode SetAddress(uint8_t address, USB::ControlContext context) override;

  USB::Speed ReadBusSpeed() const override
  {
    return (USBHSD->SPEED_TYPE & USBHS_USB_SPEED_TYPE) == USBHS_USB_SPEED_HIGH
               ? USB::Speed::HIGH
               : USB::Speed::FULL;
  }

  void OnControlReady() override;

  void Start(bool in_isr) override;
  void Stop(bool in_isr) override;

  static inline CH32USBOtgHS* self_ = nullptr;
};

#endif  // defined(USBHSD) || defined(__DOXYGEN__)

}  // namespace LibXR
