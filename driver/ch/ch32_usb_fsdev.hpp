#pragma once

#include "libxr_def.hpp"
#include DEF2STR(LIBXR_CH32_CONFIG_FILE)

#include "usb/backend/fsdev.hpp"

#if defined(RCC_APB1Periph_USB) || defined(__DOXYGEN__)

/**
 * @brief 断开时把 D+（PA12）拉低 / Drive D+ (PA12) low while detached
 * @note 默认 1，见 CH32USBDeviceFS 的说明；PA12 不能被驱动的板子定义为 0。
 *       Default 1, see the notes of CH32USBDeviceFS; a board whose PA12 must not be
 *       driven defines it as 0.
 */
#if !defined(LIBXR_CH32_USBD_DP_DETACH)
#define LIBXR_CH32_USBD_DP_DETACH 1
#endif

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
 * @note 中断线：低优先级线 USB_LP_CAN1_RX0 和高优先级线 USB_HP_CAN1_TX 都打开，都进入
 *       FSDevDevice::Dispatch；唤醒线只在内核停止时钟时才需要，不打开。
 *       - 与手册的出入（CH32V203C8 实测）：手册 p.321 说低优先级线“可由所有 USB 事件
 *         触发（正确传输……）”，实际上等时端点的正确传输不及时触发它。只开低优先级线时，
 *         等时 OUT 60 s 主机完成 60,008 包，设备只收到 56,220（缺 6.3%）；等时 IN 在
 *         批量压力旁有重复包。高优先级线也进入服务之后，等时 OUT 60,008/60,008（两次），
 *         等时 IN 没有重复。
 *       - 两条线的优先级相同（构造时和每次开线时让高优先级线取低优先级线的值）。同优先级
 *         的中断互不打断，所以 Dispatch 一次只从一条线进入，EP0 的快速确认和状态 OUT 的
 *         晚放行依赖的“控制器服务不重入”仍成立。owner 屏蔽两条线，挂起只挂低优先级线。
 *       - 两条线分别与 CAN1 的 RX0、TX 共用向量，入口在 ch32_usbcan_shared.cpp：CAN 的
 *         回调与 USB 同优先级，也不会打断 Dispatch；owner 屏蔽期间 CAN1 的 TX 中断推迟到
 *         开线之后。
 *       Interrupt lines: the low-priority line USB_LP_CAN1_RX0 and the high-priority line
 *       USB_HP_CAN1_TX are both enabled and both enter FSDevDevice::Dispatch; the
 *       wake-up line is needed only when the core stops its clocks and is not enabled.
 *       - Deviation from the manual (measured on a CH32V203C8): manual p.321 says the
 *         low-priority line "can be triggered by all USB events (correct transfer, ...)",
 *         but a correct transfer of an isochronous endpoint does not trigger it in time.
 *         With the low-priority line alone, isochronous OUT over 60 s: the host
 *         completed 60,008 packets, the device took 56,220 (6.3% missing); isochronous IN
 *         beside bulk stress repeated packets. With the high-priority line entering the
 *         service as well: isochronous OUT 60,008/60,008 (twice), isochronous IN without
 *         repeats.
 *       - Both lines have the same priority (the high-priority line takes the
 *         low-priority line's value at construction and at every unmask). Interrupts of
 *         equal priority do not interrupt each other, so Dispatch is entered from one
 *         line at a time and the "no re-entry of the controller service" that the fast
 *         acknowledgement of EP0 and the late release of the status OUT rely on still
 *         holds. The owner masks both lines; the pend sets the low-priority line only.
 *       - The two lines share their vectors with RX0 and TX of CAN1; the entries are in
 *         ch32_usbcan_shared.cpp. The CAN callbacks run at the USB priority and do not
 *         interrupt Dispatch either; while the owner masks the lines, CAN1's TX interrupt
 *         waits until the unmask.
 * @note EP0 的 OUT（与手册 p.323 的出入，CH32V203C8 实测）：EP0 为 CONTROL 类型时，控制器
 *       收下并 ACK 主机的每个 OUT 数据包，不看 STAT_RX，上一包的 CTR_RX 还没清也照收，
 *       上一包被覆盖。33 字节的控制写，第二包（1 字节）在数据请求启动之前覆盖了第一包
 *       （32 字节）；不加处理时 300 次 1–150 字节的控制写只有 82 次对。所以打开
 *       FSDevDevice::EnableEp0BulkGate（TinyUSB #3648 的做法）和 EnableEp0EarlyOut。
 *       EP0 OUT (deviation from manual p.323, measured on a CH32V203C8): with EP0 typed
 *       CONTROL the controller takes and ACKs every OUT data packet of the host, whatever
 *       STAT_RX says and with the previous packet's CTR_RX not yet cleared, overwriting
 *       that packet. In a 33-byte control write the second packet (1 byte) overwrote the
 *       first (32 bytes) before the data request started; untreated, 82 of 300 control
 *       writes of 1-150 bytes were right. FSDevDevice::EnableEp0BulkGate (the approach
 *       of TinyUSB #3648) and EnableEp0EarlyOut are therefore enabled.
 * @note 断开时驱动 D+（LIBXR_CH32_USBD_DP_DETACH 为 1，默认）。Bluepill 一类的板子可能
 *       在 D+ 上焊了固定上拉，只关内部上拉断不开，ResetController 之后主机还留着旧地址。
 *       于是构造、Stop、ResetController 的每次断开都：Connect(false) → CNTR = FRES|PDWN →
 *       关 USBD 的接口时钟 → 保存 PA12 的 CFGHR[19:16] 和 OUTDR12 → OUTDR12 = 0、推挽
 *       输出 2 MHz → 保持 DETACH_US → 恢复 PA12 → 开 USBD 时钟 → InitController →
 *       Start 时接上拉。D-（PA11）不动。
 *       - 实测（CH32V203C8 仿板，Linux 主机根口）：拉住期间 PA12 的 INDR 两次采样（拉低
 *         之后、放开之前）11/11 次都是 0，即 USBD 时钟关着时 GPIO 的推挽输出压过了收发器
 *         （手册 10-8 只说模块使能后引脚自动接到收发器，没说关时钟之后）；强制
 *         ResetController 10 次，主机每次都记下一次断开，设备都以新地址回来（10/10，
 *         0.42 s）。
 *       - 这块板子 D+ 上没有固定上拉：关掉驱动（宏为 0）同样 10/10。驱动在这里不是必需
 *         的，在带固定上拉的板子上才起作用；那种板子在 CH32 上没有测过。
 *       D+ is driven while detached (LIBXR_CH32_USBD_DP_DETACH 1, the default). Boards of
 *       the Bluepill kind may carry a fixed pull-up on D+, so switching the internal
 *       pull-up off does not detach and after ResetController the host keeps the old
 *       address. Every detach (construction, Stop, ResetController) therefore runs:
 *       Connect(false) -> CNTR = FRES|PDWN -> interface clock of USBD off -> save PA12's
 *       CFGHR[19:16] and OUTDR12 -> OUTDR12 = 0 and push-pull output 2 MHz -> hold
 *       DETACH_US -> restore PA12 -> USBD clock on -> InitController -> Start connects
 * the pull-up. D- (PA11) is left alone.
 *       - Measured (CH32V203C8 clone board, Linux host root port): during the hold both
 *         samples of PA12's INDR (after the drive, before the release) were 0 in 11/11
 *         holds, so with the USBD clock off the GPIO push-pull output overrides the
 *         transceiver (manual 10-8 says only that the pins connect to the transceiver
 * once the module is enabled, nothing about the clock being off); forced ResetController
 * 10 times, the host logged a disconnect every time and the device came back with a new
 * address every time (10/10, 0.42 s).
 *       - This board has no fixed pull-up on D+: with the drive switched off (macro 0) it
 *         is 10/10 too. The drive is not needed here and matters only on a board with a
 *         fixed pull-up; such a board has not been measured with a CH32.
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
  /// 屏蔽两条线 / Mask both lines (NVIC_DisableIRQ)
  void MaskInterrupt() override;
  /// 高优先级线取低优先级线的优先级，开两条线 / The high-priority line takes the
  /// low-priority line's priority; both lines unmasked
  void UnmaskInterrupt() override;
  /// NVIC_SetPendingIRQ(USB_LP_CAN1_RX0_IRQn)，返回 true / NVIC_SetPendingIRQ, true
  bool PendInterrupt() override;
  /// 断开期间：收发器断电、关 USBD 时钟、PA12 推挽输出低 / While detached: transceiver
  /// powered down, USBD clock off, PA12 push-pull low
  bool HoldDetached() override;
  /// 恢复 PA12 的配置、开 USBD 时钟 / Restore PA12's configuration, USBD clock on
  void ReleaseDetached() override;

 private:
  /// HoldDetached 保存的 PA12 配置（CFGHR[19:16]、OUTDR12）/ PA12's configuration saved
  /// by HoldDetached (CFGHR[19:16], OUTDR12)
  uint32_t dp_saved_[2] = {};
};

}  // namespace LibXR

#endif  // defined(RCC_APB1Periph_USB) || defined(__DOXYGEN__)
