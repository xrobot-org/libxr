#pragma once

#include "main.h"

#undef USB

#include "usb/core/ep.hpp"
#include "usb/core/ep_pool.hpp"
#include "usb/device/dev_core.hpp"

#if defined(HAL_PCD_MODULE_ENABLED) && (defined(USB_OTG_FS) || defined(USB_OTG_HS))

namespace LibXR
{
/// 设备核心类型：带 OTG HS 的器件按高速能力实例化，否则按全速 / Device core type:
/// high-speed capabilities on parts with OTG HS, full-speed otherwise
#if defined(USB_OTG_HS)
using STM32USBDeviceCore = USB::DeviceCore<USB::HighSpeedCapabilities>;
#else
using STM32USBDeviceCore = USB::DeviceCore<USB::FullSpeedCapabilities>;
#endif

class STM32OtgEndpoint;

/**
 * @brief STM32 OTG（DWC2）USB 设备：初始化之后由驱动直接读写寄存器
 *        STM32 OTG (DWC2) USB device; after initialization the driver programs the
 *        registers directly
 *
 * 不使用 PCD 句柄。构造时用临时句柄调用 CubeMX 生成的
 * HAL_PCD_MspInit()（时钟、引脚、NVIC）， 再用 LL 函数初始化
 * core；之后中断、端点启停、FIFO、DMA、连接与断开都由驱动完成。CubeMX
 * 保持默认设置：链接时包装 HAL_PCD_Init() 和 HAL_PCD_IRQHandler()，main() 里的
 * MX_USB_OTG_*_PCD_Init() 不再初始化控制器，CubeMX 生成的中断函数进入 Dispatch()。CubeMX
 * 不生成中断函数时，由 CodeGenerator 生成的向量函数调用 Dispatch()。
 *
 * 端点停止超时时，设备离开总线，所有等待中的停止被确认，会话结束；设备保持断开，直到
 * 应用调用 Stop() 和 Start()。
 *
 * No PCD handle is used. The constructor calls the CubeMX-generated HAL_PCD_MspInit()
 * (clocks, pins, NVIC) through a temporary handle and initializes the core with the LL
 * functions; from then on the driver handles interrupts, endpoint start and stop, the
 * FIFOs, DMA, connect and disconnect. CubeMX keeps its default settings: the link wraps
 * HAL_PCD_Init() and HAL_PCD_IRQHandler(), so MX_USB_OTG_*_PCD_Init() in main() no longer
 * initializes the controller and the CubeMX interrupt handler enters Dispatch(). When
 * CubeMX generates no interrupt handler, the vector function CodeGenerator writes calls
 * Dispatch().
 *
 * When an endpoint stop times out, the device leaves the bus, every waiting stop is
 * confirmed and the session ends; the device stays off the bus until the application
 * calls Stop() and Start().
 */
class STM32USBOtgDevice : public USB::EndpointPool, public STM32USBDeviceCore
{
 public:
  /// PHY 接口 / PHY interface
  enum class Phy : uint8_t
  {
    EMBEDDED_FS,  ///< 片内全速 PHY / Embedded full-speed PHY
    ULPI,         ///< 外部 ULPI 高速 PHY / External ULPI high-speed PHY
    EMBEDDED_HS   ///< 片内高速 PHY（F723/F733 等）/ Embedded high-speed PHY (F723/F733)
  };

  /// 控制器配置 / Controller configuration
  struct HardwareConfig
  {
    Phy phy = Phy::EMBEDDED_FS;  ///< PHY 接口 / PHY interface
    bool dma = false;            ///< 使用 core 内部 DMA / Use the core's internal DMA
    bool vbus_sensing = false;   ///< VBUS 检测 / VBUS sensing
    bool high_speed = false;     ///< 以高速枚举（需高速 PHY）/ Enumerate at high speed
    /// DMA 时 EP0 OUT 的驱动缓冲区：≥ 64 B，32 字节对齐，DMA 可达 / The EP0 OUT buffer of
    /// the driver with DMA: ≥ 64 B, 32-byte aligned, reachable by DMA
    RawData ep0_dma_buffer{nullptr, 0};
  };

  /// IN 端点：槽内存与 TX FIFO 字节数 / IN endpoint: slot memory and TX FIFO bytes
  struct EPInConfig
  {
    RawData buffer;    ///< 该端点的槽内存 / Slot memory of the endpoint
    size_t fifo_size;  ///< TX FIFO 字节数，至少 8 且为 4 的倍数 / TX FIFO bytes, at least
                       ///< 8 and a multiple of 4
  };

  /// 中断入口：控制器 core 的中断 / Interrupt entry for the controller core
  static void Dispatch(USB_OTG_GlobalTypeDef* core);

  void Init(bool in_isr) override { STM32USBDeviceCore::Init(in_isr); }
  void Deinit(bool in_isr) override { STM32USBDeviceCore::Deinit(in_isr); }
  /// 使能中断并连接总线 / Enable interrupts and connect to the bus
  void Start(bool in_isr) override;
  /// 断开总线、关闭中断，并确认所有等待中的端点停止 / Disconnect from the bus, disable
  /// interrupts and confirm every waiting endpoint stop
  void Stop(bool in_isr) override;

 protected:
  /**
   * @brief 构造并初始化 OTG core 与全部端点 / Construct and initialize the OTG core and
   *        all endpoints
   * @param index core 编号（小于 2）/ Core index (below 2)
   * @param core OTG core 寄存器 / OTG core registers
   * @param endpoint_count core 的端点数（不超过 16）/ Endpoint count of the core (at most
   *        16)
   * @param total_fifo core 的 FIFO RAM 总字节数 / Total FIFO RAM of the core in bytes
   * @param hw 控制器配置 / Controller configuration
   * @param rx_fifo_size 共享 RX FIFO 字节数，为 4 的倍数且不小于 64 乘以 OUT 端点数 /
   *        Shared RX FIFO bytes, a multiple of 4 and at least 64 times the OUT endpoint
   *        count
   * @param rx_ep_cfgs 各 OUT 端点的槽内存，第一项属于 EP0 / Slot memory of each OUT
   *        endpoint, the first entry belongs to EP0
   * @param tx_ep_cfgs 各 IN 端点的配置，第一项属于 EP0；RX 与全部 TX FIFO 之和不得超过
   *        total_fifo / Configuration of each IN endpoint, the first entry belongs to
   * EP0; the RX and all TX FIFOs together must fit in total_fifo
   * @param packet_size EP0 最大包长 / EP0 maximum packet size
   * @param vid 厂商 ID / Vendor ID
   * @param pid 产品 ID / Product ID
   * @param bcd 设备版本（bcdDevice）/ Device release number (bcdDevice)
   * @param lang_list 字符串描述符的语言包列表 / Language packs of the string descriptors
   * @param configs 各配置的描述符项列表 / Descriptor items of each configuration
   * @param uid 设备唯一 ID，用于序列号字符串，可为空 / Unique device ID used for the
   * serial number string, may be empty
   */
  STM32USBOtgDevice(
      uint8_t index, USB_OTG_GlobalTypeDef* core, uint8_t endpoint_count,
      size_t total_fifo, const HardwareConfig& hw, size_t rx_fifo_size,
      const std::initializer_list<RawData>& rx_ep_cfgs,
      const std::initializer_list<EPInConfig>& tx_ep_cfgs,
      USB::DeviceDescriptor::PacketSize0 packet_size, uint16_t vid, uint16_t pid,
      uint16_t bcd,
      const std::initializer_list<const USB::DescriptorStrings::LanguagePack*>& lang_list,
      const std::initializer_list<
          const std::initializer_list<USB::ConfigDescriptorItem*>>& configs,
      ConstRawData uid);

  /// 在状态阶段之前把设备地址写入 DCFG / Write the device address to DCFG before the
  /// status stage
  ErrorCode SetAddress(uint8_t address, USB::ControlContext context) override;
  void OnControllerReset() override;
  /// 重新装填 EP0 OUT 以接收下一个 SETUP / Re-arm EP0 OUT to receive the next SETUP
  void OnControlReady() override;
  void OnStopTimeout(bool in_isr) override;
  void OnStopsSettled() override;
  /// 返回枚举完成后记录的总线速度 / Return the bus speed recorded at enumeration
  USB::Speed ReadBusSpeed() const override;

 private:
  friend class STM32OtgEndpoint;

  static constexpr uint8_t MAX_ENDPOINTS = 16;
  static constexpr uint8_t MAX_DEVICES = 2;
  static constexpr uint8_t NO_FLUSH = 0xFF;
  static constexpr uint8_t FLUSH_ALL = 0x10;

  // Register blocks of this core.
  USB_OTG_DeviceTypeDef* Device() const;
  USB_OTG_INEndpointTypeDef* InEp(uint8_t num) const;
  USB_OTG_OUTEndpointTypeDef* OutEp(uint8_t num) const;
  volatile uint32_t& Fifo(uint8_t num) const;
  uint32_t CoreId() const;

  void HandleInterrupt();
  void OnUsbReset();
  void OnEnumerationDone();
  void DrainRxFifo();
  void HandleOutEndpoints();
  void HandleInEndpoints();
  void HandleGlobalOutNak();
  void HandleSof();
  void ArmSetup();
  void DeliverSetup(const uint8_t* packet);

  // INEPNE is unmasked only while an IN stop waits for it.
  void UpdateInNakMask();
  // Global OUT NAK, shared by every OUT endpoint that stops.
  bool AcquireGlobalOutNak();
  void ReleaseGlobalOutNak();
  // TX FIFO flushes: one at a time in GRSTCTL, never waited for.
  void RequestTxFlush(uint8_t fifo);
  void PollTxFlush();
  // The bus tick (SOF) runs while a stop or a flush waits, or bus time is wanted.
  void UpdateTick();
  void ForceStopAll(bool in_isr);

  void Connect();
  void ProgramFifos();

  uint8_t index_;
  USB_OTG_GlobalTypeDef* core_;
  uint8_t endpoint_count_;
  uint32_t core_id_ = 0;  // GSNPSID, read once after the core reset.
  HardwareConfig hw_;
  STM32OtgEndpoint* in_[MAX_ENDPOINTS] = {};
  STM32OtgEndpoint* out_[MAX_ENDPOINTS] = {};
  alignas(4) uint8_t setup_[8] = {};  // Non-DMA SETUP packet.
  USB::Speed bus_speed_ = USB::Speed::FULL;
  bool discard_ = false;  // Bus reset seen, the owner has not applied it yet.
  uint8_t global_nak_users_ = 0;
  uint32_t flush_pending_ = 0;  // Bit n: TX FIFO n; bit 16: every TX FIFO.
  uint8_t flush_active_ = NO_FLUSH;

  static inline STM32USBOtgDevice* devices_[MAX_DEVICES] = {};
};

/**
 * @brief STM32 OTG 端点 / STM32 OTG endpoint
 *
 * 非 DMA 时 IN 由 CPU 写 TX FIFO（启动时写第一个包，其余在 TXFE 中断里按 FIFO
 * 空间写），OUT 由 RXFLVL 中断读进槽；DMA 时 core 直接读写槽，EP0 OUT
 * 例外，始终收进设备的 EP0 缓冲区再 拷进槽。停止由硬件事件推进：IN 为 SNAK → INEPNE →
 * EPDIS → EPDISD → 清 TX FIFO；OUT 为全局 OUT NAK → EPDIS → EPDISD。
 *
 * Without DMA the CPU writes IN data to the TX FIFO (the first packet at start, the rest
 * in TXFE interrupts as FIFO space allows) and reads OUT data into the slot on RXFLVL;
 * with DMA the core reads and writes the slot itself, except EP0 OUT, which always
 * receives into the device's EP0 buffer and is copied into the slot. Stops advance on
 * hardware events: IN goes SNAK → INEPNE → EPDIS → EPDISD → TX FIFO flush; OUT goes
 * global OUT NAK → EPDIS → EPDISD.
 */
class STM32OtgEndpoint : public USB::Endpoint
{
 public:
  /// @param device 所属 OTG 设备 / Owning OTG device
  /// @param number 端点号 / Endpoint number
  /// @param direction 端点方向 / Endpoint direction
  /// @param fifo_size FIFO 字节数（OUT 为共享 RX FIFO 大小）/ FIFO bytes (the shared RX
  /// FIFO size for OUT)
  /// @param buffer 该端点的槽内存 / Slot memory of the endpoint
  STM32OtgEndpoint(STM32USBOtgDevice& device, EPNumber number, Direction direction,
                   size_t fifo_size, RawData buffer);

 protected:
  ErrorCode ConfigureHardware(const Config& cfg) override;
  void CloseHardware(bool in_isr) override;
  ErrorCode StartHardware(const HwRequest& request, bool in_isr) override;
  StopResult StopHardware(bool in_isr) override;
  size_t ReapProgress(const HwRequest& request, bool& retired) override;
  /// 置位 STALL（EP0 OUT 在 DMA 下同时重新装填 SETUP）/ Set STALL (with DMA, EP0 OUT
  /// also re-arms SETUP)
  ErrorCode StallHardware(bool in_isr) override;
  /// 清除 STALL；EP0 的 STALL 由下一个 SETUP 清除，批量/中断端点复位为 DATA0 / Clear
  /// STALL; the next SETUP clears an EP0 STALL, bulk/interrupt endpoints reset to DATA0
  ErrorCode ClearStallHardware(bool in_isr) override;
  size_t MaxPacketSizeLimit(Type type) const override;

 private:
  friend class STM32USBOtgDevice;

  enum class StopStage : uint8_t
  {
    NONE,
    IN_NAK,          ///< SNAK set; waiting for INEPNE.
    IN_DISABLE,      ///< EPDIS set; waiting for EPDISD.
    IN_FLUSH,        ///< Disabled; waiting for the TX FIFO flush.
    OUT_GLOBAL_NAK,  ///< Global OUT NAK requested; waiting for GONAKEFF.
    OUT_DISABLE      ///< EPDIS set; waiting for EPDISD.
  };

  bool IsIn() const { return GetConfig().direction == Direction::IN; }
  uint8_t Num() const { return EPNumberToInt8(GetNumber()); }
  uint32_t Packet() const { return MaxPacketSize(); }
  bool Dma() const { return device_.hw_.dma; }

  // IRQ side.
  void FillFifo();
  void ReadFifo(size_t bytes);
  void OnTransferComplete();
  void OnNakEffective();
  void OnDisabled();
  void OnGlobalOutNak();
  void OnFlushDone();
  void PollStop();
  void ConfirmStop(bool in_isr);
  void ResetTransfer();

  STM32USBOtgDevice& device_;
  size_t fifo_size_;
  // Written with the controller IRQ masked or inside it.
  uint32_t hw_tag_ = 0;  // Tag of the transfer in hardware; 0 without one.
  uint8_t* hw_data_ = nullptr;
  size_t hw_length_ = 0;      // Request length.
  size_t hw_programmed_ = 0;  // XFRSIZ written at start (OUT).
  size_t hw_capacity_ = 0;    // Bytes the slot can take (OUT).
  size_t hw_done_ = 0;        // IN: bytes written to the FIFO; OUT: bytes received.
  size_t stopped_actual_ = 0;
  StopStage stop_stage_ = StopStage::NONE;
};

#if defined(USB_OTG_FS)
/**
 * @brief STM32 OTG FS 设备 / STM32 OTG FS device
 */
class STM32USBDeviceOtgFS : public STM32USBOtgDevice
{
 public:
  /**
   * @brief 构造 OTG FS 设备并初始化 core / Construct the OTG FS device and initialize the
   * core
   * @param hw 控制器配置 / Controller configuration
   * @param rx_fifo_size 共享 RX FIFO 字节数 / Shared RX FIFO bytes
   * @param RX_EP_CFGS 各 OUT 端点的槽内存，第一项属于 EP0 / Slot memory of each OUT
   *        endpoint, the first entry belongs to EP0
   * @param TX_EP_CFGS 各 IN 端点的配置，第一项属于 EP0 / Configuration of each IN
   *        endpoint, the first entry belongs to EP0
   * @param packet_size EP0 最大包长 / EP0 maximum packet size
   * @param vid 厂商 ID / Vendor ID
   * @param pid 产品 ID / Product ID
   * @param bcd 设备版本（bcdDevice）/ Device release number (bcdDevice)
   * @param LANG_LIST 字符串描述符的语言包列表 / Language packs of the string descriptors
   * @param CONFIGS 各配置的描述符项列表 / Descriptor items of each configuration
   * @param uid 设备唯一 ID，可为空 / Unique device ID, may be empty
   */
  STM32USBDeviceOtgFS(
      const HardwareConfig& hw, size_t rx_fifo_size,
      const std::initializer_list<RawData> RX_EP_CFGS,
      const std::initializer_list<EPInConfig> TX_EP_CFGS,
      USB::DeviceDescriptor::PacketSize0 packet_size, uint16_t vid, uint16_t pid,
      uint16_t bcd,
      const std::initializer_list<const USB::DescriptorStrings::LanguagePack*> LANG_LIST,
      const std::initializer_list<const std::initializer_list<USB::ConfigDescriptorItem*>>
          CONFIGS,
      ConstRawData uid = {nullptr, 0});
};
#endif

#if defined(USB_OTG_HS)
/**
 * @brief STM32 OTG HS 设备 / STM32 OTG HS device
 */
class STM32USBDeviceOtgHS : public STM32USBOtgDevice
{
 public:
  /**
   * @brief 构造 OTG HS 设备并初始化 core / Construct the OTG HS device and initialize the
   * core
   * @param hw 控制器配置 / Controller configuration
   * @param rx_fifo_size 共享 RX FIFO 字节数 / Shared RX FIFO bytes
   * @param RX_EP_CFGS 各 OUT 端点的槽内存，第一项属于 EP0 / Slot memory of each OUT
   *        endpoint, the first entry belongs to EP0
   * @param TX_EP_CFGS 各 IN 端点的配置，第一项属于 EP0 / Configuration of each IN
   *        endpoint, the first entry belongs to EP0
   * @param packet_size EP0 最大包长 / EP0 maximum packet size
   * @param vid 厂商 ID / Vendor ID
   * @param pid 产品 ID / Product ID
   * @param bcd 设备版本（bcdDevice）/ Device release number (bcdDevice)
   * @param LANG_LIST 字符串描述符的语言包列表 / Language packs of the string descriptors
   * @param CONFIGS 各配置的描述符项列表 / Descriptor items of each configuration
   * @param uid 设备唯一 ID，可为空 / Unique device ID, may be empty
   */
  STM32USBDeviceOtgHS(
      const HardwareConfig& hw, size_t rx_fifo_size,
      const std::initializer_list<RawData> RX_EP_CFGS,
      const std::initializer_list<EPInConfig> TX_EP_CFGS,
      USB::DeviceDescriptor::PacketSize0 packet_size, uint16_t vid, uint16_t pid,
      uint16_t bcd,
      const std::initializer_list<const USB::DescriptorStrings::LanguagePack*> LANG_LIST,
      const std::initializer_list<const std::initializer_list<USB::ConfigDescriptorItem*>>
          CONFIGS,
      ConstRawData uid = {nullptr, 0});
};
#endif

}  // namespace LibXR

#endif
