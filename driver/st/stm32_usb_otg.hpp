#pragma once

#include "main.h"

#undef USB

#include <atomic>

#include "driver_async_condition.hpp"
#include "usb/core/ep.hpp"
#include "usb/core/ep_pool.hpp"
#include "usb/device/dev_core.hpp"

#if defined(HAL_PCD_MODULE_ENABLED) && (defined(USB_OTG_FS) || defined(USB_OTG_HS))

namespace LibXR
{
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
 * 端点停止超时时，设备离开总线并在毫秒节拍上复位 core（DriverAsyncCondition：等 AHB
 * 空闲、
 * 软复位、强制设备模式并等待生效），然后在自己的中断里恢复配置并重新连接，主机重新枚举。
 * 复位期间端点操作只改软件状态。复位超时时设备保持断开，下一次 Start() 重新复位。
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
 * When an endpoint stop times out, the device leaves the bus and resets the core on the
 * millisecond tick (DriverAsyncCondition: wait for the AHB to idle, soft reset, force
 * device mode and wait for it to take effect), then restores the configuration in its own
 * interrupt and reconnects; the host enumerates again. During the reset endpoint
 * operations change only the software state. When the reset times out the device stays
 * disconnected and the next Start() resets again.
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
    Phy phy = Phy::EMBEDDED_FS;
    bool dma = false;           ///< 使用 core 内部 DMA / Use the core's internal DMA
    bool vbus_sensing = false;  ///< VBUS 检测 / VBUS sensing
    bool high_speed = false;    ///< 以高速枚举（需高速 PHY）/ Enumerate at high speed
    /// DMA 时 EP0 OUT 的驱动缓冲区：≥ 64 B，32 字节对齐，DMA 可达 / The EP0 OUT buffer of
    /// the driver with DMA: ≥ 64 B, 32-byte aligned, reachable by DMA
    RawData ep0_dma_buffer{nullptr, 0};
  };

  /// IN 端点：槽内存与 TX FIFO 字节数 / IN endpoint: slot memory and TX FIFO bytes
  struct EPInConfig
  {
    RawData buffer;
    size_t fifo_size;
  };

  /// 中断入口：控制器 core 的中断 / Interrupt entry for the controller core
  static void Dispatch(USB_OTG_GlobalTypeDef* core);

  void Init(bool in_isr) override { STM32USBDeviceCore::Init(in_isr); }
  void Deinit(bool in_isr) override { STM32USBDeviceCore::Deinit(in_isr); }
  void Start(bool in_isr) override;
  void Stop(bool in_isr) override;

 protected:
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

  ErrorCode SetAddress(uint8_t address, USB::ControlContext context) override;
  void OnControllerReset() override;
  void OnControlReady() override;
  void OnStopTimeout(bool in_isr) override;
  void OnStopsSettled() override;
  USB::Speed ReadBusSpeed() const override;

 private:
  friend class STM32OtgEndpoint;

  static constexpr uint8_t MAX_ENDPOINTS = 16;
  static constexpr uint8_t MAX_DEVICES = 2;
  static constexpr uint8_t NO_FLUSH = 0xFF;
  static constexpr uint8_t FLUSH_ALL = 0x10;

  // Core reset after a stop timeout. The stages up to DEVICE_MODE run on the millisecond
  // tick and own the registers; RESET_DONE waits for the interrupt to restore the rest.
  enum class Recovery : uint8_t
  {
    IDLE,
    AHB_IDLE,     ///< Waiting for GRSTCTL.AHBIDL before the soft reset.
    CORE_RESET,   ///< CSRST set; waiting for it to clear.
    DEVICE_MODE,  ///< FDMOD written; waiting for GINTSTS.CMOD to show device mode.
    RESET_DONE,   ///< Reset finished; the interrupt restores the device registers.
    FAILED        ///< A stage timed out; the device stays off the bus.
  };

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

  // Registers belong to the reset chain from AHB_IDLE until the interrupt has restored
  // them.
  bool Recovering() const
  {
    return recovery_.load(std::memory_order_acquire) != Recovery::IDLE;
  }
  void Connect();
  void ProgramFifos();
  void BeginRecovery();
  void FinishRecovery();
  static bool RecoveryCheck(void* arg);
  static void RecoveryDone(void* arg, ErrorCode result);

  uint8_t index_;
  USB_OTG_GlobalTypeDef* core_;
  uint8_t endpoint_count_;
  uint32_t core_id_ = 0;  // GSNPSID, read once after the core reset.
  HardwareConfig hw_;
  STM32OtgEndpoint* in_[MAX_ENDPOINTS] = {};
  STM32OtgEndpoint* out_[MAX_ENDPOINTS] = {};
  alignas(4) uint8_t setup_[8] = {};  // Non-DMA SETUP packet.
  USB::Speed bus_speed_ = USB::Speed::FULL;
  bool connected_ = false;  // Start() and Stop(): whether the device belongs on the bus.
  bool discard_ = false;    // Bus reset seen, the owner has not applied it yet.
  uint8_t global_nak_users_ = 0;
  uint32_t flush_pending_ = 0;  // Bit n: TX FIFO n; bit 16: every TX FIFO.
  uint8_t flush_active_ = NO_FLUSH;
  IRQn_Type irq_;
  // Configuration the soft reset may clear, saved after initialization.
  uint32_t gotgctl_ = 0;  // Session valid override bits only.
  uint32_t gahbcfg_ = 0;  // Without GINT.
  uint32_t gusbcfg_ = 0;
  uint32_t gccfg_ = 0;
  uint32_t gdfifocfg_ = 0;  // Cores with GDFIFOCFG only.
  uint32_t dcfg_ = 0;
  std::atomic<Recovery> recovery_{Recovery::IDLE};
  DriverAsyncCondition reset_wait_;

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
  STM32OtgEndpoint(STM32USBOtgDevice& device, EPNumber number, Direction direction,
                   size_t fifo_size, RawData buffer);

 protected:
  ErrorCode ConfigureHardware(const Config& cfg) override;
  void CloseHardware(bool in_isr) override;
  ErrorCode StartHardware(const HwRequest& request, bool in_isr) override;
  StopResult StopHardware(bool in_isr) override;
  size_t ReapProgress(const HwRequest& request, bool& retired) override;
  ErrorCode StallHardware(bool in_isr) override;
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
  // The transfer was in hardware when the core was reset, or started during the reset;
  // the registers no longer describe it and none of it counts as sent.
  bool hw_offline_ = false;
  StopStage stop_stage_ = StopStage::NONE;
};

#if defined(USB_OTG_FS)
/**
 * @brief STM32 OTG FS 设备 / STM32 OTG FS device
 */
class STM32USBDeviceOtgFS : public STM32USBOtgDevice
{
 public:
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
