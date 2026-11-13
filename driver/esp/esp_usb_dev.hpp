#pragma once

#include <cstddef>
#include <cstdint>
#include <initializer_list>

#include "driver_async_condition.hpp"
#include "esp_def.hpp"
#include "esp_intr_alloc.h"
#include "freertos/FreeRTOS.h"
#include "libxr_type.hpp"
#include "soc/soc_caps.h"
#include "usb/core/ep_pool.hpp"
#include "usb/device/dev_core.hpp"

#if (SOC_USB_OTG_SUPPORTED && defined(CONFIG_IDF_TARGET_ESP32S3) && \
     CONFIG_IDF_TARGET_ESP32S3) ||                                  \
    defined(__DOXYGEN__)

namespace LibXR
{

class ESP32USBEndpoint;

/**
 * @brief ESP32-S3 USB OTG 设备核心实现 / ESP32-S3 USB OTG device-core implementation
 */
class ESP32USBDevice : public USB::EndpointPool,
                       public USB::DeviceCore<USB::FullSpeedCapabilities>
{
 public:
  /**
   * @brief ESP32-S3 USB endpoint 配置项 / ESP32-S3 USB endpoint configuration entry
   */
  struct EPConfig
  {
    /**
     * @brief 该配置项应如何生成 endpoint 方向 / How this config entry should populate
     * endpoint directions
     */
    enum class DirectionHint : int8_t
    {
      BothDirections =
          -1,       ///< 同时生成 IN/OUT endpoint / Create both IN and OUT endpoints
      OutOnly = 0,  ///< 仅生成 OUT endpoint / Create only the OUT endpoint
      InOnly = 1,   ///< 仅生成 IN endpoint / Create only the IN endpoint
    };

    RawData buffer;  ///< 该 endpoint 共享的 payload buffer / Shared payload buffer for
                     ///< this endpoint entry
    DirectionHint direction_hint =
        DirectionHint::BothDirections;  ///< 该配置项的方向生成提示 / Direction-population
                                        ///< hint for this entry

    EPConfig() = delete;

    /**
     * @brief 使用同一块 buffer 同时生成 IN/OUT endpoint / Create both IN and OUT
     * endpoints from one shared buffer
     */
    EPConfig(RawData buffer) : buffer(buffer) {}

    /**
     * @brief 使用同一块 buffer 生成单方向 endpoint / Create a single-direction endpoint
     * from one shared buffer
     */
    EPConfig(RawData buffer, bool is_in)
        : buffer(buffer),
          direction_hint(is_in ? DirectionHint::InOnly : DirectionHint::OutOnly)
    {
    }
  };

  /**
   * @brief 构造 ESP32-S3 USB device core / Construct the ESP32-S3 USB device core
   *
   * @param ep_cfgs endpoint 构造配置列表 / Endpoint construction config list
   * @param packet_size EP0 包长配置 / EP0 packet-size configuration
   * @param vid USB 厂商 ID / USB vendor ID
   * @param pid USB 产品 ID / USB product ID
   * @param bcd 设备版本号（BCD）/ Device release number in BCD
   * @param lang_list 字符串语言包列表 / String language-pack list
   * @param configs 配置描述符项列表 / Configuration-descriptor item lists
   * @param uid 可选设备唯一标识数据 / Optional device-unique ID payload
   */
  ESP32USBDevice(
      const std::initializer_list<EPConfig> ep_cfgs,
      USB::DeviceDescriptor::PacketSize0 packet_size, uint16_t vid, uint16_t pid,
      uint16_t bcd,
      const std::initializer_list<const USB::DescriptorStrings::LanguagePack*> lang_list,
      const std::initializer_list<const std::initializer_list<USB::ConfigDescriptorItem*>>
          configs,
      ConstRawData uid = {nullptr, 0});

  /**
   * @brief 初始化 USB device core 状态 / Initialize the USB device-core state
   */
  void Init(bool in_isr) override;

  /**
   * @brief 去初始化 USB device core 状态 / Deinitialize the USB device-core state
   */
  void Deinit(bool in_isr) override;
  void OnControllerReset() override;
  /// 重新装填 EP0 的 SETUP 接收窗口 / Reload the EP0 SETUP receive window
  void OnControlReady() override;

  /**
   * @brief 在控制传输阶段写入设备地址 / Write the device address during control-transfer
   * sequencing
   */
  ErrorCode SetAddress(uint8_t address, USB::ControlContext context) override;

  /**
   * @brief 启动 USB device controller / Start the USB device controller
   */
  void Start(bool in_isr) override;

  /**
   * @brief 停止 USB device controller / Stop the USB device controller
   */
  void Stop(bool in_isr) override;

  /**
   * @brief 停止超时（核心按 1 ms 帧计数后调用）：设备离开总线，在毫秒节拍上复位控制器并
   *        重新连接，主机重新枚举
   *        Stop timeout (the core calls this after counting 1 ms frames): the device
   *        leaves the bus, the controller is reset on the millisecond tick and the device
   *        reconnects, so the host enumerates again
   */
  void OnStopTimeout(bool in_isr) override;

 private:
  friend class ESP32USBEndpoint;

  static constexpr uint8_t ENDPOINT_COUNT = 7;
  static constexpr uint8_t IN_ENDPOINT_LIMIT = 5;
  static constexpr uint32_t INTERRUPT_DISPATCH_GUARD = 64U;
  static constexpr size_t SETUP_PACKET_BYTES = 8U;
  static constexpr size_t SETUP_DMA_BUFFER_BYTES = 64U;

  /**
   * @brief EP0 setup 方向状态 / EP0 setup-direction state shared with the endpoint layer
   */
  struct ControlState
  {
    bool setup_direction_out =
        false;  ///< 最近一笔 setup 请求是否面向 OUT data stage / Whether the latest setup
                ///< requests an OUT data stage
  };

  /**
   * @brief 按方向索引的 endpoint 指针表 / Direction-indexed endpoint pointer table owned
   * by the device core
   */
  struct EndpointMap
  {
    USB::Endpoint* in[ENDPOINT_COUNT] =
        {};  ///< IN endpoint 指针表 / IN endpoint pointer table
    USB::Endpoint* out[ENDPOINT_COUNT] =
        {};  ///< OUT endpoint 指针表 / OUT endpoint pointer table
  };

  /**
   * @brief DWC2 FIFO 分配状态 / DWC2 FIFO allocation state tracked across endpoint
   * configuration
   */
  struct FifoState
  {
    uint16_t depth_words =
        0U;  ///< 硬件 FIFO 总深度（word）/ Total hardware FIFO depth in words
    uint16_t rx_words =
        0U;  ///< 当前 RX FIFO 配置深度（word）/ Current RX FIFO depth in words
    uint16_t tx_next_words =
        0U;  ///< 下一个 TX FIFO 起始偏移（word）/ Next TX FIFO start offset in words
    uint16_t tx_words[ENDPOINT_COUNT] =
        {};  ///< 各 endpoint 已分配 TX FIFO 深度（word）/ Allocated TX FIFO depth per
             ///< endpoint in words
    bool tx_bound[ENDPOINT_COUNT] = {};  ///< 各 endpoint 是否已绑定 TX FIFO / Whether
                                         ///< each endpoint already has a bound TX FIFO
    uint8_t allocated_in = 0U;  ///< 已占用的硬件 IN endpoint 个数 / Number of allocated
                                ///< hardware IN endpoints
  };

  /**
   * @brief 运行时资源与全局状态 / Runtime-owned resources and device-global state flags
   */
  struct RuntimeState
  {
    intr_handle_t intr_handle =
        nullptr;  ///< 注册的 USB 中断句柄 / Registered USB interrupt handle
    void* phy_handle = nullptr;  ///< USB PHY 句柄 / USB PHY handle
    bool phy_ready = false;  ///< PHY 是否已准备完成 / Whether the PHY has been prepared
    bool irq_ready =
        false;  ///< 中断是否已准备完成 / Whether the interrupt has been prepared
    bool started = false;  ///< 控制器是否处于启动态 / Whether the controller is started
    bool rom_usb_cleaned = false;  ///< ROM USB 默认状态是否已清理 / Whether the ROM USB
                                   ///< default state has been cleaned
  };

  /**
   * @brief USB 中断跳板函数 / USB interrupt trampoline
   */
  static void IsrEntry(void* arg);

  /**
   * @brief 确保内部 PHY 已就绪 / Ensure the internal PHY is ready
   */
  bool EnsurePhyReady();

  /**
   * @brief 确保 USB 中断已注册 / Ensure the USB interrupt is registered
   */
  bool EnsureInterruptReady();

  /**
   * @brief 清理 ROM 默认 USB 状态 / Clean the ROM-provided default USB state
   */
  void EnsureRomUsbCleaned();

  /**
   * @brief 初始化 DWC2 device core 寄存器 / Initialize the DWC2 device-core registers
   */
  void InitializeCore();

  /**
   * @brief 清空 TX FIFO 尺寸寄存器 / Clear the TX FIFO size registers
   */
  void ClearTxFifoRegisters();

  /**
   * @brief 冲刷 RX/TX FIFO / Flush the RX/TX FIFOs
   */
  bool FlushFifos();

  /**
   * @brief 重置 FIFO 分配账本 / Reset the FIFO-allocation bookkeeping state
   */
  void ResetFifoState();

  /**
   * @brief 重置 device-global 功能状态 / Reset the device-global functional state
   */
  void ResetDeviceState();

  /**
   * @brief 重置各 endpoint 的硬件侧状态 / Reset each endpoint's hardware-side state
   */
  void ResetEndpointHardwareState();

  /**
   * @brief 重新装填 EP0 setup 包接收计数并使能 EP0 OUT / Reload the EP0 setup-packet
   *        receive count and enable EP0 OUT
   * @param clear_nak 为 false 时 EP0 OUT 保持 NAK：只收 SETUP，不收 OUT 数据 / When
   *        false EP0 OUT keeps its NAK: it takes SETUP packets and no OUT data
   */
  void ReloadSetupPacketCount(bool clear_nak = true);

  /**
   * @brief 分发一批 USB 中断原因 / Dispatch one batch of USB interrupt causes
   */
  void HandleInterrupt();

  /**
   * @brief 处理一次总线复位 / Handle one USB bus-reset event
   */
  void HandleBusReset(bool in_isr);

  /**
   * @brief 处理一批 endpoint 中断 / Handle one batch of endpoint interrupts
   */
  void HandleEndpointInterrupt(bool in_isr, bool in_dir);

  /**
   * @brief 处理 RX FIFO level 事件 / Handle one RX-FIFO level event
   */
  void HandleRxFifoLevel();

  /**
   * @brief 清空控制传输方向状态 / Reset the control-transfer direction state
   */
  void ResetControlState();

  /**
   * @brief 从 setup 包中更新方向状态 / Update the direction state from one setup packet
   */
  void UpdateSetupState(const uint8_t* setup);

  /**
   * @brief 返回最近 setup 的 data stage 是否面向 OUT / Return whether the latest setup
   * uses an OUT data stage
   */
  bool LastSetupDirectionOut() const { return control_.setup_direction_out; }

  /**
   * @brief 返回当前是否启用 DMA 路径 / Return whether the DMA path is enabled
   */
  bool DmaEnabled() const { return true; }

  /**
   * @brief 为一个 IN endpoint 分配 TX FIFO / Allocate one TX FIFO for an IN endpoint
   */
  bool AllocateTxFifo(uint8_t ep_num, uint16_t packet_size, bool is_bulk,
                      uint16_t& fifo_words);

  /**
   * @brief 确保 RX FIFO 容量满足当前 endpoint 需求 / Ensure the RX FIFO is large enough
   * for the current endpoint need
   */
  bool EnsureRxFifo(uint16_t packet_size);

  /// 复位 core 寄存器（自清位的等待都有上界；超界返回 false）/ Reset the core registers
  /// (every wait on a self-clearing bit is bounded; false when a bound is exceeded)
  bool ResetCoreRegisters();
  /// 重建寄存器基线（复位之后；FIFO 冲刷超界返回 false）/ Rebuild the register baseline
  /// (after a reset; false when the FIFO flush exceeds its bound)
  bool RebuildCoreRegisters();
  /// 复位的各个步骤，Start() 的同步路径和停止超时后的复位链共用；步骤之间各等一个自清位。
  /// The steps of a reset, shared by the synchronous path of Start() and by the reset
  /// chain after a stop timeout; one self-clearing bit is awaited between two steps.
  /// 选择设备模式并发起 core 软复位 / Select device mode and request the core soft reset
  void BeginCoreReset();
  /// 软复位之后：时钟门控、VBUS 覆盖和 TX FIFO 尺寸寄存器 / After the soft reset: clock
  /// gating, VBUS override and the TX FIFO size registers
  void BeginBaseline();
  /// 发起全部 TX FIFO 的冲刷 / Request the flush of every TX FIFO
  void StartTxFlushAll();
  /// 发起 RX FIFO 的冲刷 / Request the RX FIFO flush
  void StartRxFlush();
  /// FIFO 冲刷之后：设备状态、中断标志和中断掩码 / After the FIFO flushes: device state,
  /// interrupt flags and interrupt masks
  void FinishBaseline();
  /// 确认所有等待中的停止（控制器复位路径）/ Confirm every waiting stop (the
  /// controller-reset path)
  void ForceStopAll(bool in_isr);
  /// 停止超时后控制器复位链的阶段。每个阶段等一个寄存器条件，由毫秒节拍检查；条件成立后
  /// 做下一步并等下一个条件 / Stage of the controller-reset chain after a stop timeout.
  /// Every stage waits for one register condition, checked on the millisecond tick; when
  /// it holds the next step is taken and the next condition awaited
  enum class ResetStage : uint8_t
  {
    IDLE,        ///< 没有复位在进行 / No reset in progress
    AHB_IDLE,    ///< 等 AHB 空闲，然后发起软复位 / AHB idle, then the soft reset
    CORE_RESET,  ///< 等软复位位自清 / The soft-reset bit clears itself
    RESET_IDLE,  ///< 软复位后等 AHB 空闲，然后冲刷 TX FIFO / AHB idle after the reset,
                 ///< then the TX FIFO flush
    TX_FLUSH,    ///< 等 TX FIFO 冲刷完成，然后冲刷 RX FIFO / TX flush done, then the RX
                 ///< FIFO flush
    RX_FLUSH     ///< 等 RX FIFO 冲刷完成，然后重建基线并重连 / RX flush done, then the
                 ///< baseline and the reconnect
  };
  /// 进入一个阶段并开始等它的条件；排不上时留在总线外 / Enter a stage and start waiting
  /// for its condition; stay off the bus when the wait cannot be armed
  void ArmResetStage(ResetStage stage);
  /// 复位链的条件：当前阶段等待的寄存器位 / Reset-chain condition: the register bit the
  /// current stage waits for
  static bool ResetCheck(void* arg);
  /// 复位链的完成：做当前阶段之后的一步，最后一个阶段重连 / Reset-chain completion: take
  /// the step after the current stage; the last stage reconnects
  static void ResetDone(void* arg, ErrorCode result);

  /// 复位确认的最长等待 / Longest wait for the reset to confirm
  static constexpr uint32_t RESET_STEP_TIMEOUT_MS = 20U;
  /// 停止超时后的控制器复位链（毫秒节拍驱动）/ Controller-reset chain after a stop
  /// timeout (driven by the millisecond tick)
  DriverAsyncCondition reset_wait_;
  /// 复位链当前的阶段，在硬件保护内读写 / Current stage of the reset chain, read and
  /// written inside the hardware guard
  ResetStage reset_stage_ = ResetStage::IDLE;

  portMUX_TYPE hardware_mux_ = portMUX_INITIALIZER_UNLOCKED;
  EndpointMap endpoint_map_ = {};  ///< Endpoint 所有权映射表 / Endpoint ownership map
  FifoState fifo_state_ =
      {};  ///< FIFO 尺寸与分配账本 / FIFO sizing and allocation bookkeeping
  RuntimeState runtime_ =
      {};  ///< 运行时资源与全局状态 / Runtime resource ownership and global flags
  alignas(SETUP_DMA_BUFFER_BYTES) uint8_t setup_packet_[SETUP_DMA_BUFFER_BYTES] =
      {};  ///< DMA 可见的共享 setup 包缓冲区 / Shared DMA-visible setup-packet buffer
  /// EP0 OUT 停止后的 DMA 落点（一个 EP0 最大包）：迟到的一包落在这里，不会写进已归还
  /// 的槽；DMA 地址寄存器要求按字对齐 / DMA target of EP0 OUT after a stop (one EP0
  /// maximum packet): a late packet lands here instead of in the returned slot; the DMA
  /// address register needs word alignment
  alignas(uint32_t) uint8_t ep0_out_scratch_[64] = {};
  ControlState control_ = {};  ///< 需跨 status completion 保留的 EP0 setup 状态 / EP0
                               ///< setup state that must survive until status completion
};

}  // namespace LibXR

#endif
