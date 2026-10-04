#pragma once

#include <cstddef>
#include <cstdint>
#include <initializer_list>

#include "esp_def.hpp"
#include "esp_intr_alloc.h"
#include "libxr_type.hpp"
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
class ESP32USBDevice : public USB::EndpointPool, public USB::DeviceCore
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
   * @param vid USB vendor ID
   * @param pid USB product ID
   * @param bcd Device release number in BCD
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

  /**
   * @brief 在控制传输阶段写入设备地址 / Write the device address during control-transfer
   * sequencing
   */
  ErrorCode SetAddress(uint8_t address, USB::DeviceCore::Context context) override;

  /**
   * @brief 启动 USB device controller / Start the USB device controller
   */
  void Start(bool in_isr) override;

  /**
   * @brief 停止 USB device controller / Stop the USB device controller
   */
  void Stop(bool in_isr) override;

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
  static void IRAM_ATTR IsrEntry(void* arg);

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
  void FlushFifos();

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
   * @brief 重新装填 EP0 setup 包接收计数 / Reload the EP0 setup-packet receive count
   */
  void ReloadSetupPacketCount();

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

  EndpointMap endpoint_map_ = {};  ///< Endpoint 所有权映射表 / Endpoint ownership map
  FifoState fifo_state_ =
      {};  ///< FIFO 尺寸与分配账本 / FIFO sizing and allocation bookkeeping
  RuntimeState runtime_ =
      {};  ///< 运行时资源与全局状态 / Runtime resource ownership and global flags
  alignas(SETUP_DMA_BUFFER_BYTES) uint8_t setup_packet_[SETUP_DMA_BUFFER_BYTES] =
      {};  ///< DMA 可见的共享 setup 包缓冲区 / Shared DMA-visible setup-packet buffer
  ControlState control_ = {};  ///< 需跨 status completion 保留的 EP0 setup 状态 / EP0
                               ///< setup state that must survive until status completion
};

}  // namespace LibXR

#endif
