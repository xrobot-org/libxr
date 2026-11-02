#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

#include "double_buffer.hpp"
#include "driver/gpio.h"
#include "esp_def.hpp"
#include "esp_intr_alloc.h"
#include "esp_private/critical_section.h"
#include "flag.hpp"
#include "hal/uart_hal.h"
#include "hal/uart_types.h"
#include "serialized_service.hpp"
#include "soc/periph_defs.h"
#include "soc/soc_caps.h"
#include "uart.hpp"

#if SOC_GDMA_SUPPORTED && SOC_UHCI_SUPPORTED
#include "esp_private/gdma.h"
#include "esp_private/gdma_link.h"
#include "hal/uhci_hal.h"
#endif

namespace LibXR
{

class ESP32UART;

/**
 * @brief ESP32 UART 读端口 / ESP32 UART read port
 *
 * 当上层从软件 RX 队列成功出队后，该读端口会回调所属 UART 后端继续尝试排空
 * 硬件 FIFO。
 * After software dequeues bytes from the RX queue, this read port calls back
 * into the owning UART backend so the hardware FIFO can be drained again.
 */
class ESP32UARTReadPort : public ReadPort
{
 public:
  /**
   * @brief 构造读端口 / Construct the read port
   *
   * @param size RX 队列容量（字节） / RX queue capacity in bytes
   * @param owner 所属 UART 后端 / Owning UART backend
   */
  explicit ESP32UARTReadPort(size_t size, ESP32UART& owner)
      : ReadPort(size), owner_(owner)
  {
  }

 protected:
  /**
   * @brief 通知后端处理接收空间 / Notify the backend of RX space.
   * @param in_isr 当前是否在中断中 / Whether called in an ISR.
   */
  void OnReadQueueSpaceAvailable(bool in_isr) override;

 private:
  ESP32UART& owner_;  ///< 所属 UART 后端 / Owning UART backend
};

/**
 * @brief 支持 FIFO 和 GDMA 的 ESP UART 后端 / ESP UART backend with FIFO and GDMA.
 *
 * FIFO 按硬件实际接收量消费 WriteQueue；GDMA 将完整请求复制到发送半区后完成请求，
 * DMA 完成仅释放半区。接收通过 ReadQueue 发布，配置和中断进展由同一处理器串行执行。
 * FIFO consumes actual accepted prefixes; GDMA completes requests after copying them
 * into TX halves, with DMA completion only releasing storage. RX uses ReadQueue;
 * one service serializes configuration and interrupt progress.
 *
 * @note UART 使用普通中断，Flash 擦写导致缓存关闭时暂缓执行；持续接收可能溢出。
 *       UART interrupts are deferred while flash operations disable cache; sustained
 *       input may overflow hardware storage.
 * @note 接收 FIFO 满时保留数据等待软件空间；GDMA 则回收已完成描述符，丢弃放不下的尾部。
 *       FIFO RX retains bytes until software space is available; GDMA recycles completed
 *       descriptors and discards excess bytes that do not fit.
 */
class ESP32UART : public UART
{
  friend class ESP32UARTReadPort;

 public:
  static constexpr int PIN_NO_CHANGE =
      -1;  ///< 保持引脚映射不变 / Keep pin mapping unchanged.

  /**
   * @brief 构造并初始化 ESP UART / Construct and initialize an ESP UART.
   * @param uart_num UART 外设编号 / UART peripheral number.
   * @param tx_pin 发送 GPIO / TX GPIO.
   * @param rx_pin 接收 GPIO / RX GPIO.
   * @param rts_pin RTS GPIO，PIN_NO_CHANGE 表示不修改 / RTS GPIO, or PIN_NO_CHANGE.
   * @param cts_pin CTS GPIO，PIN_NO_CHANGE 表示不修改 / CTS GPIO, or PIN_NO_CHANGE.
   * @param rx_buffer_size 接收软件队列容量 / RX software queue capacity.
   * @param tx_buffer_size 发送软件队列和单个 DMA 半区容量 / TX queue and DMA half
   * capacity.
   * @param tx_queue_size 写请求队列容量，须大于零 / Positive write request capacity.
   * @param config 初始帧格式和波特率 / Initial framing and baud rate.
   * @param enable_dma 在芯片支持时使用 GDMA，否则使用 FIFO / Use GDMA if supported, else
   * FIFO.
   */
  ESP32UART(uart_port_t uart_num, int tx_pin, int rx_pin, int rts_pin = PIN_NO_CHANGE,
            int cts_pin = PIN_NO_CHANGE, size_t rx_buffer_size = 1024,
            size_t tx_buffer_size = 512, uint32_t tx_queue_size = 5,
            UART::Configuration config = {115200, UART::Parity::NO_PARITY, 8, 1},
            bool enable_dma = true);

  /**
   * @brief 提交帧格式和波特率配置 / Submit framing and baud configuration.
   * @param config 目标配置 / Requested configuration.
   * @param in_isr 当前是否在中断中 / Whether called in an ISR.
   * @return 接纳返回 OK，配置槽被占用返回 BUSY，参数不支持返回 ARG_ERR；
   *         硬件尚未初始化时返回 STATE_ERR。
   *         OK on admission, BUSY for an occupied slot, ARG_ERR for
   *         unsupported settings, STATE_ERR before hardware initialization.
   * @note GDMA 等已有半区传输结束后应用；FIFO 等软件队列和硬件 FIFO 空闲后应用。
   *       GDMA waits for claimed transfers; FIFO waits for software and hardware drain.
   * @note 初始化时选定的源时钟频率须保持不变 / Keep the selected source clock frequency
   * fixed.
   */
  ErrorCode SetConfig(UART::Configuration config, bool in_isr = false) override;

  /**
   * @brief 切换 UART 内部环回模式 / Toggle UART internal loopback.
   * @param enable 是否启用 / Whether to enable loopback.
   * @return 成功为 OK，未初始化为 STATE_ERR / OK on success, STATE_ERR before
   * initialization.
   */
  ErrorCode SetLoopback(bool enable);

  /**
   * @brief `WritePort` 使用的队列驱动 TX 入口 / Queue-driven TX entry used by
   * `WritePort`.
   */
  static void WriteFun(WritePort& port, bool in_isr);

 private:
  /**
   * @brief 分配可供 DMA 访问的发送存储 / Allocate DMA-accessible TX storage.
   */
  static uint8_t* AllocateTxStorage(size_t size);

  /**
   * @brief 将一个 UART 序号映射到对应的外设模块 / Map one UART index to its peripheral
   * module.
   */
  static ErrorCode ResolveUartPeriph(uart_port_t uart_num, periph_module_t& out);

  /**
   * @brief 将配置的数据位转换为 HAL 枚举 / Convert configured data bits into the HAL
   * enum.
   */
  static bool ResolveWordLength(uint8_t data_bits, uart_word_length_t& out);

  /**
   * @brief 将配置的停止位转换为 HAL 枚举 / Convert configured stop bits into the HAL
   * enum.
   */
  static bool ResolveStopBits(uint8_t stop_bits, uart_stop_bits_t& out);

  /**
   * @brief 将配置的校验位转换为 HAL 枚举 / Convert configured parity into the HAL enum.
   */
  static uart_parity_t ResolveParity(UART::Parity parity);

  /**
   * @brief 转发 UART 中断到对象实例 / Forward a UART interrupt to its instance.
   */
  static void UartIsrEntry(void* arg);

#if SOC_GDMA_SUPPORTED && SOC_UHCI_SUPPORTED
  /**
   * @brief GDMA TX 完成回调 / GDMA TX completion callback.
   */
  static bool DmaTxEofCallback(gdma_channel_handle_t dma_chan,
                               gdma_event_data_t* event_data, void* user_data);

  /**
   * @brief GDMA TX 描述符错误回调 / GDMA TX descriptor error callback.
   */
  static bool DmaTxDescrErrCallback(gdma_channel_handle_t dma_chan,
                                    gdma_event_data_t* event_data, void* user_data);

  /**
   * @brief GDMA RX 完成回调 / GDMA RX completion callback.
   */
  static bool DmaRxDoneCallback(gdma_channel_handle_t dma_chan,
                                gdma_event_data_t* event_data, void* user_data);

  /**
   * @brief GDMA RX 描述符错误回调 / GDMA RX descriptor error callback.
   */
  static bool DmaRxDescrErrCallback(gdma_channel_handle_t dma_chan,
                                    gdma_event_data_t* event_data, void* user_data);
#endif

  /**
   * @brief 初始化 UART 硬件和基础 HAL 状态 / Initialize UART hardware and base HAL state.
   */
  ErrorCode InitUartHardware();

  /**
   * @brief 配置选定的 GPIO 引脚 / Configure the selected GPIO pins.
   */
  ErrorCode ConfigurePins();

  /**
   * @brief 安装 UART 中断处理函数 / Install the UART interrupt handler.
   */
  ErrorCode InstallUartIsr();

  /**
   * @brief 配置 RX 中断阈值和掩码 / Program RX interrupt thresholds and masks.
   */
  void ConfigureRxInterruptPath();

#if SOC_GDMA_SUPPORTED && SOC_UHCI_SUPPORTED
  /**
   * @brief 初始化 UHCI 和 GDMA 收发资源 / Initialize UHCI and GDMA resources.
   */
  ErrorCode InitDmaBackend();

  /**
   * @brief 启动当前发送半区的 GDMA 传输 / Start GDMA for the active TX half.
   */
  bool StartDmaTx();

  /**
   * @brief 处理一次 RX DMA 完成事件 / Handle one RX DMA completion.
   */
  void HandleDmaRxDone(gdma_event_data_t* event_data, bool in_isr);

  /**
   * @brief RX DMA 出错后恢复环形队列 / Recover the RX DMA ring after an error.
   */
  void HandleDmaRxError(bool in_isr);

  /// 停止并复位接收 DMA / Stop and reset RX DMA.
  void StopDmaRx(bool in_isr);
  /// 复位描述符并归还 DMA / Reset descriptors and return them to DMA.
  void ResetDmaRxDescriptors(bool in_isr);
  /// 从描述符环头启动接收 / Start RX from the ring head.
  void StartDmaRx(bool in_isr);

  /**
   * @brief 复位发送 DMA 并释放已完成请求的存储 / Reset TX DMA and release settled request
   * storage.
   */
  void HandleDmaTxError(bool in_isr);

#endif

  /**
   * @brief 尝试启动排队的发送工作 / Try to start queued transmit work.
   */
  ErrorCode TryStartTx(bool in_isr);

  /**
   * @brief 将一个完整请求装入当前 DMA 半区 / Load a complete request into the active DMA
   * half.
   */
  bool LoadActiveTxFromQueue(bool in_isr);

  /**
   * @brief 将一个完整请求预装到待发送 DMA 半区 / Preload a complete request into the
   * pending DMA half.
   */
  bool LoadPendingTxFromQueue(bool in_isr);

  /**
   * @brief 将待发送半区切换为当前半区 / Promote the pending DMA half to active.
   */
  bool PromotePendingTxToActive();

  /**
   * @brief 启动当前半区传输 / Start the active buffer transfer.
   */
  bool StartActiveTransfer(bool in_isr);

  /**
   * @brief 启动当前传输并检查启动结果 / Start the active transfer and check its result.
   */
  bool StartAndReportActive(bool in_isr);

  /**
   * @brief 清除当前发送半区的记账 / Clear active TX buffer bookkeeping.
   */
  void ClearActiveTx();

  /**
   * @brief 将已提交请求的前缀写入 UART FIFO / Write a released request prefix into the
   * UART FIFO.
   */
  void FillTxFifo(bool in_isr);

  /**
   * @brief 将 RX 字节推入软件队列 / Push RX bytes into the software queue.
   */
  void PushRxBytes(const uint8_t* data, size_t size, bool in_isr);

  /**
   * @brief 从硬件 RX FIFO 中取出待处理字节 / Drain pending bytes from the hardware RX
   * FIFO.
   */
  void DrainRxFifo(bool in_isr);

  /**
   * @brief 分发待处理的 UART 中断原因 / Dispatch pending UART interrupt reasons.
   */
  void HandleUartInterrupt();

  /// 启用指定中断源 / Enable selected interrupt sources.
  void EnableUartInterrupt(uint32_t mask);
  /// 屏蔽指定中断源 / Mask selected interrupt sources.
  void DisableUartInterrupt(uint32_t mask);
  /// 清除指定中断状态 / Clear selected interrupt status.
  void ClearUartInterrupt(uint32_t mask);
  /// 屏蔽并清除指定中断 / Mask and clear selected interrupts.
  void DisableAndClearUartInterrupt(uint32_t mask);
  /// 读取并确认受保护的中断快照 / Capture and acknowledge protected IRQ status.
  uint32_t CaptureUartInterrupt(uint32_t mask);

  /**
   * @brief 释放当前发送存储并推进后续数据 / Release active TX storage and advance later
   * data.
   */
  void OnTxTransferDone(bool in_isr, ErrorCode result);

  static constexpr uint32_t EVENT_WRITE = 1U << 0U;
  static constexpr uint32_t EVENT_TX_DONE = 1U << 1U;
  static constexpr uint32_t EVENT_RX_WORK = 1U << 2U;
  static constexpr uint32_t EVENT_CONFIG = 1U << 3U;
  static constexpr uint32_t EVENT_TX_WORK = 1U << 4U;
  static constexpr uint32_t EVENT_TX_ERROR = 1U << 5U;
  static constexpr uint32_t EVENT_RX_ERROR = 1U << 6U;

  enum class ConfigState : uint8_t
  {
    EMPTY,
    RESERVED,
    PUBLISHED,
  };

  /// 串行处理收发、错误和配置进展 / Serialize I/O, errors, and configuration.
  void ServiceEvents(uint32_t events, bool in_isr);
  void InvokeService(uint32_t events, bool in_isr);
  /// 使用已验证参数配置硬件 / Apply validated settings to hardware.
  ErrorCode ApplyConfig(const UART::Configuration& config);
  /// 在当前模式的边界应用配置 / Apply configuration at the mode-specific boundary.
  bool TryApplyPublishedConfig(bool in_isr);

  uart_port_t uart_num_;  ///< UART 外设编号 / UART peripheral number.
  int tx_pin_;            ///< 发送 GPIO / TX GPIO or PIN_NO_CHANGE.
  int rx_pin_;            ///< 接收 GPIO / RX GPIO or PIN_NO_CHANGE.
  int rts_pin_;           ///< RTS GPIO / RTS GPIO or PIN_NO_CHANGE.
  int cts_pin_;           ///< CTS GPIO / CTS GPIO or PIN_NO_CHANGE.

  UART::Configuration config_;  ///< 已应用的帧格式 / Applied framing configuration.
  uint32_t uart_sclk_hz_ = 0U;  ///< 源时钟频率快照 / Source-clock frequency snapshot.

  uint8_t* rx_isr_buffer_ = nullptr;  ///< FIFO 接收临时缓冲 / FIFO RX staging buffer.
  size_t rx_isr_buffer_size_ = 0;     ///< 接收临时缓冲大小 / RX staging capacity.

  uint8_t* tx_storage_ = nullptr;  ///< 发送半区存储 / TX half-buffer storage.
  DoubleBuffer tx_dma_buffer_{};   ///< DMA 发送双缓冲 / DMA TX double buffer.
  size_t tx_active_length_ = 0U;   ///< 当前发送字节数 / Active TX byte count.
  bool tx_active_valid_ = false;  ///< 当前半区是否含有效数据 / Active half contains data.
  Flag::Plain tx_busy_;           ///< 发送硬件是否忙 / Whether TX hardware is busy.

  bool uart_hw_enabled_ = false;              ///< UART 是否已初始化 / UART initialized.
  uart_hal_context_t uart_hal_ = {};          ///< UART HAL 上下文 / UART HAL context.
  intr_handle_t uart_intr_handle_ = nullptr;  ///< UART 中断句柄 / UART interrupt handle.
  bool uart_isr_installed_ = false;           ///< 中断是否已注册 / ISR registered.
  bool dma_requested_ = true;                 ///< 构造时的 DMA 请求 / Requested DMA mode.

  /// 收发与配置的串行处理器 / Serialized I/O and configuration service.
  SerializedService service_{};
  /// 任务持有执行权时 FIFO 模式要发送：释放执行权后再打开 TXFIFO_EMPTY /
  /// FIFO mode has bytes to send while a task held the service: enable TXFIFO_EMPTY
  /// once the service is released.
  std::atomic<bool> tx_kick_{false};
  /// 待应用配置 / Pending configuration.
  UART::Configuration requested_config_{};
  /// 配置槽发布状态 / Configuration slot publication state.
  std::atomic<uint8_t> config_state_{static_cast<uint8_t>(ConfigState::EMPTY)};
  /// 仅保护中断寄存器访问 / Protects interrupt-register access only.
  DECLARE_CRIT_SECTION_LOCK_IN_STRUCT(irq_lock_)

  ESP32UARTReadPort _read_port;  ///< 接收端口 / Read port.
  WritePort _write_port;         ///< 发送端口 / Write port.

#if SOC_GDMA_SUPPORTED && SOC_UHCI_SUPPORTED
  bool dma_backend_enabled_ = false;  ///< GDMA 模式是否启用 / GDMA enabled.
  uhci_hal_context_t uhci_hal_ = {};  ///< UHCI HAL 上下文 / UHCI HAL context.
  gdma_channel_handle_t tx_dma_channel_ = nullptr;  ///< 发送 GDMA 通道 / TX GDMA channel.
  gdma_channel_handle_t rx_dma_channel_ = nullptr;  ///< 接收 GDMA 通道 / RX GDMA channel.
  uintptr_t tx_dma_head_addr_[2] = {0U, 0U};  ///< 两个发送链表头地址 / Two TX list heads.
  gdma_link_list_handle_t rx_dma_link_ = nullptr;  ///< 接收描述符环 / RX descriptor ring.
  uint8_t* rx_dma_storage_ = nullptr;              ///< 接收 DMA 存储 / RX DMA storage.
  size_t rx_dma_chunk_size_ = 0;                   ///< 单个接收节点大小 / RX node size.
  uint32_t rx_dma_node_index_ = 0;  ///< 最早待处理的接收节点 / Oldest RX node index.
#endif
};

}  // namespace LibXR
