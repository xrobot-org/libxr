#pragma once

#include <cstddef>
#include <cstdint>

#include "esp_def.hpp"
#include "esp_intr_alloc.h"
#include "esp_private/critical_section.h"
#include "serialized_service.hpp"
#include "soc/soc_caps.h"
#include "uart.hpp"

#if (SOC_USB_SERIAL_JTAG_SUPPORTED &&                                        \
     ((defined(CONFIG_IDF_TARGET_ESP32C3) && CONFIG_IDF_TARGET_ESP32C3) ||   \
      (defined(CONFIG_IDF_TARGET_ESP32C6) && CONFIG_IDF_TARGET_ESP32C6))) || \
    defined(__DOXYGEN__)

namespace LibXR
{

class ESP32CDCJtag;

/**
 * @brief 通知 CDC-JTAG 恢复接收的读端口 / Read port that resumes CDC-JTAG RX.
 */
class ESP32CDCJtagReadPort : public ReadPort
{
 public:
  /// 构造接收端口并关联后端 / Construct the RX port associated with its backend.
  explicit ESP32CDCJtagReadPort(size_t size, ESP32CDCJtag& owner)
      : ReadPort(size), owner_(owner)
  {
  }

 protected:
  /// 发布接收空间通知 / Publish RX space availability.
  void OnReadQueueSpaceAvailable(bool in_isr) override;

 private:
  /// 所属后端 / Associated backend.
  ESP32CDCJtag& owner_;
};

#if defined(CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG) && CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
static_assert(false,
              "ESP32CDCJtag conflicts with ESP-IDF primary USB Serial/JTAG console. "
              "Set CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=n or disable this backend.");
#endif

/**
 * @brief 直接使用硬件 FIFO 的 ESP USB Serial/JTAG 后端
 *        / ESP USB Serial/JTAG backend using the hardware FIFO directly.
 *
 * 发送字节进入硬件 FIFO 后从 WriteQueue 消费，接收通过 ReadQueue 发布。
 * 中断、写入和接收空间通知使用同一个串行处理器；Flash 缓存关闭期间中断会延后。
 * TX consumes WriteQueue bytes after FIFO acceptance; RX publishes via ReadQueue.
 * One service handles IRQ, write, and RX-space notifications. Interrupts are deferred
 * while flash cache is disabled.
 */
class ESP32CDCJtag : public UART
{
  friend class ESP32CDCJtagReadPort;

 public:
  /**
   * @brief 构造并初始化 USB Serial/JTAG 后端 / Construct the USB Serial/JTAG backend.
   * @param rx_buffer_size 接收队列容量，须大于零 / Positive RX queue capacity.
   * @param tx_buffer_size 发送队列容量，须大于零 / Positive TX queue capacity.
   * @param tx_queue_size 写请求队列容量，须大于零 / Positive request capacity.
   * @param config 初始配置，仅支持 8 位数据、无校验、1 位停止位 / Initial framing, 8N1
   * only.
   * @note 初始化前到达的 FIFO 数据不保留 / Does not preserve pre-initialization FIFO
   * data.
   */
  explicit ESP32CDCJtag(size_t rx_buffer_size = 1024, size_t tx_buffer_size = 512,
                        uint32_t tx_queue_size = 5,
                        UART::Configuration config = {115200, UART::Parity::NO_PARITY, 8,
                                                      1});

  /**
   * @brief 检查虚拟串口帧格式 / Check virtual UART framing.
   * @param config 目标帧格式 / Requested framing.
   * @param in_isr 调用上下文，本实现无需使用 / Caller context, unused by this
   * implementation.
   * @return 支持 8N1 时返回 OK，否则 ARG_ERR / OK for 8N1, ARG_ERR otherwise.
   * @note 波特率不改变 USB 传输速度 / Baud rate does not alter USB transfer speed.
   */
  ErrorCode SetConfig(UART::Configuration config, bool in_isr = false) override;

  /// 通知处理已提交写入 / Notify released writes.
  static void WriteFun(WritePort& port, bool in_isr);

 private:
  static constexpr uint32_t EVENT_WRITE = 1U << 0U;
  static constexpr uint32_t EVENT_TX_EMPTY = 1U << 1U;
  static constexpr uint32_t EVENT_RX_DATA = 1U << 2U;
  static constexpr uint32_t EVENT_RX_SPACE = 1U << 3U;

  /// 将中断转发到实例 / Forward an interrupt to its instance.
  static void IsrEntry(void* arg);
  /// 分配普通中断并初始化硬件 / Allocate a normal interrupt and initialize hardware.
  ErrorCode InitHardware();

  /// 处理已记录的收发进展 / Process recorded I/O progress.
  void ServiceEvents(uint32_t events, bool in_isr);
  /// 通知接收空间已释放 / Notify freed RX space.
  void ResumeRx(bool in_isr);
  /// 接收数据并按空闲空间重新启用中断 / Receive and rearm according to free space.
  void ServiceRx(ReadPort::ReadQueue& queue, bool in_isr);
  /// 只读取软件队列可容纳的数据 / Read only bytes that fit the software queue.
  void DrainRxToQueue(ReadPort::ReadQueue& queue, bool in_isr);
  /// 复制本次接收到的字节 / Copy received bytes.
  void PushRxBytes(ReadPort::ReadQueue& queue, const uint8_t* data, size_t size,
                   bool in_isr);

  /// 推进 FIFO 写入及满包结束通知 / Progress FIFO writes and full-packet termination.
  void ProgressTx(bool in_isr);
  /// 写入队头前缀，返回实际接收量 / Write a front prefix and return accepted bytes.
  size_t FillTxFifo(WritePort::WriteQueue& queue, bool in_isr);
  /// 启用发送空间通知 / Enable TX-space interrupts.
  void ArmTxEmptyInterrupt();
  /// 停用并清除发送空间通知 / Disable and clear TX-space interrupts.
  void DisarmTxEmptyInterrupt();
  /// 提交待发送短包或结束零长度包 / Flush a pending short packet or terminating ZLP.
  void FlushTxFifo();
  /// 先确认中断快照，再运行处理器 / Acknowledge the IRQ snapshot before service.
  void HandleInterrupt();

  /// 启用指定中断源 / Enable selected interrupt sources.
  void EnableInterrupt(uint32_t mask);
  /// 屏蔽指定中断源 / Mask selected interrupt sources.
  void DisableInterrupt(uint32_t mask);
  /// 清除指定中断状态 / Clear selected interrupt status.
  void ClearInterrupt(uint32_t mask);
  /// 屏蔽并清除指定中断 / Mask and clear selected interrupts.
  void DisableAndClearInterrupt(uint32_t mask);
  /// 读取并确认受保护的中断快照 / Capture and acknowledge protected IRQ status.
  uint32_t CaptureInterrupt(uint32_t mask);

  /// 中断句柄 / Interrupt handle.
  intr_handle_t intr_handle_ = nullptr;
  /// 硬件是否初始化 / Hardware initialized.
  bool hw_inited_ = false;
  /// 发送空间中断是否启用 / TX-space interrupt armed.
  bool tx_empty_interrupt_armed_ = false;
  /// 是否还需提交数据包或结束 ZLP / Pending packet flush or terminating ZLP.
  bool tx_flush_pending_ = false;

  /// 仅保护中断寄存器访问 / Protects interrupt-register access only.
  DECLARE_CRIT_SECTION_LOCK_IN_STRUCT(irq_lock_)
  /// 收发进展处理器 / I/O progress service.
  SerializedService service_{};
  /// 接收端口 / Read port.
  ESP32CDCJtagReadPort _read_port;
  /// 发送端口 / Write port.
  WritePort _write_port;
};

}  // namespace LibXR

#endif
