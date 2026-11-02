#include <algorithm>

#include "esp_attr.h"
#include "esp_uart.hpp"
#include "soc/uart_periph.h"

namespace
{
// 非 DMA 路径处理的 FIFO RX 中断原因。
// FIFO RX interrupt reasons handled by the non-DMA path.
constexpr uint32_t UART_RX_INTR_MASK =
    UART_INTR_RXFIFO_FULL | UART_INTR_RXFIFO_TOUT | UART_INTR_RXFIFO_OVF;

// 非 DMA 路径处理的 FIFO TX 中断原因。
// FIFO TX interrupt reason handled by the non-DMA path.
constexpr uint32_t UART_TX_INTR_MASK = UART_INTR_TXFIFO_EMPTY;
constexpr uint32_t UART_TX_IDLE_INTR_MASK = UART_INTR_TX_DONE;
}  // namespace

namespace LibXR
{

// ISR 入口只负责把控制转发给对象实例。
// ISR entry only forwards control into the object instance.
void IRAM_ATTR ESP32UART::UartIsrEntry(void* arg)
{
  auto* self = static_cast<ESP32UART*>(arg);
  if (self != nullptr)
  {
    self->HandleUartInterrupt();
  }
}

ErrorCode ESP32UART::InstallUartIsr()
{
  if (uart_isr_installed_)
  {
    return ErrorCode::OK;
  }

  // RW 接口和用户回调可能位于 Flash，缓存关闭时由 SDK 暂缓处理中断。
  // RW methods and user callbacks may reside in flash; defer IRQs while cache is off.
  constexpr int UART_INTR_FLAGS = 0;

  const esp_err_t err = esp_intr_alloc(uart_periph_signal[uart_num_].irq, UART_INTR_FLAGS,
                                       UartIsrEntry, this, &uart_intr_handle_);
  if (err != ESP_OK)
  {
    return ErrorCode::INIT_ERR;
  }

  uart_isr_installed_ = true;
  return ErrorCode::OK;
}

// FIFO TX 路径会直接把字节从队列流式写入硬件 FIFO；与 DMA 模式不同，它不
// 会暂存第二块 payload。只有 UART 中断写 FIFO：任务（写入方取得服务时）只记下要
// 发送，释放执行权后由 InvokeService 打开 TXFIFO_EMPTY 中断。否则任务和中断可能在
// 两个核上轮流写 FIFO，后写的一方读到的 txfifo_cnt 可能还没反映另一核刚写入的
// 字节，多写的字节会让 FIFO 溢出。
// FIFO TX path streams bytes directly from the queue into the hardware FIFO.
// Unlike DMA mode, it does not stage a second payload block. Only the UART interrupt
// writes the FIFO: a task (the writer that won the service) only notes that bytes
// wait, and InvokeService enables TXFIFO_EMPTY once the service is released.
// Otherwise the task and the interrupt could take turns on two cores, and the later
// one could read a txfifo_cnt that does not yet count the bytes the other core just
// wrote, overfilling the FIFO.
void IRAM_ATTR ESP32UART::FillTxFifo(bool in_isr)
{
  if (!in_isr)
  {
    if (_write_port.Size() != 0U)
    {
      tx_kick_.store(true, std::memory_order_release);
    }
    return;
  }

  const uint32_t fifo_space = uart_hal_get_txfifo_len(&uart_hal_);
  if (fifo_space == 0U)
  {
    if (_write_port.Size() != 0U)
    {
      EnableUartInterrupt(UART_INTR_TXFIFO_EMPTY);
    }
    return;
  }

  size_t accepted = 0U;
  {
    auto queue = _write_port.GetWriteQueue(in_isr);
    if (queue.Empty())
    {
      DisableUartInterrupt(UART_INTR_TXFIFO_EMPTY);
      return;
    }

    accepted = queue.PopWithWriter(
        std::min<size_t>(fifo_space, queue.AvailableSize()),
        [this](const uint8_t* first, size_t first_size, const uint8_t* second,
               size_t second_size) -> size_t
        {
          size_t written = 0U;
          if (first_size != 0U)
          {
            uint32_t first_written = 0U;
            uart_hal_write_txfifo(&uart_hal_, first, static_cast<uint32_t>(first_size),
                                  &first_written);
            written = std::min<size_t>(first_written, first_size);
            if (written != first_size)
            {
              return written;
            }
          }
          if (second_size != 0U)
          {
            uint32_t second_written = 0U;
            uart_hal_write_txfifo(&uart_hal_, second, static_cast<uint32_t>(second_size),
                                  &second_written);
            written += std::min<size_t>(second_written, second_size);
          }
          return written;
        });
  }

  if (accepted == 0U || _write_port.Size() != 0U)
  {
    EnableUartInterrupt(UART_INTR_TXFIFO_EMPTY);
  }
  else
  {
    DisableUartInterrupt(UART_INTR_TXFIFO_EMPTY);
  }
}

// RX FIFO 清空采用尽力而为策略：尽量读取软件队列还能容纳的部分，然后停止，
// 不会粗暴重置整个硬件 FIFO。
// RX FIFO draining is best-effort: read as much as the software queue can
// absorb, then stop without resetting the whole hardware FIFO.
void IRAM_ATTR ESP32UART::DrainRxFifo(bool in_isr)
{
  auto queue = _read_port.GetReadQueue(in_isr);
  while (uart_hal_get_rxfifo_len(&uart_hal_) > 0U)
  {
    const size_t free_space = queue.EmptySize();
    if (free_space == 0U)
    {
      break;
    }

    const size_t chunk =
        std::min({free_space, rx_isr_buffer_size_,
                  static_cast<size_t>(uart_hal_get_rxfifo_len(&uart_hal_))});
    DEV_ASSERT_FROM_CALLBACK(chunk != 0U, in_isr);
    int read_size = static_cast<int>(chunk);
    uart_hal_read_rxfifo(&uart_hal_, rx_isr_buffer_, &read_size);
    DEV_ASSERT_FROM_CALLBACK(read_size > 0 && static_cast<size_t>(read_size) <= chunk,
                             in_isr);
    [[maybe_unused]] const auto push_batch_result =
        queue.PushBatch(rx_isr_buffer_, static_cast<size_t>(read_size));
    DEV_ASSERT_FROM_CALLBACK(push_batch_result == ErrorCode::OK, in_isr);
  }

  queue.Publish();
  if (queue.EmptySize() == 0U)
  {
    DisableUartInterrupt(UART_RX_INTR_MASK);
  }
  else
  {
    EnableUartInterrupt(UART_RX_INTR_MASK);
  }
}

// FIFO 后端会在离开 ISR 前排空当前所有待处理 UART 中断原因，避免同周期的
// RX/TX 事件被遗留。
// The FIFO backend drains all currently pending UART interrupt reasons before
// leaving the ISR to avoid orphaning a same-cycle RX/TX event.
void IRAM_ATTR ESP32UART::HandleUartInterrupt()
{
  const uint32_t handled_mask =
      UART_RX_INTR_MASK | UART_TX_INTR_MASK | UART_TX_IDLE_INTR_MASK;
  uint32_t handled = CaptureUartInterrupt(handled_mask);
  while (handled != 0U)
  {
    uint32_t events = 0U;
    if ((handled & UART_RX_INTR_MASK) != 0U)
    {
      events |= EVENT_RX_WORK;
    }
    if ((handled & (UART_TX_INTR_MASK | UART_TX_IDLE_INTR_MASK)) != 0U)
    {
      events |= EVENT_TX_WORK;
    }
    service_.Invoke(events, true, [this](uint32_t owner_events, bool owner_isr)
                    { ServiceEvents(owner_events, owner_isr); });
    handled = CaptureUartInterrupt(handled_mask);
  }
}

uint32_t IRAM_ATTR ESP32UART::CaptureUartInterrupt(uint32_t mask)
{
  esp_os_enter_critical_safe(&irq_lock_);
  const uint32_t handled = uart_hal_get_intsts_mask(&uart_hal_) & mask;
  if (handled != 0U)
  {
    uart_hal_disable_intr_mask(&uart_hal_, handled);
    // 在中断使能寄存器仍受保护时确认本次快照。
    // Acknowledge the snapshot while the enable register is still protected.
    uart_hal_clr_intsts_mask(&uart_hal_, handled);
  }
  esp_os_exit_critical_safe(&irq_lock_);
  return handled;
}

}  // namespace LibXR
