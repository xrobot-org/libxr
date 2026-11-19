#include "cdc_to_uart.hpp"

using namespace LibXR::USB;
using LibXR::ErrorCode;
using LibXR::ReadOperation;
using LibXR::UART;
using LibXR::WriteOperation;

CDCToUart::CDCToUart(Endpoint::EPNumber data_in_ep_num,
                     Endpoint::EPNumber data_out_ep_num, Endpoint::EPNumber comm_ep_num,
                     LibXR::UART& uart, size_t rx_buffer_size, size_t tx_buffer_size,
                     size_t tx_queue_size)
    : CDCUart(data_in_ep_num, data_out_ep_num, comm_ep_num, rx_buffer_size,
              tx_buffer_size, tx_queue_size),
      rx_buffer_(new uint8_t[rx_buffer_size], rx_buffer_size),
      tx_buffer_(new uint8_t[tx_buffer_size], tx_buffer_size),
      uart_(uart)
{
  // UART 的写队列至少要装下一次 CDC 读出的量。
  // The UART write queue must hold at least one CDC read.
  ASSERT(uart_.write_port_->Capacity() >= rx_buffer_size);

  // 1. CDC 读完成：把读到的写进 UART。回调都是 Guarded，把链压平。
  // 1. CDC read done: write it to the UART. All callbacks are guarded to flatten the
  //    chain.
  cb_read_cdc_ = Callback<ErrorCode>::CreateGuarded(
      [](bool in_isr, CDCToUart* self, ErrorCode)
      {
        const size_t size = LibXR::min(self->read_port_->Size(), self->rx_buffer_.size_);
        static ReadOperation op_read_cdc_noblock;
        ErrorCode ans =
            self->Read({self->rx_buffer_.addr_, size}, op_read_cdc_noblock, in_isr);
        DEV_ASSERT_FROM_CALLBACK(ans == ErrorCode::OK, in_isr);
        ans = self->uart_.Write({self->rx_buffer_.addr_, size}, self->op_write_uart_,
                                in_isr);
        DEV_ASSERT_FROM_CALLBACK(ans == ErrorCode::OK, in_isr);
        UNUSED(ans);
      },
      this);
  op_read_cdc_ = ReadOperation(cb_read_cdc_);

  // 2. UART 写完成：再读 CDC / 2. UART write done: read CDC again
  cb_uart_write_ = Callback<ErrorCode>::CreateGuarded(
      [](bool in_isr, CDCToUart* self, ErrorCode)
      {
        const ErrorCode ans = self->Read({nullptr, 0}, self->op_read_cdc_, in_isr);
        DEV_ASSERT_FROM_CALLBACK(ans == ErrorCode::OK, in_isr);
        UNUSED(ans);
      },
      this);
  op_write_uart_ = WriteOperation(cb_uart_write_);

  // 3. UART 读完成：写进 CDC / 3. UART read done: write it to CDC
  cb_uart_read_ = Callback<ErrorCode>::CreateGuarded(
      [](bool in_isr, CDCToUart* self, ErrorCode)
      {
        const size_t size =
            LibXR::min(self->uart_.read_port_->Size(), self->tx_buffer_.size_);
        static ReadOperation op_read_uart_noblock;
        ErrorCode ans = self->uart_.Read({self->tx_buffer_.addr_, size},
                                         op_read_uart_noblock, in_isr);
        DEV_ASSERT_FROM_CALLBACK(ans == ErrorCode::OK, in_isr);
        ans = self->Write({self->tx_buffer_.addr_, size}, self->op_write_cdc_, in_isr);
        DEV_ASSERT_FROM_CALLBACK(ans == ErrorCode::OK, in_isr);
        UNUSED(ans);
      },
      this);
  op_read_uart_ = ReadOperation(cb_uart_read_);

  // 4. CDC 写完成：再读 UART / 4. CDC write done: read the UART again
  cb_cdc_write_ = Callback<ErrorCode>::CreateGuarded(
      [](bool in_isr, CDCToUart* self, ErrorCode)
      {
        const ErrorCode ans = self->uart_.Read({nullptr, 0}, self->op_read_uart_, in_isr);
        DEV_ASSERT_FROM_CALLBACK(ans == ErrorCode::OK, in_isr);
        UNUSED(ans);
      },
      this);
  op_write_cdc_ = WriteOperation(cb_cdc_write_);

  // 各挂一次读，进入两条链 / One read each to enter both chains
  (void)Read({nullptr, 0}, op_read_cdc_, false);
  (void)uart_.Read({nullptr, 0}, op_read_uart_, false);
}

void CDCToUart::ApplyLineCoding(bool in_isr, CDCLineCoding& coding,
                                UART::Configuration& config)
{
  UART::Configuration candidates[5];
  size_t count = 0;
  auto add = [&](const UART::Configuration& candidate)
  {
    for (size_t i = 0; i < count; ++i)
    {
      if (SameConfiguration(candidates[i], candidate))
      {
        return;
      }
    }
    candidates[count++] = candidate;
  };

  UART::Configuration relaxed = config;
  add(relaxed);
  relaxed.stop_bits = 1U;
  add(relaxed);
  relaxed.parity = UART::Parity::NO_PARITY;
  add(relaxed);
  relaxed.data_bits = 8U;
  add(relaxed);
  add(ToUartConfiguration(GetLineCoding()));

  for (size_t i = 0; i < count; ++i)
  {
    if (uart_.SetConfig(candidates[i], in_isr) == ErrorCode::OK)
    {
      config = candidates[i];
      coding = ToLineCoding(config);
      return;
    }
  }
  coding = GetLineCoding();
  config = ToUartConfiguration(coding);
}

bool CDCToUart::SameConfiguration(const UART::Configuration& a,
                                  const UART::Configuration& b)
{
  return a.baudrate == b.baudrate && a.parity == b.parity && a.data_bits == b.data_bits &&
         a.stop_bits == b.stop_bits;
}
