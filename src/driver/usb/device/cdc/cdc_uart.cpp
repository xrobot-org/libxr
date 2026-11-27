#include "cdc_uart.hpp"

#include "libxr_mem.hpp"

using namespace LibXR::USB;
using LibXR::ErrorCode;
using LibXR::UART;
using LibXR::WritePort;

void CDCUartReadPort::OnReadQueueSpaceAvailable(bool in_isr)
{
  owner_.OnReceiveSpace(in_isr);
}

CDCUart::CDCUart(Endpoint::EPNumber data_in_ep_num, Endpoint::EPNumber data_out_ep_num,
                 Endpoint::EPNumber comm_ep_num, size_t rx_buffer_size,
                 size_t tx_buffer_size, size_t tx_queue_size,
                 const char* control_interface_string, const char* data_interface_string)
    : CDCBase(data_in_ep_num, data_out_ep_num, comm_ep_num, control_interface_string,
              data_interface_string),
      LibXR::UART(&read_port_cdc_, &write_port_cdc_),
      read_port_cdc_(rx_buffer_size, *this),
      write_port_cdc_(tx_queue_size, tx_buffer_size)
{
  write_port_cdc_ = WriteFun;  // NOLINT
}

ErrorCode CDCUart::SetConfig(UART::Configuration cfg, bool in_isr)
{
  if (cfg.baudrate == 0U || (cfg.stop_bits != 1U && cfg.stop_bits != 2U) ||
      (cfg.parity != UART::Parity::NO_PARITY && cfg.parity != UART::Parity::ODD &&
       cfg.parity != UART::Parity::EVEN) ||
      (cfg.data_bits != 5U && cfg.data_bits != 6U && cfg.data_bits != 7U &&
       cfg.data_bits != 8U && cfg.data_bits != 16U))
  {
    return ErrorCode::ARG_ERR;
  }
  const CDCLineCoding coding = ToLineCoding(cfg);
  config_sequence_.fetch_add(1U, std::memory_order_acq_rel);
  // 载荷以 release 写：读者看到新载荷时，最后一次序号检查一定拒绝旧快照。
  // The payload is stored with release: a reader that sees a newer payload makes its
  // final sequence check reject an older snapshot.
  config_baud_.store(coding.dwDTERate, std::memory_order_release);
  config_format_.store(static_cast<uint32_t>(coding.bCharFormat) |
                           (static_cast<uint32_t>(coding.bParityType) << 8U) |
                           (static_cast<uint32_t>(coding.bDataBits) << 16U),
                       std::memory_order_release);
  config_sequence_.fetch_add(1U, std::memory_order_release);
  RequestService(in_isr);
  return ErrorCode::OK;
}

ErrorCode CDCUart::BindEndpoints(EndpointPool& pool, uint8_t first_interface, bool in_isr)
{
  const ErrorCode result = CDCBase::BindEndpoints(pool, first_interface, in_isr);
  if (result != ErrorCode::OK)
  {
    return result;
  }
  rx_head_ = 0U;
  rx_count_ = 0U;
  rx_offset_ = 0U;
  rx_waiting_.store(0U, std::memory_order_relaxed);
  ArmRx(in_isr);
  // 绑定之前写入的数据现在发送。
  // Data written before the bind is sent now.
  PumpTx(in_isr);
  return ErrorCode::OK;
}

void CDCUart::UnbindEndpoints(EndpointPool& pool, bool in_isr)
{
  Endpoint* out = GetDataOutEndpoint();
  CDCBase::UnbindEndpoints(pool, in_isr);
  // Close 使这些槽作废，交回得 OK 并释放它们。
  // Close made these slots stale; handing them back returns OK and frees them.
  ASSERT(rx_count_ == 0U || out != nullptr);
  for (; rx_count_ != 0U; --rx_count_)
  {
    const ErrorCode result = out->ReleaseSlot(rx_slots_[rx_head_]);
    ASSERT(result == ErrorCode::OK);
    UNUSED(result);
    rx_head_ = static_cast<uint8_t>((rx_head_ + 1U) % Endpoint::MAX_SLOTS);
  }
  rx_offset_ = 0U;
}

void CDCUart::OnService(bool in_isr)
{
  ApplyPendingConfig(in_isr);
  CDCBase::OnService(in_isr);
  PumpTx(in_isr);
  DrainRx(in_isr);
}

void CDCUart::OnDataOutComplete(bool in_isr, const Endpoint::Completion& completion)
{
  if (completion.result != ErrorCode::OK)
  {
    // 失败或被 STALL 中止：槽已空闲。STALLED 时挂的接收排队到 ClearStall。
    // Failed, or aborted by a halt: the slot is free. A receive armed while STALLED
    // waits for ClearStall.
    ArmRx(in_isr);
    return;
  }
  ASSERT(rx_count_ < Endpoint::MAX_SLOTS);
  rx_slots_[(rx_head_ + rx_count_) % Endpoint::MAX_SLOTS] = completion.slot;
  ++rx_count_;
  DrainRx(in_isr);
}

void CDCUart::OnDataInComplete(bool in_isr, const Endpoint::Completion& completion)
{
  // 字节在拷进槽时已从写端口结算；失败或中止的槽里的字节丢弃。
  // The bytes were settled with the write port when copied into the slot; those of a
  // failed or aborted slot are dropped.
  UNUSED(completion);
  PumpTx(in_isr);
}

void CDCUart::WriteFun(WritePort& port, bool in_isr)
{
  auto* cdc = LibXR::ContainerOf(&port, &CDCUart::write_port_cdc_);
  cdc->RequestService(in_isr);
}

void CDCUart::PumpTx(bool in_isr)
{
  Endpoint* ep = GetDataInEndpoint();
  while (ep != nullptr && write_port_cdc_.Size() != 0U)
  {
    Endpoint::Slot slot;
    if (ep->AcquireSlot(slot) != ErrorCode::OK)
    {
      // 没有空闲槽：IN 的完成回调会再来。
      // No free slot: the IN completion comes back here.
      return;
    }
    // 连续的写入共用一个槽，直到槽满或写端口空。
    // Consecutive writes share one slot until it is full or the write port is empty.
    size_t size = 0U;
    while (size < slot.data.size_)
    {
      auto queue = write_port_cdc_.GetWriteQueue(in_isr);
      if (queue.Empty())
      {
        break;
      }
      auto* output = static_cast<uint8_t*>(slot.data.addr_) + size;
      const size_t taken = queue.PopWithWriter(
          slot.data.size_ - size,
          [output](const uint8_t* first, size_t first_size, const uint8_t* second,
                   size_t second_size) -> size_t
          {
            if (first_size != 0U)
            {
              LibXR::Memory::FastCopy(output, first, first_size);
            }
            if (second_size != 0U)
            {
              LibXR::Memory::FastCopy(output + first_size, second, second_size);
            }
            return first_size + second_size;
          });
      if (taken == 0U)
      {
        break;
      }
      size += taken;
    }
    if (size == 0U)
    {
      // 写入者还没发布请求；它发布时 WriteFun 会再请 owner。
      // The writer has not released its request yet; WriteFun asks the owner when it
      // does.
      (void)ep->ReleaseSlot(slot);
      return;
    }
    // 写端口已空时这是最后一段，满包结尾要补零长度包；否则后面的数据接着发。
    // With the write port empty this is the last piece and a full last packet needs the
    // closing zero-length packet; otherwise the following data goes right after it.
    const bool last = write_port_cdc_.Size() == 0U;
    const ErrorCode result = ep->Submit(slot, size, last, in_isr);
    // ABORTED：端点在借出之后被关闭，槽已空闲，这些字节丢弃。
    // ABORTED: the endpoint closed after the lend; the slot is free, the bytes dropped.
    ASSERT(result == ErrorCode::OK || result == ErrorCode::ABORTED);
    if (result != ErrorCode::OK)
    {
      return;
    }
  }
}

void CDCUart::ArmRx(bool in_isr)
{
  Endpoint* ep = GetDataOutEndpoint();
  if (ep == nullptr)
  {
    return;
  }
  // 每个请求一个包：CDC 是没有边界的字节流，更长的请求会把收到的字节压到填满为止。
  // One packet per request: CDC is a byte stream without boundaries, and a longer
  // request would hold received bytes until it fills.
  const size_t size = ep->MaxPacketSize();
  while (ep->Receive(size, in_isr) == ErrorCode::OK)
  {
  }
}

void CDCUart::DrainRx(bool in_isr)
{
  Endpoint* ep = GetDataOutEndpoint();
  if (ep == nullptr)
  {
    return;
  }
  while (rx_count_ != 0U)
  {
    const Endpoint::Slot& slot = rx_slots_[rx_head_];
    if (rx_offset_ < slot.data.size_)
    {
      // 先置位再看空间：期间腾出的空间一定会再请一次 owner。
      // Set before looking at the space: space freed meanwhile always asks the owner
      // again.
      (void)rx_waiting_.exchange(1U, std::memory_order_seq_cst);
      auto queue = read_port_cdc_.GetReadQueue(in_isr);
      const size_t room = queue.EmptySize();
      const size_t left = slot.data.size_ - rx_offset_;
      const size_t amount = room < left ? room : left;
      if (amount != 0U)
      {
        const ErrorCode result = queue.PushBatch(
            static_cast<const uint8_t*>(slot.data.addr_) + rx_offset_, amount);
        ASSERT(result == ErrorCode::OK);
        UNUSED(result);
        rx_offset_ += amount;
      }
      queue.Publish();
      if (rx_offset_ < slot.data.size_)
      {
        // 读端口满：留着这个槽，主机被 NAK。
        // Read port full: keep the slot, so the host is NAKed.
        return;
      }
      rx_waiting_.store(0U, std::memory_order_relaxed);
    }
    const ErrorCode result = ep->ReleaseSlot(slot);
    ASSERT(result == ErrorCode::OK);
    UNUSED(result);
    rx_head_ = static_cast<uint8_t>((rx_head_ + 1U) % Endpoint::MAX_SLOTS);
    --rx_count_;
    rx_offset_ = 0U;
    ArmRx(in_isr);
  }
}

void CDCUart::ApplyPendingConfig(bool in_isr)
{
  const uint32_t before = config_sequence_.load(std::memory_order_acquire);
  if (before == applied_config_sequence_ || (before & 1U) != 0U)
  {
    return;
  }
  const uint32_t baud = config_baud_.load(std::memory_order_acquire);
  const uint32_t format = config_format_.load(std::memory_order_acquire);
  if (config_sequence_.load(std::memory_order_acquire) != before)
  {
    // 写入中途：写者结束时会再请 owner。
    // Mid-write: the writer asks the owner again when it finishes.
    return;
  }
  CDCLineCoding& coding = GetLineCoding();
  coding.dwDTERate = baud;
  coding.bCharFormat = static_cast<uint8_t>(format);
  coding.bParityType = static_cast<uint8_t>(format >> 8U);
  coding.bDataBits = static_cast<uint8_t>(format >> 16U);
  applied_config_sequence_ = before;
  (void)SendSerialState(in_isr);
}

void CDCUart::OnReceiveSpace(bool in_isr)
{
  // 两边都用读-改-写：要么 DrainRx 看到腾出的空间，要么这里看到它的标志。
  // Both sides use a read-modify-write: either DrainRx sees the freed space or this sees
  // its flag.
  if (rx_waiting_.exchange(0U, std::memory_order_seq_cst) != 0U)
  {
    RequestService(in_isr);
  }
}
