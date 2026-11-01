#pragma once

#include <atomic>

#include "cdc_base.hpp"
#include "ep.hpp"
#include "libxr_def.hpp"
#include "libxr_rw.hpp"

namespace LibXR::USB
{

class CDCUart;

/**
 * @brief USB CDC 接收端口 / USB CDC receive port
 */
class CDCUartReadPort : public ReadPort
{
 public:
  /**
   * @brief 构造接收端口 / Construct the receive port
   * @param size 接收缓冲区大小 / Receive buffer size
   * @param owner 所属 CDC UART / Owning CDC UART
   */
  explicit CDCUartReadPort(uint32_t size, CDCUart& owner) : ReadPort(size), owner_(owner)
  {
  }

 protected:
  /**
   * @brief 在接收队列腾出空间后恢复接收 / Resume reception when queue space is available
   * @param in_isr 是否在中断中调用 / Whether called in an ISR
   */
  void OnReadQueueSpaceAvailable(bool in_isr) override;

 public:
  CDCUart& owner_;  ///< 所属 CDC UART / Owning CDC UART
};

/**
 * @brief USB CDC-ACM UART 适配器 / USB CDC-ACM UART adapter
 *
 * 发送：owner 取一个空闲槽，从写端口队头拷入一段数据后提交；以满包结束的数据在后面没有
 * 更多数据、也没有未完成的请求时补一个零长包。接收：收完的槽作为租约交给本类，数据搬进读端口后归还并重新挂接收；读端口放
 * 不下时保留租约，主机随即被
 * NAK。复位或切换配置时写端口里尚未取出的数据保留到下一次配置。
 *
 * Transmit: the owner acquires a free slot, copies a prefix of the write-port front into
 * it and submits it; data ending on a full packet is followed by a ZLP once nothing else
 * is queued or in flight.
 * Receive: a filled slot is leased to this class, moved into the read port and returned
 * for rearming; while the read port is full the lease is kept and the host is NAKed.
 * Data still queued in the write port across a reset or reconfiguration is sent after the
 * next configuration.
 */
class CDCUart : public CDCBase, public LibXR::UART
{
 public:
  using LibXR::UART::Read;
  using LibXR::UART::read_port_;
  using LibXR::UART::Write;
  using LibXR::UART::write_port_;

  /**
   * @brief 构造 CDC UART / Construct the CDC UART
   * @param data_in_ep_num 数据 IN 端点号 / Data IN endpoint number
   * @param data_out_ep_num 数据 OUT 端点号 / Data OUT endpoint number
   * @param comm_ep_num 通信端点号 / Communication endpoint number
   * @param rx_buffer_size 接收队列容量 / Receive queue capacity
   * @param tx_buffer_size 发送数据队列容量 / Transmit data queue capacity
   * @param tx_queue_size 发送请求队列容量 / Transmit request queue capacity
   * @param control_interface_string 控制接口名称 / Control interface name
   * @param data_interface_string 数据接口名称 / Data interface name
   */
  CDCUart(
      Endpoint::EPNumber data_in_ep_num, Endpoint::EPNumber data_out_ep_num,
      Endpoint::EPNumber comm_ep_num, size_t rx_buffer_size = 128,
      size_t tx_buffer_size = 128, size_t tx_queue_size = 5,
      const char* control_interface_string = CDCBase::DEFAULT_CONTROL_INTERFACE_STRING,
      const char* data_interface_string = CDCBase::DEFAULT_DATA_INTERFACE_STRING)
      : CDCBase(data_in_ep_num, data_out_ep_num, comm_ep_num, control_interface_string,
                data_interface_string),
        LibXR::UART(&read_port_cdc_, &write_port_cdc_),
        read_port_cdc_(rx_buffer_size, *this),
        write_port_cdc_(tx_queue_size, tx_buffer_size)
  {
    write_port_cdc_ = WriteFun;  // NOLINT
  }

  /**
   * @brief 设置 CDC 线路编码 / Set CDC line coding
   * @param cfg UART 配置 / UART configuration
   * @param in_isr 是否在中断中调用 / Whether called in an ISR
   * @return 配置结果 / Configuration result
   */
  ErrorCode SetConfig(UART::Configuration cfg, bool in_isr = false) override
  {
    if (cfg.baudrate == 0U || (cfg.stop_bits != 1U && cfg.stop_bits != 2U) ||
        (cfg.parity != UART::Parity::NO_PARITY && cfg.parity != UART::Parity::ODD &&
         cfg.parity != UART::Parity::EVEN) ||
        (cfg.data_bits != 5U && cfg.data_bits != 6U && cfg.data_bits != 7U &&
         cfg.data_bits != 8U && cfg.data_bits != 16U))
      return ErrorCode::ARG_ERR;
    config_sequence_.fetch_add(1U, std::memory_order_acq_rel);
    // Payload release/acquire also orders the odd sequence marker. Observing a
    // newer payload must make the final sequence check reject an older snapshot.
    config_baud_.store(cfg.baudrate, std::memory_order_release);
    const uint32_t parity = cfg.parity == UART::Parity::ODD    ? 1U
                            : cfg.parity == UART::Parity::EVEN ? 2U
                                                               : 0U;
    config_format_.store((cfg.stop_bits == 2U ? 2U : 0U) | (parity << 8U) |
                             (static_cast<uint32_t>(cfg.data_bits) << 16U),
                         std::memory_order_release);
    config_sequence_.fetch_add(1U, std::memory_order_release);
    RequestClassService(in_isr);
    return ErrorCode::OK;
  }

  void OnService(bool in_isr) override
  {
    const uint32_t before = config_sequence_.load(std::memory_order_acquire);
    if (before != applied_config_sequence_ && (before & 1U) == 0U)
    {
      const uint32_t baud = config_baud_.load(std::memory_order_acquire);
      const uint32_t format = config_format_.load(std::memory_order_acquire);
      if (config_sequence_.load(std::memory_order_acquire) == before)
      {
        auto& coding = GetLineCoding();
        coding.dwDTERate = baud;
        coding.bCharFormat = static_cast<uint8_t>(format);
        coding.bParityType = static_cast<uint8_t>(format >> 8U);
        coding.bDataBits = static_cast<uint8_t>(format >> 16U);
        applied_config_sequence_ = before;
        SendSerialState(in_isr);
      }
    }
    CDCBase::OnService(in_isr);
    PumpTx(in_isr);
    DrainRx(in_isr);
  }

  void OnDeviceEvent(bool in_isr, DeviceEvent event, uint8_t endpoint) override
  {
    if (event != DeviceEvent::ENDPOINT_RESUMED) return;
    if (auto* out = GetDataOutEndpoint(); out && endpoint == out->GetAddress())
      ArmRx(in_isr);
    if (auto* in = GetDataInEndpoint(); in && endpoint == in->GetAddress())
      PumpTx(in_isr);
  }

 protected:
  void BindEndpoints(EndpointPool& endpoint_pool, uint8_t interface_number,
                     bool in_isr) override
  {
    CDCBase::BindEndpoints(endpoint_pool, interface_number, in_isr);
    rx_head_ = 0;
    rx_count_ = 0;
    rx_offset_ = 0;
    tx_outstanding_ = 0;
    tx_tail_ = false;
    ArmRx(in_isr);
    // Data written before this configuration is sent now.
    RequestClassService(in_isr);
  }

  /**
   * @brief 解绑端点并归还收到的租约 / Unbind endpoints and return held leases
   * @param endpoint_pool 端点池 / Endpoint pool
   * @param in_isr 是否在中断中调用 / Whether called in an ISR
   */
  void UnbindEndpoints(EndpointPool& endpoint_pool, bool in_isr) override
  {
    Endpoint* out = GetDataOutEndpoint();
    CDCBase::UnbindEndpoints(endpoint_pool, in_isr);
    // Leases became stale at the unbind; returning them frees the slots.
    while (rx_count_ != 0U)
    {
      if (out) out->ReleaseSlot(rx_leases_[rx_head_], in_isr);
      rx_head_ ^= 1U;
      --rx_count_;
    }
    rx_offset_ = 0;
  }

  /**
   * @brief 请 owner 推进发送 / Ask the owner to advance transmission
   * @param port 写端口 / Write port
   * @param in_isr 是否在中断中调用 / Whether called in an ISR
   */
  static void WriteFun(WritePort& port, bool in_isr)
  {
    auto* cdc = LibXR::ContainerOf(&port, &CDCUart::write_port_cdc_);
    cdc->RequestClassService(in_isr);
  }

  void OnDataOutComplete(bool in_isr, const Endpoint::Completion& completion) override
  {
    if (completion.result == ErrorCode::OK && completion.slot.Valid())
    {
      ASSERT(rx_count_ < 2U);
      rx_leases_[(rx_head_ + rx_count_) & 1U] = completion.slot;
      ++rx_count_;
      DrainRx(in_isr);
    }
    else if (completion.result != ErrorCode::ABORTED)
    {
      ArmRx(in_isr);  // A failed receive frees its slot; arm it again.
    }
  }

  void OnDataInComplete(bool in_isr, const Endpoint::Completion& completion) override
  {
    // WriteQueue settlement already completed the bytes when they entered the slot.
    if (tx_outstanding_ != 0U) --tx_outstanding_;
    // Nothing reached the host for a failed or aborted request; it needs no ZLP.
    if (completion.result != ErrorCode::OK) tx_tail_ = false;
    PumpTx(in_isr);
  }

 private:
  /// Owner: fill one free slot from the write-port front and submit it.
  void PumpTx(bool in_isr)
  {
    Endpoint* ep = GetDataInEndpoint();
    if (!Inited() || ep == nullptr) return;
    if (write_port_cdc_.Size() == 0U)
    {
      // A transfer ending on a packet boundary is terminated once nothing follows it.
      if (tx_tail_ && tx_outstanding_ == 0U &&
          ep->SubmitZeroLength(in_isr) == ErrorCode::OK)
      {
        tx_tail_ = false;
        ++tx_outstanding_;
      }
      return;
    }
    Endpoint::Slot slot;
    if (ep->AcquireSlot(slot, in_isr) != ErrorCode::OK) return;
    // Consecutive writes share one slot until it is full or the write port is empty.
    size_t size = 0U;
    while (size < slot.data.size_)
    {
      auto queue = write_port_cdc_.GetWriteQueue(in_isr);
      if (queue.Empty()) break;
      auto* output = static_cast<uint8_t*>(slot.data.addr_) + size;
      const size_t taken = queue.PopWithWriter(
          slot.data.size_ - size,
          [output](const uint8_t* first, size_t first_size, const uint8_t* second,
                   size_t second_size) -> size_t
          {
            if (first_size) std::memcpy(output, first, first_size);
            if (second_size) std::memcpy(output + first_size, second, second_size);
            return first_size + second_size;
          });
      if (taken == 0U) break;
      size += taken;
    }
    if (size == 0U)
    {
      ep->ReleaseSlot(slot, in_isr);
      return;
    }
    // ABORTED: unbound meanwhile; the bytes already left the write port and are dropped.
    if (ep->Submit(slot, size, false, in_isr) == ErrorCode::OK)
    {
      ++tx_outstanding_;
      tx_tail_ = size % ep->MaxPacketSize() == 0U;
    }
    if (write_port_cdc_.Size() != 0U) RequestClassService(in_isr);
  }

  /// Owner: arm every free OUT slot, one packet each (see CDCBase::BindEndpoints).
  void ArmRx(bool in_isr)
  {
    Endpoint* ep = GetDataOutEndpoint();
    if (!Inited() || ep == nullptr) return;
    const size_t size = ep->MaxPacketSize();
    while (ep->Receive(size, in_isr) == ErrorCode::OK)
    {
    }
  }

  /// Owner: move leased data into the read port in arrival order.
  void DrainRx(bool in_isr)
  {
    Endpoint* ep = GetDataOutEndpoint();
    while (rx_count_ != 0U && ep != nullptr)
    {
      const Endpoint::Slot& lease = rx_leases_[rx_head_];
      if (rx_offset_ < lease.data.size_)
      {
        if (!read_port_cdc_.Readable()) return;
        // Set before trying, so space freed meanwhile still requests another pass.
        (void)rx_waiting_.exchange(1U, std::memory_order_seq_cst);
        auto queue = read_port_cdc_.GetReadQueue(in_isr);
        const size_t amount =
            LibXR::min(queue.EmptySize(), lease.data.size_ - rx_offset_);
        if (amount != 0U)
        {
          const auto result = queue.PushBatch(
              static_cast<const uint8_t*>(lease.data.addr_) + rx_offset_, amount);
          DEV_ASSERT_FROM_CALLBACK(result == ErrorCode::OK, in_isr);
          UNUSED(result);
          rx_offset_ += amount;
        }
        queue.Publish();
        if (rx_offset_ < lease.data.size_) return;  // Keep the lease: backpressure.
        rx_waiting_.store(0U, std::memory_order_relaxed);
      }
      const Endpoint::Slot done = lease;
      rx_head_ ^= 1U;
      --rx_count_;
      rx_offset_ = 0;
      (void)ep->ReturnSlot(done, in_isr);  // ABORTED only after an unbind.
    }
  }

  friend class CDCUartReadPort;
  void OnReceiveSpace(bool in_isr)
  {
    // Only a lease held back by a full read port needs the owner. Both sides use a
    // read-modify-write on the flag: either DrainRx sees the freed space or this sees
    // its flag.
    if (rx_waiting_.exchange(0U, std::memory_order_seq_cst) != 0U)
      RequestClassService(in_isr);
  }

  CDCUartReadPort read_port_cdc_;    ///< CDC 接收端口 / CDC receive port
  LibXR::WritePort write_port_cdc_;  ///< CDC 发送端口 / CDC transmit port
  std::atomic<uint32_t> config_sequence_{0}, config_baud_{115200},
      config_format_{8U << 16U};
  uint32_t applied_config_sequence_ = 0;
  Endpoint::Slot rx_leases_[2];  ///< 收到未搬完的租约，按到达顺序 / Leases in order
  uint8_t rx_head_ = 0;
  uint8_t rx_count_ = 0;
  std::atomic<uint32_t> rx_waiting_{0};  ///< 租约等读端口腾空间 / A lease waits for space
  size_t rx_offset_ = 0;  ///< 队首租约已搬走的字节 / Bytes moved from the front lease
  uint8_t tx_outstanding_ =
      0;  ///< 已提交未完成的 IN 请求 / Submitted, unfinished IN requests
  bool tx_tail_ =
      false;  ///< 最后一笔以满包结束，尚未补零长包 / Last one ended on a packet
};

inline void CDCUartReadPort::OnReadQueueSpaceAvailable(bool in_isr)
{
  owner_.OnReceiveSpace(in_isr);
}

}  // namespace LibXR::USB
