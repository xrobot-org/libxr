#pragma once

#include "cdc_base.hpp"
#include "ep.hpp"
#include "flag.hpp"
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
  CDCUart& owner_;           ///< 所属 CDC UART / Owning CDC UART
  bool recv_pause_ = false;  ///< 接收暂停标志 / Receive pause flag
  ConstRawData pending_data_{nullptr,
                             0};  ///< 待入队的端点数据 / Endpoint data awaiting enqueue
};

/**
 * @brief USB CDC-ACM UART 适配器 / USB CDC-ACM UART adapter
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
   * @param in_isr 是否在中断中调用，本实现不使用 / ISR context, unused by this
   * implementation
   * @return 配置结果 / Configuration result
   */
  ErrorCode SetConfig(UART::Configuration cfg, bool in_isr = false) override
  {
    UNUSED(in_isr);
    auto& line_coding = GetLineCoding();

    switch (cfg.stop_bits)
    {
      case 1:
        line_coding.bCharFormat = 0;
        break;
      case 2:
        line_coding.bCharFormat = 2;
        break;
      default:
        return ErrorCode::ARG_ERR;
    }

    switch (cfg.parity)
    {
      case UART::Parity::NO_PARITY:
        line_coding.bParityType = 0;
        break;
      case UART::Parity::ODD:
        line_coding.bParityType = 1;
        break;
      case UART::Parity::EVEN:
        line_coding.bParityType = 2;
        break;
      default:
        return ErrorCode::ARG_ERR;
    }

    switch (cfg.data_bits)
    {
      case 5:
      case 6:
      case 7:
      case 8:
      case 16:
        line_coding.bDataBits = static_cast<uint8_t>(cfg.data_bits);
        break;
      default:
        return ErrorCode::ARG_ERR;
    }

    line_coding.dwDTERate = cfg.baudrate;
    SendSerialState();
    return ErrorCode::OK;
  }

  /**
   * @brief 回填暂存数据并重新启动 OUT 接收 / Enqueue saved data and restart OUT reception
   * @param in_isr 是否在中断中调用 / Whether called in an ISR
   * @return 是否成功启动接收 / Whether reception was started
   */
  bool TryRearmOut(bool in_isr)
  {
    auto* ep = GetDataOutEndpoint();
    if (ep == nullptr || ep->MaxPacketSize() == 0U || !read_port_cdc_.Readable())
    {
      return false;
    }

    if (read_port_cdc_.recv_pause_ && read_port_cdc_.pending_data_.size_ != 0U)
    {
      auto queue = read_port_cdc_.GetReadQueue(in_isr);
      if (queue.EmptySize() < read_port_cdc_.pending_data_.size_)
      {
        queue.Publish();
        return false;
      }

      [[maybe_unused]] const auto ans =
          queue.PushBatch(static_cast<const uint8_t*>(read_port_cdc_.pending_data_.addr_),
                          read_port_cdc_.pending_data_.size_);
      DEV_ASSERT_FROM_CALLBACK(ans == ErrorCode::OK, in_isr);
      read_port_cdc_.pending_data_ = {nullptr, 0};
      queue.Publish();
    }

    if (ep->GetState() != Endpoint::State::IDLE)
    {
      return false;
    }

    [[maybe_unused]] const auto ans = ep->Transfer(ep->MaxPacketSize());
    if (ans == ErrorCode::OK)
    {
      read_port_cdc_.recv_pause_ = false;
      return true;
    }
    return false;
  }

 protected:
  /**
   * @brief 解绑端点并清理接收暂存状态 / Unbind endpoints and clear saved receive state
   * @param endpoint_pool 端点池 / Endpoint pool
   * @param in_isr 是否在中断中调用 / Whether called in an ISR
   */
  void UnbindEndpoints(EndpointPool& endpoint_pool, bool in_isr) override
  {
    CDCBase::UnbindEndpoints(endpoint_pool, in_isr);
    need_write_zlp_ = false;
    read_port_cdc_.recv_pause_ = false;
    read_port_cdc_.pending_data_ = {nullptr, 0};
  }

  /**
   * @brief 推进发送队列 / Advance the transmit queue
   * @param port 写端口 / Write port
   * @param in_isr 是否在中断中调用 / Whether called in an ISR
   */
  static void WriteFun(WritePort& port, bool in_isr)
  {
    auto* cdc = LibXR::ContainerOf(&port, &CDCUart::write_port_cdc_);
    if (!cdc->in_write_isr_.IsSet())
    {
      cdc->ProgressTx(in_isr);
    }
  }

  /**
   * @brief 将 OUT 数据放入接收队列 / Enqueue completed OUT data
   * @param in_isr 是否在中断中调用 / Whether called in an ISR
   * @param data 收到的端点数据 / Received endpoint data
   */
  void OnDataOutComplete(bool in_isr, ConstRawData& data) override
  {
    if (data.size_ != 0U)
    {
      auto queue = read_port_cdc_.GetReadQueue(in_isr);
      if (queue.EmptySize() < data.size_)
      {
        read_port_cdc_.recv_pause_ = true;
        read_port_cdc_.pending_data_ = data;
        queue.Publish();
        return;
      }

      [[maybe_unused]] const auto ans =
          queue.PushBatch(static_cast<const uint8_t*>(data.addr_), data.size_);
      DEV_ASSERT_FROM_CALLBACK(ans == ErrorCode::OK, in_isr);
      queue.Publish();
    }
    (void)TryRearmOut(in_isr);
  }

  /**
   * @brief 在 IN 完成后续发数据或零长度包 / Send more data or a ZLP after IN completion
   * @param in_isr 是否在中断中调用 / Whether called in an ISR
   * @param data 已发送的端点数据，本实现不使用 / Sent endpoint data, unused here
   */
  void OnDataInComplete(bool in_isr, ConstRawData& data) override
  {
    UNUSED(data);
    Flag::ScopedRestore isr_flag(in_write_isr_);
    auto* ep = GetDataInEndpoint();
    if (ep == nullptr || !Inited())
    {
      return;
    }

    if (need_write_zlp_)
    {
      if (ep->GetActiveLength() == 0U && write_port_cdc_.Size() == 0U)
      {
        [[maybe_unused]] const auto ans = ep->TransferZLP();
        DEV_ASSERT_FROM_CALLBACK(ans == ErrorCode::OK, in_isr);
        need_write_zlp_ = false;
        return;
      }
      need_write_zlp_ = false;
    }
    ProgressTx(in_isr);
  }

 private:
  /**
   * @brief 启动已填充的 IN 缓冲区 / Start the filled IN buffer
   * @param ep 数据 IN 端点 / Data IN endpoint
   * @param in_isr 是否在中断中调用 / Whether called in an ISR
   */
  void StartTxBuffer(Endpoint& ep, bool in_isr)
  {
    const size_t LENGTH = ep.GetActiveLength();
    DEV_ASSERT_FROM_CALLBACK(LENGTH != 0U, in_isr);
    ep.SetActiveLength(0U);
    const auto ans = ep.Transfer(LENGTH);
    REQUIRE_FROM_CALLBACK(ans == ErrorCode::OK, in_isr);
    const size_t MPS = ep.MaxPacketSize();
    need_write_zlp_ = MPS != 0U && LENGTH % MPS == 0U;
  }

  /**
   * @brief 将队首数据复制到 IN 缓冲区 / Copy the front request into the IN buffer
   * @param ep 数据 IN 端点 / Data IN endpoint
   * @param in_isr 是否在中断中调用 / Whether called in an ISR
   * @note 完成回调前先发布缓冲区长度和传输状态。
   *       Publish buffer length and transfer state before the completion callback.
   */
  void FillTxBuffer(Endpoint& ep, bool in_isr)
  {
    auto queue = write_port_cdc_.GetWriteQueue(in_isr);
    if (queue.Empty())
    {
      return;
    }

    const auto buffer = ep.GetBuffer();
    ASSERT(buffer.addr_ != nullptr && buffer.size_ != 0U);
    const size_t LENGTH =
        queue.PopWithWriter(buffer.size_,
                            [buffer](const uint8_t* first, size_t first_size,
                                     const uint8_t* second, size_t second_size) -> size_t
                            {
                              auto* dst = static_cast<uint8_t*>(buffer.addr_);
                              if (first_size != 0U)
                              {
                                std::memcpy(dst, first, first_size);
                              }
                              if (second_size != 0U)
                              {
                                std::memcpy(dst + first_size, second, second_size);
                              }
                              return first_size + second_size;
                            });
    DEV_ASSERT_FROM_CALLBACK(LENGTH != 0U && LENGTH <= UINT16_MAX, in_isr);
    ep.SetActiveLength(static_cast<uint16_t>(LENGTH));
    need_write_zlp_ = false;
    if (ep.GetState() == Endpoint::State::IDLE)
    {
      StartTxBuffer(ep, in_isr);
    }
  }

  /**
   * @brief 续发已填充的数据并预填下一缓冲区 / Send prepared data and prefill the next
   * buffer
   * @param in_isr 是否在中断中调用 / Whether called in an ISR
   */
  void ProgressTx(bool in_isr)
  {
    auto* ep = GetDataInEndpoint();
    if (ep == nullptr || !Inited() || ep->GetState() != Endpoint::State::IDLE)
    {
      return;
    }

    if (ep->GetActiveLength() != 0U)
    {
      StartTxBuffer(*ep, in_isr);
    }
    else
    {
      FillTxBuffer(*ep, in_isr);
    }

    if (ep->GetState() == Endpoint::State::BUSY && ep->UseDoubleBuffer() &&
        ep->GetActiveLength() == 0U)
    {
      FillTxBuffer(*ep, in_isr);
    }
  }

  CDCUartReadPort read_port_cdc_;    ///< CDC 接收端口 / CDC receive port
  LibXR::WritePort write_port_cdc_;  ///< CDC 发送端口 / CDC transmit port
  Flag::Plain
      in_write_isr_;  ///< IN 完成回调执行标志 / IN completion callback active flag
  bool need_write_zlp_ = false;  ///< 待发送零长度包 / Pending zero-length packet
};

inline void CDCUartReadPort::OnReadQueueSpaceAvailable(bool in_isr)
{
  if (recv_pause_)
  {
    (void)owner_.TryRearmOut(in_isr);
  }
}

}  // namespace LibXR::USB
