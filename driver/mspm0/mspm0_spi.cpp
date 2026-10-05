#include "mspm0_spi.hpp"

#include <cstring>

#include "timebase.hpp"

using namespace LibXR;

namespace
{
constexpr uint32_t MSPM0_SPI_DMA_INTERRUPT_MASK =
    DL_SPI_INTERRUPT_DMA_DONE_RX | DL_SPI_INTERRUPT_DMA_DONE_TX |
    DL_SPI_INTERRUPT_TX_UNDERFLOW | DL_SPI_INTERRUPT_PARITY_ERROR |
    DL_SPI_INTERRUPT_RX_OVERFLOW | DL_SPI_INTERRUPT_RX_TIMEOUT;
}  // namespace

MSPM0SPI* MSPM0SPI::instance_map_[MAX_SPI_INSTANCES] = {nullptr};

MSPM0SPI::MSPM0SPI(Resources res, RawData dma_rx_buffer, RawData dma_tx_buffer,
                   uint32_t dma_enable_min_size, SPI::Configuration config)
    : SPI(dma_rx_buffer, dma_tx_buffer),
      res_(res),
      dma_enable_min_size_(dma_enable_min_size)
{
  ASSERT(res_.instance != nullptr);
  ASSERT(res_.clock_freq > 0);
  ASSERT(res_.index < MAX_SPI_INSTANCES);
  ASSERT(instance_map_[res_.index] == nullptr);
  ASSERT(dma_rx_buffer.addr_ != nullptr);
  ASSERT(dma_tx_buffer.addr_ != nullptr);
  ASSERT(dma_rx_buffer.size_ > 0);
  ASSERT(dma_tx_buffer.size_ > 0);

  instance_map_[res_.index] = this;

  NVIC_ClearPendingIRQ(res_.irqn);
  NVIC_EnableIRQ(res_.irqn);

  const ErrorCode SET_CFG_ANS = SetConfig(config);
  ASSERT(SET_CFG_ANS == ErrorCode::OK);
}

ErrorCode MSPM0SPI::SetConfig(SPI::Configuration config)
{
  DL_SPI_FRAME_FORMAT frame_format = DL_SPI_FRAME_FORMAT_MOTO4_POL0_PHA0;
  if (config.clock_polarity == ClockPolarity::LOW)
  {
    frame_format = (config.clock_phase == ClockPhase::EDGE_1)
                       ? DL_SPI_FRAME_FORMAT_MOTO4_POL0_PHA0
                       : DL_SPI_FRAME_FORMAT_MOTO4_POL0_PHA1;
  }
  else
  {
    frame_format = (config.clock_phase == ClockPhase::EDGE_1)
                       ? DL_SPI_FRAME_FORMAT_MOTO4_POL1_PHA0
                       : DL_SPI_FRAME_FORMAT_MOTO4_POL1_PHA1;
  }

  const uint32_t DIV = SPI::PrescalerToDiv(config.prescaler);
  if (DIV < 2 || DIV > 512 || (DIV & 0x1) != 0)
  {
    return ErrorCode::NOT_SUPPORT;
  }

  const uint32_t SCR = (DIV >> 1) - 1;

  DL_SPI_disable(res_.instance);
  DL_SPI_setFrameFormat(res_.instance, frame_format);
  DL_SPI_setBitRateSerialClockDivider(res_.instance, SCR);
  DL_SPI_enable(res_.instance);

  GetConfig() = config;
  return ErrorCode::OK;
}

uint32_t MSPM0SPI::GetMaxBusSpeed() const { return res_.clock_freq; }

SPI::Prescaler MSPM0SPI::GetMaxPrescaler() const { return SPI::Prescaler::DIV_512; }

ErrorCode MSPM0SPI::PollingTransfer(uint8_t* rx, const uint8_t* tx, uint32_t len)
{
  constexpr uint32_t POLLING_TIMEOUT_US = 20000U;
  constexpr uint32_t POLLING_FALLBACK_SPIN_BUDGET = 1000000U;

  if (len == 0)
  {
    return ErrorCode::OK;
  }

  const uint64_t START_US = static_cast<uint64_t>(Timebase::GetMicroseconds());

  uint32_t spin_budget = POLLING_FALLBACK_SPIN_BUDGET;
  auto polling_timed_out = [&]() -> bool
  {
    const uint64_t NOW_US = static_cast<uint64_t>(Timebase::GetMicroseconds());
    if ((NOW_US - START_US) >= POLLING_TIMEOUT_US)
    {
      return true;
    }
    if (spin_budget > 0U)
    {
      --spin_budget;
    }
    return false;
  };

  for (uint32_t i = 0; i < len; ++i)
  {
    while (DL_SPI_isTXFIFOFull(res_.instance))
    {
      if (polling_timed_out())
      {
        return ErrorCode::TIMEOUT;
      }
    }
    const uint8_t TX_BYTE = (tx == nullptr) ? 0 : tx[i];
    DL_SPI_transmitData8(res_.instance, TX_BYTE);

    uint8_t rx_byte = 0;
    while (!DL_SPI_receiveDataCheck8(res_.instance, &rx_byte))
    {
      if (polling_timed_out())
      {
        return ErrorCode::TIMEOUT;
      }
    }

    if (rx != nullptr)
    {
      rx[i] = rx_byte;
    }
  }

  while (DL_SPI_isBusy(res_.instance))
  {
    if (polling_timed_out())
    {
      return ErrorCode::TIMEOUT;
    }
  }

  return ErrorCode::OK;
}

bool MSPM0SPI::DmaBusy() const
{
  if (busy_)
  {
    return true;
  }

  return DL_DMA_isChannelEnabled(DMA, res_.dma_rx_channel) ||
         DL_DMA_isChannelEnabled(DMA, res_.dma_tx_channel);
}

void MSPM0SPI::StartDmaDuplex(uint32_t count)
{
  masked_interrupts_for_tx_only_ = 0;

  RawData rx = GetRxBuffer();
  RawData tx = GetTxBuffer();

  DL_DMA_setSrcAddr(DMA, res_.dma_rx_channel,
                    reinterpret_cast<uint32_t>(&res_.instance->RXDATA));
  DL_DMA_setDestAddr(DMA, res_.dma_rx_channel, reinterpret_cast<uint32_t>(rx.addr_));
  DL_DMA_setTransferSize(DMA, res_.dma_rx_channel, count);

  DL_DMA_setSrcAddr(DMA, res_.dma_tx_channel, reinterpret_cast<uint32_t>(tx.addr_));
  DL_DMA_setDestAddr(DMA, res_.dma_tx_channel,
                     reinterpret_cast<uint32_t>(&res_.instance->TXDATA));
  DL_DMA_setTransferSize(DMA, res_.dma_tx_channel, count);

  DL_DMA_enableChannel(DMA, res_.dma_rx_channel);
  DL_DMA_enableChannel(DMA, res_.dma_tx_channel);
}

void MSPM0SPI::StartDmaRxOnly(uint32_t offset, uint32_t count)
{
  RawData rx = GetRxBuffer();

  DEV_ASSERT(offset < rx.size_);
  DEV_ASSERT(count > 0U);
  DEV_ASSERT(count <= RX_ONLY_REPEAT_TX_MAX_FRAMES);
  DEV_ASSERT((offset + count) <= rx.size_);

  masked_interrupts_for_tx_only_ = 0;

  auto* rx_bytes = static_cast<uint8_t*>(rx.addr_);

  DL_DMA_disableChannel(DMA, res_.dma_tx_channel);
  DL_DMA_disableChannel(DMA, res_.dma_rx_channel);
  DL_DMA_setSrcAddr(DMA, res_.dma_rx_channel,
                    reinterpret_cast<uint32_t>(&res_.instance->RXDATA));
  DL_DMA_setDestAddr(DMA, res_.dma_rx_channel,
                     reinterpret_cast<uint32_t>(rx_bytes + offset));
  DL_DMA_setTransferSize(DMA, res_.dma_rx_channel, count);
  DL_DMA_enableChannel(DMA, res_.dma_rx_channel);

  // RX-only DMA 仍需要主机时钟，使用硬件重复 dummy TX 提供时钟 / RX-only DMA
  // still needs controller clocks; use repeated dummy TX in hardware so read-only
  // transfers can avoid occupying the TX DMA channel.
  DL_SPI_setRepeatTransmit(res_.instance, static_cast<uint8_t>(count - 1U));
  DL_SPI_transmitData8(res_.instance, 0U);
}

void MSPM0SPI::StartDmaTxOnly(uint32_t count)
{
  constexpr uint32_t TX_ONLY_MASKED_INTERRUPTS = DL_SPI_INTERRUPT_DMA_DONE_RX |
                                                 DL_SPI_INTERRUPT_RX_OVERFLOW |
                                                 DL_SPI_INTERRUPT_RX_TIMEOUT;

  masked_interrupts_for_tx_only_ =
      DL_SPI_getEnabledInterrupts(res_.instance, TX_ONLY_MASKED_INTERRUPTS);
  if (masked_interrupts_for_tx_only_ != 0U)
  {
    DL_SPI_disableInterrupt(res_.instance, masked_interrupts_for_tx_only_);
  }

  RawData tx = GetTxBuffer();

  DL_DMA_disableChannel(DMA, res_.dma_rx_channel);
  DL_DMA_setSrcAddr(DMA, res_.dma_tx_channel, reinterpret_cast<uint32_t>(tx.addr_));
  DL_DMA_setDestAddr(DMA, res_.dma_tx_channel,
                     reinterpret_cast<uint32_t>(&res_.instance->TXDATA));
  DL_DMA_setTransferSize(DMA, res_.dma_tx_channel, count);
  DL_DMA_enableChannel(DMA, res_.dma_tx_channel);
}

void MSPM0SPI::StopDma()
{
  DL_DMA_disableChannel(DMA, res_.dma_tx_channel);
  DL_DMA_disableChannel(DMA, res_.dma_rx_channel);

  if (dma_mode_ == DmaMode::RX_ONLY)
  {
    DL_SPI_setRepeatTransmit(res_.instance, 0U);
  }

  rx_only_offset_ = 0U;
  rx_only_remaining_ = 0U;

  if (masked_interrupts_for_tx_only_ != 0U)
  {
    DL_SPI_enableInterrupt(res_.instance, masked_interrupts_for_tx_only_);
    masked_interrupts_for_tx_only_ = 0U;
  }
}

ErrorCode MSPM0SPI::CompleteDmaOperation(OperationRW& op, bool in_isr)
{
  op.MarkAsRunning();
  if (op.type != OperationRW::OperationType::BLOCK)
  {
    return ErrorCode::OK;
  }

  ASSERT(!in_isr);
  const ErrorCode WAIT_ANS = op.data.sem_info.sem->Wait(op.data.sem_info.timeout);
  if (WAIT_ANS == ErrorCode::TIMEOUT)
  {
    const bool IRQ_WAS_ENABLED = (NVIC_GetEnableIRQ(res_.irqn) != 0U);
    if (IRQ_WAS_ENABLED)
    {
      NVIC_DisableIRQ(res_.irqn);
    }

    StopDma();
    DL_SPI_clearInterruptStatus(res_.instance, MSPM0_SPI_DMA_INTERRUPT_MASK);
    NVIC_ClearPendingIRQ(res_.irqn);

    busy_ = false;
    dma_mode_ = DmaMode::DUPLEX;
    dma_result_ = ErrorCode::TIMEOUT;
    rw_op_ = OperationRW();
    // Drain any timeout-race semaphore credits posted right before IRQ disable,
    // so the next BLOCK transfer cannot consume a stale completion signal.
    while (op.data.sem_info.sem->Wait(0U) == ErrorCode::OK)
    {
    }

    if (IRQ_WAS_ENABLED)
    {
      NVIC_EnableIRQ(res_.irqn);
    }
    return ErrorCode::TIMEOUT;
  }

  if (WAIT_ANS != ErrorCode::OK)
  {
    return WAIT_ANS;
  }

  return dma_result_;
}

ErrorCode MSPM0SPI::ReadAndWrite(RawData read_data, ConstRawData write_data,
                                 OperationRW& op, bool in_isr)
{
  const uint32_t NEED = static_cast<uint32_t>(max(read_data.size_, write_data.size_));
  const bool IS_READ_ONLY = (write_data.size_ == 0U) && (read_data.size_ > 0U);

  if (NEED == 0)
  {
    if (op.type != OperationRW::OperationType::BLOCK)
    {
      op.UpdateStatus(in_isr, ErrorCode::OK);
    }
    return ErrorCode::OK;
  }

  if (DmaBusy())
  {
    return ErrorCode::BUSY;
  }

  RawData rx = GetRxBuffer();
  RawData tx = GetTxBuffer();

  if (rx.size_ < NEED)
  {
    return ErrorCode::SIZE_ERR;
  }
  if (!IS_READ_ONLY && tx.size_ < NEED)
  {
    return ErrorCode::SIZE_ERR;
  }

  uint8_t* tx_bytes = nullptr;
  if (!IS_READ_ONLY)
  {
    tx_bytes = static_cast<uint8_t*>(tx.addr_);
    if (write_data.size_ > 0)
    {
      Memory::FastCopy(tx_bytes, write_data.addr_, write_data.size_);
    }
    if (write_data.size_ < NEED)
    {
      Memory::FastSet(tx_bytes + write_data.size_, 0, NEED - write_data.size_);
    }
  }

  if (NEED > dma_enable_min_size_)
  {
    DL_SPI_clearInterruptStatus(res_.instance, MSPM0_SPI_DMA_INTERRUPT_MASK);
    NVIC_ClearPendingIRQ(res_.irqn);

    mem_read_ = false;
    read_buff_ = read_data;
    rw_op_ = op;
    dma_result_ = ErrorCode::PENDING;
    busy_ = true;

    if (IS_READ_ONLY)
    {
      dma_mode_ = DmaMode::RX_ONLY;
      const uint32_t FIRST_CHUNK = min(NEED, RX_ONLY_REPEAT_TX_MAX_FRAMES);
      rx_only_offset_ = FIRST_CHUNK;
      rx_only_remaining_ = NEED - FIRST_CHUNK;
      StartDmaRxOnly(0U, FIRST_CHUNK);
    }
    else
    {
      dma_mode_ = DmaMode::DUPLEX;
      StartDmaDuplex(NEED);
    }
    return CompleteDmaOperation(op, in_isr);
  }

  ErrorCode ans = PollingTransfer(static_cast<uint8_t*>(rx.addr_), tx_bytes, NEED);

  if (ans == ErrorCode::OK && read_data.size_ > 0)
  {
    Memory::FastCopy(read_data.addr_, rx.addr_, read_data.size_);
  }

  SwitchBuffer();

  if (op.type != OperationRW::OperationType::BLOCK)
  {
    op.UpdateStatus(in_isr, ans);
  }

  return ans;
}

ErrorCode MSPM0SPI::Transfer(size_t size, OperationRW& op, bool in_isr)
{
  if (size == 0)
  {
    if (op.type != OperationRW::OperationType::BLOCK)
    {
      op.UpdateStatus(in_isr, ErrorCode::OK);
    }
    return ErrorCode::OK;
  }

  if (DmaBusy())
  {
    return ErrorCode::BUSY;
  }

  RawData rx = GetRxBuffer();
  RawData tx = GetTxBuffer();

  ASSERT(rx.size_ >= size);
  ASSERT(tx.size_ >= size);

  if (size > dma_enable_min_size_)
  {
    DL_SPI_clearInterruptStatus(res_.instance, MSPM0_SPI_DMA_INTERRUPT_MASK);
    NVIC_ClearPendingIRQ(res_.irqn);

    mem_read_ = false;
    dma_mode_ = DmaMode::DUPLEX;
    read_buff_ = {nullptr, 0};
    rw_op_ = op;
    dma_result_ = ErrorCode::PENDING;
    busy_ = true;

    StartDmaDuplex(static_cast<uint32_t>(size));
    return CompleteDmaOperation(op, in_isr);
  }

  ErrorCode ans =
      PollingTransfer(static_cast<uint8_t*>(rx.addr_),
                      static_cast<const uint8_t*>(tx.addr_), static_cast<uint32_t>(size));

  SwitchBuffer();

  if (op.type != OperationRW::OperationType::BLOCK)
  {
    op.UpdateStatus(in_isr, ans);
  }

  return ans;
}

ErrorCode MSPM0SPI::MemRead(uint16_t reg, RawData read_data, OperationRW& op, bool in_isr)
{
  const uint32_t NEED_READ = static_cast<uint32_t>(read_data.size_);
  if (NEED_READ == 0)
  {
    if (op.type != OperationRW::OperationType::BLOCK)
    {
      op.UpdateStatus(in_isr, ErrorCode::OK);
    }
    return ErrorCode::OK;
  }

  if (DmaBusy())
  {
    return ErrorCode::BUSY;
  }

  RawData rx = GetRxBuffer();
  RawData tx = GetTxBuffer();

  ASSERT(rx.size_ >= (NEED_READ + 1));
  ASSERT(tx.size_ >= (NEED_READ + 1));

  auto* tx_bytes = static_cast<uint8_t*>(tx.addr_);
  tx_bytes[0] = static_cast<uint8_t>(reg | 0x80);
  Memory::FastSet(tx_bytes + 1, 0, NEED_READ);

  const uint32_t TOTAL = NEED_READ + 1;

  if (TOTAL > dma_enable_min_size_)
  {
    DL_SPI_clearInterruptStatus(res_.instance, MSPM0_SPI_DMA_INTERRUPT_MASK);
    NVIC_ClearPendingIRQ(res_.irqn);

    mem_read_ = true;
    dma_mode_ = DmaMode::DUPLEX;
    read_buff_ = read_data;
    rw_op_ = op;
    dma_result_ = ErrorCode::PENDING;
    busy_ = true;

    StartDmaDuplex(TOTAL);
    return CompleteDmaOperation(op, in_isr);
  }

  ErrorCode ans = PollingTransfer(static_cast<uint8_t*>(rx.addr_), tx_bytes, TOTAL);

  if (ans == ErrorCode::OK)
  {
    auto* rx_bytes = static_cast<uint8_t*>(rx.addr_);
    Memory::FastCopy(read_data.addr_, rx_bytes + 1, NEED_READ);
  }

  SwitchBuffer();

  if (op.type != OperationRW::OperationType::BLOCK)
  {
    op.UpdateStatus(in_isr, ans);
  }

  return ans;
}

ErrorCode MSPM0SPI::MemWrite(uint16_t reg, ConstRawData write_data, OperationRW& op,
                             bool in_isr)
{
  const uint32_t NEED_WRITE = static_cast<uint32_t>(write_data.size_);
  if (NEED_WRITE == 0)
  {
    if (op.type != OperationRW::OperationType::BLOCK)
    {
      op.UpdateStatus(in_isr, ErrorCode::OK);
    }
    return ErrorCode::OK;
  }

  if (DmaBusy())
  {
    return ErrorCode::BUSY;
  }

  RawData tx = GetTxBuffer();
  ASSERT(tx.size_ >= (NEED_WRITE + 1));

  auto* tx_bytes = static_cast<uint8_t*>(tx.addr_);
  tx_bytes[0] = static_cast<uint8_t>(reg & 0x7F);
  Memory::FastCopy(tx_bytes + 1, write_data.addr_, NEED_WRITE);

  const uint32_t TOTAL = NEED_WRITE + 1;

  if (TOTAL > dma_enable_min_size_)
  {
    DL_SPI_clearInterruptStatus(res_.instance, MSPM0_SPI_DMA_INTERRUPT_MASK);
    NVIC_ClearPendingIRQ(res_.irqn);

    mem_read_ = false;
    dma_mode_ = DmaMode::TX_ONLY;
    read_buff_ = {nullptr, 0};
    rw_op_ = op;
    dma_result_ = ErrorCode::PENDING;
    busy_ = true;

    StartDmaTxOnly(TOTAL);
    return CompleteDmaOperation(op, in_isr);
  }

  RawData rx = GetRxBuffer();
  ErrorCode ans = PollingTransfer(static_cast<uint8_t*>(rx.addr_), tx_bytes, TOTAL);

  SwitchBuffer();

  if (op.type != OperationRW::OperationType::BLOCK)
  {
    op.UpdateStatus(in_isr, ans);
  }

  return ans;
}

void MSPM0SPI::OnInterrupt(uint8_t index)
{
  if (index >= MAX_SPI_INSTANCES)
  {
    return;
  }

  MSPM0SPI* spi = instance_map_[index];
  if (spi == nullptr)
  {
    return;
  }

  spi->HandleInterrupt();
}

void MSPM0SPI::HandleInterrupt()
{
  if (!busy_)
  {
    DL_SPI_clearInterruptStatus(res_.instance, MSPM0_SPI_DMA_INTERRUPT_MASK);
    return;
  }

  auto drain_rx_fifo = [this]() -> bool
  {
    constexpr uint32_t RX_FIFO_DRAIN_MAX_ITERATIONS = 1024U;
    uint32_t remaining = RX_FIFO_DRAIN_MAX_ITERATIONS;
    uint8_t discard = 0U;
    while (remaining > 0U && DL_SPI_receiveDataCheck8(res_.instance, &discard))
    {
      --remaining;
    }

    return (remaining > 0U);
  };

  auto complete_dma_ok = [this]()
  {
    if (read_buff_.size_ > 0)
    {
      RawData rx = GetRxBuffer();
      auto* rx_bytes = static_cast<uint8_t*>(rx.addr_);
      if (mem_read_)
      {
        Memory::FastCopy(read_buff_.addr_, rx_bytes + 1, read_buff_.size_);
      }
      else
      {
        Memory::FastCopy(read_buff_.addr_, rx_bytes, read_buff_.size_);
      }
      read_buff_.size_ = 0;
    }

    SwitchBuffer();
    busy_ = false;
    dma_mode_ = DmaMode::DUPLEX;
    dma_result_ = ErrorCode::OK;
    rw_op_.UpdateStatus(true, dma_result_);
  };

  switch (DL_SPI_getPendingInterrupt(res_.instance))
  {
    case DL_SPI_IIDX_DMA_DONE_RX:
    {
      if (dma_mode_ == DmaMode::TX_ONLY)
      {
        break;
      }

      if (dma_mode_ == DmaMode::RX_ONLY && rx_only_remaining_ > 0U)
      {
        const uint32_t NEXT_CHUNK = min(rx_only_remaining_, RX_ONLY_REPEAT_TX_MAX_FRAMES);
        const uint32_t NEXT_OFFSET = rx_only_offset_;
        rx_only_offset_ += NEXT_CHUNK;
        rx_only_remaining_ -= NEXT_CHUNK;
        StartDmaRxOnly(NEXT_OFFSET, NEXT_CHUNK);
        break;
      }

      StopDma();
      complete_dma_ok();
      break;
    }

    case DL_SPI_IIDX_DMA_DONE_TX:
      if (dma_mode_ == DmaMode::TX_ONLY)
      {
        StopDma();
        const bool DRAIN_DONE = drain_rx_fifo();
        DL_SPI_clearInterruptStatus(
            res_.instance, DL_SPI_INTERRUPT_RX_OVERFLOW | DL_SPI_INTERRUPT_RX_TIMEOUT);

        if (!DRAIN_DONE)
        {
          busy_ = false;
          dma_mode_ = DmaMode::DUPLEX;
          dma_result_ = ErrorCode::FAILED;
          rw_op_.UpdateStatus(true, dma_result_);
          break;
        }

        complete_dma_ok();
      }
      break;

    case DL_SPI_IIDX_TX_UNDERFLOW:
    case DL_SPI_IIDX_PARITY_ERROR:
      StopDma();
      busy_ = false;
      dma_mode_ = DmaMode::DUPLEX;
      dma_result_ = ErrorCode::FAILED;
      rw_op_.UpdateStatus(true, dma_result_);
      break;

    case DL_SPI_IIDX_RX_OVERFLOW:
      if (dma_mode_ == DmaMode::TX_ONLY)
      {
        (void)drain_rx_fifo();
        DL_SPI_clearInterruptStatus(res_.instance, DL_SPI_INTERRUPT_RX_OVERFLOW);
        break;
      }

      StopDma();
      busy_ = false;
      dma_mode_ = DmaMode::DUPLEX;
      dma_result_ = ErrorCode::FAILED;
      rw_op_.UpdateStatus(true, dma_result_);
      break;

    case DL_SPI_IIDX_RX_TIMEOUT:
      if (dma_mode_ == DmaMode::TX_ONLY)
      {
        DL_SPI_clearInterruptStatus(res_.instance, DL_SPI_INTERRUPT_RX_TIMEOUT);
        break;
      }

      StopDma();
      busy_ = false;
      dma_mode_ = DmaMode::DUPLEX;
      dma_result_ = ErrorCode::TIMEOUT;
      rw_op_.UpdateStatus(true, dma_result_);
      break;

    default:
      break;
  }
}

#if defined(SPI0_BASE)
extern "C" void SPI0_IRQHandler(void)  // NOLINT
{
  LibXR::MSPM0SPI::OnInterrupt(0);
}
#endif

#if defined(SPI1_BASE)
extern "C" void SPI1_IRQHandler(void)  // NOLINT
{
  LibXR::MSPM0SPI::OnInterrupt(1);
}
#endif

#if defined(SPI2_BASE)
extern "C" void SPI2_IRQHandler(void)  // NOLINT
{
  LibXR::MSPM0SPI::OnInterrupt(2);
}
#endif
