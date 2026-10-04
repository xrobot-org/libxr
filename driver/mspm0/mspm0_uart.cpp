#include "mspm0_uart.hpp"

#include <algorithm>
#include <cstdint>
#include <limits>

using namespace LibXR;

namespace
{

constexpr uint64_t UART_UINT32_MAX = 0xFFFFFFFFULL;
constexpr uint64_t UART_BAUD_DIVISOR_MIN = 1ULL << 6U;
constexpr uint64_t UART_BAUD_DIVISOR_MAX = 0xFFFFULL << 6U;
constexpr DL_DMA_Config UART_DMA_TX_CONFIG = {
    .trigger = 0U,
    .triggerType = DL_DMA_TRIGGER_TYPE_EXTERNAL,
    .transferMode = DL_DMA_SINGLE_TRANSFER_MODE,
    .extendedMode = DL_DMA_NORMAL_MODE,
    .srcWidth = DL_DMA_WIDTH_BYTE,
    .destWidth = DL_DMA_WIDTH_BYTE,
    .srcIncrement = DL_DMA_ADDR_INCREMENT,
    .destIncrement = DL_DMA_ADDR_UNCHANGED,
};

#if defined(DEVICE_HAS_DMA_FULL_CHANNEL)
constexpr DL_DMA_Config UART_DMA_RX_CONFIG = {
    .trigger = 0U,
    .triggerType = DL_DMA_TRIGGER_TYPE_EXTERNAL,
    .transferMode = DL_DMA_FULL_CH_REPEAT_SINGLE_TRANSFER_MODE,
    .extendedMode = DL_DMA_NORMAL_MODE,
    .srcWidth = DL_DMA_WIDTH_BYTE,
    .destWidth = DL_DMA_WIDTH_BYTE,
    .srcIncrement = DL_DMA_ADDR_UNCHANGED,
    .destIncrement = DL_DMA_ADDR_INCREMENT,
};
#endif

// 校验 DriverLib 分频算式及寄存器可表示范围 / Check divider arithmetic and register
// range.
bool BaudRateRepresentable(uint32_t clock_freq, uint32_t baudrate)
{
  if (baudrate == 0U || baudrate > (UART_UINT32_MAX / 16U))
  {
    return false;
  }

  const uint64_t clock = clock_freq;
  const uint64_t baud = baudrate;
  uint64_t divisor = 0U;
  if ((baud * 8U) > clock)
  {
    if (clock > (UART_UINT32_MAX / 64U))
    {
      return false;
    }
    divisor = (clock * 64U) / (baud * 3U);
  }
  else if ((baud * 16U) > clock)
  {
    if (clock > (UART_UINT32_MAX / 8U))
    {
      return false;
    }
    const uint64_t half_baud = baud / 2U;
    if (half_baud == 0U)
    {
      return false;
    }
    divisor = (((clock * 8U) / half_baud) + 1U) / 2U;
  }
  else
  {
    if (clock > (UART_UINT32_MAX / 8U))
    {
      return false;
    }
    divisor = (((clock * 8U) / baud) + 1U) / 2U;
  }

  return divisor >= UART_BAUD_DIVISOR_MIN && divisor <= UART_BAUD_DIVISOR_MAX;
}

// 间隔阈值覆盖最长帧内高电平和一个完整字符 / Cover the longest high span plus one
// character.
uint64_t GapCompare(uint32_t clock_freq, UART::Configuration config)
{
  if (clock_freq == 0U || config.baudrate == 0U)
  {
    return 0U;
  }

  const uint64_t parity_bits = config.parity == UART::Parity::NO_PARITY ? 0U : 1U;
  const uint64_t high_bits = config.data_bits + parity_bits + config.stop_bits;
  const uint64_t gap_bits = high_bits + (1U + high_bits);
  const uint64_t numerator = static_cast<uint64_t>(clock_freq) * gap_bits;
  return (numerator / config.baudrate) + ((numerator % config.baudrate) != 0U ? 1U : 0U);
}

}  // namespace

MSPM0UART* MSPM0UART::instance_map_[MAX_UART_INSTANCES] = {nullptr};

MSPM0UART::MSPM0UART(Resources res, RawData tx_dma_storage, RawData rx_dma_storage,
                     uint32_t tx_queue_size, uint32_t rx_queue_capacity,
                     UART::Configuration config)
    : UART(&_read_port, &_write_port),
      _read_port(rx_queue_capacity),
      _write_port(tx_queue_size, tx_dma_storage.size_ / 2U),
      res_(res),
      tx_dma_storage_(tx_dma_storage),
      rx_dma_storage_(rx_dma_storage),
      tx_half_size_(tx_dma_storage.size_ / 2U)
{
  ASSERT(res_.instance != nullptr);
  ASSERT(res_.clock_freq > 0U);
  ASSERT(res_.index < MAX_UART_INSTANCES);
  ASSERT(res_.index == ResolveIndex(res_.irqn));
  ASSERT(instance_map_[res_.index] == nullptr);
  ASSERT(tx_dma_storage_.addr_ != nullptr);
  ASSERT(tx_dma_storage_.size_ > 1U);
  ASSERT((tx_dma_storage_.size_ % 2U) == 0U);
  ASSERT((reinterpret_cast<uintptr_t>(tx_dma_storage_.addr_) % alignof(size_t)) == 0U);
  ASSERT((tx_dma_storage_.size_ % (2U * alignof(size_t))) == 0U);
  ASSERT(tx_half_size_ <= MSPM0_UART_DMA_MAX_TRANSFER_SIZE);
  ASSERT(tx_queue_size > 0U);
  ASSERT(rx_queue_capacity > 0U);

  if (res_.rx_mode == RxMode::EXTEND_DMA)
  {
    ASSERT(rx_dma_storage_.addr_ != nullptr);
    ASSERT(rx_dma_storage_.size_ > 1U);
    ASSERT((rx_dma_storage_.size_ % 2U) == 0U);
    ASSERT(rx_dma_storage_.size_ <= MSPM0_UART_DMA_MAX_TRANSFER_SIZE);
    ASSERT(res_.dma_rx_channel < DMA_SYS_N_DMA_FULL_CHANNEL);
  }
  else
  {
    ASSERT(rx_dma_storage_.addr_ == nullptr);
    ASSERT(rx_dma_storage_.size_ == 0U);
    ASSERT(!res_.rx_half_interrupt);
    ASSERT(res_.dma_rx_channel == INVALID_DMA_CHANNEL);
  }

  ASSERT(res_.dma_tx_channel < MAX_DMA_CHANNELS);
  ASSERT(res_.dma_tx_trigger != 0U);
  if (res_.rx_mode == RxMode::EXTEND_DMA)
  {
    ASSERT(res_.dma_rx_channel < MAX_DMA_CHANNELS);
    ASSERT(res_.dma_rx_channel != res_.dma_tx_channel);
    ASSERT(res_.dma_rx_trigger != 0U);
    ASSERT(res_.dma_rx_trigger != res_.dma_tx_trigger);
    ASSERT(!res_.rx_half_interrupt || res_.dma_rx_channel < 8U);
  }

  for (const MSPM0UART* other : instance_map_)
  {
    if (other == nullptr)
    {
      continue;
    }

    ASSERT(res_.dma_tx_channel != other->res_.dma_tx_channel);
    if (other->res_.rx_mode == RxMode::EXTEND_DMA)
    {
      ASSERT(res_.dma_tx_channel != other->res_.dma_rx_channel);
    }

    if (res_.rx_mode == RxMode::EXTEND_DMA)
    {
      ASSERT(res_.dma_rx_channel != other->res_.dma_tx_channel);
      if (other->res_.rx_mode == RxMode::EXTEND_DMA)
      {
        ASSERT(res_.dma_rx_channel != other->res_.dma_rx_channel);
      }
    }
  }

  ASSERT(ValidateConfig(config) == ErrorCode::OK);
  _read_port.SetOwner(this);
  _write_port = WriteFun;

  NVIC_DisableIRQ(res_.irqn);
  NVIC_ClearPendingIRQ(res_.irqn);
  ConfigureTxDma();
  ApplyConfig(config);
  if (res_.rx_mode == RxMode::EXTEND_DMA)
  {
    ConfigureRxDma();
  }
  instance_map_[res_.index] = this;
  StartDataPath();

  NVIC_EnableIRQ(res_.irqn);
}

ErrorCode MSPM0UART::ValidateConfig(UART::Configuration config) const
{
  if (!BaudRateRepresentable(res_.clock_freq, config.baudrate) || config.data_bits < 5U ||
      config.data_bits > 8U || (config.stop_bits != 1U && config.stop_bits != 2U))
  {
    return ErrorCode::ARG_ERR;
  }
  if (config.parity != UART::Parity::NO_PARITY && config.parity != UART::Parity::EVEN &&
      config.parity != UART::Parity::ODD)
  {
    return ErrorCode::ARG_ERR;
  }
  if (res_.rx_mode == RxMode::EXTEND_DMA)
  {
    const uint64_t gap = GapCompare(res_.clock_freq, config);
    if (gap == 0U || gap > std::numeric_limits<uint16_t>::max())
    {
      return ErrorCode::ARG_ERR;
    }
  }
  return ErrorCode::OK;
}

UART::Configuration MSPM0UART::BuildConfigFromSysCfg(UART_Regs* instance,
                                                     uint32_t baudrate)
{
  DEV_ASSERT(instance != nullptr);
  DEV_ASSERT(baudrate > 0U);

  UART::Configuration config = {baudrate, UART::Parity::NO_PARITY, 8U, 1U};
  switch (DL_UART_getWordLength(instance))
  {
    case DL_UART_WORD_LENGTH_5_BITS:
      config.data_bits = 5U;
      break;
    case DL_UART_WORD_LENGTH_6_BITS:
      config.data_bits = 6U;
      break;
    case DL_UART_WORD_LENGTH_7_BITS:
      config.data_bits = 7U;
      break;
    case DL_UART_WORD_LENGTH_8_BITS:
    default:
      config.data_bits = 8U;
      break;
  }

  switch (DL_UART_getParityMode(instance))
  {
    case DL_UART_PARITY_NONE:
      config.parity = UART::Parity::NO_PARITY;
      break;
    case DL_UART_PARITY_EVEN:
      config.parity = UART::Parity::EVEN;
      break;
    case DL_UART_PARITY_ODD:
      config.parity = UART::Parity::ODD;
      break;
    default:
      DEV_ASSERT(false);
      break;
  }
  config.stop_bits = DL_UART_getStopBits(instance) == DL_UART_STOP_BITS_TWO ? 2U : 1U;
  return config;
}

ErrorCode MSPM0UART::SetConfig(UART::Configuration config, bool in_isr)
{
  const ErrorCode validation = ValidateConfig(config);
  if (validation != ErrorCode::OK)
  {
    return validation;
  }

  ConfigState expected = ConfigState::EMPTY;
  if (!config_state_.compare_exchange_strong(expected, ConfigState::RESERVED,
                                             std::memory_order_acq_rel,
                                             std::memory_order_acquire))
  {
    return ErrorCode::BUSY;
  }

  // 配置内容通过下方事件发布，由处理器将槽标记为 PUBLISHED。
  // Publish the payload through the event; the service marks the slot PUBLISHED.
  pending_config_ = config;
  service_.Invoke(EVENT_CONFIG, in_isr, [this](uint32_t events, bool owner_in_isr)
                  { HandleService(events, owner_in_isr); });
  return ErrorCode::OK;
}

void MSPM0UART::WriteFun(WritePort& port, bool in_isr)
{
  auto* uart = LibXR::ContainerOf(&port, &MSPM0UART::_write_port);
  uart->service_.Invoke(EVENT_WRITE, in_isr, [uart](uint32_t events, bool owner_in_isr)
                        { uart->HandleService(events, owner_in_isr); });
}

void MSPM0UART::NotifyReadSpace(bool in_isr)
{
  service_.Invoke(EVENT_RX_WORK, in_isr, [this](uint32_t events, bool owner_in_isr)
                  { HandleService(events, owner_in_isr); });
}

void MSPM0UART::HandleService(uint32_t events, bool in_isr)
{
  if ((events & EVENT_CONFIG) != 0U)
  {
    DEV_ASSERT_FROM_CALLBACK(
        config_state_.load(std::memory_order_acquire) == ConfigState::RESERVED, in_isr);
    config_state_.store(ConfigState::PUBLISHED, std::memory_order_release);
  }

  if ((events & EVENT_DMA_DONE_TX) != 0U)
  {
    HandleTxDone(in_isr);
  }

  if ((events & EVENT_RX_WORK) != 0U)
  {
    HandleRxWork(in_isr);
  }

  if ((events & (EVENT_CONFIG | EVENT_DMA_DONE_TX | EVENT_EOT_DONE | EVENT_RX_WORK)) !=
      0U)
  {
    TryApplyPublishedConfig(in_isr);
  }

  if (events != 0U && config_state_.load(std::memory_order_acquire) == ConfigState::EMPTY)
  {
    if ((events & (EVENT_WRITE | EVENT_DMA_DONE_TX | EVENT_CONFIG | EVENT_EOT_DONE)) !=
        0U)
    {
      FillTx(in_isr);
    }
  }
}

void MSPM0UART::FillTx(bool in_isr)
{
  while (config_state_.load(std::memory_order_acquire) == ConfigState::EMPTY)
  {
    if (active_half_ < 0)
    {
      for (uint8_t half = 0U; half < 2U; ++half)
      {
        if (tx_half_size_used_[half] != 0U)
        {
          if (config_state_.load(std::memory_order_acquire) != ConfigState::EMPTY)
          {
            return;
          }
          active_half_ = static_cast<int8_t>(half);
          StartTxDma(half, tx_half_size_used_[half]);
          break;
        }
      }
    }

    uint8_t free_half = INVALID_DMA_CHANNEL;
    for (uint8_t i = 0U; i < 2U; ++i)
    {
      if (tx_half_size_used_[i] == 0U)
      {
        free_half = i;
        break;
      }
    }
    if (free_half == INVALID_DMA_CHANNEL)
    {
      return;
    }

    size_t size = 0U;
    {
      auto queue = _write_port.GetWriteQueue(in_isr);
      if (queue.Empty())
      {
        return;
      }
      if (config_state_.load(std::memory_order_acquire) != ConfigState::EMPTY)
      {
        return;
      }

      size = queue.AvailableSize();
      DEV_ASSERT_FROM_CALLBACK(size > 0U && size <= tx_half_size_, in_isr);
      queue.PopAll(TxHalf(free_half));
      tx_half_size_used_[free_half] = size;
    }

    if (active_half_ < 0 &&
        config_state_.load(std::memory_order_acquire) == ConfigState::EMPTY)
    {
      active_half_ = static_cast<int8_t>(free_half);
      StartTxDma(free_half, size);
    }
  }
}

void MSPM0UART::HandleTxDone(bool in_isr)
{
  DEV_ASSERT_FROM_CALLBACK(active_half_ >= 0 && active_half_ < 2, in_isr);
  const uint8_t completed = static_cast<uint8_t>(active_half_);
  tx_half_size_used_[completed] = 0U;
  active_half_ = -1;

  const uint8_t next = completed ^ 1U;
  if (tx_half_size_used_[next] != 0U &&
      config_state_.load(std::memory_order_acquire) == ConfigState::EMPTY)
  {
    active_half_ = static_cast<int8_t>(next);
    StartTxDma(next, tx_half_size_used_[next]);
  }
  else
  {
    DL_UART_disableInterrupt(res_.instance, TX_DONE_INTERRUPT_MASK);
    DL_UART_disableDMATransmitEvent(res_.instance);
  }
}

void MSPM0UART::HandleRxWork(bool in_isr)
{
  if (res_.rx_mode == RxMode::MAIN_BYTE_IRQ)
  {
    HandleMainRx(in_isr);
  }
  else
  {
    HandleExtendRx(in_isr);
  }
}

void MSPM0UART::HandleMainRx(bool in_isr)
{
  auto queue = _read_port.GetReadQueue(in_isr);
  if (queue.EmptySize() != 0U)
  {
    DL_UART_enableInterrupt(res_.instance, RX_INTERRUPT_MASK);
  }

  while (!DL_UART_isRXFIFOEmpty(res_.instance))
  {
    if (queue.EmptySize() == 0U)
    {
      DL_UART_disableInterrupt(res_.instance, RX_INTERRUPT_MASK);
      break;
    }

    const uint8_t byte = DL_UART_receiveData(res_.instance);
    [[maybe_unused]] const auto push_batch_result = queue.PushBatch(&byte, 1U);
    DEV_ASSERT_FROM_CALLBACK(push_batch_result == ErrorCode::OK, in_isr);
  }
  queue.Publish();
}

void MSPM0UART::HandleExtendRx(bool in_isr)
{
  const size_t capacity = RxCapacity();
  DEV_ASSERT_FROM_CALLBACK(capacity > 1U, in_isr);

  const uint32_t remaining = DL_DMA_getTransferSize(DMA, res_.dma_rx_channel);
  DEV_ASSERT_FROM_CALLBACK(remaining <= capacity, in_isr);

  const size_t position = remaining == 0U ? capacity : capacity - remaining;
  const size_t cursor = rx_dma_cursor_;
  DEV_ASSERT_FROM_CALLBACK(cursor < capacity, in_isr);

  const size_t first_size = position >= cursor ? position - cursor : capacity - cursor;
  const size_t second_size = position >= cursor ? 0U : position;
  const size_t observed = first_size + second_size;

  auto queue = _read_port.GetReadQueue(in_isr);
  size_t accepted = std::min(observed, queue.EmptySize());
  size_t first_accepted = std::min(first_size, accepted);
  if (first_accepted != 0U)
  {
    auto* source = static_cast<const uint8_t*>(rx_dma_storage_.addr_) + cursor;
    [[maybe_unused]] const auto push_batch_result =
        queue.PushBatch(source, first_accepted);
    DEV_ASSERT_FROM_CALLBACK(push_batch_result == ErrorCode::OK, in_isr);
    accepted -= first_accepted;
  }
  if (accepted != 0U)
  {
    [[maybe_unused]] const auto push_batch_result =
        queue.PushBatch(static_cast<const uint8_t*>(rx_dma_storage_.addr_), accepted);
    DEV_ASSERT_FROM_CALLBACK(push_batch_result == ErrorCode::OK, in_isr);
  }

  // 超出软件容量的尾部作为溢出丢弃，回调前先推进游标。
  // Drop excess bytes and advance the cursor before callbacks can run.
  rx_dma_cursor_ = position == capacity ? 0U : position;
  queue.Publish();
}

void MSPM0UART::HandleUartInterrupt()
{
  constexpr uint32_t MASK = RX_ERROR_INTERRUPT_MASK | RX_INTERRUPT_MASK |
                            RX_TIMEOUT_INTERRUPT_MASK | TX_DONE_INTERRUPT_MASK |
                            EOT_INTERRUPT_MASK | RX_GAP_INTERRUPT_MASK;
  const uint32_t pending = DL_UART_getEnabledInterruptStatus(res_.instance, MASK);
  if (pending == 0U)
  {
    return;
  }

  if ((pending & RX_TIMEOUT_INTERRUPT_MASK) != 0U)
  {
    DL_UART_disableInterrupt(res_.instance, RX_TIMEOUT_INTERRUPT_MASK);
  }
  DL_UART_clearInterruptStatus(res_.instance, pending & MASK);

  uint32_t events = 0U;
  if ((pending &
       (RX_INTERRUPT_MASK | RX_TIMEOUT_INTERRUPT_MASK | RX_GAP_INTERRUPT_MASK)) != 0U)
  {
    events |= EVENT_RX_WORK;
  }
  if ((pending & TX_DONE_INTERRUPT_MASK) != 0U)
  {
    events |= EVENT_DMA_DONE_TX;
  }
  if ((pending & EOT_INTERRUPT_MASK) != 0U)
  {
    events |= EVENT_EOT_DONE;
  }

  if (events != 0U)
  {
    service_.Invoke(events, true, [this](uint32_t handled, bool in_isr)
                    { HandleService(handled, in_isr); });
  }
}

void MSPM0UART::HandleRxDmaInterrupt()
{
  service_.Invoke(EVENT_RX_WORK, true, [this](uint32_t events, bool in_isr)
                  { HandleService(events, in_isr); });
}

void MSPM0UART::OnInterrupt(uint8_t index)
{
  if (index < MAX_UART_INSTANCES && instance_map_[index] != nullptr)
  {
    instance_map_[index]->HandleUartInterrupt();
  }
}

void MSPM0UART::OnDmaInterrupt()
{
  uint32_t owned_mask = 0U;
  for (MSPM0UART* uart : instance_map_)
  {
    if (uart != nullptr && uart->res_.rx_mode == RxMode::EXTEND_DMA)
    {
      owned_mask |= DmaRawMask(uart->res_.dma_rx_channel, uart->res_.rx_half_interrupt);
    }
  }

  if (owned_mask == 0U)
  {
    return;
  }

  const uint32_t pending = DL_DMA_getEnabledInterruptStatus(DMA, owned_mask);
  for (MSPM0UART* uart : instance_map_)
  {
    if (uart == nullptr || uart->res_.rx_mode != RxMode::EXTEND_DMA)
    {
      continue;
    }

    const uint32_t mask =
        DmaRawMask(uart->res_.dma_rx_channel, uart->res_.rx_half_interrupt);
    const uint32_t hit = pending & mask;
    if (hit != 0U)
    {
      DL_DMA_clearInterruptStatus(DMA, hit);
      uart->HandleRxDmaInterrupt();
    }
  }
  __DSB();
}

void MSPM0UARTReadPort::OnReadQueueSpaceAvailable(bool in_isr)
{
  DEV_ASSERT_FROM_CALLBACK(owner_ != nullptr, in_isr);
  owner_->NotifyReadSpace(in_isr);
}

void MSPM0UART::ConfigureTxDma()
{
  DL_DMA_Config config = UART_DMA_TX_CONFIG;
  config.trigger = res_.dma_tx_trigger;
  DL_DMA_disableChannel(DMA, res_.dma_tx_channel);
  DL_DMA_clearInterruptStatus(DMA, DmaCompleteMask(res_.dma_tx_channel));
  DL_DMA_initChannel(DMA, res_.dma_tx_channel, &config);
}

void MSPM0UART::ConfigureRxDma()
{
#if defined(DEVICE_HAS_DMA_FULL_CHANNEL)
  if (res_.rx_mode != RxMode::EXTEND_DMA)
  {
    return;
  }

  DL_DMA_Config config = UART_DMA_RX_CONFIG;
  config.trigger = res_.dma_rx_trigger;
  DL_DMA_disableChannel(DMA, res_.dma_rx_channel);
  DL_DMA_initChannel(DMA, res_.dma_rx_channel, &config);
  DL_DMA_setSrcAddr(
      DMA, res_.dma_rx_channel,
      static_cast<uint32_t>(reinterpret_cast<uintptr_t>(&res_.instance->RXDATA)));
  DL_DMA_setDestAddr(
      DMA, res_.dma_rx_channel,
      static_cast<uint32_t>(reinterpret_cast<uintptr_t>(rx_dma_storage_.addr_)));
  DL_DMA_setTransferSize(DMA, res_.dma_rx_channel,
                         static_cast<uint16_t>(rx_dma_storage_.size_));
  if (res_.rx_half_interrupt)
  {
    DL_DMA_Full_Ch_setEarlyInterruptThreshold(DMA, res_.dma_rx_channel,
                                              DL_DMA_EARLY_INTERRUPT_THRESHOLD_HALF);
  }
#else
  DEV_ASSERT(false);
#endif
}

void MSPM0UART::ApplyConfig(UART::Configuration config)
{
  // 配置路径已等待空闲；保留 FEN，避免反复切换 FIFO 暴露旧的 RX 内容。
  // The configuration path has reached idle. Keep FEN unchanged: toggling FIFO
  // mode on G3507 can expose previously consumed RX bytes.
  DL_UART_disable(res_.instance);
  while (DL_UART_isBusy(res_.instance))
  {
  }

  DL_UART_WORD_LENGTH word_length = DL_UART_WORD_LENGTH_8_BITS;
  switch (config.data_bits)
  {
    case 5U:
      word_length = DL_UART_WORD_LENGTH_5_BITS;
      break;
    case 6U:
      word_length = DL_UART_WORD_LENGTH_6_BITS;
      break;
    case 7U:
      word_length = DL_UART_WORD_LENGTH_7_BITS;
      break;
    case 8U:
      break;
    default:
      DEV_ASSERT(false);
      break;
  }

  DL_UART_PARITY parity = DL_UART_PARITY_NONE;
  if (config.parity == UART::Parity::EVEN)
  {
    parity = DL_UART_PARITY_EVEN;
  }
  else if (config.parity == UART::Parity::ODD)
  {
    parity = DL_UART_PARITY_ODD;
  }

  DL_UART_setWordLength(res_.instance, word_length);
  DL_UART_setParityMode(res_.instance, parity);
  DL_UART_setStopBits(res_.instance, config.stop_bits == 2U ? DL_UART_STOP_BITS_TWO
                                                            : DL_UART_STOP_BITS_ONE);
  DL_UART_setCommunicationMode(res_.instance, DL_UART_MODE_NORMAL);
  DL_UART_setDirection(res_.instance, DL_UART_DIRECTION_TX_RX);
  DL_UART_enableFIFOs(res_.instance);
  DL_UART_setTXFIFOThreshold(res_.instance, DL_UART_TX_FIFO_LEVEL_ONE_ENTRY);
  DL_UART_setRXFIFOThreshold(res_.instance, DL_UART_RX_FIFO_LEVEL_ONE_ENTRY);
  DL_UART_setRXInterruptTimeout(res_.instance, 0U);
  DL_UART_configBaudRate(res_.instance, res_.clock_freq, config.baudrate);

  if (res_.rx_mode == RxMode::EXTEND_DMA)
  {
    DL_UART_setLINCounterValue(res_.instance, 0U);
    DL_UART_setLINCounterCompareValue(
        res_.instance, static_cast<uint16_t>(GapCompare(res_.clock_freq, config)));
    DL_UART_enableLINCounterCompareMatch(res_.instance);
    DL_UART_disableLINCountWhileLow(res_.instance);
    DL_UART_enableLINCounterClearOnFallingEdge(res_.instance);
    DL_UART_enableLINCounter(res_.instance);
  }
}

void MSPM0UART::StartDataPath()
{
  DL_UART_disableInterrupt(res_.instance, 0xFFFFFFFFU);
  DL_UART_clearInterruptStatus(res_.instance, 0xFFFFFFFFU);
  DL_UART_clearDMATransmitEventStatus(res_.instance);
  DL_UART_clearDMAReceiveEventStatus(res_.instance, DL_UART_DMA_INTERRUPT_RX);
  DL_UART_disableDMAReceiveEvent(res_.instance, DL_UART_DMA_INTERRUPT_RX);
  DL_UART_disableDMATransmitEvent(res_.instance);
  DL_UART_enableInterrupt(res_.instance, RX_ERROR_INTERRUPT_MASK);

  if (res_.rx_mode == RxMode::MAIN_BYTE_IRQ)
  {
    DL_UART_enableInterrupt(res_.instance, RX_INTERRUPT_MASK);
  }
  else
  {
    const uint32_t dma_mask = DmaRawMask(res_.dma_rx_channel, res_.rx_half_interrupt);
    DL_DMA_clearInterruptStatus(DMA, dma_mask);
    DL_DMA_enableInterrupt(DMA, dma_mask);
    DL_DMA_enableChannel(DMA, res_.dma_rx_channel);
    DL_UART_enableInterrupt(res_.instance, RX_GAP_INTERRUPT_MASK);
    DL_UART_enableDMAReceiveEvent(res_.instance, DL_UART_DMA_INTERRUPT_RX);
    rx_dma_cursor_ = 0U;
  }

  __DMB();
  DL_UART_enable(res_.instance);
}

void MSPM0UART::StopDataPath()
{
  DL_UART_disableInterrupt(res_.instance, RX_ERROR_INTERRUPT_MASK | RX_INTERRUPT_MASK |
                                              RX_TIMEOUT_INTERRUPT_MASK |
                                              TX_DONE_INTERRUPT_MASK |
                                              EOT_INTERRUPT_MASK | RX_GAP_INTERRUPT_MASK);
  DL_UART_disableDMAReceiveEvent(res_.instance, DL_UART_DMA_INTERRUPT_RX);
  DL_UART_disableDMATransmitEvent(res_.instance);
  if (res_.rx_mode == RxMode::EXTEND_DMA)
  {
    DL_DMA_disableChannel(DMA, res_.dma_rx_channel);
    DL_DMA_disableInterrupt(DMA, DmaRawMask(res_.dma_rx_channel, res_.rx_half_interrupt));
  }
  DL_UART_disable(res_.instance);
}

void MSPM0UART::DiscardRxFifo()
{
  while (!DL_UART_isRXFIFOEmpty(res_.instance))
  {
    (void)DL_UART_receiveData(res_.instance);
  }
}

void MSPM0UART::StartTxDma(uint8_t half, size_t size)
{
  DEV_ASSERT(half < 2U);
  DEV_ASSERT(size > 0U && size <= tx_half_size_);

  DL_DMA_disableChannel(DMA, res_.dma_tx_channel);
  DL_DMA_clearInterruptStatus(DMA, DmaCompleteMask(res_.dma_tx_channel));
  DL_DMA_setSrcAddr(DMA, res_.dma_tx_channel,
                    static_cast<uint32_t>(reinterpret_cast<uintptr_t>(TxHalf(half))));
  DL_DMA_setDestAddr(
      DMA, res_.dma_tx_channel,
      static_cast<uint32_t>(reinterpret_cast<uintptr_t>(&res_.instance->TXDATA)));
  DL_DMA_setTransferSize(DMA, res_.dma_tx_channel, static_cast<uint16_t>(size));
  DL_UART_clearInterruptStatus(res_.instance,
                               TX_DONE_INTERRUPT_MASK | EOT_INTERRUPT_MASK);
  DL_UART_enableInterrupt(res_.instance, TX_DONE_INTERRUPT_MASK);
  DL_UART_enableDMATransmitEvent(res_.instance);
  __DMB();
  DL_DMA_enableChannel(DMA, res_.dma_tx_channel);
}

void MSPM0UART::TryApplyPublishedConfig(bool in_isr)
{
  if (config_state_.load(std::memory_order_acquire) != ConfigState::PUBLISHED)
  {
    return;
  }

  if (active_half_ >= 0)
  {
    return;
  }

  if (DL_UART_isBusy(res_.instance))
  {
    DL_UART_enableInterrupt(res_.instance, EOT_INTERRUPT_MASK);
    if (res_.rx_mode == RxMode::MAIN_BYTE_IRQ)
    {
      DL_UART_disableInterrupt(res_.instance, RX_TIMEOUT_INTERRUPT_MASK);
      DL_UART_clearInterruptStatus(res_.instance, RX_TIMEOUT_INTERRUPT_MASK);
      DL_UART_setRXInterruptTimeout(res_.instance, CONFIG_RX_TIMEOUT);
      DL_UART_enableInterrupt(res_.instance, RX_TIMEOUT_INTERRUPT_MASK);
    }
    return;
  }

  const UART::Configuration config = pending_config_;
  StopDataPath();
  DiscardRxFifo();
  ApplyConfig(config);
  if (res_.rx_mode == RxMode::EXTEND_DMA)
  {
    ConfigureRxDma();
  }
  config_state_.store(ConfigState::EMPTY, std::memory_order_release);
  StartDataPath();
  // 当前处理器仍持有执行权，会在本轮结束后处理这些通知。
  // The active service will process these hints after this pass.
  service_.Publish(EVENT_WRITE | EVENT_RX_WORK);
  UNUSED(in_isr);
}

#if defined(UART0_BASE)
extern "C" void UART0_IRQHandler(void) { MSPM0UART::OnInterrupt(0U); }
#endif
#if defined(UART1_BASE)
extern "C" void UART1_IRQHandler(void) { MSPM0UART::OnInterrupt(1U); }
#endif
#if defined(UART2_BASE)
extern "C" void UART2_IRQHandler(void) { MSPM0UART::OnInterrupt(2U); }
#endif
#if defined(UART3_BASE)
extern "C" void UART3_IRQHandler(void) { MSPM0UART::OnInterrupt(3U); }
#endif
#if defined(UART4_BASE)
extern "C" void UART4_IRQHandler(void) { MSPM0UART::OnInterrupt(4U); }
#endif
#if defined(UART5_BASE)
extern "C" void UART5_IRQHandler(void) { MSPM0UART::OnInterrupt(5U); }
#endif
#if defined(UART6_BASE)
extern "C" void UART6_IRQHandler(void) { MSPM0UART::OnInterrupt(6U); }
#endif
#if defined(UART7_BASE)
extern "C" void UART7_IRQHandler(void) { MSPM0UART::OnInterrupt(7U); }
#endif
