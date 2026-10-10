#include "stm32_uart.hpp"

#include <algorithm>

#include "stm32_dcache.hpp"
#ifdef HAL_UART_MODULE_ENABLED

using namespace LibXR;

namespace
{

bool Stm32DataBitsSupported(const UART::Configuration& config)
{
  if (config.parity == UART::Parity::NO_PARITY)
  {
    return config.data_bits == 8U;
  }

  return (config.parity == UART::Parity::EVEN || config.parity == UART::Parity::ODD) &&
         (config.data_bits == 7U || config.data_bits == 8U);
}

bool Stm32StopBitsSupported(const UART::Configuration& config)
{
  return config.stop_bits == 1U || config.stop_bits == 2U;
}

}  // namespace

STM32UART* STM32UART::map[STM32_UART_NUMBER] = {nullptr};

stm32_uart_id_t stm32_uart_get_id(USART_TypeDef* addr)
{
  if (addr == nullptr)
  {  // NOLINT
    return stm32_uart_id_t::STM32_UART_ID_ERROR;
  }
#ifdef USART1
  else if (addr == USART1)  // NOLINT
  {
    return stm32_uart_id_t::STM32_USART1;
  }
#endif
#ifdef USART2
  else if (addr == USART2)  // NOLINT
  {
    return stm32_uart_id_t::STM32_USART2;
  }
#endif
#ifdef USART3
  else if (addr == USART3)  // NOLINT
  {
    return stm32_uart_id_t::STM32_USART3;
  }
#endif
#ifdef USART4
  else if (addr == USART4)  // NOLINT
  {
    return stm32_uart_id_t::STM32_USART4;
  }
#endif
#ifdef USART5
  else if (addr == USART5)  // NOLINT
  {
    return stm32_uart_id_t::STM32_USART5;
  }
#endif
#ifdef USART6
  else if (addr == USART6)  // NOLINT
  {
    return stm32_uart_id_t::STM32_USART6;
  }
#endif
#ifdef USART7
  else if (addr == USART7)  // NOLINT
  {
    return stm32_uart_id_t::STM32_USART7;
  }
#endif
#ifdef USART8
  else if (addr == USART8)  // NOLINT
  {
    return stm32_uart_id_t::STM32_USART8;
  }
#endif
#ifdef USART9
  else if (addr == USART9)  // NOLINT
  {
    return stm32_uart_id_t::STM32_USART9;
  }
#endif
#ifdef USART10
  else if (addr == USART10)  // NOLINT
  {
    return stm32_uart_id_t::STM32_USART10;
  }
#endif
#ifdef USART11
  else if (addr == USART11)  // NOLINT
  {
    return stm32_uart_id_t::STM32_USART11;
  }
#endif
#ifdef USART12
  else if (addr == USART12)  // NOLINT
  {
    return stm32_uart_id_t::STM32_USART12;
  }
#endif
#ifdef USART13
  else if (addr == USART13)  // NOLINT
  {
    return stm32_uart_id_t::STM32_USART13;
  }
#endif
#ifdef UART1
  else if (addr == UART1)  // NOLINT
  {
    return stm32_uart_id_t::STM32_UART1;
  }
#endif
#ifdef UART2
  else if (addr == UART2)  // NOLINT
  {
    return stm32_uart_id_t::STM32_UART2;
  }
#endif
#ifdef UART3
  else if (addr == UART3)  // NOLINT
  {
    return stm32_uart_id_t::STM32_UART3;
  }
#endif
#ifdef UART4
  else if (addr == UART4)  // NOLINT
  {
    return stm32_uart_id_t::STM32_UART4;
  }
#endif
#ifdef UART5
  else if (addr == UART5)  // NOLINT
  {
    return stm32_uart_id_t::STM32_UART5;
  }
#endif
#ifdef UART6
  else if (addr == UART6)  // NOLINT
  {
    return stm32_uart_id_t::STM32_UART6;
  }
#endif
#ifdef UART7
  else if (addr == UART7)  // NOLINT
  {
    return stm32_uart_id_t::STM32_UART7;
  }
#endif
#ifdef UART8
  else if (addr == UART8)  // NOLINT
  {
    return stm32_uart_id_t::STM32_UART8;
  }
#endif
#ifdef UART9
  else if (addr == UART9)  // NOLINT
  {
    return stm32_uart_id_t::STM32_UART9;
  }
#endif
#ifdef UART10
  else if (addr == UART10)  // NOLINT
  {
    return stm32_uart_id_t::STM32_UART10;
  }
#endif
#ifdef UART11
  else if (addr == UART11)  // NOLINT
  {
    return stm32_uart_id_t::STM32_UART11;
  }
#endif
#ifdef UART12
  else if (addr == UART12)  // NOLINT
  {
    return stm32_uart_id_t::STM32_UART12;
  }
#endif
#ifdef UART13
  else if (addr == UART13)  // NOLINT
  {
    return stm32_uart_id_t::STM32_UART13;
  }
#endif
#ifdef LPUART1
  else if (addr == LPUART1)  // NOLINT
  {
    return stm32_uart_id_t::STM32_LPUART1;
  }
#endif
#ifdef LPUART2
  else if (addr == LPUART2)  // NOLINT
  {
    return stm32_uart_id_t::STM32_LPUART2;
  }
#endif
#ifdef LPUART3
  else if (addr == LPUART3)  // NOLINT
  {
    return stm32_uart_id_t::STM32_LPUART3;
  }
#endif
  else
  {
    return stm32_uart_id_t::STM32_UART_ID_ERROR;
  }
}

void STM32UART::WriteFun(WritePort& port, bool in_isr)
{
  auto* uart = LibXR::ContainerOf(&port, &STM32UART::_write_port);

  uart->tx_service_.Invoke(TX_EVENT_WRITE, in_isr,
                           [uart](uint32_t events, bool owner_in_isr)
                           { uart->HandleTxService(events, owner_in_isr); });
}

void STM32UART::HandleTxService(uint32_t events, bool in_isr)
{
  if ((events & TX_EVENT_CONFIG) != 0U)
  {
    ConfigState state = config_state_.load(std::memory_order_acquire);
    if (state == ConfigState::RESERVED)
    {
      config_state_.store(ConfigState::PUBLISHED, std::memory_order_release);
    }
    else
    {
      DEV_ASSERT_FROM_CALLBACK(state == ConfigState::PUBLISHED, in_isr);
    }
  }

  if (abort_pending_)
  {
#if defined(LIBXR_STM32_UART_GPDMA)
    // HAL 自有的 RX 中止通过错误回调通知，因此按硬件状态判断是否全部停止。
    // HAL-owned RX aborts notify through the error callback; check actual stop state.
    if (!gpdma_adapter_.AllStopsComplete())
    {
      return;
    }
    events |= TX_EVENT_ABORT;
    [[maybe_unused]] const auto uart_abort_result = HAL_UART_Abort(uart_handle_);
    REQUIRE_FROM_CALLBACK(uart_abort_result == HAL_OK, in_isr);
    if (uart_handle_->hdmatx != nullptr)
    {
      STM32GpdmaUartAdapter::FinalizeStopped(uart_handle_->hdmatx, in_isr);
    }
    if (uart_handle_->hdmarx != nullptr)
    {
      STM32GpdmaUartAdapter::FinalizeStopped(uart_handle_->hdmarx, in_isr);
    }
    HandleTxDone(in_isr);
    if (config_state_.load(std::memory_order_acquire) == ConfigState::PUBLISHED)
    {
      ApplyConfig(pending_config_, in_isr);
      config_state_.store(ConfigState::EMPTY, std::memory_order_release);
    }
#else
    if ((events & TX_EVENT_ABORT) == 0U)
    {
      return;
    }
#endif
    last_rx_pos_ = 0U;
    SetRxDMA(in_isr);
#if !defined(LIBXR_STM32_UART_GPDMA)
    HandleTxDone(in_isr);
#endif
    abort_pending_ = false;
  }
  else if ((events & TX_EVENT_ERROR) != 0U)
  {
    // 中止可能同步完成，调用 HAL 前先记录等待状态。
    // Record the wait before calling HAL, which may complete the abort synchronously.
    abort_pending_ = true;
    BeginAbort(in_isr);
    return;
  }
  else if ((events & TX_EVENT_DONE) != 0U)
  {
    HandleTxDone(in_isr);
  }

#if defined(LIBXR_STM32_UART_GPDMA)
  if ((events & TX_EVENT_START_RX) != 0U)
  {
    SetRxDMA(in_isr);
  }
#endif
  if ((events & TX_EVENT_RX_WORK) != 0U)
  {
    HandleRxData(in_isr);
  }

  if ((events & (TX_EVENT_CONFIG | TX_EVENT_DONE | TX_EVENT_ABORT)) != 0U)
  {
    TryApplyConfig(in_isr);
  }

  if ((uart_handle_->Init.Mode & UART_MODE_TX) != 0U &&
      (events & (TX_EVENT_WRITE | TX_EVENT_DONE | TX_EVENT_CONFIG | TX_EVENT_ABORT)) !=
          0U &&
      config_state_.load(std::memory_order_acquire) == ConfigState::EMPTY)
  {
    FillTx(in_isr);
  }
}

void STM32UART::FillTx(bool in_isr)
{
  if (!tx_busy_.IsSet())
  {
    if (dma_buff_tx_.HasPending())
    {
      if (config_state_.load(std::memory_order_acquire) != ConfigState::EMPTY)
      {
        return;
      }
      const size_t size = dma_buff_tx_.GetPendingLength();
      dma_buff_tx_.Switch();
      dma_buff_tx_.SetActiveLength(size);
      StartTxDma(in_isr);
    }
    else if (dma_buff_tx_.GetActiveLength() != 0U)
    {
      if (config_state_.load(std::memory_order_acquire) != ConfigState::EMPTY)
      {
        return;
      }
      StartTxDma(in_isr);
    }
    else
    {
      if (config_state_.load(std::memory_order_acquire) != ConfigState::EMPTY)
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
        size = queue.AvailableSize();
        DEV_ASSERT_FROM_CALLBACK(size <= dma_buff_tx_.Size(), in_isr);
        queue.PopAll(dma_buff_tx_.ActiveBuffer());
        dma_buff_tx_.SetActiveLength(size);
      }

      if (config_state_.load(std::memory_order_acquire) != ConfigState::EMPTY)
      {
        return;
      }
      StartTxDma(in_isr);
    }
  }

  if (tx_busy_.IsSet() && !dma_buff_tx_.HasPending() &&
      config_state_.load(std::memory_order_acquire) == ConfigState::EMPTY)
  {
    auto queue = _write_port.GetWriteQueue(in_isr);
    if (!queue.Empty())
    {
      const size_t size = queue.AvailableSize();
      DEV_ASSERT_FROM_CALLBACK(size <= dma_buff_tx_.Size(), in_isr);
      queue.PopAll(dma_buff_tx_.PendingBuffer());
      dma_buff_tx_.SetPendingLength(size);
      dma_buff_tx_.EnablePending();
    }
  }
}

void STM32UART::HandleTxDone(bool in_isr)
{
  UNUSED(in_isr);
  if (!tx_busy_.IsSet())
  {
    return;
  }

  tx_busy_.Clear();
  dma_buff_tx_.SetActiveLength(0U);
}

STM32UART::STM32UART(UART_HandleTypeDef* uart_handle, RawData dma_buff_rx,
                     RawData dma_buff_tx, uint32_t tx_queue_size)
    : UART(&_read_port, &_write_port),
      _read_port(dma_buff_rx.size_),
      _write_port(tx_queue_size, dma_buff_tx.size_ / 2),
      dma_buff_rx_(dma_buff_rx),
      dma_buff_tx_(dma_buff_tx),
      uart_handle_(uart_handle),
      id_(stm32_uart_get_id(uart_handle_->Instance))
#if defined(LIBXR_STM32_UART_GPDMA)
      ,
      gpdma_adapter_(uart_handle)
#endif
{
  ASSERT(id_ != STM32_UART_ID_ERROR);
  ASSERT(dma_buff_rx.size_ == 0U || dma_buff_rx.addr_ != nullptr);

  map[id_] = this;

  if ((uart_handle->Init.Mode & UART_MODE_TX) == UART_MODE_TX)
  {
    ASSERT(tx_queue_size > 0U);
    ASSERT(uart_handle_->hdmatx != NULL);
#if defined(LIBXR_STM32_UART_GPDMA)
    ASSERT(uart_handle_->hdmatx->Mode == DMA_NORMAL);
#endif
    _write_port = WriteFun;
  }

#if defined(LIBXR_STM32_UART_GPDMA)
  tx_service_.Invoke(TX_EVENT_START_RX, false, [this](uint32_t events, bool in_isr)
                     { HandleTxService(events, in_isr); });
#else
  SetRxDMA(false);
#endif
}

ErrorCode STM32UART::SetConfig(UART::Configuration config, bool in_isr)
{
  if (config.baudrate == 0U)
  {
    return ErrorCode::ARG_ERR;
  }

  switch (config.parity)
  {
    case UART::Parity::NO_PARITY:
    case UART::Parity::EVEN:
    case UART::Parity::ODD:
      break;
    default:
      return ErrorCode::NOT_SUPPORT;
  }

  if (!Stm32DataBitsSupported(config))
  {
    return ErrorCode::ARG_ERR;
  }

  if (!Stm32StopBitsSupported(config))
  {
    return ErrorCode::ARG_ERR;
  }

  ConfigState expected = ConfigState::EMPTY;
  if (!config_state_.compare_exchange_strong(expected, ConfigState::RESERVED,
                                             std::memory_order_acq_rel,
                                             std::memory_order_acquire))
  {
    return ErrorCode::BUSY;
  }

  pending_config_ = config;
  tx_service_.Invoke(TX_EVENT_CONFIG, in_isr, [this](uint32_t events, bool owner_in_isr)
                     { HandleTxService(events, owner_in_isr); });
  return ErrorCode::OK;
}

void STM32UART::ApplyConfig(UART::Configuration config, bool in_isr)
{
  uart_handle_->Init.BaudRate = config.baudrate;

  switch (config.parity)
  {
    case UART::Parity::NO_PARITY:
      uart_handle_->Init.Parity = UART_PARITY_NONE;
      uart_handle_->Init.WordLength = UART_WORDLENGTH_8B;
      break;
    case UART::Parity::EVEN:
      uart_handle_->Init.Parity = UART_PARITY_EVEN;
      uart_handle_->Init.WordLength =
          config.data_bits == 7U ? UART_WORDLENGTH_8B : UART_WORDLENGTH_9B;
      break;
    case UART::Parity::ODD:
      uart_handle_->Init.Parity = UART_PARITY_ODD;
      uart_handle_->Init.WordLength =
          config.data_bits == 7U ? UART_WORDLENGTH_8B : UART_WORDLENGTH_9B;
      break;
    default:
      DEV_ASSERT_FROM_CALLBACK(false, in_isr);
      return;
  }

  switch (config.stop_bits)
  {
    case 1:
      uart_handle_->Init.StopBits = UART_STOPBITS_1;
      break;
    case 2:
      uart_handle_->Init.StopBits = UART_STOPBITS_2;
      break;
    default:
      DEV_ASSERT_FROM_CALLBACK(false, in_isr);
      return;
  }

#if defined(LIBXR_STM32_UART_GPDMA)
  // 通道已停止，直接配置寄存器，避免在中断中等待 HAL tick。
  // Channels are stopped; configure registers without waiting for HAL ticks in an ISR.
  __HAL_UART_DISABLE(uart_handle_);
  [[maybe_unused]] const auto uart_set_config_result = UART_SetConfig(uart_handle_);
  REQUIRE_FROM_CALLBACK(uart_set_config_result == HAL_OK, in_isr);
  CLEAR_BIT(uart_handle_->Instance->CR2, USART_CR2_LINEN | USART_CR2_CLKEN);
  CLEAR_BIT(uart_handle_->Instance->CR3,
            USART_CR3_SCEN | USART_CR3_HDSEL | USART_CR3_IREN);
  __HAL_UART_ENABLE(uart_handle_);
#else
  [[maybe_unused]] const auto uart_init_result = HAL_UART_Init(uart_handle_);
  REQUIRE_FROM_CALLBACK(uart_init_result == HAL_OK, in_isr);
#endif
}

void STM32UART::TryApplyConfig(bool in_isr)
{
  if (config_state_.load(std::memory_order_acquire) != ConfigState::PUBLISHED)
  {
    return;
  }

  if (tx_busy_.IsSet())
  {
    return;
  }

#if defined(LIBXR_STM32_UART_GPDMA)
  abort_pending_ = true;
  BeginAbort(in_isr);
#else
  ApplyConfig(pending_config_, in_isr);
  last_rx_pos_ = 0U;
  SetRxDMA(in_isr);
  config_state_.store(ConfigState::EMPTY, std::memory_order_release);
#endif
}

void STM32UART::SetRxDMA(bool in_isr)
{
  if ((uart_handle_->Init.Mode & UART_MODE_RX) == UART_MODE_RX)
  {
    ASSERT(uart_handle_->hdmarx != NULL);

#if defined(LIBXR_STM32_UART_GPDMA)
    [[maybe_unused]] const auto start_linked_list_dma_rx_result =
        gpdma_adapter_.StartLinkedListDmaRx(static_cast<uint8_t*>(dma_buff_rx_.addr_),
                                            dma_buff_rx_.size_, in_isr);
    REQUIRE_FROM_CALLBACK(start_linked_list_dma_rx_result == HAL_OK, in_isr);
#else
    uart_handle_->hdmarx->Init.Mode = DMA_CIRCULAR;
    [[maybe_unused]] const auto dma_init_result = HAL_DMA_Init(uart_handle_->hdmarx);
    REQUIRE_FROM_CALLBACK(dma_init_result == HAL_OK, in_isr);

    [[maybe_unused]] const auto uart_ex_receive_to_idle_dma_result =
        HAL_UARTEx_ReceiveToIdle_DMA(uart_handle_,
                                     reinterpret_cast<uint8_t*>(dma_buff_rx_.addr_),
                                     dma_buff_rx_.size_);
    REQUIRE_FROM_CALLBACK(uart_ex_receive_to_idle_dma_result == HAL_OK, in_isr);
#endif
  }
}

void STM32UART::HandleRxData(bool in_isr)
{
  const size_t dma_size = dma_buff_rx_.size_;
  if (dma_size == 0U)
  {
    return;
  }

  auto* const rx_buf = static_cast<uint8_t*>(dma_buff_rx_.addr_);

#if defined(LIBXR_STM32_UART_GPDMA)
  const uintptr_t destination =
      reinterpret_cast<uintptr_t>(gpdma_adapter_.GetLinkedListDmaRxProducer());
  const uintptr_t begin = reinterpret_cast<uintptr_t>(rx_buf);
  DEV_ASSERT_FROM_CALLBACK(destination >= begin && destination - begin <= dma_size,
                           in_isr);
  const size_t curr_pos = destination - begin;
#else
  const size_t remaining = __HAL_DMA_GET_COUNTER(uart_handle_->hdmarx);
  DEV_ASSERT_FROM_CALLBACK(remaining <= dma_size, in_isr);
  const size_t curr_pos = remaining == 0U ? dma_size : dma_size - remaining;
#endif
  const size_t last_pos = last_rx_pos_;
  DEV_ASSERT_FROM_CALLBACK(last_pos < dma_size, in_isr);

  STM32_InvalidateDCacheByAddr(rx_buf, dma_size);

  if (curr_pos != last_pos)
  {
    const size_t first_size =
        curr_pos > last_pos ? curr_pos - last_pos : dma_size - last_pos;
    const size_t second_size = curr_pos > last_pos ? 0U : curr_pos;
    auto queue = _read_port.GetReadQueue(in_isr);
    size_t accepted = std::min(first_size + second_size, queue.EmptySize());

    if (accepted != 0U)
    {
      const size_t first_accepted = std::min(first_size, accepted);
      if (first_accepted != 0U)
      {
        [[maybe_unused]] const auto push_batch_result =
            queue.PushBatch(rx_buf + last_pos, first_accepted);
        DEV_ASSERT_FROM_CALLBACK(push_batch_result == ErrorCode::OK, in_isr);
        accepted -= first_accepted;
      }
      if (accepted != 0U)
      {
        [[maybe_unused]] const auto push_batch_result = queue.PushBatch(rx_buf, accepted);
        DEV_ASSERT_FROM_CALLBACK(push_batch_result == ErrorCode::OK, in_isr);
      }
    }

    last_rx_pos_ = curr_pos == dma_size ? 0U : curr_pos;
    queue.Publish();
  }
}

void STM32UART::TxCompleteIRQHandler()
{
  tx_service_.Invoke(TX_EVENT_DONE, true, [this](uint32_t events, bool in_isr)
                     { HandleTxService(events, in_isr); });
}

void STM32UART::RxEventIRQHandler()
{
  tx_service_.Invoke(TX_EVENT_RX_WORK, true, [this](uint32_t events, bool in_isr)
                     { HandleTxService(events, in_isr); });
}

void STM32UART::ErrorIRQHandler()
{
  tx_service_.Invoke(TX_EVENT_ERROR, true, [this](uint32_t events, bool in_isr)
                     { HandleTxService(events, in_isr); });
}

void STM32UART::AbortCompleteIRQHandler()
{
  tx_service_.Invoke(TX_EVENT_ABORT, true, [this](uint32_t events, bool in_isr)
                     { HandleTxService(events, in_isr); });
}

// HAL 在串口发送完成后调用此入口，此处只通知后端处理器。
// HAL invokes this entry at UART transmission completion; only notify the service.
void STM32_UART_ISR_Handler_TX_CPLT(stm32_uart_id_t id)
{
  auto uart = STM32UART::map[id];
  if (uart != nullptr)
  {
    uart->TxCompleteIRQHandler();
  }
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t)
{
  auto uart = STM32UART::map[stm32_uart_get_id(huart->Instance)];
  if (uart != nullptr)
  {
    uart->RxEventIRQHandler();
  }
}

extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart)
{
  STM32_UART_ISR_Handler_TX_CPLT(stm32_uart_get_id(huart->Instance));
}

extern "C" __attribute__((used)) void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart)
{
  const auto id = stm32_uart_get_id(huart->Instance);
  if (id == STM32_UART_ID_ERROR)
  {
    return;
  }
  auto* uart = STM32UART::map[id];
  if (uart != nullptr)
  {
    uart->ErrorIRQHandler();
  }
}

extern "C" void HAL_UART_AbortCpltCallback(UART_HandleTypeDef* huart)
{
  auto uart = STM32UART::map[stm32_uart_get_id(huart->Instance)];
  if (uart != nullptr)
  {
    uart->AbortCompleteIRQHandler();
  }
}

void STM32UART::BeginAbort(bool in_isr)
{
#if defined(LIBXR_STM32_UART_GPDMA)
  gpdma_adapter_.CloseTxTerminalSource();
  if (uart_handle_->hdmatx != nullptr)
  {
    [[maybe_unused]] const auto launch_stop_result =
        gpdma_adapter_.LaunchStop(uart_handle_->hdmatx, DmaAbortCallback, in_isr);
    REQUIRE_FROM_CALLBACK(launch_stop_result, in_isr);
  }
  if (uart_handle_->hdmarx != nullptr)
  {
    [[maybe_unused]] const auto launch_stop_result =
        gpdma_adapter_.LaunchStop(uart_handle_->hdmarx, DmaAbortCallback, in_isr);
    REQUIRE_FROM_CALLBACK(launch_stop_result, in_isr);
  }
  if (gpdma_adapter_.AllStopsComplete())
  {
    tx_service_.Publish(TX_EVENT_ABORT);
  }
#else
  [[maybe_unused]] const auto uart_abort_it_result = HAL_UART_Abort_IT(uart_handle_);
  REQUIRE_FROM_CALLBACK(uart_abort_it_result == HAL_OK, in_isr);
#endif
}

#if defined(LIBXR_STM32_UART_GPDMA)
void STM32UART::DmaAbortCallback(DMA_HandleTypeDef* dma_handle)
{
  auto* handle = static_cast<UART_HandleTypeDef*>(dma_handle->Parent);
  const auto id = stm32_uart_get_id(handle->Instance);
  if (id != STM32_UART_ID_ERROR && map[id] != nullptr)
  {
    map[id]->AbortCompleteIRQHandler();
  }
}
#endif

void STM32UART::StartTxDma(bool in_isr)
{
  const size_t size = dma_buff_tx_.GetActiveLength();
  DEV_ASSERT_FROM_CALLBACK(size != 0U && size <= dma_buff_tx_.Size(), in_isr);

  STM32_CleanDCacheByAddr(dma_buff_tx_.ActiveBuffer(), size);
  tx_busy_.Set();
  const HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(
      uart_handle_, static_cast<uint8_t*>(dma_buff_tx_.ActiveBuffer()), size);
  REQUIRE_FROM_CALLBACK(status == HAL_OK, in_isr);
}

#endif
