// NOLINTBEGIN(cppcoreguidelines-pro-type-cstyle-cast,performance-no-int-to-ptr)

#include "ch32_uart.hpp"

#include <algorithm>

#include "ch32_dma.hpp"
#include "ch32_gpio.hpp"

using namespace LibXR;

namespace
{

bool Ch32DataBitsSupported(const UART::Configuration& config)
{
  if (config.parity == UART::Parity::NO_PARITY)
  {
    return config.data_bits == 8U;
  }

  return (config.parity == UART::Parity::EVEN || config.parity == UART::Parity::ODD) &&
         (config.data_bits == 7U || config.data_bits == 8U);
}

bool Ch32StopBitsSupported(const UART::Configuration& config)
{
  return config.stop_bits == 1U || config.stop_bits == 2U;
}

}  // namespace

CH32UART* CH32UART::map_[ch32_uart_id_t::CH32_UART_NUMBER] = {nullptr};

CH32UART::CH32UART(ch32_uart_id_t id, RawData dma_rx, RawData dma_tx,
                   GPIO_TypeDef* tx_gpio_port, uint16_t tx_gpio_pin,
                   GPIO_TypeDef* rx_gpio_port, uint16_t rx_gpio_pin, uint32_t pin_remap,
                   uint32_t tx_queue_size, UART::Configuration config)
    : UART(&_read_port, &_write_port),
      id_(id),
      _read_port(dma_rx.size_),
      _write_port(tx_queue_size, dma_tx.size_ / 2),
      dma_buff_rx_(dma_rx),
      dma_buff_tx_(dma_tx),
      instance_(ch32_uart_get_instance_id(id)),
      dma_rx_channel_(CH32_UART_RX_DMA_CHANNEL_MAP[id]),
      dma_tx_channel_(CH32_UART_TX_DMA_CHANNEL_MAP[id])
{
  map_[id] = this;

  bool tx_enable = dma_tx.size_ > 1;
  bool rx_enable = dma_rx.size_ > 0;

  ASSERT(tx_enable || rx_enable);
  if (tx_enable)
  {
    ASSERT(tx_queue_size > 0U);
    ASSERT(dma_tx_channel_ != nullptr);
    ASSERT(CH32_UART_TX_DMA_IT_MAP[id] != 0);
  }
  if (rx_enable)
  {
    ASSERT(dma_rx_channel_ != nullptr);
    ASSERT(CH32_UART_RX_DMA_IT_TC_MAP[id] != 0);
    ASSERT(CH32_UART_RX_DMA_IT_HT_MAP[id] != 0);
  }

  // TX 使用复用推挽，RX 使用浮空输入 / TX alternate push-pull; RX floating input.
  GPIO_InitTypeDef gpio_init = {};
  gpio_init.GPIO_Speed = GPIO_Speed_50MHz;

  if (tx_enable)
  {
    RCC_APB2PeriphClockCmd(ch32_get_gpio_periph(tx_gpio_port), ENABLE);
    gpio_init.GPIO_Pin = tx_gpio_pin;
    gpio_init.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(tx_gpio_port, &gpio_init);
    (*write_port_) = WriteFun;
  }

  if (rx_enable)
  {
    RCC_APB2PeriphClockCmd(ch32_get_gpio_periph(rx_gpio_port), ENABLE);
    gpio_init.GPIO_Pin = rx_gpio_pin;
    gpio_init.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(rx_gpio_port, &gpio_init);
  }

  // 可选引脚重映射 / Optional pin remapping.
  if (pin_remap != 0)
  {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(pin_remap, ENABLE);
  }

  // 开启串口外设时钟 / Enable the UART peripheral clock.
  if (CH32_UART_APB_MAP[id] == 1)
  {
    RCC_APB1PeriphClockCmd(CH32_UART_RCC_PERIPH_MAP[id], ENABLE);
  }
  else if (CH32_UART_APB_MAP[id] == 2)
  {
    RCC_APB2PeriphClockCmd(CH32_UART_RCC_PERIPH_MAP[id], ENABLE);
  }
  else
  {
    DEV_ASSERT(false);
  }
  RCC_AHBPeriphClockCmd(CH32_UART_RCC_PERIPH_MAP_DMA[id], ENABLE);

  // 设置初始串口参数 / Apply initial UART settings.
  ASSERT(config.baudrate > 0U);
  ASSERT(Ch32DataBitsSupported(config));
  ASSERT(Ch32StopBitsSupported(config));
  USART_InitTypeDef usart_cfg = {};
  usart_cfg.USART_BaudRate = config.baudrate;
  usart_cfg.USART_StopBits =
      (config.stop_bits == 2) ? USART_StopBits_2 : USART_StopBits_1;
  switch (config.parity)
  {
    case UART::Parity::NO_PARITY:
      usart_cfg.USART_Parity = USART_Parity_No;
      usart_cfg.USART_WordLength = USART_WordLength_8b;
      break;
    case UART::Parity::EVEN:
      usart_cfg.USART_Parity = USART_Parity_Even;
      usart_cfg.USART_WordLength =
          config.data_bits == 7U ? USART_WordLength_8b : USART_WordLength_9b;
      break;
    case UART::Parity::ODD:
      usart_cfg.USART_Parity = USART_Parity_Odd;
      usart_cfg.USART_WordLength =
          config.data_bits == 7U ? USART_WordLength_8b : USART_WordLength_9b;
      break;
    default:
      ASSERT(false);
  }

  usart_cfg.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  usart_cfg.USART_Mode =
      (tx_enable ? USART_Mode_Tx : 0) | (rx_enable ? USART_Mode_Rx : 0);
  uart_mode_ = usart_cfg.USART_Mode;
  USART_Init(instance_, &usart_cfg);

  // 配置收发 DMA / Configure RX and TX DMA.
  DMA_InitTypeDef dma_init = {};
  dma_init.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  dma_init.DMA_MemoryInc = DMA_MemoryInc_Enable;
  dma_init.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  dma_init.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  dma_init.DMA_Priority = DMA_Priority_High;
  dma_init.DMA_M2M = DMA_M2M_Disable;

  if (rx_enable)
  {
    ch32_dma_callback_t rx_cb_fun = [](void* arg)
    { reinterpret_cast<CH32UART*>(arg)->RxDmaIRQHandler(); };

    ch32_dma_register_callback(ch32_dma_get_id(CH32_UART_RX_DMA_CHANNEL_MAP[id]),
                               rx_cb_fun, this);

    DMA_DeInit(dma_rx_channel_);
    dma_init.DMA_PeripheralBaseAddr = (uint32_t)&instance_->DATAR;
    dma_init.DMA_MemoryBaseAddr = (uint32_t)dma_buff_rx_.addr_;
    dma_init.DMA_DIR = DMA_DIR_PeripheralSRC;
    dma_init.DMA_Mode = DMA_Mode_Circular;
    dma_init.DMA_BufferSize = dma_buff_rx_.size_;
    DMA_Init(dma_rx_channel_, &dma_init);
    DMA_Cmd(dma_rx_channel_, ENABLE);
    DMA_ITConfig(dma_rx_channel_, DMA_IT_TC, ENABLE);
    DMA_ITConfig(dma_rx_channel_, DMA_IT_HT, ENABLE);
    USART_DMACmd(instance_, USART_DMAReq_Rx, ENABLE);
  }

  if (tx_enable)
  {
    ch32_dma_callback_t tx_cb_fun = [](void* arg)
    { reinterpret_cast<CH32UART*>(arg)->TxDmaIRQHandler(); };

    ch32_dma_register_callback(ch32_dma_get_id(CH32_UART_TX_DMA_CHANNEL_MAP[id]),
                               tx_cb_fun, this);
    DMA_DeInit(dma_tx_channel_);
    dma_init.DMA_PeripheralBaseAddr = (u32)(&instance_->DATAR);
    dma_init.DMA_MemoryBaseAddr = 0;
    dma_init.DMA_DIR = DMA_DIR_PeripheralDST;
    dma_init.DMA_Mode = DMA_Mode_Normal;
    dma_init.DMA_BufferSize = 0;
    DMA_Init(dma_tx_channel_, &dma_init);
    DMA_ITConfig(dma_tx_channel_, DMA_IT_TC, ENABLE);
    USART_DMACmd(instance_, USART_DMAReq_Tx, ENABLE);
  }

  // 开启串口及对应中断 / Enable the UART and its interrupts.
  USART_Cmd(instance_, ENABLE);

  if (rx_enable)
  {
    USART_ITConfig(instance_, USART_IT_IDLE, ENABLE);
    NVIC_EnableIRQ(CH32_DMA_IRQ_MAP[ch32_dma_get_id(dma_rx_channel_)]);
  }

  if (tx_enable)
  {
    NVIC_EnableIRQ(CH32_DMA_IRQ_MAP[ch32_dma_get_id(dma_tx_channel_)]);
  }

  NVIC_EnableIRQ(CH32_UART_IRQ_MAP[id]);
}

ErrorCode CH32UART::SetConfig(UART::Configuration config, bool in_isr)
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

  if (!Ch32DataBitsSupported(config))
  {
    return ErrorCode::ARG_ERR;
  }

  if (!Ch32StopBitsSupported(config))
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

void CH32UART::ApplyConfig(UART::Configuration config)
{
  USART_ITConfig(instance_, USART_IT_TC, DISABLE);

  USART_InitTypeDef usart_cfg = {};
  usart_cfg.USART_BaudRate = config.baudrate;
  usart_cfg.USART_StopBits =
      (config.stop_bits == 2) ? USART_StopBits_2 : USART_StopBits_1;

  switch (config.parity)
  {
    case UART::Parity::NO_PARITY:
      usart_cfg.USART_Parity = USART_Parity_No;
      usart_cfg.USART_WordLength = USART_WordLength_8b;
      break;
    case UART::Parity::EVEN:
      usart_cfg.USART_Parity = USART_Parity_Even;
      usart_cfg.USART_WordLength =
          config.data_bits == 7U ? USART_WordLength_8b : USART_WordLength_9b;
      break;
    case UART::Parity::ODD:
      usart_cfg.USART_Parity = USART_Parity_Odd;
      usart_cfg.USART_WordLength =
          config.data_bits == 7U ? USART_WordLength_8b : USART_WordLength_9b;
      break;
    default:
      DEV_ASSERT(false);
      return;
  }

  usart_cfg.USART_Mode = uart_mode_;
  usart_cfg.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_Init(instance_, &usart_cfg);

  if (uart_mode_ & USART_Mode_Rx)
  {
    USART_DMACmd(instance_, USART_DMAReq_Rx, ENABLE);
    USART_ITConfig(instance_, USART_IT_IDLE, ENABLE);
  }

  if (uart_mode_ & USART_Mode_Tx)
  {
    USART_DMACmd(instance_, USART_DMAReq_Tx, ENABLE);
  }

  USART_Cmd(instance_, ENABLE);
}

void CH32UART::WriteFun(WritePort& port, bool in_isr)
{
  auto* uart = LibXR::ContainerOf(&port, &CH32UART::_write_port);

  uart->tx_service_.Invoke(TX_EVENT_WRITE, in_isr,
                           [uart](uint32_t events, bool owner_in_isr)
                           { uart->HandleTxService(events, owner_in_isr); });
}

void CH32UART::HandleTxService(uint32_t events, bool in_isr)
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

  if ((events & TX_EVENT_DMA_DONE) != 0U)
  {
    HandleTxDone(in_isr);
  }

  if ((events & TX_EVENT_RX_WORK) != 0U)
  {
    HandleRxData(in_isr);
  }

  if ((events & (TX_EVENT_CONFIG | TX_EVENT_DMA_DONE | TX_EVENT_TC)) != 0U)
  {
    TryApplyConfig(in_isr);
  }

  if ((uart_mode_ & USART_Mode_Tx) != 0U &&
      (events & (TX_EVENT_WRITE | TX_EVENT_DMA_DONE | TX_EVENT_CONFIG | TX_EVENT_TC)) !=
          0U &&
      config_state_.load(std::memory_order_acquire) == ConfigState::EMPTY)
  {
    FillTx(in_isr);
  }
}

void CH32UART::FillTx(bool in_isr)
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

void CH32UART::HandleTxDone(bool in_isr)
{
  UNUSED(in_isr);
  if (!tx_busy_.IsSet())
  {
    return;
  }

  tx_busy_.Clear();
  dma_buff_tx_.SetActiveLength(0U);
}

void CH32UART::TryApplyConfig(bool in_isr)
{
  if (config_state_.load(std::memory_order_acquire) != ConfigState::PUBLISHED)
  {
    return;
  }

  if (tx_busy_.IsSet())
  {
    return;
  }

  if ((uart_mode_ & USART_Mode_Tx) != 0U &&
      USART_GetFlagStatus(instance_, USART_FLAG_TC) == RESET)
  {
    USART_ITConfig(instance_, USART_IT_TC, ENABLE);
    return;
  }

  const UART::Configuration config = pending_config_;
  ApplyConfig(config);

  if ((uart_mode_ & USART_Mode_Rx) != 0U && dma_buff_rx_.size_ != 0U)
  {
    const size_t remaining = dma_rx_channel_->CNTR;
    DEV_ASSERT_FROM_CALLBACK(remaining <= dma_buff_rx_.size_, in_isr);
    const size_t position =
        remaining == 0U ? dma_buff_rx_.size_ : dma_buff_rx_.size_ - remaining;
    last_rx_pos_ = position == dma_buff_rx_.size_ ? 0U : position;
  }

  config_state_.store(ConfigState::EMPTY, std::memory_order_release);
  UNUSED(in_isr);
}

void CH32UART::StartTxDma(bool in_isr)
{
  const size_t size = dma_buff_tx_.GetActiveLength();
  DEV_ASSERT_FROM_CALLBACK(size != 0U && size <= dma_buff_tx_.Size(), in_isr);

  DMA_Cmd(dma_tx_channel_, DISABLE);
  dma_tx_channel_->MADDR = reinterpret_cast<uint32_t>(dma_buff_tx_.ActiveBuffer());
  dma_tx_channel_->CNTR = size;
  tx_busy_.Set();
  DMA_Cmd(dma_tx_channel_, ENABLE);
  UNUSED(in_isr);
}

void CH32UART::HandleRxData(bool in_isr)
{
  const size_t dma_size = dma_buff_rx_.size_;
  if (dma_size == 0U)
  {
    return;
  }

  const size_t remaining = dma_rx_channel_->CNTR;
  DEV_ASSERT_FROM_CALLBACK(remaining <= dma_size, in_isr);
  const size_t curr_pos = remaining == 0U ? dma_size : dma_size - remaining;
  const size_t last_pos = last_rx_pos_;
  DEV_ASSERT_FROM_CALLBACK(last_pos < dma_size, in_isr);

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
        [[maybe_unused]] const auto push_batch_result = queue.PushBatch(
            static_cast<const uint8_t*>(dma_buff_rx_.addr_) + last_pos, first_accepted);
        DEV_ASSERT_FROM_CALLBACK(push_batch_result == ErrorCode::OK, in_isr);
        accepted -= first_accepted;
      }
      if (accepted != 0U)
      {
        [[maybe_unused]] const auto push_batch_result =
            queue.PushBatch(static_cast<const uint8_t*>(dma_buff_rx_.addr_), accepted);
        DEV_ASSERT_FROM_CALLBACK(push_batch_result == ErrorCode::OK, in_isr);
      }
    }

    last_rx_pos_ = curr_pos == dma_size ? 0U : curr_pos;
    queue.Publish();
  }
}

void CH32UART::UartIRQHandler()
{
  const bool idle = USART_GetITStatus(instance_, USART_IT_IDLE) != RESET;
  const bool tc = USART_GetITStatus(instance_, USART_IT_TC) != RESET;

  if (idle)
  {
    USART_ReceiveData(instance_);
    tx_service_.Invoke(TX_EVENT_RX_WORK, true, [this](uint32_t events, bool in_isr)
                       { HandleTxService(events, in_isr); });
  }

  if (tc)
  {
    if (config_state_.load(std::memory_order_acquire) != ConfigState::EMPTY)
    {
      USART_ITConfig(instance_, USART_IT_TC, DISABLE);
      tx_service_.Invoke(TX_EVENT_TC, true, [this](uint32_t events, bool in_isr)
                         { HandleTxService(events, in_isr); });
    }
    else
    {
      USART_ITConfig(instance_, USART_IT_TC, DISABLE);
    }
  }
}

extern "C" void ch32_uart_isr_handler_idle(ch32_uart_id_t id)
{
  auto uart = CH32UART::map_[id];
  if (uart)
  {
    uart->UartIRQHandler();
  }
}

extern "C" void ch32_uart_isr_handler_tx_cplt(CH32UART* uart) { uart->TxDmaIRQHandler(); }

void CH32UART::TxDmaIRQHandler()
{
  if (DMA_GetITStatus(CH32_UART_TX_DMA_IT_MAP[id_]) == RESET)
  {
    return;
  }

  if (dma_tx_channel_->CNTR == 0)
  {
    DMA_ClearITPendingBit(CH32_UART_TX_DMA_IT_MAP[id_]);
    tx_service_.Invoke(TX_EVENT_DMA_DONE, true, [this](uint32_t events, bool in_isr)
                       { HandleTxService(events, in_isr); });
  }
}

/**
 * @brief 清除接收 DMA 中断标志并通知后端 / Clear RX DMA flags and notify the backend.
 */
void CH32UART::RxDmaIRQHandler()
{
  if (DMA_GetITStatus(CH32_UART_RX_DMA_IT_HT_MAP[id_]) == SET)
  {
    DMA_ClearITPendingBit(CH32_UART_RX_DMA_IT_HT_MAP[id_]);
    tx_service_.Invoke(TX_EVENT_RX_WORK, true, [this](uint32_t events, bool in_isr)
                       { HandleTxService(events, in_isr); });
  }

  if (DMA_GetITStatus(CH32_UART_RX_DMA_IT_TC_MAP[id_]) == SET)
  {
    DMA_ClearITPendingBit(CH32_UART_RX_DMA_IT_TC_MAP[id_]);
    tx_service_.Invoke(TX_EVENT_RX_WORK, true, [this](uint32_t events, bool in_isr)
                       { HandleTxService(events, in_isr); });
  }
}

#if defined(USART1)
// NOLINTNEXTLINE(readability-identifier-naming)
extern "C" void USART1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
// NOLINTNEXTLINE(readability-identifier-naming)
extern "C" void USART1_IRQHandler(void) { ch32_uart_isr_handler_idle(CH32_USART1); }
#endif
#if defined(USART2)
// NOLINTNEXTLINE(readability-identifier-naming)
extern "C" void USART2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
// NOLINTNEXTLINE(readability-identifier-naming)
extern "C" void USART2_IRQHandler(void) { ch32_uart_isr_handler_idle(CH32_USART2); }
#endif
#if defined(USART3)
// NOLINTNEXTLINE(readability-identifier-naming)
extern "C" void USART3_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
// NOLINTNEXTLINE(readability-identifier-naming)
extern "C" void USART3_IRQHandler(void) { ch32_uart_isr_handler_idle(CH32_USART3); }
#endif
#if defined(USART4)
// NOLINTNEXTLINE(readability-identifier-naming)
extern "C" void USART4_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
// NOLINTNEXTLINE(readability-identifier-naming)
extern "C" void USART4_IRQHandler(void) { ch32_uart_isr_handler_idle(CH32_USART4); }
#endif
#if defined(USART5)
// NOLINTNEXTLINE(readability-identifier-naming)
extern "C" void USART5_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
// NOLINTNEXTLINE(readability-identifier-naming)
extern "C" void USART5_IRQHandler(void) { ch32_uart_isr_handler_idle(CH32_USART5); }
#endif
#if defined(USART6)
// NOLINTNEXTLINE(readability-identifier-naming)
extern "C" void USART6_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
// NOLINTNEXTLINE(readability-identifier-naming)
extern "C" void USART6_IRQHandler(void) { ch32_uart_isr_handler_idle(CH32_USART6); }
#endif
#if defined(USART7)
// NOLINTNEXTLINE(readability-identifier-naming)
extern "C" void USART7_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
// NOLINTNEXTLINE(readability-identifier-naming)
extern "C" void USART7_IRQHandler(void) { ch32_uart_isr_handler_idle(CH32_USART7); }
#endif
#if defined(USART8)
// NOLINTNEXTLINE(readability-identifier-naming)
extern "C" void USART8_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
// NOLINTNEXTLINE(readability-identifier-naming)
extern "C" void USART8_IRQHandler(void) { ch32_uart_isr_handler_idle(CH32_USART8); }
#endif
#if defined(UART1)
// NOLINTNEXTLINE(readability-identifier-naming)
extern "C" void UART1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
// NOLINTNEXTLINE(readability-identifier-naming)
extern "C" void UART1_IRQHandler(void) { ch32_uart_isr_handler_idle(CH32_UART1); }
#endif
#if defined(UART2)
// NOLINTNEXTLINE(readability-identifier-naming)
extern "C" void UART2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
// NOLINTNEXTLINE(readability-identifier-naming)
extern "C" void UART2_IRQHandler(void) { ch32_uart_isr_handler_idle(CH32_UART2); }
#endif
#if defined(UART3)
// NOLINTNEXTLINE(readability-identifier-naming)
extern "C" void UART3_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
// NOLINTNEXTLINE(readability-identifier-naming)
extern "C" void UART3_IRQHandler(void) { ch32_uart_isr_handler_idle(CH32_UART3); }
#endif
#if defined(UART4)
// NOLINTNEXTLINE(readability-identifier-naming)
extern "C" void UART4_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
// NOLINTNEXTLINE(readability-identifier-naming)
extern "C" void UART4_IRQHandler(void) { ch32_uart_isr_handler_idle(CH32_UART4); }
#endif
#if defined(UART5)
// NOLINTNEXTLINE(readability-identifier-naming)
extern "C" void UART5_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
// NOLINTNEXTLINE(readability-identifier-naming)
extern "C" void UART5_IRQHandler(void) { ch32_uart_isr_handler_idle(CH32_UART5); }
#endif
#if defined(UART6)
// NOLINTNEXTLINE(readability-identifier-naming)
extern "C" void UART6_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
// NOLINTNEXTLINE(readability-identifier-naming)
extern "C" void UART6_IRQHandler(void) { ch32_uart_isr_handler_idle(CH32_UART6); }
#endif
#if defined(UART7)
// NOLINTNEXTLINE(readability-identifier-naming)
extern "C" void UART7_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
// NOLINTNEXTLINE(readability-identifier-naming)
extern "C" void UART7_IRQHandler(void) { ch32_uart_isr_handler_idle(CH32_UART7); }
#endif
#if defined(UART8)
// NOLINTNEXTLINE(readability-identifier-naming)
extern "C" void UART8_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
// NOLINTNEXTLINE(readability-identifier-naming)
extern "C" void UART8_IRQHandler(void) { ch32_uart_isr_handler_idle(CH32_UART8); }
#endif

// NOLINTEND(cppcoreguidelines-pro-type-cstyle-cast,performance-no-int-to-ptr)
