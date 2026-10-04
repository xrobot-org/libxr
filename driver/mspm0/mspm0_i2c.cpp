#include "mspm0_i2c.hpp"

#include <ti/driverlib/dl_dma.h>

#include <cstdint>
#include <cstring>

using namespace LibXR;

namespace
{

constexpr uint32_t MSPM0_I2C_WAIT_IDLE_TIMEOUT = 300000;
constexpr uint32_t MSPM0_I2C_WAIT_BUS_TIMEOUT = 300000;
constexpr uint32_t MSPM0_I2C_WAIT_FIFO_TIMEOUT = 300000;
constexpr uint32_t MSPM0_I2C_POLLING_ATTEMPTS = 2;
constexpr uint32_t MSPM0_I2C_MEMREAD_RS_ATTEMPTS = 2;
constexpr uint32_t MSPM0_I2C_MEMREAD_FALLBACK_ATTEMPTS = 3;

constexpr uint16_t MSPM0_I2C_MAX_TRANSFER_SIZE = 0x0FFF;

// 两个 DMA 绑定都存在时才启用 DMA；未绑定时保留轮询 I2C。
// Enable DMA only when both channels are bound; otherwise retain polling I2C.
#if defined(DMA_CH_TX_CHAN_ID) != defined(DMA_CH_RX_CHAN_ID)
#error "MSPM0I2C requires both TX and RX DMA bindings, or neither."
#endif
#if defined(DMA_CH_TX_CHAN_ID) && defined(DMA_CH_RX_CHAN_ID)
constexpr bool I2C_DMA_CONFIGURED = true;
constexpr uint8_t I2C_DMA_TX_CHANNEL = DMA_CH_TX_CHAN_ID;
constexpr uint8_t I2C_DMA_RX_CHANNEL = DMA_CH_RX_CHAN_ID;
static_assert(DMA_CH_TX_CHAN_ID < DMA_SYS_N_DMA_CHANNEL);
static_assert(DMA_CH_RX_CHAN_ID < DMA_SYS_N_DMA_CHANNEL);
static_assert(DMA_CH_TX_CHAN_ID != DMA_CH_RX_CHAN_ID);
#else
constexpr bool I2C_DMA_CONFIGURED = false;
// Unused placeholders: no DMA register may be accessed in polling-only mode.
constexpr uint8_t I2C_DMA_TX_CHANNEL = 0U;
constexpr uint8_t I2C_DMA_RX_CHANNEL = 0U;
#endif

constexpr DL_DMA_Config MSPM0_I2C_DMA_TX_CONFIG_BASE = {
    .trigger = 0U,
    .triggerType = DL_DMA_TRIGGER_TYPE_EXTERNAL,
    .transferMode = DL_DMA_SINGLE_TRANSFER_MODE,
    .extendedMode = DL_DMA_NORMAL_MODE,
    .srcWidth = DL_DMA_WIDTH_BYTE,
    .destWidth = DL_DMA_WIDTH_BYTE,
    .srcIncrement = DL_DMA_ADDR_INCREMENT,
    .destIncrement = DL_DMA_ADDR_UNCHANGED,
};

constexpr DL_DMA_Config MSPM0_I2C_DMA_RX_CONFIG_BASE = {
    .trigger = 0U,
    .triggerType = DL_DMA_TRIGGER_TYPE_EXTERNAL,
    .transferMode = DL_DMA_SINGLE_TRANSFER_MODE,
    .extendedMode = DL_DMA_NORMAL_MODE,
    .srcWidth = DL_DMA_WIDTH_BYTE,
    .destWidth = DL_DMA_WIDTH_BYTE,
    .srcIncrement = DL_DMA_ADDR_UNCHANGED,
    .destIncrement = DL_DMA_ADDR_INCREMENT,
};

constexpr uint32_t mspm0_i2c_dma_channel_mask(uint8_t channel_id)
{
  return (1UL << channel_id);
}

bool mspm0_i2c_resolve_dma_triggers(I2C_Regs* instance, uint8_t& tx_trigger,
                                    uint8_t& rx_trigger)
{
#if defined(I2C0_BASE) && defined(DMA_I2C0_TX_TRIG) && defined(DMA_I2C0_RX_TRIG)
  if (instance == I2C0)
  {
    tx_trigger = DMA_I2C0_TX_TRIG;
    rx_trigger = DMA_I2C0_RX_TRIG;
    return true;
  }
#endif
#if defined(I2C1_BASE) && defined(DMA_I2C1_TX_TRIG) && defined(DMA_I2C1_RX_TRIG)
  if (instance == I2C1)
  {
    tx_trigger = DMA_I2C1_TX_TRIG;
    rx_trigger = DMA_I2C1_RX_TRIG;
    return true;
  }
#endif
#if defined(I2C2_BASE) && defined(DMA_I2C2_TX_TRIG) && defined(DMA_I2C2_RX_TRIG)
  if (instance == I2C2)
  {
    tx_trigger = DMA_I2C2_TX_TRIG;
    rx_trigger = DMA_I2C2_RX_TRIG;
    return true;
  }
#endif
#if defined(I2C3_BASE) && defined(DMA_I2C3_TX_TRIG) && defined(DMA_I2C3_RX_TRIG)
  if (instance == I2C3)
  {
    tx_trigger = DMA_I2C3_TX_TRIG;
    rx_trigger = DMA_I2C3_RX_TRIG;
    return true;
  }
#endif
  return false;
}

constexpr bool mspm0_i2c_is_valid_addr7(uint16_t slave_addr)
{
  return slave_addr <= 0x7FU;
}

void mspm0_i2c_recover_controller(I2C_Regs* instance)
{
  DL_I2C_disableInterrupt(instance, 0xFFFFFFFFU);
  DL_I2C_clearInterruptStatus(instance, 0xFFFFFFFFU);
  DL_I2C_disableControllerReadOnTXEmpty(instance);
  DL_I2C_resetControllerTransfer(instance);
  DL_I2C_enableController(instance);
}

}  // namespace

MSPM0I2C::MSPM0I2C(Resources res, RawData stage_buffer, uint32_t dma_enable_min_size,
                   I2C::Configuration config)
    : I2C(),
      res_(res),
      stage_buffer_(stage_buffer),
      dma_enable_min_size_(dma_enable_min_size)
{
  ASSERT(res_.instance != nullptr);
  ASSERT(res_.clock_freq > 0);
  ASSERT(res_.index < MAX_I2C_INSTANCES);
  ASSERT(stage_buffer_.addr_ != nullptr);
  ASSERT(stage_buffer_.size_ > 0);

  if (config.clock_speed == 0)
  {
    config.clock_speed = res_.default_bus_speed_hz;
  }
  const ErrorCode SET_CFG_ANS = SetConfig(config);
  ASSERT(SET_CFG_ANS == ErrorCode::OK);
}

ErrorCode MSPM0I2C::CheckControllerError() const
{
  const uint32_t STATUS = DL_I2C_getControllerStatus(res_.instance);
  if ((STATUS & DL_I2C_CONTROLLER_STATUS_ERROR) != 0 ||
      (STATUS & DL_I2C_CONTROLLER_STATUS_ARBITRATION_LOST) != 0)
  {
    return ErrorCode::FAILED;
  }
  return ErrorCode::OK;
}

ErrorCode MSPM0I2C::WaitControllerIdle() const
{
  uint32_t timeout = MSPM0_I2C_WAIT_IDLE_TIMEOUT;
  while (timeout-- > 0)
  {
    if ((DL_I2C_getControllerStatus(res_.instance) & DL_I2C_CONTROLLER_STATUS_IDLE) != 0)
    {
      return ErrorCode::OK;
    }
    if (CheckControllerError() != ErrorCode::OK)
    {
      return ErrorCode::FAILED;
    }
  }
  return ErrorCode::BUSY;
}

ErrorCode MSPM0I2C::WaitTransactionDone() const
{
  uint32_t timeout = MSPM0_I2C_WAIT_FIFO_TIMEOUT;
  while (timeout-- > 0)
  {
    if (DL_I2C_getTransactionCount(res_.instance) == 0)
    {
      return ErrorCode::OK;
    }
    if (CheckControllerError() != ErrorCode::OK)
    {
      return ErrorCode::FAILED;
    }
  }
  return ErrorCode::BUSY;
}

ErrorCode MSPM0I2C::WaitBusIdle() const
{
  uint32_t timeout = MSPM0_I2C_WAIT_BUS_TIMEOUT;
  while (timeout-- > 0)
  {
    if ((DL_I2C_getControllerStatus(res_.instance) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS) ==
        0)
    {
      return ErrorCode::OK;
    }
    if (CheckControllerError() != ErrorCode::OK)
    {
      return ErrorCode::FAILED;
    }
  }
  return ErrorCode::BUSY;
}

ErrorCode MSPM0I2C::SetConfig(Configuration config)
{
  if (res_.instance == nullptr || res_.clock_freq == 0U || config.clock_speed == 0U)
  {
    return ErrorCode::ARG_ERR;
  }
  if (config.clock_speed > 1000000U)
  {
    return ErrorCode::NOT_SUPPORT;
  }

  // SCL = BUSCLK / (10 * (TPR + 1)); round up to avoid exceeding the request.
  const uint64_t PERIOD_DEN = uint64_t(config.clock_speed) * 10U;
  const uint64_t period_factor =
      (uint64_t(res_.clock_freq) + PERIOD_DEN - 1U) / PERIOD_DEN;
  if (period_factor == 0)
  {
    return ErrorCode::NOT_SUPPORT;
  }
  if (period_factor > 128U)
  {
    return ErrorCode::NOT_SUPPORT;
  }

  if ((DL_I2C_getControllerStatus(res_.instance) &
       (DL_I2C_CONTROLLER_STATUS_BUSY | DL_I2C_CONTROLLER_STATUS_BUSY_BUS)) != 0U)
  {
    return ErrorCode::BUSY;
  }

  const uint8_t TIMER_PERIOD = static_cast<uint8_t>(period_factor - 1U);
  const DL_I2C_ClockConfig CLOCK_CONFIG = {
      .clockSel = DL_I2C_CLOCK_BUSCLK,
      .divideRatio = DL_I2C_CLOCK_DIVIDE_1,
  };

  uint8_t dma_tx_trigger = 0U;
  uint8_t dma_rx_trigger = 0U;
  const bool USE_DMA =
      I2C_DMA_CONFIGURED &&
      mspm0_i2c_resolve_dma_triggers(res_.instance, dma_tx_trigger, dma_rx_trigger);
  dma_enabled_ = USE_DMA;

  DL_I2C_disableController(res_.instance);
  DL_I2C_setClockConfig(res_.instance, &CLOCK_CONFIG);
  DL_I2C_resetControllerTransfer(res_.instance);
  DL_I2C_setTimerPeriod(res_.instance, TIMER_PERIOD);
  DL_I2C_setControllerTXFIFOThreshold(
      res_.instance, USE_DMA ? DL_I2C_TX_FIFO_LEVEL_EMPTY : DL_I2C_TX_FIFO_LEVEL_BYTES_1);
  DL_I2C_setControllerRXFIFOThreshold(res_.instance, DL_I2C_RX_FIFO_LEVEL_BYTES_1);
  DL_I2C_enableControllerClockStretching(res_.instance);
  DL_I2C_disableInterrupt(res_.instance, 0xFFFFFFFFU);
  DL_I2C_clearInterruptStatus(res_.instance, 0xFFFFFFFFU);

  DL_I2C_disableDMAEvent(res_.instance, DL_I2C_EVENT_ROUTE_1,
                         DL_I2C_DMA_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER);
  DL_I2C_disableDMAEvent(res_.instance, DL_I2C_EVENT_ROUTE_2,
                         DL_I2C_DMA_INTERRUPT_CONTROLLER_RXFIFO_TRIGGER);
  if (USE_DMA)
  {
    DL_DMA_disableChannel(DMA, I2C_DMA_TX_CHANNEL);
    DL_DMA_disableChannel(DMA, I2C_DMA_RX_CHANNEL);
    DL_DMA_clearInterruptStatus(DMA, mspm0_i2c_dma_channel_mask(I2C_DMA_TX_CHANNEL) |
                                         mspm0_i2c_dma_channel_mask(I2C_DMA_RX_CHANNEL));
  }

  if (USE_DMA)
  {
    DL_DMA_Config dma_tx_config = MSPM0_I2C_DMA_TX_CONFIG_BASE;
    DL_DMA_Config dma_rx_config = MSPM0_I2C_DMA_RX_CONFIG_BASE;
    dma_tx_config.trigger = dma_tx_trigger;
    dma_rx_config.trigger = dma_rx_trigger;

    DL_I2C_enableDMAEvent(res_.instance, DL_I2C_EVENT_ROUTE_1,
                          DL_I2C_DMA_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER);
    DL_I2C_enableDMAEvent(res_.instance, DL_I2C_EVENT_ROUTE_2,
                          DL_I2C_DMA_INTERRUPT_CONTROLLER_RXFIFO_TRIGGER);
    DL_DMA_initChannel(DMA, I2C_DMA_TX_CHANNEL, &dma_tx_config);
    DL_DMA_initChannel(DMA, I2C_DMA_RX_CHANNEL, &dma_rx_config);
  }

  DL_I2C_enableController(res_.instance);

  return ErrorCode::OK;
}

ErrorCode MSPM0I2C::PollingWrite7(uint16_t addr7, const uint8_t* data, size_t size)
{
  if (size == 0)
  {
    return ErrorCode::OK;
  }
  if (size > MSPM0_I2C_MAX_TRANSFER_SIZE)
  {
    return ErrorCode::ARG_ERR;
  }

  auto attempt_once = [&]() -> ErrorCode
  {
    ErrorCode ans = WaitControllerIdle();
    if (ans != ErrorCode::OK)
    {
      return ans;
    }

    DL_I2C_clearInterruptStatus(res_.instance, 0xFFFFFFFFU);

    size_t sent =
        DL_I2C_fillControllerTXFIFO(res_.instance, data, static_cast<uint16_t>(size));
    DL_I2C_startControllerTransfer(res_.instance, addr7, DL_I2C_CONTROLLER_DIRECTION_TX,
                                   static_cast<uint16_t>(size));

    while (sent < size)
    {
      uint32_t timeout = MSPM0_I2C_WAIT_FIFO_TIMEOUT;
      while (DL_I2C_getRawInterruptStatus(
                 res_.instance, DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER) == 0U)
      {
        if (CheckControllerError() != ErrorCode::OK)
        {
          return ErrorCode::FAILED;
        }
        if (timeout-- == 0)
        {
          return ErrorCode::BUSY;
        }
      }

      sent += DL_I2C_fillControllerTXFIFO(res_.instance, data + sent,
                                          static_cast<uint16_t>(size - sent));
    }

    ans = WaitBusIdle();
    if (ans != ErrorCode::OK)
    {
      return ans;
    }

    return CheckControllerError();
  };

  ErrorCode last_error = ErrorCode::FAILED;
  for (uint32_t attempt = 0; attempt < MSPM0_I2C_POLLING_ATTEMPTS; ++attempt)
  {
    last_error = attempt_once();
    if (last_error == ErrorCode::OK)
    {
      return ErrorCode::OK;
    }
    mspm0_i2c_recover_controller(res_.instance);
  }

  return last_error;
}

ErrorCode MSPM0I2C::PollingRead7(uint16_t addr7, uint8_t* data, size_t size)
{
  if (size == 0)
  {
    return ErrorCode::OK;
  }
  if (size > MSPM0_I2C_MAX_TRANSFER_SIZE)
  {
    return ErrorCode::ARG_ERR;
  }

  auto attempt_once = [&]() -> ErrorCode
  {
    ErrorCode ans = WaitControllerIdle();
    if (ans != ErrorCode::OK)
    {
      return ans;
    }

    DL_I2C_clearInterruptStatus(res_.instance, 0xFFFFFFFFU);
    DL_I2C_startControllerTransfer(res_.instance, addr7, DL_I2C_CONTROLLER_DIRECTION_RX,
                                   static_cast<uint16_t>(size));

    size_t received = 0;
    while (received < size)
    {
      uint32_t timeout = MSPM0_I2C_WAIT_FIFO_TIMEOUT;
      while (DL_I2C_isControllerRXFIFOEmpty(res_.instance))
      {
        if (CheckControllerError() != ErrorCode::OK)
        {
          return ErrorCode::FAILED;
        }
        if (timeout-- == 0)
        {
          return ErrorCode::BUSY;
        }
      }

      while (!DL_I2C_isControllerRXFIFOEmpty(res_.instance) && received < size)
      {
        data[received++] = DL_I2C_receiveControllerData(res_.instance);
      }
    }

    ans = WaitBusIdle();
    if (ans != ErrorCode::OK)
    {
      return ans;
    }

    return CheckControllerError();
  };

  ErrorCode last_error = ErrorCode::FAILED;
  for (uint32_t attempt = 0; attempt < MSPM0_I2C_POLLING_ATTEMPTS; ++attempt)
  {
    last_error = attempt_once();
    if (last_error == ErrorCode::OK)
    {
      return ErrorCode::OK;
    }
    mspm0_i2c_recover_controller(res_.instance);
  }

  return last_error;
}

ErrorCode MSPM0I2C::WaitDmaTransferDone(uint8_t channel_id) const
{
  uint32_t timeout = MSPM0_I2C_WAIT_FIFO_TIMEOUT;
  while (DL_DMA_getTransferSize(DMA, channel_id) != 0U)
  {
    if (CheckControllerError() != ErrorCode::OK)
    {
      return ErrorCode::FAILED;
    }
    if (timeout-- == 0U)
    {
      return ErrorCode::BUSY;
    }
  }
  return ErrorCode::OK;
}

ErrorCode MSPM0I2C::DmaWrite7(uint16_t addr7, ConstRawData write_data)
{
  if (write_data.size_ == 0)
  {
    return ErrorCode::OK;
  }
  if (write_data.size_ > MSPM0_I2C_MAX_TRANSFER_SIZE)
  {
    return ErrorCode::ARG_ERR;
  }

  auto stop_dma = [&]()
  {
    const uint32_t DMA_TX_MASK = mspm0_i2c_dma_channel_mask(I2C_DMA_TX_CHANNEL);
    DL_DMA_disableChannel(DMA, I2C_DMA_TX_CHANNEL);
    DL_DMA_clearInterruptStatus(DMA, DMA_TX_MASK);
  };

  auto attempt_once = [&]() -> ErrorCode
  {
    ErrorCode ans = WaitControllerIdle();
    if (ans != ErrorCode::OK)
    {
      return ans;
    }

    DL_I2C_disableInterrupt(res_.instance, 0xFFFFFFFFU);
    DL_I2C_clearInterruptStatus(res_.instance, 0xFFFFFFFFU);

    const uint32_t DMA_TX_MASK = mspm0_i2c_dma_channel_mask(I2C_DMA_TX_CHANNEL);
    DL_DMA_disableChannel(DMA, I2C_DMA_TX_CHANNEL);
    DL_DMA_clearInterruptStatus(DMA, DMA_TX_MASK);
    DL_DMA_setSrcAddr(
        DMA, I2C_DMA_TX_CHANNEL,
        static_cast<uint32_t>(reinterpret_cast<uintptr_t>(write_data.addr_)));
    DL_DMA_setDestAddr(DMA, I2C_DMA_TX_CHANNEL,
                       static_cast<uint32_t>(
                           reinterpret_cast<uintptr_t>(&res_.instance->MASTER.MTXDATA)));
    DL_DMA_setTransferSize(DMA, I2C_DMA_TX_CHANNEL,
                           static_cast<uint16_t>(write_data.size_));
    DL_DMA_enableChannel(DMA, I2C_DMA_TX_CHANNEL);

    DL_I2C_startControllerTransfer(res_.instance, addr7, DL_I2C_CONTROLLER_DIRECTION_TX,
                                   static_cast<uint16_t>(write_data.size_));

    ans = WaitDmaTransferDone(I2C_DMA_TX_CHANNEL);
    if (ans != ErrorCode::OK)
    {
      stop_dma();
      return ans;
    }

    ans = WaitTransactionDone();
    if (ans != ErrorCode::OK)
    {
      stop_dma();
      return ans;
    }

    ans = WaitBusIdle();
    if (ans != ErrorCode::OK)
    {
      stop_dma();
      return ans;
    }

    stop_dma();
    return CheckControllerError();
  };

  ErrorCode last_error = ErrorCode::FAILED;
  for (uint32_t attempt = 0; attempt < MSPM0_I2C_POLLING_ATTEMPTS; ++attempt)
  {
    last_error = attempt_once();
    if (last_error == ErrorCode::OK)
    {
      return ErrorCode::OK;
    }
    mspm0_i2c_recover_controller(res_.instance);
  }

  return last_error;
}

ErrorCode MSPM0I2C::DmaRead7(uint16_t addr7, RawData read_data)
{
  if (read_data.size_ == 0)
  {
    return ErrorCode::OK;
  }
  if (read_data.size_ > MSPM0_I2C_MAX_TRANSFER_SIZE)
  {
    return ErrorCode::ARG_ERR;
  }

  auto stop_dma = [&]()
  {
    const uint32_t DMA_RX_MASK = mspm0_i2c_dma_channel_mask(I2C_DMA_RX_CHANNEL);
    DL_DMA_disableChannel(DMA, I2C_DMA_RX_CHANNEL);
    DL_DMA_clearInterruptStatus(DMA, DMA_RX_MASK);
  };

  auto attempt_once = [&]() -> ErrorCode
  {
    ErrorCode ans = WaitControllerIdle();
    if (ans != ErrorCode::OK)
    {
      return ans;
    }

    DL_I2C_disableInterrupt(res_.instance, 0xFFFFFFFFU);
    DL_I2C_clearInterruptStatus(res_.instance, 0xFFFFFFFFU);

    const uint32_t DMA_RX_MASK = mspm0_i2c_dma_channel_mask(I2C_DMA_RX_CHANNEL);
    DL_DMA_disableChannel(DMA, I2C_DMA_RX_CHANNEL);
    DL_DMA_clearInterruptStatus(DMA, DMA_RX_MASK);
    DL_DMA_setSrcAddr(DMA, I2C_DMA_RX_CHANNEL,
                      static_cast<uint32_t>(
                          reinterpret_cast<uintptr_t>(&res_.instance->MASTER.MRXDATA)));
    DL_DMA_setDestAddr(
        DMA, I2C_DMA_RX_CHANNEL,
        static_cast<uint32_t>(reinterpret_cast<uintptr_t>(read_data.addr_)));
    DL_DMA_setTransferSize(DMA, I2C_DMA_RX_CHANNEL,
                           static_cast<uint16_t>(read_data.size_));
    DL_DMA_enableChannel(DMA, I2C_DMA_RX_CHANNEL);

    DL_I2C_startControllerTransfer(res_.instance, addr7, DL_I2C_CONTROLLER_DIRECTION_RX,
                                   static_cast<uint16_t>(read_data.size_));

    ans = WaitDmaTransferDone(I2C_DMA_RX_CHANNEL);
    if (ans != ErrorCode::OK)
    {
      stop_dma();
      return ans;
    }

    ans = WaitTransactionDone();
    if (ans != ErrorCode::OK)
    {
      stop_dma();
      return ans;
    }

    ans = WaitBusIdle();
    if (ans != ErrorCode::OK)
    {
      stop_dma();
      return ans;
    }

    stop_dma();
    return CheckControllerError();
  };

  ErrorCode last_error = ErrorCode::FAILED;
  for (uint32_t attempt = 0; attempt < MSPM0_I2C_POLLING_ATTEMPTS; ++attempt)
  {
    last_error = attempt_once();
    if (last_error == ErrorCode::OK)
    {
      return ErrorCode::OK;
    }
    mspm0_i2c_recover_controller(res_.instance);
  }

  return last_error;
}

ErrorCode MSPM0I2C::Read(uint16_t slave_addr, RawData read_data, ReadOperation& op,
                         bool in_isr)
{
  if (read_data.size_ == 0)
  {
    if (op.type != ReadOperation::OperationType::BLOCK)
    {
      op.UpdateStatus(in_isr, ErrorCode::OK);
    }
    return ErrorCode::OK;
  }

  if (!mspm0_i2c_is_valid_addr7(slave_addr))
  {
    if (op.type != ReadOperation::OperationType::BLOCK)
    {
      op.UpdateStatus(in_isr, ErrorCode::ARG_ERR);
    }
    return ErrorCode::ARG_ERR;
  }

  const uint16_t ADDR7 = slave_addr;
  ErrorCode ans = ErrorCode::NOT_SUPPORT;
  if (dma_enabled_ && read_data.size_ > dma_enable_min_size_)
  {
    ans = DmaRead7(ADDR7, read_data);
  }
  else
  {
    ans = PollingRead7(ADDR7, static_cast<uint8_t*>(read_data.addr_), read_data.size_);
  }

  if (op.type != ReadOperation::OperationType::BLOCK)
  {
    op.UpdateStatus(in_isr, ans);
  }
  return ans;
}

ErrorCode MSPM0I2C::Write(uint16_t slave_addr, ConstRawData write_data,
                          WriteOperation& op, bool in_isr)
{
  if (write_data.size_ == 0)
  {
    if (op.type != WriteOperation::OperationType::BLOCK)
    {
      op.UpdateStatus(in_isr, ErrorCode::OK);
    }
    return ErrorCode::OK;
  }

  if (!mspm0_i2c_is_valid_addr7(slave_addr))
  {
    if (op.type != WriteOperation::OperationType::BLOCK)
    {
      op.UpdateStatus(in_isr, ErrorCode::ARG_ERR);
    }
    return ErrorCode::ARG_ERR;
  }

  const uint16_t ADDR7 = slave_addr;
  ErrorCode ans = ErrorCode::NOT_SUPPORT;
  if (dma_enabled_ && write_data.size_ > dma_enable_min_size_)
  {
    ans = DmaWrite7(ADDR7, write_data);
  }
  else
  {
    ans = PollingWrite7(ADDR7, static_cast<const uint8_t*>(write_data.addr_),
                        write_data.size_);
  }

  if (op.type != WriteOperation::OperationType::BLOCK)
  {
    op.UpdateStatus(in_isr, ans);
  }
  return ans;
}

ErrorCode MSPM0I2C::MemWrite(uint16_t slave_addr, uint16_t mem_addr,
                             ConstRawData write_data, WriteOperation& op,
                             MemAddrLength mem_addr_size, bool in_isr)
{
  if (!mspm0_i2c_is_valid_addr7(slave_addr))
  {
    if (op.type != WriteOperation::OperationType::BLOCK)
    {
      op.UpdateStatus(in_isr, ErrorCode::ARG_ERR);
    }
    return ErrorCode::ARG_ERR;
  }
  const size_t ADDR_SIZE = (mem_addr_size == MemAddrLength::BYTE_8) ? 1 : 2;
  const size_t TOTAL_SIZE = ADDR_SIZE + write_data.size_;
  if (TOTAL_SIZE > MSPM0_I2C_MAX_TRANSFER_SIZE)
  {
    return ErrorCode::ARG_ERR;
  }
  if (TOTAL_SIZE > stage_buffer_.size_)
  {
    return ErrorCode::ARG_ERR;
  }

  auto* tx = static_cast<uint8_t*>(stage_buffer_.addr_);
  if (ADDR_SIZE == 1)
  {
    tx[0] = static_cast<uint8_t>(mem_addr & 0xFF);
  }
  else
  {
    tx[0] = static_cast<uint8_t>((mem_addr >> 8) & 0xFF);
    tx[1] = static_cast<uint8_t>(mem_addr & 0xFF);
  }
  if (write_data.size_ > 0)
  {
    memcpy(tx + ADDR_SIZE, write_data.addr_, write_data.size_);
  }

  return Write(slave_addr, ConstRawData(tx, TOTAL_SIZE), op, in_isr);
}

ErrorCode MSPM0I2C::MemRead(uint16_t slave_addr, uint16_t mem_addr, RawData read_data,
                            ReadOperation& op, MemAddrLength mem_addr_size, bool in_isr)
{
  if (read_data.size_ > MSPM0_I2C_MAX_TRANSFER_SIZE)
  {
    return ErrorCode::ARG_ERR;
  }
  if (read_data.size_ == 0)
  {
    if (op.type != ReadOperation::OperationType::BLOCK)
    {
      op.UpdateStatus(in_isr, ErrorCode::OK);
    }
    return ErrorCode::OK;
  }

  if (!mspm0_i2c_is_valid_addr7(slave_addr))
  {
    if (op.type != ReadOperation::OperationType::BLOCK)
    {
      op.UpdateStatus(in_isr, ErrorCode::ARG_ERR);
    }
    return ErrorCode::ARG_ERR;
  }

  const uint16_t ADDR7 = slave_addr;
  const size_t ADDR_SIZE = (mem_addr_size == MemAddrLength::BYTE_8) ? 1 : 2;
  auto* addr_bytes = static_cast<uint8_t*>(stage_buffer_.addr_);
  if (stage_buffer_.size_ < ADDR_SIZE)
  {
    return ErrorCode::ARG_ERR;
  }

  if (ADDR_SIZE == 1)
  {
    addr_bytes[0] = static_cast<uint8_t>(mem_addr & 0xFF);
  }
  else
  {
    addr_bytes[0] = static_cast<uint8_t>((mem_addr >> 8) & 0xFF);
    addr_bytes[1] = static_cast<uint8_t>(mem_addr & 0xFF);
  }

  auto finalize = [&](ErrorCode code)
  {
    if (op.type != ReadOperation::OperationType::BLOCK)
    {
      op.UpdateStatus(in_isr, code);
    }
    return code;
  };

  auto try_repeated_start = [&]() -> ErrorCode
  {
    ErrorCode ans = WaitControllerIdle();
    if (ans != ErrorCode::OK)
    {
      return ans;
    }

    DL_I2C_clearInterruptStatus(res_.instance, 0xFFFFFFFFU);

    const uint16_t ADDR_SENT = DL_I2C_fillControllerTXFIFO(
        res_.instance, addr_bytes, static_cast<uint16_t>(ADDR_SIZE));
    if (ADDR_SENT != ADDR_SIZE)
    {
      return ErrorCode::FAILED;
    }

    DL_I2C_startControllerTransferAdvanced(
        res_.instance, ADDR7, DL_I2C_CONTROLLER_DIRECTION_TX,
        static_cast<uint16_t>(ADDR_SIZE), DL_I2C_CONTROLLER_START_ENABLE,
        DL_I2C_CONTROLLER_STOP_DISABLE, DL_I2C_CONTROLLER_ACK_ENABLE);

    uint32_t tx_done_timeout = MSPM0_I2C_WAIT_FIFO_TIMEOUT;
    while (DL_I2C_getRawInterruptStatus(res_.instance,
                                        DL_I2C_INTERRUPT_CONTROLLER_TX_DONE) == 0U)
    {
      if (CheckControllerError() != ErrorCode::OK)
      {
        return ErrorCode::FAILED;
      }
      if (tx_done_timeout-- == 0U)
      {
        return ErrorCode::BUSY;
      }
    }
    DL_I2C_clearInterruptStatus(res_.instance, DL_I2C_INTERRUPT_CONTROLLER_TX_DONE);

    if (CheckControllerError() != ErrorCode::OK)
    {
      return ErrorCode::FAILED;
    }

    DL_I2C_startControllerTransferAdvanced(
        res_.instance, ADDR7, DL_I2C_CONTROLLER_DIRECTION_RX,
        static_cast<uint16_t>(read_data.size_), DL_I2C_CONTROLLER_START_ENABLE,
        DL_I2C_CONTROLLER_STOP_ENABLE, DL_I2C_CONTROLLER_ACK_DISABLE);

    size_t received = 0;
    auto* read_ptr = static_cast<uint8_t*>(read_data.addr_);
    while (received < read_data.size_)
    {
      uint32_t timeout = MSPM0_I2C_WAIT_FIFO_TIMEOUT;
      while (DL_I2C_isControllerRXFIFOEmpty(res_.instance))
      {
        if (CheckControllerError() != ErrorCode::OK)
        {
          return ErrorCode::FAILED;
        }
        if (timeout-- == 0)
        {
          return ErrorCode::BUSY;
        }
      }

      while (!DL_I2C_isControllerRXFIFOEmpty(res_.instance) && received < read_data.size_)
      {
        read_ptr[received++] = DL_I2C_receiveControllerData(res_.instance);
      }
    }

    uint32_t rx_done_timeout = MSPM0_I2C_WAIT_FIFO_TIMEOUT;
    while (DL_I2C_getRawInterruptStatus(res_.instance,
                                        DL_I2C_INTERRUPT_CONTROLLER_RX_DONE) == 0U)
    {
      if (CheckControllerError() != ErrorCode::OK)
      {
        return ErrorCode::FAILED;
      }
      if (rx_done_timeout-- == 0U)
      {
        return ErrorCode::BUSY;
      }
    }
    DL_I2C_clearInterruptStatus(res_.instance, DL_I2C_INTERRUPT_CONTROLLER_RX_DONE);

    ans = WaitBusIdle();
    if (ans != ErrorCode::OK)
    {
      return ans;
    }

    return CheckControllerError();
  };

  ErrorCode last_error = ErrorCode::FAILED;
  for (uint32_t attempt = 0; attempt < MSPM0_I2C_MEMREAD_RS_ATTEMPTS; ++attempt)
  {
    last_error = try_repeated_start();
    if (last_error == ErrorCode::OK)
    {
      return finalize(ErrorCode::OK);
    }
    mspm0_i2c_recover_controller(res_.instance);
  }

  for (uint32_t attempt = 0; attempt < MSPM0_I2C_MEMREAD_FALLBACK_ATTEMPTS; ++attempt)
  {
    last_error = PollingWrite7(ADDR7, addr_bytes, ADDR_SIZE);
    if (last_error == ErrorCode::OK)
    {
      last_error =
          PollingRead7(ADDR7, static_cast<uint8_t*>(read_data.addr_), read_data.size_);
      if (last_error == ErrorCode::OK)
      {
        return finalize(ErrorCode::OK);
      }
    }
    mspm0_i2c_recover_controller(res_.instance);
  }

  return finalize(last_error);
}
