// NOLINTBEGIN(cppcoreguidelines-pro-type-cstyle-cast,performance-no-int-to-ptr)
// ch32_i2c.cpp
#include "ch32_i2c.hpp"

#include <cstring>

#include "ch32_dma.hpp"
#include "ch32_gpio.hpp"

using namespace LibXR;

CH32I2C* CH32I2C::map_[CH32_I2C_NUMBER] = {nullptr};

namespace
{

constexpr uint32_t CH32_I2C_ERROR_FLAGS =
    ((I2C_FLAG_BERR & 0xFFFFu) | (I2C_FLAG_ARLO & 0xFFFFu) | (I2C_FLAG_AF & 0xFFFFu) |
     (I2C_FLAG_OVR & 0xFFFFu) | (I2C_FLAG_PECERR & 0xFFFFu) |
     (I2C_FLAG_TIMEOUT & 0xFFFFu) | (I2C_FLAG_SMBALERT & 0xFFFFu));

}  // namespace

static inline void ch32_i2c_enable_clocks(ch32_i2c_id_t id)
{
  RCC_APB1PeriphClockCmd(CH32_I2C_RCC_PERIPH_MAP[id], ENABLE);
  RCC_AHBPeriphClockCmd(CH32_I2C_RCC_PERIPH_MAP_DMA[id], ENABLE);
}

CH32I2C::CH32I2C(ch32_i2c_id_t id, RawData dma_buff, GPIO_TypeDef* scl_port,
                 uint16_t scl_pin, GPIO_TypeDef* sda_port, uint16_t sda_pin,
                 uint32_t pin_remap, uint32_t dma_enable_min_size,
                 uint32_t default_clock_hz, bool ten_bit_addr)
    : I2C(),
      instance_(ch32_i2c_get_instance_id(id)),
      dma_rx_channel_(CH32_I2C_RX_DMA_CHANNEL_MAP[id]),
      dma_tx_channel_(CH32_I2C_TX_DMA_CHANNEL_MAP[id]),
      id_(id),
      dma_enable_min_size_(dma_enable_min_size),
      dma_buff_(dma_buff),
      scl_port_(scl_port),
      scl_pin_(scl_pin),
      sda_port_(sda_port),
      sda_pin_(sda_pin),
      ten_bit_addr_(ten_bit_addr)
{
  ASSERT(instance_ != nullptr);
  ASSERT(dma_buff_.addr_ != nullptr && dma_buff_.size_ > 0);

  map_[id_] = this;

  // Clock configuration.
  ch32_i2c_enable_clocks(id_);

  // GPIO configuration (I2C alternate-function open-drain).
  {
    GPIO_InitTypeDef gpio = {};
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_AF_OD;

    RCC_APB2PeriphClockCmd(ch32_get_gpio_periph(scl_port_), ENABLE);
    RCC_APB2PeriphClockCmd(ch32_get_gpio_periph(sda_port_), ENABLE);

    GPIO_SetBits(scl_port_, scl_pin_);
    GPIO_SetBits(sda_port_, sda_pin_);

    gpio.GPIO_Pin = scl_pin_;
    GPIO_Init(scl_port_, &gpio);

    gpio.GPIO_Pin = sda_pin_;
    GPIO_Init(sda_port_, &gpio);

    if (pin_remap != 0)
    {
      RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
      GPIO_PinRemapConfig(pin_remap, ENABLE);
    }
  }

  // DMA configuration.
  {
    // RX
    {
      ch32_dma_callback_t cb = [](void* arg)
      { reinterpret_cast<CH32I2C*>(arg)->RxDmaIRQHandler(); };
      ch32_dma_register_callback(ch32_dma_get_id(dma_rx_channel_), cb, this);

      DMA_InitTypeDef di = {};
      DMA_DeInit(dma_rx_channel_);
      di.DMA_PeripheralBaseAddr = (uint32_t)&instance_->DATAR;
      di.DMA_MemoryBaseAddr = (uint32_t)dma_buff_.addr_;
      di.DMA_DIR = DMA_DIR_PeripheralSRC;
      di.DMA_BufferSize = 0;
      di.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
      di.DMA_MemoryInc = DMA_MemoryInc_Enable;
      di.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
      di.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
      di.DMA_Mode = DMA_Mode_Normal;
      di.DMA_Priority = DMA_Priority_High;
      di.DMA_M2M = DMA_M2M_Disable;
      DMA_Init(dma_rx_channel_, &di);
      DMA_ITConfig(dma_rx_channel_, DMA_IT_TC, ENABLE);
      NVIC_EnableIRQ(CH32_DMA_IRQ_MAP[ch32_dma_get_id(dma_rx_channel_)]);
    }

    // TX
    {
      ch32_dma_callback_t cb = [](void* arg)
      { reinterpret_cast<CH32I2C*>(arg)->TxDmaIRQHandler(); };
      ch32_dma_register_callback(ch32_dma_get_id(dma_tx_channel_), cb, this);

      DMA_InitTypeDef di = {};
      DMA_DeInit(dma_tx_channel_);
      di.DMA_PeripheralBaseAddr = (uint32_t)&instance_->DATAR;
      di.DMA_MemoryBaseAddr = (uint32_t)dma_buff_.addr_;
      di.DMA_DIR = DMA_DIR_PeripheralDST;
      di.DMA_BufferSize = 0;
      di.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
      di.DMA_MemoryInc = DMA_MemoryInc_Enable;
      di.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
      di.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
      di.DMA_Mode = DMA_Mode_Normal;
      di.DMA_Priority = DMA_Priority_VeryHigh;
      di.DMA_M2M = DMA_M2M_Disable;
      DMA_Init(dma_tx_channel_, &di);
      DMA_ITConfig(dma_tx_channel_, DMA_IT_TC, ENABLE);
      NVIC_EnableIRQ(CH32_DMA_IRQ_MAP[ch32_dma_get_id(dma_tx_channel_)]);
    }
  }

  // I2C error interrupt for asynchronous transfer abort handling.
  I2C_ITConfig(instance_, I2C_IT_ERR, ENABLE);
  NVIC_EnableIRQ(CH32_I2C_ER_IRQ_MAP[id_]);

  // Default runtime parameters.
  cfg_.clock_speed = default_clock_hz;
  (void)SetConfig(cfg_);
}

ErrorCode CH32I2C::SetConfig(Configuration config)
{
  if (instance_ == nullptr || config.clock_speed == 0U)
  {
    return ErrorCode::ARG_ERR;
  }
  if (config.clock_speed > 400000U)
  {
    return ErrorCode::NOT_SUPPORT;
  }
  if (recovering_ || DmaBusy() || I2C_GetFlagStatus(instance_, I2C_FLAG_BUSY) != RESET)
  {
    return ErrorCode::BUSY;
  }

  RCC_ClocksTypeDef clocks{};
  RCC_GetClocksFreq(&clocks);
  const uint64_t denominator =
      uint64_t(config.clock_speed) * (config.clock_speed <= 100000U ? 2U : 3U);
  // CKCFGR.CCR is 12 bits; the SDK otherwise silently truncates low-rate dividers.
  if (clocks.PCLK1_Frequency == 0U ||
      (uint64_t(clocks.PCLK1_Frequency) + denominator - 1U) / denominator > 0x0FFFU)
  {
    return ErrorCode::NOT_SUPPORT;
  }

  ApplyConfig(config);
  cfg_ = config;
  return ErrorCode::OK;
}

void CH32I2C::ApplyConfig(Configuration config)
{
  I2C_Cmd(instance_, DISABLE);
  I2C_DeInit(instance_);

  I2C_InitTypeDef init = {};
  init.I2C_ClockSpeed = config.clock_speed;
  init.I2C_Mode = I2C_Mode_I2C;
  init.I2C_DutyCycle = I2C_DutyCycle_2;
  init.I2C_OwnAddress1 = 0;
  init.I2C_Ack = I2C_Ack_Enable;
  init.I2C_AcknowledgedAddress =
      ten_bit_addr_ ? I2C_AcknowledgedAddress_10bit : I2C_AcknowledgedAddress_7bit;

  I2C_Init(instance_, &init);
  I2C_Cmd(instance_, ENABLE);

  // 默认 ACK/NACK 状态
  I2C_AcknowledgeConfig(instance_, ENABLE);
  I2C_NACKPositionConfig(instance_, I2C_NACKPosition_Current);
  // I2C_DeInit resets CTLR2, including the asynchronous error interrupt.
  I2C_ITConfig(instance_, I2C_IT_ERR, ENABLE);
}

bool CH32I2C::WaitEvent(uint32_t evt, uint32_t timeout_us)
{
  const uint64_t START = static_cast<uint64_t>(Timebase::GetMicroseconds());
  while ((static_cast<uint64_t>(Timebase::GetMicroseconds()) - START) < timeout_us)
  {
    if ((instance_->STAR1 & CH32_I2C_ERROR_FLAGS) != 0u)
    {
      return false;
    }
    if (I2C_CheckEvent(instance_, evt) == READY)
    {
      return true;
    }
  }
  return false;
}

bool CH32I2C::WaitFlag(uint32_t flag, FlagStatus st, uint32_t timeout_us)
{
  const uint64_t START = static_cast<uint64_t>(Timebase::GetMicroseconds());
  while ((static_cast<uint64_t>(Timebase::GetMicroseconds()) - START) < timeout_us)
  {
    if ((flag != I2C_FLAG_BUSY) && ((instance_->STAR1 & CH32_I2C_ERROR_FLAGS) != 0u))
    {
      return false;
    }
    if (I2C_GetFlagStatus(instance_, flag) == st)
    {
      return true;
    }
  }
  return false;
}

ErrorCode CH32I2C::WaitEventOrRecover(uint32_t evt, uint32_t timeout_us)
{
  if (WaitEvent(evt, timeout_us))
  {
    return ErrorCode::OK;
  }
  RecoverAfterImmediateFailure();
  return ErrorCode::BUSY;
}

ErrorCode CH32I2C::WaitFlagOrRecover(uint32_t flag, FlagStatus st, uint32_t timeout_us)
{
  if (WaitFlag(flag, st, timeout_us))
  {
    return ErrorCode::OK;
  }
  RecoverAfterImmediateFailure();
  return ErrorCode::BUSY;
}

void CH32I2C::ClearAddrFlag()
{
  volatile uint16_t tmp1 = instance_->STAR1;
  volatile uint16_t tmp2 = instance_->STAR2;
  (void)tmp1;
  (void)tmp2;
}

ErrorCode CH32I2C::MasterStartAndAddress10Bit(uint16_t addr10, uint8_t final_dir)
{
  addr10 = Addr10Clamp(addr10);

  // 等待 BUSY 释放
  if (auto ec = WaitFlagOrRecover(I2C_FLAG_BUSY, RESET); ec != ErrorCode::OK)
  {
    return ec;
  }

  // --- 1) START ---
  I2C_GenerateSTART(instance_, ENABLE);
  if (auto ec = WaitEventOrRecover(I2C_EVENT_MASTER_MODE_SELECT); ec != ErrorCode::OK)
  {
    return ec;
  }

  // --- 2) 发送 10-bit 头字节（地址阶段固定以写方向发送，R/W=0）---
  // header (8-bit,left-shifted): 11110 A9 A8 0 => 0xF0/0xF2/0xF4/0xF6
  const uint8_t HEADER = static_cast<uint8_t>(0xF0 | ((addr10 >> 7) & 0x06));
  I2C_Send7bitAddress(instance_, HEADER, I2C_Direction_Transmitter);

  // 等待 EVT9（ADD10）
  if (auto ec = WaitEventOrRecover(I2C_EVENT_MASTER_MODE_ADDRESS10); ec != ErrorCode::OK)
  {
    return ec;
  }

  // --- 3) 发送地址低 8 位 ---
  I2C_SendData(instance_, static_cast<uint8_t>(addr10 & 0xFF));

  // 等待 EVT6（作为 Transmitter 完成地址阶段）
  if (auto ec = WaitEventOrRecover(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);
      ec != ErrorCode::OK)
  {
    return ec;
  }
  ClearAddrFlag();

  if (final_dir == I2C_Direction_Transmitter)
  {
    return ErrorCode::OK;
  }

  // --- 4) 若最终为读：Repeated START + 头字节（R/W=1） ---
  I2C_GenerateSTART(instance_, ENABLE);
  if (auto ec = WaitEventOrRecover(I2C_EVENT_MASTER_MODE_SELECT); ec != ErrorCode::OK)
  {
    return ec;
  }

  I2C_Send7bitAddress(instance_, HEADER, I2C_Direction_Receiver);
  if (auto ec = WaitEventOrRecover(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);
      ec != ErrorCode::OK)
  {
    return ec;
  }
  ClearAddrFlag();

  return ErrorCode::OK;
}

ErrorCode CH32I2C::MasterStartAndAddress(uint16_t slave_addr, uint8_t dir)
{
  if (!ten_bit_addr_)
  {
    // 7-bit：输入为原始 7-bit 地址
    const uint8_t ADDR8 = Addr7ToAddr8(slave_addr);

    if (auto ec = WaitFlagOrRecover(I2C_FLAG_BUSY, RESET); ec != ErrorCode::OK)
    {
      return ec;
    }

    I2C_GenerateSTART(instance_, ENABLE);
    if (auto ec = WaitEventOrRecover(I2C_EVENT_MASTER_MODE_SELECT); ec != ErrorCode::OK)
    {
      return ec;
    }

    I2C_Send7bitAddress(instance_, ADDR8, dir);

    if (dir == I2C_Direction_Transmitter)
    {
      if (auto ec = WaitEventOrRecover(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);
          ec != ErrorCode::OK)
      {
        return ec;
      }
    }
    else
    {
      if (auto ec = WaitEventOrRecover(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);
          ec != ErrorCode::OK)
      {
        return ec;
      }
    }

    ClearAddrFlag();
    return ErrorCode::OK;
  }

  // 10-bit：输入为原始 10-bit 地址
  return MasterStartAndAddress10Bit(slave_addr, dir);
}

ErrorCode CH32I2C::SendMemAddr(uint16_t mem_addr, MemAddrLength len)
{
  if (len == MemAddrLength::BYTE_16)
  {
    if (auto ec = WaitFlagOrRecover(I2C_FLAG_TXE, SET); ec != ErrorCode::OK)
    {
      return ec;
    }
    I2C_SendData(instance_, static_cast<uint8_t>((mem_addr >> 8) & 0xFF));
    if (auto ec = WaitFlagOrRecover(I2C_FLAG_TXE, SET); ec != ErrorCode::OK)
    {
      return ec;
    }
    I2C_SendData(instance_, static_cast<uint8_t>(mem_addr & 0xFF));
  }
  else
  {
    if (auto ec = WaitFlagOrRecover(I2C_FLAG_TXE, SET); ec != ErrorCode::OK)
    {
      return ec;
    }
    I2C_SendData(instance_, static_cast<uint8_t>(mem_addr & 0xFF));
  }

  if (auto ec = WaitFlagOrRecover(I2C_FLAG_BTF, SET); ec != ErrorCode::OK)
  {
    return ec;
  }
  return ErrorCode::OK;
}

ErrorCode CH32I2C::PollingWriteBytes(const uint8_t* data, uint32_t len)
{
  for (uint32_t i = 0; i < len; ++i)
  {
    if (auto ec = WaitFlagOrRecover(I2C_FLAG_TXE, SET); ec != ErrorCode::OK)
    {
      return ec;
    }
    I2C_SendData(instance_, data[i]);
  }
  if (auto ec = WaitFlagOrRecover(I2C_FLAG_BTF, SET); ec != ErrorCode::OK)
  {
    return ec;
  }
  return ErrorCode::OK;
}

ErrorCode CH32I2C::PollingReadBytes(uint8_t* data, uint32_t len)
{
  if (len == 0)
  {
    return ErrorCode::OK;
  }

  // 长度=1
  if (len == 1)
  {
    I2C_AcknowledgeConfig(instance_, DISABLE);
    I2C_GenerateSTOP(instance_, ENABLE);
    if (auto ec = WaitFlagOrRecover(I2C_FLAG_RXNE, SET); ec != ErrorCode::OK)
    {
      return ec;
    }
    data[0] = I2C_ReceiveData(instance_);
    I2C_AcknowledgeConfig(instance_, ENABLE);
    return ErrorCode::OK;
  }

  // 长度=2
  if (len == 2)
  {
    I2C_NACKPositionConfig(instance_, I2C_NACKPosition_Next);
    I2C_AcknowledgeConfig(instance_, DISABLE);

    if (auto ec = WaitFlagOrRecover(I2C_FLAG_BTF, SET); ec != ErrorCode::OK)
    {
      return ec;
    }
    I2C_GenerateSTOP(instance_, ENABLE);

    data[0] = I2C_ReceiveData(instance_);
    data[1] = I2C_ReceiveData(instance_);

    I2C_NACKPositionConfig(instance_, I2C_NACKPosition_Current);
    I2C_AcknowledgeConfig(instance_, ENABLE);
    return ErrorCode::OK;
  }

  // 长度>=3
  I2C_AcknowledgeConfig(instance_, ENABLE);
  I2C_NACKPositionConfig(instance_, I2C_NACKPosition_Current);

  uint32_t idx = 0;
  while (len > 3)
  {
    if (auto ec = WaitFlagOrRecover(I2C_FLAG_RXNE, SET); ec != ErrorCode::OK)
    {
      return ec;
    }
    data[idx++] = I2C_ReceiveData(instance_);
    --len;
  }

  // 剩余 3 字节处理
  if (auto ec = WaitFlagOrRecover(I2C_FLAG_BTF, SET); ec != ErrorCode::OK)
  {
    return ec;
  }
  I2C_AcknowledgeConfig(instance_, DISABLE);
  data[idx++] = I2C_ReceiveData(instance_);
  I2C_GenerateSTOP(instance_, ENABLE);
  data[idx++] = I2C_ReceiveData(instance_);

  if (auto ec = WaitFlagOrRecover(I2C_FLAG_RXNE, SET); ec != ErrorCode::OK)
  {
    return ec;
  }
  data[idx++] = I2C_ReceiveData(instance_);

  I2C_AcknowledgeConfig(instance_, ENABLE);
  return ErrorCode::OK;
}

void CH32I2C::StartTxDma(uint32_t len)
{
  dma_tx_channel_->MADDR = reinterpret_cast<uint32_t>(dma_buff_.addr_);
  dma_tx_channel_->CNTR = len;

  I2C_DMACmd(instance_, ENABLE);
  DMA_Cmd(dma_tx_channel_, ENABLE);
}

void CH32I2C::StartRxDma(uint32_t len)
{
  // DMA 接收需保持 ACK 使能
  I2C_AcknowledgeConfig(instance_, ENABLE);
  I2C_NACKPositionConfig(instance_, I2C_NACKPosition_Current);

  dma_rx_channel_->MADDR = reinterpret_cast<uint32_t>(dma_buff_.addr_);
  dma_rx_channel_->CNTR = len;

  I2C_DMALastTransferCmd(instance_, ENABLE);
  I2C_DMACmd(instance_, ENABLE);
  DMA_Cmd(dma_rx_channel_, ENABLE);
}

void CH32I2C::AbortTransfer(ErrorCode ec)
{
  I2C_DMACmd(instance_, DISABLE);
  I2C_DMALastTransferCmd(instance_, DISABLE);
  DMA_Cmd(dma_tx_channel_, DISABLE);
  DMA_Cmd(dma_rx_channel_, DISABLE);

  // 恢复默认 ACK/NACK 配置
  I2C_AcknowledgeConfig(instance_, ENABLE);
  I2C_NACKPositionConfig(instance_, I2C_NACKPosition_Current);

  I2C_GenerateSTOP(instance_, ENABLE);

  busy_ = false;

  if (read_)
  {
    if (read_op_.type == ReadOperation::OperationType::BLOCK)
    {
      (void)block_wait_.TryPost(true, ec);
    }
    else
    {
      read_op_.UpdateStatus(true, ec);
    }
  }
  else
  {
    if (write_op_.type == WriteOperation::OperationType::BLOCK)
    {
      (void)block_wait_.TryPost(true, ec);
    }
    else
    {
      write_op_.UpdateStatus(true, ec);
    }
  }
}

void CH32I2C::RecoverAfterImmediateFailure()
{
  recovering_ = true;
  I2C_ITConfig(instance_, I2C_IT_ERR, DISABLE);
  I2C_DMACmd(instance_, DISABLE);
  I2C_DMALastTransferCmd(instance_, DISABLE);
  DMA_Cmd(dma_tx_channel_, DISABLE);
  DMA_Cmd(dma_rx_channel_, DISABLE);
  dma_tx_channel_->CNTR = 0;
  dma_rx_channel_->CNTR = 0;

  DMA_ClearITPendingBit(CH32_I2C_TX_DMA_IT_MAP[id_]);
  DMA_ClearITPendingBit(CH32_I2C_RX_DMA_IT_MAP[id_]);

  const uint32_t err_its[] = {I2C_IT_BERR,   I2C_IT_ARLO,    I2C_IT_AF,      I2C_IT_OVR,
                              I2C_IT_PECERR, I2C_IT_TIMEOUT, I2C_IT_SMBALERT};
  for (uint32_t it : err_its)
  {
    I2C_ClearITPendingBit(instance_, it);
  }

  I2C_GenerateSTOP(instance_, ENABLE);
  if (!WaitFlag(I2C_FLAG_BUSY, RESET, K_DEFAULT_TIMEOUT_US))
  {
    // DMA is stopped; recovery must not use the public BUSY admission check.
    ApplyConfig(cfg_);
  }
  else
  {
    I2C_AcknowledgeConfig(instance_, ENABLE);
    I2C_NACKPositionConfig(instance_, I2C_NACKPosition_Current);
  }

  busy_ = false;
  read_ = false;
  read_op_ = {};
  write_op_ = {};
  read_buff_ = {nullptr, 0};
  I2C_ITConfig(instance_, I2C_IT_ERR, ENABLE);
  recovering_ = false;
}

ErrorCode CH32I2C::Write(uint16_t slave_addr, ConstRawData write_data, WriteOperation& op,
                         bool in_isr)
{
  if (write_data.size_ == 0)
  {
    if (op.type != WriteOperation::OperationType::BLOCK)
    {
      op.UpdateStatus(in_isr, ErrorCode::OK);
    }
    return ErrorCode::OK;
  }

  ASSERT(write_data.size_ <= dma_buff_.size_);
  if (DmaBusy())
  {
    return ErrorCode::BUSY;
  }

  read_ = false;
  I2C_ITConfig(instance_, I2C_IT_ERR, DISABLE);

  ErrorCode ec = MasterStartAndAddress(slave_addr, I2C_Direction_Transmitter);
  if (ec != ErrorCode::OK)
  {
    return ec;
  }

  // 短传输：轮询
  if (write_data.size_ <= dma_enable_min_size_)
  {
    ec = PollingWriteBytes(reinterpret_cast<const uint8_t*>(write_data.addr_),
                           write_data.size_);
    I2C_GenerateSTOP(instance_, ENABLE);
    I2C_ITConfig(instance_, I2C_IT_ERR, ENABLE);

    if (op.type != WriteOperation::OperationType::BLOCK)
    {
      op.UpdateStatus(in_isr, ec);
    }
    return ec;
  }

  // 长传输：DMA
  Memory::FastCopy(dma_buff_.addr_, write_data.addr_, write_data.size_);

  write_op_ = op;
  busy_ = true;

  if (op.type == WriteOperation::OperationType::BLOCK)
  {
    block_wait_.Start(*op.data.sem_info.sem);
  }
  op.MarkAsRunning();
  I2C_ITConfig(instance_, I2C_IT_ERR, ENABLE);
  StartTxDma(write_data.size_);

  if (op.type == WriteOperation::OperationType::BLOCK)
  {
    return block_wait_.Wait(op.data.sem_info.timeout);
  }
  return ErrorCode::OK;
}

ErrorCode CH32I2C::Read(uint16_t slave_addr, RawData read_data, ReadOperation& op,
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

  ASSERT(read_data.size_ <= dma_buff_.size_);
  if (DmaBusy())
  {
    return ErrorCode::BUSY;
  }

  read_ = true;
  I2C_ITConfig(instance_, I2C_IT_ERR, DISABLE);

  ErrorCode ec = MasterStartAndAddress(slave_addr, I2C_Direction_Receiver);
  if (ec != ErrorCode::OK)
  {
    return ec;
  }

  // 短传输：轮询
  if (read_data.size_ <= dma_enable_min_size_)
  {
    ec = PollingReadBytes(reinterpret_cast<uint8_t*>(read_data.addr_), read_data.size_);
    I2C_ITConfig(instance_, I2C_IT_ERR, ENABLE);
    if (op.type != ReadOperation::OperationType::BLOCK)
    {
      op.UpdateStatus(in_isr, ec);
    }
    return ec;
  }

  // 长传输：DMA
  read_op_ = op;
  read_buff_ = read_data;
  busy_ = true;

  if (op.type == ReadOperation::OperationType::BLOCK)
  {
    block_wait_.Start(*op.data.sem_info.sem);
  }
  op.MarkAsRunning();
  I2C_ITConfig(instance_, I2C_IT_ERR, ENABLE);
  StartRxDma(read_data.size_);

  if (op.type == ReadOperation::OperationType::BLOCK)
  {
    return block_wait_.Wait(op.data.sem_info.timeout);
  }
  return ErrorCode::OK;
}

ErrorCode CH32I2C::MemWrite(uint16_t slave_addr, uint16_t mem_addr,
                            ConstRawData write_data, WriteOperation& op,
                            MemAddrLength mem_addr_size, bool in_isr)
{
  if (write_data.size_ == 0)
  {
    if (op.type != WriteOperation::OperationType::BLOCK)
    {
      op.UpdateStatus(in_isr, ErrorCode::OK);
    }
    return ErrorCode::OK;
  }

  ASSERT(write_data.size_ <= dma_buff_.size_);
  if (DmaBusy())
  {
    return ErrorCode::BUSY;
  }

  read_ = false;
  I2C_ITConfig(instance_, I2C_IT_ERR, DISABLE);

  ErrorCode ec = MasterStartAndAddress(slave_addr, I2C_Direction_Transmitter);
  if (ec != ErrorCode::OK)
  {
    return ec;
  }

  ec = SendMemAddr(mem_addr, mem_addr_size);
  if (ec != ErrorCode::OK)
  {
    I2C_GenerateSTOP(instance_, ENABLE);
    if (op.type != WriteOperation::OperationType::BLOCK)
    {
      op.UpdateStatus(in_isr, ec);
    }
    return ec;
  }

  if (write_data.size_ <= dma_enable_min_size_)
  {
    ec = PollingWriteBytes(reinterpret_cast<const uint8_t*>(write_data.addr_),
                           write_data.size_);
    I2C_GenerateSTOP(instance_, ENABLE);
    I2C_ITConfig(instance_, I2C_IT_ERR, ENABLE);

    if (op.type != WriteOperation::OperationType::BLOCK)
    {
      op.UpdateStatus(in_isr, ec);
    }
    return ec;
  }

  Memory::FastCopy(dma_buff_.addr_, write_data.addr_, write_data.size_);

  write_op_ = op;
  busy_ = true;

  if (op.type == WriteOperation::OperationType::BLOCK)
  {
    block_wait_.Start(*op.data.sem_info.sem);
  }
  op.MarkAsRunning();
  I2C_ITConfig(instance_, I2C_IT_ERR, ENABLE);
  StartTxDma(write_data.size_);

  if (op.type == WriteOperation::OperationType::BLOCK)
  {
    return block_wait_.Wait(op.data.sem_info.timeout);
  }
  return ErrorCode::OK;
}

ErrorCode CH32I2C::MemRead(uint16_t slave_addr, uint16_t mem_addr, RawData read_data,
                           ReadOperation& op, MemAddrLength mem_addr_size, bool in_isr)
{
  if (read_data.size_ == 0)
  {
    if (op.type != ReadOperation::OperationType::BLOCK)
    {
      op.UpdateStatus(in_isr, ErrorCode::OK);
    }
    return ErrorCode::OK;
  }

  ASSERT(read_data.size_ <= dma_buff_.size_);
  if (DmaBusy())
  {
    return ErrorCode::BUSY;
  }

  read_ = true;
  I2C_ITConfig(instance_, I2C_IT_ERR, DISABLE);

  // 1) 地址阶段：发送写地址 + mem addr
  ErrorCode ec = MasterStartAndAddress(slave_addr, I2C_Direction_Transmitter);
  if (ec != ErrorCode::OK)
  {
    return ec;
  }

  ec = SendMemAddr(mem_addr, mem_addr_size);
  if (ec != ErrorCode::OK)
  {
    I2C_GenerateSTOP(instance_, ENABLE);
    if (op.type != ReadOperation::OperationType::BLOCK)
    {
      op.UpdateStatus(in_isr, ec);
    }
    return ec;
  }

  // 2) Repeated START + 读地址阶段
  if (!ten_bit_addr_)
  {
    const uint8_t ADDR8 = Addr7ToAddr8(slave_addr);

    I2C_GenerateSTART(instance_, ENABLE);
    if (!WaitEvent(I2C_EVENT_MASTER_MODE_SELECT))
    {
      I2C_GenerateSTOP(instance_, ENABLE);
      ec = ErrorCode::BUSY;
      if (op.type != ReadOperation::OperationType::BLOCK)
      {
        op.UpdateStatus(in_isr, ec);
      }
      return ec;
    }

    I2C_Send7bitAddress(instance_, ADDR8, I2C_Direction_Receiver);
    if (!WaitEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
    {
      I2C_GenerateSTOP(instance_, ENABLE);
      ec = ErrorCode::BUSY;
      if (op.type != ReadOperation::OperationType::BLOCK)
      {
        op.UpdateStatus(in_isr, ec);
      }
      return ec;
    }
    ClearAddrFlag();
  }
  else
  {
    const uint16_t ADDR10 = Addr10Clamp(slave_addr);
    const uint8_t HEADER = static_cast<uint8_t>(0xF0 | ((ADDR10 >> 7) & 0x06));

    I2C_GenerateSTART(instance_, ENABLE);
    if (!WaitEvent(I2C_EVENT_MASTER_MODE_SELECT))
    {
      I2C_GenerateSTOP(instance_, ENABLE);
      ec = ErrorCode::BUSY;
      if (op.type != ReadOperation::OperationType::BLOCK)
      {
        op.UpdateStatus(in_isr, ec);
      }
      return ec;
    }

    I2C_Send7bitAddress(instance_, HEADER, I2C_Direction_Receiver);
    if (!WaitEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
    {
      I2C_GenerateSTOP(instance_, ENABLE);
      ec = ErrorCode::BUSY;
      if (op.type != ReadOperation::OperationType::BLOCK)
      {
        op.UpdateStatus(in_isr, ec);
      }
      return ec;
    }
    ClearAddrFlag();
  }

  // 短传输：轮询
  if (read_data.size_ <= dma_enable_min_size_)
  {
    ec = PollingReadBytes(reinterpret_cast<uint8_t*>(read_data.addr_), read_data.size_);
    I2C_ITConfig(instance_, I2C_IT_ERR, ENABLE);
    if (op.type != ReadOperation::OperationType::BLOCK)
    {
      op.UpdateStatus(in_isr, ec);
    }
    return ec;
  }

  // 长传输：DMA
  read_op_ = op;
  read_buff_ = read_data;
  busy_ = true;

  if (op.type == ReadOperation::OperationType::BLOCK)
  {
    block_wait_.Start(*op.data.sem_info.sem);
  }
  op.MarkAsRunning();
  I2C_ITConfig(instance_, I2C_IT_ERR, ENABLE);
  StartRxDma(read_data.size_);

  if (op.type == ReadOperation::OperationType::BLOCK)
  {
    return block_wait_.Wait(op.data.sem_info.timeout);
  }
  return ErrorCode::OK;
}

void CH32I2C::TxDmaIRQHandler()
{
  if (DMA_GetITStatus(CH32_I2C_TX_DMA_IT_MAP[id_]) == RESET)
  {
    return;
  }
  DMA_ClearITPendingBit(CH32_I2C_TX_DMA_IT_MAP[id_]);

  DMA_Cmd(dma_tx_channel_, DISABLE);
  I2C_DMACmd(instance_, DISABLE);

  if (recovering_ || !busy_)
  {
    return;
  }

  (void)WaitFlag(I2C_FLAG_BTF, SET, 20000);
  I2C_GenerateSTOP(instance_, ENABLE);

  busy_ = false;
  if (write_op_.type == WriteOperation::OperationType::BLOCK)
  {
    (void)block_wait_.TryPost(true, ErrorCode::OK);
  }
  else
  {
    write_op_.UpdateStatus(true, ErrorCode::OK);
  }
}

void CH32I2C::RxDmaIRQHandler()
{
  if (DMA_GetITStatus(CH32_I2C_RX_DMA_IT_MAP[id_]) == RESET)
  {
    return;
  }
  DMA_ClearITPendingBit(CH32_I2C_RX_DMA_IT_MAP[id_]);

  DMA_Cmd(dma_rx_channel_, DISABLE);
  I2C_DMACmd(instance_, DISABLE);
  I2C_DMALastTransferCmd(instance_, DISABLE);

  if (recovering_ || !busy_)
  {
    return;
  }

  I2C_GenerateSTOP(instance_, ENABLE);

  if (read_buff_.size_ > 0)
  {
    Memory::FastCopy(read_buff_.addr_, dma_buff_.addr_, read_buff_.size_);
    read_buff_.size_ = 0;
  }

  // 恢复默认 ACK/NACK 配置
  I2C_AcknowledgeConfig(instance_, ENABLE);
  I2C_NACKPositionConfig(instance_, I2C_NACKPosition_Current);

  busy_ = false;
  if (read_op_.type == ReadOperation::OperationType::BLOCK)
  {
    (void)block_wait_.TryPost(true, ErrorCode::OK);
  }
  else
  {
    read_op_.UpdateStatus(true, ErrorCode::OK);
  }
}

void CH32I2C::ErrorIRQHandler()
{
  bool has_err = false;

  const uint32_t ITS[] = {I2C_IT_BERR,    I2C_IT_ARLO,   I2C_IT_AF,      I2C_IT_OVR,
                          I2C_IT_TIMEOUT, I2C_IT_PECERR, I2C_IT_SMBALERT};

  for (uint32_t it : ITS)
  {
    if (I2C_GetITStatus(instance_, it) == SET)
    {
      I2C_ClearITPendingBit(instance_, it);
      has_err = true;
    }
  }

  if (has_err && busy_ && !recovering_)
  {
    AbortTransfer(ErrorCode::FAILED);
  }
}

// I2C ER IRQ entry.
extern "C"
{
#if defined(I2C1)
  void i2_c1_er_irq_handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
  void i2_c1_er_irq_handler(void)
  {
    auto* p = LibXR::CH32I2C::map_[ch32_i2c_get_id(I2C1)];
    if (p)
    {
      p->ErrorIRQHandler();
    }
  }
#endif

#if defined(I2C2)
  void i2_c2_er_irq_handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
  void i2_c2_er_irq_handler(void)
  {
    auto* p = LibXR::CH32I2C::map_[ch32_i2c_get_id(I2C2)];
    if (p)
    {
      p->ErrorIRQHandler();
    }
  }
#endif
}  // extern "C"

// NOLINTEND(cppcoreguidelines-pro-type-cstyle-cast,performance-no-int-to-ptr)
