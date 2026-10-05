// NOLINTBEGIN(cppcoreguidelines-pro-type-cstyle-cast,performance-no-int-to-ptr)
#include "ch32_spi.hpp"

#include <cstring>

#include "ch32_dma.hpp"
#include "ch32_gpio.hpp"

using namespace LibXR;

CH32SPI* CH32SPI::map_[CH32_SPI_NUMBER] = {nullptr};

bool CH32SPI::MapEnumToCH32Prescaler(SPI::Prescaler p, uint16_t& out)
{
  switch (p)
  {
#if defined(SPI_BaudRatePrescaler_2)
    case SPI::Prescaler::DIV_2:
      out = SPI_BaudRatePrescaler_2;
      return true;
#endif
#if defined(SPI_BaudRatePrescaler_4)
    case SPI::Prescaler::DIV_4:
      out = SPI_BaudRatePrescaler_4;
      return true;
#endif
#if defined(SPI_BaudRatePrescaler_8)
    case SPI::Prescaler::DIV_8:
      out = SPI_BaudRatePrescaler_8;
      return true;
#endif
#if defined(SPI_BaudRatePrescaler_16)
    case SPI::Prescaler::DIV_16:
      out = SPI_BaudRatePrescaler_16;
      return true;
#endif
#if defined(SPI_BaudRatePrescaler_32)
    case SPI::Prescaler::DIV_32:
      out = SPI_BaudRatePrescaler_32;
      return true;
#endif
#if defined(SPI_BaudRatePrescaler_64)
    case SPI::Prescaler::DIV_64:
      out = SPI_BaudRatePrescaler_64;
      return true;
#endif
#if defined(SPI_BaudRatePrescaler_128)
    case SPI::Prescaler::DIV_128:
      out = SPI_BaudRatePrescaler_128;
      return true;
#endif
#if defined(SPI_BaudRatePrescaler_256)
    case SPI::Prescaler::DIV_256:
      out = SPI_BaudRatePrescaler_256;
      return true;
#endif
    default:
      return false;  // 不支持 DIV_1 或 >256 的枚举
  }
}

CH32SPI::CH32SPI(ch32_spi_id_t id, RawData dma_rx, RawData dma_tx, GPIO_TypeDef* sck_port,
                 uint16_t sck_pin, GPIO_TypeDef* miso_port, uint16_t miso_pin,
                 GPIO_TypeDef* mosi_port, uint16_t mosi_pin, uint32_t pin_remap,
                 bool master_mode, bool firstbit_msb, uint16_t prescaler,
                 uint32_t dma_enable_min_size, SPI::Configuration config)
    : SPI(dma_rx, dma_tx),
      instance_(ch32_spi_get_instance_id(id)),
      dma_rx_channel_(CH32_SPI_RX_DMA_CHANNEL_MAP[id]),
      dma_tx_channel_(CH32_SPI_TX_DMA_CHANNEL_MAP[id]),
      id_(id),
      dma_enable_min_size_(dma_enable_min_size),
      mode_(master_mode ? SPI_Mode_Master : SPI_Mode_Slave),
      datasize_(SPI_DataSize_8b),
      firstbit_(firstbit_msb ? SPI_FirstBit_MSB : SPI_FirstBit_LSB),
      prescaler_(prescaler),
      sck_port_(sck_port),
      sck_pin_(sck_pin),
      miso_port_(miso_port),
      miso_pin_(miso_pin),
      mosi_port_(mosi_port),
      mosi_pin_(mosi_pin)
{
  ASSERT(instance_ != nullptr);

  map_[id] = this;

  // 时钟配置。
  // Clock configuration.
  if (CH32_SPI_APB_MAP[id] == 1)
  {
    RCC_APB1PeriphClockCmd(CH32_SPI_RCC_PERIPH_MAP[id], ENABLE);
  }
  else if (CH32_SPI_APB_MAP[id] == 2)
  {
    RCC_APB2PeriphClockCmd(CH32_SPI_RCC_PERIPH_MAP[id], ENABLE);
  }
  else
  {
    DEV_ASSERT(false);
  }
  RCC_AHBPeriphClockCmd(CH32_SPI_RCC_PERIPH_MAP_DMA[id], ENABLE);

  // GPIO 配置。
  // GPIO configuration.
  {
    GPIO_InitTypeDef gpio = {};
    gpio.GPIO_Speed = GPIO_Speed_50MHz;

    // SCK 引脚。
    // SCK.
    RCC_APB2PeriphClockCmd(ch32_get_gpio_periph(sck_port_), ENABLE);
    if (mode_ == SPI_Mode_Master)
    {
      gpio.GPIO_Pin = sck_pin_;
      gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    }
    else
    {
      gpio.GPIO_Pin = sck_pin_;
      gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    }
    GPIO_Init(sck_port_, &gpio);

    // MISO 引脚。
    // MISO.
    RCC_APB2PeriphClockCmd(ch32_get_gpio_periph(miso_port_), ENABLE);
    if (mode_ == SPI_Mode_Master)
    {
      gpio.GPIO_Pin = miso_pin_;
      gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    }
    else
    {
      gpio.GPIO_Pin = miso_pin_;
      gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    }
    GPIO_Init(miso_port_, &gpio);

    // MOSI 引脚。
    // MOSI.
    RCC_APB2PeriphClockCmd(ch32_get_gpio_periph(mosi_port_), ENABLE);
    if (mode_ == SPI_Mode_Master)
    {
      gpio.GPIO_Pin = mosi_pin_;
      gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    }
    else
    {
      gpio.GPIO_Pin = mosi_pin_;
      gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    }
    GPIO_Init(mosi_port_, &gpio);

    if (pin_remap != 0)
    {
      RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
      GPIO_PinRemapConfig(pin_remap, ENABLE);
    }
  }

  // SPI 基础配置。
  // SPI base configuration.
  {
    SPI_InitTypeDef init = {};
    init.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    init.SPI_Mode = mode_;
    init.SPI_DataSize = datasize_;
    init.SPI_CPOL =
        (config.clock_polarity == SPI::ClockPolarity::LOW) ? SPI_CPOL_Low : SPI_CPOL_High;
    init.SPI_CPHA =
        (config.clock_phase == SPI::ClockPhase::EDGE_1) ? SPI_CPHA_1Edge : SPI_CPHA_2Edge;
    init.SPI_NSS = nss_;
    init.SPI_BaudRatePrescaler = prescaler_;
    init.SPI_FirstBit = firstbit_;
    init.SPI_CRCPolynomial = 7;

    SPI_Init(instance_, &init);
    SPI_Cmd(instance_, ENABLE);
  }

  // DMA 通道基础配置（MADDR/CNTR 在运行时设置）。
  // DMA channel base configuration (MADDR/CNTR set at runtime).
  {
    // RX 通道。
    // RX.
    {
      ch32_dma_callback_t cb = [](void* arg)
      { reinterpret_cast<CH32SPI*>(arg)->RxDmaIRQHandler(); };
      ch32_dma_register_callback(ch32_dma_get_id(dma_rx_channel_), cb, this);

      DMA_InitTypeDef di = {};
      DMA_DeInit(dma_rx_channel_);
      di.DMA_PeripheralBaseAddr = (uint32_t)&instance_->DATAR;
      di.DMA_MemoryBaseAddr = (uint32_t)dma_rx.addr_;
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
    // TX 通道。
    // TX.
    {
      ch32_dma_callback_t cb = [](void* arg)
      { reinterpret_cast<CH32SPI*>(arg)->TxDmaIRQHandler(); };
      ch32_dma_register_callback(ch32_dma_get_id(dma_tx_channel_), cb, this);

      DMA_InitTypeDef di = {};
      DMA_DeInit(dma_tx_channel_);
      di.DMA_PeripheralBaseAddr = (uint32_t)&instance_->DATAR;
      di.DMA_MemoryBaseAddr = 0;
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

  // 同步基类配置，供速率查询辅助逻辑使用。
  // Sync the base-class configuration for rate-query helpers.
  GetConfig() = config;
  GetConfig().prescaler = MapCH32PrescalerToEnum(prescaler_);
}

// SetConfig 会更新 polarity / phase，并同步基类配置。
// SetConfig updates polarity / phase and synchronizes the base-class configuration.
ErrorCode CH32SPI::SetConfig(SPI::Configuration config)
{
  // 1) 把 SPI::Prescaler → CH32 Prescaler 宏
  uint16_t ch32_presc = 0;
  if (!MapEnumToCH32Prescaler(config.prescaler, ch32_presc))
  {
    return ErrorCode::NOT_SUPPORT;
  }

  // 2) 关 / 复位 / 重新初始化
  SPI_Cmd(instance_, DISABLE);
  SPI_I2S_DeInit(instance_);

  SPI_InitTypeDef init = {};
  init.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  init.SPI_Mode = mode_;
  init.SPI_DataSize = datasize_;
  init.SPI_CPOL =
      (config.clock_polarity == SPI::ClockPolarity::LOW) ? SPI_CPOL_Low : SPI_CPOL_High;
  init.SPI_CPHA =
      (config.clock_phase == SPI::ClockPhase::EDGE_1) ? SPI_CPHA_1Edge : SPI_CPHA_2Edge;
  init.SPI_NSS = nss_;
  init.SPI_BaudRatePrescaler = ch32_presc;
  init.SPI_FirstBit = firstbit_;
  init.SPI_CRCPolynomial = 7;

  SPI_Init(instance_, &init);
  SPI_Cmd(instance_, ENABLE);

  // 3) 同步本地缓存与基类配置（影响 GetBusSpeed/IsDoubleBuffer）
  prescaler_ = ch32_presc;
  GetConfig() = config;

  return ErrorCode::OK;
}

ErrorCode CH32SPI::PollingTransfer(uint8_t* rx, const uint8_t* tx, uint32_t len)
{
  for (uint32_t i = 0; i < len; ++i)
  {
    while (SPI_I2S_GetFlagStatus(instance_, SPI_I2S_FLAG_TXE) == RESET)
    {
    }
    SPI_I2S_SendData(instance_, tx ? tx[i] : 0x00);

    while (SPI_I2S_GetFlagStatus(instance_, SPI_I2S_FLAG_RXNE) == RESET)
    {
    }
    uint16_t d = SPI_I2S_ReceiveData(instance_);
    if (rx)
    {
      rx[i] = static_cast<uint8_t>(d & 0xff);
    }
  }
  return ErrorCode::OK;
}

ErrorCode CH32SPI::ReadAndWrite(RawData read_data, ConstRawData write_data,
                                OperationRW& op, bool in_isr)
{
  const uint32_t RSZ = read_data.size_;
  const uint32_t WSZ = write_data.size_;
  const uint32_t NEED = (RSZ > WSZ) ? RSZ : WSZ;

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

  ASSERT(rx.size_ >= NEED);
  ASSERT(tx.size_ >= NEED);

  if (NEED > dma_enable_min_size_)
  {
    mem_read_ = false;
    read_buff_ = read_data;
    rw_op_ = op;
    busy_ = true;

    PrepareTxBuffer(write_data, NEED);

    if (op.type == OperationRW::OperationType::BLOCK)
    {
      block_wait_.Start(*op.data.sem_info.sem);
    }
    StartDmaDuplex(NEED);

    op.MarkAsRunning();
    if (op.type == OperationRW::OperationType::BLOCK)
    {
      return block_wait_.Wait(op.data.sem_info.timeout);
    }
    return ErrorCode::OK;
  }
  else
  {
    // 轮询路径（使用活动缓冲）
    uint8_t* rxp = static_cast<uint8_t*>(rx.addr_);
    uint8_t* txp = static_cast<uint8_t*>(tx.addr_);

    // 准备 tx
    if (WSZ)
    {
      Memory::FastCopy(txp, write_data.addr_, WSZ);
      if (NEED > WSZ)
      {
        Memory::FastSet(txp + WSZ, 0x00, NEED - WSZ);
      }
    }
    else
    {
      Memory::FastSet(txp, 0x00, NEED);
    }

    ErrorCode ec = PollingTransfer(rxp, txp, NEED);

    if (RSZ)
    {
      Memory::FastCopy(read_data.addr_, rxp, RSZ);
    }

    SwitchBuffer();

    if (op.type != OperationRW::OperationType::BLOCK)
    {
      op.UpdateStatus(in_isr, ec);
    }
    return ec;
  }
}

ErrorCode CH32SPI::MemRead(uint16_t reg, RawData read_data, OperationRW& op, bool in_isr)
{
  const uint32_t N = read_data.size_;
  if (N == 0)
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

  ASSERT(rx.size_ >= N + 1);
  ASSERT(tx.size_ >= N + 1);

  const uint32_t TOTAL = N + 1;

  if (TOTAL > dma_enable_min_size_)
  {
    uint8_t* txp = static_cast<uint8_t*>(tx.addr_);
    txp[0] = static_cast<uint8_t>(reg | 0x80);
    Memory::FastSet(txp + 1, 0x00, N);

    mem_read_ = true;
    read_buff_ = read_data;
    rw_op_ = op;
    busy_ = true;

    if (op.type == OperationRW::OperationType::BLOCK)
    {
      block_wait_.Start(*op.data.sem_info.sem);
    }
    StartDmaDuplex(TOTAL);
    op.MarkAsRunning();
    if (op.type == OperationRW::OperationType::BLOCK)
    {
      return block_wait_.Wait(op.data.sem_info.timeout);
    }
    return ErrorCode::OK;
  }
  else
  {
    // 轮询
    uint8_t* rxp = static_cast<uint8_t*>(rx.addr_);
    uint8_t* txp = static_cast<uint8_t*>(tx.addr_);
    txp[0] = static_cast<uint8_t>(reg | 0x80);
    Memory::FastSet(txp + 1, 0x00, N);

    ErrorCode ec = PollingTransfer(rxp, txp, TOTAL);
    Memory::FastCopy(read_data.addr_, rxp + 1, N);

    SwitchBuffer();

    if (op.type != OperationRW::OperationType::BLOCK)
    {
      op.UpdateStatus(in_isr, ec);
    }
    return ec;
  }
}

ErrorCode CH32SPI::MemWrite(uint16_t reg, ConstRawData write_data, OperationRW& op,
                            bool in_isr)
{
  const uint32_t N = write_data.size_;
  if (N == 0)
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
  (void)rx;  // RX 仅用于丢弃

  ASSERT(tx.size_ >= N + 1);

  const uint32_t TOTAL = N + 1;

  if (TOTAL > dma_enable_min_size_)
  {
    uint8_t* txp = static_cast<uint8_t*>(tx.addr_);
    txp[0] = static_cast<uint8_t>(reg & 0x7F);
    Memory::FastCopy(txp + 1, write_data.addr_, N);

    mem_read_ = false;
    read_buff_ = {nullptr, 0};
    rw_op_ = op;
    busy_ = true;

    if (op.type == OperationRW::OperationType::BLOCK)
    {
      block_wait_.Start(*op.data.sem_info.sem);
    }
    StartDmaDuplex(TOTAL);

    op.MarkAsRunning();
    if (op.type == OperationRW::OperationType::BLOCK)
    {
      return block_wait_.Wait(op.data.sem_info.timeout);
    }
    return ErrorCode::OK;
  }
  else
  {
    // 轮询
    uint8_t* rxp = static_cast<uint8_t*>(rx.addr_);
    uint8_t* txp = static_cast<uint8_t*>(tx.addr_);
    txp[0] = static_cast<uint8_t>(reg & 0x7F);
    Memory::FastCopy(txp + 1, write_data.addr_, N);

    ErrorCode ec = PollingTransfer(rxp, txp, TOTAL);

    SwitchBuffer();

    if (op.type != OperationRW::OperationType::BLOCK)
    {
      op.UpdateStatus(in_isr, ec);
    }
    return ec;
  }
}

void CH32SPI::PrepareTxBuffer(ConstRawData write_data, uint32_t need_len, uint32_t prefix,
                              uint8_t dummy)
{
  RawData tx = GetTxBuffer();
  uint8_t* txp = static_cast<uint8_t*>(tx.addr_);

  if (write_data.size_ > 0)
  {
    const uint32_t COPY =
        (write_data.size_ > need_len - prefix) ? (need_len - prefix) : write_data.size_;
    Memory::FastCopy(txp + prefix, write_data.addr_, COPY);
    if (prefix + COPY < need_len)
    {
      Memory::FastSet(txp + prefix + COPY, dummy, need_len - prefix - COPY);
    }
  }
  else
  {
    Memory::FastSet(txp + prefix, dummy, need_len - prefix);
  }
}

void CH32SPI::StartDmaDuplex(uint32_t count)
{
  RawData rx = GetRxBuffer();
  RawData tx = GetTxBuffer();

  dma_rx_channel_->MADDR = reinterpret_cast<uint32_t>(rx.addr_);
  dma_rx_channel_->CNTR = count;

  dma_tx_channel_->MADDR = reinterpret_cast<uint32_t>(tx.addr_);
  dma_tx_channel_->CNTR = count;

  SPI_I2S_DMACmd(instance_, SPI_I2S_DMAReq_Rx, ENABLE);
  SPI_I2S_DMACmd(instance_, SPI_I2S_DMAReq_Tx, ENABLE);

  DMA_Cmd(dma_rx_channel_, ENABLE);
  DMA_Cmd(dma_tx_channel_, ENABLE);
}

void CH32SPI::StopDma()
{
  DMA_Cmd(dma_tx_channel_, DISABLE);
  DMA_Cmd(dma_rx_channel_, DISABLE);
}

void CH32SPI::RxDmaIRQHandler()
{
  if (DMA_GetITStatus(CH32_SPI_RX_DMA_IT_MAP[id_]) == RESET)
  {
    return;
  }

  DMA_ClearITPendingBit(CH32_SPI_RX_DMA_IT_MAP[id_]);

  SPI_I2S_DMACmd(instance_, SPI_I2S_DMAReq_Rx, DISABLE);
  DMA_Cmd(dma_rx_channel_, DISABLE);

  if (!busy_)
  {
    return;
  }

  // 拷贝读数据（若有）
  const bool has_user_read_copy = (read_buff_.size_ > 0);
  if (has_user_read_copy)
  {
    RawData rx = GetRxBuffer();
    uint8_t* rxp = static_cast<uint8_t*>(rx.addr_);
    if (mem_read_)
    {
      Memory::FastCopy(read_buff_.addr_, rxp + 1, read_buff_.size_);
    }
    else
    {
      Memory::FastCopy(read_buff_.addr_, rxp, read_buff_.size_);
    }
    read_buff_.size_ = 0;
  }

  // 双缓冲切换与状态更新
  SwitchBuffer();
  busy_ = false;
  if (rw_op_.type == OperationRW::OperationType::BLOCK)
  {
    (void)block_wait_.TryPost(true, ErrorCode::OK);
  }
  else
  {
    rw_op_.UpdateStatus(true, ErrorCode::OK);
  }
}

void CH32SPI::TxDmaIRQHandler()
{
  if (DMA_GetITStatus(CH32_SPI_TX_DMA_IT_MAP[id_]) == RESET)
  {
    return;
  }
  DMA_ClearITPendingBit(CH32_SPI_TX_DMA_IT_MAP[id_]);

  SPI_I2S_DMACmd(instance_, SPI_I2S_DMAReq_Tx, DISABLE);
  DMA_Cmd(dma_tx_channel_, DISABLE);

  if (!busy_)
  {
    return;
  }
}

// 零拷贝传输路径。
// Zero-copy transfer path.
ErrorCode CH32SPI::Transfer(size_t size, OperationRW& op, bool in_isr)
{
  if (DmaBusy())
  {
    return ErrorCode::BUSY;
  }

  if (size == 0)
  {
    if (op.type != OperationRW::OperationType::BLOCK)
    {
      op.UpdateStatus(in_isr, ErrorCode::OK);
    }
    return ErrorCode::OK;
  }

  RawData rx = GetRxBuffer();
  RawData tx = GetTxBuffer();
  ASSERT(rx.size_ >= size);
  ASSERT(tx.size_ >= size);

  if (size > dma_enable_min_size_)
  {
    rw_op_ = op;
    read_buff_ = {nullptr, 0};
    mem_read_ = false;
    busy_ = true;

    if (op.type == OperationRW::OperationType::BLOCK)
    {
      block_wait_.Start(*op.data.sem_info.sem);
    }
    StartDmaDuplex(static_cast<uint32_t>(size));

    op.MarkAsRunning();
    if (op.type == OperationRW::OperationType::BLOCK)
    {
      return block_wait_.Wait(op.data.sem_info.timeout);
    }
    return ErrorCode::OK;
  }

  // 小包轮询
  ErrorCode ec =
      PollingTransfer(static_cast<uint8_t*>(rx.addr_),
                      static_cast<const uint8_t*>(tx.addr_), static_cast<uint32_t>(size));

  SwitchBuffer();

  if (op.type != OperationRW::OperationType::BLOCK)
  {
    op.UpdateStatus(in_isr, ec);
  }
  return ec;
}

// 总线最大时钟频率（Hz）。
// Maximum bus clock in Hz.
uint32_t CH32SPI::GetMaxBusSpeed() const
{
  RCC_ClocksTypeDef clocks{};
  RCC_GetClocksFreq(&clocks);

  if (CH32_SPI_APB_MAP[id_] == 2)
  {
    return clocks.PCLK2_Frequency;
  }
  else if (CH32_SPI_APB_MAP[id_] == 1)
  {
    return clocks.PCLK1_Frequency;
  }
  return 0u;
}

// 可用的最大分频枚举值。
// Maximum available prescaler enumeration.
SPI::Prescaler CH32SPI::GetMaxPrescaler() const
{
#if defined(SPI_BaudRatePrescaler_1024)
  return SPI::Prescaler::DIV_1024;
#elif defined(SPI_BaudRatePrescaler_512)
  return SPI::Prescaler::DIV_512;
#elif defined(SPI_BaudRatePrescaler_256)
  return SPI::Prescaler::DIV_256;
#elif defined(SPI_BaudRatePrescaler_128)
  return SPI::Prescaler::DIV_128;
#elif defined(SPI_BaudRatePrescaler_64)
  return SPI::Prescaler::DIV_64;
#elif defined(SPI_BaudRatePrescaler_32)
  return SPI::Prescaler::DIV_32;
#elif defined(SPI_BaudRatePrescaler_16)
  return SPI::Prescaler::DIV_16;
#elif defined(SPI_BaudRatePrescaler_8)
  return SPI::Prescaler::DIV_8;
#elif defined(SPI_BaudRatePrescaler_4)
  return SPI::Prescaler::DIV_4;
#elif defined(SPI_BaudRatePrescaler_2)
  return SPI::Prescaler::DIV_2;
#else
  return SPI::Prescaler::UNKNOWN;
#endif
}

SPI::Prescaler CH32SPI::MapCH32PrescalerToEnum(uint16_t p)
{
  switch (p)
  {
#if defined(SPI_BaudRatePrescaler_2)
    case SPI_BaudRatePrescaler_2:
      return SPI::Prescaler::DIV_2;
#endif
#if defined(SPI_BaudRatePrescaler_4)
    case SPI_BaudRatePrescaler_4:
      return SPI::Prescaler::DIV_4;
#endif
#if defined(SPI_BaudRatePrescaler_8)
    case SPI_BaudRatePrescaler_8:
      return SPI::Prescaler::DIV_8;
#endif
#if defined(SPI_BaudRatePrescaler_16)
    case SPI_BaudRatePrescaler_16:
      return SPI::Prescaler::DIV_16;
#endif
#if defined(SPI_BaudRatePrescaler_32)
    case SPI_BaudRatePrescaler_32:
      return SPI::Prescaler::DIV_32;
#endif
#if defined(SPI_BaudRatePrescaler_64)
    case SPI_BaudRatePrescaler_64:
      return SPI::Prescaler::DIV_64;
#endif
#if defined(SPI_BaudRatePrescaler_128)
    case SPI_BaudRatePrescaler_128:
      return SPI::Prescaler::DIV_128;
#endif
#if defined(SPI_BaudRatePrescaler_256)
    case SPI_BaudRatePrescaler_256:
      return SPI::Prescaler::DIV_256;
#endif
    default:
      return SPI::Prescaler::UNKNOWN;
  }
}

// NOLINTEND(cppcoreguidelines-pro-type-cstyle-cast,performance-no-int-to-ptr)
