#include "stm32_spi.hpp"

#include "libxr_def.hpp"
#include "stm32_dcache.hpp"

#ifdef HAL_SPI_MODULE_ENABLED

using namespace LibXR;

STM32SPI* STM32SPI::map[STM32_SPI_NUMBER] = {nullptr};

stm32_spi_id_t STM32_SPI_GetID(SPI_TypeDef* addr)
{
  if (addr == nullptr)
  {  // NOLINT
    return stm32_spi_id_t::STM32_SPI_ID_ERROR;
  }
#ifdef SPI1
  else if (addr == SPI1)
  {  // NOLINT
    return stm32_spi_id_t::STM32_SPI1;
  }
#endif
#ifdef SPI2
  else if (addr == SPI2)
  {  // NOLINT
    return stm32_spi_id_t::STM32_SPI2;
  }
#endif
#ifdef SPI3
  else if (addr == SPI3)
  {  // NOLINT
    return stm32_spi_id_t::STM32_SPI3;
  }
#endif
#ifdef SPI4
  else if (addr == SPI4)
  {  // NOLINT
    return stm32_spi_id_t::STM32_SPI4;
  }
#endif
#ifdef SPI5
  else if (addr == SPI5)
  {  // NOLINT
    return stm32_spi_id_t::STM32_SPI5;
  }
#endif
#ifdef SPI6
  else if (addr == SPI6)
  {  // NOLINT
    return stm32_spi_id_t::STM32_SPI6;
  }
#endif
#ifdef SPI7
  else if (addr == SPI7)
  {  // NOLINT
    return stm32_spi_id_t::STM32_SPI7;
  }
#endif
#ifdef SPI8
  else if (addr == SPI8)
  {  // NOLINT
    return stm32_spi_id_t::STM32_SPI8;
  }
#endif
  else
  {
    return stm32_spi_id_t::STM32_SPI_ID_ERROR;
  }
}

STM32SPI::STM32SPI(SPI_HandleTypeDef* spi_handle, RawData rx, RawData tx,
                   uint32_t dma_enable_min_size)
    : SPI(rx, tx),
      spi_handle_(spi_handle),
      id_(STM32_SPI_GetID(spi_handle_->Instance)),
      dma_enable_min_size_(dma_enable_min_size)
{
  ASSERT(id_ != STM32_SPI_ID_ERROR);

  map[id_] = this;
}

ErrorCode STM32SPI::ReadAndWrite(RawData read_data, ConstRawData write_data,
                                 OperationRW& op, bool in_isr)
{
  uint32_t need_write = max(write_data.size_, read_data.size_);

  RawData rx = GetRxBuffer();
  RawData tx = GetTxBuffer();

  if (rx.size_ > 0)
  {
    ASSERT(need_write <= rx.size_);
  }
  if (tx.size_ > 0)
  {
    ASSERT(need_write <= tx.size_);
  }

  if (spi_handle_->State != HAL_SPI_STATE_READY)
  {
    return ErrorCode::BUSY;
  }

  mem_read_ = false;

  if (need_write > dma_enable_min_size_)
  {
    rw_op_ = op;
    if (op.type == OperationRW::OperationType::BLOCK)
    {
      block_wait_.Start(*op.data.sem_info.sem);
    }

    HAL_StatusTypeDef st = HAL_OK;

    if (write_data.size_ > 0 && read_data.size_ > 0)
    {
      Memory::FastCopy(tx.addr_, write_data.addr_, write_data.size_);
      if (write_data.size_ < need_write)
      {
        Memory::FastSet(reinterpret_cast<uint8_t*>(tx.addr_) + write_data.size_, 0,
                        need_write - write_data.size_);
      }
      STM32_CleanDCacheByAddr(tx.addr_, need_write);
      read_buff_ = read_data;

      st = HAL_SPI_TransmitReceive_DMA(spi_handle_, static_cast<uint8_t*>(tx.addr_),
                                       static_cast<uint8_t*>(rx.addr_), need_write);
    }
    else if (write_data.size_ > 0)
    {
      Memory::FastCopy(tx.addr_, write_data.addr_, write_data.size_);
      if (write_data.size_ < need_write)
      {
        Memory::FastSet(reinterpret_cast<uint8_t*>(tx.addr_) + write_data.size_, 0,
                        need_write - write_data.size_);
      }
      STM32_CleanDCacheByAddr(tx.addr_, need_write);
      read_buff_ = {nullptr, 0};

      st = HAL_SPI_Transmit_DMA(spi_handle_, static_cast<uint8_t*>(tx.addr_), need_write);
    }
    else if (read_data.size_ > 0)
    {
      read_buff_ = read_data;

      st = HAL_SPI_Receive_DMA(spi_handle_, static_cast<uint8_t*>(rx.addr_),
                               read_data.size_);
    }
    else
    {
      if (op.type != OperationRW::OperationType::BLOCK)
      {
        op.UpdateStatus(in_isr, ErrorCode::OK);
      }
      return ErrorCode::OK;
    }

    if (st != HAL_OK)
    {
      if (op.type == OperationRW::OperationType::BLOCK)
      {
        block_wait_.Cancel();
      }
      return ErrorCode::BUSY;
    }

    op.MarkAsRunning();
    if (op.type == OperationRW::OperationType::BLOCK)
    {
      return block_wait_.Wait(op.data.sem_info.timeout);
    }
    return ErrorCode::OK;
  }

  ErrorCode ans = ErrorCode::OK;

  if (write_data.size_ > 0 && read_data.size_ > 0)
  {
    Memory::FastCopy(tx.addr_, write_data.addr_, write_data.size_);
    if (write_data.size_ < need_write)
    {
      Memory::FastSet(reinterpret_cast<uint8_t*>(tx.addr_) + write_data.size_, 0,
                      need_write - write_data.size_);
    }
    STM32_CleanDCacheByAddr(tx.addr_, need_write);
    ans = (HAL_SPI_TransmitReceive(spi_handle_, static_cast<uint8_t*>(tx.addr_),
                                   static_cast<uint8_t*>(rx.addr_), need_write,
                                   20) == HAL_OK)
              ? ErrorCode::OK
              : ErrorCode::BUSY;

    if (ans == ErrorCode::OK)
    {
      Memory::FastCopy(read_data.addr_, rx.addr_, read_data.size_);
    }
    SwitchBuffer();
  }
  else if (read_data.size_ > 0)
  {
    ans = (HAL_SPI_Receive(spi_handle_, static_cast<uint8_t*>(rx.addr_), read_data.size_,
                           20) == HAL_OK)
              ? ErrorCode::OK
              : ErrorCode::BUSY;

    if (ans == ErrorCode::OK)
    {
      Memory::FastCopy(read_data.addr_, rx.addr_, read_data.size_);
    }
    SwitchBuffer();
  }
  else if (write_data.size_ > 0)
  {
    Memory::FastCopy(tx.addr_, write_data.addr_, write_data.size_);
    STM32_CleanDCacheByAddr(tx.addr_, need_write);
    ans = (HAL_SPI_Transmit(spi_handle_, static_cast<uint8_t*>(tx.addr_),
                            write_data.size_, 20) == HAL_OK)
              ? ErrorCode::OK
              : ErrorCode::BUSY;

    SwitchBuffer();
  }
  else
  {
    if (op.type != OperationRW::OperationType::BLOCK)
    {
      op.UpdateStatus(in_isr, ErrorCode::OK);
    }
    return ErrorCode::OK;
  }

  if (op.type != OperationRW::OperationType::BLOCK)
  {
    op.UpdateStatus(in_isr, ans);
  }
  return ans;
}

ErrorCode STM32SPI::SetConfig(SPI::Configuration config)
{
  switch (config.clock_polarity)
  {
    case SPI::ClockPolarity::LOW:
      spi_handle_->Init.CLKPolarity = SPI_POLARITY_LOW;
      break;
    case SPI::ClockPolarity::HIGH:
      spi_handle_->Init.CLKPolarity = SPI_POLARITY_HIGH;
      break;
  }

  switch (config.clock_phase)
  {
    case SPI::ClockPhase::EDGE_1:
      spi_handle_->Init.CLKPhase = SPI_PHASE_1EDGE;
      break;
    case SPI::ClockPhase::EDGE_2:
      spi_handle_->Init.CLKPhase = SPI_PHASE_2EDGE;
      break;
  }

  bool ok = true;
  uint32_t hal_presc = 0;

  switch (config.prescaler)
  {
#ifdef SPI_BAUDRATEPRESCALER_1
    case Prescaler::DIV_1:
      hal_presc = SPI_BAUDRATEPRESCALER_1;
      break;
#endif
#ifdef SPI_BAUDRATEPRESCALER_2
    case Prescaler::DIV_2:
      hal_presc = SPI_BAUDRATEPRESCALER_2;
      break;
#endif
#ifdef SPI_BAUDRATEPRESCALER_4
    case Prescaler::DIV_4:
      hal_presc = SPI_BAUDRATEPRESCALER_4;
      break;
#endif
#ifdef SPI_BAUDRATEPRESCALER_8
    case Prescaler::DIV_8:
      hal_presc = SPI_BAUDRATEPRESCALER_8;
      break;
#endif
#ifdef SPI_BAUDRATEPRESCALER_16
    case Prescaler::DIV_16:
      hal_presc = SPI_BAUDRATEPRESCALER_16;
      break;
#endif
#ifdef SPI_BAUDRATEPRESCALER_32
    case Prescaler::DIV_32:
      hal_presc = SPI_BAUDRATEPRESCALER_32;
      break;
#endif
#ifdef SPI_BAUDRATEPRESCALER_64
    case Prescaler::DIV_64:
      hal_presc = SPI_BAUDRATEPRESCALER_64;
      break;
#endif
#ifdef SPI_BAUDRATEPRESCALER_128
    case Prescaler::DIV_128:
      hal_presc = SPI_BAUDRATEPRESCALER_128;
      break;
#endif
#ifdef SPI_BAUDRATEPRESCALER_256
    case Prescaler::DIV_256:
      hal_presc = SPI_BAUDRATEPRESCALER_256;
      break;
#endif
#ifdef SPI_BAUDRATEPRESCALER_512
    case Prescaler::DIV_512:
      hal_presc = SPI_BAUDRATEPRESCALER_512;
      break;
#endif
#ifdef SPI_BAUDRATEPRESCALER_1024
    case Prescaler::DIV_1024:
      hal_presc = SPI_BAUDRATEPRESCALER_1024;
      break;
#endif
    default:
      ok = false;
      break;
  }

  if (!ok)
  {
    return ErrorCode::NOT_SUPPORT;
  }

  spi_handle_->Init.BaudRatePrescaler = hal_presc;

  GetConfig() = config;

  return (HAL_SPI_Init(spi_handle_) == HAL_OK) ? ErrorCode::OK : ErrorCode::BUSY;
}

ErrorCode STM32SPI::MemRead(uint16_t reg, RawData read_data, OperationRW& op, bool in_isr)
{
  uint32_t need_read = read_data.size_;

  if (spi_handle_->State != HAL_SPI_STATE_READY)
  {
    return ErrorCode::BUSY;
  }

  RawData rx = GetRxBuffer();
  RawData tx = GetTxBuffer();

  ASSERT(rx.size_ >= need_read + 1);
  ASSERT(tx.size_ >= need_read + 1);

  if (need_read + 1 > dma_enable_min_size_)
  {
    mem_read_ = true;
    rw_op_ = op;
    if (op.type == OperationRW::OperationType::BLOCK)
    {
      block_wait_.Start(*op.data.sem_info.sem);
    }

    uint8_t* txb = reinterpret_cast<uint8_t*>(tx.addr_);
    Memory::FastSet(txb, 0, need_read + 1);
    txb[0] = static_cast<uint8_t>(reg | 0x80);
    STM32_CleanDCacheByAddr(tx.addr_, need_read + 1);
    read_buff_ = read_data;

    HAL_StatusTypeDef st =
        HAL_SPI_TransmitReceive_DMA(spi_handle_, static_cast<uint8_t*>(tx.addr_),
                                    static_cast<uint8_t*>(rx.addr_), need_read + 1);

    if (st != HAL_OK)
    {
      if (op.type == OperationRW::OperationType::BLOCK)
      {
        block_wait_.Cancel();
      }
      return ErrorCode::BUSY;
    }

    op.MarkAsRunning();
    if (op.type == OperationRW::OperationType::BLOCK)
    {
      return block_wait_.Wait(op.data.sem_info.timeout);
    }
    return ErrorCode::OK;
  }

  mem_read_ = false;
  uint8_t* txb = reinterpret_cast<uint8_t*>(tx.addr_);
  Memory::FastSet(txb, 0, need_read + 1);
  txb[0] = static_cast<uint8_t>(reg | 0x80);
  STM32_CleanDCacheByAddr(tx.addr_, need_read + 1);

  ErrorCode ans = (HAL_SPI_TransmitReceive(spi_handle_, static_cast<uint8_t*>(tx.addr_),
                                           static_cast<uint8_t*>(rx.addr_), need_read + 1,
                                           20) == HAL_OK)
                      ? ErrorCode::OK
                      : ErrorCode::BUSY;

  if (ans == ErrorCode::OK)
  {
    uint8_t* rxb = reinterpret_cast<uint8_t*>(rx.addr_);
    Memory::FastCopy(read_data.addr_, rxb + 1, need_read);
  }
  SwitchBuffer();

  if (op.type != OperationRW::OperationType::BLOCK)
  {
    op.UpdateStatus(in_isr, ans);
  }
  return ans;
}

ErrorCode STM32SPI::MemWrite(uint16_t reg, ConstRawData write_data, OperationRW& op,
                             bool in_isr)
{
  uint32_t need_write = write_data.size_;

  if (spi_handle_->State != HAL_SPI_STATE_READY)
  {
    return ErrorCode::BUSY;
  }

  RawData tx = GetTxBuffer();
  ASSERT(tx.size_ >= need_write + 1);

  if (need_write + 1 > dma_enable_min_size_)
  {
    mem_read_ = false;
    rw_op_ = op;
    if (op.type == OperationRW::OperationType::BLOCK)
    {
      block_wait_.Start(*op.data.sem_info.sem);
    }

    uint8_t* txb = reinterpret_cast<uint8_t*>(tx.addr_);
    txb[0] = static_cast<uint8_t>(reg & 0x7F);
    Memory::FastCopy(txb + 1, write_data.addr_, need_write);
    STM32_CleanDCacheByAddr(tx.addr_, need_write + 1);
    read_buff_ = {nullptr, 0};

    HAL_StatusTypeDef st = HAL_SPI_Transmit_DMA(
        spi_handle_, static_cast<uint8_t*>(tx.addr_), need_write + 1);

    if (st != HAL_OK)
    {
      if (op.type == OperationRW::OperationType::BLOCK)
      {
        block_wait_.Cancel();
      }
      return ErrorCode::BUSY;
    }

    op.MarkAsRunning();
    if (op.type == OperationRW::OperationType::BLOCK)
    {
      return block_wait_.Wait(op.data.sem_info.timeout);
    }
    return ErrorCode::OK;
  }

  uint8_t* txb = reinterpret_cast<uint8_t*>(tx.addr_);
  txb[0] = static_cast<uint8_t>(reg & 0x7F);
  Memory::FastCopy(txb + 1, write_data.addr_, need_write);
  STM32_CleanDCacheByAddr(tx.addr_, need_write + 1);

  ErrorCode ans = (HAL_SPI_Transmit(spi_handle_, static_cast<uint8_t*>(tx.addr_),
                                    need_write + 1, 20) == HAL_OK)
                      ? ErrorCode::OK
                      : ErrorCode::BUSY;

  SwitchBuffer();

  if (op.type != OperationRW::OperationType::BLOCK)
  {
    op.UpdateStatus(in_isr, ans);
  }
  return ans;
}

uint32_t STM32SPI::GetMaxBusSpeed() const
{
  SPI_TypeDef* inst = spi_handle_ ? spi_handle_->Instance : nullptr;
  if (!inst)
  {
    return 0u;
  }

#if defined(HAL_RCC_MODULE_ENABLED)
// 1) 优先读取独立 SPI 内核时钟。
// 1) Prefer dedicated SPI kernel clock when available.
// H7: 分组宏 SPI123 / SPI45 / SPI6
#if defined(RCC_PERIPHCLK_SPI123) || defined(RCC_PERIPHCLK_SPI45) || \
    defined(RCC_PERIPHCLK_SPI6)
  // SPI1/2/3 → RCC_PERIPHCLK_SPI123
  if (
#ifdef SPI1
      inst == SPI1 ||
#endif
#ifdef SPI2
      inst == SPI2 ||
#endif
#ifdef SPI3
      inst == SPI3 ||
#endif
      false)
  {
#ifdef RCC_PERIPHCLK_SPI123
    return HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI123);
#endif
  }
  // SPI4/5 → RCC_PERIPHCLK_SPI45
  if (
#ifdef SPI4
      inst == SPI4 ||
#endif
#ifdef SPI5
      inst == SPI5 ||
#endif
      false)
  {
#ifdef RCC_PERIPHCLK_SPI45
    return HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI45);
#endif
  }
// SPI6 → RCC_PERIPHCLK_SPI6
#ifdef SPI6
  if (inst == SPI6)
  {
#ifdef RCC_PERIPHCLK_SPI6
    return HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI6);
#endif
  }
#endif
#endif  // H7 分组

// 其它系列：通常是逐个 SPIx 宏
#if defined(RCC_PERIPHCLK_SPI1)
#ifdef SPI1
  if (inst == SPI1) return HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI1);
#endif
#endif
#if defined(RCC_PERIPHCLK_SPI2)
#ifdef SPI2
  if (inst == SPI2) return HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI2);
#endif
#endif
#if defined(RCC_PERIPHCLK_SPI3)
#ifdef SPI3
  if (inst == SPI3) return HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI3);
#endif
#endif
#if defined(RCC_PERIPHCLK_SPI4)
#ifdef SPI4
  if (inst == SPI4) return HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI4);
#endif
#endif
#if defined(RCC_PERIPHCLK_SPI5)
#ifdef SPI5
  if (inst == SPI5) return HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI5);
#endif
#endif
#if defined(RCC_PERIPHCLK_SPI6)
#ifdef SPI6
  if (inst == SPI6) return HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI6);
#endif
#endif
#endif  // HAL_RCC_MODULE_ENABLED

  // 2) 回退到所在 APB 的 PCLK。
  // 2) Fallback to APB PCLK if no dedicated kernel clock is available.
#if defined(STM32H7) && defined(SPI6)
  if (inst == SPI6)
  {
#if defined(HAL_RCC_GetPCLK4Freq)
    return HAL_RCC_GetPCLK4Freq();
#else
    return HAL_RCC_GetHCLKFreq();
#endif
  }
#endif

  // 大多数系列：SPI1/4/5/6/7/8 → APB2； / SPI2/3 → APB1
  if (
#ifdef SPI1
      inst == SPI1 ||
#endif
#ifdef SPI4
      inst == SPI4 ||
#endif
#ifdef SPI5
      inst == SPI5 ||
#endif
#ifdef SPI6
      inst == SPI6 ||
#endif
#ifdef SPI7
      inst == SPI7 ||
#endif
#ifdef SPI8
      inst == SPI8 ||
#endif
      false)
  {
#if defined(HAL_RCC_GetPCLK2Freq)
    return HAL_RCC_GetPCLK2Freq();
#elif defined(HAL_RCC_GetPCLK1Freq)
    return HAL_RCC_GetPCLK1Freq();  // 低端/单APB系列兜底
#else
    return HAL_RCC_GetHCLKFreq();
#endif
  }
  else
  {
#if defined(HAL_RCC_GetPCLK1Freq)
    return HAL_RCC_GetPCLK1Freq();
#elif defined(HAL_RCC_GetPCLK2Freq)
    return HAL_RCC_GetPCLK2Freq();
#else
    return HAL_RCC_GetHCLKFreq();
#endif
  }
}

STM32SPI::Prescaler STM32SPI::GetMaxPrescaler() const
{
#if defined(SPI_BAUDRATEPRESCALER_1024)
  return Prescaler::DIV_1024;
#elif defined(SPI_BAUDRATEPRESCALER_512)
  return Prescaler::DIV_512;
#elif defined(SPI_BAUDRATEPRESCALER_256)
  return Prescaler::DIV_256;
#elif defined(SPI_BAUDRATEPRESCALER_128)
  return Prescaler::DIV_128;
#elif defined(SPI_BAUDRATEPRESCALER_64)
  return Prescaler::DIV_64;
#elif defined(SPI_BAUDRATEPRESCALER_32)
  return Prescaler::DIV_32;
#elif defined(SPI_BAUDRATEPRESCALER_16)
  return Prescaler::DIV_16;
#elif defined(SPI_BAUDRATEPRESCALER_8)
  return Prescaler::DIV_8;
#elif defined(SPI_BAUDRATEPRESCALER_4)
  return Prescaler::DIV_4;
#elif defined(SPI_BAUDRATEPRESCALER_2)
  return Prescaler::DIV_2;
#elif defined(SPI_BAUDRATEPRESCALER_1)
  return Prescaler::DIV_1;
#else
  return Prescaler::UNKNOWN;
#endif
}

ErrorCode STM32SPI::Transfer(size_t size, OperationRW& op, bool in_isr)
{
  if (spi_handle_->State != HAL_SPI_STATE_READY)
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

  const uint16_t xfer = static_cast<uint16_t>(size);

  if (size > dma_enable_min_size_)
  {
    rw_op_ = op;
    if (op.type == OperationRW::OperationType::BLOCK)
    {
      block_wait_.Start(*op.data.sem_info.sem);
    }

    STM32_CleanDCacheByAddr(tx.addr_, xfer);
    STM32_InvalidateDCacheByAddr(rx.addr_, xfer);

    HAL_StatusTypeDef st =
        HAL_SPI_TransmitReceive_DMA(spi_handle_, static_cast<uint8_t*>(tx.addr_),
                                    static_cast<uint8_t*>(rx.addr_), xfer);

    if (st != HAL_OK)
    {
      if (op.type == OperationRW::OperationType::BLOCK)
      {
        block_wait_.Cancel();
      }
      return ErrorCode::BUSY;
    }

    op.MarkAsRunning();
    if (op.type == OperationRW::OperationType::BLOCK)
    {
      return block_wait_.Wait(op.data.sem_info.timeout);
    }
    return ErrorCode::OK;
  }

  ErrorCode ans =
      (HAL_SPI_TransmitReceive(spi_handle_, static_cast<uint8_t*>(tx.addr_),
                               static_cast<uint8_t*>(rx.addr_), xfer, 20) == HAL_OK)
          ? ErrorCode::OK
          : ErrorCode::BUSY;

  STM32_InvalidateDCacheByAddr(rx.addr_, xfer);

  SwitchBuffer();

  if (op.type != OperationRW::OperationType::BLOCK)
  {
    op.UpdateStatus(in_isr, ans);
  }
  return ans;
}

extern "C" void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef* hspi)
{
  const auto spi_id = STM32_SPI_GetID(hspi->Instance);
  STM32SPI* spi = (spi_id == STM32_SPI_ID_ERROR) ? nullptr : STM32SPI::map[spi_id];
  if (spi == nullptr)
  {
    return;
  }
  if (spi->rw_op_.type == STM32SPI::OperationRW::OperationType::NONE)
  {
    return;
  }
  spi->SwitchBuffer();
  if (spi->rw_op_.type == STM32SPI::OperationRW::OperationType::BLOCK)
  {
    (void)spi->block_wait_.TryPost(true, ErrorCode::OK);
  }
  else
  {
    spi->rw_op_.UpdateStatus(true, ErrorCode::OK);
  }
}

extern "C" void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef* hspi)
{
  const auto spi_id = STM32_SPI_GetID(hspi->Instance);
  STM32SPI* spi = (spi_id == STM32_SPI_ID_ERROR) ? nullptr : STM32SPI::map[spi_id];
  if (spi == nullptr)
  {
    return;
  }
  if (spi->rw_op_.type == STM32SPI::OperationRW::OperationType::NONE)
  {
    return;
  }

  RawData rx = spi->GetRxBuffer();
  const bool has_user_read_copy = (spi->read_buff_.size_ > 0);

  if (has_user_read_copy)
  {
    if (!spi->mem_read_)
    {
      STM32_InvalidateDCacheByAddr(rx.addr_, spi->read_buff_.size_);
      Memory::FastCopy(spi->read_buff_.addr_, rx.addr_, spi->read_buff_.size_);
    }
    else
    {
      STM32_InvalidateDCacheByAddr(rx.addr_, spi->read_buff_.size_ + 1);
      uint8_t* rx_dma_buff = reinterpret_cast<uint8_t*>(rx.addr_);
      Memory::FastCopy(spi->read_buff_.addr_, rx_dma_buff + 1, spi->read_buff_.size_);
    }
    spi->read_buff_.size_ = 0;
  }

  spi->SwitchBuffer();
  if (spi->rw_op_.type == STM32SPI::OperationRW::OperationType::BLOCK)
  {
    (void)spi->block_wait_.TryPost(true, ErrorCode::OK);
  }
  else
  {
    spi->rw_op_.UpdateStatus(true, ErrorCode::OK);
  }
}

extern "C" void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi)
{
  HAL_SPI_RxCpltCallback(hspi);
}

extern "C" void HAL_SPI_ErrorCallback(SPI_HandleTypeDef* hspi)
{
  const auto spi_id = STM32_SPI_GetID(hspi->Instance);
  STM32SPI* spi = (spi_id == STM32_SPI_ID_ERROR) ? nullptr : STM32SPI::map[spi_id];
  if (spi == nullptr)
  {
    return;
  }
  if (spi->rw_op_.type == STM32SPI::OperationRW::OperationType::NONE)
  {
    return;
  }
  if (spi->rw_op_.type == STM32SPI::OperationRW::OperationType::BLOCK)
  {
    (void)spi->block_wait_.TryPost(true, ErrorCode::FAILED);
  }
  else
  {
    spi->rw_op_.UpdateStatus(true, ErrorCode::FAILED);
  }
}

#endif
