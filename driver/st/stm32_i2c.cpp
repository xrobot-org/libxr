#include "stm32_i2c.hpp"

#include "stm32_dcache.hpp"
#include "stm32_i2c_timing.hpp"
#if defined(STM32F7)
#include "stm32f7xx_ll_rcc.h"
#endif
#if defined(STM32H7)
#include "stm32h7xx_ll_rcc.h"
#endif
#ifdef HAL_I2C_MODULE_ENABLED

using namespace LibXR;

STM32I2C* STM32I2C::map[STM32_I2C_NUMBER] = {nullptr};

stm32_i2c_id_t STM32_I2C_GetID(I2C_TypeDef* hi2c)
{  // NOLINT
  if (hi2c == nullptr)
  {
    return stm32_i2c_id_t::STM32_I2C_ID_ERROR;
  }
#ifdef I2C1
  else if (hi2c == I2C1)
  {  // NOLINT
    return stm32_i2c_id_t::STM32_I2C1;
  }
#endif
#ifdef I2C2
  else if (hi2c == I2C2)
  {  // NOLINT
    return stm32_i2c_id_t::STM32_I2C2;
  }
#endif
#ifdef I2C3
  else if (hi2c == I2C3)
  {  // NOLINT
    return stm32_i2c_id_t::STM32_I2C3;
  }
#endif
#ifdef I2C4
  else if (hi2c == I2C4)
  {  // NOLINT
    return stm32_i2c_id_t::STM32_I2C4;
  }
#endif
#ifdef I2C5
  else if (hi2c == I2C5)
  {  // NOLINT
    return stm32_i2c_id_t::STM32_I2C5;
  }
#endif
#ifdef I2C6
  else if (hi2c == I2C6)
  {  // NOLINT
    return stm32_i2c_id_t::STM32_I2C6;
  }
#endif
#ifdef I2C7
  else if (hi2c == I2C7)
  {  // NOLINT
    return stm32_i2c_id_t::STM32_I2C7;
  }
#endif
#ifdef I2C8
  else if (hi2c == I2C8)
  {  // NOLINT
    return stm32_i2c_id_t::STM32_I2C8;
  }
#endif
  return stm32_i2c_id_t::STM32_I2C_ID_ERROR;
}

namespace
{

// 统一输入：
// - 7-bit 模式：传 0x00~0x7F（不带 R/W 位），内部左移 1 给 HAL
// - 10-bit 模式：传 0x000~0x3FF（不带 R/W 位），内部直接给 HAL
static inline uint16_t EncodeHalDevAddress(const I2C_HandleTypeDef* hi2c,
                                           uint16_t slave_addr)
{
  ASSERT(hi2c != nullptr);

#if defined(I2C_ADDRESSINGMODE_10BIT)
  if (hi2c->Init.AddressingMode == I2C_ADDRESSINGMODE_10BIT)
  {
    ASSERT(slave_addr <= 0x03FF);
    return static_cast<uint16_t>(slave_addr & 0x03FF);
  }
#endif

  // 默认按 7-bit
  ASSERT(slave_addr <= 0x007F);
  return static_cast<uint16_t>((slave_addr & 0x007F) << 1);
}

static inline ErrorCode MapHalStartFailure(const I2C_HandleTypeDef* hi2c,
                                           HAL_StatusTypeDef st)
{
  // Preserve HAL's immediate start-failure signal without forcing a full controller
  // reset.
#ifdef HAL_I2C_ERROR_NONE
  const uint32_t err = hi2c->ErrorCode;
#ifdef HAL_I2C_WRONG_START
  if ((err & (HAL_I2C_ERROR_TIMEOUT | HAL_I2C_WRONG_START)) != 0U)
#else
  if ((err & HAL_I2C_ERROR_TIMEOUT) != 0U)
#endif
  {
    return ErrorCode::TIMEOUT;
  }
  if (err != HAL_I2C_ERROR_NONE)
  {
    return ErrorCode::FAILED;
  }
#else
  UNUSED(hi2c);
#endif
  return (st == HAL_BUSY) ? ErrorCode::BUSY : ErrorCode::FAILED;
}

struct I2CFilterState
{
  bool analog_filter{false};
  uint32_t digital_filter{0U};
};

static I2CFilterState CaptureI2CFilterState(const I2C_HandleTypeDef* handle);
static void RestoreI2CFilterState(I2C_HandleTypeDef* handle, I2CFilterState state);
static bool ResetI2CPeripheral(I2C_TypeDef* instance);

static void RecoverAfterBlockTimeout(STM32I2C* i2c)
{
  ASSERT(i2c != nullptr);

  auto* hi2c = i2c->i2c_handle_;
  const I2CFilterState filter_state = CaptureI2CFilterState(hi2c);
  i2c->recovering_ = true;
  if (hi2c->hdmarx != nullptr)
  {
    (void)HAL_DMA_Abort(hi2c->hdmarx);
  }
  if (hi2c->hdmatx != nullptr)
  {
    (void)HAL_DMA_Abort(hi2c->hdmatx);
  }

  // Reset only the I2C peripheral so GPIO/NVIC and the selected RCC kernel
  // clock source survive timeout recovery. Fall back to the legacy HAL
  // DeInit path only on families without an RCC peripheral-reset primitive.
  bool initialized = false;
  if (ResetI2CPeripheral(hi2c->Instance))
  {
    initialized = HAL_I2C_Init(hi2c) == HAL_OK;
  }
  else
  {
    (void)HAL_I2C_DeInit(hi2c);
    initialized = HAL_I2C_Init(hi2c) == HAL_OK;
  }
  if (initialized)
  {
    RestoreI2CFilterState(hi2c, filter_state);
  }

  i2c->read_ = false;
  i2c->read_op_ = {};
  i2c->write_op_ = {};
  i2c->read_buff_ = {nullptr, 0};
  i2c->recovering_ = false;
}

static inline ErrorCode WaitBlockResultAndRecoverTimeout(STM32I2C* i2c, uint32_t timeout)
{
  ASSERT(i2c != nullptr);
  const ErrorCode ans = i2c->block_wait_.Wait(timeout);
  if (ans == ErrorCode::TIMEOUT)
  {
    RecoverAfterBlockTimeout(i2c);
  }
  return ans;
}

static bool ResetI2CPeripheral(I2C_TypeDef* instance)
{
#if defined(I2C1) && defined(__HAL_RCC_I2C1_FORCE_RESET) && \
    defined(__HAL_RCC_I2C1_RELEASE_RESET)
  if (instance == I2C1)
  {
    __HAL_RCC_I2C1_FORCE_RESET();
    __NOP();
    __HAL_RCC_I2C1_RELEASE_RESET();
    return true;
  }
#endif

#if defined(I2C2) && defined(__HAL_RCC_I2C2_FORCE_RESET) && \
    defined(__HAL_RCC_I2C2_RELEASE_RESET)
  if (instance == I2C2)
  {
    __HAL_RCC_I2C2_FORCE_RESET();
    __NOP();
    __HAL_RCC_I2C2_RELEASE_RESET();
    return true;
  }
#endif

#if defined(I2C3) && defined(__HAL_RCC_I2C3_FORCE_RESET) && \
    defined(__HAL_RCC_I2C3_RELEASE_RESET)
  if (instance == I2C3)
  {
    __HAL_RCC_I2C3_FORCE_RESET();
    __NOP();
    __HAL_RCC_I2C3_RELEASE_RESET();
    return true;
  }
#endif

#if defined(I2C4) && defined(__HAL_RCC_I2C4_FORCE_RESET) && \
    defined(__HAL_RCC_I2C4_RELEASE_RESET)
  if (instance == I2C4)
  {
    __HAL_RCC_I2C4_FORCE_RESET();
    __NOP();
    __HAL_RCC_I2C4_RELEASE_RESET();
    return true;
  }
#endif

#if defined(I2C5) && defined(__HAL_RCC_I2C5_FORCE_RESET) && \
    defined(__HAL_RCC_I2C5_RELEASE_RESET)
  if (instance == I2C5)
  {
    __HAL_RCC_I2C5_FORCE_RESET();
    __NOP();
    __HAL_RCC_I2C5_RELEASE_RESET();
    return true;
  }
#endif

#if defined(I2C6) && defined(__HAL_RCC_I2C6_FORCE_RESET) && \
    defined(__HAL_RCC_I2C6_RELEASE_RESET)
  if (instance == I2C6)
  {
    __HAL_RCC_I2C6_FORCE_RESET();
    __NOP();
    __HAL_RCC_I2C6_RELEASE_RESET();
    return true;
  }
#endif

#if defined(I2C7) && defined(__HAL_RCC_I2C7_FORCE_RESET) && \
    defined(__HAL_RCC_I2C7_RELEASE_RESET)
  if (instance == I2C7)
  {
    __HAL_RCC_I2C7_FORCE_RESET();
    __NOP();
    __HAL_RCC_I2C7_RELEASE_RESET();
    return true;
  }
#endif

#if defined(I2C8) && defined(__HAL_RCC_I2C8_FORCE_RESET) && \
    defined(__HAL_RCC_I2C8_RELEASE_RESET)
  if (instance == I2C8)
  {
    __HAL_RCC_I2C8_FORCE_RESET();
    __NOP();
    __HAL_RCC_I2C8_RELEASE_RESET();
    return true;
  }
#endif

  return false;
}

/* Resolve a non-zero clock only when the selected source is known. */
#if defined(STM32H7) && defined(HAL_RCC_MODULE_ENABLED)
static uint32_t GetH7HSIClock()
{
#if defined(RCC_CR_HSIRDY)
  if ((RCC->CR & RCC_CR_HSIRDY) == 0U)
  {
    return 0U;
  }
#endif
#if defined(__HAL_RCC_GET_HSI_DIVIDER) && defined(RCC_CR_HSIDIV_Pos)
  return HSI_VALUE >> (__HAL_RCC_GET_HSI_DIVIDER() >> RCC_CR_HSIDIV_Pos);
#else
  return HSI_VALUE;
#endif
}

static uint32_t GetH7CSIClock()
{
#if defined(RCC_CR_CSIRDY)
  return (RCC->CR & RCC_CR_CSIRDY) != 0U ? CSI_VALUE : 0U;
#else
  return 0U;
#endif
}

static uint32_t GetH7Pll3RClock()
{
#if defined(RCC_CR_PLL3RDY)
  if ((RCC->CR & RCC_CR_PLL3RDY) == 0U)
  {
    return 0U;
  }
#endif
#if defined(RCC_PLLCFGR_DIVR3EN)
  if ((RCC->PLLCFGR & RCC_PLLCFGR_DIVR3EN) == 0U)
  {
    return 0U;
  }
#elif defined(RCC_PLLCFGR_PLL3REN)
  if ((RCC->PLLCFGR & RCC_PLLCFGR_PLL3REN) == 0U)
  {
    return 0U;
  }
#else
  return 0U;
#endif
  PLL3_ClocksTypeDef pll3{};
  HAL_RCCEx_GetPLL3ClockFreq(&pll3);
  return pll3.PLL3_R_Frequency;
}

static uint32_t GetH7I2CClock(I2C_TypeDef* instance)
{
#if defined(I2C4) && defined(LL_RCC_I2C4_CLKSOURCE)
  if (instance == I2C4)
  {
    const uint32_t source = LL_RCC_GetI2CClockSource(LL_RCC_I2C4_CLKSOURCE);
#if defined(LL_RCC_I2C4_CLKSOURCE_PCLK4)
    if (source == LL_RCC_I2C4_CLKSOURCE_PCLK4)
    {
      return HAL_RCCEx_GetD3PCLK1Freq();
    }
#endif
#if defined(LL_RCC_I2C4_CLKSOURCE_PLL3R)
    if (source == LL_RCC_I2C4_CLKSOURCE_PLL3R)
    {
      return GetH7Pll3RClock();
    }
#endif
#if defined(LL_RCC_I2C4_CLKSOURCE_HSI)
    if (source == LL_RCC_I2C4_CLKSOURCE_HSI)
    {
      return GetH7HSIClock();
    }
#endif
#if defined(LL_RCC_I2C4_CLKSOURCE_CSI)
    if (source == LL_RCC_I2C4_CLKSOURCE_CSI)
    {
      return GetH7CSIClock();
    }
#endif
    return 0U;
  }
#endif

#if defined(LL_RCC_I2C123_CLKSOURCE)
  const uint32_t source = LL_RCC_GetI2CClockSource(LL_RCC_I2C123_CLKSOURCE);
#if defined(LL_RCC_I2C123_CLKSOURCE_PCLK1)
  if (source == LL_RCC_I2C123_CLKSOURCE_PCLK1)
  {
    return HAL_RCC_GetPCLK1Freq();
  }
#endif
#if defined(LL_RCC_I2C123_CLKSOURCE_PLL3R)
  if (source == LL_RCC_I2C123_CLKSOURCE_PLL3R)
  {
    return GetH7Pll3RClock();
  }
#endif
#if defined(LL_RCC_I2C123_CLKSOURCE_HSI)
  if (source == LL_RCC_I2C123_CLKSOURCE_HSI)
  {
    return GetH7HSIClock();
  }
#endif
#if defined(LL_RCC_I2C123_CLKSOURCE_CSI)
  if (source == LL_RCC_I2C123_CLKSOURCE_CSI)
  {
    return GetH7CSIClock();
  }
#endif
#elif defined(LL_RCC_I2C1235_CLKSOURCE)
  const uint32_t source = LL_RCC_GetI2CClockSource(LL_RCC_I2C1235_CLKSOURCE);
#if defined(LL_RCC_I2C1235_CLKSOURCE_PCLK1)
  if (source == LL_RCC_I2C1235_CLKSOURCE_PCLK1)
  {
    return HAL_RCC_GetPCLK1Freq();
  }
#endif
#if defined(LL_RCC_I2C1235_CLKSOURCE_PLL3R)
  if (source == LL_RCC_I2C1235_CLKSOURCE_PLL3R)
  {
    return GetH7Pll3RClock();
  }
#endif
#if defined(LL_RCC_I2C1235_CLKSOURCE_HSI)
  if (source == LL_RCC_I2C1235_CLKSOURCE_HSI)
  {
    return GetH7HSIClock();
  }
#endif
#if defined(LL_RCC_I2C1235_CLKSOURCE_CSI)
  if (source == LL_RCC_I2C1235_CLKSOURCE_CSI)
  {
    return GetH7CSIClock();
  }
#endif
#endif
  return 0U;
}
#endif

#if defined(STM32F7) && defined(HAL_RCC_MODULE_ENABLED)
static uint32_t GetF7I2CClock(I2C_TypeDef* instance)
{
#define LIBXR_F7_I2C_CLOCK(ID)                                                         \
  if (instance == I2C##ID)                                                             \
  {                                                                                    \
    const uint32_t source = LL_RCC_GetI2CClockSource(LL_RCC_I2C##ID##_CLKSOURCE);      \
    if (source == LL_RCC_I2C##ID##_CLKSOURCE_PCLK1) return HAL_RCC_GetPCLK1Freq();     \
    if (source == LL_RCC_I2C##ID##_CLKSOURCE_SYSCLK) return HAL_RCC_GetSysClockFreq(); \
    if (source == LL_RCC_I2C##ID##_CLKSOURCE_HSI)                                      \
    {                                                                                  \
      return (RCC->CR & RCC_CR_HSIRDY) != 0U ? HSI_VALUE : 0U;                         \
    }                                                                                  \
    return 0U;                                                                         \
  }
#if defined(I2C1)
  LIBXR_F7_I2C_CLOCK(1)
#endif
#if defined(I2C2)
  LIBXR_F7_I2C_CLOCK(2)
#endif
#if defined(I2C3)
  LIBXR_F7_I2C_CLOCK(3)
#endif
#if defined(I2C4)
  LIBXR_F7_I2C_CLOCK(4)
#endif
#undef LIBXR_F7_I2C_CLOCK
  return 0U;
}
#endif

template <typename I2CInstance>
static uint32_t GetI2CClock(I2CInstance* instance)
{
#if defined(STM32H7) && defined(HAL_RCC_MODULE_ENABLED)
  return GetH7I2CClock(instance);
#endif
#if defined(STM32F7) && defined(HAL_RCC_MODULE_ENABLED)
  return GetF7I2CClock(instance);
#endif
#if defined(HAL_RCC_MODULE_ENABLED)
#if defined(RCC_PERIPHCLK_I2C8) && defined(I2C8)
  if (instance == I2C8) return HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_I2C8);
#endif
#if defined(RCC_PERIPHCLK_I2C7) && defined(I2C7)
  if (instance == I2C7) return HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_I2C7);
#endif
#if defined(RCC_PERIPHCLK_I2C6) && defined(I2C6)
  if (instance == I2C6) return HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_I2C6);
#endif
#if defined(RCC_PERIPHCLK_I2C5) && defined(I2C5)
  if (instance == I2C5) return HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_I2C5);
#endif
#if defined(RCC_PERIPHCLK_I2C4) && defined(I2C4)
  if (instance == I2C4) return HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_I2C4);
#endif
#if defined(RCC_PERIPHCLK_I2C1_I3C1) && defined(I2C1)
  if (instance == I2C1) return HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_I2C1_I3C1);
#endif
#if defined(RCC_PERIPHCLK_I2C23)
#if defined(I2C2)
  if (instance == I2C2) return HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_I2C23);
#endif
#if defined(I2C3)
  if (instance == I2C3) return HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_I2C23);
#endif
#endif
#if defined(RCC_PERIPHCLK_I2C3) && defined(I2C3)
  if (instance == I2C3) return HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_I2C3);
#endif
#if defined(RCC_PERIPHCLK_I2C2) && defined(I2C2)
  if (instance == I2C2) return HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_I2C2);
#endif
#if defined(RCC_PERIPHCLK_I2C1) && defined(I2C1)
  if (instance == I2C1) return HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_I2C1);
#endif
#if defined(RCC_PERIPHCLK_I2C1235) && defined(I2C5)
  if (instance == I2C5) return HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_I2C1235);
#endif
#if defined(RCC_PERIPHCLK_I2C123)
#if defined(I2C1)
  if (instance == I2C1) return HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_I2C123);
#endif
#if defined(I2C2)
  if (instance == I2C2) return HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_I2C123);
#endif
#if defined(I2C3)
  if (instance == I2C3) return HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_I2C123);
#endif
#endif
#endif
#if defined(HAL_RCC_MODULE_ENABLED) && defined(STM32C0) && defined(I2C2) && \
    defined(RCC_PERIPHCLK_I2C1) && !defined(RCC_CCIPR_I2C2SEL)
  // C0 aliases I2C2's selector to I2C1's RCC field.
  if (instance == I2C2)
  {
    return HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_I2C1);
  }
#endif
#if defined(HAL_RCC_MODULE_ENABLED) && defined(STM32F0) && defined(I2C2) && \
    !defined(RCC_CFGR3_I2C2SW)
  // F0's I2C2 is fixed to the APB1 clock; only I2C1 has a selector.
  if (instance == I2C2)
  {
    return HAL_RCC_GetPCLK1Freq();
  }
#endif
#if defined(HAL_RCC_MODULE_ENABLED) && defined(STM32G0) && defined(I2C3) && \
    !defined(RCC_CCIPR_I2C3SEL)
  // I2C3 on the G0 parts with this instance is a fixed APB1 peripheral.
  if (instance == I2C3)
  {
    return HAL_RCC_GetPCLK1Freq();
  }
#endif
#if defined(HAL_RCC_MODULE_ENABLED) && defined(STM32U0) && defined(I2C2) && \
    !defined(RCC_CCIPR_I2C2SEL)
  if (instance == I2C2)
  {
    return HAL_RCC_GetPCLK1Freq();
  }
#endif
#if defined(HAL_RCC_MODULE_ENABLED) && defined(STM32U0) && defined(I2C4) && \
    !defined(RCC_CCIPR_I2C4SEL)
  if (instance == I2C4)
  {
    return HAL_RCC_GetPCLK1Freq();
  }
#endif
#if defined(HAL_RCC_MODULE_ENABLED) && defined(STM32WB0) && \
    (defined(I2C1) || defined(I2C2))
  // WB0 has no independent I2C kernel selector; APB1 follows SYSCLK.
#if defined(I2C2)
  if (instance == I2C2)
  {
    return HAL_RCC_GetSysClockFreq();
  }
#endif
#if defined(I2C1)
  if (instance == I2C1)
  {
    return HAL_RCC_GetSysClockFreq();
  }
#endif
#endif
#if defined(HAL_RCC_MODULE_ENABLED) && defined(STM32WL3) && \
    (defined(I2C1) || defined(I2C2))
  // WL3 has no independent I2C kernel selector; APB1 follows SYSCLK.
#if defined(I2C2)
  if (instance == I2C2)
  {
    return HAL_RCC_GetSysClockFreq();
  }
#endif
#if defined(I2C1)
  if (instance == I2C1)
  {
    return HAL_RCC_GetSysClockFreq();
  }
#endif
#endif
  return 0U;
}

static I2CFilterState CaptureI2CFilterState(const I2C_HandleTypeDef* handle)
{
  UNUSED(handle);
  I2CFilterState state;
#if defined(I2C_CR1_ANFOFF)
  state.analog_filter = (handle->Instance->CR1 & I2C_CR1_ANFOFF) == 0U;
#elif defined(I2C_FLTR_ANOFF)
  state.analog_filter = (handle->Instance->FLTR & I2C_FLTR_ANOFF) == 0U;
#endif
#if defined(I2C_CR1_DNF) && defined(I2C_CR1_DNF_Pos)
  state.digital_filter = (handle->Instance->CR1 & I2C_CR1_DNF) >> I2C_CR1_DNF_Pos;
#elif defined(I2C_FLTR_DNF)
  state.digital_filter = handle->Instance->FLTR & I2C_FLTR_DNF;
#endif
  return state;
}

static void RestoreI2CFilterState(I2C_HandleTypeDef* handle, I2CFilterState state)
{
  UNUSED(state);
  const uint32_t was_enabled = handle->Instance->CR1 & I2C_CR1_PE;
  // PE must already be clear before modifying a filter register.
  CLEAR_BIT(handle->Instance->CR1, I2C_CR1_PE);
#if defined(I2C_CR1_ANFOFF)
  MODIFY_REG(handle->Instance->CR1, I2C_CR1_ANFOFF,
             state.analog_filter ? 0U : I2C_CR1_ANFOFF);
#elif defined(I2C_FLTR_ANOFF)
  MODIFY_REG(handle->Instance->FLTR, I2C_FLTR_ANOFF,
             state.analog_filter ? 0U : I2C_FLTR_ANOFF);
#endif
#if defined(I2C_CR1_DNF) && defined(I2C_CR1_DNF_Pos)
  MODIFY_REG(handle->Instance->CR1, I2C_CR1_DNF, state.digital_filter << I2C_CR1_DNF_Pos);
#elif defined(I2C_FLTR_DNF)
  MODIFY_REG(handle->Instance->FLTR, I2C_FLTR_DNF, state.digital_filter);
#endif
  SET_BIT(handle->Instance->CR1, was_enabled);
}

static STM32I2C* FindI2C(I2C_HandleTypeDef* handle)
{
  if (handle == nullptr || handle->Instance == nullptr)
  {
    return nullptr;
  }
  const stm32_i2c_id_t id = STM32_I2C_GetID(handle->Instance);
  if (id == STM32_I2C_ID_ERROR || id >= STM32_I2C_NUMBER)
  {
    return nullptr;
  }
  return STM32I2C::map[id];
}
}  // namespace

STM32I2C::STM32I2C(I2C_HandleTypeDef* hi2c, RawData dma_buff,
                   uint32_t dma_enable_min_size)
    : I2C(),
      id_(hi2c == nullptr ? STM32_I2C_ID_ERROR : STM32_I2C_GetID(hi2c->Instance)),
      i2c_handle_(hi2c),
      dma_enable_min_size_(dma_enable_min_size),
      dma_buff_(dma_buff)
{
  ASSERT(id_ != STM32_I2C_ID_ERROR);
  ASSERT(id_ < STM32_I2C_NUMBER);
  if (id_ == STM32_I2C_ID_ERROR || id_ >= STM32_I2C_NUMBER)
  {
    return;
  }
  map[id_] = this;
}

ErrorCode STM32I2C::Read(uint16_t slave_addr, RawData read_data, ReadOperation& op,
                         bool in_isr)
{
  if (i2c_handle_->State != HAL_I2C_STATE_READY)
  {
    return ErrorCode::BUSY;
  }

  read_ = true;

  const uint16_t dev_addr = EncodeHalDevAddress(i2c_handle_, slave_addr);

  if (read_data.size_ > dma_enable_min_size_)
  {
    read_op_ = op;
    read_buff_ = read_data;
    if (op.type == ReadOperation::OperationType::BLOCK)
    {
      // Arm the BLOCK waiter before HAL exposes completion to IRQ context.
      block_wait_.Start(*op.data.sem_info.sem);
    }
    op.MarkAsRunning();
    const HAL_StatusTypeDef st = HAL_I2C_Master_Receive_DMA(
        i2c_handle_, dev_addr, reinterpret_cast<uint8_t*>(dma_buff_.addr_),
        read_data.size_);
    if (st != HAL_OK)
    {
      if (op.type == ReadOperation::OperationType::BLOCK)
      {
        block_wait_.Cancel();
        return MapHalStartFailure(i2c_handle_, st);
      }
      if (op.type == ReadOperation::OperationType::POLLING)
      {
        op.UpdateStatus(in_isr, ErrorCode::BUSY);
      }
      return ErrorCode::BUSY;
    }
    if (op.type == ReadOperation::OperationType::BLOCK)
    {
      return WaitBlockResultAndRecoverTimeout(this, op.data.sem_info.timeout);
    }
    return ErrorCode::OK;
  }
  else
  {
    auto ans = HAL_I2C_Master_Receive(i2c_handle_, dev_addr,
                                      reinterpret_cast<uint8_t*>(read_data.addr_),
                                      read_data.size_, 20) == HAL_OK
                   ? ErrorCode::OK
                   : ErrorCode::BUSY;
    // BLOCK 模式下沿用统一状态更新路径。
    // Reuse the same status update path for BLOCK mode.
    if (op.type != ReadOperation::OperationType::BLOCK)
    {
      op.UpdateStatus(in_isr, ans);
    }
    return ans;
  }
}

ErrorCode STM32I2C::Write(uint16_t slave_addr, ConstRawData write_data,
                          WriteOperation& op, bool in_isr)
{
  if (i2c_handle_->State != HAL_I2C_STATE_READY)
  {
    return ErrorCode::BUSY;
  }

  read_ = false;

  const uint16_t dev_addr = EncodeHalDevAddress(i2c_handle_, slave_addr);

  Memory::FastCopy(dma_buff_.addr_, write_data.addr_, write_data.size_);

  if (write_data.size_ > dma_enable_min_size_)
  {
    write_op_ = op;
    if (op.type == WriteOperation::OperationType::BLOCK)
    {
      // Arm the BLOCK waiter before HAL exposes completion to IRQ context.
      block_wait_.Start(*op.data.sem_info.sem);
    }
    STM32_CleanDCacheByAddr(dma_buff_.addr_, write_data.size_);
    op.MarkAsRunning();
    const HAL_StatusTypeDef st = HAL_I2C_Master_Transmit_DMA(
        i2c_handle_, dev_addr, reinterpret_cast<uint8_t*>(dma_buff_.addr_),
        write_data.size_);
    if (st != HAL_OK)
    {
      if (op.type == WriteOperation::OperationType::BLOCK)
      {
        block_wait_.Cancel();
        return MapHalStartFailure(i2c_handle_, st);
      }
      if (op.type == WriteOperation::OperationType::POLLING)
      {
        op.UpdateStatus(in_isr, ErrorCode::BUSY);
      }
      return ErrorCode::BUSY;
    }
    if (op.type == WriteOperation::OperationType::BLOCK)
    {
      return WaitBlockResultAndRecoverTimeout(this, op.data.sem_info.timeout);
    }
    return ErrorCode::OK;
  }
  else
  {
    auto ans = HAL_I2C_Master_Transmit(i2c_handle_, dev_addr,
                                       reinterpret_cast<uint8_t*>(dma_buff_.addr_),
                                       write_data.size_, 20) == HAL_OK
                   ? ErrorCode::OK
                   : ErrorCode::BUSY;
    if (op.type != WriteOperation::OperationType::BLOCK)
    {
      op.UpdateStatus(in_isr, ans);
    }
    return ans;
  }
}

ErrorCode STM32I2C::MemRead(uint16_t slave_addr, uint16_t mem_addr, RawData read_data,
                            ReadOperation& op, MemAddrLength mem_addr_size, bool in_isr)
{
  ASSERT(read_data.size_ <= dma_buff_.size_);

  if (i2c_handle_->State != HAL_I2C_STATE_READY)
  {
    return ErrorCode::BUSY;
  }

  read_ = true;

  const uint16_t dev_addr = EncodeHalDevAddress(i2c_handle_, slave_addr);

  if (read_data.size_ > dma_enable_min_size_)
  {
    read_op_ = op;
    read_buff_ = read_data;
    if (op.type == ReadOperation::OperationType::BLOCK)
    {
      // Arm the BLOCK waiter before HAL exposes completion to IRQ context.
      block_wait_.Start(*op.data.sem_info.sem);
    }
    op.MarkAsRunning();
    const HAL_StatusTypeDef st = HAL_I2C_Mem_Read_DMA(
        i2c_handle_, dev_addr, mem_addr,
        mem_addr_size == MemAddrLength::BYTE_8 ? I2C_MEMADD_SIZE_8BIT
                                               : I2C_MEMADD_SIZE_16BIT,
        reinterpret_cast<uint8_t*>(dma_buff_.addr_), read_data.size_);
    if (st != HAL_OK)
    {
      if (op.type == ReadOperation::OperationType::BLOCK)
      {
        block_wait_.Cancel();
        return MapHalStartFailure(i2c_handle_, st);
      }
      if (op.type == ReadOperation::OperationType::POLLING)
      {
        op.UpdateStatus(in_isr, ErrorCode::BUSY);
      }
      return ErrorCode::BUSY;
    }
    if (op.type == ReadOperation::OperationType::BLOCK)
    {
      return WaitBlockResultAndRecoverTimeout(this, op.data.sem_info.timeout);
    }
    return ErrorCode::OK;
  }
  else
  {
    auto ans =
        HAL_I2C_Mem_Read(i2c_handle_, dev_addr, mem_addr,
                         mem_addr_size == MemAddrLength::BYTE_8 ? I2C_MEMADD_SIZE_8BIT
                                                                : I2C_MEMADD_SIZE_16BIT,
                         reinterpret_cast<uint8_t*>(read_data.addr_), read_data.size_,
                         20) == HAL_OK
            ? ErrorCode::OK
            : ErrorCode::BUSY;

    if (op.type != ReadOperation::OperationType::BLOCK)
    {
      op.UpdateStatus(in_isr, ans);
    }
    return ans;
  }
}

ErrorCode STM32I2C::MemWrite(uint16_t slave_addr, uint16_t mem_addr,
                             ConstRawData write_data, WriteOperation& op,
                             MemAddrLength mem_addr_size, bool in_isr)
{
  ASSERT(write_data.size_ <= dma_buff_.size_);

  if (i2c_handle_->State != HAL_I2C_STATE_READY)
  {
    return ErrorCode::BUSY;
  }

  read_ = false;

  const uint16_t dev_addr = EncodeHalDevAddress(i2c_handle_, slave_addr);

  Memory::FastCopy(dma_buff_.addr_, write_data.addr_, write_data.size_);

  if (write_data.size_ > dma_enable_min_size_)
  {
    write_op_ = op;
    if (op.type == WriteOperation::OperationType::BLOCK)
    {
      // Arm the BLOCK waiter before HAL exposes completion to IRQ context.
      block_wait_.Start(*op.data.sem_info.sem);
    }
    STM32_CleanDCacheByAddr(dma_buff_.addr_, write_data.size_);
    op.MarkAsRunning();
    const HAL_StatusTypeDef st = HAL_I2C_Mem_Write_DMA(
        i2c_handle_, dev_addr, mem_addr,
        mem_addr_size == MemAddrLength::BYTE_8 ? I2C_MEMADD_SIZE_8BIT
                                               : I2C_MEMADD_SIZE_16BIT,
        reinterpret_cast<uint8_t*>(dma_buff_.addr_), write_data.size_);
    if (st != HAL_OK)
    {
      if (op.type == WriteOperation::OperationType::BLOCK)
      {
        block_wait_.Cancel();
        return MapHalStartFailure(i2c_handle_, st);
      }
      if (op.type == WriteOperation::OperationType::POLLING)
      {
        op.UpdateStatus(in_isr, ErrorCode::BUSY);
      }
      return ErrorCode::BUSY;
    }
    if (op.type == WriteOperation::OperationType::BLOCK)
    {
      return WaitBlockResultAndRecoverTimeout(this, op.data.sem_info.timeout);
    }
    return ErrorCode::OK;
  }
  else
  {
    auto ans =
        HAL_I2C_Mem_Write(i2c_handle_, dev_addr, mem_addr,
                          mem_addr_size == MemAddrLength::BYTE_8 ? I2C_MEMADD_SIZE_8BIT
                                                                 : I2C_MEMADD_SIZE_16BIT,
                          reinterpret_cast<uint8_t*>(dma_buff_.addr_), write_data.size_,
                          20) == HAL_OK
            ? ErrorCode::OK
            : ErrorCode::BUSY;

    if (op.type != WriteOperation::OperationType::BLOCK)
    {
      op.UpdateStatus(in_isr, ans);
    }
    return ans;
  }
}

ErrorCode STM32I2C::SetConfig(Configuration config)
{
  if (i2c_handle_ == nullptr ||
      STM32_I2C_GetID(i2c_handle_->Instance) == STM32_I2C_ID_ERROR ||
      config.clock_speed == 0U)
  {
    return ErrorCode::ARG_ERR;
  }
  const auto is_busy = [this]()
  {
    return recovering_ || i2c_handle_->State != HAL_I2C_STATE_READY ||
           i2c_handle_->Lock == HAL_LOCKED ||
           __HAL_I2C_GET_FLAG(i2c_handle_, I2C_FLAG_BUSY) != RESET;
  };
  uint32_t timing = 0U;
  const auto old_init = i2c_handle_->Init;
  const I2CFilterState old_filter_state = CaptureI2CFilterState(i2c_handle_);
  if constexpr (HasClockSpeed<decltype(i2c_handle_)>::value)
  {
    if (config.clock_speed > 400000U)
    {
      return ErrorCode::NOT_SUPPORT;
    }
#if defined(I2C_CCR_CCR) && defined(HAL_RCC_MODULE_ENABLED)
    const uint32_t pclk = HAL_RCC_GetPCLK1Freq();
    const uint32_t factor =
        config.clock_speed <= 100000U
            ? 2U
            : (i2c_handle_->Init.DutyCycle == I2C_DUTYCYCLE_16_9 ? 25U : 3U);
    const uint64_t denominator = uint64_t(config.clock_speed) * factor;
    if (pclk == 0U || (uint64_t(pclk) + denominator - 1U) / denominator > I2C_CCR_CCR)
    {
      return ErrorCode::NOT_SUPPORT;
    }
#endif
  }
  else if constexpr (HasTiming<decltype(i2c_handle_)>::value)
  {
    if (!STM32I2CTiming::Compute(GetI2CClock(i2c_handle_->Instance), config.clock_speed,
                                 old_filter_state.analog_filter,
                                 old_filter_state.digital_filter, timing))
    {
      return ErrorCode::NOT_SUPPORT;
    }
  }
  else
  {
    return ErrorCode::NOT_SUPPORT;
  }

  // Report intrinsic configuration errors before transient controller state.
  // Calculation can be preempted, so re-check immediately before reset.
  if (is_busy())
  {
    return ErrorCode::BUSY;
  }
  if constexpr (HasClockSpeed<decltype(i2c_handle_)>::value)
  {
    SetClockSpeed(i2c_handle_, config);
  }
  else
  {
    SetTiming(i2c_handle_, timing);
  }
  if (!ResetI2CPeripheral(i2c_handle_->Instance))
  {
    i2c_handle_->Init = old_init;
    return ErrorCode::NOT_SUPPORT;
  }
  if (HAL_I2C_Init(i2c_handle_) != HAL_OK)
  {
    i2c_handle_->Init = old_init;
    if (ResetI2CPeripheral(i2c_handle_->Instance) && HAL_I2C_Init(i2c_handle_) == HAL_OK)
    {
      RestoreI2CFilterState(i2c_handle_, old_filter_state);
    }
    return ErrorCode::INIT_ERR;
  }
  RestoreI2CFilterState(i2c_handle_, old_filter_state);
  return ErrorCode::OK;
}

extern "C" void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef* hi2c)
{
  STM32I2C* i2c = FindI2C(hi2c);
  if (i2c && !i2c->recovering_ &&
      (i2c->read_op_.type != ReadOperation::OperationType::NONE))
  {
    ErrorCode ec = ErrorCode::OK;
#ifdef HAL_I2C_ERROR_NONE
    ec = (hi2c->ErrorCode == HAL_I2C_ERROR_NONE) ? ErrorCode::OK : ErrorCode::FAILED;
#endif
    STM32_InvalidateDCacheByAddr(i2c->dma_buff_.addr_, i2c->read_buff_.size_);
    if (ec == ErrorCode::OK)
    {
      Memory::FastCopy(i2c->read_buff_.addr_, i2c->dma_buff_.addr_,
                       i2c->read_buff_.size_);
    }
    if (i2c->read_op_.type == ReadOperation::OperationType::BLOCK)
    {
      (void)i2c->block_wait_.TryPost(true, ec);
    }
    else
    {
      i2c->read_op_.UpdateStatus(true, ec);
    }
  }
}

extern "C" void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef* hi2c)
{
  STM32I2C* i2c = FindI2C(hi2c);
  if (i2c && !i2c->recovering_ &&
      (i2c->write_op_.type != WriteOperation::OperationType::NONE))
  {
    ErrorCode ec = ErrorCode::OK;
#ifdef HAL_I2C_ERROR_NONE
    ec = (hi2c->ErrorCode == HAL_I2C_ERROR_NONE) ? ErrorCode::OK : ErrorCode::FAILED;
#endif
    if (i2c->write_op_.type == WriteOperation::OperationType::BLOCK)
    {
      (void)i2c->block_wait_.TryPost(true, ec);
    }
    else
    {
      i2c->write_op_.UpdateStatus(true, ec);
    }
  }
}

extern "C" void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef* hi2c)
{
  STM32I2C* i2c = FindI2C(hi2c);
  if (i2c && !i2c->recovering_ &&
      (i2c->write_op_.type != WriteOperation::OperationType::NONE))
  {
    ErrorCode ec = ErrorCode::OK;
#ifdef HAL_I2C_ERROR_NONE
    ec = (hi2c->ErrorCode == HAL_I2C_ERROR_NONE) ? ErrorCode::OK : ErrorCode::FAILED;
#endif
    if (i2c->write_op_.type == WriteOperation::OperationType::BLOCK)
    {
      (void)i2c->block_wait_.TryPost(true, ec);
    }
    else
    {
      i2c->write_op_.UpdateStatus(true, ec);
    }
  }
}

extern "C" void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef* hi2c)
{
  STM32I2C* i2c = FindI2C(hi2c);
  if (i2c && !i2c->recovering_ &&
      (i2c->read_op_.type != ReadOperation::OperationType::NONE))
  {
    ErrorCode ec = ErrorCode::OK;
#ifdef HAL_I2C_ERROR_NONE
    ec = (hi2c->ErrorCode == HAL_I2C_ERROR_NONE) ? ErrorCode::OK : ErrorCode::FAILED;
#endif
    STM32_InvalidateDCacheByAddr(i2c->dma_buff_.addr_, i2c->read_buff_.size_);
    if (ec == ErrorCode::OK)
    {
      Memory::FastCopy(i2c->read_buff_.addr_, i2c->dma_buff_.addr_,
                       i2c->read_buff_.size_);
    }
    if (i2c->read_op_.type == ReadOperation::OperationType::BLOCK)
    {
      (void)i2c->block_wait_.TryPost(true, ec);
    }
    else
    {
      i2c->read_op_.UpdateStatus(true, ec);
    }
  }
}

extern "C" void HAL_I2C_ErrorCallback(I2C_HandleTypeDef* hi2c)
{
  STM32I2C* i2c = FindI2C(hi2c);

  if (i2c && !i2c->recovering_)
  {
    if (i2c->read_)
    {
      if (i2c->read_op_.type == ReadOperation::OperationType::BLOCK)
      {
        (void)i2c->block_wait_.TryPost(true, ErrorCode::FAILED);
      }
      else
      {
        i2c->read_op_.UpdateStatus(true, ErrorCode::FAILED);
      }
    }
    else
    {
      if (i2c->write_op_.type == WriteOperation::OperationType::BLOCK)
      {
        (void)i2c->block_wait_.TryPost(true, ErrorCode::FAILED);
      }
      else
      {
        i2c->write_op_.UpdateStatus(true, ErrorCode::FAILED);
      }
    }
  }
}

#endif
