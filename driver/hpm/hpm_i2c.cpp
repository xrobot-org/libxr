#include "hpm_i2c.hpp"

#include <cstdint>

using namespace LibXR;

namespace
{

constexpr uint16_t kSharedMax7BitAddress = 0x7FU;
constexpr uint16_t kSharedMax10BitAddress = 0x3FFU;

uint16_t ResolveHpmI2cMaxSlaveAddress(HPMI2C::AddressMode mode)
{
  return mode == HPMI2C::AddressMode::ADDR_10BIT ? kSharedMax10BitAddress
                                                 : kSharedMax7BitAddress;
}

ErrorCode ValidateHpmI2cSlaveAddress(HPMI2C::AddressMode mode, uint16_t slave_addr)
{
  return (slave_addr <= ResolveHpmI2cMaxSlaveAddress(mode)) ? ErrorCode::OK
                                                            : ErrorCode::ARG_ERR;
}

ErrorCode ResolveHpmI2cMemAddressSize(I2C::MemAddrLength len, uint32_t& addr_size)
{
  switch (len)
  {
    case I2C::MemAddrLength::BYTE_8:
      addr_size = 1U;
      return ErrorCode::OK;
    case I2C::MemAddrLength::BYTE_16:
      addr_size = 2U;
      return ErrorCode::OK;
    default:
      addr_size = 0U;
      return ErrorCode::ARG_ERR;
  }
}

void FillHpmI2cMemAddress(uint16_t mem_addr, I2C::MemAddrLength len, uint8_t out[2])
{
  if (len == I2C::MemAddrLength::BYTE_16)
  {
    out[0] = static_cast<uint8_t>((mem_addr >> 8) & 0xFFU);
    out[1] = static_cast<uint8_t>(mem_addr & 0xFFU);
  }
  else
  {
    out[0] = static_cast<uint8_t>(mem_addr & 0xFFU);
  }
}

}  // namespace

uint16_t HPMI2C::GetMaxSlaveAddress(AddressMode mode)
{
  return ResolveHpmI2cMaxSlaveAddress(mode);
}

ErrorCode HPMI2C::ValidateSlaveAddress(uint16_t slave_addr) const
{
  return ValidateHpmI2cSlaveAddress(address_mode_, slave_addr);
}

ErrorCode HPMI2C::ResolveMemAddressSize(MemAddrLength len, uint32_t& addr_size)
{
  return ResolveHpmI2cMemAddressSize(len, addr_size);
}

void HPMI2C::FillMemAddress(uint16_t mem_addr, MemAddrLength len, uint8_t out[2])
{
  FillHpmI2cMemAddress(mem_addr, len, out);
}

ErrorCode HPMI2C::SetWaitPolicy(WaitPolicy policy)
{
  if (policy.addr_hit_timeout_us == 0U || policy.stop_timeout_us == 0U ||
      policy.bus_idle_timeout_us == 0U || policy.transfer_timeout_us == 0U)
  {
    return ErrorCode::ARG_ERR;
  }

#if LIBXR_HPM_I2C_HAS_DMA_MGR
  if (AsyncTransferActive())
  {
    return ErrorCode::BUSY;
  }
#endif

  wait_policy_ = policy;
  return ErrorCode::OK;
}

#if LIBXR_HPM_I2C_SUPPORTED

#if __has_include("board.h")
extern "C"
{
#include "board.h"
  void board_i2c_bus_clear(I2C_Type* ptr);
}
#define LIBXR_HPM_I2C_HAS_BOARD_HELPER 1
#else
#define LIBXR_HPM_I2C_HAS_BOARD_HELPER 0
#endif

#if __has_include("hpm_interrupt.h")
#include "hpm_interrupt.h"
#define LIBXR_HPM_I2C_HAS_INTERRUPT 1
#else
#define LIBXR_HPM_I2C_HAS_INTERRUPT 0
#endif

#if __has_include("hpm_l1c_drv.h")
#include "hpm_l1c_drv.h"
#define LIBXR_HPM_I2C_HAS_L1C 1
#else
#define LIBXR_HPM_I2C_HAS_L1C 0
#endif

#if defined(__GNUC__) || defined(__clang__)
/**
 * @brief 可选等待放松钩子，工程可提供强定义以在 I2C 忙等路径中让出 CPU /
 * Optional wait relaxation hook; projects may provide a strong definition to yield
 * CPU time inside I2C busy-wait paths.
 */
extern "C" void libxr_hpm_i2c_wait_relax_hook(void) __attribute__((weak));
#define LIBXR_HPM_I2C_HAS_WAIT_RELAX_HOOK 1
#else
#define LIBXR_HPM_I2C_HAS_WAIT_RELAX_HOOK 0
#endif

#if LIBXR_HPM_I2C_HAS_DMA_MGR
#include "hpm_i2c_platform.hpp"
#endif

namespace
{

// Core blocking flags and timeout policy shared by polling and async setup paths.
constexpr uint16_t kI2CFlagRead = I2C_RD;
constexpr uint16_t kI2CFlagAddr10Bit = I2C_ADDR_10BIT;
constexpr uint16_t kI2CFlagNoStart = I2C_NO_START;
constexpr uint16_t kI2CFlagNoAddress = I2C_NO_ADDRESS;
constexpr uint16_t kI2CFlagNoStop = I2C_NO_STOP;
constexpr uint16_t kI2CFlagWriteCheckAck = I2C_WRITE_CHECK_ACK;

// DMA manager state and ISR instance slots stay in this TU to avoid build-system
// churn; platform resource resolution lives in hpm_i2c_platform.hpp.
#if LIBXR_HPM_I2C_HAS_DMA_MGR
static_assert(sizeof(uintptr_t) <= sizeof(uint32_t),
              "HPM I2C DMA helper assumes a 32-bit address space.");

uint32_t ToHpmI2cDmaAddress(const volatile void* addr)
{
  return static_cast<uint32_t>(reinterpret_cast<uintptr_t>(addr));
}

#if defined(HPM_I2C7)
constexpr size_t kHpmI2cInstanceCount = 8U;
#elif defined(HPM_I2C6)
constexpr size_t kHpmI2cInstanceCount = 7U;
#elif defined(HPM_I2C5)
constexpr size_t kHpmI2cInstanceCount = 6U;
#elif defined(HPM_I2C4)
constexpr size_t kHpmI2cInstanceCount = 5U;
#else
constexpr size_t kHpmI2cInstanceCount = 4U;
#endif
HPMI2C* g_hpm_i2c_instance_map[kHpmI2cInstanceCount] = {};

using HPMI2CPlatform::kInvalidDmaSource;
using HPMI2CPlatform::ResolveBoardI2cDmaSource;
using HPMI2CPlatform::ResolveBoardI2cIrq;
using HPMI2CPlatform::ResolveI2cIndex;
#endif

#if LIBXR_HPM_I2C_HAS_DMA_MGR
#if LIBXR_HPM_I2C_HAS_L1C
static bool ResolveDCacheRange(const void* addr, uint32_t size, uint32_t& start,
                               uint32_t& aligned_size)
{
  if (addr == nullptr || size == 0U)
  {
    start = 0U;
    aligned_size = 0U;
    return false;
  }

  const uint64_t line_size = HPM_L1C_CACHELINE_SIZE;
  const uint64_t address = ToHpmI2cDmaAddress(addr);
  const uint64_t end = address + static_cast<uint64_t>(size);
  constexpr uint64_t kAddressSpaceSize = static_cast<uint64_t>(UINT32_MAX) + 1ULL;
  if (end > kAddressSpaceSize)
  {
    start = 0U;
    aligned_size = 0U;
    return false;
  }

  const uint64_t aligned_start = address - (address % line_size);
  const uint64_t aligned_end = ((end + line_size - 1U) / line_size) * line_size;
  const uint64_t range_size = aligned_end - aligned_start;
  if (aligned_end > kAddressSpaceSize || range_size > UINT32_MAX)
  {
    start = 0U;
    aligned_size = 0U;
    return false;
  }

  start = static_cast<uint32_t>(aligned_start);
  aligned_size = static_cast<uint32_t>(range_size);
  return aligned_size > 0U;
}
#endif

static void FlushDCacheIfNeeded(const void* addr, uint32_t size)
{
#if LIBXR_HPM_I2C_HAS_L1C
  if (addr != nullptr && size > 0U && l1c_dc_is_enabled())
  {
    uint32_t start = 0U;
    uint32_t aligned_size = 0U;
    if (ResolveDCacheRange(addr, size, start, aligned_size))
    {
      l1c_dc_flush(start, aligned_size);
    }
  }
#else
  UNUSED(addr);
  UNUSED(size);
#endif
}
#endif

#if LIBXR_HPM_I2C_HAS_DMA_MGR
static void InvalidateDCacheIfNeeded(const void* addr, uint32_t size)
{
#if LIBXR_HPM_I2C_HAS_L1C
  if (addr != nullptr && size > 0U && l1c_dc_is_enabled())
  {
    uint32_t start = 0U;
    uint32_t aligned_size = 0U;
    if (ResolveDCacheRange(addr, size, start, aligned_size))
    {
      l1c_dc_invalidate(start, aligned_size);
    }
  }
#else
  UNUSED(addr);
  UNUSED(size);
#endif
}
#endif

void I2cWaitRelax()
{
#if LIBXR_HPM_I2C_HAS_WAIT_RELAX_HOOK
  if (libxr_hpm_i2c_wait_relax_hook != nullptr)
  {
    libxr_hpm_i2c_wait_relax_hook();
    return;
  }
#endif
  __asm volatile("nop");
}

uint32_t GetI2cWaitTicksPerUs()
{
  static uint32_t ticks_per_us = 0U;
  if (ticks_per_us == 0U)
  {
    ticks_per_us = clock_get_core_clock_ticks_per_us();
  }
  return ticks_per_us;
}

uint64_t GetI2cWaitTimeoutTicks(uint64_t timeout_us)
{
  const uint64_t ticks_per_us = GetI2cWaitTicksPerUs();
  if (ticks_per_us != 0U && timeout_us > (UINT64_MAX / ticks_per_us))
  {
    return UINT64_MAX;
  }
  return ticks_per_us * timeout_us;
}

uint64_t GetI2cWaitCycle() { return hpm_csr_get_core_cycle(); }

template <typename Predicate>
bool WaitUntil(const Predicate& predicate, uint64_t timeout_us)
{
  const uint64_t start = GetI2cWaitCycle();
  const uint64_t timeout_ticks = GetI2cWaitTimeoutTicks(timeout_us);
  while (!predicate())
  {
    if ((GetI2cWaitCycle() - start) > timeout_ticks)
    {
      return false;
    }
    I2cWaitRelax();
  }
  return true;
}

bool WaitForBusIdle(I2C_Type* i2c, uint64_t timeout_us)
{
  return WaitUntil([i2c]()
                   { return (i2c_get_status(i2c) & I2C_STATUS_BUSBUSY_MASK) == 0U; },
                   timeout_us);
}

void IssueStopAndWait(I2C_Type* i2c, const HPMI2C::WaitPolicy& policy)
{
  if (i2c == nullptr)
  {
    return;
  }

  i2c_clear_status(i2c, I2C_STATUS_CMPL_MASK);
  i2c->CTRL = I2C_CTRL_PHASE_STOP_MASK;
  i2c_master_issue_data_transmission(i2c);
  (void)WaitUntil(
      [i2c]()
      {
        const uint32_t status = i2c_get_status(i2c);
        return ((status & I2C_STATUS_CMPL_MASK) != 0U) ||
               ((status & I2C_STATUS_BUSBUSY_MASK) == 0U);
      },
      policy.stop_timeout_us);
  i2c_clear_status(
      i2c, I2C_STATUS_CMPL_MASK | I2C_STATUS_ADDRHIT_MASK | I2C_STATUS_BYTETRANS_MASK);
  i2c_clear_fifo(i2c);
}

hpm_stat_t DoManualTransferWithFlagsImpl(I2C_Type* i2c, uint16_t slave_addr, RawData data,
                                         uint16_t flags, const HPMI2C::WaitPolicy& policy)
{
  if (i2c == nullptr || data.addr_ == nullptr || data.size_ == 0U)
  {
    return status_invalid_argument;
  }

  i2c_enable_10bit_address_mode(i2c, (flags & I2C_ADDR_10BIT) != 0U);
  i2c_clear_status(
      i2c, I2C_STATUS_CMPL_MASK | I2C_STATUS_ADDRHIT_MASK | I2C_STATUS_BYTETRANS_MASK);
  i2c_clear_fifo(i2c);
  i2c_master_set_slave_address(i2c, slave_addr);

  if ((flags & I2C_RD) != 0U)
  {
    i2c_set_direction(i2c, I2C_DIR_MASTER_READ);
  }
  else
  {
    i2c_set_direction(i2c, I2C_DIR_MASTER_WRITE);
  }

  if ((flags & I2C_NO_START) != 0U)
  {
    i2c_master_disable_start_phase(i2c);
  }
  else
  {
    i2c_master_enable_start_phase(i2c);
  }

  if ((flags & I2C_NO_ADDRESS) != 0U)
  {
    i2c_master_disable_addr_phase(i2c);
  }
  else
  {
    i2c_master_enable_addr_phase(i2c);
  }

  if ((flags & I2C_NO_STOP) != 0U)
  {
    i2c_master_disable_stop_phase(i2c);
  }
  else
  {
    i2c_master_enable_stop_phase(i2c);
  }

  i2c_master_enable_data_phase(i2c);
  i2c_set_data_count(i2c, static_cast<uint32_t>(data.size_));
  const uint32_t saved_inten = i2c->INTEN;
  i2c->INTEN = saved_inten | I2C_EVENT_BYTE_RECEIVED;
  i2c_master_issue_data_transmission(i2c);

  hpm_stat_t raw_status = status_success;
  auto* bytes = static_cast<uint8_t*>(data.addr_);
  uint32_t left = static_cast<uint32_t>(data.size_);

  if ((flags & I2C_NO_ADDRESS) == 0U)
  {
    const bool addr_hit = WaitUntil(
        [i2c]() { return (i2c_get_status(i2c) & I2C_STATUS_ADDRHIT_MASK) != 0U; },
        policy.addr_hit_timeout_us);
    if (!addr_hit)
    {
      raw_status = status_i2c_no_addr_hit;
    }
    else
    {
      i2c_clear_status(i2c, I2C_STATUS_ADDRHIT_MASK);
    }
  }

  if (raw_status == status_success && (flags & I2C_RD) != 0U)
  {
    while (left > 0U)
    {
      const bool ready = WaitUntil(
          [i2c]() { return (i2c_get_status(i2c) & I2C_STATUS_FIFOEMPTY_MASK) == 0U; },
          policy.transfer_timeout_us);
      if (!ready)
      {
        raw_status = status_timeout;
        break;
      }

      *(bytes++) = i2c_read_byte(i2c);
      left--;
      if (left == 0U)
      {
        i2c_respond_Nack(i2c);
      }
      else if ((flags & I2C_NO_READ_ACK) == 0U)
      {
        i2c_respond_ack(i2c);
      }
    }
  }
  else if (raw_status == status_success)
  {
    while (left > 0U)
    {
      if ((flags & I2C_WRITE_CHECK_ACK) != 0U)
      {
        i2c_write_byte(i2c, *(bytes++));
        left--;

        const bool ready = WaitUntil(
            [i2c]() { return (i2c_get_status(i2c) & I2C_STATUS_BYTETRANS_MASK) != 0U; },
            policy.transfer_timeout_us);
        if (!ready)
        {
          raw_status = status_timeout;
          break;
        }

        const uint32_t status = i2c_get_status(i2c);
        i2c_clear_status(i2c, I2C_STATUS_BYTETRANS_MASK);
        if (!I2C_STATUS_ACK_GET(status))
        {
          raw_status = status_i2c_no_ack;
          break;
        }
      }
      else
      {
        const bool ready = WaitUntil(
            [i2c]() { return (i2c_get_status(i2c) & I2C_STATUS_FIFOFULL_MASK) == 0U; },
            policy.transfer_timeout_us);
        if (!ready)
        {
          raw_status = status_timeout;
          break;
        }

        i2c_write_byte(i2c, *(bytes++));
        left--;
      }
    }
  }

  if (raw_status == status_success)
  {
    const bool complete =
        WaitUntil([i2c]() { return (i2c_get_status(i2c) & I2C_STATUS_CMPL_MASK) != 0U; },
                  policy.transfer_timeout_us);
    if (!complete)
    {
      raw_status = status_timeout;
    }
    else
    {
      i2c_clear_status(i2c, I2C_STATUS_CMPL_MASK);
      if (i2c_get_data_count(i2c) != 0U)
      {
        raw_status = status_i2c_transmit_not_completed;
      }
    }
  }

  if (raw_status != status_success)
  {
    IssueStopAndWait(i2c, policy);
    i2c_clear_status(
        i2c, I2C_STATUS_CMPL_MASK | I2C_STATUS_ADDRHIT_MASK | I2C_STATUS_BYTETRANS_MASK);
    i2c_master_disable_data_phase(i2c);
    i2c_master_disable_addr_phase(i2c);
    i2c_master_disable_start_phase(i2c);
    i2c_master_disable_stop_phase(i2c);
  }

  i2c->INTEN = saved_inten;
  return raw_status;
}

}  // namespace

#if LIBXR_HPM_I2C_HAS_DMA_MGR && LIBXR_HPM_I2C_HAS_INTERRUPT
extern "C" void libxr_hpm_i2c_process_interrupt(I2C_Type* ptr)
{
  const int32_t index = ResolveI2cIndex(ptr);
  if (index >= 0 && static_cast<size_t>(index) < kHpmI2cInstanceCount &&
      g_hpm_i2c_instance_map[index] != nullptr)
  {
    g_hpm_i2c_instance_map[index]->HandleAsyncInterrupt(true);
  }
}

#if defined(IRQn_I2C0) && defined(HPM_I2C0)
SDK_DECLARE_EXT_ISR_M(IRQn_I2C0, libxr_hpm_i2c0_isr)
void libxr_hpm_i2c0_isr(void) { libxr_hpm_i2c_process_interrupt(HPM_I2C0); }
#endif
#if defined(IRQn_I2C1) && defined(HPM_I2C1)
SDK_DECLARE_EXT_ISR_M(IRQn_I2C1, libxr_hpm_i2c1_isr)
void libxr_hpm_i2c1_isr(void) { libxr_hpm_i2c_process_interrupt(HPM_I2C1); }
#endif
#if defined(IRQn_I2C2) && defined(HPM_I2C2)
SDK_DECLARE_EXT_ISR_M(IRQn_I2C2, libxr_hpm_i2c2_isr)
void libxr_hpm_i2c2_isr(void) { libxr_hpm_i2c_process_interrupt(HPM_I2C2); }
#endif
#if defined(IRQn_I2C3) && defined(HPM_I2C3)
SDK_DECLARE_EXT_ISR_M(IRQn_I2C3, libxr_hpm_i2c3_isr)
void libxr_hpm_i2c3_isr(void) { libxr_hpm_i2c_process_interrupt(HPM_I2C3); }
#endif
#if defined(IRQn_I2C4) && defined(HPM_I2C4)
SDK_DECLARE_EXT_ISR_M(IRQn_I2C4, libxr_hpm_i2c4_isr)
void libxr_hpm_i2c4_isr(void) { libxr_hpm_i2c_process_interrupt(HPM_I2C4); }
#endif
#if defined(IRQn_I2C5) && defined(HPM_I2C5)
SDK_DECLARE_EXT_ISR_M(IRQn_I2C5, libxr_hpm_i2c5_isr)
void libxr_hpm_i2c5_isr(void) { libxr_hpm_i2c_process_interrupt(HPM_I2C5); }
#endif
#if defined(IRQn_I2C6) && defined(HPM_I2C6)
SDK_DECLARE_EXT_ISR_M(IRQn_I2C6, libxr_hpm_i2c6_isr)
void libxr_hpm_i2c6_isr(void) { libxr_hpm_i2c_process_interrupt(HPM_I2C6); }
#endif
#if defined(IRQn_I2C7) && defined(HPM_I2C7)
SDK_DECLARE_EXT_ISR_M(IRQn_I2C7, libxr_hpm_i2c7_isr)
void libxr_hpm_i2c7_isr(void) { libxr_hpm_i2c_process_interrupt(HPM_I2C7); }
#endif
#else
extern "C" void libxr_hpm_i2c_process_interrupt(I2C_Type* ptr) { UNUSED(ptr); }
#endif

HPMI2C::HPMI2C(I2C_Type* i2c, clock_name_t clock, bool auto_board_init,
               I2C::Configuration config)
    : i2c_(i2c), clock_(clock), current_config_(config), auto_board_init_(auto_board_init)
{
  ASSERT(i2c_ != nullptr);

#if LIBXR_HPM_I2C_HAS_BOARD_HELPER
  if (auto_board_init_)
  {
    source_clock_hz_ = board_init_i2c_clock(i2c_);
    init_i2c_pins(i2c_);
    TryRecoverBusLines();
  }
#else
  (void)auto_board_init_;
#endif

  if (source_clock_hz_ == 0)
  {
    clock_add_to_group(clock_, 0);
    source_clock_hz_ = clock_get_frequency(clock_);
  }

  ASSERT(source_clock_hz_ != 0);
  const ErrorCode ans = SetConfig(config);
  REQUIRE(ans == ErrorCode::OK);
}

ErrorCode HPMI2C::SetAddressMode(AddressMode mode)
{
  if (i2c_ == nullptr)
  {
    return ErrorCode::PTR_NULL;
  }

  if (address_mode_ == mode && configured_)
  {
    return ErrorCode::OK;
  }

#if LIBXR_HPM_I2C_HAS_DMA_MGR
  if (AsyncTransferActive())
  {
    return ErrorCode::BUSY;
  }
  DisableAsyncI2cIrq();
  StopAsyncDma();
#endif
  IssueStopAndWait(i2c_, wait_policy_);
  (void)WaitForBusIdle(i2c_, wait_policy_.bus_idle_timeout_us);

  const AddressMode previous_mode = address_mode_;
  address_mode_ = mode;
  const ErrorCode ans = ApplyConfig(current_config_);
  (void)WaitForBusIdle(i2c_, wait_policy_.bus_idle_timeout_us);
  if (ans != ErrorCode::OK)
  {
    address_mode_ = previous_mode;
    if (configured_)
    {
      (void)ApplyConfig(current_config_);
    }
  }
  return ans;
}

ErrorCode HPMI2C::ConvertStatus(hpm_stat_t status)
{
  switch (status)
  {
    case status_success:
      return ErrorCode::OK;
    case status_timeout:
      return ErrorCode::TIMEOUT;
    case status_invalid_argument:
      return ErrorCode::ARG_ERR;
    case status_i2c_bus_busy:
      return ErrorCode::BUSY;
    case status_i2c_not_supported:
      return ErrorCode::NOT_SUPPORT;
    case status_i2c_no_ack:
    case status_i2c_no_addr_hit:
      return ErrorCode::NO_RESPONSE;
    case status_i2c_invalid_data:
      return ErrorCode::CHECK_ERR;
    case status_i2c_transmit_not_completed:
      return ErrorCode::FAILED;
    default:
      return ErrorCode::FAILED;
  }
}

ErrorCode HPMI2C::ResolveMode(uint32_t clock_speed, i2c_mode_t& mode)
{
  if (clock_speed == 100000UL)
  {
    mode = i2c_mode_normal;
  }
  else if (clock_speed == 400000UL)
  {
    mode = i2c_mode_fast;
  }
  else if (clock_speed == 1000000UL)
  {
    mode = i2c_mode_fast_plus;
  }
  else if (clock_speed == 0)
  {
    return ErrorCode::ARG_ERR;
  }
  else
  {
    return ErrorCode::NOT_SUPPORT;
  }

  return ErrorCode::OK;
}

#if LIBXR_HPM_I2C_HAS_DMA_MGR
ErrorCode HPMI2C::ConvertDmaStatus(hpm_stat_t status)
{
  switch (status)
  {
    case status_success:
      return ErrorCode::OK;
    case status_invalid_argument:
      return ErrorCode::ARG_ERR;
    case status_dma_mgr_no_resource:
      return ErrorCode::FULL;
    default:
      return ErrorCode::FAILED;
  }
}
#endif

i2c_seq_transfer_opt_t HPMI2C::ConvertSequenceFrame(SequenceFrame frame)
{
  switch (frame)
  {
    case SequenceFrame::FIRST:
      return i2c_frist_frame;
    case SequenceFrame::NEXT:
      return i2c_next_frame;
    case SequenceFrame::LAST:
    default:
      return i2c_last_frame;
  }
}

uint16_t HPMI2C::BuildTransferFlags(uint16_t flags) const
{
  return (address_mode_ == AddressMode::ADDR_10BIT) ? (flags | kI2CFlagAddr10Bit) : flags;
}

ErrorCode HPMI2C::EnsureClockReady()
{
  if (i2c_ == nullptr)
  {
    return ErrorCode::PTR_NULL;
  }

  // Runtime reconfiguration must use the current clock, not the constructor's cache.
  clock_add_to_group(clock_, 0);
  source_clock_hz_ = clock_get_frequency(clock_);
  if (source_clock_hz_ == 0)
  {
    return ErrorCode::INIT_ERR;
  }

  return ErrorCode::OK;
}

ErrorCode HPMI2C::EnsureControllerReady()
{
  if (i2c_ == nullptr)
  {
    return ErrorCode::PTR_NULL;
  }

  if (configured_)
  {
    return ErrorCode::OK;
  }

  return ApplyConfig(current_config_);
}

ErrorCode HPMI2C::ApplyConfig(const Configuration& config)
{
  if (i2c_ == nullptr)
  {
    return ErrorCode::PTR_NULL;
  }

  i2c_mode_t mode = i2c_mode_normal;
  ErrorCode ans = ResolveMode(config.clock_speed, mode);
  if (ans != ErrorCode::OK)
  {
    return ans;
  }

  ans = EnsureClockReady();
  if (ans != ErrorCode::OK)
  {
    return ans;
  }

  i2c_config_t i2c_config{};
  i2c_config.i2c_mode = mode;
  i2c_config.is_10bit_addressing = (address_mode_ == AddressMode::ADDR_10BIT);

  const uint32_t previous_setup = i2c_->SETUP;
  const uint32_t previous_tpm = i2c_->TPM;
  const uint32_t previous_interrupts = i2c_->INTEN;
  ans = ConvertStatus(i2c_init_master(i2c_, source_clock_hz_, &i2c_config));
  if (ans == ErrorCode::OK)
  {
    current_config_ = config;
    configured_ = true;
  }
  else if (configured_)
  {
    // The SDK resets the controller before validating its timing configuration.
    // Restore the previous idle configuration without reissuing a transaction.
    i2c_->TPM = previous_tpm;
    i2c_->SETUP = previous_setup;
    i2c_->INTEN = previous_interrupts;
  }

  return ans;
}

bool HPMI2C::ShouldRecover(hpm_stat_t status)
{
  switch (status)
  {
    case status_timeout:
    case status_i2c_bus_busy:
    case status_i2c_no_ack:
    case status_i2c_no_addr_hit:
      return true;
    default:
      return false;
  }
}

void HPMI2C::RecoverController()
{
  if (i2c_ == nullptr)
  {
    return;
  }

  IssueStopAndWait(i2c_, wait_policy_);
  (void)WaitForBusIdle(i2c_, wait_policy_.bus_idle_timeout_us);
  TryRecoverBusLines();
  i2c_reset(i2c_);
  if (configured_)
  {
    (void)ApplyConfig(current_config_);
  }
  (void)WaitForBusIdle(i2c_, wait_policy_.bus_idle_timeout_us);
}

void HPMI2C::TryRecoverBusLines()
{
  if (i2c_ == nullptr)
  {
    return;
  }

#if LIBXR_HPM_I2C_HAS_BOARD_HELPER
  if (auto_board_init_)
  {
    if (i2c_get_line_scl_status(i2c_) && !i2c_get_line_sda_status(i2c_))
    {
      board_i2c_bus_clear(i2c_);
    }
    return;
  }
#endif

#if defined(HPM_IP_FEATURE_I2C_SUPPORT_RESET) && (HPM_IP_FEATURE_I2C_SUPPORT_RESET == 1)
  if (i2c_get_line_scl_status(i2c_) && !i2c_get_line_sda_status(i2c_))
  {
    i2c_gen_reset_signal(i2c_, 9);
  }
#endif
}

#if LIBXR_HPM_I2C_HAS_DMA_MGR
uint32_t HPMI2C::ToSystemAddress(const void* addr)
{
  return core_local_mem_to_sys_address(HPM_CORE0, ToHpmI2cDmaAddress(addr));
}

ErrorCode HPMI2C::PrepareAsyncTransfer(uint16_t slave_addr, uint16_t flags, uint32_t size,
                                       bool clear_fifo, bool require_bus_idle)
{
  if (i2c_ == nullptr)
  {
    return ErrorCode::PTR_NULL;
  }

  if (require_bus_idle)
  {
    if (!WaitForBusIdle(i2c_, wait_policy_.transfer_timeout_us))
    {
      return ErrorCode::BUSY;
    }
  }

  i2c_clear_status(i2c_, I2C_STATUS_CMPL_MASK);
  if (clear_fifo)
  {
    i2c_clear_fifo(i2c_);
  }

  const uint16_t final_flags = BuildTransferFlags(flags);
  i2c_master_set_slave_address(i2c_, slave_addr);
  if ((final_flags & I2C_RD) != 0U)
  {
    i2c_set_direction(i2c_, I2C_DIR_MASTER_READ);
  }
  else
  {
    i2c_set_direction(i2c_, I2C_DIR_MASTER_WRITE);
  }

  if ((final_flags & I2C_NO_START) != 0U)
  {
    i2c_master_disable_start_phase(i2c_);
  }
  else
  {
    i2c_master_enable_start_phase(i2c_);
  }

  if ((final_flags & I2C_NO_ADDRESS) != 0U)
  {
    i2c_master_disable_addr_phase(i2c_);
  }
  else
  {
    i2c_master_enable_addr_phase(i2c_);
  }

  if ((final_flags & I2C_NO_STOP) != 0U)
  {
    i2c_master_disable_stop_phase(i2c_);
  }
  else
  {
    i2c_master_enable_stop_phase(i2c_);
  }

  if (size > 0U)
  {
    i2c_master_enable_data_phase(i2c_);
    i2c_set_data_count(i2c_, size);
  }
  else
  {
    i2c_master_disable_data_phase(i2c_);
  }

  i2c_dma_disable(i2c_);
  i2c_master_issue_data_transmission(i2c_);

  if ((final_flags & I2C_NO_ADDRESS) == 0U)
  {
    const bool addr_hit = WaitUntil(
        [this]() { return (i2c_get_status(i2c_) & I2C_STATUS_ADDRHIT_MASK) != 0U; },
        wait_policy_.addr_hit_timeout_us);
    if (!addr_hit)
    {
      StopAndReleaseAsyncBus();
      return ErrorCode::NO_RESPONSE;
    }
    i2c_clear_status(i2c_, I2C_STATUS_ADDRHIT_MASK);
  }

  return ErrorCode::OK;
}

ErrorCode HPMI2C::StartAsyncReadDma(void* dst, uint32_t size)
{
  if (i2c_ == nullptr || dst == nullptr || size == 0U)
  {
    return ErrorCode::ARG_ERR;
  }
  ErrorCode ans = EnsureAsyncDmaReady();
  if (ans != ErrorCode::OK)
  {
    return ans;
  }

  FlushDCacheIfNeeded(dst, size);

  hpm_stat_t status =
      dma_mgr_set_chn_dst_addr(&async_dma_resource_, ToSystemAddress(dst));
  if (status != status_success)
  {
    return ConvertDmaStatus(status);
  }
  status =
      dma_mgr_set_chn_dst_work_mode(&async_dma_resource_, DMA_MGR_HANDSHAKE_MODE_NORMAL);
  if (status != status_success)
  {
    return ConvertDmaStatus(status);
  }
  status = dma_mgr_set_chn_dst_addr_ctrl(&async_dma_resource_,
                                         DMA_MGR_ADDRESS_CONTROL_INCREMENT);
  if (status != status_success)
  {
    return ConvertDmaStatus(status);
  }
  status =
      dma_mgr_set_chn_src_addr(&async_dma_resource_, ToHpmI2cDmaAddress(&i2c_->DATA));
  if (status != status_success)
  {
    return ConvertDmaStatus(status);
  }
  status = dma_mgr_set_chn_src_work_mode(&async_dma_resource_,
                                         DMA_MGR_HANDSHAKE_MODE_HANDSHAKE);
  if (status != status_success)
  {
    return ConvertDmaStatus(status);
  }
  status =
      dma_mgr_set_chn_src_addr_ctrl(&async_dma_resource_, DMA_MGR_ADDRESS_CONTROL_FIXED);
  if (status != status_success)
  {
    return ConvertDmaStatus(status);
  }
  status = dma_mgr_set_chn_src_width(&async_dma_resource_, DMA_MGR_TRANSFER_WIDTH_BYTE);
  if (status != status_success)
  {
    return ConvertDmaStatus(status);
  }
  status = dma_mgr_set_chn_dst_width(&async_dma_resource_, DMA_MGR_TRANSFER_WIDTH_BYTE);
  if (status != status_success)
  {
    return ConvertDmaStatus(status);
  }
  status = dma_mgr_set_chn_transize(&async_dma_resource_, size);
  if (status != status_success)
  {
    return ConvertDmaStatus(status);
  }

  i2c_dma_enable(i2c_);
  dma_clear_transfer_status(async_dma_resource_.base, async_dma_resource_.channel);
  status = dma_mgr_enable_channel(&async_dma_resource_);
  if (status != status_success)
  {
    i2c_dma_disable(i2c_);
    return ConvertDmaStatus(status);
  }
  return ErrorCode::OK;
}

ErrorCode HPMI2C::StartAsyncWriteDma(const void* src, uint32_t size)
{
  if (i2c_ == nullptr || src == nullptr || size == 0U)
  {
    return ErrorCode::ARG_ERR;
  }
  ErrorCode ans = EnsureAsyncDmaReady();
  if (ans != ErrorCode::OK)
  {
    return ans;
  }

  FlushDCacheIfNeeded(src, size);

  hpm_stat_t status =
      dma_mgr_set_chn_src_addr(&async_dma_resource_, ToSystemAddress(src));
  if (status != status_success)
  {
    return ConvertDmaStatus(status);
  }
  status =
      dma_mgr_set_chn_src_work_mode(&async_dma_resource_, DMA_MGR_HANDSHAKE_MODE_NORMAL);
  if (status != status_success)
  {
    return ConvertDmaStatus(status);
  }
  status = dma_mgr_set_chn_src_addr_ctrl(&async_dma_resource_,
                                         DMA_MGR_ADDRESS_CONTROL_INCREMENT);
  if (status != status_success)
  {
    return ConvertDmaStatus(status);
  }
  status =
      dma_mgr_set_chn_dst_addr(&async_dma_resource_, ToHpmI2cDmaAddress(&i2c_->DATA));
  if (status != status_success)
  {
    return ConvertDmaStatus(status);
  }
  status = dma_mgr_set_chn_dst_work_mode(&async_dma_resource_,
                                         DMA_MGR_HANDSHAKE_MODE_HANDSHAKE);
  if (status != status_success)
  {
    return ConvertDmaStatus(status);
  }
  status =
      dma_mgr_set_chn_dst_addr_ctrl(&async_dma_resource_, DMA_MGR_ADDRESS_CONTROL_FIXED);
  if (status != status_success)
  {
    return ConvertDmaStatus(status);
  }
  status = dma_mgr_set_chn_src_width(&async_dma_resource_, DMA_MGR_TRANSFER_WIDTH_BYTE);
  if (status != status_success)
  {
    return ConvertDmaStatus(status);
  }
  status = dma_mgr_set_chn_dst_width(&async_dma_resource_, DMA_MGR_TRANSFER_WIDTH_BYTE);
  if (status != status_success)
  {
    return ConvertDmaStatus(status);
  }
  status = dma_mgr_set_chn_transize(&async_dma_resource_, size);
  if (status != status_success)
  {
    return ConvertDmaStatus(status);
  }

  i2c_dma_enable(i2c_);
  dma_clear_transfer_status(async_dma_resource_.base, async_dma_resource_.channel);
  status = dma_mgr_enable_channel(&async_dma_resource_);
  if (status != status_success)
  {
    i2c_dma_disable(i2c_);
    return ConvertDmaStatus(status);
  }
  return ErrorCode::OK;
}

void HPMI2C::StopAsyncDma()
{
  if (async_dma_ready_)
  {
    (void)dma_mgr_disable_channel(&async_dma_resource_);
  }
  if (i2c_ != nullptr)
  {
    i2c_dma_disable(i2c_);
  }
}

void HPMI2C::StopAndReleaseAsyncBus()
{
  IssueStopAndWait(i2c_, wait_policy_);
  ReleaseAsyncBus();
}

void HPMI2C::AsyncCompletionStateMachine::Reset(AsyncTransferContext& ctx)
{
  ctx.final_status.store(status_success, std::memory_order_release);
  ctx.should_recover.store(false, std::memory_order_release);
  ctx.dma_done.store(false, std::memory_order_release);
  ctx.cmpl_done.store(false, std::memory_order_release);
}

void HPMI2C::AsyncCompletionStateMachine::MarkDmaDone(AsyncTransferContext& ctx)
{
  ctx.dma_done.store(true, std::memory_order_release);
}

void HPMI2C::AsyncCompletionStateMachine::MarkI2cDone(AsyncTransferContext& ctx,
                                                      hpm_stat_t status, bool recover)
{
  ctx.cmpl_done.store(true, std::memory_order_release);
  ctx.final_status.store(status, std::memory_order_release);
  ctx.should_recover.store(recover, std::memory_order_release);
}

void HPMI2C::AsyncCompletionStateMachine::MarkFailure(AsyncTransferContext& ctx,
                                                      hpm_stat_t status, bool recover)
{
  ctx.final_status.store(status, std::memory_order_release);
  ctx.should_recover.store(recover, std::memory_order_release);
}

void HPMI2C::AsyncCompletionStateMachine::SetFinalStatus(AsyncTransferContext& ctx,
                                                         hpm_stat_t status)
{
  ctx.final_status.store(status, std::memory_order_release);
}

bool HPMI2C::AsyncCompletionStateMachine::DmaDone(const AsyncTransferContext& ctx)
{
  return ctx.dma_done.load(std::memory_order_acquire);
}

bool HPMI2C::AsyncCompletionStateMachine::Ready(const AsyncTransferContext& ctx)
{
  return DmaDone(ctx) && ctx.cmpl_done.load(std::memory_order_acquire);
}

hpm_stat_t HPMI2C::AsyncCompletionStateMachine::FinalStatus(
    const AsyncTransferContext& ctx)
{
  return ctx.final_status.load(std::memory_order_acquire);
}

bool HPMI2C::AsyncCompletionStateMachine::ShouldRecover(const AsyncTransferContext& ctx)
{
  return ctx.should_recover.load(std::memory_order_acquire);
}

void HPMI2C::ClearAsyncContext()
{
  async_ctx_.kind = AsyncTransferKind::NONE;
  async_ctx_.slave_addr = 0U;
  async_ctx_.mem_addr_size = MemAddrLength::BYTE_8;
  async_ctx_.mem_addr = 0U;
  async_ctx_.mem_addr_size_in_byte = 0U;
  async_ctx_.mem_addr_bytes[0] = 0U;
  async_ctx_.mem_addr_bytes[1] = 0U;
  async_ctx_.flags = 0U;
  async_ctx_.read_data = {nullptr, 0};
  async_ctx_.write_data = {nullptr, 0};
  async_ctx_.read_op = {};
  async_ctx_.write_op = {};
  AsyncCompletionStateMachine::Reset(async_ctx_);
}

void HPMI2C::ResetAsyncState()
{
  ClearAsyncContext();
  async_busy_.store(0U, std::memory_order_release);
  async_completion_claim_.store(0U, std::memory_order_release);
}

void HPMI2C::AbortAsyncStart(bool stop_dma, bool disable_irq, bool recover_controller)
{
  if (stop_dma)
  {
    StopAsyncDma();
  }
  if (disable_irq)
  {
    DisableAsyncI2cIrq();
  }

  if (recover_controller)
  {
    RecoverController();
  }
  else
  {
    ReleaseAsyncBus();
  }

  ResetAsyncState();
}

void HPMI2C::CompleteAsyncTransfer(bool in_isr, ErrorCode ans)
{
  if (!TryClaimAsyncCompletion())
  {
    return;
  }
  DisableAsyncI2cIrq();
  StopAsyncDma();
  ReleaseAsyncBus();

  const bool should_recover = AsyncCompletionStateMachine::ShouldRecover(async_ctx_);
  if (ans != ErrorCode::OK && should_recover)
  {
    RecoverController();
  }

  const AsyncTransferKind kind = async_ctx_.kind;
  ReadOperation read_op = async_ctx_.read_op;
  WriteOperation write_op = async_ctx_.write_op;
  async_ctx_.kind = AsyncTransferKind::NONE;
  ResetAsyncState();

  if (kind == AsyncTransferKind::WRITE)
  {
    CompleteAsyncOperation(write_op, in_isr, ans);
  }
  else
  {
    CompleteAsyncOperation(read_op, in_isr, ans);
  }
}

ErrorCode HPMI2C::StartWriteAsync(uint16_t slave_addr, ConstRawData write_data,
                                  WriteOperation& op)
{
  if (AsyncTransferActive())
  {
    return ErrorCode::BUSY;
  }

  ErrorCode ans = EnsureControllerReady();
  if (ans != ErrorCode::OK)
  {
    return ans;
  }

  async_busy_.store(1U, std::memory_order_release);
  ClearAsyncContext();
  async_ctx_.kind = AsyncTransferKind::WRITE;
  async_ctx_.slave_addr = slave_addr;
  async_ctx_.write_data = write_data;
  async_ctx_.write_op = op;
  async_ctx_.flags = BuildTransferFlags(kI2CFlagWriteCheckAck);
  async_completion_claim_.store(0U, std::memory_order_release);

  op.MarkAsRunning();
  StartAsyncBlockWaitIfNeeded(op);

  ans = EnableAsyncI2cIrq();
  if (ans != ErrorCode::OK)
  {
    AsyncCompletionStateMachine::MarkFailure(async_ctx_, status_fail, true);
    AbortAsyncStart(false, false, false);
    CancelAsyncBlockWaitIfNeeded(op);
    return ErrorCode::NOT_SUPPORT;
  }

  i2c_clear_fifo(i2c_);
  ans = StartAsyncWriteDma(write_data.addr_, static_cast<uint32_t>(write_data.size_));
  if (ans != ErrorCode::OK)
  {
    AsyncCompletionStateMachine::MarkFailure(async_ctx_, status_fail, true);
    AbortAsyncStart(false, true, false);
    CancelAsyncBlockWaitIfNeeded(op);
    return ans;
  }

  hpm_stat_t start_status = i2c_master_start_dma_write(
      i2c_, slave_addr, static_cast<uint32_t>(write_data.size_));
  ans = ConvertStatus(start_status);
  if (ans != ErrorCode::OK)
  {
    AsyncCompletionStateMachine::MarkFailure(async_ctx_, start_status,
                                             ShouldRecover(start_status));
    AbortAsyncStart(true, true, ShouldRecover(start_status));
    CancelAsyncBlockWaitIfNeeded(op);
    return ans;
  }

  if (op.type == WriteOperation::OperationType::BLOCK)
  {
    return WaitForAsyncBlockResult(op.data.sem_info.timeout);
  }
  return ErrorCode::OK;
}

ErrorCode HPMI2C::StartReadAsync(uint16_t slave_addr, RawData read_data,
                                 ReadOperation& op)
{
  if (AsyncTransferActive())
  {
    return ErrorCode::BUSY;
  }

  ErrorCode ans = EnsureControllerReady();
  if (ans != ErrorCode::OK)
  {
    return ans;
  }

  async_busy_.store(1U, std::memory_order_release);
  ClearAsyncContext();
  async_ctx_.kind = AsyncTransferKind::READ;
  async_ctx_.slave_addr = slave_addr;
  async_ctx_.read_data = read_data;
  async_ctx_.read_op = op;
  async_ctx_.flags = kI2CFlagRead;
  async_completion_claim_.store(0U, std::memory_order_release);

  op.MarkAsRunning();
  StartAsyncBlockWaitIfNeeded(op);

  ans = EnableAsyncI2cIrq();
  if (ans != ErrorCode::OK)
  {
    AsyncCompletionStateMachine::MarkFailure(async_ctx_, status_fail, true);
    AbortAsyncStart(false, false, false);
    CancelAsyncBlockWaitIfNeeded(op);
    return ErrorCode::NOT_SUPPORT;
  }

  i2c_clear_fifo(i2c_);
  ans = StartAsyncReadDma(read_data.addr_, static_cast<uint32_t>(read_data.size_));
  if (ans != ErrorCode::OK)
  {
    AsyncCompletionStateMachine::MarkFailure(async_ctx_, status_fail, true);
    AbortAsyncStart(false, true, false);
    CancelAsyncBlockWaitIfNeeded(op);
    return ans;
  }

  hpm_stat_t start_status =
      i2c_master_start_dma_read(i2c_, slave_addr, static_cast<uint32_t>(read_data.size_));
  ans = ConvertStatus(start_status);
  if (ans != ErrorCode::OK)
  {
    AsyncCompletionStateMachine::MarkFailure(async_ctx_, start_status,
                                             ShouldRecover(start_status));
    AbortAsyncStart(true, true, ShouldRecover(start_status));
    CancelAsyncBlockWaitIfNeeded(op);
    return ans;
  }

  if (op.type == ReadOperation::OperationType::BLOCK)
  {
    return WaitForAsyncBlockResult(op.data.sem_info.timeout);
  }
  return ErrorCode::OK;
}

ErrorCode HPMI2C::StartMemReadAsync(uint16_t slave_addr, uint16_t mem_addr,
                                    RawData read_data, ReadOperation& op,
                                    MemAddrLength mem_addr_size)
{
  if (AsyncTransferActive())
  {
    return ErrorCode::BUSY;
  }

  uint32_t addr_size = 0U;
  ErrorCode ans = ResolveMemAddressSize(mem_addr_size, addr_size);
  if (ans != ErrorCode::OK)
  {
    return ans;
  }
  if (addr_size > I2C_SOC_TRANSFER_COUNT_MAX)
  {
    return ErrorCode::SIZE_ERR;
  }

  ans = EnsureControllerReady();
  if (ans != ErrorCode::OK)
  {
    return ans;
  }

  async_busy_.store(1U, std::memory_order_release);
  ClearAsyncContext();
  async_ctx_.kind = AsyncTransferKind::MEM_READ;
  async_ctx_.slave_addr = slave_addr;
  async_ctx_.mem_addr = mem_addr;
  async_ctx_.mem_addr_size = mem_addr_size;
  async_ctx_.mem_addr_size_in_byte = addr_size;
  FillMemAddress(mem_addr, mem_addr_size, async_ctx_.mem_addr_bytes);
  async_ctx_.read_data = read_data;
  async_ctx_.read_op = op;
  async_ctx_.flags = kI2CFlagRead;
  async_completion_claim_.store(0U, std::memory_order_release);

  op.MarkAsRunning();
  StartAsyncBlockWaitIfNeeded(op);

  ans = PrepareAsyncTransfer(slave_addr, kI2CFlagNoStop, async_ctx_.mem_addr_size_in_byte,
                             true, true);
  if (ans != ErrorCode::OK)
  {
    ResetAsyncState();
    CancelAsyncBlockWaitIfNeeded(op);
    return ans;
  }

  for (uint32_t i = 0U; i < async_ctx_.mem_addr_size_in_byte; ++i)
  {
    i2c_write_byte(i2c_, async_ctx_.mem_addr_bytes[i]);
  }

  const bool mem_addr_complete =
      WaitUntil([this]() { return (i2c_get_status(i2c_) & I2C_STATUS_CMPL_MASK) != 0U; },
                wait_policy_.transfer_timeout_us);
  if (!mem_addr_complete)
  {
    AsyncCompletionStateMachine::MarkFailure(async_ctx_, status_timeout, true);
    StopAndReleaseAsyncBus();
    ResetAsyncState();
    CancelAsyncBlockWaitIfNeeded(op);
    return ErrorCode::TIMEOUT;
  }
  i2c_clear_status(i2c_, I2C_STATUS_CMPL_MASK);
  i2c_clear_fifo(i2c_);

  const uint16_t final_flags = BuildTransferFlags(kI2CFlagRead);
  i2c_master_set_slave_address(i2c_, slave_addr);
  i2c_set_direction(i2c_, I2C_DIR_MASTER_READ);
  i2c_master_enable_start_phase(i2c_);
  i2c_master_enable_addr_phase(i2c_);
  i2c_master_enable_stop_phase(i2c_);
  i2c_master_enable_data_phase(i2c_);
  i2c_set_data_count(i2c_, static_cast<uint32_t>(read_data.size_));
  i2c_dma_disable(i2c_);
  if ((final_flags & kI2CFlagAddr10Bit) != 0U)
  {
    i2c_enable_10bit_address_mode(i2c_, true);
  }

  ans = EnableAsyncI2cIrq();
  if (ans != ErrorCode::OK)
  {
    AsyncCompletionStateMachine::MarkFailure(async_ctx_, status_fail, true);
    StopAndReleaseAsyncBus();
    ResetAsyncState();
    CancelAsyncBlockWaitIfNeeded(op);
    return ErrorCode::NOT_SUPPORT;
  }

  ans = StartAsyncReadDma(read_data.addr_, static_cast<uint32_t>(read_data.size_));
  if (ans != ErrorCode::OK)
  {
    AsyncCompletionStateMachine::MarkFailure(async_ctx_, status_fail, true);
    DisableAsyncI2cIrq();
    StopAndReleaseAsyncBus();
    ResetAsyncState();
    CancelAsyncBlockWaitIfNeeded(op);
    return ans;
  }

  i2c_master_issue_data_transmission(i2c_);
  const bool addr_hit = WaitUntil(
      [this]() { return (i2c_get_status(i2c_) & I2C_STATUS_ADDRHIT_MASK) != 0U; },
      wait_policy_.addr_hit_timeout_us);
  if (!addr_hit)
  {
    AsyncCompletionStateMachine::MarkFailure(async_ctx_, status_i2c_no_addr_hit, true);
    AbortAsyncStart(true, true, true);
    CancelAsyncBlockWaitIfNeeded(op);
    return ErrorCode::NO_RESPONSE;
  }
  i2c_clear_status(i2c_, I2C_STATUS_ADDRHIT_MASK);

  if (op.type == ReadOperation::OperationType::BLOCK)
  {
    return WaitForAsyncBlockResult(op.data.sem_info.timeout);
  }
  return ErrorCode::OK;
}

ErrorCode HPMI2C::EnsureAsyncDmaReady()
{
  if (async_dma_ready_)
  {
    return ErrorCode::OK;
  }

  if (i2c_ == nullptr)
  {
    return ErrorCode::PTR_NULL;
  }

  async_dma_source_ = ResolveBoardI2cDmaSource(i2c_);
  if (async_dma_source_ == kInvalidDmaSource)
  {
    return ErrorCode::NOT_SUPPORT;
  }

  dma_mgr_init();
#if defined(BOARD_APP_I2C_DMA)
  hpm_stat_t status =
      dma_mgr_request_specified_resource(&async_dma_resource_, BOARD_APP_I2C_DMA);
#else
  hpm_stat_t status = status_fail;
#endif
  if (status != status_success)
  {
    status = dma_mgr_request_resource(&async_dma_resource_);
    if (status != status_success)
    {
      return ConvertDmaStatus(status);
    }
  }

  dma_mgr_chn_conf_t cfg{};
  dma_mgr_get_default_chn_config(&cfg);
  cfg.src_width = DMA_MGR_TRANSFER_WIDTH_BYTE;
  cfg.dst_width = DMA_MGR_TRANSFER_WIDTH_BYTE;
  cfg.en_dmamux = true;
  cfg.dmamux_src = async_dma_source_;
  cfg.interrupt_mask = DMA_MGR_INTERRUPT_MASK_ALL;
  status = dma_mgr_setup_channel(&async_dma_resource_, &cfg);
  if (status != status_success)
  {
    (void)dma_mgr_release_resource(&async_dma_resource_);
    async_dma_resource_ = {nullptr, 0U, -1};
    return ConvertDmaStatus(status);
  }

  status = dma_mgr_install_chn_tc_callback(&async_dma_resource_, &HPMI2C::OnDmaTcCallback,
                                           this);
  if (status != status_success)
  {
    (void)dma_mgr_release_resource(&async_dma_resource_);
    async_dma_resource_ = {nullptr, 0U, -1};
    return ConvertDmaStatus(status);
  }
  status = dma_mgr_install_chn_error_callback(&async_dma_resource_,
                                              &HPMI2C::OnDmaErrorCallback, this);
  if (status != status_success)
  {
    (void)dma_mgr_release_resource(&async_dma_resource_);
    async_dma_resource_ = {nullptr, 0U, -1};
    return ConvertDmaStatus(status);
  }
  status = dma_mgr_install_chn_abort_callback(&async_dma_resource_,
                                              &HPMI2C::OnDmaAbortCallback, this);
  if (status != status_success)
  {
    (void)dma_mgr_release_resource(&async_dma_resource_);
    async_dma_resource_ = {nullptr, 0U, -1};
    return ConvertDmaStatus(status);
  }
  status = dma_mgr_enable_chn_irq(&async_dma_resource_, DMA_MGR_INTERRUPT_MASK_TC |
                                                            DMA_MGR_INTERRUPT_MASK_ERROR |
                                                            DMA_MGR_INTERRUPT_MASK_ABORT);
  if (status != status_success)
  {
    (void)dma_mgr_release_resource(&async_dma_resource_);
    async_dma_resource_ = {nullptr, 0U, -1};
    return ConvertDmaStatus(status);
  }
  status = dma_mgr_enable_dma_irq_with_priority(&async_dma_resource_, 1U);
  if (status != status_success)
  {
    (void)dma_mgr_release_resource(&async_dma_resource_);
    async_dma_resource_ = {nullptr, 0U, -1};
    return ConvertDmaStatus(status);
  }

  async_dma_ready_ = true;
  return ErrorCode::OK;
}

ErrorCode HPMI2C::WaitForAsyncBlockResult(uint32_t timeout)
{
  const ErrorCode ans = block_wait_.Wait(timeout);
  if (ans == ErrorCode::TIMEOUT && AsyncTransferActive())
  {
    AsyncCompletionStateMachine::MarkFailure(async_ctx_, status_timeout, true);
    CompleteAsyncTransfer(false, ErrorCode::TIMEOUT);
  }
  return ans;
}

void HPMI2C::ReleaseAsyncBus()
{
  if (i2c_ == nullptr)
  {
    return;
  }

  i2c_clear_status(i2c_, I2C_STATUS_CMPL_MASK | I2C_STATUS_ADDRHIT_MASK);
  i2c_master_disable_data_phase(i2c_);
  i2c_master_disable_addr_phase(i2c_);
  i2c_master_disable_start_phase(i2c_);
  i2c_master_disable_stop_phase(i2c_);
  i2c_master_issue_data_transmission(i2c_);
}

ErrorCode HPMI2C::EnableAsyncI2cIrq()
{
#if !LIBXR_HPM_I2C_HAS_INTERRUPT
  return ErrorCode::NOT_SUPPORT;
#else
  const int32_t irq = ResolveBoardI2cIrq(i2c_);
  const int32_t index = ResolveI2cIndex(i2c_);
  if (irq < 0 || index < 0 || static_cast<size_t>(index) >= kHpmI2cInstanceCount)
  {
    return ErrorCode::NOT_SUPPORT;
  }
  if (g_hpm_i2c_instance_map[index] != nullptr && g_hpm_i2c_instance_map[index] != this)
  {
    return ErrorCode::NOT_SUPPORT;
  }

  g_hpm_i2c_instance_map[index] = this;
  i2c_clear_status(
      i2c_, I2C_STATUS_CMPL_MASK | I2C_STATUS_ADDRHIT_MASK | I2C_STATUS_ARBLOSE_MASK);
  i2c_enable_irq(i2c_, I2C_EVENT_TRANSACTION_COMPLETE | I2C_EVENT_LOSS_ARBITRATION);
  intc_m_enable_irq_with_priority(static_cast<uint32_t>(irq), 1U);
  return ErrorCode::OK;
#endif
}

void HPMI2C::DisableAsyncI2cIrq()
{
#if LIBXR_HPM_I2C_HAS_INTERRUPT
  if (i2c_ != nullptr)
  {
    i2c_disable_irq(i2c_, I2C_EVENT_TRANSACTION_COMPLETE | I2C_EVENT_LOSS_ARBITRATION);
    const int32_t irq = ResolveBoardI2cIrq(i2c_);
    if (irq >= 0)
    {
      intc_m_disable_irq(static_cast<uint32_t>(irq));
    }
  }
  const int32_t index = ResolveI2cIndex(i2c_);
  if (index >= 0 && static_cast<size_t>(index) < kHpmI2cInstanceCount &&
      g_hpm_i2c_instance_map[index] == this)
  {
    g_hpm_i2c_instance_map[index] = nullptr;
  }
#endif
}

void HPMI2C::HandleAsyncInterrupt(bool in_isr)
{
  if (!AsyncTransferActive() || i2c_ == nullptr)
  {
    return;
  }

  const uint32_t status = i2c_get_status(i2c_);
  if ((status & I2C_STATUS_ARBLOSE_MASK) != 0U)
  {
    i2c_clear_status(i2c_, I2C_STATUS_ARBLOSE_MASK);
    AsyncCompletionStateMachine::MarkI2cDone(async_ctx_, status_timeout, true);
    CompleteAsyncTransfer(in_isr, ErrorCode::TIMEOUT);
    return;
  }

  if ((status & I2C_STATUS_CMPL_MASK) != 0U)
  {
    i2c_clear_status(i2c_, I2C_STATUS_CMPL_MASK);
    if (async_ctx_.kind == AsyncTransferKind::WRITE &&
        (async_ctx_.flags & kI2CFlagWriteCheckAck) != 0U && !I2C_STATUS_ACK_GET(status))
    {
      AsyncCompletionStateMachine::MarkI2cDone(async_ctx_, status_i2c_no_ack, true);
    }
    else
    {
      AsyncCompletionStateMachine::MarkI2cDone(async_ctx_, status_success, false);
    }
  }

  MaybeCompleteAsyncTransfer(in_isr);
}

void HPMI2C::MaybeCompleteAsyncTransfer(bool in_isr)
{
  if (!AsyncTransferActive() || !AsyncCompletionStateMachine::Ready(async_ctx_))
  {
    return;
  }

  if (async_ctx_.kind == AsyncTransferKind::READ ||
      async_ctx_.kind == AsyncTransferKind::MEM_READ)
  {
    InvalidateDCacheIfNeeded(async_ctx_.read_data.addr_,
                             static_cast<uint32_t>(async_ctx_.read_data.size_));
  }
  hpm_stat_t final_status = AsyncCompletionStateMachine::FinalStatus(async_ctx_);
  if (final_status == status_success && i2c_get_data_count(i2c_) != 0U)
  {
    final_status = status_i2c_transmit_not_completed;
    AsyncCompletionStateMachine::SetFinalStatus(async_ctx_, final_status);
  }
  CompleteAsyncTransfer(in_isr, ConvertStatus(final_status));
}

bool HPMI2C::TryClaimAsyncCompletion()
{
  uint32_t expected = 0U;
  return async_completion_claim_.compare_exchange_strong(
      expected, 1U, std::memory_order_acq_rel, std::memory_order_acquire);
}

void HPMI2C::OnDmaTcCallback(DMA_Type* base, uint32_t channel, void* cb_data_ptr)
{
  UNUSED(base);
  UNUSED(channel);
  auto* self = static_cast<HPMI2C*>(cb_data_ptr);
  if (self == nullptr || !self->AsyncTransferActive())
  {
    return;
  }

  AsyncCompletionStateMachine::MarkDmaDone(self->async_ctx_);
  self->MaybeCompleteAsyncTransfer(true);
}

void HPMI2C::OnDmaErrorCallback(DMA_Type* base, uint32_t channel, void* cb_data_ptr)
{
  UNUSED(base);
  UNUSED(channel);
  auto* self = static_cast<HPMI2C*>(cb_data_ptr);
  if (self == nullptr || !self->AsyncTransferActive())
  {
    return;
  }
  if (AsyncCompletionStateMachine::DmaDone(self->async_ctx_))
  {
    return;
  }
  AsyncCompletionStateMachine::MarkFailure(self->async_ctx_, status_fail, true);
  self->CompleteAsyncTransfer(true, ErrorCode::FAILED);
}

void HPMI2C::OnDmaAbortCallback(DMA_Type* base, uint32_t channel, void* cb_data_ptr)
{
  UNUSED(base);
  UNUSED(channel);
  auto* self = static_cast<HPMI2C*>(cb_data_ptr);
  if (self == nullptr || !self->AsyncTransferActive())
  {
    return;
  }
  if (AsyncCompletionStateMachine::DmaDone(self->async_ctx_))
  {
    return;
  }
  AsyncCompletionStateMachine::MarkFailure(self->async_ctx_, status_fail, true);
  self->CompleteAsyncTransfer(true, ErrorCode::FAILED);
}
#endif

ErrorCode HPMI2C::ValidateTransferArgs(uint16_t slave_addr, RawData data,
                                       bool allow_zero_size) const
{
  if (data.size_ == 0)
  {
    return allow_zero_size ? ErrorCode::OK : ErrorCode::ARG_ERR;
  }
  if (data.addr_ == nullptr)
  {
    return ErrorCode::PTR_NULL;
  }
  if (data.size_ > I2C_SOC_TRANSFER_COUNT_MAX)
  {
    return ErrorCode::SIZE_ERR;
  }
  return ValidateSlaveAddress(slave_addr);
}

ErrorCode HPMI2C::ValidateTransferArgs(uint16_t slave_addr, ConstRawData data,
                                       bool allow_zero_size) const
{
  if (data.size_ == 0)
  {
    return allow_zero_size ? ErrorCode::OK : ErrorCode::ARG_ERR;
  }
  if (data.addr_ == nullptr)
  {
    return ErrorCode::PTR_NULL;
  }
  if (data.size_ > I2C_SOC_TRANSFER_COUNT_MAX)
  {
    return ErrorCode::SIZE_ERR;
  }
  return ValidateSlaveAddress(slave_addr);
}

ErrorCode HPMI2C::ValidateMemWriteArgs(uint16_t slave_addr, ConstRawData write_data,
                                       MemAddrLength mem_addr_size,
                                       uint32_t& addr_size) const
{
  if (write_data.size_ > 0 && write_data.addr_ == nullptr)
  {
    return ErrorCode::PTR_NULL;
  }

  ErrorCode ans = ResolveMemAddressSize(mem_addr_size, addr_size);
  if (ans != ErrorCode::OK)
  {
    return ans;
  }
  if ((addr_size + write_data.size_) > I2C_SOC_TRANSFER_COUNT_MAX)
  {
    return ErrorCode::SIZE_ERR;
  }
  return ValidateSlaveAddress(slave_addr);
}

ErrorCode HPMI2C::SetConfig(Configuration config)
{
  if (i2c_ == nullptr)
  {
    return ErrorCode::PTR_NULL;
  }
#if LIBXR_HPM_I2C_HAS_DMA_MGR
  if (AsyncTransferActive())
  {
    return ErrorCode::BUSY;
  }
#endif
  if ((i2c_get_status(i2c_) & I2C_STATUS_BUSBUSY_MASK) != 0U)
  {
    return ErrorCode::BUSY;
  }

  return ApplyConfig(config);
}

ErrorCode HPMI2C::Read(uint16_t slave_addr, RawData read_data, ReadOperation& op,
                       bool in_isr)
{
  const ErrorCode arg_ans = ValidateTransferArgs(slave_addr, read_data, true);
  if (arg_ans != ErrorCode::OK || read_data.size_ == 0)
  {
    return FinishOperation(op, in_isr, arg_ans);
  }

#if LIBXR_HPM_I2C_HAS_DMA_MGR
  if (AsyncTransferActive())
  {
    return FinishOperation(op, in_isr, ErrorCode::BUSY);
  }

  if (op.type != ReadOperation::OperationType::BLOCK)
  {
    const ErrorCode ans = StartReadAsync(slave_addr, read_data, op);
    if (ans != ErrorCode::OK)
    {
      op.UpdateStatus(in_isr, ans);
    }
    return ans;
  }
#endif

  ErrorCode ans = EnsureControllerReady();
  if (ans != ErrorCode::OK)
  {
    return FinishOperation(op, in_isr, ans);
  }

  hpm_stat_t status =
      i2c_master_read(i2c_, slave_addr, static_cast<uint8_t*>(read_data.addr_),
                      static_cast<uint32_t>(read_data.size_));
  ans = ConvertStatus(status);
  if (ShouldRecover(status))
  {
    RecoverController();
  }
  return FinishOperation(op, in_isr, ans);
}

ErrorCode HPMI2C::DoSequenceWrite(uint16_t slave_addr, ConstRawData write_data,
                                  SequenceFrame frame, bool check_ack)
{
#if LIBXR_HPM_I2C_HAS_DMA_MGR
  if (AsyncTransferActive())
  {
    return ErrorCode::BUSY;
  }
#endif

  ErrorCode ans = EnsureControllerReady();
  if (ans != ErrorCode::OK)
  {
    return ans;
  }

  hpm_stat_t status;
  if (address_mode_ == AddressMode::ADDR_10BIT)
  {
    uint16_t flags = kI2CFlagWriteCheckAck;
    if (!check_ack)
    {
      flags = 0U;
    }
    switch (frame)
    {
      case SequenceFrame::FIRST:
        flags |= 0U;
        flags |= kI2CFlagAddr10Bit;
        flags |= kI2CFlagNoStop;
        break;
      case SequenceFrame::NEXT:
        flags |= kI2CFlagAddr10Bit | I2C_NO_START | I2C_NO_ADDRESS | kI2CFlagNoStop;
        break;
      case SequenceFrame::LAST:
      default:
        flags |= kI2CFlagAddr10Bit | I2C_NO_START | I2C_NO_ADDRESS;
        break;
    }
    const hpm_stat_t raw_status = DoManualTransferWithFlags(
        slave_addr, RawData(const_cast<void*>(write_data.addr_), write_data.size_),
        flags);
    status = raw_status;
  }
  else
  {
    status = i2c_master_seq_transmit_check_ack(
        i2c_, slave_addr,
        const_cast<uint8_t*>(static_cast<const uint8_t*>(write_data.addr_)),
        static_cast<uint32_t>(write_data.size_), ConvertSequenceFrame(frame), check_ack);
  }

  ans = ConvertStatus(status);
  if (ShouldRecover(status))
  {
    RecoverController();
  }
  return ans;
}

ErrorCode HPMI2C::DoSequenceRead(uint16_t slave_addr, RawData read_data,
                                 SequenceFrame frame)
{
#if LIBXR_HPM_I2C_HAS_DMA_MGR
  if (AsyncTransferActive())
  {
    return ErrorCode::BUSY;
  }
#endif

  ErrorCode ans = EnsureControllerReady();
  if (ans != ErrorCode::OK)
  {
    return ans;
  }

  hpm_stat_t status;
  if (address_mode_ == AddressMode::ADDR_10BIT)
  {
    uint16_t flags = kI2CFlagRead | kI2CFlagAddr10Bit;
    switch (frame)
    {
      case SequenceFrame::FIRST:
        flags |= kI2CFlagNoStop;
        break;
      case SequenceFrame::NEXT:
        flags |= I2C_NO_START | I2C_NO_ADDRESS | kI2CFlagNoStop;
        break;
      case SequenceFrame::LAST:
      default:
        flags |= I2C_NO_START | I2C_NO_ADDRESS;
        break;
    }
    status = DoManualTransferWithFlags(slave_addr, read_data, flags);
  }
  else
  {
    status = i2c_master_seq_receive(
        i2c_, slave_addr, static_cast<uint8_t*>(read_data.addr_),
        static_cast<uint32_t>(read_data.size_), ConvertSequenceFrame(frame));
  }

  ans = ConvertStatus(status);
  if (ShouldRecover(status))
  {
    RecoverController();
  }
  return ans;
}

ErrorCode HPMI2C::DoTransferWithFlags(uint16_t slave_addr, RawData data, uint16_t flags)
{
#if LIBXR_HPM_I2C_HAS_DMA_MGR
  if (AsyncTransferActive())
  {
    return ErrorCode::BUSY;
  }
#endif

  ErrorCode ans = EnsureControllerReady();
  if (ans != ErrorCode::OK)
  {
    return ans;
  }

  const uint16_t final_flags = BuildTransferFlags(flags);
  const hpm_stat_t status = DoManualTransferWithFlags(slave_addr, data, final_flags);
  ans = ConvertStatus(status);
  if (ShouldRecover(status))
  {
    RecoverController();
  }
  return ans;
}

hpm_stat_t HPMI2C::DoManualTransferWithFlags(uint16_t slave_addr, RawData data,
                                             uint16_t flags)
{
#if LIBXR_HPM_I2C_HAS_DMA_MGR
  if (AsyncTransferActive())
  {
    return status_i2c_bus_busy;
  }
#endif

  return DoManualTransferWithFlagsImpl(i2c_, slave_addr, data, flags, wait_policy_);
}

ErrorCode HPMI2C::SequenceWrite(uint16_t slave_addr, ConstRawData write_data,
                                SequenceFrame frame, bool check_ack, WriteOperation& op,
                                bool in_isr)
{
  const ErrorCode arg_ans = ValidateTransferArgs(slave_addr, write_data, false);
  if (arg_ans != ErrorCode::OK)
  {
    return FinishOperation(op, in_isr, arg_ans);
  }
#if LIBXR_HPM_I2C_HAS_DMA_MGR
  if (AsyncTransferActive())
  {
    return FinishOperation(op, in_isr, ErrorCode::BUSY);
  }
#endif
  return FinishOperation(op, in_isr,
                         DoSequenceWrite(slave_addr, write_data, frame, check_ack));
}

ErrorCode HPMI2C::SequenceRead(uint16_t slave_addr, RawData read_data,
                               SequenceFrame frame, ReadOperation& op, bool in_isr)
{
  const ErrorCode arg_ans = ValidateTransferArgs(slave_addr, read_data, false);
  if (arg_ans != ErrorCode::OK)
  {
    return FinishOperation(op, in_isr, arg_ans);
  }
#if LIBXR_HPM_I2C_HAS_DMA_MGR
  if (AsyncTransferActive())
  {
    return FinishOperation(op, in_isr, ErrorCode::BUSY);
  }
#endif
  return FinishOperation(op, in_isr, DoSequenceRead(slave_addr, read_data, frame));
}

ErrorCode HPMI2C::TransferWithFlags(uint16_t slave_addr, RawData data, uint16_t flags,
                                    ReadOperation& op, bool in_isr)
{
  const ErrorCode arg_ans = ValidateTransferArgs(slave_addr, data, false);
  if (arg_ans != ErrorCode::OK)
  {
    return FinishOperation(op, in_isr, arg_ans);
  }
#if LIBXR_HPM_I2C_HAS_DMA_MGR
  if (AsyncTransferActive())
  {
    return FinishOperation(op, in_isr, ErrorCode::BUSY);
  }
#endif
  return FinishOperation(op, in_isr, DoTransferWithFlags(slave_addr, data, flags));
}

ErrorCode HPMI2C::TransferWithFlags(uint16_t slave_addr, ConstRawData data,
                                    uint16_t flags, WriteOperation& op, bool in_isr)
{
  const ErrorCode arg_ans = ValidateTransferArgs(slave_addr, data, false);
  if (arg_ans != ErrorCode::OK)
  {
    return FinishOperation(op, in_isr, arg_ans);
  }
#if LIBXR_HPM_I2C_HAS_DMA_MGR
  if (AsyncTransferActive())
  {
    return FinishOperation(op, in_isr, ErrorCode::BUSY);
  }
#endif
  return FinishOperation(
      op, in_isr,
      DoTransferWithFlags(slave_addr, RawData(const_cast<void*>(data.addr_), data.size_),
                          static_cast<uint16_t>(flags & ~kI2CFlagRead)));
}

ErrorCode HPMI2C::Write(uint16_t slave_addr, ConstRawData write_data, WriteOperation& op,
                        bool in_isr)
{
  const ErrorCode arg_ans = ValidateTransferArgs(slave_addr, write_data, true);
  if (arg_ans != ErrorCode::OK || write_data.size_ == 0)
  {
    return FinishOperation(op, in_isr, arg_ans);
  }

#if LIBXR_HPM_I2C_HAS_DMA_MGR
  if (AsyncTransferActive())
  {
    return FinishOperation(op, in_isr, ErrorCode::BUSY);
  }

  if (op.type != WriteOperation::OperationType::BLOCK)
  {
    const ErrorCode ans = StartWriteAsync(slave_addr, write_data, op);
    if (ans != ErrorCode::OK)
    {
      op.UpdateStatus(in_isr, ans);
    }
    return ans;
  }
#endif

  ErrorCode ans = EnsureControllerReady();
  if (ans != ErrorCode::OK)
  {
    return FinishOperation(op, in_isr, ans);
  }

  hpm_stat_t status = i2c_master_write(
      i2c_, slave_addr,
      const_cast<uint8_t*>(static_cast<const uint8_t*>(write_data.addr_)),
      static_cast<uint32_t>(write_data.size_));
  ans = ConvertStatus(status);
  if (ShouldRecover(status))
  {
    RecoverController();
  }
  return FinishOperation(op, in_isr, ans);
}

ErrorCode HPMI2C::MemRead(uint16_t slave_addr, uint16_t mem_addr, RawData read_data,
                          ReadOperation& op, MemAddrLength mem_addr_size, bool in_isr)
{
  const ErrorCode arg_ans = ValidateTransferArgs(slave_addr, read_data, true);
  if (arg_ans != ErrorCode::OK || read_data.size_ == 0)
  {
    return FinishOperation(op, in_isr, arg_ans);
  }

#if LIBXR_HPM_I2C_HAS_DMA_MGR
  if (AsyncTransferActive())
  {
    return FinishOperation(op, in_isr, ErrorCode::BUSY);
  }

  if (op.type != ReadOperation::OperationType::BLOCK)
  {
    const ErrorCode ans =
        StartMemReadAsync(slave_addr, mem_addr, read_data, op, mem_addr_size);
    if (ans != ErrorCode::OK)
    {
      op.UpdateStatus(in_isr, ans);
    }
    return ans;
  }
#endif

  uint32_t addr_size = 0;
  ErrorCode ans = ResolveMemAddressSize(mem_addr_size, addr_size);
  if (ans != ErrorCode::OK)
  {
    return FinishOperation(op, in_isr, ans);
  }
  if (addr_size > I2C_SOC_TRANSFER_COUNT_MAX)
  {
    return FinishOperation(op, in_isr, ErrorCode::SIZE_ERR);
  }

  ans = EnsureControllerReady();
  if (ans != ErrorCode::OK)
  {
    return FinishOperation(op, in_isr, ans);
  }

  uint8_t addr[2] = {};
  FillMemAddress(mem_addr, mem_addr_size, addr);

  if (address_mode_ == AddressMode::ADDR_10BIT)
  {
    ans = DoSequenceWrite(slave_addr, ConstRawData(addr, addr_size), SequenceFrame::FIRST,
                          true);
    if (ans == ErrorCode::OK)
    {
      ans = DoSequenceRead(slave_addr, read_data, SequenceFrame::LAST);
    }
  }
  else
  {
    const hpm_stat_t status = i2c_master_address_read(
        i2c_, slave_addr, addr, addr_size, static_cast<uint8_t*>(read_data.addr_),
        static_cast<uint32_t>(read_data.size_));
    ans = ConvertStatus(status);
    if (ShouldRecover(status))
    {
      RecoverController();
    }
  }
  return FinishOperation(op, in_isr, ans);
}

ErrorCode HPMI2C::MemWrite(uint16_t slave_addr, uint16_t mem_addr,
                           ConstRawData write_data, WriteOperation& op,
                           MemAddrLength mem_addr_size, bool in_isr)
{
  uint32_t addr_size = 0;
  ErrorCode ans = ValidateMemWriteArgs(slave_addr, write_data, mem_addr_size, addr_size);
  if (ans != ErrorCode::OK)
  {
    return FinishOperation(op, in_isr, ans);
  }

#if LIBXR_HPM_I2C_HAS_DMA_MGR
  if (AsyncTransferActive())
  {
    return FinishOperation(op, in_isr, ErrorCode::BUSY);
  }
#endif

  ans = EnsureControllerReady();
  if (ans != ErrorCode::OK)
  {
    return FinishOperation(op, in_isr, ans);
  }

  uint8_t addr[2] = {};
  FillMemAddress(mem_addr, mem_addr_size, addr);

  if (address_mode_ == AddressMode::ADDR_10BIT)
  {
    if (write_data.size_ == 0)
    {
      hpm_stat_t status =
          DoManualTransferWithFlags(slave_addr, RawData(addr, addr_size),
                                    kI2CFlagAddr10Bit | kI2CFlagWriteCheckAck);
      ans = ConvertStatus(status);
      if (ShouldRecover(status))
      {
        RecoverController();
      }
    }
    else
    {
      ans = DoSequenceWrite(slave_addr, ConstRawData(addr, addr_size),
                            SequenceFrame::FIRST, true);
      if (ans == ErrorCode::OK)
      {
        ans = DoSequenceWrite(slave_addr, write_data, SequenceFrame::LAST, true);
      }
    }
  }
  else
  {
    hpm_stat_t status;
    if (write_data.size_ == 0)
    {
      status = i2c_master_write(i2c_, slave_addr, addr, addr_size);
    }
    else
    {
      status = i2c_master_address_write(
          i2c_, slave_addr, addr, addr_size,
          const_cast<uint8_t*>(static_cast<const uint8_t*>(write_data.addr_)),
          static_cast<uint32_t>(write_data.size_));
    }

    ans = ConvertStatus(status);
    if (ShouldRecover(status))
    {
      RecoverController();
    }
  }
  return FinishOperation(op, in_isr, ans);
}

#else

extern "C" void libxr_hpm_i2c_process_interrupt(LibXRHpmI2cType* ptr) { UNUSED(ptr); }

HPMI2C::HPMI2C(LibXRHpmI2cType* i2c, clock_name_t clock, bool auto_board_init,
               I2C::Configuration config)
    : i2c_(i2c), clock_(clock), current_config_(config), auto_board_init_(auto_board_init)
{
  (void)i2c_;
  (void)clock_;
  (void)auto_board_init_;
}

ErrorCode HPMI2C::SetAddressMode(AddressMode mode)
{
  address_mode_ = mode;
  return ErrorCode::NOT_SUPPORT;
}

ErrorCode HPMI2C::ConvertStatus(hpm_stat_t status)
{
  UNUSED(status);
  return ErrorCode::NOT_SUPPORT;
}

uint16_t HPMI2C::BuildTransferFlags(uint16_t flags) const { return flags; }

ErrorCode HPMI2C::DoSequenceWrite(uint16_t slave_addr, ConstRawData write_data,
                                  SequenceFrame frame, bool check_ack)
{
  UNUSED(slave_addr);
  UNUSED(write_data);
  UNUSED(frame);
  UNUSED(check_ack);
  return ErrorCode::NOT_SUPPORT;
}

ErrorCode HPMI2C::DoSequenceRead(uint16_t slave_addr, RawData read_data,
                                 SequenceFrame frame)
{
  UNUSED(slave_addr);
  UNUSED(read_data);
  UNUSED(frame);
  return ErrorCode::NOT_SUPPORT;
}

ErrorCode HPMI2C::DoTransferWithFlags(uint16_t slave_addr, RawData data, uint16_t flags)
{
  UNUSED(slave_addr);
  UNUSED(data);
  UNUSED(flags);
  return ErrorCode::NOT_SUPPORT;
}

hpm_stat_t HPMI2C::DoManualTransferWithFlags(uint16_t slave_addr, RawData data,
                                             uint16_t flags)
{
  UNUSED(slave_addr);
  UNUSED(data);
  UNUSED(flags);
  return status_fail;
}

ErrorCode HPMI2C::EnsureClockReady() { return ErrorCode::NOT_SUPPORT; }

ErrorCode HPMI2C::EnsureControllerReady() { return ErrorCode::NOT_SUPPORT; }

ErrorCode HPMI2C::ApplyConfig(const Configuration& config)
{
  current_config_ = config;
  configured_ = false;
  return ErrorCode::NOT_SUPPORT;
}

bool HPMI2C::ShouldRecover(hpm_stat_t status)
{
  UNUSED(status);
  return false;
}

void HPMI2C::RecoverController() {}

void HPMI2C::TryRecoverBusLines() {}

ErrorCode HPMI2C::SetConfig(Configuration config)
{
  current_config_ = config;
  configured_ = false;
  return ErrorCode::NOT_SUPPORT;
}

ErrorCode HPMI2C::Read(uint16_t slave_addr, RawData read_data, ReadOperation& op,
                       bool in_isr)
{
  UNUSED(slave_addr);
  UNUSED(read_data);
  return FinishOperation(op, in_isr, ErrorCode::NOT_SUPPORT);
}

ErrorCode HPMI2C::Write(uint16_t slave_addr, ConstRawData write_data, WriteOperation& op,
                        bool in_isr)
{
  UNUSED(slave_addr);
  UNUSED(write_data);
  return FinishOperation(op, in_isr, ErrorCode::NOT_SUPPORT);
}

ErrorCode HPMI2C::MemRead(uint16_t slave_addr, uint16_t mem_addr, RawData read_data,
                          ReadOperation& op, MemAddrLength mem_addr_size, bool in_isr)
{
  UNUSED(slave_addr);
  UNUSED(mem_addr);
  UNUSED(read_data);
  UNUSED(mem_addr_size);
  return FinishOperation(op, in_isr, ErrorCode::NOT_SUPPORT);
}

ErrorCode HPMI2C::MemWrite(uint16_t slave_addr, uint16_t mem_addr,
                           ConstRawData write_data, WriteOperation& op,
                           MemAddrLength mem_addr_size, bool in_isr)
{
  UNUSED(slave_addr);
  UNUSED(mem_addr);
  UNUSED(write_data);
  UNUSED(mem_addr_size);
  return FinishOperation(op, in_isr, ErrorCode::NOT_SUPPORT);
}

ErrorCode HPMI2C::SequenceWrite(uint16_t slave_addr, ConstRawData write_data,
                                SequenceFrame frame, bool check_ack, WriteOperation& op,
                                bool in_isr)
{
  UNUSED(slave_addr);
  UNUSED(write_data);
  UNUSED(frame);
  UNUSED(check_ack);
  return FinishOperation(op, in_isr, ErrorCode::NOT_SUPPORT);
}

ErrorCode HPMI2C::SequenceRead(uint16_t slave_addr, RawData read_data,
                               SequenceFrame frame, ReadOperation& op, bool in_isr)
{
  UNUSED(slave_addr);
  UNUSED(read_data);
  UNUSED(frame);
  return FinishOperation(op, in_isr, ErrorCode::NOT_SUPPORT);
}

ErrorCode HPMI2C::TransferWithFlags(uint16_t slave_addr, RawData data, uint16_t flags,
                                    ReadOperation& op, bool in_isr)
{
  UNUSED(slave_addr);
  UNUSED(data);
  UNUSED(flags);
  return FinishOperation(op, in_isr, ErrorCode::NOT_SUPPORT);
}

ErrorCode HPMI2C::TransferWithFlags(uint16_t slave_addr, ConstRawData data,
                                    uint16_t flags, WriteOperation& op, bool in_isr)
{
  UNUSED(slave_addr);
  UNUSED(data);
  UNUSED(flags);
  return FinishOperation(op, in_isr, ErrorCode::NOT_SUPPORT);
}

#endif
