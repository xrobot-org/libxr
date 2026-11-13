#include "hpm_can.hpp"

#if LIBXR_HPM_CAN_SUPPORTED

#include "hpm_interrupt.h"

using namespace LibXR;

namespace
{
constexpr uint32_t HPM_CAN_INTERRUPT_MASK = LibXR::detail::MCAN_DRIVER_INTERRUPT_MASK;
}  // namespace

HPMCAN::HPMCAN(MCAN_Type* can, clock_name_t clock, uint32_t irq, uint32_t queue_size,
               void* msg_buf, uint32_t msg_buf_size)
    : can_(can),
      clock_(clock),
      irq_(irq),
      msg_buf_(msg_buf),
      msg_buf_size_(msg_buf_size),
      tx_queue_(queue_size)
{
  index_ = detail::GetMcanInstanceIndex(can_);
  if (can_ == nullptr || index_ >= MAX_INSTANCES)
  {
    ASSERT(false);
    can_ = nullptr;
    index_ = MAX_INSTANCES;
    return;
  }

  if (!detail::RegisterMcanOwner(index_, this, detail::McanOwnerKind::CLASSIC_CAN,
                                 [](void* owner, bool)
                                 { static_cast<HPMCAN*>(owner)->ProcessInterrupt(); }))
  {
    ASSERT(false);
    can_ = nullptr;
    index_ = MAX_INSTANCES;
  }
}

HPMCAN::~HPMCAN()
{
  configured_.store(false, std::memory_order_release);
  if (can_ != nullptr)
  {
    (void)DisableInterrupt();
    if (clock_ready_)
    {
      DisableCanInterrupts();
      mcan_clear_interrupt_flags(can_, 0xFFFFFFFFUL);
      mcan_deinit(can_);
    }
  }
  detail::UnregisterMcanOwner(index_, this);
}

ErrorCode HPMCAN::SetConfig(const CAN::Configuration& cfg)
{
  if (can_ == nullptr)
  {
    return ErrorCode::PTR_NULL;
  }
  if (cfg.mode.loopback && cfg.mode.listen_only)
  {
    return ErrorCode::NOT_SUPPORT;
  }
  if (cfg.mode.triple_sampling)
  {
    return ErrorCode::NOT_SUPPORT;
  }
  if (!detail::IsValidMcanSamplePoint(cfg.sample_point))
  {
    return ErrorCode::ARG_ERR;
  }

  const bool use_low_level = detail::HasAnyLowLevelTiming(cfg.bit_timing);
  if (use_low_level)
  {
    if (!detail::CanApplyMcanLowLevelTiming(cfg.bit_timing))
    {
      return ErrorCode::ARG_ERR;
    }
  }
  else if (cfg.bitrate == 0U)
  {
    return ErrorCode::ARG_ERR;
  }

  uint32_t expected = 0U;
  if (!tx_lock_.compare_exchange_strong(expected, 1U, std::memory_order_acquire,
                                        std::memory_order_relaxed))
  {
    return ErrorCode::BUSY;
  }

  const uint32_t clock_hz = detail::AcquireMcanClock(clock_);
  if (clock_hz == 0U)
  {
    tx_lock_.store(0U, std::memory_order_release);
    return ErrorCode::INIT_ERR;
  }
  clock_ready_ = true;

  configured_.store(false, std::memory_order_release);
  (void)DisableInterrupt();
  DisableCanInterrupts();
  mcan_clear_interrupt_flags(can_, 0xFFFFFFFFUL);
  mcan_deinit(can_);

  const ErrorCode MSG_BUF_STATUS = ApplyMessageBuffer();
  if (MSG_BUF_STATUS != ErrorCode::OK)
  {
    tx_lock_.store(0U, std::memory_order_release);
    return MSG_BUF_STATUS;
  }

  mcan_config_t config{};
  mcan_get_default_config(can_, &config);

  config.enable_canfd = false;
  config.mode = detail::ConvertMcanMode(cfg.mode);
  config.disable_auto_retransmission = cfg.mode.one_shot;
  config.interrupt_mask = HPM_CAN_INTERRUPT_MASK;
  config.txbuf_trans_interrupt_mask = ~0UL;

  if (use_low_level)
  {
    config.use_lowlevel_timing_setting = true;
    detail::ApplyLowLevelTiming(cfg.bit_timing, config.can_timing);
  }
  else
  {
    config.use_lowlevel_timing_setting = false;
    config.baudrate = cfg.bitrate;
    config.can20_samplepoint_min = detail::SamplePointToHpmRange(cfg.sample_point, false);
    config.can20_samplepoint_max = detail::SamplePointToHpmRange(cfg.sample_point, true);
  }

  const ErrorCode RESULT = detail::ConvertMcanStatus(mcan_init(can_, &config, clock_hz));
  if (RESULT == ErrorCode::OK)
  {
    mcan_disable_standby_pin(can_);
    configured_.store(true, std::memory_order_release);
    EnableCanInterrupts();
    (void)EnableInterrupt();
  }
  else
  {
    DisableCanInterrupts();
    mcan_clear_interrupt_flags(can_, 0xFFFFFFFFUL);
    mcan_deinit(can_);
  }
  tx_pend_.store(0U, std::memory_order_release);
  tx_lock_.store(0U, std::memory_order_release);
  if (RESULT == ErrorCode::OK)
  {
    TxService();
  }
  return RESULT;
}

uint32_t HPMCAN::GetClockFreq() const { return clock_get_frequency(clock_); }

ErrorCode HPMCAN::SetMessageBuffer(void* msg_buf, uint32_t msg_buf_size)
{
#if !defined(MCAN_SOC_MSG_BUF_IN_AHB_RAM) || (MCAN_SOC_MSG_BUF_IN_AHB_RAM != 1)
  UNUSED(msg_buf);
  UNUSED(msg_buf_size);
  return ErrorCode::NOT_SUPPORT;
#else
  if (msg_buf == nullptr || !detail::HasMcanDefaultMessageBufferCapacity(msg_buf_size))
  {
    return ErrorCode::ARG_ERR;
  }

  uint32_t expected = 0U;
  if (!tx_lock_.compare_exchange_strong(expected, 1U, std::memory_order_acquire,
                                        std::memory_order_relaxed))
  {
    return ErrorCode::BUSY;
  }
  if (configured_.load(std::memory_order_acquire))
  {
    tx_lock_.store(0U, std::memory_order_release);
    return ErrorCode::BUSY;
  }

  msg_buf_ = msg_buf;
  msg_buf_size_ = msg_buf_size;
  tx_lock_.store(0U, std::memory_order_release);
  return ErrorCode::OK;
#endif
}

void HPMCAN::BuildTxFrame(const ClassicPack& pack, mcan_tx_frame_t& frame)
{
  detail::BuildMcanClassicTxFrame(pack, frame);
}

void HPMCAN::BuildRxPack(const mcan_rx_message_t& frame, ClassicPack& pack)
{
  detail::BuildMcanClassicRxPack(frame, pack);
}

void HPMCAN::TxService()
{
  if (!configured_.load(std::memory_order_acquire) || can_ == nullptr)
  {
    return;
  }

  tx_pend_.store(1u, std::memory_order_release);

  uint32_t expected = 0u;
  if (!tx_lock_.compare_exchange_strong(expected, 1u, std::memory_order_acquire,
                                        std::memory_order_relaxed))
  {
    return;
  }

  for (;;)
  {
    tx_pend_.store(0u, std::memory_order_release);

    while (detail::HardwareTxQueueEmptySize(can_) != 0U)
    {
      ClassicPack pack{};
      if (tx_retry_valid_)
      {
        pack = tx_retry_pack_;
      }
      else if (tx_queue_.Pop(pack) != ErrorCode::OK)
      {
        break;
      }

      mcan_tx_frame_t frame{};
      BuildTxFrame(pack, frame);

      uint32_t fifo_index = 0u;
      const hpm_stat_t STATUS =
          mcan_transmit_via_txfifo_nonblocking(can_, &frame, &fifo_index);
      if (STATUS != status_success)
      {
        tx_retry_pack_ = pack;
        tx_retry_valid_ = true;
        break;
      }

      tx_retry_valid_ = false;
    }

    tx_lock_.store(0u, std::memory_order_release);

    if (tx_pend_.load(std::memory_order_acquire) == 0u)
    {
      return;
    }

    expected = 0u;
    if (!tx_lock_.compare_exchange_strong(expected, 1u, std::memory_order_acquire,
                                          std::memory_order_relaxed))
    {
      return;
    }
  }
}

ErrorCode HPMCAN::AddMessage(const ClassicPack& pack)
{
  if (can_ == nullptr)
  {
    return ErrorCode::PTR_NULL;
  }
  if (!configured_.load(std::memory_order_acquire))
  {
    return ErrorCode::INIT_ERR;
  }
  if (!detail::IsValidMcanClassicPack(pack))
  {
    return ErrorCode::ARG_ERR;
  }

  if (tx_queue_.Push(pack) != ErrorCode::OK)
  {
    TxService();
    if (tx_queue_.Push(pack) != ErrorCode::OK)
    {
      return ErrorCode::FULL;
    }
  }

  TxService();
  return ErrorCode::OK;
}

void HPMCAN::ProcessRxFifo(uint32_t fifo_index)
{
  if (can_ == nullptr)
  {
    return;
  }

  detail::DrainMcanRxFifo(
      can_, fifo_index,
      [this](const mcan_rx_message_t& frame)
      {
        ClassicPack pack{};
        BuildRxPack(frame, pack);
        OnMessage(pack, true);
      },
      [this]()
      {
        ClassicPack pack{};
        pack.type = Type::ERROR;
        pack.id = FromErrorID(ErrorID::CAN_ERROR_ID_OTHER);
        OnMessage(pack, true);
      });
}

void HPMCAN::ProcessTx() { TxService(); }

void HPMCAN::ProcessError()
{
  if (can_ == nullptr)
  {
    return;
  }

  mcan_protocol_status_t protocol{};
  if (mcan_get_protocol_status(can_, &protocol) != status_success)
  {
    return;
  }

  ClassicPack pack{};
  pack.type = Type::ERROR;

  if (protocol.in_bus_off_state)
  {
    // Bus-off sets CCCR.INIT; clear it so MCAN can run the recovery sequence.
    mcan_enter_normal_mode(can_);
    pack.id = FromErrorID(ErrorID::CAN_ERROR_ID_BUS_OFF);
  }
  else if (protocol.in_error_passive_state)
  {
    pack.id = FromErrorID(ErrorID::CAN_ERROR_ID_ERROR_PASSIVE);
  }
  else if (protocol.in_warning_state)
  {
    pack.id = FromErrorID(ErrorID::CAN_ERROR_ID_ERROR_WARNING);
  }
  else
  {
    pack.id = FromErrorID(detail::ConvertMcanProtocolError(protocol.last_error_code,
                                                           ErrorID::CAN_ERROR_ID_OTHER));
  }

  OnMessage(pack, true);
}

void HPMCAN::ProcessInterrupt()
{
  detail::ProcessMcanInterrupt(
      can_, configured_.load(std::memory_order_acquire), true,
      [this](uint32_t fifo_index, uint32_t, bool) { ProcessRxFifo(fifo_index); },
      [this](bool)
      {
        ClassicPack pack{};
        pack.type = Type::ERROR;
        pack.id = FromErrorID(ErrorID::CAN_ERROR_ID_OTHER);
        OnMessage(pack, true);
      },
      [this]() { ProcessTx(); }, [this](uint32_t, bool) { ProcessError(); });
}

void HPMCAN::OnInterrupt(uint8_t index)
{
  detail::ProcessMcanRegisteredInterrupt(index, true);
}

void HPMCAN::EnableCanInterrupts()
{
  if (can_ == nullptr)
  {
    return;
  }
  mcan_enable_interrupts(can_, HPM_CAN_INTERRUPT_MASK);
}

void HPMCAN::DisableCanInterrupts()
{
  if (can_ == nullptr)
  {
    return;
  }
  mcan_disable_interrupts(can_, HPM_CAN_INTERRUPT_MASK);
}

ErrorCode HPMCAN::ApplyMessageBuffer()
{
  return detail::ApplyMcanMessageBuffer(can_, msg_buf_, msg_buf_size_);
}

ErrorCode HPMCAN::EnableInterrupt()
{
  if (irq_ == INVALID_IRQ)
  {
    return ErrorCode::OK;
  }

  intc_m_enable_irq_with_priority(irq_, 1);
  return ErrorCode::OK;
}

ErrorCode HPMCAN::DisableInterrupt()
{
  if (irq_ == INVALID_IRQ)
  {
    return ErrorCode::OK;
  }

  intc_m_disable_irq(irq_);
  return ErrorCode::OK;
}

ErrorCode HPMCAN::GetErrorState(CAN::ErrorState& state) const
{
  if (can_ == nullptr)
  {
    return ErrorCode::PTR_NULL;
  }
  if (!configured_.load(std::memory_order_acquire))
  {
    return ErrorCode::INIT_ERR;
  }

  return detail::ReadMcanErrorState(can_, state);
}

#endif
