#include "hpm_canfd.hpp"

#if LIBXR_HPM_MCAN_SUPPORTED

#include <cstring>

using namespace LibXR;

namespace
{
constexpr uint8_t kCanFdDlc12Bytes = 9U;
constexpr uint8_t kCanFdDlc16Bytes = 10U;
constexpr uint8_t kCanFdDlc20Bytes = 11U;
constexpr uint8_t kCanFdDlc24Bytes = 12U;
constexpr uint8_t kCanFdDlc32Bytes = 13U;
constexpr uint8_t kCanFdDlc48Bytes = 14U;
constexpr uint8_t kCanFdDlc64Bytes = 15U;
}  // namespace

uint8_t HPMCANFD::BytesToDlc(uint8_t bytes)
{
  if (bytes <= 8U)
  {
    return bytes;
  }
  if (bytes <= 12U)
  {
    return kCanFdDlc12Bytes;
  }
  if (bytes <= 16U)
  {
    return kCanFdDlc16Bytes;
  }
  if (bytes <= 20U)
  {
    return kCanFdDlc20Bytes;
  }
  if (bytes <= 24U)
  {
    return kCanFdDlc24Bytes;
  }
  if (bytes <= 32U)
  {
    return kCanFdDlc32Bytes;
  }
  if (bytes <= 48U)
  {
    return kCanFdDlc48Bytes;
  }
  return kCanFdDlc64Bytes;
}

uint8_t HPMCANFD::DlcToBytes(uint8_t dlc) { return mcan_get_message_size_from_dlc(dlc); }

void HPMCANFD::BuildTxFrame(const ClassicPack& pack, mcan_tx_frame_t& frame)
{
  detail::BuildMcanClassicTxFrame(pack, frame);
}

void HPMCANFD::BuildTxFrame(const FDPack& pack, mcan_tx_frame_t& frame,
                            bool bitrate_switch, bool error_state_indicator)
{
  std::memset(&frame, 0, sizeof(frame));
  const uint8_t payload_size = pack.len <= 64U ? pack.len : 64U;
  frame.use_ext_id = (pack.type == Type::EXTENDED) ? 1U : 0U;
  frame.rtr = 0U;
  if (frame.use_ext_id != 0U)
  {
    frame.ext_id = pack.id & 0x1FFFFFFFUL;
  }
  else
  {
    frame.std_id = pack.id & 0x7FFUL;
  }
  frame.dlc = BytesToDlc(payload_size);
  frame.canfd_frame = 1U;
  frame.bitrate_switch = bitrate_switch ? 1U : 0U;
  frame.error_state_indicator = error_state_indicator ? 1U : 0U;
  if (payload_size > 0U)
  {
    std::memcpy(frame.data_8, pack.data, payload_size);
  }
}

bool HPMCANFD::BuildRxPack(const mcan_rx_message_t& frame, ClassicPack& pack)
{
  if (frame.canfd_frame != 0U)
  {
    return false;
  }

  detail::BuildMcanClassicRxPack(frame, pack);
  return true;
}

bool HPMCANFD::BuildRxPack(const mcan_rx_message_t& frame, FDPack& pack)
{
  std::memset(&pack, 0, sizeof(pack));
  if (frame.canfd_frame == 0U || frame.rtr != 0U)
  {
    return false;
  }

  if (frame.use_ext_id != 0U)
  {
    pack.id = frame.ext_id;
    pack.type = Type::EXTENDED;
  }
  else
  {
    pack.id = frame.std_id;
    pack.type = Type::STANDARD;
  }

  pack.len = DlcToBytes(static_cast<uint8_t>(frame.dlc));
  if (pack.len > 64U)
  {
    pack.len = 64U;
  }
  if (pack.len > 0U)
  {
    std::memcpy(pack.data, frame.data_8, pack.len);
  }
  return true;
}

void HPMCANFD::EmitErrorFrame(CAN::ErrorID error_id, bool in_isr)
{
  ClassicPack pack{};
  pack.id = FromErrorID(error_id);
  pack.type = Type::ERROR;
  pack.dlc = 0U;
  OnMessage(pack, in_isr);
}

HPMCANFD::HPMCANFD(MCAN_Type* can, clock_name_t clock, uint8_t index, uint32_t irq,
                   bool auto_enable_irq, uint32_t queue_size, void* msg_buf,
                   uint32_t msg_buf_size)
    : can_(can),
      clock_(clock),
      index_(detail::GetMcanInstanceIndex(can)),
      irq_(irq),
      auto_enable_irq_(auto_enable_irq),
      msg_buf_(msg_buf),
      msg_buf_size_(msg_buf_size),
      tx_queue_(queue_size),
      tx_fd_queue_(queue_size)
{
  if (can_ == nullptr || index_ >= MAX_INSTANCES || index_ != index)
  {
    ASSERT(false);
    can_ = nullptr;
    index_ = MAX_INSTANCES;
    return;
  }

  if (!detail::RegisterMcanOwner(
          index_, this, detail::McanOwnerKind::FD_CAN, [](void* owner, bool in_isr)
          { static_cast<HPMCANFD*>(owner)->ProcessInterrupt(in_isr); }))
  {
    ASSERT(false);
    can_ = nullptr;
    index_ = MAX_INSTANCES;
  }
}

HPMCANFD::~HPMCANFD()
{
  Shutdown();
  detail::UnregisterMcanOwner(index_, this);
}

void HPMCANFD::Shutdown()
{
  configured_.store(false, std::memory_order_release);
  if (clock_ready_)
  {
    detail::ShutdownMcan(can_, irq_, auto_enable_irq_,
                         detail::MCAN_DRIVER_INTERRUPT_MASK);
  }
  fd_enabled_.store(false, std::memory_order_release);
  brs_enabled_.store(false, std::memory_order_release);
  esi_enabled_.store(false, std::memory_order_release);
  tx_pend_.store(0U, std::memory_order_release);
}

ErrorCode HPMCANFD::SetMessageBuffer(void* msg_buf, uint32_t msg_buf_size)
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

ErrorCode HPMCANFD::ApplyMessageBuffer()
{
  return detail::ApplyMcanMessageBuffer(can_, msg_buf_, msg_buf_size_);
}

ErrorCode HPMCANFD::SetConfig(const CAN::Configuration& cfg)
{
  FDCAN::Configuration fd_cfg{};
  fd_cfg.bitrate = cfg.bitrate;
  fd_cfg.sample_point = cfg.sample_point;
  fd_cfg.bit_timing = cfg.bit_timing;
  fd_cfg.mode = cfg.mode;
  return SetConfig(fd_cfg);
}

ErrorCode HPMCANFD::SetConfig(const FDCAN::Configuration& cfg)
{
  if (can_ == nullptr)
  {
    return ErrorCode::PTR_NULL;
  }
  if (cfg.mode.triple_sampling)
  {
    return ErrorCode::NOT_SUPPORT;
  }
  if (cfg.mode.loopback && cfg.mode.listen_only)
  {
    return ErrorCode::NOT_SUPPORT;
  }
  if (cfg.fd_mode.esi && !cfg.fd_mode.fd_enabled)
  {
    return ErrorCode::ARG_ERR;
  }
  if (cfg.fd_mode.brs && !cfg.fd_mode.fd_enabled)
  {
    return ErrorCode::ARG_ERR;
  }
  if (!detail::IsValidMcanSamplePoint(cfg.sample_point) ||
      !detail::IsValidMcanSamplePoint(cfg.data_sample_point))
  {
    return ErrorCode::ARG_ERR;
  }

  const bool nominal_timing_set = detail::HasAnyLowLevelTiming(cfg.bit_timing);
  const bool data_timing_set = detail::HasAnyLowLevelTiming(cfg.data_timing);
  const bool nominal_low_level = detail::HasLowLevelTiming(cfg.bit_timing);
  const bool data_low_level = detail::HasLowLevelTiming(cfg.data_timing);

  if ((nominal_timing_set && !nominal_low_level) || (data_timing_set && !data_low_level))
  {
    return ErrorCode::ARG_ERR;
  }
  if (!cfg.fd_mode.fd_enabled &&
      (data_timing_set || cfg.data_bitrate != 0U || cfg.data_sample_point != 0.0f))
  {
    return ErrorCode::ARG_ERR;
  }
  if ((nominal_low_level && !detail::CanApplyMcanLowLevelTiming(cfg.bit_timing)) ||
      (data_low_level && !detail::CanApplyMcanLowLevelTiming(cfg.data_timing)))
  {
    return ErrorCode::ARG_ERR;
  }

  if (nominal_low_level || data_low_level)
  {
    if (!nominal_low_level || (cfg.fd_mode.fd_enabled && !data_low_level &&
                               (cfg.data_bitrate != 0U || cfg.data_sample_point > 0.0f)))
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

  Shutdown();

  const ErrorCode msg_buf_status = ApplyMessageBuffer();
  if (msg_buf_status != ErrorCode::OK)
  {
    tx_lock_.store(0U, std::memory_order_release);
    return msg_buf_status;
  }

  mcan_config_t config{};
  mcan_get_default_config(can_, &config);
  detail::PrepareMcanCommonConfig(can_, config, cfg.fd_mode.fd_enabled);
  config.mode = detail::ConvertMcanMode(cfg.mode);
  config.disable_auto_retransmission = cfg.mode.one_shot;
  config.enable_non_iso_mode = false;
  config.enable_tdc = cfg.fd_mode.fd_enabled && cfg.fd_mode.brs;

  if (nominal_low_level || data_low_level)
  {
    config.use_lowlevel_timing_setting = true;
    detail::ApplyLowLevelTiming(cfg.bit_timing, config.can_timing);
    if (cfg.fd_mode.fd_enabled)
    {
      if (data_low_level)
      {
        detail::ApplyLowLevelTiming(cfg.data_timing, config.canfd_timing);
      }
      else
      {
        config.canfd_timing = config.can_timing;
      }
      config.canfd_timing.enable_tdc = config.enable_tdc;
    }
  }
  else
  {
    config.use_lowlevel_timing_setting = false;
    config.baudrate = cfg.bitrate;
    const uint16_t sample_point = detail::SamplePointToPermille(cfg.sample_point);
    if (sample_point != 0U)
    {
      config.can20_samplepoint_min = sample_point;
      config.can20_samplepoint_max = sample_point;
    }

    if (cfg.fd_mode.fd_enabled)
    {
      config.baudrate_fd = (cfg.data_bitrate != 0U) ? cfg.data_bitrate : cfg.bitrate;
      const uint16_t data_sample_point =
          detail::SamplePointToPermille(cfg.data_sample_point);
      if (data_sample_point != 0U)
      {
        config.canfd_samplepoint_min = data_sample_point;
        config.canfd_samplepoint_max = data_sample_point;
      }
    }
  }

  detail::PrepareMcanAcceptAllFilters(config);

  ErrorCode ans = detail::ConvertMcanStatus(mcan_init(can_, &config, clock_hz));
  if (ans != ErrorCode::OK)
  {
    Shutdown();
    tx_lock_.store(0U, std::memory_order_release);
    return ans;
  }

  mcan_disable_standby_pin(can_);
  fd_enabled_.store(cfg.fd_mode.fd_enabled, std::memory_order_release);
  brs_enabled_.store(cfg.fd_mode.brs, std::memory_order_release);
  esi_enabled_.store(cfg.fd_mode.esi, std::memory_order_release);
  configured_.store(true, std::memory_order_release);
  tx_pend_.store(0U, std::memory_order_release);
  detail::EnableMcanInterrupts(can_, irq_, auto_enable_irq_,
                               detail::MCAN_DRIVER_INTERRUPT_MASK);
  tx_lock_.store(0U, std::memory_order_release);
  TxService();
  return ErrorCode::OK;
}

uint32_t HPMCANFD::GetClockFreq() const { return clock_get_frequency(clock_); }

ErrorCode HPMCANFD::AddMessage(const ClassicPack& pack)
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

ErrorCode HPMCANFD::AddMessage(const FDPack& pack)
{
  if (can_ == nullptr)
  {
    return ErrorCode::PTR_NULL;
  }
  if (!configured_.load(std::memory_order_acquire))
  {
    return ErrorCode::INIT_ERR;
  }
  if (!fd_enabled_.load(std::memory_order_acquire))
  {
    return ErrorCode::NOT_SUPPORT;
  }
  if (!detail::IsValidMcanFdIdentifier(pack))
  {
    return ErrorCode::ARG_ERR;
  }
  if (pack.len > 64U)
  {
    return ErrorCode::ARG_ERR;
  }
  if (tx_fd_queue_.Push(pack) != ErrorCode::OK)
  {
    TxService();
    if (tx_fd_queue_.Push(pack) != ErrorCode::OK)
    {
      return ErrorCode::FULL;
    }
  }

  TxService();
  return ErrorCode::OK;
}

ErrorCode HPMCANFD::GetErrorState(CAN::ErrorState& state) const
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

size_t HPMCANFD::HardwareTxQueueEmptySize() const
{
  if (!configured_.load(std::memory_order_acquire))
  {
    return 0U;
  }
  return detail::HardwareTxQueueEmptySize(can_);
}

void HPMCANFD::ProcessRxInterrupt(uint32_t fifo, bool in_isr)
{
  if (!configured_.load(std::memory_order_acquire) || can_ == nullptr)
  {
    return;
  }

  detail::DrainMcanRxFifo(
      can_, fifo,
      [this, in_isr](const mcan_rx_message_t& frame)
      {
        rx_buff_.frame = frame;
        if (BuildRxPack(rx_buff_.frame, rx_buff_.pack_fd))
        {
          OnMessage(rx_buff_.pack_fd, in_isr);
          return;
        }

        if (BuildRxPack(rx_buff_.frame, rx_buff_.pack))
        {
          OnMessage(rx_buff_.pack, in_isr);
        }
      },
      [this, in_isr]() { EmitErrorFrame(ErrorID::CAN_ERROR_ID_OTHER, in_isr); });
}

void HPMCANFD::ProcessErrorStatusInterrupt(uint32_t error_status_its, bool in_isr)
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

  if ((error_status_its & MCAN_INT_BUS_OFF_STATUS) != 0U && protocol.in_bus_off_state)
  {
    // Bus-off sets CCCR.INIT; clear it so MCAN can run the recovery sequence.
    mcan_enter_normal_mode(can_);
    EmitErrorFrame(ErrorID::CAN_ERROR_ID_BUS_OFF, in_isr);
    return;
  }
  if ((error_status_its & MCAN_INT_ERROR_PASSIVE) != 0U &&
      protocol.in_error_passive_state)
  {
    EmitErrorFrame(ErrorID::CAN_ERROR_ID_ERROR_PASSIVE, in_isr);
    return;
  }
  if ((error_status_its & MCAN_INT_WARNING_STATUS) != 0U && protocol.in_warning_state)
  {
    EmitErrorFrame(ErrorID::CAN_ERROR_ID_ERROR_WARNING, in_isr);
    return;
  }
  if ((error_status_its &
       (MCAN_INT_PROTOCOL_ERR_IN_ARB_PHASE | MCAN_INT_PROTOCOL_ERR_IN_DATA_PHASE |
        MCAN_INT_BIT_ERROR_UNCORRECTED)) != 0U)
  {
    EmitErrorFrame(detail::ConvertMcanProtocolError(protocol.last_error_code), in_isr);
  }
}

void HPMCANFD::ProcessInterrupt(bool in_isr)
{
  detail::ProcessMcanInterrupt(
      can_, configured_.load(std::memory_order_acquire), in_isr,
      [this](uint32_t fifo_index, uint32_t, bool rx_in_isr)
      { ProcessRxInterrupt(fifo_index, rx_in_isr); }, [this](bool err_in_isr)
      { EmitErrorFrame(ErrorID::CAN_ERROR_ID_OTHER, err_in_isr); },
      [this]() { TxService(); }, [this](uint32_t error_flags, bool err_in_isr)
      { ProcessErrorStatusInterrupt(error_flags, err_in_isr); });
}

void HPMCANFD::OnInterrupt(uint8_t index)
{
  detail::ProcessMcanRegisteredInterrupt(index, true);
}

void HPMCANFD::TxService()
{
  if (!configured_.load(std::memory_order_acquire) || can_ == nullptr)
  {
    return;
  }

  tx_pend_.store(1U, std::memory_order_release);

  uint32_t expected = 0U;
  if (!tx_lock_.compare_exchange_strong(expected, 1U, std::memory_order_acquire,
                                        std::memory_order_relaxed))
  {
    return;
  }

  for (;;)
  {
    tx_pend_.store(0U, std::memory_order_release);

    while (HardwareTxQueueEmptySize() != 0U)
    {
      bool submitted = false;
      for (uint32_t attempt = 0U; attempt < 2U; ++attempt)
      {
        const bool try_fd = (attempt == 0U) ? prefer_fd_next_ : !prefer_fd_next_;
        if (try_fd)
        {
          if (!fd_enabled_.load(std::memory_order_acquire))
          {
            continue;
          }

          FDPack pack{};
          bool have_pack = false;
          if (tx_fd_retry_valid_)
          {
            pack = tx_fd_retry_pack_;
            have_pack = true;
          }
          else
          {
            have_pack = tx_fd_queue_.Pop(pack) == ErrorCode::OK;
          }
          if (!have_pack)
          {
            continue;
          }

          BuildTxFrame(pack, tx_buff_.frame, brs_enabled_.load(std::memory_order_relaxed),
                       esi_enabled_.load(std::memory_order_relaxed));
          uint32_t fifo_index = 0U;
          const hpm_stat_t status =
              mcan_transmit_via_txfifo_nonblocking(can_, &tx_buff_.frame, &fifo_index);
          UNUSED(fifo_index);
          if (status != status_success)
          {
            tx_fd_retry_pack_ = pack;
            tx_fd_retry_valid_ = true;
            break;
          }

          tx_fd_retry_valid_ = false;
          prefer_fd_next_ = false;
          submitted = true;
          break;
        }

        ClassicPack pack{};
        bool have_pack = false;
        if (tx_retry_valid_)
        {
          pack = tx_retry_pack_;
          have_pack = true;
        }
        else
        {
          have_pack = tx_queue_.Pop(pack) == ErrorCode::OK;
        }
        if (!have_pack)
        {
          continue;
        }

        BuildTxFrame(pack, tx_buff_.frame);
        uint32_t fifo_index = 0U;
        const hpm_stat_t status =
            mcan_transmit_via_txfifo_nonblocking(can_, &tx_buff_.frame, &fifo_index);
        UNUSED(fifo_index);
        if (status != status_success)
        {
          tx_retry_pack_ = pack;
          tx_retry_valid_ = true;
          break;
        }

        tx_retry_valid_ = false;
        prefer_fd_next_ = true;
        submitted = true;
        break;
      }

      if (!submitted)
      {
        break;
      }
    }

    tx_lock_.store(0U, std::memory_order_release);
    if (tx_pend_.load(std::memory_order_acquire) == 0U)
    {
      return;
    }

    expected = 0U;
    if (!tx_lock_.compare_exchange_strong(expected, 1U, std::memory_order_acquire,
                                          std::memory_order_relaxed))
    {
      return;
    }
  }
}

extern "C" void libxr_hpm_mcan_irq_handler(uint8_t index)
{
  LibXR::detail::ProcessMcanRegisteredInterrupt(index, true);
}

#endif
