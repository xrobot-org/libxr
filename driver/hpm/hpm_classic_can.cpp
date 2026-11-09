#include "hpm_classic_can.hpp"

#if !defined(MCAN_SOC_MAX_COUNT) || (MCAN_SOC_MAX_COUNT == 0)

#include <cstring>

#if LIBXR_HPM_CLASSIC_CAN_SUPPORTED

#include "hpm_interrupt.h"

using namespace LibXR;

HPMClassicCAN* HPMClassicCAN::instance_map_[HPMClassicCAN::MAX_INSTANCES] = {};

HPMClassicCAN::HPMClassicCAN(CAN_Type* can, clock_name_t clock, uint8_t index,
                             uint32_t irq, bool auto_enable_irq, uint32_t tx_pool_size)
    : can_(can),
      clock_(clock),
      index_(index),
      irq_(irq),
      auto_enable_irq_(auto_enable_irq),
      tx_pool_(tx_pool_size)
{
  if (can_ == nullptr || index_ >= MAX_INSTANCES || instance_map_[index_] != nullptr)
  {
    ASSERT(false);
    can_ = nullptr;
    index_ = MAX_INSTANCES;
    return;
  }

  instance_map_[index_] = this;
}

ErrorCode HPMClassicCAN::ConvertStatus(hpm_stat_t status)
{
  switch (status)
  {
    case status_success:
      return ErrorCode::OK;
    case status_timeout:
      return ErrorCode::TIMEOUT;
    case status_invalid_argument:
    case status_can_filter_index_invalid:
    case status_can_filter_num_invalid:
    case status_can_invalid_bit_timing:
      return ErrorCode::ARG_ERR;
    case status_can_tx_fifo_full:
      return ErrorCode::FULL;
    case status_can_bit_error:
    case status_can_form_error:
    case status_can_stuff_error:
    case status_can_ack_error:
    case status_can_crc_error:
    case status_can_other_error:
      return ErrorCode::CHECK_ERR;
    case status_can_timestamping_disabled:
      return ErrorCode::STATE_ERR;
    default:
      return ErrorCode::FAILED;
  }
}

ErrorCode HPMClassicCAN::ValidateConfig(const CAN::Configuration& cfg)
{
  if (cfg.mode.triple_sampling)
  {
    return ErrorCode::NOT_SUPPORT;
  }
  return ErrorCode::OK;
}

void HPMClassicCAN::EmitErrorFrame(CAN::ErrorID error_id, bool in_isr)
{
  ClassicPack pack{};
  pack.id = FromErrorID(error_id);
  pack.type = Type::ERROR;
  pack.dlc = 0U;
  OnMessage(pack, in_isr);
}

void HPMClassicCAN::Shutdown()
{
  if (auto_enable_irq_ && irq_ != INVALID_IRQ)
  {
    intc_m_disable_irq(irq_);
  }
  if (can_ != nullptr)
  {
    can_disable_tx_rx_irq(can_, INTERRUPT_MASK);
    can_disable_error_irq(can_, CAN_ERROR_INTERRUPT_MASK);
    can_abort_message_transmit(can_);
    can_clear_tx_rx_flags(can_, 0xFFU);
    can_clear_error_interrupt_flags(can_, 0xFFU);
    can_deinit(can_);
  }
  configured_ = false;
  tx_lock_.store(0U, std::memory_order_release);
  tx_pend_.store(0U, std::memory_order_release);
}

bool HPMClassicCAN::HasLowLevelTiming(const CAN::BitTiming& timing)
{
  return timing.brp != 0U && timing.phase_seg1 != 0U && timing.phase_seg2 != 0U &&
         timing.sjw != 0U;
}

uint16_t HPMClassicCAN::SamplePointToPermille(float sample_point)
{
  if (sample_point <= 0.0f)
  {
    return 0U;
  }
  if (sample_point > 1.0f)
  {
    sample_point = 1.0f;
  }
  return static_cast<uint16_t>(sample_point * 1000.0f);
}

can_node_mode_t HPMClassicCAN::ConvertMode(const CAN::Mode& mode)
{
  if (mode.loopback && mode.listen_only)
  {
    return can_mode_loopback_external;
  }
  if (mode.loopback)
  {
    return can_mode_loopback_internal;
  }
  if (mode.listen_only)
  {
    return can_mode_listen_only;
  }
  return can_mode_normal;
}

void HPMClassicCAN::ApplyLowLevelTiming(const CAN::BitTiming& src,
                                        can_bit_timing_param_t& dst)
{
  dst.prescaler = static_cast<uint16_t>(src.brp);
  dst.num_seg1 = static_cast<uint16_t>(src.prop_seg + src.phase_seg1);
  dst.num_seg2 = static_cast<uint16_t>(src.phase_seg2);
  dst.num_sjw = static_cast<uint16_t>(src.sjw);
}

void HPMClassicCAN::BuildTxFrame(const ClassicPack& pack, can_transmit_buf_t& frame)
{
  std::memset(&frame, 0, sizeof(frame));
  frame.extend_id =
      (pack.type == Type::EXTENDED || pack.type == Type::REMOTE_EXTENDED) ? 1U : 0U;
  frame.remote_frame =
      (pack.type == Type::REMOTE_STANDARD || pack.type == Type::REMOTE_EXTENDED) ? 1U
                                                                                 : 0U;
  frame.id = frame.extend_id ? (pack.id & 0x1FFFFFFFUL) : (pack.id & 0x7FFUL);
  frame.dlc = pack.dlc > 8U ? 8U : pack.dlc;
  frame.canfd_frame = 0U;
  frame.bitrate_switch = 0U;
  frame.transmit_timestamp_enable = 0U;
  if (frame.remote_frame == 0U && frame.dlc > 0U)
  {
    std::memcpy(frame.data, pack.data, frame.dlc);
  }
}

bool HPMClassicCAN::BuildRxPack(const can_receive_buf_t& frame, ClassicPack& pack)
{
  std::memset(&pack, 0, sizeof(pack));
  if (frame.canfd_frame != 0U)
  {
    return false;
  }

  pack.id = frame.extend_id ? (frame.id & 0x1FFFFFFFUL) : (frame.id & 0x7FFUL);
  if (frame.extend_id)
  {
    pack.type = frame.remote_frame ? Type::REMOTE_EXTENDED : Type::EXTENDED;
  }
  else
  {
    pack.type = frame.remote_frame ? Type::REMOTE_STANDARD : Type::STANDARD;
  }

  pack.dlc = frame.dlc > 8U ? 8U : static_cast<uint8_t>(frame.dlc);
  if (!frame.remote_frame && pack.dlc > 0U)
  {
    std::memcpy(pack.data, frame.data, pack.dlc);
  }
  return true;
}

CAN::ErrorID HPMClassicCAN::ConvertProtocolError(uint8_t error_kind)
{
  switch (error_kind)
  {
    case CAN_KIND_OF_ERROR_STUFF_ERROR:
      return ErrorID::CAN_ERROR_ID_STUFF;
    case CAN_KIND_OF_ERROR_FORM_ERROR:
      return ErrorID::CAN_ERROR_ID_FORM;
    case CAN_KIND_OF_ERROR_ACK_ERROR:
      return ErrorID::CAN_ERROR_ID_ACK;
    case CAN_KIND_OF_ERROR_CRC_ERROR:
      return ErrorID::CAN_ERROR_ID_CRC;
    case CAN_KIND_OF_ERROR_BIT_ERROR:
      return ErrorID::CAN_ERROR_ID_PROTOCOL;
    case CAN_KIND_OF_ERROR_BUS_OFF:
      return ErrorID::CAN_ERROR_ID_BUS_OFF;
    case CAN_KIND_OF_ERROR_NO_ERROR:
    case CAN_KIND_OF_ERROR_OTHER_ERROR:
    default:
      return ErrorID::CAN_ERROR_ID_OTHER;
  }
}

ErrorCode HPMClassicCAN::SetConfig(const CAN::Configuration& cfg)
{
  if (can_ == nullptr)
  {
    ASSERT(false);
    return ErrorCode::ARG_ERR;
  }
  ErrorCode cfg_ans = ValidateConfig(cfg);
  if (cfg_ans != ErrorCode::OK)
  {
    return cfg_ans;
  }

  Shutdown();

  clock_add_to_group(clock_, 0);
  const uint32_t clock_hz = clock_get_frequency(clock_);
  if (clock_hz == 0U)
  {
    return ErrorCode::INIT_ERR;
  }

  can_config_t config{};
  ErrorCode ans = ConvertStatus(can_get_default_config(&config));
  if (ans != ErrorCode::OK)
  {
    return ans;
  }

  config.enable_canfd = false;
  config.mode = ConvertMode(cfg.mode);
  config.disable_ptb_retransmission = cfg.mode.one_shot;
  config.disable_stb_retransmission = cfg.mode.one_shot;
  config.enable_self_ack = cfg.mode.loopback;
  config.enable_tx_buffer_priority_mode = false;
  config.irq_txrx_enable_mask = INTERRUPT_MASK;
  config.irq_error_enable_mask = CAN_ERROR_INTERRUPT_MASK;

  if (HasLowLevelTiming(cfg.bit_timing))
  {
    config.use_lowlevel_timing_setting = true;
    ApplyLowLevelTiming(cfg.bit_timing, config.can_timing);
  }
  else
  {
    if (cfg.bitrate == 0U)
    {
      ASSERT(false);
      return ErrorCode::ARG_ERR;
    }
    config.use_lowlevel_timing_setting = false;
    config.baudrate = cfg.bitrate;
    const uint16_t sample_point = SamplePointToPermille(cfg.sample_point);
    if (sample_point != 0U)
    {
      config.can20_samplepoint_min = sample_point;
      config.can20_samplepoint_max = sample_point;
    }
  }

  can_filter_config_t filter{};
  filter.index = 0;
  filter.id_mode = can_filter_id_mode_both_frames;
  filter.enable = true;
  filter.code = 0U;
  filter.mask = 0x3FFFFFFFUL;
  config.filter_list_num = 1;
  config.filter_list = &filter;

  ans = ConvertStatus(can_init(can_, &config, clock_hz));
  if (ans != ErrorCode::OK)
  {
    return ans;
  }

  configured_ = true;
  tx_lock_.store(0U, std::memory_order_release);
  tx_pend_.store(0U, std::memory_order_release);
  if (auto_enable_irq_ && irq_ != INVALID_IRQ)
  {
    can_clear_tx_rx_flags(can_, 0xFFU);
    can_clear_error_interrupt_flags(can_, 0xFFU);
    can_enable_tx_rx_irq(can_, INTERRUPT_MASK);
    can_enable_error_irq(can_, CAN_ERROR_INTERRUPT_MASK);
    intc_m_enable_irq_with_priority(irq_, 1);
  }
  return ErrorCode::OK;
}

uint32_t HPMClassicCAN::GetClockFreq() const { return clock_get_frequency(clock_); }

ErrorCode HPMClassicCAN::AddMessage(const ClassicPack& pack)
{
  if (can_ == nullptr)
  {
    return ErrorCode::PTR_NULL;
  }
  if (!configured_)
  {
    return ErrorCode::INIT_ERR;
  }
  if (pack.type == Type::ERROR || pack.type == Type::TYPE_NUM || pack.dlc > 8U)
  {
    return ErrorCode::ARG_ERR;
  }

  if (tx_pool_.Put(pack) != ErrorCode::OK)
  {
    TxService();
    if (tx_pool_.Put(pack) != ErrorCode::OK)
    {
      return ErrorCode::FULL;
    }
  }

  TxService();
  return ErrorCode::OK;
}

ErrorCode HPMClassicCAN::GetErrorState(CAN::ErrorState& state) const
{
  state = {};
  if (can_ == nullptr)
  {
    return ErrorCode::PTR_NULL;
  }

  state.tx_error_counter = can_get_transmit_error_count(can_);
  state.rx_error_counter = can_get_receive_error_count(can_);
  state.bus_off = can_is_in_bus_off_mode(can_);

  const uint8_t flags = can_get_error_interrupt_flags(can_);
  state.error_passive = (flags & CAN_ERROR_PASSIVE_MODE_ACTIVE_FLAG) != 0U;
  state.error_warning = (flags & CAN_ERROR_WARNING_LIMIT_FLAG) != 0U;
  return ErrorCode::OK;
}

void HPMClassicCAN::ProcessRx(bool in_isr)
{
  if (!configured_)
  {
    return;
  }
  ProcessRxBuffer(in_isr);
}

void HPMClassicCAN::ProcessInterrupt(bool in_isr)
{
  if (!configured_ || can_ == nullptr)
  {
    return;
  }

  const uint8_t flags = can_get_tx_rx_flags(can_);
  const uint8_t err_flags = can_get_error_interrupt_flags(can_);
  if (flags == 0U && err_flags == 0U)
  {
    return;
  }
  if (flags != 0U)
  {
    can_clear_tx_rx_flags(can_, flags);
  }

  if ((flags & RX_INTERRUPT_MASK) != 0U)
  {
    ProcessRx(in_isr);
  }
  if ((flags & (CAN_EVENT_RX_BUF_OVERRUN | CAN_EVENT_RX_BUF_FULL)) != 0U)
  {
    EmitErrorFrame(ErrorID::CAN_ERROR_ID_OTHER, in_isr);
  }
  if ((flags & TX_INTERRUPT_MASK) != 0U)
  {
    TxService();
  }
  if (((flags & ERROR_INTERRUPT_MASK) != 0U) || err_flags != 0U)
  {
    ProcessError(in_isr);
    TxService();
  }
}

void HPMClassicCAN::OnInterrupt(uint8_t index)
{
  if (index >= MAX_INSTANCES)
  {
    return;
  }
  if (auto* can = instance_map_[index])
  {
    can->ProcessInterrupt(true);
  }
}

void HPMClassicCAN::TxService()
{
  if (!configured_ || can_ == nullptr)
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

    while (!can_is_secondary_transmit_buffer_full(can_))
    {
      ClassicPack pack{};
      if (tx_pool_.Get(pack) != ErrorCode::OK)
      {
        break;
      }

      can_transmit_buf_t frame{};
      BuildTxFrame(pack, frame);
      const hpm_stat_t status = can_send_message_nonblocking(can_, &frame);
      if (status != status_success)
      {
        if (tx_pool_.Put(pack) != ErrorCode::OK)
        {
          ASSERT(false);
        }
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

void HPMClassicCAN::ProcessRxBuffer(bool in_isr)
{
  while (can_is_data_available_in_receive_buffer(can_))
  {
    can_receive_buf_t rx_frame{};
    const hpm_stat_t status = can_read_received_message(can_, &rx_frame);
    if (status != status_success)
    {
      EmitErrorFrame(ConvertProtocolError(can_get_last_error_kind(can_)), in_isr);
      break;
    }

    ClassicPack pack{};
    if (BuildRxPack(rx_frame, pack))
    {
      OnMessage(pack, in_isr);
    }
  }
}

void HPMClassicCAN::ProcessError(bool in_isr)
{
  const uint8_t err_flags = can_get_error_interrupt_flags(can_);
  if (err_flags != 0U)
  {
    can_clear_error_interrupt_flags(can_, err_flags);
  }

  if (can_is_in_bus_off_mode(can_))
  {
    EmitErrorFrame(ErrorID::CAN_ERROR_ID_BUS_OFF, in_isr);
  }
  else if ((err_flags & CAN_ERROR_PASSIVE_MODE_ACTIVE_FLAG) != 0U)
  {
    EmitErrorFrame(ErrorID::CAN_ERROR_ID_ERROR_PASSIVE, in_isr);
  }
  else if ((err_flags & CAN_ERROR_WARNING_LIMIT_FLAG) != 0U)
  {
    EmitErrorFrame(ErrorID::CAN_ERROR_ID_ERROR_WARNING, in_isr);
  }
  else
  {
    EmitErrorFrame(ConvertProtocolError(can_get_last_error_kind(can_)), in_isr);
  }
}

#endif

#endif
