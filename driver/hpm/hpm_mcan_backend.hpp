#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <utility>

#include "can.hpp"
#include "hpm_clock_drv.h"
#include "hpm_common.h"
#include "hpm_soc.h"

#if __has_include("hpm_sdk_version.h")
#include "hpm_sdk_version.h"
#endif

#if defined(MCAN_SOC_MAX_COUNT)
#define LIBXR_HPM_MCAN_INSTANCE_COUNT MCAN_SOC_MAX_COUNT
#elif defined(HPM_MCAN7)
#define LIBXR_HPM_MCAN_INSTANCE_COUNT 8U
#elif defined(HPM_MCAN6)
#define LIBXR_HPM_MCAN_INSTANCE_COUNT 7U
#elif defined(HPM_MCAN5)
#define LIBXR_HPM_MCAN_INSTANCE_COUNT 6U
#elif defined(HPM_MCAN4)
#define LIBXR_HPM_MCAN_INSTANCE_COUNT 5U
#elif defined(HPM_MCAN3)
#define LIBXR_HPM_MCAN_INSTANCE_COUNT 4U
#elif defined(HPM_MCAN2)
#define LIBXR_HPM_MCAN_INSTANCE_COUNT 3U
#elif defined(HPM_MCAN1)
#define LIBXR_HPM_MCAN_INSTANCE_COUNT 2U
#elif defined(HPM_MCAN0)
#define LIBXR_HPM_MCAN_INSTANCE_COUNT 1U
#else
#define LIBXR_HPM_MCAN_INSTANCE_COUNT 0U
#endif

#if defined(HPMSOC_HAS_HPMSDK_MCAN) && __has_include("hpm_mcan_drv.h") && \
                                                     (LIBXR_HPM_MCAN_INSTANCE_COUNT > 0)
#include "hpm_interrupt.h"
#include "hpm_mcan_drv.h"
#define LIBXR_HPM_MCAN_SUPPORTED 1
#else
#define LIBXR_HPM_MCAN_SUPPORTED 0
#endif

#if LIBXR_HPM_MCAN_SUPPORTED

namespace LibXR::detail
{

inline constexpr uint8_t MCAN_INSTANCE_COUNT = LIBXR_HPM_MCAN_INSTANCE_COUNT;

inline constexpr uint32_t MCAN_RX_INTERRUPT_MASK =
    MCAN_INT_RXFIFO0_NEW_MSG | MCAN_INT_RXFIFO1_NEW_MSG;
inline constexpr uint32_t MCAN_TX_INTERRUPT_MASK =
    MCAN_EVENT_TRANSMIT | MCAN_INT_TXFIFO_EMPTY;
inline constexpr uint32_t MCAN_ERROR_INTERRUPT_MASK =
    MCAN_EVENT_ERROR | MCAN_INT_BUS_OFF_STATUS | MCAN_INT_ERROR_PASSIVE |
    MCAN_INT_WARNING_STATUS | MCAN_INT_PROTOCOL_ERR_IN_ARB_PHASE |
    MCAN_INT_PROTOCOL_ERR_IN_DATA_PHASE | MCAN_INT_BIT_ERROR_UNCORRECTED;
inline constexpr uint32_t MCAN_INTERRUPT_MASK =
    MCAN_RX_INTERRUPT_MASK | MCAN_TX_INTERRUPT_MASK | MCAN_ERROR_INTERRUPT_MASK;
inline constexpr uint32_t MCAN_RX_FAULT_MASK =
    MCAN_INT_RXFIFO0_FULL | MCAN_INT_RXFIFO1_FULL | MCAN_INT_RXFIFO0_MSG_LOST |
    MCAN_INT_RXFIFO1_MSG_LOST | MCAN_INT_MSG_RAM_ACCESS_FAILURE;
inline constexpr uint32_t MCAN_DRIVER_INTERRUPT_MASK =
    MCAN_INTERRUPT_MASK | MCAN_RX_FAULT_MASK;
inline constexpr uint32_t MCAN_RX_FIFO0_ACTIVITY_MASK =
    MCAN_INT_RXFIFO0_NEW_MSG | MCAN_INT_RXFIFO0_FULL | MCAN_INT_RXFIFO0_MSG_LOST;
inline constexpr uint32_t MCAN_RX_FIFO1_ACTIVITY_MASK =
    MCAN_INT_RXFIFO1_NEW_MSG | MCAN_INT_RXFIFO1_FULL | MCAN_INT_RXFIFO1_MSG_LOST;

enum class McanOwnerKind : uint8_t
{
  NONE,
  CLASSIC_CAN,
  FD_CAN,
};

using McanInterruptHandler = void (*)(void* owner, bool in_isr);

struct McanOwnerSlot
{
  void* owner = nullptr;
  McanOwnerKind kind = McanOwnerKind::NONE;
  McanInterruptHandler handler = nullptr;
};

inline McanOwnerSlot mcan_owner_slots[MCAN_INSTANCE_COUNT] = {};

template <typename FrameConsumer, typename ErrorConsumer>
void DrainMcanRxFifo(MCAN_Type* can, uint32_t fifo_index, FrameConsumer&& on_frame,
                     ErrorConsumer&& on_error)
{
  while (true)
  {
    mcan_rx_message_t frame{};
    const hpm_stat_t status = mcan_read_rxfifo(can, fifo_index, &frame);
    if (status == status_mcan_rxfifo_empty)
    {
      break;
    }
    if (status != status_success)
    {
      on_error();
      break;
    }
    on_frame(frame);
  }
}

template <typename RxHandler, typename ErrorEmitter, typename TxHandler,
          typename ErrorHandler>
void ProcessMcanInterrupt(MCAN_Type* can, bool configured, bool in_isr,
                          RxHandler&& on_rx_fifo, ErrorEmitter&& emit_other_error,
                          TxHandler&& on_tx, ErrorHandler&& on_error)
{
  if (!configured || can == nullptr)
  {
    return;
  }

  const uint32_t flags = mcan_get_interrupt_flags(can);
  if (flags == 0U)
  {
    return;
  }

  mcan_clear_interrupt_flags(can, flags);

  if ((flags & MCAN_RX_FIFO0_ACTIVITY_MASK) != 0U)
  {
    on_rx_fifo(0U, flags, in_isr);
  }
  if ((flags & MCAN_RX_FIFO1_ACTIVITY_MASK) != 0U)
  {
    on_rx_fifo(1U, flags, in_isr);
  }
  if ((flags & MCAN_RX_FAULT_MASK) != 0U)
  {
    emit_other_error(in_isr);
  }
  if ((flags & MCAN_TX_INTERRUPT_MASK) != 0U)
  {
    on_tx();
  }
  if ((flags & MCAN_ERROR_INTERRUPT_MASK) != 0U)
  {
    on_error(flags & MCAN_ERROR_INTERRUPT_MASK, in_isr);
    on_tx();
  }
}

inline ErrorCode ConvertMcanStatus(hpm_stat_t status)
{
  switch (status)
  {
    case status_success:
      return ErrorCode::OK;
    case status_timeout:
    case status_mcan_timeout:
      return ErrorCode::TIMEOUT;
    case status_invalid_argument:
    case status_mcan_filter_index_out_of_range:
    case status_mcan_txbuf_index_out_of_range:
    case status_mcan_rxbuf_index_out_of_range:
    case status_mcan_ram_out_of_range:
    case status_mcan_invalid_bit_timing:
#if defined(SDK_VERSION_NUMBER) && (SDK_VERSION_NUMBER >= 0x010C00U)
    case status_mcan_txqueue_not_enabled:
#endif
      return ErrorCode::ARG_ERR;
    case status_mcan_txfifo_full:
    case status_mcan_txbuf_full:
      return ErrorCode::FULL;
    case status_mcan_rxfifo_empty:
    case status_mcan_rxbuf_empty:
    case status_mcan_tx_evt_fifo_empty:
      return ErrorCode::EMPTY;
    case status_mcan_rxfifo_full:
    case status_mcan_rxfifo0_busy:
    case status_mcan_rxfifo1_busy:
      return ErrorCode::BUSY;
    default:
      return ErrorCode::FAILED;
  }
}

inline bool HasAnyLowLevelTiming(const CAN::BitTiming& timing)
{
  return timing.brp != 0U || timing.prop_seg != 0U || timing.phase_seg1 != 0U ||
         timing.phase_seg2 != 0U || timing.sjw != 0U;
}

inline bool HasAnyLowLevelTiming(const FDCAN::DataBitTiming& timing)
{
  return timing.brp != 0U || timing.prop_seg != 0U || timing.phase_seg1 != 0U ||
         timing.phase_seg2 != 0U || timing.sjw != 0U;
}

inline bool HasLowLevelTiming(const CAN::BitTiming& timing)
{
  return timing.brp != 0U && timing.phase_seg1 != 0U && timing.phase_seg2 != 0U &&
         timing.sjw != 0U;
}

inline bool HasLowLevelTiming(const FDCAN::DataBitTiming& timing)
{
  return timing.brp != 0U && timing.phase_seg1 != 0U && timing.phase_seg2 != 0U &&
         timing.sjw != 0U;
}

template <typename Timing>
inline bool CanApplyMcanLowLevelTiming(const Timing& timing)
{
  const uint64_t seg1 = static_cast<uint64_t>(timing.prop_seg) + timing.phase_seg1;
  return HasLowLevelTiming(timing) && seg1 <= UINT16_MAX && timing.brp <= UINT16_MAX &&
         timing.phase_seg2 <= UINT16_MAX && timing.sjw <= UINT8_MAX &&
         timing.sjw <= timing.phase_seg2;
}

inline bool IsValidMcanSamplePoint(float sample_point)
{
  return sample_point >= 0.0f && sample_point < 1.0f;
}

inline uint16_t SamplePointToPermille(float sample_point)
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

inline uint16_t SamplePointToHpmRange(float sample_point, bool upper)
{
  if (sample_point <= 0.0f)
  {
    return upper ? 875U : 750U;
  }

  uint32_t sp = static_cast<uint32_t>(sample_point * 1000.0f);
  if (sp < 1U)
  {
    sp = 1U;
  }
  else if (sp > 999U)
  {
    sp = 999U;
  }

  if (upper)
  {
    const uint32_t high = sp + 25U;
    return static_cast<uint16_t>(high > 999U ? 999U : high);
  }

  return static_cast<uint16_t>(sp > 25U ? sp - 25U : 1U);
}

inline mcan_node_mode_t ConvertMcanMode(const CAN::Mode& mode)
{
  if (mode.loopback)
  {
    return mcan_mode_loopback_internal;
  }
  if (mode.listen_only)
  {
    return mcan_mode_listen_only;
  }
  return mcan_mode_normal;
}

inline void ApplyLowLevelTiming(const CAN::BitTiming& src, mcan_bit_timing_param_t& dst)
{
  dst.prescaler = static_cast<uint16_t>(src.brp);
  dst.num_seg1 = static_cast<uint16_t>(src.prop_seg + src.phase_seg1);
  dst.num_seg2 = static_cast<uint16_t>(src.phase_seg2);
  dst.num_sjw = static_cast<uint8_t>(src.sjw);
  dst.enable_tdc = false;
}

inline void ApplyLowLevelTiming(const FDCAN::DataBitTiming& src,
                                mcan_bit_timing_param_t& dst)
{
  dst.prescaler = static_cast<uint16_t>(src.brp);
  dst.num_seg1 = static_cast<uint16_t>(src.prop_seg + src.phase_seg1);
  dst.num_seg2 = static_cast<uint16_t>(src.phase_seg2);
  dst.num_sjw = static_cast<uint8_t>(src.sjw);
  dst.enable_tdc = false;
}

inline CAN::ErrorID ConvertMcanProtocolError(
    mcan_last_err_code_t code, CAN::ErrorID fallback = CAN::ErrorID::CAN_ERROR_ID_GENERIC)
{
  switch (code)
  {
    case mcan_last_error_code_stuff_error:
      return CAN::ErrorID::CAN_ERROR_ID_STUFF;
    case mcan_last_error_code_format_error:
      return CAN::ErrorID::CAN_ERROR_ID_FORM;
    case mcan_last_error_code_ack_error:
      return CAN::ErrorID::CAN_ERROR_ID_ACK;
    case mcan_last_error_code_bit1_error:
      return CAN::ErrorID::CAN_ERROR_ID_BIT1;
    case mcan_last_error_code_bit0_error:
      return CAN::ErrorID::CAN_ERROR_ID_BIT0;
    case mcan_last_error_code_crc_error:
      return CAN::ErrorID::CAN_ERROR_ID_CRC;
    case mcan_last_error_code_no_error:
    case mcan_last_error_code_no_change:
    default:
      return fallback;
  }
}

inline uint32_t AcquireMcanClock(clock_name_t clock)
{
  clock_add_to_group(clock, 0);
  return clock_get_frequency(clock);
}

inline bool HasMcanDefaultMessageBufferCapacity(uint32_t msg_buf_size)
{
  if (msg_buf_size == 0U)
  {
    return false;
  }
#if defined(MCAN_MSG_BUF_SIZE_IN_WORDS)
  constexpr uint64_t required_size =
      static_cast<uint64_t>(MCAN_MSG_BUF_SIZE_IN_WORDS) * sizeof(uint32_t);
  return static_cast<uint64_t>(msg_buf_size) >= required_size;
#else
  return true;
#endif
}

inline ErrorCode ApplyMcanMessageBuffer(MCAN_Type* can, void* msg_buf,
                                        uint32_t msg_buf_size)
{
#if defined(MCAN_SOC_MSG_BUF_IN_AHB_RAM) && (MCAN_SOC_MSG_BUF_IN_AHB_RAM == 1)
  if (can == nullptr || msg_buf == nullptr ||
      !HasMcanDefaultMessageBufferCapacity(msg_buf_size))
  {
    return ErrorCode::ARG_ERR;
  }

  const uintptr_t msg_buf_addr = reinterpret_cast<uintptr_t>(msg_buf);
  if (msg_buf_addr > UINT32_MAX)
  {
    return ErrorCode::ARG_ERR;
  }

  const mcan_msg_buf_attr_t attr = {static_cast<uint32_t>(msg_buf_addr), msg_buf_size};
  return ConvertMcanStatus(mcan_set_msg_buf_attr(can, &attr));
#else
  UNUSED(msg_buf);
  UNUSED(msg_buf_size);
  return can == nullptr ? ErrorCode::ARG_ERR : ErrorCode::OK;
#endif
}

inline void PrepareMcanCommonConfig(MCAN_Type* can, mcan_config_t& config,
                                    bool enable_canfd)
{
  config.enable_canfd = enable_canfd;
  if (enable_canfd)
  {
    mcan_get_default_ram_config(can, &config.ram_config, true);
  }
  config.ram_config.enable_rxbuf = false;
  config.ram_config.rxbuf_elem_count = 0U;
  config.interrupt_mask = MCAN_DRIVER_INTERRUPT_MASK;
}

inline void PrepareMcanAcceptAllFilters(mcan_config_t& config)
{
  config.all_filters_config.global_filter_config.accept_non_matching_std_frame_option =
      MCAN_ACCEPT_NON_MATCHING_FRAME_OPTION_IN_RXFIFO0;
  config.all_filters_config.global_filter_config.accept_non_matching_ext_frame_option =
      MCAN_ACCEPT_NON_MATCHING_FRAME_OPTION_IN_RXFIFO0;
  config.all_filters_config.global_filter_config.reject_remote_std_frame = false;
  config.all_filters_config.global_filter_config.reject_remote_ext_frame = false;
}

inline void ShutdownMcan(MCAN_Type* can, uint32_t irq, bool auto_enable_irq,
                         uint32_t interrupt_mask)
{
  if (can == nullptr)
  {
    return;
  }
  if (auto_enable_irq && irq != 0xFFFFFFFFUL)
  {
    intc_m_disable_irq(irq);
  }

  mcan_disable_interrupts(can, interrupt_mask);
  mcan_clear_interrupt_flags(can, 0xFFFFFFFFUL);
  mcan_deinit(can);
}

inline void EnableMcanInterrupts(MCAN_Type* can, uint32_t irq, bool auto_enable_irq,
                                 uint32_t interrupt_mask)
{
  if (can == nullptr)
  {
    return;
  }

  mcan_clear_interrupt_flags(can, 0xFFFFFFFFUL);
  mcan_enable_interrupts(can, interrupt_mask);
  if (auto_enable_irq && irq != 0xFFFFFFFFUL)
  {
    intc_m_enable_irq_with_priority(irq, 1);
  }
}

inline ErrorCode ReadMcanErrorState(MCAN_Type* can, CAN::ErrorState& state)
{
  state = {};
  if (can == nullptr)
  {
    return ErrorCode::PTR_NULL;
  }

  mcan_error_count_t count{};
  mcan_protocol_status_t protocol{};
  mcan_get_error_counter(can, &count);
  ErrorCode ans = ConvertMcanStatus(mcan_get_protocol_status(can, &protocol));
  if (ans != ErrorCode::OK)
  {
    return ans;
  }

  state.tx_error_counter = count.transmit_error_count;
  state.rx_error_counter = count.receive_error_count;
  state.bus_off = protocol.in_bus_off_state;
  state.error_passive = protocol.in_error_passive_state || count.receive_error_passive;
  state.error_warning = protocol.in_warning_state;
  return ErrorCode::OK;
}

inline size_t HardwareTxQueueEmptySize(MCAN_Type* can)
{
  if (can == nullptr)
  {
    return 0U;
  }
  return static_cast<size_t>(MCAN_TXFQS_TFFL_GET(can->TXFQS));
}

inline bool IsValidMcanClassicPack(const CAN::ClassicPack& pack)
{
  if (pack.dlc > 8U)
  {
    return false;
  }

  switch (pack.type)
  {
    case CAN::Type::STANDARD:
    case CAN::Type::REMOTE_STANDARD:
      return pack.id <= 0x7FFUL;
    case CAN::Type::EXTENDED:
    case CAN::Type::REMOTE_EXTENDED:
      return pack.id <= 0x1FFFFFFFUL;
    case CAN::Type::ERROR:
    case CAN::Type::TYPE_NUM:
    default:
      return false;
  }
}

inline bool IsValidMcanFdIdentifier(const FDCAN::FDPack& pack)
{
  if (pack.type == CAN::Type::STANDARD)
  {
    return pack.id <= 0x7FFUL;
  }
  if (pack.type == CAN::Type::EXTENDED)
  {
    return pack.id <= 0x1FFFFFFFUL;
  }
  return false;
}

inline void BuildMcanClassicTxFrame(const CAN::ClassicPack& pack, mcan_tx_frame_t& frame)
{
  std::memset(&frame, 0, sizeof(frame));

  const bool is_ext =
      pack.type == CAN::Type::EXTENDED || pack.type == CAN::Type::REMOTE_EXTENDED;
  const bool is_rtr =
      pack.type == CAN::Type::REMOTE_STANDARD || pack.type == CAN::Type::REMOTE_EXTENDED;

  frame.use_ext_id = is_ext ? 1U : 0U;
  frame.rtr = is_rtr ? 1U : 0U;
  frame.dlc = pack.dlc <= 8U ? pack.dlc : 8U;
  frame.canfd_frame = 0U;
  frame.bitrate_switch = 0U;
  frame.error_state_indicator = 0U;

  if (is_ext)
  {
    frame.ext_id = pack.id & 0x1FFFFFFFUL;
  }
  else
  {
    frame.std_id = pack.id & 0x7FFUL;
  }

  if (!is_rtr && frame.dlc > 0U)
  {
    std::memcpy(frame.data_8, pack.data, frame.dlc);
  }
}

inline void BuildMcanClassicRxPack(const mcan_rx_message_t& frame, CAN::ClassicPack& pack)
{
  std::memset(&pack, 0, sizeof(pack));

  if (frame.use_ext_id != 0U)
  {
    pack.id = frame.ext_id & 0x1FFFFFFFUL;
    pack.type = (frame.rtr != 0U) ? CAN::Type::REMOTE_EXTENDED : CAN::Type::EXTENDED;
  }
  else
  {
    pack.id = frame.std_id & 0x7FFUL;
    pack.type = (frame.rtr != 0U) ? CAN::Type::REMOTE_STANDARD : CAN::Type::STANDARD;
  }

  pack.dlc = frame.dlc <= 8U ? static_cast<uint8_t>(frame.dlc) : 8U;
  if (frame.rtr == 0U && pack.dlc > 0U)
  {
    std::memcpy(pack.data, frame.data_8, pack.dlc);
  }
}

inline uint8_t GetMcanInstanceIndex(MCAN_Type* can)
{
  if (can == nullptr)
  {
    return MCAN_INSTANCE_COUNT;
  }

#if defined(HPM_MCAN0)
  if (can == HPM_MCAN0)
  {
    return 0U;
  }
#endif
#if defined(HPM_MCAN1)
  if (can == HPM_MCAN1)
  {
    return 1U;
  }
#endif
#if defined(HPM_MCAN2)
  if (can == HPM_MCAN2)
  {
    return 2U;
  }
#endif
#if defined(HPM_MCAN3)
  if (can == HPM_MCAN3)
  {
    return 3U;
  }
#endif
#if defined(HPM_MCAN4)
  if (can == HPM_MCAN4)
  {
    return 4U;
  }
#endif
#if defined(HPM_MCAN5)
  if (can == HPM_MCAN5)
  {
    return 5U;
  }
#endif
#if defined(HPM_MCAN6)
  if (can == HPM_MCAN6)
  {
    return 6U;
  }
#endif
#if defined(HPM_MCAN7)
  if (can == HPM_MCAN7)
  {
    return 7U;
  }
#endif
  return MCAN_INSTANCE_COUNT;
}

inline bool RegisterMcanOwner(uint8_t index, void* owner, McanOwnerKind kind,
                              McanInterruptHandler handler)
{
  if (index >= MCAN_INSTANCE_COUNT || owner == nullptr || handler == nullptr)
  {
    return false;
  }

  auto& slot = mcan_owner_slots[index];
  if (slot.owner != nullptr && slot.owner != owner)
  {
    return false;
  }

  slot.owner = owner;
  slot.kind = kind;
  slot.handler = handler;
  return true;
}

inline void UnregisterMcanOwner(uint8_t index, void* owner)
{
  if (index >= MCAN_INSTANCE_COUNT || owner == nullptr)
  {
    return;
  }

  auto& slot = mcan_owner_slots[index];
  if (slot.owner == owner)
  {
    slot = {};
  }
}

inline void ProcessMcanRegisteredInterrupt(uint8_t index, bool in_isr = true)
{
  if (index >= MCAN_INSTANCE_COUNT)
  {
    return;
  }

  auto& slot = mcan_owner_slots[index];
  if (slot.owner != nullptr && slot.handler != nullptr)
  {
    slot.handler(slot.owner, in_isr);
  }
}

}  // namespace LibXR::detail

extern "C" void libxr_hpm_mcan_irq_handler(uint8_t index);

#endif
