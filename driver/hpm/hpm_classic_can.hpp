#pragma once

#include <atomic>

#include "can.hpp"
#include "hpm_clock_drv.h"
#include "hpm_common.h"
#include "hpm_soc.h"
#include "lockfree_pool.hpp"

#if defined(HPMSOC_HAS_HPMSDK_CAN) && __has_include("hpm_can_drv.h") &&           \
    defined(CAN_SOC_MAX_COUNT) && (CAN_SOC_MAX_COUNT > 0) &&                      \
    (!defined(MCAN_SOC_MAX_COUNT) || (MCAN_SOC_MAX_COUNT == 0))
#include "hpm_can_drv.h"
#define LIBXR_HPM_CLASSIC_CAN_SUPPORTED 1
#else
#define LIBXR_HPM_CLASSIC_CAN_SUPPORTED 0
#endif

// Classic CAN is only enabled for HPM SDK targets that expose the legacy
// HPM_CANx peripheral. MCAN-only targets use the separate MCAN driver.

#if LIBXR_HPM_CLASSIC_CAN_SUPPORTED

namespace LibXR
{

/**
 * @brief HPM classic CAN 驱动实现 / HPM classic CAN driver implementation.
 */
class HPMClassicCAN : public CAN
{
 public:
  static constexpr uint32_t INVALID_IRQ = 0xFFFFFFFFu;
  static constexpr uint32_t DEFAULT_TX_POOL_SIZE = 8;
  static constexpr uint8_t MAX_INSTANCES = CAN_SOC_MAX_COUNT;
  static constexpr uint8_t RX_INTERRUPT_MASK =
      CAN_EVENT_RECEIVE | CAN_EVENT_RX_BUF_OVERRUN | CAN_EVENT_RX_BUF_FULL;
  static constexpr uint8_t TX_INTERRUPT_MASK =
      CAN_EVENT_TX_SECONDARY_BUF | CAN_EVENT_TX_PRIMARY_BUF;
  static constexpr uint8_t ERROR_INTERRUPT_MASK = CAN_EVENT_ERROR | CAN_EVENT_ABORT;
  static constexpr uint8_t INTERRUPT_MASK =
      RX_INTERRUPT_MASK | TX_INTERRUPT_MASK | ERROR_INTERRUPT_MASK;
  static constexpr uint8_t CAN_ERROR_INTERRUPT_MASK =
      CAN_ERROR_PASSIVE_INT_ENABLE | CAN_ERROR_ARBITRATION_LOST_INT_ENABLE |
      CAN_ERROR_BUS_ERROR_INT_ENABLE;

  /**
   * @brief 构造 classic CAN 适配器 / Construct the classic CAN adapter.
   */
  HPMClassicCAN(CAN_Type* can, clock_name_t clock, uint8_t index = 0,
                uint32_t irq = INVALID_IRQ, bool auto_enable_irq = true,
                uint32_t tx_pool_size = DEFAULT_TX_POOL_SIZE);

  /**
   * @brief 设置 CAN 配置 / Set CAN configuration.
   */
  ErrorCode SetConfig(const CAN::Configuration& cfg) override;

  uint32_t GetClockFreq() const override;

  ErrorCode AddMessage(const ClassicPack& pack) override;

  /**
   * @brief 查询当前错误状态 / Query current error state.
   */
  ErrorCode GetErrorState(CAN::ErrorState& state) const override;

  /**
   * @brief 轮询 RX buffer 并分发 LibXR 回调 / Poll RX buffer.
   */
  void ProcessRx(bool in_isr = false);

  /**
   * @brief 处理 classic CAN 中断标志 / Process interrupt flags.
   */
  void ProcessInterrupt(bool in_isr = true);

  static void OnInterrupt(uint8_t index);

 private:
  static ErrorCode ConvertStatus(hpm_stat_t status);
  static ErrorCode ValidateConfig(const CAN::Configuration& cfg);
  void EmitErrorFrame(CAN::ErrorID error_id, bool in_isr);
  void Shutdown();
  static bool HasLowLevelTiming(const CAN::BitTiming& timing);
  static uint16_t SamplePointToPermille(float sample_point);
  void TxService();

  static can_node_mode_t ConvertMode(const CAN::Mode& mode);
  static void ApplyLowLevelTiming(const CAN::BitTiming& src, can_bit_timing_param_t& dst);
  static void BuildTxFrame(const ClassicPack& pack, can_transmit_buf_t& frame);
  static bool BuildRxPack(const can_receive_buf_t& frame, ClassicPack& pack);
  static CAN::ErrorID ConvertProtocolError(uint8_t error_kind);
  void ProcessRxBuffer(bool in_isr);
  void ProcessError(bool in_isr);

  CAN_Type* can_;
  clock_name_t clock_;
  uint8_t index_;
  uint32_t irq_;
  bool auto_enable_irq_;
  bool configured_ = false;

  LockFreePool<ClassicPack> tx_pool_;
  std::atomic<uint32_t> tx_lock_{0};
  std::atomic<uint32_t> tx_pend_{0};

  static HPMClassicCAN* instance_map_[MAX_INSTANCES];
};

}  // namespace LibXR

#endif
