#pragma once

#include <atomic>
#include <cstdint>

#include "hpm_mcan_backend.hpp"
#include "queue.hpp"

#if LIBXR_HPM_MCAN_SUPPORTED

namespace LibXR
{

/**
 * @brief HPM MCAN FDCAN 驱动实现 / HPM MCAN FDCAN driver implementation.
 */
class HPMCANFD : public FDCAN
{
 public:
  static constexpr uint32_t INVALID_IRQ = 0xFFFFFFFFu;
  static constexpr uint32_t DEFAULT_TX_POOL_SIZE = 8;
  static constexpr uint8_t MAX_INSTANCES = detail::MCAN_INSTANCE_COUNT;

  /**
   * @brief 构造 MCAN FDCAN wrapper / Construct the MCAN FDCAN wrapper.
   */
  HPMCANFD(MCAN_Type* can, clock_name_t clock, uint8_t index = 0,
           uint32_t irq = INVALID_IRQ, bool auto_enable_irq = true,
           uint32_t queue_size = DEFAULT_TX_POOL_SIZE, void* msg_buf = nullptr,
           uint32_t msg_buf_size = 0);
  ~HPMCANFD() override;

  /** @brief 配置外部 MCAN message RAM / Configure external MCAN message RAM. */
  ErrorCode SetMessageBuffer(void* msg_buf, uint32_t msg_buf_size);

  /**
   * @brief 设置 CAN 配置 / Set CAN configuration.
   */
  ErrorCode SetConfig(const CAN::Configuration& cfg) override;

  /**
   * @brief 设置 FDCAN 全配置 / Set full FDCAN configuration.
   */
  ErrorCode SetConfig(const FDCAN::Configuration& cfg) override;

  uint32_t GetClockFreq() const override;

  ErrorCode AddMessage(const ClassicPack& pack) override;

  ErrorCode AddMessage(const FDPack& pack) override;

  /**
   * @brief 查询当前错误状态 / Query current error state.
   */
  ErrorCode GetErrorState(CAN::ErrorState& state) const override;

  /**
   * @brief 查询硬件 TX FIFO 空闲槽数量 / Query TX FIFO free slots.
   */
  size_t HardwareTxQueueEmptySize() const;

  /**
   * @brief 处理指定 RX FIFO 的接收中断 / Process RX FIFO interrupt.
   */
  void ProcessRxInterrupt(uint32_t fifo, bool in_isr = true);

  /** @brief 处理 MCAN error/status 中断 / Process MCAN error/status interrupt. */
  void ProcessErrorStatusInterrupt(uint32_t error_status_its, bool in_isr = true);

  void ProcessInterrupt(bool in_isr = true);

  static void OnInterrupt(uint8_t index);

  static void BuildTxFrame(const ClassicPack& pack, mcan_tx_frame_t& frame);

  static void BuildTxFrame(const FDPack& pack, mcan_tx_frame_t& frame,
                           bool bitrate_switch, bool error_state_indicator);

  void TxService();

 private:
  static uint8_t DlcToBytes(uint8_t dlc);
  static uint8_t BytesToDlc(uint8_t bytes);
  ErrorCode ApplyMessageBuffer();
  void Shutdown();
  static bool BuildRxPack(const mcan_rx_message_t& frame, ClassicPack& pack);
  static bool BuildRxPack(const mcan_rx_message_t& frame, FDPack& pack);
  void EmitErrorFrame(CAN::ErrorID error_id, bool in_isr);

  MCAN_Type* can_;
  clock_name_t clock_;
  bool clock_ready_{false};
  uint8_t index_;
  uint32_t irq_;
  bool auto_enable_irq_;
  void* msg_buf_{nullptr};
  uint32_t msg_buf_size_{0};
  std::atomic<bool> configured_{false};
  std::atomic<bool> fd_enabled_{false};
  std::atomic<bool> brs_enabled_{false};
  std::atomic<bool> esi_enabled_{false};

  std::atomic<uint32_t> tx_lock_{0};
  std::atomic<uint32_t> tx_pend_{0};

  MPMCQueue<ClassicPack> tx_queue_;
  MPMCQueue<FDPack> tx_fd_queue_;
  bool tx_retry_valid_ = false;
  ClassicPack tx_retry_pack_{};
  bool tx_fd_retry_valid_ = false;
  FDPack tx_fd_retry_pack_{};
  bool prefer_fd_next_ = true;

  struct
  {
    mcan_rx_message_t frame;
    ClassicPack pack;
    FDPack pack_fd;
  } rx_buff_;

  struct
  {
    mcan_tx_frame_t frame;
  } tx_buff_;
};

}  // namespace LibXR

#endif
