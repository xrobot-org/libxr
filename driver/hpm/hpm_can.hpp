#pragma once

#include "hpm_mcan_backend.hpp"

#if LIBXR_HPM_MCAN_SUPPORTED
#define LIBXR_HPM_CAN_SUPPORTED 1
#else
#define LIBXR_HPM_CAN_SUPPORTED 0
#endif

#if LIBXR_HPM_CAN_SUPPORTED

#include <atomic>
#include <cstdint>

#include "queue.hpp"

namespace LibXR
{

class HPMCAN final : public CAN
{
 public:
  static constexpr uint32_t INVALID_IRQ = 0xFFFFFFFFu;
  static constexpr uint8_t MAX_INSTANCES = detail::MCAN_INSTANCE_COUNT;

  HPMCAN(MCAN_Type* can, clock_name_t clock, uint32_t irq, uint32_t queue_size,
         void* msg_buf, uint32_t msg_buf_size);
  ~HPMCAN() override;

  ErrorCode SetConfig(const CAN::Configuration& cfg) override;
  uint32_t GetClockFreq() const override;
  ErrorCode AddMessage(const ClassicPack& pack) override;
  ErrorCode GetErrorState(CAN::ErrorState& state) const override;

  void ProcessInterrupt();

  ErrorCode EnableInterrupt();

  ErrorCode DisableInterrupt();

  /** @brief 配置外部 MCAN message RAM / Configure external MCAN message RAM. */
  ErrorCode SetMessageBuffer(void* msg_buf, uint32_t msg_buf_size);

  static void OnInterrupt(uint8_t index);

 private:
  static void BuildTxFrame(const ClassicPack& pack, mcan_tx_frame_t& frame);
  static void BuildRxPack(const mcan_rx_message_t& frame, ClassicPack& pack);

  void EnableCanInterrupts();
  void DisableCanInterrupts();
  ErrorCode ApplyMessageBuffer();
  void TxService();
  void ProcessRxFifo(uint32_t fifo_index);
  void ProcessTx();
  void ProcessError();

  MCAN_Type* can_;
  clock_name_t clock_;
  bool clock_ready_{false};
  uint32_t irq_;
  uint8_t index_{MAX_INSTANCES};
  void* msg_buf_{nullptr};
  uint32_t msg_buf_size_{0};
  std::atomic<bool> configured_{false};

  MPMCQueue<ClassicPack> tx_queue_;
  bool tx_retry_valid_{false};
  ClassicPack tx_retry_pack_{};

  std::atomic<uint32_t> tx_lock_{0};
  std::atomic<uint32_t> tx_pend_{0};
};

}  // namespace LibXR

#endif
