#pragma once

#include <atomic>
#include <cstdint>
#include <cstring>

#include "can.hpp"
#include "ch32_can_def.hpp"
#include "queue.hpp"

#include DEF2STR(LIBXR_CH32_CONFIG_FILE)

namespace LibXR
{
/**
 * @brief CH32 CAN 驱动实现 / CH32 CAN driver implementation
 *
 * 设计目标与 STM32 CAN 驱动行为保持一致。
 * Designed to keep behavior aligned with the STM32 CAN driver.
 *
 * @note 基于 WCH StdPeriph CAN 接口实现 / Implemented on top of WCH StdPeriph CAN APIs.
 */
class CH32CAN : public CAN
{
 public:
  /**
   * @brief 构造 CAN 驱动对象 / Construct CAN driver object
   *
   * @param id CAN 实例编号 / CAN instance ID
   * @param queue_size 发送队列大小 / TX queue size (ClassicPack entries)
   * @pre queue_size 必须大于 1 / queue_size must be greater than 1.
   */
  explicit CH32CAN(ch32_can_id_t id, uint32_t queue_size);
  ~CH32CAN() override = default;

  /**
   * @brief 初始化过滤器和中断路由 / Initialize filters and IRQ routing
   */
  ErrorCode Init();

  /**
   * @brief 设置 CAN 配置 / Set CAN configuration
   */
  ErrorCode SetConfig(const CAN::Configuration& cfg) override;

  /**
   * @brief 获取 CAN 时钟频率 / Get CAN clock frequency
   */
  uint32_t GetClockFreq() const override;

  /**
   * @brief 发送消息入队 / Enqueue TX message
   */
  ErrorCode AddMessage(const ClassicPack& pack) override;

  /**
   * @brief 获取总线错误状态 / Get bus error state
   */
  ErrorCode GetErrorState(CAN::ErrorState& state) const override;

  /**
   * @brief 处理接收中断 / Handle RX interrupt
   */
  void ProcessRxInterrupt();

  /**
   * @brief 处理发送中断 / Handle TX interrupt
   */
  void ProcessTxInterrupt();

  /**
   * @brief 处理错误中断 / Handle error interrupt
   */
  void ProcessErrorInterrupt();

  /// 中断分发表 / IRQ dispatch map
  static CH32CAN* map[CH32_CAN_NUMBER];  // NOLINT

 private:
  static inline void BuildTxMsg(const ClassicPack& p, CanTxMsg& m);

  void EnableIRQs();
  void DisableIRQs();

  void TxService();

 private:
  CAN_TypeDef* instance_{nullptr};
  ch32_can_id_t id_{CH32_CAN_ID_ERROR};

  uint8_t fifo_{0};
  uint8_t filter_bank_{0};

  /// 发送软件队列 / TX software queue.
  MPMCQueue<ClassicPack> tx_queue_;
  /// 待重试帧有效标记 / Pending retry frame flag.
  bool tx_retry_valid_{false};
  /// 待重试帧 / Pending retry frame.
  ClassicPack tx_retry_pack_{};

  std::atomic<uint32_t> tx_lock_{0};
  std::atomic<uint32_t> tx_pend_{0};

  /// 缓存配置（用于保留原值语义） / Cached configuration for keep-previous semantics
  CAN::Configuration cfg_cache_{};

  /// 中断上下文收发缓冲 / RX/TX buffers in IRQ context
  CanRxMsg rx_msg_{};
  CanTxMsg tx_msg_{};
};
}  // namespace LibXR
