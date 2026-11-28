#pragma once

#include "main.h"

#ifdef HAL_CAN_MODULE_ENABLED

#ifdef CAN
#if !defined(CAN1)
#define CAN1 ((CAN_TypeDef*)CAN_BASE)
#endif
#undef CAN
#endif

#include "can.hpp"
#include "serialized_service.hpp"
#include "slot_pool.hpp"

typedef enum
{
#ifdef CAN1
  STM32_CAN1,
#endif
#ifdef CAN2
  STM32_CAN2,
#endif
#ifdef CAN3
  STM32_CAN3,
#endif
  STM32_CAN_NUMBER,
  STM32_CAN_ID_ERROR
} stm32_can_id_t;

static_assert(STM32_CAN_NUMBER >= 1, "No CAN instance detected for this MCU");

stm32_can_id_t STM32_CAN_GetID(CAN_TypeDef* addr);  // NOLINT

namespace LibXR
{
/**
 * @brief STM32 CAN 驱动实现 / STM32 CAN driver implementation
 */
class STM32CAN : public CAN
{
 public:
  /**
   * @brief 构造 CAN 驱动对象 / Construct CAN driver object
   *
   * @param hcan HAL CAN 句柄 / HAL CAN handle
   * @param queue_size 发送队列大小 / TX queue size
   * @pre queue_size 必须大于 1 / queue_size must be greater than 1.
   */
  STM32CAN(CAN_HandleTypeDef* hcan, uint32_t queue_size);

  /**
   * @brief 初始化驱动 / Initialize driver
   *
   * @return ErrorCode 错误码 / Error code
   */
  ErrorCode Init(void);

  /**
   * @brief 设置 CAN 配置 / Set CAN configuration
   *
   * @param cfg CAN 配置参数 / CAN configuration
   * @return ErrorCode 操作结果 / Operation result
   */
  ErrorCode SetConfig(const CAN::Configuration& cfg) override;

  uint32_t GetClockFreq() const override;

  ErrorCode AddMessage(const ClassicPack& pack) override;

  ErrorCode GetErrorState(CAN::ErrorState& state) const override;

  /**
   * @brief 处理接收中断 / Handle RX interrupt
   *
   */
  void ProcessRxInterrupt();

  /**
   * @brief 处理错误中断 / Handle error interrupt
   * @note 由 `HAL_CAN_ErrorCallback` 触发；在 STM32 bxCAN HAL 中既可能运行于错误中断
   *       （SCE），也可能因 TX 邮箱错误运行于发送中断上下文中。
   *       Triggered from `HAL_CAN_ErrorCallback`; in STM32 bxCAN HAL this may execute in
   *       either the error (SCE) IRQ or a TX IRQ mailbox-error path.
   */
  void ProcessErrorInterrupt();

  CAN_HandleTypeDef* hcan_;  ///< STM32 HAL CAN 句柄 / STM32 HAL CAN handle.

  stm32_can_id_t id_;                      ///< CAN 实例 ID / CAN instance ID.
  uint32_t fifo_;                          ///< 接收 FIFO 编号 / RX FIFO index.
  static STM32CAN* map[STM32_CAN_NUMBER];  ///< 按实例 ID 索引的驱动映射表 / Driver map
                                           ///< indexed by CAN ID. NOLINT

  /// 接收缓冲区（HAL 头 + Classic CAN 数据包） / Receive buffer (HAL header + classic
  /// packet).
  struct
  {
    CAN_RxHeaderTypeDef header;  ///< HAL CAN 接收帧头 / HAL CAN RX frame header.
    ClassicPack pack;            ///< Classic CAN 数据包 / Classic CAN packet.
  } rx_buff_;

  /// 发送缓冲区（HAL 头 + Classic CAN 数据包） / Transmit buffer (HAL header + classic
  /// packet).
  struct
  {
    CAN_TxHeaderTypeDef header;  ///< HAL CAN 发送帧头 / HAL CAN TX frame header.
    ClassicPack pack;            ///< Classic CAN 数据包 / Classic CAN packet.
  } tx_buff_;

  uint32_t txMailbox;  ///< 最近提交的 HAL 发送邮箱号 / Last submitted HAL TX mailbox.

  /// 发送槽池 / TX slot pool.
  SlotPool<ClassicPack> tx_queue_;
  /// 待重试帧有效标记 / Pending retry frame flag.
  bool tx_retry_valid_{false};
  /// 待重试帧 / Pending retry frame.
  ClassicPack tx_retry_pack_{};

  SerializedService tx_service_;  ///< 发送服务串行化调度器 / TX service serializer.

  /**
   * @brief 由 ClassicPack 构造 HAL 发送帧头 / Build HAL TX header from a `ClassicPack`.
   * @param p Classic CAN 帧结构 / Source `ClassicPack`.
   * @param h 输出 HAL 发送帧头 / Output HAL TX header.
   */
  static inline void BuildTxHeader(const ClassicPack& p, CAN_TxHeaderTypeDef& h);

  /**
   * @brief 从发送队列提取待发帧并提交到 HAL 邮箱 / Drain TX queue into HAL TX mailbox.
   */
  void TxService();
};
}  // namespace LibXR

#endif
