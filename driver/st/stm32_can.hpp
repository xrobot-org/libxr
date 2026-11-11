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
   *
   */
  void ProcessErrorInterrupt();

  /// HAL CAN 句柄 / HAL CAN handle
  CAN_HandleTypeDef* hcan_;

  /// 外设编号，同时是 map 的下标 / Peripheral ID, also the index into map
  stm32_can_id_t id_;
  /// 本实例使用的接收 FIFO（CAN_RX_FIFO0 或 CAN_RX_FIFO1）/ RX FIFO used by this instance
  /// (CAN_RX_FIFO0 or CAN_RX_FIFO1)
  uint32_t fifo_;
  /// 外设编号到实例的映射，供 HAL 回调找回对象 / Instance table by peripheral ID, used
  /// by HAL callbacks to find the object
  static STM32CAN* map[STM32_CAN_NUMBER];  // NOLINT

  /// 接收中断里读取 HAL 报文的暂存区 / Staging area for the frame read from the HAL in
  /// the RX interrupt
  struct
  {
    CAN_RxHeaderTypeDef header;
    ClassicPack pack;
  } rx_buff_;

  /// 保留的发送暂存区，当前发送路径不使用 / Reserved TX staging area, not used by the
  /// current TX path
  struct
  {
    CAN_TxHeaderTypeDef header;
    ClassicPack pack;
  } tx_buff_;

  /// 最近一次交给 HAL 的发送邮箱号 / Mailbox of the frame most recently handed to the HAL
  uint32_t txMailbox;

  /// 发送槽池 / TX slot pool.
  SlotPool<ClassicPack> tx_queue_;
  /// 待重试帧有效标记 / Pending retry frame flag.
  bool tx_retry_valid_{false};
  /// 待重试帧 / Pending retry frame.
  ClassicPack tx_retry_pack_{};

  /// 发送服务锁：1 表示某个上下文正在执行 TxService / TX service lock: 1 while a
  /// context is running TxService
  std::atomic<uint32_t> tx_lock_{0};
  /// 发送服务请求标记：1 表示有新的发送需求待处理 / TX service request flag: 1 when new
  /// TX work is pending
  std::atomic<uint32_t> tx_pend_{0};

  /// 把经典帧转换为 HAL 发送头（长度限制为 8，标准/扩展 ID 与远程帧标志）/ Convert a
  /// classic frame to a HAL TX header (DLC capped at 8, standard/extended ID, RTR flag)
  static inline void BuildTxHeader(const ClassicPack& p, CAN_TxHeaderTypeDef& h);

  /// 发送服务：持锁，把队列（及待重试帧）填入空闲邮箱；可在任意上下文调用 / TX service:
  /// takes the lock and fills free mailboxes from the queue (and the retry frame); may be
  /// called from any context
  void TxService();
};
}  // namespace LibXR

#endif
