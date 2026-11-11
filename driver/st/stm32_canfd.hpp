#pragma once

#include <cstdint>
#include <type_traits>
#include <utility>

#include "libxr_def.hpp"
#include "main.h"

#ifdef HAL_FDCAN_MODULE_ENABLED

#ifdef FDCAN
#undef FDCAN
#endif

#include "can.hpp"
#include "slot_pool.hpp"

#ifndef FDCAN_MESSAGE_RAM_WORDS_MAX
#define FDCAN_MESSAGE_RAM_WORDS_MAX 2560u
#endif

typedef enum : uint8_t
{
#ifdef FDCAN1
  STM32_FDCAN1,
#endif
#ifdef FDCAN2
  STM32_FDCAN2,
#endif
#ifdef FDCAN3
  STM32_FDCAN3,
#endif
  STM32_FDCAN_NUMBER,
  STM32_FDCAN_ID_ERROR
} stm32_fdcan_id_t;

stm32_fdcan_id_t STM32_FDCAN_GetID(FDCAN_GlobalTypeDef* addr);  // NOLINT

template <typename, typename = void>
struct HasTxFifoQueueElmtsNbr : std::false_type
{
};

template <typename T>
struct HasTxFifoQueueElmtsNbr<
    T, std::void_t<decltype(std::declval<T>()->Init.TxFifoQueueElmtsNbr)>>
    : std::true_type
{
};

// 当没有TxFifoQueueElmtsNbr成员时的处理函数
template <typename T>
typename std::enable_if<!HasTxFifoQueueElmtsNbr<T>::value, uint32_t>::type
GetTxFifoTotalElements(T& hcan)  // NOLINT
{
  UNUSED(hcan);
  return 3;
}

// 当有TxFifoQueueElmtsNbr成员时的处理函数
template <typename T>
typename std::enable_if<HasTxFifoQueueElmtsNbr<T>::value, uint32_t>::type
GetTxFifoTotalElements(T& hcan)  // NOLINT
{
  return hcan->Init.TxFifoQueueElmtsNbr;
}

namespace LibXR
{
/**
 * @brief STM32 FDCAN 驱动实现 / STM32 FDCAN driver implementation
 */
class STM32CANFD : public FDCAN
{
 public:
  template <typename, typename = void>
  struct HasMessageRAMOffset : std::false_type
  {
  };

  template <typename T>
  struct HasMessageRAMOffset<
      T, std::void_t<decltype(std::declval<T>()->Init.MessageRAMOffset)>> : std::true_type
  {
  };

  /// 句柄没有 MessageRAMOffset 字段时不做检查 / No check when the handle has no
  /// MessageRAMOffset field
  template <typename T>
  typename std::enable_if<!HasMessageRAMOffset<T>::value, void>::type
  CheckMessageRAMOffset(T&, uint32_t = FDCAN_MESSAGE_RAM_WORDS_MAX)
  {
  }

  /// 断言本实例的消息 RAM 区间不越界，且与已登记的其他 FDCAN 实例不重叠 / Assert that
  /// this instance's message RAM range is in bounds and does not overlap any other
  /// registered FDCAN instance
  /// @param fdcan_handle FDCAN 句柄 / FDCAN handle
  /// @param max_words 消息 RAM 总字数 / Total message RAM size in words
  template <typename T>
  typename std::enable_if<HasMessageRAMOffset<T>::value, void>::type
  CheckMessageRAMOffset(T& fdcan_handle, uint32_t max_words = FDCAN_MESSAGE_RAM_WORDS_MAX)
  {
    using HandleT = std::remove_reference_t<decltype(*fdcan_handle)>;
    using InitT = decltype(HandleT{}.Init);

    // NOLINTNEXTLINE
    static const auto FDCAN_ElmtWords = [](uint32_t sz) -> uint32_t
    {
#if defined(FDCAN_DATA_BYTES_8)
      switch (sz)
      {
        case FDCAN_DATA_BYTES_8:
          return 4;  // 2 words header + 2 words data(0..8B)
        case FDCAN_DATA_BYTES_12:
          return 5;
        case FDCAN_DATA_BYTES_16:
          return 6;
        case FDCAN_DATA_BYTES_20:
          return 7;
        case FDCAN_DATA_BYTES_24:
          return 8;
        case FDCAN_DATA_BYTES_32:
          return 10;
        case FDCAN_DATA_BYTES_48:
          return 14;
        case FDCAN_DATA_BYTES_64:
          return 18;
        default:
          return 4;
      }
#else
      static_assert(sizeof(T) == 0, "FDCAN_DATA_BYTES_8 is not defined by the HAL");
      return 4;
#endif
    };

    const uint32_t TX_FIFO_ELEMS =
        HasTxFifoQueueElmtsNbr<T>::value ? fdcan_handle->Init.TxFifoQueueElmtsNbr : 0u;

    // NOLINTNEXTLINE
    const auto FDCAN_MessageRAMWords = [&](const InitT& c) -> uint32_t
    {
      return c.StdFiltersNbr * 1u + c.ExtFiltersNbr * 2u +
             c.RxFifo0ElmtsNbr * FDCAN_ElmtWords(c.RxFifo0ElmtSize) +
             c.RxFifo1ElmtsNbr * FDCAN_ElmtWords(c.RxFifo1ElmtSize) +
             c.RxBuffersNbr * FDCAN_ElmtWords(c.RxBufferSize) + c.TxEventsNbr * 2u +
             (c.TxBuffersNbr + TX_FIFO_ELEMS) * FDCAN_ElmtWords(c.TxElmtSize);
    };

    static struct
    {
      bool inited;
      uint32_t offset;
      uint32_t size;
    } offset_map[STM32_FDCAN_NUMBER] = {};  // NOLINT

    auto id = STM32_FDCAN_GetID(fdcan_handle->Instance);
    ASSERT(id >= 0 && id < STM32_FDCAN_NUMBER);

    offset_map[id].offset = fdcan_handle->Init.MessageRAMOffset;
    offset_map[id].size = FDCAN_MessageRAMWords(fdcan_handle->Init);

    const uint32_t START = offset_map[id].offset;
    const uint32_t END = START + offset_map[id].size;

    /* 内存越界 */
    ASSERT(START <= max_words);
    ASSERT(END <= max_words);

    for (auto& it : offset_map)
    {
      if (!it.inited)
      {
        continue;
      }

      const uint32_t A0 = it.offset, A1 = it.offset + it.size;
      const uint32_t B0 = START, B1 = END;
      if (A0 < B1 && B0 < A1)
      {
        /* 内存区域重叠 */
        ASSERT(false);
      }
    }

    offset_map[id].inited = true;
  }

  /**
   * @brief 构造 FDCAN 驱动对象 / Construct FDCAN driver object
   *
   * @param hcan HAL FDCAN 句柄 / HAL FDCAN handle
   * @param queue_size 发送队列大小 / TX queue size
   * @pre queue_size 必须大于 1 / queue_size must be greater than 1.
   */
  STM32CANFD(FDCAN_HandleTypeDef* hcan, uint32_t queue_size);

  /**
   * @brief 初始化驱动 / Initialize driver
   *
   * @return ErrorCode 错误码 / Error code
   */
  ErrorCode Init(void);

  /**
   * @brief 设置 CAN/FDCAN 配置 / Set CAN/FDCAN configuration
   *
   * @param cfg 仲裁相位配置 / Nominal (arbitration) configuration
   * @return ErrorCode 操作结果 / Operation result
   */
  ErrorCode SetConfig(const CAN::Configuration& cfg) override;

  /**
   * @brief 设置 FDCAN 全配置 / Set full FDCAN configuration
   *
   * @param cfg FDCAN 配置参数 / FDCAN configuration
   * @return ErrorCode 操作结果 / Operation result
   */
  ErrorCode SetConfig(const FDCAN::Configuration& cfg) override;

  /**
   * @brief 获取 FDCAN 外设时钟 / Get FDCAN kernel clock
   */
  uint32_t GetClockFreq() const override;

  ErrorCode AddMessage(const ClassicPack& pack) override;

  ErrorCode AddMessage(const FDPack& pack) override;

  /**
   * @brief 查询当前错误状态 / Query current FDCAN error state
   */
  ErrorCode GetErrorState(CAN::ErrorState& state) const override;

  /**
   * @brief 处理接收中断 / Handle RX interrupt
   *
   * @param fifo 接收 FIFO 编号 / RX FIFO index
   */
  void ProcessRxInterrupt(uint32_t fifo);

  /**
   * @brief 处理错误状态中断 / Handle error-status interrupt
   *
   * @param error_status_its 错误状态标志 / Error-status flags
   */
  void ProcessErrorStatusInterrupt(uint32_t error_status_its);

  /**
   * @brief 获取硬件发送队列空闲数 / Get free level of hardware TX queue
   *
   * @return size_t 空闲元素数量 / Number of free elements
   */
  size_t HardwareTxQueueEmptySize()
  {
    auto free = HAL_FDCAN_GetTxFifoFreeLevel(hcan_);
    return free;
  }

  /// 把经典帧转换为 HAL 发送头 / Convert a classic frame to a HAL TX header
  static inline void BuildTxHeader(const ClassicPack& p, FDCAN_TxHeaderTypeDef& h);
  /// 把 FD 帧转换为 HAL 发送头 / Convert an FD frame to a HAL TX header
  static inline void BuildTxHeader(const FDPack& p, FDCAN_TxHeaderTypeDef& h);

  /// 发送服务：持锁，把队列（及待重试帧）填入硬件发送 FIFO；可在任意上下文调用 / TX
  /// service: takes the lock and fills the hardware TX FIFO from the queues (and retry
  /// frames); may be called from any context
  void TxService();

  /// 发送服务锁：1 表示某个上下文正在执行 TxService / TX service lock: 1 while a
  /// context is running TxService
  std::atomic<uint32_t> tx_lock_{0};
  /// 发送服务请求标记：1 表示有新的发送需求待处理 / TX service request flag: 1 when new
  /// TX work is pending
  std::atomic<uint32_t> tx_pend_{0};

  /// HAL FDCAN 句柄 / HAL FDCAN handle
  FDCAN_HandleTypeDef* hcan_;

  /// 外设编号，同时是 map 的下标 / Peripheral ID, also the index into map
  stm32_fdcan_id_t id_;
  /// Classic 发送槽池 / Classic TX slot pool.
  SlotPool<ClassicPack> tx_queue_;
  /// FD 发送槽池 / FD TX slot pool.
  SlotPool<FDPack> tx_fd_queue_;
  /// 待重试 Classic 帧有效标记 / Pending Classic retry frame flag.
  bool tx_classic_retry_valid_{false};
  /// 待重试 Classic 帧 / Pending Classic retry frame.
  ClassicPack tx_classic_retry_pack_{};
  /// 待重试 FD 帧有效标记 / Pending FD retry frame flag.
  bool tx_fd_retry_valid_{false};
  /// 待重试 FD 帧 / Pending FD retry frame.
  FDPack tx_fd_retry_pack_{};

  /// 外设编号到实例的映射，供 HAL 回调找回对象 / Instance table by peripheral ID, used
  /// by HAL callbacks to find the object
  static STM32CANFD* map[STM32_FDCAN_NUMBER];  // NOLINT

  /// 接收中断里读取 HAL 报文的暂存区 / Staging area for the frame read from the HAL in
  /// the RX interrupt
  struct
  {
    FDCAN_RxHeaderTypeDef header;
    ClassicPack pack;
    FDPack pack_fd;
  } rx_buff_;

  /// 保留的发送暂存区，当前发送路径不使用 / Reserved TX staging area, not used by the
  /// current TX path
  struct
  {
    FDCAN_TxHeaderTypeDef header;
    ClassicPack pack;
    FDPack pack_fd;
  } tx_buff_;

  /// 保留的邮箱号，当前未被写入 / Reserved mailbox number, not written at present
  uint32_t txMailbox;
};
}  // namespace LibXR

#endif
