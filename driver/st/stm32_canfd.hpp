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
#include "queue.hpp"

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

  template <typename T>
  typename std::enable_if<!HasMessageRAMOffset<T>::value, void>::type
  CheckMessageRAMOffset(T&, uint32_t = FDCAN_MESSAGE_RAM_WORDS_MAX)
  {
  }

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
      ASSERT(false);
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

  static inline void BuildTxHeader(const ClassicPack& p, FDCAN_TxHeaderTypeDef& h);
  static inline void BuildTxHeader(const FDPack& p, FDCAN_TxHeaderTypeDef& h);

  void TxService();

  std::atomic<uint32_t> tx_lock_{0};
  std::atomic<uint32_t> tx_pend_{0};

  FDCAN_HandleTypeDef* hcan_;

  stm32_fdcan_id_t id_;
  /// Classic 发送软件队列 / Classic TX software queue.
  MPMCQueue<ClassicPack> tx_queue_;
  /// FD 发送软件队列 / FD TX software queue.
  MPMCQueue<FDPack> tx_fd_queue_;
  /// 待重试 Classic 帧有效标记 / Pending Classic retry frame flag.
  bool tx_classic_retry_valid_{false};
  /// 待重试 Classic 帧 / Pending Classic retry frame.
  ClassicPack tx_classic_retry_pack_{};
  /// 待重试 FD 帧有效标记 / Pending FD retry frame flag.
  bool tx_fd_retry_valid_{false};
  /// 待重试 FD 帧 / Pending FD retry frame.
  FDPack tx_fd_retry_pack_{};

  static STM32CANFD* map[STM32_FDCAN_NUMBER];  // NOLINT

  struct
  {
    FDCAN_RxHeaderTypeDef header;
    ClassicPack pack;
    FDPack pack_fd;
  } rx_buff_;

  struct
  {
    FDCAN_TxHeaderTypeDef header;
    ClassicPack pack;
    FDPack pack_fd;
  } tx_buff_;

  uint32_t txMailbox;
};
}  // namespace LibXR

#endif
