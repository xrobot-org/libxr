#pragma once

#include <ti/driverlib/dl_dma.h>
#include <ti/driverlib/dl_uart_main.h>

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

#include "libxr_type.hpp"
#include "serialized_service.hpp"
#include "ti_msp_dl_config.h"
#include "uart.hpp"

namespace LibXR
{

class MSPM0UART;

/**
 * @brief 通知串口恢复接收的读端口 / Read port that resumes UART reception.
 */
class MSPM0UARTReadPort : public ReadPort
{
 public:
  /// 按字节容量构造接收队列 / Construct an RX queue with the byte capacity.
  explicit MSPM0UARTReadPort(size_t capacity) : ReadPort(capacity) {}

  /// 在串口初始化时绑定后端 / Bind the backend during UART initialization.
  void SetOwner(MSPM0UART* owner) { owner_ = owner; }

 protected:
  /// 发布接收空间可用通知 / Publish RX space availability.
  void OnReadQueueSpaceAvailable(bool in_isr) override;

 private:
  MSPM0UART* owner_ = nullptr;  ///< 初始化时绑定的后端 / Backend bound at initialization.
};

/**
 * @brief 使用双缓冲 DMA 发送的 MSPM0 串口 / MSPM0 UART with double-buffered DMA TX.
 *
 * 主 UART 按字节中断接收，扩展 UART 使用循环 DMA。写请求复制到空闲发送半区后完成，
 * DMA 完成仅释放半区；配置在当前发送结束且 UART 空闲后应用。
 * Main RX uses byte interrupts; Extend RX uses circular DMA. Writes complete after
 * copying into a free TX half; DMA completion only releases that half. Configuration
 * waits for the active transfer and UART idle boundary.
 *
 * @pre BSP 串行初始化实例并独占分配 DMA 通道；共享 DMA 中断由 BSP 开启，优先级与
 *      UART 中断相同（1）。对象及借用缓冲区在运行期间须保持有效。
 *      The BSP initializes instances serially, reserves distinct DMA channels, and
 *      enables the shared DMA interrupt at the UART interrupt priority (1).
 *      Keep the object and borrowed buffers valid throughout operation.
 * @note 构造时由驱动设置 UART 中断源，并把 UART 中断设为优先级 1；SysConfig 中该实例
 *       的中断和优先级设置会被覆盖。
 *       The constructor sets the UART interrupt sources and the UART interrupt priority
 *       (1), replacing the interrupt and priority settings SysConfig made for the
 *       instance.
 * @note 软件接收队列满时，Main 保留 FIFO 字节，Extend 丢弃放不下的 DMA 尾部。
 *       硬件仍持续接收；没有流控时不保证无限输入不丢失。
 *       On a full software queue, Main retains FIFO bytes and Extend drops excess DMA
 *       bytes. Hardware keeps receiving; unbounded input requires external flow control.
 */
class MSPM0UART : public UART
{
  friend class MSPM0UARTReadPort;

 public:
  /// 接收路径 / RX path.
  enum class RxMode : uint8_t
  {
    MAIN_BYTE_IRQ,  ///< 主 UART 字节中断 / Main UART byte interrupts.
    EXTEND_DMA,     ///< 扩展 UART DMA / Extend UART DMA.
  };

  /// 由 BSP 提供的 UART 与 DMA 绑定 / UART and DMA bindings supplied by the BSP.
  struct Resources
  {
    /// UART 寄存器 / UART registers.
    UART_Regs* instance;
    /// UART 中断号 / UART interrupt number.
    IRQn_Type irqn;
    /// UART 输入时钟频率 / UART input clock frequency.
    uint32_t clock_freq;
    /// ResolveIndex 返回的编号 / Index returned by ResolveIndex.
    uint8_t index;
    /// 选择的接收路径 / Selected RX path.
    RxMode rx_mode;
    /// 是否使用接收半传输中断 / Enable RX half-transfer interrupts.
    bool rx_half_interrupt;
    /// 发送 DMA 通道 / TX DMA channel.
    uint8_t dma_tx_channel;
    /// UART 发送 DMA 触发源 / UART TX DMA trigger.
    uint8_t dma_tx_trigger;
    /// 接收 FULL-DMA 通道，Main 使用无效编号 / RX full channel, invalid for Main.
    uint8_t dma_rx_channel;
    /// UART 接收 DMA 触发源，Main 为零 / RX DMA trigger, zero for Main.
    uint8_t dma_rx_trigger;
  };

  /// 无 DMA 通道的标记 / Sentinel for no DMA channel.
  static constexpr uint8_t INVALID_DMA_CHANNEL = 0xFFU;

  /**
   * @brief 构造串口并配置所选 DMA 路径 / Construct the UART and selected DMA paths.
   * @param res UART 与 DMA 资源绑定 / UART and DMA resource bindings.
   * @param tx_dma_storage 调用者提供的连续 2N 字节存储，分为两个 N 字节半区；
   *        地址和半区边界按 sizeof(size_t) 对齐，N 不超过 65535。
   *        Caller-owned contiguous 2N bytes, split into N-byte halves aligned to
   *        sizeof(size_t), with N no greater than 65535.
   * @param rx_dma_storage Main 传空视图；Extend 提供连续、偶数字节数的接收环。
   *        Empty for Main; a contiguous even-sized RX ring for Extend.
   * @param tx_queue_size 写请求队列容量，必须大于零 / Positive write request capacity.
   * @param rx_queue_capacity 软件接收队列字节容量，必须大于零 / Positive RX queue
   * capacity.
   * @param config 初始串口配置 / Initial UART configuration.
   * @note WritePort 字节容量为 N；配置错误和资源冲突在构造时检查。
   *       WritePort byte capacity is N; checks configuration and resource conflicts.
   */
  MSPM0UART(Resources res, RawData tx_dma_storage, RawData rx_dma_storage,
            uint32_t tx_queue_size = 5U, uint32_t rx_queue_capacity = 128U,
            UART::Configuration config = {115200, UART::Parity::NO_PARITY, 8U, 1U});

  /**
   * @brief 提交串口配置 / Submit a UART configuration.
   * @param config 目标配置 / Requested configuration.
   * @param in_isr 当前是否在中断中 / Whether called in an ISR.
   * @return 接纳返回 OK，配置槽被占用返回 BUSY，参数不可表示返回 ARG_ERR。
   *         OK on admission, BUSY for an occupied slot, ARG_ERR for unsupported values.
   * @note 等发送和 UART 空闲后生效；保留已发布的读数据和待发送请求。
   *       切换时丢弃硬件 FIFO 及未发布 DMA 接收数据。
   *       Applies after TX and UART become idle, preserving published RX and pending
   *       writes. Discards hardware FIFO and unpublished DMA RX data at the boundary.
   */
  ErrorCode SetConfig(UART::Configuration config, bool in_isr = false) override;

  /// 通知已提交的写请求 / Notify released write requests.
  static void WriteFun(WritePort& port, bool in_isr);
  /// 按实例编号分派 UART 中断 / Dispatch a UART IRQ by instance index.
  static void OnInterrupt(uint8_t index);

  /**
   * @brief 分派本驱动登记的接收 DMA 中断 / Dispatch registered UART RX DMA interrupts.
   * @note BSP 在共享 DMA 向量中调用本函数；其他 DMA 来源仍由 BSP 处理。
   *       只确认登记的 Extend RX 通道，不安装或接管共享向量。
   *       Called by the BSP shared DMA vector, which still handles other sources.
   *       Acknowledges only registered Extend RX channels; does not install the vector.
   */
  static void OnDmaInterrupt();

  /// 从已初始化硬件读取帧格式，使用给定波特率 / Read framing and use the supplied baud
  /// rate.
  static UART::Configuration BuildConfigFromSysCfg(UART_Regs* instance,
                                                   uint32_t baudrate);

  /// 获取构造时选择的接收路径 / Get the selected RX path.
  [[nodiscard]] RxMode GetRxMode() const { return res_.rx_mode; }
  /// 查询是否使用接收半传输通知 / Check RX half-transfer notification use.
  [[nodiscard]] bool RxHalfInterruptEnabled() const { return res_.rx_half_interrupt; }

  MSPM0UARTReadPort _read_port;  // NOLINT(readability-identifier-naming)
  WritePort _write_port;         // NOLINT(readability-identifier-naming)

  /// 根据 UART 中断号取得实例编号 / Resolve an instance index from its UART IRQ.
  static constexpr uint8_t ResolveIndex(IRQn_Type irqn)
  {
    switch (irqn)
    {
#if defined(UART0_BASE)
      case UART0_INT_IRQn:
        return 0U;
#endif
#if defined(UART1_BASE)
      case UART1_INT_IRQn:
        return 1U;
#endif
#if defined(UART2_BASE)
      case UART2_INT_IRQn:
        return 2U;
#endif
#if defined(UART3_BASE)
      case UART3_INT_IRQn:
        return 3U;
#endif
#if defined(UART4_BASE)
      case UART4_INT_IRQn:
        return 4U;
#endif
#if defined(UART5_BASE)
      case UART5_INT_IRQn:
        return 5U;
#endif
#if defined(UART6_BASE)
      case UART6_INT_IRQn:
        return 6U;
#endif
#if defined(UART7_BASE)
      case UART7_INT_IRQn:
        return 7U;
#endif
      default:
        return INVALID_DMA_CHANNEL;
    }
  }

 private:
  /// 配置槽的发布状态 / Configuration slot publication state.
  enum class ConfigState : uint32_t
  {
    EMPTY = 0U,
    RESERVED = 1U,
    PUBLISHED = 2U,
  };

  static constexpr uint8_t MAX_UART_INSTANCES = 8U;
  static_assert(DMA_SYS_N_DMA_CHANNEL <= 16U);
  static constexpr uint8_t MAX_DMA_CHANNELS = static_cast<uint8_t>(DMA_SYS_N_DMA_CHANNEL);
  static constexpr uint32_t MSPM0_UART_DMA_MAX_TRANSFER_SIZE = 0xFFFFU;

  static constexpr uint32_t EVENT_WRITE = 1U << 0U;
  static constexpr uint32_t EVENT_DMA_DONE_TX = 1U << 1U;
  static constexpr uint32_t EVENT_EOT_DONE = 1U << 2U;
  static constexpr uint32_t EVENT_CONFIG = 1U << 3U;
  static constexpr uint32_t EVENT_RX_WORK = 1U << 4U;

  static constexpr uint32_t RX_ERROR_INTERRUPT_MASK =
      DL_UART_INTERRUPT_OVERRUN_ERROR | DL_UART_INTERRUPT_BREAK_ERROR |
      DL_UART_INTERRUPT_PARITY_ERROR | DL_UART_INTERRUPT_FRAMING_ERROR |
      DL_UART_INTERRUPT_NOISE_ERROR;

  static constexpr uint32_t RX_INTERRUPT_MASK = DL_UART_INTERRUPT_RX;
  static constexpr uint32_t RX_TIMEOUT_INTERRUPT_MASK =
      DL_UART_INTERRUPT_RX_TIMEOUT_ERROR;
  static constexpr uint32_t RX_GAP_INTERRUPT_MASK = DL_UART_INTERRUPT_LINC0_MATCH;
  static constexpr uint32_t TX_DONE_INTERRUPT_MASK = DL_UART_INTERRUPT_DMA_DONE_TX;
  static constexpr uint32_t EOT_INTERRUPT_MASK = DL_UART_INTERRUPT_EOT_DONE;

  /// 串行处理收发和配置进展 / Serialize RX, TX, and configuration progress.
  void HandleService(uint32_t events, bool in_isr);
  /// 填充空闲发送半区并按需启动 DMA / Fill free TX halves and start DMA as needed.
  void FillTx(bool in_isr);
  /// 释放发送半区并切换已准备数据 / Release the sent half and switch prepared data.
  void HandleTxDone(bool in_isr);
  /// 按所选模式推进接收 / Advance the selected RX path.
  void HandleRxWork(bool in_isr);
  /// 软件队列有空间时才读取 RXDATA / Read RXDATA only when software space exists.
  void HandleMainRx(bool in_isr);
  /// 复制 DMA 接收前缀，推进游标后发布 / Copy DMA RX, advance the cursor, then publish.
  void HandleExtendRx(bool in_isr);
  /// 通知接收空间释放 / Notify RX space release.
  void NotifyReadSpace(bool in_isr);
  /// 确认 UART 中断快照并发布进展 / Acknowledge UART status and publish progress.
  void HandleUartInterrupt();
  /// 发布接收 DMA 进展 / Publish RX DMA progress.
  void HandleRxDmaInterrupt();

  /// 验证帧格式、波特率和间隔计数范围 / Validate framing, baud, and gap-counter range.
  [[nodiscard]] ErrorCode ValidateConfig(UART::Configuration config) const;
  /// 在处理器内应用已验证参数 / Apply validated parameters inside the service.
  void ApplyConfig(UART::Configuration config);
  /// UART 空闲时应用配置并恢复数据路径 / Apply at UART idle and resume data paths.
  void TryApplyPublishedConfig(bool in_isr);
  /// 配置发送 DMA 通道 / Configure the TX DMA channel.
  void ConfigureTxDma();
  /// 配置 Extend 循环接收 DMA / Configure Extend circular RX DMA.
  void ConfigureRxDma();
  /// 开启所选接收路径和 UART / Enable the selected RX path and UART.
  void StartDataPath();
  /// 暂停收发请求源及选定的接收 DMA / Stop request sources and selected RX DMA.
  void StopDataPath();
  /// 配置切换时丢弃未读硬件字节 / Discard unread hardware bytes during configuration.
  void DiscardRxFifo();
  /// 启动指定半区，配置后屏障保证数据可见 / Start a half after the publication barrier.
  void StartTxDma(uint8_t half, size_t size);

  /// 获取发送半区地址 / Get a TX half address.
  [[nodiscard]] uint8_t* TxHalf(uint8_t half) const
  {
    return static_cast<uint8_t*>(tx_dma_storage_.addr_) +
           static_cast<size_t>(half) * tx_half_size_;
  }

  /// 获取接收 DMA 环容量 / Get RX DMA ring capacity.
  [[nodiscard]] size_t RxCapacity() const { return rx_dma_storage_.size_; }

  /// 生成 DMA 完成中断掩码 / Build a DMA completion mask.
  static uint32_t DmaCompleteMask(uint8_t channel)
  {
    return channel < MAX_DMA_CHANNELS ? (1UL << channel) : 0U;
  }

  /// 生成前八个通道的提前中断掩码 / Build an early-interrupt mask for channels 0–7.
  static uint32_t DmaEarlyMask(uint8_t channel)
  {
    return channel < 8U ? (1UL << (16U + channel)) : 0U;
  }

  /// 组合完成和可选半传输掩码 / Combine completion and optional half-transfer masks.
  static uint32_t DmaRawMask(uint8_t channel, bool half)
  {
    return DmaCompleteMask(channel) | (half ? DmaEarlyMask(channel) : 0U);
  }

  /// 固定资源绑定 / Fixed resource bindings.
  Resources res_{};
  /// 借用的发送双缓冲 / Borrowed TX double buffer.
  RawData tx_dma_storage_{};
  /// 借用的 Extend 接收环 / Borrowed Extend RX ring.
  RawData rx_dma_storage_{};
  /// 单个发送半区容量 / TX half capacity.
  size_t tx_half_size_ = 0U;
  /// 每半区有效长度，零表示空闲 / Per-half length; zero means free.
  std::array<size_t, 2U> tx_half_size_used_{};
  /// 当前发送半区，负值表示 DMA 空闲 / Active half, negative when DMA is idle.
  int8_t active_half_ = -1;
  /// 已启动发送且尚未收到 EOT，最后的字节可能仍在发送 / TX was started and EOT has not
  /// been seen yet, so the last bytes may still be shifting out.
  bool tx_draining_ = false;
  /// 上次已处理的接收位置 / Last processed RX position.
  size_t rx_dma_cursor_ = 0U;

  /// 唯一收发与配置处理器 / Sole I/O and configuration service.
  SerializedService service_{};
  /// 配置发布与占用状态 / Configuration publication and occupancy.
  std::atomic<ConfigState> config_state_{ConfigState::EMPTY};
  /// 等待应用的配置 / Configuration awaiting application.
  UART::Configuration pending_config_{};

  /// 初始化时登记的中断路由表 / IRQ routing table populated at initialization.
  static MSPM0UART* instance_map_[MAX_UART_INSTANCES];
};

namespace Detail
{

// 以下构造辅助函数检查 BSP 声明的 DMA 归属，不增加运行时资源状态。
// These helpers check BSP-declared DMA ownership without adding runtime state.
template <IRQn_Type UartIrqn, uint32_t TxDmaChannel, IRQn_Type TxDmaOwnerIrqn,
          bool TxBinding>
MSPM0UART::Resources MakeMSPM0MainUartResources(UART_Regs* instance, uint32_t clock_freq,
                                                uint8_t tx_dma_trigger)
{
  static_assert(TxBinding);
  static_assert(TxDmaOwnerIrqn == UartIrqn);
  static_assert(TxDmaChannel < DMA_SYS_N_DMA_CHANNEL);
  return {instance,
          UartIrqn,
          clock_freq,
          MSPM0UART::ResolveIndex(UartIrqn),
          MSPM0UART::RxMode::MAIN_BYTE_IRQ,
          false,
          static_cast<uint8_t>(TxDmaChannel),
          tx_dma_trigger,
          MSPM0UART::INVALID_DMA_CHANNEL,
          0U};
}

template <IRQn_Type UartIrqn, uint32_t TxDmaChannel, IRQn_Type TxDmaOwnerIrqn,
          bool TxBinding, uint32_t RxDmaChannel, IRQn_Type RxDmaOwnerIrqn, bool RxBinding,
          bool ExtendCapable, bool FullRxChannel, bool HalfRxInterrupt>
MSPM0UART::Resources MakeMSPM0ExtendUartResources(UART_Regs* instance,
                                                  uint32_t clock_freq,
                                                  uint8_t tx_dma_trigger,
                                                  uint8_t rx_dma_trigger)
{
  static_assert(TxBinding);
  static_assert(RxBinding);
  static_assert(TxDmaOwnerIrqn == UartIrqn);
  static_assert(RxDmaOwnerIrqn == UartIrqn);
  static_assert(TxDmaChannel < DMA_SYS_N_DMA_CHANNEL);
  static_assert(RxDmaChannel < DMA_SYS_N_DMA_FULL_CHANNEL);
  static_assert(TxDmaChannel != RxDmaChannel);
  static_assert(ExtendCapable);
  static_assert(FullRxChannel);
  static_assert(!HalfRxInterrupt || RxDmaChannel < 8U);
  return {instance,
          UartIrqn,
          clock_freq,
          MSPM0UART::ResolveIndex(UartIrqn),
          MSPM0UART::RxMode::EXTEND_DMA,
          HalfRxInterrupt,
          static_cast<uint8_t>(TxDmaChannel),
          tx_dma_trigger,
          static_cast<uint8_t>(RxDmaChannel),
          rx_dma_trigger};
}

}  // namespace Detail

#define LIBXR_MSPM0_UART_CAT_IMPL(left, right) left##right
#define LIBXR_MSPM0_UART_CAT(left, right) LIBXR_MSPM0_UART_CAT_IMPL(left, right)
#define LIBXR_MSPM0_UART_PROPERTY(name, suffix) LIBXR_MSPM0_UART_CAT(name, suffix)
#define LIBXR_MSPM0_UART_DMA_CHANNEL(name) LIBXR_MSPM0_UART_CAT(name, _CHAN_ID)
#define LIBXR_MSPM0_UART_DMA_PROPERTY(name, suffix) LIBXR_MSPM0_UART_CAT(name, suffix)
#define LIBXR_MSPM0_UART_DMA_TRIGGER_IMPL(instance, direction) \
  DMA_##instance##_##direction##_TRIG
#define LIBXR_MSPM0_UART_DMA_TRIGGER(instance, direction) \
  LIBXR_MSPM0_UART_DMA_TRIGGER_IMPL(instance, direction)

/**
 * @brief 使用 SysConfig 名称构造 Main 资源参数 / Build Main arguments from SysConfig
 * names.
 *
 * BSP 须提供 tx_dma_CHAN_ID、tx_dma_LIBXR_UART_IRQN、tx_dma_LIBXR_UART_TX 标注。
 * _LIBXR_ 标注由 BSP 声明，不是 TI SysConfig 默认生成项。可直接使用 Resources 构造。
 * The BSP supplies tx_dma_CHAN_ID, tx_dma_LIBXR_UART_IRQN and tx_dma_LIBXR_UART_TX.
 * _LIBXR_ annotations are BSP declarations, not default TI-generated fields.
 * Direct construction with Resources is also available.
 */
#define MSPM0_UART_MAIN_INIT(name, tx_dma, tx_storage_addr, tx_storage_size,             \
                             tx_queue_size, rx_queue_capacity)                           \
  ::LibXR::Detail::MakeMSPM0MainUartResources<                                           \
      name##_INST_INT_IRQN, static_cast<uint32_t>(LIBXR_MSPM0_UART_DMA_CHANNEL(tx_dma)), \
      LIBXR_MSPM0_UART_DMA_PROPERTY(tx_dma, _LIBXR_UART_IRQN),                           \
      static_cast<bool>(LIBXR_MSPM0_UART_DMA_PROPERTY(tx_dma, _LIBXR_UART_TX))>(         \
      name##_INST, name##_INST_FREQUENCY,                                                \
      static_cast<uint8_t>(LIBXR_MSPM0_UART_DMA_TRIGGER(name##_INST, TX))),              \
      ::LibXR::RawData{static_cast<void*>(tx_storage_addr), (tx_storage_size)},          \
      ::LibXR::RawData{}, (tx_queue_size), (rx_queue_capacity),                          \
      ::LibXR::MSPM0UART::BuildConfigFromSysCfg(name##_INST,                             \
                                                static_cast<uint32_t>(name##_BAUD_RATE))

/**
 * @brief 使用 SysConfig 名称构造 Extend 资源参数 / Build Extend arguments from SysConfig
 * names.
 *
 * 除 Main 的发送标注外，BSP 须声明 rx_dma_CHAN_ID、rx_dma_LIBXR_UART_IRQN、
 * rx_dma_LIBXR_UART_RX、rx_dma_LIBXR_FULL_CHANNEL、rx_dma_LIBXR_HALF_INTERRUPT，
 * 以及 name_LIBXR_EXTEND_CAPABLE。收发通道须不同，RX 必须为 FULL-DMA 通道。
 * In addition to TX annotations, declare RX channel, UART ownership, RX binding,
 * FULL-channel and half-interrupt annotations, plus name_LIBXR_EXTEND_CAPABLE.
 * TX and RX channels must differ, and RX requires a FULL-DMA channel.
 */
#define MSPM0_UART_EXTEND_INIT(name, tx_dma, rx_dma, tx_storage_addr, tx_storage_size,   \
                               tx_queue_size, rx_dma_storage_addr, rx_dma_storage_size,  \
                               rx_queue_capacity)                                        \
  ::LibXR::Detail::MakeMSPM0ExtendUartResources<                                         \
      name##_INST_INT_IRQN, static_cast<uint32_t>(LIBXR_MSPM0_UART_DMA_CHANNEL(tx_dma)), \
      LIBXR_MSPM0_UART_DMA_PROPERTY(tx_dma, _LIBXR_UART_IRQN),                           \
      static_cast<bool>(LIBXR_MSPM0_UART_DMA_PROPERTY(tx_dma, _LIBXR_UART_TX)),          \
      static_cast<uint32_t>(LIBXR_MSPM0_UART_DMA_CHANNEL(rx_dma)),                       \
      LIBXR_MSPM0_UART_DMA_PROPERTY(rx_dma, _LIBXR_UART_IRQN),                           \
      static_cast<bool>(LIBXR_MSPM0_UART_DMA_PROPERTY(rx_dma, _LIBXR_UART_RX)),          \
      static_cast<bool>(LIBXR_MSPM0_UART_PROPERTY(name, _LIBXR_EXTEND_CAPABLE)),         \
      static_cast<bool>(LIBXR_MSPM0_UART_DMA_PROPERTY(rx_dma, _LIBXR_FULL_CHANNEL)),     \
      static_cast<bool>(LIBXR_MSPM0_UART_DMA_PROPERTY(rx_dma, _LIBXR_HALF_INTERRUPT))>(  \
      name##_INST, name##_INST_FREQUENCY,                                                \
      static_cast<uint8_t>(LIBXR_MSPM0_UART_DMA_TRIGGER(name##_INST, TX)),               \
      static_cast<uint8_t>(LIBXR_MSPM0_UART_DMA_TRIGGER(name##_INST, RX))),              \
      ::LibXR::RawData{static_cast<void*>(tx_storage_addr), (tx_storage_size)},          \
      ::LibXR::RawData{static_cast<void*>(rx_dma_storage_addr), (rx_dma_storage_size)},  \
      (tx_queue_size), (rx_queue_capacity),                                              \
      ::LibXR::MSPM0UART::BuildConfigFromSysCfg(name##_INST,                             \
                                                static_cast<uint32_t>(name##_BAUD_RATE))

}  // namespace LibXR
