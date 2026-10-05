#include "esp_uart.hpp"

#include <algorithm>
#include <cstring>

#include "esp_attr.h"
#include "esp_clk_tree.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_private/periph_ctrl.h"
#include "esp_rom_gpio.h"
#include "hal/uart_ll.h"
#include "soc/gpio_sig_map.h"
#include "soc/uart_periph.h"

namespace
{
// FIFO 接收路径处理的 RX 中断原因。
// RX interrupt reasons handled by the FIFO receive path.
constexpr uint32_t UART_RX_INTR_MASK =
    UART_INTR_RXFIFO_FULL | UART_INTR_RXFIFO_TOUT | UART_INTR_RXFIFO_OVF;

// FIFO 发送路径使用的 TX 中断原因。
// TX interrupt reason used by the FIFO transmit path.
constexpr uint32_t UART_TX_INTR_MASK = UART_INTR_TXFIFO_EMPTY;
constexpr uint32_t UART_TX_IDLE_INTR_MASK = UART_INTR_TX_DONE;

// 使用最小非零 timeout，让短 RX 尾包尽量贴近 idle 风格边界被冲刷出来。
// Use the minimum non-zero timeout so short RX tails are flushed as close as
// possible to an idle-style boundary.
constexpr uint8_t RX_TOUT_THRESHOLD = 1;

// 当硬件 FIFO 下降到约一半深度时，请求补充更多 TX 字节。
// Ask for more TX bytes once the hardware FIFO drops to roughly half depth.
constexpr uint16_t TX_EMPTY_THRESHOLD = SOC_UART_FIFO_LEN / 2U;

// 该后端不能与 ESP-IDF 的控制台 UART 占用同时存在。
// This backend cannot coexist with the ESP-IDF console UART reservation.
bool IsConsoleUartInUse(uart_port_t uart_num)
{
#if defined(CONFIG_ESP_CONSOLE_UART) && CONFIG_ESP_CONSOLE_UART
  return static_cast<int>(uart_num) == CONFIG_ESP_CONSOLE_UART_NUM;
#else
  (void)uart_num;
  return false;
#endif
}

bool IsBaudrateRepresentable(uint32_t baudrate, uint32_t source_clock_hz)
{
  if ((baudrate == 0U) || (source_clock_hz == 0U))
  {
    return false;
  }

#if defined(UART_SCLK_DIV_NUM_V) || defined(PCR_UART0_SCLK_DIV_NUM_V) || \
    defined(HP_SYS_CLKRST_REG_UART0_SCLK_DIV_NUM_V)
#if defined(UART_SCLK_DIV_NUM_V)
  constexpr uint32_t max_source_divider = UART_SCLK_DIV_NUM_V + 1U;
#elif defined(PCR_UART0_SCLK_DIV_NUM_V)
  constexpr uint32_t max_source_divider = PCR_UART0_SCLK_DIV_NUM_V + 1U;
#else
  constexpr uint32_t max_source_divider = HP_SYS_CLKRST_REG_UART0_SCLK_DIV_NUM_V + 1U;
#endif

  const uint64_t max_uart_divider = UART_CLKDIV_V;
  const uint64_t denominator = max_uart_divider * baudrate;
  const uint64_t source_divider =
      (static_cast<uint64_t>(source_clock_hz) + denominator - 1U) / denominator;
  if ((source_divider == 0U) || (source_divider > max_source_divider))
  {
    return false;
  }

  const uint64_t clock_divider = (static_cast<uint64_t>(source_clock_hz) << 4U) /
                                 (static_cast<uint64_t>(baudrate) * source_divider);
  return (clock_divider >> 4U) <= max_uart_divider;
#else
  const uint64_t clock_divider =
      (static_cast<uint64_t>(source_clock_hz) << 4U) / baudrate;
  return (clock_divider >> 4U) <= UART_CLKDIV_V;
#endif
}

bool GetUartClockFrequency(uint32_t& out_frequency_hz)
{
  out_frequency_hz = 0U;
  return (esp_clk_tree_src_get_freq_hz(static_cast<soc_module_clk_t>(UART_SCLK_DEFAULT),
                                       ESP_CLK_TREE_SRC_FREQ_PRECISION_CACHED,
                                       &out_frequency_hz) == ESP_OK) &&
         (out_frequency_hz != 0U);
}
}  // namespace

namespace LibXR
{

void ESP32UARTReadPort::OnReadQueueSpaceAvailable(bool in_isr)
{
  owner_.service_.Invoke(ESP32UART::EVENT_RX_WORK, in_isr,
                         [this](uint32_t events, bool owner_isr)
                         { owner_.ServiceEvents(events, owner_isr); });
}

// 优先使用对齐的 DMA 可访问分配；若失败，再退回到更宽松的 DMA heap。
// Prefer aligned DMA-capable allocation first, then fall back to the broader
// DMA-capable heap if the strict aligned allocation API is unavailable.
uint8_t* ESP32UART::AllocateTxStorage(size_t size)
{
  void* aligned = heap_caps_aligned_alloc(
      4, size, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
  if (aligned != nullptr)
  {
    return static_cast<uint8_t*>(aligned);
  }

  return static_cast<uint8_t*>(
      heap_caps_malloc(size, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA | MALLOC_CAP_8BIT));
}

// 将公开 UART 序号转换为 ESP 外设模块门控对象。
// Translate the public UART index into the ESP peripheral module gate.
ErrorCode ESP32UART::ResolveUartPeriph(uart_port_t uart_num, periph_module_t& out)
{
  switch (uart_num)
  {
    case UART_NUM_0:
      out = PERIPH_UART0_MODULE;
      return ErrorCode::OK;
    case UART_NUM_1:
      out = PERIPH_UART1_MODULE;
      return ErrorCode::OK;
#if SOC_UART_HP_NUM > 2
    case UART_NUM_2:
      out = PERIPH_UART2_MODULE;
      return ErrorCode::OK;
#endif
    default:
      return ErrorCode::NOT_SUPPORT;
  }
}

// 先构造队列连接，再把 storage 绑定到 TX 双缓冲视图，最后再触碰硬件。
// Construct queue plumbing first, then bind the storage into the TX
// double-buffer view before touching hardware.
ESP32UART::ESP32UART(uart_port_t uart_num, int tx_pin, int rx_pin, int rts_pin,
                     int cts_pin, size_t rx_buffer_size, size_t tx_buffer_size,
                     uint32_t tx_queue_size, UART::Configuration config, bool enable_dma)
    : UART(&_read_port, &_write_port),
      uart_num_(uart_num),
      tx_pin_(tx_pin),
      rx_pin_(rx_pin),
      rts_pin_(rts_pin),
      cts_pin_(cts_pin),
      config_(config),
      rx_isr_buffer_(new uint8_t[rx_buffer_size]),
      rx_isr_buffer_size_(rx_buffer_size),
      tx_storage_(AllocateTxStorage(tx_buffer_size * 2)),
      dma_requested_(enable_dma),
      _read_port(rx_buffer_size, *this),
      _write_port(tx_queue_size, tx_buffer_size)
{
  ASSERT(!IsConsoleUartInUse(uart_num_));
  ASSERT(uart_num_ < UART_NUM_MAX);
  ASSERT(uart_num_ < SOC_UART_HP_NUM);
  ASSERT(rx_isr_buffer_size_ > 0);
  ASSERT(tx_buffer_size > 0);
  ASSERT(tx_queue_size > 0U);

  INIT_CRIT_SECTION_LOCK_RUNTIME(&irq_lock_);

  tx_dma_buffer_.Init({tx_storage_, tx_buffer_size * 2U});

  _write_port = WriteFun;

  if (InitUartHardware() != ErrorCode::OK)
  {
    REQUIRE(false);
    return;
  }

#if SOC_GDMA_SUPPORTED && SOC_UHCI_SUPPORTED
  if (dma_requested_)
  {
    if (InitDmaBackend() != ErrorCode::OK)
    {
      REQUIRE(false);
      return;
    }
  }

  if (!dma_backend_enabled_)
  {
    if (InstallUartIsr() != ErrorCode::OK)
    {
      REQUIRE(false);
      return;
    }
    ConfigureRxInterruptPath();
  }
#else
  if (InstallUartIsr() != ErrorCode::OK)
  {
    REQUIRE(false);
    return;
  }
  ConfigureRxInterruptPath();
#endif
}

// FIFO 模式下使用半 FIFO 的 RX 阈值，剩余短尾包交给上面的最小非零 RX timeout。
// Use a half-FIFO RX threshold in FIFO mode. Short residual tails are handled
// by the minimum non-zero RX timeout above.
void ESP32UART::ConfigureRxInterruptPath()
{
  const uint16_t full_thr = static_cast<uint16_t>(SOC_UART_FIFO_LEN / 2U);

  uart_hal_set_rxfifo_full_thr(&uart_hal_, full_thr);
  uart_hal_set_rx_timeout(&uart_hal_, RX_TOUT_THRESHOLD);
  ClearUartInterrupt(UART_RX_INTR_MASK);
  EnableUartInterrupt(UART_RX_INTR_MASK);
}

// 验证并发布单个待应用配置，在当前传输模式的边界应用。
// Validate and publish one configuration for the mode-specific transfer boundary.
ErrorCode ESP32UART::SetConfig(UART::Configuration config, bool in_isr)
{
  if (!uart_hw_enabled_)
  {
    return ErrorCode::STATE_ERR;
  }

  uart_word_length_t word_length = UART_DATA_8_BITS;
  uart_stop_bits_t stop_bits = UART_STOP_BITS_1;
  if (config.baudrate == 0U || !ResolveWordLength(config.data_bits, word_length) ||
      !ResolveStopBits(config.stop_bits, stop_bits) ||
      (config.parity != UART::Parity::NO_PARITY && config.parity != UART::Parity::EVEN &&
       config.parity != UART::Parity::ODD))
  {
    return ErrorCode::ARG_ERR;
  }

  // 使用初始化时的时钟快照，校验时不访问硬件。
  // Validate against the initial clock snapshot without touching hardware.
  if (!IsBaudrateRepresentable(config.baudrate, uart_sclk_hz_))
  {
    return ErrorCode::ARG_ERR;
  }

  uint8_t expected = static_cast<uint8_t>(ConfigState::EMPTY);
  if (!config_state_.compare_exchange_strong(
          expected, static_cast<uint8_t>(ConfigState::RESERVED),
          std::memory_order_acq_rel, std::memory_order_acquire))
  {
    return ErrorCode::BUSY;
  }

  requested_config_ = config;
  config_state_.store(static_cast<uint8_t>(ConfigState::PUBLISHED),
                      std::memory_order_release);
  service_.Invoke(EVENT_CONFIG, in_isr, [this](uint32_t events, bool owner_isr)
                  { ServiceEvents(events, owner_isr); });
  return ErrorCode::OK;
}

ErrorCode ESP32UART::ApplyConfig(const UART::Configuration& config)
{
  if (!uart_hw_enabled_)
  {
    return ErrorCode::STATE_ERR;
  }

  uart_word_length_t word_length = UART_DATA_8_BITS;
  uart_stop_bits_t stop_bits = UART_STOP_BITS_1;
  [[maybe_unused]] const auto resolve_word_length_result =
      ResolveWordLength(config.data_bits, word_length);
  DEV_ASSERT(resolve_word_length_result);
  [[maybe_unused]] const auto resolve_stop_bits_result =
      ResolveStopBits(config.stop_bits, stop_bits);
  DEV_ASSERT(resolve_stop_bits_result);

  const uart_sclk_t sclk = UART_SCLK_DEFAULT;
  uart_hal_set_sclk(&uart_hal_, static_cast<soc_module_clk_t>(sclk));

  if (uart_sclk_hz_ == 0U)
  {
    return ErrorCode::INIT_ERR;
  }

  if (!uart_hal_set_baudrate(&uart_hal_, config.baudrate, uart_sclk_hz_))
  {
    return ErrorCode::INIT_ERR;
  }

  uart_hal_set_data_bit_num(&uart_hal_, word_length);
  uart_hal_set_stop_bits(&uart_hal_, stop_bits);
  uart_hal_set_parity(&uart_hal_, ResolveParity(config.parity));
  uart_hal_set_hw_flow_ctrl(&uart_hal_, UART_HW_FLOWCTRL_DISABLE, 0);
  uart_hal_set_mode(&uart_hal_, UART_MODE_UART);
  uart_hal_set_txfifo_empty_thr(&uart_hal_, TX_EMPTY_THRESHOLD);
  uart_hal_rxfifo_rst(&uart_hal_);
  ClearUartInterrupt(UART_INTR_RXFIFO_FULL | UART_INTR_RXFIFO_TOUT);
  return ErrorCode::OK;
}

void IRAM_ATTR ESP32UART::EnableUartInterrupt(uint32_t mask)
{
  esp_os_enter_critical_safe(&irq_lock_);
  uart_hal_ena_intr_mask(&uart_hal_, mask);
  esp_os_exit_critical_safe(&irq_lock_);
}

void IRAM_ATTR ESP32UART::DisableUartInterrupt(uint32_t mask)
{
  esp_os_enter_critical_safe(&irq_lock_);
  uart_hal_disable_intr_mask(&uart_hal_, mask);
  esp_os_exit_critical_safe(&irq_lock_);
}

void IRAM_ATTR ESP32UART::ClearUartInterrupt(uint32_t mask)
{
  esp_os_enter_critical_safe(&irq_lock_);
  uart_hal_clr_intsts_mask(&uart_hal_, mask);
  esp_os_exit_critical_safe(&irq_lock_);
}

void IRAM_ATTR ESP32UART::DisableAndClearUartInterrupt(uint32_t mask)
{
  esp_os_enter_critical_safe(&irq_lock_);
  uart_hal_disable_intr_mask(&uart_hal_, mask);
  uart_hal_clr_intsts_mask(&uart_hal_, mask);
  esp_os_exit_critical_safe(&irq_lock_);
}

bool IRAM_ATTR ESP32UART::TryApplyPublishedConfig(bool in_isr)
{
  if (config_state_.load(std::memory_order_acquire) !=
      static_cast<uint8_t>(ConfigState::PUBLISHED))
  {
    return false;
  }

  bool tx_idle = !tx_busy_.IsSet() && !tx_active_valid_
#if SOC_GDMA_SUPPORTED && SOC_UHCI_SUPPORTED
                 && !tx_dma_buffer_.HasPending()
#endif
      ;
#if SOC_GDMA_SUPPORTED && SOC_UHCI_SUPPORTED
  if (!dma_backend_enabled_)
#endif
  {
    tx_idle = tx_idle && (_write_port.Size() == 0U) && uart_hal_is_tx_idle(&uart_hal_);
  }

  if (!tx_idle)
  {
#if SOC_GDMA_SUPPORTED && SOC_UHCI_SUPPORTED
    if (!dma_backend_enabled_)
#endif
    {
      // Keep a line-idle carrier armed while FIFO data is draining. The source
      // is disabled again when the published configuration is applied.
      EnableUartInterrupt(UART_TX_IDLE_INTR_MASK);
    }
    return false;
  }

#if SOC_GDMA_SUPPORTED && SOC_UHCI_SUPPORTED
  if (!dma_backend_enabled_)
#endif
  {
    DisableAndClearUartInterrupt(UART_TX_IDLE_INTR_MASK);
  }

#if SOC_GDMA_SUPPORTED && SOC_UHCI_SUPPORTED
  if (dma_backend_enabled_)
  {
    // 在 ApplyConfig 重置 UART RX FIFO 前，先停止 RX DMA 消费者。
    // Stop the RX DMA consumer before ApplyConfig resets the UART RX FIFO.
    StopDmaRx(in_isr);
  }
#endif
  [[maybe_unused]] const auto apply_config_result = ApplyConfig(requested_config_);
  REQUIRE_FROM_CALLBACK(apply_config_result == ErrorCode::OK, in_isr);
  config_ = requested_config_;
  config_state_.store(static_cast<uint8_t>(ConfigState::EMPTY),
                      std::memory_order_release);
#if SOC_GDMA_SUPPORTED && SOC_UHCI_SUPPORTED
  if (dma_backend_enabled_)
  {
    ResetDmaRxDescriptors(in_isr);
    StartDmaRx(in_isr);
  }
#endif
  return true;
}

void IRAM_ATTR ESP32UART::ServiceEvents(uint32_t events, bool in_isr)
{
  if ((events & EVENT_RX_ERROR) != 0U)
  {
#if SOC_GDMA_SUPPORTED && SOC_UHCI_SUPPORTED
    if (dma_backend_enabled_)
    {
      HandleDmaRxError(in_isr);
    }
#endif
  }

  const bool rx_error = (events & EVENT_RX_ERROR) != 0U;
  const bool tx_error = (events & EVENT_TX_ERROR) != 0U;
  if (tx_error)
  {
#if SOC_GDMA_SUPPORTED && SOC_UHCI_SUPPORTED
    if (dma_backend_enabled_)
    {
      HandleDmaTxError(in_isr);
    }
#endif
  }

  if (!tx_error && (events & EVENT_TX_DONE) != 0U)
  {
    OnTxTransferDone(in_isr, ErrorCode::OK);
  }

  if ((events & (EVENT_WRITE | EVENT_TX_DONE | EVENT_TX_ERROR | EVENT_TX_WORK |
                 EVENT_CONFIG)) != 0U)
  {
    (void)TryStartTx(in_isr);
  }

  // Re-evaluate after TX progress. In particular, a coalesced CONFIG/TX_DONE
  // snapshot may become idle only after OnTxTransferDone().
  if (TryApplyPublishedConfig(in_isr))
  {
    // A GDMA queue front intentionally stayed in WriteQueue behind the
    // configuration barrier; resume ordinary loading under the new config.
    (void)TryStartTx(in_isr);
  }

  if (!rx_error && (events & EVENT_RX_WORK) != 0U)
  {
#if SOC_GDMA_SUPPORTED && SOC_UHCI_SUPPORTED
    if (dma_backend_enabled_)
    {
      HandleDmaRxDone(nullptr, in_isr);
    }
    else
#endif
    {
      DrainRxFifo(in_isr);
    }
  }
}

// UART 内部环回直接作为外设开关暴露，用于后端自测和无需外部短接的链路检查。
// Internal UART loopback is exposed as a direct peripheral toggle for backend
// self-test and board-free link checks.
ErrorCode ESP32UART::SetLoopback(bool enable)
{
  if (!uart_hw_enabled_)
  {
    return ErrorCode::STATE_ERR;
  }

  uart_ll_set_loop_back(uart_hal_.dev, enable);
  return ErrorCode::OK;
}

// `WritePort` 只需要一个回跳到所属 UART 实例的跳板。
// `WritePort` only needs a trampoline back into the owning UART instance.
void IRAM_ATTR ESP32UART::WriteFun(WritePort& port, bool in_isr)
{
  auto* uart = LibXR::ContainerOf(&port, &ESP32UART::_write_port);
  uart->service_.Invoke(ESP32UART::EVENT_WRITE, in_isr,
                        [uart](uint32_t events, bool owner_isr)
                        { uart->ServiceEvents(events, owner_isr); });
}

// 将 libxr 数据位语义转换为 ESP HAL 取值。
// Convert libxr data-bit semantics into the ESP HAL value set.
bool ESP32UART::ResolveWordLength(uint8_t data_bits, uart_word_length_t& out)
{
  switch (data_bits)
  {
    case 5:
      out = UART_DATA_5_BITS;
      return true;
    case 6:
      out = UART_DATA_6_BITS;
      return true;
    case 7:
      out = UART_DATA_7_BITS;
      return true;
    case 8:
      out = UART_DATA_8_BITS;
      return true;
    default:
      return false;
  }
}

// 将 libxr 停止位语义转换为 ESP HAL 取值。
// Convert libxr stop-bit semantics into the ESP HAL value set.
bool ESP32UART::ResolveStopBits(uint8_t stop_bits, uart_stop_bits_t& out)
{
  switch (stop_bits)
  {
    case 1:
      out = UART_STOP_BITS_1;
      return true;
    case 2:
      out = UART_STOP_BITS_2;
      return true;
    default:
      return false;
  }
}

// 将 libxr 校验位语义转换为 ESP HAL 取值。
// Convert libxr parity semantics into the ESP HAL value set.
uart_parity_t ESP32UART::ResolveParity(UART::Parity parity)
{
  switch (parity)
  {
    case UART::Parity::NO_PARITY:
      return UART_PARITY_DISABLE;
    case UART::Parity::EVEN:
      return UART_PARITY_EVEN;
    case UART::Parity::ODD:
      return UART_PARITY_ODD;
    default:
      return UART_PARITY_DISABLE;
  }
}

// 在挂接更高层 ISR 或 DMA 连接前，先把 UART 模块拉到已知空闲状态。
// Bring the UART block into a known idle state before higher-level ISR or DMA
// plumbing is attached.
ErrorCode ESP32UART::InitUartHardware()
{
  if (uart_num_ >= UART_NUM_MAX)
  {
    return ErrorCode::NOT_SUPPORT;
  }

  periph_module_t uart_module = PERIPH_MODULE_MAX;
  if (ResolveUartPeriph(uart_num_, uart_module) != ErrorCode::OK)
  {
    return ErrorCode::NOT_SUPPORT;
  }

  uart_hal_.dev = UART_LL_GET_HW(uart_num_);
  if (uart_hal_.dev == nullptr)
  {
    return ErrorCode::NOT_SUPPORT;
  }

  periph_module_enable(uart_module);
  periph_module_reset(uart_module);

  uart_ll_sclk_enable(uart_hal_.dev);
  uart_hal_init(&uart_hal_, uart_num_);

  const uart_sclk_t sclk = UART_SCLK_DEFAULT;
  uart_hal_set_sclk(&uart_hal_, static_cast<soc_module_clk_t>(sclk));
  if (!GetUartClockFrequency(uart_sclk_hz_))
  {
    return ErrorCode::INIT_ERR;
  }

  uart_hw_enabled_ = true;
  if (SetConfig(config_) != ErrorCode::OK)
  {
    uart_hw_enabled_ = false;
    return ErrorCode::INIT_ERR;
  }

  if (ConfigurePins() != ErrorCode::OK)
  {
    uart_hw_enabled_ = false;
    return ErrorCode::INIT_ERR;
  }

  uart_hal_txfifo_rst(&uart_hal_);
  uart_hal_rxfifo_rst(&uart_hal_);
  DisableAndClearUartInterrupt(UINT32_MAX);

  return ErrorCode::OK;
}

// GPIO 映射保持显式写法，因为 ESP UART 路由是逐引脚可配置的。
// GPIO mapping stays explicit because ESP UART routing is per-pin configurable.
ErrorCode ESP32UART::ConfigurePins()
{
  if (tx_pin_ >= 0)
  {
    if (!GPIO_IS_VALID_OUTPUT_GPIO(tx_pin_))
    {
      return ErrorCode::ARG_ERR;
    }
    esp_rom_gpio_pad_select_gpio(static_cast<uint32_t>(tx_pin_));
    esp_rom_gpio_connect_out_signal(
        tx_pin_, UART_PERIPH_SIGNAL(uart_num_, SOC_UART_TX_PIN_IDX), false, false);
  }

  if (rx_pin_ >= 0)
  {
    if (!GPIO_IS_VALID_GPIO(rx_pin_))
    {
      return ErrorCode::ARG_ERR;
    }
    gpio_input_enable(static_cast<gpio_num_t>(rx_pin_));
    esp_rom_gpio_connect_in_signal(
        rx_pin_, UART_PERIPH_SIGNAL(uart_num_, SOC_UART_RX_PIN_IDX), false);
  }

  if (rts_pin_ >= 0)
  {
    if (!GPIO_IS_VALID_OUTPUT_GPIO(rts_pin_))
    {
      return ErrorCode::ARG_ERR;
    }
    esp_rom_gpio_pad_select_gpio(static_cast<uint32_t>(rts_pin_));
    esp_rom_gpio_connect_out_signal(
        rts_pin_, UART_PERIPH_SIGNAL(uart_num_, SOC_UART_RTS_PIN_IDX), false, false);
  }

  if (cts_pin_ >= 0)
  {
    if (!GPIO_IS_VALID_GPIO(cts_pin_))
    {
      return ErrorCode::ARG_ERR;
    }
    gpio_pullup_en(static_cast<gpio_num_t>(cts_pin_));
    gpio_input_enable(static_cast<gpio_num_t>(cts_pin_));
    esp_rom_gpio_connect_in_signal(
        cts_pin_, UART_PERIPH_SIGNAL(uart_num_, SOC_UART_CTS_PIN_IDX), false);
  }

  return ErrorCode::OK;
}

// Active 状态由 FIFO 和 DMA 两条 TX 后端共用。
// Active state is shared by FIFO and DMA TX backends.
void IRAM_ATTR ESP32UART::ClearActiveTx()
{
  tx_active_length_ = 0U;
  tx_active_valid_ = false;
}

// 启动当前半区，检查启动结果；请求完成已在装载时结算。
// Start the active half; request completion was settled during loading.
bool IRAM_ATTR ESP32UART::StartAndReportActive(bool in_isr)
{
  const bool started = StartActiveTransfer(in_isr);
  REQUIRE_FROM_CALLBACK(started, in_isr);
  return true;
}

// TX 发起路径只做两件事：
// 1. TX 完全空闲时，启动一个 active 请求；
// 2. DMA TX 已忙且还没有预装 pending 时，补装一个。
// 配置处于 RESERVED 或 PUBLISHED 时，只关闭从队列装入 DMA 的入口；
// 已经认领的 active/ready 存储仍由 owner 继续推进。
// TX start only does two things:
// 1. when TX is fully idle, start one active request;
// 2. when DMA TX is busy and no pending request is preloaded yet, preload one.
// A reserved or published configuration closes only the queue-to-DMA load points;
// already claimed active/ready storage is still progressed by the owner.
ErrorCode IRAM_ATTR ESP32UART::TryStartTx(bool in_isr)
{
#if SOC_GDMA_SUPPORTED && SOC_UHCI_SUPPORTED
  if (dma_backend_enabled_)
  {
    if (!tx_busy_.IsSet())
    {
      if (!tx_active_valid_ && config_state_.load(std::memory_order_acquire) !=
                                   static_cast<uint8_t>(ConfigState::EMPTY))
      {
        return ErrorCode::PENDING;
      }
      if (!tx_active_valid_ && !LoadActiveTxFromQueue(in_isr))
      {
        return ErrorCode::PENDING;
      }
      if (tx_active_valid_ && !StartAndReportActive(in_isr))
      {
        return ErrorCode::PENDING;
      }
    }
    if (tx_busy_.IsSet() && !tx_dma_buffer_.HasPending() &&
        config_state_.load(std::memory_order_acquire) ==
            static_cast<uint8_t>(ConfigState::EMPTY))
    {
      (void)LoadPendingTxFromQueue(in_isr);
    }
    return ErrorCode::PENDING;
  }
#endif

  FillTxFifo(in_isr);
  return ErrorCode::PENDING;
}

// GDMA 装载完整请求并记录半区占用，随后由队列接口析构完成请求。
// Load a complete GDMA request and record occupancy before scope completion.
bool IRAM_ATTR ESP32UART::LoadActiveTxFromQueue(bool in_isr)
{
#if !(SOC_GDMA_SUPPORTED && SOC_UHCI_SUPPORTED)
  UNUSED(in_isr);
  return false;
#else
  if (!dma_backend_enabled_)
  {
    return false;
  }

  auto queue = _write_port.GetWriteQueue(in_isr);
  if (queue.Empty())
  {
    return false;
  }

  const size_t size = queue.AvailableSize();
  DEV_ASSERT_FROM_CALLBACK(size <= tx_dma_buffer_.Size(), in_isr);
  queue.PopAll(tx_dma_buffer_.ActiveBuffer());
  tx_dma_buffer_.SetActiveLength(size);
  tx_active_length_ = size;
  tx_active_valid_ = true;
  return true;
#endif
}

// Pending 预装只存在于 DMA 后端，因为 FIFO 模式可以直接从队列流式发送，
// 不需要再暂存第二块 payload。
// Pending preload exists only for the DMA backend because FIFO mode can stream
// directly from the queue without staging a second payload block.
bool IRAM_ATTR ESP32UART::LoadPendingTxFromQueue(bool in_isr)
{
#if !(SOC_GDMA_SUPPORTED && SOC_UHCI_SUPPORTED)
  UNUSED(in_isr);
  return false;
#else
  if (!dma_backend_enabled_ || tx_dma_buffer_.HasPending())
  {
    return false;
  }

  auto queue = _write_port.GetWriteQueue(in_isr);
  if (queue.Empty())
  {
    return false;
  }

  const size_t size = queue.AvailableSize();
  DEV_ASSERT_FROM_CALLBACK(size <= tx_dma_buffer_.Size(), in_isr);
  queue.PopAll(tx_dma_buffer_.PendingBuffer());
  tx_dma_buffer_.SetPendingLength(size);
  tx_dma_buffer_.EnablePending();
  return true;
#endif
}

// 提升步骤只负责把一个已经预装的 pending 请求转成 active；
// 真正启动硬件传输是单独一步。
// Promotion only moves one preloaded pending request into the active slot.
// Hardware start is a separate step.
bool IRAM_ATTR ESP32UART::PromotePendingTxToActive()
{
#if !(SOC_GDMA_SUPPORTED && SOC_UHCI_SUPPORTED)
  return false;
#else
  if (!dma_backend_enabled_)
  {
    return false;
  }

  if (tx_active_valid_ || !tx_dma_buffer_.HasPending())
  {
    return false;
  }

  const size_t pending_length = tx_dma_buffer_.GetPendingLength();
  tx_dma_buffer_.Switch();
  tx_dma_buffer_.SetActiveLength(pending_length);

  tx_active_length_ = pending_length;
  tx_active_valid_ = true;

  return true;
#endif
}

// 启动已准备的数据；GDMA 路径使用当前半区。
// Start prepared data using the active half for GDMA.
bool IRAM_ATTR ESP32UART::StartActiveTransfer(bool)
{
  if (!tx_active_valid_)
  {
    return false;
  }

  if (tx_busy_.TestAndSet())
  {
    return true;
  }

#if SOC_GDMA_SUPPORTED && SOC_UHCI_SUPPORTED
  if (dma_backend_enabled_)
  {
    if (StartDmaTx())
    {
      return true;
    }

    tx_busy_.Clear();
    return false;
  }
#endif

  ClearUartInterrupt(UART_TX_INTR_MASK);
  EnableUartInterrupt(UART_TX_INTR_MASK);
  FillTxFifo(false);

  return true;
}

// RX 字节会尽量推进软件队列，直到队列满；完成后按批次触发待读回调。
// RX bytes are pushed opportunistically until the software queue is full, then
// pending read callbacks are serviced once per batch.
void IRAM_ATTR ESP32UART::PushRxBytes(const uint8_t* data, size_t size, bool in_isr)
{
  auto queue = _read_port.GetReadQueue(in_isr);
  size_t offset = 0;
  while (offset < size)
  {
    const size_t free_space = queue.EmptySize();
    if (free_space == 0)
    {
      break;
    }

    const size_t chunk = std::min(free_space, size - offset);
    [[maybe_unused]] const auto push_batch_result = queue.PushBatch(data + offset, chunk);
    DEV_ASSERT_FROM_CALLBACK(push_batch_result == ErrorCode::OK, in_isr);

    offset += chunk;
  }
  queue.Publish();
}

// 释放已结束的传输存储，推进后续半区；不重复完成写请求。
// Release finished transfer storage and advance halves without completing again.
void IRAM_ATTR ESP32UART::OnTxTransferDone(bool in_isr, ErrorCode result)
{
  tx_busy_.Clear();

  ClearActiveTx();

#if SOC_GDMA_SUPPORTED && SOC_UHCI_SUPPORTED
  if (dma_backend_enabled_)
  {
    if (result != ErrorCode::OK)
    {
      tx_dma_buffer_.Reset();
      (void)TryStartTx(in_isr);
      return;
    }

    if (tx_dma_buffer_.HasPending())
    {
      [[maybe_unused]] const auto promote_pending_tx_to_active_result =
          PromotePendingTxToActive();
      DEV_ASSERT_FROM_CALLBACK(promote_pending_tx_to_active_result, in_isr);
      [[maybe_unused]] const auto start_and_report_active_result =
          StartAndReportActive(in_isr);
      REQUIRE_FROM_CALLBACK(start_and_report_active_result, in_isr);
    }
    else if (config_state_.load(std::memory_order_acquire) ==
                 static_cast<uint8_t>(ConfigState::EMPTY) &&
             LoadActiveTxFromQueue(in_isr))
    {
      [[maybe_unused]] const auto start_and_report_active_result =
          StartAndReportActive(in_isr);
      REQUIRE_FROM_CALLBACK(start_and_report_active_result, in_isr);
    }

    if (tx_busy_.IsSet() && !tx_dma_buffer_.HasPending() &&
        config_state_.load(std::memory_order_acquire) ==
            static_cast<uint8_t>(ConfigState::EMPTY))
    {
      (void)LoadPendingTxFromQueue(in_isr);
    }
    return;
  }
#endif

  if (result != ErrorCode::OK)
  {
    return;
  }

  if (LoadActiveTxFromQueue(in_isr))
  {
    (void)StartAndReportActive(in_isr);
  }
}

}  // namespace LibXR
