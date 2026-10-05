#pragma once

/**
 * @file hpm_i2c.hpp
 * @brief HPM I2C 主机驱动适配头文件 / Adapter header for the HPM I2C master driver.
 *
 * @details
 * 本文件在 HPM SDK `hpm_i2c_drv` 之上实现 LibXR `I2C` 抽象，覆盖 blocking
 * 字节流、寄存器地址式传输、基于 `dma_mgr` 的 DMA 后台路径，以及 `ADDRHIT`、`CMPL`
 * 等关键主机状态的错误映射。多系列兼容依赖 `HPMSOC_HAS_HPMSDK_I2C`、
 * `__has_include("hpm_i2c_drv.h")`、实例/IRQ/DMA request header 宏和
 * `I2C_SOC_TRANSFER_COUNT_MAX` 等 SDK 限制宏；硬件时序和 bus recovery 仍待上板验证。
 *
 * This file implements the LibXR `I2C` abstraction on top of the HPM SDK
 * `hpm_i2c_drv` APIs, covering blocking byte-stream transfers, register-address
 * transfers, a `dma_mgr` based DMA background path, and error mapping for key
 * master states such as `ADDRHIT` and `CMPL`. Multi-series compatibility relies on
 * `HPMSOC_HAS_HPMSDK_I2C`, `__has_include("hpm_i2c_drv.h")`, instance/IRQ/DMA
 * request header macros, and SDK limit macros such as `I2C_SOC_TRANSFER_COUNT_MAX`;
 * hardware timing and bus recovery still require board validation.
 */

#include <atomic>

#include "hpm_clock_drv.h"
#include "hpm_soc.h"
#include "i2c.hpp"
#include "libxr_rw.hpp"

#if defined(HPMSOC_HAS_HPMSDK_I2C) && __has_include("hpm_i2c_drv.h")
#include "hpm_i2c_drv.h"
#define LIBXR_HPM_I2C_SUPPORTED 1
using LibXRHpmI2cType = I2C_Type;
#else
#define LIBXR_HPM_I2C_SUPPORTED 0
using LibXRHpmI2cType = void;
#endif

#include "hpm_dma.hpp"

extern "C" void libxr_hpm_i2c_process_interrupt(LibXRHpmI2cType* ptr);

namespace LibXR
{

/**
 * @class HPMI2C
 * @brief HPM SDK I2C 主机驱动，适配 LibXR I2C 接口 /
 * HPM SDK based I2C master driver for the LibXR I2C interface.
 *
 * 该类持有一个 HPM I2C 外设实例，并通过 LibXR::I2C 提供普通字节流传输和
 * 寄存器/存储器地址式传输。
 * This class owns one HPM I2C peripheral instance and exposes byte-stream transfers
 * and register/memory-address transfers through LibXR::I2C.
 *
 * 默认按 7-bit 主机寻址模式启动，也可通过 HPMI2C::SetAddressMode() 切到 HPM SDK
 * 已支持的 10-bit 主机寻址模式；若遇到超时、总线忙或无响应等典型主机故障，驱动会
 * 尝试用最近一次成功配置重建控制器。
 * The driver starts in 7-bit master addressing mode by default and can switch to
 * the HPM SDK supported 10-bit master addressing mode through
 * HPMI2C::SetAddressMode(). On typical master-side failures such as timeout, bus
 * busy, or no response, the driver attempts to rebuild the controller with the most
 * recent successful configuration.
 *
 * BLOCK 模式保持同步阻塞式传输；POLLING / CALLBACK / NONE 的 Write / Read / MemRead
 * 走 DMA 后台路径。DMA 通道和共享 HDMA 中断由 HPM SDK `dma_mgr` 组件管理，该组件是
 * 必需依赖（`CONFIG_DMA_MGR=1`，见 `hpm_dma.hpp`），构造函数通过
 * HPMDmaManager::EnsureInitialized() 完成一次性初始化。MemWrite 始终保持阻塞路径。
 * 对 10-bit / 扩展 flags 的 blocking 手工 phase 路径，地址阶段会显式等待
 * `ADDRHIT`，失败时按适配层策略强制 cleanup 并按原始 HPM 状态触发恢复链路。
 * BLOCK mode stays on synchronous blocking transfers. POLLING / CALLBACK / NONE use
 * the DMA backed path for Write / Read / MemRead. DMA channels and the shared HDMA
 * interrupt are managed by the HPM SDK `dma_mgr` component, which is a required
 * dependency (`CONFIG_DMA_MGR=1`, see `hpm_dma.hpp`); the constructor performs the
 * one-time initialization through HPMDmaManager::EnsureInitialized(). MemWrite always
 * stays on the blocking path.
 * For the blocking manual phase path used by 10-bit / extended-flag transfers, the
 * address phase explicitly waits for `ADDRHIT`; failures force adapter-level cleanup
 * and preserve the original HPM status for recovery handling.
 *
 * 后台 DMA 完成并不直接代表事务成功，驱动会在 DMA 结束后继续确认 I2C `CMPL`
 * 和后置状态，再通过 UpdateStatus() 或 AsyncBlockWait 报告最终结果。
 * DMA completion alone is not treated as transfer success. After DMA finishes, the
 * driver still verifies I2C `CMPL` and post-transfer status before reporting the
 * final result through UpdateStatus() or AsyncBlockWait.
 * 异步完成路径的 `dma_done`、`cmpl_done`、`final_status` 和 `should_recover`
 * 原子状态由内部 AsyncCompletionStateMachine helper 统一更新，IRQ 和 DMA 回调只声明
 * 发生的事件。
 * The asynchronous completion atomics `dma_done`, `cmpl_done`, `final_status`, and
 * `should_recover` are updated through the internal AsyncCompletionStateMachine
 * helper, leaving IRQ and DMA callbacks to report events only.
 *
 * 当 DMA 后台事务正在进行时，同步 Read / Write / MemRead / MemWrite、顺序
 * SequenceRead / SequenceWrite 以及扩展 flags 手动 phase 入口会返回 `BUSY`；
 * 低层手动 helper 在清 FIFO、改 CTRL/INTEN 或发 START 前返回 HPM SDK 的
 * `status_i2c_bus_busy`，再由 ConvertStatus() 映射到 LibXR 错误码。
 * While a DMA-backed transfer is active, synchronous Read / Write / MemRead /
 * MemWrite, sequential SequenceRead / SequenceWrite, and extended-flag manual
 * phase entry points return `BUSY`. The low-level manual helper returns the HPM SDK
 * `status_i2c_bus_busy` before clearing FIFO, changing CTRL/INTEN, or issuing START,
 * and ConvertStatus() maps that status to the LibXR error code.
 *
 * 忙等路径统一经实现文件内部的 `WaitUntil()` 处理；工程可提供强定义
 * `extern "C" void libxr_hpm_i2c_wait_relax_hook(void)`，在等待 `ADDRHIT`、`CMPL`
 * 或 FIFO 状态时插入调度让出、低功耗等待或板级短延时。默认实现仅执行一个 `nop`，
 * 不改变无 RTOS 裸机场景的时序假设。`WaitUntil()` 缓存首次读取的
 * `clock_get_core_clock_ticks_per_us()` 结果，并使用 unsigned elapsed ticks 比较以
 * 避免 deadline 加法回绕；HPM SDK `hpm_csr_get_core_cycle()` 返回 64-bit CYCLE 值，
 * 事务期间仍假设 core clock 不被动态改频。
 * Busy-wait paths are centralized through the implementation-local `WaitUntil()`.
 * A project may provide a strong
 * `extern "C" void libxr_hpm_i2c_wait_relax_hook(void)` definition to yield to a
 * scheduler, enter a low-power wait, or add a board-level short delay while waiting
 * for `ADDRHIT`, `CMPL`, or FIFO status. The default fallback only executes one
 * `nop`, preserving bare-metal timing assumptions when no hook is supplied.
 * `WaitUntil()` caches the first `clock_get_core_clock_ticks_per_us()` result and
 * uses an unsigned elapsed-tick comparison to avoid deadline-addition wrap-around.
 * The HPM SDK `hpm_csr_get_core_cycle()` API returns a 64-bit CYCLE value; the
 * core clock is still assumed not to be dynamically retuned during I2C transfers.
 * 每个实例可通过 SetWaitPolicy() 调整这些超时值。
 * Per-instance timeout values can be adjusted with SetWaitPolicy().
 *
 * 默认 ISR wrapper 按 `HPM_I2Cn`、`IRQn_I2Cn` 和 `HPM_DMA_SRC_I2Cn` 等 HPM SDK
 * header 宏构建同一个实例资源表；该表已拆到 `hpm_i2c_platform.hpp` 内部 helper，
 * 并由该 helper 解析 index、IRQ 和 DMA request source；若项目层后续要自行接管同一
 * IRQ，需要单独调整 ownership 方案。
 * The default ISR wrapper adapts to multi-series instances through HPM SDK header
 * macros such as `HPM_I2Cn`, `IRQn_I2Cn`, and `HPM_DMA_SRC_I2Cn`. The shared
 * instance resource table lives in the internal `hpm_i2c_platform.hpp` helper,
 * which resolves index, IRQ, and DMA request source. If project code needs to own
 * the same IRQ later, the ownership scheme should be revised in a separate change.
 *
 * 编译期支持由 `HPMSOC_HAS_HPMSDK_I2C` 和 `__has_include("hpm_i2c_drv.h")`
 * 同时 gate；缺少能力宏或裁剪 SDK 头时仍保留 LibXR API 形状，但所有事务返回
 * `NOT_SUPPORT`，避免 `driver/hpm` glob 构建在无 I2C SoC 上因 SDK 头缺失失败。
 * Compile-time support is gated by both `HPMSOC_HAS_HPMSDK_I2C` and
 * `__has_include("hpm_i2c_drv.h")`. When the capability macro or trimmed SDK header
 * is missing, the LibXR API shape remains available but transfer APIs return
 * `NOT_SUPPORT`, preventing the `driver/hpm` glob build from failing on SoCs without
 * I2C.
 */
class HPMI2C final : public I2C
{
 public:
  /**
   * @brief HPM I2C 主机寻址模式 / HPM I2C master addressing mode.
   */
  enum class AddressMode : uint8_t
  {
    ADDR_7BIT,  ///< 7-bit 从地址 / 7-bit slave addressing.
    ADDR_10BIT  ///< 10-bit 从地址 / 10-bit slave addressing.
  };

  /**
   * @brief HPM 主机顺序事务分段类型 / HPM master sequential transfer frame type.
   */
  enum class SequenceFrame : uint8_t
  {
    FIRST,  ///< 带 START 和地址阶段 / Includes START and address phase.
    NEXT,   ///< 中间段，不带 START/STOP / Middle frame without START/STOP.
    LAST    ///< 末段，带 STOP / Final frame with STOP.
  };

  /**
   * @brief HPM 主机扩展事务标志 / HPM master extended transfer flags.
   */
  enum class TransferFlag : uint16_t
  {
    NONE = 0,
#if LIBXR_HPM_I2C_SUPPORTED
    READ = I2C_RD,
    ADDR_10BIT = I2C_ADDR_10BIT,
    NO_START = I2C_NO_START,
    NO_ADDRESS = I2C_NO_ADDRESS,
    NO_READ_ACK = I2C_NO_READ_ACK,
    NO_STOP = I2C_NO_STOP,
    WRITE_CHECK_ACK = I2C_WRITE_CHECK_ACK
#else
    READ = 0x0001U,
    ADDR_10BIT = 0x0002U,
    NO_START = 0x0004U,
    NO_ADDRESS = 0x0008U,
    NO_READ_ACK = 0x0010U,
    NO_STOP = 0x0020U,
    WRITE_CHECK_ACK = 0x0040U
#endif
  };

  /**
   * @brief HPM I2C 后台事务类型 / HPM I2C asynchronous transfer kind.
   */
  enum class AsyncTransferKind : uint8_t
  {
    NONE,
    READ,
    WRITE,
    MEM_READ
  };

  /**
   * @brief HPM I2C 忙等超时策略 / HPM I2C busy-wait timeout policy.
   *
   * @details
   * 单位均为微秒，用于等待地址命中、STOP 完成、总线空闲和单笔传输阶段完成。所有字段
   * 必须非零；硬件时序仍需按目标板和外设上板确认。
   * All fields are in microseconds and are used for address-hit, STOP-complete,
   * bus-idle, and transfer-stage waits. Every field must be nonzero; hardware
   * timing still needs board/peripheral validation.
   */
  struct WaitPolicy
  {
    uint64_t addr_hit_timeout_us;
    uint64_t stop_timeout_us;
    uint64_t bus_idle_timeout_us;
    uint64_t transfer_timeout_us;
  };

  /**
   * @brief 返回默认 HPM I2C 忙等超时策略 / Return the default HPM I2C wait policy.
   * @return 默认超时策略 / Default timeout policy.
   */
  static constexpr WaitPolicy DefaultWaitPolicy()
  {
    return {500ULL, 1000ULL, 1000ULL, 500000ULL};
  }

  /**
   * @brief 构造 HPM I2C 主机对象 / Construct an HPM I2C master object.
   * @param i2c HPM I2C 外设基地址，不能为空 /
   * HPM I2C peripheral base address. Must not be nullptr.
   * @param clock HPM 时钟树名称；当板级 helper 未提供时用于开启并查询外设源时钟 /
   * HPM clock name used to enable and query the peripheral source clock when the
   * optional board helper does not provide one.
   * @param auto_board_init 若为 true 且存在 board.h，则自动调用板级 I2C 时钟和引脚
   * 初始化 helper / If true and board.h is available, call board I2C clock and pin
   * helper functions automatically.
   * @param config 初始 I2C 总线配置；当前 HPM 后端只接受 100 kHz、400 kHz、1 MHz
   * 三个固定速率，其他速率由 SetConfig() 返回 NOT_SUPPORT。默认地址模式为 7-bit，
   * 如需 10-bit 可在构造后调用 SetAddressMode() /
   * Initial I2C bus configuration. The current HPM backend accepts only three fixed
   * rates: 100 kHz, 400 kHz, and 1 MHz. Other rates are rejected by SetConfig().
   * The default address mode is 7-bit; call SetAddressMode() after construction to
   * switch to 10-bit when needed.
   *
   * @note 构造函数会在外设指针为空、源时钟无法解析或初始配置无效时断言失败 /
   * The constructor asserts when the peripheral pointer is null, the source clock
   * cannot be resolved, or the initial configuration is invalid.
   */
  HPMI2C(LibXRHpmI2cType* i2c, clock_name_t clock, bool auto_board_init = true,
         I2C::Configuration config = {100000});

  /**
   * @brief 设置 HPM 主机寻址模式并重建控制器 / Set HPM master addressing mode and
   * rebuild the controller.
   * @param mode 目标寻址模式，ADDR_7BIT 或 ADDR_10BIT /
   * Target addressing mode, either ADDR_7BIT or ADDR_10BIT.
   * @return 成功返回 OK；外设指针为空返回 PTR_NULL；时钟/配置重建失败时返回相应错误码 /
   * Returns OK on success, PTR_NULL for null peripheral, or the underlying clock /
   * configuration failure otherwise.
   *
   * @note LibXR::I2C 的通用 Configuration 目前只描述总线速率，因此 10-bit 控制
   * 以 HPMI2C 扩展接口暴露，而不是塞进基类配置里 /
   * LibXR::I2C::Configuration currently describes only bus speed, so 10-bit control
   * is exposed as an HPMI2C extension instead of being packed into the generic base
   * configuration.
   */
  ErrorCode SetAddressMode(AddressMode mode);

  /**
   * @brief 设置当前实例的忙等超时策略 / Set the busy-wait timeout policy for this
   * instance.
   * @param policy 新策略；所有字段必须非零 / New policy. Every field must be nonzero.
   * @return 成功返回 OK；字段为 0 返回 ARG_ERR；后台 DMA 事务在途时返回 BUSY /
   * Returns OK on success, ARG_ERR when any field is zero, or BUSY while a DMA-backed
   * transfer is active.
   */
  ErrorCode SetWaitPolicy(WaitPolicy policy);

  /**
   * @brief 获取当前实例的忙等超时策略 / Get the current busy-wait timeout policy.
   * @return 当前策略 / Current policy.
   */
  WaitPolicy GetWaitPolicy() const { return wait_policy_; }

  /**
   * @brief 获取当前 HPM 主机寻址模式 / Get current HPM master addressing mode.
   * @return 当前地址模式 / Current address mode.
   */
  AddressMode GetAddressMode() const { return address_mode_; }

  /**
   * @brief 执行一段 HPM 主机顺序写事务 / Execute one HPM master sequential write frame.
   * @param slave_addr 当前寻址模式下的从设备地址，不包含 R/W 位 / Slave address in
   * the current addressing mode, without the R/W bit.
   * @param write_data 当前分段要写出的数据，长度必须大于 0 / Payload for this frame.
   * Size must be greater than 0.
   * @param frame 分段类型：FIRST/NEXT/LAST / Frame type: FIRST, NEXT, or LAST.
   * @param check_ack 是否逐字节检查从机 ACK / Whether to check slave ACK byte by byte.
   * @param op LibXR 写操作描述符 / LibXR write operation descriptor.
   * @param in_isr 仅用于状态分发的中断上下文标志 / ISR-context flag for status dispatch.
   * @return 返回本段事务的最终错误码 / Final error code for this frame.
   */
  ErrorCode SequenceWrite(uint16_t slave_addr, ConstRawData write_data,
                          SequenceFrame frame, bool check_ack, WriteOperation& op,
                          bool in_isr = false);

  /**
   * @brief 执行一段 HPM 主机顺序读事务 / Execute one HPM master sequential read frame.
   * @param slave_addr 当前寻址模式下的从设备地址，不包含 R/W 位 / Slave address in
   * the current addressing mode, without the R/W bit.
   * @param read_data 当前分段要读入的数据缓冲区，长度必须大于 0 / Destination buffer for
   * this frame. Size must be greater than 0.
   * @param frame 分段类型：FIRST/NEXT/LAST / Frame type: FIRST, NEXT, or LAST.
   * @param op LibXR 读操作描述符 / LibXR read operation descriptor.
   * @param in_isr 仅用于状态分发的中断上下文标志 / ISR-context flag for status dispatch.
   * @return 返回本段事务的最终错误码 / Final error code for this frame.
   */
  ErrorCode SequenceRead(uint16_t slave_addr, RawData read_data, SequenceFrame frame,
                         ReadOperation& op, bool in_isr = false);

  /**
   * @brief 执行一段 HPM 扩展标志主机事务 / Execute one HPM master transfer with
   * extended SDK flags.
   * @param slave_addr 当前寻址模式下的从设备地址，不包含 R/W 位 / Slave address in
   * the current addressing mode, without the R/W bit.
   * @param data 数据缓冲区；按 flags 指示读或写 / Data buffer used for read or write
   * according to flags.
   * @param flags HPM 扩展事务标志组合 / Combined HPM extended transfer flags.
   * @param op LibXR 读写操作描述符 / LibXR read/write operation descriptor.
   * @param in_isr 仅用于状态分发的中断上下文标志 / ISR-context flag for status dispatch.
   * @return 返回事务最终错误码 / Final transfer error code.
   */
  ErrorCode TransferWithFlags(uint16_t slave_addr, RawData data, uint16_t flags,
                              ReadOperation& op, bool in_isr = false);

  /**
   * @brief 执行一段 HPM 扩展标志主机写事务 / Execute one HPM master write transfer
   * with extended SDK flags.
   * @param slave_addr 当前寻址模式下的从设备地址，不包含 R/W 位 / Slave address in
   * the current addressing mode, without the R/W bit.
   * @param data 要写出的数据缓冲区 / Data buffer to be transmitted.
   * @param flags HPM 扩展事务标志组合；READ 位若被传入会被忽略 / Combined HPM
   * extended transfer flags. The READ bit is ignored when provided.
   * @param op LibXR 写操作描述符 / LibXR write operation descriptor.
   * @param in_isr 仅用于状态分发的中断上下文标志 / ISR-context flag for status dispatch.
   * @return 返回事务最终错误码 / Final transfer error code.
   */
  ErrorCode TransferWithFlags(uint16_t slave_addr, ConstRawData data, uint16_t flags,
                              WriteOperation& op, bool in_isr = false);

  /**
   * @brief 从 I2C 从设备读取字节 / Read bytes from an I2C slave.
   * @param slave_addr 当前寻址模式下的从设备地址，不包含 R/W 位 /
   * Slave address for the current addressing mode, without the R/W bit.
   * @param read_data 目标缓冲区；size_ 为 0 时不访问总线并返回 OK。非零长度时
   * addr_ 必须非空，且 size_ 不超过 I2C_SOC_TRANSFER_COUNT_MAX /
   * Destination buffer. A zero-size buffer completes with OK without bus access.
   * For non-zero size, addr_ must be non-null and size_ must not exceed
   * I2C_SOC_TRANSFER_COUNT_MAX.
   * @param op LibXR 读操作描述符，见类注释中的操作模式说明 /
   * LibXR read operation descriptor. See class-level operation mode notes.
   * @param in_isr 仅用于 CALLBACK/POLLING 完成分发的中断上下文标志 /
   * ISR-context flag forwarded only to CALLBACK/POLLING completion handling.
   * @return 成功返回 OK；空缓冲指针返回 PTR_NULL；从地址超出当前寻址模式范围返回
   * ARG_ERR；
   * 长度超限返回 SIZE_ERR；其余返回 HPM SDK 状态转换后的 TIMEOUT、BUSY、
   * NO_RESPONSE、CHECK_ERR 或 FAILED /
   * Returns OK on success, PTR_NULL for null non-empty buffer, SIZE_ERR for oversized
   * transfer, ARG_ERR for slave addresses beyond the active addressing range, or
   * converted HPM
   * SDK status such as TIMEOUT, BUSY, NO_RESPONSE, CHECK_ERR, or FAILED.
   */
  ErrorCode Read(uint16_t slave_addr, RawData read_data, ReadOperation& op,
                 bool in_isr = false) override;

  /**
   * @brief 向 I2C 从设备写入字节 / Write bytes to an I2C slave.
   * @param slave_addr 当前寻址模式下的从设备地址，不包含 R/W 位 /
   * Slave address for the current addressing mode, without the R/W bit.
   * @param write_data 源缓冲区；size_ 为 0 时不访问总线并返回 OK。非零长度时
   * addr_ 必须非空，且 size_ 不超过 I2C_SOC_TRANSFER_COUNT_MAX /
   * Source buffer. A zero-size buffer completes with OK without bus access.
   * For non-zero size, addr_ must be non-null and size_ must not exceed
   * I2C_SOC_TRANSFER_COUNT_MAX.
   * @param op LibXR 写操作描述符，见类注释中的操作模式说明 /
   * LibXR write operation descriptor. See class-level operation mode notes.
   * @param in_isr 仅用于 CALLBACK/POLLING 完成分发的中断上下文标志 /
   * ISR-context flag forwarded only to CALLBACK/POLLING completion handling.
   * @return 成功返回 OK；空缓冲指针返回 PTR_NULL；从地址超出当前寻址模式范围返回
   * ARG_ERR；
   * 长度超限返回 SIZE_ERR；其余返回 HPM SDK 状态转换后的 TIMEOUT、BUSY、
   * NO_RESPONSE、CHECK_ERR 或 FAILED /
   * Returns OK on success, PTR_NULL for null non-empty buffer, SIZE_ERR for oversized
   * transfer, ARG_ERR for slave addresses beyond the active addressing range, or
   * converted HPM
   * SDK status such as TIMEOUT, BUSY, NO_RESPONSE, CHECK_ERR, or FAILED.
   */
  ErrorCode Write(uint16_t slave_addr, ConstRawData write_data, WriteOperation& op,
                  bool in_isr = false) override;

  /**
   * @brief 从 I2C 从设备寄存器或存储器地址读取字节 /
   * Read bytes from an I2C slave register or memory address.
   *
   * 该接口先发送寄存器/存储器地址，再进入读阶段。BYTE_8 只发送 mem_addr 低 8 位；
   * BYTE_16 按高字节在前、低字节在后的顺序发送。
   * This API sends the register/memory address before the read phase. BYTE_8 sends
   * only the low 8 bits of mem_addr; BYTE_16 sends the high byte first, then the
   * low byte.
   *
   * @param slave_addr 当前寻址模式下的从设备地址，不包含 R/W 位 /
   * Slave address for the current addressing mode, without the R/W bit.
   * @param mem_addr 读阶段前发送的寄存器或存储器地址 /
   * Register or memory address sent before reading.
   * @param read_data 目标缓冲区；size_ 为 0 时不访问总线并返回 OK。非零长度时
   * addr_ 必须非空，且 size_ 不超过 I2C_SOC_TRANSFER_COUNT_MAX /
   * Destination buffer. A zero-size buffer completes with OK without bus access.
   * For non-zero size, addr_ must be non-null and size_ must not exceed
   * I2C_SOC_TRANSFER_COUNT_MAX.
   * @param op LibXR 读操作描述符，见类注释中的操作模式说明 /
   * LibXR read operation descriptor. See class-level operation mode notes.
   * @param mem_addr_size 寄存器地址宽度：BYTE_8 或 BYTE_16 /
   * Register address width: BYTE_8 or BYTE_16.
   * @param in_isr 仅用于 CALLBACK/POLLING 完成分发的中断上下文标志 /
   * ISR-context flag forwarded only to CALLBACK/POLLING completion handling.
   * @return 成功返回 OK；空缓冲指针返回 PTR_NULL；从地址超出当前寻址模式范围返回
   * ARG_ERR；寄存器地址长度枚举无效时返回 ARG_ERR；
   * 读载荷长度超限返回 SIZE_ERR；其余返回 HPM SDK 状态转换后的错误码 /
   * Returns OK on success, PTR_NULL for null non-empty buffer, SIZE_ERR for oversized
   * read payload, ARG_ERR for slave addresses beyond the active addressing range or
   * invalid register-address enum, or converted HPM SDK status otherwise.
   */
  ErrorCode MemRead(uint16_t slave_addr, uint16_t mem_addr, RawData read_data,
                    ReadOperation& op,
                    MemAddrLength mem_addr_size = MemAddrLength::BYTE_8,
                    bool in_isr = false) override;

  /**
   * @brief 向 I2C 从设备寄存器或存储器地址写入字节 /
   * Write bytes to an I2C slave register or memory address.
   *
   * 该接口先发送寄存器/存储器地址，再发送载荷。BYTE_8 只发送 mem_addr 低 8 位；
   * BYTE_16 按高字节在前、低字节在后的顺序发送。零长度载荷合法，只写地址字节。
   * This API sends the register/memory address before the payload. BYTE_8 sends only
   * the low 8 bits of mem_addr; BYTE_16 sends the high byte first, then the low byte.
   * A zero-size payload is valid and writes only the address bytes.
   *
   * @param slave_addr 当前寻址模式下的从设备地址，不包含 R/W 位 /
   * Slave address for the current addressing mode, without the R/W bit.
   * @param mem_addr 载荷前发送的寄存器或存储器地址 /
   * Register or memory address sent before the payload.
   * @param write_data 源载荷缓冲区；非零长度时 addr_ 必须非空；地址字节加载荷长度
   * 不能超过 I2C_SOC_TRANSFER_COUNT_MAX /
   * Source payload buffer. For non-zero size, addr_ must be non-null. Address bytes
   * plus payload size must not exceed I2C_SOC_TRANSFER_COUNT_MAX.
   * @param op LibXR 写操作描述符，见类注释中的操作模式说明 /
   * LibXR write operation descriptor. See class-level operation mode notes.
   * @param mem_addr_size 寄存器地址宽度：BYTE_8 或 BYTE_16 /
   * Register address width: BYTE_8 or BYTE_16.
   * @param in_isr 仅用于 CALLBACK/POLLING 完成分发的中断上下文标志 /
   * ISR-context flag forwarded only to CALLBACK/POLLING completion handling.
   * @return 成功返回 OK；空载荷指针返回 PTR_NULL；从地址超出当前寻址模式范围返回
   * ARG_ERR；寄存器地址长度枚举无效时返回 ARG_ERR；地址加载荷长度超限返回
   * SIZE_ERR；其余返回 HPM SDK 状态转换后的错误码 /
   * Returns OK on success, PTR_NULL for null non-empty payload, SIZE_ERR when address
   * bytes plus payload exceed the hardware limit, ARG_ERR for slave addresses beyond
   * the active addressing range or invalid register-address enum, or converted HPM
   * SDK status otherwise.
   */
  ErrorCode MemWrite(uint16_t slave_addr, uint16_t mem_addr, ConstRawData write_data,
                     WriteOperation& op,
                     MemAddrLength mem_addr_size = MemAddrLength::BYTE_8,
                     bool in_isr = false) override;

  /**
   * @brief 应用 I2C 时序配置 / Apply I2C timing configuration.
   * @param config 目标 I2C 时钟配置 / Desired I2C clock configuration.
   * @return 成功返回 OK；外设指针为空返回 PTR_NULL；源时钟无法解析返回 INIT_ERR；
   * 时钟为 0 返回 ARG_ERR；非 100 kHz / 400 kHz / 1 MHz 的速率返回 NOT_SUPPORT；
   * 其余返回 HPM SDK 初始化状态 /
   * Returns OK on success, PTR_NULL for null peripheral pointer, INIT_ERR when source
   * clock cannot be resolved, ARG_ERR for zero clock, NOT_SUPPORT for rates other
   * than 100 kHz / 400 kHz / 1 MHz, or converted HPM SDK initialization status
   * otherwise.
   */
  ErrorCode SetConfig(Configuration config) override;

  /**
   * @brief 获取当前缓存的 I2C 源时钟频率 / Get cached I2C source clock frequency.
   * @return 源时钟频率，单位 Hz；成功解析时钟前为 0 /
   * Source clock frequency in Hz, or 0 before successful clock discovery.
   */
  uint32_t GetClockFreq() const { return source_clock_hz_; }

  /**
   * @brief 供 DMA / 中断完成路径调用的完成入口 /
   * Completion entry used by DMA / interrupt driven paths.
   * @param in_isr 当前是否位于中断上下文 / Whether current context is ISR.
   * @param ans 事务最终错误码 / Final transfer error code.
   */
  void CompleteAsyncTransfer(bool in_isr, ErrorCode ans);

 private:
  friend void ::libxr_hpm_i2c_process_interrupt(LibXRHpmI2cType* ptr);

  /**
   * @brief HPM I2C 后台事务上下文 /
   * HPM I2C asynchronous transfer context.
   */
  struct AsyncTransferContext
  {
    AsyncTransferKind kind = AsyncTransferKind::NONE;
    uint16_t slave_addr = 0U;
    MemAddrLength mem_addr_size = MemAddrLength::BYTE_8;
    uint16_t mem_addr = 0U;
    uint32_t mem_addr_size_in_byte = 0U;
    uint8_t mem_addr_bytes[2] = {};
    uint16_t flags = 0U;
    RawData read_data = {nullptr, 0};
    ConstRawData write_data = {nullptr, 0};
    ReadOperation read_op = {};
    WriteOperation write_op = {};
    std::atomic<hpm_stat_t> final_status{
        status_success};  ///< DMA/IRQ 共享最终状态 / Final status shared by DMA/IRQ.
    std::atomic<bool> should_recover{
        false};  ///< DMA/IRQ 共享恢复标志 / Recovery flag shared by DMA/IRQ.
    std::atomic<bool> dma_done{
        false};  ///< DMA 回调完成标志 / DMA callback completion flag.
    std::atomic<bool> cmpl_done{false};  ///< I2C IRQ 完成标志 / I2C IRQ completion flag.
  };

  /**
   * @brief 异步 DMA/I2C 完成状态 helper / Async DMA/I2C completion state helper.
   *
   * 该 helper 集中更新 `dma_done`、`cmpl_done`、`final_status` 和 `should_recover`，
   * 使 DMA 回调、I2C IRQ 和 BLOCK timeout 只表达发生的事件。
   * This helper centralizes updates to `dma_done`, `cmpl_done`, `final_status`, and
   * `should_recover`, so DMA callbacks, I2C IRQ handling, and BLOCK timeout paths
   * only report events.
   */
  struct AsyncCompletionStateMachine
  {
    static void Reset(AsyncTransferContext& ctx);
    static void MarkDmaDone(AsyncTransferContext& ctx);
    static void MarkI2cDone(AsyncTransferContext& ctx, hpm_stat_t status, bool recover);
    static void MarkFailure(AsyncTransferContext& ctx, hpm_stat_t status, bool recover);
    static void SetFinalStatus(AsyncTransferContext& ctx, hpm_stat_t status);
    static bool DmaDone(const AsyncTransferContext& ctx);
    static bool Ready(const AsyncTransferContext& ctx);
    static hpm_stat_t FinalStatus(const AsyncTransferContext& ctx);
    static bool ShouldRecover(const AsyncTransferContext& ctx);
  };

  /**
   * @brief 当前 HPM 后端支持的最大 7-bit 从地址 / Maximum 7-bit slave address
   * supported by the current HPM backend.
   */
  static constexpr uint16_t kMax7BitAddress = 0x7FU;

  /**
   * @brief 当前 HPM 后端支持的最大 10-bit 从地址 / Maximum 10-bit slave address
   * supported by the current HPM backend.
   */
  static constexpr uint16_t kMax10BitAddress = 0x3FFU;

  /**
   * @brief 将 HPM SDK I2C 状态码转换为 LibXR 错误码 /
   * Convert HPM SDK I2C status to LibXR ErrorCode.
   */
  static ErrorCode ConvertStatus(hpm_stat_t status);

  /**
   * @brief 根据目标固定速率解析 HPM I2C 工作模式 /
   * Resolve HPM I2C operating mode from a fixed target clock rate.
   */
#if LIBXR_HPM_I2C_SUPPORTED
  static ErrorCode ResolveMode(uint32_t clock_speed, i2c_mode_t& mode);
#endif

  /**
   * @brief 将 DMA manager 状态码转换为 LibXR 错误码 /
   * Convert DMA manager status to LibXR ErrorCode.
   */
  static ErrorCode ConvertDmaStatus(hpm_stat_t status);

  /**
   * @brief 转换 HPM 顺序分段枚举到 SDK 枚举 / Convert HPM sequence frame enum to SDK
   * enum.
   */
#if LIBXR_HPM_I2C_SUPPORTED
  static i2c_seq_transfer_opt_t ConvertSequenceFrame(SequenceFrame frame);
#endif

  /**
   * @brief 从当前地址模式补齐 HPM SDK 事务标志 / Merge current address-mode bit into SDK
   * transfer flags.
   */
  uint16_t BuildTransferFlags(uint16_t flags) const;

  /**
   * @brief 获取指定寻址模式可接受的最大从地址 /
   * Get the maximum valid slave address for the given mode.
   */
  static uint16_t GetMaxSlaveAddress(AddressMode mode);

  /**
   * @brief 校验当前寻址模式下的从地址 /
   * Validate slave address for the current addressing mode.
   */
  ErrorCode ValidateSlaveAddress(uint16_t slave_addr) const;

  /**
   * @brief 校验普通读写事务入口参数 / Validate common read/write entry arguments.
   */
  ErrorCode ValidateTransferArgs(uint16_t slave_addr, RawData data,
                                 bool allow_zero_size) const;

  /**
   * @brief 校验普通写事务入口参数 / Validate common write entry arguments.
   */
  ErrorCode ValidateTransferArgs(uint16_t slave_addr, ConstRawData data,
                                 bool allow_zero_size) const;

  /**
   * @brief 校验寄存器写事务入口参数并解析寄存器地址长度 /
   * Validate memory-write entry arguments and resolve memory-address length.
   */
  ErrorCode ValidateMemWriteArgs(uint16_t slave_addr, ConstRawData write_data,
                                 MemAddrLength mem_addr_size, uint32_t& addr_size) const;

  /**
   * @brief 解析寄存器地址宽度枚举到字节数 /
   * Resolve register-address enum to byte count.
   */
  static ErrorCode ResolveMemAddressSize(MemAddrLength len, uint32_t& addr_size);

  /**
   * @brief 按 BYTE_8/BYTE_16 格式填充寄存器地址字节 /
   * Fill register address bytes according to BYTE_8/BYTE_16 format.
   */
  static void FillMemAddress(uint16_t mem_addr, MemAddrLength len, uint8_t out[2]);

  /**
   * @brief 执行一段顺序写事务，不做状态分发 / Execute one sequential write frame
   * without LibXR status dispatch.
   */
  ErrorCode DoSequenceWrite(uint16_t slave_addr, ConstRawData write_data,
                            SequenceFrame frame, bool check_ack);

  /**
   * @brief 执行一段顺序读事务，不做状态分发 / Execute one sequential read frame
   * without LibXR status dispatch.
   */
  ErrorCode DoSequenceRead(uint16_t slave_addr, RawData read_data, SequenceFrame frame);

  /**
   * @brief 执行一段扩展标志事务，不做状态分发 / Execute one flag-based transfer
   * without LibXR status dispatch.
   */
  ErrorCode DoTransferWithFlags(uint16_t slave_addr, RawData data, uint16_t flags);

  /**
   * @brief 通过手工 phase 编排执行一笔 blocking 事务 /
   * Execute one blocking transfer through manual phase orchestration.
   */
  hpm_stat_t DoManualTransferWithFlags(uint16_t slave_addr, RawData data, uint16_t flags);

  /**
   * @brief 为后台事务准备当前控制器的 phase / FIFO / CMPL 状态 /
   * Prepare phase/FIFO/CMPL state before asynchronous transfer submission.
   */
  ErrorCode PrepareAsyncTransfer(uint16_t slave_addr, uint16_t flags, uint32_t size,
                                 bool clear_fifo = true, bool require_bus_idle = true);

  /**
   * @brief 将本地内存地址转换成 DMA 可访问的系统地址 /
   * Convert local-core address to DMA visible system address.
   */
  static uint32_t ToSystemAddress(const void* addr);

  /**
   * @brief 安装一笔读事务的 DMA 描述并启动 /
   * Program and start one RX DMA transaction.
   */
  ErrorCode StartAsyncReadDma(void* dst, uint32_t size);

  /**
   * @brief 安装一笔写事务的 DMA 描述并启动 /
   * Program and start one TX DMA transaction.
   */
  ErrorCode StartAsyncWriteDma(const void* src, uint32_t size);

  /**
   * @brief 结束当前 I2C DMA 事务 /
   * Stop the current I2C DMA transaction.
   */
  void StopAsyncDma();

  /**
   * @brief 显式发出 STOP 并释放异步事务占用的总线相位 /
   * Issue an explicit STOP and release the bus phase held by an async transaction.
   */
  void StopAndReleaseAsyncBus();

  /**
   * @brief Reset async bookkeeping after a start path aborts or completes.
   */
  void ResetAsyncState();

  /**
   * @brief 清空异步事务上下文但不改变 busy 标志 /
   * Clear async transaction context without changing the busy flag.
   */
  void ClearAsyncContext();

  /**
   * @brief Shared cleanup for async submit failures before the operation is running.
   */
  void AbortAsyncStart(bool stop_dma, bool disable_irq, bool recover_controller);

  /**
   * @brief Start BLOCK wait state only for blocking operation descriptors.
   */
  template <typename Op>
  void StartAsyncBlockWaitIfNeeded(Op& op)
  {
    if (op.type == Op::OperationType::BLOCK)
    {
      block_wait_.Start(*op.data.sem_info.sem);
    }
  }

  /**
   * @brief Cancel BLOCK wait state only for blocking operation descriptors.
   */
  template <typename Op>
  void CancelAsyncBlockWaitIfNeeded(Op& op)
  {
    if (op.type == Op::OperationType::BLOCK)
    {
      block_wait_.Cancel();
    }
  }

  /**
   * @brief Dispatch a completed async operation to its LibXR completion target.
   */
  template <typename Op>
  void CompleteAsyncOperation(Op& op, bool in_isr, ErrorCode ans)
  {
    if (op.type == Op::OperationType::BLOCK)
    {
      (void)block_wait_.TryPost(in_isr, ans);
    }
    else
    {
      op.UpdateStatus(in_isr, ans);
    }
  }

  /**
   * @brief 后台主机写事务提交 /
   * Submit one asynchronous master write transaction.
   */
  ErrorCode StartWriteAsync(uint16_t slave_addr, ConstRawData write_data,
                            WriteOperation& op);

  /**
   * @brief 后台主机读事务提交 /
   * Submit one asynchronous master read transaction.
   */
  ErrorCode StartReadAsync(uint16_t slave_addr, RawData read_data, ReadOperation& op);

  /**
   * @brief 后台寄存器读事务提交 /
   * Submit one asynchronous memory/register read transaction.
   */
  ErrorCode StartMemReadAsync(uint16_t slave_addr, uint16_t mem_addr, RawData read_data,
                              ReadOperation& op, MemAddrLength mem_addr_size);

  /**
   * @brief DMA manager 资源就绪检查 / Ensure DMA manager resource is ready.
   */
  ErrorCode EnsureAsyncDmaReady();

  /**
   * @brief 等待异步 BLOCK 事务最终结果，并在超时时主动清理 /
   * Wait for an asynchronous BLOCK result and actively clean up on timeout.
   */
  ErrorCode WaitForAsyncBlockResult(uint32_t timeout);

  /**
   * @brief 释放当前异步事务占用的总线相位 / Release the current asynchronous bus phase.
   */
  void ReleaseAsyncBus();

  /**
   * @brief 打开当前实例异步完成所需的 I2C IRQ /
   * Enable the I2C IRQ needed by the current instance's asynchronous completion path.
   */
  ErrorCode EnableAsyncI2cIrq();

  /**
   * @brief 关闭当前实例异步完成所需的 I2C IRQ /
   * Disable the I2C IRQ used by the current instance's asynchronous completion path.
   */
  void DisableAsyncI2cIrq();

  /**
   * @brief 处理当前实例的 I2C IRQ 完成/错误状态 /
   * Handle I2C IRQ completion/error state for the current instance.
   *
   * @note ARBLOSE 在适配层里按 fatal bus error 处理，并触发恢复路径；这里不是把
   * 它解释成 HPM 官方语义上的 timeout /
   * ARBLOSE is treated as a fatal bus error in this adapter and routed into the
   * recovery path. It is not meant to reinterpret the HPM hardware event as an
   * official timeout condition.
   */
  void HandleAsyncInterrupt(bool in_isr);

  /**
   * @brief 当 DMA 与 I2C 两阶段都满足时统一完成事务 /
   * Complete the transfer once both DMA and I2C phases are satisfied.
   */
  void MaybeCompleteAsyncTransfer(bool in_isr);

  /**
   * @brief 抢占当前异步事务的唯一完成权 /
   * Claim the single completion ownership of the current asynchronous transfer.
   */
  bool TryClaimAsyncCompletion();

  /**
   * @brief DMA 终端完成回调入口 / DMA terminal-count callback entry.
   */
  static void OnDmaTcCallback(DMA_Type* base, uint32_t channel, void* cb_data_ptr);

  /**
   * @brief DMA 错误回调入口 / DMA error callback entry.
   */
  static void OnDmaErrorCallback(DMA_Type* base, uint32_t channel, void* cb_data_ptr);

  /**
   * @brief DMA 中止回调入口 / DMA abort callback entry.
   */
  static void OnDmaAbortCallback(DMA_Type* base, uint32_t channel, void* cb_data_ptr);

  /**
   * @brief 查询当前是否有后台事务在途 /
   * Query whether an asynchronous transfer is in flight.
   */
  bool AsyncTransferActive() const
  {
    return async_busy_.load(std::memory_order_acquire) != 0U;
  }

  /**
   * @brief 确保 I2C 源时钟已可用 /
   * Ensure the I2C source clock is available.
   */
  ErrorCode EnsureClockReady();

  /**
   * @brief 确保控制器已完成一次可用配置 /
   * Ensure the controller has a usable configuration applied.
   */
  ErrorCode EnsureControllerReady();

  /**
   * @brief 按当前后端限制下发配置并缓存成功配置 /
   * Apply configuration under current backend limits and cache it on success.
   */
  ErrorCode ApplyConfig(const Configuration& config);

  /**
   * @brief 判断失败后是否需要重建控制器 /
   * Decide whether the controller should be reinitialized after a failure.
   */
  static bool ShouldRecover(hpm_stat_t status);

  /**
   * @brief 复位并按缓存配置重建 I2C 控制器 /
   * Reset and reinitialize the I2C controller with cached configuration.
   */
  void RecoverController();

  /**
   * @brief 在 SDA 被拉低且硬件支持时尝试发出 I2C reset clocks /
   * Try to generate I2C reset clocks when SDA is stuck low and the hardware supports it.
   */
  void TryRecoverBusLines();

  /**
   * @brief 完成 LibXR 操作状态更新或回调 /
   * Complete LibXR operation status update or callback dispatch.
   */
  template <typename Op>
  static ErrorCode FinishOperation(Op& op, bool in_isr, ErrorCode ans)
  {
    if (op.type != Op::OperationType::BLOCK)
    {
      op.UpdateStatus(in_isr, ans);
    }
    return ans;
  }

  LibXRHpmI2cType* i2c_;          ///< I2C 外设实例 / I2C peripheral instance.
  clock_name_t clock_;            ///< I2C 源时钟名称 / I2C source clock name.
  uint32_t source_clock_hz_ = 0;  ///< 缓存的源时钟频率 / Cached source clock frequency.
  Configuration current_config_{
      100000};  ///< 最近一次成功总线时序配置 / Most recent successful bus-timing config.
  WaitPolicy wait_policy_ =
      DefaultWaitPolicy();  ///< 当前忙等超时策略 / Current busy-wait timeout policy.
  AddressMode address_mode_ =
      AddressMode::ADDR_7BIT;  ///< 当前主机寻址模式 / Current master addressing mode.
  bool configured_ = false;    ///< 是否已有成功配置 / Whether a valid config was applied.
  std::atomic<uint32_t> async_busy_{
      0U};  ///< 后台事务活动标志 / Asynchronous transfer active flag.
  AsyncTransferContext async_ctx_{};  ///< 后台事务上下文 / Async transfer context.
  AsyncBlockWait block_wait_{};  ///< BLOCK 模式后台等待器 / BLOCK waiter for async path.
  std::atomic<uint32_t> async_completion_claim_{
      0U};  ///< 异步完成抢占标志 / Async completion claim flag.
  dma_resource_t async_dma_resource_{nullptr, 0U,
                                     -1};  ///< DMA 资源句柄 / DMA resource handle.
  uint8_t async_dma_source_ =
      0U;  ///< 当前 I2C 对应的 DMAMUX source / DMAMUX source for this I2C.
  bool async_dma_ready_ =
      false;  ///< DMA 资源是否已经初始化 / Whether DMA resource has been initialized.
  bool auto_board_init_;  ///< 是否自动调用板级初始化 / Whether board init is automatic.
};

}  // namespace LibXR
