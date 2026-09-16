#pragma once

/**
 * @file hpm_spi.hpp
 * @brief HPM SPI 主机驱动适配头文件 / Adapter header for the HPM SPI master driver.
 *
 * @details
 * 本文件在 HPM SDK `hpm_spi_drv` 之上实现 LibXR `SPI` 抽象，默认采用阻塞式
 * master 传输，并可在工程启用 `USE_DMA_MGR=1` 时为普通 data-phase 流式传输打开
 * HPM SDK SPI component DMA manager 路径。驱动把 CPOL/CPHA、分频、8-bit 命令式
 * 寄存器访问、SPI flash command phase helper 和 SDK 状态码映射到 LibXR API。
 * 多系列兼容依赖 `HPMSOC_HAS_HPMSDK_SPI`、`__has_include("hpm_spi_drv.h")`、
 * 构造参数传入的实例/clock，以及 `SPI_SOC_TRANSFER_COUNT_MAX` 等 SDK header
 * 限制宏；真实片选、波形和错误恢复仍需 board 级验证。
 *
 * This file implements the LibXR `SPI` abstraction on top of the HPM SDK
 * `hpm_spi_drv` APIs. The default backend uses blocking master transfers, and
 * projects built with `USE_DMA_MGR=1` may explicitly enable the HPM SDK SPI
 * component DMA-manager path for regular data-phase stream transfers. The driver
 * maps CPOL/CPHA, prescaler selection, 8-bit command-style register access,
 * SPI-flash command-phase helpers, and SDK status codes into the LibXR API.
 * Multi-series compatibility relies on `HPMSOC_HAS_HPMSDK_SPI`,
 * `__has_include("hpm_spi_drv.h")`, constructor-provided instance/clock values,
 * and SDK header limit macros such as `SPI_SOC_TRANSFER_COUNT_MAX`; real
 * chip-select behavior, waveforms, and error recovery still require board-level
 * validation.
 */

#include <atomic>

#include "hpm_clock_drv.h"
#include "hpm_soc.h"
#include "spi.hpp"

#if __has_include("hpm_sdk_version.h")
#include "hpm_sdk_version.h"
#endif

#if defined(HPMSOC_HAS_HPMSDK_SPI) && __has_include("hpm_spi_drv.h")
#include "hpm_spi_drv.h"
#define LIBXR_HPM_SPI_SUPPORTED 1
using LibXRHpmSpiType = SPI_Type;
using LibXRHpmSpiStatusType = hpm_stat_t;
#else
#define LIBXR_HPM_SPI_SUPPORTED 0
using LibXRHpmSpiType = void;
using LibXRHpmSpiStatusType = int;
#endif

#if LIBXR_HPM_SPI_SUPPORTED && defined(USE_DMA_MGR) && (USE_DMA_MGR) && \
    defined(SDK_VERSION_NUMBER) && (SDK_VERSION_NUMBER >= 0x010B00U) && \
    __has_include("hpm_spi.h") && __has_include("hpm_dma_mgr.h")
#include "hpm_spi.h"
#define LIBXR_HPM_SPI_HAS_DMA_MGR 1
#else
#define LIBXR_HPM_SPI_HAS_DMA_MGR 0
#endif

namespace LibXR
{

/**
 * @class HPMSPI
 * @brief HPM SDK SPI 主机驱动，适配 LibXR SPI 接口 /
 * HPM SDK based SPI master driver for the LibXR SPI interface.
 *
 * 该类持有一个 HPM SPI 外设实例，提供普通流式传输和简单寄存器式 SPI 访问。
 * This class owns one HPM SPI peripheral instance and provides stream transfers
 * and simple register-style SPI access.
 *
 * 默认实现调用 HPM SDK 阻塞式传输 API；显式调用 SetDmaEnabled(true) 后，
 * ReadAndWrite() 和 Transfer() 会使用 HPM SDK SPI component 的 DMA manager
 * nonblocking data-phase API，直到 DMA 回调完成前同一实例的其它事务会返回 BUSY。
 * CommandRead()、CommandWriteRead()、MemRead() 和 MemWrite() 保持阻塞路径。
 * The default implementation calls HPM SDK blocking transfer APIs. After
 * SetDmaEnabled(true), ReadAndWrite() and Transfer() use the HPM SDK SPI component
 * DMA-manager nonblocking data-phase APIs, and other transactions on the same
 * instance return BUSY until the DMA callback completes. CommandRead(),
 * CommandWriteRead(), MemRead(), and MemWrite() stay on the blocking path.
 *
 * 支持 LibXR 操作模式参数；默认同步完成，DMA 启用后仅 BLOCK 流式传输使用 DMA /
 * LibXR operation mode parameters are accepted; transfers complete synchronously
 * by default, and only BLOCK stream transfers use DMA when DMA is enabled:
 * - BLOCK：直接返回最终 ErrorCode，不触发回调或状态更新 /
 *   BLOCK: returns the final ErrorCode directly without callback/status update.
 * - POLLING：同步路径返回前更新为 DONE/ERROR；DMA 启用时返回 NOT_SUPPORT。
 *   POLLING: synchronous paths update DONE/ERROR before return; DMA-enabled
 *   POLLING transfers return NOT_SUPPORT.
 * - CALLBACK：同步路径返回前执行回调；DMA 启用时返回 NOT_SUPPORT。
 *   CALLBACK: synchronous paths invoke the callback before return; DMA-enabled
 *   CALLBACK transfers return NOT_SUPPORT.
 * - NONE：同步路径只返回最终 ErrorCode；DMA 启用时返回 NOT_SUPPORT。
 *   NONE: synchronous paths only return the final ErrorCode; DMA-enabled NONE
 *   transfers return NOT_SUPPORT.
 *
 * 多系列适配依赖构造参数传入的 `SPI_Type*`/`clock_name_t`、HPM SDK
 * `SPI_SOC_TRANSFER_COUNT_MAX` 传输上限，以及工程/board header 暴露的实例、IRQ、DMA
 * 宏；驱动本身不按 HPM 系列名分叉。
 * Multi-series adaptation relies on the constructor-provided `SPI_Type*` /
 * `clock_name_t`, the HPM SDK `SPI_SOC_TRANSFER_COUNT_MAX` transfer limit, and
 * instance, IRQ, and DMA macros exposed by project or board headers. The driver
 * itself does not branch by HPM series name.
 *
 * 编译期支持由 `HPMSOC_HAS_HPMSDK_SPI` 和 `__has_include("hpm_spi_drv.h")`
 * 同时 gate；缺少能力宏或裁剪 SDK 头时仍保留 LibXR API 形状，但配置和传输接口返回
 * `NOT_SUPPORT`，避免无 SPI SoC 或裁剪 SDK 的 glob 构建直接失败。
 * Compile-time support is gated by both `HPMSOC_HAS_HPMSDK_SPI` and
 * `__has_include("hpm_spi_drv.h")`. When the capability macro or trimmed SDK header
 * is missing, the LibXR API shape remains available but configuration and transfer
 * APIs return `NOT_SUPPORT`, avoiding direct glob-build failures on SoCs or trimmed
 * SDKs without SPI.
 *
 * `SetChipSelect()`, `SetDmaEnabled()`, `IsDmaEnabled()`, `IsDmaSupported()`,
 * `CommandRead()`, and `CommandWriteRead()` are HPM-specific convenience
 * extensions. They are intentionally not part of the generic `SPI` virtual
 * interface in `src/driver/spi.hpp`.
 */
class HPMSPI final : public SPI
{
 public:
  /**
   * @enum ChipSelect
   * @brief 选择 HPM SPI 硬件片选线 / Select the HPM SPI hardware chip-select line.
   *
   * 该枚举只影响 HPM SPI 控制器的硬件 CS 输出。当工程使用 GPIO 手动片选时，
   * 调用方仍应在一次完整事务外部拉低/拉高 GPIO CS。
   * This enum affects only the HPM SPI controller hardware CS output. When a project
   * uses a GPIO-managed chip select, the caller must still assert/deassert the GPIO
   * around one complete logical transaction.
   */
  enum class ChipSelect : uint8_t
  {
    CS0 = 0,  ///< 硬件片选 0 / Hardware chip-select 0.
    CS1,      ///< 硬件片选 1 / Hardware chip-select 1.
    CS2,      ///< 硬件片选 2 / Hardware chip-select 2.
    CS3       ///< 硬件片选 3 / Hardware chip-select 3.
  };

  /**
   * @brief 构造 HPM SPI 主机对象 / Construct an HPM SPI master object.
   * @param spi HPM SPI 外设基地址，不能为空 /
   * HPM SPI peripheral base address. Must not be nullptr.
   * @param clock HPM 时钟树名称；当板级 helper 未提供时用于开启并查询外设源时钟 /
   * HPM clock name used to enable and query the peripheral source clock when the
   * optional board helper does not provide one.
   * @param rx_buffer 内部 RX 暂存缓冲区，供 ReadAndWrite()、Transfer()、MemRead()
   * 使用；驱动生命周期内必须保持非空且有效，并满足 `alignof(size_t)` 对齐及
   * `2 * alignof(size_t)` 整倍数容量要求 / Internal RX staging buffer used by
   * ReadAndWrite(), Transfer(), and MemRead(). It must remain non-null and valid for
   * the driver lifetime, be aligned to `alignof(size_t)`, and have a size divisible
   * by `2 * alignof(size_t)`.
   * @param tx_buffer 内部 TX 暂存缓冲区，供所有传输接口使用；驱动生命周期内必须
   * 保持非空且有效，并满足相同的对齐和容量要求 / Internal TX staging buffer used
   * by all transfer methods. It must satisfy the same lifetime, alignment, and size
   * requirements.
   * @param auto_board_init 若为 true 且存在 board.h，则自动调用板级 SPI 时钟和引脚
   * 初始化 helper / If true and board.h is available, call board SPI clock and pin
   * helper functions automatically.
   * @param config 初始 SPI 配置；该驱动按 8-bit、MSB-first、single-I/O、master
   * 模式配置硬件；HPM 后端当前支持 DIV_1 和不超过 DIV_256 的偶数分频 /
   * Initial SPI configuration. This driver configures hardware as 8-bit,
   * MSB-first, single-I/O master. The current HPM backend supports DIV_1 and
   * even prescalers up to DIV_256.
   * @param cs 硬件片选线；仅在 HPM SDK/SoC 暴露 `cs_index` 时写入控制寄存器 /
   * Hardware chip-select line. It is written to the transfer control register only
   * when the HPM SDK/SoC exposes `cs_index`.
   *
   * @note RX/TX 暂存区在全双工传输使用的范围内不得重叠；检测到重叠时传输返回
   * ARG_ERR。RX and TX staging ranges used by a full-duplex transfer must not overlap;
   * an overlapping transfer returns ARG_ERR.
   * @note 构造函数会断言外设指针为空、暂存缓冲区为空或不满足基类对齐/容量要求、
   * 源时钟无法解析或初始配置无效 / The constructor asserts on null peripheral pointer,
   * null/empty staging buffers, staging buffers that violate the base-class
   * alignment/size contract, unresolved source clock, or invalid initial configuration.
   */
  HPMSPI(LibXRHpmSpiType* spi, clock_name_t clock, RawData rx_buffer, RawData tx_buffer,
         bool auto_board_init = true,
         SPI::Configuration config = {SPI::ClockPolarity::LOW, SPI::ClockPhase::EDGE_1,
                                      SPI::Prescaler::DIV_4, false},
         ChipSelect cs = ChipSelect::CS0);

  HPMSPI(const HPMSPI&) = delete;
  HPMSPI& operator=(const HPMSPI&) = delete;

  /**
   * @brief 传输 SPI 字节，并可同时采集接收数据 /
   * Transfer SPI bytes while optionally collecting received bytes.
   *
   * 当读写缓冲区都非空时，实际传输长度取两者较大值，较短的写载荷由内部 TX
   * 暂存缓冲区补 0。只有 read_data 非空时执行只读传输；只有 write_data 非空时
   * 执行只写传输。
   * When both buffers are non-empty, the transfer length is the larger size and the
   * shorter write payload is padded with zero bytes in the internal TX staging buffer.
   * If only read_data is non-empty, a read-only transfer is used; if only write_data
   * is non-empty, a write-only transfer is used.
   *
   * @note 该流式接口不会自动丢弃命令阶段收到的前缀字节；W25Q128 `0x9F` 这类
   * opcode-read 命令应使用 CommandRead() 或 CommandWriteRead()。
   * This stream API does not discard bytes received during a command phase. Use
   * CommandRead() or CommandWriteRead() for opcode-read commands such as W25Q128
   * `0x9F`.
   *
   * @param read_data 目标缓冲区；size_ 为 0 表示不拷贝接收数据，非零长度时 addr_
   * 必须非空 / Destination buffer. A zero-size buffer disables receive copy; for
   * non-zero size, addr_ must be non-null.
   * @param write_data 源缓冲区；size_ 为 0 表示不发送用户载荷，非零长度时 addr_
   * 必须非空 / Source buffer. A zero-size buffer disables user transmit payload; for
   * non-zero size, addr_ must be non-null.
   * @param op LibXR 读写操作描述符，见类注释中的操作模式说明 /
   * LibXR read/write operation descriptor. See class-level operation mode notes.
   * @param in_isr 仅用于 CALLBACK/POLLING 完成分发的中断上下文标志 /
   * ISR-context flag forwarded only to CALLBACK/POLLING completion handling.
   * @return 成功返回 OK，包括读写长度都为 0 的情况；空用户缓冲区返回 PTR_NULL；
   * 传输长度超过 SPI_SOC_TRANSFER_COUNT_MAX 或内部暂存缓冲区容量返回 SIZE_ERR；
   * 其余返回 HPM SDK 状态转换后的 TIMEOUT、BUSY 或 FAILED /
   * Returns OK on success, including when both sizes are zero; PTR_NULL for null
   * non-empty user buffer; SIZE_ERR when the effective length exceeds
   * SPI_SOC_TRANSFER_COUNT_MAX or staging buffer capacity; otherwise converted HPM
   * SDK status such as TIMEOUT, BUSY, or FAILED.
   */
  ErrorCode ReadAndWrite(RawData read_data, ConstRawData write_data, OperationRW& op,
                         bool in_isr = false) override;

  /**
   * @brief 应用 SPI 格式和时序配置 / Apply SPI format and timing configuration.
   * @param config 目标时钟极性、相位、分频和双缓冲标志 /
   * Desired clock polarity, phase, prescaler, and double-buffer flag.
   * @return 成功返回 OK；外设指针为空返回 PTR_NULL；源时钟无法解析返回 INIT_ERR；
   * 分频无效或计算出的 SCLK 为 0 返回 ARG_ERR；超过 DIV_256 返回 NOT_SUPPORT；
   * 其余返回 HPM SDK 时序配置状态 /
   * Returns OK on success, PTR_NULL for null peripheral pointer, INIT_ERR when source
   * clock cannot be resolved, ARG_ERR for invalid prescaler or zero SCLK, NOT_SUPPORT
   * beyond DIV_256, or converted HPM SDK timing status otherwise.
   */
  ErrorCode SetConfig(SPI::Configuration config) override;

  /**
   * @brief 设置后续传输使用的硬件片选线 /
   * Set the hardware chip-select line used by subsequent transfers.
   *
   * @param cs 目标硬件 CS0..CS3 / Target hardware CS0..CS3.
   * @return 成功返回 OK；当前 SDK/SoC 未暴露硬件 CS 选择时返回 NOT_SUPPORT /
   * Returns OK on success, or NOT_SUPPORT when the current SDK/SoC does not expose
   * hardware CS selection.
   *
   * @note 该函数只更新驱动内部选择，实际 CS 字段在下一次构造 HPM transfer control
   * 时写入；不要在传输期间调用。
   * This function only updates the driver's stored selection. The HPM transfer
   * control register is programmed on the next transfer; do not call it while a
   * transfer is active.
   *
   * @note 如果当前 SoC/SDK 不暴露 `cs_index`，构造函数传入的 CS
   * 只会被保存，硬件传输不会使用它； SetChipSelect() 将返回 NOT_SUPPORT。/ If the current
   * SoC/SDK does not expose `cs_index`, the constructor-provided CS is only stored and is
   * not applied to hardware transfers; SetChipSelect() returns NOT_SUPPORT.
   */
  ErrorCode SetChipSelect(ChipSelect cs);

  /**
   * @brief 获取当前硬件片选线 / Get the current hardware chip-select line.
   * @return 当前配置的硬件 CS / Current configured hardware CS.
   */
  ChipSelect GetChipSelect() const { return cs_; }

  /**
   * @brief 启用或关闭 HPM SPI DMA manager 路径 /
   * Enable or disable the HPM SPI DMA-manager path.
   *
   * DMA 路径仅在工程启用 `USE_DMA_MGR=1` 且 HPM SDK 版本不低于 1.11.0、提供
   * `hpm_spi.h` 自定义 DMA 回调 API 时可用。它只覆盖普通 data-phase 流式传输，即
   * ReadAndWrite() 和 Transfer()；CommandRead() / CommandWriteRead() 仍使用 HPM SDK
   * command phase 阻塞传输，以保持 SPI flash opcode-read 时序。
   *
   * The DMA path is available only when the project builds with `USE_DMA_MGR=1`,
   * HPM SDK 1.11.0 or newer is used, and the `hpm_spi.h` custom DMA callback APIs
   * are present. It covers regular data-phase stream transfers through ReadAndWrite()
   * and Transfer() only. CommandRead() / CommandWriteRead() keep using the blocking
   * HPM SDK command phase so SPI-flash opcode-read timing remains unchanged.
   * 在 system/none、webasm 等轮询等待系统中，BLOCK DMA 操作依赖 LibXR
   * Timebase 进行超时等待；未初始化 Timebase 时，ReadAndWrite() / Transfer()
   * 会在启动 DMA 前返回 INIT_ERR。RTOS/POSIX 系统使用各自的 semaphore 等待机制。
   * On polling-wait systems such as system/none and webasm, BLOCK DMA operations
   * require a LibXR Timebase for timeout-aware waits; otherwise ReadAndWrite() /
   * Transfer() return INIT_ERR before starting DMA. RTOS/POSIX systems use their
   * native semaphore wait paths.
   *
   * @note 启用 D-cache 的平台上，DMA RX 会优先使用内部 RX staging buffer 中的
   * cache-line 独占区域；若原始 staging buffer 不满足直接 DMA 条件，驱动会尝试在 buffer
   * 内选择对齐子区并在完成后搬回原始 staging 起点。若空间不足，BLOCK 传输回退到同步路径，
   * 避免无效化共享 cache line 时丢失其他 dirty data。On D-cache-enabled platforms,
   * DMA RX first uses a cache-line-exclusive range inside the internal RX staging
   * buffer. If the original staging buffer is not directly DMA-safe, the driver
   * tries an aligned subrange and moves received bytes back after completion. If
   * no such range fits, BLOCK transfers fall back to the synchronous path, avoiding
   * unrelated dirty-data loss when invalidating shared cache lines.
   * @note 当前 HPM SDK SPI component 固定按 core0 转换 DMA 地址，因此双核 SoC 的
   * core1 上 SetDmaEnabled(true) 返回 NOT_SUPPORT。若传输长度超过 DMA 控制器计数范围，
   * BLOCK 操作回退到同步路径 / The current HPM SDK SPI component converts DMA
   * addresses as core0, so SetDmaEnabled(true) returns NOT_SUPPORT on core1 of a
   * dual-core SoC. BLOCK transfers larger than the DMA controller count field fall
   * back to the synchronous path.
   *
   * @param enabled true 启用 DMA，false 回到同步阻塞路径 /
   * true to enable DMA, false to use the synchronous blocking path.
   * @return OK 表示状态已更新；BUSY 表示有后台 DMA 事务未完成；NOT_SUPPORT 表示
   * 当前构建未提供 SPI DMA manager 支持 /
   * OK when updated, BUSY while an async DMA transaction is active, or
   * NOT_SUPPORT when this build does not provide SPI DMA-manager support.
   */
  ErrorCode SetDmaEnabled(bool enabled);

  /**
   * @brief 查询当前是否启用 DMA 流式传输 / Query whether DMA stream transfers are
   * enabled.
   * @return true 表示 ReadAndWrite()/Transfer() 会优先使用 DMA /
   * true when ReadAndWrite()/Transfer() prefer DMA.
   */
  bool IsDmaEnabled() const { return dma_enabled_; }

  /**
   * @brief 查询当前构建是否支持 HPM SPI DMA manager /
   * Query build-time HPM SPI DMA-manager support.
   * @return true 表示可调用 SetDmaEnabled(true) /
   * true when DMA-manager support is compiled into this build.
   * @note This is a build-time indicator only. SetDmaEnabled(true) remains the
   * authoritative runtime check because resource allocation and callback setup
   * can still fail.
   */
  static constexpr bool IsDmaSupported() { return LIBXR_HPM_SPI_HAS_DMA_MGR != 0; }

  /**
   * @brief 查询控制器是否已有可用配置 / Query whether the controller has a
   * valid applied configuration.
   *
   * 当底层 HPM 控制器复位/恢复失败时，该状态会被清除，后续传输将返回 INIT_ERR，
   * 调用方可用它区分普通传输错误和控制器已不可用状态。
   * The flag is cleared when low-level HPM controller reset/recovery fails. Later
   * transfers return INIT_ERR, and callers can use this query to distinguish a
   * normal transfer error from a controller that is no longer configured.
   *
   * @return true 表示 SetConfig() 已成功应用且恢复流程未清除该状态 /
   * true when SetConfig() succeeded and recovery has not cleared the state.
   */
  bool IsConfigured() const { return configured_; }

  /**
   * @brief 发送 8-bit 命令后读取数据 / Read data after sending an 8-bit command.
   *
   * 该接口使用 HPM SDK 的 command phase：先在同一次硬件事务中发送 `command`，
   * 再进入 read-only data phase 读取 `read_data.size_` 字节。它适合 SPI flash
   * JEDEC ID、状态寄存器、电子 ID 等 opcode-read 命令，避免调用方手工拼接
   * dummy 字节并丢弃前缀接收字节。
   * This method uses the HPM SDK command phase: it sends `command` and then reads
   * `read_data.size_` bytes in the read-only data phase of the same hardware
   * transaction. It is intended for opcode-read commands such as SPI flash JEDEC ID,
   * status-register, and electronic-ID reads, avoiding manual dummy bytes and prefix
   * RX discard in the caller.
   *
   * 当工程使用 GPIO 手动片选时，调用方仍需在调用该函数外层包住一次完整 CS 低电平窗口。
   * When GPIO chip select is used, the caller must still wrap this call in one full
   * active CS window.
   *
   * @param command 8-bit 命令字节 / 8-bit command byte.
   * @param read_data 接收缓冲区；size_ 为 0 时不访问总线并返回 OK /
   * Destination buffer. A zero-size buffer completes with OK without bus access.
   * @param op LibXR 读写操作描述符 / LibXR read/write operation descriptor.
   * @param in_isr CALLBACK/POLLING 完成分发的中断上下文标志 /
   * ISR-context flag forwarded to CALLBACK/POLLING completion handling.
   * @return 成功返回 OK；空目标缓冲区返回 PTR_NULL；长度超过硬件限制或暂存缓冲区容量返回
   * SIZE_ERR；其余返回 HPM SDK 状态转换后的错误码 /
   * Returns OK on success, PTR_NULL for a null non-empty destination, SIZE_ERR when
   * the read length exceeds hardware limits or staging capacity, or converted HPM SDK
   * status otherwise.
   */
  ErrorCode CommandRead(uint8_t command, RawData read_data, OperationRW& op,
                        bool in_isr = false);

  /**
   * @brief 发送 8-bit 命令后顺序写入并读取数据 /
   * Write and then read data after sending an 8-bit command.
   *
   * 该接口把 `command` 放入 HPM SDK command phase，随后在 data phase 中按
   * `write_data`、`read_data` 的存在情况选择 write-only、read-only、write-then-read
   * 或 no-data 事务。它可覆盖 SPI flash 的 command-only、command+read、
   * command+address-as-write-data+read 等常见命令模式；如需 dummy 字节，请把它们
   * 放入 `write_data`，本 API 不配置硬件 dummy cycle。
   * This method places `command` in the HPM SDK command phase, then selects
   * write-only, read-only, write-then-read, or no-data transaction mode from the
   * presence of `write_data` and `read_data`. It covers common SPI flash command
   * shapes such as command-only, command+read, and command+address-as-write-data+read.
   * If dummy bytes are required, include them in `write_data`; this API does not
   * configure hardware dummy cycles.
   *
   * 当工程使用 GPIO 手动片选时，调用方仍需在调用该函数外层包住一次完整 CS 低电平窗口。
   * When GPIO chip select is used, the caller must still wrap this call in one full
   * active CS window.
   *
   * @param command 8-bit 命令字节 / 8-bit command byte.
   * @param write_data 命令后的写入数据；size_ 为 0 时跳过写数据阶段 /
   * Data written after the command. A zero-size buffer skips the write data phase.
   * @param read_data 写入数据后的读取缓冲区；size_ 为 0 时跳过读数据阶段 /
   * Destination buffer read after the write data phase. A zero-size buffer skips the
   * read data phase.
   * @param op LibXR 读写操作描述符 / LibXR read/write operation descriptor.
   * @param in_isr CALLBACK/POLLING 完成分发的中断上下文标志 /
   * ISR-context flag forwarded to CALLBACK/POLLING completion handling.
   * @return 成功返回 OK；非零长度空缓冲区返回
   * PTR_NULL；任一阶段长度超过硬件限制或暂存缓冲区容量返回 SIZE_ERR；其余返回 HPM SDK
   * 状态转换后的错误码 / Returns OK on success, PTR_NULL for a null non-empty buffer,
   * SIZE_ERR when either phase exceeds hardware limits or staging capacity, or converted
   * HPM SDK status otherwise.
   *
   * @note command-only 和 write-then-read 路径依赖 HPM SPI command phase 以及对应
   * transfer mode；较旧 SDK 若不支持这些模式，底层会返回 SDK 错误并映射为 LibXR
   * ErrorCode。/ The command-only and write-then-read paths rely on the HPM SPI command
   * phase and matching transfer modes. Older SDKs that do not implement those modes will
   * return an SDK error that is converted to a LibXR ErrorCode.
   */
  ErrorCode CommandWriteRead(uint8_t command, ConstRawData write_data, RawData read_data,
                             OperationRW& op, bool in_isr = false);

  /**
   * @brief 使用 8-bit 命令字节读取 SPI 寄存器 /
   * Read bytes from an SPI register using an 8-bit command byte.
   *
   * 发送的命令字节为 static_cast<uint8_t>(reg | 0x80)：bit7 表示读操作，
   * bit0..6 为寄存器地址，reg 高于 bit7 的位视为越界。
   * The transmitted command byte is static_cast<uint8_t>(reg | 0x80): bit 7 marks
   * a read operation and bits 0..6 carry the register address. Bits above bit 7 are
   * treated as out of range.
   *
   * @param reg 寄存器地址；置读位前的有效范围为 0x00..0x7F /
   * Register address. Effective range before applying the read bit is 0x00..0x7F.
   * @param read_data 目标载荷缓冲区；size_ 为 0 时不访问总线并返回 OK；非零长度时
   * addr_ 必须非空 / Destination payload buffer. A zero-size buffer completes with OK
   * without bus access; for non-zero size, addr_ must be non-null.
   * @param op LibXR 读写操作描述符，见类注释中的操作模式说明 /
   * LibXR read/write operation descriptor. See class-level operation mode notes.
   * @param in_isr 仅用于 CALLBACK/POLLING 完成分发的中断上下文标志 /
   * ISR-context flag forwarded only to CALLBACK/POLLING completion handling.
   * @return 成功返回 OK；空目标缓冲区返回 PTR_NULL；寄存器地址超过 0x7F 返回
   * OUT_OF_RANGE；命令字节加载荷超过硬件限制或暂存缓冲区容量返回 SIZE_ERR；其余返回
   * HPM SDK 状态转换后的错误码 / Returns OK on success, PTR_NULL for null non-empty
   * destination, OUT_OF_RANGE for register addresses above 0x7F, SIZE_ERR when command
   * byte plus payload exceeds hardware limit or staging capacity, or converted HPM SDK
   * status otherwise.
   * @note 该接口保留给“bit7 表示读写”的寄存器型 SPI 设备；它不是通用 SPI flash
   * opcode-read API。读取 W25Q128 `0x9F` 这类命令应优先使用 CommandRead()。
   * This method is kept for register-style SPI devices where bit 7 marks read/write.
   * It is not a generic SPI flash opcode-read API; prefer CommandRead() for W25Q128
   * commands such as `0x9F`.
   */
  ErrorCode MemRead(uint16_t reg, RawData read_data, OperationRW& op,
                    bool in_isr = false) override;

  /**
   * @brief 使用 8-bit 命令字节写入 SPI 寄存器 /
   * Write bytes to an SPI register using an 8-bit command byte.
   *
   * 发送的命令字节为 static_cast<uint8_t>(reg & 0x7F)：bit7 被清零表示写操作，
   * bit0..6 为寄存器地址，reg 高于 bit7 的位视为越界。零长度载荷合法，只发送命令字节。
   * The transmitted command byte is static_cast<uint8_t>(reg & 0x7F): bit 7 is
   * cleared for write operation and bits 0..6 carry the register address. Bits above
   * bit 7 are treated as out of range. A zero-size payload is valid and sends only the
   * command byte.
   *
   * @param reg 寄存器地址；有效范围为 0x00..0x7F /
   * Register address. Effective range is 0x00..0x7F.
   * @param write_data 源载荷缓冲区；非零长度时 addr_ 必须非空 /
   * Source payload buffer. For non-zero size, addr_ must be non-null.
   * @param op LibXR 读写操作描述符，见类注释中的操作模式说明 /
   * LibXR read/write operation descriptor. See class-level operation mode notes.
   * @param in_isr 仅用于 CALLBACK/POLLING 完成分发的中断上下文标志 /
   * ISR-context flag forwarded only to CALLBACK/POLLING completion handling.
   * @return 成功返回 OK；空载荷指针返回 PTR_NULL；寄存器地址超过 0x7F 返回 OUT_OF_RANGE；
   * 命令字节加载荷超过硬件限制或 TX 暂存缓冲区容量返回 SIZE_ERR；其余返回 HPM SDK
   * 状态转换后的错误码 / Returns OK on success, PTR_NULL for null non-empty payload,
   * OUT_OF_RANGE for register addresses above 0x7F, SIZE_ERR when command byte plus
   * payload exceeds hardware limit or TX staging capacity, or converted HPM SDK status
   * otherwise.
   */
  ErrorCode MemWrite(uint16_t reg, ConstRawData write_data, OperationRW& op,
                     bool in_isr = false) override;

  /**
   * @brief 获取 SPI 源时钟频率，即理论最高总线速度 /
   * Get SPI source clock frequency, the theoretical fastest bus speed.
   * @return 源时钟频率，单位 Hz；成功解析时钟前为 0 /
   * Source clock frequency in Hz, or 0 before successful clock discovery.
   */
  uint32_t GetMaxBusSpeed() const override;

  /**
   * @brief 获取该 HPM SPI 后端支持的最慢分频 /
   * Get the slowest prescaler supported by this HPM SPI backend.
   * @return Prescaler::DIV_256 / Prescaler::DIV_256.
   */
  Prescaler GetMaxPrescaler() const override;

  /**
   * @brief 从当前活动 TX 暂存缓冲区传输到 RX 暂存缓冲区 /
   * Transfer from the active TX staging buffer into the RX staging buffer.
   *
   * 这是给已通过 GetTxBuffer() 准备数据的调用者使用的零拷贝路径。当前活动 RX/TX
   * 暂存缓冲区都必须至少有 size 字节。传输结束后调用 SwitchBuffer()，使双缓冲用户
   * 切换到另一组缓冲区；当前实现仅在传输成功后执行 SwitchBuffer()。
   * This is the zero-copy path for callers that prepared data through GetTxBuffer().
   * The active RX/TX staging buffers must both contain at least size bytes. After the
   * transfer, SwitchBuffer() is called so double-buffer users advance to the other
   * buffer pair; the current implementation switches only after successful transfer.
   *
   * @param size 传输字节数；0 合法并直接返回 OK；非零时不能超过
   * SPI_SOC_TRANSFER_COUNT_MAX / Number of bytes to transfer. Zero is valid and
   * completes with OK; non-zero size must not exceed SPI_SOC_TRANSFER_COUNT_MAX.
   * @param op LibXR 读写操作描述符，见类注释中的操作模式说明 /
   * LibXR read/write operation descriptor. See class-level operation mode notes.
   * @param in_isr 仅用于 CALLBACK/POLLING 完成分发的中断上下文标志 /
   * ISR-context flag forwarded only to CALLBACK/POLLING completion handling.
   * @return 成功返回 OK；长度超过硬件限制或活动暂存缓冲区容量返回 SIZE_ERR；活动
   * 暂存指针为空返回 PTR_NULL；其余返回 HPM SDK 状态转换后的错误码 /
   * Returns OK on success, SIZE_ERR when size exceeds hardware limit or active staging
   * capacity, PTR_NULL for null active staging pointer, or converted HPM SDK status
   * otherwise.
   */
  ErrorCode Transfer(size_t size, OperationRW& op, bool in_isr = false) override;

 private:
  /**
   * @brief 将 HPM SDK SPI 状态码转换为 LibXR 错误码 /
   * Convert HPM SDK SPI status to LibXR ErrorCode.
   */
  static ErrorCode ConvertStatus(LibXRHpmSpiStatusType status);

#if LIBXR_HPM_SPI_SUPPORTED
  /**
   * @brief 将片选枚举转换为 HPM SDK CS 掩码 / Convert chip-select enum to HPM SDK CS
   * mask.
   */
  static uint8_t ConvertChipSelect(ChipSelect cs);
#endif

  /**
   * @brief 转换 LibXR SPI 时钟极性到 HPM SDK 枚举 /
   * Convert LibXR SPI clock polarity to HPM SDK enum.
   */
#if LIBXR_HPM_SPI_SUPPORTED
  static spi_sclk_idle_state_t ConvertPolarity(ClockPolarity polarity);

  /**
   * @brief 转换 LibXR SPI 时钟相位到 HPM SDK 枚举 /
   * Convert LibXR SPI clock phase to HPM SDK enum.
   */
  static spi_sclk_sampling_clk_edges_t ConvertPhase(ClockPhase phase);
#endif

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

  /**
   * @brief 原子获取实例事务所有权 / Atomically acquire instance transaction ownership.
   */
  bool TryAcquireTransaction()
  {
    uint32_t expected = 0U;
    return transaction_busy_.compare_exchange_strong(
        expected, 1U, std::memory_order_acq_rel, std::memory_order_acquire);
  }

  /**
   * @brief 释放事务所有权并返回结果 / Release transaction ownership and return a result.
   */
  ErrorCode EndTransaction(ErrorCode ans)
  {
    transaction_busy_.store(0U, std::memory_order_release);
    return ans;
  }

  /**
   * @brief 释放事务所有权后分发操作结果 / Release ownership before dispatching a result.
   */
  ErrorCode EndTransaction(OperationRW& op, bool in_isr, ErrorCode ans)
  {
    transaction_busy_.store(0U, std::memory_order_release);
    return FinishOperation(op, in_isr, ans);
  }

  /**
   * @brief 校验 LibXR SPI 分频参数是否可映射到 HPM SPI timing /
   * Validate whether a LibXR SPI prescaler can map to HPM SPI timing.
   */
  static ErrorCode ValidatePrescaler(Prescaler prescaler);

  /**
   * @brief 根据 LibXR 分频参数配置 HPM SPI 时序 /
   * Configure HPM SPI timing from LibXR prescaler.
   */
  ErrorCode ApplyTiming(Prescaler prescaler);

  /**
   * @brief 校验 HPM SPI 配置参数 / Validate HPM SPI configuration.
   */
  ErrorCode ValidateConfiguration(const Configuration& config) const;

  /**
   * @brief 确保源时钟频率已可用 / Ensure the source clock frequency is available.
   */
  ErrorCode EnsureClockReady();

  /**
   * @brief 按当前 LibXR 配置写入 HPM SPI 格式寄存器 /
   * Apply HPM SPI format registers from the current LibXR configuration.
   */
  void ApplyFormat(const Configuration& config);

  /**
   * @brief 构造一次普通主机传输的控制参数 /
   * Build control configuration for a regular master transfer.
   */
#if LIBXR_HPM_SPI_SUPPORTED
  spi_control_config_t MakeControlConfig(spi_trans_mode_t mode) const;

  /**
   * @brief 将已保存的硬件片选写入控制器 / Apply the stored hardware CS.
   */
  void ApplyChipSelect() const;
#endif

  /**
   * @brief 判断失败后是否需要复位控制器 / Decide whether a failed transfer should
   * recover the controller.
   */
  static bool ShouldRecover(LibXRHpmSpiStatusType status);

  /**
   * @brief 复位 HPM SPI 控制器并恢复上次成功配置 /
   * Reset the HPM SPI controller and restore the last successful configuration.
   */
  void RecoverController();

  /**
   * @brief 执行全双工写读同时传输 / Execute full-duplex write-read transfer.
   */
  ErrorCode DoTransfer(uint8_t* rx, const uint8_t* tx, uint32_t size);

  /**
   * @brief 执行只写传输 / Execute write-only transfer.
   */
  ErrorCode DoWriteOnly(const uint8_t* tx, uint32_t size);

  /**
   * @brief 执行只读传输 / Execute read-only transfer.
   */
  ErrorCode DoReadOnly(uint8_t* rx, uint32_t size);

  /**
   * @brief 执行 command phase + read data phase 传输 /
   * Execute a command-phase plus read data-phase transfer.
   */
  ErrorCode DoCommandRead(uint8_t command, uint8_t* rx, uint32_t size);

  /**
   * @brief 执行 command phase + write/read data phase 传输 /
   * Execute a command-phase plus optional write/read data-phase transfer.
   */
  ErrorCode DoCommandWriteRead(uint8_t command, const uint8_t* tx, uint32_t tx_size,
                               uint8_t* rx, uint32_t rx_size);

#if LIBXR_HPM_SPI_HAS_DMA_MGR
  /**
   * @brief HPM SPI DMA 后台事务类型 / HPM SPI DMA background transfer kind.
   */
  enum class DmaTransferKind : uint8_t
  {
    NONE,
    READ_ONLY,
    WRITE_ONLY,
    WRITE_READ
  };

  /**
   * @brief HPM SPI DMA 事务上下文 / HPM SPI DMA transfer context.
   */
  struct DmaTransferContext
  {
    DmaTransferKind kind = DmaTransferKind::NONE;
    OperationRW* op = nullptr;
    uint8_t* rx = nullptr;
    uint8_t* staging_rx = nullptr;
    uint8_t* tx = nullptr;
    RawData user_read = {nullptr, 0};
    uint32_t size = 0;
    bool copy_rx_to_user = false;
    bool copy_rx_to_staging = false;
    bool switch_buffer_on_success = false;
    std::atomic<uint32_t> block_dma_ready{0U};
    std::atomic<uint32_t> rx_done{0U};
    std::atomic<uint32_t> tx_done{0U};
  };

  /**
   * @brief 将 DMA manager 状态码转换为 LibXR 错误码 /
   * Convert DMA-manager status to LibXR ErrorCode.
   */
  static ErrorCode ConvertDmaStatus(hpm_stat_t status);

  /**
   * @brief 准备并安装 SPI DMA manager 回调 / Prepare and install SPI DMA-manager
   * callbacks.
   */
  ErrorCode EnsureDmaReady();

  /**
   * @brief 查询当前是否有 DMA 后台事务 / Query whether a DMA transaction is active.
   */
  bool DmaTransferActive() const
  {
    return dma_busy_.load(std::memory_order_acquire) != 0U;
  }

  /**
   * @brief 启动一次 data-phase DMA 传输 / Start one data-phase DMA transfer.
   */
  ErrorCode StartDmaTransfer(uint8_t* rx, uint8_t* tx, uint32_t size,
                             size_t rx_capacity, DmaTransferKind kind, RawData user_read,
                             bool copy_rx_to_user, bool switch_buffer_on_success,
                             OperationRW& op, bool in_isr);

  /**
   * @brief 停止当前 DMA 请求和通道 / Stop current DMA requests and channels.
   */
  void StopDmaTransfer();

  /**
   * @brief 清除 RX/TX DMA 通道的残留状态 / Clear stale RX/TX DMA channel status.
   */
  void ClearDmaTransferStatus();

  /**
   * @brief 清空 DMA 事务上下文 / Clear DMA transfer context.
   */
  void ClearDmaContext();

  /**
   * @brief 在 DMA 启动失败后回滚状态 / Roll back state after DMA start failure.
   */
  void AbortDmaStart();

  /**
   * @brief 等待 BLOCK 模式 DMA 结果 / Wait for a BLOCK-mode DMA result.
   */
  ErrorCode WaitForDmaBlockResult(uint32_t timeout);

  /**
   * @brief 查询 DMA 接收缓存契约是否安全 / Check whether the DMA RX cache contract is safe.
   */
  static bool DmaRxBufferCacheSafe(const void* addr, size_t capacity);

  /**
   * @brief 使用同步路径执行一次流式传输 / Run one stream transfer through the blocking path.
   */
  ErrorCode RunBlockingStreamTransfer(uint8_t* rx, const uint8_t* tx, uint32_t size,
                                      DmaTransferKind kind, RawData user_read,
                                      bool copy_rx_to_user,
                                      bool switch_buffer_on_success,
                                      OperationRW& op, bool in_isr);

  /**
   * @brief DMA 回调后检查事务是否完成 / Check whether DMA callbacks complete the
   * transfer.
   */
  void MaybeCompleteDmaTransfer(bool in_isr);

  /**
   * @brief 完成并分发 DMA 事务结果 / Complete and dispatch a DMA transaction result.
   */
  ErrorCode CompleteDmaTransfer(bool in_isr, ErrorCode ans, bool notify_block = true);

  /**
   * @brief 抢占 DMA 完成所有权 / Claim single DMA completion ownership.
   */
  bool TryClaimDmaCompletion();

  /**
   * @brief RX DMA terminal-count callback / RX DMA 终端计数回调。
   */
  static void OnRxDmaTcCallback(DMA_Type* base, uint32_t channel, void* cb_data_ptr);

  /**
   * @brief TX DMA terminal-count callback / TX DMA 终端计数回调。
   */
  static void OnTxDmaTcCallback(DMA_Type* base, uint32_t channel, void* cb_data_ptr);

  /**
   * @brief DMA error/abort callback / DMA 错误或中止回调。
   */
  static void OnDmaFailureCallback(DMA_Type* base, uint32_t channel,
                                   void* cb_data_ptr);
#endif

  LibXRHpmSpiType* spi_;           ///< SPI 外设实例 / SPI peripheral instance.
  clock_name_t clock_;             ///< SPI 源时钟名称 / SPI source clock name.
  uint32_t source_clock_hz_ = 0;   ///< 缓存的源时钟频率 / Cached source clock frequency.
  size_t rx_buffer_capacity_ = 0;  ///< 原始 RX 缓冲区容量 / Raw RX buffer capacity.
  size_t tx_buffer_capacity_ = 0;  ///< 原始 TX 缓冲区容量 / Raw TX buffer capacity.
  std::atomic<uint32_t> transaction_busy_{
      0U};  ///< 实例事务占用标志 / Instance transaction ownership flag.
  bool configured_ = false;  ///< 是否已有成功配置 / Whether a config was applied.
  bool dma_enabled_ =
      false;  ///< 是否启用 DMA 流式传输 / Whether DMA stream path is enabled.
#if LIBXR_HPM_SPI_HAS_DMA_MGR
  bool dma_ready_ =
      false;  ///< DMA manager 回调是否已安装 / Whether DMA callbacks are installed.
  std::atomic<uint32_t> dma_busy_{
      0U};  ///< DMA 后台事务活动标志 / DMA background transfer active flag.
  std::atomic<uint32_t> dma_completion_claim_{
      0U};                           ///< DMA 完成抢占标志 / DMA completion claim flag.
  DmaTransferContext dma_ctx_{};     ///< DMA 事务上下文 / DMA transfer context.
  AsyncBlockWait dma_block_wait_{};  ///< BLOCK 模式 DMA 等待器 / BLOCK-mode DMA waiter.
#endif
  bool auto_board_init_;  ///< 是否自动调用板级初始化 / Whether board init is automatic.
  ChipSelect cs_ = ChipSelect::CS0;  ///< 当前硬件片选 / Current hardware chip-select.
};

}  // namespace LibXR
