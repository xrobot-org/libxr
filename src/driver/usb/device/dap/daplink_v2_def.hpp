#pragma once

#include <cstdint>

#include "libxr_def.hpp"
#include "timebase.hpp"

namespace LibXR::USB
{

/**
 * @brief CMSIS-DAP v2（Bulk）协议定义（供 DapLinkV2Class 使用）。
 *        CMSIS-DAP v2 (Bulk) protocol definitions (for DapLinkV2Class).
 *
 * 仅包含定义/常量/结构体；不包含任何实现逻辑。
 * Definitions only; no implementation logic here.
 */
namespace DapLinkV2Def
{

// ==============================
// 限制 / Limits
// ==============================

static constexpr std::uint16_t MAX_REQUEST_SIZE =
    512u;  ///< 最大请求长度 / Max request size.
static constexpr std::uint16_t MAX_RESPONSE_SIZE =
    512u;  ///< 最大响应长度 / Max response size.

// ==============================
// 命令号 / CMSIS-DAP v2 Command IDs
// 注意：按规范值定义（SWO_Data=0x1C, SWD_Sequence=0x1D）
// Note: Values follow the CMSIS-DAP specification (SWO_Data=0x1C, SWD_Sequence=0x1D).
// ==============================

/**
 * @brief CMSIS-DAP v2 命令号枚举 / CMSIS-DAP v2 command identifiers.
 */
enum class CommandId : std::uint8_t
{
  // Core (0x00-0x0F)
  INFO = 0x00,         ///< 查询适配器或目标信息 / Query adapter or target information.
  HOST_STATUS = 0x01,  ///< 设置主机状态指示灯 / Set host status LEDs.
  CONNECT = 0x02,      ///< 连接调试端口 / Connect debug port.
  DISCONNECT = 0x03,   ///< 断开调试端口 / Disconnect debug port.
  TRANSFER_CONFIGURE = 0x04,  ///< 配置传输参数 / Configure DAP transfer parameters.
  TRANSFER = 0x05,            ///< 执行单组读写传输 / Execute single DAP transfer batch.
  TRANSFER_BLOCK = 0x06,      ///< 执行块读写传输 / Execute block DAP transfer.
  TRANSFER_ABORT = 0x07,      ///< 中止正在进行的传输 / Abort active transfer.
  WRITE_ABORT = 0x08,   ///< 写 CoreSight ABORT 寄存器 / Write CoreSight ABORT register.
  DELAY = 0x09,         ///< 微秒延时 / Delay for specified microseconds.
  RESET_TARGET = 0x0A,  ///< 复位目标设备 / Reset target device.

  // SWJ (0x10-0x1F)
  SWJ_PINS = 0x10,        ///< 读写 SWJ 引脚电平 / Read or drive SWJ pins.
  SWJ_CLOCK = 0x11,       ///< 设置 SWJ 时钟频率 / Set SWJ clock frequency.
  SWJ_SEQUENCE = 0x12,    ///< 发送 SWJ 原始位序列 / Send raw SWJ bit sequence.
  SWD_CONFIGURE = 0x13,   ///< 配置 SWD 协议参数 / Configure SWD protocol parameters.
  JTAG_SEQUENCE = 0x14,   ///< 发送 JTAG 位序列 / Send JTAG TMS/TDI sequence.
  JTAG_CONFIGURE = 0x15,  ///< 配置 JTAG 扫描链参数 / Configure JTAG scan chain.
  JTAG_IDCODE = 0x16,     ///< 读取 JTAG IDCODE / Read JTAG IDCODE.

  // SWO (v2)
  SWO_TRANSPORT = 0x17,  ///< 设置 SWO 传输模式 / Set SWO transport mode.
  SWO_MODE = 0x18,       ///< 设置 SWO 编码格式 / Set SWO trace mode.
  SWO_BAUDRATE = 0x19,   ///< 设置 SWO 波特率 / Set SWO baudrate.
  SWO_CONTROL = 0x1A,    ///< 启动或停止 SWO 采集 / Control SWO capture.
  SWO_STATUS = 0x1B,     ///< 查询 SWO 采集状态 / Query SWO capture status.
  SWO_DATA = 0x1C,       ///< 读取 SWO 跟踪数据 / Read captured SWO data.

  // SWD sequence (v2)
  SWD_SEQUENCE = 0x1D,  ///< 执行紧凑 SWD 双向位序列 / Execute packed SWD bit sequences.

  // Queue (0x7E-0x7F)
  QUEUE_COMMANDS = 0x7E,    ///< 入队批量 DAP 命令 / Queue batched DAP commands.
  EXECUTE_COMMANDS = 0x7F,  ///< 执行已入队命令 / Execute queued commands.

  INVALID = 0xFF  ///< 无效命令占位值 / Invalid command sentinel.
};

// ==============================
// 信息 ID / DAP_Info IDs
// ==============================

/**
 * @brief `DAP_Info` 查询信息项标识 / `DAP_Info` query item identifiers.
 */
enum class InfoId : std::uint8_t
{
  VENDOR = 0x01,            ///< 调试器厂商名 / Adapter vendor name.
  PRODUCT = 0x02,           ///< 调试器产品名 / Adapter product name.
  SERIAL_NUMBER = 0x03,     ///< 调试器序列号 / Adapter serial number.
  FIRMWARE_VERSION = 0x04,  ///< CMSIS-DAP 固件版本 / CMSIS-DAP firmware version.

  DEVICE_VENDOR = 0x05,             ///< 目标器件厂商名 / Target device vendor.
  DEVICE_NAME = 0x06,               ///< 目标器件型号 / Target device name.
  BOARD_VENDOR = 0x07,              ///< 开发板厂商名 / Target board vendor.
  BOARD_NAME = 0x08,                ///< 开发板型号 / Target board name.
  PRODUCT_FIRMWARE_VERSION = 0x09,  ///< 产品固件版本 / Product firmware version.

  CAPABILITIES = 0xF0,     ///< 调试器能力位图 / Adapter capabilities bitmask.
  TIMESTAMP_CLOCK = 0xF1,  ///< 时间戳计数器频率 / Timestamp timer frequency.
  SWO_BUFFER_SIZE = 0xFD,  ///< SWO 缓冲区大小 / SWO trace buffer size.
  PACKET_COUNT = 0xFE,     ///< 最大排队报文数 / Maximum queued packet count.
  PACKET_SIZE = 0xFF       ///< 单包最大字节数 / Maximum packet size.
};

// ==============================
// 能力位（DAP_Info: Capabilities） / Capabilities bits
// ==============================

static constexpr std::uint8_t DAP_CAP_SWD = 0x01u;   ///< 支持 SWD / Supports SWD.
static constexpr std::uint8_t DAP_CAP_JTAG = 0x02u;  ///< 支持 JTAG / Supports JTAG.
static constexpr std::uint8_t DAP_CAP_SWO = 0x04u;   ///< 支持 SWO / Supports SWO.

// ==============================
// 状态与端口 / Status / Port
// ==============================

/**
 * @brief CMSIS-DAP 命令执行状态码 / CMSIS-DAP command response status.
 */
enum class Status : std::uint8_t
{
  OK = 0x00,    ///< 命令执行成功 / Command succeeded.
  ERROR = 0xFF  ///< 命令执行失败 / Command failed.
};

/**
 * @brief `DAP_Connect` 请求的调试端口选择 / Debug port selection for `DAP_Connect`.
 */
enum class Port : std::uint8_t
{
  DEFAULT = 0x00,  ///< 使用默认端口 / Use default port.
  SWD = 0x01,      ///< 选择 SWD 端口 / Select SWD port.
  JTAG = 0x02      ///< 选择 JTAG 端口 / Select JTAG port.
};

/**
 * @brief 当前激活的调试端口状态 / Currently active debug port mode.
 */
enum class DebugPort : std::uint8_t
{
  DISABLED = 0,  ///< 未连接任何端口 / No debug port active.
  SWD = 1,       ///< SWD 模式已激活 / SWD mode active.
  JTAG = 2       ///< JTAG 模式已激活 / JTAG mode active.
};

// ==============================
// 传输请求位 / Transfer Request bits
// ==============================

static constexpr std::uint8_t DAP_TRANSFER_APNDP =
    (1u << 0);  ///< AP(1) 或 DP(0) 访问选择位 / AP (1) or DP (0) access bit.
static constexpr std::uint8_t DAP_TRANSFER_RNW =
    (1u << 1);  ///< 读(1) 或写(0) 方向位 / Read (1) or write (0) direction bit.
static constexpr std::uint8_t DAP_TRANSFER_A2 =
    (1u << 2);  ///< 寄存器地址位 A[2] / Register address bit A[2].
static constexpr std::uint8_t DAP_TRANSFER_A3 =
    (1u << 3);  ///< 寄存器地址位 A[3] / Register address bit A[3].
static constexpr std::uint8_t DAP_TRANSFER_MATCH_VALUE =
    (1u << 4);  ///< 值匹配轮询模式位 / Value-match polling mode bit.
static constexpr std::uint8_t DAP_TRANSFER_MATCH_MASK =
    (1u << 5);  ///< 写匹配掩码标记位 / Match-mask write indicator bit.
static constexpr std::uint8_t DAP_TRANSFER_TIMESTAMP =
    (1u << 7);  ///< v2 附加时间戳标记位 / v2 timestamp request bit.

// 传输响应位 / Transfer Response bits
static constexpr std::uint8_t DAP_TRANSFER_OK =
    (1u << 0);  ///< ACK: OK 应答掩码 / ACK OK mask.
static constexpr std::uint8_t DAP_TRANSFER_WAIT =
    (1u << 1);  ///< ACK: WAIT 应答掩码 / ACK WAIT mask.
static constexpr std::uint8_t DAP_TRANSFER_FAULT =
    (1u << 2);  ///< ACK: FAULT 应答掩码 / ACK FAULT mask.
static constexpr std::uint8_t DAP_TRANSFER_ERROR =
    (1u << 3);  ///< 协议或奇偶校验错误掩码 / Protocol or parity error mask.
static constexpr std::uint8_t DAP_TRANSFER_MISMATCH =
    (1u << 4);  ///< 值匹配轮询不匹配掩码 / Value-match mismatch mask.
static constexpr std::uint8_t DAP_TRANSFER_NO_TARGET =
    (1u << 7);  ///< v2 无目标响应标记 / v2 no-target response bit.

// ==============================
// SWJ 引脚位 / SWJ Pins bits
// ==============================

static constexpr std::uint8_t DAP_SWJ_SWCLK_TCK =
    (1u << 0);  ///< SWCLK / TCK 引脚位 / SWCLK / TCK pin bit.
static constexpr std::uint8_t DAP_SWJ_SWDIO_TMS =
    (1u << 1);  ///< SWDIO / TMS 引脚位 / SWDIO / TMS pin bit.
static constexpr std::uint8_t DAP_SWJ_TDI =
    (1u << 2);  ///< JTAG TDI 引脚位 / JTAG TDI pin bit.
static constexpr std::uint8_t DAP_SWJ_TDO =
    (1u << 3);  ///< JTAG TDO 引脚位 / JTAG TDO pin bit.
static constexpr std::uint8_t DAP_SWJ_NTRST =
    (1u << 5);  ///< nTRST 复位引脚位 / nTRST pin bit.
static constexpr std::uint8_t DAP_SWJ_NRESET =
    (1u << 7);  ///< nRESET 复位引脚位 / nRESET pin bit.

// ==============================
// 序列字段 / SWD / JTAG Sequence fields
// ==============================

static constexpr std::uint8_t SWD_SEQUENCE_CLK =
    0x3Fu;  ///< SWD 序列周期数字段掩码 / SWD sequence cycle-count mask.
static constexpr std::uint8_t SWD_SEQUENCE_DIN =
    (1u << 7);  ///< SWD 序列输入捕获方向位 / SWD sequence input capture bit.

static constexpr std::uint8_t JTAG_SEQUENCE_TCK =
    0x3Fu;  ///< JTAG 序列 TCK 周期数字段掩码 / JTAG sequence TCK cycle mask.
static constexpr std::uint8_t JTAG_SEQUENCE_TMS =
    (1u << 6);  ///< JTAG 序列 TMS 电平位 / JTAG sequence TMS level bit.
static constexpr std::uint8_t JTAG_SEQUENCE_TDO =
    (1u << 7);  ///< JTAG 序列 TDO 捕获使能位 / JTAG sequence TDO capture bit.

// ==============================
// 工具函数 / Helpers
// ==============================

/// 从 transfer request 字节提取 A[3:2]（2-bit）。
/// Extract A[3:2] from transfer request (2-bit).
static inline constexpr std::uint8_t req_addr2b(std::uint8_t req)
{
  return static_cast<std::uint8_t>(((req & DAP_TRANSFER_A2) ? 1u : 0u) |
                                   ((req & DAP_TRANSFER_A3) ? 2u : 0u));
}

/// 判断是否为 AP 访问 / Check if request targets AP.
static inline constexpr bool req_is_ap(std::uint8_t req)
{
  return (req & DAP_TRANSFER_APNDP) != 0u;
}

/// 判断是否为读操作 / Check if request is read (RnW=1).
static inline constexpr bool req_is_read(std::uint8_t req)
{
  return (req & DAP_TRANSFER_RNW) != 0u;
}

/// 判断是否需要 timestamp / Check if request needs timestamp.
static inline constexpr bool req_need_timestamp(std::uint8_t req)
{
  return (req & DAP_TRANSFER_TIMESTAMP) != 0u;
}

// ==============================
// 运行态结构体 / State structs
// ==============================

/**
 * @brief CMSIS-DAP Transfer 配置参数 / CMSIS-DAP transfer configuration parameters.
 */
struct TransferConfig
{
  std::uint8_t idle_cycles = 0u;     ///< 空闲时钟插入 / Idle cycles insertion.
  std::uint16_t retry_count = 100u;  ///< WAIT 重试次数 / WAIT retry count.
  std::uint16_t match_retry = 0u;    ///< MATCH 重试次数 / MATCH retry count.
  std::uint32_t match_mask = 0u;     ///< MATCH 掩码 / MATCH mask.
};

/**
 * @brief SWD 物理层时序参数配置 / SWD physical-layer protocol configuration.
 */
struct SwdConfig
{
  std::uint8_t turnaround = 1u;  ///< Turnaround 周期 / Turnaround cycles.
  bool data_phase = false;       ///< Data phase 标志 / Data phase flag.
};

/**
 * @brief DAP 协议解释器运行态 / Runtime state for the DAP protocol interpreter.
 */
struct State
{
  DebugPort debug_port = DebugPort::DISABLED;  ///< 当前调试端口 / Current debug port.
  volatile bool transfer_abort = false;        ///< 传输中止标志 / Transfer abort flag.

  TransferConfig transfer_cfg;  ///< Transfer 配置 / Transfer configuration.
  SwdConfig swd_cfg;            ///< SWD 配置 / SWD configuration.
};

/**
 * @brief 单条 DAP 请求处理的收发字节统计 / Consumed and generated byte counts for one
 *        DAP command.
 */
struct CommandResult
{
  std::uint16_t request_consumed = 0u;    ///< 已消耗请求字节数 / Request bytes consumed.
  std::uint16_t response_generated = 0u;  ///< 已生成响应字节数 / Response bytes
                                          ///< generated.
};

}  // namespace DapLinkV2Def

}  // namespace LibXR::USB
