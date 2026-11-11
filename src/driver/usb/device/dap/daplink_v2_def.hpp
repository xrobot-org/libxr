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

/// CMSIS-DAP 命令号（请求的第一个字节）/ CMSIS-DAP command IDs (first byte of a request)
enum class CommandId : std::uint8_t
{
  // Core (0x00-0x0F)
  INFO = 0x00,
  HOST_STATUS = 0x01,
  CONNECT = 0x02,
  DISCONNECT = 0x03,
  TRANSFER_CONFIGURE = 0x04,
  TRANSFER = 0x05,
  TRANSFER_BLOCK = 0x06,
  TRANSFER_ABORT = 0x07,
  WRITE_ABORT = 0x08,
  DELAY = 0x09,
  RESET_TARGET = 0x0A,

  // SWJ (0x10-0x1F)
  SWJ_PINS = 0x10,
  SWJ_CLOCK = 0x11,
  SWJ_SEQUENCE = 0x12,
  SWD_CONFIGURE = 0x13,
  JTAG_SEQUENCE = 0x14,
  JTAG_CONFIGURE = 0x15,
  JTAG_IDCODE = 0x16,

  // SWO (v2)
  SWO_TRANSPORT = 0x17,
  SWO_MODE = 0x18,
  SWO_BAUDRATE = 0x19,
  SWO_CONTROL = 0x1A,
  SWO_STATUS = 0x1B,
  SWO_DATA = 0x1C,

  // SWD sequence (v2)
  SWD_SEQUENCE = 0x1D,

  // Queue (0x7E-0x7F)
  QUEUE_COMMANDS = 0x7E,
  EXECUTE_COMMANDS = 0x7F,

  INVALID = 0xFF
};

// ==============================
// 信息 ID / DAP_Info IDs
// ==============================

/// DAP_Info 命令的查询项 / Items queried by the DAP_Info command
enum class InfoId : std::uint8_t
{
  VENDOR = 0x01,
  PRODUCT = 0x02,
  SERIAL_NUMBER = 0x03,
  FIRMWARE_VERSION = 0x04,

  DEVICE_VENDOR = 0x05,
  DEVICE_NAME = 0x06,
  BOARD_VENDOR = 0x07,
  BOARD_NAME = 0x08,
  PRODUCT_FIRMWARE_VERSION = 0x09,

  CAPABILITIES = 0xF0,
  TIMESTAMP_CLOCK = 0xF1,
  SWO_BUFFER_SIZE = 0xFD,
  PACKET_COUNT = 0xFE,
  PACKET_SIZE = 0xFF
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

/// 命令响应中的状态字节 / Status byte in command responses
enum class Status : std::uint8_t
{
  OK = 0x00,
  ERROR = 0xFF
};

/// DAP_Connect 的端口选择与返回值编码 / Port selection and result encoding of
/// DAP_Connect
enum class Port : std::uint8_t
{
  DISABLED = 0x00,
  SWD = 0x01,
  JTAG = 0x02
};

/// 当前已连接的调试端口 / Debug port that is currently connected
enum class DebugPort : std::uint8_t
{
  DISABLED = 0,
  SWD = 1,
  JTAG = 2
};

// ==============================
// 传输请求位 / Transfer Request bits
// ==============================

static constexpr std::uint8_t DAP_TRANSFER_APNDP =
    (1u << 0);  ///< 1 = AP，0 = DP / 1 = AP, 0 = DP
static constexpr std::uint8_t DAP_TRANSFER_RNW =
    (1u << 1);  ///< 1 = 读，0 = 写 / 1 = read, 0 = write
static constexpr std::uint8_t DAP_TRANSFER_A2 =
    (1u << 2);  ///< 寄存器地址位 A[2] / Register address bit A[2]
static constexpr std::uint8_t DAP_TRANSFER_A3 =
    (1u << 3);  ///< 寄存器地址位 A[3] / Register address bit A[3]
static constexpr std::uint8_t DAP_TRANSFER_MATCH_VALUE =
    (1u << 4);  ///< 写入匹配值 / Write the match value
static constexpr std::uint8_t DAP_TRANSFER_MATCH_MASK =
    (1u << 5);  ///< 写入匹配掩码 / Write the match mask
static constexpr std::uint8_t DAP_TRANSFER_TIMESTAMP = (1u << 7);  ///< v2

// 传输响应位 / Transfer Response bits
static constexpr std::uint8_t DAP_TRANSFER_OK = (1u << 0);     ///< ACK = OK
static constexpr std::uint8_t DAP_TRANSFER_WAIT = (1u << 1);   ///< ACK = WAIT
static constexpr std::uint8_t DAP_TRANSFER_FAULT = (1u << 2);  ///< ACK = FAULT
static constexpr std::uint8_t DAP_TRANSFER_ERROR =
    (1u << 3);  ///< 协议错误 / Protocol error
static constexpr std::uint8_t DAP_TRANSFER_MISMATCH =
    (1u << 4);  ///< 读值与匹配值不符 / Read value did not match
static constexpr std::uint8_t DAP_TRANSFER_NO_TARGET = (1u << 7);  ///< v2

// ==============================
// SWJ 引脚位 / SWJ Pins bits
// ==============================

static constexpr std::uint8_t DAP_SWJ_SWCLK_TCK =
    (1u << 0);  ///< SWCLK/TCK 引脚 / SWCLK/TCK pin
static constexpr std::uint8_t DAP_SWJ_SWDIO_TMS =
    (1u << 1);  ///< SWDIO/TMS 引脚 / SWDIO/TMS pin
static constexpr std::uint8_t DAP_SWJ_TDI = (1u << 2);     ///< TDI 引脚 / TDI pin
static constexpr std::uint8_t DAP_SWJ_TDO = (1u << 3);     ///< TDO 引脚 / TDO pin
static constexpr std::uint8_t DAP_SWJ_NTRST = (1u << 5);   ///< nTRST 引脚 / nTRST pin
static constexpr std::uint8_t DAP_SWJ_NRESET = (1u << 7);  ///< nRESET 引脚 / nRESET pin

// ==============================
// 序列字段 / SWD / JTAG Sequence fields
// ==============================

static constexpr std::uint8_t SWD_SEQUENCE_CLK =
    0x3Fu;  ///< SWD 序列信息字节中的时钟数字段（位 0-5，0 表示 64）/ Clock-count field of
            ///< an SWD sequence info byte (bits 0-5, 0 means 64)
static constexpr std::uint8_t SWD_SEQUENCE_DIN =
    (1u << 7);  ///< 置位表示采样 SWDIO（输入）/ Set to sample SWDIO (input)

static constexpr std::uint8_t JTAG_SEQUENCE_TCK =
    0x3Fu;  ///< JTAG 序列信息字节中的 TCK 数字段（位 0-5，0 表示 64）/ TCK-count field of
            ///< a JTAG sequence info byte (bits 0-5, 0 means 64)
static constexpr std::uint8_t JTAG_SEQUENCE_TMS =
    (1u << 6);  ///< 序列期间 TMS 的电平 / TMS level during the sequence
static constexpr std::uint8_t JTAG_SEQUENCE_TDO =
    (1u << 7);  ///< 置位表示捕获 TDO / Set to capture TDO

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

/// DAP_TransferConfigure 设定的传输参数 / Transfer parameters set by
/// DAP_TransferConfigure
struct TransferConfig
{
  std::uint8_t idle_cycles = 0u;     ///< 空闲时钟插入 / Idle cycles insertion.
  std::uint16_t retry_count = 100u;  ///< WAIT 重试次数 / WAIT retry count.
  std::uint16_t match_retry = 0u;    ///< MATCH 重试次数 / MATCH retry count.
  std::uint32_t match_mask = 0u;     ///< MATCH 掩码 / MATCH mask.
};

/// DAP_SWD_Configure 设定的 SWD 参数 / SWD parameters set by DAP_SWD_Configure
struct SwdConfig
{
  std::uint8_t turnaround = 1u;  ///< Turnaround 周期 / Turnaround cycles.
  bool data_phase = false;       ///< Data phase 标志 / Data phase flag.
};

/// DAP 会话状态 / DAP session state
struct State
{
  DebugPort debug_port = DebugPort::DISABLED;  ///< 当前调试端口 / Current debug port.
  volatile bool transfer_abort = false;        ///< 传输中止标志 / Transfer abort flag.

  TransferConfig transfer_cfg;  ///< Transfer 配置 / Transfer configuration.
  SwdConfig swd_cfg;            ///< SWD 配置 / SWD configuration.
};

/// 一条命令处理后消耗与生成的字节数 / Bytes consumed and generated by one processed
/// command
struct CommandResult
{
  std::uint16_t request_consumed = 0u;    ///< 已消耗请求字节数 / Request bytes consumed.
  std::uint16_t response_generated = 0u;  ///< 已生成响应字节数 / Response bytes
                                          ///< generated.
};

}  // namespace DapLinkV2Def

}  // namespace LibXR::USB
