#pragma once

#include <cstddef>
#include <cstdint>

namespace LibXR::USB::GsUsb
{
// =================== CAN ID 标志（与 Linux can.h 对齐） ===================

/**
 * @brief 扩展帧标志（29-bit） / Extended frame flag (29-bit)
 */
constexpr uint32_t CAN_EFF_FLAG = 0x80000000U;

/**
 * @brief RTR 标志（远程帧） / RTR flag (remote frame)
 */
constexpr uint32_t CAN_RTR_FLAG = 0x40000000U;

/**
 * @brief 错误帧标志 / Error frame flag
 */
constexpr uint32_t CAN_ERR_FLAG = 0x20000000U;

/**
 * @brief 标准帧 ID 掩码（11-bit） / Standard frame ID mask (11-bit)
 */
constexpr uint32_t CAN_SFF_MASK = 0x000007FFU;

/**
 * @brief 扩展帧 ID 掩码（29-bit） / Extended frame ID mask (29-bit)
 */
constexpr uint32_t CAN_EFF_MASK = 0x1FFFFFFFU;

/**
 * @brief 错误帧 DLC（固定 8） / Error frame DLC (fixed 8)
 */
constexpr uint8_t CAN_ERR_DLC = 8;

// =================== CAN 错误码定义 ===================

/** @brief 发送超时 / Transmit timeout */
constexpr uint32_t CAN_ERR_TX_TIMEOUT = 0x00000001U;
/** @brief 仲裁丢失 / Arbitration lost */
constexpr uint32_t CAN_ERR_LOSTARB = 0x00000002U;
/** @brief 控制器状态错误 / Controller status error */
constexpr uint32_t CAN_ERR_CRTL = 0x00000004U;
/** @brief 协议错误 / Protocol error */
constexpr uint32_t CAN_ERR_PROT = 0x00000008U;
/** @brief 收发器错误 / Transceiver error */
constexpr uint32_t CAN_ERR_TRX = 0x00000010U;
/** @brief ACK 错误 / ACK error */
constexpr uint32_t CAN_ERR_ACK = 0x00000020U;
/** @brief 总线关闭 / Bus-off */
constexpr uint32_t CAN_ERR_BUSOFF = 0x00000040U;
/** @brief 总线错误 / Bus error */
constexpr uint32_t CAN_ERR_BUSERROR = 0x00000080U;
/** @brief 控制器重启 / Controller restarted */
constexpr uint32_t CAN_ERR_RESTARTED = 0x00000100U;

// =================== 错误 ID 定义 ===================

/** @brief 仲裁丢失位置未知 / Arbitration lost location unspecified */
constexpr uint8_t CAN_ERR_LOSTARB_UNSPEC = 0x00;
/** @brief 控制器错误细节未知 / Controller error detail unspecified */
constexpr uint8_t CAN_ERR_CRTL_UNSPEC = 0x00;
/** @brief 协议错误细节未知 / Protocol error detail unspecified */
constexpr uint8_t CAN_ERR_PROT_UNSPEC = 0x00;
/** @brief 协议错误位置未知 / Protocol error location unspecified */
constexpr uint8_t CAN_ERR_PROT_LOC_UNSPEC = 0x00;
/** @brief 收发器错误细节未知 / Transceiver error detail unspecified */
constexpr uint8_t CAN_ERR_TRX_UNSPEC = 0x00;

// =================== BREQ（控制请求号，与 gs_usb proto 对齐） ===================

/**
 * @brief 控制请求号（与 gs_usb proto 对齐） / Control request number (aligned with gs_usb
 * proto)
 */
enum class BReq : uint8_t
{
  HOST_FORMAT = 0,  ///< 主机字节序协商 / Host byte order negotiation
  BITTIMING,        ///< 设置仲裁相位比特定时 / Set arbitration bit timing
  MODE,             ///< 设置模式与标志 / Set mode and flags
  BERR,             ///< 错误报告相关 / Bus error reporting
  BT_CONST,         ///< 获取比特定时常量 / Get bit timing constants
  DEVICE_CONFIG,    ///< 获取设备配置 / Get device configuration
  TIMESTAMP,        ///< 时间戳相关 / Timestamp control
  IDENTIFY,         ///< 识别指示 / Identify indication
  GET_USER_ID,      ///< 获取用户 ID / Get user ID
  SET_USER_ID,      ///< 设置用户 ID / Set user ID
  DATA_BITTIMING,   ///< 设置数据相位定时（FD） / Set data-phase timing (FD)
  BT_CONST_EXT,     ///< 获取扩展 BT 常量 / Get extended BT constants
  SET_TERMINATION,  ///< 设置终端电阻 / Set termination
  GET_TERMINATION,  ///< 获取终端电阻 / Get termination
  GET_STATE,        ///< 获取状态/错误计数 / Get state / error counters
};

// =================== CAN 模式 / 状态 ===================

/**
 * @brief CAN 通道模式 / CAN channel mode
 */
enum class CanMode : uint32_t
{
  RESET = 0,  ///< 复位/停止 / Reset/stop
  START = 1,  ///< 启动 / Start
};

/**
 * @brief CAN 控制器状态 / CAN controller state
 */
enum class CanState : uint32_t
{
  ERROR_ACTIVE = 0,  ///< 错误主动 / Error-active
  ERROR_WARNING,     ///< 错误告警 / Error-warning
  ERROR_PASSIVE,     ///< 错误被动 / Error-passive
  BUS_OFF,           ///< 总线关闭 / Bus-off
  STOPPED,           ///< 已停止 / Stopped
  SLEEPING           ///< 休眠中 / Sleeping
};

/**
 * @brief 识别模式 / Identify mode
 */
enum class IdentifyMode : uint32_t
{
  OFF = 0,  ///< 关闭 / Off
  ON = 1,   ///< 打开 / On
};

/**
 * @brief 终端电阻状态 / Termination state
 */
enum class TerminationState : uint32_t
{
  OFF = 0,  ///< 关闭 / Off
  ON = 1,   ///< 打开 / On
};

// =================== 控制传输结构体（packed 对齐原协议） ===================

LIBXR_PACKED_BEGIN

/**
 * @brief 主机配置（字节序协商） / Host configuration (byte order negotiation)
 */
struct HostConfig
{
  uint32_t byte_order;  ///< host 写 0x0000beef（小端） / Host writes 0x0000beef
                        ///< (little-endian)
};

/**
 * @brief 设备配置（per-device） / Device configuration (per-device)
 */
struct DeviceConfig
{
  uint8_t reserved1;    ///< 保留 / Reserved
  uint8_t reserved2;    ///< 保留 / Reserved
  uint8_t reserved3;    ///< 保留 / Reserved
  uint8_t icount;       ///< CAN 通道数 - 1 / CAN channel count minus 1
  uint32_t sw_version;  ///< 软件版本 / Software version
  uint32_t hw_version;  ///< 硬件版本 / Hardware version
};

/**
 * @brief 通道模式设置（per-channel） / Channel mode configuration (per-channel)
 */
struct DeviceMode
{
  uint32_t mode;   ///< CanMode
  uint32_t flags;  ///< GS_CAN_MODE_*
};

/**
 * @brief 通道状态（per-channel） / Channel state (per-channel)
 */
struct DeviceState
{
  uint32_t state;  ///< CanState
  uint32_t rxerr;  ///< 接收错误计数 / Receive error counter
  uint32_t txerr;  ///< 发送错误计数 / Transmit error counter
};

/**
 * @brief 比特定时参数 / Bit timing parameters
 */
struct DeviceBitTiming
{
  uint32_t prop_seg;    ///< PROP_SEG
  uint32_t phase_seg1;  ///< PHASE_SEG1
  uint32_t phase_seg2;  ///< PHASE_SEG2
  uint32_t sjw;         ///< SJW
  uint32_t brp;         ///< BRP
};

/**
 * @brief 比特定时常量范围 / Bit timing constant ranges
 */
struct CanBitTimingConst
{
  uint32_t tseg1_min;  ///< TSEG1 最小 / Min TSEG1
  uint32_t tseg1_max;  ///< TSEG1 最大 / Max TSEG1
  uint32_t tseg2_min;  ///< TSEG2 最小 / Min TSEG2
  uint32_t tseg2_max;  ///< TSEG2 最大 / Max TSEG2
  uint32_t sjw_max;    ///< SJW 最大 / Max SJW
  uint32_t brp_min;    ///< BRP 最小 / Min BRP
  uint32_t brp_max;    ///< BRP 最大 / Max BRP
  uint32_t brp_inc;    ///< BRP 步进 / BRP increment
};

/**
 * @brief 设备比特定时能力（经典/仲裁） / Device bit timing capabilities
 * (classic/arbitration)
 */
struct DeviceBTConst
{
  uint32_t feature;       ///< CAN_FEAT_*
  uint32_t fclk_can;      ///< CAN 时钟 / CAN clock
  CanBitTimingConst btc;  ///< 定时常量 / Timing constants
};

/**
 * @brief 扩展比特定时能力（含 FD 数据相位） / Extended timing capabilities (with FD data
 * phase)
 */
struct DeviceBTConstExtended
{
  uint32_t feature;        ///< CAN_FEAT_*（含 FD） / CAN_FEAT_* (with FD)
  uint32_t fclk_can;       ///< CAN 时钟 / CAN clock
  CanBitTimingConst btc;   ///< 仲裁相位 / Arbitration phase
  CanBitTimingConst dbtc;  ///< 数据相位 / Data phase
};

/**
 * @brief 识别控制 / Identify control
 */
struct Identify
{
  uint32_t mode;  ///< IdentifyMode
};

/**
 * @brief 终端电阻控制 / Termination control
 */
struct DeviceTerminationState
{
  uint32_t state;  ///< TerminationState
};

/**
 * @brief USB HostFrame 最大布局结构 / USB HostFrame maximum-layout structure
 */
struct HostFrame
{
  uint32_t echo_id;  ///< 回显 ID / Echo ID
  uint32_t can_id;   ///< CAN ID（含 CAN_*_FLAG） / CAN ID (with CAN_*_FLAG)

  uint8_t can_dlc;   ///< DLC
  uint8_t channel;   ///< 通道号 / Channel index
  uint8_t flags;     ///< 帧标志（GS_CAN_FLAG_*） / Frame flags (GS_CAN_FLAG_*)
  uint8_t reserved;  ///< 保留 / Reserved

  uint8_t data[64];  ///< 数据（classic 用前 8 字节） / Data (classic uses first 8 bytes)
  uint32_t timestamp_us;  ///< 时间戳（可选） / Timestamp (optional)
};

LIBXR_PACKED_END

// =================== MODE 标志位（GS_CAN_MODE_*） ===================

/** @brief 普通模式 / Normal mode */
constexpr uint32_t GSCAN_MODE_NORMAL = 0;
/** @brief 只听模式 / Listen-only mode */
constexpr uint32_t GSCAN_MODE_LISTEN_ONLY = (1u << 0);
/** @brief 回环模式 / Loopback mode */
constexpr uint32_t GSCAN_MODE_LOOP_BACK = (1u << 1);
/** @brief 三采样 / Triple sampling */
constexpr uint32_t GSCAN_MODE_TRIPLE_SAMPLE = (1u << 2);
/** @brief One-shot（不重发） / One-shot (no retransmit) */
constexpr uint32_t GSCAN_MODE_ONE_SHOT = (1u << 3);
/** @brief 硬件时间戳 / Hardware timestamping */
constexpr uint32_t GSCAN_MODE_HW_TIMESTAMP = (1u << 4);
/** @brief 填充到最大包长度 / Pad packets to max packet size */
constexpr uint32_t GSCAN_MODE_PAD_PKTS_TO_MAX_PKT_SIZE = (1u << 7);
/** @brief CAN FD */
constexpr uint32_t GSCAN_MODE_FD = (1u << 8);
/** @brief 错误报告 / Bus error reporting */
constexpr uint32_t GSCAN_MODE_BERR_REPORTING = (1u << 12);

// =================== 功能位（GS_CAN_FEATURE_*） ===================

/** @brief 支持只听 / Supports listen-only */
constexpr uint32_t CAN_FEAT_LISTEN_ONLY = (1u << 0);
/** @brief 支持回环 / Supports loopback */
constexpr uint32_t CAN_FEAT_LOOP_BACK = (1u << 1);
/** @brief 支持三采样 / Supports triple sampling */
constexpr uint32_t CAN_FEAT_TRIPLE_SAMPLE = (1u << 2);
/** @brief 支持 one-shot / Supports one-shot */
constexpr uint32_t CAN_FEAT_ONE_SHOT = (1u << 3);
/** @brief 支持硬件时间戳 / Supports hardware timestamp */
constexpr uint32_t CAN_FEAT_HW_TIMESTAMP = (1u << 4);
/** @brief 支持识别指示 / Supports identify */
constexpr uint32_t CAN_FEAT_IDENTIFY = (1u << 5);
/** @brief 支持用户 ID / Supports user ID */
constexpr uint32_t CAN_FEAT_USER_ID = (1u << 6);
/** @brief 支持包填充 / Supports padding */
constexpr uint32_t CAN_FEAT_PAD_PKTS_TO_MAX_PKT_SIZE = (1u << 7);
/** @brief 支持 CAN FD / Supports CAN FD */
constexpr uint32_t CAN_FEAT_FD = (1u << 8);
/** @brief 需要 USB quirk / Requires USB quirk */
constexpr uint32_t CAN_FEAT_REQ_USB_QUIRK_LPC546XX = (1u << 9);
/** @brief 支持扩展 BT 常量 / Supports extended BT constants */
constexpr uint32_t CAN_FEAT_BT_CONST_EXT = (1u << 10);
/** @brief 支持终端电阻 / Supports termination */
constexpr uint32_t CAN_FEAT_TERMINATION = (1u << 11);
/** @brief 支持错误报告 / Supports bus error reporting */
constexpr uint32_t CAN_FEAT_BERR_REPORTING = (1u << 12);
/** @brief 支持获取状态 / Supports get state */
constexpr uint32_t CAN_FEAT_GET_STATE = (1u << 13);

// =================== 帧标志（GS_CAN_FLAG_*） ===================

/** @brief 溢出标志 / Overflow flag */
constexpr uint8_t CAN_FLAG_OVERFLOW = (1u << 0);
/** @brief FD 帧标志 / FD frame flag */
constexpr uint8_t CAN_FLAG_FD = (1u << 1);
/** @brief BRS 标志 / BRS flag */
constexpr uint8_t CAN_FLAG_BRS = (1u << 2);
/** @brief ESI 标志 / ESI flag */
constexpr uint8_t CAN_FLAG_ESI = (1u << 3);

/** @brief 无效 echo_id / Invalid echo_id */
constexpr uint32_t ECHO_ID_INVALID = 0xFFFFFFFFu;

/** @brief classic：header + 8 data / Classic: header + 8 data */
constexpr std::size_t HOST_FRAME_CLASSIC_SIZE =
    sizeof(uint32_t) + sizeof(uint32_t) + 4 + 8;
/** @brief classic + timestamp / Classic + timestamp */
constexpr std::size_t HOST_FRAME_CLASSIC_TS_SIZE =
    HOST_FRAME_CLASSIC_SIZE + sizeof(uint32_t);

/** @brief FD：header + 64 data / FD: header + 64 data */
constexpr std::size_t HOST_FRAME_FD_SIZE = sizeof(uint32_t) + sizeof(uint32_t) + 4 + 64;
/** @brief FD + timestamp */
constexpr std::size_t HOST_FRAME_FD_TS_SIZE = HOST_FRAME_FD_SIZE + sizeof(uint32_t);

}  // namespace LibXR::USB::GsUsb
