// swd_protocol.hpp
#pragma once

#include <cstdint>

namespace LibXR::Debug::SwdProtocol
{
/**
 * @brief SWD 传输端口选择 / SWD transfer port selector
 */
enum class Port : uint8_t
{
  DP = 0,  ///< 调试端口 / Debug Port
  AP = 1,  ///< 访问端口 / Access Port
};

/**
 * @brief SWD 引脚枚举 / SWD pin selector
 */
enum class Pin : uint8_t
{
  SWCLK = 0,  ///< SWCLK
  SWDIO = 1,  ///< SWDIO
};

/**
 * @brief SWD ACK 返回码 / SWD ACK response codes
 *
 * @note ACK 为 3-bit（LSB-first）编码；此处以解码后的枚举值表示。
 *       ACK is a 3-bit (LSB-first) encoding; this enum represents decoded values.
 */
enum class Ack : uint8_t
{
  NO_ACK = 0x0,    ///< 无应答 / No ACK
  OK = 0x1,        ///< OK
  WAIT = 0x2,      ///< WAIT
  FAULT = 0x4,     ///< FAULT
  PROTOCOL = 0x7,  ///< 协议错误（非法 ACK）/ Protocol error (invalid ACK)
};

/**
 * @brief SWD 传输请求 / SWD transfer request
 */
struct Request
{
  Port port = Port::DP;  ///< 目标端口（DP/AP）/ Target port (DP/AP)
  bool rnw =
      true;  ///< 读写标志：true=读，false=写 / Read-not-write: true=read, false=write
  uint8_t addr2b = 0;  ///< A[3:2] 两位地址编码（0..3）/ A[3:2] encoded as 0..3
  uint32_t wdata = 0;  ///< 写数据（仅写请求有效）/ Write data (valid for write requests)
};

/**
 * @brief SWD 传输响应 / SWD transfer response
 */
struct Response
{
  Ack ack = Ack::PROTOCOL;  ///< ACK
  uint32_t rdata = 0;  ///< 读数据（仅读响应有效）/ Read data (valid for read responses)
  bool parity_ok = true;  ///< 奇偶校验是否正确 / Whether parity is OK
};

/**
 * @brief DP 读寄存器选择（A[3:2]）/ DP read register selector (A[3:2])
 */
enum class DpReadReg : uint8_t
{
  IDCODE = 0,     ///< IDCODE
  CTRL_STAT = 1,  ///< CTRL/STAT
  SELECT = 2,     ///< SELECT
  RDBUFF = 3,     ///< RDBUFF
};

/**
 * @brief DP 写寄存器选择（A[3:2]）/ DP write register selector (A[3:2])
 */
enum class DpWriteReg : uint8_t
{
  ABORT = 0,      ///< ABORT
  CTRL_STAT = 1,  ///< CTRL/STAT
  SELECT = 2,     ///< SELECT
};

/**
 * @brief DP ABORT 寄存器位定义 / DP ABORT register bit definitions
 *
 * @note C++17 起建议用 inline constexpr，避免头文件多重定义问题。
 *       Prefer inline constexpr in C++17+ to avoid multiple definition in headers.
 */
inline constexpr uint32_t DP_ABORT_DAPABORT = (1u << 0);    ///< DAPABORT
inline constexpr uint32_t DP_ABORT_STKCMPCLR = (1u << 1);   ///< STKCMPCLR
inline constexpr uint32_t DP_ABORT_STKERRCLR = (1u << 2);   ///< STKERRCLR
inline constexpr uint32_t DP_ABORT_WDERRCLR = (1u << 3);    ///< WDERRCLR
inline constexpr uint32_t DP_ABORT_ORUNERRCLR = (1u << 4);  ///< ORUNERRCLR

/**
 * @brief DP CTRL/STAT 寄存器位定义 / DP CTRL/STAT register bit definitions
 */
inline constexpr uint32_t DP_CTRLSTAT_CDBGPWRUPREQ = (1u << 28);  ///< CDBGPWRUPREQ
inline constexpr uint32_t DP_CTRLSTAT_CDBGPWRUPACK = (1u << 29);  ///< CDBGPWRUPACK
inline constexpr uint32_t DP_CTRLSTAT_CSYSPWRUPREQ = (1u << 30);  ///< CSYSPWRUPREQ
inline constexpr uint32_t DP_CTRLSTAT_CSYSPWRUPACK = (1u << 31);  ///< CSYSPWRUPACK

/**
 * @brief 构造 SELECT 寄存器值 / Build SELECT register value
 *
 * @param apsel     APSEL（AP 选择）/ APSEL (AP selection)
 * @param apbanksel APBANKSEL（AP bank 选择）/ APBANKSEL (AP bank selection)
 * @param dpbanksel DPBANKSEL（DP bank 选择，默认 0）/ DPBANKSEL (DP bank selection,
 * default 0)
 * @return SELECT 寄存器值 / SELECT register value
 */
constexpr uint32_t make_select(uint8_t apsel, uint8_t apbanksel, uint8_t dpbanksel = 0)
{
  return (static_cast<uint32_t>(apsel) << 24) |
         ((static_cast<uint32_t>(apbanksel) & 0x0Fu) << 4) |
         (static_cast<uint32_t>(dpbanksel) & 0x0Fu);
}

/**
 * @brief 构造 DP 读请求 / Build DP read request
 *
 * @param reg DP 读寄存器 / DP read register
 * @return Request 请求结构 / Request
 */
constexpr Request make_dp_read_req(DpReadReg reg)
{
  return Request{Port::DP, true, static_cast<uint8_t>(reg), 0u};
}

/**
 * @brief 构造 DP 写请求 / Build DP write request
 *
 * @param reg   DP 写寄存器 / DP write register
 * @param wdata 写数据 / Write data
 * @return Request 请求结构 / Request
 */
constexpr Request make_dp_write_req(DpWriteReg reg, uint32_t wdata)
{
  return Request{Port::DP, false, static_cast<uint8_t>(reg), wdata};
}

/**
 * @brief 构造 AP 读请求 / Build AP read request
 *
 * @param addr2b A[3:2]（0..3）/ A[3:2] (0..3)
 * @return Request 请求结构 / Request
 */
constexpr Request make_ap_read_req(uint8_t addr2b)
{
  return Request{Port::AP, true, static_cast<uint8_t>(addr2b & 0x03u), 0u};
}

/**
 * @brief 构造 AP 写请求 / Build AP write request
 *
 * @param addr2b A[3:2]（0..3）/ A[3:2] (0..3)
 * @param wdata  写数据 / Write data
 * @return Request 请求结构 / Request
 */
constexpr Request make_ap_write_req(uint8_t addr2b, uint32_t wdata)
{
  return Request{Port::AP, false, static_cast<uint8_t>(addr2b & 0x03u), wdata};
}

}  // namespace LibXR::Debug::SwdProtocol
