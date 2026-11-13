// jtag_protocol.hpp
#pragma once

#include <cstdint>

namespace LibXR::Debug::JtagProtocol
{
/**
 * @brief JTAG 传输端口选择 / JTAG transfer port selector
 */
enum class Port : uint8_t
{
  DP = 0,  ///< 调试端口 / Debug Port
  AP = 1,  ///< 访问端口 / Access Port
};

/**
 * @brief JTAG ACK 返回码 / JTAG ACK response codes
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
 * @brief JTAG 传输请求 / JTAG transfer request
 *
 * @note 请求的位域与 SWD 相同：APnDP、RnW、A2、A3。JTAG 中由 DPACC 或 APACC 指令选择
 *       端口。
 *       The request fields are those of SWD: APnDP, RnW, A2, A3. In JTAG the DPACC or
 *       APACC instruction selects the port.
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
 * @brief JTAG 传输响应 / JTAG transfer response
 */
struct Response
{
  Ack ack = Ack::PROTOCOL;  ///< ACK
  uint32_t rdata = 0;  ///< 读数据（仅读响应有效）/ Read data (valid for read responses)
};

/**
 * @brief JTAG IR 指令码 / JTAG IR codes
 */
inline constexpr uint32_t JTAG_IR_ABORT = 0x08u;   ///< ABORT
inline constexpr uint32_t JTAG_IR_DPACC = 0x0Au;   ///< DPACC：访问 DP / DP access
inline constexpr uint32_t JTAG_IR_APACC = 0x0Bu;   ///< APACC：访问 AP / AP access
inline constexpr uint32_t JTAG_IR_IDCODE = 0x0Eu;  ///< IDCODE
inline constexpr uint32_t JTAG_IR_BYPASS = 0x0Fu;  ///< BYPASS

/**
 * @brief JTAG Sequence 信息字段 / JTAG sequence info fields
 */
inline constexpr uint8_t JTAG_SEQUENCE_TCK = 0x3Fu;  ///< TCK 个数 / TCK count (0: 64).
inline constexpr uint8_t JTAG_SEQUENCE_TMS = 0x40u;  ///< TMS 电平 / TMS value.
inline constexpr uint8_t JTAG_SEQUENCE_TDO = 0x80u;  ///< 采集 TDO / Capture TDO.

/**
 * @brief JTAG-DP DR 长度（bit）/ JTAG-DP DR length in bits
 */
inline constexpr uint32_t JTAG_DP_DR_LEN = 35u;

/**
 * @brief 把 JTAG-DP 的 3 位原始 ACK 映射成 Ack / Map the raw 3-bit ACK of a JTAG-DP to
 * Ack
 *
 * JTAG-DP 的编码与 SWD 不同：原始值 OK = 0x2、WAIT = 0x1、FAULT = 0x4；全 0 视为无应答，
 * 其余组合都是协议错误。
 * The encoding of a JTAG-DP differs from SWD: raw OK = 0x2, WAIT = 0x1, FAULT = 0x4; all
 * zero is taken as no answer, every other pattern is a protocol error.
 *
 * @param raw_ack 原始 ACK，只看低 3 位 / Raw ACK; only the low 3 bits are used
 * @return 映射后的 ACK / Mapped ACK
 */
constexpr Ack MapJtagAck(uint8_t raw_ack)
{
  switch (raw_ack & 0x7u)
  {
    case 0x2:
      return Ack::OK;
    case 0x1:
      return Ack::WAIT;
    case 0x4:
      return Ack::FAULT;
    case 0x0:
      return Ack::NO_ACK;
    default:
      return Ack::PROTOCOL;
  }
}

/**
 * @brief JTAG 设备链配置 / JTAG device chain configuration
 *
 * @note 该结构只描述设备链的参数，不分配内存。
 *       The structure only describes the chain; it allocates nothing.
 */
struct ChainConfig
{
  uint8_t count = 0;  ///< 设备数（TDO 端为 index=0）/ Device count (TDO side index=0)
  uint8_t index = 0;  ///< 当前设备索引 / Current device index
  const uint8_t* ir_length = nullptr;   ///< 各设备 IR 长度数组 / IR length array
  const uint16_t* ir_before = nullptr;  ///< 当前设备前的 bypass 位数 / Bypass bits before
  const uint16_t* ir_after = nullptr;   ///< 当前设备后的 bypass 位数 / Bypass bits after

  // 可选的运行期缓存，由 UpdateChainCache() 填写，省去每次移位时的计算。
  // Optional run-time cache filled by UpdateChainCache(); saves the computation on every
  // shift.
  uint32_t ir_before_bits_len = 0;  ///< 前面设备 IR 总位数 / IR bits before
  uint32_t ir_after_bits_len = 0;   ///< 后面设备 IR 总位数 / IR bits after
  uint32_t dr_before_bits_len = 0;  ///< 前面设备 DR 总位数 / DR bits before
  uint32_t dr_after_bits_len = 0;   ///< 后面设备 DR 总位数 / DR bits after
};

/**
 * @brief 更新链路缓存（可选） / Update chain cache (optional).
 *
 * @note index = 0 是最靠近 TDO 的设备；缓存只用于加速，可以不用。
 *       Index 0 is the device nearest to TDO; the cache only saves time and is optional.
 *
 * @param cfg 设备链配置，其中的缓存字段被重写 / Chain configuration; its cache fields
 *        are rewritten
 */
inline void UpdateChainCache(ChainConfig& cfg)
{
  cfg.ir_before_bits_len = 0;
  cfg.ir_after_bits_len = 0;
  cfg.dr_before_bits_len = 0;
  cfg.dr_after_bits_len = 0;

  if (cfg.count == 0 || cfg.index >= cfg.count)
  {
    return;
  }

  if (cfg.ir_length != nullptr)
  {
    for (uint8_t i = 0; i < cfg.index; ++i)
    {
      cfg.ir_before_bits_len += cfg.ir_length[i];
    }
    for (uint8_t i = static_cast<uint8_t>(cfg.index + 1u); i < cfg.count; ++i)
    {
      cfg.ir_after_bits_len += cfg.ir_length[i];
    }
  }

  cfg.dr_before_bits_len = cfg.index;
  cfg.dr_after_bits_len = static_cast<uint32_t>(cfg.count - cfg.index - 1u);
}

/**
 * @brief 把 Request 组装成 35 位 DR，LSB 在先 / Pack a Request into the 35-bit DR, LSB
 *        first
 *
 * 布局 / Layout：
 *   bit0: RnW
 *   bit1: A2
 *   bit2: A3
 *   bit3..34: DATA[31:0]
 *
 * @param req 请求（用 rnw、addr2b、wdata） / Request (rnw, addr2b and wdata are used)
 * @return 35 位 DR 值 / The 35-bit DR value
 */
inline uint64_t PackDpDr(const Request& req)
{
  const uint64_t RNW = req.rnw ? 1u : 0u;
  const uint64_t A2 = static_cast<uint64_t>(req.addr2b & 0x1u);
  const uint64_t A3 = static_cast<uint64_t>((req.addr2b >> 1) & 0x1u);
  return (static_cast<uint64_t>(req.wdata) << 3) | (A3 << 2) | (A2 << 1) | RNW;
}

/**
 * @brief 把移出的 35 位 DR 解析成 Response / Unpack the 35-bit DR shifted out into a
 *        Response
 *
 * 低 3 位是原始 ACK，经 MapJtagAck() 映射；其后 32 位是读数据。
 * The low 3 bits are the raw ACK, mapped by MapJtagAck(); the next 32 bits are the read
 * data.
 *
 * @param dr_lsb_first 从 TDO 移出的 DR，LSB 在先 / DR shifted out of TDO, LSB first
 * @param resp 解析结果 / Unpacked response
 */
inline void UnpackDpDr(uint64_t dr_lsb_first, Response& resp)
{
  const uint8_t RAW_ACK = static_cast<uint8_t>(dr_lsb_first & 0x7u);
  resp.ack = MapJtagAck(RAW_ACK);
  resp.rdata = static_cast<uint32_t>((dr_lsb_first >> 3) & 0xFFFF'FFFFu);
}

/**
 * @brief 构造 DP 请求 / Build DP request
 *
 * @param rnw true 为读，false 为写 / true for a read, false for a write
 * @param addr2b A[3:2]（0..3）/ A[3:2] (0..3)
 * @param wdata 写数据，读请求不用 / Write data; unused by a read
 * @return 请求 / Request
 */
constexpr Request MakeDpRequest(bool rnw, uint8_t addr2b, uint32_t wdata = 0u)
{
  return Request{Port::DP, rnw, static_cast<uint8_t>(addr2b & 0x03u), wdata};
}

/**
 * @brief 构造 AP 请求 / Build AP request
 *
 * @param rnw true 为读，false 为写 / true for a read, false for a write
 * @param addr2b A[3:2]（0..3）/ A[3:2] (0..3)
 * @param wdata 写数据，读请求不用 / Write data; unused by a read
 * @return 请求 / Request
 */
constexpr Request MakeApRequest(bool rnw, uint8_t addr2b, uint32_t wdata = 0u)
{
  return Request{Port::AP, rnw, static_cast<uint8_t>(addr2b & 0x03u), wdata};
}

}  // namespace LibXR::Debug::JtagProtocol
