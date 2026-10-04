#pragma once

#include <cstdint>

#include "libxr_def.hpp"
#include "swd_protocol.hpp"

namespace LibXR::Debug
{
/**
 * @class Swd
 * @brief SWD 探针抽象基类，提供链路控制、传输与 DP/AP 辅助接口。
 *        Abstract SWD probe base class providing link control, transfer, and DP/AP
 * helpers.
 */
class Swd
{
 public:
  /**
   * @struct TransferPolicy
   * @brief 传输策略（WAIT 重试与空闲周期插入）。
   * Transfer policy (WAIT retry & idle insertion).
   *
   * - idle_cycles：每次传输尝试后插入（包括 WAIT 重试） / idle_cycles: inserted after
   * EACH transfer attempt, including WAIT retries.
   * - wait_retry：最大 WAIT 重试次数 / wait_retry: maximum WAIT retries.
   * - clear_sticky_on_fault：当 ACK==FAULT 时清除 sticky 错误 / clear_sticky_on_fault:
   * clear sticky errors when ACK==FAULT.
   */
  struct TransferPolicy
  {
    uint8_t idle_cycles = 0;    ///< 空闲周期数 / Idle cycles.
    uint16_t wait_retry = 100;  ///< WAIT 最大重试次数 / Maximum WAIT retries.
    bool clear_sticky_on_fault =
        true;  ///< FAULT 时清除 sticky 错误 / Clear sticky errors on FAULT.
  };

  /**
   * @brief 虚析构函数 / Virtual destructor.
   */
  virtual ~Swd() = default;

  Swd(const Swd&) = delete;
  Swd& operator=(const Swd&) = delete;

  /**
   * @brief 设置传输策略 / Set transfer policy.
   * @param policy 传输策略 / Transfer policy.
   */
  void SetTransferPolicy(const TransferPolicy& policy) { policy_ = policy; }

  /**
   * @brief 获取传输策略 / Get transfer policy.
   */
  [[nodiscard]] const TransferPolicy& GetTransferPolicy() const { return policy_; }

  /**
   * @brief 设置 SWCLK 频率（可选） / Set SWCLK frequency (optional).
   * @param hz 目标频率（Hz） / Target frequency in Hz.
   * @return ErrorCode 操作结果 / Error code.
   */
  virtual ErrorCode SetClockHz(uint32_t hz) = 0;

  /**
   * @brief 关闭探针并释放资源 / Close probe and release resources.
   */
  virtual void Close() = 0;

  /**
   * @brief 执行 SWD 线复位 / Perform SWD line reset.
   * @return ErrorCode 操作结果 / Error code.
   */
  virtual ErrorCode LineReset() = 0;

  /**
   * @brief 进入 SWD 模式（如需从 JTAG 切换）。
   * Enter SWD mode (e.g., switch from JTAG if needed).
   * @return ErrorCode 操作结果 / Error code.
   */
  virtual ErrorCode EnterSwd() = 0;

  /**
   * @brief 执行一次 SWD 传输（不含重试） / Perform one SWD transfer (no retry).
   * @param req 请求包 / Request.
   * @param resp 响应包 / Response.
   * @return ErrorCode 总线级错误码 / Bus-level error code.
   */
  virtual ErrorCode Transfer(const SwdProtocol::Request& req,
                             SwdProtocol::Response& resp) = 0;

  // --------------------------------------------------------------------------
  // Retry wrapper
  // --------------------------------------------------------------------------

  /**
   * @brief 带重试的 SWD 传输封装（WAIT 重试 + IdleCycles 插入）。
   *        SWD transfer wrapper with retry (WAIT retry + IdleCycles insertion).
   *
   * 规则 / Rules:
   * - 每次传输尝试后均插入 idle_cycles（包括 WAIT 重试）。
   * Insert idle_cycles after EACH attempt (including WAIT retries).
   * - WAIT 最多重试 wait_retry 次 / Retry WAIT up to wait_retry times.
   * - 若 ACK==FAULT 且策略允许，则尝试清除 sticky 错误。
   * If ACK==FAULT and enabled, best-effort clear sticky errors.
   *
   * @param req 请求包 / Request.
   * @param resp 响应包 / Response.
   * @return ErrorCode 操作结果（传输流程级） / Error code (flow-level).
   */
  ErrorCode TransferWithRetry(const SwdProtocol::Request& req,
                              SwdProtocol::Response& resp)
  {
    ResetResponse(resp);

    uint32_t retries = 0;

    while (true)
    {
      auto ec = Transfer(req, resp);
      if (ec != ErrorCode::OK)
      {
        resp.ack = SwdProtocol::Ack::PROTOCOL;
        InvalidateSelectCache();
        return ec;
      }

      // CMSIS-DAP IdleCycles：每次传输尝试后插入。
      // CMSIS-DAP IdleCycles: insert after EACH transfer attempt.
      if (policy_.idle_cycles != 0u)
      {
        IdleClocks(policy_.idle_cycles);
      }

      if (resp.ack != SwdProtocol::Ack::WAIT)
      {
        break;
      }
      if (retries >= policy_.wait_retry)
      {
        break;
      }
      ++retries;
    }

    if (resp.ack != SwdProtocol::Ack::OK)
    {
      InvalidateSelectCache();
    }

    if (resp.ack == SwdProtocol::Ack::FAULT && policy_.clear_sticky_on_fault)
    {
      (void)ClearStickyErrors();
    }

    return ErrorCode::OK;
  }

  // --------------------------------------------------------------------------
  // DP/AP helpers
  // --------------------------------------------------------------------------

  /**
   * @brief DP 寄存器读取（无重试） / DP register read (no retry).
   * @param reg DP 读寄存器 / DP read register.
   * @param val 输出：读到的数据 / Output: read value.
   * @param ack 输出：ACK / Output: ACK.
   * @return ErrorCode 操作结果 / Error code.
   */
  ErrorCode DpRead(SwdProtocol::DpReadReg reg, uint32_t& val, SwdProtocol::Ack& ack)
  {
    SwdProtocol::Response resp;
    const ErrorCode EC = Transfer(SwdProtocol::make_dp_read_req(reg), resp);
    if (EC != ErrorCode::OK)
    {
      ack = SwdProtocol::Ack::PROTOCOL;
      return EC;
    }

    ack = resp.ack;

    if (resp.ack != SwdProtocol::Ack::OK || !resp.parity_ok)
    {
      return ErrorCode::FAILED;
    }

    val = resp.rdata;
    return ErrorCode::OK;
  }

  /**
   * @brief DP 寄存器写入（无重试） / DP register write (no retry).
   * @param reg DP 写寄存器 / DP write register.
   * @param val 写入数据 / Write value.
   * @param ack 输出：ACK / Output: ACK.
   * @return ErrorCode 操作结果 / Error code.
   */
  ErrorCode DpWrite(SwdProtocol::DpWriteReg reg, uint32_t val, SwdProtocol::Ack& ack)
  {
    SwdProtocol::Response resp;
    const ErrorCode EC = Transfer(SwdProtocol::make_dp_write_req(reg, val), resp);
    if (EC != ErrorCode::OK)
    {
      ack = SwdProtocol::Ack::PROTOCOL;
      return EC;
    }

    ack = resp.ack;
    return (resp.ack == SwdProtocol::Ack::OK) ? ErrorCode::OK : ErrorCode::FAILED;
  }

  /**
   * @brief DP 读事务（带重试） / DP read transaction (with retry).
   * @param reg DP 读寄存器 / DP read register.
   * @param val 输出：读到的数据 / Output: read value.
   * @param ack 输出：ACK / Output: ACK.
   * @return ErrorCode 操作结果 / Error code.
   */
  ErrorCode DpReadTxn(SwdProtocol::DpReadReg reg, uint32_t& val, SwdProtocol::Ack& ack)
  {
    SwdProtocol::Response resp;
    const ErrorCode EC = TransferWithRetry(SwdProtocol::make_dp_read_req(reg), resp);
    ack = resp.ack;

    if (EC != ErrorCode::OK)
    {
      return EC;
    }

    if (resp.ack != SwdProtocol::Ack::OK || !resp.parity_ok)
    {
      return ErrorCode::FAILED;
    }

    val = resp.rdata;
    return ErrorCode::OK;
  }

  /**
   * @brief DP 写事务（带重试） / DP write transaction (with retry).
   * @param reg DP 写寄存器 / DP write register.
   * @param val 写入数据 / Write value.
   * @param ack 输出：ACK / Output: ACK.
   * @return ErrorCode 操作结果 / Error code.
   */
  ErrorCode DpWriteTxn(SwdProtocol::DpWriteReg reg, uint32_t val, SwdProtocol::Ack& ack)
  {
    SwdProtocol::Response resp;
    const ErrorCode EC =
        TransferWithRetry(SwdProtocol::make_dp_write_req(reg, val), resp);
    ack = resp.ack;

    if (EC != ErrorCode::OK)
    {
      return EC;
    }

    return (resp.ack == SwdProtocol::Ack::OK) ? ErrorCode::OK : ErrorCode::FAILED;
  }

  /**
   * @brief AP 读事务（带重试，包含 RDBUFF 回读）。
   * AP read transaction (with retry, with RDBUFF readback).
   *
   * 注意：AP 读为 posted；该辅助函数执行：1) AP READ（获得 posted 数据）。2) 读取 DP
   * RDBUFF 获取本次 AP READ 的实际数据。
   * Note: AP reads are posted; this helper performs: 1) AP READ (gets posted data).
   * 2) DP RDBUFF read to obtain the actual data for this AP read.
   *
   * @param addr2b AP 寄存器地址（A2/A3，两位） / AP register address (A2/A3, 2-bit).
   * @param val 输出：实际读取的数据 / Output: actual read value.
   * @param ack 输出：ACK / Output: ACK.
   * @return ErrorCode 操作结果 / Error code.
   */
  ErrorCode ApReadTxn(uint8_t addr2b, uint32_t& val, SwdProtocol::Ack& ack)
  {
    SwdProtocol::Response resp;
    const ErrorCode EC = TransferWithRetry(SwdProtocol::make_ap_read_req(addr2b), resp);
    ack = resp.ack;

    if (EC != ErrorCode::OK)
    {
      return EC;
    }
    if (resp.ack != SwdProtocol::Ack::OK)
    {
      return ErrorCode::FAILED;
    }

    return DpReadTxn(SwdProtocol::DpReadReg::RDBUFF, val, ack);
  }

  /**
   * @brief AP 读事务（带重试，不读 RDBUFF；返回 posted 数据）。
   *        AP read transaction (with retry, no RDBUFF; returns posted data).
   *
   * @note 返回的 rdata 是上一笔 AP READ 结果（posted）。
   *       The returned rdata is the previous AP READ result (posted).
   *       调用方需额外读取一次 DP RDBUFF 以获得最后一次 AP READ 的真实数据。
   *       Caller must read DP RDBUFF once to obtain the last AP READ value.
   *
   * @param addr2b AP 寄存器地址（A2/A3，两位） / AP register address (A2/A3, 2-bit).
   * @param posted_val 输出：posted 数据 / Output: posted value.
   * @param ack 输出：ACK / Output: ACK.
   * @return ErrorCode 操作结果 / Error code.
   */
  ErrorCode ApReadPostedTxn(uint8_t addr2b, uint32_t& posted_val, SwdProtocol::Ack& ack)
  {
    SwdProtocol::Response resp;
    const ErrorCode EC = TransferWithRetry(SwdProtocol::make_ap_read_req(addr2b), resp);
    ack = resp.ack;

    if (EC != ErrorCode::OK)
    {
      return EC;
    }
    if (resp.ack != SwdProtocol::Ack::OK || !resp.parity_ok)
    {
      return ErrorCode::FAILED;
    }

    posted_val = resp.rdata;
    return ErrorCode::OK;
  }

  /**
   * @brief 读取 DP RDBUFF（带重试） / Read DP RDBUFF (with retry).
   * @param val 输出：读到的数据 / Output: read value.
   * @param ack 输出：ACK / Output: ACK.
   * @return ErrorCode 操作结果 / Error code.
   */
  ErrorCode DpReadRdbuffTxn(uint32_t& val, SwdProtocol::Ack& ack)
  {
    return DpReadTxn(SwdProtocol::DpReadReg::RDBUFF, val, ack);
  }

  /**
   * @brief AP 写事务（带重试） / AP write transaction (with retry).
   * @param addr2b AP 寄存器地址（A2/A3，两位） / AP register address (A2/A3, 2-bit).
   * @param val 写入数据 / Write value.
   * @param ack 输出：ACK / Output: ACK.
   * @return ErrorCode 操作结果 / Error code.
   */
  ErrorCode ApWriteTxn(uint8_t addr2b, uint32_t val, SwdProtocol::Ack& ack)
  {
    SwdProtocol::Response resp;
    const ErrorCode EC =
        TransferWithRetry(SwdProtocol::make_ap_write_req(addr2b, val), resp);
    ack = resp.ack;

    if (EC != ErrorCode::OK)
    {
      return EC;
    }

    return (resp.ack == SwdProtocol::Ack::OK) ? ErrorCode::OK : ErrorCode::FAILED;
  }

  /**
   * @brief 读取 DP IDCODE / Read DP IDCODE.
   * @param idcode 输出：IDCODE / Output: IDCODE.
   * @param ack 输出：ACK / Output: ACK.
   * @return ErrorCode 操作结果 / Error code.
   */
  ErrorCode ReadIdCode(uint32_t& idcode, SwdProtocol::Ack& ack)
  {
    return DpRead(SwdProtocol::DpReadReg::IDCODE, idcode, ack);
  }

  /**
   * @brief 写入 DP ABORT（无重试） / Write DP ABORT (no retry).
   * @param flags ABORT 标志位 / ABORT flags.
   * @param ack 输出：ACK / Output: ACK.
   * @return ErrorCode 操作结果 / Error code.
   */
  ErrorCode WriteAbort(uint32_t flags, SwdProtocol::Ack& ack)
  {
    return DpWrite(SwdProtocol::DpWriteReg::ABORT, flags, ack);
  }

  /**
   * @brief 写入 DP ABORT（带重试） / Write DP ABORT (with retry).
   * @param flags ABORT 标志位 / ABORT flags.
   * @param ack 输出：ACK / Output: ACK.
   * @return ErrorCode 操作结果 / Error code.
   */
  ErrorCode WriteAbortTxn(uint32_t flags, SwdProtocol::Ack& ack)
  {
    return DpWriteTxn(SwdProtocol::DpWriteReg::ABORT, flags, ack);
  }

  // --------------------------------------------------------------------------
  // SELECT cache
  // --------------------------------------------------------------------------

  /**
   * @brief 写 SELECT（带缓存；命中则跳过写入）。
   * Write SELECT with cache (skip write on hit).
   * @param select SELECT 值 / SELECT value.
   * @param ack 输出：ACK / Output: ACK.
   * @return ErrorCode 操作结果 / Error code.
   */
  ErrorCode SetSelectCached(uint32_t select, SwdProtocol::Ack& ack)
  {
    if (select_valid_ && (select_cache_ == select))
    {
      ack = SwdProtocol::Ack::OK;
      return ErrorCode::OK;
    }

    const ErrorCode EC = DpWriteTxn(SwdProtocol::DpWriteReg::SELECT, select, ack);
    if (EC == ErrorCode::OK && ack == SwdProtocol::Ack::OK)
    {
      select_cache_ = select;
      select_valid_ = true;
    }
    return EC;
  }

  /**
   * @brief 失效 SELECT 缓存 / Invalidate SELECT cache.
   */
  void InvalidateSelectCache()
  {
    select_valid_ = false;
    select_cache_ = 0u;
  }

 public:
  /**
   * @brief 构造函数 / Constructor.
   */
  Swd() = default;

  /**
   * @brief 插入空闲时钟周期 / Insert idle clock cycles.
   * @param cycles 周期数 / Number of cycles.
   */
  virtual void IdleClocks(uint32_t cycles) = 0;

  // 输出：在 SWDIO 上按 LSB-first 输出 data，并产生 SWCLK 脉冲。
  // cycles: 位数（例如 1..256）；data_lsb_first 长度 >= (cycles+7)/8
  virtual ErrorCode SeqWriteBits(uint32_t cycles, const uint8_t* data_lsb_first) = 0;

  // 输入：产生 SWCLK 脉冲并采样 SWDIO，按 LSB-first 写入 out_lsb_first。
  // out_lsb_first 长度 >= (cycles+7)/8
  virtual ErrorCode SeqReadBits(uint32_t cycles, uint8_t* out_lsb_first) = 0;

 private:
  /**
   * @brief 重置响应结构体为默认值 / Reset response to defaults.
   * @param resp 响应结构体 / Response structure.
   */
  static inline void ResetResponse(SwdProtocol::Response& resp)
  {
    resp.ack = SwdProtocol::Ack::PROTOCOL;
    resp.rdata = 0u;
    resp.parity_ok = true;
  }

  /**
   * @brief 清除 DP sticky 错误（尽力而为） / Clear DP sticky errors (best-effort).
   * @return ErrorCode 操作结果 / Error code.
   */
  ErrorCode ClearStickyErrors()
  {
    SwdProtocol::Ack ack = SwdProtocol::Ack::NO_ACK;
    const uint32_t FLAGS =
        SwdProtocol::DP_ABORT_STKCMPCLR | SwdProtocol::DP_ABORT_STKERRCLR |
        SwdProtocol::DP_ABORT_WDERRCLR | SwdProtocol::DP_ABORT_ORUNERRCLR;
    return DpWrite(SwdProtocol::DpWriteReg::ABORT, FLAGS, ack);
  }

 private:
  TransferPolicy policy_{};  ///< 传输策略 / Transfer policy.

  uint32_t select_cache_ = 0u;  ///< SELECT 缓存值 / SELECT cached value.
  bool select_valid_ = false;   ///< SELECT 缓存是否有效 / SELECT cache valid.
};

}  // namespace LibXR::Debug
