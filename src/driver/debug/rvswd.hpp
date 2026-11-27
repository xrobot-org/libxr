#pragma once

#include <cstdint>

#include "libxr_def.hpp"
#include "rvswd_protocol.hpp"

namespace LibXR::Debug {
/**
 * @class RvSwd
 * @brief WCH RVSWD（Serial Debug Interface）抽象基类。
 *
 * RVSWD 在 WCH-Link RV 语义中以 DMI（addr/data/op）事务形式暴露。
 * 本类定义了 RVSWD 链路控制与 DMI 读写辅助接口，便于 USB 协议层
 * （如 WCH-LinkRV 类）与具体硬件实现解耦。
 */
class RvSwd
{
 public:
  /**
   * @struct TransferPolicy
   * @brief RVSWD 传输策略。
   */
  struct TransferPolicy
  {
    uint16_t busy_retry = 100;  ///< DMI busy 重试次数上限。
    uint8_t idle_cycles = 0;    ///< 每次传输尝试后插入的空闲周期。
  };

  using Op = RvSwdProtocol::Op;
  using Ack = RvSwdProtocol::Ack;
  using Request = RvSwdProtocol::Request;
  using Response = RvSwdProtocol::Response;

  virtual ~RvSwd() = default;

  RvSwd(const RvSwd&) = delete;
  RvSwd& operator=(const RvSwd&) = delete;

  void SetTransferPolicy(const TransferPolicy& policy) { policy_ = policy; }
  [[nodiscard]] const TransferPolicy& GetTransferPolicy() const { return policy_; }

  /**
   * @brief 设置调试时钟频率（可选实现）。
   */
  virtual ErrorCode SetClockHz(uint32_t hz) = 0;

  /**
   * @brief 关闭探针并释放资源。
   */
  virtual void Close() = 0;

  /**
   * @brief 线复位。
   */
  virtual ErrorCode LineReset() = 0;

  /**
   * @brief 进入 RVSWD 模式。
   */
  virtual ErrorCode EnterRvSwd() = 0;

  /**
   * @brief 执行一次 DMI 传输（不含 busy 重试）。
   */
  virtual ErrorCode Transfer(const Request& req, Response& resp) = 0;

  /**
   * @brief 插入空闲时钟周期（可选）。
   */
  virtual void IdleClocks(uint32_t cycles) { UNUSED(cycles); }

  /**
   * @brief 带 busy 重试的 DMI 传输封装。
   */
  ErrorCode TransferWithRetry(const Request& req, Response& resp)
  {
    ResetResponse(resp);

    uint16_t retry = 0;
    while (true)
    {
      const ErrorCode ec = Transfer(req, resp);
      if (ec != ErrorCode::OK)
      {
        resp.ack = Ack::PROTOCOL;
        return ec;
      }

      if (policy_.idle_cycles != 0u)
      {
        IdleClocks(policy_.idle_cycles);
      }

      if (resp.ack != Ack::BUSY)
      {
        break;
      }

      if (retry >= policy_.busy_retry)
      {
        return ErrorCode::TIMEOUT;
      }
      ++retry;
    }

    return (resp.ack == Ack::OK) ? ErrorCode::OK : ErrorCode::FAILED;
  }

  /**
   * @brief DMI 读事务（带 busy 重试）。
   */
  ErrorCode DmiReadTxn(uint8_t addr, uint32_t& data, Ack& ack)
  {
    Response resp = {};
    const ErrorCode ec = TransferWithRetry({addr, 0u, Op::READ}, resp);
    ack = resp.ack;
    if (ec != ErrorCode::OK)
    {
      return ec;
    }

    data = resp.data;
    return ErrorCode::OK;
  }

  /**
   * @brief DMI 写事务（带 busy 重试）。
   */
  ErrorCode DmiWriteTxn(uint8_t addr, uint32_t data, Ack& ack)
  {
    Response resp = {};
    const ErrorCode ec = TransferWithRetry({addr, data, Op::WRITE}, resp);
    ack = resp.ack;
    return ec;
  }

 protected:
  RvSwd() = default;

 private:
  static void ResetResponse(Response& resp)
  {
    resp.addr = 0u;
    resp.data = 0u;
    resp.ack = Ack::PROTOCOL;
  }

 private:
  TransferPolicy policy_ = {};
};

}  // namespace LibXR::Debug

