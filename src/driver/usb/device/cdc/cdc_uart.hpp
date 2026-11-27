#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

#include "cdc_base.hpp"
#include "libxr_def.hpp"
#include "libxr_rw.hpp"
#include "uart.hpp"

namespace LibXR::USB
{

class CDCUart;

/**
 * @brief CDC UART 的读端口：读者腾出空间时通知 CDC / Read port of the CDC UART: tells
 *        the CDC when the reader frees space
 */
class CDCUartReadPort : public ReadPort
{
 public:
  /// 构造；size 为接收队列字节数 / Construct; size is the receive queue in bytes
  CDCUartReadPort(size_t size, CDCUart& owner) : ReadPort(size), owner_(owner) {}

 protected:
  /// 读者取走数据后调用（读者的上下文）/ Called after the reader took data (in the
  /// reader's context)
  void OnReadQueueSpaceAvailable(bool in_isr) override;

 private:
  CDCUart& owner_;  ///< 所属 CDC UART / Owning CDC UART
};

/**
 * @class CDCUart
 * @brief USB CDC ACM 虚拟串口 / USB CDC ACM virtual serial port
 *
 * 发送：写端口里的数据由 owner 拷进借来的 IN 槽并提交，一个槽可以装多次写入；提交时写
 * 端口已空，就要求结尾零长度包（Submit 的 zlp），否则后面还有数据，不需要。接收：每个
 * OUT 请求一个包；收到的槽留在本类，数据搬进读端口后交回并重新挂接收；读端口满时不交回，
 * 主机因此被 NAK。复位或换配置时写端口里还没取走的数据留到下一次绑定后发送；已提交但没
 * 发完的槽随端点关闭丢弃。
 * Transmit: the owner copies write-port data into a borrowed IN slot and submits it; one
 * slot may carry several writes. When the write port is empty at the submit, the closing
 * zero-length packet is requested (Submit's zlp); otherwise more data follows and none is
 * needed. Receive: one packet per OUT request; a received slot stays with this class
 * until its bytes are in the read port, then it is handed back and armed again; while
 * the read port is full it is kept, so the host is NAKed. Across a reset or a
 * configuration change, data still in the write port is sent after the next bind; a
 * submitted slot not yet sent is dropped when the endpoint closes.
 *
 * @note 应用在任何上下文 Read、Write；收发的推进只在 owner 中（OnService、端点完成、
 *       BindEndpoints）。
 *       The application reads and writes in any context; transfers advance in the owner
 *       only (OnService, endpoint completions, BindEndpoints).
 */
class CDCUart : public CDCBase, public LibXR::UART
{
 public:
  using LibXR::UART::Read;
  using LibXR::UART::read_port_;
  using LibXR::UART::Write;
  using LibXR::UART::write_port_;

  /**
   * @brief 构造 / Construct
   * @param data_in_ep_num 数据 IN 端点号 / Data IN endpoint number
   * @param data_out_ep_num 数据 OUT 端点号 / Data OUT endpoint number
   * @param comm_ep_num 通知端点号 / Notification endpoint number
   * @param rx_buffer_size 接收队列字节数 / Receive queue bytes
   * @param tx_buffer_size 发送队列字节数 / Transmit queue bytes
   * @param tx_queue_size 发送请求个数 / Transmit request count
   * @param control_interface_string 通信接口字符串 / Communication interface string
   * @param data_interface_string 数据接口字符串 / Data interface string
   */
  CDCUart(Endpoint::EPNumber data_in_ep_num, Endpoint::EPNumber data_out_ep_num,
          Endpoint::EPNumber comm_ep_num, size_t rx_buffer_size = 128,
          size_t tx_buffer_size = 128, size_t tx_queue_size = 5,
          const char* control_interface_string = DEFAULT_CONTROL_INTERFACE_STRING,
          const char* data_interface_string = DEFAULT_DATA_INTERFACE_STRING);

  /**
   * @brief 由设备一侧改线路编码 / Change the line coding from the device side
   * @return OK；ARG_ERR：取值不是 CDC 能表示的。
   *         OK; ARG_ERR: a value CDC cannot express.
   * @note 经单写者序号锁交给 owner，owner 在 OnService 中更新线路编码并发 SERIAL_STATE。
   *       只能由一个上下文调用：不能两个线程同时，也不能线程与中断同时。
   *       Handed to the owner through a single-writer sequence lock; the owner updates
   *       the line coding in OnService and sends SERIAL_STATE. One context only: never
   *       two threads at once, nor a thread and an interrupt.
   */
  ErrorCode SetConfig(UART::Configuration cfg, bool in_isr = false) override;

 protected:
  /// 基类绑定成功后挂接收并发送写端口里的数据 / After the base bind succeeds, arm the
  /// receives and send what the write port holds
  [[nodiscard]] ErrorCode BindEndpoints(EndpointPool& pool, uint8_t first_interface,
                                        bool in_isr) override;
  /// 解绑，再交回留在本类的接收槽 / Unbind, then hand back the receive slots kept here
  void UnbindEndpoints(EndpointPool& pool, bool in_isr) override;
  /// 应用 SetConfig，发通知，推进发送和接收 / Apply SetConfig, notify, advance both
  /// directions
  void OnService(bool in_isr) override;
  /// 留下收到的槽并搬数据；其他结果重新挂接收 / Keep the received slot and move its
  /// bytes; any other result arms again
  void OnDataOutComplete(bool in_isr, const Endpoint::Completion& completion) override;
  /// 继续发送 / Keep sending
  void OnDataInComplete(bool in_isr, const Endpoint::Completion& completion) override;

 private:
  friend class CDCUartReadPort;

  /// 写端口的通知：请 owner 发送 / Write-port notification: ask the owner to send
  static void WriteFun(WritePort& port, bool in_isr);

  /// owner：写端口有数据且有空闲槽时装满一个槽并提交，直到没有 / Owner: while the
  /// write port has data and a slot is free, fill one slot and submit it
  void PumpTx(bool in_isr);
  /// owner：为每个空闲的 OUT 槽挂一个包的接收 / Owner: arm one packet on every free
  /// OUT slot
  void ArmRx(bool in_isr);
  /// owner：按到达顺序把收到的数据搬进读端口 / Owner: move received bytes into the read
  /// port in arrival order
  void DrainRx(bool in_isr);
  /// owner：应用 SetConfig 交来的新配置 / Owner: apply the configuration from SetConfig
  void ApplyPendingConfig(bool in_isr);
  /// 读者腾出空间：有被挡住的接收槽时请 owner / The reader freed space: ask the owner if
  /// a received slot is held back
  void OnReceiveSpace(bool in_isr);

  CDCUartReadPort read_port_cdc_;    ///< 读端口 / Read port
  LibXR::WritePort write_port_cdc_;  ///< 写端口 / Write port

  /// @name 单写者序号锁：SetConfig 写，owner 读 / Single-writer sequence lock: written
  ///       by SetConfig, read by the owner
  /// @{
  std::atomic<uint32_t> config_sequence_{0U};   ///< 奇数表示写入中 / Odd while writing
  std::atomic<uint32_t> config_baud_{115200U};  ///< 波特率 / Baud rate
  /// 停止位、校验、数据位各一字节 / Stop bits, parity, data bits, one byte each
  std::atomic<uint32_t> config_format_{8U << 16U};
  /// @}

  /// @name 仅 owner / Owner only
  /// @{
  uint32_t applied_config_sequence_ = 0U;  ///< 已应用的序号 / Sequence applied
  /**
   * @brief 收到未搬完的槽，按到达顺序的环 / Received slots not yet moved, a ring in
   *        arrival order
   * @note 槽数由后端定，最多 MAX_SLOTS。
   *       The backend sets the count, MAX_SLOTS at most.
   */
  Endpoint::Slot rx_slots_[Endpoint::MAX_SLOTS];
  uint8_t rx_head_ = 0U;   ///< 最早收到的槽 / Oldest received slot
  uint8_t rx_count_ = 0U;  ///< 留在本类的槽数 / Slots kept here
  size_t rx_offset_ = 0U;  ///< 最早的槽已搬走的字节 / Bytes moved from the oldest
  /// @}

  /**
   * @brief 有接收槽因读端口满而留着 / A received slot is held because the read port is
   *        full
   * @note owner 置位，读者一侧清除并请 owner。
   *       Set by the owner, cleared by the reader side, which then asks the owner.
   */
  std::atomic<uint32_t> rx_waiting_{0U};
};

}  // namespace LibXR::USB
