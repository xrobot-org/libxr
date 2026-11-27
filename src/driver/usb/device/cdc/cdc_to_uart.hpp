#pragma once

#include <cstddef>

#include "cdc_uart.hpp"
#include "libxr_cb.hpp"
#include "libxr_rw.hpp"
#include "uart.hpp"

namespace LibXR::USB
{

/**
 * @class CDCToUart
 * @brief CDC 与一个 UART 之间的双向桥 / Bidirectional bridge between CDC and a UART
 *
 * CDC 收到的数据写进 UART，UART 收到的数据写进 CDC；两个方向各由一条回调链推进：读完成
 * 后写，写完成后再读。主机的 SET_LINE_CODING 应用到 UART。
 * CDC data goes to the UART and UART data goes to CDC; each direction runs on a callback
 * chain: write after a read completes, read again after the write completes. The host's
 * SET_LINE_CODING is applied to the UART.
 */
class CDCToUart : public CDCUart
{
 public:
  /**
   * @brief 构造并准备两条回调链（首次绑定端点后启动）/ Construct and prepare both
   *        callback chains (started upon the first endpoint bind)
   * @param data_in_ep_num 数据 IN 端点号 / Data IN endpoint number
   * @param data_out_ep_num 数据 OUT 端点号 / Data OUT endpoint number
   * @param comm_ep_num 通知端点号 / Notification endpoint number
   * @param uart 被桥接的 UART / The bridged UART
   * @param rx_buffer_size CDC 到 UART 的中转字节数 / CDC-to-UART staging bytes
   * @param tx_buffer_size UART 到 CDC 的中转字节数 / UART-to-CDC staging bytes
   * @param tx_queue_size CDC 发送请求个数 / CDC transmit request count
   * @note 分配两块中转缓冲和四个回调，只在构造时分配。
   *       Allocates two staging buffers and four callbacks, at construction only.
   */
  CDCToUart(Endpoint::EPNumber data_in_ep_num, Endpoint::EPNumber data_out_ep_num,
            Endpoint::EPNumber comm_ep_num, LibXR::UART& uart,
            size_t rx_buffer_size = 128, size_t tx_buffer_size = 128,
            size_t tx_queue_size = 5);

 protected:
  /// 基类绑定成功后首次启动两条回调链 / Start both callback chains once after first bind
  [[nodiscard]] ErrorCode BindEndpoints(EndpointPool& pool, uint8_t first_interface,
                                        bool in_isr) override;
  /**
   * @brief 把线路编码应用到 UART / Apply the line coding to the UART
   * @note 总是应答，不 STALL。UART 不接受时逐项放宽：先 1 位停止位，再无校验，再 8
   *       数据位，最后回到上一次生效的配置；第一个被接受的就是生效值。都不接受时保留
   *       上一次的线路编码。
   *       Always acknowledged, never stalled. If the UART refuses, fields are relaxed
   *       one at a time: 1 stop bit, then no parity, then 8 data bits, then the
   *       previously applied configuration; the first accepted one is applied. If none
   *       is accepted, the previous line coding stays.
   */
  void ApplyLineCoding(bool in_isr, CDCLineCoding& coding,
                       UART::Configuration& config) override;

 private:
  /// 两份配置的四个字段是否都相同 / Whether all four fields of two configurations match
  static bool SameConfiguration(const UART::Configuration& a,
                                const UART::Configuration& b);

  RawData rx_buffer_;  ///< CDC 到 UART 的中转 / CDC-to-UART staging
  RawData tx_buffer_;  ///< UART 到 CDC 的中转 / UART-to-CDC staging
  LibXR::UART& uart_;  ///< 被桥接的 UART / The bridged UART

  Callback<ErrorCode> cb_read_cdc_;    ///< CDC 读完成 / CDC read done
  Callback<ErrorCode> cb_uart_write_;  ///< UART 写完成 / UART write done
  Callback<ErrorCode> cb_uart_read_;   ///< UART 读完成 / UART read done
  Callback<ErrorCode> cb_cdc_write_;   ///< CDC 写完成 / CDC write done

  WriteOperation op_write_cdc_;   ///< CDC 写 / CDC write
  WriteOperation op_write_uart_;  ///< UART 写 / UART write
  ReadOperation op_read_cdc_;     ///< CDC 读 / CDC read
  ReadOperation op_read_uart_;    ///< UART 读 / UART read
  bool chains_started_ = false;   ///< 两条回调链已启动 / Both chains have started
};

}  // namespace LibXR::USB
