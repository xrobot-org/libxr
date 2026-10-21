#pragma once
#include <cstring>

#include "cdc_base.hpp"

namespace LibXR::USB
{

/**
 * @brief USB CDC ACM 写测试类
 *        USB CDC ACM write test class
 *
 * 用于测试设备向主机的数据发送通道。
 * - 每收到一次主机 OUT 数据（租约归还后重新挂接收）；
 * - 当 DTR 已置位时，通过 IN 端点发送一个满包零数据。
 *
 * Used for testing the device-to-host (TX) path.
 * - Each host OUT lease is returned so the receive re-arms; the payload is ignored.
 * - If DTR is asserted, one full zero-filled packet is sent via the IN endpoint.
 */
class CDCWriteTest : public CDCBase
{
 public:
  CDCWriteTest(
      Endpoint::EPNumber data_in_ep_num, Endpoint::EPNumber data_out_ep_num,
      Endpoint::EPNumber comm_ep_num,
      const char* control_interface_string = CDCBase::DEFAULT_CONTROL_INTERFACE_STRING,
      const char* data_interface_string = CDCBase::DEFAULT_DATA_INTERFACE_STRING)
      : CDCBase(data_in_ep_num, data_out_ep_num, comm_ep_num, control_interface_string,
                data_interface_string)
  {
  }

  /**
   * @brief 绑定端点并挂上第一次 OUT 接收
   *        Bind the endpoints and arm the first OUT receive
   *
   * CDCBase 只做 Configure，不会挂接收（接收由具体类负责，见 CDCUart::ArmRx）；
   * 不挂这一下，写测试永远收不到主机的 OUT 数据。
   * CDCBase only configures; arming a receive is the concrete class's job (see
   * CDCUart::ArmRx). Without it the write test would never receive host OUT data.
   */
  void BindEndpoints(EndpointPool& endpoint_pool, uint8_t start_itf_num,
                     bool in_isr) override
  {
    CDCBase::BindEndpoints(endpoint_pool, start_itf_num, in_isr);
    auto ep_data_out = GetDataOutEndpoint();
    if (ep_data_out != nullptr)
    {
      (void)ep_data_out->Receive(ep_data_out->MaxPacketSize(), in_isr);
    }
  }

  /**
   * @brief OUT 端点完成回调（写测试：归还租约并按 DTR 触发一次发送）
   *        Data OUT complete callback (write test: return the lease, send once on DTR)
   *
   * 写测试场景下不消费主机->设备数据；租约归还后按 receive_size 重新挂接收。
   * 若 DTR 置位，则借槽发送一个满包零数据。
   * In write test the host-to-device payload is ignored; returning the lease re-arms
   * the receive. With DTR asserted, one full zero-filled packet is submitted.
   */
  void OnDataOutComplete(bool in_isr, const Endpoint::Completion& completion) override
  {
    auto ep_data_out = GetDataOutEndpoint();
    if (completion.result == ErrorCode::OK && completion.slot.Valid() && ep_data_out)
    {
      (void)ep_data_out->ReturnSlot(completion.slot, in_isr);
    }
    if (IsDtrSet())
    {
      TrySendOnePacket(in_isr);
    }
  }

  /**
   * @brief IN 端点完成回调（写测试：无后续动作）
   *        Data IN complete callback (write test: nothing to chain)
   */
  void OnDataInComplete(bool in_isr, const Endpoint::Completion& completion) override
  {
    UNUSED(in_isr);
    UNUSED(completion);
  }

 private:
  /// 借槽发送一个满包零数据；没有空闲槽就跳过，下一次 OUT 完成会再试。
  /// Send one full zero-filled packet from a fresh slot; skipped when no slot is free,
  /// and retried on the next OUT completion.
  void TrySendOnePacket(bool in_isr)
  {
    auto ep_data_in = GetDataInEndpoint();
    if (ep_data_in == nullptr)
    {
      return;
    }
    Endpoint::Slot slot;
    if (ep_data_in->AcquireSlot(slot, in_isr) != ErrorCode::OK)
    {
      return;
    }
    const size_t length =
        LibXR::min<size_t>(ep_data_in->MaxPacketSize(), slot.data.size_);
    std::memset(slot.data.addr_, 0, length);
    (void)ep_data_in->Submit(slot, length, false, in_isr);
  }
};

/**
 * @brief USB CDC ACM 读测试类
 *        USB CDC ACM read test class
 *
 * 用于测试主机向设备的数据接收通道。
 * - 绑定阶段由本类挂上第一次 OUT 接收（CDCBase 只做 Configure）；
 * - 每次接收完成后立即归还租约（按 receive_size 重新挂接收），实现持续读入。
 *
 * Used for testing the host-to-device (RX) path.
 * - This class arms the first OUT receive at bind time (CDCBase only configures).
 * - Each completed lease is returned at once, which re-arms the receive and keeps
 *   reading continuously.
 */
class CDCReadTest : public CDCBase
{
 public:
  CDCReadTest(
      Endpoint::EPNumber data_in_ep_num, Endpoint::EPNumber data_out_ep_num,
      Endpoint::EPNumber comm_ep_num,
      const char* control_interface_string = CDCBase::DEFAULT_CONTROL_INTERFACE_STRING,
      const char* data_interface_string = CDCBase::DEFAULT_DATA_INTERFACE_STRING)
      : CDCBase(data_in_ep_num, data_out_ep_num, comm_ep_num, control_interface_string,
                data_interface_string)
  {
  }

  void BindEndpoints(EndpointPool& endpoint_pool, uint8_t start_itf_num,
                     bool in_isr) override
  {
    // 基类只做 Configure；首次接收必须由本类挂上，否则永远收不到数据。
    // The base only configures; this class must arm the first receive or it would never
    // receive anything.
    CDCBase::BindEndpoints(endpoint_pool, start_itf_num, in_isr);
    auto ep_data_out = GetDataOutEndpoint();
    if (ep_data_out != nullptr)
    {
      (void)ep_data_out->Receive(ep_data_out->MaxPacketSize(), in_isr);
    }
  }

  /**
   * @brief OUT 端点完成回调（读测试：归还租约即重新挂接收）
   *        Data OUT complete callback (read test: returning the lease re-arms it)
   */
  void OnDataOutComplete(bool in_isr, const Endpoint::Completion& completion) override
  {
    auto ep_data_out = GetDataOutEndpoint();
    if (completion.result == ErrorCode::OK && completion.slot.Valid() && ep_data_out)
    {
      (void)ep_data_out->ReturnSlot(completion.slot, in_isr);
    }
  }

  /**
   * @brief IN 端点完成回调（读测试：无操作）
   *        Data IN complete callback (read test: no-op)
   *
   * 读测试场景下不产生设备->主机数据，因此此处为空实现。
   * In read test, no device-to-host data is generated; this is a no-op.
   */
  void OnDataInComplete(bool in_isr, const Endpoint::Completion& completion) override
  {
    UNUSED(in_isr);
    UNUSED(completion);
  }
};

}  // namespace LibXR::USB
