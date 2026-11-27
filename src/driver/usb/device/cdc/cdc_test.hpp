#pragma once

#include "cdc_base.hpp"
#include "libxr_mem.hpp"

namespace LibXR::USB
{

/// 在 OUT 端点的每个空闲槽上挂 bytes 字节的接收，直到 BUSY；仅 owner / Arm a
/// receive of bytes bytes on every free slot of an OUT endpoint until BUSY; owner only
inline void ArmReceives(Endpoint* out, size_t bytes, bool in_isr)
{
  if (out == nullptr)
  {
    return;
  }
  while (out->Receive(bytes, in_isr) == ErrorCode::OK)
  {
  }
}

/**
 * @class CDCWriteTest
 * @brief CDC 写测试：测设备到主机方向 / CDC write test: exercises device-to-host
 *
 * 正常收下并丢弃主机发来的 OUT 包、重新挂接收；DTR 置位时在每个 OUT 完成或 IN 完成后
 * 按最大传输长度填满全零提交给所有空闲 IN 槽，实现持续发送；DTR 清零时停止继续提交。
 * Receives and discards OUT packets from the host and re-arms receives; while DTR is
 * set, each OUT or IN completion fills every free IN slot with zeros of MaxTransferSize()
 * for continuous transmission until DTR clears.
 */
class CDCWriteTest : public CDCBase
{
 public:
  using CDCBase::CDCBase;

 protected:
  /// 基类绑定成功后按单包长在每个 OUT 槽上挂接收 / Arm one-packet receives on every
  /// free OUT slot after base bind succeeds
  [[nodiscard]] ErrorCode BindEndpoints(EndpointPool& pool, uint8_t first_interface,
                                        bool in_isr) override
  {
    const ErrorCode result = CDCBase::BindEndpoints(pool, first_interface, in_isr);
    if (result == ErrorCode::OK)
    {
      Endpoint* out = GetDataOutEndpoint();
      ArmReceives(out, out->MaxPacketSize(), in_isr);
    }
    return result;
  }

  /// 交回槽并重挂接收（STALLED 时排队等 ClearStall）；仅 OK 时在 DTR 置位下泵送 IN /
  /// Release slot and re-arm receive (queues until ClearStall when STALLED); pump IN
  /// under DTR on OK completion only
  void OnDataOutComplete(bool in_isr, const Endpoint::Completion& completion) override
  {
    Endpoint* out = GetDataOutEndpoint();
    if (completion.slot.Valid())
    {
      (void)out->ReleaseSlot(completion.slot);
    }
    ArmReceives(out, out->MaxPacketSize(), in_isr);
    if (completion.result != ErrorCode::OK)
    {
      return;
    }
    PumpTx(in_isr);
  }

  /// 上一包 IN 完成后若成功且 DTR 置位则继续填满空闲 IN 槽 / Continue filling free IN
  /// slots on successful IN completion while DTR is set
  void OnDataInComplete(bool in_isr, const Endpoint::Completion& completion) override
  {
    if (completion.result == ErrorCode::OK)
    {
      PumpTx(in_isr);
    }
  }

 private:
  /// 向空闲 IN 槽填零并提交，直到无空闲槽、DTR 清零或未绑定；仅 owner / Fill free IN
  /// slots with zeros until BUSY, DTR clears, or unbound; owner only
  void PumpTx(bool in_isr)
  {
    if (pumping_tx_)
    {
      return;
    }
    pumping_tx_ = true;
    while (IsDtrSet())
    {
      Endpoint* in = GetDataInEndpoint();
      if (in == nullptr)
      {
        break;
      }
      Endpoint::Slot slot;
      if (in->AcquireSlot(slot) != ErrorCode::OK)
      {
        break;
      }
      const size_t length = in->MaxTransferSize();
      LibXR::Memory::FastSet(slot.data.addr_, 0, length);
      if (in->Submit(slot, length, false, in_isr) != ErrorCode::OK)
      {
        (void)in->ReleaseSlot(slot);
        break;
      }
    }
    pumping_tx_ = false;
  }

  bool pumping_tx_ = false;  ///< PumpTx 重入保护 / Reentry guard for PumpTx
};

/**
 * @class CDCReadTest
 * @brief CDC 读测试：测主机到设备方向 / CDC read test: exercises host-to-device
 *
 * 每个 OUT 槽按 MaxTransferSize() 挂满接收，收到后立即释放并重挂；非 OK 结果直接返回。
 * Every OUT slot is armed with MaxTransferSize(); completed slots are released and
 * re-armed immediately, while non-OK completions return without re-arming.
 */
class CDCReadTest : public CDCBase
{
 public:
  using CDCBase::CDCBase;

 protected:
  /// 基类绑定成功后按最大传输长度挂满每个 OUT 槽 / Arm MaxTransferSize() receives
  /// on every free OUT slot after base bind succeeds
  [[nodiscard]] ErrorCode BindEndpoints(EndpointPool& pool, uint8_t first_interface,
                                        bool in_isr) override
  {
    const ErrorCode result = CDCBase::BindEndpoints(pool, first_interface, in_isr);
    if (result == ErrorCode::OK)
    {
      Endpoint* out = GetDataOutEndpoint();
      ArmReceives(out, out->MaxTransferSize(), in_isr);
    }
    return result;
  }

  /// 交回槽并按最大传输长度重挂接收（STALLED 时排队等 ClearStall）/
  /// Release slot and re-arm MaxTransferSize() receive (queues until ClearStall when
  /// STALLED)
  void OnDataOutComplete(bool in_isr, const Endpoint::Completion& completion) override
  {
    Endpoint* out = GetDataOutEndpoint();
    if (completion.slot.Valid())
    {
      (void)out->ReleaseSlot(completion.slot);
    }
    ArmReceives(out, out->MaxTransferSize(), in_isr);
  }

  /// 读测试不发数据 / The read test sends nothing
  void OnDataInComplete(bool in_isr, const Endpoint::Completion& completion) override
  {
    UNUSED(in_isr);
    UNUSED(completion);
  }
};

}  // namespace LibXR::USB
