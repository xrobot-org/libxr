#pragma once

#include "cdc_base.hpp"
#include "libxr_mem.hpp"

namespace LibXR::USB
{

/// 在 OUT 端点的每个空闲槽上挂一个包的接收，直到 BUSY；仅 owner / Arm one packet on
/// every free slot of an OUT endpoint until BUSY; owner only
inline void ArmReceives(Endpoint* out, bool in_isr)
{
  while (out->Receive(out->MaxPacketSize(), in_isr) == ErrorCode::OK)
  {
  }
}

/**
 * @class CDCWriteTest
 * @brief CDC 写测试：测设备到主机方向 / CDC write test: exercises device-to-host
 *
 * 主机每发来一个 OUT 包，数据丢弃、槽交回、重新挂接收；DTR 置位时在 IN 上发一个满包
 * 零数据。每个空闲的 OUT 槽都挂着接收。
 * For each OUT packet from the host the data is dropped, the slot handed back and the
 * receive armed again; while DTR is set one full zero-filled packet goes out on IN. Every
 * free OUT slot has a receive armed.
 */
class CDCWriteTest : public CDCBase
{
 public:
  using CDCBase::CDCBase;

 protected:
  /// 基类绑定成功后在每个槽上挂接收 / Arm a receive on every slot after the base bind
  /// succeeds
  [[nodiscard]] ErrorCode BindEndpoints(EndpointPool& pool, uint8_t first_interface,
                                        bool in_isr) override
  {
    const ErrorCode result = CDCBase::BindEndpoints(pool, first_interface, in_isr);
    if (result == ErrorCode::OK)
    {
      ArmReceives(GetDataOutEndpoint(), in_isr);
    }
    return result;
  }

  /// 交回槽、重新挂接收，DTR 置位时发一个满包 / Hand the slot back, arm again, send
  /// one full packet while DTR is set
  void OnDataOutComplete(bool in_isr, const Endpoint::Completion& completion) override
  {
    Endpoint* out = GetDataOutEndpoint();
    if (completion.slot.Valid())
    {
      (void)out->ReleaseSlot(completion.slot);
    }
    ArmReceives(out, in_isr);
    if (!IsDtrSet())
    {
      return;
    }
    Endpoint* in = GetDataInEndpoint();
    Endpoint::Slot slot;
    if (in->AcquireSlot(slot) != ErrorCode::OK)
    {
      // 没有空闲槽：跳过，下一个 OUT 包再试。
      // No free slot: skip, the next OUT packet tries again.
      return;
    }
    const size_t length = in->MaxPacketSize();
    LibXR::Memory::FastSet(slot.data.addr_, 0, length);
    (void)in->Submit(slot, length, false, in_isr);
  }

  /// 不需要后续动作 / Nothing follows
  void OnDataInComplete(bool in_isr, const Endpoint::Completion& completion) override
  {
    UNUSED(in_isr);
    UNUSED(completion);
  }
};

/**
 * @class CDCReadTest
 * @brief CDC 读测试：测主机到设备方向 / CDC read test: exercises host-to-device
 *
 * 每个 OUT 包到达后立即交回槽并重新挂接收，持续读入；每个空闲的 OUT 槽都挂着接收。
 * Each OUT packet's slot is handed back and the receive armed again at once, so reading
 * never stops; every free OUT slot has a receive armed.
 */
class CDCReadTest : public CDCBase
{
 public:
  using CDCBase::CDCBase;

 protected:
  /// 基类绑定成功后在每个槽上挂接收 / Arm a receive on every slot after the base bind
  /// succeeds
  [[nodiscard]] ErrorCode BindEndpoints(EndpointPool& pool, uint8_t first_interface,
                                        bool in_isr) override
  {
    const ErrorCode result = CDCBase::BindEndpoints(pool, first_interface, in_isr);
    if (result == ErrorCode::OK)
    {
      ArmReceives(GetDataOutEndpoint(), in_isr);
    }
    return result;
  }

  /// 交回槽并重新挂接收 / Hand the slot back and arm again
  void OnDataOutComplete(bool in_isr, const Endpoint::Completion& completion) override
  {
    Endpoint* out = GetDataOutEndpoint();
    if (completion.slot.Valid())
    {
      (void)out->ReleaseSlot(completion.slot);
    }
    ArmReceives(out, in_isr);
  }

  /// 读测试不发数据 / The read test sends nothing
  void OnDataInComplete(bool in_isr, const Endpoint::Completion& completion) override
  {
    UNUSED(in_isr);
    UNUSED(completion);
  }
};

}  // namespace LibXR::USB
