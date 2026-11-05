/**
 * @file test_cdc_test.cpp
 * @brief cdc_test 设备类（槽模型）主机测试 / Host tests for the cdc_test device classes.
 *
 * 覆盖 5B 迁移改动的两条路径：OUT 完成回调改成新签名后归还租约（按 receive_size 重新
 * 挂接收，持续读入），以及 CDCWriteTest 在 DTR 置位时借槽发送一个满包零数据。
 * Covers the two paths the 5B migration changed: the OUT completion callback now returns
 * the lease (which re-arms the receive by receive_size, keeping the read going) and
 * CDCWriteTest submits one full zero-filled packet while DTR is asserted.
 */

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <vector>

#include "../../mock_endpoint.hpp"
#include "cdc_test.hpp"
#include "device_composition.hpp"

using namespace LibXR;
using namespace LibXR::USB;
using namespace LibXR::USB::Test;

namespace
{
static constexpr auto TEST_LANGUAGE = DescriptorStrings::MakeLanguagePack(
    DescriptorStrings::Language::EN_US, "LibXR", "USB tests", "0");

/// Exposes the protected bind/request entry points of a device class.
template <typename Base>
class Exposed : public Base
{
 public:
  using Base::Base;
  using Base::BindEndpoints;
  using Base::OnClassRequest;
  using Base::UnbindEndpoints;
  using ControlTransferResult = DeviceClass::ControlTransferResult;
};

using TestCdcWrite = Exposed<CDCWriteTest>;
using TestCdcRead = Exposed<CDCReadTest>;

/// CDC control request: SET_CONTROL_LINE_STATE (DTR = bit 0).
constexpr uint8_t SET_CONTROL_LINE_STATE = 0x22;
/// The serial-state notification is a 10-byte class notification.
constexpr size_t SERIAL_STATE_NOTIFICATION_SIZE = 10;

/// One CDC class plus the composition that gives it a class-service pool.
template <typename Class>
struct CdcFixture : Fixture
{
  Class cdc;
  DeviceComposition composition;
  Callback<uint32_t> handler;

  CdcFixture()
      : cdc(Number::EP1, Number::EP2, Number::EP3),
        composition(pool, {&TEST_LANGUAGE}, {{&cdc}})
  {
    // Stand-in for DeviceCore's class-service dispatch.
    handler = Callback<uint32_t>::Create(
        [](bool in_isr, DeviceClass* self, uint32_t events)
        {
          if ((events & EndpointPool::CLASS_EVENT) != 0U) self->OnService(in_isr);
        },
        static_cast<DeviceClass*>(&cdc));
    pool.SetControlHandler(handler);
  }
};

void TestWriteTest()
{
  CdcFixture<TestCdcWrite> f;
  f.cdc.BindEndpoints(f.pool, 0, false);

  // The class arms the first OUT receive itself: one packet per lease.
  TEST_ASSERT(f.out.active && f.out.Last().length == 64 && f.out.starts.size() == 1);
  TEST_ASSERT(!f.cdc.IsDtrSet());

  // Without DTR the host payload is dropped and the lease re-arms the receive.
  f.out.HostSends(Bytes(64, 3));
  TEST_ASSERT(f.out.active && f.out.starts.size() == 2 && f.out.Last().length == 64);
  TEST_ASSERT(f.in.starts.empty());

  // DTR asserted: the class reports the serial state on the notification endpoint.
  TestCdcWrite::ControlTransferResult result;
  TEST_ASSERT(f.cdc.OnClassRequest(false, SET_CONTROL_LINE_STATE, 0x01, 0, 0, result) ==
              ErrorCode::OK);
  TEST_ASSERT(f.cdc.IsDtrSet());
  TEST_ASSERT(f.comm.active && f.comm.Last().length == SERIAL_STATE_NOTIFICATION_SIZE);
  TEST_ASSERT(f.comm.Last().data[0] == 0xA1 && f.comm.Last().data[1] == 0x20);
  f.comm.Complete();

  // Each OUT report now triggers exactly one full zero-filled packet on the IN endpoint.
  f.out.HostSends(Bytes(10, 7));
  TEST_ASSERT(f.in.active && f.in.Last().length == 64);
  TEST_ASSERT(std::all_of(f.in.Last().data.begin(), f.in.Last().data.end(),
                          [](uint8_t byte) { return byte == 0U; }));
  TEST_ASSERT(f.out.active && f.out.starts.size() == 3);
  f.in.Complete();
  // A short OUT packet works the same way.
  f.out.HostSends(Bytes(5, 1));
  TEST_ASSERT(f.in.active && f.in.Last().length == 64 && f.out.active &&
              f.out.starts.size() == 4);
  f.in.Complete();

  // Clearing DTR stops the sending but keeps receiving.
  TEST_ASSERT(f.cdc.OnClassRequest(false, SET_CONTROL_LINE_STATE, 0x00, 0, 0, result) ==
              ErrorCode::OK);
  f.comm.Complete();
  TEST_ASSERT(!f.cdc.IsDtrSet());
  const size_t in_starts = f.in.starts.size();
  f.out.HostSends(Bytes(8, 2));
  TEST_ASSERT(f.in.starts.size() == in_starts && f.out.active);

  f.cdc.UnbindEndpoints(f.pool, false);
  TEST_ASSERT(f.in.closes == 1 && f.out.closes == 1 && f.comm.closes == 1);
}

void TestReadTest()
{
  CdcFixture<TestCdcRead> f;
  f.cdc.BindEndpoints(f.pool, 0, false);

  TEST_ASSERT(f.out.active && f.out.Last().length == 64);
  // Every lease is returned at once, so the receive re-arms continuously.
  for (unsigned i = 0; i < 4; ++i)
  {
    f.out.HostSends(Bytes(64, static_cast<uint8_t>(i)));
    TEST_ASSERT(f.out.active && f.out.starts.size() == 2 + i);
  }
  // The read test never drives the IN or the notification endpoint.
  TEST_ASSERT(f.in.starts.empty() && f.comm.starts.empty());
}
}  // namespace

int main()
{
  TestWriteTest();
  TestReadTest();
  std::puts(
      "cdc_test classes: lease return, DTR-triggered packet and read re-arm passed");
}
