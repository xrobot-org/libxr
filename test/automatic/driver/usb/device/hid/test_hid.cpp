/**
 * @file test_hid.cpp
 * @brief HID 设备类（槽模型）主机测试 / Host tests for the HID device class.
 *
 * 覆盖 HID 基类与 keyboard/mouse/gamepad 派生类：SendInputReport 的取槽-拷贝-提交与
 * 单条在途、OUT 报告租约与归还、解绑/复位语义、空闲重发、键盘 LED（OUT 报告与
 * SET_REPORT 控制数据两条路径）、以及槽装不下一个包时的配置拒绝。
 * Covers the HID base class and the keyboard/mouse/gamepad derivatives: the acquire-copy-
 * submit flow and one-report-in-flight rule of SendInputReport, the OUT report lease and
 * its return, unbind/reset semantics, idle resend, the keyboard LED (both through an OUT
 * report and through the SET_REPORT control data stage) and the configuration rejection
 * when a slot cannot hold one packet.
 */

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <thread>
#include <vector>

#include "../../mock_endpoint.hpp"
#include "hid.hpp"
#include "hid_gamepad.hpp"
#include "hid_keyboard.hpp"
#include "hid_mouse.hpp"
#include "timebase.hpp"

using namespace LibXR;
using namespace LibXR::USB;
using namespace LibXR::USB::Test;

namespace
{
static constexpr uint8_t TEST_REPORT_DESC[8] = {0x05, 0x01, 0x09, 0x02,
                                                0xA1, 0x01, 0xC0, 0x00};

using TestHidBase = HID<sizeof(TEST_REPORT_DESC), 8, 2>;

/// Concrete HID with 8-byte input and 2-byte output reports.
class TestHid : public TestHidBase
{
 public:
  using ControlTransferResult = DeviceClass::ControlTransferResult;

  TestHid(Endpoint::EPNumber in, Endpoint::EPNumber out, bool enable_out)
      : TestHidBase(in, out, enable_out)
  {
  }

  using TestHidBase::BindEndpoints;
  using TestHidBase::GetIDLERate;
  using TestHidBase::GetInEndpoint;
  using TestHidBase::GetOutEndpoint;
  using TestHidBase::LastInputReport;
  using TestHidBase::OnClassData;
  using TestHidBase::OnClassRequest;
  using TestHidBase::SendInputReport;
  using TestHidBase::UnbindEndpoints;

  ConstRawData GetReportDesc() override
  {
    return ConstRawData{TEST_REPORT_DESC, sizeof(TEST_REPORT_DESC)};
  }

  size_t in_completions = 0;
  ErrorCode last_in_result = ErrorCode::OK;
  size_t out_reports = 0;
  std::vector<uint8_t> last_out;

 protected:
  void OnDataInComplete(bool, const Endpoint::Completion& completion) override
  {
    ++in_completions;
    last_in_result = completion.result;
  }
  void OnOutReport(bool, const Endpoint::Completion& completion) override
  {
    ++out_reports;
    const auto* data = static_cast<const uint8_t*>(completion.slot.data.addr_);
    last_out.assign(data, data + completion.slot.data.size_);
  }
};

class TestMouse : public HIDMouse
{
 public:
  explicit TestMouse(Endpoint::EPNumber in) : HIDMouse(in) {}
  using HID<sizeof(HID_MOUSE_REPORT_DESC), 4, 0>::BindEndpoints;
  using HID<sizeof(HID_MOUSE_REPORT_DESC), 4, 0>::UnbindEndpoints;
};

class TestKeyboard : public HIDKeyboard
{
 public:
  using ControlTransferResult = DeviceClass::ControlTransferResult;

  TestKeyboard(Endpoint::EPNumber in, Endpoint::EPNumber out, bool enable_out)
      : HIDKeyboard(in, out, enable_out)
  {
  }
  using HIDKeyboard::BindEndpoints;
  using HIDKeyboard::GetInEndpoint;
  using HIDKeyboard::GetOutEndpoint;
  using HIDKeyboard::OnClassData;
  using HIDKeyboard::OnClassRequest;
  using HIDKeyboard::UnbindEndpoints;
};

class TestGamepad : public HIDGamepad
{
 public:
  explicit TestGamepad(Endpoint::EPNumber in) : HIDGamepad(in) {}
  using HID<50, 9, 0>::BindEndpoints;
  using HID<50, 9, 0>::UnbindEndpoints;
};

void TestSendInputReport()
{
  Fixture f;
  TestHid hid(Number::EP1, Number::EP2, false);
  hid.BindEndpoints(f.pool, 0, false);
  TEST_ASSERT(hid.GetInEndpoint() == &f.in && hid.GetOutEndpoint() == nullptr);
  TEST_ASSERT(f.in.MaxPacketSize() == 8 && f.in.GetGranted().slot_capacity == 64);
  TEST_ASSERT(f.in.starts.empty());

  // Bad arguments never consume the one-in-flight flag.
  TEST_ASSERT(hid.SendInputReport({nullptr, 8}, false) == ErrorCode::ARG_ERR);
  std::array<uint8_t, 8> report{};
  TEST_ASSERT(hid.SendInputReport({report.data(), 0}, false) == ErrorCode::ARG_ERR);
  TEST_ASSERT(hid.SendInputReport({report.data(), 9}, false) == ErrorCode::ARG_ERR);

  for (size_t i = 0; i < report.size(); ++i) report[i] = static_cast<uint8_t>(0x40 + i);
  TEST_ASSERT(hid.SendInputReport({report.data(), report.size()}, false) ==
              ErrorCode::OK);
  TEST_ASSERT(f.in.active && f.in.Last().length == 8);
  TEST_ASSERT(std::equal(report.begin(), report.end(), f.in.Last().data.begin()));
  // One report in flight: the next submission is BUSY, not queued.
  TEST_ASSERT(hid.SendInputReport({report.data(), report.size()}, false) ==
              ErrorCode::BUSY);
  TEST_ASSERT(f.in.starts.size() == 1);
  // The class keeps the published report for the idle resend.
  TEST_ASSERT(hid.LastInputReport().size_ == 8 &&
              std::equal(report.begin(), report.end(),
                         static_cast<const uint8_t*>(hid.LastInputReport().addr_)));

  f.in.Complete();
  TEST_ASSERT(hid.in_completions == 1 && hid.last_in_result == ErrorCode::OK);
  TEST_ASSERT(!f.in.active);
  // The completion frees the writer.
  TEST_ASSERT(hid.SendInputReport({report.data(), report.size()}, false) ==
              ErrorCode::OK);
  f.in.Complete();
  TEST_ASSERT(hid.in_completions == 2);
}

void TestInputReportAcrossReset()
{
  Fixture f;
  TestHid hid(Number::EP1, Number::EP2, false);
  hid.BindEndpoints(f.pool, 0, false);
  std::array<uint8_t, 8> report{};
  report[0] = 0x11;
  TEST_ASSERT(hid.SendInputReport({report.data(), report.size()}, false) ==
              ErrorCode::OK);
  TEST_ASSERT(f.in.active);

  // A bus reset closes the endpoint: the in-flight report ends ABORTED and the writer is
  // freed (the host never received it).
  f.in.Close(false);
  TEST_ASSERT(hid.in_completions == 1 && hid.last_in_result == ErrorCode::ABORTED);
  TEST_ASSERT(hid.SendInputReport({report.data(), report.size()}, false) ==
              ErrorCode::FAILED);
  TEST_ASSERT(f.in.starts.size() == 1);

  // After the unbind the doorbell is gone: FAILED without touching the endpoint.
  hid.UnbindEndpoints(f.pool, false);
  TEST_ASSERT(hid.SendInputReport({report.data(), report.size()}, false) ==
              ErrorCode::FAILED);
}

void TestOutReportLease()
{
  Fixture f;
  TestHid hid(Number::EP1, Number::EP2, true);
  hid.BindEndpoints(f.pool, 0, false);
  TEST_ASSERT(hid.GetOutEndpoint() == &f.out);
  // One slot armed, one report per lease.
  TEST_ASSERT(f.out.GetGranted().slot_capacity == 64);
  TEST_ASSERT(f.out.active && f.out.Last().length == 2 && f.out.starts.size() == 1);

  f.out.HostSends({0x5A, 0xA5});
  TEST_ASSERT(hid.out_reports == 1 && hid.last_out == std::vector<uint8_t>({0x5A, 0xA5}));
  // The base returns the lease, which rearms the receive.
  TEST_ASSERT(f.out.active && f.out.starts.size() == 2 && f.out.Last().length == 2);

  // A second report arrives on the rearmed slot.
  f.out.HostSends({0x01, 0x02});
  TEST_ASSERT(hid.out_reports == 2 && hid.last_out == std::vector<uint8_t>({0x01, 0x02}));
  TEST_ASSERT(f.out.active && f.out.starts.size() == 3);
}

void TestOutResumeRearms()
{
  Fixture f;
  TestHid hid(Number::EP1, Number::EP2, true);
  hid.BindEndpoints(f.pool, 0, false);
  // An aborted receive (here: a flush) is not rearmed by the base.
  f.out.Flush(false);
  TEST_ASSERT(hid.out_reports == 0 && !f.out.active);
  // ENDPOINT_RESUMED for the OUT endpoint arms it again.
  hid.OnDeviceEvent(true, DeviceEvent::ENDPOINT_RESUMED,
                    static_cast<uint8_t>(f.out.GetAddress()));
  TEST_ASSERT(f.out.active && f.out.Last().length == 2);
  // A resume for somebody else's endpoint does nothing.
  const size_t starts = f.out.starts.size();
  hid.OnDeviceEvent(true, DeviceEvent::ENDPOINT_RESUMED,
                    static_cast<uint8_t>(f.comm.GetAddress()));
  TEST_ASSERT(f.out.starts.size() == starts);
}

void TestIdleResend()
{
  Fixture f;
  TestHid hid(Number::EP1, Number::EP2, false);
  hid.BindEndpoints(f.pool, 0, false);

  // SET_IDLE with idle rate 1 (units of 4 ms), report id 0.
  TestHid::ControlTransferResult result;
  TEST_ASSERT(hid.OnClassRequest(false,
                                 static_cast<uint8_t>(TestHid::ClassRequest::SET_IDLE),
                                 0x0100, 0, 0, result) == ErrorCode::OK);
  TEST_ASSERT(hid.GetIDLERate() == 1);

  std::array<uint8_t, 8> report{};
  report[0] = 0x7E;
  TEST_ASSERT(hid.SendInputReport({report.data(), report.size()}, false) ==
              ErrorCode::OK);
  f.in.Complete();
  TEST_ASSERT(hid.in_completions == 1);

  // Nothing to resend before the idle interval elapsed.
  hid.OnBusTime(false, BusTime{});
  TEST_ASSERT(f.in.starts.size() == 1);
  std::this_thread::sleep_for(std::chrono::milliseconds(6));
  hid.OnBusTime(false, BusTime{});
  TEST_ASSERT(f.in.active && f.in.starts.size() == 2);
  // While the resend is in flight a further tick is BUSY and starts nothing.
  hid.OnBusTime(false, BusTime{});
  TEST_ASSERT(f.in.starts.size() == 2);
  f.in.Complete();
  TEST_ASSERT(hid.in_completions == 2);
}

void TestConfigurationRejectedWhenSlotTooSmall()
{
  Fixture f;
  f.in.limit = 0;  // The backend can hold no legal packet on this endpoint.
  TestHid hid(Number::EP1, Number::EP2, false);
  hid.BindEndpoints(f.pool, 0, false);
  // Configure refused the size: the endpoint is in ERROR and no report can be sent.
  TEST_ASSERT(f.in.GetState() == State::ERROR);
  std::array<uint8_t, 8> report{};
  TEST_ASSERT(hid.SendInputReport({report.data(), report.size()}, false) ==
              ErrorCode::FAILED);
  TEST_ASSERT(f.in.starts.empty());
}

void TestKeyboardReports()
{
  Fixture f;
  TestKeyboard keyboard(Number::EP1, Number::EP2, true);
  keyboard.BindEndpoints(f.pool, 0, false);
  TEST_ASSERT(keyboard.GetInEndpoint() == &f.in && keyboard.GetOutEndpoint() == &f.out);
  TEST_ASSERT(f.in.MaxPacketSize() == 8);
  // One byte of LED report per lease.
  TEST_ASSERT(f.out.active && f.out.Last().length == 1);

  struct LedCapture
  {
    bool called = false;
    bool num = false, caps = false, scroll = false;
  } led;
  keyboard.SetOnLedChangeCallback(Callback<bool, bool, bool>::Create(
      [](bool, LedCapture* self, bool n, bool c, bool s)
      {
        self->called = true;
        self->num = n;
        self->caps = c;
        self->scroll = s;
      },
      &led));

  // Path 1: an LED report on the OUT endpoint.
  f.out.HostSends({0x03});  // NumLock + CapsLock
  TEST_ASSERT(led.called && led.num && led.caps && !led.scroll);
  TEST_ASSERT(keyboard.GetNumLock() && keyboard.GetCapsLock() &&
              !keyboard.GetScrollLock());
  TEST_ASSERT(f.out.active && f.out.starts.size() == 2);

  // Path 2: the SET_REPORT OUT data stage handed to the class by the core.
  led.called = false;
  std::array<uint8_t, 1> led_report{0x04};  // ScrollLock
  ConstRawData data{led_report.data(), led_report.size()};
  TEST_ASSERT(keyboard.OnClassData(
                  false, static_cast<uint8_t>(TestHid::ClassRequest::SET_REPORT), data) ==
              ErrorCode::OK);
  TEST_ASSERT(led.called && !led.num && !led.caps && led.scroll);
  TEST_ASSERT(keyboard.GetScrollLock() && !keyboard.GetNumLock());

  // SET_REPORT without a data stage is refused.
  TestKeyboard::ControlTransferResult result;
  TEST_ASSERT(keyboard.OnClassRequest(
                  false, static_cast<uint8_t>(TestHid::ClassRequest::SET_REPORT), 0x0200,
                  0, 0, result) == ErrorCode::ARG_ERR);

  // Input reports: press and release.
  keyboard.PressKey({HIDKeyboard::KeyCode::A, HIDKeyboard::KeyCode::B},
                    HIDKeyboard::Modifier::LEFT_SHIFT, false);
  TEST_ASSERT(f.in.active && f.in.Last().length == 8);
  const auto pressed = f.in.Last().data;
  TEST_ASSERT(pressed[0] == HIDKeyboard::Modifier::LEFT_SHIFT && pressed[1] == 0 &&
              pressed[2] == static_cast<uint8_t>(HIDKeyboard::KeyCode::A) &&
              pressed[3] == static_cast<uint8_t>(HIDKeyboard::KeyCode::B));
  f.in.Complete();
  keyboard.ReleaseAll(false);
  TEST_ASSERT(f.in.Last().data == std::vector<uint8_t>(8, 0));
  f.in.Complete();
}

void TestMouseAndGamepad()
{
  Fixture f;
  TestMouse mouse(Number::EP1);
  mouse.BindEndpoints(f.pool, 0, false);
  TEST_ASSERT(f.in.MaxPacketSize() == 4);
  mouse.Move(HIDMouse::Button::LEFT, 5, -3, 1, false);
  TEST_ASSERT(f.in.active && f.in.Last().length == 4);
  TEST_ASSERT(f.in.Last().data == std::vector<uint8_t>({0x01, 0x05, 0xFD, 0x01}));
  f.in.Complete();
  mouse.Release(false);
  TEST_ASSERT(f.in.Last().data == std::vector<uint8_t>({0, 0, 0, 0}));
  f.in.Complete();

  // The gamepad has no OUT endpoint and reports its own status.
  Block<128> in_storage;
  EndpointPool pool{2};
  MockEndpoint in{Number::EP1, Direction::IN, in_storage.Raw()};
  TEST_ASSERT(pool.Put(&in) == ErrorCode::OK);
  TestGamepad pad(Number::EP1);
  pad.BindEndpoints(pool, 0, false);
  TEST_ASSERT(in.MaxPacketSize() == 9);
  TEST_ASSERT(pad.Send(100, -100, 0, 0, 0x03, false) == ErrorCode::OK);
  TEST_ASSERT(in.active && in.Last().length == 9);
  TEST_ASSERT(pad.Send(0, 0, 0, 0, 0, false) == ErrorCode::BUSY);
  in.Complete();
  TEST_ASSERT(pad.SendButtons(0x00, false) == ErrorCode::OK);
  in.Complete();
}
}  // namespace

int main()
{
  TestSendInputReport();
  TestInputReportAcrossReset();
  TestOutReportLease();
  TestOutResumeRearms();
  TestIdleResend();
  TestConfigurationRejectedWhenSlotTooSmall();
  TestKeyboardReports();
  TestMouseAndGamepad();
  std::puts("HID class: slot model, OUT lease, reset, idle resend and LED passed");
}
