/**
 * @file test_hid.cpp
 * @brief HID 设备类（槽模型）主机测试 / Host tests for the HID device class.
 *
 * 覆盖 HID 基类与 keyboard/mouse/gamepad 派生类：SendInputReport 的取槽-拷贝-提交与
 * 单条在途、OUT 报告租约与归还、解绑/复位语义、SET_IDLE/GET_IDLE、键盘 LED（OUT 报告与
 * SET_REPORT 控制数据两条路径）、以及槽装不下一个包时的配置拒绝。
 * Covers the HID base class and the keyboard/mouse/gamepad derivatives: the acquire-copy-
 * submit flow and one-report-in-flight rule of SendInputReport, the OUT report lease and
 * its return, unbind/reset semantics, SET_IDLE/GET_IDLE, the keyboard LED (both through
 * an OUT report and through the SET_REPORT control data stage) and the configuration
 * rejection when a slot cannot hold one packet.
 */

#include <algorithm>
#include <array>
#include <atomic>
#include <cstdio>
#include <cstring>
#include <thread>
#include <type_traits>
#include <vector>

#include "../../mock_endpoint.hpp"
#include "dev_core.hpp"
#include "ep_pool.hpp"
#include "hid.hpp"
#include "hid_gamepad.hpp"
#include "hid_keyboard.hpp"
#include "hid_mouse.hpp"

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
  using ControlTransferResult = DeviceClass::ControlTransferResult;
  explicit TestMouse(Endpoint::EPNumber in) : HIDMouse(in) {}
  using HIDMouse::BindEndpoints;
  using HIDMouse::OnClassRequest;
  using HIDMouse::UnbindEndpoints;
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
  using ControlTransferResult = DeviceClass::ControlTransferResult;
  explicit TestGamepad(Endpoint::EPNumber in) : HIDGamepad(in) {}
  using HIDGamepad::BindEndpoints;
  using HIDGamepad::OnClassRequest;
  using HIDGamepad::UnbindEndpoints;
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

/// SET_IDLE stores the idle rate (HID 1.11 7.2.4: duration in the high byte of wValue,
/// report ID in the low byte) and GET_IDLE returns it as one byte (HID 1.11 7.2.3).
void TestIdleRate()
{
  Fixture f;
  TestHid hid(Number::EP1, Number::EP2, false);
  hid.BindEndpoints(f.pool, 0, false);

  TestHid::ControlTransferResult result;
  auto get_idle = [&]()
  {
    result.write_data = {};
    TEST_ASSERT(hid.OnClassRequest(false,
                                   static_cast<uint8_t>(TestHid::ClassRequest::GET_IDLE),
                                   0, 1, 0, result) == ErrorCode::OK);
    TEST_ASSERT(result.write_data.size_ == 1U);
    return static_cast<const uint8_t*>(result.write_data.addr_)[0];
  };

  TEST_ASSERT(get_idle() == 0U);
  // Duration 0x7D (500 ms in 4 ms units) for all reports.
  TEST_ASSERT(hid.OnClassRequest(false,
                                 static_cast<uint8_t>(TestHid::ClassRequest::SET_IDLE),
                                 0x7D00, 0, 0, result) == ErrorCode::OK);
  TEST_ASSERT(get_idle() == 0x7DU && hid.GetIDLERate() == 0x7DU);
  // Duration 0 means indefinite.
  TEST_ASSERT(hid.OnClassRequest(false,
                                 static_cast<uint8_t>(TestHid::ClassRequest::SET_IDLE),
                                 0x0000, 0, 0, result) == ErrorCode::OK);
  TEST_ASSERT(get_idle() == 0U && hid.GetIDLERate() == 0U);
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

  // Six keys fill every slot; more than six is the phantom state: all six slots hold
  // ErrorRollOver and the modifiers are reported as usual.
  using Key = HIDKeyboard::KeyCode;
  keyboard.PressKey({Key::A, Key::B, Key::C, Key::D, Key::E, Key::F},
                    HIDKeyboard::Modifier::LEFT_CTRL, false);
  TEST_ASSERT(
      f.in.Last().data ==
      std::vector<uint8_t>({HIDKeyboard::Modifier::LEFT_CTRL, 0,
                            static_cast<uint8_t>(Key::A), static_cast<uint8_t>(Key::B),
                            static_cast<uint8_t>(Key::C), static_cast<uint8_t>(Key::D),
                            static_cast<uint8_t>(Key::E), static_cast<uint8_t>(Key::F)}));
  f.in.Complete();
  keyboard.PressKey({Key::A, Key::B, Key::C, Key::D, Key::E, Key::F, Key::G},
                    HIDKeyboard::Modifier::LEFT_SHIFT, false);
  TEST_ASSERT(f.in.Last().length == 8);
  TEST_ASSERT(f.in.Last().data == std::vector<uint8_t>({HIDKeyboard::Modifier::LEFT_SHIFT,
                                                        0, 1, 1, 1, 1, 1, 1}));
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
/// GET_REPORT as the host sends it: wValue = report type << 8 | report ID.
enum : uint8_t
{
  REPORT_INPUT = 1,
  REPORT_OUTPUT = 2,
  REPORT_FEATURE = 3
};

/// Returns the control error code; `data` receives the IN data stage when it is OK.
template <typename C>
ErrorCode GetReport(C& hid, uint8_t type, uint8_t id, std::vector<uint8_t>& data,
                    uint16_t length = 64)
{
  typename C::ControlTransferResult result{};
  const ErrorCode ans = hid.OnClassRequest(
      false, static_cast<uint8_t>(HIDKeyboard::ClassRequest::GET_REPORT),
      static_cast<uint16_t>((static_cast<uint16_t>(type) << 8) | id), length, 0, result);
  data.clear();
  if (ans == ErrorCode::OK && result.write_data.size_ != 0U)
  {
    const auto* p = static_cast<const uint8_t*>(result.write_data.addr_);
    data.assign(p, p + result.write_data.size_);
  }
  return ans;
}

/// HID 1.11 7.2.1: Get_Report is mandatory. A class answers it with the report it has;
/// a report type or ID it does not have is refused (STALL), not answered with an empty
/// data stage.
void TestGetReport()
{
  std::vector<uint8_t> data;

  // The base default has no report at all: every type stalls.
  {
    Fixture f;
    TestHid hid(Number::EP1, Number::EP2, false);
    hid.BindEndpoints(f.pool, 0, false);
    TEST_ASSERT(GetReport(hid, REPORT_INPUT, 0, data) != ErrorCode::OK);
    TEST_ASSERT(GetReport(hid, REPORT_OUTPUT, 0, data) != ErrorCode::OK);
    TEST_ASSERT(GetReport(hid, REPORT_FEATURE, 0, data) != ErrorCode::OK);
  }

  // Keyboard: input = last report sent, output = last LED report, no feature report.
  {
    Fixture f;
    TestKeyboard kbd(Number::EP1, Number::EP2, true);
    kbd.BindEndpoints(f.pool, 0, false);
    TEST_ASSERT(GetReport(kbd, REPORT_INPUT, 0, data) == ErrorCode::OK);
    TEST_ASSERT(data == std::vector<uint8_t>(8, 0));
    TEST_ASSERT(GetReport(kbd, REPORT_OUTPUT, 0, data) == ErrorCode::OK);
    TEST_ASSERT(data == std::vector<uint8_t>(1, 0));

    kbd.PressKey({HIDKeyboard::KeyCode::A}, HIDKeyboard::Modifier::LEFT_SHIFT, false);
    const std::vector<uint8_t> pressed = {
        HIDKeyboard::Modifier::LEFT_SHIFT, 0, 0x04, 0, 0, 0, 0, 0};
    TEST_ASSERT(GetReport(kbd, REPORT_INPUT, 0, data) == ErrorCode::OK);
    TEST_ASSERT(data == pressed);
    // A report that the endpoint refused (one already in flight) was never sent.
    kbd.ReleaseAll(false);
    TEST_ASSERT(GetReport(kbd, REPORT_INPUT, 0, data) == ErrorCode::OK);
    TEST_ASSERT(data == pressed);
    f.in.Complete();
    kbd.ReleaseAll(false);
    TEST_ASSERT(GetReport(kbd, REPORT_INPUT, 0, data) == ErrorCode::OK);
    TEST_ASSERT(data == std::vector<uint8_t>(8, 0));
    f.in.Complete();

    f.out.HostSends({0x03});
    TEST_ASSERT(GetReport(kbd, REPORT_OUTPUT, 0, data) == ErrorCode::OK);
    TEST_ASSERT(data == std::vector<uint8_t>({0x03}));
    // The upper three bits of the LED report are constant padding.
    f.out.HostSends({0xFF});
    TEST_ASSERT(GetReport(kbd, REPORT_OUTPUT, 0, data) == ErrorCode::OK);
    TEST_ASSERT(data == std::vector<uint8_t>({0x1F}));
    f.out.HostSends({0x03});
    // The SET_REPORT data stage updates the same state.
    std::array<uint8_t, 1> led_report{0x04};
    ConstRawData led{led_report.data(), led_report.size()};
    TEST_ASSERT(kbd.OnClassData(false,
                                static_cast<uint8_t>(TestHid::ClassRequest::SET_REPORT),
                                led) == ErrorCode::OK);
    TEST_ASSERT(GetReport(kbd, REPORT_OUTPUT, 0, data) == ErrorCode::OK);
    TEST_ASSERT(data == std::vector<uint8_t>({0x04}));

    // Nothing else exists: no feature report, no report ID, no unknown type.
    TEST_ASSERT(GetReport(kbd, REPORT_FEATURE, 0, data) != ErrorCode::OK);
    TEST_ASSERT(GetReport(kbd, REPORT_INPUT, 1, data) != ErrorCode::OK);
    TEST_ASSERT(GetReport(kbd, REPORT_OUTPUT, 1, data) != ErrorCode::OK);
    TEST_ASSERT(GetReport(kbd, 0, 0, data) != ErrorCode::OK);
    TEST_ASSERT(GetReport(kbd, 4, 0, data) != ErrorCode::OK);
  }

  // Mouse: input only.
  {
    Fixture f;
    TestMouse mouse(Number::EP1);
    mouse.BindEndpoints(f.pool, 0, false);
    TEST_ASSERT(GetReport(mouse, REPORT_INPUT, 0, data) == ErrorCode::OK);
    TEST_ASSERT(data == std::vector<uint8_t>(4, 0));
    mouse.Move(HIDMouse::Button::LEFT, 5, -3, 1, false);
    TEST_ASSERT(GetReport(mouse, REPORT_INPUT, 0, data) == ErrorCode::OK);
    TEST_ASSERT(data == std::vector<uint8_t>({0x01, 0x05, 0xFD, 0x01}));
    f.in.Complete();
    mouse.Release(false);
    TEST_ASSERT(GetReport(mouse, REPORT_INPUT, 0, data) == ErrorCode::OK);
    TEST_ASSERT(data == std::vector<uint8_t>(4, 0));
    TEST_ASSERT(GetReport(mouse, REPORT_OUTPUT, 0, data) != ErrorCode::OK);
    TEST_ASSERT(GetReport(mouse, REPORT_FEATURE, 0, data) != ErrorCode::OK);
    TEST_ASSERT(GetReport(mouse, REPORT_INPUT, 2, data) != ErrorCode::OK);
  }

  // Gamepad: input only. Before the first report it answers the neutral state the class
  // starts in (axes at mid-range, no button), 9 bytes.
  {
    Block<128> in_storage;
    EndpointPool pool{2};
    MockEndpoint in{Number::EP1, Direction::IN, in_storage.Raw()};
    TEST_ASSERT(pool.Put(&in) == ErrorCode::OK);
    TestGamepad pad(Number::EP1);
    pad.BindEndpoints(pool, 0, false);
    TEST_ASSERT(GetReport(pad, REPORT_INPUT, 0, data) == ErrorCode::OK);
    // Mid of 0..2047 is 1023 = 0x03FF, little endian.
    TEST_ASSERT(data == std::vector<uint8_t>(
                            {0xFF, 0x03, 0xFF, 0x03, 0xFF, 0x03, 0xFF, 0x03, 0x00}));
    TEST_ASSERT(pad.Send(100, 200, 300, 400, 0x81, false) == ErrorCode::OK);
    TEST_ASSERT(GetReport(pad, REPORT_INPUT, 0, data) == ErrorCode::OK);
    TEST_ASSERT(data ==
                std::vector<uint8_t>({100, 0, 200, 0, 0x2C, 0x01, 0x90, 0x01, 0x81}));
    // A report that was not sent (previous one still in flight) is not reported.
    TEST_ASSERT(pad.Send(1, 2, 3, 4, 0x00, false) == ErrorCode::BUSY);
    TEST_ASSERT(GetReport(pad, REPORT_INPUT, 0, data) == ErrorCode::OK);
    TEST_ASSERT(data ==
                std::vector<uint8_t>({100, 0, 200, 0, 0x2C, 0x01, 0x90, 0x01, 0x81}));
    in.Complete();
    TEST_ASSERT(GetReport(pad, REPORT_OUTPUT, 0, data) != ErrorCode::OK);
    TEST_ASSERT(GetReport(pad, REPORT_FEATURE, 0, data) != ErrorCode::OK);
  }
}

static constexpr auto TEST_LANGUAGE = DescriptorStrings::MakeLanguagePack(
    DescriptorStrings::Language::EN_US, "LibXR", "USB tests", "0");

/// A full-speed device core around one or two HID functions of one configuration.
class HidDevice : public DeviceCore<FullSpeedCapabilities>
{
 public:
  HidDevice(EndpointPool& pool, ConfigDescriptorItem* a)
      : DeviceCore(pool, USBSpec::USB_2_0, Speed::FULL,
                   DeviceDescriptor::PacketSize0::SIZE_64, 0x1209, 0x4321, 0x0100,
                   {&TEST_LANGUAGE}, {{a}})
  {
  }
  HidDevice(EndpointPool& pool, ConfigDescriptorItem* a, ConfigDescriptorItem* b)
      : DeviceCore(pool, USBSpec::USB_2_0, Speed::FULL,
                   DeviceDescriptor::PacketSize0::SIZE_64, 0x1209, 0x4321, 0x0100,
                   {&TEST_LANGUAGE}, {{a, b}})
  {
  }
  void Start(bool) override {}
  void Stop(bool) override {}

 protected:
  ErrorCode SetAddress(uint8_t, ControlContext) override { return ErrorCode::OK; }
  void OnStopTimeout(bool) override {}
};

/// Endpoint 0 and two interrupt IN endpoints (EP1, EP3) of a device under test.
struct HidDeviceFixture
{
  Block<64> ep0_in_storage, ep0_out_storage;
  Block<128> in1_storage, in3_storage;
  EndpointPool pool{4};
  MockEndpoint in0{Number::EP0, Direction::IN, ep0_in_storage.Raw()};
  MockEndpoint out0{Number::EP0, Direction::OUT, ep0_out_storage.Raw()};
  MockEndpoint in1{Number::EP1, Direction::IN, in1_storage.Raw()};
  MockEndpoint in3{Number::EP3, Direction::IN, in3_storage.Raw()};
  HidDeviceFixture()
  {
    TEST_ASSERT(pool.Put(&in1) == ErrorCode::OK);
    TEST_ASSERT(pool.Put(&in3) == ErrorCode::OK);
    pool.SetEndpoint0(&in0, &out0);
  }
  /// Plays one SETUP and returns the IN data stage the device started.
  template <typename Device>
  std::vector<uint8_t> Control(Device& device, uint8_t kind, uint8_t request,
                               uint16_t value, uint16_t length, uint16_t index = 0)
  {
    in0.active = false;
    out0.active = false;
    const size_t stalls_before = in0.stalls;
    const size_t starts_before = in0.starts.size();
    SetupPacket packet{kind, request, value, index, length};
    device.OnSetupPacket(true, &packet);
    stalled = in0.stalls != stalls_before;
    if (in0.starts.size() == starts_before) return {};
    return in0.Last().data;
  }
  bool stalled = false;
};

struct ClassCodes
{
  uint8_t cls, sub, proto;
  bool operator==(const ClassCodes& o) const
  {
    return cls == o.cls && sub == o.sub && proto == o.proto;
  }
};

/// bDeviceClass / bDeviceSubClass / bDeviceProtocol of the device descriptor the host
/// reads.
template <typename Device>
ClassCodes ReadDeviceCodes(HidDeviceFixture& f, Device& device)
{
  const auto desc = f.Control(device, 0x80, 6, 0x0100, 18);
  TEST_ASSERT(desc.size() == 18 && desc[0] == 18 && desc[1] == 1);
  return {desc[4], desc[5], desc[6]};
}

/// The class, subclass and protocol of every interface descriptor in the configuration
/// descriptor the host reads.
template <typename Device>
std::vector<ClassCodes> ReadInterfaceCodes(HidDeviceFixture& f, Device& device)
{
  const auto cfg = f.Control(device, 0x80, 6, 0x0200, 255);
  TEST_ASSERT(cfg.size() >= 9 && cfg[1] == 2);
  TEST_ASSERT(cfg.size() == static_cast<size_t>(cfg[2] | (cfg[3] << 8)));
  std::vector<ClassCodes> list;
  for (size_t at = cfg[0]; at + 2 <= cfg.size(); at += cfg[at])
  {
    TEST_ASSERT(cfg[at] >= 2 && at + cfg[at] <= cfg.size());
    if (cfg[at + 1] == 4) list.push_back({cfg[at + 5], cfg[at + 6], cfg[at + 7]});
  }
  return list;
}

/// HID 1.11 4.1 / 4.2: the HID class (and, for a boot device, the boot subclass and the
/// keyboard / mouse protocol) belongs to the interface descriptor; a device that is
/// nothing but HID keeps bDeviceClass 0 so the host reads the class per interface.
void TestClassCodes()
{
  // Keyboard alone: boot interface, keyboard protocol, nothing on the device.
  {
    HidDeviceFixture f;
    TestKeyboard kbd(Number::EP1, Number::EP2, false);
    HidDevice device(f.pool, &kbd);
    device.Init(false);
    TEST_ASSERT(ReadDeviceCodes(f, device) == (ClassCodes{0, 0, 0}));
    const auto itf = ReadInterfaceCodes(f, device);
    TEST_ASSERT(itf.size() == 1 && itf[0] == (ClassCodes{3, 1, 1}));
  }
  // Mouse alone: boot interface, mouse protocol.
  {
    HidDeviceFixture f;
    TestMouse mouse(Number::EP1);
    HidDevice device(f.pool, &mouse);
    device.Init(false);
    TEST_ASSERT(ReadDeviceCodes(f, device) == (ClassCodes{0, 0, 0}));
    const auto itf = ReadInterfaceCodes(f, device);
    TEST_ASSERT(itf.size() == 1 && itf[0] == (ClassCodes{3, 1, 2}));
  }
  // Gamepad alone: no boot protocol.
  {
    HidDeviceFixture f;
    TestGamepad pad(Number::EP1);
    HidDevice device(f.pool, &pad);
    device.Init(false);
    TEST_ASSERT(ReadDeviceCodes(f, device) == (ClassCodes{0, 0, 0}));
    const auto itf = ReadInterfaceCodes(f, device);
    TEST_ASSERT(itf.size() == 1 && itf[0] == (ClassCodes{3, 0, 0}));
  }
  // Keyboard and mouse in one configuration: the device descriptor is not taken over by
  // either function (it stays the default of a composite device) and each interface
  // carries its own codes.
  {
    HidDeviceFixture f;
    TestKeyboard kbd(Number::EP1, Number::EP2, false);
    TestMouse mouse(Number::EP3);
    HidDevice device(f.pool, &kbd, &mouse);
    device.Init(false);
    const ClassCodes dev = ReadDeviceCodes(f, device);
    TEST_ASSERT(dev.cls == 0xEF && dev.sub == 2 && dev.proto == 1);
    const auto itf = ReadInterfaceCodes(f, device);
    TEST_ASSERT(itf.size() == 2);
    TEST_ASSERT(itf[0] == (ClassCodes{3, 1, 1}) && itf[1] == (ClassCodes{3, 1, 2}));
  }
  // Keyboard and gamepad: only the keyboard interface is a boot interface.
  {
    HidDeviceFixture f;
    TestKeyboard kbd(Number::EP1, Number::EP2, false);
    TestGamepad pad(Number::EP3);
    HidDevice device(f.pool, &kbd, &pad);
    device.Init(false);
    const auto itf = ReadInterfaceCodes(f, device);
    TEST_ASSERT(itf.size() == 2);
    TEST_ASSERT(itf[0] == (ClassCodes{3, 1, 1}) && itf[1] == (ClassCodes{3, 0, 0}));
  }
}

/// A boot interface must honour SET_PROTOCOL / GET_PROTOCOL (HID 1.11 7.2.5, 7.2.6). Its
/// report has the same bytes in both protocols: keyboard = modifier, reserved, six keys;
/// mouse = buttons, X, Y (and a wheel byte after them), none with a report ID
/// (HID 1.11 appendix B).
void TestBootProtocol()
{
  constexpr uint8_t GET_PROTOCOL = 0x03, SET_PROTOCOL = 0x0B;
  Fixture f;
  TestKeyboard kbd(Number::EP1, Number::EP2, false);
  kbd.BindEndpoints(f.pool, 0, false);
  TestKeyboard::ControlTransferResult result;
  auto protocol = [&]()
  {
    result.write_data = {};
    TEST_ASSERT(kbd.OnClassRequest(false, GET_PROTOCOL, 0, 1, 0, result) ==
                ErrorCode::OK);
    TEST_ASSERT(result.write_data.size_ == 1U);
    return static_cast<const uint8_t*>(result.write_data.addr_)[0];
  };
  TEST_ASSERT(protocol() == 1U);  // Report protocol after enumeration.
  TEST_ASSERT(kbd.OnClassRequest(false, SET_PROTOCOL, 0, 0, 0, result) == ErrorCode::OK);
  TEST_ASSERT(protocol() == 0U);  // Boot protocol.
  kbd.PressKey({HIDKeyboard::KeyCode::A}, HIDKeyboard::Modifier::LEFT_CTRL, false);
  TEST_ASSERT(f.in.Last().data == std::vector<uint8_t>({HIDKeyboard::Modifier::LEFT_CTRL,
                                                        0, 0x04, 0, 0, 0, 0, 0}));
  TEST_ASSERT(kbd.OnClassRequest(false, SET_PROTOCOL, 1, 0, 0, result) == ErrorCode::OK);
  TEST_ASSERT(protocol() == 1U);

  Fixture g;
  TestMouse mouse(Number::EP1);
  mouse.BindEndpoints(g.pool, 0, false);
  TEST_ASSERT(mouse.OnClassRequest(false, SET_PROTOCOL, 0, 0, 0, result) ==
              ErrorCode::OK);
  result.write_data = {};
  TEST_ASSERT(mouse.OnClassRequest(false, GET_PROTOCOL, 0, 1, 0, result) ==
              ErrorCode::OK);
  TEST_ASSERT(result.write_data.size_ == 1U &&
              static_cast<const uint8_t*>(result.write_data.addr_)[0] == 0U);
  mouse.Move(HIDMouse::Button::RIGHT, 7, -8, 0, false);
  TEST_ASSERT(g.in.Last().length >= 3 &&
              g.in.Last().data == std::vector<uint8_t>({0x02, 0x07, 0xF8, 0x00}));
}

/// The sender runs in an interrupt or another thread while the owner answers GET_REPORT.
/// The reply handed to the core must be one complete report that no later send changes,
/// so the core reads the same bytes whenever it copies them out.
void TestGetReportReplyIsASnapshot()
{
  auto reply_of = [](auto& hid)
  {
    typename std::remove_reference_t<decltype(hid)>::ControlTransferResult result;
    TEST_ASSERT(hid.OnClassRequest(
                    false, static_cast<uint8_t>(HIDKeyboard::ClassRequest::GET_REPORT),
                    static_cast<uint16_t>(REPORT_INPUT << 8), 64, 0,
                    result) == ErrorCode::OK);
    return result.write_data;
  };
  auto bytes = [](const ConstRawData& data)
  {
    const auto* p = static_cast<const uint8_t*>(data.addr_);
    return std::vector<uint8_t>(p, p + data.size_);
  };

  {
    Fixture f;
    TestKeyboard kbd(Number::EP1, Number::EP2, false);
    kbd.BindEndpoints(f.pool, 0, false);
    kbd.PressKey({HIDKeyboard::KeyCode::A}, HIDKeyboard::Modifier::LEFT_CTRL, false);
    f.in.Complete();
    const auto first =
        std::vector<uint8_t>({HIDKeyboard::Modifier::LEFT_CTRL, 0, 4, 0, 0, 0, 0, 0});
    const ConstRawData reply = reply_of(kbd);
    // A send between the request and the moment the core copies the reply.
    kbd.PressKey({HIDKeyboard::KeyCode::B}, HIDKeyboard::Modifier::LEFT_ALT, false);
    f.in.Complete();
    TEST_ASSERT(bytes(reply) == first);
    TEST_ASSERT(
        bytes(reply_of(kbd)) ==
        std::vector<uint8_t>({HIDKeyboard::Modifier::LEFT_ALT, 0, 5, 0, 0, 0, 0, 0}));
  }
  {
    Fixture f;
    TestMouse mouse(Number::EP1);
    mouse.BindEndpoints(f.pool, 0, false);
    mouse.Move(HIDMouse::Button::LEFT, 5, -3, 1, false);
    f.in.Complete();
    const ConstRawData reply = reply_of(mouse);
    mouse.Move(HIDMouse::Button::RIGHT, 7, 8, 9, false);
    f.in.Complete();
    TEST_ASSERT(bytes(reply) == std::vector<uint8_t>({0x01, 0x05, 0xFD, 0x01}));
    TEST_ASSERT(bytes(reply_of(mouse)) == std::vector<uint8_t>({0x02, 7, 8, 9}));
  }
  {
    Block<128> in_storage;
    EndpointPool pool{2};
    MockEndpoint in{Number::EP1, Direction::IN, in_storage.Raw()};
    TEST_ASSERT(pool.Put(&in) == ErrorCode::OK);
    TestGamepad pad(Number::EP1);
    pad.BindEndpoints(pool, 0, false);
    TEST_ASSERT(pad.Send(100, 200, 300, 400, 0x81, false) == ErrorCode::OK);
    in.Complete();
    const ConstRawData reply = reply_of(pad);
    TEST_ASSERT(pad.Send(1, 2, 3, 4, 0x02, false) == ErrorCode::OK);
    in.Complete();
    TEST_ASSERT(bytes(reply) ==
                std::vector<uint8_t>({100, 0, 200, 0, 0x2C, 0x01, 0x90, 0x01, 0x81}));
    TEST_ASSERT(bytes(reply_of(pad)) ==
                std::vector<uint8_t>({1, 0, 2, 0, 3, 0, 4, 0, 0x02}));
  }
}

/// A sender thread keeps sending while GET_REPORT is answered: every reply is one report
/// that was sent, never a mix of two (the bytes of a report are all equal to its number).
/// Under ThreadSanitizer this is also the data race check.
void TestGetReportWhileSending()
{
  constexpr int ROUNDS = 20000;
  std::atomic<bool> done{false};
  auto run = [&](auto& hid, auto&& send, auto&& complete, auto&& uniform)
  {
    std::thread sender(
        [&]
        {
          for (int i = 0; i < ROUNDS; ++i)
          {
            send(static_cast<uint8_t>(i % 100 + 1));
            complete();
          }
          done.store(true);
        });
    std::vector<uint8_t> data;
    size_t replies = 0;
    while (!done.load())
    {
      TEST_ASSERT(GetReport(hid, REPORT_INPUT, 0, data) == ErrorCode::OK);
      TEST_ASSERT(uniform(data));
      ++replies;
    }
    sender.join();
    TEST_ASSERT(replies > 0);
    done.store(false);
  };

  {
    Fixture f;
    TestKeyboard kbd(Number::EP1, Number::EP2, false);
    kbd.BindEndpoints(f.pool, 0, false);
    run(
        kbd,
        [&](uint8_t v)
        {
          const auto k = static_cast<HIDKeyboard::KeyCode>(v);
          kbd.PressKey({k, k, k, k, k, k}, v, false);
        },
        [&] { f.in.Complete(false); },
        [](const std::vector<uint8_t>& d)
        {
          if (d.size() != 8 || d[1] != 0) return false;
          for (size_t i = 2; i < 8; ++i)
            if (d[i] != d[0]) return false;
          return true;
        });
  }
  {
    Fixture f;
    TestMouse mouse(Number::EP1);
    mouse.BindEndpoints(f.pool, 0, false);
    run(
        mouse,
        [&](uint8_t v)
        {
          mouse.Move(v, static_cast<int8_t>(v), static_cast<int8_t>(v),
                     static_cast<int8_t>(v), false);
        },
        [&] { f.in.Complete(false); }, [](const std::vector<uint8_t>& d)
        { return d.size() == 4 && d[1] == d[0] && d[2] == d[0] && d[3] == d[0]; });
  }
  {
    Block<128> in_storage;
    EndpointPool pool{2};
    MockEndpoint in{Number::EP1, Direction::IN, in_storage.Raw()};
    TEST_ASSERT(pool.Put(&in) == ErrorCode::OK);
    TestGamepad pad(Number::EP1);
    pad.BindEndpoints(pool, 0, false);
    run(
        pad, [&](uint8_t v) { (void)pad.Send(v, v, v, v, v, false); },
        [&] { in.Complete(false); },
        [](const std::vector<uint8_t>& d)
        {
          if (d.size() != 9) return false;
          if (d[0] == 0xFF && d[1] == 0x03) return d[8] == 0;  // The neutral state.
          for (size_t i = 0; i < 8; i += 2)
            if (d[i] != d[8] || d[i + 1] != 0) return false;
          return true;
        });
  }
}

/// HID 1.11 7.2.6: SET_PROTOCOL carries 0 (boot) or 1 (report) in wValue. Any other
/// value is refused and the protocol the interface is in stays. A class without a boot
/// protocol keeps answering the request, with the same rule for the value.
void TestSetProtocolValue()
{
  constexpr uint8_t GET_PROTOCOL = 0x03, SET_PROTOCOL = 0x0B;
  Fixture f;
  TestKeyboard kbd(Number::EP1, Number::EP2, false);
  kbd.BindEndpoints(f.pool, 0, false);
  Block<128> in_storage;
  EndpointPool pool{2};
  MockEndpoint in{Number::EP1, Direction::IN, in_storage.Raw()};
  TEST_ASSERT(pool.Put(&in) == ErrorCode::OK);
  TestGamepad pad(Number::EP1);
  pad.BindEndpoints(pool, 0, false);

  auto check = [&](auto& hid)
  {
    typename std::remove_reference_t<decltype(hid)>::ControlTransferResult result;
    auto protocol = [&]()
    {
      result.write_data = {};
      TEST_ASSERT(hid.OnClassRequest(false, GET_PROTOCOL, 0, 1, 0, result) ==
                  ErrorCode::OK);
      TEST_ASSERT(result.write_data.size_ == 1U);
      return static_cast<const uint8_t*>(result.write_data.addr_)[0];
    };
    TEST_ASSERT(protocol() == 1U);
    for (uint16_t bad : {2, 3, 0x0100, 0x0101, 0x8000, 0xFFFF})
    {
      TEST_ASSERT(hid.OnClassRequest(false, SET_PROTOCOL, bad, 0, 0, result) !=
                  ErrorCode::OK);
      TEST_ASSERT(protocol() == 1U);
    }
    TEST_ASSERT(hid.OnClassRequest(false, SET_PROTOCOL, 0, 0, 0, result) ==
                ErrorCode::OK);
    TEST_ASSERT(protocol() == 0U);
    for (uint16_t bad : {2, 0x0100, 0x0101, 0xFFFF})
    {
      TEST_ASSERT(hid.OnClassRequest(false, SET_PROTOCOL, bad, 0, 0, result) !=
                  ErrorCode::OK);
      TEST_ASSERT(protocol() == 0U);  // A refused value does not reset it either.
    }
    TEST_ASSERT(hid.OnClassRequest(false, SET_PROTOCOL, 1, 0, 0, result) ==
                ErrorCode::OK);
    TEST_ASSERT(protocol() == 1U);
  };
  check(kbd);
  check(pad);

  // End to end: the control pipe stalls for the bad value and answers the good one.
  HidDeviceFixture d;
  TestKeyboard dev_kbd(Number::EP1, Number::EP2, false);
  HidDevice device(d.pool, &dev_kbd);
  device.Init(false);
  (void)d.Control(device, 0x00, 9, 1, 0);  // SET_CONFIGURATION 1
  (void)d.Control(device, 0x21, SET_PROTOCOL, 2, 0, 0);
  TEST_ASSERT(d.stalled);
  (void)d.Control(device, 0x21, SET_PROTOCOL, 0, 0, 0);
  TEST_ASSERT(!d.stalled);
  auto data = d.Control(device, 0xA1, GET_PROTOCOL, 0, 1, 0);
  TEST_ASSERT(!d.stalled && data == std::vector<uint8_t>({0}));
  (void)d.Control(device, 0x21, SET_PROTOCOL, 0x0100, 0, 0);
  TEST_ASSERT(d.stalled);
  data = d.Control(device, 0xA1, GET_PROTOCOL, 0, 1, 0);
  TEST_ASSERT(!d.stalled && data == std::vector<uint8_t>({0}));
}

/// One Input main item of a report descriptor with the globals and locals in effect.
struct InputItem
{
  uint32_t flags;
  int32_t logical_min, logical_max;
  uint32_t usage_min, usage_max;
  uint32_t size, count;
};

/// Walks the short items of a report descriptor the way a host parser does: Logical
/// Minimum / Maximum are signed two's complement of the item's own size
/// (HID 1.11 6.2.2.7), Usage Minimum / Maximum are unsigned.
std::vector<InputItem> ParseInputItems(const uint8_t* desc, size_t length)
{
  std::vector<InputItem> items;
  InputItem now{};
  for (size_t at = 0; at < length;)
  {
    const uint8_t prefix = desc[at];
    const size_t size = (prefix & 3U) == 3U ? 4U : (prefix & 3U);
    const uint8_t type = (prefix >> 2) & 3U;
    const uint8_t tag = prefix >> 4;
    TEST_ASSERT(at + 1 + size <= length);
    uint32_t value = 0;
    for (size_t i = 0; i < size; ++i)
      value |= static_cast<uint32_t>(desc[at + 1 + i]) << (8 * i);
    int32_t sign = static_cast<int32_t>(value);
    if (size == 1U) sign = static_cast<int8_t>(value);
    if (size == 2U) sign = static_cast<int16_t>(value);
    if (type == 1U)  // Global
    {
      if (tag == 1U) now.logical_min = sign;
      if (tag == 2U) now.logical_max = sign;
      if (tag == 7U) now.size = value;
      if (tag == 9U) now.count = value;
    }
    else if (type == 2U)  // Local
    {
      if (tag == 1U) now.usage_min = value;
      if (tag == 2U) now.usage_max = value;
    }
    else if (type == 0U && tag == 8U)  // Main: Input
    {
      now.flags = value;
      items.push_back(now);
    }
    if (type == 0U)
    {
      now.usage_min = 0;
      now.usage_max = 0;
    }
    at += 1 + size;
  }
  return items;
}

/// The key array of the keyboard report must cover every usage the class can send: the
/// Keyboard/Keypad page up to ExSel (0xA4), which holds F13-F24 (0x68-0x73). The Logical
/// Maximum 0xA4 does not fit a signed byte (it would read as -92), so it takes two bytes.
void TestKeyboardReportDescriptorRange()
{
  const auto items =
      ParseInputItems(HID_KEYBOARD_REPORT_DESC, sizeof(HID_KEYBOARD_REPORT_DESC));
  // Modifier bits, reserved byte, key array.
  TEST_ASSERT(items.size() == 3);
  const InputItem& keys = items[2];
  TEST_ASSERT((keys.flags & 0x03U) == 0U);  // Data, Array
  TEST_ASSERT(keys.size == 8 && keys.count == 6);
  TEST_ASSERT(keys.logical_min == 0 && keys.logical_max == 0xA4);
  TEST_ASSERT(keys.usage_min == 0 && keys.usage_max == 0xA4);

  using Key = HIDKeyboard::KeyCode;
  for (Key key : {Key::A, Key::F12, Key::F13, Key::F20, Key::F24, Key::EXECUTE, Key::MUTE,
                  Key::LANG9, Key::CRSEL_PROPS, Key::EXSEL})
  {
    const int value = static_cast<uint8_t>(key);
    TEST_ASSERT(value >= keys.logical_min && value <= keys.logical_max);
    TEST_ASSERT(static_cast<uint32_t>(value) >= keys.usage_min &&
                static_cast<uint32_t>(value) <= keys.usage_max);
  }

  // F13-F24 go out as sent.
  Fixture f;
  TestKeyboard kbd(Number::EP1, Number::EP2, false);
  kbd.BindEndpoints(f.pool, 0, false);
  kbd.PressKey({Key::F13, Key::F24}, 0, false);
  TEST_ASSERT(f.in.Last().data == std::vector<uint8_t>({0, 0, 0x68, 0x73, 0, 0, 0, 0}));
  f.in.Complete();
}

/// End to end through the device core: after SET_CONFIGURATION the host's GET_REPORT is
/// answered with the report, or the control pipe stalls for one the class does not have.
void TestGetReportThroughDeviceCore()
{
  HidDeviceFixture f;
  TestKeyboard kbd(Number::EP1, Number::EP2, false);
  HidDevice device(f.pool, &kbd);
  device.Init(false);
  (void)f.Control(device, 0x00, 9, 1, 0);  // SET_CONFIGURATION 1
  TEST_ASSERT(!f.stalled);

  // GET_REPORT(Input, ID 0), wLength 8: the last report, here still all zero.
  auto data = f.Control(device, 0xA1, 0x01, 0x0100, 8, 0);
  TEST_ASSERT(!f.stalled && data == std::vector<uint8_t>(8, 0));
  // GET_REPORT(Output, ID 0), wLength 1: the LED report.
  data = f.Control(device, 0xA1, 0x01, 0x0200, 1, 0);
  TEST_ASSERT(!f.stalled && data == std::vector<uint8_t>(1, 0));
  // A report the keyboard does not have stalls the control pipe.
  (void)f.Control(device, 0xA1, 0x01, 0x0300, 8, 0);  // Feature
  TEST_ASSERT(f.stalled);
  (void)f.Control(device, 0xA1, 0x01, 0x0101, 8, 0);  // Input, ID 1
  TEST_ASSERT(f.stalled);
}

}  // namespace

int main()
{
  TestSendInputReport();
  TestInputReportAcrossReset();
  TestOutReportLease();
  TestOutResumeRearms();
  TestIdleRate();
  TestConfigurationRejectedWhenSlotTooSmall();
  TestKeyboardReports();
  TestMouseAndGamepad();
  TestGetReport();
  TestClassCodes();
  TestBootProtocol();
  TestSetProtocolValue();
  TestGetReportReplyIsASnapshot();
  TestGetReportWhileSending();
  TestKeyboardReportDescriptorRange();
  TestGetReportThroughDeviceCore();
  std::puts("HID class: slot model, OUT lease, reset, idle rate and LED passed");
}
