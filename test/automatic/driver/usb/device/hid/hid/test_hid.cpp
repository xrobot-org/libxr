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
#include <cstdio>
#include <cstring>
#include <vector>

#include "driver/usb/usb_test_common.hpp"
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

/// 具体的 HID，输入报告 8 字节，输出报告 2 字节
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
  using TestHidBase::ClaimInput;
  using TestHidBase::DropInputClaim;
  using TestHidBase::GetIDLERate;
  using TestHidBase::GetInEndpoint;
  using TestHidBase::GetOutEndpoint;
  using TestHidBase::OnClassData;
  using TestHidBase::OnClassRequest;
  using TestHidBase::SendClaimedReport;
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

  // 非法参数从不占用单条在途标志。
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
  // 已有一个报告在途：下一次提交返回 BUSY，而不是排队。
  // One report in flight: the next submission is BUSY, not queued.
  TEST_ASSERT(hid.SendInputReport({report.data(), report.size()}, false) ==
              ErrorCode::BUSY);
  TEST_ASSERT(f.in.starts.size() == 1);

  f.in.Complete();
  TEST_ASSERT(hid.in_completions == 1 && hid.last_in_result == ErrorCode::OK);
  TEST_ASSERT(!f.in.active);
  // 完成事件释放发送方 / The completion frees the writer.
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

  // 总线复位关闭端点：在途报告以 ABORTED 结束，发送方被释放（主机从未收到它）。
  // A bus reset closes the endpoint: the in-flight report ends ABORTED and the writer is
  // freed (the host never received it).
  f.in.Close(false);
  TEST_ASSERT(hid.in_completions == 1 && hid.last_in_result == ErrorCode::ABORTED);
  TEST_ASSERT(hid.SendInputReport({report.data(), report.size()}, false) ==
              ErrorCode::FAILED);
  TEST_ASSERT(f.in.starts.size() == 1);

  // 解绑之后门铃已失效：返回 FAILED，且不访问端点。
  // After the unbind the doorbell is gone: FAILED without touching the endpoint.
  hid.UnbindEndpoints(f.pool, false);
  TEST_ASSERT(hid.SendInputReport({report.data(), report.size()}, false) ==
              ErrorCode::FAILED);
}

/// 一次发送在取得声明之后被解绑打断，端点随后重新绑定并由另一次发送占用。旧的发送失败时
/// 只放弃自己的声明，新报告的在途标志保持不变。测试分步调用 SendInputReport 的两个阶段
/// 来构造这个顺序。
/// A send is interrupted by an unbind after it took the claim; the endpoint is bound
/// again and another send takes it. When the old send fails it drops only its own claim
/// and the in-flight flag of the new report stays. The test calls the two stages of
/// SendInputReport separately to build this order.
void TestInterruptedSendKeepsNewClaim()
{
  Fixture f;
  TestHid hid(Number::EP1, Number::EP2, false);
  hid.BindEndpoints(f.pool, 0, false);
  std::array<uint8_t, 8> report{};

  uint32_t old_claim = 0;
  TEST_ASSERT(hid.ClaimInput(old_claim));
  TEST_ASSERT(hid.SendInputReport({report.data(), report.size()}, false) ==
              ErrorCode::BUSY);

  hid.UnbindEndpoints(f.pool, false);
  hid.BindEndpoints(f.pool, 0, false);
  TEST_ASSERT(hid.SendInputReport({report.data(), report.size()}, false) ==
              ErrorCode::OK);
  TEST_ASSERT(f.in.active && f.in.starts.size() == 1);

  // 旧的发送此时失败并放弃声明：新报告仍在途，下一次发送返回 BUSY。
  // The old send fails now and drops its claim: the new report is still in flight and
  // the next send is BUSY.
  hid.DropInputClaim(old_claim);
  TEST_ASSERT(hid.SendInputReport({report.data(), report.size()}, false) ==
              ErrorCode::BUSY);

  f.in.Complete();
  TEST_ASSERT(hid.SendInputReport({report.data(), report.size()}, false) ==
              ErrorCode::OK);
  f.in.Complete();
}

/// 取得声明之后发生过解绑的发送不会在新的绑定里提交报告：它返回 FAILED 并归还取到的槽。
/// A send whose claim an unbind has ended does not submit a report in the new binding:
/// it returns FAILED and gives the slot back.
void TestInterruptedSendDoesNotSubmit()
{
  Fixture f;
  TestHid hid(Number::EP1, Number::EP2, false);
  hid.BindEndpoints(f.pool, 0, false);
  std::array<uint8_t, 8> report{};

  uint32_t old_claim = 0;
  TEST_ASSERT(hid.ClaimInput(old_claim));
  hid.UnbindEndpoints(f.pool, false);
  hid.BindEndpoints(f.pool, 0, false);

  TEST_ASSERT(hid.SendClaimedReport(old_claim, {report.data(), report.size()}, false) ==
              ErrorCode::FAILED);
  TEST_ASSERT(f.in.starts.empty());

  // 两个槽都还可用，在途标志空闲 / Both slots are still usable and the flag is free.
  Endpoint::Slot first;
  Endpoint::Slot second;
  TEST_ASSERT(f.in.AcquireSlot(first, false) == ErrorCode::OK);
  TEST_ASSERT(f.in.AcquireSlot(second, false) == ErrorCode::OK);
  TEST_ASSERT(f.in.ReleaseSlot(first, false) == ErrorCode::OK);
  TEST_ASSERT(f.in.ReleaseSlot(second, false) == ErrorCode::OK);
  TEST_ASSERT(hid.SendInputReport({report.data(), report.size()}, false) ==
              ErrorCode::OK);
  f.in.Complete();
  TEST_ASSERT(hid.in_completions == 1 && hid.last_in_result == ErrorCode::OK);
}

void TestOutReportLease()
{
  Fixture f;
  TestHid hid(Number::EP1, Number::EP2, true);
  hid.BindEndpoints(f.pool, 0, false);
  TEST_ASSERT(hid.GetOutEndpoint() == &f.out);
  // 挂出一个槽，每个租约一个报告。
  // One slot armed, one report per lease.
  TEST_ASSERT(f.out.GetGranted().slot_capacity == 64);
  TEST_ASSERT(f.out.active && f.out.Last().length == 2 && f.out.starts.size() == 1);

  f.out.HostSends({0x5A, 0xA5});
  TEST_ASSERT(hid.out_reports == 1 && hid.last_out == std::vector<uint8_t>({0x5A, 0xA5}));
  // 基类归还租约，从而重新挂接收。
  // The base returns the lease, which rearms the receive.
  TEST_ASSERT(f.out.active && f.out.starts.size() == 2 && f.out.Last().length == 2);

  // 第二个报告到达已重新挂包的槽。
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
  // 被中止的接收（这里是 flush）不会被基类重新挂包。
  // An aborted receive (here: a flush) is not rearmed by the base.
  f.out.Flush(false);
  TEST_ASSERT(hid.out_reports == 0 && !f.out.active);
  // OUT 端点的 ENDPOINT_RESUMED 使其重新挂包。
  // ENDPOINT_RESUMED for the OUT endpoint arms it again.
  hid.OnDeviceEvent(true, DeviceEvent::ENDPOINT_RESUMED,
                    static_cast<uint8_t>(f.out.GetAddress()));
  TEST_ASSERT(f.out.active && f.out.Last().length == 2);
  // 针对其他端点的 resume 不产生任何影响。
  // A resume for somebody else's endpoint does nothing.
  const size_t starts = f.out.starts.size();
  hid.OnDeviceEvent(true, DeviceEvent::ENDPOINT_RESUMED,
                    static_cast<uint8_t>(f.comm.GetAddress()));
  TEST_ASSERT(f.out.starts.size() == starts);
}

/// SET_IDLE 保存空闲速率（HID 1.11 7.2.4：duration 在 wValue 高字节，report ID 在低字节）
/// ，GET_IDLE 以一个字节返回它（HID 1.11 7.2.3）。
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
  // 所有报告的 duration 为 0x7D（以 4 ms 为单位即 500 ms）。
  // Duration 0x7D (500 ms in 4 ms units) for all reports.
  TEST_ASSERT(hid.OnClassRequest(false,
                                 static_cast<uint8_t>(TestHid::ClassRequest::SET_IDLE),
                                 0x7D00, 0, 0, result) == ErrorCode::OK);
  TEST_ASSERT(get_idle() == 0x7DU && hid.GetIDLERate() == 0x7DU);
  // duration 为 0 表示无限期 / Duration 0 means indefinite.
  TEST_ASSERT(hid.OnClassRequest(false,
                                 static_cast<uint8_t>(TestHid::ClassRequest::SET_IDLE),
                                 0x0000, 0, 0, result) == ErrorCode::OK);
  TEST_ASSERT(get_idle() == 0U && hid.GetIDLERate() == 0U);
}

void TestConfigurationRejectedWhenSlotTooSmall()
{
  Fixture f;
  // 后端在该端点上放不下任何合法的包。
  // The backend can hold no legal packet on this endpoint.
  f.in.limit = 0;
  TestHid hid(Number::EP1, Number::EP2, false);
  hid.BindEndpoints(f.pool, 0, false);
  // Configure 拒绝了该大小：端点处于 ERROR，无法发送任何报告。
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
  // 每个租约一个字节的 LED 报告。
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

  // 路径 1：OUT 端点上的 LED 报告。
  // Path 1: an LED report on the OUT endpoint.
  f.out.HostSends({0x03});  // NumLock + CapsLock
  TEST_ASSERT(led.called && led.num && led.caps && !led.scroll);
  TEST_ASSERT(keyboard.GetNumLock() && keyboard.GetCapsLock() &&
              !keyboard.GetScrollLock());
  TEST_ASSERT(f.out.active && f.out.starts.size() == 2);

  // 路径 2：核心交给类的 SET_REPORT OUT 数据阶段。
  // Path 2: the SET_REPORT OUT data stage handed to the class by the core.
  led.called = false;
  std::array<uint8_t, 1> led_report{0x04};  // ScrollLock
  ConstRawData data{led_report.data(), led_report.size()};
  TEST_ASSERT(keyboard.OnClassData(
                  false, static_cast<uint8_t>(TestHid::ClassRequest::SET_REPORT), data) ==
              ErrorCode::OK);
  TEST_ASSERT(led.called && !led.num && !led.caps && led.scroll);
  TEST_ASSERT(keyboard.GetScrollLock() && !keyboard.GetNumLock());

  // 没有数据阶段的 SET_REPORT 被拒绝 / SET_REPORT without a data stage is refused.
  TestKeyboard::ControlTransferResult result;
  TEST_ASSERT(keyboard.OnClassRequest(
                  false, static_cast<uint8_t>(TestHid::ClassRequest::SET_REPORT), 0x0200,
                  0, 0, result) == ErrorCode::ARG_ERR);

  // 输入报告：按下与释放 / Input reports: press and release.
  keyboard.PressKey({HIDKeyboard::KeyCode::A, HIDKeyboard::KeyCode::B},
                    HIDKeyboard::Modifier::LEFT_SHIFT, false);
  TEST_ASSERT(f.in.active && f.in.Last().length == 8);
  const auto pressed = f.in.Last().data;
  TEST_ASSERT(pressed[0] == HIDKeyboard::Modifier::LEFT_SHIFT && pressed[1] == 0 &&
              pressed[2] == static_cast<uint8_t>(HIDKeyboard::KeyCode::A) &&
              pressed[3] == static_cast<uint8_t>(HIDKeyboard::KeyCode::B));
  f.in.Complete();

  // 六个键填满所有键位；超过六个是幻影状态：六个键位都填 ErrorRollOver，修饰键照常上报。
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

  // 游戏手柄没有 OUT 端点，并报告自己的状态。
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

void test_usb_hid()
{
  TestSendInputReport();
  TestInputReportAcrossReset();
  TestInterruptedSendKeepsNewClaim();
  TestInterruptedSendDoesNotSubmit();
  TestOutReportLease();
  TestOutResumeRearms();
  TestIdleRate();
  TestConfigurationRejectedWhenSlotTooSmall();
  TestKeyboardReports();
  TestMouseAndGamepad();
}
