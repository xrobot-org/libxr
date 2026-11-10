/**
 * @file test_usb_sim_profiles.cpp
 * @brief 硬件档位上的设备类 / Device classes on the hardware profiles.
 *
 * 每个档位先跑使用它的 BSP 的组合（F103 与 DevC 的 CDC，XR-DAP-S3 的 DAP v2 + CDC）：
 * 枚举、描述符里的包长不超过 FIFO/PMA、数据双向通过。再跑该档位的限制：FSDEV 不支持
 * 等时、同号 IN/OUT 类型冲突，OTG FS 的小 IN FIFO，ESP32-S3 的 TX FIFO 个数。超出限制时
 * 端点配置失败，SET_CONFIGURATION 回 STALL，设备留在地址状态，换一个合法配置仍能用。
 * 最后是帧中断：UAC 麦克风在 SET_INTERFACE 之后要靠每帧的 OnBusTime 出数据。后端在
 * 自己的重估点（Start、OnStopsSettled、DeviceCore 的 OnBusTimeNeedChanged，CH32 还有
 * OnControlReady）才看 WantsBusTime()：STM32 OTG、ESP32、CH32 风格和理想控制器上 UAC
 * 都每帧出一个包，类解除绑定后帧中断重新关闭。
 * Each profile first runs the composition of the BSP that uses it (CDC on F103 and DevC,
 * DAP v2 + CDC on XR-DAP-S3): enumeration, descriptor packet sizes within the FIFO or
 * PMA, data in both directions. Then the profile's limits: no isochronous and IN/OUT
 * type conflicts on FSDEV, a small IN FIFO on OTG FS, the TX FIFO count on ESP32-S3. Past
 * a limit the endpoint configuration fails, SET_CONFIGURATION is stalled, the device
 * stays addressed, and a legal configuration still works.
 * Last the frame interrupt: after SET_INTERFACE the UAC microphone produces data from the
 * per-frame OnBusTime. A backend looks at WantsBusTime() only at its own re-evaluation
 * points (Start, OnStopsSettled, the DeviceCore hook OnBusTimeNeedChanged, and
 * OnControlReady on CH32): on the STM32 OTG, ESP32 and CH32 style and on an ideal
 * controller the UAC sends one packet per frame, and the frame interrupt is off again
 * once the class is unbound.
 */

#include <atomic>
#include <chrono>
#include <cstdio>
#include <map>
#include <string>
#include <thread>
#include <vector>

#include "cdc_uart.hpp"
#include "daplink_v2.hpp"
#include "debug/swd_general_gpio.hpp"
#include "hid.hpp"
#include "swd_target.hpp"
#include "uac_mic.hpp"
#include "usb_sim_profiles.hpp"

using namespace LibXR;
using namespace LibXR::USB;
using namespace LibXR::USB::Sim;

namespace
{
constexpr auto LANGUAGE = DescriptorStrings::MakeLanguagePack(
    DescriptorStrings::Language::EN_US, "LibXR", "USB sim profile", "0");

using Caps = FullSpeedCapabilities;
using SimDevice = Sim::Device<Caps>;
using SimHost = Sim::Host<Caps>;
using EP = Endpoint::EPNumber;

constexpr uint8_t VENDOR_REPORT_DESC[] = {
    0x06, 0x00, 0xFF,  // Usage Page (Vendor 0xFF00)
    0x09, 0x01,        // Usage (1)
    0xA1, 0x01,        // Collection (Application)
    0x15, 0x00,        //   Logical Minimum (0)
    0x26, 0xFF, 0x00,  //   Logical Maximum (255)
    0x75, 0x08,        //   Report Size (8)
    0x95, 0x08,        //   Report Count (8)
    0x09, 0x01,        //   Usage (1)
    0x81, 0x02,        //   Input (Data, Var, Abs)
    0x95, 0x08,        //   Report Count (8)
    0x09, 0x01,        //   Usage (1)
    0x91, 0x02,        //   Output (Data, Var, Abs)
    0xC0               // End Collection
};

/// 厂商用途页的 HID，8 字节输入/输出报告 / Vendor usage page HID with 8 byte reports
class VendorHid : public HID<sizeof(VENDOR_REPORT_DESC), 8, 8>
{
 public:
  VendorHid(EP in, EP out, bool enable_out)
      : HID<sizeof(VENDOR_REPORT_DESC), 8, 8>(in, out, enable_out)
  {
  }

  ConstRawData GetReportDesc() override
  {
    return ConstRawData{VENDOR_REPORT_DESC, sizeof(VENDOR_REPORT_DESC)};
  }
};

struct EndpointInfo
{
  uint8_t attributes = 0;
  uint16_t packet = 0;
};

/// 配置描述符里的端点：地址 -> 类型与包长 / Endpoints of a configuration descriptor:
/// address -> type and packet size
std::map<uint8_t, EndpointInfo> Endpoints(const std::vector<uint8_t>& config)
{
  std::map<uint8_t, EndpointInfo> result;
  for (size_t off = 0; off + 1 < config.size() && config[off] != 0; off += config[off])
  {
    if (config[off + 1] == 5 && off + 7 <= config.size())
    {
      result[config[off + 2]] = {
          static_cast<uint8_t>(config[off + 3] & 3u),
          static_cast<uint16_t>(config[off + 4] | (config[off + 5] << 8))};
    }
  }
  return result;
}

std::vector<uint8_t> Bytes(size_t n, uint8_t seed)
{
  std::vector<uint8_t> v(n);
  for (size_t i = 0; i < n; ++i) v[i] = static_cast<uint8_t>(seed + i * 7u);
  return v;
}

void PowerUp(SimDevice& device)
{
  device.Init(false);
  device.Start(false);
  device.BusReset();
}

/// 主机写进 CDC，应用读出；应用写出，主机读回 / The host writes into the CDC and the
/// application reads it; the application writes and the host reads it back
void CdcRoundTrip(SimHost& host, CDCUart& cdc, uint8_t in_number, uint8_t out_number,
                  uint16_t in_packet, uint16_t out_packet, size_t length)
{
  const auto down = Bytes(length, 3);
  TEST_ASSERT(host.BulkOut(out_number, out_packet, down));
  std::vector<uint8_t> got(length);
  // 主机拿到 ACK 时数据未必已经进了读端口：设备的 owner 可能正被别的线程（DAP 的 Step
  // 线程）占着，处理要等它让出。
  // The data need not be in the read port when the host gets the ACK: the device's owner
  // may be held by another thread (the DAP Step thread) and the work waits for it.
  for (int i = 0; i < 2000 && cdc.read_port_->Size() != length; ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  TEST_ASSERT(cdc.read_port_->Size() == length);
  ReadOperation read;
  TEST_ASSERT(cdc.Read({got.data(), got.size()}, read) == ErrorCode::OK);
  TEST_ASSERT(got == down);

  const auto up = Bytes(length, 90);
  WriteOperation write;
  TEST_ASSERT(cdc.Write({up.data(), up.size()}, write) == ErrorCode::OK);
  std::vector<uint8_t> back;
  while (back.size() < length)
  {
    const auto part = host.BulkIn(in_number, in_packet, length - back.size());
    TEST_ASSERT(!part.empty());
    back.insert(back.end(), part.begin(), part.end());
  }
  TEST_ASSERT(back == up);
}

/// SET_CONFIGURATION，状态阶段之前推进几帧：待决停止要靠帧确认，真实主机在重试期间也
/// 一直发 SOF。
/// SET_CONFIGURATION with a few frames before the status stage: pending stops are
/// confirmed by frames, and a real host keeps sending SOFs while it retries.
void SetConfiguration(SimDevice& device, uint8_t value)
{
  device.Setup(MakeSetup(0x00, 9, value, 0, 0));
  for (int i = 0; i < 4; ++i) device.Sof();
  const InResult status = device.InToken(0);
  TEST_ASSERT(status.handshake == Handshake::ACK && status.data.empty());
}

/// 配置失败后设备留在地址状态，主机仍能取描述符 / After a refused configuration the
/// device stays addressed and still answers descriptor requests
void ExpectConfigurationRefused(SimDevice& device, SimHost& host)
{
  TEST_ASSERT(!host.Enumerate());
  TEST_ASSERT(std::string(host.last_failure) == "SET_CONFIGURATION");
  TEST_ASSERT(device.Address() == 1);
  bool ok = false;
  const auto desc = host.ControlIn(MakeSetup(0x80, 6, 0x0100, 0, 18), ok);
  TEST_ASSERT(ok && desc.size() == 18);
}

// ---------------------------------------------------------------------------
// FSDEV
// ---------------------------------------------------------------------------

/// bsp_stm32f103：一个 CDC，EP0 8 B / bsp_stm32f103: one CDC, an 8 B EP0
void TestFsdevF103Cdc()
{
  CDCUart cdc(EP::EP1, EP::EP1, EP::EP2, 128, 128, 3);
  SimDevice device(FsdevF103Profile(), DeviceDescriptor::PacketSize0::SIZE_8, 0x1209,
                   0x0010, 0x0100, {&LANGUAGE}, {{&cdc}});
  SimHost host(device, 8);
  PowerUp(device);
  TEST_ASSERT(host.Enumerate());
  auto eps = Endpoints(host.ConfigDescriptor());
  TEST_ASSERT(eps.size() == 3);
  TEST_ASSERT(eps[0x81].attributes == 2 && eps[0x81].packet == 64);
  TEST_ASSERT(eps[0x01].attributes == 2 && eps[0x01].packet == 64);
  TEST_ASSERT(eps[0x82].attributes == 3 && eps[0x82].packet <= 16);
  CdcRoundTrip(host, cdc, 1, 1, 64, 64, 100);
}

/// FSDEV 不支持等时：UAC 的配置被拒绝 / FSDEV has no isochronous endpoints: the UAC
/// configuration is refused
void TestFsdevRefusesIsochronous()
{
  UAC1MicrophoneQ<1, 16> mic(EP::EP1, 16000);
  SimDevice device(FsdevF103Profile(), DeviceDescriptor::PacketSize0::SIZE_8, 0x1209,
                   0x0011, 0x0100, {&LANGUAGE}, {{&mic}});
  SimHost host(device, 8);
  PowerUp(device);
  ExpectConfigurationRefused(device, host);
  TEST_ASSERT(device.In(1)->GetState() != Endpoint::State::IDLE);
}

/// HID 的中断 OUT 落在 CDC 批量 IN 的同一个端点号上：FSDEV 的 EP_TYPE 只有一份，配置被
/// 拒绝。换成不冲突的端点号后正常。
/// The HID interrupt OUT lands on the number of the CDC bulk IN: FSDEV has one EP_TYPE
/// per number, so the configuration is refused. With numbers that do not clash it works.
void TestFsdevSharedTypeConflict()
{
  Profile profile = FsdevF103Profile();
  profile.endpoints = {{1, Endpoint::Direction::BOTH, 128, 0, 64},
                       {2, Endpoint::Direction::BOTH, 128, 0, 64},
                       {3, Endpoint::Direction::BOTH, 128, 0, 16}};
  {
    CDCUart cdc(EP::EP1, EP::EP2, EP::EP3, 128, 128, 3);
    VendorHid hid(EP::EP2, EP::EP1, true);
    SimDevice device(profile, DeviceDescriptor::PacketSize0::SIZE_8, 0x1209, 0x0012,
                     0x0100, {&LANGUAGE}, {{&cdc, &hid}});
    SimHost host(device, 8);
    PowerUp(device);
    ExpectConfigurationRefused(device, host);
  }
  {
    CDCUart cdc(EP::EP1, EP::EP1, EP::EP3, 128, 128, 3);
    VendorHid hid(EP::EP2, EP::EP2, true);
    SimDevice device(profile, DeviceDescriptor::PacketSize0::SIZE_8, 0x1209, 0x0013,
                     0x0100, {&LANGUAGE}, {{&cdc, &hid}});
    SimHost host(device, 8);
    PowerUp(device);
    TEST_ASSERT(host.Enumerate());
    CdcRoundTrip(host, cdc, 1, 1, 64, 64, 70);
  }
}

// ---------------------------------------------------------------------------
// OTG FS
// ---------------------------------------------------------------------------

/// bsp-dev-c：一个 CDC；数据传输过程中重新配置，待决停止按帧确认
/// bsp-dev-c: one CDC; a reconfiguration in the middle of the data, with pending stops
/// confirmed by frames
void TestOtgFsDevCCdc()
{
  CDCUart cdc(EP::EP1, EP::EP1, EP::EP2, 128, 128, 3);
  SimDevice device(OtgFsDevCProfile(), DeviceDescriptor::PacketSize0::SIZE_8, 0x1209,
                   0x0020, 0x0100, {&LANGUAGE}, {{&cdc}});
  SimHost host(device, 8);
  PowerUp(device);
  TEST_ASSERT(host.Enumerate());
  auto eps = Endpoints(host.ConfigDescriptor());
  TEST_ASSERT(eps[0x81].packet == 64 && eps[0x01].packet == 64);
  TEST_ASSERT(eps[0x82].packet <= 16);
  CdcRoundTrip(host, cdc, 1, 1, 64, 64, 128);

  // 有未取走的 IN 数据时切到配置 0 再回配置 1：停止待决，SOF 推进后确认
  // Switch to configuration 0 and back with IN data not taken: the stop is pending and
  // confirmed as SOFs advance.
  const auto pending = Bytes(40, 5);
  WriteOperation write;
  TEST_ASSERT(cdc.Write({pending.data(), pending.size()}, write) == ErrorCode::OK);
  SetConfiguration(device, 0);
  SetConfiguration(device, 1);
  // 已交给端点的 40 字节随停止丢弃（CDCUart 的行为），之后的数据完整
  // The 40 bytes already handed to the endpoint are dropped with the stop (CDCUart
  // behaviour); the data after it arrives intact.
  CdcRoundTrip(host, cdc, 1, 1, 64, 64, 64);
}

/// IN FIFO 只有 32 B：CDC 数据 IN 取 32 B 包长，数据照常通过
/// An IN FIFO of only 32 B: the CDC data IN takes a 32 B packet and data still flows
void TestOtgFsSmallInFifo()
{
  Profile profile = OtgFsDevCProfile();
  profile.endpoints[1].packet_limit = 32;
  CDCUart cdc(EP::EP1, EP::EP1, EP::EP2, 128, 128, 3);
  SimDevice device(profile, DeviceDescriptor::PacketSize0::SIZE_8, 0x1209, 0x0021, 0x0100,
                   {&LANGUAGE}, {{&cdc}});
  SimHost host(device, 8);
  PowerUp(device);
  TEST_ASSERT(host.Enumerate());
  auto eps = Endpoints(host.ConfigDescriptor());
  TEST_ASSERT(eps[0x81].packet == 32 && eps[0x01].packet == 64);
  CdcRoundTrip(host, cdc, 1, 1, 32, 64, 120);
}

// ---------------------------------------------------------------------------
// ESP32-S3
// ---------------------------------------------------------------------------

using Engine = Debug::SwdGeneralGPIO<Debug::Sim::SimSwclkPin, Debug::Sim::SimSwdioPin>;

/// XR-DAP-S3：DAP v2 + CDC，DAP 命令由 Step 线程执行；SWD 是真实的位操作引擎，接仿真引脚
/// 和 SWD 目标仿真，DPIDR 来自目标
/// XR-DAP-S3: DAP v2 + CDC, DAP commands run by the Step thread; SWD is the real
/// bit-banging engine on simulated pins with the simulated SWD target behind them, and
/// the DPIDR comes from the target
void TestEsp32S3XrDap()
{
  Debug::Sim::Target target;
  Debug::Sim::SimSwclkPin swclk(target.wire);
  Debug::Sim::SimSwdioPin swdio(target.wire);
  Engine engine(swclk, swdio, 0u, Debug::SwdGpioTiming{});
  DapLinkV2Class<Engine> dap(EP::EP1, EP::EP1, engine);
  CDCUart cdc(EP::EP2, EP::EP2, EP::EP3, 1024, 1024, 8);
  SimDevice device(Esp32S3Profile(), DeviceDescriptor::PacketSize0::SIZE_64, 0x1209,
                   0x0030, 0x0100, {&LANGUAGE}, {{&dap, &cdc}});
  SimHost host(device);
  std::atomic<bool> stop{false};
  std::thread step(
      [&]
      {
        while (!stop.load()) (void)dap.Step(10);
      });
  PowerUp(device);
  TEST_ASSERT(host.Enumerate());
  auto eps = Endpoints(host.ConfigDescriptor());
  TEST_ASSERT(eps[0x81].packet == 64 && eps[0x01].packet == 64);
  TEST_ASSERT(eps[0x82].packet == 64 && eps[0x02].packet == 64);

  // DAP_Info(CAPABILITIES) / DAP_Info(CAPABILITIES)
  TEST_ASSERT(host.BulkOut(1, 64, {0x00, 0xF0}));
  const auto info = host.BulkIn(1, 64, 64);
  TEST_ASSERT(info.size() >= 3 && info[0] == 0x00 && info[1] >= 1);
  // DAP_Connect(SWD) 后读 DPIDR / DAP_Connect(SWD), then read DPIDR
  TEST_ASSERT(host.BulkOut(1, 64, {0x02, 0x01}));
  TEST_ASSERT(host.BulkIn(1, 64, 64) == std::vector<uint8_t>({0x02, 0x01}));
  TEST_ASSERT(host.BulkOut(1, 64, {0x12, 51, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}));
  TEST_ASSERT(host.BulkIn(1, 64, 64) == std::vector<uint8_t>({0x12, 0x00}));
  TEST_ASSERT(host.BulkOut(1, 64, {0x12, 16, 0x9E, 0xE7}));
  TEST_ASSERT(host.BulkIn(1, 64, 64) == std::vector<uint8_t>({0x12, 0x00}));
  TEST_ASSERT(host.BulkOut(1, 64, {0x12, 51, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}));
  TEST_ASSERT(host.BulkIn(1, 64, 64) == std::vector<uint8_t>({0x12, 0x00}));
  TEST_ASSERT(host.BulkOut(1, 64, {0x12, 8, 0x00}));
  TEST_ASSERT(host.BulkIn(1, 64, 64) == std::vector<uint8_t>({0x12, 0x00}));
  TEST_ASSERT(host.BulkOut(1, 64, {0x05, 0x00, 0x01, 0x02}));  // Transfer: DP read 0
  const auto idr = host.BulkIn(1, 64, 64);
  TEST_ASSERT(idr.size() == 7 && idr[1] == 1 && idr[2] == 1);
  const uint32_t dpidr = static_cast<uint32_t>(idr[3] | (idr[4] << 8) | (idr[5] << 16) |
                                               (static_cast<uint32_t>(idr[6]) << 24));
  TEST_ASSERT(dpidr == Debug::Sim::DebugPort::DPIDR);
  TEST_ASSERT(target.wire.protocol_errors == 0 && target.wire.contention == 0);

  CdcRoundTrip(host, cdc, 2, 2, 64, 64, 300);
  stop.store(true);
  step.join();
}

/// TX FIFO 个数：两个 CDC 占 4 个 IN（加 EP0 共 5 个）可以；再加一个 HID IN 超出，配置
/// 被拒绝。总线复位后换成合法配置仍可用。
/// The TX FIFO count: two CDCs take 4 IN endpoints (5 with EP0), which fits; one more HID
/// IN is past the limit and the configuration is refused. After a bus reset a legal
/// configuration still works.
void TestEsp32S3InEndpointLimit()
{
  Profile profile = Esp32S3Profile();
  profile.endpoints = {
      {1, Endpoint::Direction::BOTH, 512, 0}, {2, Endpoint::Direction::BOTH, 512, 0},
      {3, Endpoint::Direction::BOTH, 512, 0}, {4, Endpoint::Direction::BOTH, 512, 0},
      {5, Endpoint::Direction::BOTH, 512, 0}, {6, Endpoint::Direction::BOTH, 512, 0}};
  {
    CDCUart a(EP::EP1, EP::EP1, EP::EP2, 128, 128, 3);
    CDCUart b(EP::EP3, EP::EP3, EP::EP4, 128, 128, 3);
    SimDevice device(profile, DeviceDescriptor::PacketSize0::SIZE_64, 0x1209, 0x0031,
                     0x0100, {&LANGUAGE}, {{&a, &b}});
    SimHost host(device);
    PowerUp(device);
    TEST_ASSERT(host.Enumerate());
    CdcRoundTrip(host, b, 3, 3, 64, 64, 64);
    // 同一组端点号重新配置不再占新的 FIFO / Reconfiguring the same numbers takes no new
    // FIFO.
    SetConfiguration(device, 0);
    SetConfiguration(device, 1);
    CdcRoundTrip(host, a, 1, 1, 64, 64, 64);
  }
  {
    CDCUart a(EP::EP1, EP::EP1, EP::EP2, 128, 128, 3);
    CDCUart b(EP::EP3, EP::EP3, EP::EP4, 128, 128, 3);
    VendorHid hid(EP::EP5, EP::EP5, false);
    SimDevice device(profile, DeviceDescriptor::PacketSize0::SIZE_64, 0x1209, 0x0032,
                     0x0100, {&LANGUAGE}, {{&a, &b, &hid}});
    SimHost host(device);
    PowerUp(device);
    ExpectConfigurationRefused(device, host);
  }
}

// ---------------------------------------------------------------------------
// Frame interrupt and the UAC microphone
// ---------------------------------------------------------------------------

constexpr uint8_t UAC_AS_INTERFACE = 1;

/// 选中备用设置 1 后，每帧取一个等时 IN 包，返回取到的包长。UAC 的数据来自每帧的
/// OnBusTime：帧中断没有到核心，只有 SET_INTERFACE 时提交的那一个包。
/// After selecting alternate setting 1, take one isochronous IN packet per frame and
/// return the packet sizes. The UAC data comes from the per-frame OnBusTime: if the frame
/// interrupt does not reach the core, only the packet submitted by SET_INTERFACE exists.
std::vector<size_t> StreamPackets(SimDevice& device, SimHost& host,
                                  UAC1MicrophoneQ<1, 16>& mic, unsigned frames)
{
  const std::vector<uint8_t> pcm(32U * (frames + 8U), 0x5A);
  TEST_ASSERT(mic.WritePcm(pcm.data(), pcm.size()) == ErrorCode::OK);
  TEST_ASSERT(host.ControlOut(MakeSetup(0x01, 11, 1, UAC_AS_INTERFACE, 0)));
  std::vector<size_t> sizes;
  for (unsigned i = 0; i < frames; ++i)
  {
    const InResult in = device.InToken(1);
    if (in.handshake == Handshake::ACK) sizes.push_back(in.data.size());
    device.Sof();
  }
  return sizes;
}

/// 在给定档位上枚举 UAC 并取 frames 帧的包 / Enumerate the UAC on a profile and take
/// frames worth of packets
std::vector<size_t> UacOnProfile(const Profile& profile, unsigned frames)
{
  UAC1MicrophoneQ<1, 16> mic(EP::EP1, 16000);
  SimDevice device(profile, DeviceDescriptor::PacketSize0::SIZE_64, 0x1209, 0x0040,
                   0x0100, {&LANGUAGE}, {{&mic}});
  SimHost host(device);
  PowerUp(device);
  TEST_ASSERT(host.Enumerate());
  return StreamPackets(device, host, mic, frames);
}

/// 理想控制器（每帧都送）、STM32 OTG 与 ESP32 风格（OnBusTimeNeedChanged 重估帧中断）
/// 和 CH32 风格上，UAC 每帧一个包，包长 32 B（16 kHz 单声道 16 bit 每毫秒 16 个采样）。
/// 枚举和 SET_INTERFACE 不产生待决停止，帧中断只能靠 OnBusTimeNeedChanged 打开。
/// On an ideal controller (every frame delivered), with the STM32 OTG and ESP32 style
/// (OnBusTimeNeedChanged re-evaluates the frame interrupt) and with the CH32 style, the
/// UAC sends one packet per frame of 32 B (16 samples of 16 kHz mono 16 bit per
/// millisecond). Enumeration and SET_INTERFACE produce no pending stop, so the frame
/// interrupt can only come from OnBusTimeNeedChanged.
void TestUacStreamsWhenFramesArrive()
{
  Profile ideal;
  ideal.sof_policy = SofPolicy::ALWAYS;
  for (const Profile& profile :
       {ideal, OtgFsDevCProfile(), Esp32S3Profile(), Ch32OtgFsProfile()})
  {
    const auto sizes = UacOnProfile(profile, 12);
    TEST_ASSERT(sizes.size() == 12);
    for (size_t size : sizes) TEST_ASSERT(size == 32U);
  }
}

/// 帧中断随 UAC 的需要开关：配置完成（UAC 绑定）后打开，取消配置（UAC 解除绑定）并等
/// 待决停止确认后关闭。STM32 OTG、ESP32 和 CH32 风格都一样。
/// The frame interrupt follows the UAC's need: on once configured (the UAC is bound), off
/// again after the deconfiguration (the UAC is unbound) and once pending stops are
/// confirmed. The same on the STM32 OTG, ESP32 and CH32 style.
void TestFrameInterruptFollowsTheUac()
{
  uint16_t pid = 0x0041;
  for (const Profile& profile :
       {OtgFsDevCProfile(), Esp32S3Profile(), Ch32OtgFsProfile()})
  {
    UAC1MicrophoneQ<1, 16> mic(EP::EP1, 16000);
    SimDevice device(profile, DeviceDescriptor::PacketSize0::SIZE_64, 0x1209, pid++,
                     0x0100, {&LANGUAGE}, {{&mic}});
    SimHost host(device);
    PowerUp(device);
    TEST_ASSERT(!device.FrameInterruptOn());
    TEST_ASSERT(host.Enumerate());
    TEST_ASSERT(device.FrameInterruptOn());
    const unsigned before = device.SofDelivered();
    for (int i = 0; i < 3; ++i) device.Sof();
    TEST_ASSERT(device.SofDelivered() == before + 3);
    // 取消配置后 UAC 解除绑定，不再需要帧时间 / The deconfiguration unbinds the UAC,
    // which no longer wants bus time
    TEST_ASSERT(host.ControlOut(MakeSetup(0x00, 9, 0, 0, 0)));
    for (int i = 0; i < 4; ++i) device.Sof();
    TEST_ASSERT(!device.FrameInterruptOn());
  }
}

void RegularCases()
{
  TestFsdevF103Cdc();
  TestFsdevRefusesIsochronous();
  TestFsdevSharedTypeConflict();
  TestOtgFsDevCCdc();
  TestOtgFsSmallInFifo();
  TestEsp32S3XrDap();
  TestEsp32S3InEndpointLimit();
  TestUacStreamsWhenFramesArrive();
  TestFrameInterruptFollowsTheUac();
  std::puts("usb sim profiles: FSDEV, OTG FS and ESP32-S3 limits");
}
}  // namespace

int main()
{
  RegularCases();
  return 0;
}
