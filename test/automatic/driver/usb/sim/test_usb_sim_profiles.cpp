/**
 * @file test_usb_sim_profiles.cpp
 * @brief 硬件档位上的真实设备类 / Real device classes on the hardware profiles
 *
 * 每个用例在一个档位上跑真实的设备类：限制以内照常工作（包长按 FIFO 或 PMA 缩小，描述
 * 符跟着变），越过限制时 CheckHardware 拒绝、SET_CONFIGURATION 被 STALL、设备保持已
 * 编址。还有 TX FIFO 的占用在 Configure 当场判定（配置推迟到停止结束也不会超额）、高速
 * 下两个请求挂在硬件上的 CDC，以及 ESP32-S3 布局上的 DAP v2 加 CDC。
 * Every case runs real device classes on one profile: within the limits they work (the
 * packet size shrinks to the FIFO or PMA and the descriptor follows), past a limit
 * CheckHardware refuses, SET_CONFIGURATION is stalled and the device stays addressed.
 * Also the TX FIFO holding decided at Configure (no overcommit even when the
 * configuration waits for the end of a stop), CDC at high speed with two requests on
 * the hardware, and DAP v2 with CDC on the ESP32-S3 layout.
 */

#include <cstdio>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include "cdc_uart.hpp"
#include "daplink_v2.hpp"
#include "debug/swd_general_gpio.hpp"
#include "sim_probe.hpp"
#include "swd_target.hpp"
#include "uac_mic.hpp"
#include "usb_sim.hpp"
#include "usb_sim_profiles.hpp"

using namespace LibXR;
using namespace LibXR::USB;
using namespace LibXR::USB::Sim;

namespace
{
constexpr auto LANGUAGE = DescriptorStrings::MakeLanguagePack(
    DescriptorStrings::Language::EN_US, "LibXR", "USB sim profile", "0003");

using Dir = Endpoint::Direction;
using Type = Endpoint::Type;
using EP = Endpoint::EPNumber;
using Bytes = std::vector<uint8_t>;

/// 配置描述符里某个端点的包长 / Packet size of one endpoint in the configuration
/// descriptor
uint16_t PacketOf(const Bytes& config, uint8_t addr)
{
  for (size_t off = 0; off + 7U <= config.size() && config[off] != 0U; off += config[off])
  {
    if (config[off + 1U] == 5U && config[off + 2U] == addr)
    {
      return static_cast<uint16_t>((config[off + 4U] | (config[off + 5U] << 8U)) &
                                   0x7FFU);
    }
  }
  return 0U;
}

/// 主机把 data 写进 CDC，设备读出后原样写回，主机读回 / The host writes data to CDC, the
/// device reads it and writes it back, and the host reads it back
void CdcRoundTrip(Host& host, CDCUart& cdc, uint8_t out_ep, uint8_t in_ep,
                  uint16_t out_packet, uint16_t in_packet, const Bytes& data)
{
  TEST_ASSERT(host.BulkOut(out_ep, out_packet, data));
  for (int i = 0; i < 1000 && cdc.read_port_->Size() < data.size(); ++i)
  {
    std::this_thread::yield();
  }
  TEST_ASSERT(cdc.read_port_->Size() == data.size());
  Bytes got(data.size());
  ReadOperation read;
  TEST_ASSERT(cdc.Read({got.data(), got.size()}, read) == ErrorCode::OK);
  TEST_ASSERT(got == data);
  WriteOperation write;
  TEST_ASSERT(cdc.Write({got.data(), got.size()}, write) == ErrorCode::OK);
  Bytes back;
  for (int i = 0; i < 100 && back.size() < data.size(); ++i)
  {
    const Bytes part = host.BulkIn(in_ep, in_packet, data.size() - back.size());
    back.insert(back.end(), part.begin(), part.end());
  }
  TEST_ASSERT(back == data);
}

Bytes Pattern(size_t n, uint8_t seed)
{
  Bytes b(n);
  for (size_t i = 0; i < n; ++i)
  {
    b[i] = static_cast<uint8_t>(seed + i * 13U);
  }
  return b;
}

/// 枚举到 SET_CONFIGURATION 之前 / Enumerate up to SET_CONFIGURATION
void Address(SimDevice& dev, Host& host)
{
  dev.Init(false);
  dev.Start(false);
  TEST_ASSERT(host.Enumerate(1, 0));
}

// ---------------------------------------------------------------------------

/// FSDEV F103：CDC，通知端点按 16 B 的 PMA 缩小 / FSDEV F103: CDC, the notification
/// endpoint shrinks to its 16 B of PMA
void TestFsdevF103Cdc()
{
  CDCUart cdc(EP::EP1, EP::EP1, EP::EP2, 256, 256, 3);
  SimDevice dev(FsdevF103Profile(), DeviceDescriptor::PacketSize0::SIZE_64, 0x1209,
                0x0003, 0x0100, {&LANGUAGE}, {{&cdc}});
  Host host(dev);
  Address(dev, host);
  TEST_ASSERT(PacketOf(host.ConfigDescriptor(), 0x82) == 16U);
  TEST_ASSERT(PacketOf(host.ConfigDescriptor(), 0x81) == 64U);
  TEST_ASSERT(host.SetConfiguration(1));
  CdcRoundTrip(host, cdc, 1, 1, 64, 64, Pattern(150, 1));
}

/// FSDEV：等时端点占用整个端点号，另一方向在用时拒绝 / FSDEV: an isochronous endpoint
/// takes the whole number and is refused while the other direction is in use
void TestFsdevIsoTakesNumber()
{
  {
    UAC1MicrophoneQ<1, 16> mic(EP::EP1, 16000);
    // 另一方向也是等时：类型一致，只有“等时占用整个端点号”拒绝它。
    // The other direction is isochronous too: the types agree and only "an isochronous
    // endpoint takes the whole number" refuses it.
    Probe other({{1, Dir::OUT, Type::ISOCHRONOUS, 64, 1, 1}});
    SimDevice dev(FsdevF103Profile(), DeviceDescriptor::PacketSize0::SIZE_64, 0x1209,
                  0x0003, 0x0100, {&LANGUAGE}, {{&other, &mic}});
    Host host(dev);
    Address(dev, host);
    // 麦克风在绑定时就配置等时端点，这时 EP1 OUT 已在用 / The microphone configures
    // its isochronous endpoint at bind, while EP1 OUT is in use.
    TEST_ASSERT(!host.SetConfiguration(1));
    TEST_ASSERT(dev.Address() == 1U);
    TEST_ASSERT(dev.Out(1)->GetState() == Endpoint::State::DISABLED);
  }
  {
    UAC1MicrophoneQ<1, 16> mic(EP::EP1, 16000);
    SimDevice dev(FsdevF103Profile(), DeviceDescriptor::PacketSize0::SIZE_64, 0x1209,
                  0x0003, 0x0100, {&LANGUAGE}, {{&mic}});
    Host host(dev);
    Address(dev, host);
    TEST_ASSERT(host.SetConfiguration(1));
    TEST_ASSERT(host.SetInterface(1, 1));
    TEST_ASSERT(dev.In(1)->GetState() == Endpoint::State::IDLE);
  }
}

/// FSDEV：同号两个方向类型不同时 SET_CONFIGURATION 被 STALL / FSDEV: SET_CONFIGURATION
/// is stalled when both directions of a number differ in type
void TestFsdevSharedTypeConflict()
{
  {
    Probe a({{1, Dir::IN, Type::INTR, 64, 1, 1}});
    Probe b({{1, Dir::OUT, Type::BULK, 64, 1, 0}});
    SimDevice dev(FsdevF103Profile(), DeviceDescriptor::PacketSize0::SIZE_64, 0x1209,
                  0x0003, 0x0100, {&LANGUAGE}, {{&a, &b}});
    Host host(dev);
    Address(dev, host);
    TEST_ASSERT(!host.SetConfiguration(1));
    TEST_ASSERT(dev.Address() == 1U);
    TEST_ASSERT(dev.In(1)->GetState() == Endpoint::State::DISABLED);
    TEST_ASSERT(dev.Out(1)->GetState() == Endpoint::State::DISABLED);
  }
  {
    Probe a({{1, Dir::IN, Type::BULK, 64, 1, 0}});
    Probe b({{1, Dir::OUT, Type::BULK, 64, 1, 0}});
    SimDevice dev(FsdevF103Profile(), DeviceDescriptor::PacketSize0::SIZE_64, 0x1209,
                  0x0003, 0x0100, {&LANGUAGE}, {{&a, &b}});
    Host host(dev);
    Address(dev, host);
    TEST_ASSERT(host.SetConfiguration(1));
  }
}

/// OTG FS DevC：IN FIFO 只有 32 B 时批量 IN 包长缩到 32，传输照常 / OTG FS DevC: with a
/// 32 B IN FIFO the bulk IN packet shrinks to 32 and transfers work
void TestOtgFsSmallInFifo()
{
  Profile profile = OtgFsDevCProfile();
  profile.endpoints[1].packet_limit = 32;
  CDCUart cdc(EP::EP1, EP::EP1, EP::EP2, 256, 256, 3);
  SimDevice dev(profile, DeviceDescriptor::PacketSize0::SIZE_64, 0x1209, 0x0003, 0x0100,
                {&LANGUAGE}, {{&cdc}});
  Host host(dev);
  Address(dev, host);
  TEST_ASSERT(PacketOf(host.ConfigDescriptor(), 0x81) == 32U);
  TEST_ASSERT(PacketOf(host.ConfigDescriptor(), 0x01) == 64U);
  TEST_ASSERT(host.SetConfiguration(1));
  CdcRoundTrip(host, cdc, 1, 1, 64, 32, Pattern(200, 2));
}

/// TX FIFO：占用保持到总线复位；Configure 当场判定，推迟的配置也不会超额 / TX FIFOs:
/// held until a bus reset; Configure decides at once, and a deferred configuration
/// never overcommits
void TestTxFifoHolding()
{
  Profile profile;
  profile.in_endpoint_limit = 3;  // EP0 加两个 / EP0 plus two
  profile.stop_mode = StopMode::FRAMES;
  profile.endpoints = {{1, Dir::BOTH, 1024, 0, 0, 1},
                       {2, Dir::BOTH, 1024, 0, 0, 1},
                       {3, Dir::BOTH, 1024, 0, 0, 1}};
  Probe two({{1, Dir::IN, Type::BULK, 64, 1, 0}, {2, Dir::IN, Type::BULK, 64, 1, 0}});
  Probe other({{3, Dir::IN, Type::BULK, 64, 1, 0}});
  Probe three({{1, Dir::IN, Type::BULK, 64, 1, 0},
               {2, Dir::IN, Type::BULK, 64, 1, 0},
               {3, Dir::IN, Type::BULK, 64, 1, 0}});
  SimDevice dev(profile, DeviceDescriptor::PacketSize0::SIZE_64, 0x1209, 0x0003, 0x0100,
                {&LANGUAGE}, {{&two}, {&other}, {&three}});
  Host host(dev);
  host.auto_frames = false;
  Address(dev, host);
  TEST_ASSERT(!host.SetConfiguration(3));
  TEST_ASSERT(host.SetConfiguration(1));
  TEST_ASSERT(two.Send(0, Pattern(16, 1)) == ErrorCode::OK);
  // 配置 2：EP1、EP2 的 FIFO 还占着（停止也未结束）/ Configuration 2: the FIFOs of EP1
  // and EP2 are still held (and the stop has not ended).
  TEST_ASSERT(!host.SetConfiguration(2));
  dev.Sof();
  dev.Sof();
  TEST_ASSERT(!host.SetConfiguration(2));
  // 总线复位放掉 FIFO / A bus reset frees the FIFOs.
  host.auto_frames = true;
  TEST_ASSERT(host.Enumerate(1, 2));
  TEST_ASSERT(other.Send(0, Pattern(16, 2)) == ErrorCode::OK);
  TEST_ASSERT(host.In(3).data == Pattern(16, 2));
}

/// HPM 高速：CDC 批量包长 512，两个请求挂在硬件上 / HPM high speed: CDC bulk packets of
/// 512 with two requests on the hardware
void TestHpmHighSpeedCdc()
{
  CDCUart cdc(EP::EP1, EP::EP1, EP::EP3, 8192, 8192, 8);
  SimDevice dev(HpmHighSpeedProfile(), DeviceDescriptor::PacketSize0::SIZE_64, 0x1209,
                0x0003, 0x0100, {&LANGUAGE}, {{&cdc}}, Speed::HIGH);
  Host host(dev);
  dev.Init(false);
  dev.Start(false);
  TEST_ASSERT(host.Enumerate(1, 1, Speed::HIGH));
  TEST_ASSERT(PacketOf(host.ConfigDescriptor(), 0x81) == 512U);
  TEST_ASSERT(dev.Out(1)->OnHardware() == 2U);
  CdcRoundTrip(host, cdc, 1, 1, 512, 512, Pattern(5000, 3));
}

/// ESP32-S3 布局：DAP v2 加 CDC，DAP_Info 和一次 SWD 读 IDCODE / The ESP32-S3 layout:
/// DAP v2 with CDC, DAP_Info and one SWD read of IDCODE
void TestEsp32S3DapAndCdc()
{
  using Engine = Debug::SwdGeneralGPIO<Debug::Sim::SimSwclkPin, Debug::Sim::SimSwdioPin>;
  Debug::Sim::Target target;
  Debug::Sim::SimSwclkPin swclk(target.wire);
  Debug::Sim::SimSwdioPin swdio(target.wire);
  Engine engine(swclk, swdio, 0U, Debug::SwdGpioTiming{});
  DapLinkV2Class<Engine> dap(EP::EP1, EP::EP1, engine);
  CDCUart cdc(EP::EP2, EP::EP2, EP::EP3, 512, 512, 4);
  SimDevice dev(Esp32S3Profile(), DeviceDescriptor::PacketSize0::SIZE_64, 0x1209, 0x0002,
                0x0100, {&LANGUAGE}, {{&dap, &cdc}});
  Host host(dev);
  std::atomic<bool> run{true};
  std::thread step(
      [&]
      {
        while (run)
        {
          (void)dap.Step(5);
        }
      });
  dev.Init(false);
  dev.Start(false);
  TEST_ASSERT(host.Enumerate());
  // DAP_Info(0xF0：包长) / DAP_Info(0xF0: packet size)
  TEST_ASSERT(host.BulkOut(1, 64, {0x00, 0xFF}, false));
  Bytes info;
  for (int i = 0; i < 100 && info.empty(); ++i)
  {
    info = host.BulkIn(1, 64, 512);
  }
  TEST_ASSERT(info.size() >= 2U && info[0] == 0x00);
  // DAP_Connect(SWD)、SWJ_Sequence 线复位、读 DP IDCODE / DAP_Connect(SWD), a
  // SWJ_Sequence line reset, read DP IDCODE.
  TEST_ASSERT(host.BulkOut(1, 64, {0x02, 0x01}, false));
  info.clear();
  for (int i = 0; i < 100 && info.empty(); ++i)
  {
    info = host.BulkIn(1, 64, 512);
  }
  TEST_ASSERT(info.size() >= 2U && info[0] == 0x02 && info[1] == 0x01);
  TEST_ASSERT(
      host.BulkOut(1, 64, {0x12, 0x38, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, false));
  info.clear();
  for (int i = 0; i < 100 && info.empty(); ++i)
  {
    info = host.BulkIn(1, 64, 512);
  }
  TEST_ASSERT(info.size() >= 2U && info[0] == 0x12 && info[1] == 0x00);
  TEST_ASSERT(host.BulkOut(1, 64, {0x12, 0x10, 0x00, 0x00}, false));
  info.clear();
  for (int i = 0; i < 100 && info.empty(); ++i)
  {
    info = host.BulkIn(1, 64, 512);
  }
  TEST_ASSERT(host.BulkOut(1, 64, {0x05, 0x00, 0x01, 0x02}, false));
  info.clear();
  for (int i = 0; i < 100 && info.empty(); ++i)
  {
    info = host.BulkIn(1, 64, 512);
  }
  // 响应：0x05、次数 1、ACK OK、IDCODE / Response: 0x05, count 1, ACK OK, IDCODE.
  TEST_ASSERT(info.size() >= 7U && info[0] == 0x05 && info[1] == 0x01 && info[2] == 0x01);
  uint32_t idcode = 0;
  std::memcpy(&idcode, &info[3], 4);
  TEST_ASSERT(idcode == Debug::Sim::DebugPort::DPIDR);
  CdcRoundTrip(host, cdc, 2, 2, 64, 64, Pattern(100, 4));
  run = false;
  step.join();
}

struct Case
{
  const char* name;
  void (*run)();
};

const Case CASES[] = {
    {"fsdev_f103_cdc", TestFsdevF103Cdc},
    {"fsdev_iso_takes_number", TestFsdevIsoTakesNumber},
    {"fsdev_shared_type", TestFsdevSharedTypeConflict},
    {"otg_fs_small_in_fifo", TestOtgFsSmallInFifo},
    {"tx_fifo_holding", TestTxFifoHolding},
    {"hpm_hs_cdc", TestHpmHighSpeedCdc},
    {"esp32s3_dap_cdc", TestEsp32S3DapAndCdc},
};

}  // namespace

int main(int argc, char** argv)
{
  const std::string only = argc > 1 ? argv[1] : "";
  unsigned ran = 0;
  for (const Case& c : CASES)
  {
    if (!only.empty() && only != c.name)
    {
      continue;
    }
    std::printf("usb_sim_profiles: %s\n", c.name);
    std::fflush(stdout);
    c.run();
    ++ran;
  }
  TEST_ASSERT(ran != 0U);
  std::printf("usb_sim_profiles: %u cases passed\n", ran);
  return 0;
}
