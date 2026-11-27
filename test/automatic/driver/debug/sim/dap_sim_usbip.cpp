/**
 * @file dap_sim_usbip.cpp
 * @brief 端到端：真实调试器 → DAP v1/v2 类 → USB/IP → SWD/JTAG 目标仿真 / End to end: a
 *        real debugger, DAP v1/v2, USB/IP and simulated SWD/JTAG targets.
 *
 * DAP 类和真实位操作引擎 `SwdGeneralGPIO` / `JtagGeneralGPIO` 原样运行，引擎接仿真引脚，
 * 引脚后面是 SWD 或 JTAG 目标仿真；USB 侧是仿真后端，经 USB/IP 交给内核。主机上的
 * OpenOCD、pyOCD、probe-rs 把它当作普通的 CMSIS-DAP v1/v2 探针。
 * The DAP classes and real bit-banging engines `SwdGeneralGPIO` / `JtagGeneralGPIO` run
 * unchanged on simulated pins behind SWD or JTAG target simulation over USB/IP.
 *
 * 用法 / Usage: dap_sim_usbip [port] [--mode=v2-swd|v1-swd|v2-jtag|v1-jtag]，不传
 * --mode 时默认 v2-swd（保持与录制脚本兼容）。环境变量 SWD_WAIT_EVERY /
 * SWD_WAIT_BURST 在目标上周期性注入 WAIT。
 */

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>

#include "daplink_v1.hpp"
#include "daplink_v2.hpp"
#include "debug/jtag_general_gpio.hpp"
#include "debug/swd_general_gpio.hpp"
#include "jtag_target.hpp"
#include "swd_target.hpp"
#include "usbip_server.hpp"

using namespace LibXR;
using namespace LibXR::USB;

namespace
{
constexpr auto LANGUAGE = DescriptorStrings::MakeLanguagePack(
    DescriptorStrings::Language::EN_US, "LibXR", "USB sim CMSIS-DAP", "0002");

using SwdEngine = Debug::SwdGeneralGPIO<Debug::Sim::SimSwclkPin, Debug::Sim::SimSwdioPin>;
using JtagEngine = Debug::JtagGeneralGPIO<Debug::Sim::SimTckPin, Debug::Sim::SimTmsPin,
                                          Debug::Sim::SimTdiPin, Debug::Sim::SimTdoPin>;

template <typename DpType>
void ApplyWaitEnv(DpType& dp)
{
  if (const char* every = std::getenv("SWD_WAIT_EVERY"))
  {
    dp.wait_every = static_cast<unsigned>(std::atoi(every));
    if (const char* burst = std::getenv("SWD_WAIT_BURST"))
    {
      dp.wait_burst = static_cast<unsigned>(std::atoi(burst));
    }
  }
}

template <typename DapType, typename TargetType>
int RunProbe(DapType& dap, TargetType& target, uint16_t port, Sim::Profile profile)
{
  Sim::ApplyStopModeFromEnv(profile);
  static Sim::SimDevice device(profile, DeviceDescriptor::PacketSize0::SIZE_64, 0x1209,
                               0x0002, 0x0100, {&LANGUAGE}, {{&dap}});

  // DAP 线程：SWD/JTAG 操作只在这里发生 / The DAP thread: all probe work runs here.
  std::thread(
      [&dap]
      {
        while (true)
        {
          (void)dap.Step();
        }
      })
      .detach();

  // 目标侧的统计有变化时打印 / Print the target-side counters when they change.
  std::thread(
      [&target]
      {
        unsigned last[5] = {~0u, ~0u, ~0u, ~0u, ~0u};
        while (true)
        {
          std::this_thread::sleep_for(std::chrono::seconds(1));
          const unsigned now[5] = {target.wire.line_resets, target.wire.protocol_errors,
                                   target.wire.contention, target.memory.resets,
                                   target.dp.CtrlStat()};
          if (std::equal(now, now + 5, last))
          {
            continue;
          }
          std::copy(now, now + 5, last);
          std::printf(
              "target: line_resets=%u protocol_errors=%u contention=%u resets=%u "
              "ctrl_stat=0x%08X\n",
              now[0], now[1], now[2], now[3], now[4]);
          std::fflush(stdout);
        }
      })
      .detach();

  Sim::UsbIpServer server(device, port);
  server.Run();
  return 0;
}
}  // namespace

int main(int argc, char** argv)
{
  std::string mode = "v2-swd";
  uint16_t port = 3240;
  for (int i = 1; i < argc; ++i)
  {
    const std::string arg = argv[i];
    if (arg.rfind("--mode=", 0) == 0)
    {
      mode = arg.substr(7);
    }
    else if (!arg.empty() && arg[0] >= '0' && arg[0] <= '9')
    {
      port = static_cast<uint16_t>(std::atoi(arg.c_str()));
    }
    else
    {
      std::fprintf(stderr,
                   "usage: dap_sim_usbip [port] "
                   "[--mode=v2-swd|v1-swd|v2-jtag|v1-jtag]\n");
      return 2;
    }
  }

  const bool is_v1 = (mode == "v1-swd" || mode == "v1-jtag");
  const bool is_jtag = (mode == "v2-jtag" || mode == "v1-jtag");
  const bool valid = (mode == "v2-swd" || mode == "v1-swd" || is_jtag);
  if (!valid)
  {
    std::fprintf(stderr, "unknown --mode=%s\n", mode.c_str());
    return 2;
  }

  static Debug::Sim::Target swd_target;
  static Debug::Sim::JtagTarget jtag_target;
  ApplyWaitEnv(swd_target.dp);
  ApplyWaitEnv(jtag_target.dp);

  static Debug::Sim::SimSwclkPin swclk(swd_target.wire);
  static Debug::Sim::SimSwdioPin swdio(swd_target.wire);
  static SwdEngine swd_engine(swclk, swdio, 0u, Debug::SwdGpioTiming{});

  static Debug::Sim::SimTckPin tck(jtag_target.wire);
  static Debug::Sim::SimTmsPin tms(jtag_target.wire);
  static Debug::Sim::SimTdiPin tdi(jtag_target.wire);
  static Debug::Sim::SimTdoPin tdo(jtag_target.wire);
  static JtagEngine jtag_engine(tck, tms, tdi, tdo, 0u);

  Sim::Profile profile;
  if (is_v1)
  {
    profile.endpoints = {{1, Endpoint::Direction::IN, 256, 0, 0, 1},
                         {2, Endpoint::Direction::OUT, 256, 0, 0, 1}};
    static DapLinkV1Class<SwdEngine> dap1(Endpoint::EPNumber::EP1,
                                          Endpoint::EPNumber::EP2, swd_engine);
    if (is_jtag)
    {
      dap1.SetJtag(&jtag_engine);
      return RunProbe(dap1, jtag_target, port, profile);
    }
    return RunProbe(dap1, swd_target, port, profile);
  }

  profile.endpoints = {{1, Endpoint::Direction::IN, 2048, 0, 0, 1},
                       {2, Endpoint::Direction::OUT, 2048, 0, 0, 1}};
  static DapLinkV2Class<SwdEngine> dap2(Endpoint::EPNumber::EP1, Endpoint::EPNumber::EP2,
                                        swd_engine);
  if (is_jtag)
  {
    dap2.SetJtag(&jtag_engine);
    return RunProbe(dap2, jtag_target, port, profile);
  }
  return RunProbe(dap2, swd_target, port, profile);
}
