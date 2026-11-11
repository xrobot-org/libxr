/**
 * @file dap_sim_usbip.cpp
 * @brief 端到端：真实调试器 → DAP v2 类 → USB/IP → SWD 目标仿真 / End to end: a real
 *        debugger, the DAP v2 class, USB/IP and the simulated SWD target.
 *
 * DAP v2 类和真实位操作引擎 `SwdGeneralGPIO` 原样运行，引擎接仿真引脚，引脚后面是 SWD
 * 目标仿真；USB 侧是仿真后端，经 USB/IP 交给内核。主机上的 OpenOCD、pyOCD 把它当作一个
 * 普通的 CMSIS-DAP v2 探针。
 * The DAP v2 class and the real bit-banging engine `SwdGeneralGPIO` run unchanged; the
 * engine drives simulated pins with the simulated SWD target behind them, and the USB
 * side is the simulation backend handed to the kernel over USB/IP. OpenOCD and pyOCD on
 * the host see an ordinary CMSIS-DAP v2 probe.
 *
 * 用法 / Usage: dap_sim_usbip [port]，然后
 * `sudo usbip --tcp-port <port> attach -r 127.0.0.1 -b 1-1`。环境变量 SWD_WAIT_EVERY /
 * SWD_WAIT_BURST 在目标上周期性注入 WAIT。
 * Then `sudo usbip --tcp-port <port> attach -r 127.0.0.1 -b 1-1`. The environment
 * variables SWD_WAIT_EVERY / SWD_WAIT_BURST inject periodic WAITs on the target.
 */

#include <array>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <thread>

#include "automatic/driver/usb/sim/usbip_server.hpp"
#include "daplink_v2.hpp"
#include "debug/swd_general_gpio.hpp"
#include "swd_target.hpp"

using namespace LibXR;
using namespace LibXR::USB;

namespace
{
constexpr auto LANGUAGE = DescriptorStrings::MakeLanguagePack(
    DescriptorStrings::Language::EN_US, "LibXR", "USB sim CMSIS-DAP", "0002");

using Engine = Debug::SwdGeneralGPIO<Debug::Sim::SimSwclkPin, Debug::Sim::SimSwdioPin>;
}  // namespace

int main(int argc, char** argv)
{
  const uint16_t port = argc > 1 ? static_cast<uint16_t>(std::atoi(argv[1])) : 3240;

  static Debug::Sim::Target target;
  if (const char* every = std::getenv("SWD_WAIT_EVERY"))
  {
    target.dp.wait_every = static_cast<unsigned>(std::atoi(every));
    if (const char* burst = std::getenv("SWD_WAIT_BURST"))
      target.dp.wait_burst = static_cast<unsigned>(std::atoi(burst));
  }
  static Debug::Sim::SimSwclkPin swclk(target.wire);
  static Debug::Sim::SimSwdioPin swdio(target.wire);
  static Engine engine(swclk, swdio, 0u, Debug::SwdGpioTiming{});
  static DapLinkV2Class<Engine> dap(Endpoint::EPNumber::EP1, Endpoint::EPNumber::EP2,
                                    engine);

  Sim::Profile profile;
  profile.sof_policy = Sim::SofPolicyFromEnvironment();
  profile.endpoints = {{1, Endpoint::Direction::IN, 2048, 0},
                       {2, Endpoint::Direction::OUT, 2048, 0}};
  static Sim::Device<FullSpeedCapabilities> device(
      profile, DeviceDescriptor::PacketSize0::SIZE_64, 0x1209, 0x0002, 0x0100,
      {&LANGUAGE}, {{&dap}});

  // 目标侧的统计：DAP 线程在每次 Step 之后拷一份，打印线程只读这份拷贝 / Target-side
  // counters: the DAP thread copies them after every Step and the printing thread only
  // reads the copy, so no target field is read while the DAP thread may write it.
  static std::mutex statistics_lock;
  static std::array<unsigned, 5> statistics{};

  // DAP 线程：SWD 操作只在这里发生 / The DAP thread: all SWD work happens here.
  std::thread(
      []
      {
        while (true)
        {
          (void)dap.Step();
          const std::array<unsigned, 5> now = {
              target.wire.line_resets, target.wire.protocol_errors,
              target.wire.contention, target.memory.resets, target.dp.CtrlStat()};
          std::lock_guard<std::mutex> guard(statistics_lock);
          statistics = now;
        }
      })
      .detach();

  // 统计有变化时打印 / Print the counters when they change.
  std::thread(
      []
      {
        std::array<unsigned, 5> last = {~0u, ~0u, ~0u, ~0u, ~0u};
        while (true)
        {
          std::this_thread::sleep_for(std::chrono::seconds(1));
          std::array<unsigned, 5> now;
          {
            std::lock_guard<std::mutex> guard(statistics_lock);
            now = statistics;
          }
          if (now == last) continue;
          last = now;
          std::printf(
              "target: line_resets=%u protocol_errors=%u contention=%u resets=%u "
              "ctrl_stat=0x%08X\n",
              now[0], now[1], now[2], now[3], now[4]);
          std::fflush(stdout);
        }
      })
      .detach();

  Sim::UsbIpServer<FullSpeedCapabilities> server(device, port);
  server.Run();
  return 0;
}
