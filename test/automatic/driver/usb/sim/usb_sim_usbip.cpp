/**
 * @file usb_sim_usbip.cpp
 * @brief 仿真设备经 USB/IP 交给内核：CDC 回环 / The simulated device handed to the kernel
 *        over USB/IP: a CDC loopback
 *
 * 用法 / Usage: usb_sim_usbip [port]；然后在同一台机器上
 * `sudo usbip --tcp-port <port> attach -r 127.0.0.1 -b 1-1`，内核把它枚举成 ttyACM，
 * 写进去的字节原样读回。环境变量 USBSIM_STOP、USBSIM_PEND 见 usbip_server.hpp。
 * Then on the same machine `sudo usbip --tcp-port <port> attach -r 127.0.0.1 -b 1-1`;
 * the kernel enumerates it as a ttyACM, and the bytes written to it are read back. For
 * the environment variables USBSIM_STOP and USBSIM_PEND see usbip_server.hpp.
 */

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <thread>
#include <vector>

#include "cdc_uart.hpp"
#include "usbip_server.hpp"

using namespace LibXR;
using namespace LibXR::USB;

namespace
{
constexpr auto LANGUAGE = DescriptorStrings::MakeLanguagePack(
    DescriptorStrings::Language::EN_US, "LibXR", "USB sim CDC", "0001");
}

int main(int argc, char** argv)
{
  const uint16_t port = argc > 1 ? static_cast<uint16_t>(std::atoi(argv[1])) : 3240;
  static CDCUart cdc{Endpoint::EPNumber::EP1,
                     Endpoint::EPNumber::EP1,
                     Endpoint::EPNumber::EP2,
                     4096,
                     4096,
                     8};
  Sim::Profile profile;
  Sim::ApplyStopModeFromEnv(profile);
  static Sim::SimDevice device(profile, DeviceDescriptor::PacketSize0::SIZE_64, 0x1209,
                               0x0007, 0x0100, {&LANGUAGE}, {{&cdc}});

  std::thread(
      []
      {
        std::vector<uint8_t> buf(1024);
        while (true)
        {
          const size_t n = std::min(cdc.read_port_->Size(), buf.size());
          if (n == 0)
          {
            std::this_thread::sleep_for(std::chrono::microseconds(200));
            continue;
          }
          ReadOperation read;
          if (cdc.Read({buf.data(), n}, read) != ErrorCode::OK)
          {
            continue;
          }
          while (true)
          {
            WriteOperation write;
            if (cdc.Write({buf.data(), n}, write) == ErrorCode::OK)
            {
              break;
            }
            std::this_thread::sleep_for(std::chrono::microseconds(200));
          }
        }
      })
      .detach();

  Sim::UsbIpServer server(device, port);
  server.Run();
  return 0;
}
