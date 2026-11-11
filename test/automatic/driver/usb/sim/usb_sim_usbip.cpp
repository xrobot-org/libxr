/**
 * @file usb_sim_usbip.cpp
 * @brief 仿真设备经 USB/IP 交给内核：CDC 回环 / The simulated device handed to the kernel
 *        over USB/IP: a CDC loopback.
 *
 * 用法 / Usage: usb_sim_usbip [port]；端口默认 3290、busid 默认 sim-1（环境变量
 * USBIP_BUSID 覆盖），都不是系统 usbipd 的 3240 和 1-1；服务端启动时打印实际的端口和
 * busid。然后在同一台 机器上 `sudo usbip --tcp-port <port> attach -r 127.0.0.1 -b
 * <busid>`，内核把它枚举成 ttyACM，写进去的字节原样读回。 The port defaults to 3290 and
 * the busid to sim-1 (USBIP_BUSID overrides it), neither the 3240 nor the 1-1 of a system
 * usbipd; the server prints the port and busid it uses. Then on the same machine `sudo
 * usbip --tcp-port <port> attach -r 127.0.0.1 -b <busid>`; the kernel enumerates it as a
 * ttyACM, and the bytes written to it are read back.
 */

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
  const uint16_t port = argc > 1 ? static_cast<uint16_t>(std::atoi(argv[1])) : 3290;
  CDCUart cdc{Endpoint::EPNumber::EP1,
              Endpoint::EPNumber::EP1,
              Endpoint::EPNumber::EP2,
              1024,
              1024,
              8};
  Sim::Profile profile;
  Sim::Device<FullSpeedCapabilities> device(
      profile, DeviceDescriptor::PacketSize0::SIZE_64, 0x1209, 0x0001, 0x0100,
      {&LANGUAGE}, {{&cdc}});

  std::thread echo(
      [&cdc]
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
          if (cdc.Read({buf.data(), n}, read) != ErrorCode::OK) continue;
          size_t off = 0;
          while (off < n)
          {
            WriteOperation write;
            if (cdc.Write({buf.data() + off, n - off}, write) == ErrorCode::OK) break;
            std::this_thread::sleep_for(std::chrono::microseconds(200));
          }
        }
      });
  echo.detach();

  Sim::UsbIpServer<FullSpeedCapabilities> server(device, port);
  return server.Run();
}
