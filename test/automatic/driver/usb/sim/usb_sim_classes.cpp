/**
 * @file usb_sim_classes.cpp
 * @brief 仿真设备经 USB/IP 交给内核：HID、gs_usb、DFU / The simulated device handed to
 *        the kernel over USB/IP: HID, gs_usb, DFU.
 *
 * 用法 / Usage: usb_sim_classes <hid|gsusb|dfu> [port]，然后
 * `sudo usbip --tcp-port <port> attach -r 127.0.0.1 -b 1-1`。
 * - hid：厂商自定义用法页，8 字节输入/输出报告；主机写入的输出报告按位取反后作为输入
 *   报告回传（hidraw）。不用键盘、鼠标，免得往主机注入真实输入。
 *   hid: vendor-defined usage page, 8-byte input/output reports; an output report from
 *   the host comes back bitwise inverted as an input report (hidraw). No keyboard or
 *   mouse, so nothing is injected into the host's input.
 * - gsusb：一个 CAN 通道，VID:PID 1D50:606F（内核 gs_usb 驱动绑定）。设备发出的每一帧
 *   在另一个线程里以 ID+1 收回来，模拟总线上另一个节点的应答。
 *   gsusb: one CAN channel, VID:PID 1D50:606F (bound by the kernel gs_usb driver).
 *   Every frame the device sends comes back from another thread with ID+1, standing in
 *   for another node on the bus.
 * - dfu：DFU bootloader，128 KiB 镜像区（含 seal）在 RAM 里（dfu-util 下载、上传）。
 *   dfu: DFU bootloader, a 128 KiB image area in RAM (dfu-util download, upload).
 */

#include <array>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "can.hpp"
#include "dfu_bootloader.hpp"
#include "flash.hpp"
#include "gs_usb.hpp"
#include "hid.hpp"
#include "uac_mic.hpp"
#include "usbip_server.hpp"

using namespace LibXR;
using namespace LibXR::USB;

namespace
{
using Caps = FullSpeedCapabilities;

// ---------------------------------------------------------------------------
// HID
// ---------------------------------------------------------------------------

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

using VendorHidBase = HID<sizeof(VENDOR_REPORT_DESC), 8, 8>;

class VendorHid : public VendorHidBase
{
 public:
  VendorHid() : VendorHidBase(Endpoint::EPNumber::EP1, Endpoint::EPNumber::EP1, true) {}

  ConstRawData GetReportDesc() override
  {
    return ConstRawData{VENDOR_REPORT_DESC, sizeof(VENDOR_REPORT_DESC)};
  }

 protected:
  void OnOutReport(bool in_isr, const Endpoint::Completion& completion) override
  {
    const auto* data = static_cast<const uint8_t*>(completion.slot.data.addr_);
    uint8_t reply[8] = {};
    for (size_t i = 0; i < 8 && i < completion.actual; ++i) reply[i] = ~data[i];
    (void)SendInputReport(ConstRawData{reply, sizeof(reply)}, in_isr);
  }
};

// ---------------------------------------------------------------------------
// gs_usb
// ---------------------------------------------------------------------------

/// 回环 CAN：发出的帧由另一个线程以 ID+1 收回 / Loopback CAN: a sent frame comes back
/// from another thread with ID+1
class LoopbackCan : public CAN
{
 public:
  LoopbackCan()
  {
    std::thread(
        [this]
        {
          while (true)
          {
            ClassicPack pack{};
            {
              std::unique_lock<std::mutex> lock(mutex_);
              cv_.wait(lock, [this] { return !pending_.empty(); });
              pack = pending_.front();
              pending_.pop_front();
            }
            std::this_thread::sleep_for(std::chrono::microseconds(200));
            pack.id =
                (pack.id + 1u) & (pack.type == Type::EXTENDED ? 0x1FFFFFFFu : 0x7FFu);
            OnMessage(pack, false);
          }
        })
        .detach();
  }

  ErrorCode SetConfig(const CAN::Configuration&) override { return ErrorCode::OK; }
  uint32_t GetClockFreq() const override { return 48000000u; }
  ErrorCode AddMessage(const ClassicPack& pack) override
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      pending_.push_back(pack);
    }
    cv_.notify_one();
    return ErrorCode::OK;
  }

 private:
  std::mutex mutex_;
  std::condition_variable cv_;
  std::deque<ClassicPack> pending_;
};

// ---------------------------------------------------------------------------
// DFU
// ---------------------------------------------------------------------------

class RamFlash : public Flash
{
 public:
  RamFlash(size_t min_erase, size_t min_write, RawData area)
      : Flash(min_erase, min_write, area), area_(area)
  {
  }
  ErrorCode Erase(size_t offset, size_t size) override
  {
    if (offset + size > area_.size_) return ErrorCode::OUT_OF_RANGE;
    std::memset(static_cast<uint8_t*>(area_.addr_) + offset, 0xFF, size);
    return ErrorCode::OK;
  }
  ErrorCode Write(size_t offset, ConstRawData data) override
  {
    if (offset + data.size_ > area_.size_) return ErrorCode::OUT_OF_RANGE;
    std::memcpy(static_cast<uint8_t*>(area_.addr_) + offset, data.addr_, data.size_);
    return ErrorCode::OK;
  }

 private:
  RawData area_;
};

void JumpToApp(void*) { std::printf("dfu: jump to application requested\n"); }

template <typename Class>
int Serve(Class& klass, uint16_t vid, uint16_t pid, const char* product, uint16_t port,
          Sim::Profile profile)
{
  profile.sof_policy = Sim::SofPolicyFromEnvironment();
  static auto language = DescriptorStrings::MakeLanguagePack(
      DescriptorStrings::Language::EN_US, "LibXR", "USB sim device", "0004");
  (void)product;
  static Sim::Device<Caps> device(profile, DeviceDescriptor::PacketSize0::SIZE_64, vid,
                                  pid, 0x0100, {&language}, {{&klass}});
  Sim::UsbIpServer<Caps> server(device, port);
  server.Run();
  return 0;
}
}  // namespace

int main(int argc, char** argv)
{
  const std::string kind = argc > 1 ? argv[1] : "";
  const uint16_t port = argc > 2 ? static_cast<uint16_t>(std::atoi(argv[2])) : 3240;

  if (kind == "hid")
  {
    static VendorHid hid;
    Sim::Profile profile;
    profile.endpoints = {{1, Endpoint::Direction::BOTH, 256, 0}};
    return Serve(hid, 0x1209, 0x0004, "hid", port, profile);
  }
  if (kind == "gsusb")
  {
    static LoopbackCan can0;
    static GsUsbClass<1> gs(Endpoint::EPNumber::EP1, Endpoint::EPNumber::EP2, {&can0});
    Sim::Profile profile;
    profile.endpoints = {{1, Endpoint::Direction::IN, 512, 0},
                         {2, Endpoint::Direction::OUT, 512, 0}};
    return Serve(gs, 0x1D50, 0x606F, "gsusb", port, profile);
  }
  if (kind == "dfu")
  {
    static std::array<uint8_t, 256 * 1024> memory{};
    static RamFlash flash(1024, 8, {memory.data(), memory.size()});
    // 镜像区 128 KiB，seal 记录占最后一个擦除块，载荷是它之前的 127 KiB。
    // A 128 KiB image region; the seal record takes the last erase block and the
    // payload is the 127 KiB before it.
    static DfuBootloaderClassT<1024> dfu(flash, 0, 128 * 1024, 127 * 1024, &JumpToApp,
                                         nullptr, false);
    // DFU 的擦写和校验在应用线程里执行：这个线程循环调用 Process()。
    // The erase, program and verify of DFU run in an application thread: this thread
    // calls Process() in a loop.
    std::thread(
        []
        {
          while (true)
          {
            dfu.Process();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
          }
        })
        .detach();
    Sim::Profile profile;
    profile.endpoints = {};
    return Serve(dfu, 0x1209, 0x0005, "dfu", port, profile);
  }
  if (kind == "uac")
  {
    // 48 kHz 单声道 16 bit 麦克风；应用线程按实时速率写入递增的样本（锯齿波）。
    // A 48 kHz mono 16-bit microphone; the application thread writes an incrementing
    // sample (a sawtooth) at the real-time rate.
    static UAC1MicrophoneQ<1, 16> mic(Endpoint::EPNumber::EP1, 48000);
    std::thread(
        []
        {
          int16_t sample = 0;
          auto next = std::chrono::steady_clock::now();
          while (true)
          {
            int16_t block[48];
            for (auto& s : block) s = sample++;
            while (mic.WritePcm(block, sizeof(block)) != ErrorCode::OK)
              std::this_thread::sleep_for(std::chrono::microseconds(200));
            next += std::chrono::milliseconds(1);
            std::this_thread::sleep_until(next);
          }
        })
        .detach();
    Sim::Profile profile;
    profile.endpoints = {{1, Endpoint::Direction::IN, 512, 0}};
    return Serve(mic, 0x1209, 0x0006, "uac", port, profile);
  }
  std::fprintf(stderr, "usage: usb_sim_classes <hid|gsusb|dfu|uac> [port]\n");
  return 2;
}
