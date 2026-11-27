/**
 * @file usb_sim_classes.cpp
 * @brief 仿真设备经 USB/IP 交给内核 / Simulated USB classes handed to the kernel over
 *        USB/IP
 *
 * 用法 / Usage: usb_sim_classes
 * <cdc_read|cdc_write|cdc_to_uart|hid|hid_std|gsusb|dfu|dfu_runtime|uac> [port]，然后
 * `sudo usbip --tcp-port <port> attach -r 127.0.0.1 -b 1-1`。
 * - cdc_read：CDCReadTest 连续丢弃主机发来的 OUT 流。
 *   cdc_read: CDCReadTest draining host-to-device OUT traffic.
 * - cdc_write：CDCWriteTest 在 DTR 置位后持续回传全零 IN 流。
 *   cdc_write: CDCWriteTest streaming zero-filled IN data while DTR is asserted.
 * - cdc_to_uart：CDCToUart 接线程安全的回环 UART（含线路编码配置回调）。
 *   cdc_to_uart: CDCToUart bridged to a thread-safe loopback UART with line coding.
 * - hid：厂商自定义用法页，8 字节输入/输出报告；主机写入的输出报告按位取反后作为输入
 *   报告回传（hidraw）。不用键盘、鼠标，免得往主机注入真实输入。
 *   hid: vendor-defined usage page, 8-byte input/output reports; an output report from
 *   the host comes back bitwise inverted as an input report (hidraw).
 * - hid_std：HIDKeyboard + HIDMouse + HIDGamepad + HIDGamepadBipolar 四接口复合设备，
 *   由键盘 CapsLock LED OUT 报告触发发送。
 *   hid_std: 4-interface composite (Keyboard + Mouse + Gamepad + GamepadBipolar),
 *   triggered only after CapsLock LED OUT report.
 * - gsusb：一个 CAN 通道，VID:PID 1D50:606F（内核 gs_usb 驱动绑定）。
 *   gsusb: one CAN channel, VID:PID 1D50:606F (bound by kernel gs_usb driver).
 * - dfu：DFU bootloader，128 KiB 镜像区在 RAM 里。
 *   dfu: DFU bootloader with a 128 KiB image area in RAM.
 * - dfu_runtime：DfuRuntimeClass（带 WebUSB 落地页与 WinUSB MS OS 2.0）。
 *   dfu_runtime: DfuRuntimeClass with WebUSB landing page and WinUSB MS OS 2.0.
 * - uac：48 kHz 单声道 16 bit 麦克风。
 *   uac: 48 kHz mono 16-bit microphone.
 */

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
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
#include "cdc_test.hpp"
#include "cdc_to_uart.hpp"
#include "dfu_bootloader.hpp"
#include "dfu_runtime.hpp"
#include "flash.hpp"
#include "gs_usb.hpp"
#include "hid.hpp"
#include "hid_gamepad.hpp"
#include "hid_keyboard.hpp"
#include "hid_mouse.hpp"
#include "libxr_pipe.hpp"
#include "uac_mic.hpp"
#include "uart.hpp"
#include "usbip_server.hpp"

using namespace LibXR;
using namespace LibXR::USB;

namespace
{

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

/// 厂商 HID：输出报告取反后作为输入报告回传 / Vendor HID: an output report comes back
/// inverted as an input report
class VendorHid : public HID
{
 public:
  VendorHid() : HID(Endpoint::EPNumber::EP1, Endpoint::EPNumber::EP1, 8, 8, 1, 1) {}

 protected:
  ConstRawData GetReportDesc() const override
  {
    return ConstRawData{VENDOR_REPORT_DESC, sizeof(VENDOR_REPORT_DESC)};
  }

  ErrorCode OnOutReport(bool in_isr, ConstRawData report) override
  {
    const auto* data = static_cast<const uint8_t*>(report.addr_);
    uint8_t reply[8] = {};
    for (size_t i = 0; i < 8 && i < report.size_; ++i)
    {
      reply[i] = static_cast<uint8_t>(~data[i]);
    }
    (void)SendInputReport(ConstRawData{reply, sizeof(reply)}, in_isr);
    return ErrorCode::OK;
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

/// 内存里的闪存 / Flash in RAM
class RamFlash : public Flash
{
 public:
  RamFlash(size_t min_erase, size_t min_write, RawData area)
      : Flash(min_erase, min_write, area), area_(area)
  {
  }
  ErrorCode Erase(size_t offset, size_t size) override
  {
    if (offset + size > area_.size_)
    {
      return ErrorCode::OUT_OF_RANGE;
    }
    std::memset(static_cast<uint8_t*>(area_.addr_) + offset, 0xFF, size);
    return ErrorCode::OK;
  }
  ErrorCode Write(size_t offset, ConstRawData data) override
  {
    if (offset + data.size_ > area_.size_)
    {
      return ErrorCode::OUT_OF_RANGE;
    }
    std::memcpy(static_cast<uint8_t*>(area_.addr_) + offset, data.addr_, data.size_);
    return ErrorCode::OK;
  }

 private:
  RawData area_;
};

// ---------------------------------------------------------------------------
// CDC-to-UART loopback
// ---------------------------------------------------------------------------

/// 线程安全回环 UART：接受 8 数据位 1 停止位，后台线程将 TX 字节环回至 RX /
/// Thread-safe loopback UART accepting 8 data bits and 1 stop bit
class LoopbackUart : public UART
{
 public:
  LoopbackUart()
      : UART(static_cast<LibXR::ReadPort*>(nullptr),
             static_cast<LibXR::WritePort*>(nullptr)),
        rx_(65536),
        tx_pipe_(131072)
  {
    read_port_ = &rx_;
    write_port_ = &tx_pipe_.GetWritePort();
    std::thread(
        [this]
        {
          uint8_t buf[512];
          LibXR::ReadPort& pipe_out = tx_pipe_.GetReadPort();
          while (true)
          {
            const size_t n = std::min(pipe_out.Size(), sizeof(buf));
            if (n == 0)
            {
              std::this_thread::sleep_for(std::chrono::microseconds(100));
              continue;
            }
            ReadOperation rop;
            if (pipe_out({buf, n}, rop) != ErrorCode::OK)
            {
              continue;
            }
            size_t pushed = 0;
            while (pushed < n)
            {
              auto q = rx_.GetReadQueue(false);
              const size_t chunk = std::min(q.EmptySize(), n - pushed);
              if (chunk != 0)
              {
                (void)q.PushBatch(buf + pushed, chunk);
                q.Publish();
                pushed += chunk;
              }
              else
              {
                q.Publish();
                std::this_thread::sleep_for(std::chrono::microseconds(100));
              }
            }
          }
        })
        .detach();
  }

  ErrorCode SetConfig(Configuration config, bool = false) override
  {
    if (config.baudrate < 300u || config.baudrate > 3000000u || config.data_bits != 8u ||
        config.stop_bits != 1u)
    {
      return ErrorCode::NOT_SUPPORT;
    }
    std::printf(
        "cdc_to_uart: line coding baud=%u parity=%u data=%u stop=%u\n",
        static_cast<unsigned>(config.baudrate), static_cast<unsigned>(config.parity),
        static_cast<unsigned>(config.data_bits), static_cast<unsigned>(config.stop_bits));
    std::fflush(stdout);
    return ErrorCode::OK;
  }

 private:
  LibXR::ReadPort rx_;
  LibXR::Pipe tx_pipe_;
};

void JumpToApp(void*)
{
  std::printf("dfu: jump to application requested\n");
  std::fflush(stdout);
}

constexpr auto LANGUAGE = DescriptorStrings::MakeLanguagePack(
    DescriptorStrings::Language::EN_US, "LibXR", "USB sim device", "0004");

int ServeList(std::initializer_list<DeviceClass*> classes, uint16_t vid, uint16_t pid,
              uint16_t port, Sim::Profile profile)
{
  Sim::ApplyStopModeFromEnv(profile);
  static Sim::SimDevice device(profile, DeviceDescriptor::PacketSize0::SIZE_64, vid, pid,
                               0x0100, {&LANGUAGE}, {classes});
  Sim::UsbIpServer server(device, port);
  server.Run();
  return 0;
}

int Serve(DeviceClass& klass, uint16_t vid, uint16_t pid, uint16_t port,
          Sim::Profile profile)
{
  return ServeList({&klass}, vid, pid, port, profile);
}
}  // namespace

int main(int argc, char** argv)
{
  const std::string kind = argc > 1 ? argv[1] : "";
  const uint16_t port = argc > 2 ? static_cast<uint16_t>(std::atoi(argv[2])) : 3240;

  if (kind == "cdc_read")
  {
    static CDCReadTest cdc(Endpoint::EPNumber::EP1, Endpoint::EPNumber::EP1,
                           Endpoint::EPNumber::EP2);
    return Serve(cdc, 0x1209, 0x0008, port, Sim::Profile{});
  }
  if (kind == "cdc_write")
  {
    static CDCWriteTest cdc(Endpoint::EPNumber::EP1, Endpoint::EPNumber::EP1,
                            Endpoint::EPNumber::EP2);
    return Serve(cdc, 0x1209, 0x0009, port, Sim::Profile{});
  }
  if (kind == "cdc_to_uart")
  {
    static LoopbackUart uart;
    static CDCToUart cdc(Endpoint::EPNumber::EP1, Endpoint::EPNumber::EP1,
                         Endpoint::EPNumber::EP2, uart, 512, 512, 8);
    return Serve(cdc, 0x1209, 0x000A, port, Sim::Profile{});
  }
  if (kind == "hid")
  {
    static VendorHid hid;
    Sim::Profile profile;
    profile.endpoints = {{1, Endpoint::Direction::BOTH, 256, 0, 0, 1}};
    return Serve(hid, 0x1209, 0x0004, port, profile);
  }
  if (kind == "hid_std")
  {
    static HIDKeyboard kbd(Endpoint::EPNumber::EP1, Endpoint::EPNumber::EP1, true);
    static HIDMouse mouse(Endpoint::EPNumber::EP2);
    static HIDGamepad pad(Endpoint::EPNumber::EP3);
    static HIDGamepadBipolar pad_bi(Endpoint::EPNumber::EP4);
    static std::atomic<uint32_t> trigger_seq{0};
    static bool prev_caps = false;

    kbd.SetOnLedChangeCallback(Callback<bool, bool, bool>::Create(
        [](bool, std::atomic<uint32_t>* seq, bool, bool caps, bool)
        {
          if (caps && !prev_caps)
          {
            seq->fetch_add(1u, std::memory_order_release);
          }
          prev_caps = caps;
        },
        &trigger_seq));

    std::thread(
        []
        {
          uint32_t handled = 0u;
          while (true)
          {
            const uint32_t seq = trigger_seq.load(std::memory_order_acquire);
            if (seq == handled)
            {
              std::this_thread::sleep_for(std::chrono::milliseconds(5));
              continue;
            }
            handled = seq;
            std::this_thread::sleep_for(std::chrono::milliseconds(15));
            (void)kbd.PressKey({HIDKeyboard::KeyCode::A});
            (void)mouse.Move(HIDMouse::Button::LEFT, 12, -7, 2);
            (void)pad.Send(100, 1900, 500, 1500, HIDGamepad::BTN1 | HIDGamepad::BTN3);
            (void)pad_bi.Send(-1500, 1500, -500, 500, HIDGamepadBipolar::BTN2);
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
            (void)kbd.ReleaseAll();
            (void)mouse.Release();
            (void)pad.Send(1023, 1023, 1023, 1023, 0);
            (void)pad_bi.Send(0, 0, 0, 0, 0);
          }
        })
        .detach();

    Sim::Profile profile;
    profile.endpoints = {{1, Endpoint::Direction::BOTH, 256, 0, 0, 1},
                         {2, Endpoint::Direction::IN, 256, 0, 0, 1},
                         {3, Endpoint::Direction::IN, 256, 0, 0, 1},
                         {4, Endpoint::Direction::IN, 256, 0, 0, 1}};
    return ServeList({&kbd, &mouse, &pad, &pad_bi}, 0x1209, 0x000C, port, profile);
  }
  if (kind == "gsusb")
  {
    static LoopbackCan can0;
    static GsUsbClass<1> gs(Endpoint::EPNumber::EP1, Endpoint::EPNumber::EP2, {&can0});
    Sim::Profile profile;
    profile.endpoints = {{1, Endpoint::Direction::IN, 512, 0, 0, 1},
                         {2, Endpoint::Direction::OUT, 512, 0, 0, 1}};
    return Serve(gs, 0x1D50, 0x606F, port, profile);
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
    // 擦写在应用调用 Process() 的上下文里 / Erase and write run in the context that
    // calls Process().
    std::thread(
        []
        {
          while (true)
          {
            dfu.Process();
            std::this_thread::sleep_for(std::chrono::microseconds(100));
          }
        })
        .detach();
    Sim::Profile profile;
    profile.endpoints = {};
    return Serve(dfu, 0x1209, 0x0005, port, profile);
  }
  if (kind == "dfu_runtime")
  {
    static DfuRuntimeClass dfu_rt(&JumpToApp, nullptr, 50U,
                                  DfuRuntimeClass::DEFAULT_INTERFACE_STRING,
                                  "https://example.com/dfu");
    std::thread(
        []
        {
          while (true)
          {
            dfu_rt.Process();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
          }
        })
        .detach();
    Sim::Profile profile;
    profile.endpoints = {};
    return Serve(dfu_rt, 0x1209, 0x000B, port, profile);
  }
  if (kind == "uac")
  {
    static UAC1MicrophoneQ<1, 16> mic(Endpoint::EPNumber::EP1, 48000);
    std::thread(
        []
        {
          int16_t sample = 0;
          auto next = std::chrono::steady_clock::now();
          while (true)
          {
            int16_t block[48];
            for (auto& s : block)
            {
              s = sample++;
            }
            while (mic.WritePcm(block, sizeof(block)) != ErrorCode::OK)
            {
              std::this_thread::sleep_for(std::chrono::microseconds(200));
            }
            next += std::chrono::milliseconds(1);
            std::this_thread::sleep_until(next);
          }
        })
        .detach();
    Sim::Profile profile;
    profile.endpoints = {{1, Endpoint::Direction::IN, 512, 0, 0, 1}};
    return Serve(mic, 0x1209, 0x0006, port, profile);
  }
  std::fprintf(stderr,
               "usage: usb_sim_classes "
               "<cdc_read|cdc_write|cdc_to_uart|hid|hid_std|gsusb|dfu|dfu_runtime|uac> "
               "[port]\n");
  return 2;
}
