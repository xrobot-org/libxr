/**
 * @file usbip_server.hpp
 * @brief 把仿真设备通过 USB/IP 交给真实主机栈 / Hand the simulated device to a real host
 *        stack over USB/IP.
 *
 * 服务端扮演主机控制器：内核 vhci 送来 URB，服务端按端点包长把它拆成包，用仿真设备的
 * 主机令牌（SETUP、OUT、IN）推进；设备 NAK 时下一轮再试，等价于主机控制器的重试。同一
 * 端点上的 URB 按到达顺序处理，不同端点之间轮转。UNLINK 撤回还没完成的 URB。
 * The server acts as the host controller: the kernel vhci sends URBs, and the server
 * splits each into packets by the endpoint packet size and advances them with the host
 * tokens (SETUP, OUT, IN) of the simulated device; a NAK is retried on the next round,
 * as a host controller retries. URBs on one endpoint run in arrival order, endpoints
 * take turns. UNLINK withdraws a URB that has not completed.
 *
 * 协议依据 Linux `Documentation/usb/usbip_protocol.rst`；所有字段大端序。vhci 在本地
 * 处理 SET_ADDRESS 与端口复位，不转发，所以导入时服务端自己对设备做总线复位并设地址。
 * Protocol per Linux `Documentation/usb/usbip_protocol.rst`; every field is big-endian.
 * vhci handles SET_ADDRESS and port reset locally without forwarding them, so on import
 * the server resets the device and sets its address itself.
 *
 * 环境变量 USBIP_RECORD=<文件> 按完成顺序录下每个完成的 URB，供回放测试使用：
 * Environment variable USBIP_RECORD=<file> records every completed URB in completion
 * order, for replay tests:
 *   C <setup 8 字节十六进制> <status> <数据或 ->   控制传输（枚举请求除外）
 *   O <ep> <zlp 0/1> <status> <数据或 ->          批量/中断 OUT
 *   I <ep> <URB 长度> <status> <数据或 ->         批量/中断 IN
 *   C <setup as 8 hex bytes> <status> <data or ->  control (enumeration excluded)
 *   O <ep> <zlp 0/1> <status> <data or ->          bulk/interrupt OUT
 *   I <ep> <URB length> <status> <data or ->       bulk/interrupt IN
 */

#pragma once

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <map>
#include <string>
#include <thread>
#include <vector>

#include "usb_sim.hpp"

namespace LibXR::USB::Sim
{

template <typename Caps>
class UsbIpServer
{
 public:
  static constexpr uint16_t VERSION = 0x0111;
  static constexpr uint16_t OP_REQ_DEVLIST = 0x8005, OP_REP_DEVLIST = 0x0005;
  static constexpr uint16_t OP_REQ_IMPORT = 0x8003, OP_REP_IMPORT = 0x0003;
  static constexpr uint32_t CMD_SUBMIT = 1, CMD_UNLINK = 2, RET_SUBMIT = 3,
                            RET_UNLINK = 4;
  static constexpr uint32_t URB_ZERO_PACKET = 0x0040;
  static constexpr int32_t EPIPE_STATUS = -32, ECONNRESET_STATUS = -104;

  /// busid 默认是 "sim-1"，不用 usbipd 的 "1-1"；环境变量 USBIP_BUSID 覆盖它，脚本给每次
  /// 运行一个随机的 busid，挂接前据此确认连上的是自己启动的服务端
  /// The default busid is "sim-1", not the "1-1" of a system usbipd; the environment
  /// variable USBIP_BUSID overrides it, and the scripts give each run a random busid so
  /// that they can tell, before attaching, that they reached the server they started
  UsbIpServer(Device<Caps>& device, uint16_t port, const char* busid = "sim-1")
      : device_(device),
        host_(device),
        port_(port),
        busid_(std::getenv("USBIP_BUSID") != nullptr ? std::getenv("USBIP_BUSID") : busid)
  {
  }

  /// 监听并依次服务客户端（一次一个）；监听失败时返回 1，否则永不返回 / Listen and
  /// serve clients one at a time; returns 1 when it cannot listen, otherwise never
  /// returns
  int Run()
  {
    const int listener = socket(AF_INET, SOCK_STREAM, 0);
    const int one = 1;
    setsockopt(listener, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port_);
    // 默认只监听本机回环；在容器里运行时用 USBIP_BIND=0.0.0.0，再只把端口映射到宿主机
    // 的回环 / Loopback only by default; inside a container use USBIP_BIND=0.0.0.0 and
    // publish the port on the host's loopback only.
    const char* bind_to = std::getenv("USBIP_BIND");
    if (bind_to == nullptr) bind_to = "127.0.0.1";
    if (inet_pton(AF_INET, bind_to, &addr.sin_addr) != 1 ||
        bind(listener, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0 ||
        listen(listener, 1) != 0)
    {
      std::perror("usbip listen");
      return 1;
    }
    std::printf("usbip: listening on %s:%u busid %s\n", bind_to, port_, busid_.c_str());
    std::fflush(stdout);
    PrepareDevice();
    while (true)
    {
      const int fd = accept(listener, nullptr, nullptr);
      if (fd < 0) continue;
      setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
      ServeClient(fd);
      close(fd);
    }
  }

 private:
  struct Urb
  {
    uint32_t seqnum = 0;
    uint32_t ep = 0;
    bool in = false;
    uint32_t flags = 0;
    uint32_t length = 0;
    uint8_t setup[8] = {};
    std::vector<uint8_t> out;  ///< OUT 数据 / OUT data
    std::vector<uint8_t> got;  ///< IN 已收 / IN received
    size_t sent = 0;           ///< OUT 已发 / OUT sent
    bool zlp_sent = false;
    /// 等时包：偏移、长度、实际长度 / Isochronous packets: offset, length, actual
    struct IsoPacket
    {
      uint32_t offset = 0, length = 0, actual = 0;
    };
    std::vector<IsoPacket> iso;
    size_t iso_index = 0;
    uint32_t start_frame = 0;
    enum class Stage : uint8_t
    {
      SETUP,
      DATA,
      STATUS
    } stage = Stage::SETUP;
  };

  // ---- 编解码 / Encoding ----

  static void Put32(std::vector<uint8_t>& b, uint32_t v)
  {
    for (int i = 3; i >= 0; --i) b.push_back(static_cast<uint8_t>(v >> (i * 8)));
  }
  static void Put16(std::vector<uint8_t>& b, uint16_t v)
  {
    b.push_back(static_cast<uint8_t>(v >> 8));
    b.push_back(static_cast<uint8_t>(v));
  }
  static uint32_t Get32(const uint8_t* p)
  {
    return (uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16) | (uint32_t(p[2]) << 8) | p[3];
  }
  static uint16_t Get16(const uint8_t* p) { return uint16_t((p[0] << 8) | p[1]); }

  static bool ReadAll(int fd, void* buf, size_t n)
  {
    auto* p = static_cast<uint8_t*>(buf);
    while (n != 0)
    {
      const ssize_t r = recv(fd, p, n, 0);
      if (r <= 0) return false;
      p += r;
      n -= static_cast<size_t>(r);
    }
    return true;
  }
  static bool WriteAll(int fd, const std::vector<uint8_t>& b)
  {
    size_t off = 0;
    while (off < b.size())
    {
      const ssize_t r = send(fd, b.data() + off, b.size() - off, MSG_NOSIGNAL);
      if (r <= 0) return false;
      off += static_cast<size_t>(r);
    }
    return true;
  }

  // ---- 设备准备 / Device preparation ----

  /// 上电、复位、设地址，读出导入应答要用的描述符字段 / Power, reset, set the address and
  /// read the descriptor fields the import reply needs
  void PrepareDevice()
  {
    device_.Init(false);
    device_.Start(false);
    ResetAndAddress();
    bool ok = false;
    device_desc_ = host_.ControlIn(MakeSetup(0x80, 6, 0x0100, 0, 18), ok);
    REQUIRE(ok && device_desc_.size() == 18);
    auto head = host_.ControlIn(MakeSetup(0x80, 6, 0x0200, 0, 9), ok);
    REQUIRE(ok && head.size() == 9);
    const uint16_t total = static_cast<uint16_t>(head[2] | (head[3] << 8));
    config_desc_ = host_.ControlIn(MakeSetup(0x80, 6, 0x0200, 0, total), ok);
    REQUIRE(ok && config_desc_.size() == total);
  }

  void ResetAndAddress()
  {
    device_.Attach();
    device_.BusReset();
    REQUIRE(host_.ControlOut(MakeSetup(0x00, 5, 1, 0, 0)));
  }

  /// struct usbip_usb_device（312 字节）/ struct usbip_usb_device (312 bytes)
  void PutDevice(std::vector<uint8_t>& b) const
  {
    char path[256] = {};
    std::snprintf(path, sizeof(path), "/sys/devices/libxr-sim/%s", busid_.c_str());
    b.insert(b.end(), path, path + 256);
    char busid[32] = {};
    std::snprintf(busid, sizeof(busid), "%s", busid_.c_str());
    b.insert(b.end(), busid, busid + 32);
    Put32(b, 1);  // busnum
    Put32(b, 1);  // devnum
    Put32(b, 2);  // speed: USB_SPEED_FULL
    Put16(b, uint16_t(device_desc_[8] | (device_desc_[9] << 8)));
    Put16(b, uint16_t(device_desc_[10] | (device_desc_[11] << 8)));
    Put16(b, uint16_t(device_desc_[12] | (device_desc_[13] << 8)));
    b.push_back(device_desc_[4]);
    b.push_back(device_desc_[5]);
    b.push_back(device_desc_[6]);
    b.push_back(0);  // bConfigurationValue
    b.push_back(device_desc_[17]);
    b.push_back(config_desc_[4]);  // bNumInterfaces
  }

  // ---- 会话 / Session ----

  void ServeClient(int fd)
  {
    uint8_t head[8];
    if (!ReadAll(fd, head, sizeof(head))) return;
    const uint16_t code = Get16(head + 2);
    if (code == OP_REQ_DEVLIST)
    {
      std::vector<uint8_t> b;
      Put16(b, VERSION);
      Put16(b, OP_REP_DEVLIST);
      Put32(b, 0);
      Put32(b, 1);
      PutDevice(b);
      for (size_t off = 0; off + 9 <= config_desc_.size(); off += config_desc_[off])
      {
        if (config_desc_[off] == 0) break;
        if (config_desc_[off + 1] == 4 && config_desc_[off + 3] == 0)
        {
          b.push_back(config_desc_[off + 5]);
          b.push_back(config_desc_[off + 6]);
          b.push_back(config_desc_[off + 7]);
          b.push_back(0);
        }
      }
      WriteAll(fd, b);
      return;
    }
    if (code != OP_REQ_IMPORT) return;
    char busid[32];
    if (!ReadAll(fd, busid, sizeof(busid))) return;
    std::vector<uint8_t> b;
    Put16(b, VERSION);
    Put16(b, OP_REP_IMPORT);
    const bool match = std::strncmp(busid, busid_.c_str(), sizeof(busid)) == 0;
    Put32(b, match ? 0 : 1);
    if (match) PutDevice(b);
    if (!WriteAll(fd, b) || !match) return;
    std::printf("usbip: imported\n");
    std::fflush(stdout);
    ResetAndAddress();
    RunUrbs(fd);
    // 客户端断开：设备掉线 / Client gone: the device is unplugged.
    pending_.clear();
    device_.Disconnect();
    std::printf("usbip: detached\n");
    std::fflush(stdout);
  }

  void RunUrbs(int fd)
  {
    while (true)
    {
      // 有活干时不等，否则最多等 1 ms 看新命令 / No wait while there is work, otherwise
      // wait up to 1 ms for new commands.
      pollfd p{fd, POLLIN, 0};
      const int ready = poll(&p, 1, pending_.empty() ? 50 : (progress_ ? 0 : 1));
      if (ready < 0) return;
      if (ready > 0)
      {
        if ((p.revents & (POLLERR | POLLHUP)) != 0) return;
        if (!ReadCommand(fd)) return;
      }
      if (!Advance(fd)) return;
    }
  }

  bool ReadCommand(int fd)
  {
    uint8_t h[48];
    if (!ReadAll(fd, h, sizeof(h))) return false;
    const uint32_t command = Get32(h);
    const uint32_t seqnum = Get32(h + 4);
    if (command == CMD_UNLINK)
    {
      const uint32_t victim = Get32(h + 20);
      int32_t status = 0;
      for (auto& [ep, queue] : pending_)
      {
        for (auto it = queue.begin(); it != queue.end(); ++it)
        {
          if (it->seqnum == victim)
          {
            if (trace_ && it->ep != 0) Trace("unlink", *it, it->got, 0);
            queue.erase(it);
            status = ECONNRESET_STATUS;
            break;
          }
        }
        if (status != 0) break;
      }
      std::vector<uint8_t> b;
      Put32(b, RET_UNLINK);
      Put32(b, seqnum);
      Put32(b, 0);
      Put32(b, 0);
      Put32(b, 0);
      Put32(b, static_cast<uint32_t>(status));
      b.resize(48, 0);
      return WriteAll(fd, b);
    }
    if (command != CMD_SUBMIT) return false;
    Urb urb;
    urb.seqnum = seqnum;
    urb.in = Get32(h + 12) == 1;
    urb.ep = Get32(h + 16);
    urb.flags = Get32(h + 20);
    urb.length = Get32(h + 24);
    std::memcpy(urb.setup, h + 40, 8);
    const uint32_t packets = Get32(h + 32);
    const bool iso = packets != 0 && packets != 0xFFFFFFFFU;
    if (!urb.in && urb.length != 0)
    {
      urb.out.resize(urb.length);
      if (!ReadAll(fd, urb.out.data(), urb.length)) return false;
    }
    if (iso)
    {
      // 等时 IN：头之后是每个包的描述符；等时 OUT 不支持 / Isochronous IN: one
      // descriptor per packet follows the header; isochronous OUT is not supported
      if (!urb.in) return false;
      for (uint32_t i = 0; i < packets; ++i)
      {
        uint8_t d[16];
        if (!ReadAll(fd, d, sizeof(d))) return false;
        urb.iso.push_back({Get32(d), Get32(d + 4), 0});
      }
    }
    const uint32_t key = urb.ep | (urb.ep == 0 ? 0 : (urb.in ? 0x80U : 0U));
    if (trace_ && urb.ep != 0)
    {
      Trace("submit", urb, urb.in ? std::vector<uint8_t>() : urb.out, 0);
    }
    pending_[key].push_back(std::move(urb));
    return true;
  }

  /// 跟踪批量/中断 URB（环境变量 USBIP_TRACE）：端点、长度和前 16 字节 / Trace bulk and
  /// interrupt URBs (environment variable USBIP_TRACE): endpoint, length, first 16 bytes
  void Trace(const char* what, const Urb& urb, const std::vector<uint8_t>& data,
             int32_t status) const
  {
    std::printf("urb %-6s seq %u ep%u %s len %u status %d data", what, urb.seqnum, urb.ep,
                urb.in ? "IN " : "OUT", urb.length, status);
    for (size_t i = 0; i < data.size() && i < 16; ++i) std::printf(" %02X", data[i]);
    std::printf(data.size() > 16 ? " ... (%zu)\n" : "\n", data.size());
    std::fflush(stdout);
  }

  /// 每个端点的队首 URB 推进一步 / Advance the head URB of every endpoint by one step
  bool Advance(int fd)
  {
    progress_ = false;
    for (auto it = pending_.begin(); it != pending_.end();)
    {
      auto& queue = it->second;
      if (queue.empty())
      {
        it = pending_.erase(it);
        continue;
      }
      Urb& urb = queue.front();
      int32_t status = 0;
      if (Step(urb, status))
      {
        if (!Reply(fd, urb, status)) return false;
        queue.pop_front();
        progress_ = true;
      }
      ++it;
    }
    return true;
  }

  uint16_t PacketSize(const Urb& urb)
  {
    if (urb.ep == 0) return device_desc_[7];
    auto* ep = urb.in ? device_.In(static_cast<uint8_t>(urb.ep))
                      : device_.Out(static_cast<uint8_t>(urb.ep));
    REQUIRE(ep != nullptr && ep->MaxPacketSize() != 0U);
    return ep->MaxPacketSize();
  }

  /// 总线带宽：环境变量 USBIP_BULK_PER_MS 限制每毫秒（真实时间）完成的批量/中断事务数，
  /// 0 或不设置为不限。全速总线一帧装得下的事务数有限，设备的 IN、OUT 事务要共用它。
  /// Bus bandwidth: the environment variable USBIP_BULK_PER_MS limits the bulk and
  /// interrupt transactions completed per millisecond of real time; 0 or unset is
  /// unlimited. A full-speed frame holds a limited number of transactions that the
  /// device's IN and OUT transactions share.
  bool BulkAllowed()
  {
    if (bulk_per_ms_ == 0) return true;
    const auto now = std::chrono::steady_clock::now();
    if (now - bulk_window_ >= std::chrono::milliseconds(100)) bulk_window_ = now;
    while (now - bulk_window_ >= std::chrono::milliseconds(1))
    {
      bulk_window_ += std::chrono::milliseconds(1);
      bulk_used_ = 0;
    }
    return bulk_used_ < bulk_per_ms_;
  }

  /// URB 推进一步；完成时返回 true 并给出状态 / One step; true with a status when done
  bool Step(Urb& urb, int32_t& status)
  {
    if (urb.ep == 0) return StepControl(urb, status);
    if (!urb.iso.empty()) return StepIso(urb, status);
    const uint16_t packet = PacketSize(urb);
    if (urb.in)
    {
      if (!BulkAllowed()) return false;
      const InResult r = device_.InToken(static_cast<uint8_t>(urb.ep));
      if (r.handshake == Handshake::NAK) return false;
      ++bulk_used_;
      progress_ = true;
      if (r.handshake == Handshake::STALL)
      {
        status = EPIPE_STATUS;
        return true;
      }
      const size_t room = urb.length - urb.got.size();
      urb.got.insert(urb.got.end(), r.data.begin(),
                     r.data.begin() + std::min(room, r.data.size()));
      return r.data.size() < packet || urb.got.size() >= urb.length;
    }
    // OUT
    const bool need_zlp =
        (urb.flags & URB_ZERO_PACKET) != 0 && urb.length != 0 && urb.length % packet == 0;
    if (urb.sent >= urb.length && (!need_zlp || urb.zlp_sent) && urb.length != 0)
      return true;
    const size_t n = std::min<size_t>(packet, urb.length - urb.sent);
    std::vector<uint8_t> chunk(urb.out.begin() + urb.sent,
                               urb.out.begin() + urb.sent + n);
    if (!BulkAllowed()) return false;
    const Handshake hs = device_.OutPacket(static_cast<uint8_t>(urb.ep), chunk);
    if (hs == Handshake::NAK) return false;
    ++bulk_used_;
    progress_ = true;
    if (hs == Handshake::STALL)
    {
      status = EPIPE_STATUS;
      return true;
    }
    if (n == 0)
    {
      urb.zlp_sent = true;
      return true;
    }
    urb.sent += n;
    return urb.sent >= urb.length && !need_zlp;
  }

  /// 等时 IN 推进一帧：按真实时间每毫秒一帧，先发 SOF，再取一个包；设备没数据时是零长
  /// 包（等时传输不 NAK）。
  /// Advance isochronous IN by one frame: one frame per millisecond of real time, a SOF
  /// first, then one packet; a device with no data gives a zero-length packet
  /// (isochronous transfers do not NAK).
  bool StepIso(Urb& urb, int32_t& status)
  {
    const auto now = std::chrono::steady_clock::now();
    if (next_frame_ < now - std::chrono::milliseconds(100)) next_frame_ = now;
    if (now < next_frame_) return false;
    next_frame_ += std::chrono::milliseconds(1);
    progress_ = true;
    device_.Sof();
    if (urb.iso_index == 0) urb.start_frame = device_.Frame();
    auto& packet = urb.iso[urb.iso_index];
    const InResult r = device_.InToken(static_cast<uint8_t>(urb.ep));
    if (r.handshake == Handshake::ACK)
    {
      const size_t n = std::min<size_t>(packet.length, r.data.size());
      urb.got.insert(urb.got.end(), r.data.begin(), r.data.begin() + n);
      packet.actual = static_cast<uint32_t>(n);
    }
    status = 0;
    return ++urb.iso_index == urb.iso.size();
  }

  bool StepControl(Urb& urb, int32_t& status)
  {
    const uint16_t packet = PacketSize(urb);
    SetupPacket setup;
    std::memcpy(&setup, urb.setup, sizeof(setup));
    const bool data_in = (setup.bmRequestType & 0x80U) != 0;
    switch (urb.stage)
    {
      case Urb::Stage::SETUP:
        device_.Setup(setup);
        progress_ = true;
        urb.stage = setup.wLength == 0 ? Urb::Stage::STATUS : Urb::Stage::DATA;
        return false;
      case Urb::Stage::DATA:
        if (data_in)
        {
          const InResult r = device_.InToken(0);
          if (r.handshake == Handshake::NAK) return false;
          progress_ = true;
          if (r.handshake == Handshake::STALL)
          {
            status = EPIPE_STATUS;
            return true;
          }
          const size_t room = urb.length - urb.got.size();
          urb.got.insert(urb.got.end(), r.data.begin(),
                         r.data.begin() + std::min(room, r.data.size()));
          if (r.data.size() < packet || urb.got.size() >= urb.length)
            urb.stage = Urb::Stage::STATUS;
          return false;
        }
        else
        {
          const size_t n = std::min<size_t>(packet, urb.length - urb.sent);
          std::vector<uint8_t> chunk(urb.out.begin() + urb.sent,
                                     urb.out.begin() + urb.sent + n);
          const Handshake hs = device_.OutPacket(0, chunk);
          if (hs == Handshake::NAK) return false;
          progress_ = true;
          if (hs == Handshake::STALL)
          {
            status = EPIPE_STATUS;
            return true;
          }
          urb.sent += n;
          if (urb.sent >= urb.length) urb.stage = Urb::Stage::STATUS;
          return false;
        }
      case Urb::Stage::STATUS:
      {
        // 有 IN 数据阶段时状态是 OUT 零长包，否则是 IN 零长包 / After an IN data stage
        // the status is an OUT ZLP, otherwise an IN ZLP.
        Handshake hs;
        if (data_in && setup.wLength != 0)
        {
          hs = device_.OutPacket(0, {});
        }
        else
        {
          const InResult r = device_.InToken(0);
          hs = r.handshake;
        }
        if (hs == Handshake::NAK) return false;
        progress_ = true;
        status = hs == Handshake::STALL ? EPIPE_STATUS : 0;
        return true;
      }
    }
    return false;
  }

  /// 录制一个完成的 URB（USBIP_RECORD）/ Record one completed URB (USBIP_RECORD)
  void Record(const Urb& urb, int32_t status)
  {
    if (record_ == nullptr || !urb.iso.empty()) return;
    auto hex = [this](const std::vector<uint8_t>& data, size_t n)
    {
      if (n == 0)
      {
        std::fputs(" -", record_);
        return;
      }
      std::fputc(' ', record_);
      for (size_t i = 0; i < n; ++i) std::fprintf(record_, "%02x", data[i]);
    };
    if (urb.ep == 0)
    {
      // 枚举请求由回放端自己做 / The replay side enumerates by itself.
      const uint8_t type = urb.setup[0], request = urb.setup[1];
      if (type == 0x80 && request == 6) return;  // GET_DESCRIPTOR
      if (type == 0x00 && (request == 5 || request == 9)) return;
      std::fputc('C', record_);
      std::fputc(' ', record_);
      for (uint8_t b : urb.setup) std::fprintf(record_, "%02x", b);
      std::fprintf(record_, " %d", status);
      const bool in = (urb.setup[0] & 0x80U) != 0;
      hex(in ? urb.got : urb.out, in ? urb.got.size() : urb.sent);
    }
    else if (urb.in)
    {
      std::fprintf(record_, "I %u %u %d", urb.ep, urb.length, status);
      hex(urb.got, urb.got.size());
    }
    else
    {
      std::fprintf(record_, "O %u %d %d", urb.ep, urb.zlp_sent ? 1 : 0, status);
      hex(urb.out, urb.sent);
    }
    std::fputc('\n', record_);
    std::fflush(record_);
  }

  static std::FILE* OpenRecord()
  {
    const char* path = std::getenv("USBIP_RECORD");
    return path != nullptr ? std::fopen(path, "w") : nullptr;
  }

  bool Reply(int fd, const Urb& urb, int32_t status)
  {
    if (trace_ && urb.ep != 0)
    {
      Trace("done", urb, urb.in ? urb.got : std::vector<uint8_t>(), status);
    }
    Record(urb, status);
    std::vector<uint8_t> b;
    Put32(b, RET_SUBMIT);
    Put32(b, urb.seqnum);
    Put32(b, 0);
    Put32(b, 0);
    Put32(b, 0);
    Put32(b, static_cast<uint32_t>(status));
    const uint32_t actual =
        urb.in ? static_cast<uint32_t>(urb.got.size()) : static_cast<uint32_t>(urb.sent);
    Put32(b, status == 0 ? actual : 0);
    Put32(b, urb.start_frame);
    // number_of_packets：非等时为 0xFFFFFFFF / 0xFFFFFFFF when not isochronous
    Put32(b, urb.iso.empty() ? 0xFFFFFFFFu : static_cast<uint32_t>(urb.iso.size()));
    Put32(b, 0);  // error_count
    b.resize(48, 0);
    if (urb.in && status == 0) b.insert(b.end(), urb.got.begin(), urb.got.end());
    // 等时：数据按实际长度紧排，之后是每个包的描述符 / Isochronous: the data is packed
    // by actual length, followed by one descriptor per packet
    for (const auto& packet : urb.iso)
    {
      Put32(b, packet.offset);
      Put32(b, packet.length);
      Put32(b, packet.actual);
      Put32(b, 0);
    }
    return WriteAll(fd, b);
  }

  Device<Caps>& device_;
  Host<Caps> host_;
  uint16_t port_;
  std::string busid_;
  std::vector<uint8_t> device_desc_, config_desc_;
  std::map<uint32_t, std::deque<Urb>> pending_;
  bool progress_ = false;
  std::chrono::steady_clock::time_point next_frame_ = std::chrono::steady_clock::now();
  bool trace_ = std::getenv("USBIP_TRACE") != nullptr;
  unsigned bulk_per_ms_ =
      std::getenv("USBIP_BULK_PER_MS") != nullptr
          ? static_cast<unsigned>(std::atoi(std::getenv("USBIP_BULK_PER_MS")))
          : 0U;
  std::chrono::steady_clock::time_point bulk_window_ = std::chrono::steady_clock::now();
  unsigned bulk_used_ = 0;
  std::FILE* record_ = OpenRecord();
};

}  // namespace LibXR::USB::Sim
