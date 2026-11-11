/**
 * @file test_dap_fuzz.cpp
 * @brief DAP v2 随机测试：随机命令 × 随机分包 × 随机在途深度与读取时机 / Randomised DAP
 *        v2 test: random commands, random packetisation, random depth and read timing.
 *
 * DAP v2 类、USB 仿真后端和 SWD 目标仿真组成一个完整探针，Step 在自己的线程里运行，主机
 * 线程按包发送请求、按包读取响应。每条命令的应答逐字节和预期比较，读回的内存对照影子
 * 内存。随机的部分：命令种类与参数、原子命令组、分包方式（整倍数不补零长包、补零长包、
 * 补零到 DAP 包长）、在途深度 1..4、主机读响应的时机、目标侧注入的 WAIT。另有一组在命令
 * 在途时随机拔插，检查拔插后重新枚举、重新连接后一切照常。
 * The DAP v2 class, the USB simulation backend and the simulated SWD target form one
 * complete probe; Step runs in its own thread, the host thread sends requests and reads
 * responses packet by packet. Every response is compared byte for byte with the
 * expected one, and memory read back is checked against a shadow copy. Randomised:
 * command kinds and arguments, atomic groups, packetisation (packet multiples without a
 * ZLP, with a ZLP, zero-padded to the DAP packet size), depth 1..4, when the host reads
 * responses, and WAITs injected on the target. Another set unplugs at a random point
 * with commands in flight and checks that after re-enumeration and reconnection
 * everything works as before.
 *
 * 种子：DAP_FUZZ_SEEDS（默认 6）、DAP_FUZZ_CMDS（默认 120）；失败时打印种子。
 * Seeds: DAP_FUZZ_SEEDS (default 6), DAP_FUZZ_CMDS (default 120); a failure prints the
 * seed.
 *
 * 每个 DAP_QueueCommands 包就地应答（src/driver/usb/README.md 的迁移说明和
 * test_daplink_v2 的 TestQueueAndExecuteAnswerPerPacket 规定如此），所以排队包不等
 * 触发包。
 * Every DAP_QueueCommands packet is answered in place (the migration notes in
 * src/driver/usb/README.md and TestQueueAndExecuteAnswerPerPacket in test_daplink_v2 say
 * so), so queued packets do not wait for a trigger packet.
 */

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <random>
#include <thread>
#include <vector>

#include "automatic/driver/usb/sim/usb_sim.hpp"
#include "daplink_v2.hpp"
#include "swd_target.hpp"

using namespace LibXR;
using namespace LibXR::USB;
using Bytes = std::vector<uint8_t>;

namespace
{
constexpr auto LANGUAGE = DescriptorStrings::MakeLanguagePack(
    DescriptorStrings::Language::EN_US, "LibXR", "DAP fuzz", "0003");

constexpr uint32_t RAM = 0x20000000u;
constexpr uint32_t RAM_WORDS = 64u * 1024u / 4u;
constexpr uint8_t OUT_EP = 2, IN_EP = 1;
constexpr size_t USB_PACKET = 64;

unsigned g_seed = 0;

#define FUZZ_ASSERT(cond)                                                              \
  do                                                                                   \
  {                                                                                    \
    if (!(cond))                                                                       \
    {                                                                                  \
      std::fprintf(stderr, "%s:%d: seed %u: %s\n", __FILE__, __LINE__, g_seed, #cond); \
      std::abort();                                                                    \
    }                                                                                  \
  } while (0)

void Put32(Bytes& b, uint32_t v)
{
  for (int i = 0; i < 4; ++i) b.push_back(static_cast<uint8_t>(v >> (8 * i)));
}

/// 探针加目标，Step 在自己的线程里 / Probe plus target, Step in its own thread
struct Rig
{
  Debug::Sim::Target target;
  Debug::Sim::SimSwd swd{target.wire};
  DapLinkV2Class<Debug::Sim::SimSwd> dap{Endpoint::EPNumber::EP1, Endpoint::EPNumber::EP2,
                                         swd};
  Sim::Device<FullSpeedCapabilities> device;
  Sim::Host<FullSpeedCapabilities> host;
  std::atomic<bool> stop{false};
  std::thread step;

  static Sim::Profile MakeProfile()
  {
    Sim::Profile p;
    p.endpoints = {{1, Endpoint::Direction::IN, 2048, 0},
                   {2, Endpoint::Direction::OUT, 2048, 0}};
    return p;
  }

  Rig(unsigned wait_every, unsigned wait_burst)
      : device(MakeProfile(), DeviceDescriptor::PacketSize0::SIZE_64, 0x1209, 0x0003,
               0x0100, {&LANGUAGE}, {{&dap}}),
        host(device)
  {
    target.dp.wait_every = wait_every;
    target.dp.wait_burst = wait_burst;
    device.Init(false);
    device.Start(false);
    device.BusReset();
    FUZZ_ASSERT(host.Enumerate());
    step = std::thread(
        [this]
        {
          while (!stop.load()) (void)dap.Step(5);
        });
  }
  ~Rig()
  {
    stop = true;
    step.join();
  }
};

/// 主机侧：按包发请求、按包收响应、逐条比对 / Host side: sends requests and reads
/// responses packet by packet, comparing each one
class Session
{
 public:
  Session(Rig& rig, std::mt19937& rng) : r_(rig), rng_(rng) {}

  uint16_t dap_packet = 64;
  unsigned depth = 1;

  /// 一条请求包（含原子命令里的排队包）；expect 为空表示没有响应 / One request packet
  /// (queued atomic packets included); an empty expect means no response
  void Send(const Bytes& req, const Bytes& expect, bool has_response = true)
  {
    FUZZ_ASSERT(req.size() <= dap_packet && expect.size() <= dap_packet);
    // 在途达到深度时先收响应 / Read responses while the depth is reached
    while (outstanding_ >= depth) Pump(true);
    Packetize(req);
    if (has_response)
    {
      expected_.push_back(expect);
      ++outstanding_;
    }
    for (int i = Rand(0, 3); i > 0; --i) Pump(Rand(0, 1) == 1);
  }

  /// -1 随机分包；0..2 固定分包方式 / -1 random packetisation; 0..2 a fixed mode
  int force_mode = -1;
  /// 没有任何进展多久算失败 / How long without progress counts as a failure
  std::chrono::seconds idle_limit{30};

  void Drain()
  {
    while (!expected_.empty() || !out_.empty()) Pump(true);
  }

  /// 拔插：丢弃所有在途状态 / Unplug: drop every in-flight state
  void Forget()
  {
    out_.clear();
    expected_.clear();
    rx_.clear();
    outstanding_ = 0;
  }

  unsigned Outstanding() const { return outstanding_; }
  bool Busy() const { return !out_.empty() || !expected_.empty(); }

  int Rand(int lo, int hi) { return std::uniform_int_distribution<int>(lo, hi)(rng_); }

  /// 推进一步：发一个包，可能读一个包 / One step: send a packet, maybe read one
  void Pump(bool read)
  {
    bool progress = false;
    if (!out_.empty())
    {
      const auto hs = r_.device.OutPacket(OUT_EP, out_.front());
      FUZZ_ASSERT(hs != Sim::Handshake::STALL);
      if (hs == Sim::Handshake::ACK)
      {
        out_.pop_front();
        progress = true;
      }
    }
    if (read && !expected_.empty())
    {
      const auto in = r_.device.InToken(IN_EP);
      FUZZ_ASSERT(in.handshake != Sim::Handshake::STALL);
      if (in.handshake == Sim::Handshake::ACK)
      {
        progress = true;
        rx_.insert(rx_.end(), in.data.begin(), in.data.end());
        if (in.data.size() < USB_PACKET || rx_.size() >= dap_packet) Complete();
      }
    }
    if (progress)
    {
      idle_since_ = std::chrono::steady_clock::now();
    }
    else
    {
      std::this_thread::yield();
      FUZZ_ASSERT(std::chrono::steady_clock::now() - idle_since_ < idle_limit);
    }
  }

 private:
  void Complete()
  {
    FUZZ_ASSERT(!expected_.empty());
    const Bytes& want = expected_.front();
    if (rx_ != want)
    {
      std::fprintf(stderr, "seed %u: response mismatch (%zu bytes, want %zu)\n  got ",
                   g_seed, rx_.size(), want.size());
      for (size_t i = 0; i < rx_.size() && i < 24; ++i)
        std::fprintf(stderr, "%02X ", rx_[i]);
      std::fprintf(stderr, "\n  want ");
      for (size_t i = 0; i < want.size() && i < 24; ++i)
        std::fprintf(stderr, "%02X ", want[i]);
      std::fprintf(stderr, "\n");
      std::abort();
    }
    expected_.pop_front();
    rx_.clear();
    --outstanding_;
  }

  /// 分包：整倍数不补零长包（OpenOCD）、补零长包、补零到 DAP 包长（WinUSB 客户端）
  /// Packetise: packet multiples without a ZLP (OpenOCD), with a ZLP, or zero-padded to
  /// the DAP packet size (WinUSB clients)
  void Packetize(Bytes req)
  {
    int mode = Rand(0, 2);
    if (force_mode >= 0) mode = force_mode;
    if (mode == 2) req.resize(dap_packet, 0);
    for (size_t off = 0; off < req.size(); off += USB_PACKET)
    {
      out_.emplace_back(req.begin() + off,
                        req.begin() + std::min(req.size(), off + USB_PACKET));
    }
    if (mode == 1 && req.size() % USB_PACKET == 0) out_.emplace_back();
  }

  Rig& r_;
  std::mt19937& rng_;
  std::deque<Bytes> out_;
  std::deque<Bytes> expected_;
  Bytes rx_;
  unsigned outstanding_ = 0;
  std::chrono::steady_clock::time_point idle_since_ = std::chrono::steady_clock::now();
};

/// 命令生成器与影子内存 / Command generator and shadow memory
class Generator
{
 public:
  Generator(Session& s, std::mt19937& rng) : s_(s), rng_(rng) {}

  uint8_t caps = 0, packet_count = 0;
  std::vector<uint32_t> shadow = std::vector<uint32_t>(RAM_WORDS, 0);

  int Rand(int lo, int hi) { return std::uniform_int_distribution<int>(lo, hi)(rng_); }
  uint32_t Rand32() { return static_cast<uint32_t>(rng_()); }

  /// 连接：同 OpenOCD 的顺序 / Connect, in OpenOCD's order
  void Connect()
  {
    s_.Send({0x02, 0x01}, {0x02, 0x01});
    s_.Send({0x11, 0x40, 0x42, 0x0F, 0x00}, {0x11, 0x00});  // 1 MHz
    s_.Send({0x04, 0x00, 0x40, 0x00, 0x00, 0x00}, {0x04, 0x00});
    const Bytes ones{0x12, 51, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    s_.Send(ones, {0x12, 0x00});
    s_.Send({0x12, 16, 0x9E, 0xE7}, {0x12, 0x00});
    s_.Send(ones, {0x12, 0x00});
    s_.Send({0x12, 8, 0x00}, {0x12, 0x00});
    Bytes dpidr{0x05, 0x01, 0x01};
    Put32(dpidr, Debug::Sim::DebugPort::DPIDR);
    s_.Send({0x05, 0x00, 0x01, 0x02}, dpidr);
    Bytes power{0x05, 0x00, 0x03, 0x00};
    Put32(power, 0x1E);
    power.push_back(0x04);
    Put32(power, 0x50000000u);
    power.push_back(0x06);
    Bytes power_resp{0x05, 0x03, 0x01};
    Put32(power_resp, 0xF0000000u);
    s_.Send(power, power_resp);
    Bytes csw{0x05, 0x00, 0x02, 0x08};
    Put32(csw, 0);
    csw.push_back(0x01);
    Put32(csw, 0x23000012u);
    s_.Send(csw, {0x05, 0x02, 0x01});
  }

  void QueryInfo()
  {
    // 先单条往返取能力和包长，之后作为常量比对 / One round trip each for the
    // capabilities and the packet size, compared as constants afterwards
    caps = r_caps_;
    packet_count = r_count_;
  }

  /// 整个 RAM 写一遍已知内容 / Fill the whole RAM with known content
  void FillRam()
  {
    for (uint32_t page = 0; page < RAM_WORDS; page += 1024)
    {
      for (uint32_t off = 0; off < 1024;)
      {
        const uint32_t n =
            std::min<uint32_t>(1024 - off, static_cast<uint32_t>(Rand(64, 200)));
        WriteBlock(page + off, n);
        off += n;
      }
    }
  }

  void Random()
  {
    switch (Rand(0, 9))
    {
      case 0:
        Info();
        break;
      case 1:
      case 2:
        WriteBlock(RandomWord(nullptr), 0);
        break;
      case 3:
      case 4:
        ReadBlock();
        break;
      case 5:
        MixedTransfer();
        break;
      case 6:
        Delay();
        break;
      case 7:
        Atomic();
        break;
      case 8:
        Configure();
        break;
      default:
        if (s_.Outstanding() == 0)
          s_.Send({0x07}, {}, false);  // TransferAbort：没有响应 / no response
        else
          Info();
        break;
    }
  }

  void SetIdentity(uint8_t c, uint8_t n)
  {
    r_caps_ = c;
    r_count_ = n;
  }

 private:
  /// 在一个 4 KiB 页内随机取起点，n 为 0 时随机长度 / Random start inside one 4 KiB page;
  /// a random length when n is 0
  uint32_t RandomWord(uint32_t* count)
  {
    const uint32_t page = static_cast<uint32_t>(Rand(0, RAM_WORDS / 1024 - 1)) * 1024;
    const uint32_t n = count ? *count : 1;
    return page + static_cast<uint32_t>(Rand(0, static_cast<int>(1024 - n)));
  }

  Bytes TarSet(uint32_t word)
  {
    Bytes b{0x05, 0x00, 0x01, 0x05};
    Put32(b, RAM + word * 4);
    return b;
  }

  void Info()
  {
    switch (Rand(0, 2))
    {
      case 0:
        s_.Send({0x00, 0xF0}, {0x00, 0x01, caps});
        break;
      case 1:
        s_.Send({0x00, 0xFE}, {0x00, 0x01, packet_count});
        break;
      default:
        s_.Send({0x00, 0xFF}, {0x00, 0x02, static_cast<uint8_t>(s_.dap_packet),
                               static_cast<uint8_t>(s_.dap_packet >> 8)});
        break;
    }
  }

  void WriteBlock(uint32_t word, uint32_t n)
  {
    const uint32_t max_words = (s_.dap_packet - 5u) / 4u;
    if (n == 0)
    {
      n = static_cast<uint32_t>(
          Rand(1, static_cast<int>(std::min<uint32_t>(max_words, 254))));
      word = RandomWord(&n);
    }
    // 不跨 4 KiB 页：TAR 自增在页内回绕 / Never cross a 4 KiB page: TAR wraps inside it
    n = std::min<uint32_t>(n, 1024 - (word % 1024));
    s_.Send(TarSet(word), {0x05, 0x01, 0x01});
    Bytes b{0x06, 0x00, static_cast<uint8_t>(n), static_cast<uint8_t>(n >> 8), 0x0D};
    for (uint32_t i = 0; i < n; ++i)
    {
      const uint32_t v = Rand32();
      shadow[word + i] = v;
      Put32(b, v);
    }
    s_.Send(b, {0x06, static_cast<uint8_t>(n), static_cast<uint8_t>(n >> 8), 0x01});
  }

  void ReadBlock()
  {
    const uint32_t max_words = (s_.dap_packet - 4u) / 4u;
    uint32_t n = static_cast<uint32_t>(
        Rand(1, static_cast<int>(std::min<uint32_t>(max_words, 255))));
    const uint32_t word = RandomWord(&n);
    s_.Send(TarSet(word), {0x05, 0x01, 0x01});
    Bytes want{0x06, static_cast<uint8_t>(n), static_cast<uint8_t>(n >> 8), 0x01};
    for (uint32_t i = 0; i < n; ++i) Put32(want, shadow[word + i]);
    s_.Send({0x06, 0x00, static_cast<uint8_t>(n), static_cast<uint8_t>(n >> 8), 0x0F},
            want);
  }

  /// 一条 DAP_Transfer 里写 TAR、写两个字、回到起点读两个字 / One DAP_Transfer: write
  /// TAR, write two words, back to the start and read them
  Bytes MixedRequest(Bytes& want)
  {
    uint32_t n = 2;
    const uint32_t word = RandomWord(&n);
    const uint32_t a = Rand32(), b = Rand32();
    Bytes req{0x05, 0x00, 0x06};
    req.push_back(0x05);
    Put32(req, RAM + word * 4);
    req.push_back(0x0D);
    Put32(req, a);
    req.push_back(0x0D);
    Put32(req, b);
    req.push_back(0x05);
    Put32(req, RAM + word * 4);
    req.push_back(0x0F);
    req.push_back(0x0F);
    shadow[word] = a;
    shadow[word + 1] = b;
    want = {0x05, 0x06, 0x01};
    Put32(want, a);
    Put32(want, b);
    return req;
  }

  void MixedTransfer()
  {
    Bytes want;
    const Bytes req = MixedRequest(want);
    s_.Send(req, want);
  }

  void Delay()
  {
    const uint16_t us = static_cast<uint16_t>(Rand(0, 300));
    s_.Send({0x09, static_cast<uint8_t>(us), static_cast<uint8_t>(us >> 8)},
            {0x09, 0x00});
  }

  void Configure()
  {
    // 空闲周期 0..3，WAIT 重试足够多 / idle cycles 0..3, plenty of WAIT retries
    s_.Send({0x04, static_cast<uint8_t>(Rand(0, 3)), 0x40, 0x00, 0x00, 0x00},
            {0x04, 0x00});
  }

  /// 一个原子命令组的子命令 / Sub-commands of one atomic group
  void GroupCommands(Bytes& req, Bytes& want, uint8_t& n)
  {
    n = static_cast<uint8_t>(Rand(1, 3));
    for (uint8_t i = 0; i < n; ++i)
    {
      switch (Rand(0, 2))
      {
        case 0:
          req.insert(req.end(), {0x00, 0xFE});
          want.insert(want.end(), {0x00, 0x01, packet_count});
          break;
        case 1:
        {
          Bytes w;
          const Bytes r = MixedRequest(w);
          req.insert(req.end(), r.begin(), r.end());
          want.insert(want.end(), w.begin(), w.end());
          break;
        }
        default:
          req.insert(req.end(), {0x09, 0x10, 0x00});
          want.insert(want.end(), {0x09, 0x00});
          break;
      }
    }
  }

  /// 原子命令：每个 DAP_QueueCommands / DAP_ExecuteCommands 包就地执行并立即应答，应答
  /// 是 DAP_ExecuteCommands 的格式 [0x7F, n, ...]（src/driver/usb/README.md 的迁移说明和
  /// test_daplink_v2 的 TestQueueAndExecuteAnswerPerPacket 都是这样规定的；有的固件把
  /// 排队包的应答推迟到触发包，这里不是）。
  /// Atomic commands: every DAP_QueueCommands / DAP_ExecuteCommands packet runs its
  /// commands in place and is answered at once, in the DAP_ExecuteCommands format
  /// [0x7F, n, ...] (the migration notes in src/driver/usb/README.md and
  /// TestQueueAndExecuteAnswerPerPacket in test_daplink_v2 say so; some firmware defers
  /// the responses of queued packets to the trigger packet, this one does not).
  void Atomic()
  {
    const int k = Rand(0, 2);
    for (int g = 0; g < k; ++g)
    {
      Bytes req{0x7E, 0}, want{0x7F, 0};
      uint8_t n = 0;
      GroupCommands(req, want, n);
      req[1] = want[1] = n;
      s_.Send(req, want);
    }
    if (Rand(0, 1) == 0)
    {
      Bytes req{0x7F, 0}, want{0x7F, 0};
      uint8_t n = 0;
      GroupCommands(req, want, n);
      req[1] = want[1] = n;
      s_.Send(req, want);
    }
    else
    {
      s_.Send({0x00, 0xFE}, {0x00, 0x01, packet_count});
    }
  }

  Session& s_;
  std::mt19937& rng_;
  uint8_t r_caps_ = 0, r_count_ = 0;
};

/// 单条往返查 DAP_Info / One DAP_Info round trip
Bytes Query(Rig& r, Session& s, uint8_t id)
{
  (void)s;
  FUZZ_ASSERT(r.device.OutPacket(OUT_EP, {0x00, id}) == Sim::Handshake::ACK);
  for (int i = 0; i < 100000; ++i)
  {
    auto in = r.device.InToken(IN_EP);
    if (in.handshake == Sim::Handshake::ACK) return in.data;
    std::this_thread::yield();
  }
  FUZZ_ASSERT(false);
  return {};
}

void Setup(Rig& r, Session& s, Generator& g)
{
  const Bytes size = Query(r, s, 0xFF);
  FUZZ_ASSERT(size.size() == 4 && size[0] == 0x00 && size[1] == 0x02);
  s.dap_packet = static_cast<uint16_t>(size[2] | (size[3] << 8));
  const Bytes caps = Query(r, s, 0xF0);
  const Bytes count = Query(r, s, 0xFE);
  FUZZ_ASSERT(caps.size() == 3 && count.size() == 3);
  // 不声明原子命令位（0x10），每个组包就地应答；SWD 位必须在
  // The atomic-commands bit (0x10) is not announced; every group packet is answered in
  // place. The SWD bit must be set.
  FUZZ_ASSERT((caps[2] & 0x01u) != 0u);
  g.SetIdentity(caps[2], count[2]);
  g.QueryInfo();
}

unsigned EnvOr(const char* name, unsigned fallback)
{
  const char* v = std::getenv(name);
  return v ? static_cast<unsigned>(std::atoi(v)) : fallback;
}

void RunSeed(unsigned seed, unsigned commands)
{
  g_seed = seed;
  std::mt19937 rng(seed);
  const unsigned wait_every = std::uniform_int_distribution<unsigned>(0, 2)(rng) * 7;
  Rig r(wait_every, 1 + seed % 3);
  Session s(r, rng);
  Generator g(s, rng);
  Setup(r, s, g);
  s.depth = 1 + seed % 4;
  g.Connect();
  g.FillRam();
  for (unsigned i = 0; i < commands; ++i) g.Random();
  s.Drain();
  FUZZ_ASSERT(r.target.wire.protocol_errors == 0 && r.target.wire.contention == 0);
}

/// 命令在途时拔插，之后重新枚举、重新连接，一切照常 / Unplug with commands in flight,
/// then re-enumerate and reconnect; everything works as before
void RunUnplugSeed(unsigned seed, unsigned commands)
{
  g_seed = seed;
  std::mt19937 rng(seed);
  Rig r(0, 1);
  Session s(r, rng);
  Generator g(s, rng);
  Setup(r, s, g);
  s.depth = 4;
  for (int round = 0; round < 3; ++round)
  {
    g.Connect();
    g.FillRam();
    const unsigned cut =
        commands / 2 + static_cast<unsigned>(g.Rand(0, static_cast<int>(commands / 2)));
    for (unsigned i = 0; i < cut; ++i) g.Random();
    // 拔线：在途的请求和响应都作废 / Unplug: in-flight requests and responses are void
    if (g.Rand(0, 1) == 0)
      r.device.Disconnect();
    else
      r.device.BusReset();
    s.Forget();
    r.device.Attach();
    r.device.BusReset();
    const bool enumerated = r.host.Enumerate();
    if (!enumerated)
      std::fprintf(stderr, "seed %u: enumeration failed at %s\n", seed,
                   r.host.last_failure ? r.host.last_failure : "?");
    FUZZ_ASSERT(enumerated);
  }
  g.Connect();
  g.FillRam();
  for (unsigned i = 0; i < commands; ++i) g.Random();
  s.Drain();
}

/// 命令池占满、后到的包被留住（held）时拔插：留住的 OUT 槽要还给端点，否则重新枚举后
/// 端点少了接收槽，主机再也发不出包。一条 DAP_Delay（65.5 ms）占住 Step 线程，三条
/// DAP_Info 填满命令池，再到的两个包留在 held 里，下一个包被 NAK。
/// 这个场景要求延时命令在填池和确认 NAK 的整个过程中还没有结束；主机线程被调度走得太久
/// （负载高、ASan）时延时先结束，Step 线程放出命令槽，多收下的包不是缺陷。所以只有
/// 从延时命令被收下到最后一次 NAK 不超过 HOLD_WINDOW 时才判定，超过就丢弃这一轮重来。
/// Unplug while the command pool is full and later packets are held: the held OUT slots
/// must go back to the endpoint, otherwise after re-enumeration the endpoint is short of
/// receive slots and the host can no longer send. A DAP_Delay (65.5 ms) keeps the Step
/// thread busy, three DAP_Info commands fill the command pool, the next two packets stay
/// in the held list and the one after that is NAKed.
/// The scenario needs the delay to be still running while the pool is filled and the NAK
/// is confirmed. If the host thread is descheduled for long (high load, ASan) the delay
/// ends first, the Step thread gives command slots back and an extra accepted packet is
/// not a defect. So the result counts only when no more than HOLD_WINDOW passed between
/// the delay being accepted and the last NAK; otherwise the round is discarded and
/// repeated.
bool TryUnplugWithHeldPackets()
{
  constexpr auto HOLD_WINDOW = std::chrono::milliseconds(30);
  g_seed = 7;
  std::mt19937 rng(7);
  Rig r(0, 1);
  Session s(r, rng);
  Generator g(s, rng);
  Setup(r, s, g);
  g.Connect();
  s.Drain();
  // 上一条命令的接收槽可能还没重新挂上，NAK 就重试 / The receive slot of the previous
  // command may not be re-armed yet: retry on NAK
  bool delay_sent = false;
  for (unsigned attempts = 0; attempts < 20000 && !delay_sent; ++attempts)
  {
    delay_sent = r.device.OutPacket(OUT_EP, {0x09, 0xFF, 0xFF}) == Sim::Handshake::ACK;
    if (!delay_sent) std::this_thread::yield();
  }
  FUZZ_ASSERT(delay_sent);
  const auto delay_accepted = std::chrono::steady_clock::now();
  // 命令池的槽数是通告的包数，一个槽被延时命令占着，再加 2 个留住的包
  // The command pool has as many slots as the advertised packet count, one is taken by
  // the delay, plus two held packets
  const unsigned expected = g.packet_count + 1u;
  unsigned accepted = 0;
  for (unsigned attempts = 0; attempts < 20000 && accepted < expected; ++attempts)
  {
    if (r.device.OutPacket(OUT_EP, {0x00, 0xFE}) == Sim::Handshake::ACK)
      ++accepted;
    else
      std::this_thread::yield();
  }
  // 池和两个留住的包都满了：后面的包必须全部被 NAK / Pool and held packets are full:
  // every further packet must be NAKed
  unsigned extra = 0;
  for (unsigned attempts = 0; attempts < 200u; ++attempts)
  {
    if (r.device.OutPacket(OUT_EP, {0x00, 0xFE}) == Sim::Handshake::ACK) ++extra;
  }
  if (std::chrono::steady_clock::now() - delay_accepted > HOLD_WINDOW) return false;
  FUZZ_ASSERT(accepted == expected);
  FUZZ_ASSERT(extra == 0u);
  r.device.Disconnect();
  s.Forget();
  s.idle_limit = std::chrono::seconds(5);
  r.device.Attach();
  r.device.BusReset();
  FUZZ_ASSERT(r.host.Enumerate());
  g.Connect();
  for (int i = 0; i < 16; ++i) g.Random();
  s.Drain();
  return true;
}

void TestUnplugWithHeldPackets()
{
  for (int round = 0; round < 50; ++round)
    if (TryUnplugWithHeldPackets()) return;
  FUZZ_ASSERT(false);
}

/// WAIT 要如实报告：DAP_TransferConfigure 的 WAIT 重试为 0，目标连回两个 WAIT，
/// DAP_Transfer 的应答是已完成 0 个、ack 为 WAIT（2），第二个 WAIT 还留在目标里（没有
/// 重试）；重试够多时同样的 WAIT 被吸收。
/// A WAIT is reported as such: with the WAIT retry of DAP_TransferConfigure at 0 and two
/// WAITs from the target, DAP_Transfer answers zero transfers done and ack WAIT (2), and
/// the second WAIT is still in the target (no retry); with enough retries the same WAITs
/// are absorbed.
void TestWaitIsReportedWithoutRetries()
{
  g_seed = 8;
  std::mt19937 rng(8);
  Rig r(0, 1);
  Session s(r, rng);
  Generator g(s, rng);
  Setup(r, s, g);
  g.Connect();
  s.Drain();
  s.Send({0x04, 0x00, 0x00, 0x00, 0x00, 0x00}, {0x04, 0x00});
  s.Drain();
  r.target.dp.inject_wait = 2;
  s.Send({0x05, 0x00, 0x01, 0x0F}, {0x05, 0x00, 0x02});
  s.Drain();
  FUZZ_ASSERT(r.target.dp.inject_wait == 1);
  s.Send({0x04, 0x00, 0x40, 0x00, 0x00, 0x00}, {0x04, 0x00});
  s.Drain();
  r.target.dp.inject_wait = 2;
  Bytes absorbed{0x05, 0x01, 0x01};
  Put32(absorbed, 0);
  s.Send({0x05, 0x00, 0x01, 0x0F}, absorbed);
  s.Drain();
  FUZZ_ASSERT(r.target.dp.inject_wait == 0);
}

/// WAIT 重试次数为 N 表示第一次尝试之后最多再重试 N 次：N 为 0 时一个 WAIT 就报告出来；
/// N 为 2 时 2 个 WAIT 被吸收，3 个 WAIT 报告（目标刚好用掉 3 个），4 个 WAIT 报告且目标
/// 还剩 1 个。
/// A WAIT retry count of N means at most N retries after the first attempt: with 0 a
/// single WAIT is reported; with 2, two WAITs are absorbed, three are reported (the
/// target used exactly three), and four are reported with one left in the target.
void TestWaitRetryCount()
{
  g_seed = 9;
  std::mt19937 rng(9);
  Rig r(0, 1);
  Session s(r, rng);
  Generator g(s, rng);
  Setup(r, s, g);
  g.Connect();
  s.Drain();
  Bytes absorbed{0x05, 0x01, 0x01};
  Put32(absorbed, 0);
  struct Case
  {
    uint8_t retry;
    unsigned waits;
    bool absorbed;
    unsigned left;
  };
  for (const Case c : {Case{0, 1, false, 0}, Case{2, 2, true, 0}, Case{2, 3, false, 0},
                       Case{2, 4, false, 1}})
  {
    s.Send({0x04, 0x00, c.retry, 0x00, 0x00, 0x00}, {0x04, 0x00});
    s.Drain();
    r.target.dp.inject_wait = c.waits;
    s.Send({0x05, 0x00, 0x01, 0x0F}, c.absorbed ? absorbed : Bytes{0x05, 0x00, 0x02});
    s.Drain();
    FUZZ_ASSERT(r.target.dp.inject_wait == c.left);
    r.target.dp.inject_wait = 0;
  }
}

/// 主机把 DAP_ExecuteCommands / DAP_QueueCommands 补零到 DAP 包长（WinUSB
/// 客户端的做法）： 子命令之后多出的零字节不影响这一组的应答。 The host zero-pads
/// DAP_ExecuteCommands / DAP_QueueCommands to the DAP packet size (what WinUSB clients
/// do): the zero bytes after the sub-commands do not change the answer of the group.
void TestPaddedAtomicGroup()
{
  g_seed = 1;
  std::mt19937 rng(1);
  Rig r(0, 1);
  Session s(r, rng);
  Generator g(s, rng);
  Setup(r, s, g);
  g.Connect();
  s.force_mode = 2;
  s.Send({0x7F, 1, 0x00, 0xFE}, {0x7F, 1, 0x00, 0x01, g.packet_count});
  s.Send({0x7E, 1, 0x00, 0xFE}, {0x7F, 1, 0x00, 0x01, g.packet_count});
  s.Drain();
}

/// 常规运行：随机序列 + 拔插序列 / The regular run: random sequences and unplug sequences
void Regular()
{
  const unsigned seeds = EnvOr("DAP_FUZZ_SEEDS", 6);
  const unsigned commands = EnvOr("DAP_FUZZ_CMDS", 120);
  const unsigned base = EnvOr("DAP_FUZZ_BASE", 1);
  for (unsigned seed = base; seed < base + seeds; ++seed) RunSeed(seed, commands);
  for (unsigned seed = base; seed < base + (seeds + 1) / 2; ++seed)
    RunUnplugSeed(1000 + seed, commands);
  TestUnplugWithHeldPackets();
  TestWaitIsReportedWithoutRetries();
  TestWaitRetryCount();
  TestPaddedAtomicGroup();
  std::printf("dap fuzz: %u seeds x %u commands, %u unplug seeds\n", seeds, commands,
              (seeds + 1) / 2);
}

}  // namespace

int main()
{
  Regular();
  return 0;
}
