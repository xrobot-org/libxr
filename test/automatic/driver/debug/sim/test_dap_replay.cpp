/**
 * @file test_dap_replay.cpp
 * @brief 回放真实主机工具的 DAP 会话 / Replay of DAP sessions recorded from real host
 *        tools.
 *
 * `replay/` 下每个文件是一次真实会话（OpenOCD、pyOCD、probe-rs 经内核 CMSIS-DAP 驱动
 * 连 `dap_sim_usbip`，由 `dap_record.sh` 用 USBIP_RECORD 录下）：按完成顺序记录的每个
 * URB。这里用同样的组合（DAP v2 + SwdGeneralGPIO + 仿真目标）在仿真后端上重放：OUT 原样
 * 送出，IN 读到短包为止并与录制逐字节比较。录制时的回应来自当时的实现，所以这是回归
 * 基准：命令层、流水线或 SWD 引擎的行为一变，回应就对不上。
 * Each file under `replay/` is one real session (OpenOCD, pyOCD and probe-rs through the
 * kernel CMSIS-DAP driver to `dap_sim_usbip`, recorded by `dap_record.sh` with
 * USBIP_RECORD): every URB in completion order. The same composition (DAP v2 +
 * SwdGeneralGPIO + simulated target) replays it on the simulation backend: OUT data goes
 * out as recorded, IN is read up to a short packet and compared byte for byte. The
 * recorded responses came from the implementation of that time, so this is a regression
 * baseline: a change in the command layer, the pipelining or the SWD engine shows up as
 * a mismatching response.
 *
 * 录制文件目录由编译定义 DAP_REPLAY_DIR 给出（CMake 设为 `replay/` 的源码路径）。
 * 环境变量 DAP_REPLAY_KEEP_GOING=1 在不一致后继续并列出全部不一致，最后仍然失败。
 * The directory of the recordings is the compile definition DAP_REPLAY_DIR (CMake sets it
 * to the source path of `replay/`). The environment variable DAP_REPLAY_KEEP_GOING=1
 * carries on after a mismatch and lists them all; the run still fails at the end.
 */

#include <algorithm>
#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "automatic/driver/usb/sim/usb_sim.hpp"
#include "daplink_v2.hpp"
#include "debug/swd_general_gpio.hpp"
#include "swd_target.hpp"

using namespace LibXR;
using namespace LibXR::USB;
using namespace LibXR::USB::Sim;

namespace
{
constexpr auto LANGUAGE = DescriptorStrings::MakeLanguagePack(
    DescriptorStrings::Language::EN_US, "LibXR", "USB sim CMSIS-DAP", "0002");

using Engine = Debug::SwdGeneralGPIO<Debug::Sim::SimSwclkPin, Debug::Sim::SimSwdioPin>;
using Caps = FullSpeedCapabilities;

/// DAP_REPLAY_KEEP_GOING 下记下有过不一致 / Set when an entry mismatched under
/// DAP_REPLAY_KEEP_GOING
bool g_mismatch_seen = false;

std::vector<uint8_t> FromHex(const std::string& text)
{
  std::vector<uint8_t> data;
  if (text == "-") return data;
  for (size_t i = 0; i + 1 < text.size(); i += 2)
    data.push_back(static_cast<uint8_t>(std::stoul(text.substr(i, 2), nullptr, 16)));
  return data;
}

std::string ToHex(const std::vector<uint8_t>& data)
{
  std::string text;
  char byte[3];
  for (uint8_t b : data)
  {
    std::snprintf(byte, sizeof(byte), "%02x", b);
    text += byte;
  }
  return text.empty() ? "-" : text;
}

/// 录制来自 Line A，它的 DAP_Info(CAPABILITIES) 声明原子命令位（0x10）；候选线不声明，
/// 每个 DAP_QueueCommands/DAP_ExecuteCommands 包就地应答（候选线 README 的迁移说明和
/// TestQueueAndExecuteAnswerPerPacket 规定如此，C-missing-fixes 第 12a 项）。所以只有
/// 这一个应答里的这一位不参与比较，其余字节全部逐字节比。
/// The recordings come from Line A, whose DAP_Info(CAPABILITIES) announces the atomic
/// commands bit (0x10). The candidate does not: every DAP_QueueCommands /
/// DAP_ExecuteCommands packet is answered in place (the candidate's README migration
/// note and TestQueueAndExecuteAnswerPerPacket say so, C-missing-fixes item 12a). So that
/// one bit of that one response is left out of the comparison; every other byte is
/// compared exactly.
std::string WithoutAtomicBit(const std::string& request_hex, std::string response_hex)
{
  if (request_hex.compare(0, 4, "00f0") == 0 && response_hex.size() == 6 &&
      response_hex.compare(0, 4, "0001") == 0)
  {
    const auto caps =
        static_cast<uint8_t>(std::stoul(response_hex.substr(4, 2), nullptr, 16));
    char byte[3];
    std::snprintf(byte, sizeof(byte), "%02x", caps & ~0x10u);
    response_hex.replace(4, 2, byte);
  }
  return response_hex;
}

/// 头行 "# target wait_every=N wait_burst=M" / The header line
void ApplyHeader(const std::string& line, Debug::Sim::Target& target)
{
  std::istringstream in(line);
  std::string word;
  while (in >> word)
  {
    const auto eq = word.find('=');
    if (eq == std::string::npos) continue;
    const unsigned value = static_cast<unsigned>(std::stoul(word.substr(eq + 1)));
    if (word.compare(0, eq, "wait_every") == 0) target.dp.wait_every = value;
    if (word.compare(0, eq, "wait_burst") == 0) target.dp.wait_burst = value;
  }
}

/// 回放一个文件，返回回放的 URB 数 / Replay one file; returns the URBs replayed
size_t Replay(const std::filesystem::path& path)
{
  Debug::Sim::Target target;
  Debug::Sim::SimSwclkPin swclk(target.wire);
  Debug::Sim::SimSwdioPin swdio(target.wire);
  Engine engine(swclk, swdio, 0u, Debug::SwdGpioTiming{});
  DapLinkV2Class<Engine> dap(Endpoint::EPNumber::EP1, Endpoint::EPNumber::EP2, engine);
  // 与 dap_sim_usbip 相同 / Same as dap_sim_usbip
  Profile profile;
  profile.endpoints = {{1, Endpoint::Direction::IN, 2048, 0},
                       {2, Endpoint::Direction::OUT, 2048, 0}};

  std::ifstream file(path);
  TEST_ASSERT(file.good());
  std::string line;
  std::vector<std::string> lines;
  while (std::getline(file, line))
  {
    if (line.rfind("# target", 0) == 0) ApplyHeader(line, target);
    if (!line.empty() && line[0] != '#') lines.push_back(line);
  }

  Device<Caps> device(profile, DeviceDescriptor::PacketSize0::SIZE_64, 0x1209, 0x0002,
                      0x0100, {&LANGUAGE}, {{&dap}});
  Host<Caps> host(device);
  std::atomic<bool> stop{false};
  std::thread step(
      [&]
      {
        while (!stop.load()) (void)dap.Step(10);
      });
  device.Init(false);
  device.Start(false);
  device.BusReset();
  TEST_ASSERT(host.Enumerate());

  size_t number = 0, mismatches = 0;
  std::deque<std::string>
      requests;  // OUT 数据，按序与 IN 配对 / OUT data, paired with INs in order
  for (const auto& entry : lines)
  {
    ++number;
    std::istringstream in(entry);
    std::string kind;
    in >> kind;
    bool match = true;
    std::string expected, got;
    if (kind == "O")
    {
      unsigned ep = 0, zlp = 0;
      int status = 0;
      std::string data;
      in >> ep >> zlp >> status >> data;
      const bool ok = host.BulkOut(static_cast<uint8_t>(ep), 64, FromHex(data), zlp == 0);
      requests.push_back(data);
      match = ok == (status == 0);
      expected = "status " + std::to_string(status);
      got = ok ? "ok" : "failed";
    }
    else if (kind == "I")
    {
      unsigned ep = 0, length = 0;
      int status = 0;
      std::string data;
      in >> ep >> length >> status >> data;
      std::string request;
      if (!requests.empty())
      {
        request = requests.front();
        requests.pop_front();
      }
      data = WithoutAtomicBit(request, data);
      const auto read = host.BulkIn(static_cast<uint8_t>(ep), 64, length);
      expected = data;
      got = ToHex(read);
      match = status == 0 && expected == got;
    }
    else if (kind == "C")
    {
      std::string setup_hex, data;
      int status = 0;
      in >> setup_hex >> status >> data;
      const auto raw = FromHex(setup_hex);
      TEST_ASSERT(raw.size() == 8);
      const SetupPacket setup =
          MakeSetup(raw[0], raw[1], static_cast<uint16_t>(raw[2] | (raw[3] << 8)),
                    static_cast<uint16_t>(raw[4] | (raw[5] << 8)),
                    static_cast<uint16_t>(raw[6] | (raw[7] << 8)));
      bool ok = false;
      if ((raw[0] & 0x80U) != 0)
      {
        const auto read = host.ControlIn(setup, ok);
        got = ToHex(read);
        match = ok == (status == 0) && (!ok || got == data);
      }
      else
      {
        ok = host.ControlOut(setup, FromHex(data));
        match = ok == (status == 0);
      }
      expected = data + " status " + std::to_string(status);
    }
    else
    {
      TEST_ASSERT(false);
    }
    if (!match)
    {
      std::fprintf(stderr, "%s: entry %zu (%s) mismatch\n  expected %s\n  got      %s\n",
                   path.filename().c_str(), number, entry.substr(0, 40).c_str(),
                   expected.c_str(), got.c_str());
      if (std::getenv("DAP_REPLAY_KEEP_GOING") == nullptr) TEST_ASSERT(false);
      ++mismatches;
    }
  }
  stop.store(true);
  step.join();
  TEST_ASSERT(target.wire.protocol_errors == 0 && target.wire.contention == 0);
  if (mismatches != 0)
  {
    std::fprintf(stderr, "%s: %zu mismatching entries\n", path.filename().c_str(),
                 mismatches);
    g_mismatch_seen = true;
  }
  return lines.size();
}
}  // namespace

int main()
{
  std::vector<std::filesystem::path> files;
  for (const auto& entry : std::filesystem::directory_iterator(DAP_REPLAY_DIR))
    if (entry.path().extension() == ".txt") files.push_back(entry.path());
  std::sort(files.begin(), files.end());
  TEST_ASSERT(!files.empty());
  for (const auto& path : files)
  {
    const size_t count = Replay(path);
    std::printf("replayed %s: %zu URBs\n", path.filename().c_str(), count);
  }
  TEST_ASSERT(!g_mismatch_seen);
  std::puts("dap replay: recorded host sessions match");
  return 0;
}
