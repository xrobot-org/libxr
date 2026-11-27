/**
 * @file test_dap_replay.cpp
 * @brief 回放真实主机工具的 DAP 会话 / Replay of DAP sessions recorded from real host
 *        tools.
 *
 * `replay/` 下每个文件是一次真实会话（OpenOCD、pyOCD、probe-rs 经内核 CMSIS-DAP 驱动
 * 与 USB/IP 仿真器录下）：按完成顺序记录的每个 URB。这里用同样的
 * 组合（DAP v2 + SwdGeneralGPIO + 仿真目标）在仿真后端上重放：OUT 原样送出，IN 读到短
 * 包为止并与录制逐字节比较。录制时的回应来自当时的实现，所以这是回归基准：命令层、
 * 流水线或 SWD 引擎的行为一变，回应就对不上。
 * Each file under `replay/` is one real session recorded from OpenOCD, pyOCD, or probe-rs
 * through the Linux kernel CMSIS-DAP driver over USB/IP: every URB
 * in completion order. The same composition (DAP v2 + SwdGeneralGPIO + simulated target)
 * replays it on the simulation backend: OUT data goes out as recorded, IN is read up to
 * a short packet and compared byte for byte. The recorded responses came from the
 * implementation of that time, so this is a regression baseline: a change in the
 * command layer, the pipelining or the SWD engine shows up as a mismatching response.
 */

#include <algorithm>
#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "daplink_v2.hpp"
#include "debug/swd_general_gpio.hpp"
#include "swd_target.hpp"
#include "usb_sim.hpp"

using namespace LibXR;
using namespace LibXR::USB;
using namespace LibXR::USB::Sim;

namespace
{
constexpr auto LANGUAGE = DescriptorStrings::MakeLanguagePack(
    DescriptorStrings::Language::EN_US, "LibXR", "USB sim CMSIS-DAP", "0002");

using Engine = Debug::SwdGeneralGPIO<Debug::Sim::SimSwclkPin, Debug::Sim::SimSwdioPin>;

/// 不一致的条目数 / Mismatching entries
size_t g_mismatches = 0;

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
  // 与录制时的端点配置一致 / Match the endpoint configuration used when recording
  Profile profile;
  profile.endpoints = {{1, Endpoint::Direction::IN, 2048, 0, 0, 1},
                       {2, Endpoint::Direction::OUT, 2048, 0, 0, 1}};

  std::ifstream file(path);
  TEST_ASSERT(file.good());
  std::string line;
  std::vector<std::string> lines;
  while (std::getline(file, line))
  {
    if (line.rfind("# target", 0) == 0) ApplyHeader(line, target);
    if (!line.empty() && line[0] != '#') lines.push_back(line);
  }

  SimDevice device(profile, DeviceDescriptor::PacketSize0::SIZE_64, 0x1209, 0x0002,
                   0x0100, {&LANGUAGE}, {{&dap}});
  Host host(device);
  std::atomic<bool> stop{false};
  std::thread step(
      [&]
      {
        while (!stop.load()) (void)dap.Step(10);
      });
  device.Init(false);
  device.Start(false);
  TEST_ASSERT(host.Enumerate());

  size_t number = 0;
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
      // DAP_REPLAY_REPORT：列出全部不一致后再失败 / DAP_REPLAY_REPORT: list every
      // mismatch before failing
      ++g_mismatches;
      TEST_ASSERT(std::getenv("DAP_REPLAY_REPORT") != nullptr);
    }
  }
  stop.store(true);
  step.join();
  TEST_ASSERT(target.wire.protocol_errors == 0 && target.wire.contention == 0);
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
  TEST_ASSERT(g_mismatches == 0U);
  std::puts("dap replay: recorded host sessions match");
  return 0;
}
