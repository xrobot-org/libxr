/**
 * @file swd_target_bitbang.cpp
 * @brief 仿真 SWD 目标的 remote_bitbang 服务端 / remote_bitbang server for the
 *        simulated SWD target.
 *
 * 真实的 OpenOCD（`adapter driver remote_bitbang`）通过它直接驱动仿真目标的引脚，用来
 * 校准目标模型。协议按 OpenOCD `doc/manual/jtag/drivers/remote_bitbang.txt`：
 * `d`..`g` 写 SWCLK（位 1）和 SWDIO（位 0），`O`/`o` 把 SWDIO 设为主机输出/输入，`c`
 * 读 SWDIO 回 `0`/`1`；`r`..`u` 的 srst 位拉低时目标复位；`B`/`b`/`Z`/`z` 忽略；`Q`
 * 结束会话。pyocd-bitbang 用 `0`..`7`（SWCLK、SWDIO 输出使能、SWDIO）和 `R`（最近一个
 * 上升沿的采样值）。
 * Real OpenOCD (`adapter driver remote_bitbang`) drives the pins of the simulated target
 * through it to calibrate the target model. Protocol per OpenOCD
 * `doc/manual/jtag/drivers/remote_bitbang.txt`: `d`..`g` write SWCLK (bit 1) and SWDIO
 * (bit 0), `O`/`o` make SWDIO a host output/input, `c` reads SWDIO and answers `0`/`1`;
 * asserting srst in `r`..`u` resets the target; `B`/`b`/`Z`/`z` are ignored; `Q` ends
 * the session. pyocd-bitbang uses `0`..`7` (SWCLK, SWDIO output enable, SWDIO) and `R`
 * (the value sampled at the latest rising edge).
 *
 * 用法 / Usage: swd_target_bitbang [port]（默认 44853，只监听 127.0.0.1）
 */

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdio>
#include <cstdlib>
#include <vector>

#include "swd_target.hpp"

using namespace LibXR::Debug::Sim;

namespace
{
/// 上升沿：先记下这一刻线上的值（主机在此采样），再推进目标 / Rising edge: record the
/// line at this instant (the host samples it here), then advance the target
void Rise(Target& t, bool& sampled)
{
  sampled = t.wire.Line();
  t.wire.Rising();
}

void Serve(int fd, Target& t)
{
  bool clock = false;
  bool srst = false;
  bool sampled = true;
  std::vector<char> out;
  char buf[4096];
  while (true)
  {
    const ssize_t n = recv(fd, buf, sizeof(buf), 0);
    if (n <= 0) return;
    out.clear();
    for (ssize_t i = 0; i < n; ++i)
    {
      const char c = buf[i];
      if (c >= 'd' && c <= 'g')
      {
        const int v = c - 'd';
        const bool clk = (v & 2) != 0, dio = (v & 1) != 0;
        t.wire.SetHostLevel(dio);
        if (clk && !clock) Rise(t, sampled);
        clock = clk;
      }
      else if (c >= '0' && c <= '7')
      {
        // pyocd-bitbang 的约定：位 2 SWCLK、位 1 SWDIO 输出使能、位 0 SWDIO / The
        // pyocd-bitbang convention: bit 2 SWCLK, bit 1 SWDIO output enable, bit 0 SWDIO
        const int v = c - '0';
        const bool clk = (v & 4) != 0, oe = (v & 2) != 0, dio = (v & 1) != 0;
        t.wire.SetHostDriving(oe);
        t.wire.SetHostLevel(dio);
        if (clk && !clock) Rise(t, sampled);
        clock = clk;
      }
      else if (c == 'O' || c == 'o')
      {
        t.wire.SetHostDriving(c == 'O');
      }
      else if (c == 'c')
      {
        // OpenOCD：在时钟低电平阶段读线上当前值 / OpenOCD reads the line as it is in the
        // clock-low phase
        out.push_back(t.wire.Line() ? '1' : '0');
      }
      else if (c == 'R')
      {
        // pyocd-bitbang：读最近一个上升沿采到的值 / pyocd-bitbang reads the value
        // sampled at the latest rising edge
        out.push_back(sampled ? '1' : '0');
      }
      else if (c >= 'r' && c <= 'u')
      {
        const bool assert_srst = ((c - 'r') & 1) != 0;
        if (assert_srst && !srst) t.memory.Reset();
        srst = assert_srst;
      }
      else if (c == 'Q')
      {
        if (!out.empty()) (void)send(fd, out.data(), out.size(), MSG_NOSIGNAL);
        return;
      }
      // B, b, Z, z：忽略 / ignored
    }
    if (!out.empty() && send(fd, out.data(), out.size(), MSG_NOSIGNAL) <= 0) return;
  }
}
}  // namespace

int main(int argc, char** argv)
{
  const uint16_t port = argc > 1 ? static_cast<uint16_t>(std::atoi(argv[1])) : 44853;
  const int listener = socket(AF_INET, SOCK_STREAM, 0);
  const int one = 1;
  setsockopt(listener, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  if (bind(listener, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0 ||
      listen(listener, 1) != 0)
  {
    std::perror("listen");
    return 1;
  }
  std::printf("swd target: remote_bitbang on 127.0.0.1:%u\n", port);
  std::fflush(stdout);
  while (true)
  {
    const int fd = accept(listener, nullptr, nullptr);
    if (fd < 0) continue;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    // 每个会话一个新上电的目标 / a freshly powered target per session
    Target t;
    // 周期性 WAIT：SWD_WAIT_EVERY=N 每 N 次 AP 访问注入 SWD_WAIT_BURST 个 WAIT
    // Periodic WAITs: SWD_WAIT_EVERY=N injects SWD_WAIT_BURST WAITs every N AP accesses
    if (const char* every = std::getenv("SWD_WAIT_EVERY"))
    {
      t.dp.wait_every = static_cast<unsigned>(std::atoi(every));
      if (const char* burst = std::getenv("SWD_WAIT_BURST"))
        t.dp.wait_burst = static_cast<unsigned>(std::atoi(burst));
    }
    if (std::getenv("SWD_TRACE") != nullptr)
    {
      t.wire.trace = [](const char* what, uint32_t edge, uint32_t value)
      { std::printf("  edge %u: %s 0x%X\n", edge, what, value); };
    }
    Serve(fd, t);
    close(fd);
    std::printf(
        "session: line_resets=%u protocol_errors=%u contention=%u resets=%u swd=%d\n",
        t.wire.line_resets, t.wire.protocol_errors, t.wire.contention, t.memory.resets,
        t.wire.SwdSelected());
    std::fflush(stdout);
  }
}
