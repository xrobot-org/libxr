/**
 * @file linux_can_vcan.cpp
 * @brief LinuxCANFD 的自动化测试（vcan 或真实互连接口）。
 *        Automated tests for LinuxCANFD on vcan or on two wired interfaces.
 *
 * 默认创建 vcan 接口（需要 CAP_NET_ADMIN），无法创建时以退出码 77 跳过，供 CTest 的
 * SKIP_RETURN_CODE 使用；传入 --a/--b 时改用两个真实接口（例如互连的 can0/can1），
 * 此时不创建也不删除接口。
 *
 * By default a vcan interface is created (needs CAP_NET_ADMIN); when that fails the
 * process exits with 77 so CTest reports SKIP. With --a/--b the test uses two real
 * interfaces instead (for example wired can0/can1) and leaves them untouched.
 */

#include <linux/can.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>

#include "linux_can.hpp"
#include "test_assert.hpp"

namespace
{
using namespace LibXR;

/// 回调队列容量 / Ring capacity used by the receive sinks.
constexpr uint32_t FRAME_CAPACITY = 512;
/// 单帧等待上限 / Per-frame wait limit in milliseconds.
constexpr uint32_t WAIT_TIMEOUT_MS = 500;
/// 无法创建 vcan 时的退出码 / Exit code used when vcan is unavailable.
constexpr int SKIP_EXIT_CODE = 77;

/// 执行 shell 命令并返回退出码 / Run a shell command and return its exit status.
int RunShell(const std::string& command)
{
  const int status = std::system(command.c_str());
  if (status < 0 || !WIFEXITED(status))
  {
    return -1;
  }
  return WEXITSTATUS(status);
}

/// 一帧的归一化副本 / Normalized copy of one received frame.
struct Frame
{
  bool fd = false;
  uint32_t id = 0;
  CAN::Type type = CAN::Type::STANDARD;
  uint8_t len = 0;
  uint8_t data[64] = {};
};

/// 预分配环形队列：接收线程写、测试线程读 / Preallocated SPSC ring.
class FrameQueue
{
 public:
  void Push(const Frame& frame)
  {
    const uint32_t head = head_.load(std::memory_order_relaxed);
    const uint32_t tail = tail_.load(std::memory_order_acquire);
    if (head - tail >= FRAME_CAPACITY)
    {
      overflowed_.store(true, std::memory_order_relaxed);
      return;
    }
    frames_[head % FRAME_CAPACITY] = frame;
    head_.store(head + 1, std::memory_order_release);
  }

  bool Pop(Frame& out)
  {
    const uint32_t tail = tail_.load(std::memory_order_relaxed);
    if (tail == head_.load(std::memory_order_acquire))
    {
      return false;
    }
    out = frames_[tail % FRAME_CAPACITY];
    tail_.store(tail + 1, std::memory_order_release);
    return true;
  }

  bool Wait(Frame& out, uint32_t timeout_ms = WAIT_TIMEOUT_MS)
  {
    for (uint32_t waited = 0; waited < timeout_ms; ++waited)
    {
      if (Pop(out))
      {
        return true;
      }
      usleep(1000);
    }
    return Pop(out);
  }

  void Reset()
  {
    tail_.store(head_.load(std::memory_order_acquire), std::memory_order_release);
    overflowed_.store(false, std::memory_order_relaxed);
  }

  bool Empty()
  {
    return head_.load(std::memory_order_acquire) == tail_.load(std::memory_order_relaxed);
  }

  bool Overflowed() const { return overflowed_.load(std::memory_order_relaxed); }

 private:
  Frame frames_[FRAME_CAPACITY];
  std::atomic<uint32_t> head_{0};
  std::atomic<uint32_t> tail_{0};
  std::atomic<bool> overflowed_{false};
};

/// 回调行为：记录、回调内关闭、回调内回发 / Callback behaviour under test.
enum class SinkMode
{
  RECORD,
  CLOSE_IN_CALLBACK,
  SEND_IN_CALLBACK,
};

/// 被测端点：驱动实例 + 回调入口 / One driver instance with its receive sinks.
class Endpoint
{
 public:
  Endpoint(const std::string& interface_name, bool enable_fd, bool brs)
      : can_(interface_name, enable_fd, brs)
  {
  }

  LinuxCANFD& can() { return can_; }
  FrameQueue& queue() { return queue_; }
  void set_mode(SinkMode mode) { mode_.store(mode, std::memory_order_release); }

  void reset()
  {
    queue_.Reset();
    calls_.store(0, std::memory_order_relaxed);
    close_result_.store(0, std::memory_order_relaxed);
    send_result_.store(0, std::memory_order_relaxed);
  }

  uint32_t calls() const { return calls_.load(std::memory_order_acquire); }
  int close_result() const { return close_result_.load(std::memory_order_acquire); }
  int send_result() const { return send_result_.load(std::memory_order_acquire); }

  void Register()
  {
    auto classic = CAN::Callback::Create(&Endpoint::OnClassic, this);
    can_.Register(classic, CAN::Type::STANDARD);
    can_.Register(classic, CAN::Type::EXTENDED);
    can_.Register(classic, CAN::Type::REMOTE_STANDARD);
    can_.Register(classic, CAN::Type::REMOTE_EXTENDED);

    auto fd = FDCAN::CallbackFD::Create(&Endpoint::OnFd, this);
    can_.Register(fd, CAN::Type::STANDARD);
    can_.Register(fd, CAN::Type::EXTENDED);
  }

 private:
  static void OnClassic(bool, Endpoint* self, const CAN::ClassicPack& pack)
  {
    Frame frame;
    frame.fd = false;
    frame.id = pack.id;
    frame.type = pack.type;
    frame.len = pack.dlc;
    std::memcpy(frame.data, pack.data, pack.dlc <= 8 ? pack.dlc : 8);
    self->Handle(frame);
  }

  static void OnFd(bool, Endpoint* self, const FDCAN::FDPack& pack)
  {
    Frame frame;
    frame.fd = true;
    frame.id = pack.id;
    frame.type = pack.type;
    frame.len = pack.len;
    std::memcpy(frame.data, pack.data, pack.len <= 64 ? pack.len : 64);
    self->Handle(frame);
  }

  /// 回调在接收线程执行：此处不得分配内存或阻塞 / Runs on the RX thread.
  void Handle(const Frame& frame)
  {
    switch (mode_.load(std::memory_order_acquire))
    {
      case SinkMode::RECORD:
        queue_.Push(frame);
        break;
      case SinkMode::CLOSE_IN_CALLBACK:
        close_result_.store(static_cast<int>(can_.Close()), std::memory_order_release);
        break;
      case SinkMode::SEND_IN_CALLBACK:
      {
        CAN::ClassicPack reply{};
        reply.id = 0x7FE;
        reply.type = CAN::Type::STANDARD;
        reply.dlc = 1;
        reply.data[0] = 0x5A;
        send_result_.store(static_cast<int>(can_.AddMessage(reply)),
                           std::memory_order_release);
        break;
      }
    }
    calls_.fetch_add(1, std::memory_order_release);
  }

  LinuxCANFD can_;
  FrameQueue queue_;
  std::atomic<SinkMode> mode_{SinkMode::RECORD};
  std::atomic<uint32_t> calls_{0};
  std::atomic<int> close_result_{0};
  std::atomic<int> send_result_{0};
};

/// 测试使用的接口组合 / Interface pair used by the suite.
struct TestConfig
{
  std::string a;
  std::string b;
  bool owns_interface = false;

  ~TestConfig()
  {
    if (owns_interface)
    {
      (void)RunShell("ip link del " + a + " >/dev/null 2>&1");
    }
  }
};

/// 创建临时 vcan 接口 / Create the temporary vcan interface.
bool CreateVcan(TestConfig& config)
{
  const std::string name = "xr" + std::to_string(getpid());
  if (RunShell("ip link add " + name + " type vcan >/dev/null 2>&1") != 0)
  {
    return false;
  }
  config.a = name;
  config.b = name;
  config.owns_interface = true;
  return true;
}

/// 把任意长度换成驱动补齐后的长度 / Map a length to the padded wire length.
uint8_t WireLength(uint8_t len)
{
  if (len <= 8)
  {
    return len;
  }
  const uint8_t canonical[] = {12, 16, 20, 24, 32, 48, 64};
  for (const uint8_t size : canonical)
  {
    if (len <= size)
    {
      return size;
    }
  }
  return 64;
}

/// 读取接口 MTU：CANFD_MTU 表示 FD 模式 / Link MTU tells whether FD is enabled.
int InterfaceMtu(const std::string& name)
{
  const int fd = socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
  if (fd < 0)
  {
    return -1;
  }
  ifreq request{};
  std::snprintf(request.ifr_name, IFNAMSIZ, "%s", name.c_str());
  const int result = ioctl(fd, SIOCGIFMTU, &request);
  (void)close(fd);
  return result == 0 ? request.ifr_mtu : -1;
}

/// 设置接口 up/down / Bring one interface up or down.
bool SetLink(const std::string& name, bool up)
{
  return RunShell("ip link set " + name + (up ? " up" : " down") + " >/dev/null 2>&1") ==
         0;
}

/// 构造已完成打开，这里只注册回调 / The constructor opens; this only registers sinks.
void OpenPair(Endpoint& a, Endpoint& b)
{
  a.Register();
  b.Register();
}

void ClosePair(Endpoint& a, Endpoint& b)
{
  (void)a.can().Close();
  (void)b.can().Close();
}

/// 收发一帧经典 CAN 并逐字段核对 / Send one classic frame and verify every field.
void CheckClassicFrame(Endpoint& tx, Endpoint& rx, uint32_t id, CAN::Type type,
                       uint8_t dlc)
{
  CAN::ClassicPack pack{};
  pack.id = id;
  pack.type = type;
  pack.dlc = dlc;
  for (uint8_t i = 0; i < dlc; ++i)
  {
    pack.data[i] = static_cast<uint8_t>(id + i * 7U + 3U);
  }

  rx.reset();
  tx.reset();
  TEST_ASSERT(tx.can().AddMessage(pack) == ErrorCode::OK);

  Frame received;
  TEST_ASSERT(rx.queue().Wait(received));
  TEST_ASSERT(!received.fd);
  TEST_ASSERT(received.id == id);
  TEST_ASSERT(received.type == type);
  TEST_ASSERT(received.len == dlc);
  TEST_ASSERT(dlc == 0 || std::memcmp(received.data, pack.data, dlc) == 0);
  TEST_ASSERT(tx.queue().Empty());
}

/// 收发一帧远程帧并核对类型与 DLC / Send one remote frame and verify type and DLC.
void CheckRemoteFrame(Endpoint& tx, Endpoint& rx, uint32_t id, CAN::Type type,
                      uint8_t dlc)
{
  CAN::ClassicPack pack{};
  pack.id = id;
  pack.type = type;
  pack.dlc = dlc;

  rx.reset();
  TEST_ASSERT(tx.can().AddMessage(pack) == ErrorCode::OK);

  Frame received;
  TEST_ASSERT(rx.queue().Wait(received));
  TEST_ASSERT(!received.fd);
  TEST_ASSERT(received.id == id);
  TEST_ASSERT(received.type == type);
  TEST_ASSERT(received.len == dlc);
}

/// 收发一帧 FD 并核对长度与补零 / Send one FD frame and verify length and padding.
void CheckFdFrame(Endpoint& tx, Endpoint& rx, uint32_t id, CAN::Type type, uint8_t len)
{
  const uint8_t expected_len = WireLength(len);
  FDCAN::FDPack pack{};
  pack.id = id;
  pack.type = type;
  pack.len = len;
  for (uint8_t i = 0; i < len; ++i)
  {
    pack.data[i] = static_cast<uint8_t>(i * 11U + 5U);
  }

  rx.reset();
  TEST_ASSERT(tx.can().AddMessage(pack) == ErrorCode::OK);

  Frame received;
  TEST_ASSERT(rx.queue().Wait(received));
  TEST_ASSERT(received.fd);
  TEST_ASSERT(received.id == id);
  TEST_ASSERT(received.type == type);
  TEST_ASSERT(received.len == expected_len);
  TEST_ASSERT(len == 0 || std::memcmp(received.data, pack.data, len) == 0);
  for (uint8_t i = len; i < expected_len; ++i)
  {
    TEST_ASSERT(received.data[i] == 0);
  }
}

/// 接口 down 时构造应等待，直到接口被拉起 / The constructor waits for the link.
void TestConstructorWaitsForUp(const TestConfig& config)
{
  TEST_ASSERT(SetLink(config.a, false));

  std::atomic<bool> link_raised{false};
  std::thread bring_up(
      [&]()
      {
        usleep(300 * 1000);
        link_raised.store(true);
        (void)SetLink(config.a, true);
      });

  const auto begin = std::chrono::steady_clock::now();
  {
    Endpoint endpoint(config.a, true, false);
    const auto waited = std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::steady_clock::now() - begin)
                            .count();
    TEST_ASSERT(link_raised.load());
    TEST_ASSERT(waited >= 250);
  }
  bring_up.join();
}

void TestLifecycle(const TestConfig& config)
{
  Endpoint endpoint(config.a, true, false);
  TEST_ASSERT(endpoint.can().Close() == ErrorCode::OK);

  CAN::ClassicPack pack{};
  pack.id = 0x100;
  pack.type = CAN::Type::STANDARD;
  pack.dlc = 1;
  TEST_ASSERT(endpoint.can().AddMessage(pack) == ErrorCode::STATE_ERR);
  TEST_ASSERT(endpoint.can().Close() == ErrorCode::OK);
}

/// 非法配置在触碰链路前就会被拒绝，不需要 CAP_NET_ADMIN。
/// Invalid configurations are rejected before touching the link, without privileges.
void TestConfigValidation(const TestConfig& config)
{
  Endpoint endpoint(config.a, true, false);

  CAN::Configuration invalid_classic{};
  invalid_classic.bitrate = 500000;
  invalid_classic.sample_point = 1.5f;
  TEST_ASSERT(endpoint.can().SetConfig(invalid_classic) == ErrorCode::ARG_ERR);

  FDCAN::Configuration data_without_fd{};
  data_without_fd.bitrate = 500000;
  data_without_fd.data_bitrate = 2000000;
  TEST_ASSERT(endpoint.can().SetConfig(data_without_fd) == ErrorCode::ARG_ERR);

  FDCAN::Configuration incomplete_timing{};
  incomplete_timing.bit_timing.brp = 2;
  TEST_ASSERT(endpoint.can().SetConfig(incomplete_timing) == ErrorCode::ARG_ERR);

  CAN::Configuration explicit_timing{};
  explicit_timing.bit_timing = {2, 1, 5, 2, 1};
  TEST_ASSERT(endpoint.can().SetConfig(explicit_timing) == ErrorCode::NOT_SUPPORT);
  explicit_timing.bitrate = 500000;
  TEST_ASSERT(endpoint.can().SetConfig(explicit_timing) == ErrorCode::NOT_SUPPORT);

  FDCAN::Configuration explicit_data{};
  explicit_data.bitrate = 500000;
  explicit_data.fd_mode.fd_enabled = true;
  explicit_data.data_timing = {2, 1, 5, 2, 1};
  TEST_ASSERT(endpoint.can().SetConfig(explicit_data) == ErrorCode::NOT_SUPPORT);
}

void TestClassicTraffic(const TestConfig& config, bool with_remote)
{
  Endpoint a(config.a, false, false);
  Endpoint b(config.b, false, false);
  OpenPair(a, b);

  for (uint8_t dlc = 0; dlc <= 8; ++dlc)
  {
    CheckClassicFrame(a, b, 0x123, CAN::Type::STANDARD, dlc);
    CheckClassicFrame(b, a, 0x456, CAN::Type::STANDARD, dlc);
  }
  CheckClassicFrame(a, b, 0x000, CAN::Type::STANDARD, 8);
  CheckClassicFrame(a, b, 0x7FF, CAN::Type::STANDARD, 8);
  CheckClassicFrame(a, b, 0x0, CAN::Type::EXTENDED, 8);
  CheckClassicFrame(a, b, 0x1FFFFFFF, CAN::Type::EXTENDED, 8);
  CheckClassicFrame(b, a, 0x1ABCDE, CAN::Type::EXTENDED, 4);

  // CAN FD 不支持远程帧，FD 模式下内核会丢弃 / Remote frames do not exist in CAN FD.
  if (with_remote)
  {
    for (const uint8_t dlc : {0, 4, 8})
    {
      CheckRemoteFrame(a, b, 0x321, CAN::Type::REMOTE_STANDARD, dlc);
    }
    CheckRemoteFrame(b, a, 0x12345, CAN::Type::REMOTE_EXTENDED, 4);
  }

  ClosePair(a, b);
}

void TestFdTraffic(const TestConfig& config)
{
  Endpoint a(config.a, true, true);
  Endpoint b(config.b, true, true);
  OpenPair(a, b);
  TEST_ASSERT(a.can().SupportsFD());
  TEST_ASSERT(b.can().SupportsFD());

  const uint8_t lengths[] = {0, 1, 2, 4, 8, 9, 12, 16, 20, 24, 32, 48, 64};
  for (const uint8_t len : lengths)
  {
    CheckFdFrame(a, b, 0x1A0, CAN::Type::STANDARD, len);
  }
  CheckFdFrame(a, b, 0x1FFFFFFF, CAN::Type::EXTENDED, 12);
  CheckFdFrame(b, a, 0x1A1, CAN::Type::STANDARD, 64);

  ClosePair(a, b);
}

void TestFdNotSupported(const TestConfig& config)
{
  Endpoint a(config.a, false, false);
  Endpoint b(config.b, false, false);
  OpenPair(a, b);
  TEST_ASSERT(!a.can().SupportsFD());

  FDCAN::FDPack pack{};
  pack.id = 0x1A1;
  pack.type = CAN::Type::STANDARD;
  pack.len = 8;
  TEST_ASSERT(a.can().AddMessage(pack) == ErrorCode::NOT_SUPPORT);
  ClosePair(a, b);
}

void TestCallbackReentry(const TestConfig& config)
{
  Endpoint a(config.a, true, false);
  Endpoint b(config.b, true, false);
  OpenPair(a, b);

  // 回调内调用 Close 必须返回 BUSY / Close from the RX callback must report BUSY.
  b.set_mode(SinkMode::CLOSE_IN_CALLBACK);
  CAN::ClassicPack pack{};
  pack.id = 0x200;
  pack.type = CAN::Type::STANDARD;
  pack.dlc = 1;
  TEST_ASSERT(a.can().AddMessage(pack) == ErrorCode::OK);
  for (uint32_t waited = 0; waited < WAIT_TIMEOUT_MS && b.calls() == 0; ++waited)
  {
    usleep(1000);
  }
  TEST_ASSERT(b.calls() > 0);
  TEST_ASSERT(b.close_result() == static_cast<int>(ErrorCode::BUSY));

  // 回调内发送必须成功，且对端能收到 / Sending from the callback must reach the peer.
  b.set_mode(SinkMode::SEND_IN_CALLBACK);
  a.reset();
  TEST_ASSERT(a.can().AddMessage(pack) == ErrorCode::OK);
  for (uint32_t waited = 0; waited < WAIT_TIMEOUT_MS && b.calls() < 2; ++waited)
  {
    usleep(1000);
  }
  Frame reply;
  TEST_ASSERT(b.send_result() == static_cast<int>(ErrorCode::OK));
  TEST_ASSERT(a.queue().Wait(reply));
  TEST_ASSERT(reply.id == 0x7FE);

  ClosePair(a, b);
}

void TestErrorState(const TestConfig& config)
{
  Endpoint a(config.a, true, false);
  CAN::ErrorState state{};
  const ErrorCode code = a.can().GetErrorState(state);
  TEST_ASSERT(code == ErrorCode::OK);
  TEST_ASSERT(state.tx_error_counter <= 255);
}

void TestConcurrentSenders(const TestConfig& config)
{
  Endpoint a(config.a, true, false);
  Endpoint b(config.b, true, false);
  OpenPair(a, b);
  b.reset();

  constexpr uint32_t PER_THREAD = 200;
  auto sender = [&a](uint32_t thread_id)
  {
    for (uint32_t i = 0; i < PER_THREAD; ++i)
    {
      CAN::ClassicPack pack{};
      pack.id = 0x300 + thread_id;
      pack.type = CAN::Type::STANDARD;
      pack.dlc = 4;
      const uint32_t value = (thread_id << 16) | i;
      std::memcpy(pack.data, &value, sizeof(value));
      while (a.can().AddMessage(pack) == ErrorCode::FULL)
      {
        usleep(50);
      }
    }
  };

  std::thread first(sender, 1U);
  std::thread second(sender, 2U);
  first.join();
  second.join();

  uint32_t received = 0;
  Frame frame;
  while (received < 2 * PER_THREAD && b.queue().Wait(frame, 200))
  {
    ++received;
  }
  TEST_ASSERT(received == 2 * PER_THREAD);
  TEST_ASSERT(!b.queue().Overflowed());

  ClosePair(a, b);
}
}  // namespace

int main(int argc, char** argv)
{
  TestConfig config;
  bool remote_opt_in = false;
  for (int i = 1; i < argc; ++i)
  {
    if (std::strcmp(argv[i], "--a") == 0 && i + 1 < argc)
    {
      config.a = argv[++i];
    }
    else if (std::strcmp(argv[i], "--b") == 0 && i + 1 < argc)
    {
      config.b = argv[++i];
    }
    else if (std::strcmp(argv[i], "--remote") == 0)
    {
      remote_opt_in = true;
    }
  }

  if (config.a.empty())
  {
    if (!CreateVcan(config))
    {
      std::printf("SKIP: cannot create vcan (CAP_NET_ADMIN required)\n");
      return SKIP_EXIT_CODE;
    }
  }
  else if (config.b.empty())
  {
    config.b = config.a;
  }

  if (!SetLink(config.a, true))
  {
    std::printf("SKIP: cannot bring %s up\n", config.a.c_str());
    return SKIP_EXIT_CODE;
  }
  if (config.a != config.b && !SetLink(config.b, true))
  {
    std::printf("SKIP: cannot bring %s up\n", config.b.c_str());
    return SKIP_EXIT_CODE;
  }

  // 无缓冲输出，abort 时也能看到进度 / Unbuffered so progress survives an abort.
  (void)std::setvbuf(stdout, nullptr, _IONBF, 0);
  std::printf("interfaces: %s / %s\n", config.a.c_str(), config.b.c_str());

  if (config.owns_interface)
  {
    std::printf("case: constructor_waits_for_up\n");
    TestConstructorWaitsForUp(config);
  }
  std::printf("case: lifecycle\n");
  TestLifecycle(config);
  std::printf("case: config_validation\n");
  TestConfigValidation(config);
  const bool fd_link = InterfaceMtu(config.a) == CANFD_MTU;
  const bool with_remote = remote_opt_in && !fd_link;
  std::printf("case: classic (remote=%s)\n", with_remote ? "on" : "skipped");
  TestClassicTraffic(config, with_remote);
  if (fd_link)
  {
    std::printf("case: fd\n");
    TestFdTraffic(config);
    std::printf("case: fd_not_supported\n");
    TestFdNotSupported(config);
  }
  std::printf("case: callback_reentry\n");
  TestCallbackReentry(config);
  std::printf("case: error_state\n");
  TestErrorState(config);
  std::printf("case: concurrent\n");
  TestConcurrentSenders(config);

  std::printf("linux_can tests done: %s / %s\n", config.a.c_str(), config.b.c_str());
  return 0;
}
