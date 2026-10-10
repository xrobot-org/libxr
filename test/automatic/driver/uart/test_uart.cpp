/**
 * @file test_uart.cpp
 * @brief UART 在 Linux 伪终端上的测试 / UART tests using Linux pseudo-terminals.
 *
 * 检查收发背压、阻塞超时和回调续写，以及 Linux 后端的配置与断线重连。
 * Check RX/TX backpressure, blocking timeouts, callback writes, and Linux
 * configuration/reconnection.
 */

#include <poll.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <string>
#include <vector>

#include "linux_uart.hpp"
#include "test.hpp"
#include "test_assert.hpp"

namespace
{
using namespace LibXR;
using Clock = std::chrono::steady_clock;
using PollingStatus = WriteOperation::OperationPollingStatus;
constexpr auto WAIT_LIMIT = std::chrono::seconds(5);

template <typename Predicate>
void WaitUntil(Predicate predicate)
{
  const auto deadline = Clock::now() + WAIT_LIMIT;
  while (!predicate())
  {
    TEST_ASSERT(Clock::now() < deadline);
    Thread::Sleep(1);
  }
}

struct Pty
{
  Pty()
  {
    master = posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    TEST_ASSERT(master >= 0);
    TEST_ASSERT(grantpt(master) == 0);
    TEST_ASSERT(unlockpt(master) == 0);
    char* name = ptsname(master);
    TEST_ASSERT(name != nullptr);
    path = name;
    observer = open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    TEST_ASSERT(observer >= 0);
  }

  uint32_t Baudrate() const
  {
    termios2 config{};
    TEST_ASSERT(ioctl(observer, TCGETS2, &config) == 0);
    return config.c_ospeed;
  }

  void Send(const std::vector<uint8_t>& data)
  {
    size_t offset = 0;
    const auto deadline = Clock::now() + WAIT_LIMIT;
    while (offset < data.size())
    {
      const ssize_t count = write(master, data.data() + offset, data.size() - offset);
      if (count > 0)
      {
        offset += static_cast<size_t>(count);
        continue;
      }
      TEST_ASSERT(count < 0 && (errno == EINTR || errno == EAGAIN));
      TEST_ASSERT(Clock::now() < deadline);
      Thread::Sleep(1);
    }
  }

  std::vector<uint8_t> Receive(size_t size)
  {
    std::vector<uint8_t> data(size);
    size_t offset = 0;
    const auto deadline = Clock::now() + WAIT_LIMIT;
    while (offset < size)
    {
      const ssize_t count = read(master, data.data() + offset, size - offset);
      if (count > 0)
      {
        offset += static_cast<size_t>(count);
        continue;
      }
      TEST_ASSERT(count < 0 && (errno == EINTR || errno == EAGAIN));
      TEST_ASSERT(Clock::now() < deadline);
      Thread::Sleep(1);
    }
    return data;
  }

  void ExpectNoOutput()
  {
    pollfd item{master, POLLIN, 0};
    TEST_ASSERT(poll(&item, 1, 20) == 0);
  }

  int master = -1;
  int observer = -1;
  std::string path;
};

// 通知对象和驱动同寿命，等待与检查使用相同的 acquire 读取。
// Completion targets live with the driver; waits and checks use acquire loads.
struct Completion
{
  bool Is(PollingStatus expected) const
  {
    return status.load(std::memory_order_acquire) == expected;
  }

  void Wait(PollingStatus expected = PollingStatus::DONE)
  {
    WaitUntil([&] { return Is(expected); });
  }

  std::atomic<PollingStatus> status{PollingStatus::READY};
  WriteOperation op{status};
};

// 临时路径可在两台伪终端之间切换，退出测试时移除链接和目录。
// Switch a temporary path between PTYs; remove the link and directory on scope exit.
struct DevicePath
{
  explicit DevicePath(const Pty* initial = nullptr)
  {
    TEST_ASSERT(mkdtemp(directory) != nullptr);
    path = std::string(directory) + "/uart";
    if (initial != nullptr)
    {
      Connect(*initial);
    }
    else
    {
      // 普通文件可打开，但无法配置为串口，用于验证首次打开恢复。
      // A regular file opens but rejects TTY configuration, exercising initial recovery.
      const int fd = open(path.c_str(), O_CREAT | O_EXCL | O_WRONLY, 0600);
      TEST_ASSERT(fd >= 0);
      TEST_ASSERT(close(fd) == 0);
    }
  }

  ~DevicePath()
  {
    Remove();
    TEST_ASSERT(rmdir(directory) == 0);
  }

  void Remove() { TEST_ASSERT(unlink(path.c_str()) == 0); }
  void Connect(const Pty& pty)
  {
    TEST_ASSERT(symlink(pty.path.c_str(), path.c_str()) == 0);
  }

  void Disconnect(const Pty& pty)
  {
    Remove();
    TEST_ASSERT(close(pty.observer) == 0);
    TEST_ASSERT(close(pty.master) == 0);
  }

  char directory[sizeof("/tmp/libxr-uart-test-XXXXXX")] = "/tmp/libxr-uart-test-XXXXXX";
  std::string path;
};

std::vector<uint8_t> Pattern(size_t size)
{
  std::vector<uint8_t> result(size);
  for (size_t i = 0; i < size; ++i)
  {
    result[i] = static_cast<uint8_t>(i * 17U + i / 251U);
  }
  return result;
}

// 每项由现有 runner 在独立子进程运行；驱动与通知对象保持到进程退出。
// The runner isolates each case; driver and notification objects live until process exit.
struct Fixture
{
  explicit Fixture(size_t capacity = 64)
  {
    uart = new LinuxUART(pty.path.c_str(), 115200, UART::Parity::NO_PARITY, 8, 1, 8,
                         capacity);
    TEST_ASSERT(pty.Baudrate() == 115200);
  }

  Pty pty;
  LinuxUART* uart = nullptr;
  Completion completion;
};
}  // namespace

void test_linux_uart_rx_backpressure()
{
  auto* fixture = new Fixture;
  const auto expected = Pattern(192);
  fixture->pty.Send(expected);
  WaitUntil([&] { return fixture->uart->read_port_->Size() == 64; });

  std::vector<uint8_t> received;
  auto* sem = new Semaphore;
  for (size_t i = 0; i < 3; ++i)
  {
    std::array<uint8_t, 64> chunk{};
    ReadOperation op(*sem, 1000);
    TEST_ASSERT(fixture->uart->Read(RawData{chunk.data(), chunk.size()}, op) ==
                ErrorCode::OK);
    received.insert(received.end(), chunk.begin(), chunk.end());
  }
  TEST_ASSERT(received == expected);
  TEST_ASSERT(fixture->uart->read_port_->Size() == 0);
}

void test_linux_uart_tx_backpressure()
{
  constexpr size_t PAYLOAD_SIZE = 256 * 1024;
  auto* fixture = new Fixture(PAYLOAD_SIZE + 64);
  auto expected = Pattern(PAYLOAD_SIZE);
  TEST_ASSERT(fixture->uart->Write(ConstRawData{expected.data(), expected.size()},
                                   fixture->completion.op) == ErrorCode::OK);
  WaitUntil(
      [&]
      {
        const auto remaining = fixture->uart->write_port_->Size();
        return remaining > 0 && remaining < PAYLOAD_SIZE;
      });
  TEST_ASSERT(fixture->completion.Is(PollingStatus::RUNNING));
  const UART::Configuration config{57600, UART::Parity::NO_PARITY, 8, 1};
  TEST_ASSERT(fixture->uart->SetConfig(config) == ErrorCode::OK);
  TEST_ASSERT(fixture->pty.Baudrate() == 115200);

  const std::vector<uint8_t> suffix{0, 0xFF, '\r', '\n', 0x11, 0x13};
  auto* suffix_completion = new Completion;
  TEST_ASSERT(fixture->uart->Write(ConstRawData{suffix.data(), suffix.size()},
                                   suffix_completion->op) == ErrorCode::OK);
  expected.insert(expected.end(), suffix.begin(), suffix.end());
  TEST_ASSERT(fixture->pty.Receive(expected.size()) == expected);
  suffix_completion->Wait();
  TEST_ASSERT(fixture->completion.Is(PollingStatus::DONE));
  WaitUntil([&] { return fixture->pty.Baudrate() == config.baudrate; });
}

void test_linux_uart_block_timeout()
{
  auto* fixture = new Fixture(256 * 1024);
  const auto expected = Pattern(256 * 1024);
  auto* sem = new Semaphore;
  WriteOperation op(*sem, 30);
  TEST_ASSERT(fixture->uart->Write(ConstRawData{expected.data(), expected.size()}, op) ==
              ErrorCode::TIMEOUT);
  TEST_ASSERT(fixture->uart->write_port_->Size() > 0);
  TEST_ASSERT(fixture->pty.Receive(expected.size()) == expected);
  WaitUntil([&] { return fixture->uart->write_port_->Size() == 0; });
  TEST_ASSERT(sem->Value() == 0);
}

void test_linux_uart_callback_write()
{
  struct Context
  {
    Fixture fixture;
    std::atomic<int> calls{0};
    ErrorCode submit_result = ErrorCode::FAILED;
    std::array<uint8_t, 3> second{4, 5, 6};
  };
  auto* ctx = new Context;
  auto* callback = new WriteOperation::Callback(WriteOperation::Callback::Create(
      [](bool in_isr, Context* self, ErrorCode result)
      {
        TEST_ASSERT(!in_isr && result == ErrorCode::OK);
        self->submit_result = self->fixture.uart->Write(
            ConstRawData{self->second.data(), self->second.size()},
            self->fixture.completion.op);
        self->calls.fetch_add(1, std::memory_order_release);
      },
      ctx));
  WriteOperation op(*callback);
  const std::array<uint8_t, 3> first{1, 2, 3};
  TEST_ASSERT(ctx->fixture.uart->Write(ConstRawData{first.data(), first.size()}, op) ==
              ErrorCode::OK);
  TEST_ASSERT(ctx->fixture.pty.Receive(6) == std::vector<uint8_t>({1, 2, 3, 4, 5, 6}));
  ctx->fixture.completion.Wait();
  TEST_ASSERT(ctx->calls.load(std::memory_order_acquire) == 1);
  TEST_ASSERT(ctx->submit_result == ErrorCode::OK);
}

void test_linux_uart_stream_config()
{
  auto* fixture = new Fixture;
  auto* stream =
      new WritePort::Stream(fixture->uart->write_port_, fixture->completion.op);
  const auto expected = Pattern(32);
  TEST_ASSERT(stream->Write(ConstRawData{expected.data(), expected.size()}) ==
              ErrorCode::OK);
  fixture->pty.ExpectNoOutput();
  UART::Configuration config{57600, UART::Parity::NO_PARITY, 8, 1};
  TEST_ASSERT(fixture->uart->SetConfig(config) == ErrorCode::OK);
  WaitUntil([&] { return fixture->pty.Baudrate() == config.baudrate; });
  fixture->pty.ExpectNoOutput();
  TEST_ASSERT(stream->Commit() == ErrorCode::OK);
  TEST_ASSERT(fixture->pty.Receive(expected.size()) == expected);
  fixture->completion.Wait();
  config.baudrate = 0;
  TEST_ASSERT(fixture->uart->SetConfig(config) == ErrorCode::ARG_ERR);
}

void test_linux_uart_reconnect()
{
  auto* old_pty = new Pty;
  auto* next_pty = new Pty;
  DevicePath device(old_pty);
  auto* uart = new LinuxUART(device.path.c_str());
  TEST_ASSERT(old_pty->Baudrate() == 115200);
  device.Disconnect(*old_pty);

  // 先确认旧 fd 已关闭，再提交尚未尝试发送的新请求。
  // Observe old fd closure before admitting a request that has never been sent.
  WaitUntil(
      [&]
      {
        for (const auto& entry : std::filesystem::directory_iterator("/proc/self/fd"))
        {
          std::error_code ec;
          const auto target = std::filesystem::read_symlink(entry.path(), ec).string();
          if (!ec && (target == old_pty->path || target == old_pty->path + " (deleted)"))
          {
            return false;
          }
        }
        return true;
      });
  auto* completion = new Completion;
  const auto expected = Pattern(48);
  TEST_ASSERT(uart->Write(ConstRawData{expected.data(), expected.size()},
                          completion->op) == ErrorCode::OK);
  const UART::Configuration config{38400, UART::Parity::NO_PARITY, 8, 1};
  TEST_ASSERT(uart->SetConfig(config) == ErrorCode::OK);
  TEST_ASSERT(uart->SetConfig(config) == ErrorCode::BUSY);
  auto* received = new std::array<uint8_t, 48>;
  auto* read_completion = new Completion;
  TEST_ASSERT(uart->Read(RawData{received->data(), received->size()},
                         read_completion->op) == ErrorCode::OK);

  device.Connect(*next_pty);
  TEST_ASSERT(next_pty->Receive(expected.size()) == expected);
  TEST_ASSERT(next_pty->Baudrate() == config.baudrate);
  next_pty->Send(expected);
  read_completion->Wait();
  TEST_ASSERT(std::memcmp(received->data(), expected.data(), expected.size()) == 0);
  TEST_ASSERT(completion->Is(PollingStatus::DONE));
}

void test_linux_uart_partial_disconnect()
{
  auto* old_pty = new Pty;
  auto* next_pty = new Pty;
  DevicePath device(old_pty);
  constexpr size_t PAYLOAD_SIZE = 256 * 1024;
  auto* uart = new LinuxUART(device.path.c_str(), 115200, UART::Parity::NO_PARITY, 8, 1,
                             8, PAYLOAD_SIZE + 64);
  const auto first = Pattern(PAYLOAD_SIZE);
  auto* first_completion = new Completion;
  TEST_ASSERT(uart->Write(ConstRawData{first.data(), first.size()},
                          first_completion->op) == ErrorCode::OK);
  WaitUntil(
      [&]
      {
        const auto remaining = uart->write_port_->Size();
        return remaining > 0 && remaining < PAYLOAD_SIZE;
      });
  const auto second = Pattern(32);
  auto* second_completion = new Completion;
  TEST_ASSERT(uart->Write(ConstRawData{second.data(), second.size()},
                          second_completion->op) == ErrorCode::OK);

  device.Disconnect(*old_pty);
  first_completion->Wait(PollingStatus::ERROR);
  TEST_ASSERT(second_completion->Is(PollingStatus::RUNNING));
  device.Connect(*next_pty);
  TEST_ASSERT(next_pty->Receive(second.size()) == second);
  second_completion->Wait();
  next_pty->ExpectNoOutput();
}

void test_linux_uart_initial_open_recovery()
{
  auto* pty = new Pty;
  DevicePath device;
  auto* uart = new LinuxUART(device.path.c_str());
  auto* completion = new Completion;
  const auto expected = Pattern(32);
  TEST_ASSERT(uart->Write(ConstRawData{expected.data(), expected.size()},
                          completion->op) == ErrorCode::OK);
  device.Remove();
  device.Connect(*pty);
  TEST_ASSERT(pty->Receive(expected.size()) == expected);
  completion->Wait();
}
