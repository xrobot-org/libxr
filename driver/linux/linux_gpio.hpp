#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string>

#include "gpio.hpp"
#include "thread.hpp"

namespace LibXR
{

/**
 * @class LinuxGPIO
 * @brief 基于 Linux GPIO character device uAPI 的 Linux GPIO 实现
 *        Linux GPIO implementation based on the Linux GPIO character-device uAPI
 *
 * @note 优先使用 GPIO chardev v2；旧内核仅在需要时回退到 v1。
 * @note Prefer GPIO chardev v2; fall back to v1 only on older kernels.
 * @note 当前运行级验证聚焦在 chardev v2；v1 fallback 目前仅作为兼容路径保留。
 * @note Runtime validation currently focuses on chardev v2; the v1 fallback is kept as a
 *       compatibility path.
 */
class LinuxGPIO : public GPIO
{
 public:
  /**
   * @brief 构造并打开 GPIO 芯片设备 / Construct and open the GPIO chip device
   * @param chip_path GPIO 字符设备路径，例如 /dev/gpiochip0 / GPIO character device path,
   * for example /dev/gpiochip0
   * @param line_offset 芯片内的线偏移 / Line offset within the chip
   */
  LinuxGPIO(const std::string& chip_path, unsigned int line_offset);
  ~LinuxGPIO() override;

  LinuxGPIO(const LinuxGPIO&) = delete;
  LinuxGPIO& operator=(const LinuxGPIO&) = delete;

  bool Read() override;
  void Write(bool value) override;
  ErrorCode EnableInterrupt() override;
  ErrorCode DisableInterrupt() override;
  ErrorCode SetConfig(Configuration config) override;

 private:
  static constexpr size_t EVENT_BUFFER_CAPACITY = 64;
  static constexpr size_t INTERRUPT_THREAD_STACK_SIZE = 16384;

  enum class AbiVersion : uint8_t
  {
    UNKNOWN = 0,
    V2 = 1,
    V1 = 2,
  };

  enum class RequestKind : uint8_t
  {
    NONE = 0,
    HANDLE = 1,
    EVENT = 2,
  };

  std::string chip_path_;
  unsigned int line_offset_ = 0U;
  int chip_fd_ = -1;
  std::atomic<int> request_fd_{-1};
  std::atomic<AbiVersion> abi_version_{AbiVersion::UNKNOWN};
  std::atomic<RequestKind> request_kind_{RequestKind::NONE};
  Configuration current_config_ = {Direction::INPUT, Pull::NONE};
  bool has_config_ = false;
  std::atomic<bool> interrupt_enabled_{false};
  std::atomic<bool> interrupt_thread_started_{false};
  std::atomic<bool> interrupt_thread_exit_{false};
  std::atomic<bool> interrupt_poll_active_{false};
  Thread interrupt_thread_;
  int interrupt_wake_pipe_[2] = {-1, -1};

  ErrorCode OpenChip();
  void CloseChip();
  void CloseRequest();
  ErrorCode InitInterruptWakePipe();
  void CloseInterruptWakePipe();
  void NotifyInterruptThread();
  void WaitForInterruptLoopIdle();
  void WaitForInterruptThreadStopped();
  ErrorCode DetectAbiVersion();
  ErrorCode ReopenRequest(Configuration config);
  ErrorCode OpenRequestV2(Configuration config);
  ErrorCode ReconfigureRequestV2(Configuration config);
  ErrorCode OpenRequestV1(Configuration config);
  ErrorCode ReconfigureRequestV1(Configuration config);
  ErrorCode PumpEventQueue(int fd, AbiVersion abi_version, size_t& event_count,
                           int timeout_ms) const;
  ErrorCode ReadEventsV2(int fd, size_t& event_count) const;
  ErrorCode ReadEventsV1(int fd, size_t& event_count) const;
  void StartInterruptThread();
  void InterruptLoop();
  ErrorCode EnsureConfigured() const;
  static bool IsInterruptDirection(Direction direction);
  bool NeedsRequestReopen(Configuration config) const;
};

}  // namespace LibXR
