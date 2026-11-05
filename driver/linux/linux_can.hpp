#pragma once

#include <pthread.h>

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>

#include "can.hpp"

struct canfd_frame;

namespace LibXR
{

/**
 * @brief SocketCAN 驱动，同时支持经典 CAN 与 CAN FD。
 *        SocketCAN driver supporting both classic CAN and CAN FD.
 * @note 两种构造方式 / Two construction modes:
 *       不传配置时等待外部把接口置为 up，与 LinuxUART 等待设备节点一致，构造可能阻塞；
 *       传入 CAN::Configuration / FDCAN::Configuration 时先等接口出现，再用 rtnetlink
 *       设置 bittiming/ctrlmode/FD 并把接口拉起，最后打开 socket，需要 CAP_NET_ADMIN。
 *       Without a configuration the constructor waits for an externally configured, up
 *       interface. With one, it waits for the interface to appear, applies
 *       bittiming/ctrlmode/FD through rtnetlink, brings the link up and opens the
 *       socket (CAP_NET_ADMIN required). Only kernel C APIs are used, so neither
 *       libsocketcan nor iproute2 is required.
 * @note 配置仅支持 bitrate 和 sample_point；显式位时序尚不支持，构造时会触发 REQUIRE。
 *       Configuration supports bitrate and sample_point; explicit timing is unsupported
 *       and triggers REQUIRE when supplied to a constructor.
 * @note CAN::Configuration 的 bitrate 和时序全为 0 时保留当前时序，mode 各开关始终写入。
 *       Zero bitrate/timing in CAN::Configuration preserves current timing; all mode
 *       switches are always applied, including those in a default configuration.
 * @note Open/Close/析构/SetConfig 须由调用方串行执行，不可与其他调用并发。
 *       Serialize Open/Close/destruction/SetConfig externally against other calls.
 *       Close joins the RX thread; do not destroy the object from an RX callback.
 *       每次有效的 SetConfig 都会短暂 down/up / Each valid SetConfig cycles the link.
 * @note 回调在接收线程执行，不应阻塞；帧引用仅在回调期间有效。
 *       Callbacks run on the RX thread and must not block; frame references are borrowed.
 *       接收线程在允许时使用 SCHED_FIFO，阻塞回调会连带拖延同机的实时线程。
 *       The RX thread uses SCHED_FIFO when permitted, so a blocking callback also
 *       delays other real-time threads on the same system.
 */
class LinuxCANFD : public FDCAN
{
 public:
  /// 默认接收线程栈大小 / Default RX thread stack size.
  static constexpr size_t DEFAULT_RX_STACK_SIZE = 256 * 1024;

  /**
   * @brief 保存接口与收发选项，构造即打开驱动 / Store interface/TX options; the
   *        constructor opens the driver.
   * @param interface_name SocketCAN 接口名 / SocketCAN interface name, e.g. can0.
   * @param enable_fd 接口已是 FD 模式时是否使用 FD 帧 / Use FD frames when the link is
   *        already in FD mode.
   * @param brs FD 发送是否请求位速率切换 / Request bit-rate switching on FD TX.
   * @param rx_stack_size 接收线程栈大小，用户回调也在此栈运行 / RX thread stack size,
   *        which also backs user callbacks.
   * @param realtime 接收线程是否使用 SCHED_FIFO（无权限时回退默认策略）/ Use SCHED_FIFO
   *        for RX, falling back to the default policy without privileges.
   */
  explicit LinuxCANFD(const std::string& interface_name, bool enable_fd = true,
                      bool brs = false, size_t rx_stack_size = DEFAULT_RX_STACK_SIZE,
                      bool realtime = true);

  /**
   * @brief 构造并接管接口：等待接口出现，用 rtnetlink 配置经典 CAN 后拉起
   *        Construct and take over the interface, configuring classic CAN via rtnetlink.
   * @param config 仲裁相位与工作模式；只改这两个部分，FD 开关保持系统原状
   *        Nominal phase and mode; FD state is left untouched.
   * @note 构造会阻塞到接口出现且配置成功；需要 CAP_NET_ADMIN。
   *       Blocks until the interface exists and is configured; needs CAP_NET_ADMIN.
   */
  LinuxCANFD(const std::string& interface_name, const CAN::Configuration& config,
             size_t rx_stack_size = DEFAULT_RX_STACK_SIZE, bool realtime = true);

  /**
   * @brief 构造并接管接口：等待接口出现，用 rtnetlink 配置仲裁/数据相位与 FD 模式
   *        Construct and take over the interface, configuring both phases and FD mode.
   * @param config FD 配置；fd_mode.fd_enabled 决定是否开启 FD 及其 MTU
   *        FD configuration; fd_mode.fd_enabled selects FD mode and the link MTU.
   * @note 构造会阻塞到接口出现且配置成功；需要 CAP_NET_ADMIN。
   *       Blocks until the interface exists and is configured; needs CAP_NET_ADMIN.
   */
  LinuxCANFD(const std::string& interface_name, const FDCAN::Configuration& config,
             size_t rx_stack_size = DEFAULT_RX_STACK_SIZE, bool realtime = true);

  /// 关闭接口并等待接收线程退出 / Close the interface and join RX.
  ~LinuxCANFD() override;

  /// 驱动不可复制 / Drivers cannot be copied.
  LinuxCANFD(const LinuxCANFD&) = delete;
  /// 驱动不可复制赋值 / Drivers cannot be copy-assigned.
  LinuxCANFD& operator=(const LinuxCANFD&) = delete;

  /// 停止并等待接收线程；从回调调用返回 BUSY / Stop/join RX; BUSY in RX callbacks.
  /// 构造已完成打开，Close 只用于主动停止 / The constructor opens; Close only stops.
  ErrorCode Close();
  /// 当前打开的接口是否允许 FD / Whether the open interface allows FD traffic.
  bool SupportsFD() const { return fd_available_.load(std::memory_order_acquire); }

  /**
   * @brief 通过 rtnetlink 修改接口配置并重新拉起 / Reconfigure the link via rtnetlink.
   *
   * 只使用内核 C API（NETLINK_ROUTE + IFLA_CAN_*），不依赖 libsocketcan 或 iproute2。
   * 每次有效调用都会将接口短暂 down/up，即使配置与当前相同；需要 CAP_NET_ADMIN，
   * 且不应与 Close 并发。
   * Uses only kernel C APIs; every valid call cycles the link down/up, even if the
   * configuration is unchanged. CAP_NET_ADMIN is needed; Close must not run concurrently.
   *
   * CAN::Configuration 重载只改仲裁相位和工作模式，FD 开关与数据相位保持原状；
   * FDCAN::Configuration 重载同时按 fd_mode 开关 FD、设置数据相位，并据此切换 MTU
   * （72 或 16）。BRS 是逐帧发送选项，由 fd_mode.brs 更新；ESI 由控制器决定，忽略。
   * The CAN overload keeps FD state; the FDCAN overload also applies the data phase and
   * the FD enable flag, switching the MTU accordingly. BRS is a per-frame TX option
   * updated from fd_mode.brs; ESI is controller-driven and ignored.
   * bitrate 和时序全为 0 时保留当前时序；mode 各开关始终按配置置位或清除，
   * 因此默认 CAN::Configuration 并非“不修改”。
   * Zero bitrate/timing preserves current timing; mode switches are always set or
   * cleared as configured, so a default CAN::Configuration is not a no-op.
   *
   * @note 仅支持按 bitrate 配置；完整显式位时序返回 NOT_SUPPORT，不完整时序返回 ARG_ERR。
   *       Only bitrate-based timing is supported; complete explicit timing returns
   *       NOT_SUPPORT, and incomplete timing returns ARG_ERR.
   * @return 成功返回 OK，参数非法返回 ARG_ERR，显式时序返回 NOT_SUPPORT。
   *         OK on success, ARG_ERR for invalid parameters, NOT_SUPPORT for explicit
   *         timing; other failures return the mapped kernel error.
   */
  ErrorCode SetConfig(const CAN::Configuration& config) override;
  /// FD 配置重载 / FD configuration overload.
  ErrorCode SetConfig(const FDCAN::Configuration& config) override;
  /// 未查询控制器时钟，返回 0 / Controller clock is not queried; returns zero.
  uint32_t GetClockFreq() const override { return 0; }
  /// 通过 rtnetlink 读取 TEC/REC 与控制器状态 / Read TEC/REC and controller state via
  /// rtnetlink.
  /// @param state 输出参数，返回 TEC/REC 与 bus-off 等状态 / TEC/REC and bus-off
  /// snapshot.
  /// @return 查询成功返回 OK，接口不存在返回 NOT_FOUND / OK on success, NOT_FOUND when
  ///         the interface does not exist.
  ErrorCode GetErrorState(CAN::ErrorState& state) const override;

  /// OK 仅表示内核接受帧，并非总线 ACK / OK means kernel acceptance, not bus ACK.
  /// @param pack 经典 CAN 帧，dlc 上限 8 / Classic frame with a maximum DLC of 8.
  ErrorCode AddMessage(const ClassicPack& pack) override;
  /// 非标准长度向上补零到合法 FD 长度 / Noncanonical FD lengths are padded with zeros.
  /// @param pack FD 帧，len 上限 64 / FD frame with a length of at most 64 bytes.
  ErrorCode AddMessage(const FDPack& pack) override;

 private:
  /// 接收线程入口 / RX thread entry point.
  static void* ReceiveEntry(void* self);
  /// 打开 socket 并绑定接口，必要时等待接口 up / Open and bind, waiting for the link.
  ErrorCode OpenSocket();
  /// 接收致命错误后关闭并重连 / Close and reconnect after a fatal receive error.
  void ReopenSocket();
  /// 给线程属性套用 SCHED_FIFO，失败返回 false / Apply SCHED_FIFO; false if unsupported.
  bool ConfigureRealtime(pthread_attr_t& attr) const;
  /// 创建接收线程，必要时退回默认调度策略 / Create RX, retrying without the RT policy.
  ErrorCode StartReceiveThread();
  /// 批量读取并分发，直到停止或发生致命错误 / Drain frames until stopped or fatal.
  void ReceiveLoop();
  /// 分发一帧 / Dispatch one received frame.
  void Dispatch(const struct canfd_frame& frame, size_t size);
  /// 每个内核错误帧只分发一个错误事件 / Dispatch one LibXR event per kernel error frame.
  void DispatchError(const struct canfd_frame& frame, uint8_t length);
  /// 加锁写入，避免与 Close 关闭 fd 竞争 / Serialize the write against fd close.
  ErrorCode Send(const void* frame, size_t size);

  /// 校验接口名长度与内嵌空字符 / Validate the interface name.
  void ValidateInterfaceName() const;
  /// 等待接口出现 / Wait for the interface to appear.
  void WaitForInterface();
  /// 创建唤醒描述符、打开 socket 并启动接收线程 / Wake fd, socket, RX thread.
  void StartRuntime();
  /// 用 rtnetlink 应用配置并拉起接口 / Apply link configuration through rtnetlink.
  ErrorCode ApplyLinkConfig(const FDCAN::Configuration& config, bool apply_fd);
  /// 公共的 SetConfig 实现 / Shared SetConfig implementation.
  ErrorCode SetConfig(const FDCAN::Configuration& config, bool apply_fd);
  /// 按当前 MTU 刷新 socket 的 FD 帧能力 / Refresh FD-frame capability from the MTU.
  ErrorCode RefreshSocketCapabilities();

  std::string interface_name_;          ///< 接口名 / Interface name.
  std::atomic<bool> enable_fd_{false};  ///< 是否允许 FD / Whether FD is requested.
  std::atomic<bool> brs_{false};        ///< FD 发送是否带 BRS / Request BRS on FD TX.
  size_t rx_stack_size_;                ///< 接收线程栈大小 / RX thread stack size.
  bool realtime_;                       ///< 接收线程是否用 SCHED_FIFO / Use SCHED_FIFO.
  int socket_fd_ = -1;                  ///< CAN raw socket / CAN raw socket.
  int wake_fd_ = -1;                    ///< 停止唤醒用 eventfd / eventfd waking RX.
  pthread_t receive_thread_{};          ///< 接收线程句柄 / RX thread handle.
  bool thread_started_ = false;         ///< 接收线程是否已启动 / Whether RX was started.
  std::atomic<bool> open_{false};       ///< 是否已打开 / Whether the driver is open.
  std::atomic<bool> stopping_{false};   ///< 停止或致命错误 / Stop or fatal error.
  std::atomic<bool> fd_available_{false};  ///< 当前 socket 是否允许 FD / Current socket.
  std::mutex io_mutex_;  ///< 保护 socket 配置与生命周期 / Guards config/lifetime.
};

}  // namespace LibXR
