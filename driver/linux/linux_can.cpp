// SocketCAN 驱动实现：经典 CAN 与 CAN FD / SocketCAN driver: classic CAN and CAN FD.
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include "linux_can.hpp"

#include <linux/can.h>
#include <linux/can/error.h>
#include <linux/can/netlink.h>
#include <linux/can/raw.h>
#include <linux/if_link.h>
#include <linux/netlink.h>
#include <linux/rtnetlink.h>
#include <net/if.h>
#include <poll.h>
#include <sched.h>
#include <sys/eventfd.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <climits>
#include <cstdio>
#include <cstring>

#include "linux_can_error.hpp"
#include "thread.hpp"

namespace
{
using namespace LibXR;

// 单次唤醒最多连续读取的帧数，摊薄 poll 系统调用 / Frames drained per poll wakeup.
constexpr int RX_BATCH = 32;
// poll 超时：即使停止唤醒丢失，接收线程也会复核停止标志 / Stop-request safety net.
constexpr int RX_POLL_TIMEOUT_MS = 100;
// 接收缓冲放大到 1 MiB（内核再翻倍），缓解突发丢帧 / Larger RX buffer for FD bursts.
constexpr int RX_SOCKET_BUFFER = 1 << 20;
// 接收线程 SCHED_FIFO 优先级相对最小值的偏移 / SCHED_FIFO offset over the minimum.
constexpr int RX_REALTIME_PRIORITY = 1;
// rtnetlink 查询超时（微秒） / rtnetlink query timeout in microseconds.
constexpr long NETLINK_TIMEOUT_US = 200000;
// 等待接口 up 或重连的间隔 / Interval between interface-ready or reconnect attempts.
constexpr uint32_t INTERFACE_RETRY_MS = 100;

// 即使接收线程先于 pthread_create 返回执行，也能识别回调中的 Close 调用。
// Identify callbacks even if the worker runs before pthread_create returns to Open.
thread_local const LinuxCANFD* active_receiver = nullptr;

ErrorCode SocketError(int error)
{
  switch (error)
  {
    case EAGAIN:
    case ENOBUFS:
      return ErrorCode::FULL;
    case ENODEV:
    case ENXIO:
    case ENOENT:
      return ErrorCode::NOT_FOUND;
    case ENETDOWN:
    case ENETUNREACH:
    case EBADF:
      return ErrorCode::STATE_ERR;
    case EINVAL:
      return ErrorCode::ARG_ERR;
    case EBUSY:
      return ErrorCode::BUSY;
    case ETIMEDOUT:
      return ErrorCode::TIMEOUT;
    case EOPNOTSUPP:
    case ENOPROTOOPT:
    case EPROTONOSUPPORT:
    case EAFNOSUPPORT:
      return ErrorCode::NOT_SUPPORT;
    default:
      return ErrorCode::FAILED;
  }
}

// ---------------------------------------------------------------------------
// 接口配置：只使用内核 C API（NETLINK_ROUTE / rtnetlink），不依赖 libsocketcan。
// Link configuration: implemented only with kernel C APIs (NETLINK_ROUTE).
// ---------------------------------------------------------------------------
constexpr uint32_t MODE_MASK = CAN_CTRLMODE_LOOPBACK | CAN_CTRLMODE_LISTENONLY |
                               CAN_CTRLMODE_3_SAMPLES | CAN_CTRLMODE_ONE_SHOT;

// 通过临时 AF_INET socket 读取管理状态，不依赖 CAN socket 的生命周期。
// Read administrative state through a temporary AF_INET socket.
ErrorCode ReadLinkFlags(const char* interface_name, unsigned int* flags)
{
  const int fd = socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
  if (fd < 0)
  {
    return SocketError(errno);
  }
  ifreq request{};
  std::memcpy(request.ifr_name, interface_name, std::strlen(interface_name) + 1);
  ErrorCode result = ErrorCode::OK;
  if (ioctl(fd, SIOCGIFFLAGS, &request) < 0)
  {
    result = SocketError(errno);
  }
  else
  {
    *flags = static_cast<unsigned int>(request.ifr_flags);
  }
  (void)close(fd);
  return result;
}

ErrorCode ReadLinkMtu(const char* interface_name, int* mtu)
{
  const int fd = socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
  if (fd < 0)
  {
    return SocketError(errno);
  }
  ifreq request{};
  std::memcpy(request.ifr_name, interface_name, std::strlen(interface_name) + 1);
  ErrorCode result = ErrorCode::OK;
  if (ioctl(fd, SIOCGIFMTU, &request) < 0)
  {
    result = SocketError(errno);
  }
  else
  {
    *mtu = request.ifr_mtu;
  }
  (void)close(fd);
  return result;
}

bool HasBitTiming(const CAN::BitTiming& timing)
{
  return timing.brp != 0U || timing.prop_seg != 0U || timing.phase_seg1 != 0U ||
         timing.phase_seg2 != 0U || timing.sjw != 0U;
}

bool CompleteBitTiming(const CAN::BitTiming& timing)
{
  return timing.brp != 0U && timing.prop_seg != 0U && timing.phase_seg1 != 0U &&
         timing.phase_seg2 != 0U && timing.sjw != 0U;
}

bool HasDataTimingFields(const FDCAN::Configuration& config)
{
  const auto& timing = config.data_timing;
  return timing.brp != 0U || timing.prop_seg != 0U || timing.phase_seg1 != 0U ||
         timing.phase_seg2 != 0U || timing.sjw != 0U;
}

bool HasDataTiming(const FDCAN::Configuration& config)
{
  return config.data_bitrate != 0U || HasDataTimingFields(config);
}

bool CompleteDataTiming(const FDCAN::Configuration& config)
{
  const auto& timing = config.data_timing;
  return timing.brp != 0U && timing.prop_seg != 0U && timing.phase_seg1 != 0U &&
         timing.phase_seg2 != 0U && timing.sjw != 0U;
}

bool ValidSamplePoint(float sample_point)
{
  return sample_point >= 0.0f && sample_point < 1.0f;
}

// bitrate 和时序全为 0 时保留当前时序；mode 开关始终按配置写入。
// Zero bitrate/timing preserves current timing; mode switches are always applied.
// 显式时序必须给全，其余由内核按 bitrate 计算 / Explicit timing must be complete.
bool ValidNominalTiming(uint32_t bitrate, const CAN::BitTiming& timing)
{
  if (bitrate == 0U && !HasBitTiming(timing))
  {
    return true;
  }
  if (HasBitTiming(timing))
  {
    return CompleteBitTiming(timing);
  }
  return true;
}

bool ValidDataTiming(const FDCAN::Configuration& config)
{
  // 只给 data_bitrate 时由内核计算时序；显式时序必须给全。
  // With only data_bitrate the kernel computes the timing; explicit timing must be
  // complete.
  if (!HasDataTimingFields(config))
  {
    return true;
  }
  return CompleteDataTiming(config);
}

bool FdRequested(const FDCAN::Configuration& config, bool apply_fd)
{
  return apply_fd && config.fd_mode.fd_enabled;
}

ErrorCode ValidateConfiguration(const FDCAN::Configuration& config, bool apply_fd)
{
  if (!ValidSamplePoint(config.sample_point) ||
      !ValidSamplePoint(config.data_sample_point) ||
      !ValidNominalTiming(config.bitrate, config.bit_timing) || !ValidDataTiming(config))
  {
    return ErrorCode::ARG_ERR;
  }

  if (FdRequested(config, apply_fd))
  {
    // 内核开启 FD 时要求同时给出仲裁相位与数据相位。
    // Enabling FD requires both nominal and data bittiming.
    if (config.bitrate == 0U && !CompleteBitTiming(config.bit_timing))
    {
      return ErrorCode::ARG_ERR;
    }
    if (config.data_bitrate == 0U && !CompleteDataTiming(config))
    {
      return ErrorCode::ARG_ERR;
    }
  }
  else if (HasDataTiming(config))
  {
    return ErrorCode::ARG_ERR;  // FD 未开启却给了数据相位 / Data phase without FD.
  }
  // 显式时序需要用控制器时钟把 brp 换算为 tq，目前尚未查询该时钟。
  // Explicit timing needs the controller clock to convert brp to tq.
  if (HasBitTiming(config.bit_timing) || HasDataTimingFields(config))
  {
    return ErrorCode::NOT_SUPPORT;
  }
  return ErrorCode::OK;
}

uint32_t ModeFlags(const CAN::Mode& mode)
{
  uint32_t flags = 0U;
  if (mode.loopback)
  {
    flags |= CAN_CTRLMODE_LOOPBACK;
  }
  if (mode.listen_only)
  {
    flags |= CAN_CTRLMODE_LISTENONLY;
  }
  if (mode.triple_sampling)
  {
    flags |= CAN_CTRLMODE_3_SAMPLES;
  }
  if (mode.one_shot)
  {
    flags |= CAN_CTRLMODE_ONE_SHOT;
  }
  return flags;
}

can_bittiming ToKernelBittiming(uint32_t bitrate, float sample_point,
                                const CAN::BitTiming& timing)
{
  can_bittiming result{};
  result.bitrate = bitrate;
  if (sample_point > 0.0f)
  {
    // 内核 sample_point 单位是千分之一（0~999）。
    // The kernel expresses sample_point in one-tenth of a percent.
    const uint32_t scaled = static_cast<uint32_t>(sample_point * 1000.0f + 0.5f);
    result.sample_point = scaled > 999U ? 999U : scaled;
  }
  result.brp = timing.brp;
  result.prop_seg = timing.prop_seg;
  result.phase_seg1 = timing.phase_seg1;
  result.phase_seg2 = timing.phase_seg2;
  result.sjw = timing.sjw;
  return result;
}

can_bittiming ToKernelDataBittiming(const FDCAN::Configuration& config)
{
  can_bittiming result{};
  result.bitrate = config.data_bitrate;
  if (config.data_sample_point > 0.0f)
  {
    const uint32_t scaled =
        static_cast<uint32_t>(config.data_sample_point * 1000.0f + 0.5f);
    result.sample_point = scaled > 999U ? 999U : scaled;
  }
  result.brp = config.data_timing.brp;
  result.prop_seg = config.data_timing.prop_seg;
  result.phase_seg1 = config.data_timing.phase_seg1;
  result.phase_seg2 = config.data_timing.phase_seg2;
  result.sjw = config.data_timing.sjw;
  return result;
}

// RTM_NEWLINK 消息构造器 / RTM_NEWLINK message builder.
class NetlinkMessage
{
 public:
  NetlinkMessage(uint16_t type, uint16_t flags, int ifindex, uint16_t interface_flags,
                 uint32_t interface_change)
  {
    std::memset(buffer_.data(), 0, buffer_.size());
    auto* header = reinterpret_cast<nlmsghdr*>(buffer_.data());
    header->nlmsg_len = static_cast<uint32_t>(NLMSG_LENGTH(sizeof(ifinfomsg)));
    header->nlmsg_type = type;
    header->nlmsg_flags = flags;
    header->nlmsg_seq = 1U;
    auto* info = static_cast<ifinfomsg*>(NLMSG_DATA(header));
    info->ifi_family = AF_UNSPEC;
    info->ifi_index = ifindex;
    info->ifi_flags = interface_flags;
    info->ifi_change = interface_change;
  }

  bool Add(uint16_t type, const void* data, size_t length)
  {
    auto* header = reinterpret_cast<nlmsghdr*>(buffer_.data());
    const size_t offset = NLMSG_ALIGN(header->nlmsg_len);
    const size_t attribute_length = RTA_LENGTH(length);
    const size_t total = RTA_ALIGN(attribute_length);
    if (offset + total > buffer_.size())
    {
      return false;
    }
    auto* attribute = reinterpret_cast<rtattr*>(buffer_.data() + offset);
    std::memset(attribute, 0, total);
    attribute->rta_type = type;
    attribute->rta_len = static_cast<uint16_t>(attribute_length);
    if (data != nullptr && length != 0U)
    {
      std::memcpy(RTA_DATA(attribute), data, length);
    }
    header->nlmsg_len = static_cast<uint32_t>(offset + total);
    return true;
  }

  bool BeginNested(uint16_t type, rtattr*& attribute)
  {
    auto* header = reinterpret_cast<nlmsghdr*>(buffer_.data());
    const size_t offset = NLMSG_ALIGN(header->nlmsg_len);
    const size_t total = RTA_ALIGN(RTA_LENGTH(0));
    if (offset + total > buffer_.size())
    {
      return false;
    }
    attribute = reinterpret_cast<rtattr*>(buffer_.data() + offset);
    std::memset(attribute, 0, total);
    attribute->rta_type = static_cast<uint16_t>(type | NLA_F_NESTED);
    attribute->rta_len = static_cast<uint16_t>(RTA_LENGTH(0));
    header->nlmsg_len = static_cast<uint32_t>(offset + total);
    return true;
  }

  void EndNested(rtattr* attribute)
  {
    auto* header = reinterpret_cast<nlmsghdr*>(buffer_.data());
    const auto* base = reinterpret_cast<const uint8_t*>(buffer_.data());
    const auto* start = reinterpret_cast<const uint8_t*>(attribute);
    attribute->rta_len = static_cast<uint16_t>(base + header->nlmsg_len - start);
  }

  ErrorCode Send(int fd) const
  {
    const auto* header = reinterpret_cast<const nlmsghdr*>(buffer_.data());
    ssize_t written = 0;
    do
    {
      written = send(fd, buffer_.data(), header->nlmsg_len, 0);
    } while (written < 0 && errno == EINTR);
    if (written < 0)
    {
      return SocketError(errno);
    }
    return written == static_cast<ssize_t>(header->nlmsg_len) ? ErrorCode::OK
                                                              : ErrorCode::FAILED;
  }

 private:
  alignas(4) std::array<uint8_t, 512> buffer_{};
};

int OpenNetlinkRoute()
{
  const int fd = socket(AF_NETLINK, SOCK_RAW | SOCK_CLOEXEC, NETLINK_ROUTE);
  if (fd < 0)
  {
    return -1;
  }
  timeval timeout{};
  timeout.tv_usec = NETLINK_TIMEOUT_US;
  (void)setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
  return fd;
}

// 等待 RTM_NEWLINK 的 ACK；超时返回 TIMEOUT，内核错误码按 SocketError 映射。
ErrorCode WaitNetlinkAck(int fd)
{
  std::array<uint8_t, 1024> reply{};
  for (;;)
  {
    ssize_t size = 0;
    do
    {
      size = recv(fd, reply.data(), reply.size(), 0);
    } while (size < 0 && errno == EINTR);
    if (size < 0)
    {
      return (errno == EAGAIN || errno == EWOULDBLOCK) ? ErrorCode::TIMEOUT
                                                       : SocketError(errno);
    }
    if (size == 0)
    {
      return ErrorCode::FAILED;
    }
    int remaining = static_cast<int>(size);
    for (auto* header = reinterpret_cast<nlmsghdr*>(reply.data());
         NLMSG_OK(header, remaining); header = NLMSG_NEXT(header, remaining))
    {
      if (header->nlmsg_type != NLMSG_ERROR)
      {
        continue;
      }
      if (header->nlmsg_len < NLMSG_LENGTH(sizeof(nlmsgerr)))
      {
        return ErrorCode::FAILED;
      }
      const auto* error = reinterpret_cast<const nlmsgerr*>(NLMSG_DATA(header));
      return error->error == 0 ? ErrorCode::OK : SocketError(-error->error);
    }
  }
}

ErrorCode SendLinkState(int netlink_fd, unsigned int ifindex, bool up)
{
  NetlinkMessage message(RTM_NEWLINK, NLM_F_REQUEST | NLM_F_ACK,
                         static_cast<int>(ifindex), up ? IFF_UP : 0U, IFF_UP);
  const ErrorCode result = message.Send(netlink_fd);
  if (result != ErrorCode::OK)
  {
    return result;
  }
  return WaitNetlinkAck(netlink_fd);
}

ErrorCode SendLinkMtu(int netlink_fd, unsigned int ifindex, int mtu)
{
  NetlinkMessage message(RTM_NEWLINK, NLM_F_REQUEST | NLM_F_ACK,
                         static_cast<int>(ifindex), 0U, 0U);
  if (!message.Add(static_cast<uint16_t>(IFLA_MTU), &mtu, sizeof(mtu)))
  {
    return ErrorCode::NO_BUFF;
  }
  const ErrorCode result = message.Send(netlink_fd);
  if (result != ErrorCode::OK)
  {
    return result;
  }
  return WaitNetlinkAck(netlink_fd);
}

// 按 iproute2 生成 IFLA_LINKINFO/IFLA_INFO_DATA 的方式下发 bittiming/ctrlmode。
// Submit bittiming/ctrlmode the same way iproute2 builds IFLA_LINKINFO.
ErrorCode SendLinkConfig(int netlink_fd, unsigned int ifindex,
                         const FDCAN::Configuration& config, bool apply_fd)
{
  NetlinkMessage message(RTM_NEWLINK, NLM_F_REQUEST | NLM_F_ACK,
                         static_cast<int>(ifindex), 0U, 0U);

  rtattr* link_info = nullptr;
  if (!message.BeginNested(static_cast<uint16_t>(IFLA_LINKINFO), link_info) ||
      !message.Add(static_cast<uint16_t>(IFLA_INFO_KIND), "can", sizeof("can")))
  {
    return ErrorCode::NO_BUFF;
  }

  rtattr* info_data = nullptr;
  if (!message.BeginNested(static_cast<uint16_t>(IFLA_INFO_DATA), info_data))
  {
    return ErrorCode::NO_BUFF;
  }

  if (config.bitrate != 0U || HasBitTiming(config.bit_timing))
  {
    const can_bittiming bittiming =
        ToKernelBittiming(config.bitrate, config.sample_point, config.bit_timing);
    if (!message.Add(static_cast<uint16_t>(IFLA_CAN_BITTIMING), &bittiming,
                     sizeof(bittiming)))
    {
      return ErrorCode::NO_BUFF;
    }
  }

  const bool fd_enabled = FdRequested(config, apply_fd);
  if (fd_enabled)
  {
    const can_bittiming data_bittiming = ToKernelDataBittiming(config);
    if (!message.Add(static_cast<uint16_t>(IFLA_CAN_DATA_BITTIMING), &data_bittiming,
                     sizeof(data_bittiming)))
    {
      return ErrorCode::NO_BUFF;
    }
  }

  can_ctrlmode control_mode{};
  control_mode.mask = MODE_MASK;
  if (apply_fd)
  {
    control_mode.mask |= CAN_CTRLMODE_FD;
    if (fd_enabled)
    {
      control_mode.flags |= CAN_CTRLMODE_FD;
    }
  }
  control_mode.flags |= ModeFlags(config.mode);
  if (!message.Add(static_cast<uint16_t>(IFLA_CAN_CTRLMODE), &control_mode,
                   sizeof(control_mode)))
  {
    return ErrorCode::NO_BUFF;
  }

  message.EndNested(info_data);
  message.EndNested(link_info);

  const ErrorCode result = message.Send(netlink_fd);
  if (result != ErrorCode::OK)
  {
    return result;
  }
  return WaitNetlinkAck(netlink_fd);
}

bool ValidId(uint32_t id, CAN::Type type)
{
  return id <= ((type == CAN::Type::EXTENDED || type == CAN::Type::REMOTE_EXTENDED)
                    ? CAN_EFF_MASK
                    : CAN_SFF_MASK);
}

canid_t EncodeId(uint32_t id, CAN::Type type)
{
  if (type == CAN::Type::EXTENDED || type == CAN::Type::REMOTE_EXTENDED)
  {
    id |= CAN_EFF_FLAG;
  }
  if (type == CAN::Type::REMOTE_STANDARD || type == CAN::Type::REMOTE_EXTENDED)
  {
    id |= CAN_RTR_FLAG;
  }
  return id;
}

CAN::Type DecodeType(canid_t id)
{
  if (id & CAN_RTR_FLAG)
  {
    return (id & CAN_EFF_FLAG) ? CAN::Type::REMOTE_EXTENDED : CAN::Type::REMOTE_STANDARD;
  }
  return (id & CAN_EFF_FLAG) ? CAN::Type::EXTENDED : CAN::Type::STANDARD;
}

// 把任意长度向上取到 CAN FD 合法长度 / Round a length up to a valid CAN FD length.
constexpr uint8_t WireLength(uint8_t len)
{
  constexpr uint8_t kCanonical[] = {12, 16, 20, 24, 32, 48, 64};
  if (len <= CAN_MAX_DLEN)
  {
    return len;
  }
  for (uint8_t size : kCanonical)
  {
    if (len <= size)
    {
      return size;
    }
  }
  return CANFD_MAX_DLEN;
}

// 解析一层 rtnetlink 属性；CAN 属性嵌在 IFLA_LINKINFO 的 IFLA_INFO_DATA 里。
// Parse one rtnetlink attribute layer; CAN attributes sit inside
// IFLA_LINKINFO -> IFLA_INFO_DATA.
void ParseAttributes(const uint8_t* attributes, size_t attributes_len,
                     CAN::ErrorState& state)
{
  size_t offset = 0;
  while (offset + sizeof(rtattr) <= attributes_len)
  {
    const auto* attribute = reinterpret_cast<const rtattr*>(attributes + offset);
    if (attribute->rta_len < sizeof(rtattr) ||
        offset + attribute->rta_len > attributes_len)
    {
      break;
    }
    const uint8_t* value = attributes + offset + sizeof(rtattr);
    const size_t value_len = attribute->rta_len - sizeof(rtattr);

    switch (attribute->rta_type)
    {
      case IFLA_CAN_BERR_COUNTER:
        if (value_len >= sizeof(can_berr_counter))
        {
          can_berr_counter counter{};
          std::memcpy(&counter, value, sizeof(counter));
          // 内核在 bus-off 时报告 256，而 ErrorState 用 uint8_t 表示，需要裁剪。
          // The kernel reports 256 while bus-off; ErrorState carries a uint8_t.
          state.tx_error_counter =
              static_cast<uint8_t>(counter.txerr > 255U ? 255U : counter.txerr);
          state.rx_error_counter =
              static_cast<uint8_t>(counter.rxerr > 255U ? 255U : counter.rxerr);
        }
        break;
      case IFLA_CAN_STATE:
        if (value_len >= sizeof(uint32_t))
        {
          uint32_t controller_state = 0;
          std::memcpy(&controller_state, value, sizeof(controller_state));
          state.bus_off = controller_state == CAN_STATE_BUS_OFF;
          state.error_passive = controller_state == CAN_STATE_ERROR_PASSIVE;
          state.error_warning = controller_state == CAN_STATE_ERROR_WARNING;
        }
        break;
      case IFLA_LINKINFO:
      case IFLA_INFO_DATA:
        ParseAttributes(value, value_len, state);
        break;
      default:
        break;
    }

    offset += RTA_ALIGN(attribute->rta_len);
  }
}

// 解析 RTM_NEWLINK 回复中的 IFLA_CAN_* 属性 / Parse IFLA_CAN_* from an RTM_NEWLINK
// reply.
ErrorCode ParseCanErrorState(const uint8_t* data, size_t size, CAN::ErrorState& state)
{
  size_t offset = 0;
  while (offset + sizeof(nlmsghdr) <= size)
  {
    const auto* header = reinterpret_cast<const nlmsghdr*>(data + offset);
    if (header->nlmsg_len < sizeof(nlmsghdr) || offset + header->nlmsg_len > size)
    {
      break;
    }
    offset += NLMSG_ALIGN(header->nlmsg_len);

    if (header->nlmsg_type == NLMSG_ERROR)
    {
      return ErrorCode::FAILED;
    }
    if (header->nlmsg_type != RTM_NEWLINK)
    {
      continue;
    }

    const size_t payload = header->nlmsg_len - NLMSG_HDRLEN;
    if (payload < NLMSG_ALIGN(sizeof(ifinfomsg)))
    {
      continue;
    }

    const auto* info = reinterpret_cast<const ifinfomsg*>(NLMSG_DATA(header));
    const uint8_t* attributes =
        reinterpret_cast<const uint8_t*>(info) + NLMSG_ALIGN(sizeof(ifinfomsg));
    const size_t attributes_len = payload - NLMSG_ALIGN(sizeof(ifinfomsg));
    ParseAttributes(attributes, attributes_len, state);
    return ErrorCode::OK;
  }

  return ErrorCode::FAILED;
}

// 用 rtnetlink 查询接口的 CAN 错误状态 / Query CAN error state through rtnetlink.
ErrorCode QueryCanErrorState(const char* interface_name, CAN::ErrorState& state)
{
  const unsigned index = if_nametoindex(interface_name);
  if (index == 0)
  {
    return ErrorCode::NOT_FOUND;
  }

  const int netlink = socket(AF_NETLINK, SOCK_RAW | SOCK_CLOEXEC, NETLINK_ROUTE);
  if (netlink < 0)
  {
    return SocketError(errno);
  }

  timeval timeout{};
  timeout.tv_usec = NETLINK_TIMEOUT_US;
  (void)setsockopt(netlink, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

  struct
  {
    nlmsghdr header;
    ifinfomsg info;
  } request{};
  request.header.nlmsg_len = NLMSG_LENGTH(sizeof(ifinfomsg));
  request.header.nlmsg_type = RTM_GETLINK;
  request.header.nlmsg_flags = NLM_F_REQUEST;
  request.header.nlmsg_seq = 1;
  request.info.ifi_family = AF_UNSPEC;
  request.info.ifi_index = static_cast<int>(index);

  ErrorCode result = ErrorCode::FAILED;
  if (send(netlink, &request, request.header.nlmsg_len, 0) >= 0)
  {
    std::array<uint8_t, 4096> buffer{};
    const ssize_t received = recv(netlink, buffer.data(), buffer.size(), 0);
    if (received > 0)
    {
      result = ParseCanErrorState(buffer.data(), static_cast<size_t>(received), state);
    }
  }

  (void)close(netlink);
  return result;
}
}  // namespace

namespace LibXR
{
LinuxCANFD::LinuxCANFD(const std::string& interface_name, bool enable_fd, bool brs,
                       size_t rx_stack_size, bool realtime)
    : interface_name_(interface_name),
      enable_fd_(enable_fd),
      brs_(brs),
      rx_stack_size_(rx_stack_size),
      realtime_(realtime)
{
  ValidateInterfaceName();
  StartRuntime();
}

LinuxCANFD::LinuxCANFD(const std::string& interface_name,
                       const CAN::Configuration& config, size_t rx_stack_size,
                       bool realtime)
    : interface_name_(interface_name),
      enable_fd_(false),
      brs_(false),
      rx_stack_size_(rx_stack_size),
      realtime_(realtime)
{
  ValidateInterfaceName();

  // 兼容接口：只配置仲裁相位和模式，FD 状态保持系统原状。
  // Compatibility overload: nominal phase and mode only; FD state is untouched.
  FDCAN::Configuration fd_config{};
  fd_config.bitrate = config.bitrate;
  fd_config.sample_point = config.sample_point;
  fd_config.bit_timing = config.bit_timing;
  fd_config.mode = config.mode;

  REQUIRE(ValidateConfiguration(fd_config, false) == ErrorCode::OK);
  WaitForInterface();
  REQUIRE(ApplyLinkConfig(fd_config, false) == ErrorCode::OK);
  StartRuntime();
}

LinuxCANFD::LinuxCANFD(const std::string& interface_name,
                       const FDCAN::Configuration& config, size_t rx_stack_size,
                       bool realtime)
    : interface_name_(interface_name),
      enable_fd_(config.fd_mode.fd_enabled),
      brs_(config.fd_mode.brs),
      rx_stack_size_(rx_stack_size),
      realtime_(realtime)
{
  ValidateInterfaceName();
  REQUIRE(ValidateConfiguration(config, true) == ErrorCode::OK);
  WaitForInterface();
  REQUIRE(ApplyLinkConfig(config, true) == ErrorCode::OK);
  StartRuntime();
}

void LinuxCANFD::ValidateInterfaceName() const
{
  REQUIRE(!interface_name_.empty());
  REQUIRE(interface_name_.size() < IFNAMSIZ);
  REQUIRE(interface_name_.find('\0') == std::string::npos);
}

void LinuxCANFD::WaitForInterface()
{
  while (if_nametoindex(interface_name_.c_str()) == 0U)
  {
    XR_LOG_WARN("CAN interface %s does not exist, retrying...", interface_name_.c_str());
    Thread::Sleep(INTERFACE_RETRY_MS);
  }
}

void LinuxCANFD::StartRuntime()
{
  wake_fd_ = eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
  REQUIRE(wake_fd_ >= 0);

  // 与 LinuxUART 一致：构造期完成等待与打开，调用方不再处理 down 状态。
  // Like LinuxUART: the constructor waits and opens, so callers never see down.
  REQUIRE(OpenSocket() == ErrorCode::OK);
  REQUIRE(StartReceiveThread() == ErrorCode::OK);
  open_.store(true, std::memory_order_release);
}

LinuxCANFD::~LinuxCANFD()
{
  const auto result = Close();
  REQUIRE(result == ErrorCode::OK);
}

bool LinuxCANFD::ConfigureRealtime(pthread_attr_t& attr) const
{
  const int min_priority = sched_get_priority_min(SCHED_FIFO);
  const int max_priority = sched_get_priority_max(SCHED_FIFO);
  if (min_priority < 0 || max_priority < 0 ||
      max_priority - min_priority < RX_REALTIME_PRIORITY)
  {
    return false;
  }

  sched_param param{};
  param.sched_priority = min_priority + RX_REALTIME_PRIORITY;
  if (pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED) != 0)
  {
    return false;
  }
  if (pthread_attr_setschedpolicy(&attr, SCHED_FIFO) != 0)
  {
    return false;
  }
  return pthread_attr_setschedparam(&attr, &param) == 0;
}

ErrorCode LinuxCANFD::StartReceiveThread()
{
  const size_t stack_size =
      std::max(rx_stack_size_, static_cast<size_t>(PTHREAD_STACK_MIN));

  // 无 CAP_SYS_NICE 时带 SCHED_FIFO 的创建会失败，退回默认调度策略再试一次。
  // Without CAP_SYS_NICE the SCHED_FIFO creation fails; retry with default scheduling.
  auto create = [&](bool realtime)
  {
    pthread_attr_t attr;
    if (pthread_attr_init(&attr) != 0)
    {
      return EAGAIN;
    }
    (void)pthread_attr_setstacksize(&attr, stack_size);
    if (realtime)
    {
      (void)ConfigureRealtime(attr);
    }
    const int result = pthread_create(&receive_thread_, &attr, ReceiveEntry, this);
    (void)pthread_attr_destroy(&attr);
    return result;
  };

  int result = create(realtime_);
  if (result != 0 && realtime_)
  {
    result = create(false);
  }
  if (result != 0)
  {
    return SocketError(result);
  }

  thread_started_ = true;
  return ErrorCode::OK;
}

ErrorCode LinuxCANFD::OpenSocket()
{
  const int fd = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK | SOCK_CLOEXEC, CAN_RAW);
  if (fd < 0)
  {
    return SocketError(errno);
  }
  auto fail = [fd](ErrorCode error)
  {
    (void)close(fd);
    return error;
  };

  ifreq request{};
  std::memcpy(request.ifr_name, interface_name_.c_str(), interface_name_.size() + 1);

  // 等待接口出现并 up；与 LinuxUART 构造期等待设备节点一致，重连也走这条路径。
  // Wait for the interface to appear and come up; reconnect uses the same path.
  for (;;)
  {
    if (ioctl(fd, SIOCGIFINDEX, &request) == 0)
    {
      ifreq flags{};
      std::memcpy(flags.ifr_name, interface_name_.c_str(), interface_name_.size() + 1);
      if (ioctl(fd, SIOCGIFFLAGS, &flags) == 0 && (flags.ifr_flags & IFF_UP) != 0)
      {
        break;
      }
    }
    if (stopping_.load(std::memory_order_acquire))
    {
      return fail(ErrorCode::STATE_ERR);
    }
    XR_LOG_WARN("CAN interface %s is not up, retrying...", interface_name_.c_str());
    Thread::Sleep(INTERFACE_RETRY_MS);
  }
  const int index = request.ifr_ifindex;

  const can_err_mask_t errors = CAN_ERR_MASK;
  if (setsockopt(fd, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &errors, sizeof(errors)) < 0)
  {
    return fail(SocketError(errno));
  }

  // 按突发量放大接收缓冲；失败不致命 / Enlarge the RX buffer; failure is not fatal.
  int receive_buffer = RX_SOCKET_BUFFER;
  (void)setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &receive_buffer, sizeof(receive_buffer));

  sockaddr_can address{};
  address.can_family = AF_CAN;
  address.can_ifindex = index;
  if (bind(fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0)
  {
    return fail(SocketError(errno));
  }

  // 与 SetConfig 的能力刷新串行，避免重连发布过期的 FD 状态。
  // Serialize with SetConfig's refresh so reconnect cannot publish stale FD state.
  std::lock_guard<std::mutex> lock(io_mutex_);
  ifreq mtu{};
  std::memcpy(mtu.ifr_name, interface_name_.c_str(), interface_name_.size() + 1);
  if (ioctl(fd, SIOCGIFMTU, &mtu) < 0)
  {
    return fail(SocketError(errno));
  }
  if (mtu.ifr_mtu != CAN_MTU && mtu.ifr_mtu != CANFD_MTU)
  {
    return fail(ErrorCode::NOT_SUPPORT);
  }

  const bool allow_fd =
      enable_fd_.load(std::memory_order_acquire) && mtu.ifr_mtu == CANFD_MTU;
  if (allow_fd)
  {
    const int enable = 1;
    if (setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable, sizeof(enable)) < 0)
    {
      return fail(SocketError(errno));
    }
  }

  socket_fd_ = fd;
  fd_available_.store(allow_fd, std::memory_order_release);
  return ErrorCode::OK;
}

void LinuxCANFD::ReopenSocket()
{
  {
    std::lock_guard<std::mutex> lock(io_mutex_);
    if (socket_fd_ >= 0)
    {
      (void)close(socket_fd_);
      socket_fd_ = -1;
    }
  }
  fd_available_.store(false, std::memory_order_release);

  // 接口被 down 或内核报告错误后自动重连，行为对齐 LinuxUART 的重连线程。
  // Reconnect after the link goes down, mirroring LinuxUART's reconnect loop.
  while (!stopping_.load(std::memory_order_acquire))
  {
    const ErrorCode result = OpenSocket();
    if (result == ErrorCode::OK)
    {
      return;
    }
    if (result != ErrorCode::STATE_ERR)
    {
      XR_LOG_ERROR("Cannot reopen CAN interface %s", interface_name_.c_str());
    }
    Thread::Sleep(INTERFACE_RETRY_MS);
  }
}

ErrorCode LinuxCANFD::Close()
{
  if (active_receiver == this)
  {
    return ErrorCode::BUSY;
  }

  ErrorCode result = ErrorCode::OK;
  if (thread_started_)
  {
    stopping_.store(true, std::memory_order_release);
    if (wake_fd_ >= 0)
    {
      const uint64_t wake = 1;
      ssize_t written;
      do
      {
        written = write(wake_fd_, &wake, sizeof(wake));
      } while (written < 0 && errno == EINTR);
      // EAGAIN 表示已有唤醒事件；其它错误也不提前返回，poll 超时会兜底。
      // EAGAIN means a wake is already pending; other errors are still not fatal
      // because the poll timeout lets the RX thread observe the stop request.
      if (written < 0 && errno != EAGAIN && errno != EWOULDBLOCK)
      {
        result = ErrorCode::FAILED;
      }
    }
    if (pthread_join(receive_thread_, nullptr) != 0)
    {
      result = ErrorCode::FAILED;
    }
    thread_started_ = false;
  }

  if (wake_fd_ >= 0)
  {
    (void)close(wake_fd_);
    wake_fd_ = -1;
  }

  // join 之后才关闭 socket，并持锁等待在途 write 结束。
  // Close the socket only after the join, holding the lock to drain in-flight sends.
  {
    std::lock_guard<std::mutex> lock(io_mutex_);
    if (socket_fd_ >= 0)
    {
      (void)close(socket_fd_);
      socket_fd_ = -1;
    }
  }

  open_.store(false, std::memory_order_release);
  fd_available_.store(false, std::memory_order_release);
  return result;
}

ErrorCode LinuxCANFD::SetConfig(const CAN::Configuration& config)
{
  // 兼容接口：只改仲裁相位和模式，FD 状态与数据相位保持原状。
  // Compatibility overload: nominal phase and mode only; FD state is untouched.
  FDCAN::Configuration fd_config{};
  fd_config.bitrate = config.bitrate;
  fd_config.sample_point = config.sample_point;
  fd_config.bit_timing = config.bit_timing;
  fd_config.mode = config.mode;
  return SetConfig(fd_config, false);
}

ErrorCode LinuxCANFD::SetConfig(const FDCAN::Configuration& config)
{
  return SetConfig(config, true);
}

ErrorCode LinuxCANFD::SetConfig(const FDCAN::Configuration& config, bool apply_fd)
{
  if (!open_.load(std::memory_order_acquire))
  {
    return ErrorCode::STATE_ERR;
  }

  const ErrorCode validation = ValidateConfiguration(config, apply_fd);
  if (validation != ErrorCode::OK)
  {
    return validation;
  }

  const ErrorCode result = ApplyLinkConfig(config, apply_fd);
  if (result != ErrorCode::OK)
  {
    XR_LOG_WARN("Cannot configure CAN interface %s: %d", interface_name_.c_str(),
                static_cast<int>(result));
    return result;
  }

  if (apply_fd)
  {
    enable_fd_.store(config.fd_mode.fd_enabled, std::memory_order_release);
    brs_.store(config.fd_mode.brs, std::memory_order_release);
  }
  return RefreshSocketCapabilities();
}

ErrorCode LinuxCANFD::ApplyLinkConfig(const FDCAN::Configuration& config, bool apply_fd)
{
  const unsigned int index = if_nametoindex(interface_name_.c_str());
  if (index == 0U)
  {
    return ErrorCode::NOT_FOUND;
  }

  unsigned int previous_flags = 0U;
  ErrorCode result = ReadLinkFlags(interface_name_.c_str(), &previous_flags);
  if (result != ErrorCode::OK)
  {
    return result;
  }

  const int netlink_fd = OpenNetlinkRoute();
  if (netlink_fd < 0)
  {
    return SocketError(errno);
  }

  // bittiming/ctrlmode 只允许在接口 down 时修改，因此先停接口再配置。
  // bittiming/ctrlmode may only change while the link is down.
  if ((previous_flags & IFF_UP) != 0U)
  {
    result = SendLinkState(netlink_fd, index, false);
  }

  if (result == ErrorCode::OK)
  {
    result = SendLinkConfig(netlink_fd, index, config, apply_fd);
  }

  if (result == ErrorCode::OK && apply_fd)
  {
    // FD 开关决定 MTU；部分内核不会自动切换，这里显式补齐。
    // FD mode decides the MTU; some kernels do not switch it implicitly.
    const int target_mtu = config.fd_mode.fd_enabled ? CANFD_MTU : CAN_MTU;
    int mtu = 0;
    result = ReadLinkMtu(interface_name_.c_str(), &mtu);
    if (result == ErrorCode::OK && mtu != target_mtu)
    {
      result = SendLinkMtu(netlink_fd, index, target_mtu);
    }
  }

  if (result == ErrorCode::OK)
  {
    result = SendLinkState(netlink_fd, index, true);
  }

  (void)close(netlink_fd);

  if (result != ErrorCode::OK && (previous_flags & IFF_UP) != 0U)
  {
    // 失败时尽力恢复原来的 up 状态 / Best-effort restore of the previous up state.
    const int restore_fd = OpenNetlinkRoute();
    if (restore_fd >= 0)
    {
      (void)SendLinkState(restore_fd, index, true);
      (void)close(restore_fd);
    }
  }
  return result;
}

ErrorCode LinuxCANFD::RefreshSocketCapabilities()
{
  int mtu = 0;
  const ErrorCode result = ReadLinkMtu(interface_name_.c_str(), &mtu);
  if (result != ErrorCode::OK)
  {
    return result;
  }

  const bool use_fd = enable_fd_.load(std::memory_order_acquire) && mtu == CANFD_MTU;

  std::lock_guard<std::mutex> lock(io_mutex_);
  if (socket_fd_ >= 0 && open_.load(std::memory_order_acquire))
  {
    const int enable = use_fd ? 1 : 0;
    if (setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable, sizeof(enable)) <
        0)
    {
      return SocketError(errno);
    }
  }
  // 即使 socket 正在重连，也先发布目标能力，重连会按同样规则重建。
  // Publish the target capability even while a reconnect is in progress; the reopen
  // derives the same state from the MTU and enable_fd_.
  fd_available_.store(use_fd, std::memory_order_release);
  return ErrorCode::OK;
}

ErrorCode LinuxCANFD::GetErrorState(CAN::ErrorState& state) const
{
  state = CAN::ErrorState{};
  return QueryCanErrorState(interface_name_.c_str(), state);
}

ErrorCode LinuxCANFD::Send(const void* frame, size_t size)
{
  std::lock_guard<std::mutex> lock(io_mutex_);
  if (socket_fd_ < 0 || !open_.load(std::memory_order_acquire) ||
      stopping_.load(std::memory_order_acquire))
  {
    return ErrorCode::STATE_ERR;
  }

  ssize_t result;
  do
  {
    result = write(socket_fd_, frame, size);
  } while (result < 0 && errno == EINTR);

  if (result < 0)
  {
    return SocketError(errno);
  }
  return result == static_cast<ssize_t>(size) ? ErrorCode::OK : ErrorCode::FAILED;
}

ErrorCode LinuxCANFD::AddMessage(const ClassicPack& pack)
{
  if (pack.type > Type::REMOTE_EXTENDED || !ValidId(pack.id, pack.type))
  {
    return ErrorCode::ARG_ERR;
  }
  if (pack.dlc > CAN_MAX_DLEN)
  {
    return ErrorCode::SIZE_ERR;
  }

  can_frame frame{};
  frame.can_id = EncodeId(pack.id, pack.type);
  frame.len = pack.dlc;
  if (!(frame.can_id & CAN_RTR_FLAG))
  {
    std::memcpy(frame.data, pack.data, pack.dlc);
  }
  return Send(&frame, CAN_MTU);
}

ErrorCode LinuxCANFD::AddMessage(const FDPack& pack)
{
  if (pack.type > Type::EXTENDED || !ValidId(pack.id, pack.type))
  {
    return ErrorCode::ARG_ERR;
  }
  if (pack.len > CANFD_MAX_DLEN)
  {
    return ErrorCode::SIZE_ERR;
  }
  if (!open_.load(std::memory_order_acquire))
  {
    return ErrorCode::STATE_ERR;
  }
  if (!fd_available_.load(std::memory_order_acquire))
  {
    return ErrorCode::NOT_SUPPORT;
  }

  canfd_frame frame{};
  frame.can_id = EncodeId(pack.id, pack.type);
  frame.len = WireLength(pack.len);
  frame.flags = brs_.load(std::memory_order_acquire) ? CANFD_BRS : 0;
  std::memcpy(frame.data, pack.data, pack.len);
  return Send(&frame, CANFD_MTU);
}

void* LinuxCANFD::ReceiveEntry(void* self)
{
  auto* driver = static_cast<LinuxCANFD*>(self);
  active_receiver = driver;

  char name[16] = {};
  (void)std::snprintf(name, sizeof(name), "can:%s", driver->interface_name_.c_str());
  (void)pthread_setname_np(pthread_self(), name);

  driver->ReceiveLoop();
  active_receiver = nullptr;
  return nullptr;
}

void LinuxCANFD::ReceiveLoop()
{
  while (!stopping_.load(std::memory_order_acquire))
  {
    // socket_fd_ 可能被重连替换，每轮重新读取 / The fd may change on reconnect.
    pollfd descriptors[2] = {{socket_fd_, POLLIN, 0}, {wake_fd_, POLLIN, 0}};
    const int result = poll(descriptors, 2, RX_POLL_TIMEOUT_MS);
    if (result < 0)
    {
      if (errno == EINTR)
      {
        continue;
      }
      ReopenSocket();
      continue;
    }
    if (result == 0)
    {
      // 超时只用于复核停止标志 / A timeout only rechecks the stop flag.
      continue;
    }
    if (descriptors[1].revents != 0)
    {
      break;
    }

    bool link_lost = false;
    if (descriptors[0].revents & POLLIN)
    {
      // 一次唤醒排空最多 RX_BATCH 帧，减少 poll/read 系统调用次数。
      // Drain up to RX_BATCH frames per wakeup to amortize poll/read syscalls.
      for (int budget = RX_BATCH; budget > 0; --budget)
      {
        canfd_frame frame{};
        const ssize_t size = read(socket_fd_, &frame, sizeof(frame));
        if (size == CAN_MTU || size == CANFD_MTU)
        {
          Dispatch(frame, static_cast<size_t>(size));
          continue;
        }
        if (size < 0 && errno == EINTR)
        {
          continue;
        }
        if (size < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
        {
          break;
        }
        link_lost = true;
        break;
      }
    }
    else if (descriptors[0].revents & (POLLERR | POLLHUP | POLLNVAL))
    {
      link_lost = true;
    }

    if (link_lost && !stopping_.load(std::memory_order_acquire))
    {
      XR_LOG_WARN("CAN interface %s stopped, reconnecting...", interface_name_.c_str());
      ReopenSocket();
    }
  }
  stopping_.store(true, std::memory_order_release);
}

void LinuxCANFD::Dispatch(const canfd_frame& frame, size_t size)
{
  if (size == CAN_MTU && frame.len <= CAN_MAX_DLEN)
  {
    if (frame.can_id & CAN_ERR_FLAG)
    {
      DispatchError(frame, frame.len);
      return;
    }

    ClassicPack pack{};
    pack.dlc = frame.len;
    pack.type = DecodeType(frame.can_id);
    pack.id =
        frame.can_id & ((frame.can_id & CAN_EFF_FLAG) ? CAN_EFF_MASK : CAN_SFF_MASK);
    if (!(frame.can_id & CAN_RTR_FLAG))
    {
      std::memcpy(pack.data, frame.data, pack.dlc);
    }
    CAN::OnMessage(pack, false);
    return;
  }

  if (size == CANFD_MTU && fd_available_.load(std::memory_order_acquire) &&
      frame.len <= CANFD_MAX_DLEN && !(frame.can_id & (CAN_RTR_FLAG | CAN_ERR_FLAG)))
  {
    FDPack pack{};
    pack.type = DecodeType(frame.can_id);
    pack.id =
        frame.can_id & ((frame.can_id & CAN_EFF_FLAG) ? CAN_EFF_MASK : CAN_SFF_MASK);
    pack.len = frame.len;
    std::memcpy(pack.data, frame.data, pack.len);
    FDCAN::OnMessage(pack, false);
  }
}

void LinuxCANFD::DispatchError(const canfd_frame& frame, uint8_t length)
{
  // 保留内核错误字节；事件 ID 使用 LibXR 的跨平台错误空间。
  // Linux error bytes are retained; IDs use LibXR's portable error namespace.
  ClassicPack pack{};
  pack.type = Type::ERROR;
  pack.dlc = length;
  const uint8_t copy_length = length < CAN_MAX_DLEN ? length : CAN_MAX_DLEN;
  std::memcpy(pack.data, frame.data, copy_length);

  pack.id = FromErrorID(LinuxCANDetail::FrameErrorID(frame, copy_length));
  CAN::OnMessage(pack, false);
}
}  // namespace LibXR
