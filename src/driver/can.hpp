#pragma once

#include "libxr.hpp"
#include "message.hpp"

namespace LibXR
{

/**
 * @class CAN
 * @brief CAN 通信抽象类，定义经典 CAN 帧与订阅接口。
 *        Abstract class for CAN communication with classic CAN frames and subscription
 * API.
 */
class CAN
{
 public:
  /**
   * @enum Type
   * @brief CAN 消息类型 / CAN frame type.
   */
  enum class Type : uint8_t
  {
    STANDARD = 0,         ///< 标准数据帧（11-bit ID） / Standard data frame (11-bit ID).
    EXTENDED = 1,         ///< 扩展数据帧（29-bit ID） / Extended data frame (29-bit ID).
    REMOTE_STANDARD = 2,  ///< 标准远程帧 / Standard remote frame.
    REMOTE_EXTENDED = 3,  ///< 扩展远程帧 / Extended remote frame.
    ERROR = 4,            ///< 错误帧（虚拟事件） / Error frame (virtual event).
    TYPE_NUM = 5          ///< 类型数量上界 / Number of frame types.
  };

  /**
   * @struct BitTiming
   * @brief CAN 位时序配置 / Bit timing configuration for CAN.
   */
  struct BitTiming
  {
    uint32_t brp = 0;         ///< 预分频 / Baud rate prescaler.
    uint32_t prop_seg = 0;    ///< 传播段 / Propagation segment.
    uint32_t phase_seg1 = 0;  ///< 相位段 1 / Phase segment 1.
    uint32_t phase_seg2 = 0;  ///< 相位段 2 / Phase segment 2.
    uint32_t sjw = 0;         ///< 同步跳宽 / Synchronization jump width.
  };

  /**
   * @struct Mode
   * @brief CAN 工作模式 / CAN operating mode.
   */
  struct Mode
  {
    bool loopback = false;         ///< 回环模式 / Loopback mode.
    bool listen_only = false;      ///< 只听（静默）模式 / Listen-only (silent) mode.
    bool triple_sampling = false;  ///< 三采样 / Triple sampling.
    bool one_shot = false;         ///< 单次发送模式 / One-shot transmission.
  };

  /**
   * @struct Configuration
   * @brief CAN 配置参数 / CAN configuration parameters.
   */
  struct Configuration
  {
    uint32_t bitrate = 0;       ///< 仲裁相位目标波特率 / Target nominal bitrate.
    float sample_point = 0.0f;  ///< 仲裁相位采样点（0~1） / Nominal sample point (0–1).
    BitTiming bit_timing;       ///< 位时序配置 / Bit timing configuration.
    Mode mode;                  ///< 工作模式 / Operating mode.
  };

  /**
   * @brief 设置 CAN 配置 / Set CAN configuration.
   * @param cfg 配置参数 / Configuration parameters.
   * @return ErrorCode 操作结果 / Error code.
   */
  virtual ErrorCode SetConfig(const CAN::Configuration& cfg) = 0;

  /**
   * @brief 获取 CAN 外设时钟频率（Hz）。
   *        Get CAN peripheral clock frequency in Hz.
   */
  virtual uint32_t GetClockFreq() const = 0;

  /**
   * @struct ErrorState
   * @brief CAN 当前错误状态快照（来自硬件计数器/状态机）。
   *        Snapshot of current CAN controller error state (from HW counters/state).
   */
  struct ErrorState
  {
    uint8_t tx_error_counter = 0;  ///< 发送错误计数 TEC / Transmit error counter (TEC).
    uint8_t rx_error_counter = 0;  ///< 接收错误计数 REC / Receive error counter (REC).

    bool bus_off = false;        ///< 是否处于 BUS-OFF / True if controller is bus-off.
    bool error_passive = false;  ///< 是否处于 Error Passive / True if error-passive.
    bool error_warning = false;  ///< 是否处于 Error Warning / True if error-warning.
  };

  /**
   * @brief 查询当前错误状态（快照）。
   *        Query current CAN controller error state (snapshot).
   *
   * 默认实现返回 ErrorCode::NOT_SUPPORT；具体实现（如 bxCAN/FDCAN）可重载，
   * 从硬件寄存器读取 TEC/REC 及状态位，并填充 ErrorState。
   *
   * @param state 输出参数，用于返回当前错误状态。
   * @return ErrorCode 操作结果；若未实现则返回 ErrorCode::NOT_SUPPORT。
   */
  virtual ErrorCode GetErrorState(ErrorState& state) const
  {
    (void)state;
    return ErrorCode::NOT_SUPPORT;
  }

  /**
   * @brief 构造函数 / Constructor.
   */
  CAN() = default;

  /**
   * @brief 虚析构函数 / Virtual destructor.
   */
  virtual ~CAN() = default;

  LIBXR_PACKED_BEGIN
  /**
   * @struct ClassicPack
   * @brief 经典 CAN 帧数据结构 / Classic CAN frame structure.
   */
  struct ClassicPack
  {
    uint32_t id;      ///< CAN ID（11/29 bit 或 ErrorID） / CAN ID (11/29 bits or
                      ///< ErrorID).
    Type type;        ///< 帧类型 / Frame type.
    uint8_t dlc;      ///< 有效数据长度（0~8） / Data length code (0–8).
    uint8_t data[8];  ///< 数据载荷 / Data payload (up to 8 bytes).
  };
  LIBXR_PACKED_END

  /// 错误 ID 前缀 / Error ID prefix.
  static constexpr uint32_t CAN_ERROR_ID_PREFIX = 0xFFFF0000u;

  /**
   * @enum ErrorID
   * @brief ClassicPack::type == Type::ERROR 时使用的虚拟 ID。
   *        Virtual IDs used when ClassicPack::type == Type::ERROR.
   */
  enum class ErrorID : uint32_t
  {
    CAN_ERROR_ID_GENERIC = CAN_ERROR_ID_PREFIX,
    CAN_ERROR_ID_BUS_OFF = CAN_ERROR_ID_PREFIX + 1,
    CAN_ERROR_ID_ERROR_PASSIVE = CAN_ERROR_ID_PREFIX + 2,
    CAN_ERROR_ID_ERROR_WARNING = CAN_ERROR_ID_PREFIX + 3,
    CAN_ERROR_ID_PROTOCOL = CAN_ERROR_ID_PREFIX + 4,
    CAN_ERROR_ID_ACK = CAN_ERROR_ID_PREFIX + 5,
    CAN_ERROR_ID_STUFF = CAN_ERROR_ID_PREFIX + 6,
    CAN_ERROR_ID_FORM = CAN_ERROR_ID_PREFIX + 7,
    CAN_ERROR_ID_BIT0 = CAN_ERROR_ID_PREFIX + 8,
    CAN_ERROR_ID_BIT1 = CAN_ERROR_ID_PREFIX + 9,
    CAN_ERROR_ID_CRC = CAN_ERROR_ID_PREFIX + 10,
    CAN_ERROR_ID_OTHER = CAN_ERROR_ID_PREFIX + 11
  };

  /// 将 ErrorID 转为 id / Convert ErrorID to ClassicPack::id.
  static constexpr uint32_t FromErrorID(ErrorID e) noexcept
  {
    return static_cast<uint32_t>(e);
  }

  /// 判断 id 是否处于错误 ID 空间 / Check if id is in error ID space.
  static constexpr bool IsErrorId(uint32_t id) noexcept
  {
    return (id & 0xFFFF0000u) == CAN_ERROR_ID_PREFIX;
  }

  /// 将 id 解释为 ErrorID / Interpret id as ErrorID.
  static constexpr ErrorID ToErrorID(uint32_t id) noexcept
  {
    return static_cast<ErrorID>(id);
  }

  /// 回调类型 / Callback type.
  using Callback = LibXR::Callback<const ClassicPack&>;

  /**
   * @enum FilterMode
   * @brief CAN 过滤器模式 / CAN filter mode.
   */
  enum class FilterMode : uint8_t
  {
    ID_MASK = 0,  ///< 掩码匹配：(id & start_id_mask) == end_id_mask。
                  ///< Mask match: (id & start_id_mask) == end_id_mask.
    ID_RANGE = 1  ///< 区间匹配：start_id_mask <= id <= end_id_mask。
                  ///< Range match: start_id_mask <= id <= end_id_mask.
  };

  /**
   * @struct Filter
   * @brief 经典 CAN 订阅过滤器 / Classic CAN subscription filter.
   */
  struct Filter
  {
    FilterMode mode;         ///< 过滤模式 / Filter mode.
    uint32_t start_id_mask;  ///< 起始 ID 或掩码 / Start ID or mask.
    uint32_t end_id_mask;    ///< 结束 ID 或匹配值 / End ID or match value.
    Type type;               ///< 帧类型 / Frame type.
    Callback cb;             ///< 回调函数 / Callback function.
  };

  /**
   * @brief 注册经典 CAN 消息回调。
   *        Register classic CAN message callback.
   *
   * @param cb 回调函数 / Callback function.
   * @param type 帧类型 / Frame type.
   * @param mode 过滤器模式 / Filter mode.
   * @param start_id_mask 起始 ID 或掩码 / Start ID or mask.
   * @param end_id_mask 结束 ID 或匹配值 / End ID or match value.
   */
  void Register(Callback cb, Type type, FilterMode mode = FilterMode::ID_RANGE,
                uint32_t start_id_mask = 0, uint32_t end_id_mask = UINT32_MAX);

  /**
   * @brief 添加经典 CAN 消息 / Add classic CAN message.
   * @param pack 经典 CAN 帧 / Classic CAN frame.
   * @return ErrorCode 操作结果 / Error code.
   */
  virtual ErrorCode AddMessage(const ClassicPack& pack) = 0;

 protected:
  /**
   * @brief 分发接收到的经典 CAN 帧。
   *        Dispatch a received classic CAN frame.
   * @param pack 接收到的帧 / Received frame.
   * @param in_isr 是否在中断上下文中 / True if called in ISR context.
   */
  void OnMessage(const ClassicPack& pack, bool in_isr);

 private:
  /// 按帧类型划分的订阅者链表数组 / Subscriber lists per frame type.
  LockFreeList subscriber_list_[static_cast<uint8_t>(Type::TYPE_NUM)];
};

/**
 * @class FDCAN
 * @brief FDCAN 通信抽象类，扩展支持 CAN FD 帧。
 *        Abstract class for FDCAN communication with CAN FD frame support.
 */
class FDCAN : public CAN
{
 public:
  /**
   * @brief 构造函数 / Constructor.
   */
  FDCAN() = default;

  /**
   * @brief 虚析构函数 / Virtual destructor.
   */
  virtual ~FDCAN() = default;

  LIBXR_PACKED_BEGIN
  /**
   * @struct FDPack
   * @brief CAN FD 帧数据结构 / CAN FD frame structure.
   */
  struct FDPack
  {
    uint32_t id;       ///< CAN ID / CAN ID.
    Type type;         ///< 帧类型 / Frame type.
    uint8_t len;       ///< 数据长度（0~64） / Data length (0–64 bytes).
    uint8_t data[64];  ///< 数据载荷 / Data payload.
  };
  LIBXR_PACKED_END

  using CAN::AddMessage;
  using CAN::FilterMode;
  using CAN::Register;

  /// FD 帧回调类型 / Callback type for FD frames.
  using CallbackFD = LibXR::Callback<const FDPack&>;

  /**
   * @struct Filter
   * @brief FDCAN 订阅过滤器 / FDCAN subscription filter.
   */
  struct Filter
  {
    FilterMode mode;         ///< 过滤模式 / Filter mode.
    uint32_t start_id_mask;  ///< 起始 ID 或掩码 / Start ID or mask.
    uint32_t end_id_mask;    ///< 结束 ID 或匹配值 / End ID or match value.
    Type type;               ///< 帧类型 / Frame type.
    CallbackFD cb;           ///< 回调函数 / Callback function.
  };

  /**
   * @struct DataBitTiming
   * @brief 数据相位位时序配置 / Data phase bit timing configuration.
   */
  struct DataBitTiming
  {
    uint32_t brp = 0;         ///< 预分频 / Prescaler.
    uint32_t prop_seg = 0;    ///< 传播段 / Propagation segment.
    uint32_t phase_seg1 = 0;  ///< 相位段 1 / Phase segment 1.
    uint32_t phase_seg2 = 0;  ///< 相位段 2 / Phase segment 2.
    uint32_t sjw = 0;         ///< 同步跳宽 / Synchronization jump width.
  };

  /**
   * @struct FDMode
   * @brief FDCAN FD 模式配置 / FDCAN FD mode configuration.
   */
  struct FDMode
  {
    bool fd_enabled = false;  ///< 是否启用 CAN FD / Enable CAN FD.
    bool brs = false;         ///< 是否启用 BRS / Enable Bit Rate Switch.
    bool esi = false;         ///< 全局 ESI 标志 / Global ESI flag.
  };

  /**
   * @struct Configuration
   * @brief FDCAN 配置参数，扩展 CAN::Configuration。
   *        FDCAN configuration, extending CAN::Configuration.
   */
  struct Configuration : public CAN::Configuration
  {
    uint32_t data_bitrate = 0;       ///< 数据相位波特率 / Data-phase bitrate.
    float data_sample_point = 0.0f;  ///< 数据相位采样点 / Data-phase sample point.
    DataBitTiming data_timing;       ///< 数据相位位时序 / Data-phase bit timing.
    FDMode fd_mode;                  ///< FD 模式配置 / FD mode configuration.
  };

  /**
   * @brief 设置经典 CAN 配置。
   *        Set classic CAN configuration.
   *
   * 作为 FDCAN 的兼容接口，具体实现可仅使用仲裁相位配置。
   *
   * @param cfg CAN 配置 / CAN configuration.
   * @return ErrorCode 操作结果 / Error code.
   */
  virtual ErrorCode SetConfig(const CAN::Configuration& cfg) override = 0;

  /**
   * @brief 设置 FDCAN 配置。
   *        Set FDCAN configuration.
   * @param cfg FDCAN 配置 / FDCAN configuration.
   * @return ErrorCode 操作结果 / Error code.
   */
  virtual ErrorCode SetConfig(const FDCAN::Configuration& cfg) = 0;

  /**
   * @brief 注册 FDCAN FD 帧回调。
   *        Register FDCAN FD frame callback.
   * @param cb 回调函数 / Callback function.
   * @param type 帧类型 / Frame type.
   * @param mode 过滤器模式 / Filter mode.
   * @param start_id_mask 起始 ID 或掩码 / Start ID or mask.
   * @param end_id_mask 结束 ID 或匹配值 / End ID or match value.
   */
  void Register(CallbackFD cb, Type type, FilterMode mode = FilterMode::ID_RANGE,
                uint32_t start_id_mask = 0, uint32_t end_id_mask = UINT32_MAX);

  /**
   * @brief 添加 FD CAN 消息 / Add FD CAN message.
   * @param pack FD CAN 帧 / FD CAN frame.
   * @return ErrorCode 操作结果 / Error code.
   */
  virtual ErrorCode AddMessage(const FDPack& pack) = 0;

 protected:
  using CAN::OnMessage;

  /**
   * @brief 分发接收到的 FD CAN 帧。
   *        Dispatch a received FD CAN frame.
   * @param pack 接收到的 FD 帧 / Received FD frame.
   * @param in_isr 是否在中断上下文中 / True if called in ISR context.
   */
  void OnMessage(const FDPack& pack, bool in_isr);

 private:
  /// 按帧类型划分的 FD 订阅者链表数组 / FD subscriber lists per frame type.
  LockFreeList subscriber_list_fd_[static_cast<uint8_t>(Type::REMOTE_STANDARD)];
};

}  // namespace LibXR
