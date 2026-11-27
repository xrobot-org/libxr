#pragma once

#include <cstddef>
#include <cstdint>

#include "core.hpp"
#include "ep.hpp"

namespace LibXR::USB
{

/**
 * @brief USB端点池类 / USB endpoint pool class
 *
 * 按「端点号 × 方向」二维索引管理固定的 Endpoint 指针。所有 Endpoint 对象由后端在
 * 初始化时通过 Put() 填入，池只负责登记、分配、查找与回收，不负责对象生命周期。
 * 端点 0 的 IN/OUT 不进入索引数组，由 SetEndpoint0/GetEndpoint0* 单独管理。另有一个
 * 只读查询 PacketSizeFor，供设备类在绑定之前写描述符。
 *
 * Manages fixed Endpoint pointers indexed by (endpoint-number, direction). All Endpoint
 * objects are populated by the backend at init time via Put(); the pool only registers,
 * allocates, looks up and recycles them, and does not own their lifetime. Endpoint 0's
 * IN/OUT are kept out of the index array and managed by SetEndpoint0/GetEndpoint0*. A
 * read-only query, PacketSizeFor, lets device classes write their descriptors before
 * any bind.
 *
 * @note 登记只在初始化时进行；分配、回收和按地址查找只在设备的 owner 中进行（配置切换、
 *       复位、端点级标准请求），因此使用普通数组与非原子状态。端点的串行执行和启动闸门
 *       不在本类，见 DeviceService。
 *       Registration happens at init only; allocation, release and lookup by address
 *       happen only in the device's owner (configuration change, reset, endpoint-level
 *       standard requests), so a plain array with non-atomic state is used. Serialized
 *       execution and the start gate of the endpoints are not in this class; see
 *       DeviceService.
 */
class EndpointPool
{
 public:
  /// 构造空池 / Construct an empty pool
  EndpointPool() = default;

  EndpointPool(const EndpointPool&) = delete;
  EndpointPool& operator=(const EndpointPool&) = delete;

  /**
   * @brief 登记一个端点到池中 / Register an endpoint into the pool
   *
   * 按端点自身的号（GetNumber）与可用方向（AvailableDirection）入位。端点 0 不应
   * 通过此接口登记（应使用 SetEndpoint0）。
   * Registered by the endpoint's own number (GetNumber) and available direction
   * (AvailableDirection). Endpoint 0 must not be registered here (use SetEndpoint0).
   *
   * @param ep 待登记的端点对象指针 / Endpoint pointer to register
   * @retval ErrorCode 操作结果（OK/ARG_ERR/FULL）/ Operation result
   * @note 调用者：后端设备构造时，每个数据端点一次；在控制器中断使能之前，不可并发。
   *       Caller: the backend device at construction, once per data endpoint; before
   *       the controller interrupt is enabled, never concurrently.
   */
  ErrorCode Put(Endpoint* ep);

  /**
   * @brief 分配端点 / Allocate endpoint
   *
   * 按指定端点号与方向精确匹配一个已登记且空闲的端点。
   * Precisely matches a registered, available endpoint by the given number and direction.
   *
   * @param[out] ep_info 分配得到的端点对象指针 / Allocated endpoint pointer
   * @param direction 端点方向（IN/OUT）/ Endpoint direction
   * @param ep_num 指定端点号 / Endpoint number
   * @retval ErrorCode 操作结果（OK/NOT_FOUND）/ Operation result
   * @note 调用者：设备类的 BindEndpoints，仅 owner。BOTH 端点分配后两个方向都被占用。
   *       Caller: a device class's BindEndpoints, owner only. Allocating a BOTH
   *       endpoint takes both directions.
   */
  ErrorCode Get(Endpoint*& ep_info, Endpoint::Direction direction,
                Endpoint::EPNumber ep_num);

  /**
   * @brief 回收端点 / Release endpoint
   * @param ep_info 待回收的端点对象指针 / Endpoint pointer to release
   * @retval ErrorCode 操作结果（OK/NOT_FOUND）/ Operation result
   * @note 调用者：设备类的 UnbindEndpoints，仅 owner。只改分配记录，不关闭端点。
   *       Caller: a device class's UnbindEndpoints, owner only. Changes the allocation
   *       record only and does not close the endpoint.
   */
  ErrorCode Release(Endpoint* ep_info);

  /**
   * @brief 查找端点/ Lookup endpoint
   *
   * 端点 0 走 ep0_in_/ep0_out_ 旁路；其余端点仅在「已分配（in-use）」状态下可被查到，
   * 未分配的端点返回 NOT_FOUND（收到端点级请求视为异常）。
   * Endpoint 0 goes through the ep0_in_/ep0_out_ bypass; other endpoints are visible only
   * while allocated (in-use); an unallocated endpoint returns NOT_FOUND (an
   * endpoint-level request for it is treated as an error).
   *
   * @param ep_addr 端点地址，IN端点高位需加0x80 / Endpoint address (0x80 for IN
   * endpoints)
   * @param[out] ans 查找到的端点对象指针 / Found endpoint pointer
   * @retval ErrorCode 操作结果（OK/NOT_FOUND）/ Operation result
   * @note 调用者：设备核心处理端点级标准请求时，仅 owner（读端点的当前配置）。
   *       Caller: the device core handling endpoint-level standard requests, owner only
   *       (reads the endpoint's current configuration).
   */
  ErrorCode FindEndpoint(uint8_t ep_addr, Endpoint*& ans);

  /// 获取端点0的OUT对象；调用者：设备核心 / Get Endpoint 0's OUT object; caller: the
  /// device core
  Endpoint* GetEndpoint0Out();

  /// 获取端点0的IN对象；调用者：设备核心 / Get Endpoint 0's IN object; caller: the
  /// device core
  Endpoint* GetEndpoint0In();

  /**
   * @brief 设置端点0的IN/OUT对象 / Set Endpoint 0 IN/OUT objects
   * @param ep0_in 端点0 IN对象指针 / Endpoint 0 IN pointer
   * @param ep0_out 端点0 OUT对象指针 / Endpoint 0 OUT pointer
   * @note 调用者：后端设备构造时一次，条件同 Put。
   *       Caller: the backend device once at construction, under the conditions of Put.
   */
  void SetEndpoint0(Endpoint* ep0_in, Endpoint* ep0_out);

  /**
   * @brief 某个端点按某个配置在某个速度下会得到的包长 / Packet size an endpoint would
   *        get with a configuration at a speed
   * @param ep_num 端点号；EP0 查 SetEndpoint0 登记的对象 / Endpoint number; EP0 asks
   *        the objects given to SetEndpoint0
   * @param cfg 方向（IN 或 OUT）、类型和包长上限 / Direction (IN or OUT), type and
   *        packet-size upper bound
   * @param speed 总线速度 / Bus speed
   * @return 包长；0 表示该号和方向没有登记端点，或该端点在该速度下不能这样配置。
   *         The packet size; 0 when no endpoint is registered for that number and
   *         direction, or that endpoint cannot be configured so at that speed.
   * @note 只读：不分配、不配置端点，与端点是否已分配、已配置无关，结果等于之后以同一
   *       cfg 在同一速度下 Configure 选出的包长。调用者：设备类写描述符的函数（经组合
   *       层），在 owner 中或在初始化时；不与 Put、SetEndpoint0 并发。
   *       Read-only: allocates and configures nothing and does not depend on whether
   *       the endpoint is allocated or configured; the result equals the size a later
   *       Configure with the same cfg at the same speed chooses. Caller: a device
   *       class's descriptor writer (through the composition), in the owner or at init;
   *       never concurrent with Put or SetEndpoint0.
   */
  uint16_t PacketSizeFor(Endpoint::EPNumber ep_num, const Endpoint::Config& cfg,
                         Speed speed) const;

  /**
   * @brief 某个端点按某个配置在某个速度下会得到的“一个服务区间最多几个包”/ Packets in
   *        one service interval an endpoint would get with a configuration at a speed
   * @param ep_num 端点号 / Endpoint number
   * @param cfg 方向、类型、包长上限和包数上限 / Direction, type and the upper bounds of
   *        the packet size and of the count
   * @param speed 总线速度 / Bus speed
   * @return 包数；0 的条件同 PacketSizeFor，另加 cfg.max_packets_per_interval 为 0。
   *         The count; 0 under the conditions of PacketSizeFor, and when
   *         cfg.max_packets_per_interval is 0.
   * @note 只读，调用条件同 PacketSizeFor；结果等于之后以同一 cfg 在同一速度下
   *       Configure 选出、PacketsPerInterval() 读回的包数。
   *       Read-only, called as PacketSizeFor; the result equals the count a later
   *       Configure with the same cfg at the same speed chooses and PacketsPerInterval()
   *       reads back.
   */
  uint8_t PacketsPerIntervalFor(Endpoint::EPNumber ep_num, const Endpoint::Config& cfg,
                                Speed speed) const;

 private:
  /// 方向数量（OUT=0, IN=1）/ Number of direction slots (OUT=0, IN=1)
  static constexpr size_t DIR_COUNT = 2;
  /// 索引数组容量（端点号 0..EP_MAX_NUM-1）/ Index array size (endpoint numbers)
  static constexpr size_t SLOT_COUNT =
      static_cast<size_t>(Endpoint::EPNumber::EP_MAX_NUM);

  /// 槽占用状态 / Slot usage state
  enum class SlotUse : uint8_t
  {
    AVAILABLE,  ///< 已登记、空闲可分配 / Registered and free to allocate
    IN_USE      ///< 已分配、正在使用 / Allocated and in use
  };

  /// 方向枚举转索引下标（OUT=0, IN=1）/ Direction to array index (OUT=0, IN=1)
  static size_t DirIndex(Endpoint::Direction dir)
  {
    return (dir == Endpoint::Direction::IN) ? 1U : 0U;
  }

  /// 查询用：该号该方向登记的端点，没有时为空 / For the queries: the endpoint
  /// registered at that number and direction, or null
  const Endpoint* Registered(Endpoint::EPNumber ep_num, Endpoint::Direction dir) const;

  /// [端点号][方向] 端点指针；Put 在初始化时写，之后只读 / [num][dir] endpoint; written
  /// by Put at init, read-only afterwards
  Endpoint* slots_[SLOT_COUNT][DIR_COUNT] = {};
  /// 对应槽占用状态；Get、Release 在 owner 中写，FindEndpoint 读 / Per-slot usage;
  /// written by Get and Release in the owner, read by FindEndpoint
  SlotUse use_[SLOT_COUNT][DIR_COUNT] = {};

  /// 端点0 IN对象；SetEndpoint0 写 / Endpoint 0 IN; written by SetEndpoint0
  Endpoint* ep0_in_ = nullptr;
  /// 端点0 OUT对象；SetEndpoint0 写 / Endpoint 0 OUT; written by SetEndpoint0
  Endpoint* ep0_out_ = nullptr;
};

}  // namespace LibXR::USB
