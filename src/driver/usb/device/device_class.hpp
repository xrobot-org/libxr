#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

#include "libxr_def.hpp"
#include "libxr_type.hpp"
#include "usb/core/bos.hpp"
#include "usb/core/core.hpp"
#include "usb/core/desc_cfg.hpp"
#include "usb/core/desc_dev.hpp"
#include "usb/core/ep_pool.hpp"

namespace LibXR::USB
{

class DeviceService;
class DeviceComposition;
class DeviceCore;

/**
 * @class DeviceClass
 * @brief USB 设备类的基类 / Base class of a USB device class
 *
 * 每个 USB 类（CDC、HID、DFU……）派生自本类，实现描述符、端点绑定和控制请求。本类的
 * 虚函数只由组合层和设备核心调用，全部在设备的 owner 中运行（控制器中断或持有 owner 的
 * 线程，见 DeviceService），不得阻塞；同一设备的回调不会并发。
 * Every USB class (CDC, HID, DFU, ...) derives from this class and implements its
 * descriptors, its endpoint binding and its control requests. The virtual functions are
 * called only by the composition and the device core, all in the device's owner (the
 * controller interrupt or the thread that owns, see DeviceService), and must not block;
 * callbacks of one device never run concurrently.
 *
 * @note 描述符和绑定是两步：WriteDescriptors 只写描述符，包长取自端点池的只读查询，
 *       可在任何时候、为任何配置和速度调用；BindEndpoints 只在 SET_CONFIGURATION 时
 *       调用，紧接在同一配置、当前速度的 WriteDescriptors 之后。
 *       Descriptors and binding are two steps: WriteDescriptors only writes descriptors,
 *       taking packet sizes from the endpoint pool's read-only query, and may be called
 *       at any time for any configuration and speed; BindEndpoints is called only on
 *       SET_CONFIGURATION, right after WriteDescriptors of that configuration at the
 *       current speed.
 */
class DeviceClass : public BosCapabilityProvider
{
 public:
  /// 配置描述符头 / Configuration descriptor header
  using Header = ConfigDescriptorItem::Header;
  /// 接口关联描述符 / Interface association descriptor
  using IADDescriptor = ConfigDescriptorItem::IADDescriptor;
  /// 接口描述符 / Interface descriptor
  using InterfaceDescriptor = ConfigDescriptorItem::InterfaceDescriptor;
  /// 端点描述符 / Endpoint descriptor
  using EndpointDescriptor = ConfigDescriptorItem::EndpointDescriptor;

  /// 构造；服务指针和字符串号由组合层构造时写入 / Construct; the service pointer and
  /// the string base are written by the composition's constructor
  DeviceClass() = default;
  /// 经基类指针销毁 / Destroy through the base class
  virtual ~DeviceClass() = default;
  /// 不可复制：组合层按地址持有类 / Not copyable: the composition holds classes by
  /// address
  DeviceClass(const DeviceClass&) = delete;
  /// 不可复制赋值 / Not copy-assignable
  DeviceClass& operator=(const DeviceClass&) = delete;

 protected:
  /// @name 描述符形状：组合层在构造时调用 / Descriptor shape: called by the
  ///       composition at construction
  /// @{

  /// 本类占用的接口个数 / Number of interfaces this class takes
  virtual size_t GetInterfaceCount() const = 0;
  /// 是否在接口前写 IAD / Whether an IAD precedes the interfaces
  virtual bool HasIAD() const = 0;
  /// WriteDescriptors 写出的最大字节数 / Largest number of bytes WriteDescriptors
  /// writes
  virtual size_t GetMaxConfigSize() const = 0;

  /// @}
  /// @name 描述符与绑定：仅 owner / Descriptors and binding: owner only
  /// @{

  /**
   * @brief 写本类的配置描述符块 / Write this class's block of the configuration
   *        descriptor
   * @param pool 端点池，只用于查询 PacketSizeFor 和 PacketsPerIntervalFor / The
   *        endpoint pool, used only for the queries PacketSizeFor and
   *        PacketsPerIntervalFor
   * @param speed 描述符所对应的速度 / Speed the descriptors are for
   * @param first_interface 本类第一个接口号 / Number of the first interface
   * @param block 输出：本类拥有的描述符字节，长度不超过 GetMaxConfigSize()。
   *        Output: descriptor bytes owned by this class, no longer than
   *        GetMaxConfigSize().
   * @return OK；其他：本类在该速度下无法这样描述（例如查询得 0），整个配置不可用。
   *         OK; otherwise this class cannot be described so at that speed (for
   *         example the query returned 0) and the whole configuration is unusable.
   * @note 调用者：组合层，回答 GET_DESCRIPTOR(CONFIGURATION / OTHER_SPEED) 时，以及
   *       BindEndpoints 之前。可在已绑定时为别的配置或速度调用，因此只写 block，不改
   *       运行状态，不碰端点对象。block 在下一次调用前有效。
   *       Caller: the composition, when answering GET_DESCRIPTOR (CONFIGURATION or
   *       OTHER_SPEED) and before BindEndpoints. It may be called while bound, for
   *       another configuration or speed, so it writes block only, changes no runtime
   *       state and never touches endpoint objects. block stays valid until the next
   *       call.
   */
  virtual ErrorCode WriteDescriptors(const EndpointPool& pool, Speed speed,
                                     uint8_t first_interface, ConstRawData& block) = 0;

  /**
   * @brief 取端点、配置并开始工作 / Take the endpoints, configure them and start
   * @param pool 端点池 / The endpoint pool
   * @param first_interface 本类第一个接口号 / Number of the first interface
   * @param in_isr 是否在中断中 / Whether in an ISR
   * @return OK；其他：绑定失败，组合层随后对本配置中已调用过 BindEndpoints 的每个类
   *         （含本类）调用 UnbindEndpoints，并以该错误 STALL SET_CONFIGURATION。
   *         OK; otherwise the bind failed, and the composition then calls
   *         UnbindEndpoints on every class of the configuration whose BindEndpoints
   *         ran (this one included) and stalls SET_CONFIGURATION.
   * @note 每个端点 Configure 之后，MaxPacketSize() 和 PacketsPerInterval() 读回的包长和
   *       包数必须等于刚写的描述符中的 wMaxPacketSize，不等时返回错误。
   *       After each endpoint's Configure, the size and the count read back with
   *       MaxPacketSize() and PacketsPerInterval() must equal the wMaxPacketSize of the
   *       descriptor just written; otherwise return an error.
   */
  [[nodiscard]] virtual ErrorCode BindEndpoints(EndpointPool& pool,
                                                uint8_t first_interface, bool in_isr) = 0;

  /**
   * @brief 关闭并交回本类的端点 / Close and return this class's endpoints
   * @param pool 端点池 / The endpoint pool
   * @param in_isr 是否在中断中 / Whether in an ISR
   * @note 撤销 BindEndpoints 做过的一切，包括失败的一半。每个端点 Close 返回时请求都已
   *       回调（硬件上的停止可以还没结束，见 Endpoint::Close），之后 Release。
   *       Undoes whatever BindEndpoints did, a failed half included. Each endpoint's
   *       Close returns with every request called back (the stop in hardware may still
   *       be pending, see Endpoint::Close), then it is released.
   */
  virtual void UnbindEndpoints(EndpointPool& pool, bool in_isr) = 0;

  /**
   * @brief 单功能设备改写设备描述符的类字段 / A single-function device rewrites the
   *        class fields of the device descriptor
   * @return OK：已改写；NOT_SUPPORT（默认）：不改。
   *         OK: rewritten; NOT_SUPPORT (default): left alone.
   * @note 调用者：设备核心构造时一次，且只在设备不是复合设备时。
   *       Caller: the device core, once at construction, only for a non-composite
   *       device.
   */
  virtual ErrorCode WriteDeviceDescriptor(DeviceDescriptor& descriptor)
  {
    UNUSED(descriptor);
    return ErrorCode::NOT_SUPPORT;
  }

  /**
   * @brief SET_INTERFACE / SET_INTERFACE
   * @param in_isr 是否在中断中，传给切换中的 Close、Configure 和 Submit / Whether in
   *        an ISR; passed on to the Close, Configure and Submit of the switch
   * @param itf 接口号（设备内编号，取自 wIndex），属于本类 / Interface number (device
   *        numbering, from wIndex), one of this class's
   * @param alt 主机要求的备用设置（wValue，0-255）/ Alternate setting the host asks
   *        for (wValue, 0-255)
   * @return OK：已切换，核心发状态阶段；其他：核心 STALL。默认只接受 0。
   *         OK: switched and the core sends the status stage; otherwise the core
   *         stalls. The default accepts 0 only.
   * @note 切换中的 Close 返回时请求都已回调。硬件上的停止可以还没结束：Configure 的
   *       返回值这时同样是最终结果，只是硬件配置推迟到停止结束（见
   *       Endpoint::Configure）。
   *       A Close during the switch returns with every request called back. The stop in
   *       hardware may still be pending: the result of Configure is final then too, only
   *       the hardware configuration is deferred to the end of the stop (see
   *       Endpoint::Configure).
   */
  virtual ErrorCode SetAltSetting(bool in_isr, uint8_t itf, uint8_t alt)
  {
    UNUSED(in_isr);
    UNUSED(itf);
    return alt == 0U ? ErrorCode::OK : ErrorCode::NOT_SUPPORT;
  }

  /// GET_INTERFACE；OK 时写入 alt，否则核心回答 0 / GET_INTERFACE; on OK writes alt,
  /// otherwise the core answers 0
  virtual ErrorCode GetAltSetting(uint8_t itf, uint8_t& alt)
  {
    UNUSED(itf);
    UNUSED(alt);
    return ErrorCode::NOT_SUPPORT;
  }

  /// 端点地址是否属于本类；用于端点为接收者的类请求 / Whether the endpoint address is
  /// this class's; routes class requests whose recipient is an endpoint
  virtual bool OwnsEndpoint(uint8_t ep_addr) const
  {
    UNUSED(ep_addr);
    return false;
  }

  /// @}
  /// @name 构造时查询 / Queried at construction
  /// @{

  /// 第 n 个接口的字符串（UTF-8）；nullptr 表示没有 / String of the n-th interface
  /// (UTF-8); nullptr for none
  virtual const char* GetInterfaceString(size_t local_interface_index) const
  {
    UNUSED(local_interface_index);
    return nullptr;
  }

  /// 要 WinUSB 绑定时返回接口 GUID，否则 nullptr / The interface GUID when WinUSB
  /// should bind, else nullptr
  virtual const char* GetWinUsbInterfaceGuid() const { return nullptr; }

  /// 控制 OUT 数据阶段要的缓冲字节数；核心取所有类的最大值，至少 64 / Bytes of control
  /// OUT data stage this class needs; the core takes the maximum, at least 64
  virtual size_t GetControlReceiveCapacity() const { return 64U; }

  /// @}
  /// @name 控制请求：仅 owner / Control requests: owner only
  /// @{

  /**
   * @brief 接口为接收者的标准 GET_DESCRIPTOR（类特定描述符）/ Standard
   *        GET_DESCRIPTOR whose recipient is an interface (class-specific descriptor)
   * @param in_isr owner 是否在中断中 / Whether the owner is in an ISR
   * @param setup 本次传输的 SETUP；wIndex 是本类的一个接口号 / SETUP of this transfer;
   *        wIndex is one of this class's interface numbers
   * @param data 输出：描述符；核心按 wLength 截短，有效到本次传输结束。
   *        Output: the descriptor; the core truncates it to wLength; valid until the
   *        transfer ends.
   * @return OK；其他：核心 STALL / OK; otherwise the core stalls
   */
  virtual ErrorCode OnGetDescriptor(bool in_isr, const SetupPacket& setup,
                                    ConstRawData& data)
  {
    UNUSED(in_isr);
    UNUSED(setup);
    UNUSED(data);
    return ErrorCode::NOT_SUPPORT;
  }

  /**
   * @brief 类或厂商请求的 SETUP 阶段 / SETUP stage of a class or vendor request
   * @param in_isr owner 是否在中断中 / Whether the owner is in an ISR
   * @param setup 本次传输的 SETUP / SETUP of this transfer
   * @param in_data 输出：IN 请求的数据，核心按 wLength 截短并补结尾零长度包；必须保持
   *        有效且不变，直到本次传输的 OnControlComplete 或 OnControlAbort。OUT 和无数据
   *        阶段的请求不写它。
   *        Output: the data of an IN request; the core truncates it to wLength and
   *        adds the closing zero-length packet. It must stay valid and unchanged until
   *        this transfer's OnControlComplete or OnControlAbort. OUT and no-data
   *        requests leave it alone.
   * @return OK：接受，本次传输之后恰好得到一次 OnControlComplete 或 OnControlAbort；
   *         其他：核心 STALL，不再回调。
   *         OK: accepted, and the transfer later gets exactly one OnControlComplete or
   *         OnControlAbort; otherwise the core stalls and makes no further call.
   * @note 接收者为接口或端点时才交给类（按接口号或 OwnsEndpoint 找类）。
   *       Only requests whose recipient is an interface or an endpoint reach a class
   *       (found by interface number or OwnsEndpoint).
   */
  virtual ErrorCode OnControlRequest(bool in_isr, const SetupPacket& setup,
                                     ConstRawData& in_data)
  {
    UNUSED(in_isr);
    UNUSED(setup);
    UNUSED(in_data);
    return ErrorCode::NOT_SUPPORT;
  }

  /**
   * @brief OUT 数据阶段收齐 / The OUT data stage has been received in full
   * @param in_isr owner 是否在中断中 / Whether the owner is in an ISR
   * @param setup 本次传输的 SETUP，即本类在 OnControlRequest 中接受的那个 / SETUP of
   *        this transfer, the one this class accepted in OnControlRequest
   * @param data 收到的 wLength 字节，只在本次调用内有效 / The wLength bytes received,
   *        valid during this call only
   * @return OK：核心发状态阶段；其他：核心 STALL 并调用 OnControlAbort。
   *         OK: the core sends the status stage; otherwise the core stalls and calls
   *         OnControlAbort.
   * @note 只用于 OUT 请求，IN 请求不调用。
   *       For OUT requests only; never called for an IN request.
   */
  virtual ErrorCode OnControlData(bool in_isr, const SetupPacket& setup,
                                  ConstRawData data)
  {
    UNUSED(in_isr);
    UNUSED(setup);
    UNUSED(data);
    return ErrorCode::NOT_SUPPORT;
  }

  /// 被接受的传输的状态阶段完成 / The status stage of an accepted transfer completed
  virtual void OnControlComplete(bool in_isr, const SetupPacket& setup)
  {
    UNUSED(in_isr);
    UNUSED(setup);
  }

  /**
   * @brief 被接受的传输没有完成 / An accepted transfer did not complete
   * @note 被新的 SETUP 取代、复位、Deinit、断开、停止失败，或 OnControlData 出错后
   *       STALL 时调用。
   *       Called when the transfer is superseded by a new SETUP, on reset, Deinit,
   *       disconnect or a failed stop, or when it stalls after an OnControlData error.
   */
  virtual void OnControlAbort(bool in_isr, const SetupPacket& setup)
  {
    UNUSED(in_isr);
    UNUSED(setup);
  }

  /// @}
  /// @name 其他回调：仅 owner / Other callbacks: owner only
  /// @{

  /// RequestService 之后在 owner 中调用一次或多次合并为一次 / Runs in the owner after
  /// RequestService; several requests may merge into one call
  virtual void OnService(bool in_isr) { UNUSED(in_isr); }

  /**
   * @brief 主机清除了本类某端点的 HALT / The host cleared the halt of one of this
   *        class's endpoints
   * @param in_isr owner 是否在中断中 / Whether the owner is in an ISR
   * @param ep_addr 端点地址 / Endpoint address
   * @note 端点已 ClearStall 回到 IDLE；被 STALL 中止的接收需要类重新挂。在 ABORTED 的
   *       完成回调里重挂即可：STALLED 期间挂的接收排队到 ClearStall 后启动，这样的类
   *       不需要本函数。
   *       The endpoint has been cleared back to IDLE; a receive the halt aborted must
   *       be armed again by the class. Arming it again in the ABORTED completion is
   *       enough: a receive armed while STALLED waits until ClearStall, and a class doing
   *       so needs no override here.
   */
  virtual void OnHaltCleared(bool in_isr, uint8_t ep_addr)
  {
    UNUSED(in_isr);
    UNUSED(ep_addr);
  }

  /**
   * @brief 帧起始 / Start of frame
   * @param in_isr owner 是否在中断中 / Whether the owner is in an ISR
   * @param frame 帧号（11 位）；0xffff 表示后端不知道 / Frame number (11 bits);
   *        0xffff when the backend does not know it
   * @param microframe 微帧 0-7；0xff 表示后端只报告整帧 / Microframe 0-7; 0xff when
   *        the backend reports whole frames only
   * @note 只调用当前配置中的类。帧间隔等推算由类自己做，核心不存帧号。
   *       Called only for the classes of the current configuration. The class derives
   *       intervals itself; the core keeps no frame number.
   * @note 总线在运行时每个后端都报告帧起始，但不保证每个（微）帧恰好一次：靠读帧号
   *       上报的后端在两次调用之间可能已经过了几个（微）帧，只报最新的一个。
   *       Every backend reports starts of frame while the bus runs, but not exactly once
   *       per (micro)frame: a backend that reports by reading the frame number may have
   *       let several (micro)frames pass between two calls and reports the latest only.
   */
  virtual void OnStartOfFrame(bool in_isr, uint16_t frame, uint8_t microframe)
  {
    UNUSED(in_isr);
    UNUSED(frame);
    UNUSED(microframe);
  }

  /// @}

  /**
   * @brief 请 owner 调用本类的 OnService / Ask the owner to call this class's OnService
   * @note 任何上下文，设备构造完成后；可能就地运行 owner，OnService 可能在返回前运行。
   *       Any context, once the device is constructed; may run the owner in place, so
   *       OnService can run before this returns.
   */
  void RequestService(bool in_isr);

  /// 本类第 n 个接口分到的字符串号，0 表示没有 / String index assigned to the n-th
  /// interface, 0 for none
  [[nodiscard]] uint8_t GetInterfaceStringIndex(size_t local_interface_index) const;

  /// @name 给派生类的端点辅助函数 / Endpoint helpers for derived classes
  /// @{

  /**
   * @brief 按端点池的查询写一个端点描述符 / Write one endpoint descriptor from the
   *        pool's queries
   * @param pool 端点池，只用于查询 PacketSizeFor 和 PacketsPerIntervalFor / The
   *        endpoint pool, used only for the queries PacketSizeFor and
   *        PacketsPerIntervalFor
   * @param speed 描述符所对应的速度 / Speed the descriptor is for
   * @param number 端点号；与 cfg.direction 一起给出 bEndpointAddress / Endpoint
   *        number; with cfg.direction it gives bEndpointAddress
   * @param cfg 方向、类型、包长上限和包数上限，与之后绑定时的 Configure 相同 /
   *        Direction, type and the bounds of the packet size and count, as given to
   *        Configure at the bind later
   * @param interval 原样写入的 bInterval / bInterval, written as given
   * @param out 输出：描述符；返回 false 时内容无意义 / Output: the descriptor;
   *        meaningless when false is returned
   * @return true；false：该端点在该速度下查询得 0 / true; false: the query returned 0
   *         for that endpoint at that speed
   * @note wMaxPacketSize 是包长加上一个服务区间的包数（USB 2.0 9.6.6）。包数上限为 1
   *       时只查包长。只在 WriteDescriptors 里调用。
   *       wMaxPacketSize is the packet size plus the packets in one service interval
   *       (USB 2.0 9.6.6). With a packet-count bound of 1 only the size is queried.
   *       Called from WriteDescriptors only.
   */
  static bool WriteEndpointDescriptor(const EndpointPool& pool, Speed speed,
                                      Endpoint::EPNumber number,
                                      const Endpoint::Config& cfg, uint8_t interval,
                                      EndpointDescriptor& out);

  /**
   * @brief Configure 一个端点，并与描述符核对包长和包数 / Configure an endpoint and
   *        check the packet size and count against the descriptor
   * @param ep 已取得的端点 / The endpoint already taken
   * @param cfg 传给 Configure 的配置，与写描述符时相同 / Configuration passed to
   *        Configure, the same as when the descriptor was written
   * @param descriptor_field 刚写的描述符中的 wMaxPacketSize / wMaxPacketSize of the
   *        descriptor just written
   * @param in_isr 是否在中断中，传给 Configure / Whether in an ISR; passed on to
   *        Configure
   * @return OK；CHECK_ERR：读回的包长或包数与描述符不同（BindEndpoints 的要求）；
   *         其他：Configure 的错误。
   *         OK; CHECK_ERR: the size or count read back differs from the descriptor (what
   *         BindEndpoints requires); otherwise the error of Configure.
   * @note 仅 owner。CHECK_ERR 时端点保持已配置，由调用者关闭。
   *       Owner only. On CHECK_ERR the endpoint stays configured and the caller closes
   *       it.
   */
  [[nodiscard]] static ErrorCode ConfigureEndpoint(Endpoint& ep,
                                                   const Endpoint::Config& cfg,
                                                   uint16_t descriptor_field,
                                                   bool in_isr);

  /**
   * @brief 取一个端点、Configure、核对包长和包数 / Take one endpoint, configure it and
   *        check the packet size and count
   * @param pool 端点池，从中按 cfg.direction 和 number 取端点 / The endpoint pool the
   *        endpoint is taken from by cfg.direction and number
   * @param number 端点号 / Endpoint number
   * @param cfg 传给 Configure 的配置，与写描述符时相同 / Configuration passed to
   *        Configure, the same as when the descriptor was written
   * @param descriptor_field 刚写的描述符中的 wMaxPacketSize / wMaxPacketSize of the
   *        descriptor just written
   * @param in_isr 是否在中断中，传给 Configure / Whether in an ISR; passed on to
   *        Configure
   * @param ep 输出：取到的端点；Get 失败时为空 / Output: the endpoint taken; null when
   *        Get fails
   * @return OK；NOT_FOUND：Get 失败；其他：同 ConfigureEndpoint。
   *         OK; NOT_FOUND: Get failed; otherwise as ConfigureEndpoint.
   * @note 仅 owner，在 BindEndpoints 里调用。失败时取到的端点留在 ep 里，由随后的
   *       UnbindEndpoints 经 UnbindEndpoint 收回。
   *       Owner only, called from BindEndpoints. On failure the endpoint taken stays in
   *       ep and the following UnbindEndpoints takes it back through UnbindEndpoint.
   */
  [[nodiscard]] static ErrorCode BindEndpoint(EndpointPool& pool,
                                              Endpoint::EPNumber number,
                                              const Endpoint::Config& cfg,
                                              uint16_t descriptor_field, bool in_isr,
                                              Endpoint*& ep);

  /**
   * @brief 关闭并交回一个端点 / Close and return one endpoint
   * @note 仅 owner。ep 为空时什么也不做。先把 ep 置空再 Close，所以 Close 交付的完成回调
   *       看到未绑定；之后 Release。
   *       Owner only. Does nothing when ep is null. ep is cleared before Close, so the
   *       completions Close delivers see "unbound"; then Release.
   */
  static void UnbindEndpoint(EndpointPool& pool, Endpoint*& ep, bool in_isr);

  /// @}

 private:
  friend class DeviceComposition;
  friend class DeviceCore;

  /**
   * @brief RequestService 发布的设备事件位，设备核心按它调用组合层的 ServiceClasses /
   *        Device event bit published by RequestService; the device core calls the
   *        composition's ServiceClasses on it
   */
  static constexpr uint32_t SERVICE_EVENT = 1U << 1U;

  /// 所属设备服务；组合层构造时写，RequestService 读 / Owning device service; written
  /// by the composition's constructor, read by RequestService
  DeviceService* service_ = nullptr;
  /**
   * @brief 有待执行的 OnService / OnService pending
   * @note RequestService 在任何上下文置位，组合层在 owner 中取走。
   *       Set by RequestService anywhere, taken by the composition in the owner.
   */
  std::atomic<uint32_t> service_pending_{0U};
  /// 第一个接口字符串号；组合层构造时写，之后只读 / First interface string index;
  /// written by the composition's constructor, read-only afterwards
  uint8_t interface_string_base_ = 0U;
};

}  // namespace LibXR::USB
