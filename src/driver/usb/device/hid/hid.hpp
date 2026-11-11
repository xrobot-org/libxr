#pragma once
#include <atomic>
#include <cstring>

#include "dev_core.hpp"
#include "latest_snapshot.hpp"
#include "libxr_mem.hpp"
#include "usb/core/desc_cfg.hpp"
#include "usb/core/usb_macro_guard.hpp"

namespace LibXR::USB
{

/**
 * @brief 最近发送的输入报告，供 GET_REPORT 读取 / The last input report sent, read by
 *        GET_REPORT
 *
 * 写者是 HID::SendInputReport 独占窗口里的发送方（PublishInputReport /
 * WithdrawInputReport），读者是 owner（GET_REPORT）。每份报告带着发布时的会话号；
 * 会话由 owner 在绑定时推进，会话号不符的报告属于上一次会话，读到时按中立值处理，
 * 所以 owner 不必写快照，也就不会与发送方争当生产者。
 * The writer is the sender inside the exclusive window of HID::SendInputReport
 * (PublishInputReport / WithdrawInputReport); the reader is the owner (GET_REPORT). Each
 * report carries the session it was published in. The owner advances the session on
 * bind, and a report of another session belongs to the previous one and reads as the
 * neutral value, so the owner never writes the snapshot and never competes with the
 * sender as a second producer.
 *
 * @tparam Report 报告类型 / Report type
 */
template <typename Report>
class LastInputReport
{
 public:
  /// @param neutral 会话开始时的报告 / The report a session starts with
  explicit LastInputReport(const Report& neutral)
      : neutral_(neutral), current_{neutral, 0U}, previous_(current_), snapshot_(current_)
  {
  }

  /// 发布 report；只在 SendInputReport 的独占窗口里调用 / Publish report; only inside
  /// the exclusive window of SendInputReport
  void Publish(const Report& report, uint32_t session)
  {
    previous_ = current_;
    current_ = Stamped{report, session};
    snapshot_.Store(current_);
  }

  /// 撤回刚发布的报告，恢复发布前的那份；与 Publish 在同一个独占窗口里调用 /
  /// Take back the report just published and restore the one before it; called in the
  /// same exclusive window as Publish
  void Withdraw()
  {
    current_ = previous_;
    snapshot_.Store(current_);
  }

  /// 本会话最近发送的报告，本会话还没有发送时为中立值；只由 owner 调用，返回值在下次
  /// 调用前保持不变 / The report last sent in this session, the neutral value before the
  /// first; owner only, the result stays valid until the next call
  const Report& Read(uint32_t session)
  {
    (void)snapshot_.LoadLatest(reply_);
    if (reply_.session != session) reply_.report = neutral_;
    return reply_.report;
  }

 private:
  struct Stamped
  {
    Report report;
    uint32_t session;
  };

  Report neutral_;
  Stamped current_;   ///< 只由独占窗口里的发送方写 / Written by the sender in the window
  Stamped previous_;  ///< 同上 / Same
  LatestSnapshot<Stamped> snapshot_;
  Stamped reply_{};  ///< 只由 owner 写 / Owner only
};

/**
 * @brief
 * USB HID（Human Interface Device）基类，支持可选 OUT
 * 端点、自动生成描述符，适合键盘、鼠标、手柄等扩展。
 * USB HID (Human Interface Device) base class with optional OUT endpoint and auto
 * descriptor generation, suitable for extension as keyboard, mouse, gamepad, etc.
 *
 * 输入报告走槽模型：SendInputReport 在任何上下文取槽、拷贝、提交，同一时刻只有一条
 * 报告在途；OUT 报告以租约交给类，默认策略是取走数据后归还租约继续接收。
 * Input reports use the slot model: SendInputReport acquires a slot, copies and submits
 * in any context, with one report in flight at a time. OUT reports arrive as leases;
 * the default policy consumes the report and returns the lease to keep receiving.
 *
 * @tparam REPORT_DESC_LEN 报告描述符长度 / Report descriptor length (bytes)
 * @tparam TX_REPORT_LEN Input Report 长度 / Input report length (bytes)
 * @tparam RX_REPORT_LEN Output Report 长度 / Output report length (bytes) [default: 0]
 * @tparam BOOT_PROTOCOL 接口声明的启动协议：0 无，1 键盘，2 鼠标（HID 1.11 4.3）。
 * 非 0 时接口子类为 1（boot），接口协议为该值；报告在启动协议下必须是 HID 1.11 附录 B
 * 的格式。
 * Boot protocol the interface declares: 0 none, 1 keyboard, 2 mouse (HID 1.11 4.3). When
 * non-zero the interface subclass is 1 (boot) and the interface protocol is this value;
 * the report must then have the format of HID 1.11 appendix B in the boot protocol.
 */
template <size_t REPORT_DESC_LEN, size_t TX_REPORT_LEN, size_t RX_REPORT_LEN = 0,
          uint8_t BOOT_PROTOCOL = 0>
class HID : public DeviceClass
{
  static_assert(BOOT_PROTOCOL <= 2,
                "BOOT_PROTOCOL is 0 (none), 1 (keyboard) or 2 (mouse)");

 public:
  // 暴露单接口的 HID 功能默认使用这个接口字符串。
  // Default interface string for HID functions that expose one interface.
  static constexpr const char* DEFAULT_INTERFACE_STRING = "XRUSB HID";

  /** @brief HID 描述符类型 / HID Descriptor Types */
  enum class HIDDescriptorType : uint8_t
  {
    HID = 0x21,      ///< HID 类描述符 / HID Class Descriptor
    REPORT = 0x22,   ///< 报告描述符 / Report Descriptor
    PHYSICAL = 0x23  ///< 物理描述符 / Physical Descriptor (rarely used)
  };

  /** @brief HID 类请求代码 / HID Class-Specific Requests */
  enum class ClassRequest : uint8_t
  {
    GET_REPORT = 0x01,    ///< 获取报告 / Get Report
    GET_IDLE = 0x02,      ///< 获取空闲率 / Get Idle
    GET_PROTOCOL = 0x03,  ///< 获取协议 / Get Protocol
    SET_REPORT = 0x09,    ///< 设置报告 / Set Report
    SET_IDLE = 0x0A,      ///< 设置空闲率 / Set Idle
    SET_PROTOCOL = 0x0B   ///< 设置协议 / Set Protocol
  };

  /** @brief HID 协议类型 / HID Protocol Types */
  enum class Protocol : uint8_t
  {
    BOOT = 0x00,   ///< 启动协议 / Boot protocol (键盘/鼠标)
    REPORT = 0x01  ///< 报告协议 / Report protocol (通用)
  };

  /** @brief HID 报告类型 / HID Report Types */
  enum class ReportType : uint8_t
  {
    INPUT = 1,   ///< 输入报告 / Input report
    OUTPUT = 2,  ///< 输出报告 / Output report
    FEATURE = 3  ///< 特征报告 / Feature report
  };

  LIBXR_PACKED_BEGIN
  /**
   * @brief HID描述符结构体
   *        HID descriptor structure
   */
  struct HIDDescriptor
  {
    uint8_t bLength;                    ///< 描述符长度 / Descriptor length
    HIDDescriptorType bDescriptorType;  ///< 描述符类型 / Descriptor type (0x21)
    uint16_t bcdHID;                    ///< HID 版本号 / HID class specification release
    uint8_t bCountryCode;               ///< 国家码 / Country code
    uint8_t bNumDescriptors;  ///< 后续描述符数量 / Number of subordinate descriptors
    HIDDescriptorType
        bReportDescriptorType;         ///< 报告描述符类型 / Report descriptor type (0x22)
    uint16_t wReportDescriptorLength;  ///< 报告描述符长度 / Report descriptor length
  };

  /**
   * @brief 包含 IN 端点的描述符块
   * Descriptor block with IN endpoint.
   */
  struct HIDDescBlockIN
  {
    InterfaceDescriptor intf;  ///< 接口描述符 / Interface descriptor
    HIDDescriptor hid;         ///< HID 描述符 / HID descriptor
    EndpointDescriptor ep_in;  ///< IN 端点描述符 / IN endpoint descriptor
  };

  /**
   * @brief 包含 IN+OUT 端点的描述符块
   * Descriptor block with IN and OUT endpoints.
   */
  struct HIDDescBlockINOUT
  {
    InterfaceDescriptor intf;   ///< 接口描述符 / Interface descriptor
    HIDDescriptor hid;          ///< HID 描述符 / HID descriptor
    EndpointDescriptor ep_in;   ///< IN 端点描述符 / IN endpoint descriptor
    EndpointDescriptor ep_out;  ///< OUT 端点描述符 / OUT endpoint descriptor
  };
  LIBXR_PACKED_END

  /**
   * @brief HID 构造函数
   * HID class constructor.
   *
   * @param in_ep_num IN 端点号（必填）/ IN endpoint number (required)
   * @param out_ep_num OUT 端点号（必填，未启用 OUT 时传 EP_INVALID）/ OUT endpoint number
   * (required; pass EP_INVALID when OUT is disabled)
   * @param enable_out_endpoint 是否启用 OUT 端点 / Enable OUT endpoint
   * @param in_ep_interval IN 端点间隔 / IN endpoint interval
   * @param out_ep_interval OUT 端点间隔 / OUT endpoint interval
   * @param interface_string 接口字符串 / Interface string
   */
  HID(Endpoint::EPNumber in_ep_num, Endpoint::EPNumber out_ep_num,
      bool enable_out_endpoint = false, uint8_t in_ep_interval = 10,
      uint8_t out_ep_interval = 10,
      const char* interface_string = DEFAULT_INTERFACE_STRING)
      : in_ep_interval_(in_ep_interval),
        out_ep_interval_(out_ep_interval),
        in_ep_num_(in_ep_num),
        out_ep_num_(out_ep_num),
        enable_out_endpoint_(enable_out_endpoint),
        interface_string_(interface_string)
  {
  }

  const char* GetInterfaceString(size_t local_interface_index) const override
  {
    // HID 只暴露一个接口。
    // HID contributes exactly one interface.
    return (local_interface_index == 0u) ? interface_string_ : nullptr;
  }

 protected:
  /**
   * @brief 初始化 HID 设备，自动选择端点与描述符块
   * Initialize HID device and select descriptor block (IN or IN+OUT).
   * @param endpoint_pool 端点池 / Endpoint pool
   * @param start_itf_num 接口号起始 / Starting interface number
   * @param in_isr 是否在中断上下文 / Whether in ISR
   */
  void BindEndpoints(EndpointPool& endpoint_pool, uint8_t start_itf_num,
                     bool in_isr) override
  {
    // HID 1.11 7.2.6：每次会话从报告协议开始，也不带上一次会话的报告。先推进会话号，
    // 再放开 in_ep_ref_，发送方看到新端点时一定看到新会话号。
    // HID 1.11 7.2.6: every session starts in the report protocol and without the
    // reports of the previous one. The session number advances before in_ep_ref_ is
    // published, so a sender that sees the new endpoint sees the new session.
    protocol_ = Protocol::REPORT;
    session_.fetch_add(1U, std::memory_order_release);
    OnSessionStart();

    inited_ = false;
    itf_num_ = start_itf_num;
    ep_in_ = nullptr;
    ep_out_ = nullptr;

    // 获取IN端点
    auto ans = endpoint_pool.Get(ep_in_, Endpoint::Direction::IN, in_ep_num_);
    ASSERT(ans == ErrorCode::OK);
    // 包长显式：取报告长度与当前速度下中断端点合法包长的较小者；槽装不下一个包时
    // Configure 返回 NO_BUFF，端点进入 ERROR，组合层会拒绝该配置。
    // The packet size is explicit: the smaller of the report length and the largest
    // legal interrupt packet at this speed. NO_BUFF means the slot cannot hold one
    // packet; the endpoint enters ERROR and the composition rejects the configuration.
    (void)ep_in_->Configure(
        {Endpoint::Direction::IN, Endpoint::Type::INTERRUPT,
         static_cast<uint16_t>(LibXR::min<size_t>(
             TX_REPORT_LEN, ep_in_->LargestPacketSize(Endpoint::Type::INTERRUPT)))},
        in_isr);

    if (enable_out_endpoint_)
    {
      ans = endpoint_pool.Get(ep_out_, Endpoint::Direction::OUT, out_ep_num_);
      ASSERT(ans == ErrorCode::OK);
      const uint16_t packet = static_cast<uint16_t>(LibXR::min<size_t>(
          RX_REPORT_LEN, ep_out_->LargestPacketSize(Endpoint::Type::INTERRUPT)));
      // receive_size 与包长一致：每个租约恰是一条报告。
      // receive_size equals one packet: one report per lease.
      (void)ep_out_->Configure(
          {Endpoint::Direction::OUT, Endpoint::Type::INTERRUPT, packet, packet}, in_isr);
    }

    // 填充接口描述符
    desc_.intf = {
        9,                                                   // bLength
        static_cast<uint8_t>(DescriptorType::INTERFACE),     // bDescriptorType
        static_cast<uint8_t>(itf_num_),                      // bInterfaceNumber
        0,                                                   // bAlternateSetting
        static_cast<uint8_t>(enable_out_endpoint_ ? 2 : 1),  // bNumEndpoints
        0x03,                                                // bInterfaceClass (HID)
        static_cast<uint8_t>(BOOT_PROTOCOL != 0 ? 1 : 0),    // bInterfaceSubClass (boot)
        BOOT_PROTOCOL,               // bInterfaceProtocol (1 键盘 / 2 鼠标)
        GetInterfaceStringIndex(0u)  // iInterface
    };

    // 填充HID描述符
    desc_.hid = {9,
                 HIDDescriptorType::HID,
                 0x0111,  // HID v1.11
                 0x00,    // 国家码
                 0x01,    // 只有一个后续描述符（Report Desc）
                 HIDDescriptorType::REPORT,
                 REPORT_DESC_LEN};

    // 填充IN端点描述符
    desc_.ep_in = {
        7,
        static_cast<uint8_t>(DescriptorType::ENDPOINT),
        ep_in_->GetAddress(),
        static_cast<uint8_t>(Endpoint::Type::INTERRUPT),
        ep_in_->MaxPacketSize(),
        in_ep_interval_  // 轮询间隔ms
    };

    // 填充OUT端点描述符（如启用）
    if (enable_out_endpoint_)
    {
      desc_.ep_out = {7,
                      static_cast<uint8_t>(DescriptorType::ENDPOINT),
                      ep_out_->GetAddress(),
                      static_cast<uint8_t>(Endpoint::Type::INTERRUPT),
                      ep_out_->MaxPacketSize(),
                      out_ep_interval_};
    }

    // 设置最终数据指针
    if (enable_out_endpoint_)
    {
      SetData(RawData{reinterpret_cast<uint8_t*>(&desc_), sizeof(HIDDescBlockINOUT)});
    }
    else
    {
      SetData(RawData{reinterpret_cast<uint8_t*>(&desc_), sizeof(HIDDescBlockIN)});
    }

    ep_in_->SetOnComplete(on_data_in_complete_cb_);

    if (enable_out_endpoint_)
    {
      ep_out_->SetOnComplete(on_data_out_complete_cb_);
      // 挂一个槽接收 OUT 报告；租约处理完归还时按 receive_size 重新挂接收。
      // Arm one slot for OUT reports; returning a lease rearms it by receive_size.
      (void)ep_out_->Receive(ep_out_->MaxPacketSize(), in_isr);
    }

    inited_ = true;
    input_in_flight_.store(0U, std::memory_order_release);
    in_ep_ref_.store(ep_in_, std::memory_order_release);
  }

  /**
   * @brief 反初始化 HID 设备
   * Deinitialize HID device.
   * @param endpoint_pool 端点池 / Endpoint pool
   * @param in_isr 是否在中断中 / Whether in ISR
   */
  void UnbindEndpoints(EndpointPool& endpoint_pool, bool in_isr) override
  {
    inited_ = false;
    // 先摘掉引用再关闭：新的 SendInputReport 立刻得到 FAILED，不与新绑定竞争。
    // Drop the reference before closing: a new SendInputReport fails at once and does
    // not race the next binding.
    in_ep_ref_.store(nullptr, std::memory_order_release);
    input_in_flight_.store(0U, std::memory_order_release);
    if (ep_in_)
    {
      ep_in_->Close(in_isr);
      endpoint_pool.Release(ep_in_);
      ep_in_ = nullptr;
    }
    if (ep_out_)
    {
      ep_out_->Close(in_isr);
      endpoint_pool.Release(ep_out_);
      ep_out_ = nullptr;
    }
  }

  /**
   * @brief 获取接口数量
   * Get number of interfaces
   * @return size_t 接口数量 / Number of interfaces
   */
  size_t GetInterfaceCount() override { return 1; }

  /**
   * @brief 检查是否包含IAD
   *        Check if IAD is present
   *
   * @return true
   * @return false
   */
  bool HasIAD() override { return false; }

  bool OwnsEndpoint(uint8_t ep_addr) const override
  {
    if (!inited_)
    {
      return false;
    }

    return ep_in_->GetAddress() == ep_addr ||
           (enable_out_endpoint_ && ep_out_->GetAddress() == ep_addr);
  }

  /**
   * @brief 获取最大配置描述符块长度
   * Get max config descriptor size.
   * @return size_t 描述符块最大长度 / Max size of descriptor block
   */
  size_t GetMaxConfigSize() override
  {
    return enable_out_endpoint_ ? sizeof(HIDDescBlockINOUT) : sizeof(HIDDescBlockIN);
  }

  /**
   * @brief 处理标准请求 GET_DESCRIPTOR（HID/Report 描述符）。
   * Handle standard GET_DESCRIPTOR requests for HID/Report Descriptor.
   *
   * @param in_isr 是否在中断中 / In ISR
   * @param bRequest 请求码 / Request code
   * @param wValue 描述符类型与索引 / Descriptor type and index
   * @param wLength 请求长度 / Requested length
   * @param need_write 返回数据指针 / Output buffer
   * @return ErrorCode 错误码 / Error code
   */
  ErrorCode OnGetDescriptor(bool in_isr, uint8_t bRequest, uint16_t wValue,
                            uint16_t wLength, ConstRawData& need_write) override
  {
    UNUSED(in_isr);
    UNUSED(bRequest);

    uint8_t desc_type = (wValue >> 8) & 0xFF;
    // uint8_t desc_index = wValue & 0xFF; // 一般为0，暂不需要

    switch (desc_type)
    {
      case static_cast<uint8_t>(HIDDescriptorType::HID):  // 0x21
      {
        // 返回 HID 描述符
        ConstRawData desc = GetHIDDesc();
        need_write.addr_ = desc.addr_;
        need_write.size_ = (wLength < desc.size_) ? wLength : desc.size_;
        return ErrorCode::OK;
      }
      case static_cast<uint8_t>(HIDDescriptorType::REPORT):  // 0x22
      {
        // 返回 Report Descriptor
        ConstRawData desc = GetReportDesc();
        need_write.addr_ = desc.addr_;
        need_write.size_ = (wLength < desc.size_) ? wLength : desc.size_;
        return ErrorCode::OK;
      }
      case static_cast<uint8_t>(HIDDescriptorType::PHYSICAL):
        // 物理描述符（很少用，未实现）
      default:
        return ErrorCode::NOT_SUPPORT;
    }
  }

  /**
   * @brief 处理 HID 类请求
   * Handle HID class-specific requests
   *
   * @param in_isr 是否在中断中 / In ISR
   * @param bRequest 请求码 / Request code
   * @param wValue 请求值 / Request value
   * @param wLength 请求长度 / Request length
   * @param result 返回结果 / Result
   * @return ErrorCode 错误码 / Error code
   */
  ErrorCode OnClassRequest(bool in_isr, uint8_t bRequest, uint16_t wValue,
                           uint16_t wLength, uint16_t wIndex,
                           DeviceClass::ControlTransferResult& result) override
  {
    UNUSED(in_isr);
    UNUSED(wIndex);

    uint8_t report_id = wValue & 0xFF;

    switch (static_cast<ClassRequest>(bRequest))
    {
      case ClassRequest::GET_REPORT:
      {
        ReportType report_type = static_cast<ReportType>((wValue >> 8) & 0xFF);
        switch (report_type)
        {
          case ReportType::INPUT:
            // 派生类返回最近发送的 Input Report；没有该报告则 STALL。
            // The derived class answers with the last Input Report it sent; a report it
            // does not have stalls.
            return OnGetInputReport(report_id, result);
          case ReportType::OUTPUT:
            // 最近收到的 Output Report；没有则 STALL。
            // The last Output Report received; stalls when there is none.
            return OnGetLastOutputReport(report_id, result);
          case ReportType::FEATURE:
            // Feature Report；没有则 STALL。
            // A Feature Report; stalls when there is none.
            return OnGetFeatureReport(report_id, result);
          default:
            return OnCustomClassRequest(in_isr, bRequest, wValue, wLength, result);
        }
      }

      case ClassRequest::SET_REPORT:
        if (wLength == 0)
        {
          return ErrorCode::ARG_ERR;
        }
        // OUT 数据阶段由核心接收，完整后经 OnClassData → OnSetReportData 交给类。
        // The core receives the OUT data stage and hands it over via OnClassData →
        // OnSetReportData once it is complete.
        return OnSetReport(report_id, result);

      case ClassRequest::GET_IDLE:
        // 仅支持一个Idle rate
        if (wLength != 1 || report_id != 0)
        {
          return ErrorCode::ARG_ERR;
        }
        result.write_data = ConstRawData{&idle_rate_, 1};
        return ErrorCode::OK;

      case ClassRequest::SET_IDLE:
        if (report_id != 0)
        {
          return ErrorCode::ARG_ERR;
        }
        idle_rate_ = wValue >> 8;
        return ErrorCode::OK;

      case ClassRequest::GET_PROTOCOL:
        result.write_data = ConstRawData{reinterpret_cast<uint8_t*>(&protocol_), 1};
        return ErrorCode::OK;

      case ClassRequest::SET_PROTOCOL:
        // HID 1.11 7.2.6: wValue 0 selects the boot protocol, 1 the report protocol;
        // any other value is a request error (STALL) and the protocol stays as it is.
        // The request is required only for boot interfaces (7.2.5, 7.2.6); the other
        // classes keep answering it, so a host that sends it still gets an answer.
        if (wValue != static_cast<uint16_t>(Protocol::BOOT) &&
            wValue != static_cast<uint16_t>(Protocol::REPORT))
        {
          return ErrorCode::ARG_ERR;
        }
        protocol_ = static_cast<Protocol>(wValue);
        return ErrorCode::OK;

      default:
        return ErrorCode::NOT_SUPPORT;
    }
  }

  /**
   * @brief 处理类请求数据阶段
   *        Handle class data stage
   */
  ErrorCode OnClassData(bool in_isr, uint8_t bRequest, LibXR::ConstRawData& data) override
  {
    UNUSED(in_isr);

    switch (static_cast<ClassRequest>(bRequest))
    {
      case ClassRequest::SET_REPORT:
      {
        // 通常为 Output Report 或 Feature Report
        // 你可以区分是 Output 还是 Feature，也可以统一交给
        // OnSetReportData（推荐如下写法）
        auto ans = OnSetReportData(in_isr, data);
        if (ans == ErrorCode::OK)
        {
          // 可选：记录最近一次 report id（假定第一个字节是 report id，没有 report id
          // 就写0）
          last_output_report_id_ =
              (data.size_ > 0) ? reinterpret_cast<const uint8_t*>(data.addr_)[0] : 0;
        }
        return ans;
      }
      default:
        return OnCustomClassData(in_isr, bRequest, data);
    }
  }

  /**
   * @brief 获取最近一次 Output Report 的 Report ID
   * Get the last received Output Report ID.
   * @return uint8_t 最近收到的 Output Report ID / Last Output Report ID
   */
  uint8_t GetLastOutputReportID() const { return last_output_report_id_; }

  /**
   * @brief 获取 HID 报告描述符
   * Get HID Report Descriptor
   * @return ConstRawData 报告描述符数据及长度 / Report descriptor data and length
   */
  virtual ConstRawData GetReportDesc() = 0;

  /**
   * @brief 获取 HID 描述符
   * Get HID Descriptor
   * @return ConstRawData HID 描述符及长度 / HID descriptor and length
   */
  virtual ConstRawData GetHIDDesc()
  {
    return ConstRawData{&desc_.hid, sizeof(HIDDescriptor)};
  }

  /**
   * @brief 获取输入报告
   * Get Input Report
   *
   * 类有该报告时把数据填入 result.write_data 并返回 OK；默认实现没有任何报告，返回
   * NOT_SUPPORT，设备核心据此 STALL 控制管道（HID 1.11 7.2.1）。
   * A class that has the report fills result.write_data and returns OK. The default has
   * no report and returns NOT_SUPPORT, which the device core answers with a STALL of the
   * control pipe (HID 1.11 7.2.1).
   */
  virtual ErrorCode OnGetInputReport(uint8_t report_id,
                                     DeviceClass::ControlTransferResult& result)
  {
    UNUSED(report_id);
    UNUSED(result);
    return ErrorCode::NOT_SUPPORT;
  }

  /**
   * @brief 获取最近一次输出报告；默认没有，STALL
   * Get last Output Report; the default has none and stalls
   */
  virtual ErrorCode OnGetLastOutputReport(uint8_t report_id,
                                          DeviceClass::ControlTransferResult& result)
  {
    UNUSED(report_id);
    UNUSED(result);
    return ErrorCode::NOT_SUPPORT;
  }

  /**
   * @brief 获取特征报告；默认没有，STALL
   * Get Feature Report; the default has none and stalls
   */
  virtual ErrorCode OnGetFeatureReport(uint8_t report_id,
                                       DeviceClass::ControlTransferResult& result)
  {
    UNUSED(report_id);
    UNUSED(result);
    return ErrorCode::NOT_SUPPORT;
  }

  /**
   * @brief 处理自定义类请求
   * Handle custom class request
   *
   * @param in_isr 是否在中断中 / In ISR
   * @param bRequest 请求码 / Request code
   * @param wValue 请求值 / Request value
   * @param wLength 请求长度 / Requested length
   * @param result 结果 / Result
   * @return ErrorCode 错误码 / Error code
   */
  virtual ErrorCode OnCustomClassRequest(bool in_isr, uint8_t bRequest, uint16_t wValue,
                                         uint16_t wLength,
                                         DeviceClass::ControlTransferResult& result)
  {
    UNUSED(in_isr);
    UNUSED(bRequest);
    UNUSED(wValue);
    UNUSED(wLength);
    UNUSED(result);
    return ErrorCode::NOT_SUPPORT;
  }

  virtual ErrorCode OnCustomClassData(bool in_isr, uint8_t bRequest, ConstRawData& data)
  {
    UNUSED(in_isr);
    UNUSED(bRequest);
    UNUSED(data);
    return ErrorCode::NOT_SUPPORT;
  }

  /**
   * @brief 处理 SET_REPORT 请求
   * Handle SET_REPORT request
   */
  virtual ErrorCode OnSetReport(uint8_t report_id,
                                DeviceClass::ControlTransferResult& result)
  {
    UNUSED(report_id);
    UNUSED(result);

    return ErrorCode::NOT_SUPPORT;
  }

  /**
   * @brief 处理 SET_REPORT 数据阶段
   * Handle SET_REPORT data stage
   */
  virtual ErrorCode OnSetReportData(bool in_isr, ConstRawData& data)
  {
    UNUSED(in_isr);
    UNUSED(data);

    return ErrorCode::OK;
  }

  /**
   * @brief 发送输入报告到主机（任何上下文）
   * Send Input Report to host (any context)
   *
   * 取槽 → 拷贝 → 提交，同一时刻只有一条报告在途；上一条报告完成前返回 BUSY。
   * 取槽之后发生复位/解绑时 Submit 返回 ABORTED，本方法与未绑定时一样返回 FAILED：
   * 报告未发布，不重试。
   * Acquire a slot, copy and submit, with one report in flight at a time; BUSY while the
   * previous report is outstanding. A reset or unbind after the slot was acquired makes
   * Submit return ABORTED; this returns FAILED, as for an unbound endpoint: the report
   * was never published and is not retried.
   *
   * 在途标志从取得到 IN 完成（或失败路径）一直由当前发送方持有，所以同一时刻只有一个
   * 发送方在 PublishInputReport 与 WithdrawInputReport 里；Publish 在 Submit 之前，
   * 先于完成，后一个发送方的 Publish 一定排在它之后。Submit 失败时撤回。
   * The in-flight flag is held by the current sender from the moment it is taken until
   * the IN completion (or a failure path), so one sender at a time is inside
   * PublishInputReport and WithdrawInputReport. Publish runs before Submit, hence before
   * the completion, and the next sender's Publish is always ordered after it. A failed
   * Submit withdraws the report.
   *
   * @param report 输入报告数据指针及长度 / Input report data and length
   * @param in_isr 是否在中断中 / Whether in ISR
   * @return ErrorCode OK/BUSY/FAILED/ARG_ERR / Error code
   */
  ErrorCode SendInputReport(ConstRawData report, bool in_isr = false)
  {
    if (report.addr_ == nullptr || report.size_ == 0U || report.size_ > TX_REPORT_LEN)
      return ErrorCode::ARG_ERR;
    uint32_t expected = 0U;
    if (!input_in_flight_.compare_exchange_strong(expected, 1U,
                                                  std::memory_order_acq_rel))
      return ErrorCode::BUSY;
    // 先读会话号再读端点：看到新会话号时，端点要么还没放开，要么是新端点。
    // Read the session before the endpoint: with a new session number the endpoint is
    // either not published yet or the new one.
    const uint32_t session = session_.load(std::memory_order_acquire);
    Endpoint* ep = in_ep_ref_.load(std::memory_order_acquire);
    if (ep == nullptr)
    {
      input_in_flight_.store(0U, std::memory_order_release);
      return ErrorCode::FAILED;
    }
    Endpoint::Slot slot;
    ErrorCode ans = ep->AcquireSlot(slot, in_isr);
    if (ans != ErrorCode::OK)
    {
      input_in_flight_.store(0U, std::memory_order_release);
      // 端点未绑定（STATE_ERR）时返回 FAILED。
      // An unbound endpoint (STATE_ERR) returns FAILED.
      return ans == ErrorCode::STATE_ERR ? ErrorCode::FAILED : ans;
    }
    if (report.size_ > slot.data.size_)
    {
      ep->ReleaseSlot(slot, in_isr);
      input_in_flight_.store(0U, std::memory_order_release);
      return ErrorCode::ARG_ERR;
    }
    Memory::FastCopy(slot.data.addr_, report.addr_, report.size_);

    // 在独占窗口里、Submit 之前发布：Submit 之后 IN 完成随时会放开窗口。发布另有一把只有
    // 发送方拿放的写者锁：绑定/解绑会清在途标志，暂停在发布里的旧发送方不能因此与新
    // 发送方同时写快照；拿不到时和在途一样返回 BUSY。
    // Publish inside the exclusive window and before Submit: after Submit the IN
    // completion can open the window at any moment. Publishing also takes a writer guard
    // that only senders take and release: bind and unbind clear the in-flight flag, and a
    // sender paused inside the publish must not overlap a new sender writing the
    // snapshot. A sender that cannot take it gets BUSY, as for a report in flight.
    {
      PublishGuard guard(publishing_);
      if (!guard.Held())
      {
        ep->ReleaseSlot(slot, in_isr);
        input_in_flight_.store(0U, std::memory_order_release);
        return ErrorCode::BUSY;
      }
      PublishInputReport(report, session);
    }

    ans = ep->Submit(slot, report.size_, false, in_isr);
    if (ans != ErrorCode::OK)
    {
      // ABORTED：槽在解绑/停止时已被核心收回，再 Release 是双重释放；其余失败
      // （SIZE_ERR 等）槽仍在手里，正常归还。
      // ABORTED: the core already reclaimed the slot on unbind/stop, so releasing
      // again would be a double release; other failures (SIZE_ERR etc.) leave the
      // slot in our hands and are returned normally.
      if (ans != ErrorCode::ABORTED)
      {
        ep->ReleaseSlot(slot, in_isr);
      }
      // 报告没有发出，撤回。会话已经换了（解绑后重新绑定）时，这份报告属于旧会话，读者
      // 本来就不会采用它，不再碰快照。
      // The report was not sent: take it back. After a rebind the report belongs to the
      // old session and readers ignore it, so the snapshot is left alone.
      if (session_.load(std::memory_order_acquire) == session)
      {
        // 拿不到写者锁说明另一个发送方正在发布（重新绑定之后），那时这份报告已属于旧
        // 会话，不用撤回。
        // Without the writer guard another sender is publishing (after a rebind), and
        // this report belongs to an old session: nothing to take back.
        PublishGuard guard(publishing_);
        if (guard.Held()) WithdrawInputReport();
      }
      input_in_flight_.store(0U, std::memory_order_release);
      return ErrorCode::FAILED;
    }
    return ErrorCode::OK;
  }

  /**
   * @brief 会话开始（绑定时，owner 上下文）：类在这里清掉只属于上一次会话的状态
   * A session starts (on bind, owner context): the class clears the state that belonged
   * to the previous session here
   */
  virtual void OnSessionStart() {}

  /**
   * @brief 记下即将提交的输入报告，供 GET_REPORT 返回；在 SendInputReport 的独占窗口里、
   *        Submit 之前调用，session 是本次发送读到的会话号
   * Record the input report about to be submitted, for GET_REPORT to return; called
   * inside the exclusive window of SendInputReport before Submit, with the session number
   * this send read
   */
  virtual void PublishInputReport(ConstRawData report, uint32_t session)
  {
    UNUSED(report);
    UNUSED(session);
  }

  /**
   * @brief 撤回刚发布的输入报告：Submit 失败，主机不会看到它；与 PublishInputReport 在
   *        同一个独占窗口里调用
   * Take back the input report just published: Submit failed and the host will not see
   * it; called in the same exclusive window as PublishInputReport
   */
  virtual void WithdrawInputReport() {}

  /// 当前会话号；绑定时 owner 推进 / Current session number; the owner advances it on
  /// bind
  uint32_t Session() const { return session_.load(std::memory_order_acquire); }

  /**
   * @brief 获取IDLE报告率
   * Get IDLE report rate
   *
   * @return uint8_t IDLE报告率 / IDLE report rate
   */
  uint8_t GetIDLERate() const { return idle_rate_; }

  /**
   * @brief 获取输入端点
   * Get IN endpoint
   *
   * @return Endpoint*
   */
  Endpoint* GetInEndpoint() { return ep_in_; }

  /**
   * @brief 获取输出端点
   * Get OUT endpoint
   *
   * @return Endpoint*
   */
  Endpoint* GetOutEndpoint() { return ep_out_; }

  /**
   * @brief 查询是否支持OUT端点
   *        Check if OUT endpoint is enabled
   */
  bool HasOutEndpoint() const { return enable_out_endpoint_; }

 public:
  size_t GetControlReceiveCapacity() const override
  {
    return LibXR::max<size_t>(64U, LibXR::max<size_t>(TX_REPORT_LEN, RX_REPORT_LEN));
  }
  void OnDeviceEvent(bool in_isr, DeviceEvent event, uint8_t endpoint) override
  {
    if (event != DeviceEvent::ENDPOINT_RESUMED || ep_out_ == nullptr) return;
    if (endpoint == ep_out_->GetAddress())
      (void)ep_out_->Receive(ep_out_->MaxPacketSize(), in_isr);
  }

 protected:
  /**
   * @brief OUT 端点传输完成（owner 上下文）。
   * OUT endpoint completion (owner context).
   *
   * 默认 HID 策略：派生类经 OnOutReport 取走报告后归还租约继续接收；接收失败（非
   * ABORTED）时重新挂接收。DAP 覆盖本函数以保留租约作背压。
   * Default HID policy: the derived class consumes the report via OnOutReport, then the
   * lease is returned to keep receiving; a failed receive (not ABORTED) is armed again.
   * DAP overrides this to withhold the lease as backpressure.
   */
  virtual void OnDataOutComplete(bool in_isr, const Endpoint::Completion& completion)
  {
    if (completion.result == ErrorCode::OK && completion.slot.Valid())
    {
      OnOutReport(in_isr, completion);
      if (ep_out_ != nullptr) (void)ep_out_->ReturnSlot(completion.slot, in_isr);
    }
    else if (completion.result != ErrorCode::ABORTED && ep_out_ != nullptr)
    {
      // 接收失败会释放槽；重新挂上保持接收。
      // A failed receive frees its slot; arm it again.
      (void)ep_out_->Receive(ep_out_->MaxPacketSize(), in_isr);
    }
  }

  /**
   * @brief 收到 OUT 报告（租约由基类归还）/ An OUT report arrived; the base returns the
   * lease afterwards.
   */
  virtual void OnOutReport(bool, const Endpoint::Completion&) {}

  /**
   * @brief IN 端点传输完成（owner 上下文）：主机已取走输入报告。
   * IN endpoint completion (owner context): the host took the input report.
   */
  virtual void OnDataInComplete(bool, const Endpoint::Completion&) {}

 private:
  static void OnDataOutCompleteStatic(bool in_isr, HID* self,
                                      const Endpoint::Completion& completion)
  {
    if (!self->inited_ || self->ep_out_ == nullptr)
    {
      // 解绑期间交出的租约立即归还。
      // A lease handed over while unbinding is returned at once.
      if (completion.slot.Valid() && self->ep_out_ != nullptr)
        self->ep_out_->ReleaseSlot(completion.slot, in_isr);
      return;
    }
    self->OnDataOutComplete(in_isr, completion);
  }

  static void OnDataInCompleteStatic(bool in_isr, HID* self,
                                     const Endpoint::Completion& completion)
  {
    // 单条在途：报告完成（无论结果）即释放写者标志。
    // One report in flight: its completion (whatever the result) frees the writer flag.
    self->input_in_flight_.store(0U, std::memory_order_release);
    if (!self->inited_ || self->ep_in_ == nullptr) return;
    self->OnDataInComplete(in_isr, completion);
  }

  /// 发布窗口的写者锁：只有发送方拿放，绑定、解绑和完成回调不碰它 / Writer guard of the
  /// publish step: taken and released by senders only; bind, unbind and completions never
  /// touch it
  class PublishGuard
  {
   public:
    explicit PublishGuard(std::atomic<uint32_t>& flag) : flag_(flag)
    {
      uint32_t expected = 0U;
      held_ = flag_.compare_exchange_strong(expected, 1U, std::memory_order_acq_rel);
    }
    ~PublishGuard()
    {
      if (held_) flag_.store(0U, std::memory_order_release);
    }
    PublishGuard(const PublishGuard&) = delete;
    PublishGuard& operator=(const PublishGuard&) = delete;
    bool Held() const { return held_; }

   private:
    std::atomic<uint32_t>& flag_;
    bool held_ = false;
  };

  std::atomic<uint32_t> input_in_flight_{
      0};                                ///< 单条在途标志 / One input report in flight
  std::atomic<uint32_t> publishing_{0};  ///< 发布写者锁 / Publish writer guard
  std::atomic<Endpoint*> in_ep_ref_{
      nullptr};  ///< SendInputReport 的 IN 端点 / IN endpoint doorbell
  std::atomic<uint32_t> session_{
      0};  ///< 会话号，每次绑定加一 / Session number, advanced on every bind

  uint8_t in_ep_interval_;         ///< 输入端点间隔 / IN endpoint interval
  uint8_t out_ep_interval_;        ///< 输出端点间隔 / OUT endpoint interval
  HIDDescBlockINOUT desc_;         ///< HID 描述符块/ Descriptor block
  Endpoint::EPNumber in_ep_num_;   ///< 输入端点号 / IN endpoint number
  Endpoint::EPNumber out_ep_num_;  ///< 输出端点号 / OUT endpoint number
  Endpoint* ep_in_ = nullptr;      ///< 输入端点指针 / IN endpoint pointer
  Endpoint* ep_out_ = nullptr;     ///< 输出端点指针 / OUT endpoint pointer
  bool enable_out_endpoint_;  ///< 是否启用 OUT 端点 / Whether OUT endpoint is enabled
  bool inited_ = false;       ///< 初始化标志 / Initialization flag
  size_t itf_num_;            ///< 接口号 / Interface number
  const char* interface_string_ = nullptr;  ///< 接口字符串 / Interface string

  Protocol protocol_ = Protocol::REPORT;  ///< 当前协议类型 / Current protocol
  uint8_t idle_rate_ = 0;                 ///< 当前空闲率/ Current idle rate (unit 4ms)
  uint8_t last_output_report_id_ =
      0;  ///< 最近的 Output Report ID / Last Output Report ID

  Callback<const Endpoint::Completion&> on_data_out_complete_cb_ =
      Callback<const Endpoint::Completion&>::Create(OnDataOutCompleteStatic, this);

  Callback<const Endpoint::Completion&> on_data_in_complete_cb_ =
      Callback<const Endpoint::Completion&>::Create(OnDataInCompleteStatic, this);
};

}  // namespace LibXR::USB
