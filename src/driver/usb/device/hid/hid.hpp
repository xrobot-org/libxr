#pragma once
#include <cstring>

#include "dev_core.hpp"
#include "libxr_mem.hpp"
#include "usb/core/desc_cfg.hpp"

namespace LibXR::USB
{

/**
 * @brief
 * USB HID（Human Interface Device）基类，支持可选 OUT
 * 端点、自动生成描述符，适合键盘、鼠标、手柄等扩展。
 * USB HID (Human Interface Device) base class with optional OUT endpoint and auto
 * descriptor generation, suitable for extension as keyboard, mouse, gamepad, etc.
 *
 * @tparam REPORT_DESC_LEN 报告描述符长度 / Report descriptor length (bytes)
 * @tparam TX_REPORT_LEN Input Report 长度 / Input report length (bytes)
 * @tparam RX_REPORT_LEN Output Report 长度 / Output report length (bytes) [default: 0]
 */
template <size_t REPORT_DESC_LEN, size_t TX_REPORT_LEN, size_t RX_REPORT_LEN = 0>
class HID : public DeviceClass
{
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
  void BindEndpoints(EndpointPool& endpoint_pool, uint8_t start_itf_num, bool) override
  {
    inited_ = false;
    itf_num_ = start_itf_num;
    ep_in_ = nullptr;
    ep_out_ = nullptr;

    // 获取IN端点
    auto ans = endpoint_pool.Get(ep_in_, Endpoint::Direction::IN, in_ep_num_);
    ASSERT(ans == ErrorCode::OK);
    ep_in_->Configure(
        {Endpoint::Direction::IN, Endpoint::Type::INTERRUPT, TX_REPORT_LEN});

    if (enable_out_endpoint_)
    {
      ans = endpoint_pool.Get(ep_out_, Endpoint::Direction::OUT, out_ep_num_);
      ASSERT(ans == ErrorCode::OK);
      ep_out_->Configure(
          {Endpoint::Direction::OUT, Endpoint::Type::INTERRUPT, RX_REPORT_LEN});
    }

    // 填充接口描述符
    desc_.intf = {
        9,                                                   // bLength
        static_cast<uint8_t>(DescriptorType::INTERFACE),     // bDescriptorType
        static_cast<uint8_t>(itf_num_),                      // bInterfaceNumber
        0,                                                   // bAlternateSetting
        static_cast<uint8_t>(enable_out_endpoint_ ? 2 : 1),  // bNumEndpoints
        0x03,                                                // bInterfaceClass (HID)
        0x00,                                                // bInterfaceSubClass
        0x00,                        // bInterfaceProtocol (可选键盘/鼠标设置1/2)
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
        TX_REPORT_LEN,
        in_ep_interval_  // 轮询间隔ms
    };

    // 填充OUT端点描述符（如启用）
    if (enable_out_endpoint_)
    {
      desc_.ep_out = {7,
                      static_cast<uint8_t>(DescriptorType::ENDPOINT),
                      ep_out_->GetAddress(),
                      static_cast<uint8_t>(Endpoint::Type::INTERRUPT),
                      RX_REPORT_LEN,
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

    ep_in_->SetOnTransferCompleteCallback(on_data_in_complete_cb_);

    if (enable_out_endpoint_)
    {
      ep_out_->SetOnTransferCompleteCallback(on_data_out_complete_cb_);
      ep_out_->Transfer(RX_REPORT_LEN);
    }

    inited_ = true;
  }

  static void OnDataOutCompleteStatic(bool in_isr, HID* self, LibXR::ConstRawData& data)
  {
    if (self == nullptr || !self->inited_ || self->ep_out_ == nullptr)
    {
      return;
    }
    self->OnDataOutComplete(in_isr, data);
    self->ep_out_->Transfer(RX_REPORT_LEN);
  }

  static void OnDataInCompleteStatic(bool in_isr, HID* self, LibXR::ConstRawData& data)
  {
    if (self == nullptr || !self->inited_ || self->ep_in_ == nullptr)
    {
      return;
    }
    self->OnDataInComplete(in_isr, data);
  }

  virtual void OnDataOutComplete(bool in_isr, LibXR::ConstRawData& data)
  {
    UNUSED(in_isr);
    UNUSED(data);
  }

  virtual void OnDataInComplete(bool in_isr, LibXR::ConstRawData& data)
  {
    UNUSED(in_isr);
    UNUSED(data);
  }

  /**
   * @brief 反初始化 HID 设备
   * Deinitialize HID device.
   * @param endpoint_pool 端点池 / Endpoint pool
   * @param in_isr 是否在中断中 / Whether in ISR
   */
  void UnbindEndpoints(EndpointPool& endpoint_pool, bool) override
  {
    inited_ = false;
    if (ep_in_)
    {
      ep_in_->Close();
      endpoint_pool.Release(ep_in_);
      ep_in_ = nullptr;
    }
    if (ep_out_)
    {
      ep_out_->Close();
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
            // 查找或生成 Input Report（按你的设备实际逻辑）
            return OnGetInputReport(report_id, result);
          case ReportType::OUTPUT:
            // 查找或返回最近收到的 Output Report（通常很少实现GET Output）
            return OnGetLastOutputReport(report_id, result);
          case ReportType::FEATURE:
            // 查找或生成 Feature Report
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
        // 由OnClassData阶段接收
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
        result.write_zlp = true;
        return ErrorCode::OK;

      case ClassRequest::GET_PROTOCOL:
        result.write_data = ConstRawData{reinterpret_cast<uint8_t*>(&protocol_), 1};
        return ErrorCode::OK;

      case ClassRequest::SET_PROTOCOL:
        protocol_ = static_cast<Protocol>(wValue & 0xFF);
        result.write_zlp = true;
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
   */
  virtual ErrorCode OnGetInputReport(uint8_t report_id,
                                     DeviceClass::ControlTransferResult& result)
  {
    UNUSED(report_id);
    result.write_data = ConstRawData{nullptr, 0};
    return ErrorCode::OK;
  }

  /**
   * @brief 获取最近一次输出报告
   * Get last Output Report
   */
  virtual ErrorCode OnGetLastOutputReport(uint8_t report_id,
                                          DeviceClass::ControlTransferResult& result)
  {
    UNUSED(report_id);
    result.write_data = ConstRawData{nullptr, 0};
    return ErrorCode::OK;
  }

  /**
   * @brief 获取特征报告
   * Get Feature Report
   */
  virtual ErrorCode OnGetFeatureReport(uint8_t report_id,
                                       DeviceClass::ControlTransferResult& result)
  {
    UNUSED(report_id);
    result.write_data = ConstRawData{nullptr, 0};
    return ErrorCode::OK;
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
   * @brief 发送输入报告到主机
   * Send Input Report to host
   *
   * @param report 输入报告数据指针及长度 / Input report data and length
   * @return ErrorCode 错误码 / Error code
   */
  ErrorCode SendInputReport(ConstRawData report)
  {
    if (!inited_ || !ep_in_)
    {
      return ErrorCode::FAILED;
    }
    if (!report.addr_ || report.size_ == 0 || report.size_ > TX_REPORT_LEN)
    {
      return ErrorCode::ARG_ERR;
    }

    if (ep_in_->GetState() != Endpoint::State::IDLE)
    {
      return ErrorCode::BUSY;
    }

    // 数据拷贝到端点缓冲区
    auto buf = ep_in_->GetBuffer();
    if (report.size_ > buf.size_)
    {
      return ErrorCode::NO_BUFF;
    }

    LibXR::Memory::FastCopy(buf.addr_, report.addr_, report.size_);

    // 启动端点传输
    return ep_in_->Transfer(report.size_);
  }

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

 private:
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

  LibXR::Callback<LibXR::ConstRawData&> on_data_out_complete_cb_ =
      LibXR::Callback<LibXR::ConstRawData&>::Create(OnDataOutCompleteStatic, this);

  LibXR::Callback<LibXR::ConstRawData&> on_data_in_complete_cb_ =
      LibXR::Callback<LibXR::ConstRawData&>::Create(OnDataInCompleteStatic, this);
};

}  // namespace LibXR::USB
