#pragma once
#include <cstring>

#include "dev_core.hpp"
#include "libxr_def.hpp"
#include "uart.hpp"
#include "usb/core/desc_cfg.hpp"

namespace LibXR::USB
{

/**
 * @brief USB CDC ACM (Abstract Control Model) 设备类实现
 *        USB CDC ACM (Abstract Control Model) device class implementation
 *
 * 实现USB CDC ACM规范定义的虚拟串口功能，提供完整的USB设备描述符配置、
 * 类请求处理和数据传输机制。继承自DeviceClass。
 *
 * Implements virtual serial port functionality as defined by USB CDC ACM specification,
 * providing complete USB descriptor configuration, class request handling, and data
 * transfer mechanisms. Inherits from DeviceClass.
 */
class CDCBase : public DeviceClass
{
 public:
  static constexpr const char* DEFAULT_CONTROL_INTERFACE_STRING = "XRUSB CDC Control";
  static constexpr const char* DEFAULT_DATA_INTERFACE_STRING = "XRUSB CDC Data";

 private:
  /// CDC功能描述符子类型定义 / CDC functional descriptor subtypes
  enum class DescriptorSubtype : uint8_t
  {
    HEADER = 0x00,  ///< 头功能描述符 / Header functional descriptor
    CALL_MANAGEMENT =
        0x01,      ///< 呼叫管理功能描述符 / Call management functional descriptor
    ACM = 0x02,    ///< 抽象控制模型描述符 / Abstract control model descriptor
    UNION = 0x06,  ///< 联合功能描述符 / Union functional descriptor
  };

  /// USB设备类代码 / USB device class codes
  enum class Class : uint8_t
  {
    COMM = 0x02,  ///< 通信设备类 / Communications device class
    DATA = 0x0A   ///< 数据接口类 / Data interface class
  };

  /// CDC协议代码 / CDC protocol codes
  enum class Protocol : uint8_t
  {
    NONE = 0x00,        ///< 无协议 / No protocol
    AT_COMMAND = 0x01,  ///< AT命令协议 / AT Command protocol
  };

  /// CDC子类代码 / CDC subclass codes
  enum class Subclass : uint8_t
  {
    NONE = 0x00,  ///< 无子类 / No subclass
    DIRECT_LINE_CONTROL_MODEL =
        0x01,  ///< 直接控制模型（CDC-DCM） / Direct Control Model (CDC-DCM)
    ABSTRACT_CONTROL_MODEL =
        0x02,  ///< 抽象控制模型（CDC-ACM） / Abstract Control Model (CDC-ACM)
  };

  /// CDC类特定请求 / CDC class-specific requests
  enum class ClassRequest : uint8_t
  {
    SET_LINE_CODING = 0x20,         ///< 设置串行线路编码 / Set line coding parameters
    GET_LINE_CODING = 0x21,         ///< 获取当前线路编码 / Get current line coding
    SET_CONTROL_LINE_STATE = 0x22,  ///< 设置控制线路状态 / Set control line state
    SEND_BREAK = 0x23               ///< 发送BREAK信号 / Send BREAK signal
  };

  /// CDC通知类型 / CDC notification types
  enum class CDCNotification : uint8_t
  {
    NETWORK_CONNECTION = 0x00,   ///< 网络连接通知 / Network connection
    RESPONSE_AVAILABLE = 0x01,   ///< 响应可用通知 / Response available
    AUX_JACK_HOOK_STATE = 0x08,  ///< 辅助插槽状态通知 / Aux jack hook state
    RING_DETECT = 0x09,          ///< 响铃检测通知 / Ring detect
    SERIAL_STATE = 0x20,  ///< 串行状态通知（CDC-ACM 必需） / Serial state notification
                          ///< (required for CDC-ACM)
  };

  LIBXR_PACKED_BEGIN
  /**
   * @brief CDC线路编码参数结构体
   *        CDC line coding parameters structure
   *
   * 定义串行通信参数：波特率、停止位、校验位和数据位
   * Defines serial communication parameters: baud rate, stop bits, parity and data bits
   */
  struct CDCLineCoding
  {
    uint32_t dwDTERate;   ///< 波特率（小端格式） / Baud rate (little-endian)
    uint8_t bCharFormat;  ///< 停止位：0=1位，1=1.5位，2=2位 / Stop bits: 0=1, 1=1.5, 2=2
    uint8_t bParityType;  ///< 校验：0=None,1=Odd,2=Even,3=Mark,4=Space / Parity:
                          ///< 0=None,1=Odd,2=Even,3=Mark,4=Space
    uint8_t bDataBits;    ///< 数据位：5,6,7,8或16 / Data bits: 5,6,7,8 or 16
  };

  /**
   * @brief 串行状态通知结构体
   *        Serial state notification structure
   *
   * 用于通过中断端点向主机报告串行端口状态变化
   * Used to report serial port state changes to host via interrupt endpoint
   */
  struct SerialStateNotification
  {
    uint8_t bmRequestType;  ///< 请求类型（固定为 0xA1） / Request type (fixed to 0xA1)
    uint8_t bNotification;  ///< 通知类型（固定为 SERIAL_STATE） / Notification type
                            ///< (fixed to SERIAL_STATE)
    uint16_t wValue;        ///< 值（固定为 0） / Value (fixed to 0)
    uint16_t wIndex;        ///< 接口号 / Interface number
    uint16_t wLength;       ///< 数据长度（固定为2） / Data length (fixed to 2)
    uint16_t serialState;   ///< 串行状态位图 / Serial state bitmap
  };
  LIBXR_PACKED_END

  // 确保 CDCLineCoding 结构体大小为 7 字节。
  // Ensure CDCLineCoding is exactly 7 bytes.
  static_assert(sizeof(CDCLineCoding) == 7, "LineCoding结构必须7字节");

  // 控制线路状态位定义
  static constexpr uint16_t CDC_CONTROL_LINE_DTR = 0x01;  ///< DTR控制位 / DTR control bit
  static constexpr uint16_t CDC_CONTROL_LINE_RTS = 0x02;  ///< RTS控制位 / RTS control bit

 public:
  /**
   * @brief CDC构造函数
   *        CDC constructor
   *
   * @param data_in_ep_num 数据输入端点号 / Data IN endpoint number
   * @param data_out_ep_num 数据输出端点号 / Data OUT endpoint number
   * @param comm_ep_num 通信端点号 / Communication endpoint number
   * @param control_interface_string 控制接口字符串 / Control interface string
   * @param data_interface_string 数据接口字符串 / Data interface string
   */
  CDCBase(Endpoint::EPNumber data_in_ep_num, Endpoint::EPNumber data_out_ep_num,
          Endpoint::EPNumber comm_ep_num,
          const char* control_interface_string = DEFAULT_CONTROL_INTERFACE_STRING,
          const char* data_interface_string = DEFAULT_DATA_INTERFACE_STRING)
      : data_in_ep_num_(data_in_ep_num),
        data_out_ep_num_(data_out_ep_num),
        comm_ep_num_(comm_ep_num),
        control_interface_string_(control_interface_string),
        data_interface_string_(data_interface_string)
  {
  }

  const char* GetInterfaceString(size_t local_interface_index) const override
  {
    switch (local_interface_index)
    {
      case 0:
        return control_interface_string_;
      case 1:
        return data_interface_string_;
      default:
        return nullptr;
    }
  }

  /**
   * @brief 检查DTR状态
   *        Check DTR state
   *
   * @return true DTR已设置
   * @return false DTR未设置
   */
  bool IsDtrSet() const { return (control_line_state_ & CDC_CONTROL_LINE_DTR) != 0; }

  /**
   * @brief 检查RTS状态
   *        Check RTS state
   *
   * @return true RTS已设置
   * @return false RTS未设置
   */
  bool IsRtsSet() const { return (control_line_state_ & CDC_CONTROL_LINE_RTS) != 0; }

  /**
   * @brief 发送串行状态通知
   *        Send serial state notification
   *
   * 通过中断端点向主机报告当前串行端口状态
   * Reports current serial port state to host via interrupt endpoint
   */
  ErrorCode SendSerialState()
  {
    if (ep_comm_in_->GetState() == Endpoint::State::BUSY)
    {
      return ErrorCode::BUSY;
    }
    auto buffer = ep_comm_in_->GetBuffer();
    ASSERT(buffer.size_ >= sizeof(SerialStateNotification));
    SerialStateNotification* notification =
        reinterpret_cast<SerialStateNotification*>(buffer.addr_);
    notification->wIndex = itf_comm_in_num_;

    // 设置串行状态位。
    // Fill the serial-state bitmap.
    if (IsDtrSet())
    {
      // DTR 有效时报告载波检测（DCD）和数据集就绪（DSR）。
      // When DTR is asserted, report DCD and DSR as active.
      notification->serialState = 0x03;  // DCD / DSR
    }
    else
    {
      notification->serialState = 0x00;  // 无状态
    }

    // 填充固定字段。
    // Fill the fixed notification header fields.
    notification->bmRequestType = 0xA1;  // 设备到主机，类，接口
    notification->bNotification = static_cast<uint8_t>(CDCNotification::SERIAL_STATE);
    notification->wValue = 0;
    notification->wLength = 2;

    ep_comm_in_->Transfer(sizeof(SerialStateNotification));

    return ErrorCode::OK;
  }

  /**
   * @brief 设置控制线路状态变更回调
   *        Set control line state change callback
   */
  void SetOnSetControlLineStateCallback(LibXR::Callback<bool, bool> cb)
  {
    has_control_line_state_cb_ = !cb.Empty();
    on_set_control_line_state_cb_ = cb;
  }

  /**
   * @brief 设置线路编码变更回调
   *        Set line coding change callback
   */
  void SetOnSetLineCodingCallback(LibXR::Callback<LibXR::UART::Configuration> cb)
  {
    has_line_coding_cb_ = !cb.Empty();
    on_set_line_coding_cb_ = cb;
  }

 protected:
  /**
   * @brief 初始化CDC设备
   *        Initialize CDC device
   *
   * 配置端点、填充描述符并初始化数据传输机制
   * Configures endpoints, populates descriptors and initializes data transfer mechanisms
   *
   * @param endpoint_pool 端点资源池 / Endpoint resource pool
   * @param start_itf_num 起始接口号 / Starting interface number
   * @param in_isr        是否在中断上下文 / Whether in ISR context
   */
  virtual void BindEndpoints(EndpointPool& endpoint_pool, uint8_t start_itf_num,
                             bool) override
  {
    control_line_state_ = 0;
    // 获取并配置数据IN端点
    auto ans = endpoint_pool.Get(ep_data_in_, Endpoint::Direction::IN, data_in_ep_num_);
    ASSERT(ans == ErrorCode::OK);

    // 获取并配置数据OUT端点
    ans = endpoint_pool.Get(ep_data_out_, Endpoint::Direction::OUT, data_out_ep_num_);
    ASSERT(ans == ErrorCode::OK);

    // 获取并配置通信端点。
    // Acquire and configure the communication endpoint.
    ans = endpoint_pool.Get(ep_comm_in_, Endpoint::Direction::IN, comm_ep_num_);
    ASSERT(ans == ErrorCode::OK);

    // 配置端点参数。
    // Configure endpoint parameters.
    ep_data_in_->Configure(
        {Endpoint::Direction::IN, Endpoint::Type::BULK, UINT16_MAX, true});
    ep_data_out_->Configure(
        {Endpoint::Direction::OUT, Endpoint::Type::BULK, UINT16_MAX, true});
    ep_comm_in_->Configure({Endpoint::Direction::IN, Endpoint::Type::INTERRUPT, 16});

    // === 填充CDC描述符块 ===
    static constexpr uint8_t COMM_INTERFACE = 0;  // 通信接口号
    static constexpr uint8_t DATA_INTERFACE = 1;  // 数据接口号

    // IAD 描述符（关联两组 CDC 接口）。
    // IAD descriptor that groups the two CDC interfaces.
    desc_block_.iad = {8,
                       static_cast<uint8_t>(DescriptorType::IAD),
                       static_cast<uint8_t>(COMM_INTERFACE + start_itf_num),
                       2,
                       static_cast<uint8_t>(Class::COMM),
                       static_cast<uint8_t>(Subclass::ABSTRACT_CONTROL_MODEL),
                       static_cast<uint8_t>(Protocol::NONE),
                       0};

    // 通信接口描述符。
    // Communication interface descriptor.
    desc_block_.comm_intf = {9,
                             static_cast<uint8_t>(DescriptorType::INTERFACE),
                             static_cast<uint8_t>(COMM_INTERFACE + start_itf_num),
                             0,
                             1,
                             static_cast<uint8_t>(Class::COMM),
                             static_cast<uint8_t>(Subclass::ABSTRACT_CONTROL_MODEL),
                             static_cast<uint8_t>(Protocol::NONE),
                             GetInterfaceStringIndex(COMM_INTERFACE)};

    // CDC 头功能描述符。
    // CDC header functional descriptor.
    desc_block_.cdc_header = {5, DescriptorType::CS_INTERFACE, DescriptorSubtype::HEADER,
                              0x0110};  // CDC规范版本1.10

    // 呼叫管理功能描述符。
    // Call-management functional descriptor.
    desc_block_.cdc_callmgmt = {
        5, DescriptorType::CS_INTERFACE, DescriptorSubtype::CALL_MANAGEMENT,
        0x00,                                                   // 无呼叫管理能力
        static_cast<uint8_t>(DATA_INTERFACE + start_itf_num)};  // 数据接口号

    // ACM 功能描述符。
    // ACM functional descriptor.
    desc_block_.cdc_acm = {4, DescriptorType::CS_INTERFACE, DescriptorSubtype::ACM,
                           0x02};  // 支持SetLineCoding/GetLineCoding/SetControlLineState

    // 联合功能描述符。
    // Union functional descriptor.
    desc_block_.cdc_union = {5, DescriptorType::CS_INTERFACE, DescriptorSubtype::UNION,
                             static_cast<uint8_t>(COMM_INTERFACE + start_itf_num),
                             static_cast<uint8_t>(DATA_INTERFACE + start_itf_num)};

    // 数据接口描述符。
    // Data interface descriptor.
    desc_block_.data_intf = {9,
                             static_cast<uint8_t>(DescriptorType::INTERFACE),
                             static_cast<uint8_t>(DATA_INTERFACE + start_itf_num),
                             0,
                             2,
                             static_cast<uint8_t>(Class::DATA),
                             0x00,  // 无子类
                             0x00,  // 无协议
                             GetInterfaceStringIndex(DATA_INTERFACE)};

    // 数据OUT端点描述符
    desc_block_.data_ep_out = {7,
                               static_cast<uint8_t>(DescriptorType::ENDPOINT),
                               ep_data_out_->GetAddress(),
                               static_cast<uint8_t>(Endpoint::Type::BULK),
                               ep_data_out_->MaxPacketSize(),
                               0};  // 轮询间隔0（BULK忽略）

    // 数据IN端点描述符
    desc_block_.data_ep_in = {7,
                              static_cast<uint8_t>(DescriptorType::ENDPOINT),
                              static_cast<uint8_t>(ep_data_in_->GetAddress()),
                              static_cast<uint8_t>(Endpoint::Type::BULK),
                              ep_data_in_->MaxPacketSize(),
                              0};  // 轮询间隔0（BULK忽略）

    // 通信端点描述符。
    // Communication endpoint descriptor.
    desc_block_.comm_ep = {
        7,
        static_cast<uint8_t>(DescriptorType::ENDPOINT),
        static_cast<uint8_t>(ep_comm_in_->GetAddress()),  // IN端点地址
        static_cast<uint8_t>(Endpoint::Type::INTERRUPT),
        16,   // 16字节最大包大小
        0x04  // 轮询间隔FS-4ms HS-1ms
    };

    itf_comm_in_num_ = start_itf_num;

    // 设置描述符原始数据。
    // Publish the raw descriptor block.
    SetData(RawData{reinterpret_cast<uint8_t*>(&desc_block_), sizeof(desc_block_)});

    // 设置端点传输完成回调。
    // Install endpoint transfer-complete callbacks.
    ep_data_out_->SetOnTransferCompleteCallback(on_data_out_complete_cb_);
    ep_data_in_->SetOnTransferCompleteCallback(on_data_in_complete_cb_);

    inited_ = true;

    // 启动OUT端点传输
    ep_data_out_->Transfer(ep_data_out_->MaxPacketSize());
  }

  /**
   * @brief 反初始化CDC设备
   *        Deinitialize CDC device
   *
   * 释放所有占用的资源
   * Releases all allocated resources
   */
  virtual void UnbindEndpoints(EndpointPool& endpoint_pool, bool) override
  {
    inited_ = false;
    control_line_state_ = 0;
    ep_data_in_->Close();
    ep_data_out_->Close();
    ep_comm_in_->Close();
    ep_data_in_->SetActiveLength(0);
    ep_data_out_->SetActiveLength(0);
    ep_comm_in_->SetActiveLength(0);
    endpoint_pool.Release(ep_data_in_);
    endpoint_pool.Release(ep_data_out_);
    endpoint_pool.Release(ep_comm_in_);
    ep_data_in_ = nullptr;
    ep_data_out_ = nullptr;
    ep_comm_in_ = nullptr;
  }

  /**
   * @brief 数据OUT端点传输完成静态回调
   *        Static callback for data OUT endpoint transfer completion
   */
  static void OnDataOutCompleteStatic(bool in_isr, CDCBase* self, ConstRawData& data)
  {
    if (!self->inited_)
    {
      return;
    }
    self->OnDataOutComplete(in_isr, data);
  }

  /**
   * @brief 数据IN端点传输完成静态回调
   *        Static callback for data IN endpoint transfer completion
   */
  static void OnDataInCompleteStatic(bool in_isr, CDCBase* self, ConstRawData& data)
  {
    if (!self->inited_)
    {
      return;
    }
    self->OnDataInComplete(in_isr, data);
  }

  /**
   * @brief 数据OUT端点传输完成处理
   *        Handle data OUT endpoint transfer completion
   *
   * 接收数据并放入接收缓冲区，然后重新启动传输
   * Receives data into receive buffer and restarts transfer
   */
  virtual void OnDataOutComplete(bool in_isr, ConstRawData& data) = 0;

  /**
   * @brief 数据IN端点传输完成处理
   *        Handle data IN endpoint transfer completion
   *
   * 完成发送操作并处理发送队列中的下一个数据包
   * Completes transmission and processes next packet in send queue
   */
  virtual void OnDataInComplete(bool in_isr, ConstRawData& data) = 0;

  /**
   * @brief 获取接口数量
   *        Get number of interfaces
   *
   * @return size_t 接口数量（固定为2：通信接口+数据接口）
   */
  size_t GetInterfaceCount() override { return 2; }

  /**
   * @brief 检查是否包含IAD
   *        Check if IAD is present
   *
   * @return true
   * @return false
   */
  bool HasIAD() override { return true; }

  bool OwnsEndpoint(uint8_t ep_addr) const override
  {
    if (!inited_)
    {
      return false;
    }

    return ep_data_in_->GetAddress() == ep_addr ||
           ep_data_out_->GetAddress() == ep_addr || ep_comm_in_->GetAddress() == ep_addr;
  }

  /**
   * @brief 获取最大配置描述符大小
   *        Get maximum configuration descriptor size
   *
   * @return size_t 描述符块大小
   */
  size_t GetMaxConfigSize() override { return sizeof(desc_block_); }

  /**
   * @brief 处理类特定请求
   *        Handle class-specific requests
   *
   * 实现CDC ACM规范定义的标准请求
   * Implements standard requests defined by CDC ACM specification
   */
  ErrorCode OnClassRequest(bool in_isr, uint8_t bRequest, uint16_t wValue,
                           uint16_t wLength, uint16_t wIndex,
                           DeviceClass::ControlTransferResult& result) override
  {
    UNUSED(in_isr);
    UNUSED(wIndex);

    switch (static_cast<ClassRequest>(bRequest))
    {
      case ClassRequest::SET_LINE_CODING:
        // 主机将在数据阶段发送7字节参数
        if (wLength != sizeof(line_coding_))
        {
          return ErrorCode::ARG_ERR;
        }
        result.read_data =
            RawData{reinterpret_cast<uint8_t*>(&line_coding_), sizeof(line_coding_)};
        return ErrorCode::OK;

      case ClassRequest::GET_LINE_CODING:
        // 返回当前线路编码设置。
        // Return the current line-coding settings.
        if (wLength != sizeof(line_coding_))
        {
          return ErrorCode::ARG_ERR;
        }

        result.write_data = ConstRawData{reinterpret_cast<const uint8_t*>(&line_coding_),
                                         sizeof(line_coding_)};
        return ErrorCode::OK;

      case ClassRequest::SET_CONTROL_LINE_STATE:
        // 设置 DTR / RTS 状态。
        // Update the DTR / RTS control-line state.
        control_line_state_ = wValue;
        result.write_zlp = true;
        SendSerialState();
        if (has_control_line_state_cb_)
        {
          on_set_control_line_state_cb_.Run(in_isr, IsDtrSet(), IsRtsSet());
        }
        return ErrorCode::OK;

      case ClassRequest::SEND_BREAK:
        // BREAK 信号通常忽略。
        // BREAK is typically ignored on this path.
        result.write_zlp = true;
        return ErrorCode::OK;

      default:
        return ErrorCode::NOT_SUPPORT;
    }
  }

  /**
   * @brief 处理类请求数据阶段
   *        Handle class request data stage
   */
  ErrorCode OnClassData(bool in_isr, uint8_t bRequest, LibXR::ConstRawData& data) override
  {
    UNUSED(in_isr);
    UNUSED(data);

    switch (static_cast<ClassRequest>(bRequest))
    {
      case ClassRequest::SET_LINE_CODING:
      {
        // 将 CDC 线路编码转换为 UART 配置。
        // Convert CDC line coding into a UART configuration.
        LibXR::UART::Configuration cfg;
        cfg.baudrate = line_coding_.dwDTERate;
        switch (line_coding_.bCharFormat)
        {
          case 0:
            cfg.stop_bits = 1;
            break;
          // TODO: 1.5
          case 2:
            cfg.stop_bits = 2;
            break;
          default:
            cfg.stop_bits = 1;
        }
        switch (line_coding_.bParityType)
        {
          case 1:
            cfg.parity = LibXR::UART::Parity::ODD;
            break;
          case 2:
            cfg.parity = LibXR::UART::Parity::EVEN;
            break;
          default:
            cfg.parity = LibXR::UART::Parity::NO_PARITY;
        }
        cfg.data_bits = line_coding_.bDataBits;
        if (has_line_coding_cb_)
        {
          on_set_line_coding_cb_.Run(in_isr, cfg);
        }
      }
        return ErrorCode::OK;
      default:
        return ErrorCode::NOT_SUPPORT;
    }
  }

  LIBXR_PACKED_BEGIN
  /**
   * @brief CDC描述符块结构
   *        CDC descriptor block structure
   *
   * 包含CDC ACM设备所需的所有描述符
   * Contains all descriptors required for a CDC ACM device
   */
  struct CDCDescBlock
  {
    IADDescriptor iad;  ///< 接口关联描述符 / Interface association descriptor
    InterfaceDescriptor
        comm_intf;  ///< 通信接口描述符 / Communication interface descriptor

    // CDC类特定描述符
    struct
    {
      uint8_t bFunctionLength = 5;
      DescriptorType bDescriptorType = DescriptorType::CS_INTERFACE;
      DescriptorSubtype bDescriptorSubtype = DescriptorSubtype::HEADER;
      uint16_t bcdCDC = 0x0110;  ///< CDC规范版本 / CDC specification version
    } cdc_header;

    struct
    {
      uint8_t bFunctionLength = 5;
      DescriptorType bDescriptorType = DescriptorType::CS_INTERFACE;
      DescriptorSubtype bDescriptorSubtype = DescriptorSubtype::CALL_MANAGEMENT;
      uint8_t bmCapabilities = 0x00;  ///< 呼叫管理能力 / Call management capabilities
      uint8_t bDataInterface;         ///< 数据接口号 / Data interface number
    } cdc_callmgmt;

    struct
    {
      uint8_t bFunctionLength = 4;
      DescriptorType bDescriptorType = DescriptorType::CS_INTERFACE;
      DescriptorSubtype bDescriptorSubtype = DescriptorSubtype::ACM;
      uint8_t bmCapabilities = 0x02;  ///< ACM能力 / ACM capabilities
    } cdc_acm;

    struct
    {
      uint8_t bFunctionLength = 5;
      DescriptorType bDescriptorType = DescriptorType::CS_INTERFACE;
      DescriptorSubtype bDescriptorSubtype = DescriptorSubtype::UNION;
      uint8_t bMasterInterface;  ///< 主接口号 / Master interface number
      uint8_t bSlaveInterface0;  ///< 从接口号 / Slave interface number
    } cdc_union;

    EndpointDescriptor comm_ep;  ///< 通信端点描述符 / Communication endpoint descriptor

    InterfaceDescriptor data_intf;  ///< 数据接口描述符 / Data interface descriptor

    EndpointDescriptor data_ep_out;  ///< 数据OUT端点描述符 / Data OUT endpoint descriptor
    EndpointDescriptor data_ep_in;   ///< 数据IN端点描述符 / Data IN endpoint descriptor
  } desc_block_;
  LIBXR_PACKED_END

 protected:
  CDCLineCoding& GetLineCoding() { return line_coding_; }

  bool Inited() { return inited_; }

  Endpoint* GetDataInEndpoint() { return ep_data_in_; }

  Endpoint* GetDataOutEndpoint() { return ep_data_out_; }

  Endpoint* GetCommInEndpoint() { return ep_comm_in_; }

 private:
  // 端点号。
  // Endpoint numbers.
  Endpoint::EPNumber data_in_ep_num_;   ///< 数据IN端点号 / Data IN endpoint number
  Endpoint::EPNumber data_out_ep_num_;  ///< 数据OUT端点号 / Data OUT endpoint number
  Endpoint::EPNumber comm_ep_num_;      ///< 通信端点号 / Communication endpoint number
  const char* control_interface_string_ =
      nullptr;  ///< 控制接口字符串 / Control interface string
  const char* data_interface_string_ =
      nullptr;  ///< 数据接口字符串 / Data interface string

  // 端点指针。
  // Endpoint pointers.
  Endpoint* ep_data_in_ = nullptr;   ///< 数据IN端点 / Data IN endpoint
  Endpoint* ep_data_out_ = nullptr;  ///< 数据OUT端点 / Data OUT endpoint
  Endpoint* ep_comm_in_ = nullptr;   ///< 通信IN端点 / Communication IN endpoint

  // 端点回调。
  // Endpoint callbacks.
  LibXR::Callback<LibXR::ConstRawData&> on_data_out_complete_cb_ =
      LibXR::Callback<LibXR::ConstRawData&>::Create(OnDataOutCompleteStatic, this);

  LibXR::Callback<LibXR::ConstRawData&> on_data_in_complete_cb_ =
      LibXR::Callback<LibXR::ConstRawData&>::Create(OnDataInCompleteStatic, this);

  // 用户回调。
  // User callbacks.
  LibXR::Callback<bool, bool>
      on_set_control_line_state_cb_;  ///< 控制线路状态变更回调 / Control line state
                                      ///< change callback
  LibXR::Callback<LibXR::UART::Configuration>
      on_set_line_coding_cb_;  ///< 线路编码变更回调 / Line coding change callback

  // 状态标志。
  // State flags.
  bool inited_ = false;                     ///< 初始化标志 / Initialization flag
  bool has_control_line_state_cb_ = false;  ///< Control-line callback registered
  bool has_line_coding_cb_ = false;         ///< Line-coding callback registered

  // 接口信息。
  // Interface metadata.
  uint8_t itf_comm_in_num_;  ///< 通信接口号 / Communication interface number

  // CDC 参数。
  // CDC parameters.
  CDCLineCoding line_coding_ = {115200, 0, 0, 8};  ///< 当前线路编码 / Current line coding
  uint16_t control_line_state_ = 0;                ///< 控制线路状态 / Control line state
};

}  // namespace LibXR::USB
