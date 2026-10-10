#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <initializer_list>
#include <type_traits>

#include "can.hpp"
#include "dev_core.hpp"
#include "gpio.hpp"
#include "gs_usb_protocol.hpp"
#include "libxr_def.hpp"
#include "libxr_mem.hpp"
#include "timebase.hpp"
#include "usb/core/desc_cfg.hpp"

namespace LibXR::USB
{

/**
 * @class GsUsbClass
 * @brief GsUsb 设备类，实现 Linux gs_usb 协议（经典 CAN + CAN FD） /
 *        GsUsb device class implementing Linux gs_usb protocol (Classic CAN + CAN FD)
 * @tparam CanChNum 编译期固定的 CAN 通道数 / Compile-time fixed CAN channel count
 */
template <std::size_t CanChNum>
class GsUsbClass : public DeviceClass
{
 public:
  static_assert(CanChNum > 0 && CanChNum <= 255, "CanChNum must be in (0, 255]");
  static constexpr uint8_t CAN_CH_NUM = static_cast<uint8_t>(CanChNum);
  // GS USB 功能的默认接口字符串。
  // Default interface string for the GS USB function.
  static constexpr const char* DEFAULT_INTERFACE_STRING = "XRUSB GS USB";

 private:
  // ===== Linux gs_usb 线缆格式（固定 12 字节头） / Linux gs_usb wire format
  // (fixed 12-byte header) =====
  LIBXR_PACKED_BEGIN

  /**
   * @brief gs_usb 线缆格式头（12 字节） / gs_usb wire-format header (12 bytes)
   */
  struct WireHeader
  {
    uint32_t echo_id;  ///< 回显 ID / Echo ID
    uint32_t can_id;   ///< CAN ID（含标志位） / CAN ID (with flags)
    uint8_t can_dlc;   ///< DLC
    uint8_t channel;   ///< 通道号 / Channel index
    uint8_t flags;     ///< 帧标志（GS_CAN_FLAG_*） / Frame flags (GS_CAN_FLAG_*)
    uint8_t reserved;  ///< 保留 / Reserved
  };
  LIBXR_PACKED_END

  static constexpr uint32_t ECHO_ID_RX =
      0xFFFFFFFFu;  ///< RX 帧 echo_id 固定值 / Fixed echo_id for RX frames

  static constexpr std::size_t WIRE_HDR_SIZE =
      sizeof(WireHeader);  ///< Header 长度 / Header size
  static constexpr std::size_t WIRE_CLASSIC_DATA_SIZE =
      8;  ///< classic 数据长度 / Classic data size
  static constexpr std::size_t WIRE_FD_DATA_SIZE = 64;  ///< FD 数据长度 / FD data size
  static constexpr std::size_t WIRE_TS_SIZE =
      4;  ///< 时间戳字段长度 / Timestamp field size

  static constexpr std::size_t WIRE_CLASSIC_SIZE =
      WIRE_HDR_SIZE + 8;  ///< classic 总长度 / Classic total size
  static constexpr std::size_t WIRE_CLASSIC_TS_SIZE =
      WIRE_HDR_SIZE + 8 + 4;  ///< classic+ts 总长度 / Classic+ts total size
  static constexpr std::size_t WIRE_FD_SIZE =
      WIRE_HDR_SIZE + 64;  ///< FD 总长度 / FD total size
  static constexpr std::size_t WIRE_FD_TS_SIZE =
      WIRE_HDR_SIZE + 64 + 4;  ///< FD+ts 总长度 / FD+ts total size
  static constexpr std::size_t WIRE_MAX_SIZE =
      WIRE_FD_TS_SIZE;  ///< 最大线缆帧长度 / Maximum wire frame size

 public:
  /**
   * @brief 构造：经典 CAN / Construct: Classic CAN
   * @param cans 经典 CAN 指针列表，数量必须等于 CAN_CH_NUM / Classic CAN pointers, count
   * must equal CAN_CH_NUM
   * @param data_in_ep_num Bulk IN 端点号 / Bulk IN endpoint number
   * @param data_out_ep_num Bulk OUT 端点号 / Bulk OUT endpoint number
   * @param rx_queue_size 接收队列大小 / RX queue size
   * @param echo_queue_size 回显队列大小 / Echo queue size
   * @param identify_gpio Identify GPIO（可选） / Identify GPIO (optional)
   * @param termination_gpios 每通道终端电阻 GPIO（可选） / Per-channel termination GPIOs
   * (optional)
   * @param database 数据库存储（可选，用于 USER_ID） / Database storage (optional, used
   * by USER_ID)
   * @param interface_string 接口字符串 / Interface string
   */
  GsUsbClass(Endpoint::EPNumber data_in_ep_num, Endpoint::EPNumber data_out_ep_num,
             std::initializer_list<LibXR::CAN*> cans, size_t rx_queue_size = 32,
             size_t echo_queue_size = 32, LibXR::GPIO* identify_gpio = nullptr,
             std::initializer_list<LibXR::GPIO*> termination_gpios = {},
             LibXR::Database* database = nullptr,
             const char* interface_string = DEFAULT_INTERFACE_STRING)
      : data_in_ep_num_(data_in_ep_num),
        data_out_ep_num_(data_out_ep_num),
        identify_gpio_(identify_gpio),
        database_(database),
        rx_queue_(rx_queue_size),
        echo_queue_(echo_queue_size),
        interface_string_(interface_string)
  {
    ASSERT(cans.size() == CAN_CH_NUM);
    std::size_t i = 0;
    for (auto* p : cans)
    {
      cans_[i++] = p;
    }

    if (termination_gpios.size() == 0)
    {
      termination_gpios_.fill(nullptr);
    }
    else
    {
      ASSERT(termination_gpios.size() == CAN_CH_NUM);
      i = 0;
      for (auto* g : termination_gpios)
      {
        termination_gpios_[i++] = g;
      }
    }

    InitDeviceConfigClassic();
  }

  const char* GetInterfaceString(size_t local_interface_index) const override
  {
    // GS USB 暴露一个 vendor-specific 接口。
    // GS USB contributes one vendor-specific interface.
    return (local_interface_index == 0u) ? interface_string_ : nullptr;
  }

  /**
   * @brief 构造：FDCAN / Construct: FDCAN
   * @param fd_cans FDCAN 指针列表，数量必须等于 CAN_CH_NUM / FDCAN pointers, count must
   * equal CAN_CH_NUM
   * @param data_in_ep_num Bulk IN 端点号（必填） / Bulk IN endpoint number (required)
   * @param data_out_ep_num Bulk OUT 端点号（必填） / Bulk OUT endpoint number (required)
   * @param rx_queue_size 接收队列大小 / RX queue size
   * @param echo_queue_size 回显队列大小 / Echo queue size
   * @param identify_gpio Identify GPIO（可选） / Identify GPIO (optional)
   * @param termination_gpios 每通道终端电阻 GPIO（可选） / Per-channel termination GPIOs
   * (optional)
   * @param database 数据库存储（可选，用于 USER_ID） / Database storage (optional, used
   * by USER_ID)
   * @param interface_string 接口字符串 / Interface string
   */
  GsUsbClass(Endpoint::EPNumber data_in_ep_num, Endpoint::EPNumber data_out_ep_num,
             std::initializer_list<LibXR::FDCAN*> fd_cans, size_t rx_queue_size = 32,
             size_t echo_queue_size = 32, LibXR::GPIO* identify_gpio = nullptr,
             std::initializer_list<LibXR::GPIO*> termination_gpios = {},
             LibXR::Database* database = nullptr,
             const char* interface_string = DEFAULT_INTERFACE_STRING)
      : fd_supported_(true),
        data_in_ep_num_(data_in_ep_num),
        data_out_ep_num_(data_out_ep_num),
        identify_gpio_(identify_gpio),
        database_(database),
        rx_queue_(rx_queue_size),
        echo_queue_(echo_queue_size),
        interface_string_(interface_string)
  {
    ASSERT(fd_cans.size() == CAN_CH_NUM);
    std::size_t i = 0;
    for (auto* p : fd_cans)
    {
      fdcans_[i] = p;
      cans_[i] = p;  // 向上转 CAN* / Upcast to CAN*
      ++i;
    }

    if (termination_gpios.size() == 0)
    {
      termination_gpios_.fill(nullptr);
    }
    else
    {
      ASSERT(termination_gpios.size() == CAN_CH_NUM);
      i = 0;
      for (auto* g : termination_gpios)
      {
        termination_gpios_[i++] = g;
      }
    }

    InitDeviceConfigFd();
  }

  /**
   * @brief Host 字节序协商是否通过 / Whether host format negotiation passed
   * @return true 协商通过 / Passed
   * @return false 协商未通过 / Not passed
   */
  bool IsHostFormatOK() const { return host_format_ok_; }

 protected:
  /**
   * @brief 初始化接口与端点资源 / Initialize interface and endpoints
   * @param endpoint_pool 端点池 / Endpoint pool
   * @param start_itf_num 起始接口号 / Starting interface number
   * @param in_isr 是否在中断中 / Whether in ISR
   */
  void BindEndpoints(EndpointPool& endpoint_pool, uint8_t start_itf_num, bool) override
  {
    inited_ = false;
    interface_num_ = start_itf_num;

    auto ans = endpoint_pool.Get(ep_data_in_, Endpoint::Direction::IN, data_in_ep_num_);
    ASSERT(ans == ErrorCode::OK);

    ans = endpoint_pool.Get(ep_data_out_, Endpoint::Direction::OUT, data_out_ep_num_);
    ASSERT(ans == ErrorCode::OK);

    // UINT16_MAX 只是上限，底层会选不超过该值的可用最大长度。
    // UINT16_MAX is only an upper bound; the backend still chooses the largest valid
    // length.
    ep_data_in_->Configure(
        {Endpoint::Direction::IN, Endpoint::Type::BULK, UINT16_MAX, true});
    ep_data_out_->Configure(
        {Endpoint::Direction::OUT, Endpoint::Type::BULK, UINT16_MAX, true});

    desc_block_.intf = {9,
                        static_cast<uint8_t>(DescriptorType::INTERFACE),
                        interface_num_,
                        0,
                        2,
                        0xFF,
                        0xFF,
                        0xFF,
                        GetInterfaceStringIndex(0u)};

    desc_block_.ep_out = {7,
                          static_cast<uint8_t>(DescriptorType::ENDPOINT),
                          static_cast<uint8_t>(ep_data_out_->GetAddress()),
                          static_cast<uint8_t>(Endpoint::Type::BULK),
                          ep_data_out_->MaxPacketSize(),
                          0};

    desc_block_.ep_in = {7,
                         static_cast<uint8_t>(DescriptorType::ENDPOINT),
                         static_cast<uint8_t>(ep_data_in_->GetAddress()),
                         static_cast<uint8_t>(Endpoint::Type::BULK),
                         ep_data_in_->MaxPacketSize(),
                         0};

    SetData(RawData{reinterpret_cast<uint8_t*>(&desc_block_), sizeof(desc_block_)});

    ep_data_out_->SetOnTransferCompleteCallback(on_data_out_cb_);
    ep_data_in_->SetOnTransferCompleteCallback(on_data_in_cb_);

    host_format_ok_ = false;

    for (uint8_t i = 0; i < can_count_; ++i)
    {
      can_enabled_[i] = false;
      berr_enabled_[i] = false;
      fd_enabled_[i] = false;
      term_state_[i] = GsUsb::TerminationState::OFF;
      timestamps_enabled_ch_[i] = false;
    }

    // 注册 CAN RX 回调 / Register CAN RX callbacks
    if (!can_rx_registered_)
    {
      for (uint8_t ch = 0; ch < can_count_; ++ch)
      {
        if (!cans_[ch])
        {
          continue;
        }

        can_rx_ctx_[ch].self = this;
        can_rx_ctx_[ch].ch = ch;
        can_rx_cb_[ch] = LibXR::CAN::Callback::Create(OnCanRxStatic, &can_rx_ctx_[ch]);

        cans_[ch]->Register(can_rx_cb_[ch], LibXR::CAN::Type::STANDARD);
        cans_[ch]->Register(can_rx_cb_[ch], LibXR::CAN::Type::EXTENDED);
        cans_[ch]->Register(can_rx_cb_[ch], LibXR::CAN::Type::REMOTE_STANDARD);
        cans_[ch]->Register(can_rx_cb_[ch], LibXR::CAN::Type::REMOTE_EXTENDED);
        cans_[ch]->Register(can_rx_cb_[ch], LibXR::CAN::Type::ERROR);
      }
      can_rx_registered_ = true;
    }

    if (fd_supported_ && !fd_can_rx_registered_)
    {
      for (uint8_t ch = 0; ch < can_count_; ++ch)
      {
        if (!fdcans_[ch])
        {
          continue;
        }

        fd_can_rx_ctx_[ch].self = this;
        fd_can_rx_ctx_[ch].ch = ch;
        fd_can_rx_cb_[ch] =
            LibXR::FDCAN::CallbackFD::Create(OnFdCanRxStatic, &fd_can_rx_ctx_[ch]);

        fdcans_[ch]->Register(fd_can_rx_cb_[ch], LibXR::CAN::Type::STANDARD);
        fdcans_[ch]->Register(fd_can_rx_cb_[ch], LibXR::CAN::Type::EXTENDED);
      }
      fd_can_rx_registered_ = true;
    }

    inited_ = true;
    MaybeArmOutTransfer();
  }

  /**
   * @brief 释放端点资源并复位状态 / Release endpoint resources and reset state
   * @param endpoint_pool 端点池 / Endpoint pool
   * @param in_isr 是否在中断中 / Whether in ISR
   */
  void UnbindEndpoints(EndpointPool& endpoint_pool, bool) override
  {
    inited_ = false;
    host_format_ok_ = false;

    for (uint8_t i = 0; i < can_count_; ++i)
    {
      can_enabled_[i] = false;
      berr_enabled_[i] = false;
      fd_enabled_[i] = false;
      term_state_[i] = GsUsb::TerminationState::OFF;
      timestamps_enabled_ch_[i] = false;
    }

    if (ep_data_in_)
    {
      ep_data_in_->Close();
      ep_data_in_->SetActiveLength(0);
      endpoint_pool.Release(ep_data_in_);
      ep_data_in_ = nullptr;
    }

    if (ep_data_out_)
    {
      ep_data_out_->Close();
      ep_data_out_->SetActiveLength(0);
      endpoint_pool.Release(ep_data_out_);
      ep_data_out_ = nullptr;
    }
  }

  /**
   * @brief 返回接口数量（实现侧固定 1） / Return interface count (fixed to 1)
   * @return size_t 接口数 / Interface count
   */
  size_t GetInterfaceCount() override { return 1; }

  /**
   * @brief 是否包含 IAD / Whether class has IAD
   * @return true 有 IAD / Has IAD
   * @return false 无 IAD / No IAD
   */
  bool HasIAD() override { return false; }

  /**
   * @brief 判断端点地址是否属于本类 / Check whether an endpoint belongs to this class
   * @param ep_addr 端点地址 / Endpoint address
   * @return true 属于 / Owned
   * @return false 不属于 / Not owned
   */
  bool OwnsEndpoint(uint8_t ep_addr) const override
  {
    if (!inited_)
    {
      return false;
    }

    return (ep_data_in_ && ep_data_in_->GetAddress() == ep_addr) ||
           (ep_data_out_ && ep_data_out_->GetAddress() == ep_addr);
  }

  /**
   * @brief 返回描述符块最大长度 / Return max configuration descriptor block size
   * @return size_t 长度 / Size
   */
  size_t GetMaxConfigSize() override { return sizeof(desc_block_); }

  /**
   * @brief 标准 Class Request 处理（此类不支持） / Handle standard class request (not
   * supported)
   * @return ErrorCode 错误码 / Error code
   */
  ErrorCode OnClassRequest(bool, uint8_t, uint16_t, uint16_t, uint16_t,
                           DeviceClass::ControlTransferResult&) override
  {
    return ErrorCode::NOT_SUPPORT;
  }

  /**
   * @brief Vendor Request（gs_usb BREQ）处理 / Handle vendor requests (gs_usb BREQ)
   * @param in_isr 是否在中断上下文 / Whether in ISR
   * @param bRequest 请求号（GsUsb::BReq） / Request number (GsUsb::BReq)
   * @param wValue wValue
   * @param wLength 数据阶段长度 / Data stage length
   * @param wIndex wIndex
   * @param result 读写缓冲设置 / Read/write buffer setup
   * @return ErrorCode 错误码 / Error code
   */
  ErrorCode OnVendorRequest(bool in_isr, uint8_t bRequest, uint16_t wValue,
                            uint16_t wLength, uint16_t wIndex,
                            DeviceClass::ControlTransferResult& result) override
  {
    UNUSED(in_isr);
    UNUSED(wIndex);

    auto req = static_cast<GsUsb::BReq>(bRequest);

    switch (req)
    {
      // ===== Device -> Host =====
      case GsUsb::BReq::BT_CONST:
      {
        if (wLength < sizeof(bt_const_))
        {
          return ErrorCode::ARG_ERR;
        }
        result.write_data =
            ConstRawData{reinterpret_cast<const uint8_t*>(&bt_const_), sizeof(bt_const_)};
        return ErrorCode::OK;
      }

      case GsUsb::BReq::BT_CONST_EXT:
      {
        if (!fd_supported_)
        {
          return ErrorCode::NOT_SUPPORT;
        }
        if (wLength < sizeof(bt_const_ext_))
        {
          return ErrorCode::ARG_ERR;
        }
        result.write_data = ConstRawData{reinterpret_cast<const uint8_t*>(&bt_const_ext_),
                                         sizeof(bt_const_ext_)};
        return ErrorCode::OK;
      }

      case GsUsb::BReq::DEVICE_CONFIG:
      {
        if (wLength < sizeof(dev_cfg_))
        {
          return ErrorCode::ARG_ERR;
        }
        result.write_data =
            ConstRawData{reinterpret_cast<const uint8_t*>(&dev_cfg_), sizeof(dev_cfg_)};
        return ErrorCode::OK;
      }

      case GsUsb::BReq::TIMESTAMP:
      {
        ctrl_buf_.timestamp_us = MakeTimestampUsGlobal();
        if (wLength < sizeof(ctrl_buf_.timestamp_us))
        {
          return ErrorCode::ARG_ERR;
        }

        result.write_data =
            ConstRawData{reinterpret_cast<const uint8_t*>(&ctrl_buf_.timestamp_us),
                         sizeof(ctrl_buf_.timestamp_us)};
        return ErrorCode::OK;
      }

      case GsUsb::BReq::GET_TERMINATION:
      {
        if (wValue >= can_count_)
        {
          return ErrorCode::ARG_ERR;
        }
        ctrl_buf_.term.state = static_cast<uint32_t>(term_state_[wValue]);
        if (wLength < sizeof(ctrl_buf_.term))
        {
          return ErrorCode::ARG_ERR;
        }

        result.write_data = ConstRawData{
            reinterpret_cast<const uint8_t*>(&ctrl_buf_.term), sizeof(ctrl_buf_.term)};
        return ErrorCode::OK;
      }

      case GsUsb::BReq::GET_STATE:
      {
        if (wValue >= can_count_)
        {
          return ErrorCode::ARG_ERR;
        }

        GsUsb::CanState st = GsUsb::CanState::ERROR_ACTIVE;
        uint32_t rxerr = 0;
        uint32_t txerr = 0;

        auto* can = cans_[wValue];
        if (can != nullptr)
        {
          LibXR::CAN::ErrorState es;
          if (can->GetErrorState(es) == ErrorCode::OK)
          {
            if (es.bus_off)
            {
              st = GsUsb::CanState::BUS_OFF;
            }
            else if (es.error_passive)
            {
              st = GsUsb::CanState::ERROR_PASSIVE;
            }
            else if (es.error_warning)
            {
              st = GsUsb::CanState::ERROR_WARNING;
            }
            else
            {
              st = GsUsb::CanState::ERROR_ACTIVE;
            }

            rxerr = es.rx_error_counter;
            txerr = es.tx_error_counter;
          }
        }

        ctrl_buf_.dev_state.state = static_cast<uint32_t>(st);
        ctrl_buf_.dev_state.rxerr = rxerr;
        ctrl_buf_.dev_state.txerr = txerr;

        if (wLength < sizeof(ctrl_buf_.dev_state))
        {
          return ErrorCode::ARG_ERR;
        }

        result.write_data =
            ConstRawData{reinterpret_cast<const uint8_t*>(&ctrl_buf_.dev_state),
                         sizeof(ctrl_buf_.dev_state)};
        return ErrorCode::OK;
      }

      case GsUsb::BReq::GET_USER_ID:
      {
        ctrl_buf_.user_id = GetUserIdFromStorage();
        if (wLength < sizeof(ctrl_buf_.user_id))
        {
          return ErrorCode::ARG_ERR;
        }

        result.write_data =
            ConstRawData{reinterpret_cast<const uint8_t*>(&ctrl_buf_.user_id),
                         sizeof(ctrl_buf_.user_id)};
        return ErrorCode::OK;
      }

      // ===== Host -> Device（有 DATA 阶段） / Host -> Device (with DATA stage) =====
      case GsUsb::BReq::HOST_FORMAT:
      {
        if (wLength != sizeof(GsUsb::HostConfig))
        {
          return ErrorCode::ARG_ERR;
        }
        result.read_data = RawData{reinterpret_cast<uint8_t*>(&ctrl_buf_.host_cfg),
                                   sizeof(GsUsb::HostConfig)};
        return ErrorCode::OK;
      }

      case GsUsb::BReq::BITTIMING:
      {
        if (wLength != sizeof(GsUsb::DeviceBitTiming) || wValue >= can_count_)
        {
          return ErrorCode::ARG_ERR;
        }
        ctrl_target_channel_ = static_cast<uint8_t>(wValue);
        result.read_data = RawData{reinterpret_cast<uint8_t*>(&ctrl_buf_.bt),
                                   sizeof(GsUsb::DeviceBitTiming)};
        return ErrorCode::OK;
      }

      case GsUsb::BReq::DATA_BITTIMING:
      {
        if (!fd_supported_)
        {
          return ErrorCode::NOT_SUPPORT;
        }
        if (wLength != sizeof(GsUsb::DeviceBitTiming) || wValue >= can_count_)
        {
          return ErrorCode::ARG_ERR;
        }
        ctrl_target_channel_ = static_cast<uint8_t>(wValue);
        result.read_data = RawData{reinterpret_cast<uint8_t*>(&ctrl_buf_.bt),
                                   sizeof(GsUsb::DeviceBitTiming)};
        return ErrorCode::OK;
      }

      case GsUsb::BReq::MODE:
      {
        if (wLength != sizeof(GsUsb::DeviceMode) || wValue >= can_count_)
        {
          return ErrorCode::ARG_ERR;
        }
        ctrl_target_channel_ = static_cast<uint8_t>(wValue);
        result.read_data = RawData{reinterpret_cast<uint8_t*>(&ctrl_buf_.mode),
                                   sizeof(GsUsb::DeviceMode)};
        return ErrorCode::OK;
      }

      case GsUsb::BReq::BERR:
      {
        if (wLength != sizeof(uint32_t) || wValue >= can_count_)
        {
          return ErrorCode::ARG_ERR;
        }
        ctrl_target_channel_ = static_cast<uint8_t>(wValue);
        result.read_data =
            RawData{reinterpret_cast<uint8_t*>(&ctrl_buf_.berr_on), sizeof(uint32_t)};
        return ErrorCode::OK;
      }

      case GsUsb::BReq::IDENTIFY:
      {
        if (wLength != sizeof(GsUsb::Identify) || wValue >= can_count_)
        {
          return ErrorCode::ARG_ERR;
        }
        ctrl_target_channel_ = static_cast<uint8_t>(wValue);
        result.read_data = RawData{reinterpret_cast<uint8_t*>(&ctrl_buf_.identify),
                                   sizeof(GsUsb::Identify)};
        return ErrorCode::OK;
      }

      case GsUsb::BReq::SET_TERMINATION:
      {
        if (wLength != sizeof(GsUsb::DeviceTerminationState) || wValue >= can_count_)
        {
          return ErrorCode::ARG_ERR;
        }
        ctrl_target_channel_ = static_cast<uint8_t>(wValue);
        result.read_data = RawData{reinterpret_cast<uint8_t*>(&ctrl_buf_.term),
                                   sizeof(GsUsb::DeviceTerminationState)};
        return ErrorCode::OK;
      }

      case GsUsb::BReq::SET_USER_ID:
      {
        if (wLength != sizeof(uint32_t))
        {
          return ErrorCode::ARG_ERR;
        }
        result.read_data =
            RawData{reinterpret_cast<uint8_t*>(&ctrl_buf_.user_id), sizeof(uint32_t)};
        return ErrorCode::OK;
      }

      default:
        return ErrorCode::NOT_SUPPORT;
    }
  }

  /**
   * @brief Vendor Request 的 DATA 阶段处理 / Handle DATA stage for vendor requests
   * @param in_isr 是否在中断上下文 / Whether in ISR
   * @param bRequest 请求号 / Request number
   * @param data DATA 阶段数据 / DATA stage data
   * @return ErrorCode 错误码 / Error code
   */
  ErrorCode OnClassData(bool in_isr, uint8_t bRequest, LibXR::ConstRawData& data) override
  {
    UNUSED(in_isr);

    auto req = static_cast<GsUsb::BReq>(bRequest);

    switch (req)
    {
      case GsUsb::BReq::HOST_FORMAT:
        if (data.size_ != sizeof(GsUsb::HostConfig))
        {
          return ErrorCode::ARG_ERR;
        }
        return HandleHostFormat(ctrl_buf_.host_cfg);

      case GsUsb::BReq::BITTIMING:
        if (data.size_ != sizeof(GsUsb::DeviceBitTiming))
        {
          return ErrorCode::ARG_ERR;
        }
        return HandleBitTiming(ctrl_target_channel_, ctrl_buf_.bt);

      case GsUsb::BReq::DATA_BITTIMING:
        if (!fd_supported_)
        {
          return ErrorCode::NOT_SUPPORT;
        }
        if (data.size_ != sizeof(GsUsb::DeviceBitTiming))
        {
          return ErrorCode::ARG_ERR;
        }
        return HandleDataBitTiming(ctrl_target_channel_, ctrl_buf_.bt);

      case GsUsb::BReq::MODE:
        if (data.size_ != sizeof(GsUsb::DeviceMode))
        {
          return ErrorCode::ARG_ERR;
        }
        return HandleMode(ctrl_target_channel_, ctrl_buf_.mode);

      case GsUsb::BReq::BERR:
        if (data.size_ != sizeof(uint32_t))
        {
          return ErrorCode::ARG_ERR;
        }
        return HandleBerr(ctrl_target_channel_, ctrl_buf_.berr_on);

      case GsUsb::BReq::IDENTIFY:
        if (data.size_ != sizeof(GsUsb::Identify))
        {
          return ErrorCode::ARG_ERR;
        }
        return HandleIdentify(ctrl_target_channel_, ctrl_buf_.identify);

      case GsUsb::BReq::SET_TERMINATION:
        if (data.size_ != sizeof(GsUsb::DeviceTerminationState))
        {
          return ErrorCode::ARG_ERR;
        }
        return HandleSetTermination(ctrl_target_channel_, ctrl_buf_.term);

      case GsUsb::BReq::SET_USER_ID:
        if (data.size_ != sizeof(uint32_t))
        {
          return ErrorCode::ARG_ERR;
        }
        SetUserIdToStorage(ctrl_buf_.user_id);
        return ErrorCode::OK;

      case GsUsb::BReq::DEVICE_CONFIG:
      case GsUsb::BReq::BT_CONST:
      case GsUsb::BReq::BT_CONST_EXT:
      case GsUsb::BReq::TIMESTAMP:
      case GsUsb::BReq::GET_TERMINATION:
      case GsUsb::BReq::GET_STATE:
      case GsUsb::BReq::GET_USER_ID:
        return ErrorCode::OK;

      default:
        return ErrorCode::NOT_SUPPORT;
    }
  }

  // ================= Bulk 端点回调 / Bulk endpoint callbacks =================

  /**
   * @brief Bulk OUT 完成静态回调包装 / Static wrapper for Bulk OUT completion
   * @param in_isr 是否在中断上下文 / Whether in ISR
   * @param self this 指针 / this pointer
   * @param data 收到的数据 / Received data
   */
  static void OnDataOutCompleteStatic(bool in_isr, GsUsbClass* self, ConstRawData& data)
  {
    if (self == nullptr || !self->inited_)
    {
      return;
    }
    self->OnDataOutComplete(in_isr, data);
  }

  /**
   * @brief Bulk IN 完成静态回调包装 / Static wrapper for Bulk IN completion
   * @param in_isr 是否在中断上下文 / Whether in ISR
   * @param self this 指针 / this pointer
   * @param data 发送完成的数据视图 / Completed transfer data view
   */
  static void OnDataInCompleteStatic(bool in_isr, GsUsbClass* self, ConstRawData& data)
  {
    if (self == nullptr || !self->inited_)
    {
      return;
    }
    self->OnDataInComplete(in_isr, data);
  }

  /**
   * @brief Bulk OUT 完成处理（Host->Device） / Handle Bulk OUT completion (Host->Device)
   * @param in_isr 是否在中断上下文 / Whether in ISR
   * @param data 收到的数据 / Received data
   */
  void OnDataOutComplete(bool in_isr, ConstRawData& data)
  {
    UNUSED(in_isr);

    if (!ep_data_out_)
    {
      return;
    }
    if (!host_format_ok_)
    {
      MaybeArmOutTransfer();
      return;
    }

    const std::size_t RXLEN = data.size_;
    if (RXLEN < WIRE_CLASSIC_SIZE)
    {
      MaybeArmOutTransfer();
      return;
    }

    // 解析 wire header / Parse wire header
    const WireHeader& wh = *reinterpret_cast<const WireHeader*>(data.addr_);

    const uint8_t CH = wh.channel;
    if (CH >= can_count_ || !cans_[CH])
    {
      MaybeArmOutTransfer();
      return;
    }

    const bool IS_FD = (wh.flags & GsUsb::CAN_FLAG_FD) != 0;
    const uint8_t* payload = reinterpret_cast<const uint8_t*>(data.addr_) + WIRE_HDR_SIZE;

    if (IS_FD)
    {
      if (!fd_supported_ || !fdcans_[CH] || !fd_enabled_[CH])
      {
        MaybeArmOutTransfer();
        return;
      }

      if (RXLEN < WIRE_FD_SIZE)
      {
        MaybeArmOutTransfer();
        return;
      }

      LibXR::FDCAN::FDPack pack;
      HostWireToFdPack(wh, payload, pack);
      (void)fdcans_[CH]->AddMessage(pack);
    }
    else
    {
      if (!can_enabled_[CH])
      {
        MaybeArmOutTransfer();
        return;
      }

      if (RXLEN < WIRE_CLASSIC_SIZE)
      {
        MaybeArmOutTransfer();
        return;
      }

      LibXR::CAN::ClassicPack pack;
      HostWireToClassicPack(wh, payload, pack);
      (void)cans_[CH]->AddMessage(pack);
    }

    // TX echo：host 通过 echo_id 跟踪 TX buffer；设备需回送 echo_id。
    // TX echo: the host tracks TX buffers via echo_id, and the device should echo it
    // back.
    if (wh.echo_id != ECHO_ID_RX)
    {
      QueueItem qi;
      qi.hdr = wh;
      qi.is_fd = IS_FD;
      qi.data_len = IS_FD ? 64u : 8u;
      qi.timestamp_us = MakeTimestampUs(CH);
      Memory::FastCopy(qi.data.data(), payload, qi.data_len);
      (void)EnqueueFrame(qi, true, in_isr);
    }

    MaybeArmOutTransfer();
  }

  /**
   * @brief Bulk IN 完成处理（Device->Host） / Handle Bulk IN completion (Device->Host)
   * @param in_isr 是否在中断上下文 / Whether in ISR
   * @param data 发送完成的数据视图 / Completed transfer data view
   */
  void OnDataInComplete(bool in_isr, ConstRawData& data)
  {
    UNUSED(in_isr);
    UNUSED(data);
    TryKickTx(false);
  }

 private:
  // ================= 成员 / Members =================
  std::array<LibXR::CAN*, CanChNum> cans_{};  ///< CAN 通道列表 / CAN channel pointers
  std::array<LibXR::FDCAN*, CanChNum>
      fdcans_{};               ///< FDCAN 通道列表 / FDCAN channel pointers
  bool fd_supported_ = false;  ///< 是否支持 FD / FD supported

  uint8_t can_count_ = CAN_CH_NUM;  ///< 实际通道数 / Actual channel count

  Endpoint::EPNumber data_in_ep_num_;   ///< Bulk IN 端点号 / Bulk IN endpoint number
  Endpoint::EPNumber data_out_ep_num_;  ///< Bulk OUT 端点号 / Bulk OUT endpoint number

  Endpoint* ep_data_in_ = nullptr;   ///< Bulk IN 端点对象 / Bulk IN endpoint object
  Endpoint* ep_data_out_ = nullptr;  ///< Bulk OUT 端点对象 / Bulk OUT endpoint object

  bool inited_ = false;                     ///< 是否已初始化 / Initialized
  uint8_t interface_num_ = 0;               ///< 接口号 / Interface number
  const char* interface_string_ = nullptr;  ///< 接口字符串 / Interface string

  LibXR::GPIO* identify_gpio_ =
      nullptr;  ///< Identify GPIO（可选） / Identify GPIO (optional)
  std::array<LibXR::GPIO*, CanChNum>
      termination_gpios_{};  ///< 终端电阻 GPIO（可选） / Termination GPIOs (optional)

  LibXR::Database* database_ = nullptr;  ///< 数据库存储（可选，用于 USER_ID） / Database
                                         ///< storage (optional, used by USER_ID)
  uint32_t user_id_ram_ = 0;             ///< USER_ID 的 RAM 备份 / USER_ID RAM backup

  LibXR::Callback<LibXR::ConstRawData&> on_data_out_cb_ =
      LibXR::Callback<LibXR::ConstRawData&>::Create(OnDataOutCompleteStatic,
                                                    this);  ///< OUT 回调 / OUT callback

  LibXR::Callback<LibXR::ConstRawData&> on_data_in_cb_ =
      LibXR::Callback<LibXR::ConstRawData&>::Create(OnDataInCompleteStatic,
                                                    this);  ///< IN 回调 / IN callback

  GsUsb::DeviceConfig dev_cfg_{};    ///< 设备配置 / Device configuration
  GsUsb::DeviceBTConst bt_const_{};  ///< BT 常量（classic） / BT constants (classic)
  GsUsb::DeviceBTConstExtended
      bt_const_ext_{};  ///< BT 常量（扩展/FD） / BT constants (extended/FD)

  /**
   * @brief 控制传输复用缓冲区 / Union buffer for control transfers
   */
  union
  {
    GsUsb::HostConfig host_cfg;          ///< HOST_FORMAT
    GsUsb::DeviceBitTiming bt;           ///< BITTIMING / DATA_BITTIMING
    GsUsb::DeviceMode mode;              ///< MODE
    uint32_t berr_on;                    ///< BERR
    GsUsb::Identify identify;            ///< IDENTIFY
    uint32_t timestamp_us;               ///< TIMESTAMP
    GsUsb::DeviceTerminationState term;  ///< SET_TERMINATION / GET_TERMINATION
    GsUsb::DeviceState dev_state;        ///< GET_STATE
    uint32_t user_id;                    ///< GET_USER_ID / SET_USER_ID
  } ctrl_buf_{};

  std::array<LibXR::CAN::Configuration, CanChNum>
      config_{};  ///< 经典 CAN 配置 / Classic CAN configuration
  std::array<LibXR::FDCAN::Configuration, CanChNum>
      fd_config_{};  ///< FD 配置 / FD configuration

  bool host_format_ok_ = false;  ///< HOST_FORMAT 是否通过 / HOST_FORMAT OK

  bool can_enabled_[CanChNum] = {
      false};  ///< 通道启用（classic） / Channel enabled (classic)
  bool berr_enabled_[CanChNum] = {false};  ///< 错误报告启用 / Bus error reporting enabled
  bool fd_enabled_[CanChNum] = {false};    ///< 通道启用（FD） / Channel enabled (FD)

  bool timestamps_enabled_ch_[CanChNum] = {
      false};  ///< 通道时间戳启用 / Per-channel timestamp enabled
  GsUsb::TerminationState term_state_[CanChNum] = {
      GsUsb::TerminationState::OFF};  ///< 终端电阻状态 / Termination state
  uint8_t ctrl_target_channel_ =
      0;  ///< 控制请求目标通道 / Control request target channel

  LIBXR_PACKED_BEGIN
  /**
   * @brief 本类的接口与端点描述符块 / Descriptor block for interface and endpoints
   */
  struct GsUsbDescBlock
  {
    InterfaceDescriptor intf;   ///< Interface 描述符 / Interface descriptor
    EndpointDescriptor ep_out;  ///< OUT 端点描述符 / OUT endpoint descriptor
    EndpointDescriptor ep_in;   ///< IN 端点描述符 / IN endpoint descriptor
  } desc_block_{};
  LIBXR_PACKED_END

  uint8_t rx_buf_[WIRE_MAX_SIZE]{};  ///< OUT 接收缓冲区 / OUT receive buffer
  std::array<uint8_t, WIRE_MAX_SIZE>
      tx_buf_{};  ///< IN 发送暂存区 / IN transmit staging buffer

  // ================= CAN RX 回调 & BULK IN 发送队列 / CAN RX callbacks & Bulk IN TX
  // queues =================

  /**
   * @brief CAN RX 回调上下文 / CAN RX callback context
   */
  struct CanRxCtx
  {
    GsUsbClass* self;  ///< 实例指针 / Instance pointer
    uint8_t ch;        ///< 通道号 / Channel index
  };

  /**
   * @brief FDCAN RX 回调上下文 / FDCAN RX callback context
   */
  struct FdCanRxCtx
  {
    GsUsbClass* self;  ///< 实例指针 / Instance pointer
    uint8_t ch;        ///< 通道号 / Channel index
  };

  bool can_rx_registered_ =
      false;  ///< classic RX 回调是否已注册 / Classic RX callback registered
  CanRxCtx can_rx_ctx_[CanChNum]{};  ///< classic RX 上下文数组 / Classic RX contexts
  LibXR::CAN::Callback
      can_rx_cb_[CanChNum]{};  ///< classic RX 回调数组 / Classic RX callbacks

  bool fd_can_rx_registered_ =
      false;  ///< FD RX 回调是否已注册 / FD RX callback registered
  FdCanRxCtx fd_can_rx_ctx_[CanChNum]{};  ///< FD RX 上下文数组 / FD RX contexts
  LibXR::FDCAN::CallbackFD
      fd_can_rx_cb_[CanChNum]{};  ///< FD RX 回调数组 / FD RX callbacks

  /**
   * @brief classic CAN RX 静态回调入口 / Static entry for classic CAN RX callback
   */
  static void OnCanRxStatic(bool in_isr, CanRxCtx* ctx,
                            const LibXR::CAN::ClassicPack& pack)
  {
    if (!ctx || !ctx->self || !ctx->self->inited_)
    {
      return;
    }
    ctx->self->OnCanRx(in_isr, ctx->ch, pack);
  }

  /**
   * @brief FD CAN RX 静态回调入口 / Static entry for FD CAN RX callback
   */
  static void OnFdCanRxStatic(bool in_isr, FdCanRxCtx* ctx,
                              const LibXR::FDCAN::FDPack& pack)
  {
    if (!ctx || !ctx->self || !ctx->self->inited_)
    {
      return;
    }
    ctx->self->OnFdCanRx(in_isr, ctx->ch, pack);
  }

  /**
   * @brief 发送队列元素（包含 header/data/timestamp） / TX queue item
   * (header/data/timestamp)
   */
  struct QueueItem
  {
    WireHeader hdr;                ///< 线缆头 / Wire header
    bool is_fd;                    ///< 是否 FD / Is FD
    uint8_t data_len;              ///< payload 长度（8/64） / Payload length (8/64)
    uint32_t timestamp_us;         ///< 时间戳 / Timestamp
    std::array<uint8_t, 64> data;  ///< 数据段 / Data bytes
  };

  LibXR::SPSCQueue<QueueItem>
      rx_queue_;  ///< RX 队列（Device->Host） / RX queue (Device->Host)
  LibXR::SPSCQueue<QueueItem>
      echo_queue_;  ///< Echo 队列（回送 echo_id） / Echo queue (echo back echo_id)

  /**
   * @brief classic CAN RX 处理 / Handle classic CAN RX
   */
  void OnCanRx(bool in_isr, uint8_t ch, const LibXR::CAN::ClassicPack& pack)
  {
    if (ch >= can_count_ || !ep_data_in_)
    {
      return;
    }

    if (pack.type == LibXR::CAN::Type::ERROR)
    {
      QueueItem qi;
      if (ErrorPackToHostErrorFrame(ch, pack, qi))
      {
        (void)EnqueueFrame(qi, false, in_isr);
      }
      return;
    }

    if (!can_enabled_[ch])
    {
      return;
    }

    QueueItem qi;
    ClassicPackToQueueItem(pack, ch, qi);
    (void)EnqueueFrame(qi, false, in_isr);
  }

  /**
   * @brief FD CAN RX 处理 / Handle FD CAN RX
   */
  void OnFdCanRx(bool in_isr, uint8_t ch, const LibXR::FDCAN::FDPack& pack)
  {
    if (!fd_supported_ || ch >= can_count_ || !fd_enabled_[ch] || !ep_data_in_)
    {
      return;
    }

    QueueItem qi;
    FdPackToQueueItem(pack, ch, qi);

    const auto& fd_cfg = fd_config_[ch];
    if (fd_cfg.fd_mode.brs)
    {
      qi.hdr.flags |= GsUsb::CAN_FLAG_BRS;
    }
    if (fd_cfg.fd_mode.esi)
    {
      qi.hdr.flags |= GsUsb::CAN_FLAG_ESI;
    }

    (void)EnqueueFrame(qi, false, in_isr);
  }

  /**
   * @brief 入队并尝试触发发送 / Enqueue and try to trigger TX
   * @param qi 队列元素 / Queue item
   * @param is_echo 是否 echo 队列 / Whether enqueue to echo queue
   * @param in_isr 是否在中断上下文 / Whether in ISR
   * @return true 入队成功 / Enqueued
   * @return false 入队失败 / Failed
   */
  bool EnqueueFrame(const QueueItem& qi, bool is_echo, bool in_isr)
  {
    UNUSED(in_isr);

    const ErrorCode EC = is_echo ? echo_queue_.Push(qi) : rx_queue_.Push(qi);
    if (EC != ErrorCode::OK)
    {
      return false;
    }

    TryKickTx(in_isr);
    MaybeArmOutTransfer();
    return true;
  }

  /**
   * @brief 尝试启动 Bulk IN 发送 / Try to start Bulk IN transmit
   * @param in_isr 是否在中断上下文 / Whether in ISR
   */
  void TryKickTx(bool in_isr)
  {
    UNUSED(in_isr);

    if (!ep_data_in_)
    {
      return;
    }
    if (ep_data_in_->GetState() != Endpoint::State::IDLE)
    {
      return;
    }

    QueueItem qi;
    ErrorCode ec = echo_queue_.Pop(qi);
    if (ec != ErrorCode::OK)
    {
      ec = rx_queue_.Pop(qi);
      if (ec != ErrorCode::OK)
      {
        return;
      }
    }

    const std::size_t SEND_LEN = PackQueueItemToWire(qi, tx_buf_.data(), tx_buf_.size());
    if (SEND_LEN == 0)
    {
      return;
    }

    RawData tx_raw{tx_buf_.data(), SEND_LEN};
    (void)ep_data_in_->TransferMultiBulk(tx_raw);

    MaybeArmOutTransfer();
  }

  /**
   * @brief 确保 Bulk OUT 保持挂起接收 / Ensure Bulk OUT is armed for receiving
   */
  void MaybeArmOutTransfer()
  {
    if (!ep_data_out_)
    {
      return;
    }
    if (ep_data_out_->GetState() != Endpoint::State::IDLE)
    {
      return;
    }

    if (rx_queue_.EmptySize() == 0 || echo_queue_.EmptySize() == 0)
    {
      return;
    }

    RawData rx_raw{rx_buf_, static_cast<size_t>(WIRE_MAX_SIZE)};
    (void)ep_data_out_->TransferMultiBulk(rx_raw);
  }

  // ================= 业务处理函数 / Handlers =================

  /**
   * @brief 是否存在任意 termination GPIO / Whether any termination GPIO exists
   * @return true 存在 / Exists
   * @return false 不存在 / Not exists
   */
  bool HasAnyTerminationGpio() const
  {
    for (uint8_t i = 0; i < can_count_; ++i)
    {
      if (termination_gpios_[i] != nullptr)
      {
        return true;
      }
    }
    return false;
  }

  /**
   * @brief 初始化 classic CAN 设备配置与能力位 / Initialize classic CAN device config and
   * feature bits
   */
  void InitDeviceConfigClassic()
  {
    can_count_ = CAN_CH_NUM;
    ASSERT(cans_[0] != nullptr);

    dev_cfg_.reserved1 = 0;
    dev_cfg_.reserved2 = 0;
    dev_cfg_.reserved3 = 0;
    dev_cfg_.icount = static_cast<uint8_t>(can_count_ - 1);
    dev_cfg_.sw_version = 2;
    dev_cfg_.hw_version = 1;

    const uint32_t FCLK = cans_[0]->GetClockFreq();

    bt_const_.feature = GsUsb::CAN_FEAT_LISTEN_ONLY | GsUsb::CAN_FEAT_LOOP_BACK |
                        GsUsb::CAN_FEAT_TRIPLE_SAMPLE | GsUsb::CAN_FEAT_ONE_SHOT |
                        GsUsb::CAN_FEAT_HW_TIMESTAMP | GsUsb::CAN_FEAT_BERR_REPORTING |
                        GsUsb::CAN_FEAT_GET_STATE | GsUsb::CAN_FEAT_USER_ID;

    if (identify_gpio_)
    {
      bt_const_.feature |= GsUsb::CAN_FEAT_IDENTIFY;
    }
    if (HasAnyTerminationGpio())
    {
      bt_const_.feature |= GsUsb::CAN_FEAT_TERMINATION;
    }

    bt_const_.fclk_can = FCLK;
    bt_const_.btc.tseg1_min = 1;
    bt_const_.btc.tseg1_max = 16;
    bt_const_.btc.tseg2_min = 1;
    bt_const_.btc.tseg2_max = 8;
    bt_const_.btc.sjw_max = 4;
    bt_const_.btc.brp_min = 1;
    bt_const_.btc.brp_max = 1024;
    bt_const_.btc.brp_inc = 1;

    bt_const_ext_.feature = bt_const_.feature;
    bt_const_ext_.fclk_can = FCLK;
    bt_const_ext_.btc = bt_const_.btc;
    bt_const_ext_.dbtc = bt_const_.btc;

    Memory::FastSet(config_.data(), 0, sizeof(config_));
    Memory::FastSet(fd_config_.data(), 0, sizeof(fd_config_));
  }

  /**
   * @brief 初始化 FD 设备配置与能力位 / Initialize FD device config and feature bits
   */
  void InitDeviceConfigFd()
  {
    can_count_ = CAN_CH_NUM;
    ASSERT(cans_[0] != nullptr);

    dev_cfg_.reserved1 = 0;
    dev_cfg_.reserved2 = 0;
    dev_cfg_.reserved3 = 0;
    dev_cfg_.icount = static_cast<uint8_t>(can_count_ - 1);
    dev_cfg_.sw_version = 2;
    dev_cfg_.hw_version = 1;

    const uint32_t FCLK = cans_[0]->GetClockFreq();

    bt_const_.feature = GsUsb::CAN_FEAT_LISTEN_ONLY | GsUsb::CAN_FEAT_LOOP_BACK |
                        GsUsb::CAN_FEAT_TRIPLE_SAMPLE | GsUsb::CAN_FEAT_ONE_SHOT |
                        GsUsb::CAN_FEAT_HW_TIMESTAMP | GsUsb::CAN_FEAT_BERR_REPORTING |
                        GsUsb::CAN_FEAT_GET_STATE | GsUsb::CAN_FEAT_USER_ID |
                        GsUsb::CAN_FEAT_FD | GsUsb::CAN_FEAT_BT_CONST_EXT;

    if (identify_gpio_)
    {
      bt_const_.feature |= GsUsb::CAN_FEAT_IDENTIFY;
    }
    if (HasAnyTerminationGpio())
    {
      bt_const_.feature |= GsUsb::CAN_FEAT_TERMINATION;
    }

    bt_const_.fclk_can = FCLK;
    bt_const_.btc.tseg1_min = 1;
    bt_const_.btc.tseg1_max = 16;
    bt_const_.btc.tseg2_min = 1;
    bt_const_.btc.tseg2_max = 8;
    bt_const_.btc.sjw_max = 4;
    bt_const_.btc.brp_min = 1;
    bt_const_.btc.brp_max = 1024;
    bt_const_.btc.brp_inc = 1;

    bt_const_ext_.feature = bt_const_.feature;
    bt_const_ext_.fclk_can = FCLK;
    bt_const_ext_.btc = bt_const_.btc;
    bt_const_ext_.dbtc = bt_const_.btc;

    Memory::FastSet(config_.data(), 0, sizeof(config_));
    Memory::FastSet(fd_config_.data(), 0, sizeof(fd_config_));
  }

  /**
   * @brief 处理 HOST_FORMAT / Handle HOST_FORMAT
   * @param cfg HostConfig
   * @return ErrorCode 错误码 / Error code
   */
  ErrorCode HandleHostFormat(const GsUsb::HostConfig& cfg)
  {
    host_format_ok_ = (cfg.byte_order == 0x0000beefu);
    return ErrorCode::OK;
  }

  /**
   * @brief 处理 BITTIMING（仲裁相位） / Handle BITTIMING (arbitration phase)
   */
  ErrorCode HandleBitTiming(uint8_t ch, const GsUsb::DeviceBitTiming& bt)
  {
    if (!host_format_ok_ || ch >= can_count_ || !cans_[ch])
    {
      return ErrorCode::ARG_ERR;
    }

    const uint32_t TSEG1 = bt.prop_seg + bt.phase_seg1;
    const uint32_t TSEG2 = bt.phase_seg2;
    const uint32_t TQ_NUM = 1u + TSEG1 + TSEG2;

    const uint32_t FCLK = cans_[ch]->GetClockFreq();

    auto& cfg = config_[ch];
    cfg.bit_timing.brp = bt.brp;
    cfg.bit_timing.prop_seg = bt.prop_seg;
    cfg.bit_timing.phase_seg1 = bt.phase_seg1;
    cfg.bit_timing.phase_seg2 = bt.phase_seg2;
    cfg.bit_timing.sjw = bt.sjw;

    if (bt.brp != 0u && TQ_NUM != 0u)
    {
      cfg.bitrate = FCLK / (bt.brp * TQ_NUM);
      cfg.sample_point = static_cast<float>(1u + TSEG1) / static_cast<float>(TQ_NUM);
    }
    else
    {
      cfg.bitrate = 0;
      cfg.sample_point = 0.0f;
    }

    if (fd_supported_ && fdcans_[ch])
    {
      auto& fd_cfg = fd_config_[ch];
      static_cast<CAN::Configuration&>(fd_cfg) = cfg;
    }

    return cans_[ch]->SetConfig(cfg);
  }

  /**
   * @brief 处理 DATA_BITTIMING（数据相位） / Handle DATA_BITTIMING (data phase)
   */
  ErrorCode HandleDataBitTiming(uint8_t ch, const GsUsb::DeviceBitTiming& bt)
  {
    if (!host_format_ok_)
    {
      return ErrorCode::ARG_ERR;
    }
    if (!fd_supported_ || ch >= can_count_ || !fdcans_[ch])
    {
      return ErrorCode::NOT_SUPPORT;
    }

    const uint32_t TSEG1 = bt.prop_seg + bt.phase_seg1;
    const uint32_t TSEG2 = bt.phase_seg2;
    const uint32_t TQ_NUM = 1u + TSEG1 + TSEG2;

    const uint32_t FCLK = fdcans_[ch]->GetClockFreq();

    auto& fd_cfg = fd_config_[ch];
    static_cast<CAN::Configuration&>(fd_cfg) = config_[ch];

    fd_cfg.data_timing.brp = bt.brp;
    fd_cfg.data_timing.prop_seg = bt.prop_seg;
    fd_cfg.data_timing.phase_seg1 = bt.phase_seg1;
    fd_cfg.data_timing.phase_seg2 = bt.phase_seg2;
    fd_cfg.data_timing.sjw = bt.sjw;

    if (bt.brp != 0u && TQ_NUM != 0u)
    {
      fd_cfg.data_bitrate = FCLK / (bt.brp * TQ_NUM);
      fd_cfg.data_sample_point =
          static_cast<float>(1u + TSEG1) / static_cast<float>(TQ_NUM);
    }
    else
    {
      fd_cfg.data_bitrate = 0;
      fd_cfg.data_sample_point = 0.0f;
    }

    return fdcans_[ch]->SetConfig(fd_cfg);
  }

  /**
   * @brief 处理 MODE（启动/复位 + 标志位） / Handle MODE (start/reset + flags)
   */
  ErrorCode HandleMode(uint8_t ch, const GsUsb::DeviceMode& mode)
  {
    if (!host_format_ok_ || ch >= can_count_ || !cans_[ch])
    {
      return ErrorCode::ARG_ERR;
    }

    switch (static_cast<GsUsb::CanMode>(mode.mode))
    {
      case GsUsb::CanMode::RESET:
        can_enabled_[ch] = false;
        fd_enabled_[ch] = false;
        break;

      case GsUsb::CanMode::START:
        can_enabled_[ch] = true;
        if (fd_supported_ && fdcans_[ch] && (mode.flags & GsUsb::GSCAN_MODE_FD))
        {
          fd_enabled_[ch] = true;
        }
        break;

      default:
        return ErrorCode::ARG_ERR;
    }

    auto& cfg = config_[ch];
    cfg.mode.loopback = (mode.flags & GsUsb::GSCAN_MODE_LOOP_BACK) != 0;
    cfg.mode.listen_only = (mode.flags & GsUsb::GSCAN_MODE_LISTEN_ONLY) != 0;
    cfg.mode.triple_sampling = (mode.flags & GsUsb::GSCAN_MODE_TRIPLE_SAMPLE) != 0;
    cfg.mode.one_shot = (mode.flags & GsUsb::GSCAN_MODE_ONE_SHOT) != 0;

    timestamps_enabled_ch_[ch] = (mode.flags & GsUsb::GSCAN_MODE_HW_TIMESTAMP) != 0;
    berr_enabled_[ch] = (mode.flags & GsUsb::GSCAN_MODE_BERR_REPORTING) != 0;

    const ErrorCode EC = cans_[ch]->SetConfig(cfg);

    if (fd_supported_ && fdcans_[ch])
    {
      auto& fd_cfg = fd_config_[ch];
      static_cast<CAN::Configuration&>(fd_cfg) = cfg;
      fd_cfg.fd_mode.fd_enabled = (mode.flags & GsUsb::GSCAN_MODE_FD) != 0;
      (void)fdcans_[ch]->SetConfig(fd_cfg);
    }

    return EC;
  }

  /**
   * @brief 处理 BERR 开关 / Handle BERR enable/disable
   */
  ErrorCode HandleBerr(uint8_t ch, uint32_t berr_on)
  {
    if (ch >= can_count_)
    {
      return ErrorCode::ARG_ERR;
    }
    berr_enabled_[ch] = (berr_on != 0);
    return ErrorCode::OK;
  }

  /**
   * @brief 处理 IDENTIFY / Handle IDENTIFY
   */
  ErrorCode HandleIdentify(uint8_t, const GsUsb::Identify& id)
  {
    const bool ON = (id.mode == static_cast<uint32_t>(GsUsb::IdentifyMode::ON));
    if (identify_gpio_)
    {
      (void)identify_gpio_->Write(ON);
    }
    return ErrorCode::OK;
  }

  /**
   * @brief 处理 SET_TERMINATION / Handle SET_TERMINATION
   */
  ErrorCode HandleSetTermination(uint8_t ch, const GsUsb::DeviceTerminationState& st)
  {
    if (ch >= can_count_)
    {
      return ErrorCode::ARG_ERR;
    }

    term_state_[ch] = static_cast<GsUsb::TerminationState>(
        st.state != 0 ? static_cast<uint32_t>(GsUsb::TerminationState::ON)
                      : static_cast<uint32_t>(GsUsb::TerminationState::OFF));

    if (termination_gpios_[ch])
    {
      const bool ON = (term_state_[ch] == GsUsb::TerminationState::ON);
      (void)termination_gpios_[ch]->Write(ON);
    }

    return ErrorCode::OK;
  }

  /**
   * @brief wire -> ClassicPack / Convert wire frame to ClassicPack
   */
  static void HostWireToClassicPack(const WireHeader& wh, const uint8_t* payload,
                                    LibXR::CAN::ClassicPack& pack)
  {
    const uint32_t CID = wh.can_id;
    const bool IS_EFF = (CID & GsUsb::CAN_EFF_FLAG) != 0;
    const bool IS_RTR = (CID & GsUsb::CAN_RTR_FLAG) != 0;

    if (IS_EFF)
    {
      pack.id = CID & GsUsb::CAN_EFF_MASK;
      pack.type = IS_RTR ? LibXR::CAN::Type::REMOTE_EXTENDED : LibXR::CAN::Type::EXTENDED;
    }
    else
    {
      pack.id = CID & GsUsb::CAN_SFF_MASK;
      pack.type = IS_RTR ? LibXR::CAN::Type::REMOTE_STANDARD : LibXR::CAN::Type::STANDARD;
    }

    uint8_t dlc = wh.can_dlc;
    if (dlc > 8u)
    {
      dlc = 8u;
    }
    pack.dlc = dlc;

    if (dlc > 0u)
    {
      Memory::FastCopy(pack.data, payload, dlc);
    }
  }

  /**
   * @brief wire -> FDPack / Convert wire frame to FDPack
   */
  static void HostWireToFdPack(const WireHeader& wh, const uint8_t* payload,
                               LibXR::FDCAN::FDPack& pack)
  {
    const uint32_t CID = wh.can_id;
    const bool IS_EFF = (CID & GsUsb::CAN_EFF_FLAG) != 0;
    const bool IS_RTR = (CID & GsUsb::CAN_RTR_FLAG) != 0;

    if (IS_EFF)
    {
      pack.id = CID & GsUsb::CAN_EFF_MASK;
      pack.type = IS_RTR ? LibXR::CAN::Type::REMOTE_EXTENDED : LibXR::CAN::Type::EXTENDED;
    }
    else
    {
      pack.id = CID & GsUsb::CAN_SFF_MASK;
      pack.type = IS_RTR ? LibXR::CAN::Type::REMOTE_STANDARD : LibXR::CAN::Type::STANDARD;
    }

    uint8_t len = DlcToLen(wh.can_dlc);
    if (len > 64)
    {
      len = 64;
    }
    pack.len = len;

    if (len > 0u)
    {
      Memory::FastCopy(pack.data, payload, len);
    }
  }

  /**
   * @brief ClassicPack -> QueueItem / Convert ClassicPack to QueueItem
   */
  void ClassicPackToQueueItem(const LibXR::CAN::ClassicPack& pack, uint8_t ch,
                              QueueItem& qi)
  {
    uint32_t cid = 0;
    switch (pack.type)
    {
      case LibXR::CAN::Type::STANDARD:
        cid = (pack.id & GsUsb::CAN_SFF_MASK);
        break;
      case LibXR::CAN::Type::EXTENDED:
        cid = (pack.id & GsUsb::CAN_EFF_MASK) | GsUsb::CAN_EFF_FLAG;
        break;
      case LibXR::CAN::Type::REMOTE_STANDARD:
        cid = (pack.id & GsUsb::CAN_SFF_MASK) | GsUsb::CAN_RTR_FLAG;
        break;
      case LibXR::CAN::Type::REMOTE_EXTENDED:
        cid = (pack.id & GsUsb::CAN_EFF_MASK) | GsUsb::CAN_EFF_FLAG | GsUsb::CAN_RTR_FLAG;
        break;
      default:
        cid = pack.id & GsUsb::CAN_SFF_MASK;
        break;
    }

    qi.hdr.echo_id = ECHO_ID_RX;
    qi.hdr.can_id = cid;
    qi.hdr.can_dlc = (pack.dlc <= 8u) ? pack.dlc : 8u;
    qi.hdr.channel = ch;
    qi.hdr.flags = 0;
    qi.hdr.reserved = 0;

    qi.is_fd = false;
    qi.data_len = 8;
    qi.timestamp_us = MakeTimestampUs(ch);

    Memory::FastSet(qi.data.data(), 0, 8);
    if (qi.hdr.can_dlc > 0u)
    {
      Memory::FastCopy(qi.data.data(), pack.data, qi.hdr.can_dlc);
    }
  }

  /**
   * @brief FDPack -> QueueItem / Convert FDPack to QueueItem
   */
  void FdPackToQueueItem(const LibXR::FDCAN::FDPack& pack, uint8_t ch, QueueItem& qi)
  {
    uint32_t cid = 0;
    switch (pack.type)
    {
      case LibXR::CAN::Type::STANDARD:
        cid = (pack.id & GsUsb::CAN_SFF_MASK);
        break;
      case LibXR::CAN::Type::EXTENDED:
        cid = (pack.id & GsUsb::CAN_EFF_MASK) | GsUsb::CAN_EFF_FLAG;
        break;
      case LibXR::CAN::Type::REMOTE_STANDARD:
        cid = (pack.id & GsUsb::CAN_SFF_MASK) | GsUsb::CAN_RTR_FLAG;
        break;
      case LibXR::CAN::Type::REMOTE_EXTENDED:
        cid = (pack.id & GsUsb::CAN_EFF_MASK) | GsUsb::CAN_EFF_FLAG | GsUsb::CAN_RTR_FLAG;
        break;
      default:
        cid = pack.id & GsUsb::CAN_SFF_MASK;
        break;
    }

    qi.hdr.echo_id = ECHO_ID_RX;
    qi.hdr.can_id = cid;
    qi.hdr.can_dlc = LenToDlc(pack.len);
    qi.hdr.channel = ch;
    qi.hdr.flags = GsUsb::CAN_FLAG_FD;
    qi.hdr.reserved = 0;

    qi.is_fd = true;
    qi.data_len = 64;
    qi.timestamp_us = MakeTimestampUs(ch);

    const uint8_t LEN = (pack.len <= 64) ? pack.len : 64;

    if (LEN > 0u)
    {
      Memory::FastCopy(qi.data.data(), pack.data, LEN);
    }
    if (LEN < 64u)
    {
      Memory::FastSet(qi.data.data() + LEN, 0, 64u - LEN);
    }
  }

  /**
   * @brief QueueItem -> wire bytes / Convert QueueItem to wire bytes
   * @param qi 队列元素 / Queue item
   * @param out 输出缓冲区 / Output buffer
   * @param cap 输出缓冲容量 / Output buffer capacity
   * @return std::size_t 实际写入长度（0 表示失败） / Bytes written (0 means failed)
   */
  std::size_t PackQueueItemToWire(const QueueItem& qi, uint8_t* out,
                                  std::size_t cap) const
  {
    if (cap < WIRE_HDR_SIZE)
    {
      return 0;
    }

    const uint8_t CH = qi.hdr.channel;
    const bool TS = (CH < can_count_) ? timestamps_enabled_ch_[CH] : false;
    const std::size_t PAYLOAD = qi.is_fd ? WIRE_FD_DATA_SIZE : WIRE_CLASSIC_DATA_SIZE;
    const std::size_t TOTAL = WIRE_HDR_SIZE + PAYLOAD + (TS ? WIRE_TS_SIZE : 0);

    if (TOTAL > cap)
    {
      return 0;
    }

    Memory::FastCopy(out, &qi.hdr, WIRE_HDR_SIZE);
    Memory::FastCopy(out + WIRE_HDR_SIZE, qi.data.data(), PAYLOAD);

    if (TS)
    {
      Memory::FastCopy(out + WIRE_HDR_SIZE + PAYLOAD, &qi.timestamp_us, WIRE_TS_SIZE);
    }

    return TOTAL;
  }

  /**
   * @brief 将 LibXR 错误帧转换为 Host 错误帧（SocketCAN 语义） /
   *        Convert LibXR error pack to host error frame (SocketCAN semantics)
   * @param ch 通道号 / Channel index
   * @param err_pack 错误帧 / Error pack
   * @param qi 输出队列元素 / Output queue item
   * @return true 转换成功并应上报 / Converted and should report
   * @return false 不上报 / Do not report
   */
  bool ErrorPackToHostErrorFrame(uint8_t ch, const LibXR::CAN::ClassicPack& err_pack,
                                 QueueItem& qi)
  {
    if (ch >= can_count_ || !cans_[ch])
    {
      return false;
    }
    if (!berr_enabled_[ch])
    {
      return false;
    }
    if (!LibXR::CAN::IsErrorId(err_pack.id))
    {
      return false;
    }

    // 与 Linux can/error.h 对齐（沿用之前版本的映射）。
    // Keep these definitions aligned with Linux can/error.h, matching the previous
    // mapping.
    constexpr uint8_t LNX_CAN_ERR_CRTL_UNSPEC = 0x00;
    constexpr uint8_t LNX_CAN_ERR_CRTL_RX_WARNING = 0x04;
    constexpr uint8_t LNX_CAN_ERR_CRTL_TX_WARNING = 0x08;
    constexpr uint8_t LNX_CAN_ERR_CRTL_RX_PASSIVE = 0x10;
    constexpr uint8_t LNX_CAN_ERR_CRTL_TX_PASSIVE = 0x20;

    constexpr uint8_t LNX_CAN_ERR_PROT_UNSPEC = 0x00;
    constexpr uint8_t LNX_CAN_ERR_PROT_FORM = 0x02;
    constexpr uint8_t LNX_CAN_ERR_PROT_STUFF = 0x04;
    constexpr uint8_t LNX_CAN_ERR_PROT_BIT0 = 0x08;
    constexpr uint8_t LNX_CAN_ERR_PROT_BIT1 = 0x10;
    constexpr uint8_t LNX_CAN_ERR_PROT_TX = 0x80;

    constexpr uint8_t LNX_CAN_ERR_PROT_LOC_UNSPEC = 0x00;
    constexpr uint8_t LNX_CAN_ERR_PROT_LOC_ACK = 0x19;

    constexpr uint32_t LNX_CAN_ERR_CNT = 0x00000200U;

    bool ec_valid = false;
    uint32_t txerr = 0, rxerr = 0;
    {
      LibXR::CAN::ErrorState es;
      if (cans_[ch]->GetErrorState(es) == ErrorCode::OK)
      {
        ec_valid = true;
        txerr = es.tx_error_counter;
        rxerr = es.rx_error_counter;
      }
    }

    const uint8_t TXERR_U8 = (txerr > 255U) ? 255U : static_cast<uint8_t>(txerr);
    const uint8_t RXERR_U8 = (rxerr > 255U) ? 255U : static_cast<uint8_t>(rxerr);

    const auto EID = LibXR::CAN::ToErrorID(err_pack.id);

    uint32_t cid = GsUsb::CAN_ERR_FLAG;
    std::array<uint8_t, 8> d{};
    switch (EID)
    {
      case LibXR::CAN::ErrorID::CAN_ERROR_ID_BUS_OFF:
        cid |= GsUsb::CAN_ERR_BUSOFF;
        break;

      case LibXR::CAN::ErrorID::CAN_ERROR_ID_ERROR_WARNING:
      case LibXR::CAN::ErrorID::CAN_ERROR_ID_ERROR_PASSIVE:
      {
        cid |= GsUsb::CAN_ERR_CRTL;
        uint8_t ctrl = LNX_CAN_ERR_CRTL_UNSPEC;

        if (ec_valid)
        {
          if (EID == LibXR::CAN::ErrorID::CAN_ERROR_ID_ERROR_PASSIVE)
          {
            if (txerr >= 128U)
            {
              ctrl |= LNX_CAN_ERR_CRTL_TX_PASSIVE;
            }
            if (rxerr >= 128U)
            {
              ctrl |= LNX_CAN_ERR_CRTL_RX_PASSIVE;
            }
          }
          else
          {
            if (txerr >= 96U)
            {
              ctrl |= LNX_CAN_ERR_CRTL_TX_WARNING;
            }
            if (rxerr >= 96U)
            {
              ctrl |= LNX_CAN_ERR_CRTL_RX_WARNING;
            }
          }
        }

        if (ctrl == LNX_CAN_ERR_CRTL_UNSPEC)
        {
          ctrl = (EID == LibXR::CAN::ErrorID::CAN_ERROR_ID_ERROR_PASSIVE)
                     ? static_cast<uint8_t>(LNX_CAN_ERR_CRTL_TX_PASSIVE |
                                            LNX_CAN_ERR_CRTL_RX_PASSIVE)
                     : static_cast<uint8_t>(LNX_CAN_ERR_CRTL_TX_WARNING |
                                            LNX_CAN_ERR_CRTL_RX_WARNING);
        }

        d[1] = ctrl;
        break;
      }

      case LibXR::CAN::ErrorID::CAN_ERROR_ID_ACK:
        cid |= GsUsb::CAN_ERR_ACK;
        cid |= GsUsb::CAN_ERR_PROT;
        d[2] = static_cast<uint8_t>(LNX_CAN_ERR_PROT_UNSPEC | LNX_CAN_ERR_PROT_TX);
        d[3] = LNX_CAN_ERR_PROT_LOC_ACK;
        break;

      case LibXR::CAN::ErrorID::CAN_ERROR_ID_STUFF:
        cid |= GsUsb::CAN_ERR_PROT;
        d[2] = static_cast<uint8_t>(LNX_CAN_ERR_PROT_STUFF | LNX_CAN_ERR_PROT_TX);
        d[3] = LNX_CAN_ERR_PROT_LOC_UNSPEC;
        break;

      case LibXR::CAN::ErrorID::CAN_ERROR_ID_FORM:
        cid |= GsUsb::CAN_ERR_PROT;
        d[2] = static_cast<uint8_t>(LNX_CAN_ERR_PROT_FORM | LNX_CAN_ERR_PROT_TX);
        d[3] = LNX_CAN_ERR_PROT_LOC_UNSPEC;
        break;

      case LibXR::CAN::ErrorID::CAN_ERROR_ID_BIT0:
        cid |= GsUsb::CAN_ERR_PROT;
        d[2] = static_cast<uint8_t>(LNX_CAN_ERR_PROT_BIT0 | LNX_CAN_ERR_PROT_TX);
        d[3] = LNX_CAN_ERR_PROT_LOC_UNSPEC;
        break;

      case LibXR::CAN::ErrorID::CAN_ERROR_ID_BIT1:
        cid |= GsUsb::CAN_ERR_PROT;
        d[2] = static_cast<uint8_t>(LNX_CAN_ERR_PROT_BIT1 | LNX_CAN_ERR_PROT_TX);
        d[3] = LNX_CAN_ERR_PROT_LOC_UNSPEC;
        break;

      default:
        cid |= GsUsb::CAN_ERR_PROT;
        d[2] = static_cast<uint8_t>(LNX_CAN_ERR_PROT_UNSPEC | LNX_CAN_ERR_PROT_TX);
        d[3] = LNX_CAN_ERR_PROT_LOC_UNSPEC;
        break;
    }

    cid |= LNX_CAN_ERR_CNT;
    d[6] = TXERR_U8;
    d[7] = RXERR_U8;

    qi.hdr.echo_id = ECHO_ID_RX;
    qi.hdr.can_id = cid;
    qi.hdr.can_dlc = GsUsb::CAN_ERR_DLC;
    qi.hdr.channel = ch;
    qi.hdr.flags = 0;
    qi.hdr.reserved = 0;

    qi.is_fd = false;
    qi.data_len = 8;
    qi.timestamp_us = MakeTimestampUs(ch);

    Memory::FastCopy(qi.data.data(), d.data(), 8);

    return true;
  }

  // ================= 工具函数 / Utilities =================

  /**
   * @brief 获取通道时间戳（us, 32-bit） / Get per-channel timestamp (us, 32-bit)
   * @param ch 通道号 / Channel index
   * @return uint32_t 时间戳 / Timestamp
   */
  uint32_t MakeTimestampUs(uint8_t ch) const
  {
    if (ch < can_count_ && timestamps_enabled_ch_[ch])
    {
      return static_cast<uint32_t>(LibXR::Timebase::GetMicroseconds() & 0xFFFFFFFFu);
    }
    return 0u;
  }

  /**
   * @brief 获取全局时间戳（us, 32-bit） / Get global timestamp (us, 32-bit)
   * @return uint32_t 时间戳 / Timestamp
   */
  uint32_t MakeTimestampUsGlobal() const
  {
    return static_cast<uint32_t>(LibXR::Timebase::GetMicroseconds() & 0xFFFFFFFFu);
  }

  /**
   * @brief DLC 转长度（FD 表） / DLC to length (FD table)
   */
  static uint8_t DlcToLen(uint8_t dlc)
  {
    static constexpr uint8_t TABLE[16] = {0, 1,  2,  3,  4,  5,  6,  7,
                                          8, 12, 16, 20, 24, 32, 48, 64};
    return (dlc < 16) ? TABLE[dlc] : 64;
  }

  /**
   * @brief 长度转 DLC（FD 表） / Length to DLC (FD table)
   */
  static uint8_t LenToDlc(uint8_t len)
  {
    if (len <= 8)
    {
      return len;
    }
    if (len <= 12)
    {
      return 9;
    }
    if (len <= 16)
    {
      return 10;
    }
    if (len <= 20)
    {
      return 11;
    }
    if (len <= 24)
    {
      return 12;
    }
    if (len <= 32)
    {
      return 13;
    }
    if (len <= 48)
    {
      return 14;
    }
    return 15;
  }

  /**
   * @brief 读取 USER_ID（RAM 或 Database） / Read USER_ID (RAM or Database)
   * @return uint32_t USER_ID
   */
  uint32_t GetUserIdFromStorage() const
  {
    if (database_ == nullptr)
    {
      return user_id_ram_;
    }

    LibXR::Database::Key<uint32_t> key(*database_, "user_id", user_id_ram_);
    return static_cast<uint32_t>(key);
  }

  /**
   * @brief 写入 USER_ID（RAM 或 Database） / Write USER_ID (RAM or Database)
   * @param value USER_ID
   */
  void SetUserIdToStorage(uint32_t value)
  {
    user_id_ram_ = value;

    if (database_ == nullptr)
    {
      return;
    }

    LibXR::Database::Key<uint32_t> key(*database_, "user_id", 0u);
    (void)key.Set(value);
  }
};

}  // namespace LibXR::USB
