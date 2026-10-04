#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "daplink_v2_def.hpp"
#include "debug/jtag_dp.hpp"
#include "debug/swd.hpp"
#include "dev_core.hpp"
#include "gpio.hpp"
#include "libxr_def.hpp"
#include "libxr_mem.hpp"
#include "libxr_type.hpp"
#include "timebase.hpp"
#include "usb/core/desc_cfg.hpp"
#include "winusb_msos20.hpp"

namespace LibXR::USB
{
#ifndef LIBXR_USB_DAPLINK_V2_BUILD_CONFIG_DEFINED
#define LIBXR_USB_DAPLINK_V2_BUILD_CONFIG_DEFINED
namespace DapLinkV2BuildConfig
{
static constexpr bool kEnableJtag = true;
}
#endif

/**
 * @brief CMSIS-DAP v2（Bulk）USB 类，支持 SWD 和 JTAG（daplink_v2_profile_swd.hpp
 *        可关闭 JTAG），可选 nRESET 控制 / CMSIS-DAP v2 (Bulk) USB class with SWD and
 *        JTAG (JTAG can be disabled with daplink_v2_profile_swd.hpp), optional
 *        nRESET control.
 *
 *
 * @tparam SwdPort SWD link type
 */
template <typename SwdPort, uint16_t DefaultDapPacketSize = 512u,
          uint8_t AdvertisedPacketCount = 8u, uint16_t MaxDapPacketSize = 1024u,
          uint16_t QueuedRequestBufferSize = 2048u, uint16_t QueuedCommandCountMax = 255u>
class DapLinkV2Class : public DeviceClass
{
 public:
  static constexpr bool JTAG_ENABLED = DapLinkV2BuildConfig::kEnableJtag;
  static constexpr const char* DEFAULT_INTERFACE_STRING = "CMSIS-DAP v2";

  /**
   * @brief Info string set
   */
  struct InfoStrings
  {
    const char* vendor = nullptr;        ///< Vendor string
    const char* product = nullptr;       ///< Product string
    const char* serial = nullptr;        ///< Serial string
    const char* firmware_ver = nullptr;  ///< Firmware version

    const char* device_vendor = nullptr;   ///< Device vendor
    const char* device_name = nullptr;     ///< Device name
    const char* board_vendor = nullptr;    ///< Board vendor
    const char* board_name = nullptr;      ///< Board name
    const char* product_fw_ver = nullptr;  ///< Product FW version
  };

 public:
  /**
   * @brief Constructor
   *
   * @param data_in_ep_num  Bulk IN endpoint number (required)
   * @param data_out_ep_num Bulk OUT endpoint number (required)
   * @param swd_link        SWD link reference
   * @param nreset_gpio     Optional nRESET GPIO
   *
   */
  explicit DapLinkV2Class(Endpoint::EPNumber data_in_ep_num,
                          Endpoint::EPNumber data_out_ep_num, SwdPort& swd_link,
                          LibXR::GPIO* nreset_gpio = nullptr,
                          const char* interface_string = DEFAULT_INTERFACE_STRING)
      : DeviceClass(),
        swd_(swd_link),
        nreset_gpio_(nreset_gpio),
        interface_string_(interface_string),
        data_in_ep_num_(data_in_ep_num),
        data_out_ep_num_(data_out_ep_num)
  {
    (void)swd_.SetClockHz(swj_clock_hz_);

    // Init WinUSB descriptor templates (constant parts)
    InitWinUsbDescriptors();
  }

  /**
   * @brief Virtual destructor
   */
  ~DapLinkV2Class() override = default;

  DapLinkV2Class(const DapLinkV2Class&) = delete;
  DapLinkV2Class& operator=(const DapLinkV2Class&) = delete;

 public:
  const char* GetInterfaceString(size_t local_interface_index) const override
  {
    return (local_interface_index == 0u) ? interface_string_ : nullptr;
  }

  /**
   * @brief Set info strings
   * @param info Info string set
   */
  void SetInfoStrings(const InfoStrings& info) { info_ = info; }

  void SetJtag(LibXR::Debug::Jtag* jtag)
  {
    if constexpr (JTAG_ENABLED)
    {
      jtag_ = jtag;
      jtag_chain_.ir_length = jtag_ir_len_;
      ResetJtagChainState();
      if (jtag_ != nullptr)
      {
        (void)jtag_->SetClockHz(swj_clock_hz_);
      }
    }
    else
    {
      (void)jtag;
    }
  }

  /**
   * @brief Get internal state
   * @return State reference
   */
  const LibXR::USB::DapLinkV2Def::State& GetState() const { return dap_state_; }

  /**
   * @brief Whether initialized
   * @return true if initialized
   */
  bool IsInited() const { return inited_; }

  /**
   * @brief Whether IN endpoint is busy
   *
   * @return true
   * @return false
   */
  bool EpInBusy()
  {
    auto ep = ep_data_in_;
    return ep->GetState() != Endpoint::State::IDLE;
  }

  /**
   * @brief Whether OUT endpoint is busy
   *
   * @return true
   * @return false
   */
  bool EpOutBusy()
  {
    auto ep = ep_data_out_;
    return ep->GetState() != Endpoint::State::IDLE;
  }

 protected:
  /**
   * @brief Bind endpoint resources
   * @param endpoint_pool Endpoint pool
   * @param start_itf_num Start interface number
   * @param in_isr Whether called from ISR context
   */
  void BindEndpoints(EndpointPool& endpoint_pool, uint8_t start_itf_num, bool) override
  {
    inited_ = false;

    interface_num_ = start_itf_num;

    // Patch WinUSB function subset to match this interface number
    UpdateWinUsbInterfaceFields();

    // Allocate endpoints
    auto ans =
        endpoint_pool.Get(ep_data_out_, Endpoint::Direction::OUT, data_out_ep_num_);
    ASSERT(ans == ErrorCode::OK);

    ans = endpoint_pool.Get(ep_data_in_, Endpoint::Direction::IN, data_in_ep_num_);
    ASSERT(ans == ErrorCode::OK);

    // Configure endpoints
    // - Use upper bound; core will choose a valid max packet size <= this limit.
    // - Enable double_buffer for USB pipeline overlap.
    ep_data_out_->Configure(
        {Endpoint::Direction::OUT, Endpoint::Type::BULK, UINT16_MAX, true});
    ep_data_in_->Configure(
        {Endpoint::Direction::IN, Endpoint::Type::BULK, UINT16_MAX, true});

    // Hook callbacks
    ep_data_out_->SetOnTransferCompleteCallback(on_data_out_cb_);
    ep_data_in_->SetOnTransferCompleteCallback(on_data_in_cb_);

    // Interface descriptor (vendor specific, 2 endpoints)
    desc_block_.intf = {9,
                        static_cast<uint8_t>(DescriptorType::INTERFACE),
                        interface_num_,
                        0,
                        2,
                        0xFF,  // vendor specific
                        0x00,
                        0x00,
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

    // Runtime defaults
    dap_state_ = {};
    dap_state_.debug_port = LibXR::USB::DapLinkV2Def::DebugPort::DISABLED;
    dap_state_.transfer_abort = false;

    swj_clock_hz_ = 1'000'000u;
    (void)swd_.SetClockHz(swj_clock_hz_);
    if constexpr (JTAG_ENABLED)
    {
      if (jtag_ != nullptr)
      {
        (void)jtag_->SetClockHz(swj_clock_hz_);
      }
    }

    // SWJ shadow defaults: SWDIO=1, nRESET=1, SWCLK=0
    last_nreset_level_high_ = true;
    swj_shadow_ = static_cast<uint8_t>(DapLinkV2Def::DAP_SWJ_SWDIO_TMS |
                                       DapLinkV2Def::DAP_SWJ_NRESET);

    ResetResponseQueue();
    ResetQueuedCommandState();
    ResetJtagChainState();
    ep_data_in_->SetActiveLength(0);

    inited_ = true;
    ArmOutTransferIfIdle();
  }

  /**
   * @brief Unbind endpoint resources
   * @param endpoint_pool Endpoint pool
   * @param in_isr Whether called from ISR context
   */
  void UnbindEndpoints(EndpointPool& endpoint_pool, bool) override
  {
    inited_ = false;
    ResetResponseQueue();
    ResetQueuedCommandState();

    dap_state_.debug_port = LibXR::USB::DapLinkV2Def::DebugPort::DISABLED;
    dap_state_.transfer_abort = false;

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

    swd_.Close();
    if constexpr (JTAG_ENABLED)
    {
      if (jtag_ != nullptr)
      {
        jtag_->Close();
      }
    }

    // Reset shadow defaults
    last_nreset_level_high_ = true;
    swj_shadow_ = static_cast<uint8_t>(DapLinkV2Def::DAP_SWJ_SWDIO_TMS |
                                       DapLinkV2Def::DAP_SWJ_NRESET);
  }

  /**
   * @brief Number of interfaces contributed
   * @return Interface count
   */
  size_t GetInterfaceCount() override { return 1; }

  /**
   * @brief Whether an IAD is used
   * @return true if this class uses an IAD
   */
  bool HasIAD() override { return false; }

  /**
   * @brief Endpoint ownership check
   * @param ep_addr Endpoint address
   * @return true if owned by this class
   */
  bool OwnsEndpoint(uint8_t ep_addr) const override
  {
    if (!inited_)
    {
      return false;
    }
    return (ep_data_in_ && ep_addr == ep_data_in_->GetAddress()) ||
           (ep_data_out_ && ep_addr == ep_data_out_->GetAddress());
  }

  /**
   * @brief Maximum bytes required in configuration descriptor
   * @return Maximum bytes
   */
  size_t GetMaxConfigSize() override { return sizeof(desc_block_); }

  /**
   * @brief Get WinUSB MS OS 2.0 descriptor set
   * @return ConstRawData descriptor set bytes
   */
  ConstRawData GetWinUsbMsOs20DescriptorSet() const
  {
    return ConstRawData{reinterpret_cast<const uint8_t*>(&winusb_msos20_),
                        sizeof(winusb_msos20_)};
  }

  /**
   * @brief BOS capability count
   * @return Capability count
   */
  size_t GetBosCapabilityCount() override { return 1; }

  /**
   * @brief Get BOS capability
   * @param index Capability index
   * @return Capability pointer
   */
  BosCapability* GetBosCapability(size_t index) override
  {
    if (index == 0)
    {
      return &winusb_msos20_cap_;
    }
    return nullptr;
  }

 private:
  // ============================================================================
  // Local helpers (kept with original comments, moved inside the class)
  // ============================================================================

  // Cast enum/integer to uint8_t.
  template <typename E>
  static constexpr uint8_t ToU8(E e)
  {
    return static_cast<uint8_t>(e);
  }

  // CMSIS-DAP status bytes
  static constexpr uint8_t DAP_OK = 0x00u;     ///< DAP_OK
  static constexpr uint8_t DAP_ERROR = 0xFFu;  ///< DAP_ERROR

  // Unknown command response: single byte 0xFF.
  static inline ErrorCode BuildUnknownCmdResponse(uint8_t* resp, uint16_t cap,
                                                  uint16_t& out_len)
  {
    if (!resp || cap < 1u)
    {
      out_len = 0u;
      return ErrorCode::NOT_FOUND;
    }
    resp[0] = 0xFFu;
    out_len = 1u;
    return ErrorCode::OK;
  }

  // Command status response: <cmd, status>.
  static inline ErrorCode BuildCmdStatusResponse(uint8_t cmd, uint8_t status,
                                                 uint8_t* resp, uint16_t cap,
                                                 uint16_t& out_len)
  {
    if (!resp || cap < 2u)
    {
      out_len = 0u;
      return ErrorCode::NOT_FOUND;
    }
    resp[0] = cmd;
    resp[1] = status;
    out_len = 2u;
    return ErrorCode::OK;
  }

 private:
  // ============================================================================
  // WinUSB descriptors
  // ============================================================================

  void InitWinUsbDescriptors()
  {
    winusb_msos20_.set.wLength = static_cast<uint16_t>(sizeof(winusb_msos20_.set));
    winusb_msos20_.set.wDescriptorType =
        LibXR::USB::WinUsbMsOs20::MS_OS_20_SET_HEADER_DESCRIPTOR;
    winusb_msos20_.set.dwWindowsVersion = 0x06030000;  // Win 8.1+
    winusb_msos20_.set.wTotalLength = static_cast<uint16_t>(sizeof(winusb_msos20_));

    winusb_msos20_.cfg.wLength = static_cast<uint16_t>(sizeof(winusb_msos20_.cfg));
    winusb_msos20_.cfg.wDescriptorType =
        LibXR::USB::WinUsbMsOs20::MS_OS_20_SUBSET_HEADER_CONFIGURATION;

    // MS OS 2.0 configuration subset uses a zero-based configuration index.
    winusb_msos20_.cfg.bConfigurationValue = 0u;
    winusb_msos20_.cfg.bReserved = 0;
    winusb_msos20_.cfg.wTotalLength = static_cast<uint16_t>(
        sizeof(winusb_msos20_) - offsetof(WinUsbMsOs20DescSet, cfg));

    winusb_msos20_.func.wLength = static_cast<uint16_t>(sizeof(winusb_msos20_.func));
    winusb_msos20_.func.wDescriptorType =
        LibXR::USB::WinUsbMsOs20::MS_OS_20_SUBSET_HEADER_FUNCTION;
    winusb_msos20_.func.bReserved = 0;
    winusb_msos20_.func.wTotalLength = static_cast<uint16_t>(
        sizeof(winusb_msos20_) - offsetof(WinUsbMsOs20DescSet, func));

    winusb_msos20_.compat.wLength = static_cast<uint16_t>(sizeof(winusb_msos20_.compat));
    winusb_msos20_.compat.wDescriptorType =
        LibXR::USB::WinUsbMsOs20::MS_OS_20_FEATURE_COMPATIBLE_ID;

    winusb_msos20_.prop.header.wDescriptorType =
        LibXR::USB::WinUsbMsOs20::MS_OS_20_FEATURE_REG_PROPERTY;
    winusb_msos20_.prop.header.wPropertyDataType = LibXR::USB::WinUsbMsOs20::REG_MULTI_SZ;
    winusb_msos20_.prop.header.wPropertyNameLength =
        LibXR::USB::WinUsbMsOs20::PROP_NAME_DEVICE_INTERFACE_GUIDS_BYTES;

    Memory::FastCopy(winusb_msos20_.prop.name,
                     LibXR::USB::WinUsbMsOs20::PROP_NAME_DEVICE_INTERFACE_GUIDS_UTF16,
                     LibXR::USB::WinUsbMsOs20::PROP_NAME_DEVICE_INTERFACE_GUIDS_BYTES);

    // DeviceInterfaceGUIDs: REG_MULTI_SZ UTF-16LE, include double-NUL terminator
    // Single GUID plus double-NUL terminator for REG_MULTI_SZ.
    const char GUID_STR[] = "{CDB3B5AD-293B-4663-AA36-1AAE46463776}";
    const size_t GUID_LEN = sizeof(GUID_STR) - 1;

    for (size_t i = 0; i < GUID_LEN; ++i)
    {
      winusb_msos20_.prop.data[i * 2] = static_cast<uint8_t>(GUID_STR[i]);
      winusb_msos20_.prop.data[i * 2 + 1] = 0x00;
    }

    // Append UTF-16 NUL + extra UTF-16 NUL (REG_MULTI_SZ end)
    winusb_msos20_.prop.data[GUID_LEN * 2 + 0] = 0x00;
    winusb_msos20_.prop.data[GUID_LEN * 2 + 1] = 0x00;
    winusb_msos20_.prop.data[GUID_LEN * 2 + 2] = 0x00;
    winusb_msos20_.prop.data[GUID_LEN * 2 + 3] = 0x00;

    winusb_msos20_.prop.wPropertyDataLength = static_cast<uint16_t>((GUID_LEN * 2) + 4);
    winusb_msos20_.prop.header.wLength =
        static_cast<uint16_t>(sizeof(winusb_msos20_.prop));

    // Sync to BOS capability object
    winusb_msos20_cap_.SetVendorCode(WINUSB_VENDOR_CODE);
    winusb_msos20_cap_.SetDescriptorSet(GetWinUsbMsOs20DescriptorSet());
  }

  void UpdateWinUsbInterfaceFields()
  {
    // Function subset interface number
    winusb_msos20_.func.bFirstInterface = interface_num_;

    // Content changes but total size stays the same; resync for consistency.
    winusb_msos20_cap_.SetDescriptorSet(GetWinUsbMsOs20DescriptorSet());
  }

 private:
  // ============================================================================
  // USB callbacks
  // ============================================================================

  /**
   * @brief OUT complete callback static entry
   */
  static void OnDataOutCompleteStatic(bool in_isr, DapLinkV2Class* self,
                                      LibXR::ConstRawData& data)
  {
    if (self && self->inited_)
    {
      self->OnDataOutComplete(in_isr, data);
    }
  }

  /**
   * @brief IN complete callback static entry
   */
  static void OnDataInCompleteStatic(bool in_isr, DapLinkV2Class* self,
                                     LibXR::ConstRawData& data)
  {
    if (self && self->inited_)
    {
      self->OnDataInComplete(in_isr, data);
    }
  }

  /**
   * @brief OUT complete callback instance method
   */
  void OnDataOutComplete(bool in_isr, LibXR::ConstRawData& data)
  {
    (void)in_isr;

    if (!inited_ || !ep_data_in_ || !ep_data_out_)
    {
      return;
    }

    const auto* req = static_cast<const uint8_t*>(data.addr_);
    const uint16_t REQ_LEN = static_cast<uint16_t>(data.size_);

    // Fast path: when IN is idle and no backlog exists, build directly
    // in the IN endpoint buffer and submit without extra response copy.
    if (!HasDeferredResponseInEpBuffer() && IsResponseQueueEmpty() &&
        ep_data_in_->GetState() == Endpoint::State::IDLE &&
        CanBuildResponseDirectlyInEpBuffer())
    {
      auto tx_buff = ep_data_in_->GetBuffer();
      if (tx_buff.addr_ && tx_buff.size_ > 0u)
      {
        auto* tx_buf = static_cast<uint8_t*>(tx_buff.addr_);
        uint16_t out_len = 0u;
        auto ans = ProcessOneCommand(in_isr, req, REQ_LEN, tx_buf,
                                     static_cast<uint16_t>(tx_buff.size_), out_len);
        UNUSED(ans);

        out_len = ClipResponseLength(out_len, static_cast<uint16_t>(tx_buff.size_));
        if (StartInTransferFromCurrentBuffer(out_len))
        {
          return;
        }

        if (!EnqueueResponse(tx_buf, out_len))
        {
          (void)SubmitNextQueuedResponseIfIdle();
          (void)EnqueueResponse(tx_buf, out_len);
        }

        (void)SubmitNextQueuedResponseIfIdle();
        ArmOutTransferIfIdle();
        return;
      }
    }

    // Deferred fast path: when IN is busy and no backlog exists, build the next
    // response in the alternate IN double buffer to avoid queue copy.
    if (!HasDeferredResponseInEpBuffer() && IsResponseQueueEmpty() &&
        ep_data_in_->GetState() == Endpoint::State::BUSY &&
        CanBuildResponseDirectlyInEpBuffer())
    {
      auto tx_buff = ep_data_in_->GetBuffer();
      if (tx_buff.addr_ && tx_buff.size_ > 0u)
      {
        auto* tx_buf = static_cast<uint8_t*>(tx_buff.addr_);
        uint16_t out_len = 0u;
        auto ans = ProcessOneCommand(in_isr, req, REQ_LEN, tx_buf,
                                     static_cast<uint16_t>(tx_buff.size_), out_len);
        UNUSED(ans);

        out_len = ClipResponseLength(out_len, static_cast<uint16_t>(tx_buff.size_));
        SetDeferredResponseInEpBuffer(out_len);
        ArmOutTransferIfIdle();
        return;
      }
    }

    if (TryBuildAndEnqueueResponse(in_isr, req, REQ_LEN))
    {
      (void)SubmitDeferredResponseIfIdle();
      (void)SubmitNextQueuedResponseIfIdle();
      ArmOutTransferIfIdle();
      return;
    }

    // Should not happen with proper backpressure; drain once and retry queue build.
    (void)SubmitNextQueuedResponseIfIdle();
    (void)TryBuildAndEnqueueResponse(in_isr, req, REQ_LEN);

    (void)SubmitDeferredResponseIfIdle();
    (void)SubmitNextQueuedResponseIfIdle();
    ArmOutTransferIfIdle();
  }

  /**
   * @brief IN complete callback instance method
   */
  void OnDataInComplete(bool /*in_isr*/, LibXR::ConstRawData& /*data*/)
  {
    (void)SubmitDeferredResponseIfIdle();
    (void)SubmitNextQueuedResponseIfIdle();
    ArmOutTransferIfIdle();
  }

 private:
  /**
   * @brief Get effective DAP packet size
   */
  uint16_t GetDapPacketSize() const
  {
    constexpr uint16_t OPENOCD_SAFE_PS = static_cast<uint16_t>((255u * 5u) + 4u);
    constexpr uint16_t RAW_PS =
        (MaxDapPacketSize == 0u) ? DefaultDapPacketSize : MaxDapPacketSize;
    return (RAW_PS > OPENOCD_SAFE_PS) ? OPENOCD_SAFE_PS : RAW_PS;
  }
  bool CanBuildResponseDirectlyInEpBuffer() const
  {
    if (!ep_data_in_)
    {
      return false;
    }
    auto tx_buff = ep_data_in_->GetBuffer();
    if (!tx_buff.addr_ || tx_buff.size_ == 0u)
    {
      return false;
    }
    return GetDapPacketSize() <= static_cast<uint16_t>(tx_buff.size_);
  }

  bool StartInTransferFromCurrentBuffer(uint16_t len)
  {
    if (!ep_data_in_)
    {
      return false;
    }
    auto tx_buff = ep_data_in_->GetBuffer();
    if (!tx_buff.addr_ || tx_buff.size_ == 0u)
    {
      return false;
    }
    uint16_t tx_len = len;
    if (tx_len > tx_buff.size_)
    {
      tx_len = static_cast<uint16_t>(tx_buff.size_);
    }
    if (tx_len <= static_cast<uint16_t>(ep_data_in_->MaxTransferSize()))
    {
      return ep_data_in_->Transfer(tx_len) == ErrorCode::OK;
    }
    LibXR::RawData in_multi{tx_buff.addr_, tx_len};
    return ep_data_in_->TransferMultiBulk(in_multi) == ErrorCode::OK;
  }
  bool StartInTransferFromPayload(const uint8_t* data, uint16_t len)
  {
    if (!ep_data_in_ || !data)
    {
      return false;
    }
    auto tx_buff = ep_data_in_->GetBuffer();
    if (!tx_buff.addr_ || tx_buff.size_ == 0u)
    {
      return false;
    }
    uint16_t tx_len = len;
    if (tx_len > MAX_DAP_PACKET_SIZE)
    {
      tx_len = MAX_DAP_PACKET_SIZE;
    }
    const uint16_t MAX_XFER = static_cast<uint16_t>(ep_data_in_->MaxTransferSize());
    if (tx_len <= MAX_XFER && tx_len <= static_cast<uint16_t>(tx_buff.size_))
    {
      if (tx_len > 0u)
      {
        Memory::FastCopy(tx_buff.addr_, data, tx_len);
      }
      return ep_data_in_->Transfer(tx_len) == ErrorCode::OK;
    }
    if (tx_len > static_cast<uint16_t>(sizeof(in_tx_multi_storage_)))
    {
      tx_len = static_cast<uint16_t>(sizeof(in_tx_multi_storage_));
    }
    if (tx_len > 0u && data != in_tx_multi_storage_)
    {
      Memory::FastCopy(in_tx_multi_storage_, data, tx_len);
    }
    in_tx_multi_buf_.addr_ = in_tx_multi_storage_;
    in_tx_multi_buf_.size_ = tx_len;
    return ep_data_in_->TransferMultiBulk(in_tx_multi_buf_) == ErrorCode::OK;
  }

  /**
   * @brief Clip response length by DAP packet size and buffer capacity
   */
  uint16_t ClipResponseLength(uint16_t len, uint16_t cap) const
  {
    const uint16_t DAP_PS = GetDapPacketSize();
    if (len > DAP_PS)
    {
      len = DAP_PS;
    }
    if (len > RESP_SLOT_SIZE)
    {
      len = RESP_SLOT_SIZE;
    }
    if (len > cap)
    {
      len = cap;
    }
    return len;
  }

  static constexpr uint8_t NextRespQueueIndex(uint8_t idx)
  {
    return static_cast<uint8_t>((idx + 1u) & (RESP_QUEUE_DEPTH - 1u));
  }

  void ResetResponseQueue()
  {
    resp_q_head_ = 0u;
    resp_q_tail_ = 0u;
    resp_q_count_ = 0u;
    deferred_in_resp_valid_ = false;
    deferred_in_resp_len_ = 0u;
  }

  bool HasDeferredResponseInEpBuffer() const { return deferred_in_resp_valid_; }

  void SetDeferredResponseInEpBuffer(uint16_t len)
  {
    deferred_in_resp_len_ = len;
    deferred_in_resp_valid_ = true;
  }

  bool SubmitDeferredResponseIfIdle()
  {
    if (!deferred_in_resp_valid_ || !ep_data_in_ ||
        ep_data_in_->GetState() != Endpoint::State::IDLE)
    {
      return false;
    }

    const uint16_t TX_LEN = deferred_in_resp_len_;
    if (!StartInTransferFromCurrentBuffer(TX_LEN))
    {
      return false;
    }

    deferred_in_resp_valid_ = false;
    deferred_in_resp_len_ = 0u;
    return true;
  }

  bool IsResponseQueueEmpty() const { return resp_q_count_ == 0u; }

  bool IsResponseQueueFull() const { return resp_q_count_ >= RESP_QUEUE_DEPTH; }

  void ResetQueuedCommandState()
  {
    queued_request_length_ = 0u;
    queued_command_count_ = 0u;
  }

  // ============================================================================
  // Queued command helpers
  // ============================================================================

  /**
   * @brief Parse one queued command length from a packed queue stream
   * @param req request pointer
   * @param avail available bytes in the queue stream
   * @param cmd_len parsed command length
   * @return true parse success
   * @return false parse failure
   */
  bool ParseQueuedCommandLength(const uint8_t* req, uint16_t avail,
                                uint16_t& cmd_len) const
  {
    cmd_len = 0u;
    if (!req || avail < 1u)
    {
      return false;
    }

    const uint8_t CMD = req[0];
    switch (CMD)
    {
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::INFO):
        cmd_len = 2u;
        return avail >= cmd_len;
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::HOST_STATUS):
        cmd_len = 3u;
        return avail >= cmd_len;
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::CONNECT):
        cmd_len = 2u;
        return avail >= cmd_len;
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::DISCONNECT):
        cmd_len = 1u;
        return avail >= cmd_len;
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::TRANSFER_CONFIGURE):
        cmd_len = 6u;
        return avail >= cmd_len;
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::TRANSFER_ABORT):
        cmd_len = 1u;
        return avail >= cmd_len;
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::WRITE_ABORT):
        cmd_len = 6u;
        return avail >= cmd_len;
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::DELAY):
        cmd_len = 3u;
        return avail >= cmd_len;
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::RESET_TARGET):
        cmd_len = 1u;
        return avail >= cmd_len;
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::SWJ_PINS):
        cmd_len = 7u;
        return avail >= cmd_len;
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::SWJ_CLOCK):
        cmd_len = 5u;
        return avail >= cmd_len;
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::SWD_CONFIGURE):
        cmd_len = 2u;
        return avail >= cmd_len;
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::SWJ_SEQUENCE):
      {
        if (avail < 2u)
        {
          return false;
        }
        const uint32_t BIT_COUNT = (req[1] == 0u) ? 256u : static_cast<uint32_t>(req[1]);
        const uint32_t BYTE_COUNT = (BIT_COUNT + 7u) / 8u;
        const uint32_t NEED = 2u + BYTE_COUNT;
        if (NEED > avail)
        {
          return false;
        }
        cmd_len = static_cast<uint16_t>(NEED);
        return true;
      }
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::SWD_SEQUENCE):
      {
        if (avail < 2u)
        {
          return false;
        }
        const uint8_t SEQ_CNT = req[1];
        uint32_t off = 2u;
        for (uint32_t i = 0u; i < SEQ_CNT; ++i)
        {
          if (off >= avail)
          {
            return false;
          }

          const uint8_t INFO = req[off++];
          uint32_t cycles = static_cast<uint32_t>(INFO & 0x3Fu);
          if (cycles == 0u)
          {
            cycles = 64u;
          }

          const bool MODE_IN = ((INFO & 0x80u) != 0u);
          const uint32_t BYTES = (cycles + 7u) / 8u;
          if (!MODE_IN)
          {
            if (off + BYTES > avail)
            {
              return false;
            }
            off += BYTES;
          }
        }
        cmd_len = static_cast<uint16_t>(off);
        return true;
      }
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::TRANSFER):
      {
        if (avail < 3u)
        {
          return false;
        }
        const uint8_t COUNT = req[2];
        uint32_t off = 3u;
        for (uint32_t i = 0u; i < COUNT; ++i)
        {
          if (off >= avail)
          {
            return false;
          }

          const uint8_t RQ = req[off++];
          const bool RNW = LibXR::USB::DapLinkV2Def::req_is_read(RQ);
          const bool MATCH_VALUE =
              ((RQ & LibXR::USB::DapLinkV2Def::DAP_TRANSFER_MATCH_VALUE) != 0u);
          if (!RNW || MATCH_VALUE)
          {
            if (off + 4u > avail)
            {
              return false;
            }
            off += 4u;
          }
        }
        cmd_len = static_cast<uint16_t>(off);
        return true;
      }
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::TRANSFER_BLOCK):
      {
        if (avail < 5u)
        {
          return false;
        }
        uint16_t count = 0u;
        Memory::FastCopy(&count, &req[2], sizeof(count));
        const bool RNW = LibXR::USB::DapLinkV2Def::req_is_read(req[4]);
        uint32_t need = 5u;
        if (!RNW)
        {
          need += static_cast<uint32_t>(count) * 4u;
        }
        if (need > avail)
        {
          return false;
        }
        cmd_len = static_cast<uint16_t>(need);
        return true;
      }

      // Reject nested queue/execute in queued stream.
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::QUEUE_COMMANDS):
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::EXECUTE_COMMANDS):
      default:
        return false;
    }
  }

  /**
   * @brief Execute queued commands into one combined EXECUTE_COMMANDS response
   * @param in_isr current context flag
   * @param resp response buffer
   * @param resp_cap response capacity
   * @param out_len output response length
   * @return true execute success
   * @return false execute failure
   */
  bool TryExecuteQueuedCommandStream(bool in_isr, uint8_t* resp, uint16_t resp_cap,
                                     uint16_t& out_len)
  {
    out_len = 0u;
    if (!resp || resp_cap < 1u)
    {
      return false;
    }

    resp[0] = ToU8(LibXR::USB::DapLinkV2Def::CommandId::EXECUTE_COMMANDS);
    out_len = 1u;

    if (queued_command_count_ == 0u)
    {
      return true;
    }

    uint16_t req_off = 0u;
    uint16_t resp_off = 1u;
    for (uint32_t i = 0u; i < queued_command_count_; ++i)
    {
      if (req_off >= queued_request_length_)
      {
        return false;
      }

      uint16_t cmd_len = 0u;
      if (!ParseQueuedCommandLength(
              &queued_request_buffer_[req_off],
              static_cast<uint16_t>(queued_request_length_ - req_off), cmd_len) ||
          cmd_len == 0u)
      {
        return false;
      }

      if (resp_off >= resp_cap)
      {
        return false;
      }

      uint16_t cmd_out = 0u;
      const ErrorCode ans = ProcessOneCommand(
          in_isr, &queued_request_buffer_[req_off], cmd_len, &resp[resp_off],
          static_cast<uint16_t>(resp_cap - resp_off), cmd_out);
      UNUSED(ans);

      if (static_cast<uint32_t>(resp_off) + static_cast<uint32_t>(cmd_out) >
          static_cast<uint32_t>(resp_cap))
      {
        return false;
      }

      req_off = static_cast<uint16_t>(req_off + cmd_len);
      resp_off = static_cast<uint16_t>(resp_off + cmd_out);
    }

    if (req_off != queued_request_length_)
    {
      return false;
    }

    out_len = resp_off;
    return true;
  }

  bool TryBuildAndEnqueueResponse(bool in_isr, const uint8_t* req, uint16_t req_len)
  {
    if (!req || IsResponseQueueFull())
    {
      return false;
    }

    auto& slot = resp_q_[resp_q_tail_];
    uint16_t out_len = 0u;
    auto ans =
        ProcessOneCommand(in_isr, req, req_len, slot.payload, RESP_SLOT_SIZE, out_len);
    UNUSED(ans);
    slot.len = ClipResponseLength(out_len, RESP_SLOT_SIZE);

    resp_q_tail_ = NextRespQueueIndex(resp_q_tail_);
    ++resp_q_count_;
    return true;
  }

  uint8_t OutstandingResponseCount() const
  {
    const uint8_t IN_FLIGHT =
        (ep_data_in_ && ep_data_in_->GetState() == Endpoint::State::BUSY) ? 1u : 0u;
    const uint8_t DEFERRED = deferred_in_resp_valid_ ? 1u : 0u;
    return static_cast<uint8_t>(resp_q_count_ + IN_FLIGHT + DEFERRED);
  }

  bool EnqueueResponse(const uint8_t* data, uint16_t len)
  {
    if (!data || IsResponseQueueFull())
    {
      return false;
    }

    auto& slot = resp_q_[resp_q_tail_];
    const uint16_t CLIPPED = ClipResponseLength(len, RESP_SLOT_SIZE);
    slot.len = CLIPPED;
    if (CLIPPED > 0u)
    {
      Memory::FastCopy(slot.payload, data, CLIPPED);
    }

    resp_q_tail_ = NextRespQueueIndex(resp_q_tail_);
    ++resp_q_count_;
    return true;
  }

  bool SubmitNextQueuedResponseIfIdle()
  {
    if (!ep_data_in_ || ep_data_in_->GetState() != Endpoint::State::IDLE ||
        IsResponseQueueEmpty())
    {
      return false;
    }
    auto& slot = resp_q_[resp_q_head_];
    if (!StartInTransferFromPayload(slot.payload, slot.len))
    {
      return false;
    }
    resp_q_head_ = NextRespQueueIndex(resp_q_head_);
    --resp_q_count_;
    return true;
  }

  void ArmOutTransferIfIdle()
  {
    if (!inited_ || ep_data_out_ == nullptr || ep_data_in_ == nullptr)
    {
      return;
    }

    if (ep_data_out_->GetState() != Endpoint::State::IDLE)
    {
      return;
    }

    // Backpressure on total outstanding responses (in-flight + queued).
    if (OutstandingResponseCount() >= MAX_OUTSTANDING_RESPONSES)
    {
      return;
    }

    uint16_t out_rx_len = GetDapPacketSize();
    if (out_rx_len == 0u)
    {
      out_rx_len = DEFAULT_DAP_PACKET_SIZE;
    }
    if (out_rx_len > MAX_DAP_PACKET_SIZE)
    {
      out_rx_len = MAX_DAP_PACKET_SIZE;
    }

    const uint16_t OUT_MAX_XFER = static_cast<uint16_t>(ep_data_out_->MaxTransferSize());
    if (OUT_MAX_XFER == 0u)
    {
      return;
    }

    if (out_rx_len <= OUT_MAX_XFER)
    {
      (void)ep_data_out_->Transfer(out_rx_len);
      return;
    }

    if (out_rx_len > static_cast<uint16_t>(sizeof(out_req_multi_storage_)))
    {
      out_rx_len = static_cast<uint16_t>(sizeof(out_req_multi_storage_));
    }

    // Endpoint MPS is capped by hardware (HS bulk = 512), so larger DAP requests
    // must be received with multi-bulk reassembly.
    out_req_multi_buf_.addr_ = out_req_multi_storage_;
    out_req_multi_buf_.size_ = out_rx_len;
    (void)ep_data_out_->TransferMultiBulk(out_req_multi_buf_);
  }

 private:
  // ============================================================================
  // Command dispatch
  // ============================================================================

  /**
   * @brief Process one command
   *
   * @param in_isr   ISR context flag
   * @param req      Request buffer
   * @param req_len  Request length
   * @param resp     Response buffer
   * @param resp_cap Response capacity
   * @param out_len  Output response length
   * @return Error code
   */
  ErrorCode ProcessOneCommand(bool in_isr, const uint8_t* req, uint16_t req_len,
                              uint8_t* resp, uint16_t resp_cap, uint16_t& out_len)
  {
    out_len = 0;

    if (!req || !resp || req_len < 1u || resp_cap < 1u)
    {
      if (resp && resp_cap >= 1u)
      {
        BuildNotSupportResponse(resp, resp_cap, out_len);
      }
      return ErrorCode::ARG_ERR;
    }

    const uint8_t CMD = req[0];

    switch (CMD)
    {
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::INFO):
        return HandleInfo(in_isr, req, req_len, resp, resp_cap, out_len);
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::HOST_STATUS):
        return HandleHostStatus(in_isr, req, req_len, resp, resp_cap, out_len);
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::CONNECT):
        return HandleConnect(in_isr, req, req_len, resp, resp_cap, out_len);
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::DISCONNECT):
        return HandleDisconnect(in_isr, req, req_len, resp, resp_cap, out_len);
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::TRANSFER_CONFIGURE):
        return HandleTransferConfigure(in_isr, req, req_len, resp, resp_cap, out_len);
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::TRANSFER):
        return HandleTransfer(in_isr, req, req_len, resp, resp_cap, out_len);
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::TRANSFER_BLOCK):
        return HandleTransferBlock(in_isr, req, req_len, resp, resp_cap, out_len);
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::TRANSFER_ABORT):
        return HandleTransferAbort(in_isr, req, req_len, resp, resp_cap, out_len);
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::WRITE_ABORT):
        return HandleWriteABORT(in_isr, req, req_len, resp, resp_cap, out_len);
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::DELAY):
        return HandleDelay(in_isr, req, req_len, resp, resp_cap, out_len);
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::RESET_TARGET):
        return HandleResetTarget(in_isr, req, req_len, resp, resp_cap, out_len);

      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::SWJ_PINS):
        return HandleSWJPins(in_isr, req, req_len, resp, resp_cap, out_len);
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::SWJ_CLOCK):
        return HandleSWJClock(in_isr, req, req_len, resp, resp_cap, out_len);
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::SWJ_SEQUENCE):
        return HandleSWJSequence(in_isr, req, req_len, resp, resp_cap, out_len);
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::SWD_CONFIGURE):
        return HandleSWDConfigure(in_isr, req, req_len, resp, resp_cap, out_len);
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::SWD_SEQUENCE):
        return HandleSWDSequence(in_isr, req, req_len, resp, resp_cap, out_len);

      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::JTAG_SEQUENCE):
        if constexpr (JTAG_ENABLED)
        {
          return HandleJTAGSequence(in_isr, req, req_len, resp, resp_cap, out_len);
        }
        (void)BuildUnknownCmdResponse(resp, resp_cap, out_len);
        return ErrorCode::NOT_SUPPORT;
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::JTAG_CONFIGURE):
        if constexpr (JTAG_ENABLED)
        {
          return HandleJTAGConfigure(in_isr, req, req_len, resp, resp_cap, out_len);
        }
        (void)BuildUnknownCmdResponse(resp, resp_cap, out_len);
        return ErrorCode::NOT_SUPPORT;
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::JTAG_IDCODE):
        if constexpr (JTAG_ENABLED)
        {
          return HandleJTAGIdCode(in_isr, req, req_len, resp, resp_cap, out_len);
        }
        (void)BuildUnknownCmdResponse(resp, resp_cap, out_len);
        return ErrorCode::NOT_SUPPORT;

      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::QUEUE_COMMANDS):
        return HandleQueueCommands(in_isr, req, req_len, resp, resp_cap, out_len);
      case ToU8(LibXR::USB::DapLinkV2Def::CommandId::EXECUTE_COMMANDS):
        return HandleExecuteCommands(in_isr, req, req_len, resp, resp_cap, out_len);

      default:
        (void)BuildUnknownCmdResponse(resp, resp_cap, out_len);
        return ErrorCode::NOT_SUPPORT;
    }
  }

  /**
   * @brief Build NOT_SUPPORT response
   *
   * @param resp     Response buffer
   * @param resp_cap Response capacity
   * @param out_len  Output response length
   */
  void BuildNotSupportResponse(uint8_t* resp, uint16_t resp_cap, uint16_t& out_len)
  {
    if (!resp || resp_cap < 1u)
    {
      out_len = 0u;
      return;
    }
    resp[0] = 0xFFu;
    out_len = 1u;
  }

 private:
  // ============================================================================
  // DAP_Info
  // ============================================================================

  ErrorCode HandleInfo(bool /*in_isr*/, const uint8_t* req, uint16_t req_len,
                       uint8_t* resp, uint16_t resp_cap, uint16_t& out_len)
  {
    if (req_len < 2u)
    {
      resp[0] = ToU8(LibXR::USB::DapLinkV2Def::CommandId::INFO);
      resp[1] = 0u;
      out_len = 2u;
      return ErrorCode::ARG_ERR;
    }

    const uint8_t INFO_ID = req[1];

    resp[0] = ToU8(LibXR::USB::DapLinkV2Def::CommandId::INFO);

    switch (INFO_ID)
    {
      case ToU8(LibXR::USB::DapLinkV2Def::InfoId::VENDOR):
        return BuildInfoStringResponse(resp[0], info_.vendor, resp, resp_cap, out_len);
      case ToU8(LibXR::USB::DapLinkV2Def::InfoId::PRODUCT):
        return BuildInfoStringResponse(resp[0], info_.product, resp, resp_cap, out_len);
      case ToU8(LibXR::USB::DapLinkV2Def::InfoId::SERIAL_NUMBER):
        return BuildInfoStringResponse(resp[0], info_.serial, resp, resp_cap, out_len);
      case ToU8(LibXR::USB::DapLinkV2Def::InfoId::FIRMWARE_VERSION):
        return BuildInfoStringResponse(resp[0], info_.firmware_ver, resp, resp_cap,
                                       out_len);

      case ToU8(LibXR::USB::DapLinkV2Def::InfoId::DEVICE_VENDOR):
        return BuildInfoStringResponse(resp[0], info_.device_vendor, resp, resp_cap,
                                       out_len);
      case ToU8(LibXR::USB::DapLinkV2Def::InfoId::DEVICE_NAME):
        return BuildInfoStringResponse(resp[0], info_.device_name, resp, resp_cap,
                                       out_len);
      case ToU8(LibXR::USB::DapLinkV2Def::InfoId::BOARD_VENDOR):
        return BuildInfoStringResponse(resp[0], info_.board_vendor, resp, resp_cap,
                                       out_len);
      case ToU8(LibXR::USB::DapLinkV2Def::InfoId::BOARD_NAME):
        return BuildInfoStringResponse(resp[0], info_.board_name, resp, resp_cap,
                                       out_len);
      case ToU8(LibXR::USB::DapLinkV2Def::InfoId::PRODUCT_FIRMWARE_VERSION):
        return BuildInfoStringResponse(resp[0], info_.product_fw_ver, resp, resp_cap,
                                       out_len);

      case ToU8(LibXR::USB::DapLinkV2Def::InfoId::CAPABILITIES):
      {
        uint8_t caps = LibXR::USB::DapLinkV2Def::DAP_CAP_SWD;
        if constexpr (JTAG_ENABLED)
        {
          if (jtag_ != nullptr)
          {
            caps = static_cast<uint8_t>(caps | LibXR::USB::DapLinkV2Def::DAP_CAP_JTAG);
          }
        }
        return BuildInfoU8Response(resp[0], caps, resp, resp_cap, out_len);
      }
      case ToU8(LibXR::USB::DapLinkV2Def::InfoId::PACKET_COUNT):
        return BuildInfoU8Response(resp[0], PACKET_COUNT_EFFECTIVE, resp, resp_cap,
                                   out_len);
      case ToU8(LibXR::USB::DapLinkV2Def::InfoId::PACKET_SIZE):
      {
        const uint16_t DAP_PS = GetDapPacketSize();
        return BuildInfoU16Response(resp[0], DAP_PS, resp, resp_cap, out_len);
      }
      case ToU8(LibXR::USB::DapLinkV2Def::InfoId::TIMESTAMP_CLOCK):
        return BuildInfoU32Response(resp[0], 1000000U, resp, resp_cap, out_len);

      default:
        resp[1] = 0u;
        out_len = 2u;
        return ErrorCode::OK;
    }
  }

  ErrorCode BuildInfoStringResponse(uint8_t cmd, const char* str, uint8_t* resp,
                                    uint16_t resp_cap, uint16_t& out_len)
  {
    resp[0] = cmd;
    resp[1] = 0u;

    if (!str)
    {
      out_len = 2u;
      return ErrorCode::OK;
    }

    const size_t N_WITH_NUL = std::strlen(str) + 1;  // include '\0'
    const size_t MAX_PAYLOAD = (resp_cap >= 2u) ? (resp_cap - 2u) : 0u;
    if (MAX_PAYLOAD == 0u)
    {
      out_len = 2u;
      return ErrorCode::OK;
    }

    const size_t COPY_N = (N_WITH_NUL > MAX_PAYLOAD) ? MAX_PAYLOAD : N_WITH_NUL;
    Memory::FastCopy(&resp[2], str, COPY_N);

    // Ensure termination even when truncated
    resp[2 + COPY_N - 1] = 0x00;

    resp[1] = static_cast<uint8_t>(COPY_N);
    out_len = static_cast<uint16_t>(2u + COPY_N);
    return ErrorCode::OK;
  }

  ErrorCode BuildInfoU8Response(uint8_t cmd, uint8_t val, uint8_t* resp,
                                uint16_t resp_cap, uint16_t& out_len)
  {
    if (resp_cap < 3u)
    {
      out_len = 0;
      return ErrorCode::NOT_FOUND;
    }
    resp[0] = cmd;
    resp[1] = 1u;
    resp[2] = val;
    out_len = 3u;
    return ErrorCode::OK;
  }

  ErrorCode BuildInfoU16Response(uint8_t cmd, uint16_t val, uint8_t* resp,
                                 uint16_t resp_cap, uint16_t& out_len)
  {
    if (resp_cap < 4u)
    {
      out_len = 0;
      return ErrorCode::NOT_FOUND;
    }
    resp[0] = cmd;
    resp[1] = 2u;

    // Little-endian device only: direct memcpy to payload (alignment-safe).
    Memory::FastCopy(&resp[2], &val, sizeof(val));

    out_len = 4u;
    return ErrorCode::OK;
  }

  ErrorCode BuildInfoU32Response(uint8_t cmd, uint32_t val, uint8_t* resp,
                                 uint16_t resp_cap, uint16_t& out_len)
  {
    if (resp_cap < 6u)
    {
      out_len = 0;
      return ErrorCode::NOT_FOUND;
    }
    resp[0] = cmd;
    resp[1] = 4u;

    // Little-endian device only: direct memcpy to payload (alignment-safe).
    Memory::FastCopy(&resp[2], &val, sizeof(val));

    out_len = 6u;
    return ErrorCode::OK;
  }

 private:
  // ============================================================================
  // Simple control handlers
  // ============================================================================

  ErrorCode HandleHostStatus(bool /*in_isr*/, const uint8_t* /*req*/,
                             uint16_t /*req_len*/, uint8_t* resp, uint16_t resp_cap,
                             uint16_t& out_len)
  {
    if (resp_cap < 2u)
    {
      out_len = 0;
      return ErrorCode::NOT_FOUND;
    }
    resp[0] = ToU8(LibXR::USB::DapLinkV2Def::CommandId::HOST_STATUS);
    resp[1] = ToU8(LibXR::USB::DapLinkV2Def::Status::OK);
    out_len = 2u;
    return ErrorCode::OK;
  }

  ErrorCode HandleConnect(bool /*in_isr*/, const uint8_t* req, uint16_t req_len,
                          uint8_t* resp, uint16_t resp_cap, uint16_t& out_len)
  {
    if (resp_cap < 2u)
    {
      out_len = 0;
      return ErrorCode::NOT_FOUND;
    }

    resp[0] = ToU8(LibXR::USB::DapLinkV2Def::CommandId::CONNECT);

    uint8_t port = 0u;
    if (req_len >= 2u)
    {
      port = req[1];
    }

    if (port == 0u || port == ToU8(LibXR::USB::DapLinkV2Def::Port::SWD))
    {
      (void)swd_.EnterSwd();
      (void)swd_.SetClockHz(swj_clock_hz_);

      dap_state_.debug_port = LibXR::USB::DapLinkV2Def::DebugPort::SWD;
      dap_state_.transfer_abort = false;
      ResetQueuedCommandState();

      resp[1] = ToU8(LibXR::USB::DapLinkV2Def::Port::SWD);
    }
    else if (port == ToU8(LibXR::USB::DapLinkV2Def::Port::JTAG))
    {
      if constexpr (!JTAG_ENABLED)
      {
        resp[1] = ToU8(LibXR::USB::DapLinkV2Def::Port::DISABLED);
      }
      else
      {
        if (jtag_ == nullptr)
        {
          resp[1] = ToU8(LibXR::USB::DapLinkV2Def::Port::DISABLED);
        }
        else
        {
          static constexpr uint8_t SWD_TO_JTAG_SEQUENCE[2] = {0x3Cu, 0xE7u};
          (void)swd_.SetClockHz(swj_clock_hz_);
          (void)swd_.LineReset();
          (void)swd_.SeqWriteBits(16u, SWD_TO_JTAG_SEQUENCE);
          (void)swd_.LineReset();
          (void)jtag_->SetClockHz(swj_clock_hz_);
          (void)jtag_->ResetTap();

          dap_state_.debug_port = LibXR::USB::DapLinkV2Def::DebugPort::JTAG;
          dap_state_.transfer_abort = false;
          ResetQueuedCommandState();
          ResetJtagChainState();
          resp[1] = ToU8(LibXR::USB::DapLinkV2Def::Port::JTAG);
        }
      }
    }
    else
    {
      resp[1] = ToU8(LibXR::USB::DapLinkV2Def::Port::DISABLED);
    }

    out_len = 2u;
    return ErrorCode::OK;
  }

  ErrorCode HandleDisconnect(bool /*in_isr*/, const uint8_t* /*req*/,
                             uint16_t /*req_len*/, uint8_t* resp, uint16_t resp_cap,
                             uint16_t& out_len)
  {
    if (resp_cap < 2u)
    {
      out_len = 0;
      return ErrorCode::NOT_FOUND;
    }

    swd_.Close();
    if constexpr (JTAG_ENABLED)
    {
      if (jtag_ != nullptr)
      {
        jtag_->Close();
      }
    }
    dap_state_.debug_port = LibXR::USB::DapLinkV2Def::DebugPort::DISABLED;
    dap_state_.transfer_abort = false;
    ResetQueuedCommandState();

    resp[0] = ToU8(LibXR::USB::DapLinkV2Def::CommandId::DISCONNECT);
    resp[1] = ToU8(LibXR::USB::DapLinkV2Def::Status::OK);
    out_len = 2u;
    return ErrorCode::OK;
  }

  ErrorCode HandleTransferConfigure(bool /*in_isr*/, const uint8_t* req, uint16_t req_len,
                                    uint8_t* resp, uint16_t resp_cap, uint16_t& out_len)
  {
    if (resp_cap < 2u)
    {
      out_len = 0;
      return ErrorCode::NOT_FOUND;
    }

    resp[0] = ToU8(LibXR::USB::DapLinkV2Def::CommandId::TRANSFER_CONFIGURE);

    // Req: [0]=0x04 [1]=idle_cycles [2..3]=wait_retry [4..5]=match_retry
    if (req_len < 6u)
    {
      resp[1] = ToU8(LibXR::USB::DapLinkV2Def::Status::ERROR);
      out_len = 2u;
      return ErrorCode::ARG_ERR;
    }

    const uint8_t IDLE = req[1];

    uint16_t wait_retry = 0u;
    uint16_t match_retry = 0u;
    Memory::FastCopy(&wait_retry, &req[2], sizeof(wait_retry));
    Memory::FastCopy(&match_retry, &req[4], sizeof(match_retry));

    dap_state_.transfer_cfg.idle_cycles = IDLE;
    dap_state_.transfer_cfg.retry_count = wait_retry;
    dap_state_.transfer_cfg.match_retry = match_retry;

    // Map to SWD transaction policy
    Debug::Swd::TransferPolicy pol = swd_.GetTransferPolicy();
    pol.idle_cycles = IDLE;
    pol.wait_retry = wait_retry;
    swd_.SetTransferPolicy(pol);

    resp[1] = ToU8(LibXR::USB::DapLinkV2Def::Status::OK);
    out_len = 2u;
    return ErrorCode::OK;
  }

  ErrorCode HandleTransferAbort(bool /*in_isr*/, const uint8_t* /*req*/,
                                uint16_t /*req_len*/, uint8_t* resp, uint16_t resp_cap,
                                uint16_t& out_len)
  {
    if (resp_cap < 2u)
    {
      out_len = 0;
      return ErrorCode::NOT_FOUND;
    }

    SetTransferAbortFlag(true);

    resp[0] = ToU8(LibXR::USB::DapLinkV2Def::CommandId::TRANSFER_ABORT);
    resp[1] = ToU8(LibXR::USB::DapLinkV2Def::Status::OK);
    out_len = 2u;
    return ErrorCode::OK;
  }

  ErrorCode HandleWriteABORT(bool in_isr, const uint8_t* req, uint16_t req_len,
                             uint8_t* resp, uint16_t resp_cap, uint16_t& out_len)
  {
    if (resp_cap < 2u)
    {
      out_len = 0;
      return ErrorCode::NOT_FOUND;
    }

    if constexpr (JTAG_ENABLED)
    {
      if (dap_state_.debug_port == LibXR::USB::DapLinkV2Def::DebugPort::JTAG)
      {
        return HandleJtagWriteAbort(in_isr, req, req_len, resp, resp_cap, out_len);
      }
    }

    resp[0] = ToU8(LibXR::USB::DapLinkV2Def::CommandId::WRITE_ABORT);

    if (req_len < 6u)
    {
      resp[1] = ToU8(LibXR::USB::DapLinkV2Def::Status::ERROR);
      out_len = 2u;
      return ErrorCode::ARG_ERR;
    }

    uint32_t flags = 0u;
    Memory::FastCopy(&flags, &req[2], sizeof(flags));

    LibXR::Debug::SwdProtocol::Ack ack = LibXR::Debug::SwdProtocol::Ack::PROTOCOL;

    const ErrorCode EC = swd_.WriteAbortTxn(flags, ack);
    resp[1] = (EC == ErrorCode::OK && ack == LibXR::Debug::SwdProtocol::Ack::OK)
                  ? ToU8(LibXR::USB::DapLinkV2Def::Status::OK)
                  : ToU8(LibXR::USB::DapLinkV2Def::Status::ERROR);

    out_len = 2u;
    return ErrorCode::OK;
  }

  ErrorCode HandleDelay(bool /*in_isr*/, const uint8_t* req, uint16_t req_len,
                        uint8_t* resp, uint16_t resp_cap, uint16_t& out_len)
  {
    if (resp_cap < 2u)
    {
      out_len = 0;
      return ErrorCode::NOT_FOUND;
    }

    resp[0] = ToU8(LibXR::USB::DapLinkV2Def::CommandId::DELAY);

    if (req_len < 3u)
    {
      resp[1] = ToU8(LibXR::USB::DapLinkV2Def::Status::ERROR);
      out_len = 2u;
      return ErrorCode::ARG_ERR;
    }

    uint16_t us = 0u;
    Memory::FastCopy(&us, &req[1], sizeof(us));

    LibXR::Timebase::DelayMicroseconds(us);

    resp[1] = ToU8(LibXR::USB::DapLinkV2Def::Status::OK);
    out_len = 2u;
    return ErrorCode::OK;
  }

  ErrorCode HandleResetTarget(bool in_isr, const uint8_t* /*req*/, uint16_t /*req_len*/,
                              uint8_t* resp, uint16_t resp_cap, uint16_t& out_len)
  {
    if (!resp || resp_cap < 3u)
    {
      out_len = 0u;
      return ErrorCode::NOT_FOUND;
    }

    resp[0] = ToU8(LibXR::USB::DapLinkV2Def::CommandId::RESET_TARGET);

    uint8_t execute = 0u;

    if (nreset_gpio_ != nullptr)
    {
      DriveReset(false);
      DelayUsIfAllowed(in_isr, 1000u);
      DriveReset(true);
      DelayUsIfAllowed(in_isr, 1000u);
      execute = 1u;
    }

    // Always return DAP_OK; if reset is not implemented, Execute=0.
    resp[1] = DAP_OK;
    resp[2] = execute;
    out_len = 3u;
    return ErrorCode::OK;
  }

 private:
  // ============================================================================
  // SWJ / SWD handlers
  // ============================================================================
  ErrorCode HandleSWJPins(bool /*in_isr*/, const uint8_t* req, uint16_t req_len,
                          uint8_t* resp, uint16_t resp_cap, uint16_t& out_len)
  {
    if (!resp || resp_cap < 2u)
    {
      out_len = 0u;
      return ErrorCode::NOT_FOUND;
    }

    resp[0] = ToU8(LibXR::USB::DapLinkV2Def::CommandId::SWJ_PINS);

    // Req: [0]=0x10 [1]=PinOut [2]=PinSelect [3..6]=PinWait(us)
    if (!req || req_len < 7u)
    {
      resp[1] = 0u;
      out_len = 2u;
      return ErrorCode::ARG_ERR;
    }

    const uint8_t PIN_OUT = req[1];
    const uint8_t PIN_SEL = req[2];

    uint32_t wait_us = 0u;
    Memory::FastCopy(&wait_us, &req[3], sizeof(wait_us));

    // 0) Latch requested states into shadow for ALL selected pins (even if unsupported)
    swj_shadow_ = static_cast<uint8_t>((swj_shadow_ & static_cast<uint8_t>(~PIN_SEL)) |
                                       (PIN_OUT & PIN_SEL));

    // 1) Only physically support nRESET (best-effort)
    if ((PIN_SEL & LibXR::USB::DapLinkV2Def::DAP_SWJ_NRESET) != 0u)
    {
      const bool LEVEL_HIGH =
          ((PIN_OUT & LibXR::USB::DapLinkV2Def::DAP_SWJ_NRESET) != 0u);
      // DriveReset updates last_nreset_level_high_ and shadow, and writes GPIO if
      // present.
      DriveReset(LEVEL_HIGH);
    }

    // Read helper: start from shadow; override nRESET with physical level if wired.
    auto read_pins = [&]() -> uint8_t
    {
      uint8_t pin_in = swj_shadow_;

      if (nreset_gpio_ != nullptr)
      {
        if (nreset_gpio_->Read())
        {
          pin_in |= LibXR::USB::DapLinkV2Def::DAP_SWJ_NRESET;
        }
        else
        {
          pin_in = static_cast<uint8_t>(
              pin_in & static_cast<uint8_t>(~LibXR::USB::DapLinkV2Def::DAP_SWJ_NRESET));
        }
      }

      return pin_in;
    };

    // 2) PinWait: wait until (PinInput & PinSelect) matches (PinOut & PinSelect), or
    // timeout. Since SWCLK/SWDIO are shadow-only now, their selected bits will already
    // match immediately.
    uint8_t pin_in = 0u;

    if (wait_us == 0u || PIN_SEL == 0u)
    {
      pin_in = read_pins();
    }
    else
    {
      const uint64_t START = LibXR::Timebase::GetMicroseconds();
      const uint8_t EXPECT = static_cast<uint8_t>(PIN_OUT & PIN_SEL);

      do
      {
        pin_in = read_pins();
        if ((pin_in & PIN_SEL) == EXPECT)
        {
          break;
        }
      } while ((static_cast<uint64_t>(LibXR::Timebase::GetMicroseconds()) - START) <
               wait_us);
    }

    resp[1] = pin_in;
    out_len = 2u;
    return ErrorCode::OK;
  }

  ErrorCode HandleSWJClock(bool /*in_isr*/, const uint8_t* req, uint16_t req_len,
                           uint8_t* resp, uint16_t resp_cap, uint16_t& out_len)
  {
    if (resp_cap < 2u)
    {
      out_len = 0;
      return ErrorCode::NOT_FOUND;
    }

    resp[0] = ToU8(LibXR::USB::DapLinkV2Def::CommandId::SWJ_CLOCK);

    if (req_len < 5u)
    {
      resp[1] = ToU8(LibXR::USB::DapLinkV2Def::Status::ERROR);
      out_len = 2u;
      return ErrorCode::ARG_ERR;
    }

    uint32_t hz = 0u;
    Memory::FastCopy(&hz, &req[1], sizeof(hz));

    swj_clock_hz_ = hz;
    (void)swd_.SetClockHz(hz);
    if constexpr (JTAG_ENABLED)
    {
      if (jtag_ != nullptr)
      {
        (void)jtag_->SetClockHz(hz);
      }
    }

    resp[1] = ToU8(LibXR::USB::DapLinkV2Def::Status::OK);
    out_len = 2u;
    return ErrorCode::OK;
  }

  ErrorCode HandleSWJSequence(bool /*in_isr*/, const uint8_t* req, uint16_t req_len,
                              uint8_t* resp, uint16_t resp_cap, uint16_t& out_len)
  {
    if (!resp || resp_cap < 2u)
    {
      out_len = 0u;
      return ErrorCode::NOT_FOUND;
    }

    resp[0] = ToU8(LibXR::USB::DapLinkV2Def::CommandId::SWJ_SEQUENCE);
    resp[1] = ToU8(LibXR::USB::DapLinkV2Def::Status::OK);
    out_len = 2u;

    // Req: [0]=0x12 [1]=bit_count(0=>256) [2..]=data (LSB-first)
    if (!req || req_len < 2u)
    {
      resp[1] = ToU8(LibXR::USB::DapLinkV2Def::Status::ERROR);
      return ErrorCode::ARG_ERR;
    }

    const uint8_t RAW_COUNT = req[1];
    const uint32_t BIT_COUNT =
        (RAW_COUNT == 0u) ? 256u : static_cast<uint32_t>(RAW_COUNT);
    const uint32_t BYTE_COUNT = (BIT_COUNT + 7u) / 8u;

    if (2u + BYTE_COUNT > req_len)
    {
      resp[1] = ToU8(LibXR::USB::DapLinkV2Def::Status::ERROR);
      return ErrorCode::ARG_ERR;
    }

    const uint8_t* data = &req[2];

    // Delegate to SwdPort implementation (no RawMode / no pin-level control here)
    const ErrorCode EC = swd_.SeqWriteBits(BIT_COUNT, data);
    if (EC != ErrorCode::OK)
    {
      resp[1] = ToU8(LibXR::USB::DapLinkV2Def::Status::ERROR);
      // Keep transport-level OK so host still gets a valid response.
      return ErrorCode::OK;
    }

    // Maintain shadow semantics: keep SWCLK=0, SWDIO=last bit
    swj_shadow_ = static_cast<uint8_t>(
        swj_shadow_ & static_cast<uint8_t>(~LibXR::USB::DapLinkV2Def::DAP_SWJ_SWCLK_TCK));

    bool last_swdio = false;
    if (BIT_COUNT != 0u)
    {
      const uint32_t LAST_I = BIT_COUNT - 1u;
      last_swdio = (((data[LAST_I / 8u] >> (LAST_I & 7u)) & 0x01u) != 0u);
    }

    if (last_swdio)
    {
      swj_shadow_ |= LibXR::USB::DapLinkV2Def::DAP_SWJ_SWDIO_TMS;
    }
    else
    {
      swj_shadow_ = static_cast<uint8_t>(
          swj_shadow_ &
          static_cast<uint8_t>(~LibXR::USB::DapLinkV2Def::DAP_SWJ_SWDIO_TMS));
    }

    return ErrorCode::OK;
  }

  ErrorCode HandleSWDConfigure(bool /*in_isr*/, const uint8_t* req, uint16_t req_len,
                               uint8_t* resp, uint16_t resp_cap, uint16_t& out_len)
  {
    if (resp_cap < 2u)
    {
      out_len = 0;
      return ErrorCode::NOT_FOUND;
    }

    resp[0] = ToU8(LibXR::USB::DapLinkV2Def::CommandId::SWD_CONFIGURE);

    if (req == nullptr || req_len < 2u)
    {
      resp[1] = ToU8(LibXR::USB::DapLinkV2Def::Status::ERROR);
      out_len = 2u;
      return ErrorCode::ARG_ERR;
    }

    const uint8_t cfg = req[1];
    dap_state_.swd_cfg.turnaround = static_cast<uint8_t>((cfg & 0x03u) + 1u);
    dap_state_.swd_cfg.data_phase = ((cfg & 0x04u) != 0u);

    resp[1] = ToU8(LibXR::USB::DapLinkV2Def::Status::OK);
    out_len = 2u;
    return ErrorCode::OK;
  }

  ErrorCode HandleSWDSequence(bool /*in_isr*/, const uint8_t* req, uint16_t req_len,
                              uint8_t* resp, uint16_t resp_cap, uint16_t& out_len)
  {
    if (!req || !resp || resp_cap < 2u)
    {
      out_len = 0u;
      return ErrorCode::ARG_ERR;
    }

    resp[0] = ToU8(LibXR::USB::DapLinkV2Def::CommandId::SWD_SEQUENCE);
    resp[1] = DAP_OK;
    out_len = 2u;

    // Req: [0]=0x1D [1]=SequenceCount [ ... sequences ... ]
    // Each sequence:
    //   INFO: [7]=Direction (1=input,0=output), [5:0]=cycles (0=>64)
    //   If output: followed by ceil(cycles/8) bytes data (LSB-first)
    //   If input : no data in request; response appends ceil(cycles/8) bytes data
    //   (LSB-first)
    if (req_len < 2u)
    {
      resp[1] = DAP_ERROR;
      out_len = 2u;
      return ErrorCode::ARG_ERR;
    }

    const uint8_t SEQ_CNT = req[1];
    uint16_t req_off = 2u;
    uint16_t resp_off = 2u;

    for (uint32_t s = 0; s < SEQ_CNT; ++s)
    {
      if (req_off >= req_len)
      {
        resp[1] = DAP_ERROR;
        out_len = 2u;
        return ErrorCode::ARG_ERR;
      }

      const uint8_t INFO = req[req_off++];

      uint32_t cycles = static_cast<uint32_t>(INFO & 0x3Fu);
      if (cycles == 0u)
      {
        cycles = 64u;
      }

      const bool MODE_IN = ((INFO & 0x80u) != 0u);
      const uint16_t BYTES = static_cast<uint16_t>((cycles + 7u) / 8u);

      if (!MODE_IN)
      {
        // Output: data bytes are in request
        if (req_off + BYTES > req_len)
        {
          resp[1] = DAP_ERROR;
          out_len = 2u;
          return ErrorCode::ARG_ERR;
        }

        const uint8_t* data = &req[req_off];
        req_off = static_cast<uint16_t>(req_off + BYTES);

        const ErrorCode EC = swd_.SeqWriteBits(cycles, data);
        if (EC != ErrorCode::OK)
        {
          resp[1] = DAP_ERROR;
          out_len = 2u;
          // Return OK so host gets a valid response packet
          return ErrorCode::OK;
        }
      }
      else
      {
        // Input: captured data is appended to response
        if (resp_off + BYTES > resp_cap)
        {
          resp[1] = DAP_ERROR;
          out_len = 2u;
          return ErrorCode::NOT_FOUND;
        }

        Memory::FastSet(&resp[resp_off], 0, BYTES);

        const ErrorCode EC = swd_.SeqReadBits(cycles, &resp[resp_off]);
        if (EC != ErrorCode::OK)
        {
          resp[1] = DAP_ERROR;
          out_len = 2u;
          return ErrorCode::OK;
        }

        resp_off = static_cast<uint16_t>(resp_off + BYTES);
      }
    }

    out_len = resp_off;
    return ErrorCode::OK;
  }

  ErrorCode HandleJTAGSequence(bool /*in_isr*/, const uint8_t* req, uint16_t req_len,
                               uint8_t* resp, uint16_t resp_cap, uint16_t& out_len)
  {
    if (!req || !resp || resp_cap < 2u)
    {
      out_len = 0u;
      return ErrorCode::ARG_ERR;
    }

    resp[0] = ToU8(LibXR::USB::DapLinkV2Def::CommandId::JTAG_SEQUENCE);
    resp[1] = DAP_OK;
    out_len = 2u;

    if (jtag_ == nullptr)
    {
      resp[1] = DAP_ERROR;
      return ErrorCode::OK;
    }

    if (req_len < 2u)
    {
      resp[1] = DAP_ERROR;
      return ErrorCode::ARG_ERR;
    }

    const uint8_t SEQ_CNT = req[1];
    uint16_t req_off = 2u;
    uint16_t resp_off = 2u;

    for (uint32_t s = 0; s < SEQ_CNT; ++s)
    {
      if (req_off >= req_len)
      {
        resp[1] = DAP_ERROR;
        out_len = 2u;
        return ErrorCode::ARG_ERR;
      }

      const uint8_t INFO = req[req_off++];
      uint32_t cycles = static_cast<uint32_t>(INFO & 0x3Fu);
      if (cycles == 0u)
      {
        cycles = 64u;
      }

      const bool TMS = ((INFO & LibXR::Debug::JtagProtocol::JTAG_SEQUENCE_TMS) != 0u);
      const bool CAPTURE = ((INFO & LibXR::Debug::JtagProtocol::JTAG_SEQUENCE_TDO) != 0u);
      const uint16_t BYTES = static_cast<uint16_t>((cycles + 7u) / 8u);

      if (req_off + BYTES > req_len)
      {
        resp[1] = DAP_ERROR;
        out_len = 2u;
        return ErrorCode::ARG_ERR;
      }
      if (CAPTURE && (resp_off + BYTES > resp_cap))
      {
        resp[1] = DAP_ERROR;
        out_len = 2u;
        return ErrorCode::NOT_FOUND;
      }

      const uint8_t* data = &req[req_off];
      uint8_t* out = CAPTURE ? &resp[resp_off] : nullptr;
      if (CAPTURE)
      {
        Memory::FastSet(out, 0, BYTES);
      }

      const ErrorCode ec = jtag_->Sequence(cycles, TMS, data, out);
      if (ec != ErrorCode::OK)
      {
        resp[1] = DAP_ERROR;
        out_len = 2u;
        return ErrorCode::OK;
      }

      req_off = static_cast<uint16_t>(req_off + BYTES);
      if (CAPTURE)
      {
        resp_off = static_cast<uint16_t>(resp_off + BYTES);
      }
    }

    out_len = resp_off;
    return ErrorCode::OK;
  }

  ErrorCode HandleJTAGConfigure(bool /*in_isr*/, const uint8_t* req, uint16_t req_len,
                                uint8_t* resp, uint16_t resp_cap, uint16_t& out_len)
  {
    if (!req || !resp || resp_cap < 2u)
    {
      out_len = 0u;
      return ErrorCode::ARG_ERR;
    }

    resp[0] = ToU8(LibXR::USB::DapLinkV2Def::CommandId::JTAG_CONFIGURE);
    resp[1] = DAP_OK;
    out_len = 2u;

    if (jtag_ == nullptr)
    {
      resp[1] = DAP_ERROR;
      return ErrorCode::OK;
    }

    if (req_len < 2u)
    {
      resp[1] = DAP_ERROR;
      return ErrorCode::ARG_ERR;
    }

    const uint8_t count = req[1];
    if (count > JTAG_MAX_DEVICES || req_len < static_cast<uint16_t>(2u + count))
    {
      resp[1] = DAP_ERROR;
      return ErrorCode::ARG_ERR;
    }

    Memory::FastSet(jtag_ir_len_, 0, sizeof(jtag_ir_len_));

    uint32_t bits = 0u;
    for (uint32_t i = 0; i < count; ++i)
    {
      const uint8_t len = req[static_cast<uint16_t>(2u + i)];
      jtag_ir_len_[i] = len;
      bits += len;
    }

    jtag_chain_.count = count;
    if (jtag_chain_.index >= jtag_chain_.count)
    {
      jtag_chain_.index = 0u;
    }
    jtag_chain_.ir_before_bits_len = 0u;
    jtag_chain_.ir_after_bits_len = 0u;
    jtag_chain_.dr_before_bits_len = 0u;
    jtag_chain_.dr_after_bits_len = 0u;
    return ErrorCode::OK;
  }

  ErrorCode HandleJTAGIdCode(bool /*in_isr*/, const uint8_t* req, uint16_t req_len,
                             uint8_t* resp, uint16_t resp_cap, uint16_t& out_len)
  {
    if (!req || !resp || resp_cap < 6u)
    {
      out_len = 0u;
      return ErrorCode::ARG_ERR;
    }

    resp[0] = ToU8(LibXR::USB::DapLinkV2Def::CommandId::JTAG_IDCODE);
    resp[1] = DAP_ERROR;
    out_len = 2u;

    if (jtag_ == nullptr ||
        dap_state_.debug_port != LibXR::USB::DapLinkV2Def::DebugPort::JTAG)
    {
      return ErrorCode::OK;
    }

    if (req_len < 2u)
    {
      return ErrorCode::ARG_ERR;
    }

    const uint8_t index = req[1];
    if (index >= jtag_chain_.count)
    {
      return ErrorCode::OK;
    }

    jtag_chain_.index = index;
    LibXR::Debug::JtagProtocol::UpdateChainCache(jtag_chain_);

    LibXR::Debug::JtagDp dp(*jtag_, &jtag_chain_);
    uint32_t idcode = 0u;
    ErrorCode ec = dp.ReadIdCode(idcode);
    if (ec == ErrorCode::OK &&
        (idcode == 0u || idcode == 0xFFFF'FFFFu || ((idcode & 0x1u) == 0u)) &&
        jtag_ir_len_[index] != 4u)
    {
      uint32_t fallback_idcode = 0u;
      ec = dp.ReadIdCodeWithIr(0x01u, fallback_idcode);
      if (ec != ErrorCode::OK)
      {
        return ec;
      }
      if (fallback_idcode != 0u && fallback_idcode != 0xFFFF'FFFFu &&
          ((fallback_idcode & 0x1u) != 0u))
      {
        idcode = fallback_idcode;
      }
    }
    if (ec != ErrorCode::OK)
    {
      return ec;
    }

    resp[1] = DAP_OK;
    Memory::FastCopy(&resp[2], &idcode, sizeof(idcode));
    out_len = 6u;
    return ErrorCode::OK;
  }

  /**
   * @brief Queue one packed command batch for later EXECUTE_COMMANDS
   */
  ErrorCode HandleQueueCommands(bool /*in_isr*/, const uint8_t* req, uint16_t req_len,
                                uint8_t* resp, uint16_t resp_cap, uint16_t& out_len)
  {
    if (!req || req_len < 2u)
    {
      return BuildCmdStatusResponse(
          ToU8(LibXR::USB::DapLinkV2Def::CommandId::QUEUE_COMMANDS), DAP_ERROR, resp,
          resp_cap, out_len);
    }

    const uint8_t NUM = req[1];
    if (NUM == 0u)
    {
      return BuildCmdStatusResponse(
          ToU8(LibXR::USB::DapLinkV2Def::CommandId::QUEUE_COMMANDS), DAP_OK, resp,
          resp_cap, out_len);
    }

    uint16_t req_off = 2u;
    for (uint32_t i = 0u; i < NUM; ++i)
    {
      if (req_off >= req_len)
      {
        return BuildCmdStatusResponse(
            ToU8(LibXR::USB::DapLinkV2Def::CommandId::QUEUE_COMMANDS), DAP_ERROR, resp,
            resp_cap, out_len);
      }

      uint16_t cmd_len = 0u;
      if (!ParseQueuedCommandLength(&req[req_off],
                                    static_cast<uint16_t>(req_len - req_off), cmd_len) ||
          cmd_len == 0u)
      {
        return BuildCmdStatusResponse(
            ToU8(LibXR::USB::DapLinkV2Def::CommandId::QUEUE_COMMANDS), DAP_ERROR, resp,
            resp_cap, out_len);
      }
      req_off = static_cast<uint16_t>(req_off + cmd_len);
    }

    if (req_off != req_len)
    {
      return BuildCmdStatusResponse(
          ToU8(LibXR::USB::DapLinkV2Def::CommandId::QUEUE_COMMANDS), DAP_ERROR, resp,
          resp_cap, out_len);
    }

    if (static_cast<uint32_t>(queued_request_length_) +
                static_cast<uint32_t>(req_len - 2u) >
            static_cast<uint32_t>(QUEUED_REQ_BUFFER_SIZE) ||
        static_cast<uint32_t>(queued_command_count_) + static_cast<uint32_t>(NUM) >
            static_cast<uint32_t>(QUEUED_CMD_COUNT_MAX))
    {
      return BuildCmdStatusResponse(
          ToU8(LibXR::USB::DapLinkV2Def::CommandId::QUEUE_COMMANDS), DAP_ERROR, resp,
          resp_cap, out_len);
    }

    Memory::FastCopy(&queued_request_buffer_[queued_request_length_], &req[2],
                     req_len - 2u);
    queued_request_length_ =
        static_cast<uint16_t>(queued_request_length_ + (req_len - 2u));
    queued_command_count_ = static_cast<uint16_t>(queued_command_count_ + NUM);

    return BuildCmdStatusResponse(
        ToU8(LibXR::USB::DapLinkV2Def::CommandId::QUEUE_COMMANDS), DAP_OK, resp, resp_cap,
        out_len);
  }

  /**
   * @brief Execute all previously queued commands
   */
  ErrorCode HandleExecuteCommands(bool in_isr, const uint8_t* /*req*/,
                                  uint16_t /*req_len*/, uint8_t* resp, uint16_t resp_cap,
                                  uint16_t& out_len)
  {
    const bool ok = TryExecuteQueuedCommandStream(in_isr, resp, resp_cap, out_len);
    ResetQueuedCommandState();

    if (!ok)
    {
      return BuildCmdStatusResponse(
          ToU8(LibXR::USB::DapLinkV2Def::CommandId::EXECUTE_COMMANDS), DAP_ERROR, resp,
          resp_cap, out_len);
    }

    return ErrorCode::OK;
  }

 private:
  // ============================================================================
  // Transfer helpers
  // ============================================================================

  uint8_t MapAckToDapResp(LibXR::Debug::SwdProtocol::Ack ack) const
  {
    static constexpr uint8_t ACK_MAP[8] = {7u, 1u, 2u, 7u, 4u, 7u, 7u, 7u};
    return ACK_MAP[static_cast<uint8_t>(ack) & 0x07u];
  }

  uint8_t MapAckToDapResp(LibXR::Debug::JtagProtocol::Ack ack) const
  {
    switch (ack)
    {
      case LibXR::Debug::JtagProtocol::Ack::OK:
        return 1u;
      case LibXR::Debug::JtagProtocol::Ack::WAIT:
        return 2u;
      case LibXR::Debug::JtagProtocol::Ack::FAULT:
        return 4u;
      case LibXR::Debug::JtagProtocol::Ack::NO_ACK:
      case LibXR::Debug::JtagProtocol::Ack::PROTOCOL:
      default:
        return 7u;
    }
  }

  static inline uint16_t LoadU16Le(const uint8_t* p)
  {
    return static_cast<uint16_t>(
        static_cast<uint16_t>(p[0]) |
        static_cast<uint16_t>(static_cast<uint16_t>(p[1]) << 8u));
  }

  static inline void StoreU16Le(uint8_t* p, uint16_t v)
  {
    p[0] = static_cast<uint8_t>(v & 0xFFu);
    p[1] = static_cast<uint8_t>((v >> 8u) & 0xFFu);
  }

  static inline uint32_t LoadU32Le(const uint8_t* p)
  {
    return static_cast<uint32_t>(
        static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8u) |
        (static_cast<uint32_t>(p[2]) << 16u) | (static_cast<uint32_t>(p[3]) << 24u));
  }

  static inline void StoreU32Le(uint8_t* p, uint32_t v)
  {
    p[0] = static_cast<uint8_t>(v & 0xFFu);
    p[1] = static_cast<uint8_t>((v >> 8u) & 0xFFu);
    p[2] = static_cast<uint8_t>((v >> 16u) & 0xFFu);
    p[3] = static_cast<uint8_t>((v >> 24u) & 0xFFu);
  }

  ErrorCode TransferTxnFast(const LibXR::Debug::SwdProtocol::Request& req,
                            LibXR::Debug::SwdProtocol::Response& resp)
  {
    const ErrorCode EC0 = swd_.Transfer(req, resp);
    if (EC0 != ErrorCode::OK)
    {
      return EC0;
    }

    if (resp.ack == LibXR::Debug::SwdProtocol::Ack::WAIT)
    {
      return swd_.TransferWithRetry(req, resp);
    }

    if (resp.ack == LibXR::Debug::SwdProtocol::Ack::FAULT)
    {
      const auto POL = swd_.GetTransferPolicy();
      if (POL.clear_sticky_on_fault)
      {
        LibXR::Debug::SwdProtocol::Ack abort_ack =
            LibXR::Debug::SwdProtocol::Ack::PROTOCOL;
        (void)swd_.WriteAbort(LibXR::Debug::SwdProtocol::DP_ABORT_STKCMPCLR |
                                  LibXR::Debug::SwdProtocol::DP_ABORT_STKERRCLR |
                                  LibXR::Debug::SwdProtocol::DP_ABORT_WDERRCLR |
                                  LibXR::Debug::SwdProtocol::DP_ABORT_ORUNERRCLR,
                              abort_ack);
      }
    }

    return ErrorCode::OK;
  }

  ErrorCode DpReadRdbuffFast(uint32_t& val, LibXR::Debug::SwdProtocol::Ack& ack_out)
  {
    LibXR::Debug::SwdProtocol::Response swd_resp = {};
    const auto req = LibXR::Debug::SwdProtocol::make_dp_read_req(
        LibXR::Debug::SwdProtocol::DpReadReg::RDBUFF);
    const ErrorCode ec = TransferTxnFast(req, swd_resp);
    ack_out = swd_resp.ack;
    if (ec != ErrorCode::OK)
    {
      return ec;
    }
    if (swd_resp.ack != LibXR::Debug::SwdProtocol::Ack::OK || !swd_resp.parity_ok)
    {
      return ErrorCode::FAILED;
    }
    val = swd_resp.rdata;
    return ErrorCode::OK;
  }

  ErrorCode CompletePendingApReadFast(uint32_t& val,
                                      LibXR::Debug::SwdProtocol::Ack& ack_out)
  {
    return DpReadRdbuffFast(val, ack_out);
  }

  ErrorCode CheckPostedApWriteFast(LibXR::Debug::SwdProtocol::Ack& ack_out)
  {
    LibXR::Debug::SwdProtocol::Response swd_resp = {};
    const auto req = LibXR::Debug::SwdProtocol::make_dp_read_req(
        LibXR::Debug::SwdProtocol::DpReadReg::CTRL_STAT);
    const ErrorCode ec = TransferTxnFast(req, swd_resp);
    ack_out = swd_resp.ack;
    if (ec != ErrorCode::OK)
    {
      return ec;
    }
    if (swd_resp.ack != LibXR::Debug::SwdProtocol::Ack::OK || !swd_resp.parity_ok)
    {
      return ErrorCode::FAILED;
    }
    return ErrorCode::OK;
  }

  ErrorCode ApReadPostedFast(uint8_t addr2b, uint32_t& posted,
                             LibXR::Debug::SwdProtocol::Ack& ack_out)
  {
    LibXR::Debug::SwdProtocol::Response swd_resp = {};
    const auto req = LibXR::Debug::SwdProtocol::make_ap_read_req(addr2b);
    const ErrorCode ec = TransferTxnFast(req, swd_resp);
    ack_out = swd_resp.ack;
    if (ec != ErrorCode::OK)
    {
      return ec;
    }
    if (swd_resp.ack != LibXR::Debug::SwdProtocol::Ack::OK || !swd_resp.parity_ok)
    {
      return ErrorCode::FAILED;
    }
    posted = swd_resp.rdata;
    return ErrorCode::OK;
  }

  void SetTransferAbortFlag(bool on) { dap_state_.transfer_abort = on; }

 private:
  // ============================================================================
  // DAP_Transfer / DAP_TransferBlock
  // ============================================================================

  // The following long functions are kept as-is; API docs live at the header boundary.

  ErrorCode HandleTransfer(bool in_isr, const uint8_t* req, uint16_t req_len,
                           uint8_t* resp, uint16_t resp_cap, uint16_t& out_len)
  {
    out_len = 0u;
    if (!req || !resp || resp_cap < 3u)
    {
      return ErrorCode::ARG_ERR;
    }

    if constexpr (JTAG_ENABLED)
    {
      if (dap_state_.debug_port == LibXR::USB::DapLinkV2Def::DebugPort::JTAG)
      {
        return HandleJtagTransfer(in_isr, req, req_len, resp, resp_cap, out_len);
      }
    }

    resp[0] = ToU8(LibXR::USB::DapLinkV2Def::CommandId::TRANSFER);
    resp[1] = 0u;  // response_count = #successful transfers
    resp[2] = 0u;  // response_value (ACK bits / ERROR / MISMATCH)
    uint16_t resp_off = 3u;

    // Req: [0]=CMD [1]=DAP index [2]=count [3..]=transfers...
    if (req_len < 3u)
    {
      resp[2] = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
      out_len = 3u;
      return ErrorCode::ARG_ERR;
    }

    if (dap_state_.transfer_abort)
    {
      dap_state_.transfer_abort = false;
      resp[2] = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
      out_len = 3u;
      return ErrorCode::OK;
    }

    const uint8_t COUNT = req[2];
    uint16_t req_off = 3u;

    auto push_u32 = [&](uint32_t v) -> bool
    {
      if (resp_off + 4u > resp_cap)
      {
        return false;
      }
      Memory::FastCopy(&resp[resp_off], &v, sizeof(v));
      resp_off = static_cast<uint16_t>(resp_off + 4u);
      return true;
    };

    auto push_timestamp = [&]() -> bool
    {
      const uint32_t T = static_cast<uint32_t>(LibXR::Timebase::GetMicroseconds());
      return push_u32(T);
    };

    auto ensure_space = [&](uint16_t bytes) -> bool
    { return (resp_off + bytes) <= resp_cap; };

    auto bytes_for_read = [&](bool need_ts) -> uint16_t
    { return static_cast<uint16_t>(need_ts ? 8u : 4u); };

    uint8_t response_count = 0u;
    // Keep reference behavior: if no transfer executes, response_value remains 0.
    uint8_t response_value = 0u;

    // AP writes are posted by the target. Keep the write batch fast and check
    // it once at a transfer boundary with DP CTRL/STAT, not DP RDBUFF.
    bool pending_ap_write = false;

    // -------- posted-read pipeline state (AP read only) --------
    struct PendingApRead
    {
      bool valid = false;
      bool need_ts = false;  // Whether this AP read transfer needs timestamp.
    } pending;

    auto emit_read_with_ts = [&](bool need_ts, uint32_t data) -> bool
    {
      if (need_ts)
      {
        if (!push_timestamp())
        {
          return false;
        }
      }
      if (!push_u32(data))
      {
        return false;
      }
      response_count++;
      return true;
    };

    // Complete a pending AP read by DP RDBUFF.
    // RDBUFF is the AP read-result pipe outlet, not the AP-write checker.
    auto complete_pending_ap_read = [&]() -> bool
    {
      if (!pending.valid)
      {
        return true;
      }

      if (!ensure_space(bytes_for_read(pending.need_ts)))
      {
        response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
        return false;
      }

      uint32_t rdata = 0u;
      LibXR::Debug::SwdProtocol::Ack ack = LibXR::Debug::SwdProtocol::Ack::PROTOCOL;
      const ErrorCode EC = CompletePendingApReadFast(rdata, ack);

      const uint8_t V = MapAckToDapResp(ack);
      if (V != LibXR::USB::DapLinkV2Def::DAP_TRANSFER_OK)
      {
        response_value = V;
        return false;
      }
      if (EC != ErrorCode::OK)
      {
        response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
        return false;
      }

      if (!emit_read_with_ts(pending.need_ts, rdata))
      {
        response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
        return false;
      }

      pending.valid = false;
      pending.need_ts = false;

      // Successful path keeps OK.
      response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_OK;
      return true;
    };

    auto check_posted_ap_write_if_pending = [&]() -> bool
    {
      if (!pending_ap_write)
      {
        return true;
      }
      LibXR::Debug::SwdProtocol::Ack check_ack = LibXR::Debug::SwdProtocol::Ack::PROTOCOL;
      const ErrorCode check_ec = CheckPostedApWriteFast(check_ack);
      response_value = MapAckToDapResp(check_ack);
      if (response_value != LibXR::USB::DapLinkV2Def::DAP_TRANSFER_OK)
      {
        return false;
      }
      if (check_ec != ErrorCode::OK)
      {
        response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
        return false;
      }
      pending_ap_write = false;
      return true;
    };

    // Complete pending AP read before non-AP operations to keep response order.
    auto complete_pending_ap_read_if_any = [&]() -> bool
    { return pending.valid ? complete_pending_ap_read() : true; };

    auto close_posted_state = [&]() -> bool
    { return complete_pending_ap_read_if_any() && check_posted_ap_write_if_pending(); };

    // -------- main loop --------
    for (uint32_t i = 0; i < COUNT; ++i)
    {
      if (req_off >= req_len)
      {
        response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
        break;
      }

      const uint8_t RQ = req[req_off++];

      const bool AP = LibXR::USB::DapLinkV2Def::req_is_ap(RQ);
      const bool RNW = LibXR::USB::DapLinkV2Def::req_is_read(RQ);
      const uint8_t ADDR2B = LibXR::USB::DapLinkV2Def::req_addr2b(RQ);

      const bool TS = LibXR::USB::DapLinkV2Def::req_need_timestamp(RQ);
      const bool MATCH_VALUE =
          ((RQ & LibXR::USB::DapLinkV2Def::DAP_TRANSFER_MATCH_VALUE) != 0u);
      const bool MATCH_MASK =
          ((RQ & LibXR::USB::DapLinkV2Def::DAP_TRANSFER_MATCH_MASK) != 0u);

      // Spec rule: timestamp cannot be combined with match bits.
      if (TS && (MATCH_VALUE || MATCH_MASK))
      {
        response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
        break;
      }

      LibXR::Debug::SwdProtocol::Ack ack = LibXR::Debug::SwdProtocol::Ack::PROTOCOL;
      ErrorCode ec = ErrorCode::OK;

      if (!RNW)
      {
        // ---------------- WRITE ----------------
        // A write can follow previous AP writes without forcing a recovery.
        // Only close an AP-read pipeline before consuming write data.
        if (!complete_pending_ap_read_if_any())
        {
          break;
        }

        if (req_off + 4u > req_len)
        {
          response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
          break;
        }

        uint32_t wdata = LoadU32Le(&req[req_off]);
        req_off = static_cast<uint16_t>(req_off + 4u);

        if (MATCH_MASK)
        {
          match_mask_ = wdata;
          response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_OK;
          response_count++;  // MATCH_MASK counts as a successful transfer.
          continue;
        }
        if (AP)
        {
          ec = swd_.ApWriteTxn(ADDR2B, wdata, ack);
        }
        else
        {
          ec = swd_.DpWriteTxn(static_cast<LibXR::Debug::SwdProtocol::DpWriteReg>(ADDR2B),
                               wdata, ack);
        }

        response_value = MapAckToDapResp(ack);
        if (response_value != LibXR::USB::DapLinkV2Def::DAP_TRANSFER_OK)
        {
          break;
        }
        if (ec != ErrorCode::OK)
        {
          response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
          break;
        }

        if (AP)
        {
          pending_ap_write = true;
        }

        if (TS)
        {
          if (!push_timestamp())
          {
            response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
            break;
          }
        }

        response_count++;
      }
      else
      {
        // ---------------- READ ----------------

        if (MATCH_VALUE)
        {
          // MATCH_VALUE performs real reads and can observe earlier AP writes,
          // so close both posted read and posted AP-write state first.
          if (!close_posted_state())
          {
            break;
          }

          if (req_off + 4u > req_len)
          {
            response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
            break;
          }

          uint32_t match_val = LoadU32Le(&req[req_off]);
          req_off = static_cast<uint16_t>(req_off + 4u);

          uint32_t rdata = 0u;
          uint32_t retry = dap_state_.transfer_cfg.match_retry;
          bool matched = false;

          while (true)
          {
            if (AP)
            {
              ec = swd_.ApReadTxn(ADDR2B, rdata, ack);  // Includes RDBUFF.
              if (ec == ErrorCode::OK && ack == LibXR::Debug::SwdProtocol::Ack::OK)
              {
                // ApReadTxn completes the AP read pipeline only.
              }
            }
            else
            {
              ec = swd_.DpReadTxn(
                  static_cast<LibXR::Debug::SwdProtocol::DpReadReg>(ADDR2B), rdata, ack);
            }

            response_value = MapAckToDapResp(ack);
            if (response_value != LibXR::USB::DapLinkV2Def::DAP_TRANSFER_OK)
            {
              break;
            }
            if (ec != ErrorCode::OK)
            {
              response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
              break;
            }

            if ((rdata & match_mask_) == (match_val & match_mask_))
            {
              matched = true;
              break;
            }

            if (retry == 0u)
            {
              break;
            }
            --retry;
          }

          if (response_value != LibXR::USB::DapLinkV2Def::DAP_TRANSFER_OK)
          {
            break;
          }

          if (!matched)
          {
            response_value =
                static_cast<uint8_t>(LibXR::USB::DapLinkV2Def::DAP_TRANSFER_OK |
                                     LibXR::USB::DapLinkV2Def::DAP_TRANSFER_MISMATCH);
            // MISMATCH does not increase response_count.
            break;
          }

          response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_OK;
          response_count++;
          continue;
        }

        // ---- Normal read ----
        if (!AP)
        {
          // DP read does not participate in posted pipelines.
          if (!close_posted_state())
          {
            break;
          }

          uint32_t rdata = 0u;
          ec = swd_.DpReadTxn(static_cast<LibXR::Debug::SwdProtocol::DpReadReg>(ADDR2B),
                              rdata, ack);

          response_value = MapAckToDapResp(ack);
          if (response_value != LibXR::USB::DapLinkV2Def::DAP_TRANSFER_OK)
          {
            break;
          }
          if (ec != ErrorCode::OK)
          {
            response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
            break;
          }

          if (!ensure_space(bytes_for_read(TS)))
          {
            response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
            break;
          }

          if (!emit_read_with_ts(TS, rdata))
          {
            response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
            break;
          }

          response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_OK;
          continue;
        }

        // AP reads can observe previous AP writes, so close posted writes
        // before starting or extending the posted-read pipeline.
        if (!check_posted_ap_write_if_pending())
        {
          break;
        }

        // AP normal read: posted-read pipeline.
        if (!pending.valid)
        {
          // First AP read in this contiguous AP-read segment:
          // start one posted AP read and discard returned posted data.
          uint32_t dummy_posted = 0u;
          ec = ApReadPostedFast(ADDR2B, dummy_posted, ack);

          response_value = MapAckToDapResp(ack);
          if (response_value != LibXR::USB::DapLinkV2Def::DAP_TRANSFER_OK)
          {
            break;
          }
          if (ec != ErrorCode::OK)
          {
            response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
            {
              break;
            }
          }

          pending.valid = true;
          pending.need_ts = TS;

          // Transfer is not "complete" yet.
          // Data will be produced by next AP read or tail RDBUFF.
          response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_OK;
        }
        else
        {
          // Pending exists: current AP read returns posted data
          // from the previous AP read.
          if (!ensure_space(bytes_for_read(pending.need_ts)))
          {
            response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
            break;
          }

          uint32_t posted_prev = 0u;
          ec = ApReadPostedFast(ADDR2B, posted_prev, ack);

          const uint8_t CUR_V = MapAckToDapResp(ack);
          if (CUR_V != LibXR::USB::DapLinkV2Def::DAP_TRANSFER_OK || ec != ErrorCode::OK)
          {
            // Current AP read failed: try best effort to complete pending AP read.
            // Otherwise, response_count may miss one and pipeline may remain dirty.
            const uint8_t PROOR_FAIL =
                (CUR_V != LibXR::USB::DapLinkV2Def::DAP_TRANSFER_OK)
                    ? CUR_V
                    : LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;

            if (!complete_pending_ap_read())
            {
              // Pending itself failed: return earlier failure
              // (complete_pending_ap_read already wrote response_value).
              break;
            }

            // Pending completion succeeded: keep current failure.
            response_value = PROOR_FAIL;
            break;
          }

          // Current AP read succeeded: emit pending first (using posted_prev),
          // then mark current read as new pending.
          if (!emit_read_with_ts(pending.need_ts, posted_prev))
          {
            response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
            break;
          }

          pending.valid = true;
          pending.need_ts = TS;

          response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_OK;
        }
      }

      if (dap_state_.transfer_abort)
      {
        dap_state_.transfer_abort = false;
        break;
      }
    }

    // If pending AP read still exists at tail:
    // - when response_value == OK: normal tail by one RDBUFF;
    // - when response_value != OK: try pending completion first; if completion
    //   succeeds, keep original failure; otherwise use pending failure.
    if (pending.valid)
    {
      const uint8_t PRIOR_FAIL = response_value;

      if (!complete_pending_ap_read())
      {
        // Pending failure wins here (response_value already written).
      }
      else
      {
        // Pending completion succeeded: keep original failure
        // (if original was OK, keep OK).
        if (PRIOR_FAIL != 0u && PRIOR_FAIL != LibXR::USB::DapLinkV2Def::DAP_TRANSFER_OK)
        {
          response_value = PRIOR_FAIL;
        }
      }
    }

    if (response_value == LibXR::USB::DapLinkV2Def::DAP_TRANSFER_OK &&
        !check_posted_ap_write_if_pending())
    {
      // response_value is already set by the recovery helper.
    }
    resp[1] = response_count;
    resp[2] = response_value;
    out_len = resp_off;
    return ErrorCode::OK;
  }
  ErrorCode HandleTransferBlock(bool in_isr, const uint8_t* req, uint16_t req_len,
                                uint8_t* resp, uint16_t resp_cap, uint16_t& out_len)
  {
    if constexpr (JTAG_ENABLED)
    {
      if (dap_state_.debug_port == LibXR::USB::DapLinkV2Def::DebugPort::JTAG)
      {
        return HandleJtagTransferBlock(in_isr, req, req_len, resp, resp_cap, out_len);
      }
    }

    // Req:  [0]=0x06 [1]=index [2..3]=count [4]=request [5..]=data(write)
    // Resp: [0]=0x06 [1..2]=done [3]=resp [4..]=data(read)
    if (!resp || resp_cap < 4u)
    {
      out_len = 0u;
      return ErrorCode::NOT_FOUND;
    }

    resp[0] = ToU8(LibXR::USB::DapLinkV2Def::CommandId::TRANSFER_BLOCK);
    resp[1] = 0u;
    resp[2] = 0u;
    resp[3] = 0u;
    out_len = 4u;

    if (!req || req_len < 5u)
    {
      resp[3] = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
      return ErrorCode::ARG_ERR;
    }

    if (dap_state_.transfer_abort)
    {
      dap_state_.transfer_abort = false;
      resp[3] = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
      return ErrorCode::OK;
    }

    uint16_t count = 0u;
    count = LoadU16Le(&req[2]);

    const uint8_t DAP_RQ = req[4];

    // TransferBlock does not support match or timestamp
    if ((DAP_RQ & (LibXR::USB::DapLinkV2Def::DAP_TRANSFER_MATCH_VALUE |
                   LibXR::USB::DapLinkV2Def::DAP_TRANSFER_MATCH_MASK)) != 0u)
    {
      resp[3] = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
      return ErrorCode::NOT_SUPPORT;
    }
    if (LibXR::USB::DapLinkV2Def::req_need_timestamp(DAP_RQ))
    {
      resp[3] = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
      return ErrorCode::NOT_SUPPORT;
    }

    // Count==0: return OK
    if (count == 0u)
    {
      const uint16_t DONE0 = 0u;
      StoreU16Le(&resp[1], DONE0);
      resp[3] = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_OK;
      out_len = 4u;
      return ErrorCode::OK;
    }

    const bool AP = LibXR::USB::DapLinkV2Def::req_is_ap(DAP_RQ);
    const bool RNW = LibXR::USB::DapLinkV2Def::req_is_read(DAP_RQ);
    const uint8_t ADDR2B = LibXR::USB::DapLinkV2Def::req_addr2b(DAP_RQ);

    uint16_t done = 0u;
    uint8_t xresp = 0u;

    uint16_t req_off = 5u;
    uint16_t resp_off = 4u;

    // WRITE path: keep original behavior
    if (!RNW)
    {
      const uint32_t REQ_NEED =
          static_cast<uint32_t>(req_off) + (static_cast<uint32_t>(count) * 4u);
      if (REQ_NEED > req_len)
      {
        xresp |= LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
        StoreU16Le(&resp[1], done);
        resp[3] = xresp;
        out_len = resp_off;
        return ErrorCode::OK;
      }
      if (AP)
      {
        auto swd_req = LibXR::Debug::SwdProtocol::make_ap_write_req(ADDR2B, 0u);
        LibXR::Debug::SwdProtocol::Response swd_resp = {};
        for (uint32_t i = 0; i < count; ++i)
        {
          uint32_t wdata = LoadU32Le(&req[req_off]);
          req_off = static_cast<uint16_t>(req_off + 4u);
          swd_req.wdata = wdata;
          const ErrorCode ec = TransferTxnFast(swd_req, swd_resp);
          xresp = MapAckToDapResp(swd_resp.ack);
          if (swd_resp.ack != LibXR::Debug::SwdProtocol::Ack::OK)
          {
            break;
          }
          if (ec != ErrorCode::OK)
          {
            xresp |= LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
            break;
          }
          done = static_cast<uint16_t>(i + 1u);
        }
        if (xresp == LibXR::USB::DapLinkV2Def::DAP_TRANSFER_OK && done != 0u)
        {
          LibXR::Debug::SwdProtocol::Ack check_ack =
              LibXR::Debug::SwdProtocol::Ack::PROTOCOL;
          const ErrorCode check_ec = CheckPostedApWriteFast(check_ack);
          xresp = MapAckToDapResp(check_ack);
          if (check_ack == LibXR::Debug::SwdProtocol::Ack::OK &&
              check_ec != ErrorCode::OK)
          {
            xresp |= LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
          }
        }
      }
      else
      {
        for (uint32_t i = 0; i < count; ++i)
        {
          LibXR::Debug::SwdProtocol::Ack ack = LibXR::Debug::SwdProtocol::Ack::PROTOCOL;
          ErrorCode ec = ErrorCode::OK;
          uint32_t wdata = LoadU32Le(&req[req_off]);
          req_off = static_cast<uint16_t>(req_off + 4u);
          ec = swd_.DpWriteTxn(static_cast<LibXR::Debug::SwdProtocol::DpWriteReg>(ADDR2B),
                               wdata, ack);
          xresp = MapAckToDapResp(ack);
          if (ack != LibXR::Debug::SwdProtocol::Ack::OK)
          {
            break;
          }
          if (ec != ErrorCode::OK)
          {
            xresp |= LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
            break;
          }
          done = static_cast<uint16_t>(i + 1u);
        }
      }
      StoreU16Le(&resp[1], done);
      resp[3] = xresp;
      out_len = resp_off;
      return ErrorCode::OK;
    }

    // READ path
    if (!AP)
    {
      const uint32_t RESP_NEED =
          static_cast<uint32_t>(resp_off) + (static_cast<uint32_t>(count) * 4u);
      if (RESP_NEED > resp_cap)
      {
        xresp |= LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
        StoreU16Le(&resp[1], done);
        resp[3] = xresp;
        out_len = resp_off;
        return ErrorCode::OK;
      }

      // DP read: keep original behavior
      for (uint32_t i = 0; i < count; ++i)
      {
        LibXR::Debug::SwdProtocol::Ack ack = LibXR::Debug::SwdProtocol::Ack::PROTOCOL;
        ErrorCode ec = ErrorCode::OK;
        uint32_t rdata = 0u;
        ec = swd_.DpReadTxn(static_cast<LibXR::Debug::SwdProtocol::DpReadReg>(ADDR2B),
                            rdata, ack);
        xresp = MapAckToDapResp(ack);
        if (ack != LibXR::Debug::SwdProtocol::Ack::OK)
        {
          break;
        }
        if (ec != ErrorCode::OK)
        {
          xresp |= LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
          break;
        }
        StoreU32Le(&resp[resp_off], rdata);
        resp_off = static_cast<uint16_t>(resp_off + 4u);
        done = static_cast<uint16_t>(i + 1u);
      }
      StoreU16Le(&resp[1], done);
      resp[3] = xresp;
      out_len = resp_off;
      return ErrorCode::OK;
    }

    // AP read: posted-read pipeline
    {
      const uint32_t RESP_NEED =
          static_cast<uint32_t>(resp_off) + (static_cast<uint32_t>(count) * 4u);
      if (RESP_NEED > resp_cap)
      {
        xresp |= LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
        StoreU16Le(&resp[1], done);
        resp[3] = xresp;
        out_len = resp_off;
        return ErrorCode::OK;
      }

      auto ap_read_req = LibXR::Debug::SwdProtocol::make_ap_read_req(ADDR2B);
      LibXR::Debug::SwdProtocol::Response ap_read_resp = {};
      ErrorCode ec = TransferTxnFast(ap_read_req, ap_read_resp);
      xresp = MapAckToDapResp(ap_read_resp.ack);
      if (ap_read_resp.ack != LibXR::Debug::SwdProtocol::Ack::OK)
      {
        goto out_ap_read;  // NOLINT
      }
      if (ec != ErrorCode::OK || !ap_read_resp.parity_ok)
      {
        xresp |= LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
        goto out_ap_read;  // NOLINT
      }

      // i=1..COUNT-1: each AP read returns previous posted data.
      for (uint32_t i = 1; i < count; ++i)
      {
        ec = TransferTxnFast(ap_read_req, ap_read_resp);
        const uint8_t CUR = MapAckToDapResp(ap_read_resp.ack);

        if (ap_read_resp.ack != LibXR::Debug::SwdProtocol::Ack::OK ||
            ec != ErrorCode::OK || !ap_read_resp.parity_ok)
        {
          // Current AP read failed; try to complete the previous AP read result.
          if (resp_off + 4u <= resp_cap)
          {
            uint32_t rdbuff_data = 0u;
            LibXR::Debug::SwdProtocol::Ack rdbuff_ack =
                LibXR::Debug::SwdProtocol::Ack::PROTOCOL;
            const ErrorCode ec2 = CompletePendingApReadFast(rdbuff_data, rdbuff_ack);
            const uint8_t v2 = MapAckToDapResp(rdbuff_ack);

            if (v2 == LibXR::USB::DapLinkV2Def::DAP_TRANSFER_OK && ec2 == ErrorCode::OK)
            {
              StoreU32Le(&resp[resp_off], rdbuff_data);
              resp_off = static_cast<uint16_t>(resp_off + 4u);
              done = static_cast<uint16_t>(i);  // done includes transfer [0..i-1]
            }
            else
            {
              xresp = v2;
              if (ec2 != ErrorCode::OK)
              {
                xresp |= LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
              }
              goto out_ap_read;  // NOLINT
            }
          }

          xresp = CUR;
          if (ec != ErrorCode::OK)
          {
            xresp |= LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
          }
          goto out_ap_read;  // NOLINT
        }

        StoreU32Le(&resp[resp_off], ap_read_resp.rdata);
        resp_off = static_cast<uint16_t>(resp_off + 4u);
        done = static_cast<uint16_t>(i);  // completed [0..i-1]
        xresp = CUR;
      }

      // Complete the final posted AP read by DP RDBUFF.
      {
        uint32_t rdbuff_data = 0u;
        LibXR::Debug::SwdProtocol::Ack rdbuff_ack =
            LibXR::Debug::SwdProtocol::Ack::PROTOCOL;
        const ErrorCode ec2 = CompletePendingApReadFast(rdbuff_data, rdbuff_ack);
        const uint8_t v2 = MapAckToDapResp(rdbuff_ack);

        xresp = v2;
        if (v2 != LibXR::USB::DapLinkV2Def::DAP_TRANSFER_OK)
        {
          goto out_ap_read;  // NOLINT
        }
        if (ec2 != ErrorCode::OK)
        {
          xresp |= LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
          goto out_ap_read;  // NOLINT
        }

        StoreU32Le(&resp[resp_off], rdbuff_data);
        resp_off = static_cast<uint16_t>(resp_off + 4u);
        done = count;
      }

    out_ap_read:
      StoreU16Le(&resp[1], done);
      resp[3] = xresp;
      out_len = resp_off;
      return ErrorCode::OK;
    }
  }

  ErrorCode HandleJtagTransfer(bool /*in_isr*/, const uint8_t* req, uint16_t req_len,
                               uint8_t* resp, uint16_t resp_cap, uint16_t& out_len)
  {
    out_len = 0u;
    if (!req || !resp || resp_cap < 3u)
    {
      return ErrorCode::ARG_ERR;
    }

    resp[0] = ToU8(LibXR::USB::DapLinkV2Def::CommandId::TRANSFER);
    resp[1] = 0u;
    resp[2] = 0u;
    uint16_t resp_off = 3u;

    if (req_len < 3u || jtag_ == nullptr || jtag_chain_.count == 0u)
    {
      resp[2] = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
      out_len = 3u;
      return ErrorCode::OK;
    }

    if (dap_state_.transfer_abort)
    {
      dap_state_.transfer_abort = false;
      resp[2] = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
      out_len = 3u;
      return ErrorCode::OK;
    }

    const uint8_t TAP_INDEX = req[1];
    if (TAP_INDEX >= jtag_chain_.count)
    {
      resp[2] = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
      out_len = 3u;
      return ErrorCode::OK;
    }

    jtag_chain_.index = TAP_INDEX;
    LibXR::Debug::JtagProtocol::UpdateChainCache(jtag_chain_);
    LibXR::Debug::JtagDp dp(*jtag_, &jtag_chain_);

    const uint8_t COUNT = req[2];
    uint16_t req_off = 3u;

    auto push_u32 = [&](uint32_t v) -> bool
    {
      if (resp_off + 4u > resp_cap)
      {
        return false;
      }
      Memory::FastCopy(&resp[resp_off], &v, sizeof(v));
      resp_off = static_cast<uint16_t>(resp_off + 4u);
      return true;
    };

    auto push_timestamp = [&]() -> bool
    {
      const uint32_t t = static_cast<uint32_t>(LibXR::Timebase::GetMicroseconds());
      return push_u32(t);
    };

    auto ensure_space = [&](uint16_t bytes) -> bool
    { return (resp_off + bytes) <= resp_cap; };

    auto bytes_for_read = [&](bool need_ts) -> uint16_t
    { return static_cast<uint16_t>(need_ts ? 8u : 4u); };

    auto idle_after_attempt = [&]()
    {
      if (dap_state_.transfer_cfg.idle_cycles != 0u)
      {
        jtag_->IdleClocks(dap_state_.transfer_cfg.idle_cycles);
      }
    };

    auto dp_read_retry = [&](uint8_t addr2b, uint32_t& val,
                             LibXR::Debug::JtagProtocol::Ack& ack) -> ErrorCode
    {
      uint32_t retry = dap_state_.transfer_cfg.retry_count;
      while (true)
      {
        const ErrorCode ec = dp.DpReadTxn(addr2b, val, ack);
        idle_after_attempt();
        if (ack != LibXR::Debug::JtagProtocol::Ack::WAIT || retry == 0u ||
            dap_state_.transfer_abort)
        {
          return ec;
        }
        --retry;
      }
    };

    auto dp_write_retry = [&](uint8_t addr2b, uint32_t val,
                              LibXR::Debug::JtagProtocol::Ack& ack) -> ErrorCode
    {
      uint32_t retry = dap_state_.transfer_cfg.retry_count;
      while (true)
      {
        const ErrorCode ec = dp.DpWriteTxn(addr2b, val, ack);
        idle_after_attempt();
        if (ack != LibXR::Debug::JtagProtocol::Ack::WAIT || retry == 0u ||
            dap_state_.transfer_abort)
        {
          return ec;
        }
        --retry;
      }
    };

    auto ap_read_retry = [&](uint8_t addr2b, uint32_t& val,
                             LibXR::Debug::JtagProtocol::Ack& ack) -> ErrorCode
    {
      uint32_t retry = dap_state_.transfer_cfg.retry_count;
      while (true)
      {
        const ErrorCode ec = dp.ApReadTxn(addr2b, val, ack);
        idle_after_attempt();
        if (ack != LibXR::Debug::JtagProtocol::Ack::WAIT || retry == 0u ||
            dap_state_.transfer_abort)
        {
          return ec;
        }
        --retry;
      }
    };

    auto ap_write_retry = [&](uint8_t addr2b, uint32_t val,
                              LibXR::Debug::JtagProtocol::Ack& ack) -> ErrorCode
    {
      uint32_t retry = dap_state_.transfer_cfg.retry_count;
      while (true)
      {
        const ErrorCode ec = dp.ApWriteTxn(addr2b, val, ack);
        idle_after_attempt();
        if (ack != LibXR::Debug::JtagProtocol::Ack::WAIT || retry == 0u ||
            dap_state_.transfer_abort)
        {
          return ec;
        }
        --retry;
      }
    };

    auto check_posted_ap_write_jtag =
        [&](LibXR::Debug::JtagProtocol::Ack& ack) -> ErrorCode
    {
      uint32_t ctrl_stat = 0u;
      return dp_read_retry(
          static_cast<uint8_t>(LibXR::Debug::SwdProtocol::DpReadReg::CTRL_STAT),
          ctrl_stat, ack);
    };

    uint8_t response_count = 0u;
    uint8_t response_value = 0u;
    bool pending_ap_write = false;

    auto emit_read_with_ts = [&](bool need_ts, uint32_t data) -> bool
    {
      if (need_ts)
      {
        if (!push_timestamp())
        {
          return false;
        }
      }
      if (!push_u32(data))
      {
        return false;
      }
      response_count++;
      return true;
    };

    auto check_posted_ap_write_if_pending = [&]() -> bool
    {
      if (!pending_ap_write)
      {
        return true;
      }

      LibXR::Debug::JtagProtocol::Ack ack = LibXR::Debug::JtagProtocol::Ack::PROTOCOL;
      const ErrorCode ec = check_posted_ap_write_jtag(ack);
      const uint8_t v = MapAckToDapResp(ack);

      if (v != LibXR::USB::DapLinkV2Def::DAP_TRANSFER_OK)
      {
        response_value = v;
        return false;
      }
      if (ec != ErrorCode::OK)
      {
        response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
        return false;
      }

      pending_ap_write = false;
      return true;
    };

    auto should_check_pending_ap_write_at_tail = [&]() -> bool
    {
      return response_value == LibXR::USB::DapLinkV2Def::DAP_TRANSFER_OK ||
             response_value ==
                 static_cast<uint8_t>(LibXR::USB::DapLinkV2Def::DAP_TRANSFER_OK |
                                      LibXR::USB::DapLinkV2Def::DAP_TRANSFER_MISMATCH);
    };

    for (uint32_t i = 0; i < COUNT; ++i)
    {
      if (req_off >= req_len)
      {
        response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
        break;
      }

      const uint8_t RQ = req[req_off++];
      const bool AP = LibXR::USB::DapLinkV2Def::req_is_ap(RQ);
      const bool RNW = LibXR::USB::DapLinkV2Def::req_is_read(RQ);
      const uint8_t ADDR2B = LibXR::USB::DapLinkV2Def::req_addr2b(RQ);
      const bool TS = LibXR::USB::DapLinkV2Def::req_need_timestamp(RQ);
      const bool MATCH_VALUE =
          ((RQ & LibXR::USB::DapLinkV2Def::DAP_TRANSFER_MATCH_VALUE) != 0u);
      const bool MATCH_MASK =
          ((RQ & LibXR::USB::DapLinkV2Def::DAP_TRANSFER_MATCH_MASK) != 0u);

      if (TS && (MATCH_VALUE || MATCH_MASK))
      {
        response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
        break;
      }

      LibXR::Debug::JtagProtocol::Ack ack = LibXR::Debug::JtagProtocol::Ack::PROTOCOL;
      ErrorCode ec = ErrorCode::OK;

      if (!RNW)
      {
        if (req_off + 4u > req_len)
        {
          response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
          break;
        }

        uint32_t wdata = 0u;
        Memory::FastCopy(&wdata, &req[req_off], sizeof(wdata));
        req_off = static_cast<uint16_t>(req_off + 4u);

        if (MATCH_MASK)
        {
          match_mask_ = wdata;
          response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_OK;
          response_count++;
          continue;
        }

        if (AP)
        {
          ec = ap_write_retry(ADDR2B, wdata, ack);
        }
        else
        {
          ec = dp_write_retry(ADDR2B, wdata, ack);
        }

        response_value = MapAckToDapResp(ack);
        if (response_value != LibXR::USB::DapLinkV2Def::DAP_TRANSFER_OK)
        {
          break;
        }
        if (ec != ErrorCode::OK)
        {
          response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
          break;
        }

        if (TS)
        {
          if (!push_timestamp())
          {
            response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
            break;
          }
        }

        response_count++;
        if (AP)
        {
          pending_ap_write = true;
        }
      }
      else
      {
        if (MATCH_VALUE)
        {
          if (!check_posted_ap_write_if_pending())
          {
            break;
          }

          if (req_off + 4u > req_len)
          {
            response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
            break;
          }

          uint32_t match_val = 0u;
          Memory::FastCopy(&match_val, &req[req_off], sizeof(match_val));
          req_off = static_cast<uint16_t>(req_off + 4u);

          uint32_t rdata = 0u;
          uint32_t retry = dap_state_.transfer_cfg.match_retry;
          bool matched = false;

          while (true)
          {
            if (AP)
            {
              ec = ap_read_retry(ADDR2B, rdata, ack);
            }
            else
            {
              ec = dp_read_retry(ADDR2B, rdata, ack);
            }

            response_value = MapAckToDapResp(ack);
            if (response_value != LibXR::USB::DapLinkV2Def::DAP_TRANSFER_OK)
            {
              break;
            }
            if (ec != ErrorCode::OK)
            {
              response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
              break;
            }

            if ((rdata & match_mask_) == (match_val & match_mask_))
            {
              matched = true;
              break;
            }

            if (retry == 0u)
            {
              break;
            }
            --retry;
          }

          if (response_value != LibXR::USB::DapLinkV2Def::DAP_TRANSFER_OK)
          {
            break;
          }

          if (!matched)
          {
            response_value =
                static_cast<uint8_t>(LibXR::USB::DapLinkV2Def::DAP_TRANSFER_OK |
                                     LibXR::USB::DapLinkV2Def::DAP_TRANSFER_MISMATCH);
            break;
          }

          response_count++;
          continue;
        }

        if (!AP)
        {
          if (!check_posted_ap_write_if_pending())
          {
            break;
          }

          uint32_t rdata = 0u;
          ec = dp_read_retry(ADDR2B, rdata, ack);

          response_value = MapAckToDapResp(ack);
          if (response_value != LibXR::USB::DapLinkV2Def::DAP_TRANSFER_OK)
          {
            break;
          }
          if (ec != ErrorCode::OK)
          {
            response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
            break;
          }

          if (!ensure_space(bytes_for_read(TS)))
          {
            response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
            break;
          }

          if (!emit_read_with_ts(TS, rdata))
          {
            response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
            break;
          }

          response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_OK;
          continue;
        }

        uint32_t rdata = 0u;
        if (!check_posted_ap_write_if_pending())
        {
          break;
        }

        ec = ap_read_retry(ADDR2B, rdata, ack);

        response_value = MapAckToDapResp(ack);
        if (response_value != LibXR::USB::DapLinkV2Def::DAP_TRANSFER_OK)
        {
          break;
        }
        if (ec != ErrorCode::OK)
        {
          response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
          break;
        }

        if (!ensure_space(bytes_for_read(TS)))
        {
          response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
          break;
        }

        if (!emit_read_with_ts(TS, rdata))
        {
          response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
          break;
        }

        response_value = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_OK;
      }

      if (dap_state_.transfer_abort)
      {
        dap_state_.transfer_abort = false;
        break;
      }
    }

    if (pending_ap_write && should_check_pending_ap_write_at_tail())
    {
      const uint8_t PRIOR_RESPONSE = response_value;
      if (check_posted_ap_write_if_pending())
      {
        response_value = PRIOR_RESPONSE;
      }
    }

    resp[1] = response_count;
    resp[2] = response_value;
    out_len = resp_off;
    return ErrorCode::OK;
  }

  ErrorCode HandleJtagTransferBlock(bool /*in_isr*/, const uint8_t* req, uint16_t req_len,
                                    uint8_t* resp, uint16_t resp_cap, uint16_t& out_len)
  {
    if (!req || !resp || resp_cap < 4u)
    {
      out_len = 0u;
      return ErrorCode::ARG_ERR;
    }

    resp[0] = ToU8(LibXR::USB::DapLinkV2Def::CommandId::TRANSFER_BLOCK);
    resp[1] = 0u;
    resp[2] = 0u;
    resp[3] = 0u;
    out_len = 4u;

    if (req_len < 5u || jtag_ == nullptr || jtag_chain_.count == 0u)
    {
      resp[3] = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
      return ErrorCode::OK;
    }

    if (dap_state_.transfer_abort)
    {
      dap_state_.transfer_abort = false;
      resp[3] = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
      return ErrorCode::OK;
    }

    const uint8_t TAP_INDEX = req[1];
    if (TAP_INDEX >= jtag_chain_.count)
    {
      resp[3] = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
      return ErrorCode::OK;
    }

    jtag_chain_.index = TAP_INDEX;
    LibXR::Debug::JtagProtocol::UpdateChainCache(jtag_chain_);

    uint16_t count = 0u;
    Memory::FastCopy(&count, &req[2], sizeof(count));
    const uint8_t DAP_RQ = req[4];

    if ((DAP_RQ & (LibXR::USB::DapLinkV2Def::DAP_TRANSFER_MATCH_VALUE |
                   LibXR::USB::DapLinkV2Def::DAP_TRANSFER_MATCH_MASK)) != 0u)
    {
      resp[3] = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
      return ErrorCode::NOT_SUPPORT;
    }
    if (LibXR::USB::DapLinkV2Def::req_need_timestamp(DAP_RQ))
    {
      resp[3] = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
      return ErrorCode::NOT_SUPPORT;
    }

    if (count == 0u)
    {
      const uint16_t DONE0 = 0u;
      Memory::FastCopy(&resp[1], &DONE0, sizeof(DONE0));
      resp[3] = LibXR::USB::DapLinkV2Def::DAP_TRANSFER_OK;
      out_len = 4u;
      return ErrorCode::OK;
    }

    LibXR::Debug::JtagDp dp(*jtag_, &jtag_chain_);
    auto idle_after_attempt = [&]()
    {
      if (dap_state_.transfer_cfg.idle_cycles != 0u)
      {
        jtag_->IdleClocks(dap_state_.transfer_cfg.idle_cycles);
      }
    };

    auto dp_read_retry = [&](uint8_t addr2b, uint32_t& val,
                             LibXR::Debug::JtagProtocol::Ack& ack) -> ErrorCode
    {
      uint32_t retry = dap_state_.transfer_cfg.retry_count;
      while (true)
      {
        const ErrorCode ec = dp.DpReadTxn(addr2b, val, ack);
        idle_after_attempt();
        if (ack != LibXR::Debug::JtagProtocol::Ack::WAIT || retry == 0u ||
            dap_state_.transfer_abort)
        {
          return ec;
        }
        --retry;
      }
    };

    auto dp_write_retry = [&](uint8_t addr2b, uint32_t val,
                              LibXR::Debug::JtagProtocol::Ack& ack) -> ErrorCode
    {
      uint32_t retry = dap_state_.transfer_cfg.retry_count;
      while (true)
      {
        const ErrorCode ec = dp.DpWriteTxn(addr2b, val, ack);
        idle_after_attempt();
        if (ack != LibXR::Debug::JtagProtocol::Ack::WAIT || retry == 0u ||
            dap_state_.transfer_abort)
        {
          return ec;
        }
        --retry;
      }
    };

    auto ap_read_retry = [&](uint8_t addr2b, uint32_t& val,
                             LibXR::Debug::JtagProtocol::Ack& ack) -> ErrorCode
    {
      uint32_t retry = dap_state_.transfer_cfg.retry_count;
      while (true)
      {
        const ErrorCode ec = dp.ApReadTxn(addr2b, val, ack);
        idle_after_attempt();
        if (ack != LibXR::Debug::JtagProtocol::Ack::WAIT || retry == 0u ||
            dap_state_.transfer_abort)
        {
          return ec;
        }
        --retry;
      }
    };

    auto ap_write_retry = [&](uint8_t addr2b, uint32_t val,
                              LibXR::Debug::JtagProtocol::Ack& ack) -> ErrorCode
    {
      uint32_t retry = dap_state_.transfer_cfg.retry_count;
      while (true)
      {
        const ErrorCode ec = dp.ApWriteTxn(addr2b, val, ack);
        idle_after_attempt();
        if (ack != LibXR::Debug::JtagProtocol::Ack::WAIT || retry == 0u ||
            dap_state_.transfer_abort)
        {
          return ec;
        }
        --retry;
      }
    };

    auto check_posted_ap_write_jtag =
        [&](LibXR::Debug::JtagProtocol::Ack& ack) -> ErrorCode
    {
      uint32_t ctrl_stat = 0u;
      return dp_read_retry(
          static_cast<uint8_t>(LibXR::Debug::SwdProtocol::DpReadReg::CTRL_STAT),
          ctrl_stat, ack);
    };

    const bool AP = LibXR::USB::DapLinkV2Def::req_is_ap(DAP_RQ);
    const bool RNW = LibXR::USB::DapLinkV2Def::req_is_read(DAP_RQ);
    const uint8_t ADDR2B = LibXR::USB::DapLinkV2Def::req_addr2b(DAP_RQ);

    uint16_t done = 0u;
    uint8_t xresp = 0u;
    uint16_t req_off = 5u;
    uint16_t resp_off = 4u;

    if (!RNW)
    {
      for (uint32_t i = 0; i < count; ++i)
      {
        if (req_off + 4u > req_len)
        {
          xresp |= LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
          break;
        }

        uint32_t wdata = 0u;
        Memory::FastCopy(&wdata, &req[req_off], sizeof(wdata));
        req_off = static_cast<uint16_t>(req_off + 4u);

        LibXR::Debug::JtagProtocol::Ack ack = LibXR::Debug::JtagProtocol::Ack::PROTOCOL;
        ErrorCode ec =
            AP ? ap_write_retry(ADDR2B, wdata, ack) : dp_write_retry(ADDR2B, wdata, ack);

        xresp = MapAckToDapResp(ack);
        if (xresp == 0u)
        {
          xresp |= LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
          break;
        }

        if (ack != LibXR::Debug::JtagProtocol::Ack::OK)
        {
          break;
        }

        if (ec != ErrorCode::OK)
        {
          xresp |= LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
          break;
        }

        done = static_cast<uint16_t>(i + 1u);
      }

      if (AP && xresp == LibXR::USB::DapLinkV2Def::DAP_TRANSFER_OK && done > 0u)
      {
        LibXR::Debug::JtagProtocol::Ack ack = LibXR::Debug::JtagProtocol::Ack::PROTOCOL;
        const ErrorCode ec = check_posted_ap_write_jtag(ack);
        const uint8_t v = MapAckToDapResp(ack);
        if (v != LibXR::USB::DapLinkV2Def::DAP_TRANSFER_OK)
        {
          xresp = v;
        }
        else if (ec != ErrorCode::OK)
        {
          xresp |= LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
        }
      }

      Memory::FastCopy(&resp[1], &done, sizeof(done));
      resp[3] = xresp;
      out_len = resp_off;
      return ErrorCode::OK;
    }

    for (uint32_t i = 0; i < count; ++i)
    {
      if (resp_off + 4u > resp_cap)
      {
        xresp |= LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
        break;
      }

      LibXR::Debug::JtagProtocol::Ack ack = LibXR::Debug::JtagProtocol::Ack::PROTOCOL;
      ErrorCode ec = ErrorCode::OK;
      uint32_t rdata = 0u;

      if (AP)
      {
        ec = ap_read_retry(ADDR2B, rdata, ack);
      }
      else
      {
        ec = dp_read_retry(ADDR2B, rdata, ack);
      }

      xresp = MapAckToDapResp(ack);
      if (xresp == 0u)
      {
        xresp |= LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
        break;
      }

      if (ack != LibXR::Debug::JtagProtocol::Ack::OK)
      {
        break;
      }

      if (ec != ErrorCode::OK)
      {
        xresp |= LibXR::USB::DapLinkV2Def::DAP_TRANSFER_ERROR;
        break;
      }

      Memory::FastCopy(&resp[resp_off], &rdata, sizeof(rdata));
      resp_off = static_cast<uint16_t>(resp_off + 4u);
      done = static_cast<uint16_t>(i + 1u);
    }

    Memory::FastCopy(&resp[1], &done, sizeof(done));
    resp[3] = xresp;
    out_len = resp_off;
    return ErrorCode::OK;
  }

  ErrorCode HandleJtagWriteAbort(bool /*in_isr*/, const uint8_t* req, uint16_t req_len,
                                 uint8_t* resp, uint16_t resp_cap, uint16_t& out_len)
  {
    if (!req || !resp || resp_cap < 2u)
    {
      out_len = 0u;
      return ErrorCode::ARG_ERR;
    }

    resp[0] = ToU8(LibXR::USB::DapLinkV2Def::CommandId::WRITE_ABORT);
    resp[1] = DAP_ERROR;
    out_len = 2u;

    if (req_len < 6u || jtag_ == nullptr || jtag_chain_.count == 0u)
    {
      return ErrorCode::ARG_ERR;
    }

    const uint8_t TAP_INDEX = req[1];
    if (TAP_INDEX >= jtag_chain_.count)
    {
      return ErrorCode::OK;
    }

    jtag_chain_.index = TAP_INDEX;
    LibXR::Debug::JtagProtocol::UpdateChainCache(jtag_chain_);

    uint32_t flags = 0u;
    Memory::FastCopy(&flags, &req[2], sizeof(flags));

    LibXR::Debug::JtagDp dp(*jtag_, &jtag_chain_);
    LibXR::Debug::JtagProtocol::Ack ack = LibXR::Debug::JtagProtocol::Ack::PROTOCOL;
    const ErrorCode ec = dp.WriteAbort(flags, ack);
    if (ec == ErrorCode::OK && ack == LibXR::Debug::JtagProtocol::Ack::OK)
    {
      resp[1] = DAP_OK;
    }
    return ErrorCode::OK;
  }

 private:
  // ============================================================================
  // Reset helpers
  // ============================================================================

  void DriveReset(bool release)
  {
    last_nreset_level_high_ = release;

    // Update shadow regardless of whether GPIO exists
    if (release)
    {
      swj_shadow_ |= LibXR::USB::DapLinkV2Def::DAP_SWJ_NRESET;
    }
    else
    {
      swj_shadow_ = static_cast<uint8_t>(
          swj_shadow_ & static_cast<uint8_t>(~LibXR::USB::DapLinkV2Def::DAP_SWJ_NRESET));
    }

    if (nreset_gpio_ != nullptr)
    {
      (void)nreset_gpio_->Write(release);
    }
  }

  void ResetJtagChainState()
  {
    if constexpr (JTAG_ENABLED)
    {
      jtag_chain_.ir_length = jtag_ir_len_;
      jtag_chain_.count = 0u;
      jtag_chain_.index = 0u;
      jtag_chain_.ir_before_bits_len = 0u;
      jtag_chain_.ir_after_bits_len = 0u;
      jtag_chain_.dr_before_bits_len = 0u;
      jtag_chain_.dr_after_bits_len = 0u;
      Memory::FastSet(jtag_ir_len_, 0, sizeof(jtag_ir_len_));
    }
  }

  void DelayUsIfAllowed(bool /*in_isr*/, uint32_t us)
  {
    LibXR::Timebase::DelayMicroseconds(us);
  }

 private:
  static constexpr uint16_t DEFAULT_DAP_PACKET_SIZE = DefaultDapPacketSize;
  static constexpr uint8_t PACKET_COUNT_ADVERTISED = AdvertisedPacketCount;
  static constexpr uint8_t PACKET_COUNT_EFFECTIVE =
      (PACKET_COUNT_ADVERTISED > 4u) ? 4u : PACKET_COUNT_ADVERTISED;
  static constexpr uint8_t MAX_OUTSTANDING_RESPONSES = PACKET_COUNT_EFFECTIVE;
  // OpenOCD 0.12 computes pending_queue_len = (packet_size - 4) / 5 for CMD_DAP_TFER.
  // CMD_DAP_TFER transfer_count is u8, so cap host-visible packet size to avoid
  // transfer_count > 255 and "expected X, got Y" mismatch.
  static constexpr uint16_t OPENOCD_SAFE_DAP_PACKET_SIZE =
      static_cast<uint16_t>((255u * 5u) + 4u);
  static constexpr uint8_t JTAG_MAX_DEVICES = 16u;
  // Host-visible CMSIS-DAP packet size; endpoint MPS limits are handled by
  // TransferMultiBulk segmentation/reassembly.
  static constexpr uint16_t MAX_DAP_PACKET_SIZE = MaxDapPacketSize;
  static constexpr uint16_t QUEUED_REQ_BUFFER_SIZE = QueuedRequestBufferSize;
  static constexpr uint16_t QUEUED_CMD_COUNT_MAX = QueuedCommandCountMax;
  static constexpr uint16_t RESP_SLOT_SIZE = MAX_DAP_PACKET_SIZE;
  static constexpr uint8_t RESP_QUEUE_DEPTH = PACKET_COUNT_EFFECTIVE;
  static constexpr uint16_t QUEUED_REQ_BUFFER_ALLOC_SIZE = QUEUED_REQ_BUFFER_SIZE;
  static_assert(PACKET_COUNT_ADVERTISED > 0u, "PACKET_COUNT_ADVERTISED must be > 0");
  static_assert(PACKET_COUNT_EFFECTIVE > 0u, "PACKET_COUNT_EFFECTIVE must be > 0");
  static_assert(MAX_DAP_PACKET_SIZE >= DEFAULT_DAP_PACKET_SIZE,
                "MAX_DAP_PACKET_SIZE must be >= DEFAULT_DAP_PACKET_SIZE");
  static_assert(MAX_DAP_PACKET_SIZE <= OPENOCD_SAFE_DAP_PACKET_SIZE,
                "MAX_DAP_PACKET_SIZE exceeds OpenOCD-safe limit");
  static_assert(((OPENOCD_SAFE_DAP_PACKET_SIZE - 4u) / 5u) <= 255u,
                "OPENOCD_SAFE_DAP_PACKET_SIZE too large for CMD_DAP_TFER u8 count");
  static_assert(QUEUED_REQ_BUFFER_SIZE > 0u, "QUEUED_REQ_BUFFER_SIZE must be > 0");
  static_assert(QUEUED_CMD_COUNT_MAX > 0u, "QUEUED_CMD_COUNT_MAX must be > 0");
  static_assert(RESP_SLOT_SIZE >= DEFAULT_DAP_PACKET_SIZE,
                "RESP_SLOT_SIZE must cover FS bulk packet size");
  static_assert((RESP_QUEUE_DEPTH & (RESP_QUEUE_DEPTH - 1u)) == 0u,
                "Response queue depth must be power-of-two");

  static constexpr uint8_t WINUSB_VENDOR_CODE = 0x20;  ///< WinUSB vendor code

  // REG_MULTI_SZ: "<GUID>\0\0" (UTF-16LE). GUID_STR_UTF16_BYTES should already include
  // the first UTF-16 NUL.
  static constexpr uint16_t GUID_MULTI_SZ_UTF_16_BYTES =
      static_cast<uint16_t>(LibXR::USB::WinUsbMsOs20::GUID_STR_UTF16_BYTES +
                            2);  ///< extra UTF-16 NUL for REG_MULTI_SZ end

  struct ResponseSlot
  {
    uint16_t len = 0u;
    uint8_t payload[RESP_SLOT_SIZE] = {};
  };

  ResponseSlot resp_q_[RESP_QUEUE_DEPTH] = {};
  uint8_t resp_q_head_ = 0u;
  uint8_t resp_q_tail_ = 0u;
  uint8_t resp_q_count_ = 0u;
  bool deferred_in_resp_valid_ = false;
  uint16_t deferred_in_resp_len_ = 0u;

  uint8_t queued_request_buffer_[QUEUED_REQ_BUFFER_ALLOC_SIZE] = {};
  uint16_t queued_request_length_ = 0u;
  uint16_t queued_command_count_ = 0u;

  uint8_t out_req_multi_storage_[MAX_DAP_PACKET_SIZE] = {};
  LibXR::RawData out_req_multi_buf_{out_req_multi_storage_, DEFAULT_DAP_PACKET_SIZE};
  uint8_t in_tx_multi_storage_[MAX_DAP_PACKET_SIZE] = {};
  LibXR::RawData in_tx_multi_buf_{in_tx_multi_storage_, DEFAULT_DAP_PACKET_SIZE};

  LIBXR_PACKED_BEGIN
  /**
   * @brief MS OS 2.0 descriptor set layout
   */
  struct WinUsbMsOs20DescSet
  {
    LibXR::USB::WinUsbMsOs20::MsOs20SetHeader set;                  ///< Set header
    LibXR::USB::WinUsbMsOs20::MsOs20SubsetHeaderConfiguration cfg;  ///< Config subset
    LibXR::USB::WinUsbMsOs20::MsOs20SubsetHeaderFunction func;      ///< Function subset
    LibXR::USB::WinUsbMsOs20::MsOs20FeatureCompatibleId compat;  ///< CompatibleId feature

    /**
     * @brief DeviceInterfaceGUIDs registry property
     */
    struct RegProp
    {
      LibXR::USB::WinUsbMsOs20::MsOs20FeatureRegPropertyHeader
          header;  ///< RegProperty header
      uint8_t name[LibXR::USB::WinUsbMsOs20::
                       PROP_NAME_DEVICE_INTERFACE_GUIDS_BYTES];  ///< Property name /
                                                                 ///< Property name
      uint16_t wPropertyDataLength;                              ///< UTF-16 byte length
      uint8_t data[GUID_MULTI_SZ_UTF_16_BYTES];                  ///< REG_MULTI_SZ data
    } prop;
  } winusb_msos20_{};
  LIBXR_PACKED_END

 private:
  SwdPort& swd_;  ///< SWD link

  LibXR::Debug::Jtag* jtag_ = nullptr;                    ///< JTAG link
  LibXR::Debug::JtagProtocol::ChainConfig jtag_chain_{};  ///< JTAG chain state
  uint8_t jtag_ir_len_[JTAG_MAX_DEVICES] = {};            ///< JTAG IR lengths

  LibXR::GPIO* nreset_gpio_ = nullptr;  ///< Optional nRESET GPIO

  uint8_t swj_shadow_ =
      static_cast<uint8_t>(DapLinkV2Def::DAP_SWJ_SWDIO_TMS |
                           DapLinkV2Def::DAP_SWJ_NRESET);  ///< Shadow SWJ pin levels

  bool last_nreset_level_high_ = true;  ///< Last nRESET level (high = release)

  LibXR::USB::DapLinkV2Def::State dap_state_{};  ///< DAP state
  InfoStrings info_{"XRobot", "DAPLinkV2", "00000001", "2.0.0", "XRUSB",
                    "XRDAP",  "XRobot",    "DAP_DEMO", "0.1.0"};  ///< Info strings

  uint32_t swj_clock_hz_ = 1000000u;  ///< SWJ clock (Hz)

  const char* interface_string_ = nullptr;  ///< Interface string

  Endpoint::EPNumber data_in_ep_num_;   ///< Bulk IN EP number
  Endpoint::EPNumber data_out_ep_num_;  ///< Bulk OUT EP number

  Endpoint* ep_data_in_ = nullptr;   ///< Bulk IN endpoint
  Endpoint* ep_data_out_ = nullptr;  ///< Bulk OUT endpoint

  bool inited_ = false;        ///< Initialized flag
  uint8_t interface_num_ = 0;  ///< Interface number

  LIBXR_PACKED_BEGIN
  /**
   * @brief Descriptor block (Interface + 2x Endpoint)
   *
   */
  struct DapLinkV2DescBlock
  {
    InterfaceDescriptor intf;   ///< Interface descriptor
    EndpointDescriptor ep_out;  ///< OUT endpoint descriptor
    EndpointDescriptor ep_in;   ///< IN endpoint descriptor
  } desc_block_{};
  LIBXR_PACKED_END

 private:
  LibXR::USB::WinUsbMsOs20::MsOs20BosCapability winusb_msos20_cap_{
      LibXR::ConstRawData{nullptr, 0}, WINUSB_VENDOR_CODE};  ///< WinUSB BOS capability

  uint32_t match_mask_ = 0xFFFFFFFFu;  ///< Match mask

  LibXR::Callback<LibXR::ConstRawData&> on_data_out_cb_ =
      LibXR::Callback<LibXR::ConstRawData&>::Create(OnDataOutCompleteStatic, this);

  LibXR::Callback<LibXR::ConstRawData&> on_data_in_cb_ =
      LibXR::Callback<LibXR::ConstRawData&>::Create(OnDataInCompleteStatic, this);
};

}  // namespace LibXR::USB
