#pragma once

#include <cstddef>
#include <cstdint>
#include <initializer_list>

#include "hpm_soc.h"
#include "hpm_usb_drv.h"
#include "libxr_def.hpp"
#include "libxr_type.hpp"
#include "usb/core/ep.hpp"
#include "usb/core/ep_pool.hpp"
#include "usb/device/dev_core.hpp"

namespace LibXR
{

class HPMUSBDevice;

struct HPMUSBDtd
{
  volatile uint32_t next;
  volatile uint32_t token;
  volatile uint32_t buffer[USB_SOC_DCD_QHD_BUFFER_COUNT];
  volatile uint16_t expected_bytes;
  volatile uint8_t in_use;
  volatile uint8_t reserved;
};

static_assert(sizeof(HPMUSBDtd) == 32, "HPM USB dTD must be 32 bytes");

struct HPMUSBDqh
{
  volatile uint32_t caps;
  volatile uint32_t qtd_addr;
  volatile HPMUSBDtd qtd_overlay;
  volatile usb_control_request_t setup_request;
  volatile uint32_t attached_buffer;
  HPMUSBDtd* volatile attached_qtd;
  volatile uint8_t reserved[8];
};

static_assert(sizeof(HPMUSBDqh) == 64, "HPM USB dQH must be 64 bytes");

struct HPMUSBDcdData
{
  HPMUSBDqh qhd[USB_SOS_DCD_MAX_QHD_COUNT];
  HPMUSBDtd qtd[USB_SOC_DCD_MAX_QTD_COUNT];
};

class HPMUSBEndpoint : public USB::Endpoint
{
 public:
  HPMUSBEndpoint(HPMUSBDevice& device, EPNumber ep_num, Direction dir,
                 LibXR::RawData buffer);

  void Configure(const Config& cfg) override;
  void Close() override;
  ErrorCode Transfer(size_t size) override;

  ErrorCode Stall() override;
  ErrorCode ClearStall() override;

  size_t MaxTransferSize() const override;

  void TransferComplete(bool in_isr);

 private:
  HPMUSBDevice& device_;
  size_t last_transfer_size_ = 0;
};

class HPMUSBDevice : public USB::EndpointPool, public USB::DeviceCore
{
 public:
  static constexpr uint8_t MAX_USB_INSTANCES = 1;
  static constexpr uint8_t INVALID_INSTANCE_INDEX = 0xFFu;
  static constexpr uint32_t INVALID_IRQ = 0xFFFFFFFFu;

  struct EPConfig
  {
    LibXR::RawData buffer_out;
    LibXR::RawData buffer_in;
    int8_t is_in = -1;

    explicit EPConfig(LibXR::RawData buffer)
        : buffer_out(buffer), buffer_in(buffer), is_in(-1)
    {
    }

    EPConfig(LibXR::RawData buffer, bool in)
        : buffer_out(buffer), buffer_in(buffer), is_in(in ? 1 : 0)
    {
    }

    EPConfig(LibXR::RawData out, LibXR::RawData in)
        : buffer_out(out), buffer_in(in), is_in(-1)
    {
    }
  };

  struct BoardConfig
  {
    constexpr explicit BoardConfig(
        void (*post_dcd_init_hook)(USB_Type* instance) = nullptr)
        : post_dcd_init(post_dcd_init_hook)
    {
    }

    void (*post_dcd_init)(USB_Type* instance) = nullptr;
  };

  HPMUSBDevice(
      USB_Type* instance, uint32_t irq, const std::initializer_list<EPConfig> EP_CFGS,
      USB::DeviceDescriptor::PacketSize0 packet_size, uint16_t vid, uint16_t pid,
      uint16_t bcd,
      const std::initializer_list<const USB::DescriptorStrings::LanguagePack*> LANG_LIST,
      const std::initializer_list<const std::initializer_list<USB::ConfigDescriptorItem*>>
          CONFIGS,
      ConstRawData uid = {nullptr, 0}, USB::Speed speed = USB::Speed::FULL,
      bool auto_board_init = true, USB::USBSpec spec = USB::USBSpec::USB_2_0,
      BoardConfig board_config = BoardConfig());

  struct Diagnostics
  {
    uint32_t start_count = 0;
    uint32_t irq_count = 0;
    uint32_t reset_count = 0;
    uint32_t setup_count = 0;
    uint32_t complete_count = 0;
    uint32_t error_count = 0;
    uint32_t port_change_count = 0;
    uint32_t suspend_count = 0;
    uint32_t transfer_error_count = 0;
    uint32_t last_status = 0;
    uint32_t last_complete = 0;
    uint32_t last_setup_status = 0;
    uint32_t last_usbcmd = 0;
    uint32_t last_usbmode = 0;
    uint32_t last_portsc1 = 0;
    uint32_t current_status = 0;
    uint32_t current_setup_status = 0;
    uint32_t current_complete = 0;
    uint32_t current_usbcmd = 0;
    uint32_t current_usbmode = 0;
    uint32_t current_portsc1 = 0;
    uint32_t current_otgsc = 0;
    uint32_t current_phy_ctrl0 = 0;
    uint32_t current_phy_ctrl1 = 0;
    uint32_t current_phy_status = 0;
    uint8_t last_address = 0;
    uint8_t last_setup_request_type = 0;
    uint8_t last_setup_request = 0;
    uint16_t last_setup_value = 0;
    uint16_t last_setup_index = 0;
    uint16_t last_setup_length = 0;
  };

  Diagnostics GetDiagnostics() const;

  void Init(bool in_isr) override { USB::DeviceCore::Init(in_isr); }
  void Deinit(bool in_isr) override { USB::DeviceCore::Deinit(in_isr); }

  void Start(bool in_isr) override;
  void Stop(bool in_isr) override;

  ErrorCode SetAddress(uint8_t address, USB::DeviceCore::Context context) override;

  void HandleInterrupt();
  void HandleBusReset(bool in_isr);

  static uint8_t ResolveIndex(USB_Type* instance);
  static HPMUSBDevice* GetByIrq(uint32_t irq);

  HPMUSBDqh* Qhd(uint8_t ep_idx);
  HPMUSBDtd* Qtd(uint8_t ep_idx);
  HPMUSBEndpoint* Endpoint(uint8_t ep_idx);

  bool OpenEndpoint(uint8_t ep_addr, USB::Endpoint::Type type, uint16_t max_packet_size);
  bool SubmitTransfer(uint8_t ep_addr, uint8_t* buffer, uint32_t total_bytes);

  void RegisterEndpoint(HPMUSBEndpoint* ep);

  USB_Type* instance_ = nullptr;
  uint32_t irq_ = INVALID_IRQ;
  uint8_t index_ = INVALID_INSTANCE_INDEX;
  USB::Speed speed_ = USB::Speed::FULL;
  USB::DeviceDescriptor::PacketSize0 ep0_packet_size_;
  bool auto_board_init_ = true;
  BoardConfig board_config_{};
  bool running_ = false;

  static HPMUSBDevice* instances_[MAX_USB_INSTANCES];

 private:
  friend class HPMUSBEndpoint;

  void ResetDcdData();
  void HandleTransferComplete(bool in_isr);
  void HandleSetupPacket(bool in_isr);
  void ConfigureHardware();

  volatile uint32_t diag_start_count_ = 0;
  volatile uint32_t diag_irq_count_ = 0;
  volatile uint32_t diag_reset_count_ = 0;
  volatile uint32_t diag_setup_count_ = 0;
  volatile uint32_t diag_complete_count_ = 0;
  volatile uint32_t diag_error_count_ = 0;
  volatile uint32_t diag_port_change_count_ = 0;
  volatile uint32_t diag_suspend_count_ = 0;
  volatile uint32_t diag_transfer_error_count_ = 0;
  volatile uint32_t diag_last_status_ = 0;
  volatile uint32_t diag_last_complete_ = 0;
  volatile uint32_t diag_last_setup_status_ = 0;
  volatile uint32_t diag_last_usbcmd_ = 0;
  volatile uint32_t diag_last_usbmode_ = 0;
  volatile uint32_t diag_last_portsc1_ = 0;
  volatile uint8_t diag_last_address_ = 0;
  volatile uint8_t diag_last_setup_request_type_ = 0;
  volatile uint8_t diag_last_setup_request_ = 0;
  volatile uint16_t diag_last_setup_value_ = 0;
  volatile uint16_t diag_last_setup_index_ = 0;
  volatile uint16_t diag_last_setup_length_ = 0;

  HPMUSBEndpoint* endpoints_[USB_SOS_DCD_MAX_QHD_COUNT] = {};
};

}  // namespace LibXR
