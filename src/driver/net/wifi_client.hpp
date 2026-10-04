#pragma once

#include "libxr_def.hpp"
#include "libxr_rw.hpp"
#include "net.hpp"

namespace LibXR
{

/**
 * @class WifiClient
 * @brief WiFi 客户端接口 / WiFi Client Interface
 *
 * 提供对 WiFi 模块的基本控制、连接管理、网络状态查询和扫描等接口，
 * 同时继承 NetworkInterface，可统一作为网络接口使用。
 */
class WifiClient : public NetworkInterface
{
 public:
  // =========================
  // 类型定义 / Type Definitions
  // =========================

  /**
   * @enum WifiError
   * @brief WiFi 错误码 / Enumeration of WiFi error codes
   */
  enum class WifiError
  {
    NONE,                   ///< 无错误 / No error
    ALREADY_ENABLED,        ///< 已启用 / Already enabled
    NOT_ENABLED,            ///< 未启用 / Not enabled
    CONNECTION_TIMEOUT,     ///< 连接超时 / Connection timeout
    AUTHENTICATION_FAILED,  ///< 身份验证失败 / Authentication failed
    DHCP_FAILED,            ///< DHCP 获取失败 / DHCP acquisition failed
    SSID_NOT_FOUND,         ///< 找不到 SSID / SSID not found
    INVALID_CONFIG,         ///< 配置无效 / Invalid configuration
    HARDWARE_FAILURE,       ///< 硬件故障 / Hardware failure
    SCAN_FAILED,            ///< 扫描失败 / Scan failed
    UNKNOWN,                ///< 未知错误 / Unknown error
  };

  /**
   * @enum Security
   * @brief WiFi 安全类型 / WiFi security types
   */
  enum class Security
  {
    OPEN,             ///< 开放网络 / Open network
    WPA2_PSK,         ///< WPA2-PSK
    WPA2_ENTERPRISE,  ///< WPA2 企业认证 / WPA2 Enterprise
    UNKNOWN,          ///< 未知类型 / Unknown type
  };

  /**
   * @struct EnterpriseConfig
   * @brief 企业 WiFi 配置 / Enterprise WiFi configuration
   *
   * 包含身份验证相关的配置，如用户名、密码和证书路径。
   */
  struct EnterpriseConfig
  {
    const char* identity;     ///< EAP 身份标识 / EAP identity
    const char* username;     ///< 用户名 / Username
    const char* password;     ///< 密码 / Password
    const char* ca_cert;      ///< CA 证书路径 / CA certificate path
    const char* client_cert;  ///< 客户端证书路径 / Client certificate path
    const char* client_key;   ///< 客户端密钥路径 / Client key path
  };

  /**
   * @struct StaticIPConfig
   * @brief 静态 IP 配置 / Static IP configuration
   *
   * 包含静态 IP、网关、子网掩码和 DNS 配置。
   */
  struct StaticIPConfig
  {
    IPAddressRaw ip;       ///< IP 地址 / IP address
    IPAddressRaw gateway;  ///< 网关地址 / Gateway address
    IPAddressRaw netmask;  ///< 子网掩码 / Netmask
    IPAddressRaw dns;      ///< DNS 服务器 / DNS server
  };

  /**
   * @struct Config
   * @brief WiFi 连接配置 / WiFi connection configuration
   *
   * 包含 SSID、密码、DHCP 使用与可选企业配置/静态 IP 设置。
   */
  struct Config
  {
    char ssid[33];                           ///< SSID 名称 / SSID name
    char password[64];                       ///< 密码 / Password
    Security security = Security::WPA2_PSK;  ///< 安全类型 / Security type

    const EnterpriseConfig* enterprise_config =
        nullptr;  ///< 企业认证配置（可选） / Enterprise config (optional)
    const StaticIPConfig* static_ip_config =
        nullptr;  ///< 静态 IP 配置（可选） / Static IP config (optional)

    bool use_dhcp = true;  ///< 是否使用 DHCP / Use DHCP or not
  };

  /**
   * @struct ScanResult
   * @brief WiFi 扫描结果 / WiFi scan result
   *
   * 描述每个可用网络的 SSID、信号强度和加密方式。
   */
  struct ScanResult
  {
    char ssid[33]{};                        ///< 发现的 SSID / Detected SSID
    int rssi = 0;                           ///< 信号强度 / Signal strength (RSSI)
    Security security = Security::UNKNOWN;  ///< 安全类型 / Security type
  };

  /**
   * @typedef WifiCallback
   * @brief WiFi 状态回调类型 / Callback type for WiFi status
   */
  using WifiCallback = LibXR::Callback<WifiError>;

  /**
   * @brief 析构函数 / Destructor
   */
  virtual ~WifiClient() = default;

  /**
   * @brief 启用网络接口（WiFi） / Enable the network interface
   * @return true 表示成功 / true if enabled successfully
   */
  virtual bool Enable() override = 0;

  /**
   * @brief 禁用网络接口（WiFi） / Disable the network interface
   */
  virtual void Disable() override = 0;

  /**
   * @brief 检查是否已连接 / Check if currently connected
   * @return true 如果连接上了 / true if connected
   */
  virtual bool IsConnected() const override = 0;

  /**
   * @brief 获取当前 IP 地址 / Get current IP address
   * @return 当前 IP 地址（结构形式） / Current IP address in raw form
   */
  virtual IPAddressRaw GetIPAddress() const override = 0;

  /**
   * @brief 获取当前 MAC 地址 / Get MAC address
   * @return 当前 MAC 地址（结构形式） / Current MAC address in raw form
   */
  virtual MACAddressRaw GetMACAddress() const override = 0;

  /**
   * @brief 连接到指定 WiFi 网络 / Connect to a WiFi network
   * @param[in] config WiFi 连接配置 / Configuration parameters
   * @return WifiError 连接结果 / Error code
   */
  virtual WifiError Connect(const Config& config) = 0;

  /**
   * @brief 断开当前 WiFi 连接 / Disconnect from the WiFi network
   * @return WifiError 断开结果 / Error code
   */
  virtual WifiError Disconnect() = 0;

  /**
   * @brief 扫描可用网络 / Scan for available WiFi networks
   * @param[out] out_list 扫描结果数组 / Output list buffer
   * @param[in] max_count 最大可填入数量 / Max result count
   * @param[out] out_found 实际找到数量 / Number found
   * @return WifiError 错误码 / Scan result code
   */
  virtual WifiError Scan(ScanResult* out_list, size_t max_count, size_t& out_found) = 0;

  /**
   * @brief 获取当前 WiFi 信号强度（RSSI） / Get current signal strength
   * @return 信号强度（dBm） / Signal strength in dBm
   */
  virtual int GetRSSI() const = 0;
};

}  // namespace LibXR
