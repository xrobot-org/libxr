#pragma once

#include <cstdint>
#include <cstdio>
#include <cstring>

#include "libxr_mem.hpp"

namespace LibXR
{

constexpr size_t IPADDR_STRLEN =
    16;  ///< IPv4 字符串最大长度（含结尾空字符） / Max IPv4 string buffer length
         ///< ("255.255.255.255" + '\0').
constexpr size_t MACADDR_STRLEN =
    18;  ///< MAC 字符串最大长度（含结尾空字符） / Max MAC string buffer length
         ///< ("FF:FF:FF:FF:FF:FF" + '\0').

/**
 * @brief 原始 IPv4 地址 / Raw IPv4 address
 */
struct IPAddressRaw
{
  uint8_t bytes[4]{};  ///< IPv4 地址 4 个字节 / Four octets of the IPv4 address.

  /**
   * @brief 从点分十进制字符串解析 IPv4 地址 / Parse IPv4 address from a dotted-decimal
   *        string.
   * @param str 输入 IPv4 字符串 / Input IPv4 string.
   * @return 解析得到的原始 IPv4 地址 / Parsed raw IPv4 address.
   */
  static IPAddressRaw FromString(const char* str)
  {
    IPAddressRaw ip{};
    std::sscanf(str, "%hhu.%hhu.%hhu.%hhu", &ip.bytes[0], &ip.bytes[1], &ip.bytes[2],
                &ip.bytes[3]);
    return ip;
  }

  /**
   * @brief 将原始 IPv4 地址格式化为点分十进制字符串 / Format raw IPv4 address as
   *        dotted-decimal string.
   * @param out 输出字符缓冲区（长度至少 `IPADDR_STRLEN`） / Output string buffer (at
   * least `IPADDR_STRLEN` bytes).
   */
  void ToString(char out[IPADDR_STRLEN]) const
  {
    std::snprintf(out, IPADDR_STRLEN, "%u.%u.%u.%u", bytes[0], bytes[1], bytes[2],
                  bytes[3]);
  }

  /**
   * @brief 比较两个 IPv4 地址是否相等 / Check equality of two IPv4 addresses.
   * @param other 另一个 IPv4 地址 / Another IPv4 address.
   * @return 相等返回 true，否则返回 false / True if equal, false otherwise.
   */
  bool operator==(const IPAddressRaw& other) const
  {
    return Memory::FastCmp(bytes, other.bytes, 4) == 0;
  }

  /**
   * @brief 比较两个 IPv4 地址是否不相等 / Check inequality of two IPv4 addresses.
   * @param other 另一个 IPv4 地址 / Another IPv4 address.
   * @return 不相等返回 true，否则返回 false / True if not equal, false otherwise.
   */
  bool operator!=(const IPAddressRaw& other) const { return !(*this == other); }
};

/**
 * @brief 字符串形式 IPv4 地址 / IPv4 address as string
 */
struct IPAddressStr
{
  char str[IPADDR_STRLEN]{};  ///< 点分十进制 IPv4 字符串缓冲区 / Null-terminated IPv4
                              ///< string buffer.

  /**
   * @brief 从原始 IPv4 地址构造字符串封装 / Convert raw IPv4 address to string form.
   * @param raw 原始 IPv4 地址 / Raw IPv4 address.
   * @return 字符串形式的 IPv4 地址 / String representation of the IPv4 address.
   */
  static IPAddressStr FromRaw(const IPAddressRaw& raw)
  {
    IPAddressStr s{};
    raw.ToString(s.str);
    return s;
  }

  /**
   * @brief 转换为原始 4 字节 IPv4 地址 / Convert to raw 4-byte IPv4 address.
   * @return 原始 IPv4 地址 / Raw IPv4 address.
   */
  IPAddressRaw ToRaw() const { return IPAddressRaw::FromString(str); }
};

/**
 * @brief 原始 MAC 地址 / Raw MAC address
 */
struct MACAddressRaw
{
  uint8_t bytes[6]{};  ///< MAC 地址 6 个字节 / Six octets of the MAC address.

  /**
   * @brief 从冒号分隔十六进制字符串解析 MAC 地址 / Parse MAC address from a
   *        colon-separated hexadecimal string.
   * @param str 输入 MAC 字符串 / Input MAC string.
   * @return 解析得到的原始 MAC 地址 / Parsed raw MAC address.
   */
  static MACAddressRaw FromString(const char* str)
  {
    MACAddressRaw mac{};
    std::sscanf(str, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx", &mac.bytes[0], &mac.bytes[1],
                &mac.bytes[2], &mac.bytes[3], &mac.bytes[4], &mac.bytes[5]);
    return mac;
  }

  /**
   * @brief 将原始 MAC 地址格式化为冒号分隔十六进制字符串 / Format raw MAC address as
   *        colon-separated hex string.
   * @param out 输出字符缓冲区（长度至少 `MACADDR_STRLEN`） / Output buffer (at least
   *        `MACADDR_STRLEN` bytes).
   */
  void ToString(char out[MACADDR_STRLEN]) const
  {
    std::snprintf(out, MACADDR_STRLEN, "%02X:%02X:%02X:%02X:%02X:%02X", bytes[0],
                  bytes[1], bytes[2], bytes[3], bytes[4], bytes[5]);
  }

  /**
   * @brief 比较两个 MAC 地址是否相等 / Check equality of two MAC addresses.
   * @param other 另一个 MAC 地址 / Another MAC address.
   * @return 相等返回 true，否则返回 false / True if equal, false otherwise.
   */
  bool operator==(const MACAddressRaw& other) const
  {
    return Memory::FastCmp(bytes, other.bytes, 6) == 0;
  }

  /**
   * @brief 比较两个 MAC 地址是否不相等 / Check inequality of two MAC addresses.
   * @param other 另一个 MAC 地址 / Another MAC address.
   * @return 不相等返回 true，否则返回 false / True if not equal, false otherwise.
   */
  bool operator!=(const MACAddressRaw& other) const { return !(*this == other); }
};

/**
 * @brief 字符串形式 MAC 地址 / MAC address as string
 */
struct MACAddressStr
{
  char str[MACADDR_STRLEN]{};  ///< 冒号分隔 MAC 字符串缓冲区 / Null-terminated MAC string
                               ///< buffer.

  /**
   * @brief 从原始 MAC 地址构造字符串封装 / Convert raw MAC address to string form.
   * @param raw 原始 MAC 地址 / Raw MAC address.
   * @return 字符串形式的 MAC 地址 / String representation of the MAC address.
   */
  static MACAddressStr FromRaw(const MACAddressRaw& raw)
  {
    MACAddressStr s{};
    raw.ToString(s.str);
    return s;
  }

  /**
   * @brief 转换为原始 6 字节 MAC 地址 / Convert to raw 6-byte MAC address.
   * @return 原始 MAC 地址 / Raw MAC address.
   */
  MACAddressRaw ToRaw() const { return MACAddressRaw::FromString(str); }
};

/**
 * @brief 抽象网络接口类 / Abstract base for network interfaces
 */
class NetworkInterface
{
 public:
  virtual ~NetworkInterface() = default;

  /**
   * @brief 启用网络接口 / Enable the network interface.
   * @return 成功返回 true，否则返回 false / True on success, false otherwise.
   */
  virtual bool Enable() = 0;

  /**
   * @brief 禁用网络接口 / Disable the network interface.
   */
  virtual void Disable() = 0;

  /**
   * @brief 查询网络连接状态 / Check whether the interface is connected.
   * @return 已连接返回 true，否则返回 false / True if connected, false otherwise.
   */
  virtual bool IsConnected() const = 0;

  /**
   * @brief 获取当前 IPv4 地址 / Get the current IPv4 address.
   * @return 原始 IPv4 地址 / Current raw IPv4 address.
   */
  virtual IPAddressRaw GetIPAddress() const = 0;

  /**
   * @brief 获取接口硬件 MAC 地址 / Get the interface hardware MAC address.
   * @return 原始 MAC 地址 / Hardware raw MAC address.
   */
  virtual MACAddressRaw GetMACAddress() const = 0;
};

}  // namespace LibXR
