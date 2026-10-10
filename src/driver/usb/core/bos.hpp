#pragma once

#include <cstddef>
#include <cstdint>

#include "desc_dev.hpp"
#include "libxr_def.hpp"
#include "libxr_mem.hpp"
#include "libxr_type.hpp"
#include "usb/core/core.hpp"

namespace LibXR::USB
{
/**
 * @brief BOS（Binary Object Store）描述符支持：能力收集、BOS 拼装、Vendor 请求分发
 *        BOS (Binary Object Store) support: capability collection, descriptor building,
 *        and vendor-request dispatching.
 */
enum class DevCapabilityType : uint8_t
{
  USB20_EXTENSION = 0x02,  ///< USB 2.0 扩展能力 / USB 2.0 Extension capability
};

static constexpr uint8_t DESCRIPTOR_TYPE_BOS =
    static_cast<uint8_t>(DescriptorType::BOS);  ///< BOS 描述符类型 / BOS descriptor type

static constexpr uint8_t DESCRIPTOR_TYPE_DEVICE_CAPABILITY = static_cast<uint8_t>(
    DescriptorType::DEVICE_CAPABILITY);  ///< 设备能力描述符类型 / Device capability
                                         ///< descriptor type

static constexpr uint8_t DEV_CAPABILITY_TYPE_USB20EXT = static_cast<uint8_t>(
    DevCapabilityType::USB20_EXTENSION);  ///< USB2.0 扩展能力类型 / USB 2.0 extension
                                          ///< capability type

static constexpr size_t BOS_HEADER_SIZE =
    5;  ///< BOS 头部长度（字节）/ BOS header size (bytes)
static constexpr size_t USB2_EXT_CAP_SIZE =
    7;  ///< USB2.0 扩展能力块长度（字节）/ USB 2.0 extension block size (bytes)

/**
 * @brief Vendor 请求处理结果（EP0 控制传输）
 *        Vendor request handling result for EP0 control transfers.
 */
struct BosVendorResult
{
  bool handled = false;              ///< 已处理该请求 / Request consumed by capability
  ConstRawData in_data{nullptr, 0};  ///< IN 数据阶段返回数据 / IN data stage payload
  bool write_zlp = false;            ///< 状态阶段发送 ZLP / Send ZLP at status stage
  bool early_read_zlp =
      true;  ///< 提前准备 OUT 以适配主机短读 / Arm OUT early for short-read tolerance
};

/**
 * @brief BOS 能力接口 / BOS capability interface
 */
class BosCapability
{
 public:
  /**
   * @brief 返回能力块（不含 BOS 头）
   *        Return capability block (without BOS header).
   *
   * @return ConstRawData 能力块字节序列 / Capability block bytes
   */
  virtual ConstRawData GetCapabilityDescriptor() const = 0;

  /**
   * @brief 处理该能力相关 Vendor 请求 / Handle vendor request for this capability
   *
   * @param in_isr ISR 上下文标志 / ISR context flag
   * @param setup  Setup 包 / Setup packet
   * @param result 输出结果 / Output result
   * @return OK：匹配且处理；NOT_SUPPORT：不匹配；其他：匹配但失败（上层应 STALL）
   */
  virtual ErrorCode OnVendorRequest(bool /*in_isr*/, const SetupPacket* /*setup*/,
                                    BosVendorResult& /*result*/)
  {
    return ErrorCode::NOT_SUPPORT;
  }
};

/**
 * @brief BOS 能力提供者接口 / BOS capability provider interface
 */
class BosCapabilityProvider
{
 public:
  /**
   * @brief 获取 BOS 能力数量 / Get BOS capability count
   * @return 能力数量 / Capability count
   */
  virtual size_t GetBosCapabilityCount() { return 0; }

  /**
   * @brief 获取指定索引的 BOS 能力 / Get BOS capability at index
   * @param index 能力索引 / Capability index
   * @return BosCapability 指针（可能为空）/ BosCapability pointer (nullable)
   */
  virtual BosCapability* GetBosCapability(size_t index)
  {
    UNUSED(index);
    return nullptr;
  }
};

/**
 * @brief BOS 管理器：能力收集、BOS 描述符拼装、Vendor 请求链式分发
 *        BOS manager: capability collection, descriptor building, vendor dispatch.
 *
 * @note 仅拥有内部数组/缓冲区，不拥有 BosCapability 对象。
 */
class BosManager
{
 public:
  /**
   * @brief 构造函数 / Constructor
   *
   * @param buffer_size BOS 描述符缓冲区大小 / BOS buffer size
   * @param cap_num     能力指针容量 / Capability pointer capacity
   */
  BosManager(size_t buffer_size, size_t cap_num)
      : cap_capacity_(cap_num),
        caps_(new BosCapability*[cap_num]),
        bos_buffer_({new uint8_t[buffer_size], buffer_size})
  {
  }

  BosManager(const BosManager&) = delete;
  BosManager& operator=(const BosManager&) = delete;

  /**
   * @brief 清空已注册能力 / Clear registered capabilities
   */
  void ClearCapabilities() { count_ = 0; }

  /**
   * @brief 添加能力 / Add capability
   *
   * @param cap 能力指针 / Capability pointer
   * @return true：成功；false：失败 / true: success; false: failure
   */
  bool AddCapability(BosCapability* cap)
  {
    DEV_ASSERT(bos_buffer_.addr_);
    ASSERT(cap != nullptr);
    ASSERT(count_ < cap_capacity_);

    caps_[count_++] = cap;
    return true;
  }

  /**
   * @brief 构建 BOS 描述符（BOS 头 + 能力块）
   *        Build BOS descriptor (header + blocks).
   *
   * @return ConstRawData BOS 描述符数据 / BOS descriptor bytes
   */
  ConstRawData GetBosDescriptor()
  {
    DEV_ASSERT(bos_buffer_.addr_);

    bool has_usb2_ext = false;
    for (size_t i = 0; i < count_; ++i)
    {
      ConstRawData blk = caps_[i]->GetCapabilityDescriptor();
      ASSERT(blk.addr_ != nullptr);
      ASSERT(blk.size_ >= 3);

      const uint8_t* p = reinterpret_cast<const uint8_t*>(blk.addr_);
      if (p[1] == DESCRIPTOR_TYPE_DEVICE_CAPABILITY &&
          p[2] == DEV_CAPABILITY_TYPE_USB20EXT)
      {
        has_usb2_ext = true;
      }
    }

    static constexpr uint8_t USB2_EXT_CAP[USB2_EXT_CAP_SIZE] = {7,    0x10, 0x02, 0x00,
                                                                0x00, 0x00, 0x00};
    const bool NEED_AUTO_USB2_EXT = !has_usb2_ext;

    size_t total = BOS_HEADER_SIZE;
    for (size_t i = 0; i < count_; ++i)
    {
      ConstRawData blk = caps_[i]->GetCapabilityDescriptor();
      ASSERT(blk.addr_ != nullptr);
      ASSERT(blk.size_ >= 1);
      total += blk.size_;
    }
    if (NEED_AUTO_USB2_EXT)
    {
      total += sizeof(USB2_EXT_CAP);
    }

    ASSERT(total <= bos_buffer_.size_);
    ASSERT(total <= 0xFFFF);

    auto buffer = reinterpret_cast<uint8_t*>(bos_buffer_.addr_);

    buffer[0] = static_cast<uint8_t>(BOS_HEADER_SIZE);
    buffer[1] = DESCRIPTOR_TYPE_BOS;
    buffer[2] = static_cast<uint8_t>(total & 0xFF);
    buffer[3] = static_cast<uint8_t>((total >> 8) & 0xFF);
    buffer[4] = static_cast<uint8_t>(count_ + (NEED_AUTO_USB2_EXT ? 1 : 0));

    size_t offset = BOS_HEADER_SIZE;

    for (size_t i = 0; i < count_; ++i)
    {
      ConstRawData blk = caps_[i]->GetCapabilityDescriptor();
      DEV_ASSERT(offset + blk.size_ <= bos_buffer_.size_);

      LibXR::Memory::FastCopy(&buffer[offset], blk.addr_, blk.size_);
      offset += blk.size_;
    }

    if (NEED_AUTO_USB2_EXT)
    {
      DEV_ASSERT(offset + sizeof(USB2_EXT_CAP) <= bos_buffer_.size_);
      LibXR::Memory::FastCopy(&buffer[offset], USB2_EXT_CAP, sizeof(USB2_EXT_CAP));
      offset += sizeof(USB2_EXT_CAP);
    }

    return {buffer, offset};
  }

  /**
   * @brief Vendor 请求分发 / Vendor request dispatch
   *
   * @param in_isr ISR 上下文标志 / ISR context flag
   * @param setup  Setup 包 / Setup packet
   * @param result 输出结果 / Output result
   * @return OK：已处理；NOT_SUPPORT：无匹配；其他：匹配但失败
   */
  ErrorCode ProcessVendorRequest(bool in_isr, const SetupPacket* setup,
                                 BosVendorResult& result)
  {
    if (setup == nullptr)
    {
      return ErrorCode::ARG_ERR;
    }

    for (size_t i = 0; i < count_; ++i)
    {
      BosVendorResult tmp{};
      auto ec = caps_[i]->OnVendorRequest(in_isr, setup, tmp);

      if (ec == ErrorCode::NOT_SUPPORT)
      {
        continue;
      }
      if (ec != ErrorCode::OK)
      {
        return ec;
      }
      if (tmp.handled)
      {
        result = tmp;
        return ErrorCode::OK;
      }
      return ErrorCode::FAILED;
    }

    return ErrorCode::NOT_SUPPORT;
  }

 private:
  size_t cap_capacity_ = 0;         ///< 能力指针容量 / Capability pointer capacity
  BosCapability** caps_ = nullptr;  ///< 能力指针表 / Capability pointer table
  size_t count_ = 0;                ///< 已注册能力数量 / Registered capability count

  RawData bos_buffer_ = {nullptr, 0};  ///< BOS 缓冲区 / BOS buffer
};

}  // namespace LibXR::USB
