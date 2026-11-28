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
   * @brief 返回能力块（不含 BOS 头） / Return capability block (without BOS header)
   * @return 能力块字节序列 / Capability block bytes
   * @note 返回 `{nullptr, 0}` 表示能力当前未启用，构建时将跳过；返回 `{nullptr, n>0}`
   *       或 `{non-null, 0}` 视为非法描述符。
   *       Returning `{nullptr, 0}` means the capability is disabled and skipped while
   *       building; `{nullptr, n>0}` or `{non-null, 0}` is treated as invalid.
   */
  virtual ConstRawData GetCapabilityDescriptor() const = 0;

  /**
   * @brief 返回能力块的最大字节数 / Return maximum capability descriptor byte length
   * @return 运行期能力块长度上界 / Runtime upper bound of capability block bytes
   * @note 组合层用此上界预留 BOS 缓冲，覆盖初始化后的启停变化；默认返回当前长度。
   *       运行期可能变长的自定义能力必须覆写此函数返回最大长度。
   *       Composition uses this bound to size the BOS buffer across enable/disable
   *       changes; defaults to current length. Custom capabilities that may grow at
   *       runtime must override this to return their upper bound.
   */
  virtual size_t GetMaxCapabilityDescriptorSize() const
  {
    return GetCapabilityDescriptor().size_;
  }

  /**
   * @brief 处理能力相关 Vendor 请求 / Handle vendor request for this capability
   * @param in_isr 是否处于中断上下文 / Whether running in ISR context
   * @param setup  Setup 包指针 / Setup packet pointer
   * @param result 输出应答结果 / Output vendor response
   * @return OK：匹配且处理；NOT_SUPPORT：不匹配或未启用；其他：失败（上层应 STALL） /
   *         OK if handled; NOT_SUPPORT if unmatched or disabled; else STALL
   */
  virtual ErrorCode OnVendorRequest(bool in_isr, const SetupPacket* setup,
                                    BosVendorResult& result)
  {
    UNUSED(in_isr);
    UNUSED(setup);
    UNUSED(result);
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
   * @note 初始禁用但允许运行期启用的能力仍须在 DeviceComposition 构造期通过本接口返回，
   *       以预留 BOS 槽位和最大字节预算。
   *       Capabilities initialized disabled but enabled at runtime must still be
   *       returned here during DeviceComposition construction to reserve BOS slots and
   *       the max-byte budget.
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
   * @return true：成功；false：指针为空或容量已满 / true: added; false: null or full
   */
  bool AddCapability(BosCapability* cap)
  {
    DEV_ASSERT(bos_buffer_.addr_);
    if (cap == nullptr || count_ >= cap_capacity_)
    {
      return false;
    }

    caps_[count_++] = cap;
    return true;
  }

  /**
   * @brief 构建 BOS 描述符（BOS 头 + 能力块） / Build BOS descriptor (header + blocks)
   *
   * @return BOS 描述符数据，失败时返回 `{nullptr, 0}` / BOS descriptor bytes, or
   *         `{nullptr, 0}` on failure
   * @note 返回视图在下一次调用 GetBosDescriptor() 前有效；遇到非法能力块或缓冲区不足时
   *       返回空视图供上层 STALL，不触发致命断言（可在 ISR 中调用）。
   *       The returned view stays valid until the next GetBosDescriptor() call; invalid
   *       capability blocks or insufficient capacity return `{nullptr, 0}` so the caller
   *       can STALL instead of trapping (safe in ISR context).
   */
  ConstRawData GetBosDescriptor()
  {
    if (bos_buffer_.addr_ == nullptr || bos_buffer_.size_ < BOS_HEADER_SIZE)
    {
      return {nullptr, 0};
    }

    auto buffer = reinterpret_cast<uint8_t*>(bos_buffer_.addr_);
    static constexpr uint8_t USB2_EXT_CAP[USB2_EXT_CAP_SIZE] = {7,    0x10, 0x02, 0x00,
                                                                0x00, 0x00, 0x00};

    bool has_usb2_ext = false;
    size_t offset = BOS_HEADER_SIZE;
    size_t active_caps = 0U;

    for (size_t i = 0; i < count_; ++i)
    {
      ConstRawData blk = caps_[i]->GetCapabilityDescriptor();
      if (blk.addr_ == nullptr && blk.size_ == 0U)
      {
        continue;
      }

      if (active_caps >= 0xFFU || blk.addr_ == nullptr || blk.size_ < 3U ||
          blk.size_ > 0xFFU || blk.size_ > bos_buffer_.size_ - offset)
      {
        return {nullptr, 0};
      }

      const uint8_t* p = reinterpret_cast<const uint8_t*>(blk.addr_);
      if (p[0] != blk.size_ || p[1] != DESCRIPTOR_TYPE_DEVICE_CAPABILITY)
      {
        return {nullptr, 0};
      }

      if (p[2] == DEV_CAPABILITY_TYPE_USB20EXT)
      {
        if (has_usb2_ext || blk.size_ != USB2_EXT_CAP_SIZE)
        {
          return {nullptr, 0};
        }
        has_usb2_ext = true;
      }

      LibXR::Memory::FastCopy(&buffer[offset], blk.addr_, blk.size_);
      offset += blk.size_;
      ++active_caps;
    }

    if (!has_usb2_ext)
    {
      if (active_caps >= 0xFFU || sizeof(USB2_EXT_CAP) > bos_buffer_.size_ - offset)
      {
        return {nullptr, 0};
      }
      LibXR::Memory::FastCopy(&buffer[offset], USB2_EXT_CAP, sizeof(USB2_EXT_CAP));
      offset += sizeof(USB2_EXT_CAP);
      ++active_caps;
    }

    if (offset > 0xFFFFU)
    {
      return {nullptr, 0};
    }

    buffer[0] = static_cast<uint8_t>(BOS_HEADER_SIZE);
    buffer[1] = DESCRIPTOR_TYPE_BOS;
    buffer[2] = static_cast<uint8_t>(offset & 0xFFU);
    buffer[3] = static_cast<uint8_t>((offset >> 8U) & 0xFFU);
    buffer[4] = static_cast<uint8_t>(active_caps);

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
        if (tmp.in_data.size_ != 0U && tmp.in_data.addr_ == nullptr)
        {
          return ErrorCode::FAILED;
        }
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
