/**
 * @file usb_device_test_common.hpp
 * @brief 设备类主机测试共用的外设替身 / Peripheral doubles shared by the device-class
 * host tests.
 */

#pragma once

#include <cstdint>
#include <cstring>

#include "flash.hpp"
#include "test_assert.hpp"

namespace LibXR::USB::Test
{
/// 以内存为后备的 Flash：擦除填 0xFF，写入直接拷贝，读取用基类的默认实现
/// RAM-backed Flash: erase fills with 0xFF, write copies, read is the base default.
class RamFlash : public LibXR::Flash
{
 public:
  RamFlash(size_t min_erase, size_t min_write, RawData area)
      : Flash(min_erase, min_write, area), area_(area)
  {
  }
  ErrorCode Erase(size_t offset, size_t size) override
  {
    TEST_ASSERT(offset + size <= area_.size_);
    std::memset(static_cast<uint8_t*>(area_.addr_) + offset, 0xFF, size);
    return ErrorCode::OK;
  }
  ErrorCode Write(size_t offset, ConstRawData data) override
  {
    TEST_ASSERT(offset + data.size_ <= area_.size_);
    std::memcpy(static_cast<uint8_t*>(area_.addr_) + offset, data.addr_, data.size_);
    return ErrorCode::OK;
  }

 private:
  RawData area_;  ///< 后备内存 / Backing memory.
};

}  // namespace LibXR::USB::Test
