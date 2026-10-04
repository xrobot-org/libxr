/**
 * @file test_type.cpp
 * @brief 检查 RawData 和 ConstRawData 的地址及长度。 /
 * Tests RawData and ConstRawData addresses and lengths.
 *
 * 使用字符串、非零结尾数组和含零字节的 string_view；std::string 和 std::string_view
 * 只能显式转换。
 * Uses strings, non-terminated arrays and string_view with embedded zero bytes;
 * std::string and std::string_view convert only explicitly.
 */

#include <string>
#include <string_view>
#include <type_traits>

#include "libxr_def.hpp"
#include "libxr_type.hpp"
#include "test.hpp"
#include "test_assert.hpp"

// 隐式转换会取对象本身的字节，因此必须编译失败；显式转换取文本。
// An implicit conversion would view the object's own bytes, so it must not compile;
// the explicit conversion views the text.
static_assert(!std::is_convertible_v<std::string, LibXR::ConstRawData>);
static_assert(!std::is_convertible_v<const std::string&, LibXR::ConstRawData>);
static_assert(!std::is_convertible_v<std::string_view, LibXR::ConstRawData>);
static_assert(!std::is_convertible_v<std::string&, LibXR::RawData>);
static_assert(!std::is_convertible_v<std::string_view&, LibXR::RawData>);
static_assert(std::is_constructible_v<LibXR::ConstRawData, const std::string&>);
static_assert(std::is_constructible_v<LibXR::ConstRawData, std::string_view>);
static_assert(std::is_constructible_v<LibXR::RawData, std::string&>);

void test_type()
{
  char mutable_text[] = "abc";
  LibXR::RawData mutable_text_view(mutable_text);
  TEST_ASSERT(mutable_text_view.addr_ == mutable_text);
  TEST_ASSERT(mutable_text_view.size_ == 3);

  char mutable_payload[3] = {'i', 'm', 'u'};
  LibXR::RawData mutable_payload_view(mutable_payload);
  TEST_ASSERT(mutable_payload_view.addr_ == mutable_payload);
  TEST_ASSERT(mutable_payload_view.size_ == 3);

  const char literal_text[] = "abc";
  LibXR::ConstRawData literal_text_view(literal_text);
  TEST_ASSERT(literal_text_view.addr_ == literal_text);
  TEST_ASSERT(literal_text_view.size_ == 3);

  const char embedded_text[] = "ab\0cd";
  LibXR::ConstRawData embedded_text_view(embedded_text);
  TEST_ASSERT(embedded_text_view.size_ == 5);

  const char bounded_payload[3] = {'g', 'p', 'u'};
  LibXR::ConstRawData bounded_payload_view(bounded_payload);
  TEST_ASSERT(bounded_payload_view.addr_ == bounded_payload);
  TEST_ASSERT(bounded_payload_view.size_ == 3);

  const std::string_view explicit_view("ab\0cd", 5);
  LibXR::ConstRawData explicit_view_data(explicit_view);
  TEST_ASSERT(explicit_view_data.addr_ == explicit_view.data());
  TEST_ASSERT(explicit_view_data.size_ == explicit_view.size());

  const std::string const_text("hello");
  LibXR::ConstRawData const_text_view(const_text);
  TEST_ASSERT(const_text_view.addr_ == const_text.data());
  TEST_ASSERT(const_text_view.size_ == const_text.size());

  std::string mutable_string("world!");
  LibXR::RawData mutable_string_view(mutable_string);
  TEST_ASSERT(mutable_string_view.addr_ == mutable_string.data());
  TEST_ASSERT(mutable_string_view.size_ == mutable_string.size());
}
