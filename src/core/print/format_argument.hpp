#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>

#include "format_protocol.hpp"

namespace LibXR::Print::Detail::FormatArgument
{
/**
 * @brief 编译期参数匹配与运行期参数打包共用的内部 C++ 参数类型工具。
 *        Internal C++ argument-type utilities shared by compile-time matching
 *        and runtime packing.
 *
 * 此命名空间内的内容全部属于实现细节，不应泄漏到公开 LibXR::Print 接口中。
 * Everything in this namespace is implementation detail and should not leak into
 * the public LibXR::Print surface.
 * @tparam T 待归类的源 C++ 实参类型 / Source C++ argument type being classified.
 */
template <typename T>
struct TypeTraits
{
  using Decayed = std::remove_cvref_t<T>;
  using Normalized =
      typename std::conditional_t<std::is_enum_v<Decayed>, std::underlying_type<Decayed>,
                                  std::type_identity<Decayed>>::type;

  static constexpr bool is_signed_integer = std::is_integral_v<Normalized> &&
                                            !std::is_same_v<Normalized, bool> &&
                                            std::is_signed_v<Normalized>;
  static constexpr bool is_unsigned_integer = std::is_integral_v<Normalized> &&
                                              !std::is_same_v<Normalized, bool> &&
                                              std::is_unsigned_v<Normalized>;
  static constexpr bool is_default_signed_integer =
      is_signed_integer && sizeof(Normalized) <= sizeof(int);
  static constexpr bool is_default_unsigned_integer =
      is_unsigned_integer && sizeof(Normalized) <= sizeof(unsigned int);
  static constexpr bool is_char_array =
      std::is_array_v<Decayed> &&
      std::is_same_v<std::remove_cv_t<std::remove_extent_t<Decayed>>, char>;
  static constexpr bool is_c_string = std::is_same_v<Decayed, const char*> ||
                                      std::is_same_v<Decayed, char*> || is_char_array;
  static constexpr bool is_string_like = is_c_string ||
                                         std::is_same_v<Decayed, std::string_view> ||
                                         std::is_same_v<Decayed, std::string>;
  static constexpr bool is_pointer_like =
      (std::is_pointer_v<Decayed> &&
       !std::is_function_v<std::remove_pointer_t<Decayed>>) ||
      std::is_same_v<Decayed, std::nullptr_t>;
  static constexpr bool is_character_like = std::is_integral_v<Normalized>;
  static constexpr bool is_float =
      std::is_same_v<Decayed, float> || std::is_same_v<Decayed, double>;
  static constexpr bool is_long_double = std::is_same_v<Decayed, long double>;

  /**
   * @brief 判断当前 C++ 实参类型是否满足某条编译期匹配规则 / Returns whether this C++
   * argument type satisfies one compile-time rule.
   * @param rule 待测试的编译期匹配规则 / Compile-time matching rule to test.
   * @return 当前类型满足 `rule` 时返回 `true`，否则返回 `false`。
   *         Returns `true` when the current type satisfies `rule`, otherwise `false`.
   */
  [[nodiscard]] static consteval bool MatchesRule(FormatArgumentRule rule)
  {
    if (rule == FormatArgumentRule::None)
    {
      return true;
    }

    switch (rule)
    {
      case FormatArgumentRule::SignedAny:
        return is_default_signed_integer;
      case FormatArgumentRule::SignedChar:
        return std::is_same_v<Normalized, signed char>;
      case FormatArgumentRule::SignedShort:
        return std::is_same_v<Normalized, short>;
      case FormatArgumentRule::SignedLong:
        return std::is_same_v<Normalized, long>;
      case FormatArgumentRule::SignedLongLong:
        return std::is_same_v<Normalized, long long>;
      case FormatArgumentRule::SignedIntMax:
        return std::is_same_v<Normalized, intmax_t>;
      case FormatArgumentRule::SignedSize:
        return std::is_same_v<Normalized, std::make_signed_t<size_t>>;
      case FormatArgumentRule::SignedPtrDiff:
        return std::is_same_v<Normalized, ptrdiff_t>;
      case FormatArgumentRule::UnsignedAny:
        return is_default_unsigned_integer;
      case FormatArgumentRule::UnsignedChar:
        return std::is_same_v<Normalized, unsigned char>;
      case FormatArgumentRule::UnsignedShort:
        return std::is_same_v<Normalized, unsigned short>;
      case FormatArgumentRule::UnsignedLong:
        return std::is_same_v<Normalized, unsigned long>;
      case FormatArgumentRule::UnsignedLongLong:
        return std::is_same_v<Normalized, unsigned long long>;
      case FormatArgumentRule::UnsignedIntMax:
        return std::is_same_v<Normalized, uintmax_t>;
      case FormatArgumentRule::UnsignedSize:
        return std::is_same_v<Normalized, size_t>;
      case FormatArgumentRule::UnsignedPtrDiff:
        return std::is_same_v<Normalized, std::make_unsigned_t<ptrdiff_t>>;
      case FormatArgumentRule::Pointer:
        return is_pointer_like;
      case FormatArgumentRule::Character:
        return is_character_like;
      case FormatArgumentRule::String:
        return is_string_like;
      case FormatArgumentRule::Float:
        return is_float;
      case FormatArgumentRule::LongDouble:
        return is_long_double;
      default:
        return false;
    }
  }
};

/**
 * @brief 将一组模板实参与一组规则数组逐项展开比对 / Expand one argument list against one
 * rule array
 * @tparam Args 参与匹配的 C++ 实参类型列表 / C++ argument types being matched
 * @tparam N 规则数组元素个数 / Rule-array element count
 * @tparam I 展开的索引序列 / Expanded index sequence
 * @param arguments 编译后的参数元信息数组 / Compiled argument metadata array
 * @return 仅当每个参数都满足对应规则时返回 `true` / Returns `true` only when every
 * argument satisfies its matching rule
 */
template <typename... Args, size_t N, size_t... I>
[[nodiscard]] consteval bool MatchesImpl(
    const std::array<FormatArgumentInfo, N>& arguments, std::index_sequence<I...>)
{
  return (TypeTraits<Args>::MatchesRule(arguments[I].rule) && ...);
}

/**
 * @brief 判断给定实参列表是否与编译得到的参数元信息数组完全匹配 / Return whether the
 * provided argument list exactly matches the compiled argument metadata array
 * @tparam Args 参与匹配的 C++ 实参类型列表 / C++ argument types being matched
 * @tparam N 元信息数组元素个数 / Metadata-array element count
 * @param arguments 编译后的参数元信息数组 / Compiled argument metadata array
 * @return 当参数个数和每一项规则都完全匹配时返回 `true`，否则返回 `false` / Returns
 * `true` when the argument count and every argument rule match exactly, otherwise `false`
 */
template <typename... Args, size_t N>
[[nodiscard]] consteval bool Matches(const std::array<FormatArgumentInfo, N>& arguments)
{
  if constexpr (sizeof...(Args) != N)
  {
    return false;
  }
  else
  {
    return MatchesImpl<Args...>(arguments, std::make_index_sequence<N>{});
  }
}
}  // namespace LibXR::Print::Detail::FormatArgument
