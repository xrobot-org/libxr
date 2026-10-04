#pragma once

#include <array>
#include <bit>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <string_view>
#include <tuple>
#include <type_traits>
#include <utility>

#include "../libxr_def.hpp"
#include "format_argument.hpp"
#include "format_protocol.hpp"
#include "print_contract.hpp"

namespace LibXR::Print
{
namespace Detail::WriterFloatLimit
{
/**
 * @brief 返回一个正整数的十进制位数 / Return the base-10 digit count of one positive
 * integer value
 * @param value 正整数值 / Positive integer value
 * @return 十进制位数 / Returns the decimal digit count
 */
[[nodiscard]] constexpr size_t DecimalDigitCount(size_t value)
{
  size_t digits = 1;
  while (value >= 10)
  {
    value /= 10;
    ++digits;
  }
  return digits;
}

/**
 * @brief 返回某个浮点类型在定点格式下的有界文本长度 / Return the bounded fixed-format
 * text length for one float family
 * @tparam Float 浮点类型 / Float type
 * @return 有界定点文本长度 / Returns the bounded fixed-format text length
 */
template <typename Float>
[[nodiscard]] constexpr size_t FixedTextLimit()
{
  constexpr size_t decimal_point =
      (Config::max_float_precision != 0 || Config::enable_alternate) ? 1U : 0U;
  return Config::max_float_integer_digits + decimal_point +
         static_cast<size_t>(Config::max_float_precision);
}

/**
 * @brief 返回某个浮点类型在科学计数法下的有界文本长度 / Return the bounded
 * scientific-format text length for one float family
 * @tparam Float 浮点类型 / Float type
 * @return 有界科学计数法文本长度 / Returns the bounded scientific-format text length
 */
template <typename Float>
[[nodiscard]] constexpr size_t ScientificTextLimit()
{
  constexpr size_t decimal_point =
      (Config::max_float_precision != 0 || Config::enable_alternate) ? 1U : 0U;
  constexpr size_t exponent_digits =
      DecimalDigitCount(static_cast<size_t>(std::numeric_limits<Float>::max_exponent10));
  return 1U + decimal_point + static_cast<size_t>(Config::max_float_precision) + 2U +
         exponent_digits;
}

/**
 * @brief 返回定点与科学计数法文本长度上界中的较大者 / Return the larger bound between
 * fixed and scientific text lengths
 * @tparam Float 浮点类型 / Float type
 * @return 两种有界文本长度中的较大值 / Returns the larger of the two bounded text lengths
 */
template <typename Float>
[[nodiscard]] constexpr size_t FloatTextLimit()
{
  constexpr size_t fixed_limit = FixedTextLimit<Float>();
  constexpr size_t scientific_limit = ScientificTextLimit<Float>();
  return fixed_limit > scientific_limit ? fixed_limit : scientific_limit;
}

/**
 * @brief 计算当前启用的浮点格式族共用的本地文本缓冲区容量 / Compute the shared local
 * float text-buffer capacity required by the enabled float families
 * @return 所需的共享浮点缓冲区容量 / Returns the required shared float buffer capacity
 */
[[nodiscard]] constexpr size_t ComputeBufferCapacity()
{
  size_t capacity = 4;
  if constexpr (Config::enable_float_fixed || Config::enable_float_scientific ||
                Config::enable_float_general)
  {
    capacity = FloatTextLimit<float>();
  }
  if constexpr (Config::enable_float_double)
  {
    constexpr size_t double_limit = FloatTextLimit<double>();
    capacity = capacity > double_limit ? capacity : double_limit;
  }
  if constexpr (Config::enable_float_long_double)
  {
    constexpr size_t long_double_limit = FloatTextLimit<long double>();
    capacity = capacity > long_double_limit ? capacity : long_double_limit;
  }
  return capacity;
}
}  // namespace Detail::WriterFloatLimit

/**
 * @brief 运行期 writer，负责打包调用点参数并执行编译好的字节流 / Runtime writer that
 * packs call-site arguments and executes the compiled byte stream
 */
class Writer
{
 public:
  /**
   * @brief 执行一份编译格式；它的字段顺序与源参数顺序可以不同 / Run one compiled format
   * whose field order and source argument order may differ
   * @tparam Sink 输出端类型，需满足 `OutputSink` / Sink type satisfying `OutputSink`
   * @tparam Format 编译格式类型 / Compiled format type
   * @tparam ArgumentOrder 按字段顺序排列的源参数索引表 / Source-argument index list
   * ordered by emitted fields
   * @tparam Args 调用点实参类型列表 / Call-site argument types
   * @param sink 输出端 / Destination sink
   * @param format 编译格式对象，仅用于约束与类型推导 / Compiled format object, used only
   * for constraints and type deduction
   * @param args 调用点实参 / Call-site arguments
   *
   * 这条路径用于 `{1} {0}` 这种格式：字段仍然按源串顺序执行，
   * 但运行期参数包要按另一种顺序构造。
   * This path is used by formats such as `{1} {0}`: fields are still executed
   * in source order, but the runtime argument pack must be built in another order.
   *
   * @note 这条接口不重新解析源格式串；它直接执行编译好的字节流 / This function does not
   * re-parse the source format string; it executes the compiled byte stream directly
   * @return 运行期 writer 执行状态 / Returns the runtime writer status
   */
  template <typename Sink, typename Format, auto ArgumentOrder, typename... Args>
    requires OutputSink<Sink> && CompiledFormat<std::remove_cvref_t<Format>>
  [[nodiscard]] static ErrorCode RunArgumentOrder(Sink& sink, const Format&,
                                                  Args&&... args)
  {
    using Built = std::remove_cvref_t<Format>;
    static_assert(ArgumentOrder.size() == Built::ArgumentList().size(),
                  "LibXR::Print::Writer: argument reorder list must match the "
                  "compiled field count");
    static_assert(
        Built::template Matches<Args...>(),
        "LibXR::Print::Writer::RunArgumentOrder: format arguments do not match");

    return RunTaggedArgumentOrder<Sink, Built::ArgumentList(), ArgumentOrder,
                                  Built::Profile()>(sink, Built::Codes().data(),
                                                    std::forward<Args>(args)...);
  }

 private:
  /**
   * @brief 为某个已编译参数列表生成栈上参数字节块 / Emit the stack argument byte blob for
   * one compiled argument list
   */
  static constexpr uint8_t unspecified_precision = 0xFF;
  template <FormatPackKind K>
  static constexpr bool dependent_false_v = false;
  static constexpr size_t float_buffer_capacity =
      Detail::WriterFloatLimit::ComputeBufferCapacity();

  /**
   * @brief 判断某个已解码字段修饰位是否被设置 / Test whether one decoded field-spec bit
   * is set
   */
  [[nodiscard]] static constexpr bool HasFlag(uint8_t flags, uint8_t bit)
  {
    return (flags & bit) != 0;
  }

  /**
   * @brief 运行期对单个字段描述字节组的解码视图 / Runtime view of one decoded field
   * specification byte group
   */
  struct Spec
  {
    uint8_t flags = 0;  ///< 字段修饰位集合 / FormatFlag bitset
    char fill = ' ';    ///< 字段填充字符 / field fill character
    uint8_t width = 0;  ///< 字段宽度，未指定时为 0 / field width, or zero when absent
    uint8_t precision = unspecified_precision;  ///< 字段精度，未指定时为哨兵值
                                                ///< precision, or unspecified_precision

    /**
     * @brief 判断是否请求了左对齐 / Return whether left alignment is requested
     * @return 开启左对齐时返回 `true`，否则返回 `false` / Returns `true` when left
     * alignment is enabled, otherwise `false`
     */
    [[nodiscard]] constexpr bool LeftAlign() const
    {
      return Writer::HasFlag(flags, static_cast<uint8_t>(FormatFlag::LeftAlign));
    }

    /**
     * @brief 判断是否请求了显式正号输出 / Return whether explicit plus-sign output is
     * requested
     * @return 开启显式正号输出时返回 `true`，否则返回 `false` / Returns `true` when
     * explicit plus-sign output is enabled, otherwise `false`
     */
    [[nodiscard]] constexpr bool ForceSign() const
    {
      return Writer::HasFlag(flags, static_cast<uint8_t>(FormatFlag::ForceSign));
    }

    /**
     * @brief 判断是否请求了居中对齐 / Return whether center alignment is requested
     * @return 开启居中对齐时返回 `true`，否则返回 `false` / Returns `true` when center
     * alignment is enabled, otherwise `false`
     */
    [[nodiscard]] constexpr bool CenterAlign() const
    {
      return Writer::HasFlag(flags, static_cast<uint8_t>(FormatFlag::CenterAlign));
    }

    /**
     * @brief 判断是否请求了正值前补空格的符号策略 / Return whether space-sign output for
     * positive values is requested
     * @return 开启空格符号策略时返回 `true`，否则返回 `false` / Returns `true` when the
     * space-sign policy is enabled, otherwise `false`
     */
    [[nodiscard]] constexpr bool SpaceSign() const
    {
      return Writer::HasFlag(flags, static_cast<uint8_t>(FormatFlag::SpaceSign));
    }

    /**
     * @brief 判断是否请求了备用格式 / Return whether alternate form is requested
     * @return 开启备用格式时返回 `true`，否则返回 `false` / Returns `true` when alternate
     * form is enabled, otherwise `false`
     */
    [[nodiscard]] constexpr bool Alternate() const
    {
      return Writer::HasFlag(flags, static_cast<uint8_t>(FormatFlag::Alternate));
    }

    /**
     * @brief 判断是否请求了零填充 / Return whether zero-padding is requested
     * @return 开启零填充时返回 `true`，否则返回 `false` / Returns `true` when
     * zero-padding is enabled, otherwise `false`
     */
    [[nodiscard]] constexpr bool ZeroPad() const
    {
      return Writer::HasFlag(flags, static_cast<uint8_t>(FormatFlag::ZeroPad));
    }

    /**
     * @brief 判断是否请求了大写输出 / Return whether uppercase output is requested
     * @return 开启大写输出时返回 `true`，否则返回 `false` / Returns `true` when uppercase
     * output is enabled, otherwise `false`
     */
    [[nodiscard]] constexpr bool UpperCase() const
    {
      return Writer::HasFlag(flags, static_cast<uint8_t>(FormatFlag::UpperCase));
    }

    /**
     * @brief 判断是否显式指定了精度 / Return whether precision was explicitly specified
     * @return 显式给出精度时返回 `true`，否则返回 `false` / Returns `true` when precision
     * is present, otherwise `false`
     */
    [[nodiscard]] constexpr bool HasPrecision() const
    {
      return precision != unspecified_precision;
    }
  };

  /**
   * @brief 返回一个 char 数组参数在边界内的 C 字符串长度 / Return the bounded C-string
   * length stored inside one char array argument
   * @tparam N 数组长度 / Array extent
   * @param text char 数组参数 / Char array argument
   * @return 返回边界内首个 NUL 的位置 / Returns the first in-range NUL position
   */
  template <size_t N>
  [[nodiscard]] static constexpr size_t BoundedTextLength(const char (&text)[N]) noexcept;

  /**
   * @brief 将一个字符串类运行期参数归一化为 `std::string_view` / Normalize one
   * string-like runtime argument into `std::string_view`
   * @tparam T 运行期实参类型 / Runtime argument type
   * @param text 运行期实参值 / Runtime argument value
   * @return 返回归一化后的文本视图 / Returns the normalized text view
   */
  template <typename T>
  [[nodiscard]] static constexpr std::string_view ToStringView(const T& text);

  /**
   * @brief 将一个运行期实参转换为某个编译参数槽要求的打包存储形态 / Convert one runtime
   * argument into the packed storage shape required by one compiled argument slot
   * @tparam pack 目标打包存储类型 / Target packed storage kind
   * @tparam T 运行期实参类型 / Runtime argument type
   * @param value 运行期实参值 / Runtime argument value
   * @return 返回该参数槽对应的打包值 / Returns the packed value for that slot
   */
  template <FormatPackKind pack, typename T>
  [[nodiscard]] static constexpr auto PackValue(T&& value);

  /**
   * @brief 把一个已打包值复制进参数字节块，并推进写指针 / Copy one packed value into the
   * argument byte blob and advance the cursor
   * @tparam T 已打包值类型 / Packed value type
   * @param out 当前写指针 / Current write cursor
   * @param value 待写入的已打包值 / Packed value to store
   */
  template <typename T>
  static void StoreArgument(uint8_t*& out, const T& value);

  /**
   * @brief 返回一组编译参数元信息在运行期需要的打包字节数 / Return the runtime
   * packed-byte size required by one compiled argument metadata list
   * @tparam ArgumentInfoList 编译期参数元信息列表 / Compile-time argument metadata list
   * @return 返回运行期打包总字节数 / Returns the packed runtime byte size
   */
  template <auto ArgumentInfoList>
  [[nodiscard]] static consteval size_t PackedArgumentBytes();

  /**
   * @brief 按字段执行顺序把运行期参数打包成最终字节块 / Pack runtime arguments into the
   * final byte blob in field execution order
   * @tparam ArgumentInfoList 编译期参数元信息列表 / Compile-time argument metadata list
   * @tparam ArgumentOrder 按字段顺序排列的源参数索引表 / Source-argument index list
   * ordered by emitted fields
   * @tparam Tuple 保存转发后运行期实参的 tuple 类型 / Tuple type holding forwarded
   * runtime arguments
   * @param out 当前写指针 / Current write cursor
   * @param tuple 转发后的运行期实参集合 / Forwarded runtime arguments
   */
  template <auto ArgumentInfoList, auto ArgumentOrder, typename Tuple>
  static void StoreArgumentsOrdered(uint8_t*& out, Tuple& tuple);

  /**
   * @brief 返回某个无符号整数在指定进制下所需的最大数字个数 / Return the maximum digit
   * count required for one unsigned integer under the selected radix
   *
   * 本辅助函数有意使用一个简短且精确的整数除法循环，而不是浮点近似公式。
   * 它被声明为 `consteval`，只能用于数组长度、`static_assert` 等立即编译期
   * 场景，因此不会引入任何运行期开销。
   * This helper intentionally uses a short exact integer division loop instead
   * of a floating-point approximation. It is `consteval`, so it can only be
   * used in immediate compile-time contexts such as array extents and
   * `static_assert`, and therefore adds no runtime cost.
   * @tparam UInt 无符号整数类型 / Unsigned integer type
   * @tparam Base 整数进制 / Integer radix
   * @return 返回该整型在所选进制下的最大数字个数 / Returns the maximum digit count under
   * the selected radix
   */
  template <std::unsigned_integral UInt, uint8_t Base>
  [[nodiscard]] static consteval size_t UnsignedDigitCapacity();

  /**
   * @brief 把一个无符号整数写进调用方提供的定长数字缓冲区 / Append one unsigned integer
   * into a caller-provided fixed-size digit buffer
   * @tparam Base 整数进制 / Integer radix
   * @tparam UpperCase 十六进制数字是否使用大写字母 / Whether hexadecimal digits should
   * use uppercase letters
   * @tparam N 目标缓冲区长度 / Destination buffer size
   * @tparam UInt 无符号整数类型 / Unsigned integer type
   * @param out 目标数字缓冲区 / Destination digit buffer
   * @param value 待编码的无符号值 / Unsigned value to encode
   * @return 返回输出的数字个数 / Returns the emitted digit count
   */
  template <uint8_t Base, bool UpperCase = false, size_t N, std::unsigned_integral UInt>
  [[nodiscard]] static size_t AppendUnsigned(char (&out)[N], UInt value);

  /**
   * @brief 以十进制形式追加一个较小的无符号整数 / Append one small unsigned integer in
   * base 10
   * @param out 目标数字缓冲区 / Destination digit buffer
   * @param value 待编码的无符号值 / Unsigned value to encode
   * @return 返回输出的数字个数 / Returns the emitted digit count
   */
  template <size_t N>
  [[nodiscard]] static size_t AppendSmallUnsigned(char (&out)[N], uint8_t value);

  /**
   * @brief 返回把某段载荷扩展到目标字段宽度所需的填充长度 / Return the padding width
   * needed to expand one payload to the requested field width
   * @param width 请求的字段宽度 / Requested field width
   * @param payload_size 填充前的可见载荷长度 / Visible payload size before padding
   * @return 返回需要补上的填充个数 / Returns the required padding count
   */
  [[nodiscard]] static constexpr size_t FieldPadding(uint8_t width, size_t payload_size);

  /**
   * @brief 返回整数精度引入的额外前导零个数 / Return the extra leading-zero count
   * introduced by integer precision
   * @param spec 解码后的字段规格 / Decoded field spec
   * @param digit_count 当前已有的数字个数 / Existing digit count
   * @return 返回额外前导零个数 / Returns the extra leading-zero count
   */
  [[nodiscard]] static constexpr size_t IntegerPrecisionZeros(const Spec& spec,
                                                              size_t digit_count);

  /**
   * @brief 返回放在数字载荷之外的备用格式前缀 / Return the alternate-form prefix carried
   * outside the digit payload
   * @tparam UInt 无符号整数类型 / Unsigned integer type
   * @param type 运行期字段类型 / Runtime field type
   * @param spec 解码后的字段规格 / Decoded field spec
   * @param value 当前要输出的无符号值 / Unsigned value being emitted
   * @return 返回分离在数字之外的备用格式前缀 / Returns the detached alternate-form prefix
   */
  template <std::unsigned_integral UInt>
  [[nodiscard]] static constexpr std::string_view IntegerPrefix(FormatType type,
                                                                const Spec& spec,
                                                                UInt value);

  /**
   * @brief 直接在已生成的数字载荷上应用 `%#o` 的特殊规则 / Apply `%#o` special rules
   * directly onto the generated digit payload
   *
   * 八进制备用格式与十六进制不同：它通过数字载荷本体前导一个 0 来表示，
   * 而不是额外分离出的前缀字符串。本函数也负责在 %#.0o 格式化零值时保留
   * 必需的单个 0。
   * Octal alternate form differs from hex: it is represented by a leading zero
   * in the digit payload itself, not by a detached prefix string. This helper
   * also preserves the required single 0 when %#.0o formats zero.
   * @tparam UInt 无符号整数类型 / Unsigned integer type
   * @param digits 可修改的数字缓冲区 / Mutable digit buffer
   * @param digit_count 当前数字个数 / Current digit count
   * @param spec 解码后的字段规格 / Decoded field spec
   * @param value 当前要输出的无符号值 / Unsigned value being emitted
   * @return 返回更新后的数字个数 / Returns the updated digit count
   */
  template <std::unsigned_integral UInt>
  [[nodiscard]] static size_t ApplyAlternateOctal(char* digits, size_t digit_count,
                                                  const Spec& spec, UInt value);

  /**
   * @brief 判断这个字段类型是否走共享的浮点文本输出路径 / Return whether this field type
   * is printed by the shared float-text path
   * @param type 运行期字段类型 / Runtime field type
   * @return 该类型走共享浮点文本后端时返回 `true`，否则返回 `false` / Returns `true` when
   * this type uses the shared float-text backend, otherwise `false`
   */
  [[nodiscard]] static constexpr bool UsesFloatTextBackend(FormatType type);

  /**
   * @brief 判断这个浮点字段类型是否被当前功能开关启用 / Return whether this float field
   * type is enabled by current feature switches
   * @param type 运行期浮点字段类型 / Runtime float field type
   * @return 当前功能开关允许该浮点字段类型时返回 `true`，否则返回 `false` / Returns
   * `true` when this float field type is enabled, otherwise `false`
   */
  [[nodiscard]] static constexpr bool FloatEnabled(FormatType type);
  /**
   * @brief 返回格式串没有显式写精度时使用的默认浮点精度 / Return the default float
   * precision used when the format string did not specify one
   * @return 返回默认浮点精度 / Returns the default float precision
   */
  [[nodiscard]] static constexpr uint8_t DefaultFloatPrecision();
  /**
   * @brief 判断定点形态的浮点输出是否会超出当前配置的整数位数上限 / Return whether
   * fixed-shape float output would exceed the configured integer-digit limit
   * @tparam Float 浮点类型 / Float type
   * @param value 待检查的浮点绝对值 / Float magnitude to test
   * @param precision 请求的小数精度 / Requested fractional precision
   * @return 若定点输出的整数部分会超出配置上限则返回 `true`，否则返回 `false` / Returns
   * `true` when the fixed-form integer part would exceed the configured limit, otherwise
   * `false`
   */
  template <typename Float>
  [[nodiscard]] static bool ExceedsFixedIntegerDigits(Float value, uint8_t precision);

  /**
   * @brief 向有界本地格式化缓冲区追加一个字符 / Append one character to a bounded local
   * formatting buffer
   * @param buffer 目标缓冲区 / Destination buffer
   * @param capacity 缓冲区总容量 / Total buffer capacity
   * @param size 当前已保留长度；成功时会推进 / Current retained size; advanced on success
   * @param ch 待追加字符 / Character to append
   * @return 成功返回 `true`，否则返回 `false` / Returns `true` on success, otherwise
   * `false`
   */
  [[nodiscard]] static bool AppendBufferChar(char* buffer, size_t capacity, size_t& size,
                                             char ch);

  /**
   * @brief 向有界本地格式化缓冲区追加一段文本 / Append one text span to a bounded local
   * formatting buffer
   * @param buffer 目标缓冲区 / Destination buffer
   * @param capacity 缓冲区总容量 / Total buffer capacity
   * @param size 当前已保留长度；成功时会推进 / Current retained size; advanced on success
   * @param text 待追加文本片段 / Text span to append
   * @return 成功返回 `true`，否则返回 `false` / Returns `true` on success, otherwise
   * `false`
   */
  [[nodiscard]] static bool AppendBufferText(char* buffer, size_t capacity, size_t& size,
                                             std::string_view text);

  /**
   * @brief 追加一个 `uint32_t` 十进制值，并在前面补零到目标宽度 / Append one `uint32_t`
   * decimal value and pad leading zeros up to the target width
   * @param buffer 目标缓冲区 / Destination buffer
   * @param capacity 缓冲区总容量 / Total buffer capacity
   * @param size 当前已保留长度；成功时会推进 / Current retained size; advanced on success
   * @param value 待追加的无符号值 / Unsigned value to append
   * @param width 目标零填充宽度 / Target zero-padded width
   * @return 成功返回 `true`，否则返回 `false` / Returns `true` on success, otherwise
   * `false`
   */
  [[nodiscard]] static bool AppendBufferU32ZeroPad(char* buffer, size_t capacity,
                                                   size_t& size, uint32_t value,
                                                   uint8_t width);

#if LIBXR_PRINT_ENABLE_FLOAT
  /**
   * @brief 科学计数法与通用浮点格式化使用的十进制规范化结果 / Decimal normalization
   * result used by scientific and general float formatting
   * @tparam Float 浮点类型 / Float type
   */
  template <typename Float>
  struct DecimalScale;

  /**
   * @brief 科学计数法舍入后的十进制数字块 / Rounded decimal digit block for scientific
   * notation
   * @tparam Float 浮点类型 / Float type
   */
  template <typename Float>
  struct ScientificDigits;

  /**
   * @brief 以指定浮点类型返回 `10^exponent` / Return `10^exponent` in the selected float
   * type
   * @tparam Float 浮点类型 / Float type
   * @param exponent 十进制指数 / Base-10 exponent
   * @return 对应的十进制幂 / Returns the corresponding power of ten
   */
  template <typename Float>
  [[nodiscard]] static Float Power10(int exponent);

  /**
   * @brief 把有限非负值舍入到指定小数位 / Round one finite non-negative value to a
   * decimal precision
   * @tparam Float 浮点类型 / Float type
   * @param value 有限非负值 / Finite non-negative value
   * @param precision 保留的小数位数 / Decimal places to retain
   * @return 舍入后的值；若内部缩放溢出则返回原值 / Rounded value, or the original value
   * if internal scaling overflows
   */
  template <typename Float>
  [[nodiscard]] static Float RoundDecimal(Float value, uint8_t precision);

  /**
   * @brief 计算科学计数法舍入后的数字块与指数 / Build the rounded digit block and
   * exponent for scientific notation
   * @tparam Float 浮点类型 / Float type
   * @param value 有限非负值 / Finite non-negative value
   * @param precision 小数点后的位数 / Digits after the decimal point
   * @return 舍入后的数字块、数字块权重和指数 / Rounded digits, digit scale, and exponent
   */
  template <typename Float>
  [[nodiscard]] static ScientificDigits<Float> RoundScientificDigits(Float value,
                                                                     uint8_t precision);

  /**
   * @brief 将一个有限正值规范化，使 `value / scale` 落在 `[1, 10)` / Normalize one finite
   * positive value so that `value / scale` stays in `[1, 10)`
   * @tparam Float 浮点类型 / Float type
   * @param value 有限正数绝对值 / Finite positive magnitude
   * @return 规范化后的十进制缩放描述 / Returns the normalized decimal scale description
   */
  template <typename Float>
  [[nodiscard]] static DecimalScale<Float> NormalizeDecimal(Float value);

  /**
   * @brief 在当前十进制权重下提取一位数字，并容忍微小浮点残差 / Extract one base-10 digit
   * at the current scale while tolerating tiny floating-point residue
   * @tparam Float 浮点类型 / Float type
   * @param value 剩余规范化值；会原地减少 / Remaining normalized value; reduced in place
   * @param scale 当前十进制权重 / Current decimal scale
   * @return 提取出的十进制数字 / Returns the extracted decimal digit
   */
  template <typename Float>
  [[nodiscard]] static uint8_t ExtractDigit(Float& value, Float scale);

  /**
   * @brief 在未启用备用格式时修剪通用浮点输出的尾随零，并去掉孤立小数点 / Trim trailing
   * zeros for general float output and remove a dangling decimal point when alternate
   * form is absent
   * @param text 可修改的浮点文本缓冲区 / Mutable float text buffer
   * @param size 当前文本长度 / Current text size
   * @return 修剪后的文本长度 / Returns the trimmed text size
   */
  [[nodiscard]] static size_t TrimGeneralText(char* text, size_t size);

  /**
   * @brief 通用定点浮点文本生成器 / Generic fixed-format float text generator
   * @tparam Float 浮点类型 / Float type
   * @param value 浮点绝对值 / Float magnitude
   * @param precision 请求的小数精度 / Requested fractional precision
   * @param alternate 是否启用备用格式 / Whether alternate form is enabled
   * @param out 目标文本缓冲区 / Destination text buffer
   * @param out_size 输出文本长度 / Output text size
   * @return 成功返回 `true`，否则返回 `false` / Returns `true` on success, otherwise
   * `false`
   */
  template <typename Float>
  [[nodiscard]] static bool FormatFixedText(Float value, uint8_t precision,
                                            bool alternate, char* out, size_t& out_size);

  /**
   * @brief 追加一个科学计数法指数后缀 / Append one scientific-notation exponent suffix
   * @param out 目标文本缓冲区 / Destination text buffer
   * @param out_size 当前输出长度；成功时会推进 / Current output size; advanced on success
   * @param exponent 十进制指数 / Decimal exponent
   * @param upper_case 指数标记是否使用 `E` / Whether the exponent marker should use `E`
   * @return 成功返回 `true`，否则返回 `false` / Returns `true` on success, otherwise
   * `false`
   */
  [[nodiscard]] static bool AppendExponentText(char* out, size_t& out_size, int exponent,
                                               bool upper_case);

  /**
   * @brief 通用科学计数法浮点文本生成器 / Generic scientific-format float text generator
   * @tparam Float 浮点类型 / Float type
   * @param value 浮点绝对值 / Float magnitude
   * @param precision 请求的小数精度 / Requested fractional precision
   * @param alternate 是否启用备用格式 / Whether alternate form is enabled
   * @param upper_case 指数标记是否使用 `E` / Whether the exponent marker should use `E`
   * @param out 目标文本缓冲区 / Destination text buffer
   * @param out_size 输出文本长度 / Output text size
   * @return 成功返回 `true`，否则返回 `false` / Returns `true` on success, otherwise
   * `false`
   */
  template <typename Float>
  [[nodiscard]] static bool FormatScientificText(Float value, uint8_t precision,
                                                 bool alternate, bool upper_case,
                                                 char* out, size_t& out_size);

  /**
   * @brief 按运行期字段类型选择的顶层浮点文本格式化器 / Top-level float text formatter
   * selected by runtime field type
   * @tparam Float 浮点类型 / Float type
   * @param type 运行期浮点字段类型 / Runtime float field type
   * @param spec 解码后的字段规格 / Decoded field spec
   * @param value 浮点绝对值 / Float magnitude
   * @param out 目标文本缓冲区 / Destination text buffer
   * @param out_size 输出文本长度 / Output text size
   * @return 成功返回 `true`，否则返回 `false` / Returns `true` on success, otherwise
   * `false`
   */
  template <typename Float>
  [[nodiscard]] static bool FormatFloatText(FormatType type, const Spec& spec,
                                            Float value, char* out, size_t& out_size);
#endif

  class CodeReader;
  class ArgumentReader;

  template <OutputSink Sink>
  class Executor;

  /**
   * @brief 使用一份已打包参数字节块执行一段编译字节流 / Execute one compiled byte stream
   * against one already packed argument blob
   * @tparam Sink 输出端类型，需满足 `OutputSink` / Sink type satisfying `OutputSink`
   * @tparam Profile 编译期 writer 摘要 / Compile-time writer profile
   * @param sink 输出端 / Destination sink
   * @param codes 指向编译字节流的指针 / Pointer to the compiled byte stream
   * @param args 指向已打包参数字节块的指针；无参数时可为空 / Pointer to the packed
   * argument blob, or null when no arguments exist
   * @return 执行器运行结果 / Returns the executor result
   */
  template <OutputSink Sink, FormatProfile Profile>
  [[nodiscard]] static LIBXR_NOINLINE ErrorCode Execute(Sink& sink, const uint8_t* codes,
                                                        const uint8_t* args)
  {
    return Executor<Sink>(sink, codes, args).template Run<Profile>();
  }

  /**
   * @brief 先打包转发后的运行期参数，再执行编译字节流 / Pack forwarded runtime arguments,
   * then execute the compiled byte stream
   * @tparam Sink 输出端类型，需满足 `OutputSink` / Sink type satisfying `OutputSink`
   * @tparam ArgumentInfoList 编译期参数元信息列表 / Compile-time argument metadata list
   * @tparam ArgumentOrder 按字段顺序排列的源参数索引表 / Source-argument index list
   * ordered by emitted fields
   * @tparam Profile 编译期 writer 摘要 / Compile-time writer profile
   * @tparam Args 转发后的运行期实参类型列表 / Forwarded runtime argument types
   * @param sink 输出端 / Destination sink
   * @param codes 指向编译字节流的指针 / Pointer to the compiled byte stream
   * @param args 转发后的运行期实参 / Forwarded runtime arguments
   * @return 执行器运行结果 / Returns the executor result
   */
  template <OutputSink Sink, auto ArgumentInfoList, auto ArgumentOrder,
            FormatProfile Profile, typename... Args>
  [[nodiscard]] static LIBXR_NOINLINE ErrorCode
  RunTaggedArgumentOrder(Sink& sink, const uint8_t* codes, Args&&... args)
  {
    if constexpr (ArgumentInfoList.size() == 0)
    {
      return Execute<Sink, Profile>(sink, codes, nullptr);
    }
    else
    {
      constexpr size_t packed_arg_bytes = PackedArgumentBytes<ArgumentInfoList>();
      uint8_t packed[packed_arg_bytes];
      auto tuple = std::forward_as_tuple(std::forward<Args>(args)...);
      auto* cursor = packed;
      StoreArgumentsOrdered<ArgumentInfoList, ArgumentOrder>(cursor, tuple);
      return Execute<Sink, Profile>(sink, codes, packed);
    }
  }
};

// These implementation headers form an ordered dependency chain.
// clang-format off
#include "writer/writer_argument.hpp"
#include "writer/writer_integer.hpp"
#if LIBXR_PRINT_ENABLE_FLOAT
#include "writer/writer_float_runtime.hpp"
#include "writer/writer_float_math.hpp"
#include "writer/writer_float_fixed.hpp"
#include "writer/writer_float_general.hpp"
#endif
#include "writer/writer_reader.hpp"
#include "writer/writer_executor.hpp"
// clang-format on
}  // namespace LibXR::Print
