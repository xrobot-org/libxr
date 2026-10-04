#pragma once

/**
 * @brief brace 前端的编译期分析与降级入口层 / Brace frontend compile-time analysis and
 * lowering entry surface
 */

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string_view>
#include <type_traits>

#include "../format_argument.hpp"
#include "../format_compile.hpp"
#include "../printf.hpp"

namespace LibXR::Print::Detail::FormatFrontend
{
/**
 * @brief brace 风格 format 前端的编译期失败类别 / Compile-time failure categories for
 * the brace-style format frontend.
 */
enum class Error : uint8_t
{
  None,            ///< 成功 / success
  NumberOverflow,  ///< 索引、宽度或精度超出目标字段容量
                   ///< index / width / precision does not fit its target field
  FloatPrecisionLimitExceeded,  ///< 浮点精度超出当前配置的前端上限
                                ///< float precision exceeds the configured frontend limit
  UnexpectedEnd,   ///< 字段尚未结束时源串已结束 / source ended in the middle of one field
  EmbeddedNul,     ///< 终止字节之前出现嵌入式 NUL / embedded NUL before the terminator
  UnmatchedBrace,  ///< 花括号未正确配对 / unmatched { or }
  MixedIndexing,  ///< 混用了自动索引与手动索引 / automatic and manual indexing were mixed
  ManualIndexingDisabled,  ///< 显式 {0}/{1} 参数索引已被配置关闭
                           ///< explicit {0}/{1} argument indexing is disabled by
                           ///< configuration
  DynamicField,            ///< 不支持宽度或精度中的嵌套替换字段
                           ///< nested replacement field in width / precision is
                           ///< unsupported
  InvalidArgumentIndex,    ///< 字段开头不是合法的十进制参数索引
                           ///< field head is not a valid decimal argument index
  InvalidSpecifier,        ///< format-spec 语法非法 / malformed format-spec grammar
  InvalidPresentation,     ///< 不支持的展示类型字符
                           ///< unsupported presentation type character
  MissingArgument,         ///< 引用的参数索引超出实参数量
                           ///< referenced argument index is out of range
  ArgumentTypeMismatch,    ///< 格式选项与选中的参数类型不兼容
                           ///< format options are incompatible with the selected argument
                           ///< type
  UnsupportedArgumentType,  ///< 当前前端不支持该 C++ 参数类型
                            ///< C++ argument type is not supported by this frontend
  TextOffsetOverflow,       ///< 文本池偏移超出 uint16_t
                            ///< referenced text offset no longer fits in uint16_t
  TextSizeOverflow,         ///< 文本长度超出 uint16_t
                            ///< referenced text size no longer fits in uint16_t
};

/**
 * @brief 降为 FormatFlag 位之前的已解析对齐方式 / Parsed alignment directive before
 * lowering into FormatFlag bits.
 */
enum class Align : uint8_t
{
  None,    ///< 默认对齐 / default alignment
  Left,    ///< 左对齐 / '<'
  Right,   ///< 右对齐 / '>'
  Center,  ///< 居中对齐 / '^'
};

/**
 * @brief 在绑定到具体 C++ 参数类型之前的 brace 字段解析结果 / Parsed brace field before
 * binding it to one concrete C++ argument type.
 */
struct ParsedField
{
  size_t arg_index = 0;       ///< 选中的源参数索引 / selected source argument index
  Align align = Align::None;  ///< 已解析对齐方式 / parsed alignment
  char fill = ' ';            ///< 已解析填充字符 / parsed fill character
  bool force_sign = false;    ///< 已解析正号选项 / parsed plus-sign option
  bool space_sign = false;    ///< 已解析空格符号选项 / parsed space-sign option
  bool alternate = false;     ///< 已解析备用格式选项 / parsed alternate-form option
  bool zero_pad = false;      ///< 已解析零填充选项 / parsed zero-pad option
  uint8_t width = 0;          ///< 已解析常量宽度 / parsed constant width
  bool has_precision =
      false;  ///< 是否显式给出了精度 / whether precision was explicitly present
  uint8_t precision = 0;  ///< 显式给出时的精度值 / parsed precision when present
  char presentation =
      0;  ///< 展示类型字符；缺省时为 0 / parsed presentation character, or 0 for default
};

/**
 * @brief 前端侧的参数类别，用来选择字段该走哪条解析路径 / Frontend-side argument
 * categories used to choose one field-resolution path.
 */
enum class ArgumentKind : uint8_t
{
  Unsupported,  ///< 不支持的 C++ 参数类型 / unsupported C++ argument type
  Bool,         ///< bool
  Character,    ///< 精确 char / exact char
  Signed,       ///< 有符号整数 / signed integer
  Unsigned,     ///< 无符号整数 / unsigned integer
  String,       ///< 字符串类 / string-like
  Pointer,      ///< 指针类 / pointer-like
  Float32,      ///< float
  Float64,      ///< double
  LongDouble,   ///< long double
};

/**
 * @brief 单个 C++ 参数在前端里的类别摘要，以及对应的宽度策略 / Frontend summary of one
 * C++ argument category plus width policy.
 */
struct ArgumentSummary
{
  ArgumentKind kind =
      ArgumentKind::Unsupported;  ///< 前端侧参数类别 / frontend-side argument category
  bool uses_64bit_storage =
      false;  ///< 整数是否必须走 64 位存储 / whether integer storage must be 64-bit
};

/**
 * @brief 将单个已解析 brace 字段解析成共享格式协议后的结果 / Result of resolving one
 * parsed brace field into the shared format protocol.
 */
struct ResolvedField
{
  Error error = Error::None;  ///< 字段解析结果 / field-resolution result
  FormatField field{};        ///< 解析后的共享字段 / resolved shared field
};

// These implementation headers form an ordered dependency chain.
// clang-format off
#include "format_frontend_source.hpp"

#include "format_frontend_binding_base.hpp"
#include "format_frontend_binding_integer.hpp"
#include "format_frontend_binding_float.hpp"
// clang-format on

/**
 * @brief 对单条 brace 风格字面量执行仅源串分析 / Run source-only analysis for one
 * brace-style literal
 * @tparam Source brace 风格格式串字面量 / Brace-style format literal
 * @return 源串分析结果，包含字段顺序、所需参数个数与首个源级错误 / Returns the source
 * analysis result, including field order, required argument count, and the first
 * source-level error
 */
template <Text Source>
[[nodiscard]] consteval auto Analyze()
{
  return SourceSyntax::Analyze<Source>();
}

/**
 * @brief 遍历一条 brace 风格字面量，并针对具体 C++ 参数列表产出共享 `FormatField` 记录 /
 * Walk one brace-style literal and emit shared `FormatField` records selected for the
 * concrete C++ argument list
 * @tparam Source brace 风格格式串字面量 / Brace-style format literal
 * @tparam Args 这次绑定使用的 C++ 实参类型列表 / Concrete C++ argument types used for
 * this binding
 * @param visitor 接收文本片段与最终 `FormatField` 记录的 visitor / Visitor receiving text
 * spans and final `FormatField` records
 * @return 首个源级错误或字段解析错误；成功时返回 `Error::None` / Returns the first
 * source-level or field-resolution error; returns `Error::None` on success
 */
template <Text Source, typename... Args>
[[nodiscard]] consteval Error WalkSourceAsFormatFields(auto& visitor)
{
  struct ResolvingVisitor
  {
    decltype(visitor)& inner;

    [[nodiscard]] consteval Error Text(size_t offset, size_t text_size)
    {
      return inner.Text(offset, text_size);
    }

    [[nodiscard]] consteval Error Field(const ParsedField& parsed)
    {
      auto resolved = ArgumentResolution::ResolveField<Args...>(parsed);
      if (resolved.error != Error::None)
      {
        return resolved.error;
      }
      return inner.Field(resolved.field);
    }
  };

  ResolvingVisitor resolving{visitor};
  return SourceSyntax::WalkSource(std::string_view(Source.Data(), Source.Size()),
                                  resolving);
}

/**
 * @brief 将一条 brace 风格源字面量绑定到一组具体 C++ 参数类型上的前端适配器 / Frontend
 * adapter that binds one brace-style source literal to one concrete C++ argument list
 * @tparam Source brace 风格格式串字面量 / Brace-style format literal
 * @tparam Args 这次绑定使用的 C++ 实参类型列表 / Concrete C++ argument types used for
 * this binding
 */
template <Text Source, typename... Args>
class Compiler
{
 public:
  using ErrorType = Error;

  /**
   * @brief 返回不含结尾零字节的源字符串字节序列 / Return the source string bytes without
   * a terminating zero byte
   * @return 指向当前格式字面量正文的指针 / Returns a pointer to the literal body
   */
  [[nodiscard]] static constexpr const char* SourceData() { return Source.Data(); }
  /**
   * @brief 返回不含结尾零字节的源字符串长度 / Return the source string length without a
   * terminating zero byte
   * @return 当前格式字面量正文长度 / Returns the literal-body size
   */
  [[nodiscard]] static constexpr size_t SourceSize() { return Source.Size(); }

  /**
   * @brief 遍历当前已绑定前端，并产出文本片段和最终 `FormatField` 记录 / Walk this bound
   * frontend and emit text spans plus final `FormatField` records
   * @param visitor 接收文本片段与最终字段记录的 visitor / Visitor receiving text spans
   * and final field records
   * @return 首个源级错误或字段解析错误；成功时返回 `Error::None` / Returns the first
   * source-level or field-resolution error; returns `Error::None` on success
   */
  [[nodiscard]] static consteval ErrorType Walk(auto& visitor)
  {
    return WalkSourceAsFormatFields<Source, Args...>(visitor);
  }
};
}  // namespace LibXR::Print::Detail::FormatFrontend
