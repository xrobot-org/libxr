/**
 * @brief `Logger` 的字面量前端选择片段
 *        Literal-frontend selection fragment of `Logger`
 *
 * @note 这一组 helper 只负责在编译期判断一条日志字面量和一组参数应该落到
 *       brace 前端还是 printf 前端，不直接处理运行时 topic 发布。
 *       This helper set is responsible only for deciding at compile time
 *       whether one log literal plus one argument list should route to the
 *       brace frontend or the printf frontend; it does not perform runtime
 *       topic publication directly.
 */
namespace Detail::LoggerLiteral
{
/**
 * @brief Logger 字面量前端选择模式 / Logger literal frontend selection mode.
 */
enum class Frontend : uint8_t
{
  Auto,    ///< 自动选择 brace 或 printf / select brace or printf automatically
  Format,  ///< 强制使用 brace 风格前端 / force brace-style frontend
  Printf,  ///< 强制使用 printf 风格前端 / force printf-style frontend
};

/**
 * @brief 将一条 logger 字面量与可用前端进行解析后的结果。
 *        Result of resolving one logger literal against the available
 *        frontends.
 */
enum class Resolution : uint8_t
{
  None,       ///< 两个前端都不匹配 / matches neither frontend
  Format,     ///< 选择 brace 风格前端 / select brace-style frontend
  Printf,     ///< 选择 printf 风格前端 / select printf-style frontend
  Ambiguous,  ///< 两个前端都可用且都真的使用了自己的语法
              ///< both frontends remain valid and both syntaxes are used
};

/**
 * @brief 判断一条合法 brace 字面量是否真的使用了 brace 语法。
 *        Returns whether one valid brace literal actually uses brace syntax.
 */
template <Print::Text Source>
[[nodiscard]] consteval bool UsesFormatSyntax()
{
  for (size_t i = 0; i < Source.Size(); ++i)
  {
    if (Source.data[i] == '{' || Source.data[i] == '}')
    {
      return true;
    }
  }
  return false;
}

/**
 * @brief 判断一条合法 printf 字面量是否真的使用了 printf 语法。
 *        Returns whether one valid printf literal actually uses printf syntax.
 */
template <Print::Text Source>
[[nodiscard]] consteval bool UsesPrintfSyntax()
{
  for (size_t i = 0; i < Source.Size(); ++i)
  {
    if (Source.data[i] == '%')
    {
      return true;
    }
  }
  return false;
}

/**
 * @brief 判断一条 brace 风格源串在源级上是否合法。
 *        Returns whether one brace-style source is source-level valid.
 */
template <Print::Text Source>
[[nodiscard]] consteval bool FormatSourceValid()
{
  return Print::Detail::FormatFrontend::Analyze<Source>().error ==
         Print::Detail::FormatFrontend::Error::None;
}

/**
 * @brief 判断一条 printf 风格源串在源级上是否合法。
 *        Returns whether one printf-style source is source-level valid.
 */
template <Print::Text Source>
[[nodiscard]] consteval bool PrintfSourceValid()
{
  return Print::Detail::PrintfCompile::Analyze<Source>().error ==
         Print::Printf::Error::None;
}

/**
 * @brief 判断一组参数是否能被 brace 前端接受；会先做源级合法性保护。
 *        Returns whether one argument list is accepted by the brace frontend,
 *        guarded by source-level validity first.
 *
 * logger 自动检测不能把多余实参当作 brace 字面量的无害输入，否则不受支持的
 * printf 风格源串可能回退成 brace 纯文本并静默丢弃实参。
 * Logger auto-detection must not treat extra call-site arguments as harmless
 * for brace literals, otherwise unsupported printf-like sources can fall back
 * to brace plain text and silently drop their arguments.
 */
template <Print::Text Source, typename... Args>
[[nodiscard]] consteval bool FormatMatches()
{
  if constexpr (!FormatSourceValid<Source>())
  {
    return false;
  }
  else
  {
    return LibXR::Format<Source>::template Matches<Args...>();
  }
}

/**
 * @brief 判断一组参数是否能被 printf 前端接受；会先做源级合法性保护。
 *        Returns whether one argument list is accepted by the printf frontend,
 *        guarded by source-level validity first.
 */
template <Print::Text Source, typename... Args>
[[nodiscard]] consteval bool PrintfMatches()
{
  if constexpr (!PrintfSourceValid<Source>())
  {
    return false;
  }
  else
  {
    return Print::Printf::template Matches<Source, Args...>();
  }
}

/**
 * @brief 为一条 logger 字面量及一组具体参数选择前端。
 *        Selects the logger frontend for one literal plus one concrete
 *        argument list.
 */
template <Frontend Forced, Print::Text Source, typename... Args>
[[nodiscard]] consteval Resolution ResolveFrontend()
{
  constexpr bool format_match = FormatMatches<Source, Args...>();
  constexpr bool printf_match = PrintfMatches<Source, Args...>();

  if constexpr (Forced == Frontend::Format)
  {
    return format_match ? Resolution::Format : Resolution::None;
  }
  else if constexpr (Forced == Frontend::Printf)
  {
    return printf_match ? Resolution::Printf : Resolution::None;
  }
  else if constexpr (format_match && !printf_match)
  {
    return Resolution::Format;
  }
  else if constexpr (!format_match && printf_match)
  {
    return Resolution::Printf;
  }
  else if constexpr (!format_match && !printf_match)
  {
    return Resolution::None;
  }
  else
  {
    constexpr bool format_uses_syntax = UsesFormatSyntax<Source>();
    constexpr bool printf_uses_syntax = UsesPrintfSyntax<Source>();

    if constexpr (format_uses_syntax && !printf_uses_syntax)
    {
      return Resolution::Format;
    }
    else if constexpr (!format_uses_syntax && printf_uses_syntax)
    {
      return Resolution::Printf;
    }
    else if constexpr (!format_uses_syntax && !printf_uses_syntax)
    {
      return Resolution::Format;
    }
    else
    {
      return Resolution::Ambiguous;
    }
  }
}

/**
 * @brief 在校验解析结果后，选择最终 logger 前端。
 *        Selects the final logger frontend after validating the resolution
 *        result.
 */
template <Frontend Forced, Print::Text Source, typename... Args>
[[nodiscard]] consteval Frontend SelectFrontend()
{
  constexpr auto resolution = ResolveFrontend<Forced, Source, Args...>();

  if constexpr (Forced == Frontend::Format)
  {
    static_assert(
        resolution == Resolution::Format,
        "LibXR::Logger: XR_FMT(...) literal is not accepted by the brace frontend");
    return Frontend::Format;
  }
  else if constexpr (Forced == Frontend::Printf)
  {
    static_assert(
        resolution == Resolution::Printf,
        "LibXR::Logger: XR_PRINTF(...) literal is not accepted by the printf frontend");
    return Frontend::Printf;
  }
  else if constexpr (resolution == Resolution::Format)
  {
    return Frontend::Format;
  }
  else if constexpr (resolution == Resolution::Printf)
  {
    return Frontend::Printf;
  }
  else if constexpr (resolution == Resolution::Ambiguous)
  {
    static_assert(resolution != Resolution::Ambiguous,
                  "LibXR::Logger: literal is ambiguous between brace and printf "
                  "frontends; use XR_FMT(...) or XR_PRINTF(...)");
    return Frontend::Auto;
  }
  else
  {
    static_assert(resolution != Resolution::None,
                  "LibXR::Logger: literal matches neither brace nor printf frontend");
    return Frontend::Auto;
  }
}
}  // namespace Detail::LoggerLiteral
