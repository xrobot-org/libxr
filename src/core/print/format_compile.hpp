#pragma once

#include <array>
#include <bit>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <limits>

#include "format_protocol.hpp"
#include "print_contract.hpp"

namespace LibXR::Print
{
/**
 * @brief 共享编译期后端，把前端事件整理成最终字节流和参数表 / Shared compile-time
 * backend that turns frontend events into the final byte stream plus argument table.
 *
 * 前端协议：
 * 前端必须提供如下成员：using ErrorType、static constexpr const char*
 * SourceData()、static constexpr size_t SourceSize()、以及
 * static consteval ErrorType Walk(visitor)。
 * Frontend contract:
 * The frontend must provide the members using ErrorType, static constexpr
 * const char* SourceData(), static constexpr size_t SourceSize(), and
 * static consteval ErrorType Walk(visitor).
 *
 * Walk(visitor) 必须按源串顺序发射两类事件：
 * - visitor.Text(offset, text_size)
 * - visitor.Field(const FormatField&)
 * Walk(visitor) must emit source-ordered events through:
 * - visitor.Text(offset, text_size)
 * - visitor.Field(const FormatField&)
 *
 * 这个后端只遍历前端一次。那一遍里会同时收集：
 * - 输出字节码
 * - 长字面文本存储
 * - 运行期参数元信息
 * - 运行期执行器摘要
 *
 * 最后再把这些临时缓冲收拢成精确尺寸的最终结果。
 * The backend only walks the frontend once. During that single walk it collects:
 * - output bytecode
 * - long literal-text storage
 * - runtime argument metadata
 * - the runtime executor profile
 *
 * Then it packs those scratch buffers into the final exact-size result.
 */
template <typename Frontend>
class FormatCompiler
{
 private:
  static_assert(SourceFrontend<Frontend>,
                "LibXR::Print::FormatCompiler: frontend must expose ErrorType, "
                "SourceData(), and SourceSize()");

  using Error = typename Frontend::ErrorType;

  /**
   * @brief 当前后端实例产出的最终精确尺寸编译格式 / Final exact-size compiled format
   * produced by this backend instance
   * @tparam BlobBytes 最终保留字节块大小 / Final retained byte-block size
   * @tparam ArgCount 最终参数元信息个数 / Final argument metadata count
   */
  template <size_t BlobBytes, size_t ArgCount>
  struct ResultData
  {
    using CodeList =
        std::array<uint8_t, BlobBytes>;  ///< 最终运行期字节块 / final runtime byte block
    using ArgumentListData =
        std::array<FormatArgumentInfo,
                   ArgCount>;  ///< 按顺序排列的编译期参数元信息
                               ///< ordered compile-time argument metadata

    CodeList codes{};  ///< 前半记录流，后半文本池 / code stream first, text pool second
    ArgumentListData
        arg_info{};  ///< 按参数顺序排列的参数元信息 / ordered argument metadata
    FormatProfile profile =
        FormatProfile::None;  ///< 编译期执行器配置 / compile-time executor profile
    Error compile_error =
        Error::None;  ///< 首个编译期失败原因 / first compile-time failure, if any
  };

  template <size_t BlobBytes, size_t ArgCount>
  using Result = ResultData<BlobBytes, ArgCount>;

  /**
   * @brief 超过该长度后，字面文本会从内嵌模式切换为 TextRef / Maximum inline literal size
   * before the backend spills to TextRef.
   */
  static constexpr size_t inline_text_limit = 2 * sizeof(size_t) - 1;

  // 基于原始源串给单遍构建器预留的保守上界：
  // - 普通文本最坏可膨胀为每字节 3 个码流字节（TextInline）
  // - 值记录最坏约为每字节 2 个码流字节（如 %x -> 4 字节）
  // - 因此 3 * SourceSize() + 1 足以覆盖临时记录流与最终结束标记。
  // Conservative single-pass scratch bounds derived from the original source:
  // - literal text can cost up to 3 code bytes per source byte (TextInline)
  // - value records cost at most 2 code bytes per source byte (%x -> 4 bytes)
  // - therefore 3 * SourceSize() + 1 safely covers the temporary record stream
  //   plus the final End marker.
  static constexpr size_t max_code_bytes = 3 * Frontend::SourceSize() + 1;
  static constexpr size_t max_text_pool_bytes = Frontend::SourceSize();
  static constexpr size_t max_arg_count = Frontend::SourceSize();
  static constexpr uint8_t unspecified_precision = std::numeric_limits<uint8_t>::max();

  /**
   * @brief 向临时或最终字节缓冲区追加一个原始字节 / Append one raw byte into a scratch or
   * final byte buffer
   * @param data 目标字节缓冲区 / Target byte buffer
   * @param out 当前写位置；会前进一个字节 / Current write position; advanced by one byte
   * @param value 待追加的字节值 / Byte value to append
   */
  static consteval void EmitByte(auto& data, size_t& out, uint8_t value)
  {
    data[out++] = value;
  }

  /**
   * @brief 向临时或最终字节缓冲区追加一个按本机字节序编码的 POD 值 / Append one
   * native-endian POD value into a scratch or final byte buffer
   * @tparam T 待编码的 POD 类型 / POD type to encode
   * @param data 目标字节缓冲区 / Target byte buffer
   * @param out 当前写位置；会前进 `sizeof(T)` / Current write position; advanced by
   * `sizeof(T)`
   * @param value 待追加的 POD 值 / POD value to append
   */
  template <typename T>
  static consteval void EmitNative(auto& data, size_t& out, T value)
  {
    auto bytes = std::bit_cast<std::array<uint8_t, sizeof(T)>>(value);
    for (auto byte : bytes)
    {
      data[out++] = byte;
    }
  }

  /**
   * @brief 从临时字节缓冲区读取一个按本机字节序编码的 POD 值 / Read one native-endian POD
   * value from a scratch byte buffer
   * @tparam T 待读取的 POD 类型 / POD type to decode
   * @param data 源字节缓冲区 / Source byte buffer
   * @param pos 当前读取位置；会前进 `sizeof(T)` / Current read position; advanced by
   * `sizeof(T)`
   * @return 返回解码后的 POD 值 / Returns the decoded POD value
   */
  template <typename T>
  [[nodiscard]] static consteval T ReadNative(const auto& data, size_t& pos)
  {
    std::array<uint8_t, sizeof(T)> bytes{};
    for (size_t i = 0; i < sizeof(T); ++i)
    {
      bytes[i] = data[pos++];
    }
    return std::bit_cast<T>(bytes);
  }

  /**
   * @brief 构造只携带编译错误的最小失败结果 / Builds the minimal failed result carrying
   * only the compile error.
   * @param error 编译期失败类别 / Compile-time failure category.
   * @return 返回最小失败结果对象 / Returns a minimal failed result object.
   */
  [[nodiscard]] static consteval auto Failed(Error error)
  {
    Result<1, 0> result{};
    result.compile_error = error;
    return result;
  }

  /**
   * @brief 指出某个操作码族需要打开哪一个 writer-profile 位 / Says which writer-profile
   * bit one opcode family needs.
   * @param op 待分类的操作码 / Opcode to classify
   * @return 对应的窄路径 profile 位；无需窄路径时返回 `None` / Required narrow-path
   * profile bit, or `None` when no narrow path is needed
   */
  [[nodiscard]] static consteval FormatProfile ProfileForOp(FormatOp op)
  {
    switch (op)
    {
      case FormatOp::U32Dec:
      case FormatOp::U32ZeroPadWidth:
      case FormatOp::I32Dec:
      case FormatOp::U32Binary:
      case FormatOp::U32Octal:
      case FormatOp::U32HexLower:
      case FormatOp::U32HexUpper:
        return FormatProfile::NarrowInt;
      case FormatOp::StringRaw:
      case FormatOp::CharacterRaw:
        return FormatProfile::TextArg;
      case FormatOp::GenericField:
        return FormatProfile::None;
      case FormatOp::TextInline:
      case FormatOp::TextRef:
      case FormatOp::TextSpace:
      case FormatOp::End:
        return FormatProfile::None;
    }

    return FormatProfile::None;
  }

  /**
   * @brief 为一个字段选择仍能正确打印它的最小操作码 / Chooses the smallest opcode that
   * can still print this field correctly.
   */
  [[nodiscard]] static consteval FormatOp FastFieldOp(const FormatField& field)
  {
    const bool raw_integer_field = IsRawIntegerField(field);
    const bool raw_text_field = IsRawTextField(field);

    if (raw_integer_field && field.type == FormatType::Signed32 &&
        field.pack == FormatPackKind::I32)
    {
      return FormatOp::I32Dec;
    }

    if (field.type == FormatType::Unsigned32 && field.pack == FormatPackKind::U32 &&
        raw_integer_field)
    {
      return FormatOp::U32Dec;
    }

    if (raw_integer_field && field.type == FormatType::Binary32 &&
        field.pack == FormatPackKind::U32)
    {
      return FormatOp::U32Binary;
    }

    if (raw_integer_field && field.type == FormatType::Octal32 &&
        field.pack == FormatPackKind::U32)
    {
      return FormatOp::U32Octal;
    }

    if (raw_integer_field && field.type == FormatType::HexLower32 &&
        field.pack == FormatPackKind::U32)
    {
      return FormatOp::U32HexLower;
    }

    if (raw_integer_field && field.type == FormatType::HexUpper32 &&
        field.pack == FormatPackKind::U32)
    {
      return FormatOp::U32HexUpper;
    }

    if (raw_text_field && field.type == FormatType::Character &&
        field.pack == FormatPackKind::Character)
    {
      return FormatOp::CharacterRaw;
    }

    if (raw_text_field && field.type == FormatType::String &&
        field.pack == FormatPackKind::StringView)
    {
      return FormatOp::StringRaw;
    }

    if (field.type == FormatType::Unsigned32 && field.pack == FormatPackKind::U32 &&
        field.flags == static_cast<uint8_t>(FormatFlag::ZeroPad) && field.fill == ' ' &&
        field.width != 0 && field.precision == unspecified_precision)
    {
      return FormatOp::U32ZeroPadWidth;
    }

    return FormatOp::GenericField;
  }

  /**
   * @brief 判断字段是否满足裸整数快路径条件 / Tests whether one field matches the raw
   * integer fast-path gate.
   */
  [[nodiscard]] static consteval bool IsRawIntegerField(const FormatField& field)
  {
    return field.flags == 0 && field.fill == ' ' && field.width == 0 &&
           field.precision == unspecified_precision;
  }

  /**
   * @brief 判断字段是否满足裸文本快路径条件 / Tests whether one field matches the raw
   * text fast-path gate.
   */
  [[nodiscard]] static consteval bool IsRawTextField(const FormatField& field)
  {
    return (field.flags == 0 ||
            field.flags == static_cast<uint8_t>(FormatFlag::LeftAlign)) &&
           field.fill == ' ' && field.width == 0 &&
           field.precision == unspecified_precision;
  }

  /**
   * @brief 由前端文本/字段事件直接喂给的单遍临时构建器 / One-pass scratch builder fed
   * directly by frontend text/field events.
   */
  struct ScratchBuilder
  {
    std::array<uint8_t, max_code_bytes>
        code_scratch{};  ///< 临时记录流，不含最终 End
                         ///< temporary record stream without final End
    std::array<uint8_t, max_text_pool_bytes>
        text_scratch{};  ///< 临时尾部文本池 / temporary trailing text pool
    std::array<FormatArgumentInfo, max_arg_count>
        arg_scratch{};  ///< 临时参数元信息表 / temporary ordered argument metadata

    size_t code_bytes =
        0;  ///< 临时记录流字节数，不含 End / scratch record-stream bytes, excluding End
    size_t text_pool_bytes = 0;  ///< 临时文本池字节数 / scratch text-pool bytes
    size_t arg_count = 0;        ///< 运行期参数个数 / consumed runtime arguments
    FormatProfile profile = FormatProfile::None;  ///< 需要的运行期执行器配置
                                                  ///< required runtime executor profile
    Error frontend_error = Error::None;  ///< 首个前端失败原因 / first frontend failure

    /**
     * @brief 将单个字面文本片段追加到临时缓冲区 / Appends one literal-text span into
     * the scratch buffers.
     * @param offset 文本在 `Frontend::SourceData()` 中的偏移。
     *        Text offset inside `Frontend::SourceData()`.
     * @param text_size 文本字节数 / Text byte count.
     * @return 成功返回 `Error::None`；写入临时缓冲区失败时返回对应错误。
     *         Returns the first source-to-scratch conversion error, or `Error::None` on
     *         success.
     */
    [[nodiscard]] consteval Error Text(size_t offset, size_t text_size)
    {
      if (text_size == 0)
      {
        return Error::None;
      }

      if (text_size == 1 && Frontend::SourceData()[offset] == ' ')
      {
        EmitByte(code_scratch, code_bytes, static_cast<uint8_t>(FormatOp::TextSpace));
        return Error::None;
      }

      if (text_size <= inline_text_limit)
      {
        EmitByte(code_scratch, code_bytes, static_cast<uint8_t>(FormatOp::TextInline));
        for (size_t i = 0; i < text_size; ++i)
        {
          EmitByte(code_scratch, code_bytes,
                   static_cast<uint8_t>(Frontend::SourceData()[offset + i]));
        }
        EmitByte(code_scratch, code_bytes, 0);
        return Error::None;
      }

      if (text_pool_bytes > std::numeric_limits<uint16_t>::max())
      {
        return Error::TextOffsetOverflow;
      }
      if (text_size > std::numeric_limits<uint16_t>::max())
      {
        return Error::TextSizeOverflow;
      }

      EmitByte(code_scratch, code_bytes, static_cast<uint8_t>(FormatOp::TextRef));
      EmitNative(code_scratch, code_bytes, static_cast<uint16_t>(text_pool_bytes));
      EmitNative(code_scratch, code_bytes, static_cast<uint16_t>(text_size));

      for (size_t i = 0; i < text_size; ++i)
      {
        text_scratch[text_pool_bytes++] =
            static_cast<uint8_t>(Frontend::SourceData()[offset + i]);
      }

      return Error::None;
    }

    /**
     * @brief 将一个值字段追加到临时字节流和临时参数表中 / Appends one value field into
     * the scratch byte stream and scratch argument table.
     * @param field 前端产出的共享字段记录 / Shared field record produced by the frontend.
     * @return 返回字段追加结果 / Returns the field append result.
     */
    [[nodiscard]] consteval Error Field(const FormatField& field)
    {
      auto op = FastFieldOp(field);
      EmitByte(code_scratch, code_bytes, static_cast<uint8_t>(op));

      switch (op)
      {
        case FormatOp::U32Dec:
        case FormatOp::I32Dec:
        case FormatOp::U32Binary:
        case FormatOp::U32Octal:
        case FormatOp::U32HexLower:
        case FormatOp::U32HexUpper:
        case FormatOp::StringRaw:
        case FormatOp::CharacterRaw:
          break;
        case FormatOp::U32ZeroPadWidth:
          EmitByte(code_scratch, code_bytes, field.width);
          break;
        case FormatOp::GenericField:
          EmitByte(code_scratch, code_bytes, static_cast<uint8_t>(field.type));
          EmitByte(code_scratch, code_bytes, field.flags);
          EmitByte(code_scratch, code_bytes, static_cast<uint8_t>(field.fill));
          EmitByte(code_scratch, code_bytes, field.width);
          EmitByte(code_scratch, code_bytes, field.precision);
          break;
        case FormatOp::TextInline:
        case FormatOp::TextRef:
        case FormatOp::TextSpace:
        case FormatOp::End:
          break;
      }

      profile |=
          op == FormatOp::GenericField ? GenericProfileFor(field.type) : ProfileForOp(op);

      arg_scratch[arg_count++] = FormatArgumentInfo{
          .pack = field.pack,
          .rule = field.rule,
      };
      return Error::None;
    }

    /**
     * @brief 含 End 结束标记在内的最终记录流字节数 / Final record-stream size including
     * the End marker.
     * @return 返回最终记录流字节数 / Returns the final record-stream byte count.
     */
    [[nodiscard]] constexpr size_t FinalCodeBytes() const { return code_bytes + 1; }
    /**
     * @brief 含尾部文本池在内的最终保留字节块大小 / Final retained byte-block size
     * including the trailing text pool.
     * @return 返回最终保留字节块总大小 / Returns the total retained byte-block size.
     */
    [[nodiscard]] constexpr size_t FinalBlobBytes() const
    {
      return FinalCodeBytes() + text_pool_bytes;
    }

    /**
     * @brief 把临时缓冲打包成最终精确尺寸的编译格式 / Packs the scratch buffers into
     * the final exact-size compiled format.
     *
     * TextRef 记录在临时阶段保存的是相对文本池起点的偏移；收尾阶段会把它们重定位为
     * 指向最终 codes 字节块尾部文本池的绝对偏移。
     * TextRef records store temporary offsets relative to the text scratch pool.
     * Finalization rebases those offsets so they point at the trailing text pool
     * inside the final codes byte block.
     * @tparam CodeBytes 含 `End` 在内的最终记录流字节数。
     *         Final record-stream byte count including `End`.
     * @tparam BlobBytes 最终总字节块大小 / Final total byte-block size
     * @tparam ArgCount 最终参数元信息个数 / Final argument metadata count
     * @return 精确尺寸的编译结果 / Returns the exact-size compiled result
     */
    template <size_t CodeBytes, size_t BlobBytes, size_t ArgCount>
    [[nodiscard]] consteval auto Finish() const
    {
      Result<BlobBytes, ArgCount> result{};
      result.profile = profile;
      size_t scratch_in = 0;
      size_t code_out = 0;

      while (scratch_in < code_bytes)
      {
        auto op = static_cast<FormatOp>(code_scratch[scratch_in++]);
        EmitByte(result.codes, code_out, static_cast<uint8_t>(op));

        if (op == FormatOp::TextInline)
        {
          while (true)
          {
            uint8_t byte = code_scratch[scratch_in++];
            EmitByte(result.codes, code_out, byte);
            if (byte == 0)
            {
              break;
            }
          }
          continue;
        }

        const size_t payload_bytes = FormatOpPayloadBytes(op);
        if (payload_bytes == 0)
        {
          continue;
        }

        if (op == FormatOp::TextRef)
        {
          auto relative_offset = ReadNative<uint16_t>(code_scratch, scratch_in);
          auto text_size = ReadNative<uint16_t>(code_scratch, scratch_in);
          size_t absolute_offset = CodeBytes + relative_offset;
          if (absolute_offset > std::numeric_limits<uint16_t>::max())
          {
            result.compile_error = Error::TextOffsetOverflow;
            return result;
          }
          EmitNative(result.codes, code_out, static_cast<uint16_t>(absolute_offset));
          EmitNative(result.codes, code_out, text_size);
          continue;
        }

        if (payload_bytes == 1)
        {
          EmitByte(result.codes, code_out, code_scratch[scratch_in++]);
          continue;
        }

        if (op == FormatOp::GenericField)
        {
          for (size_t i = 0; i < payload_bytes; ++i)
          {
            EmitByte(result.codes, code_out, code_scratch[scratch_in++]);
          }
        }
      }

      EmitByte(result.codes, code_out, static_cast<uint8_t>(FormatOp::End));

      for (size_t i = 0; i < text_pool_bytes; ++i)
      {
        result.codes[CodeBytes + i] = text_scratch[i];
      }
      for (size_t i = 0; i < ArgCount; ++i)
      {
        result.arg_info[i] = arg_scratch[i];
      }

      return result;
    }
  };

  static_assert(WalkableFrontend<Frontend, ScratchBuilder>,
                "LibXR::Print::FormatCompiler: frontend Walk(visitor) must accept "
                "the shared builder and return ErrorType");

 public:
  /**
   * @brief 把一个前端编译成最终字节流、参数表和 writer 摘要 / Compiles one frontend
   * into the final byte stream, argument table, and writer profile.
   * @return 返回最终编译结果对象；若失败则返回携带首个编译期错误的失败结果。
   *         Returns the final compiled result object, or a failed result that carries the
   *         first compile-time error.
   */
  [[nodiscard]] static consteval auto Compile()
  {
    constexpr auto scratch = []() consteval
    {
      ScratchBuilder builder{};
      builder.frontend_error = Frontend::Walk(builder);
      return builder;
    }();

    if constexpr (scratch.frontend_error != Error::None)
    {
      return Failed(scratch.frontend_error);
    }
    else
    {
      return scratch.template Finish<scratch.FinalCodeBytes(), scratch.FinalBlobBytes(),
                                     scratch.arg_count>();
    }
  }
};
}  // namespace LibXR::Print
