#pragma once

/**
 * @brief writer 执行器的顶层运行期操作码循环 / Top-level runtime opcode loop for the
 * writer executor.
 */

/**
 * @brief 将一个输出端、码流和参数字节块绑定到当前执行器 / Bind one sink, code stream, and
 * packed-argument blob to this executor
 * @param sink 输出端 / Destination sink
 * @param codes 指向编译字节流的指针 / Pointer to the compiled byte stream
 * @param args 指向已打包参数字节块的指针；无参数时可为空 / Pointer to the packed argument
 * blob, or null when no arguments exist
 */
template <OutputSink Sink>
Writer::Executor<Sink>::Executor(Sink& sink, const uint8_t* codes, const uint8_t* args)
    : sink_(sink), codes_(codes), args_(args)
{
}

/**
 * @brief 运行操作码循环，直到遇到 End 或首个 sink/运行期错误 / Runs the opcode loop until
 * End or the first sink/runtime error.
 * @tparam Profile 当前编译格式需要的操作码配置 / Opcode profile required by the current
 * compiled format
 * @return 正常结束返回 `ErrorCode::OK`；否则返回首个 sink/运行期错误。
 *         Returns `ErrorCode::OK` on normal completion, or the first sink / runtime
 *         error.
 */
template <OutputSink Sink>
template <FormatProfile Profile>
ErrorCode Writer::Executor<Sink>::Run()
{
  while (true)
  {
    auto op = codes_.ReadOp();
    if (op == FormatOp::End)
    {
      return ErrorCode::OK;
    }

    auto ec = DispatchOp<Profile>(op);
    if (ec != ErrorCode::OK)
    {
      return ec;
    }
  }
}

/**
 * @brief 将一个顶层操作码分发到其特化运行期路径 / Dispatches one top-level opcode to its
 * specialized runtime path.
 * @tparam Profile 当前编译格式需要的操作码配置 / Opcode profile required by the current
 * compiled format
 * @param op 解码后的运行期操作码 / Decoded runtime opcode.
 * @return 返回该操作码对应特化路径的运行结果。
 *         Returns the specialized runtime result for that opcode.
 */
template <OutputSink Sink>
template <FormatProfile Profile>
ErrorCode Writer::Executor<Sink>::DispatchOp(FormatOp op)
{
  switch (op)
  {
    case FormatOp::TextInline:
      return WriteRaw(codes_.ReadInlineText());
    case FormatOp::TextRef:
      return WriteRaw(codes_.ReadTextRef());
    case FormatOp::TextSpace:
      return WriteRaw(" ");
    case FormatOp::U32Dec:
      if constexpr (!HasProfile(Profile, FormatProfile::NarrowInt) ||
                    !Config::enable_integer)
      {
        return ErrorCode::STATE_ERR;
      }
      return WriteU32Dec(args_.Read<uint32_t>());
    case FormatOp::I32Dec:
      if constexpr (!HasProfile(Profile, FormatProfile::NarrowInt) ||
                    !Config::enable_integer)
      {
        return ErrorCode::STATE_ERR;
      }
      return WriteI32Dec(args_.Read<int32_t>());
    case FormatOp::U32ZeroPadWidth:
      if constexpr (!HasProfile(Profile, FormatProfile::NarrowInt) ||
                    !Config::enable_integer)
      {
        return ErrorCode::STATE_ERR;
      }
      return WriteU32ZeroPadWidth(codes_.Read<uint8_t>(), args_.Read<uint32_t>());
    case FormatOp::U32Binary:
      if constexpr (!HasProfile(Profile, FormatProfile::NarrowInt) ||
                    !Config::enable_integer || !Config::enable_integer_base8_16)
      {
        return ErrorCode::STATE_ERR;
      }
      return WriteU32Base<2>(args_.Read<uint32_t>());
    case FormatOp::U32Octal:
      if constexpr (!HasProfile(Profile, FormatProfile::NarrowInt) ||
                    !Config::enable_integer || !Config::enable_integer_base8_16)
      {
        return ErrorCode::STATE_ERR;
      }
      return WriteU32Base<8>(args_.Read<uint32_t>());
    case FormatOp::U32HexLower:
      if constexpr (!HasProfile(Profile, FormatProfile::NarrowInt) ||
                    !Config::enable_integer || !Config::enable_integer_base8_16)
      {
        return ErrorCode::STATE_ERR;
      }
      return WriteU32Base<16>(args_.Read<uint32_t>());
    case FormatOp::U32HexUpper:
      if constexpr (!HasProfile(Profile, FormatProfile::NarrowInt) ||
                    !Config::enable_integer || !Config::enable_integer_base8_16)
      {
        return ErrorCode::STATE_ERR;
      }
      return WriteU32Base<16, true>(args_.Read<uint32_t>());
    case FormatOp::StringRaw:
      if constexpr (!HasProfile(Profile, FormatProfile::TextArg) || !Config::enable_text)
      {
        return ErrorCode::STATE_ERR;
      }
      return WriteStringRaw(args_.Read<std::string_view>());
    case FormatOp::CharacterRaw:
      if constexpr (!HasProfile(Profile, FormatProfile::TextArg) || !Config::enable_text)
      {
        return ErrorCode::STATE_ERR;
      }
      return WriteCharacterRaw(args_.Read<char>());
    case FormatOp::GenericField:
      if constexpr (!HasProfile(Profile, FormatProfile::Generic))
      {
        return ErrorCode::STATE_ERR;
      }
      return DispatchGenericField<Profile>(codes_.ReadFormatType());
    case FormatOp::End:
    default:
      return ErrorCode::STATE_ERR;
  }
}
