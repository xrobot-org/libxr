#pragma once

/**
 * @brief 运行期执行器共享的字段写出原语，供所有操作码路径复用 / Executor-side
 * field-writing primitives shared by all runtime opcodes
 */

/**
 * @brief 将一段原始文本直接写入输出端 / Write one raw text chunk directly into the sink
 * @param text 待写出的文本片段 / Text chunk to write.
 * @return 返回 sink 写出结果 / Returns the sink write result.
 */
template <OutputSink Sink>
ErrorCode Writer::Executor<Sink>::WriteRaw(std::string_view text)
{
  return sink_.Write(text);
}

/**
 * @brief 向输出端写入重复填充字符 / Write repeated fill characters into the sink
 * @param fill 填充字符 / Fill character
 * @param count 重复次数 / Repeat count
 * @return 返回首个 sink 错误；全部填充字节写出后返回 `ErrorCode::OK` / Returns the first
 * sink error, or `ErrorCode::OK` when all fill bytes are written
 */
template <OutputSink Sink>
ErrorCode Writer::Executor<Sink>::WritePadding(char fill, size_t count)
{
  char chunk[16];
  std::memset(chunk, fill, sizeof(chunk));

  while (count != 0)
  {
    size_t step = (count < sizeof(chunk)) ? count : sizeof(chunk);
    auto ec = WriteRaw(std::string_view(chunk, step));
    if (ec != ErrorCode::OK)
    {
      return ec;
    }
    count -= step;
  }

  return ErrorCode::OK;
}

/**
 * @brief 按宽度与对齐策略写出一个文本字段 / Write one text field with width/alignment
 * policy applied
 * @param text 上层归一化后的文本载荷 / Text payload after higher-level normalization
 * @param spec 解码后的字段规格 / Decoded field spec
 * @return 返回首个 sink 错误；成功时返回 `ErrorCode::OK` / Returns the first sink error,
 * or `ErrorCode::OK` on success
 */
template <OutputSink Sink>
ErrorCode Writer::Executor<Sink>::WriteTextField(std::string_view text, const Spec& spec)
{
  size_t pad = FieldPadding(spec.width, text.size());
  size_t left_pad = 0;
  size_t right_pad = 0;
  if (spec.LeftAlign())
  {
    right_pad = pad;
  }
  else if (spec.CenterAlign())
  {
    left_pad = pad / 2;
    right_pad = pad - left_pad;
  }
  else
  {
    left_pad = pad;
  }

  if (auto ec = WritePadding(spec.fill, left_pad); ec != ErrorCode::OK)
  {
    return ec;
  }
  if (auto ec = WriteRaw(text); ec != ErrorCode::OK)
  {
    return ec;
  }
  return WritePadding(spec.fill, right_pad);
}

/**
 * @brief 按符号、前缀、精度与填充策略写出一个整数载荷 / Write one integer payload with
 * sign, prefix, precision, and padding policy applied
 * @param sign_char 最终可见的符号字符；无符号时为 `'\0'` / Visible sign character, or
 * `'\0'` when absent
 * @param prefix 输出在数字前面的前缀 / Prefix emitted ahead of the digits
 * @param digits 十进制、二进制、八进制或十六进制数字载荷 / Decimal, binary, octal, or hex
 * digit payload
 * @param spec 解码后的字段规格 / Decoded field spec
 * @return 返回首个 sink 错误；成功时返回 `ErrorCode::OK` / Returns the first sink error,
 * or `ErrorCode::OK` on success
 */
template <OutputSink Sink>
ErrorCode Writer::Executor<Sink>::WriteIntegerField(char sign_char,
                                                    std::string_view prefix,
                                                    std::string_view digits,
                                                    const Spec& spec)
{
  auto write_char = [this](char ch) -> ErrorCode
  {
    if (ch == '\0')
    {
      return ErrorCode::OK;
    }
    return WriteRaw(std::string_view(&ch, 1));
  };
  auto write_text = [this](std::string_view text) -> ErrorCode
  {
    if (text.empty())
    {
      return ErrorCode::OK;
    }
    return WriteRaw(text);
  };

  size_t zeros = IntegerPrecisionZeros(spec, digits.size());
  size_t total =
      digits.size() + zeros + prefix.size() + static_cast<size_t>(sign_char != '\0');
  size_t pad = FieldPadding(spec.width, total);
  bool zero_fill =
      spec.ZeroPad() && !spec.LeftAlign() && !spec.CenterAlign() && !spec.HasPrecision();
  size_t left_pad = 0;
  size_t middle_zeros = zero_fill ? pad : 0;
  size_t right_pad = 0;
  if (!zero_fill)
  {
    if (spec.LeftAlign())
    {
      right_pad = pad;
    }
    else if (spec.CenterAlign())
    {
      left_pad = pad / 2;
      right_pad = pad - left_pad;
    }
    else
    {
      left_pad = pad;
    }
  }

  if (auto ec = WritePadding(spec.fill, left_pad); ec != ErrorCode::OK)
  {
    return ec;
  }
  if (auto ec = write_char(sign_char); ec != ErrorCode::OK)
  {
    return ec;
  }
  if (auto ec = write_text(prefix); ec != ErrorCode::OK)
  {
    return ec;
  }
  if (auto ec = WritePadding('0', middle_zeros); ec != ErrorCode::OK)
  {
    return ec;
  }
  if (auto ec = WritePadding('0', zeros); ec != ErrorCode::OK)
  {
    return ec;
  }
  if (auto ec = write_text(digits); ec != ErrorCode::OK)
  {
    return ec;
  }
  return WritePadding(spec.fill, right_pad);
}

/**
 * @brief 按符号与字段填充策略写出一个浮点文本载荷 / Write one float text payload with
 * sign and field padding applied
 * @param sign_char 最终可见的符号字符；无符号时为 `'\0'` / Visible sign character, or
 * `'\0'` when absent
 * @param text 已完成格式化的浮点文本载荷 / Already formatted float text payload
 * @param spec 解码后的字段规格 / Decoded field spec
 * @return 返回首个 sink 错误；成功时返回 `ErrorCode::OK` / Returns the first sink error,
 * or `ErrorCode::OK` on success
 */
template <OutputSink Sink>
ErrorCode Writer::Executor<Sink>::WriteFloatField(char sign_char, std::string_view text,
                                                  const Spec& spec)
{
  auto write_char = [this](char ch) -> ErrorCode
  {
    if (ch == '\0')
    {
      return ErrorCode::OK;
    }
    return WriteRaw(std::string_view(&ch, 1));
  };

  size_t total = text.size() + static_cast<size_t>(sign_char != '\0');
  size_t pad = FieldPadding(spec.width, total);
  bool zero_fill = spec.ZeroPad() && !spec.LeftAlign() && !spec.CenterAlign();
  size_t left_pad = 0;
  size_t middle_zeros = zero_fill ? pad : 0;
  size_t right_pad = 0;
  if (!zero_fill)
  {
    if (spec.LeftAlign())
    {
      right_pad = pad;
    }
    else if (spec.CenterAlign())
    {
      left_pad = pad / 2;
      right_pad = pad - left_pad;
    }
    else
    {
      left_pad = pad;
    }
  }

  if (auto ec = WritePadding(spec.fill, left_pad); ec != ErrorCode::OK)
  {
    return ec;
  }
  if (auto ec = write_char(sign_char); ec != ErrorCode::OK)
  {
    return ec;
  }
  if (auto ec = WritePadding('0', middle_zeros); ec != ErrorCode::OK)
  {
    return ec;
  }
  if (auto ec = WriteRaw(text); ec != ErrorCode::OK)
  {
    return ec;
  }
  return WritePadding(spec.fill, right_pad);
}
