#pragma once

/**
 * @brief 按输出端共享重后端、按调用点 profile 裁剪操作码分发的字节码执行器 / Per-sink
 * bytecode executor with shared heavy backends and per-call profile-pruned opcode
 * dispatch
 * @tparam Sink 输出端类型，需满足 `OutputSink` / Sink type satisfying `OutputSink`
 */
template <OutputSink Sink>
class Writer::Executor
{
 public:
  /**
   * @brief 将一个输出端、一份编译字节块和一份参数字节块绑定起来 / Bind one sink with one
   * compiled byte blob and one packed argument blob
   * @param sink 输出端 / Destination sink
   * @param codes 指向编译字节块的指针 / Pointer to the compiled byte blob
   * @param args 指向运行期参数打包字节块的指针；无参数时可为空 / Pointer to the packed
   * runtime argument blob, or null when no arguments exist
   */
  Executor(Sink& sink, const uint8_t* codes, const uint8_t* args);

  /**
   * @brief 持续执行记录流，直到遇到 `FormatOp::End` / Run until the compiled record
   * stream reaches `FormatOp::End`
   * @tparam Profile 当前编译格式需要的操作码配置 / Opcode profile required by the current
   * compiled format
   * @return 返回首个 sink 或运行期错误；正常执行到记录流结束时返回 `ErrorCode::OK` /
   * Returns the first sink or runtime error, or `ErrorCode::OK` when the record stream
   * finishes normally
   */
  template <FormatProfile Profile>
  [[nodiscard]] ErrorCode Run();

 private:
  // 原始输出与通用字段写出辅助函数 / Raw sink and generic field-writing helpers.
  [[nodiscard]] ErrorCode WriteRaw(std::string_view text);
  [[nodiscard]] ErrorCode WritePadding(char fill, size_t count);
  [[nodiscard]] ErrorCode WriteTextField(std::string_view text, const Spec& spec);
  [[nodiscard]] ErrorCode WriteIntegerField(char sign_char, std::string_view prefix,
                                            std::string_view digits, const Spec& spec);
  [[nodiscard]] ErrorCode WriteFloatField(char sign_char, std::string_view text,
                                          const Spec& spec);

  template <std::signed_integral Int>
  [[nodiscard]] static char ResolveSignChar(Int value, const Spec& spec);

#if LIBXR_PRINT_ENABLE_FLOAT
  template <typename T>
  [[nodiscard]] static char ResolveFloatSignChar(T value, const Spec& spec);
#endif

  template <std::signed_integral Int>
  [[nodiscard]] ErrorCode WriteSigned(const Spec& spec, Int value);

  template <FormatType Type, std::unsigned_integral UInt>
  [[nodiscard]] ErrorCode DispatchUnsigned(const Spec& spec, UInt value);

  /**
   * @brief 按编译期进制/大小写/八进制备用格式参数复用无符号数字载荷写出逻辑 / Reuse the
   * unsigned-digit payload writer with compile-time radix, case, and octal-alternate
   * parameters
   * @tparam Base 整数进制 / Integer radix
   * @tparam UpperCase 十六进制数字是否使用大写字符 / Whether hexadecimal digits should
   * use uppercase characters
   * @tparam PrependOctalZero 是否把 `%#o` 的前导 `0` 直接并入数字载荷 / Whether `%#o`
   * should inline its leading `0` into the digit payload
   * @tparam UInt 无符号整数类型 / Unsigned integer type
   * @param prefix 脱离数字载荷输出的前缀 / Prefix emitted outside the digit payload
   * @param spec 解码后的字段规格 / Decoded field spec
   * @param value 待写出的整数值 / Integer value to write
   * @return 返回共享无符号数字写出路径的结果 / Returns the shared unsigned-digit write
   * result
   */
  template <uint8_t Base, bool UpperCase = false, bool PrependOctalZero = false,
            std::unsigned_integral UInt>
  [[nodiscard]] ErrorCode WriteUnsignedDigits(std::string_view prefix, const Spec& spec,
                                              UInt value);
  [[nodiscard]] ErrorCode WritePointer(const Spec& spec, uintptr_t value);
  [[nodiscard]] ErrorCode WriteCharacter(const Spec& spec, char ch);
  [[nodiscard]] ErrorCode WriteString(const Spec& spec, std::string_view text);

#if LIBXR_PRINT_ENABLE_FLOAT
  template <typename T>
  [[nodiscard]] ErrorCode WriteFloat(FormatType type, const Spec& spec, T value);
#endif

  /**
   * @brief 单个原始 uint32_t 十进制字段的快路径 / Fast path for one raw uint32_t
   * decimal field.
   */
  [[nodiscard]] ErrorCode WriteU32Dec(uint32_t value);

  /**
   * @brief 单个原始 int32_t 十进制字段的快路径 / Fast path for one raw int32_t decimal
   * field.
   */
  [[nodiscard]] ErrorCode WriteI32Dec(int32_t value);

  /**
   * @brief 单个原始 uint32_t 非十进制字段的快路径 / Fast path for one raw uint32_t
   * non-decimal field.
   */
  template <uint8_t Base, bool UpperCase = false>
  [[nodiscard]] ErrorCode WriteU32Base(uint32_t value);

  /**
   * @brief 单个零填充 uint32_t 十进制字段的快路径 / Fast path for one zero-padded
   * uint32_t decimal field.
   */
  [[nodiscard]] ErrorCode WriteU32ZeroPadWidth(uint8_t width, uint32_t value);

  /**
   * @brief 单个原始字符串参数的快路径 / Fast path for one raw string argument.
   */
  [[nodiscard]] ErrorCode WriteStringRaw(std::string_view text);

  /**
   * @brief 单个原始字符参数的快路径 / Fast path for one raw character argument.
   */
  [[nodiscard]] ErrorCode WriteCharacterRaw(char ch);

  // 这些小桥接函数只负责让 GenericField 分发更易读，同时保持原有的
  // “读 spec -> 读下一个已打包参数 -> 调具体 writer” 执行顺序不变。
  // Small bridges that keep GenericField dispatch readable while preserving the
  // existing "read spec -> read next packed argument -> call concrete writer"
  // execution order.
  template <std::signed_integral Int>
  [[nodiscard]] ErrorCode DispatchSignedField();

  template <FormatType Type, std::unsigned_integral UInt>
  [[nodiscard]] ErrorCode DispatchUnsignedField();

#if LIBXR_PRINT_ENABLE_FLOAT
  template <FormatType Type, typename Float>
  [[nodiscard]] ErrorCode DispatchFloatField();
#endif

  [[nodiscard]] ErrorCode DispatchPointerField();

  [[nodiscard]] ErrorCode DispatchCharacterField();

  [[nodiscard]] ErrorCode DispatchStringField();

  /**
   * @brief 将一个 `GenericField` 载荷分发到对应的宽回退路径 / Dispatch one `GenericField`
   * payload to the corresponding wide fallback
   * @tparam Profile 当前编译格式使用的精确通用字段类型集合 / Exact generic-field type set
   * used by the current compiled format
   * @param type `GenericField` 记录携带的语义字段类型 / Semantic field type carried by
   * the `GenericField` record
   * @return 返回该字段的具体写出结果 / Returns the concrete writer result for that field
   */
  template <FormatProfile Profile>
  [[nodiscard]] ErrorCode DispatchGenericField(FormatType type);

  /**
   * @brief 将一个运行期操作码分发到选中的特化路径 / Dispatch one runtime opcode to the
   * selected specialized path
   * @tparam Profile 当前编译格式需要的操作码配置 / Opcode profile required by the current
   * compiled format
   * @param op 解码后的运行期操作码 / Decoded runtime opcode
   * @return 返回该操作码对应特化路径的运行结果 / Returns the specialized runtime result
   * for that opcode
   */
  template <FormatProfile Profile>
  [[nodiscard]] ErrorCode DispatchOp(FormatOp op);

  Sink& sink_;
  CodeReader codes_;
  ArgumentReader args_;
};

// These implementation headers form an ordered dependency chain.
// clang-format off
#include "writer_executor_field.hpp"
#include "writer_executor_value.hpp"
#include "writer_executor_generic.hpp"
#include "writer_executor_opcode.hpp"
// clang-format on
