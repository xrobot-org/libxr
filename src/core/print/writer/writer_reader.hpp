#pragma once

#include <cstdint>
#include <cstring>
#include <string_view>

/**
 * @brief 编译后记录流的顺序读取器 / Sequential reader for the compiled record stream
 */
class Writer::CodeReader
{
 public:
  /**
   * @brief 从单个编译字节块起点构造读取器 / Create a reader over the beginning of one
   * compiled byte blob
   * @param codes 指向单个编译字节块起点的指针 / Pointer to the beginning of one compiled
   * byte blob
   */
  explicit CodeReader(const uint8_t* codes);

  /**
   * @brief 读取下一条运行期操作码 / Read the next runtime opcode
   * @return 返回下一条解码后的运行期操作码 / Returns the next decoded runtime opcode
   */
  [[nodiscard]] FormatOp ReadOp();

  /**
   * @brief 读取编译期发射器按本机字节序写入的 POD 值 / Read a native-endian POD value
   * emitted by the compile-time emitter
   * @tparam T 待读取的 POD 类型 / POD type to read
   * @return 返回解码后的 POD 值 / Returns the decoded POD value
   */
  template <typename T>
  [[nodiscard]] T Read()
  {
    T value{};
    std::memcpy(&value, pos_, sizeof(T));
    pos_ += sizeof(T);
    return value;
  }

  /**
   * @brief 读取 `GenericField` 载荷中的语义类型字节 / Read the semantic type byte carried
   * by one `GenericField` payload
   * @return 返回解码后的字段语义类型 / Returns the decoded field semantic type
   */
  [[nodiscard]] FormatType ReadFormatType();

  /**
   * @brief 读取紧跟在 `GenericField` 类型字节后的 4 字节字段载荷 / Read the 4-byte field
   * payload that follows one `GenericField` type byte
   *
   * 操作码与语义类型字节由外层单独读取；本函数只继续读取后续
   * flags、fill、width、precision 这 4 个字节，顺序固定。
   * The opcode and semantic type bytes are read separately. This call only
   * consumes flags, fill, width, and precision, in that order.
   * @return 返回解码后的运行期字段规格 / Returns the decoded runtime field spec
   */
  [[nodiscard]] Spec ReadSpec();

  /**
   * @brief 读取内嵌在记录流中的短文本 / Read a null-terminated short text payload
   * embedded in the record stream
   * @return 返回该内嵌文本片段的视图 / Returns a view of the embedded text span
   */
  [[nodiscard]] std::string_view ReadInlineText();

  /**
   * @brief 读取指向尾部文本池的偏移和长度 / Read an offset-size pair pointing into the
   * trailing text pool
   *
   * 该偏移已经按最终代码块起点完成重定位。
   * The offset is already rebased against the final code blob base.
   * @return 返回尾部文本池中被引用的文本片段 / Returns the referenced text span inside
   * the trailing text pool
   */
  [[nodiscard]] std::string_view ReadTextRef();

 private:
  const uint8_t* pos_ = nullptr;
  const uint8_t* base_ = nullptr;
};

/**
 * @brief 运行期参数字节块的顺序读取器 / Sequential reader for the packed runtime argument
 * byte blob
 */
class Writer::ArgumentReader
{
 public:
  /**
   * @brief 从单个运行期参数打包字节块构造读取器 / Create a reader over one packed runtime
   * argument blob
   * @param data 指向运行期参数打包字节块的指针 / Pointer to the packed runtime argument
   * blob
   */
  explicit ArgumentReader(const uint8_t* data);

  /**
   * @brief 以无对齐要求的方式读取一个已打包参数值 / Read one packed argument value
   * without requiring alignment
   * @tparam T 待读取的已打包参数类型 / Packed argument type to read
   * @return 返回解码后的已打包参数值 / Returns the decoded packed argument value
   */
  template <typename T>
  [[nodiscard]] T Read()
  {
    T value{};
    std::memcpy(&value, pos_, sizeof(T));
    pos_ += sizeof(T);
    return value;
  }

 private:
  const uint8_t* pos_ = nullptr;
};
