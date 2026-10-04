#pragma once

#include <cstddef>
#include <string_view>
#include <tuple>
#include <utility>

#include "mutex.hpp"
#include "print.hpp"
#include "read_port.hpp"
#include "write_port.hpp"

namespace LibXR
{

/**
 * @brief 提供静态全局的输入输出接口绑定与写会话管理 / STDIO interface for read and write
 * ports
 */
class STDIO
{
 public:
  // 共享的全局 stdio 绑定状态 / Shared global stdio binding state.
  // NOLINTBEGIN
  static inline ReadPort* read_ = nullptr;    ///< 读取端口 / Read port instance.
  static inline WritePort* write_ = nullptr;  ///< 写入端口 / Write port instance.
  static inline LibXR::Mutex* write_mutex_ =
      nullptr;  ///< 写入端口互斥锁 / Write port mutex.
  static inline LibXR::WritePort::Stream* write_stream_ =
      nullptr;  ///< 可选的外部托管写流 / Optional externally owned write stream.
                // NOLINTEND

 private:
  // brace 与 printf 两个前端共用的编译格式桥接层。
  // Compiled-format bridge shared by the brace and printf frontends.

  /**
   * @brief STDIO 编译格式会话使用的流式截断输出端 / Stream-backed truncating sink used by
   * one STDIO compiled-format session
   */
  class CompiledSink
  {
   public:
    /**
     * @brief 构造一个绑定到指定流的编译格式输出端 / Construct one compiled-format sink
     * bound to the given stream
     * @param stream 当前会话绑定的写入流 / Write stream bound to the current session
     */
    explicit CompiledSink(WritePort::Stream& stream);

    /**
     * @brief 追加一个文本片段；必要时按会话剩余空间直接截断 / Append one text chunk and
     * truncate directly to the remaining session capacity when needed
     * @param chunk 待写出的文本片段 / Text chunk to write
     * @return 写出状态；若会话写流失败则返回对应错误 / Write status; returns the
     * underlying stream error on failure
     */
    [[nodiscard]] ErrorCode Write(std::string_view chunk);

    /**
     * @brief 返回当前会话最终保留下来的字节数 / Return the retained byte count of the
     * current session
     * @return 当前会话最终保留下来的字节数 / Returns the retained byte count of the
     * current session
     */
    [[nodiscard]] size_t RetainedSize() const { return retained_size_; }

   private:
    WritePort::Stream& stream_;  ///< 接收保留字节的活动流会话。
                                 ///< Active stream session receiving retained bytes.
    size_t retained_size_ = 0;   ///< 当前已保留的字节数 / Bytes retained so far.
    bool saturated_ = false;     ///< 当前会话不再继续保留输出。
                                 ///< No more bytes should be retained in this session.
  };

  // 一次编译格式 STDIO 调用的类型擦除桥接函数。
  // Type-erased bridge for one compiled STDIO call.
  using CompiledWriteFun = ErrorCode (*)(void* context, CompiledSink& sink);

  /**
   * @brief 一次编译格式 STDIO 调用的模板上下文 / Template context for one compiled-format
   * STDIO call
   * @tparam CompiledFormat 编译后的格式类型 / Compiled format type
   * @tparam Args 转发保存的运行时参数类型列表 / Forwarded runtime argument types
   */
  template <typename CompiledFormat, typename... Args>
  struct CompiledCall
  {
    const CompiledFormat&
        format;  ///< 编译期已编译的格式对象 / Compile-time compiled format object.
    std::tuple<Args&&...> args;  ///< 转发保存的运行时参数 / Forwarded runtime arguments.

    /**
     * @brief 将当前模板上下文桥接到编译格式前端写入入口 / Bridge the current template
     * context into the compiled-format write entry
     * @param context 指向当前类型化调用上下文的指针 / Pointer to the current typed call
     * context
     * @param sink 当前会话使用的编译格式输出端 / Compiled-format sink used by the current
     * session
     * @return 编译格式写出状态 / Compiled-format write status
     */
    [[nodiscard]] static ErrorCode Write(void* context, CompiledSink& sink)
    {
      auto& compiled_call = *static_cast<CompiledCall*>(context);
      return std::apply(
          [&](auto&&... unpacked) -> ErrorCode
          {
            return Print::Write(sink, compiled_call.format,
                                std::forward<decltype(unpacked)>(unpacked)...);
          },
          compiled_call.args);
    }
  };

  /**
   * @brief 在指定 `Stream` 上执行一次完整的 STDIO 编译格式写入与收尾 / Run one complete
   * STDIO compiled-format write and finalize pass on the given `Stream`
   *
   * 该 helper 统一负责：构造 CompiledSink、调用前端桥接函数、再用
   * FinishWriteSession() 做最终提交或丢弃。
   * This helper centralizes: constructing CompiledSink, invoking the frontend bridge,
   * then finalizing through FinishWriteSession().
   * @param stream 本次会话绑定的写入流 / Write stream bound to the current session
   * @param context 前端桥接函数对应的类型擦除上下文 / Type-erased context consumed by the
   * frontend bridge
   * @param write_fun 前端桥接写入函数 / Frontend bridge write function
   * @return 当前 STDIO 会话最终提交的字节数；失败返回 `-1` / Byte count finally committed
   * by the current STDIO session; returns `-1` on failure
   */
  [[nodiscard]] static int WriteCompiledToStream(WritePort::Stream& stream, void* context,
                                                 CompiledWriteFun write_fun);

  /**
   * @brief 执行一次完整的 STDIO 编译格式流会话选择、写入与收尾 / Run one complete STDIO
   * compiled-format stream session: stream selection, write, and finalization
   *
   * 若当前已存在外部绑定的 write_stream_，则复用它；否则创建一个临时的
   * WritePort::Stream 供本次会话使用。
   * Reuses the externally bound write_stream_ when available; otherwise creates one
   * temporary WritePort::Stream for the current session.
   * @param context 前端桥接函数对应的类型擦除上下文 / Type-erased context consumed by the
   * frontend bridge
   * @param write_fun 前端桥接写入函数 / Frontend bridge write function
   * @return 当前 STDIO 会话最终提交的字节数；失败返回 `-1` / Byte count finally committed
   * by the current STDIO session; returns `-1` on failure
   */
  [[nodiscard]] static int WriteCompiledSession(void* context,
                                                CompiledWriteFun write_fun);

  /**
   * @brief 执行一次模板已知的 STDIO 编译格式会话 / Run one STDIO compiled-format session
   * whose format and argument types are already known at compile time
   *
   * 该 helper 只保留模板相关的最薄一层：拿共享会话，再把类型化调用对象交给
   * WriteCompiledSession()。
   * This helper keeps only the thinnest template layer: acquire the shared session, then
   * pass the typed call object into WriteCompiledSession().
   * @tparam Call 类型化编译格式调用对象 / Typed compiled-format call object
   * @param call 当前会话的类型化调用对象 / Typed call object for the current session
   * @return 当前 STDIO 会话最终提交的字节数；失败返回 `-1` / Byte count finally committed
   * by the current STDIO session; returns `-1` on failure
   */
  template <typename Call>
  [[nodiscard]] static int RunCompiledSession(Call& call)
  {
    if (!BeginWriteSession())
    {
      return -1;
    }

    return WriteCompiledSession(&call, &Call::Write);
  }

  /**
   * @brief 用一份已编译格式和一组运行时参数执行一次完整的 STDIO 写会话 / Run one complete
   * STDIO write session with one compiled format and one runtime argument pack
   * @tparam CompiledFormat 编译后的格式类型 / Compiled format type
   * @tparam Args 运行时参数类型列表 / Runtime argument types
   * @param format 编译后的格式对象 / Compiled format object
   * @param args 参与本次写会话的运行时参数 / Runtime arguments used by the current write
   * session
   * @return 当前 STDIO 会话最终提交的字节数；失败返回 `-1` / Byte count finally committed
   * by the current STDIO session; returns `-1` on failure
   */
  template <typename CompiledFormat, typename... Args>
  [[nodiscard]] static int RunCompiled(const CompiledFormat& format, Args&&... args)
  {
    CompiledCall<CompiledFormat, Args...> call{
        format, std::forward_as_tuple(std::forward<Args>(args)...)};
    return RunCompiledSession(call);
  }

  /**
   * @brief 获取一个共享的 STDIO 写入会话 / Acquire one shared STDIO write session
   * @return 成功获取共享写会话返回 `true`，否则返回 `false` / Returns `true` on success,
   * otherwise `false`
   */
  [[nodiscard]] static bool BeginWriteSession();

  /**
   * @brief 提交当前编译格式会话的写入流并释放共享会话 / Commit the current
   * compiled-format session stream and release the shared session
   * @param stream 当前会话使用的写入流 / Write stream used by the current session
   * @param retained_size 当前会话真正保留下来的字节数 / Byte count actually retained by
   * the current session
   * @param format_result 本次格式化写出阶段的状态 / Status returned by the formatting
   * write phase
   * @return 当前 STDIO 会话最终提交的字节数；失败返回 `-1` / Byte count finally committed
   * by the current STDIO session; returns `-1` on failure
   */
  [[nodiscard]] static int FinishWriteSession(WritePort::Stream& stream,
                                              size_t retained_size,
                                              ErrorCode format_result);

 public:
  /**
   * @brief 将一个编译期 brace 字面量打印到当前 STDIO 输出 / Print one compile-time brace
   * literal to the active STDIO output
   * @tparam Source 编译期 brace 格式字面量 / Compile-time brace format literal
   * @tparam Args 调用点运行时参数类型列表 / Call-site runtime argument types
   * @param args 参与本次打印的运行时参数 / Runtime arguments used by the current print
   * @return 返回当前 STDIO 流实际保留并提交的字节数；若会话建立或提交失败则返回 `-1` /
   * Returns the byte count actually retained and committed to the current STDIO stream;
   * returns `-1` on session or commit failure
   */
  template <Print::Text Source, typename... Args>
  static int Print(Args&&... args)
  {
    constexpr LibXR::Format<Source> format{};
    return RunCompiled(format, std::forward<Args>(args)...);
  }

  /**
   * @brief 将一个编译期 printf 字面量打印到当前 STDIO 输出 / Print one compile-time
   * printf literal to the active STDIO output
   * @tparam Source 编译期 printf 格式字面量 / Compile-time printf format literal
   * @tparam Args 调用点运行时参数类型列表 / Call-site runtime argument types
   * @param args 参与本次打印的运行时参数 / Runtime arguments used by the current print
   * @return 返回当前 STDIO 流实际保留并提交的字节数；若会话建立或提交失败则返回 `-1` /
   * Returns the byte count actually retained and committed to the current STDIO stream;
   * returns `-1` on session or commit failure
   */
  template <Print::Text Source, typename... Args>
  static int Printf(Args&&... args)
  {
    constexpr auto format = Print::Printf::Build<Source>();
    return RunCompiled(format, std::forward<Args>(args)...);
  }
};

}  // namespace LibXR
