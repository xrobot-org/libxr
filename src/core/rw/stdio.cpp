#include "stdio.hpp"

#include <limits>

using namespace LibXR;

// STDIO 编译格式桥接层 / STDIO compiled-format bridge.
STDIO::CompiledSink::CompiledSink(WritePort::Stream& stream) : stream_(stream) {}

ErrorCode STDIO::CompiledSink::Write(std::string_view chunk)
{
  if (saturated_)
  {
    return ErrorCode::OK;
  }

  auto ec = stream_.Acquire();
  if (ec != ErrorCode::OK)
  {
    return ec;
  }

  size_t copy_size = chunk.size();
  size_t stream_writable = stream_.EmptySize();
  if (copy_size > stream_writable)
  {
    copy_size = stream_writable;
  }

  if (copy_size == 0)
  {
    saturated_ = true;
    return ErrorCode::OK;
  }

  ec = stream_.Write(chunk.substr(0, copy_size));
  if (ec != ErrorCode::OK)
  {
    return ec;
  }

  retained_size_ += copy_size;
  if (copy_size != chunk.size() || stream_.EmptySize() == 0)
  {
    saturated_ = true;
  }
  return ErrorCode::OK;
}

bool STDIO::BeginWriteSession()
{
  if (!STDIO::write_ || !STDIO::write_->Writable())
  {
    return false;
  }

  if (!write_mutex_)
  {
    write_mutex_ = new LibXR::Mutex();
  }

  return write_mutex_->Lock() == ErrorCode::OK;
}

int STDIO::WriteCompiledToStream(WritePort::Stream& stream, void* context,
                                 CompiledWriteFun write_fun)
{
  CompiledSink sink(stream);
  auto ec = write_fun(context, sink);
  return FinishWriteSession(stream, sink.RetainedSize(), ec);
}

int STDIO::WriteCompiledSession(void* context, CompiledWriteFun write_fun)
{
  ASSERT(write_mutex_ != nullptr);
  ASSERT(write_fun != nullptr);

  if (write_stream_ != nullptr)
  {
    return WriteCompiledToStream(*write_stream_, context, write_fun);
  }

  static WriteOperation op;  // NOLINT
  WritePort::Stream stream(write_, op);
  return WriteCompiledToStream(stream, context, write_fun);
}

int STDIO::FinishWriteSession(WritePort::Stream& stream, size_t retained_size,
                              ErrorCode format_result)
{
  ASSERT(write_mutex_ != nullptr);

  // 基于 Stream 的 STDIO 现在明确采用“前缀保留”语义：格式化失败时，
  // 本会话已保留的字节仍然提交，不再尝试伪回滚。
  // Stream-backed STDIO is now explicitly prefix-preserving on formatting
  // failure: any bytes already retained in this session are committed instead
  // of being pseudo-rolled-back.
  auto ec = stream.Commit();

  write_mutex_->Unlock();

  if (format_result != ErrorCode::OK || ec != ErrorCode::OK ||
      retained_size > static_cast<size_t>(std::numeric_limits<int>::max()))
  {
    return -1;
  }

  return static_cast<int>(retained_size);
}
