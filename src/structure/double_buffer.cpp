#include "double_buffer.hpp"

#include "libxr_mem.hpp"

using namespace LibXR;

// 构造过程委托给 `Init()`，这样直接构造和延迟初始化共用同一套校验路径。
// Construction is delegated to `Init()` so the same validation path is shared
// by direct construction and delayed initialization.
DoubleBuffer::DoubleBuffer(const LibXR::RawData& raw_data) { Init(raw_data); }

// `Reset()` 只清运行时状态，不会解绑两块 backing storage。
// Reset only clears runtime state. The two backing blocks remain bound.
void DoubleBuffer::Reset()
{
  active_ = 0;
  pending_valid_ = false;
  active_len_ = 0;
  pending_len_ = 0;
}

// `Init()` 一次性校验 backing storage 契约，然后把连续内存永久对半切分。
// `Init()` validates the backing-storage contract once, then permanently splits
// the continuous block into two equal halves.
void DoubleBuffer::Init(const LibXR::RawData& raw_data)
{
  constexpr size_t ALIGN = alignof(size_t);
  const bool EMPTY_STORAGE = (raw_data.addr_ == nullptr) && (raw_data.size_ == 0U);

  ASSERT(EMPTY_STORAGE || (raw_data.addr_ != nullptr));

  if (!EMPTY_STORAGE)
  {
    ASSERT(raw_data.size_ > 0U);
    ASSERT((reinterpret_cast<uintptr_t>(raw_data.addr_) % ALIGN) == 0U);
    ASSERT((raw_data.size_ % (2U * ALIGN)) == 0U);
  }

  size_ = raw_data.size_ / 2U;
  buffer_[0] = static_cast<uint8_t*>(raw_data.addr_);
  buffer_[1] = EMPTY_STORAGE ? nullptr : (static_cast<uint8_t*>(raw_data.addr_) + size_);
  Reset();
}

// `ActiveBuffer()` 只是对已绑定双半块的索引视图访问。
// Active-buffer lookup is just an indexed view over the bound backing halves.
uint8_t* DoubleBuffer::ActiveBuffer() const { return buffer_[active_]; }

// `PendingBuffer()` 总是返回当前 active block 的另一半。
// Pending-buffer lookup always returns the opposite half of the current active
// block.
uint8_t* DoubleBuffer::PendingBuffer() const { return buffer_[1 - active_]; }

// 这个接口只提供只读块编号访问，给需要稳定 block 语义的上层状态机使用。
// Direct block access is intentionally read-only metadata access for higher
// level state machines that need stable block numbering.
uint8_t* DoubleBuffer::Buffer(int block) const
{
  ASSERT((block == 0) || (block == 1));
  return buffer_[block];
}

// 初始化完成后，两半缓冲区的大小始终固定且相等。
// Each half always has the same fixed size after initialization.
size_t DoubleBuffer::Size() const { return size_; }

// 只有当生产侧标记 pending 有效后，`Switch()` 才会真正切换 active block。
// `Switch()` only changes the active block after pending data has been marked
// valid by the producer side.
void DoubleBuffer::Switch()
{
  if (pending_valid_)
  {
    active_ ^= 1;
    pending_valid_ = false;
  }
}

// Pending 是否就绪由一个显式有效位表示。
// Pending readiness is carried by one explicit validity bit.
bool DoubleBuffer::HasPending() const { return pending_valid_; }

// `FillPending()` 把 payload 拷进非活动半块，并把该半块标记为下次可切换。
// `FillPending()` copies payload bytes into the non-active half and marks that
// half ready for the next switch.
bool DoubleBuffer::FillPending(const uint8_t* data, size_t len)
{
  if (pending_valid_ || len > size_)
  {
    return false;
  }

  LibXR::Memory::FastCopy(PendingBuffer(), data, len);
  pending_len_ = len;
  pending_valid_ = true;
  return true;
}

// `FillActive()` 只原地更新当前活动半块，不会触碰 pending 状态簿记。
// `FillActive()` updates the currently active half in place without touching
// pending-state bookkeeping.
bool DoubleBuffer::FillActive(const uint8_t* data, size_t len)
{
  if (len > size_)
  {
    return false;
  }

  LibXR::Memory::FastCopy(ActiveBuffer(), data, len);
  return true;
}

// 某些调用方会手动写入字节，因此这里只需要补 state-bit 迁移。
// Some callers stage bytes manually and only need the state-bit transition.
void DoubleBuffer::EnablePending() { pending_valid_ = true; }

// 只有当生产侧宣布该半块有效后，pending length 才对外可见。
// Pending length is only observable after the producer has declared that half
// valid.
size_t DoubleBuffer::GetPendingLength() const
{
  return pending_valid_ ? pending_len_ : 0;
}
