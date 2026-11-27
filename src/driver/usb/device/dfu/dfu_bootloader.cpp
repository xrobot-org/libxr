#include "dfu_bootloader.hpp"

#include <cstring>

#include "crc.hpp"
#include "libxr_mem.hpp"

using namespace LibXR::USB;
using LibXR::ConstRawData;
using LibXR::ErrorCode;
using LibXR::RawData;
using LibXR::Timebase;

namespace
{
uint32_t NowMs() { return static_cast<uint32_t>(Timebase::GetMilliseconds()); }

size_t NonZero(size_t value) { return (value == 0U) ? 1U : value; }

size_t AlignUp(size_t value, size_t align)
{
  if (align <= 1U)
  {
    return value;
  }
  const size_t rem = value % align;
  return (rem == 0U) ? value : (value + align - rem);
}
}  // namespace

DfuBootloaderBackend::DfuBootloaderBackend(Flash& flash, size_t image_offset,
                                           size_t image_size_limit, size_t seal_offset,
                                           JumpFun jump_to_app, void* jump_app_ctx,
                                           bool autorun)
    : flash_(flash),
      image_offset_(image_offset),
      image_size_limit_(image_size_limit),
      seal_offset_(seal_offset),
      jump_to_app_(jump_to_app),
      jump_app_ctx_(jump_app_ctx),
      autorun_(autorun)
{
  erase_block_size_ = NonZero(flash_.MinEraseSize());
  erase_block_count_ = (image_size_limit_ + erase_block_size_ - 1U) / erase_block_size_;
  if (erase_block_count_ > 0U)
  {
    erased_blocks_ = new uint8_t[erase_block_count_];
    std::memset(erased_blocks_, 0, erase_block_count_);
  }
  seal_storage_size_ = AlignUp(sizeof(SealRecord), NonZero(flash_.MinWriteSize()));
  seal_storage_ = new uint8_t[seal_storage_size_];
  transfer_size_ = PayloadLimit();
  if (transfer_size_ == 0U)
  {
    transfer_size_ = 1024U;
  }
  if (transfer_size_ > 4096U)
  {
    transfer_size_ = 4096U;
  }
  write_buffer_ = new uint8_t[transfer_size_];
  // 只在初始化时读 Flash 校验，绝不在 SETUP 中断里。
  // The flash is read for the check at initialization only, never in a SETUP interrupt.
  size_t stored = 0U;
  if (ProbeStoredImage(&stored))
  {
    published_image_size_.store(stored, std::memory_order_release);
  }
}

LibXR::USB::DFUCapabilities DfuBootloaderBackend::GetDfuCapabilities() const
{
  DFUCapabilities caps{};
  caps.can_download = true;
  caps.can_upload = true;
  caps.manifestation_tolerant = false;
  caps.will_detach = false;
  caps.detach_timeout_ms = 0U;
  caps.transfer_size =
      static_cast<uint16_t>((transfer_size_ > 0xFFFFU) ? 0xFFFFU : transfer_size_);
  return caps;
}

ErrorCode DfuBootloaderBackend::DfuSetAlternate(uint8_t alt)
{
  return (alt == 0U) ? ErrorCode::OK : ErrorCode::NOT_SUPPORT;
}

void DfuBootloaderBackend::DfuAbort(uint8_t alt)
{
  UNUSED(alt);
  // 启动请求属于镜像而不属于传输会话：复位或解绑之后它仍然有效，只有新的下载会话清掉它。
  // A launch request belongs to the image, not to the transfer session: it survives a
  // reset or an unbind; only a new download session clears it.
  CancelWork();
  ResetTransferState();
}

void DfuBootloaderBackend::DfuClearStatus(uint8_t alt) { DfuAbort(alt); }

LibXR::USB::DFUStatusCode DfuBootloaderBackend::DfuDownload(uint8_t alt,
                                                            uint16_t block_num,
                                                            ConstRawData data,
                                                            uint32_t& poll_timeout_ms)
{
  poll_timeout_ms = 0U;
  if (alt != 0U)
  {
    return DFUStatusCode::ERR_TARGET;
  }
  if (data.addr_ == nullptr || data.size_ == 0U || data.size_ > transfer_size_)
  {
    return DFUStatusCode::ERR_USBR;
  }
  if (HasPendingWork())
  {
    return DFUStatusCode::ERR_NOTDONE;
  }
  if (block_num == 0U && download_.session_started &&
      (download_.received_bytes != 0U || download_.expected_block_num != 0U))
  {
    // 主机没有先 ABORT/CLRSTATUS 就从块 0 重来：当作新会话，丢掉旧的部分传输。
    // The host restarted from block 0 without ABORT/CLRSTATUS first: a fresh session,
    // the old partial transfer is dropped.
    ResetTransferState();
  }
  if (!download_.session_started)
  {
    if (block_num != 0U)
    {
      return DFUStatusCode::ERR_ADDRESS;
    }
    StartDownloadSession();
  }
  else if (block_num != download_.expected_block_num)
  {
    return DFUStatusCode::ERR_ADDRESS;
  }

  const size_t offset = download_.received_bytes;
  if (offset + data.size_ < offset || offset + data.size_ > PayloadLimit())
  {
    return DFUStatusCode::ERR_ADDRESS;
  }
  LibXR::Memory::FastCopy(write_buffer_, data.addr_, data.size_);
  write_.offset = offset;
  write_.len = data.size_;
  write_.block_num = block_num;
  write_.pending = true;
  download_.last_status = DFUStatusCode::OK;
  download_.next_poll_timeout_ms = ComputeWritePollTimeout(offset, data.size_);
  poll_timeout_ms = download_.next_poll_timeout_ms;
  job_.kind = WorkKind::WRITE;
  job_.write = write_;
  job_.image_size = 0U;
  job_state_.store(STAGED, std::memory_order_release);
  return DFUStatusCode::OK;
}

LibXR::USB::DFUStatusCode DfuBootloaderBackend::DfuGetDownloadStatus(
    uint8_t alt, bool& busy, uint32_t& poll_timeout_ms)
{
  if (alt != 0U)
  {
    busy = false;
    poll_timeout_ms = 0U;
    return DFUStatusCode::ERR_TARGET;
  }
  busy = write_.pending;
  poll_timeout_ms = busy ? RemainingWriteTime() : 0U;
  return download_.last_status;
}

size_t DfuBootloaderBackend::DfuUpload(uint8_t alt, uint16_t block_num, RawData data,
                                       DFUStatusCode& status, uint32_t& poll_timeout_ms)
{
  poll_timeout_ms = 0U;
  status = DFUStatusCode::OK;
  if (alt != 0U)
  {
    status = DFUStatusCode::ERR_TARGET;
    return 0U;
  }
  if (data.addr_ == nullptr || data.size_ == 0U)
  {
    status = DFUStatusCode::ERR_USBR;
    return 0U;
  }
  if (HasPendingWork())
  {
    status = DFUStatusCode::ERR_NOTDONE;
    return 0U;
  }
  if (block_num == 0U)
  {
    // 块 0 开始新会话并锁定当前镜像长度。
    // Block 0 starts a new session and latches the current image size.
    upload_.session_started = true;
    upload_.offset = 0U;
    upload_.expected_block_num = 1U;
    upload_.image_size = ImageSize();
    if (upload_.image_size == 0U)
    {
      status = DFUStatusCode::ERR_FIRMWARE;
      return 0U;
    }
  }
  else if (!upload_.session_started || block_num != upload_.expected_block_num)
  {
    status = DFUStatusCode::ERR_ADDRESS;
    return 0U;
  }
  if (upload_.offset >= upload_.image_size)
  {
    return 0U;
  }
  size_t read_size = upload_.image_size - upload_.offset;
  if (read_size > data.size_)
  {
    read_size = data.size_;
  }
  if (flash_.Read(image_offset_ + upload_.offset,
                  {static_cast<uint8_t*>(data.addr_), read_size}) != ErrorCode::OK)
  {
    status = DFUStatusCode::ERR_FIRMWARE;
    return 0U;
  }
  upload_.offset += read_size;
  upload_.expected_block_num = static_cast<uint16_t>(block_num + 1U);
  return read_size;
}

LibXR::USB::DFUStatusCode DfuBootloaderBackend::DfuManifest(uint8_t alt,
                                                            uint32_t& poll_timeout_ms)
{
  poll_timeout_ms = 0U;
  if (alt != 0U)
  {
    return DFUStatusCode::ERR_TARGET;
  }
  if (HasPendingWork() || !download_.session_started || download_.received_bytes == 0U)
  {
    return DFUStatusCode::ERR_NOTDONE;
  }
  manifest_.pending = true;
  manifest_.last_status = DFUStatusCode::OK;
  poll_timeout_ms = manifest_.poll_timeout_ms;
  job_.kind = WorkKind::MANIFEST;
  job_.image_size = download_.received_bytes;
  job_state_.store(STAGED, std::memory_order_release);
  return DFUStatusCode::OK;
}

LibXR::USB::DFUStatusCode DfuBootloaderBackend::DfuGetManifestStatus(
    uint8_t alt, bool& busy, uint32_t& poll_timeout_ms)
{
  if (alt != 0U)
  {
    busy = false;
    poll_timeout_ms = 0U;
    return DFUStatusCode::ERR_TARGET;
  }
  busy = manifest_.pending;
  poll_timeout_ms = busy ? manifest_.poll_timeout_ms : 0U;
  return manifest_.last_status;
}

void DfuBootloaderBackend::AllowPendingWork()
{
  uint32_t expected = STAGED;
  (void)job_state_.compare_exchange_strong(expected, READY, std::memory_order_release,
                                           std::memory_order_relaxed);
}

bool DfuBootloaderBackend::RunPendingWork()
{
  if (job_state_.load(std::memory_order_acquire) != READY)
  {
    return false;
  }
  // 开始时刻在发布 RUNNING 之前写：GETSTATUS 只在看到 RUNNING/DONE 之后读它；下面的
  // CAS 失败（工作刚被作废）时没有人读这次写入。
  // The start time is written before RUNNING is published: GETSTATUS reads it only after
  // seeing RUNNING/DONE; when the CAS below fails (the work was just voided) nobody reads
  // this write.
  job_started_ms_ = NowMs();
  uint32_t expected = READY;
  if (!job_state_.compare_exchange_strong(expected, RUNNING, std::memory_order_acq_rel))
  {
    return false;
  }
  DFUStatusCode result = DFUStatusCode::OK;
  if (job_.kind == WorkKind::WRITE)
  {
    if (!EnsureBlocksErased(job_.write.offset, job_.write.len))
    {
      result = DFUStatusCode::ERR_ERASE;
    }
    else if (!BeginFlashStep())
    {
      result = DFUStatusCode::ERR_NOTDONE;
    }
    else if (flash_.Write(image_offset_ + job_.write.offset,
                          {write_buffer_, job_.write.len}) != ErrorCode::OK)
    {
      result = DFUStatusCode::ERR_PROG;
    }
    // 这类步骤的实测耗时作为下一次报告的 bwPollTimeout。
    // The measured duration of this kind of step is the next reported bwPollTimeout.
    uint32_t spent = NowMs() - job_started_ms_;
    spent = (spent < MIN_WRITE_POLL_MS)   ? MIN_WRITE_POLL_MS
            : (spent > MAX_WRITE_POLL_MS) ? MAX_WRITE_POLL_MS
                                          : spent;
    (write_needs_erase_ ? measured_erase_write_ms_ : measured_write_ms_) = spent;
  }
  else
  {
    uint32_t crc = 0U;
    if (!ComputeImageCrc32(job_.image_size, crc) || !WriteSeal(job_.image_size, crc))
    {
      result = DFUStatusCode::ERR_VERIFY;
    }
  }
  job_result_ = result;
  uint32_t observed = job_state_.load(std::memory_order_acquire);
  while (!job_state_.compare_exchange_weak(observed, (observed & CANCELLED) | DONE,
                                           std::memory_order_release,
                                           std::memory_order_acquire))
  {
  }
  return true;
}

void DfuBootloaderBackend::CommitWorkResult()
{
  const uint32_t state = job_state_.load(std::memory_order_acquire);
  if ((state & ~CANCELLED) != DONE)
  {
    return;
  }
  if ((state & CANCELLED) == 0U)
  {
    if (job_.kind == WorkKind::WRITE)
    {
      download_.last_status = job_result_;
      if (job_result_ == DFUStatusCode::OK)
      {
        download_.received_bytes = job_.write.offset + job_.write.len;
        download_.expected_block_num = static_cast<uint16_t>(job_.write.block_num + 1U);
      }
      write_.pending = false;
    }
    else
    {
      manifest_.last_status = job_result_;
      manifest_.pending = false;
      if (job_result_ == DFUStatusCode::OK)
      {
        published_image_size_.store(job_.image_size, std::memory_order_release);
        download_ = {};
        upload_ = {};
      }
    }
  }
  // 作废的旧工作只释放，不改替代它的新会话。
  // A voided old job is only freed; it never updates the session that replaced it.
  job_state_.store(EMPTY, std::memory_order_release);
}

void DfuBootloaderBackend::DfuCommitManifestWaitReset(uint8_t alt)
{
  if (alt == 0U && autorun_ && HasValidImage())
  {
    launch_request_.store(1U, std::memory_order_release);
  }
}

bool DfuBootloaderBackend::TryRequestRunApp()
{
  if (HasPendingWork() || ImageSize() == 0U)
  {
    return false;
  }
  launch_request_.store(1U, std::memory_order_release);
  return true;
}

bool DfuBootloaderBackend::TryConsumeAppLaunch(uint32_t now_ms)
{
  UNUSED(now_ms);
  if (HasPendingWork() || !HasValidImage() ||
      launch_request_.exchange(0U, std::memory_order_acq_rel) == 0U)
  {
    return false;
  }
  if (jump_to_app_ != nullptr)
  {
    jump_to_app_(jump_app_ctx_);
  }
  return true;
}

bool DfuBootloaderBackend::HasPendingWork() const
{
  return job_state_.load(std::memory_order_acquire) != EMPTY;
}

size_t DfuBootloaderBackend::ImageSize() const
{
  return published_image_size_.load(std::memory_order_acquire);
}

size_t DfuBootloaderBackend::PayloadLimit() const
{
  // 下载、上传只碰 seal 之前的区域。
  // Download and upload touch only the area before the seal.
  return (seal_offset_ > image_size_limit_) ? 0U : seal_offset_;
}

void DfuBootloaderBackend::StartDownloadSession()
{
  ResetTransferState();
  if (erase_block_count_ > 0U)
  {
    std::memset(erased_blocks_, 0, erase_block_count_);
  }
  download_.session_started = true;
  // 新会话开始改写 Flash：旧镜像和它的启动请求都作废。
  // A new session starts rewriting the flash: the old image and its launch request are
  // void.
  published_image_size_.store(0U, std::memory_order_release);
  launch_request_.store(0U, std::memory_order_release);
}

void DfuBootloaderBackend::ResetTransferState()
{
  download_ = {};
  write_ = {};
  manifest_ = {};
  upload_ = {};
}

uint32_t DfuBootloaderBackend::ComputeWritePollTimeout(size_t offset, size_t len)
{
  // 下一次报告：同类写入（要擦 / 只编程）上次实测的耗时，没有实测时 25 / 10 ms。
  // Next report: the measured duration of the last write of the same kind (erase /
  // program only), or 25 / 10 ms before one was measured.
  const size_t first_block = offset / erase_block_size_;
  const size_t last_block = (offset + len - 1U) / erase_block_size_;
  write_needs_erase_ = false;
  for (size_t block = first_block; block <= last_block && block < erase_block_count_;
       ++block)
  {
    if (erased_blocks_[block] == 0U)
    {
      write_needs_erase_ = true;
      break;
    }
  }
  return write_needs_erase_ ? measured_erase_write_ms_ : measured_write_ms_;
}

uint32_t DfuBootloaderBackend::RemainingWriteTime() const
{
  // 步骤开始前报完整估计，开始后报估计减已用，至少 MIN_REMAINING_POLL_MS。
  // Before the step starts the whole estimate, afterwards the estimate minus the time
  // spent, at least MIN_REMAINING_POLL_MS.
  const uint32_t estimate = download_.next_poll_timeout_ms;
  const uint32_t state = job_state_.load(std::memory_order_acquire) & ~CANCELLED;
  if (state != RUNNING && state != DONE)
  {
    return estimate;
  }
  const uint32_t spent = NowMs() - job_started_ms_;
  return (spent + MIN_REMAINING_POLL_MS < estimate) ? estimate - spent
                                                    : MIN_REMAINING_POLL_MS;
}

bool DfuBootloaderBackend::ComputeImageCrc32(size_t image_size, uint32_t& crc32_out)
{
  // seal 用的 CRC32 只覆盖镜像区，不做最后的取反。
  // The seal's CRC32 covers the image area only, without the final inversion.
  uint32_t crc = 0xFFFFFFFFU;
  size_t offset = 0U;
  while (offset < image_size)
  {
    size_t chunk = image_size - offset;
    if (chunk > sizeof(crc_buffer_))
    {
      chunk = sizeof(crc_buffer_);
    }
    if (!BeginFlashStep() ||
        flash_.Read(image_offset_ + offset, {crc_buffer_, chunk}) != ErrorCode::OK)
    {
      return false;
    }
    for (size_t i = 0; i < chunk; ++i)
    {
      crc = LibXR::CRC32::tab_[(crc ^ crc_buffer_[i]) & 0xFFU] ^ (crc >> 8U);
    }
    offset += chunk;
  }
  crc32_out = crc;
  return true;
}

bool DfuBootloaderBackend::ReadSeal(SealRecord& seal)
{
  return flash_.Read(image_offset_ + seal_offset_,
                     {reinterpret_cast<uint8_t*>(&seal), sizeof(seal)}) == ErrorCode::OK;
}

bool DfuBootloaderBackend::WriteSeal(size_t image_size, uint32_t crc32)
{
  if (seal_offset_ > image_size_limit_ ||
      seal_storage_size_ > image_size_limit_ - seal_offset_)
  {
    return false;
  }
  // seal 可能和镜像共用最后一个擦除块；沿用本会话的擦除表，不会擦掉已写的镜像。
  // The seal may share the last erase block with the image; the session's erase map is
  // reused, so the image already programmed is not erased.
  std::memset(seal_storage_, 0xFF, seal_storage_size_);
  SealRecord seal{};
  seal.image_size = static_cast<uint32_t>(image_size);
  seal.crc32 = crc32;
  seal.crc32_inv = ~crc32;
  std::memcpy(seal_storage_, &seal, sizeof(seal));
  if (!EnsureBlocksErased(seal_offset_, seal_storage_size_) || !BeginFlashStep())
  {
    return false;
  }
  return flash_.Write(image_offset_ + seal_offset_,
                      {seal_storage_, seal_storage_size_}) == ErrorCode::OK;
}

bool DfuBootloaderBackend::ProbeStoredImage(size_t* out_image_size)
{
  SealRecord seal{};
  if (!ReadSeal(seal) || seal.magic != SEAL_MAGIC ||
      (seal.crc32 ^ seal.crc32_inv) != 0xFFFFFFFFU)
  {
    return false;
  }
  const auto image_size = static_cast<size_t>(seal.image_size);
  if (image_size == 0U || image_size > PayloadLimit())
  {
    return false;
  }
  uint32_t actual_crc = 0U;
  if (!ComputeImageCrc32(image_size, actual_crc) || actual_crc != seal.crc32)
  {
    return false;
  }
  *out_image_size = image_size;
  return true;
}

bool DfuBootloaderBackend::EnsureBlocksErased(size_t offset, size_t len)
{
  // 一次下载会话里每块最多擦一次。
  // Each block is erased at most once per download session.
  if (len == 0U)
  {
    return true;
  }
  const size_t first_block = offset / erase_block_size_;
  const size_t last_block = (offset + len - 1U) / erase_block_size_;
  if (last_block >= erase_block_count_)
  {
    return false;
  }
  for (size_t block = first_block; block <= last_block; ++block)
  {
    if (erased_blocks_[block] != 0U)
    {
      continue;
    }
    if (!BeginFlashStep() || flash_.Erase(image_offset_ + block * erase_block_size_,
                                          erase_block_size_) != ErrorCode::OK)
    {
      return false;
    }
    erased_blocks_[block] = 1U;
  }
  return true;
}

void DfuBootloaderBackend::CancelWork()
{
  // 还没开始的（STAGED、READY）直接清空；在跑或已完成的置 CANCELLED，留给
  // CommitWorkResult 释放。
  // Work not started yet (STAGED, READY) is emptied at once; running or finished work
  // gets CANCELLED and is freed by CommitWorkResult.
  uint32_t state = job_state_.load(std::memory_order_acquire);
  while (state != EMPTY)
  {
    const uint32_t phase = state & ~CANCELLED;
    const uint32_t next =
        (phase == RUNNING || phase == DONE) ? (state | CANCELLED) : EMPTY;
    if (job_state_.compare_exchange_weak(state, next, std::memory_order_acq_rel))
    {
      break;
    }
  }
}

bool DfuBootloaderBackend::BeginFlashStep()
{
  // 构造时校验已存镜像：工作槽为空，允许。之后每次 Flash 调用前确认工作没被作废；
  // 作废发生在确认之后时，这一次调用照常完成，下一次不再进行。
  // The stored-image check at construction runs with the slot empty and is allowed.
  // Afterwards every flash call first checks the work was not voided; a void that comes
  // after the check lets this one call finish and stops the next.
  const uint32_t state = job_state_.load(std::memory_order_acquire);
  return state == EMPTY || state == RUNNING;
}
