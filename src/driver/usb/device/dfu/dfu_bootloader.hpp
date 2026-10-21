#pragma once

#include <atomic>
#include <cstring>

#include "dfu/dfu_def.hpp"
#include "timebase.hpp"

namespace LibXR::USB
{
/**
 * @brief Bootloader DFU 的通用后端：围绕 Flash 基类实现 download/upload/manifest
 *        Generic bootloader DFU backend built around the Flash base interface.
 */
class DfuBootloaderBackend
{
 public:
  using JumpCallback = void (*)(void*);
  static constexpr uint32_t SEAL_MAGIC = 0x4C414553u;  // "SEAL"

  LIBXR_PACKED_BEGIN
  /**
   * @brief seal 区固定记录 / Fixed record stored in the seal region
   */
  struct SealRecord
  {
    uint32_t magic = SEAL_MAGIC;
    uint32_t image_size = 0u;
    uint32_t crc32 = 0u;
    uint32_t crc32_inv = 0u;
  };
  LIBXR_PACKED_END

  // Backend 负责：
  // - 管理 download/upload/manifest 状态
  // - 通过 Flash 基类接口驱动擦写读
  // - 维护镜像有效性与 seal 元数据
  // Backend responsibilities:
  // - own download/upload/manifest bookkeeping
  // - drive flash erase/write/read through the Flash base interface
  // - maintain image validity and seal metadata
  DfuBootloaderBackend(Flash& flash, size_t image_offset, size_t image_size_limit,
                       size_t seal_offset, JumpCallback jump_to_app,
                       void* jump_app_ctx = nullptr, bool autorun = true)
      : flash_(flash),
        image_offset_(image_offset),
        image_size_limit_(image_size_limit),
        seal_offset_(seal_offset),
        jump_to_app_(jump_to_app),
        jump_app_ctx_(jump_app_ctx),
        autorun_(autorun)
  {
    erase_block_size_ = flash_.MinEraseSize();
    if (erase_block_size_ == 0u)
    {
      erase_block_size_ = 1u;
    }
    erase_block_count_ = (image_size_limit_ + erase_block_size_ - 1u) / erase_block_size_;
    if (erase_block_count_ > 0u)
    {
      erased_blocks_ = new uint8_t[erase_block_count_];
      std::memset(erased_blocks_, 0, erase_block_count_);
    }
    seal_storage_size_ =
        AlignUp(sizeof(SealRecord), NonZeroWriteSize(flash_.MinWriteSize()));
    seal_storage_ = new uint8_t[seal_storage_size_];
    transfer_size_ = PayloadLimit();
    if (transfer_size_ == 0u)
    {
      transfer_size_ = 1024u;
    }
    if (transfer_size_ > 4096u)
    {
      transfer_size_ = 4096u;
    }
    write_buffer_ = new uint8_t[transfer_size_];
    ResetTransferState();
    image_.launch_requested = false;
    image_.ready = false;
    image_.stored_size = 0u;
    size_t stored = 0;
    image_.ready = ProbeStoredImage(&stored);  // Initialization, never a SETUP IRQ.
    image_.stored_size = image_.ready ? stored : 0;
    published_image_size_.store(image_.stored_size, std::memory_order_release);
  }

  DfuBootloaderBackend(const DfuBootloaderBackend&) = delete;
  DfuBootloaderBackend& operator=(const DfuBootloaderBackend&) = delete;

  /**
   * @brief 报告 DFU 能力集 / Report DFU capabilities
   */
  DFUCapabilities GetDfuCapabilities() const
  {
    DFUCapabilities caps = {};
    caps.can_download = true;
    caps.can_upload = true;
    caps.manifestation_tolerant = false;
    caps.will_detach = false;
    caps.detach_timeout_ms = 0u;

    if (transfer_size_ > 0xFFFFu)
    {
      caps.transfer_size = 0xFFFFu;
    }
    else
    {
      caps.transfer_size = static_cast<uint16_t>(transfer_size_);
    }
    return caps;
  }

  ErrorCode DfuSetAlternate(uint8_t alt)
  {
    return (alt == 0u) ? ErrorCode::OK : ErrorCode::NOT_SUPPORT;
  }

  /**
   * @brief 放弃当前协议会话 / Abort the current protocol session
   */
  void DfuAbort(uint8_t)
  {
    CancelWork();
    ResetTransferState();
    launch_request_.store(0U, std::memory_order_release);
  }

  /**
   * @brief 清除 DFU 错误态 / Clear the DFU error state
   */
  void DfuClearStatus(uint8_t alt) { DfuAbort(alt); }

  DFUStatusCode DfuDownload(uint8_t alt, uint16_t block_num, ConstRawData data,
                            uint32_t& poll_timeout_ms)
  {
    poll_timeout_ms = 0u;

    if (alt != 0u)
    {
      return DFUStatusCode::ERR_TARGET;
    }
    if (data.addr_ == nullptr || data.size_ == 0u || data.size_ > transfer_size_)
    {
      return DFUStatusCode::ERR_USBR;
    }
    if (HasPendingWork())
    {
      return DFUStatusCode::ERR_NOTDONE;
    }

    if ((block_num == 0u) &&
        (download_.session_started &&
         (download_.received_bytes != 0u || download_.expected_block_num != 0u)))
    {
      // 主机没有先发 ABORT/CLRSTATUS，就直接从 block 0 重启 DNLOAD；
      // 这里把它视为新会话，并丢弃旧的部分传输状态。
      // The host restarted DNLOAD from block 0 without an explicit
      // ABORT/CLRSTATUS first; treat that as a fresh session and discard the
      // old partial transfer state.
      ResetTransferState();
    }

    if (!download_.session_started)
    {
      if (block_num != 0u)
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
    const size_t payload_limit = PayloadLimit();
    if (offset + data.size_ < offset || (offset + data.size_) > payload_limit)
    {
      return DFUStatusCode::ERR_ADDRESS;
    }

    std::memcpy(write_buffer_, data.addr_, data.size_);
    write_.offset = offset;
    write_.len = data.size_;
    write_.block_num = block_num;
    write_.pending = true;
    download_.last_status = DFUStatusCode::OK;
    download_.next_poll_timeout_ms = ComputeWritePollTimeout(offset, data.size_);
    poll_timeout_ms = download_.next_poll_timeout_ms;
    job_.kind = WorkKind::WRITE;
    job_.write = write_;
    job_.image_size = 0;
    job_state_.store(STAGED, std::memory_order_release);
    return DFUStatusCode::OK;
  }

  DFUStatusCode DfuGetDownloadStatus(uint8_t alt, bool& busy, uint32_t& poll_timeout_ms)
  {
    if (alt != 0u)
    {
      busy = false;
      poll_timeout_ms = 0u;
      return DFUStatusCode::ERR_TARGET;
    }
    busy = HasPendingWrite();
    poll_timeout_ms = busy ? RemainingWriteTime() : 0u;
    return download_.last_status;
  }

  size_t DfuUpload(uint8_t alt, uint16_t block_num, RawData data, DFUStatusCode& status,
                   uint32_t& poll_timeout_ms)
  {
    poll_timeout_ms = 0u;
    status = DFUStatusCode::OK;

    if (alt != 0u)
    {
      status = DFUStatusCode::ERR_TARGET;
      return 0u;
    }
    if (data.addr_ == nullptr || data.size_ == 0u)
    {
      status = DFUStatusCode::ERR_USBR;
      return 0u;
    }

    if (HasPendingWork())
    {
      status = DFUStatusCode::ERR_NOTDONE;
      return 0;
    }
    if (block_num == 0u)
    {
      upload_.session_started = true;
      upload_.offset = 0u;
      upload_.expected_block_num = 1u;
      size_t image_size = 0u;
      if (image_.ready)
      {
        image_size = image_.stored_size;
      }
      else
        image_size = 0u;
      upload_.image_size = image_size;
      if (upload_.image_size == 0u)
      {
        status = DFUStatusCode::ERR_FIRMWARE;
        return 0u;
      }
    }
    else if (!upload_.session_started || block_num != upload_.expected_block_num)
    {
      status = DFUStatusCode::ERR_ADDRESS;
      return 0u;
    }

    if (upload_.offset >= upload_.image_size)
    {
      return 0u;
    }

    size_t read_size = upload_.image_size - upload_.offset;
    if (read_size > data.size_)
    {
      read_size = data.size_;
    }
    if (flash_.Read(image_offset_ + upload_.offset,
                    {reinterpret_cast<uint8_t*>(data.addr_), read_size}) != ErrorCode::OK)
    {
      status = DFUStatusCode::ERR_FIRMWARE;
      return 0u;
    }

    upload_.offset += read_size;
    upload_.expected_block_num = static_cast<uint16_t>(block_num + 1u);
    return read_size;
  }

  /**
   * @brief 启动 manifest 阶段 / Start the manifest stage
   */
  DFUStatusCode DfuManifest(uint8_t alt, uint32_t& poll_timeout_ms)
  {
    poll_timeout_ms = 0u;

    if (alt != 0u)
    {
      return DFUStatusCode::ERR_TARGET;
    }
    if (HasPendingWork() || !download_.session_started || download_.received_bytes == 0u)
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

  /**
   * @brief 查询 manifest 异步状态 / Query manifest async status
   */
  DFUStatusCode DfuGetManifestStatus(uint8_t alt, bool& busy, uint32_t& poll_timeout_ms)
  {
    if (alt != 0u)
    {
      busy = false;
      poll_timeout_ms = 0u;
      return DFUStatusCode::ERR_TARGET;
    }
    busy = HasPendingManifest();
    poll_timeout_ms = busy ? manifest_.poll_timeout_ms : 0u;
    return manifest_.last_status;
  }

  // 仅 owner 上下文：提交已完成工作的结果。擦写与校验由 RunPendingWork() 在
  // DFUClass::Process() 的调用者上下文里执行。
  // Owner context only: commit the result of finished work. Erase, program and verify
  // run in RunPendingWork(), in the context that calls DFUClass::Process().
  void Process() { CommitWorkResult(); }

  void AllowPendingWork()
  {
    uint32_t expected = STAGED;
    (void)job_state_.compare_exchange_strong(expected, READY, std::memory_order_release,
                                             std::memory_order_relaxed);
  }

  bool RunPendingWork()
  {
    uint32_t expected = READY;
    if (!job_state_.compare_exchange_strong(expected, RUNNING, std::memory_order_acq_rel))
      return false;
    DFUStatusCode result = DFUStatusCode::OK;
    job_started_ms_ = static_cast<uint32_t>(Timebase::GetMilliseconds());
    if (job_.kind == WorkKind::WRITE)
    {
      if (!EnsureBlocksErased(job_.write.offset, job_.write.len))
        result = DFUStatusCode::ERR_ERASE;
      else if (!BeginFlashStep())
        result = DFUStatusCode::ERR_NOTDONE;
      else if (flash_.Write(image_offset_ + job_.write.offset,
                            {write_buffer_, job_.write.len}) != ErrorCode::OK)
        result = DFUStatusCode::ERR_PROG;
      // 记下这类步骤的实测耗时，作为下一次报告的 poll timeout。
      // Keep the measured duration of this kind of step for the next poll timeout.
      uint32_t spent =
          static_cast<uint32_t>(Timebase::GetMilliseconds()) - job_started_ms_;
      spent = spent < MIN_WRITE_POLL_MS   ? MIN_WRITE_POLL_MS
              : spent > MAX_WRITE_POLL_MS ? MAX_WRITE_POLL_MS
                                          : spent;
      (write_needs_erase_ ? measured_erase_write_ms_ : measured_write_ms_) = spent;
    }
    else
    {
      uint32_t crc = 0;
      if (!ComputeImageCrc32(job_.image_size, crc))
        result = DFUStatusCode::ERR_VERIFY;
      else if (!WriteSeal(job_.image_size, crc))
        result = DFUStatusCode::ERR_VERIFY;
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

  void CommitWorkResult()
  {
    const uint32_t state = job_state_.load(std::memory_order_acquire);
    if ((state & ~CANCELLED) != DONE) return;
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
          image_.stored_size = job_.image_size;
          image_.ready = true;
          published_image_size_.store(job_.image_size, std::memory_order_release);
          download_.session_started = false;
          download_.received_bytes = 0;
          download_.expected_block_num = 0;
          upload_ = {};
        }
      }
    }
    // Only the protocol owner releases the immutable job/data slot after reading
    // its result. A cancelled old job never updates a replacement USB session.
    job_state_.store(EMPTY, std::memory_order_release);
  }

  bool TryRequestRunApp()
  {
    if (HasPendingWork() || published_image_size_.load(std::memory_order_acquire) == 0U)
      return false;
    launch_request_.store(1U, std::memory_order_release);
    return true;
  }

  void DfuCommitManifestWaitReset(uint8_t alt)
  {
    if (alt == 0U && autorun_ && HasValidImage())
      launch_request_.store(1U, std::memory_order_release);
  }

  bool TryConsumeAppLaunch(uint32_t)
  {
    if (HasPendingWork() || !HasValidImage() ||
        launch_request_.exchange(0U, std::memory_order_acq_rel) == 0U)
      return false;
    if (jump_to_app_) jump_to_app_(jump_app_ctx_);
    return true;
  }

  bool HasPendingWork() const
  {
    return job_state_.load(std::memory_order_acquire) != EMPTY;
  }

  bool HasValidImage() const { return ImageSize() != 0U; }
  size_t ImageSize() const
  {
    return published_image_size_.load(std::memory_order_acquire);
  }

 private:
  // download/upload 只允许访问 seal 记录之前的 payload 区域。
  // Download/upload only operate on the payload area before the seal record.
  size_t PayloadLimit() const
  {
    if (seal_offset_ > image_size_limit_)
    {
      return 0u;
    }
    return seal_offset_;
  }

  static size_t NonZeroWriteSize(size_t write_size)
  {
    return (write_size == 0u) ? 1u : write_size;
  }

  static size_t AlignUp(size_t value, size_t align)
  {
    if (align <= 1u)
    {
      return value;
    }
    const size_t rem = value % align;
    return (rem == 0u) ? value : (value + align - rem);
  }

  // 开启新的下载会话，并使之前缓存的镜像有效性视图失效。
  // Start a fresh download session and invalidate any previously cached image view.
  void StartDownloadSession()
  {
    ResetTransferState();
    if (erase_block_count_ > 0u)
    {
      std::memset(erased_blocks_, 0, erase_block_count_);
    }
    download_.session_started = true;
    image_.ready = false;
    image_.stored_size = 0u;
    published_image_size_.store(0U, std::memory_order_release);
    launch_request_.store(0U, std::memory_order_release);
  }

  void ResetTransferState()
  {
    // 这里只重置协议拥有的传输会话状态；
    // 镜像有效性和启动策略单独放在 ImageState 里。
    // Reset only protocol-owned transfer session state;
    // image validity and launch policy are tracked separately in ImageState.
    download_ = {};
    write_ = {};
    manifest_ = {};
    upload_ = {};
  }

  bool HasPendingWrite() const { return write_.pending; }

  bool HasPendingManifest() const { return manifest_.pending; }

  // 下一次 GETSTATUS 报告的 poll timeout：同类写入步骤（要擦除 / 只编程）上一次实测
  // 的耗时，还没有实测时用 25 / 10 ms。主机按它等待，一般一次 GETSTATUS 就能看到
  // 写完。
  // The poll timeout reported by the next GETSTATUS: the measured duration of the last
  // write step of the same kind (with erase / program only), or 25 / 10 ms before one
  // was measured. A host that waits for it usually sees the step done at the first
  // GETSTATUS.
  uint32_t ComputeWritePollTimeout(size_t offset, size_t len)
  {
    const size_t first_block = offset / erase_block_size_;
    const size_t last_block = (offset + len - 1u) / erase_block_size_;
    write_needs_erase_ = false;
    for (size_t block = first_block; block <= last_block && block < erase_block_count_;
         ++block)
    {
      if (erased_blocks_[block] == 0u)
      {
        write_needs_erase_ = true;
        break;
      }
    }
    return write_needs_erase_ ? measured_erase_write_ms_ : measured_write_ms_;
  }

  // 写入步骤还剩的估计时间：步骤开始前为完整估计，开始后为估计减去已用时间，至少
  // MIN_REMAINING_POLL_MS。 / Estimated time left for the write step: the whole estimate
  // before the step starts, the estimate minus the time spent once it runs, at least
  // MIN_REMAINING_POLL_MS.
  uint32_t RemainingWriteTime() const
  {
    const uint32_t estimate = download_.next_poll_timeout_ms;
    const uint32_t state = job_state_.load(std::memory_order_acquire) & ~CANCELLED;
    if (state != RUNNING && state != DONE)
    {
      return estimate;
    }
    const uint32_t spent =
        static_cast<uint32_t>(Timebase::GetMilliseconds()) - job_started_ms_;
    return spent + MIN_REMAINING_POLL_MS < estimate ? estimate - spent
                                                    : MIN_REMAINING_POLL_MS;
  }

  // 执行一次延迟写入步骤：
  // 1) 对新覆盖到的擦除块各擦一次
  // 2) 再把刚收到的 payload 分片编程进去
  // Execute one deferred write step:
  // 1) erase each newly touched erase block once
  // 2) program the just-received payload chunk

  // 通过计算固定 CRC32 并写入 seal 记录来完成镜像定稿。
  // Finalize the image by computing the fixed CRC32 and writing the seal record.

  // 计算 seal 记录使用的固定 CRC32，只覆盖 payload 区域。
  // Compute the fixed CRC32 used by the seal record over the payload area only.
  bool ComputeImageCrc32(size_t image_size, uint32_t& crc32_out)
  {
    uint32_t crc = 0xFFFFFFFFu;
    size_t offset = 0u;
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
      const uint8_t* buf = crc_buffer_;
      size_t remain = chunk;
      while (remain-- > 0u)
      {
        crc = LibXR::CRC32::tab_[(crc ^ *buf++) & 0xFFu] ^ (crc >> 8);
      }
      offset += chunk;
    }
    crc32_out = crc;
    return true;
  }

  // 从 flash 中读取当前 seal 记录。
  // Read the current seal record from flash.
  bool ReadSeal(SealRecord& seal)
  {
    return flash_.Read(image_offset_ + seal_offset_, {reinterpret_cast<uint8_t*>(&seal),
                                                      sizeof(seal)}) == ErrorCode::OK;
  }

  // Seal may share the last erase block with payload. Reuse the per-session
  // erase map so manifest does not wipe a block that was already erased and
  // partially programmed by DNLOAD payload chunks.
  bool WriteSeal(size_t image_size, uint32_t crc32)
  {
    std::memset(seal_storage_, 0xFF, seal_storage_size_);
    auto* seal = reinterpret_cast<SealRecord*>(seal_storage_);
    seal->magic = SEAL_MAGIC;
    seal->image_size = static_cast<uint32_t>(image_size);
    seal->crc32 = crc32;
    seal->crc32_inv = ~crc32;

    if (!EnsureBlocksErased(seal_offset_, seal_storage_size_))
    {
      return false;
    }
    if (!BeginFlashStep())
    {
      return false;
    }
    return flash_.Write(image_offset_ + seal_offset_,
                        {seal_storage_, seal_storage_size_}) == ErrorCode::OK;
  }

  // 探测 flash 当前是否已有有效的已封印镜像，
  // 并可选返回它记录的 payload 大小。
  // Probe whether flash already contains a valid sealed image, and optionally
  // return its stored payload size.
  bool ProbeStoredImage(size_t* out_image_size)
  {
    SealRecord seal = {};
    if (!ReadSeal(seal))
    {
      return false;
    }
    if (seal.magic != SEAL_MAGIC)
    {
      return false;
    }
    if ((seal.crc32 ^ seal.crc32_inv) != 0xFFFFFFFFu)
    {
      return false;
    }
    const size_t image_size = static_cast<size_t>(seal.image_size);
    if (image_size == 0u || image_size > PayloadLimit())
    {
      return false;
    }
    uint32_t actual_crc = 0u;
    if (!ComputeImageCrc32(image_size, actual_crc))
    {
      return false;
    }
    if (actual_crc != seal.crc32)
    {
      return false;
    }
    if (out_image_size != nullptr)
    {
      *out_image_size = image_size;
    }
    return true;
  }

  // 对 [offset, offset+len) 覆盖到的每个块，
  // 在一次下载会话里最多只擦除一次。
  // Erase every block touched by [offset, offset+len) at most once per
  // download session.
  bool EnsureBlocksErased(size_t offset, size_t len)
  {
    if (len == 0u)
    {
      return true;
    }

    const size_t first_block = offset / erase_block_size_;
    const size_t last_block = (offset + len - 1u) / erase_block_size_;
    if (last_block >= erase_block_count_)
    {
      return false;
    }

    for (size_t block = first_block; block <= last_block; ++block)
    {
      if (erased_blocks_[block] != 0u)
      {
        continue;
      }
      const size_t block_offset = block * erase_block_size_;
      if (!BeginFlashStep() ||
          flash_.Erase(image_offset_ + block_offset, erase_block_size_) != ErrorCode::OK)
      {
        return false;
      }
      erased_blocks_[block] = 1u;
    }
    return true;
  }

  /**
   * @brief 镜像级状态：独立于一次具体的 DFU 传输会话
   *        Image-level state kept outside any single DFU transfer session.
   */
  struct ImageState
  {
    bool launch_requested = false;
    bool ready = false;
    size_t stored_size = 0u;
  };

  /**
   * @brief Download 会话状态 / Download session state
   */
  struct DownloadState
  {
    bool session_started = false;
    size_t received_bytes = 0u;
    uint16_t expected_block_num = 0u;
    uint32_t next_poll_timeout_ms = 0u;
    DFUStatusCode last_status = DFUStatusCode::OK;
  };

  /**
   * @brief 延迟写入步骤状态 / Deferred write-step state
   */
  struct WriteState
  {
    size_t offset = 0u;
    size_t len = 0u;
    uint16_t block_num = 0u;
    bool pending = false;
  };

  /**
   * @brief Manifest 会话状态 / Manifest session state
   */
  struct ManifestState
  {
    bool pending = false;
    uint32_t poll_timeout_ms = 50u;
    DFUStatusCode last_status = DFUStatusCode::OK;
  };

  /**
   * @brief Upload 会话状态 / Upload session state
   */
  struct UploadState
  {
    bool session_started = false;
    size_t offset = 0u;
    uint16_t expected_block_num = 0u;
    size_t image_size = 0u;
  };

  enum class WorkKind : uint8_t
  {
    WRITE,
    MANIFEST
  };
  static constexpr uint32_t EMPTY = 0U, STAGED = 1U, READY = 2U, RUNNING = 3U, DONE = 4U;
  static constexpr uint32_t CANCELLED = 0x80000000U;
  struct Work
  {
    WorkKind kind = WorkKind::WRITE;
    WriteState write{};
    size_t image_size = 0;
  } job_;
  std::atomic<uint32_t> job_state_{EMPTY};
  static constexpr uint32_t MIN_WRITE_POLL_MS = 10u;    ///< 报告的下限 / Lower bound
  static constexpr uint32_t MAX_WRITE_POLL_MS = 1000u;  ///< 报告的上限 / Upper bound
  static constexpr uint32_t MIN_REMAINING_POLL_MS = 5u;
  uint32_t job_started_ms_ = 0u;    ///< 当前步骤开始时刻 / Start of the current step
  bool write_needs_erase_ = false;  ///< 当前写入步骤要擦除 / Current step erases
  uint32_t measured_erase_write_ms_ = 25u;  ///< 上次擦除+编程耗时 / Last erase+program
  uint32_t measured_write_ms_ = 10u;        ///< 上次只编程耗时 / Last program-only step
  DFUStatusCode job_result_ = DFUStatusCode::OK;
  std::atomic<size_t> published_image_size_{0};
  std::atomic<uint32_t> launch_request_{0};

  void CancelWork()
  {
    uint32_t state = job_state_.load(std::memory_order_acquire);
    while (state != EMPTY)
    {
      const uint32_t phase = state & ~CANCELLED;
      const uint32_t next = phase == RUNNING || phase == DONE ? state | CANCELLED : EMPTY;
      if (job_state_.compare_exchange_weak(state, next, std::memory_order_acq_rel)) break;
    }
  }

  bool BeginFlashStep()
  {
    uint32_t state = job_state_.load(std::memory_order_acquire);
    if (state == EMPTY) return true;  // Constructor-only stored-image validation.
    if (state != RUNNING) return false;
    // A successful claim authorizes this one noncancelable Flash call. Cancel
    // after the claim lets it reach a safe boundary, not the following call.
    return job_state_.compare_exchange_strong(state, RUNNING, std::memory_order_acq_rel);
  }

  Flash& flash_;                  ///< 底层 flash 设备 / Underlying flash device
  size_t image_offset_ = 0u;      ///< 镜像区起始偏移 / Image base offset
  size_t image_size_limit_ = 0u;  ///< 镜像区总边界 / Image region limit
  size_t seal_offset_ = 0u;  ///< seal 相对镜像区偏移 / Seal offset inside image region
  JumpCallback jump_to_app_ = nullptr;  ///< 跳 app 回调 / App jump callback
  void* jump_app_ctx_ = nullptr;        ///< 跳转上下文 / Jump callback context
  bool autorun_ = true;    ///< manifest 后是否自动请求运行 / Autorun after manifest
  ImageState image_ = {};  ///< 镜像级状态 / Image-level state
  size_t erase_block_size_ = 1u;      ///< 最小擦除粒度 / Minimum erase granularity
  size_t erase_block_count_ = 0u;     ///< 受管块数量 / Number of tracked erase blocks
  uint8_t* erased_blocks_ = nullptr;  ///< 每块擦除标记 / Per-block erase marks
  size_t seal_storage_size_ = 0u;     ///< seal 暂存大小 / Seal scratch size
  uint8_t* seal_storage_ = nullptr;   ///< seal 暂存区 / Seal scratch buffer
  size_t transfer_size_ = 0u;         ///< 单次 DFU 传输上限 / Per-transfer DFU limit
  uint8_t* write_buffer_ = nullptr;   ///< 下载块暂存区 / Download chunk buffer
  uint8_t crc_buffer_[256] = {};      ///< CRC 分块缓冲 / CRC chunk buffer
  DownloadState download_ = {};       ///< Download 状态 / Download state
  WriteState write_ = {};             ///< 写入步骤状态 / Write-step state
  ManifestState manifest_ = {};       ///< Manifest 状态 / Manifest state
  UploadState upload_ = {};           ///< Upload 状态 / Upload state
};

/**
 * @brief 通用 DFU 前端：实现标准 DFU 状态机，backend 只负责存储细节
 *        Generic DFU frontend: implements the standard DFU state machine while
 *        the backend handles storage details only.
 */
template <typename Backend, size_t MAX_TRANSFER_SIZE = 4096u>
class DFUClass : public DfuInterfaceClassBase
{
  static_assert(MAX_TRANSFER_SIZE > 0u, "DFU transfer size must be non-zero.");
  static_assert(MAX_TRANSFER_SIZE <= 0xFFFFu,
                "DFU transfer size must fit in wTransferSize.");

  static constexpr uint8_t INTERFACE_CLASS = 0xFEu;
  static constexpr uint8_t INTERFACE_SUB_CLASS = 0x01u;
  static constexpr uint8_t INTERFACE_PROTOCOL = 0x02u;
  static constexpr uint16_t DFU_VERSION = 0x0110u;

  static constexpr uint8_t ATTR_CAN_DOWNLOAD = 0x01u;
  static constexpr uint8_t ATTR_CAN_UPLOAD = 0x02u;
  static constexpr uint8_t ATTR_MANIFESTATION_TOLERANT = 0x04u;
  static constexpr uint8_t ATTR_WILL_DETACH = 0x08u;

 public:
  LIBXR_PACKED_BEGIN
  /**
   * @brief DFU Functional Descriptor（固件模式）
   *        DFU Functional Descriptor for firmware mode.
   */
  struct FunctionalDescriptor
  {
    uint8_t bLength = 9;
    uint8_t bDescriptorType = 0x21;
    uint8_t bmAttributes = 0;
    uint16_t wDetachTimeOut = 0;
    uint16_t wTransferSize = 0;
    uint16_t bcdDFUVersion = DFU_VERSION;
  };

  /**
   * @brief GETSTATUS 返回包 / GETSTATUS response payload
   */
  struct StatusResponse
  {
    uint8_t bStatus = 0;
    uint8_t bwPollTimeout[3] = {0, 0, 0};
    uint8_t bState = 0;
    uint8_t iString = 0;

    void SetPollTimeout(uint32_t timeout_ms)
    {
      bwPollTimeout[0] = static_cast<uint8_t>(timeout_ms & 0xFFu);
      bwPollTimeout[1] = static_cast<uint8_t>((timeout_ms >> 8) & 0xFFu);
      bwPollTimeout[2] = static_cast<uint8_t>((timeout_ms >> 16) & 0xFFu);
    }
  };

  /**
   * @brief Bootloader DFU 的接口描述符块 / Bootloader DFU descriptor block
   */
  struct DescriptorBlock
  {
    ConfigDescriptorItem::InterfaceDescriptor interface_desc = {
        9,
        static_cast<uint8_t>(DescriptorType::INTERFACE),
        0,
        0,
        0,
        INTERFACE_CLASS,
        INTERFACE_SUB_CLASS,
        INTERFACE_PROTOCOL,
        0};
    FunctionalDescriptor func_desc = {};
  };
  LIBXR_PACKED_END

  static_assert(sizeof(FunctionalDescriptor) == 9,
                "DFU functional descriptor size mismatch.");
  static_assert(sizeof(StatusResponse) == 6, "DFU status response size mismatch.");

 public:
  static constexpr const char* DEFAULT_INTERFACE_STRING = "XRUSB DFU";
  static constexpr const char* DEFAULT_WINUSB_DEVICE_INTERFACE_GUID =
      DfuInterfaceClassBase::DEFAULT_WINUSB_DEVICE_INTERFACE_GUID;

  /**
   * @brief Backend 需要满足的接口契约 / Backend contract requirements
   *
   * - `DFUCapabilities GetDfuCapabilities() const`
   * - `ErrorCode DfuSetAlternate(uint8_t alt)`
   * - `void DfuAbort(uint8_t alt)`
   * - `void DfuClearStatus(uint8_t alt)`
   * - `DFUStatusCode DfuDownload(uint8_t alt, uint16_t block_num, ConstRawData data,
   *                              uint32_t& poll_timeout_ms)`
   * - `DFUStatusCode DfuGetDownloadStatus(uint8_t alt, bool& busy,
   *                                       uint32_t& poll_timeout_ms)`
   * - `size_t DfuUpload(uint8_t alt, uint16_t block_num, RawData data,
   *                     DFUStatusCode& status, uint32_t& poll_timeout_ms)`
   * - `DFUStatusCode DfuManifest(uint8_t alt, uint32_t& poll_timeout_ms)`
   * - `DFUStatusCode DfuGetManifestStatus(uint8_t alt, bool& busy,
   *                                       uint32_t& poll_timeout_ms)`
   * - `void AllowPendingWork()`：owner 上下文，主机确认 GETSTATUS 后放行排好的工作 /
   *   owner context, releases the staged work once the host acknowledged GETSTATUS
   * - `bool RunPendingWork()`：Process() 的调用者上下文，执行放行的工作，执行了返回
   *   true / the context that calls Process(); runs released work, true if it ran
   * - `void CommitWorkResult()`：owner 上下文（OnService），提交完成的结果 /
   *   owner context (OnService), commits a finished result
   * - `void DfuCommitManifestWaitReset(uint8_t alt)`
   */
  explicit DFUClass(
      Backend& backend, const char* interface_string = DEFAULT_INTERFACE_STRING,
      const char* webusb_landing_page_url = nullptr,
      uint8_t webusb_vendor_code = LibXR::USB::WebUsb::WEBUSB_VENDOR_CODE_DEFAULT,
      const char* winusb_device_interface_guid = DEFAULT_WINUSB_DEVICE_INTERFACE_GUID)
      : DfuInterfaceClassBase(interface_string, webusb_landing_page_url,
                              webusb_vendor_code, winusb_device_interface_guid),
        backend_(backend)
  {
  }

  /**
   * @brief 应用主循环周期调用：执行后端放行的擦写/校验工作 /
   *        Called periodically by the application loop: run the erase/program/verify
   *        work the backend has released
   *
   * 慢工作只在调用本函数的上下文里执行，不在 USB 中断里；执行完请求类服务，由 owner
   * 在 OnService 里提交结果，随后的 GETSTATUS 看到它。
   * The slow work runs only in the context that calls this function, never in the USB
   * interrupt; afterwards the class service is requested and the owner commits the
   * result in OnService, where the next GETSTATUS sees it.
   */
  void Process()
  {
    if (backend_.RunPendingWork()) this->RequestClassService(false);
  }

  void OnService(bool) override { backend_.CommitWorkResult(); }

  size_t GetControlReceiveCapacity() const override { return MAX_TRANSFER_SIZE; }

 protected:
  void BindEndpoints(EndpointPool&, uint8_t start_itf_num, bool) override
  {
    // 固件态 DFU 在绑定阶段发布单接口描述符，并校验 backend 能力。
    // Firmware DFU publishes one interface and validates backend capabilities during
    // bind.
    interface_num_ = start_itf_num;
    current_alt_setting_ = 0u;

    caps_ = backend_.GetDfuCapabilities();
    if (caps_.transfer_size == 0u || caps_.transfer_size > MAX_TRANSFER_SIZE)
    {
      caps_.transfer_size = static_cast<uint16_t>(MAX_TRANSFER_SIZE);
    }

    desc_block_.interface_desc.bInterfaceNumber = interface_num_;
    desc_block_.interface_desc.bAlternateSetting = 0u;
    desc_block_.interface_desc.iInterface = GetInterfaceStringIndex(0u);
    desc_block_.func_desc.bmAttributes = BuildAttributeBitmap(caps_);
    desc_block_.func_desc.wDetachTimeOut = caps_.detach_timeout_ms;
    desc_block_.func_desc.wTransferSize = caps_.transfer_size;
    desc_block_.func_desc.bcdDFUVersion = DFU_VERSION;

    SetData(RawData{reinterpret_cast<uint8_t*>(&desc_block_), sizeof(desc_block_)});

    ResetProtocolState();
    auto ec = backend_.DfuSetAlternate(current_alt_setting_);
    if (ec != ErrorCode::OK)
    {
      inited_ = false;
      return;
    }
    inited_ = true;
  }

  void UnbindEndpoints(EndpointPool&, bool) override
  {
    if (inited_)
    {
      // 解绑时先把 backend 置回 abort 状态，
      // 这样下一次绑定总是从干净的协议会话开始。
      // Unbind must leave the backend in an aborted state so the next bind
      // always starts from a clean protocol session.
      backend_.DfuAbort(current_alt_setting_);
    }
    inited_ = false;
    ResetProtocolState();
  }

  size_t GetInterfaceCount() override { return 1u; }
  bool HasIAD() override { return false; }
  size_t GetMaxConfigSize() override { return sizeof(desc_block_); }
  ErrorCode WriteDeviceDescriptor(DeviceDescriptor& header) override
  {
    header.data_.bDeviceClass = DeviceDescriptor::ClassID::APPLICATION_SPECIFIC;
    header.data_.bDeviceSubClass = INTERFACE_SUB_CLASS;
    header.data_.bDeviceProtocol = INTERFACE_PROTOCOL;
    return ErrorCode::OK;
  }

  ErrorCode SetAltSetting(uint8_t itf, uint8_t alt) override
  {
    if (itf != interface_num_)
    {
      return ErrorCode::NOT_FOUND;
    }
    if (alt != 0u)
    {
      return ErrorCode::NOT_SUPPORT;
    }
    if (state_ == DFUState::DFU_DNBUSY || state_ == DFUState::DFU_MANIFEST ||
        state_ == DFUState::DFU_MANIFEST_WAIT_RESET)
    {
      // 协议仍处于非空闲阶段时，不允许切换 alternate setting。
      // Alternate setting cannot change while the protocol is in a non-idle phase.
      return ErrorCode::BUSY;
    }

    auto ec = backend_.DfuSetAlternate(alt);
    if (ec != ErrorCode::OK)
    {
      return ec;
    }

    backend_.DfuAbort(current_alt_setting_);
    current_alt_setting_ = alt;
    ClearErrorState();
    return ErrorCode::OK;
  }

  ErrorCode GetAltSetting(uint8_t itf, uint8_t& alt) override
  {
    if (itf != interface_num_)
    {
      return ErrorCode::NOT_FOUND;
    }
    alt = current_alt_setting_;
    return ErrorCode::OK;
  }

  ErrorCode OnGetDescriptor(bool, uint8_t, uint16_t wValue, uint16_t,
                            ConstRawData& out_data) override
  {
    const uint8_t desc_type = static_cast<uint8_t>((wValue >> 8) & 0xFFu);
    if (desc_type != desc_block_.func_desc.bDescriptorType)
    {
      return ErrorCode::NOT_SUPPORT;
    }

    out_data = {reinterpret_cast<const uint8_t*>(&desc_block_.func_desc),
                sizeof(desc_block_.func_desc)};
    return ErrorCode::OK;
  }

  ErrorCode OnControlRequest(bool in_isr, const SetupPacket& setup,
                             ControlTransferResult& result) override
  {
    if ((setup.bmRequestType & REQ_TYPE_MASK) != static_cast<uint8_t>(RequestType::CLASS))
      return DeviceClass::OnControlRequest(in_isr, setup, result);
    const auto request = static_cast<DFURequest>(setup.bRequest);
    const bool input = request == DFURequest::UPLOAD ||
                       request == DFURequest::GETSTATUS ||
                       request == DFURequest::GETSTATE;
    if ((setup.bmRequestType & REQ_DIRECTION_MASK) != (input ? 0x80U : 0U) ||
        (setup.bmRequestType & REQ_RECIPIENT_MASK) !=
            static_cast<uint8_t>(Recipient::INTERFACE) ||
        (request == DFURequest::GETSTATUS && setup.wLength != sizeof(StatusResponse)) ||
        (request == DFURequest::GETSTATE && setup.wLength != 1U))
      return ErrorCode::ARG_ERR;
    const uint32_t now = static_cast<uint32_t>(Timebase::GetMilliseconds());
    if (state_ == DFUState::DFU_DNBUSY || state_ == DFUState::DFU_MANIFEST)
    {
      if (!poll_waiting_ || static_cast<int32_t>(now - poll_deadline_) < 0)
        return ProtocolStall(DFUStatusCode::ERR_STALLEDPKT);
      state_ = state_ == DFUState::DFU_DNBUSY ? DFUState::DFU_DNLOAD_SYNC
                                              : DFUState::DFU_MANIFEST_SYNC;
      poll_waiting_ = false;
    }
    return OnClassRequest(in_isr, setup.bRequest, setup.wValue, setup.wLength,
                          setup.wIndex, result);
  }

  void OnControlAbort(bool, const SetupPacket& setup) override
  {
    if (setup.bRequest == static_cast<uint8_t>(DFURequest::DNLOAD))
    {
      backend_.DfuAbort(current_alt_setting_);
      ClearErrorState();
    }
    else if (setup.bRequest == static_cast<uint8_t>(DFURequest::GETSTATUS) &&
             !poll_waiting_)
    {
      if (state_ == DFUState::DFU_DNBUSY)
        state_ = DFUState::DFU_DNLOAD_SYNC;
      else if (state_ == DFUState::DFU_MANIFEST)
        state_ = DFUState::DFU_MANIFEST_SYNC;
    }
  }

  ErrorCode OnClassRequest(bool, uint8_t bRequest, uint16_t wValue, uint16_t wLength,
                           uint16_t wIndex, ControlTransferResult& result) override
  {
    if (!inited_)
    {
      return ErrorCode::INIT_ERR;
    }
    if ((wIndex & 0xFFu) != interface_num_)
    {
      return ErrorCode::NOT_FOUND;
    }

    switch (static_cast<DFURequest>(bRequest))
    {
      case DFURequest::DETACH:
        return HandleDetach(result);

      case DFURequest::DNLOAD:
        return HandleDnload(wValue, wLength, result);

      case DFURequest::UPLOAD:
        return HandleUpload(wValue, wLength, result);

      case DFURequest::GETSTATUS:
        return HandleGetStatus(result);

      case DFURequest::CLRSTATUS:
        return HandleClearStatus(result);

      case DFURequest::GETSTATE:
        return HandleGetState(result);

      case DFURequest::ABORT:
        return HandleAbort(result);

      default:
        return ProtocolStall(DFUStatusCode::ERR_STALLEDPKT);
    }
  }

  ErrorCode OnClassData(bool, uint8_t bRequest, LibXR::ConstRawData& data) override
  {
    const auto request = static_cast<DFURequest>(bRequest);
    UNUSED(data);

    if (request != DFURequest::DNLOAD)
    {
      return ErrorCode::NOT_SUPPORT;
    }

    if (pending_dnload_length_ == 0u)
    {
      // 这里出现零长度 OUT data stage 只在“DNLOAD 已结束”的路径上才是合法的。
      // A zero-length OUT data stage is legal here only for the already-finished
      // DNLOAD path.
      return ErrorCode::OK;
    }

    // 数据阶段长度必须与 SETUP 里承诺的 wLength 一致：短包意味着主机发的是
    // 截断的块，按整块暂存会把垃圾尾部一起写进 flash。
    // The data stage length must match the wLength promised by the SETUP: a short
    // packet means the host sent a truncated block, and staging it as a full
    // block would write the garbage tail into flash as well.
    if (data.size_ != pending_dnload_length_)
    {
      pending_dnload_length_ = 0u;
      state_ = DFUState::DFU_ERROR;
      return ErrorCode::ARG_ERR;
    }

    uint32_t poll_timeout_ms = 0u;
    auto status = backend_.DfuDownload(current_alt_setting_, pending_block_num_, data,
                                       poll_timeout_ms);
    pending_dnload_length_ = 0u;
    poll_timeout_ms_ = poll_timeout_ms;

    if (status == DFUStatusCode::OK)
    {
      state_ = DFUState::DFU_DNLOAD_SYNC;
      status_ = DFUStatusCode::OK;
      download_started_ = true;
    }
    else
    {
      EnterErrorState(status);
    }
    return ErrorCode::OK;
  }

  void OnClassInDataStatusComplete(bool, uint8_t bRequest) override
  {
    if (static_cast<DFURequest>(bRequest) != DFURequest::GETSTATUS)
    {
      return;
    }
    if (state_ == DFUState::DFU_DNBUSY || state_ == DFUState::DFU_MANIFEST)
    {
      // 期限已在应答时设好；主机确认了应答，后端才开始这一步的工作。
      // The deadline was set with the answer; the backend starts the step's work once
      // the host has acknowledged it.
      backend_.AllowPendingWork();
    }
    if (state_ == DFUState::DFU_MANIFEST_WAIT_RESET)
    {
      // 只有在主机完成了报告 MANIFEST-WAIT-RESET 的那笔 GETSTATUS 的 STATUS OUT 后，
      // 才真正提交 autorun。
      // Commit autorun only after the host completes the STATUS OUT of the
      // GETSTATUS request that reported MANIFEST-WAIT-RESET.
      backend_.DfuCommitManifestWaitReset(current_alt_setting_);
    }
  }

 private:
  static constexpr uint8_t BuildAttributeBitmap(const DFUCapabilities& caps)
  {
    return static_cast<uint8_t>(
        (caps.can_download ? ATTR_CAN_DOWNLOAD : 0u) |
        (caps.can_upload ? ATTR_CAN_UPLOAD : 0u) |
        (caps.manifestation_tolerant ? ATTR_MANIFESTATION_TOLERANT : 0u) |
        (caps.will_detach ? ATTR_WILL_DETACH : 0u));
  }

  ErrorCode HandleDetach(ControlTransferResult&)
  {
    // 固件态 DFU 不实现 DETACH；该请求由 runtime DFU 处理。
    // Firmware-mode DFU does not implement DETACH; runtime DFU owns that request.
    return ProtocolStall(DFUStatusCode::ERR_STALLEDPKT);
  }

  ErrorCode HandleDnload(uint16_t block_num, uint16_t wLength,
                         ControlTransferResult& result)
  {
    if (!caps_.can_download)
    {
      return ProtocolStall(DFUStatusCode::ERR_STALLEDPKT);
    }
    if (!(state_ == DFUState::DFU_IDLE || state_ == DFUState::DFU_DNLOAD_IDLE))
    {
      return ProtocolStall(DFUStatusCode::ERR_STALLEDPKT);
    }
    if (wLength > caps_.transfer_size)
    {
      return ProtocolStall(DFUStatusCode::ERR_USBR);
    }

    if (wLength == 0u)
    {
      // 零长度 DNLOAD 表示结束数据阶段并进入 manifest，
      // 但前提是之前至少成功接收过一个 payload block。
      // A zero-length DNLOAD terminates the data phase and requests manifest,
      // but only after at least one payload block was accepted.
      pending_dnload_length_ = 0u;
      pending_block_num_ = block_num;
      if (!download_started_)
      {
        EnterErrorState(DFUStatusCode::ERR_NOTDONE);
      }
      else
      {
        state_ = DFUState::DFU_MANIFEST_SYNC;
        status_ = DFUStatusCode::OK;
        poll_timeout_ms_ = 0u;
      }
      // 无数据阶段的控制传输由核心自行完成状态阶段（STATUS ZLP）。
      // A control transfer without a data stage finishes its status stage in the core.
      return ErrorCode::OK;
    }

    pending_block_num_ = block_num;
    pending_dnload_length_ = wLength;
    // OUT 数据阶段由核心收进自己的缓冲区，收完整段后经 OnControlData ->
    // OnClassData 交给 backend；wTransferSize 块长由 GetControlReceiveCapacity 覆盖。
    // The core receives the whole OUT data stage into its own buffer and hands it to
    // the backend via OnControlData -> OnClassData; the wTransferSize block length is
    // covered by GetControlReceiveCapacity.
    return ErrorCode::OK;
  }

  ErrorCode HandleUpload(uint16_t block_num, uint16_t wLength,
                         ControlTransferResult& result)
  {
    if (!caps_.can_upload)
    {
      return ProtocolStall(DFUStatusCode::ERR_STALLEDPKT);
    }
    if (!(state_ == DFUState::DFU_IDLE || state_ == DFUState::DFU_UPLOAD_IDLE))
    {
      return ProtocolStall(DFUStatusCode::ERR_STALLEDPKT);
    }
    if (wLength == 0u)
    {
      return ProtocolStall(DFUStatusCode::ERR_USBR);
    }

    size_t req_size = wLength;
    if (req_size > caps_.transfer_size)
    {
      req_size = caps_.transfer_size;
    }

    DFUStatusCode op_status = DFUStatusCode::OK;
    uint32_t poll_timeout_ms = 0u;
    const size_t read_size =
        backend_.DfuUpload(current_alt_setting_, block_num, {transfer_buffer_, req_size},
                           op_status, poll_timeout_ms);

    poll_timeout_ms_ = poll_timeout_ms;
    if (op_status != DFUStatusCode::OK)
    {
      return ProtocolStall(op_status);
    }
    if (read_size > req_size)
    {
      return ProtocolStall(DFUStatusCode::ERR_USBR);
    }

    status_ = DFUStatusCode::OK;
    state_ = (read_size < req_size) ? DFUState::DFU_IDLE : DFUState::DFU_UPLOAD_IDLE;
    result.InData() = {transfer_buffer_, read_size};
    return ErrorCode::OK;
  }

  ErrorCode HandleGetStatus(ControlTransferResult& result)
  {
    AdvanceStateForStatusRead();
    if (state_ == DFUState::DFU_DNBUSY || state_ == DFUState::DFU_MANIFEST)
    {
      // 等待从应答交出时算起，并少算 1 ms 抵消毫秒取整：按 bwPollTimeout 等待的主机
      // 不会早于期限。状态阶段的完成中断可能因 flash 擦写关中断而推迟，不能从那里算。
      // The wait counts from handing over this answer, 1 ms short to absorb the
      // millisecond rounding, so a host that waits bwPollTimeout never comes early.
      // The status stage can complete late while flash work runs with interrupts off,
      // so the wait must not count from there.
      const uint32_t wait = poll_timeout_ms_ > 0u ? poll_timeout_ms_ - 1u : 0u;
      poll_deadline_ = static_cast<uint32_t>(Timebase::GetMilliseconds()) + wait;
      poll_waiting_ = true;
    }
    status_response_.bStatus = static_cast<uint8_t>(status_);
    status_response_.SetPollTimeout(poll_timeout_ms_);
    status_response_.bState = static_cast<uint8_t>(state_);
    status_response_.iString = 0u;
    result.InData() = {reinterpret_cast<const uint8_t*>(&status_response_),
                       sizeof(status_response_)};
    return ErrorCode::OK;
  }

  ErrorCode HandleClearStatus(ControlTransferResult& result)
  {
    if (state_ != DFUState::DFU_ERROR)
    {
      return ProtocolStall(DFUStatusCode::ERR_STALLEDPKT);
    }
    backend_.DfuClearStatus(current_alt_setting_);
    ClearErrorState();
    return ErrorCode::OK;
  }

  ErrorCode HandleGetState(ControlTransferResult& result)
  {
    state_response_ = static_cast<uint8_t>(state_);
    result.InData() = {&state_response_, sizeof(state_response_)};
    return ErrorCode::OK;
  }

  ErrorCode HandleAbort(ControlTransferResult& result)
  {
    switch (state_)
    {
      case DFUState::DFU_IDLE:
      case DFUState::DFU_DNLOAD_SYNC:
      case DFUState::DFU_DNLOAD_IDLE:
      case DFUState::DFU_UPLOAD_IDLE:
      case DFUState::DFU_MANIFEST_SYNC:
        backend_.DfuAbort(current_alt_setting_);
        ClearErrorState();
        return ErrorCode::OK;

      default:
        return ProtocolStall(DFUStatusCode::ERR_STALLEDPKT);
    }
  }

  void AdvanceStateForStatusRead()
  {
    // 标准 DFU 状态机是在 GETSTATUS 这个同步点上推进的。
    // GETSTATUS is the synchronization point that advances the standard DFU state
    // machine.
    switch (state_)
    {
      case DFUState::DFU_DNLOAD_SYNC:
        RefreshDownloadStatus();
        break;

      case DFUState::DFU_MANIFEST_SYNC:
      {
        if (manifest_started_)
        {
          RefreshManifestStatus();
          break;
        }
        if (status_ != DFUStatusCode::OK)
        {
          state_ = DFUState::DFU_ERROR;
          break;
        }

        uint32_t poll_timeout_ms = 0u;
        auto manifest_status =
            backend_.DfuManifest(current_alt_setting_, poll_timeout_ms);
        poll_timeout_ms_ = poll_timeout_ms;
        if (manifest_status == DFUStatusCode::OK)
        {
          status_ = DFUStatusCode::OK;
          state_ = DFUState::DFU_MANIFEST;
          manifest_started_ = true;
        }
        else
        {
          EnterErrorState(manifest_status);
        }
        break;
      }

      case DFUState::DFU_DNBUSY:
        RefreshDownloadStatus();
        break;

      case DFUState::DFU_MANIFEST:
        RefreshManifestStatus();
        break;

      default:
        break;
    }
  }

  void RefreshDownloadStatus()
  {
    if (status_ != DFUStatusCode::OK)
    {
      state_ = DFUState::DFU_ERROR;
      return;
    }

    bool busy = false;
    uint32_t poll_timeout_ms = 0u;
    const auto download_status =
        backend_.DfuGetDownloadStatus(current_alt_setting_, busy, poll_timeout_ms);
    poll_timeout_ms_ = poll_timeout_ms;
    if (download_status != DFUStatusCode::OK)
    {
      EnterErrorState(download_status);
      return;
    }

    // backend 报 busy 时映射到 DFU_DNBUSY；
    // 否则主机可以从 DFU_DNLOAD_IDLE 继续发送下一块 DNLOAD。
    // A backend-reported busy maps to DFU_DNBUSY; otherwise the host may
    // continue sending the next DNLOAD block from DFU_DNLOAD_IDLE.
    state_ = busy ? DFUState::DFU_DNBUSY : DFUState::DFU_DNLOAD_IDLE;
  }

  void RefreshManifestStatus()
  {
    if (status_ != DFUStatusCode::OK)
    {
      state_ = DFUState::DFU_ERROR;
      return;
    }

    bool busy = false;
    uint32_t poll_timeout_ms = 0u;
    const auto manifest_status =
        backend_.DfuGetManifestStatus(current_alt_setting_, busy, poll_timeout_ms);
    poll_timeout_ms_ = poll_timeout_ms;
    if (manifest_status != DFUStatusCode::OK)
    {
      EnterErrorState(manifest_status);
      return;
    }

    if (busy)
    {
      state_ = DFUState::DFU_MANIFEST;
      return;
    }

    // manifest 完成后，要么回到 IDLE（tolerant），要么进入 WAIT_RESET（non-tolerant）。
    // Manifest completion returns either to IDLE (tolerant) or WAIT_RESET (non-tolerant).
    download_started_ = false;
    state_ = caps_.manifestation_tolerant ? DFUState::DFU_IDLE
                                          : DFUState::DFU_MANIFEST_WAIT_RESET;
  }

  void ResetProtocolState()
  {
    // 这里只重置前端拥有的协议状态；
    // 镜像级 bookkeeping 保留在 backend_ 里。
    // Reset only frontend-owned protocol state;
    // image-level bookkeeping stays in backend_.
    poll_waiting_ = false;
    manifest_started_ = false;
    pending_block_num_ = 0u;
    pending_dnload_length_ = 0u;
    poll_timeout_ms_ = 0u;
    current_alt_setting_ = 0u;
    download_started_ = false;
    ClearErrorState();
  }

  void ClearErrorState()
  {
    poll_waiting_ = false;
    manifest_started_ = false;
    status_ = DFUStatusCode::OK;
    state_ = DFUState::DFU_IDLE;
    poll_timeout_ms_ = 0u;
  }

  void EnterErrorState(DFUStatusCode status)
  {
    // 一旦锁存错误，前端会保持在 DFU_ERROR，
    // 直到 CLRSTATUS/ABORT 把它清掉。
    // Once an error is latched, the frontend stays in DFU_ERROR
    // until CLRSTATUS/ABORT clears it.
    status_ = status;
    state_ = DFUState::DFU_ERROR;
    poll_timeout_ms_ = 0u;
  }

  ErrorCode ProtocolStall(DFUStatusCode status)
  {
    EnterErrorState(status);
    return ErrorCode::ARG_ERR;
  }

 private:
  Backend& backend_;                     ///< 后端实现 / Backend implementation
  DFUCapabilities caps_ = {};            ///< 能力集缓存 / Capability cache
  DescriptorBlock desc_block_ = {};      ///< 描述符缓存 / Descriptor cache
  StatusResponse status_response_ = {};  ///< GETSTATUS 缓冲区 / GETSTATUS buffer
  uint8_t state_response_ = 0u;          ///< GETSTATE 缓冲字节 / GETSTATE byte buffer
  uint8_t transfer_buffer_[MAX_TRANSFER_SIZE] =
      {};                          ///< EP0 传输缓冲 / EP0 transfer buffer
  bool download_started_ = false;  ///< 是否已有有效下载数据 / Whether payload has started
  uint16_t pending_block_num_ = 0u;      ///< 待提交 block 编号 / Pending block number
  uint16_t pending_dnload_length_ = 0u;  ///< 待提交 DNLOAD 长度 / Pending DNLOAD length
  uint32_t poll_deadline_ = 0;
  bool poll_waiting_ = false;
  bool manifest_started_ = false;
  uint32_t poll_timeout_ms_ = 0u;             ///< 当前轮询超时 / Current poll timeout
  DFUState state_ = DFUState::DFU_IDLE;       ///< DFU 状态 / DFU state
  DFUStatusCode status_ = DFUStatusCode::OK;  ///< DFU 状态码 / DFU status code
};

/**
 * @brief 把 backend 与前端 DFUClass 组装在一起的存储基类
 *        Storage base that assembles the backend and the DFU frontend class.
 */
class DfuBootloaderClassStorage
{
 protected:
  using JumpCallback = DfuBootloaderBackend::JumpCallback;

  DfuBootloaderClassStorage(Flash& flash, size_t image_base, size_t image_limit,
                            size_t seal_offset, JumpCallback jump_to_app,
                            void* jump_app_ctx = nullptr, bool autorun = true)
      : backend_(flash, image_base, image_limit, seal_offset, jump_to_app, jump_app_ctx,
                 autorun),
        image_base_(image_base),
        image_limit_(image_limit),
        seal_offset_(seal_offset)
  {
  }

  DfuBootloaderBackend backend_;
  size_t image_base_ = 0u;
  size_t image_limit_ = 0u;
  size_t seal_offset_ = 0u;
};

/**
 * @brief 面向单镜像 bootloader 区的 DFU 类
 *        Bootloader DFU class for a single image region.
 */
template <size_t MAX_TRANSFER_SIZE = 4096u>
class DfuBootloaderClassT : private DfuBootloaderClassStorage,
                            public DFUClass<DfuBootloaderBackend, MAX_TRANSFER_SIZE>
{
  using Storage = DfuBootloaderClassStorage;
  using Base = DFUClass<DfuBootloaderBackend, MAX_TRANSFER_SIZE>;

 public:
  using JumpCallback = DfuBootloaderBackend::JumpCallback;
  static constexpr uint8_t VENDOR_REQUEST_RUN_APP = 0x5Au;

  DfuBootloaderClassT(
      Flash& flash, size_t image_base, size_t image_limit, size_t seal_offset,
      JumpCallback jump_to_app, void* jump_app_ctx = nullptr, bool autorun = true,
      const char* interface_string = Base::DEFAULT_INTERFACE_STRING,
      const char* webusb_landing_page_url = nullptr,
      uint8_t webusb_vendor_code = LibXR::USB::WebUsb::WEBUSB_VENDOR_CODE_DEFAULT,
      const char* winusb_device_interface_guid =
          Base::DEFAULT_WINUSB_DEVICE_INTERFACE_GUID)
      : Storage(flash, image_base, image_limit, seal_offset, jump_to_app, jump_app_ctx,
                autorun),
        Base(Storage::backend_, interface_string, webusb_landing_page_url,
             webusb_vendor_code, winusb_device_interface_guid)
  {
  }

  // Process() 来自 DFUClass，只推进 backend 的擦写/校验工作；跳 app 仍保持显式调用
  // TryConsumeAppLaunch()。
  // Process() comes from DFUClass and only advances the backend's erase/program/verify
  // work; launching the app stays an explicit TryConsumeAppLaunch() call.
  using Base::Process;
  bool RequestRunApp() { return Storage::backend_.TryRequestRunApp(); }

  bool TryConsumeAppLaunch(uint32_t now_ms)
  {
    return Storage::backend_.TryConsumeAppLaunch(now_ms);
  }

  bool HasPendingWork() const { return Storage::backend_.HasPendingWork(); }
  bool HasValidImage() const { return Storage::backend_.HasValidImage(); }
  size_t ImageSize() const { return Storage::backend_.ImageSize(); }
  size_t ImageBase() const { return Storage::image_base_; }
  size_t ImageLimit() const { return Storage::image_limit_; }
  size_t SealOffset() const { return Storage::seal_offset_; }

 protected:
  void OnControlComplete(bool in_isr, const SetupPacket& setup) override
  {
    Base::OnControlComplete(in_isr, setup);
    if ((setup.bmRequestType & REQ_TYPE_MASK) ==
            static_cast<uint8_t>(RequestType::VENDOR) &&
        setup.bRequest == VENDOR_REQUEST_RUN_APP)
      (void)Storage::backend_.TryRequestRunApp();
  }

  ErrorCode OnVendorRequest(bool, uint8_t bRequest, uint16_t wValue, uint16_t wLength,
                            uint16_t,
                            typename Base::ControlTransferResult& result) override
  {
    if (bRequest != VENDOR_REQUEST_RUN_APP)
    {
      return ErrorCode::NOT_SUPPORT;
    }
    if (wLength != 0u || wValue != 0u)
    {
      return ErrorCode::ARG_ERR;
    }
    if (Storage::backend_.HasPendingWork())
    {
      // 协议拥有的异步工作尚未完成时，不允许启动 app。
      // Do not launch the app while protocol-owned async work is still pending.
      return ErrorCode::BUSY;
    }
    if (!Storage::backend_.HasValidImage())
    {
      // RUN_APP 只在已知存在有效 seal 镜像时才会接受。
      // RUN_APP is only accepted when a valid sealed image is known.
      return ErrorCode::FAILED;
    }
    // 无数据阶段的控制传输由核心自行完成状态阶段（STATUS ZLP）。
    // A control transfer without a data stage finishes its status stage in the core.
    return ErrorCode::OK;
  }
};

using DfuBootloaderClass = DfuBootloaderClassT<4096u>;

}  // namespace LibXR::USB
