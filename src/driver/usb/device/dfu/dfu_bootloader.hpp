#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

#include "dfu_def.hpp"
#include "flash.hpp"
#include "timebase.hpp"

namespace LibXR::USB
{

/**
 * @class DfuBootloaderBackend
 * @brief bootloader DFU 的 Flash 后端：下载、上传、manifest 与 seal / Flash backend of
 *        the bootloader DFU: download, upload, manifestation and the seal
 *
 * 镜像放在 Flash 的 [image_offset, image_offset + seal_offset)，其后是 seal 记录（魔数、
 * 长度、CRC32 和它的反码）。下载的块先拷进内部缓冲，擦写和 manifest 的校验只在
 * RunPendingWork() 里做，不在 USB 中断里。
 * The image lives at [image_offset, image_offset + seal_offset) of the flash, followed
 * by the seal record (magic, length, CRC32 and its inverse). A downloaded block is first
 * copied into an internal buffer; erase, program and the manifestation check run only in
 * RunPendingWork(), never in the USB interrupt.
 *
 * @note 工作槽 job_state_ 的状态：EMPTY → STAGED（owner 暂存）→ READY（owner 放行）→
 *       RUNNING（RunPendingWork 认领）→ DONE → EMPTY（owner 在 CommitWorkResult 里取
 *       走）；CANCELLED 位由 owner 在 RUNNING、DONE 时置上，表示结果作废。标"仅 owner"的
 *       函数只在设备 owner 中调用。
 *       States of the job slot job_state_: EMPTY → STAGED (staged by the owner) → READY
 *       (released by the owner) → RUNNING (claimed by RunPendingWork) → DONE → EMPTY
 *       (taken by the owner in CommitWorkResult); the owner sets the CANCELLED bit while
 *       RUNNING or DONE to void the result. Functions marked "owner only" are called in
 *       the device's owner only.
 */
class DfuBootloaderBackend
{
 public:
  /// 跳到应用的函数，参数为 jump_app_ctx / Function that jumps to the application; its
  /// argument is jump_app_ctx
  using JumpFun = void (*)(void*);
  /// seal 魔数（ASCII "SEAL" 小端）/ Seal magic (ASCII "SEAL", little-endian)
  static constexpr uint32_t SEAL_MAGIC = 0x4C414553U;

  LIBXR_PACKED_BEGIN
  /// seal 记录 / Seal record
  struct SealRecord
  {
    uint32_t magic = SEAL_MAGIC;  ///< SEAL_MAGIC
    uint32_t image_size = 0U;     ///< 镜像字节数 / Image size in bytes
    uint32_t crc32 = 0U;          ///< 镜像的 CRC32 / CRC32 of the image
    uint32_t crc32_inv = 0U;      ///< crc32 的反码 / Inverse of crc32
  };
  LIBXR_PACKED_END

  /**
   * @brief 构造并检查 Flash 里已有的镜像 / Construct and check the image already in
   *        the flash
   * @param flash 放镜像和 seal 的 Flash / Flash that holds the image and the seal
   * @param image_offset 镜像区起点 / Start of the image region
   * @param image_size_limit 镜像区字节数，含 seal / Bytes of the image region, seal
   *        included
   * @param seal_offset seal 相对镜像区起点的偏移，也是镜像的上限 / Offset of the seal
   *        from the image start, also the image limit
   * @param jump_to_app 启动应用的函数 / Function that launches the application
   * @param jump_app_ctx 传给它的参数 / Its argument
   * @param autorun manifest 完成后是否自动请求启动 / Whether manifestation requests a
   *        launch by itself
   * @note 初始化时；分配擦除表、seal 暂存和下载缓冲，读 Flash 算 CRC。
   *       At initialization; allocates the erase map, the seal scratch and the download
   *       buffer, and reads the flash for the CRC.
   */
  DfuBootloaderBackend(Flash& flash, size_t image_offset, size_t image_size_limit,
                       size_t seal_offset, JumpFun jump_to_app,
                       void* jump_app_ctx = nullptr, bool autorun = true);

  DfuBootloaderBackend(const DfuBootloaderBackend&) = delete;
  DfuBootloaderBackend& operator=(const DfuBootloaderBackend&) = delete;

  /// @name 前端调用，仅 owner / Called by the frontend, owner only
  /// @{

  /// 能力；构造后不变 / Capabilities; fixed after construction
  DFUCapabilities GetDfuCapabilities() const;
  /// 只支持 alt 0 / Only alternate 0
  ErrorCode DfuSetAlternate(uint8_t alt);
  /// 放弃会话：作废工作，清传输状态；不取消已登记的启动 / Abort the session: void the
  /// work and clear the transfer state; a recorded launch is not cancelled
  void DfuAbort(uint8_t alt);
  /// 同 DfuAbort / Same as DfuAbort
  void DfuClearStatus(uint8_t alt);

  /**
   * @brief 接收一块 DNLOAD，暂存为待写 / Take one DNLOAD block and stage it
   * @param alt 备用设置号；非 0 返回 ERR_TARGET / Alternate setting; non-zero returns
   *        ERR_TARGET
   * @param block_num 块号；0 开始新会话，之后必须连续 / Block number; 0 starts a new
   *        session, then consecutive
   * @param data 块数据，不超过传输大小 / Block data, at most the transfer size
   * @param poll_timeout_ms 输出：主机应等的毫秒数 / Output: milliseconds the host waits
   * @return OK：已暂存；其他：DFU 状态码 / OK: staged; otherwise a DFU status code
   */
  DFUStatusCode DfuDownload(uint8_t alt, uint16_t block_num, ConstRawData data,
                            uint32_t& poll_timeout_ms);

  /// 最近一次写入的结果；busy 表示还在写 / Result of the latest write; busy while it
  /// still runs
  DFUStatusCode DfuGetDownloadStatus(uint8_t alt, bool& busy, uint32_t& poll_timeout_ms);

  /**
   * @brief UPLOAD 一块，从 Flash 读回镜像 / UPLOAD one block, read back from the flash
   * @return 写进 data 的字节数；0 表示结束或出错（看 status）/ Bytes written into
   *         data; 0 at the end or on error (see status)
   */
  size_t DfuUpload(uint8_t alt, uint16_t block_num, RawData data, DFUStatusCode& status,
                   uint32_t& poll_timeout_ms);

  /// 暂存 manifest（算 CRC、写 seal）/ Stage manifestation (CRC, seal)
  DFUStatusCode DfuManifest(uint8_t alt, uint32_t& poll_timeout_ms);
  /// manifest 的结果；busy 表示还在做 / Manifestation result; busy while it runs
  DFUStatusCode DfuGetManifestStatus(uint8_t alt, bool& busy, uint32_t& poll_timeout_ms);
  /// 主机确认了 GETSTATUS：STAGED 改为 READY / The host acknowledged GETSTATUS: STAGED
  /// becomes READY
  void AllowPendingWork();
  /// 把 DONE 的结果并入会话状态并清空工作槽 / Fold a DONE result into the session and
  /// empty the job slot
  void CommitWorkResult();
  /// 到 MANIFEST-WAIT-RESET 且主机确认：autorun 时登记启动 / Reached
  /// MANIFEST-WAIT-RESET and acknowledged: record a launch when autorun is on
  void DfuCommitManifestWaitReset(uint8_t alt);
  /// @}

  /**
   * @brief 执行放行的工作 / Run the released work
   * @return 是否执行了 / Whether work ran
   * @note 调用者：DFUClass::Process，应用的一个上下文；可能阻塞（擦写 Flash）。
   *       Caller: DFUClass::Process, one application context; may block (flash
   *       erase and program).
   */
  bool RunPendingWork();

  /// @name 任何上下文 / Any context
  /// @{

  /// 请求启动应用；有未完成工作或没有镜像时拒绝 / Request a launch; refused while work
  /// is pending or no image exists
  bool TryRequestRunApp();
  /// 有启动请求且条件满足时取走请求并跳转 / Take a launch request and jump when allowed
  bool TryConsumeAppLaunch(uint32_t now_ms);
  /// 工作槽是否非空 / Whether the job slot is not empty
  bool HasPendingWork() const;
  /// 是否有通过校验的镜像 / Whether a verified image exists
  bool HasValidImage() const { return ImageSize() != 0U; }
  /// 已校验镜像的字节数，没有为 0 / Bytes of the verified image, 0 without one
  size_t ImageSize() const;
  /// @}

 private:
  /// 下载会话 / Download session
  struct DownloadState
  {
    bool session_started = false;
    size_t received_bytes = 0U;
    uint16_t expected_block_num = 0U;
    uint32_t next_poll_timeout_ms = 0U;
    DFUStatusCode last_status = DFUStatusCode::OK;
  };

  /// 一次写入 / One write step
  struct WriteState
  {
    size_t offset = 0U;
    size_t len = 0U;
    uint16_t block_num = 0U;
    bool pending = false;
  };

  /// manifest 会话 / Manifestation session
  struct ManifestState
  {
    bool pending = false;
    uint32_t poll_timeout_ms = 50U;
    DFUStatusCode last_status = DFUStatusCode::OK;
  };

  /// 上传会话 / Upload session
  struct UploadState
  {
    bool session_started = false;
    size_t offset = 0U;
    uint16_t expected_block_num = 0U;
    size_t image_size = 0U;
  };

  /// 工作种类 / Kind of work
  enum class WorkKind : uint8_t
  {
    WRITE,
    MANIFEST
  };

  /// 一份工作：owner 在 STAGED 之前写，RunPendingWork 在 READY 之后读 / One job:
  /// written by the owner before STAGED, read by RunPendingWork after READY
  struct Work
  {
    WorkKind kind = WorkKind::WRITE;
    WriteState write{};
    size_t image_size = 0U;
  };

  static constexpr uint32_t EMPTY = 0U, STAGED = 1U, READY = 2U, RUNNING = 3U, DONE = 4U;
  static constexpr uint32_t CANCELLED = 0x80000000U;
  static constexpr uint32_t MIN_WRITE_POLL_MS = 10U;     ///< 报告下限 / Lower bound
  static constexpr uint32_t MAX_WRITE_POLL_MS = 1000U;   ///< 报告上限 / Upper bound
  static constexpr uint32_t MIN_REMAINING_POLL_MS = 5U;  ///< 剩余下限 / Remaining floor

  /// 镜像上限：seal 之前的字节数 / Image limit: the bytes before the seal
  size_t PayloadLimit() const;
  /// 块 0：清传输状态和擦除表，旧镜像和启动请求作废 / Block 0: clear the transfer
  /// state and the erase map; the old image and launch request become void
  void StartDownloadSession();
  /// 清下载、写入、manifest、上传四个会话 / Clear the download, write, manifest and
  /// upload sessions
  void ResetTransferState();
  /// 这一块要不要擦除，返回对应的实测耗时 / Whether this block erases; returns the
  /// matching measured duration
  uint32_t ComputeWritePollTimeout(size_t offset, size_t len);
  /// 写入还剩的估计时间 / Estimated time left for the write
  uint32_t RemainingWriteTime() const;
  /// 从 Flash 读回镜像算 CRC32 / Read the image back from flash and compute its CRC32
  bool ComputeImageCrc32(size_t image_size, uint32_t& crc32_out);
  /// 读 seal 记录 / Read the seal record
  bool ReadSeal(SealRecord& seal);
  /// 写 seal 记录 / Write the seal record
  bool WriteSeal(size_t image_size, uint32_t crc32);
  /// 检查 Flash 里的 seal 和 CRC / Check the seal and CRC in flash
  bool ProbeStoredImage(size_t* out_image_size);
  /// 擦除区间覆盖的块，每会话每块一次 / Erase the blocks of a range, once per block
  /// and session
  bool EnsureBlocksErased(size_t offset, size_t len);
  /// 作废工作槽里的工作 / Void the work in the job slot
  void CancelWork();
  /// 每次 Flash 调用之前：工作没被作废 / Before each flash call: the work is not void
  bool BeginFlashStep();

  /// @name 构造时固定 / Fixed at construction
  /// @{
  Flash& flash_;                     ///< 镜像所在 Flash / Flash of the image
  size_t image_offset_;              ///< 镜像区起点 / Image region start
  size_t image_size_limit_;          ///< 镜像区字节数 / Image region bytes
  size_t seal_offset_;               ///< seal 偏移 / Seal offset
  JumpFun jump_to_app_;              ///< 跳转函数 / Jump function
  void* jump_app_ctx_;               ///< 跳转参数 / Jump argument
  bool autorun_;                     ///< manifest 后自动启动 / Launch after manifestation
  size_t erase_block_size_ = 1U;     ///< 擦除块大小 / Erase block size
  size_t erase_block_count_ = 0U;    ///< 擦除块个数 / Erase block count
  size_t seal_storage_size_ = 0U;    ///< seal 按写入粒度对齐的长度 / Aligned seal size
  uint8_t* seal_storage_ = nullptr;  ///< seal 暂存，工作中写 / Seal scratch, work only
  size_t transfer_size_ = 0U;        ///< 一块的上限 / Largest block
  /// 暂存的块；owner 在 STAGED 之前写，工作中读 / The staged block; written by the
  /// owner before STAGED, read by the work
  uint8_t* write_buffer_ = nullptr;
  /// @}

  /// @name 仅 owner / Owner only
  /// @{
  DownloadState download_{};
  WriteState write_{};
  ManifestState manifest_{};
  UploadState upload_{};
  Work job_{};
  /// @}

  /// @name owner 与 RunPendingWork 之间，按 job_state_ 交接 / Between the owner and
  ///       RunPendingWork, handed over through job_state_
  /// @{
  std::atomic<uint32_t> job_state_{EMPTY};
  uint32_t job_started_ms_ = 0U;    ///< 当前步骤开始时刻 / Start of the current step
  bool write_needs_erase_ = false;  ///< 当前写入要擦除 / The current write erases
  uint32_t measured_erase_write_ms_ = 25U;  ///< 上次擦除+编程 / Last erase+program
  uint32_t measured_write_ms_ = 10U;        ///< 上次只编程 / Last program only
  DFUStatusCode job_result_ = DFUStatusCode::OK;
  uint8_t* erased_blocks_ = nullptr;  ///< 每块是否已擦 / Per-block erase marks
  uint8_t crc_buffer_[256] = {};      ///< CRC 分块缓冲 / CRC chunk buffer
  /// @}

  /// @name 任何上下文 / Any context
  /// @{
  /**
   * @brief 已校验镜像的字节数，0 为没有 / Bytes of the verified image, 0 for none
   * @note owner 写（新会话清零、manifest 成功写入）；任何上下文读。
   *       Written by the owner (cleared by a new session, set by a successful
   *       manifestation); read anywhere.
   */
  std::atomic<size_t> published_image_size_{0U};
  /**
   * @brief 有启动请求 / A launch is requested
   * @note 置位：RUN_APP 完成、autorun 的 WAIT_RESET 确认（owner）或 TryRequestRunApp；
   *       清零：新的下载会话（owner）；TryConsumeAppLaunch 取走。
   *       Set by a completed RUN_APP, an acknowledged autorun WAIT_RESET (owner) or
   *       TryRequestRunApp; cleared by a new download session (owner); taken by
   *       TryConsumeAppLaunch.
   */
  std::atomic<uint32_t> launch_request_{0U};
  /// @}
};

/**
 * @class DFUClass
 * @brief DFU 模式的前端：标准 DFU 状态机，后端只管存储 / DFU-mode frontend: the
 *        standard DFU state machine, the backend only handles storage
 *
 * 只用 EP0。DNLOAD 的数据由核心收齐后经 OnControlData 交给后端暂存；GETSTATUS 的状态
 * 阶段完成（OnControlComplete）才放行后端的擦写，擦写在应用调用 Process() 的上下文里
 * 执行，结果经 RequestService 由 owner 在 OnService 中取回。
 * EP0 only. The core receives the DNLOAD data in full and OnControlData hands it to the
 * backend to stage; only the completed status stage of GETSTATUS (OnControlComplete)
 * releases the backend's flash work, which runs in the context that calls Process(),
 * and the owner collects the result in OnService after RequestService.
 *
 * @tparam Backend 后端，需提供 DfuBootloaderBackend 的"前端调用"组和 RunPendingWork /
 *         The backend, providing the "called by the frontend" group of
 *         DfuBootloaderBackend and RunPendingWork
 * @tparam MAX_TRANSFER_SIZE wTransferSize 上限，也是 UPLOAD 缓冲和控制 OUT 缓冲的大小
 *         / Upper bound of wTransferSize, also the size of the UPLOAD buffer and of the
 *         control OUT buffer
 * @note 除 Process() 外只在设备的 owner 中运行。
 *       Apart from Process(), runs in the device's owner only.
 */
template <typename Backend, size_t MAX_TRANSFER_SIZE = 4096U>
class DFUClass : public DfuInterfaceClassBase
{
  static_assert(MAX_TRANSFER_SIZE > 0U, "DFU transfer size must be non-zero.");
  static_assert(MAX_TRANSFER_SIZE <= 0xFFFFU, "DFU transfer size must fit in 16 bits.");

 public:
  /// 默认接口字符串 / Default interface string
  static constexpr const char* DEFAULT_INTERFACE_STRING = "XRUSB DFU";

  /**
   * @brief 构造 / Construct
   * @param backend 后端，须已构造 / The backend, already constructed
   * @param interface_string 接口字符串 / Interface string
   * @param webusb_landing_page_url WebUSB 落地页；nullptr 不启用 / WebUSB landing page;
   *        nullptr disables it
   * @param webusb_vendor_code WebUSB 厂商请求码 / WebUSB vendor request code
   * @param winusb_device_interface_guid WinUSB GUID；nullptr 或空串不请求绑定 / WinUSB
   *        GUID; nullptr or "" requests no binding
   * @note 读一次后端能力，写功能描述符。
   *       Reads the backend's capabilities once and writes the functional descriptor.
   */
  explicit DFUClass(
      Backend& backend, const char* interface_string = DEFAULT_INTERFACE_STRING,
      const char* webusb_landing_page_url = nullptr,
      uint8_t webusb_vendor_code = WebUsb::WEBUSB_VENDOR_CODE_DEFAULT,
      const char* winusb_device_interface_guid = DEFAULT_WINUSB_DEVICE_INTERFACE_GUID)
      : DfuInterfaceClassBase(interface_string, PROTOCOL_DFU_MODE,
                              webusb_landing_page_url, webusb_vendor_code,
                              winusb_device_interface_guid),
        backend_(backend),
        caps_(ClampCapabilities(backend.GetDfuCapabilities()))
  {
    functional_.bmAttributes = static_cast<uint8_t>(
        (caps_.can_download ? ATTR_CAN_DOWNLOAD : 0U) |
        (caps_.can_upload ? ATTR_CAN_UPLOAD : 0U) |
        (caps_.manifestation_tolerant ? ATTR_MANIFESTATION_TOLERANT : 0U) |
        (caps_.will_detach ? ATTR_WILL_DETACH : 0U));
    functional_.wDetachTimeOut = caps_.detach_timeout_ms;
    functional_.wTransferSize = caps_.transfer_size;
  }

  /**
   * @brief 执行后端放行的擦写或校验 / Run the erase/program or check the backend
   *        released
   * @note 应用主循环周期调用，一个上下文，可能阻塞。执行了就 RequestService，owner 在
   *       OnService 里取回结果，下一个 GETSTATUS 看到它。
   *       Called periodically by the application loop, from one context; may block. When
   *       work ran it calls RequestService; the owner collects the result in OnService
   *       and the next GETSTATUS sees it.
   */
  void Process()
  {
    if (backend_.RunPendingWork())
    {
      RequestService(false);
    }
  }

 protected:
  /// @name DeviceClass 的实现 / DeviceClass implementation
  /// @{

  /// 记下接口号，协议状态回到 DFU_IDLE，后端选 alt 0 / Record the interface number,
  /// protocol state back to DFU_IDLE, the backend selects alt 0
  [[nodiscard]] ErrorCode BindEndpoints(EndpointPool& pool, uint8_t first_interface,
                                        bool in_isr) override
  {
    UNUSED(pool);
    UNUSED(in_isr);
    interface_ = first_interface;
    ClearErrorState();
    return backend_.DfuSetAlternate(ALT_SETTING);
  }

  /// 后端放弃会话（作废工作），协议状态回到 DFU_IDLE / The backend drops the session
  /// (voids the work), protocol state back to DFU_IDLE
  void UnbindEndpoints(EndpointPool& pool, bool in_isr) override
  {
    UNUSED(pool);
    UNUSED(in_isr);
    backend_.DfuAbort(ALT_SETTING);
    ClearErrorState();
  }

  /// 单功能设备：设备类改为 0xFE/1/2 / Single-function device: device class 0xFE/1/2
  ErrorCode WriteDeviceDescriptor(DeviceDescriptor& descriptor) override
  {
    descriptor.data_.bDeviceClass = DeviceDescriptor::ClassID::APPLICATION_SPECIFIC;
    descriptor.data_.bDeviceSubClass = 0x01U;
    descriptor.data_.bDeviceProtocol = PROTOCOL_DFU_MODE;
    return ErrorCode::OK;
  }

  /// 控制 OUT 缓冲至少 MAX_TRANSFER_SIZE / Control OUT buffer of at least
  /// MAX_TRANSFER_SIZE
  size_t GetControlReceiveCapacity() const override { return MAX_TRANSFER_SIZE; }

  /**
   * @brief SET_INTERFACE / SET_INTERFACE
   * @return OK：alt 0，放弃会话、清错误；NOT_SUPPORT：alt 非 0；BUSY：DNBUSY、MANIFEST、
   *         MANIFEST-WAIT-RESET 中。
   *         OK: alt 0, the session is dropped and the error cleared; NOT_SUPPORT: a
   *         non-zero alt; BUSY: in DNBUSY, MANIFEST or MANIFEST-WAIT-RESET.
   */
  ErrorCode SetAltSetting(bool in_isr, uint8_t itf, uint8_t alt) override
  {
    UNUSED(in_isr);
    UNUSED(itf);
    if (alt != ALT_SETTING)
    {
      return ErrorCode::NOT_SUPPORT;
    }
    if (state_ == DFUState::DFU_DNBUSY || state_ == DFUState::DFU_MANIFEST ||
        state_ == DFUState::DFU_MANIFEST_WAIT_RESET)
    {
      return ErrorCode::BUSY;
    }
    const ErrorCode result = backend_.DfuSetAlternate(alt);
    if (result != ErrorCode::OK)
    {
      return result;
    }
    backend_.DfuAbort(ALT_SETTING);
    ClearErrorState();
    return ErrorCode::OK;
  }

  /**
   * @brief DFU 类请求的 SETUP 阶段 / SETUP stage of a DFU class request
   * @return 形状不对（见 CheckClassRequest）时 NOT_SUPPORT 或 ARG_ERR，不改状态。
   *         DNBUSY、MANIFEST 中只有 GETSTATUS 合法，其余请求 STALL 并进入 DFU_ERROR；
   *         GETSTATUS 不因时间原因被
   * STALL，已完成就前进，仍忙就继续报当前状态与剩余等待。 协议上不允许的请求进入
   * DFU_ERROR 并返回 ARG_ERR（核心 STALL）。 NOT_SUPPORT or ARG_ERR for a wrong shape
   * (see CheckClassRequest), without a state change. In DNBUSY and MANIFEST only
   * GETSTATUS is valid and any other request stalls and enters DFU_ERROR; GETSTATUS is
   * never stalled for timing, advances when the work is done, and otherwise repeats the
   * busy state with the remaining timeout. A request the protocol does not allow enters
   * DFU_ERROR and returns ARG_ERR (the core stalls).
   * @note in_data 指向 status_response_、state_response_ 或 transfer_buffer_，它们只在
   *       下一个同类请求时改写。
   *       in_data points to status_response_, state_response_ or transfer_buffer_, which
   *       change only at the next request of the same kind.
   */
  ErrorCode OnControlRequest(bool in_isr, const SetupPacket& setup,
                             ConstRawData& in_data) override
  {
    UNUSED(in_isr);
    const ErrorCode check = CheckClassRequest(setup);
    if (check != ErrorCode::OK)
    {
      return check;
    }
    const auto request = static_cast<DFURequest>(setup.bRequest);
    if ((state_ == DFUState::DFU_DNBUSY || state_ == DFUState::DFU_MANIFEST) &&
        request != DFURequest::GETSTATUS)
    {
      // DFU 1.1 A.2.5：设备忙时只有 GETSTATUS 有意义，其余请求 STALL。GETSTATUS 从不
      // 因时间原因被 STALL：bwPollTimeout 只是主机至少要等多久，设备忙时靠 NAK 控制
      // 流量；步骤是否完成由后端的忙状态决定（见 HandleGetStatus）。
      // DFU 1.1 A.2.5: while the device is busy only GETSTATUS makes sense, other
      // requests are stalled. GETSTATUS is never stalled for timing: bwPollTimeout is
      // only the minimum host wait, and NAK is the flow control while memory is being
      // updated. Whether the step is done is decided by the backend's busy state (see
      // HandleGetStatus).
      return ProtocolStall(DFUStatusCode::ERR_STALLEDPKT);
    }
    switch (request)
    {
      case DFURequest::DNLOAD:
        return HandleDnload(setup);
      case DFURequest::UPLOAD:
        return HandleUpload(setup, in_data);
      case DFURequest::GETSTATUS:
        return HandleGetStatus(in_data);
      case DFURequest::CLRSTATUS:
        if (state_ != DFUState::DFU_ERROR)
        {
          return ProtocolStall(DFUStatusCode::ERR_STALLEDPKT);
        }
        backend_.DfuClearStatus(ALT_SETTING);
        ClearErrorState();
        return ErrorCode::OK;
      case DFURequest::GETSTATE:
        state_response_ = static_cast<uint8_t>(state_);
        in_data = {&state_response_, sizeof(state_response_)};
        return ErrorCode::OK;
      case DFURequest::ABORT:
        return HandleAbort();
      case DFURequest::DETACH:  // 属于运行时类 / Belongs to the runtime class
      default:
        return ProtocolStall(DFUStatusCode::ERR_STALLEDPKT);
    }
  }

  /**
   * @brief DNLOAD 的数据阶段收齐 / The DNLOAD data stage arrived in full
   * @return OK：块交给了后端（后端拒绝时进入 DFU_ERROR，仍返回 OK，主机从 GETSTATUS
   *         看到）；ARG_ERR：收到的字节数不等于 wLength（短包），进入 DFU_ERROR。
   *         OK: the block went to the backend (a backend refusal enters DFU_ERROR and
   *         still returns OK; the host sees it in GETSTATUS); ARG_ERR: the bytes
   *         received differ from wLength (a short packet), DFU_ERROR is entered.
   * @note 块号就是 setup.wValue，长度就是 setup.wLength，类不另存。
   *       The block number is setup.wValue and the length setup.wLength; the class
   *       stores neither.
   */
  ErrorCode OnControlData(bool in_isr, const SetupPacket& setup,
                          ConstRawData data) override
  {
    UNUSED(in_isr);
    if (setup.bRequest != static_cast<uint8_t>(DFURequest::DNLOAD))
    {
      return ErrorCode::NOT_SUPPORT;
    }
    // 短包意味着块被截断，按整块写会把垃圾写进 Flash。
    // A short packet means a truncated block; writing it as a whole block would put
    // garbage into the flash.
    if (data.size_ != setup.wLength)
    {
      EnterErrorState(DFUStatusCode::ERR_STALLEDPKT);
      return ErrorCode::ARG_ERR;
    }
    uint32_t poll_timeout_ms = 0U;
    const DFUStatusCode status =
        backend_.DfuDownload(ALT_SETTING, setup.wValue, data, poll_timeout_ms);
    poll_timeout_ms_ = poll_timeout_ms;
    if (status != DFUStatusCode::OK)
    {
      EnterErrorState(status);
      return ErrorCode::OK;
    }
    state_ = DFUState::DFU_DNLOAD_SYNC;
    status_ = DFUStatusCode::OK;
    return ErrorCode::OK;
  }

  /**
   * @brief 状态阶段完成 / A status stage completed
   * @note 只看 GETSTATUS：应答为 DNBUSY 或 MANIFEST 时放行后端工作；为
   *       MANIFEST-WAIT-RESET 时让后端登记 autorun。应答被主机确认之前不放行，主机没
   *       收到的应答不触发任何工作。
   *       Only GETSTATUS matters: an answer of DNBUSY or MANIFEST releases the backend
   *       work; MANIFEST-WAIT-RESET lets the backend record autorun. Nothing is released
   *       before the host acknowledged the answer, so an answer the host never got
   *       starts no work.
   */
  void OnControlComplete(bool in_isr, const SetupPacket& setup) override
  {
    UNUSED(in_isr);
    if ((setup.bmRequestType & REQ_TYPE_MASK) !=
            static_cast<uint8_t>(RequestType::CLASS) ||
        setup.bRequest != static_cast<uint8_t>(DFURequest::GETSTATUS))
    {
      return;
    }
    if (state_ == DFUState::DFU_DNBUSY || state_ == DFUState::DFU_MANIFEST)
    {
      backend_.AllowPendingWork();
    }
    else if (state_ == DFUState::DFU_MANIFEST_WAIT_RESET)
    {
      backend_.DfuCommitManifestWaitReset(ALT_SETTING);
    }
  }

  /**
   * @brief 被接受的传输没有完成 / An accepted transfer did not complete
   * @note 只看 DNLOAD：后端放弃会话（暂存的块作废）；已进入 DFU_ERROR 的（数据阶段
   *       出错后的 STALL）保留错误给 GETSTATUS，否则回到 DFU_IDLE。
   *       Only DNLOAD matters: the backend drops the session (a staged block is voided);
   *       a transfer that already entered DFU_ERROR (the stall after a bad data stage)
   *       keeps the error for GETSTATUS, otherwise the state returns to DFU_IDLE.
   */
  void OnControlAbort(bool in_isr, const SetupPacket& setup) override
  {
    UNUSED(in_isr);
    if ((setup.bmRequestType & REQ_TYPE_MASK) !=
            static_cast<uint8_t>(RequestType::CLASS) ||
        setup.bRequest != static_cast<uint8_t>(DFURequest::DNLOAD))
    {
      return;
    }
    backend_.DfuAbort(ALT_SETTING);
    if (state_ != DFUState::DFU_ERROR)
    {
      ClearErrorState();
    }
  }

  /// 取回后端完成的工作 / Collect the backend's finished work
  void OnService(bool in_isr) override
  {
    UNUSED(in_isr);
    backend_.CommitWorkResult();
  }
  /// @}

 private:
  /// 本类只支持的备用设置 / The only alternate setting this class supports
  static constexpr uint8_t ALT_SETTING = 0U;
  static constexpr uint8_t PROTOCOL_DFU_MODE = 0x02U;
  static constexpr uint8_t ATTR_CAN_DOWNLOAD = 0x01U;
  static constexpr uint8_t ATTR_CAN_UPLOAD = 0x02U;
  static constexpr uint8_t ATTR_MANIFESTATION_TOLERANT = 0x04U;
  static constexpr uint8_t ATTR_WILL_DETACH = 0x08U;

  /// wTransferSize 为 0 或超过 MAX_TRANSFER_SIZE 时取 MAX_TRANSFER_SIZE / wTransferSize
  /// becomes MAX_TRANSFER_SIZE when 0 or above it
  static DFUCapabilities ClampCapabilities(DFUCapabilities caps)
  {
    if (caps.transfer_size == 0U || caps.transfer_size > MAX_TRANSFER_SIZE)
    {
      caps.transfer_size = static_cast<uint16_t>(MAX_TRANSFER_SIZE);
    }
    return caps;
  }

  ErrorCode HandleDnload(const SetupPacket& setup)
  {
    if (!caps_.can_download)
    {
      return ProtocolStall(DFUStatusCode::ERR_STALLEDPKT);
    }
    if (state_ != DFUState::DFU_IDLE && state_ != DFUState::DFU_DNLOAD_IDLE)
    {
      return ProtocolStall(DFUStatusCode::ERR_STALLEDPKT);
    }
    if (setup.wLength > caps_.transfer_size)
    {
      return ProtocolStall(DFUStatusCode::ERR_USBR);
    }
    if (setup.wLength != 0U)
    {
      // 核心收齐数据阶段后调用 OnControlData。
      // The core receives the data stage, then calls OnControlData.
      return ErrorCode::OK;
    }
    // 零长度 DNLOAD 结束下载、请求 manifest；只在已经收过块（DNLOAD-IDLE）时合法。
    // A zero-length DNLOAD ends the download and asks for manifestation; legal only
    // after blocks were taken (DNLOAD-IDLE).
    if (state_ != DFUState::DFU_DNLOAD_IDLE)
    {
      // DFU 1.1 A.2.3：dfuIDLE 里只有 wLength > 0 的 DNLOAD 是合法转移；零长度 DNLOAD
      // 没有可结束的下载，控制管道 STALL，进入 dfuERROR（errSTALLEDPKT）。
      // DFU 1.1 A.2.3: in dfuIDLE only a DNLOAD with wLength > 0 is a valid transition;
      // a zero-length DNLOAD has no download to end, so the control pipe is stalled and
      // the device enters dfuERROR (errSTALLEDPKT).
      return ProtocolStall(DFUStatusCode::ERR_STALLEDPKT);
    }
    state_ = DFUState::DFU_MANIFEST_SYNC;
    status_ = DFUStatusCode::OK;
    poll_timeout_ms_ = 0U;
    return ErrorCode::OK;
  }

  ErrorCode HandleUpload(const SetupPacket& setup, ConstRawData& in_data)
  {
    if (!caps_.can_upload)
    {
      return ProtocolStall(DFUStatusCode::ERR_STALLEDPKT);
    }
    if (state_ != DFUState::DFU_IDLE && state_ != DFUState::DFU_UPLOAD_IDLE)
    {
      return ProtocolStall(DFUStatusCode::ERR_STALLEDPKT);
    }
    if (setup.wLength == 0U)
    {
      return ProtocolStall(DFUStatusCode::ERR_USBR);
    }
    const size_t want =
        (setup.wLength > caps_.transfer_size) ? caps_.transfer_size : setup.wLength;
    DFUStatusCode status = DFUStatusCode::OK;
    uint32_t poll_timeout_ms = 0U;
    const size_t got = backend_.DfuUpload(
        ALT_SETTING, setup.wValue, {transfer_buffer_, want}, status, poll_timeout_ms);
    poll_timeout_ms_ = poll_timeout_ms;
    if (status != DFUStatusCode::OK)
    {
      return ProtocolStall(status);
    }
    if (got > want)
    {
      return ProtocolStall(DFUStatusCode::ERR_USBR);
    }
    // 短块表示上传结束 / A short block ends the upload
    status_ = DFUStatusCode::OK;
    state_ = (got < want) ? DFUState::DFU_IDLE : DFUState::DFU_UPLOAD_IDLE;
    in_data = {transfer_buffer_, got};
    return ErrorCode::OK;
  }

  ErrorCode HandleGetStatus(ConstRawData& in_data)
  {
    // 在 DNBUSY / MANIFEST 中：步骤已完成则前进，否则仍答 busy 并给出剩余时间。
    // In DNBUSY / MANIFEST: advance if the step is done, otherwise answer busy again with
    // the time left.
    AdvanceStateForStatusRead();
    status_response_.bStatus = static_cast<uint8_t>(status_);
    status_response_.SetPollTimeout(poll_timeout_ms_);
    status_response_.bState = static_cast<uint8_t>(state_);
    status_response_.iString = 0U;
    in_data = {&status_response_, sizeof(status_response_)};
    return ErrorCode::OK;
  }

  ErrorCode HandleAbort()
  {
    switch (state_)
    {
      case DFUState::DFU_IDLE:
      case DFUState::DFU_DNLOAD_SYNC:
      case DFUState::DFU_DNLOAD_IDLE:
      case DFUState::DFU_UPLOAD_IDLE:
      case DFUState::DFU_MANIFEST_SYNC:
        backend_.DfuAbort(ALT_SETTING);
        ClearErrorState();
        return ErrorCode::OK;
      default:
        return ProtocolStall(DFUStatusCode::ERR_STALLEDPKT);
    }
  }

  /// GETSTATUS 是推进状态机的同步点 / GETSTATUS is the point that advances the state
  /// machine
  void AdvanceStateForStatusRead()
  {
    switch (state_)
    {
      case DFUState::DFU_DNLOAD_SYNC:
      case DFUState::DFU_DNBUSY:
        RefreshDownloadStatus();
        break;
      case DFUState::DFU_MANIFEST_SYNC:
        if (manifest_started_)
        {
          RefreshManifestStatus();
        }
        else
        {
          StartManifest();
        }
        break;
      case DFUState::DFU_MANIFEST:
        RefreshManifestStatus();
        break;
      default:
        break;
    }
  }

  void StartManifest()
  {
    if (status_ != DFUStatusCode::OK)
    {
      state_ = DFUState::DFU_ERROR;
      return;
    }
    uint32_t poll_timeout_ms = 0U;
    const DFUStatusCode status = backend_.DfuManifest(ALT_SETTING, poll_timeout_ms);
    poll_timeout_ms_ = poll_timeout_ms;
    if (status != DFUStatusCode::OK)
    {
      EnterErrorState(status);
      return;
    }
    status_ = DFUStatusCode::OK;
    state_ = DFUState::DFU_MANIFEST;
    manifest_started_ = true;
  }

  void RefreshDownloadStatus()
  {
    if (status_ != DFUStatusCode::OK)
    {
      state_ = DFUState::DFU_ERROR;
      return;
    }
    bool busy = false;
    uint32_t poll_timeout_ms = 0U;
    const DFUStatusCode status =
        backend_.DfuGetDownloadStatus(ALT_SETTING, busy, poll_timeout_ms);
    poll_timeout_ms_ = poll_timeout_ms;
    if (status != DFUStatusCode::OK)
    {
      EnterErrorState(status);
      return;
    }
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
    uint32_t poll_timeout_ms = 0U;
    const DFUStatusCode status =
        backend_.DfuGetManifestStatus(ALT_SETTING, busy, poll_timeout_ms);
    poll_timeout_ms_ = poll_timeout_ms;
    if (status != DFUStatusCode::OK)
    {
      EnterErrorState(status);
      return;
    }
    if (busy)
    {
      state_ = DFUState::DFU_MANIFEST;
      return;
    }
    // 完成后：能容忍 manifest 的回 IDLE，否则等复位。
    // Done: back to IDLE when manifestation tolerant, otherwise wait for a reset.
    manifest_started_ = false;
    state_ = caps_.manifestation_tolerant ? DFUState::DFU_IDLE
                                          : DFUState::DFU_MANIFEST_WAIT_RESET;
  }

  void ClearErrorState()
  {
    manifest_started_ = false;
    status_ = DFUStatusCode::OK;
    state_ = DFUState::DFU_IDLE;
    poll_timeout_ms_ = 0U;
  }

  /// 锁存错误，直到 CLRSTATUS 或 ABORT / Latch an error until CLRSTATUS or ABORT
  void EnterErrorState(DFUStatusCode status)
  {
    status_ = status;
    state_ = DFUState::DFU_ERROR;
    poll_timeout_ms_ = 0U;
  }

  ErrorCode ProtocolStall(DFUStatusCode status)
  {
    EnterErrorState(status);
    return ErrorCode::ARG_ERR;
  }

  /// 后端；构造时固定 / The backend; fixed at construction
  Backend& backend_;
  /// 能力，wTransferSize 已截到 MAX_TRANSFER_SIZE；构造时固定 / Capabilities with
  /// wTransferSize clipped to MAX_TRANSFER_SIZE; fixed at construction
  const DFUCapabilities caps_;

  /// @name 仅 owner / Owner only
  /// @{
  DFUState state_ = DFUState::DFU_IDLE;       ///< 协议状态 / Protocol state
  DFUStatusCode status_ = DFUStatusCode::OK;  ///< 锁存的状态码 / Latched status code
  /// 最近一次报告的 bwPollTimeout / Latest reported bwPollTimeout
  uint32_t poll_timeout_ms_ = 0U;
  /// MANIFEST-SYNC 是 manifest 之后的那一次（不再启动 manifest）/ This MANIFEST-SYNC
  /// follows manifestation (do not start it again)
  bool manifest_started_ = false;
  DFUStatusResponse status_response_{};  ///< GETSTATUS 应答 / GETSTATUS answer
  uint8_t state_response_ = 0U;          ///< GETSTATE 应答 / GETSTATE answer
  /// UPLOAD 应答 / UPLOAD answer
  uint8_t transfer_buffer_[MAX_TRANSFER_SIZE] = {};
  /// @}
};

/// 先于 DFUClass 构造后端的存储基类 / Storage base that constructs the backend before
/// DFUClass
class DfuBootloaderClassStorage
{
 protected:
  /// 参数同 DfuBootloaderBackend / Parameters as for DfuBootloaderBackend
  DfuBootloaderClassStorage(Flash& flash, size_t image_base, size_t image_limit,
                            size_t seal_offset, DfuBootloaderBackend::JumpFun jump_to_app,
                            void* jump_app_ctx, bool autorun)
      : storage_backend_(flash, image_base, image_limit, seal_offset, jump_to_app,
                         jump_app_ctx, autorun),
        image_base_(image_base),
        image_limit_(image_limit),
        seal_offset_(seal_offset)
  {
  }

  DfuBootloaderBackend storage_backend_;  ///< Flash 后端 / Flash backend
  const size_t image_base_;               ///< 镜像区起点 / Image region start
  const size_t image_limit_;              ///< 镜像区字节数 / Image region bytes
  const size_t seal_offset_;              ///< seal 偏移 / Seal offset
};

/**
 * @class DfuBootloaderClassT
 * @brief 单镜像区的 bootloader DFU 类 / Bootloader DFU class for one image region
 *
 * 另有一个厂商请求 RUN_APP（0x5A，接收者为本接口，无数据阶段，wValue 为 0）：状态阶段
 * 完成后登记启动请求，由应用调用 TryConsumeAppLaunch 执行。
 * Adds the vendor request RUN_APP (0x5A, recipient this interface, no data stage,
 * wValue 0): once its status stage completes a launch is recorded, which the
 * application carries out by calling TryConsumeAppLaunch.
 */
template <size_t MAX_TRANSFER_SIZE = 4096U>
class DfuBootloaderClassT : private DfuBootloaderClassStorage,
                            public DFUClass<DfuBootloaderBackend, MAX_TRANSFER_SIZE>
{
  using Storage = DfuBootloaderClassStorage;
  using Base = DFUClass<DfuBootloaderBackend, MAX_TRANSFER_SIZE>;

 public:
  /// 跳到应用的函数 / Function that jumps to the application
  using JumpFun = DfuBootloaderBackend::JumpFun;
  /// 厂商请求码 RUN_APP / Vendor request code RUN_APP
  static constexpr uint8_t VENDOR_REQUEST_RUN_APP = 0x5AU;

  /**
   * @brief 构造 / Construct
   * @param flash 放镜像的 Flash / Flash that stores the image
   * @param image_base 镜像区起点 / Start of the image region
   * @param image_limit 镜像区字节数，含 seal / Bytes of the image region, seal included
   * @param seal_offset seal 相对镜像区起点的偏移 / Seal offset from the image start
   * @param jump_to_app 启动应用的函数 / Function that launches the application
   * @param jump_app_ctx 它的参数 / Its argument
   * @param autorun manifest 完成后是否自动请求启动 / Whether manifestation requests a
   *        launch by itself
   * @param interface_string 接口字符串 / Interface string
   * @param webusb_landing_page_url WebUSB 落地页；nullptr 不启用 / WebUSB landing page;
   *        nullptr disables it
   * @param webusb_vendor_code WebUSB 厂商请求码 / WebUSB vendor request code
   * @param winusb_device_interface_guid WinUSB GUID
   */
  DfuBootloaderClassT(Flash& flash, size_t image_base, size_t image_limit,
                      size_t seal_offset, JumpFun jump_to_app,
                      void* jump_app_ctx = nullptr, bool autorun = true,
                      const char* interface_string = Base::DEFAULT_INTERFACE_STRING,
                      const char* webusb_landing_page_url = nullptr,
                      uint8_t webusb_vendor_code = WebUsb::WEBUSB_VENDOR_CODE_DEFAULT,
                      const char* winusb_device_interface_guid =
                          DfuInterfaceClassBase::DEFAULT_WINUSB_DEVICE_INTERFACE_GUID)
      : Storage(flash, image_base, image_limit, seal_offset, jump_to_app, jump_app_ctx,
                autorun),
        Base(Storage::storage_backend_, interface_string, webusb_landing_page_url,
             webusb_vendor_code, winusb_device_interface_guid)
  {
  }

  using Base::Process;

  /// 请求启动应用；任何上下文 / Request an application launch; any context
  bool RequestRunApp() { return Storage::storage_backend_.TryRequestRunApp(); }
  /// 有启动请求且允许时跳转并返回 true；任何上下文 / Jump and return true when a launch
  /// is requested and allowed; any context
  bool TryConsumeAppLaunch(uint32_t now_ms)
  {
    return Storage::storage_backend_.TryConsumeAppLaunch(now_ms);
  }
  /// 是否有未完成的工作 / Whether work is pending
  bool HasPendingWork() const { return Storage::storage_backend_.HasPendingWork(); }
  /// 是否有校验过的镜像 / Whether a verified image exists
  bool HasValidImage() const { return Storage::storage_backend_.HasValidImage(); }
  /// 镜像字节数，没有为 0 / Image bytes, 0 without one
  size_t ImageSize() const { return Storage::storage_backend_.ImageSize(); }
  /// 镜像区起点 / Image region start
  size_t ImageBase() const { return Storage::image_base_; }
  /// 镜像区字节数 / Image region bytes
  size_t ImageLimit() const { return Storage::image_limit_; }
  /// seal 偏移 / Seal offset
  size_t SealOffset() const { return Storage::seal_offset_; }

 protected:
  /**
   * @brief 处理 RUN_APP，其他请求交给 DFU 前端 / Handle RUN_APP and pass every other
   *        request to the DFU frontend
   * @note RUN_APP 的 wLength、wValue 为 0；有工作时 BUSY，没有镜像时 FAILED。
   *       RUN_APP has wLength and wValue 0; BUSY while work is pending, FAILED without
   *       an image.
   */
  ErrorCode OnControlRequest(bool in_isr, const SetupPacket& setup,
                             ConstRawData& in_data) override
  {
    if (!IsRunApp(setup))
    {
      return Base::OnControlRequest(in_isr, setup, in_data);
    }
    if (setup.wLength != 0U || setup.wValue != 0U || setup.wIndex != this->interface_)
    {
      return ErrorCode::ARG_ERR;
    }
    if (Storage::storage_backend_.HasPendingWork())
    {
      return ErrorCode::BUSY;
    }
    if (!Storage::storage_backend_.HasValidImage())
    {
      return ErrorCode::FAILED;
    }
    return ErrorCode::OK;
  }

  /// RUN_APP 的状态阶段完成后登记启动 / Record the launch once the RUN_APP status
  /// stage completed
  void OnControlComplete(bool in_isr, const SetupPacket& setup) override
  {
    Base::OnControlComplete(in_isr, setup);
    if (IsRunApp(setup))
    {
      (void)Storage::storage_backend_.TryRequestRunApp();
    }
  }

 private:
  static bool IsRunApp(const SetupPacket& setup)
  {
    return (setup.bmRequestType & REQ_TYPE_MASK) ==
               static_cast<uint8_t>(RequestType::VENDOR) &&
           setup.bRequest == VENDOR_REQUEST_RUN_APP;
  }
};

/// 4096 字节传输的 bootloader DFU 类 / Bootloader DFU class with 4096-byte transfers
using DfuBootloaderClass = DfuBootloaderClassT<4096U>;

}  // namespace LibXR::USB
