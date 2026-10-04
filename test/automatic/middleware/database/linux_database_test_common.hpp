/**
 * @file linux_database_test_common.hpp
 * @brief 数据库测试辅助代码 / Database test helpers.
 *
 * 提供 Flash 故障注入、文件内容损坏与重读、预期致命退出的检查，以及在任意一次 Flash
 * 操作处掉电的模拟。
 * Provide Flash fault injection, file corruption and reopening, expected fatal-exit
 * checks, and power loss at any single Flash operation.
 */

#pragma once

#include <sys/mman.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <array>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <limits>
#include <new>
#include <vector>

#include "database.hpp"
#include "libxr_def.hpp"
#include "linux_flash.hpp"
#include "test.hpp"
#include "test_assert.hpp"

namespace LinuxDatabaseTestCommon
{

using namespace LibXR;

constexpr size_t XR_DB_FLASH_SIZE = 4096;
constexpr size_t XR_DB_MIN_ERASE_SIZE = 512;
constexpr size_t XR_DB_MIN_WRITE_SIZE = 16;
constexpr size_t XR_DB_BLOCK_SIZE = XR_DB_FLASH_SIZE / 2;
constexpr size_t XR_DB_CHECKSUM_OFFSET = XR_DB_BLOCK_SIZE - XR_DB_MIN_WRITE_SIZE;
constexpr uint32_t XR_DB_FLASH_HEADER = 0x12345678 + LIBXR_DATABASE_VERSION;
constexpr uint32_t XR_DB_CHECKSUM = 0x9abcedf0;
constexpr uint8_t XR_DB_SEQ_CHECKSUM = 0x56;
constexpr size_t XR_DB_RAW_KEYINFO_ALIGNED_SIZE = XR_DB_MIN_WRITE_SIZE * 4;
constexpr size_t XR_DB_RAW_SENTINEL_KEY_OFFSET = XR_DB_MIN_WRITE_SIZE;
constexpr size_t XR_DB_RAW_FIRST_KEY_OFFSET =
    XR_DB_RAW_SENTINEL_KEY_OFFSET + XR_DB_RAW_KEYINFO_ALIGNED_SIZE;
constexpr size_t XR_DB_RAW_AVAILABLE_FLAG_OFFSET =
    XR_DB_RAW_FIRST_KEY_OFFSET + XR_DB_MIN_WRITE_SIZE;
constexpr size_t XR_DB_RAW_UNINIT_FLAG_LAST_BYTE_OFFSET =
    XR_DB_RAW_FIRST_KEY_OFFSET + XR_DB_MIN_WRITE_SIZE * 3 - 1;
constexpr size_t XR_DB_RAW_FIRST_KEY_RAW_INFO_OFFSET =
    XR_DB_RAW_FIRST_KEY_OFFSET + XR_DB_MIN_WRITE_SIZE * 3;
constexpr int XR_DB_FATAL_KEY_ADD = 97;
constexpr int XR_DB_FATAL_SEQ_READ = 91;
constexpr int XR_DB_FATAL_SEQ_WRITE = 92;
constexpr int XR_DB_FATAL_SEQ_ERASE = 93;
constexpr int XR_DB_FATAL_RAW_READ = 94;
constexpr int XR_DB_FATAL_RAW_WRITE = 95;
constexpr int XR_DB_FATAL_RAW_ERASE = 96;

enum class MainChecksum
{
  VALID,
  INVALID,
};

class FailingFlash : public Flash
{
 public:
  enum class FailOp
  {
    NONE,
    READ,
    WRITE,
    ERASE,
  };

  explicit FailingFlash(size_t min_erase_size = 512, size_t min_write_size = 8,
                        size_t sequential_buffer_size = 256)
      : Flash(min_erase_size, min_write_size,
              RawData(flash_area_.data(), flash_area_.size())),
        sequential_buffer_size_(sequential_buffer_size)
  {
    SeedValidSequentialBlocks();
  }

  void SetFailOp(FailOp op) { fail_op_ = op; }

  ErrorCode Erase(size_t offset, size_t size) override
  {
    if (fail_op_ == FailOp::ERASE)
    {
      return ErrorCode::FAILED;
    }
    TEST_ASSERT(offset + size <= flash_area_.size());
    std::memset(flash_area_.data() + offset, 0xFF, size);
    return ErrorCode::OK;
  }

  ErrorCode Write(size_t offset, ConstRawData data) override
  {
    if (fail_op_ == FailOp::WRITE)
    {
      return ErrorCode::FAILED;
    }
    TEST_ASSERT(offset + data.size_ <= flash_area_.size());
    std::memcpy(flash_area_.data() + offset, data.addr_, data.size_);
    return ErrorCode::OK;
  }

  ErrorCode Read(size_t offset, RawData data) override
  {
    if (fail_op_ == FailOp::READ)
    {
      return ErrorCode::FAILED;
    }
    return Flash::Read(offset, data);
  }

 private:
  void SeedValidSequentialBlocks()
  {
    std::memset(flash_area_.data(), 0xFF, flash_area_.size());

    const size_t block_size = flash_area_.size() / 2;
    const uint32_t empty_key = 0;

    std::memcpy(flash_area_.data(), &XR_DB_FLASH_HEADER, sizeof(XR_DB_FLASH_HEADER));
    std::memcpy(flash_area_.data() + sizeof(XR_DB_FLASH_HEADER), &empty_key,
                sizeof(empty_key));
    flash_area_[sequential_buffer_size_ - 1] = XR_DB_SEQ_CHECKSUM;

    std::memcpy(flash_area_.data() + block_size, &XR_DB_FLASH_HEADER,
                sizeof(XR_DB_FLASH_HEADER));
    std::memcpy(flash_area_.data() + block_size + sizeof(XR_DB_FLASH_HEADER), &empty_key,
                sizeof(empty_key));
    flash_area_[block_size + sequential_buffer_size_ - 1] = XR_DB_SEQ_CHECKSUM;
  }

  std::array<uint8_t, XR_DB_FLASH_SIZE> flash_area_{};
  size_t sequential_buffer_size_ = 0;
  FailOp fail_op_ = FailOp::NONE;
};

template <typename Func>
void ExpectFatalExit(int exit_code, Func&& func)
{
  // 单线程测试进程 fork 后安装退出回调；必须以指定错误码退出才算命中预期失败。
  // Fork from a single-threaded case and install an exit callback; only the expected
  // fatal code passes.
  pid_t child = fork();
  TEST_ASSERT(child >= 0);

  if (child == 0)
  {
    auto cb = LibXR::Assert::FatalCallback::Create(
        [](bool in_isr, int code, const char*, uint32_t)
        {
          UNUSED(in_isr);
          _exit(code);
        },
        exit_code);
    LibXR::Assert::RegisterFatalErrorCallback(cb);
    func();
    _exit(0);
  }

  int status = 0;
  TEST_ASSERT(waitpid(child, &status, 0) == child);
  TEST_ASSERT(WIFEXITED(status));
  TEST_ASSERT(WEXITSTATUS(status) == exit_code);
}

[[nodiscard]] inline uint32_t ReadLe32(const std::vector<uint8_t>& bytes, size_t offset)
{
  return static_cast<uint32_t>(bytes[offset]) |
         (static_cast<uint32_t>(bytes[offset + 1]) << 8) |
         (static_cast<uint32_t>(bytes[offset + 2]) << 16) |
         (static_cast<uint32_t>(bytes[offset + 3]) << 24);
}

inline void WriteLe32(std::vector<uint8_t>& bytes, size_t offset, uint32_t value)
{
  bytes[offset] = static_cast<uint8_t>(value & 0xFF);
  bytes[offset + 1] = static_cast<uint8_t>((value >> 8) & 0xFF);
  bytes[offset + 2] = static_cast<uint8_t>((value >> 16) & 0xFF);
  bytes[offset + 3] = static_cast<uint8_t>((value >> 24) & 0xFF);
}

[[nodiscard]] inline std::vector<uint8_t> ReadAllBytes(const char* path)
{
  std::ifstream file(path, std::ios::binary);
  TEST_ASSERT(static_cast<bool>(file));

  std::vector<uint8_t> bytes(XR_DB_FLASH_SIZE, 0);
  file.read(reinterpret_cast<char*>(bytes.data()),
            static_cast<std::streamsize>(bytes.size()));
  TEST_ASSERT(file.gcount() == static_cast<std::streamsize>(bytes.size()));
  return bytes;
}

inline void WriteAllBytes(const char* path, const std::vector<uint8_t>& bytes)
{
  std::ofstream file(path, std::ios::binary | std::ios::trunc);
  TEST_ASSERT(static_cast<bool>(file));
  file.write(reinterpret_cast<const char*>(bytes.data()),
             static_cast<std::streamsize>(bytes.size()));
  TEST_ASSERT(static_cast<bool>(file));
}

inline void CraftPartialBackup(std::vector<uint8_t>& bytes, size_t partial_len)
{
  TEST_ASSERT(partial_len < XR_DB_BLOCK_SIZE);

  const size_t backup_offset = XR_DB_BLOCK_SIZE;
  for (size_t i = 0; i < partial_len; ++i)
  {
    bytes[backup_offset + i] = bytes[i];
  }

  WriteLe32(bytes, backup_offset + XR_DB_CHECKSUM_OFFSET, XR_DB_CHECKSUM);
}

inline void MirrorMainBlockToBackup(std::vector<uint8_t>& bytes)
{
  for (size_t i = 0; i < XR_DB_BLOCK_SIZE; ++i)
  {
    bytes[XR_DB_BLOCK_SIZE + i] = bytes[i];
  }
}

inline void CraftCutCopyOfSecondKey(std::vector<uint8_t>& bytes)
{
  // 旧版本回收在写完第二个键的键头之后被打断：备份块校验已写，链表停在第二个键，名字和
  // 数据仍是擦除态。
  // An older-version recycle cut right after the header of the second key: the backup
  // checksum is written, the chain stops at the second key, and its name and payload
  // are still erased.
  MirrorMainBlockToBackup(bytes);
  constexpr size_t SECOND_KEY_OFFSET = XR_DB_RAW_FIRST_KEY_OFFSET +
                                       XR_DB_RAW_KEYINFO_ALIGNED_SIZE +
                                       (2 * XR_DB_MIN_WRITE_SIZE);
  constexpr size_t SECOND_KEY_END =
      SECOND_KEY_OFFSET + XR_DB_RAW_KEYINFO_ALIGNED_SIZE + (2 * XR_DB_MIN_WRITE_SIZE);
  for (size_t i = SECOND_KEY_OFFSET + XR_DB_RAW_KEYINFO_ALIGNED_SIZE; i < SECOND_KEY_END;
       ++i)
  {
    bytes[XR_DB_BLOCK_SIZE + i] = 0xFF;
  }
}

inline void InvalidateMainChecksum(std::vector<uint8_t>& bytes)
{
  WriteLe32(bytes, XR_DB_CHECKSUM_OFFSET, 0);
}

inline void MarkMainFirstKeyAsUninitialized(std::vector<uint8_t>& bytes)
{
  bytes[XR_DB_RAW_UNINIT_FLAG_LAST_BYTE_OFFSET] = 0xFF;
}

inline void CorruptBackupFirstKeyAvailableFlag(std::vector<uint8_t>& bytes)
{
  bytes[XR_DB_BLOCK_SIZE + XR_DB_RAW_AVAILABLE_FLAG_OFFSET] = 0x00;
}

inline void CorruptMainFirstKeyRawInfo(std::vector<uint8_t>& bytes, uint32_t raw_info)
{
  WriteLe32(bytes, XR_DB_RAW_FIRST_KEY_RAW_INFO_OFFSET, raw_info);
}

inline void CreateSeedDatabase(const char* path)
{
  LinuxBinaryFileFlash<XR_DB_FLASH_SIZE> flash(path, XR_DB_MIN_ERASE_SIZE,
                                               XR_DB_MIN_WRITE_SIZE, false, true);
  DatabaseRaw<16> db(flash, 5);
  db.Restore();
  DatabaseRaw<16>::Key<uint32_t> key(db, "key", 1234);
  TEST_ASSERT(key.data_ == 1234);
}

inline void CreateTwoKeyDatabase(const char* path)
{
  LinuxBinaryFileFlash<XR_DB_FLASH_SIZE> flash(path, XR_DB_MIN_ERASE_SIZE,
                                               XR_DB_MIN_WRITE_SIZE, false, true);
  DatabaseRaw<16> db(flash, 5);
  db.Restore();
  DatabaseRaw<16>::Key<uint32_t> key1(db, "key1", 1111);
  DatabaseRaw<16>::Key<uint32_t> key2(db, "key2", 2222);
  TEST_ASSERT(key1.data_ == 1111);
  TEST_ASSERT(key2.data_ == 2222);
}

[[nodiscard]] inline uint32_t ReopenDatabaseValue(const char* path,
                                                  uint32_t default_value)
{
  LinuxBinaryFileFlash<XR_DB_FLASH_SIZE> flash(path, XR_DB_MIN_ERASE_SIZE,
                                               XR_DB_MIN_WRITE_SIZE, false, true);
  DatabaseRaw<16> db(flash, 5);
  DatabaseRaw<16>::Key<uint32_t> key(db, "key", default_value);
  return key.data_;
}

[[nodiscard]] inline uint32_t ReopenDatabaseValue(const char* path,
                                                  uint32_t default_value,
                                                  const char* key_name)
{
  LinuxBinaryFileFlash<XR_DB_FLASH_SIZE> flash(path, XR_DB_MIN_ERASE_SIZE,
                                               XR_DB_MIN_WRITE_SIZE, false, true);
  DatabaseRaw<16> db(flash, 5);
  DatabaseRaw<16>::Key<uint32_t> key(db, key_name, default_value);
  return key.data_;
}

[[nodiscard]] inline uint32_t ReopenSequentialDatabaseValue(const char* path,
                                                            uint32_t default_value,
                                                            const char* key_name)
{
  LinuxBinaryFileFlash<XR_DB_FLASH_SIZE> flash(path, 512, 8, true, true);
  DatabaseRawSequential db(flash);
  DatabaseRawSequential::Key<uint32_t> key(db, key_name, default_value);
  return key.data_;
}

inline void AssertMainValidBackupNotRecoverable(const char* path)
{
  // 主块有效；备份块要么校验无效，要么是空块，都不会再被当作恢复来源。
  // The main block is valid; the backup is either invalid or empty, so it is no longer a
  // recovery source.
  auto bytes = ReadAllBytes(path);
  TEST_ASSERT(ReadLe32(bytes, 0) == XR_DB_FLASH_HEADER);
  TEST_ASSERT(ReadLe32(bytes, XR_DB_CHECKSUM_OFFSET) == XR_DB_CHECKSUM);
  const bool backup_valid =
      ReadLe32(bytes, XR_DB_BLOCK_SIZE) == XR_DB_FLASH_HEADER &&
      ReadLe32(bytes, XR_DB_BLOCK_SIZE + XR_DB_CHECKSUM_OFFSET) == XR_DB_CHECKSUM;
  bool backup_empty = true;
  for (size_t i = 0; i < XR_DB_MIN_WRITE_SIZE; ++i)
  {
    backup_empty =
        backup_empty && bytes[XR_DB_BLOCK_SIZE + XR_DB_RAW_SENTINEL_KEY_OFFSET +
                              XR_DB_MIN_WRITE_SIZE + i] == 0xFF;
  }
  TEST_ASSERT(!backup_valid || backup_empty);
}

inline void RunPartialBackupCase(const char* path, MainChecksum main_checksum,
                                 uint32_t default_value, uint32_t expected_value)
{
  // 模拟备份只写了一部分：主区有效时保留原值，否则使用默认值；不采纳半份备份。
  // Simulate a partial backup: keep a valid main value or use the default, never restore
  // the incomplete backup.
  CreateSeedDatabase(path);

  auto bytes = ReadAllBytes(path);
  CraftPartialBackup(bytes, 128);
  if (main_checksum == MainChecksum::INVALID)
  {
    InvalidateMainChecksum(bytes);
  }
  WriteAllBytes(path, bytes);

  TEST_ASSERT(ReopenDatabaseValue(path, default_value) == expected_value);
  AssertMainValidBackupNotRecoverable(path);
}

// ---- 掉电模拟 / Power-loss simulation ----

constexpr int XR_DB_POWER_CUT_SCRIPT_DONE = 3;

/**
 * @brief 运行脚本的子进程与检查结果的父进程共享的状态
 *        State shared by the child process that runs a script and the parent that
 *        checks the result.
 */
struct PowerCutShared
{
  std::array<uint8_t, XR_DB_FLASH_SIZE> image{};
  /// 已完成的单元编程和页擦除次数 / Unit programs and page erases done
  size_t operations = 0;
  /// 掉电前允许的操作数 / Operations allowed before the power cut
  size_t budget = std::numeric_limits<size_t>::max();
  /// 掉电落在下一次操作中间 / The cut lands inside the next operation
  bool torn = false;
  /// 擦除被打断时每一位回到 1 的概率为 1/2^n；0 表示擦除不会被打断在中间
  /// Each bit returns to 1 with probability 1/2^n when an erase is cut; 0 means an
  /// erase is never cut part-way
  int torn_erase_rounds = 0;
  /// 对既非擦除态也不相同的单元的编程次数 / Programs of units that were neither erased
  /// nor equal
  size_t violations = 0;
  /// 掉电时正在执行的脚本步骤 / Script step running at the cut
  int step = -1;
};

[[nodiscard]] inline PowerCutShared* CreatePowerCutShared()
{
  void* memory = mmap(nullptr, sizeof(PowerCutShared), PROT_READ | PROT_WRITE,
                      MAP_SHARED | MAP_ANONYMOUS, -1, 0);
  TEST_ASSERT(memory != MAP_FAILED);
  return new (memory) PowerCutShared();
}

/**
 * @brief 在第 `budget` 次操作处掉电的 Flash
 *        Flash that loses power at operation number `budget`.
 *
 * 每编程一个最小写入单元、每擦除一页算一次操作；与 Flash 内容相同的单元像 STM32Flash
 * 一样跳过。用完预算的进程立即退出；`torn` 时，正在编程的单元只清掉一部分该清的位，
 * `torn_erase_rounds` 不为 0 时，正在擦除的页只有一部分位回到 1。对既非擦除态、也不等于新
 * 内容的单元编程计入 `violations`：带 ECC 的 Flash 做不到。
 * Programming one minimum write unit or erasing one page is one operation; a unit equal
 * to the Flash content is skipped as STM32Flash does. The process exits as soon as the
 * budget is used; with `torn`, the unit being programmed gets only some of the bits it
 * should clear, and with a non-zero `torn_erase_rounds`, the page being erased gets only
 * some of its bits back to 1.
 * Programming a unit that is neither erased nor equal to the new content counts in
 * `violations`: ECC Flash cannot do it.
 */
class PowerCutFlash : public Flash
{
 public:
  PowerCutFlash(PowerCutShared& shared, size_t min_erase_size, size_t min_write_size)
      : Flash(min_erase_size, min_write_size,
              RawData(shared.image.data(), shared.image.size())),
        shared_(shared)
  {
  }

  ErrorCode Erase(size_t offset, size_t size) override
  {
    TEST_ASSERT(offset % MinEraseSize() == 0 && size % MinEraseSize() == 0);
    TEST_ASSERT(offset + size <= shared_.image.size());
    for (size_t page = offset; page < offset + size; page += MinEraseSize())
    {
      if (shared_.operations == shared_.budget)
      {
        if (shared_.torn && shared_.torn_erase_rounds > 0)
        {
          // 只有少量位回到 1，块头或校验往往还读作有效。
          // Only a few bits return to 1, so the header or checksum often still reads as
          // valid.
          uint32_t state = static_cast<uint32_t>(shared_.operations * 2654435761U) | 1U;
          for (size_t i = 0; i < MinEraseSize(); ++i)
          {
            uint8_t mask = 0xFF;
            for (int round = 0; round < shared_.torn_erase_rounds; ++round)
            {
              state ^= state << 13;
              state ^= state >> 17;
              state ^= state << 5;
              mask = static_cast<uint8_t>(mask & state);
            }
            shared_.image[page + i] =
                static_cast<uint8_t>(shared_.image[page + i] | mask);
          }
        }
        _exit(0);
      }
      std::memset(shared_.image.data() + page, 0xFF, MinEraseSize());
      shared_.operations++;
    }
    return ErrorCode::OK;
  }

  ErrorCode Write(size_t offset, ConstRawData data) override
  {
    const size_t unit = MinWriteSize();
    TEST_ASSERT(offset % unit == 0 && data.size_ % unit == 0);
    TEST_ASSERT(offset + data.size_ <= shared_.image.size());
    const auto* src = static_cast<const uint8_t*>(data.addr_);
    for (size_t i = 0; i < data.size_; i += unit)
    {
      uint8_t* dst = shared_.image.data() + offset + i;
      if (std::memcmp(dst, src + i, unit) == 0)
      {
        continue;
      }
      if (shared_.operations == shared_.budget)
      {
        if (shared_.torn)
        {
          for (size_t j = 0; j < unit; ++j)
          {
            const auto mask =
                static_cast<uint8_t>(((shared_.operations * 131) + (j * 29)) ^ 0x5A);
            dst[j] = static_cast<uint8_t>(dst[j] & (src[i + j] | mask));
          }
        }
        _exit(0);
      }
      for (size_t j = 0; j < unit; ++j)
      {
        if (dst[j] != 0xFF)
        {
          shared_.violations++;
          break;
        }
      }
      std::memcpy(dst, src + i, unit);
      shared_.operations++;
    }
    return ErrorCode::OK;
  }

 private:
  PowerCutShared& shared_;
};

/**
 * @brief 掉电脚本用到的 13 字节值，长度不是写入单元的整数倍
 *        13-byte value used by the power-cut script; its size is not a multiple of the
 *        write unit.
 */
struct PowerCutBlob
{
  uint8_t bytes[13];
  bool operator==(const PowerCutBlob&) const = default;
};

/**
 * @brief 脚本执行若干步之后三个键的值 / Values of the three keys after some script steps.
 *
 * 第 0 步打开数据库，第 1–3 步新增 a、b、c，之后每 3 步依次更新 a、b、c。
 * Step 0 opens the database, steps 1–3 add a, b and c, and every 3 steps after that
 * update a, b and c in turn.
 */
struct PowerCutValues
{
  bool has_a = false;
  bool has_b = false;
  bool has_c = false;
  uint32_t a = 0;
  PowerCutBlob b{};
  uint64_t c = 0;
};

/// b 的键名跨过多个写入单元 / The name of b spans several write units.
constexpr const char* POWER_CUT_NAME_B = "blob-thirteen";

[[nodiscard]] inline uint32_t PowerCutA(uint32_t n) { return 0xA0000000U + n; }

[[nodiscard]] inline PowerCutBlob PowerCutB(uint32_t n)
{
  PowerCutBlob blob{};
  for (size_t i = 0; i < sizeof(blob.bytes); ++i)
  {
    blob.bytes[i] = static_cast<uint8_t>((n * 7) + i);
  }
  return blob;
}

[[nodiscard]] inline uint64_t PowerCutC(uint32_t n)
{
  return 0xC000000000000000ULL + (static_cast<uint64_t>(n) * 3);
}

[[nodiscard]] inline PowerCutValues PowerCutValuesAfter(int steps)
{
  PowerCutValues values;
  for (int step = 1; step < steps; ++step)
  {
    const auto n = static_cast<uint32_t>((step - 1) / 3);
    switch ((step - 1) % 3)
    {
      case 0:
        values.has_a = true;
        values.a = PowerCutA(n);
        break;
      case 1:
        values.has_b = true;
        values.b = PowerCutB(n);
        break;
      default:
        values.has_c = true;
        values.c = PowerCutC(n);
        break;
    }
  }
  return values;
}

template <typename OpenDatabase>
void RunPowerCutScript(OpenDatabase& open, Flash& flash, PowerCutShared& shared,
                       int steps)
{
  shared.step = 0;
  auto db = open(flash);
  shared.step = 1;
  Database::Key<uint32_t> a(*db, "a", PowerCutA(0));
  shared.step = 2;
  Database::Key<PowerCutBlob> b(*db, POWER_CUT_NAME_B, PowerCutB(0));
  shared.step = 3;
  Database::Key<uint64_t> c(*db, "c", PowerCutC(0));
  for (int step = 4; step < steps; ++step)
  {
    shared.step = step;
    const auto n = static_cast<uint32_t>((step - 1) / 3);
    switch ((step - 1) % 3)
    {
      case 0:
        TEST_ASSERT(a.Set(PowerCutA(n)) == ErrorCode::OK);
        break;
      case 1:
        TEST_ASSERT(b.Set(PowerCutB(n)) == ErrorCode::OK);
        break;
      default:
        TEST_ASSERT(c.Set(PowerCutC(n)) == ErrorCode::OK);
        break;
    }
  }
}

/**
 * @brief 在脚本的每一次 Flash 操作处掉电，检查重新打开后的值
 *        Cut the power at every Flash operation of the script and check the values
 *        after reopening.
 * @param open 在给定 Flash 上打开数据库并返回指针 / Opens the database on the given
 *        Flash and returns a pointer.
 * @param min_erase_size 擦除页大小 / Erase page size.
 * @param min_write_size 最小写入单元 / Minimum write unit.
 * @param steps 脚本步数 / Number of script steps.
 * @param torn_erase_rounds 擦除被打断时每一位回到 1 的概率为 1/2^n，0 表示不模拟
 *        Each bit returns to 1 with probability 1/2^n when an erase is cut; 0 turns
 *        this off.
 *
 * 被打断的那一步涉及的键必须是旧值或新值（新增被打断时可以不存在），其他键必须保持
 * 原值；重新打开和之后的写入都不能对已编程单元二次编程。
 * The key of the interrupted step must hold its old or new value (or be absent when its
 * add was cut), every other key must keep its value, and neither reopening nor later
 * writes may program a unit twice.
 */
template <typename OpenDatabase>
void RunPowerCutCases(OpenDatabase open, size_t min_erase_size, size_t min_write_size,
                      int steps, int torn_erase_rounds)
{
  constexpr uint32_t MISSING_A = 0x5A5A5A5AU;
  constexpr uint64_t MISSING_C = 0x5A5A5A5A5A5A5A5AULL;
  PowerCutBlob missing_b{};
  std::memset(missing_b.bytes, 0x5A, sizeof(missing_b.bytes));

  PowerCutShared* shared = CreatePowerCutShared();
  size_t cuts = 0;
  bool finished = false;
  for (size_t budget = 0; !finished; ++budget)
  {
    for (bool torn : {false, true})
    {
      shared->image.fill(0xFF);
      shared->operations = 0;
      shared->budget = budget;
      shared->torn = torn;
      shared->torn_erase_rounds = torn_erase_rounds;
      shared->violations = 0;
      shared->step = -1;

      pid_t child = fork();
      TEST_ASSERT(child >= 0);
      if (child == 0)
      {
        PowerCutFlash flash(*shared, min_erase_size, min_write_size);
        RunPowerCutScript(open, flash, *shared, steps);
        _exit(XR_DB_POWER_CUT_SCRIPT_DONE);
      }
      int status = 0;
      TEST_ASSERT(waitpid(child, &status, 0) == child);
      TEST_ASSERT(WIFEXITED(status));
      if (WEXITSTATUS(status) == XR_DB_POWER_CUT_SCRIPT_DONE)
      {
        finished = true;
        break;
      }
      TEST_ASSERT(WEXITSTATUS(status) == 0);
      TEST_ASSERT(shared->violations == 0);
      cuts++;

      const PowerCutValues before = PowerCutValuesAfter(shared->step);
      const PowerCutValues after = PowerCutValuesAfter(shared->step + 1);
      shared->budget = std::numeric_limits<size_t>::max();

      {
        PowerCutFlash flash(*shared, min_erase_size, min_write_size);
        auto db = open(flash);
        Database::Key<uint32_t> a(*db, "a", MISSING_A);
        Database::Key<PowerCutBlob> b(*db, POWER_CUT_NAME_B, missing_b);
        Database::Key<uint64_t> c(*db, "c", MISSING_C);
        const uint32_t a_before = before.has_a ? before.a : MISSING_A;
        const uint32_t a_after = after.has_a ? after.a : MISSING_A;
        const PowerCutBlob b_before = before.has_b ? before.b : missing_b;
        const PowerCutBlob b_after = after.has_b ? after.b : missing_b;
        const uint64_t c_before = before.has_c ? before.c : MISSING_C;
        const uint64_t c_after = after.has_c ? after.c : MISSING_C;
        TEST_ASSERT(a.data_ == a_before || a.data_ == a_after);
        TEST_ASSERT(b.data_ == b_before || b.data_ == b_after);
        TEST_ASSERT(c.data_ == c_before || c.data_ == c_after);

        TEST_ASSERT(a.Set(0x12345678U) == ErrorCode::OK);
        TEST_ASSERT(b.Set(PowerCutB(1000)) == ErrorCode::OK);
        TEST_ASSERT(c.Set(0x1122334455667788ULL) == ErrorCode::OK);
      }
      {
        PowerCutFlash flash(*shared, min_erase_size, min_write_size);
        auto db = open(flash);
        Database::Key<uint32_t> a(*db, "a", MISSING_A);
        Database::Key<PowerCutBlob> b(*db, POWER_CUT_NAME_B, missing_b);
        Database::Key<uint64_t> c(*db, "c", MISSING_C);
        TEST_ASSERT(a.data_ == 0x12345678U);
        TEST_ASSERT(b.data_ == PowerCutB(1000));
        TEST_ASSERT(c.data_ == 0x1122334455667788ULL);
      }
      TEST_ASSERT(shared->violations == 0);
    }
  }
  TEST_ASSERT(cuts > 100);
  munmap(shared, sizeof(PowerCutShared));
}

}  // namespace LinuxDatabaseTestCommon
