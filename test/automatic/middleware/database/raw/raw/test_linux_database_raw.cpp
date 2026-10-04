/**
 * @file test_linux_database_raw.cpp
 * @brief DatabaseRaw 存储与恢复测试 / DatabaseRaw storage and recovery tests.
 *
 * 借助文件模拟 Flash，检查多键更新、重新打开、大小不匹配、损坏恢复和致命 I/O 失败，
 * 以及在每一次 Flash 操作处掉电之后的值。
 * Use file-backed Flash to check key updates, reopening, size mismatches, recovery and
 * fatal I/O failures, and the values after a power cut at every Flash operation.
 */

#include <memory>

#include "middleware/database/linux_database_test_common.hpp"
#include "test_assert.hpp"

namespace
{

using namespace LinuxDatabaseTestCommon;

void TestLinuxDatabaseRawSmoke()
{
  constexpr size_t FLASH_SIZE = XR_DB_FLASH_SIZE;

  std::array<uint32_t, 1> data_k1 = {1};
  std::array<uint32_t, 2> data_k2 = {11, 22};
  std::array<uint32_t, 3> data_k3 = {111, 222, 333};
  std::array<uint32_t, 4> data_k4 = {1111, 2222, 3333, 4444};

  LinuxBinaryFileFlash<FLASH_SIZE> flash_2("/tmp/flash_test_2.bin", 512, 16, false, true);
  DatabaseRaw<16> test_db_2(flash_2, 5);

  DatabaseRaw<16>::Key k1_2(test_db_2, "key1", data_k1);
  DatabaseRaw<16>::Key k2_2(test_db_2, "keasdasy2", data_k2);
  DatabaseRaw<16>::Key k3_2(test_db_2, "keaasdasdy3", data_k3);
  DatabaseRaw<16>::Key k4_2(test_db_2, "keyaskdhasjh4", data_k4);

  data_k4[1] = 1234567;

  k1_2 = data_k1;
  k2_2 = data_k2;
  k3_2 = data_k3;
  k4_2 = data_k4;

  k1_2.Load();
  k2_2.Load();
  k3_2.Load();
  k4_2.Load();

  TEST_ASSERT(std::memcmp(&data_k1[0], &k1_2.data_[0], sizeof(data_k1)) == 0);
  TEST_ASSERT(std::memcmp(&data_k2[0], &k2_2.data_[0], sizeof(data_k2)) == 0);
  TEST_ASSERT(std::memcmp(&data_k3[0], &k3_2.data_[0], sizeof(data_k3)) == 0);
  TEST_ASSERT(std::memcmp(&data_k4[0], &k4_2.data_[0], sizeof(data_k4)) == 0);

  for (size_t i = 0; i < 1000; i++)
  {
    for (uint32_t j = 0; j < Thread::GetTime() % 100; j++)
    {
      data_k1[0] = Thread::GetTime() + j;
      k1_2 = data_k1;
    }
    for (uint32_t j = 0; j < Thread::GetTime() % 100; j++)
    {
      data_k2[0] = Thread::GetTime() + j;
      k2_2 = data_k2;
    }
    for (uint32_t j = 0; j < Thread::GetTime() % 100; j++)
    {
      data_k3[0] = Thread::GetTime() + j;
      k3_2 = data_k3;
    }
    for (uint32_t j = 0; j < Thread::GetTime() % 100; j++)
    {
      data_k4[0] = Thread::GetTime() + j;
      k4_2 = data_k4;
    }

    k1_2.Load();
    k2_2.Load();
    k3_2.Load();
    k4_2.Load();
    TEST_ASSERT(std::memcmp(&data_k1[0], &k1_2.data_[0], sizeof(data_k1)) == 0);
    TEST_ASSERT(std::memcmp(&data_k2[0], &k2_2.data_[0], sizeof(data_k2)) == 0);
    TEST_ASSERT(std::memcmp(&data_k3[0], &k3_2.data_[0], sizeof(data_k3)) == 0);
    TEST_ASSERT(std::memcmp(&data_k4[0], &k4_2.data_[0], sizeof(data_k4)) == 0);
  }
}

void TestDatabasePartialBackupRecovery()
{
  RunPartialBackupCase("/tmp/flash_test_partial_valid_main.bin", MainChecksum::VALID, 0,
                       1234);
  RunPartialBackupCase("/tmp/flash_test_partial_broken_main.bin", MainChecksum::INVALID,
                       55, 55);
}

void TestDatabaseRawSaveCurrentValue()
{
  const char* path = "/tmp/flash_test_raw_save_current.bin";
  LinuxBinaryFileFlash<XR_DB_FLASH_SIZE> flash(path, XR_DB_MIN_ERASE_SIZE,
                                               XR_DB_MIN_WRITE_SIZE, false, true);
  DatabaseRaw<16> db(flash, 5);
  db.Restore();

  DatabaseRaw<16>::Key<uint32_t> key(db, "raw", 1);
  key.data_ = 2;
  TEST_ASSERT(key.Save() == ErrorCode::OK);
  TEST_ASSERT(ReopenDatabaseValue(path, 0, "raw") == 2);
}

void TestDatabaseRawRequiresExactStoredSize()
{
  // 按一种大小保存，再用另一种大小打开同名键，检查不把不同大小的数据直接当成同一类型。
  // Store one size and reopen the same key with another; do not treat differently sized
  // data as the same type.
  const char* path = "/tmp/flash_test_raw_exact_size.bin";
  {
    LinuxBinaryFileFlash<XR_DB_FLASH_SIZE> flash(path, XR_DB_MIN_ERASE_SIZE,
                                                 XR_DB_MIN_WRITE_SIZE, false, true);
    DatabaseRaw<16> db(flash, 5);
    db.Restore();
    DatabaseRaw<16>::Key<uint32_t> key(db, "shape", 0x11223344U);
    TEST_ASSERT(key.data_ == 0x11223344U);
  }

  {
    LinuxBinaryFileFlash<XR_DB_FLASH_SIZE> flash(path, XR_DB_MIN_ERASE_SIZE,
                                                 XR_DB_MIN_WRITE_SIZE, false, true);
    DatabaseRaw<16> db(flash, 5);
    DatabaseRaw<16>::Key<uint64_t> wider_key(db, "shape", 0ULL);
    TEST_ASSERT(wider_key.data_ == 0ULL);
    TEST_ASSERT(wider_key.Load() == ErrorCode::FAILED);
  }

  TEST_ASSERT(ReopenDatabaseValue(path, 0, "shape") == 0x11223344U);
}

}  // namespace

void RunLinuxDatabaseRawSmokeTests()
{
  TestLinuxDatabaseRawSmoke();
  TestDatabasePartialBackupRecovery();
  TestDatabaseRawSaveCurrentValue();
  TestDatabaseRawRequiresExactStoredSize();
}

namespace
{

using namespace LinuxDatabaseTestCommon;

void TestDatabaseKeyAddFailureRequires()
{
  ExpectFatalExit(XR_DB_FATAL_KEY_ADD,
                  []
                  {
                    class MemoryDatabase : public Database
                    {
                     public:
                      ErrorCode get_result = ErrorCode::NOT_FOUND;
                      ErrorCode add_result = ErrorCode::FAILED;

                      ErrorCode Get(KeyBase&) override { return get_result; }
                      ErrorCode Set(KeyBase&, RawData) override { return ErrorCode::OK; }
                      ErrorCode Add(KeyBase&) override { return add_result; }
                    } db;

                    Database::Key<uint32_t> key(db, "mock", 123);
                    UNUSED(key);
                  });
}

void TestDatabaseRawReadFailureRequires()
{
  ExpectFatalExit(XR_DB_FATAL_RAW_READ,
                  []
                  {
                    FailingFlash flash(XR_DB_MIN_ERASE_SIZE, XR_DB_MIN_WRITE_SIZE);
                    flash.SetFailOp(FailingFlash::FailOp::READ);
                    DatabaseRaw<XR_DB_MIN_WRITE_SIZE> db(flash, 5);
                    UNUSED(db);
                  });
}

void TestDatabaseRawWriteFailureRequires()
{
  ExpectFatalExit(XR_DB_FATAL_RAW_WRITE,
                  []
                  {
                    FailingFlash flash(XR_DB_MIN_ERASE_SIZE, XR_DB_MIN_WRITE_SIZE);
                    flash.SetFailOp(FailingFlash::FailOp::WRITE);
                    DatabaseRaw<XR_DB_MIN_WRITE_SIZE> db(flash, 5);
                    UNUSED(db);
                  });
}

void TestDatabaseRawEraseFailureRequires()
{
  ExpectFatalExit(XR_DB_FATAL_RAW_ERASE,
                  []
                  {
                    FailingFlash flash(XR_DB_MIN_ERASE_SIZE, XR_DB_MIN_WRITE_SIZE);
                    flash.SetFailOp(FailingFlash::FailOp::ERASE);
                    DatabaseRaw<XR_DB_MIN_WRITE_SIZE> db(flash, 5);
                    UNUSED(db);
                  });
}

}  // namespace

void RunLinuxDatabaseRawFailureTests()
{
  TestDatabaseKeyAddFailureRequires();
  TestDatabaseRawReadFailureRequires();
  TestDatabaseRawWriteFailureRequires();
  TestDatabaseRawEraseFailureRequires();
}

namespace
{

using namespace LinuxDatabaseTestCommon;

void TestDatabaseRawUncommittedKeyIsDropped()
{
  // 把首个键改回未提交状态后重新打开：只丢弃这个键，后面的键保持原值。
  // Reopen after turning the first key back into an uncommitted one: only that key is
  // dropped, and the key after it keeps its value.
  const char* path = "/tmp/flash_test_raw_invalid_main_key_metadata.bin";
  CreateTwoKeyDatabase(path);

  auto bytes = ReadAllBytes(path);
  MarkMainFirstKeyAsUninitialized(bytes);
  WriteAllBytes(path, bytes);

  TEST_ASSERT(ReopenDatabaseValue(path, 77, "key1") == 77);
  TEST_ASSERT(ReopenDatabaseValue(path, 88, "key2") == 2222);
  auto repaired = ReadAllBytes(path);
  TEST_ASSERT(ReadLe32(repaired, 0) == XR_DB_FLASH_HEADER);
  TEST_ASSERT(ReadLe32(repaired, XR_DB_CHECKSUM_OFFSET) == XR_DB_CHECKSUM);
}

void TestDatabaseRawInvalidBackupMetadataDoesNotRestore()
{
  // 备份元数据损坏时，即使主区需要恢复，也不能采用这份备份。
  // A backup with corrupt metadata must not be used even when the main block needs
  // recovery.
  const char* path = "/tmp/flash_test_raw_invalid_backup_metadata.bin";
  CreateSeedDatabase(path);

  auto bytes = ReadAllBytes(path);
  MirrorMainBlockToBackup(bytes);
  CorruptBackupFirstKeyAvailableFlag(bytes);
  InvalidateMainChecksum(bytes);
  WriteAllBytes(path, bytes);

  TEST_ASSERT(ReopenDatabaseValue(path, 55) == 55);
  AssertMainValidBackupNotRecoverable(path);
}

void TestDatabaseRawRestoresFromValidBackup()
{
  // 主区校验损坏但备份完整时，应恢复原来的值，而不是使用调用者的默认值。
  // When the main checksum is corrupt but the backup is valid, restore the stored value
  // rather than the default.
  const char* path = "/tmp/flash_test_raw_restore_from_valid_backup.bin";
  CreateSeedDatabase(path);

  auto bytes = ReadAllBytes(path);
  MirrorMainBlockToBackup(bytes);
  InvalidateMainChecksum(bytes);
  WriteAllBytes(path, bytes);

  TEST_ASSERT(ReopenDatabaseValue(path, 55) == 1234);
  AssertMainValidBackupNotRecoverable(path);
}

void TestDatabaseRawKeepsValidMainOverCutBackupCopy()
{
  // 备份块是旧版本回收中途留下的拷贝，链表恰好完整：主块有效时保留主块。
  // The backup is a copy cut in the middle of an older-version recycle whose chain
  // happens to be complete: a valid main block is kept.
  const char* path = "/tmp/flash_test_raw_cut_backup_copy.bin";
  CreateTwoKeyDatabase(path);

  auto bytes = ReadAllBytes(path);
  CraftCutCopyOfSecondKey(bytes);
  WriteAllBytes(path, bytes);

  TEST_ASSERT(ReopenDatabaseValue(path, 77, "key1") == 1111);
  TEST_ASSERT(ReopenDatabaseValue(path, 88, "key2") == 2222);
  AssertMainValidBackupNotRecoverable(path);
}

void TestDatabaseRawFullBlockSetReturnsFull()
{
  // 一个键占满主块时，整理也腾不出空间：更新返回 FULL，原值不变。
  // When one key fills the main block, compacting frees nothing: the update returns
  // FULL and the stored value stays.
  struct Big
  {
    uint8_t bytes[XR_DB_CHECKSUM_OFFSET - XR_DB_RAW_FIRST_KEY_OFFSET -
                  XR_DB_RAW_KEYINFO_ALIGNED_SIZE - XR_DB_MIN_WRITE_SIZE];
  };
  const char* path = "/tmp/flash_test_raw_full_block.bin";
  Big first{};
  std::memset(first.bytes, 0x11, sizeof(first.bytes));
  Big second{};
  std::memset(second.bytes, 0x22, sizeof(second.bytes));
  {
    LinuxBinaryFileFlash<XR_DB_FLASH_SIZE> flash(path, XR_DB_MIN_ERASE_SIZE,
                                                 XR_DB_MIN_WRITE_SIZE, false, true);
    DatabaseRaw<16> db(flash, 5);
    db.Restore();
    DatabaseRaw<16>::Key<Big> key(db, "big", first);
    TEST_ASSERT(key.Set(second) == ErrorCode::FULL);
  }
  LinuxBinaryFileFlash<XR_DB_FLASH_SIZE> flash(path, XR_DB_MIN_ERASE_SIZE,
                                               XR_DB_MIN_WRITE_SIZE, false, true);
  DatabaseRaw<16> db(flash, 5);
  DatabaseRaw<16>::Key<Big> key(db, "big", second);
  TEST_ASSERT(std::memcmp(key.data_.bytes, first.bytes, sizeof(first.bytes)) == 0);
}

void TestDatabaseRawCorruptFirstKeySizeInMultiKeyDatabaseReinitializes()
{
  // 把首个键的长度改成越界值，检查重新初始化后的两个键都使用默认值。
  // Make the first key length exceed the block; after reinitialization both keys must use
  // their defaults.
  const char* path = "/tmp/flash_test_raw_corrupt_first_key_size.bin";
  CreateTwoKeyDatabase(path);

  auto bytes = ReadAllBytes(path);
  CorruptMainFirstKeyRawInfo(bytes, 0x7FFFFFFFU);
  WriteAllBytes(path, bytes);

  TEST_ASSERT(ReopenDatabaseValue(path, 77, "key1") == 77);
  TEST_ASSERT(ReopenDatabaseValue(path, 88, "key2") == 88);
  auto repaired = ReadAllBytes(path);
  TEST_ASSERT(ReadLe32(repaired, 0) == XR_DB_FLASH_HEADER);
  TEST_ASSERT(ReadLe32(repaired, XR_DB_CHECKSUM_OFFSET) == XR_DB_CHECKSUM);
}

}  // namespace

void RunLinuxDatabaseRawRecoveryTests()
{
  TestDatabaseRawUncommittedKeyIsDropped();
  TestDatabaseRawInvalidBackupMetadataDoesNotRestore();
  TestDatabaseRawRestoresFromValidBackup();
  TestDatabaseRawKeepsValidMainOverCutBackupCopy();
  TestDatabaseRawFullBlockSetReturnsFull();
  TestDatabaseRawCorruptFirstKeySizeInMultiKeyDatabaseReinitializes();
}

namespace
{

using namespace LinuxDatabaseTestCommon;

void TestDatabaseRawPowerCutDoubleWord()
{
  // 8 字节写入单元（G4 等带 ECC 的双字编程）：每次 Flash 操作处掉电（包括编程和擦除
  // 被打断在中间，擦除时 1/16 或 1/512 的位回到 1），键保持旧值或新值。
  // 8-byte write unit (double-word programming with ECC, as on G4): a power cut at any
  // Flash operation, including a program or erase cut part-way (with 1/16 or 1/512 of
  // the bits back to 1), leaves each key at its old or new value.
  RunPowerCutCases([](Flash& flash)
                   { return std::make_unique<DatabaseRaw<8>>(flash, 3); },
                   XR_DB_MIN_ERASE_SIZE, 8, 4 + (3 * 24), 4);
  RunPowerCutCases([](Flash& flash)
                   { return std::make_unique<DatabaseRaw<8>>(flash, 3); },
                   XR_DB_MIN_ERASE_SIZE, 8, 4 + (3 * 24), 9);
}

void TestDatabaseRawPowerCutFullBlock()
{
  // 阈值很大时只有块写满才回收：覆盖新增、更新在空间不足时触发的回收。
  // With a large threshold a recycle happens only when the block is full: this covers
  // the recycles triggered by an add or update that does not fit.
  RunPowerCutCases([](Flash& flash)
                   { return std::make_unique<DatabaseRaw<8>>(flash, 1000); },
                   XR_DB_MIN_ERASE_SIZE, 8, 4 + (3 * 30), 4);
}

void TestDatabaseRawPowerCutByte()
{
  // 1 字节写入单元（F4 等按字节编程）。 / 1-byte write unit (byte programming, as on F4).
  RunPowerCutCases([](Flash& flash)
                   { return std::make_unique<DatabaseRaw<1>>(flash, 3); },
                   XR_DB_MIN_ERASE_SIZE, 1, 4 + (3 * 10), 9);
}

}  // namespace

void RunLinuxDatabaseRawPowerCutTests()
{
  TestDatabaseRawPowerCutDoubleWord();
  TestDatabaseRawPowerCutFullBlock();
  TestDatabaseRawPowerCutByte();
}

void test_linux_database_raw()
{
  RunLinuxDatabaseRawSmokeTests();
  RunLinuxDatabaseRawFailureTests();
  RunLinuxDatabaseRawRecoveryTests();
  RunLinuxDatabaseRawPowerCutTests();
}
