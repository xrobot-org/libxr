/**
 * @brief `DatabaseRaw` 的生命周期与主流程片段 / Lifecycle and main-flow fragment of
 *        `DatabaseRaw`
 *
 * @note 这一组函数组成后端真正对外可见的行为：构造时约束检查、启动恢复、运行期
 *       读写、显式还原，以及回收流程。
 *       This group forms the backend's externally visible behavior: contract
 *       checks during construction, startup recovery, runtime read/write,
 *       explicit restore, and recycling flow.
 */

/**
 * @brief 获取数据库中的键值
 *        Retrieve the key's value from the database.
 * @param key 需要获取的键 / Key to retrieve.
 * @return 操作结果，如果找到则返回 `ErrorCode::OK`，否则返回 `ErrorCode::NOT_FOUND`
 *         Operation result, returns `ErrorCode::OK` if found, otherwise
 *         `ErrorCode::NOT_FOUND`.
 */
ErrorCode Get(Database::KeyBase& key) override
{
  auto ans = SearchKey(key.name_);
  if (!ans)
  {
    return ErrorCode::NOT_FOUND;
  }

  KeyInfo key_buffer;
  ReadFlashOrExit(ans, key_buffer);

  if (key.raw_data_.size_ != key_buffer.GetDataSize())
  {
    return ErrorCode::FAILED;
  }

  ReadFlashOrExit(GetKeyData(ans), key.raw_data_);

  return ErrorCode::OK;
}

/**
 * @brief 设置数据库中的键值
 *        Set the key's value in the database.
 * @param key 目标键 / Target key.
 * @param data 需要存储的新值 / New value to store.
 * @return 操作结果 / Operation result.
 */
ErrorCode Set(KeyBase& key, RawData data) override
{
  return SetKey(key.name_, data.addr_, data.size_);
}

/**
 * @brief 添加新键到数据库 / Add a new key to the database.
 * @param key 需要添加的键 / Key to add.
 * @return 操作结果 / Operation result.
 */
ErrorCode Add(KeyBase& key) override
{
  return AddKey(key.name_, key.raw_data_.addr_, key.raw_data_.size_);
}

/**
 * @brief 构造函数，初始化 Flash 存储和缓冲区
 *        Constructor to initialize Flash storage and buffer.
 *
 * @param flash 目标 Flash 存储设备 / Target Flash storage device.
 * @param recycle_threshold 回收阈值 / Recycle threshold.
 */
explicit DatabaseRaw(Flash& flash, size_t recycle_threshold = 128)
    : recycle_threshold_(recycle_threshold), flash_(flash)
{
  ASSERT(flash.MinEraseSize() * 2 <= flash_.Size());
  ASSERT(flash_.MinWriteSize() <= MinWriteSize);
  auto block_num = static_cast<size_t>(flash_.Size() / flash.MinEraseSize());
  block_size_ = block_num / 2 * flash.MinEraseSize();
  Init();
}

/**
 * @brief 初始化数据库存储区，确保主备块正确
 *        Initialize database storage, ensuring main and backup blocks are valid.
 * @note 备份块是一次回收写完的完整拷贝时（见 `IsBackupComplete()`），回收是在重写主块
 *       期间或前后被打断的，主块可能只擦了一部分，因此以备份块为准恢复主块；主块无效时
 *       也接受旧版本留下的完整备份块；否则相信有效的主块，主块无效时初始化为空块。仍有
 *       键的其他备份块重新准备，不再作为恢复来源。最后检查主块链表，修复掉电留下的状态。
 *       When the backup is the complete copy written by a recycle (see
 *       `IsBackupComplete()`), that recycle was cut while, just before, or just
 *       after it rewrote the main block, which may be partly erased, so the main
 *       block is restored from the backup; an invalid main block also accepts a
 *       complete backup left by older versions; otherwise a valid main block is
 *       trusted and an invalid one is initialized empty. Any other backup that
 *       still holds keys is prepared again and is no longer a recovery source.
 *       The main chain is then checked and repaired after a power loss.
 */
void Init()
{
  size_t used_size = 0;
  if (IsBackupComplete(used_size) ||
      (!IsBlockValid(BlockType::MAIN) && IsBackupRestorable(used_size)))
  {
    EraseFlashOrExit(0, block_size_);
    CopyBlockPrefixAndChecksum(BlockType::MAIN, BlockType::BACKUP, used_size);
    PrepareBackup();
  }
  else
  {
    if (!IsBlockValid(BlockType::MAIN))
    {
      InitBlock(BlockType::MAIN);
    }
    if (IsBlockValid(BlockType::BACKUP) && !IsBlockEmpty(BlockType::BACKUP))
    {
      PrepareBackup();
    }
  }

  RepairMainChain();
}

/**
 * @brief 还原存储数据，清空 Flash 区域
 *        Restore storage data, clearing Flash memory area.
 * @note 这里会把主块初始化为空块、重新准备备份块，不保留任何旧键。
 *       This initializes the main block as an empty block and prepares the
 *       backup block again, keeping no previous key.
 */
void Restore()
{
  InitBlock(BlockType::MAIN);
  PrepareBackup();
}

/**
 * @brief 回收 Flash 空间，整理数据
 *        Recycle Flash storage space and organize data.
 *
 * 将主存储块中的有效键移动到备份块，并擦除主存储块。
 * Moves valid keys from the main block to the backup block and erases the main
 * block.
 *
 * @return 操作结果；整理后连同 CRC 单元仍放不下时不做任何写入，返回
 *         `ErrorCode::FULL`
 *         Operation result; when the compacted keys plus the CRC unit do not fit,
 *         nothing is written and `ErrorCode::FULL` is returned.
 * @note 当前流程把备份块当作临时整理区：先把主块链表中的有效键逐个写入备份块，每个键
 *       提交后再链接；全部写完后写 CRC 单元和备份块校验，它从此是完整拷贝；再擦除主块、
 *       用备份块的有效前缀回写主块，最后重新准备备份块。备份块不处于准备好的状态时先
 *       重新准备。
 *       The current flow treats the backup block as a temporary compaction
 *       area: it writes the live keys of the main chain into the backup block
 *       one by one, linking each key after committing it; after all of them it
 *       writes the CRC unit and the backup checksum, which makes the backup the
 *       complete copy; it then erases the main block, rewrites it from the
 *       backup's live prefix, and finally prepares the backup block again. A
 *       backup block that is not in the prepared state is prepared first.
 */
ErrorCode Recycle()
{
  size_t used_size = LibXR::OffsetOf(&FlashInfo::key) + AlignSize(sizeof(KeyInfo));
  size_t key_offset = GetSentinelOffset(BlockType::MAIN);
  KeyInfo key;
  ReadFlashOrExit(key_offset, key);
  while (NextChainKey(BlockType::MAIN, key_offset, key))
  {
    if (IsKeyLive(key) && IsKeyNameTerminated(key_offset, key))
    {
      used_size += GetKeySpan(key);
    }
  }
  if (used_size + GetCrcSize() > GetChecksumOffset())
  {
    return ErrorCode::FULL;
  }

  if (!IsBackupPrepared())
  {
    PrepareBackup();
  }

  size_t backup_last_key = GetSentinelOffset(BlockType::BACKUP);
  key_offset = GetSentinelOffset(BlockType::MAIN);
  ReadFlashOrExit(key_offset, key);
  while (NextChainKey(BlockType::MAIN, key_offset, key))
  {
    if (!IsKeyLive(key) || !IsKeyNameTerminated(key_offset, key))
    {
      continue;
    }

    const size_t new_key_offset = GetNextKey(backup_last_key);
    WriteKeyHeader(new_key_offset, key.GetNameLength(), key.GetDataSize());
    CopyFlashData(GetKeyName(new_key_offset), GetKeyName(key_offset),
                  key.GetNameLength());
    CopyFlashData(GetKeyData(new_key_offset), GetKeyData(key_offset), key.GetDataSize());
    ClearKeyFlag(new_key_offset, &KeyInfo::uninit);
    LinkKey(BlockType::BACKUP, backup_last_key);
    backup_last_key = new_key_offset;
  }
  DEV_ASSERT(GetNextKey(backup_last_key) - block_size_ == used_size);

  const uint32_t crc = GetBackupCrc(used_size);
  WriteFlashOrExit(GetBlockOffset(BlockType::BACKUP) + used_size, crc);
  WriteBlockChecksum(BlockType::BACKUP);

  EraseFlashOrExit(0, block_size_);
  CopyBlockPrefixAndChecksum(BlockType::MAIN, BlockType::BACKUP, used_size);

  PrepareBackup();

  return ErrorCode::OK;
}
