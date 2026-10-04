/**
 * @brief `DatabaseRaw` 的块级操作片段 / Block-operation fragment of `DatabaseRaw`
 *
 * @note 这一组函数只负责整块语义：块头、尾校验、空块初始化、损坏判断、已用空间计算、
 *       以及块间前缀复制。
 *       This group owns only block-level semantics: block headers, trailing
 *       checksums, empty-block initialization, corruption checks, used-space
 *       accounting, and block-prefix copying.
 */

/**
 * @brief 计算可用的存储空间大小
 *        Calculate the available storage size.
 * @return 剩余的可用字节数 / Remaining available bytes.
 * @note 这里统计的是主块当前已用前缀到块尾校验区之间还剩多少可写空间。
 *       This counts how much writable space remains between the current live
 *       prefix of the main block and its trailing checksum area.
 */
size_t AvailableSize() { return GetChecksumOffset() - GetUsedBlockSize(BlockType::MAIN); }

/**
 * @brief 计算指定块的起始偏移 / Compute the starting offset of one block.
 * @param block 目标块类型 / Target block type.
 * @return 块起始偏移 / Starting offset of the block.
 */
size_t GetBlockOffset(BlockType block)
{
  return block == BlockType::BACKUP ? block_size_ : 0;
}

/**
 * @brief 计算块尾校验区起始偏移 / Compute the starting offset of the checksum area.
 * @return 块尾校验区起始偏移 / Starting offset of the checksum area.
 */
size_t GetChecksumOffset() { return block_size_ - GetChecksumSize(); }

/**
 * @brief 计算块尾校验区字节数 / Compute the byte size of the checksum area.
 * @return 块尾校验区字节数 / Byte size of the checksum area.
 */
size_t GetChecksumSize() { return AlignSize(sizeof(CHECKSUM_BYTE)); }

/**
 * @brief 擦除一个块并写入块头和空哨兵 / Erase one block and write its header and an
 *        empty sentinel.
 * @param block 目标块类型 / Target block type.
 * @note 不写块尾校验。
 *       The trailing checksum is not written.
 */
void WriteEmptyBlockPrefix(BlockType block)
{
  const size_t offset = GetBlockOffset(block);
  EraseFlashOrExit(offset, block_size_);

  FlashInfo info;  // padding filled with 0xFF by constructor
  info.header = FLASH_HEADER;
  KeyInfo& tmp_key = info.key;
  BlockBoolUtil<MinWriteSize>::SetFlag(tmp_key.no_next_key, true);
  BlockBoolUtil<MinWriteSize>::SetFlag(tmp_key.available_flag, true);
  BlockBoolUtil<MinWriteSize>::SetFlag(tmp_key.uninit, false);
  tmp_key.SetNameLength(0);
  tmp_key.SetDataSize(0);
  WriteFlashOrExit(offset, {reinterpret_cast<uint8_t*>(&info), sizeof(FlashInfo)});
}

/**
 * @brief 写入块尾校验 / Write the trailing checksum of a block.
 * @param block 目标块类型 / Target block type.
 */
void WriteBlockChecksum(BlockType block)
{
  WriteFlashOrExit(GetBlockOffset(block) + GetChecksumOffset(),
                   {&CHECKSUM_BYTE, sizeof(CHECKSUM_BYTE)});
}

/**
 * @brief 把指定块初始化为空数据库块 / Initialize one block as an empty database block.
 * @param block 目标块类型 / Target block type.
 * @note 初始化后的块是“结构合法但没有任何用户键”的空块：块头有效、首个哨兵键有效、
 *       尾校验有效。
 *       After initialization, the block is a structurally valid empty block:
 *       valid header, valid sentinel key, and valid trailing checksum.
 */
void InitBlock(BlockType block)
{
  WriteEmptyBlockPrefix(block);
  WriteBlockChecksum(block);
}

/**
 * @brief 把备份块准备成回收可以直接写入的状态
 *        Prepare the backup block so a recycle can write into it directly.
 * @note 备份块只有块头和空哨兵，没有块尾校验。
 *       The backup block holds only the header and an empty sentinel, without
 *       the trailing checksum.
 */
void PrepareBackup() { WriteEmptyBlockPrefix(BlockType::BACKUP); }

/**
 * @brief 判断备份块是否处于 `PrepareBackup()` 之后的状态
 *        Check whether the backup block is in the state `PrepareBackup()` leaves.
 * @return 块头有效、哨兵为空、哨兵之后直到块尾都是擦除态时返回 `true`
 *         Returns `true` when the header is valid, the sentinel is empty, and
 *         everything after the sentinel up to the block end is erased.
 * @note 被打断的回收会在哨兵后面留下已写入的键，再往这些位置写入就是对已编程单元的
 *       二次编程，因此这里检查到块尾。
 *       An interrupted recycle leaves written keys after the sentinel, and
 *       writing there again would program units twice, so the check runs to the
 *       block end.
 */
bool IsBackupPrepared()
{
  const size_t sentinel_offset = GetSentinelOffset(BlockType::BACKUP);
  FlashInfo info;
  ReadFlashOrExit(GetBlockOffset(BlockType::BACKUP), info);
  if (info.header != FLASH_HEADER ||
      !BlockBoolUtil<MinWriteSize>::ReadFlag(info.key.no_next_key) ||
      !BlockBoolUtil<MinWriteSize>::ReadFlag(info.key.available_flag) ||
      info.key.raw_info != 0)
  {
    return false;
  }
  return IsErased(sentinel_offset + AlignSize(sizeof(KeyInfo)),
                  GetBlockOffset(BlockType::BACKUP) + block_size_);
}

/**
 * @brief 计算备份块 CRC 单元的字节数 / Compute the byte size of the backup CRC unit.
 * @return CRC 单元字节数 / Byte size of the CRC unit.
 */
size_t GetCrcSize() { return AlignSize(sizeof(uint32_t)); }

/**
 * @brief 计算备份块前缀的 CRC32 / Compute the CRC32 of the backup block prefix.
 * @param used_size 前缀字节数 / Byte size of the prefix.
 * @return CRC32（多项式 0xEDB88320） / CRC32 (polynomial 0xEDB88320).
 */
uint32_t GetBackupCrc(size_t used_size)
{
  uint32_t crc = 0xFFFFFFFFU;
  uint8_t buffer[32];
  const size_t offset = GetBlockOffset(BlockType::BACKUP);
  for (size_t done = 0; done < used_size;)
  {
    const size_t size = LibXR::min<size_t>(sizeof(buffer), used_size - done);
    ReadFlashOrExit(offset + done, RawData(buffer, size));
    for (size_t i = 0; i < size; ++i)
    {
      crc ^= buffer[i];
      for (int bit = 0; bit < 8; ++bit)
      {
        crc = (crc >> 1) ^ (0xEDB88320U & (0U - (crc & 1U)));
      }
    }
    done += size;
  }
  return ~crc;
}

/**
 * @brief 判断备份块是否可以用来恢复主块（旧版本的判断）
 *        Check whether the backup block can restore the main block (the check of
 *        older versions).
 * @param used_size 返回备份块活跃前缀的字节数 / Receives the byte size of the live prefix
 *        of the backup block.
 * @return 备份块有效、非空且链表完整时返回 `true`
 *         Returns `true` when the backup is valid and not empty and its chain is
 *         complete.
 * @note 旧版本在回收开始时就写好备份块校验，回收被打断时链表也可能恰好完整，所以只在
 *       主块无效时才用这个判断。
 *       Older versions wrote the backup checksum when a recycle started, and the
 *       chain of a cut copy can happen to be complete, so this check is used only
 *       when the main block is invalid.
 */
bool IsBackupRestorable(size_t& used_size)
{
  return IsBlockValid(BlockType::BACKUP) && !IsBlockEmpty(BlockType::BACKUP) &&
         TryGetUsedBlockSize(BlockType::BACKUP, used_size);
}

/**
 * @brief 判断备份块是否是一次回收写完的完整拷贝
 *        Check whether the backup block is the complete copy written by a recycle.
 * @param used_size 返回备份块活跃前缀的字节数 / Receives the byte size of the live prefix
 *        of the backup block.
 * @return 备份块可以恢复主块，且链表之后的 CRC 单元与前缀的 CRC32 相符时返回 `true`
 *         Returns `true` when the backup can restore the main block and the CRC
 *         unit after its chain matches the CRC32 of the prefix.
 * @note 回收写完所有键之后才写 CRC 单元和块尾校验。擦除被打断时，前缀或 CRC 单元里
 *       任何一位回到 1 都会使 CRC 不符，所以不需要另外作废用过的拷贝。
 *       A recycle writes the CRC unit and the trailing checksum only after all
 *       keys. When an erase is interrupted, any bit of the prefix or the CRC unit
 *       that returns to 1 breaks the CRC, so a used copy needs no separate
 *       invalidation.
 */
bool IsBackupComplete(size_t& used_size)
{
  if (!IsBackupRestorable(used_size) || used_size + GetCrcSize() > GetChecksumOffset())
  {
    return false;
  }
  uint32_t stored_crc = 0;
  ReadFlashOrExit(GetBlockOffset(BlockType::BACKUP) + used_size, stored_crc);
  return stored_crc == GetBackupCrc(used_size);
}

/**
 * @brief 判断块头是否已初始化 / Check whether the block header is initialized.
 * @param block 目标块类型 / Target block type.
 * @return 若块头有效则返回 `true` / Returns `true` when the block header is valid.
 */
bool IsBlockInited(BlockType block)
{
  const size_t offset = GetBlockOffset(block);
  FlashInfo flash_data;
  ReadFlashOrExit(offset, flash_data);
  return flash_data.header == FLASH_HEADER;
}

/**
 * @brief 判断块当前是否为空 / Check whether the block is currently empty.
 * @param block 目标块类型 / Target block type.
 * @return 若块内没有有效键则返回 `true`
 *         Returns `true` when the block contains no valid key.
 * @note 这里的“空”指只有初始化哨兵，没有任何可发布的用户键。
 *       Here, "empty" means the block contains only the initialized sentinel
 *       and no publishable user key.
 */
bool IsBlockEmpty(BlockType block)
{
  const size_t offset = GetBlockOffset(block);
  FlashInfo flash_data;
  ReadFlashOrExit(offset, flash_data);
  return BlockBoolUtil<MinWriteSize>::ReadFlag(flash_data.key.available_flag) == true;
}

/**
 * @brief 判断块尾校验是否损坏 / Check whether the block checksum is corrupted.
 * @param block 目标块类型 / Target block type.
 * @return 若块尾校验不符则返回 `true`
 *         Returns `true` when the trailing checksum is invalid.
 */
bool IsBlockError(BlockType block)
{
  const size_t offset = GetBlockOffset(block);
  uint32_t checksum = 0;
  ReadFlashOrExit(offset + GetChecksumOffset(), checksum);
  return checksum != CHECKSUM_BYTE;
}

/**
 * @brief 判断块整体是否处于可用状态
 *        Check whether the block as a whole is currently usable.
 * @param block 目标块类型 / Target block type.
 * @return 若块头、校验有效且哨兵没有名字和数据则返回 `true`
 *         Returns `true` when the header and checksum are valid and the sentinel
 *         has no name and no payload.
 * @note 链表从哨兵开始，哨兵的长度字段不为 0 时链表的起点就不可信。
 *       The chain starts at the sentinel, so a non-zero length field in the
 *       sentinel makes the start of the chain untrustworthy.
 */
bool IsBlockValid(BlockType block)
{
  if (!IsBlockInited(block) || IsBlockError(block))
  {
    return false;
  }
  KeyInfo sentinel;
  ReadFlashOrExit(GetSentinelOffset(block), sentinel);
  return sentinel.raw_info == 0;
}

/**
 * @brief 计算块内哨兵键头的偏移 / Compute the offset of the sentinel key header of a
 *        block.
 * @param block 目标块类型 / Target block type.
 * @return 哨兵键头偏移 / Offset of the sentinel key header.
 */
size_t GetSentinelOffset(BlockType block)
{
  return GetBlockOffset(block) + LibXR::OffsetOf(&FlashInfo::key);
}

/**
 * @brief 判断一段 Flash 是否全部处于擦除态 / Check whether a Flash range is fully erased.
 * @param offset 起始偏移 / Start offset.
 * @param end 结束偏移（不含） / End offset, exclusive.
 * @return 若每个字节都是 `0xFF` 则返回 `true`
 *         Returns `true` when every byte is `0xFF`.
 */
bool IsErased(size_t offset, size_t end)
{
  uint8_t buffer[32];
  while (offset < end)
  {
    const size_t size = LibXR::min<size_t>(sizeof(buffer), end - offset);
    ReadFlashOrExit(offset, RawData(buffer, size));
    for (size_t i = 0; i < size; ++i)
    {
      if (buffer[i] != 0xFF)
      {
        return false;
      }
    }
    offset += size;
  }
  return true;
}

/**
 * @brief 试算一个块里已用空间，并在发现布局损坏时提前失败
 *        Try to compute used block size and fail early on invalid layout.
 * @param block 目标块类型 / Target block type.
 * @param used_size 输出已用字节数 / Receives the used byte size.
 * @return 若成功计算则返回 `true`
 *         Returns `true` when the used size is computed successfully.
 * @note 这个版本给恢复路径用；它不会假设块内键布局一定合法，而是逐步验证每个键头的
 *       位图编码和边界。
 *       This variant is used by recovery paths; it does not assume the key
 *       layout is valid and instead verifies each key header's bitmap
 *       encoding and bounds incrementally.
 */
bool TryGetUsedBlockSize(BlockType block, size_t& used_size)
{
  const size_t block_offset = GetBlockOffset(block);
  const size_t checksum_offset = block_offset + GetChecksumOffset();
  size_t key_offset = block_offset + LibXR::OffsetOf(&FlashInfo::key);

  while (true)
  {
    if (key_offset + AlignSize(sizeof(KeyInfo)) > checksum_offset)
    {
      return false;
    }

    KeyInfo key;
    ReadFlashOrExit(key_offset, key);

    // Recovery cannot trust erased or half-written key metadata to bound copies.
    if (!BlockBoolUtil<MinWriteSize>::Valid(key.no_next_key) ||
        !BlockBoolUtil<MinWriteSize>::Valid(key.available_flag) ||
        !BlockBoolUtil<MinWriteSize>::Valid(key.uninit))
    {
      return false;
    }

    const size_t next_key_offset = key_offset + AlignSize(sizeof(KeyInfo)) +
                                   AlignSize(key.GetNameLength()) +
                                   AlignSize(key.GetDataSize());
    if (next_key_offset > checksum_offset)
    {
      return false;
    }

    if (BlockBoolUtil<MinWriteSize>::ReadFlag(key.no_next_key))
    {
      used_size = next_key_offset - block_offset;
      return true;
    }

    key_offset = next_key_offset;
  }
}

/**
 * @brief 计算一个块当前已用的总字节数 / Compute the currently used byte span of one
 * block.
 * @param block 目标块类型 / Target block type.
 * @return 当前已用的总字节数 / Currently used byte span of the block.
 */
size_t GetUsedBlockSize(BlockType block)
{
  return GetNextKey(GetLastKey(block)) - GetBlockOffset(block);
}

/**
 * @brief 复制活跃键前缀和块尾校验 / Copy the live key prefix and trailing checksum.
 * @param dst_block 目标块类型 / Destination block type.
 * @param src_block 源块类型 / Source block type.
 * @param used_size 活跃前缀总字节数 / Byte size of the live prefix.
 * @note 已擦除尾部不参与复制；只要活跃前缀和块尾校验能重建块语义就够了。
 *       The erased tail is not copied; reproducing the live prefix plus the
 *       trailing checksum is sufficient to rebuild the block semantics.
 */
void CopyBlockPrefixAndChecksum(BlockType dst_block, BlockType src_block,
                                size_t used_size)
{
  const size_t dst_offset = GetBlockOffset(dst_block);
  const size_t src_offset = GetBlockOffset(src_block);
  const size_t checksum_offset = GetChecksumOffset();

  DEV_ASSERT(used_size <= checksum_offset);
  // Only the live key prefix and checksum are needed; erased tail bytes are irrelevant.
  CopyFlashData(dst_offset, src_offset, used_size);
  CopyFlashData(dst_offset + checksum_offset, src_offset + checksum_offset,
                GetChecksumSize());
}
