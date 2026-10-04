/**
 * @brief `DatabaseRaw` 的键级操作片段 / Key-operation fragment of `DatabaseRaw`
 *
 * @note 这一组函数只负责单个键条目的语义：地址计算、名称比较、数据比较、逻辑追加、
 *       以及按名称查找。
 *       This group owns only per-entry semantics: address calculations, name
 *       comparisons, data comparisons, logical appends, and name-based
 *       lookup.
 *
 * @note 写入顺序保证任何时刻掉电都不丢已提交的值：新键先写键头、名字和数据，再清
 *       `uninit` 提交；更新时随后把旧版本标为失效；最后才把上一键的 `no_next_key`
 *       清掉，让新键进入链表。每一步都只编程从未写过的单元。启动时 `Init()` 补完链尾
 *       已提交但未链接的键，丢弃未提交的键。
 *       The write order keeps every committed value across a power loss at any
 *       point: a new key gets its header, name, and payload first, then clearing
 *       `uninit` commits it; an update then marks the old version dead; only
 *       the last step clears `no_next_key` of the previous key and puts the new
 *       key into the chain. Every step programs only units that were never
 *       written. At startup `Init()` links a committed but unlinked key found
 *       after the chain and drops an uncommitted one.
 */

/**
 * @brief 计算某个键的数据区起始偏移
 *        Compute the starting offset of one key payload.
 * @param offset 键头偏移 / Key-header offset.
 * @return 数据区起始偏移 / Starting offset of the payload.
 */
size_t GetKeyData(size_t offset)
{
  KeyInfo key;
  ReadFlashOrExit(offset, key);
  return offset + AlignSize(sizeof(KeyInfo)) + AlignSize(key.GetNameLength());
}

/**
 * @brief 计算某个键名字区起始偏移
 *        Compute the starting offset of one key name.
 * @param offset 键头偏移 / Key-header offset.
 * @return 名字区起始偏移 / Starting offset of the key name.
 */
size_t GetKeyName(size_t offset) { return offset + AlignSize(sizeof(KeyInfo)); }

/**
 * @brief 按键头计算一个键总共占用的字节数 / Compute the total byte span of one key
 *        from its header.
 * @param key 键头 / Key header.
 * @return 该键占用的总字节数 / Total byte size occupied by the key.
 */
size_t GetKeySpan(const KeyInfo& key)
{
  return AlignSize(sizeof(KeyInfo)) + AlignSize(key.GetNameLength()) +
         AlignSize(key.GetDataSize());
}

/**
 * @brief 计算一个键总共占用的字节数 / Compute the total byte span of one key.
 * @param offset 键头偏移 / Key-header offset.
 * @return 该键占用的总字节数 / Total byte size occupied by the key.
 */
size_t GetKeySize(size_t offset)
{
  KeyInfo key;
  ReadFlashOrExit(offset, key);
  return GetKeySpan(key);
}

/**
 * @brief 计算下一键的起始偏移 / Compute the starting offset of the next key.
 * @param offset 当前键头偏移 / Current key-header offset.
 * @return 下一键的起始偏移 / Starting offset of the next key.
 */
size_t GetNextKey(size_t offset) { return offset + GetKeySize(offset); }

/**
 * @brief 判断键是否是已提交且有效的键 / Check whether a key is committed and live.
 * @param key 键头 / Key header.
 * @return 若键有效且已提交则返回 `true`
 *         Returns `true` when the key is live and committed.
 */
static bool IsKeyLive(const KeyInfo& key)
{
  return BlockBoolUtil<MinWriteSize>::ReadFlag(key.available_flag) &&
         !BlockBoolUtil<MinWriteSize>::ReadFlag(key.uninit);
}

/**
 * @brief 把键头中的一个标志写成 `false` / Write one flag of a key header to `false`.
 * @param key_offset 键头偏移 / Key-header offset.
 * @param flag 目标标志 / Target flag.
 * @note 只写这一个最小写入单元；标志已经不是擦除态时不再写入。
 *       Only this one minimum write unit is written, and nothing is written
 *       when the flag is no longer erased.
 */
void ClearKeyFlag(size_t key_offset, BlockBoolData<MinWriteSize> KeyInfo::* flag)
{
  const size_t flag_offset = key_offset + LibXR::OffsetOf(flag);
  BlockBoolData<MinWriteSize> value;
  ReadFlashOrExit(flag_offset, value);
  if (!BlockBoolUtil<MinWriteSize>::ReadFlag(value))
  {
    return;
  }
  BlockBoolUtil<MinWriteSize>::SetFlag(value, false);
  WriteFlashOrExit(flag_offset, value);
}

/**
 * @brief 把紧跟在链尾键之后的键接入链表 / Put the key right after the last key into the
 *        chain.
 * @param block 目标块类型 / Target block type.
 * @param last_key_offset 当前链尾键（可以是哨兵）的偏移 / Offset of the current last key,
 *        which may be the sentinel.
 * @note 链尾是哨兵时，先把哨兵的 `available_flag` 清掉，表示块不再为空。
 *       When the last key is the sentinel, its `available_flag` is cleared
 *       first to mark the block as no longer empty.
 */
void LinkKey(BlockType block, size_t last_key_offset)
{
  if (last_key_offset == GetSentinelOffset(block))
  {
    ClearKeyFlag(last_key_offset, &KeyInfo::available_flag);
  }
  ClearKeyFlag(last_key_offset, &KeyInfo::no_next_key);
}

/**
 * @brief 写入一个未提交的键头 / Write an uncommitted key header.
 * @param key_offset 键头偏移 / Key-header offset.
 * @param name_len 键名长度 / Key name length.
 * @param size 数据字节数 / Payload size in bytes.
 * @note 三个标志保持擦除态（`true`），只写长度字段。
 *       The three flags stay erased (`true`); only the length field is written.
 */
void WriteKeyHeader(size_t key_offset, size_t name_len, size_t size)
{
  KeyInfo key;
  key.SetNameLength(static_cast<uint8_t>(name_len));
  key.SetDataSize(static_cast<uint32_t>(size));
  WriteFlashOrExit(key_offset + LibXR::OffsetOf(&KeyInfo::raw_info), key.raw_info);
}

/**
 * @brief 沿链表前进到下一个键 / Advance to the next key of the chain.
 * @param block 目标块类型 / Target block type.
 * @param key_offset 当前键头偏移，成功时更新为下一键 / Current key-header offset,
 *        updated to the next key on success.
 * @param key 当前键头，成功时更新为下一键 / Current key header, updated to the next
 *        key on success.
 * @return 前进成功返回 `true`；到达链尾或下一键越过校验区时返回 `false`
 *         Returns `true` on success, and `false` at the end of the chain or when
 *         the next key would extend into the checksum area.
 * @note 返回 `false` 而当前键的 `no_next_key` 仍为 `false`，说明链表在此处损坏。
 *       A `false` return while `no_next_key` of the current key is still
 *       `false` means the chain is damaged here.
 */
bool NextChainKey(BlockType block, size_t& key_offset, KeyInfo& key)
{
  if (BlockBoolUtil<MinWriteSize>::ReadFlag(key.no_next_key))
  {
    return false;
  }

  const size_t limit = GetBlockOffset(block) + GetChecksumOffset();
  const size_t next_offset = key_offset + GetKeySpan(key);
  if (next_offset + AlignSize(sizeof(KeyInfo)) > limit)
  {
    return false;
  }

  KeyInfo next_key;
  ReadFlashOrExit(next_offset, next_key);
  if (next_offset + GetKeySpan(next_key) > limit)
  {
    return false;
  }

  key_offset = next_offset;
  key = next_key;
  return true;
}

/**
 * @brief 计算块里链表最后一个键的偏移 / Locate the last key of the chain in a block.
 * @param block 目标块类型 / Target block type.
 * @return 最后一个键的偏移；链表里没有键时为哨兵的偏移
 *         Offset of the last key, or of the sentinel when the chain has no key.
 */
size_t GetLastKey(BlockType block)
{
  size_t key_offset = GetSentinelOffset(block);
  KeyInfo key;
  ReadFlashOrExit(key_offset, key);
  while (NextChainKey(block, key_offset, key))
  {
  }
  return key_offset;
}

/**
 * @brief 比较存储中的键数据和给定数据是否不同
 *        Compare whether the stored payload differs from the given payload.
 * @param offset 键头偏移 / Key-header offset.
 * @param data 待比较数据地址 / Address of the candidate payload.
 * @param size 待比较数据字节数 / Payload size in bytes.
 * @return 若内容不同则返回 `true`
 *         Returns `true` when the payloads differ.
 */
bool KeyDataCompare(size_t offset, const void* data, size_t size)
{
  KeyInfo key;
  ReadFlashOrExit(offset, key);
  size_t key_data_offset = GetKeyData(offset);
  uint8_t data_buffer = 0;
  for (size_t i = 0; i < size; i++)
  {
    ReadFlashOrExit(key_data_offset + i, data_buffer);
    if (data_buffer != (reinterpret_cast<const uint8_t*>(data))[i])
    {
      return true;
    }
  }
  return false;
}

/**
 * @brief 比较存储中的键名和给定名称是否不同
 *        Compare whether the stored key name differs from the given name.
 * @param offset 键头偏移 / Key-header offset.
 * @param name 待比较键名 / Key name to compare against.
 * @return 若名称不同则返回 `true`
 *         Returns `true` when the names differ.
 */
bool KeyNameCompare(size_t offset, const char* name)
{
  KeyInfo key;
  ReadFlashOrExit(offset, key);
  for (size_t i = 0; i < key.GetNameLength(); i++)
  {
    uint8_t data_buffer = 0;
    ReadFlashOrExit(offset + AlignSize(sizeof(KeyInfo)) + i, data_buffer);
    if (data_buffer != name[i])
    {
      return true;
    }
  }
  return false;
}

/**
 * @brief 判断键名是否以 `\0` 结尾 / Check whether a key name ends with `\0`.
 * @param key_offset 键头偏移 / Key-header offset.
 * @param key 键头 / Key header.
 * @return 若键名非空且最后一个字节为 `\0` 则返回 `true`
 *         Returns `true` when the name is not empty and its last byte is `\0`.
 * @note 旧版本在写名字之前就提交键，掉电会留下名字全为 `0xFF` 的有效键；回收时据此
 *       丢弃它们。
 *       Older versions committed a key before writing its name, so a power
 *       loss could leave a live key whose name is all `0xFF`; recycling drops
 *       such keys by this check.
 */
bool IsKeyNameTerminated(size_t key_offset, const KeyInfo& key)
{
  const size_t name_len = key.GetNameLength();
  if (name_len == 0)
  {
    return false;
  }
  uint8_t last_byte = 0xFF;
  ReadFlashOrExit(GetKeyName(key_offset) + name_len - 1, last_byte);
  return last_byte == 0;
}

/**
 * @brief 在主块链表里按名称查找有效键，不触发回收
 *        Find a live key by name in the main chain without recycling.
 * @param name 待查找键名 / Key name to search for.
 * @param dead_keys 返回找到之前经过的失效键数量 / Receives the number of dead keys
 *        passed before the match.
 * @return 找到时返回键头偏移，找不到返回 `0`
 *         Returns the key-header offset when found, otherwise `0`.
 */
size_t FindKey(const char* name, size_t& dead_keys)
{
  size_t key_offset = GetSentinelOffset(BlockType::MAIN);
  KeyInfo key;
  ReadFlashOrExit(key_offset, key);
  dead_keys = 0;

  while (NextChainKey(BlockType::MAIN, key_offset, key))
  {
    if (!IsKeyLive(key))
    {
      dead_keys++;
      continue;
    }
    if (!KeyNameCompare(key_offset, name))
    {
      return key_offset;
    }
  }
  return 0;
}

/**
 * @brief 在主块里按名称查找键，并在删除项过多时触发回收
 *        Search one key by name in the main block and trigger recycle when
 *        too many tombstones are observed.
 * @param name 待查找键名 / Key name to search for.
 * @return 找到时返回键头偏移，找不到返回 `0`
 *         Returns the key-header offset when found, otherwise `0`.
 * @note 这里会顺手统计沿途遇到的失效键数量；超过阈值时，查找结束后会先做一次回收，
 *       再重新查一遍。
 *       This also counts invalidated keys seen along the scan; when that
 *       count exceeds the threshold, it recycles first and then retries the
 *       lookup once.
 */
size_t SearchKey(const char* name)
{
  size_t dead_keys = 0;
  const size_t ans = FindKey(name, dead_keys);
  if (dead_keys > recycle_threshold_)
  {
    Recycle();
    return FindKey(name, dead_keys);
  }
  return ans;
}

/**
 * @brief 确认主块尾部放得下一个新键，必要时先回收
 *        Make sure the tail of the main block can hold one new key, recycling
 *        first when needed.
 * @param name_len 键名长度 / Key name length.
 * @param size 数据字节数 / Payload size in bytes.
 * @param last_key_offset 返回当前链尾键的偏移 / Receives the offset of the current last
 *        key.
 * @return 操作结果，回收后仍放不下时返回 `ErrorCode::FULL`
 *         Operation result; `ErrorCode::FULL` when the key does not fit even
 *         after recycling.
 */
ErrorCode ReserveKey(size_t name_len, size_t size, size_t& last_key_offset)
{
  const size_t needed =
      AlignSize(sizeof(KeyInfo)) + AlignSize(name_len) + AlignSize(size);
  if (AvailableSize() < needed)
  {
    Recycle();
    if (AvailableSize() < needed)
    {
      return ErrorCode::FULL;
    }
  }
  last_key_offset = GetLastKey(BlockType::MAIN);
  return ErrorCode::OK;
}

/**
 * @brief 按名称新增一个键 / Add one key by name.
 * @param name 键名 / Key name.
 * @param data 键数据地址 / Address of the key payload.
 * @param size 键数据字节数 / Payload size in bytes.
 * @return 操作结果 / Operation result.
 */
ErrorCode AddKey(const char* name, const void* data, size_t size)
{
  if (SearchKey(name))
  {
    return SetKey(name, data, size);
  }

  const size_t name_len = strlen(name) + 1;
  size_t last_key_offset = 0;
  const ErrorCode ec = ReserveKey(name_len, size, last_key_offset);
  if (ec != ErrorCode::OK)
  {
    return ec;
  }

  const size_t key_offset = GetNextKey(last_key_offset);
  WriteKeyHeader(key_offset, name_len, size);
  WriteFlashOrExit(GetKeyName(key_offset),
                   {reinterpret_cast<const uint8_t*>(name), name_len});
  WriteFlashOrExit(GetKeyData(key_offset),
                   {reinterpret_cast<const uint8_t*>(data), size});
  ClearKeyFlag(key_offset, &KeyInfo::uninit);
  LinkKey(BlockType::MAIN, last_key_offset);
  return ErrorCode::OK;
}

/**
 * @brief 按名称更新一个键，并在需要时触发回收
 *        Update one key by name and recycle storage when needed.
 * @param name 键名 / Key name.
 * @param data 新数据地址 / Address of the new payload.
 * @param size 新数据字节数 / Payload size in bytes.
 * @param recycle 是否允许本次调用触发回收
 *                Whether this call may trigger recycle.
 * @return 操作结果 / Operation result.
 * @note 当前实现只支持“同名且同尺寸”的逻辑更新；尺寸不一致时直接返回
 *       `ErrorCode::FAILED`，不会自动改写布局。
 *       The current implementation supports only logical replacement of the
 *       same named key with the same payload size; when the size differs, it
 *       returns `ErrorCode::FAILED` directly and does not rewrite the
 *       on-flash layout automatically.
 * @note 新版本提交之后才把旧版本标为失效，掉电时读到的是旧值或新值。
 *       The old version is marked dead only after the new one is committed,
 *       so a power loss leaves either the old or the new value.
 */
ErrorCode SetKey(const char* name, const void* data, size_t size, bool recycle = true)
{
  const size_t key_offset = SearchKey(name);
  if (!key_offset)
  {
    return ErrorCode::FAILED;
  }

  KeyInfo key;
  ReadFlashOrExit(key_offset, key);
  if (key.GetDataSize() != size)
  {
    return ErrorCode::FAILED;
  }
  if (!KeyDataCompare(key_offset, data, size))
  {
    return ErrorCode::OK;
  }

  const size_t name_len = key.GetNameLength();
  if (AvailableSize() <
      AlignSize(sizeof(KeyInfo)) + AlignSize(name_len) + AlignSize(size))
  {
    if (!recycle)
    {
      return ErrorCode::FULL;
    }
    Recycle();
    return SetKey(name, data, size, false);
  }

  const size_t last_key_offset = GetLastKey(BlockType::MAIN);
  const size_t new_key_offset = GetNextKey(last_key_offset);
  WriteKeyHeader(new_key_offset, name_len, size);
  CopyFlashData(GetKeyName(new_key_offset), GetKeyName(key_offset), name_len);
  WriteFlashOrExit(GetKeyData(new_key_offset),
                   {reinterpret_cast<const uint8_t*>(data), size});
  ClearKeyFlag(new_key_offset, &KeyInfo::uninit);
  ClearKeyFlag(key_offset, &KeyInfo::available_flag);
  LinkKey(BlockType::MAIN, last_key_offset);
  return ErrorCode::OK;
}

/**
 * @brief 把链尾之后已提交但未链接的键接入链表
 *        Put a committed but unlinked key found after the chain into the chain.
 * @param last_key_offset 链尾键（可以是哨兵）的偏移 / Offset of the last key, which may
 *        be the sentinel.
 * @param key_offset 链尾之后第一个字节的偏移 / Offset of the first byte after the chain.
 * @return 那里是一个完整的已提交键且其后全为擦除态时补完并返回 `true`，否则返回
 *         `false`
 *         Returns `true` after completing it when a whole committed key is
 *         there and everything after it is erased, otherwise `false`.
 * @note 这是新增或更新在提交之后、链接之前掉电留下的状态：完成被打断的那次写入，
 *       同名的旧版本若仍有效则先标为失效。
 *       This is the state left by a power loss after an add or update was
 *       committed and before it was linked: the interrupted write is finished,
 *       first marking a still-live older version of the same key dead.
 */
bool CompleteUnlinkedKey(size_t last_key_offset, size_t key_offset)
{
  const size_t checksum_offset = GetChecksumOffset();
  if (key_offset + AlignSize(sizeof(KeyInfo)) > checksum_offset)
  {
    return false;
  }

  KeyInfo key;
  ReadFlashOrExit(key_offset, key);
  const size_t end_offset = key_offset + GetKeySpan(key);
  if (!BlockBoolUtil<MinWriteSize>::ReadFlag(key.no_next_key) || !IsKeyLive(key) ||
      end_offset > checksum_offset || !IsKeyNameTerminated(key_offset, key) ||
      !IsErased(end_offset, checksum_offset))
  {
    return false;
  }

  char name[0x80] = {};
  ReadFlashOrExit(GetKeyName(key_offset), RawData(name, key.GetNameLength()));
  size_t dead_keys = 0;
  const size_t old_offset = FindKey(name, dead_keys);
  if (old_offset != 0)
  {
    KeyInfo old_key;
    ReadFlashOrExit(old_offset, old_key);
    if (old_key.GetDataSize() != key.GetDataSize())
    {
      return false;
    }
    ClearKeyFlag(old_offset, &KeyInfo::available_flag);
  }

  LinkKey(BlockType::MAIN, last_key_offset);
  return true;
}

/**
 * @brief 启动时检查主块链表，修复掉电留下的状态
 *        Check the main chain at startup and repair what a power loss left.
 * @note 链表完整且链尾之后全为擦除态时只按失效键数量决定是否回收；链尾之后是已提交的
 *       键时补完链接；其余情况（未提交的键、写了一半的数据、损坏的链表）用回收重建主块，
 *       只保留链表还能到达的有效键。链表中间未提交的键（旧版本留下的）按失效键处理。
 *       When the chain is complete and everything after it is erased, only the
 *       dead-key count decides on a recycle; a committed key after the chain is
 *       linked; anything else (an uncommitted key, half-written data, a damaged
 *       chain) is cleared by rebuilding the main block through a recycle, which
 *       keeps the live keys the chain still reaches. An uncommitted key inside
 *       the chain, left by older versions, counts as dead.
 */
void RepairMainChain()
{
  size_t key_offset = GetSentinelOffset(BlockType::MAIN);
  KeyInfo key;
  ReadFlashOrExit(key_offset, key);
  size_t dead_keys = 0;
  while (NextChainKey(BlockType::MAIN, key_offset, key))
  {
    if (!IsKeyLive(key))
    {
      dead_keys++;
    }
  }

  const bool chain_complete = BlockBoolUtil<MinWriteSize>::ReadFlag(key.no_next_key);
  const size_t tail_offset = key_offset + GetKeySpan(key);
  if (chain_complete && IsErased(tail_offset, GetChecksumOffset()))
  {
    if (dead_keys > recycle_threshold_)
    {
      Recycle();
    }
    return;
  }

  if (chain_complete && CompleteUnlinkedKey(key_offset, tail_offset))
  {
    return;
  }

  // Recycle() 只在整理后的键加上 CRC 单元放不下时返回 FULL，而链尾之后的残留至少占一个
  // 键头，不小于 CRC 单元，所以需要清理残留时回收总能进行。
  // Recycle() returns FULL only when the compacted keys plus the CRC unit do not fit,
  // and leftovers after the chain take at least one key header, no smaller than the CRC
  // unit, so a recycle can always run when leftovers have to be cleared.
  Recycle();
}
