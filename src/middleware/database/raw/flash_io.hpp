/**
 * @brief `DatabaseRaw` 的底层 Flash IO 片段 / Low-level Flash-I/O fragment of
 *        `DatabaseRaw`
 *
 * @note 这一组函数只处理“怎么按最小写入单元安全读写 Flash”，不负责数据库条目或
 *       块级语义。
 *       This group handles only "how to read and write Flash safely under
 *       the minimum write-unit rule"; it does not own database-entry or
 *       block-level semantics.
 */

/**
 * @brief 读取 Flash 数据，失败则直接触发强约束
 *        Read flash data and fail fast on error.
 * @param offset 读取偏移 / Read offset.
 * @param data 接收数据的缓冲区 / Destination buffer receiving the data.
 */
void ReadFlashOrExit(size_t offset, RawData data)
{
  [[maybe_unused]] const auto flash_io_result = flash_.Read(offset, data);
  REQUIRE(flash_io_result == ErrorCode::OK);
}

/**
 * @brief 读取 Flash 数据到对象里，失败则直接触发强约束
 *        Read flash data into one object and fail fast on error.
 * @tparam Data 接收对象类型 / Destination object type.
 * @param offset 读取偏移 / Read offset.
 * @param data 接收数据的对象 / Destination object receiving the data.
 */
template <typename Data>
void ReadFlashOrExit(size_t offset, Data& data)
{
  ReadFlashOrExit(offset, RawData(data));
}

/**
 * @brief 写入 Flash 数据，失败则直接触发强约束
 *        Write flash data and fail fast on error.
 * @param offset 写入偏移 / Write offset.
 * @param data 待写入数据 / Data to write.
 */
void WriteFlashOrExit(size_t offset, ConstRawData data)
{
  [[maybe_unused]] const auto flash_io_result = Write(offset, data);
  REQUIRE(flash_io_result == ErrorCode::OK);
}

/**
 * @brief 写入一个对象到 Flash，失败则直接触发强约束
 *        Write one object to flash and fail fast on error.
 * @tparam Data 待写入对象类型 / Object type to write.
 * @param offset 写入偏移 / Write offset.
 * @param data 待写入对象 / Object to write.
 */
template <typename Data>
void WriteFlashOrExit(size_t offset, const Data& data)
{
  WriteFlashOrExit(offset, ConstRawData(data));
}

/**
 * @brief 擦除 Flash 区域，失败则直接触发强约束
 *        Erase a flash range and fail fast on error.
 * @param offset 擦除偏移 / Erase offset.
 * @param size 擦除字节数 / Erase size in bytes.
 */
void EraseFlashOrExit(size_t offset, size_t size)
{
  [[maybe_unused]] const auto flash_io_result = flash_.Erase(offset, size);
  REQUIRE(flash_io_result == ErrorCode::OK);
}

/**
 * @brief 在两个块之间按最小写入单元复制数据
 *        Copy data between blocks in minimum-write-size chunks.
 * @param dst_offset 目标偏移 / Destination offset.
 * @param src_offset 源偏移 / Source offset.
 * @param size 待复制字节数 / Byte count to copy.
 */
void CopyFlashData(size_t dst_offset, size_t src_offset, size_t size)
{
  for (size_t i = 0; i < size; i += MinWriteSize)
  {
    ReadFlashOrExit(src_offset + i, {write_buffer_, MinWriteSize});
    WriteFlashOrExit(dst_offset + i, {write_buffer_, MinWriteSize});
  }
}

/**
 * @brief 计算对齐后的大小
 *        Calculate the aligned size.
 * @param size 需要对齐的大小 / Size to align.
 * @return 对齐后的大小 / Aligned size.
 */
size_t AlignSize(size_t size)
{
  return static_cast<size_t>((size + MinWriteSize - 1) / MinWriteSize) * MinWriteSize;
}

/**
 * @brief 以最小写入单元对齐的方式写入数据
 *        Write data aligned to the minimum write unit.
 * @param offset 写入偏移量 / Write offset.
 * @param data 要写入的数据 / Data to write.
 * @return 操作结果 / Operation result.
 * @note 当末尾不足一个最小写入单元时，当前实现会用 `0xFF` 补齐尾块后再写入。
 *       When the tail is shorter than one minimum write unit, the current
 *       implementation pads that final unit with `0xFF` before writing it.
 */
ErrorCode Write(size_t offset, ConstRawData data)
{
  if (data.size_ == 0)
  {
    return ErrorCode::OK;
  }

  if (data.size_ % MinWriteSize == 0)
  {
    return flash_.Write(offset, data);
  }
  else
  {
    auto final_block_index = data.size_ - data.size_ % MinWriteSize;
    if (final_block_index != 0)
    {
      auto ec = flash_.Write(offset, {data.addr_, final_block_index});
      if (ec != ErrorCode::OK)
      {
        return ec;
      }
    }
    Memory::FastSet(write_buffer_, 0xff, MinWriteSize);
    LibXR::Memory::FastCopy(
        write_buffer_, reinterpret_cast<const uint8_t*>(data.addr_) + final_block_index,
        data.size_ % MinWriteSize);
    return flash_.Write(offset + final_block_index, {write_buffer_, MinWriteSize});
  }
}
