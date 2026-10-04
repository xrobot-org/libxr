/**
 * @enum BlockType
 * @brief 当前 raw 数据库使用的两个物理块角色 / Two physical block roles used by the
 *        current raw database
 *
 * 一个块承载当前主数据，另一个块在恢复和回收流程里充当暂存 / 备份块。
 * One block is treated as the current main data block, while the other acts
 * as the scratch / backup block during recovery and recycle.
 */
enum class BlockType : uint8_t
{
  MAIN = 0,   ///< 主块 / Main block.
  BACKUP = 1  ///< 备份块 / Backup block.
};

LIBXR_PACKED_BEGIN
/**
 * @brief 按最小写入单元存放布尔位图块
 *        Boolean flag block stored in one aligned write unit span.
 * @tparam BlockSize 位图块字节数 / Flag-block size in bytes.
 */
template <size_t BlockSize>
struct BlockBoolData
{
  uint8_t data[BlockSize];
};
LIBXR_PACKED_END

/**
 * @brief 读写对齐布尔位图块的工具
 *        Helpers for reading and writing aligned boolean flag blocks.
 * @tparam BlockSize 位图块字节数 / Flag-block size in bytes.
 */
template <size_t BlockSize>
class BlockBoolUtil
{
 public:
  /**
   * @brief 把一个布尔值编码进位图块 / Encode one boolean value into a flag block.
   * @param obj 目标位图块 / Target flag block.
   * @param value 待编码布尔值 / Boolean value to encode.
   */
  static void SetFlag(BlockBoolData<BlockSize>& obj, bool value)
  {
    Memory::FastSet(obj.data, 0xFF, BlockSize);
    if (!value)
    {
      obj.data[BlockSize - 1] &= 0xF0;
    }
  }

  /**
   * @brief 从位图块读取布尔值 / Decode one boolean value from a flag block.
   * @param obj 待读取位图块 / Flag block to inspect.
   * @return 解码出的布尔值 / Decoded boolean value.
   * @note 标志只会从 `true`（擦除态）写成 `false`，且只写一次；只要有一位已被编程，
   *       就说明这次写入已经开始，因此只有整块都是 `0xFF` 才读作 `true`。被掉电打断的
   *       写入读作它要写成的 `false`。
   *       A flag is written at most once, from `true` (erased) to `false`; any
   *       programmed bit means that write has started, so only a fully erased
   *       block reads as `true`, and a write cut by a power loss reads as the
   *       `false` it was writing.
   */
  static bool ReadFlag(const BlockBoolData<BlockSize>& obj)
  {
    for (size_t i = 0; i < BlockSize; ++i)
    {
      if (obj.data[i] != 0xFF)
      {
        return false;
      }
    }
    return true;
  }

  /**
   * @brief 检查位图块内容是否仍是合法编码 / Check whether a flag block still contains
   *        a valid encoding.
   * @param obj 待检查位图块 / Flag block to validate.
   * @return 若编码合法则返回 `true` / Returns `true` when the encoding is valid.
   */
  static bool Valid(const BlockBoolData<BlockSize>& obj)
  {
    if (BlockSize == 0)
    {
      return false;
    }

    for (size_t i = 0; i < BlockSize - 1; ++i)
    {
      if (obj.data[i] != 0xFF)
      {
        return false;
      }
    }

    uint8_t last_byte = obj.data[BlockSize - 1];
    if ((last_byte & 0xF0) != 0xF0)
    {
      return false;
    }

    uint8_t last_4bits = last_byte & 0x0F;
    if (!(last_4bits == 0x0F || last_4bits == 0x00))
    {
      return false;
    }

    return true;
  }
};

LIBXR_PACKED_BEGIN
/**
 * @brief 键信息结构，存储键的元数据
 *        Structure containing key metadata.
 */
struct KeyInfo
{
  BlockBoolData<MinWriteSize> no_next_key;     ///< 是否是最后一个键
  BlockBoolData<MinWriteSize> available_flag;  ///< 该键是否有效
  BlockBoolData<MinWriteSize> uninit;          ///< 该键是否未初始化

  uint32_t raw_info =
      0;  ///< 高 7 位保存 nameLength，低 25 位保存 dataSize。
          ///< Upper 7 bits store nameLength and lower 25 bits store dataSize.

  /**
   * @brief 构造一个默认可写的键头元数据
   *        Construct one default writable key-header metadata object.
   */
  KeyInfo()
  {
    BlockBoolUtil<MinWriteSize>::SetFlag(no_next_key, true);
    BlockBoolUtil<MinWriteSize>::SetFlag(available_flag, true);
    BlockBoolUtil<MinWriteSize>::SetFlag(uninit, true);
  }

  /**
   * @brief 设置键名长度 / Set the key name length.
   * @param len 键名长度 / Key name length.
   */
  void SetNameLength(uint8_t len)
  {
    raw_info = (raw_info & 0x01FFFFFF) | ((len & 0x7F) << 25);
  }

  /**
   * @brief 获取键名长度 / Get the key name length.
   * @return 键名长度 / Key name length.
   */
  uint8_t GetNameLength() const { return (raw_info >> 25) & 0x7F; }

  /**
   * @brief 设置数据字节数 / Set the payload size in bytes.
   * @param size 数据字节数 / Payload size in bytes.
   */
  void SetDataSize(uint32_t size)
  {
    raw_info = (raw_info & 0xFE000000) | (size & 0x01FFFFFF);
  }

  /**
   * @brief 获取数据字节数 / Get the payload size in bytes.
   * @return 数据字节数 / Payload size in bytes.
   */
  uint32_t GetDataSize() const { return raw_info & 0x01FFFFFF; }
};
LIBXR_PACKED_END

LIBXR_PACKED_BEGIN
/**
 * @brief Flash 存储的块信息结构
 *        Structure representing a Flash storage block.
 */
struct FlashInfo
{
  /**
   * @brief 构造一个擦除态 FlashInfo 缓冲对象
   *        Construct one erased-state FlashInfo buffer object.
   */
  FlashInfo()
  {
    header = 0xFFFFFFFF;
    Memory::FastSet(padding, 0xFF, MinWriteSize);
  }

  /**
   * @brief 让块前缀既能按签名读取，也能按最小写入单元整体写入 /
   *        Expose the block prefix both as a signature field and as one full
   *        minimum-write-unit write span
   */
  union
  {
    uint32_t header;                ///< Flash 块头签名 / Flash block-header signature.
    uint8_t padding[MinWriteSize];  ///< 按最小写入单元对齐的原始块前缀 / Raw block prefix
                                    ///< aligned to one minimum write unit.
  };
  KeyInfo key;  ///< 紧跟在块头后的首个键头 / First key header stored right after the
                ///< block header.
};
LIBXR_PACKED_END
