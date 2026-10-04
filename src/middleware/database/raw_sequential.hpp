#pragma once

#include <cstdint>
#include <cstring>

#include "flash.hpp"
#include "interface.hpp"

namespace LibXR
{

/**
 * @brief 适用于不支持逆序写入的 Flash 存储的数据库实现
 *        Database implementation for Flash storage that does not support reverse
 * writing.
 *
 * 此类管理 Flash 内存区域中的键值存储，其中数据只能顺序写入。
 * 它维护一个备份系统，以防止数据损坏。
 * This class manages key-value storage in a Flash memory region where
 * data can only be written sequentially. It maintains a backup system
 * to prevent data corruption.
 *
 * @note 若底层 Flash 读写擦失败，当前实现视为不可恢复故障并直接触发 `REQUIRE`。
 *       If the underlying Flash read, write, or erase operation fails, the
 *       current implementation treats it as an unrecoverable fault and triggers
 *       `REQUIRE` immediately.
 */
class DatabaseRawSequential : public Database
{
 public:
  /**
   * @brief 构造函数，初始化 Flash 存储和缓冲区
   *        Constructor initializing Flash storage and buffer.
   *
   * @param flash 目标 Flash 存储设备 / Target Flash storage device.
   * @param max_buffer_size 最大缓冲区大小，默认 256 字节
   *        Maximum buffer size, default is 256 bytes.
   *
   * @note 包含动态内存分配。
   *       Contains dynamic memory allocation.
   * @note `max_buffer_size` 必须不超过整片 Flash 容量的一半。
   *       `max_buffer_size` must not exceed half of the total flash capacity.
   */
  explicit DatabaseRawSequential(Flash& flash, size_t max_buffer_size = 256);

  /**
   * @brief 初始化数据库存储区，确保主备块正确
   *        Initialize database storage, ensuring main and backup blocks are valid.
   */
  void Init();

  /**
   * @brief 保存当前缓冲区内容到 Flash
   *        Save the current buffer content to Flash.
   */
  void Save();

  /**
   * @brief 从 Flash 加载数据到缓冲区
   *        Load data from Flash into the buffer.
   */
  void Load();

  /**
   * @brief 还原存储数据，清空 Flash 区域
   *        Restore storage data, clearing Flash memory area.
   */
  void Restore();

  /**
   * @brief 获取数据库中的键值
   *        Retrieve the key's value from the database.
   * @param key 需要获取的键 / Key to retrieve.
   * @return 找到且记录大小与键一致时返回 `ErrorCode::OK`；键不存在返回
   *         `ErrorCode::NOT_FOUND`；记录大小与键不一致返回 `ErrorCode::FAILED`
   *         Returns `ErrorCode::OK` if found with a matching size, `ErrorCode::NOT_FOUND`
   *         if the key does not exist, `ErrorCode::FAILED` if the stored size differs.
   */
  ErrorCode Get(Database::KeyBase& key) override;

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
   * @brief 添加新键到数据库
   *        Add a new key to the database.
   * @param key 需要添加的键 / Key to add.
   * @return 操作结果 / Operation result.
   */
  ErrorCode Add(KeyBase& key) override
  {
    return AddKey(key.name_, key.raw_data_.addr_, key.raw_data_.size_);
  }

 private:
  /**
   * @brief 存储块类型 / Storage block type.
   */
  enum class BlockType : uint8_t
  {
    MAIN = 0,   ///< 主块 / Main block.
    BACKUP = 1  ///< 备份块 / Backup block.
  };

  LIBXR_PACKED_BEGIN
  /**
   * @brief 键信息结构，存储键的元数据
   *        Structure containing key metadata.
   */
  struct KeyInfo
  {
    uint32_t raw_data;  ///< 1 位后继键标志、7 位键名长度、24 位数据长度
                        ///< 1-bit next-key flag, 7-bit name length, 24-bit payload size.

    /**
     * @brief 构造一个擦除态键头 / Construct one erased-state key header.
     */
    KeyInfo();

    /**
     * @brief 构造一个指定元数据的键头
     *        Construct one key header with explicit metadata.
     * @param nextKey 是否还有后继键 / Whether another key follows this one.
     * @param nameLength 键名长度 / Key name length.
     * @param dataSize 数据字节数 / Payload size in bytes.
     */
    KeyInfo(bool nextKey, uint8_t nameLength, uint32_t dataSize);

    /**
     * @brief 设置是否存在后继键 / Set whether another key follows.
     * @param value 是否存在后继键 / Whether another key follows.
     */
    void SetNextKeyExist(bool value);

    /**
     * @brief 获取是否存在后继键 / Get whether another key follows.
     * @return 若存在后继键则返回 `true`
     *         Returns `true` when another key follows.
     */
    bool GetNextKeyExist() const;

    /**
     * @brief 设置键名长度 / Set the key name length.
     * @param len 键名长度 / Key name length.
     */
    void SetNameLength(uint8_t len);

    /**
     * @brief 获取键名长度 / Get the key name length.
     * @return 键名长度 / Key name length.
     */
    uint8_t GetNameLength() const;

    /**
     * @brief 设置数据字节数 / Set the payload size in bytes.
     * @param size 数据字节数 / Payload size in bytes.
     */
    void SetDataSize(uint32_t size);

    /**
     * @brief 获取数据字节数 / Get the payload size in bytes.
     * @return 数据字节数 / Payload size in bytes.
     */
    uint32_t GetDataSize() const;
  };

  static_assert(sizeof(KeyInfo) == 4, "KeyInfo size must be 4 bytes");
  LIBXR_PACKED_END

  /**
   * @brief Flash 存储的块信息结构
   *        Structure representing a Flash storage block.
   */
  struct FlashInfo
  {
    uint32_t header;  ///< Flash 块头标识 / Flash block header identifier.
    KeyInfo key;      ///< 该块的键信息 / Key metadata in this block.
  };

  ErrorCode AddKey(const char* name, const void* data, size_t size);
  ErrorCode SetKey(const char* name, const void* data, size_t size);
  ErrorCode SetKey(size_t offset, const void* data, size_t size);
  ErrorCode GetKeyData(size_t offset, RawData data);
  void InitBlock(BlockType block);
  void ReadFlashOrExit(size_t offset, RawData data);
  void WriteFlashOrExit(size_t offset, ConstRawData data);
  void EraseFlashOrExit(size_t offset, size_t size);
  bool IsBlockInited(BlockType block);
  bool IsBlockEmpty(BlockType block);
  bool IsBlockError(BlockType block);
  bool HasLastKey(size_t offset);
  size_t GetKeySize(size_t offset);
  size_t GetNextKey(size_t offset);
  size_t GetLastKey(BlockType block);
  void SetNestKeyExist(size_t offset, bool exist);
  bool KeyDataCompare(size_t offset, const void* data, size_t size);
  bool KeyNameCompare(size_t offset, const char* name);
  size_t SearchKey(const char* name);

  static constexpr uint32_t FLASH_HEADER =
      0x12345678 + LIBXR_DATABASE_VERSION;  ///< Flash 头部标识 / Flash header identifier.
  static constexpr uint8_t CHECKSUM_BYTE = 0x56;  ///< 校验字节 / Checksum byte.

  Flash& flash_;              ///< 目标 Flash 存储设备 / Target Flash storage device.
  uint8_t* buffer_;           ///< 数据缓冲区 / Data buffer.
  uint32_t block_size_;       ///< Flash 块大小 / Flash block size.
  uint32_t max_buffer_size_;  ///< 最大缓冲区大小 / Maximum buffer size.
};

}  // namespace LibXR
