#pragma once

#include <cstdint>
#include <cstring>

#include "libxr_def.hpp"
#include "libxr_mem.hpp"
#include "libxr_type.hpp"

namespace LibXR
{

/**
 * @brief 数据库存储格式版本号
 *        Database storage-format version.
 */
static constexpr uint16_t LIBXR_DATABASE_VERSION = 3;

/**
 * @brief 数据库接口，提供键值存储和管理功能
 *        Database interface providing key-value storage and management.
 */
class Database
{
 public:
  /**
   * @brief 键的基类，存储键名及其数据
   *        Base class for keys, storing key name and associated data.
   */
  class KeyBase
  {
   public:
    const char* name_;  ///< 键名 / Key name.
    RawData raw_data_;  ///< 原始数据 / Raw data associated with the key.

    /**
     * @brief 构造函数，初始化键名和原始数据
     *        Constructor to initialize key name and raw data.
     * @param name 键名 / Key name.
     * @param raw_data 关联的原始数据 / Raw data associated with the key.
     */
    KeyBase(const char* name, RawData raw_data) : name_(name), raw_data_(raw_data) {}
  };

  /**
   * @brief 模板类，表示数据库中的具体键
   *        Template class representing a specific key in the database.
   * @tparam Data 存储的数据类型 / The type of data stored.
   */
  template <typename Data>
  class Key : public KeyBase
  {
   public:
    Data data_;           ///< 键存储的数据 / The data stored in the key.
    Database& database_;  ///< 关联的数据库对象 / Reference to the associated database.

    /**
     * @brief 构造函数，初始化键并从数据库加载数据
     *        Constructor to initialize key and load data from the database.
     *
     * 如果键在数据库中不存在，则使用提供的值进行初始化。
     * If the key does not exist in the database, it is initialized with the provided
     * value.
     *
     * @param database 关联的数据库对象 / Reference to the associated database.
     * @param name 键名 / Key name.
     * @param init_value 初始化值 / Initial value for the key.
     */
    Key(Database& database, const char* name, Data init_value)
        : KeyBase(name, RawData(data_)), database_(database)
    {
      ErrorCode status = database.Get(*this);
      if (status != ErrorCode::OK)
      {
        data_ = init_value;
        if (status == ErrorCode::NOT_FOUND)
        {
          [[maybe_unused]] const auto add_result = database.Add(*this);
          REQUIRE(add_result == ErrorCode::OK);
        }
      }
    }

    /**
     * @brief 构造函数，初始化键，并在数据库不存在时赋默认值
     *        Constructor to initialize key, assigning default value if not found in the
     * database.
     *
     * 如果键在数据库中不存在，则初始化为零。
     * If the key does not exist in the database, it is initialized with zero.
     *
     * @param database 关联的数据库对象 / Reference to the associated database.
     * @param name 键名 / Key name.
     */
    Key(Database& database, const char* name)
        : KeyBase(name, RawData(data_)), database_(database)
    {
      ErrorCode status = database.Get(*this);
      if (status != ErrorCode::OK)
      {
        Memory::FastSet(&data_, 0, sizeof(Data));
        if (status == ErrorCode::NOT_FOUND)
        {
          [[maybe_unused]] const auto add_result = database.Add(*this);
          REQUIRE(add_result == ErrorCode::OK);
        }
      }
    }

    /**
     * @brief 禁止拷贝数据库键对象
     *        Copy construction is disabled for database keys.
     * @param other 被拷贝的键对象 / Database key to copy from.
     */
    Key(const Key& other) = delete;

    /**
     * @brief 禁止移动数据库键对象
     *        Move construction is disabled for database keys.
     * @param other 被转移的键对象 / Database key to move from.
     */
    Key(Key&& other) = delete;

    /**
     * @brief 禁止拷贝赋值数据库键对象
     *        Copy assignment is disabled for database keys.
     * @param other 被拷贝的键对象 / Database key to copy from.
     * @return 当前键对象引用 / Reference to the current key object.
     */
    Key& operator=(const Key& other) = delete;

    /**
     * @brief 禁止移动赋值数据库键对象
     *        Move assignment is disabled for database keys.
     * @param other 被转移的键对象 / Database key to move from.
     * @return 当前键对象引用 / Reference to the current key object.
     */
    Key& operator=(Key&& other) = delete;

    /**
     * @brief 类型转换运算符，返回存储的数据
     *        Type conversion operator returning stored data.
     * @return 存储的数据 / Stored data.
     */
    operator Data() { return data_; }

    /**
     * @brief 保存当前键值到数据库
     *        Save the current key value to the database.
     * @return 操作结果 / Operation result.
     */
    ErrorCode Save() { return database_.Set(*this, this->raw_data_); }

    /**
     * @brief 设置键的值并更新数据库
     *        Set the key's value and update the database.
     * @param data 需要存储的新值 / New value to store.
     * @return 操作结果 / Operation result.
     */
    ErrorCode Set(Data data)
    {
      data_ = data;
      return Save();
    }

    /**
     * @brief 从数据库加载键的值
     *        Load the key's value from the database.
     * @return 操作结果 / Operation result.
     */
    ErrorCode Load() { return database_.Get(*this); }

    /**
     * @brief 赋值运算符，设置键的值
     *        Assignment operator to set the key's value.
     * @param data 需要存储的新值 / New value to store.
     * @return 操作结果 / Operation result.
     */
    ErrorCode operator=(Data data) { return Set(data); }  // NOLINT
  };

 private:
  /**
   * @brief 从数据库获取键的值
   *        Retrieve the key's value from the database.
   * @param key 需要获取的键 / Key to retrieve.
   * @return 操作结果 / Operation result.
   */
  virtual ErrorCode Get(KeyBase& key) = 0;

  /**
   * @brief 设置数据库中的键值
   *        Set the key's value in the database.
   * @param key 目标键 / Target key.
   * @param data 需要存储的新值 / New value to store.
   * @return 操作结果 / Operation result.
   */
  virtual ErrorCode Set(KeyBase& key, RawData data) = 0;

  /**
   * @brief 添加新键到数据库
   *        Add a new key to the database.
   * @param key 需要添加的键 / Key to add.
   * @return 操作结果 / Operation result.
   */
  virtual ErrorCode Add(KeyBase& key) = 0;
};

}  // namespace LibXR
