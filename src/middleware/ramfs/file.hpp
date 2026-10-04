/**
 * @brief `RamFS` 的文件节点片段 / File-node fragment of `RamFS`
 */
/**
 * @class File
 * @brief 内存文件或可执行文件 / Memory file or executable file
 */
class File : public FsNode
{
 public:
  /**
   * @brief 判断文件是否只读 / Check whether the file is read-only
   * @return 只读返回 true / True if the file is read-only
   */
  [[nodiscard]] bool IsReadOnly() const { return file_type_ == FileType::READ_ONLY; }

  /**
   * @brief 判断文件是否可写 / Check whether the file is writable
   * @return 可写返回 true / True if the file is writable
   */
  [[nodiscard]] bool IsReadWrite() const { return file_type_ == FileType::READ_WRITE; }

  /**
   * @brief 判断文件是否可执行 / Check whether the file is executable
   * @return 可执行返回 true / True if the file is executable
   */
  [[nodiscard]] bool IsExecutable() const { return file_type_ == FileType::EXEC; }

  /**
   * @brief 执行可执行文件 / Run an executable file
   * @param argc 参数数量 / Argument count
   * @param argv 参数数组 / Argument vector
   * @return 执行返回值 / Execution return value
   */
  int Run(int argc, char** argv);

  /**
   * @brief 访问类型化数据 / Access typed data
   *
   * `Data<T>()` 返回可写引用并要求 READ_WRITE / `Data<T>()` returns a writable
   * reference and requires READ_WRITE.
   * `Data<const T>()` 返回只读引用 / `Data<const T>()` returns a read-only
   * reference for both READ_ONLY and READ_WRITE.
   *
   * @tparam DataType 数据类型；使用 const T 表示只读访问 / Data type; use const T
   *                  for read-only access.
   * @tparam LimitMode 大小检查模式 / Size-check mode
   * @return 类型化数据引用 / Typed data reference
   */
  template <typename DataType, SizeLimitMode LimitMode = SizeLimitMode::MORE>
  decltype(auto) Data()
  {
    using RequestedType = std::remove_reference_t<DataType>;
    using StoredType = std::remove_cv_t<RequestedType>;
    static_assert(!std::is_reference_v<DataType>);
    static_assert(!std::is_volatile_v<RequestedType>);

    ASSERT(LibXR::SizeLimitCheck(LimitMode, sizeof(StoredType), size_));
    if constexpr (std::is_const_v<RequestedType>)
    {
      return *ReadableDataPtr<StoredType>();
    }
    else
    {
      return *WritableDataPtr<StoredType>();
    }
  }

  /**
   * @brief 从 const 文件对象访问类型化只读数据 / Access typed read-only data from a
   * const file object
   *
   * @tparam DataType 数据类型 / Data type
   * @tparam LimitMode 大小检查模式 / Size-check mode
   * @return 类型化只读数据引用 / Typed read-only data reference
   */
  template <typename DataType, SizeLimitMode LimitMode = SizeLimitMode::MORE>
  decltype(auto) Data() const
  {
    using RequestedType = std::remove_reference_t<DataType>;
    using StoredType = std::remove_cv_t<RequestedType>;
    static_assert(!std::is_reference_v<DataType>);
    static_assert(!std::is_volatile_v<RequestedType>);

    ASSERT(LibXR::SizeLimitCheck(LimitMode, sizeof(StoredType), size_));
    return *ReadableDataPtr<StoredType>();
  }

  /**
   * @brief 访问可写原始数据，要求文件为 READ_WRITE / Access writable raw data;
   * requires READ_WRITE
   * @return 可写原始数据视图 / Writable raw data view
   */
  [[nodiscard]] RawData Data() { return RawData(WritableDataPtr<void>(), size_); }

  /**
   * @brief 访问只读原始数据 / Access read-only raw data
   * @return 只读原始数据视图 / Read-only raw data view
   */
  [[nodiscard]] ConstRawData Data() const
  {
    return ConstRawData(ReadableDataPtr<void>(), size_);
  }

 private:
  /**
   * @brief 可执行文件调用入口类型 / Executable entry function type
   */
  using ExecFun = int (*)(void* raw, int argc, char** argv);

  /**
   * @brief 处理不应到达的 File 数据访问路径
   *        Handle one File data-access path that should be unreachable
   *
   * @note 这里保留强约束终止语义，但不再依赖“断言后继续解引用空指针”来满足返回类型，
   *       从而避免发布构建中的未定义行为。
   *       This keeps the strong-failure semantics while no longer relying on
   *       "assert then dereference a null pointer" to satisfy the return
   *       type, avoiding undefined behavior in release builds.
   */
  [[noreturn]] static void DataAccessPanic()
  {
    REQUIRE(false);
    std::abort();
  }

  /**
   * @brief 获取只读数据地址
   *        Get the readable data pointer
   * @tparam DataType 目标数据类型 / Target data type
   * @return 只读数据指针 / Read-only data pointer
   */
  template <typename DataType>
  const DataType* ReadableDataPtr() const
  {
    if (file_type_ == FileType::READ_WRITE)
    {
      return static_cast<const DataType*>(addr_);
    }
    if (file_type_ == FileType::READ_ONLY)
    {
      return static_cast<const DataType*>(addr_const_);
    }
    DataAccessPanic();
  }

  /**
   * @brief 获取可写数据地址
   *        Get the writable data pointer
   * @tparam DataType 目标数据类型 / Target data type
   * @return 可写数据指针 / Writable data pointer
   */
  template <typename DataType>
  DataType* WritableDataPtr()
  {
    if (file_type_ != FileType::READ_WRITE)
    {
      DataAccessPanic();
    }
    return static_cast<DataType*>(addr_);
  }

  /**
   * @brief 构造一个空文件壳 / Construct one empty file shell
   */
  File();

  /**
   * @brief 构造一个具名文件壳 / Construct one named file shell
   * @param name 文件名 / File name
   */
  explicit File(const char* name);

  /**
   * @brief 文件负载联合体 / File payload union
   *
   * 这块存储会根据 `file_type_` 被解释成可写数据、只读数据或可执行入口。
   * The same storage is interpreted either as mutable data, const data, or an
   * executable entry depending on `file_type_`.
   */
  union
  {
    void* addr_;              ///< 可写数据地址 / Writable payload address.
    const void* addr_const_;  ///< 只读数据地址 / Read-only payload address.
    ExecFun exec_;            ///< 可执行入口函数 / Executable entry function.
  };

  void* arg_ = nullptr;  ///< 可执行文件上下文块 / Executable context block.
  size_t size_ = 0;      ///< 数据负载字节数 / Payload size in bytes.
  FileType file_type_ =
      FileType::READ_ONLY;  ///< 当前文件存储形态 / Current file storage kind.

  friend class RamFS;
};
