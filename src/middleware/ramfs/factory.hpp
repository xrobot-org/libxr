/**
 * @brief `RamFS` 的工厂与根入口片段 / Factory and root-entry fragment of `RamFS`
 */
/**
 * @brief 创建引用外部数据的文件 / Create a file referencing external data
 * @tparam DataType 外部数据类型 / External data type
 * @param name 文件名 / File name
 * @param raw 外部数据引用 / External data reference
 * @return 文件节点 / File node
 *
 * @note 包含动态内存分配 / Contains dynamic memory allocation
 */
template <typename DataType>
static File CreateFile(const char* name, DataType& raw)
{
  using StoredType = std::remove_reference_t<DataType>;

  File file(name);
  if constexpr (std::is_const_v<StoredType>)
  {
    file.file_type_ = FileType::READ_ONLY;
    file.addr_const_ = &raw;
  }
  else
  {
    file.file_type_ = FileType::READ_WRITE;
    file.addr_ = &raw;
  }

  file.size_ = sizeof(StoredType);
  return file;
}

/**
 * @brief 创建可执行文件 / Create an executable file
 * @tparam ArgType 执行上下文参数类型 / Execution context argument type
 * @param name 文件名 / File name
 * @param exec 执行函数 / Execution function
 * @param arg 执行上下文参数 / Execution context argument
 * @return 可执行文件节点 / Executable file node
 *
 * @note 包含动态内存分配 / Contains dynamic memory allocation
 * @note 可执行块按当前 RamFS 设计属于初始化后常驻状态；
 *       当前实现不提供运行期回收路径。
 *       Executable blocks are retained for startup / lifetime registration in
 *       the current RamFS design; the implementation does not provide a
 *       runtime reclamation path for them.
 */
template <typename ArgType>
static File CreateFile(const char* name, int (*exec)(ArgType arg, int argc, char** argv),
                       ArgType&& arg)
{
  using StoredArgType = std::remove_reference_t<ArgType>;
  struct ExecutableBlock
  {
    StoredArgType arg_;
    decltype(exec) exec_fun_;
  };

  File file(name);

  auto block = new ExecutableBlock{std::forward<ArgType>(arg), exec};
  file.file_type_ = FileType::EXEC;
  file.arg_ = block;

  file.exec_ = [](void* raw, int argc, char** argv)
  {
    auto* block = static_cast<ExecutableBlock*>(raw);
    return block->exec_fun_(block->arg_, argc, argv);
  };

  return file;
}

/**
 * @brief 创建命令兼容入口，返回可执行文件 / Create a command-compatible executable
 * file
 * @tparam ArgType 执行上下文参数类型 / Execution context argument type
 * @param name 文件名 / File name
 * @param exec 执行函数 / Execution function
 * @param arg 执行上下文参数 / Execution context argument
 * @return 可执行文件节点 / Executable file node
 *
 * @note 包含动态内存分配 / Contains dynamic memory allocation
 * @note 运行期持久化语义与上面的 executable factory 相同；
 *       命令块按当前设计默认常驻。
 *       This follows the same lifetime-retained semantics as the executable
 *       factory above; command blocks are intentionally retained by design.
 */
template <typename ArgType>
static File CreateCommand(const char* name,
                          int (*exec)(ArgType arg, int argc, char** argv), ArgType&& arg)
{
  return CreateFile(name, exec, std::forward<ArgType>(arg));
}

/**
 * @brief 创建不需要执行上下文的命令 / Create a command that needs no execution context
 * @param name 文件名 / File name
 * @param exec 命令函数，参数为 argc 和 argv / Command function taking argc and argv
 * @return 可执行文件节点 / Executable file node
 *
 * @note 等价于带执行上下文的写法，上下文为函数本身；
 *       不再需要 `CreateCommand<void*>(name, [](void*, int, char**) {...}, nullptr)`。
 *       Equivalent to the form with an execution context, the context being the function
 *       itself; `CreateCommand<void*>(name, [](void*, int, char**) {...}, nullptr)` is no
 *       longer needed.
 * @note 包含动态内存分配，命令块按当前设计默认常驻。
 *       Contains dynamic memory allocation; the command block is retained by design.
 */
static File CreateCommand(const char* name, int (*exec)(int argc, char** argv))
{
  using CommandFn = int (*)(int, char**);
  return CreateFile<CommandFn>(
      name, [](CommandFn fn, int argc, char** argv) -> int { return fn(argc, argv); },
      std::move(exec));
}

/**
 * @brief 创建目录节点 / Create a directory node
 * @param name 目录名称 / Directory name
 * @return 目录节点 / Directory node
 *
 * @note 包含动态内存分配 / Contains dynamic memory allocation
 */
static Dir CreateDir(const char* name) { return Dir(name); }

/**
 * @brief 添加文件节点到根目录 / Add a file node to the root directory
 * @param file 文件节点 / File node
 *
 * @note RamFS 只保存节点地址，节点必须是在整个运行期内有效的左值；
 *       `Add(RamFS::CreateCommand(...))` 传入临时对象会留下悬空指针，因此不接受右值。
 *       RamFS only stores the node address, so the node must be an lvalue that lives for
 *       the whole run; passing a temporary from `CreateCommand(...)` would leave a
 *       dangling pointer, which is why rvalues are not accepted.
 */
void Add(File& file) { root_.Add(file); }

/**
 * @brief 添加目录节点到根目录 / Add a directory node to the root directory
 * @param dir 目录节点 / Directory node
 */
void Add(Dir& dir) { root_.Add(dir); }

/**
 * @brief 添加自定义节点到根目录 / Add a custom node to the root directory
 * @param custom 自定义节点 / Custom node
 */
void Add(Custom& custom) { root_.Add(custom); }

/**
 * @brief 从整个 RamFS 递归查找文件 / Find a file recursively from the RamFS root
 * @param name 文件名 / File name
 * @return 文件指针；未找到返回 nullptr / File pointer, or nullptr
 */
File* FindFile(const char* name) { return root_.FindFileRev(name); }

/**
 * @brief 从整个 RamFS 递归查找目录 / Find a directory recursively from the RamFS root
 * @param name 目录名 / Directory name
 * @return 目录指针；未找到返回 nullptr / Directory pointer, or nullptr
 */
Dir* FindDir(const char* name) { return root_.FindDirRev(name); }

/**
 * @brief 从整个 RamFS 递归查找自定义节点 / Find a custom node recursively from the
 * RamFS root
 * @param name 节点名称 / Node name
 * @return 自定义节点指针；未找到返回 nullptr / Custom node pointer, or nullptr
 */
Custom* FindCustom(const char* name) { return root_.FindCustomRev(name); }

Dir root_;  ///< 根目录；所有外部 Add()/Find*() 默认都从这里进入 / Root directory; all
            ///< external Add()/Find*() entry through here.
Dir bin_;   ///< 预留的可执行文件目录 / Reserved executable-file directory.
