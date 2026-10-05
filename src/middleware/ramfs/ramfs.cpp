#include "ramfs.hpp"

using namespace LibXR;

/**
 * @brief 构造 RamFS，并挂接根目录下的 `bin` 目录
 *        Construct RamFS and attach the `bin` directory under the root
 * @param name 根目录名称 / Root directory name
 */
RamFS::RamFS(const char* name) : root_(name), bin_("bin") { root_.Add(bin_); }

/**
 * @brief RamFS 目录树里的名称比较函数
 *        Name comparator used by the RamFS directory tree
 * @param a 左侧名称 / Left name
 * @param b 右侧名称 / Right name
 * @return 比较结果，遵循 `strcmp()` 语义 / Comparison result following `strcmp()`
 *         semantics
 */
int RamFS::CompareStr(const char* const& a, const char* const& b) { return strcmp(a, b); }

/**
 * @brief 复制并持有一个节点名称
 *        Duplicate and retain one node name
 * @param name 原始名称，不得为空 / Source name, must not be null
 * @return 新分配的名称缓冲区 / Newly allocated name buffer
 */
char* RamFS::DuplicateName(const char* name)
{
  ASSERT(name != nullptr);

  char* name_buff = new char[strlen(name) + 1];
  strcpy(name_buff, name);
  return name_buff;
}

/**
 * @brief 用节点类型构造 FsNode 基类部分
 *        Construct the FsNode base part from a node type
 * @param node_type 节点类型 / Node type
 */
RamFS::FsNode::FsNode(FsNodeType node_type) : type_(node_type), tree_node_(this) {}

/**
 * @brief 拷贝构造 FsNode 基类部分
 *        Copy-construct the FsNode base part
 * @param other 被拷贝的节点 / Node to copy from
 */
RamFS::FsNode::FsNode(const FsNode& other)
    : name_(other.name_), type_(other.type_), parent_(nullptr), tree_node_(this)
{
}

/**
 * @brief 构造一个空文件壳
 *        Construct one empty file shell
 */
RamFS::File::File() : FsNode(FsNodeType::FILE), addr_(nullptr), arg_(nullptr), size_(0) {}

/**
 * @brief 构造一个具名文件壳
 *        Construct one named file shell
 * @param name 文件名 / File name
 */
RamFS::File::File(const char* name) : File() { this->name_ = DuplicateName(name); }

/**
 * @brief 执行可执行文件节点
 *        Run one executable file node
 * @param argc 参数数量 / Argument count
 * @param argv 参数数组 / Argument vector
 * @return 执行返回值 / Execution return value
 */
int RamFS::File::Run(int argc, char** argv)
{
  ASSERT(file_type_ == FileType::EXEC);
  ASSERT(exec_ != nullptr);
  return exec_(arg_, argc, argv);
}

/**
 * @brief 构造一个空自定义节点壳
 *        Construct one empty custom-node shell
 */
RamFS::Custom::Custom() : FsNode(FsNodeType::CUSTOM) {}

/**
 * @brief 构造一个具名自定义节点
 *        Construct one named custom node
 * @param name 节点名称 / Node name
 * @param kind 用户定义类型 / User-defined kind
 * @param context 用户上下文指针 / User context pointer
 */
RamFS::Custom::Custom(const char* name, uint32_t kind, void* context) : Custom()
{
  this->name_ = DuplicateName(name);
  this->kind_ = kind;
  this->context_ = context;
}

/**
 * @brief 构造一个空目录壳
 *        Construct one empty directory shell
 */
RamFS::Dir::Dir() : FsNode(FsNodeType::DIR), rbt_(RamFS::CompareStr) {}

/**
 * @brief 构造一个具名目录壳
 *        Construct one named directory shell
 * @param name 目录名称 / Directory name
 */
RamFS::Dir::Dir(const char* name) : Dir() { this->name_ = DuplicateName(name); }

/**
 * @brief 把一个节点挂到当前目录下
 *        Attach one node under the current directory
 * @param node 待挂接节点 / Node to attach
 */
void RamFS::Dir::AddNode(FsNode& node)
{
  ASSERT(node.name_ != nullptr);
  ASSERT(FindNode(node.name_) == nullptr);
  node.parent_ = this;
  rbt_.Insert(node.tree_node_, node.name_);
}

/**
 * @brief 在当前目录查找直属子节点
 *        Find one direct child node in the current directory
 * @param name 节点名称 / Node name
 * @return 找到的节点；未找到返回 nullptr
 *         Matching node, or nullptr if not found
 */
RamFS::FsNode* RamFS::Dir::FindNode(const char* name)
{
  if (name == nullptr)
  {
    return nullptr;
  }

  auto* node = rbt_.Search<FsNode*>(name);
  return node != nullptr ? node->data_ : nullptr;
}

/**
 * @brief 在当前目录按类型查找直属节点
 *        Find one direct child node of a given type
 * @param name 节点名称 / Node name
 * @param type 目标节点类型 / Desired node type
 * @return 找到的节点；未找到返回 nullptr
 *         Matching node, or nullptr if not found
 */
RamFS::FsNode* RamFS::Dir::FindNodeByType(const char* name, FsNodeType type)
{
  auto* ans = FindNode(name);
  if (ans == nullptr || ans->GetNodeType() != type)
  {
    return nullptr;
  }
  return ans;
}

/**
 * @brief 递归查找指定类型节点
 *        Find one node of a given type recursively
 * @param name 节点名称 / Node name
 * @param type 目标节点类型 / Desired node type
 * @return 找到的节点；未找到返回 nullptr
 *         Matching node, or nullptr if not found
 *
 * @note 当前实现先检查直属节点；若未命中，再通过局部 visitor 继续深入直属子目录。
 *       The current implementation first checks direct children; if that misses,
 *       it continues descending into direct child directories through a local
 *       visitor.
 */
RamFS::FsNode* RamFS::Dir::FindNodeRevByType(const char* name, FsNodeType type)
{
  auto* ans = FindNodeByType(name, type);
  if (ans != nullptr)
  {
    return ans;
  }

  struct FindNodeRevFn
  {
    const char* name_;
    FsNodeType type_;
    FsNode*& ans_;

    ErrorCode operator()(Tree::Node<FsNode*>& item)
    {
      auto* node = item.data_;
      if (node->GetNodeType() != FsNodeType::DIR)
      {
        return ErrorCode::OK;
      }

      ans_ = static_cast<Dir*>(node)->FindNodeRevByType(name_, type_);
      return ans_ != nullptr ? ErrorCode::FAILED : ErrorCode::OK;
    }
  };

  FindNodeRevFn find{name, type, ans};
  rbt_.Foreach<FsNode*>(find);
  return ans;
}

/**
 * @brief 查找直属文件节点
 *        Find one direct child file node
 * @param name 文件名 / File name
 * @return 文件节点；未找到返回 nullptr
 *         File node, or nullptr if not found
 */
RamFS::File* RamFS::Dir::FindFile(const char* name)
{
  return static_cast<File*>(FindNodeByType(name, FsNodeType::FILE));
}

/**
 * @brief 递归查找文件节点
 *        Find one file node recursively
 * @param name 文件名 / File name
 * @return 文件节点；未找到返回 nullptr
 *         File node, or nullptr if not found
 */
RamFS::File* RamFS::Dir::FindFileRev(const char* name)
{
  return static_cast<File*>(FindNodeRevByType(name, FsNodeType::FILE));
}

/**
 * @brief 查找直属目录节点，支持 "." 和 ".."
 *        Find one direct child directory node, supporting "." and ".."
 * @param name 目录名称 / Directory name
 * @return 目录节点；未找到返回 nullptr
 *         Directory node, or nullptr if not found
 */
RamFS::Dir* RamFS::Dir::FindDir(const char* name)
{
  if (name == nullptr)
  {
    return nullptr;
  }

  if (name[0] == '.' && name[1] == '\0')
  {
    return this;
  }

  if (name[0] == '.' && name[1] == '.' && name[2] == '\0')
  {
    return parent_;
  }

  return static_cast<Dir*>(FindNodeByType(name, FsNodeType::DIR));
}

/**
 * @brief 递归查找目录节点，支持 "." 和 ".."
 *        Find one directory node recursively, supporting "." and ".."
 * @param name 目录名称 / Directory name
 * @return 目录节点；未找到返回 nullptr
 *         Directory node, or nullptr if not found
 */
RamFS::Dir* RamFS::Dir::FindDirRev(const char* name)
{
  if (name == nullptr)
  {
    return nullptr;
  }

  if (name[0] == '.' && name[1] == '\0')
  {
    return this;
  }

  if (name[0] == '.' && name[1] == '.' && name[2] == '\0')
  {
    return parent_;
  }

  return static_cast<Dir*>(FindNodeRevByType(name, FsNodeType::DIR));
}

/**
 * @brief 查找直属自定义节点
 *        Find one direct child custom node
 * @param name 节点名称 / Node name
 * @return 自定义节点；未找到返回 nullptr
 *         Custom node, or nullptr if not found
 */
RamFS::Custom* RamFS::Dir::FindCustom(const char* name)
{
  return static_cast<Custom*>(FindNodeByType(name, FsNodeType::CUSTOM));
}

/**
 * @brief 递归查找自定义节点
 *        Find one custom node recursively
 * @param name 节点名称 / Node name
 * @return 自定义节点；未找到返回 nullptr
 *         Custom node, or nullptr if not found
 */
RamFS::Custom* RamFS::Dir::FindCustomRev(const char* name)
{
  return static_cast<Custom*>(FindNodeRevByType(name, FsNodeType::CUSTOM));
}
