#pragma once

#include <cstring>

#include "libxr_assert.hpp"
#include "mutex.hpp"

namespace LibXR
{
/**
 * @brief 红黑树实现，支持泛型键和值，并提供线程安全操作
 *        Red-Black Tree implementation supporting generic keys and values with
 * thread-safe operations.
 *
 * 该类实现了自平衡二叉查找树（红黑树），以提供高效的插入、删除和查找操作。
 * This class implements a self-balancing binary search tree (Red-Black Tree)
 * to provide efficient insert, delete, and search operations.
 *
 * @tparam Key 用作节点键的类型 / Type used as node key.
 */
template <typename Key>
class RBTree
{
 public:
  /**
   * @brief 定义红黑树节点的颜色 / Enumeration for node colors in Red-Black Tree.
   */
  enum class RbtColor : uint8_t
  {
    RED,   ///< 红色节点 / Red node.
    BLACK  ///< 黑色节点 / Black node.
  };

  /**
   * @brief 红黑树的基本节点结构 / Base node structure of the Red-Black Tree.
   */
  class BaseNode
  {
   public:
    Key key;                     ///< 节点键值 / Key associated with the node.
    RbtColor color;              ///< 节点颜色 / Color of the node.
    BaseNode* left = nullptr;    ///< 左子节点 / Left child node.
    BaseNode* right = nullptr;   ///< 右子节点 / Right child node.
    BaseNode* parent = nullptr;  ///< 父节点 / Parent node.
    size_t size;                 ///< 节点大小 / Size of the node.

   protected:
    /**
     * @brief 基本节点构造函数 / Constructor for BaseNode.
     * @param size 节点数据大小 / Size of the node's data.
     */
    explicit BaseNode(size_t size) : size(size) {}
  };

  /**
   * @brief 红黑树的泛型数据节点，继承自 `BaseNode`
   *        Generic data node for Red-Black Tree, inheriting from `BaseNode`.
   *
   * @tparam Data 存储的数据类型 / Type of data stored in the node.
   */
  template <typename Data>
  class Node : public BaseNode
  {
   public:
    /**
     * @brief 默认构造函数，初始化数据为空
     *        Default constructor initializing an empty node.
     */
    Node() : BaseNode(sizeof(Data)), data_{} {}

    /**
     * @brief 使用指定数据构造节点
     *        Constructor initializing a node with the given data.
     * @param data 要存储的数据 / Data to store in the node.
     */
    explicit Node(const Data& data) : BaseNode(sizeof(Data)), data_(data) {}

    /**
     * @brief 通过参数列表构造节点 / Constructor initializing a node using arguments list.
     * @tparam Args 参数类型 / Types of arguments for data initialization.
     * @param args 数据构造参数 / Arguments used for constructing the data.
     */
    template <typename... Args>
    explicit Node(Args... args) : BaseNode(sizeof(Data)), data_{args...}
    {
    }

    operator Data&() { return data_; }
    Node& operator=(const Data& data)
    {
      data_ = data;
      return *this;
    }
    Data* operator->() { return &data_; }
    const Data* operator->() const { return &data_; }
    Data& operator*() { return data_; }

    Data data_;  ///< 存储的数据 / Stored data.
  };

  /**
   * @brief 构造函数，初始化红黑树 / Constructor initializing the Red-Black Tree.
   * @param compare_fun 比较函数指针，用于键值比较 / Comparison function pointer for key
   * comparison.
   */
  explicit RBTree(int (*compare_fun)(const Key&, const Key&)) : compare_fun_(compare_fun)
  {
    ASSERT(compare_fun_);
  }

  /**
   * @brief 搜索红黑树中的节点 / Search for a node in the Red-Black Tree.
   * @tparam Data 存储的数据类型 / Type of data stored in the node.
   * @tparam LimitMode 结构大小检查模式 / Size limit check mode.
   * @param key 要搜索的键 / Key to search for.
   * @return 指向找到的节点的指针，如果未找到返回 `nullptr`
   *         Pointer to the found node, or `nullptr` if not found.
   */
  template <typename Data, SizeLimitMode LimitMode = SizeLimitMode::MORE>
  Node<Data>* Search(const Key& key)
  {
    mutex_.Lock();
    Node<Data>* result = nullptr;
    if (root_)
    {
      if (BaseNode* found = Search(root_, key))
      {
        result = ToDerivedType<Data, LimitMode>(found);
      }
    }
    mutex_.Unlock();
    return result;
  }

  /**
   * @brief 从树中删除指定节点 / Delete a specified node from the tree.
   * @param node 要删除的节点 / Node to be deleted.
   */
  void Delete(BaseNode& node)
  {
    mutex_.Lock();

    BaseNode *child = nullptr, *parent = nullptr;
    RbtColor color = RbtColor::BLACK;

    if (node.left && node.right)
    {
      BaseNode* replace = node.right;
      while (replace->left)
      {
        replace = replace->left;
      }

      if (node.parent)
      {
        (node.parent->left == &node ? node.parent->left : node.parent->right) = replace;
      }
      else
      {
        root_ = replace;
      }

      child = replace->right;
      parent = replace->parent;
      color = replace->color;

      if (parent == &node)
      {
        parent = replace;
      }
      else
      {
        if (child)
        {
          child->parent = parent;
        }

        if (parent)
        {
          parent->left = child;
        }

        if (node.right)
        {
          replace->right = node.right;
          node.right->parent = replace;
        }
      }

      replace->parent = node.parent;
      replace->color = node.color;
      replace->left = node.left;
      node.left->parent = replace;

      if (color == RbtColor::BLACK)
      {
        RbtreeDeleteFixup(child, parent);
      }
      mutex_.Unlock();
      return;
    }

    child = node.left ? node.left : node.right;
    parent = node.parent;
    color = node.color;

    if (child)
    {
      child->parent = parent;
    }

    if (parent)
    {
      (parent->left == &node ? parent->left : parent->right) = child;
    }
    else
    {
      root_ = child;
    }

    if (color == RbtColor::BLACK)
    {
      RbtreeDeleteFixup(child, parent);
    }
    mutex_.Unlock();
  }

  /**
   * @brief 在树中插入新节点 / Insert a new node into the tree.
   * @tparam KeyType 插入键的类型 / Type of the key to insert.
   * @param node 要插入的节点 / Node to insert.
   * @param key 节点键 / Key of the node.
   */
  template <typename KeyType>
  void Insert(BaseNode& node, KeyType&& key)
  {
    mutex_.Lock();
    node.left = nullptr;
    node.right = nullptr;
    node.parent = nullptr;
    node.color = RbtColor::RED;
    node.key = std::forward<KeyType>(key);
    RbtreeInsert(node);
    mutex_.Unlock();
  }

  /**
   * @brief 获取树中的节点数量 / Get the number of nodes in the tree.
   * @return 树中节点的数量 / Number of nodes in the tree.
   */
  uint32_t GetNum()
  {
    mutex_.Lock();
    uint32_t count = 0;
    RbtreeGetNum(root_, &count);
    mutex_.Unlock();
    return count;
  }

  /**
   * @brief 遍历红黑树并执行用户提供的操作 (Traverse the Red-Black Tree and apply a
   * user-defined function).
   * @tparam Data 存储的数据类型 / Type of data stored in the node.
   * @tparam Func 用户定义的操作函数 / User-defined function to apply.
   * @tparam LimitMode 结构大小检查模式 / Size limit check mode.
   * @param func 作用于每个节点的函数 / Function applied to each node.
   * @return 操作结果，成功返回 `ErrorCode::OK`
   *         Operation result: `ErrorCode::OK` on success.
   */
  template <typename Data, typename Func, SizeLimitMode LimitMode = SizeLimitMode::MORE>
  ErrorCode Foreach(Func func)
  {
    mutex_.Lock();
    ErrorCode result = RbtreeForeachStart<Data>(root_, func);
    mutex_.Unlock();
    return result;
  }

  /**
   * @brief 获取红黑树的下一个中序遍历节点
   *        Get the next node in in-order traversal.
   * @tparam Data 存储的数据类型 / Type of data stored in the node.
   * @param node 当前节点 / Current node.
   * @return 指向下一个节点的指针 / Pointer to the next node.
   */
  template <typename Data>
  Node<Data>* ForeachDisc(Node<Data>* node)
  {
    mutex_.Lock();
    Node<Data>* result = nullptr;
    if (!node)
    {
      result = static_cast<Node<Data>*>(root_);
      while (result && result->left)
      {
        result = static_cast<Node<Data>*>(result->left);
      }
    }
    else if (node->right)
    {
      result = static_cast<Node<Data>*>(node->right);
      while (result && result->left)
      {
        result = static_cast<Node<Data>*>(result->left);
      }
    }
    else if (node->parent)
    {
      if (node == node->parent->left)
      {
        result = static_cast<Node<Data>*>(node->parent);
      }
      else
      {
        while (node->parent && node == node->parent->right)
        {
          node = static_cast<Node<Data>*>(node->parent);
        }
        result = static_cast<Node<Data>*>(node->parent);
      }
    }
    mutex_.Unlock();
    return result;
  }

 private:
  BaseNode* root_ = nullptr;  ///< 红黑树的根节点 / Root node of the Red-Black Tree.
  LibXR::Mutex mutex_;        ///< 互斥锁，确保线程安全 / Mutex for thread-safety.
  int (*compare_fun_)(const Key&,
                      const Key&);  ///< 键值比较函数 / Function for key comparison.

  void RbtreeInsert(BaseNode& node)
  {
    BaseNode* parent = nullptr;
    BaseNode** current = &root_;
    while (*current)
    {
      parent = *current;
      current =
          (compare_fun_(node.key, parent->key) < 0) ? &parent->left : &parent->right;
    }
    node.parent = parent;
    *current = &node;
    RbtreeInsertFixup(&node);
  }

  void RbtreeInsertFixup(BaseNode* node)
  {
    BaseNode *parent = nullptr, *gparent = nullptr;

    while ((parent = node->parent) && parent->color == RbtColor::RED)
    {
      gparent = parent->parent;

      if (parent == gparent->left)
      {
        BaseNode* uncle = gparent->right;
        if (uncle && uncle->color == RbtColor::RED)
        {
          uncle->color = RbtColor::BLACK;
          parent->color = RbtColor::BLACK;
          gparent->color = RbtColor::RED;
          node = gparent;
          continue;
        }

        if (node == parent->right)
        {
          BaseNode* tmp = nullptr;
          RbtreeLeftRotate(parent);
          tmp = parent;
          parent = node;
          node = tmp;
        }

        parent->color = RbtColor::BLACK;
        gparent->color = RbtColor::RED;
        RbtreeRightRotate(gparent);
      }
      else
      {
        BaseNode* uncle = gparent->left;
        if (uncle && uncle->color == RbtColor::RED)
        {
          uncle->color = RbtColor::BLACK;
          parent->color = RbtColor::BLACK;
          gparent->color = RbtColor::RED;
          node = gparent;
          continue;
        }

        if (node == parent->left)
        {
          BaseNode* tmp = nullptr;
          RbtreeRightRotate(parent);
          tmp = parent;
          parent = node;
          node = tmp;
        }

        parent->color = RbtColor::BLACK;
        gparent->color = RbtColor::RED;
        RbtreeLeftRotate(gparent);
      }
    }
    root_->color = RbtColor::BLACK;
  }

  void RbtreeLeftRotate(BaseNode* x)
  {
    if (!x || !x->right)
    {
      return;
    }

    BaseNode* y = x->right;
    x->right = y->left;
    if (y->left)
    {
      y->left->parent = x;
    }

    y->parent = x->parent;

    if (!x->parent)
    {
      root_ = y;
    }
    else
    {
      if (x == x->parent->left)
      {
        x->parent->left = y;
      }
      else
      {
        x->parent->right = y;
      }
    }

    y->left = x;
    x->parent = y;
  }

  void RbtreeRightRotate(BaseNode* y)
  {
    if (!y || !y->left)
    {
      return;
    }

    BaseNode* x = y->left;
    y->left = x->right;
    if (x->right)
    {
      x->right->parent = y;
    }

    x->parent = y->parent;

    if (!y->parent)
    {
      root_ = x;
    }
    else
    {
      if (y == y->parent->right)
      {
        y->parent->right = x;
      }
      else
      {
        y->parent->left = x;
      }
    }

    x->right = y;
    y->parent = x;
  }

  void RbtreeDeleteFixup(BaseNode* node, BaseNode* parent)
  {
    BaseNode* other = nullptr;

    while ((!node || node->color == RbtColor::BLACK) && node != root_)
    {
      if (parent->left == node)
      {
        other = parent->right;
        if (other->color == RbtColor::RED)
        {
          other->color = RbtColor::BLACK;
          parent->color = RbtColor::RED;
          RbtreeLeftRotate(parent);
          other = parent->right;
        }
        if ((!other->left || other->left->color == RbtColor::BLACK) &&
            (!other->right || other->right->color == RbtColor::BLACK))
        {
          other->color = RbtColor::RED;
          node = parent;
          parent = node->parent;
        }
        else
        {
          if (!other->right || other->right->color == RbtColor::BLACK)
          {
            other->left->color = RbtColor::BLACK;
            other->color = RbtColor::RED;
            RbtreeRightRotate(other);
            other = parent->right;
          }
          other->color = parent->color;
          parent->color = RbtColor::BLACK;
          other->right->color = RbtColor::BLACK;
          RbtreeLeftRotate(parent);
          node = root_;
          break;
        }
      }
      else
      {
        other = parent->left;
        if (other->color == RbtColor::RED)
        {
          other->color = RbtColor::BLACK;
          parent->color = RbtColor::RED;
          RbtreeRightRotate(parent);
          other = parent->left;
        }
        if ((!other->left || other->left->color == RbtColor::BLACK) &&
            (!other->right || other->right->color == RbtColor::BLACK))
        {
          other->color = RbtColor::RED;
          node = parent;
          parent = node->parent;
        }
        else
        {
          if (!other->left || other->left->color == RbtColor::BLACK)
          {
            other->right->color = RbtColor::BLACK;
            other->color = RbtColor::RED;
            RbtreeLeftRotate(other);
            other = parent->left;
          }
          other->color = parent->color;
          parent->color = RbtColor::BLACK;
          other->left->color = RbtColor::BLACK;
          RbtreeRightRotate(parent);
          node = root_;
          break;
        }
      }
    }
    if (node)
    {
      node->color = RbtColor::BLACK;
    }
  }

  template <typename Data, typename Func>
  ErrorCode RbtreeForeachStart(BaseNode* node, Func func)
  {
    if (!node)
    {
      return ErrorCode::OK;
    }

    if (ErrorCode code =
            RbtreeForeach<Data, Func>(reinterpret_cast<Node<Data>*>(node->left), func);
        code != ErrorCode::OK)
    {
      return code;
    }

    if (ErrorCode code = func(*reinterpret_cast<Node<Data>*>(node));
        code != ErrorCode::OK)
    {
      return code;
    }

    return RbtreeForeach<Data, Func>(reinterpret_cast<Node<Data>*>(node->right), func);
  }

  template <typename Data, typename Func>
  ErrorCode RbtreeForeach(BaseNode* node, Func func)
  {
    if (!node)
    {
      return ErrorCode::OK;
    }

    if (ErrorCode code =
            RbtreeForeach<Data, Func>(reinterpret_cast<Node<Data>*>(node->left), func);
        code != ErrorCode::OK)
    {
      return code;
    }

    if (ErrorCode code = func(*reinterpret_cast<Node<Data>*>(node));
        code != ErrorCode::OK)
    {
      return code;
    }

    return RbtreeForeach<Data, Func>(reinterpret_cast<Node<Data>*>(node->right), func);
  }

  void RbtreeGetNum(BaseNode* node, uint32_t* count)
  {
    if (!node)
    {
      return;
    }
    ++(*count);
    RbtreeGetNum(node->left, count);
    RbtreeGetNum(node->right, count);
  }

  BaseNode* Search(BaseNode* x, const Key& key)
  {
    while (x)
    {
      int cmp = compare_fun_(key, x->key);
      if (cmp == 0)
      {
        break;
      }
      x = cmp < 0 ? x->left : x->right;
    }
    return x;
  }

  template <typename Data, SizeLimitMode LimitMode>
  static Node<Data>* ToDerivedType(BaseNode* node)
  {
    if (node)
    {
      ASSERT(LibXR::SizeLimitCheck(LimitMode, sizeof(Data), node->size));
    }
    return static_cast<Node<Data>*>(node);
  }
};
}  // namespace LibXR
