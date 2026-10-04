/**
 * @file stack_insert_nontrivial.cpp
 * @brief Stack::Insert 拒绝不可平凡复制的元素类型 / Stack::Insert rejects an element type
 * that is not trivially copyable.
 *
 * 这个文件必须编译失败。CTest 单独构建它，并检查输出中有 static_assert 的报错文字。
 * This file must fail to compile. CTest builds it separately and checks the output for
 * the static_assert message.
 */

#include <string>

#include "stack.hpp"

LibXR::ErrorCode InsertNonTriviallyCopyable(LibXR::Stack<std::string>& stack)
{
  return stack.Insert(std::string("text"), 0);
}
