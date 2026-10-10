/**
 * @file test_ramfs.cpp
 * @brief RamFS 节点与目录测试 / RamFS node and directory tests.
 *
 * 检查文件数据引用、只读属性、命令执行、节点查找、父目录关系和直接子节点遍历。
 * Check file references, read-only data, commands, lookup, parent links and direct-child
 * traversal.
 */

#include "libxr.hpp"
#include "libxr_def.hpp"
#include "test.hpp"
#include "test_assert.hpp"

void test_ramfs()
{
  auto ramfs = LibXR::RamFS();

  int ramfs_arg = 0;

  auto command = LibXR::RamFS::CreateFile<int*>(
      "test_command",
      [](int* arg, int argc, char** argv)
      {
        UNUSED(argc);
        UNUSED(argv);
        *arg = *arg + 1;
        return 0;
      },
      &ramfs_arg);

  auto file_1 = LibXR::RamFS::CreateFile("test_file1", ramfs_arg);
  TEST_ASSERT(command.IsExecutable());
  TEST_ASSERT(!file_1.IsExecutable());

  // A command without an execution context.
  static int plain_calls = 0;
  static int plain_argc = -1;
  static char* plain_argv0 = nullptr;
  auto plain = LibXR::RamFS::CreateCommand("plain",
                                           [](int argc, char** argv) -> int
                                           {
                                             plain_calls++;
                                             plain_argc = argc;
                                             plain_argv0 = argv[0];
                                             return 7;
                                           });
  TEST_ASSERT(plain.IsExecutable());
  char plain_name[] = "plain";
  char* plain_argv[] = {plain_name};
  TEST_ASSERT(plain.Run(1, plain_argv) == 7);
  TEST_ASSERT(plain_calls == 1 && plain_argc == 1 && plain_argv0 == plain_name);
  ramfs.Add(plain);
  TEST_ASSERT(ramfs.FindFile("plain") == &plain);

  auto dir = LibXR::RamFS::CreateDir("test_dir");
  auto nested_dir = LibXR::RamFS::CreateDir("nested_dir");

  for (int i = 1; i < 10; i++)
  {
    command.Run(0, nullptr);
    TEST_ASSERT(file_1.Data<const int>() == i);
  }

  file_1.Data<int>() = 42;
  TEST_ASSERT(ramfs_arg == 42);

  const uint32_t ro_value = 0x12345678;
  auto ro_file = LibXR::RamFS::CreateFile("ro_file", ro_value);
  TEST_ASSERT(ro_file.IsReadOnly());
  TEST_ASSERT(ro_file.Data<const uint32_t>() == ro_value);
  const auto& ro_file_view = ro_file;
  TEST_ASSERT(ro_file_view.Data().size_ == sizeof(ro_value));

  uint16_t nested_value = 0x55AA;
  auto nested_file = LibXR::RamFS::CreateFile("nested_file", nested_value);

  uint32_t custom_context = 0xA5A55A5A;
  auto custom = LibXR::RamFS::Custom("custom_node", 0x42, &custom_context);

  ramfs.Add(dir);
  ramfs.Add(file_1);
  dir.Add(command);
  dir.Add(ro_file);
  dir.Add(nested_dir);
  dir.Add(custom);
  nested_dir.Add(nested_file);

  TEST_ASSERT(ramfs.FindDir("test") == nullptr);
  TEST_ASSERT(ramfs.FindFile("test") == nullptr);
  TEST_ASSERT(ramfs.FindCustom("test") == nullptr);
  TEST_ASSERT(dir.FindFile("test") == nullptr);

  TEST_ASSERT(ramfs.FindDir("test_dir") == &dir);
  TEST_ASSERT(ramfs.FindFile("test_file1") == &file_1);
  TEST_ASSERT(ramfs.FindFile("test_command") == &command);
  TEST_ASSERT(ramfs.FindFile("nested_file") == &nested_file);
  TEST_ASSERT(ramfs.FindCustom("custom_node") == &custom);
  TEST_ASSERT(dir.FindFile("test_command") == &command);
  TEST_ASSERT(dir.FindCustom("custom_node") == &custom);
  TEST_ASSERT(dir.FindNode("custom_node") == &custom);
  TEST_ASSERT(custom.kind_ == 0x42);
  TEST_ASSERT(custom.context_ == &custom_context);
  TEST_ASSERT(dir.FindFile("ro_file") == &ro_file);
  TEST_ASSERT(dir.FindDir(".") == &dir);
  TEST_ASSERT(dir.FindDir("..") == &ramfs.root_);
  TEST_ASSERT(nested_dir.FindDir("..") == &dir);

  uint32_t direct_child_count = 0;
  uint32_t direct_file_count = 0;
  uint32_t direct_dir_count = 0;
  uint32_t direct_custom_count = 0;
  dir.Foreach(
      [&](LibXR::RamFS::FsNode& node)
      {
        direct_child_count++;
        switch (node.GetNodeType())
        {
          case LibXR::RamFS::FsNodeType::FILE:
            direct_file_count++;
            break;
          case LibXR::RamFS::FsNodeType::DIR:
            direct_dir_count++;
            break;
          case LibXR::RamFS::FsNodeType::CUSTOM:
            direct_custom_count++;
            break;
          default:
            TEST_ASSERT(false);
            break;
        }
        return LibXR::ErrorCode::OK;
      });
  TEST_ASSERT(direct_child_count == 4);
  TEST_ASSERT(direct_file_count == 2);
  TEST_ASSERT(direct_dir_count == 1);
  TEST_ASSERT(direct_custom_count == 1);
}
