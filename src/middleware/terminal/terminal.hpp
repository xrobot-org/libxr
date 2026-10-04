#pragma once

#include <array>
#include <atomic>
#include <cctype>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <utility>

#include "libxr_rw.hpp"
#include "ramfs.hpp"
#include "semaphore.hpp"
#include "stack.hpp"

namespace LibXR
{

/**
 * @brief 基于 RamFS 的命令行终端 / Command-line terminal backed by RamFS.
 * @tparam READ_BUFF_SIZE 读取缓冲区大小 / Read buffer size.
 * @tparam MAX_LINE_SIZE 最大输入行长度（字符数）。超出的字符不回显、不写入当前行，
 *         回车时先输出一行截断提示，再照常执行已保留的部分。
 *         Maximum input line length in characters. Further characters are neither
 *         echoed nor stored; on Enter a truncation notice is printed and the kept part
 *         is executed as usual.
 * @tparam MAX_ARG_NUMBER 最大参数数量 / Maximum argument count.
 * @tparam MAX_HISTORY_NUMBER 最大历史记录数量 / Maximum history entry count.
 */
template <size_t READ_BUFF_SIZE = 32, size_t MAX_LINE_SIZE = READ_BUFF_SIZE,
          size_t MAX_ARG_NUMBER = 5, size_t MAX_HISTORY_NUMBER = 5>
class Terminal
{
 private:
  /**
   * @brief 单条历史命令存储 / Storage for one command history entry.
   */
  using HistoryLine = std::array<char, MAX_LINE_SIZE + 1>;

  /**
   * @brief 终端控制序列 / Terminal control sequences.
   *
   * 直接写入输出流，用于清屏、清行、保存和恢复光标位置以及移动光标。
   * Written to the output stream to clear the screen or line, save and restore the
   * cursor, or move it.
   */
  static constexpr char CLEAR_ALL[] =
      "\033[2J\033[1H";  ///< 清屏命令 / Clear screen command
  static constexpr char CLEAR_LINE[] =
      "\033[2K\r";  ///< 清除当前行命令 / Clear current line command
  static constexpr char CLEAR_BEHIND[] =
      "\033[K";  ///< 清除光标后内容命令 / Clear content after cursor command
  static constexpr char KEY_RIGHT[] = "\033[C";  ///< 右箭头键 / Right arrow key
  static constexpr char KEY_LEFT[] = "\033[D";   ///< 左箭头键 / Left arrow key
  static constexpr char KEY_SAVE[] = "\033[s";   ///< 保存光标位置 / Save cursor position
  static constexpr char KEY_LOAD[] =
      "\033[u";  ///< 恢复光标位置 / Restore cursor position
  static constexpr char DELETE_CHAR[] =
      "\b \b";  ///< 退格删除字符 / Backspace delete character

  /**
   * @brief 反向查找字符 / Find a character from the end of a string.
   * @param str 输入字符串 / Input string.
   * @param c 待查找字符 / Character to find.
   * @return 最后一次出现的位置，未找到为 nullptr / Last occurrence, or nullptr if absent.
   */
  char* StrchrRev(char* str, char c)
  {
    auto len = strlen(str);
    for (int i = static_cast<int>(len - 1); i >= 0; i--)
    {
      if (str[i] == c)
      {
        return str + i;
      }
    }
    return nullptr;
  }

 public:
  /**
   * @brief 终端换行模式 / Terminal line ending mode.
   */
  enum class Mode : uint8_t
  {
    CRLF = 0,  ///< 回车换行 / Carriage Return + Line Feed (`\r\n`)
    LF = 1,    ///< 仅换行 / Line Feed (`\n`)
    CR = 2     ///< 仅回车 / Carriage Return (`\r`)
  };

  /**
   * @brief 初始化终端的文件系统和读写端口
   *        / Initialize the terminal filesystem and ports.
   * @param ramfs 关联的文件系统 / Associated filesystem.
   * @param current_dir 当前目录，默认根目录 / Current directory; defaults to the root.
   * @param read_port 读端口，默认标准输入 / Read port; defaults to standard input.
   * @param write_port 写端口，默认标准输出 / Write port; defaults to standard output.
   * @param MODE 换行模式，默认 CRLF / Line ending mode; defaults to CRLF.
   * @note 构造时分配存储 / Allocates storage during construction.
   */
  Terminal(LibXR::RamFS& ramfs, RamFS::Dir* current_dir = nullptr,
           ReadPort* read_port = STDIO::read_, WritePort* write_port = STDIO::write_,
           Mode MODE = Mode::CRLF)
      : read_status_(ReadOperation::OperationPollingStatus::READY),
        write_status_(WriteOperation::OperationPollingStatus::READY),
        MODE(MODE),
        write_op_(write_status_),
        read_port_(read_port),
        write_port_(write_port),
        write_stream_(write_port_, write_op_),
        ramfs_(ramfs),
        current_dir_(current_dir ? current_dir : &ramfs_.root_),
        input_line_(MAX_LINE_SIZE + 1),
        history_(MAX_HISTORY_NUMBER)
  {
    ASSERT(read_port != nullptr);
    ASSERT(write_port != nullptr);
    ASSERT(read_port->Readable());
    ASSERT(write_port->Writable());

    if (write_port == STDIO::write_)
    {
      if (STDIO::write_mutex_ == nullptr)
      {
        STDIO::write_mutex_ = new LibXR::Mutex();
      }

      write_mutex_ = STDIO::write_mutex_;
      STDIO::write_stream_ = &write_stream_;
    }
    else
    {
      write_mutex_ = new LibXR::Mutex();
    }
  }

  /// 读完成方发布、轮询入口读取的状态 / Read completion status used by the polling entry.
  std::atomic<ReadOperation::OperationPollingStatus> read_status_;
  /// 写完成方发布的轮询状态 / Polling status published by write completion.
  std::atomic<WriteOperation::OperationPollingStatus> write_status_;

  const Mode MODE;                  ///< 终端换行模式 / Terminal line feed mode
  WriteOperation write_op_;         ///< 终端写操作 / Terminal write operation
  ReadPort* read_port_;             ///< 读取端口 / Read port
  WritePort* write_port_;           ///< 写入端口 / Write port
  WritePort::Stream write_stream_;  ///< 写入流 / Write stream

  LibXR::Mutex* write_mutex_ = nullptr;  ///< 写入端口互斥锁 / Write port mutex

  RamFS& ramfs_;                    ///< 关联的文件系统 / Associated file system
  char read_buff_[READ_BUFF_SIZE];  ///< 读取缓冲区 / Read buffer

  size_t request_read_size_ =
      0;  ///< 本轮计划读取的字节数 / Byte count requested for the current read attempt.
  RamFS::Dir* current_dir_;        ///< 当前目录 / Current directory
  uint8_t flag_ansi_ = 0;          ///< ANSI 控制字符状态 / ANSI control character state
  int offset_ = 0;                 ///< 光标偏移 / Cursor offset
  Stack<char> input_line_;         ///< 输入行缓冲区 / Input line buffer
  char* arg_tab_[MAX_ARG_NUMBER];  ///< 命令参数列表 / Command argument list
  size_t arg_number_ = 0;          ///< 参数数量 / Number of arguments
  Queue<HistoryLine> history_;     ///< 历史命令 / History of commands
  int history_index_ = -1;         ///< 当前历史索引 / Current history index
  bool linefeed_flag_ = false;     ///< CRLF 抑制标志 / CRLF suppression flag
  char linefeed_char_ = '\0';      ///< 上一个换行字符 / Previous line feed character
  bool line_truncated_ =
      false;  ///< 当前输入行丢弃过超长字符 / Characters beyond MAX_LINE_SIZE were dropped

  /**
   * @brief 行编辑、显示与历史记录实现 / Line editing, display, and command history.
   */
#include "display.hpp"

  /**
   * @brief 路径解析、命令执行与补全实现
   *        / Path parsing, command execution, and completion.
   */
#include "command.hpp"

  /**
   * @brief 输入与控制字符解析实现 / Input and control-character parsing.
   */
#include "input.hpp"

  /**
   * @brief 线程与轮询入口实现 / Thread and polling entry point implementations.
   */
#include "driver.hpp"
};

}  // namespace LibXR
