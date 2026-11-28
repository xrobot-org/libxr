/**
 * @file test_display.cpp
 * @brief Terminal 显示测试 / Terminal display tests.
 *
 * 逐字节检查换行、提示符、清屏清行、历史显示，以及行中编辑后的重绘。
 * Check exact bytes for line endings, prompts, clearing, history and redraws after
 * mid-line edits.
 */

#include <cstring>
#include <string>

#include "libxr.hpp"
#include "libxr_def.hpp"
#include "libxr_pipe.hpp"
#include "test.hpp"
#include "test_assert.hpp"

namespace
{

template <LibXR::Terminal<>::Mode ModeValue>
struct TerminalDisplayFixture
{
  LibXR::RamFS ramfs;
  LibXR::Pipe input;
  LibXR::Pipe output;
  LibXR::Terminal<> terminal;

  TerminalDisplayFixture()
      : input(64),
        output(1024),
        terminal(ramfs, nullptr, &input.GetReadPort(), &output.GetWritePort(), ModeValue)
  {
  }

  std::string FlushOutput()
  {
    TEST_ASSERT(terminal.write_stream_.Commit() == LibXR::ErrorCode::OK);

    const size_t output_size = output.GetReadPort().Size();
    if (output_size == 0)
    {
      return {};
    }

    std::string text(output_size, '\0');
    LibXR::ReadOperation read_op;
    TEST_ASSERT(output.GetReadPort()(LibXR::RawData{text.data(), output_size}, read_op) ==
                LibXR::ErrorCode::OK);
    return text;
  }
};

void FillInputLine(LibXR::Terminal<>& terminal, const char* text)
{
  const size_t len = std::strlen(text);
  TEST_ASSERT(len < terminal.input_line_.size());
  if (len > 0)
  {
    std::memcpy(terminal.input_line_.data(), text, len);
  }
  terminal.input_len_ = len;
  terminal.input_line_[len] = '\0';
}

void TestHistoryDisplayAndRestore()
{
  // 检查历史项的显示和切换，再恢复到输入行，确认文本没有被显示操作改坏。
  // Display and select history, then restore the input line and check that rendering
  // preserved its text.
  TerminalDisplayFixture<LibXR::Terminal<>::Mode::CRLF> fixture;

  FillInputLine(fixture.terminal, "alpha");
  fixture.terminal.AddHistory();
  FillInputLine(fixture.terminal, "beta");
  fixture.terminal.AddHistory();

  fixture.terminal.history_index_ = 0;
  fixture.terminal.offset_ = -3;
  fixture.terminal.ShowHistory();
  TEST_ASSERT(fixture.terminal.offset_ == 0);
  TEST_ASSERT(fixture.FlushOutput() == "\033[2K\rramfs:/$ beta");

  fixture.terminal.history_index_ = 1;
  fixture.terminal.ShowHistory();
  TEST_ASSERT(fixture.FlushOutput() == "\033[2K\rramfs:/$ alpha");

  fixture.terminal.CopyHistoryToInputLine();
  TEST_ASSERT(fixture.terminal.history_index_ == -1);
  TEST_ASSERT(fixture.terminal.offset_ == 0);
  TEST_ASSERT(fixture.terminal.input_len_ == 5);
  TEST_ASSERT(std::strcmp(&fixture.terminal.input_line_[0], "alpha") == 0);
}

void TestMidLineDisplayEditing()
{
  // 在行中插入或删除时，终端必须重新绘制光标之后的文本。
  // Inserting or deleting within a line must redraw the text after the cursor.
  TerminalDisplayFixture<LibXR::Terminal<>::Mode::CRLF> fixture;

  FillInputLine(fixture.terminal, "ab");
  fixture.terminal.offset_ = -1;
  fixture.terminal.DisplayChar('X');
  TEST_ASSERT(std::strcmp(&fixture.terminal.input_line_[0], "aXb") == 0);
  TEST_ASSERT(fixture.FlushOutput() == "X\033[s\033[Kb\033[u");

  fixture.terminal.DeleteChar();
  TEST_ASSERT(std::strcmp(&fixture.terminal.input_line_[0], "ab") == 0);
  TEST_ASSERT(fixture.FlushOutput() == "\b \b\033[s\033[Kb\033[u");
}

}  // namespace

void RunTerminalDisplayHistoryTests()
{
  TestHistoryDisplayAndRestore();
  TestMidLineDisplayEditing();
}

namespace
{

void TestLineFeedModes()
{
  // 三种换行设置分别输出约定的字节，不把 CR、LF 和 CRLF 混用。
  // Each line-ending mode emits its own bytes without mixing CR, LF and CRLF.
  TerminalDisplayFixture<LibXR::Terminal<>::Mode::CRLF> crlf_fixture;
  crlf_fixture.terminal.LineFeed();
  TEST_ASSERT(crlf_fixture.FlushOutput() == "\r\n");

  TerminalDisplayFixture<LibXR::Terminal<>::Mode::LF> lf_fixture;
  lf_fixture.terminal.LineFeed();
  TEST_ASSERT(lf_fixture.FlushOutput() == "\n");

  TerminalDisplayFixture<LibXR::Terminal<>::Mode::CR> cr_fixture;
  cr_fixture.terminal.LineFeed();
  TEST_ASSERT(cr_fixture.FlushOutput() == "\r");
}

void TestHeaderAndClearSequences()
{
  // 逐字节检查提示符、清行和清屏序列，避免多一个控制字符也被当成正确。
  // Check prompt, line-clear and screen-clear sequences exactly, including control
  // characters.
  TerminalDisplayFixture<LibXR::Terminal<>::Mode::CRLF> fixture;

  fixture.terminal.ShowHeader();
  TEST_ASSERT(fixture.FlushOutput() == "ramfs:/$ ");

  auto dir = LibXR::RamFS::CreateDir("dir1");
  fixture.ramfs.Add(dir);
  fixture.terminal.current_dir_ = &dir;
  fixture.terminal.ShowHeader();
  TEST_ASSERT(fixture.FlushOutput() == "ramfs:dir1$ ");

  fixture.terminal.ClearLine();
  TEST_ASSERT(fixture.FlushOutput() == "\033[2K\r");

  fixture.terminal.Clear();
  TEST_ASSERT(fixture.FlushOutput() == "\033[2J\033[1H");
}

}  // namespace

void RunTerminalDisplayModeTests()
{
  TestLineFeedModes();
  TestHeaderAndClearSequences();
}

void test_terminal_display()
{
  RunTerminalDisplayModeTests();
  RunTerminalDisplayHistoryTests();
}
