/**
 * @brief  `Terminal` 的显示与历史记录片段
 *         Display and history fragment of `Terminal`
 *
 * @note 这一组函数只负责行编辑、提示符显示、光标移动，以及历史记录的展示/拷回。
 *       This group is responsible only for line editing, prompt rendering,
 *       cursor movement, and displaying / restoring history entries.
 */

/**
 * @brief  计算一条历史命令的实际文本长度
 *         Calculates the actual text length of one stored history line
 * @param  line 历史命令缓存 / Stored history line
 * @return size_t 历史命令的实际长度
 *         Actual length of the history line
 */
static size_t HistoryLineSize(const HistoryLine& line)
{
  size_t size = 0;
  while (size < MAX_LINE_SIZE && line[size] != '\0')
  {
    size++;
  }
  return size;
}

/**
 * @brief  按“从最新到最旧”的顺序取历史记录
 *         Fetches one history entry counting from newest to oldest
 * @param  index_from_newest 从最新一条开始的偏移；0 表示最新一条
 *         Offset counted from the newest entry; 0 means the newest one
 * @return const HistoryLine& 对应的历史记录
 *         Reference to the requested history entry
 *
 * @note `Queue::operator[]` 支持负索引从尾部反向访问，但这里把这个细节包起来，
 *       避免调用点直接暴露 `history_[-index - 1]` 这种难读写法。
 *       `Queue::operator[]` supports negative indices to walk backward from
 *       the tail, but this helper hides that detail so call sites do not have
 *       to expose the hard-to-read `history_[-index - 1]` form directly.
 */
const HistoryLine& HistoryFromNewest(int index_from_newest)
{
  DEV_ASSERT(index_from_newest >= 0);
  DEV_ASSERT(index_from_newest < static_cast<int>(history_.Size()));
  return history_[-index_from_newest - 1];
}

/**
 * @brief  执行换行操作
 *         Performs a line feed operation
 *
 * @note 具体输出 `\r\n`、`\n` 还是 `\r` 由构造时选定的 `MODE` 决定。
 *       The actual emitted line ending is selected by the `MODE` chosen at
 *       construction time.
 */
void LineFeed()
{
  if (MODE == Mode::CRLF)
  {
    write_stream_ << ConstRawData("\r\n");
  }
  else if (MODE == Mode::LF)
  {
    write_stream_ << ConstRawData('\n');
  }
  else if (MODE == Mode::CR)
  {
    write_stream_ << ConstRawData('\r');
  }
}

/**
 * @brief  更新光标位置
 *         Updates cursor position
 *
 * @note 这里只重绘光标右侧的可见后缀；光标左侧的内容已经在输入缓冲区中保留。
 *       This redraws only the visible suffix to the right of the cursor; the
 *       left side is already preserved in the input buffer.
 */
void UpdateDisplayPosition()
{
  write_stream_ << ConstRawData(KEY_SAVE) << ConstRawData(CLEAR_BEHIND)
                << ConstRawData(&input_line_[input_line_.Size() + offset_], -offset_)
                << ConstRawData(KEY_LOAD);
}

/**
 * @brief  检查是否可以显示字符
 *         Checks if a character can be displayed
 * @return bool 是否可以显示字符 / Whether the character can be displayed
 */
bool CanDisplayChar() { return input_line_.EmptySize() > 1; }

/**
 * @brief  检查是否可以删除字符
 *         Checks if a character can be deleted
 * @return bool 是否可以删除字符 / Whether the character can be deleted
 */
bool CanDeleteChar() { return input_line_.Size() + offset_ > 0; }

/**
 * @brief  向输入行中添加字符，支持在光标位置插入
 *         Adds a character to the input line, supports insertion at the cursor position
 * @param  data 要添加的字符 / The character to add
 */
void AddCharToInputLine(char data)
{
  if (offset_ == 0)
  {
    input_line_.Push(data);
  }
  else
  {
    input_line_.Insert(data, input_line_.Size() + offset_);
  }
  input_line_[input_line_.Size()] = '\0';
}

/**
 * @brief  在终端上显示字符，并根据历史记录模式进行相应操作
 *         Displays a character on the terminal and updates accordingly if history mode
 * is active
 * @param  data 要显示的字符 / The character to display
 */
void DisplayChar(char data)
{
  bool use_history = false;

  if (history_index_ >= 0)
  {
    CopyHistoryToInputLine();
    use_history = true;
  }

  if (CanDisplayChar())
  {
    AddCharToInputLine(data);
    if (use_history)
    {
      ShowHistory();
    }
    else
    {
      write_stream_ << ConstRawData(input_line_[input_line_.Size() - 1 + offset_]);
    }
    if (offset_ != 0)
    {
      UpdateDisplayPosition();
    }
  }
  else
  {
    line_truncated_ = true;
  }
}

/**
 * @brief  从输入行中删除字符，支持在光标位置删除
 *         Removes a character from the input line, supports deletion at the cursor
 * position
 */
void RemoveCharFromInputLine()
{
  if (offset_ == 0)
  {
    input_line_.Pop();
  }
  else
  {
    input_line_.Delete(input_line_.Size() + offset_ - 1);
  }
  input_line_[input_line_.Size()] = '\0';
}

/**
 * @brief  处理删除字符操作，支持回退删除，并在历史模式下更新显示
 *         Handles the delete character operation, supports backspace deletion, and
 * updates display in history mode
 */
void DeleteChar()
{
  bool use_history = false;

  if (history_index_ >= 0)
  {
    CopyHistoryToInputLine();
    use_history = true;
  }

  if (CanDeleteChar())
  {
    RemoveCharFromInputLine();
    if (use_history)
    {
      ShowHistory();
    }
    else
    {
      write_stream_ << ConstRawData(DELETE_CHAR);
    }

    if (offset_ != 0)
    {
      UpdateDisplayPosition();
    }
  }
}

/**
 * @brief  显示终端提示符，包括当前目录信息
 *         Displays the terminal prompt, including the current directory information
 */
void ShowHeader()
{
  write_stream_ << ConstRawData(ramfs_.root_.GetName(), strlen(ramfs_.root_.GetName()));
  if (current_dir_ == &ramfs_.root_)
  {
    write_stream_ << ConstRawData(":/");
  }
  else
  {
    write_stream_ << ConstRawData(":") << ConstRawData(current_dir_->GetName());
  }

  write_stream_ << ConstRawData("$ ");
}

/**
 * @brief  输出输入行被截断的提示
 *         Prints the notice that the input line was truncated
 *
 * @note 提示写明保留的字符数，即模板参数 `MAX_LINE_SIZE`，随后换行。
 *       The notice names the kept character count, the template parameter
 *       `MAX_LINE_SIZE`, and ends with a line feed.
 */
void ShowTruncatedLineNotice()
{
  char digits[20];
  size_t pos = sizeof(digits);
  size_t value = MAX_LINE_SIZE;
  do
  {
    digits[--pos] = static_cast<char>('0' + value % 10);
    value /= 10;
  } while (value != 0);

  write_stream_ << ConstRawData("Line truncated to ")
                << ConstRawData(&digits[pos], sizeof(digits) - pos)
                << ConstRawData(" characters (MAX_LINE_SIZE).");
  LineFeed();
}

/**
 * @brief  清除当前行
 *         Clears the current line
 */
void ClearLine() { write_stream_ << ConstRawData(CLEAR_LINE); }

/**
 * @brief  清除整个终端屏幕
 *         Clears the entire terminal screen
 */
void Clear() { write_stream_ << ConstRawData(CLEAR_ALL); }

/**
 * @brief  显示历史记录中的输入行，更新终端显示
 *         Displays the input line from history and updates the terminal display
 *
 * @note 当 `history_index_ < 0` 时，这里回显的是当前尚未提交的输入行，而不是历史项。
 *       When `history_index_ < 0`, this echoes the current in-progress input
 *       line rather than one history entry.
 */
void ShowHistory()
{
  ClearLine();
  ShowHeader();
  offset_ = 0;
  if (history_index_ >= 0)
  {
    const auto& line = HistoryFromNewest(history_index_);
    write_stream_ << ConstRawData(line.data(), HistoryLineSize(line));
  }
  else
  {
    write_stream_ << ConstRawData(&input_line_[0], input_line_.Size());
  }
}

/**
 * @brief  将历史命令复制到输入行，并重置历史索引、光标偏移和截断标记
 *         Copies the history command to the input line and resets the history index,
 *         the cursor offset and the truncation mark
 */
void CopyHistoryToInputLine()
{
  input_line_.Reset();
  const auto& line = HistoryFromNewest(history_index_);
  for (size_t i = 0; i < HistoryLineSize(line); i++)
  {
    input_line_.Push(line[i]);
  }
  input_line_[input_line_.Size()] = '\0';
  history_index_ = -1;
  offset_ = 0;
  line_truncated_ = false;
}

/**
 * @brief  将当前输入行添加到历史记录
 *         Adds the current input line to the history
 *
 * @note 超过 `MAX_HISTORY_NUMBER` 时，当前实现会丢掉最旧的一条，再压入最新输入。
 *       When `MAX_HISTORY_NUMBER` is exceeded, the current implementation
 *       drops the oldest entry before pushing the newest input line.
 */
void AddHistory()
{
  HistoryLine line{};
  const size_t line_size =
      LibXR::min(static_cast<size_t>(input_line_.Size()), MAX_LINE_SIZE);
  input_line_.Push('\0');
  if (line_size > 0)
  {
    std::memcpy(line.data(), &input_line_[0], line_size);
  }
  line[line_size] = '\0';

  if (history_.EmptySize() == 0)
  {
    history_.Pop();
  }
  history_.Push(line);
}
