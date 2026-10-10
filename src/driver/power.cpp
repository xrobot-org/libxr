#include "power.hpp"

#include <cstring>

using namespace LibXR;

namespace
{

/**
 * @brief power 命令的执行函数 / Executes the power command
 *
 * argv[0] 是命令名，argv[1] 选择操作。
 * argv[0] is the command name and argv[1] selects the action.
 */
int PowerCommand(PowerManager* power, int argc, char** argv)
{
  if (argc == 2 && std::strcmp(argv[1], "reset") == 0)
  {
    power->Reset();
    return 0;
  }
  if (argc == 2 && std::strcmp(argv[1], "shutdown") == 0)
  {
    power->Shutdown();
    return 0;
  }
  if (argc == 2 && std::strcmp(argv[1], "bootloader") == 0)
  {
    power->JumpToBootloader();
    return 0;
  }
  STDIO::Print<"usage: power reset|shutdown|bootloader\r\n">();
  return -1;
}

}  // namespace

void PowerManager::RegisterCommand(RamFS& ramfs)
{
  auto* command = new RamFS::File(RamFS::CreateCommand("power", PowerCommand, this));
  ramfs.Add(*command);
}

void PowerManager::CheckBootloaderPin(GPIO& pin, bool level)
{
  if (pin.Read() == level)
  {
    JumpToBootloader();
  }
}
