/**
 * @file test_power.cpp
 * @brief PowerManager 命令与启动引脚测试 / PowerManager command and boot pin tests.
 *
 * 检查 power 命令按参数调用 Reset、Shutdown、JumpToBootloader，
 * 参数错误时返回 -1 且不调用；检查 CheckBootloaderPin 只在引脚电平匹配时
 * 进入启动加载器。
 * Check that the power command calls Reset, Shutdown and JumpToBootloader by its
 * argument and returns -1 without a call on a bad argument, and that
 * CheckBootloaderPin enters the bootloader only on a matching pin level.
 */

#include "gpio.hpp"
#include "libxr.hpp"
#include "power.hpp"
#include "test_assert.hpp"

namespace
{

class CountingPower : public LibXR::PowerManager
{
 public:
  void Reset() override { resets++; }
  void Shutdown() override { shutdowns++; }
  void JumpToBootloader() override { bootloaders++; }

  int resets = 0;
  int shutdowns = 0;
  int bootloaders = 0;
};

class FixedGPIO : public LibXR::GPIO
{
 public:
  explicit FixedGPIO(bool level) : level_(level) {}

  bool Read() override { return level_; }
  void Write(bool) override {}
  LibXR::ErrorCode EnableInterrupt() override { return LibXR::ErrorCode::OK; }
  LibXR::ErrorCode DisableInterrupt() override { return LibXR::ErrorCode::OK; }
  LibXR::ErrorCode SetConfig(Configuration) override { return LibXR::ErrorCode::OK; }

 private:
  bool level_;
};

int Run(LibXR::RamFS::File& command, const char* arg)
{
  char name[] = "power";
  char buffer[16] = {};
  char* argv[2] = {name, buffer};
  if (arg == nullptr)
  {
    return command.Run(1, argv);
  }
  for (int i = 0; arg[i] != '\0' && i < 15; i++)
  {
    buffer[i] = arg[i];
  }
  return command.Run(2, argv);
}

}  // namespace

void test_power()
{
  LibXR::RamFS ramfs("test");
  CountingPower power;
  power.RegisterCommand(ramfs);

  auto* command = ramfs.FindFile("power");
  TEST_ASSERT(command != nullptr);
  TEST_ASSERT(command->IsExecutable());

  TEST_ASSERT(Run(*command, "reset") == 0);
  TEST_ASSERT(power.resets == 1);
  TEST_ASSERT(Run(*command, "shutdown") == 0);
  TEST_ASSERT(power.shutdowns == 1);
  TEST_ASSERT(Run(*command, "bootloader") == 0);
  TEST_ASSERT(power.bootloaders == 1);

  TEST_ASSERT(Run(*command, nullptr) == -1);
  TEST_ASSERT(Run(*command, "reboot") == -1);
  TEST_ASSERT(power.resets == 1 && power.shutdowns == 1 && power.bootloaders == 1);

  FixedGPIO low(false);
  FixedGPIO high(true);
  power.CheckBootloaderPin(high, false);
  TEST_ASSERT(power.bootloaders == 1);
  power.CheckBootloaderPin(low, false);
  TEST_ASSERT(power.bootloaders == 2);
  power.CheckBootloaderPin(high, true);
  TEST_ASSERT(power.bootloaders == 3);
}
