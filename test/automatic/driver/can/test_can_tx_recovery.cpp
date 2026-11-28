/**
 * @file test_can_tx_recovery.cpp
 * @brief 生产 STM32CANFD / STM32CAN / CH32CAN 发送串行化与恢复路径主机回归 /
 *        Host regression tests exercising real production STM32CANFD, STM32CAN, and
 *        CH32CAN TX serialization, config validation, and recovery paths.
 */

#include <cstdio>

void RunStm32CanfdProdTests();
void RunStm32CanProdTests();
void RunCh32CanProdTests();

int main()
{
  RunStm32CanfdProdTests();
  RunStm32CanProdTests();
  RunCh32CanProdTests();
  std::puts("can_tx_recovery: production STM32CANFD, STM32CAN, and CH32CAN tests passed");
  return 0;
}
