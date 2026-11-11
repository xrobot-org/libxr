/**
 * @file lsan_suppressions.hpp
 * @brief 两个 DWC2 测试在 LeakSanitizer 下的抑制规则 / LeakSanitizer suppressions of the
 *        two DWC2 tests.
 *
 * 后端和 DeviceCore 在构造和初始化时用 new 创建端点对象、描述符和回调块，库里没有释放
 * 它们的接口（初始化期分配、长期持有，见仓库根目录 AGENTS.md 的"内存管理"和
 * `Callback::Create` 的说明），所以测试在用例结束时也没法释放，LeakSanitizer 会把它们当
 * 成泄漏。下面只抑制这几个分配点；测试自己的对象（控制器模型、主机脚本、探针类）在用例
 * 结束时释放，不在抑制之列，它们泄漏仍然会被报告。
 * The backend and DeviceCore create endpoint objects, descriptors and callback blocks
 * with new at construction and initialization, and the library has no interface that
 * frees them (they are allocated once at initialization and kept, see "Memory
 * management" in AGENTS.md at the repository root and the note of `Callback::Create`),
 * so the test cannot free them at the end of a case either and LeakSanitizer reports
 * them. Only these allocation sites are suppressed; the objects of the test itself (the
 * controller model, the host script, the probe class) are released at the end of a case
 * and are not covered, so a leak of theirs is still reported.
 *
 * 不是 LeakSanitizer 的构建里这个函数没有用到。
 * In a build without LeakSanitizer the function is never called.
 */

#pragma once

extern "C" __attribute__((used)) const char* __lsan_default_suppressions()
{
  return
      // STM32 backend: endpoint objects, DeviceCore, descriptors, callbacks.
      "leak:LibXR::STM32USBOtgDevice::STM32USBOtgDevice\n"
      // The descriptors DeviceCore builds when it is initialized.
      "leak:LibXR::USB::DeviceComposition::PrepareDescriptors\n"
      // Callback blocks, created once and kept.
      "leak:LibXR::Callback<*>::Create\n";
}
