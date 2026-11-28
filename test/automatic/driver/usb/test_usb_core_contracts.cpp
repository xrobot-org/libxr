// USB 核心构造前置契约负向测试：验证非法配置、空类指针与超限语言包以 REQUIRE 拒绝。
// USB core construction contract tests: verify invalid configs and packs fail REQUIRE.
#include <sys/wait.h>
#include <unistd.h>

#include <cstdint>
#include <cstdio>
#include <initializer_list>

#include "desc_str.hpp"
#include "device_class.hpp"
#include "device_composition.hpp"
#include "ep_pool.hpp"
#include "libxr_def.hpp"
#include "test_assert.hpp"
#include "usb_service.hpp"

namespace
{
constexpr int EXIT_REQUIRE_NONE = 43;
constexpr int EXIT_UNEXPECTED = 44;

class StubService final : public LibXR::USB::DeviceService
{
 protected:
  void ServiceHardware(bool) override {}
  void MaskInterrupt() override {}
  void UnmaskInterrupt() override {}
};

constexpr auto TEST_PACK = LibXR::USB::DescriptorStrings::MakeLanguagePack(
    LibXR::USB::DescriptorStrings::Language::EN_US, "M", "P", "S");

void ExpectRequireFailure(void (*fn)())
{
  std::fflush(nullptr);
  const pid_t child = fork();
  TEST_ASSERT(child >= 0);
  if (child == 0)
  {
    auto cb = LibXR::Assert::FatalCallback::Create(
        [](bool in_isr, int, const char* file, uint32_t line)
        {
          const bool is_require = (!in_isr && file == nullptr && line == 0U);
          _exit(is_require ? EXIT_REQUIRE_NONE : EXIT_UNEXPECTED);
        },
        0);
    LibXR::Assert::RegisterFatalErrorCallback(cb);
    fn();
    _exit(0);
  }
  int status = 0;
  TEST_ASSERT(waitpid(child, &status, 0) == child);
  TEST_ASSERT(WIFEXITED(status) && WEXITSTATUS(status) == EXIT_REQUIRE_NONE);
}

void RejectEmptyOuterConfigs()
{
  LibXR::USB::EndpointPool pool;
  StubService svc;
  LibXR::USB::DeviceComposition comp(pool, svc, {&TEST_PACK}, {});
  (void)comp;
}

void RejectNullClassInConfig()
{
  LibXR::USB::EndpointPool pool;
  StubService svc;
  LibXR::USB::DeviceComposition comp(pool, svc, {&TEST_PACK}, {{nullptr}});
  (void)comp;
}

#define XR_REP_4(x) x, x, x, x
#define XR_REP_15(x) XR_REP_4(x), XR_REP_4(x), XR_REP_4(x), x, x, x
#define XR_REP_16(x) XR_REP_4(x), XR_REP_4(x), XR_REP_4(x), XR_REP_4(x)
#define XR_REP_48(x) XR_REP_16(x), XR_REP_16(x), XR_REP_16(x)
#define XR_REP_64(x) XR_REP_16(x), XR_REP_16(x), XR_REP_16(x), XR_REP_16(x)
#define XR_REP_127(x) XR_REP_64(x), XR_REP_48(x), XR_REP_15(x)
#define XR_REP_255(x) XR_REP_64(x), XR_REP_64(x), XR_REP_64(x), XR_REP_48(x), XR_REP_15(x)
#define XR_REP_256(x) XR_REP_64(x), XR_REP_64(x), XR_REP_64(x), XR_REP_64(x)

void RejectTooManyConfigs()
{
  LibXR::USB::EndpointPool pool;
  StubService svc;
  const std::initializer_list<LibXR::USB::DeviceClass*> empty = {};
  LibXR::USB::DeviceComposition comp(pool, svc, {&TEST_PACK}, {XR_REP_256(empty)});
  (void)comp;
}

void RejectTooManyLanguagePacks()
{
  LibXR::USB::DescriptorStrings strs({XR_REP_127(&TEST_PACK)});
  (void)strs;
}

void RejectNonNulCharArrayAtRuntime()
{
  volatile char fill = 'A';
  char bad[2] = {fill, fill};
  const auto pack = LibXR::USB::DescriptorStrings::MakeLanguagePack(
      LibXR::USB::DescriptorStrings::Language::EN_US, bad, "P", "S");
  (void)pack;
}

class FlippingInterfaceStringsClass final : public LibXR::USB::DeviceClass
{
 public:
  explicit FlippingInterfaceStringsClass(bool shrink) : shrink_(shrink) {}

 protected:
  size_t GetInterfaceCount() const override { return 2U; }
  bool HasIAD() const override { return false; }
  size_t GetMaxConfigSize() const override { return 9U; }

  LibXR::ErrorCode WriteDescriptors(const LibXR::USB::EndpointPool& pool,
                                    LibXR::USB::Speed speed, uint8_t first_interface,
                                    LibXR::ConstRawData& block) override
  {
    UNUSED(pool);
    UNUSED(speed);
    UNUSED(first_interface);
    block = {nullptr, 0U};
    return LibXR::ErrorCode::OK;
  }

  LibXR::ErrorCode BindEndpoints(LibXR::USB::EndpointPool& pool, uint8_t first_interface,
                                 bool in_isr) override
  {
    UNUSED(pool);
    UNUSED(first_interface);
    UNUSED(in_isr);
    return LibXR::ErrorCode::OK;
  }

  void UnbindEndpoints(LibXR::USB::EndpointPool& pool, bool in_isr) override
  {
    UNUSED(pool);
    UNUSED(in_isr);
  }

  const char* GetInterfaceString(size_t index) const override
  {
    UNUSED(index);
    const size_t c = calls_++;
    if (shrink_)
    {
      return (c < 3U) ? "IF" : nullptr;
    }
    return (c == 1U) ? nullptr : "IF";
  }

 private:
  bool shrink_ = false;
  mutable size_t calls_ = 0U;
};

void RejectInterfaceStringUnderfill()
{
  LibXR::USB::EndpointPool pool;
  StubService svc;
  FlippingInterfaceStringsClass cls(true);
  LibXR::USB::DeviceComposition comp(pool, svc, {&TEST_PACK}, {{&cls}});
  (void)comp;
}

void RejectInterfaceStringOverfill()
{
  LibXR::USB::EndpointPool pool;
  StubService svc;
  FlippingInterfaceStringsClass cls(false);
  LibXR::USB::DeviceComposition comp(pool, svc, {&TEST_PACK}, {{&cls}});
  (void)comp;
}

class WinUsbGuidClass final : public LibXR::USB::DeviceClass
{
 public:
  enum class Mode : uint8_t
  {
    ALWAYS,
    SHRINK,
    GROW,
  };

  explicit WinUsbGuidClass(Mode mode) : mode_(mode) {}

 protected:
  size_t GetInterfaceCount() const override { return 1U; }
  bool HasIAD() const override { return false; }
  size_t GetMaxConfigSize() const override { return 9U; }

  LibXR::ErrorCode WriteDescriptors(const LibXR::USB::EndpointPool& pool,
                                    LibXR::USB::Speed speed, uint8_t first_interface,
                                    LibXR::ConstRawData& block) override
  {
    UNUSED(pool);
    UNUSED(speed);
    UNUSED(first_interface);
    block = {nullptr, 0U};
    return LibXR::ErrorCode::OK;
  }

  LibXR::ErrorCode BindEndpoints(LibXR::USB::EndpointPool& pool, uint8_t first_interface,
                                 bool in_isr) override
  {
    UNUSED(pool);
    UNUSED(first_interface);
    UNUSED(in_isr);
    return LibXR::ErrorCode::OK;
  }

  void UnbindEndpoints(LibXR::USB::EndpointPool& pool, bool in_isr) override
  {
    UNUSED(pool);
    UNUSED(in_isr);
  }

  const char* GetWinUsbInterfaceGuid() const override
  {
    static constexpr const char* GUID = "{12345678-1234-1234-1234-1234567890AB}";
    const size_t c = calls_++;
    switch (mode_)
    {
      case Mode::ALWAYS:
        return GUID;
      case Mode::SHRINK:
        return (c == 0U) ? GUID : nullptr;
      case Mode::GROW:
        return (c == 0U) ? nullptr : GUID;
    }
    return nullptr;
  }

 private:
  Mode mode_;
  mutable size_t calls_ = 0U;
};

void RejectWinUsbUnderfill()
{
  LibXR::USB::EndpointPool pool;
  StubService svc;
  WinUsbGuidClass a(WinUsbGuidClass::Mode::ALWAYS);
  WinUsbGuidClass b(WinUsbGuidClass::Mode::SHRINK);
  LibXR::USB::DeviceComposition comp(pool, svc, {&TEST_PACK}, {{&a, &b}});
  (void)comp;
}

void RejectWinUsbOverfill()
{
  LibXR::USB::EndpointPool pool;
  StubService svc;
  WinUsbGuidClass a(WinUsbGuidClass::Mode::ALWAYS);
  WinUsbGuidClass b(WinUsbGuidClass::Mode::GROW);
  LibXR::USB::DeviceComposition comp(pool, svc, {&TEST_PACK}, {{&a, &b}});
  (void)comp;
}

void RejectWinUsbTotalOverflow()
{
  LibXR::USB::EndpointPool pool;
  StubService svc;
  WinUsbGuidClass a(WinUsbGuidClass::Mode::ALWAYS);
  WinUsbGuidClass b(WinUsbGuidClass::Mode::ALWAYS);
  const std::initializer_list<LibXR::USB::DeviceClass*> pair = {&a, &b};
  LibXR::USB::DeviceComposition comp(pool, svc, {&TEST_PACK}, {XR_REP_255(pair)});
  (void)comp;
}

#undef XR_REP_256
#undef XR_REP_255
#undef XR_REP_127
#undef XR_REP_64
#undef XR_REP_48
#undef XR_REP_16
#undef XR_REP_15
#undef XR_REP_4
}  // namespace

int main()
{
  ExpectRequireFailure(&RejectEmptyOuterConfigs);
  ExpectRequireFailure(&RejectNullClassInConfig);
  ExpectRequireFailure(&RejectTooManyConfigs);
  ExpectRequireFailure(&RejectTooManyLanguagePacks);
  ExpectRequireFailure(&RejectNonNulCharArrayAtRuntime);
  ExpectRequireFailure(&RejectInterfaceStringUnderfill);
  ExpectRequireFailure(&RejectInterfaceStringOverfill);
  ExpectRequireFailure(&RejectWinUsbUnderfill);
  ExpectRequireFailure(&RejectWinUsbOverfill);
  ExpectRequireFailure(&RejectWinUsbTotalOverflow);
  return 0;
}
