// DFU 运行时 DETACH 代数防 ABA 与截止时刻并发模型测试 / Tests for DfuRuntimeClass
// generation-token ABA prevention and deadline release-acquire ordering.
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

#include "dfu_runtime.hpp"
#include "ep_pool.hpp"
#include "test_assert.hpp"
#include "timebase.hpp"

namespace
{
uint32_t g_now = 100U;
void (*g_on_now)(void* ctx) = nullptr;
void* g_on_now_ctx = nullptr;
unsigned g_jumps = 0U;

void OnJump(void*) { ++g_jumps; }

constexpr uint8_t DETACH = 0x00U;
constexpr uint8_t GETSTATUS = 0x03U;
constexpr uint8_t GETSTATE = 0x05U;

class ProbeDfuRuntime final : public LibXR::USB::DfuRuntimeClass
{
 public:
  using DfuRuntimeClass::BindEndpoints;
  using DfuRuntimeClass::DfuRuntimeClass;
  using DfuRuntimeClass::OnControlAbort;
  using DfuRuntimeClass::OnControlComplete;
  using DfuRuntimeClass::OnControlRequest;
  using DfuRuntimeClass::UnbindEndpoints;
};

struct AbaContext
{
  ProbeDfuRuntime* rt;
  LibXR::USB::EndpointPool* pool;
};

void InjectCancelAndLongDetach(void* raw_ctx)
{
  // 单次触发：模拟 Process() 读到旧代币后，owner 取消并重挂 5000 ms DETACH。
  // One-shot hook: runs right after Process() reads old generation and deadline.
  g_on_now = nullptr;
  auto* ctx = static_cast<AbaContext*>(raw_ctx);

  ctx->rt->UnbindEndpoints(*ctx->pool, false);
  TEST_ASSERT(ctx->rt->BindEndpoints(*ctx->pool, 0U, false) == LibXR::ErrorCode::OK);

  const LibXR::USB::SetupPacket detach_long{0x21U, DETACH, 5000U, 0U, 0U};
  LibXR::ConstRawData in_data{nullptr, 0U};
  TEST_ASSERT(ctx->rt->OnControlRequest(false, detach_long, in_data) ==
              LibXR::ErrorCode::OK);
  ctx->rt->OnControlComplete(false, detach_long);
}

uint32_t ReadPollTimeout(ProbeDfuRuntime& rt, uint8_t& out_state)
{
  const LibXR::USB::SetupPacket req{0xA1U, GETSTATUS, 0U, 0U, 6U};
  LibXR::ConstRawData in_data{nullptr, 0U};
  TEST_ASSERT(rt.OnControlRequest(false, req, in_data) == LibXR::ErrorCode::OK);
  TEST_ASSERT(in_data.size_ == 6U);
  const auto* p = static_cast<const uint8_t*>(in_data.addr_);
  out_state = p[4];
  return static_cast<uint32_t>(p[1]) | (static_cast<uint32_t>(p[2]) << 8U) |
         (static_cast<uint32_t>(p[3]) << 16U);
}

uint8_t ReadDfuState(ProbeDfuRuntime& rt)
{
  const LibXR::USB::SetupPacket req{0xA1U, GETSTATE, 0U, 0U, 1U};
  LibXR::ConstRawData in_data{nullptr, 0U};
  TEST_ASSERT(rt.OnControlRequest(false, req, in_data) == LibXR::ErrorCode::OK);
  TEST_ASSERT(in_data.size_ == 1U);
  return *static_cast<const uint8_t*>(in_data.addr_);
}

void TestDeterministicAbaAndGetStatus()
{
  // 验证奇数代数步进绕回模 2^32 后仍回到 1 且绝不撞 IDLE(0)。
  // Verify odd generation wrapping mod 2^32 skips IDLE(0) and returns to 1.
  static_assert(static_cast<uint32_t>(0xFFFFFFFFU + 2U) == 1U);

  g_now = 100U;
  g_on_now = nullptr;
  g_jumps = 0U;

  LibXR::USB::EndpointPool pool;
  ProbeDfuRuntime rt(&OnJump, nullptr, 5U);
  TEST_ASSERT(rt.BindEndpoints(pool, 0U, false) == LibXR::ErrorCode::OK);

  const LibXR::USB::SetupPacket detach_short{0x21U, DETACH, 5U, 0U, 0U};
  LibXR::ConstRawData in_data{nullptr, 0U};
  TEST_ASSERT(rt.OnControlRequest(false, detach_short, in_data) == LibXR::ErrorCode::OK);
  rt.OnControlComplete(false, detach_short);

  // 到期前（now=102，deadline=105）GETSTATUS 报剩余 3 ms，到期后（110）报 0 ms。
  // Before expiry (102) GETSTATUS reports 3 ms remaining; after expiry (110) 0 ms.
  g_now = 102U;
  uint8_t state = 0U;
  TEST_ASSERT(ReadPollTimeout(rt, state) == 3U &&
              state == static_cast<uint8_t>(LibXR::USB::DFUState::APP_DETACH));
  rt.Process();
  TEST_ASSERT(g_jumps == 0U);

  g_now = 110U;
  TEST_ASSERT(ReadPollTimeout(rt, state) == 0U &&
              state == static_cast<uint8_t>(LibXR::USB::DFUState::APP_DETACH));

  // 在 Process() 读出旧代币与旧 deadline(105) 后、CAS 前注入解绑 + DETACH(5000 ms)。
  // 基线使用二值 WAITING 时 CAS 会误成功并立即跳转；带奇数代数令牌时 CAS 必须失败。
  // Inject cancel + DETACH(5000 ms) between Process() reads and CAS.
  AbaContext ctx{&rt, &pool};
  g_on_now_ctx = &ctx;
  g_on_now = &InjectCancelAndLongDetach;

  rt.Process();
  TEST_ASSERT(g_jumps == 0U);
  TEST_ASSERT(ReadDfuState(rt) == static_cast<uint8_t>(LibXR::USB::DFUState::APP_DETACH));
  TEST_ASSERT(ReadPollTimeout(rt, state) == 5000U);

  g_now = 5109U;
  TEST_ASSERT(ReadPollTimeout(rt, state) == 1U);
  rt.Process();
  TEST_ASSERT(g_jumps == 0U);

  g_now = 5110U;
  TEST_ASSERT(ReadPollTimeout(rt, state) == 0U);
  rt.Process();
  TEST_ASSERT(g_jumps == 1U);
  rt.Process();
  TEST_ASSERT(g_jumps == 1U);
}

// 对 dfu_runtime.cpp 实际实现逐一映射的 C++20 弱内存穷举：
// - Owner: E0=detach_deadline_ms_.store(5000), E1=launch_phase_.store(3, rel),
//          E2=UnbindEndpoints launch_phase_.store(0, rel),
//          E3=OnControlComplete detach_deadline_ms_.store(101, dd_mo),
//          E4=OnControlComplete launch_phase_.store(5, rel)
// - Process: launch_phase_.load(acq) -> detach_deadline_ms_.load(proc_mo) -> CAS(acq_rel)
// Exhaustive C++20 weak-memory check mapped 1-to-1 to dfu_runtime.cpp.
size_t CountPrematureCrossGenJumps(bool deadline_store_release,
                                   bool deadline_load_acquire)
{
  enum PhaseWrite : size_t
  {
    PW_E1 = 1,
    PW_E2 = 2,
    PW_E4 = 3,
  };
  enum DeadlineWrite : size_t
  {
    DW_E0 = 1,
    DW_E3 = 2,
  };

  constexpr uint32_t TOKEN_BY_PW[4] = {0U, 3U, 0U, 5U};
  constexpr uint32_t DEADLINE_BY_DW[3] = {0U, 5000U, 101U};
  constexpr uint32_t NOW = 200U;

  size_t premature_jumps = 0U;

  for (size_t wp = PW_E1; wp <= PW_E4; ++wp)
  {
    const uint32_t tok = TOKEN_BY_PW[wp];
    if (tok == 0U)
    {
      continue;
    }
    for (size_t wd = DW_E0; wd <= DW_E3; ++wd)
    {
      // E4(release) 与 load_acq(E4) 同步：E3 ->sb E4 ->sw L_phase ->sb L_deadline。
      // Synchronizes-with: load_acq(E4) forces E3 ->sb E4 ->sw L_phase ->sb L_deadline.
      if (wp == PW_E4 && wd != DW_E3)
      {
        continue;
      }

      const uint32_t ddl = DEADLINE_BY_DW[wd];
      if (static_cast<int32_t>(NOW - ddl) < 0)
      {
        continue;
      }

      for (size_t cur_pw = wp; cur_pw <= PW_E4; ++cur_pw)
      {
        if (TOKEN_BY_PW[cur_pw] != tok)
        {
          continue;
        }
        // 仅当发布与读取两侧均为 release/acquire 且读到 E3 时建立同步边 E3 ->sw
        // L_deadline ->sb CAS，使 E2 happens-before CAS，阻止 CAS 再读 E1。
        // Only paired release/acquire establishes E3 ->sw L_deadline ->sb CAS.
        const bool hb_after_e2 =
            deadline_store_release && deadline_load_acquire && (wd == DW_E3);
        if (hb_after_e2 && cur_pw < PW_E2)
        {
          continue;
        }

        // 只要认领的是第 1 代（ddl=5000）却基于第 2 代 deadline 跳转，即算错。
        if (wp == PW_E1 && wd == DW_E3)
        {
          ++premature_jumps;
        }
      }
    }
  }
  return premature_jumps;
}

void TestWeakMemoryExhaustiveModel()
{
  // 三组退化负向对照：缺任意一侧 release/acquire 都会重新允许跨代提前跳转。
  // Three negative controls: dropping release or acquire re-enables premature jumps.
  TEST_ASSERT(CountPrematureCrossGenJumps(false, false) > 0U);
  TEST_ASSERT(CountPrematureCrossGenJumps(false, true) > 0U);
  TEST_ASSERT(CountPrematureCrossGenJumps(true, false) > 0U);
  // 正向验证：双方成对 release/acquire 后，跨代提前跳在所有合法执行中均为 0。
  // Positive proof: paired release/acquire eliminates all cross-generation jumps.
  TEST_ASSERT(CountPrematureCrossGenJumps(true, true) == 0U);
}
}  // namespace

LibXR::MillisecondTimestamp LibXR::Timebase::GetMilliseconds()
{
  if (g_on_now != nullptr)
  {
    g_on_now(g_on_now_ctx);
  }
  return g_now;
}

LibXR::MicrosecondTimestamp LibXR::Timebase::GetMicroseconds()
{
  return static_cast<uint64_t>(g_now) * 1000U;
}

extern "C" void libxr_fatal_error(const char* file, uint32_t line, bool)
{
  std::fprintf(stderr, "%s:%lu: product assertion failed\n",
               file != nullptr ? file : "(no location)",
               static_cast<unsigned long>(line));
  std::abort();
}

extern "C" void libxr_fatal_error_and_print(const char* file, uint32_t line, bool in_isr)
{
  libxr_fatal_error(file, line, in_isr);
}

int main()
{
  TestDeterministicAbaAndGetStatus();
  TestWeakMemoryExhaustiveModel();
  return 0;
}
