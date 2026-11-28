#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

#include "ch32_can.hpp"
#include "test_assert.hpp"

extern "C"
{
  CAN_TypeDef g_ch32_can1_regs{};
  CAN_TypeDef g_ch32_can2_regs{};
}

namespace
{
struct Ch32CanStubState
{
  std::mutex mtx;
  uint32_t mailbox_mask = 0U;
  bool force_next_tx_fail = false;
  bool normal_mode = false;
  bool keep_tme_in_init = false;
  bool tme_pending = false;
  bool fail_init_mode = false;
  bool fail_can_init = false;
  uint32_t fail_normal_count = 0U;
  uint32_t disable_irq_calls = 0U;
  uint32_t enable_irq_calls = 0U;
  uint32_t mode_req_calls = 0U;
  uint32_t clear_tme_calls = 0U;
  uint32_t tx_attempt_calls = 0U;
  std::function<void()> on_init_mode_hook;
  std::function<void()> on_zero_tstatr_hook;
  std::atomic<uint32_t> active_tx_calls{0U};
  std::vector<uint32_t> transmitted_ids;

  void UpdateTstatr(CAN_TypeDef* regs) const
  {
    uint32_t t = 0U;
    if (normal_mode || keep_tme_in_init)
    {
      if ((mailbox_mask & 1U) == 0U)
      {
        t |= CAN_TSTATR_TME0;
      }
      if ((mailbox_mask & 2U) == 0U)
      {
        t |= CAN_TSTATR_TME1;
      }
      if ((mailbox_mask & 4U) == 0U)
      {
        t |= CAN_TSTATR_TME2;
      }
      if (force_next_tx_fail)
      {
        t |= CAN_TSTATR_TME0;
      }
    }
    regs->TSTATR = t;
  }

  void Reset(CAN_TypeDef* regs)
  {
    std::lock_guard<std::mutex> lk(mtx);
    mailbox_mask = 0U;
    force_next_tx_fail = false;
    normal_mode = false;
    keep_tme_in_init = false;
    tme_pending = false;
    fail_init_mode = false;
    fail_can_init = false;
    fail_normal_count = 0U;
    disable_irq_calls = 0U;
    enable_irq_calls = 0U;
    mode_req_calls = 0U;
    clear_tme_calls = 0U;
    tx_attempt_calls = 0U;
    on_init_mode_hook = nullptr;
    on_zero_tstatr_hook = nullptr;
    regs->TSTATR.on_zero_read.store(nullptr, std::memory_order_relaxed);
    active_tx_calls.store(0U, std::memory_order_relaxed);
    transmitted_ids.clear();
    UpdateTstatr(regs);
  }
};

Ch32CanStubState g_ch32_state;

void DispatchCh32ZeroTstatrHook()
{
  std::function<void()> hook;
  {
    std::lock_guard<std::mutex> lk(g_ch32_state.mtx);
    hook = std::move(g_ch32_state.on_zero_tstatr_hook);
  }
  if (hook)
  {
    hook();
  }
}
}  // namespace

extern "C"
{
  void NVIC_EnableIRQ(IRQn_Type) {}
  void RCC_APB1PeriphClockCmd(uint32_t, FunctionalState) {}
  void RCC_GetClocksFreq(RCC_ClocksTypeDef* RCC_Clocks)
  {
    RCC_Clocks->PCLK1_Frequency = 36000000U;
  }

  uint8_t CAN_OperatingModeRequest(CAN_TypeDef* CANx, uint8_t CAN_OperatingMode)
  {
    std::function<void()> hook;
    uint8_t status = CAN_ModeStatus_Success;
    {
      std::lock_guard<std::mutex> lk(g_ch32_state.mtx);
      ++g_ch32_state.mode_req_calls;
      if (CAN_OperatingMode == CAN_OperatingMode_Normal)
      {
        if (g_ch32_state.fail_normal_count > 0U)
        {
          --g_ch32_state.fail_normal_count;
          status = CAN_ModeStatus_Failed;
        }
        else
        {
          g_ch32_state.normal_mode = true;
          g_ch32_state.mailbox_mask = 0U;
          g_ch32_state.force_next_tx_fail = false;
        }
      }
      else
      {
        if (g_ch32_state.fail_init_mode)
        {
          status = CAN_ModeStatus_Failed;
        }
        else
        {
          g_ch32_state.normal_mode = false;
          hook = std::move(g_ch32_state.on_init_mode_hook);
        }
      }
      g_ch32_state.UpdateTstatr(CANx);
    }
    if (hook)
    {
      hook();
    }
    return status;
  }

  void CAN_SlaveStartBank(uint8_t) {}
  void CAN_FilterInit(CAN_FilterInitTypeDef*) {}

  void CAN_ITConfig(CAN_TypeDef*, uint32_t, FunctionalState NewState)
  {
    std::lock_guard<std::mutex> lk(g_ch32_state.mtx);
    if (NewState == DISABLE)
    {
      ++g_ch32_state.disable_irq_calls;
    }
    else
    {
      ++g_ch32_state.enable_irq_calls;
    }
  }

  void CAN_ClearITPendingBit(CAN_TypeDef*, uint32_t CAN_IT)
  {
    std::lock_guard<std::mutex> lk(g_ch32_state.mtx);
    if ((CAN_IT & CAN_IT_TME) != 0U)
    {
      g_ch32_state.tme_pending = false;
      ++g_ch32_state.clear_tme_calls;
    }
  }

  ITStatus CAN_GetITStatus(CAN_TypeDef*, uint32_t CAN_IT)
  {
    std::lock_guard<std::mutex> lk(g_ch32_state.mtx);
    if ((CAN_IT & CAN_IT_TME) != 0U)
    {
      return g_ch32_state.tme_pending ? SET : RESET;
    }
    return RESET;
  }

  void CAN_StructInit(CAN_InitTypeDef*) {}

  uint8_t CAN_Init(CAN_TypeDef*, CAN_InitTypeDef*)
  {
    std::lock_guard<std::mutex> lk(g_ch32_state.mtx);
    return g_ch32_state.fail_can_init ? CAN_InitStatus_Failed : CAN_InitStatus_Success;
  }

  uint8_t CAN_Transmit(CAN_TypeDef* CANx, CanTxMsg* TxMessage)
  {
    const uint32_t prev =
        g_ch32_state.active_tx_calls.fetch_add(1U, std::memory_order_acq_rel);
    TEST_ASSERT(prev == 0U);

    uint8_t mb_ret = CAN_TxStatus_NoMailBox;
    {
      std::lock_guard<std::mutex> lk(g_ch32_state.mtx);
      ++g_ch32_state.tx_attempt_calls;
      if (!g_ch32_state.normal_mode || g_ch32_state.force_next_tx_fail)
      {
        g_ch32_state.force_next_tx_fail = false;
        g_ch32_state.UpdateTstatr(CANx);
        mb_ret = CAN_TxStatus_NoMailBox;
      }
      else
      {
        for (uint8_t mb = 0U; mb < 3U; ++mb)
        {
          if ((g_ch32_state.mailbox_mask & (1U << mb)) == 0U)
          {
            g_ch32_state.mailbox_mask |= (1U << mb);
            g_ch32_state.transmitted_ids.push_back(
                TxMessage->IDE == CAN_ID_EXT ? TxMessage->ExtId : TxMessage->StdId);
            g_ch32_state.UpdateTstatr(CANx);
            mb_ret = mb;
            break;
          }
        }
      }
    }

    const uint32_t after =
        g_ch32_state.active_tx_calls.fetch_sub(1U, std::memory_order_acq_rel);
    TEST_ASSERT(after == 1U);
    return mb_ret;
  }

  uint8_t CAN_MessagePending(CAN_TypeDef*, uint8_t) { return 0U; }
  void CAN_Receive(CAN_TypeDef*, uint8_t, CanRxMsg*) {}
  FlagStatus CAN_GetFlagStatus(CAN_TypeDef*, uint32_t) { return RESET; }
  uint8_t CAN_GetLastErrorCode(CAN_TypeDef*) { return 0U; }
  uint8_t CAN_GetReceiveErrorCounter(CAN_TypeDef*) { return 0U; }
  uint8_t CAN_GetLSBTransmitErrorCounter(CAN_TypeDef*) { return 0U; }
}

void RunCh32CanProdTests()
{
  using namespace LibXR;

  g_ch32_can1_regs = {};
  g_ch32_can2_regs = {};
  g_ch32_state.Reset(CAN1);

  CH32CAN can(CH32_CAN1, 4U);

  // 1. 验证非法位时序参数返回 ARG_ERR 且不切换控制器工作模式（保留基线 IrqGuard 行为）。
  // Invalid timing returns ARG_ERR without switching controller mode.
  {
    auto assert_arg_err_no_mode_switch = [&](const CAN::Configuration& bad)
    {
      const uint32_t mr0 = g_ch32_state.mode_req_calls;
      TEST_ASSERT(can.SetConfig(bad) == ErrorCode::ARG_ERR);
      TEST_ASSERT(g_ch32_state.mode_req_calls == mr0);
    };

    CAN::Configuration bad{};
    assert_arg_err_no_mode_switch(bad);  // uninitialized cache

    bad.bit_timing = {1U, 10U, 8U, 2U, 1U};
    assert_arg_err_no_mode_switch(bad);  // BS1 > 16

    bad.bit_timing = {1U, 4U, 4U, 2U, 3U};
    assert_arg_err_no_mode_switch(bad);  // SJW > BS2
  }

  CAN::Configuration valid_cfg{};
  valid_cfg.bit_timing = {4U, 3U, 4U, 2U, 1U};
  TEST_ASSERT(can.SetConfig(valid_cfg) == ErrorCode::OK);

  auto make_pack = [](uint32_t id)
  {
    CAN::ClassicPack p{};
    p.id = id;
    p.type = CAN::Type::STANDARD;
    p.dlc = 4U;
    return p;
  };

  // 2. 填满 3 个硬件邮箱，触发 retry 槽与 FULL，通过真实共享向量
  // USB_HP_CAN1_TX_IRQHandler -> ProcessTxInterrupt 排空。
  // Saturate 3 HW mailboxes, trigger retry & FULL, then drain via real shared IRQ
  // vector USB_HP_CAN1_TX_IRQHandler -> ProcessTxInterrupt.
  TEST_ASSERT(can.AddMessage(make_pack(0x30U)) == ErrorCode::OK);
  TEST_ASSERT(can.AddMessage(make_pack(0x31U)) == ErrorCode::OK);
  TEST_ASSERT(can.AddMessage(make_pack(0x32U)) == ErrorCode::OK);
  TEST_ASSERT(g_ch32_state.transmitted_ids.size() == 3U);

  {
    std::lock_guard<std::mutex> lk(g_ch32_state.mtx);
    g_ch32_state.force_next_tx_fail = true;
    g_ch32_state.UpdateTstatr(CAN1);
  }
  TEST_ASSERT(can.AddMessage(make_pack(0x33U)) == ErrorCode::OK);  // captured in retry
  TEST_ASSERT(can.AddMessage(make_pack(0x34U)) == ErrorCode::OK);
  TEST_ASSERT(can.AddMessage(make_pack(0x35U)) == ErrorCode::OK);
  TEST_ASSERT(can.AddMessage(make_pack(0x36U)) == ErrorCode::OK);
  TEST_ASSERT(can.AddMessage(make_pack(0x37U)) == ErrorCode::OK);
  TEST_ASSERT(can.AddMessage(make_pack(0x38U)) == ErrorCode::FULL);

  for (uint32_t i = 0U; i < 6U; ++i)
  {
    {
      std::lock_guard<std::mutex> lk(g_ch32_state.mtx);
      g_ch32_state.mailbox_mask &= ~(1U << (i % 3U));
      g_ch32_state.tme_pending = true;
      g_ch32_state.UpdateTstatr(CAN1);
    }
    USB_HP_CAN1_TX_IRQHandler();
  }

  TEST_ASSERT(g_ch32_state.transmitted_ids.size() == 8U);
  for (uint32_t i = 0U; i < 8U; ++i)
  {
    TEST_ASSERT(g_ch32_state.transmitted_ids[i] == 0x30U + i);
  }

  // 3. 停机重配 SetConfig() 恢复：EnableIRQs() 会清除 pending TME 标志，
  // SetConfig() 末尾的 TxService() 必须在无新 AddMessage() 与无 TME IRQ
  // 时续发全部滞留帧。 SetConfig() recovery: EnableIRQs() clears pending TME; post-guard
  // TxService() must resume all stranded retry + queued frames with ZERO new AddMessage()
  // and ZERO TX IRQ.
  {
    std::lock_guard<std::mutex> lk(g_ch32_state.mtx);
    g_ch32_state.transmitted_ids.clear();
    g_ch32_state.mailbox_mask = 0x7U;
    g_ch32_state.force_next_tx_fail = true;
    g_ch32_state.tme_pending = true;
    g_ch32_state.UpdateTstatr(CAN1);
  }
  TEST_ASSERT(can.AddMessage(make_pack(0x40U)) == ErrorCode::OK);  // retry slot
  TEST_ASSERT(can.AddMessage(make_pack(0x41U)) == ErrorCode::OK);  // software queue
  TEST_ASSERT(g_ch32_state.transmitted_ids.empty());

  const uint32_t clear_tme_before = g_ch32_state.clear_tme_calls;
  CAN::Configuration keep_cfg{};  // all zero -> reuse cached valid_cfg
  TEST_ASSERT(can.SetConfig(keep_cfg) == ErrorCode::OK);
  TEST_ASSERT(g_ch32_state.clear_tme_calls > clear_tme_before);
  TEST_ASSERT(!g_ch32_state.tme_pending);
  TEST_ASSERT(g_ch32_state.transmitted_ids.size() == 2U);
  TEST_ASSERT(g_ch32_state.transmitted_ids[0] == 0x40U);
  TEST_ASSERT(g_ch32_state.transmitted_ids[1] == 0x41U);

  // 3b. 停机窗口确定性注入：在进入 Initialization 模式窗口内入队，
  // 尽管 EnableIRQs() 清除 TME，SetConfig() 返回前仍自动续发。
  // Stop-window deterministic injection during CAN_OperatingMode_Initialization.
  {
    uint32_t init_hook_ran = 0U;
    {
      std::lock_guard<std::mutex> lk(g_ch32_state.mtx);
      g_ch32_state.transmitted_ids.clear();
      g_ch32_state.mailbox_mask = 0U;
      g_ch32_state.UpdateTstatr(CAN1);
      g_ch32_state.on_init_mode_hook = [&]
      {
        ++init_hook_ran;
        TEST_ASSERT(can.AddMessage(make_pack(0x45U)) == ErrorCode::OK);
        TEST_ASSERT(g_ch32_state.transmitted_ids.empty());
      };
    }
    TEST_ASSERT(can.SetConfig(keep_cfg) == ErrorCode::OK);
    TEST_ASSERT(init_hook_ran == 1U);
    TEST_ASSERT(g_ch32_state.transmitted_ids.size() == 1U);
    TEST_ASSERT(g_ch32_state.transmitted_ids[0] == 0x45U);
  }

  // 3c. Owner 退出窗口确定性注入：填满 3 个邮箱后读 TSTATR==0 准备退出循环瞬间释放邮箱
  // 并触发重入 AddMessage + USB_HP_CAN1_TX_IRQHandler，须由 SerializedService 重检排空。
  // Owner-exit-window deterministic injection via TSTATR==0 read hook.
  {
    {
      std::lock_guard<std::mutex> lk(g_ch32_state.mtx);
      g_ch32_state.transmitted_ids.clear();
      g_ch32_state.mailbox_mask = 0U;
      g_ch32_state.UpdateTstatr(CAN1);
    }
    TEST_ASSERT(can.AddMessage(make_pack(0x60U)) == ErrorCode::OK);
    TEST_ASSERT(can.AddMessage(make_pack(0x61U)) == ErrorCode::OK);
    TEST_ASSERT(g_ch32_state.transmitted_ids.size() == 2U);

    uint32_t exit_hook_ran = 0U;
    {
      std::lock_guard<std::mutex> lk(g_ch32_state.mtx);
      g_ch32_state.on_zero_tstatr_hook = [&]
      {
        ++exit_hook_ran;
        {
          std::lock_guard<std::mutex> inner(g_ch32_state.mtx);
          TEST_ASSERT(g_ch32_state.transmitted_ids.size() == 3U);
          g_ch32_state.mailbox_mask &= ~1U;
          g_ch32_state.tme_pending = true;
          g_ch32_state.UpdateTstatr(CAN1);
        }
        TEST_ASSERT(can.AddMessage(make_pack(0x63U)) == ErrorCode::OK);
        USB_HP_CAN1_TX_IRQHandler();
        {
          std::lock_guard<std::mutex> inner(g_ch32_state.mtx);
          TEST_ASSERT(g_ch32_state.transmitted_ids.size() == 3U);
        }
      };
    }
    CAN1->TSTATR.on_zero_read.store(&DispatchCh32ZeroTstatrHook,
                                    std::memory_order_release);

    TEST_ASSERT(can.AddMessage(make_pack(0x62U)) == ErrorCode::OK);
    TEST_ASSERT(exit_hook_ran == 1U);
    TEST_ASSERT(g_ch32_state.transmitted_ids.size() == 4U);
    TEST_ASSERT(g_ch32_state.transmitted_ids[0] == 0x60U);
    TEST_ASSERT(g_ch32_state.transmitted_ids[1] == 0x61U);
    TEST_ASSERT(g_ch32_state.transmitted_ids[2] == 0x62U);
    TEST_ASSERT(g_ch32_state.transmitted_ids[3] == 0x63U);
  }

  // 3d. 模式切换故障注入（含真实 Init 态 TME 置位负控与跨调用滞留 Init 态检查）：
  // 滞留 Init 模式时即使 TSTATR.TME 置位也不得调用 TxService()，且后续 ARG_ERR 同样
  // 不得误判为 Normal；确认回到 Normal 后须自动续发。
  // Mode-switch failure injection with non-tautological Init-mode TME=1 negative control.
  {
    // (A) CAN_Init 失败且恢复 Normal 失败 -> 停留在 Init 模式，即使 TSTATR.TME0..2 置位
    // 也不得触发 CAN_Transmit（若删除 controller_normal_ 门控，此处将立即失败）。
    {
      std::lock_guard<std::mutex> lk(g_ch32_state.mtx);
      g_ch32_state.transmitted_ids.clear();
      g_ch32_state.mailbox_mask = 0x7U;
      g_ch32_state.UpdateTstatr(CAN1);
    }
    TEST_ASSERT(can.AddMessage(make_pack(0x70U)) == ErrorCode::OK);
    TEST_ASSERT(g_ch32_state.transmitted_ids.empty());
    {
      std::lock_guard<std::mutex> lk(g_ch32_state.mtx);
      g_ch32_state.keep_tme_in_init = true;
      g_ch32_state.mailbox_mask = 0U;  // TSTATR.TME0..2 置位 / TME bits set even in Init
      g_ch32_state.fail_can_init = true;
      g_ch32_state.fail_normal_count = 1U;
      g_ch32_state.tx_attempt_calls = 0U;
      g_ch32_state.UpdateTstatr(CAN1);
    }
    TEST_ASSERT(can.SetConfig(keep_cfg) == ErrorCode::FAILED);
    TEST_ASSERT(!g_ch32_state.normal_mode);
    TEST_ASSERT((static_cast<uint32_t>(CAN1->TSTATR) &
                 (CAN_TSTATR_TME0 | CAN_TSTATR_TME1 | CAN_TSTATR_TME2)) != 0U);
    TEST_ASSERT(g_ch32_state.tx_attempt_calls == 0U);
    TEST_ASSERT(g_ch32_state.transmitted_ids.empty());

    // (A2) 控制器仍停在 Init 模式时，下一次非法配置返回 ARG_ERR 且同样不得触发
    // TxService()。
    CAN::Configuration bad_while_init{};
    bad_while_init.bit_timing = {1U, 10U, 8U, 2U, 1U};  // BS1 > 16
    TEST_ASSERT(can.SetConfig(bad_while_init) == ErrorCode::ARG_ERR);
    TEST_ASSERT(!g_ch32_state.normal_mode);
    TEST_ASSERT(g_ch32_state.tx_attempt_calls == 0U);
    TEST_ASSERT(g_ch32_state.transmitted_ids.empty());

    // (B) CAN_Init 失败但恢复 Normal 成功 -> 返回 FAILED 且续发滞留帧 0x70U。
    {
      std::lock_guard<std::mutex> lk(g_ch32_state.mtx);
      g_ch32_state.keep_tme_in_init = false;
      g_ch32_state.fail_can_init = true;
      g_ch32_state.fail_normal_count = 0U;
      g_ch32_state.tx_attempt_calls = 0U;
    }
    TEST_ASSERT(can.SetConfig(keep_cfg) == ErrorCode::FAILED);
    TEST_ASSERT(g_ch32_state.normal_mode);
    TEST_ASSERT(g_ch32_state.tx_attempt_calls == 1U);
    TEST_ASSERT(g_ch32_state.transmitted_ids.size() == 1U);
    TEST_ASSERT(g_ch32_state.transmitted_ids[0] == 0x70U);

    // (C) Normal 态非法配置返回 ARG_ERR：即使软件队列有待发帧且邮箱空闲，
    // 也不得在 ARG_ERR 路径附加发送动作（与基线一致）。
    {
      std::lock_guard<std::mutex> lk(g_ch32_state.mtx);
      g_ch32_state.transmitted_ids.clear();
      g_ch32_state.fail_can_init = false;
      g_ch32_state.mailbox_mask = 0x7U;
      g_ch32_state.UpdateTstatr(CAN1);
    }
    TEST_ASSERT(can.AddMessage(make_pack(0x71U)) == ErrorCode::OK);
    {
      std::lock_guard<std::mutex> lk(g_ch32_state.mtx);
      g_ch32_state.mailbox_mask = 0U;
      g_ch32_state.tx_attempt_calls = 0U;
      g_ch32_state.UpdateTstatr(CAN1);
    }
    TEST_ASSERT(can.SetConfig(bad_while_init) == ErrorCode::ARG_ERR);
    TEST_ASSERT(g_ch32_state.normal_mode);
    TEST_ASSERT(g_ch32_state.tx_attempt_calls == 0U);
    TEST_ASSERT(g_ch32_state.transmitted_ids.empty());

    // (D) 进入 Init 模式超时失败（真实 WCH SDK 已置位 INRQ）-> 返回 FAILED 且不得调用
    // TxService()（即使 TSTATR.TME 置位）；随后成功 SetConfig() 才续发滞留帧 0x71U。
    {
      std::lock_guard<std::mutex> lk(g_ch32_state.mtx);
      g_ch32_state.fail_init_mode = true;
      g_ch32_state.keep_tme_in_init = true;
      g_ch32_state.tx_attempt_calls = 0U;
      g_ch32_state.UpdateTstatr(CAN1);
    }
    TEST_ASSERT(can.SetConfig(keep_cfg) == ErrorCode::FAILED);
    TEST_ASSERT(g_ch32_state.tx_attempt_calls == 0U);
    TEST_ASSERT(g_ch32_state.transmitted_ids.empty());

    // 恢复正常桩配置后，一次成功 SetConfig() 恢复 Normal 并续发 0x71U。
    {
      std::lock_guard<std::mutex> lk(g_ch32_state.mtx);
      g_ch32_state.keep_tme_in_init = false;
      g_ch32_state.fail_init_mode = false;
      g_ch32_state.fail_can_init = false;
      g_ch32_state.fail_normal_count = 0U;
    }
    TEST_ASSERT(can.SetConfig(keep_cfg) == ErrorCode::OK);
    TEST_ASSERT(g_ch32_state.transmitted_ids.size() == 1U);
    TEST_ASSERT(g_ch32_state.transmitted_ids[0] == 0x71U);
  }

  // 4. 多线程并发 AddMessage() 与 ProcessTxInterrupt() 压力测试。
  // Multi-thread stress: concurrent AddMessage() vs ProcessTxInterrupt().
  {
    {
      std::lock_guard<std::mutex> lk(g_ch32_state.mtx);
      g_ch32_state.transmitted_ids.clear();
      g_ch32_state.mailbox_mask = 0U;
      g_ch32_state.UpdateTstatr(CAN1);
    }

    constexpr uint32_t TOTAL = 250U;
    std::atomic<uint32_t> prod_done{0U};
    std::thread irq_thread(
        [&]
        {
          uint32_t mb = 0U;
          for (;;)
          {
            bool freed = false;
            {
              std::lock_guard<std::mutex> lk(g_ch32_state.mtx);
              if ((g_ch32_state.mailbox_mask & (1U << mb)) != 0U)
              {
                g_ch32_state.mailbox_mask &= ~(1U << mb);
                g_ch32_state.tme_pending = true;
                g_ch32_state.UpdateTstatr(CAN1);
                freed = true;
              }
              else if (prod_done.load(std::memory_order_acquire) != 0U &&
                       g_ch32_state.transmitted_ids.size() >= TOTAL)
              {
                break;
              }
            }
            if (freed)
            {
              USB_HP_CAN1_TX_IRQHandler();
            }
            else
            {
              std::this_thread::yield();
            }
            mb = (mb + 1U) % 3U;
          }
        });

    for (uint32_t i = 0U; i < TOTAL; ++i)
    {
      while (can.AddMessage(make_pack(0x200U + i)) == ErrorCode::FULL)
      {
        std::this_thread::yield();
      }
    }
    prod_done.store(1U, std::memory_order_release);
    irq_thread.join();

    TEST_ASSERT(g_ch32_state.transmitted_ids.size() == TOTAL);
    for (uint32_t i = 0U; i < TOTAL; ++i)
    {
      TEST_ASSERT(g_ch32_state.transmitted_ids[i] == 0x200U + i);
    }
  }
}
