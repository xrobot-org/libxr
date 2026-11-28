#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

#include "stm32_can.hpp"
#include "test_assert.hpp"

extern "C"
{
  CAN_TypeDef g_stm32_can1_regs{};
  CAN_TypeDef g_stm32_can2_regs{};
}

namespace
{
struct BxCanStubState
{
  std::mutex mtx;
  uint32_t mailbox_mask = 0U;  // bit 0..2 occupied
  bool force_next_add_fail = false;
  bool started = false;
  std::function<void()> on_stop_hook;
  std::function<void()> on_zero_tsr_hook;
  std::atomic<uint32_t> active_add_calls{0U};
  std::vector<uint32_t> transmitted_ids;

  void UpdateTsr(CAN_TypeDef* regs) const
  {
    uint32_t tsr = 0U;
    if (started)
    {
      if ((mailbox_mask & 1U) == 0U)
      {
        tsr |= CAN_TSR_TME0;
      }
      if ((mailbox_mask & 2U) == 0U)
      {
        tsr |= CAN_TSR_TME1;
      }
      if ((mailbox_mask & 4U) == 0U)
      {
        tsr |= CAN_TSR_TME2;
      }
      if (force_next_add_fail)
      {
        tsr |= CAN_TSR_TME0;
      }
    }
    regs->TSR = tsr;
  }

  void Reset(CAN_TypeDef* regs)
  {
    std::lock_guard<std::mutex> lk(mtx);
    mailbox_mask = 0U;
    force_next_add_fail = false;
    started = false;
    on_stop_hook = nullptr;
    on_zero_tsr_hook = nullptr;
    regs->TSR.on_zero_read.store(nullptr, std::memory_order_relaxed);
    active_add_calls.store(0U, std::memory_order_relaxed);
    transmitted_ids.clear();
    UpdateTsr(regs);
  }
};

BxCanStubState g_bxcan_state;

void DispatchBxCanZeroTsrHook()
{
  std::function<void()> hook;
  {
    std::lock_guard<std::mutex> lk(g_bxcan_state.mtx);
    hook = std::move(g_bxcan_state.on_zero_tsr_hook);
  }
  if (hook)
  {
    hook();
  }
}
}  // namespace

extern "C"
{
  HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef*, const CAN_FilterTypeDef*)
  {
    return HAL_OK;
  }

  HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef* hcan)
  {
    std::lock_guard<std::mutex> lk(g_bxcan_state.mtx);
    g_bxcan_state.started = true;
    g_bxcan_state.mailbox_mask = 0U;
    g_bxcan_state.force_next_add_fail = false;
    g_bxcan_state.UpdateTsr(hcan->Instance);
    return HAL_OK;
  }

  HAL_StatusTypeDef HAL_CAN_Stop(CAN_HandleTypeDef* hcan)
  {
    std::function<void()> hook;
    {
      std::lock_guard<std::mutex> lk(g_bxcan_state.mtx);
      g_bxcan_state.started = false;
      g_bxcan_state.UpdateTsr(hcan->Instance);
      hook = std::move(g_bxcan_state.on_stop_hook);
    }
    if (hook)
    {
      hook();
    }
    return HAL_OK;
  }

  HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef*, uint32_t)
  {
    return HAL_OK;
  }

  HAL_StatusTypeDef HAL_CAN_DeactivateNotification(CAN_HandleTypeDef*, uint32_t)
  {
    return HAL_OK;
  }

  HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef* hcan,
                                         const CAN_TxHeaderTypeDef* pHeader,
                                         const uint8_t[], uint32_t* pTxMailbox)
  {
    const uint32_t prev =
        g_bxcan_state.active_add_calls.fetch_add(1U, std::memory_order_acq_rel);
    TEST_ASSERT(prev == 0U);

    HAL_StatusTypeDef res = HAL_ERROR;
    {
      std::lock_guard<std::mutex> lk(g_bxcan_state.mtx);
      if (!g_bxcan_state.started || g_bxcan_state.force_next_add_fail)
      {
        g_bxcan_state.force_next_add_fail = false;
        g_bxcan_state.UpdateTsr(hcan->Instance);
        res = HAL_ERROR;
      }
      else
      {
        for (uint32_t mb = 0U; mb < 3U; ++mb)
        {
          if ((g_bxcan_state.mailbox_mask & (1U << mb)) == 0U)
          {
            g_bxcan_state.mailbox_mask |= (1U << mb);
            *pTxMailbox = (1U << mb);
            g_bxcan_state.transmitted_ids.push_back(
                pHeader->IDE == CAN_ID_EXT ? pHeader->ExtId : pHeader->StdId);
            g_bxcan_state.UpdateTsr(hcan->Instance);
            res = HAL_OK;
            break;
          }
        }
      }
    }

    const uint32_t after =
        g_bxcan_state.active_add_calls.fetch_sub(1U, std::memory_order_acq_rel);
    TEST_ASSERT(after == 1U);
    return res;
  }

  HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef*, uint32_t,
                                         CAN_RxHeaderTypeDef*, uint8_t[])
  {
    return HAL_ERROR;
  }

  HAL_StatusTypeDef HAL_CAN_ResetError(CAN_HandleTypeDef*) { return HAL_OK; }

  uint32_t HAL_RCC_GetPCLK1Freq(void) { return 42000000U; }
}

void RunStm32CanProdTests()
{
  using namespace LibXR;

  g_stm32_can1_regs = {};
  g_bxcan_state.Reset(CAN1);
  CAN_HandleTypeDef hcan{CAN1};
  STM32CAN can(&hcan, 4U);

  auto make_pack = [](uint32_t id)
  {
    CAN::ClassicPack p{};
    p.id = id;
    p.type = CAN::Type::STANDARD;
    p.dlc = 4U;
    return p;
  };

  // 1. 填满 3 个硬件邮箱，触发 retry 槽与软件队列 FULL，再通过真实邮箱完成回调排空。
  // Saturate 3 HW mailboxes, capture retry slot, hit FULL, then drain via real HAL IRQ
  // callbacks.
  TEST_ASSERT(can.AddMessage(make_pack(0x10U)) == ErrorCode::OK);
  TEST_ASSERT(can.AddMessage(make_pack(0x11U)) == ErrorCode::OK);
  TEST_ASSERT(can.AddMessage(make_pack(0x12U)) == ErrorCode::OK);
  TEST_ASSERT(g_bxcan_state.transmitted_ids.size() == 3U);

  {
    std::lock_guard<std::mutex> lk(g_bxcan_state.mtx);
    g_bxcan_state.force_next_add_fail = true;
    g_bxcan_state.UpdateTsr(CAN1);
  }
  TEST_ASSERT(can.AddMessage(make_pack(0x13U)) == ErrorCode::OK);
  TEST_ASSERT(can.tx_retry_valid_);
  TEST_ASSERT(can.tx_retry_pack_.id == 0x13U);

  TEST_ASSERT(can.AddMessage(make_pack(0x14U)) == ErrorCode::OK);
  TEST_ASSERT(can.AddMessage(make_pack(0x15U)) == ErrorCode::OK);
  TEST_ASSERT(can.AddMessage(make_pack(0x16U)) == ErrorCode::OK);
  TEST_ASSERT(can.AddMessage(make_pack(0x17U)) == ErrorCode::OK);
  TEST_ASSERT(can.AddMessage(make_pack(0x18U)) == ErrorCode::FULL);

  for (uint32_t i = 0U; i < 6U; ++i)
  {
    {
      std::lock_guard<std::mutex> lk(g_bxcan_state.mtx);
      g_bxcan_state.mailbox_mask &= ~(1U << (i % 3U));
      g_bxcan_state.UpdateTsr(CAN1);
    }
    if ((i % 3U) == 0U)
    {
      HAL_CAN_TxMailbox0CompleteCallback(&hcan);
    }
    else if ((i % 3U) == 1U)
    {
      HAL_CAN_TxMailbox1CompleteCallback(&hcan);
    }
    else
    {
      HAL_CAN_TxMailbox2CompleteCallback(&hcan);
    }
  }

  TEST_ASSERT(!can.tx_retry_valid_);
  TEST_ASSERT(g_bxcan_state.transmitted_ids.size() == 8U);
  for (uint32_t i = 0U; i < 8U; ++i)
  {
    TEST_ASSERT(g_bxcan_state.transmitted_ids[i] == 0x10U + i);
  }

  // 2. 停机重配 SetConfig() 恢复：无新 AddMessage() 与完成回调时须自动续发滞留帧。
  // SetConfig() recovery resumes stranded retry + queued frames with zero new AddMessage.
  {
    std::lock_guard<std::mutex> lk(g_bxcan_state.mtx);
    g_bxcan_state.transmitted_ids.clear();
    g_bxcan_state.mailbox_mask = 0x7U;
    g_bxcan_state.force_next_add_fail = true;
    g_bxcan_state.UpdateTsr(CAN1);
  }
  TEST_ASSERT(can.AddMessage(make_pack(0x20U)) == ErrorCode::OK);
  TEST_ASSERT(can.tx_retry_valid_);
  TEST_ASSERT(can.AddMessage(make_pack(0x21U)) == ErrorCode::OK);
  TEST_ASSERT(g_bxcan_state.transmitted_ids.empty());

  CAN::Configuration cfg{};
  cfg.bit_timing.brp = 3U;
  cfg.bit_timing.prop_seg = 4U;
  cfg.bit_timing.phase_seg1 = 6U;
  cfg.bit_timing.phase_seg2 = 3U;
  cfg.bit_timing.sjw = 1U;
  TEST_ASSERT(can.SetConfig(cfg) == ErrorCode::OK);
  TEST_ASSERT(!can.tx_retry_valid_);
  TEST_ASSERT(g_bxcan_state.transmitted_ids.size() == 2U);
  TEST_ASSERT(g_bxcan_state.transmitted_ids[0] == 0x20U);
  TEST_ASSERT(g_bxcan_state.transmitted_ids[1] == 0x21U);

  // 2b. 停机窗口确定性注入：在 HAL_CAN_Stop 期间入队，SetConfig() 结束后自动续发。
  // Stop-window deterministic injection inside HAL_CAN_Stop.
  {
    uint32_t stop_hook_ran = 0U;
    {
      std::lock_guard<std::mutex> lk(g_bxcan_state.mtx);
      g_bxcan_state.transmitted_ids.clear();
      g_bxcan_state.mailbox_mask = 0U;
      g_bxcan_state.UpdateTsr(CAN1);
      g_bxcan_state.on_stop_hook = [&]
      {
        ++stop_hook_ran;
        TEST_ASSERT(can.AddMessage(make_pack(0x25U)) == ErrorCode::OK);
        TEST_ASSERT(g_bxcan_state.transmitted_ids.empty());
      };
    }
    TEST_ASSERT(can.SetConfig(cfg) == ErrorCode::OK);
    TEST_ASSERT(stop_hook_ran == 1U);
    TEST_ASSERT(g_bxcan_state.transmitted_ids.size() == 1U);
    TEST_ASSERT(g_bxcan_state.transmitted_ids[0] == 0x25U);
  }

  // 2c. Owner 退出窗口确定性注入：第 3 个邮箱填满后，TxService 读到 TSR==0 准备退出循环
  // 瞬间释放邮箱并触发重入 AddMessage + TxMailbox0CompleteCallback；须由
  // SerializedService 退出前重检 PENDING 续发第 4 帧。 Owner-exit-window deterministic
  // injection via TSR==0 read hook.
  {
    {
      std::lock_guard<std::mutex> lk(g_bxcan_state.mtx);
      g_bxcan_state.transmitted_ids.clear();
      g_bxcan_state.mailbox_mask = 0U;
      g_bxcan_state.UpdateTsr(CAN1);
    }
    TEST_ASSERT(can.AddMessage(make_pack(0x60U)) == ErrorCode::OK);
    TEST_ASSERT(can.AddMessage(make_pack(0x61U)) == ErrorCode::OK);
    TEST_ASSERT(g_bxcan_state.transmitted_ids.size() == 2U);

    uint32_t exit_hook_ran = 0U;
    {
      std::lock_guard<std::mutex> lk(g_bxcan_state.mtx);
      g_bxcan_state.on_zero_tsr_hook = [&]
      {
        ++exit_hook_ran;
        {
          std::lock_guard<std::mutex> inner(g_bxcan_state.mtx);
          TEST_ASSERT(g_bxcan_state.transmitted_ids.size() == 3U);
          g_bxcan_state.mailbox_mask &= ~1U;
          g_bxcan_state.UpdateTsr(CAN1);
        }
        TEST_ASSERT(can.AddMessage(make_pack(0x63U)) == ErrorCode::OK);
        HAL_CAN_TxMailbox0CompleteCallback(&hcan);
        {
          std::lock_guard<std::mutex> inner(g_bxcan_state.mtx);
          TEST_ASSERT(g_bxcan_state.transmitted_ids.size() == 3U);
        }
      };
    }
    CAN1->TSR.on_zero_read.store(&DispatchBxCanZeroTsrHook, std::memory_order_release);

    TEST_ASSERT(can.AddMessage(make_pack(0x62U)) == ErrorCode::OK);
    TEST_ASSERT(exit_hook_ran == 1U);
    TEST_ASSERT(g_bxcan_state.transmitted_ids.size() == 4U);
    TEST_ASSERT(g_bxcan_state.transmitted_ids[0] == 0x60U);
    TEST_ASSERT(g_bxcan_state.transmitted_ids[1] == 0x61U);
    TEST_ASSERT(g_bxcan_state.transmitted_ids[2] == 0x62U);
    TEST_ASSERT(g_bxcan_state.transmitted_ids[3] == 0x63U);
  }

  // 3. 多线程并发 AddMessage() 与 HAL_CAN_TxMailbox*CompleteCallback 交错。
  // Multi-thread stress: concurrent AddMessage() vs TX mailbox completion callbacks.
  {
    {
      std::lock_guard<std::mutex> lk(g_bxcan_state.mtx);
      g_bxcan_state.transmitted_ids.clear();
      g_bxcan_state.mailbox_mask = 0U;
      g_bxcan_state.UpdateTsr(CAN1);
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
              std::lock_guard<std::mutex> lk(g_bxcan_state.mtx);
              if ((g_bxcan_state.mailbox_mask & (1U << mb)) != 0U)
              {
                g_bxcan_state.mailbox_mask &= ~(1U << mb);
                g_bxcan_state.UpdateTsr(CAN1);
                freed = true;
              }
              else if (prod_done.load(std::memory_order_acquire) != 0U &&
                       g_bxcan_state.transmitted_ids.size() >= TOTAL)
              {
                break;
              }
            }
            if (freed)
            {
              if (mb == 0U)
              {
                HAL_CAN_TxMailbox0CompleteCallback(&hcan);
              }
              else if (mb == 1U)
              {
                HAL_CAN_TxMailbox1CompleteCallback(&hcan);
              }
              else
              {
                HAL_CAN_TxMailbox2CompleteCallback(&hcan);
              }
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
      while (can.AddMessage(make_pack(0x100U + i)) == ErrorCode::FULL)
      {
        std::this_thread::yield();
      }
    }
    prod_done.store(1U, std::memory_order_release);
    irq_thread.join();

    TEST_ASSERT(g_bxcan_state.transmitted_ids.size() == TOTAL);
    for (uint32_t i = 0U; i < TOTAL; ++i)
    {
      TEST_ASSERT(g_bxcan_state.transmitted_ids[i] == 0x100U + i);
    }
  }
}
