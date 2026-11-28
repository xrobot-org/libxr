#include <atomic>
#include <cstdint>
#include <cstring>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

#include "stm32_canfd.hpp"
#include "test_assert.hpp"

extern "C"
{
  FDCAN_GlobalTypeDef g_stm32_fdcan1_regs{};
  FDCAN_GlobalTypeDef g_stm32_fdcan2_regs{};
  FDCAN_GlobalTypeDef g_stm32_fdcan3_regs{};
}

namespace
{
struct TxRecord
{
  bool is_fd = false;
  uint32_t id = 0U;
  uint32_t data_length = 0U;
  uint8_t first_byte = 0U;
};

struct FdcanStubState
{
  std::mutex mtx;
  uint32_t hw_capacity = 2U;
  uint32_t hw_occupied = 0U;
  bool force_next_add_fail = false;
  bool started = false;
  bool bus_off = false;
  uint32_t deactivate_calls = 0U;
  uint32_t stop_calls = 0U;
  uint32_t start_calls = 0U;
  uint32_t activate_calls = 0U;
  uint32_t add_attempt_calls = 0U;
  uint32_t last_filter_config = 0U;
  uint32_t active_its = 0U;
  std::function<void()> on_stop_hook;
  std::function<void()> on_zero_free_hook;
  std::atomic<uint32_t> active_add_calls{0U};
  std::vector<TxRecord> transmitted;

  void Reset(uint32_t cap = 2U)
  {
    std::lock_guard<std::mutex> lk(mtx);
    hw_capacity = cap;
    hw_occupied = 0U;
    force_next_add_fail = false;
    started = false;
    bus_off = false;
    deactivate_calls = 0U;
    stop_calls = 0U;
    start_calls = 0U;
    activate_calls = 0U;
    add_attempt_calls = 0U;
    last_filter_config = 0U;
    active_its = 0U;
    on_stop_hook = nullptr;
    on_zero_free_hook = nullptr;
    active_add_calls.store(0U, std::memory_order_relaxed);
    transmitted.clear();
  }
};

FdcanStubState g_fdcan_state[3];

FdcanStubState& StubFor(const FDCAN_HandleTypeDef* h)
{
  if (h->Instance == FDCAN1)
  {
    return g_fdcan_state[0];
  }
  if (h->Instance == FDCAN2)
  {
    return g_fdcan_state[1];
  }
  TEST_ASSERT(h->Instance == FDCAN3);
  return g_fdcan_state[2];
}
}  // namespace

extern "C"
{
  HAL_StatusTypeDef HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef* hfdcan,
                                           const FDCAN_FilterTypeDef* sFilterConfig)
  {
    auto& st = StubFor(hfdcan);
    std::lock_guard<std::mutex> lk(st.mtx);
    st.last_filter_config = sFilterConfig->FilterConfig;
    return HAL_OK;
  }

  HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef* hfdcan)
  {
    auto& st = StubFor(hfdcan);
    std::lock_guard<std::mutex> lk(st.mtx);
    ++st.start_calls;
    st.started = true;
    st.hw_occupied = 0U;
    st.force_next_add_fail = false;
    return HAL_OK;
  }

  HAL_StatusTypeDef HAL_FDCAN_Stop(FDCAN_HandleTypeDef* hfdcan)
  {
    auto& st = StubFor(hfdcan);
    std::function<void()> hook;
    {
      std::lock_guard<std::mutex> lk(st.mtx);
      ++st.stop_calls;
      st.started = false;
      hook = std::move(st.on_stop_hook);
    }
    if (hook)
    {
      hook();
    }
    return HAL_OK;
  }

  HAL_StatusTypeDef HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef* hfdcan,
                                                   uint32_t ActiveITs, uint32_t)
  {
    auto& st = StubFor(hfdcan);
    std::lock_guard<std::mutex> lk(st.mtx);
    ++st.activate_calls;
    st.active_its |= ActiveITs;
    return HAL_OK;
  }

  HAL_StatusTypeDef HAL_FDCAN_DeactivateNotification(FDCAN_HandleTypeDef* hfdcan,
                                                     uint32_t InactiveITs)
  {
    auto& st = StubFor(hfdcan);
    std::lock_guard<std::mutex> lk(st.mtx);
    ++st.deactivate_calls;
    st.active_its &= ~InactiveITs;
    return HAL_OK;
  }

  uint32_t HAL_FDCAN_GetTxFifoFreeLevel(const FDCAN_HandleTypeDef* hfdcan)
  {
    auto& st = StubFor(hfdcan);
    uint32_t free_lvl = 0U;
    std::function<void()> hook;
    {
      std::lock_guard<std::mutex> lk(st.mtx);
      if (!st.started || st.bus_off)
      {
        free_lvl = 0U;
      }
      else if (st.force_next_add_fail)
      {
        free_lvl = 1U;
      }
      else
      {
        free_lvl =
            (st.hw_capacity > st.hw_occupied) ? (st.hw_capacity - st.hw_occupied) : 0U;
      }
      if (free_lvl == 0U && st.on_zero_free_hook)
      {
        hook = std::move(st.on_zero_free_hook);
      }
    }
    if (hook)
    {
      hook();
    }
    return free_lvl;
  }

  HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef* hfdcan,
                                                  const FDCAN_TxHeaderTypeDef* pTxHeader,
                                                  const uint8_t* pTxData)
  {
    auto& st = StubFor(hfdcan);
    // 断言 TxService 串行互斥：绝不允许并发重入 HAL TX 接口。
    // Assert strict SerializedService mutual exclusion on HAL TX calls.
    const uint32_t prev = st.active_add_calls.fetch_add(1U, std::memory_order_acq_rel);
    TEST_ASSERT(prev == 0U);

    HAL_StatusTypeDef res = HAL_OK;
    {
      std::lock_guard<std::mutex> lk(st.mtx);
      ++st.add_attempt_calls;
      if (!st.started || st.bus_off || st.force_next_add_fail ||
          st.hw_occupied >= st.hw_capacity)
      {
        st.force_next_add_fail = false;
        res = HAL_ERROR;
      }
      else
      {
        ++st.hw_occupied;
        st.transmitted.push_back({pTxHeader->FDFormat == FDCAN_FD_CAN,
                                  pTxHeader->Identifier, pTxHeader->DataLength,
                                  pTxData != nullptr ? pTxData[0] : uint8_t{0U}});
        res = HAL_OK;
      }
    }

    const uint32_t after = st.active_add_calls.fetch_sub(1U, std::memory_order_acq_rel);
    TEST_ASSERT(after == 1U);
    return res;
  }

  HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef*, uint32_t,
                                           FDCAN_RxHeaderTypeDef*, uint8_t*)
  {
    return HAL_ERROR;
  }

  uint32_t HAL_FDCAN_GetRxFifoFillLevel(const FDCAN_HandleTypeDef*, uint32_t)
  {
    return 0U;
  }

  HAL_StatusTypeDef HAL_FDCAN_GetProtocolStatus(
      const FDCAN_HandleTypeDef* hfdcan, FDCAN_ProtocolStatusTypeDef* ProtocolStatus)
  {
    auto& st = StubFor(hfdcan);
    std::lock_guard<std::mutex> lk(st.mtx);
    *ProtocolStatus = {};
    ProtocolStatus->BusOff = st.bus_off ? 1U : 0U;
    return HAL_OK;
  }

  HAL_StatusTypeDef HAL_FDCAN_GetErrorCounters(const FDCAN_HandleTypeDef*,
                                               FDCAN_ErrorCountersTypeDef* ErrorCounters)
  {
    *ErrorCounters = {};
    return HAL_OK;
  }

  uint32_t HAL_RCCEx_GetPeriphCLKFreq(uint32_t) { return 80000000U; }
}

void RunStm32CanfdProdTests()
{
  using namespace LibXR;

  g_stm32_fdcan1_regs = {};
  g_stm32_fdcan2_regs = {};
  g_stm32_fdcan3_regs = {};
  g_fdcan_state[0].Reset(2U);
  g_fdcan_state[1].Reset(2U);
  g_fdcan_state[2].Reset(2U);

  FDCAN_HandleTypeDef h1{FDCAN1, {}, 0U};
  FDCAN_HandleTypeDef h2{FDCAN2, {}, 0U};
  FDCAN_HandleTypeDef h3{FDCAN3, {}, 0U};

  STM32CANFD can1(&h1, 4U);
  STM32CANFD can2(&h2, 4U);
  STM32CANFD can3(&h3, 4U);

  // 1. 验证 FDCAN1 -> FIFO0，FDCAN2/3 -> FIFO1 路由，以及 SetConfig 保留正确 FIFO 通知。
  // Verify FDCAN1 -> FIFO0, FDCAN2/3 -> FIFO1 in both Init() and SetConfig().
  TEST_ASSERT(g_fdcan_state[0].last_filter_config == FDCAN_FILTER_TO_RXFIFO0);
  TEST_ASSERT((g_fdcan_state[0].active_its & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0U);
  TEST_ASSERT((g_fdcan_state[0].active_its & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) == 0U);

  TEST_ASSERT(g_fdcan_state[1].last_filter_config == FDCAN_FILTER_TO_RXFIFO1);
  TEST_ASSERT((g_fdcan_state[1].active_its & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != 0U);
  TEST_ASSERT((g_fdcan_state[1].active_its & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U);

  TEST_ASSERT(g_fdcan_state[2].last_filter_config == FDCAN_FILTER_TO_RXFIFO1);
  TEST_ASSERT((g_fdcan_state[2].active_its & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != 0U);

  {
    FDCAN::Configuration valid_cfg{};
    valid_cfg.bit_timing.brp = 2U;
    valid_cfg.bit_timing.prop_seg = 6U;
    valid_cfg.bit_timing.phase_seg1 = 4U;
    valid_cfg.bit_timing.phase_seg2 = 4U;
    valid_cfg.bit_timing.sjw = 2U;
    TEST_ASSERT(can2.SetConfig(valid_cfg) == ErrorCode::OK);
    TEST_ASSERT((g_fdcan_state[1].active_its & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != 0U);
    TEST_ASSERT((g_fdcan_state[1].active_its & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U);
  }

  // 2. 验证 SetConfig 纯参数 ARG_ERR（含溢出与保留字段组合有效 SJW <= TSEG2）：
  // 零通知关闭、零 Stop/Start、零 NBTP/DBTP/CCCR/TEST 写入。
  // Pure-parameter ARG_ERR (including overflow and effective kept SJW <= TSEG2):
  // zero notification/stop/start calls and zero register writes.
  {
    // Seed NBTP: BRP=2, TSEG1=10, TSEG2=4, SJW=4; DBTP: BRP=2, TSEG1=6, TSEG2=4, SJW=4.
    const uint32_t init_nbtp =
        ((2U - 1U) << FDCAN_NBTP_NBRP_Pos) | ((10U - 1U) << FDCAN_NBTP_NTSEG1_Pos) |
        ((4U - 1U) << FDCAN_NBTP_NTSEG2_Pos) | ((4U - 1U) << FDCAN_NBTP_NSJW_Pos);
    const uint32_t init_dbtp =
        ((2U - 1U) << FDCAN_DBTP_DBRP_Pos) | ((6U - 1U) << FDCAN_DBTP_DTSEG1_Pos) |
        ((4U - 1U) << FDCAN_DBTP_DTSEG2_Pos) | ((4U - 1U) << FDCAN_DBTP_DSJW_Pos);
    const uint32_t init_cccr = 0x00000010U;
    const uint32_t init_test = 0x00000000U;
    FDCAN1->NBTP = init_nbtp;
    FDCAN1->DBTP = init_dbtp;
    FDCAN1->CCCR = init_cccr;
    FDCAN1->TEST = init_test;

    auto assert_arg_err_untouched = [&](const FDCAN::Configuration& bad)
    {
      auto& st = g_fdcan_state[0];
      const uint32_t d0 = st.deactivate_calls;
      const uint32_t st0 = st.stop_calls;
      const uint32_t sr0 = st.start_calls;
      const uint32_t a0 = st.activate_calls;
      TEST_ASSERT(can1.SetConfig(bad) == ErrorCode::ARG_ERR);
      TEST_ASSERT(st.deactivate_calls == d0);
      TEST_ASSERT(st.stop_calls == st0);
      TEST_ASSERT(st.start_calls == sr0);
      TEST_ASSERT(st.activate_calls == a0);
      TEST_ASSERT(FDCAN1->NBTP == init_nbtp);
      TEST_ASSERT(FDCAN1->DBTP == init_dbtp);
      TEST_ASSERT(FDCAN1->CCCR == init_cccr);
      TEST_ASSERT(FDCAN1->TEST == init_test);
    };

    FDCAN::Configuration c{};
    c.bit_timing.brp = 513U;
    assert_arg_err_untouched(c);

    c = {};
    c.bit_timing.prop_seg = UINT32_MAX;
    c.bit_timing.phase_seg1 = 2U;
    assert_arg_err_untouched(c);

    c = {};
    c.bit_timing.prop_seg = 200U;
    c.bit_timing.phase_seg1 = 60U;
    assert_arg_err_untouched(c);

    c = {};
    c.bit_timing.phase_seg2 = 129U;
    assert_arg_err_untouched(c);

    c = {};
    c.bit_timing.sjw = 129U;
    assert_arg_err_untouched(c);

    // 同时提供非零 phase_seg2 与 sjw 且 sjw > phase_seg2 必须拒绝。
    // Explicit non-zero phase_seg2 and sjw with sjw > phase_seg2 must return ARG_ERR.
    c = {};
    c.bit_timing.phase_seg2 = 2U;
    c.bit_timing.sjw = 3U;
    assert_arg_err_untouched(c);

    c = {};
    c.data_timing.brp = 33U;
    assert_arg_err_untouched(c);

    c = {};
    c.data_timing.prop_seg = UINT32_MAX;
    c.data_timing.phase_seg1 = 2U;
    assert_arg_err_untouched(c);

    c = {};
    c.data_timing.prop_seg = 20U;
    c.data_timing.phase_seg1 = 15U;
    assert_arg_err_untouched(c);

    c = {};
    c.data_timing.phase_seg2 = 17U;
    assert_arg_err_untouched(c);

    c = {};
    c.data_timing.sjw = 17U;
    assert_arg_err_untouched(c);

    // 数据相位同时提供非零 phase_seg2 与 sjw 且 sjw > phase_seg2 必须拒绝。
    c = {};
    c.data_timing.phase_seg2 = 2U;
    c.data_timing.sjw = 3U;
    assert_arg_err_untouched(c);

    // 仅更新 phase_seg2=2 而 sjw=0（保留原值语义）与基线一致返回 OK，且只改对应字段。
    // Partial update (phase_seg2=2, sjw=0) preserves baseline 0-keeps-existing semantics.
    c = {};
    c.bit_timing.phase_seg2 = 2U;
    c.data_timing.phase_seg2 = 3U;
    TEST_ASSERT(can1.SetConfig(c) == ErrorCode::OK);
    TEST_ASSERT(((FDCAN1->NBTP & FDCAN_NBTP_NTSEG2_Msk) >> FDCAN_NBTP_NTSEG2_Pos) == 1U);
    TEST_ASSERT(((FDCAN1->NBTP & FDCAN_NBTP_NSJW_Msk) >> FDCAN_NBTP_NSJW_Pos) == 3U);
    TEST_ASSERT(((FDCAN1->DBTP & FDCAN_DBTP_DTSEG2_Msk) >> FDCAN_DBTP_DTSEG2_Pos) == 2U);
    TEST_ASSERT(((FDCAN1->DBTP & FDCAN_DBTP_DSJW_Msk) >> FDCAN_DBTP_DSJW_Pos) == 3U);

    // 合法同时修改 NTSEG2=2, NSJW=2 与 DTSEG2=3, DSJW=2 应生效。
    c = {};
    c.bit_timing.phase_seg2 = 2U;
    c.bit_timing.sjw = 2U;
    c.data_timing.phase_seg2 = 3U;
    c.data_timing.sjw = 2U;
    c.mode.loopback = true;
    TEST_ASSERT(can1.SetConfig(c) == ErrorCode::OK);
    TEST_ASSERT(((FDCAN1->NBTP & FDCAN_NBTP_NTSEG2_Msk) >> FDCAN_NBTP_NTSEG2_Pos) == 1U);
    TEST_ASSERT(((FDCAN1->NBTP & FDCAN_NBTP_NSJW_Msk) >> FDCAN_NBTP_NSJW_Pos) == 1U);
    TEST_ASSERT(((FDCAN1->DBTP & FDCAN_DBTP_DTSEG2_Msk) >> FDCAN_DBTP_DTSEG2_Pos) == 2U);
    TEST_ASSERT(((FDCAN1->DBTP & FDCAN_DBTP_DSJW_Msk) >> FDCAN_DBTP_DSJW_Pos) == 1U);
    TEST_ASSERT((FDCAN1->TEST & FDCAN_TEST_LBCK) != 0U);
    TEST_ASSERT((FDCAN1->CCCR & FDCAN_CCCR_MON) != 0U);
  }

  // 3. 验证邮箱满、FULL、Classic 与 FD 两种 retry 槽、FD 优先顺序以及真实 TX 完成回调。
  // Verify mailbox full, FULL, both Classic & FD retry slots, FD-priority order, and
  // real HAL_FDCAN_TxBufferCompleteCallback / TxFifoEmptyCallback draining.
  {
    auto& st = g_fdcan_state[0];
    st.Reset(2U);
    st.started = true;

    auto make_classic = [](uint32_t id, uint8_t tag)
    {
      CAN::ClassicPack p{};
      p.id = id;
      p.type = CAN::Type::STANDARD;
      p.dlc = 4U;
      p.data[0] = tag;
      return p;
    };
    auto make_fd = [](uint32_t id, uint8_t tag)
    {
      FDCAN::FDPack p{};
      p.id = id;
      p.type = CAN::Type::STANDARD;
      p.len = 12U;
      p.data[0] = tag;
      return p;
    };

    // 占满 2 个硬件槽 / Fill both HW slots.
    TEST_ASSERT(can1.AddMessage(make_classic(0x100U, 1U)) == ErrorCode::OK);
    TEST_ASSERT(can1.AddMessage(make_classic(0x101U, 2U)) == ErrorCode::OK);
    TEST_ASSERT(st.transmitted.size() == 2U);

    // 强制下一次 AddMessageToTxFifoQ 返回失败，精准进入 tx_classic_retry_valid_。
    // Force HAL add failure to enter tx_classic_retry_valid_ deterministically.
    st.force_next_add_fail = true;
    TEST_ASSERT(can1.AddMessage(make_classic(0x102U, 3U)) == ErrorCode::OK);
    TEST_ASSERT(can1.tx_classic_retry_valid_);
    TEST_ASSERT(can1.tx_classic_retry_pack_.id == 0x102U);

    // 继续入队直至 Classic 软件队列返回 FULL（queue_size=4）。
    // Fill Classic software queue until FULL.
    TEST_ASSERT(can1.AddMessage(make_classic(0x103U, 4U)) == ErrorCode::OK);
    TEST_ASSERT(can1.AddMessage(make_classic(0x104U, 5U)) == ErrorCode::OK);
    TEST_ASSERT(can1.AddMessage(make_classic(0x105U, 6U)) == ErrorCode::OK);
    TEST_ASSERT(can1.AddMessage(make_classic(0x106U, 7U)) == ErrorCode::OK);
    TEST_ASSERT(can1.AddMessage(make_classic(0x107U, 8U)) == ErrorCode::FULL);

    // 再强制捕获 FD retry 槽并填满 FD 队列至 FULL。
    // Capture FD retry slot and saturate FD software queue until FULL.
    st.force_next_add_fail = true;
    TEST_ASSERT(can1.AddMessage(make_fd(0x200U, 10U)) == ErrorCode::OK);
    TEST_ASSERT(can1.tx_fd_retry_valid_);
    TEST_ASSERT(can1.tx_fd_retry_pack_.id == 0x200U);

    TEST_ASSERT(can1.AddMessage(make_fd(0x201U, 11U)) == ErrorCode::OK);
    TEST_ASSERT(can1.AddMessage(make_fd(0x202U, 12U)) == ErrorCode::OK);
    TEST_ASSERT(can1.AddMessage(make_fd(0x203U, 13U)) == ErrorCode::OK);
    TEST_ASSERT(can1.AddMessage(make_fd(0x204U, 14U)) == ErrorCode::OK);
    TEST_ASSERT(can1.AddMessage(make_fd(0x205U, 15U)) == ErrorCode::FULL);

    // 通过真实 HAL 完成回调逐槽释放并续发，核对严格 FD 优先 + 组内 FIFO。
    // Drain via real HAL callbacks and verify strict FD-first then Classic FIFO order.
    for (int i = 0; i < 12; ++i)
    {
      {
        std::lock_guard<std::mutex> lk(st.mtx);
        if (st.hw_occupied > 0U)
        {
          --st.hw_occupied;
        }
      }
      if ((i & 1) == 0)
      {
        HAL_FDCAN_TxBufferCompleteCallback(&h1, 1U);
      }
      else
      {
        HAL_FDCAN_TxFifoEmptyCallback(&h1);
      }
    }

    TEST_ASSERT(!can1.tx_fd_retry_valid_);
    TEST_ASSERT(!can1.tx_classic_retry_valid_);

    const std::vector<std::pair<bool, uint32_t>> expected_order = {
        {false, 0x100U}, {false, 0x101U},  // initial 2 in-flight Classic
        {true, 0x200U},  {true, 0x201U},  {true, 0x202U},
        {true, 0x203U},  {true, 0x204U},  // all 5 FD frames strictly first
        {false, 0x102U}, {false, 0x103U}, {false, 0x104U},
        {false, 0x105U}, {false, 0x106U},  // then all 5 Classic frames (retry first)
    };
    TEST_ASSERT(st.transmitted.size() == expected_order.size());
    for (size_t i = 0; i < expected_order.size(); ++i)
    {
      TEST_ASSERT(st.transmitted[i].is_fd == expected_order[i].first);
      TEST_ASSERT(st.transmitted[i].id == expected_order[i].second);
    }
  }

  // 4. 停机重配 SetConfig() 恢复：无任何新 AddMessage() 与外部 TX IRQ 时须自动续发
  // retry 槽与软件队列中滞留的帧。
  // SetConfig() stop/start recovery: must resume stranded retry + queued frames with
  // ZERO new AddMessage() calls and ZERO external TX callbacks.
  {
    auto& st = g_fdcan_state[0];
    st.Reset(4U);
    st.started = true;
    st.hw_occupied = 4U;  // HW full

    FDCAN::FDPack pfd{};
    pfd.id = 0x300U;
    pfd.type = CAN::Type::STANDARD;
    pfd.len = 8U;
    pfd.data[0] = 0x55U;
    st.force_next_add_fail = true;
    TEST_ASSERT(can1.AddMessage(pfd) == ErrorCode::OK);
    TEST_ASSERT(can1.tx_fd_retry_valid_);

    CAN::ClassicPack pc{};
    pc.id = 0x301U;
    pc.type = CAN::Type::STANDARD;
    pc.dlc = 4U;
    pc.data[0] = 0x66U;
    TEST_ASSERT(can1.AddMessage(pc) == ErrorCode::OK);
    TEST_ASSERT(st.transmitted.empty());

    FDCAN::Configuration reconfig{};
    TEST_ASSERT(can1.SetConfig(reconfig) == ErrorCode::OK);
    TEST_ASSERT(!can1.tx_fd_retry_valid_);
    TEST_ASSERT(st.transmitted.size() == 2U);
    TEST_ASSERT(st.transmitted[0].is_fd && st.transmitted[0].id == 0x300U);
    TEST_ASSERT(!st.transmitted[1].is_fd && st.transmitted[1].id == 0x301U);
  }

  // 4b. 停机窗口确定性注入：在 HAL_FDCAN_Stop 与 HAL_FDCAN_Start 之间入队新帧，
  // 验证 SetConfig() 末尾 TxService() 在无外部 TX IRQ 时自动续发。
  // Stop-window deterministic injection: submit Classic + FD frames inside
  // HAL_FDCAN_Stop.
  {
    auto& st = g_fdcan_state[0];
    st.Reset(4U);
    st.started = true;
    uint32_t stop_hook_ran = 0U;
    st.on_stop_hook = [&]
    {
      ++stop_hook_ran;
      CAN::ClassicPack pc{};
      pc.id = 0x310U;
      pc.type = CAN::Type::STANDARD;
      pc.dlc = 4U;
      FDCAN::FDPack pfd{};
      pfd.id = 0x311U;
      pfd.type = CAN::Type::STANDARD;
      pfd.len = 8U;
      TEST_ASSERT(can1.AddMessage(pc) == ErrorCode::OK);
      TEST_ASSERT(can1.AddMessage(pfd) == ErrorCode::OK);
      TEST_ASSERT(st.transmitted.empty());
    };

    FDCAN::Configuration reconfig{};
    TEST_ASSERT(can1.SetConfig(reconfig) == ErrorCode::OK);
    TEST_ASSERT(stop_hook_ran == 1U);
    TEST_ASSERT(st.transmitted.size() == 2U);
    TEST_ASSERT(st.transmitted[0].is_fd && st.transmitted[0].id == 0x311U);
    TEST_ASSERT(!st.transmitted[1].is_fd && st.transmitted[1].id == 0x310U);
  }

  // 4c. Owner 退出窗口确定性注入：TxService 填满硬件槽后准备退出循环（GetTxFifoFreeLevel
  // 返回 0）瞬间，释放硬件槽并触发重入 AddMessage + TxBufferCompleteCallback；
  // 须零重入并由 SerializedService 退出前重检 PENDING 排空新帧。
  // Owner-exit-window deterministic injection: free HW slot + re-entrant TX IRQ right as
  // GetTxFifoFreeLevel returns 0; SerializedService pending recheck must drain before
  // exit.
  {
    auto& st = g_fdcan_state[0];
    st.Reset(2U);
    st.started = true;

    CAN::ClassicPack p0{};
    p0.id = 0x320U;
    p0.type = CAN::Type::STANDARD;
    p0.dlc = 4U;
    CAN::ClassicPack p1 = p0;
    p1.id = 0x321U;
    CAN::ClassicPack p2 = p0;
    p2.id = 0x322U;

    TEST_ASSERT(can1.AddMessage(p0) == ErrorCode::OK);
    TEST_ASSERT(st.transmitted.size() == 1U);

    uint32_t exit_hook_ran = 0U;
    st.on_zero_free_hook = [&]
    {
      ++exit_hook_ran;
      {
        std::lock_guard<std::mutex> lk(st.mtx);
        TEST_ASSERT(st.transmitted.size() == 2U);
        --st.hw_occupied;
      }
      // 此时外层 TxService 仍持有所有权，嵌套调用只能置 PENDING 而不能重入 HAL TX。
      TEST_ASSERT(can1.AddMessage(p2) == ErrorCode::OK);
      HAL_FDCAN_TxBufferCompleteCallback(&h1, 1U);
      {
        std::lock_guard<std::mutex> lk(st.mtx);
        TEST_ASSERT(st.transmitted.size() == 2U);
      }
    };

    TEST_ASSERT(can1.AddMessage(p1) == ErrorCode::OK);
    TEST_ASSERT(exit_hook_ran == 1U);
    TEST_ASSERT(st.transmitted.size() == 3U);
    TEST_ASSERT(st.transmitted[0].id == 0x320U);
    TEST_ASSERT(st.transmitted[1].id == 0x321U);
    TEST_ASSERT(st.transmitted[2].id == 0x322U);
  }

  // 5. Bus-Off 进入与退出恢复：验证 OnMessage 先于 TxService() 的回调顺序、
  // 单次失败注入仅 kick 一次（无重复重试），以及退出后自动续发滞留帧。
  // Bus-Off entry/exit: verify OnMessage-before-TxService ordering, single-kick on
  // injected HAL failure, and automatic resume without any new AddMessage().
  {
    auto& st = g_fdcan_state[0];
    st.Reset(4U);
    st.started = true;
    st.bus_off = true;
    FDCAN1->CCCR |= FDCAN_CCCR_INIT;

    struct BusOffCbProbe
    {
      FdcanStubState* st = nullptr;
      FDCAN_HandleTypeDef* h = nullptr;
      uint32_t saw_bus_off_err = 0U;
      uint32_t saw_exit_err = 0U;
      uint32_t tx_size_in_exit_cb = UINT32_MAX;
      uint32_t free_lvl_in_exit_cb = 0U;
    } probe{&st, &h1};

    auto err_cb = CAN::Callback::Create(
        [](bool, BusOffCbProbe* p_probe, const CAN::ClassicPack& p)
        {
          if (p.type != CAN::Type::ERROR)
          {
            return;
          }
          if (p.id == static_cast<uint32_t>(CAN::ErrorID::CAN_ERROR_ID_BUS_OFF))
          {
            ++p_probe->saw_bus_off_err;
          }
          else if (p.id == static_cast<uint32_t>(CAN::ErrorID::CAN_ERROR_ID_OTHER))
          {
            ++p_probe->saw_exit_err;
            p_probe->tx_size_in_exit_cb =
                static_cast<uint32_t>(p_probe->st->transmitted.size());
            p_probe->free_lvl_in_exit_cb = HAL_FDCAN_GetTxFifoFreeLevel(p_probe->h);
          }
        },
        &probe);
    can1.Register(err_cb, CAN::Type::ERROR);

    CAN::ClassicPack pc1{};
    pc1.id = 0x400U;
    pc1.type = CAN::Type::STANDARD;
    pc1.dlc = 4U;
    CAN::ClassicPack pc2 = pc1;
    pc2.id = 0x401U;
    TEST_ASSERT(can1.AddMessage(pc1) == ErrorCode::OK);
    TEST_ASSERT(can1.AddMessage(pc2) == ErrorCode::OK);
    TEST_ASSERT(st.transmitted.empty());

    // 进入 Bus-Off：清除 INIT 位并上报错误帧。
    HAL_FDCAN_ErrorStatusCallback(&h1, FDCAN_IT_BUS_OFF);
    TEST_ASSERT((FDCAN1->CCCR & FDCAN_CCCR_INIT) == 0U);
    TEST_ASSERT(probe.saw_bus_off_err == 1U);
    TEST_ASSERT(st.transmitted.empty());

    // 退出 Bus-Off：错误回调期间必须尚未续发（保持基线 OnMessage -> TxService 顺序），
    // 返回后全部滞留帧自动续发。
    st.bus_off = false;
    HAL_FDCAN_ErrorStatusCallback(&h1, FDCAN_IT_BUS_OFF);
    TEST_ASSERT(probe.saw_exit_err == 1U);
    TEST_ASSERT(probe.tx_size_in_exit_cb == 0U);
    TEST_ASSERT(probe.free_lvl_in_exit_cb == 4U);
    TEST_ASSERT(st.transmitted.size() == 2U);
    TEST_ASSERT(st.transmitted[0].id == 0x400U);
    TEST_ASSERT(st.transmitted[1].id == 0x401U);

    // 直接调用公开方法 ProcessErrorStatusInterrupt 仅上报错误帧，不引入额外发送副作用
    // （保持与基线一致；续发由 HAL_FDCAN_ErrorStatusCallback 统一在回调后触发）。
    st.transmitted.clear();
    st.hw_occupied = 0U;
    st.bus_off = true;
    CAN::ClassicPack pc3 = pc1;
    pc3.id = 0x402U;
    TEST_ASSERT(can1.AddMessage(pc3) == ErrorCode::OK);
    TEST_ASSERT(st.transmitted.empty());
    st.bus_off = false;
    can1.ProcessErrorStatusInterrupt(FDCAN_IT_BUS_OFF);
    TEST_ASSERT(probe.saw_exit_err == 2U);
    TEST_ASSERT(probe.tx_size_in_exit_cb == 0U);
    TEST_ASSERT(st.transmitted.empty());
    HAL_FDCAN_TxBufferCompleteCallback(&h1, 1U);
    TEST_ASSERT(st.transmitted.size() == 1U);
    TEST_ASSERT(st.transmitted[0].id == 0x402U);

    // 单次失败注入：HAL_FDCAN_ErrorStatusCallback 内仅触发一次 TxService()，
    // 首次提交失败后进入 retry 槽而不会在同次 IRQ 立刻二次提交。
    st.transmitted.clear();
    st.hw_occupied = 0U;
    st.bus_off = true;
    CAN::ClassicPack pc4 = pc1;
    pc4.id = 0x403U;
    TEST_ASSERT(can1.AddMessage(pc4) == ErrorCode::OK);
    st.bus_off = false;
    st.force_next_add_fail = true;
    st.add_attempt_calls = 0U;
    HAL_FDCAN_ErrorStatusCallback(&h1, FDCAN_IT_BUS_OFF);
    TEST_ASSERT(st.add_attempt_calls == 1U);
    TEST_ASSERT(st.transmitted.empty());
    TEST_ASSERT(can1.tx_classic_retry_valid_);
    TEST_ASSERT(can1.tx_classic_retry_pack_.id == 0x403U);
    HAL_FDCAN_TxBufferCompleteCallback(&h1, 1U);
    TEST_ASSERT(!can1.tx_classic_retry_valid_);
    TEST_ASSERT(st.transmitted.size() == 1U && st.transmitted[0].id == 0x403U);
  }

  // 6. 多线程压力：并发 AddMessage() 与 HAL_FDCAN_TxBufferCompleteCallback() 交错，
  // 独立 oracle 核对零丢帧、零重帧、零并发重入且同类保持 FIFO。
  // Multi-thread stress: concurrent AddMessage() vs TX complete callback.
  {
    auto& st = g_fdcan_state[0];
    st.Reset(2U);
    st.started = true;

    constexpr uint32_t N_CLASSIC = 200U;
    constexpr uint32_t N_FD = 200U;
    std::atomic<uint32_t> prod_done{0U};

    std::thread irq_thread(
        [&]
        {
          uint32_t tick = 0U;
          for (;;)
          {
            bool drained = false;
            {
              std::lock_guard<std::mutex> lk(st.mtx);
              if (st.hw_occupied > 0U)
              {
                --st.hw_occupied;
                drained = true;
              }
              else if (prod_done.load(std::memory_order_acquire) != 0U &&
                       st.transmitted.size() >= N_CLASSIC + N_FD)
              {
                break;
              }
            }
            if (drained)
            {
              if ((tick++ & 1U) == 0U)
              {
                HAL_FDCAN_TxBufferCompleteCallback(&h1, 1U);
              }
              else
              {
                HAL_FDCAN_TxFifoEmptyCallback(&h1);
              }
            }
            else
            {
              std::this_thread::yield();
            }
          }
        });

    for (uint32_t i = 0U; i < N_CLASSIC; ++i)
    {
      CAN::ClassicPack pc{};
      pc.id = 0x500U + i;
      pc.type = CAN::Type::STANDARD;
      pc.dlc = 4U;
      while (can1.AddMessage(pc) == ErrorCode::FULL)
      {
        std::this_thread::yield();
      }

      FDCAN::FDPack pfd{};
      pfd.id = 0x100U + i;
      pfd.type = CAN::Type::STANDARD;
      pfd.len = 16U;
      while (can1.AddMessage(pfd) == ErrorCode::FULL)
      {
        std::this_thread::yield();
      }
    }
    prod_done.store(1U, std::memory_order_release);
    irq_thread.join();

    std::vector<uint32_t> fd_ids;
    std::vector<uint32_t> classic_ids;
    for (const auto& rec : st.transmitted)
    {
      if (rec.is_fd)
      {
        fd_ids.push_back(rec.id);
      }
      else
      {
        classic_ids.push_back(rec.id);
      }
    }
    TEST_ASSERT(fd_ids.size() == N_FD);
    TEST_ASSERT(classic_ids.size() == N_CLASSIC);
    for (uint32_t i = 0U; i < N_FD; ++i)
    {
      TEST_ASSERT(fd_ids[i] == 0x100U + i);
      TEST_ASSERT(classic_ids[i] == 0x500U + i);
    }
  }
}
