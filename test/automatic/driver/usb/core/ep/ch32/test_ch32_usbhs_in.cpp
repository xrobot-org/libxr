/**
 * @file test_ch32_usbhs_in.cpp
 * @brief CH32 USBHS 批量 IN 端点主机测试 / Host tests for the CH32 USBHS bulk IN
 *        endpoint.
 *
 * 在假寄存器块上运行真实的端点与端点核心（driver/ch/ch32_usb_endpoint_otghs.cpp），
 * 由测试扮演控制器。启用 INT_BUSY 时，UIF_TRANSFER 挂起期间控制器会对主机的下一个 IN
 * 回 busy NAK，此时 INT_ST 报告的是这次 NAK 而不是已 ACK 的包。这些用例检查端点能
 * 通过 TOG_AUTO 翻转找到这样的包，而不是让它保持挂包状态，否则控制器会把它发送两次。
 * 后面的用例检查 OUT 挂包期间丢失的 IN 翻转推进能得到修复，以及 OUT 端点的 toggle 由软件
 * 翻转。
 * Runs the real endpoint and endpoint core
 * (driver/ch/ch32_usb_endpoint_otghs.cpp) against a fake register block, with the test
 * playing the controller. With INT_BUSY the controller busy-NAKs the host's next IN
 * while UIF_TRANSFER is pending, and INT_ST then reports that NAK instead of the ACKed
 * packet. These cases check that the endpoint finds such a packet by its TOG_AUTO
 * toggle instead of leaving it armed, which made the controller send it twice. The
 * later cases check the repair of an IN toggle advance lost during an arm of the OUT
 * direction, and the software toggle of the OUT endpoint.
 */

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "ch32_usb_endpoint.hpp"
#include "driver/usb/core/ep/ch32/ch32_test_common.hpp"
#include "driver/usb/usb_test_common.hpp"
#include "test_assert.hpp"
#include "usb/core/ep_pool.hpp"

using namespace LibXR;
using namespace LibXR::USB;

namespace
{
using LibXR::USB::Test::Address;
using LibXR::USB::Test::Block;
using LibXR::USB::Test::Recorder;
using Direction = Endpoint::Direction;
using Completion = Endpoint::Completion;

/// 主机 ACK 已挂包的 EP1 IN 包：TOG_AUTO 使 toggle 翻转
/// The host ACKs the armed EP1 IN packet: TOG_AUTO advances the toggle.
void HostAcksEp1() { fake_usbhsd.UEP1_TX_CTRL ^= USBHS_UEP_T_TOG_DATA1; }

/// 一次 UIF_TRANSFER，其 INT_ST 显示 busy NAK：按 INT_ST 分发对它不做任何处理，所以
/// 只有 toggle 检查能完成该端点。
/// One UIF_TRANSFER whose INT_ST shows a busy NAK: dispatch by INT_ST does nothing for
/// it, so only the toggle check can complete the endpoint.
void InterruptWithBusyNak() { CH32EndpointOtgHs::CompleteAckedInPackets(); }

bool Armed(const void* address, uint16_t length)
{
  return (fake_usbhsd.UEP1_TX_CTRL & USBHS_UEP_T_RES_MASK) == USBHS_UEP_T_RES_ACK &&
         fake_usbhsd.UEP1_TX_DMA == Address(address) && fake_usbhsd.UEP1_TX_LEN == length;
}

/// 端点把自己登记在静态映射表中，该表的生命周期长于每个用例的端点；每个用例都从空
/// 映射表开始。
/// Endpoints register themselves in a static map that outlives each case's endpoints;
/// every case starts from an empty map.
struct MapReset
{
  MapReset()
  {
    for (auto& entry : CH32EndpointOtgHs::map_otg_hs_) entry[0] = entry[1] = nullptr;
  }
};

struct Fixture
{
  MapReset reset;
  Block<2048> storage;
  EndpointPool pool{2};
  CH32EndpointOtgHs ep{Endpoint::EPNumber::EP1, Direction::IN, storage.Raw()};
  Recorder rec;
  Fixture()
  {
    fake_usbhsd = {};
    pool.SetSpeed(Speed::HIGH);
    TEST_ASSERT(pool.Put(&ep) == ErrorCode::OK);
    ep.SetOnComplete(rec.Bind());
    TEST_ASSERT(ep.Configure({Direction::IN, Endpoint::Type::BULK, 512}, false) ==
                ErrorCode::OK);
    TEST_ASSERT(ep.MaxPacketSize() == 512);
  }
  Endpoint::Slot Submit(size_t length, bool zlp)
  {
    Endpoint::Slot slot;
    TEST_ASSERT(ep.AcquireSlot(slot, false) == ErrorCode::OK);
    TEST_ASSERT(ep.Submit(slot, length, zlp, false) == ErrorCode::OK);
    return slot;
  }
};

/// 516 字节的响应：512 + 4。第一个包的 ACK 被报告为 busy NAK。
/// A 516-byte response: 512 + 4. The ACK of the first packet is reported as a busy NAK.
void TestTwoPacketResponse()
{
  Fixture f;
  const auto slot = f.Submit(516, false);
  auto* base = static_cast<uint8_t*>(slot.data.addr_);
  TEST_ASSERT(Armed(base, 512));

  // 主机尚未取走该包：busy NAK 使它保持挂包。
  // The host has not taken the packet yet: a busy NAK leaves it armed.
  InterruptWithBusyNak();
  TEST_ASSERT(Armed(base, 512) && f.rec.list.empty());

  // 第一个包已 ACK，但 INT_ST 显示的是主机下一个 IN 的 NAK。若保持挂包，
  // 控制器会再次发送 base[0..512)。
  // The first packet is ACKed but INT_ST shows the NAK of the host's next IN. Left
  // armed, the controller would send base[0..512) again.
  HostAcksEp1();
  InterruptWithBusyNak();
  TEST_ASSERT(Armed(base + 512, 4));
  TEST_ASSERT(f.rec.list.empty());

  HostAcksEp1();
  InterruptWithBusyNak();
  TEST_ASSERT(f.rec.list.size() == 1);
  TEST_ASSERT(f.rec.list[0].result == ErrorCode::OK && f.rec.list[0].actual == 516);
  TEST_ASSERT((fake_usbhsd.UEP1_TX_CTRL & USBHS_UEP_T_RES_MASK) == USBHS_UEP_T_RES_NAK);

  // 没有挂包：后续中断不产生任何变化。
  // Nothing armed: further interrupts change nothing.
  HostAcksEp1();
  InterruptWithBusyNak();
  TEST_ASSERT(f.rec.list.size() == 1);
}

/// 恰好一个包加其 ZLP：ZLP 是第二次硬件请求。
/// Exactly one packet plus its ZLP: the ZLP is a second hardware request.
void TestPacketAndZlp()
{
  Fixture f;
  const auto slot = f.Submit(512, true);
  auto* base = static_cast<uint8_t*>(slot.data.addr_);
  TEST_ASSERT(Armed(base, 512));
  HostAcksEp1();
  InterruptWithBusyNak();
  TEST_ASSERT(fake_usbhsd.UEP1_TX_LEN == 0 &&
              (fake_usbhsd.UEP1_TX_CTRL & USBHS_UEP_T_RES_MASK) == USBHS_UEP_T_RES_ACK);
  TEST_ASSERT(f.rec.list.empty());
  InterruptWithBusyNak();
  TEST_ASSERT(f.rec.list.empty());
  HostAcksEp1();
  InterruptWithBusyNak();
  TEST_ASSERT(f.rec.list.size() == 1);
  TEST_ASSERT(f.rec.list[0].result == ErrorCode::OK && f.rec.list[0].actual == 512);
}

/// 两个排队的响应：第二个在第一个完成后才启动，其起始 toggle 是第一个留下的值。
/// Two queued responses: the second starts only after the first is complete, and the
/// starting toggle of the second is the one the first left behind.
void TestQueuedResponses()
{
  Fixture f;
  const auto a = f.Submit(1024, false);
  const auto b = f.Submit(3, false);
  auto* base_a = static_cast<uint8_t*>(a.data.addr_);
  TEST_ASSERT(Armed(base_a, 512));
  HostAcksEp1();
  InterruptWithBusyNak();
  TEST_ASSERT(Armed(base_a + 512, 512));
  HostAcksEp1();
  InterruptWithBusyNak();
  TEST_ASSERT(f.rec.list.size() == 1 && f.rec.list[0].actual == 1024);
  TEST_ASSERT(Armed(b.data.addr_, 3));
  InterruptWithBusyNak();
  TEST_ASSERT(f.rec.list.size() == 1);
  HostAcksEp1();
  InterruptWithBusyNak();
  TEST_ASSERT(f.rec.list.size() == 2 && f.rec.list[1].actual == 3);
}

/// STALL 之后的 EP0。下一个 SETUP 在设备核心清除端点 halt 之前，先在中断里把 EP0
/// 恢复为 NAK/DATA1；该清除不能复位 toggle，否则数据阶段以 DATA0 开始，主机把它当作
/// 重复包丢弃，STALL 之后的请求就会超时。
/// EP0 after a STALL. The next SETUP restores EP0 to NAK/DATA1 in the interrupt before
/// the device core clears the endpoint's halt; that clear must not reset the toggle, or
/// the data stage starts with DATA0, the host drops it as a repeat and the request after
/// a STALL times out.
void TestEp0StallClearKeepsData1()
{
  fake_usbhsd = {};
  Block<128> in_storage;
  EndpointPool pool{2};
  CH32EndpointOtgHs in0{Endpoint::EPNumber::EP0, Direction::IN, in_storage.Raw()};
  pool.SetSpeed(Speed::HIGH);
  TEST_ASSERT(pool.Put(&in0) == ErrorCode::OK);
  TEST_ASSERT(in0.Configure({Direction::IN, Endpoint::Type::CONTROL, 64}, false) ==
              ErrorCode::OK);

  TEST_ASSERT(in0.Stall(false) == ErrorCode::OK);
  TEST_ASSERT((fake_usbhsd.UEP0_TX_CTRL & USBHS_UEP_T_RES_MASK) == USBHS_UEP_T_RES_STALL);
  // SETUP 中断写入的值（PrepareEp0ForSetup）。
  // What the SETUP interrupt writes (PrepareEp0ForSetup).
  fake_usbhsd.UEP0_TX_CTRL = USBHS_UEP_T_TOG_DATA1 | USBHS_UEP_T_RES_NAK;
  TEST_ASSERT(in0.ClearStall(false) == ErrorCode::OK);
  TEST_ASSERT(fake_usbhsd.UEP0_TX_CTRL == (USBHS_UEP_T_TOG_DATA1 | USBHS_UEP_T_RES_NAK));
}

// --- Toggle ---
// IN 的 toggle 由控制器推进（TOG_AUTO）。UEPn_TX_LEN、TX_CTRL 和 RX_CTRL 共用一个 32 位
// 字：一个 IN 包完成的同时，有一次挂包在写 OUT 方向的寄存器，这次推进可能丢失。挂包会
// 记录写入之前的 toggle（tog_window_）；下一个正常包到来时若 toggle 没有变化，就推进
// 一次。这里由测试按这样的挂包留下的样子设置 tog_window_。
// BULK/INTERRUPT OUT 的 toggle 由软件在每个 TOG_OK 包之后推进：开着 TOG_AUTO 时，一个
// SOF 会让控制器翻转端点号等于帧号高四位的那个端点的 RX toggle。
// The IN toggle is advanced by the controller (TOG_AUTO). UEPn_TX_LEN, TX_CTRL and
// RX_CTRL share one 32-bit word: when an IN packet completes while an arm writes the OUT
// direction's registers, that advance can be lost. The arm records the toggle from
// before its writes (tog_window_); the next good packet advances it once if it has not
// moved. Here the test sets tog_window_ the way such an arm leaves it.
// The BULK/INTERRUPT OUT toggle is advanced by software after each TOG_OK packet: with
// TOG_AUTO a SOF would make the controller flip the RX toggle of the endpoint numbered
// like the top four bits of the frame number.

uint8_t RxToggle() { return fake_usbhsd.UEP1_RX_CTRL & USBHS_UEP_R_TOG_MASK; }
uint8_t TxToggle() { return fake_usbhsd.UEP1_TX_CTRL & USBHS_UEP_T_TOG_MASK; }

struct OutFixture
{
  MapReset reset;
  Block<2048> storage;
  EndpointPool pool{2};
  CH32EndpointOtgHs ep{Endpoint::EPNumber::EP1, Direction::OUT, storage.Raw()};
  Recorder rec;
  OutFixture()
  {
    fake_usbhsd = {};
    pool.SetSpeed(Speed::HIGH);
    TEST_ASSERT(pool.Put(&ep) == ErrorCode::OK);
    ep.SetOnComplete(rec.Bind());
    TEST_ASSERT(ep.Configure({Direction::OUT, Endpoint::Type::BULK, 512}, false) ==
                ErrorCode::OK);
    TEST_ASSERT(ep.Receive(512, false) == ErrorCode::OK);
  }
  /// 一个 `size` 字节的包到达；`tog_ok` 是 INT_ST 对它报告的结果。
  /// A packet of `size` bytes lands; `tog_ok` is what INT_ST reports for it.
  void HostSends(size_t size, bool tog_ok)
  {
    fake_usbhsd.INT_ST = tog_ok ? USBHS_UIS_TOG_OK : 0u;
    ep.TransferComplete(size);
  }
};

/// OUT 端点不开 TOG_AUTO，软件在每个 TOG_OK 包之后翻转 toggle，一个包翻转一次。
/// The OUT endpoint runs without TOG_AUTO and software flips the toggle after a TOG_OK
/// packet, one flip per packet.
void TestOutPacketFlipsTheToggle()
{
  OutFixture f;
  TEST_ASSERT((fake_usbhsd.UEP1_RX_CTRL & USBHS_UEP_R_TOG_AUTO) == 0u);
  TEST_ASSERT(RxToggle() == USBHS_UEP_R_TOG_DATA0);
  f.HostSends(3, true);
  TEST_ASSERT(f.rec.list.size() == 1 && f.rec.list[0].actual == 3);
  TEST_ASSERT(RxToggle() == USBHS_UEP_R_TOG_DATA1);
  TEST_ASSERT(f.ep.Receive(512, false) == ErrorCode::OK);
  // 挂包不动 toggle / The arm keeps the toggle.
  TEST_ASSERT(RxToggle() == USBHS_UEP_R_TOG_DATA1);
  f.HostSends(4, true);
  TEST_ASSERT(f.rec.list.size() == 2 && RxToggle() == USBHS_UEP_R_TOG_DATA0);
  TEST_ASSERT((fake_usbhsd.UEP1_RX_CTRL & USBHS_UEP_R_TOG_AUTO) == 0u);
}

/// 重发（TOG_OK=0）不翻转 toggle，也不上报；请求被重新挂包。
/// A resend (TOG_OK=0) does not flip the toggle and nothing is reported; the request is
/// armed again.
void TestResendDoesNotFlipTheToggle()
{
  OutFixture f;
  f.HostSends(3, true);
  const uint8_t after_first = RxToggle();
  TEST_ASSERT(f.ep.Receive(512, false) == ErrorCode::OK);
  f.HostSends(3, false);
  TEST_ASSERT(f.rec.list.size() == 1 && RxToggle() == after_first);
  TEST_ASSERT((fake_usbhsd.UEP1_RX_CTRL & USBHS_UEP_R_RES_MASK) == USBHS_UEP_R_RES_ACK);
}

/// 请求停止之后才收下的包（最后一个事务仍会完成）已经向主机回了 ACK，主机已推进相位：
/// 先翻转 toggle，再丢弃这次完成。停止之后的重发不翻转。
/// A packet accepted after the request was stopped (the last transaction still completes)
/// was ACKed to the host, which advanced its phase: the toggle is flipped first and the
/// completion dropped. A resend after a stop does not flip.
void TestLateCompletionAfterStopFlipsTheToggle()
{
  OutFixture f;
  f.ep.StopHardware(false);
  TEST_ASSERT(RxToggle() == USBHS_UEP_R_TOG_DATA0);
  f.HostSends(5, true);
  TEST_ASSERT(f.rec.list.empty());
  TEST_ASSERT(RxToggle() == USBHS_UEP_R_TOG_DATA1);
  f.HostSends(5, false);
  TEST_ASSERT(f.rec.list.empty() && RxToggle() == USBHS_UEP_R_TOG_DATA1);
}

/// 清除 STALL 让 toggle 从 DATA0 重新开始，仍然不开 TOG_AUTO。
/// The stall clear starts the toggle at DATA0 again, still without TOG_AUTO.
void TestStallClearRestartsTheOutToggle()
{
  OutFixture f;
  f.HostSends(3, true);
  TEST_ASSERT(RxToggle() == USBHS_UEP_R_TOG_DATA1);
  TEST_ASSERT(f.ep.ClearStallHardware(false) == ErrorCode::OK);
  TEST_ASSERT(RxToggle() == USBHS_UEP_R_TOG_DATA0);
  TEST_ASSERT((fake_usbhsd.UEP1_RX_CTRL & USBHS_UEP_R_TOG_AUTO) == 0u);
}

/// 多包 OUT 请求：主机重发一个已被取走的包（它没收到 ACK）。重发的包被丢弃，接收按
/// 原样重新挂包：请求、DMA 位置和已接收计数保持不变，所以之前已接受的包仍然有效。
/// 此前从头重启请求会丢掉这些包。
/// A multi-packet OUT request: the host resends a packet that was already taken (it
/// missed the ACK). The resend is dropped and the receive armed again as it was: the
/// request, the DMA position and the received count stay, so the packets accepted earlier
/// still count. Restarting the request from its start lost them.
void TestResendKeepsAcceptedPackets()
{
  MapReset reset;
  fake_usbhsd = {};
  Block<2048> storage;
  EndpointPool pool{2};
  CH32EndpointOtgHs ep{Endpoint::EPNumber::EP1, Direction::OUT, storage.Raw()};
  Recorder rec;
  pool.SetSpeed(Speed::HIGH);
  TEST_ASSERT(pool.Put(&ep) == ErrorCode::OK);
  ep.SetOnComplete(rec.Bind());
  TEST_ASSERT(ep.Configure({Direction::OUT, Endpoint::Type::BULK, 512}, false) ==
              ErrorCode::OK);
  TEST_ASSERT(ep.Receive(1024, false) == ErrorCode::OK);
  const uint32_t first_dma = fake_usbhsd.UEP1_RX_DMA;

  fake_usbhsd.INT_ST = USBHS_UIS_TOG_OK;
  ep.TransferComplete(512);  // 第一个包已取走 / first packet taken
  TEST_ASSERT(rec.list.empty());
  TEST_ASSERT(RxToggle() == USBHS_UEP_R_TOG_DATA1);
  const uint32_t second_dma = fake_usbhsd.UEP1_RX_DMA;
  TEST_ASSERT(second_dma == first_dma + 512u);

  // 主机重发：TOG 不匹配 / The host resends: TOG mismatch.
  fake_usbhsd.INT_ST = 0u;
  ep.TransferComplete(512);
  TEST_ASSERT(rec.list.empty() && RxToggle() == USBHS_UEP_R_TOG_DATA1);
  TEST_ASSERT(fake_usbhsd.UEP1_RX_DMA == second_dma);
  TEST_ASSERT((fake_usbhsd.UEP1_RX_CTRL & USBHS_UEP_R_RES_MASK) == USBHS_UEP_R_RES_ACK);

  fake_usbhsd.INT_ST = USBHS_UIS_TOG_OK;
  ep.TransferComplete(512);  // 第二个包已取走 / second packet taken
  TEST_ASSERT(rec.list.size() == 1 && rec.list[0].actual == 1024);
  TEST_ASSERT(RxToggle() == USBHS_UEP_R_TOG_DATA0);
}

/// 一个已 ACK 的 IN 包的推进丢失了：下一个响应会以旧的相位发出，被主机当作重复包
/// 丢弃，所以 toggle 被推进一次。
/// The advance of an ACKed IN packet was lost: the next response would go out with the
/// old phase and be dropped by the host as a repeat, so the toggle is advanced once.
void TestLostTxAdvanceRepaired()
{
  Fixture f;
  const auto slot = f.Submit(3, false);
  TEST_ASSERT(Armed(slot.data.addr_, 3) && TxToggle() == USBHS_UEP_T_TOG_DATA0);
  f.ep.tog_window_ = CH32EndpointOtgHs::TOG_WINDOW | USBHS_UEP_T_TOG_DATA0;
  // INT_ST 报告了 ACK，toggle 没有改变。
  // INT_ST reports the ACK; the toggle did not move.
  f.ep.TransferComplete(0u);
  TEST_ASSERT(f.rec.list.size() == 1 && f.rec.list[0].actual == 3);
  TEST_ASSERT(TxToggle() == USBHS_UEP_T_TOG_DATA1 && f.ep.tog_window_ == 0u);
}

/// 若挂包写入期间没有事务完成，则挂包什么也不记录，包括 UIF_TRANSFER 在写入之前就已
/// 置位的情况（此时控制器回 busy NAK）。
/// An arm records nothing when no transaction completed during its writes, including
/// when UIF_TRANSFER was already set before them (the controller busy-NAKs then).
void TestArmWithoutCompletionRecordsNothing()
{
  fake_usbhsd = {};
  MapReset reset;
  Block<2048> in_storage;
  Block<2048> out_storage;
  EndpointPool pool{3};
  CH32EndpointOtgHs in{Endpoint::EPNumber::EP1, Direction::IN, in_storage.Raw()};
  CH32EndpointOtgHs out{Endpoint::EPNumber::EP1, Direction::OUT, out_storage.Raw()};
  pool.SetSpeed(Speed::HIGH);
  TEST_ASSERT(pool.Put(&in) == ErrorCode::OK && pool.Put(&out) == ErrorCode::OK);
  TEST_ASSERT(in.Configure({Direction::IN, Endpoint::Type::BULK, 512}, false) ==
              ErrorCode::OK);
  TEST_ASSERT(out.Configure({Direction::OUT, Endpoint::Type::BULK, 512}, false) ==
              ErrorCode::OK);
  Endpoint::Slot slot;
  TEST_ASSERT(in.AcquireSlot(slot, false) == ErrorCode::OK);
  TEST_ASSERT(in.Submit(slot, 3, false, false) == ErrorCode::OK);
  TEST_ASSERT(out.tog_window_ == 0u);
  in.TransferComplete(0u);
  fake_usbhsd.INT_FG = USBHS_UIF_TRANSFER;
  TEST_ASSERT(out.Receive(512, false) == ErrorCode::OK);
  TEST_ASSERT(in.tog_window_ == 0u);
}

}  // namespace

int main()
{
  TestTwoPacketResponse();
  TestPacketAndZlp();
  TestQueuedResponses();
  TestEp0StallClearKeepsData1();
  TestOutPacketFlipsTheToggle();
  TestResendDoesNotFlipTheToggle();
  TestLateCompletionAfterStopFlipsTheToggle();
  TestStallClearRestartsTheOutToggle();
  TestResendKeepsAcceptedPackets();
  TestLostTxAdvanceRepaired();
  TestArmWithoutCompletionRecordsNothing();
  std::puts("CH32 USBHS IN completions and lost toggle advances passed");
}
