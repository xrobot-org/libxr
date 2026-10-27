// CH32 USBHS bulk IN endpoint (driver/ch/ch32_usb_endpoint_otghs.cpp) against a fake
// register block: the real endpoint and endpoint core, with the test playing the
// controller. With INT_BUSY the controller busy-NAKs the host's next IN while
// UIF_TRANSFER is pending, and INT_ST then reports that NAK instead of the ACKed
// packet. These cases check that the endpoint finds such a packet by its TOG_AUTO
// toggle instead of leaving it armed, which made the controller send it twice. The
// later cases check the repair of a toggle advance lost during an arm of the other
// direction.

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "ch32_usb_endpoint.hpp"
#include "test_assert.hpp"
#include "usb/core/ep_pool.hpp"

using namespace LibXR;
using namespace LibXR::USB;

extern "C" void libxr_fatal_error(const char* file, uint32_t line, bool)
{
  std::fprintf(stderr, "%s:%u: CH32 USBHS assertion failed\n", file, line);
  std::abort();
}

namespace
{
using Direction = Endpoint::Direction;
using Completion = Endpoint::Completion;

template <size_t N>
struct alignas(64) Block
{
  uint8_t bytes[N] = {};
  RawData Raw() { return {bytes, N}; }
};

struct Recorder
{
  std::vector<Completion> list;
  Callback<const Completion&> Bind()
  {
    return Callback<const Completion&>::Create(
        [](bool, Recorder* self, const Completion& completion)
        { self->list.push_back(completion); }, this);
  }
};

uint32_t Address(const void* pointer)
{
  return static_cast<uint32_t>(reinterpret_cast<uintptr_t>(pointer));
}

/// The host ACKs the armed EP1 IN packet: TOG_AUTO advances the toggle.
void HostAcksEp1() { fake_usbhsd.UEP1_TX_CTRL ^= USBHS_UEP_T_TOG_DATA1; }

/// One UIF_TRANSFER whose INT_ST shows a busy NAK: dispatch by INT_ST does nothing for
/// it, so only the toggle check can complete the endpoint.
void InterruptWithBusyNak() { CH32EndpointOtgHs::CompleteAckedInPackets(); }

bool Armed(const void* address, uint16_t length)
{
  return (fake_usbhsd.UEP1_TX_CTRL & USBHS_UEP_T_RES_MASK) == USBHS_UEP_T_RES_ACK &&
         fake_usbhsd.UEP1_TX_DMA == Address(address) && fake_usbhsd.UEP1_TX_LEN == length;
}

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

// A 516-byte response: 512 + 4. The ACK of the first packet is reported as a busy NAK.
void TestTwoPacketResponse()
{
  Fixture f;
  const auto slot = f.Submit(516, false);
  auto* base = static_cast<uint8_t*>(slot.data.addr_);
  TEST_ASSERT(Armed(base, 512));

  // The host has not taken the packet yet: a busy NAK leaves it armed.
  InterruptWithBusyNak();
  TEST_ASSERT(Armed(base, 512) && f.rec.list.empty());

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

  // Nothing armed: further interrupts change nothing.
  HostAcksEp1();
  InterruptWithBusyNak();
  TEST_ASSERT(f.rec.list.size() == 1);
}

// Exactly one packet plus its ZLP: the ZLP is a second hardware request.
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

// Two queued responses: the second starts only after the first is complete, and the
// starting toggle of the second is the one the first left behind.
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

// EP0 after a STALL. The next SETUP restores EP0 to NAK/DATA1 in the interrupt before
// the device core clears the endpoint's halt; that clear must not reset the toggle, or
// the data stage starts with DATA0, the host drops it as a repeat and the request after
// a STALL times out.
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
  // What the SETUP interrupt writes (PrepareEp0ForSetup).
  fake_usbhsd.UEP0_TX_CTRL = USBHS_UEP_T_TOG_DATA1 | USBHS_UEP_T_RES_NAK;
  TEST_ASSERT(in0.ClearStall(false) == ErrorCode::OK);
  TEST_ASSERT(fake_usbhsd.UEP0_TX_CTRL == (USBHS_UEP_T_TOG_DATA1 | USBHS_UEP_T_RES_NAK));
}

// --- Toggle advances the controller loses while the other direction is armed ---------
// UEPn_TX_LEN, TX_CTRL and RX_CTRL share one 32-bit word: when a packet completes while
// an arm writes the other direction's registers, the controller's TOG_AUTO advance of
// that packet can be lost. The arm records the toggle from before its writes
// (tog_window_); the next good packet of that direction advances it once if it has not
// moved. Here the test sets tog_window_ the way such an arm leaves it.

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
  /// A packet of `size` bytes lands; `tog_ok` is what INT_ST reports for it.
  void HostSends(size_t size, bool tog_ok)
  {
    fake_usbhsd.INT_ST = tog_ok ? USBHS_UIS_TOG_OK : 0u;
    ep.TransferComplete(size);
  }
};

// The advance of a good OUT packet was lost: the toggle is advanced once.
void TestLostRxAdvanceRepaired()
{
  OutFixture f;
  TEST_ASSERT(RxToggle() == 0u);
  f.ep.tog_window_ = CH32EndpointOtgHs::TOG_WINDOW | 0u;
  f.HostSends(3, true);  // the controller did not move the toggle
  TEST_ASSERT(f.rec.list.size() == 1 && f.rec.list[0].actual == 3);
  TEST_ASSERT(RxToggle() == USBHS_UEP_R_TOG_DATA1 && f.ep.tog_window_ == 0u);
}

// The advance was not lost: the toggle stays where the controller put it.
void TestKeptRxAdvanceUntouched()
{
  OutFixture f;
  f.ep.tog_window_ = CH32EndpointOtgHs::TOG_WINDOW | 0u;
  fake_usbhsd.UEP1_RX_CTRL ^= USBHS_UEP_R_TOG_DATA1;  // TOG_AUTO advanced
  f.HostSends(3, true);
  TEST_ASSERT(f.rec.list.size() == 1);
  TEST_ASSERT(RxToggle() == USBHS_UEP_R_TOG_DATA1 && f.ep.tog_window_ == 0u);
}

// A resend (TOG mismatch) does not advance the toggle and ends the window without a
// repair; the request is armed again.
void TestResendEndsWindow()
{
  OutFixture f;
  f.ep.tog_window_ = CH32EndpointOtgHs::TOG_WINDOW | 0u;
  f.HostSends(3, false);
  TEST_ASSERT(f.rec.list.empty() && RxToggle() == 0u && f.ep.tog_window_ == 0u);
  TEST_ASSERT((fake_usbhsd.UEP1_RX_CTRL & USBHS_UEP_R_RES_MASK) == USBHS_UEP_R_RES_ACK);
}

// The advance of an ACKed IN packet was lost: the next response would go out with the
// old phase and be dropped by the host as a repeat, so the toggle is advanced once.
void TestLostTxAdvanceRepaired()
{
  Fixture f;
  const auto slot = f.Submit(3, false);
  TEST_ASSERT(Armed(slot.data.addr_, 3) && TxToggle() == USBHS_UEP_T_TOG_DATA0);
  f.ep.tog_window_ = CH32EndpointOtgHs::TOG_WINDOW | USBHS_UEP_T_TOG_DATA0;
  f.ep.TransferComplete(0u);  // INT_ST reports the ACK; the toggle did not move
  TEST_ASSERT(f.rec.list.size() == 1 && f.rec.list[0].actual == 3);
  TEST_ASSERT(TxToggle() == USBHS_UEP_T_TOG_DATA1 && f.ep.tog_window_ == 0u);
}

// An arm records nothing when no transaction completed during its writes, including
// when UIF_TRANSFER was already set before them (the controller busy-NAKs then).
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
  TestLostRxAdvanceRepaired();
  TestKeptRxAdvanceUntouched();
  TestResendEndsWindow();
  TestLostTxAdvanceRepaired();
  TestArmWithoutCompletionRecordsNothing();
  std::puts("CH32 USBHS IN completions and lost toggle advances passed");
}
