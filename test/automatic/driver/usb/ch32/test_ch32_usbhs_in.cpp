// CH32 USBHS bulk IN endpoint (driver/ch/ch32_usb_endpoint_otghs.cpp) against a fake
// register block: the real endpoint and endpoint core, with the test playing the
// controller. With INT_BUSY the controller busy-NAKs the host's next IN while
// UIF_TRANSFER is pending, and INT_ST then reports that NAK instead of the ACKed
// packet. These cases check that the endpoint finds such a packet by its TOG_AUTO
// toggle instead of leaving it armed, which made the controller send it twice.

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

struct Fixture
{
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

}  // namespace

int main()
{
  TestTwoPacketResponse();
  TestPacketAndZlp();
  TestQueuedResponses();
  TestEp0StallClearKeepsData1();
  std::puts("CH32 USBHS IN completions found by toggle passed");
}
