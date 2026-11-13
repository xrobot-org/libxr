// CH32 USBFS bulk endpoints (driver/ch/ch32_usb_endpoint_otgfs.cpp) against a fake
// register block: the real endpoint and endpoint core, with the test playing the
// controller. The cases cover a shared bidirectional endpoint (one 128 byte block: OUT in
// the first 64 bytes, IN fixed at +64), the receive of a multi-packet request and a
// control write on EP0.

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

/// Endpoints register themselves in a static map that outlives each case's endpoints;
/// every case starts from an empty map.
struct MapReset
{
  MapReset()
  {
    fake_usbfsd = {};
    for (auto& entry : CH32EndpointOtgFs::map_otg_fs_) entry[0] = entry[1] = nullptr;
  }
};

/// A shared bidirectional EP1: both direction objects receive the whole 128 byte block,
/// as CH32USBOtgFS passes it.
struct SharedFixture
{
  MapReset reset;
  Block<128> block;
  EndpointPool pool{3};
  CH32EndpointOtgFs out{Endpoint::EPNumber::EP1, Direction::OUT, block.Raw(), false};
  CH32EndpointOtgFs in{Endpoint::EPNumber::EP1, Direction::IN, block.Raw(), false};
  Recorder out_rec;
  Recorder in_rec;
  explicit SharedFixture(uint16_t in_packet)
  {
    pool.SetSpeed(Speed::FULL);
    TEST_ASSERT(pool.Put(&out) == ErrorCode::OK && pool.Put(&in) == ErrorCode::OK);
    out.SetOnComplete(out_rec.Bind());
    in.SetOnComplete(in_rec.Bind());
    TEST_ASSERT(out.Configure({Direction::OUT, Endpoint::Type::BULK, 64}, false) ==
                ErrorCode::OK);
    TEST_ASSERT(in.Configure({Direction::IN, Endpoint::Type::BULK, in_packet}, false) ==
                ErrorCode::OK);
  }
};

// UEPn_DMA points at the block base for both directions: the hardware puts OUT at the
// base and IN at +64. The IN object used to step back 64 bytes from the block it was
// given, which put the DMA base 64 bytes before the block.
void TestSharedDmaBase()
{
  SharedFixture f(64);
  TEST_ASSERT(fake_usbfsd.UEP1_DMA == Address(f.block.bytes.data()));
  TEST_ASSERT(f.in.MaxPacketSize() == 64);
}

// A response in packets below 64 bytes in the shared IN area: the next packet is moved to
// the start of the IN area before it goes out, and the request completes once, after the
// last packet. The IN area used to send only the first packet.
void TestSharedInSmallPackets()
{
  SharedFixture f(32);
  Endpoint::Slot slot;
  TEST_ASSERT(f.in.AcquireSlot(slot, false) == ErrorCode::OK);
  auto* data = static_cast<uint8_t*>(slot.data.addr_);
  TEST_ASSERT(data == f.block.bytes.data() + 64);
  for (size_t i = 0; i < 64; ++i) data[i] = static_cast<uint8_t>(0x10 + i);
  TEST_ASSERT(f.in.Submit(slot, 64, false, false) == ErrorCode::OK);
  TEST_ASSERT(fake_usbfsd.UEP1_TX_LEN == 32);

  f.in.TransferComplete(0u);  // the host took packet 1
  TEST_ASSERT(f.in_rec.list.empty());
  TEST_ASSERT(fake_usbfsd.UEP1_TX_LEN == 32);
  TEST_ASSERT((fake_usbfsd.UEP1_TX_CTRL & USBFS_UEP_T_RES_MASK) == USBFS_UEP_T_RES_ACK);
  TEST_ASSERT(f.block.bytes[64] == 0x10 + 32 && f.block.bytes[64 + 31] == 0x10 + 63);

  f.in.TransferComplete(0u);  // the host took packet 2
  TEST_ASSERT(f.in_rec.list.size() == 1);
  TEST_ASSERT(f.in_rec.list[0].result == ErrorCode::OK && f.in_rec.list[0].actual == 64);
  TEST_ASSERT((fake_usbfsd.UEP1_TX_CTRL & USBFS_UEP_T_RES_MASK) == USBFS_UEP_T_RES_NAK);
}

// A multi-packet OUT request in single-direction mode: the host resends a packet that was
// already taken (it missed the ACK). The resend is dropped and the receive armed again
// as it was; the DMA position and the received count stay.
void TestResendKeepsAcceptedPackets()
{
  MapReset reset;
  Block<256> storage;  // two 128 byte slots
  EndpointPool pool{2};
  CH32EndpointOtgFs ep{Endpoint::EPNumber::EP1, Direction::OUT, storage.Raw(), true};
  Recorder rec;
  pool.SetSpeed(Speed::FULL);
  TEST_ASSERT(pool.Put(&ep) == ErrorCode::OK);
  ep.SetOnComplete(rec.Bind());
  TEST_ASSERT(ep.Configure({Direction::OUT, Endpoint::Type::BULK, 64}, false) ==
              ErrorCode::OK);
  TEST_ASSERT(ep.Receive(128, false) == ErrorCode::OK);

  fake_usbfsd.INT_ST = USBFS_U_TOG_OK;
  ep.TransferComplete(64);  // first packet taken
  TEST_ASSERT(rec.list.empty());
  const uint32_t second_dma = fake_usbfsd.UEP1_DMA;

  fake_usbfsd.INT_ST = 0u;  // the host resends: toggle mismatch
  ep.TransferComplete(64);
  TEST_ASSERT(rec.list.empty());
  TEST_ASSERT(fake_usbfsd.UEP1_DMA == second_dma);
  TEST_ASSERT((fake_usbfsd.UEP1_RX_CTRL & USBFS_UEP_R_RES_MASK) == USBFS_UEP_R_RES_ACK);

  fake_usbfsd.INT_ST = USBFS_U_TOG_OK;
  ep.TransferComplete(64);  // second packet taken
  TEST_ASSERT(rec.list.size() == 1 && rec.list[0].actual == 128);
}

// A control write of two packets on EP0. After the first packet the receive stays at NAK
// until the core arms the next one: once the interrupt flag is cleared INT_BUSY holds
// nothing off, and at ACK the hardware would acknowledge the next packet with nobody to
// take it. The second packet is armed as DATA0.
void TestEp0OutNakBetweenPackets()
{
  MapReset reset;
  Block<64> storage;
  EndpointPool pool{2};
  CH32EndpointOtgFs ep{Endpoint::EPNumber::EP0, Direction::OUT, storage.Raw(), false};
  Recorder rec;
  pool.SetSpeed(Speed::FULL);
  TEST_ASSERT(pool.Put(&ep) == ErrorCode::OK);
  ep.SetOnComplete(rec.Bind());
  TEST_ASSERT(ep.Configure({Direction::OUT, Endpoint::Type::CONTROL, 64}, false) ==
              ErrorCode::OK);
  ep.tog_ = true;  // the first packet after SETUP is DATA1
  TEST_ASSERT(ep.Receive(64, false) == ErrorCode::OK);
  TEST_ASSERT((fake_usbfsd.UEP0_RX_CTRL & USBFS_UEP_R_RES_MASK) == USBFS_UEP_R_RES_ACK);
  TEST_ASSERT((fake_usbfsd.UEP0_RX_CTRL & USBFS_UEP_R_TOG) != 0u);

  fake_usbfsd.INT_ST = USBFS_U_TOG_OK;
  ep.TransferComplete(64);
  TEST_ASSERT(rec.list.size() == 1 && rec.list[0].actual == 64);
  TEST_ASSERT((fake_usbfsd.UEP0_RX_CTRL & USBFS_UEP_R_RES_MASK) == USBFS_UEP_R_RES_NAK);

  TEST_ASSERT(ep.ReleaseSlot(rec.list[0].slot, false) == ErrorCode::OK);
  TEST_ASSERT(ep.Receive(64, false) == ErrorCode::OK);
  TEST_ASSERT((fake_usbfsd.UEP0_RX_CTRL & USBFS_UEP_R_RES_MASK) == USBFS_UEP_R_RES_ACK);
  TEST_ASSERT((fake_usbfsd.UEP0_RX_CTRL & USBFS_UEP_R_TOG) == 0u);
}
}  // namespace

int main()
{
  TestSharedDmaBase();
  TestSharedInSmallPackets();
  TestResendKeepsAcceptedPackets();
  TestEp0OutNakBetweenPackets();
  std::puts("CH32 USBFS shared block, small IN packets, OUT resend and EP0 OUT passed");
}
