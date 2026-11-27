/**
 * @file test_ch32_usb.cpp
 * @brief CH32 USB 后端的主机测试 / Host tests of the CH32 USB backend
 *
 * 真实的 CH32USBOtgHS、CH32USBOtgFS 和核心跑在假寄存器块上，测试扮演控制器和
 * 主机：置 INT_FG/INT_ST/RX_LEN，挂起中断线，再调用中断函数。覆盖：停止收下已锁存的
 * 完成并算出真实字节数，EP0 的完成先于同一轮的 SETUP，等时一次一包，SIE 忙时停止不等、
 * 之后由事件、SIE 空闲、总线复位或时间结束。
 * The real CH32USBOtgHS, CH32USBOtgFS and the core run on fake register blocks; the
 * test plays the controller and the host: it sets INT_FG/INT_ST/RX_LEN, pends the line
 * and calls the interrupt handler. Covered: a stop collects a latched completion and
 * computes the real byte count, EP0 completions come before a SETUP of the same round,
 * one isochronous packet per request, and a stop with a busy SIE does not wait and ends
 * later on an event, a free SIE, a bus reset or by time.
 */

#include <sys/wait.h>
#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <functional>
#include <thread>
#include <vector>

#include "ch32_usb_dev.hpp"
#include "libxr_assert.hpp"
#include "test_assert.hpp"

extern "C" void USBHS_IRQHandler(void);
extern "C" void USBFS_IRQHandler(void);

using namespace LibXR;
using LibXR::USB::DeviceClass;
using LibXR::USB::Endpoint;
using LibXR::USB::EndpointPool;
using LibXR::USB::SetupPacket;
using LibXR::USB::Speed;
using Direction = Endpoint::Direction;
using EPNumber = Endpoint::EPNumber;
using Type = Endpoint::Type;
using Completion = Endpoint::Completion;
using Bytes = std::vector<uint8_t>;

namespace
{

constexpr auto LANGUAGE = USB::DescriptorStrings::MakeLanguagePack(
    USB::DescriptorStrings::Language::EN_US, "XRobot", "CH32 USB", "0001");

constexpr uint8_t REQ_OUT = 0x01U;  ///< 类 OUT 请求 / Class OUT request
constexpr uint8_t REQ_IN = 0x02U;   ///< 类 IN 请求 / Class IN request

/// 一次完成的记录 / Record of one completion
struct Done
{
  ErrorCode result;
  size_t actual;
  Bytes data;
  bool in_isr;
};

/// 一个 IN 端点、一个 OUT 端点的测试类 / Test class with one IN and one OUT endpoint
class PairClass final : public DeviceClass
{
 public:
  PairClass(EPNumber in, EPNumber out, Type in_type, uint16_t bound)
      : in_num_(in), out_num_(out), in_type_(in_type), bound_(bound)
  {
    in_cb_ = Callback<const Completion&>::Create(
        [](bool isr, PairClass* self, const Completion& c)
        {
          self->in_done.push_back({c.result, c.actual, {}, isr});
          if (self->on_in)
          {
            self->on_in();
          }
        },
        this);
    out_cb_ = Callback<const Completion&>::Create(
        [](bool isr, PairClass* self, const Completion& c)
        {
          Done d{c.result, c.actual, {}, isr};
          if (c.slot.Valid())
          {
            const auto* p = static_cast<const uint8_t*>(c.slot.data.addr_);
            d.data.assign(p, p + c.slot.data.size_);
            (void)self->out->ReleaseSlot(c.slot);
          }
          self->out_done.push_back(d);
          if (self->on_out)
          {
            self->on_out();
          }
        },
        this);
  }

  /// IN：借槽、填 length 字节、提交 / IN: borrow, fill length bytes, submit
  void Send(size_t length)
  {
    Endpoint::Slot slot;
    TEST_ASSERT(in->AcquireSlot(slot) == ErrorCode::OK);
    std::memset(slot.data.addr_, 0x5a, length);
    TEST_ASSERT(in->Submit(slot, length, false, false) == ErrorCode::OK);
  }

  Endpoint* in = nullptr;
  Endpoint* out = nullptr;
  std::vector<Done> in_done;
  std::vector<Done> out_done;
  Bytes data_seen;
  size_t completes = 0;
  size_t aborts = 0;
  std::vector<uint16_t> frames;
  uint8_t answer[4] = {1U, 2U, 3U, 4U};
  std::function<void()> on_in;  ///< IN 完成回调里再做的事 / Extra work in the IN callback
  std::function<void()>
      on_out;  ///< OUT 完成回调里再做的事 / Extra work in the OUT callback

 protected:
  size_t GetInterfaceCount() const override { return 1U; }
  bool HasIAD() const override { return false; }
  size_t GetMaxConfigSize() const override { return sizeof(desc_); }

  ErrorCode WriteDescriptors(const EndpointPool& pool, Speed speed,
                             uint8_t first_interface, ConstRawData& block) override
  {
    const uint16_t in_size =
        pool.PacketSizeFor(in_num_, {Direction::IN, in_type_, bound_}, speed);
    const uint16_t out_size =
        pool.PacketSizeFor(out_num_, {Direction::OUT, Type::BULK, bound_}, speed);
    if (in_size == 0U || out_size == 0U)
    {
      return ErrorCode::NOT_SUPPORT;
    }
    const uint8_t init[sizeof(desc_)] = {
        9U,
        4U,
        first_interface,
        0U,
        2U,
        0xffU,
        0U,
        0U,
        0U,
        7U,
        5U,
        static_cast<uint8_t>(0x80U | static_cast<uint8_t>(in_num_)),
        static_cast<uint8_t>(in_type_),
        static_cast<uint8_t>(in_size),
        static_cast<uint8_t>(in_size >> 8U),
        1U,
        7U,
        5U,
        static_cast<uint8_t>(out_num_),
        2U,
        static_cast<uint8_t>(out_size),
        static_cast<uint8_t>(out_size >> 8U),
        0U};
    std::memcpy(desc_, init, sizeof(desc_));
    first_interface_ = first_interface;
    block = {desc_, sizeof(desc_)};
    return ErrorCode::OK;
  }

  ErrorCode BindEndpoints(EndpointPool& pool, uint8_t, bool in_isr) override
  {
    TEST_ASSERT(pool.Get(in, Direction::IN, in_num_) == ErrorCode::OK);
    TEST_ASSERT(pool.Get(out, Direction::OUT, out_num_) == ErrorCode::OK);
    in->SetOnComplete(in_cb_);
    out->SetOnComplete(out_cb_);
    TEST_ASSERT(in->Configure({Direction::IN, in_type_, bound_}, in_isr) ==
                ErrorCode::OK);
    TEST_ASSERT(out->Configure({Direction::OUT, Type::BULK, bound_}, in_isr) ==
                ErrorCode::OK);
    return ErrorCode::OK;
  }

  void UnbindEndpoints(EndpointPool& pool, bool in_isr) override
  {
    for (Endpoint** ep : {&in, &out})
    {
      if (*ep != nullptr)
      {
        Endpoint* e = *ep;
        *ep = nullptr;
        e->Close(in_isr);
        (void)pool.Release(e);
      }
    }
  }

  ErrorCode OnControlRequest(bool, const SetupPacket& setup,
                             ConstRawData& in_data) override
  {
    if ((setup.bmRequestType & 0x60U) != 0x20U)
    {
      return ErrorCode::NOT_SUPPORT;
    }
    if (setup.bRequest == REQ_IN)
    {
      in_data = {answer, sizeof(answer)};
      return ErrorCode::OK;
    }
    return setup.bRequest == REQ_OUT ? ErrorCode::OK : ErrorCode::NOT_SUPPORT;
  }

  ErrorCode OnControlData(bool, const SetupPacket&, ConstRawData data) override
  {
    const auto* p = static_cast<const uint8_t*>(data.addr_);
    data_seen.assign(p, p + data.size_);
    return ErrorCode::OK;
  }

  void OnControlComplete(bool, const SetupPacket&) override { ++completes; }
  void OnControlAbort(bool, const SetupPacket&) override { ++aborts; }

  void OnStartOfFrame(bool, uint16_t frame, uint8_t microframe) override
  {
    frames.push_back(static_cast<uint16_t>(frame | (microframe << 12U)));
  }

 private:
  EPNumber in_num_;
  EPNumber out_num_;
  Type in_type_;
  uint16_t bound_;
  uint8_t first_interface_ = 0U;
  uint8_t desc_[23] = {};
  Callback<const Completion&> in_cb_;
  Callback<const Completion&> out_cb_;
};

/// 测试可以直接复位控制器（扮演 owner 处理 STOP_FAILED_EVENT）/ The test may reset the
/// controller directly (playing the owner on STOP_FAILED_EVENT)
template <typename Usb>
class ResettableUsb : public Usb
{
 public:
  using Usb::Usb;
  void ForceReset() { this->ResetController(true); }
};

/// 节拍 n 次，每次之后让挂起的中断运行 / Tick n times and let a pended interrupt run
/// after each
template <typename Pended>
void TickAnd(int count, Pended pended)
{
  for (int i = 0; i < count; ++i)
  {
    DriverAsyncCondition::OnTick(1U);
    pended();
  }
}

/// 按 32 字节对齐的块 / Block aligned to 32 bytes
template <size_t N>
struct alignas(64) Block
{
  uint8_t bytes[N];
  RawData Raw() { return {bytes, N}; }
};

uint32_t Addr(const void* p)
{
  return static_cast<uint32_t>(reinterpret_cast<uintptr_t>(p));
}

// ---------------------------------------------------------------------------------------
// USBHS
// ---------------------------------------------------------------------------------------

/// 控制器举起中断线；中断函数运行到线不再挂起 / The controller raises its line; the
/// handler runs until the line is no longer pending
void RunHs()
{
  fake_pfic.pending[USBHS_IRQn] = true;
  for (int guard = 0; fake_pfic.pending[USBHS_IRQn] && fake_pfic.enabled[USBHS_IRQn];
       ++guard)
  {
    TEST_ASSERT(guard < 100);
    fake_pfic.pending[USBHS_IRQn] = false;
    USBHS_IRQHandler();
  }
}

/// 应用发出请求后让挂起的中断运行 / Let the interrupt pended by an application call run
void PendedHs()
{
  if (fake_pfic.pending[USBHS_IRQn])
  {
    RunHs();
  }
}

void LatchHs(uint8_t flags, uint8_t int_st, uint16_t rx_len = 0U)
{
  fake_usbhsd.INT_FG.value = static_cast<uint8_t>(fake_usbhsd.INT_FG.value | flags);
  fake_usbhsd.INT_ST = int_st;
  fake_usbhsd.RX_LEN = rx_len;
}

/// USBHS 设备和它的内存 / The USBHS device and its memory
struct HsBench
{
  Block<64> ep0;
  Block<2048> ep1_out;
  Block<4096> ep1_in;
  PairClass cls;
  ResettableUsb<CH32USBOtgHS> usb;

  explicit HsBench(Type in_type = Type::BULK, uint16_t bound = 512U)
      : cls(EPNumber::EP1, EPNumber::EP1, in_type, bound),
        usb({ep0.Raw(), ep1_out.Raw()}, {ep0.Raw(), ep1_in.Raw()}, 0x1209U, 0x0001U,
            0x0100U, {&LANGUAGE}, {{&cls}})
  {
    usb.Init(false);
    usb.Start(false);
    PendedHs();
    fake_usbhsd.MIS_ST = USBHS_UMS_SIE_FREE;
  }

  void Setup(const SetupPacket& setup, uint8_t extra_flags = 0U, uint8_t int_st = 0U,
             uint16_t rx_len = 0U)
  {
    std::memcpy(ep0.bytes, &setup, sizeof(setup));
    LatchHs(static_cast<uint8_t>(USBHS_UIF_SETUP_ACT | extra_flags), int_st, rx_len);
    RunHs();
  }

  /// 主机收下 EP0 上挂着的 IN 包 / The host takes the IN packet armed on EP0
  void Ep0InAck()
  {
    TEST_ASSERT((fake_usbhsd.UEP0_TX_CTRL & USBHS_UEP_T_RES_MASK) == USBHS_UEP_T_RES_ACK);
    LatchHs(USBHS_UIF_TRANSFER, USBHS_UIS_TOKEN_IN);
    RunHs();
  }

  /// 主机在 EP0 上发 OUT 数据包 / The host sends an OUT data packet on EP0
  void Ep0Out(const Bytes& data)
  {
    TEST_ASSERT((fake_usbhsd.UEP0_RX_CTRL & USBHS_UEP_R_RES_MASK) == USBHS_UEP_R_RES_ACK);
    if (!data.empty())
    {
      std::memcpy(ep0.bytes, data.data(), data.size());
    }
    LatchHs(USBHS_UIF_TRANSFER, USBHS_UIS_TOKEN_OUT | USBHS_UIS_TOG_OK,
            static_cast<uint16_t>(data.size()));
    RunHs();
  }

  void Reset(uint8_t speed_type)
  {
    fake_usbhsd.SPEED_TYPE = speed_type;
    LatchHs(USBHS_UIF_BUS_RST, 0U);
    RunHs();
  }

  /// 复位、SET_ADDRESS 5、SET_CONFIGURATION 1 / Reset, SET_ADDRESS 5,
  /// SET_CONFIGURATION 1
  void Enumerate()
  {
    Reset(USBHS_USB_SPEED_HIGH);
    Setup({0x00U, 5U, 5U, 0U, 0U});
    Ep0InAck();
    TEST_ASSERT(fake_usbhsd.DEV_AD == 5U);
    Setup({0x00U, 9U, 1U, 0U, 0U});
    Ep0InAck();
    TEST_ASSERT(cls.in != nullptr && cls.out != nullptr);
  }

  /// 主机 ACK EP1 IN 的包：自动 toggle 翻转 / The host ACKs the EP1 IN packet: the auto
  /// toggle flips
  static void HostAcksEp1() { fake_usbhsd.UEP1_TX_CTRL ^= USBHS_UEP_T_TOG_DATA1; }
};

void ResetFakes()
{
  fake_int_fg_reads_until_set = 0;
  fake_usbhsd = {};
  fake_usbfsd = {};
  fake_pfic = {};
}

/// 枚举：地址在状态 IN 之后写，端点按高速配置 / Enumeration: the address is written
/// after the status IN and the endpoints get high-speed sizes
void TestHsEnumerate()
{
  ResetFakes();
  HsBench b;
  b.Enumerate();
  TEST_ASSERT(b.cls.in->MaxPacketSize() == 512U && b.cls.out->MaxPacketSize() == 512U);
  TEST_ASSERT(fake_usbhsd.UEP1_MAX_LEN == 512U);
  TEST_ASSERT((fake_usbhsd.ENDP_CONFIG & (USBHS_UEP0_T_EN << 1U)) != 0U);
  TEST_ASSERT((fake_usbhsd.ENDP_CONFIG & (USBHS_UEP0_R_EN << 1U)) != 0U);
  TEST_ASSERT((fake_usbhsd.UEP1_TX_CTRL & USBHS_UEP_T_TOG_AUTO) != 0U);
  TEST_ASSERT(fake_pfic.enabled[USBHS_IRQn]);
  // 屏蔽只发生在控制器中断入口 / Masking happens only at the interrupt entry
  TEST_ASSERT(fake_pfic.disables > 0U);
}

/// 复位时 SPEED_TYPE 还是全速，第一个 SETUP 时已是高速：再报一次复位
/// SPEED_TYPE still reads full speed at the reset and high speed at the first SETUP: a
/// second reset is reported
void TestHsSecondResetOnSpeed()
{
  ResetFakes();
  HsBench b;
  b.Reset(USBHS_USB_SPEED_FULL);
  TEST_ASSERT(b.usb.GetSpeed() == Speed::FULL);
  fake_usbhsd.SPEED_TYPE = USBHS_USB_SPEED_HIGH;
  b.Setup({0x80U, 6U, 0x0100U, 0U, 18U});
  TEST_ASSERT(b.usb.GetSpeed() == Speed::HIGH);
  // 设备描述符的第一包已挂在 EP0 IN 上 / The first packet of the device descriptor is
  // armed on EP0 IN
  TEST_ASSERT(fake_usbhsd.UEP0_TX_LEN == 18U);
}

/// 停 OUT 时收下锁存的包，actual 是真实收到的字节
/// Stopping OUT collects the latched packet, actual is the real byte count
void TestHsOutStopCollects()
{
  ResetFakes();
  HsBench b;
  b.Enumerate();
  TEST_ASSERT(b.cls.out->Receive(1024U, false) == ErrorCode::OK);
  PendedHs();
  TEST_ASSERT((fake_usbhsd.UEP1_RX_CTRL & USBHS_UEP_R_RES_MASK) == USBHS_UEP_R_RES_ACK);
  const uint32_t slot = fake_usbhsd.UEP1_RX_DMA;
  const uint8_t tog0 = fake_usbhsd.UEP1_RX_CTRL & USBHS_UEP_R_TOG_MASK;

  // 第一包 512 字节：DMA 推进，toggle 翻转，请求未完 / First packet of 512: the DMA
  // advances, the toggle flips, the request goes on
  LatchHs(USBHS_UIF_TRANSFER, USBHS_UIS_TOKEN_OUT | USBHS_UIS_TOG_OK | 1U, 512U);
  RunHs();
  TEST_ASSERT(fake_usbhsd.UEP1_RX_DMA == slot + 512U && b.cls.out_done.empty());

  // 第二包 100 字节在停止时已锁存、尚未服务 / A second packet of 100 bytes is latched at
  // the stop and not serviced yet
  LatchHs(USBHS_UIF_TRANSFER, USBHS_UIS_TOKEN_OUT | USBHS_UIS_TOG_OK | 1U, 100U);
  b.cls.out->Flush(false);
  TEST_ASSERT(b.cls.out_done.size() == 1U);
  TEST_ASSERT(b.cls.out_done[0].result == ErrorCode::ABORTED);
  TEST_ASSERT(b.cls.out_done[0].actual == 612U);
  TEST_ASSERT((fake_usbhsd.INT_FG & USBHS_UIF_TRANSFER) == 0U);
  TEST_ASSERT((fake_usbhsd.UEP1_RX_CTRL & USBHS_UEP_R_RES_MASK) == USBHS_UEP_R_RES_NAK);
  // 两个包各翻一次 / One flip per packet
  TEST_ASSERT((fake_usbhsd.UEP1_RX_CTRL & USBHS_UEP_R_TOG_MASK) == tog0);
  TEST_ASSERT(!fake_pfic.pending[USBHS_IRQn]);

  // 只收到一包后停止：actual 为 512
  // Stop after one packet only: actual is 512
  TEST_ASSERT(b.cls.out->Receive(1024U, false) == ErrorCode::OK);
  PendedHs();
  LatchHs(USBHS_UIF_TRANSFER, USBHS_UIS_TOKEN_OUT | USBHS_UIS_TOG_OK | 1U, 512U);
  RunHs();
  b.cls.out->Flush(false);
  TEST_ASSERT(b.cls.out_done.size() == 2U && b.cls.out_done[1].actual == 512U);
}

/// 别的端点的锁存事件留给事件循环，并挂起中断线
/// Another endpoint's latched event stays for the event loop and the line is pended
void TestHsStopLeavesOtherEvent()
{
  ResetFakes();
  HsBench b;
  b.Enumerate();
  TEST_ASSERT(b.cls.out->Receive(512U, false) == ErrorCode::OK);
  PendedHs();
  LatchHs(USBHS_UIF_TRANSFER, USBHS_UIS_TOKEN_IN | 2U);
  b.cls.out->Flush(false);
  TEST_ASSERT(b.cls.out_done.size() == 1U && b.cls.out_done[0].actual == 0U);
  TEST_ASSERT((fake_usbhsd.INT_FG & USBHS_UIF_TRANSFER) != 0U);
  TEST_ASSERT(fake_pfic.pending[USBHS_IRQn]);
  RunHs();
  TEST_ASSERT((fake_usbhsd.INT_FG & USBHS_UIF_TRANSFER) == 0U);
}

/// 事件循环已分发、尚未清除的 UIF_TRANSFER 不归同一轮里嵌套的停止：回调里新挂的接收被
/// 停时 actual 为 0，不是那个旧包的长度
/// A UIF_TRANSFER the event loop has dispatched but not yet cleared does not belong to a
/// stop nested in the same round: a receive armed from a callback and then stopped has
/// actual 0, not the length of that old packet
void TestHsNestedStopIgnoresHeldEvent()
{
  ResetFakes();
  HsBench b;
  b.Enumerate();
  TEST_ASSERT(b.cls.out->Receive(512U, false) == ErrorCode::OK);
  b.cls.Send(100U);
  PendedHs();
  b.cls.on_out = [&b]() { TEST_ASSERT(b.cls.out->Receive(512U, true) == ErrorCode::OK); };
  b.cls.on_in = [&b]() { b.cls.out->Flush(true); };
  // 同一轮：OUT 的包有事件，IN 的 ACK 只在 toggle 上 / One round: the OUT packet has an
  // event, the IN ACK shows only in the toggle
  HsBench::HostAcksEp1();
  LatchHs(USBHS_UIF_TRANSFER, USBHS_UIS_TOKEN_OUT | USBHS_UIS_TOG_OK | 1U, 512U);
  RunHs();
  TEST_ASSERT(b.cls.in_done.size() == 1U && b.cls.in_done[0].result == ErrorCode::OK);
  TEST_ASSERT(b.cls.out_done.size() == 2U);
  TEST_ASSERT(b.cls.out_done[0].result == ErrorCode::OK &&
              b.cls.out_done[0].actual == 512U);
  TEST_ASSERT(b.cls.out_done[1].result == ErrorCode::ABORTED &&
              b.cls.out_done[1].actual == 0U);
}

/// IN 包已被主机取走（锁存事件或 toggle）时停止报 OK 或已确认的字节
/// An IN packet the host took (latched event or toggle) makes the stop report OK or
/// the confirmed bytes
void TestHsInStop()
{
  ResetFakes();
  HsBench b;
  b.Enumerate();

  // 锁存的 ACK 事件：OK 100 / Latched ACK event: OK 100
  b.cls.Send(100U);
  PendedHs();
  TEST_ASSERT(fake_usbhsd.UEP1_TX_LEN == 100U);
  HsBench::HostAcksEp1();
  LatchHs(USBHS_UIF_TRANSFER, USBHS_UIS_TOKEN_IN | 1U);
  b.cls.in->Flush(false);
  TEST_ASSERT(b.cls.in_done.size() == 1U && b.cls.in_done[0].result == ErrorCode::OK &&
              b.cls.in_done[0].actual == 100U);
  TEST_ASSERT((fake_usbhsd.INT_FG & USBHS_UIF_TRANSFER) == 0U);

  // 516 = 512 + 4：第一包的 ACK 只表现为 toggle（INT_ST 是忙 NAK）：ABORTED 512
  // 516 = 512 + 4: the ACK of the first packet shows only as the toggle (INT_ST is a
  // busy NAK): ABORTED 512
  b.cls.Send(516U);
  PendedHs();
  TEST_ASSERT(fake_usbhsd.UEP1_TX_LEN == 512U);
  HsBench::HostAcksEp1();
  b.cls.in->Flush(false);
  TEST_ASSERT(b.cls.in_done.size() == 2U &&
              b.cls.in_done[1].result == ErrorCode::ABORTED &&
              b.cls.in_done[1].actual == 512U);

  // 没发出去：ABORTED 0 / Not sent: ABORTED 0
  b.cls.Send(100U);
  PendedHs();
  b.cls.in->Flush(false);
  TEST_ASSERT(b.cls.in_done.size() == 3U &&
              b.cls.in_done[2].result == ErrorCode::ABORTED &&
              b.cls.in_done[2].actual == 0U);
  TEST_ASSERT((fake_usbhsd.UEP1_TX_CTRL & USBHS_UEP_T_RES_MASK) == USBHS_UEP_T_RES_NAK);

  // 写 NAK 的同时包被 ACK、toggle 的自动推进丢了：写之前 UIF_TRANSFER 未置位，写之后
  // 已置位且 INT_ST 指向本端点的 IN。算已发，toggle 补推一次
  // The packet is ACKed just as NAK is written and the automatic toggle advance is lost:
  // UIF_TRANSFER is clear before the write and set after it, with INT_ST pointing at
  // this endpoint's IN. Counted as sent, and the toggle advance is redone
  b.cls.Send(100U);
  PendedHs();
  const uint8_t armed = fake_usbhsd.UEP1_TX_CTRL & USBHS_UEP_T_TOG_MASK;
  fake_usbhsd.INT_ST = USBHS_UIS_TOKEN_IN | 1U;
  fake_int_fg_set_bits = USBHS_UIF_TRANSFER;
  fake_int_fg_reads_until_set = 2;
  b.cls.in->Flush(false);
  fake_int_fg_reads_until_set = 0;
  TEST_ASSERT(b.cls.in_done.size() == 4U && b.cls.in_done[3].result == ErrorCode::OK &&
              b.cls.in_done[3].actual == 100U);
  TEST_ASSERT((fake_usbhsd.UEP1_TX_CTRL & USBHS_UEP_T_TOG_MASK) != armed);
  TEST_ASSERT((fake_usbhsd.INT_FG & USBHS_UIF_TRANSFER) == 0U);

  // 同样的时序，但完成的是别的端点的事务：不补推，本请求没发出去
  // The same timing, but the transaction that completed is another endpoint's: no redo,
  // and this request was not sent
  b.cls.Send(100U);
  PendedHs();
  const uint8_t armed2 = fake_usbhsd.UEP1_TX_CTRL & USBHS_UEP_T_TOG_MASK;
  fake_usbhsd.INT_ST = USBHS_UIS_TOKEN_IN | 2U;
  fake_int_fg_reads_until_set = 2;
  b.cls.in->Flush(false);
  fake_int_fg_reads_until_set = 0;
  TEST_ASSERT(b.cls.in_done.size() == 5U &&
              b.cls.in_done[4].result == ErrorCode::ABORTED &&
              b.cls.in_done[4].actual == 0U);
  TEST_ASSERT((fake_usbhsd.UEP1_TX_CTRL & USBHS_UEP_T_TOG_MASK) == armed2);
  fake_usbhsd.INT_FG.value = 0U;

  // 停止时已有锁存的事件（写之前 UIF_TRANSFER 已置位）：toggle 没动就是没发，不补推
  // An event is already latched at the stop (UIF_TRANSFER set before the write): an
  // unmoved toggle means not sent, and nothing is redone
  b.cls.Send(100U);
  PendedHs();
  const uint8_t armed3 = fake_usbhsd.UEP1_TX_CTRL & USBHS_UEP_T_TOG_MASK;
  LatchHs(USBHS_UIF_TRANSFER, USBHS_UIS_TOKEN_IN | USBHS_UIS_IS_NAK | 1U);
  b.cls.in->Flush(false);
  TEST_ASSERT(b.cls.in_done.size() == 6U &&
              b.cls.in_done[5].result == ErrorCode::ABORTED &&
              b.cls.in_done[5].actual == 0U);
  TEST_ASSERT((fake_usbhsd.UEP1_TX_CTRL & USBHS_UEP_T_TOG_MASK) == armed3);
  fake_usbhsd.INT_FG.value = 0U;
  b.cls.in_done.erase(b.cls.in_done.begin() + 3, b.cls.in_done.end());

  // 正常路径：两包都由事件确认，完成一次，回调在中断里
  // Normal path: both packets confirmed by events, one completion, the callback runs in
  // the interrupt
  b.cls.Send(516U);
  PendedHs();
  HsBench::HostAcksEp1();
  LatchHs(USBHS_UIF_TRANSFER, USBHS_UIS_TOKEN_IN | 1U);
  RunHs();
  TEST_ASSERT(fake_usbhsd.UEP1_TX_LEN == 4U && b.cls.in_done.size() == 3U);
  HsBench::HostAcksEp1();
  LatchHs(USBHS_UIF_TRANSFER, USBHS_UIS_TOKEN_IN | 1U);
  RunHs();
  TEST_ASSERT(b.cls.in_done.size() == 4U && b.cls.in_done[3].result == ErrorCode::OK &&
              b.cls.in_done[3].actual == 516U && b.cls.in_done[3].in_isr);
}

/// 挂 OUT 接收时同号 IN 恰好完成、toggle 推进丢失：下一个好包时补推（板上实测的
/// 缺陷），停止和 ClearStall 清掉窗口
/// An IN of the same number completes while an OUT receive is armed and its toggle
/// advance is lost: the next good packet repairs it (a defect measured on the board); a
/// stop and ClearStall clear the window
void TestHsInToggleRepair()
{
  ResetFakes();
  HsBench b;
  b.Enumerate();
  b.cls.Send(100U);
  PendedHs();
  const uint8_t tog0 = fake_usbhsd.UEP1_TX_CTRL & USBHS_UEP_T_TOG_MASK;
  // 挂 OUT：第一次读 INT_FG 空闲，第二次读时 UIF_TRANSFER 已置位（IN 的 ACK，推进丢了）
  // Arm OUT: the first INT_FG read is idle, at the second UIF_TRANSFER is set (the IN
  // ACK, whose advance was lost)
  fake_int_fg_set_bits = USBHS_UIF_TRANSFER;
  fake_int_fg_reads_until_set = 2;
  TEST_ASSERT(b.cls.out->Receive(512U, false) == ErrorCode::OK);
  PendedHs();
  fake_int_fg_reads_until_set = 0;
  fake_usbhsd.INT_ST = USBHS_UIS_TOKEN_IN | 1U;
  RunHs();
  TEST_ASSERT(b.cls.in_done.size() == 1U && b.cls.in_done[0].result == ErrorCode::OK);
  TEST_ASSERT((fake_usbhsd.UEP1_TX_CTRL & USBHS_UEP_T_TOG_MASK) != tog0);

  // 没有窗口（同号 OUT 是在 UIF_TRANSFER 已置位的一轮里重新挂上的）：完成事件不带
  // IS_NAK 而 toggle 没动，同样补推
  // No window (the OUT of the same number was armed again in a round whose UIF_TRANSFER
  // was already set): a completion event without IS_NAK and an unmoved toggle is
  // repaired all the same
  b.cls.Send(100U);
  PendedHs();
  const uint8_t tog_armed = fake_usbhsd.UEP1_TX_CTRL & USBHS_UEP_T_TOG_MASK;
  LatchHs(USBHS_UIF_TRANSFER, USBHS_UIS_TOKEN_IN | 1U);
  RunHs();
  TEST_ASSERT(b.cls.in_done.size() == 2U && b.cls.in_done[1].result == ErrorCode::OK);
  TEST_ASSERT((fake_usbhsd.UEP1_TX_CTRL & USBHS_UEP_T_TOG_MASK) != tog_armed);
  b.cls.in_done.pop_back();

  // 窗口记下之后停止：窗口作废，下一个包被 ACK 时不再翻 / A stop after the window was
  // recorded voids it; the next ACKed packet is not flipped again
  b.cls.Send(100U);
  PendedHs();
  (void)b.cls.out->Receive(512U, false);
  b.cls.out->Flush(false);
  fake_int_fg_reads_until_set = 2;
  TEST_ASSERT(b.cls.out->Receive(512U, false) == ErrorCode::OK);
  PendedHs();
  fake_int_fg_reads_until_set = 0;
  fake_usbhsd.INT_FG.value = 0U;
  b.cls.in->Flush(false);
  b.cls.Send(100U);
  PendedHs();
  const uint8_t tog1 = fake_usbhsd.UEP1_TX_CTRL & USBHS_UEP_T_TOG_MASK;
  HsBench::HostAcksEp1();
  LatchHs(USBHS_UIF_TRANSFER, USBHS_UIS_TOKEN_IN | 1U);
  RunHs();
  TEST_ASSERT((fake_usbhsd.UEP1_TX_CTRL & USBHS_UEP_T_TOG_MASK) != tog1);

  // 窗口记下时 toggle 为 DATA1，ClearStall 把 toggle 复位为 DATA0；窗口若留着，下一个
  // 包被 ACK 后 toggle 回到 DATA1 就会被误翻。
  // The window is recorded with the toggle at DATA1 and ClearStall resets the
  // toggle to DATA0; a window left behind would wrongly flip the toggle back once the
  // next packet's ACK brings it to DATA1.
  if ((fake_usbhsd.UEP1_TX_CTRL & USBHS_UEP_T_TOG_MASK) != USBHS_UEP_T_TOG_DATA1)
  {
    b.cls.Send(100U);
    PendedHs();
    HsBench::HostAcksEp1();
    LatchHs(USBHS_UIF_TRANSFER, USBHS_UIS_TOKEN_IN | 1U);
    RunHs();
  }
  TEST_ASSERT((fake_usbhsd.UEP1_TX_CTRL & USBHS_UEP_T_TOG_MASK) == USBHS_UEP_T_TOG_DATA1);
  // 没有 IN 在途：ClearStall 不经过停止，只有它自己能清窗口。
  // No IN in flight: ClearStall involves no stop, so only it can clear the window.
  b.cls.out->Flush(false);
  fake_int_fg_reads_until_set = 2;
  TEST_ASSERT(b.cls.out->Receive(512U, false) == ErrorCode::OK);
  PendedHs();
  fake_int_fg_reads_until_set = 0;
  fake_usbhsd.INT_FG.value = 0U;
  TEST_ASSERT(b.cls.in->Stall(false) == ErrorCode::OK);
  TEST_ASSERT(b.cls.in->ClearStall(false) == ErrorCode::OK);
  PendedHs();
  TEST_ASSERT((fake_usbhsd.UEP1_TX_CTRL & USBHS_UEP_T_TOG_MASK) == USBHS_UEP_T_TOG_DATA0);
  b.cls.Send(100U);
  PendedHs();
  HsBench::HostAcksEp1();
  LatchHs(USBHS_UIF_TRANSFER, USBHS_UIS_TOKEN_IN | 1U);
  RunHs();
  TEST_ASSERT((fake_usbhsd.UEP1_TX_CTRL & USBHS_UEP_T_TOG_MASK) == USBHS_UEP_T_TOG_DATA1);
}

/// 主机对没有 STALL 的端点发 CLEAR_FEATURE(ENDPOINT_HALT) / The host sends
/// CLEAR_FEATURE(ENDPOINT_HALT) to an endpoint that is not halted
SetupPacket ClearHalt(uint8_t address) { return {0x02U, 1U, 0U, address, 0U}; }

/// 端点没有 STALL、硬件上有请求时清 HALT：翻转位回到 DATA0，请求原样留着、继续等主机，
/// 下一包收进来并完成请求；同一轮里先到的本端点事件按旧相位处理，复位在本轮末尾
/// A clear of the halt of an endpoint that is not halted, with requests on the hardware:
/// the toggle returns to DATA0, the requests stay and wait for the host, the next packet
/// is taken and completes the request; an event of this endpoint latched earlier in the
/// same round is handled at the old phase and the reset comes at the end of the round
void TestHsClearHaltKeepsRequests()
{
  ResetFakes();
  HsBench b;
  b.Enumerate();

  // OUT，多包请求的中间：第一包后 toggle 为 DATA1 / OUT, in the middle of a multi-packet
  // request: the toggle is DATA1 after the first packet
  TEST_ASSERT(b.cls.out->Receive(1024U, false) == ErrorCode::OK);
  PendedHs();
  LatchHs(USBHS_UIF_TRANSFER, USBHS_UIS_TOKEN_OUT | USBHS_UIS_TOG_OK | 1U, 512U);
  RunHs();
  TEST_ASSERT((fake_usbhsd.UEP1_RX_CTRL & USBHS_UEP_R_TOG_MASK) == USBHS_UEP_R_TOG_DATA1);
  b.Setup(ClearHalt(0x01U));
  b.Ep0InAck();
  TEST_ASSERT((fake_usbhsd.UEP1_RX_CTRL & USBHS_UEP_R_RES_MASK) == USBHS_UEP_R_RES_ACK);
  TEST_ASSERT((fake_usbhsd.UEP1_RX_CTRL & USBHS_UEP_R_TOG_MASK) == 0U);
  TEST_ASSERT(b.cls.out_done.empty());
  LatchHs(USBHS_UIF_TRANSFER, USBHS_UIS_TOKEN_OUT | USBHS_UIS_TOG_OK | 1U, 100U);
  RunHs();
  TEST_ASSERT(b.cls.out_done.size() == 1U && b.cls.out_done[0].result == ErrorCode::OK);
  TEST_ASSERT(b.cls.out_done[0].actual == 612U);

  // OUT：一包在同一轮的 SETUP 之前已锁存 / OUT: a packet latched before the SETUP of the
  // same round
  TEST_ASSERT(b.cls.out->Receive(1024U, false) == ErrorCode::OK);
  PendedHs();
  b.Setup(ClearHalt(0x01U), USBHS_UIF_TRANSFER,
          USBHS_UIS_TOKEN_OUT | USBHS_UIS_TOG_OK | 1U, 512U);
  b.Ep0InAck();
  TEST_ASSERT((fake_usbhsd.UEP1_RX_CTRL & USBHS_UEP_R_RES_MASK) == USBHS_UEP_R_RES_ACK);
  TEST_ASSERT((fake_usbhsd.UEP1_RX_CTRL & USBHS_UEP_R_TOG_MASK) == 0U);
  LatchHs(USBHS_UIF_TRANSFER, USBHS_UIS_TOKEN_OUT | USBHS_UIS_TOG_OK | 1U, 10U);
  RunHs();
  TEST_ASSERT(b.cls.out_done.size() == 2U && b.cls.out_done[1].actual == 522U);

  // IN：一包挂着，toggle 为 DATA1 / IN: a packet armed with the toggle at DATA1
  b.cls.Send(100U);
  PendedHs();
  HsBench::HostAcksEp1();
  LatchHs(USBHS_UIF_TRANSFER, USBHS_UIS_TOKEN_IN | 1U);
  RunHs();
  TEST_ASSERT(b.cls.in_done.size() == 1U);
  TEST_ASSERT((fake_usbhsd.UEP1_TX_CTRL & USBHS_UEP_T_TOG_MASK) == USBHS_UEP_T_TOG_DATA1);
  b.cls.Send(1536U);  // 三包，第三包挂在 DATA1 / three packets, the third armed at DATA1
  PendedHs();
  for (int k = 0; k < 2; ++k)
  {
    HsBench::HostAcksEp1();
    LatchHs(USBHS_UIF_TRANSFER, USBHS_UIS_TOKEN_IN | 1U);
    RunHs();
  }
  TEST_ASSERT((fake_usbhsd.UEP1_TX_CTRL & USBHS_UEP_T_TOG_MASK) == USBHS_UEP_T_TOG_DATA1);
  TEST_ASSERT(b.cls.in_done.size() == 1U);
  b.Setup(ClearHalt(0x81U));
  b.Ep0InAck();
  TEST_ASSERT((fake_usbhsd.UEP1_TX_CTRL & USBHS_UEP_T_RES_MASK) == USBHS_UEP_T_RES_ACK);
  TEST_ASSERT((fake_usbhsd.UEP1_TX_CTRL & USBHS_UEP_T_TOG_MASK) == USBHS_UEP_T_TOG_DATA0);
  TEST_ASSERT(b.cls.in_done.size() == 1U);
  HsBench::HostAcksEp1();
  LatchHs(USBHS_UIF_TRANSFER, USBHS_UIS_TOKEN_IN | 1U);
  RunHs();
  TEST_ASSERT(b.cls.in_done.size() == 2U && b.cls.in_done[1].result == ErrorCode::OK);
  TEST_ASSERT(b.cls.in_done[1].actual == 1536U);
  TEST_ASSERT((fake_usbhsd.UEP1_TX_CTRL & USBHS_UEP_T_TOG_MASK) == USBHS_UEP_T_TOG_DATA1);

  // IN：主机在同一轮的 SETUP 之前已取走挂着的包 / IN: the host took the armed packet
  // before the SETUP of the same round
  b.cls.Send(100U);
  PendedHs();
  HsBench::HostAcksEp1();
  b.Setup(ClearHalt(0x81U), USBHS_UIF_TRANSFER, USBHS_UIS_TOKEN_IN | 1U);
  b.Ep0InAck();
  TEST_ASSERT(b.cls.in_done.size() == 3U && b.cls.in_done[2].result == ErrorCode::OK);
  TEST_ASSERT((fake_usbhsd.UEP1_TX_CTRL & USBHS_UEP_T_TOG_MASK) == USBHS_UEP_T_TOG_DATA0);
  b.cls.Send(100U);
  PendedHs();
  TEST_ASSERT((fake_usbhsd.UEP1_TX_CTRL & USBHS_UEP_T_TOG_MASK) == USBHS_UEP_T_TOG_DATA0);
  HsBench::HostAcksEp1();
  LatchHs(USBHS_UIF_TRANSFER, USBHS_UIS_TOKEN_IN | 1U);
  RunHs();
  TEST_ASSERT(b.cls.in_done.size() == 4U && b.cls.in_done[3].result == ErrorCode::OK);

  // 不在事件轮里（owner 直接调用），IN 包已被 ACK、完成还锁存着：复位等那个完成交付之后
  // Outside an event round (a direct owner call), with an IN packet ACKed and its
  // completion still latched: the reset waits until that completion is delivered
  b.cls.Send(100U);
  PendedHs();
  if ((fake_usbhsd.UEP1_TX_CTRL & USBHS_UEP_T_TOG_MASK) != USBHS_UEP_T_TOG_DATA1)
  {
    HsBench::HostAcksEp1();
    LatchHs(USBHS_UIF_TRANSFER, USBHS_UIS_TOKEN_IN | 1U);
    RunHs();
    b.cls.Send(100U);
    PendedHs();
  }
  const size_t done_before = b.cls.in_done.size();
  HsBench::HostAcksEp1();  // DATA1 发出，toggle 到 DATA0 / sent as DATA1, toggle at DATA0
  LatchHs(USBHS_UIF_TRANSFER, USBHS_UIS_TOKEN_IN | 1U);
  TEST_ASSERT(b.cls.in->ClearStall(false) == ErrorCode::OK);
  RunHs();
  TEST_ASSERT(b.cls.in_done.size() == done_before + 1U);
  TEST_ASSERT(b.cls.in_done.back().result == ErrorCode::OK);
  TEST_ASSERT((fake_usbhsd.UEP1_TX_CTRL & USBHS_UEP_T_TOG_MASK) == USBHS_UEP_T_TOG_DATA0);

  // STALL 时清：和原来一样，NAK、DATA0 / A clear while halted: as before, NAK and DATA0
  TEST_ASSERT(b.cls.out->Stall(false) == ErrorCode::OK);
  PendedHs();
  b.Setup(ClearHalt(0x01U));
  b.Ep0InAck();
  TEST_ASSERT((fake_usbhsd.UEP1_RX_CTRL & USBHS_UEP_R_RES_MASK) == USBHS_UEP_R_RES_NAK);
  TEST_ASSERT((fake_usbhsd.UEP1_RX_CTRL & USBHS_UEP_R_TOG_MASK) == 0U);
  TEST_ASSERT(b.cls.in->Stall(false) == ErrorCode::OK);
  PendedHs();
  b.Setup(ClearHalt(0x81U));
  b.Ep0InAck();
  TEST_ASSERT((fake_usbhsd.UEP1_TX_CTRL & USBHS_UEP_T_RES_MASK) == USBHS_UEP_T_RES_NAK);
  TEST_ASSERT((fake_usbhsd.UEP1_TX_CTRL & USBHS_UEP_T_TOG_MASK) == USBHS_UEP_T_TOG_DATA0);
}

/// 同一轮读到 EP0 状态 OUT 完成和新 SETUP：上一个传输先正常完成
/// The EP0 status OUT completion and a new SETUP in one round: the previous
/// transfer completes normally first
void TestHsEp0BeforeSetup()
{
  ResetFakes();
  HsBench b;
  b.Enumerate();
  // 类 IN 请求，4 字节 / Class IN request, 4 bytes
  b.Setup({0xa1U, REQ_IN, 0U, 0U, 4U});
  b.Ep0InAck();
  TEST_ASSERT(b.cls.completes == 0U && b.cls.aborts == 0U);
  // 状态 OUT（零长度）和下一个 SETUP 一起锁存 / The status OUT (zero length) is
  // latched together with the next SETUP
  b.Setup({0x80U, 0U, 0U, 0U, 2U}, USBHS_UIF_TRANSFER, USBHS_UIS_TOKEN_OUT, 0U);
  TEST_ASSERT(b.cls.completes == 1U && b.cls.aborts == 0U);
  TEST_ASSERT(fake_usbhsd.UEP0_TX_LEN == 2U);

  // 类 OUT 请求的数据包和新 SETUP 同一轮：数据已被 SETUP 覆盖，报 FAILED，传输中止，
  // SETUP 之后 EP0 OUT 是 DATA1
  // A class OUT data packet in the same round as a new SETUP: the bytes were overwritten
  // by the SETUP, it is reported as FAILED, the transfer aborts, and EP0 OUT is DATA1
  // after the SETUP
  b.Ep0Out({});  // 上一个 GET_STATUS 的状态 OUT / Status OUT of the GET_STATUS before
  b.Setup({0x21U, REQ_OUT, 0U, 0U, 8U});
  TEST_ASSERT((fake_usbhsd.UEP0_RX_CTRL & USBHS_UEP_R_RES_MASK) == USBHS_UEP_R_RES_ACK);
  const size_t aborts = b.cls.aborts;
  b.Setup({0x80U, 0U, 0U, 0U, 2U}, USBHS_UIF_TRANSFER,
          USBHS_UIS_TOKEN_OUT | USBHS_UIS_TOG_OK, 8U);
  TEST_ASSERT(b.cls.data_seen.empty());
  TEST_ASSERT(b.cls.aborts == aborts + 1U);
  TEST_ASSERT((fake_usbhsd.UEP0_RX_CTRL & USBHS_UEP_R_TOG_MASK) == USBHS_UEP_R_TOG_DATA1);

  // 正常的类 OUT 请求：数据交给类 / A normal class OUT request: the data reaches the
  // class
  b.Ep0InAck();
  b.Ep0Out({});
  b.Setup({0x21U, REQ_OUT, 0U, 0U, 8U});
  b.Ep0Out({1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U});
  TEST_ASSERT(b.cls.data_seen.size() == 8U && b.cls.data_seen[7] == 8U);
  b.Ep0InAck();
}

//// SIE 忙时停止不等：请求当场 ABORTED，停止由本端点的完成、之后读到的 SIE 空闲或总线
/// 复位结束，排队的请求到那时才挂上硬件
/// With a busy SIE a stop does not wait: the request is ABORTED at once, the stop ends on
/// the endpoint's own completion, on the SIE read free later or on a bus reset, and a
/// queued request reaches the hardware only then
void TestHsPendingStop()
{
  ResetFakes();
  HsBench b;
  b.Enumerate();

  // 本端点的完成结束停止：软件 toggle 翻一次，包不计入已回调的请求
  // Its own completion ends the stop: the software toggle flips once and the packet is
  // not added to the request already called back
  TEST_ASSERT(b.cls.out->Receive(1024U, false) == ErrorCode::OK);
  PendedHs();
  const uint8_t tog0 = fake_usbhsd.UEP1_RX_CTRL & USBHS_UEP_R_TOG_MASK;
  fake_usbhsd.MIS_ST = 0U;
  b.cls.out->Flush(false);
  TEST_ASSERT(b.cls.out_done.size() == 1U &&
              b.cls.out_done[0].result == ErrorCode::ABORTED &&
              b.cls.out_done[0].actual == 0U);
  TEST_ASSERT((fake_usbhsd.UEP1_RX_CTRL & USBHS_UEP_R_RES_MASK) == USBHS_UEP_R_RES_NAK);
  TEST_ASSERT(b.cls.out->Receive(1024U, false) == ErrorCode::OK);
  TEST_ASSERT(b.cls.out->Receive(1024U, false) == ErrorCode::BUSY);
  PendedHs();
  TEST_ASSERT((fake_usbhsd.UEP1_RX_CTRL & USBHS_UEP_R_RES_MASK) == USBHS_UEP_R_RES_NAK);
  // SIE 仍忙（别的端点的事务）：结束停止的是这个完成事件本身
  // The SIE is still busy (another endpoint's transaction): the completion event
  // itself ends the stop
  LatchHs(USBHS_UIF_TRANSFER, USBHS_UIS_TOKEN_OUT | USBHS_UIS_TOG_OK | 1U, 512U);
  RunHs();
  TEST_ASSERT(b.cls.out_done.size() == 1U);
  TEST_ASSERT((fake_usbhsd.UEP1_RX_CTRL & USBHS_UEP_R_TOG_MASK) != tog0);
  TEST_ASSERT((fake_usbhsd.UEP1_RX_CTRL & USBHS_UEP_R_RES_MASK) == USBHS_UEP_R_RES_ACK);
  fake_usbhsd.MIS_ST = USBHS_UMS_SIE_FREE;
  b.cls.out->Flush(false);
  TEST_ASSERT(b.cls.out_done.size() == 2U && b.cls.out_done[1].actual == 0U);

  // 没有完成事件：SIE 仍忙的一轮不结束停止，读到空闲的一轮结束；IN 包其实已被取走，
  // 自动 toggle 保持同步
  // No completion event: a round with the SIE still busy does not end the stop, a round
  // reading it free does; the IN packet was in fact taken and the auto toggle stays in
  // step
  b.cls.Send(100U);
  PendedHs();
  fake_usbhsd.MIS_ST = 0U;
  HsBench::HostAcksEp1();
  b.cls.in->Flush(false);
  TEST_ASSERT(b.cls.in_done.size() == 1U &&
              b.cls.in_done[0].result == ErrorCode::ABORTED &&
              b.cls.in_done[0].actual == 0U);
  b.cls.Send(10U);
  RunHs();
  TEST_ASSERT(fake_usbhsd.UEP1_TX_LEN == 100U);
  TEST_ASSERT((fake_usbhsd.UEP1_TX_CTRL & USBHS_UEP_T_RES_MASK) == USBHS_UEP_T_RES_NAK);
  fake_usbhsd.MIS_ST = USBHS_UMS_SIE_FREE;
  RunHs();
  TEST_ASSERT(fake_usbhsd.UEP1_TX_LEN == 10U);
  TEST_ASSERT((fake_usbhsd.UEP1_TX_CTRL & USBHS_UEP_T_RES_MASK) == USBHS_UEP_T_RES_ACK);
  TEST_ASSERT(b.cls.in_done.size() == 1U);

  // 总线复位在报告之前结束未结束的停止，不看 SIE_FREE（复位期间它为 0），端点当场在
  // 硬件上关闭；之后照常枚举
  // A bus reset ends the pending stop before it is reported, whatever SIE_FREE reads (0
  // during a reset), and the endpoint is closed in hardware at once; enumeration works
  // afterwards
  fake_usbhsd.MIS_ST = 0U;
  b.cls.in->Flush(false);
  TEST_ASSERT(b.cls.in_done.size() == 2U &&
              b.cls.in_done[1].result == ErrorCode::ABORTED);
  TEST_ASSERT((fake_usbhsd.ENDP_CONFIG & (USBHS_UEP0_T_EN << 1U)) != 0U);
  LatchHs(USBHS_UIF_BUS_RST, 0U);
  RunHs();
  TEST_ASSERT(b.cls.in == nullptr);
  TEST_ASSERT((fake_usbhsd.ENDP_CONFIG & (USBHS_UEP0_T_EN << 1U)) == 0U);
  fake_usbhsd.MIS_ST = USBHS_UMS_SIE_FREE;
  b.Enumerate();
  TEST_ASSERT(b.cls.in != nullptr);
  b.cls.Send(10U);
  PendedHs();
  TEST_ASSERT((fake_usbhsd.UEP1_TX_CTRL & USBHS_UEP_T_RES_MASK) == USBHS_UEP_T_RES_ACK);
  b.cls.in->Flush(false);
}

/// 没有事件、SIE 也不空闲：节拍挂起 USB 中断，写 NAK 满 1 ms 的停止按时间结束
/// No event and no free SIE: the tick pends the USB interrupt and a stop whose NAK is
/// 1 ms old ends by time
void TestHsStopByTime()
{
  ResetFakes();
  HsBench b;
  b.Enumerate();
  b.cls.Send(100U);
  PendedHs();
  fake_usbhsd.MIS_ST = 0U;
  b.cls.in->Flush(false);
  TEST_ASSERT(b.cls.in_done.size() == 1U &&
              b.cls.in_done[0].result == ErrorCode::ABORTED);

  // 没有别的中断时下一个节拍就挂起 USB 中断（每轮末尾启动的等待），但停止还不满 1 ms
  // With no other interrupt coming the next tick pends the USB interrupt (the wait
  // started at the end of every pass), but the stop is not 1 ms old yet
  DriverAsyncCondition::OnTick(1U);
  TEST_ASSERT(fake_pfic.pending[USBHS_IRQn]);
  b.cls.Send(10U);
  PendedHs();
  TEST_ASSERT(fake_usbhsd.UEP1_TX_LEN == 100U);
  TEST_ASSERT((fake_usbhsd.UEP1_TX_CTRL & USBHS_UEP_T_RES_MASK) == USBHS_UEP_T_RES_NAK);

  // 等待已重新启动；满 1 ms 之后挂起的那一轮结束停止，排队的请求挂上
  // The wait was started again; the pass pended after 1 ms ends the stop and the
  // queued request is armed
  std::this_thread::sleep_for(std::chrono::microseconds(1200));
  DriverAsyncCondition::OnTick(1U);
  DriverAsyncCondition::OnTick(1U);
  TEST_ASSERT(fake_pfic.pending[USBHS_IRQn]);
  PendedHs();
  TEST_ASSERT(fake_usbhsd.UEP1_TX_LEN == 10U);
  TEST_ASSERT((fake_usbhsd.UEP1_TX_CTRL & USBHS_UEP_T_RES_MASK) == USBHS_UEP_T_RES_ACK);
  TEST_ASSERT(b.cls.in_done.size() == 1U);

  fake_usbhsd.MIS_ST = USBHS_UMS_SIE_FREE;
  b.cls.in->Flush(false);
  TEST_ASSERT(b.cls.in_done.size() == 2U);
}

/// 帧起始由 FRAME_NO 上报，不开 SOF 中断：帧号变了报一次，没变不报；没有别的中断时由
/// 节拍挂起 USB 中断；全速只报整帧；挂起期间不报、节拍停下，恢复后继续
/// Starts of frame come from FRAME_NO with the SOF interrupt off: one report per change
/// of the number and none without; the tick pends the USB interrupt when no other
/// interrupt comes; full speed reports whole frames; nothing is reported and the tick
/// stops while suspended, and both go on after the resume
void TestHsFrameReport()
{
  ResetFakes();
  HsBench b;
  b.Enumerate();
  TEST_ASSERT((fake_usbhsd.INT_EN & USBHS_UIE_SOF_ACT) == 0U);
  b.cls.frames.clear();
  // 微帧 2、帧 100；位 11、12 不算 / Microframe 2, frame 100; bits 11 and 12 do not count
  fake_usbhsd.FRAME_NO = static_cast<uint16_t>((2U << 13U) | 0x1800U | 100U);
  RunHs();
  TEST_ASSERT(b.cls.frames.size() == 1U && b.cls.frames[0] == (100U | (2U << 12U)));
  fake_usbhsd.FRAME_NO = static_cast<uint16_t>((2U << 13U) | 100U);
  RunHs();
  TEST_ASSERT(b.cls.frames.size() == 1U);
  // 没有别的中断：节拍挂起 USB 中断，那一轮看到帧号前进
  // No other interrupt: the tick pends the USB interrupt, whose pass sees the number
  // advance
  fake_usbhsd.FRAME_NO = static_cast<uint16_t>((3U << 13U) | 100U);
  DriverAsyncCondition::OnTick(1U);
  DriverAsyncCondition::OnTick(1U);
  TEST_ASSERT(fake_pfic.pending[USBHS_IRQn]);
  PendedHs();
  TEST_ASSERT(b.cls.frames.size() == 2U && b.cls.frames[1] == (100U | (3U << 12U)));
  // 中间过了几个微帧只报最新的 / Several microframes later only the latest is reported
  fake_usbhsd.FRAME_NO = static_cast<uint16_t>((1U << 13U) | 101U);
  RunHs();
  TEST_ASSERT(b.cls.frames.size() == 3U && b.cls.frames[2] == (101U | (1U << 12U)));

  // 挂起：不报，节拍不再启动 / Suspended: no report and the tick is not started again
  fake_usbhsd.MIS_ST = USBHS_UMS_SIE_FREE | USBHS_UMS_SUSPEND;
  LatchHs(USBHS_UIF_SUSPEND, 0U);
  RunHs();
  fake_usbhsd.FRAME_NO = static_cast<uint16_t>((2U << 13U) | 101U);
  DriverAsyncCondition::OnTick(1U);
  DriverAsyncCondition::OnTick(1U);
  PendedHs();
  DriverAsyncCondition::OnTick(1U);
  DriverAsyncCondition::OnTick(1U);
  TEST_ASSERT(!fake_pfic.pending[USBHS_IRQn]);
  TEST_ASSERT(b.cls.frames.size() == 3U);
  // 恢复：继续报 / Resume: reports go on
  fake_usbhsd.MIS_ST = USBHS_UMS_SIE_FREE;
  LatchHs(USBHS_UIF_SUSPEND, 0U);
  RunHs();
  TEST_ASSERT(b.cls.frames.size() == 4U && b.cls.frames[3] == (101U | (2U << 12U)));
  DriverAsyncCondition::OnTick(1U);
  DriverAsyncCondition::OnTick(1U);
  TEST_ASSERT(fake_pfic.pending[USBHS_IRQn]);
  PendedHs();

  // 恢复的那次变化没有留下标志（读状态和清标志之间变的）：下一轮按 MIS_ST 的状态补报
  // The change back left no flag (it happened between reading the state and clearing
  // the flag): the next pass reports the resume from the state MIS_ST shows
  fake_usbhsd.MIS_ST = USBHS_UMS_SIE_FREE | USBHS_UMS_SUSPEND;
  LatchHs(USBHS_UIF_SUSPEND, 0U);
  RunHs();
  fake_usbhsd.MIS_ST = USBHS_UMS_SIE_FREE;
  fake_usbhsd.FRAME_NO = static_cast<uint16_t>((3U << 13U) | 101U);
  RunHs();
  TEST_ASSERT(b.cls.frames.size() == 5U && b.cls.frames[4] == (101U | (3U << 12U)));
  DriverAsyncCondition::OnTick(1U);
  DriverAsyncCondition::OnTick(1U);
  TEST_ASSERT(fake_pfic.pending[USBHS_IRQn]);
  PendedHs();

  // 挂起中来的总线复位也结束挂起；配置之前的帧起始不交给设备类；全速只报整帧
  // A bus reset while suspended ends the suspend too; a start of frame before the
  // configuration does not reach the device class; full speed reports whole frames
  fake_usbhsd.MIS_ST = USBHS_UMS_SIE_FREE | USBHS_UMS_SUSPEND;
  LatchHs(USBHS_UIF_SUSPEND, 0U);
  RunHs();
  fake_usbhsd.MIS_ST = USBHS_UMS_SIE_FREE;
  b.cls.frames.clear();
  fake_usbhsd.FRAME_NO = 200U;
  b.Reset(USBHS_USB_SPEED_FULL);
  b.Setup({0x00U, 5U, 5U, 0U, 0U});
  b.Ep0InAck();
  b.Setup({0x00U, 9U, 1U, 0U, 0U});
  b.Ep0InAck();
  TEST_ASSERT(b.cls.frames.empty());
  fake_usbhsd.FRAME_NO = 201U;
  RunHs();
  TEST_ASSERT(b.cls.frames.size() == 1U && b.cls.frames[0] == (201U | 0xF000U));
}

/// 处理 SETUP 期间停控制端点当场完成，即使 SIE 忙
/// Stopping the control endpoint during SETUP handling is complete at once, even with a
/// busy SIE
void TestHsEp0StopAtSetup()
{
  ResetFakes();
  HsBench b;
  b.Enumerate();
  // GET_DESCRIPTOR(device)：EP0 IN 挂上应答，主机不取，直接发下一个 SETUP
  // GET_DESCRIPTOR(device): the reply is armed on EP0 IN, the host does not take it and
  // sends the next SETUP
  b.Setup({0x80U, 6U, 0x0100U, 0U, 18U});
  TEST_ASSERT((fake_usbhsd.UEP0_TX_CTRL & USBHS_UEP_T_RES_MASK) == USBHS_UEP_T_RES_ACK);
  fake_usbhsd.MIS_ST = 0U;
  b.Setup({0x80U, 6U, 0x0100U, 0U, 18U});
  TEST_ASSERT((fake_usbhsd.UEP0_TX_CTRL & USBHS_UEP_T_RES_MASK) == USBHS_UEP_T_RES_ACK);
  TEST_ASSERT(fake_usbhsd.UEP0_TX_LEN == 18U);
  fake_usbhsd.MIS_ST = USBHS_UMS_SIE_FREE;
  b.Ep0InAck();
}

/// Stop 经过 owner、不等总线：在途请求当场 ABORTED、类解绑，未结束的停止照常结束，
/// 全部结束之后才断开总线、关中断源，中断线保持使能；还没处理的 Stop 被之后的 Start
/// 撤销
/// Stop goes through the owner and never waits for the bus: requests in flight are
/// ABORTED at once and the class unbound, pending stops end as usual, and only when all
/// have ended is the bus detached and the interrupt sources switched off, with the line
/// left enabled; a Stop not served yet is cancelled by a later Start
void TestHsDeviceStop()
{
  ResetFakes();
  HsBench b;
  b.Enumerate();
  b.cls.Send(100U);
  PendedHs();
  fake_usbhsd.MIS_ST = 0U;
  b.cls.in->Flush(false);  // 停止未结束 / the stop is pending
  TEST_ASSERT(b.cls.in_done.size() == 1U);
  TEST_ASSERT(b.cls.out->Receive(512U, false) == ErrorCode::OK);
  PendedHs();
  b.usb.Stop(false);
  // 请求只发布、挂起中断：owner 运行之前什么都没变 / The request is only published and
  // the interrupt pended: nothing changes before the owner runs
  TEST_ASSERT(b.cls.in != nullptr && fake_pfic.pending[USBHS_IRQn]);
  PendedHs();
  TEST_ASSERT(b.cls.in == nullptr && b.cls.out == nullptr);
  TEST_ASSERT(b.cls.out_done.size() == 1U &&
              b.cls.out_done[0].result == ErrorCode::ABORTED);
  // SIE 还忙着：两个停止都没结束，总线还接着 / The SIE is still busy: both stops are
  // pending and the bus stays attached
  TEST_ASSERT((fake_usbhsd.CONTROL & USBHS_UC_DEV_PU_EN) != 0U);
  TEST_ASSERT(fake_usbhsd.INT_EN != 0U);
  RunHs();
  TEST_ASSERT((fake_usbhsd.CONTROL & USBHS_UC_DEV_PU_EN) != 0U);
  // 停止按时间结束之后才断开。断开前最后一轮启动的等待还会挂起一次中断，那一轮不再
  // 启动等待，之后节拍不再挂起中断
  // The detach comes once the stops have ended by time. The wait started by the last
  // pass before the detach pends the interrupt once more; that pass starts no wait, and
  // the tick pends nothing afterwards
  std::this_thread::sleep_for(std::chrono::microseconds(1200));
  DriverAsyncCondition::OnTick(1U);
  DriverAsyncCondition::OnTick(1U);
  PendedHs();
  TEST_ASSERT((fake_usbhsd.CONTROL & USBHS_UC_DEV_PU_EN) == 0U);
  TEST_ASSERT(fake_usbhsd.INT_EN == 0U && fake_pfic.enabled[USBHS_IRQn]);
  DriverAsyncCondition::OnTick(1U);
  DriverAsyncCondition::OnTick(1U);
  PendedHs();
  DriverAsyncCondition::OnTick(1U);
  DriverAsyncCondition::OnTick(1U);
  TEST_ASSERT(!fake_pfic.pending[USBHS_IRQn]);
  fake_usbhsd.MIS_ST = USBHS_UMS_SIE_FREE;
  // 断开之后已过 4 个节拍：Start 不等，第 6 个节拍才接上 / 4 ticks since the detach:
  // Start does not wait, the sixth tick attaches
  b.usb.Start(false);
  PendedHs();
  TEST_ASSERT((fake_usbhsd.CONTROL & USBHS_UC_DEV_PU_EN) == 0U);
  TickAnd(1, PendedHs);
  TEST_ASSERT((fake_usbhsd.CONTROL & USBHS_UC_DEV_PU_EN) == 0U);
  TickAnd(1, PendedHs);
  TEST_ASSERT((fake_usbhsd.CONTROL & USBHS_UC_DEV_PU_EN) != 0U);
  b.Enumerate();
  b.cls.Send(10U);
  PendedHs();
  TEST_ASSERT(fake_usbhsd.UEP1_TX_LEN == 10U);
  // 之前的停止没有留下状态：新的停止照常写 NAK、未结束、再由 SIE 空闲结束
  // The earlier stop left no state behind: a new stop writes NAK, pends and ends on a
  // free SIE as usual
  fake_usbhsd.MIS_ST = 0U;
  b.cls.in->Flush(false);
  TEST_ASSERT((fake_usbhsd.UEP1_TX_CTRL & USBHS_UEP_T_RES_MASK) == USBHS_UEP_T_RES_NAK);
  b.cls.Send(12U);
  RunHs();
  TEST_ASSERT(fake_usbhsd.UEP1_TX_LEN == 10U);
  fake_usbhsd.MIS_ST = USBHS_UMS_SIE_FREE;
  RunHs();
  TEST_ASSERT(fake_usbhsd.UEP1_TX_LEN == 12U);

  // Stop 之后、owner 运行之前又 Start：不停止 / Start after Stop and before the owner
  // runs: no stop
  b.usb.Stop(false);
  b.usb.Start(false);
  PendedHs();
  TEST_ASSERT(b.cls.in != nullptr);
  TEST_ASSERT((fake_usbhsd.CONTROL & USBHS_UC_DEV_PU_EN) != 0U);
}

/// 等时 IN 一次请求只有一包 / An isochronous IN request carries one packet
void TestHsLimits()
{
  ResetFakes();
  HsBench b(Type::ISOCHRONOUS, 1024U);
  b.Enumerate();
  TEST_ASSERT(b.cls.in->MaxPacketSize() == 1023U);
  // 槽 2048 字节放得下两包，硬件上限只许一包 / The 2048-byte slot holds two packets,
  // the hardware limit allows one
  TEST_ASSERT(b.cls.in->MaxTransferSize() == 1023U);
  TEST_ASSERT(b.cls.out->MaxTransferSize() == 1024U);  // 批量 2 包 / bulk, 2 packets
  // SOF 中断不开；万一来了 SOF 令牌也不报告 / The SOF interrupt stays off; a SOF token
  // that does come is not reported
  TEST_ASSERT((fake_usbhsd.INT_EN & USBHS_UIE_SOF_ACT) == 0U);
  LatchHs(USBHS_UIF_TRANSFER, USBHS_UIS_TOKEN_SOF);
  RunHs();
  TEST_ASSERT(b.cls.frames.empty());
}

/// 主机从 EP1 取走一包：toggle 位自动变化，UIF_TRANSFER 置位 / The host takes a packet
/// from EP1: the toggle bits change by themselves and UIF_TRANSFER is set
void HostTakesIsoEp1(uint8_t int_st)
{
  fake_usbhsd.UEP1_TX_CTRL ^= USBHS_UEP_T_TOG_DATA1;
  LatchHs(USBHS_UIF_TRANSFER, int_st);
  RunHs();
}

/// 等时 IN：发送使能常开，寄存器只在本端点刚发完一包之后改写 / Isochronous IN:
/// transmit stays enabled and the registers are rewritten only right after a packet of
/// the endpoint has gone out
void TestHsIsoIn()
{
  ResetFakes();
  HsBench b(Type::ISOCHRONOUS, 1024U);
  b.Enumerate();
  const uint32_t tx_enable = USBHS_UEP0_T_EN << 1U;
  const uint8_t own = USBHS_UIS_TOKEN_IN | 1U;
  const uint8_t other = USBHS_UIS_TOKEN_IN | USBHS_UIS_IS_NAK | 2U;
  // 配置之后：使能已开，没有包，主机来取得到零长度包
  // After configuration: enabled, no packet; a take by the host gets a zero-length one
  TEST_ASSERT((fake_usbhsd.ENDP_CONFIG & tx_enable) != 0U);
  TEST_ASSERT(fake_usbhsd.UEP1_TX_LEN == 0U);
  TEST_ASSERT((fake_usbhsd.UEP1_TX_CTRL & USBHS_UEP_T_RES_MASK) == USBHS_UEP_T_RES_ACK);

  // 空闲时提交：不动寄存器，等本端点下一次发出 / Submitted while idle: no register is
  // touched until the endpoint's next transmission
  b.cls.Send(100U);
  PendedHs();
  TEST_ASSERT(fake_usbhsd.UEP1_TX_LEN == 0U && b.cls.in_done.empty());
  // 别的端点的事件、本端点没动：仍不装入 / An event of another endpoint with this one
  // unmoved: still not loaded
  LatchHs(USBHS_UIF_TRANSFER, other);
  RunHs();
  TEST_ASSERT(fake_usbhsd.UEP1_TX_LEN == 0U);
  // 主机取走零长度包：这时装入，toggle 位回到 DATA0 / The host takes the zero-length
  // packet: the request is loaded now and the toggle bits are back at DATA0
  HostTakesIsoEp1(own);
  TEST_ASSERT(fake_usbhsd.UEP1_TX_LEN == 100U && b.cls.in_done.empty());
  TEST_ASSERT((fake_usbhsd.UEP1_TX_CTRL & USBHS_UEP_T_TOG_MASK) == USBHS_UEP_T_TOG_DATA0);

  // 第二个请求排在后面。第一包发出时 INT_ST 已指向别的端点：按 toggle 认出；第二包在
  // 同一轮里装上
  // A second request is queued behind. When the first packet goes out INT_ST points at
  // another endpoint: it is recognised by the toggle, and the second is loaded in the
  // same pass
  b.cls.Send(50U);
  PendedHs();
  TEST_ASSERT(fake_usbhsd.UEP1_TX_LEN == 100U);
  HostTakesIsoEp1(other);
  TEST_ASSERT(b.cls.in_done.size() == 1U && b.cls.in_done[0].result == ErrorCode::OK &&
              b.cls.in_done[0].actual == 100U);
  TEST_ASSERT(fake_usbhsd.UEP1_TX_LEN == 50U);
  HostTakesIsoEp1(own);
  TEST_ASSERT(b.cls.in_done.size() == 2U && b.cls.in_done[1].actual == 50U);
  // 没有包了：长度 0，使能一直没动过 / No packet left: length 0, and the enable was
  // never touched
  TEST_ASSERT(fake_usbhsd.UEP1_TX_LEN == 0U);
  TEST_ASSERT((fake_usbhsd.ENDP_CONFIG & tx_enable) != 0U);
  // 空闲时主机取到的零长度包不算任何请求的完成 / A zero-length packet the host takes
  // while idle completes no request
  HostTakesIsoEp1(own);
  TEST_ASSERT(b.cls.in_done.size() == 2U);

  // 停止还没装上硬件的请求：当场 ABORTED / Stopping a request not on the hardware yet:
  // ABORTED at once
  b.cls.Send(10U);
  PendedHs();
  b.cls.in->Flush(false);
  TEST_ASSERT(b.cls.in_done.size() == 3U &&
              b.cls.in_done[2].result == ErrorCode::ABORTED &&
              b.cls.in_done[2].actual == 0U);
  TEST_ASSERT(fake_usbhsd.UEP1_TX_LEN == 0U);

  // 停止已装上、还没发出的请求，SIE 空闲：撤回 / Stopping a loaded request that has
  // not gone out, with a free SIE: retracted
  b.cls.Send(20U);
  PendedHs();
  HostTakesIsoEp1(own);
  TEST_ASSERT(fake_usbhsd.UEP1_TX_LEN == 20U);
  b.cls.in->Flush(false);
  TEST_ASSERT(b.cls.in_done.size() == 4U &&
              b.cls.in_done[3].result == ErrorCode::ABORTED);
  TEST_ASSERT(fake_usbhsd.UEP1_TX_LEN == 0U);
  TEST_ASSERT((fake_usbhsd.ENDP_CONFIG & tx_enable) != 0U);

  // 停止时线上有事务：不等，寄存器不动；那一包发出之后停止结束，寄存器换成没有包的
  // 状态，下一个请求照常
  // A transaction is on the wire at the stop: no wait and no register write; once that
  // packet has gone out the stop ends, the registers go to the no-packet state, and the
  // next request works as usual
  b.cls.Send(30U);
  PendedHs();
  HostTakesIsoEp1(own);
  TEST_ASSERT(fake_usbhsd.UEP1_TX_LEN == 30U);
  fake_usbhsd.MIS_ST = 0U;
  b.cls.in->Flush(false);
  TEST_ASSERT(b.cls.in_done.size() == 5U &&
              b.cls.in_done[4].result == ErrorCode::ABORTED);
  TEST_ASSERT(fake_usbhsd.UEP1_TX_LEN == 30U);
  fake_usbhsd.MIS_ST = USBHS_UMS_SIE_FREE;
  HostTakesIsoEp1(own);
  TEST_ASSERT(fake_usbhsd.UEP1_TX_LEN == 0U);
  b.cls.Send(40U);
  PendedHs();
  HostTakesIsoEp1(own);
  TEST_ASSERT(fake_usbhsd.UEP1_TX_LEN == 40U);
  HostTakesIsoEp1(own);
  TEST_ASSERT(b.cls.in_done.size() == 6U && b.cls.in_done[5].result == ErrorCode::OK &&
              b.cls.in_done[5].actual == 40U);

  // 上一包完成的那一轮里主机又取走了一包（零长度包）：下一个请求不在这一轮装入，等
  // 本端点再发出一次
  // In the pass that completes a packet the host has taken another one (zero-length):
  // the next request is not loaded in this pass but at the endpoint's next transmission
  b.cls.Send(60U);
  PendedHs();
  HostTakesIsoEp1(own);
  TEST_ASSERT(fake_usbhsd.UEP1_TX_LEN == 60U);
  b.cls.Send(70U);
  PendedHs();
  b.cls.on_in = [] { fake_usbhsd.UEP1_TX_CTRL ^= USBHS_UEP_T_TOG_DATA1; };
  HostTakesIsoEp1(own);
  b.cls.on_in = nullptr;
  TEST_ASSERT(b.cls.in_done.size() == 7U && b.cls.in_done[6].actual == 60U);
  TEST_ASSERT(fake_usbhsd.UEP1_TX_LEN == 0U);
  LatchHs(USBHS_UIF_TRANSFER, own);
  RunHs();
  TEST_ASSERT(fake_usbhsd.UEP1_TX_LEN == 70U);
  HostTakesIsoEp1(own);
  TEST_ASSERT(b.cls.in_done.size() == 8U && b.cls.in_done[7].actual == 70U);

  // 关闭：这时才关发送使能 / Close: only now the transmit enable goes off
  b.Setup({0x00U, 9U, 0U, 0U, 0U});
  b.Ep0InAck();
  TEST_ASSERT((fake_usbhsd.ENDP_CONFIG & tx_enable) == 0U);
}

/// 挂起中来了总线复位，而状态位仍是挂起：复位清掉了设备核心记的挂起，后端按状态位再报
/// 一次，应答等到恢复之后才发
/// A bus reset arrives while suspended and the status bit still reads suspended: the
/// reset cleared the suspend the device core had recorded, the backend reports it once
/// more from the status bit, and the reply goes out only after the resume
void TestHsResetWhileSuspended()
{
  ResetFakes();
  HsBench b;
  b.Enumerate();
  fake_usbhsd.MIS_ST = USBHS_UMS_SIE_FREE | USBHS_UMS_SUSPEND;
  LatchHs(USBHS_UIF_SUSPEND, 0U);
  RunHs();
  b.Reset(USBHS_USB_SPEED_HIGH);
  b.Setup({0x80U, 6U, 0x0100U, 0U, 18U});
  TEST_ASSERT((fake_usbhsd.UEP0_TX_CTRL & USBHS_UEP_T_RES_MASK) == USBHS_UEP_T_RES_NAK);
  fake_usbhsd.MIS_ST = USBHS_UMS_SIE_FREE;
  LatchHs(USBHS_UIF_SUSPEND, 0U);
  RunHs();
  TEST_ASSERT((fake_usbhsd.UEP0_TX_CTRL & USBHS_UEP_T_RES_MASK) == USBHS_UEP_T_RES_ACK);
  TEST_ASSERT(fake_usbhsd.UEP0_TX_LEN == 18U);
}

/// EP0 的状态包完成时 INT_ST 带 IS_NAK（之后被忙 NAK 的令牌置的）：照常算完成
/// An EP0 status packet completes with IS_NAK in INT_ST (set by a token busy-NAKed
/// afterwards): it counts as a completion all the same
void TestHsEp0StatusWithIsNak()
{
  ResetFakes();
  HsBench b;
  // 无数据请求的状态 IN：SET_ADDRESS 的地址在状态完成时生效
  // The status IN of a no-data request: the address of SET_ADDRESS applies when the
  // status completes
  b.Reset(USBHS_USB_SPEED_HIGH);
  b.Setup({0x00U, 5U, 9U, 0U, 0U});
  TEST_ASSERT((fake_usbhsd.UEP0_TX_CTRL & USBHS_UEP_T_RES_MASK) == USBHS_UEP_T_RES_ACK);
  LatchHs(USBHS_UIF_TRANSFER, USBHS_UIS_TOKEN_IN | USBHS_UIS_IS_NAK);
  RunHs();
  TEST_ASSERT(fake_usbhsd.DEV_AD == 9U);
  TEST_ASSERT((fake_usbhsd.UEP0_TX_CTRL & USBHS_UEP_T_RES_MASK) == USBHS_UEP_T_RES_NAK);
  b.Setup({0x00U, 9U, 1U, 0U, 0U});
  b.Ep0InAck();
  TEST_ASSERT(b.cls.in != nullptr && b.cls.out != nullptr);
  // 类的控制读：状态 OUT（零长度）带 IS_NAK，传输照常完成，不是被下一个 SETUP 顶掉
  // A class control read: the status OUT (zero length) carries IS_NAK, and the transfer
  // completes as usual instead of being superseded by the next SETUP
  b.Setup({0xa1U, REQ_IN, 0U, 0U, 4U});
  b.Ep0InAck();
  TEST_ASSERT(b.cls.completes == 0U);
  LatchHs(USBHS_UIF_TRANSFER, USBHS_UIS_TOKEN_OUT | USBHS_UIS_IS_NAK, 0U);
  RunHs();
  TEST_ASSERT(b.cls.completes == 1U && b.cls.aborts == 0U);
  // 类的无数据请求：状态 IN 带 IS_NAK / A class no-data request: the status IN carries
  // IS_NAK
  b.Setup({0x21U, REQ_OUT, 0U, 0U, 0U});
  LatchHs(USBHS_UIF_TRANSFER, USBHS_UIS_TOKEN_IN | USBHS_UIS_IS_NAK);
  RunHs();
  TEST_ASSERT(b.cls.completes == 2U && b.cls.aborts == 0U);
  // 下一个传输照常 / The next transfer works as usual
  b.Setup({0x80U, 6U, 0x0100U, 0U, 18U});
  TEST_ASSERT(fake_usbhsd.UEP0_TX_LEN == 18U && b.cls.aborts == 0U);
  b.Ep0InAck();
  b.Ep0Out({});
}

/// 带 IS_NAK 的 OUT 事件按 toggle 正确的新包处理；不带 IS_NAK 的 TOG_OK=0 是重发
/// An OUT event with IS_NAK is a new packet in step; TOG_OK=0 without IS_NAK is a
/// resend
void TestHsOutIsNak()
{
  ResetFakes();
  HsBench b;
  b.Enumerate();
  TEST_ASSERT(b.cls.out->Receive(1024U, false) == ErrorCode::OK);
  PendedHs();
  const uint8_t tog0 = fake_usbhsd.UEP1_RX_CTRL & USBHS_UEP_R_TOG_MASK;
  // 重发：不计、不翻 / Resend: not counted, no flip
  LatchHs(USBHS_UIF_TRANSFER, USBHS_UIS_TOKEN_OUT | 1U, 512U);
  RunHs();
  TEST_ASSERT((fake_usbhsd.UEP1_RX_CTRL & USBHS_UEP_R_TOG_MASK) == tog0);
  // IS_NAK、TOG_OK=0：计入、翻转 / IS_NAK, TOG_OK=0: counted, flipped
  LatchHs(USBHS_UIF_TRANSFER, USBHS_UIS_TOKEN_OUT | USBHS_UIS_IS_NAK | 1U, 512U);
  RunHs();
  TEST_ASSERT((fake_usbhsd.UEP1_RX_CTRL & USBHS_UEP_R_TOG_MASK) != tog0);
  LatchHs(USBHS_UIF_TRANSFER, USBHS_UIS_TOKEN_OUT | USBHS_UIS_TOG_OK | 1U, 512U);
  RunHs();
  TEST_ASSERT(b.cls.out_done.size() == 1U && b.cls.out_done[0].result == ErrorCode::OK &&
              b.cls.out_done[0].actual == 1024U);
  // 停止时锁存的 IS_NAK 包也计入 / A latched IS_NAK packet counts in a stop too
  TEST_ASSERT(b.cls.out->Receive(1024U, false) == ErrorCode::OK);
  PendedHs();
  LatchHs(USBHS_UIF_TRANSFER, USBHS_UIS_TOKEN_OUT | USBHS_UIS_IS_NAK | 1U, 512U);
  b.cls.out->Flush(false);
  TEST_ASSERT(b.cls.out_done.size() == 2U && b.cls.out_done[1].actual == 512U);

  // SETUP 已锁存：它清掉了挂着的 OUT 事件的 TOG_OK，又不置 IS_NAK。这个包是新包，计入、
  // 翻转
  // A SETUP is latched: it cleared the TOG_OK of the pending OUT event and set no
  // IS_NAK. The packet is a new one: counted and flipped
  TEST_ASSERT(b.cls.out->Receive(1024U, false) == ErrorCode::OK);
  PendedHs();
  const uint8_t tog1 = fake_usbhsd.UEP1_RX_CTRL & USBHS_UEP_R_TOG_MASK;
  b.Setup({0x80U, 6U, 0x0100U, 0U, 18U}, USBHS_UIF_TRANSFER, USBHS_UIS_TOKEN_OUT | 1U,
          512U);
  TEST_ASSERT((fake_usbhsd.UEP1_RX_CTRL & USBHS_UEP_R_TOG_MASK) != tog1);
  LatchHs(USBHS_UIF_TRANSFER, USBHS_UIS_TOKEN_OUT | USBHS_UIS_TOG_OK | 1U, 100U);
  RunHs();
  TEST_ASSERT(b.cls.out_done.size() == 3U && b.cls.out_done[2].result == ErrorCode::OK &&
              b.cls.out_done[2].actual == 612U);
  // 停止收下的事件同样处理 / An event collected by a stop is treated alike
  TEST_ASSERT(b.cls.out->Receive(1024U, false) == ErrorCode::OK);
  PendedHs();
  LatchHs(USBHS_UIF_TRANSFER | USBHS_UIF_SETUP_ACT, USBHS_UIS_TOKEN_OUT | 1U, 512U);
  b.cls.out->Flush(false);
  TEST_ASSERT(b.cls.out_done.size() == 4U && b.cls.out_done[3].actual == 512U);
}

// ---------------------------------------------------------------------------------------
// USBFS
// ---------------------------------------------------------------------------------------

void RunFs()
{
  fake_pfic.pending[USBFS_IRQn] = true;
  for (int guard = 0; fake_pfic.pending[USBFS_IRQn] && fake_pfic.enabled[USBFS_IRQn];
       ++guard)
  {
    TEST_ASSERT(guard < 100);
    fake_pfic.pending[USBFS_IRQn] = false;
    USBFS_IRQHandler();
  }
}

void PendedFs()
{
  if (fake_pfic.pending[USBFS_IRQn])
  {
    RunFs();
  }
}

void LatchFs(uint8_t flags, uint8_t int_st, uint16_t rx_len = 0U)
{
  fake_usbfsd.INT_FG.value = static_cast<uint8_t>(fake_usbfsd.INT_FG.value | flags);
  fake_usbfsd.INT_ST = int_st;
  fake_usbfsd.RX_LEN = rx_len;
}

/// USBFS：EP1 OUT 单向、EP2 IN 单向，EP3 两个方向共享
/// USBFS: EP1 OUT single, EP2 IN single, EP3 shared by both directions
struct FsBench
{
  Block<64> ep0;
  Block<256> ep1_out;
  Block<256> ep2_in;
  Block<128> ep3;
  PairClass single;
  PairClass shared;
  ResettableUsb<CH32USBOtgFS> usb;

  explicit FsBench(Type in_type = Type::BULK)
      : single(EPNumber::EP2, EPNumber::EP1, in_type, 64U),
        shared(EPNumber::EP3, EPNumber::EP3, Type::BULK, 32U),
        usb({ep0.Raw(), ep1_out.Raw(), {nullptr, 0U}, {ep3.bytes, 64U}},
            {ep0.Raw(), {nullptr, 0U}, ep2_in.Raw(), {ep3.bytes + 64, 64U}},
            USB::DeviceDescriptor::PacketSize0::SIZE_64, 0x1209U, 0x0001U, 0x0100U,
            {&LANGUAGE}, {{&single, &shared}})
  {
    usb.Init(false);
    usb.Start(false);
    PendedFs();
    fake_usbfsd.MIS_ST = USBFS_UMS_SIE_FREE;
  }

  void Setup(const SetupPacket& setup)
  {
    std::memcpy(ep0.bytes, &setup, sizeof(setup));
    LatchFs(USBFS_UIF_TRANSFER, USBFS_UIS_TOKEN_SETUP);
    RunFs();
  }

  void Ep0InAck()
  {
    TEST_ASSERT((fake_usbfsd.UEP0_TX_CTRL & USBFS_UEP_T_RES_MASK) == USBFS_UEP_T_RES_ACK);
    LatchFs(USBFS_UIF_TRANSFER, USBFS_UIS_TOKEN_IN);
    RunFs();
  }

  void Enumerate()
  {
    LatchFs(USBFS_UIF_BUS_RST, 0U);
    RunFs();
    Setup({0x00U, 5U, 7U, 0U, 0U});
    Ep0InAck();
    TEST_ASSERT((fake_usbfsd.DEV_ADDR & 0x7fU) == 7U);
    Setup({0x00U, 9U, 1U, 0U, 0U});
    Ep0InAck();
    TEST_ASSERT(single.in != nullptr && shared.out != nullptr);
  }
};

/// 单向和共享端点的配置和上限 / Configuration and limits of single and shared endpoints
void TestFsEnumerateLimits()
{
  ResetFakes();
  FsBench b;
  b.Enumerate();
  // SOF 中断常开 / The SOF interrupt is always on
  TEST_ASSERT((fake_usbfsd.INT_EN & USBFS_UIE_HST_SOF) != 0U);
  TEST_ASSERT(b.single.out->MaxPacketSize() == 64U);
  TEST_ASSERT(b.single.out->MaxTransferSize() == 128U);  // 槽 128 / slot of 128
  // 共享块包长 32：OUT 每包完成，只许一包；IN 可以两包
  // Shared block with 32-byte packets: OUT completes per packet and allows one; IN two
  TEST_ASSERT(b.shared.out->MaxPacketSize() == 32U);
  TEST_ASSERT(b.shared.out->MaxTransferSize() == 32U);
  TEST_ASSERT(b.shared.in->MaxTransferSize() == 64U);
  TEST_ASSERT(fake_usbfsd.UEP3_DMA == Addr(b.ep3.bytes));
  TEST_ASSERT((fake_usbfsd.UEP2_3_MOD & (USBFS_UEP3_RX_EN | USBFS_UEP3_TX_EN)) ==
              (USBFS_UEP3_RX_EN | USBFS_UEP3_TX_EN));
}

/// 单向 OUT 两包请求收一包后停止报 64；锁存的第二包也计入
/// A two-packet single OUT request stopped after one packet reports 64; a
/// latched second packet counts too
void TestFsOutStop()
{
  ResetFakes();
  FsBench b;
  b.Enumerate();
  TEST_ASSERT(b.single.out->Receive(128U, false) == ErrorCode::OK);
  PendedFs();
  const uint32_t slot = fake_usbfsd.UEP1_DMA;
  LatchFs(USBFS_UIF_TRANSFER, USBFS_UIS_TOKEN_OUT | USBFS_UIS_TOG_OK | 1U, 64U);
  RunFs();
  TEST_ASSERT(fake_usbfsd.UEP1_DMA == slot + 64U);
  b.single.out->Flush(false);
  TEST_ASSERT(b.single.out_done.size() == 1U && b.single.out_done[0].actual == 64U &&
              b.single.out_done[0].result == ErrorCode::ABORTED);

  TEST_ASSERT(b.single.out->Receive(128U, false) == ErrorCode::OK);
  PendedFs();
  LatchFs(USBFS_UIF_TRANSFER, USBFS_UIS_TOKEN_OUT | USBFS_UIS_TOG_OK | 1U, 64U);
  RunFs();
  LatchFs(USBFS_UIF_TRANSFER, USBFS_UIS_TOKEN_OUT | USBFS_UIS_TOG_OK | 1U, 64U);
  b.single.out->Flush(false);
  TEST_ASSERT(b.single.out_done.size() == 2U && b.single.out_done[1].actual == 128U);
  TEST_ASSERT((fake_usbfsd.INT_FG & USBFS_UIF_TRANSFER) == 0U);

  // TOG_OK=0 的重发包不计 / A resend with TOG_OK=0 does not count
  TEST_ASSERT(b.single.out->Receive(128U, false) == ErrorCode::OK);
  PendedFs();
  LatchFs(USBFS_UIF_TRANSFER, USBFS_UIS_TOKEN_OUT | 1U, 64U);
  b.single.out->Flush(false);
  TEST_ASSERT(b.single.out_done.size() == 3U && b.single.out_done[2].actual == 0U);
}

/// 单向 IN 被锁存的 ACK：OK；共享 IN 未发：ABORTED 0
/// Single IN with a latched ACK: OK; shared IN not sent: ABORTED 0
void TestFsInStop()
{
  ResetFakes();
  FsBench b;
  b.Enumerate();
  b.single.Send(64U);
  PendedFs();
  TEST_ASSERT(fake_usbfsd.UEP2_TX_LEN == 64U);
  LatchFs(USBFS_UIF_TRANSFER, USBFS_UIS_TOKEN_IN | 2U);
  b.single.in->Flush(false);
  TEST_ASSERT(b.single.in_done.size() == 1U &&
              b.single.in_done[0].result == ErrorCode::OK &&
              b.single.in_done[0].actual == 64U);

  // 共享 IN：DMA 不动，数据从槽拷进硬件发送区（基址 +64）
  // Shared IN: the DMA stays, the data is copied from the slot into the hardware
  // transmit area (base +64)
  std::memset(b.ep3.bytes, 0, sizeof(b.ep3.bytes));
  b.shared.Send(20U);
  PendedFs();
  TEST_ASSERT(fake_usbfsd.UEP3_DMA == Addr(b.ep3.bytes));
  TEST_ASSERT(b.ep3.bytes[64] == 0x5aU && b.ep3.bytes[83] == 0x5aU &&
              b.ep3.bytes[84] == 0U);
  b.shared.in->Flush(false);
  TEST_ASSERT(b.shared.in_done.size() == 1U &&
              b.shared.in_done[0].result == ErrorCode::ABORTED &&
              b.shared.in_done[0].actual == 0U);
}

/// 共享 OUT：包从硬件接收区拷进槽；停止未结束时槽当场放回，晚到的包只丢弃
/// Shared OUT: the packet is copied from the hardware receive area into the slot; with
/// a stop pending the slot is given back at once and a late packet is only dropped
void TestFsSharedOut()
{
  ResetFakes();
  FsBench b;
  b.Enumerate();
  TEST_ASSERT(b.shared.out->Receive(32U, false) == ErrorCode::OK);
  PendedFs();
  std::memset(b.ep3.bytes, 0xa7, 32U);
  LatchFs(USBFS_UIF_TRANSFER, USBFS_UIS_TOKEN_OUT | USBFS_UIS_TOG_OK | 3U, 32U);
  RunFs();
  TEST_ASSERT(b.shared.out_done.size() == 1U &&
              b.shared.out_done[0].result == ErrorCode::OK &&
              b.shared.out_done[0].data == Bytes(32U, 0xa7U));

  // 线上有事务时停止：回调当场 ABORTED，槽已经可以再用
  // Stopped with a transaction on the wire: ABORTED at once, the slot is usable again
  TEST_ASSERT(b.shared.out->Receive(32U, false) == ErrorCode::OK);
  PendedFs();
  fake_usbfsd.MIS_ST = 0U;
  b.shared.out->Flush(false);
  TEST_ASSERT(b.shared.out_done.size() == 2U &&
              b.shared.out_done[1].result == ErrorCode::ABORTED);
  TEST_ASSERT(CH32EndpointOtgFs::PendingStops() != 0U);
  TEST_ASSERT(b.shared.out->Receive(32U, false) == ErrorCode::OK);
  TEST_ASSERT((fake_usbfsd.UEP3_RX_CTRL & USBFS_UEP_R_RES_MASK) == USBFS_UEP_R_RES_NAK);
  // 被停的那个事务结束：包不进槽，停止结束，排队的接收启动
  // The stopped transaction ends: the packet stays out of the slot, the stop ends and
  // the queued receive starts
  std::memset(b.ep3.bytes, 0x11, 32U);
  LatchFs(USBFS_UIF_TRANSFER, USBFS_UIS_TOKEN_OUT | USBFS_UIS_TOG_OK | 3U, 32U);
  fake_usbfsd.MIS_ST = USBFS_UMS_SIE_FREE;
  RunFs();
  TEST_ASSERT(b.shared.out_done.size() == 2U);
  TEST_ASSERT(CH32EndpointOtgFs::PendingStops() == 0U);
  TEST_ASSERT((fake_usbfsd.UEP3_RX_CTRL & USBFS_UEP_R_RES_MASK) == USBFS_UEP_R_RES_ACK);
  std::memset(b.ep3.bytes, 0x22, 32U);
  LatchFs(USBFS_UIF_TRANSFER, USBFS_UIS_TOKEN_OUT | USBFS_UIS_TOG_OK | 3U, 32U);
  RunFs();
  TEST_ASSERT(b.shared.out_done.size() == 3U &&
              b.shared.out_done[2].data == Bytes(32U, 0x22U));
}

/// 单向端点的停止不等线上的事务：由本端点的完成、之后的 SIE 空闲或总线复位结束
/// A stop of a single-direction endpoint does not wait for the transaction on the wire:
/// its own completion, the SIE free later, or a bus reset ends it
void TestFsPendingStop()
{
  ResetFakes();
  FsBench b;
  b.Enumerate();

  // 本端点的完成结束停止；被停的槽这时才放回
  // Its own completion ends the stop; only then is the stopped slot given back
  TEST_ASSERT(b.single.out->Receive(64U, false) == ErrorCode::OK);
  PendedFs();
  fake_usbfsd.MIS_ST = 0U;
  b.single.out->Flush(false);
  TEST_ASSERT(b.single.out_done.size() == 1U &&
              b.single.out_done[0].result == ErrorCode::ABORTED);
  TEST_ASSERT((fake_usbfsd.UEP1_RX_CTRL & USBFS_UEP_R_RES_MASK) == USBFS_UEP_R_RES_NAK);
  TEST_ASSERT(CH32EndpointOtgFs::PendingStops() != 0U);
  TEST_ASSERT(b.single.out->Receive(64U, false) == ErrorCode::OK);
  TEST_ASSERT(b.single.out->Receive(64U, false) == ErrorCode::BUSY);
  // SIE 仍忙：结束停止的是这个完成事件本身 / The SIE is still busy: the completion
  // event itself ends the stop
  LatchFs(USBFS_UIF_TRANSFER, USBFS_UIS_TOKEN_OUT | USBFS_UIS_TOG_OK | 1U, 64U);
  RunFs();
  TEST_ASSERT(b.single.out_done.size() == 1U);
  TEST_ASSERT(CH32EndpointOtgFs::PendingStops() == 0U);
  TEST_ASSERT((fake_usbfsd.UEP1_RX_CTRL & USBFS_UEP_R_RES_MASK) == USBFS_UEP_R_RES_ACK);
  fake_usbfsd.MIS_ST = USBFS_UMS_SIE_FREE;
  TEST_ASSERT(b.single.out->Receive(64U, false) == ErrorCode::OK);
  b.single.out->Flush(false);
  TEST_ASSERT(b.single.out_done.size() == 3U);

  // 没有完成事件、SIE 一直忙（别的端点的流量）：写 NAK 之后的第一个帧起始结束停止；
  // 停止之前就锁存的 SOF 不结束它，但照样报告给设备类
  // No completion event and a SIE that stays busy (other endpoints' traffic): the first
  // start of frame after the NAK ends the stop; a SOF latched before the stop does not
  // end it, but is reported to the device class all the same
  b.single.Send(64U);
  PendedFs();
  fake_usbfsd.MIS_ST = 0U;
  const size_t frames = b.single.frames.size();
  LatchFs(USBFS_UIF_HST_SOF, 0x70U);
  b.single.in->Flush(false);
  TEST_ASSERT(b.single.in_done.size() == 1U &&
              b.single.in_done[0].result == ErrorCode::ABORTED);
  TEST_ASSERT((fake_usbfsd.INT_FG & USBFS_UIF_HST_SOF) != 0U);
  RunFs();
  TEST_ASSERT(CH32EndpointOtgFs::PendingStops() != 0U);
  TEST_ASSERT(b.single.frames.size() == frames + 1U);
  LatchFs(USBFS_UIF_HST_SOF, 0x70U);
  RunFs();
  TEST_ASSERT(CH32EndpointOtgFs::PendingStops() == 0U);
  TEST_ASSERT(b.single.frames.size() == frames + 2U);
  // 这个控制器没有帧号寄存器：报“不知道” / This controller has no frame number
  // register: reported as unknown
  TEST_ASSERT(b.single.frames.back() == static_cast<uint16_t>(0xFFFFU | (0xFFU << 12U)));
  // 总线挂起期间来的 SOF 标志不报告 / A SOF flag while the bus is suspended is not
  // reported
  fake_usbfsd.MIS_ST = USBFS_UMS_SUSPEND;
  LatchFs(USBFS_UIF_HST_SOF, 0x70U);
  RunFs();
  TEST_ASSERT(b.single.frames.size() == frames + 2U);
  fake_usbfsd.MIS_ST = 0U;
  LatchFs(USBFS_UIF_HST_SOF, 0x70U);
  RunFs();
  TEST_ASSERT(b.single.frames.size() == frames + 3U);
  TEST_ASSERT(b.single.in_done.size() == 1U);
  fake_usbfsd.MIS_ST = USBFS_UMS_SIE_FREE;

  // 没有任何事件、之后读到 SIE 空闲的一轮结束停止
  // With no event at all, a round that reads the SIE free ends the stop
  b.single.Send(64U);
  PendedFs();
  fake_usbfsd.MIS_ST = 0U;
  b.single.in->Flush(false);
  TEST_ASSERT(b.single.in_done.size() == 2U);
  RunFs();
  TEST_ASSERT(CH32EndpointOtgFs::PendingStops() != 0U);
  fake_usbfsd.MIS_ST = USBFS_UMS_SIE_FREE;
  RunFs();
  TEST_ASSERT(CH32EndpointOtgFs::PendingStops() == 0U);

  // 总线复位在报告之前结束未结束的停止，不看 SIE 是否空闲；之后照常枚举
  // A bus reset ends the pending stop before it is reported, whatever the SIE reads;
  // enumeration works afterwards
  b.single.Send(64U);
  PendedFs();
  fake_usbfsd.MIS_ST = 0U;
  b.single.in->Flush(false);
  TEST_ASSERT(b.single.in_done.size() == 3U);
  LatchFs(USBFS_UIF_BUS_RST, 0U);
  RunFs();
  TEST_ASSERT(CH32EndpointOtgFs::PendingStops() == 0U);
  fake_usbfsd.MIS_ST = USBFS_UMS_SIE_FREE;
  b.Enumerate();
}

/// 同一轮快照里的 SOF 不结束本轮才发出的停止；SIE 一直忙也不复位控制器
/// A SOF in the snapshot of a round does not end a stop issued within that round; a SIE
/// that stays busy never resets the controller
void TestFsStopAtSof()
{
  ResetFakes();
  FsBench b;
  b.Enumerate();
  TEST_ASSERT(b.single.out->Receive(64U, false) == ErrorCode::OK);
  b.single.Send(64U);
  PendedFs();
  fake_usbfsd.MIS_ST = 0U;
  // OUT 完成的回调里停 IN；这一轮的快照同时带着 SOF
  // The OUT completion callback stops IN; the snapshot of this round carries a SOF too
  b.single.on_out = [&b]() { b.single.in->Flush(true); };
  LatchFs(USBFS_UIF_TRANSFER | USBFS_UIF_HST_SOF,
          USBFS_UIS_TOKEN_OUT | USBFS_UIS_TOG_OK | 1U, 64U);
  RunFs();
  b.single.on_out = nullptr;
  TEST_ASSERT(b.single.out_done.size() == 1U && b.single.in_done.size() == 1U);
  TEST_ASSERT(CH32EndpointOtgFs::PendingStops() != 0U);
  // 之后每一帧都只有 SOF、SIE 忙：下一个 SOF 就结束，不报停止失败
  // From here every frame brings a SOF only and the SIE stays busy: the next SOF ends
  // the stop and no stop failure is reported
  fake_usbfsd.BASE_CTRL = 0U;
  LatchFs(USBFS_UIF_HST_SOF, 0x70U);
  RunFs();
  TEST_ASSERT(CH32EndpointOtgFs::PendingStops() == 0U);
  TEST_ASSERT((fake_usbfsd.BASE_CTRL & USBFS_UC_DEV_PU_EN) == 0U);
  for (int i = 0; i < 20; ++i)
  {
    LatchFs(USBFS_UIF_HST_SOF, 0x70U);
    RunFs();
  }
  TEST_ASSERT((fake_usbfsd.BASE_CTRL & USBFS_UC_DEV_PU_EN) == 0U);
  TEST_ASSERT(b.single.in != nullptr);
  b.single.Send(10U);
  PendedFs();
  TEST_ASSERT((fake_usbfsd.UEP2_TX_CTRL & USBFS_UEP_T_RES_MASK) == USBFS_UEP_T_RES_ACK);
  fake_usbfsd.MIS_ST = USBFS_UMS_SIE_FREE;
  b.single.in->Flush(false);
}

/// 全速批量端点的 DATA0/DATA1 由软件管：自动翻转不开，每个被 ACK 的包翻一次，停止的读
/// 改写保留它
/// DATA0/DATA1 of a full-speed bulk endpoint is kept by software: the automatic toggle is
/// off, every acknowledged packet flips it once, and the read-modify-write of a stop
/// keeps it
void TestFsSoftwareToggle()
{
  ResetFakes();
  FsBench b;
  b.Enumerate();
  TEST_ASSERT((fake_usbfsd.UEP2_TX_CTRL & (USBFS_UEP_T_AUTO_TOG | USBFS_UEP_T_TOG)) ==
              0U);
  TEST_ASSERT((fake_usbfsd.UEP1_RX_CTRL & (USBFS_UEP_R_AUTO_TOG | USBFS_UEP_R_TOG)) ==
              0U);

  // IN：两包的请求，每个完成事件翻一次 / IN: a two-packet request, one flip per
  // completion event
  b.single.Send(128U);
  PendedFs();
  TEST_ASSERT((fake_usbfsd.UEP2_TX_CTRL & USBFS_UEP_T_TOG) == 0U);
  LatchFs(USBFS_UIF_TRANSFER, USBFS_UIS_TOKEN_IN | 2U);
  RunFs();
  TEST_ASSERT((fake_usbfsd.UEP2_TX_CTRL & USBFS_UEP_T_TOG) != 0U);
  TEST_ASSERT((fake_usbfsd.UEP2_TX_CTRL & USBFS_UEP_T_RES_MASK) == USBFS_UEP_T_RES_ACK);
  LatchFs(USBFS_UIF_TRANSFER, USBFS_UIS_TOKEN_IN | 2U);
  RunFs();
  TEST_ASSERT((fake_usbfsd.UEP2_TX_CTRL & USBFS_UEP_T_TOG) == 0U);
  TEST_ASSERT(b.single.in_done.size() == 1U &&
              b.single.in_done[0].result == ErrorCode::OK);

  // 停止收下锁存的完成：也翻 / A stop that collects a latched completion flips too
  b.single.Send(64U);
  PendedFs();
  LatchFs(USBFS_UIF_TRANSFER, USBFS_UIS_TOKEN_IN | 2U);
  b.single.in->Flush(false);
  TEST_ASSERT((fake_usbfsd.UEP2_TX_CTRL & USBFS_UEP_T_TOG) != 0U);
  // 停止时没有完成：不翻，写 NAK 的读改写保留 toggle
  // A stop without a completion: no flip, and the read-modify-write of NAK keeps the
  // toggle
  b.single.Send(64U);
  PendedFs();
  b.single.in->Flush(false);
  TEST_ASSERT((fake_usbfsd.UEP2_TX_CTRL & USBFS_UEP_T_TOG) != 0U);
  TEST_ASSERT((fake_usbfsd.UEP2_TX_CTRL & USBFS_UEP_T_RES_MASK) == USBFS_UEP_T_RES_NAK);

  // OUT：TOG_OK 的包翻；TOG_OK=0 是重发，不翻、不计入
  // OUT: a packet with TOG_OK flips; TOG_OK=0 is a resend, neither flipped nor counted
  TEST_ASSERT(b.single.out->Receive(128U, false) == ErrorCode::OK);
  PendedFs();
  LatchFs(USBFS_UIF_TRANSFER, USBFS_UIS_TOKEN_OUT | USBFS_UIS_TOG_OK | 1U, 64U);
  RunFs();
  TEST_ASSERT((fake_usbfsd.UEP1_RX_CTRL & USBFS_UEP_R_TOG) != 0U);
  LatchFs(USBFS_UIF_TRANSFER, USBFS_UIS_TOKEN_OUT | 1U, 64U);
  RunFs();
  TEST_ASSERT((fake_usbfsd.UEP1_RX_CTRL & USBFS_UEP_R_TOG) != 0U);
  TEST_ASSERT(b.single.out_done.empty());
  LatchFs(USBFS_UIF_TRANSFER, USBFS_UIS_TOKEN_OUT | USBFS_UIS_TOG_OK | 1U, 64U);
  RunFs();
  TEST_ASSERT((fake_usbfsd.UEP1_RX_CTRL & USBFS_UEP_R_TOG) == 0U);
  TEST_ASSERT(b.single.out_done.size() == 1U && b.single.out_done[0].actual == 128U);
  // 没有请求在途时到的 TOG_OK 包也翻 / A TOG_OK packet with no request in flight flips
  // too
  LatchFs(USBFS_UIF_TRANSFER, USBFS_UIS_TOKEN_OUT | USBFS_UIS_TOG_OK | 1U, 64U);
  RunFs();
  TEST_ASSERT((fake_usbfsd.UEP1_RX_CTRL & USBFS_UEP_R_TOG) != 0U);
}

/// 端点没有 STALL、硬件上有请求时清 HALT：软件 toggle 回到 DATA0，RES 保持 ACK，请求
/// 继续等主机，多包请求的下一包收进来或发出去并完成；STALL 时清和原来一样
/// A clear of the halt of an endpoint that is not halted, with requests on the hardware:
/// the software toggle returns to DATA0, RES stays ACK, the request waits for the host
/// and the next packet of a multi-packet request is taken or sent and completes it; a
/// clear while halted is as before
void TestFsClearHaltKeepsRequests()
{
  ResetFakes();
  FsBench b;
  b.Enumerate();
  const auto clear = [&b](uint8_t address)
  {
    b.Setup(ClearHalt(address));
    b.Ep0InAck();
  };

  // OUT：两包请求收了一包，toggle 为 DATA1 / OUT: a two-packet request after one packet,
  // toggle at DATA1
  TEST_ASSERT(b.single.out->Receive(128U, false) == ErrorCode::OK);
  PendedFs();
  LatchFs(USBFS_UIF_TRANSFER, USBFS_UIS_TOKEN_OUT | USBFS_UIS_TOG_OK | 1U, 64U);
  RunFs();
  TEST_ASSERT((fake_usbfsd.UEP1_RX_CTRL & USBFS_UEP_R_TOG) != 0U);
  clear(0x01U);
  TEST_ASSERT((fake_usbfsd.UEP1_RX_CTRL & USBFS_UEP_R_RES_MASK) == USBFS_UEP_R_RES_ACK);
  TEST_ASSERT((fake_usbfsd.UEP1_RX_CTRL & USBFS_UEP_R_TOG) == 0U);
  TEST_ASSERT(b.single.out_done.empty());
  LatchFs(USBFS_UIF_TRANSFER, USBFS_UIS_TOKEN_OUT | USBFS_UIS_TOG_OK | 1U, 20U);
  RunFs();
  TEST_ASSERT(b.single.out_done.size() == 1U && b.single.out_done[0].actual == 84U);

  // IN：两包请求发了一包，第二包挂在 DATA1 / IN: a two-packet request after one packet,
  // the second armed at DATA1
  b.single.Send(128U);
  PendedFs();
  LatchFs(USBFS_UIF_TRANSFER, USBFS_UIS_TOKEN_IN | 2U);
  RunFs();
  TEST_ASSERT((fake_usbfsd.UEP2_TX_CTRL & USBFS_UEP_T_TOG) != 0U);
  clear(0x82U);
  TEST_ASSERT((fake_usbfsd.UEP2_TX_CTRL & USBFS_UEP_T_RES_MASK) == USBFS_UEP_T_RES_ACK);
  TEST_ASSERT((fake_usbfsd.UEP2_TX_CTRL & USBFS_UEP_T_TOG) == 0U);
  TEST_ASSERT(b.single.in_done.empty());
  LatchFs(USBFS_UIF_TRANSFER, USBFS_UIS_TOKEN_IN | 2U);
  RunFs();
  TEST_ASSERT(b.single.in_done.size() == 1U &&
              b.single.in_done[0].result == ErrorCode::OK);
  TEST_ASSERT(b.single.in_done[0].actual == 128U);

  // 共享块 OUT 挂着接收 / A shared-block OUT with a receive armed
  TEST_ASSERT(b.shared.out->Receive(32U, false) == ErrorCode::OK);
  PendedFs();
  fake_usbfsd.UEP3_RX_CTRL |= USBFS_UEP_R_TOG;
  clear(0x03U);
  TEST_ASSERT((fake_usbfsd.UEP3_RX_CTRL & USBFS_UEP_R_RES_MASK) == USBFS_UEP_R_RES_ACK);
  TEST_ASSERT((fake_usbfsd.UEP3_RX_CTRL & USBFS_UEP_R_TOG) == 0U);
  LatchFs(USBFS_UIF_TRANSFER, USBFS_UIS_TOKEN_OUT | USBFS_UIS_TOG_OK | 3U, 7U);
  RunFs();
  TEST_ASSERT(b.shared.out_done.size() == 1U && b.shared.out_done[0].actual == 7U);

  // STALL 时清：NAK、DATA0 / A clear while halted: NAK and DATA0
  TEST_ASSERT(b.single.out->Stall(false) == ErrorCode::OK);
  PendedFs();
  clear(0x01U);
  TEST_ASSERT((fake_usbfsd.UEP1_RX_CTRL & USBFS_UEP_R_RES_MASK) == USBFS_UEP_R_RES_NAK);
  TEST_ASSERT((fake_usbfsd.UEP1_RX_CTRL & USBFS_UEP_R_TOG) == 0U);
  TEST_ASSERT(b.single.in->Stall(false) == ErrorCode::OK);
  PendedFs();
  clear(0x82U);
  TEST_ASSERT((fake_usbfsd.UEP2_TX_CTRL & USBFS_UEP_T_RES_MASK) == USBFS_UEP_T_RES_NAK);
  TEST_ASSERT((fake_usbfsd.UEP2_TX_CTRL & USBFS_UEP_T_TOG) == 0U);
}

/// 等时事务不受 INT_BUSY 约束：处理一个完成的那一轮里，等时 IN 包被主机取走，事件并进
/// 已经置位的标志。清标志之后按 INT_ST 补上它的完成；带 IS_NAK 的、指向非等时端点的不补
/// Isochronous transactions are not held off by INT_BUSY: within the round that handles
/// one completion the host takes the isochronous IN packet and the event merges into
/// the flag already set. After the clear its completion is delivered from INT_ST; one
/// with IS_NAK or pointing at a non-isochronous endpoint is not
void TestFsIsoMergedEvent()
{
  ResetFakes();
  FsBench b(Type::ISOCHRONOUS);
  b.Enumerate();
  TEST_ASSERT(b.single.in->GetType() == Type::ISOCHRONOUS);
  TEST_ASSERT(b.single.out->Receive(64U, false) == ErrorCode::OK);
  b.single.Send(32U);
  PendedFs();
  b.single.on_out = []() { fake_usbfsd.INT_ST = USBFS_UIS_TOKEN_IN | 2U; };
  LatchFs(USBFS_UIF_TRANSFER, USBFS_UIS_TOKEN_OUT | USBFS_UIS_TOG_OK | 1U, 64U);
  RunFs();
  TEST_ASSERT(b.single.out_done.size() == 1U);
  TEST_ASSERT(b.single.in_done.size() == 1U &&
              b.single.in_done[0].result == ErrorCode::OK &&
              b.single.in_done[0].actual == 32U);
  // 等时 IN 留在 DATA0，完成不翻 / The isochronous IN stays at DATA0; a completion does
  // not flip it
  TEST_ASSERT((fake_usbfsd.UEP2_TX_CTRL & USBFS_UEP_T_TOG) == 0U);
  // 没有别的事件时不会再交付一次 / It is not delivered again with no further event
  RunFs();
  TEST_ASSERT(b.single.in_done.size() == 1U);

  // 带 IS_NAK：那是被拒的令牌，不是完成 / With IS_NAK: a refused token, no completion
  TEST_ASSERT(b.single.out->Receive(64U, false) == ErrorCode::OK);
  b.single.Send(32U);
  PendedFs();
  b.single.on_out = []()
  { fake_usbfsd.INT_ST = USBFS_UIS_TOKEN_IN | USBFS_UIS_IS_NAK | 2U; };
  LatchFs(USBFS_UIF_TRANSFER, USBFS_UIS_TOKEN_OUT | USBFS_UIS_TOG_OK | 1U, 64U);
  RunFs();
  TEST_ASSERT(b.single.out_done.size() == 2U && b.single.in_done.size() == 1U);

  // 指向非等时端点（EP3 IN，批量）：不补 / Pointing at a non-isochronous endpoint (EP3
  // IN, bulk): nothing is delivered
  b.shared.Send(16U);
  PendedFs();
  TEST_ASSERT(b.single.out->Receive(64U, false) == ErrorCode::OK);
  PendedFs();
  b.single.on_out = []() { fake_usbfsd.INT_ST = USBFS_UIS_TOKEN_IN | 3U; };
  LatchFs(USBFS_UIF_TRANSFER, USBFS_UIS_TOKEN_OUT | USBFS_UIS_TOG_OK | 1U, 64U);
  RunFs();
  b.single.on_out = nullptr;
  TEST_ASSERT(b.single.out_done.size() == 3U && b.shared.in_done.empty());
  TEST_ASSERT(b.single.in_done.size() == 1U);

  // 检查之后才结束、并进标志又被清掉的等时 IN：清除之后 INT_ST 只剩端点号可用（令牌位
  // 读成 11），按端点号找到唯一在途的等时请求交付
  // An isochronous IN that ends after the check, merges into the flag and is cleared:
  // after the clear only the endpoint number of INT_ST is usable (the token bits read
  // 11), and the one isochronous request in flight on that number is completed
  TEST_ASSERT(b.single.in_done.size() == 1U);
  fake_int_st_after_clear_target = &fake_usbfsd.INT_ST;
  TEST_ASSERT(b.single.out->Receive(64U, false) == ErrorCode::OK);
  PendedFs();
  fake_int_fg_after_clear_st = static_cast<uint8_t>(USBFS_UIS_TOKEN_MASK | 2U);
  LatchFs(USBFS_UIF_TRANSFER, USBFS_UIS_TOKEN_OUT | USBFS_UIS_TOG_OK | 1U, 64U);
  RunFs();
  TEST_ASSERT(b.single.in_done.size() == 2U &&
              b.single.in_done[1].result == ErrorCode::OK &&
              b.single.in_done[1].actual == 32U);
  // 清除之后的端点号就是刚处理的那个、带 IS_NAK、或指向批量端点：都不交付
  // After the clear the endpoint number is the one just handled, or IS_NAK is set, or
  // it points at a bulk endpoint: nothing is delivered
  b.single.Send(32U);
  PendedFs();
  for (const uint8_t st :
       {static_cast<uint8_t>(USBFS_UIS_TOKEN_MASK | 1U),
        static_cast<uint8_t>(USBFS_UIS_TOKEN_MASK | USBFS_UIS_IS_NAK | 2U),
        static_cast<uint8_t>(USBFS_UIS_TOKEN_MASK | 3U)})
  {
    TEST_ASSERT(b.single.out->Receive(64U, false) == ErrorCode::OK);
    PendedFs();
    fake_int_fg_after_clear_st = st;
    LatchFs(USBFS_UIF_TRANSFER, USBFS_UIS_TOKEN_OUT | USBFS_UIS_TOG_OK | 1U, 64U);
    RunFs();
    TEST_ASSERT(b.single.in_done.size() == 2U && b.shared.in_done.empty());
  }
  fake_int_fg_after_clear_st = 0U;
  fake_int_st_after_clear_target = nullptr;
  LatchFs(USBFS_UIF_TRANSFER, USBFS_UIS_TOKEN_IN | 2U);
  RunFs();
  TEST_ASSERT(b.single.in_done.size() == 3U);
  b.single.in_done.erase(b.single.in_done.begin() + 1, b.single.in_done.end());
  b.single.Send(32U);
  PendedFs();

  // 普通的等时 IN 完成，排队的下一包随即挂上：INT_ST 还是本轮处理的令牌，新挂的包不算
  // 完成
  // A plain isochronous IN completion after which the queued next packet is armed at
  // once: INT_ST is still the token handled in this round, and the packet just armed is
  // not completed
  b.single.Send(16U);
  PendedFs();
  LatchFs(USBFS_UIF_TRANSFER, USBFS_UIS_TOKEN_IN | 2U);
  RunFs();
  TEST_ASSERT(b.single.in_done.size() == 2U);
  TEST_ASSERT(fake_usbfsd.UEP2_TX_LEN == 16U);
  LatchFs(USBFS_UIF_TRANSFER, USBFS_UIS_TOKEN_IN | 2U);
  RunFs();
  TEST_ASSERT(b.single.in_done.size() == 3U && b.single.in_done[2].actual == 16U);
}

/// 全速：Stop 不等总线，未结束的停止由下一个帧起始结束，之后才断开，SOF 中断随之关掉
/// Full speed: Stop does not wait for the bus, the pending stops end at the next start
/// of frame and only then is the bus detached, the SOF interrupt going off with it
void TestFsDeviceStop()
{
  ResetFakes();
  FsBench b;
  b.Enumerate();
  b.single.Send(64U);
  PendedFs();
  fake_usbfsd.MIS_ST = 0U;
  b.single.in->Flush(false);
  TEST_ASSERT(b.single.in_done.size() == 1U);
  TEST_ASSERT(CH32EndpointOtgFs::PendingStops() != 0U);
  TEST_ASSERT(b.single.out->Receive(64U, false) == ErrorCode::OK);
  PendedFs();
  b.usb.Stop(false);
  PendedFs();
  TEST_ASSERT(b.single.in == nullptr && b.single.out == nullptr);
  TEST_ASSERT(b.single.out_done.size() == 1U &&
              b.single.out_done[0].result == ErrorCode::ABORTED);
  TEST_ASSERT((fake_usbfsd.BASE_CTRL & USBFS_UC_DEV_PU_EN) != 0U);
  TEST_ASSERT(CH32EndpointOtgFs::PendingStops() != 0U);
  LatchFs(USBFS_UIF_HST_SOF, 0x70U);
  RunFs();
  TEST_ASSERT((fake_usbfsd.BASE_CTRL & USBFS_UC_DEV_PU_EN) == 0U);
  TEST_ASSERT(fake_usbfsd.INT_EN == 0U && fake_pfic.enabled[USBFS_IRQn]);
  fake_usbfsd.MIS_ST = USBFS_UMS_SIE_FREE;
  // Start 紧跟断开：不等，第 6 个节拍才接上 / Start right after the detach: no wait, the
  // sixth tick attaches
  b.usb.Start(false);
  PendedFs();
  TEST_ASSERT((fake_usbfsd.BASE_CTRL & USBFS_UC_DEV_PU_EN) == 0U);
  TickAnd(5, PendedFs);
  TEST_ASSERT((fake_usbfsd.BASE_CTRL & USBFS_UC_DEV_PU_EN) == 0U);
  TickAnd(1, PendedFs);
  TEST_ASSERT((fake_usbfsd.BASE_CTRL & USBFS_UC_DEV_PU_EN) != 0U);
  b.Enumerate();
  // 之前的停止没有留下计数：新的未结束停止照常打开、关掉 SOF 中断
  // The earlier stop left no count behind: a new pending stop enables and masks the SOF
  // interrupt as usual
  b.single.Send(64U);
  PendedFs();
  fake_usbfsd.MIS_ST = 0U;
  b.single.in->Flush(false);
  TEST_ASSERT(CH32EndpointOtgFs::PendingStops() != 0U);
  LatchFs(USBFS_UIF_HST_SOF, 0x70U);
  RunFs();
  TEST_ASSERT(CH32EndpointOtgFs::PendingStops() == 0U);
  fake_usbfsd.MIS_ST = USBFS_UMS_SIE_FREE;
}

/// 处理 SETUP 期间停控制端点当场完成，即使 SIE 忙
/// Stopping the control endpoint during SETUP handling is complete at once, even with a
/// busy SIE
void TestFsEp0StopAtSetup()
{
  ResetFakes();
  FsBench b;
  b.Enumerate();
  // GET_DESCRIPTOR(device)：EP0 IN 挂上应答，主机不取，直接发下一个 SETUP
  // GET_DESCRIPTOR(device): the reply is armed on EP0 IN, the host does not take it and
  // sends the next SETUP
  b.Setup({0x80U, 6U, 0x0100U, 0U, 18U});
  TEST_ASSERT((fake_usbfsd.UEP0_TX_CTRL & USBFS_UEP_T_RES_MASK) == USBFS_UEP_T_RES_ACK);
  fake_usbfsd.MIS_ST = 0U;
  b.Setup({0x80U, 6U, 0x0100U, 0U, 18U});
  TEST_ASSERT(CH32EndpointOtgFs::PendingStops() == 0U);
  TEST_ASSERT((fake_usbfsd.UEP0_TX_CTRL & USBFS_UEP_T_RES_MASK) == USBFS_UEP_T_RES_ACK);
  TEST_ASSERT(fake_usbfsd.UEP0_TX_LEN == 18U);
}

/// 复位只断开：几十微秒内返回、上拉和中断源关掉；第 6 个节拍挂起中断，owner 接上，
/// 主机以新地址重新枚举
/// The reset only detaches: back within tens of microseconds with the pull-up and the
/// interrupt sources off; the sixth tick pends the interrupt and the owner attaches, and
/// the host enumerates again with a new address
template <typename Bench, typename Pended>
void CheckResetAsync(Bench& b, volatile uint8_t& pu_reg, uint8_t pu_bit,
                     volatile uint8_t& int_en, Pended pended)
{
  b.Enumerate();
  const auto before = std::chrono::steady_clock::now();
  b.usb.ForceReset();
  TEST_ASSERT(std::chrono::steady_clock::now() - before <
              std::chrono::microseconds(1000));
  TEST_ASSERT((pu_reg & pu_bit) == 0U && int_en == 0U);
  TickAnd(5, pended);
  TEST_ASSERT((pu_reg & pu_bit) == 0U);
  TickAnd(1, pended);
  TEST_ASSERT((pu_reg & pu_bit) != 0U && int_en != 0U);
  b.Enumerate();
}

/// 复位的断开期间再复位：计时不重新开始；断开期间 Stop：不接上；之后 Start 就地接上
/// Another reset during a reset's detach: the count does not start over; Stop during
/// the detach: no attach; a later Start attaches in place
template <typename Bench, typename Pended>
void CheckDetachRequests(Bench& b, volatile uint8_t& pu_reg, uint8_t pu_bit,
                         Pended pended)
{
  b.Enumerate();
  b.usb.ForceReset();
  TickAnd(3, pended);
  b.usb.ForceReset();
  TEST_ASSERT((pu_reg & pu_bit) == 0U);
  TickAnd(2, pended);
  TEST_ASSERT((pu_reg & pu_bit) == 0U);
  TickAnd(1, pended);
  TEST_ASSERT((pu_reg & pu_bit) != 0U);
  b.Enumerate();
  b.usb.ForceReset();
  TickAnd(2, pended);
  b.usb.Stop(false);
  pended();
  TickAnd(10, pended);
  TEST_ASSERT((pu_reg & pu_bit) == 0U);
  // 时间早已够：Start 不等，就地接上 / The time passed long ago: Start does not wait
  // and attaches in place
  const auto before = std::chrono::steady_clock::now();
  b.usb.Start(false);
  TEST_ASSERT(std::chrono::steady_clock::now() - before <
              std::chrono::microseconds(1000));
  pended();
  TEST_ASSERT((pu_reg & pu_bit) != 0U);
  b.Enumerate();
}

void TestHsResetAsync()
{
  ResetFakes();
  HsBench b;
  fake_usbhsd.MIS_ST = USBHS_UMS_SIE_FREE;
  CheckResetAsync(b, fake_usbhsd.CONTROL, USBHS_UC_DEV_PU_EN, fake_usbhsd.INT_EN,
                  PendedHs);
}

void TestHsDetachRequests()
{
  ResetFakes();
  HsBench b;
  CheckDetachRequests(b, fake_usbhsd.CONTROL, USBHS_UC_DEV_PU_EN, PendedHs);
}

/// 断开期间 SIE 复位、总线没有活动，MIS_ST 可能显示挂起：不报告挂起。报告了的话核心
/// 关上闸门，断开期间（R1）启动的请求不再上硬件
/// While detached the SIE is reset and the bus idle, so MIS_ST may show suspend: no
/// suspend is reported. Were it reported, the core would close its gate and a request
/// started while detached (R1) would not reach the hardware
void TestHsNoSuspendWhileDetached()
{
  ResetFakes();
  HsBench b;
  b.Enumerate();
  b.usb.ForceReset();
  fake_usbhsd.MIS_ST = USBHS_UMS_SIE_FREE | USBHS_UMS_SUSPEND;
  LatchHs(USBHS_UIF_SUSPEND, 0U);
  RunHs();
  fake_usbhsd.UEP1_TX_LEN = 0U;
  b.cls.Send(10U);
  PendedHs();
  TEST_ASSERT(fake_usbhsd.UEP1_TX_LEN == 10U);
}

/// 全速同上：断开期间不报告挂起，断开期间启动的请求照常上硬件
/// Full speed, the same: no suspend reported while detached, a request started while
/// detached reaches the hardware as usual
void TestFsNoSuspendWhileDetached()
{
  ResetFakes();
  FsBench b;
  b.Enumerate();
  b.usb.ForceReset();
  fake_usbfsd.MIS_ST = USBFS_UMS_SIE_FREE | USBFS_UMS_SUSPEND;
  LatchFs(USBFS_UIF_SUSPEND, 0U);
  RunFs();
  fake_usbfsd.UEP2_TX_LEN = 0U;
  b.single.Send(64U);
  PendedFs();
  TEST_ASSERT(fake_usbfsd.UEP2_TX_LEN == 64U);
}

/// CH32 OTG FS/HS 构造时不写 UEP* 寄存器（避免开时钟前 BusFault），Start 时才初始化，
/// 且 HaltController 清掉 suspended_ 与 HS 帧号缓存 / Ctor writes no UEP* regs before
/// clock enable; Start initializes them; HaltController clears suspended_ &
/// last_frame_no_
void TestOtgInitAndHaltState()
{
  // HS
  {
    ResetFakes();
    fake_usbhsd.UEP0_DMA = 0xDEADBEEFU;
    fake_usbhsd.UEP1_TX_DMA = 0xDEADBEEFU;
    fake_usbhsd.UEP1_RX_DMA = 0xDEADBEEFU;
    fake_usbhsd.UEP0_TX_CTRL = 0xAAU;
    fake_usbhsd.UEP0_RX_CTRL = 0xAAU;

    Block<64> ep0;
    Block<2048> ep1_out;
    Block<4096> ep1_in;
    PairClass cls(EPNumber::EP1, EPNumber::EP1, Type::BULK, 512U);
    ResettableUsb<CH32USBOtgHS> usb({ep0.Raw(), ep1_out.Raw()}, {ep0.Raw(), ep1_in.Raw()},
                                    0x1209U, 0x0001U, 0x0100U, {&LANGUAGE}, {{&cls}});
    TEST_ASSERT(fake_usbhsd.UEP0_DMA == 0xDEADBEEFU);
    TEST_ASSERT(fake_usbhsd.UEP1_TX_DMA == 0xDEADBEEFU);
    TEST_ASSERT(fake_usbhsd.UEP1_RX_DMA == 0xDEADBEEFU);
    TEST_ASSERT(fake_usbhsd.UEP0_TX_CTRL == 0xAAU);
    TEST_ASSERT(fake_usbhsd.UEP0_RX_CTRL == 0xAAU);

    usb.Init(false);
    usb.Start(false);
    PendedHs();
    TEST_ASSERT(fake_usbhsd.UEP0_DMA == Addr(ep0.bytes));
    TEST_ASSERT(fake_usbhsd.UEP1_TX_DMA == Addr(ep1_in.bytes));
    TEST_ASSERT(fake_usbhsd.UEP1_RX_DMA == Addr(ep1_out.bytes));
    TEST_ASSERT(fake_usbhsd.UEP0_TX_CTRL == USBHS_UEP_T_RES_NAK);
    TEST_ASSERT(fake_usbhsd.UEP0_RX_CTRL == USBHS_UEP_R_RES_ACK);

    // 挂起后 Stop 再 Start，帧号清零、再次挂起与 SOF 都能上报 / Stop after suspend,
    // restart: suspended_ and last_frame_no_ reset cleanly
    fake_usbhsd.MIS_ST = USBHS_UMS_SIE_FREE | USBHS_UMS_SUSPEND;
    RunHs();
    usb.Stop(false);
    PendedHs();
    fake_usbhsd.MIS_ST = USBHS_UMS_SIE_FREE;
    fake_usbhsd.FRAME_NO = 0x0123U;
    usb.Start(false);
    PendedHs();
    TickAnd(6, PendedHs);
    cls.frames.clear();
    fake_usbhsd.FRAME_NO = 0x0123U;
    fake_usbhsd.SPEED_TYPE = USBHS_USB_SPEED_HIGH;
    LatchHs(USBHS_UIF_BUS_RST, 0U);
    RunHs();
    SetupPacket set_cfg{0x00U, 9U, 1U, 0U, 0U};
    std::memcpy(ep0.bytes, &set_cfg, sizeof(set_cfg));
    LatchHs(USBHS_UIF_SETUP_ACT, 0U, 0U);
    RunHs();
    fake_usbhsd.FRAME_NO = 0x0124U;
    RunHs();
    TEST_ASSERT(!cls.frames.empty() && cls.frames.back() == 0x0124U);
  }

  // FS
  {
    ResetFakes();
    fake_usbfsd.UEP0_DMA = 0xDEADBEEFU;
    fake_usbfsd.UEP1_DMA = 0xDEADBEEFU;
    fake_usbfsd.UEP0_TX_CTRL = 0xAAU;
    fake_usbfsd.UEP0_RX_CTRL = 0xAAU;

    Block<64> ep0;
    Block<256> ep1_out;
    Block<256> ep1_in;
    PairClass cls(EPNumber::EP1, EPNumber::EP2, Type::BULK, 64U);
    ResettableUsb<CH32USBOtgFS> usb({ep0.Raw(), ep1_out.Raw(), {nullptr, 0}},
                                    {ep0.Raw(), {nullptr, 0}, ep1_in.Raw()},
                                    USB::DeviceDescriptor::PacketSize0::SIZE_64, 0x1209U,
                                    0x0001U, 0x0100U, {&LANGUAGE}, {{&cls}});
    TEST_ASSERT(fake_usbfsd.UEP0_DMA == 0xDEADBEEFU);
    TEST_ASSERT(fake_usbfsd.UEP1_DMA == 0xDEADBEEFU);
    TEST_ASSERT(fake_usbfsd.UEP0_TX_CTRL == 0xAAU);
    TEST_ASSERT(fake_usbfsd.UEP0_RX_CTRL == 0xAAU);

    usb.Init(false);
    usb.Start(false);
    PendedFs();
    TEST_ASSERT(fake_usbfsd.UEP0_DMA == Addr(ep0.bytes));
    TEST_ASSERT(fake_usbfsd.UEP1_DMA == Addr(ep1_out.bytes));
    TEST_ASSERT(fake_usbfsd.UEP2_DMA == Addr(ep1_in.bytes));
    TEST_ASSERT(fake_usbfsd.UEP0_TX_CTRL == USBFS_UEP_T_RES_NAK);
    TEST_ASSERT(fake_usbfsd.UEP0_RX_CTRL == USBFS_UEP_R_RES_ACK);
  }
}

void TestFsResetAsync()
{
  ResetFakes();
  FsBench b;
  CheckResetAsync(b, fake_usbfsd.BASE_CTRL, USBFS_UC_DEV_PU_EN, fake_usbfsd.INT_EN,
                  PendedFs);
}

void TestFsDetachRequests()
{
  ResetFakes();
  FsBench b;
  CheckDetachRequests(b, fake_usbfsd.BASE_CTRL, USBFS_UC_DEV_PU_EN, PendedFs);
}

/// 没有登记节拍源时构造后端是致命错误（必须在登记之前运行）/ Constructing a backend
/// without a registered tick source is fatal (must run before the registration)
void CheckNoTickSourceFatal()
{
  TEST_ASSERT(!DriverAsyncCondition::HasTickSource());
  std::fflush(nullptr);
  const pid_t child = fork();
  TEST_ASSERT(child >= 0);
  constexpr int FATAL_EXIT = 77;
  if (child == 0)
  {
    auto cb = LibXR::Assert::FatalCallback::Create(
        [](bool, int code, const char*, uint32_t) { _exit(code); }, FATAL_EXIT);
    LibXR::Assert::RegisterFatalErrorCallback(cb);
    ResetFakes();
    FsBench b;
    _exit(0);
  }
  int status = 0;
  TEST_ASSERT(waitpid(child, &status, 0) == child);
  TEST_ASSERT(WIFEXITED(status) && WEXITSTATUS(status) == FATAL_EXIT);
}

struct Case
{
  const char* name;
  void (*run)();
};

}  // namespace

int main(int argc, char** argv)
{
  const Case cases[] = {
      {"hs_enumerate", TestHsEnumerate},
      {"hs_second_reset_on_speed", TestHsSecondResetOnSpeed},
      {"hs_out_stop_collects", TestHsOutStopCollects},
      {"hs_stop_leaves_other_event", TestHsStopLeavesOtherEvent},
      {"hs_in_stop", TestHsInStop},
      {"hs_nested_stop_ignores_held_event", TestHsNestedStopIgnoresHeldEvent},
      {"hs_in_toggle_repair", TestHsInToggleRepair},
      {"hs_ep0_before_setup", TestHsEp0BeforeSetup},
      {"hs_pending_stop", TestHsPendingStop},
      {"hs_stop_by_time", TestHsStopByTime},
      {"hs_frame_report", TestHsFrameReport},
      {"hs_reset_while_suspended", TestHsResetWhileSuspended},
      {"hs_ep0_stop_at_setup", TestHsEp0StopAtSetup},
      {"hs_device_stop", TestHsDeviceStop},
      {"hs_limits_sof", TestHsLimits},
      {"hs_iso_in", TestHsIsoIn},
      {"hs_out_is_nak", TestHsOutIsNak},
      {"hs_ep0_status_with_is_nak", TestHsEp0StatusWithIsNak},
      {"fs_enumerate_limits", TestFsEnumerateLimits},
      {"fs_out_stop", TestFsOutStop},
      {"fs_in_stop", TestFsInStop},
      {"fs_shared_out", TestFsSharedOut},
      {"fs_pending_stop", TestFsPendingStop},
      {"fs_stop_at_sof", TestFsStopAtSof},
      {"fs_software_toggle", TestFsSoftwareToggle},
      {"fs_iso_merged_event", TestFsIsoMergedEvent},
      {"fs_device_stop", TestFsDeviceStop},
      {"fs_ep0_stop_at_setup", TestFsEp0StopAtSetup},
      {"hs_clear_halt_keeps_requests", TestHsClearHaltKeepsRequests},
      {"fs_clear_halt_keeps_requests", TestFsClearHaltKeepsRequests},
      {"hs_reset_async", TestHsResetAsync},
      {"hs_detach_requests", TestHsDetachRequests},
      {"hs_no_suspend_while_detached", TestHsNoSuspendWhileDetached},
      {"fs_reset_async", TestFsResetAsync},
      {"fs_detach_requests", TestFsDetachRequests},
      {"fs_no_suspend_while_detached", TestFsNoSuspendWhileDetached},
      {"otg_init_and_halt_state", TestOtgInitAndHaltState},
  };
  // 登记之前：没有节拍源时构造是致命错误。之后测试扮演唯一的节拍源。
  // Before the registration: construction without a tick source is fatal. Afterwards
  // the test plays the only tick source.
  if (argc <= 1 || std::strstr("no_tick_source_fatal", argv[1]) != nullptr)
  {
    CheckNoTickSourceFatal();
    std::printf("ok no_tick_source_fatal\n");
  }
  DriverAsyncCondition::RegisterTickSource();
  for (const Case& c : cases)
  {
    // 可选参数：只跑名字含这段文字的用例 / Optional argument: run only the cases whose
    // name contains this text
    if (argc > 1 && std::strstr(c.name, argv[1]) == nullptr)
    {
      continue;
    }
    c.run();
    std::printf("ok %s\n", c.name);
  }
  return 0;
}
