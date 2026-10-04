// NOLINTBEGIN(cppcoreguidelines-pro-type-cstyle-cast,performance-no-int-to-ptr)
// ch32_usb_endpoint_dev_fs.cpp
#include <cstdint>
#include <cstring>

#include "ch32_usb_endpoint.hpp"
#include "ch32_usbcan_shared.hpp"
#include "ep.hpp"

using namespace LibXR;

#if defined(RCC_APB1Periph_USB)

#ifdef USB_BASE
static constexpr uintptr_t REG_BASE = USB_BASE;
#else
static constexpr uintptr_t REG_BASE = 0x40005C00UL;
#endif

#ifdef PMAAddr
static constexpr uintptr_t PMA_BASE = PMAAddr;
#else
static constexpr uintptr_t PMA_BASE = 0x40006000UL;
#endif

static inline uintptr_t pma_phys(uint16_t pma_off_bytes)
{
  return PMA_BASE + (static_cast<uintptr_t>(pma_off_bytes) << 1);
}

static inline volatile uint16_t* btable()
{
  return reinterpret_cast<volatile uint16_t*>(REG_BASE + 0x50U);
}

static inline volatile uint16_t* ep_reg(uint8_t ep)
{
  return reinterpret_cast<volatile uint16_t*>(REG_BASE +
                                              (static_cast<uintptr_t>(ep) * 4U));
}

#if !defined(USB_EP_CTR_RX) && defined(EP_CTR_RX)
#define USB_EP_CTR_RX EP_CTR_RX
#endif
#if !defined(USB_EP_DTOG_RX) && defined(EP_DTOG_RX)
#define USB_EP_DTOG_RX EP_DTOG_RX
#endif
#if !defined(USB_EPRX_STAT) && defined(EPRX_STAT)
#define USB_EPRX_STAT EPRX_STAT
#endif
#if !defined(USB_EP_SETUP) && defined(EP_SETUP)
#define USB_EP_SETUP EP_SETUP
#endif
#if !defined(USB_EP_T_FIELD) && defined(EP_T_FIELD)
#define USB_EP_T_FIELD EP_T_FIELD
#endif
#if !defined(USB_EP_KIND) && defined(EP_KIND)
#define USB_EP_KIND EP_KIND
#endif
#if !defined(USB_EP_CTR_TX) && defined(EP_CTR_TX)
#define USB_EP_CTR_TX EP_CTR_TX
#endif
#if !defined(USB_EP_DTOG_TX) && defined(EP_DTOG_TX)
#define USB_EP_DTOG_TX EP_DTOG_TX
#endif
#if !defined(USB_EPTX_STAT) && defined(EPTX_STAT)
#define USB_EPTX_STAT EPTX_STAT
#endif
#if !defined(USB_EPADDR_FIELD) && defined(EPADDR_FIELD)
#define USB_EPADDR_FIELD EPADDR_FIELD
#endif

#ifndef USB_EP_CTR_RX
#define USB_EP_CTR_RX ((uint16_t)0x8000U)
#endif
#ifndef USB_EP_DTOG_RX
#define USB_EP_DTOG_RX ((uint16_t)0x4000U)
#endif
#ifndef USB_EPRX_STAT
#define USB_EPRX_STAT ((uint16_t)0x3000U)
#endif
#ifndef USB_EP_SETUP
#define USB_EP_SETUP ((uint16_t)0x0800U)
#endif
#ifndef USB_EP_T_FIELD
#define USB_EP_T_FIELD ((uint16_t)0x0600U)
#endif
#ifndef USB_EP_KIND
#define USB_EP_KIND ((uint16_t)0x0100U)
#endif
#ifndef USB_EP_CTR_TX
#define USB_EP_CTR_TX ((uint16_t)0x0080U)
#endif
#ifndef USB_EP_DTOG_TX
#define USB_EP_DTOG_TX ((uint16_t)0x0040U)
#endif
#ifndef USB_EPTX_STAT
#define USB_EPTX_STAT ((uint16_t)0x0030U)
#endif
#ifndef USB_EPADDR_FIELD
#define USB_EPADDR_FIELD ((uint16_t)0x000FU)
#endif

#ifndef USB_EP_CONTROL
#define USB_EP_CONTROL ((uint16_t)0x0200U)
#endif
#ifndef USB_EP_BULK
#define USB_EP_BULK ((uint16_t)0x0000U)
#endif
#ifndef USB_EP_INTERRUPT
#define USB_EP_INTERRUPT ((uint16_t)0x0600U)
#endif
#ifndef USB_EP_ISOCHRONOUS
#define USB_EP_ISOCHRONOUS ((uint16_t)0x0400U)
#endif

#ifndef USB_EP_TX_DIS
#define USB_EP_TX_DIS ((uint16_t)0x0000U)
#endif
#ifndef USB_EP_TX_STALL
#define USB_EP_TX_STALL ((uint16_t)0x0010U)
#endif
#ifndef USB_EP_TX_NAK
#define USB_EP_TX_NAK ((uint16_t)0x0020U)
#endif
#ifndef USB_EP_TX_VALID
#define USB_EP_TX_VALID ((uint16_t)0x0030U)
#endif

#ifndef USB_EP_RX_DIS
#define USB_EP_RX_DIS ((uint16_t)0x0000U)
#endif
#ifndef USB_EP_RX_STALL
#define USB_EP_RX_STALL ((uint16_t)0x1000U)
#endif
#ifndef USB_EP_RX_NAK
#define USB_EP_RX_NAK ((uint16_t)0x2000U)
#endif
#ifndef USB_EP_RX_VALID
#define USB_EP_RX_VALID ((uint16_t)0x3000U)
#endif

#ifndef USB_EPREG_MASK
#define USB_EPREG_MASK                                                           \
  (USB_EP_CTR_RX | USB_EP_SETUP | USB_EP_T_FIELD | USB_EP_KIND | USB_EP_CTR_TX | \
   USB_EPADDR_FIELD)
#endif

#ifndef USB_EPTX_DTOGMASK
#define USB_EPTX_DTOGMASK (USB_EPTX_STAT | USB_EPREG_MASK)
#endif
#ifndef USB_EPRX_DTOGMASK
#define USB_EPRX_DTOGMASK (USB_EPRX_STAT | USB_EPREG_MASK)
#endif

struct BTableEntry
{
  uint16_t addr_tx;
  uint16_t _r0;
  uint16_t count_tx;
  uint16_t _r1;
  uint16_t addr_rx;
  uint16_t _r2;
  uint16_t count_rx;
  uint16_t _r3;
};

static inline volatile BTableEntry* btable_entries()
{
  const uint16_t BTABLE_OFF = static_cast<uint16_t>((*btable()) & 0xFFF8U);
  return reinterpret_cast<volatile BTableEntry*>(pma_phys(BTABLE_OFF));
}

static inline void pma_write(uint16_t pma_offset, const void* src, size_t len)
{
  const uint8_t* s = reinterpret_cast<const uint8_t*>(src);
  volatile uint16_t* p = reinterpret_cast<volatile uint16_t*>(pma_phys(pma_offset));

  for (size_t i = 0; i < len; i += 2)
  {
    uint16_t w = s[i];
    if (i + 1 < len)
    {
      w |= static_cast<uint16_t>(s[i + 1]) << 8;
    }
    *p = w;
    p += 2;  // stride=2 on CH32 FSDEV PMA
  }
}

static inline void pma_read(void* dst, uint16_t pma_offset, size_t len)
{
  uint8_t* d = reinterpret_cast<uint8_t*>(dst);
  const volatile uint16_t* p =
      reinterpret_cast<const volatile uint16_t*>(pma_phys(pma_offset));

  for (size_t i = 0; i < len; i += 2)
  {
    const uint16_t W = *p;
    p += 2;

    d[i] = static_cast<uint8_t>(W & 0xFFU);
    if (i + 1 < len)
    {
      d[i + 1] = static_cast<uint8_t>((W >> 8) & 0xFFU);
    }
  }
}

static inline void set_tx_status(uint8_t ep, uint16_t desired_stat)
{
  const uint16_t CUR = *ep_reg(ep);
  uint16_t reg = static_cast<uint16_t>(CUR & USB_EPREG_MASK);

  // FSDEV EPxR STAT bits are write-1-to-toggle; write the delta to reach target.
  const uint16_t TARGET = static_cast<uint16_t>(desired_stat & USB_EPTX_STAT);
  reg |= static_cast<uint16_t>((CUR ^ TARGET) & USB_EPTX_STAT);
  reg |= static_cast<uint16_t>(USB_EP_CTR_RX | USB_EP_CTR_TX);
  *ep_reg(ep) = reg;
}

static inline void set_rx_status(uint8_t ep, uint16_t desired_stat)
{
  const uint16_t CUR = *ep_reg(ep);
  uint16_t reg = static_cast<uint16_t>(CUR & USB_EPREG_MASK);

  // FSDEV EPxR STAT bits are write-1-to-toggle; write the delta to reach target.
  const uint16_t TARGET = static_cast<uint16_t>(desired_stat & USB_EPRX_STAT);
  reg |= static_cast<uint16_t>((CUR ^ TARGET) & USB_EPRX_STAT);
  reg |= static_cast<uint16_t>(USB_EP_CTR_RX | USB_EP_CTR_TX);
  *ep_reg(ep) = reg;
}

static inline void clear_ctr_tx(uint8_t ep)
{
  uint16_t reg = static_cast<uint16_t>((*ep_reg(ep)) & USB_EPREG_MASK);
  reg |= static_cast<uint16_t>(USB_EP_CTR_TX | USB_EP_CTR_RX);
  reg = static_cast<uint16_t>(reg & ~USB_EP_CTR_TX);
  *ep_reg(ep) = reg;
}

static inline void clear_ctr_rx(uint8_t ep)
{
  uint16_t reg = static_cast<uint16_t>((*ep_reg(ep)) & USB_EPREG_MASK);
  reg |= static_cast<uint16_t>(USB_EP_CTR_TX | USB_EP_CTR_RX);
  reg = static_cast<uint16_t>(reg & ~USB_EP_CTR_RX);
  *ep_reg(ep) = reg;
}

static inline uint16_t get_rx_count_from_btable(uint8_t ep)
{
  return static_cast<uint16_t>(btable_entries()[ep].count_rx & 0x03FFU);
}

static inline uint16_t encode_rx_count(uint16_t mps)
{
  if (mps <= 62)
  {
    const uint16_t BLOCKS = static_cast<uint16_t>((mps + 1U) / 2U);
    return static_cast<uint16_t>(BLOCKS << 10);
  }

  const uint16_t BLOCKS = static_cast<uint16_t>((mps + 31U) / 32U);
  return static_cast<uint16_t>(0x8000U | ((BLOCKS - 1U) << 10));
}

static constexpr uint16_t PMA_ALLOC_BASE =
    static_cast<uint16_t>(CH32EndpointDevFs::EP_DEV_FS_MAX_SIZE) * 8U;
static uint16_t g_pma_next = PMA_ALLOC_BASE;
static uint16_t g_pma_limit = LibXR::CH32UsbCanShared::USBD_PMA_BYTES_SOLO;

void CH32EndpointDevFs::ResetPMAAllocator()
{
  const uint16_t LIMIT_BYTES = static_cast<uint16_t>(
      LibXR::CH32UsbCanShared::usb_pma_limit_bytes() & static_cast<uint16_t>(~1U));
  g_pma_limit = LIMIT_BYTES;

  g_pma_next = PMA_ALLOC_BASE;
  g_pma_next = static_cast<uint16_t>((g_pma_next + 1U) & ~1U);
  REQUIRE(g_pma_next <= g_pma_limit);
}

static inline uint16_t alloc_pma(size_t bytes)
{
  // 首次 USB reset 前也可能分配端点，必须立即使用当前 CAN 预留后的上限。
  // Init may allocate endpoints before the first USB bus reset.
  g_pma_limit = LibXR::CH32UsbCanShared::usb_pma_limit_bytes();
  // PMA 最大只有 512 字节；先拒绝超大缓冲，避免对齐长度收窄到 uint16_t 时回绕。
  // PMA is at most 512 bytes. Reject before narrowing the aligned size to uint16_t,
  // so an oversized endpoint buffer cannot wrap the allocation increment.
  REQUIRE(bytes <= g_pma_limit);
  if (bytes > g_pma_limit)
  {
    return 0;
  }

  const uint16_t ADDR = g_pma_next;
  const uint32_t INC = (static_cast<uint32_t>(bytes) + 1U) & ~1U;
  const uint32_t END = static_cast<uint32_t>(ADDR) + INC;
  REQUIRE(END <= g_pma_limit);
  if (END > g_pma_limit)
  {
    return 0;
  }

  LibXR::CH32UsbCanShared::usb_pma_configured.store(true, std::memory_order_release);
  g_pma_next = static_cast<uint16_t>(END);
  return ADDR;
}

CH32EndpointDevFs::CH32EndpointDevFs(EPNumber ep_num, Direction dir,
                                     LibXR::RawData buffer, bool is_isochronous)
    : Endpoint(ep_num, dir, buffer), is_isochronous_(is_isochronous)
{
  const uint8_t EP_I = static_cast<uint8_t>(EPNumberToInt8(GetNumber()));
  map_dev_fs_[EP_I][static_cast<uint8_t>(dir)] = this;
}

void CH32EndpointDevFs::Configure(const Config& cfg)
{
  ASSERT(cfg.direction == Direction::IN || cfg.direction == Direction::OUT);
  ASSERT(cfg.direction == GetDirection());

  auto& ep_cfg = GetConfig();
  ep_cfg = cfg;

  const uint8_t EP_I = static_cast<uint8_t>(EPNumberToInt8(GetNumber()));

  size_t packet_size_limit = 64;
  if (cfg.type == Type::ISOCHRONOUS)
  {
    packet_size_limit = 1023;
  }

  const auto BUF = GetBuffer();
  if (packet_size_limit > BUF.size_)
  {
    packet_size_limit = BUF.size_;
  }

  size_t max_packet_size = cfg.max_packet_size;
  if (max_packet_size > packet_size_limit)
  {
    max_packet_size = packet_size_limit;
  }
  if (max_packet_size < 8)
  {
    max_packet_size = 8;
  }
  if (max_packet_size > BUF.size_)
  {
    max_packet_size = BUF.size_;
  }

  ep_cfg.max_packet_size = static_cast<uint16_t>(max_packet_size);

  if (pma_addr_ == 0 || pma_addr_ < PMA_ALLOC_BASE)
  {
    const uint16_t ADDR = alloc_pma(BUF.size_);
    REQUIRE(ADDR >= PMA_ALLOC_BASE);
    if (ADDR < PMA_ALLOC_BASE)
    {
      return;
    }
    pma_addr_ = ADDR;
  }

  volatile BTableEntry* bt = btable_entries();

  if (GetDirection() == Direction::IN)
  {
    bt[EP_I].addr_tx = pma_addr_;
    bt[EP_I].count_tx = 0;
  }
  else
  {
    bt[EP_I].addr_rx = pma_addr_;
    bt[EP_I].count_rx = encode_rx_count(ep_cfg.max_packet_size);
  }

  uint16_t epr = static_cast<uint16_t>((*ep_reg(EP_I)) & USB_EPREG_MASK);

  epr = static_cast<uint16_t>((epr & ~USB_EPADDR_FIELD) | (EP_I & 0x0FU));
  epr = static_cast<uint16_t>(epr & ~USB_EP_T_FIELD);

  switch (ep_cfg.type)
  {
    case Type::CONTROL:
      epr |= USB_EP_CONTROL;
      break;
    case Type::BULK:
      epr |= USB_EP_BULK;
      break;
    case Type::INTERRUPT:
      epr |= USB_EP_INTERRUPT;
      break;
    case Type::ISOCHRONOUS:
      epr |= USB_EP_ISOCHRONOUS;
      break;
    default:
      epr |= USB_EP_BULK;
      break;
  }

  *ep_reg(EP_I) = epr;

  if (GetDirection() == Direction::IN)
  {
    set_tx_status(EP_I, USB_EP_TX_NAK);
  }
  else
  {
    set_rx_status(EP_I, USB_EP_RX_NAK);
  }

  SetState(State::IDLE);
}

void CH32EndpointDevFs::Close()
{
  const uint8_t EP_I = static_cast<uint8_t>(EPNumberToInt8(GetNumber()));
  if (GetDirection() == Direction::IN)
  {
    set_tx_status(EP_I, USB_EP_TX_DIS);
  }
  else
  {
    set_rx_status(EP_I, USB_EP_RX_DIS);
  }
}

ErrorCode CH32EndpointDevFs::Transfer(size_t size)
{
  if (size > GetBuffer().size_)
  {
    return ErrorCode::OUT_OF_RANGE;
  }

  SetState(State::BUSY);

  const uint8_t EP_I = static_cast<uint8_t>(EPNumberToInt8(GetNumber()));
  last_transfer_size_ = size;

  if (GetDirection() == Direction::IN)
  {
    auto buffer = GetBuffer();
    pma_write(pma_addr_, buffer.addr_, size);

    // Keep the current transfer on the old active block and switch to the next block
    // for producer writes, matching STM32/HAL timing.
    if (UseDoubleBuffer() && size > 0)
    {
      Endpoint::SwitchBuffer();
    }

    btable_entries()[EP_I].count_tx = static_cast<uint16_t>(size);
    set_tx_status(EP_I, USB_EP_TX_VALID);
  }
  else
  {
    volatile BTableEntry* bt = btable_entries();
    bt[EP_I].addr_rx = pma_addr_;
    bt[EP_I].count_rx = encode_rx_count(GetConfig().max_packet_size);
    set_rx_status(EP_I, USB_EP_RX_VALID);
  }

  return ErrorCode::OK;
}

void CH32EndpointDevFs::CopyRxDataToBuffer(size_t size)
{
  if (size > GetBuffer().size_)
  {
    size = GetBuffer().size_;
  }
  pma_read(GetBuffer().addr_, pma_addr_, size);
}

void CH32EndpointDevFs::TransferComplete(size_t size)
{
  const uint8_t EP_I = static_cast<uint8_t>(EPNumberToInt8(GetNumber()));

  if (GetDirection() == Direction::OUT)
  {
    const uint16_t RX_CNT = get_rx_count_from_btable(EP_I);
    size_t n = RX_CNT;
    if (n > GetBuffer().size_)
    {
      n = GetBuffer().size_;
    }
    pma_read(GetBuffer().addr_, pma_addr_, n);
    size = n;
    set_rx_status(EP_I, USB_EP_RX_NAK);
  }
  else
  {
    set_tx_status(EP_I, USB_EP_TX_NAK);
    size = last_transfer_size_;
  }

  OnTransferCompleteCallback(true, size);
}

ErrorCode CH32EndpointDevFs::Stall()
{
  const uint8_t EP_I = static_cast<uint8_t>(EPNumberToInt8(GetNumber()));
  if (GetDirection() == Direction::IN)
  {
    set_tx_status(EP_I, USB_EP_TX_STALL);
  }
  else
  {
    set_rx_status(EP_I, USB_EP_RX_STALL);
  }
  return ErrorCode::OK;
}

ErrorCode CH32EndpointDevFs::ClearStall()
{
  const uint8_t EP_I = static_cast<uint8_t>(EPNumberToInt8(GetNumber()));
  if (GetDirection() == Direction::IN)
  {
    set_tx_status(EP_I, USB_EP_TX_NAK);
  }
  else
  {
    set_rx_status(EP_I, USB_EP_RX_NAK);
  }
  return ErrorCode::OK;
}

void CH32EndpointDevFs::SwitchBuffer() { Endpoint::SwitchBuffer(); }

void CH32EndpointDevFs::SetEpTxStatus(uint8_t ep, uint16_t status)
{
  set_tx_status(ep, status);
}
void CH32EndpointDevFs::SetEpRxStatus(uint8_t ep, uint16_t status)
{
  set_rx_status(ep, status);
}
void CH32EndpointDevFs::ClearEpCtrTx(uint8_t ep) { clear_ctr_tx(ep); }
void CH32EndpointDevFs::ClearEpCtrRx(uint8_t ep) { clear_ctr_rx(ep); }
uint16_t CH32EndpointDevFs::GetRxCount(uint8_t ep)
{
  return get_rx_count_from_btable(ep);
}

#endif  // defined(RCC_APB1Periph_USB)

// NOLINTEND(cppcoreguidelines-pro-type-cstyle-cast,performance-no-int-to-ptr)
