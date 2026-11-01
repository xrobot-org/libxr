#include "linux_can_error.hpp"

#include <cstdio>

#include "test_assert.hpp"

int main()
{
  using ErrorID = LibXR::CAN::ErrorID;
  struct ProtocolCase
  {
    uint8_t bits;
    ErrorID expected;
  };
  const ProtocolCase cases[] = {
      {CAN_ERR_PROT_STUFF, ErrorID::CAN_ERROR_ID_STUFF},
      {CAN_ERR_PROT_FORM, ErrorID::CAN_ERROR_ID_FORM},
      {CAN_ERR_PROT_BIT0, ErrorID::CAN_ERROR_ID_BIT0},
      {CAN_ERR_PROT_BIT1, ErrorID::CAN_ERROR_ID_BIT1},
      {CAN_ERR_PROT_BIT, ErrorID::CAN_ERROR_ID_OTHER},
      {CAN_ERR_PROT_OVERLOAD, ErrorID::CAN_ERROR_ID_OTHER},
      {0, ErrorID::CAN_ERROR_ID_PROTOCOL},
      {CAN_ERR_PROT_STUFF | CAN_ERR_PROT_FORM, ErrorID::CAN_ERROR_ID_STUFF},
  };
  unsigned checks = 0;
  for (const auto& test : cases)
  {
    canfd_frame frame{};
    // 控制器状态与协议字段故意冲突，防止回归到读取 data[1]。
    // Conflicting controller/protocol fields catch accidental reads from data[1].
    frame.data[1] =
        test.bits == CAN_ERR_PROT_STUFF ? CAN_ERR_PROT_FORM : CAN_ERR_PROT_STUFF;
    frame.data[2] = test.bits;
    TEST_ASSERT(LibXR::LinuxCANDetail::ProtocolErrorID(frame, 3) == test.expected);
    ++checks;
  }
  for (uint8_t length = 0; length <= 3; ++length)
  {
    canfd_frame frame{};
    frame.data[1] = CAN_ERR_PROT_FORM;
    frame.data[2] = CAN_ERR_PROT_STUFF;
    const auto expected =
        length < 3 ? ErrorID::CAN_ERROR_ID_PROTOCOL : ErrorID::CAN_ERROR_ID_STUFF;
    TEST_ASSERT(LibXR::LinuxCANDetail::ProtocolErrorID(frame, length) == expected);
    ++checks;
  }

  struct LocationCase
  {
    uint8_t location;
    ErrorID expected;
  };
  const LocationCase locations[] = {
      {CAN_ERR_PROT_LOC_ACK, ErrorID::CAN_ERROR_ID_ACK},
      {CAN_ERR_PROT_LOC_CRC_SEQ, ErrorID::CAN_ERROR_ID_CRC},
  };
  for (const auto& test : locations)
  {
    for (uint8_t length = 0; length <= CAN_MAX_DLEN; ++length)
    {
      canfd_frame frame{};
      frame.data[2] = CAN_ERR_PROT_BIT | CAN_ERR_PROT_STUFF;
      frame.data[3] = test.location;
      const auto expected = length >= 4   ? test.expected
                            : length >= 3 ? ErrorID::CAN_ERROR_ID_STUFF
                                          : ErrorID::CAN_ERROR_ID_PROTOCOL;
      TEST_ASSERT(LibXR::LinuxCANDetail::ProtocolErrorID(frame, length) == expected);
      ++checks;
    }
  }

  struct FrameCase
  {
    const char* name;
    canid_t classes;
    uint8_t controller;
    uint8_t protocol;
    uint8_t location;
    uint8_t length;
    ErrorID expected;
  };
  const FrameCase frames[] = {
      {"bus off takes priority",
       CAN_ERR_BUSOFF | CAN_ERR_CRTL | CAN_ERR_ACK | CAN_ERR_PROT,
       CAN_ERR_CRTL_RX_PASSIVE | CAN_ERR_CRTL_TX_WARNING, CAN_ERR_PROT_STUFF,
       CAN_ERR_PROT_LOC_CRC_SEQ, 8, ErrorID::CAN_ERROR_ID_BUS_OFF},
      {"bus off needs no payload", CAN_ERR_BUSOFF, 0, 0, 0, 0,
       ErrorID::CAN_ERROR_ID_BUS_OFF},
      {"RX passive before warning and ACK", CAN_ERR_CRTL | CAN_ERR_ACK | CAN_ERR_PROT,
       CAN_ERR_CRTL_RX_PASSIVE | CAN_ERR_CRTL_TX_WARNING, CAN_ERR_PROT_FORM,
       CAN_ERR_PROT_LOC_ACK, 8, ErrorID::CAN_ERROR_ID_ERROR_PASSIVE},
      {"TX passive", CAN_ERR_CRTL, CAN_ERR_CRTL_TX_PASSIVE, 0, 0, 2,
       ErrorID::CAN_ERROR_ID_ERROR_PASSIVE},
      {"RX warning before protocol ACK", CAN_ERR_CRTL | CAN_ERR_PROT,
       CAN_ERR_CRTL_RX_WARNING, 0, CAN_ERR_PROT_LOC_ACK, 8,
       ErrorID::CAN_ERROR_ID_ERROR_WARNING},
      {"TX warning before CRC", CAN_ERR_CRTL | CAN_ERR_PROT, CAN_ERR_CRTL_TX_WARNING,
       CAN_ERR_PROT_BIT, CAN_ERR_PROT_LOC_CRC_SEQ, 8,
       ErrorID::CAN_ERROR_ID_ERROR_WARNING},
      {"short controller field", CAN_ERR_CRTL, CAN_ERR_CRTL_RX_PASSIVE, 0, 0, 1,
       ErrorID::CAN_ERROR_ID_GENERIC},
      {"controller requires class", CAN_ERR_PROT, CAN_ERR_CRTL_RX_PASSIVE,
       CAN_ERR_PROT_FORM, 0, 8, ErrorID::CAN_ERROR_ID_FORM},
      {"ACK flag needs no payload", CAN_ERR_ACK, 0, 0, 0, 0, ErrorID::CAN_ERROR_ID_ACK},
      {"flexcan ACK", CAN_ERR_ACK | CAN_ERR_PROT, 0, 0, CAN_ERR_PROT_LOC_ACK, 8,
       ErrorID::CAN_ERROR_ID_ACK},
      {"m_can ACK", CAN_ERR_PROT, 0, 0, CAN_ERR_PROT_LOC_ACK, 8,
       ErrorID::CAN_ERROR_ID_ACK},
      {"ACK flag before CRC", CAN_ERR_ACK | CAN_ERR_PROT, 0, CAN_ERR_PROT_BIT,
       CAN_ERR_PROT_LOC_CRC_SEQ, 8, ErrorID::CAN_ERROR_ID_ACK},
      {"ACK location before stuff", CAN_ERR_PROT, 0, CAN_ERR_PROT_STUFF,
       CAN_ERR_PROT_LOC_ACK, 8, ErrorID::CAN_ERROR_ID_ACK},
      {"m_can CRC", CAN_ERR_PROT, 0, 0, CAN_ERR_PROT_LOC_CRC_SEQ, 8,
       ErrorID::CAN_ERROR_ID_CRC},
      {"flexcan CRC", CAN_ERR_PROT, 0, CAN_ERR_PROT_BIT, CAN_ERR_PROT_LOC_CRC_SEQ, 8,
       ErrorID::CAN_ERROR_ID_CRC},
      {"CRC before form", CAN_ERR_PROT, 0, CAN_ERR_PROT_FORM, CAN_ERR_PROT_LOC_CRC_SEQ, 8,
       ErrorID::CAN_ERROR_ID_CRC},
      {"ACK location requires protocol class", CAN_ERR_TRX, 0, CAN_ERR_PROT_STUFF,
       CAN_ERR_PROT_LOC_ACK, 8, ErrorID::CAN_ERROR_ID_OTHER},
      {"CRC location requires protocol class", 0, 0, CAN_ERR_PROT_BIT,
       CAN_ERR_PROT_LOC_CRC_SEQ, 8, ErrorID::CAN_ERROR_ID_GENERIC},
      {"short ACK location", CAN_ERR_PROT, 0, CAN_ERR_PROT_FORM, CAN_ERR_PROT_LOC_ACK, 3,
       ErrorID::CAN_ERROR_ID_FORM},
      {"short CRC location", CAN_ERR_PROT, 0, 0, CAN_ERR_PROT_LOC_CRC_SEQ, 3,
       ErrorID::CAN_ERROR_ID_PROTOCOL},
      {"ACK delimiter is not ACK", CAN_ERR_PROT, 0, CAN_ERR_PROT_FORM,
       CAN_ERR_PROT_LOC_ACK_DEL, 8, ErrorID::CAN_ERROR_ID_FORM},
      {"CRC delimiter is not CRC sequence", CAN_ERR_PROT, 0, CAN_ERR_PROT_FORM,
       CAN_ERR_PROT_LOC_CRC_DEL, 8, ErrorID::CAN_ERROR_ID_FORM},
      {"stuff before form", CAN_ERR_PROT, 0, CAN_ERR_PROT_STUFF | CAN_ERR_PROT_FORM, 0, 8,
       ErrorID::CAN_ERROR_ID_STUFF},
      {"form before bit1", CAN_ERR_PROT, 0, CAN_ERR_PROT_FORM | CAN_ERR_PROT_BIT1, 0, 8,
       ErrorID::CAN_ERROR_ID_FORM},
      {"bit1 before bit0", CAN_ERR_PROT, 0, CAN_ERR_PROT_BIT1 | CAN_ERR_PROT_BIT0, 0, 8,
       ErrorID::CAN_ERROR_ID_BIT1},
      {"bit0 before unspecified bit error", CAN_ERR_PROT, 0,
       CAN_ERR_PROT_BIT0 | CAN_ERR_PROT_BIT, 0, 8, ErrorID::CAN_ERROR_ID_BIT0},
      {"protocol before restarted", CAN_ERR_PROT | CAN_ERR_RESTARTED, 0, 0, 0, 8,
       ErrorID::CAN_ERROR_ID_PROTOCOL},
      {"protocol before transceiver", CAN_ERR_PROT | CAN_ERR_TRX, 0, 0, 0, 8,
       ErrorID::CAN_ERROR_ID_PROTOCOL},
      {"unspecified bit error", CAN_ERR_PROT, 0, CAN_ERR_PROT_BIT, 0, 8,
       ErrorID::CAN_ERROR_ID_OTHER},
      {"overload", CAN_ERR_PROT, 0, CAN_ERR_PROT_OVERLOAD, 0, 8,
       ErrorID::CAN_ERROR_ID_OTHER},
      {"short protocol field", CAN_ERR_PROT, 0, CAN_ERR_PROT_STUFF, 0, 2,
       ErrorID::CAN_ERROR_ID_PROTOCOL},
      {"restart", CAN_ERR_RESTARTED, 0, 0, 0, 8, ErrorID::CAN_ERROR_ID_OTHER},
      {"transceiver", CAN_ERR_TRX, 0, 0, 0, 8, ErrorID::CAN_ERROR_ID_OTHER},
      {"unmapped class", CAN_ERR_LOSTARB, 0, 0, 0, 8, ErrorID::CAN_ERROR_ID_GENERIC},
      {"no class", 0, 0, 0, 0, 0, ErrorID::CAN_ERROR_ID_GENERIC},
  };
  for (const auto& test : frames)
  {
    canfd_frame frame{};
    frame.can_id = CAN_ERR_FLAG | test.classes;
    frame.data[1] = test.controller;
    frame.data[2] = test.protocol;
    frame.data[3] = test.location;
    const auto actual = LibXR::LinuxCANDetail::FrameErrorID(frame, test.length);
    if (actual != test.expected)
    {
      std::fprintf(stderr, "%s: expected %u, got %u\n", test.name,
                   static_cast<unsigned>(test.expected), static_cast<unsigned>(actual));
    }
    TEST_ASSERT(actual == test.expected);
    ++checks;
  }
  std::printf("linux_can_error: %u checks passed\n", checks);
  return 0;
}
