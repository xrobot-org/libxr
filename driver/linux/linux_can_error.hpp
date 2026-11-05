#pragma once

#include <linux/can.h>
#include <linux/can/error.h>

#include <cstdint>

#include "can.hpp"

namespace LibXR::LinuxCANDetail
{
/// 内部协议错误解码，供驱动和无硬件回归使用 / Internal decoder shared with unit tests.
inline CAN::ErrorID ProtocolErrorID(const canfd_frame& frame, uint8_t length)
{
  const uint8_t location = length >= 4 ? frame.data[3] : CAN_ERR_PROT_LOC_UNSPEC;
  if (location == CAN_ERR_PROT_LOC_ACK)
  {
    return CAN::ErrorID::CAN_ERROR_ID_ACK;
  }
  if (location == CAN_ERR_PROT_LOC_CRC_SEQ)
  {
    return CAN::ErrorID::CAN_ERROR_ID_CRC;
  }
  const uint8_t prot_bits = length >= 3 ? frame.data[2] : 0;
  if ((prot_bits & CAN_ERR_PROT_STUFF) != 0)
  {
    return CAN::ErrorID::CAN_ERROR_ID_STUFF;
  }
  if ((prot_bits & CAN_ERR_PROT_FORM) != 0)
  {
    return CAN::ErrorID::CAN_ERROR_ID_FORM;
  }
  if ((prot_bits & CAN_ERR_PROT_BIT1) != 0)
  {
    return CAN::ErrorID::CAN_ERROR_ID_BIT1;
  }
  if ((prot_bits & CAN_ERR_PROT_BIT0) != 0)
  {
    return CAN::ErrorID::CAN_ERROR_ID_BIT0;
  }
  if ((prot_bits & (CAN_ERR_PROT_BIT | CAN_ERR_PROT_OVERLOAD)) != 0)
  {
    return CAN::ErrorID::CAN_ERROR_ID_OTHER;
  }
  return CAN::ErrorID::CAN_ERROR_ID_PROTOCOL;
}

/// 每帧只取最高优先级错误，状态优先于协议错误 / One error per frame, state first.
inline CAN::ErrorID FrameErrorID(const canfd_frame& frame, uint8_t length)
{
  if ((frame.can_id & CAN_ERR_BUSOFF) != 0)
  {
    return CAN::ErrorID::CAN_ERROR_ID_BUS_OFF;
  }
  const uint8_t controller = length >= 2 ? frame.data[1] : 0;
  if ((frame.can_id & CAN_ERR_CRTL) != 0)
  {
    if ((controller & (CAN_ERR_CRTL_RX_PASSIVE | CAN_ERR_CRTL_TX_PASSIVE)) != 0)
    {
      return CAN::ErrorID::CAN_ERROR_ID_ERROR_PASSIVE;
    }
    if ((controller & (CAN_ERR_CRTL_RX_WARNING | CAN_ERR_CRTL_TX_WARNING)) != 0)
    {
      return CAN::ErrorID::CAN_ERROR_ID_ERROR_WARNING;
    }
  }
  if ((frame.can_id & CAN_ERR_ACK) != 0)
  {
    return CAN::ErrorID::CAN_ERROR_ID_ACK;
  }
  if ((frame.can_id & CAN_ERR_PROT) != 0)
  {
    return ProtocolErrorID(frame, length);
  }
  if ((frame.can_id & (CAN_ERR_RESTARTED | CAN_ERR_TRX)) != 0)
  {
    return CAN::ErrorID::CAN_ERROR_ID_OTHER;
  }
  return CAN::ErrorID::CAN_ERROR_ID_GENERIC;
}
}  // namespace LibXR::LinuxCANDetail
