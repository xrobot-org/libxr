// NOLINTBEGIN(cppcoreguidelines-pro-type-cstyle-cast,performance-no-int-to-ptr)
#include "ch32_can_def.hpp"

ch32_can_id_t CH32_CAN_GetID(CAN_TypeDef* addr)
{
  // NOLINTBEGIN
  if (addr == nullptr)
  {
    return CH32_CAN_ID_ERROR;
  }
#if defined(CAN1)
  else if (addr == CAN1)
  {
    return CH32_CAN1;
  }
#endif
#if LIBXR_CH32_HAS_CAN2
  else if (addr == CAN2)
  {
    return CH32_CAN2;
  }
#endif
  return CH32_CAN_ID_ERROR;
  // NOLINTEND
}

CAN_TypeDef* CH32_CAN_GetInstanceID(ch32_can_id_t id)
{
  switch (id)
  {
#if defined(CAN1)
    case CH32_CAN1:
      return CAN1;
#endif
#if LIBXR_CH32_HAS_CAN2
    case CH32_CAN2:
      return CAN2;
#endif
    default:
      return nullptr;
  }
}

// NOLINTEND(cppcoreguidelines-pro-type-cstyle-cast,performance-no-int-to-ptr)
