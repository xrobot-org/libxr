#include "can.hpp"

using namespace LibXR;

template class LibXR::Callback<const CAN::ClassicPack&>;
template class LibXR::Callback<const FDCAN::FDPack&>;

void CAN::Register(Callback cb, Type type, FilterMode mode, uint32_t start_id_mask,
                   uint32_t end_id_mask)
{
  ASSERT(type < Type::TYPE_NUM);

  auto node = new (std::align_val_t(LibXR::CONCURRENCY_ALIGNMENT))
      LockFreeList::Node<Filter>(Filter{mode, start_id_mask, end_id_mask, type, cb});
  subscriber_list_[static_cast<uint8_t>(type)].Add(*node);
}

void CAN::OnMessage(const ClassicPack& pack, bool in_isr)
{
  ASSERT(pack.type < Type::TYPE_NUM);
  subscriber_list_[static_cast<uint8_t>(pack.type)].Foreach<Filter>(
      [&](Filter& node)
      {
        switch (node.mode)
        {
          case FilterMode::ID_MASK:
            if ((pack.id & node.start_id_mask) == node.end_id_mask)
            {
              node.cb.Run(in_isr, pack);
            }
            break;
          case FilterMode::ID_RANGE:
            if (pack.id >= node.start_id_mask && pack.id <= node.end_id_mask)
            {
              node.cb.Run(in_isr, pack);
            }
            break;
        }

        return ErrorCode::OK;
      });
}

void FDCAN::Register(CallbackFD cb, Type type, FilterMode mode, uint32_t start_id_mask,
                     uint32_t end_id_mask)
{
  ASSERT(type < Type::REMOTE_STANDARD);

  auto node = new (std::align_val_t(LibXR::CONCURRENCY_ALIGNMENT))
      LockFreeList::Node<Filter>(Filter{mode, start_id_mask, end_id_mask, type, cb});
  subscriber_list_fd_[static_cast<uint8_t>(type)].Add(*node);
}

void FDCAN::OnMessage(const FDPack& pack, bool in_isr)
{
  // FD 帧没有远程帧，订阅表只有 STANDARD 和 EXTENDED 两项。
  // FD frames have no remote form; the subscriber table holds only STANDARD and EXTENDED.
  ASSERT(pack.type < Type::REMOTE_STANDARD);
  subscriber_list_fd_[static_cast<uint8_t>(pack.type)].Foreach<Filter>(
      [&](Filter& node)
      {
        switch (node.mode)
        {
          case FilterMode::ID_MASK:
            if ((pack.id & node.start_id_mask) == node.end_id_mask)
            {
              node.cb.Run(in_isr, pack);
            }
            break;
          case FilterMode::ID_RANGE:
            if (pack.id >= node.start_id_mask && pack.id <= node.end_id_mask)
            {
              node.cb.Run(in_isr, pack);
            }
            break;
        }

        return ErrorCode::OK;
      });
}
