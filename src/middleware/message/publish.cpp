#include <atomic>

#include "subscriber/async.hpp"
#include "subscriber/callback.hpp"
#include "subscriber/sync.hpp"
#include "timebase.hpp"
#include "topic.hpp"

using namespace LibXR;

void Topic::DispatchSubscriber(SuberBlock& block, MicrosecondTimestamp timestamp,
                               void* payload_addr, bool from_callback, bool in_isr)
{
  switch (block.type)
  {
    case SuberType::SYNC:
    {
      auto sync = static_cast<SyncBlock*>(&block);
      uint32_t expected = SyncBlock::WAITING;
      auto wake_waiter = sync->wait_state.compare_exchange_strong(
          expected, SyncBlock::WAIT_CLAIMED, std::memory_order_acq_rel,
          std::memory_order_acquire);

      if (!wake_waiter)
      {
        break;
      }

      sync->copy_payload(sync->buff_addr, payload_addr);
      sync->timestamp = timestamp;

      if (from_callback)
      {
        sync->sem.PostFromCallback(in_isr);
      }
      else
      {
        sync->sem.Post();
      }
      break;
    }
    case SuberType::ASYNC:
    {
      auto async = static_cast<ASyncBlock*>(&block);
      if (async->state.load(std::memory_order_acquire) == ASyncSubscriberState::WAITING)
      {
        async->copy_payload(async->buff_addr, payload_addr);
        async->timestamp = timestamp;
        async->state.store(ASyncSubscriberState::DATA_READY, std::memory_order_release);
      }
      break;
    }
    case SuberType::CALLBACK:
    {
      auto cb_block = static_cast<CallbackBlock*>(&block);
      cb_block->Run(from_callback && in_isr, timestamp, payload_addr);
      break;
    }
  }
}

void Topic::DispatchSubscribers(TopicHandle topic, MicrosecondTimestamp timestamp,
                                void* payload_addr, bool from_callback, bool in_isr)
{
  topic->data_.subers.Foreach<SuberBlock>(
      [=](SuberBlock& block)
      {
        DispatchSubscriber(block, timestamp, payload_addr, from_callback, in_isr);
        return ErrorCode::OK;
      });
}

MicrosecondTimestamp Topic::NowTimestamp() { return Timebase::GetMicroseconds(); }

void Topic::CheckPublishContract(TopicHandle topic, TypeID::ID payload_type_id,
                                 size_t payload_size, size_t payload_alignment)
{
  ASSERT(topic != nullptr);
  ASSERT(payload_type_id != nullptr);
  ASSERT(topic->data_.payload_type_id == payload_type_id);
  ASSERT(topic->data_.payload_size == payload_size);
  ASSERT(topic->data_.payload_alignment == payload_alignment);
}
