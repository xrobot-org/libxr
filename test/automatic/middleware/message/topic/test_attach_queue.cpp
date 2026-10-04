/**
 * @file test_attach_queue.cpp
 * @brief 共享 Topic 的连接与队列测试 / Shared Topic attachment and queue tests.
 *
 * 检查 LinuxSharedTopic 的连接、槽位占用、满队列处理、旧唤醒提示，以及 Publish()
 * 返回 FULL 后的重试方式。
 * Check LinuxSharedTopic attachment, slot occupancy, full queues, stale wake hints and
 * how to retry after Publish() returns FULL.
 */

#include <initializer_list>
#include <thread>

#include "linux_shm_topic_test_common.hpp"
#include "test_assert.hpp"

namespace LinuxShmTopicTest
{
void RunAttachQueueScenarios()
{
  char topic_name[96] = {};
  // Basic attach-only semantics and slot backpressure.
  MakeTopicName(topic_name, sizeof(topic_name), "linux_shm_local");
  UNUSED(SharedTopic::Remove(topic_name));

  {
    LibXR::LinuxSharedTopicConfig config;
    config.slot_num = 2;
    config.subscriber_num = 2;
    config.queue_num = 4;

    SharedTopic publisher(topic_name, config);
    TEST_ASSERT(publisher.Valid());

    SharedSubscriber subscriber(topic_name);
    TEST_ASSERT(subscriber.Valid());
    TEST_ASSERT(publisher.GetSubscriberNum() == 1);

    SharedTopic attach_only(topic_name);
    TEST_ASSERT(attach_only.Valid());
    SharedData attach_data;
    TEST_ASSERT(attach_only.CreateData(attach_data) == LibXR::ErrorCode::STATE_ERR);

    SharedData data0;
    const LibXR::MicrosecondTimestamp timestamp0(101000);
    TEST_ASSERT(publisher.CreateData(data0) == LibXR::ErrorCode::OK);
    FillFrame(*data0.GetData(), 100);
    TEST_ASSERT(publisher.Publish(data0, timestamp0) == LibXR::ErrorCode::OK);

    SharedData data1;
    const LibXR::MicrosecondTimestamp timestamp1(102000);
    TEST_ASSERT(publisher.CreateData(data1) == LibXR::ErrorCode::OK);
    FillFrame(*data1.GetData(), 101);
    TEST_ASSERT(publisher.Publish(data1, timestamp1) == LibXR::ErrorCode::OK);

    SharedData data2;
    TEST_ASSERT(publisher.CreateData(data2) == LibXR::ErrorCode::FULL);

    TEST_ASSERT(subscriber.Wait(SHORT_WAIT_MS) == LibXR::ErrorCode::OK);
    TEST_ASSERT(subscriber.GetData() != nullptr);
    AssertFrame(*subscriber.GetData(), 100);
    TEST_ASSERT(static_cast<uint64_t>(subscriber.GetTimestamp()) ==
                static_cast<uint64_t>(timestamp0));
    subscriber.GetData()->seq = 1000;
    TEST_ASSERT(subscriber.GetData()->seq == 1000);
    TEST_ASSERT(subscriber.GetPendingNum() == 1);
    subscriber.Release();

    const LibXR::MicrosecondTimestamp timestamp2(103000);
    TEST_ASSERT(publisher.CreateData(data2) == LibXR::ErrorCode::OK);
    FillFrame(*data2.GetData(), 102);
    TEST_ASSERT(publisher.Publish(data2, timestamp2) == LibXR::ErrorCode::OK);

    TEST_ASSERT(subscriber.Wait(SHORT_WAIT_MS) == LibXR::ErrorCode::OK);
    TEST_ASSERT(subscriber.GetData() != nullptr);
    AssertFrame(*subscriber.GetData(), 101);
    TEST_ASSERT(static_cast<uint64_t>(subscriber.GetTimestamp()) ==
                static_cast<uint64_t>(timestamp1));
    subscriber.Release();

    TEST_ASSERT(subscriber.Wait(SHORT_WAIT_MS) == LibXR::ErrorCode::OK);
    TEST_ASSERT(subscriber.GetData() != nullptr);
    AssertFrame(*subscriber.GetData(), 102);
    TEST_ASSERT(static_cast<uint64_t>(subscriber.GetTimestamp()) ==
                static_cast<uint64_t>(timestamp2));
    TEST_ASSERT(subscriber.GetPendingNum() == 0);
    subscriber.Release();

    // 消费完一批消息后，旧唤醒提示不能被当成仍有消息。
    // A wake hint left by a drained batch must not report another message.
    TEST_ASSERT(subscriber.Wait(2) == LibXR::ErrorCode::TIMEOUT);
    std::thread next_publish(
        [&publisher]()
        {
          LibXR::Thread::Sleep(10);
          SharedData next;
          TEST_ASSERT(publisher.CreateData(next) == LibXR::ErrorCode::OK);
          FillFrame(*next.GetData(), 103);
          TEST_ASSERT(publisher.Publish(next) == LibXR::ErrorCode::OK);
        });
    const auto wait_result = subscriber.Wait(LONG_WAIT_MS);
    next_publish.join();
    TEST_ASSERT(wait_result == LibXR::ErrorCode::OK);
    AssertFrame(*subscriber.GetData(), 103);
    subscriber.Release();
    TEST_ASSERT(subscriber.Wait(2) == LibXR::ErrorCode::TIMEOUT);
  }
  // BROADCAST_FULL should fail publish when the subscriber queue is saturated.
  UNUSED(SharedTopic::Remove(topic_name));
  MakeTopicName(topic_name, sizeof(topic_name), "linux_shm_queue");
  UNUSED(SharedTopic::Remove(topic_name));

  {
    LibXR::LinuxSharedTopicConfig config;
    config.slot_num = 8;
    config.subscriber_num = 1;
    config.queue_num = 3;

    SharedTopic publisher(topic_name, config);
    TEST_ASSERT(publisher.Valid());

    SharedSubscriber subscriber(topic_name);
    TEST_ASSERT(subscriber.Valid());

    IPCFrame frame = {};
    FillFrame(frame, 201);
    TEST_ASSERT(publisher.Publish(frame) == LibXR::ErrorCode::OK);
    FillFrame(frame, 202);
    TEST_ASSERT(publisher.Publish(frame) == LibXR::ErrorCode::OK);
    FillFrame(frame, 203);
    TEST_ASSERT(publisher.Publish(frame) == LibXR::ErrorCode::FULL);

    TEST_ASSERT(subscriber.GetPendingNum() == 2);
    TEST_ASSERT(subscriber.GetDropNum() == 1);
    TEST_ASSERT(publisher.GetPublishFailedNum() == 1);

    SharedData recv_data;
    TEST_ASSERT(subscriber.Wait(recv_data, SHORT_WAIT_MS) == LibXR::ErrorCode::OK);
    TEST_ASSERT(recv_data.GetSequence() == 1);
    AssertFrame(*recv_data.GetData(), 201);
    const SharedData& const_recv_data = recv_data;
    const_recv_data.GetData()->seq = 1201;
    TEST_ASSERT(const_recv_data.GetData()->seq == 1201);
    recv_data.Reset();

    TEST_ASSERT(subscriber.Wait(recv_data, SHORT_WAIT_MS) == LibXR::ErrorCode::OK);
    TEST_ASSERT(recv_data.GetSequence() == 2);
    AssertFrame(*recv_data.GetData(), 202);
    recv_data.Reset();
  }

  // BROADCAST_DROP_OLD should keep the newest descriptors without failing publish.
  UNUSED(SharedTopic::Remove(topic_name));
  MakeTopicName(topic_name, sizeof(topic_name), "linux_shm_drop_old");
  UNUSED(SharedTopic::Remove(topic_name));

  {
    LibXR::LinuxSharedTopicConfig config;
    config.slot_num = 8;
    config.subscriber_num = 1;
    config.queue_num = 3;

    SharedTopic publisher(topic_name, config);
    TEST_ASSERT(publisher.Valid());

    SharedSubscriber subscriber(topic_name,
                                LibXR::LinuxSharedSubscriberMode::BROADCAST_DROP_OLD);
    TEST_ASSERT(subscriber.Valid());

    IPCFrame frame = {};
    FillFrame(frame, 211);
    TEST_ASSERT(publisher.Publish(frame) == LibXR::ErrorCode::OK);
    FillFrame(frame, 212);
    TEST_ASSERT(publisher.Publish(frame) == LibXR::ErrorCode::OK);
    FillFrame(frame, 213);
    TEST_ASSERT(publisher.Publish(frame) == LibXR::ErrorCode::OK);

    TEST_ASSERT(subscriber.GetPendingNum() == 2);
    TEST_ASSERT(subscriber.GetDropNum() == 1);
    TEST_ASSERT(publisher.GetPublishFailedNum() == 0);

    SharedData recv_data;
    TEST_ASSERT(subscriber.Wait(recv_data, SHORT_WAIT_MS) == LibXR::ErrorCode::OK);
    TEST_ASSERT(recv_data.GetSequence() == 2);
    AssertFrame(*recv_data.GetData(), 212);
    recv_data.Reset();

    TEST_ASSERT(subscriber.Wait(recv_data, SHORT_WAIT_MS) == LibXR::ErrorCode::OK);
    TEST_ASSERT(recv_data.GetSequence() == 3);
    AssertFrame(*recv_data.GetData(), 213);
    recv_data.Reset();
  }

  // Publish() 返回 FULL 时已经释放句柄：对同一句柄重试只得到 STATE_ERR，重新
  // CreateData() 后才能再发布。槽位数与队列长度相同（4），队列最多存 3 个描述符。
  // Publish() has released the handle when it returns FULL: retrying on the same handle
  // only gets STATE_ERR, and publishing again needs a new CreateData(). Slot count and
  // queue length are both 4, so the queue holds at most 3 descriptors.
  UNUSED(SharedTopic::Remove(topic_name));
  MakeTopicName(topic_name, sizeof(topic_name), "linux_shm_publish_retry");
  UNUSED(SharedTopic::Remove(topic_name));

  {
    LibXR::LinuxSharedTopicConfig config;
    config.slot_num = 4;
    config.subscriber_num = 1;
    config.queue_num = 4;

    SharedTopic publisher(topic_name, config);
    TEST_ASSERT(publisher.Valid());

    SharedSubscriber subscriber(topic_name);
    TEST_ASSERT(subscriber.Valid());

    auto publish_fresh = [&publisher](uint32_t seq)
    {
      SharedData data;
      const LibXR::ErrorCode create_ans = publisher.CreateData(data);
      if (create_ans != LibXR::ErrorCode::OK)
      {
        return create_ans;
      }
      FillFrame(*data.GetData(), seq);
      return publisher.Publish(data);
    };

    // 订阅者持有第一条消息的槽位，另外三条占满队列，四个槽位全部在用。
    // The subscriber holds the slot of the first message and three more fill the queue,
    // so all four slots are in use.
    TEST_ASSERT(publish_fresh(301) == LibXR::ErrorCode::OK);
    SharedData held;
    TEST_ASSERT(subscriber.Wait(held, SHORT_WAIT_MS) == LibXR::ErrorCode::OK);
    AssertFrame(*held.GetData(), 301);
    TEST_ASSERT(publish_fresh(302) == LibXR::ErrorCode::OK);
    TEST_ASSERT(publish_fresh(303) == LibXR::ErrorCode::OK);
    TEST_ASSERT(publish_fresh(304) == LibXR::ErrorCode::OK);

    SharedData data;
    TEST_ASSERT(publisher.CreateData(data) == LibXR::ErrorCode::FULL);
    TEST_ASSERT(!data.Valid());

    // 订阅者放回槽位但队列仍满：CreateData() 成功，Publish() 返回 FULL 并释放句柄。
    // The subscriber returns its slot while the queue stays full: CreateData() succeeds,
    // and Publish() returns FULL and releases the handle.
    held.Reset();
    TEST_ASSERT(publisher.CreateData(data) == LibXR::ErrorCode::OK);
    FillFrame(*data.GetData(), 305);
    TEST_ASSERT(publisher.Publish(data) == LibXR::ErrorCode::FULL);
    TEST_ASSERT(!data.Valid());
    TEST_ASSERT(publisher.GetPublishFailedNum() == 1);
    TEST_ASSERT(publisher.Publish(data) == LibXR::ErrorCode::STATE_ERR);
    TEST_ASSERT(publisher.GetPublishFailedNum() == 1);

    // 订阅者取走一条后，重新申请并填写的句柄可以发布。
    // After the subscriber takes one message, a newly acquired and filled handle
    // publishes.
    SharedData recv_data;
    TEST_ASSERT(subscriber.Wait(recv_data, SHORT_WAIT_MS) == LibXR::ErrorCode::OK);
    AssertFrame(*recv_data.GetData(), 302);
    recv_data.Reset();
    TEST_ASSERT(publish_fresh(305) == LibXR::ErrorCode::OK);

    for (uint32_t seq : {303U, 304U, 305U})
    {
      TEST_ASSERT(subscriber.Wait(recv_data, SHORT_WAIT_MS) == LibXR::ErrorCode::OK);
      AssertFrame(*recv_data.GetData(), seq);
      recv_data.Reset();
    }
    TEST_ASSERT(subscriber.GetPendingNum() == 0);
  }
  UNUSED(SharedTopic::Remove(topic_name));
}
}  // namespace LinuxShmTopicTest
