/**
 * @file test_mixed_modes.cpp
 * @brief 共享 Topic 的域隔离与混合订阅测试 / Shared Topic domain and mixed-mode tests.
 *
 * 检查 LinuxSharedTopic 的同名不同域隔离、广播与轮询共存，以及轮询组满时发布失败。
 * Check LinuxSharedTopic domain separation, mixed broadcast/round-robin delivery and
 * full-group failure.
 */

#include "linux_shm_topic_test_common.hpp"
#include "test_assert.hpp"

namespace LinuxShmTopicTest
{
void RunMixedModeScenarios()
{
  char topic_name[96] = {};
  // The same topic name in different domains must stay isolated.
  MakeTopicName(topic_name, sizeof(topic_name), "linux_shm_domain");

  {
    LibXR::Topic::Domain domain_a("linux_shm_domain_a");
    LibXR::Topic::Domain domain_b("linux_shm_domain_b");

    UNUSED(SharedTopic::Remove(topic_name, domain_a));
    UNUSED(SharedTopic::Remove(topic_name, domain_b));

    LibXR::LinuxSharedTopicConfig config;
    config.slot_num = 4;
    config.subscriber_num = 1;
    config.queue_num = 4;

    SharedTopic publisher_a(topic_name, domain_a, config);
    SharedTopic publisher_b(topic_name, "linux_shm_domain_b", config);
    TEST_ASSERT(publisher_a.Valid());
    TEST_ASSERT(publisher_b.Valid());

    SharedSubscriber subscriber_a(topic_name, "linux_shm_domain_a");
    SharedSubscriber subscriber_b(topic_name, domain_b);
    TEST_ASSERT(subscriber_a.Valid());
    TEST_ASSERT(subscriber_b.Valid());

    IPCFrame frame = {};
    FillFrame(frame, 331);
    TEST_ASSERT(publisher_a.Publish(frame) == LibXR::ErrorCode::OK);
    FillFrame(frame, 441);
    TEST_ASSERT(publisher_b.Publish(frame) == LibXR::ErrorCode::OK);

    SharedData recv_a;
    SharedData recv_b;
    TEST_ASSERT(subscriber_a.Wait(recv_a, SHORT_WAIT_MS) == LibXR::ErrorCode::OK);
    TEST_ASSERT(subscriber_b.Wait(recv_b, SHORT_WAIT_MS) == LibXR::ErrorCode::OK);
    TEST_ASSERT(recv_a.GetData()->seq == 331);
    TEST_ASSERT(recv_b.GetData()->seq == 441);
    recv_a.Reset();
    recv_b.Reset();

    UNUSED(SharedTopic::Remove(topic_name, domain_a));
    UNUSED(SharedTopic::Remove(topic_name, domain_b));
  }
  // Broadcast subscribers and balanced subscribers should coexist on one topic.
  MakeTopicName(topic_name, sizeof(topic_name), "linux_shm_mixed_modes");
  UNUSED(SharedTopic::Remove(topic_name));

  {
    LibXR::LinuxSharedTopicConfig config;
    config.slot_num = 8;
    config.subscriber_num = 3;
    config.queue_num = 4;

    SharedTopic publisher(topic_name, config);
    TEST_ASSERT(publisher.Valid());

    SharedSubscriber subscriber_broadcast(topic_name);
    SharedSubscriber subscriber_rr_a(topic_name,
                                     LibXR::LinuxSharedSubscriberMode::BALANCE_RR);
    SharedSubscriber subscriber_rr_b(topic_name,
                                     LibXR::LinuxSharedSubscriberMode::BALANCE_RR);
    TEST_ASSERT(subscriber_broadcast.Valid());
    TEST_ASSERT(subscriber_rr_a.Valid());
    TEST_ASSERT(subscriber_rr_b.Valid());

    IPCFrame frame = {};
    FillFrame(frame, 381);
    TEST_ASSERT(publisher.Publish(frame) == LibXR::ErrorCode::OK);
    FillFrame(frame, 382);
    TEST_ASSERT(publisher.Publish(frame) == LibXR::ErrorCode::OK);
    FillFrame(frame, 383);
    TEST_ASSERT(publisher.Publish(frame) == LibXR::ErrorCode::OK);

    SharedData bc0;
    SharedData bc1;
    SharedData bc2;
    TEST_ASSERT(subscriber_broadcast.Wait(bc0, SHORT_WAIT_MS) == LibXR::ErrorCode::OK);
    TEST_ASSERT(subscriber_broadcast.Wait(bc1, SHORT_WAIT_MS) == LibXR::ErrorCode::OK);
    TEST_ASSERT(subscriber_broadcast.Wait(bc2, SHORT_WAIT_MS) == LibXR::ErrorCode::OK);
    TEST_ASSERT(bc0.GetData()->seq == 381);
    TEST_ASSERT(bc1.GetData()->seq == 382);
    TEST_ASSERT(bc2.GetData()->seq == 383);

    SharedData rr_a0;
    SharedData rr_b0;
    SharedData rr_a1;
    TEST_ASSERT(subscriber_rr_a.Wait(rr_a0, SHORT_WAIT_MS) == LibXR::ErrorCode::OK);
    TEST_ASSERT(subscriber_rr_b.Wait(rr_b0, SHORT_WAIT_MS) == LibXR::ErrorCode::OK);
    TEST_ASSERT(subscriber_rr_a.Wait(rr_a1, SHORT_WAIT_MS) == LibXR::ErrorCode::OK);
    TEST_ASSERT(rr_a0.GetData()->seq == 381);
    TEST_ASSERT(rr_b0.GetData()->seq == 382);
    TEST_ASSERT(rr_a1.GetData()->seq == 383);
  }

  UNUSED(SharedTopic::Remove(topic_name));

  // If the balanced group exists but cannot accept, publish should fail for everyone.
  MakeTopicName(topic_name, sizeof(topic_name), "linux_shm_rr_group_required");
  UNUSED(SharedTopic::Remove(topic_name));

  {
    LibXR::LinuxSharedTopicConfig config;
    config.slot_num = 4;
    config.subscriber_num = 2;
    config.queue_num = 2;

    SharedTopic publisher(topic_name, config);
    TEST_ASSERT(publisher.Valid());

    SharedSubscriber subscriber_broadcast(topic_name);
    SharedSubscriber subscriber_rr(topic_name,
                                   LibXR::LinuxSharedSubscriberMode::BALANCE_RR);
    TEST_ASSERT(subscriber_broadcast.Valid());
    TEST_ASSERT(subscriber_rr.Valid());

    IPCFrame frame = {};
    FillFrame(frame, 391);
    TEST_ASSERT(publisher.Publish(frame) == LibXR::ErrorCode::OK);

    FillFrame(frame, 392);
    TEST_ASSERT(publisher.Publish(frame) == LibXR::ErrorCode::FULL);

    SharedData bc0;
    TEST_ASSERT(subscriber_broadcast.Wait(bc0, SHORT_WAIT_MS) == LibXR::ErrorCode::OK);
    TEST_ASSERT(bc0.GetData()->seq == 391);

    SharedData rr0;
    TEST_ASSERT(subscriber_rr.Wait(rr0, SHORT_WAIT_MS) == LibXR::ErrorCode::OK);
    TEST_ASSERT(rr0.GetData()->seq == 391);
  }

  UNUSED(SharedTopic::Remove(topic_name));
}
}  // namespace LinuxShmTopicTest
