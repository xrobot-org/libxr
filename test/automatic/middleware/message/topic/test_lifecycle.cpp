/**
 * @file test_lifecycle.cpp
 * @brief 共享 Topic 的生命周期测试 / Shared Topic lifetime tests.
 *
 * 检查 LinuxSharedTopic 槽位引用、发布者退出后的接管，以及死亡订阅者的回收。
 * Check LinuxSharedTopic slot references, takeover after publisher exit and
 * dead-subscriber reclamation.
 */

#include "linux_shm_topic_test_common.hpp"
#include "test_assert.hpp"

namespace LinuxShmTopicTest
{

void RunLifecycleSlotReferenceScenario()
{
  char topic_name[96] = {};
  // A slot must stay pinned until every subscriber releases it.
  MakeTopicName(topic_name, sizeof(topic_name), "linux_shm_ref");
  UNUSED(SharedTopic::Remove(topic_name));

  {
    LibXR::LinuxSharedTopicConfig config;
    config.slot_num = 1;
    config.subscriber_num = 2;
    config.queue_num = 4;

    SharedTopic publisher(topic_name, config);
    TEST_ASSERT(publisher.Valid());

    SharedSubscriber subscriber_a(topic_name);
    SharedSubscriber subscriber_b(topic_name);
    TEST_ASSERT(subscriber_a.Valid());
    TEST_ASSERT(subscriber_b.Valid());

    IPCFrame frame = {};
    FillFrame(frame, 301);
    TEST_ASSERT(publisher.Publish(frame) == LibXR::ErrorCode::OK);

    TEST_ASSERT(subscriber_a.Wait(SHORT_WAIT_MS) == LibXR::ErrorCode::OK);
    TEST_ASSERT(subscriber_b.Wait(SHORT_WAIT_MS) == LibXR::ErrorCode::OK);
    AssertFrame(*subscriber_a.GetData(), 301);
    AssertFrame(*subscriber_b.GetData(), 301);

    subscriber_a.Release();

    SharedData blocked_data;
    TEST_ASSERT(publisher.CreateData(blocked_data) == LibXR::ErrorCode::FULL);

    subscriber_b.Release();
    TEST_ASSERT(publisher.CreateData(blocked_data) == LibXR::ErrorCode::OK);
    FillFrame(*blocked_data.GetData(), 302);
    TEST_ASSERT(publisher.Publish(blocked_data) == LibXR::ErrorCode::OK);

    TEST_ASSERT(subscriber_a.Wait(SHORT_WAIT_MS) == LibXR::ErrorCode::OK);
    TEST_ASSERT(subscriber_b.Wait(SHORT_WAIT_MS) == LibXR::ErrorCode::OK);
    AssertFrame(*subscriber_a.GetData(), 302);
    AssertFrame(*subscriber_b.GetData(), 302);
    subscriber_a.Release();
    subscriber_b.Release();
  }

  UNUSED(SharedTopic::Remove(topic_name));
}

void RunLifecycleTakeoverScenario()
{
  char topic_name[96] = {};
  // Stale publisher takeover should allow a fresh creator to reopen the segment.
  MakeTopicName(topic_name, sizeof(topic_name), "linux_shm_takeover");
  UNUSED(SharedTopic::Remove(topic_name));

  {
    LibXR::LinuxSharedTopicConfig config;
    config.slot_num = 4;
    config.subscriber_num = 1;
    config.queue_num = 4;

    pid_t child = fork();
    TEST_ASSERT(child >= 0);

    if (child == 0)
    {
      SharedTopic publisher(topic_name, config);
      if (!publisher.Valid())
      {
        _exit(20);
      }

      IPCFrame frame = {};
      FillFrame(frame, 150);
      if (publisher.Publish(frame) != LibXR::ErrorCode::OK)
      {
        _exit(21);
      }

      _exit(0);
    }

    ExpectChildExit(child);

    SharedTopic publisher(topic_name, config);
    TEST_ASSERT(publisher.Valid());

    IPCFrame frame = {};
    FillFrame(frame, 151);
    TEST_ASSERT(publisher.Publish(frame) == LibXR::ErrorCode::OK);
  }

  UNUSED(SharedTopic::Remove(topic_name));
}

void RunLifecycleDeadSubscriberScenario()
{
  char topic_name[96] = {};
  // A dead subscriber should be reclaimed together with its held slot state.
  MakeTopicName(topic_name, sizeof(topic_name), "linux_shm_dead_sub");
  UNUSED(SharedTopic::Remove(topic_name));

  {
    LibXR::LinuxSharedTopicConfig config;
    config.slot_num = 2;
    config.subscriber_num = 1;
    config.queue_num = 4;

    SharedTopic publisher(topic_name, config);
    TEST_ASSERT(publisher.Valid());

    int ack_pipe[2] = {-1, -1};
    int cmd_pipe[2] = {-1, -1};
    TEST_ASSERT(pipe(ack_pipe) == 0);
    TEST_ASSERT(pipe(cmd_pipe) == 0);

    pid_t child = fork();
    TEST_ASSERT(child >= 0);

    if (child == 0)
    {
      close(ack_pipe[0]);
      close(cmd_pipe[1]);

      SharedSubscriber subscriber(topic_name);
      if (!subscriber.Valid())
      {
        _exit(10);
      }

      SharedData recv_data;
      if (subscriber.Wait(recv_data, LONG_WAIT_MS) != LibXR::ErrorCode::OK)
      {
        _exit(11);
      }

      uint8_t ack = 0xA5;
      if (write(ack_pipe[1], &ack, sizeof(ack)) != static_cast<ssize_t>(sizeof(ack)))
      {
        _exit(12);
      }

      uint8_t cmd = 0;
      if (read(cmd_pipe[0], &cmd, sizeof(cmd)) != static_cast<ssize_t>(sizeof(cmd)))
      {
        _exit(13);
      }

      _exit(0);
    }

    close(ack_pipe[1]);
    close(cmd_pipe[0]);

    WaitForSubscriberNum(publisher, 1);

    IPCFrame frame = {};
    FillFrame(frame, 401);
    TEST_ASSERT(publisher.Publish(frame) == LibXR::ErrorCode::OK);

    uint8_t ack = 0;
    TEST_ASSERT(read(ack_pipe[0], &ack, sizeof(ack)) ==
                static_cast<ssize_t>(sizeof(ack)));

    FillFrame(frame, 402);
    TEST_ASSERT(publisher.Publish(frame) == LibXR::ErrorCode::OK);

    uint8_t cmd = 0x5A;
    TEST_ASSERT(write(cmd_pipe[1], &cmd, sizeof(cmd)) ==
                static_cast<ssize_t>(sizeof(cmd)));

    ExpectChildExit(child);
    TEST_ASSERT(publisher.GetSubscriberNum() == 1);

    SharedData data0;
    SharedData data1;
    SharedData data2;
    TEST_ASSERT(publisher.CreateData(data0) == LibXR::ErrorCode::OK);
    TEST_ASSERT(publisher.GetSubscriberNum() == 0);
    TEST_ASSERT(publisher.CreateData(data1) == LibXR::ErrorCode::OK);
    TEST_ASSERT(publisher.CreateData(data2) == LibXR::ErrorCode::FULL);

    close(ack_pipe[0]);
    close(cmd_pipe[1]);
  }

  UNUSED(SharedTopic::Remove(topic_name));
}

void RunLifecycleScenarios()
{
  RunLifecycleTakeoverScenario();
  RunLifecycleSlotReferenceScenario();
  RunLifecycleDeadSubscriberScenario();
}
}  // namespace LinuxShmTopicTest
