/**
 * @file test_cross_process.cpp
 * @brief 共享 Topic 的跨进程收发测试 / Cross-process shared Topic transfer test.
 *
 * 父进程发布、子进程接收，检查 LinuxSharedTopic 消息的序号与内容校验。
 * Publish in the parent and receive in the child; check LinuxSharedTopic sequence and
 * payload checksums.
 */

#include "linux_shm_topic_test_common.hpp"
#include "test_assert.hpp"

namespace LinuxShmTopicTest
{
void RunCrossProcessScenarios()
{
  char topic_name[96] = {};
  // Cross-process publish/subscribe should preserve ordering and payload integrity.
  MakeTopicName(topic_name, sizeof(topic_name), "linux_shm_fork");
  UNUSED(SharedTopic::Remove(topic_name));

  {
    LibXR::LinuxSharedTopicConfig config;
    config.slot_num = 64;
    config.subscriber_num = 4;
    config.queue_num = 64;

    SharedTopic publisher(topic_name, config);
    TEST_ASSERT(publisher.Valid());

    pid_t child = fork();
    TEST_ASSERT(child >= 0);

    if (child == 0)
    {
      SharedSubscriber subscriber(topic_name);
      if (!subscriber.Valid())
      {
        _exit(2);
      }

      for (uint32_t seq = 1; seq <= 32; ++seq)
      {
        SharedData recv_data;
        if (subscriber.Wait(recv_data, LONG_WAIT_MS) != LibXR::ErrorCode::OK)
        {
          _exit(3);
        }

        const IPCFrame* frame = recv_data.GetData();
        if (frame == nullptr)
        {
          _exit(4);
        }

        if (recv_data.GetSequence() != seq || frame->seq != seq ||
            frame->checksum != ComputeChecksum(*frame))
        {
          _exit(5);
        }
      }

      _exit(0);
    }

    WaitForSubscriberNum(publisher, 1);

    for (uint32_t seq = 1; seq <= 32; ++seq)
    {
      SharedData data;
      TEST_ASSERT(publisher.CreateData(data) == LibXR::ErrorCode::OK);
      FillFrame(*data.GetData(), seq);
      TEST_ASSERT(publisher.Publish(data) == LibXR::ErrorCode::OK);
    }

    ExpectChildExit(child);
  }

  UNUSED(SharedTopic::Remove(topic_name));
}
}  // namespace LinuxShmTopicTest
