#include "topic.hpp"

#include <atomic>

#include "crc.hpp"
#include "libxr_def.hpp"
#include "mutex.hpp"

using namespace LibXR;

template class LibXR::RBTree<uint32_t>;

void Topic::EnsureDomainRegistry()
{
  if (!domain_)
  {
    domain_ = new RBTree<uint32_t>([](const uint32_t& a, const uint32_t& b)
                                   { return (a > b) - (a < b); });
  }
}

Topic::Domain* Topic::EnsureDefaultDomain()
{
  if (!def_domain_)
  {
    def_domain_ = new Domain("libxr_def_domain");
  }

  return def_domain_;
}

void Topic::Lock(Topic::TopicHandle topic)
{
  if (topic->data_.mutex)
  {
    topic->data_.mutex->Lock();
  }
  else
  {
    LockState expected = LockState::UNLOCKED;
    if (!topic->data_.busy.compare_exchange_strong(expected, LockState::LOCKED))
    {
      // 非 mutex topic 禁止并发发布
      ASSERT(false);
      return;
    }
  }
}

void Topic::Unlock(Topic::TopicHandle topic)
{
  if (topic->data_.mutex)
  {
    topic->data_.mutex->Unlock();
  }
  else
  {
    topic->data_.busy.store(LockState::UNLOCKED, std::memory_order_release);
  }
}

void Topic::LockFromCallback(Topic::TopicHandle topic)
{
  if (topic->data_.mutex)
  {
    ASSERT(false);
  }
  else
  {
    LockState expected = LockState::UNLOCKED;
    if (!topic->data_.busy.compare_exchange_strong(expected, LockState::LOCKED))
    {
      // 回调发布路径要求外围先串行化
      ASSERT(false);
      return;
    }
  }
}

void Topic::UnlockFromCallback(Topic::TopicHandle topic)
{
  if (topic->data_.mutex)
  {
    ASSERT(false);
  }
  else
  {
    topic->data_.busy.store(LockState::UNLOCKED, std::memory_order_release);
  }
}

Topic::Domain::Domain(const char* name)
{
  ASSERT(name != nullptr);

  EnsureDomainRegistry();

  auto crc32 = CRC32::Calculate(name, strlen(name));

  auto domain = domain_->Search<RBTree<uint32_t>>(crc32);

  if (domain != nullptr)
  {
    node_ = domain;
    return;
  }

  node_ = new LibXR::RBTree<uint32_t>::Node<LibXR::RBTree<uint32_t>>(
      [](const uint32_t& a, const uint32_t& b) { return (a > b) - (a < b); });

  domain_->Insert(*node_, crc32);
}

Topic::Topic() {}

Topic::Topic(const char* name, TypeID::ID payload_type_id, size_t payload_size,
             size_t payload_alignment, Domain* domain, bool multi_publisher)
{
  ASSERT(name != nullptr);
  ASSERT(payload_type_id != nullptr);
  ASSERT(payload_size != 0);
  ASSERT(payload_alignment != 0);
  ASSERT((payload_alignment & (payload_alignment - 1)) == 0);

  if (domain == nullptr)
  {
    domain = EnsureDefaultDomain();
  }

  auto crc32 = CRC32::Calculate(name, strlen(name));

  auto topic = domain->node_->data_.Search<Block>(crc32);

  if (topic)
  {
    RequireTypeContract(topic, payload_type_id, payload_size, payload_alignment);

    if (multi_publisher)
    {
      RequireMultiPublisher(topic);
    }

    block_ = topic;
  }
  else
  {
    block_ = new RBTree<uint32_t>::Node<Block>;
    block_->data_.payload_type_id = payload_type_id;
    block_->data_.payload_size = payload_size;
    block_->data_.payload_alignment = payload_alignment;
    block_->data_.crc32 = crc32;

    if (multi_publisher)
    {
      block_->data_.mutex = new Mutex();
      block_->data_.busy.store(LockState::USE_MUTEX, std::memory_order_release);
    }
    else
    {
      block_->data_.mutex = nullptr;
      block_->data_.busy.store(LockState::UNLOCKED, std::memory_order_release);
    }

    domain->node_->data_.Insert(*block_, crc32);
  }
}

Topic::Topic(TopicHandle topic) : block_(topic) {}

void Topic::RequireTypeContract(TopicHandle topic, TypeID::ID payload_type_id,
                                size_t payload_size, size_t payload_alignment)
{
  const auto& data = topic->data_;
  REQUIRE(data.payload_type_id == payload_type_id && data.payload_size == payload_size &&
          data.payload_alignment == payload_alignment);
}

void Topic::RequireCallbackType(TopicHandle topic, TypeID::ID payload_type_id)
{
  REQUIRE(topic->data_.payload_type_id == payload_type_id);
}

void Topic::RequireMultiPublisher(TopicHandle topic)
{
  REQUIRE(topic->data_.mutex != nullptr);
}

Topic::TopicHandle Topic::Find(const char* name, Domain* domain)
{
  ASSERT(name != nullptr);

  if (domain == nullptr)
  {
    if (def_domain_ == nullptr)
    {
      return nullptr;
    }
    domain = def_domain_;
  }

  auto crc32 = CRC32::Calculate(name, strlen(name));

  return domain->node_->data_.Search<Block>(crc32);
}

Topic::TopicHandle Topic::WaitTopic(const char* name, uint32_t timeout, Domain* domain)
{
  const uint32_t start_time = Thread::GetTime();
  TopicHandle topic = nullptr;
  do
  {
    topic = Find(name, domain);
    if (topic == nullptr)
    {
      if (timeout != UINT32_MAX &&
          static_cast<uint32_t>(Thread::GetTime() - start_time) >= timeout)
      {
        return nullptr;
      }
      Thread::Sleep(1);
    }
  } while (topic == nullptr);

  return topic;
}

uint32_t Topic::GetKey() const
{
  if (block_)
  {
    return block_->key;
  }
  else
  {
    return 0;
  }
}
