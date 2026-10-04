#pragma once

#include "libxr.hpp"

namespace LibXR
{

/**
 * @class Watchdog
 * @brief 通用看门狗（Watchdog）抽象接口
 *        General Watchdog interface for both thread and task style usage
 */
class Watchdog
{
 public:
  /**
   * @struct Configuration
   * @brief 看门狗配置结构体
   *        Configuration for the watchdog
   *
   * @var Configuration::timeout_ms
   *      看门狗溢出时间（毫秒） / Overflow/reset time in milliseconds
   * @var Configuration::feed_ms
   *      自动喂狗周期（毫秒）
   *      Auto feed interval in milliseconds (should be < timeout_ms)
   */
  struct Configuration
  {
    uint32_t timeout_ms;  ///< 看门狗溢出时间 / Watchdog overflow time (ms)
    uint32_t feed_ms;     ///< 自动喂狗周期 / Auto feed interval (ms, < timeout_ms)
  };

  /**
   * @brief 构造函数 / Constructor
   */
  Watchdog() {}

  /**
   * @brief 虚析构函数 / Virtual destructor
   */
  virtual ~Watchdog() {}

  /**
   * @brief 初始化硬件并设置超时时间
   *        Initialize hardware and set overflow time
   * @param config 配置参数 / Configuration
   * @return 操作结果的错误码 / Error code of the operation
   */
  virtual ErrorCode SetConfig(const Configuration& config) = 0;

  /**
   * @brief 立即手动喂狗
   *        Feed the watchdog immediately
   * @return 操作结果的错误码 / Error code of the operation
   */
  virtual ErrorCode Feed() = 0;

  /**
   * @brief 启动看门狗 / Start the watchdog
   */
  virtual ErrorCode Start() = 0;

  /**
   * @brief 停止看门狗 / Stop the watchdog
   */
  virtual ErrorCode Stop() = 0;

  /**
   * @brief Watchdog 自动喂狗线程函数
   *        Watchdog thread function for auto-feeding
   *
   * @details
   * 适合 RTOS/多线程环境。可直接传递给线程创建API作为入口，循环调用 Feed()，每隔
   * feed_ms 毫秒自动喂狗。
   * Suitable for RTOS or multi-threading. Call by thread system, periodically feeds the
   * watchdog.
   *
   * @param wdg 指向 Watchdog 实例的指针 / Pointer to the Watchdog instance
   */
  static void ThreadFun(Watchdog* wdg)
  {
    while (true)
    {
      if (wdg->auto_feed_)
      {
        wdg->Feed();
      }
      LibXR::Thread::Sleep(wdg->auto_feed_interval_ms);
    }
  }

  /**
   * @brief Watchdog 自动喂狗定时器任务函数
   *        Watchdog timer task function for auto-feeding
   *
   * @details
   * 适合轮询/定时任务调度环境。可直接作为Timer/Task回调，每次被调度时喂狗一次。
   * Suitable for cooperative/periodic polling system. Called by timer, feeds the watchdog
   * once per call.
   *
   * @param wdg 指向 Watchdog 实例的指针 / Pointer to the Watchdog instance
   */
  static void TaskFun(Watchdog* wdg)
  {
    if (wdg->auto_feed_)
    {
      wdg->Feed();
    }
  }

  uint32_t timeout_ms_ = 3000;            ///< 溢出时间
  uint32_t auto_feed_interval_ms = 1000;  ///< 自动喂狗间隔
  bool auto_feed_ = false;                ///< 是否自动喂狗
};

}  // namespace LibXR
