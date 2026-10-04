#pragma once

/**
 * @file queue.hpp
 * @brief 队列模块聚合入口 / Aggregate entry of the queue module.
 *
 * 该头文件聚合 LibXR 当前公开的三种强类型队列：普通 FIFO、SPSC 和 MPMC。
 * 调用方若只需要统一引入队列族，可直接包含本头文件。
 * This header aggregates the three currently public typed queues in LibXR:
 * the ordinary FIFO queue, SPSC queue, and MPMC queue.
 * Callers may include this file directly when they want one uniform queue-family entry.
 */

#include "basic_queue.hpp"
#include "mpmc_queue.hpp"
#include "spsc_queue.hpp"
