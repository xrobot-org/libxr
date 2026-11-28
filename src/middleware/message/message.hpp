#pragma once

/**
 * @brief `message` 对外包含入口 / Public include entry for `message`
 *
 * @note 外部代码仍应优先包含这个头；`topic`、`packet`、`server`、`subscriber`
 *       这些子头主要是给模块内部拆边界用的 /
 *       External code should still include this header first; the `topic`,
 *       `packet`, `server`, and `subscriber` subheaders are used to express the
 *       internal module boundaries
 */

#include "packet/packet.hpp"
#include "server/server.hpp"
#include "subscriber/async.hpp"
#include "subscriber/callback.hpp"
#include "subscriber/sync.hpp"
#include "topic.hpp"
