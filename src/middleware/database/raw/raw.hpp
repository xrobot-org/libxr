#pragma once

#include <cstdint>
#include <cstring>

#include "flash.hpp"
#include "interface.hpp"

namespace LibXR
{

/**
 * @brief 适用于最小写入单元受限的 Flash 存储的数据库实现
 *        Database implementation for Flash storage with minimum write unit
 *        restrictions.
 *
 * 此类提供适用于 Flash 存储的键值存储管理，该存储要求数据以固定大小块写入。
 * This class provides key-value storage management for Flash memory that
 * requires data to be written in fixed-size blocks.
 *
 * @note 若底层 Flash 读写擦失败，当前实现视为不可恢复故障并直接触发 `REQUIRE`。
 *       If the underlying Flash read, write, or erase operation fails, the
 *       current implementation treats it as an unrecoverable fault and triggers
 *       `REQUIRE` immediately.
 *
 * @tparam MinWriteSize Flash 的最小写入单元大小 / Minimum write unit size for Flash
 *         storage.
 * @note 这个头本身只是类壳；真正的布局、底层 IO、块操作、键操作和生命周期流程分别
 *       拆在同目录的几个类内片段头里。
 *       This header is only the class shell; the actual layout definitions,
 *       low-level I/O helpers, block operations, key operations, and lifecycle
 *       flow are split into several class-body fragments in the same
 *       directory.
 */
template <size_t MinWriteSize>
class DatabaseRaw : public Database
{
  /**
   * @brief 当前 raw 后端使用的块头签名 / Block-header signature used by the current
   *        raw backend
   *
   * @note 主块和备份块都用这份签名判断“这个块至少像是同一版 raw 数据库写出来的”。
   *       Both the main block and the backup block use this signature to decide
   *       whether the block at least looks like it was written by the same raw
   *       database format version.
   */
  static constexpr uint32_t FLASH_HEADER = 0x12345678 + LIBXR_DATABASE_VERSION;

  /**
   * @brief 当前 raw 后端使用的块尾校验常量 / Trailing checksum constant used by the
   *        current raw backend
   *
   * @note 这里不是通用 CRC 计算结果，而是“块已完整写完”的固定尾标记。
   *       This is not a generic computed CRC result; it is the fixed trailing
   *       marker meaning "this block has been fully written".
   */
  static constexpr uint32_t CHECKSUM_BYTE = 0x9abcedf0;

  // 存储布局定义：块角色、位图编码、键头与块头。
  // Storage layout definitions: block roles, bitmap encodings, key headers,
  // and block headers.
#include "layout.hpp"

  /**
   * @brief 查找时触发回收的失效键阈值 / Tombstone threshold that triggers recycle
   *        during lookup
   */
  size_t recycle_threshold_ = 0;

  /**
   * @brief 当前后端绑定的 Flash 设备引用 / Reference to the Flash device bound to this
   *        backend
   */
  Flash& flash_;

  /**
   * @brief 当前主块和备份块各自占用的物理块大小 / Physical block size occupied by each of
   *        the main and backup blocks
   */
  uint32_t block_size_;

  /**
   * @brief 复用的最小写入单元临时缓冲区 / Reused scratch buffer sized to one minimum
   *        write unit
   *
   * @note 这个缓冲区给尾块补齐和块间复制共用；它不是某个键或某个块长期持有的数据。
   *       This buffer is shared by tail padding and block-to-block copying; it
   *       does not store long-lived data owned by one specific key or block.
   */
  uint8_t write_buffer_[MinWriteSize];

  // 底层 Flash 读写辅助：按最小写入单元对齐、补齐、复制。
  // Low-level Flash I/O helpers: minimum-write-unit alignment, tail padding,
  // and raw copying.
#include "flash_io.hpp"

  // 块级语义：初始化、校验、失效、已用空间计算和块间复制。
  // Block-level semantics: initialization, validation, invalidation,
  // used-space accounting, and block-to-block copying.
#include "block_ops.hpp"

  // 键级语义：查找、追加、逻辑更新、条目地址计算。
  // Key-level semantics: lookup, append, logical replacement, and entry
  // address calculations.
#include "key_ops.hpp"

  /**
   * @brief `DatabaseRaw` 的对外生命周期入口区域 / Public lifecycle entry section of
   *        `DatabaseRaw`
   */
 public:
  // 对外生命周期与主流程：构造、初始化、读写、恢复、回收。
  // Public lifecycle and main flow: construction, initialization, read/write,
  // restore, and recycle.
#include "lifecycle.hpp"
};

}  // namespace LibXR
